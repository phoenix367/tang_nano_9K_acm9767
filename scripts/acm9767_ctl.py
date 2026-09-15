#!/usr/bin/env python3
"""Host control for the Tang Nano 9K + ACM9767 DAC generator (Modbus RTU).

Talks to the FPGA over /dev/ttyGowin (1 Mbaud 8-E-1, slave id 7 by default --
all from platform.json). The register map lives in src/acm9767_defs.vh and
README.md; this tool wraps it in physical units:

    acm9767_ctl.py status                      # id, version, PLL lock, dac_clk, channels
    acm9767_ctl.py set 1 --freq 1e6 --wave sine --amp 1.0
    acm9767_ctl.py set 2 --freq 250e3 --wave square --duty 0.25 --offset 0.1
    acm9767_ctl.py enable 1 2                  # enable channels and commit
    acm9767_ctl.py disable 1                   # disable channel 1 and commit
    acm9767_ctl.py invert 2 --on               # flip channel 2 polarity (fixes a wrong-sign output)
    acm9767_ctl.py read 0x10 7                 # raw holding registers
    acm9767_ctl.py write 0x13 0x7fff           # raw register write (no commit)
    acm9767_ctl.py commit
    acm9767_ctl.py stream --carrier 7.1e6 --tone 1000 --seconds 5      # SSB (USB) of a 1 kHz tone
    acm9767_ctl.py stream --carrier 7.1e6 --lsb --wav voice.wav        # SSB of a WAV file
    acm9767_ctl.py stream --tone 700 --tone2 1900 --level 0.45         # two-tone test (carrier as set)
    acm9767_ctl.py stream --carrier 1e6 --mode am --depth 0.8 --wav voice.wav   # AM, 80 % depth
    acm9767_ctl.py stream --carrier 10.7e6 --mode nfm --deviation 5000 --tone 1000  # NFM, 5 kHz dev

`stream` pushes signed 16-bit audio at the device's audio rate (register 0x0A,
16 kS/s by default) into the on-chip FIFO with FC16 blocks, pacing on
FIFO_LEVEL; channel 1 is switched to ssb-usb / ssb-lsb on the given carrier.

`set` stages the channel block and commits unless --no-commit is given, so a
frequency change lands atomically. The frequency tuning word is computed from
the sample clock the *device* reports (regs 0x05/0x06), so the CLI stays right
even against a bitstream built with a different platform.json.
"""

import argparse
import math
import os
import sys
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from modbus_rtu import DEFAULT_BAUD, DEFAULT_PORT, DEFAULT_SLAVE, ModbusError, ModbusRTU  # noqa: E402

try:
    import platform_config as _platform

    FALLBACK_DAC_CLK_HZ = _platform.DAC_CLK_HZ
except Exception:  # pragma: no cover
    FALLBACK_DAC_CLK_HZ = 54_000_000

# ---- register map (mirror of src/acm9767_defs.vh) ----
REG_ID, REG_VERSION, REG_STATUS, REG_CONTROL, REG_COMMIT = 0x00, 0x01, 0x02, 0x03, 0x04
REG_DAC_CLK_LO, REG_DAC_CLK_HI, REG_CHANNELS = 0x05, 0x06, 0x07
REG_FIFO_LEVEL, REG_UNDERRUNS, REG_AUDIO_RATE, REG_FIFO_DEPTH = 0x08, 0x09, 0x0A, 0x0B
REG_AUDIO_BASE = 0x0100          # ..0x017F: FIFO write window
REG_CH_BASE, REG_CH_STRIDE = 0x10, 0x10
CH_FTW_LO, CH_FTW_HI, CH_PHASE, CH_AMPLITUDE, CH_OFFSET, CH_WAVEFORM, CH_DUTY, CH_MODPARAM = range(8)
CH_BLOCK_LEN = 8
ACM9767_ID = 0x9767
STAT_PLL_LOCK, STAT_COMMIT_BUSY = 0x0001, 0x0002
WAVEFORMS = {"sine": 0, "triangle": 1, "sawtooth": 2, "square": 3, "dc": 4,
             "ssb-usb": 5, "ssb-lsb": 6, "am": 7, "fm": 8, "nfm": 8}
WAVEFORM_NAMES = {0: "sine", 1: "triangle", 2: "sawtooth", 3: "square", 4: "dc",
                  5: "ssb-usb", 6: "ssb-lsb", 7: "am", 8: "fm"}
MODES = {"usb": "ssb-usb", "lsb": "ssb-lsb", "am": "am", "fm": "fm", "nfm": "fm"}
DEFAULT_FM_DEVIATION_HZ = 5000.0     # narrowband FM
DAC_FULL_SCALE = 8191  # 14-bit signed


def ch_base(ch: int) -> int:
    if ch not in (1, 2):
        raise ValueError("channel must be 1 or 2")
    return REG_CH_BASE + (ch - 1) * REG_CH_STRIDE


# ---- unit conversions ----
def freq_to_ftw(freq_hz: float, dac_clk_hz: int) -> int:
    if not (0 <= freq_hz < dac_clk_hz / 2):
        raise ValueError(f"frequency {freq_hz} Hz outside 0..{dac_clk_hz / 2:.0f} Hz (Nyquist)")
    return int(round(freq_hz * (1 << 32) / dac_clk_hz)) & 0xFFFFFFFF


def ftw_to_freq(ftw: int, dac_clk_hz: int) -> float:
    return ftw * dac_clk_hz / (1 << 32)


def amp_to_reg(amp: float) -> int:
    """0.0..1.0 of full scale -> register (gain = (reg+1)/65536)."""
    if not (0.0 <= amp <= 1.0):
        raise ValueError("amplitude must be 0.0..1.0")
    return max(0, int(round(amp * 65536)) - 1)


def reg_to_amp(reg: int) -> float:
    return (reg + 1) / 65536


def offset_to_reg(offset: float) -> int:
    """-1.0..1.0 of full scale -> signed 16-bit DAC LSBs."""
    if not (-1.0 <= offset <= 1.0):
        raise ValueError("offset must be -1.0..1.0")
    return int(round(offset * DAC_FULL_SCALE)) & 0xFFFF


def reg_to_offset(reg: int) -> float:
    v = reg - 0x10000 if reg & 0x8000 else reg
    return v / DAC_FULL_SCALE


def phase_to_reg(deg: float) -> int:
    return int(round((deg % 360.0) / 360.0 * 65536)) & 0xFFFF


def reg_to_phase(reg: int) -> float:
    return reg / 65536 * 360.0


def duty_to_reg(duty: float) -> int:
    if not (0.0 <= duty <= 1.0):
        raise ValueError("duty must be 0.0..1.0")
    return min(0xFFFF, int(round(duty * 65536)))


def reg_to_duty(reg: int) -> float:
    return reg / 65536


def depth_to_reg(depth: float) -> int:
    """AM modulation depth 0..1 -> Q16 register."""
    if not (0.0 <= depth <= 1.0):
        raise ValueError("AM depth must be 0.0..1.0")
    return min(0xFFFF, int(round(depth * 65536)))


def reg_to_depth(reg: int) -> float:
    return reg / 65536


def deviation_to_reg(dev_hz: float, dac_clk_hz: int) -> int:
    """FM peak deviation in Hz -> register (units of 256 FTW steps)."""
    reg = int(round(dev_hz * (1 << 32) / dac_clk_hz / 256))
    if not (0 <= reg <= 0xFFFF):
        max_hz = 0xFFFF * 256 * dac_clk_hz / (1 << 32)
        raise ValueError(f"FM deviation {dev_hz} Hz outside 0..{max_hz:.0f} Hz")
    return reg


def reg_to_deviation(reg: int, dac_clk_hz: int) -> float:
    return reg * 256 * dac_clk_hz / (1 << 32)


def modparam_for(wave: str, dac_clk_hz: int, depth: float, deviation_hz: float) -> int:
    if wave == "am":
        return depth_to_reg(depth)
    if wave in ("fm", "nfm"):
        return deviation_to_reg(deviation_hz, dac_clk_hz)
    return 0


def channel_block(freq_hz, dac_clk_hz, wave, amp, offset, phase_deg, duty, modparam=0):
    """Build the 8-register channel block in register order."""
    ftw = freq_to_ftw(freq_hz, dac_clk_hz)
    return [ftw & 0xFFFF, ftw >> 16, phase_to_reg(phase_deg), amp_to_reg(amp),
            offset_to_reg(offset), WAVEFORMS[wave], duty_to_reg(duty), modparam & 0xFFFF]


# ---- device wrapper ----
class Acm9767:
    def __init__(self, mb: ModbusRTU):
        self.mb = mb
        self._dac_clk = None

    def identify(self):
        ident, ver = self.mb.read_holding(REG_ID, 2)
        if ident != ACM9767_ID:
            raise RuntimeError(f"unexpected device id 0x{ident:04X} (expected 0x{ACM9767_ID:04X})")
        return ident, ver

    @property
    def dac_clk_hz(self) -> int:
        if self._dac_clk is None:
            lo, hi = self.mb.read_holding(REG_DAC_CLK_LO, 2)
            self._dac_clk = (hi << 16) | lo
            if self._dac_clk == 0:
                self._dac_clk = FALLBACK_DAC_CLK_HZ
        return self._dac_clk

    @property
    def channels(self) -> int:
        """Number of DAC channels the bitstream brings out (register 0x07)."""
        n = self.mb.read_holding(REG_CHANNELS, 1)[0]
        return n if n in (1, 2) else 2

    def require_channel(self, ch: int):
        if ch > self.channels:
            raise ValueError(f"channel {ch} is not wired in this bitstream ({self.channels} channel(s), "
                             f"see platform.json dac.channels)")

    def status(self) -> int:
        return self.mb.read_holding(REG_STATUS, 1)[0]

    def control(self) -> int:
        return self.mb.read_holding(REG_CONTROL, 1)[0]

    def set_control(self, value: int):
        self.mb.write_single(REG_CONTROL, value & 0xF)

    def commit(self, wait=True, polls=200):
        self.mb.write_single(REG_COMMIT, 1)
        if wait:
            for _ in range(polls):
                if not (self.status() & STAT_COMMIT_BUSY):
                    return
            raise TimeoutError("commit never completed (STATUS.busy stuck) -- is the PLL locked?")

    # ---- audio FIFO ----
    @property
    def audio_rate(self) -> int:
        return self.mb.read_holding(REG_AUDIO_RATE, 1)[0]

    @property
    def fifo_depth(self) -> int:
        return self.mb.read_holding(REG_FIFO_DEPTH, 1)[0]

    def fifo_level(self) -> int:
        return self.mb.read_holding(REG_FIFO_LEVEL, 1)[0]

    def underruns(self) -> int:
        return self.mb.read_holding(REG_UNDERRUNS, 1)[0]

    def clear_underruns(self):
        self.mb.write_single(REG_UNDERRUNS, 0)

    def push_audio(self, samples):
        """Push up to 123 signed 16-bit samples with one FC16."""
        self.mb.write_multiple(REG_AUDIO_BASE, [int(v) & 0xFFFF for v in samples])

    def read_channel(self, ch: int) -> dict:
        r = self.mb.read_holding(ch_base(ch), CH_BLOCK_LEN)
        ftw = (r[CH_FTW_HI] << 16) | r[CH_FTW_LO]
        return {
            "ftw": ftw,
            "freq_hz": ftw_to_freq(ftw, self.dac_clk_hz),
            "phase_deg": reg_to_phase(r[CH_PHASE]),
            "amplitude": reg_to_amp(r[CH_AMPLITUDE]),
            "offset": reg_to_offset(r[CH_OFFSET]),
            "waveform": WAVEFORM_NAMES.get(r[CH_WAVEFORM], f"?{r[CH_WAVEFORM]}"),
            "duty": reg_to_duty(r[CH_DUTY]),
            "modparam": r[CH_MODPARAM],
            "depth": reg_to_depth(r[CH_MODPARAM]),
            "deviation_hz": reg_to_deviation(r[CH_MODPARAM], self.dac_clk_hz),
        }

    def write_channel(self, ch: int, block: list):
        self.mb.write_multiple(ch_base(ch), block)


# ---- audio sources for `stream` ----
def tone_samples(rate: int, freqs, level: float, seconds: float):
    """Sum of sine tones, `level` = total peak as a fraction of full scale."""
    n = int(rate * seconds)
    per = level * 32767 / max(1, len(freqs))
    for i in range(n):
        t = i / rate
        yield int(round(sum(per * math.sin(2 * math.pi * f * t) for f in freqs)))


def wav_samples(path: str, rate: int, level: float):
    """Mono-mix a 8/16/24/32-bit PCM WAV, resample (linear) to `rate`, scale."""
    with wave.open(path, "rb") as w:
        nch, width, src_rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if width == 1:
        pcm = [(b - 128) << 8 for b in raw]
    else:
        pcm = [int.from_bytes(raw[i:i + width], "little", signed=True) >> (8 * (width - 2))
               for i in range(0, len(raw), width)]
    mono = [sum(pcm[i:i + nch]) // nch for i in range(0, len(pcm), nch)]
    if not mono:
        return
    step = src_rate / rate
    pos = 0.0
    while pos < len(mono) - 1:
        i = int(pos)
        frac = pos - i
        v = mono[i] * (1 - frac) + mono[i + 1] * frac
        yield int(round(v * level))
        pos += step


def stream_audio(dev, samples, block: int = 96, log=print):
    """Pace `samples` into the device FIFO; returns (samples_sent, underruns)."""
    rate, depth = dev.audio_rate, dev.fifo_depth
    if rate == 0 or depth == 0:
        raise RuntimeError("device reports no audio FIFO (old bitstream?)")
    block = max(1, min(block, 123))
    dev.clear_underruns()
    sent, buf, t_report = 0, [], time.monotonic()
    it = iter(samples)
    eof = False
    while not eof or buf:
        while len(buf) < block and not eof:
            try:
                buf.append(next(it))
            except StopIteration:
                eof = True
        if not buf:
            break
        level = dev.fifo_level()
        if level + len(buf) > depth - 4:
            time.sleep(block / rate / 2)
            continue
        dev.push_audio(buf)
        sent += len(buf)
        buf = []
        if time.monotonic() - t_report > 1.0:
            t_report = time.monotonic()
            log(f"  {sent / rate:6.1f} s sent, fifo {level}/{depth}, underruns {dev.underruns()}")
    # let the FIFO drain before reporting underruns caused by the tail
    time.sleep(min(depth, sent) / rate)
    return sent, dev.underruns()


# ---- CLI ----
def cmd_status(dev: Acm9767, args):
    ident, ver = dev.identify()
    st = dev.status()
    ctrl = dev.control()
    print(f"device   0x{ident:04X}  version {ver >> 8}.{ver & 0xFF}")
    print(f"dac_clk  {dev.dac_clk_hz} Hz   channels {dev.channels}")
    print(f"status   pll_lock={int(bool(st & STAT_PLL_LOCK))} commit_busy={int(bool(st & STAT_COMMIT_BUSY))}")
    print(f"audio    {dev.audio_rate} S/s, fifo {dev.fifo_level()}/{dev.fifo_depth}, "
          f"underruns {dev.underruns()}")
    for ch in range(1, dev.channels + 1):
        c = dev.read_channel(ch)
        en = "on " if ctrl & (1 << (ch - 1)) else "off"
        inv = " inverted" if ctrl & (1 << (ch + 1)) else ""
        extra = ""
        if c["waveform"] == "am":
            extra = f"  depth {c['depth']:.2f}"
        elif c["waveform"] == "fm":
            extra = f"  deviation {c['deviation_hz']:.0f} Hz"
        print(f"ch{ch} [{en}]{inv}  {c['waveform']:8s} {c['freq_hz']:14.3f} Hz  amp {c['amplitude']:.4f}  "
              f"offset {c['offset']:+.4f}  phase {c['phase_deg']:7.2f} deg  duty {c['duty']:.3f}{extra}")


def cmd_set(dev: Acm9767, args):
    dev.require_channel(args.channel)
    mp = modparam_for(args.wave, dev.dac_clk_hz, args.depth, args.deviation)
    block = channel_block(args.freq, dev.dac_clk_hz, args.wave, args.amp, args.offset, args.phase,
                          args.duty, mp)
    dev.write_channel(args.channel, block)
    print(f"ch{args.channel}: {args.wave} {ftw_to_freq((block[1] << 16) | block[0], dev.dac_clk_hz):.3f} Hz "
          f"(ftw 0x{(block[1] << 16) | block[0]:08X}) amp {args.amp} offset {args.offset:+} "
          f"phase {args.phase} deg duty {args.duty} modparam 0x{mp:04X}")
    if not args.no_commit:
        dev.commit()
        print("committed")


def _set_ctrl_bits(dev: Acm9767, bits: int, on: bool, commit: bool):
    if bits & 0b1010 and dev.channels < 2:   # ch2 enable / invert
        dev.require_channel(2)
    ctrl = dev.control()
    ctrl = (ctrl | bits) if on else (ctrl & ~bits)
    dev.set_control(ctrl)
    if commit:
        dev.commit()
    return ctrl


def cmd_enable(dev: Acm9767, args):
    bits = sum(1 << (ch - 1) for ch in args.channels)
    ctrl = _set_ctrl_bits(dev, bits, True, not args.no_commit)
    print(f"control = 0x{ctrl:X}" + ("" if args.no_commit else " (committed)"))


def cmd_disable(dev: Acm9767, args):
    bits = sum(1 << (ch - 1) for ch in args.channels)
    ctrl = _set_ctrl_bits(dev, bits, False, not args.no_commit)
    print(f"control = 0x{ctrl:X}" + ("" if args.no_commit else " (committed)"))


def cmd_invert(dev: Acm9767, args):
    bits = sum(1 << (ch + 1) for ch in args.channels)
    ctrl = _set_ctrl_bits(dev, bits, args.on, not args.no_commit)
    print(f"control = 0x{ctrl:X}" + ("" if args.no_commit else " (committed)"))


def cmd_stream(dev: Acm9767, args):
    rate = dev.audio_rate
    if args.carrier is not None:
        cur = dev.read_channel(1)
        wave_name = MODES["lsb" if args.lsb else args.mode]
        mp = modparam_for(wave_name, dev.dac_clk_hz, args.depth, args.deviation)
        block = channel_block(args.carrier, dev.dac_clk_hz, wave_name, cur["amplitude"], cur["offset"],
                              cur["phase_deg"], cur["duty"], mp)
        dev.write_channel(1, block)
        dev.set_control(dev.control() | 0x1)
        dev.commit()
        detail = {"am": f", depth {args.depth:.2f}",
                  "fm": f", deviation {args.deviation:.0f} Hz"}.get(wave_name, "")
        print(f"ch1: {wave_name} on {args.carrier:.0f} Hz carrier{detail}, enabled")
    if args.wav:
        src = wav_samples(args.wav, rate, args.level)
        what = args.wav
    else:
        freqs = [args.tone] + ([args.tone2] if args.tone2 else [])
        src = tone_samples(rate, freqs, args.level, args.seconds)
        what = " + ".join(f"{f:.0f} Hz" for f in freqs) + f" for {args.seconds} s"
    print(f"streaming {what} at {rate} S/s (block {args.block})")
    sent, under = stream_audio(dev, src, args.block)
    print(f"done: {sent} samples ({sent / rate:.2f} s), {under} underruns")


def cmd_commit(dev: Acm9767, args):
    dev.commit()
    print("committed")


def cmd_read(dev: Acm9767, args):
    for i, v in enumerate(dev.mb.read_holding(args.addr, args.count)):
        print(f"  reg[0x{args.addr + i:02X}] = 0x{v:04X} ({v})")


def cmd_write(dev: Acm9767, args):
    dev.mb.write_single(args.addr, args.value)
    print(f"  wrote reg[0x{args.addr:02X}] = 0x{args.value & 0xFFFF:04X} (not committed)")


def parse_int(s):
    return int(s, 0)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT})")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("-s", "--slave", type=int, default=DEFAULT_SLAVE)
    ap.add_argument("--timeout", type=float, default=1.0, help="response timeout, s")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="show identity, PLL lock and both channels").set_defaults(fn=cmd_status)

    p = sub.add_parser("set", help="program one channel (and commit)")
    p.add_argument("channel", type=int, choices=(1, 2))
    p.add_argument("--freq", type=float, required=True, help="output frequency, Hz")
    p.add_argument("--wave", choices=sorted(WAVEFORMS), default="sine")
    p.add_argument("--amp", type=float, default=1.0, help="0.0..1.0 of full scale")
    p.add_argument("--offset", type=float, default=0.0, help="-1.0..1.0 of full scale")
    p.add_argument("--phase", type=float, default=0.0, help="phase offset, degrees")
    p.add_argument("--duty", type=float, default=0.5, help="square-wave duty 0.0..1.0")
    p.add_argument("--depth", type=float, default=1.0, help="AM modulation depth 0..1 (default 1.0)")
    p.add_argument("--deviation", type=float, default=DEFAULT_FM_DEVIATION_HZ,
                   help="FM peak deviation at full-scale audio, Hz (default 5000 = NFM)")
    p.add_argument("--no-commit", action="store_true", help="stage only")
    p.set_defaults(fn=cmd_set)

    for name, fn in (("enable", cmd_enable), ("disable", cmd_disable)):
        p = sub.add_parser(name, help=f"{name} channels (and commit)")
        p.add_argument("channels", type=int, nargs="+", choices=(1, 2))
        p.add_argument("--no-commit", action="store_true")
        p.set_defaults(fn=fn)

    p = sub.add_parser("invert", help="set/clear output polarity inversion")
    p.add_argument("channels", type=int, nargs="+", choices=(1, 2))
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--on", dest="on", action="store_true")
    g.add_argument("--off", dest="on", action="store_false")
    p.add_argument("--no-commit", action="store_true")
    p.set_defaults(fn=cmd_invert)

    sub.add_parser("commit", help="apply the staged registers").set_defaults(fn=cmd_commit)

    p = sub.add_parser("stream", help="stream baseband audio (SSB / AM / FM) on channel 1")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--wav", help="PCM WAV file (any rate/channels; resampled, mono-mixed)")
    src.add_argument("--tone", type=float, default=1000.0, help="test tone, Hz (default 1000)")
    p.add_argument("--tone2", type=float, help="second tone for a two-tone test, Hz")
    p.add_argument("--seconds", type=float, default=5.0, help="tone duration (default 5)")
    p.add_argument("--level", type=float, default=0.9, help="peak level 0..1 of full scale (default 0.9)")
    p.add_argument("--carrier", type=float,
                   help="set ch1 to the chosen mode on this carrier (Hz) and enable it")
    p.add_argument("--mode", choices=sorted(MODES), default="usb", help="usb (default), lsb, am, fm/nfm")
    p.add_argument("--lsb", action="store_true", help="shorthand for --mode lsb")
    p.add_argument("--depth", type=float, default=1.0, help="AM depth 0..1 (default 1.0)")
    p.add_argument("--deviation", type=float, default=DEFAULT_FM_DEVIATION_HZ,
                   help="FM peak deviation, Hz (default 5000 = NFM)")
    p.add_argument("--block", type=int, default=96, help="samples per Modbus write (<= 123)")
    p.set_defaults(fn=cmd_stream)

    p = sub.add_parser("read", help="raw holding-register read")
    p.add_argument("addr", type=parse_int)
    p.add_argument("count", type=int, nargs="?", default=1)
    p.set_defaults(fn=cmd_read)

    p = sub.add_parser("write", help="raw holding-register write (no commit)")
    p.add_argument("addr", type=parse_int)
    p.add_argument("value", type=parse_int)
    p.set_defaults(fn=cmd_write)
    return ap


def main(argv=None, transport=None):
    args = build_parser().parse_args(argv)
    try:
        mb = ModbusRTU(args.port, args.baud, args.slave, args.timeout, transport=transport)
    except Exception as e:  # serial open errors
        sys.exit(f"error: cannot open {args.port}: {e}")
    try:
        args.fn(Acm9767(mb), args)
    except (TimeoutError, ModbusError, ValueError, RuntimeError) as e:
        sys.exit(f"error: {e}")
    finally:
        mb.close()


if __name__ == "__main__":
    main()
