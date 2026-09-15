"""Host-side tests that run without hardware.

A FakeSlave implements the FPGA's Modbus register semantics (staged / committed
banks, RO registers, STATUS bits) behind the transport interface ModbusRTU
expects, so the CLI, the unit conversions and the RTU framing are exercised
end to end.

    python -m pytest scripts/tests -q
"""

import math
import os
import struct
import sys
import wave

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import acm9767_ctl as ctl  # noqa: E402
from modbus_rtu import ModbusRTU, crc16  # noqa: E402

DAC_CLK = 54_000_000


class FakeSlave:
    """In-memory model of dac_regs behind a Modbus RTU slave (id 7)."""

    def __init__(self, slave=7, dac_clk=DAC_CLK, channels=2):
        self.slave = slave
        self.regs = {0x00: ctl.ACM9767_ID, 0x01: 0x0100, 0x02: 0x0001,
                     0x05: dac_clk & 0xFFFF, 0x06: dac_clk >> 16, 0x07: channels, 0x03: 0,
                     0x08: 0, 0x09: 0, 0x0A: 16000, 0x0B: 2048}
        self.audio = []
        for ch in (1, 2):
            b = ctl.ch_base(ch)
            self.regs.update({b + i: 0 for i in range(8)})
            self.regs[b + ctl.CH_AMPLITUDE] = 0xFFFF
            self.regs[b + ctl.CH_DUTY] = 0x8000
        self.committed = None
        self.commits = 0
        self._rx = b""

    # --- transport interface ---
    def reset_input_buffer(self):
        self._rx = b""

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def close(self):
        pass

    def write(self, frame):
        assert crc16(frame) == 0, "request CRC"
        addr, func = frame[0], frame[1]
        assert addr == self.slave
        if func == 0x03:
            start, qty = struct.unpack(">HH", frame[2:6])
            if start + qty > 0x180:
                return self._exc(func, 2)
            data = b"".join(struct.pack(">H", self.regs.get(start + i, 0)) for i in range(qty))
            self._reply(bytes([addr, func, len(data)]) + data)
        elif func == 0x06:
            reg, val = struct.unpack(">HH", frame[2:6])
            self._write(reg, val)
            self._reply(frame[:6])
        elif func == 0x10:
            start, qty, bc = struct.unpack(">HHB", frame[2:7])
            vals = struct.unpack(">" + "H" * qty, frame[7:7 + bc])
            for i, v in enumerate(vals):
                self._write(start + i, v)
            self._reply(frame[:6])
        else:
            self._exc(func, 1)

    def _write(self, reg, val):
        if 0x100 <= reg <= 0x17F:
            self.audio.append(val - 0x10000 if val & 0x8000 else val)
        elif reg == ctl.REG_UNDERRUNS:
            self.regs[0x09] = 0
        elif reg == ctl.REG_COMMIT:
            self.committed = {k: v for k, v in self.regs.items() if k >= 0x10 or k == ctl.REG_CONTROL}
            self.commits += 1
        elif reg == ctl.REG_CONTROL or reg >= 0x10:
            self.regs[reg] = val & (0xF if reg % 0x10 == ctl.CH_WAVEFORM and reg >= 0x10 else 0xFFFF)
        # everything else is read-only

    def _reply(self, body):
        self._rx += body + struct.pack("<H", crc16(body))

    def _exc(self, func, code):
        self._reply(bytes([self.slave, func | 0x80, code]))


@pytest.fixture
def slave():
    return FakeSlave()


def run(slave, *argv, capsys=None):
    ctl.main(list(argv), transport=slave)
    return capsys.readouterr().out if capsys else None


def test_freq_ftw_roundtrip():
    ftw = ctl.freq_to_ftw(1_000_000, DAC_CLK)
    assert ftw == round(2**32 / 54)
    assert abs(ctl.ftw_to_freq(ftw, DAC_CLK) - 1_000_000) < 0.02
    with pytest.raises(ValueError):
        ctl.freq_to_ftw(DAC_CLK / 2, DAC_CLK)


def test_unit_conversions():
    assert ctl.amp_to_reg(1.0) == 0xFFFF
    assert ctl.amp_to_reg(0.5) == 0x7FFF
    assert ctl.amp_to_reg(0.0) == 0
    assert ctl.offset_to_reg(-1.0) == (-8191) & 0xFFFF
    assert ctl.reg_to_offset(ctl.offset_to_reg(-0.25)) == pytest.approx(-0.25, abs=1e-3)
    assert ctl.phase_to_reg(90) == 0x4000
    assert ctl.phase_to_reg(450) == 0x4000
    assert ctl.duty_to_reg(0.5) == 0x8000
    assert ctl.duty_to_reg(1.0) == 0xFFFF


def test_set_stages_block_and_commits(slave, capsys):
    out = run(slave, "set", "1", "--freq", "1e6", "--wave", "square", "--amp", "0.5",
              "--offset", "-0.25", "--phase", "90", "--duty", "0.25", capsys=capsys)
    assert "committed" in out
    b = ctl.ch_base(1)
    ftw = (slave.regs[b + ctl.CH_FTW_HI] << 16) | slave.regs[b + ctl.CH_FTW_LO]
    assert ftw == ctl.freq_to_ftw(1e6, DAC_CLK)
    assert slave.regs[b + ctl.CH_WAVEFORM] == ctl.WAVEFORMS["square"]
    assert slave.regs[b + ctl.CH_AMPLITUDE] == 0x7FFF
    assert slave.regs[b + ctl.CH_PHASE] == 0x4000
    assert slave.regs[b + ctl.CH_DUTY] == 0x4000
    assert slave.commits == 1
    # channel 2 untouched
    assert slave.regs[ctl.ch_base(2) + ctl.CH_FTW_LO] == 0


def test_set_no_commit(slave):
    run(slave, "set", "2", "--freq", "12345", "--no-commit")
    assert slave.commits == 0
    assert slave.regs[ctl.ch_base(2) + ctl.CH_FTW_LO] != 0


def test_enable_disable_invert(slave):
    run(slave, "enable", "1", "2")
    assert slave.regs[ctl.REG_CONTROL] == 0b0011 and slave.commits == 1
    run(slave, "disable", "1")
    assert slave.regs[ctl.REG_CONTROL] == 0b0010
    run(slave, "invert", "2", "--on")
    assert slave.regs[ctl.REG_CONTROL] == 0b1010
    run(slave, "invert", "2", "--off", "--no-commit")
    assert slave.regs[ctl.REG_CONTROL] == 0b0010 and slave.commits == 3


def test_status_reads_back_physical_units(slave, capsys):
    run(slave, "set", "1", "--freq", "2.5e6", "--wave", "triangle", "--amp", "0.25")
    run(slave, "enable", "1")
    out = run(slave, "status", capsys=capsys)
    assert "0x9767" in out and "pll_lock=1" in out and "channels 2" in out
    assert "ch1 [on ]" in out and "triangle" in out and "2500000.0" in out
    assert "ch2 [off]" in out


def test_raw_read_write_and_exception(slave, capsys):
    run(slave, "write", "0x13", "0x1234")
    out = run(slave, "read", "0x13", capsys=capsys)
    assert "0x1234" in out
    with pytest.raises(SystemExit) as e:
        run(slave, "read", "0x17e", "4")
    assert "illegal data address" in str(e.value)


def test_wrong_device_id_is_rejected(capsys):
    s = FakeSlave()
    s.regs[0x00] = 0x1234
    with pytest.raises(SystemExit) as e:
        run(s, "status")
    assert "unexpected device id" in str(e.value)


def test_modbus_crc_vector():
    assert crc16(bytes([1, 3, 0, 0, 0, 1])) == 0x0A84


def test_modbus_rtu_read_uses_transport(slave):
    mb = ModbusRTU(transport=slave)
    assert mb.read_holding(0x00, 2) == [ctl.ACM9767_ID, 0x0100]


def test_single_channel_build_hides_and_rejects_channel_2(capsys):
    s = FakeSlave(channels=1)
    out = run(s, "status", capsys=capsys)
    assert "channels 1" in out and "ch1 [" in out and "ch2 [" not in out
    with pytest.raises(SystemExit) as e:
        run(s, "set", "2", "--freq", "1e3")
    assert "not wired" in str(e.value)
    with pytest.raises(SystemExit):
        run(s, "enable", "2")
    run(s, "enable", "1")           # channel 1 still works
    assert s.regs[ctl.REG_CONTROL] == 0b0001


def test_stream_tone(slave, capsys, monkeypatch):
    monkeypatch.setattr(ctl.time, "sleep", lambda s: None)
    out = run(slave, "stream", "--tone", "1000", "--seconds", "0.05", "--level", "0.5",
              "--carrier", "7.1e6", capsys=capsys)
    assert "ssb-usb on 7100000 Hz" in out and "800 samples" in out
    assert len(slave.audio) == 800
    assert slave.audio[0] == 0 and max(slave.audio) == pytest.approx(0.5 * 32767, abs=40)
    # sample 4 of a 1 kHz tone at 16 kS/s is the positive peak
    assert slave.audio[4] == pytest.approx(0.5 * 32767, abs=2)
    assert slave.regs[ctl.ch_base(1) + ctl.CH_WAVEFORM] == ctl.WAVEFORMS["ssb-usb"]
    assert slave.regs[ctl.REG_CONTROL] & 1


def test_stream_two_tone_and_lsb(slave, monkeypatch):
    monkeypatch.setattr(ctl.time, "sleep", lambda s: None)
    run(slave, "stream", "--tone", "700", "--tone2", "1900", "--seconds", "0.01", "--carrier", "1e6", "--lsb")
    assert len(slave.audio) == 160
    assert slave.regs[ctl.ch_base(1) + ctl.CH_WAVEFORM] == ctl.WAVEFORMS["ssb-lsb"]


def test_stream_wav_is_resampled(slave, tmp_path, monkeypatch):
    monkeypatch.setattr(ctl.time, "sleep", lambda s: None)
    path = tmp_path / "t.wav"
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(8000)
        frames = b"".join(struct.pack("<hh", int(10000 * math.sin(2 * math.pi * 440 * i / 8000)), 0)
                          for i in range(800))
        w.writeframes(frames)
    run(slave, "stream", "--wav", str(path), "--level", "1.0")
    assert 1590 <= len(slave.audio) <= 1600          # 0.1 s at 16 kS/s
    assert max(slave.audio) == pytest.approx(5000, abs=60)   # stereo mono-mix halves the amplitude


def test_set_ssb_waveform_by_name(slave):
    run(slave, "set", "1", "--freq", "14.2e6", "--wave", "ssb-lsb")
    assert slave.regs[ctl.ch_base(1) + ctl.CH_WAVEFORM] == 6


def test_am_and_fm_parameters(slave, capsys):
    run(slave, "set", "1", "--freq", "1e6", "--wave", "am", "--depth", "0.5")
    b = ctl.ch_base(1)
    assert slave.regs[b + ctl.CH_WAVEFORM] == 7 and slave.regs[b + ctl.CH_MODPARAM] == 0x8000
    run(slave, "set", "1", "--freq", "10.7e6", "--wave", "nfm")            # default 5 kHz deviation
    assert slave.regs[b + ctl.CH_WAVEFORM] == 8
    assert slave.regs[b + ctl.CH_MODPARAM] == round(5000 * 2**32 / DAC_CLK / 256)
    out = run(slave, "status", capsys=capsys)
    assert "fm " in out and ("deviation 5000 Hz" in out or "deviation 4999 Hz" in out)   # 3.2 Hz steps
    run(slave, "set", "1", "--freq", "1e6", "--wave", "am", "--depth", "1.0")
    out = run(slave, "status", capsys=capsys)
    assert "am " in out and "depth 1.00" in out
    with pytest.raises(SystemExit):
        run(slave, "set", "1", "--freq", "1e6", "--wave", "fm", "--deviation", "1e7")


def test_stream_modes(slave, monkeypatch):
    monkeypatch.setattr(ctl.time, "sleep", lambda s: None)
    b = ctl.ch_base(1)
    run(slave, "stream", "--carrier", "1e6", "--mode", "am", "--depth", "0.8", "--seconds", "0.01")
    assert slave.regs[b + ctl.CH_WAVEFORM] == 7 and slave.regs[b + ctl.CH_MODPARAM] == round(0.8 * 65536)
    run(slave, "stream", "--carrier", "1e6", "--mode", "nfm", "--deviation", "2500", "--seconds", "0.01")
    assert slave.regs[b + ctl.CH_WAVEFORM] == 8
    assert slave.regs[b + ctl.CH_MODPARAM] == round(2500 * 2**32 / DAC_CLK / 256)
    run(slave, "stream", "--carrier", "1e6", "--lsb", "--seconds", "0.01")
    assert slave.regs[b + ctl.CH_WAVEFORM] == 6
