#!/usr/bin/env python3
"""Modulator -> demodulator loopback check for the SSB / AM / FM gateware.

1. Generates (or loads) baseband audio at the platform audio rate and writes it
   as int16 big-endian for the simulation.
2. Runs the file-driven Icarus simulation of the real ssb_baseband +
   dds_channel chain (sim/unit/ssb_baseband/loopback.sv), which dumps every
   DAC code at the DAC rate as uint16 big-endian.
3. Demodulates the DAC samples with an analytic-signal demodulator (FFT
   Hilbert transform -> complex baseband -> low-pass -> decimate) or the AM /
   FM equivalents, aligns the recovered audio to the input at full rate and
   scores it: SNR (raw and with the known CIC droop equalised), unwanted
   sideband and carrier suppression from the RF spectrum.

    scripts/ssb_loopback.py --sim build/sim/tests/unit/ssb_baseband/loopback/unit_ssb_baseband_loopback.bin
    scripts/ssb_loopback.py --sim ... --mode lsb --wav voice.wav --seconds 0.1 --save-wav
    scripts/ssb_loopback.py --demod-only --in in.bin --out out.bin --mode usb   # re-score existing files

Exit status 0 when every metric clears its threshold.
"""

import argparse
import math
import os
import subprocess
import sys
import wave

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import platform_config as pc  # noqa: E402

WAVE_CODE = {"usb": 5, "lsb": 6, "am": 7, "fm": 8}
# Platform constants; the Tiny Tapeout build overrides them with --fs-dac /
# --audio-rate / --max-delay (its clock is 50 MHz and its Hilbert network is
# a short IIR, not a 127-tap FIR).
FS_DAC = pc.DAC_CLK_HZ
FS_AUDIO = pc.SSB_AUDIO_RATE_HZ
INTERP = pc.SSB_INTERP
TAPS = pc.SSB_HILBERT_TAPS          # also the alignment search window, in audio samples
DC_SHIFT = getattr(pc, "SSB_DC_BLOCK_SHIFT", 0)   # the modulator's DC blocker leak, 2^-shift
TONES = [(400.0, 0.25), (1000.0, 0.35), (1700.0, 0.20), (2600.0, 0.20)]   # Hz, fraction of level
PAD = 256          # silence samples before/after the audio so the FIR/CIC start and end in a defined state
AUDIO_BW = 5000.0
DEFAULT_CARRIER = FS_DAC / 32          # 1.6875 MHz at 54 MHz
DEFAULT_DEVIATION = 5000.0
DEFAULT_DEPTH = 0.8


# ---------------------------------------------------------------- sources
def tone_audio(seconds: float, level: float, tones=None, sweep=None) -> np.ndarray:
    """Sum of tones, optionally plus a linear chirp `sweep` = (f0, f1) over the
    whole duration (weighted like one tone)."""
    tones = tones or TONES
    n = int(round(seconds * FS_AUDIO))
    t = np.arange(n) / FS_AUDIO
    x = sum(a * np.sin(2 * np.pi * f * t) for f, a in tones)
    if sweep:
        f0, f1 = sweep
        phase = 2 * np.pi * (f0 * t + (f1 - f0) * t * t / (2 * seconds))
        x = x + np.mean([a for _, a in tones]) * np.sin(phase)
    # gentle amplitude envelope so the check is not a pure steady-state one
    x *= 0.7 + 0.3 * np.sin(2 * np.pi * 7.0 * t)
    x = x / np.max(np.abs(x)) * level
    return np.round(fade(x) * 32767).astype(np.int16)


def fade(x: np.ndarray, ms: float = 5.0) -> np.ndarray:
    """Raised-cosine fade in/out so the on/off transients do not splatter."""
    k = min(len(x) // 4, int(ms * 1e-3 * FS_AUDIO))
    w = np.ones(len(x))
    ramp = 0.5 - 0.5 * np.cos(np.pi * np.arange(k) / k)
    w[:k] = ramp
    w[-k:] = ramp[::-1]
    return x * w


def wav_audio(path: str, seconds: float, level: float) -> np.ndarray:
    with wave.open(path, "rb") as w:
        nch, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    dt = {1: np.uint8, 2: np.int16, 4: np.int32}[width] if width != 3 else None
    if dt is None:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3)
        pcm = (b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8) | (b[:, 2].astype(np.int32) << 16))
        pcm = np.where(pcm & 0x800000, pcm - (1 << 24), pcm).astype(np.float64) / (1 << 23)
    else:
        pcm = np.frombuffer(raw, dt).astype(np.float64)
        pcm = (pcm - 128) / 128 if width == 1 else pcm / (1 << (8 * width - 1))
    mono = pcm.reshape(-1, nch).mean(axis=1)
    if rate != FS_AUDIO:
        src_t = np.arange(len(mono)) / rate
        dst_t = np.arange(int(len(mono) * FS_AUDIO / rate)) / FS_AUDIO
        mono = np.interp(dst_t, src_t, mono)
    mono = mono[: int(seconds * FS_AUDIO)]
    mono = mono / max(1e-9, np.max(np.abs(mono))) * level
    return np.round(fade(mono) * 32767).astype(np.int16)


# ---------------------------------------------------------------- simulation
def padded(audio: np.ndarray) -> np.ndarray:
    z = np.zeros(PAD, dtype=np.int16)
    return np.concatenate([z, audio.astype(np.int16), z])


def run_sim(vvp: str, sim: str, audio: np.ndarray, in_path: str, out_path: str,
            ftw: int, wave_code: int, modparam: int) -> None:
    """`audio` is written with PAD zeros on both sides (see padded())."""
    audio = padded(audio)
    audio.astype(">i2").tofile(in_path)
    cmd = [vvp, "-n", sim, f"+in={in_path}", f"+out={out_path}", f"+n={len(audio)}",
           f"+ftw={ftw}", f"+wave={wave_code}", f"+modparam={modparam}"]
    print("  " + " ".join(cmd))
    res = subprocess.run(cmd, capture_output=True, text=True)
    if "Test passed" not in res.stdout:
        sys.exit(f"simulation failed:\n{res.stdout}\n{res.stderr}")


# ---------------------------------------------------------------- demodulators
def analytic(x: np.ndarray) -> np.ndarray:
    """FFT-based analytic signal (positive frequencies doubled, negative zeroed)."""
    n = len(x)
    X = np.fft.fft(x)
    h = np.zeros(n)
    h[0] = 1
    if n % 2 == 0:
        h[n // 2] = 1
        h[1:n // 2] = 2
    else:
        h[1:(n + 1) // 2] = 2
    return np.fft.ifft(X * h)


def lowpass(x: np.ndarray, fs: float, bw: float) -> np.ndarray:
    X = np.fft.fft(x)
    f = np.fft.fftfreq(len(x), 1 / fs)
    X[np.abs(f) > bw] = 0
    return np.fft.ifft(X)


def demod_ssb(rf: np.ndarray, fc: float, mode: str) -> np.ndarray:
    """Complex analytic baseband m + j*hilbert(m) (times an unknown carrier
    phase, which the scoring fits). The real part is the audio."""
    n = np.arange(len(rf))
    bb = lowpass(analytic(rf) * np.exp(-2j * np.pi * fc * n / FS_DAC), FS_DAC, AUDIO_BW)
    return np.conj(bb) if mode == "lsb" else bb           # LSB carries the conjugate


def demod_am(rf: np.ndarray, fc: float) -> np.ndarray:
    env = np.abs(analytic(rf))                            # envelope
    env = lowpass(env, FS_DAC, AUDIO_BW).real
    return env - np.mean(env)                             # remove the carrier term


def demod_fm(rf: np.ndarray, fc: float) -> np.ndarray:
    n = np.arange(len(rf))
    bb = lowpass(analytic(rf) * np.exp(-2j * np.pi * fc * n / FS_DAC), FS_DAC, 60e3)
    ph = np.unwrap(np.angle(bb))
    inst = np.diff(ph) * FS_DAC / (2 * np.pi)             # instantaneous frequency offset, Hz
    inst = np.append(inst, inst[-1])
    return lowpass(inst, FS_DAC, AUDIO_BW).real


# ---------------------------------------------------------------- scoring
def cic_droop(f: np.ndarray) -> np.ndarray:
    x = np.pi * f / FS_AUDIO
    return np.where(np.abs(x) < 1e-12, 1.0, (np.sin(x) / np.where(np.abs(x) < 1e-12, 1, x)) ** 3)


def upsample(x: np.ndarray, factor: int) -> np.ndarray:
    """Band-limited upsample by zero-stuffing in the frequency domain."""
    n = len(x)
    X = np.fft.rfft(x)
    Y = np.zeros(n * factor // 2 + 1, dtype=complex)
    Y[: len(X)] = X
    return np.fft.irfft(Y, n * factor) * factor


def align_and_score(ref_hi: np.ndarray, got: np.ndarray, max_lag: int):
    """Find the lag (samples at the DAC rate) that best aligns `got` to `ref_hi`
    and the complex gain (magnitude + carrier phase for SSB) that fits it;
    return (lag, scale, snr_db) over the overlapping region. Works for real
    (AM/FM) and complex analytic (SSB) signals."""
    n = min(len(ref_hi), len(got))
    r = ref_hi[:n] - ref_hi[:n].mean()
    g = got[:n] - got[:n].mean()
    corr = np.fft.ifft(np.fft.fft(g, 2 * n) * np.conj(np.fft.fft(r, 2 * n)))
    lags = np.arange(2 * n)
    lags[lags >= n] -= 2 * n
    valid = (lags >= 0) & (lags <= max_lag)
    lag = int(lags[valid][np.argmax(np.abs(corr[valid]))])
    seg_r = r[: n - lag]
    seg_g = g[lag:n]
    scale = np.vdot(seg_r, seg_g) / max(1e-12, np.vdot(seg_r, seg_r).real)
    err = seg_g - scale * seg_r
    snr = 20 * math.log10(np.std(scale * seg_r) / max(1e-12, np.std(err)))
    return lag, scale, snr


def band_power(spec: np.ndarray, f: np.ndarray, lo: float, hi: float) -> float:
    m = (f >= lo) & (f <= hi)
    return float(np.sum(np.abs(spec[m]) ** 2))


def per_tone_report(x: np.ndarray, got: np.ndarray, lag: int, scale, tones) -> None:
    """Gain and phase of each tone in the recovered audio relative to the input
    (FFT bins over the middle 60 %, Hann window), against the CIC droop model,
    after removing one common complex gain fitted to the model."""
    dec = got[lag::INTERP][: len(x)]
    if np.iscomplexobj(dec):
        dec = (dec * np.exp(-1j * np.angle(scale))).real
    n = len(x)
    sl = slice(n // 5, 4 * n // 5)
    w = np.hanning(sl.stop - sl.start)
    D = np.fft.rfft(dec[sl] * w)
    X = np.fft.rfft(x[sl] * w)
    fr = np.fft.rfftfreq(sl.stop - sl.start, 1 / FS_AUDIO)
    gains, models = [], []
    for f, _ in tones:
        k = int(np.argmin(np.abs(fr - f)))
        gains.append(D[k] / X[k] if abs(X[k]) > 1e-9 else 0.0)
        models.append(float(cic_droop(np.array([f]))[0]))
    g0 = sum(g * m for g, m in zip(gains, models)) / sum(m * m for m in models)
    print("  tone [Hz]   gain [dB]  model [dB]  error [dB]  phase err [deg]  tone SNR [dB]"
          "  (vs CIC droop model)")
    for (f, _), g, model in zip(tones, gains, models):
        g = g / g0
        err_db = 20 * math.log10(max(1e-9, abs(g)) / model)
        tone_snr = -20 * math.log10(max(1e-9, abs(g / model - 1.0)))
        print(f"  {f:8.0f}   {20 * math.log10(max(1e-9, abs(g))):9.3f}  {20 * math.log10(model):10.3f}  "
              f"{err_db:10.3f}  {math.degrees(np.angle(g)):14.2f}  {tone_snr:12.1f}")


def allpass_chain(x: np.ndarray, poles) -> np.ndarray:
    """Cascade of first-order all-pass sections (a + z^-1) / (1 + a z^-1)."""
    from scipy.signal import lfilter
    for a in poles:
        x = lfilter([a, 1.0], [1.0, a], x)
    return x


def dc_block(x: np.ndarray, shift: int) -> np.ndarray:
    """The modulator's first-order DC blocker: y[n] = x[n] - x[n-1] + (1 - 2^-shift) y[n-1]."""
    from scipy.signal import lfilter
    a = 1.0 - 2.0 ** -shift
    return lfilter([1.0, -1.0], [1.0, -a], x)


def score(audio: np.ndarray, rf: np.ndarray, mode: str, fc: float, modparam: int, save_wav: str | None,
          tones=None, allpass=None, dc_shift=0):
    x = audio.astype(np.float64) / 32768
    if dc_shift:
        x = dc_block(x, dc_shift)                 # reference as the FIR sees it (FPGA build)
    if allpass:
        x = allpass_chain(x, allpass)             # reference as the modulator's I branch sees it
    ref_hi = upsample(x, INTERP)                          # input at the DAC rate
    if mode in ("usb", "lsb"):
        got = demod_ssb(rf, fc, mode)
        ref_hi = analytic(ref_hi)                         # compare analytic signals
    elif mode == "am":
        got = demod_am(rf, fc)
    else:
        got = demod_fm(rf, fc)
    max_lag = (TAPS + 20) * INTERP
    lag, scale, snr_raw = align_and_score(ref_hi, got, max_lag)

    # the same, with the reference passed through the CIC's known sinc^3 droop
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / FS_AUDIO)
    ref_eq = upsample(np.fft.irfft(X * cic_droop(f), len(x)), INTERP)
    if mode in ("usb", "lsb"):
        ref_eq = analytic(ref_eq)
    _, _, snr_eq = align_and_score(ref_eq, got, max_lag)
    # steady state: the middle 60 % of the audio, excluding the fade edges
    # (whose sub-200 Hz content lies below the Hilbert transformer's passband)
    n_audio = len(x) - 2 * PAD
    a = (PAD + n_audio // 5) * INTERP
    b = (PAD + 4 * n_audio // 5) * INTERP
    _, _, snr_steady = align_and_score(ref_eq[a:b], got[lag + a:lag + b], 2)

    # RF spectrum around the carrier, over the steady-state middle of the dump
    # (skip the FIR/CIC fill at the start and the flush at the end)
    lo_i = (PAD + TAPS) * INTERP
    hi_i = lag + (len(audio) - PAD) * INTERP
    mid = rf[lo_i:hi_i] if hi_i - lo_i > 4 * INTERP else rf
    win = np.hanning(len(mid))
    S = np.fft.rfft(mid * win)
    fr = np.fft.rfftfreq(len(mid), 1 / FS_DAC)
    p_up = band_power(S, fr, fc + 200, fc + AUDIO_BW)
    p_dn = band_power(S, fr, fc - AUDIO_BW, fc - 200)
    p_car = band_power(S, fr, fc - 60, fc + 60)
    if mode == "usb":
        wanted, unwanted = p_up, p_dn
    elif mode == "lsb":
        wanted, unwanted = p_dn, p_up
    else:
        wanted, unwanted = p_up + p_dn, None
    metrics = {
        "lag_ms": lag / FS_DAC * 1e3,
        "gain": abs(scale),
        "phase_deg": math.degrees(np.angle(scale)) if np.iscomplexobj(scale) else 0.0,
        "snr_raw_db": snr_raw,
        "snr_eq_db": snr_eq,
        "snr_steady_db": snr_steady,
        "carrier_db": 10 * math.log10(wanted / max(p_car, 1e-12)),
    }
    if unwanted is not None:
        metrics["sideband_db"] = 10 * math.log10(wanted / max(unwanted, 1e-12))
    if tones:
        per_tone_report(x, got, lag, scale, tones)
    if save_wav:
        # de-rotate the fitted carrier phase, keep the audio (real part)
        dec = (got[lag::INTERP][: len(x)] * np.exp(-1j * np.angle(scale))).real
        dec = dec / max(1e-9, np.max(np.abs(dec))) * 0.9
        with wave.open(save_wav, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(FS_AUDIO)
            w.writeframes(np.round(dec * 32767).astype("<i2").tobytes())
    return metrics


THRESHOLDS = {
    "usb": {"snr_eq_db": 30.0, "sideband_db": 40.0, "carrier_db": 40.0},
    "lsb": {"snr_eq_db": 30.0, "sideband_db": 40.0, "carrier_db": 40.0},
    "am":  {"snr_eq_db": 30.0},
    "fm":  {"snr_eq_db": 25.0},
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vvp", default="vvp")
    ap.add_argument("--sim", help="compiled loopback simulation (unit_ssb_baseband_loopback.bin)")
    ap.add_argument("--mode", choices=sorted(WAVE_CODE), default="usb")
    ap.add_argument("--carrier", type=float, default=DEFAULT_CARRIER,
                    help=f"Hz (default {DEFAULT_CARRIER:.0f})")
    ap.add_argument("--seconds", type=float, default=0.04, help="audio length (default 0.04)")
    ap.add_argument("--level", type=float, default=0.8)
    ap.add_argument("--wav", help="use this WAV file as the audio instead of the test tones")
    ap.add_argument("--tones", help="comma-separated tone frequencies in Hz, equal amplitudes "
                                    "(default 400,1000,1700,2600 with unequal amplitudes)")
    ap.add_argument("--sweep", help="add a linear chirp F0,F1 (Hz) spanning the whole duration")
    ap.add_argument("--depth", type=float, default=DEFAULT_DEPTH, help="AM depth 0..1")
    ap.add_argument("--deviation", type=float, default=DEFAULT_DEVIATION, help="FM deviation, Hz")
    ap.add_argument("--workdir", default=os.path.join(ROOT, "build", "loopback"))
    ap.add_argument("--in", dest="in_path", help="audio file to (re)use")
    ap.add_argument("--out", dest="out_path", help="DAC dump to (re)use")
    ap.add_argument("--demod-only", action="store_true", help="skip the simulation, score existing files")
    ap.add_argument("--fs-dac", type=float, help="DAC sample rate override, Hz (Tiny Tapeout build: 50e6)")
    ap.add_argument("--audio-rate", type=float, help="baseband rate override, Hz")
    ap.add_argument("--max-delay", type=int,
                    help="alignment search window in audio samples (default: Hilbert taps)")
    ap.add_argument("--dc-shift", type=int, default=DC_SHIFT,
                    help=f"model the modulator's DC blocker (2^-shift leak, default {DC_SHIFT} from "
                         "platform.json; 0 = none, e.g. for the Tiny Tapeout build)")
    ap.add_argument("--allpass",
                    help="comma-separated first-order all-pass poles of the modulator's in-phase branch "
                         "(phasing-method SSB): the reference is passed through them so the SNR is not "
                         "charged for the network's own phase response")
    ap.add_argument("--save-wav", action="store_true",
                    help="write recovered audio as <workdir>/recovered.wav")
    args = ap.parse_args()
    global FS_DAC, FS_AUDIO, INTERP, TAPS
    if args.fs_dac:
        FS_DAC = int(args.fs_dac)
    if args.audio_rate:
        FS_AUDIO = int(args.audio_rate)
    INTERP = FS_DAC // FS_AUDIO
    if args.max_delay:
        TAPS = args.max_delay

    os.makedirs(args.workdir, exist_ok=True)
    in_path = args.in_path or os.path.join(args.workdir, "audio_in.bin")
    out_path = args.out_path or os.path.join(args.workdir, "dac_out.bin")
    ftw = int(round(args.carrier * (1 << 32) / FS_DAC)) & 0xFFFFFFFF
    fc = ftw * FS_DAC / (1 << 32)
    if args.mode == "am":
        modparam = min(0xFFFF, int(round(args.depth * 65536)))
    elif args.mode == "fm":
        modparam = int(round(args.deviation * (1 << 32) / FS_DAC / 256))
    else:
        modparam = 0

    tones = [(float(f), 1.0) for f in args.tones.split(",")] if args.tones else (None if args.wav else TONES)
    if args.demod_only:
        audio = np.fromfile(in_path, ">i2")           # already padded
    else:
        if not args.sim:
            sys.exit("--sim is required unless --demod-only")
        tones = [(float(f), 1.0) for f in args.tones.split(",")] if args.tones else None
        sweep = tuple(float(f) for f in args.sweep.split(",")) if args.sweep else None
        audio = (wav_audio(args.wav, args.seconds, args.level) if args.wav
                 else tone_audio(args.seconds, args.level, tones, sweep))
        print(f"{args.mode}: {len(audio)} samples ({len(audio) / FS_AUDIO * 1e3:.1f} ms) at {FS_AUDIO} S/s, "
              f"carrier {fc:.1f} Hz, modparam {modparam}")
        run_sim(args.vvp, args.sim, audio, in_path, out_path, ftw, WAVE_CODE[args.mode], modparam)

    if not args.demod_only:
        audio = padded(audio)
    codes = np.fromfile(out_path, ">u2").astype(np.float64)
    rf = codes - 8192
    clipped = int(np.sum((codes == 0) | (codes == 0x3FFF)))
    print(f"  {len(rf)} DAC samples ({len(rf) / FS_DAC * 1e3:.1f} ms), "
          f"range {int(rf.min())}..{int(rf.max())}, clipped {clipped}"
          + ("  <-- lower --level: the modulation envelope exceeds full scale" if clipped else ""))

    m = score(audio, rf, args.mode, fc, modparam,
              os.path.join(args.workdir, "recovered.wav") if args.save_wav else None,
              tones=tones if not (args.wav or args.sweep) else None,
              allpass=[float(a) for a in args.allpass.split(",")] if args.allpass else None,
              dc_shift=args.dc_shift)
    ok = True
    for k, v in m.items():
        thr = THRESHOLDS[args.mode].get(k)
        flag = ""
        if thr is not None:
            flag = "  OK" if v >= thr else "  FAIL"
            ok = ok and v >= thr
            flag += f" (>= {thr:g})"
        print(f"  {k:12s} {v:9.3f}{flag}")
    print("RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
