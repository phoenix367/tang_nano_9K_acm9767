#!/usr/bin/env python3
"""Plot the spectrum of a loopback DAC dump (see scripts/ssb_loopback.py).

Three panels: the RF spectrum around the carrier (wanted sideband, suppressed
sideband and carrier), the full 0..f_dac/2 band (spurs, CIC images), and the
recovered baseband against the input audio.

    scripts/plot_spectrum.py --workdir build/loopback/7mhz_usb_4k --carrier 7e6 --mode usb \\
        -o doc/images/ssb_spectrum_7mhz.png
"""

import argparse
import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ssb_loopback as L  # noqa: E402


def db(p):
    return 10 * np.log10(np.maximum(p, 1e-30))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workdir", required=True, help="directory with audio_in.bin / dac_out.bin")
    ap.add_argument("--carrier", type=float, default=L.DEFAULT_CARRIER)
    ap.add_argument("--mode", default="usb", choices=sorted(L.WAVE_CODE))
    ap.add_argument("--span", type=float, default=8e3, help="half-span of the close-up, Hz")
    ap.add_argument("-o", "--output", default=None)
    args = ap.parse_args()

    audio = np.fromfile(os.path.join(args.workdir, "audio_in.bin"), ">i2")
    codes = np.fromfile(os.path.join(args.workdir, "dac_out.bin"), ">u2").astype(np.float64)
    rf = codes - 8192
    ftw = int(round(args.carrier * (1 << 32) / L.FS_DAC)) & 0xFFFFFFFF
    fc = ftw * L.FS_DAC / (1 << 32)

    # steady-state part of the dump (skip the FIR/CIC fill and the flush)
    lo = (L.PAD + L.TAPS + 20) * L.INTERP
    hi = (len(audio) - L.PAD - 20) * L.INTERP
    seg = rf[lo:hi]
    w = np.blackman(len(seg))
    S = np.fft.rfft(seg * w) / (np.sum(w) / 2)          # amplitude-normalised
    f = np.fft.rfftfreq(len(seg), 1 / L.FS_DAC)
    p = np.abs(S) ** 2
    ref = p.max()

    # recovered baseband vs input (same demodulator as the loopback score)
    if args.mode in ("usb", "lsb"):
        got = L.demod_ssb(rf, fc, args.mode)
    elif args.mode == "am":
        got = L.demod_am(rf, fc)
    else:
        got = L.demod_fm(rf, fc)
    x = audio.astype(np.float64) / 32768
    ref_hi = L.upsample(x, L.INTERP)
    if args.mode in ("usb", "lsb"):
        ref_hi = L.analytic(ref_hi)
    lag, scale, _ = L.align_and_score(ref_hi, got, (L.TAPS + 20) * L.INTERP)
    dec = got[lag::L.INTERP][: len(x)]
    if np.iscomplexobj(dec):
        dec = (dec * np.exp(-1j * np.angle(scale))).real
    dec = dec / abs(scale)
    n = len(x)
    sl = slice(L.PAD + (n - 2 * L.PAD) // 5, L.PAD + 4 * (n - 2 * L.PAD) // 5)
    wa = np.blackman(sl.stop - sl.start)
    A_in = np.abs(np.fft.rfft(x[sl] * wa)) / (np.sum(wa) / 2)
    A_out = np.abs(np.fft.rfft(dec[sl] * wa)) / (np.sum(wa) / 2)
    fa = np.fft.rfftfreq(sl.stop - sl.start, 1 / L.FS_AUDIO)

    fig, axs = plt.subplots(3, 1, figsize=(11, 12))
    fig.suptitle(f"Simulated DAC output: {args.mode.upper()} on {fc / 1e6:.4f} MHz carrier, "
                 f"{L.FS_DAC / 1e6:.0f} MS/s, {len(seg) / L.FS_DAC * 1e3:.0f} ms Blackman window",
                 fontsize=12)

    m = (f >= fc - args.span) & (f <= fc + args.span)
    axs[0].plot((f[m] - fc) / 1e3, db(p[m] / ref), lw=0.8)
    axs[0].axvline(0, color="grey", lw=0.6, ls="--")
    axs[0].set_xlabel("offset from carrier [kHz]")
    axs[0].set_ylabel("dBc")
    axs[0].set_title("RF spectrum around the carrier")
    axs[0].set_ylim(-110, 5)
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(f / 1e6, db(p / ref), lw=0.5)
    axs[1].set_xlabel("frequency [MHz]")
    axs[1].set_ylabel("dBc")
    axs[1].set_title("RF spectrum, 0 .. f_dac/2")
    axs[1].set_ylim(-110, 5)
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(fa / 1e3, 20 * np.log10(np.maximum(A_in, 1e-9)), lw=0.9, label="input audio")
    axs[2].plot(fa / 1e3, 20 * np.log10(np.maximum(A_out, 1e-9)), lw=0.9, ls="--",
                label="recovered (demodulated)")
    axs[2].set_xlim(0, 8)
    axs[2].set_ylim(-110, 0)
    axs[2].set_xlabel("audio frequency [kHz]")
    axs[2].set_ylabel("dBFS")
    axs[2].set_title("Baseband: input vs recovered by the analytic demodulator")
    axs[2].legend(loc="upper right")
    axs[2].grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = args.output or os.path.join(args.workdir, "spectrum.png")
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
