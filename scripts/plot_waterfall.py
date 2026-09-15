#!/usr/bin/env python3
"""Waterfall (spectrogram) of a loopback DAC dump around the carrier.

The 54 MS/s DAC codes are converted to the complex baseband around the carrier
(analytic signal x e^-j2*pi*fc*t), decimated, and short-time Fourier
transformed: positive offsets are the upper sideband, negative the lower one,
0 Hz the carrier -- so an SSB signal shows its audio content on one side only
and the suppressed sideband/carrier as the dark floor on the other. A second
panel shows the input audio's spectrogram for reference.

    scripts/plot_waterfall.py --workdir build/loopback/7mhz_sweep --carrier 7e6 -o doc/images/x.png
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


def stft_db(x: np.ndarray, fs: float, win_s: float, hop_s: float, complex_in: bool):
    n = int(win_s * fs)
    hop = max(1, int(hop_s * fs))
    w = np.hanning(n)
    frames = range(0, len(x) - n, hop)
    spec = []
    for i in frames:
        seg = x[i:i + n] * w
        S = np.fft.fft(seg) if complex_in else np.fft.rfft(seg)
        spec.append(np.abs(S) / (np.sum(w) / 2))
    spec = np.array(spec)
    f = np.fft.fftfreq(n, 1 / fs) if complex_in else np.fft.rfftfreq(n, 1 / fs)
    if complex_in:
        order = np.argsort(f)
        f, spec = f[order], spec[:, order]
    t = (np.array(list(frames)) + n / 2) / fs
    return t, f, 20 * np.log10(np.maximum(spec, 1e-9))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--carrier", type=float, default=L.DEFAULT_CARRIER)
    ap.add_argument("--span", type=float, default=5e3, help="+- offset shown around the carrier, Hz")
    ap.add_argument("--win-ms", type=float, default=8.0, help="STFT window, ms")
    ap.add_argument("--hop-ms", type=float, default=0.5, help="STFT hop, ms")
    ap.add_argument("--floor", type=float, default=-90.0, help="colour floor, dBc")
    ap.add_argument("-o", "--output", default=None)
    args = ap.parse_args()

    audio = np.fromfile(os.path.join(args.workdir, "audio_in.bin"), ">i2").astype(np.float64) / 32768
    codes = np.fromfile(os.path.join(args.workdir, "dac_out.bin"), ">u2").astype(np.float64)
    rf = codes - 8192
    fc = int(round(args.carrier * (1 << 32) / L.FS_DAC)) * L.FS_DAC / (1 << 32)

    # complex baseband around the carrier, decimated to a rate that holds the span
    n = np.arange(len(rf))
    bb = L.analytic(rf) * np.exp(-2j * np.pi * fc * n / L.FS_DAC)
    dec = int(L.FS_DAC // (4 * args.span))           # e.g. 54e6 / 20e3 -> 2700
    fs_bb = L.FS_DAC / dec
    bb = L.lowpass(bb, L.FS_DAC, fs_bb / 2 * 0.9)[::dec]
    t_rf, f_rf, S_rf = stft_db(bb, fs_bb, args.win_ms * 1e-3, args.hop_ms * 1e-3, True)
    t_au, f_au, S_au = stft_db(audio, L.FS_AUDIO, args.win_ms * 1e-3, args.hop_ms * 1e-3, False)
    ref = S_rf.max()
    S_rf -= ref
    S_au -= S_au.max()

    fig, axs = plt.subplots(1, 2, figsize=(13, 8), gridspec_kw={"width_ratios": [2, 1]})
    fig.suptitle(f"SSB (USB) waterfall: DAC output around the {fc / 1e6:.4f} MHz carrier "
                 f"({args.win_ms:.0f} ms Hann windows, {args.hop_ms:.1f} ms hop)", fontsize=12)

    m = np.abs(f_rf) <= args.span
    im = axs[0].pcolormesh(f_rf[m] / 1e3, t_rf * 1e3, S_rf[:, m], shading="nearest",
                           cmap="magma", vmin=args.floor, vmax=0)
    axs[0].axvline(0, color="w", lw=0.6, ls="--", alpha=0.6)
    axs[0].set_xlabel("offset from carrier [kHz]   (< 0: lower sideband, > 0: upper sideband)")
    axs[0].set_ylabel("time [ms]")
    axs[0].set_title("RF around the carrier (dBc)")
    axs[0].invert_yaxis()
    fig.colorbar(im, ax=axs[0], label="dBc")

    m2 = f_au <= args.span
    im2 = axs[1].pcolormesh(f_au[m2] / 1e3, t_au * 1e3, S_au[:, m2], shading="nearest",
                            cmap="magma", vmin=args.floor, vmax=0)
    axs[1].set_xlabel("audio frequency [kHz]")
    axs[1].set_title("input audio (dB rel. peak)")
    axs[1].invert_yaxis()
    fig.colorbar(im2, ax=axs[1], label="dB")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = args.output or os.path.join(args.workdir, "waterfall.png")
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
