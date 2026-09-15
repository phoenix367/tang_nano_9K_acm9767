#!/usr/bin/env python3
"""Time-domain view of a loopback DAC dump (see scripts/ssb_loopback.py).

Panels: the whole run (DAC codes with the modulation envelope and the input
audio), a zoom on the envelope beating, and a zoom down to individual 54 MS/s
samples on the carrier.

    scripts/plot_dac_samples.py --workdir build/loopback/7mhz_usb_4k --carrier 7e6 -o doc/images/x.png
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--carrier", type=float, default=L.DEFAULT_CARRIER)
    ap.add_argument("--zoom-ms", type=float, default=None, help="start of the zooms, ms (default: mid-run)")
    ap.add_argument("-o", "--output", default=None)
    args = ap.parse_args()

    audio = np.fromfile(os.path.join(args.workdir, "audio_in.bin"), ">i2").astype(np.float64)
    codes = np.fromfile(os.path.join(args.workdir, "dac_out.bin"), ">u2").astype(np.float64)
    t = np.arange(len(codes)) / L.FS_DAC * 1e3          # ms
    ta = np.arange(len(audio)) / L.FS_AUDIO * 1e3
    env = np.abs(L.analytic(codes - 8192))              # modulation envelope in DAC LSB

    fc = int(round(args.carrier * (1 << 32) / L.FS_DAC)) * L.FS_DAC / (1 << 32)
    z0 = args.zoom_ms if args.zoom_ms is not None else t[-1] / 2
    fig, axs = plt.subplots(3, 1, figsize=(11, 12))
    fig.suptitle(f"DAC output samples: SSB (USB) on {fc / 1e6:.4f} MHz, {L.FS_DAC / 1e6:.0f} MS/s, "
                 f"14-bit codes (mid-scale 8192)", fontsize=12)

    # whole run: decimate the code trace for drawing, keep min/max per bin so the shape survives
    step = max(1, len(codes) // 4000)
    n = len(codes) // step * step
    blk = codes[:n].reshape(-1, step)
    tb = t[:n:step]
    axs[0].fill_between(tb, blk.min(axis=1), blk.max(axis=1), color="tab:blue", alpha=0.35, lw=0,
                        label="DAC codes (min/max per bin)")
    axs[0].plot(tb, 8192 + env[:n:step], color="tab:red", lw=1.0, label="envelope |analytic|")
    axs[0].plot(tb, 8192 - env[:n:step], color="tab:red", lw=1.0)
    axs[0].plot(ta + 4.221, 8192 + audio / 32768 * 8192, color="k", lw=0.6, alpha=0.7,
                label="input audio (scaled, shifted by the 4.2 ms chain delay)")
    axs[0].set_xlabel("time [ms]")
    axs[0].set_ylabel("DAC code")
    axs[0].set_ylim(0, 16383)
    axs[0].set_title("Whole run: the SSB envelope is not the audio waveform")
    axs[0].legend(loc="upper right", fontsize=8)
    axs[0].grid(True, alpha=0.3)

    # 300 us: envelope beating between the tones, carrier cycles merge into a band
    i0 = int(z0 * 1e-3 * L.FS_DAC)
    i1 = i0 + int(300e-6 * L.FS_DAC)
    axs[1].plot(t[i0:i1] * 1e3 - z0 * 1e3, codes[i0:i1], lw=0.4, color="tab:blue")
    axs[1].plot(t[i0:i1] * 1e3 - z0 * 1e3, 8192 + env[i0:i1], color="tab:red", lw=1.0)
    axs[1].plot(t[i0:i1] * 1e3 - z0 * 1e3, 8192 - env[i0:i1], color="tab:red", lw=1.0)
    axs[1].set_xlabel(f"time [us] from {z0:.1f} ms")
    axs[1].set_ylabel("DAC code")
    axs[1].set_ylim(0, 16383)
    axs[1].set_title("300 us: carrier cycles under the beating envelope")
    axs[1].grid(True, alpha=0.3)

    # 1 us: individual samples
    j1 = i0 + int(1.0e-6 * L.FS_DAC)
    tt = (t[i0:j1] - t[i0]) * 1e3
    axs[2].step(tt, codes[i0:j1], where="post", lw=0.8, color="tab:blue",
                label="DAC holds each code for 1/54 MHz")
    axs[2].plot(tt, codes[i0:j1], "o", ms=4, color="tab:blue")
    tf = np.linspace(0, tt[-1], 2000)
    ph = np.interp(tf, tt, np.unwrap(np.angle(L.analytic(codes[i0:j1] - 8192))))
    axs[2].plot(tf, 8192 + np.interp(tf, tt, env[i0:j1]) * np.cos(ph), color="tab:red", lw=0.8, alpha=0.7,
                label="reconstructed waveform")
    axs[2].set_xlabel(f"time [us] from {z0:.1f} ms")
    axs[2].set_ylabel("DAC code")
    axs[2].set_title(f"1 us: {L.FS_DAC / fc:.1f} samples per carrier cycle")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[2].grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = args.output or os.path.join(args.workdir, "dac_samples.png")
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
