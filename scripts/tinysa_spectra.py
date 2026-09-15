#!/usr/bin/env python3
"""Capture spectra of the running DAC output with a tinySA (Ultra) and plot them.

The analyser talks a simple line protocol over its USB CDC port. For each
requested mode this script sets channel 1 (and starts a voice/tone stream for
the modulated modes), takes a max-hold over N wide sweeps (harmonics) and N
narrow sweeps (the sidebands around the carrier), and saves everything to an
.npz. `--plot` renders the two figures used in the README from that file.

  scripts/tinysa_spectra.py --tinysa /dev/ttyACM0 --wav voice.wav -o build/spectra.npz
  scripts/tinysa_spectra.py --plot build/spectra.npz --out-dir doc/images

Keep the analyser input below +6 dBm (the module gives ~+2 dBm at full
amplitude with its gain trimmer turned down; use a pad otherwise).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CTL = os.path.join(ROOT, "scripts", "acm9767_ctl.py")
MODES = ["sine", "ssb", "am", "fm"]
TITLES = {"sine": "sine, full amplitude", "ssb": "SSB (USB) voice, level 1.0",
          "am": "AM voice, depth 0.8, level 1.0", "fm": "NFM voice, 5 kHz deviation, level 1.0"}


# ---- tinySA ----
class TinySA:
    def __init__(self, port: str):
        import serial
        self.p = serial.Serial(port, 115200, timeout=3)

    def cmd(self, c: str) -> str:
        self.p.reset_input_buffer()
        self.p.write((c + "\r").encode())
        out, t0 = b"", time.time()
        while time.time() - t0 < 40:
            chunk = self.p.read(65536)
            if chunk:
                out += chunk
            if out.endswith(b"ch> "):
                break
        return out.decode(errors="replace")

    def scan(self, f0: float, f1: float, points: int = 450):
        """One sweep; the firmware occasionally prints an unparsable value, skip it."""
        f, v = [], []
        for line in self.cmd(f"scan {int(f0)} {int(f1)} {points} 3").splitlines():
            parts = line.split()
            if len(parts) < 2 or not parts[0][0].isdigit():
                continue
            try:
                a, b = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            f.append(a)
            v.append(b)
        return np.array(f), np.array(v)

    def max_hold(self, f0: float, f1: float, rbw_khz: float, sweeps: int):
        self.cmd(f"rbw {rbw_khz:g}")
        mx = None
        for _ in range(sweeps):
            f, v = self.scan(f0, f1)
            mx = v.copy() if mx is None or len(v) != len(mx) else np.maximum(mx, v)
        return f, mx


# ---- board ----
def ctl(port: str, *args: str) -> None:
    r = subprocess.run([sys.executable, CTL, "-p", port, *args], capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr.strip()[-300:])


def start_stream(port: str, mode: str, wav: str | None, tone: float):
    src = ["--wav", wav] if wav else ["--tone", str(tone), "--seconds", "3600"]
    cmd = [sys.executable, CTL, "-p", port, "stream", "--carrier", "7.1e6", "--mode",
           {"ssb": "usb", "am": "am", "fm": "fm"}[mode], "--level", "1.0", *src]
    # loop a WAV so the capture sees the whole recording
    if wav:
        script = "while true; do " + " ".join(f"'{c}'" for c in cmd) + " || exit 1; done"
        return subprocess.Popen(["bash", "-c", script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_stream(proc) -> None:
    if proc is None:
        return
    subprocess.run(["pkill", "-TERM", "-P", str(proc.pid)], capture_output=True)
    proc.terminate()
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        proc.kill()
    time.sleep(1)


def capture(args) -> dict:
    sa = TinySA(args.tinysa)
    sa.cmd("attenuate auto")
    out = {}
    for mode in args.modes:
        proc = None
        if mode == "sine":
            ctl(args.port, "set", "1", "--freq", str(args.carrier), "--wave", "sine", "--amp", "1.0", "--offset", "0")
            ctl(args.port, "enable", "1")
        else:
            ctl(args.port, "set", "1", "--freq", str(args.carrier), "--wave",
                {"ssb": "ssb-usb", "am": "am", "fm": "fm"}[mode], "--amp", "1.0", "--offset", "0")
            proc = start_stream(args.port, mode, args.wav, args.tone)
        time.sleep(6)
        fw, mw = sa.max_hold(4e6, 30e6, 300, args.sweeps)
        fn, mn = sa.max_hold(args.carrier - 15e3, args.carrier + 15e3, 1, args.sweeps)
        stop_stream(proc)
        out[f"{mode}_fw"], out[f"{mode}_mw"], out[f"{mode}_fn"], out[f"{mode}_mn"] = fw, mw, fn, mn
        fund = mw[(fw > args.carrier - 2e5) & (fw < args.carrier + 2e5)].max()
        harm = [mw[(fw > k * args.carrier - 2e5) & (fw < k * args.carrier + 2e5)].max() for k in (2, 3, 4)]
        print(f"{mode:4}: carrier {fund:6.1f} dBm, H2..H4 {harm[0]:6.1f} {harm[1]:6.1f} {harm[2]:6.1f} dBm,"
              f" floor {np.median(mw):6.1f} dBm", flush=True)
    ctl(args.port, "disable", "1")
    return out


# ---- plots ----
INK, INK2, GRID, SERIES = "#0b0b0b", "#52514e", "#e6e5e1", "#2a78d6"


def plot(npz: str, out_dir: str, carrier: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.load(npz)
    modes = [m for m in MODES if f"{m}_fw" in d]
    views = (("wide", "w", "MHz", "measured_spectra_4_30mhz.png"),
             ("narrow", "n", "kHz from the carrier", "measured_spectra_carrier_30khz.png"))
    for span, key, xlabel, fname in views:
        fig, axs = plt.subplots(len(modes), 1, figsize=(9, 2.6 * len(modes)), sharex=True,
                                facecolor="#fcfcfb")
        for ax, m in zip(np.atleast_1d(axs), modes):
            f, v = d[f"{m}_f{key}"], d[f"{m}_m{key}"]
            x = f / 1e6 if span == "wide" else (f - carrier) / 1e3
            ax.set_facecolor("#fcfcfb")
            ax.plot(x, v, color=SERIES, lw=1.2)
            ax.set_title(TITLES[m], loc="left", fontsize=10, color=INK)
            ax.set_ylabel("dBm", color=INK2, fontsize=9)
            ax.grid(color=GRID, lw=0.6)
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
            for s in ("left", "bottom"):
                ax.spines[s].set_color(GRID)
            ax.tick_params(colors=INK2, labelsize=8)
            if span == "wide":
                ax.set_ylim(-75, 10)
                for k, name in ((1, "carrier"), (2, "H2"), (3, "H3"), (4, "H4")):
                    sel = (f > k * carrier - 2e5) & (f < k * carrier + 2e5)
                    if sel.any():
                        pk = v[sel].max()
                        ax.annotate(f"{name} {pk:.0f} dBm", (k * carrier / 1e6, pk),
                                    textcoords="offset points", xytext=(6, 4), fontsize=8, color=INK2)
            else:
                ax.set_ylim(-80, 10)
        np.atleast_1d(axs)[-1].set_xlabel(xlabel, color=INK2, fontsize=9)
        rbw = "RBW 300 kHz" if span == "wide" else "RBW 1 kHz"
        fig.suptitle(f"ACM9767 output on a tinySA Ultra, {carrier / 1e6:.1f} MHz carrier, max-hold ({rbw})",
                     x=0.02, ha="left", fontsize=11, color=INK)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        path = os.path.join(out_dir, fname)
        fig.savefig(path, dpi=110)
        print("wrote", path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyGowin", help="board serial port")
    ap.add_argument("--tinysa", default="/dev/ttyACM0", help="tinySA serial port")
    ap.add_argument("--carrier", type=float, default=7.1e6)
    ap.add_argument("--modes", nargs="+", default=MODES, choices=MODES)
    ap.add_argument("--wav", help="voice WAV for the modulated modes (default: a 1 kHz tone)")
    ap.add_argument("--tone", type=float, default=1000.0)
    ap.add_argument("--sweeps", type=int, default=8, help="sweeps per max-hold (default 8)")
    ap.add_argument("-o", "--output", default="build/spectra.npz")
    ap.add_argument("--plot", metavar="NPZ", help="skip the capture, plot this file")
    ap.add_argument("--out-dir", default="doc/images")
    args = ap.parse_args(argv)
    if args.plot:
        plot(args.plot, args.out_dir, args.carrier)
        return 0
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    np.savez(args.output, **capture(args))
    print("saved", args.output)
    plot(args.output, args.out_dir, args.carrier)
    return 0


if __name__ == "__main__":
    sys.exit(main())
