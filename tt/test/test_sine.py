#!/usr/bin/env python3
"""Exhaustive check of sine_interp: run the Icarus sweep, compare all 65536
phases against round(8191*sin) and round(8191*cos). Max |error| must be <= 1 LSB.

    tt/test/test_sine.py          (needs iverilog on PATH)
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")


def main():
    out = os.path.join(HERE, "sim_build")
    os.makedirs(out, exist_ok=True)
    vvp = os.path.join(out, "sine.vvp")
    subprocess.run(["iverilog", "-g2005", "-o", vvp, os.path.join(SRC, "sine_interp.v"),
                    os.path.join(HERE, "tb_sine.v")], check=True)
    subprocess.run(["vvp", "-n", vvp], cwd=out, check=True, capture_output=True)
    rows = [tuple(int(v) for v in line.split()) for line in open(os.path.join(out, "sine_out.txt"))]
    sins = [s for _, s, _ in rows]
    coss = [c for _, _, c in rows]
    ideal_s = [round(8191 * math.sin(2 * math.pi * p / 65536)) for p in range(65536)]
    ideal_c = [round(8191 * math.cos(2 * math.pi * p / 65536)) for p in range(65536)]
    # find the pipeline lag: the output at row i corresponds to the phase applied at row i-lag
    best = None
    for lag in range(0, 40):
        err = sum(abs(sins[lag + p] - ideal_s[p]) for p in range(0, 65536, 97) if lag + p < len(sins))
        if best is None or err < best[1]:
            best = (lag, err)
    lag = best[0]
    worst_s = worst_c = 0
    hist = {}
    seen = 0
    for p in range(65536):
        if lag + p >= len(sins):
            break
        es = sins[lag + p] - ideal_s[p]
        ec = coss[lag + p] - ideal_c[p]
        worst_s = max(worst_s, abs(es))
        worst_c = max(worst_c, abs(ec))
        hist[es] = hist.get(es, 0) + 1
        seen += 1
    print(f"pipeline lag {lag} clocks; {seen} phases; max |sin err| {worst_s} LSB, "
          f"max |cos err| {worst_c} LSB; sin error histogram {dict(sorted(hist.items()))}")
    ok = seen == 65536 and worst_s <= 1 and worst_c <= 1
    print("RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
