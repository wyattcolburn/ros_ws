#!/usr/bin/env python3
"""
Skew & Corridor Offsets Plot Generator

Generates slide-ready plots that explain how curvature magnitude |kappa|
maps to skew s(|kappa|), and how skew controls inside/outside offsets
(d_in, d_out) relative to a baseline corridor half-width.

Outputs (default ./skew_out/):
  - s_vs_kappa.png
  - offsets_vs_kappa.png
  - cross_sections.png (three panels in one column: straight / medium / tight)
  - values.csv (table of |kappa|, s, d_in, d_out)

Usage example:
  python skew_plot_generator.py \
    --smax 0.6 --sgain 3.0 \
    --base 0.40 --amin 0.60 --amax 1.60 --beta 1.00 \
    --kmax 2.0 --steps 400 \
    --outdir ./skew_out
"""

import os
import argparse
import numpy as np
import math
import matplotlib.pyplot as plt

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def s_of_kappa_abs(kabs, smax, sgain):
    # s(|kappa|) = smax * (1 - exp(-sgain * |kappa|))
    return smax * (1.0 - np.exp(-sgain * np.asarray(kabs)))

def offsets_from_s(s, base, amin, amax, beta):
    d_in = np.maximum(amin * base, base * (1.0 - s))
    d_out = np.minimum(amax * base, base * (1.0 + beta * s))
    return d_in, d_out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--smax", type=float, default=0.6, help="s_max (max skew)")
    ap.add_argument("--sgain", type=float, default=3.0, help="gain gamma")
    ap.add_argument("--base", type=float, default=0.40, help="BASE half-width (OFFSET)")
    ap.add_argument("--amin", type=float, default=0.60, help="min inside ratio wrt BASE")
    ap.add_argument("--amax", type=float, default=1.60, help="max outside multiplier wrt BASE")
    ap.add_argument("--beta", type=float, default=1.0, help="outside widening factor")
    ap.add_argument("--kmax", type=float, default=2.0, help="max |kappa| to plot")
    ap.add_argument("--steps", type=int, default=400, help="number of samples in [0, kmax]")
    ap.add_argument("--outdir", type=str, default="./skew_out")
    args = ap.parse_args()

    outdir = ensure_dir(args.outdir)

    # Domain for |kappa|
    k = np.linspace(0.0, args.kmax, args.steps)
    s = s_of_kappa_abs(k, args.smax, args.sgain)
    d_in, d_out = offsets_from_s(s, args.base, args.amin, args.amax, args.beta)

    # Figure 1: s vs |kappa|
    plt.figure()
    plt.plot(k, s, linewidth=2)
    plt.xlabel("|kappa|")
    plt.ylabel("skew s(|kappa|)")
    plt.title(f"s(|kappa|) = s_max(1 - exp(-gamma|k|)),  s_max={args.smax}, gamma={args.sgain}")
    plt.grid(True, linestyle=":", linewidth=0.8)
    plt.savefig(os.path.join(outdir, "s_vs_kappa.png"), dpi=170, bbox_inches="tight")
    plt.close()

    # Figure 2: offsets vs |kappa|
    plt.figure()
    plt.plot(k, d_in, linewidth=2, label="d_in (inside)")
    plt.plot(k, d_out, linewidth=2, label="d_out (outside)")
    # baseline
    plt.axhline(args.base, linewidth=1, linestyle="--")
    plt.xlabel("|kappa|")
    plt.ylabel("offset distance [m]")
    plt.title(f"Corridor offsets vs |kappa|  (BASE={args.base}, amin={args.amin}, amax={args.amax}, beta={args.beta})")
    plt.legend()
    plt.grid(True, linestyle=":", linewidth=0.8)
    plt.savefig(os.path.join(outdir, "offsets_vs_kappa.png"), dpi=170, bbox_inches="tight")
    plt.close()

    # Figure 3: cross-sections at three |kappa| values (0, mid, max)
    # We will draw three separate small panels vertically (three separate figures to stay 'one chart per figure').
    sample_labels = ["straight (|k|=0)", "medium", "tight"]
    k_samples = [0.0, args.kmax*0.35, args.kmax]
    for idx, kval in enumerate(k_samples, start=1):
        s_val = s_of_kappa_abs([kval], args.smax, args.sgain)[0]
        di, do = offsets_from_s(np.array([s_val]), args.base, args.amin, args.amax, args.beta)
        di, do = float(di[0]), float(do[0])

        # Draw a local frame at origin; corridor walls at ±d along normal
        # We'll draw a horizontal path line segment (tangent along +x), normal along +y/-y
        x = np.linspace(-0.6, 0.6, 50)
        plt.figure()
        # path centerline
        plt.plot(x, np.zeros_like(x), linewidth=2)
        # inside and outside walls
        plt.axhline(+di, linewidth=2)
        plt.axhline(-do, linewidth=2)
        # midpoint marker
        plt.scatter([0.0], [0.0])
        plt.title(f"Cross-section: {sample_labels[idx-1]}\n s={s_val:.2f}, d_in={di:.2f}, d_out={do:.2f}")
        plt.xlabel("tangent direction")
        plt.ylabel("normal direction")
        plt.xlim(-0.7, 0.7)
        plt.ylim(-max(do, di)*1.5, max(do, di)*1.5)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True, linestyle=":", linewidth=0.8)
        plt.savefig(os.path.join(outdir, f"cross_section_{idx}.png"), dpi=170, bbox_inches="tight")
        plt.close()

    # CSV table for reference
    table_path = os.path.join(outdir, "values.csv")
    with open(table_path, "w") as f:
        f.write("abs_kappa,s,d_in,d_out\n")
        for kk, ss, di, do in zip(k, s, d_in, d_out):
            f.write(f"{kk:.6f},{ss:.6f},{di:.6f},{do:.6f}\n")

    print("[done] Wrote figures and CSV to", os.path.abspath(outdir))

if __name__ == "__main__":
    main()
