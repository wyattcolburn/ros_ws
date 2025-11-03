#!/usr/bin/env python3
"""
Symmetric Obstacle Creation — Five-Step Reveal

Sequence:
  1) Show only current & next points
  2) Add midpoint
  3) Add unit direction vector (at midpoint)
  4) Add symmetric offset points (±OFFSET along perpendicular)
  5) Add obstacle circles (RADIUS)

Outputs (default ./sym_steps_out/):
  - step_01_points.png
  - step_02_midpoint.png
  - step_03_unit_dir.png
  - step_04_offset_points.png
  - step_05_radius.png
  - steps.mp4 (or steps.gif with --gif)
  - metrics.txt

Usage:
  python obstacle_creation_sym_steps.py \
    --current 0.0 0.0 \
    --next 2.0 1.0 \
    --offset 0.5 \
    --radius 0.25 \
    --outdir ./sym_steps_out \
    --fps 2 --hold 12
"""

import os
import argparse
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def normalize(v):
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.array([0.0, 0.0], dtype=float), 0.0
    return v / n, n

def circle_points(center, r, n=200):
    t = np.linspace(0, 2*np.pi, n)
    return center[0] + r*np.cos(t), center[1] + r*np.sin(t)

def compute_geometry(p_curr, p_next, offset, radius):
    d = p_next - p_curr
    u_dir, length = normalize(d)
    u_perp = np.array([-u_dir[1], u_dir[0]], dtype=float)
    mid = 0.5*(p_curr + p_next)
    p_off1 = mid + offset*u_perp
    p_off2 = mid - offset*u_perp
    return {
        "p_curr": p_curr, "p_next": p_next, "mid": mid,
        "u_dir": u_dir, "u_perp": u_perp, "length": length,
        "p_off1": p_off1, "p_off2": p_off2, "radius": radius
    }

def set_equal_aspect(ax, pts, pad_ratio=0.2):
    pts = np.array(pts, dtype=float)
    xmin, ymin = np.min(pts, axis=0)
    xmax, ymax = np.max(pts, axis=0)
    dx, dy = xmax - xmin, ymax - ymin
    span = max(dx, dy, 1e-3)
    pad = pad_ratio * span
    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)
    ax.set_aspect('equal', adjustable='box')

def save_fig(path, dpi=160):
    plt.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close()

def draw_step(ax, G, step):
    # common baseline: endpoints + segment
    ax.plot([G["p_curr"][0], G["p_next"][0]], [G["p_curr"][1], G["p_next"][1]], linewidth=1)
    ax.scatter([G["p_curr"][0], G["p_next"][0]], [G["p_curr"][1], G["p_next"][1]])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    if step >= 2:
        ax.scatter([G["mid"][0]], [G["mid"][1]])

    if step >= 3:
        ax.quiver([G["mid"][0]], [G["mid"][1]], [G["u_dir"][0]], [G["u_dir"][1]],
                  angles='xy', scale_units='xy', scale=1.0, width=0.005)

    if step >= 4:
        ax.scatter([G["p_off1"][0], G["p_off2"][0]], [G["p_off1"][1], G["p_off2"][1]])

    if step >= 5:
        X1, Y1 = circle_points(G["p_off1"], G["radius"])
        X2, Y2 = circle_points(G["p_off2"], G["radius"])
        ax.plot(X1, Y1, linewidth=2)
        ax.plot(X2, Y2, linewidth=2)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--current", type=float, nargs=2, required=True)
    ap.add_argument("--next", type=float, nargs=2, required=True)
    ap.add_argument("--offset", type=float, default=0.5)
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--outdir", type=str, default="./sym_steps_out")
    ap.add_argument("--fps", type=int, default=2, help="Frames per second for the step animation")
    ap.add_argument("--hold", type=int, default=12, help="Frames to hold per step (animation length = 5 * hold / fps sec)")
    ap.add_argument("--gif", action="store_true")
    args = ap.parse_args()

    outdir = ensure_dir(args.outdir)

    p_curr = np.array(args.current, dtype=float)
    p_next = np.array(args.next, dtype=float)
    G = compute_geometry(p_curr, p_next, args.offset, args.radius)

    # For consistent axes across all steps
    # include both obstacle circles' extents
    pad_pts = [G["p_curr"], G["p_next"], G["mid"], G["p_off1"], G["p_off2"]]

    # Step 1: points only
    plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    draw_step(ax, G, step=1)
    ax.set_title("Step 1 — Current & Next Points")
    save_fig(os.path.join(outdir, "step_01_points.png"))

    # Step 2: + midpoint
    plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    draw_step(ax, G, step=2)
    ax.set_title("Step 2 — Add Midpoint")
    save_fig(os.path.join(outdir, "step_02_midpoint.png"))

    # Step 3: + unit direction
    plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    draw_step(ax, G, step=3)
    ax.set_title("Step 3 — Add Unit Direction Vector")
    save_fig(os.path.join(outdir, "step_03_unit_dir.png"))

    # Step 4: + offset points
    plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    draw_step(ax, G, step=4)
    ax.set_title("Step 4 — Add Offset Points (±OFFSET)")
    save_fig(os.path.join(outdir, "step_04_offset_points.png"))

    # Step 5: + circles
    plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    draw_step(ax, G, step=5)
    ax.set_title("Step 5 — Add Obstacle Radius Circles")
    save_fig(os.path.join(outdir, "step_05_radius.png"))

    # Simple 5-step animation (each step held 'hold' frames)
    fig = plt.figure()
    ax = plt.gca()
    set_equal_aspect(ax, pad_pts)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = ax.set_title("Steps 1→5")

    total_frames = 5 * args.hold

    def init():
        ax.clear()
        set_equal_aspect(ax, pad_pts)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        title = ax.set_title("Steps 1→5")
        return []

    def animate(frame):
        ax.clear()
        set_equal_aspect(ax, pad_pts)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        step = min(5, frame // args.hold + 1)
        draw_step(ax, G, step=step)
        titles = [
            "Step 1 — Current & Next Points",
            "Step 2 — Add Midpoint",
            "Step 3 — Add Unit Direction Vector",
            "Step 4 — Add Offset Points (±OFFSET)",
            "Step 5 — Add Obstacle Radius Circles",
        ]
        ax.set_title(titles[step-1])
        return []

    ani = animation.FuncAnimation(fig, animate, frames=total_frames, init_func=init, blit=False)

    mp4 = os.path.join(outdir, "steps.mp4")
    gif = os.path.join(outdir, "steps.gif")
    saved = False
    if not args.gif:
        try:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=args.fps, bitrate=3200)
            ani.save(mp4, writer=writer)
            print(f"[saved] {mp4}")
            saved = True
        except Exception as e:
            print(f"[warn] ffmpeg not available or failed: {e}")
    if (args.gif or not saved):
        try:
            ani.save(gif, writer="pillow", fps=args.fps)
            print(f"[saved] {gif}")
        except Exception as e:
            print(f"[error] could not save GIF: {e}")

    # metrics for reference
    with open(os.path.join(outdir, "metrics.txt"), "w") as f:
        f.write(f"current: {G['p_curr'].tolist()}\n")
        f.write(f"next: {G['p_next'].tolist()}\n")
        f.write(f"midpoint: {G['mid'].tolist()}\n")
        f.write(f"unit_dir: {G['u_dir'].tolist()} (segment length = {G['length']:.3f} m)\n")
        f.write(f"OFFSET: {float(args.offset):.3f} m\n")
        f.write(f"RADIUS: {float(args.radius):.3f} m\n")
        f.write(f"off1: {G['p_off1'].tolist()}\n")
        f.write(f"off2: {G['p_off2'].tolist()}\n")

    print("[done] Wrote figures to", os.path.abspath(outdir))

if __name__ == "__main__":
    main()
