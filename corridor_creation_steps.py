#!/usr/bin/env python3
import os, math, argparse, numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

def ensure_dir(p):
    os.makedirs(p, exist_ok=True); return p

def normalize(v):
    n = float(np.linalg.norm(v))
    if n < 1e-12: return np.array([0.0, 0.0], float), 0.0
    return v / n, n

def signed_curvature(p0, p1, p2):
    a, b = p1 - p0, p2 - p1
    _, la = normalize(a); _, lb = normalize(b)
    if la < 1e-9 or lb < 1e-9: return 0.0
    cross = a[0]*b[1] - a[1]*b[0]
    c = float(np.linalg.norm(p2 - p0))
    if la*lb*c < 1e-9: return 0.0
    s = (la + lb + c)/2.0
    area_sq = max(s*(s-la)*(s-lb)*(s-c), 0.0)
    if area_sq <= 0.0: return 0.0
    R = (la*lb*c)/(4.0*math.sqrt(area_sq))
    if R < 1e-9: return 0.0
    return math.copysign(1.0/R, cross)

def distance_point_to_segment(P, A, B):
    AP, AB = P - A, B - A
    ab2 = float(np.dot(AB, AB))
    if ab2 <= 1e-12: return float(np.linalg.norm(AP))
    t = max(0.0, min(1.0, float(np.dot(AP, AB) / ab2)))
    proj = A + t * AB
    return float(np.linalg.norm(P - proj))

def distance_point_to_polyline(P, poly):
    if len(poly) < 2: return float('inf')
    dmin = float('inf')
    for i in range(len(poly)-1):
        d = distance_point_to_segment(P, poly[i], poly[i+1])
        if d < dmin: dmin = d
    return dmin

def set_equal_aspect(ax, pts, pad_ratio=0.5):
    pts = np.array(pts, float)
    xmin, ymin = np.min(pts, axis=0); xmax, ymax = np.max(pts, axis=0)
    span = max(xmax - xmin, ymax - ymin, 1e-3)
    pad = pad_ratio * span
    ax.set_xlim(xmin - pad, xmax + pad); ax.set_ylim(ymin - pad, ymax + pad)
    ax.set_aspect('equal', adjustable='box')

def save_fig(path, dpi=170): plt.savefig(path, dpi=dpi, bbox_inches="tight"); plt.close()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--p0", type=float, nargs=2, required=True)
    ap.add_argument("--p1", type=float, nargs=2, required=True)
    ap.add_argument("--p2", type=float, nargs=2, required=True)
    ap.add_argument("--offset", type=float, default=0.4)
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--smax", type=float, default=0.6)
    ap.add_argument("--sgain", type=float, default=3.0)
    ap.add_argument("--min-inside", type=float, default=0.6)
    ap.add_argument("--max-outmul", type=float, default=1.6)
    ap.add_argument("--widen-out", type=float, default=1.0)
    ap.add_argument("--clear-buf", type=float, default=0.03)
    ap.add_argument("--clear-scale", type=float, default=1.10)
    ap.add_argument("--outdir", type=str, default="./corr_steps_out")
    ap.add_argument("--fps", type=int, default=2)
    ap.add_argument("--hold", type=int, default=10)
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--show_prev", action="store_true",
                    help="If set, also draw the previous segment P0→P1 (dashed). Default off.")
    args = ap.parse_args()

    outdir = ensure_dir(args.outdir)
    P0, P1, P2 = np.array(args.p0,float), np.array(args.p1,float), np.array(args.p2,float)
    path_poly = np.vstack([P0, P1, P2])

    d = P2 - P1; t, L = normalize(d)
    if L < 1e-12: raise SystemExit("Degenerate segment (P1==P2).")
    n = np.array([-t[1], t[0]], float); M = 0.5*(P1 + P2)

    kappa = signed_curvature(P0, P1, P2)
    s = args.smax * (1.0 - math.exp(-args.sgain * abs(kappa)))
    BASE = args.offset
    d_in  = max(args.min_inside * BASE, BASE * (1.0 - s))
    d_out = min(args.max_outmul * BASE, BASE * (1.0 + args.widen_out * s))

    inside_s  = +1.0 if kappa > 0.0 else -1.0
    outside_s = -inside_s

    C_in_pre, C_out_pre = M + inside_s*d_in*n, M + outside_s*d_out*n

    min_clear = max(args.clear_scale * args.radius, 1.20 * args.radius) + args.clear_buf
    dist_in  = distance_point_to_polyline(C_in_pre, path_poly)
    dist_out = distance_point_to_polyline(C_out_pre, path_poly)
    push_in  = max(0.0, min_clear - dist_in)
    push_out = max(0.0, min_clear - dist_out)
    C_in, C_out = C_in_pre + inside_s*push_in*n, C_out_pre + outside_s*push_out*n

    all_pts = [P1,P2,M,C_in,C_out,C_in_pre,C_out_pre,
               C_in + args.radius*n, C_in - args.radius*n,
               C_out + args.radius*n, C_out - args.radius*n]

    def draw_base(ax):
        # Always draw only the current segment P1→P2
        ax.plot([P1[0], P2[0]], [P1[1], P2[1]], linewidth=1)
        if args.show_prev:  # dashed previous segment, if requested
            ax.plot([P0[0], P1[0]], [P0[1], P1[1]], linestyle='--', linewidth=1, alpha=0.6)
        ax.scatter([P1[0], P2[0]], [P1[1], P2[1]])

    # Step 1
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.set_title("Step 1 — Segment (P1 → P2)"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_01_segment.png"))

    # Step 2
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.scatter([M[0]],[M[1]])
    ax.set_title("Step 2 — Midpoint M"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_02_midpoint.png"))

    # Step 3
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.scatter([M[0]],[M[1]])
    ax.quiver([M[0]],[M[1]],[t[0]],[t[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
    ax.quiver([M[0]],[M[1]],[n[0]],[n[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
    ax.set_title("Step 3 — Unit Tangent t and Normal n"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_03_vectors.png"))

    # Step 4
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.scatter([M[0]],[M[1]])
    off_in_end, off_out_end = M + inside_s*d_in*n, M + outside_s*d_out*n
    ax.plot([M[0], off_in_end[0]], [M[1], off_in_end[1]], linewidth=2)
    ax.plot([M[0], off_out_end[0]], [M[1], off_out_end[1]], linewidth=2)
    ax.text(off_in_end[0], off_in_end[1], f"d_in={d_in:.2f}", fontsize=9)
    ax.text(off_out_end[0], off_out_end[1], f"d_out={d_out:.2f}", fontsize=9)
    ax.set_title(f"Step 4 — Offsets (κ={kappa:.3f}, s={s:.2f})"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_04_offsets.png"))

    # Step 5
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.scatter([C_in_pre[0], C_out_pre[0]], [C_in_pre[1], C_out_pre[1]])
    ax.set_title("Step 5 — Center Placement (pre-clearance)"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_05_centers.png"))

    # Step 6
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    ax.scatter([C_in[0], C_out[0]],[C_in[1], C_out[1]])
    if push_in > 1e-6:
        ax.quiver([C_in_pre[0]], [C_in_pre[1]], [inside_s*push_in*n[0]], [inside_s*push_in*n[1]],
                  angles='xy', scale_units='xy', scale=1.0, width=0.005)
    if push_out > 1e-6:
        ax.quiver([C_out_pre[0]], [C_out_pre[1]], [outside_s*push_out*n[0]], [outside_s*push_out*n[1]],
                  angles='xy', scale_units='xy', scale=1.0, width=0.005)
    ax.set_title("Step 6 — Clearance Enforcement"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_06_clearance.png"))

    # Step 7
    def circle_points(center, r, npts=200):
        tt = np.linspace(0, 2*np.pi, npts); return center[0] + r*np.cos(tt), center[1] + r*np.sin(tt)
    plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts); draw_base(ax)
    X1, Y1 = circle_points(C_in,  args.radius); X2, Y2 = circle_points(C_out, args.radius)
    ax.plot(X1, Y1, linewidth=2); ax.plot(X2, Y2, linewidth=2)
    ax.scatter([C_in[0], C_out[0]],[C_in[1], C_out[1]])
    ax.set_title("Step 7 — Final Obstacles (radius)"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    save_fig(os.path.join(outdir, "step_07_obstacles.png"))

    # Animation
    steps, total_frames = 7, 7*args.hold
    fig = plt.figure(); ax = plt.gca(); set_equal_aspect(ax, all_pts)
    titles = ["Step 1 — Segment (P1→P2)","Step 2 — Midpoint M","Step 3 — Unit Tangent t and Normal n",
              f"Step 4 — Offsets (κ={kappa:.3f}, s={s:.2f})","Step 5 — Centers (pre-clearance)",
              "Step 6 — Clearance Enforcement","Step 7 — Final Obstacles (radius)"]

    def draw_step(ax, step):
        ax.clear(); set_equal_aspect(ax, all_pts); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        # Only P1→P2; optionally dashed P0→P1
        ax.plot([P1[0], P2[0]], [P1[1], P2[1]], linewidth=1)
        if args.show_prev:
            ax.plot([P0[0], P1[0]], [P0[1], P1[1]], linestyle='--', linewidth=1, alpha=0.6)
        ax.scatter([P1[0], P2[0]], [P1[1], P2[1]])
        if step >= 2: ax.scatter([M[0]],[M[1]])
        if step >= 3:
            ax.quiver([M[0]],[M[1]],[t[0]],[t[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
            ax.quiver([M[0]],[M[1]],[n[0]],[n[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
        if step >= 4:
            off_in_end, off_out_end = M + inside_s*d_in*n, M + outside_s*d_out*n
            ax.plot([M[0], off_in_end[0]], [M[1], off_in_end[1]], linewidth=2)
            ax.plot([M[0], off_out_end[0]], [M[1], off_out_end[1]], linewidth=2)
        if step >= 5: ax.scatter([C_in_pre[0], C_out_pre[0]],[C_in_pre[1], C_out_pre[1]])
        if step >= 6:
            ax.scatter([C_in[0], C_out[0]],[C_in[1], C_out[1]])
            if push_in > 1e-6:
                ax.quiver([C_in_pre[0]],[C_in_pre[1]],[inside_s*push_in*n[0]],[inside_s*push_in*n[1]],
                          angles='xy', scale_units='xy', scale=1.0, width=0.005)
            if push_out > 1e-6:
                ax.quiver([C_out_pre[0]],[C_out_pre[1]],[outside_s*push_out*n[0]],[outside_s*push_out*n[1]],
                          angles='xy', scale_units='xy', scale=1.0, width=0.005)
        if step >= 7:
            X1, Y1 = circle_points(C_in,  args.radius); X2, Y2 = circle_points(C_out, args.radius)
            ax.plot(X1, Y1, linewidth=2); ax.plot(X2, Y2, linewidth=2)

    def init(): draw_step(ax,1); ax.set_title(titles[0]); return []
    def animate(frame): step = min(7, frame // args.hold + 1); draw_step(ax, step); ax.set_title(titles[step-1]); return []

    ani = animation.FuncAnimation(fig, animate, frames=total_frames, init_func=init, blit=False)
    mp4, gif, saved = os.path.join(outdir,"steps.mp4"), os.path.join(outdir,"steps.gif"), False
    try:
        if not args.gif:
            Writer = animation.writers['ffmpeg']; writer = Writer(fps=args.fps, bitrate=3200)
            ani.save(mp4, writer=writer); print(f"[saved] {mp4}"); saved = True
    except Exception as e:
        print(f"[warn] ffmpeg not available or failed: {e}")
    if (args.gif or not saved):
        try: ani.save(gif, writer="pillow", fps=args.fps); print(f"[saved] {gif}")
        except Exception as e: print(f"[error] could not save GIF: {e}")

    with open(os.path.join(outdir, "metrics.txt"), "w") as f:
        f.write(f"P0: {P0.tolist()}\nP1: {P1.tolist()}\nP2: {P2.tolist()}\n")
        f.write(f"L: {L:.3f}\nκ: {kappa:.5f}\ns: {s:.4f}\n")
        f.write(f"BASE: {BASE:.3f}\nd_in: {d_in:.3f}\nd_out: {d_out:.3f}\n")
        f.write(f"push_in: {push_in:.4f}\npush_out: {push_out:.4f}\n")
        f.write(f"C_in: {C_in.tolist()}\nC_out: {C_out.tolist()}\n")

    print("[done] Figures & animation written to:", os.path.abspath(outdir))

if __name__ == "__main__":
    main()
