 #!/usr/bin/env python3
import os, argparse, math, numpy as np, matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon

def ensure_dir(p): os.makedirs(p, exist_ok=True); return p
def side_length(A,B): return float(np.linalg.norm(B-A))
def triangle_area_heron(a,b,c):
    s=0.5*(a+b+c); area_sq=max(s*(s-a)*(s-b)*(s-c),0.0); return math.sqrt(area_sq)
def circumcircle_center_radius(P0,P1,P2):
    x0,y0=P0; x1,y1=P1; x2,y2=P2
    d=2.0*(x0*(y1-y2)+x1*(y2-y0)+x2*(y0-y1))
    if abs(d)<1e-12: return np.array([np.nan,np.nan],float), float('inf')
    ux=((x0**2+y0**2)*(y1-y2)+(x1**2+y1**2)*(y2-y0)+(x2**2+y2**2)*(y0-y1))/d
    uy=((x0**2+y0**2)*(x2-x1)+(x1**2+y1**2)*(x0-x2)+(x2**2+y2**2)*(x1-x0))/d
    C=np.array([ux,uy],float); R=float(np.linalg.norm(C-P0)); return C,R
def set_equal_aspect(ax, pts, pad_ratio=0.15):
    pts=np.array(pts,float); xmin,ymin=np.nanmin(pts,axis=0); xmax,ymax=np.nanmax(pts,axis=0)
    span=max(xmax-xmin, ymax-ymin, 1e-3); pad=pad_ratio*span
    ax.set_xlim(xmin-pad, xmax+pad); ax.set_ylim(ymin-pad, ymax+pad); ax.set_aspect('equal', adjustable='box')
def circle_points(center,r,n=300):
    t=np.linspace(0,2*np.pi,n); return center[0]+r*np.cos(t), center[1]+r*np.sin(t)
def annotate_points(ax,P0,P1,P2):
    ax.scatter([P0[0],P1[0],P2[0]],[P0[1],P1[1],P2[1]]); ax.text(P0[0],P0[1]," P0"); ax.text(P1[0],P1[1]," P1"); ax.text(P2[0],P2[1]," P2")

def unit(v):
    n=float(np.linalg.norm(v))
    return (v/n if n>1e-12 else np.array([0.0,0.0],float)), n

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--p0",type=float,nargs=2,required=True)
    ap.add_argument("--p1",type=float,nargs=2,required=True)
    ap.add_argument("--p2",type=float,nargs=2,required=True)
    ap.add_argument("--outdir",type=str,default="./kappa_out")
    ap.add_argument("--fps",type=int,default=2)
    ap.add_argument("--hold",type=int,default=10)
    ap.add_argument("--gif",action="store_true")
    args=ap.parse_args()

    outdir=ensure_dir(args.outdir)
    P0=np.array(args.p0,float); P1=np.array(args.p1,float); P2=np.array(args.p2,float)

    # Side lengths & area
    a=side_length(P1,P0); b=side_length(P2,P1); c=side_length(P2,P0)
    A=triangle_area_heron(a,b,c)

    # Circumcircle + curvature
    C,R=circumcircle_center_radius(P0,P1,P2)
    kappa=0.0 if not np.isfinite(R) or R<=1e-12 else 1.0/R

    # Tangent at P1 (toward P2) and normal at P1
    t_vec, L = unit(P2-P1)
    n_left = np.array([-t_vec[1], t_vec[0]], float)  # CCW rotate (left normal)
    # Turn sign via cross product of chords (P1-P0) x (P2-P1)
    cross = (P1-P0)[0]*(P2-P1)[1] - (P1-P0)[1]*(P2-P1)[0]
    turn = "left" if cross>0 else ("right" if cross<0 else "straight")
    # Direction from P1 to circumcenter
    to_center = C - P1 if np.isfinite(R) else np.array([0.0,0.0],float)

    # Bounds
    pts_for_bounds=[P0,P1,P2]
    if np.isfinite(R):
        pts_for_bounds+=[C+np.array([R,0.0]),C-np.array([R,0.0]),C+np.array([0.0,R]),C-np.array([0.0,R])]

    # Step 1 — points & chords
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    ax.plot([P0[0],P1[0]],[P0[1],P1[1]], linewidth=1)
    ax.plot([P1[0],P2[0]],[P1[1],P2[1]], linewidth=1)
    ax.plot([P0[0],P2[0]],[P0[1],P2[1]], linewidth=1)
    annotate_points(ax,P0,P1,P2)
    ax.set_title("Step 1 — Points and Chords (a, b, c)"); ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_01_points.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Step 2 — triangle with side labels
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    ax.plot([P0[0],P1[0],P2[0],P0[0]],[P0[1],P1[1],P2[1],P0[1]], linewidth=2)
    annotate_points(ax,P0,P1,P2)
    ax.text(*(0.5*(P0+P1))," a"); ax.text(*(0.5*(P1+P2))," b"); ax.text(*(0.5*(P0+P2))," c")
    ax.set_title("Step 2 — Triangle with side labels"); ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_02_triangle.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Step 3 — area
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    poly=Polygon(np.vstack([P0,P1,P2]), closed=True, alpha=0.3); ax.add_patch(poly)
    annotate_points(ax,P0,P1,P2)
    ax.set_title(f"Step 3 — Triangle Area  A = {A:.3f}"); ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_03_area.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Step 4 — circumcircle
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds); annotate_points(ax,P0,P1,P2)
    if np.isfinite(R):
        Xc,Yc=circle_points(C,R); ax.plot(Xc,Yc, linewidth=2); ax.scatter([C[0]],[C[1]]); ax.text(C[0],C[1],"  C")
        ax.plot([C[0],P1[0]],[C[1],P1[1]], linewidth=1); ax.text(*(0.5*(C+P1))," R")
    ax.set_title(f"Step 4 — Circumcircle (R = {R:.3f})"); ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_04_circumcircle.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Step 5 — curvature formula
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    annotate_points(ax,P0,P1,P2)
    ax.plot([P0[0],P1[0],P2[0],P0[0]],[P0[1],P1[1],P2[1],P0[1]], linewidth=1)
    if np.isfinite(R): Xc,Yc=circle_points(C,R); ax.plot(Xc,Yc, linewidth=1)
    # ax.text(
    #     P1[0], P1[1],
    #     f"\nκ = 4A/(a·b·c) = 1/R\nA={A:.3f}, a={a:.3f}, b={b:.3f}, c={c:.3f}\nR={R:.3f}, κ={kappa:.4f}"
    # )
    ax.set_title("Step 5 — Curvature κ from Triangle"); ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_05_kappa.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Step 6 — tangent/normal and direction to center
    plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    annotate_points(ax,P0,P1,P2)
    # draw t and n at P1
    ax.quiver([P1[0]],[P1[1]],[t_vec[0]],[t_vec[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
    ax.text(P1[0]+0.02, P1[1]+0.02, " t (forward)")
    ax.quiver([P1[0]],[P1[1]],[n_left[0]],[n_left[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
    ax.text(P1[0]+0.02*n_left[0], P1[1]+0.02*n_left[1], " n (left)")
    # show ray from P1 toward circumcenter and a label with turn
    if np.isfinite(R):
        ax.plot([P1[0], C[0]], [P1[1], C[1]], linestyle='--', linewidth=1)
        ax.text(*(0.5*(P1+C)), f" to center ({turn})")
    ax.set_title("Step 6 — t & n at P1; center direction indicates turn")
    ax.set_xlabel("x"); ax.set_ylabel("y")
    plt.savefig(os.path.join(outdir,"step_06_vectors_to_center.png"), dpi=170, bbox_inches="tight"); plt.close()

    # Animation (6 steps)
    steps=6; total_frames=steps*args.hold
    fig=plt.figure(); ax=plt.gca(); set_equal_aspect(ax, pts_for_bounds)
    titles=["Step 1 — Points and Chords","Step 2 — Triangle","Step 3 — Area A",
            f"Step 4 — Circumcircle (R≈{R:.3f})","Step 5 — κ = 4A/(abc) = 1/R",
            "Step 6 — t & n at P1; direction to center"]
    def draw_step(ax, step):
        ax.clear(); set_equal_aspect(ax, pts_for_bounds)
        if step>=1:
            ax.plot([P0[0],P1[0]],[P0[1],P1[1]], linewidth=1)
            ax.plot([P1[0],P2[0]],[P1[1],P2[1]], linewidth=1)
            ax.plot([P0[0],P2[0]],[P0[1],P2[1]], linewidth=1)
            annotate_points(ax,P0,P1,P2)
        if step>=2:
            ax.plot([P0[0],P1[0],P2[0],P0[0]],[P0[1],P1[1],P2[1],P0[1]], linewidth=2)
        if step>=3:
            poly=Polygon(np.vstack([P0,P1,P2]), closed=True, alpha=0.25); ax.add_patch(poly)
        if step>=4 and np.isfinite(R):
            Xc,Yc=circle_points(C,R); ax.plot(Xc,Yc, linewidth=2); ax.scatter([C[0]],[C[1]]); ax.text(C[0],C[1],"  C")
        if step>=5:
            ax.text(P1[0], P1[1], f"κ = 4A/(a·b·c) = 1/R\nκ≈{kappa:.4f}")
        if step>=6:
            ax.quiver([P1[0]],[P1[1]],[t_vec[0]],[t_vec[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
            ax.quiver([P1[0]],[P1[1]],[n_left[0]],[n_left[1]], angles='xy', scale_units='xy', scale=1.0, width=0.006)
            if np.isfinite(R):
                ax.plot([P1[0], C[0]], [P1[1], C[1]], linestyle='--', linewidth=1)
                ax.text(*(0.5*(P1+C)), f"{turn} turn")
        ax.set_xlabel("x"); ax.set_ylabel("y")

    def init(): draw_step(ax,1); ax.set_title(titles[0]); return []
    def animate(frame): step=min(steps, frame//args.hold + 1); draw_step(ax, step); ax.set_title(titles[step-1]); return []

    ani=animation.FuncAnimation(fig, animate, frames=total_frames, init_func=init, blit=False)
    mp4=os.path.join(outdir,"steps.mp4"); gif=os.path.join(outdir,"steps.gif"); saved=False
    try:
        if not args.gif:
            Writer=animation.writers['ffmpeg']; writer=Writer(fps=args.fps, bitrate=3200)
            ani.save(mp4, writer=writer); print(f"[saved] {mp4}"); saved=True
    except Exception as e:
        print(f"[warn] ffmpeg not available or failed: {e}")
    if (args.gif or not saved):
        try: ani.save(gif, writer="pillow", fps=args.fps); print(f"[saved] {gif}")
        except Exception as e: print(f"[error] could not save GIF: {e}")

    with open(os.path.join(outdir,"metrics.txt"),"w") as f:
        f.write(f"P0: {P0.tolist()}\\nP1: {P1.tolist()}\\nP2: {P2.tolist()}\\n")
        f.write(f"a: {a:.6f}\\nb: {b:.6f}\\nc: {c:.6f}\\nA: {A:.6f}\\nR: {R:.6f}\\n")
        f.write(f"kappa: {kappa:.8f}\\nturn: {turn}\\n")

    print("[done] Figures & animation written to:", os.path.abspath(outdir))

if __name__=="__main__":
    main()   
