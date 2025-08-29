import os, re, glob, math, yaml
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

RUN_DIR = "gauss_2/2025-08-21_20-28-53_gaus"
META_YAML = os.path.join(RUN_DIR, "config_meta_data.yaml")
ODOM_CSV  = os.path.join(RUN_DIR, "input_data", "odom_data.csv")

# --- load params ---
with open(META_YAML, "r") as f:
    meta = yaml.safe_load(f) or {}
RADIUS = float(meta.get("RADIUS", 0.2))
OFFSET = float(meta.get("OFFSET", 0.4))

# --- odom path ---
odom = pd.read_csv(ODOM_CSV)
odom_x, odom_y = odom["odom_x"].to_numpy(), odom["odom_y"].to_numpy()

# --- all segment local goals (seg_0..seg_N) ---
def load_segment_goals(run_dir):
    paths = glob.glob(os.path.join(run_dir, "seg_*", "input_data", "local_goals.csv"))
    if not paths:
        # fallback: single-run file
        single = os.path.join(run_dir, "input_data", "local_goals.csv")
        if os.path.exists(single):
            df = pd.read_csv(single)
            df.columns = [c.lower() for c in df.columns]
            return df.rename(columns={"local_goals_x":"x","local_goals_y":"y"})[["x","y"]].dropna().to_numpy()
        raise FileNotFoundError("No seg_*/input_data/local_goals.csv or input_data/local_goals.csv found")

    # sort segs numerically: seg_0, seg_1, ...
    def seg_key(p):
        m = re.search(r"seg_(\d+)", p)
        return int(m.group(1)) if m else 0
    paths = sorted(paths, key=seg_key)

    frames = []
    for p in paths:
        df = pd.read_csv(p)
        df.columns = [c.lower() for c in df.columns]
        frames.append(df.rename(columns={"local_goals_x":"x","local_goals_y":"y"})[["x","y"]])
    return pd.concat(frames, ignore_index=True).dropna().to_numpy()

lg = load_segment_goals(RUN_DIR)

# --- rebuild obstacles like your Obstacle_Manager.obstacle_creation ---
def obstacles_from_goals(lg_xy, offset, radius):
    obs = []
    for i in range(len(lg_xy)-1):
        x1,y1 = lg_xy[i]; x2,y2 = lg_xy[i+1]
        dx,dy = x2-x1, y2-y1
        L = math.hypot(dx,dy)
        if L == 0: continue
        dx,dy = dx/L, dy/L
        px,py = -dy, dx                   # perpendicular
        mx,my = (x1+x2)/2, (y1+y2)/2      # midpoint
        obs.append((mx+offset*px, my+offset*py, radius))
        obs.append((mx-offset*px, my-offset*py, radius))
    return obs


import numpy as np

def kappa_three_point(p0, p1, p2):
    a = np.linalg.norm(p1 - p0)
    b = np.linalg.norm(p2 - p1)
    c = np.linalg.norm(p2 - p0)
    if a*b*c < 1e-9:
        return 0.0
    # signed 2*area via cross product of (p1-p0, p2-p0)
    cross = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
    k = abs(cross) / (a*b*c)              # unsigned curvature
    return math.copysign(k, cross)        # add sign by orientation

def segment_curvatures_from_goals(lg_xy):
    lg_xy = np.asarray(lg_xy, dtype=float)
    n = len(lg_xy)
    if n < 3:
        return [0.0]*(n-1)
    ks = []
    for i in range(n-1):
        p0 = lg_xy[max(0, i-1)]
        p1 = lg_xy[i]
        p2 = lg_xy[min(n-1, i+1)]
        ks.append(kappa_three_point(p0, p1, p2))
    return np.array(ks, dtype=float)

def smooth_kappa(kappa, win=21):
    """Simple centered moving average; ensures stable sign."""
    win = max(3, int(win) | 1)  # odd window
    pad = win//2
    x = np.pad(kappa, (pad,pad), mode='edge')
    c = np.cumsum(x)
    sm = (c[win:] - c[:-win]) / win
    return sm


def obstacles_from_goals_curved(
    lg_xy, base_offset, radius,
    robot_r=0.20, margin=0.05,
    beta=0.30, kappa_cap=0.4,
    kappa_straight=0.03, gain=2.0, s_max=0.50,
    widen_outside=0.6, max_out_mult=1.25
):
    import numpy as np, math
    def curv(lg):
        n = len(lg)
        if n < 3: return np.zeros(max(n-1, 0))
        heads = np.zeros(n)
        for i in range(n-1):
            dx, dy = lg[i+1] - lg[i]
            heads[i] = math.atan2(dy, dx)
        heads[-1] = heads[-2]
        heads = np.unwrap(heads)
        seg_len = np.linalg.norm(lg[1:] - lg[:-1], axis=1) + 1e-6
        turn = np.zeros(n)
        turn[1:-1] = heads[2:] - heads[:-2]
        turn[0] = heads[1] - heads[0]
        turn[-1] = heads[-1] - heads[-2]
        return (turn[:-1] / seg_len)

    MIN_OFF = radius + robot_r + margin
    kappa = curv(lg_xy)
    obs = []

    for i in range(len(lg_xy)-1):
        x1, y1 = lg_xy[i]; x2, y2 = lg_xy[i+1]
        dx, dy = x2-x1, y2-y1
        L = math.hypot(dx, dy)
        if L == 0: continue
        dx, dy = dx/L, dy/L
        px, py = -dy, dx
        mx, my = (x1+x2)/2, (y1+y2)/2

        k = abs(kappa[i])
        base = base_offset * (1.0 - beta * min(k, kappa_cap))
        base = max(base, MIN_OFF + 0.03)

        if k < kappa_straight:
            off_left, off_right = base, base
            typL, typR = "straight", "straight"
        else:
            skew = min(s_max, gain*k)
            off_in  = base * (1.0 - skew)
            off_out = base * (1.0 + widen_outside*skew)
            off_in  = max(off_in, MIN_OFF)
            off_out = min(off_out, base * max_out_mult)

            if kappa[i] > 0:      # left turn → inside=left
                off_left, off_right = off_in, off_out
                typL, typR = "inside", "outside"
            else:                  # right turn → inside=right
                off_left, off_right = off_out, off_in
                typL, typR = "outside", "inside"

        obs.append((mx + px*off_left,  my + py*off_left,  radius, typL))
        obs.append((mx - px*off_right, my - py*off_right, radius, typR))
    return obs

obs = obstacles_from_goals_curved(
    lg, OFFSET, RADIUS,
    robot_r=0.20, margin=0.05,
    beta=0.30, kappa_cap=0.4,
    kappa_straight=0.03,
    gain=2.0, s_max=0.50,
    widen_outside=0.6, max_out_mult=1.25
)
# --- build asymmetric obstacles (tuned values) ---
def corridor_polylines(lg_xy, base_offset):
    left_pts, right_pts = [], []
    for i in range(len(lg_xy)-1):
        x1,y1 = lg_xy[i]; x2,y2 = lg_xy[i+1]
        dx,dy = x2-x1, y2-y1
        L = math.hypot(dx,dy)
        if L == 0: 
            continue
        dx,dy = dx/L, dy/L
        px,py = -dy, dx
        mx,my = (x1+x2)/2, (y1+y2)/2

        # same offsets you used when creating obs (inside/offside)
        # replace the next 3 lines with your computed off_in/off_out & side signs:
        off_in, off_out = base_offset*0.7, base_offset*1.1
        inside_s, outside_s = +1, -1   # +1 means "left", -1 means "right"

        left_pts.append((mx + (+1)*off_in*px if inside_s==+1 else mx + (+1)*off_out*px,
                         my + (+1)*off_in*py if inside_s==+1 else my + (+1)*off_out*py))
        right_pts.append((mx + (-1)*off_in*px if inside_s==-1 else mx + (-1)*off_out*px,
                          my + (-1)*off_in*py if inside_s==-1 else my + (-1)*off_out*py))

    return np.array(left_pts), np.array(right_pts)

# --- plot with crossbars + colors so skew is obvious ---
plt.figure(figsize=(10,8))
ax = plt.gca(); ax.set_aspect('equal')
plt.plot(lg[:,0], lg[:,1], '-', linewidth=1.2, alpha=0.5, label='Path (from local goals)')
plt.scatter(lg[:,0], lg[:,1], s=8, alpha=0.7, label='Local goals')

left, right = corridor_polylines(lg, OFFSET)
plt.plot(left[:,0],  left[:,1],  '-', color='tab:orange', lw=2, label='Inside wall')
plt.plot(right[:,0], right[:,1], '-', color='tab:blue',   lw=2, label='Outside wall')
# ins  = [(x,y,r) for (x,y,r,t) in obs if t=="inside"]
# outs = [(x,y,r) for (x,y,r,t) in obs if t=="outside"]
# stra = [(x,y,r) for (x,y,r,t) in obs if t=="straight"]
#
# for x,y,r in ins:
#     ax.add_patch(Circle((x,y), r, fill=False, linewidth=1.0, alpha=0.9, edgecolor="orange"))
# for x,y,r in outs:
#     ax.add_patch(Circle((x,y), r, fill=False, linewidth=1.0, alpha=0.9, edgecolor="royalblue"))
# for x,y,r in stra:
#     ax.add_patch(Circle((x,y), r, fill=False, linewidth=1.0, alpha=0.4, edgecolor="gray"))
# # draw obstacles: inside vs outside color
# # for cx,cy,r,typ in obs:
# #     color = 'tab:orange' if typ=='inside' else 'tab:blue'
# #     ax.add_patch(Circle((cx,cy), r, fill=False, linewidth=0.9, alpha=0.95, edgecolor=color))
#
# # draw a short crossbar every N segments (connect inside↔outside)
# N = 12
# for i in range(0, len(obs)-1, 2*N):
#     (ix,iy,_,ti) = obs[i]
#     (ox,oy,_,to) = obs[i+1]
#     plt.plot([ix, ox], [iy, oy], '-', linewidth=1.2, alpha=0.9)

plt.title('Odom path with curvature-aware corridor (inside=orange, outside=blue)')
plt.xlabel('X'); plt.ylabel('Y'); plt.grid(True, alpha=0.3); plt.legend(); plt.tight_layout()
plt.show()
