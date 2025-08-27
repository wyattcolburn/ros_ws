import os, re, glob, math, yaml
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

RUN_DIR = "gauss_2/2025-08-21_19-38-54_gaus"
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

obs = obstacles_from_goals(lg, OFFSET, RADIUS)

# --- plot ---
plt.figure(figsize=(10,8))
ax = plt.gca(); ax.set_aspect('equal')
plt.plot(lg[:,0], lg[:,1], '-', linewidth=1.0, alpha=0.4, label='Path (map frame from LG)')
plt.scatter(lg[:,0], lg[:,1], s=10, alpha=0.6, label='Local goals')

for cx,cy,r in obs:
    ax.add_patch(Circle((cx,cy), r, fill=False, linewidth=1.0, alpha=0.9))

plt.title('Odom path with reconstructed obstacles')
plt.xlabel('X'); plt.ylabel('Y'); plt.grid(True, alpha=0.3); plt.legend(); plt.tight_layout()
plt.show()

print("segments used:",
      [os.path.basename(os.path.dirname(os.path.dirname(p))) for p in
       sorted(glob.glob(os.path.join(RUN_DIR, "seg_*", "input_data", "local_goals.csv")),
              key=lambda p: int(re.search(r"seg_(\d+)", p).group(1)))])
print("local goals loaded:", len(lg), "odom points:", len(odom_x))

