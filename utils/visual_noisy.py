
# visual_path_obs_poc.py
import os
import re
import glob
import math
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ------------- CONFIG -------------
RUN_DIR   = "gauss_2/2025-08-21_19-38-54_gaus"
META_YAML = os.path.join(RUN_DIR, "config_meta_data.yaml")
ODOM_CSV  = os.path.join(RUN_DIR, "input_data", "odom_data.csv")
SEED      = 42  # change for different randomized worlds

# Corridor realism knobs (tweak freely)
PARAMS = dict(
    offset_range=(0.85, 1.30),  # multiply OFFSET per pair (width changes)
    radius_range=(0.90, 1.15),  # multiply RADIUS per obstacle
    jitter_std=0.05,            # Gaussian noise (m) to obstacle centers
    along_std=0.04,             # slide along the segment (m)
    drop_prob=0.08,             # randomly drop an obstacle (gaps)
    single_side_prob=0.12,      # keep only one side sometimes
    pinch_prob=0.06,            # locally shrink corridor width
    pinch_scale=(0.45, 0.75),   # scale OFFSETS during a pinch
    clutter_prob=0.04,          # spawn a free obstacle near corridor
    clutter_rad=1.2             # max distance (m) from midpoint for clutter
)
# ----------------------------------

def load_params(meta_yaml):
    with open(meta_yaml, "r") as f:
        meta = yaml.safe_load(f) or {}
    radius = float(meta.get("RADIUS", 0.2))
    offset = float(meta.get("OFFSET", 0.4))
    return radius, offset

def load_segment_goals(run_dir):
    """Concatenate seg_*/input_data/local_goals.csv in numeric order; fallback to input_data/local_goals.csv."""
    paths = glob.glob(os.path.join(run_dir, "seg_*", "input_data", "local_goals.csv"))

    if not paths:
        single = os.path.join(run_dir, "input_data", "local_goals.csv")
        if not os.path.exists(single):
            raise FileNotFoundError(
                "No seg_*/input_data/local_goals.csv or input_data/local_goals.csv found")
        df = pd.read_csv(single)
        df.columns = [c.lower() for c in df.columns]
        lg = df.rename(columns={"local_goals_x": "x", "local_goals_y": "y"})[["x", "y"]].dropna()
        return lg.to_numpy(), []

    # sort segs numerically: seg_0, seg_1, ...
    def seg_key(p):
        m = re.search(r"seg_(\d+)", p)
        return int(m.group(1)) if m else 0

    paths = sorted(paths, key=seg_key)
    frames = []
    for p in paths:
        df = pd.read_csv(p)
        df.columns = [c.lower() for c in df.columns]
        frames.append(df.rename(columns={"local_goals_x": "x", "local_goals_y": "y"})[["x", "y"]])

    lg = pd.concat(frames, ignore_index=True).dropna()
    seg_names = [os.path.basename(os.path.dirname(os.path.dirname(p))) for p in paths]
    return lg.to_numpy(), seg_names

def obstacles_from_goals_augmented(
    lg_xy, offset, radius, rng,
    offset_range=(0.8, 1.25), radius_range=(0.85, 1.20),
    jitter_std=0.06, along_std=0.05,
    drop_prob=0.05, single_side_prob=0.10,
    pinch_prob=0.05, pinch_scale=(0.4, 0.7),
    clutter_prob=0.03, clutter_rad=1.5
):
    """
    Build obstacles with controlled randomness to avoid overly-even 'tubes'.
    Returns list of (cx, cy, r).
    """
    obs = []
    for i in range(len(lg_xy) - 1):
        x1, y1 = lg_xy[i]
        x2, y2 = lg_xy[i + 1]
        dx, dy = x2 - x1, y2 - y1
        L = math.hypot(dx, dy)
        if L == 0:
            continue
        dx, dy = dx / L, dy / L
        px, py = -dy, dx                    # unit perpendicular
        tx, ty = dx, dy                     # unit tangent
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2

        # width modulation & occasional pinch
        width_scale = rng.uniform(*offset_range)
        off = offset * width_scale
        if rng.random() < pinch_prob:
            off *= rng.uniform(*pinch_scale)

        # choose one or both sides
        keep_both = rng.random() > single_side_prob
        sides = [+1, -1] if keep_both else [rng.choice([+1, -1])]

        for s in sides:
            cx = mx + s * off * px
            cy = my + s * off * py

            # jitter + slide along the path
            cx += rng.normal(0, jitter_std) + rng.normal(0, along_std) * tx
            cy += rng.normal(0, jitter_std) + rng.normal(0, along_std) * ty

            # per-obstacle radius variation
            r = radius * rng.uniform(*radius_range)

            # random dropout (gap)
            if rng.random() < drop_prob:
                continue

            obs.append((cx, cy, r))

        # occasional clutter near (not on) the corridor
        if rng.random() < clutter_prob:
            ang = rng.uniform(0, 2 * math.pi)
            rad = rng.uniform(0.2, clutter_rad)
            cx = mx + rad * math.cos(ang)
            cy = my + rad * math.sin(ang)
            r  = radius * rng.uniform(0.7, 1.1)
            obs.append((cx, cy, r))

    return obs

def main():
    # params & data
    RADIUS, OFFSET = load_params(META_YAML)
    lg, seg_names = load_segment_goals(RUN_DIR)
    rng = np.random.default_rng(SEED)

    # make imperfect corridor
    obs = obstacles_from_goals_augmented(
        lg, OFFSET, RADIUS, rng=rng, **PARAMS
    )

    # Plot in MAP frame (LG path); plotting odom path is optional and frame-mismatched
    plt.figure(figsize=(10, 8))
    ax = plt.gca()
    ax.set_aspect('equal')

    # path (from LGs) + obstacles
    plt.plot(lg[:, 0], lg[:, 1], '-', linewidth=1.0, alpha=0.4,
             label='Path (map frame from LG)')
    for cx, cy, r in obs:
        ax.add_patch(Circle((cx, cy), r, fill=False, linewidth=1.0, alpha=0.9))

    plt.title('Path with randomized safety corridor (PoC)')
    plt.xlabel('X'); plt.ylabel('Y')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    print("segments used:", seg_names if seg_names else "[single local_goals.csv]")
    print("local goals loaded:", len(lg))

if __name__ == "__main__":
    main()
