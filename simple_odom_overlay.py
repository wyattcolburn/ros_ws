
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Overlay raw odometry from multiple rosbag2 runs (NO TF).
Also draws one reference plan from /plan_barn_odom if present.

- Plots all /odom traces exactly as recorded (raw frame).
- Picks one /plan_barn_odom once (first bag that has it) and draws it dashed.
- Optional map backdrop (will NOT align without TF; purely visual).
"""

import argparse, glob, os
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image
import yaml

# ROS 2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# ------------------------- Data structures -------------------------

@dataclass
class Series:
    xs: List[float]
    ys: List[float]
    ts: Optional[List[float]] = None  # seconds (kept for completeness)

# ------------------------- Map loading (optional) -------------------------

def load_occ_grid(yaml_path: str, emphasize: bool = False):
    """Load map_server YAML+PGM. Return (image[0..1], extent[xmin,xmax,ymin,ymax], res)."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    pgm_path = meta["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    res = float(meta["resolution"])
    ox, oy, _ = meta.get("origin", [0.0, 0.0, 0.0])
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))

    img = np.array(Image.open(pgm_path)).astype(np.float32) / 255.0
    if negate == 1:
        img = 1.0 - img
    img = np.flipud(img)  # origin='lower' for imshow

    if emphasize:
        img = np.clip((img - free_th) / max(1e-6, (occ_th - free_th)), 0, 1)

    h, w = img.shape[:2]
    extent = [ox, ox + w * res, oy, oy + h * res]
    return img, extent, res

# ------------------------- Bag reading -------------------------

def _open_reader(bag_path: str, filter_topics: Optional[List[str]] = None) -> rosbag2_py.SequentialReader:
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    r = rosbag2_py.SequentialReader()
    r.open(storage, conv)
    if filter_topics:
        try:
            from rosbag2_py import StorageFilter
            r.set_filter(StorageFilter(topics=filter_topics))
        except Exception:
            pass
    return r

def _topic_types(reader) -> Dict[str, str]:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}

def _read_xy_from_odom(msg):  # nav_msgs/Odometry
    p = msg.pose.pose.position
    return float(p.x), float(p.y)

def _read_xy_from_pose(msg):  # geometry_msgs/PoseStamped
    p = msg.pose.position
    return float(p.x), float(p.y)

def _extract_series_for_topics(bag_path: str, odom_topic: str, plan_topic: str) -> Tuple[Series, Series]:
    """Return (odom Series with timestamps, plan Series without timestamps)."""
    r = _open_reader(bag_path, [odom_topic, plan_topic])
    types = _topic_types(r)

    des: Dict[str, Tuple[type, Callable]] = {}
    if odom_topic in types:
        des[odom_topic] = (get_message(types[odom_topic]), _read_xy_from_odom)
    if plan_topic in types:
        tname = types[plan_topic]
        if tname.endswith("/Path"):
            from nav_msgs.msg import Path  # noqa
            def _from_path(msg):
                return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
            des[plan_topic] = (get_message(tname), _from_path)
        elif tname.endswith("/PoseStamped"):
            des[plan_topic] = (get_message(tname), _read_xy_from_pose)
        else:
            raise RuntimeError(f"Unsupported plan type: {tname}")

    od = Series([], [], [])
    pl = Series([], [], None)
    needed = {t for t in (odom_topic, plan_topic) if t in des}

    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic not in needed:
            continue
        mtype, fn = des[topic]
        msg = deserialize_message(data, mtype)
        if topic == odom_topic:
            x, y = fn(msg)
            od.xs.append(x); od.ys.append(y); od.ts.append(t_ns / 1e9)
        else:
            out = fn(msg)
            if isinstance(out, list):
                for x, y in out:
                    pl.xs.append(x); pl.ys.append(y)
            else:
                x, y = out
                pl.xs.append(x); pl.ys.append(y)

    return od, pl

# ------------------------- Discovery -------------------------

def discover_bag_dirs(pattern: str, recursive: bool) -> List[str]:
    dirs = [d for d in sorted(glob.glob(pattern)) if os.path.isdir(d)]
    if recursive:
        seeds = dirs or ([pattern] if os.path.isdir(pattern) else [])
        for seed in seeds:
            for root, _sub, files in os.walk(seed):
                if "metadata.yaml" in files:
                    dirs.append(root)
    if not dirs and os.path.isdir(pattern):
        if os.path.exists(os.path.join(pattern, "metadata.yaml")):
            dirs = [pattern]
    out, seen = [], set()
    for d in dirs:
        if d in seen:
            continue
        if os.path.exists(os.path.join(d, "metadata.yaml")):
            seen.add(d); out.append(d)
    return sorted(out)

# ------------------------- Main -------------------------

def main():
    ap = argparse.ArgumentParser("Overlay RAW /odom + one /plan_barn_odom across rosbag2 runs (NO TF).")
    ap.add_argument("--bags", required=True, help="Glob or dir of bag folders (containing metadata.yaml).")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--odom", default="/turtle/odom")
    ap.add_argument("--plan", default="/turtle/plan",
                    help="Reference plan topic (default: /plan_barn_odom).")
    ap.add_argument("--out", default="overlay.png")
    ap.add_argument("--title", default="")
    ap.add_argument("--max-bags", type=int, default=9999)
    ap.add_argument("--downsample", type=int, default=1)

    # Map (optional backdrop; will not align without TF)
    ap.add_argument("--map", help="Map YAML (map_server).")
    ap.add_argument("--map-alpha", type=float, default=0.25)
    ap.add_argument("--emphasize-occupied", action="store_true")
    ap.add_argument("--map-vmin", type=float, default=0.0)
    ap.add_argument("--map-vmax", type=float, default=1.0)

    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--goal-radius", type=float, default=0.5, help="Goal tolerance radius (m).")
    ap.add_argument("--drop-first-plan-point", action="store_true",
                    help="Drop first plan pose if it precedes spawn.")

    args = ap.parse_args()

    bag_dirs = discover_bag_dirs(args.bags, args.recursive)[: args.max_bags]
    if not bag_dirs:
        raise SystemExit(f"No bag directories for: {args.bags}")

    # ----- Figure (map optional) -----
    plt.figure(figsize=(7.8, 6.6), dpi=args.dpi)
    ax = plt.gca()
    map_extent = None
    if args.map:
        try:
            grid, extent, res = load_occ_grid(args.map, emphasize=args.emphasize_occupied)
            # Half-cell shift for pixel centers (optional; harmless as backdrop)
            extent = [extent[0] - 0.5*res,
                      extent[1] - 0.5*res,
                      extent[2] - 0.5*res,
                      extent[3] - 0.5*res]
            ax.imshow(grid, extent=extent, cmap="gray", origin="lower",
                      alpha=args.map_alpha, interpolation="nearest",
                      vmin=args.map_vmin, vmax=args.map_vmax, zorder=0)
            map_extent = extent
        except Exception as e:
            print(f"[WARN] Could not render map '{args.map}': {e}")

    # ----- Choose one reference plan from /plan_barn_odom (first bag that has it) -----
    cached_plan: Optional[Series] = None
    for bag in bag_dirs:
        _, plan = _extract_series_for_topics(bag, args.odom, args.plan)
        if plan.xs:
            cached_plan = plan
            break
    if cached_plan and args.drop_first_plan_point and len(cached_plan.xs) > 1:
        cached_plan = Series(cached_plan.xs[1:], cached_plan.ys[1:])
    if cached_plan:
        print(f"[PLAN] Using {args.plan} with {len(cached_plan.xs)} poses as reference.")
    else:
        print(f"[PLAN] No data found on {args.plan}; proceeding without a plan.")

    # ----- Plot + collect bounds from REAL data only -----
    xs_all, ys_all = [], []   # <--- NEW: we’ll build limits from these

    # Plot all RAW odom traces
    for bag in bag_dirs:
        label = os.path.basename(bag) or bag.rstrip("/").split("/")[-1]
        try:
            odom, _ = _extract_series_for_topics(bag, args.odom, args.plan)
        except Exception as e:
            print(f"[WARN] Skipping {label}: {e}")
            continue

        if args.downsample > 1:
            odom = Series(odom.xs[::args.downsample], odom.ys[::args.downsample],
                          odom.ts[::args.downsample] if odom.ts else None)

        if odom.xs:
            ax.plot(odom.xs, odom.ys, linewidth=1.0, alpha=0.9, zorder=2)
            xs_all.extend(odom.xs); ys_all.extend(odom.ys)
        else:
            print(f"[INFO] {label}: topic '{args.odom}' not found or empty.")

    # Plot the single reference plan once (raw, same frame as /odom)
    if cached_plan and cached_plan.xs:
        ax.plot(cached_plan.xs, cached_plan.ys,
                linestyle="--", linewidth=2.0, color="red",
                alpha=0.9, zorder=6, label=args.plan)
        xs_all.extend(cached_plan.xs); ys_all.extend(cached_plan.ys)

        # start / goal markers + tolerance
        ax.plot(cached_plan.xs[0],  cached_plan.ys[0],  marker="o", markersize=6,
                color="lime", markeredgecolor="black", zorder=8, label="start")
        ax.plot(cached_plan.xs[-1], cached_plan.ys[-1], marker="o", markersize=6,
                color="cyan", markeredgecolor="black", zorder=8, label="goal")
        gx, gy = cached_plan.xs[-1], cached_plan.ys[-1]
        ax.add_patch(Circle((gx, gy), args.goal_radius,
                            fill=False, linestyle='--', linewidth=1.2,
                            edgecolor='cyan', alpha=0.6, zorder=5, label="goal tol"))

    # ----- Axes & tight limits (NEW) -----
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")

    if (cached_plan and cached_plan.xs) or xs_all:
        ax.legend(fontsize=7, loc="upper left", ncol=1, framealpha=0.75)

    # Tighten view to data with ±0.5 m padding; union with map if present
    if xs_all and ys_all:
        print("setting limits")
        pad = 0.5  # meters
        xmin, xmax = min(odom.xs), max(odom.xs)
        ymin, ymax = min(odom.ys), max(odom.ys)
        if map_extent:
            xmin = min(xmin, map_extent[0]); xmax = max(xmax, map_extent[1])
            ymin = min(ymin, map_extent[2]); ymax = max(ymax, map_extent[3])

        print(f"limits for ax are {xmin, ymin} and {xmax, ymax}")
        ax.set_xlim(xmin - pad, xmax + pad)
        ax.set_ylim(ymin - pad, ymax + pad)
    elif map_extent:
        ax.set_xlim(map_extent[0], map_extent[1])
        ax.set_ylim(map_extent[2], map_extent[3])

    title = args.title or f"Overlays • raw {args.odom} + ref {args.plan}"
    ax.set_title(title, fontsize=11)

    # plt.tight_layout()
    plt.savefig(args.out, bbox_inches="tight", dpi=args.dpi)
    print(f"[OK] Saved {args.out}")

if __name__ == "__main__":
    main()
