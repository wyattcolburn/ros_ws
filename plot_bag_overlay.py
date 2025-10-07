
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Overlay trajectories from multiple ROS 2 bags (rosbag2):
- Plots Odometry XY (solid) and Plan XY (dashed).
- Auto-picks plan topic between /plan_barn_odom and /plan_barn.
- Optional occupancy map background from map_server YAML (.yaml -> .pgm).
- Transforms /odom → map using TF from the bag (per-point nearest-in-time).
- Optional spawn-based translation (no rotation) as fallback.
- Topic filtering for speed; optional recursive bag discovery.
"""

import argparse
import glob
import math
import os
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml

# ROS 2 imports
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from tf2_msgs.msg import TFMessage


# ------------------------- Data structures -------------------------

@dataclass
class Series:
    xs: List[float]
    ys: List[float]
    ts: Optional[List[float]] = None  # seconds (for per-point TF); None for plan


# ------------------------- Map loading -------------------------

def load_occ_grid(yaml_path: str, emphasize: bool = False):
    """Load map_server YAML+PGM and return (img in [0,1], extent=[xmin,xmax,ymin,ymax])."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    pgm_path = meta["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    res = float(meta["resolution"])
    ox, oy, _yaw = meta.get("origin", [0.0, 0.0, 0.0])
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))

    img = np.array(Image.open(pgm_path)).astype(np.float32) / 255.0
    if negate == 1:
        img = 1.0 - img

    img = np.flipud(img)  # origin='lower'
    if emphasize:
        img = np.clip((img - free_th) / max(1e-6, (occ_th - free_th)), 0, 1)

    h, w = img.shape[:2]
    extent = [ox, ox + w * res, oy, oy + h * res]
    return img, extent


# ------------------------- Bag reading -------------------------

def _open_reader(bag_path: str, filter_topics: Optional[List[str]] = None) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    if filter_topics:
        try:
            from rosbag2_py import StorageFilter
            reader.set_filter(StorageFilter(topics=filter_topics))
        except Exception:
            pass
    return reader


def _topic_types(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def _read_xy_from_odom(msg) -> Tuple[float, float]:
    return float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)


def _read_xy_from_pose(msg) -> Tuple[float, float]:
    return float(msg.pose.position.x), float(msg.pose.position.y)


def _extract_series_for_topics(bag_path: str, odom_topic: str, plan_topic: str) -> Tuple[Series, Series]:
    """Return (odom Series with timestamps, plan Series without timestamps)."""
    reader = _open_reader(bag_path, [odom_topic, plan_topic])
    types = _topic_types(reader)

    deserializers: Dict[str, Tuple[type, Callable]] = {}
    if odom_topic in types:
        odom_type = get_message(types[odom_topic])
        deserializers[odom_topic] = (odom_type, _read_xy_from_odom)
    if plan_topic in types:
        plan_type = get_message(types[plan_topic])
        if types[plan_topic].endswith('/Path'):
            from nav_msgs.msg import Path  # noqa
            def _from_path(msg):
                return [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
            plan_reader_fn = _from_path
        elif types[plan_topic].endswith('/PoseStamped'):
            plan_reader_fn = _read_xy_from_pose
        else:
            raise RuntimeError(f"Unsupported plan message type for {plan_topic}: {types[plan_topic]}")
        deserializers[plan_topic] = (plan_type, plan_reader_fn)

    odom = Series([], [], [])
    plan = Series([], [], None)
    needed = {t for t in (odom_topic, plan_topic) if t in deserializers}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in needed:
            continue
        msg_type, reader_fn = deserializers[topic]
        msg = deserialize_message(data, msg_type)

        if topic == odom_topic:
            x, y = reader_fn(msg)
            odom.xs.append(x); odom.ys.append(y); odom.ts.append(t_ns / 1e9)
        else:
            out = reader_fn(msg)
            if isinstance(out, list):  # Path
                for x, y in out:
                    plan.xs.append(x); plan.ys.append(y)
            else:  # PoseStamped
                x, y = out
                plan.xs.append(x); plan.ys.append(y)

    return odom, plan


# ------------------------- TF utilities -------------------------

def _quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def _collect_map_to_odom_series(bag_path, parent="map", child="odom"):
    """
    Return sorted list of (t_sec, tx, ty, yaw) from /tf_static and /tf
    for transform parent->child (map->odom).
    """
    reader = _open_reader(bag_path, ["/tf_static", "/tf"])
    series = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in ("/tf_static", "/tf"):
            continue
        msg = deserialize_message(data, TFMessage)
        t_sec = t_ns / 1e9
        for tr in msg.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                yaw = _quat_to_yaw(tr.transform.rotation.x,
                                   tr.transform.rotation.y,
                                   tr.transform.rotation.z,
                                   tr.transform.rotation.w)
                series.append((t_sec,
                               tr.transform.translation.x,
                               tr.transform.translation.y,
                               yaw))
    series.sort(key=lambda a: a[0])
    return series

def _nearest_tf(series, t_query):
    """Pick the TF sample closest in time to t_query (seconds)."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t_query:
            lo = mid + 1
        else:
            hi = mid
    idx = max(0, min(lo, len(series) - 1))
    if idx > 0 and abs(series[idx-1][0] - t_query) < abs(series[idx][0] - t_query):
        idx -= 1
    _, tx, ty, yaw = series[idx]
    return tx, ty, yaw

def _apply_SE2(xs, ys, tx, ty, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    xs2 = [tx + c*x - s*y for x, y in zip(xs, ys)]
    ys2 = [ty + s*x + c*y for x, y in zip(xs, ys)]
    return xs2, ys2


# ------------------------- Discovery -------------------------

def discover_bag_dirs(pattern: str, recursive: bool) -> List[str]:
    """Return sorted list of bag directories (with metadata.yaml)."""
    dirs = []
    for d in sorted(glob.glob(pattern)):
        if os.path.isdir(d):
            dirs.append(d)

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
            seen.add(d)
            out.append(d)
    return sorted(out)


# ------------------------- Helpers -------------------------

def _peek_plan_frame(bag_path: str, plan_topic: str) -> Optional[str]:
    r = _open_reader(bag_path, [plan_topic])
    types = _topic_types(r)
    if plan_topic not in types:
        return None
    mtype = get_message(types[plan_topic])
    while r.has_next():
        tname, data, _ = r.read_next()
        if tname != plan_topic:
            continue
        msg = deserialize_message(data, mtype)
        return getattr(msg, "header", None).frame_id if hasattr(msg, "header") else None
    return None


# ------------------------- Main -------------------------

def main():
    ap = argparse.ArgumentParser(description="Overlay XY trajectories from multiple ROS 2 bags.")
    ap.add_argument("--bags", required=True,
                    help="Glob or directory for bag folders (each has metadata.yaml & *.db3).")
    ap.add_argument("--recursive", action="store_true",
                    help="Recursively discover bag dirs under --bags.")
    ap.add_argument("--odom", default="/odom", help="Odometry topic (nav_msgs/Odometry).")
    ap.add_argument("--plan", default="/plan_barn_odom",
                    help="Preferred plan topic; script also tries /plan_barn automatically.")
    ap.add_argument("--out", default="overlay.png", help="Output PNG filename.")
    ap.add_argument("--title", default="", help="Figure title.")
    ap.add_argument("--max_bags", type=int, default=9999, help="Cap number of bags to plot.")
    ap.add_argument("--downsample", type=int, default=1,
                    help="Keep every Nth point for Odometry and Plan.")
    ap.add_argument("--map", help="Path to map YAML (that references the .pgm).")
    ap.add_argument("--map-alpha", type=float, default=0.35, help="Alpha for map overlay.")
    ap.add_argument("--emphasize-occupied", action="store_true", help="Boost map contrast.")
    ap.add_argument("--dpi", type=int, default=300, help="Output image DPI.")
    ap.add_argument("--map-vmin", type=float, default=0.0, help="imshow vmin for map")
    ap.add_argument("--map-vmax", type=float, default=1.0, help="imshow vmax for map")
    ap.add_argument("--no-tf-align", action="store_true",
                    help="Disable odom→map TF alignment from bag.")
    ap.add_argument("--spawn-map", nargs=2, type=float, metavar=("X","Y"),
                    help="Fallback: translate all odom so FIRST odom point maps to (X,Y).")
    ap.add_argument("--plan-color", default="red")
    ap.add_argument("--plan-width", type=float, default=2.6)
    ap.add_argument("--print-plan", action="store_true",
                    help="Print pose count for the chosen plan per bag.")
    args = ap.parse_args()

    bag_dirs = discover_bag_dirs(args.bags, args.recursive)
    if not bag_dirs:
        raise SystemExit(f"No bag directories found for: {args.bags}")
    bag_dirs = bag_dirs[: args.max_bags]

    # Figure & axes
    plt.figure(figsize=(7.8, 6.6), dpi=args.dpi)
    ax = plt.gca()

    # Map background
    map_extent = None
    if args.map:
        try:
            grid, extent = load_occ_grid(args.map, emphasize=args.emphasize_occupied)
            ax.imshow(grid, extent=extent, cmap="gray", origin="lower",
                      alpha=args.map_alpha, interpolation="nearest", zorder=0,
                      vmin=args.map_vmin, vmax=args.map_vmax)
            map_extent = extent
        except Exception as e:
            print(f("[WARN] Could not render map '{args.map}': {e}"))

    # Precompute spawn translation if requested (use first bag with odom)
    spawn_offset = None
    if args.spawn_map is not None:
        spawn_xy = np.array([args.spawn_map[0], args.spawn_map[1]], dtype=float)
        for probe in bag_dirs:
            od_probe, _ = _extract_series_for_topics(probe, args.odom, args.plan)
            if od_probe.xs:
                first_odom = np.array([od_probe.xs[0], od_probe.ys[0]], dtype=float)
                spawn_offset = (spawn_xy - first_odom)
                print(f"[INFO] Spawn translation dx,dy={spawn_offset.tolist()} "
                      f"(first odom {first_odom.tolist()} → spawn {spawn_xy.tolist()})")
                break
        if spawn_offset is None:
            print("[WARN] --spawn-map set but no odom points found to compute offset.")

    legend_labels = []
    any_plot = False

    for bag in bag_dirs:
        label = os.path.basename(bag) or bag.rstrip("/").split("/")[-1]

        # Extract data (first try args.plan, then /plan_barn; keep the richer one)
        try:
            odom, plan_pref = _extract_series_for_topics(bag, args.odom, args.plan)
            _,    plan_alt  = _extract_series_for_topics(bag, args.odom, "/plan_barn")
            if len(plan_alt.xs) > len(plan_pref.xs):
                plan = plan_alt
                plan_topic_used = "/plan_barn"
            else:
                plan = plan_pref
                plan_topic_used = args.plan
        except Exception as e:
            print(f"[WARN] Skipping {label}: {e}")
            continue

        if args.print_plan:
            print(f"[PLAN] {label} {plan_topic_used}: {len(plan.xs)} poses")

        # Downsample
        if args.downsample > 1:
            odom = Series(odom.xs[::args.downsample], odom.ys[::args.downsample],
                          odom.ts[::args.downsample] if odom.ts else None)
            plan = Series(plan.xs[::args.downsample], plan.ys[::args.downsample], None)

        # Transform odom → map using TF (per-point nearest-in-time)
        tf_series = None
        if not args.no_tf_align and odom.xs:
            tf_series = _collect_map_to_odom_series(bag, "map", "odom")
            if tf_series:
                xs2, ys2 = [], []
                for x, y, tsec in zip(odom.xs, odom.ys, odom.ts or [0.0]*len(odom.xs)):
                    tf = _nearest_tf(tf_series, tsec)
                    if tf is None:
                        continue
                    tx, ty, yaw = tf
                    x2, y2 = _apply_SE2([x], [y], tx, ty, yaw)
                    xs2.append(x2[0]); ys2.append(y2[0])
                odom = Series(xs2, ys2, odom.ts)
            else:
                print(f"[INFO] {label}: no map→odom TF found; plotting raw odom.")

        # Optional spawn translation (only if TF alignment is off or not available)
        if spawn_offset is not None and (args.no_tf_align or not tf_series) and odom.xs:
            ox = (np.array(odom.xs) + spawn_offset[0]).tolist()
            oy = (np.array(odom.ys) + spawn_offset[1]).tolist()
            odom = Series(ox, oy, odom.ts)

        # Plot odom
        if odom.xs:
            ax.plot(odom.xs, odom.ys, linewidth=1.6, alpha=0.85, zorder=2)
            legend_labels.append(f"{label} (odom)")
            any_plot = True
        else:
            print(f"[INFO] {label}: topic '{args.odom}' not found or had no data.")

        # Plot plan (raw from bag; typically in map frame)
        if plan.xs:
            # If plan frame somehow isn't map, and TF exists, transform once
            plan_frame = _peek_plan_frame(bag, plan_topic_used) or "map"
            if plan_frame != "map" and tf_series:
                tx, ty, yaw = tf_series[0][1], tf_series[0][2], tf_series[0][3]
                px, py = _apply_SE2(plan.xs, plan.ys, tx, ty, yaw)
                plan = Series(px, py)

            ax.plot(
                plan.xs, plan.ys,
                linestyle="--", linewidth=args.plan_width, alpha=0.98,
                color=args.plan_color, zorder=6
            )
            # Optional markers to make it pop (comment out if too busy)
            ax.plot(plan.xs[0], plan.ys[0], marker="o", markersize=5,
                    color="lime", markeredgecolor="black", zorder=8)
            ax.plot(plan.xs[-1], plan.ys[-1], marker="x", markersize=5,
                    color=args.plan_color, zorder=8)

            legend_labels.append(f"{label} (plan: {plan_topic_used})")
            any_plot = True
        else:
            print(f"[INFO] {label}: no poses on '{plan_topic_used}'.")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")

    # Legend
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, legend_labels, fontsize=7, loc="upper left", ncol=1, framealpha=0.75)

    # View limits: union(map extent, all plotted data)
    xs_all, ys_all = [], []
    for line in ax.get_lines():
        xd = line.get_xdata(orig=False); yd = line.get_ydata(orig=False)
        if len(xd): xs_all.extend(xd)
        if len(yd): ys_all.extend(yd)

    if xs_all and ys_all:
        xmin, xmax = min(xs_all), max(xs_all)
        ymin, ymax = min(ys_all), max(ys_all)
        if map_extent:
            xmin = min(xmin, map_extent[0]); xmax = max(xmax, map_extent[1])
            ymin = min(ymin, map_extent[2]); ymax = max(ymax, map_extent[3])
        padx = 0.03 * (xmax - xmin if xmax > xmin else 1.0)
        pady = 0.03 * (ymax - ymin if ymax > ymin else 1.0)
        ax.set_xlim(xmin - padx, xmax + padx)
        ax.set_ylim(ymin - pady, ymax + pady)
    elif map_extent:
        ax.set_xlim(map_extent[0], map_extent[1])
        ax.set_ylim(map_extent[2], map_extent[3])

    title = args.title or f"Overlays • odom: {args.odom} • plan: {args.plan} (auto-tries /plan_barn)"
    ax.set_title(title, fontsize=11)

    plt.tight_layout()
    plt.savefig(args.out, bbox_inches="tight", dpi=args.dpi)
    print(f"[OK] Saved {args.out}")


if __name__ == "__main__":
    main()
