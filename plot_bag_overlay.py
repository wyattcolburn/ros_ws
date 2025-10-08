
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Overlay trajectories from multiple rosbag2 runs with a single, per-run
reference plan and a single, per-run constant TF (map→odom).

- Learn TF once from the first N bags (median of samples), reuse for all.
- Choose one plan once (tries /plan_barn_odom then /plan_barn), reuse for all.
- Plot odom (solid) and the reference plan (dashed) over a map (YAML→PGM).
"""

import argparse, glob, math, os
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
from tf2_msgs.msg import TFMessage


# ------------------------- Data structures -------------------------

@dataclass
class Series:
    xs: List[float]
    ys: List[float]
    ts: Optional[List[float]] = None  # seconds (for /odom timing if needed)


# ------------------------- Map loading -------------------------

def load_occ_grid(yaml_path: str, emphasize: bool = False):
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
    img = np.flipud(img)  # origin='lower'
    if emphasize:
        img = np.clip((img - free_th) / max(1e-6, (occ_th - free_th)), 0, 1)

    h, w = img.shape[:2]
    extent = [ox, ox + w * res, oy, oy + h * res]
    return img, extent


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


# ------------------------- TF utilities -------------------------

def _quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def _collect_map_to_odom_series(bag_path, parent="map", child="odom"):
    """Collect many (t, tx, ty, yaw) samples from /tf_static and /tf."""
    r = _open_reader(bag_path, ["/tf_static", "/tf"])
    series = []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic not in ("/tf_static", "/tf"):
            continue
        msg = deserialize_message(data, TFMessage)
        t = t_ns / 1e9
        for tr in msg.transforms:
            if tr.header.frame_id == parent and (tr.child_frame_id == child or tr.child_frame_id.startswith(child)):
                yaw = _quat_to_yaw(tr.transform.rotation.x, tr.transform.rotation.y,
                                   tr.transform.rotation.z, tr.transform.rotation.w)
                series.append((t, tr.transform.translation.x, tr.transform.translation.y, yaw))
    series.sort(key=lambda a: a[0])
    return series

def _apply_SE2(xs, ys, tx, ty, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    xs2 = [tx + c * x - s * y for x, y in zip(xs, ys)]
    ys2 = [ty + s * x + c * y for x, y in zip(xs, ys)]
    return xs2, ys2

def _unwrap(angles):
    return np.unwrap(np.array(angles))

def _median_tf_of_series(series):
    """Robust constant TF: median tx, ty, yaw (yaw unwrapped then rewrapped)."""
    if not series:
        return None
    tx = np.median([s[1] for s in series])
    ty = np.median([s[2] for s in series])
    yaw_un = _unwrap([s[3] for s in series])
    yaw = float(np.median(yaw_un))
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]
    return {"tx": float(tx), "ty": float(ty), "yaw": yaw}


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
    ap = argparse.ArgumentParser("Overlay odom + one reference plan across rosbag2 runs.")
    ap.add_argument("--bags", required=True, help="Glob or dir of bag folders.")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--plan", default="/plan_barn_odom", help="Preferred plan topic; script also tries /plan_barn.")
    ap.add_argument("--out", default="overlay.png")
    ap.add_argument("--title", default="")
    ap.add_argument("--max-bags", type=int, default=9999)
    ap.add_argument("--downsample", type=int, default=1)

    # Map
    ap.add_argument("--map", help="Map YAML (map_server).")
    ap.add_argument("--map-alpha", type=float, default=0.4)
    ap.add_argument("--emphasize-occupied", action="store_true")
    ap.add_argument("--map-vmin", type=float, default=0.0)
    ap.add_argument("--map-vmax", type=float, default=1.0)
    ap.add_argument("--dpi", type=int, default=300)

    # TF learning (once per run)
    ap.add_argument("--tf-parent", default="map")
    ap.add_argument("--tf-child",  default="odom")
    ap.add_argument("--learn-from", type=int, default=5, help="Bags to sample when learning constant TF (median).")

    args = ap.parse_args()

    bag_dirs = discover_bag_dirs(args.bags, args.recursive)[: args.max_bags]
    if not bag_dirs:
        raise SystemExit(f"No bag directories for: {args.bags}")

    # Figure & map
    plt.figure(figsize=(7.8, 6.6), dpi=args.dpi)
    ax = plt.gca()
    map_extent = None
    if args.map:
        try:
            grid, extent = load_occ_grid(args.map, emphasize=args.emphasize_occupied)
            res = .15
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

    # ---------- Learn one constant TF (map→odom) once ----------
    samples = []
    for bag in bag_dirs[: max(1, args.learn_from)]:
        s = _collect_map_to_odom_series(bag, args.tf_parent, args.tf_child)
        samples.extend(s)
    cached_tf = _median_tf_of_series(samples)
    if not cached_tf:
        print("[WARN] No TF samples found; odom will be raw (map overlay may not line up).")
    else:
        print(f"[TF] Using constant {args.tf_parent}->{args.tf_child} "
              f"tx={cached_tf['tx']:.3f}, ty={cached_tf['ty']:.3f}, yaw={cached_tf['yaw']:.3f} rad")

    # ---------- Choose one reference plan once ----------
    cached_plan = None
    chosen_topic = None
    for bag in bag_dirs:
        # try preferred, then /plan_barn
        _, plan_pref = _extract_series_for_topics(bag, args.odom, args.plan)
        _, plan_alt  = _extract_series_for_topics(bag, args.odom, "/plan_barn")
        if len(plan_pref.xs) >= len(plan_alt.xs) and plan_pref.xs:
            cached_plan = plan_pref; chosen_topic = args.plan
            break
        if plan_alt.xs:
            cached_plan = plan_alt; chosen_topic = "/plan_barn"
            cached_plan = Series(cached_plan.xs[1:], cached_plan.ys[1:])
            break
    if cached_plan:
        print(f"[PLAN] Using '{chosen_topic}' with {len(cached_plan.xs)} poses as the reference plan.")
    else:
        print("[PLAN] No plan found in any bag; proceeding without a plan.")

    # ---------- Loop over all bags, transform odom with the constant TF ----------
    legend_labels = []
    any_plot = False

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

        # Apply constant TF to odom (if available)
        if cached_tf and odom.xs:
            tx, ty, yaw = cached_tf["tx"], cached_tf["ty"], cached_tf["yaw"]
            xs2, ys2 = _apply_SE2(odom.xs, odom.ys, tx, ty, yaw)
            odom = Series(xs2, ys2, odom.ts)

        # Plot odom
        if odom.xs:
            ax.plot(odom.xs, odom.ys, linewidth=1.0, alpha=0.85, zorder=2)
            legend_labels.append(f"{label} (odom)")
            any_plot = True
        else:
            print(f"[INFO] {label}: topic '{args.odom}' not found or had no data.")

    # Plot the single reference plan once
    if cached_plan and cached_plan.xs:
        ax.plot(cached_plan.xs, cached_plan.ys,
                linestyle="--", linewidth=2.2, color="red",
                alpha=0.78, zorder=6, label=f"plan")
        # ax.scatter(cached_plan.xs, cached_plan.ys, s=14,
        #            color="yellow", edgecolor="black", linewidths=0.6, zorder=7)
        ax.plot(cached_plan.xs[0],  cached_plan.ys[0],  marker="o", markersize=5,
                color="lime", markeredgecolor="black", zorder=8, label="start")
        ax.plot(cached_plan.xs[-1], cached_plan.ys[-1], marker="o", markersize=5,
                color="cyan", markeredgecolor="black", zorder=8, label="goal")

        goal_tol = Circle((cached_plan.xs[-1], cached_plan.ys[-1]), .5, fill=False, linestyle='--',
                  linewidth=1, edgecolor='cyan', alpha=0.5, zorder=5, label="goal tolerance")
        ax.add_patch(goal_tol)
        any_plot = True

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    if legend_labels:
        ax.legend(fontsize=7, loc="upper left", ncol=1, framealpha=0.75)

    # Union limits so nothing is clipped (map + paths)
    xs_all, ys_all = [], []
    for line in ax.get_lines():
        x = line.get_xdata(orig=False); y = line.get_ydata(orig=False)
        if len(x): xs_all.extend(x)
        if len(y): ys_all.extend(y)
    if xs_all and ys_all:
        xmin, xmax = min(xs_all), max(xs_all)
        ymin, ymax = min(ys_all), max(ys_all)
        if map_extent:
            xmin = min(xmin, map_extent[0]); xmax = max(xmax, map_extent[1])
            ymin = min(ymin, map_extent[2]); ymax = max(ymax, map_extent[3])
        pad_x = 0.03 * (xmax - xmin if xmax > xmin else 1.0)
        pad_y = 0.03 * (ymax - ymin if ymax > ymin else 1.0)
        ax.set_xlim(xmin - pad_x, xmax + pad_x)
        ax.set_ylim(ymin - pad_y, ymax + pad_y)
    elif map_extent:
        ax.set_xlim(map_extent[0], map_extent[1])
        ax.set_ylim(map_extent[2], map_extent[3])

    title = args.title or f"Overlays • odom: {args.odom} • ref plan (auto-picked)"
    ax.set_title(title, fontsize=11)

    ax.set_ylim(bottom=4)
    plt.tight_layout()
    plt.savefig(args.out, bbox_inches="tight", dpi=args.dpi)
    print(f"[OK] Saved {args.out}")


if __name__ == "__main__":
    main()
