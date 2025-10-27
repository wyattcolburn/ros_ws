
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Overlay trajectories from multiple rosbag2 runs on a map, with robust TF handling.
Now supports pulling the reference plan from a specific bag (e.g., your MLP bag).

CLI:
  --bags <glob/dir>         : rosbag2 directories (with metadata.yaml)
  --recursive               : discover nested bag dirs
  --odom /odom              : odometry topic
  --plan /plan_barn_odom    : preferred plan topic (tries /plan_barn as fallback)
  --plan-bag <bagdir>       : bag directory to read the reference plan from
  --map <map.yaml>          : map_server YAML to render
  --tf-mode {auto,dynamic,constant}
                             auto: prefer dynamic per-bag TF, fallback to constant
                             dynamic: always use time-varying TF
                             constant: always use one median map->odom TF
  --downsample N            : sample every Nth odom message
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
    ts: Optional[List[float]] = None  # seconds (odom time base)

# ------------------------- Map loading -------------------------

def load_occ_grid(yaml_path: str, emphasize: bool = False):
    """Load map_server YAML+PGM. Return (image[0..1], extent[xmin,xmax,ymin,ymax], res)."""
    with open(os.path.expanduser(yaml_path), "r") as f:
        meta = yaml.safe_load(f)

    pgm_path = meta["image"]
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(os.path.expanduser(yaml_path)), pgm_path)

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
    bag_path = os.path.expanduser(bag_path)
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

def _read_xy_t_from_odom(msg):  # nav_msgs/Odometry
    p = msg.pose.pose.position
    t = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
    return float(p.x), float(p.y), t

def _read_xy_from_pose(msg):  # geometry_msgs/PoseStamped
    p = msg.pose.position
    return float(p.x), float(p.y)

def _extract_series_for_topics(bag_path: str, odom_topic: str, plan_topic: str) -> Tuple[Series, Series]:
    """Return (odom Series with timestamps, plan Series without timestamps)."""
    r = _open_reader(bag_path, [odom_topic, plan_topic])
    types = _topic_types(r)

    des: Dict[str, Tuple[type, Callable]] = {}
    if odom_topic in types:
        des[odom_topic] = (get_message(types[odom_topic]), _read_xy_t_from_odom)
    if plan_topic in types:
        tname = types[plan_topic]
        if tname.endswith("/Path"):
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
        topic, data, _t_ns = r.read_next()
        if topic not in needed:
            continue
        mtype, fn = des[topic]
        msg = deserialize_message(data, mtype)
        if topic == odom_topic:
            x, y, t = fn(msg)
            od.xs.append(x); od.ys.append(y); od.ts.append(t)
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
    """Collect (t, tx, ty, yaw) samples from /tf_static and /tf, using *header* stamps."""
    r = _open_reader(bag_path, ["/tf_static", "/tf"])
    series = []
    while r.has_next():
        topic, data, _t_ns = r.read_next()
        if topic not in ("/tf_static", "/tf"):
            continue
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if tr.header.frame_id == parent and (tr.child_frame_id == child or tr.child_frame_id.startswith(child)):
                ts = tr.header.stamp
                t = float(ts.sec) + 1e-9 * float(ts.nanosec)
                yaw = _quat_to_yaw(tr.transform.rotation.x, tr.transform.rotation.y,
                                   tr.transform.rotation.z, tr.transform.rotation.w)
                series.append((t, tr.transform.translation.x, tr.transform.translation.y, yaw))
    series.sort(key=lambda a: a[0])
    # dedupe equal timestamps by keeping the last
    deduped = []
    last_t = None
    for s in series:
        if last_t is None or s[0] != last_t:
            deduped.append(s); last_t = s[0]
        else:
            deduped[-1] = s
    return deduped

def _unwrap(angles):
    return np.unwrap(np.array(angles, dtype=float))

def _median_tf_of_series(series):
    """Robust constant TF: median tx, ty, yaw (yaw unwrapped then rewrapped)."""
    if not series:
        return None
    tx = float(np.median([s[1] for s in series]))
    ty = float(np.median([s[2] for s in series]))
    yaw_un = _unwrap([s[3] for s in series])
    yaw = float(np.median(yaw_un))
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return {"tx": tx, "ty": ty, "yaw": yaw}

def _apply_SE2_arrays(xs, ys, tx, ty, yaw):
    """Vectorized SE(2) application when tx,ty,yaw are arrays matching xs/ys."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    tx = np.asarray(tx, dtype=float)
    ty = np.asarray(ty, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    c = np.cos(yaw); s = np.sin(yaw)
    x2 = tx + c * xs - s * ys
    y2 = ty + s * xs + c * ys
    return x2.tolist(), y2.tolist()

def _interp_tf_to_times(series, t_query_array):
    """Interpolate tx, ty, yaw (with unwrap) of TF series to query times."""
    if not series:
        z = np.zeros_like(t_query_array, dtype=float)
        return z, z, z
    times = np.array([s[0] for s in series], dtype=float)
    txs   = np.array([s[1] for s in series], dtype=float)
    tys   = np.array([s[2] for s in series], dtype=float)
    yaws  = _unwrap([s[3] for s in series])

    tq = np.clip(np.asarray(t_query_array, dtype=float), times[0], times[-1])

    txq = np.interp(tq, times, txs)
    tyq = np.interp(tq, times, tys)
    yawq = np.interp(tq, times, yaws)
    yawq = (yawq + math.pi) % (2.0 * math.pi) - math.pi
    return txq, tyq, yawq

# ------------------------- Discovery -------------------------

def discover_bag_dirs(pattern: str, recursive: bool) -> List[str]:
    pattern = os.path.expanduser(os.path.expandvars(pattern))
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
    ap.add_argument("--bags", required=True, help="Glob or dir of bag folders (containing metadata.yaml).")
    ap.add_argument("--recursive", action="store_true")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--plan", default="/plan_barn_odom", help="Preferred plan topic; also tries /plan_barn.")
    ap.add_argument("--plan-bag", default=None, help="Bag dir to pull the reference plan from (e.g., MLP bag).")
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

    # TF options
    ap.add_argument("--tf-parent", default="map")
    ap.add_argument("--tf-child",  default="odom")
    ap.add_argument("--learn-from", type=int, default=5, help="Bags to sample when learning constant TF (median).")
    ap.add_argument("--tf-dx", type=float, default=0.0, help="Trim meters added to tx for cached TF fallback.")
    ap.add_argument("--tf-dy", type=float, default=0.0, help="Trim meters added to ty for cached TF fallback.")
    ap.add_argument("--tf-dyaw", type=float, default=0.0, help="Trim radians added to yaw for cached TF fallback.")
    ap.add_argument("--tf-mode", choices=["auto", "dynamic", "constant"], default="constant",
                    help="auto: prefer dynamic per-bag TF, fallback to constant; dynamic: force time-varying; constant: force median constant.")

    # Plan options
    ap.add_argument("--drop-first-plan-point", action="store_true",
                    help="Drop first plan pose (e.g., if it’s pre-spawn).")
    ap.add_argument("--goal-radius", type=float, default=0.5, help="Goal tolerance radius (m).")

    args = ap.parse_args()

    bag_dirs = discover_bag_dirs(args.bags, args.recursive)[: args.max_bags]
    if not bag_dirs:
        raise SystemExit(f"No bag directories for: {args.bags}")

    # ----- Figure & map -----
    plt.figure(figsize=(7.8, 6.6), dpi=args.dpi)
    ax = plt.gca()
    map_extent = None
    if args.map:
        try:
            grid, extent, res = load_occ_grid(args.map, emphasize=args.emphasize_occupied)
            # Half-cell shift so pixel centers align with world coords
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

    # ----- Learn one constant TF (map->odom) once (for fallback/constant mode) -----
    samples = []
    for bag in bag_dirs[: max(1, args.learn_from)]:
        samples.extend(_collect_map_to_odom_series(bag, args.tf_parent, args.tf_child))
    cached_tf = _median_tf_of_series(samples)
    if cached_tf:
        cached_tf["tx"]  += args.tf_dx
        cached_tf["ty"]  += args.tf_dy
        cached_tf["yaw"] += args.tf_dyaw
        print(f"[TF-constant] tx={cached_tf['tx']:.3f} ty={cached_tf['ty']:.3f} yaw={cached_tf['yaw']:.4f} rad")
    else:
        print("[TF-constant] none (no /tf samples found)")

    # ----- Helper to load plan from a given bag -----
    def load_plan_from(bag_dir: Optional[str]) -> Tuple[Optional[Series], Optional[str]]:
        if not bag_dir:
            return None, None
        bag_dir = os.path.expanduser(bag_dir)
        if not os.path.exists(os.path.join(bag_dir, "metadata.yaml")):
            print(f"[PLAN] No metadata.yaml in {bag_dir}")
            return None, None
        _, plan_pref = _extract_series_for_topics(bag_dir, args.odom, args.plan)      # e.g., /plan_barn_odom
        _, plan_alt  = _extract_series_for_topics(bag_dir, args.odom, "/plan_barn")   # fallback
        if len(plan_pref.xs) >= len(plan_alt.xs) and plan_pref.xs:
            return plan_pref, args.plan
        if plan_alt.xs:
            return plan_alt, "/plan_barn"
        return None, None

    # ----- Choose one reference plan once -----
    cached_plan: Optional[Series] = None
    chosen_topic = None

    # 1) Try explicit plan-bag first (e.g., your MLP bag)
    cached_plan, chosen_topic = load_plan_from(args.plan_bag)

    # 2) Otherwise, scan the input bags for any plan
    if not cached_plan:
        for b in bag_dirs:
            cached_plan, chosen_topic = load_plan_from(b)
            if cached_plan:
                break

    if cached_plan:
        if args.drop_first_plan_point and len(cached_plan.xs) > 1:
            cached_plan = Series(cached_plan.xs[1:], cached_plan.ys[1:])
        print(f"[PLAN] Using '{chosen_topic}' with {len(cached_plan.xs)} poses as reference plan.")
    else:
        print("[PLAN] No plan found anywhere; continuing without a plan.")

    # ----- Plot all bags -----
    legend_labels = []
    for bag in bag_dirs:
        label = os.path.basename(bag) or bag.rstrip("/").split("/")[-1]
        try:
            odom, _ = _extract_series_for_topics(bag, args.odom, args.plan)
        except Exception as e:
            print(f"[WARN] Skipping {label}: {e}")
            continue

        if not odom.xs:
            print(f"[INFO] {label}: topic '{args.odom}' not found or had no data.")
            continue

        if args.downsample > 1:
            odom = Series(odom.xs[::args.downsample], odom.ys[::args.downsample],
                          odom.ts[::args.downsample] if odom.ts else None)

        # ---- Transform ODOM -> MAP according to tf-mode ----
        tf_series = _collect_map_to_odom_series(bag, args.tf_parent, args.tf_child)
        used = "raw odom"
        if args.tf_mode == "constant":
            if cached_tf:
                tx, ty, yaw = cached_tf["tx"], cached_tf["ty"], cached_tf["yaw"]
                ox, oy = _apply_SE2_arrays(odom.xs, odom.ys, tx, ty, yaw)
                odom = Series(ox, oy, odom.ts); used = "constant TF"
            else:
                print(f"[TF] {label}: no constant TF available; using raw odom.")
        elif args.tf_mode == "dynamic":
            if tf_series and odom.ts:
                txq, tyq, yawq = _interp_tf_to_times(tf_series, odom.ts)
                ox, oy = _apply_SE2_arrays(odom.xs, odom.ys, txq, tyq, yawq)
                odom = Series(ox, oy, odom.ts); used = "per-bag dynamic TF"
            else:
                print(f"[TF] {label}: missing TF or timestamps for dynamic; using raw odom.")
        else:  # auto
            if tf_series and odom.ts:
                txq, tyq, yawq = _interp_tf_to_times(tf_series, odom.ts)
                ox, oy = _apply_SE2_arrays(odom.xs, odom.ys, txq, tyq, yawq)
                odom = Series(ox, oy, odom.ts); used = "per-bag dynamic TF"
            elif cached_tf:
                tx, ty, yaw = cached_tf["tx"], cached_tf["ty"], cached_tf["yaw"]
                ox, oy = _apply_SE2_arrays(odom.xs, odom.ys, tx, ty, yaw)
                odom = Series(ox, oy, odom.ts); used = "constant TF"
            # else leave as raw odom

        print(f"[TF] {label}: {used}")

        # plot odom (now in map frame if TF applied)
        ax.plot(odom.xs, odom.ys, linewidth=1.0, alpha=0.9, zorder=2)
        legend_labels.append(f"{label}")

    # ----- Plot reference plan once (optional) -----
    if cached_plan and cached_plan.xs:
        ax.plot(cached_plan.xs, cached_plan.ys, linestyle="--", linewidth=2.2,
                color="red", alpha=0.85, zorder=6, label="plan")
        # start/goal markers
        ax.plot(cached_plan.xs[0],  cached_plan.ys[0],  marker="o", markersize=6,
                color="lime", markeredgecolor="black", zorder=8, label="start")
        ax.plot(cached_plan.xs[-1], cached_plan.ys[-1], marker="o", markersize=6,
                color="cyan", markeredgecolor="black", zorder=8, label="goal")
        # goal tolerance circle
        gx, gy = cached_plan.xs[-1], cached_plan.ys[-1]
        goal_tol = Circle((gx, gy), args.goal_radius, fill=False, linestyle='--',
                          linewidth=1.4, edgecolor='cyan', alpha=0.6, zorder=5, label="goal tolerance")
        ax.add_patch(goal_tol)

    # ----- Axes cosmetics -----
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.set_ylim(bottom=4.5)

    if legend_labels or (cached_plan and cached_plan.xs):
        ax.legend(fontsize=7, loc="upper left", ncol=1, framealpha=0.75)

    title = args.title or f"Overlays • odom: {args.odom} • tf-mode: {args.tf_mode}"
    ax.set_title(title, fontsize=11)

    plt.tight_layout()
    plt.savefig(args.out, bbox_inches="tight", dpi=args.dpi)
    print(f"[OK] Saved {args.out}")

if __name__ == "__main__":
    main()
