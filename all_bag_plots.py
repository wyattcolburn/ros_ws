
#!/usr/bin/env python3
import os, sys, glob
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
import rosbag2_py

TF_TOPICS = ("/tf", "/tf_static")

def norm_frame(name: str | None) -> str:
    if not name: return ""
    return name.strip().lstrip("/")

def T_from_tq(t, q):
    T = np.eye(4)
    T[:3,:3] = R.from_quat(q).as_matrix()
    T[:3,3]  = np.asarray(t, float)
    return T

def T_inv(T):
    Rm = T[:3,:3]; tt = T[:3,3]
    Ti = np.eye(4); Ti[:3,:3] = Rm.T; Ti[:3,3] = -Rm.T @ tt
    return Ti

def topic_types(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}

def find_bag_dirs(root):
    root = os.path.abspath(root)
    if os.path.isfile(os.path.join(root, "metadata.yaml")):
        return [root]
    out = []
    for meta in glob.glob(os.path.join(root, "**", "metadata.yaml"), recursive=True):
        out.append(os.path.dirname(meta))
    return sorted(set(out))

def extract_odom_xy_and_frame(bag_dir, odom_topic="/odom"):
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader(); r.open(so, co)
    tmap = topic_types(r)
    if odom_topic not in tmap:
        print(f"[WARN] {bag_dir}: {odom_topic} not found.")
        return [], [], "odom"
    Odom = get_message(tmap[odom_topic])
    xs, ys, oframe = [], [], None
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic != odom_topic: continue
        m = deserialize_message(raw, Odom)
        xs.append(m.pose.pose.position.x)
        ys.append(m.pose.pose.position.y)
        if oframe is None:
            oframe = norm_frame(m.header.frame_id) or "odom"
    return xs, ys, oframe or "odom"

def extract_latest_path_xy_frame_time(bag_dir, path_topic="/plan_barn_odom"):
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader(); r.open(so, co)
    tmap = topic_types(r)
    if path_topic not in tmap:
        return [], [], None, None
    PathMsg = get_message(tmap[path_topic])
    latest, ts_latest = None, None
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic != path_topic: continue
        latest = deserialize_message(raw, PathMsg); ts_latest = ts
    if latest is None or not latest.poses:
        return [], [], None, None
    px = [p.pose.position.x for p in latest.poses]
    py = [p.pose.position.y for p in latest.poses]
    pframe = norm_frame(latest.header.frame_id) or "map"
    return px, py, pframe, ts_latest

def build_tf_index(bag_dir):
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader(); r.open(so, co)
    tmap = topic_types(r)
    tf_topics = [t for t in TF_TOPICS if t in tmap]
    if not tf_topics:
        return {}, defaultdict(set), {}

    TFMsg = get_message(tmap[tf_topics[0]])
    edges = defaultdict(list)
    graph = defaultdict(set)

    r = rosbag2_py.SequentialReader(); r.open(so, co)
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic not in TF_TOPICS: continue
        msg = deserialize_message(raw, TFMsg)
        for tr in msg.transforms:
            parent = norm_frame(tr.header.frame_id)
            child  = norm_frame(tr.child_frame_id)
            t = (tr.transform.translation.x,
                 tr.transform.translation.y,
                 tr.transform.translation.z)
            q = (tr.transform.rotation.x,
                 tr.transform.rotation.y,
                 tr.transform.rotation.z,
                 tr.transform.rotation.w)
            T = T_from_tq(t, q)
            edges[(parent, child)].append((ts, T))
            graph[parent].add(child)
            graph[child].add(parent)
    for k in edges:
        edges[k].sort(key=lambda kv: kv[0])
    return edges, graph, {}

def nearest_T(edges, a, b, ts_hint):
    seq = edges.get((a, b))
    if not seq: return None
    if ts_hint is None: return seq[-1][1]
    _, T = min(seq, key=lambda kv: abs(kv[0] - ts_hint))
    return T

def find_chain_T(edges, graph, src, dst, ts_hint, chain_cache=None):
    if src == dst: return np.eye(4), [src]
    if chain_cache is None: chain_cache = {}
    key = (src, dst)
    frames = chain_cache.get(key)
    if frames is None:
        q = deque([src]); prev = {src: None}; found = False
        while q and not found:
            u = q.popleft()
            for v in graph[u]:
                if v not in prev:
                    prev[v] = u; q.append(v)
                    if v == dst: found = True; break
        if not found: return None, None
        frames = [dst]
        while frames[-1] != src: frames.append(prev[frames[-1]])
        frames.reverse()
        chain_cache[key] = frames
    T_total = np.eye(4)
    for a, b in zip(frames[:-1], frames[1:]):
        T_ab = nearest_T(edges, a, b, ts_hint)
        if T_ab is None:
            T_ba = nearest_T(edges, b, a, ts_hint)
            if T_ba is None: return None, None
            T_step = T_inv(T_ba)
        else:
            T_step = T_ab
        T_total = T_total @ T_step
    return T_total, frames

def transform_xy(xs, ys, T):
    if not xs: return [], []
    P  = np.vstack([xs, ys, np.zeros(len(xs)), np.ones(len(xs))])
    P2 = T @ P
    return P2[0,:].tolist(), P2[1,:].tolist()

def safe_name(s: str) -> str:
    return "".join(c if c.isalnum() or c in "-_." else "_" for c in s)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bag_overlay_tf_allinone.py <bag_or_root_dir> [/odom] [/plan_barn_odom] [out.png]")
        sys.exit(1)

    root_or_bag = sys.argv[1]
    odom_topic  = sys.argv[2] if len(sys.argv) >= 3 else "/odom"
    plan_topic  = sys.argv[3] if len(sys.argv) >= 4 else "/plan_barn_odom"
    out_path    = sys.argv[4] if len(sys.argv) >= 5 else None

    bag_dirs = find_bag_dirs(root_or_bag)
    if not bag_dirs:
        print(f"[ERROR] No rosbag2 dirs found under {root_or_bag}")
        sys.exit(2)

    plt.figure(figsize=(11,11))
    any_plotted = False

    for bag in bag_dirs:
        base = os.path.basename(bag.rstrip("/"))
        ox, oy, oframe = extract_odom_xy_and_frame(bag, odom_topic)
        if not ox:
            print(f"[WARN] {base}: no odom; skipping.")
            continue

        px, py, pframe, pts = extract_latest_path_xy_frame_time(bag, plan_topic)

        # Try to TF the plan to odom frame if possible
        px_o, py_o = [], []
        if pframe is not None and px:
            edges, graph, cache = build_tf_index(bag)
            if edges:
                T_po, chain = find_chain_T(edges, graph, pframe, oframe, pts, cache)
                if T_po is not None:
                    px_o, py_o = transform_xy(px, py, T_po)
                else:
                    px_o, py_o = px, py
            else:
                px_o, py_o = px, py

        # Plot
        plt.plot(ox, oy, "-", lw=2, alpha=0.9, label=f"{base} (odom)")
        if px_o:
            plt.plot(px_o, py_o, "--", lw=2, alpha=0.9, label=f"{base} (plan)")

        any_plotted = True

    if not any_plotted:
        print("[ERROR] nothing to plot.")
        sys.exit(3)

    plt.axis("equal"); plt.grid(True, alpha=.3)
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    title_bits = [
        f"All overlays ({len(bag_dirs)} bag{'s' if len(bag_dirs)!=1 else ''})",
        f"odom: {odom_topic}", f"plan: {plan_topic}"
    ]
    plt.title(" • ".join(title_bits))
    # If too many entries, legend can be huge; feel free to comment next line.
    plt.legend(fontsize=8, ncol=2)

    if out_path:
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        plt.savefig(out_path, dpi=300, bbox_inches="tight")
        print(f"[OK] saved {out_path}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
