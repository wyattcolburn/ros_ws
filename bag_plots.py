#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict, deque
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
import rosbag2_py

TF_TOPICS = ("/tf", "/tf_static")

def norm_frame(name: str | None) -> str:
    if not name:
        return ""
    return name.strip().lstrip("/")

def T_from_tq(t, q):
    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = np.asarray(t, float)
    return T

def T_inv(T):
    Rm = T[:3,:3]; tt = T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = Rm.T
    Ti[:3,3] = -Rm.T @ tt
    return Ti

def topic_types(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}

def extract_odom_xy_and_frame(bag_dir, odom_topic="/odom"):
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader(); r.open(so, co)
    tmap = topic_types(r)
    if odom_topic not in tmap:
        print(f"[ERROR] {odom_topic} not found.")
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
        print(f"[ERROR] {path_topic} not found.")
        return [], [], None, None
    PathMsg = get_message(tmap[path_topic])
    latest, ts_latest = None, None
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic != path_topic: continue
        latest = deserialize_message(raw, PathMsg); ts_latest = ts
    if latest is None or not latest.poses:
        print(f"[ERROR] empty Path on {path_topic}.")
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
    if not tf_topics: return {}, defaultdict(set)
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
            graph[child].add(parent)  # undirected search
    for k in edges:
        edges[k].sort(key=lambda kv: kv[0])
    return edges, graph

def nearest_T(edges, a, b, ts_hint):
    seq = edges.get((a, b))
    if not seq: return None
    if ts_hint is None: return seq[-1][1]
    # pick closest in time
    _, T = min(seq, key=lambda kv: abs(kv[0] - ts_hint))
    return T

def find_chain_T(edges, graph, src, dst, ts_hint):
    if src == dst: return np.eye(4), [src]
    # BFS for a frame path
    q = deque([src]); prev = {src: None}
    while q:
        u = q.popleft()
        for v in graph[u]:
            if v not in prev:
                prev[v] = u
                q.append(v)
                if v == dst:
                    q.clear(); break
    if dst not in prev: return None, None
    # reconstruct frame path
    frames = [dst]
    while frames[-1] != src:
        frames.append(prev[frames[-1]])
    frames.reverse()
    # compose along path, inverting edges as needed
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
    P = np.vstack([xs, ys, np.zeros(len(xs)), np.ones(len(xs))])
    P2 = T @ P
    return P2[0,:].tolist(), P2[1,:].tolist()

def centroid(arr):
    return float(np.mean(arr)) if len(arr) else 0.0

def plot_overlay(bag_dir, odom_topic="/odom", plan_topic="/plan_barn", out=None):
    print(f"[INFO] Bag: {bag_dir}")
    ox, oy, oframe = extract_odom_xy_and_frame(bag_dir, odom_topic)
    px, py, pframe, pts = extract_latest_path_xy_frame_time(bag_dir, plan_topic)
    if not ox:
        print("[ERROR] no odom samples.")
        return
    print(f"[INFO] odom frame: {oframe}  | plan frame: {pframe}")

    edges, graph = build_tf_index(bag_dir)
    if not edges:
        print("[WARN] no TF in bag; plotting raw plan.")
        px_o, py_o = px, py
        chain = None
    else:
        T_po, chain = find_chain_T(edges, graph, pframe, oframe, pts)
        if T_po is None:
            print(f"[WARN] no TF chain from {pframe} to {oframe}; raw plan used.")
            px_o, py_o = px, py
        else:
            print(f"[INFO] TF chain: {' -> '.join(chain)}")
            px_o, py_o = transform_xy(px, py, T_po)

    print(f"[DBG] centroids: odom({centroid(ox):.2f},{centroid(oy):.2f})  "
          f"plan({centroid(px_o):.2f},{centroid(py_o):.2f})")

    plt.figure(figsize=(10,10))
    plt.plot(ox, oy, "-", lw=2, label=f"Actual ({odom_topic}, frame={oframe})")
    if px_o:
        plt.plot(px_o, py_o, "--", lw=2, label=f"Plan ({plan_topic} → {oframe})")
    else:
        print("[WARN] no plan points after transform.")
    plt.axis("equal"); plt.grid(True, alpha=.3)
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    plt.title(f"Odom vs Plan overlay from {bag_dir}")
    plt.legend()
    if out:
        plt.savefig(out, dpi=300, bbox_inches="tight"); print(f"[OK] saved {out}")
    else:
        plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_overlay_tf.py <bag_dir> [/odom] [/plan_barn] [output.png]")
        sys.exit(1)
    bag_dir    = sys.argv[1]
    odom_topic = sys.argv[2] if len(sys.argv) >= 3 else "/odom"
    plan_topic = sys.argv[3] if len(sys.argv) >= 4 else "/plan_barn_odom"
    out        = sys.argv[4] if len(sys.argv) >= 5 else None
    plot_overlay(bag_dir, odom_topic, plan_topic, out)
