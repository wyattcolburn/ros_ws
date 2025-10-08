
#!/usr/bin/env python3
import os, sys, glob, argparse
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def find_bag_dirs(root):
    root = os.path.abspath(root)
    if os.path.isfile(os.path.join(root, "metadata.yaml")):
        return [root]
    out = []
    for meta in glob.glob(os.path.join(root, "**", "metadata.yaml"), recursive=True):
        out.append(os.path.dirname(meta))
    return sorted(set(out))

def topic_types(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}

def read_odom_xy(bag_dir, odom_topic="/odom", downsample=1):
    """Return (xs, ys) from /odom in this bag (optionally downsampled)."""
    so = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader()
    r.open(so, co)

    tmap = topic_types(r)
    if odom_topic not in tmap:
        print(f"[WARN] {os.path.basename(bag_dir)}: {odom_topic} not found.")
        return [], []

    Odom = get_message(tmap[odom_topic])
    xs, ys = [], []
    k = 0
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic != odom_topic:
            continue
        k += 1
        if downsample > 1 and (k % downsample):
            continue
        m = deserialize_message(raw, Odom)
        xs.append(float(m.pose.pose.position.x))
        ys.append(float(m.pose.pose.position.y))
    return xs, ys

def main():
    ap = argparse.ArgumentParser(description="Overlay /odom XY paths from rosbag2 directories.")
    ap.add_argument("root", help="A rosbag2 folder or a directory containing many bags")
    ap.add_argument("--odom", default="/odom", help="Odometry topic (default: /odom)")
    ap.add_argument("--downsample", type=int, default=1, help="Keep 1 of every N odom samples")
    ap.add_argument("--normalize", action="store_true",
                    help="Shift each path so its first point is at (0,0)")
    ap.add_argument("--out", default="", help="Save to image instead of showing")
    args = ap.parse_args()

    bag_dirs = find_bag_dirs(args.root)
    if not bag_dirs:
        print(f"[ERROR] No rosbag2 dirs found under {args.root}")
        sys.exit(2)

    plt.figure(figsize=(11, 11))
    any_plotted = False

    for bag in bag_dirs:
        base = os.path.basename(bag.rstrip("/"))
        xs, ys = read_odom_xy(bag, args.odom, args.downsample)
        if not xs:
            continue
        if args.normalize:
            x0, y0 = xs[0], ys[0]
            xs = [x - x0 for x in xs]
            ys = [y - y0 for y in ys]

        # path
        plt.plot(xs, ys, "-", lw=1.8, alpha=0.95, label=f"{base}")
        # start/end dots
        plt.plot(xs[0], ys[0], "o", ms=5)
        plt.plot(xs[-1], ys[-1], "x", ms=6)
        any_plotted = True

    if not any_plotted:
        print("[ERROR] nothing to plot.")
        sys.exit(3)

    plt.axis("equal"); plt.grid(True, alpha=.3)
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    title = f"Odom overlays ({len(bag_dirs)} bag{'s' if len(bag_dirs)!=1 else ''}) • topic: {args.odom}"
    if args.normalize:
        title += " • normalized origins"
    plt.title(title)
    # plt.legend(fontsize=8, ncol=2)

    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        plt.savefig(args.out, dpi=300, bbox_inches="tight")
        print(f"[OK] saved {args.out}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
