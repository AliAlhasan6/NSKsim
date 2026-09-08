#!/usr/bin/env python3
"""Write a replay-ready copy of a five-robot NSKsim bag for offline SLAM.

Why this exists (2026-09-08, B1.7):
  The b16 bag carries the live slam_toolbox output: /robot_N/map and, inside
  /tf, the five map->odom transforms. Replaying those next to an offline
  slam_toolbox gives tf2 two authorities for one frame pair, and the map->odom
  read back by tf2_echo after replay is a mixture of both. `ros2 bag play`
  can exclude topics, not individual transforms, so the strip is done here.

Keeps:  /clock, /tf_static, /robot_N/scan, /robot_N/odom, and /tf with every
        transform whose child_frame_id ends in '/odom' removed.
Drops:  /robot_N/map, /robot_N/map_metadata, /kg_share, /nsk/convergence.

Timestamps and message order are untouched. Truncation is deliberately NOT
done here -- it stays a --playback-duration decision in the runner, where the
cut is visible in the log of every map produced.

Usage:
    python3 strip_bag_for_offline_slam.py SRC_BAG_DIR DST_BAG_DIR
Expected: one full pass over the bag, roughly the cost of tf_corrected_extents.
"""
import argparse
import os
import sys
import time
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="source bag directory (mcap)")
    ap.add_argument("dst", help="destination bag directory; must not exist")
    ap.add_argument("--robots", type=int, default=5)
    ap.add_argument("--drop-child-suffix", default="/odom",
                    help="drop /tf transforms whose child_frame_id ends with this")
    ap.add_argument("--progress", type=int, default=250_000,
                    help="print progress every N messages read")
    args = ap.parse_args()

    if os.path.exists(args.dst):
        print(f"refusing to overwrite existing bag: {args.dst}", file=sys.stderr)
        return 2

    keep = {"/clock", "/tf", "/tf_static"}
    for n in range(args.robots):
        keep |= {f"/robot_{n}/scan", f"/robot_{n}/odom"}

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.src, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    topics = [t for t in reader.get_all_topics_and_types() if t.name in keep]
    missing = keep - {t.name for t in topics}
    if missing:
        print(f"source bag lacks required topics: {sorted(missing)}", file=sys.stderr)
        return 3
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(keep)))

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.dst, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    for t in topics:
        writer.create_topic(t)

    read = Counter()
    written = Counter()
    dropped_pairs = Counter()          # (parent, child) -> transforms removed
    kept_pairs = Counter()             # (parent, child) -> transforms kept
    tf_rewritten = 0
    tf_emptied = 0
    total = 0
    t0 = time.monotonic()

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        total += 1
        read[topic] += 1

        if topic == "/tf":
            msg = deserialize_message(data, TFMessage)
            keep_tr = []
            changed = False
            for tr in msg.transforms:
                pair = (tr.header.frame_id, tr.child_frame_id)
                if tr.child_frame_id.endswith(args.drop_child_suffix):
                    dropped_pairs[pair] += 1
                    changed = True
                else:
                    kept_pairs[pair] += 1
                    keep_tr.append(tr)
            if changed:
                if not keep_tr:
                    tf_emptied += 1
                    continue                     # nothing left to write
                msg.transforms = keep_tr
                data = serialize_message(msg)
                tf_rewritten += 1

        writer.write(topic, data, stamp)
        written[topic] += 1

        if args.progress and total % args.progress == 0:
            el = time.monotonic() - t0
            print(f"  {total:>9} read  {el:6.0f} s", flush=True)

    del writer                                   # flush and close the bag
    el = time.monotonic() - t0

    print(f"\ndone: {total} messages read in {el:.0f} s -> {args.dst}")
    print("\nper topic (read -> written):")
    for name in sorted(read):
        print(f"  {name:<24}{read[name]:>9} -> {written[name]:>9}")
    print(f"\n/tf messages rewritten: {tf_rewritten}   emptied and skipped: {tf_emptied}")
    print("\ntransforms DROPPED (parent -> child):")
    if not dropped_pairs:
        print("  none -- the source bag carried no matching transforms")
    for (p, c), n in sorted(dropped_pairs.items()):
        print(f"  {p:<24}-> {c:<28}{n:>9}")
    print("\ntransforms KEPT (parent -> child):")
    for (p, c), n in sorted(kept_pairs.items()):
        print(f"  {p:<24}-> {c:<28}{n:>9}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
