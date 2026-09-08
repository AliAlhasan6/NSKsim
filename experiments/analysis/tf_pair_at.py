#!/usr/bin/env python3
"""Print one parent->child transform from a bag's /tf over time.

Written to compare the live slam_toolbox's map->odom (recorded in b16_run1)
against the offline slam_toolbox's final map->odom for the same robot at the
same sim stamp. Both are estimates of the same odometry drift, so they should
broadly agree if both converged; a large disagreement means at least one did not.

Reads only /tf (storage filter), so it is a partial pass, not a full one.

Usage:
    python3 tf_pair_at.py BAG robot_1/map robot_1/odom --at 1298 --every 300
Expected: about a minute on the 2.3 GB b16 bag.
"""
import argparse
import math
import sys
import time

import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("parent")
    ap.add_argument("child")
    ap.add_argument("--at", type=float, nargs="*", default=[],
                    help="sim stamps (s) at which to print the nearest sample")
    ap.add_argument("--every", type=float, default=300.0,
                    help="also print one sample every this many sim seconds")
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))

    samples = []                       # (stamp_s, x, y, yaw_deg)
    n_tf = 0
    t0 = time.monotonic()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        n_tf += 1
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if tr.header.frame_id != args.parent or tr.child_frame_id != args.child:
                continue
            s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            t = tr.transform.translation
            samples.append((s, t.x, t.y, math.degrees(yaw_of(tr.transform.rotation))))
    el = time.monotonic() - t0

    if not samples:
        print(f"no {args.parent} -> {args.child} in /tf ({n_tf} messages read, {el:.0f} s)")
        return 1
    samples.sort()
    print(f"{args.parent} -> {args.child}: {len(samples)} samples, "
          f"sim stamps {samples[0][0]:.1f} .. {samples[-1][0]:.1f} s "
          f"({n_tf} /tf messages, {el:.0f} s)")

    def show(tag, smp):
        s, x, y, yaw = smp
        print(f"  {tag:<10} stamp {s:9.1f}  x {x:8.3f}  y {y:8.3f}  yaw {yaw:8.2f} deg")

    print("\ncadence:")
    next_mark = samples[0][0]
    for smp in samples:
        if smp[0] >= next_mark:
            show("", smp)
            next_mark = smp[0] + args.every
    show("last", samples[-1])

    if args.at:
        print("\nrequested stamps (nearest sample):")
        for target in args.at:
            best = min(samples, key=lambda smp: abs(smp[0] - target))
            show(f"at {target:g}", best)
    return 0


if __name__ == "__main__":
    sys.exit(main())
