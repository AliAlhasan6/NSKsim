#!/usr/bin/env python3
"""Yaw-rate statistics of odom->base_footprint per robot, from a bag's /tf.

Why (2026-09-08, B1.7): the b16 offline maps are rotated copies of the same
walls. A scan matcher with a bounded angular search re-anchors on odometry
when the robot turns more between scans than the search covers; a TurtleBot3
at 2.84 rad/s turns ~33 deg between 5 Hz scans. This reports how often that
happened, per robot, so the hypothesis is tested rather than argued.

Reads only /tf (storage filter). Yaw rate is taken between consecutive
odom->base samples; per-scan rotation is |rate| x the scan interval.

Usage:
    python3 odom_yaw_rate.py BAG --robots 0 1 2 3 4 [--scan-dt 0.2] [--stamp-min S --stamp-max S]
Expected: about 90 s on the 2.3 GB b16 bag.
"""
import argparse
import math
import sys
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--robots", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--scan-dt", type=float, default=0.2, help="scan interval, s")
    ap.add_argument("--stamp-min", type=float, default=None, help="sim stamp lower bound")
    ap.add_argument("--stamp-max", type=float, default=None, help="sim stamp upper bound")
    ap.add_argument("--search-deg", type=float, default=20.0,
                    help="matcher angular search half-width to compare against; "
                         "the 20.0 default is slam_toolbox's "
                         "coarse_search_angle_offset, 0.349 rad, pinned in "
                         "experiments/nav/slam_robot*.yaml and "
                         "experiments/slam/offline_mapping.yaml.template")
    args = ap.parse_args()

    want = {(f"robot_{n}/odom", f"robot_{n}/base_footprint"): n for n in args.robots}
    series = {n: [] for n in args.robots}          # (stamp, yaw)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))
    t0 = time.monotonic()
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            n = want.get((tr.header.frame_id, tr.child_frame_id))
            if n is None:
                continue
            s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            if args.stamp_min is not None and s < args.stamp_min:
                continue
            if args.stamp_max is not None and s > args.stamp_max:
                continue
            series[n].append((s, yaw_of(tr.transform.rotation)))
    print(f"read /tf in {time.monotonic() - t0:.0f} s")

    print(f"\nyaw-rate statistics (rad/s), per-scan rotation at dt={args.scan_dt} s, "
          f"search +/-{args.search_deg} deg")
    print(f"  {'robot':<8}{'samples':>8}{'span s':>9}{'|rot| tot deg':>14}"
          f"{'p50':>7}{'p90':>7}{'p99':>7}{'max':>7}"
          f"{'>1.0 %t':>9}{'>1.75 %t':>10}{'scan>search':>12}{'sec>search':>11}")
    for n in args.robots:
        ser = series[n]
        if len(ser) < 2:
            print(f"  robot_{n:<2} no samples")
            continue
        ser.sort()
        st = np.array([s for s, _ in ser])
        yw = np.array([y for _, y in ser])
        dt = np.diff(st)
        dy = np.diff(yw)
        dy = (dy + np.pi) % (2 * np.pi) - np.pi          # wrap
        ok = dt > 1e-6
        dt, dy = dt[ok], dy[ok]
        rate = np.abs(dy / dt)
        span = st[-1] - st[0]
        tot = float(np.sum(np.abs(dy)))
        frac1 = float(np.sum(dt[rate > 1.0]) / span * 100.0)
        frac175 = float(np.sum(dt[rate > 1.75]) / span * 100.0)
        per_scan = np.degrees(rate * args.scan_dt)
        over = per_scan > args.search_deg
        n_over = int(np.sum(over))
        sec_over = float(np.sum(dt[over]))
        p50, p90, p99 = np.percentile(rate, [50, 90, 99])
        print(f"  robot_{n:<2}{len(ser):>8}{span:>9.0f}{math.degrees(tot):>14.0f}"
              f"{p50:>7.2f}{p90:>7.2f}{p99:>7.2f}{rate.max():>7.2f}"
              f"{frac1:>9.1f}{frac175:>10.1f}{n_over:>12}{sec_over:>11.1f}")
    print("\n  '>1.0 %t': percent of time turning faster than 1 rad/s; 'scan>search': tf intervals whose")
    print("  rotation scaled to one scan interval exceeds the search half-width; 'sec>search': seconds so.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
