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

Timestamps and message order are untouched unless --start is given.

--start S (bag seconds from the first kept message) writes a SEGMENT: every
message before S is skipped, except /tf_static, whose receive timestamps are
moved to S so the static transforms are present when the segment begins.
Added 2026-09-08: `ros2 bag play --start-offset` skips /tf_static at bag
time 0, and without base_scan->base_footprint slam_toolbox's scan filter
drops every message; a one-second pre-play of /tf_static did not deliver
either. --duration D stops the segment D bag seconds after S.

--travel-gate K (added 2026-10-02) drops robot K's scans that the robot did not
move for. A scan is ADMITTED only if its sensor pose has moved >= 0.01 m or
turned >= 0.5 deg since the LAST ADMITTED scan; the first scan of the segment is
always admitted. Measuring against the last admitted scan rather than the last
scan is what keeps slow creep from being dropped scan after scan.

WHY IT IS HERE AND NOT DOWNSTREAM. Every count that depends on the scan count is
already derived from this bag: scan_queue_size under DETERMINISTIC=true comes from
bag_scan_facts.py --count on it (run_offline_maps.sh:501-507), and
run_b2maps_cuts.sh's drop gate places each drop against --first-stamp of it. Gate
the bag and both follow with no other edit. C4 is upstream of this script and
measures sim seconds on the RAW bag, so it is untouched.

The pose is odom_T_base(t) from /tf, interpolated at the scan's header stamp,
composed with base_T_scan from the two-hop /tf_static chain. map_T_odom is NOT
needed: it is rigid, and a rigid transform cannot change a displacement or a
heading change, so the gate's decisions do not depend on it -- which is why this
needs nothing but the bag it is already reading.

It costs a FIRST PASS, because interpolating a pose at a scan stamp needs the /tf
sample after it, which bag order does not guarantee. Pass 1 iterates the same
topic set as pass 2 -- so the segment's first kept message, and therefore the cut
window, is identical -- and deserializes only /tf, /tf_static and robot K's scans.
A scan whose stamp has no interpolable pose is ADMITTED and counted: the gate may
only drop a scan it has judged.

WITHOUT --travel-gate, pass 1 does not run and pass 2 is byte-for-byte the code
path it was before this option existed.

Usage:
    python3 strip_bag_for_offline_slam.py SRC_BAG_DIR DST_BAG_DIR [--start S] [--duration D]
    python3 strip_bag_for_offline_slam.py SRC DST --start S --duration D --travel-gate 3
Expected: one pass over the bag, roughly the cost of tf_corrected_extents; two
with --travel-gate.
"""
import argparse
import math
import os
import sys
import time
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# The travel gate's thresholds. Measured against the LAST ADMITTED scan.
GATE_M = 0.01
GATE_DEG = 0.5


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def admitted_scans(src, keep, robot, start, duration, drop_child_suffix):
    """Pass 1: which of robot `robot`'s scans the travel gate admits.

    Returns (list of bools in scan order, number unjudgeable). Iterates the same
    topic set pass 2 keeps, so the first kept message -- and with it the cut
    window -- is the same one pass 2 will see.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(keep)))

    scan_topic = f"/robot_{robot}/scan"
    parent, child = f"robot_{robot}/odom", f"robot_{robot}/base_footprint"
    hop_a = (f"robot_{robot}/base_footprint", f"robot_{robot}/base_link")
    hop_b = (f"robot_{robot}/base_link", f"robot_{robot}/base_scan")

    bag_t0 = cut_start = cut_end = None
    tf_t, tf_x, tf_y, tf_a = [], [], [], []
    stamps = []
    static = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if bag_t0 is None:
            bag_t0 = stamp
            if start is not None:
                cut_start = bag_t0 + int(start * 1e9)
                if duration is not None:
                    cut_end = cut_start + int(duration * 1e9)
        if topic == "/tf_static":
            for tr in deserialize_message(data, TFMessage).transforms:
                static[(tr.header.frame_id, tr.child_frame_id)] = (
                    tr.transform.translation.x, tr.transform.translation.y,
                    _yaw(tr.transform.rotation))
            continue
        if cut_start is not None:
            if stamp < cut_start:
                continue
            if cut_end is not None and stamp > cut_end:
                break
        if topic == "/tf":
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.child_frame_id.endswith(drop_child_suffix):
                    continue            # pass 2 removes these; the gate cannot use them
                if (tr.header.frame_id, tr.child_frame_id) == (parent, child):
                    tf_t.append(tr.header.stamp.sec
                                + tr.header.stamp.nanosec * 1e-9)
                    tf_x.append(tr.transform.translation.x)
                    tf_y.append(tr.transform.translation.y)
                    tf_a.append(_yaw(tr.transform.rotation))
        elif topic == scan_topic:
            m = deserialize_message(data, LaserScan)
            stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    if not stamps:
        return [], 0
    for hop in (hop_a, hop_b):
        if hop not in static:
            print(f"--travel-gate: /tf_static has no {hop[0]} -> {hop[1]}, so "
                  "base_T_scan cannot be composed", file=sys.stderr)
            sys.exit(2)
    bx, by, ba = 0.0, 0.0, 0.0
    for hx, hy, ha in (static[hop_a], static[hop_b]):
        bx, by = (bx + math.cos(ba) * hx - math.sin(ba) * hy,
                  by + math.sin(ba) * hx + math.cos(ba) * hy)
        ba += ha
    if not tf_t:
        print(f"--travel-gate: no {parent} -> {child} in /tf", file=sys.stderr)
        sys.exit(2)

    order = sorted(range(len(tf_t)), key=lambda i: tf_t[i])
    tf_t = [tf_t[i] for i in order]
    tf_x = [tf_x[i] for i in order]
    tf_y = [tf_y[i] for i in order]
    tf_a = [tf_a[i] for i in order]

    admit, unjudged = [], 0
    last = None
    turn = math.radians(GATE_DEG)
    import bisect
    for t in stamps:
        if t < tf_t[0] - 1e-9 or t > tf_t[-1] + 1e-9:
            admit.append(True)          # cannot be judged, so not dropped
            unjudged += 1
            continue
        j = min(max(bisect.bisect_right(tf_t, t) - 1, 0), len(tf_t) - 2)
        span = tf_t[j + 1] - tf_t[j]
        f = 0.0 if span <= 0 else min(max((t - tf_t[j]) / span, 0.0), 1.0)
        x = tf_x[j] + f * (tf_x[j + 1] - tf_x[j])
        y = tf_y[j] + f * (tf_y[j + 1] - tf_y[j])
        da = (tf_a[j + 1] - tf_a[j] + math.pi) % (2.0 * math.pi) - math.pi
        a = tf_a[j] + f * da
        sx = x + math.cos(a) * bx - math.sin(a) * by
        sy = y + math.sin(a) * bx + math.cos(a) * by
        sa = a + ba
        if last is None:
            admit.append(True)
            last = (sx, sy, sa)
            continue
        moved = math.hypot(sx - last[0], sy - last[1])
        turned = abs((sa - last[2] + math.pi) % (2.0 * math.pi) - math.pi)
        if moved >= GATE_M or turned >= turn:
            admit.append(True)
            last = (sx, sy, sa)
        else:
            admit.append(False)
    return admit, unjudged


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
    ap.add_argument("--start", type=float, default=None,
                    help="segment start, bag seconds from the first kept message; "
                         "/tf_static is re-timestamped to this point")
    ap.add_argument("--duration", type=float, default=None,
                    help="segment length in bag seconds (requires --start)")
    ap.add_argument("--travel-gate", type=int, default=None, metavar="K",
                    help="drop robot K's scans the robot did not move for: "
                         f"admit only if the sensor pose moved >= {GATE_M} m or "
                         f"turned >= {GATE_DEG} deg since the last ADMITTED "
                         "scan. Costs a first pass. Off by default, and with it "
                         "off this script is byte-for-byte what it was")
    args = ap.parse_args()
    if args.duration is not None and args.start is None:
        ap.error("--duration requires --start")
    if args.travel_gate is not None and not 0 <= args.travel_gate < args.robots:
        ap.error(f"--travel-gate must name one of the {args.robots} robots")

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

    # Pass 1, only when gating: decide which of robot K's scans are admitted.
    gate_admit, gate_unjudged, gate_topic = None, 0, None
    if args.travel_gate is not None:
        gate_topic = f"/robot_{args.travel_gate}/scan"
        g0 = time.monotonic()
        gate_admit, gate_unjudged = admitted_scans(
            args.src, keep, args.travel_gate, args.start, args.duration,
            args.drop_child_suffix)
        n_adm = sum(gate_admit)
        print(f"--travel-gate {args.travel_gate}: {n_adm} of {len(gate_admit)} "
              f"{gate_topic} scans admitted "
              f"({1 - n_adm / max(len(gate_admit), 1):.1%} dropped; "
              f"{gate_unjudged} had no pose and were admitted unjudged) "
              f"in {time.monotonic() - g0:.0f} s", flush=True)

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
    skipped_before_start = 0
    statics_moved = 0
    bag_t0 = None                                # ns, first kept message
    cut_start = None                             # ns
    cut_end = None                               # ns
    static_buffer = []                           # /tf_static seen before start
    gate_seen = 0                                # robot K's scans met in pass 2
    gate_dropped = 0
    t0 = time.monotonic()

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        total += 1
        read[topic] += 1

        if bag_t0 is None:
            bag_t0 = stamp
            if args.start is not None:
                cut_start = bag_t0 + int(args.start * 1e9)
                if args.duration is not None:
                    cut_end = cut_start + int(args.duration * 1e9)

        if cut_start is not None:
            if stamp < cut_start:
                if topic == "/tf_static":
                    static_buffer.append(data)
                skipped_before_start += 1
                continue
            if static_buffer:                    # first message at/after start
                for sdata in static_buffer:
                    writer.write("/tf_static", sdata, cut_start)
                    written["/tf_static"] += 1
                    statics_moved += 1
                static_buffer = []
            if cut_end is not None and stamp > cut_end:
                break

        if topic == gate_topic:
            # The n-th scan of this topic inside the window, in the same order
            # pass 1 counted them, so the index is the same scan.
            i = gate_seen
            gate_seen += 1
            if i < len(gate_admit) and not gate_admit[i]:
                gate_dropped += 1
                continue

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
    if cut_start is not None:
        print(f"segment: start {args.start:g} s"
              + (f", duration {args.duration:g} s" if args.duration is not None else "")
              + f"; skipped {skipped_before_start} messages before start; "
              f"{statics_moved} /tf_static messages moved to the start")
        if statics_moved == 0:
            print("  WARNING: no /tf_static was seen before the start -- the "
                  "segment has no static transforms")
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
