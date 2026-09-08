#!/usr/bin/env python3
"""tf_frame_census.py -- what frames exist in the bag, and how often.

WHY THIS EXISTS
---------------
Recovering a loop-closure-corrected pose needs map->odom from /tf, composed with
odom->base from /robot_N/odom.  Both depend on the exact frame_id strings, and
namespaced multi-robot TF has several conventions in the wild:

    map / odom / base_footprint         (no prefix, separate TF trees)
    robot_0/map / robot_0/odom          (slash prefix)
    robot_0_map / robot_0_odom          (underscore prefix)

Guessing wrong means a full pass over 2.35 M messages that silently matches
nothing.  This probe reads only the first N /tf messages plus all of /tf_static
and prints the (parent -> child) pairs it saw with counts.  It is a census, not
an analysis: it answers "what is in here", nothing else.

Deliberately bounded.  --limit caps the /tf messages deserialized, so this runs
in seconds against a 2.5 GB bag rather than minutes.  If a transform is
published rarely it may not appear in the first N; raise --limit if map->odom
is missing from the output.

USAGE (venv, not a ROS shell -- rosbags is pure python)
    cd ~/Desktop/NSKsim
    source venv/bin/activate
    python3 tf_frame_census.py experiments/logs/b16/b16_run1 --limit 20000
"""

import argparse
import pathlib
import sys
from collections import Counter

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=pathlib.Path)
    ap.add_argument("--limit", type=int, default=20000,
                    help="max /tf messages to deserialize (default 20000)")
    args = ap.parse_args()

    pairs = Counter()
    static_pairs = Counter()
    n_tf = 0

    with Reader(args.bag) as reader:
        conns = [c for c in reader.connections if c.topic in ("/tf", "/tf_static")]
        if not conns:
            sys.exit("no /tf or /tf_static connections in this bag")

        print("connections:")
        for c in conns:
            print(f"  {c.topic:<12} {c.msgtype:<28} msgs={c.msgcount}")

        for conn, _ts, raw in reader.messages(connections=conns):
            msg = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            target = static_pairs if conn.topic == "/tf_static" else pairs
            for tr in msg.transforms:
                target[(tr.header.frame_id, tr.child_frame_id)] += 1
            if conn.topic == "/tf":
                n_tf += 1
                if n_tf >= args.limit:
                    break

    print(f"\n/tf_static -- all of it ({sum(static_pairs.values())} transforms)")
    for (p, c), n in sorted(static_pairs.items()):
        print(f"  {n:>8}  {p}  ->  {c}")

    print(f"\n/tf -- first {n_tf} messages ({sum(pairs.values())} transforms)")
    for (p, c), n in sorted(pairs.items(), key=lambda kv: -kv[1]):
        print(f"  {n:>8}  {p}  ->  {c}")

    print("\nlooking for a map->odom pair per robot. if absent, raise --limit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
