#!/usr/bin/env python3
"""check_cut_reference.py -- does the rewriter keep the strip script's time zero?

check_run_bag.py's C4 prints `--start` as an offset from the first message that
strip_bag_for_offline_slam.py keeps, and it measures that on the RAW run bag.
But the strip script is run on the REWRITER's output, and it sets its own zero
there. The cut lands where C4 says only if both bags have the same first kept
message.

This script finds each bag's first kept message the way the strip script does
(reader filtered to the keep set, first message READ -- not the smallest
timestamp), compares the two, and exits 0 on a match, 1 on a mismatch, and 2 on
an input error.

Usage (ROS sourced, no simulator, a few seconds):
    python3 check_cut_reference.py <raw_bag_dir> <rewritten_bag_dir>

Show that it can fail before trusting a PASS:
    python3 check_cut_reference.py RAW RAW             -> PASS, difference 0 ns
    python3 check_cut_reference.py RAW <another run>   -> FAIL
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

# Mirror of the strip script's keep set (strip_bag_for_offline_slam.py:63-65).
# Keep in step with it, and with check_run_bag.py's strip_keep_topics().
KEEP = ["/clock", "/tf", "/tf_static"] + [
    f"/robot_{n}/{kind}" for n in range(5) for kind in ("scan", "odom")
]


def first_kept(messages, keep):
    """From an iterable of (topic, t_ns) in read order, return
    (t0, topic0, first_per_topic).

    Mirrors the strip script: time zero is the first kept message READ.
    Stops as soon as every topic in `keep` has been seen, so it never reads
    the whole bag -- `keep` must hold only topics the bag actually contains.
    """
    keep = set(keep)
    t0 = topic0 = None
    first = {}
    for topic, t in messages:
        if topic not in keep:
            continue
        if t0 is None:
            t0, topic0 = t, topic
        first.setdefault(topic, t)
        if len(first) == len(keep):
            break
    return t0, topic0, first


def verdict(raw, rew, tol_ns=0):
    """Compare two first_kept() results plus their kept-topic sets.

    raw, rew: dicts with keys t0, topic0, present (set of kept topics).
    Returns (ok, reasons). ok is True only if both bags keep the same topics
    and their time zeros agree within tol_ns.
    """
    reasons = []
    if raw["present"] != rew["present"]:
        only_raw = sorted(raw["present"] - rew["present"])
        only_rew = sorted(rew["present"] - raw["present"])
        reasons.append(
            f"kept-topic sets differ (only raw: {only_raw}; only rewritten: {only_rew})"
        )
    if raw["t0"] is None or rew["t0"] is None:
        reasons.append("a bag has no kept messages at all")
    else:
        diff = rew["t0"] - raw["t0"]
        if abs(diff) > tol_ns:
            reasons.append(
                f"time zero differs by {diff} ns ({diff / 1e9:+.6f} s); "
                f"raw starts on {raw['topic0']}, rewritten on {rew['topic0']}"
            )
    return (not reasons), reasons


def read_metadata(bag_dir):
    """Return (storage_id, starting_time_ns, {topic: message_count})."""
    meta = Path(bag_dir) / "metadata.yaml"
    if not meta.is_file():
        raise FileNotFoundError(
            f"{bag_dir}: no metadata.yaml -- not a bag, or the recorder "
            "did not shut down cleanly"
        )
    info = yaml.safe_load(meta.read_text())["rosbag2_bagfile_information"]
    counts = {
        e["topic_metadata"]["name"]: int(e["message_count"])
        for e in info["topics_with_message_count"]
    }
    start = int(info["starting_time"]["nanoseconds_since_epoch"])
    return info["storage_identifier"], start, counts


def read_order(bag_dir, storage_id, topics):
    """Yield (topic, t_ns) in the order rosbag2's SequentialReader returns them."""
    import rosbag2_py  # function-scoped, as in bag_overlap.py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, _data, t = reader.read_next()
        yield topic, t


def inspect(bag_dir):
    storage_id, start, counts = read_metadata(bag_dir)
    present = {t for t in KEEP if counts.get(t, 0) > 0}
    t0, topic0, first = first_kept(read_order(bag_dir, storage_id, present), present)
    return {
        "dir": str(bag_dir),
        "start": start,
        "present": present,
        "t0": t0,
        "topic0": topic0,
        "first": first,
    }


def report(label, r):
    print(f"{label}: {r['dir']}")
    print(f"  kept topics present   {len(r['present'])} of {len(KEEP)}")
    if r["t0"] is None:
        print("  time zero             none")
        return
    gap = (r["t0"] - r["start"]) / 1e9
    print(f"  time zero             {r['t0']} ns, on {r['topic0']}")
    print(f"  after metadata start  {gap:.6f} s")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("raw", help="raw run bag directory (what C4 measured)")
    ap.add_argument("rewritten",
                    help="rewriter output bag directory (what gets stripped)")
    ap.add_argument(
        "--tol-ms",
        type=float,
        default=0.0,
        help="allowed time-zero difference in ms (default 0: must match exactly)",
    )
    args = ap.parse_args(argv)

    try:
        raw = inspect(args.raw)
        rew = inspect(args.rewritten)
    except (FileNotFoundError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    report("raw", raw)
    report("rewritten", rew)

    print("  per-topic first message, rewritten minus raw (informational):")
    for t in KEEP:
        a, b = raw["first"].get(t), rew["first"].get(t)
        if a is None or b is None:
            print(f"    {t:<22} missing in {'raw' if a is None else 'rewritten'}")
        else:
            print(f"    {t:<22} {(b - a) / 1e6:+.3f} ms")

    ok, reasons = verdict(raw, rew, int(round(args.tol_ms * 1e6)))
    if ok:
        print("PASS: same time zero, so C4's --start applies to the "
              "rewritten bag unchanged.")
        return 0
    for r in reasons:
        print(f"FAIL: {r}")
    print("Do not cut with C4's --start until this is explained.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
