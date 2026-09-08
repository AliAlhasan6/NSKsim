#!/usr/bin/env python3
"""tf_corrected_extents_v2.py -- corrected extents with loop-closure filtering.

WHAT CHANGED FROM v1
--------------------
1. PATH LENGTH IS FILTERED.  Composing map_T_odom o odom_T_base gives a pose
   that TELEPORTS whenever slam_toolbox closes a loop: map->odom shifts
   discontinuously, and summing consecutive displacements charges that
   correction to the robot as travel.  v1's path lengths (and therefore its
   p/diag ratios) were upper bounds for this reason.

   A TurtleBot3 Burger is limited to 0.22 m/s.  At the observed odom->base
   broadcast rate (~20 Hz) a physically possible step is well under 0.02 m.
   Anything larger is a correction, not motion.

   Rather than assert one threshold, this reports the displacement
   DISTRIBUTION and computes the path at SEVERAL thresholds at once, so the
   cut point is chosen from the data and its sensitivity is visible.  If the
   path barely changes between 0.05 m and 0.50 m, the choice does not matter;
   if it changes a lot, that is itself the finding.

2. PER-ROBOT STOP TIMES.  v1 used one global cut.  Measured from the explorer
   logs, robots 1-4 stopped at 00:32:43 in a simultaneous rejection cascade
   while robot0 continued to 01:47:37.  A single cut therefore truncates robot0
   and barely touches the rest.  Stops are now per robot, defaulting to the
   measured values, overridable with --stop.

3. LOSS ACCOUNTING.  For each robot: how many samples were rejected as jumps,
   what distance they carried, and what fraction of the unfiltered path that
   was.

STILL NOT DONE
--------------
No common world frame.  Each map frame is built on that robot's initial pose,
so '%world' stays indicative and inter-robot overlap still needs B1.7.

USAGE (venv, not a ROS shell)
    cd ~/Desktop/NSKsim
    source venv/bin/activate
    python3 tf_corrected_extents_v2.py experiments/logs/b16/b16_run1 \
        --csv experiments/logs/b16/corrected_extents_v2.csv

Expected wall time: same as v1, several minutes.  Ceiling 20 min.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
from datetime import datetime

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)
WORLD_AREA_M2 = 400.0

# Measured from the b16 explorer logs, not assumed. Robots 1-4 all stopped
# within four seconds of each other on zero-duration FAILED goals; robot0
# continued with real goals for a further 76 minutes.
DEFAULT_STOPS = {
    0: "2026-09-07T01:47:37",
    1: "2026-09-07T00:32:43",
    2: "2026-09-07T00:32:43",
    3: "2026-09-07T00:32:43",
    4: "2026-09-07T00:32:43",
}

# Physically possible step is ~0.011 m at 0.22 m/s and 20 Hz. The spread below
# brackets that by more than an order of magnitude in both directions.
THRESHOLDS = [0.02, 0.05, 0.10, 0.25, 0.50, float("inf")]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw)


def xyyaw(tr):
    t = tr.transform.translation
    return (t.x, t.y, yaw_of(tr.transform.rotation))


def pct(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = min(len(sorted_vals) - 1, max(0, int(round(p / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[k]


class Track:
    """Extents, plus path length at every threshold, in one pass."""

    def __init__(self):
        self.n = 0
        self.xmin = self.ymin = float("inf")
        self.xmax = self.ymax = float("-inf")
        self.paths = {t: 0.0 for t in THRESHOLDS}
        self.disps = []
        self._last = None

    def add(self, x, y):
        self.n += 1
        self.xmin = min(self.xmin, x); self.xmax = max(self.xmax, x)
        self.ymin = min(self.ymin, y); self.ymax = max(self.ymax, y)
        if self._last is not None:
            d = math.hypot(x - self._last[0], y - self._last[1])
            self.disps.append(d)
            for t in THRESHOLDS:
                if d <= t:
                    self.paths[t] += d
        self._last = (x, y)

    def summary(self, thr):
        if self.n < 2:
            return None
        w = self.xmax - self.xmin
        h = self.ymax - self.ymin
        diag = math.hypot(w, h)
        path = self.paths[thr]
        full = self.paths[float("inf")]
        n_jump = sum(1 for d in self.disps if d > thr)
        d_jump = full - path
        return {
            "n": self.n, "w": w, "h": h, "area": w * h,
            "pct": 100.0 * w * h / WORLD_AREA_M2,
            "path": path, "diag": diag,
            "ratio": (path / diag) if diag > 1e-9 else float("nan"),
            "unfiltered_path": full,
            "jumps": n_jump,
            "jump_dist": d_jump,
            "jump_frac": (100.0 * d_jump / full) if full > 1e-9 else 0.0,
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=pathlib.Path)
    ap.add_argument("--robots", type=int, default=5)
    ap.add_argument("--stop", action="append", default=[],
                    metavar="ID=ISO",
                    help="override a robot's stop time, e.g. --stop 0=2026-09-07T01:47:37")
    ap.add_argument("--report-threshold", type=float, default=0.05,
                    help="threshold used for the headline table (default 0.05 m)")
    ap.add_argument("--csv", type=pathlib.Path, default=None)
    args = ap.parse_args()

    stops = dict(DEFAULT_STOPS)
    for spec in args.stop:
        rid, _, iso = spec.partition("=")
        stops[int(rid)] = iso
    stop_ns = {int(r): int(datetime.fromisoformat(v).astimezone().timestamp() * 1e9)
               for r, v in stops.items()}

    if args.report_threshold not in THRESHOLDS:
        THRESHOLDS.append(args.report_threshold)
        THRESHOLDS.sort()

    m_T_o = {i: None for i in range(args.robots)}
    corrected = {i: Track() for i in range(args.robots)}
    raw = {i: Track() for i in range(args.robots)}
    skipped = {i: 0 for i in range(args.robots)}
    n_msgs = 0

    with Reader(args.bag) as reader:
        conns = [c for c in reader.connections if c.topic == "/tf"]
        if not conns:
            sys.exit("no /tf connection in this bag")
        total = sum(c.msgcount for c in conns)
        print(f"reading {total} /tf messages ...", flush=True)

        for conn, ts, rawbytes in reader.messages(connections=conns):
            msg = TYPESTORE.deserialize_cdr(rawbytes, conn.msgtype)
            n_msgs += 1
            if n_msgs % 250000 == 0:
                print(f"  {n_msgs}/{total}", flush=True)

            for tr in msg.transforms:
                parent, child = tr.header.frame_id, tr.child_frame_id
                if not parent.startswith("robot_"):
                    continue
                try:
                    rid = int(parent.split("/", 1)[0].split("_", 1)[1])
                except (ValueError, IndexError):
                    continue
                if rid >= args.robots or ts >= stop_ns.get(rid, 1 << 62):
                    continue

                if parent.endswith("/map") and child.endswith("/odom"):
                    m_T_o[rid] = xyyaw(tr)
                elif parent.endswith("/odom") and child.endswith("/base_footprint"):
                    o_T_b = xyyaw(tr)
                    raw[rid].add(o_T_b[0], o_T_b[1])
                    if m_T_o[rid] is None:
                        skipped[rid] += 1
                        continue
                    x, y, _ = compose(m_T_o[rid], o_T_b)
                    corrected[rid].add(x, y)

    thr = args.report_threshold
    print("\nstop times used (measured from explorer logs):")
    for rid in range(args.robots):
        print(f"  robot{rid}: {stops[rid]}")

    print("\n=== per-sample displacement distribution, CORRECTED (metres) ===")
    print(f"  {'robot':<7}{'p50':>9}{'p90':>9}{'p99':>9}{'p99.9':>9}{'max':>9}")
    for rid in range(args.robots):
        d = sorted(corrected[rid].disps)
        print(f"  robot{rid:<2}{pct(d,50):>9.4f}{pct(d,90):>9.4f}"
              f"{pct(d,99):>9.4f}{pct(d,99.9):>9.4f}"
              f"{(d[-1] if d else float('nan')):>9.4f}")

    print("\n=== path length vs threshold, CORRECTED (metres) ===")
    hdr = "".join(f"{('inf' if t == float('inf') else f'{t:.3f}'):>10}"
                  for t in THRESHOLDS)
    print(f"  {'robot':<7}{hdr}")
    for rid in range(args.robots):
        row = "".join(f"{corrected[rid].paths[t]:>10.1f}" for t in THRESHOLDS)
        print(f"  robot{rid:<2}{row}")

    rows = []
    for label, table in (("CORRECTED", corrected), ("UNCORRECTED odom", raw)):
        print(f"\n=== {label}, filtered at {thr:.3f} m ===")
        print(f"  {'robot':<7}{'n':>8}{'w x h':>17}{'%world*':>9}"
              f"{'path':>9}{'p/diag':>8}{'jumps':>8}{'lost m':>9}{'lost %':>8}")
        for rid in range(args.robots):
            s = table[rid].summary(thr)
            if s is None:
                print(f"  robot{rid:<2}{'--':>8}  (insufficient samples)")
                continue
            print(f"  robot{rid:<2}{s['n']:>8}{s['w']:>9.2f} x{s['h']:>6.2f}"
                  f"{s['pct']:>8.1f}%{s['path']:>9.1f}{s['ratio']:>8.1f}"
                  f"{s['jumps']:>8}{s['jump_dist']:>9.1f}{s['jump_frac']:>7.1f}%")
            rows.append({"window": label, "robot": rid, "threshold": thr, **s})

    print("\n* %world indicative only -- rotated map frames. Overlap needs B1.7.")
    for rid, k in skipped.items():
        if k:
            print(f"  robot{rid}: {k} poses before first map->odom, excluded")

    if args.csv and rows:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
