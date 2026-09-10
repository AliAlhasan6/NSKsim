#!/usr/bin/env python3
"""Discriminate odometry yaw error from scan-to-pose timestamp skew, against
Gazebo ground truth, for the b16 rotated-map-copy defect.

The b16 offline maps are rotated copies of the same walls.
experiments/analysis/odom_yaw_rate.py (B1.7) ruled out the obvious mechanism:
every robot's odometry yaw rate caps at exactly 1.00 rad/s -- Nav2's configured
limit -- which is roughly 11 degrees per scan. The scan matcher's window is
+/-20 degrees: coarse_search_angle_offset is pinned at 0.349 rad in both
SLAM paths -- experiments/nav/slam_robot{0..4}.yaml online,
experiments/slam/offline_mapping.yaml.template offline -- so the b16 maps
were built under the same window the online run used, and the count below
transfers between them. The count of scans exceeding that window was zero
for all five robots. Fast rotation is therefore not the cause.

Two candidates survive: odometry mis-measured the turn, or a scan carried a
stamp pointing at a different pose than the one it was taken from. The b16 bag
cannot separate them, having no ground truth to measure either against. The
B1.8 run records per-robot Gazebo ground truth on /model/robot_N/pose (added in
662abe2, recorded by experiments/slam/record_run.sh from 9be57ac), which makes
both causes measurable and separable. This script discriminates them by:

  (a) integrate the yaw error against truth, per robot, per degree turned;
  (b) measure the scan-to-pose stamp gap empirically, then sweep a synthetic
      skew through truth to state how much apparent rotation each skew would
      have injected GIVEN THE MOTION ACTUALLY RECORDED.

(b2) is the discriminator: if the empirical gap from (b1) lands where the sweep
says the injected rotation is negligible, skew is not the cause and (a) carries
the defect -- and vice versa.

Reads /robot_N/odom, /robot_N/scan and /model/robot_N/pose in one pass, with
the same rosbag2_py reader odom_yaw_rate.py uses. Header stamps only, never bag
receive time. Pure bag read: no ROS graph, no node, no sim.

Requires ``source /opt/ros/jazzy/setup.bash``.

Every output file lands in --out and nowhere else.

Usage:
    python3 yaw_truth_audit.py BAG --out DIR [--robots 0 1 2 3 4]
                               [--stamp-min S --stamp-max S]

Not benchmarked. Expect it to run slower than odom_yaw_rate.py's ~90 s /tf-only
pass on a bag of the same size: /scan carries a float array per message and is
deserialised here for its header alone.
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")               # headless: written to file, never shown
import matplotlib.pyplot as plt     # noqa: E402
import numpy as np                  # noqa: E402
import rosbag2_py                   # noqa: E402
from geometry_msgs.msg import PoseStamped        # noqa: E402
from nav_msgs.msg import Odometry                # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan            # noqa: E402

# Synthetic skews for the (b2) sweep, milliseconds. Symmetric about zero so a
# sign convention error in (b1) cannot hide behind the sweep.
SWEEP_MS = [-200, -100, -50, -25, 0, 25, 50, 100, 200]

KINDS = {"odom": Odometry, "scan": LaserScan, "pose": PoseStamped}


def die(msg):
    """Fail loudly: no partial output, non-zero exit."""
    print(f"\nFATAL: {msg}\n", file=sys.stderr)
    sys.exit(1)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    """Sim-time header stamp, seconds. Never the bag receive time."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def wrap_to_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def topic_name(kind, n):
    return f"/model/robot_{n}/pose" if kind == "pose" else f"/robot_{n}/{kind}"


def signed_gap(query, ref):
    """For each query stamp, the signed gap to the nearest ref stamp.

    Positive means the query is LATER than the reference it is nearest to.
    ``ref`` must be sorted ascending and non-empty.
    """
    if len(ref) == 1:
        return query - ref[0]
    idx = np.clip(np.searchsorted(ref, query), 1, len(ref) - 1)
    left, right = ref[idx - 1], ref[idx]
    take_left = np.abs(query - left) <= np.abs(query - right)
    return query - np.where(take_left, left, right)


class Truth:
    """Per-robot ground truth, unwrapped once so interpolation cannot wrap.

    yaw is unwrapped BEFORE interpolation: linearly interpolating a wrapped
    series across the +/-pi seam invents a full-circle sweep between two
    samples that are milliseconds and a fraction of a degree apart.
    """

    def __init__(self, t, x, y, yaw_wrapped):
        self.t = t
        self.x = x
        self.y = y
        self.yaw = np.unwrap(yaw_wrapped)
        # Cumulative absolute true rotation, degrees, from the first GT sample.
        # Monotone non-decreasing, so it interpolates linearly like any other
        # channel.
        self.cum_rot = np.concatenate(
            ([0.0], np.cumsum(np.abs(np.degrees(np.diff(self.yaw))))))

    def at(self, t):
        """Linearly interpolate true (x, y, yaw) at arbitrary t.

        np.interp clamps outside the ground-truth span rather than
        extrapolating. Callers restrict their queries to the span (and report
        what that dropped) so the clamp is never load-bearing.
        """
        return (np.interp(t, self.t, self.x),
                np.interp(t, self.t, self.y),
                np.interp(t, self.t, self.yaw))

    def cum_rot_at(self, t):
        return np.interp(t, self.t, self.cum_rot)

    @property
    def span(self):
        return self.t[0], self.t[-1]


def read_bag(path, robots, stamp_min, stamp_max):
    """One pass, all three topics, all robots. Returns raw arrival-order lists.

    Ground truth is the reason this bag exists, so a missing topic is checked
    against the bag's own topic table BEFORE the expensive read -- there is no
    point spending minutes deserialising to then refuse to report.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))

    present = {t.name for t in reader.get_all_topics_and_types()}
    wanted = {}
    missing = []
    for n in robots:
        for kind in KINDS:
            name = topic_name(kind, n)
            if name not in present:
                missing.append(name)
            wanted[name] = (kind, n)
    if missing:
        die("topic(s) not in the bag, so this bag cannot answer the question "
            "it was recorded for:\n         " + "\n         ".join(missing))

    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    raw = {(kind, n): [] for kind in KINDS for n in robots}

    t0 = time.monotonic()
    while reader.has_next():
        name, data, _ = reader.read_next()
        entry = wanted.get(name)
        if entry is None:
            continue
        kind, n = entry
        msg = deserialize_message(data, KINDS[kind])
        s = stamp_of(msg.header)
        if stamp_min is not None and s < stamp_min:
            continue
        if stamp_max is not None and s > stamp_max:
            continue
        if kind == "odom":
            raw[(kind, n)].append((s, yaw_of(msg.pose.pose.orientation)))
        elif kind == "pose":
            p = msg.pose
            raw[(kind, n)].append((s, p.position.x, p.position.y,
                                   yaw_of(p.orientation)))
        else:
            raw[(kind, n)].append((s,))
    print(f"read 3 topics x {len(robots)} robots in {time.monotonic() - t0:.0f} s")
    return raw


def census(raw, robots):
    """Print the count read for every topic, per robot, before analysing.

    Then refuse to analyse a robot whose ground truth, odometry or scans are
    empty. An empty (b) table is not a clean result, it is a silent one.
    """
    print("\nmessages read (header stamps within any --stamp bounds)")
    print(f"  {'robot':<9}{'odom':>9}{'scan':>9}{'pose(GT)':>10}"
          f"{'stamp span s':>14}   non-monotonic")
    empty = []
    nonmono = {}
    for n in robots:
        counts, spans, flags = {}, {}, []
        for kind in KINDS:
            ser = raw[(kind, n)]
            counts[kind] = len(ser)
            if not ser:
                empty.append(topic_name(kind, n))
                continue
            st = np.array([r[0] for r in ser])
            bad = int(np.sum(np.diff(st) <= 0.0))
            nonmono[(kind, n)] = bad
            if bad:
                flags.append(f"{kind}:{bad}")
            spans[kind] = st.max() - st.min()
        span = max(spans.values()) if spans else 0.0
        print(f"  robot_{n:<3}{counts['odom']:>9}{counts['scan']:>9}"
              f"{counts['pose']:>10}{span:>14.1f}   "
              f"{', '.join(flags) if flags else '-'}")
    if any(nonmono.values()):
        total = sum(nonmono.values())
        print(f"  WARNING: {total} non-monotonic header stamp(s); sorting by "
              f"stamp before analysis. Per-topic counts in the last column.")
    if empty:
        die("topic(s) present in the bag but empty over the analysed window:\n"
            "         " + "\n         ".join(sorted(empty)) +
            "\n       Ground truth is the whole point of this audit and odom is "
            "measurement (a);\n       scans are measurement (b). Nothing is "
            "reported from a partial read.")


def sorted_cols(ser, ncol):
    """Arrival-order tuples -> stamp-sorted column arrays."""
    arr = np.array(ser, dtype=float)
    arr = arr[np.argsort(arr[:, 0], kind="stable")]
    return [arr[:, i] for i in range(ncol)]


def analyse(raw, robots):
    """All per-robot numbers. Returns (per-sample rows, scan rows, summaries)."""
    audit_rows, scan_rows, summaries = [], [], []
    max_delta_s = max(abs(d) for d in SWEEP_MS) / 1000.0

    for n in robots:
        gt_t, gt_x, gt_y, gt_yaw = sorted_cols(raw[("pose", n)], 4)
        truth = Truth(gt_t, gt_x, gt_y, gt_yaw)
        lo, hi = truth.span

        # ── measurement (a): odometry yaw error vs truth ──────────────────
        od_t, od_yaw = sorted_cols(raw[("odom", n)], 2)
        inside = (od_t >= lo) & (od_t <= hi)
        n_clamped = int(np.sum(~inside))
        od_t, od_yaw = od_t[inside], od_yaw[inside]
        if od_t.size < 2:
            die(f"robot_{n}: {od_t.size} odom sample(s) fall inside the "
                f"ground-truth span [{lo:.2f}, {hi:.2f}] s. Nothing to compare.")

        # Absolute yaws are not comparable: the odom frame is pinned to the
        # spawn pose, truth to the world origin. Only the CHANGE since the
        # first odom sample is common to both.
        d_odom = np.unwrap(od_yaw) - np.unwrap(od_yaw)[0]
        _, _, yaw_true_at = truth.at(od_t)
        d_true = yaw_true_at - truth.at(od_t[0])[2]
        e = wrap_to_pi(d_odom - d_true)
        e_deg = np.degrees(e)

        cum_rot = truth.cum_rot_at(od_t) - truth.cum_rot_at(od_t[0])
        x_true, y_true, _ = truth.at(od_t)

        window_rot = float(cum_rot[-1])
        per_100 = abs(float(e_deg[-1])) / window_rot * 100.0 if window_rot > 1e-9 else float("nan")

        for i in range(od_t.size):
            audit_rows.append([n, f"{od_t[i]:.9f}", f"{e_deg[i]:.6f}",
                               f"{cum_rot[i]:.6f}", f"{x_true[i]:.6f}",
                               f"{y_true[i]:.6f}",
                               f"{math.degrees(yaw_true_at[i]):.6f}"])

        # ── measurement (b1): empirical scan-to-pose stamp skew ───────────
        (sc_t,) = sorted_cols(raw[("scan", n)], 1)
        gap_pose = signed_gap(sc_t, gt_t) * 1000.0
        gap_odom = signed_gap(sc_t, np.sort(
            np.array([r[0] for r in raw[("odom", n)]], dtype=float))) * 1000.0
        for i in range(sc_t.size):
            scan_rows.append([n, f"{sc_t[i]:.9f}", f"{gap_pose[i]:.4f}",
                              f"{gap_odom[i]:.4f}"])

        # ── measurement (b2): what a given skew would have injected ───────
        # One scan set for every delta, so the columns compare against each
        # other. Scans within max|delta| of either GT edge are excluded: there
        # the clamp in Truth.at would report a zero that is an artefact of the
        # bag ending, not of the robot holding still.
        usable = (sc_t >= lo + max_delta_s) & (sc_t <= hi - max_delta_s)
        sw_t = sc_t[usable]
        if sw_t.size == 0:
            die(f"robot_{n}: no scan lies more than {max_delta_s * 1000:.0f} ms "
                f"inside the ground-truth span; the sweep would report only "
                f"clamp artefacts.")
        base = truth.at(sw_t)[2]
        sweep = {}
        for d in SWEEP_MS:
            shifted = truth.at(sw_t + d / 1000.0)[2]
            inj = np.abs(np.degrees(shifted - base))
            sweep[d] = (float(inj.mean()), float(np.percentile(inj, 95)))

        summaries.append({
            "robot": n,
            "n_odom": od_t.size, "n_scan": sc_t.size, "n_pose": gt_t.size,
            "n_odom_outside_gt": n_clamped,
            "n_scan_in_sweep": int(sw_t.size),
            "gt_span_s": hi - lo,
            "e_final_deg": float(e_deg[-1]),
            "e_absmax_deg": float(np.max(np.abs(e_deg))),
            "e_rms_deg": float(np.sqrt(np.mean(e_deg ** 2))),
            "cum_rot_total_deg": float(truth.cum_rot[-1]),
            "cum_rot_window_deg": window_rot,
            "deg_err_per_100deg": per_100,
            "gap_pose_mean_ms": float(gap_pose.mean()),
            "gap_pose_median_ms": float(np.median(gap_pose)),
            "gap_pose_p95abs_ms": float(np.percentile(np.abs(gap_pose), 95)),
            "gap_pose_maxabs_ms": float(np.max(np.abs(gap_pose))),
            "gap_odom_mean_ms": float(gap_odom.mean()),
            "gap_odom_median_ms": float(np.median(gap_odom)),
            "gap_odom_p95abs_ms": float(np.percentile(np.abs(gap_odom), 95)),
            "gap_odom_maxabs_ms": float(np.max(np.abs(gap_odom))),
            "sweep": sweep,
            "_plot": (od_t, e_deg, cum_rot),
        })
    return audit_rows, scan_rows, summaries


def report(summaries):
    print("\n(a) odometry yaw error vs Gazebo truth, degrees "
          "(change since each robot's first odom sample)")
    print(f"  {'robot':<9}{'samples':>9}{'final e':>10}{'max |e|':>10}"
          f"{'RMS e':>9}{'true rot':>11}{'window rot':>12}{'e/100deg':>10}"
          f"{'odom off GT':>13}")
    for s in summaries:
        print(f"  robot_{s['robot']:<3}{s['n_odom']:>9}{s['e_final_deg']:>10.2f}"
              f"{s['e_absmax_deg']:>10.2f}{s['e_rms_deg']:>9.2f}"
              f"{s['cum_rot_total_deg']:>11.0f}{s['cum_rot_window_deg']:>12.0f}"
              f"{s['deg_err_per_100deg']:>10.2f}{s['n_odom_outside_gt']:>13}")

    print("\n(b1) scan stamp minus nearest other-topic stamp, ms "
          "(positive: the scan is later)")
    print(f"  {'robot':<9}{'scans':>8}{'pose mean':>11}{'median':>9}"
          f"{'p95|.|':>9}{'max|.|':>9}   {'odom mean':>10}{'median':>9}"
          f"{'p95|.|':>9}{'max|.|':>9}")
    for s in summaries:
        print(f"  robot_{s['robot']:<3}{s['n_scan']:>8}"
              f"{s['gap_pose_mean_ms']:>11.2f}{s['gap_pose_median_ms']:>9.2f}"
              f"{s['gap_pose_p95abs_ms']:>9.2f}{s['gap_pose_maxabs_ms']:>9.2f}   "
              f"{s['gap_odom_mean_ms']:>10.2f}{s['gap_odom_median_ms']:>9.2f}"
              f"{s['gap_odom_p95abs_ms']:>9.2f}{s['gap_odom_maxabs_ms']:>9.2f}")

    for label, idx in (("mean", 0), ("p95", 1)):
        print(f"\n(b2) apparent rotation a skew would inject, {label} deg per "
              f"scan, from the motion actually recorded")
        print(f"  {'robot':<9}" + "".join(f"{d:>+9d}" for d in SWEEP_MS) + "   ms")
        for s in summaries:
            print(f"  robot_{s['robot']:<3}"
                  + "".join(f"{s['sweep'][d][idx]:>9.2f}" for d in SWEEP_MS))


def write_csvs(out, args, summaries, audit_rows, scan_rows):
    out.mkdir(parents=True, exist_ok=True)

    audit = out / "yaw_truth_audit.csv"
    with audit.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["robot", "t", "e_deg", "cum_rot_deg", "x_true", "y_true",
                    "yaw_true_deg"])
        w.writerows(audit_rows)

    scans = out / "yaw_truth_scans.csv"
    with scans.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["robot", "t", "gap_pose_ms", "gap_odom_ms"])
        w.writerows(scan_rows)

    cols = ["robot", "n_odom", "n_scan", "n_pose", "n_odom_outside_gt",
            "n_scan_in_sweep", "gt_span_s", "e_final_deg", "e_absmax_deg",
            "e_rms_deg", "cum_rot_total_deg", "cum_rot_window_deg",
            "deg_err_per_100deg", "gap_pose_mean_ms", "gap_pose_median_ms",
            "gap_pose_p95abs_ms", "gap_pose_maxabs_ms", "gap_odom_mean_ms",
            "gap_odom_median_ms", "gap_odom_p95abs_ms", "gap_odom_maxabs_ms"]
    sweep_cols = [f"sweep_{stat}_deg_at_{d:+d}ms"
                  for d in SWEEP_MS for stat in ("mean", "p95")]
    summary = out / "yaw_truth_summary.csv"
    with summary.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["# bag", args.bag])
        w.writerow(["# robots", " ".join(str(n) for n in args.robots)])
        w.writerow(["# stamp_bounds", str(args.stamp_min), str(args.stamp_max)])
        w.writerow(["# time_base", "header stamps (sim time), not bag receive time"])
        w.writerow(["# e_deg", "wrap_to_pi(d_yaw_odom - d_yaw_true) since first odom sample"])
        w.writerow([])
        w.writerow(cols + sweep_cols)
        for s in summaries:
            row = [s[c] if isinstance(s[c], int) else f"{s[c]:.6f}"
                   for c in cols]
            for d in SWEEP_MS:
                row += [f"{s['sweep'][d][0]:.6f}", f"{s['sweep'][d][1]:.6f}"]
            w.writerow(row)

    for p in (audit, scans, summary):
        print(f"wrote {p}")


def write_plots(out, summaries):
    for s in summaries:
        t, e_deg, cum_rot = s["_plot"]
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7))
        fig.suptitle(f"robot_{s['robot']}: odometry yaw error vs Gazebo truth")

        ax1.plot(t - t[0], e_deg, lw=0.8)
        ax1.axhline(0.0, color="k", lw=0.6)
        ax1.set_xlabel("time since first odom sample, s")
        ax1.set_ylabel("yaw error, deg")
        ax1.grid(True, lw=0.3)

        ax2.plot(cum_rot, e_deg, lw=0.8)
        ax2.axhline(0.0, color="k", lw=0.6)
        ax2.set_xlabel("cumulative true rotation, deg")
        ax2.set_ylabel("yaw error, deg")
        ax2.set_title(f"{s['deg_err_per_100deg']:.2f} deg error per 100 deg turned "
                      f"(endpoint)", fontsize=9)
        ax2.grid(True, lw=0.3)

        fig.tight_layout()
        path = out / f"yaw_error_robot{s['robot']}.png"
        fig.savefig(path, dpi=130)
        plt.close(fig)
        print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(
        description="Odometry yaw error and scan-to-pose stamp skew, per robot, "
                    "against Gazebo ground truth. Pure bag read.")
    ap.add_argument("bag")
    ap.add_argument("--robots", type=int, nargs="+", default=[0, 1, 2, 3, 4])
    ap.add_argument("--out", required=True, type=Path,
                    help="output directory; every file is written here")
    ap.add_argument("--stamp-min", type=float, default=None, help="sim stamp lower bound")
    ap.add_argument("--stamp-max", type=float, default=None, help="sim stamp upper bound")
    args = ap.parse_args()

    raw = read_bag(args.bag, args.robots, args.stamp_min, args.stamp_max)
    census(raw, args.robots)
    audit_rows, scan_rows, summaries = analyse(raw, args.robots)
    report(summaries)
    print()
    write_csvs(args.out, args, summaries, audit_rows, scan_rows)
    write_plots(args.out, summaries)

    print("\n  (a) 'e' is the error in yaw CHANGE since the robot's first odom sample, not in")
    print("      absolute yaw: the odom frame starts at the spawn pose, truth at the world")
    print("      origin, so absolute yaws are not comparable. 'true rot' is the total over")
    print("      the whole ground-truth span, 'window rot' the part the odom samples cover,")
    print("      and 'e/100deg' divides |final e| by 'window rot'. 'odom off GT' counts odom")
    print("      samples dropped for falling outside the ground-truth span (no extrapolation).")
    print("  (b1) a systematic nonzero mean is the finding; a mean near zero with a wide p95")
    print("      is jitter, not skew. p95 and max are of |gap|, mean and median are signed.")
    print(f"  (b2) reads as: had every scan been stamped d ms from its true pose, the front")
    print(f"      end would have seen this much spurious rotation per scan. Compare the (b1)")
    print(f"      mean against this row to decide whether skew can account for the defect.")
    print("      Scans within 200 ms of either ground-truth edge are excluded from the sweep.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
