#!/usr/bin/env python3
"""rate_profile.py v2 -- rotation error of wheel odometry against Gazebo
ground truth, read straight from a recorded bag. No simulator required.

v2 adds the three things v1's output made necessary:

  * SPEED TABLE   the same signed excess binned by true linear speed, taken
                  from ground-truth positions, not from odometry.
  * RATE x SPEED  the cross-tab, which separates in-place spinning from
                  driving-while-turning at equal turn rate.
  * EPISODES      fixed-length chunks ranked by |excess|, and how much of
                  the run's error the worst chunks carry. A uniform scale
                  error puts ~10% in the worst decile; an episodic one puts
                  most of it there.

v1's rate table, windows and reproduction check are unchanged.

SIGN CONVENTION, stated once and used everywhere:
  angles are signed, positive counter-clockwise; excess = d_odom - d_truth.
  With a net-negative (clockwise) run, a NEGATIVE excess means odometry
  reports MORE rotation than the robot actually performed.

DENOMINATOR, stated once:
  pct = 100 * sum(excess) / sum(|d_truth|)  -- absolute true rotation, NOT
  net. b18_run2's net/absolute is 0.58 and b18_run3's is 0.75, so the two
  runs are not comparable on net. Both totals are printed.

Only |d_truth| is ever summed in absolute value. |d_odom| is not, because
summing odometry increments in absolute value over ~10^5 samples picks up a
noise floor that has already produced three wrong answers in this project.

Usage:
  python3 rate_profile.py --bag experiments/logs/b18/b18_run2 \
      --robots 0 --expect run2 --episode 10
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# ---------------------------------------------------------------- constants

# Every edge of the b18_run2 table published in HANDOFF_2026-09-12
# (2, 5, 10, 20, 30, 45, 55, 200) is present here, so the fine rows
# aggregate back onto the old eight exactly.
FINE_EDGES = [0.0, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 45.0, 50.0, 55.0, 60.0,
              200.0, float("inf")]
COARSE_EDGES = [0.0, 2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 55.0, 200.0,
                float("inf")]

# TurtleBot3 Burger tops out at 0.22 m/s. The first edge separates "not
# translating at all" from "creeping".
SPEED_EDGES = [0.0, 0.005, 0.02, 0.05, 0.10, 0.15, 0.22, float("inf")]

# run2 is the independent check: its four numbers come from the published
# handoff, not from this script. run3 is a regression check: its numbers are
# this script's own v1 output, so it only catches edits that change results.
EXPECTED = {
    "run2": {
        "robot": 0,
        "source": "published HANDOFF_2026-09-12 sections 5 and 6",
        "abs_truth_deg": (51215.2, 0.01),
        "net_truth_deg": (-29852.8, 0.01),
        "net_odom_deg": (-30485.1, 0.01),
        "excess_deg": (-630.6, 0.02),   # -628.9 by increments, -632.3 direct
    },
    "run3": {
        "robot": 0,
        "source": "this script's v1 output, 2026-09-12 (regression check only)",
        "abs_truth_deg": (19823.2, 0.01),
        "net_truth_deg": (-14762.3, 0.01),
        "net_odom_deg": (-14721.3, 0.01),
        "excess_deg": (41.0, 0.05),
    },
}


# ------------------------------------------------------------------ reading

def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pose_series(bag_uri, topics):
    """Return {topic: (t, yaw, x, y)} using header stamps.

    Header stamps are sim time on every topic recorded with use_sim_time,
    which is what both b18 bags were. Bag receive time is NOT used.
    """
    from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions,
                            StorageFilter)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_uri, storage_id="mcap"),
                ConverterOptions("", ""))

    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in available]
    if missing:
        sys.exit("ABORT: %s has no %s" % (bag_uri, ", ".join(missing)))

    msg_types = {t: get_message(available[t]) for t in topics}
    try:
        reader.set_filter(StorageFilter(topics=list(topics)))
    except Exception as exc:                      # noqa: BLE001
        print("note: storage filter unavailable (%s), reading all topics" % exc)

    acc = {t: ([], [], [], []) for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        stamp = msg.header.stamp
        acc[topic][0].append(stamp.sec + stamp.nanosec * 1e-9)
        acc[topic][1].append(_yaw_from_quat(pose.orientation))
        acc[topic][2].append(pose.position.x)
        acc[topic][3].append(pose.position.y)

    return {t: tuple(np.asarray(c, dtype=float) for c in cols)
            for t, cols in acc.items()}


def clean(t, yaw, x, y):
    """Sort by time, drop non-increasing stamps, unwrap yaw."""
    order = np.argsort(t, kind="stable")
    t, yaw, x, y = t[order], yaw[order], x[order], y[order]
    keep = np.concatenate(([True], np.diff(t) > 0.0))
    return t[keep], np.unwrap(yaw[keep]), x[keep], y[keep]


# ----------------------------------------------------------------- analysis

def increments(t_truth, y_truth, t_odom, y_odom):
    """Interpolate truth onto odom sample times; return per-interval deltas.

    Both series are unwrapped before interpolation -- interpolating wrapped
    yaw across the +/-pi seam is silently wrong.
    """
    lo, hi = t_truth[0], t_truth[-1]
    mask = (t_odom >= lo) & (t_odom <= hi)
    t = t_odom[mask]
    o = y_odom[mask]
    if t.size < 3:
        sys.exit("ABORT: fewer than 3 odom samples inside the truth interval")
    tr = np.interp(t, t_truth, y_truth)

    dt = np.diff(t)
    d_odom = np.degrees(np.diff(o))
    d_truth = np.degrees(np.diff(tr))
    t_mid = 0.5 * (t[:-1] + t[1:])
    good = dt > 0.0
    return t_mid[good], dt[good], d_truth[good], d_odom[good]


def truth_speed_at(t_tr, x, y, t_query):
    """Linear speed from ground-truth positions, interpolated to t_query.

    Speed is differenced on the 20 Hz truth grid and then interpolated, not
    the other way round: differencing positions already interpolated onto
    the 50 Hz odom grid produces a staircase.
    """
    dt = np.diff(t_tr)
    v = np.hypot(np.diff(x), np.diff(y)) / dt
    t_v = 0.5 * (t_tr[:-1] + t_tr[1:])
    return np.interp(t_query, t_v, v)


def totals(dt, d_truth, d_odom):
    abs_truth = float(np.sum(np.abs(d_truth)))
    net_truth = float(np.sum(d_truth))
    net_odom = float(np.sum(d_odom))
    excess = net_odom - net_truth
    pct = 100.0 * excess / abs_truth if abs_truth > 0 else float("nan")
    return {
        "time_s": float(np.sum(dt)),
        "abs_truth_deg": abs_truth,
        "net_truth_deg": net_truth,
        "net_odom_deg": net_odom,
        "excess_deg": excess,
        "pct_of_abs": pct,
    }


def binned_table(key, edges, dt, d_truth, d_odom):
    rows = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (key >= lo) & (key < hi)
        if not np.any(sel):
            rows.append((lo, hi, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan")))
            continue
        tot = totals(dt[sel], d_truth[sel], d_odom[sel])
        rows.append((lo, hi, tot["time_s"], tot["abs_truth_deg"],
                     tot["net_truth_deg"], tot["net_odom_deg"],
                     tot["excess_deg"], tot["pct_of_abs"]))
    return rows


def print_table(title, unit, rows):
    print("\n%s" % title)
    print("%-12s %9s %12s %12s %12s %10s %9s"
          % (unit, "time s", "|truth| deg", "net truth", "net odom",
             "excess", "pct"))
    for lo, hi, t_s, abs_t, net_t, net_o, exc, pct in rows:
        hi_s = "inf" if hi == float("inf") else ("%g" % hi)
        print("%-12s %9.1f %12.1f %12.1f %12.1f %10.1f %8.3f%%"
              % ("%g-%s" % (lo, hi_s), t_s, abs_t, net_t, net_o, exc,
                 pct if not math.isnan(pct) else 0.0))


def print_crosstab(rate, speed, dt, d_truth, d_odom):
    """Rows are turn-rate bins, columns are true linear-speed bins."""
    r_edges, s_edges = COARSE_EDGES, SPEED_EDGES
    headers = ["%g-%s" % (lo, "inf" if hi == float("inf") else "%g" % hi)
               for lo, hi in zip(s_edges[:-1], s_edges[1:])]

    for what in ("abs_truth_deg", "pct_of_abs"):
        print("\nRATE x SPEED, cell = %s   (rows deg/s, cols m/s)"
              % ("absolute true rotation, deg" if what == "abs_truth_deg"
                 else "excess / |truth|, %"))
        print("%-11s" % "rate\\speed" + "".join("%11s" % h for h in headers))
        for rlo, rhi in zip(r_edges[:-1], r_edges[1:]):
            rsel = (rate >= rlo) & (rate < rhi)
            cells = []
            for slo, shi in zip(s_edges[:-1], s_edges[1:]):
                sel = rsel & (speed >= slo) & (speed < shi)
                if not np.any(sel):
                    cells.append("%11s" % "-")
                    continue
                tot = totals(dt[sel], d_truth[sel], d_odom[sel])
                if what == "abs_truth_deg":
                    cells.append("%11.1f" % tot["abs_truth_deg"])
                elif tot["abs_truth_deg"] < 5.0:
                    # A percentage over a handful of degrees is noise, not a
                    # measurement. Say so rather than printing it.
                    cells.append("%11s" % "thin")
                else:
                    cells.append("%10.2f%%" % tot["pct_of_abs"])
            rhi_s = "inf" if rhi == float("inf") else "%g" % rhi
            print("%-11s" % ("%g-%s" % (rlo, rhi_s)) + "".join(cells))


def print_episodes(t_mid, dt, d_truth, d_odom, speed, chunk_s, top_n=12):
    t0 = t_mid[0]
    idx = np.floor((t_mid - t0) / chunk_s).astype(int)
    rec = []
    for k in range(int(idx.max()) + 1):
        sel = idx == k
        if not np.any(sel):
            continue
        tot = totals(dt[sel], d_truth[sel], d_odom[sel])
        rec.append((t0 + k * chunk_s, tot["abs_truth_deg"], tot["excess_deg"],
                    float(np.mean(speed[sel])),
                    float(np.mean(np.abs(d_truth[sel]) / dt[sel]))))
    if not rec:
        return

    arr = np.array([r[2] for r in rec])
    total_exc = float(np.sum(arr))
    total_abs = float(np.sum(np.abs(arr)))
    ranked = sorted(rec, key=lambda r: -abs(r[2]))

    print("\nEPISODES, %g s chunks (%d chunks)" % (chunk_s, len(rec)))
    print("%10s %12s %11s %10s %11s"
          % ("t_sim s", "|truth| deg", "excess deg", "speed m/s",
             "rate deg/s"))
    for r in ranked[:top_n]:
        print("%10.1f %12.1f %11.2f %10.4f %11.1f" % r)

    k = max(1, len(rec) // 10)
    worst = ranked[:k]
    share_abs = 100.0 * sum(abs(r[2]) for r in worst) / max(total_abs, 1e-9)
    share_net = (100.0 * sum(r[2] for r in worst) / total_exc
                 if abs(total_exc) > 1e-9 else float("nan"))
    print("  worst %d chunks (10%% of the run) hold %.1f%% of the summed "
          "|excess| and %.1f%% of the net excess"
          % (k, share_abs, share_net))
    print("  net excess %.1f deg, summed |excess| %.1f deg, net/summed %.3f"
          % (total_exc, total_abs, abs(total_exc) / max(total_abs, 1e-9)))
    print("  A uniform scale error holds ~10% in the worst decile and a "
          "net/summed ratio near 1. An episodic error holds most of it in "
          "the decile and drives the ratio toward 0.")


def window_slices(dt, d_truth, window_deg, window_s):
    n = d_truth.size
    if window_deg:
        key, step = np.cumsum(np.abs(d_truth)), float(window_deg)
    elif window_s:
        key, step = np.cumsum(dt), float(window_s)
    else:
        return [(0, n)]

    bounds, target, start = [], step, 0
    for i in range(n):
        if key[i] >= target:
            bounds.append((start, i + 1))
            start = i + 1
            target += step
    if start < n:
        bounds.append((start, n))
    return bounds


def check_expected(tag, tot, n_truth, n_odom):
    exp = EXPECTED.get(tag)
    if exp is None:
        return True
    print("\nREPRODUCTION CHECK, robot_%d, against %s"
          % (exp["robot"], exp["source"]))
    ok = True
    for key in ("abs_truth_deg", "net_truth_deg", "net_odom_deg",
                "excess_deg"):
        want, tol = exp[key]
        got = tot[key]
        band = abs(want) * tol
        hit = abs(got - want) <= band
        ok = ok and hit
        print("  %-15s expected %10.1f +/- %6.1f   got %10.1f   %s"
              % (key, want, band, got, "PASS" if hit else "FAIL"))
    print("  samples: truth %d, odom %d" % (n_truth, n_odom))
    print("  VERDICT: %s" % ("PASS -- method matches, later tables are "
                             "comparable" if ok else
                             "FAIL -- method differs, do NOT compare tables"))
    return ok


# --------------------------------------------------------------------- main

def analyse_robot(bag, series, robot, args, outdir, tag):
    truth_topic = "/model/robot_%d/pose" % robot
    odom_topic = "/robot_%d/odom" % robot

    t_tr, y_tr, x_tr, yp_tr = clean(*series[truth_topic])
    t_od, y_od, _, _ = clean(*series[odom_topic])

    print("\n" + "=" * 70)
    print("%s  robot_%d" % (os.path.basename(bag.rstrip("/")), robot))
    print("=" * 70)
    print("truth %d samples, %.1f s sim, %.2f Hz"
          % (t_tr.size, t_tr[-1] - t_tr[0],
             (t_tr.size - 1) / max(t_tr[-1] - t_tr[0], 1e-9)))
    print("odom  %d samples, %.1f s sim, %.2f Hz"
          % (t_od.size, t_od[-1] - t_od[0],
             (t_od.size - 1) / max(t_od[-1] - t_od[0], 1e-9)))

    t_mid, dt, d_truth, d_odom = increments(t_tr, y_tr, t_od, y_od)
    speed = truth_speed_at(t_tr, x_tr, yp_tr, t_mid)
    rate = np.abs(d_truth) / dt
    tot = totals(dt, d_truth, d_odom)

    path_m = float(np.sum(np.hypot(np.diff(x_tr), np.diff(yp_tr))))
    disp_m = float(math.hypot(x_tr[-1] - x_tr[0], yp_tr[-1] - yp_tr[0]))

    print("\nTOTALS")
    print("  sim time            %10.1f s" % tot["time_s"])
    print("  absolute true turn  %10.1f deg" % tot["abs_truth_deg"])
    print("  net true turn       %10.1f deg" % tot["net_truth_deg"])
    print("  net odom turn       %10.1f deg" % tot["net_odom_deg"])
    print("  excess (odom-truth) %10.1f deg" % tot["excess_deg"])
    print("  excess / |truth|    %10.3f %%" % tot["pct_of_abs"])
    if abs(tot["net_truth_deg"]) > 1.0:
        print("  excess / net truth  %10.3f %%   (not comparable between runs)"
              % (100.0 * tot["excess_deg"] / abs(tot["net_truth_deg"])))
    print("  net / absolute turn %10.3f      (1.0 = monotone spin)"
          % (abs(tot["net_truth_deg"]) / max(tot["abs_truth_deg"], 1e-9)))
    print("  path length         %10.2f m" % path_m)
    print("  net displacement    %10.2f m" % disp_m)
    print("  mean speed          %10.4f m/s, max %.4f"
          % (float(np.mean(speed)), float(np.max(speed))))

    if tot["abs_truth_deg"] < 100.0:
        hit = abs(tot["excess_deg"]) < 1.0
        print("\nNULL CHECK (parked robot, %.1f deg of true rotation): "
              "|excess| = %.4f deg -- %s"
              % (tot["abs_truth_deg"], abs(tot["excess_deg"]),
                 "PASS" if hit else "FAIL, a parked robot is accumulating "
                                    "odometry error"))

    if robot == 0:
        check_expected(tag, tot, t_tr.size, t_od.size)

    fine = binned_table(rate, FINE_EDGES, dt, d_truth, d_odom)
    coarse = binned_table(rate, COARSE_EDGES, dt, d_truth, d_odom)
    spd = binned_table(speed, SPEED_EDGES, dt, d_truth, d_odom)
    print_table("RATE TABLE, fine", "rate deg/s", fine)
    print_table("RATE TABLE, on the published run2 edges", "rate deg/s", coarse)
    print_table("SPEED TABLE, true linear speed", "speed m/s", spd)
    print_crosstab(rate, speed, dt, d_truth, d_odom)
    print_episodes(t_mid, dt, d_truth, d_odom, speed, args.episode)

    win_rows = []
    if args.window_deg or args.window:
        bounds = window_slices(dt, d_truth, args.window_deg, args.window)
        unit = ("%g deg of true rotation" % args.window_deg if args.window_deg
                else "%g s of sim time" % args.window)
        print("\nWINDOWS, each %s" % unit)
        print("%-4s %9s %12s %12s %10s %9s"
              % ("win", "time s", "|truth| deg", "net truth", "excess", "pct"))
        for k, (i0, i1) in enumerate(bounds):
            w = totals(dt[i0:i1], d_truth[i0:i1], d_odom[i0:i1])
            win_rows.append(w)
            print("%-4d %9.1f %12.1f %12.1f %10.1f %8.3f%%"
                  % (k, w["time_s"], w["abs_truth_deg"], w["net_truth_deg"],
                     w["excess_deg"], w["pct_of_abs"]))
        if len(win_rows) > 1:
            pcts = [w["pct_of_abs"] for w in win_rows]
            print("  first %.3f%%, last %.3f%%, spread %.3f points"
                  % (pcts[0], pcts[-1], max(pcts) - min(pcts)))

    if outdir:
        base = "%s_robot%d" % (tag or os.path.basename(bag.rstrip("/")), robot)
        for name, rows in (("rate", fine), ("speed", spd)):
            path = os.path.join(outdir, "%s_%s.csv" % (base, name))
            with open(path, "w") as fh:
                fh.write("lo,hi,time_s,abs_truth_deg,net_truth_deg,"
                         "net_odom_deg,excess_deg,pct_of_abs\n")
                for r in rows:
                    fh.write("%g,%g,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f\n" % r)
        if win_rows:
            with open(os.path.join(outdir, base + "_windows.csv"), "w") as fh:
                fh.write("window,time_s,abs_truth_deg,net_truth_deg,"
                         "net_odom_deg,excess_deg,pct_of_abs\n")
                for k, w in enumerate(win_rows):
                    fh.write("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f\n"
                             % (k, w["time_s"], w["abs_truth_deg"],
                                w["net_truth_deg"], w["net_odom_deg"],
                                w["excess_deg"], w["pct_of_abs"]))
        with open(os.path.join(outdir, base + "_samples.csv"), "w") as fh:
            fh.write("t_sim,dt,d_truth_deg,d_odom_deg,excess_deg,"
                     "rate_deg_s,speed_m_s\n")
            for i in range(t_mid.size):
                fh.write("%.4f,%.5f,%.6f,%.6f,%.6f,%.4f,%.5f\n"
                         % (t_mid[i], dt[i], d_truth[i], d_odom[i],
                            d_odom[i] - d_truth[i], rate[i], speed[i]))
        print("\nwrote %s_{rate,speed,samples}.csv to %s" % (base, outdir))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="bag directory")
    ap.add_argument("--robots", type=int, nargs="+", default=[0],
                    help="robot indices; include a parked one as the null")
    ap.add_argument("--window-deg", type=float, default=0.0,
                    help="window size in degrees of absolute true rotation")
    ap.add_argument("--window", type=float, default=0.0,
                    help="window size in seconds of sim time")
    ap.add_argument("--episode", type=float, default=10.0,
                    help="episode chunk length in sim seconds")
    ap.add_argument("--expect", choices=sorted(EXPECTED), default=None,
                    help="check robot_0 against stored totals for this run")
    ap.add_argument("--outdir", default="experiments/logs/b18/analysis")
    args = ap.parse_args()

    if args.window_deg and args.window:
        sys.exit("ABORT: pass --window-deg or --window, not both")
    if not os.path.isdir(args.bag):
        sys.exit("ABORT: %s is not a directory" % args.bag)

    topics = []
    for n in args.robots:
        topics += ["/model/robot_%d/pose" % n, "/robot_%d/odom" % n]

    if args.outdir:
        os.makedirs(args.outdir, exist_ok=True)

    print("reading %s (%d topics)" % (args.bag, len(topics)))
    series = read_pose_series(args.bag, topics)
    for n in args.robots:
        if series["/model/robot_%d/pose" % n][0].size == 0:
            sys.exit("ABORT: /model/robot_%d/pose is present but empty" % n)

    for n in args.robots:
        analyse_robot(args.bag, series, n, args, args.outdir,
                      args.expect or None)
    print("\nEXIT=0")


if __name__ == "__main__":
    main()
