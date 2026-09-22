#!/usr/bin/env python3
"""When does the SLAM map stop growing? Free area against sim time, from a bag.

pgm_extent.py measures a SAVED map -- one static snapshot, which can report how
much was mapped but never when the mapping stopped. This reads the OccupancyGrid
stream itself, so the question a saturation curve answers is available:

  * did free area reach its final value early and sit there, or
  * was it still climbing when the recorder stopped, in which case the final
    value is a floor, not a result, and the run was cut short rather than done.

TIME, stated once and used everywhere:
  sim time comes from header.stamp, never from the reader's receive timestamp.
  The receive timestamp is WALL CLOCK -- on b2maps_e0t it spans 6684 s of wall
  against 6484 s of sim, so the two disagree by minutes and plotting the wrong
  one silently shifts every milestone below. The receive timestamp is discarded
  at the read loop and is not carried in any column, so it cannot be picked up
  by mistake downstream.

CELL CLASSES, stated once:
  free = 0, occupied >= 65, unknown = -1, following the nav2 trinary convention.
  Anything else is counted as `other` rather than folded into a neighbour: both
  b2maps bags publish only {-1, 0, 100}, so a non-zero `other` means the
  publisher is not trinary and every area below is measuring something else.

FREE AREA IS NOT MONOTONE. It dips in both b2maps bags -- a loop closure can
retract cells that were free a moment earlier. So the milestones are stated as
"first reaches", and max is printed next to final whenever they differ: a final
below max means the map LOST free area, and the percentages are then being read
against a target that itself moved.

Usage:
    python3 experiments/analysis/map_saturation.py \
        --bag experiments/logs/b2maps/b2maps_relay1 [--topic /robot_1/map]

Writes <out-dir>/<run>_saturation.csv and <run>_saturation.png.
Exit 1 on a failed self-check (no messages, or sim time not strictly
increasing), 2 on a bad bag or topic. Expected: seconds per bag, not minutes --
filtering to one topic skips the rest of the bag cheaply.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")               # headless: written to file, never shown
import matplotlib.pyplot as plt     # noqa: E402

import rosbag2_py                   # noqa: E402
from nav_msgs.msg import OccupancyGrid          # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402

MAP_TYPE = "nav_msgs/msg/OccupancyGrid"

# The reference mark drawn on the time axis. Not derived from the data.
BUDGET_S = 2400.0

# The window at the end of the run tested for continued growth.
TAIL_S = 60.0

# Free area is called "still rising" if it gained more than this fraction of its
# final value over the last TAIL_S. This is a reporting threshold, not a physical
# constant -- the raw gain in m^2 and its slope are printed either way, so the
# threshold can be second-guessed from the output. 0.5% over a minute is well
# above the one-or-two-cell jitter of a settled map at 0.05 m/cell.
RISE_TOL_FRAC = 0.005

MILESTONES = (0.50, 0.90, 0.99)

# Exit 2 is a bad bag or topic -- the run was never read. Exit 1 is a failed
# self-check on a bag that WAS read. Keeping them apart matters when this is
# driven over a set of bags: 2 means fix the command, 1 means look at the run.
EXIT_BAD_INPUT = 2


def abort(msg):
    print("ABORT: %s" % msg, file=sys.stderr)
    sys.exit(EXIT_BAD_INPUT)


def storage_id_of(bag_dir):
    """The bag's own storage_identifier, so this is not hardcoded to mcap."""
    path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.isfile(path):
        abort("%s has no metadata.yaml -- not a bag directory" % bag_dir)
    for line in open(path):
        line = line.strip()
        if line.startswith("storage_identifier:"):
            return line.split(":", 1)[1].strip().strip("'\"")
    abort("%s states no storage_identifier" % path)


def open_reader(bag_dir, topic):
    """Reader positioned on `topic` alone, after checking it is a map topic."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id_of(bag_dir)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        maps = sorted(n for n, t in types.items() if t == MAP_TYPE)
        abort("%s has no %s\n       %s" % (
            bag_dir, topic,
            "OccupancyGrid topics present: " + ", ".join(maps) if maps
            else "this bag has no OccupancyGrid topic at all"))
    if types[topic] != MAP_TYPE:
        abort("%s is %s, not %s" % (topic, types[topic], MAP_TYPE))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader


def read_grids(bag_dir, topic):
    """One row per OccupancyGrid message. The receive timestamp is dropped here."""
    reader = open_reader(bag_dir, topic)
    rows = []
    while reader.has_next():
        _topic, data, _recv_is_wall_clock = reader.read_next()
        msg = deserialize_message(data, OccupancyGrid)

        res = msg.info.resolution
        cell_m2 = res * res
        w, h = msg.info.width, msg.info.height

        # rclpy deserialises int8[] to array('b'), so frombuffer is a view of
        # the message's own memory -- no copy of the grid is made.
        a = np.frombuffer(msg.data, dtype=np.int8)
        free = int(np.count_nonzero(a == 0))
        occ = int(np.count_nonzero(a >= 65))
        unknown = int(np.count_nonzero(a == -1))

        rows.append({
            "i": len(rows),
            "sim_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "width": w,
            "height": h,
            "resolution_m": res,
            "width_m": w * res,
            "height_m": h * res,
            "free_cells": free,
            "occupied_cells": occ,
            "unknown_cells": unknown,
            "other_cells": int(a.size) - free - occ - unknown,
            "free_area_m2": free * cell_m2,
            "occupied_area_m2": occ * cell_m2,
        })
    return rows


COLUMNS = ["i", "sim_s", "elapsed_s", "width", "height", "resolution_m",
           "width_m", "height_m", "free_cells", "occupied_cells",
           "unknown_cells", "other_cells", "free_area_m2", "occupied_area_m2"]


def write_csv(path, rows):
    t0 = rows[0]["sim_s"]
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        for r in rows:
            writer.writerow(dict(r, elapsed_s=r["sim_s"] - t0))


def check_monotone(t):
    """Indices where sim time fails to advance. Empty means strictly increasing."""
    return [i + 1 for i in np.where(np.diff(t) <= 0)[0]]


def milestones(t, area, final):
    """Sim time at which area FIRST reaches each fraction of `final`."""
    out = []
    for frac in MILESTONES:
        hit = np.where(area >= frac * final)[0]
        out.append((frac, float(t[hit[0]]) if hit.size else None))
    return out


def tail_growth(t, area):
    """(gain_m2, slope_m2_per_s, span_s) over the last TAIL_S, or None if short."""
    if t[-1] - t[0] < TAIL_S:
        return None
    i = int(np.searchsorted(t, t[-1] - TAIL_S))
    span = t[-1] - t[i]
    if span <= 0:
        return None
    return area[-1] - area[i], (area[-1] - area[i]) / span, span


def plot(path, t, area, final, marks, run, topic, xmax):
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, area, lw=1.6, color="#1f77b4")

    for frac, at in marks:
        if at is None:
            continue
        ax.plot([at], [frac * final], "o", ms=5, color="#d62728", zorder=3)
        ax.annotate("%d%%  %.0f s" % (frac * 100, at), (at, frac * final),
                    textcoords="offset points", xytext=(6, -10), fontsize=8,
                    color="#d62728")

    hi = xmax if xmax else t[-1]
    if BUDGET_S <= hi:
        ax.axvline(BUDGET_S, color="0.35", ls="--", lw=1.2)
        ax.annotate("%.0f s" % BUDGET_S, (BUDGET_S, ax.get_ylim()[1]),
                    textcoords="offset points", xytext=(4, -12), fontsize=8,
                    color="0.35")
    else:
        # Drawing the line here would stretch the axis to ~4x the data and
        # squash the whole curve into the left quarter. State the fact instead.
        ax.annotate("run ends at %.1f s, before the %.0f s mark\n"
                    "(mark is off-scale and not drawn)" % (t[-1], BUDGET_S),
                    (0.98, 0.06), xycoords="axes fraction", ha="right",
                    fontsize=8, color="0.35")

    ax.set_xlim(t[0], hi)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("sim time (s, from header.stamp -- not bag receive time)")
    ax.set_ylabel("free area (m$^2$)")
    ax.set_title("%s  %s  --  free area, final %.1f m$^2$" % (run, topic, final))
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bag", required=True, help="bag directory")
    ap.add_argument("--topic", default="/robot_0/map",
                    help="OccupancyGrid topic (default: %(default)s)")
    ap.add_argument("--out-dir",
                    default=os.path.join("experiments", "logs", "b2maps", "analysis"),
                    help="where the CSV and PNG go (default: %(default)s)")
    ap.add_argument("--xmax", type=float, default=None,
                    help="force the time axis to end here, so two runs share "
                         "an axis; default is the last message")
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        abort("not a bag directory: %s" % args.bag)

    run = os.path.basename(os.path.normpath(args.bag))
    rows = read_grids(args.bag, args.topic)

    print("%s  %s" % (run, args.topic))
    print("  messages: %d" % len(rows))
    if not rows:
        print("SELF-CHECK FAILED: no messages on %s -- nothing written." % args.topic)
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, "%s_saturation.csv" % run)
    png_path = os.path.join(args.out_dir, "%s_saturation.png" % run)

    # Written before the time self-check so a bag that fails it is still
    # inspectable; the derived claims below are what get withheld.
    write_csv(csv_path, rows)
    print("  csv:      %s" % csv_path)

    t = np.array([r["sim_s"] for r in rows])
    area = np.array([r["free_area_m2"] for r in rows])
    last = rows[-1]

    print("  sim time: %.2f -> %.2f s  (span %.1f s)" % (t[0], t[-1], t[-1] - t[0]))
    print("  final grid: %d x %d cells @ %.3f m = %.2f x %.2f m"
          % (last["width"], last["height"], last["resolution_m"],
             last["width_m"], last["height_m"]))
    print("  final cells: free %d, occupied %d, unknown %d, other %d"
          % (last["free_cells"], last["occupied_cells"],
             last["unknown_cells"], last["other_cells"]))
    if last["other_cells"]:
        print("  NOTE: %d cells are neither free, occupied nor unknown -- this "
              "publisher is not trinary, so the areas above are not comparable "
              "with the other b2maps runs." % last["other_cells"])

    bad = check_monotone(t)
    if bad:
        print("SELF-CHECK FAILED: sim time is not strictly increasing "
              "(%d of %d steps do not advance)." % (len(bad), len(t) - 1))
        for i in bad[:5]:
            print("    msg %d: %.6f -> msg %d: %.6f  (dt %+.6f)"
                  % (i - 1, t[i - 1], i, t[i], t[i] - t[i - 1]))
        if len(bad) > 5:
            print("    ... and %d more" % (len(bad) - 5))
        print("  Milestones, the plateau verdict and the PNG are all functions "
              "of time ordering, so none is reported. The CSV above is written "
              "for diagnosis.")
        return 1

    final = float(area[-1])
    peak = float(area.max())
    print("  final free area: %.1f m^2" % final)
    if peak > final:
        print("  NOTE: peak free area was %.1f m^2 at sim %.1f s -- the map LOST "
              "%.1f m^2 (%.1f%%) by the end, so the percentages below are read "
              "against a final value that is not the maximum."
              % (peak, t[int(area.argmax())], peak - final,
                 100.0 * (peak - final) / peak))

    if final <= 0:
        print("  free area is zero at the end; no milestones to report.")
        marks = [(f, None) for f in MILESTONES]
    else:
        marks = milestones(t, area, final)
        print("  free area first reaches, as a fraction of its final value:")
        for frac, at in marks:
            if at is None:
                print("    %3d%% (%7.1f m^2): never" % (frac * 100, frac * final))
            else:
                print("    %3d%% (%7.1f m^2): sim %.1f s  (%.1f s into the map stream)"
                      % (frac * 100, frac * final, at, at - t[0]))

    tail = tail_growth(t, area)
    if tail is None:
        print("  last %.0f s: the map stream spans only %.1f s, too short to test "
              "for continued growth." % (TAIL_S, t[-1] - t[0]))
    else:
        gain, slope, span = tail
        pct = 100.0 * gain / final if final > 0 else 0.0
        print("  last %.0f s (sim %.1f -> %.1f, %.1f s of samples): %+.2f m^2 "
              "(%+.3f%% of final, %+.4f m^2/s)"
              % (TAIL_S, t[-1] - TAIL_S, t[-1], span, gain, pct, slope))
        if final > 0 and gain > RISE_TOL_FRAC * final:
            print("  STILL RISING at the end of the run. The final value of "
                  "%.1f m^2 is NOT a plateau -- it is where the recording "
                  "stopped, and the map was still growing. Treat it as a lower "
                  "bound on what this configuration would have mapped."
                  % final)
        else:
            print("  Plateaued: the gain over the last %.0f s is within the "
                  "%.1f%% reporting threshold, so %.1f m^2 is a settled value."
                  % (TAIL_S, RISE_TOL_FRAC * 100, final))

    plot(png_path, t, area, final, marks, run, args.topic, args.xmax)
    print("  png:      %s" % png_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
