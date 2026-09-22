#!/usr/bin/env python3
"""Is a map cut off early just a disc around the path, or do walls shape it?

A map that stops early is not automatically an incomplete map. Two very
different things look alike in a saturation curve:

  * the robot simply ran out of time, and the map is the disc of everything
    within sensor range of wherever it happened to drive; or
  * the robot was BOXED IN -- walls blocked the sensor, so even cells it drove
    straight past stayed unknown, and more time would not have helped.

The discriminator is what fraction of the reachable-by-line-of-sight set is
actually known. This script computes that at a series of cut times:

  reach            FINAL-map free cells within --range of any path point so
                   far, straight-line, WALLS IGNORED. This is the disc-union
                   hypothesis: what a map would cover if nothing occluded.
  occlusion index  |known free now AND reach| / |reach|. 1.0 means the map is
                   exactly the disc and nothing blocked it; well below 1.0
                   means walls shaped it and the missing cells are occluded,
                   not merely unvisited.
  stray            known free cells OUTSIDE reach, as % of known free. This is
                   the self-check, not a result -- see SELF-CHECK below.

Read-only on the bag. No simulator, no SLAM, no replay.

TIME ZERO, and why it is imported rather than written here:
  t0 is the sim time of the first nonzero /robot_K/cmd_vel. A Twist carries no
  header, so its only timestamp is the recorder's WALL-CLOCK receive stamp, and
  turning that into sim time means correlating it against /clock. check_run_bag
  C4 already does this and is the reference implementation, so first_nonzero_cmd
  and sim_window are imported from it. sim_window is called with a budget of
  0.0: its first return value is "the last /clock at or before the command",
  which is precisely C4's sim-time reading of t0, and a zero budget asks it for
  that alone rather than also requiring the bag to hold 2400 s more sim (which
  b2maps_relay1, at 529 s, does not).

SPAWN REVISION -- the trap this script would otherwise fall into:
  the path is placed in the map frame as inv(spawn) * truth, so it is only as
  good as the spawn table. DOT_POSES changed between bag eras and the two
  tables put robot_0 3.15 m apart, which is far more than the 1% stray gate
  would tolerate. bag_overlap's module default (fc050c4) is the b16-era ring
  and is WRONG for b2maps: it puts robot_0 at (3.00, 0.00) against this bag's
  measured (0.858, 0.280). --spawn-rev therefore defaults to 08617b2, matching
  check_run_bag, and robot K's first truth pose is checked against the table
  before anything is built on it. Spawn yaw is zero -- resolve_spawn_poses
  verifies that against the launch file at the revision and dies otherwise --
  so the composition is a pure translation.

SELF-CHECK (designed to be able to fail):
  nothing beyond sensor range can be known, so free cells outside reach are
  not a finding, they are evidence that the path and the map are in different
  frames. Above STRAY_MAX_PCT at any cut this exits 2. --range defaults to
  8.0 m, which is this world's LaserScan.range_max, and the bag's actual
  range_max is read and compared so a mismatched --range is visible rather
  than silently generous.

Usage:
    python3 experiments/analysis/partial_maps.py \
        --bag experiments/logs/b2maps/b2maps_relay1 --robot 0 \
        [--cuts 60,120,240] [--range 8.0] [--spawn-rev 08617b2]

Writes <out-dir>/<run>_partial/<run>_partial.csv and _partial.png.
Exit 2 if the stray gate fails or the bag/topics are unusable. Expected: well
under a minute on b2maps_relay1.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")               # headless: written to file, never shown
import matplotlib.pyplot as plt     # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "experiments" / "slam"))
sys.path.insert(0, str(REPO_ROOT / "experiments" / "analysis"))

from bag_overlap import SPAWN_TOL_M, resolve_spawn_poses          # noqa: E402
from check_run_bag import (                                        # noqa: E402
    first_nonzero_cmd,
    read_clock_series,
    read_prologue_series,
    sim_window,
    yaw_from_quaternion,
)

# check_run_bag's own --spawn-rev default. NOT bag_overlap.BAG_ERA_REV, which
# is the b16-era ring and misplaces every b2maps robot by ~2.2 m.
SPAWN_REV = "08617b2"

# A cut is only honoured if a map message lands this close to it in sim time.
# Maps arrive about every 2 s in these runs, so a gap above this means the map
# stream has a hole at the cut and the panel would be labelled with a time it
# does not depict.
MAX_CUT_GAP_S = 2.0

# The stray gate. 1% of known free cells, per the brief. Not a tuned number:
# the honest expectation is ~0, since a cell outside sensor range of every
# path point cannot have been observed at all.
STRAY_MAX_PCT = 1.0

# Free/occupied thresholds, the nav2 trinary convention shared with
# map_saturation.py.
OCC_MIN = 65

EXIT_FAIL = 2


def abort(msg):
    print("\nABORT: %s" % msg, file=sys.stderr)
    sys.exit(EXIT_FAIL)


# ────────────────────────────── grids ────────────────────────────────────────

class Grid:
    """An OccupancyGrid's payload plus the frame it lives in.

    Cell (r, c) covers x in [ox + c*res, ox + (c+1)*res) and likewise in y, so
    row 0 is the MINIMUM-y row -- hence origin="lower" when this is drawn.
    """

    def __init__(self, msg):
        info = msg.info
        self.res = info.resolution
        self.w, self.h = info.width, info.height
        self.ox = info.origin.position.x
        self.oy = info.origin.position.y
        q = info.origin.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data = np.frombuffer(msg.data, dtype=np.int8).reshape(self.h, self.w).copy()

    @property
    def extent(self):
        """(xmin, xmax, ymin, ymax) in metres, for imshow."""
        return (self.ox, self.ox + self.w * self.res,
                self.oy, self.oy + self.h * self.res)

    def counts(self):
        free = int(np.count_nonzero(self.data == 0))
        occ = int(np.count_nonzero(self.data >= OCC_MIN))
        unknown = int(np.count_nonzero(self.data == -1))
        return free, occ, unknown


def resample_onto(src: Grid, canvas: Grid) -> np.ndarray:
    """`src` expressed on `canvas`'s grid, nearest containing cell.

    Map origins drift by NON-INTEGER fractions of a cell between messages in
    these bags (measured: up to 0.49 cells on b2maps_relay1), so the cut maps
    and the final map do not share a cell lattice and cannot be aligned by an
    integer offset. Every cell is therefore looked up by world coordinate.
    The residual misregistration is bounded by half a cell, 2.5 cm here.

    Canvas cells not covered by `src` come back unknown (-1), which is what
    they were: outside the map that existed at that moment.
    """
    xs = canvas.ox + (np.arange(canvas.w) + 0.5) * canvas.res
    ys = canvas.oy + (np.arange(canvas.h) + 0.5) * canvas.res
    cc = np.floor((xs - src.ox) / src.res).astype(int)
    rr = np.floor((ys - src.oy) / src.res).astype(int)
    okc = (cc >= 0) & (cc < src.w)
    okr = (rr >= 0) & (rr < src.h)

    out = np.full((canvas.h, canvas.w), -1, dtype=np.int8)
    if okc.any() and okr.any():
        out[np.ix_(okr, okc)] = src.data[np.ix_(rr[okr], cc[okc])]
    return out


def outside_canvas_cells(src: Grid, canvas: Grid) -> int:
    """Known cells of `src` that the canvas does not cover -- lost by resampling."""
    xs = src.ox + (np.arange(src.w) + 0.5) * src.res
    ys = src.oy + (np.arange(src.h) + 0.5) * src.res
    inx = (xs >= canvas.ox) & (xs < canvas.ox + canvas.w * canvas.res)
    iny = (ys >= canvas.oy) & (ys < canvas.oy + canvas.h * canvas.res)
    inside = np.zeros((src.h, src.w), dtype=bool)
    inside[np.ix_(iny, inx)] = True
    return int(np.count_nonzero((src.data != -1) & ~inside))


# ───────────────────────────── bag reading ───────────────────────────────────

def time_zero(bag: Path, robot: int) -> float:
    """Sim time of the first nonzero cmd_vel, exactly as check_run_bag C4 reads it."""
    cmds, _poses_on_receive_time = read_prologue_series(bag, robot)
    if not cmds:
        abort("no /robot_%d/cmd_vel in %s, so there is no time zero" % (robot, bag))
    t_cmd = first_nonzero_cmd(cmds)
    if t_cmd is None:
        abort("robot_%d was never commanded to move in %s" % (robot, bag))

    # Budget 0.0: ask sim_window only for its first return value, the last
    # /clock at or before the command. A nonzero budget would additionally
    # demand the bag hold that much more sim time, which is C4's question, not
    # this script's.
    found = sim_window(read_clock_series(bag), t_cmd, 0.0)
    if found is None:
        abort("no /clock at or before the first command in %s, so the command's "
              "receive stamp cannot be converted to sim time" % bag)
    sim_start, _t_end, _sim_end = found
    return sim_start


def read_streams(bag: Path, robot: int, targets):
    """One pass: the maps nearest each target, the final map, truth, tf, scan range.

    Only the wanted grids are retained -- keeping all 272 (or 3331) would cost
    far more memory than the answer needs. Because stamps increase, tracking the
    running best |stamp - target| finds the nearest message in a single pass.
    """
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    map_topic = "/robot_%d/map" % robot
    pose_topic = "/model/robot_%d/pose" % robot
    scan_topic = "/robot_%d/scan" % robot
    parent, child = "robot_%d/map" % robot, "robot_%d/odom" % robot

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    available = {t.name for t in reader.get_all_topics_and_types()}
    for need in (map_topic, pose_topic):
        if need not in available:
            abort("%s has no %s" % (bag, need))
    wanted = [t for t in (map_topic, pose_topic, scan_topic, "/tf") if t in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    best = [None] * len(targets)        # (gap, Grid) per target
    final = None
    poses = []                          # (sim_s, x, y) in the world frame
    tf = []                             # (sim_s, x, y, yaw) for map -> odom
    scan_range_max = None
    n_maps = 0

    while reader.has_next():
        topic, data, _recv_is_wall_clock = reader.read_next()

        if topic == map_topic:
            grid = Grid(deserialize_message(data, OccupancyGrid))
            n_maps += 1
            final = grid
            for i, target in enumerate(targets):
                gap = abs(grid.stamp - target)
                if best[i] is None or gap < best[i][0]:
                    best[i] = (gap, grid)

        elif topic == pose_topic:
            m = deserialize_message(data, PoseStamped)
            poses.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          m.pose.position.x, m.pose.position.y))

        elif topic == scan_topic:
            if scan_range_max is None:
                scan_range_max = deserialize_message(data, LaserScan).range_max

        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == parent and tr.child_frame_id == child:
                    t = tr.transform.translation
                    tf.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                               t.x, t.y,
                               yaw_from_quaternion(*[getattr(tr.transform.rotation, a)
                                                     for a in "xyzw"])))

    if not n_maps:
        abort("no messages on %s" % map_topic)
    if not poses:
        abort("no messages on %s, so there is no truth path" % pose_topic)

    poses = np.array(sorted(poses))
    tf.sort()
    return best, final, poses, tf, scan_range_max, n_maps


# ───────────────────────────── geometry ──────────────────────────────────────

def reach_masks(canvas: Grid, path_xy: np.ndarray, range_m: float, final_free):
    """(within_range, reach) on the canvas.

    within_range is the union of discs of radius `range_m` about the path --
    straight-line, walls ignored, per the brief. It is computed as a Euclidean
    distance transform from the path cells, which is exact to the grid: the
    distance is to the nearest path CELL CENTRE, so it carries the same
    half-cell (2.5 cm) quantisation as everything else here, against an 8 m
    radius.

    The transform runs on a canvas padded by the range on all sides, so a path
    point outside the canvas still projects its disc inward. Cropping restores
    the canvas.
    """
    from scipy.ndimage import distance_transform_edt

    pad = int(math.ceil(range_m / canvas.res)) + 1
    seed = np.ones((canvas.h + 2 * pad, canvas.w + 2 * pad), dtype=bool)

    cc = np.floor((path_xy[:, 0] - canvas.ox) / canvas.res).astype(int) + pad
    rr = np.floor((path_xy[:, 1] - canvas.oy) / canvas.res).astype(int) + pad
    ok = (cc >= 0) & (cc < seed.shape[1]) & (rr >= 0) & (rr < seed.shape[0])
    if not ok.any():
        abort("every path point lies more than %.1f m outside the final map, so "
              "the path and the map are not in the same frame" % range_m)
    seed[rr[ok], cc[ok]] = False

    dist = distance_transform_edt(seed, sampling=canvas.res)[
        pad:pad + canvas.h, pad:pad + canvas.w]
    within = dist <= range_m
    return within, within & final_free, dist


def nearest_tf(tf, t):
    """The map->odom sample nearest sim time `t`, with its gap."""
    if not tf:
        return None
    best = min(tf, key=lambda s: abs(s[0] - t))
    return best, abs(best[0] - t)


def path_length(path_xy):
    if len(path_xy) < 2:
        return 0.0
    d = np.diff(path_xy, axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


# ────────────────────────────── plotting ─────────────────────────────────────

# free / unknown / occupied, light to dark so walls read as walls.
RGB_FREE = (0.97, 0.97, 0.94)
RGB_UNKNOWN = (0.62, 0.64, 0.66)
RGB_OCC = (0.10, 0.10, 0.12)


def render(ax, cells, canvas, path_xy, reach, title, range_m):
    img = np.empty((canvas.h, canvas.w, 3), dtype=float)
    img[...] = RGB_UNKNOWN
    img[cells == 0] = RGB_FREE
    img[cells >= OCC_MIN] = RGB_OCC
    ax.imshow(img, origin="lower", extent=canvas.extent, interpolation="nearest")

    xs = canvas.ox + (np.arange(canvas.w) + 0.5) * canvas.res
    ys = canvas.oy + (np.arange(canvas.h) + 0.5) * canvas.res
    ax.contour(xs, ys, reach.astype(float), levels=[0.5],
               colors="#d62728", linewidths=1.1)

    if len(path_xy):
        ax.plot(path_xy[:, 0], path_xy[:, 1], lw=1.2, color="#1f77b4")
    # Spawn is the map frame's own origin under inv(spawn) * truth.
    ax.plot([0.0], [0.0], marker="*", ms=11, color="#ff7f0e",
            markeredgecolor="0.15", markeredgewidth=0.5, zorder=4)

    ax.set_xlim(canvas.extent[0], canvas.extent[1])
    ax.set_ylim(canvas.extent[2], canvas.extent[3])
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7)


def write_png(path, rows, canvas, range_m, run, robot):
    panels = [r for r in rows if r["_cells"] is not None]
    n = len(panels)
    fig, axes = plt.subplots(1, n, figsize=(4.3 * n, 5.8), squeeze=False)
    for ax, r in zip(axes[0], panels):
        render(ax, r["_cells"], canvas, r["_path"], r["_reach"],
               "%s\nsim %.1f s   free %.1f m$^2$ (%.0f%% of final)\nocclusion "
               "index %.3f" % (r["label"], r["map_sim_s"], r["free_area_m2"],
                               100 * r["frac_of_final_free"],
                               r["occlusion_index"]), range_m)
    axes[0][0].set_ylabel("y in %s map frame (m)" % ("robot_%d" % robot))
    for ax in axes[0]:
        ax.set_xlabel("x (m)")
    fig.suptitle("%s  robot_%d  --  partial maps on a shared %.0f x %.0f m extent"
                 % (run, robot, canvas.w * canvas.res, canvas.h * canvas.res),
                 fontsize=11)

    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    handles = [
        Patch(facecolor=RGB_FREE, edgecolor="0.5", label="free"),
        Patch(facecolor=RGB_OCC, label="occupied"),
        Patch(facecolor=RGB_UNKNOWN, label="unknown"),
        Line2D([], [], color="#d62728", lw=1.1,
               label="reach outline: final-map free within %.1f m of the path, "
                     "walls ignored (so it traces walls too)" % range_m),
        Line2D([], [], color="#1f77b4", lw=1.2, label="true path so far"),
        Line2D([], [], color="#ff7f0e", marker="*", ms=10, ls="none",
               label="spawn = map-frame origin"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.11, 1, 0.93))
    fig.savefig(path, dpi=130)
    plt.close(fig)


CSV_COLUMNS = [
    "label", "cut_s", "target_sim_s", "map_sim_s", "gap_s", "skipped",
    "map_w", "map_h", "map_res_m",
    "free_cells", "occupied_cells", "unknown_cells",
    "free_area_m2", "frac_of_final_free",
    "map_odom_x", "map_odom_y", "map_odom_yaw_deg", "map_odom_gap_s",
    "path_points", "path_len_m",
    "reach_cells", "reach_area_m2", "free_in_reach_cells", "occlusion_index",
    "stray_cells", "stray_pct", "stray_beyond_range_cells",
    "stray_not_final_free_cells", "gate_p99_free_from_path_m",
    "furthest_free_from_path_m",
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--cuts", default="60,120,240",
                    help="sim seconds after time zero (default: %(default)s)")
    ap.add_argument("--range", type=float, default=8.0, dest="range_m",
                    help="reach radius in m (default: %(default)s, this world's "
                         "LaserScan.range_max)")
    ap.add_argument("--spawn-rev", default=SPAWN_REV,
                    help="git rev whose DOT_POSES gives the spawn poses "
                         "(default: %(default)s)")
    ap.add_argument("--out-dir",
                    default=os.path.join("experiments", "logs", "b2maps", "analysis"))
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.is_dir():
        abort("not a bag directory: %s" % bag)
    try:
        cuts = [float(c) for c in args.cuts.split(",") if c.strip()]
    except ValueError:
        abort("--cuts must be comma-separated seconds, got %r" % args.cuts)
    if not cuts:
        abort("--cuts is empty")

    run = bag.name
    out_dir = Path(args.out_dir) / ("%s_partial" % run)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("%s  robot_%d" % (run, args.robot))

    t0 = time_zero(bag, args.robot)
    print("  time zero (first nonzero cmd_vel, C4's reading): sim %.3f s" % t0)

    targets = [t0 + c for c in cuts]
    best, final, poses, tf, scan_max, n_maps = read_streams(bag, args.robot, targets)

    print("  maps: %d, final at sim %.1f s" % (n_maps, final.stamp))
    print("  truth: %d poses, sim %.1f -> %.1f s"
          % (len(poses), poses[0, 0], poses[-1, 0]))
    print("  map->odom samples in /tf: %d" % len(tf))
    if scan_max is None:
        print("  NOTE: no /robot_%d/scan in the bag, so --range %.1f m could not "
              "be checked against the sensor." % (args.robot, args.range_m))
    else:
        print("  scan range_max: %.2f m  (--range is %.2f m)" % (scan_max, args.range_m))
        if abs(scan_max - args.range_m) > 1e-3:
            print("  NOTE: --range differs from the bag's range_max. Reach is then "
                  "not the sensor's footprint, and both the occlusion index and "
                  "the stray gate move with it.")

    # ── spawn, and the guard that the table is the right era ────────────────
    spawn = resolve_spawn_poses(args.spawn_rev)
    if args.robot >= len(spawn):
        abort("DOT_POSES @ %s has no robot_%d" % (args.spawn_rev, args.robot))
    sx, sy = spawn[args.robot]
    first = poses[0]
    off = math.hypot(first[1] - sx, first[2] - sy)
    print("  spawn @ %s: (%+.3f, %+.3f); first truth (%+.3f, %+.3f); off by %.3f m"
          % (args.spawn_rev, sx, sy, first[1], first[2], off))
    if off > SPAWN_TOL_M:
        abort("robot_%d's first truth pose is %.3f m from DOT_POSES @ %s (tolerance "
              "%.2f m). That is the wrong spawn era for this bag, and every path "
              "below would be offset by it. Pass the right --spawn-rev."
              % (args.robot, off, args.spawn_rev, SPAWN_TOL_M))

    # inv(spawn) * truth. Spawn yaw is zero -- resolve_spawn_poses dies unless
    # the launch file at this rev passes no '-Y' -- so this is a translation.
    path_all = np.column_stack([poses[:, 1] - sx, poses[:, 2] - sy])
    path_t = poses[:, 0]

    # ── the canvas: the final map, which is the shared extent ───────────────
    canvas = final
    final_cells = final.data
    final_free = final_cells == 0
    n_final_free = int(final_free.sum())
    if n_final_free == 0:
        abort("the final map has no free cells, so there is nothing to compare against")
    print("  shared extent: %.1f x %.1f m (%d x %d cells @ %.3f m), final free "
          "%d cells = %.1f m^2"
          % (canvas.w * canvas.res, canvas.h * canvas.res, canvas.w, canvas.h,
             canvas.res, n_final_free, n_final_free * canvas.res ** 2))

    rows = []
    plan = [(("cut %.0f s" % c), c, tgt, b) for c, tgt, b in zip(cuts, targets, best)]
    plan.append(("final", None, final.stamp, (0.0, final)))

    for label, cut, target, found in plan:
        gap, grid = found
        row = {k: "" for k in CSV_COLUMNS}
        row.update(label=label, cut_s="" if cut is None else cut,
                   target_sim_s=target, map_sim_s=grid.stamp, gap_s=gap, skipped=0)
        row["_cells"] = None

        print("\n  %s -- target sim %.1f s" % (label, target))
        print("    nearest map: sim %.1f s, gap %.2f s" % (grid.stamp, gap))
        if gap > MAX_CUT_GAP_S:
            print("    SKIPPED: the gap exceeds %.1f s, so no map message depicts "
                  "this cut. The map stream has a hole here." % MAX_CUT_GAP_S)
            row["skipped"] = 1
            rows.append(row)
            continue

        lost = outside_canvas_cells(grid, canvas)
        if lost:
            print("    NOTE: %d known cells of this map fall outside the final "
                  "map's extent and are dropped by the resample." % lost)
        cells = resample_onto(grid, canvas)

        free_now = cells == 0
        n_free = int(free_now.sum())
        free_area = n_free * canvas.res ** 2

        # Path up to this map's stamp.
        upto = path_t <= grid.stamp
        path_xy = path_all[upto]
        within, reach, dist = reach_masks(canvas, path_xy, args.range_m, final_free)
        n_reach = int(reach.sum())

        # The gate's actual operating point. "stray >= 1% of known free" is
        # exactly "the 99th percentile of known-free-cell distance exceeds
        # --range", so that percentile -- not the maximum -- is what the gate
        # compares against, and range minus it is the real headroom. The
        # maximum is kept alongside because it is the honest worst case.
        d_free = dist[free_now]
        gate_d = (float(np.percentile(d_free, 100.0 - STRAY_MAX_PCT))
                  if d_free.size else 0.0)
        worst_d = float(d_free.max()) if d_free.size else 0.0

        in_reach = int((free_now & reach).sum())
        occl = in_reach / n_reach if n_reach else float("nan")

        stray = int((free_now & ~reach).sum())
        stray_far = int((free_now & ~within).sum())
        stray_revised = int((free_now & within & ~final_free).sum())
        stray_pct = 100.0 * stray / n_free if n_free else 0.0

        tf_at = nearest_tf(tf, grid.stamp)
        if tf_at is None:
            print("    map->odom: no robot_%d/map -> robot_%d/odom in /tf at all"
                  % (args.robot, args.robot))
        else:
            (ts, tx, ty, tyaw), tgap = tf_at
            print("    map->odom at sim %.1f s (gap %.2f s): x %+.3f  y %+.3f  "
                  "yaw %+.2f deg  |t| %.3f m"
                  % (ts, tgap, tx, ty, math.degrees(tyaw), math.hypot(tx, ty)))
            row.update(map_odom_x=tx, map_odom_y=ty,
                       map_odom_yaw_deg=math.degrees(tyaw), map_odom_gap_s=tgap)

        print("    free %d cells = %.1f m^2 (%.1f%% of the final map's free area)"
              % (n_free, free_area, 100.0 * n_free / n_final_free))
        print("    path: %d points, %.1f m driven" % (len(path_xy), path_length(path_xy)))
        print("    reach: %d cells = %.1f m^2" % (n_reach, n_reach * canvas.res ** 2))
        print("    occlusion index: %.3f  (%d of %d reach cells are known free)"
              % (occl, in_reach, n_reach))
        print("    stray: %d cells = %.3f%% of known free  (beyond range %d, "
              "within range but not free in the final map %d)"
              % (stray, stray_pct, stray_far, stray_revised))
        print("    known-free distance to path: p%.0f %.2f m (the gate's own "
              "operating point), max %.2f m, against a %.2f m radius -- %.2f m "
              "of headroom"
              % (100.0 - STRAY_MAX_PCT, gate_d, worst_d, args.range_m,
                 args.range_m - gate_d))

        occ_n, unk_n = (int((cells >= OCC_MIN).sum()), int((cells == -1).sum()))
        row.update(map_w=grid.w, map_h=grid.h, map_res_m=grid.res,
                   free_cells=n_free, occupied_cells=occ_n, unknown_cells=unk_n,
                   free_area_m2=free_area,
                   frac_of_final_free=n_free / n_final_free,
                   path_points=len(path_xy), path_len_m=path_length(path_xy),
                   reach_cells=n_reach, reach_area_m2=n_reach * canvas.res ** 2,
                   free_in_reach_cells=in_reach, occlusion_index=occl,
                   stray_cells=stray, stray_pct=stray_pct,
                   stray_beyond_range_cells=stray_far,
                   stray_not_final_free_cells=stray_revised,
                   gate_p99_free_from_path_m=gate_d,
                   furthest_free_from_path_m=worst_d)
        row["_cells"] = cells
        row["_path"] = path_xy
        row["_reach"] = reach
        rows.append(row)

    csv_path = out_dir / ("%s_partial.csv" % run)
    with open(csv_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
    print("\n  csv: %s" % csv_path)

    drawn = [r for r in rows if r["_cells"] is not None]
    if drawn:
        png_path = out_dir / ("%s_partial.png" % run)
        write_png(png_path, rows, canvas, args.range_m, run, args.robot)
        print("  png: %s" % png_path)

    # ── self-check ──────────────────────────────────────────────────────────
    bad = [r for r in drawn if r["stray_pct"] >= STRAY_MAX_PCT]
    print("\n  SELF-CHECK  stray < %.1f%% at every cut:" % STRAY_MAX_PCT)
    for r in drawn:
        print("    %-10s %.3f%%   p%.0f distance %.2f m of %.2f m  %s"
              % (r["label"], r["stray_pct"], 100.0 - STRAY_MAX_PCT,
                 r["gate_p99_free_from_path_m"], args.range_m,
                 "FAIL" if r in bad else "ok"))
    if bad:
        worst = max(bad, key=lambda r: r["stray_pct"])
        far, rev = worst["stray_beyond_range_cells"], worst["stray_not_final_free_cells"]
        print("\n  FAILED at %d cut(s). Nothing beyond sensor range can be known, "
              "so free cells outside reach are not a finding about the world."
              % len(bad))
        if far >= rev:
            print("  %d of %s's %d stray cells are beyond %.1f m of every path point. "
                  "THE PATH AND MAP FRAMES DISAGREE: inv(spawn) * truth is not "
                  "landing where the map thinks the robot drove. Check --spawn-rev "
                  "(%s) before reading any number above."
                  % (far, worst["label"], worst["stray_cells"], args.range_m,
                     args.spawn_rev))
            if scan_max is not None and args.range_m < scan_max - 1e-3:
                print("  One alternative first, though: --range is %.2f m against "
                      "the bag's range_max of %.2f m. A reach radius below what "
                      "the sensor could actually see produces exactly this "
                      "signature with the frames perfectly sound."
                      % (args.range_m, scan_max))
        else:
            print("  but %d of %s's %d stray cells are WITHIN range and merely "
                  "stopped being free by the final map, which is map revision, "
                  "not a frame error. The frames may well agree; the reach set is "
                  "the thing that moved."
                  % (rev, worst["label"], worst["stray_cells"]))
        return EXIT_FAIL

    # A pass is only as strong as the slack it passed with. When every known
    # free cell sits far inside --range, the disc union covers the map whatever
    # the path does, and the gate would clear a badly misplaced path too. Say so
    # rather than let a green line imply a frame was verified.
    slack = min(args.range_m - r["gate_p99_free_from_path_m"] for r in drawn)
    print("  passed: fewer than %.1f%% of known free cells lie beyond %.1f m of "
          "the path." % (STRAY_MAX_PCT, args.range_m))
    # State the sensitivity unconditionally. Displacing the path rigidly by d
    # raises every cell-to-path distance by at most d, so it raises the p99 by
    # at most d: any displacement within the headroom is GUARANTEED to still
    # pass. That bound is the gate's resolution, and it belongs next to the
    # pass rather than behind a threshold someone has to go looking for.
    print("  Sensitivity: the tightest cut cleared the gate by %.2f m, so a path "
          "displaced rigidly by up to %.2f m would also have passed. This is a "
          "floor on frame agreement, not a measurement of it. On a %.0f x %.0f m "
          "map an %.1f m radius covers nearly everything; to resolve finer, rerun "
          "with --range near the smallest displacement worth catching."
          % (slack, slack, canvas.w * canvas.res, canvas.h * canvas.res,
             args.range_m))
    return 0


if __name__ == "__main__":
    sys.exit(main())
