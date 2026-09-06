#!/usr/bin/env python3
"""Pairwise spatial overlap between the five robots' observed regions in
``experiments/bags/phaseB_run1``.

Question this answers (B1.7): can this bag produce five *usefully divergent*
maps for offline SLAM? If all five robots swept substantially the same region,
five separate SLAM instances yield five near-identical maps and there is
nothing for the reconciliation experiment to reconcile.

Read-only. No ROS runtime, no node, no spin -- just rosbag2_py.SequentialReader
plus rclpy.serialization. Requires ``source /opt/ros/jazzy/setup.bash``.

Writes exactly one file: experiments/logs/bag_overlap_phaseB_run1.csv.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/bag_overlap.py

The ROS imports are function-scoped inside read_bag() rather than module-scoped
so that the parts of this file which need no bag -- resolve_spawn_poses(),
world_walls(), dist_to_nearest_wall() -- can be imported from a plain venv with
no ROS on the path. They are the definition of "on a known wall" for this
project, and a caller that reimplemented them would produce numbers that could
not be compared against the ones this script reports.
"""

from __future__ import annotations

import ast
import csv
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
BAG_DIR = REPO_ROOT / 'experiments' / 'bags' / 'phaseB_run1'
OUT_CSV = REPO_ROOT / 'experiments' / 'logs' / 'bag_overlap_phaseB_run1.csv'
WORLD_SDF = REPO_ROOT / 'ros2_ws' / 'src' / 'nsk_swarm' / 'worlds' / 'knowledge_world.sdf'

LAUNCH_REL = 'ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py'

# The launch revision in effect when this bag was recorded (2026-07-22 01:20).
#
# DO NOT repoint this at HEAD. HEAD's DOT_POSES is a 0.9 m pentagon introduced
# 2026-07-25 by e864ae7 ("cluster spawn poses so all robot pairs start within
# comm_range"), three days AFTER this bag was recorded. Using it silently
# produces a wrong overlap report: transformed under HEAD's table only 4.4% of
# scan points land on a known wall (median 1.52 m off), versus 44.1% (median
# 0.20 m) under the ring below. The validate_against_world() gate at the bottom
# of this script exists to catch exactly that mistake.
BAG_ERA_REV = 'fc050c4'

NUM_ROBOTS = 5
CELL = 0.1            # m, raster resolution
MAX_RANGE = 3.5       # m, matches the bag's LaserScan.range_max
ON_WALL_TOL = 0.15    # m, "this point landed on a known wall"
ON_WALL_FLOOR = 0.15  # aggregate on-wall fraction below which we abort


def die(msg: str) -> None:
    """Fail loudly: no partial output, non-zero exit."""
    print(f'\nFATAL: {msg}\n', file=sys.stderr)
    sys.exit(1)


# ─────────────────────────── step 1: spawn poses ────────────────────────────

def resolve_spawn_poses() -> list[tuple[float, float]]:
    """Parse DOT_POSES out of the launch file *as it was at BAG_ERA_REV*.

    Returns five (x, y) world offsets. Yaw is deliberately not returned: see
    below.
    """
    target = f'{BAG_ERA_REV}:{LAUNCH_REL}'
    searched = [
        f'git show {target}   (from {REPO_ROOT})',
        f'{REPO_ROOT / LAUNCH_REL}   (HEAD -- WRONG for this bag, see BAG_ERA_REV)',
    ]

    try:
        src = subprocess.run(
            ['git', 'show', target],
            cwd=REPO_ROOT, capture_output=True, text=True, check=True,
        ).stdout
    except FileNotFoundError:
        die('git is not on PATH, so the bag-era launch file cannot be read.\n'
            'Searched:\n  ' + '\n  '.join(searched))
    except subprocess.CalledProcessError as exc:
        die(f'could not read {target!r} from git.\n'
            f'git said: {exc.stderr.strip()}\n'
            'Searched:\n  ' + '\n  '.join(searched))

    match = re.search(r'^DOT_POSES\s*=\s*(\[.*?\])', src, re.M | re.S)
    if not match:
        die(f'no DOT_POSES assignment found in {target!r}.\n'
            'Searched:\n  ' + '\n  '.join(searched))

    try:
        table = ast.literal_eval(match.group(1))
    except (ValueError, SyntaxError) as exc:
        die(f'DOT_POSES in {target!r} is not a literal: {exc}\n'
            'Searched:\n  ' + '\n  '.join(searched))

    if len(table) < NUM_ROBOTS:
        die(f'DOT_POSES in {target!r} has {len(table)} entries, need {NUM_ROBOTS}.\n'
            'Searched:\n  ' + '\n  '.join(searched))

    # The third column is dropped on purpose. At BAG_ERA_REV the spawn action
    # passes ros_gz_sim `create` only `-x -y -z` and binds the yaw to an unused
    # `_yaw`, so Gazebo spawned every robot at yaw 0 and each odom frame is
    # axis-aligned with the world. Confirmed empirically: applying the table's
    # yaw collapses the wall fit (robot_1 45.1% -> 0.3%, robot_4 81.4% -> 1.2%).
    # Spawn composition is therefore a pure translation.
    return [(float(x), float(y)) for x, y, _yaw in table[:NUM_ROBOTS]]


# ────────────────────────────── step 2: read ────────────────────────────────

def read_bag():
    """One pass over the bag. Returns (odom, scans, lidar_dxdy).

    odom[n]  -> (N, 4) float array of (t, x, y, yaw), time-sorted
    scans[n] -> list of (t, angle_min, angle_increment, range_min, ranges)
    lidar_dxdy[n] -> (dx, dy) of base_scan in base_link

    This is the only function here that touches ROS, so its imports live in it:
    see the module docstring.
    """
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    if not BAG_DIR.is_dir():
        die(f'bag not found: {BAG_DIR}')

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(BAG_DIR), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )

    available = {t.name for t in reader.get_all_topics_and_types()}
    wanted = (
        [f'/robot_{n}/odom' for n in range(NUM_ROBOTS)]
        + [f'/robot_{n}/scan' for n in range(NUM_ROBOTS)]
        + ['/tf_static']
    )
    missing = [t for t in wanted if t not in available]
    if missing:
        die(f'bag {BAG_DIR.name} is missing required topics: {missing}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    odom_rows: list[list] = [[] for _ in range(NUM_ROBOTS)]
    scans: list[list] = [[] for _ in range(NUM_ROBOTS)]
    lidar_dxdy: dict[int, tuple[float, float]] = {}

    while reader.has_next():
        topic, data, _recv = reader.read_next()

        if topic == '/tf_static':
            for tr in deserialize_message(data, TFMessage).transforms:
                m = re.fullmatch(r'robot_(\d)/base_link', tr.header.frame_id)
                if m and tr.child_frame_id == f'robot_{m.group(1)}/base_scan':
                    lidar_dxdy[int(m.group(1))] = (
                        tr.transform.translation.x, tr.transform.translation.y)
            continue

        n = int(topic[len('/robot_')])

        if topic.endswith('/odom'):
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            # Planar sim: roll/pitch are zero, so yaw reduces to this.
            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom_rows[n].append((stamp, p.x, p.y, yaw))
        else:
            msg = deserialize_message(data, LaserScan)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            scans[n].append((stamp, msg.angle_min, msg.angle_increment,
                             msg.range_min, np.asarray(msg.ranges, dtype=float)))

    for n in range(NUM_ROBOTS):
        if not odom_rows[n]:
            die(f'/robot_{n}/odom carried no messages')
        if not scans[n]:
            die(f'/robot_{n}/scan carried no messages')

    absent = [n for n in range(NUM_ROBOTS) if n not in lidar_dxdy]
    if absent:
        die('/tf_static has no robot_N/base_link -> robot_N/base_scan transform '
            f'for robot(s) {absent}. Refusing to assume a zero lidar offset -- '
            'the true offset is a recorded fact of this bag, not a guess.')

    odom = [np.array(sorted(rows), dtype=float) for rows in odom_rows]
    return odom, scans, lidar_dxdy


# ──────────────────────────── step 3: transform ─────────────────────────────

def scan_rays_world(odom, scans, spawn, lidar):
    """Transform every in-range return to world coords, keeping its direction.

    Composition is spawn_offset o odom_pose o lidar_offset o scan_point, with
    the spawn step a pure translation (spawn yaw is 0, see resolve_spawn_poses).

    The lidar is NOT coincident with base_link: this bag's /tf_static puts
    base_scan at x = -0.032 m, y = 0.0 m in base_link. Both components are read
    from the bag and applied. The z component (0.172 m) is dropped -- this is a
    planar analysis and the scan plane projects straight down onto it.

    Returns (points, bearings, clamped). bearings[n] is aligned row-for-row
    with points[n] and holds the world-frame direction each ray travelled, in
    radians. That is exactly odom_yaw + scan_angle: spawn is a pure translation
    so odom yaw is world yaw, and the lidar offset moves the ray's origin
    without rotating it. Consumed by bag_viewpoint_diversity.py.
    """
    points, bearings, clamped = [], [], []

    for n in range(NUM_ROBOTS):
        a = odom[n]
        t_odom, x_odom, y_odom = a[:, 0], a[:, 1], a[:, 2]
        # Unwrap before interpolating so the track does not tear at +/-pi.
        yaw_odom = np.unwrap(a[:, 3])
        sx, sy = spawn[n]
        dx, dy = lidar[n]

        n_clamped = 0
        chunks, dirs = [], []
        for stamp, angle_min, angle_inc, range_min, ranges in scans[n]:
            # np.interp clamps to the endpoints rather than extrapolating. A
            # handful of scans sit ~10 ms outside the odom span; clamping is
            # right there, but count them so the report can show it is small.
            if stamp < t_odom[0] or stamp > t_odom[-1]:
                n_clamped += 1
            px = np.interp(stamp, t_odom, x_odom)
            py = np.interp(stamp, t_odom, y_odom)
            th = np.interp(stamp, t_odom, yaw_odom)

            keep = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= MAX_RANGE)
            if not keep.any():
                continue
            idx = np.nonzero(keep)[0]
            ang = angle_min + angle_inc * idx
            rng = ranges[idx]

            # scan frame -> base_link
            lx = rng * np.cos(ang) + dx
            ly = rng * np.sin(ang) + dy
            # base_link -> odom frame
            c, s = math.cos(th), math.sin(th)
            bx = px + lx * c - ly * s
            by = py + lx * s + ly * c
            # odom frame -> world (pure translation by the spawn pose)
            chunks.append(np.column_stack([sx + bx, sy + by]))
            dirs.append(th + ang)

        points.append(np.vstack(chunks))
        bearings.append(np.concatenate(dirs))
        clamped.append(n_clamped)

    return points, bearings, clamped


def scan_points_world(odom, scans, spawn, lidar):
    """scan_rays_world without the bearings, for callers that only want points."""
    points, _bearings, clamped = scan_rays_world(odom, scans, spawn, lidar)
    return points, clamped


def path_world(odom, spawn):
    """Robot positions in world coords, one row per odom sample."""
    return [np.column_stack([odom[n][:, 1] + spawn[n][0],
                             odom[n][:, 2] + spawn[n][1]])
            for n in range(NUM_ROBOTS)]


# ──────────────────────────── step 4: rasterize ─────────────────────────────

def make_grid(points, paths):
    """One shared grid for all five robots, so cell indices are comparable.

    Spans the union of the observed points and every path disc, plus a margin.
    """
    xs = np.concatenate([p[:, 0] for p in points] + [q[:, 0] for q in paths])
    ys = np.concatenate([p[:, 1] for p in points] + [q[:, 1] for q in paths])
    x0 = math.floor((xs.min() - MAX_RANGE - CELL) / CELL) * CELL
    y0 = math.floor((ys.min() - MAX_RANGE - CELL) / CELL) * CELL
    nx = int(math.ceil((xs.max() + MAX_RANGE + CELL - x0) / CELL)) + 1
    ny = int(math.ceil((ys.max() + MAX_RANGE + CELL - y0) / CELL)) + 1
    return (x0, y0), (ny, nx)


def _cells(pts, origin, shape):
    ix = np.floor((pts[:, 0] - origin[0]) / CELL).astype(np.int64)
    iy = np.floor((pts[:, 1] - origin[1]) / CELL).astype(np.int64)
    ok = (ix >= 0) & (ix < shape[1]) & (iy >= 0) & (iy < shape[0])
    return ix[ok], iy[ok]


def rasterize_observed(pts, origin, shape):
    grid = np.zeros(shape, dtype=bool)
    ix, iy = _cells(pts, origin, shape)
    grid[iy, ix] = True
    return grid


def rasterize_disc(path, origin, shape):
    """The cruder baseline: a MAX_RANGE disc swept along the path.

    Ignores occlusion entirely -- every cell within 3.5 m of anywhere the robot
    stood counts as "covered", whether or not a wall was in the way.
    """
    ix, iy = _cells(path, origin, shape)
    centres = np.unique(np.column_stack([ix, iy]), axis=0)

    r = int(math.ceil(MAX_RANGE / CELL))
    ody, odx = np.mgrid[-r:r + 1, -r:r + 1]
    mask = (odx * odx + ody * ody) <= r * r
    odx, ody = odx[mask], ody[mask]

    grid = np.zeros(shape, dtype=bool)
    for cx, cy in centres:
        gx, gy = cx + odx, cy + ody
        ok = (gx >= 0) & (gx < shape[1]) & (gy >= 0) & (gy < shape[0])
        grid[gy[ok], gx[ok]] = True
    return grid


# ───────────────────────────── step 5: metrics ──────────────────────────────

def jaccard_matrix(grids):
    m = np.zeros((NUM_ROBOTS, NUM_ROBOTS))
    for i in range(NUM_ROBOTS):
        for j in range(NUM_ROBOTS):
            union = np.count_nonzero(grids[i] | grids[j])
            m[i, j] = np.count_nonzero(grids[i] & grids[j]) / union if union else 0.0
    return m


def asymmetric_matrix(grids):
    """m[i, j] = fraction of i's cells that j also covered."""
    m = np.zeros((NUM_ROBOTS, NUM_ROBOTS))
    for i in range(NUM_ROBOTS):
        own = np.count_nonzero(grids[i])
        for j in range(NUM_ROBOTS):
            m[i, j] = np.count_nonzero(grids[i] & grids[j]) / own if own else 0.0
    return m


def path_lengths(odom):
    return [float(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum()) for a in odom]


# ──────────────────────── step 6: validation gate ───────────────────────────

def world_walls():
    """Axis-aligned (x0, x1, y0, y1) rectangles for every box model in the world.

    The ground plane uses <plane> rather than <box> and carries no <pose>, so
    requiring both naturally excludes it.
    """
    if not WORLD_SDF.is_file():
        die(f'world SDF not found, cannot validate spawn poses: {WORLD_SDF}')

    text = WORLD_SDF.read_text()
    rects = []
    for model in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        body = model.group(2)
        pose = re.search(r'<pose>([^<]+)</pose>', body)
        size = re.search(r'<box>\s*<size>([^<]+)</size>\s*</box>', body)
        if not (pose and size):
            continue
        px, py = (float(v) for v in pose.group(1).split()[:2])
        sx, sy = (float(v) for v in size.group(1).split()[:2])
        rects.append((px - sx / 2, px + sx / 2, py - sy / 2, py + sy / 2))

    if not rects:
        die(f'parsed no box models from {WORLD_SDF}; cannot validate spawn poses')
    return rects


def dist_to_nearest_wall(pts, rects):
    best = np.full(len(pts), np.inf)
    for x0, x1, y0, y1 in rects:
        dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0.0)
        dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0.0)
        np.minimum(best, np.hypot(dx, dy), out=best)
    return best


def validate_against_world(points):
    """Cross-check the resolved spawn poses against the known world geometry.

    If the poses are wrong the whole cloud sits in the wrong place and almost
    nothing lands on a wall. This is warn-and-report per robot, hard-fail only
    on the aggregate: a robot that spends the run in open space legitimately
    scores low, because most of its rays return inf and many of its finite
    returns are the other four robots, which are not in the SDF.
    """
    rects = world_walls()
    per_robot, hits, total = [], 0, 0
    for n in range(NUM_ROBOTS):
        d = dist_to_nearest_wall(points[n], rects)
        on = int(np.count_nonzero(d < ON_WALL_TOL))
        per_robot.append((on / len(d), float(np.median(d))))
        hits += on
        total += len(d)
    return per_robot, hits / total


# ────────────────────────────── step 7: output ──────────────────────────────

def fmt_matrix(title, m, labels):
    out = [title, '        ' + ''.join(f'{l:>9}' for l in labels)]
    for i, l in enumerate(labels):
        out.append(f'{l:>8}' + ''.join(f'{m[i, j]:9.3f}' for j in range(len(labels))))
    return '\n'.join(out)


def write_csv(rows):
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUT_CSV.open('w', newline='') as fh:
        csv.writer(fh).writerows(rows)


def main() -> None:
    labels = [f'robot_{n}' for n in range(NUM_ROBOTS)]

    # ── step 1 ──
    spawn = resolve_spawn_poses()
    print(f'spawn poses (world), parsed from DOT_POSES @ {BAG_ERA_REV}:{LAUNCH_REL}')
    print(f'  (HEAD\'s table is the 0.9 m pentagon from e864ae7, 3 days after this')
    print(f'   bag was recorded -- it does not describe this run)')
    for n, (x, y) in enumerate(spawn):
        print(f'  robot_{n}:  x={x:+.3f}  y={y:+.3f}  yaw=0.000  (yaw not applied at spawn)')

    # ── step 2 ──
    print(f'\nreading {BAG_DIR} ...')
    odom, scans, lidar = read_bag()
    dx, dy = lidar[0]
    print(f'  lidar offset from /tf_static: base_scan at x={dx:+.3f} m, y={dy:+.3f} m '
          f'in base_link (read from the bag, not assumed)')
    for n in range(NUM_ROBOTS):
        span = odom[n][-1, 0] - odom[n][0, 0]
        print(f'  robot_{n}: {len(odom[n]):5d} odom, {len(scans[n]):4d} scans, {span:6.2f} s')

    # ── steps 3-4 ──
    points, clamped = scan_points_world(odom, scans, spawn, lidar)
    paths = path_world(odom, spawn)
    origin, shape = make_grid(points, paths)
    print(f'\ngrid: {shape[1]} x {shape[0]} cells @ {CELL} m, origin '
          f'({origin[0]:.1f}, {origin[1]:.1f})')
    if any(clamped):
        print(f'  scans clamped to the odom time span (no extrapolation): '
              f'{ {f"robot_{n}": c for n, c in enumerate(clamped) if c} }')

    observed = [rasterize_observed(points[n], origin, shape) for n in range(NUM_ROBOTS)]
    discs = [rasterize_disc(paths[n], origin, shape) for n in range(NUM_ROBOTS)]

    # ── step 6 (before reporting numbers that depend on the poses) ──
    per_robot, aggregate = validate_against_world(points)
    print('\nspawn-pose validation vs knowledge_world.sdf '
          f'(points within {ON_WALL_TOL} m of a wall)')
    for n, (frac, med) in enumerate(per_robot):
        print(f'  robot_{n}: {100 * frac:5.1f}% on-wall   median dist {med:5.3f} m')
    verdict = 'PASS' if aggregate >= ON_WALL_FLOOR else 'FAIL'
    print(f'  aggregate: {100 * aggregate:5.1f}%  (floor {100 * ON_WALL_FLOOR:.0f}%)  -> {verdict}')
    if aggregate < ON_WALL_FLOOR:
        die(f'only {100 * aggregate:.1f}% of scan points land on a known wall. The spawn '
            f'poses resolved from {BAG_ERA_REV} do not describe this bag -- every overlap '
            'number below would be wrong, so nothing was written.')

    # ── step 5 ──
    lengths = path_lengths(odom)
    print('\nper robot')
    print(f'{"":>8}{"path_m":>9}{"cells":>8}{"area_m2":>9}   bounding box (x0,y0)-(x1,y1)')
    per_rows = []
    for n in range(NUM_ROBOTS):
        p = points[n]
        bb = (p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max())
        cells = int(np.count_nonzero(observed[n]))
        print(f'{labels[n]:>8}{lengths[n]:9.2f}{cells:8d}{cells * CELL * CELL:9.2f}   '
              f'({bb[0]:+.2f},{bb[1]:+.2f})-({bb[2]:+.2f},{bb[3]:+.2f})')
        per_rows.append([labels[n], f'{lengths[n]:.3f}', cells, f'{cells * CELL * CELL:.3f}',
                         f'{bb[0]:.3f}', f'{bb[1]:.3f}', f'{bb[2]:.3f}', f'{bb[3]:.3f}',
                         int(np.count_nonzero(discs[n]))])

    obs_j, obs_a = jaccard_matrix(observed), asymmetric_matrix(observed)
    dsc_j, dsc_a = jaccard_matrix(discs), asymmetric_matrix(discs)

    print()
    print(fmt_matrix('Jaccard overlap -- observed cells', obs_j, labels))
    print()
    print(fmt_matrix('asymmetric overlap -- fraction of row i\'s cells also seen by column j',
                     obs_a, labels))
    print()
    print(fmt_matrix('Jaccard overlap -- 3.5 m disc swept along path (ignores occlusion)',
                     dsc_j, labels))
    print()
    print(fmt_matrix('asymmetric overlap -- disc method', dsc_a, labels))
    print()
    print(fmt_matrix('disc minus observed (Jaccard) -- how much occlusion the disc misses',
                     dsc_j - obs_j, labels))

    off_diag = ~np.eye(NUM_ROBOTS, dtype=bool)
    print(f'\nmean off-diagonal Jaccard:  observed {obs_j[off_diag].mean():.3f}   '
          f'disc {dsc_j[off_diag].mean():.3f}   '
          f'(disc overstates by {dsc_j[off_diag].mean() - obs_j[off_diag].mean():+.3f})')

    # ── step 7 ──
    rows = [
        ['# bag', str(BAG_DIR)],
        ['# spawn_poses_from', f'{BAG_ERA_REV}:{LAUNCH_REL}'],
        ['# spawn_pose_note', 'HEAD DOT_POSES (pentagon, e864ae7) postdates this bag'],
        ['# lidar_offset_m', f'x={dx:.4f}', f'y={dy:.4f}', 'source=/tf_static'],
        ['# cell_size_m', CELL],
        ['# max_range_m', MAX_RANGE],
        ['# grid_cells', f'{shape[1]}x{shape[0]}'],
        ['# grid_origin', f'{origin[0]:.2f}', f'{origin[1]:.2f}'],
        ['# validation_on_wall_aggregate', f'{aggregate:.4f}'],
        [],
        ['spawn_pose', 'x', 'y', 'yaw'],
        *[[labels[n], f'{spawn[n][0]:.3f}', f'{spawn[n][1]:.3f}', '0.0']
          for n in range(NUM_ROBOTS)],
        [],
        ['per_robot', 'path_length_m', 'observed_cells', 'observed_area_m2',
         'bbox_x_min', 'bbox_y_min', 'bbox_x_max', 'bbox_y_max', 'disc_cells'],
        *per_rows,
        [],
        ['validation', 'on_wall_fraction', 'median_dist_to_wall_m'],
        *[[labels[n], f'{per_robot[n][0]:.4f}', f'{per_robot[n][1]:.4f}']
          for n in range(NUM_ROBOTS)],
    ]
    for title, mat in (
        ('jaccard_observed', obs_j),
        ('asymmetric_observed_row_seen_by_col', obs_a),
        ('jaccard_disc', dsc_j),
        ('asymmetric_disc_row_seen_by_col', dsc_a),
    ):
        rows.append([])
        rows.append([title] + labels)
        rows.extend([labels[i]] + [f'{mat[i, j]:.4f}' for j in range(NUM_ROBOTS)]
                    for i in range(NUM_ROBOTS))

    write_csv(rows)
    print(f'\nwrote {OUT_CSV.relative_to(REPO_ROOT)}')

    print('\ncaveats')
    print('  - recording starts mid-run: the first odom sample is already ~0.66 m from')
    print('    spawn, so pre-recording travel is not represented. These numbers describe')
    print('    the recorded window only, not the whole run.')
    print('  - odom drift is uncorrected. Best-fit per-robot yaw corrections reach')
    print('    0.14 rad, ~0.5 m of point error at 3.5 m range (5 cells). Conclusions hold')
    print('    at the metre scale; do not over-read cell-exact figures.')
    print('  - "observed" means "returned a hit within 3.5 m", not "had line of sight".')
    print('  - the disc/observed gap is not purely occlusion. Observed cells are ~1-cell-')
    print('    thick surface traces (where a ray terminated), whereas the disc is filled')
    print('    area, so the disc reads higher even in a wide-open room. Read the gap as an')
    print('    upper bound on enclosure, and compare gaps between pairs rather than')
    print('    treating the absolute ratio as an occlusion measure.')


if __name__ == '__main__':
    main()
