#!/usr/bin/env python3
"""Viewpoint diversity of the overlap between robots in
``experiments/bags/phaseB_run1``.

Question this answers: when two robots observe the same cell, do they observe
it from the SAME side or OPPOSITE sides? bag_overlap.py measures *how much*
the five observed regions overlap; it says nothing about whether that overlap
is worth anything. Two robots that trace the same wall face from the same side
contribute near-duplicate evidence. Two that see it from opposite sides pin
down both faces, which is what the reconciliation experiment needs.

Method: every ray carries the world-frame direction it travelled. Per (robot,
cell) those directions are reduced to one circular-mean bearing. For each cell
a pair of robots share, the angle between their two mean bearings is binned:

    same side      < 60 deg
    oblique       60-120 deg
    opposite     > 120 deg

Read-only. Sibling of bag_overlap.py and reuses it wholesale -- the fc050c4
spawn poses, the /tf_static lidar offset, the 0.1 m grid and the spawn-pose
validation gate are imported, not re-derived.

Writes exactly one file: experiments/logs/bag_viewpoint_diversity.csv.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/bag_viewpoint_diversity.py
"""

from __future__ import annotations

import csv
import itertools

import numpy as np

# experiments/analysis/ is not a package; sys.path[0] is this script's own
# directory, so the sibling imports plainly.
from bag_overlap import (
    BAG_DIR,
    BAG_ERA_REV,
    CELL,
    LAUNCH_REL,
    NUM_ROBOTS,
    ON_WALL_FLOOR,
    ON_WALL_TOL,
    REPO_ROOT,
    die,
    make_grid,
    path_world,
    rasterize_observed,
    read_bag,
    resolve_spawn_poses,
    scan_rays_world,
    validate_against_world,
)

OUT_CSV = REPO_ROOT / 'experiments' / 'logs' / 'bag_viewpoint_diversity.csv'

SAME_SIDE_DEG = 60.0    # bearing difference below this: same side
OPPOSITE_DEG = 120.0    # above this: opposite sides
MIN_SHARED = 50         # fewer shared cells than this and the fractions are noise

# The pairs the question is about -- the ones bag_overlap.py found with
# non-trivial overlap. Printed first; the other six follow for context.
FOCUS_PAIRS = ((2, 3), (1, 2), (0, 4), (0, 2))


# ───────────────────── step 1: bearing per (robot, cell) ─────────────────────

def cell_bearings(pts, bearing, origin, shape):
    """Reduce every ray that terminated in a cell to one bearing for that cell.

    Mirrors bag_overlap._cells, but keeps the in-bounds mask so the bearings
    stay aligned with the points they belong to. The step-2 gate below proves
    the two agree.

    Returns (cell_ids, mean_bearing, resultant, n_obs), all sorted by cell_id:

      mean_bearing  circular mean, atan2(sum sin, sum cos)
      resultant     R in [0, 1]; 1 = every ray came in on the same heading,
                    0 = the headings cancel and the mean means nothing
      n_obs         how many rays terminated in that cell
    """
    ix = np.floor((pts[:, 0] - origin[0]) / CELL).astype(np.int64)
    iy = np.floor((pts[:, 1] - origin[1]) / CELL).astype(np.int64)
    ok = (ix >= 0) & (ix < shape[1]) & (iy >= 0) & (iy < shape[0])

    cid = iy[ok] * shape[1] + ix[ok]
    b = bearing[ok]

    uniq, inv = np.unique(cid, return_inverse=True)
    cs = np.bincount(inv, weights=np.cos(b), minlength=len(uniq))
    sn = np.bincount(inv, weights=np.sin(b), minlength=len(uniq))
    cnt = np.bincount(inv, minlength=len(uniq)).astype(float)

    return uniq, np.arctan2(sn, cs), np.hypot(cs, sn) / cnt, cnt


def circular_sd_deg(resultant):
    """Circular standard deviation, sqrt(-2 ln R), in degrees.

    A cell seen once has R = 1 and sd 0. A cell the robot drove past, viewing
    it across a wide arc, has a low R and a large sd -- its mean bearing is a
    weak summary and its bin below is correspondingly soft. Unbounded above,
    so the clip only keeps R = 0 from producing an infinity.
    """
    return np.degrees(np.sqrt(-2.0 * np.log(np.clip(resultant, 1e-12, 1.0))))


# ──────────────────────── step 2: pairwise binning ──────────────────────────

def angle_between_deg(a, b):
    """Absolute angle between two bearings, wrapped into [0, 180] deg."""
    d = a - b
    return np.degrees(np.abs(np.arctan2(np.sin(d), np.cos(d))))


def pair_stats(i, j, per_robot):
    """Bin every cell robots i and j share by the angle between their bearings."""
    cid_i, mean_i, res_i, _ = per_robot[i]
    cid_j, mean_j, res_j, _ = per_robot[j]

    _shared, idx_i, idx_j = np.intersect1d(cid_i, cid_j, return_indices=True)
    n = len(_shared)
    if n == 0:
        return {'i': i, 'j': j, 'shared': 0}

    ang = angle_between_deg(mean_i[idx_i], mean_j[idx_j])
    same = int(np.count_nonzero(ang < SAME_SIDE_DEG))
    opposite = int(np.count_nonzero(ang > OPPOSITE_DEG))
    oblique = n - same - opposite

    # How soft the bins are for this pair: per shared cell, the worse of the
    # two robots' own bearing spreads.
    spread = np.maximum(circular_sd_deg(res_i[idx_i]), circular_sd_deg(res_j[idx_j]))

    return {
        'i': i, 'j': j, 'shared': n,
        'same': same, 'oblique': oblique, 'opposite': opposite,
        'f_same': same / n, 'f_oblique': oblique / n, 'f_opposite': opposite / n,
        'median_angle': float(np.median(ang)),
        'mean_angle': float(np.mean(ang)),
        'median_spread': float(np.median(spread)),
    }


# ────────────────────────────── step 3: output ──────────────────────────────

def write_csv(rows):
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUT_CSV.open('w', newline='') as fh:
        csv.writer(fh).writerows(rows)


def print_pair_row(st, labels, focus):
    mark = '*' if focus else ' '
    thin = '  (thin)' if st['shared'] < MIN_SHARED else ''
    pair = f'{labels[st["i"]]} & {labels[st["j"]]}'
    if not st['shared']:
        print(f' {mark} {pair:<19}{0:8d}' + ' ' * 34 + '  no shared cells')
        return
    print(f' {mark} {pair:<19}{st["shared"]:8d}'
          f'{st["f_same"]:10.3f}{st["f_oblique"]:10.3f}{st["f_opposite"]:10.3f}'
          f'{st["median_angle"]:9.1f}{st["median_spread"]:9.1f}{thin}')


def main() -> None:
    labels = [f'robot_{n}' for n in range(NUM_ROBOTS)]

    spawn = resolve_spawn_poses()
    print(f'spawn poses (world), parsed from DOT_POSES @ {BAG_ERA_REV}:{LAUNCH_REL}')
    for n, (x, y) in enumerate(spawn):
        print(f'  robot_{n}:  x={x:+.3f}  y={y:+.3f}  yaw=0.000  (yaw not applied at spawn)')

    print(f'\nreading {BAG_DIR} ...')
    odom, scans, lidar = read_bag()
    dx, dy = lidar[0]
    print(f'  lidar offset from /tf_static: base_scan at x={dx:+.3f} m, y={dy:+.3f} m '
          f'in base_link (read from the bag, not assumed)')

    points, bearings, _clamped = scan_rays_world(odom, scans, spawn, lidar)
    paths = path_world(odom, spawn)
    origin, shape = make_grid(points, paths)
    print(f'\ngrid: {shape[1]} x {shape[0]} cells @ {CELL} m, origin '
          f'({origin[0]:.1f}, {origin[1]:.1f})   (same grid as bag_overlap.py)')

    # Same gate as bag_overlap.py, for the same reason: if the spawn poses are
    # wrong every point is in the wrong place -- and so is every bearing.
    per_wall, aggregate = validate_against_world(points)
    print(f'\nspawn-pose validation vs knowledge_world.sdf '
          f'(points within {ON_WALL_TOL} m of a wall)')
    verdict = 'PASS' if aggregate >= ON_WALL_FLOOR else 'FAIL'
    print(f'  aggregate: {100 * aggregate:5.1f}%  (floor {100 * ON_WALL_FLOOR:.0f}%)  -> {verdict}')
    if aggregate < ON_WALL_FLOOR:
        die(f'only {100 * aggregate:.1f}% of scan points land on a known wall. The spawn '
            f'poses resolved from {BAG_ERA_REV} do not describe this bag -- every bearing '
            'below would be wrong, so nothing was written.')

    # ── step 1 ──
    per_robot = [cell_bearings(points[n], bearings[n], origin, shape)
                 for n in range(NUM_ROBOTS)]

    # Gate: these must be exactly the cells bag_overlap.py rasterizes. A
    # mismatch means the cell-id packing here disagrees with that raster, and
    # every shared-cell count below would be built on the wrong cells.
    for n in range(NUM_ROBOTS):
        expect = int(np.count_nonzero(rasterize_observed(points[n], origin, shape)))
        got = len(per_robot[n][0])
        if got != expect:
            die(f'robot_{n}: {got} cells from the bearing pass but {expect} from '
                f'bag_overlap.rasterize_observed. The two disagree about which cells '
                'were observed, so the pair bins would be meaningless.')

    print('\nper robot -- observed cells and how tightly each was viewed')
    print(f'{"":>8}{"cells":>8}{"rays":>9}{"rays/cell":>11}{"median spread deg":>19}')
    per_rows = []
    for n in range(NUM_ROBOTS):
        cid, _mean, res, cnt = per_robot[n]
        spread = float(np.median(circular_sd_deg(res)))
        print(f'{labels[n]:>8}{len(cid):8d}{int(cnt.sum()):9d}{cnt.mean():11.1f}{spread:19.1f}')
        per_rows.append([labels[n], len(cid), int(cnt.sum()), f'{cnt.mean():.2f}',
                         f'{spread:.2f}'])

    # How many robots saw each cell at all -- context for "two or more".
    all_cells = np.concatenate([per_robot[n][0] for n in range(NUM_ROBOTS)])
    _u, seen_by = np.unique(all_cells, return_counts=True)
    multi = {k: int(np.count_nonzero(seen_by == k)) for k in range(1, NUM_ROBOTS + 1)}
    print(f'\ncells by number of observing robots: '
          f'{ {k: v for k, v in multi.items() if v} }')

    # ── step 2 ──
    stats = {(i, j): pair_stats(i, j, per_robot)
             for i, j in itertools.combinations(range(NUM_ROBOTS), 2)}
    rest = sorted((p for p in stats if p not in FOCUS_PAIRS),
                  key=lambda p: -stats[p]['shared'])

    header = (f'{"":>4}{"pair":<19}{"shared":>8}{"same<60":>10}{"oblique":>10}'
              f'{"opp>120":>10}{"med deg":>9}{"spread":>9}')
    print('\npairwise viewpoint diversity -- fraction of shared cells per bin')
    print(header)
    for p in FOCUS_PAIRS:
        print_pair_row(stats[p], labels, focus=True)
    print(f'{"":>4}{"-- other pairs --":<19}')
    for p in rest:
        print_pair_row(stats[p], labels, focus=False)
    print(f'\n  * = the four pairs with non-trivial overlap.  '
          f'"spread" is the median per-cell')
    print('  circular SD of a single robot\'s own bearings to that cell: the larger it is,')
    print('  the softer that cell\'s bin.')

    # ── step 3 ──
    print('\nreading')
    for p in FOCUS_PAIRS:
        st = stats[p]
        if not st['shared']:
            continue
        bins = (('same side', st['f_same']), ('oblique', st['f_oblique']),
                ('opposite sides', st['f_opposite']))
        top, frac = max(bins, key=lambda b: b[1])
        note = '' if st['shared'] >= MIN_SHARED else '  -- but too few cells to trust'
        print(f'  {labels[st["i"]]} & {labels[st["j"]]}: {st["shared"]} shared cells, '
              f'mostly {top} ({100 * frac:.0f}%), median {st["median_angle"]:.0f} deg{note}')

    rows = [
        ['# bag', str(BAG_DIR)],
        ['# question', 'when two robots observe the same cell, same side or opposite sides?'],
        ['# spawn_poses_from', f'{BAG_ERA_REV}:{LAUNCH_REL}'],
        ['# spawn_pose_note', 'HEAD DOT_POSES (pentagon, e864ae7) postdates this bag'],
        ['# lidar_offset_m', f'x={dx:.4f}', f'y={dy:.4f}', 'source=/tf_static'],
        ['# cell_size_m', CELL],
        ['# grid_cells', f'{shape[1]}x{shape[0]}'],
        ['# grid_origin', f'{origin[0]:.2f}', f'{origin[1]:.2f}'],
        ['# validation_on_wall_aggregate', f'{aggregate:.4f}'],
        ['# bearing_def', 'world-frame direction the ray travelled '
                          '(odom yaw + scan angle); per cell, circular mean over all '
                          'rays that terminated there'],
        ['# bins_deg', f'same_side<{SAME_SIDE_DEG:.0f}',
         f'oblique {SAME_SIDE_DEG:.0f}-{OPPOSITE_DEG:.0f}', f'opposite>{OPPOSITE_DEG:.0f}'],
        ['# spread_def', 'circular SD sqrt(-2 ln R) of one robot\'s own bearings to a '
                         'cell; per pair, median over shared cells of the larger of the two'],
        ['# caveat', 'SLAM/odom drift reaches 0.14 rad, so bearings are approximate -- '
                     'the bins are indicative, not exact'],
        [],
        ['per_robot', 'observed_cells', 'rays', 'rays_per_cell', 'median_spread_deg'],
        *per_rows,
        [],
        ['cells_seen_by_n_robots', *[f'n={k}' for k in sorted(multi)]],
        ['count', *[multi[k] for k in sorted(multi)]],
        [],
        ['pair', 'shared_cells', 'frac_same_side', 'frac_oblique', 'frac_opposite',
         'count_same_side', 'count_oblique', 'count_opposite',
         'median_angle_deg', 'mean_angle_deg', 'median_spread_deg', 'focus'],
    ]
    for p in list(FOCUS_PAIRS) + rest:
        st = stats[p]
        name = f'{labels[st["i"]]}-{labels[st["j"]]}'
        focus = 1 if p in FOCUS_PAIRS else 0
        if not st['shared']:
            rows.append([name, 0, '', '', '', '', '', '', '', '', '', focus])
            continue
        rows.append([
            name, st['shared'],
            f'{st["f_same"]:.4f}', f'{st["f_oblique"]:.4f}', f'{st["f_opposite"]:.4f}',
            st['same'], st['oblique'], st['opposite'],
            f'{st["median_angle"]:.2f}', f'{st["mean_angle"]:.2f}',
            f'{st["median_spread"]:.2f}', focus,
        ])

    write_csv(rows)
    print(f'\nwrote {OUT_CSV.relative_to(REPO_ROOT)}')

    print('\ncaveats')
    print('  - SLAM drift up to 0.14 rad means these bearings are APPROXIMATE. Treat the')
    print('    bins as indicative, not exact. A 0.14 rad tilt is ~8 deg against 60 deg bin')
    print('    edges, so only cells already near a boundary flip; the larger effect is on')
    print('    which cell a return lands in at all (~0.5 m at 3.5 m range, 5 cells).')
    print('  - one bearing per (robot, cell) is a circular mean. Where the spread column is')
    print('    wide the robot viewed that cell across an arc and the mean is a weak summary,')
    print('    independently of drift.')
    print('  - inherited from bag_overlap.py: recording starts mid-run, so this is the')
    print('    recorded window only; and "observed" means "a ray terminated here", not')
    print('    "had line of sight".')
    print('  - a shared cell is one 0.1 m cell both robots put a return in. Near a wall')
    print('    corner two robots can share a cell while looking at different faces of it.')


if __name__ == '__main__':
    main()
