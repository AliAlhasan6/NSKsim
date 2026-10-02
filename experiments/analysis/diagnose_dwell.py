#!/usr/bin/env python3
"""Why B1 fails on four of the twenty b2maps grids. A DIAGNOSIS, and nothing else.

Part B established the fact: on those four maps every off-wall return sits within
2*rho of a real wall, median 0.000 m, so the map lacks occupied cells where a wall
really is (SPEC_b2_wall_predicate §12). This asks what put them there.

HYPOTHESIS. The offline replay is DETERMINISTIC and integrates every scan. A robot
parked for a long stretch therefore contributes thousands of scans from one pose,
and those scans can clear wall cells that the moving scans had committed.

PRE-REGISTERED PREDICTIONS, printed before any number is read:

  P1  On each of the four B1 failures, off-wall returns are over-represented in
      stationary scans: the share of off-wall returns that come from stationary
      scans is greater than the share of all kept returns that come from
      stationary scans.
  P2  The four B1 failures are the four maps with the highest stationary share.
  P3  Most cells holding an off-wall return are FREE, not UNKNOWN. That would mean
      the wall was cleared, not never seen.

WHAT THIS TOUCHES. Nothing. It writes exactly one file,
experiments/logs/graph_walls/diagnose_dwell.json, and it opens no map JSON, moves
no threshold, and re-runs no check that anything is gated on. Every pose, scan and
return comes from graph_walls' own functions, so the numbers below are about the
same returns Part B measured and not about a second reading of the bags.

TRUTH. None, except where it already was: B1 is truth-free, the stationary split
is from /tf, and the commands are from the run bag. No world SDF is loaded.

TWO CLOCKS. /robot_K/cmd_vel is a Twist and carries no header, so the only stamp
it has is the bag's receive time. /clock carries both -- its receive time and the
sim time it announces -- so it is the map between them, and every command time
printed here is a SIM time interpolated through it. check_run_bag.py:243 reads the
same topic the same way, and CMD_EPS_LINEAR / CMD_EPS_ANGULAR are imported from it
so that "nonzero command" means here what it means in C3's gate.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/diagnose_dwell.py

About three minutes: the 20 stripped bags Part B reads, plus the five raw run bags
for the commands. No simulator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import graph_walls as gw                                    # noqa: E402
from check_run_bag import CMD_EPS_ANGULAR, CMD_EPS_LINEAR    # noqa: E402

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_dwell.json'

# The stationary test, as asked for. Not a threshold anything is gated on: it
# splits one population in two so the two halves can be compared with each other.
STILL_M = 0.01
STILL_DEG = 0.5

PREDICTIONS = [
    ('P1', 'on each of the four B1 failures, the share of OFF-WALL returns from '
           'stationary scans exceeds the share of ALL KEPT returns from '
           'stationary scans'),
    ('P2', 'the four B1 failures are the four maps with the highest stationary '
           'share'),
    ('P3', 'most cells holding an off-wall return are FREE, not UNKNOWN -- the '
           'wall was cleared, not never seen'),
]


def read_cmd_and_clock(bag: Path, k: int) -> tuple[list, np.ndarray]:
    """(commands, clock map) from one raw run bag.

    commands are (receive time, linear.x, angular.z); the clock map is an
    (n, 2) array of (receive time, sim time) with every /clock message in it.
    Nothing is subsampled: a sparser map would need an argument about how
    affine the two clocks are locally, and reading them all costs three seconds.
    """
    import rosbag2_py                                        # noqa: PLC0415
    from geometry_msgs.msg import Twist                      # noqa: PLC0415
    from rclpy.serialization import deserialize_message      # noqa: PLC0415
    from rosgraph_msgs.msg import Clock                      # noqa: PLC0415

    if not bag.is_dir():
        gw.die(f'raw run bag not found, so the commands cannot be read: {bag}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topic = f'/robot_{k}/cmd_vel'
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/clock', topic]))

    cmds, clock = [], []
    while reader.has_next():
        name, data, recv = reader.read_next()
        if name == '/clock':
            m = deserialize_message(data, Clock)
            clock.append((recv / 1e9,
                          m.clock.sec + m.clock.nanosec * 1e-9))
        else:
            m = deserialize_message(data, Twist)
            cmds.append((recv / 1e9, m.linear.x, m.angular.z))
    if not clock:
        gw.die(f'{bag.name}: no /clock, so receive times cannot be put on the '
               'sim clock')
    if not cmds:
        gw.die(f'{bag.name}: no {topic}')
    return cmds, np.array(sorted(clock))


def to_sim(clock: np.ndarray, recv: float) -> float:
    """One receive time on the sim clock, by interpolation. No extrapolation."""
    return float(np.interp(recv, clock[:, 0], clock[:, 1]))


def stationary_scans(sx: np.ndarray, sy: np.ndarray, syaw: np.ndarray,
                     ok: np.ndarray) -> np.ndarray:
    """Which scans did not move since the previous KEPT scan.

    Under 0.01 m AND under 0.5 deg. Measured between consecutive kept scans, so a
    scan dropped for want of a pose does not make its successor look stationary
    by accident. The first kept scan has no predecessor and is counted as MOVING:
    it is one scan out of hundreds, and calling it stationary would be asserting
    something about a comparison that does not exist.
    """
    stat = np.zeros(sx.size, dtype=bool)
    idx = np.nonzero(ok)[0]
    if idx.size < 2:
        return stat
    step = np.hypot(np.diff(sx[idx]), np.diff(sy[idx]))
    turn = np.abs((np.diff(syaw[idx]) + math.pi) % (2.0 * math.pi) - math.pi)
    stat[idx[1:]] = (step < STILL_M) & (turn < math.radians(STILL_DEG))
    return stat


def cells_of(returns: dict, rho: float, origin, shape) -> tuple:
    """(row, col, inside) of every return, the way on_occupied_share bins them."""
    h, w = shape
    col = np.floor((returns['px'] - origin[0]) / rho + 1e-9).astype(np.int64)
    row = np.floor((returns['py'] - origin[1]) / rho + 1e-9).astype(np.int64)
    return row, col, (row >= 0) & (row < h) & (col >= 0) & (col < w)


def off_wall_cells(grid: np.ndarray, row: np.ndarray, col: np.ndarray,
                   inside: np.ndarray, off: np.ndarray,
                   stat: np.ndarray) -> dict:
    """Every cell an off-wall return landed in, with its state and two counts.

    Returns that fall outside the map have no cell and no state; they are counted
    separately rather than folded in, because "the map has no wall here" and "the
    map does not reach here" are different statements.
    """
    sel = off & inside
    h, w = grid.shape
    flat = row[sel] * w + col[sel]
    uniq, inv = np.unique(flat, return_inverse=True)
    n_stat = np.bincount(inv, weights=stat[sel].astype(np.float64),
                         minlength=uniq.size)
    n_all = np.bincount(inv, minlength=uniq.size)
    urow, ucol = (uniq // w).astype(int), (uniq % w).astype(int)
    state = grid[urow, ucol]
    free = state == gw.FREE
    unknown = state == gw.UNKNOWN
    occ = state == gw.OCC
    return {
        'n_cells': int(uniq.size),
        'n_cells_free': int(np.count_nonzero(free)),
        'n_cells_unknown': int(np.count_nonzero(unknown)),
        'n_cells_occupied': int(np.count_nonzero(occ)),
        'n_returns_in_free_cells': int(n_all[free].sum()),
        'n_returns_in_unknown_cells': int(n_all[unknown].sum()),
        'n_returns_outside_map': int(np.count_nonzero(off & ~inside)),
        'cells': {
            'row': urow.tolist(), 'col': ucol.tolist(),
            'state': ['FREE' if s == gw.FREE else
                      'UNKNOWN' if s == gw.UNKNOWN else 'OCCUPIED'
                      for s in state.tolist()],
            'n_stationary': [int(v) for v in n_stat.tolist()],
            'n_moving': [int(a - s) for a, s in zip(n_all.tolist(),
                                                    n_stat.tolist())],
        },
    }


def diagnose(stem: str, k: int, cut: int, cmds: list,
             clock: np.ndarray) -> dict:
    """One map. Every pose and return comes from graph_walls, unchanged."""
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    graph, _el, _seg, grid = gw.graph_and_elements(stem)
    rho, origin = graph['resolution'], graph['origin']
    r_max, r_max_src = gw.read_replay_r_max(k, cut)
    data = gw.read_bag(gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}_cut{cut}_slamin', k)
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    fwt.RUN = f'{gw.RUN_PREFIX}_k{k}_cut{cut}'
    mto, _p = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, stem)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    returns = gw.scan_returns(data, poses, base_scan, mto, r_max)

    ok = poses[3]
    stat_scan = stationary_scans(returns['sensor_x'], returns['sensor_y'],
                                 returns['sensor_yaw'], ok)
    stat = stat_scan[returns['scan']]

    b1 = gw.on_occupied_share(returns, grid, rho, origin)
    off = ~b1['hit']
    row, col, inside = cells_of(returns, rho, origin, grid.shape)

    n_kept_scans = int(np.count_nonzero(ok))
    n_stat_scans = int(np.count_nonzero(stat_scan))
    n_ret = int(returns['n_kept'])
    n_off = int(np.count_nonzero(off))

    # B1 again, on the moving scans alone. Reported, gated on nothing.
    moving = ~stat
    b1_moving = (float(np.count_nonzero(b1['hit'] & moving))
                 / np.count_nonzero(moving)) if moving.any() else float('nan')
    b1_stationary = (float(np.count_nonzero(b1['hit'] & stat))
                     / np.count_nonzero(stat)) if stat.any() else float('nan')

    # commands, on the sim clock
    cut_end = float(data['stamps'][ok].max())
    cut_start = float(data['stamps'][ok].min())
    nonzero = [(t, lin, ang) for t, lin, ang in cmds
               if abs(lin) > CMD_EPS_LINEAR or abs(ang) > CMD_EPS_ANGULAR]
    last_run = to_sim(clock, nonzero[-1][0]) if nonzero else None
    before = [to_sim(clock, t) for t, _l, _a in nonzero]
    before = [t for t in before if t <= cut_end]
    last_in_cut = max(before) if before else None

    return {
        'map': stem, 'robot': k, 'cut_s': cut,
        'r_max': {'value': r_max, 'source': r_max_src},
        'scans': {
            'n_read': int(data['stamps'].size),
            'n_kept': n_kept_scans,
            'n_stationary': n_stat_scans,
            'share_stationary': n_stat_scans / n_kept_scans,
            'test': f'moved < {STILL_M} m AND turned < {STILL_DEG} deg since '
                    'the previous kept scan; the first kept scan counts as '
                    'moving',
        },
        'returns': {
            'n_kept': n_ret,
            'n_from_stationary': int(np.count_nonzero(stat)),
            'share_from_stationary': float(np.count_nonzero(stat)) / n_ret,
            'n_off_wall': n_off,
            'n_off_wall_from_stationary': int(np.count_nonzero(off & stat)),
            'share_off_wall_from_stationary': (float(np.count_nonzero(off & stat))
                                               / n_off) if n_off else
            float('nan'),
        },
        'B1': {
            'all_scans': b1['share'],
            'moving_scans_only': b1_moving,
            'stationary_scans_only': b1_stationary,
            'limit_for_reference': gw.B1_MIN_SHARE,
            'note': 'reported, gated on nothing. The limit is printed only so '
                    'the three numbers can be read against the one Part B '
                    'records',
        },
        'off_wall_cells': off_wall_cells(grid, row, col, inside, off, stat),
        'commands': {
            'cut_window_sim_s': [cut_start, cut_end],
            'last_nonzero_cmd_vel_sim_s_whole_run': last_run,
            'last_nonzero_cmd_vel_sim_s_within_cut': last_in_cut,
            'dwell_after_last_command_s': (cut_end - last_in_cut)
            if last_in_cut is not None else None,
            'dwell_share_of_cut': ((cut_end - last_in_cut) /
                                   (cut_end - cut_start))
            if last_in_cut is not None else None,
            'eps': {'linear_m_s': CMD_EPS_LINEAR,
                    'angular_rad_s': CMD_EPS_ANGULAR,
                    'source': 'check_run_bag.py:75-76'},
        },
    }


def main() -> int:
    # There are no options: the diagnosis is one fixed measurement over the whole
    # corpus. The parser exists so that --help answers in a moment instead of
    # reading 2.5 GB of bags first, which is what it did before it was here.
    argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog='No options. Reads the 20 stripped bags and the 5 raw run bags, '
               f'writes {OUT_PATH.relative_to(REPO_ROOT)}, and nothing else.',
        formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()

    print('why B1 fails on four of the twenty b2maps grids -- a diagnosis')
    print('  HYPOTHESIS: the replay is deterministic and integrates every scan, '
          'so a robot parked')
    print('  for a long stretch contributes thousands of scans from one pose, '
          'and those scans can')
    print('  clear wall cells that the moving scans had committed.')
    print()
    print('PRE-REGISTERED PREDICTIONS -- printed before any number is read:')
    for name, text in PREDICTIONS:
        print(f'  {name}  {text}')
    print()
    print('Writes one file and nothing else: '
          f'{OUT_PATH.relative_to(REPO_ROOT)}')
    print('  No map JSON is opened, no threshold is moved, nothing is gated.')
    print('=' * 100)
    print()

    rows: dict[str, dict] = {}
    for k in range(gw.NUM_ROBOTS):
        cmds, clock = read_cmd_and_clock(
            gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}', k)
        print(f'robot_{k}: raw bag read -- {len(cmds)} cmd_vel, '
              f'{clock.shape[0]} /clock samples spanning sim '
              f'{clock[0, 1]:.1f}..{clock[-1, 1]:.1f} s')
        for cut in gw.CUTS:
            stem = f'{gw.RUN_PREFIX}_k{k}_cut{cut}_robot{k}'
            rows[stem] = diagnose(stem, k, cut, cmds, clock)
            r = rows[stem]
            dwell = r['commands']['dwell_after_last_command_s'] or 0.0
            print(f'   {stem:<30} scans {r["scans"]["share_stationary"]:>6.1%} '
                  f'still  returns {r["returns"]["share_from_stationary"]:>6.1%} '
                  f'still  off-wall '
                  f'{r["returns"]["share_off_wall_from_stationary"]:>6.1%} still'
                  f'  dwell {dwell:>7.1f} s')
    print()

    failures = [s for s, r in rows.items()
                if r['B1']['all_scans'] < gw.B1_MIN_SHARE]
    print('=' * 100)
    print(f'{"map":<30}{"B1 all":>9}{"B1 moving":>11}{"B1 still":>10}'
          f'{"still scans":>13}{"still rets":>11}{"still off":>11}'
          f'{"dwell s":>9}{"dwell %":>9}')
    for stem, r in rows.items():
        mark = ' <-' if stem in failures else ''
        print(f'{stem:<30}{r["B1"]["all_scans"]:>8.2%}'
              f'{r["B1"]["moving_scans_only"]:>11.2%}'
              f'{r["B1"]["stationary_scans_only"]:>10.2%}'
              f'{r["scans"]["share_stationary"]:>13.1%}'
              f'{r["returns"]["share_from_stationary"]:>11.1%}'
              f'{r["returns"]["share_off_wall_from_stationary"]:>11.1%}'
              f'{(r["commands"]["dwell_after_last_command_s"] or 0.0):>9.1f}'
              f'{(r["commands"]["dwell_share_of_cut"] or 0.0):>9.1%}{mark}')
    print()

    # ── the three predictions, read off the table above ──────────────────────
    p1 = {}
    for stem in failures:
        r = rows[stem]['returns']
        p1[stem] = bool(r['share_off_wall_from_stationary']
                        > r['share_from_stationary'])
    ranked = sorted(rows, key=lambda s: -rows[s]['scans']['share_stationary'])
    p2_top4 = ranked[:4]
    p2 = set(p2_top4) == set(failures)
    p3 = {s: bool(r['off_wall_cells']['n_cells_free']
                  > r['off_wall_cells']['n_cells_unknown'])
          for s, r in rows.items() if r['off_wall_cells']['n_cells']}

    print('PREDICTIONS, read off the numbers above. Reported, never gated.')
    print(f'  P1  {"HELD" if all(p1.values()) else "FAILED"} '
          f'({sum(p1.values())}/{len(p1)} of the B1 failures)')
    for stem, held in p1.items():
        r = rows[stem]['returns']
        print(f'        {stem}: off-wall '
              f'{r["share_off_wall_from_stationary"]:.1%} still vs all kept '
              f'{r["share_from_stationary"]:.1%} still -> '
              f'{"over" if held else "UNDER"}-represented')
    print(f'  P2  {"HELD" if p2 else "FAILED"}')
    print(f'        the four highest stationary shares: '
          f'{", ".join(s.replace(gw.RUN_PREFIX + "_", "") for s in p2_top4)}')
    print(f'        the four B1 failures:              '
          f'{", ".join(s.replace(gw.RUN_PREFIX + "_", "") for s in failures)}')
    print(f'  P3  {"HELD" if all(p3.values()) else "FAILED"} '
          f'({sum(p3.values())}/{len(p3)} maps with any off-wall cell)')
    for stem, held in p3.items():
        c = rows[stem]['off_wall_cells']
        if not held:
            print(f'        {stem}: {c["n_cells_free"]} FREE vs '
                  f'{c["n_cells_unknown"]} UNKNOWN cells')
    worst = max(rows, key=lambda s: rows[s]['off_wall_cells']['n_cells'])
    c = rows[worst]['off_wall_cells']
    print(f'        worst map {worst}: {c["n_cells"]} cells, '
          f'{c["n_cells_free"]} FREE / {c["n_cells_unknown"]} UNKNOWN, '
          f'{c["n_returns_in_free_cells"]} returns in FREE cells')
    print('=' * 100)
    print()

    out = {
        'question': 'why B1 fails on four of the twenty b2maps grids',
        'hypothesis': 'the replay is deterministic and integrates every scan, so '
                      'a robot parked for a long stretch contributes thousands '
                      'of scans from one pose, and those scans can clear wall '
                      'cells that the moving scans had committed',
        'predictions': {name: text for name, text in PREDICTIONS},
        'predictions_registered': 'by Ali, in the request that produced this '
                                  'file, before any number in it was read',
        'stationary_test': {'m': STILL_M, 'deg': STILL_DEG,
                            'between': 'consecutive kept scans',
                            'first_kept_scan': 'counted as moving'},
        'truth_used': False,
        'gated': False,
        'touches': 'writes this file only; opens no map JSON and moves no '
                   'threshold',
        'poses': 'graph_walls.interpolate_poses on the same /tf samples Part B '
                 'uses; never /robot_K/odom',
        'verdicts': {
            'P1': {'held': bool(all(p1.values())), 'per_map': p1},
            'P2': {'held': bool(p2), 'top_four_by_stationary_share': p2_top4,
                   'b1_failures': failures},
            'P3': {'held': bool(all(p3.values())), 'per_map': p3},
        },
        'b1_failures': failures,
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
