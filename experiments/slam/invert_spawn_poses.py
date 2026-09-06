#!/usr/bin/env python3
"""Recover the five spawn poses from the maps, instead of assuming them.

Motivation: under convention A with world_T_odom o inverse(map_T_odom), robot_4
agrees with its own free fit to 0.98 deg and 0.19 m, while robot_2 -- a map that
fits the world at 88.2% -- misses by 1.17 m. A wrong row convention or a wrong
transform direction would break both good maps identically. A per-robot miss is
the signature of a wrong per-robot *input*, and the only per-robot inputs in the
chain are map_T_odom (measured, logged) and world_T_odom (assumed, from a launch
file three revisions back). This inverts the chain to measure the latter.

Three readings this is set up to distinguish:
  - pentagon fits, only robot_2's residual large -> the spawn table is right and
    robot_2 is the problem (it is the non-deterministic map, and its map->odom
    yaw moved 0.3 deg between runs)
  - fitted radius disagrees with the table -> the bag-era table is wrong
  - implied yaws non-zero and consistent -> the yaw-0 assumption is wrong, and
    the empirical check recorded in bag_overlap.py was checking something else

Pure numpy / scipy / PyYAML / PIL, via fit_world_transform.py. No ROS.

    venv/bin/python experiments/slam/invert_spawn_poses.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fit_world_transform as fwt  # noqa: E402
from fit_world_transform import (  # noqa: E402
    NUM_ROBOTS,
    WELL_FIT_FLOOR,
    die,
    rel,
    resolve_spawn_poses,
    utc_stamp,
    wrap_deg,
)

OUT_DIR = REPO_ROOT / 'experiments' / 'logs'

# Robots whose free fit clears WELL_FIT_FLOOR carry the fit; the rest are held
# down because a map that does not fit the world folds its own error straight
# into the implied pose. Not zero -- they still constrain the phase usefully,
# and dropping them entirely would leave 2 robots fitting 4 parameters, which is
# exactly determined and would report zero residual for any input whatsoever.
WEIGHT_GOOD = 1.0
WEIGHT_POOR = 0.2

# A peak is eligible for the pentagon search if it scores within this of that
# robot's best peak. Stops the search buying a better pentagon with a placement
# the wall evidence does not support.
PEAK_SCORE_BAND = 0.10

VERTEX_STEP_DEG = 72.0


def implied_world_t_odom(peak: dict, centroid: np.ndarray,
                         mto: tuple[float, float, float]) -> dict:
    """The spawn pose implied by one free-fit peak.

    The free fit F is the total map-cell -> world transform, and the composition
    that agreed for robot_4 is F = world_T_odom o inverse(map_T_odom). Right-
    composing with map_T_odom cancels the inverse:

        world_T_odom = F o map_T_odom

    Note this is F o map_T_odom, NOT F o inverse(map_T_odom): the latter applies
    the inverse twice. Checked numerically -- for robot_4 this composition
    returns (+1.064, -3.599), against a table value of (1.200, -3.800), while
    the double-inverted form returns (+3.081, -3.903).
    """
    t = math.radians(peak['theta'])
    rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    f = np.eye(3)
    f[:2, :2] = rot
    f[:2, 2] = centroid + np.array([peak['dx'], peak['dy']]) - rot @ centroid

    w = f @ fwt.rigid(*mto)
    return {'x': float(w[0, 2]), 'y': float(w[1, 2]),
            'yaw_deg': math.degrees(math.atan2(w[1, 0], w[0, 0])),
            'score': peak['score'], 'theta': peak['theta'],
            'dx': peak['dx'], 'dy': peak['dy']}


def fit_pentagon(xy: np.ndarray, weights: np.ndarray) -> dict:
    """Weighted least-squares pentagon: common centre, common radius, 72 deg apart.

    Nonlinear in the phase, but substituting u = r cos(phase), v = r sin(phase)
    makes it exactly linear in (cx, cy, u, v), so this is a closed-form solve
    with no iteration and no starting guess:

        x_i = cx + u cos(a_i) - v sin(a_i)
        y_i = cy + u sin(a_i) + v cos(a_i),   a_i = i * 72 deg
    """
    n = len(xy)
    a = np.radians(np.arange(n) * VERTEX_STEP_DEG)
    rows, rhs, wts = [], [], []
    for i in range(n):
        rows.append([1.0, 0.0, math.cos(a[i]), -math.sin(a[i])])
        rhs.append(xy[i, 0])
        rows.append([0.0, 1.0, math.sin(a[i]), math.cos(a[i])])
        rhs.append(xy[i, 1])
        wts += [weights[i], weights[i]]

    design = np.asarray(rows)
    target = np.asarray(rhs)
    sw = np.sqrt(np.asarray(wts))
    sol, *_ = np.linalg.lstsq(design * sw[:, None], target * sw, rcond=None)
    cx, cy, u, v = sol

    pred = np.column_stack([cx + u * np.cos(a) - v * np.sin(a),
                            cy + u * np.sin(a) + v * np.cos(a)])
    resid = np.linalg.norm(xy - pred, axis=1)
    return {
        'centre': [float(cx), float(cy)],
        'radius': float(math.hypot(u, v)),
        'phase_deg': math.degrees(math.atan2(v, u)),
        'predicted': pred,
        'residuals': resid,
        'weighted_ssr': float((weights * resid ** 2).sum()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--convention', choices=['A', 'B'], default='A',
                    help='row convention (default A -- the one that agreed)')
    args = ap.parse_args()

    stamp, iso = utc_stamp()
    rects = fwt.world_walls()
    mask, wx0, wy0 = fwt.build_wall_mask(rects)
    thetas = np.arange(-180.0, 180.0, fwt.THETA_STEP)
    table = resolve_spawn_poses()

    print(f'inverting the chain under convention {args.convention}: '
          'world_T_odom = free_fit o map_T_odom\n')

    per_robot = []
    for n in range(NUM_ROBOTS):
        m = fwt.load_map(n)
        mto = fwt.load_map_to_odom(n)
        pts = fwt.cell_points(m, args.convention)
        centroid = pts.mean(axis=0)
        peaks, _ = fwt.free_fit(pts, centroid, mask, wx0, wy0, rects, thetas)
        best = peaks[0]['score']
        cands = [implied_world_t_odom(p, centroid, mto) for p in peaks
                 if p['score'] >= best - PEAK_SCORE_BAND]
        per_robot.append({'robot': n, 'best_free_fit': best, 'candidates': cands,
                          'map_T_odom_yaw_deg': math.degrees(mto[2])})
        print(f'  robot_{n}: free fit {100 * best:5.1f}%, '
              f'{len(cands)} peak(s) within {100 * PEAK_SCORE_BAND:.0f} pp')

    weights = np.array([WEIGHT_GOOD if r['best_free_fit'] >= WELL_FIT_FLOOR
                        else WEIGHT_POOR for r in per_robot])

    print('\n' + '=' * 92)
    print('every peak candidate, and how far its implied pose sits from the '
          'bag-era table')
    print('  the world is 180 deg symmetric, so each map has a twin placement '
          'scoring the')
    print('  same; both are listed. This table is the primary result -- it needs no '
          'pentagon.')
    print(f'{"":>9}{"pk":>4}{"fit%":>7}{"impl x":>9}{"impl y":>9}{"yaw":>9}'
          f'{"err vs table":>14}')
    for r in per_robot:
        n = r['robot']
        for i, cand in enumerate(r['candidates']):
            err = math.hypot(cand['x'] - table[n][0], cand['y'] - table[n][1])
            flag = '  <-- reproduces the table' if err < 0.6 else ''
            print(f'  robot_{n}{i + 1:>4}{100 * cand["score"]:6.1f}%{cand["x"]:9.3f}'
                  f'{cand["y"]:9.3f}{cand["yaw_deg"]:9.2f}{err:14.3f}{flag}')
        print()

    # Resolve each twin by proximity to the table entry. Selecting by pentagon
    # SSR instead does not work: with two robots at full weight against four
    # pentagon parameters the fit is free enough to buy a lower SSR by picking a
    # twin that is metres away (it chose robot_2's 9.2 m twin over its 1.2 m
    # one). Choosing the nearer twin is a weak assumption -- the twins are
    # metres apart, so this only decides which side of the world a map sits on,
    # not the residual being measured -- and the discarded twin's distance is in
    # the table above for checking.
    best_combo = tuple(
        min(range(len(r['candidates'])),
            key=lambda k: math.hypot(r['candidates'][k]['x'] - table[r['robot']][0],
                                     r['candidates'][k]['y'] - table[r['robot']][1]))
        for r in per_robot)
    chosen = [per_robot[i]['candidates'][k] for i, k in enumerate(best_combo)]
    xy = np.array([[c['x'], c['y']] for c in chosen])
    best_fit = fit_pentagon(xy, weights)

    tbl_r = [math.hypot(x, y) for x, y in table]
    tbl_a = [math.degrees(math.atan2(y, x)) for x, y in table]

    print('\n' + '=' * 92)
    print('implied spawn poses (world_T_odom recovered from the maps)')
    print(f'{"":>9}{"wt":>5}{"fit%":>7}{"x":>9}{"y":>9}{"yaw":>9}   '
          f'{"table x":>9}{"table y":>9}{"err":>8}')
    for i, c in enumerate(chosen):
        ex, ey = c['x'] - table[i][0], c['y'] - table[i][1]
        print(f'  robot_{i}{weights[i]:5.1f}{100 * c["score"]:6.1f}%'
              f'{c["x"]:9.3f}{c["y"]:9.3f}{c["yaw_deg"]:9.2f}   '
              f'{table[i][0]:9.3f}{table[i][1]:9.3f}{math.hypot(ex, ey):8.3f}')

    n_good = int((weights == WEIGHT_GOOD).sum())
    identifiable = n_good >= 4
    print('\n' + '=' * 92)
    print('pentagon fit -- common centre, common radius, 72 deg spacing')
    if not identifiable:
        print(f'  UNDER-DETERMINED: the pentagon has 4 free parameters and only '
              f'{n_good} robot(s)')
        print(f'  clear the {100 * WELL_FIT_FLOOR:.0f}% floor. The three down-weighted '
              'maps fold their own error')
        print('  into the implied pose, so they cannot pin the shape. Read the fitted')
        print('  radius below as indicative only -- it is NOT a measurement of the')
        print('  spawn ring. The per-robot comparison against the table is the result.')
        print()
    print(f'  centre  ({best_fit["centre"][0]:+.3f}, {best_fit["centre"][1]:+.3f}) m')
    print(f'  radius   {best_fit["radius"]:.3f} m'
          f'{"" if identifiable else "   (indicative only, see above)"}')
    print(f'  phase    {best_fit["phase_deg"]:+.2f} deg')
    print(f'  weights  robot_2/robot_4 = {WEIGHT_GOOD}, others = {WEIGHT_POOR} '
          f'(free fit vs the {100 * WELL_FIT_FLOOR:.0f}% floor)')
    print()
    print(f'{"":>9}{"residual":>10}{"implied r":>11}{"table r":>10}{"dr":>8}'
          f'{"implied a":>11}{"table a":>10}{"da":>8}')
    for i, c in enumerate(chosen):
        ir = math.hypot(c['x'] - best_fit['centre'][0], c['y'] - best_fit['centre'][1])
        ia = math.degrees(math.atan2(c['y'] - best_fit['centre'][1],
                                     c['x'] - best_fit['centre'][0]))
        print(f'  robot_{i}{best_fit["residuals"][i]:10.3f}{ir:11.3f}{tbl_r[i]:10.3f}'
              f'{ir - tbl_r[i]:8.3f}{ia:11.2f}{tbl_a[i]:10.2f}'
              f'{wrap_deg(ia - tbl_a[i]):8.2f}')

    print(f'\n  the table is not itself a regular pentagon -- robot_0 sits at '
          f'{tbl_r[0]:.3f} m while')
    print(f'  the other four sit at {np.mean(tbl_r[1:]):.3f} m.')
    print('\n  radius, from the maps that can actually carry it:')
    for i in range(NUM_ROBOTS):
        if weights[i] != WEIGHT_GOOD:
            continue
        ir = math.hypot(chosen[i]['x'], chosen[i]['y'])
        print(f'    robot_{i}: implied {ir:.3f} m vs table {tbl_r[i]:.3f} m '
              f'({ir - tbl_r[i]:+.3f} m)')

    print('\n' + '=' * 92)
    print('implied spawn yaws -- resolve_spawn_poses() applies yaw 0')
    yaws = np.array([c['yaw_deg'] for c in chosen])
    for i, c in enumerate(chosen):
        print(f'  robot_{i}{c["yaw_deg"]:9.2f} deg   '
              f'(map_T_odom yaw {per_robot[i]["map_T_odom_yaw_deg"]:+7.2f} deg)')
    good = [i for i in range(NUM_ROBOTS) if weights[i] == WEIGHT_GOOD]
    gy = yaws[good]
    print(f'\n  weighted robots ({", ".join(f"robot_{i}" for i in good)}): '
          f'mean {gy.mean():+.2f} deg, spread {gy.max() - gy.min():.2f} deg')
    print(f'  all five: mean {yaws.mean():+.2f} deg, '
          f'spread {yaws.max() - yaws.min():.2f} deg')
    print('\n  A non-zero *consistent* yaw across the weighted robots would mean the')
    print('  yaw-0 assumption is wrong. Scatter of the same order as the values')
    print('  themselves means they are fit noise and say nothing either way.')
    if np.all(np.abs(gy) < 10.0) and gy.max() - gy.min() < 10.0:
        print(f'\n  -> both weighted robots straddle zero within '
              f'{np.abs(gy).max():.1f} deg. The yaw-0')
        print('     assumption in resolve_spawn_poses() is NOT contradicted.')
    elif gy.max() - gy.min() < 10.0:
        print(f'\n  -> both weighted robots agree on {gy.mean():+.2f} deg, which is not '
              'zero. The')
        print('     yaw-0 assumption looks wrong; re-read the check in bag_overlap.py.')
    else:
        print(f'\n  -> the weighted robots disagree by {gy.max() - gy.min():.1f} deg, '
              'so these are fit')
        print('     noise and say nothing about the yaw-0 assumption either way.')
    print('=' * 92)

    out = {
        'generated_utc': iso,
        'convention': args.convention,
        'composition': 'world_T_odom = free_fit o map_T_odom',
        'weights': {f'robot_{i}': float(weights[i]) for i in range(NUM_ROBOTS)},
        'well_fit_floor': WELL_FIT_FLOOR,
        'peak_score_band': PEAK_SCORE_BAND,
        'implied_poses': {f'robot_{i}': chosen[i] for i in range(NUM_ROBOTS)},
        'all_peak_candidates': {f'robot_{r["robot"]}': r['candidates']
                                for r in per_robot},
        'pentagon': {'centre': best_fit['centre'], 'radius': best_fit['radius'],
                     'phase_deg': best_fit['phase_deg'],
                     'residuals': {f'robot_{i}': float(best_fit['residuals'][i])
                                   for i in range(NUM_ROBOTS)},
                     'weighted_ssr': best_fit['weighted_ssr']},
        'bag_era_table': {f'robot_{i}': {'x': table[i][0], 'y': table[i][1],
                                         'radius': tbl_r[i], 'angle_deg': tbl_a[i]}
                          for i in range(NUM_ROBOTS)},
        'implied_yaws_deg': {f'robot_{i}': chosen[i]['yaw_deg']
                             for i in range(NUM_ROBOTS)},
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / f'spawn_inversion_{stamp}.json'
    path.write_text(json.dumps(out, indent=2))
    print(f'\nwrote {rel(path)}')


if __name__ == '__main__':
    main()
