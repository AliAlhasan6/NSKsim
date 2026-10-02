#!/usr/bin/env python3
"""Range-edge clipping: the last untested candidate for the cleared wall cells.

A DIAGNOSIS, and nothing else. Writes one file,
experiments/logs/graph_walls/diagnose_edge.json, and touches no map JSON, no
threshold and no reader. diagnose_karto.py is imported and NOT modified: its
rebuild produces the baseline here exactly as committed, and this file only adds
one bucket beside it.

THE CANDIDATE. Karto.h:6148-6192: a finite return with
rangeThreshold <= r < maxRange is rescaled to rangeThreshold along its own ray
and traced FREE-ONLY, because isEndPointValid = r < rangeThreshold - KT_TOLERANCE
is false for it (:6167, :6173). Its last traced cell therefore lies within 0.1 m
in front of the surface it actually hit -- often that surface's own cell. Such a
ray leaves passes where a wall is and never a hit. diagnose_karto counted these
together with ordinary finite rays, so this separates them.

WHERE THE BAND COMES FROM, confirmed from source (~/src/slam_toolbox_2.8.5,
ros-jazzy-slam-toolbox 2.8.5, orig tarball SHA256 80af3955...608583):

  maxRange        src/laser_utils.cpp:104, laser->SetMaximumRange(scan_.range_max)
                  -- the SCAN MESSAGE's own range_max, not a parameter. These
                  bags carry 8.0.
  rangeThreshold  src/laser_utils.cpp:123-129 reads the max_laser_range parameter
                  (default 25); :139-146 clamp it to scan_.range_max if it is
                  larger, then laser->SetRangeThreshold(max_laser_range). The
                  replay config sets 7.9. Karto clips it again into
                  [minRange, maxRange] at Karto.h:3949.

  so the band is [7.9, 8.0).

AND WHAT "EDGE" MUST THEREFORE MEAN. The free_space_relay fills a no-return beam
at 0.5*(7.9 + 8.0) = 7.95, which is itself INSIDE that band -- so the band alone
does not separate the two populations. An edge ray here is a ray whose RAW
reading was finite and in [7.9, 8.0): a real return near the sensor's limit that
the relay never touched. Those are exactly the rays diagnose_karto has in its
finite buckets. Relay-filled rays keep their own buckets and are not counted
twice.

Edge rays cannot produce a hit -- r >= rangeThreshold makes isEndPointValid false
-- which the code asserts rather than assumes, so counterfactual (d) removes
passes only.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/diagnose_edge.py --map b2maps_k4_cut120_robot4
    python3 experiments/analysis/diagnose_edge.py

About five minutes for the corpus: diagnose_karto's rebuild again (2 min 45 s),
plus a second pass over the same bags to find the edge rays, plus the ray-distance
distributions for two maps.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import diagnose_karto as dk                                  # noqa: E402
import graph_walls as gw                                     # noqa: E402
from trinary_map import (  # noqa: E402
    cell_centre,
    cell_centre_provenance,
)

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_edge.json'
KARTO_JSON = gw.OUT_DIR_DEFAULT / 'diagnose_karto.json'

# Step 4 asks for these two maps: the B1 failures with no dwell, the ones every
# earlier candidate has failed to explain.
DISTANCE_MAPS = ('b2maps_k0_cut240_robot0', 'b2maps_k4_cut120_robot4')
E2_MAP = 'b2maps_k4_cut120_robot4'
E2_FACE = 'wall_north'

E1_MIN_SHARE = 0.50
E2_MIN_SHARE = 0.80

PREDICTIONS = [
    ('E1', 'on k0_cut240 and k4_cut120, removing (d) recommits >= 50 % of '
           'off-wall cells'),
    ('E2', 'on k4_cut120, removing (d) recommits >= 80 % of the off-wall cells '
           'within 2*rho of wall_north'),
]


def edge_rays(stem: str, k: int, cut: int, rb: dict) -> dict:
    """The edge rays of one map, traced, plus their per-cell passes.

    Re-reads the bag because the baseline's kept returns cannot contain these
    rays at all: graph_walls.scan_returns keeps only r < r_max, and every edge
    ray is at or above it. The pose chain is recomputed with the same graph_walls
    calls and then CHECKED against the baseline's own sensor arrays, so the two
    cannot have drifted apart.
    """
    data = gw.read_bag(gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}_cut{cut}_slamin', k)
    geom = data['geom']
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    r_max, _src = gw.read_replay_r_max(k, cut)
    returns = gw.scan_returns(data, poses, base_scan, (0.0, 0.0, 0.0), r_max)
    for key in ('sensor_x', 'sensor_y', 'sensor_yaw'):
        if not np.array_equal(returns[key], rb['returns'][key]):
            gw.die(f'{stem}: the second read of the bag gives different '
                   f'{key} than the rebuild did, so the two are not describing '
                   'the same scans')

    rho, origin = rb['rho'], rb['origin']
    h, w = rb['shape']
    range_threshold = rb['range_threshold']
    raw = data['ranges']
    # The relay rewrote +inf and readings at or above range_max; an EDGE ray is a
    # finite raw reading in [rangeThreshold, range_max) that it left alone.
    _filled_ranges, filled = dk.relay_fill(raw, geom['range_max'],
                                           range_threshold)
    is_edge = (np.isfinite(raw) & ~filled
               & (raw >= range_threshold) & (raw < geom['range_max']))
    reach = dk.reaching_grid(poses[3])
    stat_scan = dk.stationary_scans(returns['sensor_x'], returns['sensor_y'],
                                    returns['sensor_yaw'], poses[3])

    beam = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']
    n_cells = h * w
    passes = np.zeros(n_cells, dtype=np.int64)
    n_rays = 0
    n_still = 0
    scan_idx = np.nonzero(reach)[0]
    for lo in range(0, scan_idx.size, 128):
        sel = scan_idx[lo:lo + 128]
        mask = is_edge[sel]
        if not mask.any():
            continue
        si, bi = np.nonzero(mask)
        n_rays += si.size
        n_still += int(np.count_nonzero(stat_scan[sel][si]))
        ox = returns['sensor_x'][sel][si]
        oy = returns['sensor_y'][sel][si]
        ang = returns['sensor_yaw'][sel][si] + beam[bi]
        # Karto.h:6173 -- rescaled to the threshold, then traced free-only.
        px = ox + range_threshold * np.cos(ang)
        py = oy + range_threshold * np.sin(ang)
        gx0, gy0 = dk.world_to_grid(ox, oy, origin, rho)
        gx1, gy1 = dk.world_to_grid(px, py, origin, rho)
        cx, cy, _ray = dk.trace_rays(gx0, gy0, gx1, gy1)
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        if inside.any():
            passes += np.bincount((cy[inside] * w + cx[inside]),
                                  minlength=n_cells)
    return {'passes': passes.reshape(h, w), 'n_rays': n_rays,
            'n_rays_stationary': n_still,
            'n_beams_total': int(raw.size),
            'band': [float(range_threshold), float(geom['range_max'])]}


def ray_distances(stem: str, k: int, cut: int, rb: dict,
                  off_flat: np.ndarray) -> dict:
    """Step 4. How far from the sensor were the off-wall cells, hit vs passed.

    Distance is from the ray's sensor origin to the KARTO centre of the cell in
    question, so a hit and a pass are measured the same way and the two
    distributions are comparable. Every ray of every scan that reached the grid
    is traced; only entries landing in an off-wall cell are kept.
    """
    data = gw.read_bag(gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}_cut{cut}_slamin', k)
    geom = data['geom']
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    r_max, _src = gw.read_replay_r_max(k, cut)
    returns = gw.scan_returns(data, poses, base_scan, (0.0, 0.0, 0.0), r_max)
    rho, origin = rb['rho'], rb['origin']
    h, w = rb['shape']
    range_threshold = rb['range_threshold']
    ranges, _filled = dk.relay_fill(data['ranges'], geom['range_max'],
                                    range_threshold)
    reach = dk.reaching_grid(poses[3])
    beam = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']
    off_set = np.zeros(h * w, dtype=bool)
    off_set[off_flat] = True

    hit_d, pass_d = [], []
    scan_idx = np.nonzero(reach)[0]
    for lo in range(0, scan_idx.size, 128):
        sel = scan_idx[lo:lo + 128]
        r = ranges[sel]
        ox = returns['sensor_x'][sel][:, None]
        oy = returns['sensor_y'][sel][:, None]
        ang = returns['sensor_yaw'][sel][:, None] + beam[None, :]
        use = (~np.isnan(r)) & (r > geom['range_min']) & (r < geom['range_max'])
        over = use & (r >= range_threshold)
        px = ox + np.where(over, range_threshold, r) * np.cos(ang)
        py = oy + np.where(over, range_threshold, r) * np.sin(ang)
        valid_end = use & (r < range_threshold - dk.KT_TOLERANCE)
        si, bi = np.nonzero(use)
        if si.size == 0:
            continue
        sox = np.broadcast_to(ox, r.shape)[si, bi]
        soy = np.broadcast_to(oy, r.shape)[si, bi]
        gx0, gy0 = dk.world_to_grid(sox, soy, origin, rho)
        gx1, gy1 = dk.world_to_grid(px[si, bi], py[si, bi], origin, rho)
        cx, cy, ray = dk.trace_rays(gx0, gy0, gx1, gy1)
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        cx, cy, ray = cx[inside], cy[inside], ray[inside]
        flat = cy * w + cx
        keep = off_set[flat]
        if keep.any():
            d = np.hypot(
                cell_centre(origin[0], cx[keep], rho) - sox[ray[keep]],
                cell_centre(origin[1], cy[keep], rho) - soy[ray[keep]])
            pass_d.append(d)
        ve = valid_end[si, bi]
        end_ok = ve & (gx1 >= 0) & (gx1 < w) & (gy1 >= 0) & (gy1 < h)
        if end_ok.any():
            eflat = gy1[end_ok] * w + gx1[end_ok]
            k2 = off_set[eflat]
            if k2.any():
                hit_d.append(np.hypot(
                    cell_centre(origin[0], gx1[end_ok][k2], rho)
                    - sox[end_ok][k2],
                    cell_centre(origin[1], gy1[end_ok][k2], rho)
                    - soy[end_ok][k2]))

    def summary(parts):
        if not parts:
            return {'n': 0}
        v = np.concatenate(parts)
        return {'n': int(v.size), 'median_m': float(np.median(v)),
                'p10_m': float(np.percentile(v, 10)),
                'p90_m': float(np.percentile(v, 90)),
                'max_m': float(v.max()),
                'share_at_or_over_7p9': float(
                    np.count_nonzero(v >= range_threshold) / v.size)}

    return {'definition': 'distance from the ray sensor origin to the Karto '
                          'centre of the off-wall cell; hits and passes '
                          'measured the same way',
            'hit': summary(hit_d), 'passed_through': summary(pass_d)}


def recommitted(rb: dict, edge_passes: np.ndarray,
                off_flat: np.ndarray) -> dict:
    """Counterfactual (d): the edge passes removed, nothing else.

    Edge rays contribute no hits, which is asserted rather than assumed: a ray in
    the band has r >= rangeThreshold, so isEndPointValid (Karto.h:6167) is false.
    So this subtracts passes and leaves the hit counts alone.
    """
    keys = ('finite_moving', 'finite_still', 'filled_moving', 'filled_still')
    total = sum(rb['passes'][key] for key in keys)
    hits = rb['hits']['moving'] + rb['hits']['still']
    if np.any(edge_passes > total):
        gw.die('edge passes exceed the total passes in some cell, so the edge '
               'rays are not a subset of the rays the rebuild traced')
    base = dk.update_cell(total, hits).reshape(-1)
    after = dk.update_cell(total - edge_passes, hits).reshape(-1)
    if not off_flat.size:
        return {'n_off_wall_cells': 0, 'n_recommitted': 0,
                'share_recommitted': None}
    was_not = base[off_flat] != dk.K_OCCUPIED
    now_occ = after[off_flat] == dk.K_OCCUPIED
    n_was = int(np.count_nonzero(was_not))
    return {'n_off_wall_cells': int(off_flat.size),
            'n_not_occupied_in_rebuild': n_was,
            'n_recommitted': int(np.count_nonzero(was_not & now_occ)),
            'share_recommitted': (float(np.count_nonzero(was_not & now_occ))
                                  / n_was) if n_was else None,
            'edge_passes_in_off_wall_cells':
                int(edge_passes.reshape(-1)[off_flat].sum())}


def near_model_cells(rb: dict, off_flat: np.ndarray, spawn, model: str) -> dict:
    """E2's subset: the off-wall cells within 2*rho of one SDF model."""
    from bag_overlap import (dist_to_nearest_wall,              # noqa: PLC0415
                             world_walls_named)

    rects = [r for n, r in world_walls_named() if n == model]
    if not rects:
        gw.die(f'no SDF model named {model}')
    h, w = rb['shape']
    rho, origin = rb['rho'], rb['origin']
    cy, cx = np.divmod(off_flat, w)
    pts = np.column_stack([cell_centre(origin[0], cx, rho) + spawn[0],
                           cell_centre(origin[1], cy, rho) + spawn[1]])
    d = dist_to_nearest_wall(pts, rects)
    return {'mask': d <= 2.0 * rho, 'n': int(np.count_nonzero(d <= 2.0 * rho))}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps',
                    help='one map stem; repeatable. Default: all 20 b2maps cuts')
    args = ap.parse_args()
    stems = args.maps or gw.corpus_stems()

    print('range-edge clipping: the last untested candidate')
    print('  a finite return with 7.9 <= r < 8.0 is rescaled to 7.9 and traced '
          'FREE-ONLY')
    print('  (Karto.h:6167, :6173), so it leaves passes within 0.1 m in front '
          'of the surface')
    print('  it hit -- often that surface\'s own cell -- and never a hit.')
    print()
    print('  maxRange = the SCAN\'s range_max (laser_utils.cpp:104), not a '
          'parameter: 8.0.')
    print('  rangeThreshold = the max_laser_range parameter '
          '(laser_utils.cpp:123-146): 7.9.')
    print('  The relay fills no-return beams at 7.95, INSIDE that band, so an '
          'edge ray here is')
    print('  a finite RAW reading in [7.9, 8.0) that the relay never touched.')
    print()
    print('PRE-REGISTERED, printed before any number is read:')
    for name, text in PREDICTIONS:
        print(f'  {name}  {text}')
    print()

    baseline = {}
    if KARTO_JSON.is_file():
        baseline = json.loads(KARTO_JSON.read_text())['per_map']
        print(f'baseline: comparing every rebuild against the committed '
              f'{KARTO_JSON.name}')
    else:
        print(f'baseline: {KARTO_JSON.name} is absent, so the rebuild cannot be '
              'checked against it')
    print()

    from fit_world_transform import resolve_spawn_poses       # noqa: PLC0415
    spawn = resolve_spawn_poses(gw.SPAWN_REV)

    rows, mismatch = {}, []
    print(f'{"map":<30}{"agree":>9}{"edge rays":>11}{"of beams":>10}'
          f'{"off cells":>11}{"(d) recommitted":>17}')
    for stem in stems:
        k, cut = gw._k_and_cut(stem, 'the edge test needs the bag and the '
                                     'replay config')
        rb = dk.rebuild(stem, k, cut)
        g = dk.gate(rb)
        base = baseline.get(stem)
        if base is not None:
            same = (base['gate']['agreement'] == g['agreement']
                    and base['passes_total'] == {kk: int(v.sum()) for kk, v
                                                 in rb['passes'].items()}
                    and base['hits_total'] == {kk: int(v.sum()) for kk, v
                                               in rb['hits'].items()})
            if not same:
                mismatch.append(stem)
        off_flat, b1stats = dk.off_wall_cells(rb)
        edge = edge_rays(stem, k, cut, rb)
        cf = recommitted(rb, edge['passes'], off_flat)
        row = {'map': stem, 'robot': k, 'cut_s': cut,
               'gate': {'agreement': g['agreement'], 'holds': g['holds']},
               'baseline_matches_diagnose_karto': (None if base is None
                                                   else bool(same)),
               'band': edge['band'],
               'edge_rays': edge['n_rays'],
               'edge_rays_stationary': edge['n_rays_stationary'],
               'beams_total': edge['n_beams_total'],
               'edge_ray_share_of_beams': (edge['n_rays']
                                           / edge['n_beams_total']),
               'edge_passes_total': int(edge['passes'].sum()),
               'b1_share': b1stats['b1_share'],
               'counterfactual_d': cf}
        if stem in DISTANCE_MAPS:
            row['ray_distances'] = ray_distances(stem, k, cut, rb, off_flat)
        if stem == E2_MAP:
            near = near_model_cells(rb, off_flat, spawn[k], E2_FACE)
            sub = off_flat[near['mask']]
            row['e2_subset'] = {
                'model': E2_FACE, 'n_cells': near['n'],
                **recommitted(rb, edge['passes'], sub)}
        rows[stem] = row
        share = cf['share_recommitted']
        print(f'{stem:<30}{g["agreement"]:>8.2%}{edge["n_rays"]:>11}'
              f'{edge["n_rays"] / edge["n_beams_total"]:>9.3%}'
              f'{off_flat.size:>11}'
              f'{(f"{share:.1%}" if share is not None else "-"):>17}'
              + ('' if g['holds'] else '  <- GATE FAILED'))
    print()

    gate_ok = [s for s, r in rows.items() if r['gate']['holds']]
    print(f'GATE: {len(gate_ok)}/{len(rows)} maps still hold')
    if baseline:
        print(f'BASELINE: reproduces the committed {KARTO_JSON.name} on '
              f'{len(rows) - len(mismatch)}/{len(rows)} maps'
              + ('' if not mismatch else f' -- MISMATCH on {mismatch}'))
    print()

    e1 = {}
    for stem in DISTANCE_MAPS:
        if stem in rows:
            s = rows[stem]['counterfactual_d']['share_recommitted']
            e1[stem] = {'share': s,
                        'holds': bool(s is not None and s >= E1_MIN_SHARE)}
    e2 = {}
    if E2_MAP in rows and 'e2_subset' in rows[E2_MAP]:
        s = rows[E2_MAP]['e2_subset']['share_recommitted']
        e2[E2_MAP] = {
            'share': s, 'n_cells': rows[E2_MAP]['e2_subset']['n_cells'],
            'holds': bool(s is not None and s >= E2_MIN_SHARE)}

    def verdict(d):
        if not d:
            return 'NOT EVALUATED'
        return 'HELD' if all(v['holds'] for v in d.values()) else 'FAILED'

    print('PREDICTIONS, read off the numbers above. Reported, never gated.')
    print(f'  E1  {verdict(e1)}')
    for stem, v in e1.items():
        shown = f'{100 * v["share"]:.1f}%' if v['share'] is not None else 'n/a'
        print(f'        {stem}: {shown} recommitted -> '
              f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    print(f'  E2  {verdict(e2)}')
    for stem, v in e2.items():
        shown = f'{100 * v["share"]:.1f}%' if v['share'] is not None else 'n/a'
        print(f'        {stem}: {shown} of the {v["n_cells"]} off-wall cells '
              f'within 2*rho of {E2_FACE} -> '
              f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    print()

    for stem in DISTANCE_MAPS:
        if stem in rows and 'ray_distances' in rows[stem]:
            rd = rows[stem]['ray_distances']
            print(f'  {stem} off-wall cells, distance from the sensor:')
            for key in ('hit', 'passed_through'):
                s = rd[key]
                if s['n']:
                    print(f'      {key:<15} n {s["n"]:>9}  median '
                          f'{s["median_m"]:.2f} m  p10 {s["p10_m"]:.2f}  p90 '
                          f'{s["p90_m"]:.2f}  max {s["max_m"]:.2f}  '
                          f'share >= 7.9 m {s["share_at_or_over_7p9"]:.1%}')
                else:
                    print(f'      {key:<15} none')
    print()

    residual = {s: {'b1_share': r['b1_share'],
                    'n_off_wall_cells': r['counterfactual_d']
                    ['n_off_wall_cells'],
                    'share_recommitted_by_d': r['counterfactual_d']
                    ['share_recommitted']}
                for s, r in rows.items()}
    e1_held = bool(e1) and all(v['holds'] for v in e1.values())
    if not e1_held:
        print('E1 did not hold, so by the stopping rule agreed before this run '
              'the diagnosis ends')
        print('  here and the residual is recorded as unexplained. There is no '
              'fifth candidate.')
        print()

    out = {
        'question': 'does range-edge clipping explain the cleared wall cells',
        'candidate': 'a finite return with rangeThreshold <= r < maxRange is '
                     'rescaled to rangeThreshold and traced free-only '
                     '(Karto.h:6167, :6173), leaving passes within 0.1 m in '
                     'front of the surface it hit and never a hit',
        'provenance': {
            'source_tree': str(dk.SOURCE_TREE),
            'package': 'ros-jazzy-slam-toolbox 2.8.5',
            'orig_tarball_sha256': dk.SOURCE_SHA256,
            'copied_into_repo': False,
            'max_range': 'src/laser_utils.cpp:104, '
                         'laser->SetMaximumRange(scan_.range_max) -- the scan '
                         "message's own range_max (8.0 here), not a parameter",
            'range_threshold': 'src/laser_utils.cpp:123-129 reads the '
                               'max_laser_range parameter (default 25); '
                               ':139-146 clamp it to scan_.range_max; :146 '
                               'SetRangeThreshold. Karto clips again at '
                               'Karto.h:3949. The replay config sets 7.9',
            'band': [7.9, 8.0],
            'edge_definition': 'a finite RAW reading in [rangeThreshold, '
                               'maxRange) that the relay did not rewrite. The '
                               'relay fills no-return beams at 7.95, inside the '
                               'band, so the band alone does not separate them',
            'baseline': 'diagnose_karto.rebuild, imported and unmodified; every '
                        'map checked against the committed diagnose_karto.json',
            'cell_centre': cell_centre_provenance(),
        },
        'predictions': {name: text for name, text in PREDICTIONS},
        'gated': False,
        'truth_used_for': "E2's wall_north subset only",
        'gate': {'maps_holding': len(gate_ok), 'maps': len(rows)},
        'baseline_mismatches': mismatch,
        'verdicts': {'E1': {'per_map': e1, 'held': e1_held,
                            'evaluated': bool(e1)},
                     'E2': {'per_map': e2,
                            'held': bool(e2 and all(v['holds']
                                                    for v in e2.values())),
                            'evaluated': bool(e2)}},
        'residual': residual,
        'stopping_rule': 'agreed before this run: if E1 fails, the residual is '
                         'recorded in spec §12 as unexplained and the diagnosis '
                         'ends. There is no fifth candidate',
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
