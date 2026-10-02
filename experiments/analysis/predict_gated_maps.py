#!/usr/bin/env python3
"""Predict the 20 travel-gated maps cell by cell, before slam_toolbox
builds them.

A PREDICTION, written before the replay that will test it. It reads today's
stripped bags, applies the proposed travel gate in Python, rebuilds what Karto
would produce from the admitted scans alone, and writes the result as a PGM/YAML
pair per map so the eventual comparison can use the readers this project already
has.

THE GATE RULE, as agreed. Admit a scan only if the sensor pose has moved
>= 0.01 m or turned >= 0.5 deg since the LAST ADMITTED scan; the first scan is
always admitted. Measuring against the last admitted scan rather than the last
scan is what keeps slow creep from being dropped scan after scan.

The pose is the Part B chain, odom_T_base(t) composed with base_T_scan from
/tf_static. map_T_odom is deliberately NOT needed: it is a rigid transform, and a
rigid transform cannot change a displacement or a heading change, so the gate's
decisions are invariant to it. That also means the gate needs nothing but the bag.

WHAT ELSE THE PREDICTION MUST MODEL, all from source (~/src/slam_toolbox_2.8.5,
ros-jazzy-slam-toolbox 2.8.5, SHA256 80af3955...608583):

  which scans reach Karto   the admitted scans minus the 2nd, 3rd and 4th of the
                            ADMITTED sequence: SlamToolbox::shouldProcessScan
                            drops scan_ctr < 5 (slam_toolbox_common.cpp:795-797)
                            and Karto's HasMovedEnough admits everything on the
                            time test with minimum_time_interval 0.0
                            (Mapper.cpp:3155-3158).
  the grid's extent         OccupancyGrid::ComputeDimensions (Karto.h:6090-6114):
                            offset = the minimum corner of the union of the
                            processed scans' bounding boxes, width and height =
                            Round(size * scale). A scan's own bounding box is its
                            SENSOR POSITION plus every FILTERED point reading
                            (Karto.h:5694-5700), filtered meaning
                            minRange <= r <= rangeThreshold (Math.h:172-175,
                            inclusive at both ends). Dropping scans can only
                            shrink that box, so A GATED MAP SITS ON ITS OWN
                            LATTICE and is not a subgrid of today's.
  hits and passes           diagnose_karto's rules, imported, not restated.

TWO SELF-CHECKS BEFORE ANY PREDICTION IS BELIEVED, both run by --check:

  extent   with the gate off, the extent computed here must reproduce the saved
           map's own origin, width and height. If it cannot reproduce the lattice
           of a map that exists, it cannot be trusted to predict the lattice of
           one that does not.
  grid     with the gate off, the rebuilt grid must agree with the saved map on
           >= 99.5 % of observed cells -- the same gate diagnose_karto passed
           20/20 -- which is what shows the trace loop here is faithful to the
           one it was copied from.

Writes experiments/logs/b2maps_gated/predicted/<map>.pgm + .yaml and one
_summary.json beside them. Touches no existing map, no threshold and no reader.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/predict_gated_maps.py --check \
        --map b2maps_k1_cut1200_robot1
    python3 experiments/analysis/predict_gated_maps.py
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

import diagnose_karto as dk                                  # noqa: E402
import graph_walls as gw                                     # noqa: E402
# Nothing here places a cell: karto_extent computes the Karto OFFSET (a bounding
# box minimum, so a position in its own right) and build() indexes through
# dk.world_to_grid. The provenance field is still written, because a reader of a
# predicted grid needs to know which lattice its `origin:` names.
from trinary_map import cell_centre_provenance                # noqa: E402

OUT_DIR = gw.LOGS_DIR / 'b2maps_gated' / 'predicted'
# The gate-OFF prediction, which C2 and its break both need: C2 asks about the
# cells where the two predictions differ, and the break compares the gated map
# against this one instead. It reproduces each saved map exactly (the --check
# figures), and it is written out anyway so the comparison rests on a file rather
# than on that equality.
OUT_DIR_UNGATED = gw.LOGS_DIR / 'b2maps_gated' / 'predicted_ungated'

GATE_M = 0.01            # diagnose_dwell's thresholds, against the last ADMITTED
GATE_DEG = 0.5
STAB_DROP = (2, 3, 4)    # slam_toolbox_common.cpp:795-797, on the gated sequence
AGREE_MIN = 0.995

# The pre-registered gate-off self-check's recorded verdict, in the words it is to
# be reported in. The bar was NOT reworded; this check did not meet it.
VERDICT = ('gate-off self-check FAILED at grid 100.00 % on 2/20 '
           '(k0_cut240: 1 cell of 30 829; k3_cut1200: 4 of 39 848), all '
           'saved-UNKNOWN -> rebuilt-committed; origin exact to 8.9e-15 m; '
           'width/height exact 20/20')

# Checked from the prediction files before the handover: none of those five cells
# lies in C2's set -- the cells where the gated and ungated predictions differ --
# so C2's figure needs no with/without split and its verdict is untouched by them.
# k0_cut240's cell is (row 70, col 73) against a 2-cell C2 set; k3_cut1200's are
# (row 70, cols 150-153) against a 98-cell one.
FIVE_VS_C2 = ("none of the five cells lies in C2's set, so C2 is reported once "
              'and its pre-registered verdict is unaffected by them')


def admit(sx, sy, syaw, ok) -> np.ndarray:
    """The gate. Admit when the pose has moved far enough since the LAST ADMITTED.

    A scan whose stamp has no interpolable pose cannot be judged, so it is
    ADMITTED and counted: the gate may only ever drop scans it has looked at.
    """
    n = sx.size
    out = np.zeros(n, dtype=bool)
    last = None
    turn = math.radians(GATE_DEG)
    for i in range(n):
        if not ok[i]:
            out[i] = True
            continue
        if last is None:
            out[i] = True
            last = i
            continue
        d = math.hypot(sx[i] - sx[last], sy[i] - sy[last])
        a = abs((syaw[i] - syaw[last] + math.pi) % (2.0 * math.pi) - math.pi)
        if d >= GATE_M or a >= turn:
            out[i] = True
            last = i
    return out


def reaching(admitted: np.ndarray) -> np.ndarray:
    """The admitted sequence minus its 2nd, 3rd and 4th scan."""
    reach = admitted.copy()
    idx = np.nonzero(admitted)[0]
    for ctr in STAB_DROP:
        if idx.size >= ctr:
            reach[idx[ctr - 1]] = False
    return reach


def karto_extent(sx, sy, ranges, beam, syaw, sel, geom, rho,
                 range_threshold) -> tuple:
    """Karto.h:6090-6114 over the scans in `sel`. Returns (offset, width, height).

    Each scan contributes its sensor position and its FILTERED readings only
    (minRange <= r <= rangeThreshold, Karto.h:5694-5700 with Math.h:172-175).
    """
    lo_x = lo_y = math.inf
    hi_x = hi_y = -math.inf
    for i in np.nonzero(sel)[0]:
        r = ranges[i]
        keep = (~np.isnan(r)) & (r >= geom['range_min']) & (r <= range_threshold)
        lo_x = min(lo_x, sx[i])
        hi_x = max(hi_x, sx[i])
        lo_y = min(lo_y, sy[i])
        hi_y = max(hi_y, sy[i])
        if keep.any():
            ang = syaw[i] + beam[keep]
            px = sx[i] + r[keep] * np.cos(ang)
            py = sy[i] + r[keep] * np.sin(ang)
            lo_x = min(lo_x, float(px.min()))
            hi_x = max(hi_x, float(px.max()))
            lo_y = min(lo_y, float(py.min()))
            hi_y = max(hi_y, float(py.max()))
    scale = 1.0 / rho
    w = int(round((hi_x - lo_x) * scale))
    h = int(round((hi_y - lo_y) * scale))
    return (lo_x, lo_y), w, h


def build(sx, sy, syaw, ranges, beam, sel, geom, rho, origin, w, h,
          range_threshold) -> tuple:
    """Hits and passes over (w, h) at `origin`, by diagnose_karto's rules."""
    n_cells = w * h
    passes = np.zeros(n_cells, dtype=np.int64)
    hits = np.zeros(n_cells, dtype=np.int64)
    idx = np.nonzero(sel)[0]
    for lo in range(0, idx.size, 128):
        s = idx[lo:lo + 128]
        r = ranges[s]
        ox = sx[s][:, None]
        oy = sy[s][:, None]
        ang = syaw[s][:, None] + beam[None, :]
        use = (~np.isnan(r)) & (r > geom['range_min']) & (r < geom['range_max'])
        over = use & (r >= range_threshold)
        rr = np.where(over, range_threshold, r)
        px = ox + rr * np.cos(ang)
        py = oy + rr * np.sin(ang)
        valid_end = use & (r < range_threshold - dk.KT_TOLERANCE)
        si, bi = np.nonzero(use)
        if si.size == 0:
            continue
        sox = np.broadcast_to(ox, r.shape)[si, bi]
        soy = np.broadcast_to(oy, r.shape)[si, bi]
        gx0, gy0 = dk.world_to_grid(sox, soy, origin, rho)
        gx1, gy1 = dk.world_to_grid(px[si, bi], py[si, bi], origin, rho)
        cx, cy, _ray = dk.trace_rays(gx0, gy0, gx1, gy1)
        ins = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        if ins.any():
            passes += np.bincount(cy[ins] * w + cx[ins], minlength=n_cells)
        ve = valid_end[si, bi]
        eok = ve & (gx1 >= 0) & (gx1 < w) & (gy1 >= 0) & (gy1 < h)
        if eok.any():
            ef = gy1[eok] * w + gx1[eok]
            passes += np.bincount(ef, minlength=n_cells)
            hits += np.bincount(ef, minlength=n_cells)
    return passes.reshape(h, w), hits.reshape(h, w)


def write_pgm(path: Path, grid: np.ndarray, rho: float, origin) -> None:
    """A trinary PGM + YAML, convention A, exactly as save_map.py writes one."""
    h, w = grid.shape
    with open(path.with_suffix('.pgm'), 'wb') as fh:
        fh.write(f'P5\n{w} {h}\n255\n'.encode())
        fh.write(grid[::-1, :].tobytes())
    path.with_suffix('.yaml').write_text(
        f'image: {path.name}.pgm\nmode: trinary\nresolution: {rho!r}\n'
        f'origin: [{origin[0]!r}, {origin[1]!r}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def one_map(stem: str, k: int, cut: int, check: bool,
            ungated: bool = False) -> dict:
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    m = gw.load_grid(stem)
    rho, saved_origin = m['rho'], m['origin']
    r_max, _src = gw.read_replay_r_max(k, cut)
    data = gw.read_bag(gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}_cut{cut}_slamin', k)
    geom = data['geom']
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    fwt.RUN = f'{gw.RUN_PREFIX}_k{k}_cut{cut}'
    mto, _p = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, stem)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    returns = gw.scan_returns(data, poses, base_scan, mto, r_max)
    sx, sy, syaw = (returns['sensor_x'], returns['sensor_y'],
                    returns['sensor_yaw'])
    range_threshold = min(max(r_max, geom['range_min']), geom['range_max'])
    ranges, _filled = dk.relay_fill(data['ranges'], geom['range_max'],
                                    range_threshold)
    beam = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']

    out = {'map': stem, 'robot': k, 'cut_s': cut, 'resolution': rho,
           'n_scans_read': int(data['stamps'].size),
           'saved': {'origin': list(saved_origin),
                     'width': m['width'], 'height': m['height'],
                     'occupied': int(np.count_nonzero(m['grid'] == gw.OCC)),
                     'free': int(np.count_nonzero(m['grid'] == gw.FREE)),
                     'unknown': int(np.count_nonzero(m['grid'] == gw.UNKNOWN))}}

    if check:
        ungated = dk.reaching_grid(poses[3])
        (ox, oy), w, h = karto_extent(sx, sy, ranges, beam, syaw, ungated,
                                      geom, rho, range_threshold)
        out['check'] = {
            'extent_origin': [ox, oy], 'extent_width': w, 'extent_height': h,
            'origin_dx': ox - saved_origin[0], 'origin_dy': oy - saved_origin[1],
            'width_matches': w == m['width'], 'height_matches': h == m['height'],
        }
        p, hh = build(sx, sy, syaw, ranges, beam, ungated, geom, rho,
                      saved_origin, m['width'], m['height'], range_threshold)
        state = dk.karto_to_trinary(dk.update_cell(p, hh))
        obs = (state != gw.UNKNOWN) | (m['grid'] != gw.UNKNOWN)
        n = int(np.count_nonzero(obs))
        agree = int(np.count_nonzero((state == m['grid']) & obs))
        # The disagreeing CELL COUNT, not only a share. The bar this check is
        # judged against is exact reproduction, and a %.2f share cannot be read
        # against it: a single wrong cell in 39,848 prints as 100.00 %.
        out['check']['n_observed'] = n
        out['check']['n_disagree'] = n - agree
        out['check']['grid_agreement'] = (agree / n) if n else 0.0
        out['check']['grid_exact'] = bool(n and agree == n)
        out['check']['grid_holds'] = out['check']['grid_agreement'] >= AGREE_MIN
        conf = {}
        for a, an in ((gw.UNKNOWN, 'unknown'), (gw.FREE, 'free'),
                      (gw.OCC, 'occupied')):
            for b, bn in ((gw.UNKNOWN, 'unknown'), (gw.FREE, 'free'),
                          (gw.OCC, 'occupied')):
                if a == b:
                    continue
                c = int(np.count_nonzero((m['grid'] == a) & (state == b) & obs))
                if c:
                    conf[f'saved_{an}__predicted_{bn}'] = c
        out['check']['off_diagonal'] = conf
        return out

    if ungated:
        # The gate off: every pose-valid scan, minus slam_toolbox's 2nd/3rd/4th.
        admitted = poses[3].copy()
        reach = dk.reaching_grid(poses[3])
    else:
        admitted = admit(sx, sy, syaw, poses[3])
        reach = reaching(admitted)
    (ox, oy), w, h = karto_extent(sx, sy, ranges, beam, syaw, reach, geom, rho,
                                  range_threshold)
    p, hh = build(sx, sy, syaw, ranges, beam, reach, geom, rho, (ox, oy), w, h,
                  range_threshold)
    pred = dk.karto_to_trinary(dk.update_cell(p, hh))

    # B1 on the predicted grid, from the ADMITTED scans' returns, computed the
    # way Part B computes it so the two numbers are comparable (both carry the
    # half-cell shift, so it cancels in the comparison).
    keep_ret = admitted[returns['scan']]
    sub = {key: returns[key][keep_ret] for key in ('px', 'py')}
    b1 = gw.on_occupied_share(sub, pred, rho, (ox, oy))

    n_adm = int(np.count_nonzero(admitted))
    out.update({
        'gate': {'m': GATE_M, 'deg': GATE_DEG,
                 'against': 'the last admitted scan'},
        'n_admitted': n_adm,
        'n_reaching_grid': int(np.count_nonzero(reach)),
        'share_dropped': 1.0 - n_adm / data['stamps'].size,
        'predicted': {'origin': [ox, oy], 'width': w, 'height': h,
                      'occupied': int(np.count_nonzero(pred == gw.OCC)),
                      'free': int(np.count_nonzero(pred == gw.FREE)),
                      'unknown': int(np.count_nonzero(pred == gw.UNKNOWN))},
        'b1_predicted': b1['share'],
        'n_returns_admitted': int(np.count_nonzero(keep_ret)),
    })
    for key in ('occupied', 'free', 'unknown'):
        out['predicted'][f'delta_{key}'] = (out['predicted'][key]
                                            - out['saved'][key])
    out_dir = OUT_DIR_UNGATED if ungated else OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    write_pgm(out_dir / stem, pred, rho, (ox, oy))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps')
    ap.add_argument('--ungated', action='store_true',
                    help='write the gate-OFF prediction to predicted_ungated/ '
                         'instead; C2 and its break read it')
    ap.add_argument('--check', action='store_true',
                    help='run the two self-checks with the gate OFF and write '
                         'nothing')
    args = ap.parse_args()
    stems = args.maps or gw.corpus_stems()

    print('predicting the travel-gated maps' if not args.check
          else 'self-check: the gate OFF must reproduce the saved maps')
    print(f'  gate: >= {GATE_M} m or >= {GATE_DEG} deg since the last ADMITTED '
          'scan; the first is always admitted')
    print('  extent: Karto.h:6090-6114 over the admitted scans; '
          'a gated map has its own lattice')
    print()
    rows = {}
    if args.check:
        print(f'{"map":<30}{"d origin x":>12}{"d origin y":>12}{"w":>6}{"h":>6}'
              f'{"w,h ok":>8}{"disagree":>10}{"exact":>7}')
    else:
        print(f'{"map":<30}{"scans":>7}{"admitted":>10}{"dropped":>9}'
              f'{"occ":>7}{"free":>8}{"unk":>7}{"B1":>8}')
    for stem in stems:
        k, cut = gw._k_and_cut(stem, 'the prediction needs the bag')
        r = one_map(stem, k, cut, args.check, args.ungated)
        rows[stem] = r
        if args.check:
            c = r['check']
            print(f'{stem:<30}{c["origin_dx"]:>12.6f}{c["origin_dy"]:>12.6f}'
                  f'{c["extent_width"]:>6}{c["extent_height"]:>6}'
                  f'{str(c["width_matches"] and c["height_matches"]):>8}'
                  f'{c["n_disagree"]:>10}{str(c["grid_exact"]):>7}')
        else:
            p = r['predicted']
            print(f'{stem:<30}{r["n_scans_read"]:>7}{r["n_admitted"]:>10}'
                  f'{r["share_dropped"]:>8.1%}'
                  f'{p["delta_occupied"]:>+7d}{p["delta_free"]:>+8d}'
                  f'{p["delta_unknown"]:>+7d}{r["b1_predicted"]:>7.1%}')
    print()
    if args.check:
        exact = sum(1 for r in rows.values()
                    if r['check']['grid_exact'] and r['check']['width_matches']
                    and r['check']['height_matches']
                    and r['check']['origin_dx'] == 0.0
                    and r['check']['origin_dy'] == 0.0)
        print(f'SELF-CHECK: {exact}/{len(rows)} maps reproduce the lattice and '
              'the grid EXACTLY with the gate off')
        for stem, r in rows.items():
            c = r['check']
            if c['n_disagree']:
                print(f"    {stem}: {c['n_disagree']} cell(s) -- "
                      f"{c['off_diagonal']}")
        (gw.LOGS_DIR / 'b2maps_gated').mkdir(parents=True, exist_ok=True)
        (gw.LOGS_DIR / 'b2maps_gated' / 'selfcheck.json').write_text(
            json.dumps({
                'bar': 'origin exact, width/height exact, grid 100.00 %, 20/20',
                'bar_reworded': False,
                'verdict': VERDICT,
                'five_cells_vs_C2': FIVE_VS_C2,
                'maps_exact': exact,
                'maps': len(rows),
                'provenance': {'cell_centre': cell_centre_provenance()},
                'per_map': {s: r['check'] for s, r in rows.items()}},
                indent=2) + '\n')
        return 0 if exact == len(rows) else 2
    out_dir = OUT_DIR_UNGATED if args.ungated else OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / '_summary.json').write_text(
        json.dumps({'gate': {'m': GATE_M, 'deg': GATE_DEG,
                             'against': 'the last admitted scan',
                             'first_scan': 'always admitted'},
                    'gate_applied': not args.ungated,
                    'provenance': {'cell_centre': cell_centre_provenance()},
                    'per_map': rows}, indent=2) + '\n')
    print(f'wrote {len(rows)} predicted grids and _summary.json to '
          f'{out_dir.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
