#!/usr/bin/env python3
"""Does the +1 extent patch do what it is supposed to, and nothing else?

Judges E2 and E3 of the pre-registration Ali set before any of these grids
existed. E1 and the B1 pair are judged by predict_gated_maps.py --extent-fix
itself, where the returns are already in hand; this file reads what that wrote.

  E1  off-grid kept returns = 0 on 20/20                  (predict_gated_maps)
  E2  every cell with an index inside the OLD width and height is identical to
      the gated prediction -- NOT to the slam_toolbox map. The fix may only ADD
      the far row and column, 20/20                       (here)
  E3  on the five cut1200 maps, max-side A9 recall is within 2 pp of min-side
      recall, 5/5                                         (here)

E2 IS THE ONE THAT COULD CATCH A MISTAKE, so it is worth saying what it compares.
The reference is the GATED PREDICTION, not the saved slam_toolbox map: the
prediction is what the same code built from the same scans with the same gate and
differs only in the extent, so any other difference is this change's fault. A
comparison against the saved map would fold in every reason a prediction and a
replay differ -- the five cells the gate-off self-check already recorded among them
-- and would not isolate the patch. E2 also requires the origin to be bit-identical
and the dimensions to be exactly +1 on each axis, because "only the far row and
column were added" is false if the offset moved.

E3 is the point of the exercise. §12.4 measured max-side A9 recall at 62-98 %
against 99 % on the min side, on the maps that saw all four boundary walls, and
traced it to the far-edge drop. If that is the whole cause, a patched extent
should bring the two sides together. 2 pp is the pre-registered tolerance.

TRUTH. E3 uses the SDF faces, through diagnose_extent's own recall_by_face so there
is one covering test rather than two. Diagnosis only, as A9 is: §6 keeps it out of
every <map>.json, and these grids are predictions, not maps.

NOTHING IS PATCHED AND NOTHING IS BUILT by this file. It reads
experiments/logs/b2maps_extent/predicted/ (written by predict_gated_maps.py
--extent-fix), experiments/logs/b2maps_gated/predicted/ (the reference) and the
world SDF, and writes experiments/logs/graph_walls/diagnose_extent_fix.json.

    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/NSKsim/install/setup.bash
    python3 experiments/analysis/predict_gated_maps.py --extent-fix
    python3 experiments/analysis/diagnose_extent_fix.py

Seconds, not minutes: it opens no bag.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import diagnose_extent as de                                 # noqa: E402
import graph_walls as gw                                     # noqa: E402
import predict_gated_maps as pg                              # noqa: E402
from trinary_map import cell_centre_provenance                # noqa: E402

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_extent_fix.json'

E3_TOL_PP = 2.0          # pre-registered
E3_CUT = 1200            # the cut E3 is judged on; §12.4 showed it is the only
                         # one where all four boundary walls were observed

PREDICTIONS = [
    ('E1', 'off-grid kept returns = 0 on 20/20 '
           '(judged by predict_gated_maps.py --extent-fix)'),
    ('E2', 'every cell with an index inside the OLD width and height is '
           'identical to the gated prediction, not to the slam_toolbox map; the '
           'fix may only add the far row and column, 20/20'),
    ('E3', f'on the five cut{E3_CUT} maps, max-side A9 recall is within '
           f'{E3_TOL_PP:g} pp of min-side recall, 5/5'),
]


def read_pair(stem: str) -> tuple[dict, dict]:
    """(the extent-fixed prediction, the gated prediction) for one map."""
    for d in (pg.OUT_DIR_EXTENT, pg.OUT_DIR):
        if not (d / f'{stem}.pgm').is_file():
            gw.die(f'{stem}.pgm is not in {d}. Run '
                   'predict_gated_maps.py --extent-fix (and the plain run for '
                   'the reference) first.')
    return (gw.load_grid(stem, pg.OUT_DIR_EXTENT),
            gw.load_grid(stem, pg.OUT_DIR))


def e2_for_map(fixed: dict, ref: dict) -> dict:
    """E2: the overlap must be identical and the growth exactly the far row/col.

    Both grids are bottom-up with cell (0, 0) at the same origin, so the OLD
    index range is simply the reference's shape and the comparison is a slice. No
    resampling: a resample would hide a half-cell slip, which is the one kind of
    mistake this check exists to catch.
    """
    fh, fw = fixed['grid'].shape
    rh, rw = ref['grid'].shape
    same_origin = (abs(fixed['origin'][0] - ref['origin'][0]) < 1e-12
                   and abs(fixed['origin'][1] - ref['origin'][1]) < 1e-12)
    grew_by_one = (fw == rw + pg.EXTENT_FIX_CELLS
                   and fh == rh + pg.EXTENT_FIX_CELLS)
    out = {
        'origin_fixed': list(fixed['origin']), 'origin_reference': list(ref['origin']),
        'origin_identical': bool(same_origin),
        'size_reference': [rw, rh], 'size_fixed': [fw, fh],
        'grew_by_exactly_one_per_axis': bool(grew_by_one),
    }
    if not (same_origin and grew_by_one):
        out['overlap_identical'] = False
        out['n_overlap_cells'] = 0
        out['n_overlap_differing'] = None
        out['holds'] = False
        return out
    over_f = fixed['grid'][:rh, :rw]
    diff = over_f != ref['grid']
    n_diff = int(np.count_nonzero(diff))
    out['n_overlap_cells'] = int(ref['grid'].size)
    out['n_overlap_differing'] = n_diff
    out['overlap_identical'] = n_diff == 0
    # What the far row and column actually came to, reported not gated: a fix that
    # added them and left them Unknown would pass E2 and have done nothing.
    far_col = fixed['grid'][:, rw]
    far_row = fixed['grid'][rh, :]
    added = np.concatenate([far_col, far_row])
    out['added_cells'] = {
        'n': int(added.size),
        'occupied': int(np.count_nonzero(added == gw.OCC)),
        'free': int(np.count_nonzero(added == gw.FREE)),
        'unknown': int(np.count_nonzero(added == gw.UNKNOWN)),
    }
    out['holds'] = bool(n_diff == 0)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--spawn-rev', default=gw.SPAWN_REV)
    ap.add_argument('--no-json', action='store_true',
                    help='print everything and write nothing')
    args = ap.parse_args()
    t0 = time.monotonic()

    print('=' * 100)
    print('the +1 extent patch: does it do what it is supposed to, and nothing '
          'else?')
    print(f'  the patch is {pg.EXTENT_FIX_PATCH}, NOT APPLIED to '
          '~/src/slam_toolbox_2.8.5. Nothing is built here.')
    print()
    print('PRE-REGISTERED, stated before any number below is read:')
    for name, text in PREDICTIONS:
        print(f'  {name}  {text}')
    print()
    print('  Also reported without a threshold: in-grid B1 and all-returns B1, '
          'which should now be')
    print('  equal, because E1 says the off-grid population is empty. Both come '
          'from the')
    print(f'  _summary.json in {pg.OUT_DIR_EXTENT.relative_to(REPO_ROOT)}.')
    print('  E2 compares against the GATED PREDICTION, not the slam_toolbox '
          'map: the prediction')
    print('  differs from the fixed one only in the extent, so any other '
          'difference belongs to')
    print('  this change. Truth (the SDF faces) enters for E3 only, and stays '
          'out of every')
    print('  <map>.json as §6 requires.')
    print('=' * 100)
    print()

    summary_path = pg.OUT_DIR_EXTENT / '_summary.json'
    if not summary_path.is_file():
        gw.die(f'{summary_path} is missing. Run predict_gated_maps.py '
               '--extent-fix first; E1 and the B1 pair are its numbers, not '
               'this file\'s.')
    summary = json.loads(summary_path.read_text())
    if 'extent_fix' not in summary:
        gw.die(f'{summary_path} holds no extent_fix block, so it was written '
               'WITHOUT --extent-fix and these are not the patched predictions.')
    e1 = summary['extent_fix']['E1']

    from fit_world_transform import resolve_spawn_poses        # numpy/yaml only
    import robot_divergence as rd                             # noqa: PLC0415
    spawn = resolve_spawn_poses(args.spawn_rev)
    world = gw.sdf_wall_faces(gw.L_MIN)
    samples = gw.face_samples(world, 0.01)
    radius, where = gw.read_robot_radius()
    params = gw.Params(g=2.0 * radius)
    rd.MAPS_DIR = gw.MAPS_DIR_DEFAULT

    rows: dict[str, dict] = {}
    print('E2 -- the overlap against the gated prediction, and what the added '
          'row and column')
    print('     came to (reported, not gated: a fix that added them empty would '
          'pass E2 and')
    print('     have done nothing).')
    print()
    print(f'{"map":<30}{"ref w,h":>10}{"fixed w,h":>12}{"origin":>8}'
          f'{"+1":>5}{"differ":>8}{"added":>7}{"occ":>6}{"free":>6}{"unk":>6}')
    for stem in gw.corpus_stems():
        fixed, ref = read_pair(stem)
        e2 = e2_for_map(fixed, ref)
        k, cut = gw._k_and_cut(stem, 'E3 needs the robot and the cut')
        rows[stem] = {'map': stem, 'robot': k, 'cut_s': cut, 'E2': e2,
                      'b1_in_grid': summary['per_map'][stem]['b1_predicted'],
                      'b1_all_returns':
                          summary['per_map'][stem]['b1_predicted_all_returns'],
                      'b1_offgrid': summary['per_map'][stem]['b1_offgrid']}
        a = e2['added_cells'] if 'added_cells' in e2 else {}
        print(f'{stem:<30}{str(e2["size_reference"]):>10}'
              f'{str(e2["size_fixed"]):>12}'
              f'{str(e2["origin_identical"]):>8}'
              f'{str(e2["grew_by_exactly_one_per_axis"]):>5}'
              f'{str(e2["n_overlap_differing"]):>8}'
              f'{a.get("n", "-"):>7}{a.get("occupied", "-"):>6}'
              f'{a.get("free", "-"):>6}{a.get("unknown", "-"):>6}')
    print()

    print(f'E3 -- A9 recall per boundary side on the cut{E3_CUT} maps, from the '
          'PATCHED predictions.')
    print('     max side = wall_east.west + wall_north.south; min side = '
          'wall_west.east + wall_south.north.')
    print()
    print(f'{"map":<30}{"E.west":>9}{"N.south":>9}{"W.east":>9}{"S.north":>9}'
          f'{"max mean":>10}{"min mean":>10}{"gap pp":>8}{"<=2pp":>7}')
    for stem, row in rows.items():
        if row['cut_s'] != E3_CUT:
            continue
        graph = gw.graph_for_map(stem, maps_dir=pg.OUT_DIR_EXTENT, params=params)
        rec = de.recall_by_face(graph, spawn[row['robot']], world, samples,
                                graph['resolution'])
        mx = [rec[f'{m}.{s}']['recall'] for m, s in de.MAX_SIDE]
        mn = [rec[f'{m}.{s}']['recall'] for m, s in de.MIN_SIDE]
        gap_pp = 100.0 * abs(float(np.mean(mx)) - float(np.mean(mn)))
        row['E3'] = {'recall_by_face': rec,
                     'max_side_mean_recall': float(np.mean(mx)),
                     'min_side_mean_recall': float(np.mean(mn)),
                     'gap_pp': gap_pp, 'tol_pp': E3_TOL_PP,
                     'holds': bool(gap_pp <= E3_TOL_PP),
                     'n_segments': len(graph['segments'])}
        print(f'{stem:<30}'
              + ''.join(f'{100 * rec[f"{m}.{s}"]["recall"]:>8.2f}%'
                        for m, s in de.MAX_SIDE + de.MIN_SIDE)
              + f'{100 * np.mean(mx):>9.2f}%{100 * np.mean(mn):>9.2f}%'
              + f'{gap_pp:>8.2f}{str(gap_pp <= E3_TOL_PP):>7}')
    print()

    print('the B1 pair, reported without a threshold')
    print(f'{"map":<30}{"in-grid":>10}{"all-returns":>13}{"off-grid":>10}'
          f'{"equal":>7}')
    for stem, r in rows.items():
        r['b1_equal'] = r['b1_in_grid'] == r['b1_all_returns']
        print(f'{stem:<30}{100 * r["b1_in_grid"]:>9.3f}%'
              f'{100 * r["b1_all_returns"]:>12.3f}%'
              f'{100 * r["b1_offgrid"]:>9.3f}%{str(r["b1_equal"]):>7}')
    print()

    e2_bad = [s for s, r in rows.items() if not r['E2']['holds']]
    e3 = {s: r['E3'] for s, r in rows.items() if 'E3' in r}
    e3_bad = [s for s, v in e3.items() if not v['holds']]
    n_eq = sum(1 for r in rows.values() if r['b1_equal'])

    print('=' * 100)
    print('PRE-REGISTERED VERDICTS')
    print(f'  E1  off-grid kept returns = 0: '
          f'{"HELD" if e1["holds"] else "FAILED"} '
          f'({e1["maps_with_no_offgrid_returns"]}/{e1["maps"]})   '
          'from predict_gated_maps')
    for s, v in e1['failing'].items():
        print(f'        {s}: {v:.4%} off-grid')
    print(f'  E2  overlap identical to the gated prediction: '
          f'{"HELD" if not e2_bad else "FAILED"} '
          f'({len(rows) - len(e2_bad)}/{len(rows)})')
    for s in e2_bad:
        v = rows[s]['E2']
        print(f'        {s}: origin identical {v["origin_identical"]}, '
              f'+1 per axis {v["grew_by_exactly_one_per_axis"]}, '
              f'{v["n_overlap_differing"]} overlap cells differ')
    print(f'  E3  max-side within {E3_TOL_PP:g} pp of min-side on the cut{E3_CUT} '
          f'maps: {"HELD" if e3 and not e3_bad else "FAILED"} '
          f'({len(e3) - len(e3_bad)}/{len(e3)})   worst gap '
          + (f'{max(v["gap_pp"] for v in e3.values()):.2f} pp'
             if e3 else 'n/a'))
    for s in e3_bad:
        print(f'        {s}: max {e3[s]["max_side_mean_recall"]:.2%} vs min '
              f'{e3[s]["min_side_mean_recall"]:.2%}, gap '
              f'{e3[s]["gap_pp"]:.2f} pp')
    print(f'  reported: in-grid B1 == all-returns B1 on {n_eq}/{len(rows)}, '
          'which is what E1 implies')
    print('=' * 100)
    print()

    elapsed = time.monotonic() - t0
    print(f'{len(rows)} maps in {elapsed:.1f} s')
    if args.no_json:
        print('--no-json: nothing written')
        return 0

    out = {
        'question': 'does the +1 extent patch close the far-edge drop without '
                    'changing anything else',
        'patch': {'file': pg.EXTENT_FIX_PATCH,
                  'cells_added_per_axis': pg.EXTENT_FIX_CELLS,
                  'applied_to_the_source_tree': False,
                  'built': False,
                  'what_this_measures': 'a PREDICTION of the patched Karto, from '
                                        'predict_gated_maps.py --extent-fix; no '
                                        'slam_toolbox was compiled or run'},
        'predictions': {name: text for name, text in PREDICTIONS},
        'predictions_registered': 'by Ali, in the request that produced this '
                                  'file, before any number in it was read',
        'inputs': {
            'patched_predictions': str(pg.OUT_DIR_EXTENT.relative_to(REPO_ROOT)),
            'reference_predictions': str(pg.OUT_DIR.relative_to(REPO_ROOT)),
            'reference_choice': 'the GATED PREDICTION, not the saved '
                                'slam_toolbox map: it is the same code on the '
                                'same scans with the same gate, differing only '
                                'in the extent, so any other difference belongs '
                                'to this change',
        },
        'truth_used': True,
        'truth_used_for': 'E3 only -- the SDF faces, through '
                          'diagnose_extent.recall_by_face so there is one '
                          'covering test. Diagnosis only',
        'gated': False,
        'touches': 'writes this file only; patches nothing, builds nothing, and '
                   'opens no bag',
        'provenance': {
            'cell_centre': cell_centre_provenance(),
            'spawn_rev': args.spawn_rev,
            'robot_radius': f'{where} robot_radius {radius}',
        },
        'verdicts': {
            'E1': e1,
            'E2': {'limit': 'zero differing cells in the overlap',
                   'maps': len(rows), 'failing': e2_bad,
                   'holds': not e2_bad},
            'E3': {'tol_pp': E3_TOL_PP, 'cut_s': E3_CUT, 'maps': len(e3),
                   'failing': e3_bad, 'holds': bool(e3) and not e3_bad,
                   'worst_gap_pp': (max(v['gap_pp'] for v in e3.values())
                                    if e3 else None)},
            'b1_pair': {'n_equal': n_eq, 'maps': len(rows),
                        'note': 'reported without a threshold; equality is what '
                                'E1 implies, not a separate claim'},
        },
        'elapsed_s': elapsed,
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
