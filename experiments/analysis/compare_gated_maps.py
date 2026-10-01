#!/usr/bin/env python3
"""Compare each travel-gated map against the prediction written before the run.

Run this AFTER the gated batch. It reads maps only, through trinary_map, and
writes one file: experiments/logs/b2maps_gated/comparison.json.

WHAT IS BEING TESTED. predict_gated_maps.py wrote, before any gated map existed,
what Karto should produce from the admitted scans alone. Two criteria:

  C1  pre-registered earlier: the gated map agrees with the gated prediction on
      >= 99.5 % of observed cells, 20/20.
  C2  pre-registered before the run: on the cells where the GATED and UNGATED
      predictions differ, the gated map matches the GATED prediction on >= 95 %
      of them, on each of the five cut1200 maps. The short cuts differ by at most
      a handful of cells, so they are reported and not judged.

C2 exists because C1 is weak on its own: the gated and ungated maps agree almost
everywhere, so a comparison against the WRONG prediction still passes C1. That is
this file's own break, `--break ungated-prediction`: it compares each gated map
against the ungated prediction instead, and C2 must then fail on all five cut1200
maps. A break that leaves both criteria standing would mean neither is measuring
the gate.

GEOMETRY. Both predictions and both maps are Karto-indexed: a cell's centre is at
origin + i*res (Karto.h:4421-4436), not origin + (i+0.5)*res. The gated and
ungated lattices are NOT the same -- Karto takes the offset from the bounding box
of the scans it was given (Karto.h:6113), and the gated run was given fewer -- so
the ungated grid is resampled onto the gated lattice with trinary_map.sample.
That resampler works in the ROS convention, so BOTH origins are handed to it
shifted by -rho/2, which turns its arithmetic into the Karto-to-Karto nearest-cell
lookup this needs. resample_self_check() pins that by resampling a grid onto its
own lattice and requiring it back unchanged.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/compare_gated_maps.py
    python3 experiments/analysis/compare_gated_maps.py --break ungated-prediction
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import graph_walls as gw                                     # noqa: E402
from trinary_map import (FREE, OCC, UNKNOWN, check_thresholds,  # noqa: E402
                         classify, load, sample)

GATED_DIR = gw.LOGS_DIR / 'b2maps_gated'
PRED_DIR = GATED_DIR / 'predicted'
PRED_UNGATED_DIR = GATED_DIR / 'predicted_ungated'
OUT_PATH = GATED_DIR / 'comparison.json'

C1_MIN = 0.995
C2_MIN = 0.95
C2_CUT = 1200           # the cut C2 is judged on; the others are reported


def read_grid(stem: str, directory: Path) -> dict:
    """One trinary map as a bottom-up grid, read only through trinary_map."""
    pgm = directory / f'{stem}.pgm'
    if not pgm.is_file():
        return {}
    m = load(pgm)
    import yaml                                              # noqa: PLC0415
    meta = yaml.safe_load((directory / f'{stem}.yaml').read_text())
    negate = int(meta['negate'])
    occ_t, free_t = float(meta['occupied_thresh']), float(meta['free_thresh'])
    check_thresholds(negate, occ_t, free_t, m['other'], pgm.name)
    states = classify(m['px'], negate, occ_t, free_t)
    return {'grid': states[::-1, :], 'px': states, 'rho': float(m['resolution']),
            'origin': (float(m['origin'][0]), float(m['origin'][1])),
            'width': int(m['width']), 'height': int(m['height']),
            'resolution': float(m['resolution'])}


def resample_onto(src: dict, dst: dict) -> np.ndarray:
    """`src` read at `dst`'s Karto cell centres, via trinary_map.sample.

    sample() asks for centres at x0 + (i+0.5)*res and looks the source up with
    floor((x - origin)/res); both are the ROS convention. Handing it each origin
    shifted by -rho/2 converts both ends to Karto's, where a cell's centre is at
    origin + i*res -- so this is the nearest-cell lookup between two Karto grids,
    with no half-cell slip introduced by the resampling itself.
    """
    rho = dst['rho']
    shifted = dict(src, origin=[src['origin'][0] - rho / 2.0,
                                src['origin'][1] - rho / 2.0],
                   px=src['px'])
    return sample(shifted, rho,
                  dst['origin'][0] - rho / 2.0, dst['origin'][1] - rho / 2.0,
                  dst['width'], dst['height'])


def resample_self_check(grid: dict) -> bool:
    """A grid resampled onto its own lattice must come back unchanged."""
    return bool(np.array_equal(resample_onto(grid, grid), grid['grid']))


def agreement(a: np.ndarray, b: np.ndarray) -> dict:
    """Observed-cell agreement: cells either side calls something."""
    obs = (a != UNKNOWN) | (b != UNKNOWN)
    n = int(np.count_nonzero(obs))
    return {'n_observed': n,
            'n_agree': int(np.count_nonzero((a == b) & obs)),
            'agreement': (float(np.count_nonzero((a == b) & obs)) / n)
            if n else float('nan')}


def compare(stem_ungated: str, k: int, cut: int, break_name: str | None) -> dict:
    gated_stem = f'{gw.RUN_PREFIX}_k{k}_cut{cut}_gated_robot{k}'
    out = {'cut_s': cut, 'robot': k, 'gated_map': gated_stem,
           'prediction': stem_ungated}
    m = read_grid(gated_stem, gw.MAPS_DIR_DEFAULT)
    if not m:
        out['status'] = (f'the gated map {gated_stem} is not in '
                         f'{gw.MAPS_DIR_DEFAULT} yet -- run the gated batch '
                         'first')
        return out
    pred = read_grid(stem_ungated, PRED_DIR)
    pred_un = read_grid(stem_ungated, PRED_UNGATED_DIR)
    if not pred or not pred_un:
        out['status'] = ('a prediction is missing -- run predict_gated_maps.py '
                         'and predict_gated_maps.py --ungated')
        return out
    if not resample_self_check(pred_un):
        gw.die(f'{stem_ungated}: resampling a grid onto its own lattice changed '
               'it, so the resampler cannot be trusted to move the ungated '
               'prediction onto the gated lattice')

    # Which prediction is being compared against. The break swaps it.
    against = pred_un if break_name == 'ungated-prediction' else pred
    out['compared_against'] = ('the UNGATED prediction (BROKEN ON PURPOSE)'
                               if break_name else 'the gated prediction')
    out['lattice'] = {
        'map': {'origin': list(m['origin']), 'width': m['width'],
                'height': m['height']},
        'gated_prediction': {'origin': list(pred['origin']),
                             'width': pred['width'], 'height': pred['height']},
        'ungated_prediction': {'origin': list(pred_un['origin']),
                               'width': pred_un['width'],
                               'height': pred_un['height']},
        'map_matches_gated_prediction': bool(
            m['width'] == pred['width'] and m['height'] == pred['height']
            and abs(m['origin'][0] - pred['origin'][0]) < 1e-9
            and abs(m['origin'][1] - pred['origin'][1]) < 1e-9),
    }

    # Everything is compared on the MAP's own lattice.
    a = resample_onto(against, m)
    g = resample_onto(pred, m)
    u = resample_onto(pred_un, m)
    out['C1'] = agreement(m['grid'], a)
    out['C1']['limit'] = C1_MIN
    out['C1']['holds'] = bool(out['C1']['agreement'] >= C1_MIN)

    differ = (g != u)
    n_diff = int(np.count_nonzero(differ))
    matched = int(np.count_nonzero(differ & (m['grid'] == a)))
    out['C2'] = {
        'n_cells_where_predictions_differ': n_diff,
        'n_map_matches_the_compared_prediction': matched,
        'share': (float(matched) / n_diff) if n_diff else None,
        'limit': C2_MIN,
        'judged': cut == C2_CUT,
        'holds': (bool(n_diff and matched / n_diff >= C2_MIN)
                  if cut == C2_CUT else None),
    }
    for name, grid in (('occupied', OCC), ('free', FREE), ('unknown', UNKNOWN)):
        out[f'map_{name}'] = int(np.count_nonzero(m['grid'] == grid))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--break', dest='break_name',
                    choices=['ungated-prediction'],
                    help='compare each gated map against the UNGATED prediction '
                         'instead. C1 will probably still pass; C2 must fail on '
                         'all five cut1200 maps. Writes nothing, exits 2')
    args = ap.parse_args()

    print('comparing each travel-gated map against the prediction written '
          'before the run')
    print(f'  C1  observed-cell agreement >= {C1_MIN:.1%}, 20/20 '
          '(pre-registered earlier)')
    print('  C2  on the cells where the gated and ungated predictions differ, '
          'the gated map matches')
    print(f'      the gated prediction on >= {C2_MIN:.0%}, on each cut{C2_CUT} '
          'map (pre-registered before the run)')
    if args.break_name:
        print(f'  (BROKEN ON PURPOSE: --break {args.break_name}. C2 must fail on '
              'all five cut1200 maps.)')
    print()

    rows, missing = {}, []
    print(f'{"map":<34}{"C1":>9}{"differ":>9}{"C2":>9}{"lattice":>9}')
    for stem in gw.corpus_stems():
        k, cut = gw._k_and_cut(stem, 'the comparison needs the robot and the cut')
        r = compare(stem, k, cut, args.break_name)
        rows[stem] = r
        if 'status' in r:
            missing.append(stem)
            print(f'{stem:<34}  {r["status"][:60]}')
            continue
        c2 = r['C2']['share']
        print(f'{r["gated_map"]:<34}{r["C1"]["agreement"]:>8.2%}'
              f'{r["C2"]["n_cells_where_predictions_differ"]:>9}'
              f'{(f"{c2:.1%}" if c2 is not None else "-"):>9}'
              f'{str(r["lattice"]["map_matches_gated_prediction"]):>9}')
    print()

    done = [s for s in rows if 'status' not in rows[s]]
    if not done:
        print('No gated map is in place yet, so nothing was compared. Run the '
              'gated batch first.')
        return 0

    c1_bad = [s for s in done if not rows[s]['C1']['holds']]
    c2_judged = [s for s in done if rows[s]['C2']['judged']]
    c2_bad = [s for s in c2_judged if not rows[s]['C2']['holds']]
    print(f'C1: {len(done) - len(c1_bad)}/{len(done)} maps at or above '
          f'{C1_MIN:.1%}' + ('' if not c1_bad else f' -- FAILED on {c1_bad}'))
    print(f'C2: {len(c2_judged) - len(c2_bad)}/{len(c2_judged)} cut{C2_CUT} maps '
          f'at or above {C2_MIN:.0%}'
          + ('' if not c2_bad else f' -- FAILED on {c2_bad}'))
    for s in done:
        if not rows[s]['C2']['judged']:
            n = rows[s]['C2']['n_cells_where_predictions_differ']
            print(f'    reported only: {s} differs on {n} cell(s)')
    print()

    if args.break_name:
        all_c2_failed = bool(c2_judged) and len(c2_bad) == len(c2_judged)
        print(f'--break {args.break_name}: C1 '
              f'{"still passed" if not c1_bad else "failed"} on '
              f'{len(done) - len(c1_bad)}/{len(done)}, which is why C2 exists; '
              f'C2 failed on {len(c2_bad)}/{len(c2_judged)} cut{C2_CUT} maps')
        if not all_c2_failed:
            print('  C2 did NOT fail everywhere under the break. Then it is not '
                  'measuring the gate either, and that is a finding.')
        print('  Nothing was written.')
        return 2

    out = {
        'question': 'does each travel-gated map match the prediction written '
                    'before the run',
        'criteria': {
            'C1': f'observed-cell agreement >= {C1_MIN}, 20/20, pre-registered '
                  'earlier',
            'C2': f'on the cells where the gated and ungated predictions differ, '
                  f'the gated map matches the gated prediction on >= {C2_MIN}, '
                  f'on each cut{C2_CUT} map; pre-registered before the run',
        },
        'break': 'compare against the ungated prediction instead '
                 '(--break ungated-prediction): C1 probably still passes, C2 '
                 'must fail on all five cut1200 maps',
        'read_through': 'trinary_map only',
        'maps_compared': len(done),
        'maps_missing': missing,
        'C1_failures': c1_bad,
        'C2_failures': c2_bad,
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
