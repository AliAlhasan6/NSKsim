#!/usr/bin/env python3
"""Compare maps built by the slam_toolbox OVERLAY against the ones already here.

Reads maps only, through trinary_map, and writes one file:
experiments/logs/b2maps_overlay/comparison.json. No bag, no truth, no threshold
moved.

WHAT IS BEING TESTED, in the order the stages run.

  R1  pre-registered: a map built by the UNPATCHED overlay is BYTE-IDENTICAL to
      the map the installed deb built. This is the gate on everything after it --
      until the overlay is shown equivalent to the binary that built the corpus,
      a difference in a patched map cannot be attributed to the patch rather than
      to the overlay's compiler flags, its CMake cache or its dependency versions.
      If R1 fails, stop.
  R2  pre-registered: in a map built by the PATCHED overlay, every cell with an
      index inside the unpatched width and height is identical to the unpatched
      map; the grid is exactly +1 in each axis; the origin is unchanged. The patch
      may only ADD the far row and column.
  R3  pre-registered: that added row and column agree with the extent PREDICTION
      (experiments/logs/b2maps_extent/predicted/) on >= 99.5 % of observed cells.
      R2 says the patch changed nothing it should not have; R3 says the thing it
      did add is what was predicted.

E3 -- max-side A9 recall within 2 pp of min-side -- is NOT here. It needs world
truth, and §6 keeps truth in the validation path: diagnose_extent.py computes it
for any stem it is given, including an _extfix one, once that replay's config and
map->odom log exist.

STEM NAMING, which is the whole reason this file can find its references:

    b2maps_k{K}_cut{C}[_gated][_ovl|_extfix]_robot{K}

`_ovl` is the overlay with the patch NOT applied, `_extfix` the overlay with
docs/patches/karto_extent.patch applied. graph_walls owns the parser
(stem_variants, run_variant_of, bag_variant_of); this file only strips tokens to
find the reference a check needs. The binary variants are a property of the BINARY,
so an _ovl or _extfix run replays the same bag as the run it is named after -- which
is why R1 is a byte comparison and not a re-derivation.

THREE BREAKS, one per check, each of which MUST fail its check:

  --break other-robot        every reference comes from a different robot. R1 and
                             R2 must fail: a check that passes against another
                             robot's map is not comparing anything.
                             NOT registered against R3, and the reason is a
                             finding about R3 rather than about the break: R3
                             looks only at the added row and column, and two
                             robots' grids differ in SIZE, so R3 refuses the pair
                             on the size check and is SKIPPED rather than failed.
                             A skip is not a failure, so this break cannot
                             demonstrate anything about R3. interior-row is R3's
                             break.
  --break shift-one-cell     the reference grid is rolled one cell in x before
                             comparing. R1 and R2 must fail. This is the one that
                             would catch a half-cell or whole-cell slip, which is
                             the mistake this project has already made once
                             (trinary_map.CELL_CENTRE_OFFSET).
  --break interior-row       R3 reads the prediction one row and column INSIDE the
                             added ones. R3 must fail: agreeing with the interior
                             would mean the comparison is not looking at what the
                             patch added.

--map is REQUIRED and repeatable: there is no default map set, because every
stem names a replay someone has to have run.

    venv/bin/python experiments/analysis/compare_overlay_maps.py --check R1 \
        --map b2maps_k0_cut60_gated_ovl_robot0 \
        --map b2maps_k2_cut60_gated_ovl_robot2
    venv/bin/python experiments/analysis/compare_overlay_maps.py --check R1 \
        --map b2maps_k0_cut60_gated_ovl_robot0 --break other-robot

No ROS is needed -- nothing here imports rclpy -- so the venv's python is enough.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import graph_walls as gw                                     # noqa: E402
import predict_gated_maps as pg                              # noqa: E402
from compare_gated_maps import read_grid                     # noqa: E402
from trinary_map import UNKNOWN, cell_centre_provenance       # noqa: E402

OUT_DIR = gw.LOGS_DIR / 'b2maps_overlay'
OUT_PATH = OUT_DIR / 'comparison.json'

R3_MIN = 0.995           # pre-registered
BREAKS = ('other-robot', 'shift-one-cell', 'interior-row')


# ─────────────────────────────── stems ───────────────────────────────────────

def strip_binary_variant(stem: str) -> str:
    """The same stem with `_ovl` or `_extfix` removed. R1's and R2's reference.

    Only the BINARY token is removed: `_gated` stays, because a gated overlay run
    is to be compared with the gated map and not with the ungated one.
    """
    for binv in gw.BINARY_VARIANTS:
        stem = stem.replace(f'_{binv}_robot', '_robot')
    return stem


def plain_stem(stem: str) -> str:
    """The stem with every variant token removed -- the prediction's file name.

    predict_gated_maps writes a gated prediction under the UNGATED corpus stem
    (that is what compare_gated_maps reads), so this is the name R3 looks for in
    b2maps_extent/predicted/.
    """
    k, cut = gw._k_and_cut(stem, 'the reference prediction is named from them')
    return f'{gw.RUN_PREFIX}_k{k}_cut{cut}_robot{k}'


def unpatched_reference(stem: str, maps_dir: Path) -> tuple[str, str]:
    """The unpatched map an _extfix map is to be compared against, and which it is.

    PREFERENCE, and it is not arbitrary. The closest control is the overlay's OWN
    unpatched build (`_ovl`): it differs from the patched one by the patch and by
    nothing else -- same compiler, same flags, same CMake cache. The deb-built map
    (binary token stripped) is the fallback, and it is only as good as R1, which is
    why R1 gates the stages. Which one was used is recorded per map, because
    "identical to the unpatched map" means a different thing for each.
    """
    stripped = strip_binary_variant(stem)
    ovl = stripped.replace('_robot', '_ovl_robot')
    if (maps_dir / f'{ovl}.pgm').is_file():
        return ovl, 'the overlay\'s own unpatched build'
    return stripped, 'the installed binary\'s map (only as good as R1)'


def other_robot_stem(stem: str) -> str:
    """The same cut and variants on a DIFFERENT robot. The other-robot break."""
    k, cut = gw._k_and_cut(stem, 'the break needs the robot and the cut')
    other = (k + 1) % gw.NUM_ROBOTS
    return (stem.replace(f'_k{k}_', f'_k{other}_')
                .replace(f'_robot{k}', f'_robot{other}'))


# ─────────────────────────────── the checks ──────────────────────────────────

def rel(path: Path) -> str:
    """`path` relative to the repo when it is inside it, else as it is.

    --maps-dir may legitimately point outside the repo (a scratch copy, a tmp dir
    in a test), and a refusal that cannot format its own path is a refusal nobody
    can act on.
    """
    try:
        return str(Path(path).relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def r1(stem: str, maps_dir: Path, break_name: str | None) -> dict:
    """R1: the overlay's map is byte-identical to the installed binary's.

    The PGM BYTES, not a resampled comparison: the claim is that two binaries
    produced the same file, and anything weaker than a byte comparison would let a
    one-cell difference through as a rounding detail.
    """
    ref_stem = (other_robot_stem(strip_binary_variant(stem))
                if break_name == 'other-robot' else strip_binary_variant(stem))
    out = {'map': stem, 'reference': ref_stem}
    a, b = maps_dir / f'{stem}.pgm', maps_dir / f'{ref_stem}.pgm'
    for p in (a, b):
        if not p.is_file():
            out['status'] = f'not found: {rel(p)}'
            return out
    ga, gb = read_grid(stem, maps_dir), read_grid(ref_stem, maps_dir)
    if break_name == 'shift-one-cell':
        gb = dict(gb, px=np.roll(gb['px'], 1, axis=1))
    out.update({
        'sha256': sha256_of(a), 'sha256_reference': sha256_of(b),
        'size_cells': [ga['width'], ga['height']],
        'size_cells_reference': [gb['width'], gb['height']],
        'origin': list(ga['origin']), 'origin_reference': list(gb['origin']),
    })
    # Under a break the bytes on disk are untouched, so the byte test is made to
    # read the (broken) grid instead -- otherwise shift-one-cell could not fail.
    if break_name == 'shift-one-cell':
        out['bytes_identical'] = bool(np.array_equal(ga['px'], gb['px']))
    else:
        out['bytes_identical'] = out['sha256'] == out['sha256_reference']
    out['geometry_identical'] = bool(
        ga['width'] == gb['width'] and ga['height'] == gb['height']
        and abs(ga['origin'][0] - gb['origin'][0]) < 1e-12
        and abs(ga['origin'][1] - gb['origin'][1]) < 1e-12)
    out['holds'] = bool(out['bytes_identical'] and out['geometry_identical'])
    return out


def r2(stem: str, maps_dir: Path, break_name: str | None) -> dict:
    """R2: the patched map adds the far row and column and changes nothing else.

    The reference is the UNPATCHED map of the same cut -- `_extfix` removed, which
    for a stage-D map is the `_ovl` one and for a stage-E map may be the deb's.
    Compared by slice and never by resample: a resample would absorb exactly the
    slip this check exists to catch.
    """
    ref_stem, ref_kind = unpatched_reference(stem, maps_dir)
    if break_name == 'other-robot':
        ref_stem = other_robot_stem(ref_stem)
        ref_kind = 'ANOTHER ROBOT (BROKEN ON PURPOSE)'
    out = {'map': stem, 'reference': ref_stem, 'reference_kind': ref_kind}
    for s in (stem, ref_stem):
        if not (maps_dir / f'{s}.pgm').is_file():
            out['status'] = f'not found: {rel(maps_dir / f"{s}.pgm")}'
            return out
    fixed, ref = read_grid(stem, maps_dir), read_grid(ref_stem, maps_dir)
    if break_name == 'shift-one-cell':
        ref = dict(ref, grid=np.roll(ref['grid'], 1, axis=1))
    fh, fw = fixed['grid'].shape
    rh, rw = ref['grid'].shape
    same_origin = (abs(fixed['origin'][0] - ref['origin'][0]) < 1e-12
                   and abs(fixed['origin'][1] - ref['origin'][1]) < 1e-12)
    grew = (fw == rw + pg.EXTENT_FIX_CELLS and fh == rh + pg.EXTENT_FIX_CELLS)
    out.update({
        'size_reference': [rw, rh], 'size_fixed': [fw, fh],
        'origin': list(fixed['origin']), 'origin_reference': list(ref['origin']),
        'origin_unchanged': bool(same_origin),
        'grew_by_exactly_one_per_axis': bool(grew),
    })
    if not grew:
        out['n_overlap_differing'] = None
        out['overlap_identical'] = False
        out['holds'] = False
        return out
    diff = fixed['grid'][:rh, :rw] != ref['grid']
    n_diff = int(np.count_nonzero(diff))
    out['n_overlap_cells'] = int(ref['grid'].size)
    out['n_overlap_differing'] = n_diff
    out['overlap_identical'] = n_diff == 0
    out['holds'] = bool(n_diff == 0 and same_origin)
    return out


def added_cells(grid: np.ndarray, rw: int, rh: int,
                inset: int = 0) -> np.ndarray:
    """The far column and far row of `grid`, as one flat array.

    `inset` steps that many cells INSIDE the added ones. It is applied to ONE side
    of the comparison only -- see r3 -- because insetting both would read the same
    pair of lines from each grid and compare like with like, which is how the first
    version of the interior-row break failed to break anything.
    """
    col = grid[:, rw - inset]
    row = grid[rh - inset, :]
    return np.concatenate([col, row])


def r3(stem: str, maps_dir: Path, pred_dir: Path,
       break_name: str | None) -> dict:
    """R3: the added row and column are what the extent prediction said.

    Observed cells are those either side calls something other than UNKNOWN, the
    same denominator C1 uses, so a pair of unknown cells is not counted as
    agreement. The prediction and the map share an origin and a resolution, so
    index i means the same cell in both and no resampling is involved.
    """
    ref_stem, ref_kind = unpatched_reference(stem, maps_dir)
    pstem = plain_stem(stem)
    if break_name == 'other-robot':
        ref_stem, pstem = other_robot_stem(ref_stem), other_robot_stem(pstem)
        ref_kind = 'ANOTHER ROBOT (BROKEN ON PURPOSE)'
    out = {'map': stem, 'unpatched_reference': ref_stem,
           'reference_kind': ref_kind, 'prediction': pstem}
    if not (maps_dir / f'{stem}.pgm').is_file():
        out['status'] = f'not found: {rel(maps_dir / f"{stem}.pgm")}'
        return out
    if not (maps_dir / f'{ref_stem}.pgm').is_file():
        out['status'] = (f'the unpatched map {ref_stem} is missing, so the old '
                         'width and height are unknown and R3 cannot say which '
                         'cells the patch added')
        return out
    if not (pred_dir / f'{pstem}.pgm').is_file():
        out['status'] = (f'{pstem}.pgm is not in '
                         f'{rel(pred_dir)} -- run '
                         'predict_gated_maps.py --extent-fix first')
        return out
    fixed = read_grid(stem, maps_dir)
    ref = read_grid(ref_stem, maps_dir)
    pred = read_grid(pstem, pred_dir)
    rh, rw = ref['grid'].shape
    fh, fw = fixed['grid'].shape
    ph, pw = pred['grid'].shape
    out['size_fixed'] = [fw, fh]
    out['size_prediction'] = [pw, ph]
    out['size_unpatched'] = [rw, rh]
    if (fw, fh) != (pw, ph):
        out['status'] = (f'the map is {fw}x{fh} and the prediction {pw}x{ph}; '
                         'R3 compares the SAME cells of two grids and will not '
                         'resample to make them line up')
        out['holds'] = False
        return out
    # The break insets the PREDICTION only: the map's added cells are still the
    # ones under test, and they are asked to agree with lines the patch did not
    # add. Symmetric insetting would agree trivially.
    inset = 1 if break_name == 'interior-row' else 0
    a = added_cells(fixed['grid'], rw, rh, 0)
    b = added_cells(pred['grid'], rw, rh, inset)
    out['prediction_inset_cells'] = inset
    obs = (a != UNKNOWN) | (b != UNKNOWN)
    n = int(np.count_nonzero(obs))
    n_agree = int(np.count_nonzero((a == b) & obs))
    out.update({
        'n_added_cells': int(a.size),
        'n_observed': n,
        'n_agree': n_agree,
        'agreement': (float(n_agree) / n) if n else float('nan'),
        'limit': R3_MIN,
        'holds': bool(n and n_agree / n >= R3_MIN),
    })
    return out


CHECKS = {'R1': 'a map built by the UNPATCHED overlay is byte-identical to the '
                'one the installed binary built',
          'R2': 'the patched map adds only the far row and column; the overlap '
                'is identical and the origin unchanged',
          'R3': f'the added row and column agree with the extent prediction on '
                f'>= {R3_MIN:.1%} of observed cells'}
BREAK_MUST_FAIL = {'other-robot': ('R1', 'R2'),
                   'shift-one-cell': ('R1', 'R2'),
                   'interior-row': ('R3',)}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='append', dest='checks',
                    choices=sorted(CHECKS),
                    help='which check to run; repeatable. Default: all three')
    ap.add_argument('--map', action='append', dest='maps', required=True,
                    help='one overlay-built stem, e.g. '
                         'b2maps_k0_cut60_gated_ovl_robot0. Repeatable')
    ap.add_argument('--maps-dir', default=str(gw.MAPS_DIR_DEFAULT))
    ap.add_argument('--pred-dir', default=str(pg.OUT_DIR_EXTENT),
                    help="R3's reference predictions")
    ap.add_argument('--break', dest='break_name', choices=BREAKS,
                    help='run a named break. Writes nothing, exits 2. '
                         + '; '.join(f'{b} must fail '
                                     f'{"/".join(BREAK_MUST_FAIL[b])}'
                                     for b in BREAKS))
    ap.add_argument('--no-json', action='store_true')
    args = ap.parse_args()
    checks = args.checks or sorted(CHECKS)
    maps_dir, pred_dir = Path(args.maps_dir), Path(args.pred_dir)

    print('comparing overlay-built maps against the maps already here')
    for name in checks:
        print(f'  {name}  {CHECKS[name]}')
    if args.break_name:
        print()
        print(f'  (BROKEN ON PURPOSE: --break {args.break_name}. '
              f'{"/".join(BREAK_MUST_FAIL[args.break_name])} must FAIL; any of '
              'them still')
        print('   holding would mean the check is not comparing what it claims. '
              'Nothing is written.)')
    print()

    rows: dict[str, dict] = {}
    for stem in args.maps:
        if not gw.stem_variants(stem):
            gw.die(f'{stem} carries no variant token, so it names a map built by '
                   'the installed binary and there is nothing to compare it '
                   'with. Pass an _ovl or _extfix stem.')
        row = {}
        if 'R1' in checks:
            row['R1'] = r1(stem, maps_dir, args.break_name)
        if 'R2' in checks:
            row['R2'] = r2(stem, maps_dir, args.break_name)
        if 'R3' in checks:
            row['R3'] = r3(stem, maps_dir, pred_dir, args.break_name)
        rows[stem] = row

    print(f'{"map":<40}{"check":>7}{"verdict":>9}  detail')
    for stem, row in rows.items():
        for name in checks:
            r = row[name]
            if 'status' in r:
                print(f'{stem:<40}{name:>7}{"SKIP":>9}  {r["status"][:60]}')
                continue
            detail = {
                'R1': lambda: (f'bytes {r["bytes_identical"]}, geometry '
                               f'{r["geometry_identical"]}'),
                'R2': lambda: (f'{r["n_overlap_differing"]} overlap cells differ, '
                               f'+1 {r["grew_by_exactly_one_per_axis"]}, origin '
                               f'{r["origin_unchanged"]}'),
                'R3': lambda: (f'{r["agreement"]:.4%} of {r["n_observed"]} '
                               f'observed added cells'),
            }[name]()
            print(f'{stem:<40}{name:>7}'
                  f'{("HELD" if r["holds"] else "FAILED"):>9}  {detail}')
    print()

    verdicts = {}
    for name in checks:
        done = [s for s in rows if 'status' not in rows[s][name]]
        bad = [s for s in done if not rows[s][name]['holds']]
        verdicts[name] = {'maps_judged': len(done), 'maps_skipped':
                          [s for s in rows if s not in done],
                          'failing': bad, 'holds': bool(done) and not bad}
        if not done:
            print(f'  {name}: NO MAP JUDGED -- every one was skipped, so this is '
                  'neither held nor failed')
        else:
            print(f'  {name}: {len(done) - len(bad)}/{len(done)} '
                  + ('HELD' if verdicts[name]['holds'] else 'FAILED')
                  + ('' if not bad else f' -- failed on {bad}'))
        if len(done) != len(rows):
            print(f'        {len(rows) - len(done)} map(s) skipped for want of '
                  'an input; a skip is not a pass')
    print()

    if args.break_name:
        must = [c for c in BREAK_MUST_FAIL[args.break_name] if c in checks]
        # A check with nothing judged did not fail, it was never run. Counting a
        # skip as the break working is the one way this report could lie.
        vacuous = [c for c in must if verdicts[c]['maps_judged'] == 0]
        still = [c for c in must if verdicts[c]['maps_judged']
                 and verdicts[c]['holds']]
        failed = [c for c in must if verdicts[c]['maps_judged']
                  and not verdicts[c]['holds']]
        print(f'--break {args.break_name}: must fail {must}; {failed} failed as '
              'required')
        if vacuous:
            print(f'  {vacuous} judged NO map -- every one was skipped for want '
                  'of an input, so the break is INCONCLUSIVE for them, not '
                  'satisfied.')
        if still:
            print(f'  {still} STILL HELD under the break. Then it is not '
                  'measuring what it claims, and that is a finding.')
        print('  Nothing was written.')
        return 2

    out = {
        'question': 'does a map built by the slam_toolbox overlay match the one '
                    'the installed binary built, and does the patch add only the '
                    'far row and column',
        'checks': CHECKS,
        'breaks': {b: f'must fail {"/".join(BREAK_MUST_FAIL[b])}' for b in BREAKS},
        'read_through': 'trinary_map only, via compare_gated_maps.read_grid',
        'truth_used': False,
        'e3_note': 'E3 (max-side A9 recall within 2 pp of min-side) is truth '
                   'derived and is NOT computed here; diagnose_extent.py does it '
                   'for any stem, including an _extfix one',
        'stem_naming': 'b2maps_k{K}_cut{C}[_gated][_ovl|_extfix]_robot{K}; '
                       'graph_walls.stem_variants owns the parser',
        'patch': {'file': pg.EXTENT_FIX_PATCH,
                  'cells_added_per_axis': pg.EXTENT_FIX_CELLS},
        'provenance': {'cell_centre': cell_centre_provenance()},
        'maps_dir': rel(maps_dir),
        'pred_dir': rel(pred_dir),
        'verdicts': verdicts,
        'per_map': rows,
    }
    rc = 0 if all(v['holds'] for v in verdicts.values()) else 2
    if args.no_json:
        print('--no-json: nothing written')
        return rc
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {rel(OUT_PATH)}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
