#!/usr/bin/env python3
"""Cell-wise agreement between nav2 trinary PGM maps of the SAME world.

Written to measure replay noise: two offline SLAM replays of one stripped
segment should produce one map, and this says by how much they do not. Cell
counts alone cannot answer that -- two maps with identical occupied/free/unknown
totals can disagree cell for cell -- so this compares classes per cell.

COMPARISON IS IN WORLD COORDINATES, and it has to be. Two replays of one
segment do NOT share a grid: measured on b2maps_k4_cut60, two maps of the same
120x140 extent carried origins 0.479 and 0.243 CELLS apart, i.e. slam_toolbox
rasterises what it mapped onto an origin it recomputes per save, and that origin
is continuous rather than snapped to the resolution. So array index i of one map
is not array index i of the other, and no integer shift makes it so.

Each map is therefore SAMPLED at the cell centres of a shared lattice: for every
lattice centre, the class of the cell containing that point, or unknown outside
the map. The lattice is anchored on the REFERENCE map's own origin and extended
by whole cells, so the reference is sampled exactly (no resampling error at all)
and only the comparison map is resampled -- by strictly less than one cell. Row 0
of a nav2 PGM is maximum y, so the vertical axis is flipped once on the way in.

That sub-cell resampling is the measurement floor: at a class boundary a shift
of half a cell can move which side a cell falls on, so a pair of maps that
differ ONLY in origin will still show some disagreement here. The origin offset
in cells is printed for every pair so that contribution stays visible, and two
byte-identical maps still measure exactly zero.

Resolution must match to 1e-9; anything else is not two maps of one world.

    python3 experiments/analysis/pgm_agreement.py REF.pgm OTHER.pgm [MORE.pgm ...]

Prints one row per comparison map: its own class counts, then the disagreement
against REF as a count and as a percentage. The percentage's denominator is
stated in the output because there is more than one defensible choice:

  %union  differing / cells KNOWN IN EITHER map (occupied or free in one or
          both). This is the headline number -- it charges the comparison for
          a cell that one replay mapped and the other left unknown, which is
          exactly the kind of difference replay noise produces.
  %ref    differing / cells known in REF. Reported alongside because it is the
          figure to quote when REF is treated as the reference map rather than
          as one sample of a distribution.

Reads only the PGM and its .yaml sidecar. numpy and PyYAML, no ROS.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

# nav2 trinary PGM values, the convention save_map.py writes and
# run_offline_maps.sh verifies against.
OCC, FREE, UNKNOWN = 0, 254, 205

RES_TOL = 1e-9          # two maps of one world share a resolution exactly
EPS = 1e-9              # float slack when flooring an exact cell boundary


def read_pgm(path: Path) -> np.ndarray:
    """Pixels of a binary P5 PGM as (h, w) uint8.

    Same header walk as run_offline_maps.sh's verifier and pgm_extent.py:
    three whitespace-separated fields after the magic, '#' comments allowed.
    """
    data = path.read_bytes()
    if data[:2] != b'P5':
        sys.exit(f'{path}: not a binary PGM')
    i, fields = 2, []
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _maxval = fields
    px = data[i:i + w * h]
    if len(px) != w * h:
        sys.exit(f'{path}: header says {w}x{h} but only {len(px)} pixel bytes follow')
    return np.frombuffer(px, dtype=np.uint8).reshape(h, w)


def load(path: Path) -> dict:
    meta_path = path.with_suffix('.yaml')
    for p in (path, meta_path):
        if not p.is_file():
            sys.exit(f'not found: {p}')
    meta = yaml.safe_load(meta_path.read_text())
    px = read_pgm(path)
    return {
        'path': path,
        'md5_px': hash(px.tobytes()),
        'px': px,
        'height': px.shape[0],
        'width': px.shape[1],
        'resolution': float(meta['resolution']),
        'origin': [float(v) for v in meta['origin']],
        'occupied': int(np.count_nonzero(px == OCC)),
        'free': int(np.count_nonzero(px == FREE)),
        'unknown': int(np.count_nonzero(px == UNKNOWN)),
        'other': int(np.count_nonzero((px != OCC) & (px != FREE) & (px != UNKNOWN))),
    }


def sample(m: dict, res: float, x0: float, y0: float,
           w: int, h: int) -> np.ndarray:
    """`m`'s classes at the centres of a (h, w) lattice starting at (x0, y0).

    Lattice cell (0, 0) has its centre at (x0 + res/2, y0 + res/2) and rows go
    UPWARD in y, so the returned array is bottom-up rather than in PGM order.
    Centres outside the map are unknown.
    """
    xs = x0 + (np.arange(w) + 0.5) * res
    ys = y0 + (np.arange(h) + 0.5) * res
    cols = np.floor((xs - m['origin'][0]) / res + EPS).astype(np.int64)
    rows = np.floor((ys - m['origin'][1]) / res + EPS).astype(np.int64)

    out = np.full((h, w), UNKNOWN, dtype=np.uint8)
    keep_c = (cols >= 0) & (cols < m['width'])
    keep_r = (rows >= 0) & (rows < m['height'])
    if keep_c.any() and keep_r.any():
        flipped = m['px'][::-1, :]        # bottom-up, so +y is +row
        out[np.ix_(keep_r, keep_c)] = flipped[np.ix_(rows[keep_r], cols[keep_c])]
    return out


def compare(ref: dict, other: dict) -> dict:
    res = ref['resolution']
    if abs(other['resolution'] - res) > RES_TOL:
        sys.exit(f'{other["path"].name}: resolution {other["resolution"]!r} != '
                 f'{res!r} -- not two maps of one world')

    rox, roy = ref['origin'][:2]
    oox, ooy = other['origin'][:2]
    # Offset in cells, reported: its fractional part is the resampling this
    # comparison has to do, and therefore its error floor.
    dx_cells = (oox - rox) / res
    dy_cells = (ooy - roy) / res

    # Lattice anchored on the reference and grown by WHOLE cells, so the
    # reference samples exactly and only `other` is resampled.
    kl = math.ceil(max(0.0, rox - oox) / res - EPS)
    kb = math.ceil(max(0.0, roy - ooy) / res - EPS)
    x0 = rox - kl * res
    y0 = roy - kb * res
    right = max(rox + ref['width'] * res, oox + other['width'] * res)
    top = max(roy + ref['height'] * res, ooy + other['height'] * res)
    w = math.ceil((right - x0) / res - EPS)
    h = math.ceil((top - y0) / res - EPS)

    a = sample(ref, res, x0, y0, w, h)
    b = sample(other, res, x0, y0, w, h)

    diff = a != b
    known_either = (a != UNKNOWN) | (b != UNKNOWN)
    n_diff = int(np.count_nonzero(diff))
    n_union = int(np.count_nonzero(known_either))
    n_ref = int(np.count_nonzero(a != UNKNOWN))

    # Which way the disagreements go, because "1% differ" reads differently if
    # it is occupied-vs-free than if it is mapped-vs-not-yet-mapped.
    def n(from_v, to_v):
        return int(np.count_nonzero((a == from_v) & (b == to_v)))

    return {
        'window': (w, h),
        'identical_bytes': (ref['width'] == other['width']
                            and ref['height'] == other['height']
                            and ref['md5_px'] == other['md5_px']
                            and ref['origin'] == other['origin']),
        'dx_cells': dx_cells,
        'dy_cells': dy_cells,
        'subcell': max(abs(dx_cells - round(dx_cells)),
                       abs(dy_cells - round(dy_cells))),
        'diff': n_diff,
        'known_union': n_union,
        'known_ref': n_ref,
        'pct_union': 100.0 * n_diff / n_union if n_union else float('nan'),
        'pct_ref': 100.0 * n_diff / n_ref if n_ref else float('nan'),
        'unknown_to_free': n(UNKNOWN, FREE),
        'unknown_to_occ': n(UNKNOWN, OCC),
        'free_to_unknown': n(FREE, UNKNOWN),
        'occ_to_unknown': n(OCC, UNKNOWN),
        'free_to_occ': n(FREE, OCC),
        'occ_to_free': n(OCC, FREE),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ref', type=Path, help='reference map (.pgm)')
    ap.add_argument('others', type=Path, nargs='+', help='maps to compare to it')
    ap.add_argument('--quiet', action='store_true',
                    help='one line per map, no class-transition breakdown')
    args = ap.parse_args()

    ref = load(args.ref)
    print(f'reference {ref["path"].name}: {ref["width"]}x{ref["height"]} @ '
          f'{ref["resolution"]} m, origin ({ref["origin"][0]:.4f}, '
          f'{ref["origin"][1]:.4f})')
    print(f'  {ref["occupied"]} occupied, {ref["free"]} free, '
          f'{ref["unknown"]} unknown'
          + (f', {ref["other"]} OTHER' if ref['other'] else ''))
    print()
    print(f'{"map":<30}{"occupied":>9}{"free":>8}{"unknown":>9}'
          f'{"differ":>8}{"%union":>8}{"%ref":>7}{"d cells":>16}  same?')
    for path in args.others:
        m = load(path)
        c = compare(ref, m)
        tag = ('IDENTICAL' if c['identical_bytes']
               else f'subcell {c["subcell"]:.3f}')
        print(f'{path.name:<30}{m["occupied"]:>9}{m["free"]:>8}{m["unknown"]:>9}'
              f'{c["diff"]:>8}{c["pct_union"]:>8.3f}{c["pct_ref"]:>7.3f}'
              f'{f"{c["dx_cells"]:+.3f},{c["dy_cells"]:+.3f}":>16}  {tag}')
        if not args.quiet and c['diff']:
            print(f'{"":<30}unknown->free {c["unknown_to_free"]}, '
                  f'unknown->occ {c["unknown_to_occ"]}, '
                  f'free->unknown {c["free_to_unknown"]}, '
                  f'occ->unknown {c["occ_to_unknown"]}, '
                  f'free->occ {c["free_to_occ"]}, '
                  f'occ->free {c["occ_to_free"]}')
    print()
    print('  %union  = differing cells / cells known (occupied or free) in either map')
    print('  %ref    = differing cells / cells known in the reference')
    print('  d cells = this map\'s origin minus the reference\'s, in cells. Its')
    print('            fractional part is resampled away and is this measure\'s floor.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
