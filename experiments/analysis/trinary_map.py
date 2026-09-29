#!/usr/bin/env python3
"""One reader for the nav2 trinary PGM/YAML maps this project writes.

THE RULE, stated once so that no reader has to restate it:

    The YAML thresholds split OCCUPIED from FREE. The reserved unknown byte 205
    then OVERRIDES them. Any fourth pixel value is a refusal, not a guess.

The override is not defensive programming, it is arithmetic. With `negate: 0`
map_server's occupancy probability for a pixel is (255 - px) / 255, so the
reserved unknown byte gives

    shade(205) = (255 - 205) / 255 = 50 / 255 = 0.19607...

and every map this project writes carries `free_thresh: 0.25` beside it. 0.196 is
BELOW 0.25, so the thresholds alone call every unknown cell FREE. The writer's own
round trip is not a fixpoint. The band the thresholds DO map to unknown is bytes
90..191, which no pixel of any map in experiments/maps/ occupies.

Applying the thresholds literally is not a subtle error. It reported all 20
offline b2maps grids as fully known: Jaccard became extent overlap, the 3526
unknown cells of b2maps_k0_cut60_robot0 vanished, and the coverage half of
robot_divergence.py silently measured nothing. That run is kept at
experiments/logs/robot_divergence_b2maps_*_BROKEN_unknown_read_as_free.json.

THREE WRITERS produce these files, and all three hardcode the same 0.65 / 0.25
beside the same 205:

    experiments/slam/save_map.py:91
    experiments/analysis/run_health.py:1878
    ros2_ws/src/nsk_swarm/nsk_swarm/frontier_explorer.py:883-884

They are deliberately NOT changed to write a recovering free_thresh. 0.65 / 0.25
are stock nav2 map_saver defaults and the files claim `mode: trinary`, so writing
something else would make them no longer what map_saver would have written --
which is the contract save_map.py:5-6 states. The reserved-byte rule is the fix
regardless of what the thresholds say, and it is the fix for stock nav2 maps too,
which carry exactly the same quirk at exactly the same defaults.

WHAT 205 MEANS here. save_map.py writes it for a true unknown (-1) AND for the
uncommitted 26..64 occupancy band alike (run_health.py:1789-1807 documents the
same conflation from the grid side). So "unknown" reads as "this robot did not
commit the cell to free or occupied" -- which is the notion a coverage question
wants anyway. A consequence worth knowing: a map's grid-side `unknown` count and
its PGM-side `unknown` count are different numbers by construction, and
test_run_health.py:1896 pins exactly that relationship.

NEGATE INVERTS THE TRAP INTO SOMETHING WORSE. With `negate: 1` the shade is
px / 255, so shade(205) = 0.804, which is ABOVE occupied_thresh 0.65 -- 205 would
read as a WALL. No writer here emits negate: 1, and check_thresholds() dies on
such a YAML rather than classify under it, because a YAML whose reserved occupied
byte does not read as occupied is describing some other image.

numpy and PyYAML only. No PIL, no scipy, no ROS -- so this imports cleanly in
CI's ros:jazzy container, where a module-level scipy import is a COLLECTION error
that takes the whole nsk_swarm suite down (fit_world_transform.py:53).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

# nav2 trinary PGM values, the convention all three writers above emit and
# run_offline_maps.sh's verifier checks against.
OCC, FREE, UNKNOWN = 0, 254, 205

RES_TOL = 1e-9          # two maps of one world share a resolution exactly
EPS = 1e-9              # float slack when flooring an exact cell boundary


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' own convention."""
    print(f'trinary_map: {msg}', file=sys.stderr)
    sys.exit(2)


def read_pgm(path) -> np.ndarray:
    """Pixels of a binary P5 PGM as (h, w) uint8.

    Three whitespace-separated fields after the magic, '#' comments allowed --
    the same header walk run_offline_maps.sh's verifier and run_b2maps_cuts.sh's
    pgm_counts implement inline.

    Accepts a str or a Path: pgm_extent.py's callers pass strings (see
    test_run_health.py:1890) and pgm_agreement.py's pass Paths.
    """
    path = Path(path)
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
    """A map's pixels, its sidecar geometry, and its raw class counts.

    The counts are by RESERVED BYTE, not by threshold -- they describe what is in
    the file. 'other' is any fourth value, which check_thresholds() refuses.
    """
    path = Path(path)
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


def shade(px: np.ndarray | int | float, negate: int) -> np.ndarray | float:
    """map_server's occupancy probability for a pixel value.

    Byte-identical to fit_world_transform.load_map:276 -- negate flips which end
    of the range means occupied. Split out so check_thresholds() can ask the same
    question of a single byte that classify() asks of a whole image.
    """
    return (px / 255.0) if negate else ((255.0 - px) / 255.0)


def classify(px: np.ndarray, negate: int, occupied_thresh: float,
             free_thresh: float) -> np.ndarray:
    """Recode raw PGM pixels to OCC / FREE / UNKNOWN for one map.

    The thresholds make the occupied/free split, as map_server's trinary mode
    does; the RESERVED UNKNOWN BYTE then overrides, and it has to -- see this
    module's docstring for why 205 cannot be recovered from the thresholds these
    files carry.

    The occupied set is untouched by any of this: 205 was never above
    occupied_thresh under negate 0. test_robot_divergence pins classify's
    occupied set equal to fit_world_transform.load_map's, cell for cell, so the
    two implementations cannot drift apart -- which is why the fitter keeps its
    own independent occupied-only path rather than calling this.

    Returned in PGM row order, still row 0 = maximum y; sample() does the flip.
    """
    p_occ = shade(px, negate)
    out = np.full(px.shape, UNKNOWN, dtype=np.uint8)
    out[p_occ > occupied_thresh] = OCC
    out[p_occ < free_thresh] = FREE
    out[px == UNKNOWN] = UNKNOWN     # the reserved byte, last so it overrides
    return out


def check_thresholds(negate: int, occupied_thresh: float, free_thresh: float,
                     other: int, where: str) -> dict:
    """Verify a map's YAML actually describes its own PGM. Reports the quirk.

    Three things, each able to fail:
      * the thresholds must be an ordered pair in [0, 1];
      * the reserved occupied byte must read as occupied and the reserved free
        byte as free -- if either does not, the YAML does not describe this PGM
        and nothing downstream means anything. This is what catches negate: 1,
        under which byte 0 reads free and byte 205 reads as a wall;
      * every pixel must be one of the three reserved bytes. The writers emit
        only those, and for any other byte the reserved-byte rule says nothing,
        so a fourth value is a refusal rather than a guess.

    Whether the reserved unknown byte reads back as unknown is REPORTED, not
    required. On the 183 maps carrying free_thresh 0.25 it does not, which is the
    whole reason classify() overrides it. On the three carrying 0.196
    (rung1b/1c/1d_robot0) it does -- by 7.8e-5, since shade(205) = 0.196078... is
    only just above 0.196. Either way classify() returns the same three states,
    and neither value is a reason to treat one map differently from another.
    """
    if not 0.0 <= free_thresh <= occupied_thresh <= 1.0:
        die(f'{where}: thresholds free={free_thresh} occupied={occupied_thresh} '
            'are not an ordered pair in [0, 1]')

    def band(byte: int) -> str:
        s = float(shade(float(byte), negate))
        if s > occupied_thresh:
            return 'occupied'
        if s < free_thresh:
            return 'free'
        return 'unknown'

    for byte, want in ((OCC, 'occupied'), (FREE, 'free')):
        if band(byte) != want:
            die(f'{where}: the reserved {want} byte {byte} reads as '
                f'{band(byte)!r} under this YAML (negate={negate}, '
                f'occupied_thresh={occupied_thresh}, free_thresh={free_thresh}). '
                'The YAML does not describe its own PGM.')
    if other:
        die(f'{where}: {other} pixels are none of the three reserved bytes '
            f'({OCC}, {FREE}, {UNKNOWN}). The writers emit only those, so the '
            'reserved-byte rule classify() relies on says nothing about these '
            'and guessing is worse than stopping.')

    return {'unknown_byte': UNKNOWN,
            'unknown_byte_shade': float(shade(float(UNKNOWN), negate)),
            'unknown_byte_band_under_thresholds': band(UNKNOWN),
            'unknown_byte_recovered_by_thresholds': band(UNKNOWN) == 'unknown',
            'threshold_unknown_band_bytes': [
                b for b in range(256) if band(b) == 'unknown'][:1] + [
                b for b in range(256) if band(b) == 'unknown'][-1:]}


def sample(m: dict, res: float, x0: float, y0: float,
           w: int, h: int) -> np.ndarray:
    """`m`'s classes at the centres of a (h, w) lattice starting at (x0, y0).

    Lattice cell (0, 0) has its centre at (x0 + res/2, y0 + res/2) and rows go
    UPWARD in y, so the returned array is bottom-up rather than in PGM order.
    Centres outside the map are unknown.

    Handed a grid already recoded by classify(), this is a three-state resampler
    with no change to it at all: the row flip and the outside-the-map fill are
    the whole of convention A.
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
