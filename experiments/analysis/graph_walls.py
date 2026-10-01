#!/usr/bin/env python3
"""Wall faces out of one robot's own trinary grid. Part A of the wall
predicate.

docs/specs/SPEC_b2_wall_predicate.md is what this file implements: §3's six
definitions, §4's extraction procedure, §7's output, and the A8-A11 corpus
validation of §6. Part B (viewing directions, §5) is a later stop point and this
writes `"obs": null` for it.

A NODE IS A FACE, NOT A WALL (D1, C2)
-------------------------------------
The unit of extraction is an ORIENTED occupied-free cell boundary -- one side of
one slab, seen from the free side. Three consequences, and they are the whole
reason v2 of the spec exists:

  * an occupied-UNKNOWN boundary gives nothing. Nothing was seen from that side,
    so there is no observation to record;
  * a segment accepts only elements whose normal agrees with its own (W4), so
    every segment is one-sided by construction and no `±` label is needed;
  * faces sit on the true surface, so the two faces of a 0.2 m maze wall come out
    0.20 m apart, not 0.10. A one-cell wall free on both sides gives two
    segments whose lines are 0.10 m apart, the same cell feeding one element to
    each.

Pairing a wall's two faces back together is an EDGE, and belongs to the later
edge spec. This file never does it.

W4 AND THE PERPENDICULAR FACE
-----------------------------
W4 is `n_e · n > 0`. Every element normal here is one of four axis directions, so
for an axis-aligned line the two PERPENDICULAR faces (the step risers at a
corner, the end caps of a wall) have `n_e · n` exactly 0 -- and `> 0` is false for
them, which is exactly right: a face at right angles to a segment is not part of
it. In floating point cos(90°) is 6.1e-17, not 0, so a literal `> 0` would admit
those faces whenever the sign of that noise happened to be positive, and a corner
cell's riser would join the wall it abuts. FACING_MIN restores the exact
arithmetic rather than departing from it.

For a line at any other angle nothing is on the knife edge: a 17° face
rasterises into a staircase whose top faces and side faces BOTH satisfy W4
against its line (dot 0.956 and 0.292), and both are genuinely part of that one
face. That is what makes an off-axis wall extractable at all.

THE HOUGH LOOP (§4, and the two choices §4 leaves open)
-------------------------------------------------------
Oriented: `theta` runs over the full 360°, and each element votes only for the
half of them its own normal agrees with. So the two faces of one slab never
compete for one accumulator cell -- they live at (theta, d) and (theta+180, -d).

  * PEAK DETECTION SUMS A 3-BIN `d` WINDOW (0.30 m, inside the 2*eps = 0.3414 m
    §4 step 2 allows). Not cosmetic: an axis-aligned face's `d` is
    `origin + k*rho`, and these maps' YAML origins are arbitrary reals, so a map
    whose origin fell near a half-cell would split one wall's votes across two
    adjacent bins on float noise alone. The window also makes the stop threshold
    `l_min / (sqrt(2)*rho)` = 3.54 votes a statement about a neighbourhood
    instead of about one bin.
  * THE PEAK LINE'S `d` IS THE MEAN `d` OF THE LIVE VOTES IN THAT WINDOW, not the
    bin centre -- a closer seed for step 4. Step 4's total-least-squares refit
    then fixes the reported line, so no reported geometry depends on this.

The loop is a LAZY MAX-HEAP over (window sum, theta, d), which is an
implementation of "take the highest unexhausted peak", not a change to it: votes
only ever decrease, so a popped entry whose stored key still equals its live
value is the true maximum. Rescanning a 720 x ~600 accumulator on every one of
several thousand iterations would give the same sequence of peaks and take
minutes.

WHICH LINE A SEGMENT REPORTS
----------------------------
Def 2 asks for the total-least-squares fit to the segment's own elements, and
that is what is reported whenever W1, W2 and W4 still hold under it. They do not
always: a short run inside a band `eps` = 1.71 cells wide is a blob with no
direction, and one on the corpus refitted 33° away from the line it was cut on,
which put its own elements 1.30*eps off it. A segment that fails W1 or W2 under
its own reported line is not a wall segment, so those fall back to the line the
run was cut on, which satisfies all three by construction. `line_source` says
which, per segment. See _segment().

BANDS READ OUTSIDE THE MAP AS UNKNOWN
-------------------------------------
Def 3's band is clipped by nothing: a band cell that falls outside the grid
counts as UNKNOWN, which is what trinary_map.sample() fills outside a map
anyway. Without that, a wall running to the edge of the PGM would have an EMPTY
band beyond its endpoint, and Def 5's precedence would fall through to `free` --
reporting "the wall visibly ends" about the one place the map cannot say
anything. Now it reports `unknown`, which is what it knows.

PART B: VIEWING DIRECTIONS (§5)
-------------------------------
Each segment also records the DIRECTIONS it was seen from, because §0.1's schema
requirement is that corroboration may not be weighted by a count of observers:
five robots that share a frame, an odometry model and a rangefinder model make
correlated errors, so what has to be stored is the geometric spread of
viewpoints. Hence `bearing_hist`, 72 bins of 5°, and `n_scans` -- not `n_hits`,
because 360 beams of one scan from one pose are one observation of a wall seen
360 ways.

Three things here are easy to get wrong and are therefore stated:

  * THE SCAN TOPIC IS THE RAW /robot_K/scan. The replay fed Karto the
    free_space_relay's /robot_K/scan_free, which no bag contains -- the relay ran
    live. It rewrites only +inf and readings AT OR ABOVE range_max, filling them
    at 7.95 m so Karto traces them as free space; every finite reading below
    r_max = 7.9 passes through untouched. Those are exactly the returns §5 keeps,
    so the raw topic carries precisely the ones that could have marked a cell
    occupied.
  * POSES COME FROM /tf, NEVER /robot_K/odom (C10). Both are in the bag; the
    odom topic drifts 8.7 m over a run and is not what the replay used.
  * `base_T_scan` IS COMPOSED FROM THE TWO-HOP /tf_static CHAIN (C7),
    base_footprint -> base_link -> base_scan, which comes to x = -0.032 m. Omit
    it and every hit point lands 3.2 cm from where the sensor saw it -- B4's
    break, and note what it does NOT move: a pure translation of the sensor
    cannot rotate a ray, so the BEARING is unchanged. What moves is where the
    ray ends, which is what B1 and the attribution see.

TRUTH
-----
The product -- one JSON per map -- contains no world geometry at all. A8-A11 are
validation of THIS extractor and nothing else, and §6 forbids any later tool from
reading them, so they are written to a separate `_validation_<stamp>.json` and
never into `<map>.json`. A number that is not in the per-map file cannot be read
out of it by accident.

The four predictions are REPORTED, never gated, following robot_divergence.py:596
-- a failed prediction is a finding to write down, not a number to move. The exit
code answers a different question: 2 when an input is unusable or a self-check
about this tool's own arithmetic fails (the SDF no longer giving 212.00 m and
128.80 m; a YAML that does not describe its own PGM; occupied cells that are
neither in a segment nor unclassified).

IMPORTS
-------
numpy, PyYAML and trinary_map at module scope, nothing else -- so this file and
its test suite keep collecting in CI's ros:jazzy container, which has no scipy,
no PIL and no ROS. bag_overlap (the SDF and the spawn table), robot_divergence
(the registration) and fit_world_transform (the map->odom logs) are imported
INSIDE the validation functions: they shell out to git and read the world, and
the extractor must need neither.

    python3 experiments/analysis/graph_walls.py
    python3 experiments/analysis/graph_walls.py --map b2maps_k0_cut60_robot0 \
        --no-json
    python3 experiments/analysis/graph_walls.py --no-validate

Writes one JSON per map to experiments/logs/graph_walls/ plus one validation
JSON. Seconds per map, no simulator.
"""

from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
import os
import re
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

# The shared reader: the P5 header walk, the YAML sidecar, and the three-state
# rule (the thresholds split occupied from free, the reserved byte 205
# OVERRIDES). sample() is the convention-A resampler; asked for a map's own
# lattice it is the identity, and load_grid() verifies exactly that rather than
# reimplementing the row flip. See trinary_map.py's docstring for why 205 cannot
# be recovered from the thresholds these files carry.
from trinary_map import (  # noqa: E402
    FREE,
    OCC,
    UNKNOWN,
    check_thresholds,
    classify,
    load,
    sample,
)

# experiments/maps by default. Overridable for the same reason
# test_trinary_map.py:52 overrides it: the corpus is gitignored, so the
# absent-corpus path has to be reachable without moving the real maps aside.
MAPS_DIR_DEFAULT = Path(os.environ.get(
    'NSK_TRINARY_MAPS_DIR', str(REPO_ROOT / 'experiments' / 'maps')))
LOGS_DIR = REPO_ROOT / 'experiments' / 'logs'
OUT_DIR_DEFAULT = LOGS_DIR / 'graph_walls'
NAV_DIR = REPO_ROOT / 'experiments' / 'nav'

NUM_ROBOTS = 5
CUTS = (60, 120, 240, 1200)
RUN_PREFIX = 'b2maps'

# The stripped bags the offline maps were replayed from, one per cut. Part B
# reads these and nothing else; Part A never opens a bag.
BAGS_DIR = LOGS_DIR / RUN_PREFIX

# The b2maps-era spawn table, as robot_divergence.py:148 states: bag_overlap's
# own default is the b16 ring and puts robot_0 3.15 m from where these bags
# spawned it.
SPAWN_REV = '08617b2'

# ── pre-registered thresholds (§8) ───────────────────────────────────────────
# Every one of these was fixed in the spec before this file existed. The two
# that are READ rather than pre-registered -- rho from each map's YAML and g from
# the nav2 params -- are read at run time and carry their source into the JSON.
L_MIN = 0.5                              # m (D2)
EPS_FACTOR = 1.0 + math.sqrt(2.0) / 2.0  # eps = EPS_FACTOR * rho = 0.1707 m
B_FACTOR = math.sqrt(2.0) / 2.0          # b   = B_FACTOR   * rho = 0.0707 m
DTHETA_DEG = 0.5                         # over the full 360°
WINDOW_BINS = 3                          # 3*rho = 0.30 m <= 2*eps = 0.3414 m

# W4's floor. See the docstring: this is what makes `n_e · n > 0` mean what it
# means in exact arithmetic when cos(90°) comes back as 6.1e-17.
FACING_MIN = 1e-9

# A cell centre exactly on a band's `t` boundary counts as inside. For
# axis-aligned lines no centre ever lands there (centres sit at whole cells from
# the endpoint, the boundary at a half), so this only settles the measure-zero
# oblique case, deterministically.
T_SLACK = 1e-9

# The narrowest physical gap that is a gap, as a fraction of a cell.
#
# Two scales meet here, three orders of magnitude apart. A line fitted 0.21° off
# the axis shortens every step to rho*cos(0.21°) and leaves 0.4 mm of "gap"
# between neighbouring face cells that are in fact touching; recording those
# would hand the later opening spec thousands of gaps that are nothing but the
# projection of a tilt. The smallest gap that is REAL is half a cell: a one-cell
# hole in a wall exposes the riser on its near side, that riser satisfies W4
# against the same slightly tilted line, and the resulting gap in the FACE runs
# from the riser to the next face cell -- 0.05 m where the hole is 0.10 m.
#
# A quarter of a cell sits between the two with a factor of 60 below it and a
# factor of 2 above. It is not a tuned number: no fraction of a cell can be a
# missing cell, and the half-cell case is arithmetic, not data.
MIN_GAP_FACTOR = 0.25

# ── Part B (§5, D4) ──────────────────────────────────────────────────────────
# 72 bearing bins of 5° is D4, settled. 9 incidence bins of 10° is §5. Both are
# stored per segment; the divergence spec reads the bearings and weights
# corroboration by their spread, which is the requirement §0.1 quotes -- never by
# a count of observers, since the robots share a frame, an odometry model and a
# rangefinder model and their errors are correlated.
BEARING_BINS = 72
INCIDENCE_BINS = 9

# Pre-registered in §6, reported and never gated.
B1_MIN_SHARE = 0.90            # kept returns within one cell of occupied
B2_MAX_SHARE = 0.01            # side contradictions, share of kept returns
B3_SCAN_TOL = 1                # scans read vs the replay's integrated count

# ── A8-A10, pre-registered in §6 ─────────────────────────────────────────────
TRUTH_TOL_CELLS = 2.0          # "within 2*rho of an SDF face"
A8_MIN_SHARE = 0.95            # pre-registered
A10_CLEAR_CELLS = 2.0          # robot_radius + 2*rho

# "a matching normal" (A9). The truth faces are axis-aligned, so requiring the
# face to be the NEAREST axis direction to the segment's normal is what matching
# means here; cos(45°) is the boundary between two axis directions, not a fitted
# number. The looser reading (same hemisphere, dot > 0) is reported beside it so
# the choice is visible rather than argued.
A9_MATCH_DOT = math.cos(math.radians(45.0))

# The pre-registered denominators of §1 and A9, recomputed from the SDF on every
# run and refused if they have moved: they are what "128.80 m" in the spec means.
SDF_FACES_TOTAL = 32
SDF_LENGTH_TOTAL_M = 212.00
SDF_LONG_FACES = 16
SDF_LONG_LENGTH_M = 209.60
SDF_INWARD_FACES = 12
SDF_INWARD_LENGTH_M = 128.80
SDF_LENGTH_TOL_M = 1e-6

# (drow, dcol, nx, ny, name). The order fixes the element order, which fixes
# every tie-break downstream, which is what A7 pins.
DIRS = (
    (0, 1, 1.0, 0.0, '+x'),
    (0, -1, -1.0, 0.0, '-x'),
    (1, 0, 0.0, 1.0, '+y'),
    (-1, 0, 0.0, -1.0, '-y'),
)

STATE_KEY = {OCC: 'O', FREE: 'F', UNKNOWN: 'U'}


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'graph_walls: {msg}', file=sys.stderr)
    sys.exit(2)


# ────────────────────────────── parameters ───────────────────────────────────

class Params:
    """Everything the extractor can be asked to do differently.

    The point of gathering them here is §6: every named break is a value in this
    object, so a test demonstrates a check failing by running the SAME extractor
    with a broken parameter. A break that needed the code edited would prove
    nothing about the code that ships.

        A1  eps_factor = 0.4            A5  endpoint_precedence swapped
        A2  dtheta_deg = 90.0           A6  enforce_facing = False
        A3  g doubled                   A7  tie_break_seed set
        A4  l_min = 0.1

    NOT a dataclass, deliberately. Every test in ros2_ws/src/nsk_swarm/test/
    loads the scripts in this directory BY LOCATION (importlib, no entry in
    sys.modules -- test_robot_divergence.py:47 is the pattern). Under
    `from __future__ import annotations` a dataclass's field annotations are
    strings, and dataclasses resolves them through
    `sys.modules[cls.__module__].__dict__`, which for a module loaded that way
    is None: `@dataclass` raises AttributeError at IMPORT time. That is a
    COLLECTION error, and one of those takes the whole nsk_swarm suite down
    rather than just this file (conftest.py says so about torch). Nine explicit
    keyword arguments cost less than that.
    """

    def __init__(self, g: float = 0.22, l_min: float = L_MIN,
                 eps_factor: float = EPS_FACTOR, b_factor: float = B_FACTOR,
                 dtheta_deg: float = DTHETA_DEG,
                 window_bins: int = WINDOW_BINS,
                 enforce_facing: bool = True,
                 endpoint_precedence: tuple = ('occupied', 'unknown'),
                 tie_break_seed: int | None = None):
        if g <= 0 or l_min <= 0:
            raise ValueError('g and l_min must be positive')
        if not 0.0 < dtheta_deg <= 360.0:
            raise ValueError('dtheta_deg must be in (0, 360]')
        if window_bins < 1 or window_bins % 2 == 0:
            raise ValueError('window_bins must be a positive odd number')
        if sorted(endpoint_precedence) != ['occupied', 'unknown']:
            raise ValueError("endpoint_precedence must permute "
                             "('occupied', 'unknown')")
        self.g = float(g)
        self.l_min = float(l_min)
        self.eps_factor = float(eps_factor)
        self.b_factor = float(b_factor)
        self.dtheta_deg = float(dtheta_deg)
        self.window_bins = int(window_bins)
        self.enforce_facing = bool(enforce_facing)
        self.endpoint_precedence = tuple(endpoint_precedence)
        self.tie_break_seed = tie_break_seed

    def __repr__(self) -> str:
        return ('Params(' + ', '.join(
            f'{k}={v!r}' for k, v in sorted(self.__dict__.items())) + ')')

    def eps(self, rho: float) -> float:
        return self.eps_factor * rho

    def b(self, rho: float) -> float:
        return self.b_factor * rho


def read_robot_radius(nav_dir: Path = NAV_DIR) -> tuple[float, str]:
    """(robot_radius, 'file:line') from the five nav2 param files.

    Read, never assumed: g = 2*robot_radius is D3, and the number that ends up in
    the JSON has to name the line it came from. All five robots carry it twice
    (local and global costmap); every occurrence must agree, because a g that
    differed per robot would make the maps' segments incomparable.
    """
    pat = re.compile(r'^\s*robot_radius:\s*([0-9.eE+-]+)\s*$')
    seen: dict[float, list[str]] = {}
    for k in range(NUM_ROBOTS):
        path = nav_dir / f'nav2_robot{k}.yaml'
        if not path.is_file():
            die(f'nav2 params not found, so g = 2*robot_radius cannot be read: '
                f'{path}')
        for i, line in enumerate(path.read_text().splitlines(), 1):
            m = pat.match(line.split('#', 1)[0])
            if m:
                seen.setdefault(float(m.group(1)), []).append(f'{path.name}:{i}')
    if not seen:
        die(f'no robot_radius in any of {nav_dir}/nav2_robot*.yaml')
    if len(seen) > 1:
        detail = '; '.join(
            f'{v}: {", ".join(w)}' for v, w in sorted(seen.items()))
        die(f'robot_radius disagrees across the nav2 params ({detail}). '
            'g = 2*robot_radius must be one number for all five robots.')
    value = next(iter(seen))
    return value, seen[value][0]


# ──────────────────────────── grid and elements ──────────────────────────────

def load_grid(stem: str, maps_dir: Path = MAPS_DIR_DEFAULT) -> dict:
    """One map as a bottom-up three-state grid in its own map frame.

    Convention A (PGM row 0 is maximum y) is applied by the imported
    trinary_map.sample() on the map's OWN lattice, where it is the identity
    resample -- and then checked against the plain row flip. Two ways of saying
    the same thing, verified per map: if they ever disagree the convention has
    moved under one of them and nothing downstream means anything.

    sha256 is of the PGM bytes. Deliberately not load()'s `md5_px`, which is
    Python's `hash()` of the pixels and therefore randomised by PYTHONHASHSEED --
    it would make A7's byte-identical JSON fail between two runs of the same map.
    """
    maps_dir = Path(maps_dir)
    pgm = maps_dir / f'{stem}.pgm'
    meta_path = maps_dir / f'{stem}.yaml'
    for p in (pgm, meta_path):
        if not p.is_file():
            die(f'map input not found: {p}')

    m = load(pgm)
    meta = yaml.safe_load(meta_path.read_text())
    negate = int(meta['negate'])
    occupied_thresh = float(meta['occupied_thresh'])
    free_thresh = float(meta['free_thresh'])
    thresholds = check_thresholds(negate, occupied_thresh, free_thresh,
                                  m['other'], meta_path.name)

    states = classify(m['px'], negate, occupied_thresh, free_thresh)
    rho = float(m['resolution'])
    ox, oy = float(m['origin'][0]), float(m['origin'][1])
    grid = sample({**m, 'px': states}, rho, ox, oy, m['width'], m['height'])
    if not np.array_equal(grid, states[::-1, :]):
        die(f'{stem}: resampling the map onto its own lattice did not reproduce '
            'the row flip, so convention A is not what this file thinks it is')

    return {
        'stem': stem,
        'pgm': pgm,
        'yaml': meta_path,
        'grid': grid,
        'rho': rho,
        'origin': (ox, oy),
        'width': int(m['width']),
        'height': int(m['height']),
        'sha256': hashlib.sha256(pgm.read_bytes()).hexdigest(),
        'negate': negate,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
        'thresholds': thresholds,
        'raw_counts': {'occupied': m['occupied'], 'free': m['free'],
                       'unknown': m['unknown'], 'other': m['other']},
    }


def shifted_view(mask: np.ndarray, drow: int, dcol: int) -> np.ndarray:
    """`out[r, c] = mask[r + drow, c + dcol]`, False where that is off the grid.

    Slices, so a shift moves cells OFF the array rather than around it -- the
    same reason robot_divergence.dilate:398 uses them.
    """
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys_dst = slice(max(0, -drow), h + min(0, -drow))
    ys_src = slice(max(0, drow), h + min(0, drow))
    xs_dst = slice(max(0, -dcol), w + min(0, -dcol))
    xs_src = slice(max(0, dcol), w + min(0, dcol))
    out[ys_dst, xs_dst] = mask[ys_src, xs_src]
    return out


def face_elements(grid: np.ndarray, rho: float,
                  origin: tuple[float, float]) -> dict:
    """Def 1. Every oriented occupied-free cell boundary of one grid.

    Four passes, one per direction, in the order of DIRS, each taking its cells
    in row-major order -- so the element list is a function of the grid alone.

    An occupied-UNKNOWN boundary yields nothing: that side was never seen, and an
    unobserved side is not a face. This is the single most consequential line in
    the file, because taking unknown for free is what once reported all 20 of
    these maps as fully known (trinary_map.py's docstring).
    """
    occ = grid == OCC
    free = grid == FREE
    xs, ys, nxs, nys, rows, cols, dirs = [], [], [], [], [], [], []
    for k, (drow, dcol, nx, ny, _name) in enumerate(DIRS):
        mask = occ & shifted_view(free, drow, dcol)
        r, c = np.nonzero(mask)
        xs.append(origin[0] + (c + 0.5 + 0.5 * dcol) * rho)
        ys.append(origin[1] + (r + 0.5 + 0.5 * drow) * rho)
        nxs.append(np.full(r.size, nx))
        nys.append(np.full(r.size, ny))
        rows.append(r)
        cols.append(c)
        dirs.append(np.full(r.size, k, dtype=np.int64))
    return {
        'n': int(sum(a.size for a in rows)),
        'x': np.concatenate(xs) if xs else np.empty(0),
        'y': np.concatenate(ys) if ys else np.empty(0),
        'nx': np.concatenate(nxs) if nxs else np.empty(0),
        'ny': np.concatenate(nys) if nys else np.empty(0),
        'row': np.concatenate(rows) if rows else np.empty(0, dtype=np.int64),
        'col': np.concatenate(cols) if cols else np.empty(0, dtype=np.int64),
        'dir': np.concatenate(dirs) if dirs else np.empty(0, dtype=np.int64),
    }


# ─────────────────────────────── line algebra ────────────────────────────────

def normal_of(theta_deg: float) -> tuple[float, float]:
    """`n = (-sin theta, cos theta)`, pointing to the free side (§2)."""
    rad = math.radians(theta_deg)
    return -math.sin(rad), math.cos(rad)


def dir_of(theta_deg: float) -> tuple[float, float]:
    """`u = (cos theta, sin theta)` (§2)."""
    rad = math.radians(theta_deg)
    return math.cos(rad), math.sin(rad)


def tls_fit(x: np.ndarray, y: np.ndarray,
            n_hint: tuple[float, float]) -> tuple[float, float, bool]:
    """(theta_deg, d, degenerate): total least squares through a point cloud.

    The normal is the eigenvector of the scatter matrix belonging to the SMALLER
    eigenvalue -- the direction in which the cloud is thinnest, i.e. the one that
    minimises perpendicular distance. Its sign is taken from `n_hint` so the fit
    keeps the orientation W4 needs; `theta` follows from
    `n = (-sin theta, cos theta)`, hence `theta = atan2(ny, nx) - 90°`.

    `degenerate` says the cloud has no direction: one point, coincident points,
    or an isotropic blob. The caller then keeps the peak's own theta rather than
    reporting an angle read off round-off.
    """
    if x.size == 0:
        return 0.0, 0.0, True
    cx, cy = float(x.mean()), float(y.mean())
    dx, dy = x - cx, y - cy
    sxx = float((dx * dx).sum())
    syy = float((dy * dy).sum())
    sxy = float((dx * dy).sum())
    tr = sxx + syy
    root = math.sqrt(max(tr * tr / 4.0 - (sxx * syy - sxy * sxy), 0.0))
    lam_min, lam_max = tr / 2.0 - root, tr / 2.0 + root
    if lam_max <= 0.0 or (lam_max - lam_min) <= 1e-12 * max(lam_max, 1.0):
        return 0.0, 0.0, True

    v1 = (sxy, lam_min - sxx)
    v2 = (lam_min - syy, sxy)
    vx, vy = v1 if math.hypot(*v1) >= math.hypot(*v2) else v2
    nrm = math.hypot(vx, vy)
    if nrm == 0.0:
        return 0.0, 0.0, True
    nx, ny = vx / nrm, vy / nrm
    if nx * n_hint[0] + ny * n_hint[1] < 0.0:
        nx, ny = -nx, -ny
    theta = (math.degrees(math.atan2(ny, nx)) - 90.0) % 360.0
    return theta, cx * nx + cy * ny, False


def line_candidates(el: dict, alive: np.ndarray, theta: float, d: float,
                    eps: float, enforce_facing: bool) -> np.ndarray:
    """Unassigned elements within `eps` of `L(theta, d)` and satisfying W4."""
    nx, ny = normal_of(theta)
    ok = alive & (np.abs(el['x'] * nx + el['y'] * ny - d) <= eps)
    if enforce_facing:
        ok = ok & ((el['nx'] * nx + el['ny'] * ny) > FACING_MIN)
    return ok


def split_runs(t: np.ndarray, rho: float, g: float) -> list[tuple[int, int]]:
    """W2: index ranges of `t` (sorted) split wherever the PHYSICAL gap is >= g.

    The physical gap between two consecutive elements is
    `t[k+1] - t[k] - rho`, i.e. the distance between the far edge of one face
    cell and the near edge of the next -- C4. Measuring `t[k+1] - t[k]` instead
    would bridge gaps one whole cell wider than g, which is what v1 of the spec
    did.
    """
    cuts = [0]
    for k in range(t.size - 1):
        if (t[k + 1] - t[k] - rho) >= g:
            cuts.append(k + 1)
    cuts.append(int(t.size))
    return [(cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]


# ──────────────────────────── the Hough accumulator ──────────────────────────

class Hough:
    """§4 step 2's oriented accumulator over live (unassigned) votes.

    ORIENTED is the whole of it: `theta` covers the full 360° and each element
    votes only where `n(theta) · n_e > 0`, so one slab's two faces vote in
    disjoint halves of the accumulator and can never merge into one peak. The
    same rule is W4, applied at vote time.

    `A[theta, dbin]` holds only votes not yet assigned to a segment; `remove()`
    takes a segment's elements out. `peaks()` then yields exactly the sequence
    "highest unexhausted peak, ties by smallest theta then smallest d" that §4
    step 3 asks for.
    """

    def __init__(self, el: dict, rho: float, params: Params):
        self.rho = rho
        self.half = params.window_bins // 2
        self.thetas = np.arange(0.0, 360.0, params.dtheta_deg)
        rad = np.radians(self.thetas)
        self.tnx, self.tny = -np.sin(rad), np.cos(rad)
        self.tux, self.tuy = np.cos(rad), np.sin(rad)
        n_theta = self.thetas.size

        if el['n'] == 0:
            self.d = np.zeros((0, n_theta))
            self.bins = np.zeros((0, n_theta), dtype=np.int64)
            self.vote = np.zeros((0, n_theta), dtype=bool)
            self.d0, self.n_d = 0.0, 1
            self.A = np.zeros((n_theta, 1), dtype=np.int64)
            self.exhausted = np.zeros((n_theta, 1), dtype=bool)
            return

        self.d = np.outer(el['x'], self.tnx) + np.outer(el['y'], self.tny)
        if params.enforce_facing:
            dot = np.outer(el['nx'], self.tnx) + np.outer(el['ny'], self.tny)
            self.vote = dot > FACING_MIN
        else:
            self.vote = np.ones((el['n'], n_theta), dtype=bool)

        self.d0 = float(self.d.min()) - rho
        self.bins = np.floor((self.d - self.d0) / rho).astype(np.int64)
        self.n_d = int(self.bins.max()) + 2
        self.A = np.zeros((n_theta, self.n_d), dtype=np.int64)
        for j in range(n_theta):
            sel = self.vote[:, j]
            if sel.any():
                self.A[j] = np.bincount(self.bins[sel, j], minlength=self.n_d)
        self.exhausted = np.zeros((n_theta, self.n_d), dtype=bool)

    def window(self, j: int, b: int) -> int:
        """Live votes in the `window_bins`-wide `d` window centred on bin `b`."""
        lo = max(0, b - self.half)
        hi = min(self.n_d, b + self.half + 1)
        return int(self.A[j, lo:hi].sum())

    def window_all(self) -> np.ndarray:
        """The same window sum for every (theta, d) at once."""
        out = self.A.copy()
        for s in range(1, self.half + 1):
            out[:, s:] += self.A[:, :-s]
            out[:, :-s] += self.A[:, s:]
        return out

    def remove(self, idx: np.ndarray) -> None:
        """Take one segment's elements out of the accumulator (§4 step 5)."""
        if idx.size == 0:
            return
        cols = np.arange(self.thetas.size)
        for e in np.asarray(idx).tolist():
            sel = self.vote[e]
            self.A[cols[sel], self.bins[e][sel]] -= 1

    def peaks(self, vote_min: float, tie_break_seed: int | None = None):
        """Yield (theta index, d bin, votes), highest first, each bin once.

        A lazy max-heap, which is an implementation of §4 step 3 rather than a
        change to it. Votes only ever decrease, so when a popped entry's stored
        key still matches its live window sum, every other entry's live value is
        at or below it: that entry IS the current maximum. A stale entry is
        re-pushed at its live value, and one whose live value has fallen below
        `vote_min` is dropped for good, since it cannot rise again.

        `tie_break_seed` is A7's break. With it set, equal peaks are ordered at
        random instead of by (theta, d), and two runs over one map stop agreeing.
        """
        rng = (np.random.default_rng(tie_break_seed)
               if tie_break_seed is not None else None)
        window = self.window_all()
        js, bs = np.nonzero(window >= vote_min)
        heap = [(-int(window[j, b]),
                 float(rng.random()) if rng is not None else 0.0,
                 int(j), int(b))
                for j, b in zip(js.tolist(), bs.tolist())]
        heapq.heapify(heap)
        while heap:
            stored, tie, j, b = heapq.heappop(heap)
            if self.exhausted[j, b]:
                continue
            live = self.window(j, b)
            if live < vote_min:
                continue
            if live != -stored:
                heapq.heappush(heap, (-live, tie, j, b))
                continue
            self.exhausted[j, b] = True
            yield j, b, live


# ──────────────────────────────── bands ──────────────────────────────────────

def band(grid: np.ndarray, rho: float, origin: tuple[float, float],
         theta: float, d: float, t_lo: float, t_hi: float, b: float) -> dict:
    """Def 3. The cells touching `L(theta, d)` with `t` in `[t_lo, t_hi]`.

    Cells off the grid are included and read as UNKNOWN, exactly as
    trinary_map.sample() fills outside a map. See the module docstring: clipping
    them away instead would let Def 5 call the edge of the map `free`.

    Returns the states, the SIGNED distance to the line (negative on the
    occupied side, since `n` points to the free side), and the cell indices so a
    caller can ask which cells were its own.
    """
    nx, ny = normal_of(theta)
    ux, uy = dir_of(theta)
    corners = [(t * ux + (d + s * b) * nx, t * uy + (d + s * b) * ny)
               for t in (t_lo, t_hi) for s in (-1.0, 1.0)]
    cxs = [p[0] for p in corners]
    cys = [p[1] for p in corners]
    c0 = int(math.floor((min(cxs) - origin[0]) / rho)) - 1
    c1 = int(math.ceil((max(cxs) - origin[0]) / rho)) + 1
    r0 = int(math.floor((min(cys) - origin[1]) / rho)) - 1
    r1 = int(math.ceil((max(cys) - origin[1]) / rho)) + 1

    rows = np.arange(r0, r1)
    cols = np.arange(c0, c1)
    xs = origin[0] + (cols + 0.5) * rho
    ys = origin[1] + (rows + 0.5) * rho
    gx, gy = np.meshgrid(xs, ys)
    proj = gx * nx + gy * ny - d
    tt = gx * ux + gy * uy
    # Perpendicular: the CENTRE within b, which is what C5's narrow band means
    # and what keeps the wall's own back row out.
    #
    # Along t: the centre within the interval, OR the interval entirely inside
    # this cell's own projected extent. The second clause is for a gap narrower
    # than one cell, which the riser case really produces (see MIN_GAP_FACTOR):
    # a 0.05 m gap between two cell centres 0.15 m apart contains no centre at
    # all, and a centre-only rule would leave Def 4 with an empty band and
    # nothing to report about a gap that is plainly inside one known cell. A
    # cell's extent along u is rho*(|ux| + |uy|), so for any interval as wide as
    # Def 5's g this clause can never fire.
    half_t = (abs(ux) + abs(uy)) * rho / 2.0
    keep = (np.abs(proj) <= b) & (
        ((tt >= t_lo - T_SLACK) & (tt <= t_hi + T_SLACK))
        | ((tt - half_t <= t_lo + T_SLACK) & (tt + half_t >= t_hi - T_SLACK)))

    rr, cc = np.meshgrid(rows, cols, indexing='ij')
    h, w = grid.shape
    states = np.full(rr.shape, UNKNOWN, dtype=grid.dtype)
    inside = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
    if inside.any():
        states[inside] = grid[rr[inside], cc[inside]]
    return {'states': states[keep], 'proj': proj[keep],
            'row': rr[keep], 'col': cc[keep], 'inside': inside[keep]}


def side_counts(states: np.ndarray) -> dict:
    return {'F': int(np.count_nonzero(states == FREE)),
            'O': int(np.count_nonzero(states == OCC)),
            'U': int(np.count_nonzero(states == UNKNOWN))}


def gap_record(grid: np.ndarray, rho: float, origin: tuple[float, float],
               theta: float, d: float, t_lo: float, t_hi: float,
               b: float) -> dict:
    """Def 4. One physical gap's state, split by side of the line.

    `free` only if every band cell is free; `unknown` if any is unknown; `mixed`
    otherwise. The two sides are counted separately because the later opening
    spec needs them: free on the far side of a gap in a wall is a doorway, and
    free only on the near side is not.
    """
    bd = band(grid, rho, origin, theta, d, t_lo, t_hi, b)
    occ_side = bd['proj'] < 0.0     # n points to the free side, so < 0 is behind
    states = bd['states']
    if states.size and np.all(states == FREE):
        state = 'free'
    elif np.any(states == UNKNOWN) or states.size == 0:
        state = 'unknown'
    else:
        state = 'mixed'
    return {'t0': float(t_lo), 't1': float(t_hi),
            'width': float(t_hi - t_lo), 'state': state,
            'occ_side': side_counts(states[occ_side]),
            'free_side': side_counts(states[~occ_side])}


def endpoint_state(grid: np.ndarray, rho: float, origin: tuple[float, float],
                   theta: float, d: float, t_lo: float, t_hi: float, b: float,
                   own: np.ndarray, precedence: tuple[str, ...]) -> str:
    """Def 5. What the band of length `g` beyond one endpoint contains.

    `occupied` needs an occupied cell that gives no element to this segment --
    the segment's own last cell is already excluded by the band starting at the
    endpoint, but a corner cell that feeds two segments is not, and it is the
    case this distinction exists for.

    The precedence is a parameter so A5's break (swap 1 and 2) can be run.
    Occupied beats unknown because a wall running into other occupied mass is a
    corner candidate whatever else is behind it; unknown beats free because the
    robot having stopped looking is not the wall having ended.
    """
    bd = band(grid, rho, origin, theta, d, t_lo, t_hi, b)
    states = bd['states']
    if states.size:
        mine = np.zeros(states.shape, dtype=bool)
        ok = bd['inside']
        if ok.any():
            mine[ok] = own[bd['row'][ok], bd['col'][ok]]
        has_occ = bool(np.any((states == OCC) & ~mine))
        has_unknown = bool(np.any(states == UNKNOWN))
    else:
        has_occ, has_unknown = False, True
    for label in precedence:
        if label == 'occupied' and has_occ:
            return 'occupied'
        if label == 'unknown' and has_unknown:
            return 'unknown'
    return 'free'


# ───────────────────────── unclassified occupied mass ────────────────────────

def components_8(mask: np.ndarray, rho: float,
                 origin: tuple[float, float]) -> list[dict]:
    """Def 6. 8-connected components of a boolean grid, with their extents.

    An iterative flood fill rather than scipy.ndimage.label, for the reason
    robot_divergence.dilate:391 gives: CI's container has no scipy and a
    module-level import of it is a collection error that takes the whole
    nsk_swarm suite down.
    """
    seen = np.zeros(mask.shape, dtype=bool)
    out = []
    h, w = mask.shape
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        queue = deque([(int(r0), int(c0))])
        seen[r0, c0] = True
        cells = []
        while queue:
            r, c = queue.popleft()
            cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] \
                            and not seen[rr, cc]:
                        seen[rr, cc] = True
                        queue.append((rr, cc))
        rs = [c[0] for c in cells]
        cs = [c[1] for c in cells]
        x0 = origin[0] + min(cs) * rho
        x1 = origin[0] + (max(cs) + 1) * rho
        y0 = origin[1] + min(rs) * rho
        y1 = origin[1] + (max(rs) + 1) * rho
        out.append({'n_cells': len(cells),
                    'bbox': [float(x0), float(y0), float(x1), float(y1)],
                    'extent': [float(x1 - x0), float(y1 - y0)]})
    return out


# ─────────────────────────────── extraction ──────────────────────────────────

def extract(grid: np.ndarray, rho: float, origin: tuple[float, float],
            params: Params) -> dict:
    """§4, steps 1-7. Grid in, wall segments out. No truth, no bag, no files.

    This is the function A1-A7 test. Everything above it is geometry on one
    array; everything below it is provenance and validation.
    """
    return extract_with_elements(grid, rho, origin, params)[0]


def extract_with_elements(grid: np.ndarray, rho: float,
                          origin: tuple[float, float],
                          params: Params) -> tuple[dict, dict, np.ndarray]:
    """extract(), plus the element table and `seg_of` behind it.

    A8 and A10 are statements about ELEMENTS, and the per-map JSON holds
    segments, so the validation needs the table. It gets it here rather than
    reconstructing membership from the reported lines afterwards: a
    reconstruction by distance would quietly include unassigned elements that
    happen to lie within eps of some segment's line, which is precisely the
    population A8 is measuring.
    """
    eps = params.eps(rho)
    b = params.b(rho)
    vote_min = params.l_min / (math.sqrt(2.0) * rho)

    el = face_elements(grid, rho, origin)
    hough = Hough(el, rho, params)
    alive = np.ones(el['n'], dtype=bool)
    seg_of = np.full(el['n'], -1, dtype=np.int64)

    segments: list[dict] = []
    lines: list[dict] = []
    n_iter = 0
    degenerate_fits = 0

    for j, bin_idx, votes in hough.peaks(vote_min, params.tie_break_seed):
        n_iter += 1
        theta_peak = float(hough.thetas[j])

        # step 3 -> the peak's own line: mean d of the live votes in the window.
        in_window = (alive & hough.vote[:, j]
                     & (np.abs(hough.bins[:, j] - bin_idx) <= hough.half))
        if not in_window.any():
            continue
        d_peak = float(hough.d[in_window, j].mean())

        # step 4 -> candidates, TLS refit, candidates once more.
        cand = line_candidates(el, alive, theta_peak, d_peak, eps,
                               params.enforce_facing)
        if not cand.any():
            continue
        theta_line, d_line, degen = _refit(el, cand, theta_peak, d_peak)
        degenerate_fits += int(degen)
        cand = line_candidates(el, alive, theta_line, d_line, eps,
                               params.enforce_facing)
        if not cand.any():
            continue

        # step 5 -> runs, then the long ones become segments.
        ux, uy = dir_of(theta_line)
        idx = np.nonzero(cand)[0]
        t_all = el['x'] * ux + el['y'] * uy
        idx = idx[np.argsort(t_all[idx], kind='stable')]
        t_sorted = t_all[idx]
        runs = split_runs(t_sorted, rho, params.g)

        accepted: list[tuple[int, int, np.ndarray]] = []
        for lo, hi in runs:
            members = idx[lo:hi]
            length = float(t_sorted[hi - 1] - t_sorted[lo] + rho)
            if length >= params.l_min:
                accepted.append((lo, hi, members))

        if not accepted:
            continue

        run_seg_id: dict[int, int] = {}
        taken = []
        for lo, hi, members in accepted:
            seg_id = len(segments)
            run_seg_id[lo] = seg_id
            segments.append(_segment(grid, rho, origin, el, members, seg_id,
                                     theta_line, d_line, b, params))
            seg_of[members] = seg_id
            alive[members] = False
            taken.append(members)

        # Splitting gaps live on the line, per Def 4, and carry which segments
        # they separate: null on a side whose run was below l_min. The opening
        # spec reads these and cannot recover the pairing from t0/t1 alone.
        gaps = []
        for k in range(len(runs) - 1):
            lo_a, hi_a = runs[k]
            lo_b, _hi_b = runs[k + 1]
            t_end = float(t_sorted[hi_a - 1]) + rho / 2.0
            t_start = float(t_sorted[lo_b]) - rho / 2.0
            rec = gap_record(grid, rho, origin, theta_line, d_line,
                             t_end, t_start, b)
            rec['between'] = [run_seg_id.get(lo_a), run_seg_id.get(lo_b)]
            gaps.append(rec)

        hough.remove(np.concatenate(taken))
        lines.append({
            'theta': float(theta_line),
            'd': float(d_line),
            'segments': [run_seg_id[lo] for lo, _hi, _m in accepted],
            'gaps': gaps,
            'peak': {'theta': theta_peak, 'votes': int(votes),
                     'degenerate_fit': bool(degen)},
        })

    graph = _finish(grid, rho, origin, el, seg_of, segments, lines, params,
                    {'n_elements': el['n'],
                     'n_assigned': int(np.count_nonzero(seg_of >= 0)),
                     'n_lines': len(lines),
                     'n_iterations': n_iter,
                     'degenerate_fits': degenerate_fits,
                     'vote_min': float(vote_min),
                     'theta_bins': int(hough.thetas.size),
                     'd_bins': int(hough.n_d)})
    return graph, el, seg_of


def _refit(el: dict, cand: np.ndarray, theta_peak: float,
           d_peak: float) -> tuple[float, float, bool]:
    """Step 4's TLS refit, falling back to the peak line when it is degenerate."""
    hx = float(el['nx'][cand].mean())
    hy = float(el['ny'][cand].mean())
    if math.hypot(hx, hy) < FACING_MIN:
        # W4 dropped (A6's break) can leave opposing normals whose mean is zero;
        # the peak's own normal is then the only orientation on offer.
        hx, hy = normal_of(theta_peak)
    theta, d, degen = tls_fit(el['x'][cand], el['y'][cand], (hx, hy))
    if degen:
        return theta_peak, d_peak, True
    return theta, d, False


def _geometry(el: dict, members: np.ndarray, theta: float, d: float,
              rho: float) -> dict:
    """One candidate line for a run, with W1/W2/W4 measured against it."""
    nx, ny = normal_of(theta)
    ux, uy = dir_of(theta)
    t = np.sort(el['x'][members] * ux + el['y'][members] * uy, kind='stable')
    delta = np.abs(el['x'][members] * nx + el['y'][members] * ny - d)
    facing = el['nx'][members] * nx + el['ny'][members] * ny
    return {
        'theta': float(theta), 'd': float(d),
        'nx': float(nx), 'ny': float(ny), 'ux': float(ux), 'uy': float(uy),
        't': t,
        't0': float(t[0]) - rho / 2.0,
        't1': float(t[-1]) + rho / 2.0,
        'max_delta': float(delta.max()),
        'max_gap': float(max((t[k + 1] - t[k] - rho
                              for k in range(t.size - 1)), default=0.0)),
        'min_facing': float(facing.min()),
        'n_not_facing': int(np.count_nonzero(facing <= FACING_MIN)),
    }


def _segment(grid: np.ndarray, rho: float, origin: tuple[float, float],
             el: dict, members: np.ndarray, seg_id: int, theta_line: float,
             d_line: float, b: float, params: Params) -> dict:
    """One segment: its reported line, endpoints, bridged gaps, endpoint states.

    WHICH LINE IS REPORTED, and why it is not always the refit (§11 P6).

    Def 2 says the reported line of S is the TLS fit to S's OWN elements, and
    that is not the line the run was cut on: step 4 fitted that one to every
    candidate of the peak, including the elements of the other runs. So S is
    refitted here -- and the refit is usually better (a 5.15 m wall cut on a
    peak 2.18° off the axis refits to 0.48°).

    But a SHORT, THICK run has no direction to find. Candidates may sit anywhere
    within eps = 1.71 cells of the line, so four elements spanning 0.55 m form a
    blob whose thinnest axis is not the wall: on b2maps_k0_cut1200_robot0 one
    such run was cut on a line at 146.1° and refitted to 179.1°, 33° away, and
    against THAT line its elements were 1.30*eps off it with a 1.13*g gap
    between two of them. W1 and W2 are what make S a wall segment, so a segment
    that fails them under its own reported line cannot be the output.

    The refit is therefore kept only when W1, W2 and W4 still hold under it;
    otherwise the reported line is the one the run was cut on, which satisfies
    all three BY CONSTRUCTION -- step 4 selected the candidates within eps of it
    and facing it, and step 5 split the runs on its own gaps. `line_source` says
    which, per segment, and the counts are in `checks`.
    """
    hx = float(el['nx'][members].mean())
    hy = float(el['ny'][members].mean())
    if math.hypot(hx, hy) < FACING_MIN:
        hx, hy = normal_of(theta_line)
    theta_fit, d_fit, degen = tls_fit(el['x'][members], el['y'][members],
                                      (hx, hy))

    cut = _geometry(el, members, theta_line, d_line, rho)
    if degen:
        geom, source = cut, 'extraction_line_degenerate_fit'
    else:
        own = _geometry(el, members, theta_fit, d_fit, rho)
        keeps_w4 = (not params.enforce_facing) or own['min_facing'] > FACING_MIN
        if (own['max_delta'] <= params.eps(rho) and own['max_gap'] < params.g
                and keeps_w4):
            geom, source = own, 'own_tls'
        else:
            geom, source = cut, 'extraction_line'

    theta, d = geom['theta'], geom['d']
    nx, ny, ux, uy = geom['nx'], geom['ny'], geom['ux'], geom['uy']
    t, t0, t1 = geom['t'], geom['t0'], geom['t1']
    p0 = (t0 * ux + d * nx, t0 * uy + d * ny)
    p1 = (t1 * ux + d * nx, t1 * uy + d * ny)

    bridged = []
    for k in range(t.size - 1):
        lo = float(t[k]) + rho / 2.0
        hi = float(t[k + 1]) - rho / 2.0
        if hi - lo >= MIN_GAP_FACTOR * rho:
            bridged.append(gap_record(grid, rho, origin, theta, d, lo, hi, b))

    own_cells = np.zeros(grid.shape, dtype=bool)
    own_cells[el['row'][members], el['col'][members]] = True
    end0 = endpoint_state(grid, rho, origin, theta, d, t0 - params.g, t0, b,
                          own_cells, params.endpoint_precedence)
    end1 = endpoint_state(grid, rho, origin, theta, d, t1, t1 + params.g, b,
                          own_cells, params.endpoint_precedence)

    return {
        'id': seg_id,
        'p0': [float(p0[0]), float(p0[1])],
        'p1': [float(p1[0]), float(p1[1])],
        'theta': float(theta),
        'normal': [float(nx), float(ny)],
        'length': float(t1 - t0),
        'n_elements': int(members.size),
        'bridged_gaps': bridged,
        'end_state': [end0, end1],
        'line_source': source,
        'obs': None,
        '_members': members,
        '_max_delta': geom['max_delta'],
        '_max_gap': geom['max_gap'],
        # W4 is the spec's `dot > 0`, so a riser at a corner whose normal is at
        # right angles to the wall is admitted whenever the line tilts a
        # fraction of a degree off the axis (dot +0.004). Counted, not hidden;
        # see the module docstring on why W4 stays `> 0` rather than a cone.
        '_min_facing': geom['min_facing'],
        '_n_not_facing': geom['n_not_facing'],
    }


def _finish(grid: np.ndarray, rho: float, origin: tuple[float, float],
            el: dict, seg_of: np.ndarray, segments: list[dict],
            lines: list[dict], params: Params, extraction: dict) -> dict:
    """Def 6, the truth-free self-checks, and the private fields stripped off."""
    eps = params.eps(rho)
    occ = grid == OCC
    contributes = np.zeros(grid.shape, dtype=bool)
    assigned = seg_of >= 0
    if assigned.any():
        contributes[el['row'][assigned], el['col'][assigned]] = True
    unclassified = occ & ~contributes

    n_occ = int(np.count_nonzero(occ))
    n_in_seg = int(np.count_nonzero(contributes))
    n_unclassified = int(np.count_nonzero(unclassified))

    gave_element = np.zeros(grid.shape, dtype=bool)
    if el['n']:
        gave_element[el['row'], el['col']] = True

    checks = {
        'elements_total': el['n'],
        'elements_assigned': int(np.count_nonzero(assigned)),
        'elements_unassigned': int(np.count_nonzero(~assigned)),
        'occupied_cells': n_occ,
        'occupied_in_a_segment': n_in_seg,
        'occupied_accounting_holds': bool(n_in_seg + n_unclassified == n_occ),
        'unclassified_giving_no_element': int(np.count_nonzero(
            unclassified & ~gave_element)),
        'w1_max_delta_over_eps': float(
            max((s['_max_delta'] for s in segments), default=0.0) / eps),
        'w2_max_gap_over_g': float(
            max((s['_max_gap'] for s in segments), default=0.0) / params.g),
        'w3_min_length_over_l_min': float(
            min((s['length'] for s in segments), default=float('nan'))
            / params.l_min) if segments else None,
        'w4_min_facing_dot': float(
            min((s['_min_facing'] for s in segments), default=float('nan')))
        if segments else None,
        # Elements that satisfied W4 against the line their run was cut on but
        # not against the line their segment ended up reporting, after the
        # per-segment TLS refit turned it. Reported so the size of the effect is
        # visible in every output rather than argued about.
        'w4_not_facing_after_refit': sum(s['_n_not_facing'] for s in segments),
        'w5_elements_in_two_segments': 0,   # set below
        'line_source_counts': {
            src: sum(1 for s in segments if s['line_source'] == src)
            for src in ('own_tls', 'extraction_line',
                        'extraction_line_degenerate_fit')},
    }

    counted = np.zeros(el['n'], dtype=np.int64)
    for s in segments:
        counted[s['_members']] += 1
    checks['w5_elements_in_two_segments'] = int(np.count_nonzero(counted > 1))
    for s in segments:
        for key in ('_members', '_max_delta', '_max_gap', '_min_facing',
                    '_n_not_facing'):
            s.pop(key)

    return {
        'lines': lines,
        'segments': segments,
        'unclassified': {
            'n_cells': n_unclassified,
            'n_cells_giving_no_element': int(np.count_nonzero(
                unclassified & ~gave_element)),
            'n_cells_with_unassigned_elements': int(np.count_nonzero(
                unclassified & gave_element)),
            'components': components_8(unclassified, rho, origin),
        },
        'extraction': extraction,
        'checks': checks,
    }


# ──────────────────────────── one map, end to end ────────────────────────────

def graph_for_map(stem: str, maps_dir: Path = MAPS_DIR_DEFAULT,
                  params: Params | None = None,
                  g_source: str | None = None) -> dict:
    """§7's JSON for one map. Writes nothing."""
    return graph_and_elements(stem, maps_dir, params, g_source)[0]


def graph_and_elements(stem: str, maps_dir: Path = MAPS_DIR_DEFAULT,
                       params: Params | None = None,
                       g_source: str | None = None
                       ) -> tuple[dict, dict, np.ndarray, np.ndarray]:
    """§7's JSON for one map, with the element table A8/A10 need.

    Also hands back the grid, which Part B's B1 needs and which would otherwise
    be read from the PGM a second time.

    `params` defaults to g read from the nav2 params, which is the only threshold
    here that comes from outside the spec and the map.
    """
    if params is None:
        radius, where = read_robot_radius()
        params = Params(g=2.0 * radius)
        g_source = f'{where} robot_radius {radius} x 2'
    m = load_grid(stem, maps_dir)
    rho = m['rho']
    result, el, seg_of = extract_with_elements(
        m['grid'], rho, m['origin'], params)
    robot = re.search(r'robot(\d+)$', stem)
    graph = {
        'map': stem,
        'map_sha256': m['sha256'],
        'frame': f'robot_{robot.group(1)} map' if robot else f'{stem} map',
        'resolution': rho,
        'origin': [m['origin'][0], m['origin'][1]],
        'size_cells': [m['width'], m['height']],
        'thresholds': {
            'rho': {'value': rho, 'unit': 'm',
                    'source': f'{m["yaml"].name}, read'},
            'eps': {'value': params.eps(rho), 'unit': 'm',
                    'source': f'pre-registered: rho*(1 + sqrt(2)/2), factor '
                              f'{params.eps_factor!r}'},
            'b': {'value': params.b(rho), 'unit': 'm',
                  'source': f'pre-registered: rho*sqrt(2)/2, factor '
                            f'{params.b_factor!r}'},
            'g': {'value': params.g, 'unit': 'm',
                  'source': g_source or 'caller-supplied'},
            'l_min': {'value': params.l_min, 'unit': 'm',
                      'source': 'pre-registered (D2)'},
            'dtheta': {'value': params.dtheta_deg, 'unit': 'deg',
                       'source': 'pre-registered (§8)'},
            'window_bins': {'value': params.window_bins, 'unit': 'd bins',
                            'source': f'chosen: {params.window_bins}*rho = '
                                      f'{params.window_bins * rho:.4f} m <= '
                                      f'2*eps = {2 * params.eps(rho):.4f} m '
                                      f'(§4 step 2, §11 P1)'},
        },
        'raw_counts': m['raw_counts'],
        'threshold_check': m['thresholds'],
        **result,
    }
    return graph, el, seg_of, m['grid']


def summary_line(graph: dict) -> str:
    """§7's one-line summary: segments, length, unclassified, endpoint states."""
    ends = {'free': 0, 'unknown': 0, 'occupied': 0}
    for s in graph['segments']:
        for e in s['end_state']:
            ends[e] += 1
    total = sum(s['length'] for s in graph['segments'])
    unc = graph['unclassified']
    return (f'{graph["map"]:<30} segs {len(graph["segments"]):>4}  '
            f'length {total:>8.2f} m  unclassified {unc["n_cells"]:>5} cells / '
            f'{len(unc["components"]):>3} comp  ends '
            f'F{ends["free"]:>4} U{ends["unknown"]:>4} O{ends["occupied"]:>4}  '
            f'iters {graph["extraction"]["n_iterations"]:>6}')


# ─────────────────── Part B: viewing directions (§5) ─────────────────────────
# Still no truth: a scan, a pose chain and the segments already extracted. What
# this adds is the one thing §0.1 says the schema cannot do without -- the set of
# DIRECTIONS a face was seen from, because the divergence measure may not weight
# corroboration by a count of observers who share a frame, an odometry model and
# a rangefinder model.

def read_replay_r_max(k: int, cut: int) -> tuple[float, str]:
    """`max_laser_range` from the cut's own replay config, with its line.

    Read, not assumed. This is the number the replay gave Karto, so it is the
    boundary between a return that could have marked a cell occupied and one
    that could only ever have cleared free space. All 20 configs carry 7.9.
    """
    path = (LOGS_DIR / f'offline_mapping_{RUN_PREFIX}_k{k}_cut{cut}'
            f'_robot_{k}.yaml')
    if not path.is_file():
        die(f'replay config not found, so r_max cannot be read: {path}')
    for i, line in enumerate(path.read_text().splitlines(), 1):
        m = re.match(r'^\s*max_laser_range:\s*([0-9.eE+-]+)\s*$',
                     line.split('#', 1)[0])
        if m:
            return float(m.group(1)), f'{path.name}:{i}'
    die(f'no max_laser_range in {path}')


def read_relayed_scan_count(k: int, cut: int) -> tuple[int, str]:
    """B3's reference: how many scans the replay actually delivered to SLAM.

    From free_space_relay's own summary line. The relay is what stood between
    the bag and Karto, so its count is the integrated count -- and the raw
    /robot_K/scan this file reads is its INPUT, which is why the two can be
    compared at all.

    Read with errors='replace': these logs carry raw terminal bytes, which is
    the same reason the explore logs need `grep -a`.
    """
    path = LOGS_DIR / f'offline_slam_{RUN_PREFIX}_k{k}_cut{cut}_robot_{k}.log'
    if not path.is_file():
        die(f'offline slam log not found, so B3 has no reference: {path}')
    text = path.read_text(errors='replace')
    hits = re.findall(r'relayed (\d+) scans', text)
    if not hits:
        die(f'no "relayed N scans" line in {path.name}, so B3 cannot be '
            'evaluated')
    return int(hits[-1]), f'{path.name} ("relayed {hits[-1]} scans")'


def yaw_of(q) -> float:
    """Planar yaw of a quaternion. The convention tf_pair_at.py:25 uses."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag: Path, k: int) -> dict:
    """One replay bag's scans, its /tf odom->base samples, and /tf_static.

    ROS lives in here and nowhere above it: rosbag2_py and rclpy.serialization
    are imported inside this function, as bag_overlap.py:17 does, so every other
    function in this file -- and the whole test suite -- imports without ROS on
    the path.

    THE SCAN TOPIC IS THE RAW ONE. The replay fed Karto /robot_K/scan_free, the
    free_space_relay's output, which is not in the bag: the relay ran live during
    the replay. That costs nothing here, because the relay rewrites only +inf,
    NaN-adjacent cases and readings AT OR ABOVE range_max, filling them with
    0.5*(max_laser_range + range_max) = 7.95 m so Karto traces them as free
    (free_space_relay.py:71-86). Every finite reading below r_max -- exactly the
    set §5 keeps -- passes through untouched. So the raw topic carries precisely
    the returns that could have marked a cell occupied.

    POSES COME FROM /tf ONLY (C10). /robot_K/odom is in the same bag and drifts
    8.7 m over a run; it is not what the replay used and it is not read here.
    """
    import rosbag2_py                                       # noqa: PLC0415
    from rclpy.serialization import deserialize_message      # noqa: PLC0415
    from sensor_msgs.msg import LaserScan                    # noqa: PLC0415
    from tf2_msgs.msg import TFMessage                       # noqa: PLC0415

    if not bag.is_dir():
        die(f'replay bag not found: {bag}')
    scan_topic = f'/robot_{k}/scan'
    parent, child = f'robot_{k}/odom', f'robot_{k}/base_footprint'

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=['/tf', '/tf_static', scan_topic]))

    stamps, ranges, geom = [], [], None
    tf_rows, static, max_tilt = [], {}, 0.0
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == scan_topic:
            msg = deserialize_message(data, LaserScan)
            stamps.append(msg.header.stamp.sec
                          + msg.header.stamp.nanosec * 1e-9)
            ranges.append(np.asarray(msg.ranges, dtype=np.float64))
            if geom is None:
                geom = {'frame': msg.header.frame_id,
                        'n_beams': len(msg.ranges),
                        'angle_min': float(msg.angle_min),
                        'angle_increment': float(msg.angle_increment),
                        'range_min': float(msg.range_min),
                        'range_max': float(msg.range_max)}
        elif topic == '/tf':
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id != parent or tr.child_frame_id != child:
                    continue
                q = tr.transform.rotation
                max_tilt = max(max_tilt, abs(q.x), abs(q.y))
                tf_rows.append((tr.header.stamp.sec
                                + tr.header.stamp.nanosec * 1e-9,
                                tr.transform.translation.x,
                                tr.transform.translation.y, yaw_of(q)))
        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                q = tr.transform.rotation
                static[(tr.header.frame_id, tr.child_frame_id)] = (
                    tr.transform.translation.x, tr.transform.translation.y,
                    yaw_of(q), abs(q.x), abs(q.y))

    if geom is None:
        die(f'{bag.name}: no {scan_topic} messages')
    if not tf_rows:
        die(f'{bag.name}: no {parent} -> {child} in /tf. §5 takes poses from '
            '/tf only, and /robot_K/odom is not a substitute for it')
    if len({r.size for r in ranges}) != 1:
        die(f'{bag.name}: the scans do not all have the same beam count')

    tf = np.array(sorted(tf_rows), dtype=np.float64)
    return {'bag': bag, 'geom': geom,
            'stamps': np.array(stamps, dtype=np.float64),
            'ranges': np.vstack(ranges),
            'tf': tf, 'tf_max_tilt': float(max_tilt), 'static': static}


def compose_base_scan(static: dict, k: int) -> tuple[tuple, list]:
    """base_footprint -> base_scan, composed from the two-hop /tf_static chain.

    C7: composed, not assumed. Both hops are pure translations with zero yaw on
    these bags, so the composition is an addition -- but it is done as a real
    2-D composition and each hop's roll/pitch is checked, because a hop with a
    yaw would make the addition wrong rather than approximate.

    The x offset is -0.032 m. That is the whole of B4's break: omit this and
    every hit point lands 3.2 cm from where the sensor actually saw it.
    """
    chain = [(f'robot_{k}/base_footprint', f'robot_{k}/base_link'),
             (f'robot_{k}/base_link', f'robot_{k}/base_scan')]
    x, y, yaw, hops = 0.0, 0.0, 0.0, []
    for hop in chain:
        if hop not in static:
            die(f'/tf_static has no {hop[0]} -> {hop[1]}, so base_T_scan '
                'cannot be composed. §5 requires the two-hop chain')
        hx, hy, hyaw, qx, qy = static[hop]
        if max(qx, qy) > 1e-6:
            die(f'{hop[0]} -> {hop[1]} is not a planar rotation '
                f'(|qx|={qx}, |qy|={qy}); this file composes 2-D transforms')
        x, y = x + math.cos(yaw) * hx - math.sin(yaw) * hy, \
            y + math.sin(yaw) * hx + math.cos(yaw) * hy
        yaw += hyaw
        hops.append({'parent': hop[0], 'child': hop[1],
                     'x': hx, 'y': hy, 'yaw': hyaw})
    return (x, y, yaw), hops


def interpolate_poses(tf: np.ndarray, stamps: np.ndarray) -> tuple:
    """odom_T_base at each scan stamp: linear in position, shortest arc in yaw.

    A stamp outside the transform span is NOT extrapolated -- `ok` is False for
    it and §5 says the scan is dropped and counted. The span is closed at both
    ends: the last scan of a cut lands exactly on the last /tf sample.

    Yaw by shortest arc, never by interpolating the wrapped value: the same rule
    SPEC_b2_steady_gate §2 states for its own resampling, and the reason is the
    same -- a robot crossing +/-180 deg would otherwise appear to spin a whole
    turn between two samples.
    """
    ts = tf[:, 0]
    ok = (stamps >= ts[0] - 1e-9) & (stamps <= ts[-1] + 1e-9)
    idx = np.clip(np.searchsorted(ts, stamps, side='right') - 1,
                  0, ts.size - 2)
    t0, t1 = ts[idx], ts[idx + 1]
    span = np.where(t1 > t0, t1 - t0, 1.0)
    frac = np.clip(np.where(t1 > t0, (stamps - t0) / span, 0.0), 0.0, 1.0)
    x = tf[idx, 1] + frac * (tf[idx + 1, 1] - tf[idx, 1])
    y = tf[idx, 2] + frac * (tf[idx + 1, 2] - tf[idx, 2])
    y0, y1 = tf[idx, 3], tf[idx + 1, 3]
    dyaw = (y1 - y0 + math.pi) % (2.0 * math.pi) - math.pi
    return x, y, y0 + frac * dyaw, ok


def scan_returns(bagdata: dict, poses: tuple, base_scan: tuple,
                 map_t_odom: tuple, r_max: float) -> dict:
    """Every kept return as a hit point, a sensor origin and a bearing.

    Kept means `range_min <= r < r_max` and finite. A return at or above r_max,
    or +inf, or NaN, never marked a cell occupied -- the relay filled exactly
    those so Karto would trace them as FREE -- so it is not an observation of a
    wall and must not become one here.

    `T_map_scan = map_T_odom . odom_T_base(t) . base_T_scan`, applied in that
    order. map_T_odom is identity on these maps and checked to be; it is still
    composed rather than dropped, so this reads as the chain §5 states.
    """
    x, y, yaw, ok = poses
    g = bagdata['geom']
    mx, my, myaw = map_t_odom
    bx, by, byaw = base_scan

    # sensor pose in the map frame, per scan
    sx = mx + np.cos(myaw) * (x + np.cos(yaw) * bx - np.sin(yaw) * by) \
        - np.sin(myaw) * (y + np.sin(yaw) * bx + np.cos(yaw) * by)
    sy = my + np.sin(myaw) * (x + np.cos(yaw) * bx - np.sin(yaw) * by) \
        + np.cos(myaw) * (y + np.sin(yaw) * bx + np.cos(yaw) * by)
    syaw = myaw + yaw + byaw

    r = bagdata['ranges']
    beam = g['angle_min'] + np.arange(g['n_beams']) * g['angle_increment']
    keep = np.isfinite(r) & (r >= g['range_min']) & (r < r_max) & ok[:, None]
    n_total = int(r.size)
    n_out_of_range = int(np.count_nonzero(
        np.isfinite(r) & (r >= r_max) & ok[:, None]))
    n_infinite = int(np.count_nonzero(~np.isfinite(r) & ok[:, None]))

    si, bi = np.nonzero(keep)
    rr = r[si, bi]
    ang = syaw[si] + beam[bi]
    ox, oy = sx[si], sy[si]
    px, py = ox + rr * np.cos(ang), oy + rr * np.sin(ang)
    bearing = np.degrees(np.arctan2(py - oy, px - ox)) % 360.0
    return {'px': px, 'py': py, 'ox': ox, 'oy': oy, 'bearing': bearing,
            'scan': si, 'stamp': bagdata['stamps'][si], 'range': rr,
            'n_beams_total': n_total, 'n_kept': int(rr.size),
            'n_at_or_over_r_max': n_out_of_range, 'n_infinite': n_infinite,
            'sensor_x': sx, 'sensor_y': sy, 'sensor_yaw': syaw}


def attribute(returns: dict, segments: list, rho: float, eps: float,
              flip_normals: bool = False) -> dict:
    """Which segment each return belongs to, and the side contradictions.

    §5: within `eps + rho/2` of the line, `t` inside the endpoints padded by
    `rho`, and the ray arriving from the FREE side. Ties to the nearer line,
    then the lower id -- which is what iterating ids in order with a strict
    `<` on the distance gives.

    A return that satisfies the first two conditions for some segment and the
    third for none is a SIDE CONTRADICTION: the map says this face is seen from
    one side and the ray came from the other. B2 counts them, and `flip_normals`
    is its break.
    """
    n = returns['px'].size
    best = np.full(n, -1, dtype=np.int64)
    best_delta = np.full(n, np.inf)
    # The nearest line a return matched on distance and extent, whatever side it
    # came from. Only the contradictions use it, and only to say WHICH face they
    # contradict -- by §5's own tie rule, nearer line then lower id, so it adds
    # no threshold of its own.
    near = np.full(n, -1, dtype=np.int64)
    near_delta = np.full(n, np.inf)
    incidence = np.full(n, np.nan)
    geom_any = np.zeros(n, dtype=bool)
    px, py, ox, oy = (returns['px'], returns['py'],
                      returns['ox'], returns['oy'])
    for s in segments:
        nx, ny = s['normal']
        if flip_normals:
            nx, ny = -nx, -ny
        ux, uy = dir_of(s['theta'])
        d = s['p0'][0] * nx + s['p0'][1] * ny
        ta = s['p0'][0] * ux + s['p0'][1] * uy
        tb = s['p1'][0] * ux + s['p1'][1] * uy
        t_lo, t_hi = min(ta, tb) - rho, max(ta, tb) + rho
        delta = np.abs(px * nx + py * ny - d)
        tt = px * ux + py * uy
        geom = (delta <= eps + rho / 2.0) & (tt >= t_lo) & (tt <= t_hi)
        geom_any |= geom
        closer = geom & (delta < near_delta)
        near[closer] = s['id']
        near_delta[closer] = delta[closer]
        vx, vy = ox - px, oy - py
        from_free = (nx * vx + ny * vy) > 0.0
        take = geom & from_free & (delta < best_delta)
        if take.any():
            best[take] = s['id']
            best_delta[take] = delta[take]
            norm = np.hypot(vx[take], vy[take])
            cos_psi = np.clip(np.where(norm > 0.0,
                                       (nx * vx[take] + ny * vy[take])
                                       / np.where(norm > 0.0, norm, 1.0), 1.0),
                              -1.0, 1.0)
            incidence[take] = np.degrees(np.arccos(cos_psi))
    return {'seg': best, 'incidence': incidence, 'near': near,
            'contradiction': geom_any & (best < 0)}


def observations(returns: dict, attrib: dict, segments: list) -> list:
    """§5's per-segment record, in segment-id order.

    `n_scans` is the OBSERVATION COUNT the schema asks for, and `n_hits` is
    never it: 360 beams of one scan from one pose are one observation of a wall
    seen 360 ways, not 360 observations. `bearing_hist` is what §0.1 requires --
    the directions, not their number -- and the divergence spec weights
    corroboration from it.
    """
    out = []
    for s in segments:
        pick = attrib['seg'] == s['id']
        # Contradictions are not attributed to a segment -- they are returns no
        # segment accepted -- but recording which face each one is nearest to
        # says a great deal: they pile up on faces whose opposite side the map
        # never recorded, so the robot saw the wall from the side that has no
        # face to belong to.
        contra = int(np.count_nonzero(attrib['contradiction']
                                      & (attrib['near'] == s['id'])))
        n_hits = int(np.count_nonzero(pick))
        if not n_hits:
            out.append({'n_scans': 0, 'n_hits': 0,
                        'bearing_hist': [0] * BEARING_BINS,
                        'incidence_hist': [0] * INCIDENCE_BINS,
                        't_first': None, 't_last': None,
                        'n_contradictions': contra})
            continue
        bearing = returns['bearing'][pick]
        psi = attrib['incidence'][pick]
        stamp = returns['stamp'][pick]
        b_idx = np.clip((bearing / (360.0 / BEARING_BINS)).astype(np.int64),
                        0, BEARING_BINS - 1)
        i_idx = np.clip((psi / (90.0 / INCIDENCE_BINS)).astype(np.int64),
                        0, INCIDENCE_BINS - 1)
        out.append({
            'n_scans': int(np.unique(returns['scan'][pick]).size),
            'n_hits': n_hits,
            'bearing_hist': np.bincount(
                b_idx, minlength=BEARING_BINS).tolist(),
            'incidence_hist': np.bincount(
                i_idx, minlength=INCIDENCE_BINS).tolist(),
            't_first': float(stamp.min()), 't_last': float(stamp.max()),
            'n_contradictions': contra,
        })
    return out


def on_occupied_share(returns: dict, grid: np.ndarray, rho: float,
                      origin: tuple[float, float],
                      convention_b: bool = False) -> dict:
    """B1: the share of kept returns landing within one cell of occupied.

    The dilation is imported from robot_divergence rather than rewritten: a
    second implementation of "within one cell" would make this number
    incomparable with the tolerant conflict that file reports.

    `convention_b` is B1's named break. Convention A is PGM row 0 = maximum y,
    which load_grid() applies and verifies; B is the same grid read without the
    row flip. Under B the returns keep their place and the walls move, so a
    correct pose chain scores near nothing.
    """
    from robot_divergence import dilate              # noqa: PLC0415

    occ = (grid[::-1, :] if convention_b else grid) == OCC
    near = dilate(occ, 1)
    h, w = grid.shape
    col = np.floor((returns['px'] - origin[0]) / rho + 1e-9).astype(np.int64)
    row = np.floor((returns['py'] - origin[1]) / rho + 1e-9).astype(np.int64)
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    hit = np.zeros(returns['px'].size, dtype=bool)
    hit[inside] = near[row[inside], col[inside]]
    n = int(hit.size)
    return {'n_returns': n, 'n_on_occupied': int(np.count_nonzero(hit)),
            'n_outside_map': int(np.count_nonzero(~inside)),
            'share': (float(np.count_nonzero(hit)) / n) if n else float('nan'),
            'convention': 'B (BROKEN)' if convention_b else 'A',
            'hit': hit}


def off_wall_sample(returns: dict, hit: np.ndarray,
                    limit: int = 4000) -> np.ndarray:
    """A small, deterministic sample of the returns B1 counted as off-wall.

    Kept so the validation pass can ask the one question B1 cannot: is a return
    that missed the MAP's walls near a real one? By stride, not by an RNG --
    nothing in this file may depend on a seed (A7).
    """
    idx = np.nonzero(~hit)[0]
    if idx.size == 0:
        return np.zeros((0, 2))
    idx = idx[::max(1, idx.size // limit)][:limit]
    return np.column_stack([returns['px'][idx], returns['py'][idx]])


def obs_for_map(graph: dict, stem: str, k: int, cut: int, grid: np.ndarray,
                params: Params, bags_dir: Path = BAGS_DIR,
                break_name: str | None = None) -> dict:
    """Part B for one map: fill every segment's `obs`, and B1-B3.

    B1-B3 are truth-free -- a map's own occupied cells, its own segments, and
    the replay's own scan count -- so they belong in `checks` beside Part A's,
    and NOT in the validation file that holds A8-A11.

    Returns a small sample of the returns B1 counted as off-wall, which the
    validation pass compares against the world. Nothing truth-derived comes back
    into the graph.
    """
    if '_gated' in stem:
        die(f'{stem}: Part B builds its per-cut inputs -- the replay bag, the '
            'replay config, the relay log -- from (robot, cut) alone, so on a '
            '_gated map it would read the UNGATED ones. Threading the variant '
            'through those three builders is a separate change; until it is '
            'made, run Part B on the ungated maps and compare the gated ones '
            'with compare_gated_maps.py, which needs only trinary_map.')
    rho = graph['resolution']
    eps = params.eps(rho)
    r_max, r_max_src = read_replay_r_max(k, cut)
    relayed, relayed_src = read_relayed_scan_count(k, cut)
    bag = bags_dir / f'{RUN_PREFIX}_k{k}_cut{cut}_slamin'
    data = read_bag(bag, k)

    geom = data['geom']
    if geom['frame'] != f'robot_{k}/base_scan':
        die(f'{stem}: scans are in frame {geom["frame"]!r}, not '
            f'robot_{k}/base_scan, so base_T_scan does not describe them')

    base_scan, hops = compose_base_scan(data['static'], k)
    if break_name == 'no-base-scan':
        base_scan = (0.0, 0.0, 0.0)

    import robot_divergence as rd                    # noqa: PLC0415
    import fit_world_transform as fwt                # noqa: PLC0415
    fwt.RUN = f'{RUN_PREFIX}_k{k}_cut{cut}'
    mto, mto_path = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, stem)                  # identity, or it dies

    poses = interpolate_poses(data['tf'], data['stamps'])
    n_dropped = int(np.count_nonzero(~poses[3]))
    returns = scan_returns(data, poses, base_scan, mto, r_max)
    attrib = attribute(returns, graph['segments'], rho, eps,
                       flip_normals=(break_name == 'flip-normals'))
    obs = observations(returns, attrib, graph['segments'])
    for s, o in zip(graph['segments'], obs):
        s['obs'] = o

    b1 = on_occupied_share(returns, grid, rho, graph['origin'],
                           convention_b=(break_name == 'convention-b'))
    n_contra = int(np.count_nonzero(attrib['contradiction']))
    n_scans_read = int(data['stamps'].size)
    graph['checks'].update({
        'B1_on_occupied_share': b1['share'],
        'B1_limit': B1_MIN_SHARE,
        'B1_holds': bool(b1['share'] >= B1_MIN_SHARE),
        'B2_side_contradiction_share': (float(n_contra) / returns['n_kept']
                                        if returns['n_kept'] else float('nan')),
        'B2_limit': B2_MAX_SHARE,
        'B2_holds': bool(n_contra <= B2_MAX_SHARE * returns['n_kept']),
        'B3_scans_read': n_scans_read,
        'B3_scans_relayed': relayed,
        'B3_difference': n_scans_read - relayed,
        'B3_tolerance': B3_SCAN_TOL,
        'B3_holds': bool(abs(n_scans_read - relayed) <= B3_SCAN_TOL),
    })
    graph['observations'] = {
        'bag': str(bag.relative_to(REPO_ROOT)),
        'scan_topic': f'/robot_{k}/scan',
        'scan_topic_note': 'the RAW scan. The replay fed Karto the relay\'s '
                           'scan_free, which is not in the bag; the relay '
                           'rewrites only +inf and readings at or above '
                           'range_max, so every return kept here passed '
                           'through it untouched',
        'r_max': {'value': r_max, 'unit': 'm', 'source': r_max_src},
        'scan_geometry': geom,
        'pose_source': f'/tf {f"robot_{k}/odom"} -> robot_{k}/base_footprint, '
                       'interpolated at the scan stamp (linear in position, '
                       'shortest arc in yaw). Never /robot_K/odom (C10)',
        'tf_samples': int(data['tf'].shape[0]),
        'tf_span_s': [float(data['tf'][0, 0]), float(data['tf'][-1, 0])],
        'tf_max_offplane_quat': data['tf_max_tilt'],
        'base_T_scan': {'x': base_scan[0], 'y': base_scan[1],
                        'yaw': base_scan[2], 'hops': hops},
        'map_T_odom': {'x': mto[0], 'y': mto[1], 'yaw': mto[2],
                       'source': str(Path(mto_path).relative_to(REPO_ROOT)),
                       'checked': 'identity within '
                                  f'{rd.MTO_IDENTITY_TOL}'},
        'n_scans_read': n_scans_read,
        'n_scans_dropped_no_pose': n_dropped,
        'n_beams': returns['n_beams_total'],
        'n_returns_kept': returns['n_kept'],
        'n_returns_at_or_over_r_max': returns['n_at_or_over_r_max'],
        'n_returns_infinite': returns['n_infinite'],
        'n_returns_attributed': int(np.count_nonzero(attrib['seg'] >= 0)),
        'n_side_contradictions': n_contra,
        'n_returns_outside_map': b1['n_outside_map'],
        'n_returns_off_wall': b1['n_returns'] - b1['n_on_occupied'],
        'broken_on_purpose': break_name,
    }
    return off_wall_sample(returns, b1['hit'])


def obs_summary_line(graph: dict) -> str:
    """One line per map: what Part B read and what B1-B3 came to."""
    o = graph['observations']
    c = graph['checks']
    attributed = (o['n_returns_attributed'] / o['n_returns_kept']
                  if o['n_returns_kept'] else float('nan'))
    seen = sum(1 for s in graph['segments'] if s['obs']['n_scans'] > 0)
    return (f'{graph["map"]:<30} scans {o["n_scans_read"]:>5}'
            f'({o["n_scans_dropped_no_pose"]:>2} dropped)  '
            f'kept {o["n_returns_kept"]:>8}  attributed {attributed:>6.1%}  '
            f'B1 {c["B1_on_occupied_share"]:>6.2%}  '
            f'B2 {c["B2_side_contradiction_share"]:>6.3%}  '
            f'B3 {c["B3_difference"]:>+2d}  '
            f'segs seen {seen:>3}/{len(graph["segments"]):<3}')


# ───────────────────── truth: A8-A11 validation only ─────────────────────────
# Everything from here to main() reads the world. §6: "A8-A10 are validation of
# the extractor only. They are the one place world truth enters this tool, and no
# later tool may read their outputs." Hence the separate output file.

def sdf_wall_faces(l_min: float) -> dict:
    """The world's faces, oriented, with the pre-registered lengths re-derived.

    32 faces of 212.00 m; 16 of them (the end caps, 0.1 m and 0.2 m) fall below
    l_min; of the 16 long ones the four OUTWARD faces of the boundary walls
    cannot be seen from inside the arena, leaving the 128.80 m that is A9's
    denominator (C6).

    The numbers are asserted, not assumed: if the SDF changes, "128.80 m" in the
    spec stops describing it and A9's pre-registered prediction is about a
    denominator that no longer exists.

    Also measured, and reported beside A9 rather than removed from its
    denominator (§11 P5): the length of inward face that is not an exposed
    surface of the union of the eight boxes, because it runs into a perpendicular
    slab at a corner or lies outside the arena.
    """
    from bag_overlap import world_walls_named        # ROS-free, function-scoped

    rects = world_walls_named()
    by_name = dict(rects)
    boundary = [n for n in by_name if n.startswith('wall_')]
    if len(boundary) != 4:
        die(f'expected four wall_* boundary models in the SDF, found '
            f'{sorted(boundary)}. A9\'s denominator is defined in terms of them.')

    faces = []
    for name, (x0, x1, y0, y1) in rects:
        for side, p0, p1, n, length in (
                ('south', (x0, y0), (x1, y0), (0.0, -1.0), x1 - x0),
                ('north', (x0, y1), (x1, y1), (0.0, 1.0), x1 - x0),
                ('west', (x0, y0), (x0, y1), (-1.0, 0.0), y1 - y0),
                ('east', (x1, y0), (x1, y1), (1.0, 0.0), y1 - y0)):
            faces.append({'model': name, 'side': side,
                          'p0': p0, 'p1': p1, 'n': n, 'length': float(length)})

    # The arena interior: inside all four boundary walls' inner faces.
    xw = by_name['wall_west'][1]
    xe = by_name['wall_east'][0]
    ys = by_name['wall_south'][3]
    yn = by_name['wall_north'][2]
    interior = (xw, xe, ys, yn)

    def inside_interior(x: float, y: float) -> bool:
        return xw < x < xe and ys < y < yn

    off = 1e-3        # a hair off the surface, on the free side
    long_faces = [f for f in faces if f['length'] >= l_min]
    inward = []
    for f in long_faces:
        mx = (f['p0'][0] + f['p1'][0]) / 2.0 + off * f['n'][0]
        my = (f['p0'][1] + f['p1'][1]) / 2.0 + off * f['n'][1]
        if inside_interior(mx, my):
            inward.append(f)

    totals = (len(faces), sum(f['length'] for f in faces),
              len(long_faces), sum(f['length'] for f in long_faces),
              len(inward), sum(f['length'] for f in inward))
    want = (SDF_FACES_TOTAL, SDF_LENGTH_TOTAL_M, SDF_LONG_FACES,
            SDF_LONG_LENGTH_M, SDF_INWARD_FACES, SDF_INWARD_LENGTH_M)
    for got, exp, what in zip(totals, want,
                              ('faces', 'total m', 'long faces', 'long m',
                               'inward faces', 'inward m')):
        if abs(got - exp) > SDF_LENGTH_TOL_M:
            die(f'the world SDF gives {what} = {got}, not the pre-registered '
                f'{exp}. §1 and A9 are stated in terms of those numbers, so '
                'they no longer describe this world.')
    return {'faces': faces, 'long_faces': long_faces, 'inward': inward,
            'interior': interior, 'rects': rects,
            'excluded_short': [f'{f["model"]}.{f["side"]} '
                               f'({f["length"]:.2f} m < {l_min} m)'
                               for f in faces if f['length'] < l_min],
            'excluded_outward': [f'{f["model"]}.{f["side"]}'
                                 for f in long_faces if f not in inward],
            'inward_length_m': float(sum(f['length'] for f in inward))}


def face_samples(world: dict, step: float) -> dict:
    """Every inward face sampled every `step` m, with a reachability flag.

    A sample is unreachable when the surface there is not exposed: pushed a hair
    onto its free side it lands inside another box, or outside the arena. That is
    the 8 x 0.15 m of inward boundary face buried in the perpendicular slabs at
    the four corners -- reported, never subtracted from A9's pre-registered
    denominator (§11 P5).
    """
    xw, xe, ys_, yn = world['interior']
    off = 1e-3
    xs, ys, nxs, nys, reach, owner, wts = [], [], [], [], [], [], []
    for fi, f in enumerate(world['inward']):
        n_steps = max(int(round(f['length'] / step)), 1)
        ts = (np.arange(n_steps) + 0.5) / n_steps
        px = f['p0'][0] + ts * (f['p1'][0] - f['p0'][0])
        py = f['p0'][1] + ts * (f['p1'][1] - f['p0'][1])
        # The weight is the metre of face each sample stands for, so recall is a
        # LENGTH share whatever the face length does to the step.
        wts.extend([f['length'] / n_steps] * n_steps)
        for x, y in zip(px.tolist(), py.tolist()):
            ox, oy = x + off * f['n'][0], y + off * f['n'][1]
            ok = xw < ox < xe and ys_ < oy < yn
            if ok:
                for name, (rx0, rx1, ry0, ry1) in world['rects']:
                    if name != f['model'] and rx0 < ox < rx1 and ry0 < oy < ry1:
                        ok = False
                        break
            xs.append(x)
            ys.append(y)
            nxs.append(f['n'][0])
            nys.append(f['n'][1])
            reach.append(ok)
            owner.append(fi)
    return {'x': np.array(xs), 'y': np.array(ys),
            'nx': np.array(nxs), 'ny': np.array(nys),
            'reachable': np.array(reach, dtype=bool),
            'face': np.array(owner, dtype=np.int64),
            'w': np.array(wts), 'step': step}


def _segment_arrays(graph: dict, spawn: tuple[float, float]) -> dict:
    """Segment endpoints and normals carried into the world frame.

    registered origin = spawn + YAML origin (robot_divergence.py:304) and this
    file's coordinates already carry the YAML origin, so the world placement of
    any point is + spawn. That the composition is the right one is a cited
    result, not one re-measured here.
    """
    n = len(graph['segments'])
    out = {k: np.zeros(n) for k in ('x0', 'y0', 'x1', 'y1', 'nx', 'ny')}
    for i, s in enumerate(graph['segments']):
        out['x0'][i] = s['p0'][0] + spawn[0]
        out['y0'][i] = s['p0'][1] + spawn[1]
        out['x1'][i] = s['p1'][0] + spawn[0]
        out['y1'][i] = s['p1'][1] + spawn[1]
        out['nx'][i] = s['normal'][0]
        out['ny'][i] = s['normal'][1]
    return out


def point_segment_distance(px: np.ndarray, py: np.ndarray, seg: dict,
                           i: int) -> np.ndarray:
    """Distance from points to segment `i`, clamped to its endpoints."""
    ax, ay = seg['x0'][i], seg['y0'][i]
    bx, by = seg['x1'][i], seg['y1'][i]
    vx, vy = bx - ax, by - ay
    vv = vx * vx + vy * vy
    if vv == 0.0:
        return np.hypot(px - ax, py - ay)
    t = np.clip(((px - ax) * vx + (py - ay) * vy) / vv, 0.0, 1.0)
    return np.hypot(px - (ax + t * vx), py - (ay + t * vy))


def validate_map(graph: dict, elements_world: dict, spawn_table: list,
                 robot: int, world: dict, samples: dict, rho: float,
                 radius: float, b1_off_sample: np.ndarray | None = None) -> dict:
    """A8-A11 for one map. Truth in, four reported numbers out.

    Plus, when Part B has run, the one question B1 cannot ask of itself: a
    return that missed every occupied cell of the MAP -- was there a real wall
    where it landed? If yes, B1's shortfall is the map's incompleteness and not
    a placement error, and the two readings must never be confused.
    """
    from bag_overlap import dist_to_nearest_wall     # ROS-free, function-scoped

    tol = TRUTH_TOL_CELLS * rho
    seg = _segment_arrays(graph, spawn_table[robot])

    # ── A8: element positions against the world's wall surfaces ──────────────
    a8 = {'tol_m': tol, 'n_assigned': 0, 'share_assigned': float('nan'),
          'n_all': int(elements_world['x'].size), 'share_all': float('nan')}
    if elements_world['x'].size:
        pts_all = np.column_stack([elements_world['x'], elements_world['y']])
        d_all = dist_to_nearest_wall(pts_all, [r for _n, r in world['rects']])
        a8['share_all'] = float(np.count_nonzero(d_all <= tol) / d_all.size)
        sel = elements_world['assigned']
        a8['n_assigned'] = int(np.count_nonzero(sel))
        if sel.any():
            a8['share_assigned'] = float(
                np.count_nonzero(d_all[sel] <= tol) / np.count_nonzero(sel))
            a8['median_offset_m'] = float(np.median(d_all[sel]))

    # ── A9: recall of the inward faces ───────────────────────────────────────
    n_s = len(graph['segments'])
    covered_strict = np.zeros(samples['x'].size, dtype=bool)
    covered_loose = np.zeros(samples['x'].size, dtype=bool)
    for i in range(n_s):
        dot = samples['nx'] * seg['nx'][i] + samples['ny'] * seg['ny'][i]
        near = point_segment_distance(samples['x'], samples['y'], seg, i) <= tol
        covered_strict |= near & (dot > A9_MATCH_DOT)
        covered_loose |= near & (dot > FACING_MIN)
    w = samples['w']
    reach = samples['reachable']
    total_m = float(w.sum())
    a9 = {
        'denominator_m': world['inward_length_m'],
        'sample_step_m': samples['step'],
        'match_dot_min': A9_MATCH_DOT,
        'covered_m': float(w[covered_strict].sum()),
        'recall': float(w[covered_strict].sum() / total_m),
        'recall_loose_match': float(w[covered_loose].sum() / total_m),
        'unreachable_m': float(w[~reach].sum()),
        'recall_of_reachable': float(w[covered_strict & reach].sum()
                                     / w[reach].sum()),
    }

    # ── A10: clearance from the four parked peers' spawns ────────────────────
    clear = A10_CLEAR_CELLS * rho + radius
    a10 = {'clearance_m': clear, 'holds': True, 'per_peer': {},
           'own_spawn_min_m': None}
    if elements_world['x'].size and elements_world['assigned'].any():
        sel = elements_world['assigned']
        ex = elements_world['x'][sel]
        ey = elements_world['y'][sel]
        for k, (sx, sy) in enumerate(spawn_table):
            dmin = float(np.min(np.hypot(ex - sx, ey - sy)))
            if k == robot:
                a10['own_spawn_min_m'] = dmin
                continue
            a10['per_peer'][f'robot_{k}'] = dmin
            if dmin < clear:
                a10['holds'] = False
        a10['min_m'] = min(a10['per_peer'].values()) if a10['per_peer'] else None

    # ── A11: angle residual to the nearest multiple of 90° ──────────────────
    # Split by length, because the split is the finding. Every residual above a
    # couple of degrees on this corpus belongs to a segment at the l_min floor:
    # four or five leftover elements at a corner, strung diagonally, spanning
    # just over 0.5 m with no gap wider than g. They satisfy W1-W5 and they are
    # what the pre-registered thresholds admit; a long wall never does this.
    res = [(abs(((s['theta'] + 45.0) % 90.0) - 45.0), s['length'],
            s['n_elements'], s['line_source']) for s in graph['segments']]
    long_res = [r for r, ln, _n, _s in res if ln >= 2.0]
    worst = max(res, default=None)
    a11 = {'n_segments': len(res),
           'max_deg': float(worst[0]) if res else None,
           'median_deg': float(np.median([r for r, *_ in res])) if res else None,
           'p95_deg': float(np.percentile([r for r, *_ in res], 95))
           if res else None,
           'max_deg_at_least_2m': float(max(long_res)) if long_res else None,
           'n_segments_at_least_2m': len(long_res),
           'n_segments_under_1m': sum(1 for _r, ln, *_ in res if ln < 1.0),
           'worst': {'residual_deg': float(worst[0]), 'length_m': worst[1],
                     'n_elements': worst[2], 'line_source': worst[3]}
           if res else None,
           'note': 'this world is axis aligned, so a large residual is a '
                   'finding about the extractor and a small one is not evidence '
                   'that off-axis walls would be found -- only A2 tests that. '
                   'Read max_deg_at_least_2m for the walls and max_deg for what '
                   'l_min admits at a corner'}

    out = {'A8': a8, 'A9': a9, 'A10': a10, 'A11': a11}
    if b1_off_sample is not None and b1_off_sample.size:
        pts = np.column_stack([b1_off_sample[:, 0] + spawn_table[robot][0],
                               b1_off_sample[:, 1] + spawn_table[robot][1]])
        d = dist_to_nearest_wall(pts, [r for _n, r in world['rects']])
        out['B1_off_wall_vs_truth'] = {
            'n_sampled': int(d.size),
            'share_within_2rho_of_a_real_wall': float(
                np.count_nonzero(d <= tol) / d.size),
            'median_distance_m': float(np.median(d)),
            'reading': 'a return that missed every occupied cell of the map but '
                       'is on a real wall says the MAP lacks the wall, not that '
                       'the pose chain is wrong. B1 cannot tell the two apart; '
                       'this can, and it is the only reason it is computed',
        }
    return out


def elements_in_world(el: dict, seg_of: np.ndarray,
                      spawn: tuple[float, float]) -> dict:
    """The element table carried into the world frame, with exact membership.

    `seg_of` comes straight out of the extraction, so `assigned` is the set of
    elements that really did become part of a segment -- not everything that
    happens to lie near one.
    """
    return {'x': el['x'] + spawn[0], 'y': el['y'] + spawn[1],
            'assigned': seg_of >= 0}


# ──────────────────────────────── main ───────────────────────────────────────

def corpus_stems() -> list[str]:
    return [f'{RUN_PREFIX}_k{k}_cut{c}_robot{k}'
            for k in range(NUM_ROBOTS) for c in CUTS]


def _k_and_cut(stem: str, why: str) -> tuple[int, int]:
    """(robot, cut) from a corpus map name, or a refusal naming what needs it.

    `_gated` is optional, so a travel-gated map parses to the same (K, cut) as the
    ungated one it is paired with -- which is the point: the comparison between
    them is between two maps of ONE cut.

    It gives the same (K, cut) and nothing more, so a caller that then builds a
    per-cut INPUT path from (K, cut) -- the bag, the replay config, the relay log
    -- would reach for the ungated artefact. obs_for_map refuses a gated stem for
    exactly that reason rather than reading the wrong bag quietly.
    """
    m = re.match(rf'{RUN_PREFIX}_k(\d+)_cut(\d+)(?:_gated)?_robot(\d+)$', stem)
    if not m:
        die(f'{stem}: {why}, and both are found from the b2maps naming.')
    if m.group(1) != m.group(3):
        die(f'{stem}: run k{m.group(1)} but map robot{m.group(3)}. These runs '
            'have one explorer each and its own map is the only one this reads.')
    return int(m.group(1)), int(m.group(2))


def print_b_predictions(rows: dict, break_name: str | None) -> bool:
    """B1-B3 across the corpus. Reported, never gated -- like A8-A11.

    Returns whether all three held, which only `--break` reads: a break that
    leaves every prediction standing has not demonstrated anything, and saying
    so is the point of running it.
    """
    b1 = [(s, r['graph']['checks']) for s, r in rows.items()]
    n1 = sum(1 for _s, c in b1 if c['B1_holds'])
    n2 = sum(1 for _s, c in b1 if c['B2_holds'])
    n3 = sum(1 for _s, c in b1 if c['B3_holds'])
    n = len(b1)
    print('PREDICTIONS -- pre-registered in §6, reported, NOT gated')
    print(f'  B1  kept returns within one cell of an occupied cell >= '
          f'{100 * B1_MIN_SHARE:.0f}%: {"HELD" if n1 == n else "FAILED"} '
          f'({n1}/{n})   worst '
          f'{min(c["B1_on_occupied_share"] for _s, c in b1):.2%}')
    for s, c in b1:
        if not c['B1_holds']:
            print(f'        {s}: {c["B1_on_occupied_share"]:.2%}')
    print(f'  B2  side contradictions <= {100 * B2_MAX_SHARE:.0f}% of kept '
          f'returns: {"HELD" if n2 == n else "FAILED"} ({n2}/{n})   worst '
          f'{max(c["B2_side_contradiction_share"] for _s, c in b1):.3%}')
    for s, c in b1:
        if not c['B2_holds']:
            print(f'        {s}: {c["B2_side_contradiction_share"]:.3%}')
    print(f'  B3  scans read equal the replay\'s integrated count within '
          f'{B3_SCAN_TOL}: {"HELD" if n3 == n else "FAILED"} ({n3}/{n})   '
          'differences '
          + ' '.join(f'{c["B3_difference"]:+d}' for _s, c in b1))
    for s, c in b1:
        if not c['B3_holds']:
            print(f'        {s}: read {c["B3_scans_read"]}, relayed '
                  f'{c["B3_scans_relayed"]}')
    worst_seg = None
    for s, r in rows.items():
        for seg in r['graph']['segments']:
            n = seg['obs'].get('n_contradictions', 0)
            if worst_seg is None or n > worst_seg[2]:
                worst_seg = (s, seg, n)
    if worst_seg and worst_seg[2]:
        s, seg, n = worst_seg
        print(f'  B2, where the contradictions sit: the worst single face is '
              f'{s} segment {seg["id"]},')
        print(f'        {seg["length"]:.2f} m long with {n} of them. A face '
              'collects contradictions when the')
        print('        robot saw that wall from the other side and the map has '
              'no face there to own the')
        print('        returns -- the same missing occupied cells B1 measures, '
              'read from the other end.')
        print('        Every segment carries its own count in '
              'obs.n_contradictions.')
    if break_name:
        print()
        print(f'  --break {break_name} was in effect. Expected to fail: '
              + {'convention-b': 'B1', 'flip-normals': 'B2',
                 'no-base-scan': 'B1 (every hit point 0.032 m off)'}[break_name])
        if n1 == n and n2 == n and n3 == n:
            print('  IT DID NOT. Every prediction still holds under the break, '
                  'which means the')
            print('  check is not measuring what it claims to measure. That is '
                  'a finding, not a pass.')
    print('=' * 100)
    print()
    return n1 == n and n2 == n and n3 == n


def print_validation(rows: dict, world: dict, samples: dict,
                     spawn_rev: str) -> dict:
    """The A8-A11 tables and the four predictions. Reported, never gated."""
    print('=' * 100)
    print('A8-A11 -- VALIDATION OF THIS EXTRACTOR, the one place world truth '
          'enters. §6 forbids')
    print('  any later tool from reading these, so they are written to '
          '_validation_*.json and')
    print('  never into a <map>.json. Predictions are reported, never gated.')
    print()
    print(f'  world: {len(world["faces"])} SDF faces / '
          f'{sum(f["length"] for f in world["faces"]):.2f} m; '
          f'{len(world["long_faces"])} of length >= l_min / '
          f'{sum(f["length"] for f in world["long_faces"]):.2f} m; '
          f'{len(world["inward"])} inward / {world["inward_length_m"]:.2f} m')
    print(f'  excluded, too short ({len(world["excluded_short"])}): '
          f'{", ".join(world["excluded_short"][:4])} ...')
    print(f'  excluded, outward   ({len(world["excluded_outward"])}): '
          f'{", ".join(world["excluded_outward"])}')
    unreachable = float(np.count_nonzero(~samples['reachable']) * samples['step'])
    print(f'  of the {world["inward_length_m"]:.2f} m denominator, '
          f'{unreachable:.2f} m is not an exposed surface of the union (each '
          'inward boundary')
    print('    face runs into the perpendicular slab at both its corners). '
          'Reported, NOT subtracted:')
    print('    A9\'s denominator was pre-registered before any recall data '
          '(C6, §11 P5).')
    print()
    print(f'{"map":<30}{"A8 on-wall":>12}{"A9 recall":>11}{"A9 reach":>10}'
          f'{"A10 min m":>11}{"A11 all °":>11}{"A11 >=2m °":>12}'
          f'{"segs":>6}{"<1m":>5}')
    for stem, r in rows.items():
        v = r['validation']
        print(f'{stem:<30}{100 * v["A8"]["share_assigned"]:>11.2f}%'
              f'{100 * v["A9"]["recall"]:>10.2f}%'
              f'{100 * v["A9"]["recall_of_reachable"]:>9.2f}%'
              f'{(v["A10"].get("min_m") or float("nan")):>11.3f}'
              f'{(v["A11"]["max_deg"] or float("nan")):>11.3f}'
              f'{(v["A11"]["max_deg_at_least_2m"] or float("nan")):>12.3f}'
              f'{v["A11"]["n_segments"]:>6}'
              f'{v["A11"]["n_segments_under_1m"]:>5}')
    print()

    a8_bad = [s for s, r in rows.items()
              if not r['validation']['A8']['share_assigned'] >= A8_MIN_SHARE]
    a10_bad = [s for s, r in rows.items() if not r['validation']['A10']['holds']]
    per_robot = {}
    for k in range(NUM_ROBOTS):
        series = []
        for c in CUTS:
            stem = f'{RUN_PREFIX}_k{k}_cut{c}_robot{k}'
            if stem in rows:
                series.append(rows[stem]['validation']['A9']['recall'])
        per_robot[k] = series
    rising = {k: all(b >= a for a, b in zip(v, v[1:])) for k, v in
              per_robot.items() if len(v) > 1}

    print('PREDICTIONS -- pre-registered in §6, reported, NOT gated')
    print(f'  A8  element positions within 2*rho of an SDF face >= '
          f'{100 * A8_MIN_SHARE:.0f}% on all {len(rows)} maps: '
          f'{"HELD" if not a8_bad else "FAILED"} '
          f'({len(rows) - len(a8_bad)}/{len(rows)})')
    for s in a8_bad:
        print(f'        {s}: '
              f'{100 * rows[s]["validation"]["A8"]["share_assigned"]:.2f}%')
    print('  A9  recall non-decreasing with cut for each robot: '
          f'{"HELD" if all(rising.values()) else "FAILED"} '
          f'({sum(rising.values())}/{len(rising)} robots)')
    for k, ok in rising.items():
        if not ok:
            print(f'        robot_{k}: '
                  + ' '.join(f'{100 * v:.2f}%' for v in per_robot[k]))
    print(f'  A10 no element within robot_radius + 2*rho of a parked peer\'s '
          f'spawn: {"HELD" if not a10_bad else "FAILED"} '
          f'({len(rows) - len(a10_bad)}/{len(rows)})')
    for s in a10_bad:
        print(f'        {s}: min {rows[s]["validation"]["A10"]["min_m"]:.3f} m')

    diag = [(s, r['validation']['B1_off_wall_vs_truth'],
             dict(r['graph']['checks'],
                  n_off=r['graph']['observations']['n_returns_off_wall'],
                  n_out=r['graph']['observations']['n_returns_outside_map']))
            for s, r in rows.items()
            if 'B1_off_wall_vs_truth' in r.get('validation', {})]
    failed = [(s, v, c) for s, v, c in diag if not c['B1_holds']]
    if diag:
        print('  B1 diagnosis -- truth, and the only reason it is computed. B1 '
              'counts a return as')
        print('        off-wall when no occupied cell of the MAP is within one '
              'cell of it. That happens')
        print('        either because the pose chain put the return in the wrong '
              'place or because the map')
        print('        has no wall where a wall really is, and B1 cannot tell '
              'those apart. This can:')
        if failed:
            for s, v, c in failed:
                print(f'        {s}: B1 {c["B1_on_occupied_share"]:.2%}, and '
                      f'{v["share_within_2rho_of_a_real_wall"]:.1%} of its '
                      'off-wall returns are')
                print(f'              within 2*rho of a REAL wall (median '
                      f'{v["median_distance_m"]:.3f} m from one).')
            print('        So every B1 shortfall above is the MAP missing '
                  'occupied cells where a wall is,')
            print('        not a misplaced return. Part A can only find faces '
                  'the map committed.')
        else:
            print('        every map holds B1, so there is nothing to diagnose.')
        keep = [(s, v, c) for s, v, c in diag if c['B1_holds']]
        if keep:
            worst = max(keep, key=lambda kv: kv[2]['n_off'])
            print(f'        On the {len(keep)} maps that HOLD B1 the off-wall '
                  f'set is at most {worst[2]["n_off"]} returns')
            print(f'        ({worst[0]}), of which {worst[2]["n_out"]} fall '
                  'outside the map extent altogether, so')
            print('        their truth share says nothing either way.')

    if rows:
        all_max = max(r['validation']['A11']['max_deg'] or 0.0
                      for r in rows.values())
        long_max = max(r['validation']['A11']['max_deg_at_least_2m'] or 0.0
                       for r in rows.values())
        short = sum(r['validation']['A11']['n_segments_under_1m']
                    for r in rows.values())
        segs = sum(r['validation']['A11']['n_segments'] for r in rows.values())
        print(f'  A11 reported only: worst angle residual to a multiple of 90° '
              f'is {all_max:.3f}° over all {segs} segments, but only '
              f'{long_max:.3f}° among those')
        print(f'        at least 2 m long. Every large residual is a segment at '
              f'the l_min floor: {short} of the {segs}')
        print('        segments are under 1 m, and four or five leftover '
              'elements strung diagonally across a')
        print('        corner satisfy W1-W5 as readily as a wall does. That is '
              'what l_min = 0.5 m admits, and')
        print('        it is a finding for the corner spec, not a failure here. '
              'What this corpus cannot')
        print('        test at all is a genuinely off-axis WALL: every wall in '
              'this world is axis aligned,')
        print('        so only A2 exercises that.')
    print('=' * 100)
    print()
    return {
        'spawn_rev': spawn_rev,
        'truth_note': 'A8-A11 validate the extractor only. §6: no later tool '
                      'may read these. They are deliberately not written into '
                      'any <map>.json.',
        'world': {
            'faces': len(world['faces']),
            'length_m': float(sum(f['length'] for f in world['faces'])),
            'long_faces': len(world['long_faces']),
            'long_length_m': float(sum(f['length']
                                       for f in world['long_faces'])),
            'inward_faces': len(world['inward']),
            'inward_length_m': world['inward_length_m'],
            'excluded_short': world['excluded_short'],
            'excluded_outward': world['excluded_outward'],
            'unreachable_m': unreachable,
            'unreachable_note': 'inward boundary face buried in the '
                                'perpendicular slab at a corner; reported, not '
                                'subtracted from the pre-registered denominator',
        },
        'predictions': {
            'A8_on_wall': {'limit': A8_MIN_SHARE, 'held': not a8_bad,
                           'failing_maps': a8_bad},
            'A9_recall_rises_with_cut': {'held': all(rising.values()),
                                         'per_robot': {f'robot_{k}': v
                                                       for k, v in
                                                       per_robot.items()}},
            'A10_parked_clear': {'held': not a10_bad, 'failing_maps': a10_bad},
            'A11_angle_residual': {'gated': False},
        },
        'per_map': {s: r['validation'] for s, r in rows.items()},
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps',
                    help='one map stem; repeatable. Default: the 20 b2maps cuts')
    ap.add_argument('--maps-dir', default=str(MAPS_DIR_DEFAULT),
                    help='default %(default)s (NSK_TRINARY_MAPS_DIR)')
    ap.add_argument('--out', default=str(OUT_DIR_DEFAULT),
                    help='default %(default)s')
    ap.add_argument('--spawn-rev', default=SPAWN_REV,
                    help='git rev whose DOT_POSES places the maps for A8-A11. '
                         'Default %(default)s, the b2maps era')
    ap.add_argument('--no-validate', action='store_true',
                    help='skip A8-A11: no git, no world SDF, no registration')
    ap.add_argument('--no-obs', action='store_true',
                    help='skip Part B: no bag is opened and every segment keeps '
                         '"obs": null. Part A alone needs no ROS at all')
    ap.add_argument('--break', dest='break_name',
                    choices=['convention-b', 'flip-normals', 'no-base-scan'],
                    help='corrupt one input so a red check is demonstrable '
                         'rather than asserted: convention-b breaks B1, '
                         'flip-normals breaks B2, no-base-scan drops the '
                         '0.032 m sensor offset. Writes nothing, exits 2')
    ap.add_argument('--no-json', action='store_true',
                    help='print the summaries and write nothing')
    args = ap.parse_args()

    maps_dir = Path(args.maps_dir)
    out_dir = Path(args.out)
    stems = args.maps or corpus_stems()

    radius, radius_where = read_robot_radius()
    params = Params(g=2.0 * radius)
    g_source = f'{radius_where} robot_radius {radius} x 2'

    print('wall faces out of each robot\'s own trinary grid -- Part A of '
          'SPEC_b2_wall_predicate')
    print(f'  g = 2*robot_radius = {params.g} m, read from {radius_where}')
    print(f'  l_min = {params.l_min} m, eps = {params.eps_factor:.6f}*rho, '
          f'b = {params.b_factor:.6f}*rho, dtheta = {params.dtheta_deg}°, '
          f'window = {params.window_bins} bins')
    print(f'  maps: {maps_dir}')
    print()

    if not maps_dir.is_dir():
        print(f'no corpus at {maps_dir}: nothing extracted, nothing validated. '
              'Set NSK_TRINARY_MAPS_DIR or pass --maps-dir.')
        return 0

    rows: dict[str, dict] = {}
    for stem in stems:
        graph, el, seg_of, grid = graph_and_elements(stem, maps_dir, params,
                                                     g_source)
        print(summary_line(graph))
        rows[stem] = {'graph': graph, 'el': el, 'seg_of': seg_of, 'grid': grid}
        c = graph['checks']
        if not c['occupied_accounting_holds'] or c['w5_elements_in_two_segments']:
            die(f'{stem}: self-check failed -- '
                f'{c["occupied_in_a_segment"]} occupied cells in a segment + '
                f'{graph["unclassified"]["n_cells"]} unclassified != '
                f'{c["occupied_cells"]} occupied, or '
                f'{c["w5_elements_in_two_segments"]} elements in two segments. '
                'Def 6 and W5 are not holding, so nothing here is worth '
                'reading.')
    print()

    if not args.no_obs:
        print('=' * 100)
        print('Part B -- viewing directions (§5). Poses from /tf only, scans '
              'from the raw topic,')
        print('  r_max from the cut\'s own replay config. B1-B3 are truth-free '
              'and live in each')
        print('  map\'s own `checks`.')
        if args.break_name:
            print(f'  (BROKEN ON PURPOSE: --break {args.break_name}. '
                  'Nothing will be written.)')
        print()
        for stem, row in rows.items():
            k, cut = _k_and_cut(stem, 'Part B needs the bag, the replay config '
                                      'and the relay log. Pass --no-obs for '
                                      'other maps')
            row['b1_off_sample'] = obs_for_map(
                row['graph'], stem, k, cut, row['grid'], params,
                break_name=args.break_name)
            print(obs_summary_line(row['graph']))
        print()
        print_b_predictions(rows, args.break_name)

    validation = None
    if not args.no_validate:
        from fit_world_transform import resolve_spawn_poses     # numpy/yaml only
        import robot_divergence as rd

        rd.MAPS_DIR = maps_dir      # the imported registration reads this global
        spawn = resolve_spawn_poses(args.spawn_rev)
        world = sdf_wall_faces(params.l_min)
        samples = face_samples(world, 0.01)
        for stem, row in rows.items():
            k, cut = _k_and_cut(stem, 'A8-A11 need the robot, the cut and the '
                                      'map->odom log. Pass --no-validate for '
                                      'other maps')
            reg = rd.register_map(k, cut, spawn[k])
            graph = row['graph']
            if [round(v, 12) for v in reg['yaml_origin']] != \
                    [round(v, 12) for v in graph['origin']]:
                die(f'{stem}: this file read the YAML origin as '
                    f'{graph["origin"]} and robot_divergence read '
                    f'{reg["yaml_origin"]}. One of the two is not reading this '
                    'map.')
            ew = elements_in_world(row['el'], row['seg_of'], spawn[k])
            row['validation'] = validate_map(graph, ew, spawn, k, world,
                                             samples, graph['resolution'],
                                             radius, row.get('b1_off_sample'))
        validation = print_validation(rows, world, samples, args.spawn_rev)

    if args.break_name:
        print(f'--break {args.break_name} did what it says, so these numbers '
              'describe a deliberately')
        print('  broken run and nothing was written. Re-run without --break for '
              'a measurement.')
        return 2

    if args.no_json:
        print('--no-json: nothing written')
        return 0

    out_dir.mkdir(parents=True, exist_ok=True)
    for stem, row in rows.items():
        path = out_dir / f'{stem}.json'
        path.write_text(json.dumps(row['graph'], indent=2, sort_keys=False)
                        + '\n')
    print(f'wrote {len(rows)} graph JSON files to {out_dir}')

    if validation is not None:
        from fit_world_transform import utc_stamp
        stamp, iso = utc_stamp()
        validation['generated_utc'] = iso
        vpath = out_dir / f'_validation_{stamp}.json'
        vpath.write_text(json.dumps(validation, indent=2, sort_keys=False)
                         + '\n')
        print(f'wrote {vpath}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
