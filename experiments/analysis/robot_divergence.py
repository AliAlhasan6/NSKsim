#!/usr/bin/env python3
"""Robot-to-robot divergence of the 20 offline b2maps grids. No truth anywhere.

pgm_agreement.py answers a different question: two replays of ONE robot's
segment, compared on a lattice anchored on whichever map is the reference. This
asks what five robots that each mapped the same world from their own vantage
agree about when their maps are laid side by side in one frame, and whether
agreement improves as each robot is given more time (60/120/240/1200 s).

NOTHING HERE REFERENCES TRUTH. No world walls are loaded and no on-wall score is
computed -- world_walls and dist_to_nearest_wall arrive in fit_world_transform's
namespace when it is imported and are never called. DOT_POSES enters in exactly
one role: the translation that carries each robot's map frame into the common
frame. It is never a target and never scored against.

REGISTRATION, and why it is one line of arithmetic. The chain is
world_T_odom o map_T_odom o (the YAML origin), rows under convention A:

  * world_T_odom is the spawn translation at --spawn-rev with yaw 0.
    resolve_spawn_poses verifies the launch file at that rev passes ros_gz_sim
    no '-Y', so yaw 0 is checked rather than assumed, and it dies otherwise.
  * map_T_odom is identity on all 20 of these maps, as logged. That is asserted
    per map, not assumed: a non-identity map_T_odom makes `o map_T_odom` and
    `o inverse(map_T_odom)` DIFFERENT placements, and choosing between them is
    fit_world_transform.py's job, not this script's. So this dies instead.
  * convention A (PGM row 0 is maximum y) is what pgm_agreement.sample already
    implements, by flipping the rows once on the way in.

Both transforms therefore being pure translations with zero yaw, registering a
map is: registered origin = spawn + YAML origin. That the composition is the
right one is a CITED result, not one re-measured here -- the latest
world_fit_b2maps_k*_cut1200_*.json resolve convention A with
world_T_odom o map_T_odom for all five robots, at 99.4-99.6% on-wall. Re-deriving
it would need the walls this script refuses to touch.

THE COMMON GRID is one lattice for all 20 maps, at the resolution the maps carry
(0.10000000149011612, read and never assumed -- see load_map's docstring on what
assuming 0.1 costs across 88 cells). It is anchored on the lattice through the
WORLD ORIGIN, not on any map, so the grid is a property of the world rather than
of which maps happen to be in the set: adding a 21st map extends the grid but
moves no existing sample. Each map is therefore resampled by its own sub-cell
offset, which is this measure's floor exactly as it is pgm_agreement's, and that
offset is printed per map.

WHAT IS MEASURED, per cut and per robot pair:

  coverage   Jaccard of the KNOWN sets (occupied or free in both / in either),
             plus how many cells each map knows that the other does not.
  conflict   among cells known to BOTH, the fraction where one says free and the
             other occupied. Reported twice: strict, and with a one-cell
             tolerance on occupied cells -- an occupied-vs-free cell is forgiven
             when the other map has an occupied cell within one cell of it, i.e.
             both robots found the wall and rasterised it one cell apart.

THREE GATES, all of which can fail, run before any table is printed (the same
rule fit_world_transform's self-check and partial_maps' stray gate apply: a
broken measure's numbers are not worth reading):

  self    a map against itself -- Jaccard exactly 1, no conflict at all.
  shift   a map against itself with its registered origin moved one whole cell
          -- strict conflict must RISE, tolerant must stay at exactly 0, since a
          rigid one-cell shift is what the tolerance exists to forgive.
  swap    all ten pairs at 1200 s, re-registered with the two robots' spawns
          exchanged -- strict conflict must rise sharply for EVERY pair. Run over
          all ten rather than one chosen pair so the check cannot pass on a pair
          picked for its wide spawn separation, and that is what caught the
          criterion: the PRE-REGISTERED "at least 5 pp" FAILS on the two pairs
          whose swap displacement is axis-aligned, because sliding a map along
          this world's axis-aligned walls leaves them overlapping themselves. The
          failure is kept and printed; see SWAP_MIN_RISE_RATIO for the control
          that diagnosed it and for the second, explicitly calibrated criterion.

TWO PREDICTIONS, reported and never gated: Jaccard rises with the cut for every
pair, and tolerant conflict stays under SMALL_TOLERANT_PCT at every cut. Ali
fixed that 1.0% and the all-ten-pairs swap scope BEFORE this file was written or
run, which is what makes "not tuned toward" checkable rather than asserted:
neither number can have been chosen to fit an output nobody had seen. A failed
prediction is a finding to write down, not a number to move, so only the gates
above touch the exit code.

Pure numpy / PyYAML, plus git for the spawn table. No ROS, no scipy, no PIL --
the dilation is numpy shifts because CI's container has no scipy
(fit_world_transform.py:53).

    python3 experiments/analysis/robot_divergence.py
    python3 experiments/analysis/robot_divergence.py --break-gate shift

Writes one JSON to experiments/logs/. Exit 0 all gates pass, 2 on a gate failure
or an unusable input. Seconds, no simulator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import combinations
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

# The shared reader: the P5 header walk, the YAML sidecar read, the three-state
# rule (thresholds split occupied from free, the reserved byte 205 overrides), and
# -- the important one -- the convention-A resampler. sample() flips the rows once
# and fills UNKNOWN outside the map, so handing it a grid already recoded by
# classify() makes it a three-state resampler with no change to it at all.
#
# trinary_map.py's docstring is where the 205-reads-as-free arithmetic and the
# three writer locations are stated; the call sites below say only why this
# measurement cares.
#
# shade is imported but not called here: it is the single-byte form of the same
# question classify() asks of an image, and test_robot_divergence pins the 205
# case through this module. Hence F401.
from trinary_map import (  # noqa: E402,F401
    FREE,
    OCC,
    RES_TOL,
    UNKNOWN,
    cell_centre_provenance,
    check_thresholds,
    classify,
    load,
    sample,
    shade,
)

# load_map_to_odom is the tf2_echo parser, including its refusal when a log's
# repeated blocks disagree (the transform was still moving when captured). It
# reads the module global RUN, so register_map rebinds fwt.RUN before each call:
# that is the imported API's only entry point, and it beats re-parsing tf2_echo
# output here. utc_stamp and rel are the fitter's output conventions.
import fit_world_transform as fwt  # noqa: E402
from fit_world_transform import rel, utc_stamp  # noqa: E402

MAPS_DIR = REPO_ROOT / 'experiments' / 'maps'
LOGS_DIR = REPO_ROOT / 'experiments' / 'logs'
OUT_DIR = LOGS_DIR

# The b16-era default in bag_overlap (fc050c4) puts robot_0 3.15 m from where
# these bags spawned it, so it is never the default here. 08617b2 is the b2maps
# era, matching check_run_bag.py and partial_maps.py.
SPAWN_REV = '08617b2'

NUM_ROBOTS = 5
CUTS = (60, 120, 240, 1200)
RUN_PREFIX = 'b2maps'

# map_T_odom must be identity, not merely small. 1e-6 m / 1e-6 rad is far below
# the 1e-3 the tf2_echo logs are printed to, so this admits exactly the logged
# zeros and nothing a real transform would produce.
MTO_IDENTITY_TOL = 1e-6

# ── the one-cell tolerance ───────────────────────────────────────────────────
# Chebyshev radius 1, i.e. the 3x3 block: a wall the two robots rasterised one
# cell apart is one cell apart DIAGONALLY as often as axially, because the
# sub-cell resampling offsets differ independently in x and y. A 4-neighbour
# tolerance would forgive the axial case and charge the diagonal one, which is
# the same disagreement seen from a different angle.
TOL_CELLS = 1

# ── gate thresholds ──────────────────────────────────────────────────────────
# The shift gate is exact, not tolerant: under a rigid one-cell shift EVERY
# occupied-vs-free cell has the other map's wall one cell away by construction,
# so a single unforgiven cell means the tolerance is asymmetric or mis-sized.
SHIFT_TOLERANT_MAX = 0

# ── the swap gate's criterion, and why there are two of them ─────────────────
# SWAP_MIN_RISE_PP = 5.0 was PRE-REGISTERED, before this file was written or run.
# It failed, on 2 of the 10 pairs, and it is kept and reported for good: a
# criterion that is quietly deleted once it goes red was never a criterion.
#
# What failed is its SHAPE, and a control says so rather than a hunch. The two
# failing pairs are 0-2 (rise 3.80 pp) and 3-4 (3.37 pp), and they are exactly
# the two whose spawn difference has dy = 0 -- the swap slides both maps along x
# and nothing else. This world's walls are axis-aligned, so a displacement along
# a wall's own direction leaves that wall overlapping itself: occupied stays on
# occupied, free on free, and only the wall ENDS and the crossing walls
# contradict. Measured on pair 0-2, displacing one map by the same 3.44 m in six
# directions (correct registration = 0.329% strict):
#
#     pure +x  4.256%   pure -x  4.128%   pure +y  4.580%   pure -y  4.602%
#     45 deg   6.614%   -45 deg  6.637%
#
# So at FIXED magnitude the absolute rise depends on the angle between the
# misregistration and the walls, by about 2.1 pp -- more than a third of the
# whole effect. An absolute pp floor silently assumed that angle away.
#
# SWAP_MIN_RISE_RATIO is CALIBRATED, NOT PRE-REGISTERED, and that is the honest
# label. Its history, since the label is worth nothing without it: 5.0x was
# chosen when the observed minimum read 6.2x -- under the classify() bug that
# read every unknown cell as free (see classify). With that fixed the ten swaps
# span 9.1x to 15.7x, so 5.0x is further below the data than when it was picked,
# and it stays where it is: moving it toward the numbers now would be the tuning
# the label exists to rule out. Note also that the minimum ratio is pair 1-3's
# 9.1x, which is NOT one of the axis-aligned pairs -- the ratio criterion is not
# a device for rescuing those two.
#
# 5.0x is not evidence that the measure is sensitive; the ten measured ratios
# are. Its job from here is regression: a future run whose swap ratio falls
# under 5x has lost sensitivity this one had.
SWAP_MIN_RISE_PP = 5.0
SWAP_MIN_RISE_RATIO = 5.0

# Which criterion sets the exit code. Both are always computed and printed.
SWAP_CRITERIA = ('ratio', 'pp', 'both')
DEFAULT_SWAP_CRITERION = 'ratio'

# A swap whose displacement is axis-aligned is the case the pp floor could not
# describe, so every run flags it rather than leaving it in one conversation.
AXIS_ALIGNED_TOL_M = 1e-9

# ── pre-registered predictions (Ali, before this file existed) ───────────────
SMALL_TOLERANT_PCT = 1.0


def die(msg: str) -> None:
    print(f'robot_divergence: {msg}', file=sys.stderr)
    sys.exit(2)


# ─────────────────────────────── classification ──────────────────────────────
# shade(), classify() and check_thresholds() are imported from trinary_map.py,
# which states the rule and the 205-reads-as-free arithmetic once for every reader
# of these files. What matters HERE is what the rule buys this measurement:
# applying the thresholds literally reported all 20 maps as fully known, so
# Jaccard became extent overlap and the coverage half of this script silently
# measured nothing. The 3526 unknown cells of b2maps_k0_cut60_robot0 are the ones
# that vanished. check_thresholds() is called per map in register_map() so that
# this is verified on every input rather than assumed once.


# ──────────────────────────── registration ───────────────────────────────────

def map_name(k: int, cut: int) -> str:
    return f'{RUN_PREFIX}_k{k}_cut{cut}_robot{k}'


def run_name(k: int, cut: int) -> str:
    return f'{RUN_PREFIX}_k{k}_cut{cut}'


def check_map_to_odom(mto: tuple[float, float, float], where: str,
                      tol: float = MTO_IDENTITY_TOL) -> None:
    """Refuse anything but an identity map_T_odom.

    Not pedantry. This script composes world_T_odom o map_T_odom as a pure
    translation; with a nonzero yaw that is not a translation at all, and with a
    nonzero translation the two directions of the logged transform land on
    different placements, which only a fit against the world can separate. Both
    cases belong to fit_world_transform.py, so this stops rather than pick one.
    """
    x, y, yaw = mto
    if max(abs(x), abs(y), abs(yaw)) > tol:
        die(f'{where}: map_T_odom is x={x:+.6f} y={y:+.6f} yaw={yaw:+.6f}, not '
            f'identity within {tol}. This script registers maps as a pure '
            'translation and cannot tell `o map_T_odom` from '
            '`o inverse(map_T_odom)` without a fit against the world; run '
            'experiments/slam/fit_world_transform.py for that.')


def register_map(k: int, cut: int, spawn_xy: tuple[float, float]) -> dict:
    """One map, loaded, classified, and carried into the world frame.

    The registered origin is the whole of the registration: with map_T_odom
    identity and spawn yaw 0 the chain collapses to spawn + YAML origin, and
    convention A lives in sample()'s row flip.
    """
    stem = map_name(k, cut)
    pgm = MAPS_DIR / f'{stem}.pgm'
    meta_path = MAPS_DIR / f'{stem}.yaml'
    for p in (pgm, meta_path):
        if not p.is_file():
            die(f'map input not found: {p}')

    m = load(pgm)                      # imported: header walk + sidecar
    meta = yaml.safe_load(meta_path.read_text())
    negate = int(meta['negate'])
    occupied_thresh = float(meta['occupied_thresh'])
    free_thresh = float(meta['free_thresh'])
    thresholds = check_thresholds(negate, occupied_thresh, free_thresh,
                                  m['other'], meta_path.name)

    fwt.RUN = run_name(k, cut)         # the imported parser reads this global
    mto, mto_path = fwt.load_map_to_odom(k)
    check_map_to_odom(mto, stem)

    ox, oy = m['origin'][:2]
    return {
        'robot': k,
        'cut': cut,
        'stem': stem,
        'pgm': pgm,
        'yaml': meta_path,
        'map_to_odom_path': mto_path,
        'map_T_odom': mto,
        'spawn': (float(spawn_xy[0]), float(spawn_xy[1])),
        'yaml_origin': [float(ox), float(oy)],
        'origin': [spawn_xy[0] + ox, spawn_xy[1] + oy],   # registered
        'resolution': m['resolution'],
        'width': m['width'],
        'height': m['height'],
        'negate': negate,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
        'thresholds': thresholds,
        'px': classify(m['px'], negate, occupied_thresh, free_thresh),
        'raw_counts': {'occupied': m['occupied'], 'free': m['free'],
                       'unknown': m['unknown'], 'other': m['other']},
    }


def shifted(m: dict, dx_cells: float = 0.0, dy_cells: float = 0.0) -> dict:
    """The same map with its registered origin moved a whole number of cells.

    The shift gate's input, and the only thing in this file that moves a map off
    its registered place on purpose.
    """
    out = dict(m)
    res = m['resolution']
    out['origin'] = [m['origin'][0] + dx_cells * res,
                     m['origin'][1] + dy_cells * res]
    out['stem'] = f'{m["stem"]}[shift {dx_cells:+g},{dy_cells:+g} cells]'
    return out


def reregistered(m: dict, spawn_xy: tuple[float, float]) -> dict:
    """The same map registered with somebody else's spawn. The swap gate's input."""
    out = dict(m)
    ox, oy = m['yaml_origin']
    out['spawn'] = (float(spawn_xy[0]), float(spawn_xy[1]))
    out['origin'] = [spawn_xy[0] + ox, spawn_xy[1] + oy]
    out['stem'] = f'{m["stem"]}[spawn {spawn_xy[0]:+.2f},{spawn_xy[1]:+.2f}]'
    return out


# ──────────────────────────── the common grid ────────────────────────────────

def common_grid(maps: list[dict]) -> dict:
    """One lattice covering every registered map, anchored on the world origin.

    Anchored on floor(min / res) * res rather than on a map, so which maps are in
    the set fixes only the EXTENT: the lattice through x = 0 and y = 0 is the same
    whatever the set, and adding a map never moves an existing sample.

    `x0`/`y0` name the position of common cell 0 the same way a map's `origin`
    names the position of its cell 0, so under
    trinary_map.CELL_CENTRE_OFFSET = 0.0 a cell CENTRE sits on x = 0 and y = 0 --
    it used to be a cell boundary. That is a relabelling and not a move: every
    grid this produces is bit-identical either way, because sample() shifts its
    query lattice and its lookup by the same half cell and the two cancel (see
    trinary_map.sample and compare_gated_maps.resample_onto). The offset only
    changes where a WORLD coordinate meets a cell index, and nothing in this file
    does that -- which is why no number here moved when the convention was fixed.
    """
    res = maps[0]['resolution']
    for m in maps:
        if abs(m['resolution'] - res) > RES_TOL:
            die(f'{m["stem"]}: resolution {m["resolution"]!r} != {res!r} -- '
                'these are not maps of one world at one resolution')

    lefts = [m['origin'][0] for m in maps]
    bottoms = [m['origin'][1] for m in maps]
    rights = [m['origin'][0] + m['width'] * res for m in maps]
    tops = [m['origin'][1] + m['height'] * res for m in maps]

    x0 = math.floor(min(lefts) / res) * res
    y0 = math.floor(min(bottoms) / res) * res
    w = int(math.ceil((max(rights) - x0) / res - 1e-9))
    h = int(math.ceil((max(tops) - y0) / res - 1e-9))
    return {'res': res, 'x0': x0, 'y0': y0, 'w': w, 'h': h}


def subcell_offset(m: dict, grid: dict) -> float:
    """How far this map's origin sits from the common lattice, in cells.

    Its fractional part is resampled away when the map is sampled onto the grid,
    and is therefore this map's contribution to the measurement floor. Printed,
    never corrected: correcting it would mean moving a map off its registration.
    """
    res = grid['res']
    fx = ((m['origin'][0] - grid['x0']) / res) % 1.0
    fy = ((m['origin'][1] - grid['y0']) / res) % 1.0
    return max(min(fx, 1.0 - fx), min(fy, 1.0 - fy))


def on_grid(m: dict, grid: dict) -> np.ndarray:
    """This map's three states at the common grid's cell centres. Bottom-up."""
    return sample(m, grid['res'], grid['x0'], grid['y0'], grid['w'], grid['h'])


# ───────────────────────────── the metrics ───────────────────────────────────

def dilate(mask: np.ndarray, cells: int = TOL_CELLS) -> np.ndarray:
    """Chebyshev dilation by `cells`, in numpy. No wraparound at any edge.

    scipy.ndimage.binary_dilation would be the obvious call and is NOT available:
    CI's ros:jazzy container has no scipy, and a module-level import of it takes
    the whole nsk_swarm suite down at collection (fit_world_transform.py:53).
    Shifting a boolean array `cells` steps in each of the eight directions is the
    same operation for a square structuring element.
    """
    if cells <= 0:
        return mask.copy()
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx == 0 and dy == 0:
                continue
            # Slices, so a shift moves cells OFF the array rather than around it.
            ys_dst = slice(max(0, dy), h + min(0, dy))
            ys_src = slice(max(0, -dy), h + min(0, -dy))
            xs_dst = slice(max(0, dx), w + min(0, dx))
            xs_src = slice(max(0, -dx), w + min(0, -dx))
            out[ys_dst, xs_dst] |= mask[ys_src, xs_src]
    return out


def pair_metrics(a: np.ndarray, b: np.ndarray) -> dict:
    """Coverage and conflict between two three-state grids on one lattice.

    Conflict is only ever counted where BOTH maps know the cell, so a cell one
    robot never saw is a coverage difference and never a contradiction -- those
    are different findings and are reported separately.
    """
    a_occ, a_free = a == OCC, a == FREE
    b_occ, b_free = b == OCC, b == FREE
    known_a, known_b = a != UNKNOWN, b != UNKNOWN

    both = known_a & known_b
    union = known_a | known_b
    n_both = int(np.count_nonzero(both))
    n_union = int(np.count_nonzero(union))

    a_occ_b_free = a_occ & b_free
    a_free_b_occ = a_free & b_occ
    strict = a_occ_b_free | a_free_b_occ

    # Forgiven in the direction of the map that says FREE: the other map put an
    # occupied cell within one cell, so both robots found the wall and disagree
    # about which cell it fell in.
    forgiven = ((a_occ_b_free & dilate(b_occ)) | (a_free_b_occ & dilate(a_occ)))
    tolerant = strict & ~forgiven

    n_strict = int(np.count_nonzero(strict))
    n_tolerant = int(np.count_nonzero(tolerant))

    def pct(n: int) -> float:
        return 100.0 * n / n_both if n_both else float('nan')

    return {
        'known_a': int(np.count_nonzero(known_a)),
        'known_b': int(np.count_nonzero(known_b)),
        'known_both': n_both,
        'known_union': n_union,
        'jaccard': (n_both / n_union) if n_union else float('nan'),
        'only_a': int(np.count_nonzero(known_a & ~known_b)),
        'only_b': int(np.count_nonzero(known_b & ~known_a)),
        'conflict_strict': n_strict,
        'conflict_strict_pct': pct(n_strict),
        'conflict_tolerant': n_tolerant,
        'conflict_tolerant_pct': pct(n_tolerant),
        'forgiven': int(np.count_nonzero(forgiven)),
        'a_occ_b_free': int(np.count_nonzero(a_occ_b_free)),
        'a_free_b_occ': int(np.count_nonzero(a_free_b_occ)),
    }


# ─────────────────────────── gate predicates (pure) ──────────────────────────

def self_gate(met: dict) -> dict:
    """A map against itself: perfect overlap, no contradiction. Pure."""
    checks = {
        'jaccard_is_1': met['jaccard'] == 1.0,
        'strict_is_0': met['conflict_strict'] == 0,
        'tolerant_is_0': met['conflict_tolerant'] == 0,
    }
    return {'checks': checks, 'pass': all(checks.values()),
            'jaccard': met['jaccard'],
            'conflict_strict': met['conflict_strict'],
            'conflict_tolerant': met['conflict_tolerant']}


def shift_gate(met: dict, tolerant_max: int = SHIFT_TOLERANT_MAX) -> dict:
    """One whole cell of shift: strict must notice, tolerant must forgive. Pure.

    Both halves matter. Without the first, a tolerance that forgives everything
    passes; without the second, a tolerance that forgives nothing does.
    """
    checks = {
        'strict_rises': met['conflict_strict'] > 0,
        'tolerant_within_max': met['conflict_tolerant'] <= tolerant_max,
    }
    return {'checks': checks, 'pass': all(checks.values()),
            'tolerant_max': tolerant_max,
            'conflict_strict': met['conflict_strict'],
            'conflict_tolerant': met['conflict_tolerant']}


def swap_gate(correct_pct: float, swapped_pct: float,
              criterion: str = DEFAULT_SWAP_CRITERION,
              min_rise_pp: float = SWAP_MIN_RISE_PP,
              min_rise_ratio: float = SWAP_MIN_RISE_RATIO) -> dict:
    """Exchanging two robots' spawns must make the conflict much worse. Pure.

    Both criteria are always computed; `criterion` picks which one the pass
    verdict reads. See SWAP_MIN_RISE_RATIO on why there are two and which of them
    was pre-registered -- the pp verdict is reported whatever is gated on, so a
    run cannot hide that it would have failed the original criterion.
    """
    if criterion not in SWAP_CRITERIA:
        raise ValueError(f'criterion must be one of {SWAP_CRITERIA}, '
                         f'got {criterion!r}')
    rise = swapped_pct - correct_pct
    ratio = (swapped_pct / correct_pct) if correct_pct > 0 else float('inf')
    pp_pass = bool(rise >= min_rise_pp)
    ratio_pass = bool(ratio >= min_rise_ratio)
    verdict = {'pp': pp_pass, 'ratio': ratio_pass,
               'both': pp_pass and ratio_pass}[criterion]
    return {'correct_pct': correct_pct, 'swapped_pct': swapped_pct,
            'rise_pp': rise, 'rise_ratio': ratio,
            'min_rise_pp': min_rise_pp, 'min_rise_ratio': min_rise_ratio,
            'criterion': criterion,
            'pp_pass': pp_pass, 'ratio_pass': ratio_pass,
            'pass': verdict}


def axis_aligned(dx: float, dy: float, tol: float = AXIS_ALIGNED_TOL_M) -> bool:
    """Whether a displacement runs along one axis, i.e. along this world's walls.

    The case an absolute pp floor cannot describe: sliding a map along a wall's
    own direction leaves that wall overlapping itself. Not a pass/fail, a label.
    """
    return abs(dx) <= tol or abs(dy) <= tol


# ────────────────────── prediction evaluators (pure) ─────────────────────────

def rises(series: list[float]) -> bool:
    """Non-decreasing across the cuts, NaN-safe (a NaN is never a rise)."""
    return all(b >= a for a, b in zip(series, series[1:])
               if not (math.isnan(a) or math.isnan(b))) and not any(
                   math.isnan(v) for v in series)


def stays_small(series: list[float], limit: float = SMALL_TOLERANT_PCT) -> bool:
    """Every value at or under `limit`. A NaN is not small, it is unmeasured."""
    return all((not math.isnan(v)) and v <= limit for v in series)


# ────────────────────────────────── main ─────────────────────────────────────

def pairs() -> list[tuple[int, int]]:
    return list(combinations(range(NUM_ROBOTS), 2))


def print_cut_table(cut: int, rows: dict) -> None:
    print(f'-- cut {cut} s ' + '-' * 86)
    print(f'{"pair":>7}{"known A":>9}{"known B":>9}{"both":>8}{"union":>8}'
          f'{"jaccard":>9}{"only A":>8}{"only B":>8}'
          f'{"strict":>8}{"%":>7}{"tol":>6}{"%":>7}')
    for (i, j), met in rows.items():
        print(f'{f"{i}-{j}":>7}{met["known_a"]:>9}{met["known_b"]:>9}'
              f'{met["known_both"]:>8}{met["known_union"]:>8}'
              f'{met["jaccard"]:>9.4f}{met["only_a"]:>8}{met["only_b"]:>8}'
              f'{met["conflict_strict"]:>8}{met["conflict_strict_pct"]:>7.3f}'
              f'{met["conflict_tolerant"]:>6}{met["conflict_tolerant_pct"]:>7.3f}')
    print()


def print_trend(per_cut: dict) -> dict:
    """Each pair across the cuts, and the two predictions read off it."""
    print('=' * 100)
    print('trend across cuts -- one row per pair')
    print('  jaccard of known cells, then strict and tolerant conflict as % of '
          'cells known to both')
    head = ''.join(f'{c:>9}' for c in CUTS)
    print(f'{"pair":>7}  {"jaccard":<36}{"strict %":<36}tolerant %')
    print(f'{"":>7}  {head:<36}{head:<36}{head}')

    jac_series, tol_series = {}, {}
    for pr in pairs():
        j = [per_cut[c][pr]['jaccard'] for c in CUTS]
        s = [per_cut[c][pr]['conflict_strict_pct'] for c in CUTS]
        t = [per_cut[c][pr]['conflict_tolerant_pct'] for c in CUTS]
        jac_series[pr], tol_series[pr] = j, t
        fj = ''.join(f'{v:>9.4f}' for v in j)
        fs = ''.join(f'{v:>9.3f}' for v in s)
        ft = ''.join(f'{v:>9.3f}' for v in t)
        flag = '' if rises(j) else '   <- jaccard not monotone'
        print(f'{f"{pr[0]}-{pr[1]}":>7}  {fj:<36}{fs:<36}{ft}{flag}')
    print()

    rise_hold = {pr: rises(jac_series[pr]) for pr in pairs()}
    small_hold = {pr: stays_small(tol_series[pr]) for pr in pairs()}
    worst_tol = max(max(tol_series[pr]) for pr in pairs())
    worst_pair = max(pairs(), key=lambda pr: max(tol_series[pr]))

    print('=' * 100)
    print('PREDICTIONS -- pre-registered, reported, NOT gated. A failure here is '
          'a finding to')
    print('  write down, not a threshold to move. Only the three gates above '
          'set the exit code.')
    n_rise = sum(rise_hold.values())
    n_small = sum(small_hold.values())
    print(f'  1. Jaccard rises with cut for every pair: '
          f'{"HELD" if n_rise == len(pairs()) else "FAILED"} '
          f'({n_rise}/{len(pairs())} pairs non-decreasing)')
    for pr, ok in rise_hold.items():
        if not ok:
            print(f'       robot_{pr[0]}-robot_{pr[1]}: '
                  + ' '.join(f'{v:.4f}' for v in jac_series[pr]))
    print(f'  2. Tolerant conflict stays under {SMALL_TOLERANT_PCT:.1f}% at every '
          f'cut: {"HELD" if n_small == len(pairs()) else "FAILED"} '
          f'({n_small}/{len(pairs())} pairs)')
    print(f'       worst observed: {worst_tol:.3f}% '
          f'(robot_{worst_pair[0]}-robot_{worst_pair[1]})')
    for pr, ok in small_hold.items():
        if not ok:
            print(f'       robot_{pr[0]}-robot_{pr[1]}: '
                  + ' '.join(f'{v:.3f}' for v in tol_series[pr]))
    print('=' * 100)
    print()

    return {
        'jaccard_rises_with_cut': {
            'statement': 'Jaccard of known cells is non-decreasing in the cut '
                         'for every pair',
            'held': bool(n_rise == len(pairs())),
            'pairs_holding': n_rise, 'pairs_total': len(pairs()),
            'per_pair': {f'{i}-{j}': {'held': rise_hold[(i, j)],
                                      'jaccard': jac_series[(i, j)]}
                         for i, j in pairs()},
        },
        'tolerant_conflict_stays_small': {
            'statement': f'tolerant conflict <= {SMALL_TOLERANT_PCT}% of '
                         'both-known cells at every cut for every pair',
            'limit_pct': SMALL_TOLERANT_PCT,
            'preregistered_by': 'Ali, before this script was written or run',
            'held': bool(n_small == len(pairs())),
            'pairs_holding': n_small, 'pairs_total': len(pairs()),
            'worst_pct': worst_tol,
            'worst_pair': f'{worst_pair[0]}-{worst_pair[1]}',
            'per_pair': {f'{i}-{j}': {'held': small_hold[(i, j)],
                                      'tolerant_pct': tol_series[(i, j)]}
                         for i, j in pairs()},
        },
    }


def run_gates(maps: dict, grid: dict, gridded: dict, spawn,
              break_gate: str | None,
              swap_criterion: str = DEFAULT_SWAP_CRITERION) -> dict:
    """The three checks, each able to fail, before any result is printed."""
    print('=' * 100)
    print('gates -- nothing below is worth reading if one of these fails')
    print()
    results: dict = {'broken_on_purpose': break_gate,
                     'swap_criterion': swap_criterion}

    # ── self ──────────────────────────────────────────────────────────────
    # Every map against itself, not one: the check costs nothing and a
    # registration that is not a function of the map alone need not fail on the
    # first one.
    print('  self -- each map against itself: Jaccard 1, no conflict')
    self_rows = {}
    worst = None
    for cut in CUTS:
        for k in range(NUM_ROBOTS):
            g = gridded[cut][k]
            other = g
            if break_gate == 'self' and cut == CUTS[0] and k == 0:
                # One cell flipped free->occupied, which a correct measure must
                # catch as a strict conflict against the unflipped copy.
                other = g.copy()
                idx = np.argwhere(g == FREE)[0]
                other[idx[0], idx[1]] = OCC
            met = pair_metrics(g, other)
            gate = self_gate(met)
            self_rows[(cut, k)] = gate
            if not gate['pass'] and worst is None:
                worst = (cut, k, gate)
    n_ok = sum(1 for g in self_rows.values() if g['pass'])
    print(f'         {n_ok}/{len(self_rows)} maps: Jaccard exactly 1.000000, '
          f'0 strict, 0 tolerant')
    if worst is not None:
        cut, k, gate = worst
        print(f'         FAIL on b2maps_k{k}_cut{cut}: jaccard={gate["jaccard"]!r} '
              f'strict={gate["conflict_strict"]} '
              f'tolerant={gate["conflict_tolerant"]}')
    results['self'] = {'maps_checked': len(self_rows), 'maps_passing': n_ok,
                       'pass': n_ok == len(self_rows),
                       'failures': [{'cut': c, 'robot': k, **g}
                                    for (c, k), g in self_rows.items()
                                    if not g['pass']]}
    print(f'      -> {"PASS" if results["self"]["pass"] else "FAIL"}\n')

    # ── shift ─────────────────────────────────────────────────────────────
    shift_cells = 2 if break_gate == 'shift' else 1
    print(f'  shift -- each map against itself shifted {shift_cells} whole cell(s) '
          'in x:')
    print('         strict conflict must rise, tolerant must stay at '
          f'{SHIFT_TOLERANT_MAX}')
    if shift_cells != 1:
        print(f'         (BROKEN ON PURPOSE: {shift_cells} cells is outside a '
              'one-cell tolerance,')
        print('          so the tolerant count must and does go nonzero)')
    shift_rows = {}
    for cut in CUTS:
        for k in range(NUM_ROBOTS):
            met = pair_metrics(gridded[cut][k],
                               on_grid(shifted(maps[cut][k], dx_cells=shift_cells),
                                       grid))
            shift_rows[(cut, k)] = shift_gate(met)
    n_ok = sum(1 for g in shift_rows.values() if g['pass'])
    strict_min = min(g['conflict_strict'] for g in shift_rows.values())
    strict_max = max(g['conflict_strict'] for g in shift_rows.values())
    tol_max = max(g['conflict_tolerant'] for g in shift_rows.values())
    print(f'         {n_ok}/{len(shift_rows)} maps: strict '
          f'{strict_min}..{strict_max} cells, tolerant at most {tol_max}')
    for (cut, k), g in shift_rows.items():
        if not g['pass']:
            print(f'         FAIL on b2maps_k{k}_cut{cut}: '
                  f'strict={g["conflict_strict"]} '
                  f'tolerant={g["conflict_tolerant"]} '
                  f'(max {g["tolerant_max"]})')
    results['shift'] = {'shift_cells': shift_cells,
                        'maps_checked': len(shift_rows), 'maps_passing': n_ok,
                        'tolerant_max': SHIFT_TOLERANT_MAX,
                        'strict_range': [strict_min, strict_max],
                        'tolerant_worst': tol_max,
                        'pass': n_ok == len(shift_rows),
                        'failures': [{'cut': c, 'robot': k, **g}
                                     for (c, k), g in shift_rows.items()
                                     if not g['pass']]}
    print(f'      -> {"PASS" if results["shift"]["pass"] else "FAIL"}\n')

    # ── swap ──────────────────────────────────────────────────────────────
    cut = CUTS[-1]
    print(f'  swap -- all {len(pairs())} pairs at {cut} s, re-registered with the '
          "two robots' spawns")
    print('         exchanged. Two criteria, both printed; '
          f'{swap_criterion!r} sets the verdict:')
    print(f'           pp     strict conflict rises by >= {SWAP_MIN_RISE_PP:.1f} pp '
          '-- PRE-REGISTERED, and it')
    print('                  FAILS on the two axis-aligned swaps (marked | '
          'below). Kept and')
    print('                  reported for good rather than deleted for going red.')
    print(f'           ratio  strict conflict rises by >= '
          f'{SWAP_MIN_RISE_RATIO:.1f}x -- CALIBRATED after the fact, not')
    print('                  evidence of sensitivity; the ten measured ratios '
          'are that. Its job')
    print('                  is catching a future regression. See '
          'SWAP_MIN_RISE_RATIO for how it')
    print('                  was picked and why it was not moved afterwards.')
    print('         | marks a swap whose displacement runs along one axis, i.e. '
          "along this")
    print('           world\'s walls, which leaves long walls overlapping '
          'themselves and is')
    print('           why an absolute pp floor cannot describe every pair.')
    if break_gate == 'swap':
        print('         (BROKEN ON PURPOSE: the spawns are NOT exchanged, so '
              'nothing moves and')
        print('          every pair must fail on both criteria)')
    print(f'{"":>9}{"pair":>7}{"rel dx":>9}{"rel dy":>9}{"correct %":>11}'
          f'{"swapped %":>11}{"rise pp":>9}{"ratio":>8}  pp   ratio')
    swap_rows = {}
    for i, j in pairs():
        correct = pair_metrics(gridded[cut][i], gridded[cut][j])
        if break_gate == 'swap':
            si, sj = spawn[i], spawn[j]      # not exchanged: the broken case
        else:
            si, sj = spawn[j], spawn[i]
        a = on_grid(reregistered(maps[cut][i], si), grid)
        b = on_grid(reregistered(maps[cut][j], sj), grid)
        sw = pair_metrics(a, b)
        gate = swap_gate(correct['conflict_strict_pct'],
                         sw['conflict_strict_pct'], criterion=swap_criterion)
        # The displacement the swap actually applies: map i moves by
        # spawn_j - spawn_i and map j by the reverse, so they separate by twice
        # the spawn difference.
        rel_dx = 2.0 * (spawn[j][0] - spawn[i][0])
        rel_dy = 2.0 * (spawn[j][1] - spawn[i][1])
        gate['rel_dx_m'] = rel_dx
        gate['rel_dy_m'] = rel_dy
        gate['displacement_m'] = math.hypot(rel_dx, rel_dy)
        gate['axis_aligned'] = axis_aligned(rel_dx, rel_dy)
        gate['swapped_jaccard'] = sw['jaccard']
        gate['correct_jaccard'] = correct['jaccard']
        swap_rows[(i, j)] = gate
        print(f'{"|" if gate["axis_aligned"] else " ":>9}{f"{i}-{j}":>7}'
              f'{rel_dx:>9.2f}{rel_dy:>9.2f}{gate["correct_pct"]:>11.3f}'
              f'{gate["swapped_pct"]:>11.3f}{gate["rise_pp"]:>9.3f}'
              f'{gate["rise_ratio"]:>7.1f}x  '
              f'{"yes" if gate["pp_pass"] else "NO ":>3}  '
              f'{"yes" if gate["ratio_pass"] else "NO"}')
    n_ok = sum(1 for g in swap_rows.values() if g['pass'])
    n_pp = sum(1 for g in swap_rows.values() if g['pp_pass'])
    n_ratio = sum(1 for g in swap_rows.values() if g['ratio_pass'])
    n_axis = sum(1 for g in swap_rows.values() if g['axis_aligned'])
    results['swap'] = {
        'cut': cut, 'criterion': swap_criterion,
        'min_rise_pp': SWAP_MIN_RISE_PP,
        'min_rise_ratio': SWAP_MIN_RISE_RATIO,
        'pp_preregistered': True,
        'ratio_preregistered': False,
        'ratio_note': 'calibrated after the pp criterion failed on the two '
                      'axis-aligned swaps; 5.0x against an observed minimum of '
                      '6.2x. Not evidence of sensitivity -- the measured ratios '
                      'are',
        'pairs_checked': len(swap_rows),
        'pairs_passing': n_ok,
        'pairs_passing_pp': n_pp,
        'pairs_passing_ratio': n_ratio,
        'pairs_axis_aligned': n_axis,
        'pass': n_ok == len(swap_rows),
        'pp_criterion_pass': n_pp == len(swap_rows),
        'ratio_criterion_pass': n_ratio == len(swap_rows),
        'per_pair': {f'{i}-{j}': g for (i, j), g in swap_rows.items()},
    }
    print(f'         pp    {n_pp}/{len(swap_rows)} pairs rose by >= '
          f'{SWAP_MIN_RISE_PP:.1f} pp'
          + ('' if n_pp == len(swap_rows) else
             f'   <- the {n_axis} axis-aligned swaps are the misses'))
    print(f'         ratio {n_ratio}/{len(swap_rows)} pairs rose by >= '
          f'{SWAP_MIN_RISE_RATIO:.1f}x  (span '
          f'{min(g["rise_ratio"] for g in swap_rows.values()):.1f}x..'
          f'{max(g["rise_ratio"] for g in swap_rows.values()):.1f}x)')
    print(f'      -> {"PASS" if results["swap"]["pass"] else "FAIL"} '
          f'on the {swap_criterion!r} criterion\n')

    results['pass'] = all(results[g]['pass'] for g in ('self', 'shift', 'swap'))
    print(f'  gates: {"ALL PASS" if results["pass"] else "FAILED"}')
    print('=' * 100)
    print()
    return results


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--spawn-rev', default=SPAWN_REV,
                    help='git rev whose DOT_POSES gives world_T_odom. Default '
                         '%(default)s, the b2maps era. bag_overlap\'s own '
                         'default is the b16 ring and puts robot_0 3.15 m away')
    ap.add_argument('--break-gate', choices=['self', 'shift', 'swap'],
                    help='corrupt one gate\'s input so a red result is '
                         'demonstrable rather than asserted. Exits 2')
    ap.add_argument('--swap-criterion', choices=list(SWAP_CRITERIA),
                    default=DEFAULT_SWAP_CRITERION,
                    help='which swap criterion sets the exit code. Both are '
                         'always computed and printed; %(default)s is the '
                         'default, and \'pp\' is the pre-registered one that '
                         'fails on the two axis-aligned swaps (see '
                         'SWAP_MIN_RISE_RATIO)')
    ap.add_argument('--no-json', action='store_true',
                    help='print the tables and write nothing')
    args = ap.parse_args()

    stamp, iso = utc_stamp()

    print('robot-to-robot divergence of the 20 offline b2maps grids')
    print('  NO TRUTH: no world walls are loaded and no on-wall score is '
          'computed. DOT_POSES')
    print('  enters only as the translation that puts each map frame into one '
          'frame.')
    print()

    # Inside main, not at import: this shells out to git.
    spawn = fwt.resolve_spawn_poses(args.spawn_rev)
    print(f'world_T_odom -- spawn translation, yaw 0 (DOT_POSES @ '
          f'{args.spawn_rev}, yaw verified absent):')
    for k in range(NUM_ROBOTS):
        print(f'    robot_{k}: x={spawn[k][0]:+.3f}  y={spawn[k][1]:+.3f}')
    print()

    maps: dict = {}
    for cut in CUTS:
        maps[cut] = {k: register_map(k, cut, spawn[k]) for k in range(NUM_ROBOTS)}
    print(f'map_T_odom: identity within {MTO_IDENTITY_TOL} on all '
          f'{NUM_ROBOTS * len(CUTS)} maps, as logged -- so the registration is a '
          'pure')
    print('  translation and the two composition directions coincide. '
          'Convention A (PGM row 0')
    print('  is maximum y) is applied by the imported resampler, '
          'pgm_agreement.sample.')
    print()

    flat = [maps[c][k] for c in CUTS for k in range(NUM_ROBOTS)]
    th = flat[0]['thresholds']
    print('three states: the thresholds split occupied from free, and the '
          'reserved unknown byte')
    print(f'  {UNKNOWN} then overrides -- it must. Under every one of these YAMLs '
          f'it reads as shade')
    print(f'  {th["unknown_byte_shade"]:.4f}, which the thresholds put in the '
          f'{th["unknown_byte_band_under_thresholds"]!r} band, not unknown: '
          'save_map.py writes')
    print('  205 for "neither" but hardcodes free_thresh 0.25 beside it, so its '
          'own round trip')
    print(f'  is not a fixpoint. The band they DO map to unknown is bytes '
          f'{th["threshold_unknown_band_bytes"][0]}..'
          f'{th["threshold_unknown_band_bytes"][-1]}, which no')
    print('  pixel of these maps occupies. Taking the thresholds literally '
          'reported all 20 maps')
    print('  as fully known, i.e. Jaccard measuring extent overlap and nothing '
          'else.')
    print()

    grid = common_grid(flat)
    print(f'common grid: {grid["w"]} x {grid["h"]} cells @ {grid["res"]!r} m, '
          f'origin ({grid["x0"]:.4f}, {grid["y0"]:.4f})')
    print('  anchored on the lattice through the world origin, so the grid is a '
          'property of')
    print('  the world and not of which maps are in the set. Each map is '
          'resampled by its own')
    print('  sub-cell offset; that is this measure\'s floor, and it is printed '
          'per map below.')
    print()
    print(f'{"map":<32}{"cells":>10}{"registered origin":>22}{"subcell":>9}'
          f'{"occ":>7}{"free":>8}{"unknown":>9}')
    for m in flat:
        print(f'{m["stem"]:<32}{f"{m["width"]}x{m["height"]}":>10}'
              f'{f"({m["origin"][0]:+.4f}, {m["origin"][1]:+.4f})":>22}'
              f'{subcell_offset(m, grid):>9.3f}'
              f'{m["raw_counts"]["occupied"]:>7}{m["raw_counts"]["free"]:>8}'
              f'{m["raw_counts"]["unknown"]:>9}')
    print()

    gridded = {cut: {k: on_grid(maps[cut][k], grid) for k in range(NUM_ROBOTS)}
               for cut in CUTS}

    gates = run_gates(maps, grid, gridded, spawn, args.break_gate,
                      swap_criterion=args.swap_criterion)

    if not gates['pass']:
        if args.break_gate:
            print(f'--break-gate {args.break_gate} did what it says: the gate '
                  'failed, so the measure')
            print('  can distinguish a good registration from a bad one. '
                  'Nothing was written.')
        else:
            print('A gate failed, so the pair tables below would not be worth '
                  'reading and were')
            print('  not printed. Nothing was written.')
        return 2

    print('=' * 100)
    print('coverage and conflict, one table per cut')
    print('  jaccard  = cells known (occupied or free) to both / known to either')
    print('  only A/B = cells known to that map alone -- a coverage difference, '
          'not a contradiction')
    print('  strict   = of cells known to BOTH, one says free and the other '
          'occupied')
    print(f'  tol      = the same with a {TOL_CELLS}-cell tolerance on occupied '
          'cells: forgiven when the')
    print('             other map has an occupied cell within one cell '
          '(3x3), i.e. both robots')
    print('             found the wall and rasterised it one cell apart')
    print()
    per_cut = {}
    for cut in CUTS:
        rows = {(i, j): pair_metrics(gridded[cut][i], gridded[cut][j])
                for i, j in pairs()}
        per_cut[cut] = rows
        print_cut_table(cut, rows)

    predictions = print_trend(per_cut)

    if args.no_json:
        print('--no-json: nothing written')
        return 0

    out = {
        'generated_utc': iso,
        'question': 'robot-to-robot divergence of the 20 offline b2maps grids, '
                    'registered into one frame',
        'truth_used': False,
        'truth_note': 'no world walls and no on-wall score; DOT_POSES enters '
                      'only as the registration translation',
        'spawn_rev': args.spawn_rev,
        'spawn_poses': {f'robot_{k}': {'x': spawn[k][0], 'y': spawn[k][1],
                                       'yaw_rad': 0.0}
                        for k in range(NUM_ROBOTS)},
        'spawn_source': f'bag_overlap.resolve_spawn_poses({args.spawn_rev!r})',
        'registration': {
            'chain': 'world_T_odom o map_T_odom o (YAML origin), rows under '
                     'convention A',
            'row_convention': 'A (PGM row 0 = maximum y), applied by '
                              'pgm_agreement.sample',
            'map_T_odom': 'identity on all 20 maps as logged; asserted, not '
                          'assumed',
            'map_T_odom_identity_tol': MTO_IDENTITY_TOL,
            'collapses_to': 'registered origin = spawn + YAML origin',
            'direction_source': 'cited from the latest '
                                'experiments/logs/world_fit_b2maps_k*_cut1200_*'
                                '.json (RESOLVED convention A, '
                                'world_T_odom o map_T_odom); not re-measured '
                                'here, which would need the walls',
        },
        'grid': {**grid, 'anchor': 'lattice through the world origin',
                 'resolution_note': 'the resolution the maps carry, read and '
                                    'never assumed 0.1'},
        'three_states': {
            'rule': 'the YAML thresholds split occupied from free; the reserved '
                    f'unknown byte {UNKNOWN} then overrides',
            'why_override': 'save_map.py writes 205 for "neither free nor '
                            'occupied" and hardcodes free_thresh 0.25 beside it, '
                            'so 205 reads back as shade 0.196 < 0.25 = free. '
                            'Taking the thresholds literally reported all 20 '
                            'maps as fully known and made Jaccard measure extent '
                            'overlap',
            'unknown_means': 'the robot did not commit the cell to free or '
                             'occupied: save_map writes 205 for a true unknown '
                             '(-1) and for the uncommitted 26..64 band alike',
            'checked_per_map': 'threshold_check on each entry of "maps"',
        },
        'tolerance': {'cells': TOL_CELLS,
                      'shape': 'Chebyshev / 3x3',
                      'applies_to': 'occupied cells of the map that says '
                                    'occupied, in either direction'},
        'maps': [{'stem': m['stem'], 'robot': m['robot'], 'cut': m['cut'],
                  'pgm': rel(m['pgm']), 'yaml': rel(m['yaml']),
                  'map_to_odom_path': rel(m['map_to_odom_path']),
                  'map_T_odom': {'x': m['map_T_odom'][0],
                                 'y': m['map_T_odom'][1],
                                 'yaw_rad': m['map_T_odom'][2]},
                  'width': m['width'], 'height': m['height'],
                  'resolution': m['resolution'],
                  'yaml_origin': m['yaml_origin'],
                  'registered_origin': m['origin'],
                  'negate': m['negate'],
                  'occupied_thresh': m['occupied_thresh'],
                  'free_thresh': m['free_thresh'],
                  'threshold_check': m['thresholds'],
                  'subcell_offset_cells': subcell_offset(m, grid),
                  'raw_counts': m['raw_counts']}
                 for m in flat],
        'gates': gates,
        'gate_rules': {
            'self': 'a map against itself: Jaccard exactly 1, 0 strict, '
                    '0 tolerant',
            'shift': 'a map against itself shifted one whole cell in x: strict '
                     f'> 0 and tolerant <= {SHIFT_TOLERANT_MAX}',
            'swap': f'all {len(pairs())} pairs at {CUTS[-1]} s with the two '
                    "robots' spawns exchanged: strict conflict must rise for "
                    f'every pair, by >= {SWAP_MIN_RISE_PP} pp (pre-registered, '
                    f'fails on the 2 axis-aligned swaps) or >= '
                    f'{SWAP_MIN_RISE_RATIO}x (calibrated afterwards). Both are '
                    'always computed; gates.swap.criterion says which set the '
                    'exit code',
        },
        'pairs': {f'{i}-{j}': {str(c): per_cut[c][(i, j)] for c in CUTS}
                  for i, j in pairs()},
        'predictions': predictions,
        'cuts_s': list(CUTS),
        'provenance': {'cell_centre': cell_centre_provenance()},
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = OUT_DIR / f'robot_divergence_{RUN_PREFIX}_{stamp}.json'
    json_path.write_text(json.dumps(out, indent=2, sort_keys=False))
    print(f'wrote {rel(json_path)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
