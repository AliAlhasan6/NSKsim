#!/usr/bin/env python3
"""B2 divergence on walls: sort A's wall faces by B's evidence.

docs/specs/SPEC_b2_divergence_walls.md v0.4, §3-§6. For an ordered pair (A, B)
and a cut C, every face element of A is corroborated or not corroborated. The
measure uses no ground truth beyond the poses that built the maps.

T2 -- the contradiction test -- left the measure in v0.4 under §7's fallback,
after C0 failed: 0/20 maps under v0.1, 4/20 under v0.3, against a floor of 1 %.
It is kept whole behind `t2`, which defaults to off, because the deep tail it
exposed is unexplained (O5) and the record is worth more than the deletion.

WHAT THIS FILE OWNS, AND WHAT IT BORROWS. It owns §4: the three tests in order,
the supercover walk and the element bookkeeping. Everything underneath is
borrowed and must not be rewritten here:

    graph_walls.face_elements          Def 1, the elements themselves
    graph_walls.graph_and_elements     the map, its segments and seg_of
    graph_walls.bag_variant_of         which bag a stem's rays come from
    diagnose_karto.bag_path_for        that name, built
    diagnose_karto.rebuild             the admitted ray set, by Karto's rules
    trinary_map.CELL_CENTRE_OFFSET     where a cell centre sits

The one piece of geometry that is NOT borrowed is the walk.
diagnose_karto.trace_rays is Karto's Bresenham line and steps diagonally past a
corner, which is correct for reproducing Karto's counters and wrong for this
question: a cell Bresenham skips is a crossing this file would miss, and the
element in it would read as unobserved. §4 therefore calls for a supercover
traversal, which is what `supercover` below is.

NO MODULE-SCOPE FILE READS. Importing this costs numpy, PyYAML and graph_walls.
diagnose_karto is imported inside the driver functions only, the way
diagnose_karto.rebuild function-scopes fit_world_transform, so that a test
machine with no bags and no turtlebot3 can still import this module.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import graph_walls as gw                                      # noqa: E402
from trinary_map import CELL_CENTRE_OFFSET, cell_index        # noqa: E402

OUT_DIR_DEFAULT = gw.LOGS_DIR / 'divergence'

# ── §4's constants ───────────────────────────────────────────────────────────
THETA_G_DEG = 30.0                 # the primary grazing bound
THETA_G_SWEEP = (15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0)   # §6's last row
MIN_RUNNING_ON = 2                 # condition 4's floor (v0.3)
T1_TOL_M = 1e-9                    # both T1 tolerances are inclusive (v0.5)
G1_TOL_M = 0.15                    # §7 G1
SEGMENT_GUARD_TOL_M = 1e-6         # G2: saved vs recomputed segment ends
CUTS = (60, 120, 240, 1200)
CORPUS_VARIANT = '_gated_extfix'   # the corpus S3 and S4 run on
C0_LIMIT = 0.01                    # §7 C0, 1 % of L_A

# The three classes, in the order the tests run (§4).
CORROBORATED, CONTRADICTED, UNOBSERVED, NOT_CORROBORATED = 0, 1, 2, 3
CLASS_NAME = {CORROBORATED: 'corroborated',
              CONTRADICTED: 'contradicted',
              UNOBSERVED: 'unobserved',
              NOT_CORROBORATED: 'not_corroborated'}

# v0.4: the measure is T1 and its complement. T2 failed C0 -- 0/20 maps under
# v0.1 and 4/20 under v0.3, against a 1 % floor -- and §7's fallback takes it
# out of the measure rather than shipping a contradiction count the self-test
# says is mostly the test firing on itself. It is kept whole, behind this flag,
# because the deep tail it exposed is unexplained (O5) and the record is worth
# more than the deletion.
T2_DEFAULT = False
MEASURE_CLASSES = (CORROBORATED, NOT_CORROBORATED)      # v0.4, flag off
T2_CLASSES = (CORROBORATED, CONTRADICTED, UNOBSERVED)   # v0.1-v0.3, flag on


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'divergence_walls: {msg}', file=sys.stderr)
    sys.exit(2)


# ───────────────────────────── the common frame ──────────────────────────────

def to_frame_a(x: np.ndarray, y: np.ndarray,
               spawn_b: tuple[float, float], spawn_a: tuple[float, float],
               flip: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """§2: a point in B's map frame into A's, `x_A = x_B + s_B - s_A`.

    Translation only, yaw 0, given by the spawn registration at 08617b2 and never
    fitted. Each map frame is anchored at its robot's spawn -- registered origin
    is spawn + YAML origin (robot_divergence.py:314) and these coordinates
    already carry the YAML origin -- so the world placement of any point is
    + spawn, and the composition between two such frames is the difference.

    `flip` is break X1, `x_B - s_B + s_A`. It is a parameter rather than an edit
    so that the test of the break runs this same function.
    """
    sign = -1.0 if flip else 1.0
    return (x + sign * (spawn_b[0] - spawn_a[0]),
            y + sign * (spawn_b[1] - spawn_a[1]))


def translate_elements(el: dict, dx: float, dy: float) -> dict:
    """An element table moved by (dx, dy). Normals and dirs are untouched.

    A pure translation cannot turn an axis-aligned normal, which is why T1 is a
    `dir` equality and not an angle (§4).
    """
    out = dict(el)
    out['x'] = el['x'] + dx
    out['y'] = el['y'] + dy
    return out


def g1_frame_check(parked_in_b: np.ndarray, spawn_b: tuple[float, float],
                   spawn_a: tuple[float, float], expected_in_a: np.ndarray,
                   tol: float = G1_TOL_M, flip: bool = False) -> dict:
    """§7 G1: every parked robot of B's map lands on its spawn in A's frame.

    `parked_in_b` and `expected_in_a` are (n, 2). The check is the one thing that
    catches a wrong sign or a wrong anchor before any share is read, which is
    exactly what X1 breaks.
    """
    if parked_in_b.size == 0:
        return {'holds': True, 'n': 0, 'max_error_m': 0.0, 'tol_m': tol}
    px, py = to_frame_a(parked_in_b[:, 0], parked_in_b[:, 1],
                        spawn_b, spawn_a, flip=flip)
    err = np.hypot(px - expected_in_a[:, 0], py - expected_in_a[:, 1])
    return {'holds': bool(np.all(err <= tol)), 'n': int(err.size),
            'max_error_m': float(err.max()), 'tol_m': tol}


# ──────────────────────────── elements and cells ─────────────────────────────

def free_cell_of(el: dict, shape: tuple[int, int]) -> np.ndarray:
    """Each element's FREE cell `f`, as a flat index, or -1 if off the grid.

    An element is the boundary between occupied `o` at (row, col) and the free
    neighbour one step along its dir (graph_walls.DIRS). Keying on `f` rather
    than on `o` is what makes the walk a lookup: a ray arriving from the free
    side is in `f` in the step just before it crosses.
    """
    h, w = shape
    if el['n'] == 0:
        return np.empty(0, dtype=np.int64)
    drow = np.array([d[0] for d in gw.DIRS], dtype=np.int64)
    dcol = np.array([d[1] for d in gw.DIRS], dtype=np.int64)
    fr = el['row'] + drow[el['dir']]
    fc = el['col'] + dcol[el['dir']]
    ok = (fr >= 0) & (fr < h) & (fc >= 0) & (fc < w)
    return np.where(ok, fr * w + fc, -1)


def element_tangent(el: dict) -> tuple[np.ndarray, np.ndarray]:
    """The in-face direction `t = (-ny, nx)`, so the segment is `m +- rho/2 * t`."""
    return -el['ny'], el['nx']


def corroborate(el_a: dict, el_b: dict, eps: float,
                rho: float) -> np.ndarray:
    """T1. Which of A's elements have a matching element of B.

    A match has the SAME dir -- element normals are axis-aligned by construction
    (graph_walls.DIRS) and the frame change is a pure translation, so the §4
    angle condition is an equality and is tested as one -- and a midpoint within
    TWO tolerances, split across and along the face (v0.5):

        |(m' - m) . n| <= eps      across, eps = rho(1 + sqrt2/2) = 0.1707 m
        |(m' - m) . t| <= rho/2    along,  t = (-n_y, n_x), rho/2 = 0.05 m

    both inclusive, with a 1e-9 m tolerance. A single Euclidean eps reached the
    NEXT element along the face, rho = 0.1 m away, so corroboration ran one
    element past the end of B's coverage; the across tolerance is unchanged and
    is the one the t1_across diagnostic measured. B's elements must already be
    in A's frame.

    Bucketed on an eps-sized lattice so the test is local: both tolerances are
    at most eps, so a match can only lie in the 3x3 block of buckets around A's
    own. Only elements of B's FACES are passed in by the caller (D1); B's
    unclassified elements, its parked robots among them, are not B's evidence of
    a wall.
    """
    out = np.zeros(el_a['n'], dtype=bool)
    if el_a['n'] == 0 or el_b['n'] == 0:
        return out

    bx = np.floor(el_b['x'] / eps).astype(np.int64)
    by = np.floor(el_b['y'] / eps).astype(np.int64)
    table: dict[tuple[int, int, int], list[int]] = {}
    for i in range(el_b['n']):
        table.setdefault((int(el_b['dir'][i]), int(bx[i]), int(by[i])),
                         []).append(i)

    ax = np.floor(el_a['x'] / eps).astype(np.int64)
    ay = np.floor(el_a['y'] / eps).astype(np.int64)
    half = 0.5 * rho                      # the along tolerance, v0.5
    for i in range(el_a['n']):
        d = int(el_a['dir'][i])
        xi, yi = el_a['x'][i], el_a['y'][i]
        nx, ny = el_a['nx'][i], el_a['ny'][i]
        tx, ty = -ny, nx
        hit = False
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                for j in table.get((d, int(ax[i]) + ddx, int(ay[i]) + ddy), ()):
                    dx = el_b['x'][j] - xi
                    dy = el_b['y'][j] - yi
                    if (abs(dx * nx + dy * ny) <= eps + T1_TOL_M
                            and abs(dx * tx + dy * ty) <= half + T1_TOL_M):
                        hit = True
                        break
                if hit:
                    break
            if hit:
                break
        out[i] = hit
    return out


def circular_emd_deg(p, q) -> float | None:
    """W1 between two bearing histograms on the circle, in degrees (O1).

    Raw counts in, each normalised by its own sum, so the result is scale free.
    `D_k = sum_{j<=k} (p_j - q_j)`; `W1 = bin * sum_k |D_k - a|` with `a` a
    median of `{D_k}`. Subtracting the median is what makes it circular rather
    than a distance on a cut line: it chooses the split point that costs least,
    so the mass may travel either way round.

    Returns None when either histogram sums to zero -- the element is then
    unweighable, which is a reported category and not a zero weight.

    The bin width comes from graph_walls.BEARING_BINS, so a change to D4's bin
    count moves this with it rather than leaving a literal behind.
    """
    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    if p.shape != (gw.BEARING_BINS,) or q.shape != (gw.BEARING_BINS,):
        raise ValueError(f'bearing histograms must be {gw.BEARING_BINS} bins, '
                         f'got {p.shape} and {q.shape}')
    sp, sq = p.sum(), q.sum()
    if sp <= 0.0 or sq <= 0.0:
        return None
    d = np.cumsum(p / sp - q / sq)
    return float((360.0 / gw.BEARING_BINS) * np.abs(d - np.median(d)).sum())


def corroborate_matches(el_a: dict, el_b: dict, eps: float,
                        rho: float) -> tuple[np.ndarray, np.ndarray]:
    """T1, and WHICH B element matched. O1 needs the matched element's segment.

    `corroborate` answers only whether a match exists, which is all the measure
    needs; the viewpoint weight needs the match itself. Where several of B's
    elements qualify, the one taken is the nearest ACROSS the face, then the
    nearest along it, then the one on the lowest B segment id -- a total order,
    so the choice does not depend on the order B's elements happen to be in.

    Returns (hit, match), `match` being -1 where there is no match.
    """
    hit = np.zeros(el_a['n'], dtype=bool)
    match = np.full(el_a['n'], -1, dtype=np.int64)
    if el_a['n'] == 0 or el_b['n'] == 0:
        return hit, match
    seg_b = el_b.get('seg')
    half = 0.5 * rho

    bx = np.floor(el_b['x'] / eps).astype(np.int64)
    by = np.floor(el_b['y'] / eps).astype(np.int64)
    table: dict[tuple[int, int, int], list[int]] = {}
    for i in range(el_b['n']):
        table.setdefault((int(el_b['dir'][i]), int(bx[i]), int(by[i])),
                         []).append(i)

    ax = np.floor(el_a['x'] / eps).astype(np.int64)
    ay = np.floor(el_a['y'] / eps).astype(np.int64)
    for i in range(el_a['n']):
        d = int(el_a['dir'][i])
        xi, yi = el_a['x'][i], el_a['y'][i]
        nx, ny = el_a['nx'][i], el_a['ny'][i]
        tx, ty = -ny, nx
        best = None
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                for j in table.get((d, int(ax[i]) + ddx, int(ay[i]) + ddy), ()):
                    dx = el_b['x'][j] - xi
                    dy = el_b['y'][j] - yi
                    across = abs(dx * nx + dy * ny)
                    along = abs(dx * tx + dy * ty)
                    if across <= eps + T1_TOL_M and along <= half + T1_TOL_M:
                        key = (across, along,
                               int(seg_b[j]) if seg_b is not None else 0, j)
                        if best is None or key < best:
                            best = key
        if best is not None:
            hit[i] = True
            match[i] = best[3]
    return hit, match


def viewpoint_weights(seg_of_a: np.ndarray, el_b: dict, corr: np.ndarray,
                      match: np.ndarray, hist_a, hist_b) -> dict:
    """O1's per-element weight, and the unweighable categories.

    `hist_a` and `hist_b` are per-segment bearing histograms, indexed by segment
    id. A corroborated element with an empty histogram on either side is not
    weighed zero -- zero means "seen from the same directions" and would be a
    claim nobody made. It is counted unweighable instead, on whichever side was
    empty, and left out of the mean and the percentiles.
    """
    n = seg_of_a.size
    w = np.full(n, np.nan)
    seg_b_of = np.full(n, -1, dtype=np.int64)
    un_a = np.zeros(n, dtype=bool)
    un_b = np.zeros(n, dtype=bool)
    seg_b = el_b.get('seg') if el_b else None
    for i in np.nonzero(corr)[0]:
        sa = int(seg_of_a[i])
        j = int(match[i])
        sb = int(seg_b[j]) if (seg_b is not None and j >= 0) else -1
        if sa < 0 or sa >= len(hist_a):
            un_a[i] = True
            continue
        if sb < 0 or sb >= len(hist_b):
            un_b[i] = True
            continue
        seg_b_of[i] = sb
        val = circular_emd_deg(hist_a[sa], hist_b[sb])
        if val is None:
            if np.sum(hist_a[sa]) <= 0:
                un_a[i] = True
            else:
                un_b[i] = True
            continue
        w[i] = val / 180.0
    return {'w': w, 'seg_b_of': seg_b_of,
            'unweighable_A': un_a, 'unweighable_B': un_b}


def segment_dirs(seg_of: np.ndarray, el: dict, segments: list) -> np.ndarray:
    """The `dir` of each segment, as P7 defines it (spec v0.9).

    "A segment's direction is the T1 element dir of the elements assigned to it,
    one of the four axis-aligned normals; if they differ, the commonest, ties
    going to the dir nearest the segment's fitted normal."

    The tie rule is what makes this total. `bincount().argmax()` would break a
    tie by dir index, which is an accident of the order DIRS happens to be
    written in; the fitted normal is a property of the face. Nearest means the
    largest dot product with it. A segment with no elements is -1, and one with
    no fitted normal falls back to the lowest tied dir, which cannot happen on a
    real segment but keeps the function total.
    """
    out = np.full(len(segments), -1, dtype=np.int64)
    dir_nx = np.array([d[2] for d in gw.DIRS], dtype=float)
    dir_ny = np.array([d[3] for d in gw.DIRS], dtype=float)
    for s in range(len(segments)):
        d = el['dir'][seg_of == s].astype(np.int64)
        if not d.size:
            continue
        counts = np.bincount(d, minlength=len(gw.DIRS))
        tied = np.nonzero(counts == counts.max())[0]
        if tied.size == 1:
            out[s] = int(tied[0])
            continue
        normal = segments[s].get('normal')
        if normal is None:
            out[s] = int(tied.min())
        else:
            dots = (dir_nx[tied] * float(normal[0])
                    + dir_ny[tied] * float(normal[1]))
            out[s] = int(tied[int(np.argmax(dots))])
    return out


def null_weights(seg_of_a: np.ndarray, weights: dict, hist_a, hist_b,
                 dirs_b: np.ndarray) -> np.ndarray:
    """P7's null: `w` measured against B's OTHER same-direction segments.

    For a weighable corroborated element, the mean of `W1(p, q_S) / 180°` over
    every B segment S with the same dir as S_B, not S_B itself, and a non-empty
    histogram. NaN where no such S exists -- that element is outside E7, which
    is a different thing from a null of zero.

    Memoised on (S_A, S_B): the answer depends on the pair of segments and not
    on which of their elements asked.
    """
    n = seg_of_a.size
    out = np.full(n, np.nan)
    cache: dict[tuple[int, int], float] = {}
    w = weights['w']
    seg_b_of = weights['seg_b_of']
    for i in np.nonzero(~np.isnan(w))[0]:
        sa, sb = int(seg_of_a[i]), int(seg_b_of[i])
        if sb < 0:
            continue
        key = (sa, sb)
        if key not in cache:
            vals = []
            for s in range(len(hist_b)):
                if s == sb or dirs_b[s] != dirs_b[sb]:
                    continue
                if np.sum(hist_b[s]) <= 0:
                    continue
                v = circular_emd_deg(hist_a[sa], hist_b[s])
                if v is not None:
                    vals.append(v / 180.0)
            cache[key] = float(np.mean(vals)) if vals else float('nan')
        out[i] = cache[key]
    return out


# ─────────────────────────────── the walk ────────────────────────────────────

def supercover(ox: np.ndarray, oy: np.ndarray, dx: np.ndarray, dy: np.ndarray,
               length: np.ndarray, rho: float, origin: tuple[float, float],
               shape: tuple[int, int], tol: float = 1e-9
               ) -> tuple[np.ndarray, np.ndarray]:
    """Every cell whose square the segment `[o, o + length*d]` touches.

    Amanatides-Woo, in the lattice's own units. With
    trinary_map.CELL_CENTRE_OFFSET = 0.0 a cell CENTRE sits at `origin + i*rho`,
    so cell `i` spans `[origin + (i-0.5)*rho, origin + (i+0.5)*rho)` and the
    substitution `u = (x - origin)/rho + 0.5` makes the lattice the unit integer
    grid: the cell index is `floor(u)` and the boundaries are the integers.

    Supercover, not Bresenham. diagnose_karto.trace_rays is Karto's line, and
    Karto's stepping passes a corner diagonally -- correct there, wrong here,
    because a skipped cell is a crossing this file never tests and an element
    that would have been contradicted reads as unobserved instead. Where two
    boundary crossings fall within `tol` of each other the segment is through a
    corner and BOTH diagonal neighbours are emitted.

    Over-approximating is safe: every candidate then takes the exact tests of
    `ray_crossings`. Under-approximating is the bug, so the tie branch errs long.

    Returns (flat cell index, ray index), unsorted and possibly with repeats.
    """
    h, w = shape
    n = ox.size
    if n == 0:
        return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)

    u0 = (ox - origin[0]) / rho + 0.5 - CELL_CENTRE_OFFSET
    v0 = (oy - origin[1]) / rho + 0.5 - CELL_CENTRE_OFFSET
    u1 = u0 + length * dx / rho
    v1 = v0 + length * dy / rho

    ix0 = np.floor(u0).astype(np.int64)
    iy0 = np.floor(v0).astype(np.int64)
    nx = np.abs(np.floor(u1).astype(np.int64) - ix0)
    ny = np.abs(np.floor(v1).astype(np.int64) - iy0)
    stepx = np.where(u1 >= u0, 1, -1).astype(np.int64)
    stepy = np.where(v1 >= v0, 1, -1).astype(np.int64)

    def crossings(i0, n_cross, step, a0, a1):
        """The parameter t at each boundary this axis crosses, per ray."""
        total = int(n_cross.sum())
        if total == 0:
            return (np.empty(0, dtype=np.int64), np.empty(0),
                    np.empty(0, dtype=np.int64))
        ray = np.repeat(np.arange(n_cross.size), n_cross)
        starts = np.concatenate([[0], np.cumsum(n_cross)[:-1]])
        j = np.arange(total) - starts[ray]
        bound = np.where(step[ray] > 0, i0[ray] + 1 + j,
                         i0[ray] - j).astype(float)
        span = (a1 - a0)[ray]
        return ray, (bound - a0[ray]) / span, j

    rx, tx, _ = crossings(ix0, nx, stepx, u0, u1)
    ry, ty, _ = crossings(iy0, ny, stepy, v0, v1)

    ray_all = np.concatenate([rx, ry])
    t_all = np.concatenate([tx, ty])
    is_x_all = np.concatenate([np.ones(rx.size, dtype=bool),
                               np.zeros(ry.size, dtype=bool)])

    cells_ix = [ix0]
    cells_iy = [iy0]
    cells_ray = [np.arange(n)]

    if ray_all.size:
        order = np.lexsort((t_all, ray_all))
        ray_s, t_s, is_x = ray_all[order], t_all[order], is_x_all[order]

        # cumulative steps per axis WITHIN each ray: the global running count
        # minus the count standing when that ray's first crossing was reached.
        gx = np.cumsum(is_x)
        gy = np.cumsum(~is_x)
        n_cross = nx + ny
        first = np.concatenate([[0], np.cumsum(n_cross)[:-1]])[ray_s]
        cx_steps = gx - np.concatenate([[0], gx[:-1]])[first]
        cy_steps = gy - np.concatenate([[0], gy[:-1]])[first]

        cells_ix.append(ix0[ray_s] + stepx[ray_s] * cx_steps)
        cells_iy.append(iy0[ray_s] + stepy[ray_s] * cy_steps)
        cells_ray.append(ray_s)

        # through a corner: emit the cell the other step order would have given
        if ray_s.size > 1:
            tie = ((ray_s[:-1] == ray_s[1:])
                   & (np.abs(t_s[1:] - t_s[:-1]) <= tol)
                   & (is_x[:-1] != is_x[1:]))
            p = np.nonzero(tie)[0]
            if p.size:
                r = ray_s[p]
                bx = ix0[r] + stepx[r] * (cx_steps[p] - is_x[p])
                by = iy0[r] + stepy[r] * (cy_steps[p] - ~is_x[p])
                cells_ix.append(bx + stepx[r] * is_x[p + 1])
                cells_iy.append(by + stepy[r] * ~is_x[p + 1])
                cells_ray.append(r)

    all_ix = np.concatenate(cells_ix)
    all_iy = np.concatenate(cells_iy)
    all_ray = np.concatenate(cells_ray)
    inside = (all_ix >= 0) & (all_ix < w) & (all_iy >= 0) & (all_iy < h)
    return (all_iy[inside] * w + all_ix[inside]), all_ray[inside]


# ──────────────────────────────── T2 ─────────────────────────────────────────

def ray_crossings(rays: dict, el: dict, free_flat: np.ndarray, rho: float,
                  origin: tuple[float, float], shape: tuple[int, int],
                  theta_g_deg: float, eps: float,
                  chunk_rays: int = 40000) -> dict:
    """T2's three conditions, for every (ray, element) the walk makes a candidate.

    `rays` holds B's admitted rays ALREADY IN A'S FRAME: origin (ox, oy), unit
    direction (dx, dy) and `free_len`, the length Karto clears along it -- the
    return range for a finite beam, the relay ceiling for a filled one. The walk
    is through A's lattice, which is why this is redone per ordered pair.

    The conditions of §4, on the exact geometry and never on the walk:

      1. crossing   `d . n < 0` and the hit lies within the closed element,
                    `|w| <= rho/2`, at `s >= 0`;
      2. not grazing `|d . n| >= sin theta_g`;
      3. runs on     `(free_len - s) * |d . n| >= eps`, a DEPTH behind the face
                     line and not a distance along the ray (v0.2). `free_len` is
                     `R` for a beam with a return and `L` for a relay-filled one,
                     so one expression covers both of §4's cases.

    Condition 4 is not here: it is a statement about the rays at an element taken
    together, so it belongs to `classify`. What this returns is the two counts it
    needs -- `n12`, the rays meeting 1 and 2, and `n123`, those also meeting 3 --
    plus the angle §6 reports.
    """
    n_el = el['n']
    n12 = np.zeros(n_el, dtype=np.int64)
    n123 = np.zeros(n_el, dtype=np.int64)
    best_cos = np.zeros(n_el)
    if n_el == 0 or rays['ox'].size == 0:
        return {'n12': n12, 'n123': n123, 'max_abs_dn': best_cos}

    # elements indexed by their free cell, so a visited cell is a lookup
    h, w = shape
    valid = free_flat >= 0
    el_idx = np.nonzero(valid)[0]
    order = np.argsort(free_flat[el_idx], kind='stable')
    el_sorted = el_idx[order]
    cell_sorted = free_flat[el_sorted]
    cell_start = np.searchsorted(cell_sorted, np.arange(h * w), side='left')
    cell_end = np.searchsorted(cell_sorted, np.arange(h * w), side='right')

    sin_g = math.sin(math.radians(theta_g_deg))
    nx_e, ny_e = el['nx'], el['ny']
    tx_e, ty_e = element_tangent(el)
    half = 0.5 * rho

    n_rays = rays['ox'].size
    for lo in range(0, n_rays, chunk_rays):
        sl = slice(lo, min(lo + chunk_rays, n_rays))
        ox, oy = rays['ox'][sl], rays['oy'][sl]
        dx, dy = rays['dx'][sl], rays['dy'][sl]
        flen = rays['free_len'][sl]

        flat, ray = supercover(ox, oy, dx, dy, flen, rho, origin, shape)
        if flat.size == 0:
            continue
        per = cell_end[flat] - cell_start[flat]
        keep = per > 0
        if not keep.any():
            continue
        flat, ray, per = flat[keep], ray[keep], per[keep]

        pair_ray = np.repeat(ray, per)
        starts = cell_start[flat]
        offs = np.arange(per.sum()) - np.repeat(
            np.concatenate([[0], np.cumsum(per)[:-1]]), per)
        pair_el = el_sorted[np.repeat(starts, per) + offs]

        # one (ray, element) candidate at most, however many cells agreed
        pair = np.unique(np.stack([pair_ray, pair_el]), axis=1)
        pr, pe = pair[0], pair[1]

        d_n = dx[pr] * nx_e[pe] + dy[pr] * ny_e[pe]
        approaching = d_n < 0.0
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (((el['x'][pe] - ox[pr]) * nx_e[pe]
                  + (el['y'][pe] - oy[pr]) * ny_e[pe])
                 / np.where(approaching, d_n, 1.0))
        hx = ox[pr] + s * dx[pr]
        hy = oy[pr] + s * dy[pr]
        wv = (hx - el['x'][pe]) * tx_e[pe] + (hy - el['y'][pe]) * ty_e[pe]

        # `s <= free_len` is part of condition 1: a ray that stopped inside the
        # free cell never crossed the element. Under v0.1 condition 3 implied it
        # (`free_len >= s + eps`), so it was never written down; condition 4
        # makes `n12` an observable in its own right, and without this a ray
        # that stopped short would be counted as a crossing that declined to
        # run on -- padding the denominator of the majority.
        ok12 = (approaching
                & (s >= 0.0)
                & (s <= flen[pr] + 1e-12)
                & (np.abs(wv) <= half + 1e-12)
                & (np.abs(d_n) >= sin_g - 1e-12))
        if not ok12.any():
            continue
        # condition 3 (v0.2): depth behind the face line, not range along the ray
        depth = (flen[pr] - s) * np.abs(d_n)
        ok123 = ok12 & (depth >= eps - 1e-12)
        n12 += np.bincount(pe[ok12], minlength=n_el)
        if ok123.any():
            n123 += np.bincount(pe[ok123], minlength=n_el)
            np.maximum.at(best_cos, pe[ok123], np.abs(d_n[ok123]))

    return {'n12': n12, 'n123': n123, 'max_abs_dn': best_cos}


# ─────────────────────────────── §4 in order ─────────────────────────────────

def weight_of_evidence(n12: np.ndarray, n123: np.ndarray) -> np.ndarray:
    """§4 condition 4 (v0.3): is the running-on evidence worth believing?

    Two clauses, and both must hold at the element. At least MIN_RUNNING_ON of
    B's rays meet conditions 1-3, and they are MORE THAN HALF of the rays that
    met 1 and 2 there. One ray running on is not evidence that B saw open space:
    the deep-tail diagnostic found 4-10 % of crossings on a robot's OWN map
    running on by metres, for a reason still unexplained (O5).

    The majority is strict, so 2 of 4 does not contradict and 2 of 3 does.
    """
    return (n123 >= MIN_RUNNING_ON) & (2 * n123 > n12)


def classify(el_a: dict, seg_of_a: np.ndarray, el_b_in_a: dict, rays_in_a: dict,
             rho: float, origin: tuple[float, float], shape: tuple[int, int],
             *, eps: float | None = None, theta_g_deg: float = THETA_G_DEG,
             skip_t1: bool = False, t2: bool = T2_DEFAULT,
             hist_a=None, hist_b=None) -> dict:
    """§4. With `t2` off (the v0.4 default) the measure is T1 and its complement.

    Only elements assigned to a face of A are classified (§3); the rest are
    counted as excluded length and never appear in a share.

    `t2` on restores v0.1-v0.3: T1, then T2, then T3, an element taking the
    first that applies. It is what C0 and the §7 breaks exercise, and it is the
    only path that walks the rays -- so with the flag off nothing here reads
    `rays_in_a` at all, and a caller that wants only the measure need not build
    a ray set. `skip_t1` is C0's switch, which applies T2 alone; it implies
    `t2`, because T2 alone is the only thing left to apply.
    """
    eps = gw.EPS_FACTOR * rho if eps is None else eps
    t2 = bool(t2 or skip_t1)

    weights = None
    if skip_t1:
        corr = np.zeros(el_a['n'], dtype=bool)
    elif hist_a is None or hist_b is None:
        corr = corroborate(el_a, el_b_in_a, eps, rho)
    else:
        # O1 needs the matched element, not just the fact of a match. The mask
        # is the same either way; corroborate_matches only also says which.
        corr, match = corroborate_matches(el_a, el_b_in_a, eps, rho)
        weights = viewpoint_weights(seg_of_a, el_b_in_a, corr, match,
                                    hist_a, hist_b)
    out = {
        'assigned': seg_of_a >= 0,
        'seg_of': seg_of_a,
        'x': el_a['x'], 'y': el_a['y'],
        'corroborated': corr,
        'theta_g_deg': theta_g_deg,
        'eps': eps,
        'rho': rho,
        't2': t2,
        'weights': weights,
    }
    if not t2:
        zero = np.zeros(el_a['n'], dtype=np.int64)
        out.update({
            'cls': np.where(corr, CORROBORATED, NOT_CORROBORATED),
            'contradicted': np.zeros(el_a['n'], dtype=bool),
            'n12': zero, 'n123': zero,
            'max_abs_dn': np.zeros(el_a['n']),
        })
        return out

    cross = ray_crossings(rays_in_a, el_a, free_cell_of(el_a, shape), rho,
                          origin, shape, theta_g_deg, eps)
    contra = (~corr) & weight_of_evidence(cross['n12'], cross['n123'])
    out.update({
        'cls': np.where(corr, CORROBORATED,
                        np.where(contra, CONTRADICTED, UNOBSERVED)),
        'contradicted': contra,
        'n12': cross['n12'], 'n123': cross['n123'],
        'max_abs_dn': cross['max_abs_dn'],
    })
    return out


def deep_share(result: dict) -> float:
    """§6: crossings meeting conditions 1-3 over those meeting 1 and 2.

    A T2 output, so it is produced only with the flag on (v0.4) and is 0.0
    otherwise. Within a T2 run it is reported whatever condition 4 decides,
    because it is the quantity O5 is about.
    """
    d = int(result['n12'].sum())
    return (int(result['n123'].sum()) / d) if d else 0.0


def classes_of(result: dict) -> tuple:
    """Which classes this result uses: two under v0.4, three with T2 on."""
    return T2_CLASSES if result.get('t2') else MEASURE_CLASSES


def shares(result: dict) -> dict:
    """§6: L_A, L_excl and the shares, which sum to 1.

    Two shares under v0.4 -- corroborated and not corroborated -- and the three
    of v0.1-v0.3 when T2 is on. Every element is `rho` long, so a share by
    length is a share by count.
    """
    rho = result['rho']
    assigned = result['assigned']
    cls = result['cls'][assigned]
    n = int(cls.size)
    out = {
        'L_A': float(n * rho),
        'L_excl': float(int((~assigned).sum()) * rho),
        'n_elements': n,
    }
    for code in classes_of(result):
        name = CLASS_NAME[code]
        c = int((cls == code).sum())
        out[f'n_{name}'] = c
        out[f'{name}'] = (c / n) if n else 0.0
    out.update(viewpoint_fields(result, n))
    return out


def viewpoint_fields(result: dict, n: int) -> dict:
    """O1's per-row fields, or {} when no histograms were supplied.

    `W` is `sum(w) / n` with `n` the corroborated share's own denominator, which
    is the same number as `sum(w * rho) / L_A`: every element is rho long. So
    `W <= C`, unweighable elements contributing 0 and making `W` a lower bound.
    """
    wts = result.get('weights')
    if wts is None:
        return {}
    keep = result['assigned']
    w = wts['w'][keep]
    good = ~np.isnan(w)
    wv = w[good]
    out = {
        'W': float(wv.sum() / n) if n else 0.0,
        'n_weighable': int(good.sum()),
        'unweighable_A': (float(wts['unweighable_A'][keep].sum()) / n
                          if n else 0.0),
        'unweighable_B': (float(wts['unweighable_B'][keep].sum()) / n
                          if n else 0.0),
    }
    for key, val in (('w_mean', float(wv.mean()) if wv.size else None),
                     ('w_p10', float(np.percentile(wv, 10)) if wv.size else None),
                     ('w_p50', float(np.percentile(wv, 50)) if wv.size else None),
                     ('w_p90', float(np.percentile(wv, 90)) if wv.size else None)):
        out[key] = val
    return out


def per_face(result: dict, n_faces: int) -> list[dict]:
    """§6: face id and the per-class lengths, so a gap can be located."""
    rho = result['rho']
    cls, seg = result['cls'], result['seg_of']
    rows = []
    for fid in range(n_faces):
        m = seg == fid
        row = {'id': fid, 'length': float(int(m.sum()) * rho)}
        for code in classes_of(result):
            row[f'{CLASS_NAME[code]}_m'] = float(
                int((m & (cls == code)).sum()) * rho)
        rows.append(row)
    return rows


def contradicted_detail(result: dict, segments: list | None = None
                        ) -> list[dict]:
    """§6, per contradicted element: the two counts, the angle, the end distance.

    `n12` and `n123` are the rays meeting conditions 1 and 2, and those also
    meeting 3 -- the pair condition 4 is a statement about, so a reader can see
    why an element was or was not contradicted. The angle is the crossing angle
    to the face line, `asin |d . n|`, which is what `theta_g` bounds. The end
    distance needs A's segments and is omitted when the caller has none.
    """
    rows = []
    idx = np.nonzero(result['contradicted'] & result['assigned'])[0]
    for i in idx:
        dn = float(result['max_abs_dn'][i])
        row = {
            'element': int(i),
            'face': int(result['seg_of'][i]),
            'n12': int(result['n12'][i]),
            'n123': int(result['n123'][i]),
            'angle_deg': math.degrees(math.asin(min(1.0, abs(dn)))),
        }
        if segments is not None:
            s = segments[int(result['seg_of'][i])]
            px, py = float(result['x'][i]), float(result['y'][i])
            row['dist_to_face_end_m'] = min(
                math.hypot(px - s['p0'][0], py - s['p0'][1]),
                math.hypot(px - s['p1'][0], py - s['p1'][1]))
        rows.append(row)
    return rows


def theta_sweep(el_a: dict, seg_of_a: np.ndarray, el_b_in_a: dict,
                rays_in_a: dict, rho: float, origin, shape, *,
                eps: float | None = None,
                sweep: tuple = THETA_G_SWEEP) -> list[dict]:
    """§6's last row: the contradicted share at each `theta_g`.

    X5 says this is non-increasing. That followed from condition 3 alone, where
    a larger `theta_g` admits strictly fewer crossings and no element can gain
    one. It does NOT follow under condition 4: raising `theta_g` drops crossings
    from `n12` as well as `n123`, and dropping one that did not run on RAISES
    the ratio, so an element can cross the majority and become contradicted.
    2 of 4 is not a contradiction; drop two shallow crossings and 2 of 2 is.

    The sweep is therefore reported, and X5 is a claim to be tested rather than
    a property to be relied on.
    """
    out = []
    for tg in sweep:
        r = classify(el_a, seg_of_a, el_b_in_a, rays_in_a, rho, origin, shape,
                     eps=eps, theta_g_deg=tg, t2=True)
        out.append({'theta_g_deg': tg,
                    'contradicted': shares(r)['contradicted'],
                    'deep_share': deep_share(r)})
    return out


def c0_floor(el_a: dict, seg_of_a: np.ndarray, rays_a_in_a: dict, rho: float,
             origin, shape, *, eps: float | None = None,
             theta_g_deg: float = THETA_G_DEG) -> dict:
    """§7 C0: T2 alone, A's own rays against A's own faces.

    How often the contradiction test fires on a map against the rays that built
    it. This is the floor every cross-robot number sits on, and §7 stops the run
    if it exceeds 1 % of L_A on any map.
    """
    r = classify(el_a, seg_of_a, {'n': 0}, rays_a_in_a, rho, origin, shape,
                 eps=eps, theta_g_deg=theta_g_deg, skip_t1=True)
    s = shares(r)
    return {'share': s['contradicted'], 'n': s['n_contradicted'],
            'L_A': s['L_A'], 'holds': s['contradicted'] <= C0_LIMIT,
            'limit': C0_LIMIT, 'theta_g_deg': theta_g_deg}


# ──────────────────────────── the corpus driver ──────────────────────────────

def rays_in_frame(rays: dict, spawn_b, spawn_a, flip: bool = False) -> dict:
    """diagnose_karto.admitted_rays, moved into A's frame.

    Only the origin moves: the transform is a translation, so directions and
    free lengths are carried across untouched. The ray set itself is never
    re-derived here -- same scan admission, same poses, same sensor offset, same
    relay ceiling -- which is what §4 means by taking the rays from the tracer.
    """
    ox, oy = to_frame_a(rays['ox'], rays['oy'], spawn_b, spawn_a, flip)
    return {'ox': ox, 'oy': oy, 'dx': rays['dx'], 'dy': rays['dy'],
            'free_len': rays['free_len']}


def corpus_stems(variant: str = CORPUS_VARIANT) -> list[str]:
    """The 20 map stems S3 and S4 run on, built the variant-aware way.

    graph_walls.corpus_stems() gives the PLAIN corpus; this is the gated,
    extent-fixed one, and the variant token is a parameter rather than baked in
    so a different build can be surveyed without editing the caller.
    """
    return [f'{gw.RUN_PREFIX}_k{k}_cut{c}{variant}_robot{k}'
            for k in range(5) for c in CUTS]


def segment_guard(saved_segments, recomputed_segments,
                  tol: float = SEGMENT_GUARD_TOL_M) -> dict:
    """Do the JSON's segments and the recomputed ones line up, index for index?

    O1 reads `bearing_hist` off the SAVED json and everything else off the
    recompute, so segment i has to be the same face in both or the weight is
    taken from the wrong wall. Endpoints only -- this never looks inside `obs`.

    Returns a verdict rather than refusing, so a survey can measure the margin
    on a corpus that passes; `load_side` is what turns a failure into a refusal.
    """
    n_saved, n_recomputed = len(saved_segments), len(recomputed_segments)
    out = {'ok': False, 'n_saved': n_saved, 'n_recomputed': n_recomputed,
           'max_endpoint_diff_m': None, 'tol_m': tol, 'message': None}
    if n_saved != n_recomputed:
        out['message'] = (f'the JSON has {n_saved} segments and the recompute '
                          f'{n_recomputed}; bearing_hist cannot be matched to '
                          'a segment by index')
        return out
    worst, worst_i = 0.0, None
    for i, (sv, rc) in enumerate(zip(saved_segments, recomputed_segments)):
        for end in ('p0', 'p1'):
            d = math.hypot(sv[end][0] - rc[end][0], sv[end][1] - rc[end][1])
            if d > worst:
                worst, worst_i = d, i
    out['max_endpoint_diff_m'] = worst
    out['ok'] = worst <= tol
    if not out['ok']:
        out['message'] = (f'segment {worst_i} differs between the JSON and the '
                          f'recompute by {worst:.3e} m, over the {tol:.0e} m '
                          'tolerance, so bearing_hist would be read off the '
                          'wrong face')
    return out


def load_side(stem: str, *, strict: bool = True) -> dict:
    """One robot's map, elements and faces, with §3's two conditions enforced.

    Elements are RECOMPUTED, never read: the JSON stores segments only. `g` comes
    from the JSON's own thresholds.g rather than from the nav2 params, and the
    map's sha256 must equal the JSON's map_sha256 or this stops -- both of which
    are what make the recomputed face set the one the JSON reports.
    """
    out_json = gw.OUT_DIR_DEFAULT / f'{stem}.json'
    if not out_json.is_file():
        die(f'{stem}: no graph JSON at {out_json}; graph_walls writes it')
    saved = json.loads(out_json.read_text())

    g = saved['thresholds']['g']['value']
    params = gw.Params(g=g)
    graph, el, seg_of, grid = gw.graph_and_elements(
        stem, params=params, g_source=f'{out_json.name} thresholds.g')
    if graph['map_sha256'] != saved['map_sha256']:
        die(f'{stem}: map sha256 {graph["map_sha256"][:12]} is not the JSON\'s '
            f'{saved["map_sha256"][:12]}; the JSON describes another map')
    # O1's histograms come from the SAVED json, not the recompute: bearing_hist
    # is written by obs_for_map, which reads the bag, and graph_and_elements
    # does not. The two segment lists must therefore line up, and that is
    # checked rather than assumed -- same count, same endpoints.
    saved_segs = saved.get('segments', [])
    guard, hist = None, None
    if saved_segs and all('obs' in s and 'bearing_hist' in s['obs']
                          for s in saved_segs):
        guard = segment_guard(saved_segs, graph['segments'])
        if not guard['ok']:
            if strict:
                die(f'{stem}: {guard["message"]}')
        else:
            hist = [s['obs']['bearing_hist'] for s in saved_segs]

    return {'stem': stem, 'graph': graph, 'el': el, 'seg_of': seg_of,
            'bearing_hist': hist, 'segment_guard': guard,
            'grid': grid, 'rho': graph['resolution'],
            'origin': tuple(graph['origin']), 'shape': grid.shape,
            'segments': graph['segments']}


def face_elements_only(el: dict, seg_of: np.ndarray) -> dict:
    """B's elements restricted to its FACES, which is what T1 may match (D1)."""
    keep = seg_of >= 0
    out = {k: (v[keep] if isinstance(v, np.ndarray) else v)
           for k, v in el.items()}
    out['n'] = int(keep.sum())
    return out


def pair_row(a: dict, b: dict, spawn_a, spawn_b, cut_a: int, cut_b: int,
             rays: dict | None = None, *, flip: bool = False,
             theta_g_deg: float = THETA_G_DEG, t2: bool = T2_DEFAULT) -> dict:
    """The computation half of a pair: two loaded sides in, one row out.

    Reads no file and resolves no spawn, so it can be exercised on hand-built
    sides. `run_pair` is this function plus the corpus I/O.

    A AND B MAY BE AT DIFFERENT CUTS (§2). Nothing in the geometry cares: the
    frame change is a translation between two spawns, and the cut only decides
    which map and which rays each side brought. P3 is stated over A held at
    cut1200 while B's cut moves, so the pair has to allow it.

    EVERY row carries `cut_A` and `cut_B`, and no row carries a bare `cut`.
    A single `cut` was right when every pair was same-cut and is a trap now: it
    would be present on some rows and absent on others, so a reader could take
    it for granted on the same-cut series and silently get nothing on P3's.
    One shape for both is worth the broken key.
    """
    keep_b = b['seg_of'] >= 0
    bx, by = to_frame_a(b['el']['x'], b['el']['y'], spawn_b, spawn_a, flip=flip)
    el_b = dict(face_elements_only(b['el'], b['seg_of']))
    el_b['x'], el_b['y'] = bx[keep_b], by[keep_b]
    el_b['seg'] = b['seg_of'][keep_b]          # O1 needs the matched segment

    t0 = time.monotonic()
    res = classify(a['el'], a['seg_of'], el_b, rays, a['rho'], a['origin'],
                   a['shape'], theta_g_deg=theta_g_deg, t2=t2,
                   hist_a=a.get('bearing_hist'), hist_b=b.get('bearing_hist'))
    seconds = time.monotonic() - t0
    row = {
        'A': a['stem'], 'B': b['stem'],
        'cut_A': cut_a, 'cut_B': cut_b, 't2': bool(t2),
        'shares': shares(res),
        'per_face': per_face(res, len(a['segments'])),
        'seconds': seconds,
    }
    if t2:
        row.update({
            'contradicted': contradicted_detail(res, a['segments']),
            'deep_share': deep_share(res),
            'theta_sweep': theta_sweep(a['el'], a['seg_of'], el_b, rays,
                                       a['rho'], a['origin'], a['shape']),
        })
    return row


def run_pair(a_stem: str, b_stem: str, *, flip: bool = False,
             theta_g_deg: float = THETA_G_DEG, t2: bool = T2_DEFAULT) -> dict:
    """One ordered pair, end to end. Reads the corpus.

    A and B need not be at the same cut (§2). The refusal that used to stand
    here -- "a pair is one cut" -- was written when every comparison was
    same-cut; P3 holds A at cut1200 and walks B's cut, so it had to go. B's
    rays, when T2 asks for them, come from B's OWN cut.

    With `t2` off (the v0.4 default) the row carries the two measure classes and
    nothing else, and no ray set is built: the rays exist only for T2, and
    building them is the expensive half of a pair. With `t2` on the row also
    carries the T2 outputs §6 lists, unchanged from v0.3.
    """
    import diagnose_karto as dk                              # noqa: PLC0415
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    a = load_side(a_stem)
    b = load_side(b_stem)
    ka, cut_a = gw._k_and_cut(a_stem, 'the pair needs the robot and the cut')
    kb, cut_b = gw._k_and_cut(b_stem, 'the pair needs the robot and the cut')

    # Inside the function, not at import: this shells out to git. rd.SPAWN_REV
    # is 08617b2, the b2maps era -- bag_overlap's own default is the b16 ring
    # and would put every map 3.15 m away.
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    spawn_a, spawn_b = spawn[ka], spawn[kb]
    rays = (rays_in_frame(dk.admitted_rays(b_stem, kb, cut_b),
                          spawn_b, spawn_a, flip=flip) if t2 else None)
    return pair_row(a, b, spawn_a, spawn_b, cut_a, cut_b, rays,
                    flip=flip, theta_g_deg=theta_g_deg, t2=t2)


def write_result(payload: dict, out_dir: Path = OUT_DIR_DEFAULT) -> Path:
    """§6: a new, dated file every time. Nothing overwrites an earlier result."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'divergence_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))
    return path


def _git(*args) -> str:
    """Read-only git, for provenance. Empty string if git is not there."""
    import subprocess                                      # noqa: PLC0415
    try:
        return subprocess.run(['git', '-C', str(REPO_ROOT), *args],
                              capture_output=True, text=True,
                              timeout=20).stdout.strip()
    except Exception:
        return ''


def run_g2(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """G2: does the segment guard hold on the corpus, and by what margin.

    Survey only. It calls load_side and reads the verdict the guard already
    produced -- it never touches `obs`, never counts an empty histogram, and
    never calls the corroboration or weighting code. What it records per map is
    the verdict, the two segment counts, the largest endpoint difference and
    the tolerance that difference was judged against.

    `strict=False`, so a map whose guard fails is measured rather than fatal;
    anything else that refuses -- a missing JSON, a sha256 mismatch -- is caught
    per map and recorded, and the survey goes on to the next one.
    """
    import contextlib                                      # noqa: PLC0415
    import io                                              # noqa: PLC0415

    stems = corpus_stems(variant)
    rows, n_pass = [], 0
    for stem in stems:
        row = {'map': stem, 'pass': False, 'n_saved': None,
               'n_recomputed': None, 'max_endpoint_diff_m': None,
               'tol_m': SEGMENT_GUARD_TOL_M, 'error': None}
        err = io.StringIO()
        try:
            with contextlib.redirect_stderr(err):
                side = load_side(stem, strict=False)
        except SystemExit:
            row['error'] = err.getvalue().strip() or 'refused, no message'
        else:
            g = side.get('segment_guard')
            if g is None:
                row['error'] = 'the JSON carries no per-segment obs to guard'
            else:
                row.update({'pass': bool(g['ok']), 'n_saved': g['n_saved'],
                            'n_recomputed': g['n_recomputed'],
                            'max_endpoint_diff_m': g['max_endpoint_diff_m'],
                            'tol_m': g['tol_m'], 'error': g['message']})
        n_pass += bool(row['pass'])
        d = row['max_endpoint_diff_m']
        print(f'{stem:<42}{"PASS" if row["pass"] else "FAIL":>6}'
              f'{str(row["n_saved"]):>5}/{str(row["n_recomputed"]):<5}'
              f'{(f"{d:.3e}" if d is not None else "-"):>12} m'
              + (f'   {row["error"]}' if row['error'] else ''), flush=True)
    print(f'G2: {n_pass}/{len(stems)} pass')

    payload = {'stage': 'g2', 'head': _git('rev-parse', 'HEAD'),
               'tree_dirty': bool(_git('status', '--porcelain')),
               'variant': variant, 'tol_m': SEGMENT_GUARD_TOL_M,
               'n_pass': n_pass, 'of': len(stems), 'maps': rows,
               'note': 'segment endpoints only; no obs, no histogram contents, '
                       'no corroboration, no weights'}
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'g2_guard_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))
    print(f'written, overwriting nothing: {path}')
    return payload


P7_MIN_PAIRS, P8_MIN_PAIRS = 18, 16


def _side_classify(a: dict, b: dict, spawn_a, spawn_b):
    """classify for one ordered pair, exactly as pair_row builds it."""
    keep_b = b['seg_of'] >= 0
    bx, by = to_frame_a(b['el']['x'], b['el']['y'], spawn_b, spawn_a)
    el_b = dict(face_elements_only(b['el'], b['seg_of']))
    el_b['x'], el_b['y'] = bx[keep_b], by[keep_b]
    el_b['seg'] = b['seg_of'][keep_b]
    res = classify(a['el'], a['seg_of'], el_b, None, a['rho'], a['origin'],
                   a['shape'], hist_a=a.get('bearing_hist'),
                   hist_b=b.get('bearing_hist'))
    return res, shares(res)


def _reported(sh: dict, n_e7: int | None = None) -> dict:
    keys = ('W', 'w_mean', 'w_p10', 'w_p50', 'w_p90', 'unweighable_A',
            'unweighable_B', 'corroborated', 'n_corroborated', 'n_weighable')
    out = {k: sh.get(k) for k in keys}
    if n_e7 is not None:
        out['n_E7'] = n_e7
    return out


def run_s5(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """S5: G3, then P7 and P8, exactly as registered at spec v0.9.

    Refuses on a dirty tree. A registered prediction evaluated against code that
    is not in history is not evidence about that code, and the result file would
    name a commit it was not produced by.
    """
    import fit_world_transform as fwt                      # noqa: PLC0415
    import robot_divergence as rd                          # noqa: PLC0415

    dirty = [ln for ln in _git('status', '--porcelain').splitlines()
             if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    sides = {s: load_side(s) for s in stems}
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S5 needs the robot and the cut')
        by_kc[(k, c)] = sides[s]
    pairs = [(a, b) for a in range(5) for b in range(5) if a != b]
    out = {'stage': 's5', 'head': head, 'variant': variant,
           'spec': 'SPEC_b2_divergence_walls v0.9, G3/P7/P8'}

    # ── G3: every map against itself ────────────────────────────────────────
    g3, g3_fail = [], []
    for s in stems:
        side = sides[s]
        res, sh = _side_classify(side, side, (0.0, 0.0), (0.0, 0.0))
        w = res['weights']['w'] if res['weights'] else np.array([])
        good = ~np.isnan(w)
        worst = float(np.abs(w[good]).max()) if good.any() else 0.0
        ok = (sh['corroborated'] == 1.0) and worst == 0.0
        row = {'map': s, 'C': sh['corroborated'], 'max_abs_w': worst,
               'n_weighable': int(good.sum()), 'holds': ok}
        g3.append(row)
        if not ok:
            g3_fail.append(row)
        print(f'{s:<42} C {sh["corroborated"]:.4f}  max|w| {worst:.3e}  '
              f'{"HOLD" if ok else "FAIL"}', flush=True)
    out['G3'] = {'rows': g3, 'holds': not g3_fail, 'failures': g3_fail}
    print(f'G3: {len(g3) - len(g3_fail)}/{len(g3)} maps  '
          f'{"HOLD" if not g3_fail else "FAIL"}')
    if g3_fail:
        print('G3 failed, so P7 and P8 are not evaluated in this run.')
        out['P7'] = out['P8'] = None
        return _write_s5(out, out_dir)

    # ── P7: w against the same-direction null, both at cut1200 ──────────────
    print(f'\nP7  A and B at cut1200{"":>6}{"mean w":>10}{"mean w_null":>13}'
          f'{"|E7|":>7}  verdict')
    p7_rows, p7_hold = [], 0
    cached1200 = {}
    for (a, b) in pairs:
        A, B = by_kc[(a, 1200)], by_kc[(b, 1200)]
        res, sh = _side_classify(A, B, spawn[a], spawn[b])
        dirs_b = segment_dirs(B['seg_of'], B['el'], B['graph']['segments'])
        wn = null_weights(A['seg_of'], res['weights'], A['bearing_hist'],
                          B['bearing_hist'], dirs_b)
        keep = res['assigned'] & res['corroborated']
        e7 = keep & ~np.isnan(res['weights']['w']) & ~np.isnan(wn)
        n7 = int(e7.sum())
        mw = float(res['weights']['w'][e7].mean()) if n7 else None
        mn = float(wn[e7].mean()) if n7 else None
        ok = bool(n7 and mw < mn)
        p7_hold += ok
        cached1200[(a, b)] = sh
        p7_rows.append({'A': a, 'B': b, 'n_E7': n7, 'mean_w': mw,
                        'mean_w_null': mn, 'holds': ok,
                        'reported': _reported(sh, n7)})
        print(f'  A=k{a} B=k{b}{"":>12}'
              + (f'{mw:>10.4f}{mn:>13.4f}' if n7 else f'{"-":>10}{"-":>13}')
              + f'{n7:>7}  {"HOLD" if ok else "FAIL"}', flush=True)
    out['P7'] = {'rows': p7_rows, 'n_holding': p7_hold, 'of': len(pairs),
                 'min_required': P7_MIN_PAIRS, 'holds': p7_hold >= P7_MIN_PAIRS}

    # ── P8: A at cut1200, B at cut1200 against B at cut60 ───────────────────
    print(f'\nP8  A at cut1200{"":>13}{"w_mean B1200":>14}{"w_mean B60":>12}'
          f'  verdict')
    p8_rows, p8_hold = [], 0
    for (a, b) in pairs:
        _r60, sh60 = _side_classify(by_kc[(a, 1200)], by_kc[(b, 60)],
                                    spawn[a], spawn[b])
        sh1200 = cached1200[(a, b)]
        m1200, m60 = sh1200.get('w_mean'), sh60.get('w_mean')
        ok = bool(m1200 is not None and m60 is not None and m1200 < m60)
        p8_hold += ok
        p8_rows.append({'A': a, 'B': b, 'w_mean_B1200': m1200,
                        'w_mean_B60': m60, 'holds': ok,
                        'reported_B60': _reported(sh60)})
        print(f'  A=k{a} B=k{b}{"":>12}'
              + (f'{m1200:>14.4f}' if m1200 is not None else f'{"-":>14}')
              + (f'{m60:>12.4f}' if m60 is not None else f'{"-":>12}')
              + f'  {"HOLD" if ok else "FAIL"}', flush=True)
    out['P8'] = {'rows': p8_rows, 'n_holding': p8_hold, 'of': len(pairs),
                 'min_required': P8_MIN_PAIRS, 'holds': p8_hold >= P8_MIN_PAIRS}

    # ── reported, not tested ────────────────────────────────────────────────
    rep = {'A_cut1200_by_B_cut': [], 'same_cut': [], 'empty_histograms': []}
    for (a, b) in pairs:
        for c in CUTS:
            _r, sh = _side_classify(by_kc[(a, 1200)], by_kc[(b, c)],
                                    spawn[a], spawn[b])
            rep['A_cut1200_by_B_cut'].append(
                {'A': a, 'B': b, 'cut_A': 1200, 'cut_B': c, **_reported(sh)})
            _r2, sh2 = _side_classify(by_kc[(a, c)], by_kc[(b, c)],
                                      spawn[a], spawn[b])
            rep['same_cut'].append(
                {'A': a, 'B': b, 'cut_A': c, 'cut_B': c, **_reported(sh2)})
    for s in stems:
        h = sides[s]['bearing_hist'] or []
        rep['empty_histograms'].append(
            {'map': s, 'n_segments': len(h),
             'n_empty': int(sum(1 for x in h if np.sum(x) <= 0))})
    out['reported'] = rep

    print(f'\nG3: {"HOLD" if not g3_fail else "FAIL"}  {len(g3)}/{len(g3)} maps')
    print(f'P7: {"HOLD" if out["P7"]["holds"] else "FAIL"}  {p7_hold}/'
          f'{len(pairs)} pairs, {P7_MIN_PAIRS} required')
    print(f'P8: {"HOLD" if out["P8"]["holds"] else "FAIL"}  {p8_hold}/'
          f'{len(pairs)} pairs, {P8_MIN_PAIRS} required')
    return _write_s5(out, out_dir)


def _write_s5(payload: dict, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f's5_viewpoint_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    print(f'written, overwriting nothing: {path}')
    return payload


# ── S6: the cut matrix, G4 and P9 (spec v0.11, "Symmetric summary and
#    convergence series (O3)") ────────────────────────────────────────────────

SPEC_REL = 'docs/specs/SPEC_b2_divergence_walls.md'
S6_REGISTRATION = '533fa52'        # the commit that registered P9 (spec v0.11)
P9_HEADER = '**P9. REGISTERED'
P9_GATE_TEXT = 'P9. REGISTERED'
S4_RESULT = 's4_predictions_20261005T230035Z.json'
S5_RESULT = 's5_viewpoint_20261006T212758Z.json'
FULL_CUT = 1200                    # the cut at which A's wall set is final
P9_A_CUTS = (60, 120, 240)         # P9's three early A cuts
N_ROBOTS = 5
N_SEEN, N_UNSEEN = 140, 180        # of the 320-row matrix; both are asserted


def ordered_pairs(n_robots: int = N_ROBOTS) -> list[tuple[int, int]]:
    """The 20 ordered pairs (A, B) of distinct robots, in S5's order."""
    return [(a, b) for a in range(n_robots) for b in range(n_robots) if a != b]


def is_seen(cut_a: int, cut_b: int) -> bool:
    """Was this row's C computed before S6? Spec v0.11, "Rows already seen".

    Seen exactly when A sits at cut1200 -- P3's 80 rows, S4 -- or when the two
    cuts are equal -- the same-cut series, S4's other 80, the two sets sharing
    the 20 rows where both maps are at cut1200. Everything else is one of the
    180 rows no result file in `experiments/logs/divergence/` holds, and the
    reason this stage computes the seen rows first and gates on them.
    """
    return cut_a == FULL_CUT or cut_a == cut_b


def matrix_keys(cuts: tuple = CUTS, n_robots: int = N_ROBOTS) -> list[tuple]:
    """The 320 keys of the cut matrix: (A, B, cut_A, cut_B), A != B."""
    return [(a, b, ca, cb) for (a, b) in ordered_pairs(n_robots)
            for ca in cuts for cb in cuts]


def s6_classify(by_kc: dict, spawn: dict, key: tuple) -> tuple:
    """The `classify` call behind one cut-matrix row, and B in A's frame.

    Returns `(res, el_b, a, b)`. Split out of `s6_row` -- which is now this
    function plus `shares` -- so that S7 can take the per-element
    corroboration mask from the SAME call that produced the share, rather than
    from a second implementation of T1. The statements and their order are
    unchanged, so S6's numbers cannot move; `test_s7_s6_row_is_the_split_plus_
    shares` holds the two together, and the S6 cases are untouched.

    `res['corroborated']` is the mask: the array `corroborate` returned.
    `res['assigned']` is `seg_of >= 0`, and `shares` counts
    `cls[assigned] == CORROBORATED`, which is `corroborated & assigned` -- so
    the mask and the share are the same object seen two ways.
    """
    ka, kb, cut_a, cut_b = key
    a, b = by_kc[(ka, cut_a)], by_kc[(kb, cut_b)]

    keep_b = b['seg_of'] >= 0
    bx, by = to_frame_a(b['el']['x'], b['el']['y'], spawn[kb], spawn[ka])
    el_b = dict(face_elements_only(b['el'], b['seg_of']))
    el_b['x'], el_b['y'] = bx[keep_b], by[keep_b]
    el_b['seg'] = b['seg_of'][keep_b]

    res = classify(a['el'], a['seg_of'], el_b, None, a['rho'], a['origin'],
                   a['shape'])
    return res, el_b, a, b


def s6_row(by_kc: dict, spawn: dict, key: tuple) -> dict:
    """One cut-matrix row: C of A@cut_A against B@cut_B, and nothing else.

    THE C FUNCTION IS `shares`, UNCHANGED -- the same function S4's P3 and
    same-cut rows and S5's reported rows were measured with, which is what
    makes G4 an equality rather than a comparison of two implementations. The
    row around it is `pair_row`'s frame change with the same arguments: B's
    face elements translated into A's frame, A's own elements classified, no
    rays (t2 is off, so `classify` never reads them).

    NO VIEWPOINT WEIGHT. `hist_a` and `hist_b` are left at None, as spec v0.11
    "What is not computed" requires, so `classify` takes the `corroborate`
    branch rather than `corroborate_matches`. The corroboration mask is the
    same either way -- `classify` says so, and
    `test_o1_the_mask_is_the_same_with_and_without_matching` holds it -- so G4
    can still compare against S5, which passed histograms.

    `key` is the whole key, so a caller (or a test's spy) sees which row is
    being computed before any C exists for it.
    """
    ka, kb, cut_a, cut_b = key
    res, el_b, a, b = s6_classify(by_kc, spawn, key)
    sh = shares(res)
    return {
        'A': ka, 'B': kb, 'cut_A': cut_a, 'cut_B': cut_b,
        'A_stem': a['stem'], 'B_stem': b['stem'],
        'L_A_m': sh['L_A'],
        'corroborated': sh['corroborated'],
        'not_corroborated': sh['not_corroborated'],
        'n_corroborated': sh['n_corroborated'],
        'n_elements': sh['n_elements'],
        'seen': is_seen(cut_a, cut_b),
    }


def cut_matrix(by_kc: dict, spawn: dict, keys) -> dict:
    """The rows for `keys`, keyed by key. Computes exactly what it is asked for."""
    return {k: s6_row(by_kc, spawn, k) for k in keys}


def s4_comparisons(s4: dict) -> list[dict]:
    """Every stored value of S4's that S6 recomputes, with its matrix row.

    THE ORDER OF `not_corroborated_by_B_cut` is the registered one: P3 is
    stated over B's cut going "60 -> 120 -> 240 -> 1200", which is `CUTS`.
    S4's own code is not in history -- no commit in this repository contains
    the script that wrote `s4_predictions_20261005T230035Z.json` -- so the
    order cannot be read off it, and the spec is the authority. The order is
    not inferred from the values. It is also cross-checked for free: S5's
    `reported.A_cut1200_by_B_cut` carries `cut_B` explicitly on the same 80
    rows, so a wrong order here would fail G4 on P3's entries alone and leave
    S5's untouched.

    P4's `max_dist_m` and `share_within_eps` are not here: they measure
    corroborated length against `knowledge_world.sdf`, and S6 reads no truth.
    `n_corroborated` is, because it is the corroborated count whose ratio is C.
    """
    out = []
    for i, row in enumerate(s4['P3']):
        vals = row['not_corroborated_by_B_cut']
        if len(vals) != len(CUTS):
            die(f'S4 P3[{i}] holds {len(vals)} values, not {len(CUTS)}; the '
                "series is B's cut over CUTS and cannot be matched to rows")
        for j, v in enumerate(vals):
            out.append({'source': 's4',
                        'path': f'P3[{i}].not_corroborated_by_B_cut[{j}]',
                        'key': (row['A'], row['B'], FULL_CUT, CUTS[j]),
                        'field': 'not_corroborated', 'stored': v})
    # P6: A and B both at cut1200, all 20 rows. The four with A = k0 are the
    # ones P6 labels seen and does not test; they are stored values all the
    # same, so G4 compares them.
    for i, row in enumerate(s4['P6']):
        out.append({'source': 's4', 'path': f'P6[{i}].corroborated',
                    'key': (row['A'], row['B'], FULL_CUT, FULL_CUT),
                    'field': 'corroborated', 'stored': row['corroborated']})
    for i, row in enumerate(s4['same_cut']):
        c = row['cut']
        for field in ('L_A_m', 'corroborated', 'not_corroborated'):
            out.append({'source': 's4', 'path': f'same_cut[{i}].{field}',
                        'key': (row['A'], row['B'], c, c),
                        'field': field, 'stored': row[field]})
    for i, row in enumerate(s4['P4']):
        c = row['cut']
        out.append({'source': 's4', 'path': f'P4[{i}].n_corroborated',
                    'key': (row['A'], row['B'], c, c),
                    'field': 'n_corroborated', 'stored': row['n_corroborated']})
    return out


def s5_comparisons(s5: dict) -> list[dict]:
    """Every stored C of S5's that S6 recomputes, with its matrix row.

    P7's AND P8's CUTS COME FROM THEIR REGISTERED WORDING, not from a field in
    the file: P7 is "A and B both at cut1200", and P8 is "A at cut1200" with B
    at cut1200 against B at cut60, so `P8.rows[i].reported_B60` is the
    (A@1200, B@60) row and P8's B-at-cut1200 column is P7's cached row, already
    compared above. The two `reported` series do carry `cut_A` and `cut_B`, and
    those are used as stored.

    G3's rows are each map against ITSELF. The cut matrix has A != B, so they
    are not rows of it and are not G4's to compare. The weight fields -- W,
    w_mean, the percentiles, the unweighable shares -- are not here either:
    S6 computes no viewpoint weight.
    """
    out = []
    for i, row in enumerate(s5['P7']['rows']):
        for field in ('corroborated', 'n_corroborated'):
            out.append({'source': 's5',
                        'path': f'P7.rows[{i}].reported.{field}',
                        'key': (row['A'], row['B'], FULL_CUT, FULL_CUT),
                        'field': field, 'stored': row['reported'][field]})
    for i, row in enumerate(s5['P8']['rows']):
        for field in ('corroborated', 'n_corroborated'):
            out.append({'source': 's5',
                        'path': f'P8.rows[{i}].reported_B60.{field}',
                        'key': (row['A'], row['B'], FULL_CUT, 60),
                        'field': field, 'stored': row['reported_B60'][field]})
    for name in ('A_cut1200_by_B_cut', 'same_cut'):
        for i, row in enumerate(s5['reported'][name]):
            for field in ('corroborated', 'n_corroborated'):
                out.append({'source': 's5',
                            'path': f'reported.{name}[{i}].{field}',
                            'key': (row['A'], row['B'], row['cut_A'],
                                    row['cut_B']),
                            'field': field, 'stored': row[field]})
    return out


def g4_compare(comparisons: list[dict], rows: dict) -> dict:
    """G4: every stored seen value must equal S6's, exactly (`==`, no tolerance).

    `rows` holds the SEEN rows and nothing else, so a comparison that names a
    row outside them is a bug in the extractors and stops the stage here,
    before any unseen row exists.
    """
    out = []
    for c in comparisons:
        if c['key'] not in rows:
            die(f'G4: {c["source"]} {c["path"]} names row {c["key"]}, which is '
                'not among the seen rows; G4 compares seen rows only')
        got = rows[c['key']][c['field']]
        rec = dict(c)
        rec['computed'] = got
        rec['equal'] = bool(got == c['stored'])
        out.append(rec)

    by_source: dict[str, dict] = {}
    for rec in out:
        s = by_source.setdefault(rec['source'], {'n': 0, 'n_equal': 0})
        s['n'] += 1
        s['n_equal'] += int(rec['equal'])
    diffs = [r for r in out if not r['equal']]
    return {'comparisons': out, 'by_source': by_source, 'n': len(out),
            'n_equal': len(out) - len(diffs), 'all_equal': not diffs,
            'differing': diffs}


def p9_verdict(rows: dict, cuts: tuple = CUTS, a_cuts: tuple = P9_A_CUTS,
               n_robots: int = N_ROBOTS) -> dict:
    """P9, exactly as registered at spec v0.11.

    60 series: 20 ordered pairs by A's cuts 60, 120 and 240, each the
    not-corroborated share of A's walls at that cut as B's cut goes
    60 -> 120 -> 240 -> 1200. A RISE OF ANY SIZE AT ANY STEP IS A FAILURE and
    is shown, so the step test is `>` with no tolerance. With A fixed the
    denominator is constant, which is asserted rather than assumed and is what
    makes `rise * L_A_m` a rise in metres. A series whose A walls have zero
    length is undefined and counts as a failure. P9 holds only if all 60 hold.
    """
    series, failures = [], []
    for (a, b) in ordered_pairs(n_robots):
        for ca in a_cuts:
            keys = [(a, b, ca, cb) for cb in cuts]
            vals = [rows[k]['not_corroborated'] for k in keys]
            lengths = {rows[k]['L_A_m'] for k in keys}
            assert len(lengths) == 1, (a, b, ca, lengths)
            l_a = lengths.pop()
            rec = {'A': a, 'B': b, 'cut_A': ca, 'L_A_m': l_a,
                   'not_corroborated_by_B_cut': vals, 'B_cuts': list(cuts),
                   'rises': [], 'undefined': False, 'holds': True}
            if l_a == 0.0:
                fail = {'A': a, 'B': b, 'cut_A': ca, 'L_A_m': l_a,
                        'step': None, 'from_cut_B': None, 'to_cut_B': None,
                        'from': None, 'to': None, 'rise': None, 'rise_m': None,
                        'reason': "A's walls have zero length, so the series "
                                  'is undefined'}
                rec.update(undefined=True, holds=False)
                failures.append(fail)
            else:
                for i in range(len(cuts) - 1):
                    if vals[i + 1] > vals[i]:          # a rise, no tolerance
                        r = vals[i + 1] - vals[i]
                        fail = {'A': a, 'B': b, 'cut_A': ca, 'L_A_m': l_a,
                                'step': f'cut{cuts[i]}->cut{cuts[i + 1]}',
                                'from_cut_B': cuts[i], 'to_cut_B': cuts[i + 1],
                                'from': vals[i], 'to': vals[i + 1],
                                'rise': r, 'rise_m': r * l_a, 'reason': None}
                        rec['rises'].append(fail)
                        failures.append(fail)
                rec['holds'] = not rec['rises']
            series.append(rec)

    n_hold = sum(1 for s in series if s['holds'])
    return {'series': series, 'of': len(series), 'n_holding': n_hold,
            'holds': n_hold == len(series), 'failures': failures}


def symmetric_table(rows: dict, cuts: tuple = CUTS,
                    n_robots: int = N_ROBOTS) -> list[dict]:
    """O3's symmetric form: both directions of each unordered pair, side by side.

    For each unordered pair {X, Y} with X < Y and each of the 16 (a, b), the
    two directions X@a -> Y@b and Y@b -> X@a with their difference, FIRST MINUS
    SECOND. No mean and no minimum: when one map is much smaller the two
    directions measure different things, so the pair of numbers is the summary.

    160 entries, and every one of the 320 rows is used exactly once -- a row
    with A < B is some entry's first direction and one with A > B is some
    entry's second. That is asserted, not asserted-looking.
    """
    entries, used = [], []
    for x in range(n_robots):
        for y in range(x + 1, n_robots):
            for a in cuts:
                for b in cuts:
                    kf, kr = (x, y, a, b), (y, x, b, a)
                    cf, cr = rows[kf]['corroborated'], rows[kr]['corroborated']
                    entries.append({
                        'X': x, 'Y': y, 'cut_X': a, 'cut_Y': b,
                        'C_X_to_Y': cf, 'C_Y_to_X': cr,
                        'difference': cf - cr,
                        'seen_X_to_Y': rows[kf]['seen'],
                        'seen_Y_to_X': rows[kr]['seen']})
                    used += [kf, kr]
    assert len(entries) == len(cuts) ** 2 * n_robots * (n_robots - 1) // 2
    assert len(used) == len(set(used)) == len(rows), (
        len(used), len(set(used)), len(rows))
    return entries


def _sha256(path: Path) -> str:
    import hashlib                                          # noqa: PLC0415
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _is_ancestor(rev: str) -> bool:
    """Is `rev` an ancestor of HEAD? Stdout only, so `_git` is enough.

    `merge-base rev HEAD` is rev's own commit exactly when rev is an ancestor.
    An empty answer -- no git, no such rev -- is not an ancestry claim and
    reads as False.
    """
    base = _git('merge-base', rev, 'HEAD')
    full = _git('rev-parse', f'{rev}^{{commit}}')
    return bool(base) and bool(full) and base == full


def spec_paragraph(text: str, header: str) -> str:
    """The paragraph starting at `header`, unwrapped to one line.

    Read by script from the spec, never retyped: a registered wording quoted
    by hand is a wording that can drift from the one that was registered. The
    paragraph runs from its header line to the first blank line; a list marker
    and the continuation indent come off, and the lines join with one space,
    which is the shape S4 stored P3, P4 and P6 in.
    """
    lines = text.splitlines()
    start = None
    for i, ln in enumerate(lines):
        if ln.lstrip().lstrip('- ').startswith(header):
            start = i
            break
    if start is None:
        die(f'the spec carries no paragraph starting {header!r}')
    out = []
    for ln in lines[start:]:
        s = ln.strip()
        if not s:
            break
        out.append(s.removeprefix('- ') if not out else s)
    return ' '.join(out)


def _write_s6(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'{prefix}_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    sha = _sha256(path)
    print(f'written, overwriting nothing: {path}')
    print(f'sha256 {sha}')
    return path, sha


def _print_cut_table(rows: dict, cuts: tuple = CUTS) -> None:
    """not_corroborated, min-max over the 20 ordered pairs, per (cut_A, cut_B)."""
    label = 'cut_A \\ cut_B'
    print(f'\nnot_corroborated, min-max over the {len(ordered_pairs())} '
          'ordered pairs   (* = unseen before this run)')
    print(f'{label:<16}' + ''.join(f'{f"cut_B {c}":>18}' for c in cuts))
    for ca in cuts:
        cells = []
        for cb in cuts:
            v = [r['not_corroborated'] for r in rows.values()
                 if r['cut_A'] == ca and r['cut_B'] == cb]
            mark = '' if is_seen(ca, cb) else '*'
            cells.append(f'{min(v):.4f}-{max(v):.4f}{mark}' if v else '-')
        print(f'{f"cut_A {ca}":<16}' + ''.join(f'{c:>18}' for c in cells))


def s6_matrix(by_kc: dict, spawn: dict, comparisons: list[dict], meta: dict,
              out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
              cuts: tuple = CUTS, n_robots: int = N_ROBOTS) -> dict:
    """The 320-row matrix, in the order that keeps the 180 unseen rows unseen.

    THE SEEN ROWS FIRST, THEN G4, AND ONLY THEN THE UNSEEN ONES. If G4 fails
    the stage writes a file of seen rows and stops at exit 2: no unseen row is
    computed, written or printed, because none has been computed yet. The
    guard is the order of the code, not a filter applied afterwards.

    Reads no file and resolves no spawn -- `run_s6` is this function plus the
    corpus I/O -- which is what lets the whole pipeline, G4's refusal
    included, be exercised on hand-built sides.
    """
    t0 = time.monotonic() if t0 is None else t0
    keys = matrix_keys(cuts, n_robots)
    seen = [k for k in keys if is_seen(k[2], k[3])]
    unseen = [k for k in keys if not is_seen(k[2], k[3])]
    assert len(seen) == N_SEEN, len(seen)
    assert len(unseen) == N_UNSEEN, len(unseen)

    rows = cut_matrix(by_kc, spawn, seen)
    g4 = g4_compare(comparisons, rows)
    srcs = ', '.join(f'{s} {v["n_equal"]}/{v["n"]}'
                     for s, v in sorted(g4['by_source'].items()))
    print(f'G4: {"HOLD" if g4["all_equal"] else "FAIL"}  '
          f'{g4["n_equal"]}/{g4["n"]} stored seen values equal  ({srcs})')

    if not g4['all_equal']:
        for rec in g4['differing']:
            a, b, ca, cb = rec['key']
            print(f'  {rec["source"]} {rec["path"]}: A=k{a} B=k{b} '
                  f'cut_A {ca} cut_B {cb}  {rec["field"]}  '
                  f'stored {rec["stored"]!r}  S6 {rec["computed"]!r}')
        fail = dict(meta)
        fail.update({'stage': 's6_g4_fail', 'seconds': time.monotonic() - t0,
                     'G4': g4, 'seen_rows': [rows[k] for k in seen]})
        assert all(r['seen'] for r in fail['seen_rows'])
        path, _ = _write_s6(fail, out_dir, 's6_g4_fail')
        die(f'G4 failed on {len(g4["differing"])} of {g4["n"]} stored seen '
            'values; the 180 unseen rows were not computed, written or '
            f'printed. {path}')

    rows.update(cut_matrix(by_kc, spawn, unseen))
    assert len(rows) == N_SEEN + N_UNSEEN, len(rows)

    p9 = p9_verdict(rows, cuts, P9_A_CUTS, n_robots)
    sym = symmetric_table(rows, cuts, n_robots)

    print(f'P9: {"HOLD" if p9["holds"] else "FAIL"}  {p9["n_holding"]}/'
          f'{p9["of"]} series non-increasing')
    for f in p9['failures']:
        if f['reason']:
            print(f'  A=k{f["A"]} B=k{f["B"]} cut_A {f["cut_A"]}: '
                  f'{f["reason"]}')
        else:
            print(f'  A=k{f["A"]} B=k{f["B"]} cut_A {f["cut_A"]}  '
                  f'{f["step"]}  {f["from"]:.6f} -> {f["to"]:.6f}  '
                  f'rise {f["rise"]:.6g} = {f["rise_m"]:.4g} m '
                  f'of {f["L_A_m"]:.1f} m')
    _print_cut_table(rows, cuts)

    payload = dict(meta)
    payload.update({'seconds': time.monotonic() - t0, 'n_rows': len(rows),
                    'n_seen': len(seen), 'n_unseen': len(unseen),
                    'rows': [rows[k] for k in keys], 'G4': g4, 'P9': p9,
                    'symmetric': sym})
    path, sha = _write_s6(payload, out_dir, 's6_cutmatrix')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s6(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """S6: the cut matrix, G4 against S4 and S5, then P9. Spec v0.11, O3.

    The gates run in this order and each one before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before a map is loaded. A registered
       prediction evaluated against code that is not in history is not
       evidence about that code. Tracked changes stop it; untracked files do
       not -- `.vscode/` has sat there for weeks -- and both counts are
       recorded.
    2. REGISTRATION BEFORE RESULT: `533fa52`, the commit that registered P9,
       must be an ancestor of HEAD, and the spec at HEAD must carry
       "P9. REGISTERED". A verdict on a prediction that is not in history
       behind this code is not a verdict.
    3. The P9 paragraph is read from the spec AT HEAD by script and stored
       verbatim, never retyped.
    """
    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S6_REGISTRATION):
        die(f'{S6_REGISTRATION} registered P9 and is not an ancestor of HEAD '
            f'{head[:7] or "(unknown)"}; the registration must be in history '
            'behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    if P9_GATE_TEXT not in spec_text:
        die(f'the spec at HEAD carries no "{P9_GATE_TEXT}"; S6 evaluates a '
            'registered prediction or it does not run')
    p9_text = spec_paragraph(spec_text, P9_HEADER)
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s4_path, s5_path = out_dir / S4_RESULT, out_dir / S5_RESULT
    for p in (s4_path, s5_path):
        if not p.is_file():
            die(f'{p} is missing; G4 has nothing to compare the seen rows to')
    s4 = json.loads(s4_path.read_text())
    s5 = json.loads(s5_path.read_text())
    comparisons = s4_comparisons(s4) + s5_comparisons(s5)

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S6 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's6', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.11, O3/G4/P9',
            'predictions_verbatim': {'P9': p9_text},
            'sources': {'s4': {'file': s4_path.name,
                               'sha256': _sha256(s4_path),
                               'head': s4.get('head')},
                        's5': {'file': s5_path.name,
                               'sha256': _sha256(s5_path),
                               'head': s5.get('head')}},
            'note': 'no viewpoint weight and no truth: S6 computes C only'}
    return s6_matrix(by_kc, spawn, comparisons, meta, out_dir, t0)


# ── S7: gains and losses, G5 and P10 (spec v0.12, "Gains and losses
#    (P10)") ───────────────────────────────────────────────────────────────────

S7_REGISTRATION = '9541ea1'        # the commit that registered P10 (spec v0.12)
P10_HEADER = '**P10. REGISTERED'
P10_GATE_TEXT = 'P10. REGISTERED'
S6_RESULT = 's6_cutmatrix_20261008T225604Z.json'
S6_SHA256 = ('a3d584f310e509c33bcca09c2fe641d4914dd4f768146f10c1ebf340f58b2'
             'f13')
P10_MIN_SERIES = 10                # "at least 10 of P3's 20 series"
NEAREST_LIMIT_M = 1.0              # the "no same-direction element within" bound
N_SERIES = 80                      # P3's 20 and P9's 60


def series_keys(cuts: tuple = CUTS, n_robots: int = N_ROBOTS) -> list[tuple]:
    """The 80 fixed-A series: every ordered pair by every A cut.

    P3's 20 are those with A at cut1200 and P9's 60 are the other three A
    cuts, which together are exactly the 80 the spec's "What is computed"
    names.
    """
    return [(a, b, ca) for (a, b) in ordered_pairs(n_robots) for ca in cuts]


def s7_mask(by_kc: dict, spawn: dict, key: tuple) -> dict:
    """One row's per-element corroboration mask, and the share it makes.

    THE MASK IS THE MEASURE'S OWN, not a reimplementation: `classify` already
    returns it as `corroborated` -- the array `corroborate` produced -- and
    `shares` counts `cls[assigned] == CORROBORATED`, which is the same array
    AND-ed with `assigned`. `s6_classify` is the call S6's rows go through, so
    the mask and S6's share come from one classification.

    Only A's ASSIGNED elements enter a gain or a loss: §3 leaves the rest as
    excluded length and they never appear in a share. `assigned` depends on A
    alone, so it is the same at every B cut of a series.
    """
    res, el_b, a, b = s6_classify(by_kc, spawn, key)
    sh = shares(res)
    return {
        'key': key, 'A': key[0], 'B': key[1], 'cut_A': key[2], 'cut_B': key[3],
        'A_stem': a['stem'], 'B_stem': b['stem'],
        'corr': res['corroborated'], 'assigned': res['assigned'],
        'rho': res['rho'], 'eps': res['eps'],
        'el_a': a['el'], 'seg_of_a': a['seg_of'], 'el_b': el_b,
        'n_elements': sh['n_elements'], 'n_corroborated': sh['n_corroborated'],
        'corroborated': sh['corroborated'], 'L_A_m': sh['L_A'],
    }


def assert_aligned(masks: dict, keys: list) -> None:
    """A's elements are the same set, in the same order, at every B cut.

    `face_elements` is a function of the grid alone -- four passes in DIRS
    order, each row-major -- and with A fixed the four rows of a series share
    one loaded side, so the arrays are the same object. S7 asserts it anyway:
    an index-wise gain and loss count is wrong the moment the order differs,
    and this is the assertion that would catch it. Count, positions,
    directions and the assigned mask.
    """
    ref = masks[keys[0]]
    for k in keys[1:]:
        m = masks[k]
        assert m['n_elements'] == ref['n_elements'], (k, keys[0])
        assert m['el_a']['n'] == ref['el_a']['n'], (k, keys[0])
        for f in ('x', 'y', 'dir'):
            assert np.array_equal(m['el_a'][f], ref['el_a'][f]), (k, f)
        assert np.array_equal(m['assigned'], ref['assigned']), (k, 'assigned')


def gain_loss_step(m_k: dict, m_kp: dict) -> dict:
    """One step's gains and losses, by the spec's definitions.

    A **gain** is an element B does not corroborate at k and does at k'; a
    **loss** is an element B corroborates at k and does not at k'. Restricted
    to A's assigned elements, and index-wise, which `assert_aligned` is what
    licenses.
    """
    keep = m_k['assigned']
    gained = keep & ~m_k['corr'] & m_kp['corr']
    lost = keep & m_k['corr'] & ~m_kp['corr']
    n_g, n_l = int(gained.sum()), int(lost.sum())
    rho = m_k['rho']
    return {'n_gained': n_g, 'n_lost': n_l,
            'gained_m': float(n_g * rho), 'lost_m': float(n_l * rho),
            'gained_idx': np.nonzero(gained)[0],
            'lost_idx': np.nonzero(lost)[0]}


def gain_loss_series(masks: dict, a: int, b: int, cut_a: int,
                     cuts: tuple = CUTS) -> dict:
    """One fixed-A series: three steps of gains and losses, as counts and metres.

    This is the reported sweep, and it is deliberately the thing a test can
    spy on: if G5 fails it must never be called.
    """
    keys = [(a, b, cut_a, cb) for cb in cuts]
    assert_aligned(masks, keys)
    rows = [masks[k] for k in keys]
    steps = []
    for i in range(len(cuts) - 1):
        gl = gain_loss_step(rows[i], rows[i + 1])
        change = rows[i + 1]['n_corroborated'] - rows[i]['n_corroborated']
        # the identity the counting has to satisfy: every element that changed
        # state is a gain or a loss, so the net is the change in the count
        assert gl['n_gained'] - gl['n_lost'] == change, (a, b, cut_a, i)
        steps.append({
            'step': f'cut{cuts[i]}->cut{cuts[i + 1]}',
            'from_cut_B': cuts[i], 'to_cut_B': cuts[i + 1],
            'n_gained': gl['n_gained'], 'n_lost': gl['n_lost'],
            'gained_m': gl['gained_m'], 'lost_m': gl['lost_m'],
            'change_in_corroborated': change,
        })
    return {
        'A': a, 'B': b, 'cut_A': cut_a, 'B_cuts': list(cuts),
        'A_stem': rows[0]['A_stem'], 'L_A_m': rows[0]['L_A_m'],
        'n_elements': rows[0]['n_elements'],
        'n_corroborated_by_B_cut': [r['n_corroborated'] for r in rows],
        'steps': steps,
        'any_loss': any(s['n_lost'] > 0 for s in steps),
        'total_gained': sum(s['n_gained'] for s in steps),
        'total_lost': sum(s['n_lost'] for s in steps),
    }


def within_t1_tolerances(across_m: float, along_m: float, eps: float,
                         rho: float) -> bool:
    """Are these two distances inside T1's tolerances? Stated once, here.

    `across <= eps` and `along <= rho/2`, both INCLUSIVE with T1's own 1e-9 m
    slack, which is `corroborate`'s comparison and nothing else. S7's
    `within_t1` flag and S9's geometry classes both go through this, so the
    boundary cannot drift between the stage that found a loss and the stage
    that explains it.
    """
    return bool(across_m <= eps + T1_TOL_M
                and along_m <= 0.5 * rho + T1_TOL_M)


def nearest_same_dir(el_a: dict, i: int, el_b: dict) -> dict | None:
    """The nearest same-direction element of B to A's element `i`.

    NEAREST IS: the smallest ACROSS distance, ties broken by the smallest
    ALONG distance, then by the lowest B element index. Across and along are
    T1's own two axes -- `|d.n|` and `|d.t|`, with `n` A's element normal and
    `t = (-n_y, n_x)` -- and the frame is the common one T1 works in, B
    already translated into A's by `s6_classify`. Same direction is an
    equality on `dir`, as T1 tests it: normals are axis-aligned by
    construction and the frame change is a pure translation.

    The tie rule decides WHICH element is named, never the distances reported:
    a tie is by definition an equal across and an equal along. It differs from
    `corroborate_matches`'s key, which puts B's segment id before the index
    because O1 needs the segment; nothing here does.

    T1 buckets B on an eps lattice and looks only at the 3x3 block around A's
    own bucket, which is sound because both its tolerances are at most eps. A
    LOST ELEMENT MAY HAVE NO B ELEMENT WITHIN EPS AT ALL -- that is the case
    this report exists for -- so this scans the whole of `el_b` instead of the
    block.

    WHAT `el_b` CONTAINS IS THE CALLER'S CHOICE, and it decides what a result
    means. In S7 the caller is `lost_elements`, which passes the `el_b` that
    `s6_classify` built: `face_elements_only(b['el'], b['seg_of'])`, which is
    B's elements ASSIGNED TO A WALL SEGMENT and no others. So S7's "none within
    1 m" means no assigned same-direction element within a metre, not no
    element at all -- which is what P12's carve-out rests on. Handed B's full
    arrays instead, the same call would answer a different question.

    Returns None when `el_b` holds no element in that direction.
    """
    sel = np.nonzero(el_b['dir'] == int(el_a['dir'][i]))[0]
    if sel.size == 0:
        return None
    dx = el_b['x'][sel] - el_a['x'][i]
    dy = el_b['y'][sel] - el_a['y'][i]
    nx, ny = el_a['nx'][i], el_a['ny'][i]
    across = np.abs(dx * nx + dy * ny)
    along = np.abs(dx * -ny + dy * nx)
    # lexsort takes the LAST key as primary: across, then along, then index
    j = np.lexsort((sel, along, across))[0]
    return {'b_index': int(sel[j]), 'across_m': float(across[j]),
            'along_m': float(along[j]),
            'dist_m': float(math.hypot(float(across[j]), float(along[j])))}


def lost_elements(masks: dict, series: list, cuts: tuple = CUTS) -> list[dict]:
    """Reported, not tested: every lost element, located and measured.

    Per element: where it is, which way it faces, and the across and along
    distances to the nearest same-direction element of B at k'. `within_t1`
    says whether that nearest element is inside T1's own two tolerances, which
    it cannot be -- the element was not corroborated at k' -- so it is a
    self-check on the search rather than a finding.
    """
    out = []
    for s in series:
        a, b, ca = s['A'], s['B'], s['cut_A']
        for i in range(len(cuts) - 1):
            k, kp = (a, b, ca, cuts[i]), (a, b, ca, cuts[i + 1])
            m_k, m_kp = masks[k], masks[kp]
            gl = gain_loss_step(m_k, m_kp)
            el_a, el_b = m_k['el_a'], m_kp['el_b']
            eps, half = m_kp['eps'], 0.5 * m_kp['rho']
            for idx in gl['lost_idx']:
                idx = int(idx)
                near = nearest_same_dir(el_a, idx, el_b)
                rec = {
                    'A': a, 'B': b, 'cut_A': ca,
                    'step': f'cut{cuts[i]}->cut{cuts[i + 1]}',
                    'from_cut_B': cuts[i], 'to_cut_B': cuts[i + 1],
                    'element': idx, 'face': int(m_k['seg_of_a'][idx]),
                    'x': float(el_a['x'][idx]), 'y': float(el_a['y'][idx]),
                    'dir': int(el_a['dir'][idx]),
                    'nx': float(el_a['nx'][idx]), 'ny': float(el_a['ny'][idx]),
                    'nearest_b_index': None, 'across_m': None,
                    'along_m': None, 'dist_m': None, 'within_t1': None,
                }
                if near is not None:
                    rec.update(nearest_b_index=near['b_index'],
                               across_m=near['across_m'],
                               along_m=near['along_m'],
                               dist_m=near['dist_m'],
                               within_t1=within_t1_tolerances(
                                   near['across_m'], near['along_m'],
                                   eps, m_kp['rho']))
                out.append(rec)
    return out


def g5_shares(masks: dict, s6_rows: dict) -> dict:
    """G5's first check: the share recomputed from each mask equals S6's.

    Exactly, with no tolerance, on all 320 rows. Two shares are compared, not
    one: the share counted straight off the mask -- corroborated-and-assigned
    over assigned -- and the share `shares` returned from the same
    classification. The first is what the spec's G5 asks for; the second
    catches a mask that has been taken from the wrong row.
    """
    comparisons, n_equal = [], 0
    for key, m in masks.items():
        if key not in s6_rows:
            die(f'G5: row {key} is not in the S6 file; G5 compares all 320 '
                'rows of the cut matrix')
        n = int(m['assigned'].sum())
        c = int((m['corr'] & m['assigned']).sum())
        share = (c / n) if n else 0.0
        s6 = s6_rows[key]
        ok = (share == s6['corroborated']
              and m['corroborated'] == s6['corroborated']
              and c == s6['n_corroborated'] and n == s6['n_elements'])
        n_equal += int(ok)
        comparisons.append({
            'key': list(key), 'n_elements': n, 'n_corroborated': c,
            'share_from_mask': share, 'share_from_shares': m['corroborated'],
            's6_corroborated': s6['corroborated'],
            's6_n_corroborated': s6['n_corroborated'],
            's6_n_elements': s6['n_elements'], 'equal': bool(ok)})
    diffs = [c for c in comparisons if not c['equal']]
    return {'comparisons': comparisons, 'n': len(comparisons),
            'n_equal': n_equal, 'all_equal': not diffs, 'differing': diffs}


def g5_steps(masks: dict, s6_failures: list, rho: float) -> dict:
    """G5's second check: losses minus gains equals S6's rise, in elements.

    NOT IN THE SPEC'S G5 WORDING, which covers the share check alone. It is
    added because the share check sees only whether the masks are right, never
    whether the gain and loss COUNTING is: for each of S6's failing P9 steps,
    `n_lost - n_gained` taken from the masks must equal S6's rise in metres
    divided by the element length, as integers with no tolerance.

    The counting goes through `gain_loss_step`, the same function the reported
    sweep uses, so this checks that function rather than a stand-in. It is
    bounded to S6's failing steps and nothing is kept from it but the verdict:
    on a failure the fail file and the terminal name the step and withhold the
    counts, because the spec's G5 says a failing run writes no gain or loss.
    """
    rows, n_equal = [], 0
    for f in s6_failures:
        if f.get('reason') is not None:            # an undefined series
            continue
        k = (f['A'], f['B'], f['cut_A'], f['from_cut_B'])
        kp = (f['A'], f['B'], f['cut_A'], f['to_cut_B'])
        gl = gain_loss_step(masks[k], masks[kp])
        s6_rise = round(f['rise_m'] / rho)
        ok = (gl['n_lost'] - gl['n_gained']) == s6_rise
        n_equal += int(ok)
        rows.append({'A': f['A'], 'B': f['B'], 'cut_A': f['cut_A'],
                     'step': f'cut{f["from_cut_B"]}->cut{f["to_cut_B"]}',
                     'from_cut_B': f['from_cut_B'],
                     'to_cut_B': f['to_cut_B'],
                     's6_rise_in_elements': s6_rise, 'equal': bool(ok)})
    diffs = [r for r in rows if not r['equal']]
    # the identities only: a count here would be a gain or a loss, and a
    # failing run writes none
    return {'steps': [{k: v for k, v in r.items() if k != 'equal'}
                      for r in rows],
            'n': len(rows), 'n_equal': n_equal, 'all_equal': not diffs,
            'differing': [{k: v for k, v in r.items() if k != 'equal'}
                          for r in diffs]}


def p10_verdict(series: list) -> dict:
    """P10, exactly as registered at spec v0.12.

    Among P3's 20 series -- A at cut1200 -- count those in which at least one
    step contains at least one loss. A loss of one element counts, so the test
    is `n_lost >= 1`. P10 holds if the count is at least 10.
    """
    p3 = [s for s in series if s['cut_A'] == FULL_CUT]
    assert len(p3) == len(ordered_pairs()), len(p3)
    with_loss = [s for s in p3 if s['any_loss']]
    n = len(with_loss)
    return {
        'of': len(p3), 'min_required': P10_MIN_SERIES, 'n_with_loss': n,
        'holds': n >= P10_MIN_SERIES,
        'series_with_losses': [
            {'A': s['A'], 'B': s['B'], 'cut_A': s['cut_A'],
             'total_lost': s['total_lost'],
             'lost_by_step': [st['n_lost'] for st in s['steps']]}
            for s in with_loss]}


def _no_gain_or_loss_in(node, path='$') -> None:
    """Assert a payload names no gain, loss or distance. The G5 fail guard."""
    banned = ('gain', 'loss', 'lost', 'across_m', 'along_m', 'dist_m')
    if isinstance(node, dict):
        for k, v in node.items():
            assert not any(w in str(k).lower() for w in banned), f'{path}.{k}'
            _no_gain_or_loss_in(v, f'{path}.{k}')
    elif isinstance(node, list):
        for v in node:
            _no_gain_or_loss_in(v, f'{path}[]')


def _write_s7(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'{prefix}_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    sha = _sha256(path)
    print(f'written, overwriting nothing: {path}')
    print(f'sha256 {sha}')
    return path, sha


def _pct(vals, q):
    return float(np.percentile(vals, q)) if len(vals) else None


def _print_s7(series: list, lost: list, cuts: tuple) -> None:
    """P3's 20 series, P9's 60 by A cut, then the lost elements."""
    steps = [f'cut{cuts[i]}->{cuts[i + 1]}' for i in range(len(cuts) - 1)]
    print(f'\nP3 series, A at cut{FULL_CUT}   gains/losses in elements per step')
    print(''.ljust(16) + ''.join(s.rjust(16) for s in steps))
    for s in (x for x in series if x['cut_A'] == FULL_CUT):
        cells = [f'+{st["n_gained"]}/-{st["n_lost"]}' for st in s['steps']]
        label = f'  A=k{s["A"]} B=k{s["B"]}'
        print(label.ljust(16) + ''.join(c.rjust(16) for c in cells))

    print('\nP9 series, by A cut'.ljust(26) + 'gains'.rjust(10)
          + 'losses'.rjust(10) + 'series with a loss'.rjust(22))
    for ca in cuts:
        if ca == FULL_CUT:
            continue
        grp = [s for s in series if s['cut_A'] == ca]
        n_loss = sum(1 for s in grp if s['any_loss'])
        print(f'  cut_A {ca}'.ljust(26)
              + str(sum(s['total_gained'] for s in grp)).rjust(10)
              + str(sum(s['total_lost'] for s in grp)).rjust(10)
              + f'{n_loss}/{len(grp)}'.rjust(22))

    across = [r['across_m'] for r in lost if r['across_m'] is not None]
    along = [r['along_m'] for r in lost if r['along_m'] is not None]
    none_dir = sum(1 for r in lost if r['nearest_b_index'] is None)
    far = sum(1 for r in lost
              if r['dist_m'] is None or r['dist_m'] > NEAREST_LIMIT_M)
    print(f'\nlost elements: {len(lost)}')
    if across:
        print(f'  across to the nearest same-dir B element at k\': '
              f'median {_pct(across, 50):.4f} m, p90 {_pct(across, 90):.4f} m')
        print(f'  along:  median {_pct(along, 50):.4f} m, '
              f'p90 {_pct(along, 90):.4f} m')
    print(f'  within T1\'s tolerances (across <= eps, along <= rho/2): '
          f'{sum(1 for r in lost if r["within_t1"])}')
    print(f'  no same-direction B element at all: {none_dir}; '
          f'none within {NEAREST_LIMIT_M:.0f} m: {far}')


def s7_gainloss(by_kc: dict, spawn: dict, s6: dict, meta: dict,
                out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
                cuts: tuple = CUTS, n_robots: int = N_ROBOTS) -> dict:
    """The order that keeps every gain, loss and distance behind G5.

    The masks come first, then G5's two checks, and only then does anything
    count a gain or a loss for the record. On a G5 failure the stage writes
    the share comparisons, names the failing steps without their counts, and
    stops at exit 2: no reported gain, loss or distance is computed, written
    or printed, because none has been computed yet.

    Reads no file and resolves no spawn -- `run_s7` is this function plus the
    corpus I/O -- so the whole pipeline, G5's refusal included, runs on
    hand-built sides.
    """
    t0 = time.monotonic() if t0 is None else t0
    s6_rows = {(r['A'], r['B'], r['cut_A'], r['cut_B']): r for r in s6['rows']}
    keys = matrix_keys(cuts, n_robots)

    masks = {k: s7_mask(by_kc, spawn, k) for k in keys}
    rhos = {m['rho'] for m in masks.values()}
    assert len(rhos) == 1, rhos
    rho = rhos.pop()

    sks = series_keys(cuts, n_robots)
    assert len(sks) == N_SERIES, len(sks)
    for (a, b, ca) in sks:
        assert_aligned(masks, [(a, b, ca, cb) for cb in cuts])

    g5s = g5_shares(masks, s6_rows)
    print(f'G5: shares {g5s["n_equal"]}/{g5s["n"]} equal', end='')
    if not g5s['all_equal']:
        print('  FAIL')
        for c in g5s['differing'][:20]:
            a, b, ca, cb = c['key']
            print(f'  A=k{a} B=k{b} cut_A {ca} cut_B {cb}: mask '
                  f'{c["share_from_mask"]!r}  S6 {c["s6_corroborated"]!r}')
        fail = dict(meta)
        fail.update({'stage': 's7_g5_fail', 'seconds': time.monotonic() - t0,
                     'G5': {'shares': g5s, 'steps': None}})
        _no_gain_or_loss_in(fail)
        path, _ = _write_s7(fail, out_dir, 's7_g5_fail')
        die(f'G5 failed on {len(g5s["differing"])} of {g5s["n"]} shares; no '
            f'gain, loss or distance was computed, written or printed. {path}')

    g5t = g5_steps(masks, s6['P9']['failures'], rho)
    print(f', steps {g5t["n_equal"]}/{g5t["n"]} equal  '
          f'{"HOLD" if g5t["all_equal"] else "FAIL"}')
    if not g5t['all_equal']:
        for s in g5t['differing']:
            print(f'  A=k{s["A"]} B=k{s["B"]} cut_A {s["cut_A"]} '
                  f'{s["step"]}: S6\'s rise in elements is '
                  f'{s["s6_rise_in_elements"]} and S7 disagrees')
        fail = dict(meta)
        fail.update({'stage': 's7_g5_fail', 'seconds': time.monotonic() - t0,
                     'G5': {'shares': g5s, 'steps': g5t}})
        _no_gain_or_loss_in(fail)
        path, _ = _write_s7(fail, out_dir, 's7_g5_fail')
        die(f'G5 failed on {len(g5t["differing"])} of {g5t["n"]} of S6\'s '
            'failing P9 steps; no gain, loss or distance was computed, '
            f'written or printed. {path}')

    series = [gain_loss_series(masks, a, b, ca, cuts) for (a, b, ca) in sks]
    p10 = p10_verdict(series)
    lost = lost_elements(masks, series, cuts)

    print(f'P10: {"HOLD" if p10["holds"] else "FAIL"}  '
          f'{p10["n_with_loss"]}/{p10["of"]} of P3\'s series have a loss, '
          f'{p10["min_required"]} required')
    _print_s7(series, lost, cuts)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0, 'rho_m': rho,
        'n_series': len(series), 'n_rows': len(masks),
        'G5': {'shares': g5s, 'steps': g5t,
               'all_equal': g5s['all_equal'] and g5t['all_equal']},
        'series': series, 'lost_elements': lost, 'P10': p10})
    path, sha = _write_s7(payload, out_dir, 's7_gainloss')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s7(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """S7: gains and losses per step, G5 against S6, then P10. Spec v0.12.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before a map is loaded, as S5 and S6 do.
    2. REGISTRATION BEFORE RESULT: `9541ea1`, the commit that registered P10,
       must be an ancestor of HEAD, and the spec at HEAD must carry
       "P10. REGISTERED".
    3. The P10 paragraph is read from the spec AT HEAD by script and stored
       verbatim, never retyped.
    4. THE S6 FILE IS PINNED BY HASH. G5 is an equality against S6's values,
       so the file it compares against is named and its sha256 required; any
       other S6 run is a different set of numbers and G5 would be measuring
       the wrong thing.
    """
    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S7_REGISTRATION):
        die(f'{S7_REGISTRATION} registered P10 and is not an ancestor of HEAD '
            f'{head[:7] or "(unknown)"}; the registration must be in history '
            'behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    if P10_GATE_TEXT not in spec_text:
        die(f'the spec at HEAD carries no "{P10_GATE_TEXT}"; S7 evaluates a '
            'registered prediction or it does not run')
    p10_text = spec_paragraph(spec_text, P10_HEADER)
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s6_path = out_dir / S6_RESULT
    if not s6_path.is_file():
        die(f'{s6_path} is missing; G5 compares S7\'s masks against it')
    s6_sha = _sha256(s6_path)
    if s6_sha != S6_SHA256:
        die(f'{s6_path.name} has sha256 {s6_sha}, not the {S6_SHA256} G5 is '
            'stated against; that is a different S6 run')
    s6 = json.loads(s6_path.read_text())

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S7 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's7', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.12, G5/P10',
            'predictions_verbatim': {'P10': p10_text},
            'sources': {'s6': {'file': s6_path.name, 'sha256': s6_sha,
                               'head': s6.get('head')}},
            'note': 'no viewpoint weight and no truth: S7 computes the element '
                    'masks the measure makes and counts them'}
    return s7_gainloss(by_kc, spawn, s6, meta, out_dir, t0)


# ── S8: cell states under losses, G6 and P11 (spec v0.13, "Cell states under
#    losses (P11)") ────────────────────────────────────────────────────────────

S8_REGISTRATION = 'df7ae16'        # the commit that registered P11 (spec v0.13)
P11_HEADER = '**P11. REGISTERED'
P11_GATE_TEXT = 'P11. REGISTERED'
S7_RESULT = 's7_gainloss_20261009T000606Z.json'
S7_SHA256 = ('b8b7cb6c4dc57e8df48ea9baa975dc2e85b8eddbeed7e75c71fe52a271461c'
             '21')
N_LOST = 213                       # P11's denominator, stated in the wording

# P11's four classes, in the spec's order. The order is the classification:
# off grid is decided first, so a freed wall cell that is off the grid is never
# counted as freed.
OFF_GRID = 'off_grid'
WALL_FREED = 'wall_cell_freed'
FRONT_CLOSED = 'front_cell_closed'
UNCHANGED = 'unchanged'
P11_CLASSES = (OFF_GRID, WALL_FREED, FRONT_CLOSED, UNCHANGED)

# S7's three distance bins, the v0.13 "S7 results" definitions
BIN_WITHIN_EPS = 'within_eps_across'
BIN_BEYOND_EPS = 'beyond_eps_across'
BIN_FAR = 'none_within_1m'
BIN_WITHIN_T1 = 'within_t1'        # cannot occur; counted, never dropped
S7_BINS = (BIN_WITHIN_EPS, BIN_BEYOND_EPS, BIN_FAR)


def cell_centres_of(el: dict, i: int, rho: float) -> tuple:
    """Element `i`'s two cells, as centres in its own map frame.

    `face_elements` places an element half a cell from the OCCUPIED cell's
    centre towards the free neighbour: `x = cell_centre(ox, c) + 0.5*dcol*rho`
    and `y = cell_centre(oy, r) + 0.5*drow*rho`, the occupied cell being
    `(r, c)` and the free one `(r + drow, c + dcol)`. DIRS makes the normal the
    same pair on all four directions -- `(nx, ny) == (dcol, drow)` -- so both
    centres are the midpoint stepped half a cell along the element's own
    normal, with no per-direction case:

        wall cell  = (x - rho/2 * nx,  y - rho/2 * ny)    occupied at k
        front cell = (x + rho/2 * nx,  y + rho/2 * ny)    free at k

    Returns `(wall_centre, front_centre)`.
    """
    half = 0.5 * rho
    x, y = float(el['x'][i]), float(el['y'][i])
    nx, ny = float(el['nx'][i]), float(el['ny'][i])
    return ((x - half * nx, y - half * ny), (x + half * nx, y + half * ny))


def state_at(side: dict, pos: tuple) -> dict:
    """The state of the cell of `side`'s grid that CONTAINS `pos`.

    By position, never by index, as the spec requires: `trinary_map.cell_index`
    is this project's one position-to-index conversion -- the inverse of
    `cell_centre`, carrying the origin, the resolution and Karto's
    `CELL_CENTRE_OFFSET` -- and `trinary_map.sample` makes the same two calls
    internally. The grid is bottom-up (`load_grid`'s convention A), so the row
    index grows with y and the PGM row order is already undone.

    OFF THE GRID THE STATE IS None, NOT UNKNOWN. `sample` fills
    outside-the-map with UNKNOWN, which is right for a resample and wrong here:
    P11's first class turns on telling those two apart, so the bounds test is
    explicit and `sample` is not used for the lookup.

    `side` is a whole loaded side rather than a grid, so a caller -- or a
    test's spy -- sees WHICH map is being read before any state comes back.
    """
    grid, origin, rho = side['grid'], side['origin'], side['rho']
    h, w = grid.shape
    col = int(cell_index(pos[0], origin[0], rho))
    row = int(cell_index(pos[1], origin[1], rho))
    on = (0 <= row < h) and (0 <= col < w)
    return {'map': side['stem'], 'row': row, 'col': col, 'on_grid': on,
            'state': (int(grid[row, col]) if on else None)}


def state_name(state) -> str | None:
    """`occupied` / `free` / `unknown`, or None off the grid."""
    return None if state is None else {gw.OCC: 'occupied', gw.FREE: 'free',
                                       gw.UNKNOWN: 'unknown'}.get(state,
                                                                  'other')


def classify_cells(wall_kp: dict, front_kp: dict) -> str:
    """P11's four classes, decided in the spec's order."""
    if not (wall_kp['on_grid'] and front_kp['on_grid']):
        return OFF_GRID
    if wall_kp['state'] != gw.OCC:
        return WALL_FREED
    if front_kp['state'] != gw.FREE:
        return FRONT_CLOSED
    return UNCHANGED


def s7_distance_bin(rec: dict, eps: float, half: float,
                    limit: float = NEAREST_LIMIT_M) -> str:
    """Which of S7's three distance bins a lost element sits in.

    The v0.13 "S7 results" definitions, with the 1 m cut outermost so the bins
    are disjoint. An element inside both of T1's tolerances would have been
    corroborated and cannot occur; it gets its own label rather than being
    folded into a bin it does not belong to.
    """
    if rec['dist_m'] is None or rec['dist_m'] > limit:
        return BIN_FAR
    if rec['across_m'] <= eps and rec['along_m'] > half:
        return BIN_WITHIN_EPS
    if rec['across_m'] > eps:
        return BIN_BEYOND_EPS
    return BIN_WITHIN_T1


def lattice_offsets(by_kc: dict, cuts: tuple = CUTS,
                    n_robots: int = N_ROBOTS) -> list[dict]:
    """How far B's two lattices sit apart at each step, in cells, x and y.

    The fractional part of `(origin_k' - origin_k) / resolution`, mapped to
    [-0.5, 0.5). Zero means the two lattices coincide, so a cell centre at k is
    a cell centre at k'; anything else means it is not, which is the whole
    reason a state is read by position and never by index. Reported, not
    tested: it says how much of any freeing the lattice alone could account
    for.
    """
    out = []
    for k in range(n_robots):
        for i in range(len(cuts) - 1):
            lo, hi = by_kc[(k, cuts[i])], by_kc[(k, cuts[i + 1])]
            row = {'B': k, 'from_cut': cuts[i], 'to_cut': cuts[i + 1],
                   'step': f'cut{cuts[i]}->cut{cuts[i + 1]}',
                   'resolution': hi['rho']}
            for ax, name in ((0, 'x'), (1, 'y')):
                d = (hi['origin'][ax] - lo['origin'][ax]) / hi['rho']
                frac = d - math.floor(d)
                row[f'offset_{name}_cells'] = float(
                    frac - 1.0 if frac >= 0.5 else frac)
            out.append(row)
    return out


def b_matches(by_kc: dict, spawn: dict, keys) -> dict:
    """`corroborate_matches` on each row a lost element came from.

    S7's records name the nearest B element at k', never the element b at k
    that corroborated e, so b is recovered by recomputing the match with
    `corroborate_matches`, unchanged. One call per distinct
    (A, B, cut_A, k) row rather than one per element.

    IT NEEDS NO BEARING HISTOGRAM: its arguments are the two element lists,
    eps and rho. It does read `el_b['seg']`, which `s6_classify` sets, and that
    is what makes its tie-break "lowest B segment id" as the spec states
    rather than lowest array index.

    `match` indexes the FACE-ONLY subset `s6_classify` passes to it, so
    `face_to_full` maps it back to B's own element arrays. B's own-frame
    coordinates are then read straight from there, never by inverting the
    frame translation, which would round-trip through two floats for nothing.
    """
    out = {}
    for key in keys:
        res, el_b, a, b = s6_classify(by_kc, spawn, key)
        hit, match = corroborate_matches(a['el'], el_b, res['eps'], a['rho'])
        # the mask corroborate_matches produces has to be the mask the measure
        # used, or b is not the element that corroborated e
        assert np.array_equal(hit, res['corroborated']), key
        out[key] = {'match': match,
                    'face_to_full': np.nonzero(b['seg_of'] >= 0)[0],
                    'b_side': b, 'eps': res['eps']}
    return out


def g6_lost_set(recomputed: list, stored: list,
                n_expected: int = N_LOST) -> dict:
    """G6 part 1: S8's lost-element set must equal S7's exactly.

    The identity is (A, B, cut_A, step, element index). Duplicates would make
    a set comparison lie about the counts, so they are rejected rather than
    silently collapsed.
    """
    def key(r):
        return (r['A'], r['B'], r['cut_A'], r['step'], r['element'])

    mine, theirs = [key(r) for r in recomputed], [key(r) for r in stored]
    assert len(set(mine)) == len(mine), 'S8 produced a duplicate lost element'
    assert len(set(theirs)) == len(theirs), 'S7 holds a duplicate lost element'
    only_s8 = sorted(set(mine) - set(theirs))
    only_s7 = sorted(set(theirs) - set(mine))
    return {'n_recomputed': len(mine), 'n_stored': len(theirs),
            'n_expected': n_expected,
            'n_common': len(set(mine) & set(theirs)),
            'only_in_s8': [list(k) for k in only_s8],
            'only_in_s7': [list(k) for k in only_s7],
            'equal': set(mine) == set(theirs) and len(mine) == n_expected}


def g6_states_at_k(by_kc: dict, matches: dict, stored: list) -> dict:
    """G6 part 2: b's wall cell occupied and its front cell free, in B at k.

    Read the same way the k' states will be read, which is the point: if the
    lookup were wrong it would be wrong here, on cells whose states the measure
    already asserts, rather than silently wrong at k' where nothing is known
    yet. Opens no grid at k'.
    """
    rows, n_ok = [], 0
    for rec in stored:
        key = (rec['A'], rec['B'], rec['cut_A'], rec['from_cut_B'])
        m = matches[key]
        j = int(m['match'][rec['element']])
        assert j >= 0, (key, rec['element'])
        b_full = int(m['face_to_full'][j])
        side_k = m['b_side']
        wall, front = cell_centres_of(side_k['el'], b_full, side_k['rho'])
        s_wall, s_front = state_at(side_k, wall), state_at(side_k, front)
        ok = (s_wall['on_grid'] and s_front['on_grid']
              and s_wall['state'] == gw.OCC and s_front['state'] == gw.FREE)
        n_ok += int(ok)
        rows.append({
            'A': rec['A'], 'B': rec['B'], 'cut_A': rec['cut_A'],
            'step': rec['step'], 'element': rec['element'],
            'b_index': b_full, 'b_dir': int(side_k['el']['dir'][b_full]),
            'wall_cell': list(wall), 'front_cell': list(front),
            'wall_at_k': state_name(s_wall['state']),
            'front_at_k': state_name(s_front['state']),
            'wall_rc_at_k': [s_wall['row'], s_wall['col']],
            'front_rc_at_k': [s_front['row'], s_front['col']],
            'map_at_k': side_k['stem'], 'equal': bool(ok)})
    diffs = [r for r in rows if not r['equal']]
    return {'rows': rows, 'n': len(rows), 'n_ok': n_ok,
            'all_ok': not diffs, 'differing': diffs}


def p11_verdict(records: list, n_expected: int = N_LOST) -> dict:
    """P11, exactly as registered at spec v0.13.

    Wall cells freed in at least half of the 213 lost elements, so the test is
    `2 * n_freed >= 213`. OFF-GRID ELEMENTS STAY IN THE DENOMINATOR -- they
    count against P11, as the wording says twice -- so the denominator is
    asserted to be the full set rather than the on-grid subset.
    """
    assert len(records) == n_expected, len(records)
    by_class = {c: sum(1 for r in records if r['class'] == c)
                for c in P11_CLASSES}
    assert sum(by_class.values()) == n_expected, by_class
    freed = by_class[WALL_FREED]
    return {'of': n_expected, 'n_freed': freed,
            'min_required': -(-n_expected // 2),
            'holds': 2 * freed >= n_expected,
            'by_class': by_class}


def _no_kp_state_in(payload: dict, by_kc: dict) -> None:
    """Assert a G6 fail payload holds nothing from k'. The fail guard.

    A MAP CANNOT BE IDENTIFIED BY ITS STEM ALONE: B at cut240 is the k' map of
    the cut120 -> cut240 step and the k map of cut240 -> cut1200, so the same
    stem appears legitimately as a k read. The invariant that does hold is per
    row -- every state in the payload was read at THAT row's own k -- plus the
    absence of any k' field, which is where a k' state could hide.
    """
    banned = ('_kp', 'class', 'off_grid_cell')

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                assert not any(w in str(k) for w in banned), \
                    f'{path}.{k} is a k\' field'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')
    block = (payload.get('G6') or {}).get('states_at_k') or {}
    for r in block.get('rows', []):
        want = by_kc[(r['B'], _from_cut(r['step']))]['stem']
        assert r['map_at_k'] == want, (r['map_at_k'], want)


def _write_s8(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'{prefix}_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    sha = _sha256(path)
    print(f'written, overwriting nothing: {path}')
    print(f'sha256 {sha}')
    return path, sha


def _print_s8(records: list, p11: dict, bins: dict, offsets: list) -> None:
    """The classes by distance bin, the freed split, the lists, the offsets."""
    print(f'\nclasses by S7 distance bin{"":<6}'
          + ''.join(b.rjust(22) for b in S7_BINS) + 'total'.rjust(8))
    for c in P11_CLASSES:
        cells = [str(bins.get((c, b), 0)) for b in S7_BINS]
        total = sum(bins.get((c, b), 0) for b in S7_BINS)
        print(f'  {c:<30}' + ''.join(v.rjust(22) for v in cells)
              + str(total).rjust(8))
    extra = {k: v for k, v in bins.items() if k[1] not in S7_BINS}
    if extra:
        print(f'  outside the three bins: {extra}')

    freed = [r for r in records if r['class'] == WALL_FREED]
    split = {n: sum(1 for r in freed if r['wall_at_kp'] == n)
             for n in sorted({r['wall_at_kp'] for r in freed})}
    print(f'\nwall cells freed: {len(freed)}   by state at k\': {split}')

    for label, cls in (('unchanged', UNCHANGED), ('off grid', OFF_GRID)):
        sel = [r for r in records if r['class'] == cls]
        print(f'\n{label}: {len(sel)}')
        for r in sel:
            tail = (f'wall {r["wall_at_kp"]}, front {r["front_at_kp"]}'
                    if cls == UNCHANGED else
                    f'off by {r["off_grid_cell"]}')
            print(f'  A=k{r["A"]} B=k{r["B"]} cut_A {r["cut_A"]} {r["step"]}'
                  f'  element {r["element"]}  b {r["b_index"]}  {tail}')

    ax = [abs(o['offset_x_cells']) for o in offsets]
    ay = [abs(o['offset_y_cells']) for o in offsets]
    print(f'\nlattice offsets over {len(offsets)} steps, in cells: '
          f'|x| {min(ax):.4f} to {max(ax):.4f}, '
          f'|y| {min(ay):.4f} to {max(ay):.4f}')


def s8_cellstates(by_kc: dict, spawn: dict, s7: dict, meta: dict,
                  out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
                  cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
                  n_expected: int = N_LOST) -> dict:
    """The order that keeps the k' states unseen behind G6.

    G6's two parts run on the lost-element set and on B's grids AT K, and only
    when both hold does anything open a grid at k'. On a failure the stage
    writes the G6 comparisons and stops at exit 2, having opened no k' grid --
    the guard is the order of the code, and it is checked against the payload
    before the file is written.

    Reads no file and resolves no spawn; `run_s8` is this function plus the
    corpus I/O, so the whole pipeline runs on hand-built sides.
    """
    t0 = time.monotonic() if t0 is None else t0
    stored = s7['lost_elements']
    rho = s7['rho_m']
    eps, half = gw.EPS_FACTOR * rho, 0.5 * rho

    # ── G6 part 1: the lost-element set, recomputed with S7's own code ───────
    masks = {k: s7_mask(by_kc, spawn, k) for k in matrix_keys(cuts, n_robots)}
    series = [gain_loss_series(masks, a, b, ca, cuts)
              for (a, b, ca) in series_keys(cuts, n_robots)]
    recomputed = lost_elements(masks, series, cuts)
    g6a = g6_lost_set(recomputed, stored, n_expected)

    # ── G6 part 2: b, its two cells, and their states in B's grid AT K ──────
    keys_k = sorted({(r['A'], r['B'], r['cut_A'], r['from_cut_B'])
                     for r in stored})
    g6b = None
    if g6a['equal']:
        matches = b_matches(by_kc, spawn, keys_k)
        g6b = g6_states_at_k(by_kc, matches, stored)

    ok = g6a['equal'] and g6b is not None and g6b['all_ok']
    print(f'G6: lost set {g6a["n_common"]}/{g6a["n_expected"]} common'
          + ('' if g6b is None else
             f', states at k {g6b["n_ok"]}/{g6b["n"]} as required')
          + f'  {"HOLD" if ok else "FAIL"}')
    if not ok:
        if not g6a['equal']:
            print(f'  only in S8: {g6a["only_in_s8"][:10]}')
            print(f'  only in S7: {g6a["only_in_s7"][:10]}')
        else:
            for r in g6b['differing'][:20]:
                print(f'  A=k{r["A"]} B=k{r["B"]} cut_A {r["cut_A"]} '
                      f'{r["step"]} element {r["element"]}: wall '
                      f'{r["wall_at_k"]}, front {r["front_at_k"]}')
        fail = dict(meta)
        fail.update({'stage': 's8_g6_fail', 'seconds': time.monotonic() - t0,
                     'G6': {'lost_set': g6a, 'states_at_k': g6b}})
        _no_kp_state_in(fail, by_kc)
        path, _ = _write_s8(fail, out_dir, 's8_g6_fail')
        die('G6 failed; no grid at k\' was opened, and no cell state at k\' '
            f'was computed, written or printed. {path}')

    # ── the k' states, and the class ────────────────────────────────────────
    records = []
    for row in g6b['rows']:
        side_kp = by_kc[(row['B'], _to_cut(row['step']))]
        wall = tuple(row['wall_cell'])
        front = tuple(row['front_cell'])
        s_wall, s_front = state_at(side_kp, wall), state_at(side_kp, front)
        cls = classify_cells(s_wall, s_front)
        off = None
        if cls == OFF_GRID:
            off = ('both' if not (s_wall['on_grid'] or s_front['on_grid'])
                   else 'wall' if not s_wall['on_grid'] else 'front')
        rec = dict(row)
        rec.pop('equal', None)
        rec.update({'map_at_kp': side_kp['stem'],
                    'wall_at_kp': state_name(s_wall['state']),
                    'front_at_kp': state_name(s_front['state']),
                    'wall_rc_at_kp': [s_wall['row'], s_wall['col']],
                    'front_rc_at_kp': [s_front['row'], s_front['col']],
                    'wall_on_grid_at_kp': s_wall['on_grid'],
                    'front_on_grid_at_kp': s_front['on_grid'],
                    'class': cls, 'off_grid_cell': off})
        records.append(rec)

    p11 = p11_verdict(records, n_expected)
    print(f'P11: {"HOLD" if p11["holds"] else "FAIL"}  {p11["n_freed"]}/'
          f'{p11["of"]} wall cells freed, {p11["min_required"]} required')

    # ── reported, not tested ────────────────────────────────────────────────
    bin_of = {(r['A'], r['B'], r['cut_A'], r['step'], r['element']):
              s7_distance_bin(r, eps, half) for r in stored}
    for r in records:
        r['s7_bin'] = bin_of[(r['A'], r['B'], r['cut_A'], r['step'],
                              r['element'])]
    table = {}
    for r in records:
        table[(r['class'], r['s7_bin'])] = table.get(
            (r['class'], r['s7_bin']), 0) + 1
    offsets = lattice_offsets(by_kc, cuts, n_robots)
    _print_s8(records, p11, table, offsets)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0, 'rho_m': rho, 'eps_m': eps,
        'n_lost': len(records),
        'G6': {'lost_set': g6a, 'states_at_k': g6b,
               'all_ok': g6a['equal'] and g6b['all_ok']},
        'elements': records,
        'reported': {
            'class_by_bin': [{'class': c, 's7_bin': b, 'n': n}
                             for (c, b), n in sorted(table.items())],
            'wall_freed_by_state': {
                n: sum(1 for r in records
                       if r['class'] == WALL_FREED and r['wall_at_kp'] == n)
                for n in sorted({r['wall_at_kp'] for r in records
                                 if r['class'] == WALL_FREED})},
            'unchanged': [r for r in records if r['class'] == UNCHANGED],
            'off_grid': [r for r in records if r['class'] == OFF_GRID],
            'lattice_offsets': offsets},
        'P11': p11})
    path, sha = _write_s8(payload, out_dir, 's8_cellstates')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def _from_cut(step: str) -> int:
    """k out of a step label, which is how S7 writes the pair."""
    return int(step.split('->')[0].removeprefix('cut'))


def _to_cut(step: str) -> int:
    """k' out of a step label, which is how S7 writes the pair."""
    return int(step.split('->')[1].removeprefix('cut'))


def run_s8(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """S8: cell states under losses, G6 against S7, then P11. Spec v0.13.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before a map is loaded, as S5 to S7 do.
    2. REGISTRATION BEFORE RESULT: `df7ae16`, the commit that registered P11,
       must be an ancestor of HEAD, and the spec at HEAD must carry
       "P11. REGISTERED".
    3. The P11 paragraph is read from the spec AT HEAD by script and stored
       verbatim, never retyped.
    4. THE S7 FILE IS PINNED BY HASH. G6 is an equality against S7's 213 lost
       elements, so the file is named and its sha256 required; any other S7 run
       is a different set and G6 would be comparing against the wrong one.
    """
    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S8_REGISTRATION):
        die(f'{S8_REGISTRATION} registered P11 and is not an ancestor of HEAD '
            f'{head[:7] or "(unknown)"}; the registration must be in history '
            'behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    if P11_GATE_TEXT not in spec_text:
        die(f'the spec at HEAD carries no "{P11_GATE_TEXT}"; S8 evaluates a '
            'registered prediction or it does not run')
    p11_text = spec_paragraph(spec_text, P11_HEADER)
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s7_path = out_dir / S7_RESULT
    if not s7_path.is_file():
        die(f'{s7_path} is missing; G6 compares S8\'s lost elements against it')
    s7_sha = _sha256(s7_path)
    if s7_sha != S7_SHA256:
        die(f'{s7_path.name} has sha256 {s7_sha}, not the {S7_SHA256} G6 is '
            'stated against; that is a different S7 run')
    s7 = json.loads(s7_path.read_text())

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S8 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's8', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.13, G6/P11',
            'predictions_verbatim': {'P11': p11_text},
            'cell_centre': CELL_CENTRE_OFFSET,
            'sources': {'s7': {'file': s7_path.name, 'sha256': s7_sha,
                               'head': s7.get('head')}},
            'note': 'no viewpoint weight and no truth: S8 reads cell states by '
                    'position on the grids the measure itself builds'}
    return s8_cellstates(by_kc, spawn, s7, meta, out_dir, t0)


# ── S9: segmentation or geometry, G7 and P12 (spec v0.14, "Unchanged losses:
#    segmentation or geometry (P12)") ─────────────────────────────────────────

S9_REGISTRATION = '4698880'        # the commit that registered P12 (spec v0.14)
P12_HEADER = '**P12. REGISTERED'
P12_GATE_TEXT = 'P12. REGISTERED'
S8_RESULT = 's8_cellstates_20261009T010602Z.json'
S8_SHA256 = ('f71f4597d3e050417d65d092295b9ccf1ec0337bbf54af39868418da9c6c31c'
             '4')
N_UNCHANGED = 132                  # S8's unchanged class
N_P12_TESTED = 78                  # those outside S7's "none within 1 m" bin
N_P12_SEEN = 54                    # the carve-out, segmentation by construction

# P12's three classes, in the spec's order
SEGMENTATION = 'segmentation'
GEOM_ACROSS = 'geometry_across'
GEOM_ALONG = 'geometry_along'
P12_CLASSES = (SEGMENTATION, GEOM_ACROSS, GEOM_ALONG)


def find_e_prime(el: dict, wall_rc: tuple, dir_b: int) -> int | None:
    """B's face element at k' on the far side of `wall_rc`, in direction `dir_b`.

    BY CELL INDICES, because the element arrays carry them: `face_elements`
    builds each direction from `occ & shifted_view(free, drow, dcol)` and keeps
    `row` and `col` of the OCCUPIED cell, so `(row, col, dir)` names an element
    uniquely -- one pass of DIRS yields at most one element per occupied cell.
    No float comparison is needed, and none is used.

    `wall_rc` is the (row, col) of the cell of B's grid at k' that contains b's
    wall-cell centre, which is what S8 recorded as `wall_rc_at_kp`. B's FULL
    arrays are searched, not `face_elements_only`'s subset: whether e' is
    assigned is the question, so it cannot be the filter.

    Returns None when that cell pair carries no element at all, or none in b's
    direction -- the two cases G7 part 2 refuses.
    """
    hit = np.nonzero((el['row'] == wall_rc[0]) & (el['col'] == wall_rc[1])
                     & (el['dir'] == dir_b))[0]
    if hit.size == 0:
        return None
    assert hit.size == 1, (wall_rc, dir_b, hit)      # (row, col, dir) is a key
    return int(hit[0])


def e_prime_distances(a_side: dict, i: int, b_side: dict, j: int,
                      spawn_a, spawn_b) -> dict | None:
    """e''s across and along distances from A's element `i`, as T1 measures them.

    S9 COMPUTES NO DISTANCE OF ITS OWN. `nearest_same_dir` is handed a
    candidate set holding e' alone, so the nearest of that set is e' and the
    two projections come back from the same code S7 measured with -- `|d.n|`
    and `|d.t|` on A's own normal and tangent. The frame change is
    `to_frame_a`, the same one `s6_classify` applies, so the comparison happens
    in the common frame T1 works in.

    Returns None when e' is not in A's element's direction, which is
    `nearest_same_dir`'s own answer to that case and which G7 part 2 refuses.
    """
    bx, by = to_frame_a(b_side['el']['x'][j:j + 1], b_side['el']['y'][j:j + 1],
                        spawn_b, spawn_a)
    one = {'n': 1, 'x': bx, 'y': by,
           'nx': b_side['el']['nx'][j:j + 1],
           'ny': b_side['el']['ny'][j:j + 1],
           'dir': b_side['el']['dir'][j:j + 1]}
    return nearest_same_dir(a_side['el'], i, one)


def classify_unchanged(assigned: bool, across_m: float, along_m: float,
                       eps: float, rho: float) -> str:
    """P12's three classes, decided in the spec's order.

    Segmentation first -- an unassigned e' is never weighed geometrically,
    because T1 would not have looked at it either. Then across, then along,
    with both boundaries `within_t1_tolerances`' own, so eps and rho/2 are
    inclusive to 1e-9 m exactly as T1 has them.
    """
    if not assigned:
        return SEGMENTATION
    if across_m > eps + T1_TOL_M:
        return GEOM_ACROSS
    return GEOM_ALONG


def recompute_unchanged(by_kc: dict, spawn: dict, rho: float,
                        cuts: tuple = CUTS,
                        n_robots: int = N_ROBOTS) -> list[dict]:
    """S8's UNCHANGED class, recomputed from S7's and S8's own code.

    Extracted from `s9_segmentation`, where it was G7 part 1, so that S10's G8
    check 1 reproduces S9's rows through the same statements in the same order
    rather than through a second copy of them. A pure extraction: the calls,
    their order and the fields of each row are the ones S9 committed.

    The one addition is `s7_bin`, computed here with S8's own
    `s7_distance_bin` from the lost record it belongs to. S9 does not read it
    -- it takes each element's bin off the S8 file it pins, and compares only
    `g7_unchanged_set`'s own field list -- so S9's output is unchanged by it.
    S10 compares the two.
    """
    masks = {k: s7_mask(by_kc, spawn, k) for k in matrix_keys(cuts, n_robots)}
    series = [gain_loss_series(masks, a, b, ca, cuts)
              for (a, b, ca) in series_keys(cuts, n_robots)]
    lost = lost_elements(masks, series, cuts)
    keys_k = sorted({(r['A'], r['B'], r['cut_A'], r['from_cut_B'])
                     for r in lost})
    matches = b_matches(by_kc, spawn, keys_k)
    at_k = g6_states_at_k(by_kc, matches, lost)
    eps, half = gw.EPS_FACTOR * rho, 0.5 * rho
    bin_of = {(r['A'], r['B'], r['cut_A'], r['step'], r['element']):
              s7_distance_bin(r, eps, half) for r in lost}
    recomputed = []
    for row in at_k['rows']:
        side_kp = by_kc[(row['B'], _to_cut(row['step']))]
        s_wall = state_at(side_kp, tuple(row['wall_cell']))
        s_front = state_at(side_kp, tuple(row['front_cell']))
        if classify_cells(s_wall, s_front) != UNCHANGED:
            continue
        rec = dict(row)
        rec.pop('equal', None)
        rec.update({'wall_at_kp': state_name(s_wall['state']),
                    'front_at_kp': state_name(s_front['state']),
                    'wall_rc_at_kp': [s_wall['row'], s_wall['col']],
                    'front_rc_at_kp': [s_front['row'], s_front['col']],
                    's7_bin': bin_of[(row['A'], row['B'], row['cut_A'],
                                      row['step'], row['element'])]})
        recomputed.append(rec)
    return recomputed


def g7_unchanged_set(recomputed: list, stored: list,
                     n_expected: int = N_UNCHANGED) -> dict:
    """G7 part 1: S9's unchanged set and its cell states must equal S8's.

    STRICTER THAN THE SPEC, which asks only that "its set of unchanged
    elements equal S8's exactly". The four cell states go in too -- wall and
    front, at k and at k' -- because the class S9 explains is defined by them:
    an unchanged element whose states S9 reads differently is not the element
    S8 classified, and the set alone would not notice.
    """
    def key(r):
        return (r['A'], r['B'], r['cut_A'], r['step'], r['element'])

    mine = {key(r): r for r in recomputed}
    theirs = {key(r): r for r in stored}
    assert len(mine) == len(recomputed), 'S9 produced a duplicate'
    assert len(theirs) == len(stored), 'S8 holds a duplicate'

    fields = ('wall_at_k', 'front_at_k', 'wall_at_kp', 'front_at_kp',
              'b_index', 'b_dir', 'wall_rc_at_kp', 'front_rc_at_kp')
    rows, n_ok = [], 0
    for k in sorted(mine.keys() & theirs.keys()):
        diff = {f: [mine[k].get(f), theirs[k].get(f)] for f in fields
                if mine[k].get(f) != theirs[k].get(f)}
        n_ok += not diff
        if diff:
            rows.append({'key': list(k), 'differing_fields': diff})
    return {'n_recomputed': len(mine), 'n_stored': len(theirs),
            'n_expected': n_expected,
            'n_common': len(mine.keys() & theirs.keys()),
            'n_states_equal': n_ok,
            'only_in_s9': [list(k) for k in sorted(mine.keys() - theirs.keys())],
            'only_in_s8': [list(k) for k in sorted(theirs.keys() - mine.keys())],
            'differing_states': rows,
            'equal': (mine.keys() == theirs.keys() and not rows
                      and len(mine) == n_expected)}


def p12_verdict(records: list, n_tested: int = N_P12_TESTED,
                n_seen: int = N_P12_SEEN) -> dict:
    """P12, exactly as registered at spec v0.14.

    Of the unchanged elements OUTSIDE S7's "none within 1 m" bin, at least half
    fall in segmentation, so the test is `2 * n >= 78`, which is 39. The 54 in
    that bin are segmentation by construction -- S7's nearest-element search
    covers only B's assigned elements -- and are reported, labelled seen, and
    kept out of both the numerator and the denominator.
    """
    seen = [r for r in records if r['s7_bin'] == BIN_FAR]
    tested = [r for r in records if r['s7_bin'] != BIN_FAR]
    assert len(tested) == n_tested, len(tested)
    assert len(seen) == n_seen, len(seen)
    n_seg = sum(1 for r in tested if r['class'] == SEGMENTATION)
    return {
        'of': len(tested), 'min_required': -(-n_tested // 2),
        'n_segmentation': n_seg, 'holds': 2 * n_seg >= n_tested,
        'by_class': {c: sum(1 for r in tested if r['class'] == c)
                     for c in P12_CLASSES},
        'seen': {'of': len(seen), 'bin': BIN_FAR,
                 'n_segmentation': sum(1 for r in seen
                                       if r['class'] == SEGMENTATION),
                 'by_class': {c: sum(1 for r in seen if r['class'] == c)
                              for c in P12_CLASSES},
                 'note': 'segmentation by construction: S7 searched only B\'s '
                         'assigned elements, so these are reported, not tested'}}


def _no_verdict_in(payload: dict) -> None:
    """Assert a G7 fail payload carries no class count and no verdict.

    EXACT KEY NAMES, not substrings: G7's own part names say whether each gate
    held, and `4_carve_out_is_segmentation` is a gate verdict, not a count of
    the segmentation class. A substring ban cannot tell those apart, so the
    banned set is the keys a classification or a verdict would actually come
    under, plus the two blocks that only exist once the stage got that far.
    """
    banned = {'P12', 'class', 'by_class', 'n_segmentation', 'elements',
              'reported', 'e_prime_class', 'verdict'}

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                assert str(k) not in banned, \
                    f'{path}.{k} is a class count or a verdict'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')


def _write_s9(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'{prefix}_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, default=str))
    sha = _sha256(path)
    print(f'written, overwriting nothing: {path}')
    print(f'sha256 {sha}')
    return path, sha


def _print_s9(records: list, p12: dict, offsets: dict) -> None:
    """The 3 x 3 table, then every geometry element with its lattice offset."""
    bins = S7_BINS
    print(f'\nclasses by S7 distance bin{"":<4}'
          + ''.join(b.rjust(22) for b in bins) + 'total'.rjust(8))
    for c in P12_CLASSES:
        cells = [sum(1 for r in records
                     if r['class'] == c and r['s7_bin'] == b) for b in bins]
        print(f'  {c:<28}' + ''.join(str(v).rjust(22) for v in cells)
              + str(sum(cells)).rjust(8))
    print(f'  {"total":<28}'
          + ''.join(str(sum(1 for r in records if r['s7_bin'] == b)).rjust(22)
                    for b in bins) + str(len(records)).rjust(8))

    geom = [r for r in records if r['class'] != SEGMENTATION]
    print(f'\ngeometry elements: {len(geom)}')
    for r in geom:
        off = offsets.get((r['B'], r['step']), {})
        print(f'  A=k{r["A"]} B=k{r["B"]} cut_A {r["cut_A"]} {r["step"]}'
              f'  element {r["element"]}  e\' {r["e_prime"]}'
              f'  across {r["across_m"]:.4f} m  along {r["along_m"]:.4f} m'
              f'  {r["class"]}'
              f'  lattice ({off.get("offset_x_cells", float("nan")):+.4f}, '
              f'{off.get("offset_y_cells", float("nan")):+.4f}) cells')


def s9_segmentation(by_kc: dict, spawn: dict, s8: dict, meta: dict,
                    out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
                    cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
                    n_unchanged: int = N_UNCHANGED,
                    n_tested: int = N_P12_TESTED,
                    n_seen: int = N_P12_SEEN) -> dict:
    """The order that keeps e' unseen behind G7's four parts.

    Nothing is classified and no verdict is reached until all four hold. On a
    failure the stage writes the G7 comparisons and the elements that failed,
    and stops at exit 2, having written no class count -- which is checked
    against the payload before the file goes out.

    Reads no file and resolves no spawn; `run_s9` is this function plus the
    corpus I/O, so the whole pipeline runs on hand-built sides.
    """
    t0 = time.monotonic() if t0 is None else t0
    stored = [r for r in s8['elements'] if r['class'] == UNCHANGED]
    rho = s8['rho_m']
    eps = gw.EPS_FACTOR * rho
    offsets = {(o['B'], o['step']): o
               for o in s8['reported']['lattice_offsets']}

    # ── G7 part 1: S8's classification, recomputed with S8's own code ───────
    recomputed = recompute_unchanged(by_kc, spawn, rho, cuts, n_robots)
    g7a = g7_unchanged_set(recomputed, stored, n_unchanged)

    # ── G7 parts 2 and 3: e', its direction, its assignment, its distances ──
    rows, missing, wrong_dir, contradicting, carve_fail = [], [], [], [], []
    if g7a['equal']:
        by_key = {(r['A'], r['B'], r['cut_A'], r['step'], r['element']): r
                  for r in recomputed}
        for s in stored:
            k = (s['A'], s['B'], s['cut_A'], s['step'], s['element'])
            mine = by_key[k]
            a_side = by_kc[(s['A'], s['cut_A'])]
            b_side = by_kc[(s['B'], _to_cut(s['step']))]
            j = find_e_prime(b_side['el'], tuple(mine['wall_rc_at_kp']),
                             s['b_dir'])
            if j is None:
                missing.append({'key': list(k),
                                'wall_rc_at_kp': mine['wall_rc_at_kp'],
                                'b_dir': s['b_dir']})
                continue
            near = e_prime_distances(a_side, s['element'], b_side, j,
                                     spawn[s['A']], spawn[s['B']])
            if near is None:
                wrong_dir.append({'key': list(k), 'e_prime': j})
                continue
            assigned = bool(b_side['seg_of'][j] >= 0)
            inside = within_t1_tolerances(near['across_m'], near['along_m'],
                                          eps, rho)
            if assigned and inside:
                contradicting.append({
                    'key': list(k), 'e_prime': j,
                    'across_m': near['across_m'],
                    'along_m': near['along_m']})
            row = {'A': s['A'], 'B': s['B'], 'cut_A': s['cut_A'],
                   'step': s['step'], 'element': s['element'],
                   'b_index': s['b_index'], 'b_dir': s['b_dir'],
                   'e_prime': j, 'e_prime_seg': int(b_side['seg_of'][j]),
                   'assigned': assigned,
                   'across_m': near['across_m'], 'along_m': near['along_m'],
                   'within_t1': inside, 's7_bin': s['s7_bin'],
                   'map_at_kp': b_side['stem']}
            rows.append(row)
        # ── G7 part 4 ───────────────────────────────────────────────────────
        # STRICTER THAN THE SPEC. The spec calls the 54 "none within 1 m"
        # elements segmentation BY CONSTRUCTION and carves them out; it does
        # not ask anyone to check it. This checks it: S7 searched only B's
        # assigned elements, so an assigned e' within a metre of e would have
        # been found, and any assigned e' here means that reasoning is wrong.
        carve_fail = [{'key': [r['A'], r['B'], r['cut_A'], r['step'],
                               r['element']], 'e_prime': r['e_prime'],
                       'e_prime_seg': r['e_prime_seg'],
                       'across_m': r['across_m'], 'along_m': r['along_m']}
                      for r in rows if r['s7_bin'] == BIN_FAR and r['assigned']]

    g7 = {'unchanged_set': g7a,
          'n_e_prime_found': len(rows),
          'missing_e_prime': missing, 'wrong_direction': wrong_dir,
          'assigned_and_within_t1': contradicting,
          'carve_out_assigned': carve_fail,
          'parts': {'1_set_and_states': g7a['equal'],
                    '2_e_prime_exists': not (missing or wrong_dir),
                    '3_none_assigned_and_within_t1': not contradicting,
                    '4_carve_out_is_segmentation': not carve_fail}}
    g7['all_ok'] = all(g7['parts'].values())
    print('G7: ' + '  '.join(
        f'part {n.split("_")[0]} {"HOLD" if ok else "FAIL"}'
        for n, ok in g7['parts'].items())
        + f'   {"HOLD" if g7["all_ok"] else "FAIL"}')

    if not g7['all_ok']:
        for label, bad in (('only in S9', g7a['only_in_s9']),
                           ('only in S8', g7a['only_in_s8']),
                           ('states differ', g7a['differing_states'][:10]),
                           ('no e\'', missing[:10]),
                           ('wrong direction', wrong_dir[:10]),
                           ('assigned and within T1', contradicting[:10]),
                           ('carve-out assigned', carve_fail[:10])):
            if bad:
                print(f'  {label}: {bad}')
        fail = dict(meta)
        fail.update({'stage': 's9_g7_fail', 'seconds': time.monotonic() - t0,
                     'G7': g7})
        _no_verdict_in(fail)
        path, _ = _write_s9(fail, out_dir, 's9_g7_fail')
        die('G7 failed; no class count and no verdict were computed, written '
            f'or printed. {path}')

    # ── the classes, and the verdict ────────────────────────────────────────
    for r in rows:
        r['class'] = classify_unchanged(r['assigned'], r['across_m'],
                                        r['along_m'], eps, rho)
    p12 = p12_verdict(rows, n_tested, n_seen)
    print(f'P12: {"HOLD" if p12["holds"] else "FAIL"}  '
          f'{p12["n_segmentation"]}/{p12["of"]} segmentation, '
          f'{p12["min_required"]} required')
    print(f'carve-out (S7\'s "{BIN_FAR}", reported not tested): '
          f'{p12["seen"]["n_segmentation"]}/{p12["seen"]["of"]} '
          'in segmentation')
    _print_s9(rows, p12, offsets)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0, 'rho_m': rho, 'eps_m': eps,
        'n_unchanged': len(rows), 'G7': g7, 'elements': rows,
        'reported': {
            'class_by_bin': [
                {'class': c, 's7_bin': b,
                 'n': sum(1 for r in rows
                          if r['class'] == c and r['s7_bin'] == b)}
                for c in P12_CLASSES for b in S7_BINS],
            'geometry': [
                {**{k: r[k] for k in ('A', 'B', 'cut_A', 'step', 'element',
                                      'e_prime', 'across_m', 'along_m',
                                      'class', 's7_bin')},
                 'lattice_offset': offsets.get((r['B'], r['step']))}
                for r in rows if r['class'] != SEGMENTATION],
            'lattice_offsets': s8['reported']['lattice_offsets']},
        'P12': p12})
    path, sha = _write_s9(payload, out_dir, 's9_segmentation')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s9(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
           ) -> dict:
    """S9: segmentation or geometry, G7 against S8, then P12. Spec v0.14.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before a map is loaded, as S5 to S8 do.
    2. REGISTRATION BEFORE RESULT: `4698880`, the commit that registered P12,
       must be an ancestor of HEAD, and the spec at HEAD must carry
       "P12. REGISTERED".
    3. The P12 paragraph is read from the spec AT HEAD by script and stored
       verbatim, never retyped.
    4. THE S8 FILE IS PINNED BY HASH. G7 is an equality against S8's 132
       unchanged elements and their four cell states, so the file is named and
       its sha256 required.
    """
    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S9_REGISTRATION):
        die(f'{S9_REGISTRATION} registered P12 and is not an ancestor of HEAD '
            f'{head[:7] or "(unknown)"}; the registration must be in history '
            'behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    if P12_GATE_TEXT not in spec_text:
        die(f'the spec at HEAD carries no "{P12_GATE_TEXT}"; S9 evaluates a '
            'registered prediction or it does not run')
    p12_text = spec_paragraph(spec_text, P12_HEADER)
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s8_path = out_dir / S8_RESULT
    if not s8_path.is_file():
        die(f'{s8_path} is missing; G7 compares S9\'s unchanged elements '
            'against it')
    s8_sha = _sha256(s8_path)
    if s8_sha != S8_SHA256:
        die(f'{s8_path.name} has sha256 {s8_sha}, not the {S8_SHA256} G7 is '
            'stated against; that is a different S8 run')
    s8 = json.loads(s8_path.read_text())

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S9 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's9', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.14, G7/P12',
            'predictions_verbatim': {'P12': p12_text},
            'sources': {'s8': {'file': s8_path.name, 'sha256': s8_sha,
                               'head': s8.get('head')}},
            'note': "e' is found by cell indices in B's full element arrays at "
                    "k'; its distances come from nearest_same_dir on a "
                    "one-element set, so S9 computes no distance of its own"}
    return s9_segmentation(by_kc, spawn, s8, meta, out_dir, t0)


# ── S10: segments behind the unassigned elements, G8, P13a, P13b and P14
#    (spec v0.16, "Segments behind the unassigned elements (P13, P14)") ───────

S10_REGISTRATION = 'e15c94b'       # registered P13a, P13b, P14 (spec v0.16)
P13A_HEADER = '**P13a. REGISTERED'
P13B_HEADER = '**P13b. REGISTERED'
P14_HEADER = '**P14. REGISTERED'
P13A_GATE_TEXT = 'P13a. REGISTERED'
P13B_GATE_TEXT = 'P13b. REGISTERED'
P14_GATE_TEXT = 'P14. REGISTERED'
S9_RESULT = 's9_segmentation_20261009T013501Z.json'
S9_SHA256 = ('42c7505992ff8047d069cb7092dcdeaf3e8644193ff36484f3892425d307de'
             'ab')
N_SEGMENTATION = 129               # S9's segmentation class, of its 132
N_SEG_HOLE = 75                    # of those, the one-cell holes
N_SEG_FAR = 54                     # of those, the carve-out
N_EVENTS = 22                      # distinct (B, k, k', e') among the 129
N_EVENTS_HOLE, N_EVENTS_FAR = 14, 8          # of that 22, by loss shape

# "the same across coordinate, within 1e-9 m", the spec's face line. The same
# inclusive 1e-9 m T1's own two tolerances carry (T1_TOL_M).
FACE_LINE_TOL_M = 1e-9

# The two loss shapes P13a and P13b are registered on, which are S9's own split
# of the 129: "the 75 one-cell holes" and "the 54 losses with no assigned
# element within 1 m". The second shape IS S7's third distance bin, so it is
# that constant rather than a second spelling of it.
SHAPE_HOLE = 'one_cell_hole'
SHAPE_FAR = BIN_FAR
S10_SHAPES = (SHAPE_HOLE, SHAPE_FAR)

# The two states of S_b at k'.
CHANGED, INTACT = 'changed', 'intact'

# The counting units, and the fields that name them (§"Counting units").
EVENT_FIELDS = ('B', 'k', 'kp', 'e_prime')
UNIT_FIELDS = ('B', 'k', 'kp', 'seg_b')

# What G8 check 1 compares, row by row, against S9's stored values.
S9_ROW_FIELDS = ('b_index', 'b_dir', 'e_prime', 'e_prime_seg', 'assigned',
                 'across_m', 'along_m', 'within_t1', 's7_bin', 'class',
                 'map_at_kp')


def at_least_half(n: int) -> int:
    """The spec's "at least half (rounded up)" of `n`, which is ceil(n/2).

    Integer arithmetic, so there is no float to round the wrong way: for every
    n >= 0, `-(-n // 2)` is ceil(n/2), which at an odd n is the strictly larger
    half. P13a and P13b are both stated this way and neither denominator is
    known until the stage runs, so the threshold is computed and never
    written down.
    """
    return -(-n // 2)


def shape_of(s7_bin: str) -> str:
    """Which of P13's two loss shapes a segmentation loss has.

    S7's distance bin is what separates them: the carve-out P13a is about is
    `none_within_1m`, and every one of the other 75 is `within_eps_across`,
    the one-cell hole along the wall (spec v0.15, "S9 results"). G8 check 1
    holds that split to 75 and 54 and reproduces S9's whole class-by-bin
    table, so this mapping is checked rather than trusted.
    """
    return SHAPE_FAR if s7_bin == BIN_FAR else SHAPE_HOLE


def _s9_key(row: dict) -> tuple:
    """A loss's identity across S7, S8, S9 and S10: the A pairing included."""
    return (row['A'], row['B'], row['cut_A'], row['step'], row['element'])


def _unit_key(row: dict) -> tuple:
    """(B, k, k', S_b), P13a's and P13b's counting unit."""
    return tuple(row[f] for f in UNIT_FIELDS)


def _event_key(row: dict) -> tuple:
    """(B, k, k', e'), P14's counting unit."""
    return tuple(row[f] for f in EVENT_FIELDS)


def recompute_s9_rows(by_kc: dict, spawn: dict, rho: float,
                      cuts: tuple = CUTS,
                      n_robots: int = N_ROBOTS) -> list[dict]:
    """S9's own rows, recomputed with S9's own code: e', then P12's class.

    `recompute_unchanged` is G7 part 1; this adds G7 part 2's finder, its
    distances and P12's classifier, in S9's order and through S9's functions,
    so that G8 check 1 is an equality against values S9 published rather than
    a second opinion about them.

    A row whose e' is missing or faces another way keeps `e_prime` None and no
    class: S9's own G7 part 2 refused that case, so check 1 is where it
    surfaces -- as a row that does not reproduce -- and not as a crash here.
    """
    eps = gw.EPS_FACTOR * rho
    out = []
    for rec in recompute_unchanged(by_kc, spawn, rho, cuts, n_robots):
        a_side = by_kc[(rec['A'], rec['cut_A'])]
        b_side = by_kc[(rec['B'], _to_cut(rec['step']))]
        j = find_e_prime(b_side['el'], tuple(rec['wall_rc_at_kp']),
                         rec['b_dir'])
        near = (None if j is None else
                e_prime_distances(a_side, rec['element'], b_side, j,
                                  spawn[rec['A']], spawn[rec['B']]))
        row = {f: rec[f] for f in ('A', 'B', 'cut_A', 'step', 'element',
                                   'b_index', 'b_dir', 's7_bin')}
        row.update({'e_prime': j, 'map_at_kp': b_side['stem'],
                    'e_prime_seg': None, 'assigned': None, 'across_m': None,
                    'along_m': None, 'within_t1': None, 'class': None})
        if near is not None:
            assigned = bool(b_side['seg_of'][j] >= 0)
            row.update({
                'e_prime_seg': int(b_side['seg_of'][j]),
                'assigned': assigned,
                'across_m': near['across_m'], 'along_m': near['along_m'],
                'within_t1': within_t1_tolerances(near['across_m'],
                                                  near['along_m'], eps, rho),
                'class': classify_unchanged(assigned, near['across_m'],
                                            near['along_m'], eps, rho)})
        out.append(row)
    return out


def g8_s9_rows(recomputed: list, s9: dict, n_unchanged: int = N_UNCHANGED,
               n_segmentation: int = N_SEGMENTATION,
               n_hole: int = N_SEG_HOLE, n_far: int = N_SEG_FAR,
               n_events: int = N_EVENTS, n_events_hole: int = N_EVENTS_HOLE,
               n_events_far: int = N_EVENTS_FAR) -> dict:
    """G8 check 1: S9's stored rows, classes and counts reproduce exactly.

    STRICTER THAN "all 132 rows and their classes". The identity of e' and of
    b goes in too, and so do the two distances: S10's units are built out of
    exactly those -- S_b is the segment of b at k, the counting unit is e' at
    k', the loss shape is S7's bin -- and a row whose class S10 agrees with
    while naming a different e' is not the loss S9 reported. A class-only
    comparison would not notice. The distances are compared the way G4 and G5
    compare a stored share: exactly, with no tolerance, because the same code
    on the same pinned maps has no reason to move by an ulp.

    The counts are the registered ones, not whatever came back: 132 rows, 129
    segmentation split 75 and 54, and 22 distinct (B, k, k', e') among them,
    14 and 8. S9's stored class-by-bin table goes in as a whole.
    """
    mine = {_s9_key(r): r for r in recomputed}
    theirs = {_s9_key(r): r for r in s9['elements']}
    assert len(mine) == len(recomputed), 'S10 produced a duplicate row'
    assert len(theirs) == len(s9['elements']), 'S9 holds a duplicate row'

    differing = []
    for k in sorted(mine.keys() & theirs.keys()):
        diff = {f: [mine[k].get(f), theirs[k].get(f)] for f in S9_ROW_FIELDS
                if mine[k].get(f) != theirs[k].get(f)}
        if diff:
            differing.append({'key': list(k), 'differing_fields': diff})

    table_mine = {(c, b): sum(1 for r in recomputed
                              if r['class'] == c and r['s7_bin'] == b)
                  for c in P12_CLASSES for b in S7_BINS}
    table_s9 = {(r['class'], r['s7_bin']): r['n']
                for r in s9['reported']['class_by_bin']}
    table_ok = table_mine == table_s9

    seg = [r for r in recomputed if r['class'] == SEGMENTATION]
    far = [r for r in seg if r['s7_bin'] == BIN_FAR]
    hole = [r for r in seg if r['s7_bin'] != BIN_FAR]

    def distinct(rows):
        return {(r['B'], _from_cut(r['step']), _to_cut(r['step']),
                 r['e_prime']) for r in rows}

    counts = {
        'n_rows': len(recomputed), 'n_rows_expected': n_unchanged,
        'n_segmentation': len(seg), 'n_segmentation_expected': n_segmentation,
        'n_one_cell_hole': len(hole), 'n_one_cell_hole_expected': n_hole,
        'n_none_within_1m': len(far), 'n_none_within_1m_expected': n_far,
        'n_distinct_e_prime': len(distinct(seg)),
        'n_distinct_e_prime_expected': n_events,
        'n_distinct_e_prime_one_cell_hole': len(distinct(hole)),
        'n_distinct_e_prime_one_cell_hole_expected': n_events_hole,
        'n_distinct_e_prime_none_within_1m': len(distinct(far)),
        'n_distinct_e_prime_none_within_1m_expected': n_events_far,
    }
    counts_ok = all(counts[n] == counts[f'{n}_expected'] for n in (
        'n_rows', 'n_segmentation', 'n_one_cell_hole', 'n_none_within_1m',
        'n_distinct_e_prime', 'n_distinct_e_prime_one_cell_hole',
        'n_distinct_e_prime_none_within_1m'))

    return {
        'n_recomputed': len(mine), 'n_stored': len(theirs),
        'n_common': len(mine.keys() & theirs.keys()),
        'n_rows_equal': len(mine.keys() & theirs.keys()) - len(differing),
        'only_in_s10': [list(k) for k in sorted(mine.keys() - theirs.keys())],
        'only_in_s9': [list(k) for k in sorted(theirs.keys() - mine.keys())],
        'differing_rows': differing,
        'class_by_bin': [{'class': c, 's7_bin': b, 'n': n}
                         for (c, b), n in sorted(table_mine.items())],
        'class_by_bin_equal': table_ok,
        'counts': counts, 'counts_equal': counts_ok,
        'ok': (mine.keys() == theirs.keys() and not differing
               and table_ok and counts_ok),
    }


def g8_assignments(by_kc: dict, rows: list[dict]) -> dict:
    """G8 check 2: b is assigned at k and e' is unassigned at k', on all 129.

    b's segment is read here and nowhere else, because it IS S_b's identity:
    every unit P13a and P13b count is keyed by it. `seg_of` runs parallel to
    B's FULL element arrays, index for index (`extract_with_elements` builds it
    off `face_elements`), so `seg_of[b_index]` at k and `seg_of[e_prime]` at k'
    are the two assignments the spec names -- and reading either off
    `face_elements_only`'s subset would silently name another element.

    Opens no cell at k': a failure here leaves `changed` and `flanked` unseen.
    """
    out, bad = [], []
    for r in rows:
        side_k, side_kp = by_kc[(r['B'], r['k'])], by_kc[(r['B'], r['kp'])]
        seg_b = int(side_k['seg_of'][r['b_index']])
        seg_e = int(side_kp['seg_of'][r['e_prime']])
        ok = seg_b >= 0 and seg_e < 0
        rec = {f: r[f] for f in ('A', 'B', 'cut_A', 'step', 'element', 'k',
                                 'kp', 'b_index', 'b_dir', 'e_prime')}
        rec.update({'seg_b': seg_b, 'seg_e_prime': seg_e,
                    'b_assigned_at_k': seg_b >= 0,
                    'e_prime_assigned_at_kp': seg_e >= 0,
                    'map_at_k': side_k['stem'], 'map_at_kp': side_kp['stem'],
                    'ok': ok})
        out.append(rec)
        if not ok:
            bad.append(rec)
    return {'rows': out, 'n': len(out),
            'n_ok': sum(1 for r in out if r['ok']), 'not_ok': bad,
            'ok': bool(out) and not bad}


def g8_segments(by_kc: dict, assign_rows: list[dict]) -> dict:
    """G8 check 3: every S_b is non-empty and holds b.

    MEMBERS COME OFF `seg_of`, not off the segment dict: `_finish` strips
    `_members` before `graph_and_elements` returns, and `seg_of == id` is the
    very membership it was built from (`extract_with_elements` sets
    `seg_of[members] = seg_id`, and an assigned element is never unassigned).
    The segment's own `n_elements` and `id` are checked against that, so the
    two disagreeing is a failure rather than a silent choice between them.

    Opens no cell at k' either.
    """
    out, bad = [], []
    for r in assign_rows:
        side_k = by_kc[(r['B'], r['k'])]
        segs = side_k['segments']
        seg = segs[r['seg_b']] if 0 <= r['seg_b'] < len(segs) else None
        members = np.nonzero(side_k['seg_of'] == r['seg_b'])[0]
        holds_b = bool(r['b_index'] in set(members.tolist()))
        rec = {f: r[f] for f in ('B', 'k', 'kp', 'b_index', 'seg_b')}
        rec.update({
            'n_members': int(members.size), 'holds_b': holds_b,
            'n_elements_agrees': seg is not None
            and int(seg['n_elements']) == int(members.size),
            'id_agrees': seg is not None and int(seg['id']) == r['seg_b'],
            'map_at_k': side_k['stem']})
        rec['ok'] = bool(members.size and holds_b and rec['n_elements_agrees']
                         and rec['id_agrees'])
        out.append(rec)
        if not rec['ok']:
            bad.append(rec)
    return {'rows': out, 'n': len(out),
            'n_ok': sum(1 for r in out if r['ok']), 'not_ok': bad,
            'ok': bool(out) and not bad}


def member_states_at_kp(side_k: dict, side_kp: dict, member: int) -> dict:
    """One member of S_b, its two cells at k', and the route it took.

    The cells are b's two cells, for every member: `cell_centres_of` gives
    them in B's own frame at k -- the frame k' shares, both maps being B's own
    and each anchored at B's spawn -- and `state_at` reads the cell of k' that
    CONTAINS each centre, by position, through `trinary_map.cell_index`.

    `trinary_map.sample` IS NEVER USED FOR THAT LOOKUP. Off the grid it fills
    UNKNOWN, which would turn a member that left the map into a freed wall
    cell; off grid and freed are separate routes here, exactly as they are
    separate classes in P11. `classify_cells` decides which, in P11's own
    order, so OFF GRID wins for a member whose cell is gone.
    """
    wall, front = cell_centres_of(side_k['el'], member, side_k['rho'])
    s_wall, s_front = state_at(side_kp, wall), state_at(side_kp, front)
    return {'member': int(member),
            'route': classify_cells(s_wall, s_front),
            'wall_at_kp': state_name(s_wall['state']),
            'front_at_kp': state_name(s_front['state']),
            'wall_on_grid_at_kp': s_wall['on_grid'],
            'front_on_grid_at_kp': s_front['on_grid'],
            'x': float(side_k['el']['x'][member]),
            'y': float(side_k['el']['y'][member])}


def segment_change(side_k: dict, side_kp: dict, seg_id: int) -> dict:
    """Is segment `seg_id` of B at k changed at k', and by which routes?

    CHANGED is the spec's disjunction over S_b's members -- some member's wall
    cell not occupied, or some member's front cell not free, or some member's
    cell off grid -- and INTACT is its negation, which is "every element of
    S_b at k then still exists at k'". More than one route can occur, so the
    routes are a set over the members and never one class.

    This is the one function that reads a member's cells at k', which is what
    makes a spy on it a proof that a gate failure computed no `changed`.
    """
    members = np.nonzero(side_k['seg_of'] == seg_id)[0]
    rows = [member_states_at_kp(side_k, side_kp, int(m)) for m in members]
    moved = [r for r in rows if r['route'] != UNCHANGED]
    return {'seg': int(seg_id), 'n_members': int(members.size),
            'changed': bool(moved), 'state': CHANGED if moved else INTACT,
            'n_changed_members': len(moved),
            'routes': [c for c in P11_CLASSES if c != UNCHANGED
                       and any(r['route'] == c for r in moved)],
            'changed_members': moved,
            'map_at_k': side_k['stem'], 'map_at_kp': side_kp['stem']}


def along_on_line(theta_deg: float, x0: float, y0: float,
                  x1: np.ndarray, y1: np.ndarray) -> np.ndarray:
    """Distance ALONG a line of angle `theta_deg`, from (x0, y0) to each point.

    `|d . u|` with `u = dir_of(theta)`, which is the projection T1 calls along
    and the one `_geometry` orders a run's members by. The sign of u does not
    matter: a '-y' face line reports theta 180 as readily as 0, and this is a
    distance.
    """
    ux, uy = gw.dir_of(theta_deg)
    return np.abs((x1 - x0) * ux + (y1 - y0) * uy)


def nearest_changed_member(seg: dict, change: dict, x: float,
                           y: float) -> dict | None:
    """The changed member of S_b nearest to (x, y) ALONG S_b's own line.

    Reported, not tested. (x, y) is e''s midpoint at k' and the members sit at
    k, both in B's own frame; the line is S_b's reported line at k, which is
    what "the distance along the line" names. None when nothing changed.
    """
    if not change['changed']:
        return None
    moved = change['changed_members']
    d = along_on_line(seg['theta'], x, y,
                      np.array([r['x'] for r in moved]),
                      np.array([r['y'] for r in moved]))
    i = int(np.argmin(d))
    return {'member': moved[i]['member'], 'route': moved[i]['route'],
            'along_m': float(d[i])}


def face_line_of(el: dict, i: int, tol: float = FACE_LINE_TOL_M) -> dict:
    """Element `i`'s face line: the same direction, the same across coordinate.

    ACROSS is the projection on the element's own normal and ALONG the
    projection on its tangent, which are T1's two axes. Normals are
    axis-aligned by construction (DIRS), so one of the two is x and the other
    y. "Within 1e-9 m" is read as an inclusive band around `i`'s own across
    coordinate, the same inclusive 1e-9 m T1's tolerances carry.
    """
    d = int(el['dir'][i])
    nx, ny = float(el['nx'][i]), float(el['ny'][i])
    same = np.nonzero(el['dir'] == d)[0]
    across_i = float(el['x'][i]) * nx + float(el['y'][i]) * ny
    on_line = same[np.abs(el['x'][same] * nx + el['y'][same] * ny
                          - across_i) <= tol]
    return {'members': on_line, 'dir': d, 'across_i': across_i, 'tol_m': tol,
            'along': -el['x'][on_line] * ny + el['y'][on_line] * nx,
            'along_i': -float(el['x'][i]) * ny + float(el['y'][i]) * nx}


def is_flanked(el: dict, seg_of: np.ndarray, i: int,
               tol: float = FACE_LINE_TOL_M) -> dict:
    """Does element `i` lie STRICTLY between two members of ONE segment?

    The audit's consequence P14 tests. Such an element is within eps of that
    segment's line -- its across distance lies between the pair's -- faces the
    line as they do, is alive since it was never assigned, and falls inside
    their run with no gap of g or more, so it would have been a member. A
    flanked e' means the audit missed a rule.

    ONE segment, BOTH sides: a member strictly below `i` along the face line
    AND a member strictly above it, with the same `seg_of >= 0`. Strictly,
    with no tolerance of its own -- an element at `i`'s own along coordinate is
    beside it, not either side of it -- and the unassigned are not a segment,
    so `seg_of < 0` is skipped rather than treated as one group.
    """
    line = face_line_of(el, i, tol)
    sides: dict[int, list[bool]] = {}
    for j, t in zip(line['members'].tolist(), line['along'].tolist()):
        s = int(seg_of[j])
        if j == i or s < 0:
            continue
        below_above = sides.setdefault(s, [False, False])
        if t < line['along_i']:
            below_above[0] = True
        elif t > line['along_i']:
            below_above[1] = True
    flanking = sorted(s for s, (below, above) in sides.items()
                      if below and above)
    return {'flanked': bool(flanking), 'flanking_segments': flanking,
            'n_on_face_line': int(line['members'].size),
            'segments_on_face_line': sorted(sides), 'tol_m': tol}


def distinct_groups(rows: list[dict], fields: tuple) -> list[dict]:
    """One group per distinct value of `fields`, sorted, with its rows.

    WHY THE UNIT IS NOT THE LOSS. Changed, intact and flanked depend on B, its
    two cuts and the element, never on A, and S9's 129 rows repeat 22 events 2
    to 14 times, once per A pairing. A count per loss would therefore weight
    each event by how many As it was matched against, so P13a, P13b and P14
    are counted on distinct keys and the per-loss counts are reported beside
    them.
    """
    groups: dict[tuple, dict] = {}
    for r in rows:
        key = tuple(r[f] for f in fields)
        groups.setdefault(key, {'key': list(key), 'rows': []})
        groups[key]['rows'].append(r)
    return [groups[k] for k in sorted(groups)]


def loss_rows(stored: list[dict]) -> list[dict]:
    """S9's segmentation losses, each with k, k' and its loss shape.

    The units are read from the S9 file the stage pins, as the spec says they
    are; G8 check 1 is what shows the file's values reproduce.
    """
    return [{**{f: r[f] for f in ('A', 'B', 'cut_A', 'step', 'element',
                                  'b_index', 'b_dir', 'e_prime', 's7_bin')},
             'k': _from_cut(r['step']), 'kp': _to_cut(r['step']),
             'shape': shape_of(r['s7_bin'])}
            for r in stored if r['class'] == SEGMENTATION]


def p13_verdict(units: list[dict], shape: str, outcome: str) -> dict:
    """P13a or P13b, exactly as registered at spec v0.16.

    Counted on distinct (B, k, k', S_b), with "at least half (rounded up)"
    being `at_least_half`. AN S_b BEHIND LOSSES OF BOTH SHAPES COUNTS IN BOTH,
    as the spec says, so the selection is membership in the unit's shape SET
    and the two denominators may overlap.

    The denominator is asserted non-empty: with no unit at all the threshold
    would be 0 and the prediction would "hold" having been tested on nothing.
    It is not asserted to be any particular number -- unlike P12's 78, how
    many distinct S_b sit behind the 129 is not registered and was unknown
    when this was written.
    """
    sel = [u for u in units if shape in u['shapes']]
    assert sel, (shape, outcome)
    n_hit = sum(1 for u in sel if u['state'] == outcome)
    need = at_least_half(len(sel))
    return {'shape': shape, 'outcome': outcome, 'of': len(sel), 'n': n_hit,
            'min_required': need, 'holds': n_hit >= need,
            'by_state': {s: sum(1 for u in sel if u['state'] == s)
                         for s in (CHANGED, INTACT)},
            'units': [u['key'] for u in sel]}


def p14_verdict(events: list[dict], n_expected: int = N_EVENTS) -> dict:
    """P14, exactly as registered at spec v0.16: NONE of the 22 e' is flanked.

    "P14 holds only at 0 of 22", so the test is `n_flanked == 0` and there is
    no threshold to round. The 8 events behind the carve-out hold by
    construction -- a flanking pair would put an assigned element within 1 m --
    and the spec asks for them to be checked rather than assumed, so they sit
    in the denominator and are reported separately as well.
    """
    assert len(events) == n_expected, len(events)
    flanked = [e for e in events if e['flanked']]
    return {'of': len(events), 'n_flanked': len(flanked),
            'holds': not flanked,
            'by_shape': {s: {'of': sum(1 for e in events if e['shape'] == s),
                             'n_flanked': sum(1 for e in flanked
                                              if e['shape'] == s)}
                         for s in S10_SHAPES},
            'flanked': [{'key': e['key'], 'shape': e['shape'],
                         'flanking_segments': e['flanking_segments']}
                        for e in flanked]}


def base_rate(by_kc: dict, steps: list[tuple]) -> list[dict]:
    """Reported, not tested: the share of ALL B's segments at k changed at k'.

    One row per B step that carries at least one of the 129, over every
    segment of B's map at k and not only the S_b behind a loss. Without it,
    "changed" could simply be what most segments do between two cuts, and P13a
    would be reading a background rate as a finding.
    """
    out = []
    for (b, k, kp) in steps:
        side_k, side_kp = by_kc[(b, k)], by_kc[(b, kp)]
        changes = [segment_change(side_k, side_kp, int(s['id']))
                   for s in side_k['segments']]
        n_ch = sum(1 for c in changes if c['changed'])
        out.append({
            'B': b, 'from_cut_B': k, 'to_cut_B': kp,
            'step': f'cut{k}->cut{kp}', 'map_at_k': side_k['stem'],
            'map_at_kp': side_kp['stem'], 'n_segments': len(changes),
            'n_changed': n_ch,
            'share_changed': (n_ch / len(changes)) if changes else None,
            'by_route': {r: sum(1 for c in changes if r in c['routes'])
                         for r in P11_CLASSES if r != UNCHANGED}})
    return out


def _no_change_in(payload: dict) -> None:
    """Assert a G8 fail payload carries no changed, intact or flanked.

    EXACT KEY NAMES, which is the lesson G7's guard cost a rewrite: a
    substring ban cannot tell a gate's own part name from a quantity. What is
    unseen at S10 is `changed`, `intact` and `flanked` and everything built out
    of them, including the base rate. S9's classes are NOT banned -- they are
    published (spec v0.15) and check 1 is an equality against them, so a fail
    file is exactly where they belong.
    """
    banned = {'changed', 'intact', 'flanked', 'state', 'routes', 'by_route',
              'n_changed', 'n_changed_members', 'changed_members',
              'flanking_segments', 'n_on_face_line', 'segments_on_face_line',
              'nearest_changed_member', 'base_rate', 'share_changed',
              'events', 'segment_units', 'by_state', 'P13a', 'P13b', 'P14',
              'reported', 'verdict'}

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                assert str(k) not in banned, \
                    f'{path}.{k} is an unseen quantity'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')


def _write_s10(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    """S9's writer, already generic in its prefix, under this stage's name.

    A dated file, never an overwrite, and the sha256 printed. The call site
    says which stage is writing, which is the whole reason for the name.
    """
    return _write_s9(payload, out_dir, prefix)


def _print_s10(units: list[dict], events: list[dict], rates: list[dict],
               per_loss: dict) -> None:
    """The two tables the verdicts are read off, then the base rate."""
    print('\ndistinct S_b by loss shape and state at k\'')
    print(f'  {"loss shape":<34}{CHANGED:>10}{INTACT:>10}{"total":>10}')

    def line(label, sel):
        n_ch = sum(1 for u in sel if u['changed'])
        print(f'  {label:<34}{n_ch:>10}{len(sel) - n_ch:>10}{len(sel):>10}')

    for s in S10_SHAPES:
        line(s, [u for u in units if s in u['shapes']])
    line('(of those, behind both shapes)',
         [u for u in units if len(u['shapes']) == 2])
    line('all distinct S_b', units)

    print('\nper loss, reported not judged: '
          + '  '.join(f'{s} {v["of"]} ({v[CHANGED]} {CHANGED}, '
                      f'{v[INTACT]} {INTACT}, {v["flanked"]} flanked)'
                      for s, v in per_loss['by_shape'].items()))

    print('\ndistinct e\' (B, k, k\', e\')')
    for e in events:
        near = e['nearest_changed_member']
        tail = ('' if near is None else
                f'  nearest changed member {near["member"]} '
                f'{near["along_m"]:.4f} m along ({near["route"]})')
        print(f'  B=k{e["B"]} {e["step"]} e\' {e["e_prime"]}  b {e["b_index"]}'
              f'  S_b {e["seg_b"]} ({e["n_members"]} members, '
              f'{e["length_m"]:.2f} m)  {e["shape"]}  {e["state"]}'
              f'  flanked {e["flanked"]}'
              + (f'  routes {",".join(e["routes"])}' if e['routes'] else '')
              + tail)

    print('\nbase rate: all of B\'s segments at k, changed at k\'')
    print(f'  {"B, step":<34}{"segments":>10}{CHANGED:>10}{"share":>10}')
    for r in rates:
        share = ('-' if r['share_changed'] is None
                 else f'{r["share_changed"]:.4f}')
        label = f'B=k{r["B"]} {r["step"]}'
        print(f'  {label:<34}{r["n_segments"]:>10}'
              f'{r["n_changed"]:>10}{share:>10}')


def s10_derive(by_kc: dict, rows: list[dict], assign_rows: list[dict],
               n_events: int = N_EVENTS) -> dict:
    """Everything S10 computes once G8 holds: units, events, verdicts, report.

    Lifted out of `s10_segments` whole, so that S11's G9 check 1 reproduces
    S10's stored values through the statements S10 itself runs and not
    through a copy of them. It prints nothing and writes nothing; the caller
    does both. `rows` are the loss rows and gain `seg_b` here, as before.
    """
    # ── S_b per loss, then one change read per distinct (B, k, k', S_b) ─────
    seg_b_of = {_s9_key(r): r['seg_b'] for r in assign_rows}
    for r in rows:
        r['seg_b'] = seg_b_of[_s9_key(r)]

    unit_groups = distinct_groups(rows, UNIT_FIELDS)
    change_of = {}
    for u in unit_groups:
        b, k, kp, seg = u['key']
        change_of[tuple(u['key'])] = segment_change(
            by_kc[(b, k)], by_kc[(b, kp)], seg)

    units = []
    for u in unit_groups:
        b, k, kp, seg = u['key']
        ch = change_of[tuple(u['key'])]
        side_k = by_kc[(b, k)]
        units.append({
            'key': list(u['key']), 'B': b, 'k': k, 'kp': kp,
            'step': f'cut{k}->cut{kp}', 'seg_b': seg,
            # in S10_SHAPES order, not sorted: an S_b behind losses of both
            # shapes carries both, and the order is the spec's, not the
            # alphabet's
            'shapes': [s for s in S10_SHAPES
                       if any(r['shape'] == s for r in u['rows'])],
            'n_losses': len(u['rows']),
            'n_e_prime': len({r['e_prime'] for r in u['rows']}),
            'state': ch['state'], 'changed': ch['changed'],
            'routes': ch['routes'],
            'n_changed_members': ch['n_changed_members'],
            'n_members': ch['n_members'],
            'length_m': float(side_k['segments'][seg]['length']),
            'map_at_k': ch['map_at_k'], 'map_at_kp': ch['map_at_kp']})

    events = []
    for g in distinct_groups(rows, EVENT_FIELDS):
        b, k, kp, j = g['key']
        side_k, side_kp = by_kc[(b, k)], by_kc[(b, kp)]
        # b, its direction, S_b and the shape are functions of the event, not
        # of A: a row disagreeing would mean one e' with two b's behind it
        fixed = {(r['b_index'], r['b_dir'], r['seg_b'], r['shape'])
                 for r in g['rows']}
        assert len(fixed) == 1, (g['key'], fixed)
        b_index, b_dir, seg, shape = fixed.pop()
        ch = change_of[(b, k, kp, seg)]
        flank = is_flanked(side_kp['el'], side_kp['seg_of'], j)
        events.append({
            'key': list(g['key']), 'B': b, 'k': k, 'kp': kp,
            'step': f'cut{k}->cut{kp}', 'e_prime': j, 'b_index': b_index,
            'b_dir': b_dir, 'seg_b': seg, 'shape': shape,
            'n_losses': len(g['rows']),
            'losses': [[r['A'], r['cut_A'], r['element']] for r in g['rows']],
            'state': ch['state'], 'changed': ch['changed'],
            'routes': ch['routes'],
            'n_changed_members': ch['n_changed_members'],
            'n_members': ch['n_members'],
            'length_m': float(side_k['segments'][seg]['length']),
            'nearest_changed_member': nearest_changed_member(
                side_k['segments'][seg], ch,
                float(side_kp['el']['x'][j]), float(side_kp['el']['y'][j])),
            'flanked': flank['flanked'],
            'flanking_segments': flank['flanking_segments'],
            'n_on_face_line': flank['n_on_face_line'],
            'segments_on_face_line': flank['segments_on_face_line'],
            'map_at_k': side_k['stem'], 'map_at_kp': side_kp['stem']})

    # ── the three verdicts, judged on the distinct units ───────────────────
    p13a = p13_verdict(units, SHAPE_FAR, CHANGED)
    p13b = p13_verdict(units, SHAPE_HOLE, INTACT)
    p14 = p14_verdict(events, n_events)

    # ── reported, not tested ───────────────────────────────────────────────
    flanked_of = {tuple(e['key']): e['flanked'] for e in events}
    per_loss = {
        'note': 'per loss, i.e. once per A pairing: reported beside the '
                'verdicts and never judged, since changed, intact and '
                'flanked do not depend on A',
        'of': len(rows),
        'by_shape': {
            s: {'of': sum(1 for r in rows if r['shape'] == s),
                CHANGED: sum(1 for r in rows if r['shape'] == s
                             and change_of[_unit_key(r)]['changed']),
                INTACT: sum(1 for r in rows if r['shape'] == s
                            and not change_of[_unit_key(r)]['changed']),
                'flanked': sum(1 for r in rows if r['shape'] == s
                               and flanked_of[_event_key(r)])}
            for s in S10_SHAPES}}
    steps = sorted({(r['B'], r['k'], r['kp']) for r in rows})
    rates = base_rate(by_kc, steps)
    return {
        'n_losses': len(rows), 'n_segment_units': len(units),
        'n_events': len(events),
        'losses': rows, 'segment_units': units, 'events': events,
        'P13a': p13a, 'P13b': p13b, 'P14': p14,
        'reported': {
            'changed_losses': [
                {k: e[k] for k in ('key', 'B', 'step', 'e_prime', 'seg_b',
                                   'shape', 'routes', 'n_changed_members',
                                   'n_members', 'nearest_changed_member')}
                for e in events if e['changed']],
            # "for every loss, the number of members of S_b at k and its
            # length": per distinct S_b, which covers every loss, with the
            # number of losses behind it beside so a per-loss reading is
            # recoverable without weighting a verdict by A
            'segment_sizes': [
                {k: u[k] for k in ('key', 'B', 'step', 'seg_b', 'n_members',
                                   'length_m', 'shapes', 'state', 'n_losses',
                                   'n_e_prime')}
                for u in units],
            'per_loss': per_loss,
            'base_rate': rates}}


def s10_segments(by_kc: dict, spawn: dict, s9: dict, meta: dict,
                 out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
                 cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
                 n_unchanged: int = N_UNCHANGED,
                 n_segmentation: int = N_SEGMENTATION,
                 n_hole: int = N_SEG_HOLE, n_far: int = N_SEG_FAR,
                 n_events: int = N_EVENTS,
                 n_events_hole: int = N_EVENTS_HOLE,
                 n_events_far: int = N_EVENTS_FAR) -> dict:
    """The order that keeps changed, intact and flanked unseen behind G8.

    G8's three checks run first, and not one cell of a member of S_b is read
    at k' until all three hold: check 1 reproduces S9, check 2 reads two
    `seg_of` entries per loss, check 3 counts S_b's members. On a failure the
    stage writes the G8 comparisons and stops at exit 2, having computed no
    `changed`, no `intact` and no `flanked` -- the guard is the order of the
    code, and the payload is walked for those names before it is written.

    `segment_change` runs ONCE PER DISTINCT (B, k, k', S_b), not once per loss:
    the three quantities do not depend on A, and running it per row would read
    the same cells up to 14 times and invite a count weighted by A pairings.

    Reads no file and resolves no spawn; `run_s10` is this function plus the
    corpus I/O, so the whole pipeline runs on hand-built sides.
    """
    t0 = time.monotonic() if t0 is None else t0
    rho = s9['rho_m']

    # ── G8 check 1: S9's own rows, recomputed with S9's own code ────────────
    recomputed = recompute_s9_rows(by_kc, spawn, rho, cuts, n_robots)
    c1 = g8_s9_rows(recomputed, s9, n_unchanged, n_segmentation, n_hole,
                    n_far, n_events, n_events_hole, n_events_far)

    # ── G8 checks 2 and 3: the two assignments, then S_b itself ────────────
    rows = loss_rows(s9['elements']) if c1['ok'] else []
    c2 = g8_assignments(by_kc, rows) if c1['ok'] else None
    c3 = g8_segments(by_kc, c2['rows']) if (c2 and c2['ok']) else None

    g8 = {'1_s9_rows_reproduce': c1,
          '2_b_assigned_e_prime_unassigned': c2,
          '3_segment_holds_b': c3,
          'parts': {'1_s9_rows_reproduce': c1['ok'],
                    '2_b_assigned_e_prime_unassigned': bool(c2 and c2['ok']),
                    '3_segment_holds_b': bool(c3 and c3['ok'])}}
    g8['all_ok'] = all(g8['parts'].values())
    print('G8: ' + '  '.join(
        f'check {n.split("_")[0]} {"HOLD" if ok else "FAIL"}'
        for n, ok in g8['parts'].items())
        + f'   {"HOLD" if g8["all_ok"] else "FAIL"}')

    if not g8['all_ok']:
        for label, bad in (
                ('only in S10', c1['only_in_s10']),
                ('only in S9', c1['only_in_s9']),
                ('rows differing', c1['differing_rows'][:10]),
                ('counts', None if c1['counts_equal'] else c1['counts']),
                ('class by bin', None if c1['class_by_bin_equal']
                 else c1['class_by_bin']),
                ('assignments', None if c2 is None else c2['not_ok'][:10]),
                ('segments', None if c3 is None else c3['not_ok'][:10])):
            if bad:
                print(f'  {label}: {bad}')
        fail = dict(meta)
        fail.update({'stage': 's10_g8_fail', 'seconds': time.monotonic() - t0,
                     'G8': g8})
        _no_change_in(fail)
        path, _ = _write_s10(fail, out_dir, 's10_g8_fail')
        die('G8 failed; no cell of a member of S_b was read at k\', and no '
            'changed, intact or flanked was computed, written or printed. '
            f'{path}')

    # ── S_b, the change per distinct unit, the verdicts and the report ─────
    d = s10_derive(by_kc, rows, c2['rows'], n_events)
    units, events = d['segment_units'], d['events']
    p13a, p13b, p14 = d['P13a'], d['P13b'], d['P14']
    for name, v in (('P13a', p13a), ('P13b', p13b)):
        print(f'{name}: {"HOLD" if v["holds"] else "FAIL"}  {v["n"]}/'
              f'{v["of"]} distinct S_b behind the {v["shape"]} losses are '
              f'{v["outcome"]}, {v["min_required"]} required')
    print(f'P14: {"HOLD" if p14["holds"] else "FAIL"}  {p14["n_flanked"]}/'
          f'{p14["of"]} distinct e\' flanked, 0 required')

    _print_s10(units, events, d['reported']['base_rate'],
               d['reported']['per_loss'])

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0, 'rho_m': rho,
        'eps_m': gw.EPS_FACTOR * rho, 'face_line_tol_m': FACE_LINE_TOL_M,
        'G8': g8, **d})
    path, sha = _write_s10(payload, out_dir, 's10_segments')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s10(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
            ) -> dict:
    """S10: the segments behind the unassigned elements. Spec v0.16.

    G8 against S9, then P13a, P13b and P14, judged separately.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before anything is loaded, as S5 to S9 do.
    2. REGISTRATION BEFORE RESULT: `e15c94b`, the commit that registered P13a,
       P13b and P14, must be an ancestor of HEAD, and the spec at HEAD must
       carry all three "REGISTERED" headers.
    3. The three paragraphs are read from the spec AT HEAD by script and
       stored verbatim, never retyped.
    4. THE S9 FILE IS PINNED BY HASH. G8 check 1 is an equality against S9's
       132 rows, their classes and its class-by-bin table, so the file is
       named and its sha256 required; any other S9 run is a different set.

    NOTHING IS LOADED UNTIL ALL FOUR HOLD. The two function-scope imports sit
    below them rather than at the top of the body, so a refusal costs no map,
    no graph JSON and no spawn table.

    S_b's members come from `load_side`, which rebuilds B's elements with
    `graph_and_elements` -- `g` from that map's own graph JSON `thresholds.g`,
    and the map's sha256 checked against the JSON -- exactly as S5 to S9 do.
    """
    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S10_REGISTRATION):
        die(f'{S10_REGISTRATION} registered P13a, P13b and P14 and is not an '
            f'ancestor of HEAD {head[:7] or "(unknown)"}; the registration '
            'must be in history behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    for gate in (P13A_GATE_TEXT, P13B_GATE_TEXT, P14_GATE_TEXT):
        if gate not in spec_text:
            die(f'the spec at HEAD carries no "{gate}"; S10 evaluates three '
                'registered predictions or it does not run')
    verbatim = {'P13a': spec_paragraph(spec_text, P13A_HEADER),
                'P13b': spec_paragraph(spec_text, P13B_HEADER),
                'P14': spec_paragraph(spec_text, P14_HEADER)}
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s9_path = out_dir / S9_RESULT
    if not s9_path.is_file():
        die(f'{s9_path} is missing; G8 check 1 reproduces its rows and the '
            'units are read from it')
    s9_sha = _sha256(s9_path)
    if s9_sha != S9_SHA256:
        die(f'{s9_path.name} has sha256 {s9_sha}, not the {S9_SHA256} G8 is '
            'stated against; that is a different S9 run')
    s9 = json.loads(s9_path.read_text())

    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S10 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's10', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.16, G8/P13a/P13b/P14',
            'predictions_verbatim': verbatim,
            'cell_centre': CELL_CENTRE_OFFSET,
            'sources': {'s9': {'file': s9_path.name, 'sha256': s9_sha,
                               'head': s9.get('head')}},
            'note': "S_b is the segment of seg_of[b] at k, its members read "
                    "off seg_of; every member's two cells are found at k' by "
                    'position with trinary_map.cell_index and never with '
                    'trinary_map.sample. No viewpoint weight is computed'}
    return s10_segments(by_kc, spawn, s9, meta, out_dir, t0)


# ── S11: the rule that drops each e', G9, P15 and P16
#    (spec v0.17, "The rule that drops each e' (P15, P16)") ──────────────────

S11_REGISTRATION = 'dc754de'       # registered P15 and P16 (spec v0.17)
P15_HEADER = '**P15. REGISTERED'
P16_HEADER = '**P16. REGISTERED'
P15_GATE_TEXT = 'P15. REGISTERED'
P16_GATE_TEXT = 'P16. REGISTERED'
S10_RESULT = 's10_segments_20261011T134922Z.json'
S10_SHA256 = ('fa750aa97fc08c5a1f177b78019832c3c5794ef42a8898d9d8f6d1d7127407'
              'e2')
P15_MIN_R3 = 11                    # "at least 11 of the 22 distinct e'"

# The five classes, in the order the spec decides them: no home first, then
# the first of R3, R4 and R5+R6 that applies at H, then Other.
NO_HOME = 'no_home'
R3_LINE = 'R3_line'
R4_FACING = 'R4_facing'
R56_SHORT_RUN = 'R5+R6_short_run'
OTHER = 'other'
DROP_CLASSES = (NO_HOME, R3_LINE, R4_FACING, R56_SHORT_RUN, OTHER)

# What an iteration that seeded a window holding the element did with it.
LATER_ABORTED = 'R7_aborted_before_runs'
LATER_BEYOND_EPS = 'R3_beyond_eps'
LATER_NOT_FACING = 'R4_not_facing'
LATER_SHORT_RUN = 'R5+R6_rejected_run'
LATER_ASSIGNED = 'assigned'
LATER_OUTCOMES = (LATER_ABORTED, LATER_BEYOND_EPS, LATER_NOT_FACING,
                  LATER_SHORT_RUN, LATER_ASSIGNED)

# The sections of S10's result file G9 check 1 reproduces: "the verdicts, the
# 19 S_b and their states, the 22 e' rows, and the base rate" are in these.
S10_COMPARED = ('n_losses', 'n_segment_units', 'n_events', 'losses',
                'segment_units', 'events', 'P13a', 'P13b', 'P14', 'reported')


def _as_stored(node):
    """`node` as `_write_s9` would store it and `json.loads` read it back."""
    return json.loads(json.dumps(node, sort_keys=True, default=str))


def _row_order_free(node):
    """`node` with every list of rows put in one canonical order.

    S10's loss rows follow the order of S9's file, and three of its lists
    inherit that order: `losses`, each event's `losses`, and the rows of G8
    checks 2 and 3. The order of rows is not a stored VALUE, so check 1
    compares those lists as sorted lists -- every row still has to be there,
    equal field for field and as many times -- and everything else as it is.
    """
    def key(v):
        return json.dumps(v, sort_keys=True)

    if isinstance(node, dict):
        return {k: (sorted((_row_order_free(x) for x in v), key=key)
                    if k in ('losses', 'rows') and isinstance(v, list)
                    else _row_order_free(v))
                for k, v in node.items()}
    if isinstance(node, list):
        return [_row_order_free(v) for v in node]
    return node


def g9_s10_reproduces(by_kc: dict, spawn: dict, s10: dict,
                      cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
                      n_events: int = N_EVENTS,
                      s9_sha: str | None = None) -> tuple[dict, dict | None]:
    """G9 check 1: S10's stored values, through the functions S10 calls.

    `recompute_s9_rows`, `loss_rows`, G8's checks 2 and 3 and `s10_derive`,
    which is the body of `s10_segments` itself -- and no `_write_s10`, so no
    second S10 result file appears. S9's file is not read: the loss rows are
    the recomputed ones, and they are held equal to the 129 S10 stored, which
    it read from S9's file. `s9_sha`, when given, is the S9 hash S10's file
    must name, so the chain behind the pinned file is the pinned chain.

    Compared exactly, with no tolerance: every section of `S10_COMPARED`, and
    G8's checks 2 and 3. Returns the check and what was derived, which is
    None unless the derivation ran.
    """
    rho = s10['rho_m']
    recomputed = recompute_s9_rows(by_kc, spawn, rho, cuts, n_robots)
    rows = loss_rows(recomputed)
    c2 = g8_assignments(by_kc, rows)
    c3 = g8_segments(by_kc, c2['rows']) if c2['ok'] else None

    derived, asserted = None, None
    if c3 is not None and c3['ok']:
        try:
            derived = s10_derive(by_kc, rows, c2['rows'], n_events)
        except AssertionError as e:
            asserted = repr(e)

    sections = {}
    if derived is not None:
        for name in S10_COMPARED:
            sections[name] = (
                _row_order_free({name: _as_stored(derived[name])})
                == _row_order_free({name: s10.get(name)}))
    stored_g8 = s10.get('G8', {})
    g8_equal = {
        name: (mine is not None and _row_order_free(_as_stored(mine))
               == _row_order_free(stored_g8.get(name)))
        for name, mine in (('2_b_assigned_e_prime_unassigned', c2),
                           ('3_segment_holds_b', c3))}
    named = (s10.get('sources', {}).get('s9', {}).get('sha256')
             if s9_sha is not None else None)
    check = {
        'n_loss_rows_recomputed': len(rows),
        'n_loss_rows_stored': len(s10.get('losses', [])),
        'g8_assignments_ok': c2['ok'],
        'g8_segments_ok': bool(c3 and c3['ok']),
        'derivation_asserted': asserted,
        # a list, not a dict keyed by section: the section names are S10's
        # own payload keys, and a fail file is walked for some of them
        'sections_compared': [{'section': n, 'equal': eq}
                              for n, eq in sections.items()],
        'sections_differing': sorted(n for n, eq in sections.items()
                                     if not eq),
        'g8_checks_equal': g8_equal,
        's9_sha256_named_by_s10': named,
        's9_sha256_expected': s9_sha,
        'wrote_an_s10_file': False,
    }
    check['ok'] = bool(
        derived is not None and all(sections.values())
        and len(sections) == len(S10_COMPARED) and all(g8_equal.values())
        and (s9_sha is None or named == s9_sha))
    return check, derived


def trace_extraction(run, tracer_factory=None) -> dict:
    """One map's extraction twice: tracer off, then tracer on.

    `run(tracer)` returns `(graph, el, seg_of)` and is the SAME call both
    times apart from that one argument, which is what G9 check 2 is a
    statement about.
    """
    tracer = (tracer_factory or gw.Tracer)()
    g0, el0, s0 = run(None)
    g1, el1, s1 = run(tracer)
    return {'off': {'graph': g0, 'el': el0, 'seg_of': s0},
            'on': {'graph': g1, 'el': el1, 'seg_of': s1},
            'tracer': tracer}


def trace_grid(side: dict, params: gw.Params | None = None,
               tracer_factory=None) -> dict:
    """`trace_extraction` on a side's own grid; what the synthetic cases use."""
    params = gw.Params() if params is None else params
    return trace_extraction(
        lambda t: gw.extract_with_elements(side['grid'], side['rho'],
                                           side['origin'], params, t),
        tracer_factory)


def trace_stem(side: dict) -> dict:
    """`trace_extraction` on a corpus map, built exactly as `load_side` builds.

    `Params(g)` with g from the map's own graph JSON `thresholds.g`, and the
    map's sha256 checked against that JSON on both runs. A different g is a
    different extraction, and the trace would describe a run S10 never saw.
    """
    stem = side['stem']
    out_json = gw.OUT_DIR_DEFAULT / f'{stem}.json'
    if not out_json.is_file():
        die(f'{stem}: no graph JSON at {out_json}; graph_walls writes it')
    saved = json.loads(out_json.read_text())
    params = gw.Params(g=saved['thresholds']['g']['value'])

    def run(tracer):
        graph, el, seg_of, _grid = gw.graph_and_elements(
            stem, params=params, g_source=f'{out_json.name} thresholds.g',
            tracer=tracer)
        if graph['map_sha256'] != saved['map_sha256']:
            die(f'{stem}: map sha256 {graph["map_sha256"][:12]} is not the '
                f'JSON\'s {saved["map_sha256"][:12]}; the JSON describes '
                'another map')
        return graph, el, seg_of

    return trace_extraction(run)


def _dumped(node) -> str:
    return json.dumps(node, sort_keys=True, default=str)


def _same_elements(a: dict, b: dict) -> bool:
    return a['n'] == b['n'] and all(
        np.array_equal(a[k], b[k])
        for k in ('x', 'y', 'nx', 'ny', 'row', 'col', 'dir'))


def g9_non_interference(by_kc: dict, traced: dict) -> dict:
    """G9 check 2: per traced map, tracer on equals tracer off, exactly.

    The registered comparison is `seg_of` and the segment list. Two things
    are held beside it and both are STRICTER THAN THE SPEC. The tracer-off run
    must be the loaded side S10's reproduction read -- same elements, same
    `seg_of`, same segments -- or an e' index would name another element in
    the trace. And the trace must pass its own `self_check`: one record per
    iteration on every exit path, every seedable bin accounted for, the
    recorded members exactly the assigned elements.

    `whole_graph_equal` is reported and does not gate.
    """
    rows = []
    for (b, c) in sorted(traced):
        t, side = traced[(b, c)], by_kc[(b, c)]
        off, on = t['off'], t['on']
        rec = {
            'B': b, 'cut': c, 'map': side['stem'],
            'n_elements': int(on['el']['n']),
            'n_segments': len(on['graph']['segments']),
            'seg_of_equal': bool(
                off['seg_of'].shape == on['seg_of'].shape
                and np.array_equal(off['seg_of'], on['seg_of'])),
            'segments_equal': (_dumped(off['graph']['segments'])
                               == _dumped(on['graph']['segments'])),
            'whole_graph_equal': _dumped(off['graph']) == _dumped(on['graph']),
            'off_is_the_loaded_side': bool(
                _same_elements(side['el'], off['el'])
                and _same_elements(side['el'], on['el'])
                and np.array_equal(side['seg_of'], off['seg_of'])
                and _dumped(side['segments'])
                == _dumped(off['graph']['segments'])),
            'trace_self_check': t['tracer'].self_check()}
        rec['ok'] = bool(rec['seg_of_equal'] and rec['segments_equal']
                         and rec['off_is_the_loaded_side']
                         and rec['trace_self_check']['ok'])
        rows.append(rec)
    return {'rows': rows, 'n': len(rows),
            'n_ok': sum(1 for r in rows if r['ok']),
            'not_ok': [r for r in rows if not r['ok']],
            'ok': bool(rows) and all(r['ok'] for r in rows)}


def g9_unassigned(traced: dict, events: list[dict]) -> dict:
    """G9 check 3, first half: every e' is unassigned in the traced run.

    Three readings of one fact, all required: the traced run's `seg_of`, the
    `seg_of` the tracer copied at the end of that run, and no accepted run of
    any recorded iteration holding e'.
    """
    rows = []
    for e in events:
        t = traced[(e['B'], e['kp'])]
        j = e['e_prime']
        rec = {'B': e['B'], 'k': e['k'], 'kp': e['kp'], 'e_prime': j,
               'seg_of_traced_run': int(t['on']['seg_of'][j]),
               'seg_of_recorded': int(t['tracer'].seg_of[j]),
               'assigning_iteration': int(t['tracer'].assigned_in()[j])}
        rec['ok'] = (rec['seg_of_traced_run'] < 0
                     and rec['seg_of_recorded'] < 0
                     and rec['assigning_iteration'] < 0)
        rows.append(rec)
    return {'rows': rows, 'n': len(rows),
            'n_ok': sum(1 for r in rows if r['ok']),
            'not_ok': [r for r in rows if not r['ok']],
            'ok': bool(rows) and all(r['ok'] for r in rows)}


def g9_one_class(classes: list, n_events: int = N_EVENTS) -> dict:
    """G9 check 3, second half: every e' lands in exactly one class.

    A post-condition on the classification, so it is the one check that runs
    after `classify_drop`; a failure still prints and writes no class. One
    class per e', each a member of `DROP_CLASSES`, and as many as there are
    registered e'.
    """
    bad = [c for c in classes if c not in DROP_CLASSES]
    return {'n': len(classes), 'n_expected': n_events,
            'n_not_a_registered_label': len(bad),
            'ok': len(classes) == n_events and not bad}


def counterparts(side_k: dict, side_kp: dict, seg_id: int) -> list[dict]:
    """Each member of S_b, and the element of k' that is its counterpart.

    The spec's definition: the element at k' with m's direction, in the cell
    found at m's wall-cell position with `trinary_map.cell_index`. `state_at`
    is that lookup -- the one S8 and S10 use, and never `trinary_map.sample`
    -- and `find_e_prime` names the element by (row, col, dir). None where the
    cell is off k''s grid or carries no element in that direction.
    """
    out = []
    for m in np.nonzero(side_k['seg_of'] == seg_id)[0].tolist():
        wall, _front = cell_centres_of(side_k['el'], m, side_k['rho'])
        cell = state_at(side_kp, wall)
        j = (find_e_prime(side_kp['el'], (cell['row'], cell['col']),
                          int(side_k['el']['dir'][m]))
             if cell['on_grid'] else None)
        out.append({'member': int(m), 'counterpart': j})
    return out


def home_iteration(assigned_in: np.ndarray, cps: list[dict]) -> dict:
    """H: the iteration at k' that assigned the most counterparts of S_b.

    Ties go to the earliest iteration. `H` is None when no counterpart is
    assigned at k', which is "e' has no home".
    """
    tally: dict[int, int] = {}
    for c in cps:
        if c['counterpart'] is None:
            continue
        it = int(assigned_in[c['counterpart']])
        if it >= 0:
            tally[it] = tally.get(it, 0) + 1
    home = min(tally, key=lambda it: (-tally[it], it)) if tally else None
    return {'H': home, 'n_members': len(cps),
            'n_counterparts': sum(1 for c in cps
                                  if c['counterpart'] is not None),
            'n_counterparts_assigned': sum(tally.values()),
            'n_counterparts_assigned_by_H': tally.get(home, 0),
            'counterparts_by_iteration': [[it, tally[it]]
                                          for it in sorted(tally)]}


def refit_line_of(rec: dict) -> tuple:
    """(theta, d) of an iteration's OWN refit, or (None, None) before it.

    The spec's "refitted line": the one the iteration's second candidate pass
    tests against (`_refit`'s result in `extract_with_elements`), not the line
    `_segment` may go on to report for a segment cut on it.
    """
    return rec['theta_line'], rec['d_line']


def element_at(tracer, it: int, e: int) -> dict:
    """The registered per-iteration record of element `e` at iteration `it`.

    The seed; whether e voted in the seed window; its distance to the
    first-pass line and to the refitted line; whether it was a candidate of
    the refitted line; and the run that held it, that run's length and whether
    it was accepted. A field the iteration never reached is None, and an
    element that was not a candidate has no run.

    Distance and facing are evaluated SEPARATELY, off the recorded lines with
    `line_candidates`' own two expressions, because the candidate mask is
    their conjunction and cannot say which of R3 and R4 excluded an element.
    """
    rec = tracer.iterations[it]
    out = {'iteration': int(it), 'exit': rec['exit'],
           'seed_theta_index': rec['theta_index'],
           'seed_d_bin': rec['d_bin'], 'seed_theta_deg': rec['theta_peak'],
           'seed_votes': rec['votes'],
           'voted_in_seed_window': bool(rec['in_window'][e]),
           'alive': bool(rec['alive'][e]),
           'dist_first_pass_m': None, 'candidate_first_pass': None,
           'dist_refit_m': None, 'within_eps_of_refit': None,
           'facing_dot_refit': None, 'faces_refit': None,
           'candidate': None, 'run': None}
    if rec['d_peak'] is not None:
        out['dist_first_pass_m'] = float(
            tracer.distance(rec['theta_peak'], rec['d_peak'])[e])
        out['candidate_first_pass'] = bool(rec['cand_first'][e])
    theta, d = refit_line_of(rec)
    if theta is not None:
        dist = float(tracer.distance(theta, d)[e])
        dot = float(tracer.facing(theta)[e])
        out.update({'dist_refit_m': dist,
                    'within_eps_of_refit': bool(dist <= tracer.eps),
                    'facing_dot_refit': dot,
                    'faces_refit': bool(dot > gw.FACING_MIN),
                    'candidate': bool(rec['cand'][e])})
    if rec['runs'] is not None and rec['cand'][e]:
        pos = int(np.nonzero(rec['order'] == e)[0][0])
        for r, (lo, hi) in enumerate(rec['runs']):
            if lo <= pos < hi:
                out['run'] = {'index': r, 'n_elements': hi - lo,
                              'length_m': rec['run_length'][r],
                              'accepted': rec['run_accepted'][r],
                              'segment': rec['run_seg'][r],
                              'position_in_candidates': pos}
    return out


def classify_drop(home, at_home: dict | None) -> str:
    """The class of one e', exactly as registered at spec v0.17.

    No home is decided first. Otherwise the FIRST that applies at H:

        R3     e' is farther than eps from H's refitted line
        R4     e' does not face H's refitted line
        R5+R6  e' is a candidate of H, in a run rejected as shorter than l_min
        Other  none of the above

    "Farther than eps" is the negation of `line_candidates`' own inclusive
    `<= eps`, and "does not face" the negation of its `> FACING_MIN`, both
    already evaluated in `at_home` against the iteration's refit.
    """
    if home is None:
        return NO_HOME
    if not at_home['within_eps_of_refit']:
        return R3_LINE
    if not at_home['faces_refit']:
        return R4_FACING
    run = at_home['run']
    if at_home['candidate'] and run is not None and not run['accepted']:
        return R56_SHORT_RUN
    return OTHER


def later_fate(tracer, e: int, since: int) -> dict:
    """Reported: what became of element `e` after iteration `since`.

    Every (theta, d) window that holds e is one seed bin, and each ends one of
    three ways: never seedable (its live sum never reached `vote_min`),
    dropped (it fell below `vote_min` unyielded -- with the first, R1), or
    exhausted by the iteration it seeded (R2). For each iteration after
    `since` that seeded a window holding e, what that iteration did with e is
    named: aborted before any run (R7), e beyond eps of its refit (R3), e not
    facing it (R4), or e in a rejected run (R5+R6). `since = -1` covers the
    whole pass.

    The tally covers every such iteration; only the ACCEPTING ones are listed
    one by one, since an element can sit in thousands of seeded windows.
    """
    half, n_d = tracer.half, tracer.n_d
    w = {'n_windows': 0, 'never_seedable': 0,
         'exhausted_up_to_since': 0, 'exhausted_after': 0,
         'dropped_up_to_since': 0, 'dropped_after': 0}
    for j in np.nonzero(tracer.vote[e])[0].tolist():
        b0 = int(tracer.bins[e, j])
        for b in range(max(0, b0 - half), min(n_d, b0 + half + 1)):
            w['n_windows'] += 1
            spent, fell = int(tracer.spent_at[j, b]), int(tracer.fell_at[j, b])
            if spent >= 0:
                w['exhausted_up_to_since' if spent <= since
                  else 'exhausted_after'] += 1
            elif fell >= 0:
                w['dropped_up_to_since' if fell <= since
                  else 'dropped_after'] += 1
            else:
                w['never_seedable'] += 1

    later, tally = [], {o: 0 for o in LATER_OUTCOMES}
    for rec in tracer.iterations[since + 1:]:
        if not rec['in_window'][e]:
            continue
        at = element_at(tracer, rec['i'], e)
        if rec['runs'] is None:
            outcome = LATER_ABORTED
        elif at['candidate']:
            outcome = (LATER_ASSIGNED if at['run']['accepted']
                       else LATER_SHORT_RUN)
        elif not at['within_eps_of_refit']:
            outcome = LATER_BEYOND_EPS
        else:
            outcome = LATER_NOT_FACING
        tally[outcome] += 1
        if rec['exit'] == gw.TRACE_ACCEPTED:
            later.append({'iteration': rec['i'], 'outcome': outcome})
    n_later = sum(tally.values())
    return {'since': int(since), 'windows': w,
            'n_later_seeded_windows': n_later,
            'R1_never_in_a_seeded_window_again': n_later == 0,
            'later_by_outcome': tally, 'later_accepting': later}


def no_home_report(tracer, cps: list[dict]) -> dict:
    """Reported, for an e' with no home: R1 or R2 behind S_b's counterparts.

    Per counterpart, over the whole pass: R1 when no window holding it was
    ever seeded, R2 when one was and the iteration it seeded did not take it.
    """
    rows = []
    for c in cps:
        if c['counterpart'] is None:
            continue
        f = later_fate(tracer, c['counterpart'], -1)
        rows.append({'member': c['member'], 'counterpart': c['counterpart'],
                     'rule': 'R1' if f['R1_never_in_a_seeded_window_again']
                     else 'R2',
                     'windows': f['windows'],
                     'n_seeded_windows': f['n_later_seeded_windows'],
                     'by_outcome': f['later_by_outcome']})
    return {'n_members': len(cps), 'n_counterparts': len(rows),
            'n_without_counterpart': len(cps) - len(rows),
            'n_R1_never_in_a_seeded_window': sum(1 for r in rows
                                                 if r['rule'] == 'R1'),
            'n_R2_seed_bins_spent': sum(1 for r in rows if r['rule'] == 'R2'),
            'seeded_windows_by_outcome': {
                o: sum(r['by_outcome'][o] for r in rows)
                for o in LATER_OUTCOMES},
            'counterparts': rows}


def line_quantities(tracer, it: int, i_ref: int, span_x: np.ndarray,
                    span_y: np.ndarray) -> dict:
    """Reported: an iteration's refitted line against one wall.

    The angle between the refitted line and the wall's axis, the axis being
    the tangent of element `i_ref`'s own direction (normals are axis-aligned
    by construction), folded to [0, 90] degrees. And the along-extent of the
    iteration's candidates against the span of the wall's elements, all
    projected on the refitted line's own direction: a candidate more than half
    a cell past either end of the span is beyond the wall. The FIRST-PASS
    candidates are the ones the line was fitted to; the refit's own are given
    beside them.
    """
    rec = tracer.iterations[it]
    theta, d = refit_line_of(rec)
    el, rho = tracer.el, tracer.rho
    ux, uy = gw.dir_of(theta)
    t_all = el['x'] * ux + el['y'] * uy
    axis = math.degrees(math.atan2(float(el['nx'][i_ref]),
                                   -float(el['ny'][i_ref])))
    a = (theta - axis) % 180.0
    span_t = span_x * ux + span_y * uy
    s_min, s_max = float(span_t.min()), float(span_t.max())

    def extent(mask):
        t = t_all[mask]
        beyond = (t < s_min - rho / 2.0) | (t > s_max + rho / 2.0)
        return {'n': int(t.size),
                't_min': float(t.min()) if t.size else None,
                't_max': float(t.max()) if t.size else None,
                'n_beyond_span': int(np.count_nonzero(beyond)),
                'overhang_below_m': float(max(0.0, s_min - t.min()))
                if t.size else None,
                'overhang_above_m': float(max(0.0, t.max() - s_max))
                if t.size else None}

    return {'iteration': int(it), 'theta_first_pass_deg': rec['theta_peak'],
            'd_first_pass_m': rec['d_peak'], 'theta_refit_deg': theta,
            'd_refit_m': d, 'degenerate_fit': rec['degenerate'],
            'angle_to_face_axis_deg': float(min(a, 180.0 - a)),
            'span': {'n': int(span_t.size), 't_min': s_min, 't_max': s_max},
            'first_pass_candidates': extent(rec['cand_first']),
            'refit_candidates': extent(rec['cand'])}


def run_gaps(tracer, it: int, e: int) -> list[dict]:
    """Reported, for R5+R6: what lies in the gaps that cut e''s run.

    One record per end of the run. `neighbour` is None where the run is the
    first or last of the candidates and no gap cut it there. Otherwise the gap
    runs between the run's end element and the next candidate along the line,
    and the elements strictly between them along the line are counted three
    ways: first-pass candidates the refit left more than eps away; elements of
    e''s own face line that are alive and more than eps from the refit; and
    elements of that face line already assigned. `thinned_by_r3` is either of
    the first two being non-zero.
    """
    rec = tracer.iterations[it]
    at = element_at(tracer, it, e)
    if at['run'] is None:
        return []
    lo, hi = rec['runs'][at['run']['index']]
    order, t_sorted = rec['order'], rec['t_sorted']
    theta, d = refit_line_of(rec)
    el, rho = tracer.el, tracer.rho
    ux, uy = gw.dir_of(theta)
    t_all = el['x'] * ux + el['y'] * uy
    beyond = ~(tracer.distance(theta, d) <= tracer.eps)
    face = np.zeros(el['n'], dtype=bool)
    face[face_line_of(el, e)['members']] = True
    out = []
    for name, edge, nb in (('below', lo, lo - 1), ('above', hi - 1, hi)):
        if nb < 0 or nb >= order.size:
            out.append({'end': name, 'neighbour': None})
            continue
        ta, tb = sorted((float(t_sorted[edge]), float(t_sorted[nb])))
        inside = (t_all > ta) & (t_all < tb)
        n_lost = int(np.count_nonzero(
            inside & rec['cand_first'] & ~rec['cand'] & beyond))
        n_face = int(np.count_nonzero(inside & face & rec['alive'] & beyond))
        out.append({
            'end': name, 'neighbour': int(order[nb]),
            'gap_m': tb - ta - rho, 'g_m': tracer.g,
            'n_first_pass_candidates_beyond_eps_of_refit': n_lost,
            'n_on_face_line_alive_beyond_eps_of_refit': n_face,
            'n_on_face_line_already_assigned': int(np.count_nonzero(
                inside & face & ~rec['alive'])),
            'thinned_by_r3': bool(n_lost or n_face)})
    return out


def nearest_assigned_same_dir(el: dict, seg_of: np.ndarray, i: int,
                              tol: float = FACE_LINE_TOL_M) -> dict | None:
    """Reported: e''s nearest ASSIGNED same-direction element at k'.

    Nearest by straight-line distance between midpoints, ties to the lowest
    index, and whether it lies on e''s own face line -- the same across
    coordinate within `tol`, as `face_line_of` reads it. None when no element
    of that direction is assigned.
    """
    sel = np.nonzero((el['dir'] == el['dir'][i]) & (seg_of >= 0))[0]
    if sel.size == 0:
        return None
    dx, dy = el['x'][sel] - el['x'][i], el['y'][sel] - el['y'][i]
    dist = np.hypot(dx, dy)
    n = int(np.lexsort((sel, dist))[0])
    across = abs(float(dx[n] * el['nx'][i] + dy[n] * el['ny'][i]))
    return {'index': int(sel[n]), 'segment': int(seg_of[sel[n]]),
            'dist_m': float(dist[n]), 'across_m': across,
            'on_e_prime_face_line': bool(across <= tol)}


def p15_verdict(events: list[dict], n_expected: int = N_EVENTS,
                min_r3: int = P15_MIN_R3) -> dict:
    """P15, exactly as registered: at least 11 of the 22 distinct e' in R3."""
    assert len(events) == n_expected, len(events)
    n = sum(1 for e in events if e['class'] == R3_LINE)
    return {'of': len(events), 'n_R3': n, 'min_required': min_r3,
            'holds': n >= min_r3,
            'by_class': {c: sum(1 for e in events if e['class'] == c)
                         for c in DROP_CLASSES}}


def p16_verdict(events: list[dict], n_expected: int = N_EVENTS) -> dict:
    """P16, exactly as registered: none of the 22 in Other; holds only at 0."""
    assert len(events) == n_expected, len(events)
    other = [e for e in events if e['class'] == OTHER]
    return {'of': len(events), 'n_other': len(other), 'holds': not other,
            'n_R4': sum(1 for e in events if e['class'] == R4_FACING),
            'other': [e['key'] for e in other]}


def _no_class_in(payload: dict) -> None:
    """Assert a G9 fail payload carries no class and nothing built on one.

    Exact key names, as `_no_change_in` does it. S10's own quantities are not
    banned: they are published (spec v0.17) and check 1 is an equality
    against them.
    """
    banned = {'class', 'classes', 'H', 'home', 'at_home', 'line_at_home',
              'line_at_k', 'run_gaps', 'after_home', 'no_home',
              'nearest_assigned_same_dir', 'trace', 'trace_tally', 'events',
              'P15', 'P16',
              'by_class', 'n_R3', 'n_R4', 'n_other', 'reported',
              'classes_by_shape', 'classes_by_state', 'segment_units',
              'verdict'}

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                assert str(k) not in banned, \
                    f'{path}.{k} is an unseen quantity'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')


def _write_s11(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    """S9's writer under this stage's name: a dated file, never an overwrite."""
    return _write_s9(payload, out_dir, prefix)


def _print_s11(events: list[dict], reported: dict) -> None:
    """Classes by shape and by S_b's state, then one line per e'."""
    for title, key, cols in (
            ('classes by loss shape', 'classes_by_shape', S10_SHAPES),
            ('classes by state of S_b at k\'', 'classes_by_state',
             (INTACT, CHANGED))):
        print(f'\n{title}')
        print(f'  {"class":<20}' + ''.join(f'{c:>18}' for c in cols)
              + f'{"total":>8}')
        for cls in DROP_CLASSES:
            row = [reported[key][c][cls] for c in cols]
            print(f'  {cls:<20}' + ''.join(f'{n:>18}' for n in row)
                  + f'{sum(row):>8}')

    u = reported['per_distinct_s_b']
    print('\nper distinct S_b, reported not judged: '
          + '  '.join(f'{c} {n}' for c, n in u['n_units_by_class'].items())
          + f'  ({u["n_units"]} S_b, {u["n_units_with_two_classes"]} with '
          'two classes)')

    print('\ndistinct e\' (B, k, k\', e\')')
    for e in events:
        at, ln = e['at_home'], e['line_at_home']
        tail = '  no home' if at is None else (
            f'  H {e["home"]["H"]}  first pass '
            f'{at["dist_first_pass_m"]:.4f} m  refit '
            f'{at["dist_refit_m"]:.4f} m  eps {e["eps_m"]:.4f} m  '
            f'angle {ln["angle_to_face_axis_deg"]:.3f} deg')
        print(f'  B=k{e["B"]} {e["step"]} e\' {e["e_prime"]}  S_b '
              f'{e["seg_b"]}  {e["shape"]}  {e["state"]}  {e["class"]}{tail}')


def s11_trace(by_kc: dict, spawn: dict, s10: dict, meta: dict,
              out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
              cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
              n_events: int = N_EVENTS, min_r3: int = P15_MIN_R3,
              trace_of=trace_stem, s9_sha: str | None = None) -> dict:
    """The order that keeps every class unseen behind G9.

    Check 1 reproduces S10. Check 2 traces B at k' and at k for each step,
    tracer off and on, and compares. Check 3's first half reads e' in the
    traced run. Only then is anything classified, and check 3's second half
    -- every e' in exactly one class -- is read off the result before a class
    is printed or written. On any failure the stage writes the G9 comparisons
    and stops at exit 2; the fail payload is walked for a class first.

    Reads no file and resolves no spawn; `run_s11` is this function plus the
    corpus I/O. `trace_of(side)` is how a map is traced, so the whole
    pipeline runs on hand-built grids.
    """
    t0 = time.monotonic() if t0 is None else t0

    # ── G9 check 1: S10, through the functions its stage calls ─────────────
    c1, derived = g9_s10_reproduces(by_kc, spawn, s10, cuts, n_robots,
                                    n_events, s9_sha)

    # ── G9 check 2: each map of each step, tracer off and on ───────────────
    traced, c2, c3a = {}, None, None
    if c1['ok']:
        for (b, k, kp) in sorted({(e['B'], e['k'], e['kp'])
                                  for e in derived['events']}):
            for c in (kp, k):
                if (b, c) not in traced:
                    traced[(b, c)] = trace_of(by_kc[(b, c)])
        c2 = g9_non_interference(by_kc, traced)

    # ── G9 check 3, first half: e' unassigned in the traced run ────────────
    if c2 is not None and c2['ok']:
        c3a = g9_unassigned(traced, derived['events'])

    g9 = {'1_s10_reproduces': c1, '2_non_interference': c2,
          '3_e_prime_unassigned_in_one_class': {'unassigned': c3a,
                                                'one_class': None},
          'parts': {'1_s10_reproduces': c1['ok'],
                    '2_non_interference': bool(c2 and c2['ok']),
                    '3_e_prime_unassigned_in_one_class':
                        bool(c3a and c3a['ok'])}}

    def gate_line():
        g9['all_ok'] = all(g9['parts'].values())
        print('G9: ' + '  '.join(
            f'check {n.split("_")[0]} {"HOLD" if ok else "FAIL"}'
            for n, ok in g9['parts'].items())
            + f'   {"HOLD" if g9["all_ok"] else "FAIL"}')

    def refuse(why: str):
        for label, bad in (
                ('S10 sections differing', c1['sections_differing']),
                ('S10 derivation', c1['derivation_asserted']),
                ('maps', None if c2 is None else
                 [r['map'] for r in c2['not_ok']]),
                ('e\' assigned in the traced run', None if c3a is None
                 else c3a['not_ok'][:10])):
            if bad:
                print(f'  {label}: {bad}')
        fail = dict(meta)
        fail.update({'stage': 's11_g9_fail',
                     'seconds': time.monotonic() - t0, 'G9': g9})
        _no_class_in(fail)
        path, _ = _write_s11(fail, out_dir, 's11_g9_fail')
        die(f'G9 failed; {why} {path}')

    if not all(g9['parts'].values()):
        gate_line()
        refuse('nothing was classified, and no class was computed, written '
               'or printed.')

    # ── counterparts, H and the class, per distinct e' ─────────────────────
    cps_of: dict[tuple, list] = {}
    events = []
    for e in derived['events']:
        b, k, kp, j = e['key']
        side_k, side_kp = by_kc[(b, k)], by_kc[(b, kp)]
        tr_kp, tr_k = traced[(b, kp)]['tracer'], traced[(b, k)]['tracer']
        unit = (b, k, kp, e['seg_b'])
        if unit not in cps_of:
            cps_of[unit] = counterparts(side_k, side_kp, e['seg_b'])
        cps = cps_of[unit]
        home = home_iteration(tr_kp.assigned_in(), cps)
        h = home['H']
        at_home = None if h is None else element_at(tr_kp, h, j)
        cls = classify_drop(h, at_home)

        members = np.array([c['member'] for c in cps], dtype=np.int64)
        span_x, span_y = side_k['el']['x'][members], side_k['el']['y'][members]
        it_b = int(tr_k.assigned_in()[e['b_index']])
        # the registered record of e', iteration by iteration: in full for H
        # and every accepting iteration -- the only ones that could have
        # taken it -- and tallied for the rest
        rows_j = [element_at(tr_kp, rec['i'], j) for rec in tr_kp.iterations
                  if rec['i'] == h or rec['exit'] == gw.TRACE_ACCEPTED]
        tally_j = {
            'n_iterations': tr_kp.n_iter,
            'n_voted_in_seed_window': sum(
                1 for rec in tr_kp.iterations if rec['in_window'][j]),
            'n_candidate_first_pass': sum(
                1 for rec in tr_kp.iterations
                if rec['cand_first'] is not None and rec['cand_first'][j]),
            'n_candidate_of_refit': sum(
                1 for rec in tr_kp.iterations
                if rec['cand'] is not None and rec['cand'][j]),
            'iterations_by_exit': {
                x: sum(1 for rec in tr_kp.iterations if rec['exit'] == x)
                for x in gw.TRACE_ABORTS + (gw.TRACE_ACCEPTED,)}}
        events.append({
            **{f: e[f] for f in ('key', 'B', 'k', 'kp', 'step', 'e_prime',
                                 'b_index', 'b_dir', 'seg_b', 'shape',
                                 'state', 'n_losses', 'map_at_k',
                                 'map_at_kp')},
            'class': cls, 'home': home, 'at_home': at_home,
            'eps_m': tr_kp.eps, 'l_min_m': tr_kp.l_min,
            'line_at_home': None if h is None else line_quantities(
                tr_kp, h, j, span_x, span_y),
            'line_at_k': {
                **line_quantities(tr_k, it_b, e['b_index'], span_x, span_y),
                'b': element_at(tr_k, it_b, e['b_index']),
                'eps_m': tr_k.eps},
            'run_gaps': run_gaps(tr_kp, h, j) if cls == R56_SHORT_RUN
            else None,
            'after_home': None if h is None else later_fate(tr_kp, j, h),
            'no_home': no_home_report(tr_kp, cps) if h is None else None,
            'nearest_assigned_same_dir': nearest_assigned_same_dir(
                side_kp['el'], side_kp['seg_of'], j),
            'trace_tally': tally_j, 'trace': rows_j})

    # ── G9 check 3, second half, before any class is shown ─────────────────
    c3b = g9_one_class([e['class'] for e in events], n_events)
    g9['3_e_prime_unassigned_in_one_class']['one_class'] = c3b
    g9['parts']['3_e_prime_unassigned_in_one_class'] = bool(
        c3a['ok'] and c3b['ok'])
    gate_line()
    if not g9['all_ok']:
        refuse('an e\' did not land in exactly one class; no class was '
               'written or printed.')

    # ── the two verdicts, judged on the distinct e' ────────────────────────
    p15 = p15_verdict(events, n_events, min_r3)
    p16 = p16_verdict(events, n_events)
    print(f'P15: {"HOLD" if p15["holds"] else "FAIL"}  {p15["n_R3"]}/'
          f'{p15["of"]} distinct e\' in R3, {p15["min_required"]} required')
    print(f'P16: {"HOLD" if p16["holds"] else "FAIL"}  {p16["n_other"]}/'
          f'{p16["of"]} distinct e\' in Other, 0 required  '
          f'(R4, reported: {p16["n_R4"]})')

    # ── reported, not tested ───────────────────────────────────────────────
    units = distinct_groups(events, UNIT_FIELDS)
    per_unit = [{'key': u['key'], 'state': u['rows'][0]['state'],
                 'e_primes': [r['e_prime'] for r in u['rows']],
                 'classes': [r['class'] for r in u['rows']]} for u in units]
    reported = {
        'classes_by_shape': {
            s: {c: sum(1 for e in events
                       if e['shape'] == s and e['class'] == c)
                for c in DROP_CLASSES} for s in S10_SHAPES},
        'classes_by_state': {
            s: {c: sum(1 for e in events
                       if e['state'] == s and e['class'] == c)
                for c in DROP_CLASSES} for s in (INTACT, CHANGED)},
        'per_distinct_s_b': {
            'note': 'counts per distinct (B, k, k\', S_b), reported beside '
                    'the verdicts and never judged; an S_b whose e\' fall '
                    'in two classes counts in both',
            'n_units': len(per_unit),
            'n_units_with_two_classes': sum(
                1 for u in per_unit if len(set(u['classes'])) > 1),
            'n_units_by_class': {
                c: sum(1 for u in per_unit if c in u['classes'])
                for c in DROP_CLASSES},
            'units': per_unit},
        'per_e_prime': 'events[*]: at_home and line_at_home (H\'s line '
                       'against the wall\'s axis, e\'\'s two distances, the '
                       'candidates\' extent against S_b\'s span), line_at_k '
                       '(the same for the iteration that assigned b), '
                       'run_gaps (R5+R6), after_home, no_home and '
                       'nearest_assigned_same_dir'}
    _print_s11(events, reported)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0, 'rho_m': s10['rho_m'],
        'n_events': len(events), 'n_maps_traced': len(traced),
        'G9': g9, 'events': events, 'P15': p15, 'P16': p16,
        'reported': reported})
    path, sha = _write_s11(payload, out_dir, 's11_trace')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s11(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
            ) -> dict:
    """S11: the rule that drops each e'. Spec v0.17.

    G9 against S10, then P15 and P16, judged separately.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before anything is loaded, as S5 to S10 do.
    2. REGISTRATION BEFORE RESULT: `dc754de`, the commit that registered P15
       and P16, must be an ancestor of HEAD, and the spec at HEAD must carry
       both "REGISTERED" headers.
    3. The two paragraphs are read from the spec AT HEAD by script and stored
       verbatim, never retyped.
    4. THE S10 FILE IS PINNED BY HASH. G9 check 1 is an equality against it.

    NOTHING IS LOADED UNTIL ALL FOUR HOLD. Sides come from `load_side`, and
    each traced map is rebuilt by `trace_stem` the same way: `g` from that
    map's own graph JSON, and the map's sha256 checked against it.
    """
    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S11_REGISTRATION):
        die(f'{S11_REGISTRATION} registered P15 and P16 and is not an '
            f'ancestor of HEAD {head[:7] or "(unknown)"}; the registration '
            'must be in history behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    for gate in (P15_GATE_TEXT, P16_GATE_TEXT):
        if gate not in spec_text:
            die(f'the spec at HEAD carries no "{gate}"; S11 evaluates two '
                'registered predictions or it does not run')
    verbatim = {'P15': spec_paragraph(spec_text, P15_HEADER),
                'P16': spec_paragraph(spec_text, P16_HEADER)}
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    s10_path = out_dir / S10_RESULT
    if not s10_path.is_file():
        die(f'{s10_path} is missing; G9 check 1 reproduces its values and '
            'the units are the 22 e\' it holds')
    s10_sha = _sha256(s10_path)
    if s10_sha != S10_SHA256:
        die(f'{s10_path.name} has sha256 {s10_sha}, not the {S10_SHA256} G9 '
            'is stated against; that is a different S10 run')
    s10 = json.loads(s10_path.read_text())

    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S11 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's11', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.17, G9/P15/P16',
            'predictions_verbatim': verbatim,
            'cell_centre': CELL_CENTRE_OFFSET,
            'sources': {'s10': {'file': s10_path.name, 'sha256': s10_sha,
                                'head': s10.get('head')}},
            'note': 'each map is extracted with the tracer off and on, with '
                    'Params(g) from its own graph JSON; the class is read at '
                    'H against the iteration\'s own refit; a counterpart is '
                    'found by position with trinary_map.cell_index and never '
                    'with trinary_map.sample. No viewpoint weight is computed'}
    return s11_trace(by_kc, spawn, s10, meta, out_dir, t0, s9_sha=S9_SHA256)


# ── S12: the viewpoint weight at mixed cuts, G10 and P17
#    (spec v0.19, "Viewpoint weight at mixed cuts (P17)") ─────────────────────

S12_REGISTRATION = '82e6462'       # registered P17 and G10 (spec v0.19)
P17_HEADER = '**P17. REGISTERED'
G10_HEADER = '**G10 (gate).**'
G10_THIRD_HEADER = 'On any difference it stops before writing a verdict.'
P17_GATE_TEXT = 'P17. REGISTERED'
G10_GATE_TEXT = 'G10 (gate).'
S5_SHA256 = ('f99e001d517734ea4b8d737436f0e9243122c0edd9d30f65a241dde39e89bb'
             '67')
P17_BOUND = 1.0 / 180.0            # "d <= 1/180 (1 deg)", inclusive
P17_MIN_PAIRS = 54                 # "at least 54 of the 60 pairs of M60"
M60_CUTS = (60, 120, 240)          # "a and b both in {60, 120, 240}"
G10_MEAN_TOL = 1e-12               # check 3: "equals w_mean within 1e-12"
M60, M1200, SAME_CUT = 'M60', 'M1200', 'same_cut'
N_M60, N_M1200, N_SAME_CUT = 60, 60, 40       # of the 160 mirror pairs
# G10 check 2's seven quantities, in the registered order.
WEIGHT_FIELDS = ('w_mean', 'W', 'w_p10', 'w_p50', 'w_p90', 'unweighable_A',
                 'unweighable_B')
# What S6 and S5 store beside the registered quantities on the same rows.
# Compared and reported; the gate is the registered comparison and no wider.
S6_BESIDE_FIELDS = ('not_corroborated', 'n_corroborated', 'n_elements',
                    'L_A_m')
S5_BESIDE_FIELDS = ('corroborated', 'n_corroborated', 'n_weighable')
# What S5 stores and S12 does NOT recompute, named so the omission is a
# statement rather than an oversight: all four are E7's or w_null's.
S5_NOT_COMPARED = ('P7.rows[].mean_w', 'P7.rows[].mean_w_null',
                   'P7.rows[].n_E7', 'P7.rows[].reported.n_E7')


class S12GateClosed(RuntimeError):
    """A weight asked for on a row S5 does not hold, before G10 allowed it."""


class S12Weights:
    """The one door through which S12 reaches S5's weight call.

    `row(key)` IS S5'S CALL: `_side_classify`, the function `run_s5` computes
    every one of its rows with, handed what `run_s5` hands it -- A's side, B's
    side and the two spawns -- so B's elements are B's FACE elements moved into
    A's frame with their segment ids, and both sides' bearing histograms go to
    `classify`. Nothing is re-derived here. `itself(k, cut)` is the same call
    as G3 makes it: one side twice, both spawns at the origin.

    THE DOOR IS SHUT ON EVERY ROW S5 DOES NOT HOLD until `open_after` is given
    G10's checks and all of them hold. `is_seen` is that set: spec v0.11 says
    every weight S5 computed lies in the 140 seen rows or on a map against
    itself, and check 2 requires S5's file to name exactly those 140. So the
    order of `s12_mirror` is not the only thing keeping the 180 unseen rows
    unseen: a call that arrives early raises, whoever makes it.

    `_side_classify` returns `classify`'s result and `shares` of it. Neither
    calls `null_weights` or `segment_dirs`, so no w_null and no E7 comes back
    through here.
    """

    def __init__(self, by_kc: dict, spawn: dict):
        self._by_kc, self._spawn = by_kc, spawn
        self._open = False
        self.asked: list[tuple] = []   # every (A, B, cut_A, cut_B), in order

    @property
    def is_open(self) -> bool:
        return self._open

    def open_after(self, check_1, check_2, check_3_on_s5_rows) -> bool:
        """Open for the rows S5 does not hold, if all three checks hold."""
        self._open = all(c is not None and c.get('ok') is True
                         for c in (check_1, check_2, check_3_on_s5_rows))
        return self._open

    def row(self, key: tuple) -> tuple:
        ka, kb, cut_a, cut_b = key
        if ka == kb:
            raise ValueError(f'{key}: a map against itself is `itself`, not '
                             'a row of the matrix')
        if not is_seen(cut_a, cut_b) and not self._open:
            raise S12GateClosed(
                f'A=k{ka} B=k{kb} cut_A {cut_a} cut_B {cut_b} is not a row S5 '
                'holds, and G10 has not passed; no weight is computed on it')
        self.asked.append((ka, kb, cut_a, cut_b))
        return _side_classify(self._by_kc[(ka, cut_a)],
                              self._by_kc[(kb, cut_b)],
                              self._spawn[ka], self._spawn[kb])

    def itself(self, k: int, cut: int) -> tuple:
        side = self._by_kc[(k, cut)]
        self.asked.append((k, k, cut, cut))
        return _side_classify(side, side, (0.0, 0.0), (0.0, 0.0))


def s6_c_comparisons(s6: dict) -> tuple[list[dict], list[dict]]:
    """Every row of S6's file: C, and what S6 stored beside it.

    Returns `(registered, beside)`. G10 check 1 is stated on C, which S6 stores
    as `corroborated`; that list is the gate. The other four fields of the row
    are compared too and reported, never gating.
    """
    registered, beside = [], []
    for i, row in enumerate(s6['rows']):
        key = (row['A'], row['B'], row['cut_A'], row['cut_B'])
        registered.append({'source': 's6', 'path': f'rows[{i}].corroborated',
                           'key': key, 'field': 'corroborated',
                           'stored': row['corroborated']})
        for field in S6_BESIDE_FIELDS:
            beside.append({'source': 's6', 'path': f'rows[{i}].{field}',
                           'key': key, 'field': field, 'stored': row[field]})
    return registered, beside


def s5_weight_comparisons(s5: dict) -> tuple[list[dict], list[dict]]:
    """Every weight quantity S5's file stores, with the matrix row it is on.

    Returns `(registered, beside)`. Each entry carries the `location` it was
    read from, so the gate can count its comparisons per location.

    THE CUTS OF P7'S AND P8'S ROWS COME FROM THEIR REGISTERED WORDING, as
    `s5_comparisons` reads them for G4: P7 is A and B both at cut1200;
    `P8.rows[i].reported_B60` and `w_mean_B60` are the (A@1200, B@60) row and
    `w_mean_B1200` the (A@1200, B@1200) one. The two `reported` series carry
    `cut_A` and `cut_B` and those are used as stored.

    NOT HERE, AND NOT COMPUTED BY S12: `S5_NOT_COMPARED`. P7's `mean_w` is a
    mean over E7 and `mean_w_null` is w_null; S12 computes neither.
    """
    registered, beside = [], []

    def take(location: str, path: str, key: tuple, stored: dict) -> None:
        for field in WEIGHT_FIELDS:
            registered.append({'source': 's5', 'location': location,
                               'path': f'{path}.{field}', 'key': key,
                               'field': field, 'stored': stored[field]})
        for field in S5_BESIDE_FIELDS:
            beside.append({'source': 's5', 'location': location,
                           'path': f'{path}.{field}', 'key': key,
                           'field': field, 'stored': stored[field]})

    for i, row in enumerate(s5['P7']['rows']):
        take('P7.rows[].reported', f'P7.rows[{i}].reported',
             (row['A'], row['B'], FULL_CUT, FULL_CUT), row['reported'])
    for i, row in enumerate(s5['P8']['rows']):
        take('P8.rows[].reported_B60', f'P8.rows[{i}].reported_B60',
             (row['A'], row['B'], FULL_CUT, 60), row['reported_B60'])
        for name, cut_b in (('w_mean_B1200', FULL_CUT), ('w_mean_B60', 60)):
            registered.append({'source': 's5',
                               'location': f'P8.rows[].{name}',
                               'path': f'P8.rows[{i}].{name}',
                               'key': (row['A'], row['B'], FULL_CUT, cut_b),
                               'field': 'w_mean', 'stored': row[name]})
    for name in ('A_cut1200_by_B_cut', 'same_cut'):
        for i, row in enumerate(s5['reported'][name]):
            take(f'reported.{name}[]', f'reported.{name}[{i}]',
                 (row['A'], row['B'], row['cut_A'], row['cut_B']), row)
    return registered, beside


def _beside(comparisons: list[dict], rows: dict, fields: tuple) -> dict:
    """The comparisons that sit beside a gate: counted, and never gating."""
    cmp = g4_compare([c for c in comparisons if c['key'] in rows], rows)
    return {'fields': list(fields), 'n': cmp['n'], 'n_equal': cmp['n_equal'],
            'differing': cmp['differing'],
            'note': 'stored on the same rows; compared and reported, not '
                    'part of the registered gate'}


def g10_c_equals_s6(by_kc: dict, spawn: dict, s6: dict, keys: list
                    ) -> tuple[dict, dict]:
    """G10 check 1: C on all 320 rows, each equal to S6's value exactly.

    C COMES FROM S6'S CALL. `cut_matrix` is what `s6_matrix` fills its rows
    with, so the number compared is produced by the statements that produced
    the stored one, and the comparison is `g4_compare`'s `==`, with no
    tolerance. S6's file must name every row of the matrix exactly once, or
    the check fails without comparing.

    Returns `(check, rows)`, the rows keyed by key.
    """
    registered, beside = s6_c_comparisons(s6)
    named = [c['key'] for c in registered]
    covers = len(named) == len(keys) and set(named) == set(keys)

    rows = cut_matrix(by_kc, spawn, keys)
    cmp = g4_compare([c for c in registered if c['key'] in rows], rows)
    check = {'ok': bool(covers and cmp['all_equal']),
             'covers_the_matrix': covers, 'n_rows_named': len(named),
             'n': cmp['n'], 'n_equal': cmp['n_equal'],
             'differing': cmp['differing'], 'comparisons': cmp['comparisons'],
             'beside': _beside(beside, rows, S6_BESIDE_FIELDS)}
    return check, rows


def g10_g3(door: S12Weights, s5: dict, by_kc: dict) -> dict:
    """G3 again: each map against itself, and against what S5 stored for it.

    The call is S5's -- one side twice, spawns at the origin -- and the
    largest |w| is taken as `run_s5` takes it. Two things are required of each
    map: C and max |w| equal S5's stored values exactly, AND they are 1.0 and
    0.0 exactly, which is G3 itself. `n_weighable` is compared beside them.
    """
    kc_of = {side['stem']: kc for kc, side in by_kc.items()}
    stored = s5['G3']['rows']
    covers = (len(stored) == len(kc_of)
              and {r['map'] for r in stored} == set(kc_of))
    rows = []
    for i, st in enumerate(stored):
        rec = {'map': st['map'], 'path': f'G3.rows[{i}]',
               'stored_C': st['C'], 'stored_max_abs_w': st['max_abs_w'],
               'stored_n_weighable': st['n_weighable']}
        kc = kc_of.get(st['map'])
        if kc is None:
            rec.update({'ok': False, 'reason': 'no such map in the corpus'})
            rows.append(rec)
            continue
        res, sh = door.itself(*kc)
        w = res['weights']['w'] if res['weights'] else np.array([])
        good = ~np.isnan(w)
        worst = float(np.abs(w[good]).max()) if good.any() else 0.0
        c = sh['corroborated']
        rec.update({
            'C': c, 'max_abs_w': worst, 'n_weighable': int(good.sum()),
            'equal_to_S5': bool(c == st['C'] and worst == st['max_abs_w']),
            'g3': bool(c == 1.0 and worst == 0.0),
            'n_weighable_equal': bool(int(good.sum()) == st['n_weighable'])})
        rec['ok'] = rec['equal_to_S5'] and rec['g3']
        rows.append(rec)
    not_ok = [r for r in rows if not r['ok']]
    return {'ok': bool(covers and not not_ok), 'covers_the_corpus': covers,
            'n_maps': len(rows), 'n_ok': len(rows) - len(not_ok),
            'n_comparisons': 2 * len(rows), 'rows': rows, 'not_ok': not_ok}


def g10_weights_equal_s5(door: S12Weights, s5: dict, by_kc: dict,
                         seen: list) -> tuple[dict, dict]:
    """G10 check 2: S5's weight quantities and G3, each equal exactly.

    The weights are computed ON THE 140 ROWS S5 HOLDS AND ON NO OTHER, through
    the door, and every location in S5's file that stores one of the seven
    quantities is compared with `==`. S5's file must name exactly those 140
    rows, or the check fails.

    Returns `(check, computed)`, `computed[key]` being `(res, sh)` as S5's
    call returned it -- kept so the stage records these rows from the very
    call the gate compared, rather than from a second one.
    """
    registered, beside = s5_weight_comparisons(s5)
    covers = {c['key'] for c in registered} == set(seen)

    computed = {k: door.row(k) for k in seen}
    flat = {k: sh for k, (_res, sh) in computed.items()}
    cmp = g4_compare([c for c in registered if c['key'] in flat], flat)
    by_location: dict[str, dict] = {}
    for rec in cmp['comparisons']:
        loc = by_location.setdefault(rec['location'], {'n': 0, 'n_equal': 0})
        loc['n'] += 1
        loc['n_equal'] += int(rec['equal'])

    g3 = g10_g3(door, s5, by_kc)
    check = {'ok': bool(covers and cmp['all_equal'] and g3['ok']),
             'covers_the_rows_s5_holds': covers,
             'weights': {'fields': list(WEIGHT_FIELDS), 'n': cmp['n'],
                         'n_equal': cmp['n_equal'],
                         'all_equal': cmp['all_equal'],
                         'by_location': by_location,
                         'differing': cmp['differing'],
                         'comparisons': cmp['comparisons']},
             'G3': g3,
             'beside': _beside(beside, flat, S5_BESIDE_FIELDS),
             'not_compared': {'locations': list(S5_NOT_COMPARED),
                              'why': 'E7 and w_null; S12 computes neither'}}
    return check, computed


def s12_pairs(res: dict) -> list[dict]:
    """The distinct (S_A, S_B) pairs behind one row's `w_mean`, from A's side.

    Read off what S5's call already returns per element: `res['seg_of']` is
    A's segment and `res['weights']['seg_b_of']` the segment of the B element
    T1 matched. The elements taken are `viewpoint_fields`' own -- assigned,
    with a weight -- so the pairs partition exactly the elements `w_mean` is
    the mean of.

    w IS ONE NUMBER PER PAIR (spec v0.10, Caution 2), and that is required
    exactly, not assumed: a pair whose elements carry two different weights
    stops the stage.
    """
    wts = res['weights']
    w = wts['w']
    good = res['assigned'] & ~np.isnan(w)
    groups: dict[tuple[int, int], list[float]] = {}
    for i in np.nonzero(good)[0]:
        groups.setdefault((int(res['seg_of'][i]), int(wts['seg_b_of'][i])),
                          []).append(float(w[i]))
    out = []
    for (sa, sb) in sorted(groups):
        vals = groups[(sa, sb)]
        if any(v != vals[0] for v in vals):
            die(f'S_A {sa}, S_B {sb}: {len(set(vals))} different weights '
                'inside one matched segment pair, which contradicts Caution '
                '2; nothing is written')
        out.append({'S_A': sa, 'S_B': sb, 'w': vals[0], 'n': len(vals),
                    'length_m': len(vals) * res['rho']})
    return out


def s12_row(key: tuple, c_row: dict, res: dict, sh: dict) -> dict:
    """One row of the record: S6's C, S5's weight quantities, and the pairs.

    `c_row` is the row check 1 computed through S6's call and `(res, sh)` what
    S5's call returned for the same key. The two calls must agree on C --
    `classify` gives the same mask with and without histograms -- and that is
    asserted on every row.
    """
    ka, kb, cut_a, cut_b = key
    assert (sh['corroborated'], sh['n_corroborated'], sh['n_elements']) == (
        c_row['corroborated'], c_row['n_corroborated'], c_row['n_elements']), \
        (key, 'the weight call and S6\'s call disagree on C')
    pairs = s12_pairs(res)
    n_w = sh['n_weighable']
    assert sum(p['n'] for p in pairs) == n_w, (key, n_w)
    row = {'A': ka, 'B': kb, 'cut_A': cut_a, 'cut_B': cut_b,
           'A_stem': c_row['A_stem'], 'B_stem': c_row['B_stem'],
           'seen_by_S5': is_seen(cut_a, cut_b),
           'L_A_m': c_row['L_A_m'], 'n_elements': c_row['n_elements'],
           'corroborated': c_row['corroborated'],
           'n_corroborated': c_row['n_corroborated']}
    row.update({f: sh[f] for f in WEIGHT_FIELDS})
    row.update({'n_weighable': n_w, 'weighable_length_m': n_w * res['rho'],
                'n_pairs': len(pairs), 'pairs': pairs})
    return row


def g10_pairs_mean(row: dict, tol: float = G10_MEAN_TOL) -> dict:
    """G10 check 3 on one row: do the recorded pairs give back `w_mean`?

    The length-weighted mean of w over the row's (S_A, S_B) pairs against the
    row's `w_mean`, within 1e-12. A row with no weighable element has no pair
    and no `w_mean`, and holds exactly when it has neither. The record names
    the row and the difference and NOT the mean: it goes into a fail file too.
    """
    key = [row['A'], row['B'], row['cut_A'], row['cut_B']]
    pairs = row['pairs']
    if not pairs or row['w_mean'] is None:
        return {'key': key, 'abs_diff': None,
                'ok': bool(not pairs and row['w_mean'] is None)}
    length = math.fsum(p['length_m'] for p in pairs)
    mean = math.fsum(p['w'] * p['length_m'] for p in pairs) / length
    diff = abs(mean - row['w_mean'])
    return {'key': key, 'abs_diff': diff, 'ok': bool(diff <= tol)}


def g10_pairs_means(rows: list[dict], tol: float = G10_MEAN_TOL) -> dict:
    """G10 check 3 over `rows`: every one, or the check fails."""
    recs = [g10_pairs_mean(r, tol) for r in rows]
    not_ok = [r for r in recs if not r['ok']]
    diffs = [r['abs_diff'] for r in recs if r['abs_diff'] is not None]
    return {'ok': not not_ok, 'tol': tol, 'n_rows': len(recs),
            'n_ok': len(recs) - len(not_ok),
            'n_rows_without_a_weighable_element': len(recs) - len(diffs),
            'max_abs_diff': max(diffs) if diffs else None,
            'not_ok': not_ok, 'rows': recs}


def mirror_set(cut_x: int, cut_y: int) -> str:
    """Which of the registered sets a mirror pair's two cuts put it in."""
    if cut_x == cut_y:
        return SAME_CUT
    if FULL_CUT in (cut_x, cut_y):
        return M1200
    assert cut_x in M60_CUTS and cut_y in M60_CUTS, (cut_x, cut_y)
    return M60


def mirror_pairs(rows: dict, cuts: tuple = CUTS,
                 n_robots: int = N_ROBOTS) -> list[dict]:
    """The mirror pairs: X@a -> Y@b beside Y@b -> X@a, and d between them.

    For each unordered robot pair {X, Y} with X before Y and each ordered pair
    of cuts (a, b) -- `symmetric_table`'s enumeration, so every row of the
    matrix is one direction of exactly one pair, which is asserted.
    `d = |w_mean(X@a -> Y@b) - w_mean(Y@b -> X@a)|`, and None when either row
    has no weighable corroborated element and so no `w_mean`.
    """
    out, used = [], []
    for x in range(n_robots):
        for y in range(x + 1, n_robots):
            for a in cuts:
                for b in cuts:
                    kf, kr = (x, y, a, b), (y, x, b, a)
                    rf, rr = rows[kf], rows[kr]
                    wf, wr = rf['w_mean'], rr['w_mean']
                    assert (wf is None) == (rf['n_weighable'] == 0), kf
                    assert (wr is None) == (rr['n_weighable'] == 0), kr
                    d = None if wf is None or wr is None else abs(wf - wr)
                    out.append({
                        'X': x, 'Y': y, 'cut_X': a, 'cut_Y': b,
                        'set': mirror_set(a, b),
                        'w_mean_X_to_Y': wf, 'w_mean_Y_to_X': wr, 'd': d,
                        'd_deg': None if d is None else d * 180.0,
                        'n_weighable_X_to_Y': rf['n_weighable'],
                        'n_weighable_Y_to_X': rr['n_weighable'],
                        'weighable_length_X_to_Y_m': rf['weighable_length_m'],
                        'weighable_length_Y_to_X_m': rr['weighable_length_m'],
                        'seen_by_S5_X_to_Y': rf['seen_by_S5'],
                        'seen_by_S5_Y_to_X': rr['seen_by_S5']})
                    used += [kf, kr]
    assert len(used) == len(set(used)) == len(rows), (
        len(used), len(set(used)), len(rows))
    return out


def p17_verdict(mirrors: list[dict], bound: float = P17_BOUND,
                min_pairs: int = P17_MIN_PAIRS,
                n_expected: int = N_M60) -> dict:
    """P17, exactly as registered at spec v0.19, on M60 and on nothing else.

    A pair holds if `d <= 1/180`, the two floats compared as they are and the
    bound inclusive. A pair in which either row has no weighable corroborated
    element has no d and counts against P17. P17 holds if at least 54 of the
    60 pairs hold. Every pair that does not hold is returned in `failing`.
    """
    m60 = [m for m in mirrors if m['set'] == M60]
    assert len(m60) == n_expected, len(m60)
    failing, ds = [], []
    for m in m60:
        undefined = (m['n_weighable_X_to_Y'] == 0
                     or m['n_weighable_Y_to_X'] == 0)
        assert undefined == (m['d'] is None), m
        if not undefined:
            ds.append(m['d'])
        if undefined or m['d'] > bound:
            rec = dict(m)
            rec['reason'] = ('a row has no weighable corroborated element'
                             if undefined else 'd is over the bound')
            failing.append(rec)
    n_hold = len(m60) - len(failing)
    return {'of': len(m60), 'min_required': min_pairs, 'bound': bound,
            'bound_deg': bound * 180.0, 'n_holding': n_hold,
            'holds': n_hold >= min_pairs,
            'n_over_the_bound': sum(1 for f in failing
                                    if f['d'] is not None),
            'n_undefined': sum(1 for f in failing if f['d'] is None),
            'max_d': max(ds) if ds else None,
            'max_d_deg': max(ds) * 180.0 if ds else None,
            'd_summary': d_summary(m60), 'failing': failing}


def d_summary(entries: list[dict]) -> dict:
    """min, median and max of d over a set of mirror pairs, and its zeroes."""
    ds = [e['d'] for e in entries if e['d'] is not None]
    lo, mid, hi = ((min(ds), float(np.median(ds)), max(ds)) if ds
                   else (None, None, None))
    return {'n': len(entries), 'n_undefined': len(entries) - len(ds),
            'n_exactly_0': sum(1 for d in ds if d == 0.0),
            'min': lo, 'median': mid, 'max': hi,
            'min_deg': None if lo is None else lo * 180.0,
            'median_deg': None if mid is None else mid * 180.0,
            'max_deg': None if hi is None else hi * 180.0}


def joined_pairs(row_xy: dict, row_yx: dict) -> list[dict]:
    """A mirror pair's two pair records, joined on (S_X, S_Y).

    X@a -> Y@b records its pairs as (S_A, S_B) = (S_X, S_Y) and the mirror
    row as (S_Y, S_X). For each (S_X, S_Y) either row holds: w from each
    side, None where that side does not hold it, and the length it carries
    from each side, 0 where absent.
    """
    f = {(p['S_A'], p['S_B']): p for p in row_xy['pairs']}
    r = {(p['S_B'], p['S_A']): p for p in row_yx['pairs']}
    out = []
    for sx, sy in sorted(set(f) | set(r)):
        pf, pr = f.get((sx, sy)), r.get((sx, sy))
        out.append({'S_X': sx, 'S_Y': sy,
                    'w_X_to_Y': pf['w'] if pf else None,
                    'w_Y_to_X': pr['w'] if pr else None,
                    'n_X_to_Y': pf['n'] if pf else 0,
                    'n_Y_to_X': pr['n'] if pr else 0,
                    'length_X_to_Y_m': pf['length_m'] if pf else 0.0,
                    'length_Y_to_X_m': pr['length_m'] if pr else 0.0})
    return out


def _mirror_keys(m: dict) -> tuple[tuple, tuple]:
    return ((m['X'], m['Y'], m['cut_X'], m['cut_Y']),
            (m['Y'], m['X'], m['cut_Y'], m['cut_X']))


def w_symmetry(rows: dict, mirrors: list[dict]) -> dict:
    """Is w the same from both sides of a matched segment pair? Reported only.

    Over every mirror pair, wherever one (S_X, S_Y) appears in both rows: the
    largest `|w(X -> Y) - w(Y -> X)|`. `circular_emd_deg` is symmetric in its
    two histograms up to rounding, so this should be at most a few ulps. It
    is a report, not a gate.
    """
    n, n_zero, worst, where = 0, 0, None, None
    for m in mirrors:
        kf, kr = _mirror_keys(m)
        for j in joined_pairs(rows[kf], rows[kr]):
            if j['w_X_to_Y'] is None or j['w_Y_to_X'] is None:
                continue
            diff = abs(j['w_X_to_Y'] - j['w_Y_to_X'])
            n += 1
            n_zero += int(diff == 0.0)
            if worst is None or diff > worst:
                worst = diff
                where = {'X': m['X'], 'Y': m['Y'], 'cut_X': m['cut_X'],
                         'cut_Y': m['cut_Y'], 'S_X': j['S_X'],
                         'S_Y': j['S_Y']}
    return {'n_segment_pairs_in_both_rows': n, 'n_exactly_0': n_zero,
            'max_abs_diff': worst, 'at': where,
            'note': 'reported, not a gate'}


def _no_verdict_in_s12(payload: dict) -> None:
    """Assert a G10 fail payload carries no verdict and no row of the record.

    Exact key names, as `_no_class_in` does it. `predictions_verbatim` is the
    registered wording, keyed by prediction, and is not walked. What a fail
    file may hold is G10's comparisons: C on all 320 rows, which S6 published;
    the weight quantities on rows S5 holds, compared as `stored` and
    `computed`; G3; and, for check 3, a row's key and its difference.
    """
    banned = {'P17', 'verdict', 'holds', 'mirror_pairs', 'pairs',
              'matched_pairs', 'failing', 'reported', 'd', 'd_deg', 'max_d',
              'd_summary', 'w_symmetry', 'w', 'w_mean_X_to_Y',
              'w_mean_Y_to_X', *WEIGHT_FIELDS}
    # the record's own rows sit at the top of a result file; a gate's `rows`
    # (G3's maps, check 3's keys) sit under G10 and are not these
    assert 'rows' not in payload, '$.rows is the record, not a comparison'

    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                if path == '$' and k == 'predictions_verbatim':
                    continue
                assert str(k) not in banned, \
                    f'{path}.{k} is an unseen quantity'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')
    c2 = payload['G10']['2_weights_and_G3_equal_S5']
    if c2 is not None:
        assert all(is_seen(c['key'][2], c['key'][3])
                   for c in c2['weights']['comparisons']), \
            'a weight comparison names a row S5 does not hold'


def _write_s12(payload: dict, out_dir: Path, prefix: str) -> tuple[Path, str]:
    """S9's writer under this stage's name: a dated file, never an overwrite."""
    return _write_s9(payload, out_dir, prefix)


def _deg(v) -> str:
    return '-' if v is None else f'{v:.6f} = {v * 180.0:.4f} deg'


def _print_s12(p17: dict, reported: dict) -> None:
    """The failing M60 pairs with their matched pairs, then what is reported."""
    for f in p17['failing']:
        print(f'  k{f["X"]}@cut{f["cut_X"]} <-> k{f["Y"]}@cut{f["cut_Y"]}  '
              f'd {_deg(f["d"])}  ({f["reason"]})  w_mean '
              f'{f["w_mean_X_to_Y"]!r} / {f["w_mean_Y_to_X"]!r}')
        for j in f['matched_pairs']:
            print(f'      S_X {j["S_X"]:>2}  S_Y {j["S_Y"]:>2}   w '
                  f'{j["w_X_to_Y"]!r} / {j["w_Y_to_X"]!r}   length '
                  f'{j["length_X_to_Y_m"]:.1f} m / '
                  f'{j["length_Y_to_X_m"]:.1f} m')
    print('\nd by set   (min / median / max, exactly 0, undefined)')
    for name, s in (('M60, judged above', p17['d_summary']),
                    ('M1200, reported', reported[M1200]),
                    ('same cut, reported', reported[SAME_CUT])):
        print(f'  {name:<20}{_deg(s["min"])} / {_deg(s["median"])} / '
              f'{_deg(s["max"])}   exactly 0: {s["n_exactly_0"]}/{s["n"]}   '
              f'undefined: {s["n_undefined"]}')
    sym = reported['w_symmetry']
    print(f'w from both sides of one (S_X, S_Y), reported: largest '
          f'difference {sym["max_abs_diff"]!r} over '
          f'{sym["n_segment_pairs_in_both_rows"]} segment pairs, '
          f'{sym["n_exactly_0"]} exactly 0')


def s12_mirror(by_kc: dict, spawn: dict, s5: dict, s6: dict, meta: dict,
               out_dir: Path = OUT_DIR_DEFAULT, t0: float | None = None,
               cuts: tuple = CUTS, n_robots: int = N_ROBOTS,
               bound: float = P17_BOUND, min_pairs: int = P17_MIN_PAIRS,
               tol: float = G10_MEAN_TOL) -> dict:
    """The order that keeps every unseen weight behind G10.

    Check 1 computes C on all 320 rows through S6's call. Check 2 computes the
    weights on the 140 rows S5 holds, and G3 on the 20 maps, through S5's
    call. Check 3 is then read on those 140 rows. ONLY IF ALL THREE HOLD is
    the door opened and a weight computed on any of the other 180 -- and the
    door enforces that, not only this order. Check 3 is then read on every
    row before a verdict is printed or written.

    On any failure the stage writes the G10 comparisons and stops at exit 2;
    the fail payload is walked for a verdict first. A failure of check 1
    computes no weight at all.

    Reads no file and resolves no spawn; `run_s12` is this function plus the
    corpus I/O. NO w_null AND NO E7: nothing here calls `null_weights` or
    `segment_dirs`.
    """
    t0 = time.monotonic() if t0 is None else t0
    keys = matrix_keys(cuts, n_robots)
    seen = [k for k in keys if is_seen(k[2], k[3])]
    unseen = [k for k in keys if not is_seen(k[2], k[3])]
    assert len(seen) == N_SEEN, len(seen)
    assert len(unseen) == N_UNSEEN, len(unseen)
    for kc in sorted(by_kc):
        if by_kc[kc].get('bearing_hist') is None:
            die(f'{by_kc[kc]["stem"]} carries no bearing histograms, and S12 '
                'weighs every row')

    door = S12Weights(by_kc, spawn)
    rows: dict[tuple, dict] = {}

    # ── G10 check 1: C on all 320 rows, through S6's call ───────────────────
    c1, c_rows = g10_c_equals_s6(by_kc, spawn, s6, keys)

    # ── G10 check 2: S5's rows and G3, through S5's call ────────────────────
    c2 = c3 = None
    if c1['ok']:
        c2, computed = g10_weights_equal_s5(door, s5, by_kc, seen)

    # ── G10 check 3 on the rows S5 holds, before any other row exists ──────
    if c2 is not None and c2['ok']:
        for k in seen:
            rows[k] = s12_row(k, c_rows[k], *computed[k])
        c3 = g10_pairs_means([rows[k] for k in seen], tol)

    g10 = {'1_C_equals_S6': c1, '2_weights_and_G3_equal_S5': c2,
           '3_pairs_give_back_w_mean': c3,
           'parts': {'1_C_equals_S6': c1['ok'],
                     '2_weights_and_G3_equal_S5': bool(c2 and c2['ok']),
                     '3_pairs_give_back_w_mean': bool(c3 and c3['ok'])}}

    def gate_line():
        g10['all_ok'] = all(g10['parts'].values())
        print('G10: ' + '  '.join(
            f'check {n.split("_")[0]} {"HOLD" if ok else "FAIL"}'
            for n, ok in g10['parts'].items())
            + f'   {"HOLD" if g10["all_ok"] else "FAIL"}')
        print(f'  check 1: C equal on {c1["n_equal"]}/{c1["n"]} rows')
        if c2 is not None:
            w, g3 = c2['weights'], c2['G3']
            print(f'  check 2: {w["n_equal"]}/{w["n"]} weight values equal '
                  f'over {len(w["by_location"])} locations in S5\'s file; '
                  f'G3 on {g3["n_ok"]}/{g3["n_maps"]} maps')
        c3_now = g10['3_pairs_give_back_w_mean']
        if c3_now is not None:
            print(f'  check 3: {c3_now["n_ok"]}/{c3_now["n_rows"]} rows '
                  f'within {c3_now["tol"]:g}')

    def refuse(why: str):
        for rec in c1['differing'][:20]:
            a, b, ca, cb = rec['key']
            print(f'  {rec["path"]}: A=k{a} B=k{b} cut_A {ca} cut_B {cb}  '
                  f'{rec["field"]}  stored {rec["stored"]!r}  S12 '
                  f'{rec["computed"]!r}')
        if not c1['covers_the_matrix']:
            print(f'  S6\'s file names {c1["n_rows_named"]} rows, not the '
                  f'{len(keys)} of the matrix once each')
        if c2 is not None:
            for rec in c2['weights']['differing'][:20]:
                a, b, ca, cb = rec['key']
                print(f'  {rec["path"]}: A=k{a} B=k{b} cut_A {ca} cut_B {cb}'
                      f'  stored {rec["stored"]!r}  S12 {rec["computed"]!r}')
            if not c2['covers_the_rows_s5_holds']:
                print('  S5\'s file does not name exactly the rows S5 holds')
            for rec in c2['G3']['not_ok']:
                print(f'  G3 {rec["map"]}: {rec.get("reason") or "differs"}')
        c3_now = g10['3_pairs_give_back_w_mean']
        if c3_now is not None:
            for rec in c3_now['not_ok'][:20]:
                a, b, ca, cb = rec['key']
                print(f'  A=k{a} B=k{b} cut_A {ca} cut_B {cb}: the pairs\' '
                      f'mean differs from w_mean by {rec["abs_diff"]!r}')
        fail = dict(meta)
        fail.update({'stage': 's12_g10_fail',
                     'seconds': time.monotonic() - t0, 'G10': g10,
                     'n_weight_calls': len(door.asked)})
        _no_verdict_in_s12(fail)
        path, _ = _write_s12(fail, out_dir, 's12_g10_fail')
        die(f'G10 failed; {why} {path}')

    if not door.open_after(c1, c2, c3):
        gate_line()
        refuse('no weight was computed on any row S5 does not hold, and no '
               'verdict was computed, written or printed.')

    # ── the 180 rows S5 does not hold ───────────────────────────────────────
    for k in unseen:
        rows[k] = s12_row(k, c_rows[k], *door.row(k))
    assert len(rows) == N_SEEN + N_UNSEEN, len(rows)
    assert sorted(door.asked) == sorted(
        keys + [(k, k, c, c) for (k, c) in by_kc]), 'one weight call per row'

    # ── G10 check 3 on every row, before any verdict ────────────────────────
    c3 = g10_pairs_means([rows[k] for k in keys], tol)
    g10['3_pairs_give_back_w_mean'] = c3
    g10['parts']['3_pairs_give_back_w_mean'] = c3['ok']
    gate_line()
    if not g10['all_ok']:
        refuse('the recorded pairs do not give back w_mean on every row; no '
               'verdict was computed, written or printed.')

    # ── the mirror pairs and the verdict, judged on M60 alone ───────────────
    mirrors = mirror_pairs(rows, cuts, n_robots)
    by_set = {s: [m for m in mirrors if m['set'] == s]
              for s in (M60, M1200, SAME_CUT)}
    assert tuple(len(by_set[s]) for s in (M60, M1200, SAME_CUT)) == (
        N_M60, N_M1200, N_SAME_CUT), {s: len(v) for s, v in by_set.items()}

    p17 = p17_verdict(mirrors, bound, min_pairs)
    for f in p17['failing']:
        kf, kr = _mirror_keys(f)
        f['matched_pairs'] = joined_pairs(rows[kf], rows[kr])
    print(f'P17: {"HOLD" if p17["holds"] else "FAIL"}  {p17["n_holding"]}/'
          f'{p17["of"]} M60 pairs with d <= 1/180, {p17["min_required"]} '
          f'required   largest d {_deg(p17["max_d"])}   '
          f'({p17["n_over_the_bound"]} over the bound, '
          f'{p17["n_undefined"]} with a row that has no weighable element)')

    # ── reported, not judged ────────────────────────────────────────────────
    reported = {M1200: d_summary(by_set[M1200]),
                SAME_CUT: d_summary(by_set[SAME_CUT]),
                'w_symmetry': w_symmetry(rows, mirrors),
                'note': 'M1200 and the same-cut pairs are reported, not '
                        'judged; d for every pair is in mirror_pairs'}
    _print_s12(p17, reported)

    rhos = {by_kc[kc]['rho'] for kc in by_kc}
    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0,
        'rho_m': rhos.pop() if len(rhos) == 1 else sorted(rhos),
        'n_rows': len(rows), 'n_seen_by_S5': len(seen),
        'n_unseen_before_S12': len(unseen), 'n_mirror_pairs': len(mirrors),
        'n_weight_calls': len(door.asked),
        'G10': g10, 'rows': [rows[k] for k in keys],
        'mirror_pairs': mirrors, 'P17': p17, 'reported': reported})
    path, sha = _write_s12(payload, out_dir, 's12_mirror')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_s12(out_dir: Path = OUT_DIR_DEFAULT, variant: str = CORPUS_VARIANT
            ) -> dict:
    """S12: the viewpoint weight at mixed cuts. Spec v0.19.

    G10 against S6 and S5, then P17.

    The gates, in this order and each before anything it could mislead:

    1. REFUSES ON AN EDITED TREE, before anything is loaded, as S5 to S11 do.
    2. REGISTRATION BEFORE RESULT: `82e6462`, the commit that registered P17
       and G10, must be an ancestor of HEAD, and the spec at HEAD must carry
       both "P17. REGISTERED" and "G10 (gate).".
    3. The registered paragraphs are read from the spec AT HEAD by script and
       stored verbatim, never retyped.
    4. THE S5 AND S6 FILES ARE PINNED BY HASH. G10's checks 1 and 2 are
       equalities against them.

    NOTHING IS LOADED UNTIL ALL FOUR HOLD.
    """
    t0 = time.monotonic()

    porcelain = _git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = _git('rev-parse', 'HEAD')

    if not _is_ancestor(S12_REGISTRATION):
        die(f'{S12_REGISTRATION} registered P17 and G10 and is not an '
            f'ancestor of HEAD {head[:7] or "(unknown)"}; the registration '
            'must be in history behind the code that tests it')
    spec_text = _git('show', f'HEAD:{SPEC_REL}')
    for gate in (P17_GATE_TEXT, G10_GATE_TEXT):
        if gate not in spec_text:
            die(f'the spec at HEAD carries no "{gate}"; S12 evaluates a '
                'registered prediction behind a registered gate or it does '
                'not run')
    verbatim = {'P17': spec_paragraph(spec_text, P17_HEADER),
                'G10': spec_paragraph(spec_text, G10_HEADER),
                'G10_third_check': spec_paragraph(spec_text,
                                                  G10_THIRD_HEADER)}
    spec_rev = _git('log', '-1', '--format=%h', '--', SPEC_REL)

    sources, loaded = {}, {}
    for name, fname, pinned, what in (
            ('s5', S5_RESULT, S5_SHA256, 'G10 check 2 reproduces its weight '
             'quantities and G3'),
            ('s6', S6_RESULT, S6_SHA256, 'G10 check 1 reproduces its C')):
        path = out_dir / fname
        if not path.is_file():
            die(f'{path} is missing; {what}')
        sha = _sha256(path)
        if sha != pinned:
            die(f'{path.name} has sha256 {sha}, not the {pinned} G10 is '
                f'stated against; that is a different {name.upper()} run')
        loaded[name] = json.loads(path.read_text())
        sources[name] = {'file': path.name, 'sha256': sha,
                         'head': loaded[name].get('head')}

    import fit_world_transform as fwt                       # noqa: PLC0415
    import robot_divergence as rd                           # noqa: PLC0415

    stems = corpus_stems(variant)
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    by_kc = {}
    for s in stems:
        k, c = gw._k_and_cut(s, 'S12 needs the robot and the cut')
        by_kc[(k, c)] = load_side(s)

    meta = {'stage': 's12', 'head': head, 'spec_rev': spec_rev,
            'tree_dirty': bool(dirty),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 't2': False,
            'spec': 'SPEC_b2_divergence_walls v0.19, G10/P17',
            'predictions_verbatim': verbatim,
            'sources': sources,
            'note': 'C comes from S6\'s call (cut_matrix) and the weights '
                    'from S5\'s (_side_classify), reached only through '
                    'S12Weights, which raises on a row S5 does not hold '
                    'until G10 has passed. The (S_A, S_B) pairs are read off '
                    'what that call already returns per element. No w_null '
                    'and no E7 is computed, and no truth is read'}
    return s12_mirror(by_kc, spawn, loaded['s5'], loaded['s6'], meta, out_dir,
                      t0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--stage',
                    choices=('g2', 's5', 's6', 's7', 's8', 's9', 's10',
                             's11', 's12'),
                    default=None,
                    help='run a corpus stage instead of one pair. g2 surveys '
                         'the segment guard on the 20 maps; s5 evaluates G3, '
                         'P7 and P8 on the viewpoint weight; s6 computes the '
                         '320-row cut matrix, gates it on S4 and S5 with G4, '
                         'and tests P9; s7 counts gains and losses per step, '
                         'gates them on S6 with G5, and tests P10; s8 reads '
                         "the cell states behind the losses, gates them on S7 "
                         'with G6, and tests P11; s9 sorts the unchanged '
                         'losses into segmentation or geometry, gates them on '
                         'S8 with G7, and tests P12; s10 reads the segment b '
                         'belonged to at k, gates it on S9 with G8, and tests '
                         'P13a, P13b and P14; s11 traces which rule of the '
                         'extraction drops each e\', gates it on S10 with '
                         'G9, and tests P15 and P16; s12 computes the '
                         'viewpoint weight on all 320 rows, gates it on S6 '
                         'and S5 with G10, and tests P17 on the mixed-cut '
                         'mirror pairs')
    ap.add_argument('--a', help='A\'s map stem')
    ap.add_argument('--b', help='B\'s map stem')
    ap.add_argument('--theta-g', type=float, default=THETA_G_DEG)
    ap.add_argument('--break', dest='brk', choices=('frame-sign',), default=None,
                    help='X1: run with the frame sign flipped')
    ap.add_argument('--t2', action='store_true',
                    help='restore T2 and its outputs (v0.1-v0.3). Off by '
                         'default: T2 left the measure in v0.4 under §7\'s '
                         'fallback, after C0 failed')
    ap.add_argument('--out-dir', type=Path, default=OUT_DIR_DEFAULT)
    args = ap.parse_args()

    if args.stage == 'g2':
        run_g2(args.out_dir)
        return 0
    if args.stage == 's5':
        run_s5(args.out_dir)
        return 0
    if args.stage == 's6':
        run_s6(args.out_dir)
        return 0
    if args.stage == 's7':
        run_s7(args.out_dir)
        return 0
    if args.stage == 's8':
        run_s8(args.out_dir)
        return 0
    if args.stage == 's9':
        run_s9(args.out_dir)
        return 0
    if args.stage == 's10':
        run_s10(args.out_dir)
        return 0
    if args.stage == 's11':
        run_s11(args.out_dir)
        return 0
    if args.stage == 's12':
        run_s12(args.out_dir)
        return 0
    if not (args.a and args.b):
        ap.error('--a and --b are required unless --stage is given')

    row = run_pair(args.a, args.b, flip=args.brk == 'frame-sign',
                   theta_g_deg=args.theta_g, t2=args.t2)
    s = row['shares']
    line = (f'{row["A"]} -> {row["B"]}  L_A {s["L_A"]:.1f} m  '
            f'corroborated {s["corroborated"]:.3f}  ')
    if args.t2:
        line += (f'unobserved {s["unobserved"]:.3f}  '
                 f'contradicted {s["contradicted"]:.3f}')
    else:
        line += f'not corroborated {s["not_corroborated"]:.3f}'
    print(line)
    print(f'written: {write_result(row, args.out_dir)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
