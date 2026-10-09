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
from trinary_map import CELL_CENTRE_OFFSET                    # noqa: E402

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
    this report exists for -- so this scans every same-direction element of B
    instead of the block. Returns None when B has none in that direction.
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
                               within_t1=bool(
                                   near['across_m'] <= eps + T1_TOL_M
                                   and near['along_m'] <= half + T1_TOL_M))
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--stage', choices=('g2', 's5', 's6', 's7'), default=None,
                    help='run a corpus stage instead of one pair. g2 surveys '
                         'the segment guard on the 20 maps; s5 evaluates G3, '
                         'P7 and P8 on the viewpoint weight; s6 computes the '
                         '320-row cut matrix, gates it on S4 and S5 with G4, '
                         'and tests P9; s7 counts gains and losses per step, '
                         'gates them on S6 with G5, and tests P10')
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
