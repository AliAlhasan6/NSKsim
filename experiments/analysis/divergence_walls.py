#!/usr/bin/env python3
"""B2 divergence on walls: sort A's wall faces by B's evidence.

docs/specs/SPEC_b2_divergence_walls.md v0.1, §3-§6. For an ordered pair (A, B)
and a cut C, every face element of A is corroborated, contradicted or unobserved.
The measure uses no ground truth beyond the poses that built the maps.

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
G1_TOL_M = 0.15                    # §7 G1
C0_LIMIT = 0.01                    # §7 C0, 1 % of L_A

# The three classes, in the order the tests run (§4).
CORROBORATED, CONTRADICTED, UNOBSERVED = 0, 1, 2
CLASS_NAME = {CORROBORATED: 'corroborated',
              CONTRADICTED: 'contradicted',
              UNOBSERVED: 'unobserved'}


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


def corroborate(el_a: dict, el_b: dict, eps: float) -> np.ndarray:
    """T1. Which of A's elements have a matching element of B.

    A match has the SAME dir -- element normals are axis-aligned by construction
    (graph_walls.DIRS) and the frame change is a pure translation, so the §4
    angle condition is an equality and is tested as one -- and a midpoint within
    `eps`. B's elements must already be in A's frame.

    Bucketed on an eps-sized lattice so the test is local: a match can only lie
    in the 3x3 block of buckets around A's own. Only elements of B's FACES are
    passed in by the caller (D1); B's unclassified elements, its parked robots
    among them, are not B's evidence of a wall.
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
    eps2 = eps * eps
    for i in range(el_a['n']):
        d = int(el_a['dir'][i])
        xi, yi = el_a['x'][i], el_a['y'][i]
        hit = False
        for ddx in (-1, 0, 1):
            for ddy in (-1, 0, 1):
                for j in table.get((d, int(ax[i]) + ddx, int(ay[i]) + ddy), ()):
                    dx = el_b['x'][j] - xi
                    dy = el_b['y'][j] - yi
                    if dx * dx + dy * dy <= eps2 + 1e-12:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                break
        out[i] = hit
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
             skip_t1: bool = False) -> dict:
    """§4: T1, then T2, then T3, and an element takes the first that applies.

    Only elements assigned to a face of A are classified (§3); the rest are
    counted as excluded length and never appear in a share. `skip_t1` is C0's
    switch, which applies T2 alone.
    """
    eps = gw.EPS_FACTOR * rho if eps is None else eps
    free_flat = free_cell_of(el_a, shape)

    corr = (np.zeros(el_a['n'], dtype=bool) if skip_t1
            else corroborate(el_a, el_b_in_a, eps))
    cross = ray_crossings(rays_in_a, el_a, free_flat, rho, origin, shape,
                          theta_g_deg, eps)
    contra = (~corr) & weight_of_evidence(cross['n12'], cross['n123'])

    cls = np.where(corr, CORROBORATED,
                   np.where(contra, CONTRADICTED, UNOBSERVED))
    return {
        'cls': cls,
        'assigned': seg_of_a >= 0,
        'seg_of': seg_of_a,
        'x': el_a['x'], 'y': el_a['y'],
        'n12': cross['n12'], 'n123': cross['n123'],
        'max_abs_dn': cross['max_abs_dn'],
        'corroborated': corr,
        'contradicted': contra,
        'theta_g_deg': theta_g_deg,
        'eps': eps,
        'rho': rho,
    }


def deep_share(result: dict) -> float:
    """§6: crossings meeting conditions 1-3 over those meeting 1 and 2.

    Reported whatever T2 decides, because it is the quantity O5 is about and it
    does not depend on condition 4's verdict.
    """
    d = int(result['n12'].sum())
    return (int(result['n123'].sum()) / d) if d else 0.0


def shares(result: dict) -> dict:
    """§6: L_A, L_excl and the three shares, which sum to 1.

    Every element is `rho` long, so a share by length is a share by count.
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
    for code, name in CLASS_NAME.items():
        c = int((cls == code).sum())
        out[f'n_{name}'] = c
        out[f'{name}'] = (c / n) if n else 0.0
    return out


def per_face(result: dict, n_faces: int) -> list[dict]:
    """§6: face id and the three lengths, so a contradiction can be located."""
    rho = result['rho']
    cls, seg = result['cls'], result['seg_of']
    rows = []
    for fid in range(n_faces):
        m = seg == fid
        row = {'id': fid, 'length': float(int(m.sum()) * rho)}
        for code, name in CLASS_NAME.items():
            row[f'{name}_m'] = float(int((m & (cls == code)).sum()) * rho)
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
                     eps=eps, theta_g_deg=tg)
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


def load_side(stem: str) -> dict:
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
    return {'stem': stem, 'graph': graph, 'el': el, 'seg_of': seg_of,
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


def run_pair(a_stem: str, b_stem: str, *, flip: bool = False,
             theta_g_deg: float = THETA_G_DEG) -> dict:
    """One ordered pair at one cut, end to end. Reads the corpus."""
    import diagnose_karto as dk                              # noqa: PLC0415
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    a = load_side(a_stem)
    b = load_side(b_stem)
    ka, cut_a = gw._k_and_cut(a_stem, 'the pair needs the robot and the cut')
    kb, cut_b = gw._k_and_cut(b_stem, 'the pair needs the robot and the cut')
    if cut_a != cut_b:
        die(f'{a_stem} is cut{cut_a} and {b_stem} is cut{cut_b}; a pair is one cut')

    # Inside the function, not at import: this shells out to git. rd.SPAWN_REV
    # is 08617b2, the b2maps era -- bag_overlap's own default is the b16 ring
    # and would put every map 3.15 m away.
    spawn = fwt.resolve_spawn_poses(rd.SPAWN_REV)
    spawn_a, spawn_b = spawn[ka], spawn[kb]
    rays = rays_in_frame(dk.admitted_rays(b_stem, kb, cut_b),
                         spawn_b, spawn_a, flip=flip)

    bx, by = to_frame_a(b['el']['x'], b['el']['y'], spawn_b, spawn_a, flip=flip)
    el_b = dict(face_elements_only(b['el'], b['seg_of']))
    el_b['x'], el_b['y'] = bx[b['seg_of'] >= 0], by[b['seg_of'] >= 0]

    t0 = time.monotonic()
    res = classify(a['el'], a['seg_of'], el_b, rays, a['rho'], a['origin'],
                   a['shape'], theta_g_deg=theta_g_deg)
    seconds = time.monotonic() - t0
    return {
        'A': a_stem, 'B': b_stem, 'cut': cut_a,
        'shares': shares(res),
        'per_face': per_face(res, len(a['segments'])),
        'contradicted': contradicted_detail(res, a['segments']),
        'theta_sweep': theta_sweep(a['el'], a['seg_of'], el_b, rays, a['rho'],
                                   a['origin'], a['shape']),
        'seconds': seconds,
    }


def write_result(payload: dict, out_dir: Path = OUT_DIR_DEFAULT) -> Path:
    """§6: a new, dated file every time. Nothing overwrites an earlier result."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = out_dir / f'divergence_{stamp}.json'
    if path.exists():
        die(f'{path} exists; this tool never overwrites a result')
    path.write_text(json.dumps(payload, indent=2, sort_keys=True))
    return path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--a', required=True, help='A\'s map stem')
    ap.add_argument('--b', required=True, help='B\'s map stem')
    ap.add_argument('--theta-g', type=float, default=THETA_G_DEG)
    ap.add_argument('--break', dest='brk', choices=('frame-sign',), default=None,
                    help='X1: run with the frame sign flipped')
    ap.add_argument('--out-dir', type=Path, default=OUT_DIR_DEFAULT)
    args = ap.parse_args()

    row = run_pair(args.a, args.b, flip=args.brk == 'frame-sign',
                   theta_g_deg=args.theta_g)
    s = row['shares']
    print(f'{row["A"]} -> {row["B"]}  L_A {s["L_A"]:.1f} m  '
          f'corroborated {s["corroborated"]:.3f}  '
          f'unobserved {s["unobserved"]:.3f}  '
          f'contradicted {s["contradicted"]:.3f}')
    print(f'written: {write_result(row, args.out_dir)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
