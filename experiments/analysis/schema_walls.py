"""The B2 schema of one map: axis ends, openings, inside corners, slabs, slab ends.

docs/specs/SPEC_b2_schema.md v0.2, §2-§7, and §8's per-map file. One map at a
time, from what graph_walls.graph_and_elements returns: the graph, the element
table, `seg_of` and the grid. Nothing here is judged. Matching to truth (§9) and
the predictions (§10) belong to the stage that judges them, not to this file.

NO TRUTH (D-S5). §2-§7 read no truth, and the per-map file holds none: no world
geometry, no spawn and no pose is imported, read or written here, and every
position in a record is in the map's own frame. A test reads this file's source
and fails on the names of the functions that would bring truth in.

WHAT THIS FILE OWNS, AND WHAT IT BORROWS. It owns the rules of §2-§7. Underneath
them, and not rewritten here:

    graph_walls.DIRS, dir_of       the four axes, and a segment's direction
    graph_walls.endpoint_state     Def 5, read at an axis end (§3)
    graph_walls.gap_record         Def 4, the state of a gap (§4)
    graph_walls.Params             eps, b, g and Def 5's precedence

No existing function or output field changes (D-S1, §8). A segment's `p0`, `p1`
and `length` stay as graph_walls reports them. Axis ends exist only in this
file's records, which go into a directory the caller names and are never
written over a file that exists.

THE RULES ARE PORTED, NOT REDESIGNED, from the scratch scripts the spec's checks
ran (experiments/logs/splice/schema_v0_1_synth.py and schema_v0_1_synth2.py).
Three differences: the thresholds come from each map rather than being fixed at
rho = 0.1; "another junction", which v0.1 removed, is not recorded; and §5's run
test is written as the spec words it, from the nearer axis end toward the other.
That has the same sign as the scripts' test from c whenever the nearer end is
the nearer one.

IMPORTS. numpy and graph_walls at module scope, which is numpy, PyYAML and
trinary_map. Nothing else, so the test file keeps collecting in CI's container.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import graph_walls as gw                                      # noqa: E402

SPEC = 'SPEC_b2_schema v0.2, §2-§7'

# ── registered in the spec before this file existed ─────────────────────────
AXIS_TIE = 1e-9          # §2: the two largest dots this close -> no axis
S_MAX_M = 0.5            # §6: the largest separation of a pair
TAU_E_CELLS = 1.0        # §7: tau_e = rho
# §5's tau_c = eps + rho, §2's rho/2 extension and §4's "wider than g" are
# applied where they are used, from the map's own rho, eps and g.

AX = np.array([[d[2], d[3]] for d in gw.DIRS])
OPP = {k: next(j for j in range(len(gw.DIRS))
               if np.array_equal(AX[j], -AX[k]))
       for k in range(len(gw.DIRS))}
END = ('q0', 'q1')


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'schema_walls: {msg}', file=sys.stderr)
    sys.exit(2)


def _pt(v) -> list[float]:
    return [float(v[0]), float(v[1])]


# ─────────────────────────────── thresholds ──────────────────────────────────

def tau_c(eps: float, rho: float) -> float:
    """§5: tau_c = eps + rho, 0.2707 m at rho = 0.1."""
    return eps + rho


def tau_e(rho: float) -> float:
    """§7: tau_e = rho."""
    return TAU_E_CELLS * rho


def thresholds(rho: float, params) -> dict:
    """§8's thresholds (rho, eps, g, tau_c, s_max, tau_e), and Def 3's b."""
    eps = params.eps(rho)
    return {'rho': rho, 'eps': eps, 'g': params.g, 'tau_c': tau_c(eps, rho),
            's_max': S_MAX_M, 'tau_e': tau_e(rho), 'b': params.b(rho)}


# ──────────────────────────── §2, §3: axis ends ──────────────────────────────

def axis_of(normal) -> int | None:
    """§2: the `graph_walls.DIRS` index with the largest dot with `normal`.

    None when the two largest dots differ by at most AXIS_TIE: such a segment
    has no axis. A fit near 45° has an axis, which can flip with rounding.
    """
    dots = AX @ np.asarray(normal, dtype=float)
    order = np.argsort(-dots, kind='stable')
    if dots[order[0]] - dots[order[1]] <= AXIS_TIE:
        return None
    return int(order[0])


def axis_ends(graph: dict, el: dict, seg_of: np.ndarray,
              rho: float) -> list[dict]:
    """§2 for every segment, in the graph's order.

    Axis members are the members whose `dir` is the axis. Every other member is
    a riser: W4 rules out a member facing the other way, and one that did would
    be counted in `n_opposite` rather than assumed away. `q0` and `q1` are the
    smallest and largest projections of the axis members on the segment's
    reported line, each extended by rho/2 as Def 2 extends `p0` and `p1`. A
    segment with no axis, or with no axis member, has no axis ends, and
    `no_axis_ends` says which.
    """
    line_of = {int(j): li for li, ln in enumerate(graph['lines'])
               for j in ln['segments']}
    out = []
    for s in graph['segments']:
        sid = int(s['id'])
        n = np.array(s['normal'], dtype=float)
        u = np.array(gw.dir_of(s['theta']), dtype=float)
        mem = np.nonzero(seg_of == sid)[0]
        info = {'id': sid, 'line': line_of.get(sid), 'theta': float(s['theta']),
                'n': n, 'u': u, 'd': float(np.dot(s['p0'], n)),
                'axis': axis_of(n), 'no_axis_ends': None,
                'n_members': int(mem.size), 'n_axis_members': 0,
                'n_risers': 0, 'n_opposite': 0,
                'axis_members': np.empty(0, dtype=np.int64),
                'tq0': None, 'tq1': None, 'q0': None, 'q1': None, 'm': None,
                'end_states': None}
        k = info['axis']
        if k is None:
            info['no_axis_ends'] = 'no_axis'
            out.append(info)
            continue
        dirs = el['dir'][mem]
        axm = mem[dirs == k]
        info['n_axis_members'] = int(axm.size)
        info['n_opposite'] = int(np.count_nonzero(dirs == OPP[k]))
        info['n_risers'] = int(mem.size) - int(axm.size) - info['n_opposite']
        if axm.size == 0:
            info['no_axis_ends'] = 'no_axis_member'
            out.append(info)
            continue
        t = el['x'][axm] * u[0] + el['y'][axm] * u[1]
        tq0 = float(t.min()) - rho / 2.0
        tq1 = float(t.max()) + rho / 2.0
        q0 = tq0 * u + info['d'] * n
        q1 = tq1 * u + info['d'] * n
        info.update(axis_members=axm, tq0=tq0, tq1=tq1, q0=q0, q1=q1,
                    m=(q0 + q1) / 2.0)
        out.append(info)
    return out


def axis_end_states(infos: list[dict], el: dict, grid: np.ndarray, rho: float,
                    origin: tuple[float, float], params) -> None:
    """§3, in place: Def 5 read at each axis end.

    The bands are [t(q0) - g, t(q0)] and [t(q1), t(q1) + g] on the segment's
    reported line. A band cell counts as the segment's own only if it is the
    cell of one of its axis members: a riser's cell does not, which is what
    lets the cell at an inside corner read `occupied`. Precedence is Def 5's,
    through graph_walls.endpoint_state unchanged.
    """
    b = params.b(rho)
    for s in infos:
        if s['q0'] is None:
            continue
        own = np.zeros(grid.shape, dtype=bool)
        axm = s['axis_members']
        own[el['row'][axm], el['col'][axm]] = True
        s['end_states'] = [
            gw.endpoint_state(grid, rho, origin, s['theta'], s['d'],
                              s['tq0'] - params.g, s['tq0'], b, own,
                              params.endpoint_precedence),
            gw.endpoint_state(grid, rho, origin, s['theta'], s['d'],
                              s['tq1'], s['tq1'] + params.g, b, own,
                              params.endpoint_precedence)]


# ─────────────────────────────── §4: openings ────────────────────────────────

def openings(graph: dict, infos: list[dict], grid: np.ndarray, rho: float,
             origin: tuple[float, float], params) -> list[dict]:
    """§4: every gap on every lines[] entry, each flagged if it is an opening.

    An entry's segments are ordered by t(p0) along the entry's line. For two
    consecutive segments that both have axis ends, the gap runs from t(q1) of
    the first to t(q0) of the second, on that line. A neighbour without axis
    ends still counts as a neighbour, so no gap is formed beside it. The state
    is Def 4's, through graph_walls.gap_record. An opening is a free gap wider
    than g.
    """
    by = {s['id']: s for s in infos}
    p0_of = {int(s['id']): s['p0'] for s in graph['segments']}
    b = params.b(rho)
    out = []
    for li, ln in enumerate(graph['lines']):
        u = np.array(gw.dir_of(ln['theta']), dtype=float)
        ids = sorted((int(j) for j in ln['segments']),
                     key=lambda j: float(np.dot(p0_of[j], u)))
        for i, j in zip(ids, ids[1:]):
            si, sj = by[i], by[j]
            if si['q0'] is None or sj['q0'] is None:
                continue
            t0 = float(si['q1'] @ u)
            t1 = float(sj['q0'] @ u)
            rec = gw.gap_record(grid, rho, origin, ln['theta'], ln['d'],
                                t0, t1, b)
            width = t1 - t0
            out.append({'line': li, 'segments': [i, j], 't0': t0, 't1': t1,
                        'width_m': width, 'state': rec['state'],
                        'opening': bool(rec['state'] == 'free'
                                        and width > params.g)})
    return out


# ──────────────────────────── §5: inside corners ─────────────────────────────

def inside_corners(infos: list[dict], tau_c_m: float) -> list[dict]:
    """§5: two segments with axis ends and perpendicular axes.

    Their reported lines meet at c. For each segment, its nearer axis end (on
    a tie, q0) must lie within tau_c of c, and from it toward its other axis
    end the segment must run in the direction of the other segment's axis.
    Two segments that fail any of this form no corner.
    """
    ok = [s for s in infos if s['q0'] is not None]
    out = []
    for a, b in itertools.combinations(ok, 2):
        if float(AX[a['axis']] @ AX[b['axis']]) != 0.0:
            continue
        c = np.linalg.solve(np.array([a['n'], b['n']]),
                            np.array([a['d'], b['d']]))
        near = []
        for s in (a, b):
            dq = (float(np.linalg.norm(s['q0'] - c)),
                  float(np.linalg.norm(s['q1'] - c)))
            e = 0 if dq[0] <= dq[1] else 1
            near.append((e, dq[e]))
        if not (near[0][1] <= tau_c_m and near[1][1] <= tau_c_m):
            continue
        runs = True
        for s, o, (e, _dist) in ((a, b, near[0]), (b, a, near[1])):
            q_near, q_far = (s['q0'], s['q1']) if e == 0 else (s['q1'], s['q0'])
            runs = runs and float((q_far - q_near) @ AX[o['axis']]) > 0.0
        if not runs:
            continue
        out.append({'segments': [a['id'], b['id']], 'c': _pt(c),
                    'near_end': [END[near[0][0]], END[near[1][0]]],
                    'near_m': [near[0][1], near[1][1]],
                    'states': [a['end_states'][near[0][0]],
                               b['end_states'][near[1][0]]]})
    return out


# ──────────────────────────── §6, §7: slabs ──────────────────────────────────

def pairs(infos: list[dict], s_max_m: float) -> list[dict]:
    """§6: back-to-back pairs, each segment in at most one.

    Opposite axes; each midpoint on the occupied side of the other's reported
    line; separation (the larger of the two midpoint-to-line distances) at
    most s_max; the axis-end intervals, projected on the first segment's
    direction, overlapping; and each the other's nearest qualifying partner by
    separation, on a tie by the larger overlap. The first segment of a pair is
    the one with the smaller id.
    """
    ok = [s for s in infos if s['q0'] is not None]
    qual: dict[int, list] = {}
    for a in ok:
        for b in ok:
            if a is b or b['axis'] != OPP[a['axis']]:
                continue
            pa = float(b['m'] @ a['n']) - a['d']
            pb = float(a['m'] @ b['n']) - b['d']
            if not (pa < 0.0 and pb < 0.0):
                continue
            tb = sorted((float(b['q0'] @ a['u']), float(b['q1'] @ a['u'])))
            overlap = min(a['tq1'], tb[1]) - max(a['tq0'], tb[0])
            if overlap <= 0.0:
                continue
            qual.setdefault(a['id'], []).append(
                (max(abs(pa), abs(pb)), overlap, b['id']))
    best = {k: min(v, key=lambda z: (z[0], -z[1])) for k, v in qual.items()}
    out = []
    for k in sorted(best):
        sep, overlap, j = best[k]
        if k < j and j in best and best[j][2] == k and sep <= s_max_m:
            out.append({'segments': [k, j], 'separation_m': sep,
                        'overlap_m': overlap})
    return out


def slab_ends(a: dict, b: dict, tau_e_m: float) -> list[dict]:
    """§7 at both ends of the pair (a, b), along a's direction.

    Each end takes a's axis end and b's on that side. It is aligned when their
    positions differ by at most tau_e. Its state is `blocked` if either axis
    end state is occupied, `unobserved` if either is unknown, and `free`
    otherwise. An aligned free end is a slab end, with an outside corner at
    each of its two axis ends. Any other end is recorded as `misaligned` or
    with its state, and gives no outside corner.
    """
    tb = sorted(((float(b['q0'] @ a['u']), 0), (float(b['q1'] @ a['u']), 1)))
    out = []
    for (t_a, ea), (t_b, eb) in zip(((a['tq0'], 0), (a['tq1'], 1)), tb):
        sa, sb = a['end_states'][ea], b['end_states'][eb]
        state = ('blocked' if 'occupied' in (sa, sb) else
                 'unobserved' if 'unknown' in (sa, sb) else 'free')
        diff = abs(t_a - t_b)
        aligned = diff <= tau_e_m
        record = ('misaligned' if not aligned else
                  'slab_end' if state == 'free' else state)
        pts = [_pt(a['q0'] if ea == 0 else a['q1']),
               _pt(b['q0'] if eb == 0 else b['q1'])]
        out.append({'axis_ends': [[a['id'], END[ea]], [b['id'], END[eb]]],
                    'points': pts, 'diff_m': diff, 'aligned': aligned,
                    'state': state, 'states': [sa, sb], 'record': record,
                    'slab_end': record == 'slab_end',
                    'outside_corners': pts if record == 'slab_end' else []})
    return out


# ─────────────────────────── one map, end to end ─────────────────────────────

def segment_record(s: dict) -> dict:
    """One segment's §2-§3 fields, as plain JSON."""
    k = s['axis']
    ends = None
    if s['q0'] is not None:
        ends = {'t': [s['tq0'], s['tq1']], 'q0': _pt(s['q0']),
                'q1': _pt(s['q1']), 'm': _pt(s['m']),
                'states': list(s['end_states'] or [])}
    return {'id': s['id'], 'line': s['line'], 'theta': s['theta'],
            'normal': _pt(s['n']), 'd': s['d'], 'axis': k,
            'axis_name': None if k is None else gw.DIRS[k][4],
            'n_members': s['n_members'], 'n_axis_members': s['n_axis_members'],
            'n_risers': s['n_risers'], 'n_opposite': s['n_opposite'],
            'no_axis_ends': s['no_axis_ends'], 'axis_ends': ends}


def schema_of(graph: dict, el: dict, seg_of: np.ndarray, grid: np.ndarray,
              rho: float, origin: tuple[float, float], params) -> dict:
    """§2-§7 for one extraction. Reads no file, no git and no truth."""
    th = thresholds(rho, params)
    infos = axis_ends(graph, el, seg_of, rho)
    axis_end_states(infos, el, grid, rho, origin, params)
    corners = inside_corners(infos, th['tau_c'])
    prs = pairs(infos, th['s_max'])
    by = {s['id']: s for s in infos}
    for p in prs:
        p['ends'] = slab_ends(by[p['segments'][0]], by[p['segments'][1]],
                              th['tau_e'])
    gaps = openings(graph, infos, grid, rho, origin, params)
    n_slab_ends = sum(e['slab_end'] for p in prs for e in p['ends'])
    counts = {
        'segments': len(infos),
        'with_axis_ends': sum(s['q0'] is not None for s in infos),
        'no_axis': sum(s['no_axis_ends'] == 'no_axis' for s in infos),
        'no_axis_member': sum(s['no_axis_ends'] == 'no_axis_member'
                              for s in infos),
        'inside_corners': len(corners),
        'pairs': len(prs),
        'slab_ends': n_slab_ends,
        'outside_corners': 2 * n_slab_ends,
        'gaps': len(gaps),
        'openings': sum(g['opening'] for g in gaps),
    }
    return {'thresholds': th, 'counts': counts,
            'segments': [segment_record(s) for s in infos],
            'inside_corners': corners, 'pairs': prs, 'gaps': gaps}


def schema_for_map(side: dict, graph_walls_commit: str) -> dict:
    """§8's record for one map, from the dict divergence_walls.load_side returns.

    g is the graph JSON's own thresholds.g, the g load_side extracted with; rho
    and the origin are the map's. The caller names the graph_walls commit, and
    the stage that calls this refuses an edited tree, so the commit is the code
    that ran.
    """
    graph = side['graph']
    params = gw.Params(g=graph['thresholds']['g']['value'])
    body = schema_of(graph, side['el'], side['seg_of'], side['grid'],
                     side['rho'], tuple(side['origin']), params)
    return {'map': side['stem'], 'map_sha256': graph['map_sha256'],
            'graph_walls_commit': graph_walls_commit, 'spec': SPEC,
            'frame': graph['frame'],
            'positions': 'the map frame, as graph_walls reports segments',
            **body}


def write_record(record: dict, out_dir: Path) -> tuple[Path, str]:
    """One map's file, `<map>.json` in `out_dir`, never over one that exists.

    The caller gives each run its own directory (§8: no per-map file that a
    stage pins is rewritten). Returns the path and the sha256 of the bytes
    written.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{record['map']}.json"
    if path.exists():
        die(f'{path} exists; a per-map file is never rewritten')
    blob = (json.dumps(record, indent=2, sort_keys=True) + '\n').encode('utf-8')
    path.write_bytes(blob)
    return path, hashlib.sha256(blob).hexdigest()
