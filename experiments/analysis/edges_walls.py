"""The B2 face graph of one map: its nodes, and corner, slab and facing edges.

docs/specs/SPEC_b2_edges.md v0.2, §2-§4. One map at a time, from two files
joined by segment `id` and `map_sha256`: the map's schema file (schema_walls,
SPEC_b2_schema §8) and its graph_walls file. Nothing here is judged. G-E1, F*
and P-E1 to P-E4 belong to the stage that judges them, not to this file.

NO TRUTH (D-E6). Every position in a record is in robot K's map frame, as the
two input files have it. No world geometry, no spawn and no pose is imported,
read or written here, and a test reads this file's source and fails on the
names of the functions that would bring truth in. The stage casts truth through
the same `cast`, so maps and truth go through one code path: what it hands in
is faces and occluders, and nothing in this file knows which it was given.

WHAT THIS FILE OWNS, AND WHAT IT BORROWS. It owns §2's node record, §3's three
edge types and §4's per-map file. Underneath them, and not rewritten here:

    graph_walls.DIRS             the four axes a(S) and their unit vectors n̂(S)
    the schema file              axis ends, inside corners, pairs and slab ends
    the graph_walls file         each segment's p0-p1 and its obs

THE CAST IS PORTED, NOT REDESIGNED, from experiments/logs/splice/
edges_truth_fstar.py (`crossing`, `sight`, `facing_pairs`, `components`), which
computed F* under v0.1 (`6dbc3354…`). The differences are v0.2's and the build
plan's, whose 34 proposals Ali accepted on 18 Oct 2026
(experiments/logs/splice/edges_build_plan.md):
  - condition 2 is opposite axes, not exactly opposite normals (A6);
  - every slab end that carries two outside corners occludes, as the segment
    between them (A5, Q3);
  - a face's own segment never occludes its own rays, and the samples whose
    first crossing it would have been are counted as `self_first` (Q1 (a)). On
    truth that changes nothing: there lambda > 1e-9 already excluded it;
  - a crossing within 1e-9 of the first is a tie, and the lowest key wins:
    segments by id, then slab-end caps (C3, Q5);
  - condition 3 projects the crossing on S_j's fitted direction and compares it
    with the schema's own t = [tq0, tq1] (Q4).
The script's through-slab diagnostic needs the world's rectangles, so it is the
stage's. `cast` hands every sample to an optional `observe` for it.

IMPORTS. schema_walls, which holds no truth, and the standard library. The
cast is pure Python, not numpy, so that on truth it reproduces the script bit
for bit (Q34). schema_walls brings in numpy, PyYAML and trinary_map through
graph_walls, which the test files already collect with in CI's container.
"""

from __future__ import annotations

import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import schema_walls as sw                                     # noqa: E402

gw = sw.gw

SPEC = 'SPEC_b2_edges v0.2, §2-§4'
POSITIONS = 'the map frame, as the schema file and graph_walls report them'

# ── registered in the spec; the stage checks each against its text ─────────
SLACK = 1e-9             # C1's closed ends; lambda > 0 as lambda > SLACK (Q26)
PARALLEL = 1e-12         # C2: |d x e| <= PARALLEL |e| is never crossed
W_MIN_M = 0.2            # §3.3, E-O4
MIN_SAMPLES = 4          # §3.3: |V_ij| + |V_ji| >= 2 w_min / rho (Q25)

AXIS_VEC = {d[4]: (float(d[2]), float(d[3])) for d in gw.DIRS}
OPPOSITE = {a: b for a, va in AXIS_VEC.items() for b, vb in AXIS_VEC.items()
            if va == (-vb[0], -vb[1])}
OBS_FIELDS = ('n_scans', 'n_hits', 'n_contradictions', 't_first', 't_last',
              'bearing_hist', 'incidence_hist')
CAUSES = ('cap', 'excluded_segment', 'outside_interval', 'perpendicular_axis',
          'same_axis')
SEGMENT, CAP = 0, 1      # the first element of an occluder's key (C3, Q27)
PROVENANCE_GIVEN = ('builder_commit', 'graph_walls_file_sha256',
                    'schema_file_sha256')


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'edges_walls: {msg}', file=sys.stderr)
    sys.exit(2)


def dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def sub(a, b) -> tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def _pt(v) -> list[float]:
    return [float(v[0]), float(v[1])]


def _bits(v):
    """A float, or a list of floats, as its exact bits."""
    if isinstance(v, (list, tuple)):
        return [_bits(x) for x in v]
    return float(v).hex()


# ─────────────────────────────── the inputs ──────────────────────────────────

def read_inputs(schema_path, graph_path) -> tuple[dict, dict, str, str]:
    """The schema file and the graph_walls file, and each one's sha256."""
    out = []
    for path in (Path(schema_path), Path(graph_path)):
        blob = path.read_bytes()
        out.append((json.loads(blob), hashlib.sha256(blob).hexdigest()))
    (schema_rec, schema_sha), (graph, graph_sha) = out
    return schema_rec, graph, schema_sha, graph_sha


def join(schema_rec: dict, graph: dict) -> dict:
    """The two files joined by `id` and `map_sha256`, or a refusal (Q7).

    Strict: one map_sha256, one set of segment ids, and on every segment a
    `theta` and a `normal` that are bit-equal in the two files. A refusal here
    is a finding to bring back, never a reason for a tolerance. Returns the
    graph_walls segments by id.
    """
    sha = schema_rec.get('map_sha256')
    if sha is None or sha != graph.get('map_sha256'):
        die(f'map_sha256 is {sha!r} in the schema file and '
            f'{graph.get("map_sha256")!r} in the graph_walls file')
    ids_s = [int(s['id']) for s in schema_rec['segments']]
    ids_g = [int(s['id']) for s in graph['segments']]
    for what, ids in (('schema file', ids_s), ('graph_walls file', ids_g)):
        if len(set(ids)) != len(ids):
            die(f'the {what} repeats a segment id: {sorted(ids)}')
    if set(ids_s) != set(ids_g):
        die(f'the segment ids differ: only in the schema file '
            f'{sorted(set(ids_s) - set(ids_g))}, only in the graph_walls file '
            f'{sorted(set(ids_g) - set(ids_s))}')
    by = {int(s['id']): s for s in graph['segments']}
    for s in schema_rec['segments']:
        g = by[int(s['id'])]
        for key in ('theta', 'normal'):
            if _bits(s[key]) != _bits(g[key]):
                die(f'segment {s["id"]}: {key} is {s[key]!r} in the schema '
                    f'file and {g[key]!r} in the graph_walls file')
    return by


# ──────────────────────────────── §2: nodes ──────────────────────────────────

def nodes_of(schema_rec: dict, graph: dict) -> tuple[list, list]:
    """§2 and D-E1: the nodes, and the segments that are not nodes.

    A node is a segment with axis ends. Its record holds the fields of §2's
    table and no others (Q23): `axis` is a(S) as graph_walls names it,
    `n_hat` its unit vector, `normal` the schema's fitted normal, kept and not
    used by §3.3's rays; `t` is the schema's [tq0, tq1], which L comes from and
    condition 3 compares against. The 7 obs fields are copied as graph_walls
    wrote them, an empty or all-zero histogram included (O2). A segment with
    no axis ends is listed in `excluded` with the schema's reason, and still
    occludes (§3.3).
    """
    by = join(schema_rec, graph)
    nodes, excluded = [], []
    for s in sorted(schema_rec['segments'], key=lambda z: int(z['id'])):
        sid = int(s['id'])
        ends = s['axis_ends']
        if ends is None:
            if s['no_axis_ends'] not in ('no_axis', 'no_axis_member'):
                die(f'segment {sid} has no axis ends and no reason the schema '
                    f'gives: {s["no_axis_ends"]!r}')
            excluded.append({'id': sid, 'reason': s['no_axis_ends']})
            continue
        name = gw.DIRS[s['axis']][4]
        obs = by[sid].get('obs')
        if obs is None:
            die(f'segment {sid} carries no obs; §2 reads graph_walls Part B')
        missing = [f for f in OBS_FIELDS if f not in obs]
        if missing:
            die(f'segment {sid}: obs lacks {missing}')
        t0, t1 = float(ends['t'][0]), float(ends['t'][1])
        node = {'id': sid, 'line': s['line'], 'axis': name,
                'n_hat': list(AXIS_VEC[name]), 'normal': _pt(s['normal']),
                'q0': _pt(ends['q0']), 'q1': _pt(ends['q1']),
                'm': _pt(ends['m']), 't': [t0, t1], 'L': t1 - t0,
                'end_states': list(ends['states'])}
        for f in OBS_FIELDS:
            v = obs[f]
            node[f] = list(v) if isinstance(v, list) else v
        nodes.append(node)
    return nodes, excluded


# ───────────────────────────── §3.1, §3.2 ────────────────────────────────────

def _ordered(ids, what: str) -> tuple[int, int]:
    a, b = (int(v) for v in ids)
    if not a < b:
        die(f'a schema {what} lists its segments as {[a, b]}; the schema puts '
            'the smaller id first, and its attributes follow that order')
    return a, b


def corner_edges(schema_rec: dict) -> list[dict]:
    """§3.1: one `corner` edge per inside corner, its four attributes copied."""
    out = []
    for k in schema_rec['inside_corners']:
        a, b = _ordered(k['segments'], 'inside corner')
        out.append({'type': 'corner', 'i': a, 'j': b,
                    'attrs': {'c': _pt(k['c']),
                              'near_end': list(k['near_end']),
                              'near_m': [float(v) for v in k['near_m']],
                              'states': list(k['states'])}})
    return sorted(out, key=lambda e: (e['i'], e['j']))


def slab_edges(schema_rec: dict) -> list[dict]:
    """§3.2: one `slab` edge per pair, with four fields of each of its ends."""
    out = []
    for p in schema_rec['pairs']:
        a, b = _ordered(p['segments'], 'pair')
        out.append({'type': 'slab', 'i': a, 'j': b,
                    'attrs': {'separation_m': p['separation_m'],
                              'overlap_m': p['overlap_m'],
                              'ends': [{f: e[f] for f in ('record', 'diff_m',
                                                          'aligned', 'state')}
                                       for e in p['ends']]}})
    return sorted(out, key=lambda e: (e['i'], e['j']))


# ─────────────────────────────── §3.3: facing ────────────────────────────────

def occluders_of(schema_rec: dict, graph: dict) -> list[dict]:
    """§3.3's O: every segment as graph_walls' closed p0-p1, then every cap.

    Segments are keyed (0, id), nodes and excluded segments alike. A slab end
    occludes when it carries two outside corners, which §7 gives only to a
    `slab_end` record (Q3), as the closed segment between them, keyed
    (1, pair, end) after every segment (Q27). Any other end leaves its slab
    open to rays (E-O6).
    """
    by = join(schema_rec, graph)
    out = [{'key': [SEGMENT, sid], 'kind': 'segment',
            'p0': _pt(by[sid]['p0']), 'p1': _pt(by[sid]['p1'])}
           for sid in sorted(by)]
    for pi, p in enumerate(schema_rec['pairs']):
        for ei, e in enumerate(p['ends']):
            oc = e['outside_corners']
            if len(oc) not in (0, 2):
                die(f'pair {pi} end {ei} carries {len(oc)} outside corners')
            if oc:
                out.append({'key': [CAP, pi, ei], 'kind': 'cap',
                            'p0': _pt(oc[0]), 'p1': _pt(oc[1])})
    return out


def n_samples(length: float, rho: float) -> int:
    """§3.3, C5: floor((L + 1e-9) / rho) samples, so no partial last cell."""
    return math.floor((length + SLACK) / rho)


def crossing(p, d, occ: dict):
    """(lambda, point) where p + lambda*d crosses occ's closed p0-p1, or None.

    C1: the occluder is closed, with 1e-9 m of slack at each end, and
    lambda > 1e-9 (Q26). C2: an occluder parallel to the ray is never crossed,
    even when the ray runs along its line.
    """
    q0, q1 = occ['p0'], occ['p1']
    e = sub(q1, q0)
    le = math.hypot(*e)
    den = cross(d, e)
    if abs(den) <= PARALLEL * le:
        return None
    w = sub(q0, p)
    lam = cross(w, e) / den
    mu = cross(w, d) / den
    if lam <= SLACK or not (-SLACK / le <= mu <= 1.0 + SLACK / le):
        return None
    return lam, (p[0] + lam * d[0], p[1] + lam * d[1])


def face_of(node: dict) -> dict:
    """A node as `cast` takes a face.

    `u` = (n̂_y, -n̂_x) steps the samples along the exact axis; it points the
    way the schema's direction does, so q0 is the low end along it too.
    `span` is condition 3's: the fitted direction (n_y, -n_x) of the fitted
    normal, which is graph_walls' dir_of(theta) bit for bit since `normal` is
    normal_of(theta), and the schema's own [tq0, tq1] along it (Q4). `own` is
    the key of the face's own segment, which never occludes its rays (Q1).
    """
    n = (float(node['n_hat'][0]), float(node['n_hat'][1]))
    fit = node['normal']
    return {'key': node['id'], 'own': (SEGMENT, node['id']),
            'axis': node['axis'], 'n_hat': n, 'u': (n[1], -n[0]),
            'q0': tuple(node['q0']), 'L': node['L'], 'm': tuple(node['m']),
            'span': ((float(fit[1]), -float(fit[0])), node['t'][0],
                     node['t'][1])}


def _first(hits: list) -> tuple[tuple, bool]:
    """C3, Q5: the lowest key within 1e-9 of the first, and whether it tied."""
    lam0 = min(h[0] for h in hits)
    near = [h for h in hits if h[0] - lam0 <= SLACK]
    return min(near, key=lambda h: h[1]), len(near) > 1


def _in_span(x, face: dict) -> bool:
    """§3.3 condition 3, closed with 1e-9 m of slack (C1)."""
    w, lo, hi = face['span']
    t = dot(x, w)
    return lo - SLACK <= t <= hi + SLACK


def cast(faces: list[dict], occluders: list[dict], rho: float,
         observe=None) -> tuple[dict, dict]:
    """§3.3's rays: V[i][j], the lambda of every sample of face i that sees j.

    A face is {key, own, axis, n_hat, u, q0, L, m, span}; an occluder is
    {key, kind, p0, p1}, with `kind` 'segment' or 'cap'. Sample k of face i is
    q0 + (k + 1/2) rho u, for k < n_samples(L, rho), and its ray runs along
    n_hat, not the fitted normal. H is the first occluder it crosses, its own
    segment left out (Q1). It sees face j when H is j's own segment, the axes
    are opposite (condition 2, A6) and the crossing lies in j's span
    (condition 3). Otherwise it is blocked, by one of CAUSES, or has no hit
    (C4).

    Per face: samples; seen, by partner; blocked, in all and by cause; no hit;
    ties; and `self_first`, the samples whose first crossing, by the same
    rule, would have been the face's own segment. `observe`, when given, is
    handed every sample as a dict, which is how the stage counts its
    through-slab rays without this file knowing where the slabs are.
    """
    occ = sorted(({**o, 'key': tuple(o['key'])} for o in occluders),
                 key=lambda o: o['key'])
    if len({o['key'] for o in occ}) != len(occ):
        die('two occluders share a key')
    by_own = {tuple(f['own']): f for f in faces}
    V = {f['key']: {} for f in faces}
    tally = {}
    for fi in faces:
        own, d = tuple(fi['own']), fi['n_hat']
        t = {'samples': n_samples(fi['L'], rho), 'seen': {}, 'blocked': 0,
             'blocked_by': dict.fromkeys(CAUSES, 0), 'no_hit': 0, 'ties': 0,
             'self_first': 0}
        tally[fi['key']] = t
        for k in range(t['samples']):
            s = (k + 0.5) * rho
            p = (fi['q0'][0] + s * fi['u'][0], fi['q0'][1] + s * fi['u'][1])
            hits, mine = [], None
            for o in occ:
                c = crossing(p, d, o)
                if c is None:
                    continue
                h = (c[0], o['key'], o, c[1])
                if o['key'] == own:
                    mine = h
                else:
                    hits.append(h)
            if mine is not None and _first(hits + [mine])[0][1] == own:
                t['self_first'] += 1
            rec = {'face': fi['key'], 'k': k, 'p': p, 'd': d, 'hit': None,
                   'lam': None, 'partner': None, 'outcome': 'no_hit'}
            if not hits:
                t['no_hit'] += 1
                if observe is not None:
                    observe(rec)
                continue
            (lam, key, o, x), tie = _first(hits)
            if tie:
                t['ties'] += 1
            fj = by_own.get(key)
            if o['kind'] == 'cap':
                cause = 'cap'
            elif fj is None:
                cause = 'excluded_segment'
            elif fj['axis'] == fi['axis']:
                cause = 'same_axis'
            elif fj['axis'] != OPPOSITE[fi['axis']]:
                cause = 'perpendicular_axis'
            elif not _in_span(x, fj):
                cause = 'outside_interval'
            else:
                cause = None
            rec.update(hit=key, lam=lam)
            if cause is None:
                V[fi['key']].setdefault(fj['key'], []).append(lam)
                t['seen'][fj['key']] = t['seen'].get(fj['key'], 0) + 1
                rec.update(partner=fj['key'], outcome='seen')
            else:
                t['blocked'] += 1
                t['blocked_by'][cause] += 1
                rec['outcome'] = cause
            if observe is not None:
                observe(rec)
    return V, tally


def gap_m(lams: list[float]) -> float:
    """§3.3's gap_m: the median of lambda; with an even count, the mean of the
    two middle values (C6)."""
    return statistics.median(lams)


def facing_edges(faces: list[dict], V: dict, rho: float) -> tuple[list, list]:
    """§3.3's edge rule and attributes, once per pair, the lower key as S_i.

    Every opposite pair with a sighting is formed; it is a `facing` edge when
    |V_ij| + |V_ji| >= MIN_SAMPLES, and is listed below w_min otherwise (A8).
    offset_m is (m(S_j) - m(S_i)) . u_i and bearing_deg atan2(offset_m, gap_m),
    so for opposite faces the reverse direction gives the same two numbers
    (A7). Pairs come in (i, j) order (Q31).
    """
    order = sorted(faces, key=lambda f: f['key'])
    edges, below = [], []
    for a, fi in enumerate(order):
        for fj in order[a + 1:]:
            if fj['axis'] != OPPOSITE[fi['axis']]:
                continue
            lij = V.get(fi['key'], {}).get(fj['key'], [])
            lji = V.get(fj['key'], {}).get(fi['key'], [])
            n = len(lij) + len(lji)
            if n == 0:
                continue
            gap = gap_m(lij + lji)
            off = dot(sub(fj['m'], fi['m']), fi['u'])
            attrs = {'gap_m': gap, 'width_m': rho * n / 2.0, 'offset_m': off,
                     'bearing_deg': math.degrees(math.atan2(off, gap)),
                     'n_ij': len(lij), 'n_ji': len(lji)}
            if n >= MIN_SAMPLES:
                edges.append({'type': 'facing', 'i': fi['key'],
                              'j': fj['key'], 'attrs': attrs})
            else:
                below.append({'i': fi['key'], 'j': fj['key'], 'attrs': attrs})
    return edges, below


# ───────────────────────────── §4: components ────────────────────────────────

def components(keys: list, pairs) -> list[list]:
    """§4: connected components over the (i, j) pairs, nodes only."""
    parent = {k: k for k in keys}

    def root(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    for a, b in pairs:
        if a not in parent or b not in parent:
            die(f'an edge joins {a} and {b}, and only nodes are components')
        ra, rb = root(a), root(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    groups: dict = {}
    for k in keys:
        groups.setdefault(root(k), []).append(k)
    return sorted(sorted(g) for g in groups.values())


# ─────────────────────────── one map, end to end ─────────────────────────────

def graph_of(schema_rec: dict, graph: dict, rho: float) -> dict:
    """§2-§4 for one map at the rho given. Reads no file, no git, no truth.

    Components are counted over all edge types (§4), and, as diagnostics,
    over facing edges alone and over facing and slab edges (Q24).
    """
    nodes, excluded = nodes_of(schema_rec, graph)
    keys = [n['id'] for n in nodes]
    corner, slab = corner_edges(schema_rec), slab_edges(schema_rec)
    for e in corner + slab:
        for end in (e['i'], e['j']):
            if end not in keys:
                die(f'a {e["type"]} edge names segment {end}, which is not a '
                    'node')
    faces = [face_of(n) for n in nodes]
    occluders = occluders_of(schema_rec, graph)
    V, tally = cast(faces, occluders, rho)
    facing, below = facing_edges(faces, V, rho)

    def n_components(edges):
        return len(components(keys, [(e['i'], e['j']) for e in edges]))

    return {'nodes': nodes, 'excluded': excluded, 'faces': faces,
            'occluders': occluders, 'corner': corner, 'slab': slab,
            'facing': facing, 'below_w_min': below, 'V': V, 'tally': tally,
            'components': {'all': n_components(corner + slab + facing),
                           'facing': n_components(facing),
                           'facing_and_slab': n_components(facing + slab)}}


def per_face(faces: list[dict], tally: dict) -> list[dict]:
    """§4's diagnostics, one row per face; partners as [id, n] lists (Q32)."""
    out = []
    for f in sorted(faces, key=lambda z: z['key']):
        t = tally[f['key']]
        out.append({'id': f['key'], 'samples': t['samples'],
                    'seen': [[j, n] for j, n in sorted(t['seen'].items())],
                    'blocked': t['blocked'],
                    'blocked_by': dict(t['blocked_by']),
                    'no_hit': t['no_hit'], 'ties': t['ties'],
                    'self_first': t['self_first']})
    return out


def edges_for_map(schema_rec: dict, graph: dict, provenance: dict) -> dict:
    """§4's record for one map, at the map's own rho (Q2).

    The caller gives the sha256 of the two files and the commit of this file
    (Q21); map_sha256 and graph_walls_commit come from the schema file, whose
    commit is the code that re-extracted the segments it read (Q22). Beyond
    §4's keys it carries `map`, `spec`, `positions`, `occluders`,
    graph_walls_file_sha256, n_ij and n_ji, and slack, parallel and
    min_samples (Q20), and diagnostics by cause, self_first and components by
    edge type (Q24).
    """
    if sorted(provenance) != sorted(PROVENANCE_GIVEN):
        die(f'provenance gives {sorted(provenance)}; the record needs exactly '
            f'{sorted(PROVENANCE_GIVEN)}')
    rho = schema_rec['thresholds']['rho']
    G = graph_of(schema_rec, graph, rho)
    return {
        'map': schema_rec['map'], 'spec': SPEC, 'frame': schema_rec['frame'],
        'positions': POSITIONS,
        'provenance': {
            'map_sha256': schema_rec['map_sha256'],
            'schema_file_sha256': provenance['schema_file_sha256'],
            'graph_walls_commit': schema_rec['graph_walls_commit'],
            'builder_commit': provenance['builder_commit'],
            'graph_walls_file_sha256': provenance['graph_walls_file_sha256']},
        'thresholds': {'rho': rho, 'w_min': W_MIN_M, 'slack': SLACK,
                       'parallel': PARALLEL, 'min_samples': MIN_SAMPLES},
        'nodes': G['nodes'], 'excluded': G['excluded'],
        'edges': G['corner'] + G['slab'] + G['facing'],
        'occluders': G['occluders'],
        'diagnostics': {
            'per_face': per_face(G['faces'], G['tally']),
            'below_w_min': G['below_w_min'],
            'components': {'facing': G['components']['facing'],
                           'facing_and_slab':
                               G['components']['facing_and_slab']}},
        'counts': {'nodes': len(G['nodes']), 'excluded': len(G['excluded']),
                   'corner': len(G['corner']), 'slab': len(G['slab']),
                   'facing': len(G['facing']),
                   'components': G['components']['all']},
    }


def write_record(record: dict, out_dir: Path) -> tuple[Path, str]:
    """One map's file, `<map>.json` in `out_dir`, never over one that exists.

    schema_walls.write_record's body under this file's name (Q30), so that a
    refusal names edges_walls. Returns the path and the sha256 of the bytes
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
