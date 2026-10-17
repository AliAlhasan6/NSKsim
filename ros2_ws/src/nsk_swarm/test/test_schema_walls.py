"""B2 schema, §2-§7 -- experiments/analysis/schema_walls.py.

docs/specs/SPEC_b2_schema.md v0.2, §2-§7 and §8's per-map file, on hand-built
segment tables and small synthetic grids only: no corpus, no map, no graph JSON,
no world SDF, no git. Matching to truth (§9) and the predictions (§10) belong to
the stage, and are not here.

THE TABLE BELOW WAS WRITTEN BEFORE THE CODE WAS RUN. Each row is what the case
must produce, fixed in advance.

  id   case                                               expected
  ───────────────────────────────────────────────────────────────────────────
  W0   the registered numbers                             tau_c 0.2707 m at rho
                                                          0.1, s_max 0.5 m,
                                                          tau_e = rho, tie 1e-9
  W1   axis of (1, 0), (0, -1), and 45° exactly           +x, -y, none
  W2   two largest dots 2e-9 apart, then 0.5e-9 apart     +x, then none (§2)
  W3   five axis members and one riser                    q0, q1 from the five,
                                                          extended rho/2; 1 riser
  W4   a 45° normal; members that are all risers          no axis ends: no_axis,
                                                          no_axis_member
  W5   the riser's cell in the band beyond q1 (§3)        occupied; free if every
                                                          member's cell were own
  W6   a 0.3 m free gap on one lines[] entry              free, an opening
  W7   the same gap with one unknown band cell            unknown, no opening
  W8   a 0.2 m free gap                                   free, no opening
  W9   a segment with no axis ends between the two        no gap formed
  W10  corner whose nearer axis ends tie (§5)             q0 taken, a corner
  W11  the tie with the other segment's axis reversed     no corner
  W12  nearer end at tau_c - 1e-6, then tau_c + 1e-6      a corner, then none
  W13  two faces meeting as an outside corner             no inside corner
  W14  opposite faces 0.5 m apart, then 0.5625 m          a pair, then none
  W15  two partners behind one face                       the nearer one only
  W16  two partners at one separation                     the larger overlap
  W17  slab ends: free; occupied; unknown; occupied and   slab_end; blocked;
       unknown; 0.1 m off; 0.125 m off                    unobserved; blocked;
                                                          slab_end; misaligned
  W18  a room of 1-cell walls, through graph_walls        4 segments, 4 inside
                                                          corners, no pair, no gap
  W19  a 0.2 m slab in free space, through graph_walls    1 pair near 0.2 m,
                                                          2 slab ends, 4 outside
                                                          corners
  W20  the per-map record                                 stem, sha256, commit,
                                                          thresholds; plain JSON;
                                                          no truth key
  W21  write_record                                       sha256 of the bytes;
                                                          refuses an existing file
  W22  the module's source                                names no truth function

W18 and W19 run graph_walls' own extraction, so their positions carry the
corner risers §2 describes. They are checked within tolerances, never to the
digit.

The module is loaded by location, as test_graph_walls.py loads its script.
"""

import hashlib
import importlib.util
import json
import math
import os

import numpy as np
import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'schema_walls.py')


def load_module():
    spec = importlib.util.spec_from_file_location('schema_walls', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sw = load_module()
gw = sw.gw

OCC, FREE, UNK = gw.OCC, gw.FREE, gw.UNKNOWN
RHO = 0.1
ORIGIN = (0.0, 0.0)
P = gw.Params(g=0.22)
AXIS = {d[4]: k for k, d in enumerate(gw.DIRS)}


# ─────────────────────────────── fixtures ────────────────────────────────────

def hand(sid, axis, a0, a1, at, states=('free', 'free')):
    """A §2 record built by hand for an axis-aligned segment.

    `axis` is a DIRS name. The segment lies on the line y = at (for ±y) or
    x = at (for ±x), and its axis ends span [a0, a1] along that line. `u` is
    (n_y, -n_x), which is graph_walls.dir_of for the theta of normal n.
    """
    k = AXIS[axis]
    n = sw.AX[k].astype(float)
    u = np.array([n[1], -n[0]])
    if axis in ('+y', '-y'):
        ends = (np.array([a0, at]), np.array([a1, at]))
    else:
        ends = (np.array([at, a0]), np.array([at, a1]))
    d = float(ends[0] @ n)
    t = sorted(float(p @ u) for p in ends)
    q0, q1 = t[0] * u + d * n, t[1] * u + d * n
    return {'id': sid, 'axis': k, 'n': n, 'u': u, 'd': d,
            'theta': (math.degrees(math.atan2(n[1], n[0])) - 90.0) % 360.0,
            'tq0': t[0], 'tq1': t[1], 'q0': q0, 'q1': q1,
            'm': (q0 + q1) / 2.0, 'end_states': list(states)}


def hand_el(cells):
    """An element table from (row, col, DIRS name) triples, placed as Def 1."""
    rows = np.array([r for r, _c, _n in cells], dtype=np.int64)
    cols = np.array([c for _r, c, _n in cells], dtype=np.int64)
    dirs = np.array([AXIS[name] for _r, _c, name in cells], dtype=np.int64)
    drow = np.array([gw.DIRS[k][0] for k in dirs], dtype=float)
    dcol = np.array([gw.DIRS[k][1] for k in dirs], dtype=float)
    return {'n': len(cells),
            'x': gw.cell_centre(ORIGIN[0], cols, RHO) + 0.5 * dcol * RHO,
            'y': gw.cell_centre(ORIGIN[1], rows, RHO) + 0.5 * drow * RHO,
            'nx': np.array([gw.DIRS[k][2] for k in dirs]),
            'ny': np.array([gw.DIRS[k][3] for k in dirs]),
            'row': rows, 'col': cols, 'dir': dirs}


def hand_graph(segments, lines):
    """A graph of (id, theta, p0) segments and (theta, d, ids) lines[] entries."""
    return {'segments': [{'id': i, 'theta': th, 'normal': list(gw.normal_of(th)),
                          'p0': list(p0)} for i, th, p0 in segments],
            'lines': [{'theta': th, 'd': d, 'segments': list(ids)}
                      for th, d, ids in lines]}


def run_of(cols, name='+y'):
    """Axis members along row 0, one per column, facing `name`."""
    return [(0, c, name) for c in cols]


def gap_map(second_cols, unknown=()):
    """Two +y segments on one lines[] entry at y = 0.05: cols 0-4 and
    `second_cols`, on a free grid whose row 0 is occupied under both."""
    grid = np.full((4, 16), FREE, dtype=np.uint8)
    grid[0, 0:5] = OCC
    grid[0, list(second_cols)] = OCC
    for rc in unknown:
        grid[rc] = UNK
    el = hand_el(run_of(range(5)) + run_of(second_cols))
    seg_of = np.array([0] * 5 + [1] * len(second_cols), dtype=np.int64)
    p0_second = gw.cell_centre(ORIGIN[0], np.array([second_cols[0]]), RHO)[0] \
        - RHO / 2.0
    graph = hand_graph([(0, 0.0, (-0.05, 0.05)), (1, 0.0, (p0_second, 0.05))],
                       [(0.0, 0.05, (0, 1))])
    return graph, el, seg_of, grid


# ─────────────────────── W0-W5: thresholds and axis ends ────────────────────

def test_w0_the_registered_numbers():
    eps = P.eps(RHO)
    assert round(sw.tau_c(eps, RHO), 4) == 0.2707
    assert round(eps + RHO / 2.0, 4) == 0.2207
    assert sw.S_MAX_M == 0.5
    assert sw.tau_e(RHO) == RHO
    assert sw.AXIS_TIE == 1e-9
    th = sw.thresholds(RHO, P)
    assert th == {'rho': RHO, 'eps': eps, 'g': 0.22, 'tau_c': eps + RHO,
                  's_max': 0.5, 'tau_e': RHO, 'b': P.b(RHO)}


def test_w1_axis_of_the_axes_and_45_degrees():
    assert sw.axis_of((1.0, 0.0)) == AXIS['+x']
    assert sw.axis_of((0.0, -1.0)) == AXIS['-y']
    assert sw.axis_of(gw.normal_of(90.0)) == AXIS['-x']
    assert sw.axis_of((math.sqrt(0.5), math.sqrt(0.5))) is None


@pytest.mark.parametrize('gap, expect', [(2e-9, '+x'), (0.5e-9, None)])
def test_w2_the_axis_tie_is_1e_9(gap, expect):
    """cos - sin = sqrt(2) sin(pi/4 - theta), so this theta puts the +x and +y
    dots `gap` apart."""
    theta = math.pi / 4.0 - gap / math.sqrt(2.0)
    got = sw.axis_of((math.cos(theta), math.sin(theta)))
    assert got == (None if expect is None else AXIS[expect])


def test_w3_axis_ends_come_from_the_axis_members_only():
    el = hand_el(run_of(range(5)) + [(0, 5, '+x')])
    seg_of = np.zeros(6, dtype=np.int64)
    graph = hand_graph([(0, 0.0, (-0.05, 0.05))], [(0.0, 0.05, (0,))])
    (s,) = sw.axis_ends(graph, el, seg_of, RHO)
    assert s['axis'] == AXIS['+y'] and s['no_axis_ends'] is None
    assert (s['n_members'], s['n_axis_members'], s['n_risers'],
            s['n_opposite']) == (6, 5, 1, 0)
    assert s['tq0'] == pytest.approx(-0.05, abs=1e-12)
    assert s['tq1'] == pytest.approx(0.45, abs=1e-12)     # not the riser's 0.6
    assert list(s['q1']) == pytest.approx([0.45, 0.05], abs=1e-12)
    assert list(s['m']) == pytest.approx([0.2, 0.05], abs=1e-12)


def test_w4_segments_without_axis_ends_say_why():
    el = hand_el([(0, 0, '+x'), (0, 1, '+y'), (3, 3, '+x')])
    seg_of = np.array([0, 0, 1], dtype=np.int64)
    graph = hand_graph([(0, 315.0, (0.0, 0.0)), (1, 0.0, (0.3, 0.3))],
                       [(315.0, 0.0, (0,)), (0.0, 0.3, (1,))])
    graph['segments'][0]['normal'] = [math.sqrt(0.5), math.sqrt(0.5)]
    a, b = sw.axis_ends(graph, el, seg_of, RHO)
    assert a['axis'] is None and a['no_axis_ends'] == 'no_axis'
    assert a['q0'] is None
    assert b['axis'] == AXIS['+y'] and b['no_axis_ends'] == 'no_axis_member'
    assert (b['n_axis_members'], b['n_risers'], b['q0']) == (0, 1, None)


def test_w5_a_risers_cell_is_not_the_segments_own():
    grid = np.full((4, 10), FREE, dtype=np.uint8)
    grid[0, 0:6] = OCC
    el = hand_el(run_of(range(5)) + [(0, 5, '+x')])
    seg_of = np.zeros(6, dtype=np.int64)
    graph = hand_graph([(0, 0.0, (-0.05, 0.05))], [(0.0, 0.05, (0,))])
    infos = sw.axis_ends(graph, el, seg_of, RHO)
    sw.axis_end_states(infos, el, grid, RHO, ORIGIN, P)
    s = infos[0]
    # Beyond q0 the band is off the grid; beyond q1 it holds the riser's cell.
    assert s['end_states'] == ['unknown', 'occupied']
    every_member = np.zeros(grid.shape, dtype=bool)
    every_member[el['row'], el['col']] = True
    assert gw.endpoint_state(grid, RHO, ORIGIN, s['theta'], s['d'], s['tq1'],
                             s['tq1'] + P.g, P.b(RHO), every_member,
                             P.endpoint_precedence) == 'free'


# ──────────────────────────── W6-W9: openings ────────────────────────────────

@pytest.mark.parametrize('second, unknown, state, opening', [
    ((8, 9, 10, 11, 12), (), 'free', True),               # W6: 0.3 m
    ((8, 9, 10, 11, 12), ((1, 6),), 'unknown', False),    # W7
    ((7, 8, 9, 10, 11), (), 'free', False),               # W8: 0.2 m <= g
])
def test_w6_w7_w8_gap_state_and_width(second, unknown, state, opening):
    graph, el, seg_of, grid = gap_map(second, unknown)
    infos = sw.axis_ends(graph, el, seg_of, RHO)
    (gap,) = sw.openings(graph, infos, grid, RHO, ORIGIN, P)
    assert gap['segments'] == [0, 1] and gap['line'] == 0
    want = (second[0] - 5) * RHO
    assert gap['width_m'] == pytest.approx(want, abs=1e-9)
    assert gap['state'] == state
    assert gap['opening'] is opening


def test_w9_no_gap_beside_a_segment_without_axis_ends():
    grid = np.full((4, 16), FREE, dtype=np.uint8)
    grid[0, 0:5] = OCC
    grid[0, 6] = OCC
    grid[0, 8:13] = OCC
    el = hand_el(run_of(range(5)) + [(0, 6, '+x')] + run_of(range(8, 13)))
    seg_of = np.array([0] * 5 + [1] + [2] * 5, dtype=np.int64)
    graph = hand_graph([(0, 0.0, (-0.05, 0.05)), (1, 0.0, (0.6, 0.05)),
                        (2, 0.0, (0.75, 0.05))], [(0.0, 0.05, (0, 1, 2))])
    infos = sw.axis_ends(graph, el, seg_of, RHO)
    assert infos[1]['no_axis_ends'] == 'no_axis_member'
    assert sw.openings(graph, infos, grid, RHO, ORIGIN, P) == []


# ─────────────────────────── W10-W13: inside corners ─────────────────────────

TAU_C = sw.tau_c(P.eps(RHO), RHO)


def test_w10_a_tie_takes_q0_and_forms_the_corner():
    a = hand(0, '+y', -0.2, 0.2, 0.0)      # both axis ends 0.2 m from c
    b = hand(1, '+x', 0.0, 1.0, 0.0)
    (corner,) = sw.inside_corners([a, b], TAU_C)
    assert corner['segments'] == [0, 1]
    assert corner['near_end'] == ['q0', 'q1']
    assert corner['c'] == pytest.approx([0.0, 0.0], abs=1e-12)
    assert corner['near_m'] == pytest.approx([0.2, 0.0], abs=1e-12)
    assert corner['states'] == ['free', 'free']


def test_w11_the_tie_with_the_other_axis_reversed_forms_none():
    """Taking q1 on the tie would make this a corner, so the rule is seen."""
    a = hand(0, '+y', -0.2, 0.2, 0.0)
    b = hand(1, '-x', 0.0, 1.0, 0.0)
    assert sw.inside_corners([a, b], TAU_C) == []


@pytest.mark.parametrize('delta, n_corners', [(-1e-6, 1), (1e-6, 0)])
def test_w12_the_nearer_end_within_tau_c(delta, n_corners):
    a = hand(0, '+y', TAU_C + delta, 2.0, 0.0)
    b = hand(1, '+x', 0.0, 1.0, 0.0)
    assert len(sw.inside_corners([a, b], TAU_C)) == n_corners


def test_w13_an_outside_meeting_is_no_inside_corner():
    a = hand(0, '+y', 0.0, 1.0, 0.0)
    b = hand(1, '-x', 0.0, 1.0, 0.0)
    assert sw.inside_corners([a, b], TAU_C) == []


# ───────────────────────────── W14-W17: slabs ────────────────────────────────

@pytest.mark.parametrize('at, n_pairs', [(0.0, 1), (-0.0625, 0)])
def test_w14_the_separation_is_at_most_s_max(at, n_pairs):
    a = hand(0, '+y', 0.0, 2.0, 0.5)
    b = hand(1, '-y', 0.0, 2.0, at)
    got = sw.pairs([a, b], sw.S_MAX_M)
    assert len(got) == n_pairs
    if n_pairs:
        assert got[0] == {'segments': [0, 1], 'separation_m': 0.5,
                          'overlap_m': 2.0}


def test_w15_each_takes_its_nearest_partner():
    a = hand(0, '+y', 0.0, 2.0, 0.5)
    b = hand(1, '-y', 0.0, 2.0, 0.25)
    c = hand(2, '-y', 0.0, 2.0, 0.0)
    assert [p['segments'] for p in sw.pairs([a, b, c], sw.S_MAX_M)] == [[0, 1]]


def test_w16_a_tie_in_separation_goes_to_the_larger_overlap():
    a = hand(0, '+y', 0.0, 2.0, 0.5)
    b = hand(1, '-y', 0.0, 1.0, 0.25)      # overlap 1.0
    c = hand(2, '-y', 0.5, 2.5, 0.25)      # overlap 1.5
    (p,) = sw.pairs([a, b, c], sw.S_MAX_M)
    assert p['segments'] == [0, 2]
    assert p['separation_m'] == 0.25 and p['overlap_m'] == 1.5


@pytest.mark.parametrize('a_states, b_states, b_from, records', [
    (('free', 'free'), ('free', 'free'), 0.0, ['slab_end', 'slab_end']),
    (('free', 'free'), ('free', 'occupied'), 0.0, ['blocked', 'slab_end']),
    (('free', 'unknown'), ('free', 'free'), 0.0, ['slab_end', 'unobserved']),
    (('occupied', 'free'), ('free', 'unknown'), 0.0, ['blocked', 'slab_end']),
    (('free', 'free'), ('free', 'free'), 0.1, ['slab_end', 'slab_end']),
    (('free', 'free'), ('free', 'free'), 0.125, ['misaligned', 'slab_end']),
])
def test_w17_slab_end_records(a_states, b_states, b_from, records):
    """End 0 joins a's q0 to b's q1 (b faces -y, so its t runs the other
    way); end 1 joins a's q1 to b's q0."""
    a = hand(0, '+y', 0.0, 2.0, 0.5, a_states)
    b = hand(1, '-y', b_from, 2.0, 0.25, b_states)
    ends = sw.slab_ends(a, b, sw.tau_e(RHO))
    assert [e['record'] for e in ends] == records
    assert ends[0]['axis_ends'] == [[0, 'q0'], [1, 'q1']]
    assert ends[1]['axis_ends'] == [[0, 'q1'], [1, 'q0']]
    assert ends[0]['diff_m'] == pytest.approx(b_from, abs=1e-12)
    for e in ends:
        assert e['slab_end'] is (e['record'] == 'slab_end')
        assert e['outside_corners'] == (e['points'] if e['slab_end'] else [])
    assert ends[0]['points'][0] == pytest.approx([0.0, 0.5], abs=1e-12)
    assert ends[0]['points'][1] == pytest.approx([b_from, 0.25], abs=1e-12)


# ──────────────────── W18-W19: through graph_walls' extraction ───────────────

def room_grid():
    """A room: 1-cell occupied walls at rows 5, 34 and cols 5, 44, free inside,
    unknown outside. Its four inward faces meet at these four corners."""
    grid = np.full((40, 50), UNK, dtype=np.uint8)
    grid[6:34, 6:44] = FREE
    grid[5, 5:45] = OCC
    grid[34, 5:45] = OCC
    grid[5:35, 5] = OCC
    grid[5:35, 44] = OCC
    return grid


ROOM_CORNERS = ((0.55, 0.55), (4.35, 0.55), (0.55, 3.35), (4.35, 3.35))


def slab_grid():
    """A 0.2 m slab (rows 10-11, cols 10-29) in a free field, unknown beyond."""
    grid = np.full((30, 50), UNK, dtype=np.uint8)
    grid[4:18, 4:36] = FREE
    grid[10:12, 10:30] = OCC
    return grid


SLAB_CORNERS = ((0.95, 0.95), (2.95, 0.95), (0.95, 1.15), (2.95, 1.15))


@pytest.fixture(scope='module')
def room():
    grid = room_grid()
    graph, el, seg_of = gw.extract_with_elements(grid, RHO, ORIGIN, P)
    return graph, el, seg_of, grid


def nearest_each(points, truth, tol):
    """Each true point has exactly one found point within `tol`."""
    for tx, ty in truth:
        near = [p for p in points if math.hypot(p[0] - tx, p[1] - ty) <= tol]
        assert len(near) == 1, ((tx, ty), points)


def test_w18_a_room_has_four_inside_corners(room):
    graph, el, seg_of, grid = room
    sch = sw.schema_of(graph, el, seg_of, grid, RHO, ORIGIN, P)
    c = sch['counts']
    assert (c['segments'], c['with_axis_ends'], c['inside_corners'],
            c['pairs'], c['gaps']) == (4, 4, 4, 0, 0)
    nearest_each([k['c'] for k in sch['inside_corners']], ROOM_CORNERS, 0.15)
    for k in sch['inside_corners']:
        assert max(k['near_m']) <= TAU_C


def test_w19_a_slab_gives_one_pair_and_two_slab_ends():
    grid = slab_grid()
    graph, el, seg_of = gw.extract_with_elements(grid, RHO, ORIGIN, P)
    sch = sw.schema_of(graph, el, seg_of, grid, RHO, ORIGIN, P)
    c = sch['counts']
    assert (c['segments'], c['inside_corners'], c['pairs'], c['slab_ends'],
            c['outside_corners'], c['gaps']) == (2, 0, 1, 2, 4, 0)
    (p,) = sch['pairs']
    assert 0.15 <= p['separation_m'] <= 0.25
    assert [e['record'] for e in p['ends']] == ['slab_end', 'slab_end']
    nearest_each([q for e in p['ends'] for q in e['outside_corners']],
                 SLAB_CORNERS, 0.05)


# ───────────────────────── W20-W22: the per-map file ─────────────────────────

STEM = 'b2maps_k0_cut1200_gated_extfix_robot0'
TRUTH_WORDS = ('world', 'truth', 'match', 'spawn', 'sdf', 'pose')


def room_record(room):
    graph, el, seg_of, grid = room
    side = {'stem': STEM, 'el': el, 'seg_of': seg_of, 'grid': grid,
            'rho': RHO, 'origin': ORIGIN,
            'graph': dict(graph, map_sha256='ab' * 32, frame='robot_0 map',
                          thresholds={'g': {'value': 0.22, 'unit': 'm',
                                            'source': 'test'}})}
    return sw.schema_for_map(side, 'dc5e2a3')


def keys_of(node):
    if isinstance(node, dict):
        for k, v in node.items():
            yield k
            yield from keys_of(v)
    elif isinstance(node, list):
        for v in node:
            yield from keys_of(v)


def test_w20_the_record(room):
    rec = room_record(room)
    assert rec['map'] == STEM and rec['map_sha256'] == 'ab' * 32
    assert rec['graph_walls_commit'] == 'dc5e2a3'
    assert rec['spec'] == 'SPEC_b2_schema v0.2, §2-§7'
    eps = P.eps(RHO)
    for name, value in (('rho', RHO), ('eps', eps), ('g', 0.22),
                        ('tau_c', eps + RHO), ('s_max', 0.5), ('tau_e', RHO)):
        assert rec['thresholds'][name] == value
    assert json.loads(json.dumps(rec)) == rec
    for key in keys_of(rec):
        assert not any(w in key for w in TRUTH_WORDS), key
    seg = rec['segments'][0]
    assert set(seg['axis_ends']) == {'t', 'q0', 'q1', 'm', 'states'}
    assert seg['axis_name'] == gw.DIRS[seg['axis']][4]


def test_w21_write_record_hashes_and_never_rewrites(room, tmp_path):
    rec = room_record(room)
    path, sha = sw.write_record(rec, tmp_path / 'run')
    assert path.name == f'{STEM}.json'
    assert sha == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads(path.read_text()) == rec
    before = path.read_bytes()
    with pytest.raises(SystemExit):
        sw.write_record(rec, tmp_path / 'run')
    assert path.read_bytes() == before


def test_w22_the_module_names_no_truth_function():
    with open(SCRIPT, encoding='utf-8') as f:
        src = f.read()
    for name in ('sdf_wall_faces', 'bag_overlap', 'resolve_spawn_poses',
                 'DOT_POSES', 'validate_map', 'face_samples', 'world_walls',
                 'knowledge_world', 'elements_in_world', '_segment_arrays',
                 'TRUTH_TOL_CELLS'):
        assert name not in src, name
