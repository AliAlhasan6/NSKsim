"""B2 face graph, §2-§4 -- experiments/analysis/edges_walls.py.

docs/specs/SPEC_b2_edges.md v0.2, §2-§4 and §4's per-map file, on hand-built
schema and graph_walls records and two small synthetic grids only: no corpus,
no map, no corpus JSON, no world SDF, no git. Truth, gate G-E1 and P-E1 to
P-E4 belong to the stage, and are not here.

THE TABLE BELOW WAS FIXED BEFORE ANY CODE EXISTED, in the build plan's §3.3
(experiments/logs/splice/edges_build_plan.md), under Ali's ruling of 18 Oct
2026 that accepted all 34 of its proposals (Q1 (a), so E10 expects 40 seen; Q7's
strict join, so E3's fourth case refuses). rho is 0.1 unless a row says
otherwise.

"The box" is four hand-built faces, exactly axis-aligned: floor 0 at y = 0 and
ceiling 1 at y = 3, both over x 0-4; left wall 2 at x = 0 and right wall 3 at
x = 4, both over y 0-3; with its four inside corners. Each face's p0-p1 runs
0.05 m past its q0 and q1, as a riser would.

  id   case                                               expected
  ───────────────────────────────────────────────────────────────────────────
  E0   the module's constants                             1e-9, 1e-12, 0.2 m, 4
  E1   D-E1: 6 segments, 1 no_axis_member, 1 no_axis      4 nodes; both excluded
                                                          with their reasons; 6
                                                          segment occluders
  E2   §2's node fields                                   n_hat from the axis;
                                                          fitted normal kept; L
                                                          = t1 - t0; m; end
                                                          states; 7 obs fields
                                                          unchanged (72, 9 bins)
  E3   the join refuses (Q7): map_sha256; an id only in   SystemExit, each
       the schema file; an id only in the graph file;
       theta on one segment
  E4   §3.1                                               4 corners, 4 `corner`
                                                          edges, i < j, attrs
                                                          copied
  E5   §3.2                                               2 pairs, 2 `slab`
                                                          edges; both ends' 4
                                                          fields
  E6   A2, C5: (19.9, 0.1); (19.9, 0.10000000149011612);  199; 198; 70; 3
       (7.0, 0.1); (0.35, 0.1)
  E7   the box                                            2 facing edges. Floor-
                                                          ceiling 40 + 40, width
                                                          4.00, gap 3.00, offset
                                                          0, bearing 0. Left-
                                                          right 30 + 30, width
                                                          3.00, gap 4.00.
                                                          Blocked, no hit, ties
                                                          0. 1 component; 2
                                                          over facing edges only
  E8   A6: the ceiling's fitted normal tilted 0.3°        still 1 edge, 40 + 40
  E9   a 1 m face 10 m opposite, fitted normal 3° away    10 of 10 see it; along
                                                          the fitted normal (a
                                                          0.52 m drift) only 5
  E10  Q1: a face tilted 0.5°, samples behind its line    40 see the face
                                                          opposite; self_first
                                                          40
  E11  C1: past an occluder's end by 2e-15, by 2e-9       crosses; does not
  E12  C2: a ray running along an occluder's line         passes
  E13  C3: two segments at equal lambda; a segment and a  the lower id wins, a
       cap at equal lambda                                tie; the segment wins
  E14  C4: 10 samples of one face                         blocked 8: same axis
                                                          3, perpendicular 2,
                                                          riser past q1 1,
                                                          excluded 2; no hit 2
  E15  A8: 3 samples, then 4                              listed below w_min;
                                                          an edge of width 0.20
  E16  C6: {1.0, 1.1, 1.2, 1.3}, then {1.0, 1.1, 1.2}     gap 1.15; gap 1.1
  E17  A7: floor 0 (x 0-4), ceiling over x 2-4, no riser  offset +1.000, gap
       extension; then the ids swapped                    3.000, bearing
                                                          18.435°; the same, i
                                                          the lower
  E18  two back-to-back faces                             a `slab` edge and no
                                                          `facing` edge
  E19  A5, E-O6: a ray inside a slab, both ends slab_end  blocked by the cap;
       then that end misaligned                           passes, sees the face
                                                          beyond
  E20  the box and one isolated face; then an excluded    2 components; still 2
       segment added
  E21  the record                                         §4's keys and Q20's;
                                                          a JSON round trip; the
                                                          counts
  E22  write_record                                       sha256 of the bytes;
                                                          refuses a second write
  E23  D-E6, the module's source                          no truth function
                                                          named
  E24  the schema tests' room grid, then a slab grid,     room (corrected after
       through extract_with_elements and schema_of,       the run, see below):
       with zeroed obs                                    4 nodes, 4 corner
                                                          edges, 2 facing edges,
                                                          37 + 37 and 24 + 24;
                                                          every sample sees or
                                                          has no hit (4 do not
                                                          see); blocked 0; 1
                                                          component. slab: 1
                                                          slab edge, 2 caps,
                                                          rays inside it blocked

E24 runs graph_walls' own extraction, so its faces carry the corner risers the
schema spec describes and are a fraction of a degree off the axis. Its room row
was corrected after Ali's run of 19 Oct 2026, under Ali's ruling of that day
(experiments/logs/splice/e24_room_diag.md); the plan had every sample of both
faces seeing its partner. The floor and the ceiling each take two of the -x
face's end cells as risers (W4; W5 puts an element in one segment only), so
that face is 2.4 m against the +x face's 2.8 m, and 4 samples of the +x face
pass its ends and hit nothing. The case now asserts that every sample sees its
partner or has no hit and that none is blocked, which holds however the risers
fall, and pins the counts the run found: 37 + 37 and 24 + 24.

The module is loaded by location, as test_schema_walls.py loads its script.
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
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'edges_walls.py')


def load_module():
    spec = importlib.util.spec_from_file_location('edges_walls', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ew = load_module()
sw, gw = ew.sw, ew.gw

RHO = 0.1
MAP_RHO = 0.10000000149011612      # the maps' rho, as their YAML gives it
ORIGIN = (0.0, 0.0)
P = gw.Params(g=0.22)
AXIS = {d[4]: k for k, d in enumerate(gw.DIRS)}
VEC = {d[4]: (float(d[2]), float(d[3])) for d in gw.DIRS}
EXT = 0.05
STEM = 'b2maps_k0_cut1200_gated_extfix_robot0'
SHA = 'ab' * 32
PROV = {'schema_file_sha256': 'cd' * 32, 'graph_walls_file_sha256': 'ef' * 32,
        'builder_commit': 'c0ffee1'}


# ─────────────────────────────── fixtures ────────────────────────────────────

def zero_obs():
    """graph_walls' obs for a segment no scan reached, as on k3 cut1200 (O2)."""
    return {'n_scans': 0, 'n_hits': 0, 'bearing_hist': [0] * 72,
            'incidence_hist': [0] * 9, 't_first': None, 't_last': None,
            'n_contradictions': 0}


def seg(sid, axis, a0, a1, at, tilt=0.0, ext=EXT, states=('free', 'free'),
        obs=None):
    """One face, as the schema file and the graph_walls file hold it.

    `axis` is a DIRS name. The axis ends span a0..a1 along the line y = at (for
    ±y) or x = at (for ±x). `tilt` turns the fitted normal that many degrees
    counter-clockwise about the midpoint, and keeps the axis ends |a1 - a0|
    apart along the fitted line. The fitted direction is (n_y, -n_x), which is
    graph_walls' dir_of(theta) for normal_of(theta) = n; t is measured along
    it. p0-p1 runs `ext` past each axis end.
    """
    nx, ny = VEC[axis]
    if axis in ('+y', '-y'):
        mid = ((a0 + a1) / 2.0, at)
    else:
        mid = (at, (a0 + a1) / 2.0)
    if tilt:
        r = math.radians(tilt)
        nx, ny = (nx * math.cos(r) - ny * math.sin(r),
                  nx * math.sin(r) + ny * math.cos(r))
    u = (ny, -nx)
    half = abs(a1 - a0) / 2.0
    tm = mid[0] * u[0] + mid[1] * u[1]
    q0 = [mid[0] - half * u[0], mid[1] - half * u[1]]
    q1 = [mid[0] + half * u[0], mid[1] + half * u[1]]
    theta = (math.degrees(math.atan2(ny, nx)) - 90.0) % 360.0
    schema = {'id': sid, 'line': sid, 'theta': theta, 'normal': [nx, ny],
              'd': q0[0] * nx + q0[1] * ny, 'axis': AXIS[axis],
              'axis_name': axis, 'n_members': 10, 'n_axis_members': 10,
              'n_risers': 0, 'n_opposite': 0, 'no_axis_ends': None,
              'axis_ends': {'t': [tm - half, tm + half], 'q0': q0, 'q1': q1,
                            'm': [(q0[0] + q1[0]) / 2.0,
                                  (q0[1] + q1[1]) / 2.0],
                            'states': list(states)}}
    graph = {'id': sid, 'p0': [q0[0] - ext * u[0], q0[1] - ext * u[1]],
             'p1': [q1[0] + ext * u[0], q1[1] + ext * u[1]],
             'theta': theta, 'normal': [nx, ny], 'length': 2 * (half + ext),
             'obs': zero_obs() if obs is None else obs}
    return schema, graph


def excluded(sid, reason, p0, p1):
    """A segment with no axis ends, `no_axis` or `no_axis_member` (D-E1)."""
    length = math.dist(p0, p1)
    u = ((p1[0] - p0[0]) / length, (p1[1] - p0[1]) / length)
    n = [-u[1], u[0]]
    theta = (math.degrees(math.atan2(n[1], n[0])) - 90.0) % 360.0
    dots = (n[0], -n[0], n[1], -n[1])               # DIRS order: +x -x +y -y
    axis = None if reason == 'no_axis' else max(range(4), key=dots.__getitem__)
    schema = {'id': sid, 'line': sid, 'theta': theta, 'normal': n,
              'd': p0[0] * n[0] + p0[1] * n[1], 'axis': axis,
              'axis_name': None if axis is None else gw.DIRS[axis][4],
              'n_members': 3, 'n_axis_members': 0, 'n_risers': 3,
              'n_opposite': 0, 'no_axis_ends': reason, 'axis_ends': None}
    graph = {'id': sid, 'p0': list(p0), 'p1': list(p1), 'theta': theta,
             'normal': n, 'length': length, 'obs': zero_obs()}
    return schema, graph


def corner(a, b, c, near_end, near_m, states=('occupied', 'occupied')):
    return {'segments': [a, b], 'c': list(c), 'near_end': list(near_end),
            'near_m': list(near_m), 'states': list(states)}


def end(points, record='slab_end', diff=0.0, state='free'):
    """One of §7's slab-end records; only a `slab_end` has outside corners."""
    pts = [list(points[0]), list(points[1])]
    return {'axis_ends': [[0, 'q0'], [1, 'q1']], 'points': pts,
            'diff_m': diff, 'aligned': record != 'misaligned', 'state': state,
            'states': ['free', 'free'], 'record': record,
            'slab_end': record == 'slab_end',
            'outside_corners': pts if record == 'slab_end' else []}


def pair(a, b, ends, sep=0.2, overlap=2.0):
    return {'segments': [a, b], 'separation_m': sep, 'overlap_m': overlap,
            'ends': list(ends)}


def records(segs, corners=(), pairs=(), rho=RHO):
    """One map's schema file and graph_walls file, through JSON as read."""
    schema = {'map': STEM, 'map_sha256': SHA, 'graph_walls_commit': 'dc5e2a3',
              'spec': sw.SPEC, 'frame': 'robot_0 map',
              'positions': 'the map frame',
              'thresholds': sw.thresholds(rho, P),
              'counts': {'segments': len(segs)},
              'segments': [s for s, _g in segs],
              'inside_corners': list(corners), 'pairs': list(pairs),
              'gaps': []}
    graph = {'map': STEM, 'map_sha256': SHA, 'frame': 'robot_0 map',
             'segments': [g for _s, g in segs]}
    return json.loads(json.dumps(schema)), json.loads(json.dumps(graph))


def box(ext=EXT):
    return [seg(0, '+y', 0.0, 4.0, 0.0, ext=ext),
            seg(1, '-y', 0.0, 4.0, 3.0, ext=ext),
            seg(2, '+x', 0.0, 3.0, 0.0, ext=ext),
            seg(3, '-x', 0.0, 3.0, 4.0, ext=ext)]


BOX_CORNERS = (corner(0, 2, (0.0, 0.0), ('q0', 'q1'), (0.01, 0.02)),
               corner(0, 3, (4.0, 0.0), ('q1', 'q0'), (0.03, 0.04)),
               corner(1, 2, (0.0, 3.0), ('q1', 'q0'), (0.05, 0.06),
                      ('occupied', 'unknown')),
               corner(1, 3, (4.0, 3.0), ('q0', 'q1'), (0.07, 0.08)))


def edge_pairs(edges):
    return [(e['i'], e['j']) for e in edges]


# ─────────────────────────── E0-E3: nodes and the join ───────────────────────

def test_e0_the_registered_constants():
    assert ew.SLACK == 1e-9 and ew.PARALLEL == 1e-12
    assert ew.W_MIN_M == 0.2 and ew.MIN_SAMPLES == 4
    # Q25: the spec's 4 is 2 w_min / rho at both values of rho.
    assert round(2 * ew.W_MIN_M / RHO) == round(2 * ew.W_MIN_M / MAP_RHO) == 4
    assert ew.OPPOSITE == {'+x': '-x', '-x': '+x', '+y': '-y', '-y': '+y'}


def test_e1_only_segments_with_axis_ends_are_nodes():
    segs = box() + [excluded(4, 'no_axis_member', (20.0, 20.0), (21.0, 20.0)),
                    excluded(5, 'no_axis', (30.0, 30.0), (31.0, 31.0))]
    G = ew.graph_of(*records(segs, BOX_CORNERS), RHO)
    assert [n['id'] for n in G['nodes']] == [0, 1, 2, 3]
    assert G['excluded'] == [{'id': 4, 'reason': 'no_axis_member'},
                             {'id': 5, 'reason': 'no_axis'}]
    assert [o['key'] for o in G['occluders']] == [[0, k] for k in range(6)]
    assert all(o['kind'] == 'segment' for o in G['occluders'])


OBS = {'n_scans': 822, 'n_hits': 114578, 'bearing_hist': list(range(72)),
       'incidence_hist': [9, 8, 7, 6, 5, 4, 3, 2, 1], 't_first': 12.5,
       't_last': 1199.75, 'n_contradictions': 3}
NODE_KEYS = {'id', 'line', 'axis', 'n_hat', 'normal', 'q0', 'q1', 'm', 't', 'L',
             'end_states', 'n_scans', 'n_hits', 'n_contradictions', 't_first',
             't_last', 'bearing_hist', 'incidence_hist'}


def test_e2_the_node_holds_section_2s_fields():
    floor = seg(0, '+y', 0.0, 4.0, 0.0, tilt=0.3,
                states=('occupied', 'unknown'), obs=OBS)
    schema, graph = records([floor] + box()[1:], BOX_CORNERS)
    nodes, _ex = ew.nodes_of(schema, graph)
    n0, s0 = nodes[0], schema['segments'][0]
    assert set(n0) == NODE_KEYS                      # Q23: §2's table only
    assert n0['id'] == 0 and n0['line'] == 0
    assert n0['axis'] == '+y' and n0['n_hat'] == [0.0, 1.0]
    assert n0['normal'] == s0['normal'] and n0['normal'] != n0['n_hat']
    for f in ('q0', 'q1', 'm', 't'):
        assert n0[f] == s0['axis_ends'][f], f
    assert n0['L'] == s0['axis_ends']['t'][1] - s0['axis_ends']['t'][0]
    assert n0['L'] == pytest.approx(4.0, abs=1e-12)
    assert n0['end_states'] == ['occupied', 'unknown']
    for f, v in OBS.items():
        assert n0[f] == v, f
    assert len(n0['bearing_hist']) == 72 and len(n0['incidence_hist']) == 9
    # O2: an empty, all-zero histogram passes through as it is.
    assert {f: nodes[1][f] for f in OBS} == zero_obs()


def _other_sha(schema, graph):
    graph['map_sha256'] = 'cd' * 32


def _only_in_schema(schema, graph):
    schema['segments'].append(
        excluded(9, 'no_axis_member', (40.0, 40.0), (41.0, 40.0))[0])


def _only_in_graph(schema, graph):
    graph['segments'].append(
        excluded(9, 'no_axis_member', (40.0, 40.0), (41.0, 40.0))[1])


def _theta_moved(schema, graph):
    graph['segments'][1]['theta'] += 1e-12


@pytest.mark.parametrize('mutate', [_other_sha, _only_in_schema,
                                    _only_in_graph, _theta_moved])
def test_e3_the_join_refuses(mutate):
    """E3, Q7: strict, bit-equal, and no tolerance."""
    schema, graph = records(box(), BOX_CORNERS)
    ew.join(schema, graph)                           # the control joins
    mutate(schema, graph)
    with pytest.raises(SystemExit):
        ew.join(schema, graph)


# ─────────────────────────── E4-E5: corner and slab ──────────────────────────

def test_e4_one_corner_edge_per_inside_corner():
    schema, graph = records(box(), BOX_CORNERS)
    G = ew.graph_of(schema, graph, RHO)
    assert edge_pairs(G['corner']) == [(0, 2), (0, 3), (1, 2), (1, 3)]
    by = {tuple(k['segments']): k for k in schema['inside_corners']}
    for e in G['corner']:
        assert e['type'] == 'corner' and e['i'] < e['j']
        k = by[(e['i'], e['j'])]
        assert e['attrs'] == {f: k[f] for f in ('c', 'near_end', 'near_m',
                                                'states')}


def test_e5_one_slab_edge_per_pair():
    segs = [seg(0, '-y', 0.0, 2.0, 0.0), seg(1, '+y', 0.0, 2.0, 0.2),
            seg(2, '-x', 0.0, 2.0, 10.0), seg(3, '+x', 0.0, 2.0, 10.2)]
    pairs = [pair(0, 1, [end([(2.0, 0.0), (2.0, 0.2)], 'slab_end', 0.01),
                         end([(0.0, 0.0), (0.0, 0.2)], 'blocked', 0.02,
                             'blocked')]),
             pair(2, 3, [end([(10.0, 0.0), (10.2, 0.0)], 'misaligned', 0.15),
                         end([(10.0, 2.0), (10.2, 2.0)], 'unobserved', 0.03,
                             'unobserved')], overlap=1.9)]
    G = ew.graph_of(*records(segs, pairs=pairs), RHO)
    assert edge_pairs(G['slab']) == [(0, 1), (2, 3)]
    assert all(e['type'] == 'slab' for e in G['slab'])
    assert G['slab'][0]['attrs'] == {
        'separation_m': 0.2, 'overlap_m': 2.0,
        'ends': [{'record': 'slab_end', 'diff_m': 0.01, 'aligned': True,
                  'state': 'free'},
                 {'record': 'blocked', 'diff_m': 0.02, 'aligned': True,
                  'state': 'blocked'}]}
    assert G['slab'][1]['attrs'] == {
        'separation_m': 0.2, 'overlap_m': 1.9,
        'ends': [{'record': 'misaligned', 'diff_m': 0.15, 'aligned': False,
                  'state': 'free'},
                 {'record': 'unobserved', 'diff_m': 0.03, 'aligned': True,
                  'state': 'unobserved'}]}
    # Q3: only the one `slab_end` carries two points, so only it occludes.
    assert [o['key'] for o in G['occluders'] if o['kind'] == 'cap'] == \
        [[1, 0, 0]]


# ──────────────────────── E6-E10: samples, rays, the box ─────────────────────

@pytest.mark.parametrize('length, rho, n', [(19.9, RHO, 199),
                                            (19.9, MAP_RHO, 198),
                                            (7.0, RHO, 70), (0.35, RHO, 3)])
def test_e6_samples_per_face(length, rho, n):
    """E6, A2 and C5: floor((L + 1e-9) / rho), no partial last cell."""
    assert ew.n_samples(length, rho) == n


def test_e7_the_box():
    G = ew.graph_of(*records(box(), BOX_CORNERS), RHO)
    facing = {(e['i'], e['j']): e['attrs'] for e in G['facing']}
    assert sorted(facing) == [(0, 1), (2, 3)]
    fc, lr = facing[(0, 1)], facing[(2, 3)]
    assert (fc['n_ij'], fc['n_ji']) == (40, 40)
    assert fc['width_m'] == pytest.approx(4.0, abs=1e-12)
    assert fc['gap_m'] == pytest.approx(3.0, abs=1e-12)
    assert fc['offset_m'] == pytest.approx(0.0, abs=1e-12)
    assert fc['bearing_deg'] == pytest.approx(0.0, abs=1e-9)
    assert (lr['n_ij'], lr['n_ji']) == (30, 30)
    assert lr['width_m'] == pytest.approx(3.0, abs=1e-12)
    assert lr['gap_m'] == pytest.approx(4.0, abs=1e-12)
    assert lr['offset_m'] == pytest.approx(0.0, abs=1e-12)
    for key, t in G['tally'].items():
        assert (t['blocked'], t['no_hit'], t['ties'], t['self_first']) == \
            (0, 0, 0, 0), key
    assert G['below_w_min'] == []
    assert G['components']['all'] == 1
    assert G['components']['facing'] == 2


def test_e8_condition_2_is_opposite_axes():
    """E8, A6: v0.1's exactly opposite normals would fail here."""
    schema, graph = records([seg(0, '+y', 0.0, 4.0, 0.0),
                             seg(1, '-y', 0.0, 4.0, 3.0, tilt=0.3)])
    n0, n1 = (s['normal'] for s in schema['segments'])
    assert abs(n0[0] + n1[0]) > ew.SLACK
    G = ew.graph_of(schema, graph, RHO)
    assert [(e['i'], e['j'], e['attrs']['n_ij'], e['attrs']['n_ji'])
            for e in G['facing']] == [(0, 1, 40, 40)]


def test_e9_rays_run_along_the_axis_not_the_fitted_normal():
    schema, graph = records([seg(0, '+y', 0.0, 1.0, 0.0, tilt=3.0),
                             seg(1, '-y', 0.0, 1.0, 10.0)])
    G = ew.graph_of(schema, graph, RHO)
    assert G['tally'][0]['samples'] == 10
    assert G['tally'][0]['seen'] == {1: 10}
    assert 10.0 * math.tan(math.radians(3.0)) == pytest.approx(0.52, abs=0.01)
    # The same samples cast along the fitted normal drift 0.52 m off the face.
    fitted = dict(G['faces'][0], n_hat=tuple(G['nodes'][0]['normal']))
    _V, t = ew.cast([fitted, G['faces'][1]], G['occluders'], RHO)
    assert t[0]['seen'] == {1: 5}


def test_e10_a_face_never_occludes_its_own_samples():
    """E10, Q1 (a): under the literal reading all 40 would be blocked."""
    schema, graph = records([seg(0, '+y', 0.0, 4.0, 0.0, tilt=0.5),
                             seg(1, '-y', 0.0, 4.0, 3.0)])
    G = ew.graph_of(schema, graph, RHO)
    t = G['tally'][0]
    assert t['samples'] == 40 and t['seen'] == {1: 40}
    assert t['self_first'] == 40 and t['blocked'] == 0
    assert G['tally'][1]['self_first'] == 0
    assert [(e['attrs']['n_ij'], e['attrs']['n_ji']) for e in G['facing']] == \
        [(40, 40)]


# ─────────────────────── E11-E14: the numeric conventions ────────────────────

@pytest.mark.parametrize('x, crosses', [(1.0 + 2e-15, True),
                                        (1.0 + 2e-9, False)])
def test_e11_an_occluder_is_closed_with_1e_9_of_slack(x, crosses):
    occ = {'key': (0, 7), 'kind': 'segment', 'p0': (0.0, 1.0),
           'p1': (1.0, 1.0)}
    got = ew.crossing((x, 0.0), (0.0, 1.0), occ)
    assert (got is not None) is crosses
    if crosses:
        assert got[0] == pytest.approx(1.0, abs=1e-12)


def test_e12_a_ray_along_an_occluders_line_passes():
    post = {'key': (0, 2), 'kind': 'segment', 'p0': (0.45, 0.5),
            'p1': (0.45, 1.5)}
    assert ew.crossing((0.45, 0.0), (0.0, 1.0), post) is None
    schema, graph = records([seg(0, '+y', 0.0, 1.0, 0.0),
                             seg(1, '-y', 0.0, 1.0, 3.0),
                             excluded(2, 'no_axis_member', (0.45, 0.5),
                                      (0.45, 1.5))])
    G = ew.graph_of(schema, graph, RHO)
    # Sample 4 of the floor stands at x = 0.45, on the post's line.
    assert G['tally'][0]['seen'] == {1: 10} and G['tally'][0]['blocked'] == 0


@pytest.mark.parametrize('other', ['segment', 'cap'])
def test_e13_a_tie_goes_to_the_lower_key(other):
    """E13, C3 and Q5: segments by id, then caps; the tie is counted."""
    segs = [seg(0, '+y', 0.0, 0.1, 0.0), seg(1, '-y', 0.0, 0.1, 1.0)]
    extra = []
    if other == 'segment':
        segs.append(seg(2, '-y', 0.0, 0.1, 1.0))
    else:
        extra = [{'key': [ew.CAP, 0, 0], 'kind': 'cap', 'p0': [-0.5, 1.0],
                  'p1': [0.5, 1.0]}]
    G = ew.graph_of(*records(segs), RHO)
    _V, t = ew.cast(G['faces'], G['occluders'] + extra, RHO)
    assert t[0]['samples'] == 1
    assert t[0]['seen'] == {1: 1} and t[0]['ties'] == 1
    assert t[0]['blocked'] == 0


def test_e14_blocked_samples_by_cause():
    """E14, C4: face 0's ten rays, x = 0.05 to 0.95, one cause each."""
    segs = [seg(0, '+y', 0.0, 1.0, 0.0),
            seg(1, '+y', 0.0, 0.3, 1.0, ext=0.02),       # its back: x <= 0.32
            seg(2, '+x', 0.85, 1.15, 0.4, tilt=30.0),    # crosses x 0.3-0.5
            excluded(3, 'no_axis_member', (0.52, 1.0), (0.68, 1.0)),
            seg(4, '-y', 1.1, 1.6, 1.0, ext=0.2)]        # its riser: x 0.9-1.1
    G = ew.graph_of(*records(segs), RHO)
    seen = []
    ew.cast(G['faces'], G['occluders'], RHO, observe=seen.append)
    assert [s['outcome'] for s in seen if s['face'] == 0] == (
        ['same_axis'] * 3 + ['perpendicular_axis'] * 2
        + ['excluded_segment'] * 2 + ['no_hit'] * 2 + ['outside_interval'])
    t = G['tally'][0]
    assert t['samples'] == 10 and t['seen'] == {}
    assert (t['blocked'], t['no_hit'], t['ties']) == (8, 2, 0)
    assert t['blocked_by'] == {'same_axis': 3, 'perpendicular_axis': 2,
                               'outside_interval': 1, 'excluded_segment': 2,
                               'cap': 0}


# ────────────────────────── E15-E17: the edge rule ───────────────────────────

@pytest.mark.parametrize('b_to, n_ji, is_edge', [(0.18, 1, False),
                                                 (0.2, 2, True)])
def test_e15_an_edge_needs_four_samples(b_to, n_ji, is_edge):
    """E15, A8: w_ij >= w_min in whole samples."""
    G = ew.graph_of(*records([seg(0, '+y', 0.0, 0.2, 0.0),
                              seg(1, '-y', 0.0, b_to, 1.0)]), RHO)
    found = G['facing'] if is_edge else G['below_w_min']
    assert (G['below_w_min'] if is_edge else G['facing']) == []
    (rec,) = found
    a = rec['attrs']
    assert (rec['i'], rec['j'], a['n_ij'], a['n_ji']) == (0, 1, 2, n_ji)
    assert a['width_m'] == pytest.approx(RHO * (2 + n_ji) / 2.0, abs=1e-12)
    if is_edge:
        assert a['width_m'] == pytest.approx(0.2, abs=1e-12)


@pytest.mark.parametrize('lij, lji, gap', [([1.0, 1.1], [1.2, 1.3], 1.15),
                                           ([1.0, 1.1], [1.2], 1.1)])
def test_e16_the_gap_is_the_median(lij, lji, gap):
    """E16, C6: an even count takes the mean of the two middle values."""
    G = ew.graph_of(*records([seg(0, '+y', 0.0, 1.0, 0.0),
                              seg(1, '-y', 0.0, 1.0, 1.0)]), RHO)
    edges, below = ew.facing_edges(G['faces'], {0: {1: lij}, 1: {0: lji}},
                                   RHO)
    (rec,) = edges + below
    assert rec['attrs']['gap_m'] == pytest.approx(gap, abs=1e-12)
    assert ew.gap_m(lij + lji) == pytest.approx(gap, abs=1e-12)


@pytest.mark.parametrize('floor_id, ceiling_id', [(0, 1), (1, 0)])
def test_e17_attributes_once_per_edge_from_the_lower_id(floor_id, ceiling_id):
    """E17, A7: the reverse direction gives the same offset and bearing."""
    segs = [seg(floor_id, '+y', 0.0, 4.0, 0.0, ext=0.0),
            seg(ceiling_id, '-y', 2.0, 4.0, 3.0, ext=0.0)]
    G = ew.graph_of(*records(segs), RHO)
    (e,) = G['facing']
    assert (e['i'], e['j']) == (0, 1)
    a = e['attrs']
    assert a['offset_m'] == pytest.approx(1.0, abs=1e-12)
    assert a['gap_m'] == pytest.approx(3.0, abs=1e-12)
    assert a['bearing_deg'] == pytest.approx(18.435, abs=5e-4)
    assert a['bearing_deg'] == pytest.approx(
        math.degrees(math.atan2(1.0, 3.0)), abs=1e-9)
    assert (a['n_ij'], a['n_ji']) == (20, 20)
    assert a['width_m'] == pytest.approx(2.0, abs=1e-12)


# ───────────────────────── E18-E20: slabs, caps, components ──────────────────

def test_e18_a_slabs_faces_get_a_slab_edge_and_no_facing_edge():
    """§3.3's note: each lies behind the other, so their rays point apart."""
    ends = [end([(2.0, 0.0), (2.0, 0.2)]), end([(0.0, 0.0), (0.0, 0.2)])]
    G = ew.graph_of(*records([seg(0, '-y', 0.0, 2.0, 0.0),
                              seg(1, '+y', 0.0, 2.0, 0.2)],
                             pairs=[pair(0, 1, ends)]), RHO)
    assert edge_pairs(G['slab']) == [(0, 1)]
    assert G['facing'] == [] and G['below_w_min'] == []
    assert all(t['seen'] == {} for t in G['tally'].values())


@pytest.mark.parametrize('near_end, n_caps, seen', [
    ('slab_end', 2, {}), ('misaligned', 1, {3: 2})])
def test_e19_a_slab_end_with_two_points_is_a_cap(near_end, n_caps, seen):
    """E19, A5 and E-O6. Face 0 stands inside a slab (y 1.0-1.2, x 2-5), and
    its two rays run along it toward the end at x = 5 and a wall at x = 6."""
    segs = [seg(0, '+x', 1.0, 1.2, 3.0),
            seg(1, '-y', 2.0, 5.0, 1.0), seg(2, '+y', 2.0, 5.0, 1.2),
            seg(3, '-x', 0.0, 2.4, 6.0)]
    ends = [end([(5.0, 1.0), (5.0, 1.2)], near_end),
            end([(2.0, 1.0), (2.0, 1.2)])]
    G = ew.graph_of(*records(segs, pairs=[pair(1, 2, ends, overlap=3.0)]),
                    RHO)
    assert len([o for o in G['occluders'] if o['kind'] == 'cap']) == n_caps
    t = G['tally'][0]
    assert t['samples'] == 2 and t['seen'] == seen
    assert t['blocked_by']['cap'] == (2 if near_end == 'slab_end' else 0)


@pytest.mark.parametrize('with_excluded', [False, True])
def test_e20_components_are_over_nodes_only(with_excluded):
    segs = box() + [seg(4, '+y', 10.0, 11.0, 10.0)]
    if with_excluded:
        segs.append(excluded(5, 'no_axis', (20.0, 20.0), (21.0, 21.0)))
    G = ew.graph_of(*records(segs, BOX_CORNERS), RHO)
    assert G['components']['all'] == 2
    assert ew.components([n['id'] for n in G['nodes']],
                         edge_pairs(G['corner'] + G['slab'] + G['facing'])) \
        == [[0, 1, 2, 3], [4]]
    assert len(G['excluded']) == (1 if with_excluded else 0)


# ──────────────────────── E21-E23: the per-map file ──────────────────────────

RECORD_KEYS = {'map', 'spec', 'frame', 'positions', 'provenance', 'thresholds',
               'nodes', 'excluded', 'edges', 'occluders', 'diagnostics',
               'counts'}


def full_map():
    """The box, a slab far from it with two caps, and an excluded segment."""
    segs = box() + [seg(4, '-y', 20.0, 22.0, 20.0),
                    seg(5, '+y', 20.0, 22.0, 20.2),
                    excluded(6, 'no_axis', (30.0, 30.0), (31.0, 31.0))]
    ends = [end([(22.0, 20.0), (22.0, 20.2)]),
            end([(20.0, 20.0), (20.0, 20.2)])]
    return records(segs, BOX_CORNERS, [pair(4, 5, ends)])


def test_e21_the_record(tmp_path):
    schema, graph = full_map()
    sp, gp = tmp_path / 'schema.json', tmp_path / 'graph.json'
    sp.write_text(json.dumps(schema, indent=2))
    gp.write_text(json.dumps(graph, indent=2))
    s2, g2, s_sha, g_sha = ew.read_inputs(sp, gp)
    assert (s2, g2) == (schema, graph)
    assert s_sha == hashlib.sha256(sp.read_bytes()).hexdigest()
    assert g_sha == hashlib.sha256(gp.read_bytes()).hexdigest()
    rec = ew.edges_for_map(s2, g2, {'schema_file_sha256': s_sha,
                                    'graph_walls_file_sha256': g_sha,
                                    'builder_commit': 'c0ffee1'})
    assert set(rec) == RECORD_KEYS
    assert rec['map'] == STEM and rec['frame'] == 'robot_0 map'
    assert rec['spec'] == 'SPEC_b2_edges v0.2, §2-§4'
    assert rec['provenance'] == {
        'map_sha256': SHA, 'schema_file_sha256': s_sha,
        'graph_walls_commit': 'dc5e2a3', 'builder_commit': 'c0ffee1',
        'graph_walls_file_sha256': g_sha}
    assert rec['thresholds'] == {'rho': RHO, 'w_min': 0.2, 'slack': 1e-9,
                                 'parallel': 1e-12, 'min_samples': 4}
    assert rec['counts'] == {'nodes': 6, 'excluded': 1, 'corner': 4,
                             'slab': 1, 'facing': 2, 'components': 2}
    assert [e['type'] for e in rec['edges']] == \
        ['corner'] * 4 + ['slab'] + ['facing'] * 2          # Q31
    assert rec['excluded'] == [{'id': 6, 'reason': 'no_axis'}]
    assert [o['kind'] for o in rec['occluders']] == \
        ['segment'] * 7 + ['cap'] * 2
    d = rec['diagnostics']
    assert set(d) == {'per_face', 'below_w_min', 'components'}
    assert [r['id'] for r in d['per_face']] == [0, 1, 2, 3, 4, 5]
    assert d['per_face'][0]['seen'] == [[1, 40]]             # Q32
    assert d['components'] == {'facing': 4, 'facing_and_slab': 3}
    assert json.loads(json.dumps(rec)) == rec
    with pytest.raises(SystemExit):                  # provenance incomplete
        ew.edges_for_map(s2, g2, {'schema_file_sha256': s_sha})


def test_e22_write_record_hashes_and_never_rewrites(tmp_path):
    rec = ew.edges_for_map(*full_map(), PROV)
    path, sha = ew.write_record(rec, tmp_path / 'run')
    assert path.name == f'{STEM}.json'
    assert sha == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads(path.read_text()) == rec
    before = path.read_bytes()
    with pytest.raises(SystemExit):
        ew.write_record(rec, tmp_path / 'run')
    assert path.read_bytes() == before


def test_e23_the_module_names_no_truth_function():
    with open(SCRIPT, encoding='utf-8') as f:
        src = f.read()
    for name in ('sdf_wall_faces', 'bag_overlap', 'resolve_spawn_poses',
                 'DOT_POSES', 'validate_map', 'face_samples', 'world_walls',
                 'knowledge_world', 'elements_in_world', '_segment_arrays',
                 'TRUTH_TOL_CELLS'):
        assert name not in src, name


# ──────────────────── E24: through graph_walls and schema_walls ──────────────

def room_grid():
    """test_schema_walls' room: 1-cell walls at rows 5, 34 and cols 5, 44."""
    grid = np.full((40, 50), gw.UNKNOWN, dtype=np.uint8)
    grid[6:34, 6:44] = gw.FREE
    grid[5, 5:45] = gw.OCC
    grid[34, 5:45] = gw.OCC
    grid[5:35, 5] = gw.OCC
    grid[5:35, 44] = gw.OCC
    return grid


def slab_grid():
    """test_schema_walls' 0.2 m slab, rows 10-11, cols 10-29, in free space."""
    grid = np.full((30, 50), gw.UNKNOWN, dtype=np.uint8)
    grid[4:18, 4:36] = gw.FREE
    grid[10:12, 10:30] = gw.OCC
    return grid


def through_extraction(grid):
    """Both files for one grid: schema_for_map's record, and graph_walls'
    segments with zeroed obs, each through JSON."""
    graph, el, seg_of = gw.extract_with_elements(grid, RHO, ORIGIN, P)
    side = {'stem': STEM, 'el': el, 'seg_of': seg_of, 'grid': grid,
            'rho': RHO, 'origin': ORIGIN,
            'graph': dict(graph, map_sha256=SHA, frame='robot_0 map',
                          thresholds={'g': {'value': 0.22, 'unit': 'm',
                                            'source': 'test'}})}
    schema = sw.schema_for_map(side, 'dc5e2a3')
    walls = {'map': STEM, 'map_sha256': SHA, 'frame': 'robot_0 map',
             'segments': [{'id': s['id'], 'p0': s['p0'], 'p1': s['p1'],
                           'theta': s['theta'], 'normal': s['normal'],
                           'obs': zero_obs()} for s in graph['segments']]}
    return json.loads(json.dumps(schema)), json.loads(json.dumps(walls))


@pytest.mark.parametrize('grid', ['room', 'slab'])
def test_e24_through_the_extraction(grid):
    if grid == 'room':
        G = ew.graph_of(*through_extraction(room_grid()), RHO)
        assert len(G['nodes']) == 4 and G['excluded'] == []
        assert len(G['corner']) == 4 and G['slab'] == []
        axis = {n['id']: n['axis'] for n in G['nodes']}
        assert sorted(tuple(sorted((axis[e['i']], axis[e['j']])))
                      for e in G['facing']) == [('+x', '-x'), ('+y', '-y')]
        # Every sample either sees its partner or has no hit; none is blocked.
        for t in G['tally'].values():
            assert t['blocked'] == 0
            assert sum(t['seen'].values()) + t['no_hit'] == t['samples']
        # Found on Ali's run of 19 Oct 2026, not fixed before it
        # (e24_room_diag.md): the floor and ceiling each take two of the -x
        # face's end cells as risers (W4, W5), so that face is 2.4 m, and 4
        # samples of the +x face pass its ends and hit nothing.
        n = {tuple(sorted((axis[e['i']], axis[e['j']]))):
             (e['attrs']['n_ij'], e['attrs']['n_ji']) for e in G['facing']}
        assert n == {('+y', '-y'): (37, 37), ('+x', '-x'): (24, 24)}
        assert sorted(t['no_hit'] for t in G['tally'].values()) == [0, 0, 0, 4]
        for e in G['facing']:
            a = e['attrs']
            assert a['width_m'] == pytest.approx(
                RHO * (a['n_ij'] + a['n_ji']) / 2.0, abs=1e-12)
        assert G['components']['all'] == 1
    else:
        G = ew.graph_of(*through_extraction(slab_grid()), RHO)
        assert len(G['nodes']) == 2 and len(G['slab']) == 1
        assert G['facing'] == []
        assert len([o for o in G['occluders'] if o['kind'] == 'cap']) == 2
        # A probe face west of the slab, at x = 0.5 over its thickness: its
        # two rays, at y = 1.10 and 1.00, run inside the slab once they enter.
        probe = ew.face_of({'id': 99, 'axis': '+x', 'n_hat': [1.0, 0.0],
                            'normal': [1.0, 0.0], 'q0': [0.5, 1.15],
                            'm': [0.5, 1.05], 't': [-1.15, -0.95], 'L': 0.2})
        _V, t = ew.cast(G['faces'] + [probe], G['occluders'], RHO)
        assert t[99]['samples'] == 2
        assert t[99]['blocked_by']['cap'] == 2 and t[99]['seen'] == {}
