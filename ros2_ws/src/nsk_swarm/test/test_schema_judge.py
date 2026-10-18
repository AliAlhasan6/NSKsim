"""The B2 schema stage -- experiments/analysis/schema_judge.py.

docs/specs/SPEC_b2_schema.md v0.2, §9 (matching to truth) and §10 (G-S1 and
P-S1 to P-S8). The registered text of the real spec is read, because the stage's
whole claim is that it applies the text that was registered; everything else runs
on hand-built records and small synthetic grids. No corpus map, no graph JSON, no
bag and no gitignored scratch file is opened: the 800-extraction gate and the 20
maps are the run's job, and the run is not here.

THE TABLE BELOW WAS WRITTEN BEFORE THE CODE WAS RUN. Each row is what the case
must produce, fixed in advance.

  id   case                                           expected
  ───────────────────────────────────────────────────────────────────────────
  Y0   the registered text, from the real spec        §9, §10 and the truth
                                                      block; D-S5 to D-S7; the
                                                      item each number is in
  Y1   the numbers found in it                        rho, eps, g, tau_c,
                                                      s_max, tau_e, tau_m,
                                                      tau_end, the shift, 1e-9
                                                      and the eight bounds
  Y2   the A9 path broken across two lines            the file name alone is
                                                      read, and its sha256
  Y3   P-S1's item naming P-S2                        refuses
  Y4   ten mutations of the registered numbers        each raises SystemExit
  Y5   a name added to or dropped from the manifest   refuses both ways
  Y6   §1-§10 at HEAD edited (ruling 4)               refuses
  Y7   truth(): 12 faces, 4 inside corners, 4 slabs,  as §9, §7 and §1 state
       8 slab ends, 16 outside corners, 6.0 m apart   them
  Y8   the decoy shift                                moves every face, corner
                                                      and slab end by +0.5
  Y9   M1 on a hand-built record                      the face whose normal,
                                                      line and extent fit
  Y10  m(S) at tau_m, then 0.001 m past it            matched, then unmatched
  Y11  an interval off the exposed extent             unmatched, and counted
  Y12  two faces qualifying                           the nearer line
  Y13  M2 true, wrong, unmatched-part                 one of each
  Y14  M2 twice for one true corner                   a duplicate, not a find
  Y15  M3 on a slab, then across two boxes            true, then wrong
  Y16  M4 at tau_end, then past it                    true, then wrong
  Y17  M4 under an unmatched-part pair                unmatched-part
  Y18  M4's outside corners                           two, with their distance
                                                      to the box corner
  Y19  M5                                             every opening spurious,
                                                      with its two faces
  Y20  M6 on a missed corner with both faces matched  n_faces_matched 2 and the
                                                      nearest axis end per face
  Y21  M6 per face (S-O2)                             segments and lines[]
                                                      entries per face
  Y22  check 1 failing                                exit 2; no A9, no per-map
                                                      file, no match, no
                                                      verdict
  Y23  check 2 one ulp out                            exit 2 before any per-map
                                                      file is written
  Y24  check 3 failing                                exit 2 after the files,
                                                      with no verdict
  Y25  check 3 itself                                 0 matches on the real
                                                      map's record, 1 on a
                                                      record laid on the decoy
  Y26  the door                                       raises until all three
                                                      checks hold
  Y27  the verdicts in integers                       P-S1 at 18 and at 17;
                                                      P-S2 at 1 map and at 2
  Y28  P-S6 and P-S1 are judged at cut1200            an opening at cut60 does
                                                      not count; a wrong corner
                                                      there does
  Y29  the refusals, before anything is loaded        a dirty tree; a
                                                      registration that is no
                                                      ancestor; a spec without
                                                      the gate; a missing or
                                                      wrong pinned file
  Y30  §8's per-map files                             written, read back,
                                                      hashed, never rewritten
  Y31  the 160-grid builder, (small, base)            the pinned block, every
                                                      field
  Y32  the pinned outputs, parsed                     one block each; an
                                                      unreadable line refuses
  Y33  a fail payload with a verdict planted in it    the walk catches it

Y31 runs 16 extractions of the pinned corpus's small world (about 3 s) and is
the only slow case. The counts it checks are quoted from
`schema_v0_1_synth2_out.txt`, sha256 `364def1b…`, which G-S1 check 1 reads in
full on the run; here they are a literal, so CI pins the restated builder
without the gitignored file.

The module is loaded by location, as test_divergence_walls.py loads its scripts.
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
ANALYSIS = os.path.join(REPO_ROOT, 'experiments', 'analysis')
SPEC_PATH = os.path.join(REPO_ROOT, 'docs', 'specs', 'SPEC_b2_schema.md')


def load_module(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ANALYSIS, f'{name}.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sj = load_module('schema_judge')
gw, sw, dw = sj.gw, sj.sw, sj.dw

RHO = 0.10000000149011612          # the maps' rho, as §9's Margin item gives it
P = gw.Params(g=0.22)
TOL = {'tau_m': 2 * RHO, 'tau_end': 2.5 * RHO}
AXIS = {d[4]: k for k, d in enumerate(gw.DIRS)}
FULL = dw.FULL_CUT


def spec_text():
    with open(SPEC_PATH, encoding='utf-8') as f:
        return f.read()


@pytest.fixture(scope='module')
def verbatim():
    return sj.schema_verbatim(spec_text())


@pytest.fixture(scope='module')
def constants(verbatim):
    return sj.schema_constants_from(verbatim)


@pytest.fixture(scope='module')
def world():
    return gw.sdf_wall_faces(gw.L_MIN)


@pytest.fixture(scope='module')
def truth(world):
    return sj.truth(world)


# ─────────────────── Y0-Y6: the registered text and its numbers ──────────────

def test_y0_the_registered_text_is_read_from_the_spec(verbatim):
    """Y0: §9, §10, the truth block and the items, by script and not retyped."""
    b, it = verbatim['blocks'], verbatim['items']
    assert b['9'].startswith('## 9. Matching to truth (Def S7, S-O3)')
    assert '### M6, missed, and why' in b['9']
    assert '## 10.' not in b['9']
    assert b['10'].startswith('## 10. Predictions (S-O4)')
    assert '### Gate G-S1' in b['10'] and '## 11.' not in b['10']
    assert b['truth'].startswith('- **Truth (§1),**')
    assert b['truth'].rstrip().endswith('−y faces alike).')
    assert it['D-S5'].startswith('**D-S5, truth after the fact** (confirmed by '
                                 'Ali, 17 Oct 2026).')
    assert it['D-S6'].endswith('No element is matched by a radius around a '
                               'point.')
    assert it['D-S7'].startswith('**D-S7, rows**')
    assert it['P-S1'] == ('**P-S1, inside corners found.** On the five cut1200 '
                          'maps, at least 18 of the 20 true inside corners are '
                          'found.')
    assert it['gate_3_decoy'].startswith('**X-S1, a decoy truth.** Against the '
                                         'world shifted by (+0.5, +0.5) m')
    # One item at a time: a paragraph read here would run into its neighbour.
    assert 'P-S2' not in it['P-S1'] and 'Frame and truth' not in \
        it['gate_1_synthetic']


def test_y1_every_number_the_stage_applies(constants):
    """Y1: the numbers of §10's manifest, each from the registered text."""
    assert constants['rho']['maps'] == RHO
    assert constants['rho']['grids'] == 0.1
    assert constants['rho']['largest_m_perp_m'] == 0.19098039120664012
    assert constants['rho']['tau_m_margin_m'] == 0.0090196
    assert constants['tau_c'] == {'value_m': 0.2707, 'formula': 'eps + rho'}
    assert constants['eps']['eps_plus_half_rho_m'] == 0.2207
    assert constants['g']['value_m'] == 0.22
    assert constants['s_max']['value_m'] == 0.5 == sw.S_MAX_M
    assert constants['tau_e']['value_m'] == 0.1
    assert constants['tau_m'] == {'value_m': 0.2, 'formula': '2*rho',
                                  'cells': 2.0}
    assert constants['tau_end']['value_m'] == 0.25
    assert constants['tau_end']['cells'] == 2.5
    assert constants['decoy shift']['m'] == (0.5, 0.5)
    assert constants['1e-9'] == {'axis_tie': 1e-9, 'gate': 1e-9,
                                 'gate_maps': (20, 20)}
    assert sw.AXIS_TIE == 1e-9 and gw.TRUTH_TOL_CELLS == 2.0
    assert constants['spawn_rev'] == '08617b2'
    assert constants['truth_counts'] == {
        'faces': 12, 'inside_corners': 4, 'slabs': 4, 'slab_ends': 8,
        'outside_corners': 16, 'min_same_facing_gap_m': 6.0}
    assert constants['rows'] == {
        'n': 20, 'pattern': 'b2maps_k{K}_cut{C}_gated_extfix_robot{K}',
        'k': (0, 4), 'cuts': (60, 120, 240, 1200)}
    assert constants['check1'] == {
        'md': sj.CHECK_MD, 'n_160': 160, 'd2_out': sj.MATCH_D2_OUT,
        'n_640': 640, 'corners_640': (640, 640)}
    assert constants['predictions']['p-s1'] == {
        'rule': 'found', 'kind': 'inside corners', 'min': 18, 'of': 20,
        'rows': 'cut1200'}
    assert constants['predictions']['p-s5']['min'] == 36
    assert constants['predictions']['p-s5']['of'] == 40
    assert constants['predictions']['p-s6'] == {
        'rule': 'maps', 'what': 'there is no opening', 'min': 4, 'of': 5,
        'at_most': 1, 'rows': 'cut1200'}
    for name in ('p-s2', 'p-s4', 'p-s7', 'p-s8'):
        assert constants['predictions'][name]['at_most'] == 1
        assert constants['predictions'][name]['of'] == 20


def test_y2_the_a9_path_breaks_across_two_lines(constants, verbatim):
    """Y2: unwrapped the path reads "graph_walls/ _validation_..."; the stage
    matches on the file name alone."""
    item = ' '.join(verbatim['items']['tau_m'].split())
    assert 'graph_walls/ _validation_' in item
    assert constants['files']['a9_file'] == sj.A9_FILE
    assert sj.A9_SHA256.startswith(constants['files']['a9_sha256_prefix'])
    assert constants['files']['sdf_sha256_prefix'] == 'f5fea67f'


def test_y3_a_prediction_item_naming_another_refuses():
    old = 'true inside corners are found.'
    assert spec_text().count(old) == 1
    text = spec_text().replace(old, 'true inside corners are found, as P-S2 '
                                    'says.')
    with pytest.raises(SystemExit):
        sj.schema_constants_from(sj.schema_verbatim(text))


def y4_apply(text, world, sides):
    """Every place a found number is checked, in the stage's own order.

    The finder checks the text against itself and against the code's own
    constants; `map_thresholds` checks the values that depend on rho against
    all 20 maps; `check_truth` and `check_spawn_rev` check the world and the
    frame.
    """
    constants = sj.schema_constants_from(sj.schema_verbatim(text))
    sj.map_thresholds(sides, constants)
    sj.check_truth(sj.truth(world), constants)
    sj.check_spawn_rev(constants)
    return constants


Y4_MUTATIONS = (
    ('`tau_c = eps + rho` (0.2707 m)', '`tau_c = eps + rho` (0.2807 m)'),
    ('at least 18 of the 20 true slabs', 'at least 17 of the 20 true slabs'),
    ('shifted by (+0.5, +0.5) m, no segment',
     'shifted by (+0.4, +0.5) m, no segment'),
    ('`tau_m = 2*rho` (0.2 m)', '`tau_m = 2*rho` (0.3 m)'),
    ('`tau_end = tau_m + rho/2` (0.25 m)',
     '`tau_end = tau_m + rho/2` (0.30 m)'),
    ('`s_max = 0.5 m`', '`s_max = 0.6 m`'),
    ('differ by at most 1e-9, S has no axis',
     'differ by at most 1e-8, S has no axis'),
    ('P-S1 and P-S3 hold when found ≥ 18', 'P-S1 and P-S3 hold when found ≥ 19'),
    ('s_K from `DOT_POSES` at `08617b2`', 's_K from `DOT_POSES` at `fc050c4`'),
    ('the 12 long observable world faces', 'the 16 long observable world faces'),
    ('Same-facing long faces are at least 6.0 m apart',
     'Same-facing long faces are at least 7.0 m apart'),
)


@pytest.mark.parametrize('old, new', Y4_MUTATIONS)
def test_y4_a_changed_number_refuses(old, new, world, room):
    """Y4: S13's Z22 test, here over every number the manifest names."""
    text = spec_text()
    assert text.count(old) == 1, old
    with pytest.raises(SystemExit):
        y4_apply(text.replace(old, new, 1), world, y_sides(room))


def test_y4_the_unmutated_text_passes_every_check(world, room):
    """Y4's control: the registered text itself raises nothing."""
    c = y4_apply(spec_text(), world, y_sides(room))
    assert c['spawn_rev'] == '08617b2'
    assert c['truth_counts']['faces'] == 12


@pytest.mark.parametrize('old, new', [
    ('`tau_end`, the decoy\'s shift', '`tau_end`, `l_min`, the decoy\'s shift'),
    ('`tau_m`, `tau_end`, the decoy', '`tau_m`, the decoy'),
    ('36 of 40 for', '35 of 40 for'),
    ("the decoy's shift, 1e-9,", "the decoy's shift,"),
    ('18 of 20 for P-S1 and P-S3', '18 of 20 for P-S1'),
])
def test_y5_the_manifest_must_name_exactly_the_finders(old, new):
    """Y5: a number in the list with no finder, and a finder not in the list."""
    text = spec_text()
    assert text.count(old) == 1, old
    with pytest.raises(SystemExit):
        sj.schema_constants_from(sj.schema_verbatim(text.replace(old, new, 1)))


def test_y6_the_registered_body_is_sections_1_to_10():
    """Y6, ruling 4: §1-§10 at HEAD against the registration, byte for byte."""
    text = spec_text()
    body = sj.registered_body(text)
    assert body.startswith('## 1. The world')
    assert '## 10. Predictions (S-O4)' in body
    assert '## 11.' not in body and '## 0. Decisions' not in body
    edited = text.replace('the 4 inside corners, each with its two boundary',
                          'the 5 inside corners, each with its two boundary')
    assert sj.registered_body(edited) != body
    # §11 and §0 are outside the compared body, so editing them does not refuse.
    outside = text.replace('- **S-O1.** Closed in v0.1.',
                           '- **S-O1.** Closed in v0.1, and noted again.')
    assert sj.registered_body(outside) == body
    with pytest.raises(SystemExit):
        sj.registered_body('no sections here')


# ──────────────────────── Y7-Y8: §9's truth, and the decoy ───────────────────

def test_y7_truth_is_what_section_9_describes(truth, constants):
    """Y7: the 12 long faces, the 4 corners, the 4 slabs and the 16 corners."""
    sj.check_truth(truth, constants)
    assert truth['counts'] == {'faces': 12, 'inside_corners': 4, 'slabs': 4,
                               'slab_ends': 8, 'outside_corners': 16}
    assert sorted(truth['faces']) == [
        'E', 'N', 'S', 'W', 'maze_h1+y', 'maze_h1-y', 'maze_h2+y', 'maze_h2-y',
        'maze_v1+x', 'maze_v1-x', 'maze_v2+x', 'maze_v2-x']
    n = truth['faces']['N']
    assert (n['v'], n['lo'], n['hi']) == (9.95, -9.95, 9.95)   # 19.9 m exposed
    assert n['length_m'] == pytest.approx(19.9, abs=1e-12)
    assert n['axis'] == AXIS['-y'] and n['source'] == 'wall_north.south'
    h1 = truth['faces']['maze_h1-y']
    assert (h1['v'], h1['lo'], h1['hi']) == (2.9, -6.0, 1.0)
    assert {k['name']: tuple(k['p']) for k in truth['corners']} == {
        'WS': (-9.95, -9.95), 'WN': (-9.95, 9.95), 'ES': (9.95, -9.95),
        'EN': (9.95, 9.95)}
    assert {k['name']: tuple(k['faces']) for k in truth['corners']}['EN'] == \
        ('E', 'N')
    by = {s['name']: s for s in truth['slabs']}
    assert sorted(by) == ['maze_h1', 'maze_h2', 'maze_v1', 'maze_v2']
    assert by['maze_h1']['horiz'] and by['maze_h1']['ends'] == (-6.0, 1.0)
    assert by['maze_h1']['faces'] == ('maze_h1-y', 'maze_h1+y')
    assert by['maze_h1']['outside'][0] == [[-6.0, 2.9], [-6.0, 3.1]]
    assert not by['maze_v1']['horiz'] and by['maze_v1']['ends'] == (-4.0, 1.0)
    assert truth['min_same_facing_gap'] == (6.0, 'maze_h1+y', 'maze_h2+y')
    assert sum(len(s['outside'][e]) for s in truth['slabs']
               for e in (0, 1)) == 16


def test_y8_the_decoy_moves_the_whole_world(world, truth):
    """Y8: X-S1's truth is the world shifted by (+0.5, +0.5) m, all of it."""
    decoy = sj.truth(world, (0.5, 0.5))
    assert decoy['shift'] == [0.5, 0.5]
    assert decoy['counts'] == truth['counts']
    for name, f in decoy['faces'].items():
        g = truth['faces'][name]
        assert f['axis'] == g['axis']
        assert f['v'] == pytest.approx(g['v'] + 0.5, abs=1e-12)
        assert f['lo'] == pytest.approx(g['lo'] + 0.5, abs=1e-12)
    for k, g in zip(decoy['corners'], truth['corners']):
        assert k['p'] == pytest.approx([g['p'][0] + 0.5, g['p'][1] + 0.5],
                                       abs=1e-12)
    for s, t in zip(decoy['slabs'], truth['slabs']):
        assert s['ends'] == pytest.approx([t['ends'][0] + 0.5,
                                           t['ends'][1] + 0.5], abs=1e-12)


# ───────────── hand-built per-map records, as §8's files hold them ───────────

def seg(sid, axis, q0, q1, line=0, states=('free', 'free')):
    """One segment's §2-§3 record, with axis ends, as schema_walls writes it."""
    return {'id': sid, 'line': line, 'theta': 0.0,
            'normal': [float(v) for v in sw.AX[AXIS[axis]]], 'd': 0.0,
            'axis': AXIS[axis], 'axis_name': axis, 'n_members': 2,
            'n_axis_members': 2, 'n_risers': 0, 'n_opposite': 0,
            'no_axis_ends': None,
            'axis_ends': {'t': [0.0, 1.0], 'q0': list(q0), 'q1': list(q1),
                          'm': [(q0[0] + q1[0]) / 2.0, (q0[1] + q1[1]) / 2.0],
                          'states': list(states)}}


def no_ends(sid, line=0):
    """A segment with no axis ends: §2's `no_axis_member`."""
    s = seg(sid, '+y', [0.0, 0.0], [0.0, 0.0], line)
    s.update(axis=None, axis_name=None, axis_ends=None,
             no_axis_ends='no_axis_member')
    return s


def corner(a, b, c, states=('occupied', 'occupied')):
    return {'segments': [a, b], 'c': list(c), 'near_end': ['q1', 'q1'],
            'near_m': [0.01, 0.02], 'states': list(states)}


def end(a, b, pa, pb, record='slab_end'):
    return {'axis_ends': [[a, 'q0'], [b, 'q1']], 'points': [list(pa),
                                                            list(pb)],
            'diff_m': 0.0, 'aligned': record != 'misaligned',
            'state': 'free' if record in ('slab_end', 'misaligned') else record,
            'states': ['free', 'free'], 'record': record,
            'slab_end': record == 'slab_end',
            'outside_corners': [list(pa), list(pb)]
                               if record == 'slab_end' else []}


def pair(a, b, ends=(), sep=0.2):
    return {'segments': [a, b], 'separation_m': sep, 'overlap_m': 1.0,
            'ends': list(ends)}


def gap(a, b, width=0.3, state='free', opening=True):
    return {'line': 0, 'segments': [a, b], 't0': 0.0, 't1': width,
            'width_m': width, 'state': state, 'opening': opening}


def record(segments, corners=(), pairs=(), gaps=()):
    """A per-map record, through JSON as §9 reads it from the file (D-S5)."""
    rec = {'map': 'synthetic', 'map_sha256': 'ab' * 32,
           'graph_walls_commit': 'dc5e2a3', 'spec': sw.SPEC,
           'frame': 'robot_0 map', 'positions': 'the map frame',
           'thresholds': sw.thresholds(RHO, P),
           'counts': {'segments': len(segments)},
           'segments': list(segments), 'inside_corners': list(corners),
           'pairs': list(pairs), 'gaps': list(gaps)}
    return json.loads(json.dumps(rec))


# A segment on each face the cases use, in the world frame (shift (0, 0)).
ON_N = seg(0, '-y', [0.0, 9.95], [2.0, 9.95])
ON_E = seg(1, '-x', [9.95, 0.0], [9.95, 2.0], line=1)
H1_LOW = seg(2, '-y', [-4.0, 2.9], [0.0, 2.9], line=2)
H1_HIGH = seg(3, '+y', [-4.0, 3.1], [0.0, 3.1], line=3)
H2_LOW = seg(4, '-y', [0.0, -3.1], [4.0, -3.1], line=4)
ZERO = (0.0, 0.0)


# ───────────────────────────── Y9-Y12: M1 ────────────────────────────────────

def test_y9_m1_matches_the_face_that_fits(truth):
    fm = sj.match_faces(record([ON_N, ON_E, H1_LOW]), truth, ZERO, TOL['tau_m'])
    assert fm == {0: 'N', 1: 'E', 2: 'maze_h1-y'}


def test_y9_m1_skips_a_segment_without_axis_ends(truth):
    fm = sj.match_faces(record([ON_N, no_ends(9)]), truth, ZERO, TOL['tau_m'])
    assert fm == {0: 'N'}            # §2: it is in no element, and not matched


def test_y9_m1_carries_the_record_by_m0(truth):
    """M0: a point in robot K's frame lies at p + s_K in the world."""
    shifted = seg(0, '-y', [-0.86, 9.67], [1.14, 9.67])
    assert sj.match_faces(record([shifted]), truth, ZERO, TOL['tau_m']) == \
        {0: None}
    assert sj.match_faces(record([shifted]), truth, (0.86, 0.28),
                          TOL['tau_m']) == {0: 'N'}


@pytest.mark.parametrize('dy, face', [(0.0, 'N'), (-0.2, 'N'), (0.2, 'N'),
                                      (-0.201, None), (0.201, None)])
def test_y10_m1_holds_m_within_tau_m_of_the_line(truth, dy, face):
    """Y10: tau_m is on the midpoint, inclusive."""
    s = seg(0, '-y', [0.0, 9.95 + dy], [2.0, 9.95 + dy])
    assert sj.match_faces(record([s]), truth, ZERO, 0.2) == {0: face}


def test_y11_m1_needs_the_extent_to_overlap(truth):
    """Y11: past the exposed 19.9 m of a boundary face, nothing matches."""
    inside = seg(0, '-y', [9.0, 9.95], [9.94, 9.95])
    outside = seg(0, '-y', [9.96, 9.95], [10.05, 9.95])
    assert sj.match_faces(record([inside]), truth, ZERO, 0.2) == {0: 'N'}
    assert sj.match_faces(record([outside]), truth, ZERO, 0.2) == {0: None}
    j = sj.judge_map(record([outside]), truth, ZERO, TOL)
    assert j['unmatched_segments'] == 1


def test_y12_m1_takes_the_nearest_line(truth):
    """Y12: the slab's two faces are 0.2 m apart, so both can qualify."""
    near_low = seg(0, '-y', [-2.0, 2.95], [-1.0, 2.95])
    assert sj.match_faces(record([near_low]), truth, ZERO, 0.2) == \
        {0: 'maze_h1-y'}
    # 3.1 faces +y, so only maze_h1+y can match a +y segment there.
    high = seg(0, '+y', [-2.0, 3.05], [-1.0, 3.05])
    assert sj.match_faces(record([high]), truth, ZERO, 0.2) == \
        {0: 'maze_h1+y'}


# ──────────────────────────── Y13-Y21: M2-M6 ─────────────────────────────────

def test_y13_m2_true_wrong_and_unmatched_part(truth):
    true = sj.judge_map(record([ON_N, ON_E], [corner(0, 1, (9.95, 9.95))]),
                        truth, ZERO, TOL)['inside_corners']
    assert list(true['found']) == ['EN']
    assert true['found']['EN']['c_to_corner_m'] == pytest.approx(0.0)
    assert true['found']['EN']['states'] == ['occupied', 'occupied']
    assert (true['wrong'], true['unmatched_part']) == ([], [])
    assert len(true['missed']) == 3

    wrong = sj.judge_map(record([ON_N, H1_LOW], [corner(0, 2, (0.0, 9.95))]),
                         truth, ZERO, TOL)['inside_corners']
    assert wrong['found'] == {} and len(wrong['wrong']) == 1
    assert wrong['wrong'][0]['faces'] == ['N', 'maze_h1-y']
    assert wrong['unmatched_part'] == [] and len(wrong['missed']) == 4

    off = seg(5, '-x', [20.0, 0.0], [20.0, 2.0], line=1)
    part = sj.judge_map(record([ON_N, off], [corner(0, 5, (20.0, 9.95))]),
                        truth, ZERO, TOL)['inside_corners']
    assert part['wrong'] == [] and len(part['unmatched_part']) == 1
    assert part['unmatched_part'][0]['faces'] == ['N', None]


def test_y14_m2_counts_a_second_corner_as_a_duplicate(truth):
    also = seg(6, '-x', [9.95, 3.0], [9.95, 5.0], line=6)
    j = sj.judge_map(record([ON_N, ON_E, also],
                            [corner(0, 1, (9.95, 9.95)),
                             corner(0, 6, (9.95, 9.95))]), truth, ZERO, TOL)
    c = j['inside_corners']
    assert list(c['found']) == ['EN'] and len(c['duplicates']) == 1
    assert c['wrong'] == [] and c['unmatched_part'] == []
    assert sj.map_counts(j)['inside_corners'] == {
        'found': 1, 'missed': 3, 'wrong': 0, 'unmatched_part': 0,
        'duplicates': 1}


def test_y15_m3_a_slab_then_a_pair_across_two_boxes(truth):
    good = sj.judge_map(record([H1_LOW, H1_HIGH], pairs=[pair(2, 3)]), truth,
                        ZERO, TOL)['slabs']
    assert list(good['found']) == ['maze_h1']
    assert good['wrong'] == [] and good['unmatched_part'] == []
    assert len(good['missed']) == 3

    across = sj.judge_map(record([H1_LOW, H2_LOW], pairs=[pair(2, 4)]), truth,
                          ZERO, TOL)['slabs']
    assert across['found'] == {} and len(across['wrong']) == 1
    assert across['wrong'][0]['faces'] == ['maze_h1-y', 'maze_h2-y']


@pytest.mark.parametrize('x, kind', [(-6.0, 'found'), (-6.25, 'found'),
                                     (-6.26, 'wrong'), (-5.0, 'wrong')])
def test_y16_m4_is_true_within_tau_end_of_the_end(truth, x, kind):
    """Y16: the mean of the end's two axis ends, along the slab's long axis."""
    e = end(2, 3, [x, 2.9], [x, 3.1])
    j = sj.judge_map(record([H1_LOW, H1_HIGH], pairs=[pair(2, 3, [e])]), truth,
                     ZERO, TOL)
    ends = j['slab_ends']
    if kind == 'found':
        assert len(ends['found']) == 1 and ends['wrong'] == []
        rec = list(ends['found'].values())[0]
        assert rec['slab'] == 'maze_h1' and rec['slab_end'] == 0
        assert rec['along_error_m'] == pytest.approx(abs(x + 6.0), abs=1e-12)
    else:
        assert ends['found'] == {} and len(ends['wrong']) == 1
        assert ends['wrong'][0]['reason'] == ('the pair is true and the end is '
                                              'beyond tau_end')
    assert len(ends['missed']) == 8 - len(ends['found'])
    assert ends['missed'][0]['slab_has_a_true_pair'] is True


@pytest.mark.parametrize('record_, counted', [('slab_end', True),
                                              ('misaligned', False),
                                              ('blocked', False),
                                              ('unobserved', False)])
def test_y16_only_a_slab_end_is_a_finding(truth, record_, counted):
    """§7: an end that is misaligned, blocked or unobserved is no finding."""
    e = end(2, 3, [-6.0, 2.9], [-6.0, 3.1], record_)
    j = sj.judge_map(record([H1_LOW, H1_HIGH], pairs=[pair(2, 3, [e])]), truth,
                     ZERO, TOL)
    assert len(j['slab_ends']['found']) == (1 if counted else 0)
    assert j['slab_ends']['wrong'] == []
    miss = [m for m in j['slab_ends']['missed']
            if m['slab'] == 'maze_h1' and m['slab_end'] == 0]
    assert len(miss) == (0 if counted else 1)
    if not counted:
        assert miss[0]['section_7_record'] == record_
    assert j['reported']['true_pair_ends'] == {str((e['aligned'],
                                                    e['state'])): 1}


def test_y17_m4_under_an_unmatched_part_pair(truth):
    off = seg(7, '+y', [30.0, 3.1], [34.0, 3.1], line=7)
    e = end(2, 7, [-6.0, 2.9], [-6.0, 3.1])
    j = sj.judge_map(record([H1_LOW, off], pairs=[pair(2, 7, [e])]), truth,
                     ZERO, TOL)
    assert len(j['slabs']['unmatched_part']) == 1
    assert len(j['slab_ends']['unmatched_part']) == 1
    assert j['slab_ends']['wrong'] == [] and j['slabs']['wrong'] == []
    assert sj.map_counts(j)['slab_ends']['unmatched_part'] == 1
    # The pair is not true, so §7's ends are not reported as a true pair's.
    assert j['reported']['true_pair_ends'] == {}


def test_y18_m4_outside_corners_follow_their_slab_end(truth):
    e = end(2, 3, [-5.9, 2.9], [-5.9, 3.1])
    j = sj.judge_map(record([H1_LOW, H1_HIGH], pairs=[pair(2, 3, [e])]), truth,
                     ZERO, TOL)
    rec = list(j['slab_ends']['found'].values())[0]
    assert [c['p'] for c in rec['outside_corners']] == [[-5.9, 2.9],
                                                        [-5.9, 3.1]]
    assert [c['to_box_corner_m'] for c in rec['outside_corners']] == \
        pytest.approx([0.1, 0.1], abs=1e-12)
    assert j['reported']['outside_corner'] == pytest.approx([0.1, 0.1],
                                                            abs=1e-12)


def test_y19_m5_every_opening_is_spurious(truth):
    also_n = seg(8, '-y', [4.0, 9.95], [6.0, 9.95])
    j = sj.judge_map(record([ON_N, also_n], gaps=[gap(0, 8),
                                                  gap(0, 8, 0.1, 'free',
                                                      False)]),
                     truth, ZERO, TOL)
    assert len(j['openings']) == 1
    assert j['openings'][0] == {'gap': 0, 'segments': [0, 8], 'width_m': 0.3,
                                'state': 'free', 'faces': ['N', 'N']}
    assert sj.map_counts(j)['openings'] == 1


def test_y20_m6_explains_a_missed_corner(truth):
    """Y20: both faces matched, and the corner region never seen."""
    far_n = seg(0, '-y', [-9.0, 9.95], [-7.0, 9.95])
    far_e = seg(1, '-x', [9.95, -9.0], [9.95, -7.0], line=1)
    j = sj.judge_map(record([far_n, far_e]), truth, ZERO, TOL)
    miss = {m['corner']: m for m in j['inside_corners']['missed']}
    assert sorted(miss) == ['EN', 'ES', 'WN', 'WS']
    en = miss['EN']
    assert en['n_faces_matched'] == 2
    assert en['nearest_axis_end_m']['N'] == pytest.approx(
        math.dist([-7.0, 9.95], [9.95, 9.95]), abs=1e-12)
    assert en['nearest_axis_end_m']['E'] == pytest.approx(
        math.dist([9.95, -7.0], [9.95, 9.95]), abs=1e-12)
    assert miss['ES']['n_faces_matched'] == 1       # E matched, S did not
    assert 'nearest_axis_end_m' not in miss['ES']
    assert miss['WS']['n_faces_matched'] == 0


def test_y21_m6_reports_each_face_per_map(truth):
    """Y21, S-O2: how many segments match a face, and on how many entries."""
    second = seg(8, '-y', [4.0, 9.95], [6.0, 9.95], line=5)
    j = sj.judge_map(record([ON_N, second, ON_E]), truth, ZERO, TOL)
    per = j['reported']['per_face']
    assert per['N'] == {'n_segments': 2, 'n_lines_entries': 2}
    assert per['E'] == {'n_segments': 1, 'n_lines_entries': 1}
    assert per['maze_v1+x'] == {'n_segments': 0, 'n_lines_entries': 0}
    assert j['reported']['split_faces'] == ['N']
    assert j['reported']['multi_segment_faces'] == {'N': 2}
    same_entry = seg(8, '-y', [4.0, 9.95], [6.0, 9.95], line=0)
    j2 = sj.judge_map(record([ON_N, same_entry]), truth, ZERO, TOL)
    assert j2['reported']['per_face']['N'] == {'n_segments': 2,
                                               'n_lines_entries': 1}
    assert j2['reported']['split_faces'] == []


# ───────────── a hand-built corpus for the gate and the stage ────────────────

def room_grid():
    """A room of one-cell walls, free inside, unknown outside."""
    grid = np.full((40, 50), gw.UNKNOWN, dtype=np.uint8)
    grid[6:34, 6:44] = gw.FREE
    grid[5, 5:45] = gw.OCC
    grid[34, 5:45] = gw.OCC
    grid[5:35, 5] = gw.OCC
    grid[5:35, 44] = gw.OCC
    return grid


# The room is laid at (20, 20), outside the arena, so none of its four faces
# lies within tau_m of a world face -- in the truth or in the decoy, which
# moves every face by +0.5 m. At the origin its top wall sits 0.05 m from the
# decoy's maze_h1-y and its right wall 0.05 m from the decoy's maze_v2-x, and
# check 3 then fails on all 20 rows.
ROOM_ORIGIN = (20.0, 20.0)


@pytest.fixture(scope='module')
def room():
    grid = room_grid()
    graph, el, seg_of = gw.extract_with_elements(grid, RHO, ROOM_ORIGIN, P)
    return graph, el, seg_of, grid


def y_sides(room, stems=None):
    """The 20 rows of §10, each a hand-built side with the maps' thresholds."""
    graph, el, seg_of, grid = room
    out = {}
    for stem in (stems or dw.corpus_stems()):
        k, _cut = gw._k_and_cut(stem, 'the test corpus')
        th = {'rho': {'value': RHO, 'unit': 'm', 'source': 'test'},
              'eps': {'value': P.eps(RHO), 'unit': 'm', 'source': 'test'},
              'g': {'value': 0.22, 'unit': 'm', 'source': 'test'}}
        out[stem] = {'stem': stem, 'el': el, 'seg_of': seg_of, 'grid': grid,
                     'rho': RHO, 'origin': ROOM_ORIGIN, 'shape': grid.shape,
                     'graph': dict(graph, map_sha256='ab' * 32,
                                   frame=f'robot_{k} map', thresholds=th),
                     'segments': graph['segments']}
    return out


def y_stored(sides):
    """The stored A8-A11 run check 2 is an equality against."""
    return {stem: {'A9': {'recall': 0.5 + i / 100.0, 'sample_step_m': 0.01,
                          'recall_of_reachable': 0.6, 'covered_m': 64.4},
                   'A8': {'share_assigned': 1.0, 'share_all': 0.9}}
            for i, stem in enumerate(sorted(sides))}


def y_a9_of(stored, nudge=None, delta=2e-9):
    """`a9_of` for the stage: the stored values, one map optionally moved."""
    def a9_of(stem):
        r = stored[stem]['A9']['recall']
        if nudge == stem:
            r += delta
        return {'A9': {'recall': r, 'recall_of_reachable': 0.6,
                       'covered_m': 64.4},
                'A8': {'share_assigned': 1.0, 'share_all': 0.9}}
    return a9_of


def y_meta(verbatim, constants):
    return sj.schema_meta('synthetic', 'synthetic', 'cd' * 32, ['?? .vscode/'],
                          'synthetic', verbatim, constants,
                          {'a9': {'sha256': 'x'}})


def y_stage(tmp_path, room, verbatim, constants, world, check1=None,
            nudge=None, delta=2e-9, monkeypatch=None, c3_fails=False):
    sides = y_sides(room)
    stored = y_stored(sides)
    spawn = [(0.0, 0.0)] * 5
    calls = {'check1': 0, 'a9': []}

    def check1_of():
        calls['check1'] += 1
        return check1 if check1 is not None else {'ok': True, 'n_blocks': 10,
                                                  'n_grids': 160,
                                                  'file': 'synthetic',
                                                  'differing': []}

    a9 = y_a9_of(stored, nudge, delta)

    def a9_of(stem):
        calls['a9'].append(stem)
        return a9(stem)

    if c3_fails:
        monkeypatch.setattr(sj, 'check3_decoy', lambda *a: {
            'ok': False, 'shift_m': [0.5, 0.5], 'n_maps': 20,
            'n_maps_clean': 19, 'maps': [], 'differing': [
                {'map': 'one', 'n_matched': 1, 'n_with_axis_ends': 4}]})
    out = sj.schema_stage(sides, spawn, world, stored, constants,
                          y_meta(verbatim, constants), check1_of, a9_of,
                          tmp_path, tmp_path / 'maps', 'dc5e2a3')
    return out, calls


def y_failed(tmp_path, capsys):
    """The stage stopped at exit 2, with one fail file and no verdict."""
    written = list(tmp_path.glob('schema_gate_fail_*.json'))
    assert len(written) == 1
    assert not list(tmp_path.glob('schema_judge_*.json'))
    payload = json.loads(written[0].read_text())
    for key in ('judgments', 'verdicts', 'reported'):
        assert key not in payload
    assert set(payload['spec_verbatim']['items']) >= {'P-S1', 'P-S8', 'D-S5'}
    out = capsys.readouterr().out
    for name in sj.PREDICTIONS:
        assert f'{name}:' not in out
    return payload


# ───────────────────── Y22-Y26: the gate, in order ───────────────────────────

def test_y22_check_1_failing_runs_nothing_after_it(
        tmp_path, capsys, room, verbatim, constants, world):
    """Y22: no A9 recomputed, no per-map file, no match, no verdict."""
    with pytest.raises(SystemExit) as e:
        y_stage(tmp_path, room, verbatim, constants, world,
                check1={'ok': False, 'n_blocks': 10, 'n_grids': 160,
                        'file': 'synthetic',
                        'differing': [{'what': 'small/base.corners',
                                       'pinned': [64, 64, 0],
                                       'stage': [63, 64, 1]}]})
    assert e.value.code == 2
    payload = y_failed(tmp_path, capsys)
    assert payload['G-S1']['parts'] == {'1_synthetic': False,
                                        '2_frame_and_truth': None,
                                        '3_decoy_X_S1': None}
    assert payload['G-S1']['all_ok'] is False
    assert payload['per_map_files'] == [] and payload['n_door_calls'] == 0
    assert not (tmp_path / 'maps').exists()


def test_y23_check_2_is_inclusive_at_1e_9():
    """Y23: "within 1e-9" on 20 of 20 maps, the bound included."""
    stems = sorted(dw.corpus_stems())
    stored = y_stored({s: None for s in stems})
    at = sj.check2_frame_and_truth(stems, stored, y_a9_of(stored, stems[2],
                                                         1e-9), 1e-9, 0.01)
    assert at['ok'] and at['n_equal'] == 20 and at['differing'] == []
    past = sj.check2_frame_and_truth(stems, stored,
                                     y_a9_of(stored, stems[2], 1.001e-9),
                                     1e-9, 0.01)
    assert not past['ok'] and past['n_equal'] == 19
    assert [m['map'] for m in past['differing']] == [stems[2]]
    assert past['maps'][2]['beside']['covered_m'] == {'stored': 64.4,
                                                      'computed': 64.4}
    short = sj.check2_frame_and_truth(stems[:19], stored,
                                      y_a9_of(stored), 1e-9, 0.01)
    assert not short['ok'] and not short['covers_the_20_maps']


def test_y23_check_2_past_its_tolerance_stops_before_a_per_map_file(
        tmp_path, capsys, room, verbatim, constants, world):
    """Y23: 2e-9 on one stored recall of the 20, against a bound of 1e-9."""
    stem = sorted(dw.corpus_stems())[7]
    with pytest.raises(SystemExit) as e:
        y_stage(tmp_path, room, verbatim, constants, world, nudge=stem)
    assert e.value.code == 2
    payload = y_failed(tmp_path, capsys)
    assert payload['G-S1']['parts']['1_synthetic'] is True
    assert payload['G-S1']['parts']['2_frame_and_truth'] is False
    assert payload['G-S1']['parts']['3_decoy_X_S1'] is None
    assert [m['map'] for m in payload['G-S1']['2_frame_and_truth']
            ['differing']] == [stem]
    assert payload['per_map_files'] == []
    assert not (tmp_path / 'maps').exists()


def test_y24_check_3_failing_writes_no_verdict(
        tmp_path, capsys, room, verbatim, constants, world, monkeypatch):
    """Y24: the per-map files are written, and nothing is judged."""
    with pytest.raises(SystemExit) as e:
        y_stage(tmp_path, room, verbatim, constants, world, c3_fails=True,
                monkeypatch=monkeypatch)
    assert e.value.code == 2
    payload = y_failed(tmp_path, capsys)
    assert payload['G-S1']['parts']['3_decoy_X_S1'] is False
    assert len(payload['per_map_files']) == 20
    assert len(list((tmp_path / 'maps').glob('*.json'))) == 20
    assert payload['n_door_calls'] == 0


def test_y25_check_3_is_the_decoy(truth, world):
    """Y25, X-S1: it is built to find nothing, and it does find a planted one."""
    decoy = sj.truth(world, (0.5, 0.5))
    stems = sorted(dw.corpus_stems())
    clean = {s: record([ON_N, ON_E]) for s in stems}
    spawn = [(0.0, 0.0)] * 5
    got = sj.check3_decoy(clean, spawn, decoy, TOL)
    assert got['ok'] and got['n_maps'] == 20 and got['n_maps_clean'] == 20
    assert all(m['n_matched'] == 0 for m in got['maps'])
    on_decoy = dict(clean)
    on_decoy[stems[3]] = record([seg(0, '-y', [0.5, 10.45], [2.5, 10.45])])
    bad = sj.check3_decoy(on_decoy, spawn, decoy, TOL)
    assert not bad['ok'] and bad['n_maps_clean'] == 19
    assert [m['map'] for m in bad['differing']] == [stems[3]]
    # The same record against the real truth matches nothing either way.
    assert sj.match_faces(on_decoy[stems[3]], truth, (0.0, 0.0),
                          TOL['tau_m']) == {0: None}


def test_y26_the_gate_passing_writes_one_results_file(
        tmp_path, capsys, room, verbatim, constants, world):
    """Y26: the door opens, all 20 maps are judged, and the file is JSON.

    The hand-built room sits nowhere near a world face, so every segment is
    unmatched and every corner unmatched-part: P-S1, P-S3, P-S5 and P-S7 fail,
    which is the spec's own point about a frame that places nothing.
    """
    out, calls = y_stage(tmp_path, room, verbatim, constants, world)
    assert calls['check1'] == 1 and len(calls['a9']) == 20
    assert out['G-S1']['all_ok'] is True
    assert len(out['judgments']) == 20 and len(out['per_map_files']) == 20
    written = list(tmp_path.glob('schema_judge_*.json'))
    assert len(written) == 1
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()
    payload = json.loads(written[0].read_text())
    assert payload['stage'] == 'schema_judge'
    assert payload['n_maps'] == 20 and payload['rho_m'] == RHO
    assert sorted(payload['verdicts']) == list(sj.PREDICTIONS)
    assert payload['truth']['counts']['faces'] == 12
    for j in payload['judgments']:
        assert j['inside_corners']['unmatched_part'] == 4
        assert j['unmatched_segments'] == 4
    v = payload['verdicts']
    assert not v['P-S1']['holds'] and v['P-S1']['n_found'] == 0
    assert not v['P-S3']['holds'] and not v['P-S5']['holds']
    assert not v['P-S7']['holds'] and v['P-S7']['n_maps_with'] == 20
    assert v['P-S2']['holds'] and v['P-S4']['holds'] and v['P-S6']['holds']
    assert v['P-S8']['holds']
    assert len(payload['reported']['per_map']) == 20
    out_text = capsys.readouterr().out
    assert 'P-S1: FAIL' in out_text and 'P-S2: HOLD' in out_text
    with pytest.raises(AssertionError):           # not a gate payload
        sj._no_verdict_in_schema(payload)


def test_y26_the_door_raises_until_the_gate_has_passed(truth):
    """Y26: before G-S1, and after any failed or missing check, it raises."""
    records = {s: record([ON_N]) for s in sorted(dw.corpus_stems())}
    stem = sorted(records)[0]
    door = sj.SchemaDoor(records, [(0.0, 0.0)] * 5, truth, TOL)
    with pytest.raises(sj.SchemaGateClosed):
        door.judge(stem)
    ok = {'ok': True}
    for checks in ((ok, ok, {'ok': False}), (ok, ok, None), (ok, ok),
                   (ok, ok, {'ok': 1}), (None, ok, ok)):
        assert not door.open_after(*checks)
        with pytest.raises(sj.SchemaGateClosed):
            door.judge(stem)
    assert door.asked == [] and not door.is_open
    assert door.open_after(ok, ok, ok) and door.is_open
    assert door.judge(stem)['faces'] == {0: 'N'}
    assert door.asked == [stem]


# ───────────────────── Y27-Y28: the verdicts in integers ─────────────────────

def judgment(stem, cut, corners=(0, 0, 0, 0), slabs=(0, 0, 0, 0),
             ends=(0, 0, 0, 0), openings=0):
    def kind(t):
        return {'found': t[0], 'missed': t[1], 'wrong': t[2],
                'unmatched_part': t[3], 'duplicates': 0}
    k, _c = gw._k_and_cut(stem, 'the test rows')
    return {'map': stem, 'robot': k, 'cut': cut,
            'inside_corners': kind(corners), 'slabs': kind(slabs),
            'slab_ends': kind(ends), 'openings': openings,
            'unmatched_segments': 0}


def y_judgments(**edits):
    """One judgment per row: every true element found, nothing spurious."""
    out = []
    for stem in sorted(dw.corpus_stems()):
        _k, cut = gw._k_and_cut(stem, 'the test rows')
        full = cut == FULL
        j = judgment(stem, cut,
                     corners=(4, 0, 0, 0) if full else (0, 4, 0, 0),
                     slabs=(4, 0, 0, 0) if full else (0, 4, 0, 0),
                     ends=(8, 0, 0, 0) if full else (0, 8, 0, 0))
        if stem in edits:
            for key, value in edits[stem].items():
                if key == 'openings':
                    j['openings'] = value
                else:
                    j[key].update(value)
        out.append(j)
    return out


def test_y27_the_found_verdicts_at_their_bound(constants, truth):
    """Y27: P-S1 at 18 and at 17 of the 20 true corners at cut1200."""
    full = [s for s in sorted(dw.corpus_stems()) if f'cut{FULL}' in s]
    assert len(full) == 5
    v = sj.verdicts_from(y_judgments(), constants, truth)
    assert v['P-S1']['holds'] and v['P-S1']['n_found'] == 20
    assert v['P-S1']['of'] == 20 and v['P-S1']['min_required'] == 18
    assert v['P-S3']['n_found'] == 20 and v['P-S5']['n_found'] == 40
    assert v['P-S5']['of'] == 40 and v['P-S5']['min_required'] == 36
    assert all(v[n]['holds'] for n in sj.PREDICTIONS)

    at18 = sj.verdicts_from(
        y_judgments(**{full[0]: {'inside_corners': {'found': 2, 'missed': 2}},
                       full[1]: {'slab_ends': {'found': 4, 'missed': 4}}}),
        constants, truth)
    assert at18['P-S1']['holds'] and at18['P-S1']['n_found'] == 18
    assert at18['P-S5']['holds'] and at18['P-S5']['n_found'] == 36

    at17 = sj.verdicts_from(
        y_judgments(**{full[0]: {'inside_corners': {'found': 1, 'missed': 3}},
                       full[1]: {'slabs': {'found': 1, 'missed': 3}}}),
        constants, truth)
    assert not at17['P-S1']['holds'] and at17['P-S1']['n_found'] == 17
    assert not at17['P-S3']['holds'] and at17['P-S3']['n_found'] == 17
    assert at17['P-S5']['holds']


def test_y27_the_maps_verdicts_at_their_bound(constants, truth):
    """Y27: P-S2, P-S4, P-S7 and P-S8 hold at one map and fail at two."""
    rows = sorted(dw.corpus_stems())
    one = sj.verdicts_from(
        y_judgments(**{rows[0]: {'inside_corners': {'wrong': 1},
                                 'slabs': {'wrong': 1},
                                 'slab_ends': {'unmatched_part': 2}},
                       rows[1]: {'inside_corners': {'unmatched_part': 1}}}),
        constants, truth)
    for name in ('P-S2', 'P-S4', 'P-S7', 'P-S8'):
        assert one[name]['holds'] and one[name]['n_maps_with'] == 1
        assert one[name]['at_most'] == 1 and one[name]['of'] == 20
    assert one['P-S2']['maps'] == [rows[0]] and one['P-S7']['maps'] == [rows[1]]

    two = sj.verdicts_from(
        y_judgments(**{rows[0]: {'inside_corners': {'wrong': 1}},
                       rows[1]: {'inside_corners': {'wrong': 2}},
                       rows[2]: {'slab_ends': {'wrong': 1}},
                       rows[3]: {'slabs': {'wrong': 1}},
                       rows[4]: {'slabs': {'unmatched_part': 1}},
                       rows[5]: {'slab_ends': {'unmatched_part': 1}}}),
        constants, truth)
    assert not two['P-S2']['holds'] and two['P-S2']['n_maps_with'] == 2
    assert not two['P-S4']['holds'] and two['P-S4']['n_maps_with'] == 2
    assert not two['P-S8']['holds'] and two['P-S8']['n_maps_with'] == 2
    assert two['P-S7']['holds'] and two['P-S7']['n_maps_with'] == 0


def test_y28_the_rows_each_prediction_is_judged_on(constants, truth):
    """Y28, D-S7: found elements and P-S6 at cut1200; the rest on all 20."""
    rows = sorted(dw.corpus_stems())
    early = [s for s in rows if f'cut{FULL}' not in s]
    full = [s for s in rows if f'cut{FULL}' in s]
    v = sj.verdicts_from(
        y_judgments(**{early[0]: {'openings': 3}, early[1]: {'openings': 1}}),
        constants, truth)
    assert v['P-S6']['holds'] and v['P-S6']['n_maps_with'] == 0
    assert v['P-S6']['of'] == 5
    at_full = sj.verdicts_from(
        y_judgments(**{full[0]: {'openings': 1}, full[1]: {'openings': 1}}),
        constants, truth)
    assert not at_full['P-S6']['holds'] and at_full['P-S6']['n_maps_with'] == 2
    # A wrong corner at an early cut does count: P-S2 is judged on all 20.
    wrong_early = sj.verdicts_from(
        y_judgments(**{early[0]: {'inside_corners': {'wrong': 1}},
                       early[1]: {'inside_corners': {'wrong': 1}}}),
        constants, truth)
    assert not wrong_early['P-S2']['holds']
    # A found element at an early cut does not: only the five cut1200 rows.
    found_early = sj.verdicts_from(
        y_judgments(**{early[0]: {'inside_corners': {'found': 4,
                                                     'missed': 0}}}),
        constants, truth)
    assert found_early['P-S1']['n_found'] == 20


def test_y28_a_denominator_that_moved_refuses(constants, truth):
    """The 20 of P-S1 is five maps times the world's four corners."""
    short = [j for j in y_judgments() if j['cut'] != FULL or j['robot'] != 4]
    with pytest.raises(SystemExit):
        sj.verdicts_from(short, constants, truth)


# ───────────────────── Y29: the refusals, before anything ────────────────────

def _spies(monkeypatch):
    seen = {'load_side': 0, 'stage': 0, 'sdf': 0}

    def load_side(*a, **k):
        seen['load_side'] += 1
        raise AssertionError('run_schema loaded a map past a refusal')

    def stage(*a, **k):
        seen['stage'] += 1
        raise AssertionError('run_schema reached the stage past a refusal')

    def sdf(*a, **k):
        seen['sdf'] += 1
        raise AssertionError('run_schema read the world past a refusal')

    monkeypatch.setattr(dw, 'load_side', load_side)
    monkeypatch.setattr(sj, 'schema_stage', stage)
    monkeypatch.setattr(gw, 'sdf_wall_faces', sdf)
    return seen


NOTHING = {'load_side': 0, 'stage': 0, 'sdf': 0}


def _git_ok(text=None, registered=None):
    """A clean tree, the registration an ancestor, and the spec at HEAD."""
    body = text if text is not None else spec_text()

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return (registered if registered is not None and
                    a[1].startswith(sj.SCHEMA_REGISTRATION) else body)
        if a[0] in ('merge-base', 'rev-parse'):
            return 'c0ffee' * 6
        return 'c0ffee'
    return fake


def test_y29_refuses_a_dirty_tree(monkeypatch, tmp_path):
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', lambda *a: (
        ' M experiments/analysis/schema_judge.py' if a[0] == 'status'
        else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING


def test_y29_refuses_when_the_registration_is_not_an_ancestor(monkeypatch,
                                                              tmp_path):
    seen, asked = _spies(monkeypatch), []

    def fake(*a):
        asked.append(a)
        return '' if a[0] in ('status', 'merge-base') else 'deadbeef'

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING
    assert ('merge-base', sj.SCHEMA_REGISTRATION, 'HEAD') in asked
    assert sj.SCHEMA_REGISTRATION == '5bf3ee2'


@pytest.mark.parametrize('missing', ['### Gate G-S1', '**P-S1,', '**P-S8,'])
def test_y29_refuses_a_spec_without_the_gate_or_a_prediction(missing,
                                                             monkeypatch,
                                                             tmp_path):
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git',
                        _git_ok(spec_text().replace(missing, 'NOT THERE')))
    with pytest.raises(SystemExit) as e:
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING


def test_y29_refuses_a_registered_body_that_moved(monkeypatch, tmp_path):
    """Y29, ruling 4: ancestry does not catch a later edit to §1-§10."""
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', _git_ok(
        registered=spec_text().replace('`s_max = 0.5 m`', '`s_max = 0.6 m`')))
    with pytest.raises(SystemExit) as e:
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING


def test_y29_refuses_a_missing_or_wrong_pinned_file(monkeypatch, tmp_path):
    """Y29: G-S1 is an equality against those four files and no others."""
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', _git_ok())
    with pytest.raises(SystemExit) as e:          # all four missing
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING
    (tmp_path / sj.A9_FILE).write_text('{}')      # present, wrong bytes
    with pytest.raises(SystemExit) as e:
        sj.run_schema(tmp_path, tmp_path, a9_dir=tmp_path)
    assert e.value.code == 2 and seen == NOTHING
    assert sj.A9_SHA256 != hashlib.sha256(b'{}').hexdigest()


# ─────────────── Y30-Y33: the files, the builder and the fail walk ───────────

def test_y30_the_per_map_files_are_written_read_back_and_hashed(tmp_path,
                                                                room):
    sides = y_sides(room, sorted(dw.corpus_stems())[:3])
    records, files = sj.write_per_map(sides, tmp_path / 'maps', 'dc5e2a3')
    assert sorted(records) == sorted(sides) and len(files) == 3
    for rec in files:
        path = tmp_path / 'maps' / rec['file']
        assert rec['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
        assert json.loads(path.read_text()) == records[rec['map']]
        assert records[rec['map']]['graph_walls_commit'] == 'dc5e2a3'
        assert rec['counts'] == records[rec['map']]['counts']
    # §9 reads the record the file holds, and the file holds no truth.
    for rec in records.values():
        assert 'world' not in json.dumps(rec)
    with pytest.raises(SystemExit):               # never a second time
        sj.write_per_map(sides, tmp_path / 'maps', 'dc5e2a3')


# From `schema_v0_1_synth2_out.txt`, sha256 `364def1b…`, block "small, base":
# the counts and distances G-S1 check 1 reads in full on the run.
Y31_SMALL_BASE = {
    'no_axis': 0,
    'corners': (64, 64, 0),
    'false_corners': 0,
    'distances': ('0.0000', '0.0000', '0.0060', '0.1881', '0.1220', '0.0000',
                  '0.2912'),
    'states': {'OO': 64},
    'slabs': {'slab': (16, 16, '0.1879', '0.2882')},
    'slab_missed': [],
    'false_pairs': 0,
    'ends': (32, '0.0044', {"(True, 'free')": 32}, 64, '0.0057', '0.1076', 0),
    'gaps': (0, 0),
}


def test_y31_the_restated_160_grid_builder(capsys):
    """Y31: one block of the pinned corpus, every field it states."""
    w = sj.check_world('small')
    assert w['inner'] == 2.95 and w['slabs'] == [('slab', -1.0, 1.5, 0.9, 1.1)]
    assert len(w['faces']) == 8 and sj.world_half(w) == 3.5
    assert sorted(w['true_in']) == [(-2.95, -2.95), (-2.95, 2.95),
                                    (2.95, -2.95), (2.95, 2.95)]
    assert sorted(w['true_out']) == [(-1.0, 0.9), (-1.0, 1.1), (1.5, 0.9),
                                     (1.5, 1.1)]
    grid, org = sj.build_grid(w, (0.0, 0.0), 3.5, 0.0, 0.0, 'base')
    assert grid.shape == (70, 70)
    assert org == (-3.45, -3.45)
    assert set(np.unique(grid)) == {gw.OCC, gw.FREE, gw.UNKNOWN}
    got = sj.as_160_block(sj.judge_160(w, 'base', P))
    assert got == Y31_SMALL_BASE


def test_y32_the_pinned_outputs_are_parsed_block_by_block():
    """Y32: a line the stage cannot read is a refusal, not a silent skip."""
    one = ('\n=== small, base: 16 grids, tau_c 0.2707, s_max 0.5 ===\n'
           'segments without axis ends: 0\n'
           'tilt of segments with risers 0.174..0.835 deg (64 segs); all '
           'segments 0.000..0.835 deg\n'
           'inside corners found 64 of 64, missed 0 []\n'
           '  nearer axis end -> c 0.0000..0.0000; farther axis end -> c '
           '0.0060..0.1881; c -> true 0.1220 max; axis end -> true corner '
           '0.0000..0.2912\n'
           "  axis end states at found corners {'OO': 64}\n"
           '  own-cell rule differences near true inside corners (draft rule, '
           "all-members): {('occupied', 'unknown'): 2}\n"
           'false inside corners []; other junctions []\n'
           '  slab slab: pairs found 16 of 16, separation 0.1879..0.2882\n'
           'false pairs []\n'
           "slab ends examined 32: alignment max 0.0044; (aligned, state) "
           "{(True, 'free'): 32}; outside corners 64, distance to true outside "
           'corner 0.0057..0.1076; false []\n'
           'gaps formed (segments consecutive on one entry) 0; openings 0\n')
    with pytest.raises(SystemExit):               # one block, ten expected
        sj.parse_160_out(one)
    monkeyed = sj.V1_WORLDS
    try:
        sj.V1_WORLDS, sj.V1_VARIANTS = ('small',), ('base',)
        block = sj.parse_160_out(one)[('small', 'base')]
        for key, value in Y31_SMALL_BASE.items():
            assert block[key] == value, key
        with pytest.raises(SystemExit):
            sj.parse_160_out(one + 'a line nobody reads\n')
        with pytest.raises(SystemExit):           # a field dropped
            sj.parse_160_out(one.replace('false pairs []\n', ''))
    finally:
        sj.V1_WORLDS, sj.V1_VARIANTS = monkeyed, sj.V1_VARIANTS


Y32_640 = (
    'tau_m 0.2000  tau_end 0.2500  spawns [(0.86, 0.28), (0.0, 0.9)]\n'
    'truth: 12 long faces, 4 inside corners, 4 slabs, 8 slab ends\n'
    "closest same-facing, overlapping long faces: (6.0, 'maze_h1+y', "
    "'maze_h2+y')\n"
    '\n=== base: 80 grids (5 spawns x 16 phases) ===\n'
    'segments without axis ends 0; segments matching no face 0\n'
    'inside corners found/missed/wrong/unmatched-part [320, 0, 0, 0] of 320\n'
    'slabs          found/missed/wrong/unmatched-part [320, 0, 0, 0] of 320\n'
    'slab ends      found/missed/wrong/unmatched-part [640, 0, 0, 0] of 640; '
    "ends of true pairs (aligned, state) {(True, 'free'): 640}\n"
    'duplicates (corners, slabs, slab ends) [0, 0, 0]\n'
    'openings 0; faces on >1 lines[] entry none; faces with >1 segment none\n'
    'max: m(S) perp 0.0910 | axis end perp 0.0968 | c->corner 0.1301 (cheb '
    '0.0939) | slab end along 0.0900 | outside corner 0.1321\n'
    'control (+0.5, +0.5): segments matched 0; corners/slabs/ends found '
    '[0, 0, 0]; wrong/unmatched-part corners [0, 320]\n'
    'largest m(S) distance, unrounded: 0.09098039120664403\n'
    '\nlargest m(S) distance over all 640 grids, unrounded: '
    '0.19098039120664012 (thickE)\n'
    'tau_m - largest, rho = 0.1: 0.0090196 m (0.00901960879335989)\n'
    "tau_m - largest, rho = 0.10000000149011612 (the maps' YAML): 0.0090196 m "
    '(0.009019611773592118)\n')


def test_y32_the_640_grid_output_is_parsed():
    with pytest.raises(SystemExit):               # one block, eight expected
        sj.parse_640_out(Y32_640)
    kept = sj.D2_VARIANTS
    try:
        sj.D2_VARIANTS = ('base',)
        got = sj.parse_640_out(Y32_640)
        assert got['head']['tolerances'][:2] == ('0.2000', '0.2500')
        assert got['head']['truth'] == ('12', '4', '4', '8')
        assert got['head']['gap'] == ("(6.0, 'maze_h1+y', 'maze_h2+y')",)
        b = got['blocks']['base']
        assert b['segments'] == (0, 0)
        assert b['corners'] == ([320, 0, 0, 0], 320)
        assert b['ends'] == ([640, 0, 0, 0], 640, {"(True, 'free')": 640})
        assert b['openings'] == (0, {}, {})
        assert b['control'] == ((0.5, 0.5), 0, [0, 0, 0], [0, 320])
        assert b['largest'] == ('0.09098039120664403',)
        assert got['tail']['largest'] == (640, '0.19098039120664012', 'thickE')
        assert got['tail']['margins'] == [
            ('0.1', '0.0090196', '0.00901960879335989'),
            ('0.10000000149011612', '0.0090196', '0.009019611773592118')]
        with pytest.raises(SystemExit):
            sj.parse_640_out(Y32_640 + 'a line nobody reads\n')
    finally:
        sj.D2_VARIANTS = kept


def test_y32_the_unrounded_value_is_compared_to_1e_14():
    """`_close`: the one quantity not compared exactly, and its bound."""
    assert sj.UNROUNDED_TOL_M == 1e-14
    out = []
    assert sj._close(0.19098039120664012, '0.19098039120664012', 'x', out)
    assert sj._close(0.19098039120664012 + 9e-15, '0.19098039120664012', 'x',
                     out)
    assert out == []
    assert not sj._close(0.19098039120664012 + 2e-14, '0.19098039120664012',
                         'x', out)
    assert not sj._close(None, '0.19098039120664012', 'y', out)
    assert [r['what'] for r in out] == ['x', 'y']


def test_y33_a_fail_payload_may_hold_no_verdict_and_no_match():
    """Y33: the walk the fail path runs, and what it is allowed to carry."""
    assert sj.BANNED_IN_A_FAIL >= set(sj.PREDICTIONS) | {'judgments',
                                                         'verdicts',
                                                         'reported', 'holds'}
    gate = {'1_synthetic': {'ok': True, 'differing': []},
            '2_frame_and_truth': {'ok': True, 'maps': [{'map': 'a'}]},
            '3_decoy_X_S1': None}
    sj._no_verdict_in_schema({'stage': 'schema_judge_gate_fail', 'G-S1': gate,
                              'spec_verbatim': {'items': {'P-S1': 'text'}}})
    for planted in ({'verdicts': {}}, {'judgments': []},
                    {'G-S1': {'deep': [{'P-S4': 1}]}},
                    {'reported': {'per_map': []}}):
        with pytest.raises(AssertionError):
            sj._no_verdict_in_schema(dict({'stage': 'x'}, **planted))
