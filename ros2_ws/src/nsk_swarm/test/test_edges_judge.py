"""The B2 face-graph stage -- experiments/analysis/edges_judge.py.

docs/specs/SPEC_b2_edges.md v0.2, §5 (F*, gate G-E1 and P-E1 to P-E4). The
registered text of the real spec is read, because the stage's whole claim is
that it applies the text that was registered; truth comes from the world SDF,
which is committed; everything else runs on hand-built records. No corpus map,
no schema or graph_walls JSON, no gitignored scratch file and no git is
touched: the 20 maps are the run's job, and the run is not here.

THE TABLE BELOW WAS FIXED BEFORE ANY CODE EXISTED, in the build plan's §4.5
(experiments/logs/splice/edges_build_plan.md), under Ali's ruling of 18 Oct
2026 that accepted all 34 of its proposals (Q8's order, Q10's whole samples,
Q13's outside decoy, Q14's pinned F* file, Q17's once-per-map count).

  id   case                                               expected
  ───────────────────────────────────────────────────────────────────────────
  J0   the registered text, one item at a time            P-E1's item holds no
                                                          "P-E2"; item 1 holds
                                                          no "Under v0.2"
  J1   every number of the plan's §4.2                    the values there; the
                                                          finders are exactly
                                                          its 21 rows
  J2   the ASCII stand-ins "1e-9", ">=" and "-"           each refuses
  J3   11 mutations of the registered numbers             each refuses
  J4   the refusals: a dirty tree; 358650a no ancestor;   each exits 2, before
       a missing label (5); §1-§5 changed; a pinned       anything is loaded
       file missing; a pinned hash wrong
  J5   the truth faces                                    12; E (+9.95, -9.95)
                                                          to (+9.95, +9.95);
                                                          4x199 + 4x70 + 4x50 =
                                                          1276 samples
  J6   without caps, v0.1 is reproduced                   §5.1's 12 widths in
                                                          whole samples; 20
                                                          through a slab, split
                                                          3, 3, 3, 3, 2, 2, 2, 2
  J7   item 2                                             §4.4's table; blocked
                                                          20, no hit 0, ties 0,
                                                          through a slab 0;
                                                          components 1, 2, 2
  J8   item 3: the outside decoy; a 1 m decoy inside      §4.4's numbers; one
       the arena (y = 0, x -0.5 to 0.5, normal +y)        edge to maze_h1-y, 10
                                                          + 10, width 1.00, gap
                                                          2.90, and item 3 fails
  J9   item 2 with the caps removed                       the four widths are
                                                          not reduced; it fails
  J10  item 1: one to one; a corner edge dropped; an      holds; fails; fails
       extra slab edge
  J11  the inputs: a per-map sha256 off the results       each refuses
       file; two map_sha256 that disagree; two rho
  J12  the gate's order, on 20 hand-built rows: item 2    no file and no M1;
       fails; item 3 fails; item 1 fails; all hold        no file; the files
                                                          and no M1; one results
                                                          file, 20 per-map files
                                                          and 4 verdicts
  J13  the door                                           raises until all three
                                                          items hold
  J14  P-E2: all 12 faces on their lines, the 12 F*       12; 11; 10
       edges; one dropped; two dropped
  J15  two segments on maze_h1+y, each joined to N        the pair counts once;
                                                          both edges reported
  J16  an edge between E and maze_v1-x                    wrong
  J17  an edge with one face unmatched                    unmatched, not wrong
  J18  P-E1 with no corner edges                          2 components
  J19  the bounds: P-E1 at 4 and 3 maps; P-E2 at 4 and    hold, fail, each
       3; P-E3 at 1 and 2; P-E4 at 1 and 2
  J20  the rows                                           the verdicts use only
                                                          the 5 cut1200 rows;
                                                          60, 120 and 240 only
                                                          under `reported`
  J21  M1 through the door, with spawn[k]                 0.19 m off N matches;
                                                          0.21 m does not
  J22  the fail walker                                    a planted verdict is
                                                          caught; a clean fail
                                                          payload passes
  J23  the results file                                   a JSON round trip;
                                                          never overwritten

J6 also pins the F* file's 12 rows to its printed decimals, quoted below from
`edges_truth_fstar_out.txt`, sha256 `6dbc3354…`, which the run reads in full
(Q14, Q34); here they are a literal, so CI checks the port without the
gitignored file. J3's control, the unmutated text, passes in J6.

The module is loaded by location, as test_schema_judge.py loads its script.
"""

import hashlib
import importlib.util
import json
import math
import os
from datetime import datetime, timezone

import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'edges_judge.py')
SPEC_PATH = os.path.join(REPO_ROOT, 'docs', 'specs', 'SPEC_b2_edges.md')
SCHEMA_SPEC_PATH = os.path.join(REPO_ROOT, 'docs', 'specs',
                                'SPEC_b2_schema.md')


def load_module():
    spec = importlib.util.spec_from_file_location('edges_judge', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ej = load_module()
ew, sj, dw, gw = ej.ew, ej.sj, ej.dw, ej.gw
sw = ew.sw

RHO = 0.1
MAP_RHO = 0.10000000149011612      # the maps' rho, as their YAML gives it
TAU_M = 2 * MAP_RHO                # M1's tau_m as the schema stage applied it
P = gw.Params(g=0.22)
AXIS = {d[4]: k for k, d in enumerate(gw.DIRS)}
VEC = {d[4]: (float(d[2]), float(d[3])) for d in gw.DIRS}
FULL = dw.FULL_CUT
STEMS = sorted(dw.corpus_stems())
FULL_STEMS = [s for s in STEMS if gw._k_and_cut(s, 'the test rows')[1] == FULL]
ZERO = (0.0, 0.0)
PROV = {'schema_file_sha256': 'cd' * 32, 'graph_walls_file_sha256': 'ef' * 32,
        'builder_commit': 'c0ffee1'}


def spec_text():
    with open(SPEC_PATH, encoding='utf-8') as f:
        return f.read()


def schema_spec_text():
    with open(SCHEMA_SPEC_PATH, encoding='utf-8') as f:
        return f.read()


@pytest.fixture(scope='module')
def verbatim():
    return ej.edges_verbatim(spec_text())


@pytest.fixture(scope='module')
def constants(verbatim):
    return ej.edges_constants_from(verbatim)


@pytest.fixture(scope='module')
def schema_constants():
    return sj.schema_constants_from(sj.schema_verbatim(schema_spec_text()))


@pytest.fixture(scope='module')
def world():
    return gw.sdf_wall_faces(gw.L_MIN)


@pytest.fixture(scope='module')
def truth(world):
    return sj.truth(world)


@pytest.fixture(scope='module')
def G01(world):
    """Truth cast without the caps: v0.1's occluders."""
    return ej.truth_graph(world, ej.TRUTH_RHO, caps=False)


@pytest.fixture(scope='module')
def G2(world):
    """Truth under v0.2: G-E1 item 2's cast."""
    return ej.truth_graph(world, ej.TRUTH_RHO)


@pytest.fixture(scope='module')
def G3(world):
    """Item 2's cast with Q13's decoy added: G-E1 item 3's."""
    return ej.truth_graph(world, ej.TRUTH_RHO, decoy=ej.DECOY)


@pytest.fixture(scope='module')
def fstar(G2):
    return ej.fstar_of(G2)


# ─────────────────── J0-J4: the registered text and its numbers ──────────────

def test_j0_the_registered_text_one_item_at_a_time(verbatim):
    it = verbatim['items']
    assert it['P-E1'] == ("**P-E1. One component.** Each map's graph is one "
                          'connected component. Bound: ≥ 4 of 5 maps.')
    assert 'P-E2' not in it['P-E1']
    assert it['gate_1'].startswith('On 20 of 20 maps, `corner` edges')
    assert 'Under v0.2' not in it['gate_1']
    assert it['gate_2'].startswith('Under v0.2, truth gives exactly the 12')
    assert it['gate_3'] == ('A decoy face placed far from every wall gets no '
                            '`facing` edge.')
    assert it['occluders'].endswith('On truth these are the 8 slab end caps.')
    assert it['rows'] == '### 5.3 Predictions, at cut1200 (5 maps)'
    assert verbatim['blocks']['5.1'].startswith('### 5.1 F*, pinned')
    assert '### 5.2' not in verbatim['blocks']['5.1']
    assert len(verbatim['table']['rows']) == 12


def test_j1_every_number_the_stage_applies(constants):
    """J1: the plan's §4.2, one finder per row and no other."""
    assert len(ej.FINDERS) == 21 and set(constants) == set(ej.FINDERS)
    assert constants['rho'] == {'truth': 0.1}
    assert constants['w_min'] == {'value_m': 0.2}
    assert constants['min_samples'] == {'n': 4, 'formula': '2*w_min/rho'}
    assert constants['slack'] == {'value_m': 1e-9}
    assert constants['parallel'] == {'value': 1e-12}
    assert constants['boundary_m'] == {'value_m': 19.9}
    assert constants['caps'] == {'n': 8}
    fs = constants['fstar']
    assert fs['n_faces'] == 12 and fs['n_pairs'] == 12
    assert fs['pairs'][0] == ['E', 'W', '11.80']
    assert fs['pairs'][1] == ['E', 'maze_v1+x', '3.00']
    assert fs['pairs'][-1] == ['maze_v1+x', 'maze_v2-x', '2.00']
    assert round(sum(float(w) for _a, _b, w in fs['pairs']), 2) == 63.8
    assert sum(ej.fstar_samples(constants).values()) == 1276
    assert constants['fstar_file'] == {'file': 'edges_truth_fstar_out.txt',
                                       'sha256_prefix': '6dbc3354',
                                       'cited': 2}
    assert ej.FSTAR_SHA256.startswith('6dbc3354')
    assert constants['drop'] == {
        'n_pairs': 4, 'm': 0.25, 'samples': 5,
        'pairs': [['E', 'maze_v1+x'], ['N', 'maze_h2+y'], ['S', 'maze_h1-y'],
                  ['W', 'maze_v2-x']]}
    assert constants['through_slab'] == {'n': 20}
    assert constants['components'] == {'with_corners': 1, 'without_corners': 2,
                                       'n_corners': 4, 'n_faces': 12}
    assert constants['gate_maps'] == {'n': 20, 'of': 20}
    assert constants['gate_through_slab'] == {'n': 0}
    assert constants['p-e1'] == {'components': 1, 'min': 4, 'of': 5}
    assert constants['p-e2'] == {'min_pairs': 11, 'of_pairs': 12, 'min': 4,
                                 'of': 5}
    assert constants['p-e3'] == {'at_most': 1, 'of': 5}
    assert constants['p-e4'] == {'at_most': 1, 'of': 5}
    assert constants['rows'] == {'cut': 1200, 'n': 5}
    assert constants['reported_cuts'] == [60, 120, 240]
    assert constants['d_e1'] == {'no_axis_member': 5, 'segments': 179,
                                 'maps': 20}


def test_j2_ascii_stand_ins_refuse():
    """J2: 10⁻⁹ in superscripts, ≥ as U+2265, and F*'s pairs split by U+2013."""
    text = spec_text()
    for old, new in (('10⁻⁹', '1e-9'), ('≥', '>='),
                     ('–', '-')):
        assert old in text, old
        with pytest.raises(SystemExit):
            ej.edges_constants_from(ej.edges_verbatim(text.replace(old, new)))


def j3_apply(text, G01):
    """Every place a found number is checked before the gate, in order."""
    constants = ej.edges_constants_from(ej.edges_verbatim(text))
    ej.check_edges_truth(G01, constants)
    return constants


J3_MUTATIONS = (
    ('that is, ≥ 4', 'that is, ≥ 5'),
    ('| E – W | 11.80 |', '| E – W | 11.90 |'),
    ('| maze_v1+x – maze_v2-x | 2.00 |', '| maze_v1+x – maze_v1-x | 2.00 |'),
    ('drop by 0.25 m', 'drop by 0.20 m'),
    ('remove the 20 samples', 'remove the 21 samples'),
    ('Bound: ≥ 4 of 5 maps.', 'Bound: ≥ 3 of 5 maps.'),
    ('≥ 11 of 12', '≥ 10 of 12'),
    ('wrong edges on ≤ 1 map', 'wrong edges on ≤ 2 map'),
    ('sha256 `6dbc3354…`), and it', 'sha256 `6dbc3355…`), and it'),
    ('10⁻¹²', '10⁻¹¹'),
    ('| maze_v1+x – maze_v2-x | 2.00 |\n',
     '| maze_v1+x – maze_v2-x | 2.00 |\n| E – N | 1.00 |\n'),
)


@pytest.mark.parametrize('old, new', J3_MUTATIONS)
def test_j3_a_changed_number_refuses(old, new, G01):
    text = spec_text()
    assert text.count(old) == 1, old
    with pytest.raises(SystemExit):
        j3_apply(text.replace(old, new, 1), G01)


def _spies(monkeypatch):
    seen = {'read_inputs': 0, 'stage': 0, 'sdf': 0}

    def read_inputs(*a, **k):
        seen['read_inputs'] += 1
        raise AssertionError('run_edges loaded a map past a refusal')

    def stage(*a, **k):
        seen['stage'] += 1
        raise AssertionError('run_edges reached the stage past a refusal')

    def sdf(*a, **k):
        seen['sdf'] += 1
        raise AssertionError('run_edges read the world past a refusal')

    monkeypatch.setattr(ew, 'read_inputs', read_inputs)
    monkeypatch.setattr(ej, 'edges_stage', stage)
    monkeypatch.setattr(gw, 'sdf_wall_faces', sdf)
    return seen


NOTHING = {'read_inputs': 0, 'stage': 0, 'sdf': 0}


def _git_ok(text=None, registered=None):
    """A clean tree, both registrations ancestors, and the specs at HEAD."""
    body = spec_text() if text is None else text
    schema = schema_spec_text()

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            rev, path = a[1].split(':', 1)
            if path != ej.SPEC_REL:
                return schema
            if registered is not None and rev == ej.EDGES_REGISTRATION:
                return registered
            return body
        if a[0] in ('merge-base', 'rev-parse'):
            return 'c0ffee' * 6
        return 'c0ffee'
    return fake


def _run(tmp_path):
    return ej.run_edges(tmp_path / 'out', tmp_path / 'results.json',
                        tmp_path / 'maps', tmp_path / 'graph', tmp_path)


def test_j4_refuses_a_dirty_tree(monkeypatch, tmp_path):
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', lambda *a: (
        ' M experiments/analysis/edges_judge.py' if a[0] == 'status'
        else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        _run(tmp_path)
    assert e.value.code == 2 and seen == NOTHING


def test_j4_refuses_when_the_registration_is_not_an_ancestor(monkeypatch,
                                                             tmp_path):
    seen, asked = _spies(monkeypatch), []

    def fake(*a):
        asked.append(a)
        return '' if a[0] in ('status', 'merge-base') else 'deadbeef'

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        _run(tmp_path)
    assert e.value.code == 2 and seen == NOTHING
    assert ('merge-base', ej.EDGES_REGISTRATION, 'HEAD') in asked
    assert ej.EDGES_REGISTRATION == '358650a'


@pytest.mark.parametrize('missing', ['### 5.2 Gate G-E1', '**P-E1.',
                                     '**P-E2.', '**P-E3.', '**P-E4.'])
def test_j4_refuses_a_spec_without_a_label(missing, monkeypatch, tmp_path):
    seen = _spies(monkeypatch)
    text = spec_text()
    assert text.count(missing) == 1, missing
    monkeypatch.setattr(dw, '_git', _git_ok(text.replace(missing, 'NOT THERE')))
    with pytest.raises(SystemExit) as e:
        _run(tmp_path)
    assert e.value.code == 2 and seen == NOTHING


def test_j4_refuses_sections_1_to_5_that_moved(monkeypatch, tmp_path):
    """J4, Q15: ancestry does not catch a later edit to §1-§5."""
    seen = _spies(monkeypatch)
    body = ej.registered_body(spec_text())
    assert body.startswith('## 1. Decisions') and '## 6.' not in body
    assert '### 5.3 Predictions' in body
    monkeypatch.setattr(dw, '_git', _git_ok(registered=spec_text().replace(
        '| E – W | 11.80 |', '| E – W | 11.90 |')))
    with pytest.raises(SystemExit) as e:
        _run(tmp_path)
    assert e.value.code == 2 and seen == NOTHING


@pytest.mark.parametrize('how', ['missing', 'wrong'])
def test_j4_refuses_a_missing_or_wrong_pinned_file(how, monkeypatch,
                                                   tmp_path):
    """J4: the results file and the F* file, each by its full sha256."""
    seen = _spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', _git_ok())
    if how == 'wrong':
        (tmp_path / 'results.json').write_text('{}')
        (tmp_path / ej.FSTAR_OUT).write_text('F*: 12 facing pairs\n')
        assert ej.SCHEMA_RESULTS_SHA256 != hashlib.sha256(b'{}').hexdigest()
    with pytest.raises(SystemExit) as e:
        _run(tmp_path)
    assert e.value.code == 2 and seen == NOTHING


# ───────────────────────── J5-J9: truth, items 2 and 3 ───────────────────────

TRUE_NAMES = ['E', 'N', 'S', 'W', 'maze_h1+y', 'maze_h1-y', 'maze_h2+y',
              'maze_h2-y', 'maze_v1+x', 'maze_v1-x', 'maze_v2+x', 'maze_v2-x']


def test_j5_the_truth_faces(truth):
    faces = ej.truth_faces(truth)
    assert [f['key'] for f in faces] == TRUE_NAMES
    e = faces[0]
    assert e['axis'] == '-x' and e['n_hat'] == (-1.0, 0.0)
    assert e['q0'] == pytest.approx((9.95, -9.95), abs=1e-12)
    assert e['q1'] == pytest.approx((9.95, 9.95), abs=1e-12)
    n = [ew.n_samples(f['L'], ej.TRUTH_RHO) for f in faces]
    assert n == [199] * 4 + [70] * 4 + [50] * 4 and sum(n) == 1276
    occ = ej.truth_occluders(truth, faces)
    assert [o['kind'] for o in occ] == ['segment'] * 12 + ['cap'] * 8
    assert occ[12]['key'] == [ew.CAP, 'maze_h1', 0]
    assert (occ[12]['p0'], occ[12]['p1']) == ([-6.0, 2.9], [-6.0, 3.1])


# From `edges_truth_fstar_out.txt`, sha256 `6dbc3354…`, its FACING PAIRS rows:
# S_i, S_j, |V_ij|, |V_ji|, width_m, gap_m, offset i->j, bearing i->j.
V01_ROWS = (
    ('E', 'W', 118, 118, '11.800', '19.9000', '0.0000', '0.000'),
    ('E', 'maze_v1+x', 30, 30, '3.000', '13.8500', '-1.5000', '-6.181'),
    ('E', 'maze_v2+x', 51, 50, '5.050', '5.8500', '1.5000', '14.381'),
    ('N', 'S', 78, 78, '7.800', '19.9000', '0.0000', '0.000'),
    ('N', 'maze_h1+y', 71, 70, '7.050', '6.8500', '2.5000', '20.050'),
    ('N', 'maze_h2+y', 50, 50, '5.000', '12.8500', '-2.5000', '-11.010'),
    ('S', 'maze_h1-y', 50, 50, '5.000', '12.8500', '-2.5000', '-11.010'),
    ('S', 'maze_h2-y', 71, 70, '7.050', '6.8500', '2.5000', '20.050'),
    ('W', 'maze_v1-x', 51, 50, '5.050', '5.8500', '1.5000', '14.381'),
    ('W', 'maze_v2-x', 30, 30, '3.000', '13.8500', '-1.5000', '-6.181'),
    ('maze_h1-y', 'maze_h2+y', 20, 20, '2.000', '5.8000', '-5.0000',
     '-40.764'),
    ('maze_v1+x', 'maze_v2-x', 20, 20, '2.000', '7.8000', '-3.0000',
     '-21.038'),
)
V01_THROUGH = [['E', 'maze_v1+x', 3], ['N', 'maze_h2+y', 3],
               ['S', 'maze_h1-y', 3], ['W', 'maze_v2-x', 3],
               ['maze_h1-y', 'S', 2], ['maze_h2+y', 'N', 2],
               ['maze_v1+x', 'E', 2], ['maze_v2-x', 'W', 2]]


def test_j6_without_caps_v0_1_is_reproduced(G01, constants):
    got = {(e['i'], e['j']): e['attrs']['n_ij'] + e['attrs']['n_ji']
           for e in G01['facing']}
    assert got == ej.fstar_samples(constants)            # whole samples
    assert G01['below_w_min'] == []
    assert G01['n_through'] == 20 and G01['through'] == V01_THROUGH
    assert sorted(c for _a, _b, c in G01['through']) == [2] * 4 + [3] * 4
    attrs = ej.fstar_of(G01)
    for i, j, nij, nji, width, gap, off, brg in V01_ROWS:
        a = attrs[(i, j)]
        assert (a['n_ij'], a['n_ji'], f'{a["width_m"]:.3f}',
                f'{a["gap_m"]:.4f}', f'{a["offset_m"]:.4f}',
                f'{a["bearing_deg"]:.3f}') == (nij, nji, width, gap, off,
                                               brg), (i, j)
    for name, t in G01['tally'].items():
        assert (t['blocked'], t['no_hit'], t['ties']) == (0, 0, 0), name
    ej.check_edges_truth(G01, constants)                 # J3's control


# The plan's §4.4, item 2: (|V_ij|, |V_ji|) under v0.2.
V02 = {('E', 'W'): (118, 118), ('E', 'maze_v1+x'): (27, 28),
       ('E', 'maze_v2+x'): (51, 50), ('N', 'S'): (78, 78),
       ('N', 'maze_h1+y'): (71, 70), ('N', 'maze_h2+y'): (47, 48),
       ('S', 'maze_h1-y'): (47, 48), ('S', 'maze_h2-y'): (71, 70),
       ('W', 'maze_v1-x'): (51, 50), ('W', 'maze_v2-x'): (27, 28),
       ('maze_h1-y', 'maze_h2+y'): (20, 20),
       ('maze_v1+x', 'maze_v2-x'): (20, 20)}
V02_WIDTHS = {('E', 'W'): 11.80, ('E', 'maze_v1+x'): 2.75,
              ('E', 'maze_v2+x'): 5.05, ('N', 'S'): 7.80,
              ('N', 'maze_h1+y'): 7.05, ('N', 'maze_h2+y'): 4.75,
              ('S', 'maze_h1-y'): 4.75, ('S', 'maze_h2-y'): 7.05,
              ('W', 'maze_v1-x'): 5.05, ('W', 'maze_v2-x'): 2.75,
              ('maze_h1-y', 'maze_h2+y'): 2.00,
              ('maze_v1+x', 'maze_v2-x'): 2.00}
V02_BLOCKED = {'E': 3, 'N': 3, 'S': 3, 'W': 3, 'maze_h1-y': 2,
               'maze_h2+y': 2, 'maze_v1+x': 2, 'maze_v2-x': 2}


def test_j7_item_2_truth_under_v0_2(G2, G01, constants):
    a2, a1 = ej.fstar_of(G2), ej.fstar_of(G01)
    assert {p: (a['n_ij'], a['n_ji']) for p, a in a2.items()} == V02
    for p, w in V02_WIDTHS.items():
        assert a2[p]['width_m'] == pytest.approx(w, abs=1e-9), p
        # Gaps, offsets and bearings are v0.1's.
        assert a2[p]['gap_m'] == pytest.approx(a1[p]['gap_m'], abs=1e-9), p
        assert a2[p]['offset_m'] == a1[p]['offset_m']
        assert a2[p]['bearing_deg'] == pytest.approx(a1[p]['bearing_deg'],
                                                     abs=1e-9)
    tally = G2['tally']
    assert {n: t['blocked'] for n, t in tally.items() if t['blocked']} == \
        V02_BLOCKED
    assert all(t['blocked_by']['cap'] == t['blocked'] for t in tally.values())
    assert sum(t['blocked'] for t in tally.values()) == 20
    assert sum(t['no_hit'] for t in tally.values()) == 0
    assert sum(t['ties'] for t in tally.values()) == 0
    assert G2['n_through'] == 0 and G2['through'] == []
    assert G2['components'] == {'all': 1, 'facing_and_slab': 2, 'facing': 2}
    g = ej.gate_2_truth(G2, constants)
    assert g['ok'] and g['pairs_equal'] and g['widths_equal']
    assert g['n_through_slab'] == 0 and g['missing'] == g['extra'] == []
    assert g['beside']['blocked_total'] == 20
    assert g['beside']['components_stated'] == {'all': 1,
                                                'facing_and_slab': 2}


@pytest.mark.parametrize('where', ['outside', 'inside'])
def test_j8_item_3_the_decoy(where, world, G2, G3):
    if where == 'outside':
        assert ej.DECOY == {'name': 'decoy', 'axis': '-x',
                            'q0': [20.0, -2.5], 'q1': [20.0, 2.5]}
        t = G3['tally']['decoy']
        assert t['samples'] == 50 and t['seen'] == {}
        assert t['blocked'] == t['blocked_by']['same_axis'] == 50
        g = ej.gate_3_decoy(G3, G2, ej.DECOY)
        assert g['ok'] and g['decoy_facing_edges'] == []
        assert g['other_edges_unchanged'] and g['tallies_unchanged']
    else:
        inside = {'name': 'decoy', 'axis': '+y', 'q0': [-0.5, 0.0],
                  'q1': [0.5, 0.0]}
        g = ej.gate_3_decoy(ej.truth_graph(world, ej.TRUTH_RHO, decoy=inside),
                            G2, inside)
        assert not g['ok']
        (e,) = g['decoy_facing_edges']
        assert (e['i'], e['j']) == ('decoy', 'maze_h1-y')
        a = e['attrs']
        assert (a['n_ij'], a['n_ji']) == (10, 10)
        assert a['width_m'] == pytest.approx(1.00, abs=1e-12)
        assert a['gap_m'] == pytest.approx(2.90, abs=1e-9)


def test_j9_item_2_fails_without_the_caps(G01, constants):
    g = ej.gate_2_truth(G01, constants)
    assert not g['ok'] and g['pairs_equal'] and not g['widths_equal']
    assert g['n_through_slab'] == 20 and not g['through_slab_ok']
    short = sorted(r['pair'] for r in g['pairs'] if not r['equal'])
    assert short == sorted(constants['drop']['pairs'])
    for r in g['pairs']:
        if not r['equal']:
            assert r['samples'] == r['v0_1_samples'] == \
                r['expected_samples'] + 5


# ───────────── hand-built per-map files, as schema run 1 holds them ──────────

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def zero_obs():
    return {'n_scans': 0, 'n_hits': 0, 'bearing_hist': [0] * 72,
            'incidence_hist': [0] * 9, 't_first': None, 't_last': None,
            'n_contradictions': 0}


def seg(sid, axis, a0, a1, at, ext=0.05):
    """One exactly axis-aligned face, as the schema and graph_walls files
    hold it: axis ends a0..a1 on the line y = at (±y) or x = at (±x)."""
    nx, ny = VEC[axis]
    u = (ny, -nx)
    pa, pb = (((a0, at), (a1, at)) if axis in ('+y', '-y')
              else ((at, a0), (at, a1)))
    q0, q1 = (pa, pb) if dot(pa, u) <= dot(pb, u) else (pb, pa)
    theta = (math.degrees(math.atan2(ny, nx)) - 90.0) % 360.0
    schema = {'id': sid, 'line': sid, 'theta': theta, 'normal': [nx, ny],
              'd': dot(q0, (nx, ny)), 'axis': AXIS[axis], 'axis_name': axis,
              'n_members': 10, 'n_axis_members': 10, 'n_risers': 0,
              'n_opposite': 0, 'no_axis_ends': None,
              'axis_ends': {'t': [dot(q0, u), dot(q1, u)], 'q0': list(q0),
                            'q1': list(q1),
                            'm': [(q0[0] + q1[0]) / 2.0,
                                  (q0[1] + q1[1]) / 2.0],
                            'states': ['free', 'free']}}
    graph = {'id': sid, 'p0': [q0[0] - ext * u[0], q0[1] - ext * u[1]],
             'p1': [q1[0] + ext * u[0], q1[1] + ext * u[1]], 'theta': theta,
             'normal': [nx, ny], 'length': abs(a1 - a0) + 2 * ext,
             'obs': zero_obs()}
    return schema, graph


def corner(a, b, c):
    return {'segments': [a, b], 'c': list(c), 'near_end': ['q0', 'q1'],
            'near_m': [0.01, 0.02], 'states': ['occupied', 'occupied']}


def end(points):
    pts = [list(points[0]), list(points[1])]
    return {'axis_ends': [[4, 'q0'], [5, 'q1']], 'points': pts, 'diff_m': 0.0,
            'aligned': True, 'state': 'free', 'states': ['free', 'free'],
            'record': 'slab_end', 'slab_end': True, 'outside_corners': pts}


def map_files(stem, rho=RHO, origin=(50.0, 50.0)):
    """A box of four faces with its corners, and a slab beside it, far from
    every world face: both files of one row, through JSON as read."""
    ox, oy = origin
    segs = [seg(0, '+y', ox, ox + 4.0, oy), seg(1, '-y', ox, ox + 4.0, oy + 3.0),
            seg(2, '+x', oy, oy + 3.0, ox), seg(3, '-x', oy, oy + 3.0, ox + 4.0),
            seg(4, '-y', ox + 10.0, ox + 12.0, oy),
            seg(5, '+y', ox + 10.0, ox + 12.0, oy + 0.2)]
    corners = [corner(0, 2, (ox, oy)), corner(0, 3, (ox + 4.0, oy)),
               corner(1, 2, (ox, oy + 3.0)), corner(1, 3, (ox + 4.0, oy + 3.0))]
    pairs = [{'segments': [4, 5], 'separation_m': 0.2, 'overlap_m': 2.0,
              'ends': [end([(ox + 12.0, oy), (ox + 12.0, oy + 0.2)]),
                       end([(ox + 10.0, oy), (ox + 10.0, oy + 0.2)])]}]
    schema = {'map': stem, 'map_sha256': 'ab' * 32,
              'graph_walls_commit': 'dc5e2a3', 'spec': sw.SPEC,
              'frame': 'robot_0 map', 'positions': 'the map frame',
              'thresholds': sw.thresholds(rho, P),
              'counts': {'segments': len(segs)},
              'segments': [s for s, _g in segs], 'inside_corners': corners,
              'pairs': pairs, 'gaps': []}
    graph = {'map': stem, 'map_sha256': 'ab' * 32, 'frame': 'robot_0 map',
             'segments': [g for _s, g in segs]}
    return json.loads(json.dumps(schema)), json.loads(json.dumps(graph))


def corpus():
    return {stem: map_files(stem) for stem in STEMS}


def write_corpus(where, files):
    """The 20 rows on disk, and a results file giving each schema file's
    sha256, as schema run 1's does."""
    maps, graphs = where / 'maps', where / 'graph'
    maps.mkdir(parents=True)
    graphs.mkdir()
    per = []
    for stem, (schema, graph) in files.items():
        sp, gp = maps / f'{stem}.json', graphs / f'{stem}.json'
        sp.write_text(json.dumps(schema, indent=2))
        gp.write_text(json.dumps(graph, indent=2))
        per.append({'map': stem, 'file': sp.name,
                    'sha256': hashlib.sha256(sp.read_bytes()).hexdigest(),
                    'counts': schema['counts']})
    results = where / 'results.json'
    results.write_text(json.dumps({'per_map_files': per}))
    return results, maps, graphs


# ───────────────────────── J10-J13: item 1, the inputs, the order ────────────

@pytest.mark.parametrize('change', ['none', 'corner_dropped', 'slab_added'])
def test_j10_item_1_is_one_to_one(change, constants):
    files = corpus()
    schema = {s: f[0] for s, f in files.items()}
    records = {s: json.loads(json.dumps(ew.edges_for_map(f[0], f[1], PROV)))
               for s, f in files.items()}
    stem = STEMS[6]
    if change == 'corner_dropped':
        records[stem]['edges'] = [e for e in records[stem]['edges']
                                  if (e['type'], e['i'], e['j'])
                                  != ('corner', 0, 2)]
    elif change == 'slab_added':
        slab = [e for e in records[stem]['edges'] if e['type'] == 'slab'][0]
        records[stem]['edges'].append(dict(slab, i=0, j=1))
    g = ej.gate_1_structure(records, schema, constants)
    if change == 'none':
        assert g['ok'] and g['n_maps_equal'] == 20 and g['differing'] == []
        assert g['maps'][0]['corner'] == {'schema': 4, 'edges': 4,
                                          'equal': True}
        assert g['maps'][0]['slab'] == {'schema': 1, 'edges': 1, 'equal': True}
    else:
        assert not g['ok'] and g['n_maps_equal'] == 19
        assert [m['map'] for m in g['differing']] == [stem]


@pytest.mark.parametrize('change', ['sha256', 'map_sha256', 'rho'])
def test_j11_the_inputs_refuse(change, tmp_path):
    got = ej.load_inputs(*write_corpus(tmp_path / 'clean', corpus()), STEMS)
    assert sorted(got['schema']) == STEMS and got['rho'] == RHO
    files, stem = corpus(), STEMS[4]
    if change == 'map_sha256':
        files[stem][1]['map_sha256'] = 'cd' * 32
    elif change == 'rho':
        files[stem][0]['thresholds']['rho'] = 2 * RHO
    results, maps, graphs = write_corpus(tmp_path / 'bad', files)
    if change == 'sha256':
        doc = json.loads(results.read_text())
        doc['per_map_files'][4]['sha256'] = '0' * 64
        results.write_text(json.dumps(doc))
    with pytest.raises(SystemExit):
        ej.load_inputs(results, maps, graphs, STEMS)


def j12_stage(tmp_path, monkeypatch, verbatim, constants, schema_constants,
              world):
    """The stage on 20 hand-built rows, counting input loads and M1 calls."""
    calls = {'inputs': 0, 'm1': 0}
    real = sj.match_faces

    def m1(*a, **k):
        calls['m1'] += 1
        return real(*a, **k)

    monkeypatch.setattr(sj, 'match_faces', m1)
    results, maps, graphs = write_corpus(tmp_path / 'in', corpus())

    def inputs_of():
        calls['inputs'] += 1
        inputs = ej.load_inputs(results, maps, graphs, STEMS)
        inputs['checks'] = {'rho': RHO, 'tau_m': 2 * RHO}
        return inputs

    meta = ej.edges_meta('synthetic', 'synthetic', 'cd' * 32, ['?? .vscode/'],
                         verbatim, constants, schema_constants,
                         {'spec': {'sha256': 'cd' * 32}}, None)
    out_dir = tmp_path / 'out'

    def run():
        return ej.edges_stage(world, [ZERO] * 5, constants, meta, inputs_of,
                              out_dir, out_dir / 'maps', 'c0ffee1')
    return run, calls, out_dir


def j12_failed(out_dir, capsys):
    """The stage stopped at exit 2, with one fail file and no verdict."""
    written = list(out_dir.glob('edges_gate_fail_*.json'))
    assert len(written) == 1
    assert not list(out_dir.glob('edges_judge_*.json'))
    payload = json.loads(written[0].read_text())
    for key in ('judgments', 'verdicts', 'reported'):
        assert key not in payload
    assert set(payload['spec_verbatim']['items']) >= set(ej.PREDICTIONS)
    out = capsys.readouterr().out
    for name in ej.PREDICTIONS:
        assert f'{name}:' not in out
    return payload


def test_j12_item_2_failing_reads_no_map(tmp_path, capsys, monkeypatch,
                                         verbatim, constants,
                                         schema_constants, world):
    item3 = []
    monkeypatch.setattr(ej, 'gate_2_truth',
                        lambda *a: {'ok': False, 'why': 'planted'})
    monkeypatch.setattr(ej, 'gate_3_decoy',
                        lambda *a: item3.append(a) or {'ok': True})
    run, calls, out_dir = j12_stage(tmp_path, monkeypatch, verbatim,
                                    constants, schema_constants, world)
    with pytest.raises(SystemExit) as e:
        run()
    assert e.value.code == 2
    payload = j12_failed(out_dir, capsys)
    assert payload['G-E1']['parts'] == {'1_structure': None,
                                        '2_truth': False, '3_decoy': None}
    assert calls == {'inputs': 0, 'm1': 0} and item3 == []
    assert payload['per_map_files'] == [] and payload['n_door_calls'] == 0
    assert not (out_dir / 'maps').exists()


def test_j12_item_3_failing_reads_no_map(tmp_path, capsys, monkeypatch,
                                         verbatim, constants,
                                         schema_constants, world):
    monkeypatch.setattr(ej, 'gate_3_decoy',
                        lambda *a: {'ok': False, 'why': 'planted'})
    run, calls, out_dir = j12_stage(tmp_path, monkeypatch, verbatim,
                                    constants, schema_constants, world)
    with pytest.raises(SystemExit) as e:
        run()
    assert e.value.code == 2
    payload = j12_failed(out_dir, capsys)
    assert payload['G-E1']['parts'] == {'1_structure': None,
                                        '2_truth': True, '3_decoy': False}
    assert calls == {'inputs': 0, 'm1': 0}
    assert not (out_dir / 'maps').exists()


def test_j12_item_1_failing_writes_the_files_and_judges_nothing(
        tmp_path, capsys, monkeypatch, verbatim, constants, schema_constants,
        world):
    monkeypatch.setattr(ej, 'gate_1_structure',
                        lambda *a: {'ok': False, 'why': 'planted'})
    run, calls, out_dir = j12_stage(tmp_path, monkeypatch, verbatim,
                                    constants, schema_constants, world)
    with pytest.raises(SystemExit) as e:
        run()
    assert e.value.code == 2
    payload = j12_failed(out_dir, capsys)
    assert payload['G-E1']['parts'] == {'1_structure': False,
                                        '2_truth': True, '3_decoy': True}
    assert calls == {'inputs': 1, 'm1': 0}
    assert len(payload['per_map_files']) == 20
    assert len(list((out_dir / 'maps').glob('*.json'))) == 20
    assert payload['n_door_calls'] == 0


def test_j12_all_three_holding_writes_one_results_file(
        tmp_path, capsys, monkeypatch, verbatim, constants, schema_constants,
        world):
    """The rows lie far from every world face: every facing edge is
    unmatched, so P-E4 fails on all five maps and P-E3 holds."""
    run, calls, out_dir = j12_stage(tmp_path, monkeypatch, verbatim,
                                    constants, schema_constants, world)
    out = run()
    assert calls == {'inputs': 1, 'm1': 20}
    assert out['G-E1']['all_ok'] is True
    written = list(out_dir.glob('edges_judge_*.json'))
    assert len(written) == 1 and not list(out_dir.glob('edges_gate_fail_*'))
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()
    maps = sorted((out_dir / 'maps').glob('*.json'))
    assert [p.stem for p in maps] == STEMS
    for f in out['per_map_files']:
        assert f['sha256'] == hashlib.sha256(
            (out_dir / 'maps' / f['file']).read_bytes()).hexdigest()
    payload = json.loads(written[0].read_text())
    assert payload['stage'] == 'edges_judge' and payload['n_maps'] == 20
    assert sorted(payload['verdicts']) == list(ej.PREDICTIONS)
    assert [j['map'] for j in payload['judgments']] == FULL_STEMS
    v = payload['verdicts']
    assert not v['P-E4']['holds'] and v['P-E4']['n_maps_with'] == 5
    assert v['P-E3']['holds'] and v['P-E3']['n_maps_with'] == 0
    assert not v['P-E2']['holds'] and v['P-E2']['n_holding'] == 0
    assert len(payload['reported']['per_map']) == 20
    assert 'P-E4: FAIL' in capsys.readouterr().out
    with pytest.raises(AssertionError):               # not a gate payload
        ej._no_verdict_in_edges(payload)


# ─────────────── hand-built records on the true faces, for M1 ────────────────

def on_face(sid, name, T, perp=0.0, shift=ZERO, span=None):
    """A segment on true face `name`, 0.5 m in from each end unless `span`
    gives its interval, `perp` off its line, in a robot frame at `shift`."""
    f = T['faces'][name]
    a0, a1 = (f['lo'] + 0.5, f['hi'] - 0.5) if span is None else span
    v = f['v'] + perp
    q0, q1 = ([a0, v], [a1, v]) if f['ax'] == 'y' else ([v, a0], [v, a1])
    q0 = [q0[0] - shift[0], q0[1] - shift[1]]
    q1 = [q1[0] - shift[0], q1[1] - shift[1]]
    return {'id': sid, 'line': sid, 'axis': f['axis'], 'no_axis_ends': None,
            'axis_ends': {'t': [0.0, 1.0], 'q0': q0, 'q1': q1,
                          'm': [(q0[0] + q1[0]) / 2.0, (q0[1] + q1[1]) / 2.0],
                          'states': ['free', 'free']}}


def edge_record(stem, ids, facing=(), corner=(), slab=()):
    """A per-map edges record with the given edges, its counts consistent."""
    edges = []
    for kind, pairs in (('corner', corner), ('slab', slab),
                        ('facing', facing)):
        for a, b in pairs:
            attrs = ({'width_m': 1.0, 'gap_m': 2.0, 'n_ij': 5, 'n_ji': 5,
                      'offset_m': 0.0, 'bearing_deg': 0.0}
                     if kind == 'facing' else {})
            edges.append({'type': kind, 'i': min(a, b), 'j': max(a, b),
                          'attrs': attrs})
    ids = sorted(ids)
    comps = ew.components(ids, [(e['i'], e['j']) for e in edges])
    return {'map': stem, 'nodes': [{'id': i} for i in ids], 'edges': edges,
            'counts': {'components': len(comps)},
            'diagnostics': {'per_face': [
                {'id': i, 'samples': 10, 'blocked': 0, 'blocked_by': {},
                 'no_hit': 0} for i in ids]}}


def twelve(T):
    """One segment on each of the 12 true faces, id k on name k."""
    names = sorted(T['faces'])
    return ({n: k for k, n in enumerate(names)},
            {'map': STEMS[0], 'segments': [on_face(k, n, T)
                                           for k, n in enumerate(names)]})


def test_j13_the_door_raises_until_g_e1_holds(truth, fstar):
    stem = STEMS[0]
    door = ej.EdgesDoor({stem: edge_record(stem, [0])},
                        {stem: {'map': stem,
                                'segments': [on_face(0, 'N', truth)]}},
                        [ZERO] * 5, truth, TAU_M, fstar)
    with pytest.raises(ej.EdgesGateClosed):
        door.judge(stem)
    ok = {'ok': True}
    for items in ((ok, ok, {'ok': False}), (ok, ok, None), (ok, ok),
                  (ok, ok, {'ok': 1}), (None, ok, ok)):
        assert not door.open_after(*items)
        with pytest.raises(ej.EdgesGateClosed):
            door.judge(stem)
    assert door.asked == [] and not door.is_open
    assert door.open_after(ok, ok, ok) and door.is_open
    assert door.judge(stem)['faces'] == {0: 'N'}
    assert door.asked == [stem]


@pytest.mark.parametrize('dropped, n', [(0, 12), (1, 11), (2, 10)])
def test_j14_p_e2_counts_the_f_star_pairs(dropped, n, truth, fstar):
    at, srec = twelve(truth)
    pairs = [(at[a], at[b]) for a, b in sorted(fstar)][dropped:]
    j = ej.judge_edges(edge_record(STEMS[0], range(12), pairs), srec, truth,
                       ZERO, TAU_M, fstar)
    assert sorted(j['faces'].values()) == TRUE_NAMES
    assert len(j['recovered_pairs']) == n
    assert j['wrong_edges'] == [] and j['unmatched_edges'] == []
    assert len(fstar) == 12
    if n == 12:
        assert j['recovered_pairs'] == sorted(list(p) for p in fstar)


def test_j15_a_split_face_counts_its_pair_once(truth, fstar):
    srec = {'map': STEMS[0], 'segments': [
        on_face(0, 'N', truth),
        on_face(1, 'maze_h1+y', truth, span=(-5.5, -3.0)),
        on_face(2, 'maze_h1+y', truth, span=(-2.5, 0.5))]}
    j = ej.judge_edges(edge_record(STEMS[0], [0, 1, 2], [(0, 1), (0, 2)]),
                       srec, truth, ZERO, TAU_M, fstar)
    assert j['faces'] == {0: 'N', 1: 'maze_h1+y', 2: 'maze_h1+y'}
    assert j['recovered_pairs'] == [['N', 'maze_h1+y']]
    assert [(e['i'], e['j'], e['pair']) for e in j['fstar_edges']] == [
        (0, 1, ['N', 'maze_h1+y']), (0, 2, ['N', 'maze_h1+y'])]
    t = fstar[('N', 'maze_h1+y')]
    assert j['fstar_edges'][0]['width_minus_truth_m'] == pytest.approx(
        1.0 - t['width_m'], abs=1e-12)
    assert j['fstar_edges'][0]['gap_minus_truth_m'] == pytest.approx(
        2.0 - t['gap_m'], abs=1e-12)


def test_j16_an_edge_between_faces_not_in_f_star_is_wrong(truth, fstar):
    srec = {'map': STEMS[0], 'segments': [on_face(0, 'E', truth),
                                          on_face(1, 'maze_v1-x', truth)]}
    j = ej.judge_edges(edge_record(STEMS[0], [0, 1], [(0, 1)]), srec, truth,
                       ZERO, TAU_M, fstar)
    assert j['wrong_edges'] == [{'i': 0, 'j': 1, 'faces': ['E', 'maze_v1-x']}]
    assert j['recovered_pairs'] == [] and j['unmatched_edges'] == []


def test_j17_an_edge_with_an_unmatched_face_is_unmatched(truth, fstar):
    far = on_face(1, 'maze_h1+y', truth, perp=27.0)          # y = 30.1
    srec = {'map': STEMS[0], 'segments': [on_face(0, 'N', truth), far]}
    j = ej.judge_edges(edge_record(STEMS[0], [0, 1], [(0, 1)]), srec, truth,
                       ZERO, TAU_M, fstar)
    assert j['faces'] == {0: 'N', 1: None}
    assert j['unmatched_edges'] == [{'i': 0, 'j': 1, 'faces': ['N', None]}]
    assert j['wrong_edges'] == [] and j['recovered_pairs'] == []


def test_j18_p_e1_without_corner_edges_two_components(truth, fstar):
    at, srec = twelve(truth)
    facing = [(at[a], at[b]) for a, b in fstar]
    slab = [(at[s['faces'][0]], at[s['faces'][1]]) for s in truth['slabs']]
    corner_ = [(at[c['faces'][0]], at[c['faces'][1]])
               for c in truth['corners']]
    no = ej.judge_edges(edge_record(STEMS[0], range(12), facing, slab=slab),
                        srec, truth, ZERO, TAU_M, fstar)
    assert no['components'] == 2
    yes = ej.judge_edges(edge_record(STEMS[0], range(12), facing,
                                     corner=corner_, slab=slab),
                         srec, truth, ZERO, TAU_M, fstar)
    assert yes['components'] == 1


# ───────────────────── J19-J23: the verdicts and the files ───────────────────

def rows_(**edits):
    """One row per map: one component, all 12 pairs, nothing spurious."""
    out = []
    for stem in STEMS:
        k, cut = gw._k_and_cut(stem, 'the test rows')
        r = {'map': stem, 'robot': k, 'cut': cut, 'components': 1,
             'n_facing': 14, 'n_recovered': 12, 'n_wrong': 0,
             'n_unmatched': 0}
        r.update(edits.get(stem, {}))
        out.append(r)
    return out


@pytest.mark.parametrize('name, edit, n_maps, holds', [
    ('P-E1', {'components': 2}, 1, True), ('P-E1', {'components': 2}, 2, False),
    ('P-E2', {'n_recovered': 10}, 1, True),
    ('P-E2', {'n_recovered': 10}, 2, False),
    ('P-E3', {'n_wrong': 1}, 1, True), ('P-E3', {'n_wrong': 1}, 2, False),
    ('P-E4', {'n_unmatched': 3}, 1, True),
    ('P-E4', {'n_unmatched': 3}, 2, False)])
def test_j19_the_bounds(name, edit, n_maps, holds, constants):
    """P-E1 and P-E2 at 4 and 3 maps of 5; P-E3 and P-E4 at 1 and 2."""
    edits = {s: edit for s in FULL_STEMS[:n_maps]}
    if name == 'P-E2':
        edits[FULL_STEMS[4]] = {'n_recovered': 11}       # at 11 of 12, it holds
    v = ej.verdicts_from(rows_(**edits), constants)
    assert v[name]['holds'] is holds
    assert all(v[n]['holds'] for n in ej.PREDICTIONS if n != name)
    if name in ('P-E1', 'P-E2'):
        assert v[name]['n_holding'] == 5 - n_maps
    else:
        assert v[name]['n_maps_with'] == n_maps


def test_j20_the_verdicts_use_only_the_five_cut1200_rows(constants):
    early = [s for s in STEMS if s not in FULL_STEMS]
    rows = rows_(**{s: {'components': 3, 'n_recovered': 0, 'n_wrong': 2,
                        'n_unmatched': 2} for s in early})
    v = ej.verdicts_from(rows, constants)
    assert all(v[n]['holds'] for n in ej.PREDICTIONS)
    assert v['P-E1']['n_holding'] == 5 and v['P-E3']['n_maps_with'] == 0
    assert [m['map'] for m in v['P-E1']['maps']] == FULL_STEMS
    split = ej.split_rows(rows, constants)
    assert [r['map'] for r in split['judged']] == FULL_STEMS
    assert sorted(split['reported']) == ['120', '240', '60']
    assert sorted(r['map'] for c in split['reported'].values() for r in c) == \
        sorted(early)
    with pytest.raises(SystemExit):                  # a cut1200 row missing
        ej.verdicts_from([r for r in rows if r['map'] != FULL_STEMS[0]],
                         constants)


def test_j21_m1_through_the_door_with_the_robots_spawn(truth, fstar):
    """M0: a point in robot K's frame lies at p + s_K in the world."""
    stem = 'b2maps_k1_cut1200_gated_extfix_robot1'
    assert stem in STEMS
    spawn = [ZERO, (0.86, 0.28), ZERO, ZERO, ZERO]
    ok = {'ok': True}
    for perp, face in ((-0.19, 'N'), (-0.21, None)):
        srec = {'map': stem, 'segments': [
            on_face(0, 'N', truth, perp=perp, shift=spawn[1],
                    span=(0.0, 2.0))]}
        door = ej.EdgesDoor({stem: edge_record(stem, [0])}, {stem: srec},
                            spawn, truth, TAU_M, fstar)
        door.open_after(ok, ok, ok)
        assert door.judge(stem)['faces'] == {0: face}, perp
    # Read without the spawn, the 0.19 m segment lies 0.47 m off N.
    srec = {'map': stem, 'segments': [on_face(0, 'N', truth, perp=-0.19,
                                              shift=spawn[1],
                                              span=(0.0, 2.0))]}
    assert ej.judge_edges(edge_record(stem, [0]), srec, truth, ZERO, TAU_M,
                          fstar)['faces'] == {0: None}


@pytest.mark.parametrize('planted', [False, True])
def test_j22_the_fail_walker(planted):
    payload = {'stage': 'edges_judge_gate_fail',
               'G-E1': {'2_truth': {'ok': False,
                                    'beside': {'components': {'all': 1}}},
                        '3_decoy': None, '1_structure': None},
               'spec_verbatim': {'items': {'P-E1': 'text'}},
               'constants': {'p-e1': {'min': 4, 'of': 5}}}
    if not planted:
        ej._no_verdict_in_edges(payload)
        return
    for bad in ({'verdicts': {}}, {'judgments': []},
                {'G-E1': {'deep': [{'P-E3': 1}]}}, {'reported': {}},
                {'per_map_files': [{'recovered_pairs': []}]}):
        with pytest.raises(AssertionError):
            ej._no_verdict_in_edges(dict(payload, **bad))


def test_j23_the_results_file_round_trips_and_is_never_overwritten(
        tmp_path, monkeypatch):
    class Fixed(datetime):
        @classmethod
        def now(cls, tz=None):
            return datetime(2026, 10, 19, 12, 0, 0, tzinfo=timezone.utc)

    monkeypatch.setattr(dw, 'datetime', Fixed)
    payload = {'stage': 'edges_judge', 'verdicts': {'P-E1': {'holds': True}},
               'judgments': [{'map': STEMS[0], 'n_recovered': 12}],
               'faces': {'0': 'N', '1': None}}
    path, sha = ej._write_edges(payload, tmp_path, 'edges_judge')
    assert path.name == 'edges_judge_20261019T120000Z.json'
    assert sha == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads(path.read_text()) == payload
    before = path.read_bytes()
    with pytest.raises(SystemExit):
        ej._write_edges(payload, tmp_path, 'edges_judge')
    assert path.read_bytes() == before
