"""Layer 1 — merging coincident analytic candidates in
experiments/slam/fit_world_transform.py.

The fitter scores four analytic candidates: two PGM row conventions crossed
with two directions of map->odom. The two directions are different
compositions, but they are not always different PLACEMENTS: world_T_odom o
map_T_odom and world_T_odom o inverse(map_T_odom) land on the same spot
whenever map_T_odom is its own inverse. Counting that placement twice used to
produce two winners and a verdict of AMBIGUOUS blaming world symmetry, which
was false -- there was one placement, not two the wall overlap could not
separate.

These tests pin the predicate that fix rests on. It must be a test of the
resulting placement, not of "map->odom is identity": identity is merely the
common involution, and a 180 deg rotation is one too.

The script lives in experiments/, which is not an importable package (nor a
colcon one), so it is loaded by location the way test_preflight_motion.py loads
preflight_motion.py. Importing it runs module-level code only -- numpy, yaml
and bag_overlap, which defers its rosbag2_py import into the two functions that
read a bag. No ROS environment is needed.

scipy and PIL are NOT module-level there; the fitter defers them into load_map,
coarse_search and render precisely so that importing it costs nothing, and the
comment at their import site says so. Deferring moves the failure out of
collection and into whichever test makes the call -- it does not remove it. So
a test that reaches one of those three needs that library at RUN time and is
marked accordingly. Run 35539327xxx is what that costs when it is not: CI's
ros:jazzy container has no scipy, and the one test here that calls
coarse_search failed on the import at fit_world_transform.py:431.

Everything else in this file is pure arithmetic over dicts and small numpy
arrays -- the merge predicate, the candidate gate, the cross-convention
reference the gate reads, and the verdict report() forms out of them -- and
MUST keep running in CI. Those are the tests that catch the defects this file
exists for; skipping the module wholesale to dodge one import would retire them.
"""

import importlib
import importlib.util
import os

import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
FITTER = os.path.join(REPO_ROOT, 'experiments', 'slam', 'fit_world_transform.py')


def load_fitter():
    spec = importlib.util.spec_from_file_location('fit_world_transform', FITTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _importable(name: str) -> bool:
    """True if `name` can actually be imported here.

    A real import attempt, not importlib.util.find_spec: find_spec answers
    "is there something on the path called this", which is not the same
    question as "will the deferred import inside coarse_search succeed". A
    half-installed scipy, or one shadowed by a meta_path blocker, satisfies
    the first and fails the second -- and the second is the one that decides
    whether the test can run. Costs one import of an already-imported module
    everywhere the library IS present.
    """
    try:
        importlib.import_module(name)
    except ImportError:
        return False
    return True


# scipy is absent from CI's ros:jazzy container by design (see the deferred
# import comment in fit_world_transform.py). This marker is the other half of
# that arrangement: the import is deferred so the MODULE loads, and the test
# that forces it is skipped so the SUITE passes. Only tests reaching
# coarse_search, load_map or render need it -- everything else here is
# numpy-only and must keep running.
needs_scipy = pytest.mark.skipif(
    not _importable('scipy'),
    reason='scipy not installed — coarse_search imports it at call time '
           '(fit_world_transform.py:431)')


fitter = load_fitter()
placements_coincide = fitter.placements_coincide
merge_groups = fitter.merge_groups

MAP_T_ODOM = 'world_T_odom o map_T_odom'
INVERSE = 'world_T_odom o inverse(map_T_odom)'


def placement(label, theta, dx, dy):
    """The only fields the merge predicate reads."""
    return {'label': label, 'theta': theta, 'dx': dx, 'dy': dy}


# ── the identity case: map->odom identity, so both directions coincide ───────
#
# Arm B of the 2026-09-19 2x2. With the matcher off, map->odom stays identity,
# so both compositions reduce to world_T_odom and the fitter printed the same
# placement twice: 96.1% at theta -179.00, dx +1.544, dy -0.425.

IDENTITY_CASE = [
    placement(MAP_T_ODOM, -179.00, 1.544, -0.425),
    placement(INVERSE, -179.00, 1.544, -0.425),
]


def test_identical_placements_coincide():
    assert placements_coincide(*IDENTITY_CASE)


def test_identity_case_merges_into_one_group():
    groups = merge_groups(IDENTITY_CASE)
    assert groups == [[MAP_T_ODOM, INVERSE]]


def test_merged_group_is_counted_once():
    """The defect itself: two entries, one placement."""
    assert len(IDENTITY_CASE) == 2
    assert len(merge_groups(IDENTITY_CASE)) == 1


# ── the arm C case: map->odom is 0.98 m and 0.44 deg from identity ───────────
#
# The matcher on truth poses drifted map->odom off identity, so the two
# directions are genuinely different placements and must survive as two.

ARM_C_CASE = [
    placement(MAP_T_ODOM, -176.24, 3.724, -0.345),
    placement(INVERSE, -177.16, 1.762, -0.490),
]


def test_arm_c_placements_do_not_coincide():
    assert not placements_coincide(*ARM_C_CASE)


def test_arm_c_stays_two_groups():
    groups = merge_groups(ARM_C_CASE)
    assert groups == [[MAP_T_ODOM], [INVERSE]]


# ── the tolerance is a duplicate test, not an agreement test ─────────────────

def test_half_a_cell_and_a_tenth_of_a_degree():
    assert fitter.MERGE_XY_M == pytest.approx(0.5 * fitter.COARSE_CELL)
    assert fitter.MERGE_THETA_DEG == pytest.approx(0.1)


def test_merge_tolerance_is_far_tighter_than_agreement():
    """A 0.25 m agreement threshold would have swallowed arm C's 0.98 m."""
    assert fitter.MERGE_XY_M < fitter.AGREE_XY_M
    assert fitter.MERGE_THETA_DEG < fitter.AGREE_THETA_DEG


@pytest.mark.parametrize('dx, dy, dtheta, expected', [
    (0.049, 0.0, 0.0, True),      # inside the translation tolerance
    (0.051, 0.0, 0.0, False),     # outside it
    (0.0, 0.051, 0.0, False),     # y is checked too, not only x
    (0.0, 0.0, 0.09, True),       # inside the angular tolerance
    (0.0, 0.0, 0.11, False),      # outside it
])
def test_tolerance_edges(dx, dy, dtheta, expected):
    a = placement(MAP_T_ODOM, 10.0, 1.0, 2.0)
    b = placement(INVERSE, 10.0 + dtheta, 1.0 + dx, 2.0 + dy)
    assert placements_coincide(a, b) is expected


# ── coincidence, not "map->odom is identity" ─────────────────────────────────

def test_a_180_degree_twin_coincides_with_itself():
    """A 180 deg rotation is its own inverse, so it duplicates exactly as
    identity does. Testing for identity instead of for coincidence would miss
    this and go on reporting AMBIGUOUS."""
    both = [placement(MAP_T_ODOM, 180.0, -2.5, 0.75),
            placement(INVERSE, -180.0, -2.5, 0.75)]
    assert placements_coincide(*both)              # +180 and -180 are one angle
    assert merge_groups(both) == [[MAP_T_ODOM, INVERSE]]


def test_theta_wraps_across_the_seam():
    a = placement(MAP_T_ODOM, 179.98, 0.0, 0.0)
    b = placement(INVERSE, -179.98, 0.0, 0.0)
    assert placements_coincide(a, b)


def test_phase_b_run1_style_transforms_never_merge():
    """The five phaseB_run1 map->odom values are non-involutions with
    metre-scale translations. Nothing there may merge, or the tolerance has
    been opened wide enough to invent agreement."""
    cands = [placement(MAP_T_ODOM, 105.98, 2.118, -3.413),
             placement(INVERSE, -105.98, -2.118, 3.413)]
    assert not placements_coincide(*cands)
    assert len(merge_groups(cands)) == 2


# ── Layer 2 — the candidate gate, and the peak list it is read against ───────
# The verdict used to compare (theta, dx, dy) against the nearest of five
# coarse peaks at 1 deg and 0.25 m. Two faults made that unachievable rather
# than strict: the peak list could be missing the very placement under test
# (translation-space NMS deletes a 180 deg twin that needs the same
# translation), and 1 deg is finer than the on-wall objective resolves, which
# the self-check already documents about itself as a plateau.
#
# So a candidate is now judged on its own score: within CANDIDATE_SCORE_TOL of
# the best FREE fit, and at or above WELL_FIT_FLOOR. Angle, offset, peak rank
# and plateau are reported and never gated.

candidate_gate = fitter.candidate_gate
free_optimum = fitter.free_optimum
coarse_search = fitter.coarse_search


def test_a_candidate_level_with_the_best_free_fit_passes():
    """b2maps_e0t_kp and b2maps_e0_kp: the spawn candidate scores EXACTLY what
    the free fit scores, to the last float digit. That must resolve.
    """
    g = candidate_gate(0.9554973821989529, 0.9554973821989529)
    assert g['pass'] is True
    assert g['score_gap'] == 0.0


def test_a_candidate_five_points_below_the_best_does_not_resolve():
    """The case the score gate must not wave through: a candidate on a map
    that fits the world well, but landing somewhere materially worse than the
    free search found. 5 pp is five times the tolerance.
    """
    g = candidate_gate(0.905, 0.955)
    assert g['pass'] is False
    assert g['within_tol'] is False
    assert g['clears_floor'] is True       # the failure is the gap, not the floor
    assert g['score_gap'] == pytest.approx(-0.05)


def test_the_tolerance_edge_is_inclusive_and_one_step_past_it_is_not():
    """Exactly one tolerance below the best still passes.

    Written against the arithmetic edge on purpose: 0.955 - 0.01 is not
    representable, so the gate carries 1e-9 of slack -- the same slack the
    self-check puts on its plateau bounds. Without it the edge would reject on
    float representation rather than on the criterion.
    """
    tol = fitter.CANDIDATE_SCORE_TOL
    best = 0.955
    assert candidate_gate(best - tol, best)['pass'] is True
    assert candidate_gate(best - tol - 1e-6, best)['pass'] is False


def test_matching_a_bad_free_fit_is_not_a_pass():
    """b18_run2 arm A: the odometry map free-fits at 21.9%, so a candidate
    that matches it matches noise. The floor is what stops that being called
    agreement, and it is why the gate is two conditions rather than one.
    """
    g = candidate_gate(0.219, 0.219)
    assert g['within_tol'] is True
    assert g['clears_floor'] is False
    assert g['pass'] is False


def test_a_candidate_above_the_best_passes():
    """Seeded refinement can land the candidate a hair ABOVE the free search's
    lattice-quantised optimum. A positive gap is agreement, not a failure.
    """
    assert candidate_gate(0.962, 0.955)['pass'] is True


def test_the_free_optimum_ignores_seeded_peaks():
    """The reference for both the gate and the reported deltas must be what
    the FREE search found. Measuring a candidate against its own seeded
    placement would be measuring it against itself.
    """
    peaks = [{'score': 0.99, 'seeded': True, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0},
             {'score': 0.95, 'seeded': False, 'theta': 1.0, 'dx': 0.0, 'dy': 0.0},
             {'score': 0.90, 'seeded': False, 'theta': 2.0, 'dx': 0.0, 'dy': 0.0}]
    assert free_optimum(peaks)['score'] == 0.95


# ── Layer 3 — the reference is the best free fit ACROSS conventions ──────────
# The gate above compared a candidate with "the best free fit". main() read
# that as the free optimum of the candidate's OWN convention, while report()
# took the well-fit floor from the max over conventions. So a mirrored (B)
# reading was only ever asked "are you the best mirrored placement of this
# map", never "is a mirrored reading of this map competitive at all".
#
# b2maps_k4_cut1200 is that defect in the committed logs: A free-fits at 99.5%
# and B at 80.7%, and B's 80.4% candidate passed on 0.3 pp of its own
# convention while clearing the 80% floor. Two winners, verdict AMBIGUOUS,
# blaming world symmetry for a 19.1 pp difference.
#
# These tests run the real wiring -- gate_all_conventions() is what main()
# calls -- and then report(), so what is pinned is the verdict, not only the
# arithmetic underneath it. All pure dict work: no scipy, no PIL, no ROS.

best_free_convention = fitter.best_free_convention
convention_margin_pp = fitter.convention_margin_pp
gate_all_conventions = fitter.gate_all_conventions
report = fitter.report

# (free fit, analytic candidate score) per convention.
K4_CUT1200 = {'A': (0.995, 0.995), 'B': (0.807, 0.804)}


def synthetic_robot(per_conv):
    """The minimum report() reads for one robot, gated the way main() gates it.

    Both directions of a convention are placed at the same spot, which is what
    every b2maps cut does -- map_T_odom is its own inverse there -- so they
    merge into one winner per convention and two winners means two CONVENTIONS
    disagreeing, which is the case at issue here.
    """
    conventions = {}
    for i, (conv, (free, cand_score)) in enumerate(per_conv.items()):
        theta, dx, dy = 179.5 - 90.0 * i, 1.0 + 3.0 * i, -2.0 - 3.0 * i
        peaks = [{'score': free, 'seeded': False,
                  'theta': theta, 'dx': dx, 'dy': dy},
                 {'score': free - 0.05, 'seeded': False,
                  'theta': theta - 7.0, 'dx': dx + 4.0, 'dy': dy + 4.0}]
        cands = [{'label': lb, 'convention': conv, 'score': cand_score,
                  'theta': theta, 'dx': dx, 'dy': dy,
                  'delta_theta_deg': 0.0, 'delta_dx_m': 0.0, 'delta_dy_m': 0.0,
                  'agrees_with_optimum': True,
                  'best_match': {'peak_rank': 1, 'peak_score': free,
                                 'delta_theta_deg': 0.0, 'delta_dx_m': 0.0,
                                 'delta_dy_m': 0.0, 'agrees': True},
                  'plateau_deg': [theta - 0.5, theta + 0.5]}
                 for lb in (MAP_T_ODOM, INVERSE)]
        conventions[conv] = {'refined_peaks': peaks,
                             'analytic_candidates': cands,
                             'merged_groups': merge_groups(cands),
                             'search_bounds': {}}
    entry = {'conventions': conventions}
    # The real thing main() calls, not a re-implementation of it: a test that
    # gated these candidates itself would pass while main() gated them
    # differently, which is the defect it is here to catch.
    info = gate_all_conventions(
        {c: conventions[c]['refined_peaks'] for c in per_conv},
        {c: conventions[c]['analytic_candidates'] for c in per_conv})
    return entry, info


def synthetic_verdict(per_conv, robot=4):
    entry, info = synthetic_robot(per_conv)
    return report({robot: entry}, [robot], list(per_conv)), entry, info


def gate_of(entry, conv, label=MAP_T_ODOM):
    return next(c for c in entry['conventions'][conv]['analytic_candidates']
                if c['label'] == label)['gate']


def test_best_free_convention_names_the_max():
    peaks = {c: [{'score': s, 'seeded': False, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0}]
             for c, s in (('A', 0.995), ('B', 0.807))}
    assert best_free_convention(peaks) == ('A', 0.995)


def test_best_free_convention_ignores_seeded_peaks():
    """Same rule free_optimum follows: a seeded peak is a candidate's own
    placement, so letting it set the reference would let a candidate raise the
    bar it is about to be measured against."""
    peaks = {'A': [{'score': 0.99, 'seeded': True, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0},
                   {'score': 0.70, 'seeded': False, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0}],
             'B': [{'score': 0.80, 'seeded': False, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0}]}
    assert best_free_convention(peaks) == ('B', 0.80)


def test_a_tie_goes_to_the_first_convention_listed():
    """A when both are fitted. The accumulator this replaced broke ties the
    same way, comparing with a strict `>`."""
    peaks = {c: [{'score': 0.9, 'seeded': False, 'theta': 0.0, 'dx': 0.0, 'dy': 0.0}]
             for c in ('A', 'B')}
    assert best_free_convention(peaks)[0] == 'A'


def test_every_candidate_of_the_map_gets_the_same_reference():
    """The wiring itself: one reference per MAP, not per convention. Only the
    reported own-convention figure may differ between them."""
    entry, info = synthetic_robot(K4_CUT1200)
    assert info['reference_convention'] == 'A'
    assert info['reference_score'] == 0.995
    for conv in ('A', 'B'):
        for cand in entry['conventions'][conv]['analytic_candidates']:
            assert cand['gate']['reference_convention'] == 'A'
            assert cand['gate']['reference_score'] == 0.995
    assert gate_of(entry, 'A')['own_convention_score'] == 0.995
    assert gate_of(entry, 'B')['own_convention_score'] == 0.807


def test_a_mirrored_reading_18_pp_below_A_fails():
    """b2maps_k4_cut1200. The old comparison and the new one, side by side."""
    # What the gate used to do: B against B, and it passed.
    assert candidate_gate(0.804, 0.807)['pass'] is True
    # What it does now.
    entry, _ = synthetic_robot(K4_CUT1200)
    g = gate_of(entry, 'B')
    assert g['pass'] is False
    assert g['within_tol'] is False
    assert g['clears_floor'] is True          # the failure is the gap, not the floor
    assert g['score_gap'] == pytest.approx(-0.191)
    # and the comparison it replaced is still recorded, reported-only
    assert g['own_convention_gap'] == pytest.approx(-0.003)
    assert gate_of(entry, 'A')['pass'] is True


def test_the_18_pp_case_resolves_A():
    verdict, _, _ = synthetic_verdict(K4_CUT1200)
    assert verdict['resolved'] is True
    assert [w['convention'] for w in verdict['winners']] == ['A']


def test_tied_conventions_stay_ambiguous():
    """Two readings genuinely within the tolerance of each other are a real
    ambiguity, and must survive as one. The fix must not simply hand every map
    to whichever convention scores higher."""
    verdict, entry, _ = synthetic_verdict({'A': (0.990, 0.990), 'B': (0.985, 0.985)})
    assert gate_of(entry, 'A')['pass'] is True
    assert gate_of(entry, 'B')['pass'] is True
    assert verdict['resolved'] is False
    assert [w['convention'] for w in verdict['winners']] == ['A', 'B']


def test_one_convention_alone_is_unchanged():
    """--convention B on the very map whose B reading fails when A is fitted
    too: with nothing to compare against, the reference is B's own optimum,
    which is exactly what the gate read before this change."""
    verdict, entry, info = synthetic_verdict({'B': (0.807, 0.804)})
    assert info['reference_convention'] == 'B'
    assert info['reference_score'] == 0.807
    g = gate_of(entry, 'B')
    assert g['pass'] is True
    assert g['score_gap'] == pytest.approx(g['own_convention_gap'])
    assert verdict['resolved'] is True
    assert [w['convention'] for w in verdict['winners']] == ['B']


def test_the_margin_is_reported_never_gated():
    verdict, _, info = synthetic_verdict(K4_CUT1200)
    assert info['convention_margin_pp'] == pytest.approx(18.8)
    assert verdict['convention_margin_pp']['robot_4'] == pytest.approx(18.8)
    assert 'convention_margin_pp' in verdict['gate']['reported_not_gated']
    assert 'gate.own_convention_gap' in verdict['gate']['reported_not_gated']


def test_the_margin_is_none_for_a_single_convention():
    _, _, info = synthetic_verdict({'B': (0.807, 0.804)})
    assert info['convention_margin_pp'] is None
    assert convention_margin_pp({'A': [{'score': 0.9, 'seeded': False,
                                        'theta': 0.0, 'dx': 0.0, 'dy': 0.0}]}) is None


def test_a_well_fitting_convention_does_not_rescue_a_noise_map():
    """The floor is still each candidate's own score. Raising the reference
    cannot turn a failing candidate into a passing one, only the reverse."""
    verdict, entry, _ = synthetic_verdict({'A': (0.99, 0.60), 'B': (0.55, 0.55)})
    assert gate_of(entry, 'A')['clears_floor'] is False
    assert gate_of(entry, 'B')['pass'] is False
    assert verdict['resolved'] is False
    assert verdict['winners'] == []


@needs_scipy
def test_a_seed_survives_the_translation_nms_that_deleted_the_twin():
    """The b2maps_e0t_kp failure, in miniature.

    The only test in this file that reaches a deferred import: coarse_search
    pulls scipy.ndimage and scipy.signal at call time, so this is skipped
    where scipy is absent. Nothing above it needs the marker.

    The map's best placement and its 180 deg twin sit 0.307 m apart in
    translation -- inside NMS_RADIUS -- so the greedy suppression drops the
    twin and the candidate is then matched against something metres away. A
    seeded placement must come through that suppression untouched.
    """
    import numpy as np

    rects = [(-1.0, 1.0, -0.05, 0.05)]
    mask, wx0, wy0 = fitter.build_wall_mask(rects)
    pts = np.array([[-0.5, 0.0], [0.0, 0.0], [0.5, 0.0]])
    c = pts.mean(axis=0)
    seed = {'theta': 180.0, 'dx': 0.30, 'dy': 0.0}

    plain, _ = coarse_search(pts, c, mask, wx0, wy0, np.array([0.0, 180.0]))
    seeded, _ = coarse_search(pts, c, mask, wx0, wy0, np.array([0.0, 180.0]),
                              seeds=[seed])

    assert all(p['seeded'] is False for p in plain)
    kept = [p for p in seeded if p['seeded']]
    assert len(kept) == 1
    assert (kept[0]['theta'], kept[0]['dx'], kept[0]['dy']) == (180.0, 0.30, 0.0)
    # and the free search's own peaks are exactly what they were without it
    assert [(p['theta'], p['dx'], p['dy']) for p in seeded if not p['seeded']] \
        == [(p['theta'], p['dx'], p['dy']) for p in plain]
