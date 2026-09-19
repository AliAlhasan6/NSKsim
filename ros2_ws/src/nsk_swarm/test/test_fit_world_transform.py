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
preflight_motion.py. Importing it runs module-level code only -- numpy, scipy,
PIL, yaml and bag_overlap, which defers its rosbag2_py import into the two
functions that read a bag. No ROS environment is needed.
"""

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
