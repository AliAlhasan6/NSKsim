"""truth_odom_tf: the SE(2) composition, the spawn anchor, and the transform.

What this covers, and why each part is here:

  The node replaces DiffDrive's odom -> base_footprint for one robot with
  inv(A) . P, A the spawn pose and P Gazebo's true pose. Every map built from
  the run rests on that composition and on that anchor, and both fail SILENTLY:
  an inverted operand order or a wrong A does not crash, it moves every map by
  a constant and the fitter happily reports a score for a map placed metres
  wrong. That is the b18_run2 lesson (HANDOFF_2026-09-19 section 4).

  So the composition is checked against a HAND-CALCULATED vector rather than
  against itself, the reversed order is checked to FAIL that vector, and the
  node's helpers are pinned directly against
  experiments/slam/rewrite_odom_from_truth.py -- the offline rewriter whose
  formula the node deliberately duplicates. The two cannot drift apart without
  this file going red.

No rclpy spin, no ROS graph: these are pure functions plus one message builder.
"""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

from nsk_swarm.truth_odom_tf import (make_transform, odom_from_truth, pose_se2,
                                     se2_compose, se2_inverse, wrap, yaw_of)

REPO_ROOT = Path(__file__).resolve().parents[4]
REWRITER = REPO_ROOT / 'experiments' / 'slam' / 'rewrite_odom_from_truth.py'

# DOT_POSES at HEAD (swarm_sim.launch.py). Duplicated as a literal on purpose:
# importing the launch module would drag in launch_ros, and a test that read
# the same table the code reads could not catch a change to it.
DOT_XY = [(0.86, 0.28), (0.00, 0.90), (-0.86, 0.28),
          (-0.53, -0.73), (0.53, -0.73)]


def _load_rewriter():
    """Import the offline rewriter by path.

    It lives outside the ROS package and imports rosbag2_py at module scope,
    which is why truth_odom_tf duplicates its helpers instead of importing
    them. rosbag2_py is importable on this machine without sourcing the
    workspace, so the pin below normally runs; it skips rather than fails
    where it is absent, since a missing rosbag2_py says nothing about the
    node.
    """
    spec = importlib.util.spec_from_file_location('_rewriter', REWRITER)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as exc:                       # pragma: no cover
        pytest.skip(f'cannot import {REWRITER.name}: {exc}')
    return module


class _Q:
    """Minimal quaternion stand-in for yaw_of()."""

    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


def _yaw_quat(yaw):
    return _Q(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _close(got, want, tol=1e-9):
    return (abs(got[0] - want[0]) <= tol and abs(got[1] - want[1]) <= tol
            and abs(wrap(got[2] - want[2])) <= tol)


# ── the composition, against a hand-calculated value ────────────────────────

def test_the_composition_matches_the_hand_calculated_vector():
    """rewrite_odom_from_truth.check_c1's vector, and it is hand-calculated
    there (that function's docstring) -- not produced by either implementation.
    """
    A = (2.0, -1.0, math.radians(30.0))
    P = (3.0, 1.0, math.radians(75.0))
    want = (1.866025, 1.232051, math.radians(45.0))

    assert _close(odom_from_truth(A, P), want, tol=1e-6)


def test_the_wrong_operand_orders_fail_that_vector():
    """The check above is only a check because the wrong orders miss it.

    Two slips are plausible, and neither raises. C1 records the first:
    inverting the wrong operand gives (-2.190671, 0.448288, -45 deg).
    Inverting the right one but composing it on the right gives a third
    answer again, and its yaw is CORRECT -- only the translation is wrong,
    which is exactly the kind of error that shifts a map without looking
    broken.
    """
    A = (2.0, -1.0, math.radians(30.0))
    P = (3.0, 1.0, math.radians(75.0))
    right = (1.866025, 1.232051, math.radians(45.0))

    inverted_operand = se2_compose(se2_inverse(P), A)
    assert _close(inverted_operand,
                  (-2.190671, 0.448288, math.radians(-45.0)), tol=1e-6)
    assert not _close(inverted_operand, right, tol=1e-6)

    composed_backwards = se2_compose(P, se2_inverse(A))
    assert _close(composed_backwards,
                  (0.878680, 0.292893, math.radians(45.0)), tol=1e-6)
    assert not _close(composed_backwards, right, tol=1e-6)


def test_the_anchor_round_trips():
    """A . (inv(A) . P) recovers P, for a pose well away from the anchor."""
    A = (2.0, -1.0, math.radians(30.0))
    P = (3.0, 1.0, math.radians(75.0))

    assert _close(se2_compose(A, odom_from_truth(A, P)), P, tol=1e-9)


# ── pinned against the offline rewriter ─────────────────────────────────────

def test_the_helpers_agree_with_the_offline_rewriter():
    """The node's formula IS the rewriter's, so a divergence is a defect.

    Swept over a grid that includes negative coordinates and yaws either side
    of the +-pi wrap, where a sign error hides.
    """
    rw = _load_rewriter()
    anchors = [(0.86, 0.28, 0.0), (-0.53, -0.73, 0.0),
               (2.0, -1.0, math.radians(30.0)), (0.0, 0.0, math.pi)]
    poses = [(0.0, 0.0, 0.0), (3.0, 1.0, math.radians(75.0)),
             (-7.5, 9.2, math.radians(-179.0)), (4.0, -4.0, math.radians(179.5))]

    for A in anchors:
        for P in poses:
            mine = odom_from_truth(A, P)
            theirs = rw.se2_compose(rw.se2_inverse(A), P)
            assert _close(mine, theirs, tol=1e-12), (A, P, mine, theirs)


def test_yaw_of_agrees_with_the_offline_rewriter():
    rw = _load_rewriter()
    for deg in (-179.9, -90.0, -0.1, 0.0, 0.1, 45.0, 179.9):
        q = _yaw_quat(math.radians(deg))
        assert abs(wrap(yaw_of(q) - rw.yaw_of(q))) <= 1e-12


# ── the spawn anchor ────────────────────────────────────────────────────────

@pytest.mark.parametrize('robot_id', range(5))
def test_a_robot_parked_at_its_spawn_reads_identity(robot_id):
    """R3, as a unit test: odom IS the spawn frame.

    fit_world_transform.py builds world_T_odom as a pure translation with spawn
    yaw 0 and this is what makes that true.
    """
    x, y = DOT_XY[robot_id]
    A = (x, y, 0.0)

    assert _close(odom_from_truth(A, A), (0.0, 0.0, 0.0), tol=1e-12)


def test_the_wrong_robots_spawn_does_not_read_identity():
    """The identity check above is only a check because a wrong A misses it.

    R3's stated failure demonstration: anchor on robot_1's spawn, feed
    robot_0's pose. The pentagon's minimum separation is 1.06 m, so the error
    is three orders of magnitude outside check_run_bag's 1 cm SPAWN_TOL_M.
    """
    A_wrong = (DOT_XY[1][0], DOT_XY[1][1], 0.0)
    P = (DOT_XY[0][0], DOT_XY[0][1], 0.0)

    got = odom_from_truth(A_wrong, P)
    offset = math.hypot(got[0], got[1])

    assert offset == pytest.approx(1.0601887, abs=1e-6)
    assert offset > 0.01           # check_run_bag.SPAWN_TOL_M
    assert not _close(got, (0.0, 0.0, 0.0), tol=1e-3)


def test_a_zero_anchor_leaves_the_world_pose_untouched():
    """The parameter defaults are the world origin, so a launch that forgets
    spawn_x/spawn_y produces a map offset by exactly the spawn pose rather
    than something that looks broken. Recorded here so the failure mode is
    known, not discovered on a run.
    """
    P = (0.86, 0.28, 0.0)

    assert _close(odom_from_truth((0.0, 0.0, 0.0), P), P, tol=1e-12)


# ── the broadcast transform ─────────────────────────────────────────────────

def test_the_transform_is_planar_and_yaw_only():
    """z is forced to 0 and the quaternion carries heading alone.

    The burger rests at about -0.71 deg of pitch (suspension settle, measured
    on b2maps_rehearsal3 -- see check_run_bag.yaw_from_quaternion). Letting
    that into base_footprint would tilt every scan.
    """
    from builtin_interfaces.msg import Time

    stamp = Time(sec=123, nanosec=456_000_000)
    yaw = math.radians(37.0)
    tr = make_transform(stamp, 'robot_0/odom', 'robot_0/base_footprint',
                        (1.5, -2.5, yaw))

    assert tr.header.frame_id == 'robot_0/odom'
    assert tr.child_frame_id == 'robot_0/base_footprint'
    assert tr.header.stamp.sec == 123
    assert tr.header.stamp.nanosec == 456_000_000
    assert tr.transform.translation.x == pytest.approx(1.5)
    assert tr.transform.translation.y == pytest.approx(-2.5)
    assert tr.transform.translation.z == 0.0
    assert tr.transform.rotation.x == 0.0
    assert tr.transform.rotation.y == 0.0
    assert abs(wrap(yaw_of(tr.transform.rotation) - yaw)) <= 1e-12


def test_the_transform_matches_the_quaternion_the_rewriter_writes():
    """Same bytes on the wire as the offline rewrite, for the same pose."""
    from builtin_interfaces.msg import Time

    yaw = math.radians(-118.0)
    tr = make_transform(Time(sec=1, nanosec=0), 'p', 'c', (0.25, -0.75, yaw))

    assert tr.transform.rotation.z == pytest.approx(math.sin(yaw / 2.0), abs=0)
    assert tr.transform.rotation.w == pytest.approx(math.cos(yaw / 2.0), abs=0)


def test_pose_se2_reads_a_posestamped():
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.pose.position.x = 3.0
    msg.pose.position.y = 1.0
    msg.pose.position.z = 0.19          # ignored: this is a planar transform
    yaw = math.radians(75.0)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)

    assert _close(pose_se2(msg), (3.0, 1.0, yaw), tol=1e-12)


def test_the_full_path_from_a_truth_message_to_a_transform():
    """One end-to-end pass of exactly what _pose_cb does, without a node."""
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped

    anchor = (DOT_XY[0][0], DOT_XY[0][1], 0.0)
    msg = PoseStamped()
    msg.header.stamp = Time(sec=42, nanosec=0)
    # 2 m north-east of spawn, turned 90 deg.
    msg.pose.position.x = DOT_XY[0][0] + 2.0
    msg.pose.position.y = DOT_XY[0][1] + 2.0
    msg.pose.orientation.z = math.sin(math.radians(90.0) / 2.0)
    msg.pose.orientation.w = math.cos(math.radians(90.0) / 2.0)

    tr = make_transform(msg.header.stamp, 'robot_0/odom',
                        'robot_0/base_footprint',
                        odom_from_truth(anchor, pose_se2(msg)))

    assert tr.transform.translation.x == pytest.approx(2.0, abs=1e-12)
    assert tr.transform.translation.y == pytest.approx(2.0, abs=1e-12)
    assert abs(wrap(yaw_of(tr.transform.rotation)
                    - math.radians(90.0))) <= 1e-12
    assert tr.header.stamp.sec == 42
