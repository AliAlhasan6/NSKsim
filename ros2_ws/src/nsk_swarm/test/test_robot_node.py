"""Layer 1 — the cmd_vel publish gate in NSKRobotNode._publish_cmd_vel.

Two independent parameters must both permit motion before a Twist reaches
the wheels: wander_enabled ("should this node drive at all", opt-in) and
nav_controlled ("does another controller own the wheels", set per-robot
from the launch file's nav_robots). Per the pattern in test_geometry.py /
test_stuck_escape.py, the real unbound _publish_cmd_vel is bound to a plain
stub with a mock publisher — this exercises the production gate with no
rclpy init and no wait_for_service block (NSKRobotNode.__init__ blocks on
the engine services).
"""

from collections import deque
from types import SimpleNamespace
from unittest import mock

import pytest

from nsk_swarm.robot_node import EXPLORE, FLOCK, NSKRobotNode


def make_robot_stub(wander_enabled=False, nav_controlled=False,
                    walk_speed=0.15, angular_z=0.2, state=EXPLORE,
                    recovery_phase=None, stuck_window_sec=4.0):
    stub = SimpleNamespace(
        robot_id=0, pos_x=0.0, pos_y=0.0, yaw=0.0,
        wander_enabled=wander_enabled, nav_controlled=nav_controlled,
        walk_speed=walk_speed, walk_turn_max=0.5,
        stuck_window_sec=stuck_window_sec,
        _state=state,
        _current_angular_z=angular_z,
        _recovery_phase=recovery_phase,
        _motion_samples=deque(),
        cmd_pub=mock.MagicMock())
    for name in ('_publish_cmd_vel', '_record_motion_sample'):
        setattr(stub, name, getattr(NSKRobotNode, name).__get__(stub))
    return stub


def published_twist(stub):
    """The single Twist handed to cmd_pub.publish, or None if silent."""
    if not stub.cmd_pub.publish.call_args_list:
        return None
    assert len(stub.cmd_pub.publish.call_args_list) == 1
    (msg,), _ = stub.cmd_pub.publish.call_args
    return msg


# ── Declared default ─────────────────────────────────────────────────────────

def test_wander_enabled_defaults_to_false():
    # The parameter's declared default is what peers inherit when a launch
    # file says nothing — it must be off, or ungated wander walks them out
    # of the spawn cluster.
    assert NSKRobotNode.PARAMS['wander_enabled'] is False


def test_nav_controlled_default_unchanged():
    # nav_controlled keeps its own meaning and default: wander_enabled is an
    # additional gate, not a replacement.
    assert NSKRobotNode.PARAMS['nav_controlled'] is False


# ── The gate ─────────────────────────────────────────────────────────────────

def test_no_cmd_vel_published_with_default_parameters():
    # Defaults (wander_enabled False, nav_controlled False): a robot nobody
    # asked to drive must sit still and publish nothing at all.
    stub = make_robot_stub()

    for _ in range(10):
        stub._publish_cmd_vel()

    stub.cmd_pub.publish.assert_not_called()
    # No publish means no motion sample either, so the stuck detector stays
    # idle rather than judging a robot this node isn't driving.
    assert len(stub._motion_samples) == 0


def test_cmd_vel_published_when_wander_enabled():
    # wander_enabled True + nav_controlled False is the one combination that
    # drives: the existing wander command reaches the wheels unchanged.
    stub = make_robot_stub(wander_enabled=True, nav_controlled=False,
                           walk_speed=0.15, angular_z=0.2)

    stub._publish_cmd_vel()

    cmd = published_twist(stub)
    assert cmd is not None
    assert cmd.linear.x == pytest.approx(0.15)
    assert cmd.angular.z == pytest.approx(0.2)
    # The commanded speed was recorded for the stuck detector.
    assert len(stub._motion_samples) == 1
    assert stub._motion_samples[0][3] == pytest.approx(0.15)


def test_no_cmd_vel_when_nav_controlled_even_with_wander_enabled():
    # Both gates must permit motion: Nav2 owning the wheels still wins over
    # an explicitly enabled wander driver.
    stub = make_robot_stub(wander_enabled=True, nav_controlled=True)

    stub._publish_cmd_vel()

    stub.cmd_pub.publish.assert_not_called()
    assert len(stub._motion_samples) == 0


def test_no_cmd_vel_when_wander_disabled_and_not_nav_controlled():
    # The rung that motivated the gate: peers with no Nav2 stack of their
    # own and wander off — silent wheels, not 0.15 m/s into a maze wall.
    stub = make_robot_stub(wander_enabled=False, nav_controlled=False)

    stub._publish_cmd_vel()

    stub.cmd_pub.publish.assert_not_called()


# ── The gated code path is intact, not deleted ───────────────────────────────

def test_flock_half_speed_still_applies_when_wander_enabled():
    # Gating must not have changed what wander commands once enabled: FLOCK
    # still halves the linear speed to allow sharing.
    stub = make_robot_stub(wander_enabled=True, state=FLOCK, walk_speed=0.15)

    stub._publish_cmd_vel()

    cmd = published_twist(stub)
    assert cmd.linear.x == pytest.approx(0.075)


def test_recovery_command_published_when_wander_enabled():
    # An active recovery leg still drives through the same publish site
    # (the gate sits ahead of the recovery branch, so recovery output is
    # muted with wander off and live with it on).
    recovery_cmd = mock.MagicMock()
    recovery_cmd.linear.x = -0.15
    stub = make_robot_stub(wander_enabled=True, recovery_phase='recover_reverse')
    stub._advance_recovery = mock.MagicMock(return_value=recovery_cmd)

    stub._publish_cmd_vel()

    stub._advance_recovery.assert_called_once()
    assert published_twist(stub) is recovery_cmd


def test_recovery_command_muted_when_wander_disabled():
    stub = make_robot_stub(wander_enabled=False, recovery_phase='recover_reverse')
    stub._advance_recovery = mock.MagicMock()

    stub._publish_cmd_vel()

    stub._advance_recovery.assert_not_called()
    stub.cmd_pub.publish.assert_not_called()
