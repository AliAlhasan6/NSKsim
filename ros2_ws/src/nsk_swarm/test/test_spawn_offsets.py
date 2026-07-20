"""Layer 1 — world-frame spawn offsets in NSKRobotNode and
ConvergenceMonitorNode.

The TurtleBot3 DiffDrive plugin publishes odometry relative to each
robot's spawn pose (the dots' OdometryPublisher was world-frame), so both
nodes add the launch-provided spawn offset to every odom reading before
any distance/comm-range computation. Per the pattern in test_geometry.py,
the real unbound callbacks are bound to plain stubs — no rclpy init.
"""

from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from nsk_swarm.convergence_monitor import ConvergenceMonitorNode
from nsk_swarm.robot_node import NSKRobotNode


def odom_msg(x: float, y: float) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0   # identity: yaw 0
    return msg


def make_robot_stub(spawn_x=0.0, spawn_y=0.0, spawn_xs=(), spawn_ys=(),
                    robot_id=0, num_robots=3, comm_range=3.0):
    stub = SimpleNamespace(
        robot_id=robot_id, num_robots=num_robots, comm_range=comm_range,
        spawn_x=spawn_x, spawn_y=spawn_y,
        spawn_xs=list(spawn_xs), spawn_ys=list(spawn_ys),
        pos_x=0.0, pos_y=0.0, yaw=0.0, peer_positions={})
    for name in ('_odom_cb', '_peer_odom_cb', '_in_range', '_dist',
                 '_peers_in_range'):
        setattr(stub, name, getattr(NSKRobotNode, name).__get__(stub))
    return stub


def make_monitor_stub(spawn_xs=(), spawn_ys=()):
    stub = SimpleNamespace(spawn_xs=list(spawn_xs), spawn_ys=list(spawn_ys),
                           robot_positions={})
    stub._odom_cb = ConvergenceMonitorNode._odom_cb.__get__(stub)
    return stub


# ── NSKRobotNode: own odometry ───────────────────────────────────────────────

def test_own_odom_offset_by_spawn():
    stub = make_robot_stub(spawn_x=3.0, spawn_y=-2.0)
    stub._odom_cb(odom_msg(1.0, 0.5))
    assert stub.pos_x == pytest.approx(4.0)
    assert stub.pos_y == pytest.approx(-1.5)
    assert stub.yaw == pytest.approx(0.0)   # yaw stays raw — no offset


def test_own_odom_default_zero_offset_passthrough():
    stub = make_robot_stub()
    stub._odom_cb(odom_msg(1.25, -0.75))
    assert (stub.pos_x, stub.pos_y) == (1.25, -0.75)


# ── NSKRobotNode: peer odometry ──────────────────────────────────────────────

def test_peer_odom_offset_by_that_peers_spawn():
    stub = make_robot_stub(spawn_x=3.0, spawn_y=-2.0,
                           spawn_xs=[3.0, -1.0], spawn_ys=[-2.0, 5.0])
    stub._peer_odom_cb(odom_msg(0.5, -0.5), peer_id=1)
    assert stub.peer_positions[1] == (pytest.approx(-0.5), pytest.approx(4.5))


def test_peer_beyond_spawn_list_gets_zero_offset():
    stub = make_robot_stub(spawn_xs=[3.0], spawn_ys=[-2.0])
    stub._peer_odom_cb(odom_msg(1.0, 2.0), peer_id=2)
    assert stub.peer_positions[2] == (1.0, 2.0)


def test_comm_range_gating_uses_world_frame():
    # All robots sit at their spawn points, so every raw odom is (0, 0) —
    # raw positions would call them all coincident. World frame must decide:
    # peer 1 spawned 2 m away (in range), peer 2 far away (out of range).
    stub = make_robot_stub(spawn_x=3.0, spawn_y=-2.0, robot_id=0,
                           spawn_xs=[3.0, 3.0, 10.0],
                           spawn_ys=[-2.0, 0.0, 10.0])
    stub._odom_cb(odom_msg(0.0, 0.0))
    stub._peer_odom_cb(odom_msg(0.0, 0.0), peer_id=1)
    stub._peer_odom_cb(odom_msg(0.0, 0.0), peer_id=2)
    assert stub._dist(1) == pytest.approx(2.0)
    assert stub._peers_in_range() == [1]


# ── ConvergenceMonitorNode odometry ──────────────────────────────────────────

def test_monitor_odom_offset_by_robot_spawn():
    stub = make_monitor_stub(spawn_xs=[4.0, 1.2], spawn_ys=[0.0, 3.8])
    stub._odom_cb(odom_msg(1.0, 2.0), 0)
    stub._odom_cb(odom_msg(-0.5, 0.25), 1)
    assert stub.robot_positions[0] == (pytest.approx(5.0), pytest.approx(2.0))
    assert stub.robot_positions[1] == (pytest.approx(0.7), pytest.approx(4.05))


def test_monitor_empty_spawn_arrays_passthrough():
    stub = make_monitor_stub()
    stub._odom_cb(odom_msg(1.0, 2.0), 0)
    assert stub.robot_positions[0] == (1.0, 2.0)
