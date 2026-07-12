"""Layer 1 — proximity math from NSKRobotNode and ConvergenceMonitorNode.

Choice (per task): _in_range / _dist / _peers_in_range are instance methods
whose math touches only plain attributes (pos_x, pos_y, peer_positions /
robot_positions, comm_range, robot_id, num_robots). Rather than extracting
them (not trivially separable without touching robot_node, which is out of
scope) or paying for a constructed node in layer 2 (NSKRobotNode.__init__
blocks in wait_for_service for the engine), the real unbound methods are
bound to a plain stub object. This exercises the exact production code with
no rclpy init at all.
"""

import math
from types import SimpleNamespace

from nsk_swarm.convergence_monitor import ConvergenceMonitorNode
from nsk_swarm.robot_node import NSKRobotNode


def make_robot_stub(pos_x=0.0, pos_y=0.0, comm_range=3.0,
                    robot_id=0, num_robots=4, peer_positions=None):
    stub = SimpleNamespace(
        robot_id=robot_id, num_robots=num_robots, comm_range=comm_range,
        pos_x=pos_x, pos_y=pos_y,
        peer_positions=peer_positions if peer_positions is not None else {})
    stub._in_range = NSKRobotNode._in_range.__get__(stub)
    stub._dist = NSKRobotNode._dist.__get__(stub)
    stub._peers_in_range = NSKRobotNode._peers_in_range.__get__(stub)
    return stub


def make_monitor_stub(robot_positions, comm_range=3.0):
    stub = SimpleNamespace(robot_positions=robot_positions,
                           comm_range=comm_range)
    stub._peers_in_range = ConvergenceMonitorNode._peers_in_range.__get__(stub)
    return stub


# ── NSKRobotNode._dist ───────────────────────────────────────────────────────

def test_dist_euclidean():
    stub = make_robot_stub(pos_x=1.0, pos_y=2.0,
                           peer_positions={1: (4.0, 6.0)})   # 3-4-5 triangle
    assert stub._dist(1) == 5.0


def test_dist_unknown_peer_is_sentinel():
    stub = make_robot_stub()
    assert stub._dist(1) == 999.0


def test_dist_zero_for_coincident_positions():
    stub = make_robot_stub(pos_x=2.5, pos_y=-1.0,
                           peer_positions={2: (2.5, -1.0)})
    assert stub._dist(2) == 0.0


# ── NSKRobotNode._in_range ───────────────────────────────────────────────────

def test_in_range_inside():
    stub = make_robot_stub(peer_positions={1: (1.0, 1.0)})
    assert stub._in_range(1) is True


def test_in_range_boundary_is_exclusive():
    # Exactly comm_range away: strict '<', so NOT in range.
    stub = make_robot_stub(comm_range=3.0, peer_positions={1: (3.0, 0.0)})
    assert stub._in_range(1) is False
    # Just inside the boundary.
    stub.peer_positions[1] = (3.0 - 1e-6, 0.0)
    assert stub._in_range(1) is True


def test_in_range_unknown_peer():
    stub = make_robot_stub()
    assert stub._in_range(5) is False


# ── NSKRobotNode._peers_in_range ─────────────────────────────────────────────

def test_peers_in_range_filters_self_distance_and_unknown():
    # Peer 1 in range, peer 2 out of range, peer 3 has no known position;
    # robot_id 0 itself must never appear.
    stub = make_robot_stub(
        robot_id=0, num_robots=4, comm_range=3.0,
        peer_positions={0: (0.0, 0.0),        # own id: must be skipped
                        1: (1.0, 1.0),        # dist ~1.41 -> in range
                        2: (10.0, 10.0)})     # dist ~14.1 -> out of range
    assert stub._peers_in_range() == [1]


def test_peers_in_range_empty_when_alone():
    stub = make_robot_stub()
    assert stub._peers_in_range() == []


# ── ConvergenceMonitorNode._peers_in_range(a, b) ─────────────────────────────

def test_monitor_pair_in_range_and_symmetric():
    stub = make_monitor_stub({0: (0.0, 0.0), 1: (2.0, 0.0)}, comm_range=3.0)
    assert stub._peers_in_range(0, 1) is True
    assert stub._peers_in_range(1, 0) is True


def test_monitor_pair_boundary_is_exclusive():
    stub = make_monitor_stub({0: (0.0, 0.0), 1: (3.0, 0.0)}, comm_range=3.0)
    assert stub._peers_in_range(0, 1) is False


def test_monitor_pair_missing_position():
    stub = make_monitor_stub({0: (0.0, 0.0)})
    assert stub._peers_in_range(0, 1) is False
    assert stub._peers_in_range(1, 0) is False
