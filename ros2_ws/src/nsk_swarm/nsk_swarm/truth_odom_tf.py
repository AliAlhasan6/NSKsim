#!/usr/bin/env python3
"""truth_odom_tf.py — publish robot_K/odom -> robot_K/base_footprint from
Gazebo ground truth, anchored at the spawn pose.

Why this exists (2026-09-20):
  b2maps_e0's online SLAM map is distorted into fans of rotated walls. Over
  that run the median gap between odometry heading and truth was 138 deg. The
  explorer steered by that map, blacklisted 20 frontiers and declared
  exploration complete, having only ever seen the central walls.

  HANDOFF_2026-09-19_known_pose_2x2.md section 5 measured the mechanism
  offline: on b18_run2's robot_0, truth poses with the matcher off fit the
  world at 96.1%, the same scans on odometry at 21.9%. That experiment rewrote
  the bag AFTER the fact, so the explorer still steered by the broken map
  during the run. This node moves the fix INTO the run.

What it does:
  Subscribes to the bridged /model/robot_K/pose (Gazebo's true model pose, from
  the PosePublisher appended by make_namespaced_burger_sdf) and broadcasts the
  robot's odom -> base_footprint transform from it. The scans are untouched:
  only the pose layer is known.

  It replaces, rather than supplements, the burger DiffDrive plugin's own
  transform. Robot K's DiffDrive is pointed at an unbridged gz topic in the
  launch file, so exactly one publisher remains for this frame pair. See
  swarm_sim.launch.py's truth_odom_robots argument.

The anchor:
  odom_T_base = inv(A) . P, with A the robot's SPAWN pose and P its true pose.
  Anchoring on spawn -- rather than on the first pose seen -- makes the odom
  frame BE the spawn frame, which is what experiments/slam/fit_world_transform.py
  assumes when it builds world_T_odom as a pure translation with spawn yaw 0,
  and what experiments/slam/rewrite_odom_from_truth.py anchors its offline
  rewrite on. A and the formula are deliberately the same in both places: the
  online map and an offline map built from the same bag then differ only in the
  mapper, never in where the scans were placed.

  A comes from the launch file's spawn_xs/spawn_ys, which are DOT_POSES -- the
  same single source robot_node and convergence_monitor already use. Every
  spawn holds yaw 0 (DOT_POSES' own comment, and resolve_spawn_poses() verifies
  per revision that no '-Y' reaches ros_gz_sim create), so inv(A) is a pure
  negated translation. The composition below does not rely on that; the fitter
  does.

Checked consequence, not an assumption: with the robot parked at spawn this
transform reads identity to within check_run_bag.py's own SPAWN_TOL_M (1 cm)
and SPAWN_TOL_DEG (0.5 deg). That is rehearsal check R3.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

# /model/robot_K/pose: latest-pose-only, exactly robot_node.ODOM_QOS' reasoning
# (robot_node.py:41-45) -- a sensor-convention stream off the gz bridge, where
# a queued backlog would make the transform LAG, which is worse than a 50 ms
# gap. Best-effort subscriber against the bridge's reliable publisher is the
# compatible direction.
POSE_QOS = qos_profile_sensor_data


# ────────────────────────────── SE(2) helpers ───────────────────────────────
# Mirrored from experiments/slam/rewrite_odom_from_truth.py:76-99 rather than
# imported: that module lives outside the ROS package and imports rosbag2_py at
# module scope. test_truth_odom_tf.py pins the two together by checking both
# against the hand-calculated vector that script's C1 uses.

def yaw_of(q) -> float:
    """Quaternion -> yaw. Same form as analysis/tf_pair_at.py."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def se2_compose(a, b):
    """a . b, both (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def se2_inverse(a):
    ax, ay, at = a
    c, s = math.cos(-at), math.sin(-at)
    return (-(c * ax - s * ay), -(s * ax + c * ay), -at)


def wrap(a: float) -> float:
    """Wrap radians to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_se2(msg: PoseStamped):
    p = msg.pose.position
    return (p.x, p.y, yaw_of(msg.pose.orientation))


def odom_from_truth(anchor, truth):
    """inv(A) . P -- the spawn-anchored odom pose. Pure, so it can be tested.

    Both arguments and the result are (x, y, yaw) in radians.
    """
    return se2_compose(se2_inverse(anchor), truth)


def make_transform(stamp, parent: str, child: str, se2) -> TransformStamped:
    """Build the broadcast transform from an (x, y, yaw).

    z is forced to 0 and the quaternion is yaw-only, exactly as
    rewrite_odom_from_truth.rewrite() does (lines 396-406). base_footprint is
    the ground-projected frame, and the burger's roughly -0.71 deg resting
    pitch is suspension settle, not heading -- see check_run_bag.py's
    yaw_from_quaternion docstring, where letting that tilt into an angular
    comparison would fail every bag on every robot. The DiffDrive transform
    this one replaces is likewise planar.
    """
    x, y, yaw = se2
    tr = TransformStamped()
    tr.header.stamp = stamp
    tr.header.frame_id = parent
    tr.child_frame_id = child
    tr.transform.translation.x = x
    tr.transform.translation.y = y
    tr.transform.translation.z = 0.0
    tr.transform.rotation.x = 0.0
    tr.transform.rotation.y = 0.0
    tr.transform.rotation.z = math.sin(yaw / 2.0)
    tr.transform.rotation.w = math.cos(yaw / 2.0)
    return tr


# ──────────────────────────────── the node ──────────────────────────────────

class TruthOdomTF(Node):
    """One instance per truth-driven robot. Launched WITHOUT a namespace so
    TransformBroadcaster lands on the global /tf, the way convergence_monitor
    publishes its markers -- the whole swarm shares one /tf tree keyed by
    robot_N/-prefixed frames.
    """

    def __init__(self):
        super().__init__('truth_odom_tf')

        self.declare_parameter('robot_id', 0)
        # Defaults of 0.0 are the world origin, NOT a plausible spawn: a launch
        # that forgets to pass these produces a map offset by the spawn pose,
        # which R3 catches. Declared with float defaults so an integer
        # spelling is refused rather than silently retyped.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        spawn_x = float(self.get_parameter('spawn_x').value)
        spawn_y = float(self.get_parameter('spawn_y').value)

        # Spawn yaw is 0 for every robot in DOT_POSES and the launch passes no
        # '-Y' to ros_gz_sim create, so the anchor is a pure translation.
        self.anchor = (spawn_x, spawn_y, 0.0)

        ns = f'robot_{self.robot_id}'
        self.parent_frame = f'{ns}/odom'
        self.child_frame = f'{ns}/base_footprint'
        self.pose_topic = f'/model/{ns}/pose'

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped, self.pose_topic, self._pose_cb, POSE_QOS)

        self._published = 0
        self._first_logged = False

        self.get_logger().info(
            f'truth odometry for {ns}: {self.pose_topic} -> /tf '
            f'{self.parent_frame} -> {self.child_frame}, '
            f'anchor A = ({spawn_x:+.4f}, {spawn_y:+.4f}, +0.0000 deg)')

    def _pose_cb(self, msg: PoseStamped):
        se2 = odom_from_truth(self.anchor, pose_se2(msg))
        # The pose's OWN stamp, never now(): it is the sim time the bridge
        # carried over from Gazebo, and every consumer of this transform runs
        # on use_sim_time. Stamping with wall time here would desynchronise
        # the whole tree from the scans.
        self.tf_broadcaster.sendTransform(
            make_transform(msg.header.stamp, self.parent_frame,
                           self.child_frame, se2))
        self._published += 1

        if not self._first_logged:
            self._first_logged = True
            # The first transform IS the spawn check: parked at spawn it must
            # read identity. Logged so a boot can be judged without tf2_echo.
            self.get_logger().info(
                f'first transform: x={se2[0]:+.4f} y={se2[1]:+.4f} '
                f'yaw={math.degrees(wrap(se2[2])):+.4f} deg '
                f'(identity expected while parked at spawn)')


def main(args=None):
    rclpy.init(args=args)
    node = TruthOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
