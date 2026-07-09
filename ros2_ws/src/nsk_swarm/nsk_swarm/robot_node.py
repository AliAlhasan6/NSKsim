#!/usr/bin/env python3
"""
robot_node.py — NSK Swarm ROS 2 robot node (v2).
Flocking movement + adaptive compression.
System Python only — no PyTorch.
"""

import json
import math
import random
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from nsk_swarm_interfaces.srv import Compress, Merge

# Movement states
EXPLORE  = 'explore'
FLOCK    = 'flock'
DISPERSE = 'disperse'

# Adaptive compression thresholds
def retention_for_similarity(sim: float) -> float:
    if sim > 0.7:
        return 0.20   # already similar — compress hard
    elif sim > 0.4:
        return 0.40   # default
    else:
        return 0.65   # very different — send rich graph


class NSKRobotNode(Node):

    PARAMS = {
        'robot_id':       0,
        'num_robots':     8,
        'comm_range':     3.0,
        'share_interval': 8.0,
        'world_size':     20.0,
        'walk_speed':     0.15,
        'walk_turn_max':  0.5,
    }

    def __init__(self):
        super().__init__('nsk_robot_node')

        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)

        self.robot_id       = self.get_parameter('robot_id').value
        self.num_robots     = self.get_parameter('num_robots').value
        self.comm_range     = self.get_parameter('comm_range').value
        self.share_interval = self.get_parameter('share_interval').value
        self.world_size     = self.get_parameter('world_size').value
        self.walk_speed     = self.get_parameter('walk_speed').value
        self.walk_turn_max  = self.get_parameter('walk_turn_max').value

        # Position state
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.peer_positions: dict[int, tuple[float, float]] = {}
        self.peer_similarity: dict[int, float] = {}

        # Movement state machine
        self._state              = EXPLORE
        self._disperse_until     = 0.0
        self._current_angular_z  = 0.0
        self._levy_steps_left    = 0
        self._levy_heading       = 0.0

        # Engine service clients.
        # Async-safety: callbacks below block on engine responses, so the
        # timers, the /kg_share subscription and the clients all share one
        # ReentrantCallbackGroup, and main() spins a MultiThreadedExecutor.
        # A blocked callback therefore never prevents another executor
        # thread from delivering the service response (no deadlock, unlike
        # a synchronous call() in a default single-threaded setup).
        self._cb_group     = ReentrantCallbackGroup()
        self._compress_cli = self.create_client(
            Compress, '/nsk/compress', callback_group=self._cb_group)
        self._merge_cli    = self.create_client(
            Merge, '/nsk/merge', callback_group=self._cb_group)
        for cli in (self._compress_cli, self._merge_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'[Robot {self.robot_id}] Waiting for {cli.srv_name} ...')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/robot_{self.robot_id}/cmd_vel', 10)
        self.kg_pub  = self.create_publisher(String, '/kg_share', 10)

        # Subscriptions
        self.create_subscription(Odometry, f'/robot_{self.robot_id}/odom', self._odom_cb, 10)
        for j in range(self.num_robots):
            if j != self.robot_id:
                self.create_subscription(
                    Odometry, f'/robot_{j}/odom',
                    lambda msg, peer=j: self._peer_odom_cb(msg, peer), 10)
        self.create_subscription(String, '/kg_share', self._on_kg_share, 10,
                                 callback_group=self._cb_group)

        # Timers
        self.create_timer(2.0,                self._update_motion_state,
                          callback_group=self._cb_group)
        self.create_timer(0.1,                self._publish_cmd_vel,
                          callback_group=self._cb_group)
        self.create_timer(self.share_interval, self._share_timer_cb,
                          callback_group=self._cb_group)

        self.get_logger().info(f'[Robot {self.robot_id}] Started.')

    # ── Odometry ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _peer_odom_cb(self, msg: Odometry, peer_id: int):
        self.peer_positions[peer_id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    # ── Motion state machine ─────────────────────────────────────────────────

    def _peers_in_range(self) -> list[int]:
        return [j for j in range(self.num_robots)
                if j != self.robot_id and self._in_range(j)]

    def _update_motion_state(self):
        now = time.time()
        peers = self._peers_in_range()

        # State transitions
        if self._state == DISPERSE:
            if now > self._disperse_until:
                self._state = EXPLORE
                self.get_logger().info(f'[Robot {self.robot_id}] → EXPLORE')

        elif self._state == EXPLORE:
            if peers:
                self._state = FLOCK
                self.get_logger().info(
                    f'[Robot {self.robot_id}] → FLOCK ({len(peers)} peers)')

        elif self._state == FLOCK:
            if not peers:
                self._state = EXPLORE

        # Compute angular velocity for current state
        half = self.world_size / 2.0 - 0.5
        near_wall = abs(self.pos_x) > half or abs(self.pos_y) > half

        if near_wall:
            # Always steer toward centre when near wall
            target = math.atan2(-self.pos_y, -self.pos_x)
            diff   = math.atan2(math.sin(target - self.yaw),
                                math.cos(target - self.yaw))
            self._current_angular_z = max(-self.walk_turn_max,
                                          min(self.walk_turn_max, diff * 2.0))

        elif self._state == FLOCK and peers:
            # Steer toward centroid of peers
            cx = sum(self.peer_positions[j][0] for j in peers) / len(peers)
            cy = sum(self.peer_positions[j][1] for j in peers) / len(peers)
            target = math.atan2(cy - self.pos_y, cx - self.pos_x)
            diff   = math.atan2(math.sin(target - self.yaw),
                                math.cos(target - self.yaw))
            self._current_angular_z = max(-self.walk_turn_max,
                                          min(self.walk_turn_max, diff * 1.5))

        elif self._state == DISPERSE:
            # Steer away from last known peer centroid
            if peers:
                cx = sum(self.peer_positions[j][0] for j in peers) / len(peers)
                cy = sum(self.peer_positions[j][1] for j in peers) / len(peers)
                # Away = opposite direction
                target = math.atan2(self.pos_y - cy, self.pos_x - cx)
                diff   = math.atan2(math.sin(target - self.yaw),
                                    math.cos(target - self.yaw))
                self._current_angular_z = max(-self.walk_turn_max,
                                              min(self.walk_turn_max, diff * 1.5))
            else:
                self._current_angular_z = random.uniform(
                    -self.walk_turn_max, self.walk_turn_max)

        else:
            # EXPLORE: Lévy flight — occasionally take a long straight run
            if self._levy_steps_left > 0:
                self._levy_steps_left -= 1
                diff = math.atan2(
                    math.sin(self._levy_heading - self.yaw),
                    math.cos(self._levy_heading - self.yaw))
                self._current_angular_z = max(-self.walk_turn_max,
                                              min(self.walk_turn_max, diff * 1.0))
            else:
                # Sample new Lévy step: exponentially distributed length
                self._levy_steps_left = int(random.expovariate(0.1))  # mean ~10 steps
                self._levy_heading    = random.uniform(-math.pi, math.pi)
                self._current_angular_z = random.uniform(
                    -self.walk_turn_max, self.walk_turn_max)

    def _publish_cmd_vel(self):
        # Slow down while flocking to allow sharing
        speed = self.walk_speed * 0.5 if self._state == FLOCK else self.walk_speed
        cmd = Twist()
        cmd.linear.x  = speed
        cmd.angular.z = self._current_angular_z
        self.cmd_pub.publish(cmd)

    # ── Proximity ────────────────────────────────────────────────────────────

    def _in_range(self, peer_id: int) -> bool:
        if peer_id not in self.peer_positions:
            return False
        dx = self.pos_x - self.peer_positions[peer_id][0]
        dy = self.pos_y - self.peer_positions[peer_id][1]
        return math.sqrt(dx * dx + dy * dy) < self.comm_range

    def _dist(self, peer_id: int) -> float:
        if peer_id not in self.peer_positions:
            return 999.0
        dx = self.pos_x - self.peer_positions[peer_id][0]
        dy = self.pos_y - self.peer_positions[peer_id][1]
        return math.sqrt(dx * dx + dy * dy)

    # ── Knowledge sharing ────────────────────────────────────────────────────

    def _share_timer_cb(self):
        peers_in_range = [j for j in range(self.num_robots)
                          if j != self.robot_id and self._in_range(j)]
        if not peers_in_range:
            return

        # Adaptive compression: use lowest similarity among peers in range
        # (send richest graph to the peer we differ from most)
        min_sim = min(
            self.peer_similarity.get(j, 0.0) for j in peers_in_range)
        retention = retention_for_similarity(min_sim)

        req = Compress.Request()
        req.agent_id        = int(self.robot_id)
        req.retention_ratio = float(retention)
        resp = self._call_engine(self._compress_cli, req)
        if resp is None:
            return

        payload = json.dumps({
            'sender_id': self.robot_id,
            'graph':     json.loads(resp.graph_json),
        })
        msg = String()
        msg.data = payload
        self.kg_pub.publish(msg)
        self.get_logger().info(
            f'[Robot {self.robot_id}] Broadcast to {len(peers_in_range)} peer(s) '
            f'retention={retention:.2f} (min_sim={min_sim:.3f})')

        # Transition to DISPERSE after sharing
        self._state          = DISPERSE
        self._disperse_until = time.time() + 10.0

    def _on_kg_share(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        sender_id = data.get('sender_id', -1)
        if sender_id == self.robot_id:
            return
        if not self._in_range(sender_id):
            return

        dist = self._dist(sender_id)
        req = Merge.Request()
        req.agent_id   = int(self.robot_id)
        req.sender_id  = int(sender_id)
        req.graph_json = json.dumps(data['graph'])
        resp = self._call_engine(self._merge_cli, req)
        if resp is None:
            return
        self.get_logger().info(
            f'[Robot {self.robot_id}] Merged from Robot {sender_id} '
            f'dist={dist:.2f}m gate={resp.gate:.3f} '
            f'z_norm={resp.z_norm:.4f}')

    # ── Engine service calls ─────────────────────────────────────────────────

    def _call_engine(self, client, request, timeout_sec: float = 5.0):
        """Blocking engine call that is safe inside a callback: call_async()
        plus a wait on the future. The ReentrantCallbackGroup and the
        MultiThreadedExecutor in main() guarantee a free thread delivers the
        response while this callback blocks. Returns the response, or None
        (with a warning logged) on unavailability/timeout/success=False."""
        if not client.service_is_ready():
            self.get_logger().warn(
                f'[Robot {self.robot_id}] {client.srv_name} unavailable '
                f'— skipping cycle')
            return None
        done   = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            future.cancel()
            self.get_logger().warn(
                f'[Robot {self.robot_id}] {client.srv_name} timeout '
                f'— skipping cycle')
            return None
        if future.exception() is not None:
            self.get_logger().warn(
                f'[Robot {self.robot_id}] {client.srv_name} failed: '
                f'{future.exception()}')
            return None
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(
                f'[Robot {self.robot_id}] {client.srv_name} error: '
                f'{resp.message}')
            return None
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = NSKRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Under `ros2 launch`, SIGINT may shut the context down at any moment,
        # even between an ok() check and the shutdown() call (check-then-act
        # race). Catch instead of check: a failed double-shutdown is a no-op.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
