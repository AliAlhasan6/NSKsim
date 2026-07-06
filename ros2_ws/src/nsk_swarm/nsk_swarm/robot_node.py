#!/usr/bin/env python3
"""
robot_node.py — NSK Swarm ROS 2 robot node (v2).
Flocking movement + adaptive compression.
System Python only — no PyTorch.
"""

import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import zmq

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
        'zmq_endpoint':   'ipc:///tmp/nsk_engine_0',
        'world_size':     20.0,
        'walk_speed':     0.15,
        'walk_turn_max':  0.5,
        'zmq_timeout_ms': 2000,
    }

    def __init__(self):
        super().__init__('nsk_robot_node')

        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)

        self.robot_id       = self.get_parameter('robot_id').value
        self.num_robots     = self.get_parameter('num_robots').value
        self.comm_range     = self.get_parameter('comm_range').value
        self.share_interval = self.get_parameter('share_interval').value
        self.zmq_endpoint   = self.get_parameter('zmq_endpoint').value
        self.world_size     = self.get_parameter('world_size').value
        self.walk_speed     = self.get_parameter('walk_speed').value
        self.walk_turn_max  = self.get_parameter('walk_turn_max').value
        self.zmq_timeout_ms = self.get_parameter('zmq_timeout_ms').value

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

        # ZMQ
        self._zmq_ctx    = zmq.Context()
        self._zmq_socket = self._zmq_ctx.socket(zmq.REQ)
        self._zmq_socket.setsockopt(zmq.LINGER, 0)
        self._zmq_socket.connect(self.zmq_endpoint)
        self._zmq_poller = zmq.Poller()
        self._zmq_poller.register(self._zmq_socket, zmq.POLLIN)

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
        self.create_subscription(String, '/kg_share', self._on_kg_share, 10)

        # Timers
        self.create_timer(2.0,                self._update_motion_state)
        self.create_timer(0.1,                self._publish_cmd_vel)
        self.create_timer(self.share_interval, self._share_timer_cb)

        self.get_logger().info(
            f'[Robot {self.robot_id}] Started. Endpoint: {self.zmq_endpoint}')

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

        resp = self._zmq_request({
            'type':            'compress_request',
            'agent_id':        self.robot_id,
            'retention_ratio': retention,
        })
        if resp is None or resp.get('type') != 'compressed_graph':
            return

        payload = json.dumps({
            'sender_id': self.robot_id,
            'graph':     resp['graph'],
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
        resp = self._zmq_request({
            'type':      'merge_request',
            'agent_id':  self.robot_id,
            'graph':     data['graph'],
            'sender_id': sender_id,
        })
        if resp is None:
            return
        if resp.get('type') == 'merge_done':
            self.get_logger().info(
                f'[Robot {self.robot_id}] Merged from Robot {sender_id} '
                f'dist={dist:.2f}m gate={resp.get("gate",0.5):.3f} '
                f'z_norm={resp.get("z_norm",0.0):.4f}')

    # ── ZMQ ──────────────────────────────────────────────────────────────────

    def _zmq_request(self, payload: dict) -> dict | None:
        try:
            self._zmq_socket.send_string(json.dumps(payload))
            if self._zmq_poller.poll(self.zmq_timeout_ms):
                return json.loads(self._zmq_socket.recv_string())
            else:
                self.get_logger().warn(
                    f'[Robot {self.robot_id}] ZMQ timeout — skipping cycle')
                self._zmq_socket.close()
                self._zmq_socket = self._zmq_ctx.socket(zmq.REQ)
                self._zmq_socket.setsockopt(zmq.LINGER, 0)
                self._zmq_socket.connect(self.zmq_endpoint)
                self._zmq_poller = zmq.Poller()
                self._zmq_poller.register(self._zmq_socket, zmq.POLLIN)
                return None
        except zmq.ZMQError as e:
            self.get_logger().warn(f'[Robot {self.robot_id}] ZMQ error: {e}')
            return None

    def destroy_node(self):
        self._zmq_socket.close()
        self._zmq_ctx.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NSKRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
