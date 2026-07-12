#!/usr/bin/env python3
"""
convergence_monitor.py — NSK Swarm convergence monitoring node.

Standalone ROS 2 node. No PyTorch.
- Queries NSK engine via the /nsk/similarity_query service for pairwise z* similarity
- Publishes Float32 to /nsk/convergence
- Publishes MarkerArray to /nsk/similarity_markers for RViz2
- Listens to /kg_share to track merge events
- Logs formatted convergence reports via the node logger
"""

import json
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from nsk_swarm_interfaces.srv import SimilarityQuery

# ── QoS profiles ─────────────────────────────────────────────────────────────

# /kg_share: RELIABLE — a lost broadcast is a lost merge, and the topic is
# low-rate (one message per share_interval) so reliability is cheap.
# VOLATILE — stale graphs must not be merged by late joiners, so no
# durable cache.
KG_SHARE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# /robot_N/odom: latest-pose-only semantics — best-effort matches the sensor
# convention and avoids the reliable-sub-vs-best-effort-pub incompatibility
# with the gz bridge. The ros_gz bridge publishes odom with default RELIABLE
# QoS, which is compatible with a best-effort subscriber (reliable pub +
# best-effort sub connects; only best-effort pub + reliable sub fails).
ODOM_QOS = qos_profile_sensor_data

# /nsk/convergence and /nsk/similarity_markers: low-rate monitoring outputs
# for RViz2 and loggers — explicit spelling of the profile the bare depth-10
# argument implied.
MONITOR_PUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def _discard_future(future):
    """Cancel a pending service future and mark any exception retrieved,
    silencing rclpy's 'exception was never retrieved' stderr noise."""
    future.cancel()
    try:
        future.exception()
    except Exception:
        pass


def convergence_step(mean_sim: float, baseline_sim: float, delta: float,
                     has_prev: bool, consec_stable: int,
                     already_converged: bool, has_merges: bool,
                     min_rise: float, stability_eps: float
                     ) -> tuple[float, int, bool]:
    """One monitor cycle's convergence decision, as pure math.

    Convergence requires at least one merge observed, a rise over the
    first reading (baseline varies per run, so an absolute threshold
    can be met at t=0 with zero merges), and a stable tail.

    Returns (rise, new_consec_stable, newly_converged).
    """
    rise = mean_sim - baseline_sim
    if has_prev and abs(delta) < stability_eps:
        consec_stable += 1
    else:
        consec_stable = 0

    newly_converged = (not already_converged
                       and has_merges
                       and rise >= min_rise
                       and consec_stable >= 3)
    return rise, consec_stable, newly_converged


# Robot colours (R, G, B) matching SDF definitions
ROBOT_COLOURS = {
    0: (0.2, 0.4, 1.0),   # blue
    1: (0.2, 0.8, 0.2),   # green
    2: (1.0, 0.5, 0.1),   # orange
    3: (0.8, 0.2, 0.2),   # red
    4: (0.6, 0.2, 0.8),   # purple
}


class ConvergenceMonitorNode(Node):

    def __init__(self):
        super().__init__('convergence_monitor')

        # Parameters
        self.declare_parameter('num_robots',           5)
        self.declare_parameter('monitor_interval',     10.0)
        self.declare_parameter('comm_range',           3.0)
        # Declared only so launch files that still pass it don't fail;
        # convergence is now judged by min_rise/stability_eps instead.
        self.declare_parameter('convergence_threshold', 0.25)
        self.declare_parameter('min_rise',             0.03)
        self.declare_parameter('stability_eps',        0.005)
        self.declare_parameter('service_timeout_sec',  5.0)

        self.num_robots           = self.get_parameter('num_robots').value
        self.monitor_interval     = self.get_parameter('monitor_interval').value
        self.comm_range           = self.get_parameter('comm_range').value
        self.min_rise             = self.get_parameter('min_rise').value
        self.stability_eps        = self.get_parameter('stability_eps').value
        self.service_timeout_sec  = self.get_parameter('service_timeout_sec').value

        # State
        self.merge_counts: dict[tuple[int, int], int] = {}
        self.similarity_history: list[float] = []
        self.start_time = time.time()
        self.robot_positions: dict[int, tuple[float, float]] = {}
        self.baseline_sim: float | None = None
        self._consec_stable = 0
        self._converged = False

        # Engine service client.
        # Async-safety: _monitor_cb blocks on the engine response, so the
        # timer and the client share one ReentrantCallbackGroup, and main()
        # spins a MultiThreadedExecutor. A blocked callback therefore never
        # prevents another executor thread from delivering the service
        # response (no deadlock, unlike a synchronous call() in a default
        # single-threaded setup).
        self._cb_group = ReentrantCallbackGroup()
        self._sim_cli  = self.create_client(
            SimilarityQuery, '/nsk/similarity_query',
            callback_group=self._cb_group)
        while not self._sim_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'[NSK Monitor] Waiting for {self._sim_cli.srv_name} ...')
        self._check_duplicate_servers()

        # Publishers
        self.conv_pub    = self.create_publisher(
            Float32,     '/nsk/convergence',        MONITOR_PUB_QOS)
        self.marker_pub  = self.create_publisher(
            MarkerArray, '/nsk/similarity_markers', MONITOR_PUB_QOS)

        # Subscriptions
        self.create_subscription(String, '/kg_share', self._on_kg_share,
                                 KG_SHARE_QOS)
        for i in range(self.num_robots):
            self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',
                lambda msg, rid=i: self._odom_cb(msg, rid),
                ODOM_QOS)

        # Timer
        self.create_timer(self.monitor_interval, self._monitor_cb,
                          callback_group=self._cb_group)

        self.get_logger().info('[NSK Monitor] Node started.')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, robot_id: int):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_positions[robot_id] = (x, y)

    def _on_kg_share(self, msg: String):
        """Count merge events from /kg_share traffic."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        sender_id = data.get('sender_id', -1)
        if sender_id < 0:
            return
        # Each receiver in range counts as a merge; we track sender here
        # (robot_node logs the actual per-robot merges)
        for receiver in range(self.num_robots):
            if receiver == sender_id:
                continue
            if self._peers_in_range(sender_id, receiver):
                key = (sender_id, receiver)
                self.merge_counts[key] = self.merge_counts.get(key, 0) + 1

    def _peers_in_range(self, a: int, b: int) -> bool:
        if a not in self.robot_positions or b not in self.robot_positions:
            return False
        ax, ay = self.robot_positions[a]
        bx, by = self.robot_positions[b]
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2) < self.comm_range

    # ── Monitor callback ─────────────────────────────────────────────────────

    def _check_duplicate_servers(self):
        """Two nsk_engine processes load-balance requests between themselves
        and their state diverges silently, so more than one server on the
        similarity service makes every reading suspect."""
        n = self.count_services(self._sim_cli.srv_name)
        if n > 1:
            self.get_logger().error(
                f'Multiple NSK engine servers detected ({n}); '
                'results are unreliable.')

    def _monitor_cb(self):
        self._check_duplicate_servers()

        req = SimilarityQuery.Request()
        req.agent_ids = list(range(self.num_robots))
        resp = self._call_engine(self._sim_cli, req)
        if resp is None:
            return

        mean_sim = resp.mean_sim
        matrix   = json.loads(resp.matrix_json)
        elapsed  = int(time.time() - self.start_time)

        if self.baseline_sim is None:
            self.baseline_sim = mean_sim

        # Publish convergence float
        msg = Float32()
        msg.data = float(mean_sim)
        self.conv_pub.publish(msg)

        # Publish RViz2 markers
        markers = self._make_markers(matrix)
        if markers:
            arr = MarkerArray()
            arr.markers = markers
            self.marker_pub.publish(arr)

        # History and trend
        self.similarity_history.append(mean_sim)
        has_prev = len(self.similarity_history) > 1
        delta    = mean_sim - self.similarity_history[-2] if has_prev else 0.0
        trend    = '↑ converging' if delta > 0.005 else (
                   '↓ diverging'  if delta < -0.005 else '→ stable')

        rise, self._consec_stable, newly_converged = convergence_step(
            mean_sim, self.baseline_sim, delta, has_prev,
            self._consec_stable, self._converged, bool(self.merge_counts),
            self.min_rise, self.stability_eps)
        if newly_converged:
            self._converged = True

        # Active pairs
        pair_strs = [f'({a},{b})×{c}'
                     for (a, b), c in sorted(self.merge_counts.items())]
        pairs_str = '  '.join(pair_strs) if pair_strs else 'none'

        status = ('CONVERGED ✓ (monitoring continues)' if self._converged
                  else 'NOT YET')

        self.get_logger().info(
            '\n' + '═' * 52 + '\n'
            f'[NSK Monitor  t={elapsed:03d}s]  '
            f'Mean pairwise sim: {mean_sim:.4f}\n'
            f'  Δ from last:  {delta:+.4f}   Trend: {trend}\n'
            f'  Active pairs: {pairs_str}\n'
            f'  Baseline: {self.baseline_sim:.4f}   '
            f'Rise: {rise:+.4f} (min {self.min_rise})   '
            f'Stable: {self._consec_stable}/3\n'
            f'  Status: {status}\n'
            + '═' * 52)

        if newly_converged:
            self._log_final_report(elapsed, mean_sim)

    def _log_final_report(self, elapsed: int, final_sim: float):
        lines = [
            '\n' + '★' * 52,
            f'[NSK Monitor] CONVERGENCE REACHED at t={elapsed}s',
            f'  Final mean pairwise similarity: {final_sim:.4f}',
            f'  Rise over baseline: {final_sim - self.baseline_sim:+.4f}',
            '  Merge events:',
        ]
        for (sender, receiver), count in sorted(self.merge_counts.items()):
            lines.append(f'    Robot {sender} → Robot {receiver}: {count} merges')
        total = sum(self.merge_counts.values())
        lines.append(f'  Total merge events: {total}')
        lines.append('★' * 52)
        self.get_logger().info('\n'.join(lines))

    # ── Marker construction ──────────────────────────────────────────────────

    def _make_markers(self, similarity_matrix: list) -> list:
        markers = []
        stamp   = self.get_clock().now().to_msg()
        mid     = 0

        # Robot cylinders — colour by mean similarity to others
        for robot_id in range(self.num_robots):
            if robot_id not in self.robot_positions:
                continue
            x, y = self.robot_positions[robot_id]

            # Mean similarity of this robot to all others (off-diagonal row mean)
            row = similarity_matrix[robot_id]
            others = [row[j] for j in range(self.num_robots) if j != robot_id]
            row_mean = sum(others) / len(others) if others else 0.0

            # Lerp red (low) → green (high) based on normalised sim
            t   = max(0.0, min(1.0, (row_mean + 1.0) / 2.0))
            r_c = 1.0 - t
            g_c = t
            b_c = 0.0

            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp    = stamp
            m.ns              = 'robots'
            m.id              = mid; mid += 1
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.1
            m.color.r = r_c
            m.color.g = g_c
            m.color.b = b_c
            m.color.a = 0.85
            markers.append(m)

            # Robot ID text label
            t_m = Marker()
            t_m.header.frame_id = 'odom'
            t_m.header.stamp    = stamp
            t_m.ns              = 'labels'
            t_m.id              = mid; mid += 1
            t_m.type            = Marker.TEXT_VIEW_FACING
            t_m.action          = Marker.ADD
            t_m.pose.position.x = x
            t_m.pose.position.y = y
            t_m.pose.position.z = 0.3
            t_m.scale.z         = 0.25
            t_m.color.r = 1.0; t_m.color.g = 1.0; t_m.color.b = 1.0; t_m.color.a = 1.0
            t_m.text = f'R{robot_id}\n{row_mean:.2f}'
            markers.append(t_m)

        # Communication range lines between pairs currently in range
        for i in range(self.num_robots):
            for j in range(i + 1, self.num_robots):
                if not self._peers_in_range(i, j):
                    continue
                if i not in self.robot_positions or j not in self.robot_positions:
                    continue
                xi, yi = self.robot_positions[i]
                xj, yj = self.robot_positions[j]

                line = Marker()
                line.header.frame_id = 'odom'
                line.header.stamp    = stamp
                line.ns              = 'comm_links'
                line.id              = mid; mid += 1
                line.type            = Marker.LINE_STRIP
                line.action          = Marker.ADD
                line.scale.x         = 0.02
                line.color.r = 0.5; line.color.g = 0.5; line.color.b = 1.0
                line.color.a = 0.6

                p1 = Point(); p1.x = xi; p1.y = yi; p1.z = 0.05
                p2 = Point(); p2.x = xj; p2.y = yj; p2.z = 0.05
                line.points = [p1, p2]
                markers.append(line)

        # Mean similarity text at top of world
        if self.similarity_history:
            mean = self.similarity_history[-1]
            hud = Marker()
            hud.header.frame_id = 'odom'
            hud.header.stamp    = stamp
            hud.ns              = 'hud'
            hud.id              = mid; mid += 1
            hud.type            = Marker.TEXT_VIEW_FACING
            hud.action          = Marker.ADD
            hud.pose.position.x = 0.0
            hud.pose.position.y = 11.0
            hud.pose.position.z = 0.5
            hud.scale.z         = 0.5
            hud.color.r = 1.0; hud.color.g = 1.0; hud.color.b = 0.0; hud.color.a = 1.0
            hud.text = f'Mean z* sim: {mean:.4f}'
            markers.append(hud)

        return markers

    # ── Engine service calls ─────────────────────────────────────────────────

    def _call_engine(self, client, request, timeout_sec: float | None = None):
        """Blocking engine call that is safe inside a callback: call_async()
        plus a wait on the future. The ReentrantCallbackGroup and the
        MultiThreadedExecutor in main() guarantee a free thread delivers the
        response while this callback blocks. Returns the response, or None
        (with a warning logged) on unavailability/timeout/success=False.
        Aborts quietly if rclpy shuts down mid-wait (not an error)."""
        if timeout_sec is None:
            timeout_sec = self.service_timeout_sec
        if not client.service_is_ready():
            self.get_logger().warn(
                f'[NSK Monitor] {client.srv_name} unavailable — skipping cycle')
            return None
        done   = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        # Wait in short slices so a shutdown mid-call aborts promptly
        # instead of holding an executor thread for the full timeout.
        deadline = time.monotonic() + timeout_sec
        while not done.is_set():
            if not rclpy.ok():
                _discard_future(future)
                return None
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                _discard_future(future)
                self.get_logger().warn(
                    f'[NSK Monitor] {client.srv_name} timeout — skipping cycle')
                return None
            done.wait(min(0.2, remaining))
        # Retrieval guarantee: fetch the exception exactly once, before any
        # of the returns below — a Future collected with an unfetched
        # exception makes rclpy print 'exception was never retrieved'.
        # Fetching from a successfully-completed future is harmless.
        try:
            exc = future.exception()
        except Exception as fetch_err:
            exc = fetch_err
        if exc is not None:
            self.get_logger().warn(
                f'[NSK Monitor] {client.srv_name} failed: {exc}')
            return None
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(
                f'[NSK Monitor] {client.srv_name} error: {resp.message}')
            return None
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ConvergenceMonitorNode()
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
