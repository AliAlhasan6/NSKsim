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
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from nsk_swarm_interfaces.srv import Compress, Merge, SimilarityQuery

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

# /robot_N/cmd_vel: command stream — each message matters.
CMD_VEL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Movement states
EXPLORE  = 'explore'
FLOCK    = 'flock'
DISPERSE = 'disperse'

# Stuck-recovery phases (see NSKRobotNode._advance_recovery). Distinct from
# the movement states above: self._state is left untouched while a recovery
# is in progress (it's simply not consulted — see _publish_cmd_vel) and is
# reset to EXPLORE only once recovery completes.
RECOVER_REVERSE = 'recover_reverse'
RECOVER_ROTATE  = 'recover_rotate'

# Recovery tuning. The reverse distance, the post-escape FLOCK-suppression
# window and the repeat-escape thresholds are ROS parameters (escape_*, see
# PARAMS) so the escape can be tuned per world; the rotation ranges and
# tolerance below describe the maneuver's fixed geometry.
_RECOVERY_ROTATE_MIN_RAD          = math.radians(90.0)
_RECOVERY_ROTATE_MAX_RAD          = math.radians(270.0)
# Escalated (repeat-escape) rotation: draw over the full [90, 270] deg range
# but reject the +/-20 deg band around a straight reversal (180 deg), so a
# robot re-pinning at the same spot turns broadly away without ever heading
# straight back along the axis it came in on.
_RECOVERY_ROTATE_ESCALATE_MIN_RAD        = math.radians(90.0)
_RECOVERY_ROTATE_ESCALATE_MAX_RAD        = math.radians(270.0)
_RECOVERY_ROTATE_ESCALATE_EXCLUDE_LO_RAD = math.radians(160.0)
_RECOVERY_ROTATE_ESCALATE_EXCLUDE_HI_RAD = math.radians(200.0)
_RECOVERY_ROTATE_TOLERANCE_RAD           = math.radians(3.0)


def _discard_future(future):
    """Cancel a pending service future and mark any exception retrieved,
    silencing rclpy's 'exception was never retrieved' stderr noise."""
    future.cancel()
    try:
        future.exception()
    except Exception:
        pass


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
        'service_timeout_sec': 5.0,
        'spawn_x':        0.0,
        'spawn_y':        0.0,
        'stuck_window_sec': 4.0,
        'stuck_epsilon_m':  0.05,
        # Wall-pinning escape maneuver (odom-only recovery): reverse
        # distance, post-escape FLOCK-suppression window, and the space/time
        # thresholds that flag a re-pin at the same spot for escalation.
        'escape_reverse_m':         0.9,
        'escape_suppress_sec':      4.0,
        'escape_repeat_radius_m':   1.0,
        'escape_repeat_window_sec': 60.0,
    }

    def __init__(self):
        super().__init__('nsk_robot_node')

        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
        # Per-robot spawn offsets by robot id, for peer odometry (peer odoms
        # are relative to *their* spawn poses). A bare [] default is
        # mis-inferred as BYTE_ARRAY (an empty sequence trivially satisfies
        # "all elements are bytes"), and a type-only declaration
        # (Parameter.Type.DOUBLE_ARRAY, no value) has no fallback value at
        # all — get_parameter().value then raises
        # ParameterUninitializedException instead of returning None when
        # this node is started without spawn_xs/spawn_ys overrides. A
        # single-element float default is unambiguously DOUBLE_ARRAY and is
        # never itself read (every index below is length-guarded).
        self.declare_parameter('spawn_xs', [0.0])
        self.declare_parameter('spawn_ys', [0.0])

        self.robot_id       = self.get_parameter('robot_id').value
        self.num_robots     = self.get_parameter('num_robots').value
        self.comm_range     = self.get_parameter('comm_range').value
        self.share_interval = self.get_parameter('share_interval').value
        self.world_size     = self.get_parameter('world_size').value
        self.walk_speed     = self.get_parameter('walk_speed').value
        self.walk_turn_max  = self.get_parameter('walk_turn_max').value
        self.service_timeout_sec = self.get_parameter('service_timeout_sec').value
        self.spawn_x        = self.get_parameter('spawn_x').value
        self.spawn_y        = self.get_parameter('spawn_y').value
        # Unset arrays read back as None → zero offsets for every peer.
        self.spawn_xs = list(self.get_parameter('spawn_xs').value or [])
        self.spawn_ys = list(self.get_parameter('spawn_ys').value or [])
        self.stuck_window_sec = self.get_parameter('stuck_window_sec').value
        self.stuck_epsilon_m  = self.get_parameter('stuck_epsilon_m').value
        self.escape_reverse_m         = self.get_parameter(
            'escape_reverse_m').value
        self.escape_suppress_sec      = self.get_parameter(
            'escape_suppress_sec').value
        self.escape_repeat_radius_m   = self.get_parameter(
            'escape_repeat_radius_m').value
        self.escape_repeat_window_sec = self.get_parameter(
            'escape_repeat_window_sec').value

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

        # Stuck detector + recovery (wall-pinning escape; odom-only, no
        # perception — interior maze walls pin a diff-drive robot in a way
        # the near_wall boundary check, which only sees the outer walls,
        # never catches). _recovery_phase is None during normal operation;
        # self._state is left untouched while it isn't (nothing consults it
        # — see _publish_cmd_vel) and is reset to EXPLORE once recovery
        # completes, so the normal state machine re-evaluates FLOCK/EXPLORE
        # on its next tick.
        self._motion_samples: deque = deque()
        self._recovery_phase            = None
        self._recovery_phase_start_time  = 0.0
        self._recovery_phase_start_pos   = (0.0, 0.0)
        self._recovery_target_yaw        = 0.0

        # Per-recovery escape parameters. These start at the configured base
        # values and are recomputed on each trigger in _enter_recovery — an
        # escalated repeat escape doubles the reverse distance and the
        # suppression window for that one instance.
        self._escape_reverse_target_m     = self.escape_reverse_m
        self._escape_suppress_sec_current = self.escape_suppress_sec
        self._escape_escalated            = False

        # Post-escape FLOCK suppression: once a recovery completes, hold the
        # escape (post-rotation) heading and suppress FLOCK/EXPLORE
        # attraction until this deadline (0.0 = not suppressing) so the flock
        # target can't immediately steer the robot back into the wall it just
        # escaped. The near_wall centre-steer stays exempt (boundary guard).
        self._escape_suppress_until = 0.0
        self._escape_heading        = 0.0

        # Repeat-escape tracking: world-frame position and time of the last
        # escape trigger. A fresh trigger close to this in space and time is
        # treated as re-pinning at the same spot and escalates the escape.
        self._last_escape_pos  = None
        self._last_escape_time = 0.0

        # Time-cap fallback for each leg, sized generously (3x the nominal
        # duration at the configured speed) so a leg that can't quite reach
        # its geometric target — e.g. still partially pinned while
        # reversing — still ends deterministically instead of running
        # forever. The reverse cap tracks the current (possibly doubled)
        # reverse target and is recomputed per trigger in _enter_recovery.
        self._reverse_time_cap_sec = 3.0 * (self.escape_reverse_m
                                            / max(self.walk_speed, 1e-3))
        self._rotate_time_cap_sec  = 3.0 * (_RECOVERY_ROTATE_MAX_RAD
                                            / max(self.walk_turn_max, 1e-3))

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
        self._sim_cli      = self.create_client(
            SimilarityQuery, '/nsk/similarity_query',
            callback_group=self._cb_group)
        for cli in (self._compress_cli, self._merge_cli, self._sim_cli):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f'[Robot {self.robot_id}] Waiting for {cli.srv_name} ...')

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', CMD_VEL_QOS)
        self.kg_pub  = self.create_publisher(String, '/kg_share', KG_SHARE_QOS)

        # Subscriptions
        self.create_subscription(Odometry, f'/robot_{self.robot_id}/odom',
                                 self._odom_cb, ODOM_QOS)
        for j in range(self.num_robots):
            if j != self.robot_id:
                self.create_subscription(
                    Odometry, f'/robot_{j}/odom',
                    lambda msg, peer=j: self._peer_odom_cb(msg, peer), ODOM_QOS)
        self.create_subscription(String, '/kg_share', self._on_kg_share,
                                 KG_SHARE_QOS, callback_group=self._cb_group)

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
        # DiffDrive odometry is relative to the spawn pose; shift into the
        # world frame so positions are comparable across robots. Yaw is left
        # spawn-relative: distance/comm-range math never uses it.
        self.pos_x = self.spawn_x + msg.pose.pose.position.x
        self.pos_y = self.spawn_y + msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _peer_odom_cb(self, msg: Odometry, peer_id: int):
        ox = self.spawn_xs[peer_id] if peer_id < len(self.spawn_xs) else 0.0
        oy = self.spawn_ys[peer_id] if peer_id < len(self.spawn_ys) else 0.0
        self.peer_positions[peer_id] = (
            ox + msg.pose.pose.position.x,
            oy + msg.pose.pose.position.y)

    # ── Motion state machine ─────────────────────────────────────────────────

    def _peers_in_range(self) -> list[int]:
        return [j for j in range(self.num_robots)
                if j != self.robot_id and self._in_range(j)]

    def _update_motion_state(self):
        # Recovery preempts the normal state machine entirely, including
        # the near_wall centre-steer below: while a recovery leg is under
        # way, _publish_cmd_vel drives the robot directly and this method
        # has nothing to decide.
        if self._recovery_phase is not None:
            return
        if self._check_stuck():
            self._enter_recovery()
            return

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
            # Always steer toward centre when near wall — a legitimate
            # boundary guard, kept live even during post-escape suppression.
            target = math.atan2(-self.pos_y, -self.pos_x)
            diff   = math.atan2(math.sin(target - self.yaw),
                                math.cos(target - self.yaw))
            self._current_angular_z = max(-self.walk_turn_max,
                                          min(self.walk_turn_max, diff * 2.0))

        elif now < self._escape_suppress_until:
            # Post-escape suppression: hold the escape (post-rotation)
            # heading instead of FLOCK/EXPLORE steering, so the robot commits
            # to leaving the area before its flock target can steer it
            # straight back into the wall it just escaped.
            diff = math.atan2(math.sin(self._escape_heading - self.yaw),
                              math.cos(self._escape_heading - self.yaw))
            self._current_angular_z = max(-self.walk_turn_max,
                                          min(self.walk_turn_max, diff * 1.5))

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
        if self._recovery_phase is not None:
            cmd = self._advance_recovery()
        else:
            # Slow down while flocking to allow sharing
            speed = (self.walk_speed * 0.5 if self._state == FLOCK
                    else self.walk_speed)
            cmd = Twist()
            cmd.linear.x  = speed
            cmd.angular.z = self._current_angular_z
        self.cmd_pub.publish(cmd)
        self._record_motion_sample(cmd.linear.x)

    # ── Stuck detector + recovery ────────────────────────────────────────────

    def _record_motion_sample(self, commanded_linear_x: float):
        """Track (time, position, commanded linear speed) over a trailing
        stuck_window_sec window, for _check_stuck. Called every
        _publish_cmd_vel tick (0.1 s) — fine enough that a transient
        zero-speed command (e.g. mid-recovery) is never missed."""
        now = time.time()
        self._motion_samples.append((now, self.pos_x, self.pos_y,
                                     commanded_linear_x))
        cutoff = now - self.stuck_window_sec
        while self._motion_samples and self._motion_samples[0][0] < cutoff:
            self._motion_samples.popleft()

    def _check_stuck(self) -> bool:
        """True if commanded linear speed has been nonzero for the whole
        trailing stuck_window_sec window, but world-frame displacement
        over that same window is under stuck_epsilon_m — i.e. the robot is
        being driven but isn't actually moving (wall-pinned)."""
        samples = self._motion_samples
        if len(samples) < 2:
            return False   # not enough history yet to judge a full window
        oldest_t, oldest_x, oldest_y, _ = samples[0]
        if time.time() - oldest_t < self.stuck_window_sec:
            return False   # window not yet fully spanned
        if any(speed <= 0.0 for _, _, _, speed in samples):
            return False   # motion was intentionally paused somewhere in it
        displacement = math.hypot(self.pos_x - oldest_x, self.pos_y - oldest_y)
        return displacement < self.stuck_epsilon_m

    def _enter_recovery(self):
        now = time.time()
        # Repeat-escape detection: a fresh trigger close in space and time to
        # the previous one means the earlier escape didn't take — the robot
        # re-pinned at (nearly) the same spot — so escalate this recovery.
        escalate = (
            self._last_escape_pos is not None
            and now - self._last_escape_time <= self.escape_repeat_window_sec
            and math.hypot(self.pos_x - self._last_escape_pos[0],
                           self.pos_y - self._last_escape_pos[1])
                <= self.escape_repeat_radius_m)

        if escalate:
            # Escalated instance: reverse twice as far, suppress FLOCK twice
            # as long, and rotate from the wider, opposite-biased range so
            # this escape doesn't re-commit to the heading that just failed.
            self._escape_reverse_target_m     = self.escape_reverse_m * 2.0
            self._escape_suppress_sec_current = self.escape_suppress_sec * 2.0
            self._escape_escalated            = True
            self.get_logger().info(
                f'[Robot {self.robot_id}] repeat stuck near '
                f'({self.pos_x:.2f}, {self.pos_y:.2f}) — escalating escape')
        else:
            self._escape_reverse_target_m     = self.escape_reverse_m
            self._escape_suppress_sec_current = self.escape_suppress_sec
            self._escape_escalated            = False
            self.get_logger().info(
                f'[Robot {self.robot_id}] stuck at '
                f'({self.pos_x:.2f}, {self.pos_y:.2f}) — escaping')

        self._last_escape_pos  = (self.pos_x, self.pos_y)
        self._last_escape_time = now

        self._recovery_phase            = RECOVER_REVERSE
        self._recovery_phase_start_time = now
        self._recovery_phase_start_pos  = (self.pos_x, self.pos_y)
        # Size the reverse leg's time-cap to the (possibly doubled) target.
        self._reverse_time_cap_sec = 3.0 * (self._escape_reverse_target_m
                                            / max(self.walk_speed, 1e-3))
        # Defensive: _update_motion_state never calls _check_stuck() again
        # while a recovery is in progress, but a clear guarantees no stale
        # pre-recovery samples could ever factor into a later check.
        self._motion_samples.clear()

    def _advance_recovery(self) -> Twist:
        """Called every _publish_cmd_vel tick (0.1 s) while a recovery is
        active: drives the current leg and, on completion (by distance/
        angle or by the time-cap fallback), advances to the next leg or
        exits recovery outright. Reverse then rotate, straight through —
        no dead tick in between."""
        cmd = Twist()
        now = time.time()

        if self._recovery_phase == RECOVER_REVERSE:
            start_x, start_y = self._recovery_phase_start_pos
            reversed_dist = math.hypot(self.pos_x - start_x,
                                       self.pos_y - start_y)
            elapsed = now - self._recovery_phase_start_time
            if (reversed_dist >= self._escape_reverse_target_m
                    or elapsed >= self._reverse_time_cap_sec):
                # Reverse leg done — pick the escape rotation (unseeded:
                # motion is deliberately unseeded) and fall through to start
                # the rotate leg in this same tick.
                if self._escape_escalated:
                    # Turn broadly away but never straight back along the
                    # incoming axis: uniform over [90, 270] deg, rejecting
                    # the +/-20 deg band around a straight reversal (180 deg).
                    while True:
                        turn = random.uniform(
                            _RECOVERY_ROTATE_ESCALATE_MIN_RAD,
                            _RECOVERY_ROTATE_ESCALATE_MAX_RAD)
                        if not (_RECOVERY_ROTATE_ESCALATE_EXCLUDE_LO_RAD
                                <= turn
                                <= _RECOVERY_ROTATE_ESCALATE_EXCLUDE_HI_RAD):
                            break
                else:
                    turn = random.uniform(_RECOVERY_ROTATE_MIN_RAD,
                                          _RECOVERY_ROTATE_MAX_RAD)
                self._recovery_target_yaw = self.yaw + turn
                self._recovery_phase = RECOVER_ROTATE
                self._recovery_phase_start_time = now
            else:
                cmd.linear.x  = -self.walk_speed
                cmd.angular.z = 0.0
                return cmd

        # RECOVER_ROTATE (reached either directly or by fallthrough above).
        diff = math.atan2(math.sin(self._recovery_target_yaw - self.yaw),
                          math.cos(self._recovery_target_yaw - self.yaw))
        elapsed = now - self._recovery_phase_start_time
        if (abs(diff) < _RECOVERY_ROTATE_TOLERANCE_RAD
                or elapsed >= self._rotate_time_cap_sec):
            self._exit_recovery()
            return cmd   # zero Twist: hold still for the rest of this tick
        cmd.linear.x  = 0.0
        cmd.angular.z = math.copysign(self.walk_turn_max, diff)
        return cmd

    def _exit_recovery(self):
        """Recovery complete: latch the escape (post-rotation) heading and
        open the FLOCK-suppression window so the robot commits to leaving the
        area, then resume the normal state machine (it settles into FLOCK or
        EXPLORE on its own next tick) and re-arm the stuck detector with a
        clean window."""
        self._recovery_phase = None
        self._state = EXPLORE
        self._escape_heading        = self.yaw
        self._escape_suppress_until = (time.time()
                                       + self._escape_suppress_sec_current)
        # Escalation is per-instance: clear the flag now the recovery is
        # done. The next trigger re-derives it from _last_escape_pos/_time,
        # so a later escape with no nearby repeat starts un-escalated.
        self._escape_escalated = False
        self._motion_samples.clear()

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

        # Refresh peer similarities from the engine before choosing the
        # retention ratio. On failure keep the cached values — stale beats
        # resetting every peer to the pessimistic 0.0 default.
        sim_req = SimilarityQuery.Request()
        sim_req.agent_ids = [int(self.robot_id)] + peers_in_range
        sim_resp = self._call_engine(self._sim_cli, sim_req)
        if sim_resp is None:
            self.get_logger().warn(
                f'[Robot {self.robot_id}] similarity refresh failed '
                f'— using cached peer similarities')
        else:
            # Matrix rows/columns follow the request's agent_ids order
            # (same convention as convergence_monitor): row 0 is this
            # robot, column k+1 is peers_in_range[k].
            matrix = json.loads(sim_resp.matrix_json)
            for k, j in enumerate(peers_in_range):
                self.peer_similarity[j] = float(matrix[0][k + 1])

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
                f'[Robot {self.robot_id}] {client.srv_name} unavailable '
                f'— skipping cycle')
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
                    f'[Robot {self.robot_id}] {client.srv_name} timeout '
                    f'— skipping cycle')
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
                f'[Robot {self.robot_id}] {client.srv_name} failed: {exc}')
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
