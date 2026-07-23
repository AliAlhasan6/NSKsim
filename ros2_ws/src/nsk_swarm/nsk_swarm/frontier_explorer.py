#!/usr/bin/env python3
"""frontier_explorer.py — namespaced frontier-based autonomous explorer.

Drives one robot (default robot_0) to map a maze under its own steam: it
subscribes to the live SLAM OccupancyGrid, detects frontier cells (free cells
that border unknown space), clusters them, and sends the nearest reachable
cluster centroid to Nav2 as a NavigateToPose goal in the robot's map frame.
After each goal completes (or aborts) it recomputes frontiers and sends the
next; when no frontiers remain the maze is considered covered and it stops.

Closes the loop: explore -> plan (Nav2) -> drive (/robot_N/cmd_vel) -> SLAM maps
new area -> new frontiers -> repeat.

Namespace/robot are parametrized via the `robot_id` parameter (default 0) plus
the launch-applied namespace, so the same node serves robots 1-4 unchanged.
Assumes swarm_sim.launch.py is already running with nav_robots:=[robot_id] (so
the target robot's wander driver is muted) and explore.launch.py's SLAM + Nav2.

Concurrency (why this is structured with two nodes)
---------------------------------------------------
BasicNavigator drives its goals by repeatedly spinning ITS node on rclpy's
single global executor (goToPose/isTaskComplete call spin_until_future_complete
on `self`). If the explorer also relied on that same single executor to service
its OWN map/TF/clock callbacks — via `spin_once(self, ...)` and a sim-time
`_settle()` — those callbacks get starved whenever the navigator is churning
the executor, and any sim-time-deadline wait (`while self._now() < end`) then
never advances and hangs SILENTLY with no timeout and no log. This is the
project's known single-threaded-executor starvation trap (see TECHNICAL.md).

Fix: the explorer's own subscriptions (map + TF + clock) live on a dedicated
`_SensorNode` that is spun continuously by a MultiThreadedExecutor in a daemon
thread, so those callbacks ALWAYS fire regardless of what the navigator is
doing. The main explore() loop never hand-spins its own node; it reads the
always-fresh map/pose from the sensor node and every wait is bounded by a
WALL-clock deadline (immune to a stalled sim clock) and emits a heartbeat, so a
stall can never again be invisible.
"""

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

UNKNOWN = -1          # OccupancyGrid unknown cell
OCC_THRESH = 50       # cost >= this counts as an obstacle; below (and >=0) is free
MIN_CLUSTER = 4       # discard frontier blobs smaller than this many cells (noise)
BLACKLIST_RADIUS = 0.4  # m; a new centroid within this of a blacklisted goal is skipped

# ── goal gating ─────────────────────────────────────────────────────────────
# The robot only maps new area if it physically MOVES. A frontier centroid sits
# on the free side of the known/unknown boundary, so it can land right next to
# (or on) the robot. A goal there is inside Nav2's xy_goal_tolerance, so Nav2
# reports "reached" in ~30 ms with ~zero velocity and the robot never moves —
# the map, and hence the frontier set, never changes, and the explorer re-picks
# the same goal at loop speed (~40 Hz). These thresholds break that spin.
MIN_GOAL_DIST = 0.5      # m; a frontier nearer than this to the robot is inside goal
                         # tolerance — a goal AT it produces no motion, so never send it as-is
GOAL_PROJECT_DIST = 0.7  # m; when every remaining frontier is too close, project the goal this
                         # far out along robot->frontier so Nav2 must actually drive to reach it
RESEND_RADIUS = 0.2      # m; a newly-selected goal within this of the last-sent one is a re-send
MIN_MOVE = 0.1           # m; if the robot travelled less than this during a goal, the frontier
                         # was unproductive (inside tolerance) and gets blacklisted
MIN_GOAL_PERIOD = 2.0    # s; floor on the interval between goal dispatches (rate-limit the loop)

# ── wait bounds (wall clock; no wait may block silently forever) ─────────────
FIRST_MAP_TIMEOUT = 30.0   # s; give up waiting for the very first SLAM map
MAP_REFRESH_TIMEOUT = 5.0  # s; after a goal, wait this long for SLAM to publish a NEWER map
                           # (so new frontiers appear) before proceeding with the map we hold
TF_WAIT = 0.5              # s; nap between retries while the map->base TF isn't ready yet
GOAL_TIMEOUT = 120.0       # s; cancel a goal that hasn't completed in this long (never hang on Nav2)
HEARTBEAT_PERIOD = 2.0     # s; min interval between "still waiting on X" heartbeat logs


class _SensorNode(Node):
    """Owns the explorer's map/TF/clock callbacks on its own executor.

    Kept separate from the BasicNavigator node so a MultiThreadedExecutor can
    spin it continuously in a background thread — its callbacks fire no matter
    what the navigator is doing on the main thread, so the map and robot pose
    the explorer reads are always fresh (no executor starvation).
    """

    def __init__(self, robot_id: int):
        # use_sim_time from birth so TF stamps / clock line up with /clock.
        super().__init__(
            'frontier_sensor', namespace=f'robot_{robot_id}',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        ns = f'robot_{robot_id}'
        self.map_frame = f'{ns}/map'
        self.base_frame = f'{ns}/base_footprint'
        cbg = ReentrantCallbackGroup()

        # SLAM publishes a latched (transient-local) map; match its QoS.
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lock = threading.Lock()
        self._map = None
        self._map_seq = 0     # bumped on every map — lets the loop detect a FRESH map
        self.create_subscription(
            OccupancyGrid, f'/{ns}/map', self._on_map, map_qos, callback_group=cbg)

        # Robot pose comes from TF (SLAM's map->odom + bridged odom->base). The
        # listener rides this node's executor, so the buffer stays current.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg
            self._map_seq += 1

    def get_map(self):
        """Return ``(latest_map_or_None, sequence_number)`` atomically."""
        with self._lock:
            return self._map, self._map_seq

    def robot_xy(self):
        """Current robot (x, y) in the map frame, or None if TF isn't ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None


class FrontierExplorer(BasicNavigator):
    """A BasicNavigator that also detects frontiers and self-assigns goals."""

    def __init__(self, robot_id: int, sensor: _SensorNode):
        # BasicNavigator uses relative action/topic names, so passing the
        # namespace here yields /robot_N/navigate_to_pose etc.
        super().__init__(node_name='frontier_explorer',
                         namespace=f'robot_{robot_id}')
        # Sim time is mandatory (Gazebo clock); force it regardless of how
        # params were delivered so goal stamps line up with /clock.
        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.robot_id = robot_id
        self.sensor = sensor       # always-spinning source of map + pose
        ns = f'robot_{robot_id}'
        self.map_frame = f'{ns}/map'
        self.base_frame = f'{ns}/base_footprint'

        self._blacklist = []       # (x, y) world centroids that proved unproductive
        self._last_goal = None     # (x, y) of the last dispatched goal (re-send guard)
        self._hb_last = {}         # heartbeat key -> last wall-clock log time

    # ── logging / timing helpers ─────────────────────────────────────────────
    def _hb(self, key, msg, period=HEARTBEAT_PERIOD):
        """Throttled heartbeat: log `msg` at most once per `period` s per key.

        Every blocking wait routes its "still waiting" message through here so a
        stall is always visible, without flooding the log. `period<=0` forces it.
        """
        now = time.monotonic()
        if period <= 0 or now - self._hb_last.get(key, 0.0) >= period:
            self._hb_last[key] = now
            self.info(f'[robot_{self.robot_id}] {msg}')

    def _sleep(self, seconds):
        """Wall-clock nap in small slices, aborting promptly if rclpy goes down.

        Wall clock (not sim time): a stalled /clock can never wedge this.
        """
        end = time.monotonic() + seconds
        while rclpy.ok():
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.1, remaining))

    # ── frontier detection ───────────────────────────────────────────────────
    def _frontier_centroids(self):
        """Return [(world_x, world_y, cell_count), ...] for each frontier cluster.

        A frontier cell is a free cell with at least one 4-neighbour that is
        unknown (-1). Frontier cells are grouped into 8-connected clusters and
        each cluster's cell centroid is converted to a world coordinate.
        """
        g, _ = self.sensor.get_map()
        if g is None:
            return []
        w, h = g.info.width, g.info.height
        if w < 3 or h < 3:
            return []
        res = g.info.resolution
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        data = g.data

        # Mark frontier cells (skip the 1-cell border to keep neighbour math simple).
        frontier = bytearray(w * h)
        for y in range(1, h - 1):
            row = y * w
            for x in range(1, w - 1):
                i = row + x
                if not (0 <= data[i] < OCC_THRESH):
                    continue
                if (data[i - 1] == UNKNOWN or data[i + 1] == UNKNOWN or
                        data[i - w] == UNKNOWN or data[i + w] == UNKNOWN):
                    frontier[i] = 1

        # Cluster frontier cells with a BFS flood fill (8-connected).
        seen = bytearray(w * h)
        centroids = []
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                start = y * w + x
                if not frontier[start] or seen[start]:
                    continue
                q = deque([start])
                seen[start] = 1
                sx = sy = n = 0
                while q:
                    c = q.popleft()
                    cx, cy = c % w, c // w
                    sx += cx
                    sy += cy
                    n += 1
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                j = ny * w + nx
                                if frontier[j] and not seen[j]:
                                    seen[j] = 1
                                    q.append(j)
                if n >= MIN_CLUSTER:
                    wx = ox + (sx / n + 0.5) * res
                    wy = oy + (sy / n + 0.5) * res
                    centroids.append((wx, wy, n))
        return centroids

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < BLACKLIST_RADIUS
                   for bx, by in self._blacklist)

    def _select_goal(self, rx, ry):
        """Choose where to drive next given the robot at (rx, ry).

        Returns ``(goal_xy, frontier_xy, ncells)`` or ``None`` when no
        non-blacklisted frontier remains (exploration complete).

        Prefers the NEAREST non-blacklisted frontier that is already far enough
        (>= MIN_GOAL_DIST) to make the robot drive, and sends it as-is. If every
        remaining frontier is within goal tolerance (a goal there yields no
        motion), the nearest is kept but its goal is PROJECTED outward along
        robot->frontier to GOAL_PROJECT_DIST, so Nav2 has to move to reach it —
        this is what actually pushes the robot into unexplored space.
        """
        centroids = [c for c in self._frontier_centroids()
                     if not self._blacklisted(c[0], c[1])]
        if not centroids:
            return None
        centroids.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))

        # Nearest frontier already beyond tolerance -> drive straight to it.
        for cx, cy, n in centroids:
            if math.hypot(cx - rx, cy - ry) >= MIN_GOAL_DIST:
                return (cx, cy), (cx, cy), n

        # All remaining frontiers are too close: project the nearest outward so
        # the goal is far enough to induce motion toward the unknown boundary.
        cx, cy, n = centroids[0]
        d = math.hypot(cx - rx, cy - ry)
        if d < 1e-2:
            # Centroid sits on the robot — no usable heading; nothing to do here.
            return None
        ux, uy = (cx - rx) / d, (cy - ry) / d
        return (rx + ux * GOAL_PROJECT_DIST, ry + uy * GOAL_PROJECT_DIST), (cx, cy), n

    def _make_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        return goal

    # ── bounded waits (each is timeout-capped and emits heartbeats) ───────────
    def _wait_for_first_map(self):
        """Block until the first map arrives or FIRST_MAP_TIMEOUT elapses."""
        start = time.monotonic()
        while rclpy.ok():
            g, _ = self.sensor.get_map()
            if g is not None:
                return True
            elapsed = time.monotonic() - start
            if elapsed > FIRST_MAP_TIMEOUT:
                return False
            self._hb('first_map', f'waiting for first /robot_{self.robot_id}/map '
                                  f'({elapsed:.0f}s)')
            self._sleep(0.2)
        return False

    def _wait_for_fresh_map(self, seq_before):
        """After a goal, wait (bounded) for SLAM to publish a NEWER map.

        A fresh map is what makes the next frontier set reflect newly-driven
        area. Never blocks forever: on timeout it logs and proceeds with the map
        already held, so the loop keeps making progress even if SLAM stalls.
        """
        start = time.monotonic()
        while rclpy.ok():
            _, seq = self.sensor.get_map()
            if seq > seq_before:
                return True
            elapsed = time.monotonic() - start
            if elapsed > MAP_REFRESH_TIMEOUT:
                self._hb('fresh_map', f'no fresh map after {elapsed:.1f}s — '
                                     f'proceeding with the map in hand', period=0)
                return False
            self._hb('fresh_map', f'waiting for map update ({elapsed:.1f}s)')
            self._sleep(0.1)
        return False

    def _await_goal(self, goal_num):
        """Wait for the active Nav2 goal to finish, bounded by GOAL_TIMEOUT.

        isTaskComplete() spins the navigator node internally (0.1 s slices), so
        this doesn't busy-spin. Returns True if the task completed, False if it
        exceeded GOAL_TIMEOUT (caller cancels + blacklists) — Nav2 can never hang
        the explorer silently.
        """
        start = time.monotonic()
        while not self.isTaskComplete():
            elapsed = time.monotonic() - start
            if elapsed > GOAL_TIMEOUT:
                return False
            self._hb('nav', f'goal #{goal_num} still navigating ({elapsed:.0f}s)',
                     period=5.0)
            time.sleep(0.05)
        return True

    # ── main loop ────────────────────────────────────────────────────────────
    def explore(self):
        # Wait for Nav2 to come up. We use SLAM (no amcl), so wait on the
        # bt_navigator lifecycle node and skip the amcl initial-pose wait.
        self.info(f'[robot_{self.robot_id}] waiting for Nav2 to activate...')
        self.waitUntilNav2Active(localizer='bt_navigator')

        self.info(f'[robot_{self.robot_id}] waiting for first /robot_'
                  f'{self.robot_id}/map...')
        if not self._wait_for_first_map():
            self.error(f'[robot_{self.robot_id}] no map after '
                       f'{FIRST_MAP_TIMEOUT:.0f}s — is SLAM running? aborting.')
            return
        self.info(f'[robot_{self.robot_id}] map received — exploring.')

        goal_num = 0
        while rclpy.ok():
            pose = self.sensor.robot_xy()
            if pose is None:
                self._hb('tf', f'waiting for {self.map_frame}->{self.base_frame} TF...')
                self._sleep(TF_WAIT)
                continue
            rx, ry = pose

            sel = self._select_goal(rx, ry)
            if sel is None:
                self.info(f'[robot_{self.robot_id}] no reachable frontiers remain — '
                          f'exploration complete after {goal_num} goals. Done.')
                break
            (gx, gy), (fx, fy), ncells = sel

            # Fix: don't re-dispatch a goal essentially identical to the last one.
            # (Its frontier already proved unproductive, so blacklist it and move on
            # instead of re-sending at loop speed.)
            if (self._last_goal is not None and
                    math.hypot(gx - self._last_goal[0],
                               gy - self._last_goal[1]) < RESEND_RADIUS):
                self.warn(f'[robot_{self.robot_id}] frontier ({fx:.2f}, {fy:.2f}) '
                          f're-selected without progress — blacklisting.')
                self._blacklist.append((fx, fy))
                continue

            goal_num += 1
            dist = math.hypot(gx - rx, gy - ry)
            self.info(f'[robot_{self.robot_id}] goal #{goal_num} -> ({gx:.2f}, {gy:.2f}) '
                      f'[{ncells} cells, frontier ({fx:.2f}, {fy:.2f}), {dist:.2f} m away]')
            self._last_goal = (gx, gy)
            dispatch_t = time.monotonic()
            # Remember the map version now so we can wait for a NEWER one afterward.
            _, seq_before = self.sensor.get_map()

            self.goToPose(self._make_goal(gx, gy))
            if not self._await_goal(goal_num):
                # Nav2 never finished — cancel it and blacklist so we don't hang.
                self.warn(f'[robot_{self.robot_id}] goal #{goal_num} exceeded '
                          f'{GOAL_TIMEOUT:.0f}s — cancelling and blacklisting frontier '
                          f'({fx:.2f}, {fy:.2f}).')
                self.cancelTask()
                self._blacklist.append((fx, fy))
                continue

            # Did the robot actually move? This is the loop-breaker: a goal that
            # completes "successfully" but with no motion means the frontier is
            # inside tolerance, so blacklist it rather than re-pick it forever.
            end = self.sensor.robot_xy() or (rx, ry)
            moved = math.hypot(end[0] - rx, end[1] - ry)
            result = self.getResult()
            if result == TaskResult.SUCCEEDED and moved >= MIN_MOVE:
                self.info(f'[robot_{self.robot_id}] goal #{goal_num} reached '
                          f'(moved {moved:.2f} m).')
            elif result == TaskResult.SUCCEEDED:
                self.warn(f'[robot_{self.robot_id}] goal #{goal_num} "reached" but robot '
                          f'moved only {moved:.2f} m — blacklisting frontier '
                          f'({fx:.2f}, {fy:.2f}) to break the spin.')
                self._blacklist.append((fx, fy))
            else:
                self.warn(f'[robot_{self.robot_id}] goal #{goal_num} {result} — '
                          f'blacklisting frontier ({fx:.2f}, {fy:.2f}).')
                self._blacklist.append((fx, fy))

            # Let SLAM publish a FRESH map (new frontiers) before the next
            # selection — bounded, heartbeated, and never blocking forever.
            self._wait_for_fresh_map(seq_before)

            # Rate-limit: floor the dispatch interval (wall clock) so instant
            # completions can't drive the loop at tens of hertz.
            elapsed = time.monotonic() - dispatch_t
            if elapsed < MIN_GOAL_PERIOD:
                self._sleep(MIN_GOAL_PERIOD - elapsed)


def main(args=None):
    rclpy.init(args=args)
    # robot_id is read via a short-lived node whose name matches the launch node
    # so the launch-delivered parameter reaches it; BasicNavigator then needs the
    # id up front to build its namespace.
    boot = rclpy.create_node('frontier_explorer')
    boot.declare_parameter('robot_id', 0)
    robot_id = int(boot.get_parameter('robot_id').value)
    boot.destroy_node()

    # The sensor node (map + TF + clock) is spun continuously by its own
    # MultiThreadedExecutor in a daemon thread, so the explorer's callbacks are
    # never starved by BasicNavigator's global-executor spins on the main thread.
    sensor = _SensorNode(robot_id)
    sensor_exec = MultiThreadedExecutor()
    sensor_exec.add_node(sensor)
    sensor_thread = threading.Thread(target=sensor_exec.spin, daemon=True)
    sensor_thread.start()

    explorer = FrontierExplorer(robot_id, sensor)
    try:
        explorer.explore()
    except KeyboardInterrupt:
        pass
    finally:
        # Catch-don't-check teardown: under launch's SIGINT the context can go
        # down mid-cleanup, so a failed double-shutdown is a harmless no-op.
        try:
            sensor_exec.shutdown()
        except Exception:
            pass
        try:
            sensor.destroy_node()
        except Exception:
            pass
        try:
            explorer.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
