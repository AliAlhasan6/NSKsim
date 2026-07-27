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
always-fresh map/pose/clock from the sensor node and emits a heartbeat, so a
stall can never again be invisible.

Nav2 readiness (why waitUntilNav2Active is not enough)
------------------------------------------------------
BasicNavigator.waitUntilNav2Active() waits on bt_navigator (plus a localizer)
and nothing else. bt_navigator reaching 'active' says nothing about the servers
that actually EXECUTE a goal: controller_server and planner_server can still be
unconfigured when it returns. Goals dispatched in that window are rejected
outright — controller_server logs "Action server is inactive. Rejecting the
goal." and bt_navigator times out on compute_path_to_pose — which the explorer
reads as a Nav2 failure and (correctly, given what it can see) charges to the
frontier. Measured in explore_rung1f.log: all 13 goals failed this way, the
stack-health guard tripped after 5 distinct frontiers in 26 s, and
controller_server only reached 'active' ~33 s AFTER the explorer had exited.

So `_wait_for_nav2_servers_active` gates the goal loop on the real thing: each
server's own lifecycle `get_state` service must report 'active' before goal #1.
This is a state QUERY, not a fixed delay — a TimerAction in explore.launch.py
would be RTF-dependent and would silently regress the moment the machine, the
robot count or the param set changed.

Goal supervision (RTF-invariant)
--------------------------------
A goal is judged in SIMULATION time, not wall-clock, so the verdict is invariant
to the real-time factor: a slow machine burning real seconds never fails a goal
that is still making progress in the sim. `GoalSupervisor` (pure Python) decides
RUNNING vs FUTILE_NO_PROGRESS (no movement over a sim-time window, which doubles
as a post-dispatch planning grace period) vs FUTILE_SIM_TIMEOUT (per-goal sim
budget). Wall-clock survives ONLY as infrastructure-failure detection — a hard
guard for a wedged stack and a "clock stalled" warning — never as progress
supervision. Every such measurement stays on time.monotonic.
"""

import math
import sys
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
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator

from nsk_swarm.goal_supervisor import GoalSupervisor
from nsk_swarm.outcome_classifier import OutcomeClassifier

UNKNOWN = -1          # OccupancyGrid unknown cell
OCC_THRESH = 50       # cost >= this counts as an obstacle; below (and >=0) is free
MIN_CLUSTER = 4       # discard frontier blobs smaller than this many cells (noise)
BLACKLIST_RADIUS = 0.4  # m; a new centroid within this of a blacklisted goal is skipped

# ── Nav2 readiness gate ─────────────────────────────────────────────────────
# The lifecycle nodes that must be 'active' before the FIRST goal is dispatched.
# waitUntilNav2Active() covers bt_navigator only; these two are the ones that
# reject/abort a goal when they are not up yet (see the module docstring).
NAV2_REQUIRED_SERVERS = ('controller_server', 'planner_server')

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

# ── planner-abort classification ────────────────────────────────────────────
# Nav2 ComputePathToPose error codes that are WORLD evidence: the goal itself is
# geometrically unusable, so retrying learns nothing.
# 204 GOAL_OUTSIDE_MAP, 206 GOAL_OCCUPIED, 208 NO_VALID_PATH.
# Deliberately EXCLUDES 201 INVALID_PLANNER, 202 TF_ERROR, 207 TIMEOUT
# (infrastructure — the retry ladder is correct for those) and 203
# START_OUTSIDE_MAP / 205 START_OCCUPIED (wrong with the ROBOT's pose, not the
# frontier — blacklisting the frontier would be wrong).
UNREACHABLE_CODES = frozenset({204, 206, 208})

# ── sim-time wait bounds (RTF-invariant; measured against /clock) ────────────
# Goal supervision itself lives in GoalSupervisor (progress + sim-timeout, both
# in sim seconds). The remaining sim-time budget here bounds the map-refresh wait
# so a slow RTF doesn't prematurely proceed with a stale map.
MAP_REFRESH_SIM = 10.0     # sim s; primary budget to await a NEWER SLAM map after a goal

# ── wall-clock bounds (time.monotonic; INFRASTRUCTURE-failure detection only) ─
# Wall clock no longer supervises goal *progress* — it only catches a wedged
# stack (Nav2/SLAM/clock dead), where sim time can't advance to save us.
NAV2_READY_TIMEOUT = 120.0  # s wall; overall budget for the lifecycle readiness gate.
                            # WALL, not sim: the gate runs BEFORE exploration, and a
                            # Nav2 stack that never activates is exactly the failure
                            # that also leaves /clock unconsumed — sim time is not a
                            # safe measuring stick for the wait that precedes it.
NAV2_READY_POLL = 1.0       # s wall; interval between get_state polls
NAV2_STATE_CALL_WAIT = 1.0  # s wall; per-call budget for one get_state response
FIRST_MAP_TIMEOUT = 120.0  # s wall; give up waiting for the very first SLAM map
MAP_REFRESH_WALL = 90.0    # s wall; backstop for the map-refresh wait if /clock stalls
TF_WAIT = 0.5              # s wall; nap between retries while the map->base TF isn't ready yet
GOAL_WALL_GUARD = 900.0    # s wall; a goal running this long is an infra failure, not slow
                           # progress — log at ERROR and cancel (never hang on a wedged stack)
CLOCK_STALL_WARN = 30.0    # s wall with no sim-time advance -> warn once per stall episode
HEARTBEAT_PERIOD = 2.0     # s wall; min interval between "still waiting on X" heartbeat logs


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
            'frontier_explorer_sensors', namespace=f'robot_{robot_id}',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        ns = f'robot_{robot_id}'
        self.ns = ns
        self.map_frame = f'{ns}/map'
        self.base_frame = f'{ns}/base_footprint'
        cbg = ReentrantCallbackGroup()
        self._cbg = cbg
        self._state_clients = {}   # server name -> lifecycle get_state client

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

    def lifecycle_state(self, server, wait=NAV2_STATE_CALL_WAIT):
        """Return `server`'s lifecycle state label, or None if it can't be read.

        `server` is a plain node name under this robot's namespace (e.g.
        'controller_server'); its /robot_<id>/<server>/get_state service is
        queried and the returned ``current_state.label`` handed back.

        Asynchronous ON PURPOSE. The request rides this node's
        ReentrantCallbackGroup on the MultiThreadedExecutor spinning in the
        background thread, and the future is polled from the CALLING thread —
        so no executor thread is ever parked waiting on a response that the
        same executor has to deliver. A synchronous `client.call()` from
        inside a callback here is the project's starvation trap in service
        form: it deadlocks and hangs silently.

        None means "no answer" — the server hasn't advertised get_state yet, or
        it didn't reply within `wait` wall seconds. It is never a state, so
        callers can treat None and a non-'active' label the same way.
        """
        client = self._state_clients.get(server)
        if client is None:
            client = self.create_client(
                GetState, f'/{self.ns}/{server}/get_state',
                callback_group=self._cbg)
            self._state_clients[server] = client
        if not client.service_is_ready():
            return None

        future = client.call_async(GetState.Request())
        end = time.monotonic() + wait
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            # Drop the request rather than leaking it into the client's
            # pending map, where a late reply would sit forever.
            future.cancel()
            client.remove_pending_request(future)
            return None
        try:
            return future.result().current_state.label
        except Exception:
            return None

    def sim_time(self):
        """Current simulation time in seconds (from /clock via use_sim_time).

        This node's clock rides its continuously-spun executor, so the value is
        always fresh. Before /clock first publishes it reads 0.0.
        """
        return self.get_clock().now().nanoseconds * 1e-9


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

        # Blacklist / stack-health policy: only WORLD evidence poisons a frontier
        # immediately; STACK (Nav2 infra) failures are tolerated a few times and
        # a run-wide streak of them stops the run loudly instead of silently
        # blacklisting the whole map. Keyed on BLACKLIST_RADIUS (the same
        # quantum the blacklist geometry uses).
        self._classifier = OutcomeClassifier(BLACKLIST_RADIUS)

        # Clock-stall detector state (wall clock; infra-failure warning only).
        self._stall_sim_last = None    # last sim time we saw advance
        self._stall_since = None       # monotonic time sim last advanced
        self._stall_warned = False     # already warned about the current stall?

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
    def _frontier_clusters(self):
        """Return [(world_x, world_y, cell_count), ...] for EVERY frontier cluster.

        A frontier cell is a free cell with at least one 4-neighbour that is
        unknown (-1). Frontier cells are grouped into 8-connected clusters and
        each cluster's cell centroid is converted to a world coordinate. Every
        cluster is returned including sub-MIN_CLUSTER noise blobs — the size
        filter is applied by callers (selection) and reported by the termination
        audit. The detection geometry itself is unchanged.
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
        clusters = []
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
                wx = ox + (sx / n + 0.5) * res
                wy = oy + (sy / n + 0.5) * res
                clusters.append((wx, wy, n))
        return clusters

    def _frontier_centroids(self):
        """Frontier cluster centroids with >= MIN_CLUSTER cells (selection input).

        Thin size filter over ``_frontier_clusters`` — the noise gate that used
        to live inside the detector, kept here so selection behaviour is
        identical while the audit can still see the sub-threshold blobs.
        """
        return [(wx, wy, n) for wx, wy, n in self._frontier_clusters()
                if n >= MIN_CLUSTER]

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
    def _wait_for_nav2_servers_active(self, servers=NAV2_REQUIRED_SERVERS):
        """Block until every server in `servers` reports lifecycle state 'active'.

        The readiness gate waitUntilNav2Active() does not give us: it returns as
        soon as bt_navigator is up, while controller_server/planner_server may
        still be configuring, and every goal sent in that window is rejected
        (see the module docstring). Each server is polled through its own
        lifecycle get_state service — a real state query, so it is RTF-invariant
        by construction, unlike a fixed launch-time delay.

        Returns True when all are active, False on timeout (NAV2_READY_TIMEOUT
        wall seconds) — at which point the caller must NOT enter the goal loop.
        Wall clock throughout: this runs before exploration begins, so there is
        no goal progress to judge and a wedged stack must not be able to park
        the gate forever on a sim clock that isn't advancing.
        """
        start = time.monotonic()
        pending = list(servers)
        labels = {}      # last label seen per server (None = not answering)
        while rclpy.ok():
            for server in list(pending):
                labels[server] = self.sensor.lifecycle_state(server)
                if labels[server] == 'active':
                    pending.remove(server)
                    self.info(f'[robot_{self.robot_id}] {server} is active '
                              f'({time.monotonic() - start:.1f}s wall)')
            if not pending:
                self.info(f'[robot_{self.robot_id}] Nav2 readiness gate passed: '
                          f'{", ".join(servers)} all active — sending goals.')
                return True

            elapsed = time.monotonic() - start
            if elapsed >= NAV2_READY_TIMEOUT:
                detail = ', '.join(
                    f'{s} ({labels.get(s) or "no get_state response"})'
                    for s in pending)
                self.error(
                    f'[robot_{self.robot_id}] Nav2 readiness gate TIMED OUT after '
                    f'{elapsed:.0f}s wall — never reached "active": {detail}. '
                    f'Refusing to explore: every goal would be rejected with '
                    f'"Action server is inactive" and be misread as a dead '
                    f'frontier. Check /robot_{self.robot_id}/'
                    f'lifecycle_manager_navigation.')
                return False

            self._hb('nav2_ready',
                     'waiting for lifecycle state active: ' + ', '.join(
                         f'{s}={labels.get(s) or "?"}' for s in pending) +
                     f' ({elapsed:.0f}s wall)')
            self._sleep(NAV2_READY_POLL)
        return False

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
        area. Primary budget is MAP_REFRESH_SIM sim seconds (RTF-invariant);
        MAP_REFRESH_WALL is a wall-clock backstop so a stalled /clock can't wedge
        the wait forever. Never blocks indefinitely: on either budget it logs and
        proceeds with the map already held, so the loop keeps making progress.
        """
        sim_start = self.sensor.sim_time()
        wall_start = time.monotonic()
        while rclpy.ok():
            _, seq = self.sensor.get_map()
            if seq > seq_before:
                return True
            sim_elapsed = self.sensor.sim_time() - sim_start
            wall_elapsed = time.monotonic() - wall_start
            if sim_elapsed >= MAP_REFRESH_SIM:
                self._hb('fresh_map', f'no fresh map after {sim_elapsed:.1f} sim s — '
                                      f'proceeding with the map in hand', period=0)
                return False
            if wall_elapsed >= MAP_REFRESH_WALL:
                self._hb('fresh_map', f'no fresh map after {wall_elapsed:.0f}s wall '
                                      f'(sim clock stalled?) — proceeding', period=0)
                return False
            self._hb('fresh_map', f'waiting for map update ({sim_elapsed:.1f} sim s)')
            self._sleep(0.1)
        return False

    def _check_clock_stall(self, sim_now):
        """Warn once per episode if sim time hasn't advanced for CLOCK_STALL_WARN.

        Pure infrastructure watchdog on the WALL clock: a healthy sim advances
        /clock continuously, so a wall-clock gap with a frozen sim time means the
        sim/clock bridge is wedged. Resets (and re-arms the warning) the moment
        sim time moves again.
        """
        now = time.monotonic()
        if self._stall_sim_last is None or sim_now > self._stall_sim_last:
            self._stall_sim_last = sim_now
            self._stall_since = now
            self._stall_warned = False
            return
        if not self._stall_warned and now - self._stall_since >= CLOCK_STALL_WARN:
            self.warn(f'[robot_{self.robot_id}] sim clock has not advanced for '
                      f'{now - self._stall_since:.0f}s wall — is /clock publishing?')
            self._stall_warned = True

    def _terminal_outcome(self):
        """Outcome string for the goal Nav2 just finished, with its error fields.

        BasicNavigator.getResult() collapses every abort to TaskResult.FAILED,
        discarding the ``error_code`` Nav2 publishes on the NavigateToPose result
        — which is exactly the discriminator between "the planner says this
        frontier is unusable" (WORLD evidence, retire it now) and "the stack
        hiccuped" (retryable). isTaskComplete() keeps only ``.status``, but it
        leaves ``self.result_future`` live, so the result message is still
        reachable from here.

        Returns ``(outcome, error_code, error_msg)``; error_code is None when the
        result carried no readable code. Defensive by construction: ANY missing
        field or unexpected shape falls back to the plain TaskResult name, so a
        change in Nav2's result type can never break navigation.
        """
        name = self.getResult().name       # fallback, computed before anything can fail
        try:
            wrapper = self.result_future.result()
            res = None if wrapper is None else wrapper.result
            code = res.error_code
            msg = getattr(res, 'error_msg', '') or ''
        except (AttributeError, TypeError):
            return name, None, ''
        if code in UNREACHABLE_CODES:
            return OutcomeClassifier.UNREACHABLE_NO_PATH, code, msg
        return name, code, msg

    def _supervise_goal(self, goal_num):
        """Drive the active Nav2 goal to a terminal outcome under RTF-invariant
        progress supervision.

        Feeds a GoalSupervisor from the sensor node's sim clock + TF every slice
        (isTaskComplete() spins the navigator internally, so Nav2 keeps making
        progress and this doesn't busy-spin). Returns ``(outcome, end_xy,
        window_move, err)`` where ``window_move`` is the supervisor's
        trailing-window displacement at the moment of the verdict (meaningful for
        FUTILE_NO_PROGRESS, where it — not the whole-goal net — is what fired it),
        ``err`` is the ``(error_code, error_msg)`` pair off the Nav2 result (only
        Nav2's own terminations carry one; the rest report ``(None, '')``), and
        outcome is one of:
          * GoalSupervisor.FUTILE_NO_PROGRESS / FUTILE_SIM_TIMEOUT — the
            supervisor gave up (sim-time judgement); the goal is cancelled here.
          * 'WALL_GUARD' — GOAL_WALL_GUARD wall seconds elapsed: an
            infrastructure failure, not slow progress; the goal is cancelled here.
          * OutcomeClassifier.UNREACHABLE_NO_PATH — Nav2 aborted with a planner
            error_code that condemns the GOAL (see UNREACHABLE_CODES), not the
            stack; WORLD evidence, so the frontier is retired on this one hit.
          * a TaskResult name ('SUCCEEDED'/'FAILED'/'CANCELED'/'UNKNOWN') when
            Nav2 finishes the goal on its own.
        Wall clock is used ONLY for the guard + the clock-stall warning.
        """
        sup = GoalSupervisor()
        wall_start = time.monotonic()
        last_xy = self.sensor.robot_xy()

        while rclpy.ok():
            xy = self.sensor.robot_xy()
            if xy is not None:
                last_xy = xy
                sup.ingest(self.sensor.sim_time(), xy[0], xy[1])
                v = sup.verdict()
                if v != GoalSupervisor.RUNNING:
                    self.cancelTask()
                    return v, last_xy, sup.window_move(), (None, '')

            self._check_clock_stall(self.sensor.sim_time())

            wall_elapsed = time.monotonic() - wall_start
            if wall_elapsed >= GOAL_WALL_GUARD:
                self.error(f'[robot_{self.robot_id}] goal #{goal_num} ran '
                           f'{wall_elapsed:.0f}s WALL — infrastructure failure '
                           f'(Nav2/SLAM/clock wedged?), not slow progress; '
                           f'cancelling.')
                self.cancelTask()
                return 'WALL_GUARD', last_xy, sup.window_move(), (None, '')

            if self.isTaskComplete():
                outcome, code, msg = self._terminal_outcome()
                return (outcome, (last_xy or self.sensor.robot_xy()),
                        sup.window_move(), (code, msg))

            self._hb('nav', f'goal #{goal_num} navigating '
                            f'(sim {sup.elapsed():.0f}s, wall {wall_elapsed:.0f}s)',
                     period=5.0)
            time.sleep(0.05)
        return 'CANCELED', last_xy, sup.window_move(), (None, '')

    # ── auditable termination ────────────────────────────────────────────────
    def _log_termination_audit(self, goal_num):
        """Log a verifiable account of WHY selection returned no goal.

        Buckets the current scan's clusters so the "complete" claim can be
        checked from the run's own log — no post-hoc map/image analysis needed.
        If live (sized, non-blacklisted) frontier cells still remain, the run was
        HALTED with work outstanding, not completed: say so at WARN.
        """
        clusters = self._frontier_clusters()
        total = len(clusters)
        below_min = sum(1 for _, _, n in clusters if n < MIN_CLUSTER)
        sized = [(x, y, n) for x, y, n in clusters if n >= MIN_CLUSTER]
        blacklisted = sum(1 for x, y, _ in sized if self._blacklisted(x, y))
        retry_pending = sum(
            1 for x, y, _ in sized
            if not self._blacklisted(x, y) and self._classifier.failure_count((x, y)) > 0)
        remaining_cells = sum(n for x, y, n in sized if not self._blacklisted(x, y))

        self.info(
            f'[robot_{self.robot_id}] termination audit after {goal_num} goals: '
            f'{total} frontier clusters detected this scan '
            f'({blacklisted} blacklisted, {retry_pending} retry-pending, '
            f'{below_min} below MIN_CLUSTER={MIN_CLUSTER}).')
        if remaining_cells > 0:
            self.warn(
                f'[robot_{self.robot_id}] exploration HALTED with {remaining_cells} '
                f'frontier cells remaining (non-blacklisted, >= MIN_CLUSTER) — NOT '
                f'complete; selection could not turn them into a goal.')
        else:
            self.info(
                f'[robot_{self.robot_id}] no reachable frontiers remain — '
                f'exploration complete after {goal_num} goals. Done.')

    # ── main loop ────────────────────────────────────────────────────────────
    def explore(self):
        """Run the explore loop to completion.

        Returns True when exploration ran and terminated on its own terms
        (frontiers exhausted, or rclpy shut down under us), False when it could
        not run at all or was stopped by an infrastructure failure — Nav2 never
        activating, SLAM never publishing a map, or a run-wide streak of stack
        failures. main() turns a False into a non-zero exit status, so a boot
        that never explored can't be mistaken for a clean run.
        """
        # Wait for Nav2 to come up. We use SLAM (no amcl), so wait on the
        # bt_navigator lifecycle node and skip the amcl initial-pose wait.
        self.info(f'[robot_{self.robot_id}] waiting for Nav2 to activate...')
        self.waitUntilNav2Active(localizer='bt_navigator')

        # bt_navigator being active is NOT the whole stack being ready: the
        # servers that execute a goal are gated separately, right here, before
        # goal #1 can be thrown away on an inactive controller_server.
        if not self._wait_for_nav2_servers_active():
            return False

        self.info(f'[robot_{self.robot_id}] waiting for first /robot_'
                  f'{self.robot_id}/map...')
        if not self._wait_for_first_map():
            self.error(f'[robot_{self.robot_id}] no map after '
                       f'{FIRST_MAP_TIMEOUT:.0f}s — is SLAM running? aborting.')
            return False
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
                self._log_termination_audit(goal_num)
                break
            (gx, gy), (fx, fy), ncells = sel

            # Fix: don't re-dispatch a goal essentially identical to the last one.
            # (Its frontier already proved unproductive, so blacklist it and move on
            # instead of re-sending at loop speed.)
            #
            # BUT defer to the classifier first: if it is mid-retry on this
            # frontier (a stack failure it chose to tolerate), an immediate
            # re-selection of the SAME frontier is the INTENDED retry, not a
            # no-progress spin — MAX_FRONTIER_RETRIES already bounds that loop, so
            # let it ride. The guard keeps full authority over frontiers with no
            # pending retry.
            if (self._last_goal is not None and
                    math.hypot(gx - self._last_goal[0],
                               gy - self._last_goal[1]) < RESEND_RADIUS):
                if self._classifier.is_retry_pending((fx, fy)):
                    self.info(
                        f'[robot_{self.robot_id}] frontier ({fx:.2f}, {fy:.2f}) '
                        f're-selected immediately — classifier retry '
                        f'{self._classifier.failure_count((fx, fy))}/'
                        f'{self._classifier.MAX_FRONTIER_RETRIES} pending; deferring '
                        f'to the retry ladder, NOT blacklisting.')
                else:
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
            sim_start = self.sensor.sim_time()
            # Remember the map version now so we can wait for a NEWER one afterward.
            _, seq_before = self.sensor.get_map()

            self.goToPose(self._make_goal(gx, gy))
            outcome, end_xy, window_move, (err_code, err_msg) = \
                self._supervise_goal(goal_num)

            # Per-goal telemetry (RTF-invariant sim duration alongside the wall
            # duration, so a slow RTF is visible as sim<<wall, not as a failure).
            sim_dur = self.sensor.sim_time() - sim_start
            wall_dur = time.monotonic() - dispatch_t
            ex, ey = end_xy if end_xy is not None else (rx, ry)
            net = math.hypot(ex - rx, ey - ry)

            # Classify the outcome: WORLD evidence blacklists immediately, a
            # STACK (Nav2 infra) failure is tolerated up to MAX_FRONTIER_RETRIES
            # on the SAME frontier before blacklisting, and a run-wide streak of
            # stack failures across distinct frontiers stops the run loudly.
            decision = self._classifier.classify(
                outcome, (fx, fy), moved=(net >= MIN_MOVE))
            if decision.blacklist:
                self._blacklist.append((fx, fy))
            if decision.stop_unhealthy:
                self.error(f'[robot_{self.robot_id}] {decision.reason} — stopping; '
                           f'the frontiers are reachable, the STACK is not. Fix Nav2 '
                           f'rather than trusting a "complete" claim.')
            elif decision.category != 'SUCCESS':
                self.warn(f'[robot_{self.robot_id}] goal #{goal_num} {decision.reason}')

            # One INFO line per goal: id, frontier, outcome, sim + wall duration,
            # net displacement, and whether it poisoned the frontier. For
            # FUTILE_NO_PROGRESS also print window= — the trailing-PROGRESS_WINDOW
            # displacement that actually triggered the verdict — since the
            # whole-goal net= can be large (the robot drove, THEN stalled) and on
            # its own makes a correct FUTILE verdict look wrong.
            # A Nav2 abort also prints the result's error_code/error_msg. The
            # NUMBER matters as much as the text: bt_navigator publishes the
            # LOWEST non-zero code across compute_path_error_code and
            # follow_path_error_code, and FollowPath's codes are the 1xx series
            # against ComputePathToPose's 2xx — so a goal that failed BOTH ways
            # reports the 1xx and is (correctly, conservatively) left on the
            # retry ladder. Without the number that case is indistinguishable
            # from the classification simply not firing.
            window_note = ''
            if outcome == GoalSupervisor.FUTILE_NO_PROGRESS:
                window_note = (f' window={window_move:.2f}m/'
                               f'{GoalSupervisor.PROGRESS_WINDOW:.0f}s')
            err_note = f' error_code={err_code}' if err_code else ''
            if err_msg:
                err_note += f' error_msg="{err_msg}"'
            self.info(f'[robot_{self.robot_id}] goal #{goal_num} END '
                      f'frontier=({fx:.2f}, {fy:.2f}) outcome={outcome} '
                      f'sim={sim_dur:.1f}s wall={wall_dur:.1f}s net={net:.2f}m'
                      f'{window_note}{err_note} blacklisted={decision.blacklist}')

            if decision.stop_unhealthy:
                return False

            # Let SLAM publish a FRESH map (new frontiers) before the next
            # selection — bounded, heartbeated, and never blocking forever.
            self._wait_for_fresh_map(seq_before)

            # Rate-limit: floor the dispatch interval (wall clock) so instant
            # completions can't drive the loop at tens of hertz.
            elapsed = time.monotonic() - dispatch_t
            if elapsed < MIN_GOAL_PERIOD:
                self._sleep(MIN_GOAL_PERIOD - elapsed)

        return True


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
    # False -> exit non-zero, so a run that never got to explore (Nav2 or SLAM
    # never came up) is not reported by launch as "finished cleanly". A SIGINT
    # from launch's own shutdown is an orderly stop, not a failure.
    ok = False
    try:
        ok = explorer.explore()
    except KeyboardInterrupt:
        ok = True
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

    if not ok:
        sys.exit(1)


if __name__ == '__main__':
    main()
