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
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator

from nsk_swarm.goal_supervisor import GoalSupervisor
from nsk_swarm.outcome_classifier import OutcomeClassifier
from nsk_swarm.reachability import (UNREACHABLE_CODES, RetirementLedger,
                                    triage)
from nsk_swarm import reachability

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
# UNREACHABLE_CODES (204/206/208 — the codes that condemn the GOAL rather than
# the stack) now lives in nsk_swarm.reachability alongside the triage that reads
# it, so the whole decision is unit-testable without rclpy. Imported above and
# re-exported here, since this module's name for it is what the tests and the
# b26d146 commentary refer to.

# ── reachability pre-check (parameter-gated, default OFF) ───────────────────
# Selection asks the planner whether a candidate is plannable BEFORE dispatching
# it, instead of discovering it from a failed goal. See nsk_swarm.reachability.
#
# Probing is ESCALATING and single-cycle: selection walks down the nearest-first
# list until a candidate is reachable or a bound is hit, rather than examining a
# fixed window and deferring. It used to probe the top 3 only, and rung2c halted
# after 15 goals with 41 candidates it had never asked about — deferring to a
# fresher map cannot help, because the robot does not move during a DEFER, so
# the map barely changes and the next cycle re-derives the same verdict from the
# same three candidates. Worse, INCONCLUSIVE retires NOTHING (correctly — see
# nsk_swarm.reachability), so an unanswered candidate keeps its nearest-first
# position forever and a narrow window parked behind one never advances at all.
# Walking the list WITHIN the cycle steps past UNREACHABLE and INCONCLUSIVE
# alike, which is the only thing that actually makes progress here.
#
# TWO bounds, because one is the wrong shape. A probe is nearly free today
# (planner_server reports planning_time ~0.000s), so any probe COUNT is
# affordable — but NavFn's cost grows with the map, and a count tuned against a
# free probe silently becomes a multi-second stall once the map is large. The
# wall deadline bounds the quantity that actually hurts, which in turn is what
# makes the count safe to set generously:
#   * healthy probe, ~20 ms round trip -> the deadline admits ~500, so the COUNT
#     binds at 12 and a bad cycle costs ~0.2 s.
#   * degenerate probe, PLAN_CALL_WAIT=2.0 s with no answer -> the deadline
#     admits 5, binding long before 12 x 2 s = 24 s of stalled selection.
# The deadline is checked BEFORE each probe, so a cycle costs at most
# PRECHECK_CYCLE_BUDGET + PLAN_CALL_WAIT ~ 12 s — the same order as
# MAP_REFRESH_SIM, i.e. never more than the map wait this escalation replaces.
PRECHECK_MAX_PROBES = 12    # candidates probed in ONE selection cycle, nearest-first;
                            # stop at the first reachable one. 4x the old window, and
                            # with retirement draining up to 12/cycle a 44-cluster map
                            # is fully examined in ~4 cycles — inside MAX_CONSEC_DEFERS.
PRECHECK_CYCLE_BUDGET = 10.0  # s wall; deadline for the whole probe sequence of one
                            # cycle. WALL for the same reason PLAN_CALL_WAIT is: it
                            # bounds a stall, and a stalled planner is exactly where
                            # sim time stops being a safe measuring stick.
RETIRE_TTL_MAPS = 10      # SLAM map versions a pre-check retirement survives before
                          # the frontier is reconsidered (reachability grows with the map)
PLAN_CALL_WAIT = 2.0      # s wall; per-candidate budget for one ComputePathToPose
                          # round trip. WALL, not sim: it bounds a call that must never
                          # park the loop, and a planner that never answers is exactly
                          # the case where sim time is not a safe measuring stick.

# ── start-side probe (fires only on an AMBIGUOUS planner code) ──────────────
# A planner code condemns an ENDPOINT, and 208 NO_VALID_PATH does not say which
# one. plan_to sends `use_start: False`, so every query plans from the robot's
# live costmap pose: when THAT pose is unusable the planner returns 208 for
# every goal it is asked about, reachable ones included, and a 208 taken at face
# value retires frontiers that were never at fault. Measured in
# rung2f_on_explore.log: 42 consecutive attempts from one pose, all 208 with
# poses=0, two of them to goals 0.70 m away; 21 frontiers retired and the run
# stopped with 1036 frontier cells outstanding.
#
# The discriminator is one query from the current pose to a nearby free cell.
# If even that fails, the start is the fault and NOTHING may be retired.
START_PROBE_DIST = 0.5    # m; nominal range of a probe target — far enough to be a
                          # real query, near enough that failure implicates the start
START_PROBE_RING = 0.15   # m; a free cell counts as a target when its range is
                          # START_PROBE_DIST +/- this. Targets are chosen FROM THE MAP
                          # within that ring, never from a fixed offset table: a fixed
                          # offset lands in a wall often enough that every target
                          # failing would say more about the wall than the start.
START_PROBE_MAX = 3       # targets probed before the start is declared blocked. Spread
                          # around the robot (see _free_targets_near), because three
                          # targets sharing one goal-side reason to fail prove nothing.
START_PROBE_CLEAR = 0.25  # m; required free neighbourhood around a probe target.
                          # MUST match inflation_radius in EXP_NAV/nav2_robot<id>.yaml
                          # (EXP_NAV is defined in explore.launch.py:62; the value is
                          # currently 0.25 for both costmaps) — the explorer does not
                          # subscribe to any costmap, so this will drift silently if
                          # that YAML is edited.

# Consecutive START-BLOCKED cycles before the run stops. Its own counter and its
# own cap, deliberately NOT folded into MAX_CONSEC_DEFERS below: a defer streak
# is a claim about the frontiers ("the planner condemned every candidate"), and
# a stuck robot is a claim about the robot. Folding them would both corrupt that
# verdict and end the run naming the wrong fault. Larger than the defer cap
# because waiting can genuinely help here — SLAM refreshing the map can clear
# the cell the robot is standing in — but bounded all the same: recovery
# (back-up-and-clear) is not implemented, so past this point the loop is only
# waiting for something it cannot cause.
MAX_CONSEC_START_BLOCKED = 30

# Consecutive DEFER cycles (see _Defer) before the run STOPS instead of waiting
# again. Each cycle is individually bounded — _wait_for_fresh_map is timeout-
# capped and MIN_GOAL_PERIOD floors the interval — so a DEFER loop cannot spin;
# but "cannot spin" is not "must end", and an uncapped one is a silent hang,
# precisely the failure mode this module's structure exists to make impossible.
#
# Why waiting cannot rescue it, and hence why the cap is small: the robot does
# NOT move during a DEFER. The map therefore barely changes, so the input to the
# next selection is nearly the input to the last one and the verdict is nearly
# guaranteed to repeat. The only moving part is a retirement TTL expiring, which
# brings back a frontier the planner already condemned. A handful of cycles is
# enough to cover SLAM being mid-update; past that, more waiting buys nothing.
#
# Unreachable with the pre-check OFF: nothing retires, so _select_goal never
# returns DEFER and this cap is inert. The A/B baseline is untouched.
MAX_CONSEC_DEFERS = 10

# NavigateToPose collapses the planner and controller codes into one field and
# the controller's 1xx series wins by message-order priority (see the END-log
# comment in explore()). A goal that FAILED reporting a code in this range is a
# candidate for the post-hoc re-query: its planner verdict, if any, is masked.
FOLLOW_PATH_CODES = range(100, 200)


class _Defer:
    """Sentinel: no goal this cycle, but exploration is NOT finished.

    A distinct type rather than None because None already means "no frontiers
    remain" and ends the run. Conflating "the pre-check could not clear a
    candidate right now" with "the maze is mapped" would stop the run early and
    report a completion that never happened — the precise failure the
    termination audit exists to make impossible.
    """

    __slots__ = ()

    def __repr__(self):
        return 'DEFER'


DEFER = _Defer()


class _StartBlocked:
    """Sentinel: the ROBOT's own pose is unplannable, so this cycle proved nothing.

    Distinct from both of the others, because it is a different kind of claim.
    ``None`` says the frontiers are exhausted; ``DEFER`` says the planner
    condemned the candidates it was asked about. This one says the query never
    reached the question — every candidate fails identically while the start is
    bad, so no candidate was tested and nothing may be retired.

    Named ``START_BLOCKED_SEL`` rather than ``START_BLOCKED`` because
    ``reachability.START_BLOCKED`` already exists and is a different thing: that
    is a verdict STRING about one planner reply, this is a selection outcome.
    Colliding the names would make ``verdict == START_BLOCKED`` and
    ``sel is START_BLOCKED`` read alike while meaning different things.
    """

    __slots__ = ()

    def __repr__(self):
        return 'START_BLOCKED'


START_BLOCKED_SEL = _StartBlocked()

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


def _sector_gap(sector, taken, n_sectors=8):
    """Smallest circular distance from `sector` to any sector in `taken`.

    The spread metric behind _free_targets_near's farthest-first walk: sectors
    are a ring, so 0 and 7 are neighbours, not opposites.
    """
    if not taken:
        return n_sectors
    return min(min((sector - t) % n_sectors, (t - sector) % n_sectors)
               for t in taken)


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
        # ComputePathToPose client for the reachability pre-check, created
        # lazily on first use so a run with the pre-check OFF never advertises
        # it. NOT named `_services`: that attribute name is the Node's own
        # internal service registry and assigning to it breaks the node.
        self._plan_client = None

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

    @staticmethod
    def _await(future, deadline):
        """Poll `future` from the CALLING thread until done or `deadline` (wall).

        The counterpart to the async discipline `lifecycle_state` documents: the
        future is completed by the MultiThreadedExecutor spinning this node in
        the background thread, and nothing here ever parks an executor thread on
        a result that same executor has to deliver.
        """
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def plan_to(self, x, y, frame, timeout=PLAN_CALL_WAIT):
        """Ask planner_server for a path to (x, y): the reachability pre-check.

        Returns ``(error_code, path_xy, planning_time_s, error_msg)`` where
        ``path_xy`` is [(x, y), ...] of the planned poses, or ``None`` when no
        answer was obtained — server not up, goal rejected, or the round trip
        exceeded `timeout` wall seconds.

        ``None`` is INCONCLUSIVE, never "unreachable". A planner that did not
        answer has told us nothing about the world, and retiring a frontier on
        that would blacklist good goals whenever the stack hiccuped.

        Asynchronous throughout, for the same reason `lifecycle_state` is (see
        its docstring): a synchronous call from a callback on a single-threaded
        executor is this project's deadlock trap, and it hangs silently. Both
        the goal-acceptance and the result futures are polled from the calling
        thread against one shared wall deadline, so the whole call is bounded
        even if Nav2 accepts the goal and then never finishes it.

        `use_start` is False and `planner_id` empty: plan from the robot's
        CURRENT pose with the configured default planner, which is exactly the
        query bt_navigator would issue for this goal.
        """
        if self._plan_client is None:
            self._plan_client = ActionClient(
                self, ComputePathToPose, f'/{self.ns}/compute_path_to_pose',
                callback_group=self._cbg)
        client = self._plan_client
        if not client.server_is_ready():
            return None

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = ''
        goal.use_start = False

        deadline = time.monotonic() + timeout
        send_future = client.send_goal_async(goal)
        if not self._await(send_future, deadline):
            send_future.cancel()
            return None
        try:
            handle = send_future.result()
        except Exception:
            return None
        if handle is None or not handle.accepted:
            return None

        result_future = handle.get_result_async()
        if not self._await(result_future, deadline):
            # Abandon the query rather than leave it running: cancel is
            # fire-and-forget, since waiting on it would reintroduce exactly the
            # unbounded block this method exists to avoid.
            result_future.cancel()
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            return None
        try:
            res = result_future.result().result
            path_xy = [(p.pose.position.x, p.pose.position.y)
                       for p in res.path.poses]
            ptime = res.planning_time.sec + res.planning_time.nanosec * 1e-9
            return (int(res.error_code), path_xy, ptime,
                    getattr(res, 'error_msg', '') or '')
        except (AttributeError, TypeError):
            # Defensive, as _terminal_outcome is: an unexpected result shape
            # reads as "no answer", never as evidence against the frontier.
            return None

    def sim_time(self):
        """Current simulation time in seconds (from /clock via use_sim_time).

        This node's clock rides its continuously-spun executor, so the value is
        always fresh. Before /clock first publishes it reads 0.0.
        """
        return self.get_clock().now().nanoseconds * 1e-9


class FrontierExplorer(BasicNavigator):
    """A BasicNavigator that also detects frontiers and self-assigns goals."""

    def __init__(self, robot_id: int, sensor: _SensorNode,
                 reachability_precheck: bool = False):
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

        # Reachability pre-check (parameter-gated, default OFF so a run is a
        # clean A/B against the previous rung). When ON, selection probes the
        # planner before dispatching and the terminal outcome re-queries it when
        # the planner's verdict was masked by a controller code. When OFF, both
        # paths are inert and the loop behaves exactly as it did before.
        self._precheck = bool(reachability_precheck)
        # Retirements are TTL'd in SLAM map versions, NOT permanent: a frontier
        # walled off by unknown space now can become plannable once the map
        # fills in. Keyed on BLACKLIST_RADIUS, the same quantum the blacklist
        # geometry and the classifier use.
        self._retirements = RetirementLedger(BLACKLIST_RADIUS, RETIRE_TTL_MAPS)
        # Set by _select_goal on every DEFER: did that cycle rest on definitive
        # planner verdicts for EVERY candidate, or was the claim incomplete —
        # either because a probe went unanswered or because the probe budget ran
        # out before the candidate list did? explore() folds both across a defer
        # streak, the first to pick WORLD-vs-STACK and the second to say which
        # of the two stack faults to report.
        self._last_defer_definitive = True
        self._last_defer_truncated = False

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

    @staticmethod
    def _cell_has_clearance(data, w, cx, cy, pad):
        """True when every cell within `pad` cells of (cx, cy) is free.

        Callers must clamp (cx, cy) to [pad, dim-pad) so the box stays in range.
        """
        for ny in range(cy - pad, cy + pad + 1):
            base = ny * w
            for nx in range(cx - pad, cx + pad + 1):
                if not (0 <= data[base + nx] < OCC_THRESH):
                    return False
        return True

    def _free_targets_near(self, rx, ry):
        """Free cells ~START_PROBE_DIST from (rx, ry), spread around the robot.

        Returns up to START_PROBE_MAX world points ``[(x, y), ...]``: free cells
        whose range from the robot lies within START_PROBE_RING of
        START_PROBE_DIST and which carry START_PROBE_CLEAR of free neighbourhood.
        ``[]`` when no map has arrived or nothing qualifies — which is not
        evidence about anything, and the caller reads it as INCONCLUSIVE.

        Why spread rather than simply the nearest N: these targets exist to test
        the START, so they must not all fail for one shared goal-side reason.
        Three cells clustered against the same wall would do exactly that and
        report a wall as a stuck robot. One candidate per 45-degree sector, then
        a farthest-first walk over the sector ring, keeps them pointing at
        genuinely different places.

        Why the SLAM map rather than a costmap: it is the only grid this node
        subscribes to (see START_PROBE_CLEAR), and its lack of an inflation layer
        is precisely what makes a failed probe informative. plan_to plans from
        the robot's live COSTMAP pose either way, so if the start is unusable
        every target fails regardless of how benign it looks in this grid.

        Grid arithmetic is _frontier_clusters': row-major ``y * w + x``, free is
        ``0 <= v < OCC_THRESH``, cell centre is ``origin + (i + 0.5) * res``.
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

        lo = START_PROBE_DIST - START_PROBE_RING
        hi = START_PROBE_DIST + START_PROBE_RING
        pad = max(1, int(round(START_PROBE_CLEAR / res)))   # clearance, in cells
        reach = int(hi / res) + 1                           # window half-width, cells
        rcx = int((rx - ox) / res)
        rcy = int((ry - oy) / res)

        best = {}      # sector -> (|range - nominal|, wx, wy)
        for cy in range(max(pad, rcy - reach), min(h - pad, rcy + reach + 1)):
            row = cy * w
            wy = oy + (cy + 0.5) * res
            for cx in range(max(pad, rcx - reach), min(w - pad, rcx + reach + 1)):
                if not (0 <= data[row + cx] < OCC_THRESH):
                    continue
                wx = ox + (cx + 0.5) * res
                d = math.hypot(wx - rx, wy - ry)
                if not lo <= d <= hi:
                    continue
                sector = int((math.atan2(wy - ry, wx - rx) + math.pi) /
                             (math.pi / 4)) % 8
                score = abs(d - START_PROBE_DIST)
                # Cheapest tests first: the clearance box is only paid for a cell
                # that would actually win its sector.
                if sector in best and best[sector][0] <= score:
                    continue
                if not self._cell_has_clearance(data, w, cx, cy, pad):
                    continue
                best[sector] = (score, wx, wy)

        # Nearest-to-nominal first, then each next target as far around the ring
        # from the ones already taken as the surviving sectors allow.
        pool = sorted(best.items(), key=lambda kv: kv[1][0])
        chosen = [pool.pop(0)] if pool else []
        while pool and len(chosen) < START_PROBE_MAX:
            taken = [s for s, _v in chosen]
            pool.sort(key=lambda kv: (-_sector_gap(kv[0], taken), kv[1][0]))
            chosen.append(pool.pop(0))
        return [(v[1], v[2]) for _s, v in chosen]

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < BLACKLIST_RADIUS
                   for bx, by in self._blacklist)

    def _ordered_candidates(self, rx, ry, map_seq=0):
        """Dispatchable candidates given the robot at (rx, ry), best first.

        Returns ``[(goal_xy, frontier_xy, ncells), ...]`` in the explorer's
        existing utility order — nearest frontier first. Element 0 is exactly
        what selection has always chosen; the rest exist so the pre-check has
        alternatives to fall back on.

        Prefers NEAREST non-blacklisted frontiers already far enough
        (>= MIN_GOAL_DIST) to make the robot drive, sent as-is. If every
        remaining frontier is within goal tolerance (a goal there yields no
        motion), the list degenerates to the single nearest one PROJECTED
        outward along robot->frontier to GOAL_PROJECT_DIST, so Nav2 has to move
        to reach it — this is what actually pushes the robot into unexplored
        space. Note the goal and the frontier differ in that case: the pre-check
        must probe the GOAL (that is what gets dispatched) while retirement
        keys on the FRONTIER.

        `map_seq` is the current SLAM map version, used to expire pre-check
        retirements; the default keeps callers that don't pre-check unaffected.
        """
        centroids = [c for c in self._frontier_centroids()
                     if not self._blacklisted(c[0], c[1])
                     and not self._retirements.is_retired((c[0], c[1]), map_seq)]
        if not centroids:
            return []
        centroids.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))

        far = [((cx, cy), (cx, cy), n) for cx, cy, n in centroids
               if math.hypot(cx - rx, cy - ry) >= MIN_GOAL_DIST]
        if far:
            return far

        # All remaining frontiers are too close: project the nearest outward so
        # the goal is far enough to induce motion toward the unknown boundary.
        cx, cy, n = centroids[0]
        d = math.hypot(cx - rx, cy - ry)
        if d < 1e-2:
            # Centroid sits on the robot — no usable heading; nothing to do here.
            return []
        ux, uy = (cx - rx) / d, (cy - ry) / d
        return [((rx + ux * GOAL_PROJECT_DIST, ry + uy * GOAL_PROJECT_DIST),
                 (cx, cy), n)]

    def _probe(self, goal_xy, idx):
        """Pre-check one candidate goal against the planner.

        Returns ``(verdict, error_code)``: the triage of one ComputePathToPose
        round trip into REACHABLE / UNREACHABLE / INCONCLUSIVE (see
        nsk_swarm.reachability), plus the planner's raw code — ``None`` when
        there was no answer. The CODE is returned alongside the verdict because
        the verdict alone cannot say which endpoint the planner condemned: 208
        NO_VALID_PATH triages as UNREACHABLE but names neither end, so selection
        has to see the number to know whether a start probe is warranted.

        Logs the planner's own planning_time, error_code and error_msg for every
        call — that is the run data the probe budget gets tuned from
        (PRECHECK_MAX_PROBES and PRECHECK_CYCLE_BUDGET), and without it a
        pre-check that quietly times out every cycle is indistinguishable from
        one that is working.
        """
        gx, gy = goal_xy
        reply = self.sensor.plan_to(gx, gy, self.map_frame)
        if reply is None:
            self.info(f'[robot_{self.robot_id}] precheck cand#{idx} '
                      f'({gx:.2f}, {gy:.2f}) -> no answer within '
                      f'{PLAN_CALL_WAIT:.1f}s wall — INCONCLUSIVE')
            return reachability.INCONCLUSIVE, None

        code, path_xy, ptime, msg = reply
        verdict = triage(code, path_xy, goal_xy)
        msg_note = f' error_msg="{msg}"' if msg else ''
        self.info(f'[robot_{self.robot_id}] precheck cand#{idx} '
                  f'({gx:.2f}, {gy:.2f}) -> {verdict} error_code={code} '
                  f'planning_time={ptime:.3f}s poses={len(path_xy)}{msg_note}')
        return verdict, code

    def _start_probe(self, rx, ry, deadline=None):
        """Ask whether the planner can plan OUT of the robot's current pose.

        The discriminator for an ambiguous planner code (AMBIGUOUS_CODES — 208
        NO_VALID_PATH, which reports that the search failed without naming the
        end that failed it). Targets come from _free_targets_near, so they are
        real free cells about START_PROBE_DIST away rather than a fixed offset.

        Returns one of:
          * ``START_OK`` — the first target that answers with a path. Short
            circuits: the remaining targets are not probed. No endpoint check is
            applied, deliberately — any path proves the pose can be planned out
            of, which is the entire question (reachability.start_verdict).
          * ``START_BLOCKED`` — every target came back condemned. The only
            verdict that may stop a cycle from retiring anything.
          * ``INCONCLUSIVE`` — no target could be chosen, a target went
            unanswered, one answered with an infrastructure code, or the cycle
            deadline arrived. Nothing was established, so nothing may be retired
            on it either way.

        Bounded by construction: at most START_PROBE_MAX round trips of
        PLAN_CALL_WAIT each, and `deadline` (the caller's PRECHECK_CYCLE_BUDGET)
        is checked BEFORE each one, the same discipline the candidate loop uses.
        """
        targets = self._free_targets_near(rx, ry)
        if not targets:
            self.info(f'[robot_{self.robot_id}] start probe: no free cell '
                      f'{START_PROBE_DIST - START_PROBE_RING:.2f}-'
                      f'{START_PROBE_DIST + START_PROBE_RING:.2f} m from '
                      f'({rx:.2f}, {ry:.2f}) with {START_PROBE_CLEAR:.2f} m '
                      f'clearance — INCONCLUSIVE')
            return reachability.INCONCLUSIVE

        total = len(targets)
        for n, (tx, ty) in enumerate(targets, start=1):
            if deadline is not None and time.monotonic() >= deadline:
                self.info(f'[robot_{self.robot_id}] start probe {n}/{total} '
                          f'skipped: cycle budget spent — INCONCLUSIVE')
                return reachability.INCONCLUSIVE

            reply = self.sensor.plan_to(tx, ty, self.map_frame)
            if reply is None:
                self.info(f'[robot_{self.robot_id}] start probe {n}/{total} '
                          f'({tx:.2f}, {ty:.2f}) -> no answer within '
                          f'{PLAN_CALL_WAIT:.1f}s wall — INCONCLUSIVE')
                return reachability.INCONCLUSIVE

            code, path_xy, ptime, msg = reply
            verdict = reachability.start_verdict(code, path_xy)
            msg_note = f' error_msg="{msg}"' if msg else ''
            self.info(f'[robot_{self.robot_id}] start probe {n}/{total} '
                      f'({tx:.2f}, {ty:.2f}) -> {verdict} error_code={code} '
                      f'planning_time={ptime:.3f}s poses={len(path_xy)}'
                      f'{msg_note}')
            if verdict == reachability.START_OK:
                return reachability.START_OK
            if verdict != reachability.START_BLOCKED:
                # An infrastructure answer about one target leaves the start
                # unestablished; claiming BLOCKED on it would be the same
                # over-reading this whole path exists to prevent.
                return reachability.INCONCLUSIVE

        # The loop can only end here with every target condemned — the other two
        # outcomes return early.
        return reachability.START_BLOCKED

    def _select_goal(self, rx, ry, map_seq=0):
        """Choose where to drive next given the robot at (rx, ry).

        Returns one of:
          * ``(goal_xy, frontier_xy, ncells)`` — dispatch it.
          * ``None`` — nothing left to try and nothing pending: exploration is
            complete, and the caller ends the run.
          * ``DEFER`` — frontiers remain but none is dispatchable THIS cycle
            (the pre-check cleared none, or the only survivors are retired and
            not yet expired). The caller waits for a fresher map and retries,
            up to MAX_CONSEC_DEFERS times. Distinct from ``None`` on purpose: a
            cycle the pre-check could not resolve is not evidence the maze is
            mapped, and collapsing the two would end the run early with work
            outstanding.
          * ``START_BLOCKED_SEL`` — the ROBOT's pose is unplannable, so no
            candidate was really judged and NOTHING is retired. Returned the
            moment that is established, abandoning the rest of the cycle: while
            the start is bad every candidate fails identically, so continuing to
            probe would only manufacture more false evidence. Distinct from
            ``DEFER`` because it is a claim about the robot rather than about the
            frontiers, and it must not feed the defer streak's verdict.

        Every DEFER also sets ``_last_defer_definitive``: True only when the
        cycle asked about EVERY candidate and got a definitive planner verdict
        for each. Any INCONCLUSIVE probe clears it, and so does running out of
        probe budget — a cycle that stopped early cannot claim "nothing is
        reachable" while candidates it never queried sit in the list.
        ``_last_defer_truncated`` records which of those two it was, so the stop
        message names the real fault. explore() accumulates both over a defer
        streak to decide whether hitting the cap is an ANSWER ("nothing
        reachable remains") or a MALFUNCTION ("the planner never told us" /
        "the budget was too small"), which is the same WORLD-vs-STACK split
        OutcomeClassifier draws for goal outcomes.

        With the pre-check OFF this is the historical behaviour exactly: the
        best candidate, dispatched unexamined. Nothing retires, so DEFER is
        unreachable on that path.
        """
        candidates = self._ordered_candidates(rx, ry, map_seq)
        # The two labels describe THIS cycle, so they start from the identity
        # values of the folds explore() runs them through (`and` / `or`). Setting
        # them here rather than on each return path is what makes that true
        # structurally: both folds are monotone, so a label inherited from the
        # previous cycle can never recover, and the streak it lands in reports
        # that cycle's fault as its own. The parked path below used to write only
        # one of them and leak the other; any return added later is now correct
        # without having to remember either.
        self._last_defer_definitive = True
        self._last_defer_truncated = False
        if not candidates:
            # Retirements are the only thing that can hide a live frontier here;
            # while any are outstanding the map is not finished, it is pending.
            # Definitive (per the reset above): every survivor is parked by a
            # verdict already in hand, and no probe loop ran to be truncated.
            if self._retirements.active(map_seq):
                return DEFER
            return None
        if not self._precheck:
            return candidates[0]

        # Probe in utility order and take the first candidate the planner can
        # actually reach. UNREACHABLE retires the frontier (TTL'd, so a growing
        # map reconsiders it); INCONCLUSIVE retires NOTHING — a timeout or a TF
        # error is evidence about the stack, not about the world.
        #
        # Keep going down the WHOLE list, not a fixed window: an all-unreachable
        # head says nothing about position 12, and deferring to re-ask a
        # stationary robot's barely-changed map is how rung2c halted with 41
        # candidates unexamined. The two bounds below are what keep this from
        # spending the run's wall clock on one selection (see PRECHECK_MAX_PROBES).
        inconclusive = 0
        probed = 0
        deadline = time.monotonic() + PRECHECK_CYCLE_BUDGET
        truncated = None            # None => the list itself ran out
        start_state = None          # this cycle's start verdict; probed at most ONCE,
                                    # however many candidates report an ambiguous code
        for idx, (goal_xy, frontier_xy, ncells) in enumerate(candidates):
            if probed >= PRECHECK_MAX_PROBES:
                truncated = f'probe cap {PRECHECK_MAX_PROBES}'
                break
            # Checked BEFORE the probe, so the worst case is this budget plus
            # one PLAN_CALL_WAIT rather than an unbounded overshoot.
            if time.monotonic() >= deadline:
                truncated = f'{PRECHECK_CYCLE_BUDGET:.0f}s wall budget'
                break
            probed += 1
            verdict, code = self._probe(goal_xy, idx)
            if verdict == reachability.REACHABLE:
                return goal_xy, frontier_xy, ncells

            # The planner named the START itself (203 START_OUTSIDE_MAP, 205
            # START_OCCUPIED). No probe can add anything — the code IS the
            # diagnosis — and the goal was never judged, so nothing retires.
            if code in reachability.START_SIDE_CODES:
                self.warn(f'[robot_{self.robot_id}] start pose '
                          f'({rx:.2f}, {ry:.2f}) unplannable — planner returned '
                          f'error_code={code} for cand#{idx}, which condemns the '
                          f'START, not the goal. The START is at fault: retiring '
                          f'NOTHING and leaving all {len(candidates)} candidates '
                          f'eligible.')
                return START_BLOCKED_SEL

            if verdict == reachability.UNREACHABLE:
                # 208 NO_VALID_PATH triages as UNREACHABLE but names no endpoint,
                # so before condemning the frontier, ask whether the robot can
                # plan out of where it is standing at all. Once per cycle: the
                # answer is a property of the pose, not of the candidate, and
                # every candidate would otherwise re-ask it identically.
                if code in reachability.AMBIGUOUS_CODES:
                    if start_state is None:
                        start_state = self._start_probe(rx, ry, deadline)
                    if start_state == reachability.START_BLOCKED:
                        self.warn(
                            f'[robot_{self.robot_id}] start pose '
                            f'({rx:.2f}, {ry:.2f}) unplannable — every probe to a '
                            f'free cell ~{START_PROBE_DIST:.1f} m out failed too, '
                            f'so error_code={code} on cand#{idx} condemns the '
                            f'START, not the goal. The START is at fault: '
                            f'retiring NOTHING and leaving all {len(candidates)} '
                            f'candidates eligible.')
                        return START_BLOCKED_SEL
                    if start_state != reachability.START_OK:
                        # Start unestablished, so the 208 is unattributed. It is
                        # not evidence against this frontier and must not retire
                        # it — the same doctrine INCONCLUSIVE follows everywhere.
                        inconclusive += 1
                        continue

                until = self._retirements.retire(frontier_xy, map_seq)
                if start_state == reachability.START_OK:
                    why = ('start probe cleared the robot pose, so the GOAL is '
                           'at fault')
                else:
                    why = f'error_code={code} names the GOAL'
                self.warn(f'[robot_{self.robot_id}] frontier '
                          f'({frontier_xy[0]:.2f}, {frontier_xy[1]:.2f}) '
                          f'unplannable — retiring until map #{until} '
                          f'(now #{map_seq}, TTL {RETIRE_TTL_MAPS} maps); {why}')
            else:
                inconclusive += 1

        # A cycle may claim "nothing reachable remains" only if it asked about
        # EVERY candidate and got a definitive answer for each. One unanswered
        # probe breaks that, and so does stopping on a bound with candidates
        # still unexamined. Conservative on purpose — the cost of being wrong
        # here is a run that reports a completion it never established.
        self._last_defer_truncated = truncated is not None
        self._last_defer_definitive = (inconclusive == 0 and truncated is None)
        stopped = (f'stopped by {truncated}, {len(candidates) - probed} unexamined'
                   if truncated else 'whole list examined')
        self._hb('precheck', f'no reachable frontier: probed {probed} of '
                             f'{len(candidates)} candidates '
                             f'({inconclusive} inconclusive, {stopped}) — '
                             f'deferring to a fresher map', period=5.0)
        return DEFER

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

    def _terminal_outcome(self, goal_xy=None):
        """Outcome string for the goal Nav2 just finished, with its error fields.

        BasicNavigator.getResult() collapses every abort to TaskResult.FAILED,
        discarding the ``error_code`` Nav2 publishes on the NavigateToPose result
        — which is exactly the discriminator between "the planner says this
        frontier is unusable" (WORLD evidence, retire it now) and "the stack
        hiccuped" (retryable). isTaskComplete() keeps only ``.status``, but it
        leaves ``self.result_future`` live, so the result message is still
        reachable from here.

        Unmasking the planner's verdict
        -------------------------------
        That single ``error_code`` is an AGGREGATE, and it loses the information
        this classification needs. bt_navigator keeps the planner's and the
        controller's codes as separate BLACKBOARD entries (the BT's
        ``compute_path_error_code`` / ``follow_path_error_code`` ports) and
        ``BtActionServer::populateErrorCode`` writes only the LOWEST non-zero one
        into the result. FollowPath's codes are the 1xx series against
        ComputePathToPose's 2xx, so whenever a goal failed both ways the
        controller code wins and the planner abort is invisible. The blackboard
        is process-local to bt_navigator; the components are never published, so
        there is no field to read them from — measured in rung2b, 22 goals
        reported 1xx and exactly ONE was classified UNREACHABLE_NO_PATH.

        Worse, ``cleanErrorCodes()`` runs only at goal COMPLETION, so a code set
        once persists across BT ticks for the rest of that goal: a transient
        early FollowPath failure masks a genuine planner abort seconds later.

        So when the aggregate reports a FollowPath code, ask the planner
        directly — one ComputePathToPose at the same goal point, whose
        ``error_code`` is the planner's alone and cannot be masked by
        construction. This is the backstop for the 208s the selection pre-check
        cannot catch (``allow_unknown: true`` means the planner plans through
        unmapped space and rarely returns 208 until it has actually tried).

        It is a FRESH query: the robot and the map have both moved on, so it
        answers "is this goal plannable now", not "what did the planner say
        then". For deciding whether to retire the frontier that is the operative
        question anyway. Gated on the pre-check parameter and only consulted for
        a FAILED goal carrying a 1xx, so it costs nothing on the OFF path and
        nothing on goals whose code was already legible.

        Returns ``(outcome, error_code, error_msg, recheck_code)``; error_code is
        None when the result carried no readable code, and recheck_code is None
        unless the re-query actually ran. Defensive by construction: ANY missing
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
            return name, None, '', None
        if code in UNREACHABLE_CODES:
            return OutcomeClassifier.UNREACHABLE_NO_PATH, code, msg, None

        if (self._precheck and goal_xy is not None and
                name == 'FAILED' and code in FOLLOW_PATH_CODES):
            reply = self.sensor.plan_to(goal_xy[0], goal_xy[1], self.map_frame)
            if reply is None:
                self.info(f'[robot_{self.robot_id}] planner re-query after '
                          f'error_code={code}: no answer — leaving on the retry '
                          f'ladder')
                return name, code, msg, None
            rcode, rpath, rtime, rmsg = reply
            verdict = triage(rcode, rpath, goal_xy)
            rmsg_note = f' error_msg="{rmsg}"' if rmsg else ''
            self.info(f'[robot_{self.robot_id}] planner re-query after '
                      f'error_code={code} -> {verdict} error_code={rcode} '
                      f'planning_time={rtime:.3f}s{rmsg_note}')
            if verdict == reachability.UNREACHABLE:
                # The controller code was masking a real planner abort.
                return OutcomeClassifier.UNREACHABLE_NO_PATH, code, msg, rcode
            return name, code, msg, rcode

        return name, code, msg, None

    def _supervise_goal(self, goal_num, goal_xy=None):
        """Drive the active Nav2 goal to a terminal outcome under RTF-invariant
        progress supervision.

        Feeds a GoalSupervisor from the sensor node's sim clock + TF every slice
        (isTaskComplete() spins the navigator internally, so Nav2 keeps making
        progress and this doesn't busy-spin). Returns ``(outcome, end_xy,
        window_move, err)`` where ``window_move`` is the supervisor's
        trailing-window displacement at the moment of the verdict (meaningful for
        FUTILE_NO_PROGRESS, where it — not the whole-goal net — is what fired it),
        ``err`` is the ``(error_code, error_msg, recheck_code)`` triple off the
        Nav2 result (only Nav2's own terminations carry one; the rest report
        ``(None, '', None)``), and outcome is one of:
          * GoalSupervisor.FUTILE_NO_PROGRESS / FUTILE_SIM_TIMEOUT — the
            supervisor gave up (sim-time judgement); the goal is cancelled here.
          * 'WALL_GUARD' — GOAL_WALL_GUARD wall seconds elapsed: an
            infrastructure failure, not slow progress; the goal is cancelled here.
          * OutcomeClassifier.UNREACHABLE_NO_PATH — Nav2 aborted with a planner
            error_code that condemns the GOAL (see UNREACHABLE_CODES), not the
            stack; WORLD evidence, so the frontier is retired on this one hit.
            Also reached via the masked-code re-query in `_terminal_outcome`,
            which `goal_xy` is threaded through for.
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
                    return v, last_xy, sup.window_move(), (None, '', None)

            self._check_clock_stall(self.sensor.sim_time())

            wall_elapsed = time.monotonic() - wall_start
            if wall_elapsed >= GOAL_WALL_GUARD:
                self.error(f'[robot_{self.robot_id}] goal #{goal_num} ran '
                           f'{wall_elapsed:.0f}s WALL — infrastructure failure '
                           f'(Nav2/SLAM/clock wedged?), not slow progress; '
                           f'cancelling.')
                self.cancelTask()
                return 'WALL_GUARD', last_xy, sup.window_move(), (None, '', None)

            if self.isTaskComplete():
                outcome, code, msg, recheck = self._terminal_outcome(goal_xy)
                return (outcome, (last_xy or self.sensor.robot_xy()),
                        sup.window_move(), (code, msg, recheck))

            self._hb('nav', f'goal #{goal_num} navigating '
                            f'(sim {sup.elapsed():.0f}s, wall {wall_elapsed:.0f}s)',
                     period=5.0)
            time.sleep(0.05)
        return 'CANCELED', last_xy, sup.window_move(), (None, '', None)

    # ── auditable termination ────────────────────────────────────────────────
    def _log_termination_audit(self, goal_num):
        """Log a verifiable account of WHY selection returned no goal.

        Buckets the current scan's clusters so the "complete" claim can be
        checked from the run's own log — no post-hoc map/image analysis needed.
        If live (sized, non-blacklisted) frontier cells still remain, the run was
        HALTED with work outstanding, not completed: say so at WARN.
        """
        _, map_seq = self.sensor.get_map()
        clusters = self._frontier_clusters()
        total = len(clusters)
        below_min = sum(1 for _, _, n in clusters if n < MIN_CLUSTER)
        sized = [(x, y, n) for x, y, n in clusters if n >= MIN_CLUSTER]
        blacklisted = sum(1 for x, y, _ in sized if self._blacklisted(x, y))
        retry_pending = sum(
            1 for x, y, _ in sized
            if not self._blacklisted(x, y) and self._classifier.failure_count((x, y)) > 0)
        remaining_cells = sum(n for x, y, n in sized if not self._blacklisted(x, y))
        # Pre-check retirements still live at this map version. Reaching the
        # audit with any outstanding would mean the run ended while frontiers
        # were merely PARKED, not exhausted — _select_goal returns DEFER rather
        # than None in that case, so this should read 0 here; it is printed so
        # the claim is checkable from the log rather than assumed.
        retired = self._retirements.active(map_seq)

        self.info(
            f'[robot_{self.robot_id}] termination audit after {goal_num} goals: '
            f'{total} frontier clusters detected this scan '
            f'({blacklisted} blacklisted, {retry_pending} retry-pending, '
            f'{retired} retired-unexpired, '
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

        Returns True when exploration ran and terminated on its own terms —
        frontiers exhausted, rclpy shut down under us, or (pre-check only)
        MAX_CONSEC_DEFERS cycles in which the planner definitively condemned
        every candidate it was asked about. False when it could not run at all
        or was stopped by an infrastructure failure: Nav2 never activating,
        SLAM never publishing a map, a run-wide streak of stack failures, or
        MAX_CONSEC_DEFERS cycles in which the pre-check never got an answer.
        main() turns a False into a non-zero exit status, so neither a boot that
        never explored nor a run wedged behind an unresponsive planner can be
        mistaken for a clean one.
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
        # Defer-streak state. `defer_definitive` stays True only while every
        # cycle in the CURRENT streak got definitive planner verdicts, and
        # `defer_truncated` records whether any cycle in it stopped on a probe
        # bound; all three reset the moment a goal is actually dispatched, so an
        # occasional deferral in an otherwise healthy run never accumulates
        # toward the cap.
        consec_defers = 0
        defer_definitive = True
        defer_truncated = False
        # Start-blocked cycles are counted SEPARATELY and on purpose: they make no
        # claim about the frontiers, so folding them into the defer streak would
        # both dilute that streak's verdict and end the run naming the wrong
        # fault. They neither advance nor reset the defer state — a defer streak
        # interrupted by a stuck-robot cycle is still the same streak.
        consec_start_blocked = 0
        while rclpy.ok():
            pose = self.sensor.robot_xy()
            if pose is None:
                self._hb('tf', f'waiting for {self.map_frame}->{self.base_frame} TF...')
                self._sleep(TF_WAIT)
                continue
            rx, ry = pose

            # Map version as SELECTION sees it: the pre-check's TTL clock, and
            # what a DEFER waits to advance past. Deliberately not reused as the
            # post-goal freshness baseline below — the pre-check spends wall
            # time, so by dispatch the map may already have moved on.
            _, seq_at_select = self.sensor.get_map()

            sel = self._select_goal(rx, ry, seq_at_select)
            if sel is None:
                self._log_termination_audit(goal_num)
                break
            if sel is DEFER:
                # Frontiers remain but none is dispatchable yet. Wait for SLAM
                # to publish a newer map — that is the only thing that can
                # change the answer — then re-select. Bounded and heartbeated
                # by _wait_for_fresh_map, and rate-limited after it, so a
                # planner that never clears a candidate cannot spin this loop.
                #
                # It could still never LEAVE it, though, and an explorer that
                # waits forever is the silent hang this module is built to
                # prevent. So the streak is capped, and which of the two ways it
                # ends is decided by what the deferrals were made of — the same
                # WORLD-vs-STACK distinction OutcomeClassifier draws for goals.
                #
                # The two labels are bound to the COUNTER, not to the loop: a
                # streak begins wherever consec_defers is 0, so they are re-armed
                # here rather than only where a streak ends. Both folds are
                # monotone — `and` can only lose True, `or` can only gain True —
                # so a label carried in from a previous streak can never recover,
                # and the run would report that streak's fault as this one's.
                # Re-arming to the folds' identity values keeps the fold below
                # unchanged and makes any future reset of consec_defers correct
                # by construction.
                if consec_defers == 0:
                    defer_definitive = True
                    defer_truncated = False
                consec_defers += 1
                defer_definitive = defer_definitive and self._last_defer_definitive
                defer_truncated = defer_truncated or self._last_defer_truncated
                if consec_defers >= MAX_CONSEC_DEFERS:
                    if defer_definitive:
                        # WORLD: the planner answered every time and the answer
                        # was "no". That is a result, not a malfunction. Let the
                        # termination audit account for the frontiers left
                        # behind (it WARNs that the map is not complete) and end
                        # the run on its own terms.
                        self.warn(
                            f'[robot_{self.robot_id}] {consec_defers} consecutive '
                            f'selection cycles produced no dispatchable goal and '
                            f'the planner condemned every candidate it was asked '
                            f'about — no reachable frontier remains within the '
                            f'pre-check budget; stopping.')
                        self._log_termination_audit(goal_num)
                        break
                    # STACK: "nothing is reachable" was never established, so a
                    # zero exit here would let a fault pass for a completed map.
                    # Two ways to land here, and they need different fixes, so
                    # name the one that actually happened.
                    if defer_truncated:
                        # The probe budget ran out before the candidate list did.
                        # Not a planner fault — we simply stopped asking, and the
                        # candidates we skipped may well have been reachable.
                        self.error(
                            f'[robot_{self.robot_id}] {consec_defers} consecutive '
                            f'selection cycles produced no dispatchable goal and at '
                            f'least one of them ran out of probe budget before it '
                            f'ran out of candidates — some frontiers were never '
                            f'examined, so this is NOT "nothing is reachable". '
                            f'Raise PRECHECK_MAX_PROBES (now {PRECHECK_MAX_PROBES}) '
                            f'or PRECHECK_CYCLE_BUDGET (now '
                            f'{PRECHECK_CYCLE_BUDGET:.0f}s wall); '
                            f'do not read this as a completed map.')
                    else:
                        # At least one candidate per cycle went unanswered — a
                        # wedged or merely slow planner_server.
                        self.error(
                            f'[robot_{self.robot_id}] {consec_defers} consecutive '
                            f'selection cycles produced no dispatchable goal and the '
                            f'pre-check never got a usable answer for some candidate '
                            f'— planner_server is not responding within '
                            f'{PLAN_CALL_WAIT:.1f}s wall, not "nothing is reachable". '
                            f'Raise PLAN_CALL_WAIT or set reachability_precheck:=false; '
                            f'do not read this as a completed map.')
                    self._log_termination_audit(goal_num)
                    return False
                self._wait_for_fresh_map(seq_at_select)
                self._sleep(MIN_GOAL_PERIOD)
                continue
            if sel is START_BLOCKED_SEL:
                # The robot's own pose is unplannable, so this cycle collected no
                # evidence about any frontier and retired nothing. Waiting is the
                # right move — SLAM refreshing the map can clear the cell the
                # robot is standing in, which is more than a DEFER can hope for,
                # since the robot does not move during either.
                #
                # This ENDS any defer streak in progress. A defer streak is a
                # claim about selection productivity — "the planner condemned
                # every candidate, N cycles running" — and a stuck robot is a
                # different fault interrupting it, not another instance of it.
                # Leaving the count standing would let 9 defers, any number of
                # start-blocked cycles, and 1 more defer trip the defer cap and
                # report a selection fault for cycles that were never contiguous.
                # The two LABELS are not reset here: they are re-armed where the
                # next streak begins (see the DEFER branch above), so they cannot
                # describe a streak they did not belong to.
                #
                # Not an escape hatch for the caps: consec_start_blocked is reset
                # only by a dispatch, never by a defer, so alternating
                # defer/start-blocked still terminates — on
                # MAX_CONSEC_START_BLOCKED with the stuck-robot ERROR, which is
                # the correct diagnosis for exactly that pattern.
                consec_defers = 0
                consec_start_blocked += 1
                if consec_start_blocked >= MAX_CONSEC_START_BLOCKED:
                    self.error(
                        f'[robot_{self.robot_id}] {consec_start_blocked} '
                        f'consecutive selection cycles ended with an unplannable '
                        f'START pose — the robot is stuck at ({rx:.2f}, {ry:.2f}) '
                        f'and no frontier evidence was collected in any of them; '
                        f'nothing was retired. This is a STUCK ROBOT, not a '
                        f'completed map: the frontiers were never judged. '
                        f'Recovery (back-up-and-clear) is not implemented, so the '
                        f'run cannot free itself; check the costmap around the '
                        f'pose above (inflation_radius vs robot_radius).')
                    self._log_termination_audit(goal_num)
                    return False
                self._wait_for_fresh_map(seq_at_select)
                self._sleep(MIN_GOAL_PERIOD)
                continue
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
            # A dispatched goal proves selection is still productive: the streak
            # is broken, not merely paused. Both verdict flags are streak-scoped
            # and reset with it: a cycle that ran out of probe budget BEFORE this
            # goal says nothing about a streak that begins after it, and a flag
            # left set makes every later cap-hit blame the budget and tell the
            # operator to raise PRECHECK_MAX_PROBES when the budget never bound.
            consec_defers = 0
            defer_definitive = True
            defer_truncated = False
            # A dispatch also proves the robot could plan out of where it was.
            consec_start_blocked = 0
            dist = math.hypot(gx - rx, gy - ry)
            self.info(f'[robot_{self.robot_id}] goal #{goal_num} -> ({gx:.2f}, {gy:.2f}) '
                      f'[{ncells} cells, frontier ({fx:.2f}, {fy:.2f}), {dist:.2f} m away]')
            self._last_goal = (gx, gy)
            dispatch_t = time.monotonic()
            sim_start = self.sensor.sim_time()
            # Remember the map version now so we can wait for a NEWER one afterward.
            _, seq_before = self.sensor.get_map()

            self.goToPose(self._make_goal(gx, gy))
            outcome, end_xy, window_move, (err_code, err_msg, recheck_code) = \
                self._supervise_goal(goal_num, (gx, gy))

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
            # reports the 1xx and the planner's verdict is masked. Without the
            # number that case is indistinguishable from the classification
            # simply not firing.
            #
            # planner_recheck= is that mask lifted: the code from the direct
            # ComputePathToPose re-query `_terminal_outcome` issues for exactly
            # those 1xx goals. Reading the two together is what turns "22 goals
            # reported 1xx" into a count of how many were really planner aborts
            # — the pair is the measurement this rung exists to take.
            window_note = ''
            if outcome == GoalSupervisor.FUTILE_NO_PROGRESS:
                window_note = (f' window={window_move:.2f}m/'
                               f'{GoalSupervisor.PROGRESS_WINDOW:.0f}s')
            err_note = f' error_code={err_code}' if err_code else ''
            if err_msg:
                err_note += f' error_msg="{err_msg}"'
            if recheck_code is not None:
                err_note += f' planner_recheck={recheck_code}'
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
    # Reachability pre-check: OFF by default so a run is a clean A/B against the
    # previous rung, and so this can never change behaviour by merely existing.
    # Read here beside robot_id because launch delivers params under the /**
    # wildcard (see explore.launch.py), which this bootstrap node also matches.
    boot.declare_parameter('reachability_precheck', False)
    robot_id = int(boot.get_parameter('robot_id').value)
    precheck = bool(boot.get_parameter('reachability_precheck').value)
    boot.destroy_node()

    # The sensor node (map + TF + clock) is spun continuously by its own
    # MultiThreadedExecutor in a daemon thread, so the explorer's callbacks are
    # never starved by BasicNavigator's global-executor spins on the main thread.
    sensor = _SensorNode(robot_id)
    sensor_exec = MultiThreadedExecutor()
    sensor_exec.add_node(sensor)
    sensor_thread = threading.Thread(target=sensor_exec.spin, daemon=True)
    sensor_thread.start()

    explorer = FrontierExplorer(robot_id, sensor, reachability_precheck=precheck)
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
