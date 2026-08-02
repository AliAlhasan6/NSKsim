#!/usr/bin/env python3
"""reachability.py — decide whether a candidate goal is worth dispatching.

Pure Python, no rclpy: the explorer hands over a ComputePathToPose result and
asks what it proves. Kept import-free of rclpy and nav2_msgs so its tests run
in any interpreter — unlike the frontier_explorer tests, which must skip when
nav2_simple_commander is absent.

Why a pre-check at all
----------------------
Selection had no reachability test: the nearest non-blacklisted centroid was
dispatched and its plannability discovered only after the goal ran and failed.
Unplannable frontiers were re-selected, the candidate set collapsed, and the
robot oscillated locally. Asking the planner FIRST turns a wasted goal into one
cheap query.

Why the endpoint check is mandatory
-----------------------------------
NavfnPlanner runs with `tolerance: 0.5` (nav2_robot0.yaml), so it may return
SUCCESS having planned to a cell up to 0.5 m from the point we asked about.
That path proves something is reachable NEAR the frontier — not that the
frontier is. Taking error_code==0 at face value would select a candidate whose
goal Nav2 then drives to and reports "reached" without the robot ever visiting
the frontier, which is the no-progress spin the explorer already fights
elsewhere. So a success is only REACHABLE if the path actually ENDS within
ENDPOINT_TOL of the requested point.

Why "inconclusive" is a first-class verdict
-------------------------------------------
Three-way, never two-way. A timeout, a TF error or a planner that never
answered says nothing about the world, and retiring a frontier on that evidence
would blacklist a perfectly good goal because the stack hiccuped — the exact
failure mode OutcomeClassifier exists to prevent. INCONCLUSIVE skips the
candidate for this cycle and leaves it fully eligible next time.

Why retirement expires
----------------------
Reachability is a function of the CURRENT map. A frontier walled off by unknown
space now may be trivially plannable once SLAM fills in the corridor leading to
it, so a permanent blacklist would throw away real exploration targets. The TTL
is counted in SLAM map versions rather than seconds: it tracks the thing that
actually changes reachability, and a stalled SLAM correctly never expires a
retirement (no new map, no new information, no reason to re-try).

Why the START needs its own verdict
-----------------------------------
`triage` asks "is this GOAL reachable". A planner reply can equally condemn the
START — the robot's own pose — and the two are different questions with opposite
remedies: one retires a frontier, the other says the frontier was never the
problem. 203/205 state it outright. 208 does not: NO_VALID_PATH reports that the
search failed without naming the end that failed it, so a robot parked inside
its own costmap inflation answers 208 for every goal it is asked about,
including reachable ones. `start_verdict` reads a reply as evidence about the
START, and is a SEPARATE function rather than a widening of `triage` (see its
docstring for why that distinction is load-bearing).
"""

import math

# ── endpoint verification ───────────────────────────────────────────────────
# m; how far the planned path's last pose may sit from the point we asked
# about before the "success" stops being evidence about THAT point. Tighter
# than the planner's own 0.5 m tolerance on purpose — see the module docstring.
ENDPOINT_TOL = 0.25

# ── planner verdicts that condemn the GOAL ──────────────────────────────────
# ComputePathToPose error codes that are WORLD evidence: the goal itself is
# geometrically unusable, so retrying learns nothing.
# 204 GOAL_OUTSIDE_MAP, 206 GOAL_OCCUPIED, 208 NO_VALID_PATH.
# Deliberately EXCLUDES 201 INVALID_PLANNER, 202 TF_ERROR, 207 TIMEOUT
# (infrastructure — the retry ladder is correct for those) and 203
# START_OUTSIDE_MAP / 205 START_OCCUPIED (wrong with the ROBOT's pose, not the
# frontier — blacklisting the frontier would be wrong).
#
# NOTE on 208 under this configuration: the planner runs with
# `allow_unknown: true`, so it plans straight THROUGH unmapped space and rarely
# declares NO_VALID_PATH. The pre-check therefore catches mostly 204/206. This
# is expected, not a bug — 208 is the case the post-hoc re-query backstops.
UNREACHABLE_CODES = frozenset({204, 206, 208})

# ── planner verdicts that condemn the START ─────────────────────────────────
# The ROBOT's own pose is at fault and the goal is not implicated at all:
# 203 START_OUTSIDE_MAP, 205 START_OCCUPIED (both defined in
# nav2_msgs/action/ComputePathToPose.action). The code IS the diagnosis here —
# no probe is needed or warranted — and retiring a frontier on one of these
# would blame the wrong end of the query.
START_SIDE_CODES = frozenset({203, 205})

# ── planner verdicts that name no endpoint at all ───────────────────────────
# 208 NO_VALID_PATH reports that the search failed; it does NOT say WHICH
# endpoint made it fail. That ambiguity is the whole problem: a robot sitting in
# its own inflation band returns 208 for every goal, reachable ones included, so
# a 208 taken at face value condemns frontiers that were never at fault. This is
# therefore the only set that warrants a start probe.
#
# 204 GOAL_OUTSIDE_MAP and 206 GOAL_OCCUPIED stay goal-side by construction —
# they name the goal — and 203/205 are already answered by START_SIDE_CODES, so
# neither group needs a probe and neither belongs here.
AMBIGUOUS_CODES = frozenset({208})

# ── verdicts ────────────────────────────────────────────────────────────────
REACHABLE = 'REACHABLE'          # plan it: the planner reached the point itself
UNREACHABLE = 'UNREACHABLE'      # retire it: the planner condemned the goal
INCONCLUSIVE = 'INCONCLUSIVE'    # skip this cycle, retire NOTHING

# Start-side verdicts, from `start_verdict`. Their names deliberately contain
# neither REACHABLE nor INCONCLUSIVE as a substring: the explorer's log tests
# (test_frontier_explorer_precheck.py:435 and :444) match verdict names as
# SUBSTRINGS of logged text, so a name containing either would make those
# assertions fire on the wrong line.
START_OK = 'START_OK'            # the planner can propagate a path from this pose
START_BLOCKED = 'START_BLOCKED'  # the START is at fault, whatever the goal was


def endpoint_within(path_xy, requested_xy, tol=ENDPOINT_TOL) -> bool:
    """True if `path_xy` ends within `tol` metres of `requested_xy`.

    `path_xy` is the planned path as [(x, y), ...]. An empty or missing path is
    False: there is no endpoint to verify, so it cannot be evidence that the
    requested point was reached (a zero-pose path with error_code 0 is a
    degenerate success, not a usable plan).
    """
    if not path_xy:
        return False
    ex, ey = path_xy[-1]
    rx, ry = requested_xy
    return math.hypot(ex - rx, ey - ry) <= tol


def triage(error_code, path_xy, requested_xy, tol=ENDPOINT_TOL) -> str:
    """Classify one ComputePathToPose result for the point we asked about.

    `error_code` is the result's code, or None when the call timed out, was
    rejected, or the server never answered.

    Exactly three outcomes (see the module docstring for why):
      * REACHABLE    — code 0, a non-empty path, and that path ENDS within
                       `tol` of `requested_xy`.
      * UNREACHABLE  — the planner condemned the goal (UNREACHABLE_CODES).
                       The only verdict that may retire a frontier.
      * INCONCLUSIVE — everything else. Notably includes a code-0 success whose
                       path stops short of the requested point: the planner
                       exercised its own 0.5 m tolerance and answered about a
                       DIFFERENT point, which is not evidence either way.
    """
    if error_code in UNREACHABLE_CODES:
        return UNREACHABLE
    if error_code == 0 and endpoint_within(path_xy, requested_xy, tol):
        return REACHABLE
    return INCONCLUSIVE


def start_verdict(error_code, path_xy) -> str:
    """Classify one ComputePathToPose result as evidence about the START pose.

    `error_code` is the result's code, or None when the call timed out, was
    rejected, or the server never answered. Three outcomes:
      * START_BLOCKED  — the robot's own pose is unplannable.
      * START_OK       — the planner propagated a path out of this pose.
      * INCONCLUSIVE   — the reply says nothing either way (reuses triage's
                         constant: the doctrine is identical, and a caller that
                         treats INCONCLUSIVE as "retire nothing, retry later"
                         is already correct for both).

    Why this is a separate function and not a fourth verdict from `triage`
    ----------------------------------------------------------------------
    `triage`'s consumers compare its result with `==` and end in a bare `else`
    (frontier_explorer.py:744-752, and again at :969). A fourth value in THAT
    vocabulary would not raise anywhere — it would fall into the `else` and be
    silently counted as inconclusive, and the `== UNREACHABLE` test at :969
    would silently be False. A separate function cannot reach those comparisons,
    so the new verdicts can only be observed by call sites written to handle
    them.

    Why there is NO endpoint check
    ------------------------------
    This is where `start_verdict` and `triage` genuinely diverge, and it is
    deliberate. `triage` must verify the path ENDS at the point asked about,
    because NavfnPlanner's `tolerance: 0.5` lets a "success" answer about a
    different point (see the module docstring). Here the question is only
    whether the planner can propagate out of the current pose AT ALL: a start
    inside an obstacle or an inflation band yields no path to anywhere, so any
    non-empty path is proof the start is usable, however far short of the probe
    target it stops. Applying the endpoint rule here would discard exactly the
    evidence being sought — the same reply is START_OK here and INCONCLUSIVE
    through `triage`.

    On the UNREACHABLE_CODES branch
    -------------------------------
    Meaningful ONLY for a probe reply. Those codes condemn the point that was
    asked about, so on a candidate they are goal-side evidence and `triage` is
    the right reader. Reaching them HERE means the caller already established
    the original code was ambiguous (AMBIGUOUS_CODES) and then failed to plan to
    a deliberately benign nearby target as well — at which point the common
    factor is the start, not the goal.
    """
    if error_code in START_SIDE_CODES:
        return START_BLOCKED
    if error_code in UNREACHABLE_CODES:
        return START_BLOCKED
    if error_code == 0 and path_xy:
        return START_OK
    return INCONCLUSIVE


class RetirementLedger:
    """Frontiers the pre-check condemned, held for a bounded number of maps.

    Keyed on the same quantisation OutcomeClassifier uses, so two centroids that
    would collapse under the blacklist radius share one entry and a frontier
    that drifts by a few centimetres between scans stays retired.

    The clock is the SLAM map sequence number (``_SensorNode._map_seq``), not
    wall or sim time: an entry retired at map N is reconsidered once map N+ttl
    has arrived, i.e. after the map has actually changed enough times to
    plausibly change the answer.
    """

    def __init__(self, quantum: float, ttl_maps: int):
        # quantum (m): frontier key resolution — pass BLACKLIST_RADIUS.
        # ttl_maps: how many newer SLAM maps must arrive before re-considering.
        self._quantum = quantum
        self._ttl_maps = ttl_maps
        self._until = {}    # key -> map_seq at which the entry expires

    def _key(self, xy):
        x, y = xy
        return (round(x / self._quantum), round(y / self._quantum))

    def retire(self, xy, map_seq: int) -> int:
        """Retire `xy` until `ttl_maps` newer maps have arrived.

        Returns the expiry sequence, for logging. Re-retiring an already-retired
        frontier extends it from the CURRENT map, which is right: the planner
        condemned it again on fresher evidence.
        """
        until = map_seq + self._ttl_maps
        self._until[self._key(xy)] = until
        return until

    def is_retired(self, xy, map_seq: int) -> bool:
        """True while `xy` is still retired at map version `map_seq`.

        Expired entries are dropped as they are read, so the ledger cannot grow
        without bound over a long run.
        """
        key = self._key(xy)
        until = self._until.get(key)
        if until is None:
            return False
        if map_seq >= until:
            del self._until[key]
            return False
        return True

    def active(self, map_seq: int) -> int:
        """How many retirements are still live at `map_seq` (audit/logging).

        Purges expired entries as a side effect, same as ``is_retired``.
        """
        for key in [k for k, until in self._until.items() if map_seq >= until]:
            del self._until[key]
        return len(self._until)
