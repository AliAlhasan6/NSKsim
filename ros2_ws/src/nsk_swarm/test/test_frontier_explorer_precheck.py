"""Layer 1 — the reachability pre-check wired into frontier selection.

The triage itself is covered in test_reachability.py; this drives the SELECTION
path around it: candidate ordering, which candidates get probed, what gets
retired, and — the load-bearing one — that a cycle the pre-check cannot resolve
returns DEFER rather than None.

Why DEFER matters enough to test six ways
-----------------------------------------
`None` from _select_goal means "no frontiers remain" and BREAKS the explore
loop, printing the termination audit. If a cycle where the planner simply
declined to clear a candidate returned None, the run would stop early and
report a completed map that was never mapped — the exact silent-false-success
the termination audit exists to prevent. So every path that fails to produce a
goal while frontiers are still live must return DEFER.

Per the stub pattern in test_frontier_explorer_gate.py, the real unbound methods
are bound to a plain SimpleNamespace — no rclpy.init, no node, no
BasicNavigator. The planner is a scripted fake, so no Nav2 runs and every
verdict is exact rather than timing-dependent.
"""

import math
import os
from types import SimpleNamespace

import pytest

# Same guard, and the same reasoning, as test_frontier_explorer_gate.py:
# FrontierExplorer subclasses BasicNavigator, so the nav2_simple_commander
# import is module-level and cannot be deferred. CI sets NSK_REQUIRE_NAV2=1 and
# installs Nav2 deliberately, so a miss there is fatal rather than skipped.
try:
    from nsk_swarm import frontier_explorer as fx
    from nsk_swarm import reachability
    from nsk_swarm.frontier_explorer import (DEFER, START_BLOCKED_SEL,
                                             FrontierExplorer)
except ImportError:
    if os.environ.get('NSK_REQUIRE_NAV2', '') not in ('', '0', 'false'):
        raise
    fx = FrontierExplorer = DEFER = START_BLOCKED_SEL = reachability = None

pytestmark = pytest.mark.skipif(
    fx is None,
    reason='nav2_simple_commander is not importable — install '
           'ros-jazzy-navigation2 to run the reachability pre-check tests')


class FakePlanner:
    """Scripted stand-in for _SensorNode.plan_to().

    `replies` maps a rounded (x, y) goal to the tuple plan_to would return —
    (error_code, path_xy, planning_time, error_msg) — or None for "no answer".
    Goals with no scripted reply default to a clean success AT the requested
    point, so a test only has to spell out the candidates it cares about.
    """

    def __init__(self, replies=None):
        self.replies = replies or {}
        self.asked = []

    def plan_to(self, x, y, frame, timeout=None):
        self.asked.append((round(x, 3), round(y, 3)))
        key = (round(x, 3), round(y, 3))
        if key in self.replies:
            return self.replies[key]
        return (0, [(0.0, 0.0), (x, y)], 0.01, '')


def reaches(x, y):
    """A planner reply that genuinely arrives at (x, y)."""
    return (0, [(0.0, 0.0), (x, y)], 0.02, '')


def condemns(code=208):
    """A planner reply condemning the goal (204/206/208)."""
    return (code, [], 0.01, 'no valid path')


def short_of(x, y, gap=0.4):
    """A code-0 success whose path stops `gap` metres short of (x, y).

    The planner's own `tolerance: 0.5` makes this a reply it really can produce.
    """
    return (0, [(0.0, 0.0), (x, y - gap)], 0.02, '')


def make_explorer_stub(centroids, planner, precheck=True, blacklist=None,
                       start_probe=None):
    """Bind the real selection methods to a stub with a scripted frontier set.

    `centroids` is what _frontier_centroids() would return: [(x, y, ncells)].

    `start_probe` is the verdict the start probe returns, defaulting to START_OK
    — "the robot's pose is fine", which is what every pre-existing test was
    written against. The probe is STUBBED rather than bound for real on purpose:
    a real one issues its own plan_to calls, and several tests assert
    `planner.asked` exactly or count calls against the wall budget, so a probe
    silently inserting round trips would break assertions that are load-bearing
    regression pins for rung2c. Each call is recorded in `stub.start_probes`, so
    "asked at most once per cycle" is still assertable. Tests that exercise the
    real _start_probe bind it explicitly.
    """
    stub = SimpleNamespace(
        robot_id=0,
        sensor=SimpleNamespace(plan_to=planner.plan_to),
        map_frame='robot_0/map',
        _precheck=precheck,
        _blacklist=list(blacklist or []),
        _retirements=fx.RetirementLedger(fx.BLACKLIST_RADIUS,
                                         fx.RETIRE_TTL_MAPS),
        _hb_last={},
        start_probes=[],
        logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    stub.error = lambda msg: stub.logged.append(('ERROR', msg))
    stub._frontier_centroids = lambda: list(centroids)
    verdict = reachability.START_OK if start_probe is None else start_probe
    stub._start_probe = lambda rx, ry, deadline=None: (
        stub.start_probes.append((rx, ry)) or verdict)
    stub._free_targets_near = lambda rx, ry: [(0.5, 0.0)]
    for name in ('_select_goal', '_ordered_candidates', '_probe',
                 '_blacklisted', '_hb'):
        setattr(stub, name, getattr(FrontierExplorer, name).__get__(stub))
    return stub


# Three frontiers due north of a robot at the origin, ordered by distance and
# all comfortably beyond MIN_GOAL_DIST so the projection branch stays out of it.
NEAR, MID, FAR = (0.0, 1.0, 10), (0.0, 2.0, 20), (0.0, 3.0, 30)
THREE = [FAR, NEAR, MID]        # deliberately unsorted: selection must order them


# ── ordering, and the OFF path ──────────────────────────────────────────────

def test_candidates_come_back_nearest_first():
    stub = make_explorer_stub(THREE, FakePlanner())
    goals = [g for g, _f, _n in stub._ordered_candidates(0.0, 0.0)]
    assert goals == [(0.0, 1.0), (0.0, 2.0), (0.0, 3.0)]


def test_precheck_off_takes_the_nearest_without_asking_the_planner():
    # The A/B baseline: with the parameter OFF the behaviour must be exactly
    # what it was before this change, including issuing no planner query at all.
    planner = FakePlanner()
    stub = make_explorer_stub(THREE, planner, precheck=False)
    goal_xy, frontier_xy, ncells = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 1.0)
    assert frontier_xy == (0.0, 1.0)
    assert ncells == 10
    assert planner.asked == []


def test_precheck_off_returns_none_when_no_frontiers_remain():
    stub = make_explorer_stub([], FakePlanner(), precheck=False)
    assert stub._select_goal(0.0, 0.0, 0) is None


def test_blacklisted_frontiers_are_excluded():
    stub = make_explorer_stub(THREE, FakePlanner(), precheck=False,
                              blacklist=[(0.0, 1.0)])
    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 2.0)


# ── the pre-check ───────────────────────────────────────────────────────────

def test_first_reachable_candidate_is_selected_and_stops_the_probing():
    planner = FakePlanner({(0.0, 1.0): reaches(0.0, 1.0)})
    stub = make_explorer_stub(THREE, planner)
    goal_xy, frontier_xy, ncells = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 1.0)
    assert ncells == 10
    # Exactly one query: the budget is spent lazily, not eagerly on all three.
    assert planner.asked == [(0.0, 1.0)]


def test_unreachable_candidates_are_skipped_and_retired():
    planner = FakePlanner({
        (0.0, 1.0): condemns(206),      # GOAL_OCCUPIED
        (0.0, 2.0): condemns(208),      # NO_VALID_PATH
        (0.0, 3.0): reaches(0.0, 3.0),
    })
    stub = make_explorer_stub(THREE, planner)
    goal_xy, frontier_xy, ncells = stub._select_goal(0.0, 0.0, 0)

    assert goal_xy == (0.0, 3.0)
    assert ncells == 30
    assert planner.asked == [(0.0, 1.0), (0.0, 2.0), (0.0, 3.0)]
    # Both condemned frontiers are retired; the selected one is not.
    assert stub._retirements.is_retired((0.0, 1.0), 0) is True
    assert stub._retirements.is_retired((0.0, 2.0), 0) is True
    assert stub._retirements.is_retired((0.0, 3.0), 0) is False


def test_retired_frontiers_are_not_re_probed_next_cycle():
    planner = FakePlanner({(0.0, 1.0): condemns(204),
                           (0.0, 2.0): reaches(0.0, 2.0)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)
    planner.asked.clear()

    # Next cycle, one map later: the retired frontier is gone from the candidate
    # list entirely, so no budget is wasted re-asking about it.
    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, 1)
    assert goal_xy == (0.0, 2.0)
    assert (0.0, 1.0) not in planner.asked


def test_retirement_expires_and_the_frontier_returns():
    planner = FakePlanner({(0.0, 1.0): condemns(208),
                           (0.0, 2.0): reaches(0.0, 2.0)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)
    assert stub._retirements.is_retired((0.0, 1.0), 0) is True

    # RETIRE_TTL_MAPS newer maps later the map has changed enough that the
    # answer might have too, so the frontier is a candidate again.
    planner.replies[(0.0, 1.0)] = reaches(0.0, 1.0)
    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, fx.RETIRE_TTL_MAPS)
    assert goal_xy == (0.0, 1.0)


# ── escalation: probe past the head of the list, within one cycle ───────────
#
# rung2c halted after 15 goals (rung2b, pre-check OFF, ran 118): selection
# probed a fixed top-3 window, and when all three came back UNREACHABLE it
# DEFERred — 10 times, then stopped, with 41 candidates it had never asked
# about. Deferring cannot help: the robot does not move during a DEFER, so the
# map barely changes and the next cycle re-derives the same verdict. The answer
# is to keep walking the list in the SAME cycle.


def line_of(count, start=1):
    """`count` frontiers due north at 1 m spacing, nearest-first when sorted."""
    return [(0.0, float(i), 10) for i in range(start, start + count)]


def condemn_all(centroids):
    """A planner that condemns every one of `centroids` (as goals)."""
    return FakePlanner({(x, y): condemns(208) for x, y, _n in centroids})


def test_escalation_finds_a_reachable_candidate_past_the_old_window():
    # THE regression test for rung2c. The first five candidates are unplannable
    # and the sixth is fine — under the old top-3 window this cycle produced
    # DEFER and the run walked toward the defer cap without ever asking about
    # the goal it could have driven to.
    many = line_of(8)
    planner = condemn_all(many)
    planner.replies[(0.0, 6.0)] = reaches(0.0, 6.0)
    stub = make_explorer_stub(many, planner)

    goal_xy, frontier_xy, ncells = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 6.0)
    assert frontier_xy == (0.0, 6.0)
    # Six probes: the five it had to reject, then the one it took. Still lazy —
    # positions 7 and 8 are never queried.
    assert planner.asked == [(0.0, float(i)) for i in range(1, 7)]
    # The five rejects are real planner verdicts and are retired as such.
    for i in range(1, 6):
        assert stub._retirements.is_retired((0.0, float(i)), 0) is True
    assert stub._retirements.is_retired((0.0, 6.0), 0) is False


def test_escalation_stops_at_the_probe_cap():
    # The count bound. Probes are nearly free here (the fake planner answers
    # instantly), so this is the bound that binds in a healthy run: one
    # selection cycle cannot walk an arbitrarily long frontier list.
    many = line_of(30)
    planner = condemn_all(many)
    stub = make_explorer_stub(many, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    # The budget is spent on the NEAREST candidates, in order — escalation
    # extends the walk, it does not reorder or skip.
    assert planner.asked == [(0.0, float(i))
                             for i in range(1, fx.PRECHECK_MAX_PROBES + 1)]
    assert len(many) > fx.PRECHECK_MAX_PROBES, 'the cap must actually bind here'
    # Stopping on a bound is NOT evidence that nothing is reachable — the
    # remaining candidates were never asked about.
    assert stub._last_defer_truncated is True
    assert stub._last_defer_definitive is False


def test_escalation_stops_at_the_wall_budget():
    # The time bound, and the reason there are two. planning_time is ~0.000s
    # today so the count cap alone looks sufficient, but NavFn's cost grows with
    # the map: once each probe costs real wall time, the deadline has to bind
    # FIRST or one selection cycle stalls the robot for PRECHECK_MAX_PROBES x
    # PLAN_CALL_WAIT seconds. Here every probe burns a full PLAN_CALL_WAIT, so
    # the budget runs out well before the 12-probe cap does.
    clock = FakeClock()
    many = line_of(30)
    planner = condemn_all(many)
    slow = planner.plan_to

    def plan_to(x, y, frame, timeout=None):
        clock.now += fx.PLAN_CALL_WAIT
        return slow(x, y, frame, timeout)

    planner.plan_to = plan_to
    stub = make_explorer_stub(many, planner)

    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(fx, 'time', clock)
        assert stub._select_goal(0.0, 0.0, 0) is DEFER

    expected = int(fx.PRECHECK_CYCLE_BUDGET // fx.PLAN_CALL_WAIT)
    assert len(planner.asked) == expected
    assert expected < fx.PRECHECK_MAX_PROBES, (
        'the wall budget must bind before the probe cap when probes are slow, '
        'or the cap alone decides how long a cycle can stall the robot')
    assert stub._last_defer_truncated is True
    assert stub._last_defer_definitive is False


def test_the_healthy_case_still_costs_one_probe():
    # Escalation must not turn into eager probing: when the nearest candidate is
    # plannable, the budget is not spent at all.
    many = line_of(30)
    planner = FakePlanner({(0.0, 1.0): reaches(0.0, 1.0)})
    stub = make_explorer_stub(many, planner)

    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 1.0)
    assert planner.asked == [(0.0, 1.0)]


def test_escalation_walks_past_inconclusive_candidates_too():
    # The second mechanism behind the rung2c stall. INCONCLUSIVE retires
    # NOTHING (a timeout is evidence about the stack, not the world), so an
    # unanswered candidate keeps its nearest-first position forever — a narrow
    # window parked behind two of them re-probes the same pair every cycle and
    # never advances. Escalation has to step over them, not just over
    # UNREACHABLE ones.
    many = line_of(8)
    planner = FakePlanner({(0.0, 1.0): None,                    # no answer
                           (0.0, 2.0): (207, [], 2.0, ''),      # TIMEOUT
                           (0.0, 3.0): condemns(206),
                           (0.0, 4.0): short_of(0.0, 4.0),      # stops short
                           (0.0, 5.0): reaches(0.0, 5.0)})
    stub = make_explorer_stub(many, planner)

    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 5.0)
    # Only the condemned one retires; the three inconclusives stay eligible.
    assert stub._retirements.is_retired((0.0, 3.0), 0) is True
    assert stub._retirements.active(0) == 1


# ── DEFER: the run must not end early ───────────────────────────────────────

def test_all_unreachable_defers_rather_than_claiming_completion():
    planner = FakePlanner({(0.0, 1.0): condemns(204),
                           (0.0, 2.0): condemns(206),
                           (0.0, 3.0): condemns(208)})
    stub = make_explorer_stub(THREE, planner)
    # None here would break the explore loop and print "exploration complete"
    # while three live frontiers sat in the map.
    assert stub._select_goal(0.0, 0.0, 0) is DEFER


def test_all_inconclusive_defers_and_retires_nothing():
    # A timeout, a TF error and a silent planner. None of these is evidence
    # about the world, so all three frontiers stay fully eligible.
    planner = FakePlanner({(0.0, 1.0): None,                  # no answer
                           (0.0, 2.0): (202, [], 0.0, 'tf'),  # TF_ERROR
                           (0.0, 3.0): (207, [], 2.0, '')})   # TIMEOUT
    stub = make_explorer_stub(THREE, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    for xy in ((0.0, 1.0), (0.0, 2.0), (0.0, 3.0)):
        assert stub._retirements.is_retired(xy, 0) is False
    assert stub._retirements.active(0) == 0


def test_a_success_short_of_the_goal_defers_and_retires_nothing():
    # The tolerance-0.5 case reaching selection: the planner returns SUCCESS
    # having planned to a point 0.4 m away. It must neither be selected (the
    # robot would "arrive" without visiting the frontier) nor retired (the
    # planner raised no objection to the point).
    planner = FakePlanner({(0.0, 1.0): short_of(0.0, 1.0),
                           (0.0, 2.0): short_of(0.0, 2.0),
                           (0.0, 3.0): short_of(0.0, 3.0)})
    stub = make_explorer_stub(THREE, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._retirements.active(0) == 0


def test_all_candidates_retired_defers_rather_than_returning_none():
    # The candidate list is empty only because everything in it is PARKED. That
    # is pending work, not a finished map, so it must not end the run.
    planner = FakePlanner({(0.0, 1.0): condemns(208),
                           (0.0, 2.0): condemns(208),
                           (0.0, 3.0): condemns(208)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)
    assert stub._retirements.active(0) == 3

    assert stub._ordered_candidates(0.0, 0.0, 1) == []
    assert stub._select_goal(0.0, 0.0, 1) is DEFER


def test_genuinely_empty_map_still_returns_none():
    # The completion path has to survive all of the above: no frontiers and no
    # retirements pending really is done, and must still end the run.
    stub = make_explorer_stub([], FakePlanner())
    assert stub._select_goal(0.0, 0.0, 0) is None


# ── the projection branch ───────────────────────────────────────────────────

def test_projected_goal_is_probed_but_the_frontier_is_retired():
    # Every frontier is inside MIN_GOAL_DIST, so the dispatched goal is a
    # PROJECTED point, not the centroid. The planner must be asked about the
    # point that will actually be sent, while retirement keys on the frontier —
    # confusing the two would retire a frontier the planner never judged.
    close = [(0.0, 0.2, 10)]
    projected = (0.0, fx.GOAL_PROJECT_DIST)
    planner = FakePlanner({projected: condemns(206)})
    stub = make_explorer_stub(close, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert planner.asked == [projected]                      # probed the GOAL
    assert stub._retirements.is_retired((0.0, 0.2), 0) is True   # retired the FRONTIER


def test_projected_goal_selected_when_reachable():
    close = [(0.0, 0.2, 10)]
    projected = (0.0, fx.GOAL_PROJECT_DIST)
    planner = FakePlanner({projected: reaches(*projected)})
    stub = make_explorer_stub(close, planner)

    goal_xy, frontier_xy, ncells = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == pytest.approx(projected)
    assert frontier_xy == (0.0, 0.2)
    assert ncells == 10


# ── logging (the run data the candidate budget gets tuned from) ─────────────

def test_every_probe_logs_planning_time_and_error_msg():
    planner = FakePlanner({(0.0, 1.0): (208, [], 0.123, 'no valid path'),
                           (0.0, 2.0): reaches(0.0, 2.0)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)

    logs = [msg for _lvl, msg in stub.logged]
    assert any('planning_time=0.123s' in m and 'error_code=208' in m
               and 'no valid path' in m for m in logs), logs
    assert any('planning_time=' in m and 'REACHABLE' in m for m in logs), logs


def test_a_silent_planner_is_logged_as_inconclusive():
    planner = FakePlanner({(0.0, 1.0): None, (0.0, 2.0): reaches(0.0, 2.0)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)

    logs = [msg for _lvl, msg in stub.logged]
    assert any('no answer' in m and 'INCONCLUSIVE' in m for m in logs), logs


def test_defer_records_whether_the_cycle_was_definitive():
    # The flag explore()'s defer cap reads. All three condemned -> we have real
    # verdicts; swap one for a timeout and the cycle is no longer definitive.
    condemned = {(0.0, 1.0): condemns(204), (0.0, 2.0): condemns(206),
                 (0.0, 3.0): condemns(208)}
    stub = make_explorer_stub(THREE, FakePlanner(dict(condemned)))
    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._last_defer_definitive is True

    tainted = dict(condemned)
    tainted[(0.0, 3.0)] = None          # the third probe never gets an answer
    stub = make_explorer_stub(THREE, FakePlanner(tainted))
    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._last_defer_definitive is False


def test_parked_candidates_defer_definitively():
    # Nothing to probe because everything is retired — but the retirements ARE
    # planner verdicts, so the cycle is definitive.
    planner = FakePlanner({(0.0, 1.0): condemns(208), (0.0, 2.0): condemns(208),
                           (0.0, 3.0): condemns(208)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)
    assert stub._select_goal(0.0, 0.0, 1) is DEFER
    assert stub._last_defer_definitive is True


def test_a_parked_cycle_does_not_inherit_the_last_cycles_truncation():
    # The labels belong to the cycle that produced them. The parked path runs no
    # probe loop, so it cannot have been stopped by a bound — but it used to
    # write only _last_defer_definitive, leaving _last_defer_truncated at
    # whatever the previous cycle left. explore() OR-folds that, so one stale
    # True makes a later streak report "raise PRECHECK_MAX_PROBES" for a cycle
    # in which the budget never bound. Same defect class as 1ea37bc, one level
    # down: state outliving the episode it described.
    planner = FakePlanner({(0.0, 1.0): condemns(208), (0.0, 2.0): condemns(208),
                           (0.0, 3.0): condemns(208)})
    stub = make_explorer_stub(THREE, planner)
    stub._select_goal(0.0, 0.0, 0)          # retires all three
    # Stand in for the cycle IMMEDIATELY before this one having hit the probe
    # cap. Planted here, not earlier: a probe-loop cycle in between would write
    # the flag itself and the parked path would never be the one under test —
    # which is the realistic shape too (a cap-truncated cycle, then a cycle
    # whose remaining candidates are all parked).
    stub._last_defer_truncated = True

    assert stub._select_goal(0.0, 0.0, 1) is DEFER
    assert stub._last_defer_truncated is False
    assert stub._last_defer_definitive is True


def test_a_dispatching_cycle_does_not_leave_a_stale_truncation_behind():
    # The same reset covers every exit from _select_goal, not just the parked
    # one — including the pre-check-OFF path, which returns before any label
    # could otherwise be written.
    stub = make_explorer_stub(THREE, FakePlanner(), precheck=False)
    stub._last_defer_truncated = True

    goal_xy, _f, _n = stub._select_goal(0.0, 0.0, 0)
    assert goal_xy == (0.0, 1.0)
    assert stub._last_defer_truncated is False


def test_defer_is_definitive_when_the_whole_list_was_examined():
    # The honest halt, and the behaviour escalation must not cost us. Five
    # candidates — fewer than PRECHECK_MAX_PROBES, so the LIST runs out before
    # either bound does — and the planner condemned every one of them. That is
    # a real answer, so the cycle is definitive and a streak of them ends the
    # run cleanly (WORLD) rather than as an infrastructure failure.
    five = line_of(5)
    planner = condemn_all(five)
    stub = make_explorer_stub(five, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert len(planner.asked) == 5, 'every candidate must be asked about'
    assert stub._last_defer_truncated is False
    assert stub._last_defer_definitive is True
    assert stub._retirements.active(0) == 5


def test_an_escalated_cycle_with_unanswered_probes_is_not_definitive():
    # The WORLD/STACK split under escalation: a longer walk down the list does
    # not make an unanswered probe any more conclusive. The list is exhausted
    # (no truncation), but one candidate never got a verdict, so the cycle
    # cannot support "nothing is reachable".
    five = line_of(5)
    planner = condemn_all(five)
    planner.replies[(0.0, 4.0)] = None          # never answers
    stub = make_explorer_stub(five, planner)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert len(planner.asked) == 5
    assert stub._last_defer_truncated is False
    assert stub._last_defer_definitive is False
    # The unanswered candidate is NOT retired — a silent planner is evidence
    # about the stack, not about the frontier.
    assert stub._retirements.is_retired((0.0, 4.0), 0) is False
    assert stub._retirements.active(0) == 4


# ── the start-side probe ────────────────────────────────────────────────────
#
# rung2f_on_explore.log: 42 consecutive plan attempts from ONE pose, every one
# error_code=208 with poses=0, two of them to goals 0.70 m away. The START was
# unplannable — the robot was inside its own costmap inflation — so every
# candidate failed identically, and selection charged each failure to the
# frontier. 21 frontiers were retired, the candidate pool starved, and the run
# halted with 1036 frontier cells outstanding. A 208 may not condemn a frontier
# until the start has been cleared.

def condemn_all_with(centroids, code):
    """A planner condemning every centroid with a specific error code."""
    return FakePlanner({(x, y): condemns(code) for x, y, _n in centroids})


def test_a_blocked_start_returns_the_sentinel_and_retires_nothing():
    many = line_of(4)
    stub = make_explorer_stub(many, condemn_all(many),
                              start_probe=reachability.START_BLOCKED)

    assert stub._select_goal(0.0, 0.0, 0) is START_BLOCKED_SEL
    # THE fix: not one frontier retired, where the old code retired every
    # candidate it probed.
    assert stub._retirements.active(0) == 0
    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any('START is at fault' in m for m in warns), warns
    # The goal-side grep key must NOT appear — prior log analysis counts it.
    assert not any('unplannable — retiring until map #' in m for m in warns)


def test_a_blocked_start_abandons_the_rest_of_the_cycle():
    # While the start is bad every candidate fails identically, so continuing to
    # probe only manufactures more false evidence (and, in rung2f, more
    # retirements). One candidate, one start probe, done.
    many = line_of(8)
    planner = condemn_all(many)
    stub = make_explorer_stub(many, planner,
                              start_probe=reachability.START_BLOCKED)

    assert stub._select_goal(0.0, 0.0, 0) is START_BLOCKED_SEL
    assert planner.asked == [(0.0, 1.0)]
    assert len(stub.start_probes) == 1


def test_a_cleared_start_retires_exactly_as_before():
    # Regression pin on the goal-side path: when the probe says the pose is
    # fine, a 208 is real evidence about the goal and behaves as it always has.
    many = line_of(3)
    stub = make_explorer_stub(many, condemn_all(many))     # start_probe=START_OK

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._retirements.active(0) == 3
    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert all('unplannable — retiring until map #' in m for m in warns), warns
    # ...and it now SAYS why it blamed the goal rather than assuming it.
    assert any('start probe cleared the robot pose' in m for m in warns), warns


def test_the_start_is_probed_at_most_once_per_cycle():
    # The start verdict is a property of the pose, not of the candidate. Asking
    # per candidate would cost a round trip each and could report two different
    # answers within one cycle.
    many = line_of(6)
    stub = make_explorer_stub(many, condemn_all(many))

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._retirements.active(0) == 6
    assert stub.start_probes == [(0.0, 0.0)]


def test_goal_side_codes_never_probe_the_start():
    # 204 GOAL_OUTSIDE_MAP and 206 GOAL_OCCUPIED name the goal, so the start is
    # not implicated and a probe would be a wasted round trip every cycle.
    for code in (204, 206):
        many = line_of(3)
        stub = make_explorer_stub(many, condemn_all_with(many, code))

        assert stub._select_goal(0.0, 0.0, 0) is DEFER
        assert stub.start_probes == []
        assert stub._retirements.active(0) == 3


def test_start_side_codes_never_probe_and_never_retire():
    # 203 START_OUTSIDE_MAP / 205 START_OCCUPIED: the code IS the diagnosis, so
    # there is nothing to probe — and the goal was never judged, so there is
    # nothing to retire either.
    for code in (203, 205):
        many = line_of(3)
        planner = condemn_all_with(many, code)
        stub = make_explorer_stub(many, planner)

        assert stub._select_goal(0.0, 0.0, 0) is START_BLOCKED_SEL
        assert stub.start_probes == []
        assert stub._retirements.active(0) == 0
        assert planner.asked == [(0.0, 1.0)]
        warns = [m for lvl, m in stub.logged if lvl == 'WARN']
        assert any('START is at fault' in m for m in warns), warns


def test_an_unattributed_208_retires_nothing_and_taints_the_cycle():
    # The probe could not establish the start either way, so the 208 belongs to
    # neither endpoint. Retiring on it would be exactly the over-reading this
    # path exists to prevent, and the cycle cannot claim to be definitive.
    many = line_of(3)
    stub = make_explorer_stub(many, condemn_all(many),
                              start_probe=reachability.INCONCLUSIVE)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    assert stub._retirements.active(0) == 0
    assert stub._last_defer_definitive is False
    # Still only one probe: an unestablished start is unestablished for every
    # candidate in the cycle.
    assert len(stub.start_probes) == 1


# ── _start_probe itself ─────────────────────────────────────────────────────

def make_probe_stub(planner, targets):
    """Bind the REAL _start_probe over a scripted target list and planner."""
    stub = SimpleNamespace(robot_id=0, map_frame='robot_0/map',
                           sensor=SimpleNamespace(plan_to=planner.plan_to),
                           logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    stub._free_targets_near = lambda rx, ry: list(targets)
    stub._start_probe = FrontierExplorer._start_probe.__get__(stub)
    return stub


TARGETS = [(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0)]


def test_the_first_reachable_target_clears_the_start_and_stops_probing():
    planner = FakePlanner({(0.5, 0.0): condemns(208),
                           (0.0, 0.5): reaches(0.0, 0.5)})
    stub = make_probe_stub(planner, TARGETS)

    assert stub._start_probe(0.0, 0.0) == reachability.START_OK
    assert planner.asked == [(0.5, 0.0), (0.0, 0.5)]


def test_a_short_path_still_clears_the_start():
    # The divergence from candidate probing: the target is not where we are
    # going, it is a question about the pose. Any path answers it.
    planner = FakePlanner({(0.5, 0.0): short_of(0.5, 0.0)})
    stub = make_probe_stub(planner, TARGETS)

    assert stub._start_probe(0.0, 0.0) == reachability.START_OK


def test_every_target_condemned_blocks_the_start():
    planner = FakePlanner({t: condemns(208) for t in TARGETS})
    stub = make_probe_stub(planner, TARGETS)

    assert stub._start_probe(0.0, 0.0) == reachability.START_BLOCKED
    assert planner.asked == TARGETS


def test_an_unanswered_target_is_inconclusive_not_blocked():
    # A silent planner is evidence about the stack. Reading it as a stuck robot
    # would halt a healthy run on the start-blocked cap.
    planner = FakePlanner({(0.5, 0.0): condemns(208), (0.0, 0.5): None})
    stub = make_probe_stub(planner, TARGETS)

    assert stub._start_probe(0.0, 0.0) == reachability.INCONCLUSIVE
    assert planner.asked == [(0.5, 0.0), (0.0, 0.5)]


def test_an_infrastructure_reply_is_inconclusive_not_blocked():
    planner = FakePlanner({(0.5, 0.0): (202, [], 0.01, 'tf')})    # TF_ERROR
    stub = make_probe_stub(planner, TARGETS)

    assert stub._start_probe(0.0, 0.0) == reachability.INCONCLUSIVE


def test_no_usable_target_is_inconclusive():
    # An all-unknown or wall-bound neighbourhood yields no target. That is not
    # evidence the robot is stuck.
    planner = FakePlanner()
    stub = make_probe_stub(planner, [])

    assert stub._start_probe(0.0, 0.0) == reachability.INCONCLUSIVE
    assert planner.asked == []


def test_the_cycle_deadline_stops_the_probe():
    planner = FakePlanner({t: condemns(208) for t in TARGETS})
    stub = make_probe_stub(planner, TARGETS)
    clock = FakeClock()

    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(fx, 'time', clock)
        # Deadline already past: not one round trip may be issued.
        assert stub._start_probe(0.0, 0.0, deadline=-1.0) == \
            reachability.INCONCLUSIVE
    assert planner.asked == []


# ── _free_targets_near ──────────────────────────────────────────────────────

def fake_grid(w, h, res=0.05, ox=-1.0, oy=-1.0, fill=0):
    """A minimal OccupancyGrid stand-in filled with one value."""
    return SimpleNamespace(
        info=SimpleNamespace(
            width=w, height=h, resolution=res,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=ox, y=oy),
                # The saver reads the yaw out of the origin quaternion, as
                # nav2's map_saver does; a real OccupancyGrid always carries one.
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[fill] * (w * h))


def make_grid_stub(grid):
    stub = SimpleNamespace(robot_id=0,
                           sensor=SimpleNamespace(get_map=lambda: (grid, 0)))
    stub._cell_has_clearance = FrontierExplorer._cell_has_clearance
    stub._free_targets_near = FrontierExplorer._free_targets_near.__get__(stub)
    return stub


def test_targets_come_from_the_map_and_sit_in_the_ring():
    stub = make_grid_stub(fake_grid(41, 41))            # 2.05 m of free space
    targets = stub._free_targets_near(0.0, 0.0)

    assert len(targets) == fx.START_PROBE_MAX
    lo = fx.START_PROBE_DIST - fx.START_PROBE_RING
    hi = fx.START_PROBE_DIST + fx.START_PROBE_RING
    for tx, ty in targets:
        assert lo <= math.hypot(tx, ty) <= hi, (tx, ty)


def test_targets_are_spread_around_the_robot():
    # Three targets bunched on one side would all fail for one goal-side reason
    # and report a wall as a stuck robot.
    stub = make_grid_stub(fake_grid(41, 41))
    angles = [math.atan2(ty, tx) for tx, ty in stub._free_targets_near(0.0, 0.0)]

    for i, a in enumerate(angles):
        for b in angles[i + 1:]:
            sep = abs(math.atan2(math.sin(a - b), math.cos(a - b)))
            assert sep >= math.pi / 4, angles


def test_no_map_yields_no_targets():
    stub = SimpleNamespace(sensor=SimpleNamespace(get_map=lambda: (None, 0)))
    stub._free_targets_near = FrontierExplorer._free_targets_near.__get__(stub)
    assert stub._free_targets_near(0.0, 0.0) == []


def test_an_occupied_or_unknown_neighbourhood_yields_no_targets():
    for fill in (100, -1):          # walled in / never mapped
        stub = make_grid_stub(fake_grid(41, 41, fill=fill))
        assert stub._free_targets_near(0.0, 0.0) == []


def test_a_grid_too_small_to_hold_the_ring_yields_no_targets():
    stub = make_grid_stub(fake_grid(5, 5))
    assert stub._free_targets_near(0.0, 0.0) == []


# ── the defer cap: the run must always end ──────────────────────────────────
#
# Binds the REAL explore() with everything around it stubbed out. Without a cap
# these tests hang rather than fail, which is the point: an uncapped DEFER loop
# is a silent hang, not a visible error.

class FakeClock:
    """Deterministic `time` for explore(); sleep advances instead of blocking."""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, 1e-9)


def make_loop_stub(selections, monkeypatch):
    """Bind the real explore() over a scripted sequence of _select_goal results.

    `selections` is consumed one per loop iteration; the last entry repeats, so
    a single-element list models "defers forever". An entry is either a bare
    selection, `(sel, definitive)`, or `(sel, definitive, truncated)` — the
    third field distinguishing the two ways a non-definitive cycle can arise
    (an unanswered probe vs. a probe budget that ran out), which explore() has
    to report differently.
    """
    monkeypatch.setattr(fx, 'time', FakeClock())
    monkeypatch.setattr(fx, 'rclpy', SimpleNamespace(ok=lambda: True))

    script = list(selections)
    stub = SimpleNamespace(
        robot_id=0,
        map_frame='robot_0/map',
        base_frame='robot_0/base_footprint',
        sensor=SimpleNamespace(
            robot_xy=lambda: (0.0, 0.0),
            get_map=lambda: (object(), 0),
            sim_time=lambda: 0.0),
        _precheck=True,
        _last_defer_definitive=True,
        _last_defer_truncated=False,
        _last_goal=None,
        _blacklist=[],
        _hb_last={},
        audits=[],
        logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    stub.error = lambda msg: stub.logged.append(('ERROR', msg))

    # Everything explore() leans on that isn't the defer logic under test.
    stub.waitUntilNav2Active = lambda localizer=None: None
    stub._wait_for_nav2_servers_active = lambda: True
    stub._wait_for_first_map = lambda: True
    stub._wait_for_fresh_map = lambda seq: True
    stub._sleep = lambda s: None
    stub._log_termination_audit = lambda goal_num: stub.audits.append(goal_num)
    # The real diagnostic, deliberately not stubbed: bound here, EVERY loop test
    # that reaches a terminal arm also proves it cannot crash the loop or move
    # the exit path — and after the dump was extended to all six arms, that is
    # every loop test in this file. This harness's get_map() hands back a bare
    # object(), which the dump must survive as a WARN — and does, before
    # touching the filesystem.
    stub._pose_occupancy_readout = \
        FrontierExplorer._pose_occupancy_readout.__get__(stub)
    stub._dump_termination_diagnostics = \
        FrontierExplorer._dump_termination_diagnostics.__get__(stub)

    def select(rx, ry, map_seq=0):
        entry = script.pop(0) if len(script) > 1 else script[0]
        if isinstance(entry, tuple):
            sel, definitive, truncated = (entry + (False,))[:3]
        else:
            sel, definitive, truncated = entry, True, False
        # A START_BLOCKED cycle makes no claim about the candidates: the real
        # _select_goal resets both flags to their identity values on entry and
        # its start-blocked returns never write them again, so explore() must not
        # read them on this path at all. The harness leaves whatever was there,
        # which is the stricter check — a test that passed only because the
        # harness happened to write a convenient value would be testing itself.
        if sel is not START_BLOCKED_SEL:
            stub._last_defer_definitive = definitive
            stub._last_defer_truncated = truncated
        return sel

    stub._select_goal = select
    stub.explore = FrontierExplorer.explore.__get__(stub)
    return stub


def test_an_endless_definitive_defer_streak_stops_and_audits(monkeypatch):
    # The planner answers every time and the answer is always "no". That is a
    # result: the run ends on its own terms (True) after the audit records the
    # frontiers left behind.
    stub = make_loop_stub([(DEFER, True)], monkeypatch)
    assert stub.explore() is True
    assert stub.audits == [0]
    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any(f'{fx.MAX_CONSEC_DEFERS} consecutive' in m
               and 'condemned every candidate' in m for m in warns), warns


def test_an_endless_unanswered_defer_streak_fails_the_run(monkeypatch):
    # Some candidate went unanswered every cycle, so "nothing is reachable" was
    # never established. Exiting zero here would let a slow or wedged
    # planner_server pass for a completed map.
    stub = make_loop_stub([(DEFER, False)], monkeypatch)
    assert stub.explore() is False
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any('never got a usable answer' in m for m in errors), errors
    assert any('do not read this as a completed map' in m for m in errors), errors
    # Still audited: the frontier accounting belongs in the log either way.
    assert stub.audits == [0]


def test_a_truncated_defer_streak_reports_the_budget_not_the_planner(monkeypatch):
    # Same non-definitive halt, different fault, so it must name a different
    # fix. Here the planner answered everything it was asked — selection just
    # stopped asking, on a bound, with candidates left over. Blaming
    # planner_server would send the next debugging session in exactly the wrong
    # direction, and calling it a completed map would be worse.
    stub = make_loop_stub([(DEFER, False, True)], monkeypatch)
    assert stub.explore() is False

    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any('ran out of probe budget' in m for m in errors), errors
    assert any(f'PRECHECK_MAX_PROBES (now {fx.PRECHECK_MAX_PROBES})' in m
               for m in errors), errors
    assert any('do not read this as a completed map' in m for m in errors), errors
    # The planner is not the suspect here and must not be named as one.
    assert not any('planner_server is not responding' in m for m in errors), errors
    assert stub.audits == [0]


def test_one_unanswered_cycle_taints_an_otherwise_definitive_streak(monkeypatch):
    # Conservative by design. A streak is only an ANSWER if every cycle in it
    # was; a single unanswered candidate anywhere means we never established it.
    script = [(DEFER, True)] * (fx.MAX_CONSEC_DEFERS - 1) + [(DEFER, False)]
    stub = make_loop_stub(script, monkeypatch)
    assert stub.explore() is False


def test_the_cap_is_not_reached_early(monkeypatch):
    # Exactly MAX_CONSEC_DEFERS-1 defers then a genuinely empty map: the run
    # must reach the normal completion path, not the cap.
    script = [(DEFER, True)] * (fx.MAX_CONSEC_DEFERS - 1) + [(None, True)]
    stub = make_loop_stub(script, monkeypatch)
    assert stub.explore() is True
    assert not [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert not [m for _l, m in stub.logged if 'consecutive' in m]


def test_a_dispatched_goal_resets_the_defer_streak(monkeypatch):
    # An intermittently-deferring but productive run must never hit the cap.
    # Nine defers, a real goal, nine more defers, repeat — forever, if the reset
    # works. The script ends in None so the test terminates on the normal path.
    near_miss = [(DEFER, True)] * (fx.MAX_CONSEC_DEFERS - 1)
    # Two DISTINCT goals: re-sending the same point trips the re-send guard,
    # which blacklists and skips the dispatch instead of performing one.
    goal_a = (((0.0, 1.0), (0.0, 1.0), 10), True)
    goal_b = (((0.0, 5.0), (0.0, 5.0), 50), True)
    script = (near_miss + [goal_a] + near_miss + [goal_b]
              + near_miss + [(None, True)])
    stub = make_loop_stub(script, monkeypatch)

    # Dispatch path stubs — only needed because this test actually sends goals.
    stub._make_goal = lambda x, y: object()
    stub.goToPose = lambda goal_msg: None
    stub._supervise_goal = lambda n, xy: ('SUCCEEDED', (1.0, 1.0), 0.0,
                                          (0, '', None))
    stub._classifier = fx.OutcomeClassifier(fx.BLACKLIST_RADIUS)

    assert stub.explore() is True
    assert not [m for lvl, m in stub.logged if lvl == 'ERROR']
    # Two goals dispatched, and the run ended via the normal completion path.
    assert stub.audits == [2]


def test_a_dispatched_goal_also_resets_the_truncation_flag(monkeypatch):
    # Truncation is streak-scoped, exactly like definitiveness. A cycle that ran
    # out of probe budget EARLY in a run says nothing about a later streak, and
    # the two faults prescribe opposite fixes: raise PRECHECK_MAX_PROBES for a
    # budget that bound, suspect planner_server for probes that went unanswered.
    # Measured in rung2f_on_explore.log, which exited telling the operator to
    # raise the probe cap while its final cycles were logging "probed 1 of 1
    # candidates (0 inconclusive, whole list examined)" — the budget was not
    # binding, and the message sent the next session the wrong way.
    goal = (((0.0, 1.0), (0.0, 1.0), 10), True)
    script = ([(DEFER, False, True)]      # one truncated cycle...
              + [goal]                    # ...ended by a real dispatch
              + [(DEFER, False, False)])  # a fresh UNANSWERED streak hits the cap
    stub = make_loop_stub(script, monkeypatch)

    stub._make_goal = lambda x, y: object()
    stub.goToPose = lambda goal_msg: None
    stub._supervise_goal = lambda n, xy: ('SUCCEEDED', (1.0, 1.0), 0.0,
                                          (0, '', None))
    stub._classifier = fx.OutcomeClassifier(fx.BLACKLIST_RADIUS)

    assert stub.explore() is False
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any('never got a usable answer' in m for m in errors), errors
    # The probe budget was not the fault in THIS streak and must not be named.
    assert not any('ran out of probe budget' in m for m in errors), errors
    assert stub.audits == [1]


# ── the start-blocked cap: a stuck robot is not a defer ─────────────────────

def test_an_endless_start_blocked_streak_stops_on_its_own_cap(monkeypatch):
    # A stuck robot must end the run too — recovery is not implemented, so past
    # some point the loop is only waiting for something it cannot cause. But it
    # must end on ITS cap with ITS message: the defer messages would send the
    # operator to raise PRECHECK_MAX_PROBES or suspect planner_server, when the
    # planner was answering correctly the whole time.
    stub = make_loop_stub([START_BLOCKED_SEL], monkeypatch)
    assert stub.explore() is False

    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any(f'{fx.MAX_CONSEC_START_BLOCKED} consecutive' in m
               and 'STUCK ROBOT' in m for m in errors), errors
    # None of the defer vocabulary: this streak said nothing about candidates.
    assert not any('produced no dispatchable goal' in m for m in errors), errors
    assert not any('ran out of probe budget' in m for m in errors), errors
    assert stub.audits == [0]


def test_start_blocked_cycles_do_not_count_toward_the_defer_cap(monkeypatch):
    # The two counters are separate. MAX_CONSEC_DEFERS-1 defers followed by a
    # run of start-blocked cycles must reach neither cap — if start-blocked
    # cycles counted as defers, the very first one would trip the defer cap and
    # the run would report "the planner condemned every candidate" about cycles
    # in which the planner was never usefully asked.
    script = ([(DEFER, True)] * (fx.MAX_CONSEC_DEFERS - 1)
              + [START_BLOCKED_SEL] * 5
              + [(None, True)])
    stub = make_loop_stub(script, monkeypatch)

    assert stub.explore() is True
    assert not [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert not [m for _l, m in stub.logged if 'consecutive' in m]
    assert stub.audits == [0]


def test_a_start_blocked_cycle_breaks_the_defer_streak(monkeypatch):
    # A defer streak claims selection productivity failed N cycles RUNNING. A
    # stuck robot interrupting it is a different fault, not another instance of
    # the same one, so the next defer starts a new streak. Without the reset,
    # MAX_CONSEC_DEFERS-1 defers + any number of start-blocked cycles + one more
    # defer trips the defer cap on cycles that were never contiguous.
    script = ([(DEFER, True)] * (fx.MAX_CONSEC_DEFERS - 1)
              + [START_BLOCKED_SEL]
              + [(DEFER, True)]
              + [(None, True)])
    stub = make_loop_stub(script, monkeypatch)

    assert stub.explore() is True
    assert not [m for lvl, m in stub.logged if lvl == 'ERROR']
    # The cap message is the tell: reaching it at all means the streak survived
    # the interruption. (The WORLD arm also returns True, so the return value
    # alone would not catch this.)
    assert not [m for _l, m in stub.logged if 'consecutive' in m]
    assert stub.audits == [0]


def test_a_new_defer_streak_does_not_inherit_the_old_streaks_labels(monkeypatch):
    # The counter is not the only per-streak state. defer_definitive is
    # AND-folded and defer_truncated OR-folded, so both are monotone: a label
    # carried across an interruption can never recover, and the run reports the
    # OLD streak's fault as the new one's. Here the first streak truncated and
    # went unanswered; the second is definitive throughout and must end on the
    # WORLD arm — no ERROR, and specifically not the probe-budget one.
    script = ([(DEFER, False, True)] * (fx.MAX_CONSEC_DEFERS - 1)
              + [START_BLOCKED_SEL]
              + [(DEFER, True)])
    stub = make_loop_stub(script, monkeypatch)

    assert stub.explore() is True
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert not errors, errors
    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any('condemned every candidate' in m for m in warns), warns
    assert stub.audits == [0]


def test_a_dispatched_goal_resets_the_start_blocked_streak(monkeypatch):
    # Same reset discipline as the defer streak: a dispatch proves the robot
    # could plan out of where it was standing, so the streak is broken.
    near_miss = [START_BLOCKED_SEL] * (fx.MAX_CONSEC_START_BLOCKED - 1)
    goal_a = (((0.0, 1.0), (0.0, 1.0), 10), True)
    goal_b = (((0.0, 5.0), (0.0, 5.0), 50), True)
    script = (near_miss + [goal_a] + near_miss + [goal_b]
              + near_miss + [(None, True)])
    stub = make_loop_stub(script, monkeypatch)

    stub._make_goal = lambda x, y: object()
    stub.goToPose = lambda goal_msg: None
    stub._supervise_goal = lambda n, xy: ('SUCCEEDED', (1.0, 1.0), 0.0,
                                          (0, '', None))
    stub._classifier = fx.OutcomeClassifier(fx.BLACKLIST_RADIUS)

    assert stub.explore() is True
    assert not [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert stub.audits == [2]


# ── the termination dump: making the next wedge decidable ──────────────────
#
# rung2f ended wedged at (-1.09, 4.07), the planner refusing a 0.70 m path with
# poses=0, and saved no map. Two explanations survived it — a real pocket, or a
# believed pose sitting inside a mapped wall — and they prescribe opposite
# fixes, one of which (backing up) is dangerous under the other.
#
# It ended on the DEFER cap, and start-pose probing did not exist yet, so
# hanging the dump on the start-blocked cap alone would not have caught the run
# that motivated it. Every arm that ends the run takes one, and each names its
# own arm: a file labelled for the wrong cap is worse than no file, because the
# exit reason is itself evidence about what the planner was doing.
#
# It stays diagnostic throughout: it may never crash the node, never move an
# exit path, and never RAISE the severity of the arm it hangs on — two of those
# arms end a run in which nothing went wrong.

def dump_stub(monkeypatch, grid, script=None, seq=7, pose=(-1.09, 4.07)):
    """A loop stub that runs to a terminal arm over a real grid."""
    stub = make_loop_stub(script if script is not None else [START_BLOCKED_SEL],
                          monkeypatch)
    stub.sensor = SimpleNamespace(robot_xy=lambda: pose,
                                  get_map=lambda: (grid, seq),
                                  sim_time=lambda: 0.0)
    return stub


# (script, slug, level the arm itself reports at, explore()'s return value)
TERMINAL_ARMS = [
    ([(None, True)], 'no-frontiers', 'INFO', True),
    ([(DEFER, True)], 'defer-world', 'WARN', True),
    ([(DEFER, False, True)], 'defer-truncated', 'ERROR', False),
    ([(DEFER, False, False)], 'defer-unanswered', 'ERROR', False),
    ([START_BLOCKED_SEL], 'startblocked', 'ERROR', False),
]


@pytest.mark.parametrize('script,slug,level,returns', TERMINAL_ARMS,
                         ids=[a[1] for a in TERMINAL_ARMS])
def test_every_terminal_arm_saves_a_map_named_for_that_arm(
        script, slug, level, returns, tmp_path, monkeypatch):
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    stub = dump_stub(monkeypatch, fake_grid(40, 40, ox=-2.0, oy=3.0),
                     script=script)

    assert stub.explore() is returns

    pgms = list(tmp_path.glob(f'*_{slug}.pgm'))
    yamls = list(tmp_path.glob(f'*_{slug}.yaml'))
    assert len(pgms) == 1 and len(yamls) == 1, (slug, list(tmp_path.iterdir()))

    readout = [m for lvl, m in stub.logged
               if lvl == level and f'termination diagnostic ({slug})' in m]
    assert len(readout) == 1, stub.logged
    assert '(-1.090, 4.070)' in readout[0]      # the pose it ended on
    assert 'map seq 7' in readout[0]            # which map version said so
    assert pgms[0].name in readout[0]           # and where to find it
    # The exit path is untouched: same audit, and no arm's diagnostic is louder
    # than the arm itself — a clean completion must still log no ERROR.
    assert stub.audits == [0]
    if level != 'ERROR':
        assert not [m for lvl, m in stub.logged if lvl == 'ERROR'], stub.logged


def test_the_stack_health_stop_saves_a_map_too(tmp_path, monkeypatch):
    # The one terminal exit that never reaches _log_termination_audit, and so
    # the one this could most easily have missed. A stack failing across
    # distinct frontiers is exactly the state a wedged pose hides inside.
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    goal = (((0.0, 1.0), (0.0, 1.0), 10), True)
    stub = dump_stub(monkeypatch, fake_grid(40, 40, ox=-2.0, oy=3.0),
                     script=[goal])
    stub._make_goal = lambda x, y: object()
    stub.goToPose = lambda goal_msg: None
    stub._supervise_goal = lambda n, xy: ('FAILED', (1.0, 1.0), 0.0,
                                          (0, '', None))
    stub._classifier = SimpleNamespace(
        classify=lambda outcome, xy, moved: SimpleNamespace(
            blacklist=False, stop_unhealthy=True, category='STACK',
            reason='5 stack failures across distinct frontiers'),
        is_retry_pending=lambda xy: False)

    assert stub.explore() is False

    pgms = list(tmp_path.glob('*_stack-unhealthy.pgm'))
    assert len(pgms) == 1, list(tmp_path.iterdir())
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any('termination diagnostic (stack-unhealthy)' in m
               for m in errors), errors
    # No audit on this path, before or after — the dump did not add one.
    assert stub.audits == []


def test_the_readout_names_the_value_of_the_cell_under_the_robot(tmp_path,
                                                                 monkeypatch):
    # The load-bearing claim. If the robot's own cell is occupied in the grid
    # the run just saved, the believed pose is inside a mapped wall and a
    # standoff or back-up recovery is the WRONG fix — so the value has to be in
    # the log, not left to be inferred from the image months later.
    grid = fake_grid(40, 40, ox=-2.0, oy=3.0)   # free everywhere, res 0.05
    grid.data[21 * 40 + 18] = 100               # (-1.09, 4.07) -> cell (18, 21)
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    stub = dump_stub(monkeypatch, grid)

    assert stub.explore() is False

    readout = [m for lvl, m in stub.logged
               if lvl == 'ERROR' and 'termination diagnostic' in m][0]
    assert 'cell (18, 21) value=100' in readout
    assert '3x3 north-row-first [0,0,0] [0,100,0] [0,0,0]' in readout
    assert 'nearest occupied 0.016 m away' in readout


def test_the_saved_map_matches_the_map_saver_format(tmp_path, monkeypatch):
    # The dump is only useful if it opens next to the other runs in
    # experiments/maps/, all of which experiments/slam/save_map.py wrote.
    grid = fake_grid(4, 3, ox=-2.0, oy=3.0, fill=-1)   # unknown everywhere
    grid.data[0] = 0        # bottom-left free
    grid.data[11] = 100     # top-right occupied
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    stub = dump_stub(monkeypatch, grid)

    assert stub.explore() is False

    raw = list(tmp_path.glob('*.pgm'))[0].read_bytes()
    magic, dims, maxval, body = raw.split(b'\n', 3)
    assert (magic, dims, maxval) == (b'P5', b'4 3', b'255')
    assert len(body) == 12
    # Rows are flipped: the TOP row (max y) is written first.
    assert body[3] == 0          # data[11], occupied -> black, first row
    assert body[8] == 254        # data[0], free -> white, last row
    assert set(body) == {0, 205, 254}

    text = list(tmp_path.glob('*.yaml'))[0].read_text()
    assert text.splitlines()[0].endswith('_startblocked.pgm')
    assert 'mode: trinary' in text
    assert 'origin: [-2.0, 3.0, 0.0]' in text
    assert 'occupied_thresh: 0.65' in text and 'free_thresh: 0.25' in text
    assert 'negate: 0' in text
    # A pose the grid does not even cover is a finding, not a crash.
    assert any('value=off-grid' in m for lvl, m in stub.logged if lvl == 'ERROR')


def test_a_missing_map_warns_and_leaves_the_exit_path_alone(tmp_path, monkeypatch):
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    stub = dump_stub(monkeypatch, None)

    assert stub.explore() is False

    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any('no map has ever been received' in m for m in warns), warns
    assert not list(tmp_path.iterdir())
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    assert any('STUCK ROBOT' in m for m in errors), errors
    assert not any('pose (' in m for m in errors), errors   # no readout to give
    assert stub.audits == [0]


def test_an_unwritable_directory_warns_and_still_logs_the_readout(tmp_path,
                                                                 monkeypatch):
    # A FILE where the dump wants a directory: fails for uid 0 too, unlike a
    # chmod, so the test means the same thing in a container as on the laptop.
    blocked = tmp_path / 'not-a-directory'
    blocked.write_text('')
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(blocked / 'maps'))
    stub = dump_stub(monkeypatch, fake_grid(40, 40, ox=-2.0, oy=3.0))

    assert stub.explore() is False

    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any('could not write the map' in m for m in warns), warns
    errors = [m for lvl, m in stub.logged if lvl == 'ERROR']
    # The file is expendable; the readout is not.
    assert any('map NOT saved' in m and 'value=' in m for m in errors), errors
    assert any('STUCK ROBOT' in m for m in errors), errors
    assert stub.audits == [0]


# ── the post-goal re-query: the same misattribution, worse consequence ──────
#
# Selection retires a frontier for RETIRE_TTL_MAPS maps. _terminal_outcome maps
# the same verdict to OutcomeClassifier.UNREACHABLE_NO_PATH, which is WORLD
# evidence and blacklists PERMANENTLY — no TTL, no way back. So a robot that
# wedges AFTER dispatch destroys frontiers where one that wedges before
# selection merely parks them. Every test here asserts the DOWNSTREAM effect
# through a real OutcomeClassifier, because the outcome string is only
# interesting for what the classifier then does with it.

def make_terminal_stub(result_code, result_name='FAILED', replies=None,
                       pose=(0.0, 0.0), start_probe=None, precheck=True):
    """Bind the real _terminal_outcome over a scripted Nav2 result.

    `result_code` is the aggregate error_code on the NavigateToPose result —
    the number bt_navigator actually published. `replies` scripts the planner
    for the re-query; `start_probe` is the verdict the start probe returns.
    """
    planner = FakePlanner(replies or {})
    stub = SimpleNamespace(
        robot_id=0,
        map_frame='robot_0/map',
        base_frame='robot_0/base_footprint',
        sensor=SimpleNamespace(plan_to=planner.plan_to, robot_xy=lambda: pose),
        _precheck=precheck,
        start_probes=[],
        planner=planner,
        logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    verdict = reachability.START_OK if start_probe is None else start_probe
    stub._start_probe = lambda rx, ry, deadline=None: (
        stub.start_probes.append((rx, ry)) or verdict)
    stub.getResult = lambda: SimpleNamespace(name=result_name)
    stub.result_future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            result=SimpleNamespace(error_code=result_code, error_msg='')))
    stub._start_at_fault = FrontierExplorer._start_at_fault.__get__(stub)
    stub._terminal_outcome = FrontierExplorer._terminal_outcome.__get__(stub)
    return stub


def blacklists(outcome, frontier=(0.0, 1.0)):
    """What a REAL OutcomeClassifier does with `outcome` — the thing that bites."""
    classifier = fx.OutcomeClassifier(fx.BLACKLIST_RADIUS)
    return classifier.classify(outcome, frontier, moved=False).blacklist


# --- the aggregate-code exit (no re-query runs) ------------------------------

def test_a_wedged_robot_does_not_permanently_blacklist_on_a_bare_208():
    # The likeliest wedged-after-dispatch path: with no path there is no
    # FollowPath failure to mask the planner, so the 208 arrives unmasked and
    # the 1xx-gated re-query never runs at all.
    stub = make_terminal_stub(208, start_probe=reachability.START_BLOCKED)
    outcome, code, _msg, recheck = stub._terminal_outcome((0.0, 1.0))

    assert outcome == 'FAILED'
    assert blacklists(outcome) is False
    assert code == 208
    assert recheck is None          # no re-query on this exit
    assert stub.start_probes == [(0.0, 0.0)]
    warns = [m for lvl, m in stub.logged if lvl == 'WARN']
    assert any('START is at fault' in m for m in warns), warns


def test_a_healthy_robot_still_condemns_the_goal_on_a_bare_208():
    # REGRESSION PIN (passes with or without the fix, by design): a cleared
    # start leaves the planner's verdict standing, permanent blacklist included.
    stub = make_terminal_stub(208)                      # start probe -> START_OK
    outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

    assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
    assert blacklists(outcome) is True


def test_goal_side_aggregate_codes_never_probe_the_start():
    # GUARD (trivially true when reverted): 204/206 name the goal.
    for code in (204, 206):
        stub = make_terminal_stub(code, start_probe=reachability.START_BLOCKED)
        outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

        assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
        assert blacklists(outcome) is True
        assert stub.start_probes == []


def test_the_precheck_off_path_issues_no_start_probe():
    # The A/B baseline: with the pre-check OFF nothing queries the planner, and
    # this exit is otherwise not gated on it.
    stub = make_terminal_stub(208, precheck=False,
                              start_probe=reachability.START_BLOCKED)
    outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

    assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
    assert stub.start_probes == []


# --- the re-query exit (a 1xx masked the planner) ----------------------------

def test_a_wedged_robot_does_not_permanently_blacklist_after_a_masked_208():
    stub = make_terminal_stub(102, replies={(0.0, 1.0): condemns(208)},
                              start_probe=reachability.START_BLOCKED)
    outcome, code, _msg, recheck = stub._terminal_outcome((0.0, 1.0))

    assert outcome == 'FAILED'
    assert blacklists(outcome) is False
    # The END line must tell the two readings apart on outcome= alone, with the
    # same planner_recheck= on both.
    assert (code, recheck) == (102, 208)
    assert stub.start_probes == [(0.0, 0.0)]


def test_a_healthy_robot_still_unmasks_the_planner_abort():
    # REGRESSION PIN (passes either way): this is 9824e3a's whole purpose and
    # must survive the start check.
    stub = make_terminal_stub(102, replies={(0.0, 1.0): condemns(208)})
    outcome, code, _msg, recheck = stub._terminal_outcome((0.0, 1.0))

    assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
    assert blacklists(outcome) is True
    assert (code, recheck) == (102, 208)


def test_a_start_side_requery_code_is_reported_and_keeps_the_frontier():
    # 203/205 on the re-query. triage() calls these INCONCLUSIVE, so the
    # frontier already survived — what was missing is the run SAYING the start
    # was at fault. No probe: the code is the diagnosis.
    for rcode in (203, 205):
        stub = make_terminal_stub(102,
                                  replies={(0.0, 1.0): condemns(rcode)},
                                  start_probe=reachability.START_BLOCKED)
        outcome, _c, _m, recheck = stub._terminal_outcome((0.0, 1.0))

        assert outcome == 'FAILED'
        assert blacklists(outcome) is False
        assert recheck == rcode
        assert stub.start_probes == []
        warns = [m for lvl, m in stub.logged if lvl == 'WARN']
        assert any('START is at fault' in m for m in warns), warns


def test_goal_side_requery_codes_never_probe_the_start():
    # GUARD (trivially true when reverted).
    for rcode in (204, 206):
        stub = make_terminal_stub(102, replies={(0.0, 1.0): condemns(rcode)},
                                  start_probe=reachability.START_BLOCKED)
        outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

        assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
        assert blacklists(outcome) is True
        assert stub.start_probes == []


def test_an_unreadable_pose_never_guesses_one():
    # GUARD (trivially true when reverted): no TF, no probe, and the planner's
    # verdict stands rather than being overridden on a guessed origin.
    stub = make_terminal_stub(102, replies={(0.0, 1.0): condemns(208)},
                              pose=None, start_probe=reachability.START_BLOCKED)
    outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

    assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
    assert stub.start_probes == []
    infos = [m for lvl, m in stub.logged if lvl == 'INFO']
    assert any('no pose to probe from' in m for m in infos), infos


def test_the_start_is_probed_at_most_once_per_terminal_outcome():
    # Both exits can reach the same question; only one of them runs per call.
    stub = make_terminal_stub(208, replies={(0.0, 1.0): condemns(208)},
                              start_probe=reachability.START_BLOCKED)
    stub._terminal_outcome((0.0, 1.0))
    assert len(stub.start_probes) == 1


def test_an_inconclusive_start_probe_leaves_the_planners_verdict_standing():
    # Anything short of START_BLOCKED is not evidence the robot is stuck, and
    # overriding a planner verdict on a non-answer would be the mirror of the
    # bug being fixed.
    stub = make_terminal_stub(208, start_probe=reachability.INCONCLUSIVE)
    outcome, _c, _m, _r = stub._terminal_outcome((0.0, 1.0))

    assert outcome == fx.OutcomeClassifier.UNREACHABLE_NO_PATH
    assert blacklists(outcome) is True
