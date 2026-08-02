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

import os
from types import SimpleNamespace

import pytest

# Same guard, and the same reasoning, as test_frontier_explorer_gate.py:
# FrontierExplorer subclasses BasicNavigator, so the nav2_simple_commander
# import is module-level and cannot be deferred. CI sets NSK_REQUIRE_NAV2=1 and
# installs Nav2 deliberately, so a miss there is fatal rather than skipped.
try:
    from nsk_swarm import frontier_explorer as fx
    from nsk_swarm.frontier_explorer import DEFER, FrontierExplorer
except ImportError:
    if os.environ.get('NSK_REQUIRE_NAV2', '') not in ('', '0', 'false'):
        raise
    fx = FrontierExplorer = DEFER = None

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


def make_explorer_stub(centroids, planner, precheck=True, blacklist=None):
    """Bind the real selection methods to a stub with a scripted frontier set.

    `centroids` is what _frontier_centroids() would return: [(x, y, ncells)].
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
        logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    stub.error = lambda msg: stub.logged.append(('ERROR', msg))
    stub._frontier_centroids = lambda: list(centroids)
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

    def select(rx, ry, map_seq=0):
        entry = script.pop(0) if len(script) > 1 else script[0]
        if isinstance(entry, tuple):
            sel, definitive, truncated = (entry + (False,))[:3]
        else:
            sel, definitive, truncated = entry, True, False
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
