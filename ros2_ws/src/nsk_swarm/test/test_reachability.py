"""Layer 1 — the reachability pre-check's decision logic.

Pure Python: nsk_swarm.reachability imports nothing from rclpy or nav2_msgs, so
unlike the frontier_explorer tests these run in any interpreter with no ROS
environment and no skip guard.

Two things are load-bearing here and neither is obvious from the code:

  * A planner SUCCESS is not proof the frontier is reachable. NavfnPlanner runs
    with `tolerance: 0.5` (nav2_robot0.yaml), so it can return error_code 0
    having planned to a cell half a metre from the point we asked about. Taking
    that at face value selects a candidate whose goal Nav2 then "reaches"
    without the robot ever visiting the frontier. The endpoint check is what
    makes a success mean what it appears to mean.

  * Only UNREACHABLE may retire a frontier. A timeout, a TF error or a silent
    planner is evidence about the STACK, and retiring on it would blacklist
    perfectly good goals whenever Nav2 hiccuped — the failure mode
    OutcomeClassifier already exists to prevent, reintroduced one layer up.
"""

from nsk_swarm.reachability import (ENDPOINT_TOL, INCONCLUSIVE, REACHABLE,
                                    UNREACHABLE, UNREACHABLE_CODES,
                                    RetirementLedger, endpoint_within, triage)

# A goal point and a path that genuinely arrives at it.
GOAL = (2.0, 3.0)
PATH_TO_GOAL = [(0.0, 0.0), (1.0, 1.5), (2.0, 3.0)]


# ── endpoint verification ───────────────────────────────────────────────────
def test_endpoint_within_accepts_a_path_that_arrives():
    assert endpoint_within(PATH_TO_GOAL, GOAL) is True


def test_endpoint_within_accepts_exactly_at_the_tolerance():
    # The boundary is inclusive: <= tol, not < tol. Pinned so a later tightening
    # is a deliberate edit rather than an accident of a comparison operator.
    path = [(0.0, 0.0), (2.0 + ENDPOINT_TOL, 3.0)]
    assert endpoint_within(path, GOAL) is True


def test_endpoint_within_rejects_just_beyond_the_tolerance():
    path = [(0.0, 0.0), (2.0 + ENDPOINT_TOL + 1e-6, 3.0)]
    assert endpoint_within(path, GOAL) is False


def test_endpoint_within_rejects_an_empty_or_missing_path():
    # error_code 0 with no poses is a degenerate success, not a usable plan:
    # there is no endpoint to verify, so it cannot be evidence of anything.
    assert endpoint_within([], GOAL) is False
    assert endpoint_within(None, GOAL) is False


# ── triage ──────────────────────────────────────────────────────────────────
def test_success_reaching_the_point_is_reachable():
    assert triage(0, PATH_TO_GOAL, GOAL) == REACHABLE


def test_success_stopping_short_of_the_point_is_rejected():
    # THE tolerance-0.5 case, and the reason endpoint verification is mandatory.
    # The planner is configured with tolerance 0.5, so this is a real reply it
    # can produce: error_code 0, a fine path, ending 0.4 m from where we asked.
    # It must NOT be selected — the path proves something near the frontier is
    # plannable, not the frontier itself.
    path = [(0.0, 0.0), (1.0, 1.5), (2.0, 2.6)]     # ends 0.4 m short
    assert abs((3.0 - 2.6) - 0.4) < 1e-9            # the gap really is 0.4 m
    verdict = triage(0, path, GOAL)
    assert verdict != REACHABLE
    # And equally it must not RETIRE the frontier: the planner said yes, so
    # there is no evidence against this point — only evidence it answered about
    # a different one.
    assert verdict == INCONCLUSIVE


def test_success_with_an_empty_path_is_inconclusive():
    assert triage(0, [], GOAL) == INCONCLUSIVE


def test_goal_condemning_codes_are_unreachable():
    # 204 GOAL_OUTSIDE_MAP, 206 GOAL_OCCUPIED, 208 NO_VALID_PATH — the planner
    # judged the GOAL, not the stack. The only verdict that may retire.
    for code in (204, 206, 208):
        assert code in UNREACHABLE_CODES
        assert triage(code, [], GOAL) == UNREACHABLE


def test_condemning_codes_win_even_if_a_path_came_back():
    # Defensive: a non-zero code is the planner's verdict regardless of whatever
    # stale path rode along with it.
    assert triage(208, PATH_TO_GOAL, GOAL) == UNREACHABLE


def test_infrastructure_codes_are_inconclusive_not_unreachable():
    # 202 TF_ERROR and 207 TIMEOUT say the stack failed, not that the world is
    # bad. 203/205 are about the ROBOT's pose — retiring the frontier for those
    # would blame the wrong thing entirely.
    for code in (200, 201, 202, 203, 205, 207):
        assert triage(code, [], GOAL) == INCONCLUSIVE


def test_no_answer_is_inconclusive():
    # plan_to returns None for a timeout, a rejected goal, or a server that is
    # not up. Silence is never evidence against a frontier.
    assert triage(None, None, GOAL) == INCONCLUSIVE


# ── retirement ledger ───────────────────────────────────────────────────────
QUANTUM = 0.4   # == BLACKLIST_RADIUS, what the explorer passes
TTL = 10        # == RETIRE_TTL_MAPS


def make_ledger():
    return RetirementLedger(QUANTUM, TTL)


def test_a_retired_frontier_is_retired_on_the_next_map():
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=100)
    assert led.is_retired((1.0, 1.0), 100) is True
    assert led.is_retired((1.0, 1.0), 101) is True
    assert led.is_retired((1.0, 1.0), 100 + TTL - 1) is True


def test_retirement_expires_once_ttl_maps_have_arrived():
    # The whole point of the TTL: a frontier walled off by unknown space now can
    # become plannable once SLAM fills in the corridor to it. A permanent
    # blacklist would throw away real exploration targets.
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=100)
    assert led.is_retired((1.0, 1.0), 100 + TTL) is False
    assert led.is_retired((1.0, 1.0), 100 + TTL + 5) is False


def test_retire_returns_the_expiry_sequence():
    led = make_ledger()
    assert led.retire((1.0, 1.0), map_seq=42) == 42 + TTL


def test_nearby_points_share_one_entry():
    # Frontier centroids drift a few cm between SLAM scans. Without the
    # quantisation a retired frontier would reappear as a "new" candidate on the
    # very next map and be probed again immediately.
    # (0.8 == 2*QUANTUM sits at a bucket CENTRE, so a small drift stays inside
    # it — see test_points_across_a_bucket_boundary_do_not_share for the
    # boundary case this deliberately avoids.)
    led = make_ledger()
    led.retire((0.8, 0.8), map_seq=0)
    assert led.is_retired((0.8 + QUANTUM / 4, 0.8 - QUANTUM / 4), 1) is True


def test_points_across_a_bucket_boundary_do_not_share():
    # Documented limitation, not a defect: keys are round(x/quantum) buckets, so
    # two points closer than one quantum still split if a boundary runs between
    # them (1.0/0.4 == 2.5 is exactly such a boundary). Inherited deliberately
    # from OutcomeClassifier._key so both use one notion of "the same frontier";
    # the TTL bounds the cost of an occasional missed match to one extra probe.
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=0)
    assert led.is_retired((1.0 + QUANTUM / 4, 1.0 - QUANTUM / 4), 1) is False


def test_distinct_points_do_not_share_an_entry():
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=0)
    assert led.is_retired((5.0, 5.0), 1) is False


def test_re_retiring_extends_from_the_current_map():
    # The planner condemned it again on fresher evidence, so the clock restarts.
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=0)
    led.retire((1.0, 1.0), map_seq=8)
    assert led.is_retired((1.0, 1.0), TTL) is True      # would have expired at TTL
    assert led.is_retired((1.0, 1.0), 8 + TTL) is False


def test_active_counts_live_entries_and_purges_expired_ones():
    led = make_ledger()
    led.retire((1.0, 1.0), map_seq=0)
    led.retire((5.0, 5.0), map_seq=0)
    led.retire((9.0, 9.0), map_seq=50)
    assert led.active(1) == 3
    # The first two expire at TTL; the third is still live.
    assert led.active(TTL) == 1
    assert led.active(50 + TTL) == 0


def test_unretired_frontier_reads_as_not_retired():
    assert make_ledger().is_retired((0.0, 0.0), 0) is False
