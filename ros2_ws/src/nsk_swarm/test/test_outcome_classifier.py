"""Layer 1 — WORLD-vs-STACK outcome classification (pure Python, no ROS).

OutcomeClassifier lives in its own module with zero rclpy/ROS imports, so these
tests drive the blacklist / stack-health policy directly with synthetic outcome
sequences — no node, no Nav2, no rclpy.init.

Core property under test (the bug that motivated it): a Nav2 FAILED is an
INFRASTRUCTURE hiccup, not evidence a frontier is unreachable, so it must not
poison a perfectly good frontier on the first occurrence.
"""

from nsk_swarm.goal_supervisor import FUTILE_NO_PROGRESS, FUTILE_SIM_TIMEOUT
from nsk_swarm.outcome_classifier import (CONSEC_STACK_FAILURES,
                                          MAX_FRONTIER_RETRIES,
                                          OutcomeClassifier)

# Frontier key quantum used in the explorer (== BLACKLIST_RADIUS).
QUANTUM = 0.4


def make_classifier():
    return OutcomeClassifier(QUANTUM)


# ── STACK failure: tolerate a few, blacklist only at the retry cap ───────────

def test_two_failures_then_success_does_not_blacklist():
    # FAILED x2 on one frontier, then SUCCEEDED: a stack hiccup that recovered.
    # The frontier must never be blacklisted.
    clf = make_classifier()
    frontier = (2.0, 3.0)

    d1 = clf.classify('FAILED', frontier)
    assert d1.blacklist is False and d1.category == 'STACK'
    d2 = clf.classify('FAILED', frontier)
    assert d2.blacklist is False and d2.category == 'STACK'

    d3 = clf.classify('SUCCEEDED', frontier)
    assert d3.blacklist is False and d3.category == 'SUCCESS'

    # And the streak was reset — a subsequent single failure is treated as the
    # first, not the third.
    assert clf.failure_count(frontier) == 0
    d4 = clf.classify('FAILED', frontier)
    assert d4.blacklist is False


def test_three_failures_same_frontier_blacklists():
    # FAILED x3 on the SAME frontier: now we give up on it (but this alone is
    # not enough distinct frontiers to indict the whole stack).
    clf = make_classifier()
    frontier = (-1.0, 5.0)

    assert clf.classify('FAILED', frontier).blacklist is False
    assert clf.classify('FAILED', frontier).blacklist is False
    d3 = clf.classify('FAILED', frontier)
    assert d3.blacklist is True
    assert d3.stop_unhealthy is False
    assert f'x{MAX_FRONTIER_RETRIES}' in d3.reason


def test_retry_counter_is_quantised_to_the_blacklist_radius():
    # Two centroids closer than one quantum collapse to the same key, so their
    # failures accumulate on one counter and trip the cap together.
    clf = make_classifier()
    a = (0.00, 0.00)
    b = (0.10, 0.05)   # < QUANTUM away -> same quantised key

    assert clf.classify('FAILED', a).blacklist is False
    assert clf.classify('FAILED', b).blacklist is False
    assert clf.classify('CANCELED', a).blacklist is True   # 3rd on the shared key


def test_canceled_counts_as_a_stack_failure():
    clf = make_classifier()
    f = (7.0, 7.0)
    assert clf.classify('CANCELED', f).category == 'STACK'
    assert clf.classify('CANCELED', f).blacklist is False
    assert clf.classify('CANCELED', f).blacklist is True


# ── WORLD evidence: blacklist on the first occurrence ────────────────────────

def test_futile_no_progress_blacklists_immediately():
    clf = make_classifier()
    d = clf.classify(FUTILE_NO_PROGRESS, (4.0, 4.0))
    assert d.blacklist is True
    assert d.stop_unhealthy is False
    assert d.category == 'WORLD'


def test_futile_sim_timeout_blacklists_immediately():
    clf = make_classifier()
    d = clf.classify(FUTILE_SIM_TIMEOUT, (4.0, 4.0))
    assert d.blacklist is True and d.category == 'WORLD'


def test_succeeded_without_motion_blacklists_but_is_not_a_failure():
    # "Reached" but the robot never moved: frontier inside goal tolerance —
    # WORLD evidence, blacklist it; but it still proves the stack works, so it
    # does not count toward any failure streak.
    clf = make_classifier()
    f = (1.0, 1.0)
    clf.classify('FAILED', f)                     # one prior stack hiccup
    d = clf.classify('SUCCEEDED', f, moved=False)
    assert d.blacklist is True and d.category == 'WORLD'
    assert clf.failure_count(f) == 0              # streak reset by the success


# ── STACK-WIDE guard: distinct-frontier failures indict the stack ────────────

def test_five_distinct_frontier_failures_trigger_stackwide_guard():
    clf = make_classifier()
    # Four distinct frontiers fail once each — no stop yet.
    for i in range(CONSEC_STACK_FAILURES - 1):
        d = clf.classify('FAILED', (10.0 * i, 0.0))
        assert d.stop_unhealthy is False, f'premature stop at frontier {i}'

    # The fifth distinct frontier failing trips the run-wide guard.
    d = clf.classify('FAILED', (10.0 * (CONSEC_STACK_FAILURES - 1), 0.0))
    assert d.stop_unhealthy is True
    assert d.category == 'STACK'
    assert 'unhealthy' in d.reason
    assert 'distinct frontiers' in d.reason


def test_same_frontier_retried_does_not_trigger_stackwide_guard():
    # Hammering ONE frontier past the retry cap blacklists it but must NOT
    # indict the whole stack — that path is per-frontier, not stack-wide.
    clf = make_classifier()
    f = (3.0, 3.0)
    for _ in range(CONSEC_STACK_FAILURES + 2):
        d = clf.classify('FAILED', f)
        assert d.stop_unhealthy is False


def test_success_between_failures_resets_the_stackwide_streak():
    # Distinct-frontier failures interrupted by a success never accumulate to
    # the guard threshold: the stack demonstrably works between hiccups.
    clf = make_classifier()
    for i in range(CONSEC_STACK_FAILURES - 1):
        assert clf.classify('FAILED', (10.0 * i, 0.0)).stop_unhealthy is False
    # A success anywhere clears the cross-frontier streak.
    clf.classify('SUCCEEDED', (999.0, 999.0))
    # Four more distinct failures still do not trip (streak restarted).
    for i in range(CONSEC_STACK_FAILURES - 1):
        d = clf.classify('FAILED', (0.0, 10.0 * i))
        assert d.stop_unhealthy is False


# ── resend-guard deferral: is_retry_pending is what the guard consults ────────
# The explorer's re-send guard blacklists a frontier that is re-selected within
# RESEND_RADIUS of the last-sent goal. It must DEFER to the classifier while the
# classifier is mid-retry on that frontier: an immediate re-selection of a spared
# frontier is the INTENDED retry, not a no-progress spin. The guard's whole
# decision reduces to classifier.is_retry_pending(frontier), tested here.

def test_retry_pending_frontier_is_not_blacklisted_by_the_guard():
    # One prior stack FAILED (retry 1/3) => classifier is holding the frontier
    # for another attempt, so an immediate re-selection is the intended retry:
    # is_retry_pending True => the guard DEFERS (does not blacklist).
    clf = make_classifier()
    f = (-1.18, 5.47)                       # the goal #25 frontier from the log
    d = clf.classify('FAILED', f)
    assert d.blacklist is False             # classifier spared it (retry 1/3)
    assert clf.is_retry_pending(f) is True  # ...so the guard must defer

    # Still pending on the second consecutive failure (retry 2/3).
    clf.classify('FAILED', f)
    assert clf.is_retry_pending(f) is True


def test_frontier_with_no_pending_retry_is_still_guarded():
    # A frontier the classifier has never failed on has nothing to retry:
    # is_retry_pending False => the guard keeps full authority and blacklists a
    # no-progress re-selection exactly as before.
    clf = make_classifier()
    assert clf.is_retry_pending((2.0, 3.0)) is False


def test_frontier_at_max_retries_is_blacklisted_by_classifier_not_deferred():
    # At MAX_FRONTIER_RETRIES the classifier itself blacklists the frontier
    # (as before), and is_retry_pending goes False — so the guard does NOT keep
    # deferring past the cap. The retry ladder is bounded, not infinite.
    clf = make_classifier()
    f = (3.3, -1.5)
    for _ in range(MAX_FRONTIER_RETRIES - 1):
        assert clf.classify('FAILED', f).blacklist is False
        assert clf.is_retry_pending(f) is True
    d_cap = clf.classify('FAILED', f)                 # the MAX-th failure
    assert d_cap.blacklist is True                    # classifier blacklists it
    assert clf.failure_count(f) == MAX_FRONTIER_RETRIES
    assert clf.is_retry_pending(f) is False           # no longer deferred
