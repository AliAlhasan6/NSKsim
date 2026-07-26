#!/usr/bin/env python3
"""outcome_classifier.py — separate WORLD evidence from STACK failure.

Pure Python, no rclpy: the frontier explorer feeds it each goal's terminal
outcome and asks whether to blacklist the frontier — and whether the Nav2 stack
itself looks unhealthy.

The distinction this class exists to make:

  * WORLD evidence — the frontier really is a bad place to send the robot. The
    supervisor watched the robot in sim time and it made no progress
    (FUTILE_NO_PROGRESS) or blew the sim budget (FUTILE_SIM_TIMEOUT); or Nav2
    reported the goal reached but the robot never actually moved (a frontier
    inside goal tolerance). Blacklist immediately — retrying learns nothing.

  * STACK failure — Nav2 could not run the goal for reasons that say nothing
    about the world: FAILED / CANCELED (classically the bt_navigator "Timed out
    while waiting for action server to acknowledge goal request for follow_path"
    infrastructure hiccup), WALL_GUARD, or an UNKNOWN result. Do NOT blacklist a
    perfectly reachable frontier on the first such failure; tolerate
    MAX_FRONTIER_RETRIES consecutive stack failures on the SAME frontier
    (quantised to the blacklist radius) before giving up on it. Any SUCCEEDED
    proves the stack works and resets that frontier's streak.

  * STACK-WIDE guard — if CONSEC_STACK_FAILURES distinct frontiers fail with no
    success between them, the problem is the stack, not the frontiers. Signal a
    hard stop with an explicit ERROR rather than silently blacklisting the whole
    map (the exact failure mode we are trying to make impossible).
"""

from collections import namedtuple

from nsk_swarm.goal_supervisor import FUTILE_NO_PROGRESS, FUTILE_SIM_TIMEOUT

# Outcome strings the explorer forwards. FUTILE_* come from GoalSupervisor; the
# rest are Nav2 TaskResult .name values plus the explorer's own 'WALL_GUARD'.
SUCCEEDED = 'SUCCEEDED'
_WORLD_OUTCOMES = frozenset({FUTILE_NO_PROGRESS, FUTILE_SIM_TIMEOUT})
_STACK_OUTCOMES = frozenset({'FAILED', 'CANCELED', 'UNKNOWN', 'WALL_GUARD'})

MAX_FRONTIER_RETRIES = 3    # consecutive STACK failures on one frontier before it is blacklisted
CONSEC_STACK_FAILURES = 5   # distinct frontiers failing (no success between) => stack unhealthy

# Category of a decision, for the caller's log level. WORLD/STACK/SUCCESS mirror
# the three cases above; UNRECOGNISED is a defensive catch-all.
Decision = namedtuple('Decision', ['blacklist', 'stop_unhealthy', 'category', 'reason'])


class OutcomeClassifier:
    """Blacklist / stack-health policy for goal outcomes (see module docstring).

    Stateful across a run: one instance per explorer. ``classify`` returns a
    ``Decision`` and mutates the internal counters; the caller acts on
    ``blacklist`` / ``stop_unhealthy`` and logs ``reason``.
    """

    MAX_FRONTIER_RETRIES = MAX_FRONTIER_RETRIES
    CONSEC_STACK_FAILURES = CONSEC_STACK_FAILURES

    def __init__(self, quantum: float):
        # quantum (m): frontier key resolution — pass BLACKLIST_RADIUS so two
        # centroids that would collapse under the blacklist share one counter.
        self._quantum = quantum
        self._fail_counts = {}                  # key -> consecutive stack-failure count
        self._failed_keys_since_success = set()  # distinct frontiers failed since last success

    def _key(self, frontier_xy):
        x, y = frontier_xy
        return (round(x / self._quantum), round(y / self._quantum))

    def failure_count(self, frontier_xy) -> int:
        """Current consecutive stack-failure count for a frontier (0 if none)."""
        return self._fail_counts.get(self._key(frontier_xy), 0)

    def is_retry_pending(self, frontier_xy) -> bool:
        """True while a frontier is mid-retry: it has taken at least one stack
        failure but not yet reached MAX_FRONTIER_RETRIES, so the classifier is
        still holding it for another attempt rather than blacklisting it.

        The explorer's re-send guard consults this to tell an INTENDED retry
        (re-selecting the very frontier the classifier just spared) from a
        genuine no-progress spin: at 0 there is nothing to retry, and at the cap
        the classifier has already blacklisted, so in both cases the guard keeps
        full authority.
        """
        return 0 < self.failure_count(frontier_xy) < self.MAX_FRONTIER_RETRIES

    def classify(self, outcome: str, frontier_xy, moved: bool = True) -> Decision:
        """Decide what a single goal ``outcome`` at ``frontier_xy`` means.

        ``moved`` distinguishes a productive SUCCEEDED from a "reached but the
        robot never moved" SUCCEEDED (frontier inside goal tolerance).
        """
        key = self._key(frontier_xy)

        # ── WORLD evidence: blacklist now, retrying learns nothing ───────────
        if outcome in _WORLD_OUTCOMES:
            return Decision(True, False, 'WORLD',
                            f'{outcome} — world evidence; blacklisting immediately')

        # ── SUCCESS: the stack works. Reset this frontier + the cross-frontier
        # streak. A no-motion success is still WORLD evidence the frontier is
        # unproductive, so blacklist it — but the resets stand (stack is fine). ─
        if outcome == SUCCEEDED:
            self._fail_counts.pop(key, None)
            self._failed_keys_since_success.clear()
            if not moved:
                return Decision(True, False, 'WORLD',
                                'SUCCEEDED but robot did not move — frontier inside '
                                'goal tolerance; blacklisting to break the spin')
            return Decision(False, False, 'SUCCESS', 'SUCCEEDED — productive')

        # ── STACK failure: infrastructure, not the world ─────────────────────
        if outcome in _STACK_OUTCOMES:
            n = self._fail_counts.get(key, 0) + 1
            self._fail_counts[key] = n
            self._failed_keys_since_success.add(key)

            distinct = len(self._failed_keys_since_success)
            if distinct >= self.CONSEC_STACK_FAILURES:
                return Decision(False, True, 'STACK',
                                f'Nav2 stack appears unhealthy — {distinct} consecutive '
                                f'failures across distinct frontiers')
            if n >= self.MAX_FRONTIER_RETRIES:
                return Decision(True, False, 'STACK',
                                f'{outcome} x{n} on this frontier — blacklisting after '
                                f'{self.MAX_FRONTIER_RETRIES} stack-failure retries')
            return Decision(False, False, 'STACK',
                            f'{outcome} x{n} on this frontier — infrastructure failure, '
                            f'not blacklisting (retry {n}/{self.MAX_FRONTIER_RETRIES})')

        # ── Unrecognised outcome: never blacklist on something we can't read ──
        return Decision(False, False, 'UNRECOGNISED',
                        f'unrecognised outcome {outcome!r} — not blacklisting')
