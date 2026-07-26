"""Layer 1 — RTF-invariant GoalSupervisor (pure Python, no ROS).

GoalSupervisor lives in its own module with zero rclpy/ROS imports, so these
tests drive it directly with synthetic (sim_time, x, y) sample streams — no
node, no executor, no rclpy.init, and (crucially) no wall clock. Every judgement
is made in SIMULATION time, which is exactly what makes it invariant to the
real-time factor; the sequences below advance sim_time by hand to prove it.
"""

from nsk_swarm.goal_supervisor import GoalSupervisor


# Local echoes of the constants under test (a drift here vs. the class is a
# real regression, so assert against literals rather than the class attrs).
PROGRESS_WINDOW = 20.0
PROGRESS_MIN_MOVE = 0.25
GOAL_SIM_TIMEOUT = 180.0


def test_constants_are_the_documented_values():
    assert GoalSupervisor.PROGRESS_WINDOW == PROGRESS_WINDOW
    assert GoalSupervisor.PROGRESS_MIN_MOVE == PROGRESS_MIN_MOVE
    assert GoalSupervisor.GOAL_SIM_TIMEOUT == GOAL_SIM_TIMEOUT


def test_empty_supervisor_is_running():
    # No samples yet (goal just dispatched) — nothing to judge.
    assert GoalSupervisor().verdict() == GoalSupervisor.RUNNING


# ── moving robot -> RUNNING ──────────────────────────────────────────────────

def test_moving_robot_stays_running():
    # Steady 0.1 m/s straight line for 60 sim s: each 20 s window covers ~2 m,
    # far past PROGRESS_MIN_MOVE, and well under the 180 s budget.
    sup = GoalSupervisor()
    t = 0.0
    while t <= 60.0:
        sup.ingest(t, 0.1 * t, 0.0)
        assert sup.verdict() == GoalSupervisor.RUNNING
        t += 0.5


# ── stalled robot -> FUTILE_NO_PROGRESS only after the window fills ───────────

def test_stalled_robot_is_running_until_window_fills_then_futile():
    # Robot pinned at a fixed pose. Before the deque spans PROGRESS_WINDOW the
    # verdict must stay RUNNING (this span IS the planning grace period); the
    # instant the window is fully spanned it flips to FUTILE_NO_PROGRESS.
    sup = GoalSupervisor()

    # 0 .. 19.5 sim s: window not yet spanned -> grace period, RUNNING.
    t = 0.0
    while t < PROGRESS_WINDOW:
        sup.ingest(t, 5.0, 5.0)
        assert sup.verdict() == GoalSupervisor.RUNNING, f'premature verdict at t={t}'
        t += 0.5

    # A sample at exactly t == PROGRESS_WINDOW makes the span reach the window
    # (oldest sample is t=0.0); no motion the whole time -> FUTILE_NO_PROGRESS.
    sup.ingest(PROGRESS_WINDOW, 5.0, 5.0)
    assert sup.verdict() == GoalSupervisor.FUTILE_NO_PROGRESS


def test_jitter_below_threshold_is_no_progress():
    # Tiny back-and-forth jitter (max ~0.1 m from the window start, under the
    # 0.25 m threshold) across a full window still counts as no progress.
    sup = GoalSupervisor()
    t = 0.0
    offsets = (0.0, 0.05, -0.05, 0.1, -0.1)
    i = 0
    while t <= PROGRESS_WINDOW:
        dx = offsets[i % len(offsets)]
        sup.ingest(t, 1.0 + dx, 1.0)
        t += 1.0
        i += 1
    assert sup.verdict() == GoalSupervisor.FUTILE_NO_PROGRESS


def test_recovered_progress_clears_no_progress():
    # A robot that sat still long enough to span the window but THEN starts
    # moving must return to RUNNING once the stale still-samples age out of the
    # trailing window.
    sup = GoalSupervisor()
    t = 0.0
    while t <= PROGRESS_WINDOW:          # fill the window while stalled
        sup.ingest(t, 0.0, 0.0)
        t += 1.0
    assert sup.verdict() == GoalSupervisor.FUTILE_NO_PROGRESS

    # Now drive steadily for another full window; the old still-samples fall
    # out and the window shows real movement.
    x = 0.0
    while t <= 2 * PROGRESS_WINDOW + 2.0:
        x += 0.1                          # 0.1 m per step
        sup.ingest(t, x, 0.0)
        t += 1.0
    assert sup.verdict() == GoalSupervisor.RUNNING


# ── slow-but-moving robot -> RUNNING past the old 120 s wall budget ──────────

def test_slow_but_moving_robot_survives_past_old_wall_budget():
    # 0.02 m/s: each 20 s window covers ~0.4 m (> 0.25 m), so it never trips
    # no-progress. Run to 150 sim s — well past the retired 120 s WALL budget
    # that used to kill slow-RTF goals — and under the 180 s sim budget.
    sup = GoalSupervisor()
    t = 0.0
    while t <= 150.0:
        sup.ingest(t, 0.02 * t, 0.0)
        assert sup.verdict() == GoalSupervisor.RUNNING, f'killed slow goal at t={t}'
        t += 1.0
    assert sup.elapsed() >= 120.0        # proves we ran past the old budget


# ── overlong goal -> FUTILE_SIM_TIMEOUT ──────────────────────────────────────

def test_overlong_goal_hits_sim_timeout_even_while_moving():
    # A goal that keeps moving (never no-progress) but runs the full sim budget
    # must still be declared futile via the sim timeout.
    sup = GoalSupervisor()
    t = 0.0
    while t < GOAL_SIM_TIMEOUT:
        sup.ingest(t, 0.1 * t, 0.0)      # always moving
        assert sup.verdict() == GoalSupervisor.RUNNING
        t += 1.0
    sup.ingest(GOAL_SIM_TIMEOUT, 0.1 * GOAL_SIM_TIMEOUT, 0.0)
    assert sup.verdict() == GoalSupervisor.FUTILE_SIM_TIMEOUT


def test_sim_timeout_takes_priority_over_no_progress():
    # Stalled AND overrun: both conditions hold at the budget boundary. The
    # sim-timeout verdict is checked first, so it wins.
    sup = GoalSupervisor()
    t = 0.0
    while t <= GOAL_SIM_TIMEOUT:
        sup.ingest(t, 3.0, 3.0)          # never moves
        t += 5.0
    assert sup.verdict() == GoalSupervisor.FUTILE_SIM_TIMEOUT


def test_timeout_uses_sim_time_not_sample_count():
    # Only three samples, but spanning the full budget in sim time — the
    # verdict is time-based, not count-based.
    sup = GoalSupervisor()
    sup.ingest(0.0, 0.0, 0.0)
    sup.ingest(90.0, 9.0, 0.0)
    sup.ingest(GOAL_SIM_TIMEOUT, 18.0, 0.0)
    assert sup.verdict() == GoalSupervisor.FUTILE_SIM_TIMEOUT


# ── window_move: the windowed displacement the no-progress verdict fires on ───

def test_window_move_is_the_trailing_window_excursion_not_the_whole_goal():
    # Robot drives 3 m over the first 15 s, then sits still for a full window.
    # window_move must report the trailing window's small excursion (< threshold,
    # which is what makes the verdict FUTILE), NOT the 3 m the goal covered net.
    sup = GoalSupervisor()
    t = 0.0
    while t <= 15.0:                       # drive out to x = 3.0
        sup.ingest(t, 0.2 * t, 0.0)
        t += 1.0
    while t <= 15.0 + PROGRESS_WINDOW:     # then stall at x = 3.0 for a window
        sup.ingest(t, 3.0, 0.0)
        t += 1.0

    assert sup.verdict() == GoalSupervisor.FUTILE_NO_PROGRESS
    # The trailing window saw ~no motion, far below the whole-goal 3.0 m net.
    assert sup.window_move() < PROGRESS_MIN_MOVE


def test_window_move_empty_is_zero():
    assert GoalSupervisor().window_move() == 0.0
