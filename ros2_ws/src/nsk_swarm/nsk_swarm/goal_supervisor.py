#!/usr/bin/env python3
"""goal_supervisor.py — RTF-invariant progress supervision for a Nav2 goal.

Pure Python, no rclpy: the frontier explorer feeds it (sim_time, x, y) samples
sourced from the robot's TF + sim clock, and asks for a verdict. Every judgement
is made in SIMULATION time, so the decision is invariant to the real-time factor
(RTF) — a goal is failed because the robot stopped making progress *in the sim*,
never because a slow machine burned real wall-clock seconds.

Two ways a goal is declared futile:
  * FUTILE_NO_PROGRESS — over the most recent PROGRESS_WINDOW sim seconds the
    robot never got farther than PROGRESS_MIN_MOVE from where it was at the
    start of that window. Only evaluated once the sample deque actually SPANS
    the window, which doubles as a post-dispatch planning grace period (Nav2
    gets a full window to plan and start moving before it can be judged idle).
  * FUTILE_SIM_TIMEOUT — the goal has been active for GOAL_SIM_TIMEOUT sim
    seconds regardless of motion; time to give up and pick another frontier.

The class keeps no wall-clock state whatsoever; wall-clock guards live in the
explorer and exist only for infrastructure-failure detection.
"""

import math
from collections import deque

# Verdicts.
RUNNING = 'RUNNING'
FUTILE_NO_PROGRESS = 'FUTILE_NO_PROGRESS'
FUTILE_SIM_TIMEOUT = 'FUTILE_SIM_TIMEOUT'


class GoalSupervisor:
    """Judge a single Nav2 goal from a stream of sim-time-stamped poses.

    One instance supervises one goal: construct it at dispatch, ``ingest`` a
    pose every control slice, and read ``verdict`` to decide whether to keep
    waiting (RUNNING) or cancel and blacklist (FUTILE_*).
    """

    # Re-exported as class attributes so callers can spell
    # GoalSupervisor.RUNNING etc. without importing the module constants.
    RUNNING = RUNNING
    FUTILE_NO_PROGRESS = FUTILE_NO_PROGRESS
    FUTILE_SIM_TIMEOUT = FUTILE_SIM_TIMEOUT

    PROGRESS_WINDOW = 20.0     # sim s; trailing window for the no-progress test
                               # (also the planning grace period)
    PROGRESS_MIN_MOVE = 0.25   # m; min displacement within the window to count
                               # as "still making progress"
    GOAL_SIM_TIMEOUT = 180.0   # sim s; hard per-goal budget in simulation time

    def __init__(self):
        self._samples = deque()   # (sim_time, x, y), oldest -> newest
        self._t_start = None      # sim_time of the very first sample (dispatch)

    def ingest(self, sim_time: float, x: float, y: float):
        """Record one pose sample at simulation time ``sim_time``.

        Prunes samples that have fallen more than PROGRESS_WINDOW behind the
        newest, but keeps the one straddling the window's leading edge so the
        deque's span stays >= PROGRESS_WINDOW once enough sim time has elapsed.
        """
        if self._t_start is None:
            self._t_start = sim_time
        self._samples.append((sim_time, x, y))
        while (len(self._samples) >= 2 and
               sim_time - self._samples[1][0] >= self.PROGRESS_WINDOW):
            self._samples.popleft()

    def elapsed(self) -> float:
        """Sim seconds since the first ingested sample (0 before any sample)."""
        if self._t_start is None or not self._samples:
            return 0.0
        return self._samples[-1][0] - self._t_start

    def window_move(self) -> float:
        """Max displacement from the current window's start over the samples held.

        This is the EXACT quantity the no-progress test thresholds against
        (PROGRESS_MIN_MOVE), computed over the trailing PROGRESS_WINDOW — not the
        whole goal. Exposed so a FUTILE_NO_PROGRESS can be logged with the windowed
        displacement that actually triggered it, rather than only the goal's net.
        0.0 before any sample.
        """
        if not self._samples:
            return 0.0
        ox, oy = self._samples[0][1], self._samples[0][2]
        return max(math.hypot(x - ox, y - oy) for _, x, y in self._samples)

    def verdict(self) -> str:
        """RUNNING, FUTILE_NO_PROGRESS, or FUTILE_SIM_TIMEOUT for the goal.

        Sim-timeout is checked first (a goal that overran its budget is futile
        even if it happens to be moving). No-progress is checked only once the
        deque spans a full PROGRESS_WINDOW — before that we are still inside the
        planning grace period and always report RUNNING.
        """
        if not self._samples:
            return RUNNING

        # Hard sim-time budget, measured from dispatch (survives window pruning
        # because _t_start is remembered independently of the deque).
        if self._samples[-1][0] - self._t_start >= self.GOAL_SIM_TIMEOUT:
            return FUTILE_SIM_TIMEOUT

        # No-progress: only judge once we hold a full window of history.
        span = self._samples[-1][0] - self._samples[0][0]
        if span >= self.PROGRESS_WINDOW:
            if self.window_move() < self.PROGRESS_MIN_MOVE:
                return FUTILE_NO_PROGRESS

        return RUNNING
