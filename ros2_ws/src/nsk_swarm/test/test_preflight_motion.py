"""Layer 1 — the geometry behind experiments/preflight_motion.py.

That script gates a timed run on whether the robots actually moved, and the
whole gate rests on one distinction: CUMULATIVE PATH LENGTH (summed inter-sample
steps) is not NET DISPLACEMENT (first sample to last). A robot bouncing between
two walls ends where it started, so asserting on displacement would report a
busily-wandering robot as stationary and abort a run that was fine.

These tests pin that distinction against synthetic tracks. No rclpy, no node, no
sim — the functions are pure and take [(x, y), ...].

The script lives in experiments/, which is not an importable package (nor a
colcon one), so it is loaded by location the way test_launch_descriptions.py
loads launch files. Importing it runs module-level code only: argparse, math and
rclpy imports, no rclpy.init() — main() is behind an __main__ guard.
"""

import importlib.util
import math
import os

import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
PREFLIGHT = os.path.join(REPO_ROOT, 'experiments', 'preflight_motion.py')


def load_preflight():
    spec = importlib.util.spec_from_file_location('preflight_motion', PREFLIGHT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


preflight = load_preflight()
path_length = preflight.path_length
net_displacement = preflight.net_displacement


def square(side=2.0):
    """A closed loop: four corners, back to the start. Perimeter 4 * side."""
    return [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]


def circle(radius=1.0, samples=360):
    """A closed loop with no straight segments, back to its start exactly."""
    points = [(radius * math.cos(2 * math.pi * i / samples),
               radius * math.sin(2 * math.pi * i / samples))
              for i in range(samples)]
    return points + [points[0]]


# ── The distinction the gate rests on ────────────────────────────────────────

def test_a_closed_loop_has_path_length_but_no_displacement():
    # THE case that decides which metric the motion assertion uses. This robot
    # drove 8 m and finished where it started; asserting on displacement would
    # call it stationary and abort a run that was working.
    track = square(side=2.0)

    assert path_length(track) == pytest.approx(8.0)
    assert net_displacement(track) == pytest.approx(0.0, abs=1e-12)


def test_a_closed_curve_has_path_length_but_no_displacement():
    # Same property without any straight segments — a robot circling, which is
    # what a wanderer pinned in a corner actually looks like.
    track = circle(radius=1.0, samples=360)

    # A 360-gon slightly undercuts its circumcircle; well clear of zero.
    assert path_length(track) == pytest.approx(2 * math.pi, rel=1e-3)
    assert path_length(track) > 6.0
    assert net_displacement(track) == pytest.approx(0.0, abs=1e-12)


def test_a_stationary_track_gives_zero_for_both():
    # The muted robot: odometry arriving steadily, pose never changing. Both
    # metrics agree here, which is why zero path length is a usable floor.
    track = [(1.5, -0.25)] * 200

    assert path_length(track) == pytest.approx(0.0, abs=1e-12)
    assert net_displacement(track) == pytest.approx(0.0, abs=1e-12)


def test_a_straight_line_makes_the_two_metrics_agree():
    # The only shape where they coincide — which is what shows the gap in the
    # tests above is doubling back, not a scale factor between two formulas.
    track = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]

    assert path_length(track) == pytest.approx(3.0)
    assert net_displacement(track) == pytest.approx(3.0)


def test_jitter_in_place_stays_far_below_a_real_floor():
    # Odometry noise accumulates into path length (every wobble is a step), so
    # the floor has to clear it. A millimetre of jitter over 200 samples is
    # ~0.2 m worst case — recorded here so the calibration pass has a number to
    # beat, not just a vibe.
    track = [(0.001 * (i % 2), 0.0) for i in range(200)]

    # 199 steps of 1 mm accumulate; displacement stays at one jitter step. The
    # gap between them IS the accumulation, and it is why a floor set from
    # displacement would be a different (and much smaller) number.
    assert path_length(track) == pytest.approx(0.199, abs=1e-9)
    assert net_displacement(track) == pytest.approx(0.001, abs=1e-9)


# ── Degenerate input ─────────────────────────────────────────────────────────

def test_no_samples_gives_zero_for_both():
    # The zero-message robot. Both must return a number rather than raise:
    # preflight_motion computes these before deciding the msgs>0 verdict, and a
    # traceback there would lose the whole table, including the robots that
    # were fine.
    assert path_length([]) == 0.0
    assert net_displacement([]) == 0.0


def test_one_sample_gives_zero_for_both():
    assert path_length([(4.0, 4.0)]) == 0.0
    assert net_displacement([(4.0, 4.0)]) == 0.0
