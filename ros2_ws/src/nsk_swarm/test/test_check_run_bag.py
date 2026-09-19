"""Layer 1 — C3, the prologue check in experiments/slam/check_run_bag.py.

C3 asks one question of a single-explorer run bag: did recording start before
robot K was ever commanded to move? It answers it over the interval from the
bag's first message to the first nonzero command on /robot_K/cmd_vel, and it
takes the MAXIMUM over that interval rather than a reading at one instant.

The yaw half is the part worth testing. Nav2 can open with an in-place turn,
which moves yaw and leaves position alone; a position-only check would call
that a quiet prologue and pass a bag whose cmd_vel subscription merely arrived
late. The rotation case below fails only because yaw is checked.

check_prologue() is pure — it takes the two series and the spawn pose — so
none of this needs a bag, rosbag2 or ROS. The script lives in experiments/,
which is not an importable package, so it is loaded by location the way
test_preflight_motion.py loads preflight_motion.py.
"""

import importlib.util
import math
import os

import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
CHECKER = os.path.join(REPO_ROOT, 'experiments', 'slam', 'check_run_bag.py')


def load_checker():
    spec = importlib.util.spec_from_file_location('check_run_bag', CHECKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


checker = load_checker()
check_prologue = checker.check_prologue
first_nonzero_cmd = checker.first_nonzero_cmd

BAG_START = 1000.0
SPAWN = (0.860, 0.280)          # robot_0's DOT_POSES entry at 08617b2
TRUTH_HZ = 16.57                # the measured /model/robot_N/pose rate


def poses(prologue_s, hz=TRUTH_HZ, drift_m=0.0, drift_deg=0.0, at=0.5):
    """Truth samples across the prologue, optionally moving partway through.

    `at` is the fraction of the window after which the robot has moved, so the
    displacement sits INSIDE the interval rather than at its edge — which is
    what makes this a test of the maximum over the window.
    """
    n = int(prologue_s * hz)
    out = []
    for i in range(n):
        t = BAG_START + i / hz
        moved = (i / n) >= at if n else False
        out.append((t,
                    SPAWN[0] + (drift_m if moved else 0.0),
                    SPAWN[1],
                    math.radians(drift_deg) if moved else 0.0))
    return out


def cmds(prologue_s, hz=20.0):
    """Zeros through the prologue, then one unmistakably nonzero command."""
    out = [(BAG_START + i / hz, 0.0, 0.0)
           for i in range(int(prologue_s * hz))]
    out.append((BAG_START + prologue_s, 0.15, 0.0))
    return out


# ── the four cases ───────────────────────────────────────────────────────────

def test_clean_prologue_passes():
    """6 s, ~99 samples, robot sitting on spawn throughout."""
    ok, lines = check_prologue(BAG_START, cmds(6.0), poses(6.0), SPAWN)
    assert ok, '\n'.join(lines)


def test_translation_before_first_command_fails():
    """It rolled 5 cm before anything commanded it to."""
    ok, lines = check_prologue(BAG_START, cmds(6.0),
                               poses(6.0, drift_m=0.05), SPAWN)
    assert not ok
    assert any('max displacement' in ln and 'FAIL' in ln for ln in lines)


def test_rotation_only_before_first_command_fails():
    """Turned 2 deg in place: position is spotless, yaw is not.

    This is the case a position-only check passes. If it ever starts passing
    here, the yaw half of C3 has stopped being load-bearing.
    """
    ok, lines = check_prologue(BAG_START, cmds(6.0),
                               poses(6.0, drift_deg=2.0), SPAWN)
    assert not ok
    assert any('max |yaw|' in ln and 'FAIL' in ln for ln in lines)
    # and the position half is genuinely clean, so yaw alone caused the failure
    assert any('max displacement' in ln and 'ok' in ln for ln in lines)


def test_short_prologue_fails():
    """2 s of quiet is not enough, even when the robot never moved.

    Sampled at 40 Hz so 80 samples clear the density gate and LENGTH is the
    only thing that fails — otherwise this would not distinguish the two.
    """
    ok, lines = check_prologue(BAG_START, cmds(2.0), poses(2.0, hz=40.0), SPAWN)
    assert not ok
    assert any('prologue length' in ln and 'FAIL' in ln for ln in lines)
    assert any('pose samples' in ln and 'ok' in ln for ln in lines)


# ── supporting behaviour ─────────────────────────────────────────────────────

def test_sparse_prologue_fails_on_density():
    """Long enough, but truth was barely sampled: 6 s at 2 Hz is 12 samples."""
    ok, lines = check_prologue(BAG_START, cmds(6.0), poses(6.0, hz=2.0), SPAWN)
    assert not ok
    assert any('pose samples' in ln and 'FAIL' in ln for ln in lines)
    assert any('prologue length' in ln and 'ok' in ln for ln in lines)


def test_no_command_at_all_fails():
    """A run that never drove is not a run that explored."""
    quiet = [(BAG_START + i / 20.0, 0.0, 0.0) for i in range(200)]
    ok, lines = check_prologue(BAG_START, quiet, poses(10.0), SPAWN)
    assert not ok
    assert any('never commanded' in ln for ln in lines)


def test_residue_below_threshold_is_not_a_command():
    """Near-zero floats while the smoother ramps down must not count."""
    eps = checker.CMD_EPS_ANGULAR / 2.0
    assert first_nonzero_cmd([(1.0, 0.0, eps), (2.0, eps, 0.0)]) is None
    assert first_nonzero_cmd([(1.0, 0.0, eps),
                              (2.0, 0.0, checker.CMD_EPS_ANGULAR * 10)]) == 2.0


def test_angular_only_command_starts_the_clock():
    """An in-place turn is a command; the window must end there, not later."""
    series = [(BAG_START + 1.0, 0.0, 0.5), (BAG_START + 2.0, 0.2, 0.0)]
    assert first_nonzero_cmd(series) == BAG_START + 1.0


# ── the resting tilt must not read as a turn ─────────────────────────────────
#
# Measured on b2maps_rehearsal3, first /model/robot_0/pose: the burger sits at
# a pitch of -0.714 deg once its suspension settles. C2 and C3 both allow only
# 0.5 deg, so if either compared the quaternion's TOTAL rotation angle instead
# of heading about Z, every bag would fail on tilt alone, on a robot that had
# not turned. Both take yaw only; these pin that.

RESTING = dict(x=-0.0000003, y=-0.0062281, z=0.0, w=0.999980605)


def test_resting_pitch_reads_as_zero_yaw():
    yaw = checker.yaw_from_quaternion(**RESTING)
    assert abs(math.degrees(yaw)) < 0.001
    assert abs(math.degrees(yaw)) <= checker.SPAWN_TOL_DEG


def test_total_rotation_angle_would_have_failed():
    """The trap this guards, stated as an assertion rather than a comment."""
    total = 2 * math.acos(min(1.0, abs(RESTING['w'])))
    assert math.degrees(total) == pytest.approx(0.714, abs=0.01)
    assert math.degrees(total) > checker.SPAWN_TOL_DEG


def test_prologue_passes_with_the_resting_pitch_throughout():
    """C3 end-to-end on a robot that is tilted for the whole prologue."""
    yaw = checker.yaw_from_quaternion(**RESTING)
    tilted = [(BAG_START + i / TRUTH_HZ, SPAWN[0], SPAWN[1], yaw)
              for i in range(int(6.0 * TRUTH_HZ))]
    ok, lines = check_prologue(BAG_START, cmds(6.0), tilted, SPAWN)
    assert ok, '\n'.join(lines)


def test_a_real_turn_is_still_caught_when_tilted():
    """Tilt is forgiven; heading is not. 2 deg of yaw on top of the pitch."""
    turned = [(BAG_START + i / TRUTH_HZ, SPAWN[0], SPAWN[1],
               math.radians(2.0) if i / int(6.0 * TRUTH_HZ) >= 0.5 else 0.0)
              for i in range(int(6.0 * TRUTH_HZ))]
    ok, lines = check_prologue(BAG_START, cmds(6.0), turned, SPAWN)
    assert not ok
    assert any('max |yaw|' in ln and 'FAIL' in ln for ln in lines)


# ── C4, the stopping rule ────────────────────────────────────────────────────
#
# check_budget() answers two things: is there enough bag left after the first
# command to map from, and where should strip_bag_for_offline_slam.py cut. The
# offset is in THAT script's convention -- seconds from the first message it
# would keep -- which is not in general the bag's first message, so the
# reference is passed in rather than assumed.

STRIP_T0 = BAG_START          # first kept message; /clock usually wins
BUDGET = 2400.0
CLOCK_STEP_SIM = 0.01         # 100 Hz, as these runs publish it


def clocks(sim_seconds, rtf, start_recv=BAG_START, start_sim=0.0,
           step=CLOCK_STEP_SIM):
    """/clock as (receive_s, sim_s), advancing `step` of sim per message.

    rtf is sim seconds per wall second, so a message costs step/rtf of
    receive time. rtf=1.0 makes the two clocks agree; rtf=0.5 makes the bag
    twice as long as the simulation it carries.
    """
    n = int(round(sim_seconds / step)) + 1
    return [(start_recv + i * step / rtf, start_sim + i * step)
            for i in range(n)]


def budget_case(sim_available, rtf=1.0, offset=55.72, strip_t0=STRIP_T0,
                budget=BUDGET):
    """A bag whose first command lands `offset` in, on a clock boundary.

    The command is placed exactly on a /clock receive time so sim_start is
    that message's value and the arithmetic below is exact.
    """
    t_cmd = strip_t0 + offset
    series = clocks(sim_available, rtf, start_recv=t_cmd, start_sim=1000.0)
    bag_end = series[-1][0]
    return checker.check_budget(strip_t0, bag_end, t_cmd, budget, series)


# ── the budget is in SIM seconds, so --duration tracks RTF ───────────────────

def test_rtf_one_gives_duration_equal_to_the_budget():
    ok, lines, offset = budget_case(BUDGET + 1.0, rtf=1.0)
    assert ok, '\n'.join(lines)
    assert offset == pytest.approx(55.72)
    assert any('--start 55.720 --duration 2400.000' in ln for ln in lines)
    assert any('mean RTF over window' in ln and '1.0000' in ln for ln in lines)


def test_rtf_half_gives_twice_the_bag_seconds():
    """The same 2400 s of simulation, bought with 4800 s of recording.

    This is the whole point of the change: a window fixed in bag seconds
    would have carried only half the simulation here.
    """
    ok, lines, _ = budget_case(BUDGET + 1.0, rtf=0.5)
    assert ok, '\n'.join(lines)
    assert any('--start 55.720 --duration 4800.000' in ln for ln in lines)
    assert any('mean RTF over window' in ln and '0.5000' in ln for ln in lines)


def test_sim_span_is_the_budget_at_either_rtf():
    for rtf in (1.0, 0.5):
        _ok, lines, _ = budget_case(BUDGET + 1.0, rtf=rtf)
        span = [ln for ln in lines if 'sim span of window' in ln][0]
        assert '2400.000' in span, span


def test_clock_ending_short_of_the_budget_fails():
    ok, lines, offset = budget_case(BUDGET - 1.0, rtf=1.0)
    assert not ok
    assert offset == pytest.approx(55.72)
    assert any('never advances' in ln for ln in lines)
    assert not any('--start' in ln for ln in lines)


def test_clock_ending_exactly_at_the_budget_passes():
    """Inclusive boundary: the clock that reaches the budget is enough."""
    ok, lines, _ = budget_case(BUDGET, rtf=1.0)
    assert ok, '\n'.join(lines)


def test_fail_prints_no_strip_arguments():
    """A short bag must not be handed a command that would silently truncate."""
    ok, lines, _ = budget_case(BUDGET - 10.0)
    assert not ok
    assert not any('--start' in ln for ln in lines)


def test_offset_is_measured_from_the_first_kept_message():
    """Not from the bag's first message.

    A bag that opens on a topic the strip script drops -- a truth pose, say --
    has a kept-message reference LATER than its own start, and the offset must
    shrink by exactly that gap or --start would overshoot the first command.
    """
    ok, _lines, offset = budget_case(BUDGET + 1.0, offset=55.72,
                                     strip_t0=BAG_START + 2.0)
    assert ok
    assert offset == pytest.approx(55.72)


def test_no_command_fails_the_budget():
    ok, lines, offset = checker.check_budget(
        STRIP_T0, STRIP_T0 + 3000.0, None, BUDGET, clocks(3000.0, 1.0))
    assert not ok
    assert offset is None
    assert any('nothing to replay' in ln for ln in lines)


def test_no_clock_before_the_command_fails():
    """sim_start is undefined, so the window cannot be placed."""
    t_cmd = STRIP_T0 + 10.0
    late = clocks(100.0, 1.0, start_recv=t_cmd + 1.0)
    ok, lines, _ = checker.check_budget(STRIP_T0, t_cmd + 200.0, t_cmd,
                                        BUDGET, late)
    assert not ok
    assert any('never advances' in ln for ln in lines)


# ── sim_window(), the mapping itself ─────────────────────────────────────────

def test_sim_time_is_the_latest_clock_at_or_before_the_event():
    series = [(100.0, 10.0), (101.0, 11.0), (102.0, 12.0), (103.0, 13.0)]
    # command at 101.5 -> sim_start is 11.0, the 101.0 message, not 12.0
    sim_start, t_end, sim_end = checker.sim_window(series, 101.5, 1.0)
    assert sim_start == pytest.approx(11.0)
    assert sim_end == pytest.approx(12.0)      # first value >= 11.0 + 1.0
    assert t_end == pytest.approx(102.0)


def test_a_clock_exactly_at_the_event_counts():
    """'at or before' is inclusive."""
    series = [(100.0, 10.0), (101.0, 11.0), (102.0, 12.0)]
    sim_start, _, _ = checker.sim_window(series, 101.0, 1.0)
    assert sim_start == pytest.approx(11.0)


def test_sim_window_returns_none_when_the_budget_is_never_reached():
    series = [(100.0, 10.0), (101.0, 11.0)]
    assert checker.sim_window(series, 100.0, 50.0) is None


def test_overshoot_beyond_the_slack_fails():
    """A coarse or gappy /clock must not pass as if it hit the budget.

    One 5 s jump across the boundary overshoots by 4 s, far past the 0.1 s
    one clock step would cost.
    """
    t_cmd = STRIP_T0
    series = [(t_cmd, 0.0), (t_cmd + 1.0, BUDGET + 4.0)]
    ok, lines, _ = checker.check_budget(STRIP_T0, t_cmd + 10.0, t_cmd,
                                        BUDGET, series)
    assert not ok
    assert any('sim span of window' in ln and 'FAIL' in ln for ln in lines)
    assert any('coarse or has a gap' in ln for ln in lines)
    assert not any('--start' in ln for ln in lines)


def test_strip_keep_set_matches_the_strip_script():
    """13 topics: /clock, /tf, /tf_static and scan+odom for five robots.

    Mirrored from strip_bag_for_offline_slam.py:63-65. If that script's keep
    set changes and this does not, C4's offset stops being in its convention.
    """
    keep = checker.strip_keep_topics()
    assert keep == {'/clock', '/tf', '/tf_static'} | {
        f'/robot_{n}/{t}' for n in range(5) for t in ('scan', 'odom')}
    assert len(keep) == 13
    # the topics C4 must NOT anchor on
    assert '/model/robot_0/pose' not in keep
    assert '/robot_0/cmd_vel' not in keep
    assert '/robot_0/map' not in keep


def test_budget_default_is_the_documented_one():
    assert checker.BUDGET_S == pytest.approx(2400.0)


def test_thresholds_are_the_documented_ones():
    assert checker.PROLOGUE_MIN_S == pytest.approx(5.0)
    assert checker.PROLOGUE_MIN_POSES == 50
    assert checker.SPAWN_TOL_M == pytest.approx(0.01)
    assert checker.SPAWN_TOL_DEG == pytest.approx(0.5)
