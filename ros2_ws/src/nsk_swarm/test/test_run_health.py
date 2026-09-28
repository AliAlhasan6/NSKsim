"""Unit tests for experiments/analysis/run_health.py -- pure logic, no bags.

Every gate here decides whether a recorded run is usable, and each one fails
QUIETLY if it is wrong: a warning pattern that no longer matches counts zero
and reads as healthy, a boundary criterion pointed at the wrong walls reports
a distance that means nothing, a transform rate measured against the wrong
denominator calls a dead publisher healthy, and a map written with the rows
flipped looks entirely plausible in a viewer. So the patterns are tested
against lines in the format the explore log actually uses, the slam_toolbox
filter is tested to REJECT the costmap's identically-worded warning, the
boundary selection is tested to be the perimeter and nothing else, R2's two
denominators are tested to disagree on the run where they should, and the
written map is read back with the project's own PGM reader.

The coverage numbers are anchored on b2maps_e0, the run being replaced: its
closest approach is 3.897 m against a 3.50 m lidar range, so the criterion
must fail there. A criterion that passed the run that motivated it would be
decoration.

Finds the module by walking up to experiments/analysis/, the convention
test_check_cut_reference.py uses.
"""
import functools
import importlib.util
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest


def _load():
    for parent in Path(__file__).resolve().parents:
        for cand in (parent / 'run_health.py',
                     parent / 'experiments' / 'analysis' / 'run_health.py'):
            if cand.is_file():
                spec = importlib.util.spec_from_file_location('run_health', cand)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
    raise ImportError('run_health.py not found above this test')


rh = _load()

# R4's analysis window is check_run_bag.py's window -- run_health imports
# first_nonzero_cmd and sim_window from it rather than restating either. Taken
# out of sys.modules, not loaded again by location: a second copy of the module
# would make the identity assertions below pass against nothing.
checker = sys.modules['check_run_bag']

# ── the stock TurtleBot3 model, and what to do without it ────────────────────
# run_health reads three numbers out of files: the lidar range and the two
# DiffDrive frequencies from the stock burger SDF, and the PosePublisher rate
# from this repo's launch file. The burger SDF belongs to turtlebot3_gazebo,
# which CI deliberately does NOT install (.github/workflows/ci.yml explains
# why), and run_health dies rather than defaulting when a file it needs is
# missing -- correctly, for a measurement rig.
#
# So the criteria below are written against LITERALS, and the reading of the
# files is one gated test per file that the literal is still what the file
# says. A missing turtlebot3 then skips three tests instead of taking the
# whole module down at import, which is what run 35506940347 did: a
# module-level rate_facts() call turned an absent optional package into
# "1 test, 1 error, collection failure" for the entire nsk_swarm suite.
BURGER_SDF_PRESENT = rh.BURGER_SDF.is_file()
needs_burger = pytest.mark.skipif(
    not BURGER_SDF_PRESENT,
    reason=f'{rh.BURGER_SDF} not installed -- the stock model is what these '
           'tests read')

# The ceiling the sim RUNS, which is not the stock model's. BURGER_RANGE_MAX in
# swarm_sim.launch.py; make_namespaced_burger_sdf rewrites model.sdf:153's 3.5
# into every spawned robot. Reading the stock file here is the bug
# lidar_max_range was changed to prevent, so the literal tracks the launch file.
LIDAR_MAX_M = 8.0
STOCK_LIDAR_MAX_M = 3.5  # burger LDS-01 <range><max>, model.sdf:153
TRUTH_HZ = 20.0          # PosePublisher <update_frequency>, swarm_sim.launch.py
DIFFDRIVE_HZ = 50.0      # the gz-sim DiffDrive default; see rate_facts()


@functools.lru_cache(maxsize=1)
def facts():
    """rate_facts(), read once, and only inside a test that needs it.

    Lazy on purpose: at module scope this is an import-time file read, and a
    file read that can sys.exit() at import time is a collection failure for
    every test in the module, including the ones that never touch a file.
    """
    return rh.rate_facts()


# Real lines from experiments/logs/b2maps/b2maps_e0_explore.log, and synthetic
# ones in the same shape for the warnings e0 never produced.
E0_NO_PROGRESS = (
    '[controller_server-2] [ERROR] [1789836623.505604781] '
    '[robot_0.controller_server]: Failed to make progress')
E0_LOOP_RATE = (
    '[controller_server-2] [WARN] [1789836201.196742048] '
    '[robot_0.controller_server]: Control loop missed its desired rate of '
    '20.0000 Hz. Current loop rate is 19.2261 Hz.')
E0_LAUNCH_LINE = '[INFO] [launch]: Default logging verbosity is set to INFO'

COSTMAP_TIMEOUT = (
    '[controller_server-2] [WARN] [1789836000.000000000] '
    '[robot_0.local_costmap.local_costmap]: Timed out waiting for transform '
    'from robot_0/base_footprint to robot_0/odom to become available')
CONTROLLER_TOO_OLD = (
    '[controller_server-2] [WARN] [1789836010.000000000] '
    '[robot_0.controller_server]: Transform data too old')
TF2_EXTRAPOLATION = (
    '[bt_navigator-7] [WARN] [1789836020.000000000] [robot_0.bt_navigator]: '
    'Lookup would require extrapolation into the future.')
SLAM_DROP = (
    '[async_slam_toolbox_node-1] [WARN] [1789836030.000000000] '
    '[robot_0.slam_toolbox]: Message Filter dropping message: frame '
    "'robot_0/base_scan' at time 1234.567 for reason 'discarding message "
    "because the queue is full'")
COSTMAP_DROP = (
    '[controller_server-2] [WARN] [1789836040.000000000] '
    '[robot_0.local_costmap.local_costmap]: Message Filter dropping message: '
    "frame 'robot_0/base_scan' at time 1234.567 for reason 'the timestamp on "
    "the message is earlier than all the data in the transform cache'")


# ── R9: what gets counted ────────────────────────────────────────────────────

def test_the_three_asked_for_categories_plus_the_reported_one_exist():
    assert set(rh.COUNTS) == {'nav2_transform', 'slam_drop', 'no_progress',
                              'loop_rate'}
    # Only the first two decide the run.
    assert set(rh.GATED) == {'nav2_transform', 'slam_drop'}


def test_each_transform_phrasing_is_counted():
    found = rh.count_health([COSTMAP_TIMEOUT, CONTROLLER_TOO_OLD,
                             TF2_EXTRAPOLATION])
    assert found['counts']['nav2_transform'] == 3
    assert found['counts']['slam_drop'] == 0


def test_a_slam_toolbox_drop_is_counted():
    found = rh.count_health([SLAM_DROP])
    assert found['counts']['slam_drop'] == 1


def test_a_costmap_drop_is_not_a_slam_toolbox_drop():
    """The same sentence comes out of libtoolbox_common.so and liblayers.so.

    Counting the costmap's copy as a slam_toolbox drop would make the gate
    fire on a costmap that is merely catching up, which has nothing to do
    with the transform this change moves.
    """
    found = rh.count_health([COSTMAP_DROP])
    assert found['counts']['slam_drop'] == 0
    assert found['counts']['nav2_transform'] == 0


def test_the_real_e0_lines_land_in_the_right_categories():
    found = rh.count_health([E0_NO_PROGRESS, E0_LOOP_RATE, E0_LAUNCH_LINE])
    assert found['counts']['no_progress'] == 1
    assert found['counts']['loop_rate'] == 1
    assert found['counts']['nav2_transform'] == 0
    assert found['counts']['slam_drop'] == 0


def test_counts_are_attributed_to_the_process_that_logged_them():
    found = rh.count_health([SLAM_DROP, COSTMAP_TIMEOUT, CONTROLLER_TOO_OLD])
    assert found['by_proc']['slam_drop'] == {'async_slam_toolbox_node': 1}
    assert found['by_proc']['nav2_transform'] == {'controller_server': 2}


def test_the_span_is_the_first_and_last_timestamp():
    found = rh.count_health([E0_LAUNCH_LINE, COSTMAP_TIMEOUT,
                             CONTROLLER_TOO_OLD, TF2_EXTRAPOLATION])
    assert found['first'] == pytest.approx(1789836000.0)
    assert found['last'] == pytest.approx(1789836020.0)
    assert found['span_s'] == pytest.approx(20.0)
    # The launch line carries no timestamp and must not be counted as one.
    assert found['stamped'] == 3


def test_a_log_with_no_timestamps_has_no_span():
    found = rh.count_health([E0_LAUNCH_LINE])
    assert found['first'] is None
    assert found['span_s'] == 0.0


# ── R9: rates and the gate ───────────────────────────────────────────────────

def test_rates_are_per_minute():
    rates = rh.per_minute({'a': 40, 'b': 0}, 5937.577)
    assert rates['a'] == pytest.approx(0.4042, abs=1e-4)   # e0's no_progress
    assert rates['b'] == 0.0


def test_a_burst_inside_a_zero_span_is_infinite_not_a_crash():
    rates = rh.per_minute({'a': 3, 'b': 0}, 0.0)
    assert rates['a'] == math.inf
    assert rates['b'] == 0.0


def test_a_zero_baseline_makes_any_occurrence_a_failure():
    """b2maps_e0 logged zero transform warnings in 98.96 minutes, so the
    limit is zero and the first warning is the signal. This is the gate that
    will actually run at the rehearsal.
    """
    baseline = {'nav2_transform': 0.0, 'slam_drop': 0.0}

    clean = rh.judge({'nav2_transform': 0.0, 'slam_drop': 0.0}, baseline)
    assert clean['nav2_transform'][0] is True
    assert clean['slam_drop'][0] is True

    one_warning = rh.judge({'nav2_transform': 0.01, 'slam_drop': 0.0}, baseline)
    assert one_warning['nav2_transform'][0] is False
    assert one_warning['slam_drop'][0] is True


def test_exactly_twice_the_baseline_passes_and_more_fails():
    baseline = {'nav2_transform': 1.0, 'slam_drop': 2.0}

    at_limit = rh.judge({'nav2_transform': 2.0, 'slam_drop': 4.0}, baseline)
    assert at_limit['nav2_transform'] == (True, 2.0)
    assert at_limit['slam_drop'] == (True, 4.0)

    over = rh.judge({'nav2_transform': 2.001, 'slam_drop': 4.001}, baseline)
    assert over['nav2_transform'][0] is False
    assert over['slam_drop'][0] is False


def test_the_ungated_categories_are_not_judged():
    verdicts = rh.judge({'nav2_transform': 0.0, 'slam_drop': 0.0,
                         'no_progress': 99.0, 'loop_rate': 99.0},
                        {'nav2_transform': 0.0, 'slam_drop': 0.0,
                         'no_progress': 0.0, 'loop_rate': 0.0})
    assert set(verdicts) == set(rh.GATED)


# ── coverage: closest approach ───────────────────────────────────────────────

def test_closest_approach_to_a_single_wall():
    wall = [(-0.05, 0.05, -10.0, 10.0)]          # a wall along x = 0
    assert rh.closest_approach([(2.05, 0.0)], wall) == pytest.approx(2.0)
    assert rh.closest_approach([(-2.05, 5.0)], wall) == pytest.approx(2.0)


def test_closest_approach_takes_the_minimum_over_the_whole_trajectory():
    wall = [(-0.05, 0.05, -10.0, 10.0)]
    path = [(9.05, 0.0), (4.05, 0.0), (0.55, 0.0), (7.05, 0.0)]
    assert rh.closest_approach(path, wall) == pytest.approx(0.5)


def test_a_point_inside_a_wall_is_zero_away():
    wall = [(-0.05, 0.05, -10.0, 10.0)]
    assert rh.closest_approach([(0.0, 3.0)], wall) == 0.0


def test_an_empty_trajectory_never_reaches_anything():
    wall = [(-0.05, 0.05, -10.0, 10.0)]
    assert rh.closest_approach([], wall) == math.inf


def test_the_boundary_is_the_four_perimeter_walls_only():
    """Selected by name, so an edited world fails loudly instead of measuring
    against a maze wall that happens to be long.
    """
    rects = rh.boundary_rects()
    assert len(rects) == 4
    for x0, x1, y0, y1 in rects:
        # Every perimeter wall spans the full 20.2 m in one axis and is thin
        # in the other; no maze wall does.
        assert max(x1 - x0, y1 - y0) == pytest.approx(20.2)
        assert min(x1 - x0, y1 - y0) == pytest.approx(0.1)


def test_the_e0_closest_approach_is_measured_the_same_way_it_always_was():
    """The geometry is fixed; what it MEANS moved with the lidar.

    e0's nearest point to the perimeter was (-6.053, +3.487), 3.897 m from
    wall_west's inner face at x = -9.95. That distance is a fact about a
    recorded trajectory and does not change, so it is still asserted here.

    What changed is the verdict. Against the old 3.50 m ceiling this run FAILED
    the criterion, and that failure is why the criterion exists. Against the
    8.0 m one it passes with 4.1 m to spare -- so COVERAGE no longer
    discriminates the run that motivated it, and on its own it is now a weak
    gate. It is kept because a run that never comes within lidar range of the
    perimeter is still definitively bad; it is no longer sufficient, and the
    criterion that carries the weight at 8 m is the b2maps README's fifth one,
    occupied cells on the outer boundary of the offline truth map.

    Both bounds are asserted so that the day either ceiling moves again, this
    test says which way and by how much.
    """
    d = rh.closest_approach([(-6.053, 3.487)], rh.boundary_rects())

    assert d == pytest.approx(3.897, abs=1e-3)
    assert d > STOCK_LIDAR_MAX_M     # failed the criterion as originally set
    assert d <= LIDAR_MAX_M          # passes it at the range the sim now runs


def test_a_trajectory_that_does_reach_the_boundary_passes():
    # Same path, plus one sample 3.0 m from the west wall's inner face.
    d = rh.closest_approach([(-6.053, 3.487), (-6.95, 0.0)],
                            rh.boundary_rects())

    assert d == pytest.approx(3.0, abs=1e-6)
    assert d <= LIDAR_MAX_M


# ── the constants are read, not retyped ──────────────────────────────────────
# These are the gated tests: each asserts that a literal used above is still
# what the file it comes from says. Skipped without turtlebot3_gazebo, which
# means the criteria keep running there and only their provenance goes
# unchecked.

@needs_burger
def test_the_lidar_range_comes_from_the_launch_file_not_the_stock_model():
    """The gate must read the range the ROBOTS HAVE, not the one on disk.

    make_namespaced_burger_sdf rewrites <range><max> into every spawned model,
    so the stock file keeps advertising a sensor nothing in this sim carries.
    Grepping it would leave COVERAGE testing against 3.5 m while the robots saw
    8 m -- and silently, because _grep_one only dies on an ABSENT pattern and
    <max>3.5</max> is still right there. Silently, and in the direction that
    makes the gate easier to pass.

    So: the effective value is 8.0, it is not the stock 3.5, and the source
    string names both files so a future divergence is readable rather than
    invisible.
    """
    reach, src = rh.lidar_max_range()

    assert reach == pytest.approx(LIDAR_MAX_M)
    assert reach != pytest.approx(STOCK_LIDAR_MAX_M)
    assert 'swarm_sim.launch.py' in src
    assert f'stock {STOCK_LIDAR_MAX_M:.2f} m' in src
    assert 'model.sdf:153' in src


@needs_burger
def test_the_transform_rates_come_from_the_files_that_set_them():
    """Values and source FILES, not source lines, and the literals above.

    This is the one test that ties TRUTH_HZ and DIFFDRIVE_HZ to the files
    they came from; every other use of them is a literal, so that a missing
    turtlebot3_gazebo costs this check alone.


    The burger SDF is a fixed upstream install, so its line numbers are worth
    pinning above -- a ROS update that moves them should be seen. swarm_sim's
    own line number moves whenever the launch file is edited, and pinning it
    would only ever report that someone added a comment.
    """
    f = facts()
    assert f['truth_hz'] == pytest.approx(TRUTH_HZ)
    assert f['diffdrive_hz'] == pytest.approx(DIFFDRIVE_HZ)
    assert f['diffdrive_src'].endswith('model.sdf:394')
    assert re.fullmatch(r'ros2_ws/src/nsk_swarm/launch/swarm_sim\.launch\.py:\d+',
                        f['truth_src']), f['truth_src']


@needs_burger
def test_the_burger_sdfs_diffdrive_frequency_is_inert():
    """The 30 on model.sdf:394 has never taken effect.

    The stock model writes <odom_publisher_frequency>; gz-sim's DiffDrive
    reads <odom_publish_frequency>, and an unknown SDF element is ignored in
    silence. So the transform this change replaces ran at the plugin default,
    50 Hz -- which is what b2maps_e0 measured: 290035 /robot_0/odom messages
    over 5801.1 s of its own sim clock is 50.0 Hz, and 30 Hz would have been
    174033.

    Reported rather than corrected. Fixing the spelling would change the rate
    every earlier run was recorded at, and this is a measurement rig.

    If the stock model is ever fixed upstream, this test goes red and the
    reported rate becomes whatever the model then asks for -- which is the
    intent, not a breakage.
    """
    f = facts()
    assert f['diffdrive_nominal_hz'] == pytest.approx(30.0)
    assert f['diffdrive_ignored'] is True
    assert f['diffdrive_hz'] == pytest.approx(DIFFDRIVE_HZ)
    assert f['diffdrive_hz'] == pytest.approx(rh.DIFFDRIVE_DEFAULT_HZ)


@needs_burger
def test_the_measured_e0_odom_rate_matches_the_effective_not_the_nominal():
    """Arithmetic on b2maps_e0's own recorded counts, so the claim above is
    checked rather than asserted.
    """
    f = facts()
    odom_messages, sim_span_s = 290035, 5801.1

    measured = odom_messages / sim_span_s
    assert measured == pytest.approx(f['diffdrive_hz'], abs=0.1)
    assert measured != pytest.approx(f['diffdrive_nominal_hz'], abs=1.0)


# ── R4/R5: the transform is the truth, the wheel is still the wheel ──────────

SPAWN0 = (0.86, 0.28, 0.0)


def _truth_driven(n=200, turn=True):
    """A synthetic truth-driven run: poses, the transforms a correct
    truth_odom_tf would publish for them, and a wheel series that drifts.
    """
    truth, tf_pairs, wheel = {}, [], []
    for i in range(n):
        ns = 1_000_000_000 + i * 50_000_000          # 20 Hz, like PosePublisher
        yaw = math.radians(i * 1.7) if turn else 0.0
        pose = (SPAWN0[0] + i * 0.01, SPAWN0[1] + i * 0.005, yaw)
        truth[ns] = pose
        x, y, t = rh._inv_compose(SPAWN0, pose)
        tf_pairs.append((ns, x, y, t, 0.0))
        # Wheel odometry drifting a degree per sample -- b2maps_e0's real
        # series reached a median of 138 deg.
        wheel.append((ns, t + math.radians(i * 1.0)))
    return truth, tf_pairs, wheel


def test_inv_compose_matches_the_hand_calculated_vector():
    """The same C1 vector truth_odom_tf and rewrite_odom_from_truth use.

    This function is written out a third time on purpose -- a check that
    imported the thing it checks would pass by construction -- so it has to be
    pinned to the same external value.
    """
    got = rh._inv_compose((2.0, -1.0, math.radians(30.0)),
                          (3.0, 1.0, math.radians(75.0)))
    assert got[0] == pytest.approx(1.866025, abs=1e-6)
    assert got[1] == pytest.approx(1.232051, abs=1e-6)
    assert math.degrees(got[2]) == pytest.approx(45.0, abs=1e-6)


PERIOD_NS = 50_000_000  # 1 / TRUTH_HZ, the spacing _truth_driven() uses

# ── the analysis window R4 gates ─────────────────────────────────────────────
# [t_cmd, t_cmd + --min-sim] in SIM time: the span the offline map is built
# from. _truth_driven() runs sim 1.000 .. 10.950 s, so two windows cover every
# case below.
#
#   FULL_WINDOW  the whole series. The default, so that every test about the
#                COMPARISON keeps saying what it always said.
#   MID_WINDOW   t_cmd at sample 20 (sim 2.000), 5 s long, closing at sample
#                120 (sim 7.000). 20 samples sit before t_cmd and 79 after the
#                window, which is what makes "outside" testable at all.
SERIES_LO, SERIES_HI = 1.0, 10.95
T_CMD_I, WINDOW_S = 20, 5.0
BAG_T_CMD = 1790203872.243517       # b2maps_k2's, whose window shape this is


def _window(sim_start, min_sim, robot=0):
    """A window as read_analysis_window() builds it, without a bag.

    The receive-clock half is R9's, not R4's, so any plausible pair of epoch
    stamps does here; these are b2maps_k2's t_cmd and a 0.97 RTF over the
    window, which is what that run measured.
    """
    return rh.window_from(sim_start, min_sim, BAG_T_CMD,
                          BAG_T_CMD + min_sim / 0.97, sim_start + min_sim,
                          f'/robot_{robot}/cmd_vel')


FULL_WINDOW = _window(SERIES_LO, SERIES_HI - SERIES_LO)
MID_WINDOW = _window(SERIES_LO + T_CMD_I * PERIOD_NS / 1e9, WINDOW_S)


def _r4(truth, tf_pairs, wheel, anchor=SPAWN0, window=FULL_WINDOW, why=None):
    """report_truth_tf at the PosePublisher rate the synthetic series runs at.

    TRUTH_HZ is the literal;
    test_the_transform_rates_come_from_the_files_that_set_them is what keeps it
    honest against swarm_sim.launch.py.
    """
    return rh.report_truth_tf(0, anchor, truth, tf_pairs, wheel, TRUTH_HZ,
                              window, why)


def test_the_window_is_exactly_min_sim_long_in_nanoseconds():
    """The bound a transform's header stamp is compared against.

    Both ends in integer nanoseconds, because that is what the stamp carries
    and because a float sim_hi would put a transform stamped at the closing
    instant on whichever side the last bit fell.
    """
    w = _window(75.82, 1200.0)                      # b2maps_k2's own window

    assert w['sim_lo_ns'] == 75_820_000_000
    assert w['sim_hi_ns'] - w['sim_lo_ns'] == 1_200_000_000_000
    assert w['sim_hi'] == pytest.approx(1275.82)
    assert w['recv_lo'] == BAG_T_CMD
    # The provenance travels with the window: the topic and both thresholds,
    # so the report can say where t_cmd came from without retyping them.
    assert '/robot_0/cmd_vel' in w['src']
    assert str(rh.CMD_EPS_LINEAR) in w['src']
    assert str(rh.CMD_EPS_ANGULAR) in w['src']


def test_the_window_thresholds_are_check_run_bags_own():
    """Not a second opinion about what "commanded to move" means.

    C3, C4 and R4 have to place their windows on the same command, or R4 gates
    a span the map was not cut to. The import is the mechanism; this is the
    assertion that it is still an import.
    """
    assert rh.first_nonzero_cmd is checker.first_nonzero_cmd
    assert rh.sim_window is checker.sim_window
    assert (rh.CMD_EPS_LINEAR, rh.CMD_EPS_ANGULAR) == (
        checker.CMD_EPS_LINEAR, checker.CMD_EPS_ANGULAR)


def test_a_correct_truth_driven_run_passes_r4_and_r5():
    truth, tf_pairs, wheel = _truth_driven()
    assert _r4(truth, tf_pairs, wheel) is True


def test_the_wrong_anchor_fails_r4():
    """DOT_POSES[1] instead of DOT_POSES[0]: 1.06 m of constant offset, which
    is exactly the silent failure the whole check exists for.
    """
    truth, tf_pairs, wheel = _truth_driven()
    assert _r4(truth, tf_pairs, wheel, anchor=(0.0, 0.90, 0.0)) is False


def test_a_transform_between_two_pose_samples_inside_the_window_fails_r4():
    """THE discriminator, and the one case the window must not weaken.

    Every transform the node publishes carries the pose's own stamp, so a
    transform on any other stamp came from somewhere else -- a surviving
    DiffDrive broadcast being what this guards. Such a publisher runs THROUGH
    the run, so it lands inside the window, where nothing is forgiven.

    Asserted against both windows deliberately: the same fixture fails whether
    the window is the whole series or the 5 s in the middle of it, so the
    change of rule cannot be what makes it fail.
    """
    truth, tf_pairs, wheel = _truth_driven()
    mid = tf_pairs[100][0] + PERIOD_NS // 2      # between samples 100 and 101
    tf_pairs.insert(101, (mid, 0.0, 0.0, 0.0, 0.0))

    assert _r4(truth, tf_pairs, wheel) is False
    assert _r4(truth, tf_pairs, wheel, window=MID_WINDOW) is False


def test_a_tilted_transform_inside_the_window_fails_r4():
    """z must stay 0: base_footprint is the ground-projected frame, and a
    non-planar transform would tilt every scan drawn through it.
    """
    truth, tf_pairs, wheel = _truth_driven()
    ns, x, y, t, _z = tf_pairs[100]
    tf_pairs[100] = (ns, x, y, t, 0.01)
    assert _r4(truth, tf_pairs, wheel) is False
    assert _r4(truth, tf_pairs, wheel, window=MID_WINDOW) is False


def test_the_same_tilt_before_t_cmd_is_reported_not_gated():
    """The mirror of the test above, and the whole change in two lines: one
    perturbation, judged by WHERE it is.

    Sample 5 is sim 1.250, before MID_WINDOW's t_cmd at 2.000. No map is built
    from that time -- the robot has not been commanded to move yet -- so a
    tilted transform there is printed and not gated.
    """
    truth, tf_pairs, wheel = _truth_driven()
    ns, x, y, t, _z = tf_pairs[5]
    tf_pairs[5] = (ns, x, y, t, 0.01)

    assert _r4(truth, tf_pairs, wheel, window=MID_WINDOW) is True
    assert _r4(truth, tf_pairs, wheel) is False      # inside FULL_WINDOW


def test_no_transforms_at_all_fails_r4():
    truth, _tf, wheel = _truth_driven()
    assert _r4(truth, [], wheel) is False


def test_no_truth_poses_at_all_fails_r4():
    """Without a pose series there is nothing to compare against, and nothing
    to define an edge either -- so this cannot be allowed to pass by having an
    empty 'inside' population.
    """
    _truth, tf_pairs, wheel = _truth_driven()
    assert _r4({}, tf_pairs, wheel) is False


# ── outside the window: reported, never gated ────────────────────────────────
# Four of the five B2 runs failed R4 on transforms recorded before their robot
# was ever commanded to move. The count is a property of the recorder, not of
# the run: rosbag2 subscribes to /tf and to /model/robot_K/pose at different
# moments, and whichever wins is a discovery race -- relay1 +142.4 ms (3
# transforms), rehearsal5 -224 ms (none), k0/k1/k2/k3/k4 32, 0, 20, 34 and 28
# periods. Every one of those edges ended before the first command: k0's at sim
# 57.150 against a command at 105.850. No map is built from that time, so
# nothing there is gated now. These tests pin that, and pin what still fails.

def _leading(truth, tf_pairs, n=3, step=PERIOD_NS):
    """n transforms before the pose series, at its own rate, as the recorder's
    attach window produces them. Returned in stream order, edge first.
    """
    first = min(truth)
    lead = [(first - (n - i) * step, 0.0, 0.0, 0.0, 0.0) for i in range(n)]
    return lead + tf_pairs


def test_unmatched_transforms_before_t_cmd_pass_and_are_reported(capsys):
    """k0's shape, which the old rule failed: 32 transforms on no truth stamp,
    ahead of the first recorded pose and well before the first command.
    """
    truth, tf_pairs, wheel = _truth_driven()

    assert _r4(truth, _leading(truth, tf_pairs, n=32), wheel,
               window=MID_WINDOW) is True
    out = capsys.readouterr().out
    assert re.search(r'before t_cmd\s+52 transforms, 32 on no truth stamp',
                     out)
    assert 'never gated: no map is built from this time' in out


def test_a_long_leading_edge_is_no_longer_a_failure():
    """The measured edges of the four runs that failed, one assertion each.

    Under the old 20-period grace, 28, 32 and 34 were failures and 20 was not.
    The length of a recorder's attach window is not a fact about the transform,
    so none of them is a failure now.
    """
    for n in (20, 28, 32, 34):
        truth, tf_pairs, wheel = _truth_driven()
        assert _r4(truth, _leading(truth, tf_pairs, n=n), wheel,
                   window=MID_WINDOW) is True, n


def test_however_many_transforms_before_t_cmd_are_not_gated():
    """An edge as long as the run, and a sparse one reaching a minute back --
    both failures under the old count and span bounds.

    Neither is gated now, and the reason is not that they are believed
    harmless: it is that a transform published before the robot was commanded
    to move cannot reach a map cut from [t_cmd, t_cmd + --min-sim]. A publisher
    that ran DURING the run leaves transforms inside the window, where they
    still fail, and one that ran through the whole bag reads as the 20 + 50 =
    70 Hz sum on the pair, which R2 rejects outright.
    """
    truth, tf_pairs, wheel = _truth_driven()
    assert _r4(truth, _leading(truth, tf_pairs, n=len(tf_pairs)), wheel,
               window=MID_WINDOW) is True

    truth, tf_pairs, wheel = _truth_driven()
    first = min(truth)
    sparse = [(first - n * PERIOD_NS, 0.0, 0.0, 0.0, 0.0)
              for n in (1200, 800, 400)]
    assert _r4(truth, sparse + tf_pairs, wheel, window=MID_WINDOW) is True


def test_out_of_window_transforms_need_not_sit_at_an_end_of_the_stream():
    """The old rule also required the out-of-span transforms to be a contiguous
    prefix or suffix of the RECORDED stream. Stream order is the order rosbag2
    happened to write messages in; it says nothing about which of them a map
    reads. The window does, so contiguity is gone.
    """
    truth, tf_pairs, wheel = _truth_driven()
    first = min(truth)
    mixed = list(tf_pairs)
    for i in range(3):
        mixed.insert(50 * (i + 1), (first - (3 - i) * PERIOD_NS,
                                    0.0, 0.0, 0.0, 0.0))
    assert _r4(truth, mixed, wheel, window=MID_WINDOW) is True


def test_a_trailing_edge_passes_the_same_way():
    """The race can leave the edge at either end -- rehearsal5 subscribed to
    the pose topic FIRST -- and the far end of the bag is past the window in
    any case: relay1's last 4 transforms are after its window closes.
    """
    truth, tf_pairs, wheel = _truth_driven()
    last = max(truth)
    tf_pairs += [(last + (i + 1) * PERIOD_NS, 0.0, 0.0, 0.0, 0.0)
                 for i in range(3)]
    assert _r4(truth, tf_pairs, wheel) is True


def _punch_hole(truth, tf_pairs, lo=100, hi=104):
    """Drop four consecutive samples from BOTH series -- a pose dropout in
    which no transform was published either, which is k1's and k2's shape.
    """
    doomed = set(sorted(truth)[lo:hi])
    for ns in doomed:
        del truth[ns]
    return [row for row in tf_pairs if row[0] not in doomed]


def _drop_poses(truth, lo, hi):
    """Drop poses only, leaving their transforms stranded with nothing to
    match. This is the hole that can fail R4, and only inside the window.
    """
    for ns in sorted(truth)[lo:hi]:
        del truth[ns]


def test_a_gap_in_the_pose_series_is_not_itself_a_failure():
    """R4 judges the transform, not the pose stream. A run that dropped poses
    but whose every transform still matches has nothing for R4 to report.
    """
    truth, tf_pairs, wheel = _truth_driven()
    tf_pairs = _punch_hole(truth, tf_pairs)
    assert rh.truth_gaps(truth, TRUTH_HZ) != []
    assert _r4(truth, tf_pairs, wheel) is True


def test_a_pose_hole_before_t_cmd_passes():
    """k2's shape, which the old rule failed for a reason it never stated
    clearly: a 200 ms hole at sim 61.850 -- 14 s BEFORE its first command --
    withheld the edge grace from 20 leading transforms, and the run failed
    while every compared transform agreed to 0.000e+00 m and 2.5e-14 deg.

    Here: 20 leading transforms, and five poses dropped at samples 5..9
    (sim 1.250 .. 1.450), both before MID_WINDOW's t_cmd at 2.000.
    """
    truth, tf_pairs, wheel = _truth_driven()
    _drop_poses(truth, 5, 10)
    assert rh.truth_gaps(truth, TRUTH_HZ) != []

    assert _r4(truth, _leading(truth, tf_pairs, n=20), wheel,
               window=MID_WINDOW) is True


def test_the_same_pose_hole_inside_the_window_fails():
    """The pair that makes the rule, one hole in two places.

    Identical perturbation to the test above -- five poses dropped, their
    transforms left stranded -- moved to samples 60..64 (sim 4.000 .. 4.200),
    inside MID_WINDOW. Those five transforms now sit on no truth stamp in the
    span the map is built from, which is the one thing R4 fails on.
    """
    truth, tf_pairs, wheel = _truth_driven()
    _drop_poses(truth, 60, 65)

    assert _r4(truth, _leading(truth, tf_pairs, n=20), wheel,
               window=MID_WINDOW) is False


def test_a_pose_with_no_transform_is_reported_not_gated(capsys):
    """relay1 has one, at sim 106.050. A transform that was never published is
    a question about the RATE, which R2 answers against /clock -- but it must
    not be invisible.
    """
    truth, tf_pairs, wheel = _truth_driven()
    dropped = tf_pairs.pop(60)
    assert dropped[0] in truth
    assert _r4(truth, tf_pairs, wheel) is True
    out = capsys.readouterr().out
    assert re.search(r'poses with no transform\s+1', out)


def test_the_window_and_its_provenance_are_named_in_the_report(capsys):
    """A reader has to be able to see WHICH span was judged, and on what
    grounds it starts where it does -- otherwise the verdict is unfalsifiable.
    """
    truth, tf_pairs, wheel = _truth_driven()
    _r4(truth, _leading(truth, tf_pairs, n=32), wheel, window=MID_WINDOW)
    out = capsys.readouterr().out

    assert re.search(r'analysis window\s+sim \[2\.000, 7\.000\]', out)
    assert '--min-sim' in out
    assert re.search(rf't_cmd\s+2\.000 s sim, bag receive {BAG_T_CMD:.6f}',
                     out)
    assert '/robot_0/cmd_vel' in out
    assert 'check_run_bag.first_nonzero_cmd' in out
    # The three populations, each with its own count.
    assert re.search(r'compared\s+101\s+transforms', out)
    assert re.search(r'before t_cmd\s+52 transforms', out)
    assert re.search(r'after the window\s+79 transforms', out)


def test_split_at_window_buckets_by_the_window(capsys):
    """The partition itself, without the report around it. Both bounds are
    inclusive: a transform stamped at the closing instant of the window is
    inside the span the map is cut to.
    """
    rows = [(50, 0, 0, 0, 0), (100, 0, 0, 0, 0), (150, 0, 0, 0, 0),
            (200, 0, 0, 0, 0), (250, 0, 0, 0, 0)]
    before, inside, after = rh.split_at_window(rows, 100, 200)
    assert [i for i, _r in before] == [0]
    assert [i for i, _r in inside] == [1, 2, 3]
    assert [i for i, _r in after] == [4]


def test_without_a_window_r4_is_not_evaluated_rather_than_passed(capsys):
    """A bag too short to place the window, or one with no command in it, has
    not been judged. It must not read as a pass -- that is what put four runs'
    R4 verdicts in doubt in the first place -- and it must not read as a
    failure of the transform either.
    """
    truth, tf_pairs, wheel = _truth_driven()
    got = _r4(truth, tf_pairs, wheel, window=None,
              why='/clock never advances 1200 s of sim after the first command')

    assert got is None
    out = capsys.readouterr().out
    assert 'NO ANALYSIS WINDOW' in out
    assert 'never advances 1200 s' in out
    assert 'NOT gated' in out
    assert '-> NOT EVALUATED' in out


def test_a_failing_r5_still_fails_without_a_window():
    """NOT EVALUATED must not become a place for a real failure to hide. R5
    needs no window, so its verdict stands on its own.
    """
    truth, tf_pairs, _wheel = _truth_driven()
    same_as_truth = [(ns, t) for ns, _x, _y, t, _z in tf_pairs]
    assert _r4(truth, tf_pairs, same_as_truth, window=None,
               why='no /clock') is False


# ── the exit code ────────────────────────────────────────────────────────────

def test_not_evaluated_is_its_own_exit_code():
    """Three outcomes, three codes. A skip that exited 0 would let a script
    chaining on the status treat a bag that could not be judged as a green one.
    """
    assert rh.summarise({'R4/R5': True, 'R9': True}) == 0
    assert rh.summarise({'R4/R5': None, 'R9': True}) == 3
    assert rh.summarise({'R4/R5': False, 'R9': True}) == 1
    # A failure outranks a skip: 1 is the louder and truer answer.
    assert rh.summarise({'R4/R5': None, 'R9': False}) == 1
    # No gate ran at all is not a pass either way -- but an empty result set
    # cannot happen from main(), which requires --log or --bag.
    assert rh.summarise({}) == 0


def test_truth_gaps_finds_only_a_missing_sample():
    truth = {i * PERIOD_NS: (0.0, 0.0, 0.0) for i in range(10)}
    assert rh.truth_gaps(truth, TRUTH_HZ) == []
    del truth[5 * PERIOD_NS]
    assert len(rh.truth_gaps(truth, TRUTH_HZ)) == 1


def test_wheel_odometry_that_is_really_truth_fails_r5():
    """If /robot_K/odom ever became a copy of the truth, the run would prove
    nothing about odometry and arm D could not be built from it. Same
    discriminator as rewrite_odom_from_truth's C4, pointed the other way.
    """
    truth, tf_pairs, _wheel = _truth_driven()
    same_as_truth = [(ns, t) for ns, _x, _y, t, _z in tf_pairs]
    assert _r4(truth, tf_pairs, same_as_truth) is False


def test_a_missing_wheel_stream_fails_r5():
    truth, tf_pairs, _wheel = _truth_driven()
    assert _r4(truth, tf_pairs, []) is False


# ── R2: the frame pair's rate names its owner ────────────────────────────────
# FACTS is the literal pair, in the shape report_tf_rates consumes, so these
# tests need no file at all; test_the_transform_rates_come_from_the_files_that
# _set_them is what keeps the literals honest against the real sources.

FACTS = {'truth_hz': TRUTH_HZ, 'diffdrive_hz': DIFFDRIVE_HZ,
         'truth_src': 'ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py:163'}
BAG_SIM_SPAN = 200.0

# (messages, transforms) as b2maps_rehearsal3 actually carries them: 19797 of
# its /tf messages are EMPTY, so the two counts differ by a third and only the
# second one is a transform count.
TF_SEEN = (77629, 57832)


def _series(hz, span_s, t0_ns=1_000_000_000):
    """Stamps in ns for a publisher running at `hz` for `span_s` of sim."""
    step_ns = round(1e9 / hz)
    return [t0_ns + i * step_ns for i in range(round(hz * span_s))]


def test_a_truth_rate_series_reads_the_truth_rate_on_both_denominators():
    stats = rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN)
    assert stats['n'] == 4000
    assert stats['rate_sim'] == pytest.approx(TRUTH_HZ, abs=1e-6)
    assert stats['rate_own'] == pytest.approx(TRUTH_HZ, rel=1e-3)
    assert rh.judge_rate(stats, TRUTH_HZ)[0] is True


def test_a_publisher_that_died_halfway_passes_its_own_span_and_fails_the_bag():
    """The failure the two denominators exist to separate.

    truth_odom_tf stopping at the halfway point leaves a series that is still
    a perfect 20 Hz series -- rate over its OWN span says 20 and proves
    nothing. Against the sim time the bag actually covers it is 10 Hz, and the
    transform was missing for half the run.
    """
    half = _series(TRUTH_HZ, BAG_SIM_SPAN / 2)
    stats = rh.tf_rate(half, BAG_SIM_SPAN)

    # n messages span n-1 intervals, so count/own_span overshoots the true
    # rate by n/(n-1) -- 0.05 % at these counts, and the same convention
    # rate_facts() uses for the measured 50.0 Hz.
    assert stats['rate_own'] == pytest.approx(TRUTH_HZ, rel=1e-3)
    assert stats['rate_sim'] == pytest.approx(TRUTH_HZ / 2, abs=1e-6)
    assert rh.judge_rate(stats, TRUTH_HZ)[0] is False


def test_two_owners_on_one_pair_read_the_sum_and_fail_either_nominal():
    """DiffDrive still broadcasting alongside the truth node: 20 + 50 = 70 Hz,
    which is the case R2 exists to catch and the reason the band is +-20%.
    """
    both = _series(TRUTH_HZ, BAG_SIM_SPAN) + _series(DIFFDRIVE_HZ,
                                                     BAG_SIM_SPAN)
    stats = rh.tf_rate(both, BAG_SIM_SPAN)

    assert stats['rate_sim'] == pytest.approx(TRUTH_HZ + DIFFDRIVE_HZ,
                                              abs=1e-6)
    assert rh.judge_rate(stats, TRUTH_HZ)[0] is False
    assert rh.judge_rate(stats, DIFFDRIVE_HZ)[0] is False


def test_the_two_robots_are_graded_by_different_yardsticks():
    """A parked robot's 50 Hz is healthy; the same 50 Hz on the explorer means
    DiffDrive never stopped publishing. One number, two verdicts -- so the
    nominal has to be chosen per robot, not shared.
    """
    parked = rh.tf_rate(_series(DIFFDRIVE_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN)
    assert rh.judge_rate(parked, DIFFDRIVE_HZ)[0] is True
    assert rh.judge_rate(parked, TRUTH_HZ)[0] is False


def test_the_band_edges_are_exactly_plus_and_minus_the_tolerance():
    ok, lo, hi = rh.judge_rate({'n': 1, 'rate_sim': TRUTH_HZ}, TRUTH_HZ)
    assert (lo, hi) == pytest.approx((16.0, 24.0))
    assert ok is True

    for edge in (lo, hi):
        assert rh.judge_rate({'n': 1, 'rate_sim': edge}, TRUTH_HZ)[0] is True
    for beyond in (lo - 1e-6, hi + 1e-6):
        assert rh.judge_rate({'n': 1, 'rate_sim': beyond},
                             TRUTH_HZ)[0] is False


def test_an_absent_pair_is_a_failure_not_a_division_by_zero():
    stats = rh.tf_rate([], BAG_SIM_SPAN)
    assert stats['n'] == 0
    assert stats['rate_own'] == 0.0 and stats['rate_sim'] == 0.0
    assert rh.judge_rate(stats, TRUTH_HZ)[0] is False


def test_no_transforms_fails_even_if_the_rate_were_in_band():
    """judge_rate's own contract, tested where tf_rate cannot reach it.

    An empty pair always rates 0.0, which is outside every band this script
    forms, so the count guard never fires today. It is what keeps the verdict
    honest if a rate is ever computed some other way: zero transforms is the
    absence of the thing being measured, not a rate that happens to look
    right.
    """
    assert rh.judge_rate({'n': 0, 'rate_sim': TRUTH_HZ}, TRUTH_HZ)[0] is False
    assert rh.judge_rate({'n': 1, 'rate_sim': TRUTH_HZ}, TRUTH_HZ)[0] is True


def test_a_bag_with_no_clock_has_no_sim_rate_at_all():
    """No /clock means no sim-time denominator. The count is still reported,
    but a rate per BAG second is not the rate R2 is about -- RTF varies inside
    a run -- so nothing is judged from it.
    """
    stats = rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), None)
    assert stats['n'] == 4000
    assert stats['rate_sim'] == 0.0
    assert stats['rate_own'] == pytest.approx(TRUTH_HZ, rel=1e-3)


def test_repeated_stamps_are_visible_in_the_distinct_count():
    """Two publishers agreeing on a stamp would double the count without
    moving 'distinct'. Reported, not gated: R4 is what tests stamps.
    """
    stamps = _series(TRUTH_HZ, 1.0)
    stats = rh.tf_rate(stamps + stamps, BAG_SIM_SPAN)
    assert stats['n'] == 40 and stats['distinct'] == 20


def test_the_r2_report_passes_a_truth_explorer_beside_a_diffdrive_control():
    ok = rh.report_tf_rates(
        0, 1,
        rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN),
        rh.tf_rate(_series(DIFFDRIVE_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN),
        FACTS, BAG_SIM_SPAN, TF_SEEN)
    assert ok is True


def test_the_r2_report_fails_when_the_explorer_is_still_on_diffdrive():
    """b2maps_rehearsal3's shape: a bag recorded before truth_odom_tf existed.
    Both robots read 50 Hz, and robot K's 50 is the finding.
    """
    both_diffdrive = rh.tf_rate(_series(DIFFDRIVE_HZ, BAG_SIM_SPAN),
                                BAG_SIM_SPAN)
    assert rh.report_tf_rates(0, 1, both_diffdrive, both_diffdrive,
                              FACTS, BAG_SIM_SPAN, TF_SEEN) is False


def test_the_r2_report_fails_when_the_control_robot_lost_its_transform():
    """The parked robot is a control, so it is gated too: an SDF edit that
    reached every robot instead of just K would leave it silent, and R2 must
    not call that a pass.
    """
    assert rh.report_tf_rates(
        0, 1,
        rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN),
        rh.tf_rate([], BAG_SIM_SPAN),
        FACTS, BAG_SIM_SPAN, TF_SEEN) is False


def test_empty_tf_messages_are_reported_as_such(capsys):
    """b2maps_rehearsal3 carries 77629 /tf messages holding 57832 transforms:
    19797 of them are empty, published at a steady 100.0 Hz. A rate taken off
    `ros2 topic hz /tf` counts those, and is a third too high for it.
    """
    stats = rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN)
    rh.report_tf_rates(0, 1, stats, stats, FACTS, BAG_SIM_SPAN, TF_SEEN)
    out = capsys.readouterr().out

    assert '77629' in out and '57832' in out
    assert '19797' in out and 'NO transform' in out


def test_a_bag_whose_tf_messages_all_carry_a_transform_says_nothing_extra(
        capsys):
    stats = rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), BAG_SIM_SPAN)
    rh.report_tf_rates(0, 1, stats, stats, FACTS, BAG_SIM_SPAN, (4000, 4000))
    assert 'NO transform' not in capsys.readouterr().out


def test_the_r2_report_is_not_evaluated_without_a_clock_span():
    stats = rh.tf_rate(_series(TRUTH_HZ, BAG_SIM_SPAN), None)
    assert rh.report_tf_rates(0, 1, stats, stats, FACTS, None, TF_SEEN) is None


# ── R7: the map out of the bag ───────────────────────────────────────────────
# Written from an OccupancyGrid built by hand, so every pixel, count and metre
# below is checked against arithmetic rather than against another run of the
# same code.

def _load_pgm_extent():
    """pgm_extent.py, loaded the same way as run_health.py above.

    Reading the written PGM back with the project's OWN reader is the point:
    it is the tool that will be pointed at rehearsal 5's map, and a PGM only
    this file can read would be no use.
    """
    for parent in Path(__file__).resolve().parents:
        for cand in (parent / 'pgm_extent.py',
                     parent / 'experiments' / 'analysis' / 'pgm_extent.py'):
            if cand.is_file():
                spec = importlib.util.spec_from_file_location('pgm_extent',
                                                              cand)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
    raise ImportError('pgm_extent.py not found above this test')


pe = _load_pgm_extent()

# One row covering every band an OccupancyGrid can carry: unknown, the free
# edge cases, the undecided middle, and the occupied edge cases.
ONE_OF_EACH = [-1, 0, 25, 26, 64, 65, 100]


def test_the_trinary_mapping_is_the_one_save_map_writes():
    img = rh.grid_to_trinary(ONE_OF_EACH, 7, 1)
    assert img.tolist() == [[205, 254, 254, 205, 205, 0, 0]]


def test_the_undecided_band_is_drawn_as_unknown_but_counted_apart():
    """26..64 renders 205, the same grey as unknown -- so a map full of
    undecided cells looks unexplored. The counts must still tell them apart,
    or 'unknown' silently means two different things.
    """
    stats = rh.map_stats(ONE_OF_EACH, 7, 1, 0.05, 0.0, 0.0)
    assert (stats['occupied'], stats['free']) == (2, 2)
    assert (stats['unknown'], stats['other']) == (1, 2)
    assert stats['total'] == 7
    assert sum((stats['occupied'], stats['free'], stats['unknown'],
                stats['other'])) == stats['total']


def test_image_row_zero_is_the_top_of_the_map_not_the_origin_row():
    """OccupancyGrid row 0 is the LOWEST y; nav2's PGM row 0 is the highest.
    Getting this wrong mirrors every map about the x axis, which looks
    entirely plausible in a viewer.
    """
    grid = [0, 0,          # row 0, at the origin: free
            100, 100]      # row 1, above it: occupied
    img = rh.grid_to_trinary(grid, 2, 2)
    assert img[0].tolist() == [0, 0]        # top of the image = grid row 1
    assert img[-1].tolist() == [254, 254]   # bottom = the origin row


def test_the_known_extent_is_the_box_of_known_cells_in_metres():
    """A 6x5 grid at 0.5 m whose known cells are columns 1..3, rows 2..3.

    Hand arithmetic, edges not centres:
      x [-1.0 + 1*0.5, -1.0 + 4*0.5] = [-0.5, +1.0]   3 cols -> 1.5 m
      y [ 2.0 + 2*0.5,  2.0 + 4*0.5] = [+3.0, +4.0]   2 rows -> 1.0 m
    """
    w, h, res, ox, oy = 6, 5, 0.5, -1.0, 2.0
    g = np.full((h, w), -1, dtype=np.int16)
    g[2, 1:4] = 0        # free
    g[3, 1:4] = 100      # occupied
    box = rh.map_stats(g.ravel().tolist(), w, h, res, ox, oy)['known_box']

    assert (box['cols'], box['rows']) == (3, 2)
    assert box['width_m'] == pytest.approx(1.5)
    assert box['height_m'] == pytest.approx(1.0)
    assert (box['x0'], box['x1']) == pytest.approx((-0.5, 1.0))
    assert (box['y0'], box['y1']) == pytest.approx((3.0, 4.0))


def test_unknown_padding_is_excluded_from_the_extent():
    """The whole reason the extent is reported at all: the grid dimensions
    include unknown padding and say nothing about how much was mapped.
    """
    w, h = 40, 40
    g = np.full((h, w), -1, dtype=np.int16)
    g[20, 20] = 0
    stats = rh.map_stats(g.ravel().tolist(), w, h, 0.05, 0.0, 0.0)

    assert stats['total'] == 1600
    assert stats['known_box']['cols'] == 1
    assert stats['known_box']['width_m'] == pytest.approx(0.05)


def test_an_entirely_unknown_map_reports_no_extent_rather_than_crashing():
    stats = rh.map_stats([-1] * 12, 4, 3, 0.05, 0.0, 0.0)
    assert stats['known_box'] is None
    assert stats['unknown'] == 12


def _asymmetric_grid():
    """A grid with no symmetry left to hide a flip or a transpose: 5 wide,
    3 tall, and every row and column different.
    """
    g = np.full((3, 5), -1, dtype=np.int16)
    g[0, 0] = 100
    g[1, 1:3] = 0
    g[2, 4] = 100
    return g


def test_the_written_pgm_is_the_format_save_map_writes(tmp_path):
    img = rh.grid_to_trinary(_asymmetric_grid().ravel().tolist(), 5, 3)
    out = tmp_path / 'm.pgm'
    rh.write_pgm(out, img)

    raw = out.read_bytes()
    assert raw.startswith(b'P5\n5 3\n255\n')
    assert len(raw) == len(b'P5\n5 3\n255\n') + 15


def test_the_pgm_reads_back_through_the_projects_own_reader(tmp_path):
    img = rh.grid_to_trinary(_asymmetric_grid().ravel().tolist(), 5, 3)
    out = tmp_path / 'm.pgm'
    rh.write_pgm(out, img)

    assert pe.read_pgm(str(out)).tolist() == img.tolist()


def test_the_png_is_the_same_image_as_the_pgm(tmp_path):
    """Two files, one array. If they ever diverge, the PNG people look at is
    not the PGM nav2 loads.
    """
    from PIL import Image

    img = rh.grid_to_trinary(_asymmetric_grid().ravel().tolist(), 5, 3)
    rh.write_pgm(tmp_path / 'm.pgm', img)
    rh.write_png(tmp_path / 'm.png', img)

    png = np.asarray(Image.open(tmp_path / 'm.png'))
    assert png.tolist() == img.tolist()
    assert png.tolist() == pe.read_pgm(str(tmp_path / 'm.pgm')).tolist()


def test_pgm_extent_agrees_with_map_stats_about_the_same_map(tmp_path):
    """The independent check that will be run on rehearsal 5's map.

    pgm_extent reads the written PGM and knows nothing of the OccupancyGrid;
    map_stats reads the grid and never sees the image. They must agree on the
    counts and on the known box, or one of the two is wrong about the map.
    """
    w, h, res, ox, oy = 6, 5, 0.5, -1.0, 2.0
    g = np.full((h, w), -1, dtype=np.int16)
    g[2, 1:4] = 0
    g[3, 1:4] = 100
    stem = tmp_path / 'm'
    img = rh.grid_to_trinary(g.ravel().tolist(), w, h)
    rh.write_pgm(Path(f'{stem}.pgm'), img)
    rh.write_map_yaml(Path(f'{stem}.yaml'), 'm.pgm', res, ox, oy, 0.0)

    stats = rh.map_stats(g.ravel().tolist(), w, h, res, ox, oy)
    px = pe.read_pgm(f'{stem}.pgm')
    got_res, got_origin = pe.read_yaml(f'{stem}.yaml')

    assert (got_res, got_origin) == (res, [ox, oy, 0.0])
    assert int((px == 0).sum()) == stats['occupied']
    assert int((px == 254).sum()) == stats['free']
    assert int((px == 205).sum()) == stats['unknown'] + stats['other']

    # box() returns (cols, rows, width_m, height_m, x0, x1, y0, y1) in image
    # coordinates; the metres must match whichever way the rows are counted.
    cols, rows, wm, hm, x0, x1, y0, y1 = pe.box(px != 205, res, got_origin, h)
    b = stats['known_box']
    assert (cols, rows) == (b['cols'], b['rows'])
    assert (wm, hm) == pytest.approx((b['width_m'], b['height_m']))
    assert (x0, x1, y0, y1) == pytest.approx((b['x0'], b['x1'], b['y0'],
                                              b['y1']))


def test_a_bag_with_no_map_message_fails_r7(tmp_path):
    """map_saver_cli's failure mode, reported as a verdict instead of an
    exception: the run published no map, so there is nothing to write.
    """
    assert rh.report_map(0, None, tmp_path / 'unused') is False
    assert not list(tmp_path.iterdir())


def test_r7_writes_all_three_files_and_passes(tmp_path):
    w, h = 6, 5
    g = np.full((h, w), -1, dtype=np.int16)
    g[2, 1:4] = 0
    g[3, 1:4] = 100
    found = {'count': 85, 'recv_s': 12.0, 'stamp_s': 210.5, 'frame_id': 'map',
             'w': w, 'h': h, 'res': 0.5, 'ox': -1.0, 'oy': 2.0, 'yaw': 0.0,
             'data': g.ravel().tolist()}
    stem = tmp_path / 'maps' / 'rehearsal5_robot0'

    assert rh.report_map(0, found, stem) is True
    for ext in ('pgm', 'png', 'yaml'):
        assert Path(f'{stem}.{ext}').is_file(), ext
