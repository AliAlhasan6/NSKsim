#!/usr/bin/env python3
"""run_health.py -- two gates on an explorer run: TF timing, and coverage.

Why this exists (2026-09-20):

  R9, TF TIMING. Robot K's odom -> base_footprint is moving out of Gazebo and
  into a ROS node (nsk_swarm/truth_odom_tf.py). That changes both the rate and
  the path the transform takes to reach slam_toolbox and Nav2:

      before  DiffDrive, 50 Hz      gz -> ros_gz bridge -> /tf
      after   truth_odom_tf, 20 Hz  gz -> bridge -> node -> /tf

  50, not the 30 the burger SDF appears to ask for: that line misspells the
  element and has never taken effect. rate_facts() says so at runtime and
  explains how it was established; b2maps_e0 measured 50.0 Hz.

  A slower transform published one process hop further away is exactly the
  kind of change that shows up as transform-timeout warnings rather than as
  anything obvious, and this rig is already known to starve under load: RViz
  alone once collapsed /clock from 99 Hz to 0.5 Hz. So the warnings are
  counted -- and, since 2026-09-28, gated on WHERE they fell rather than on how
  many there were: a consumer that could not get a transform inside the window
  the map is built from fails the run, and the same warning outside it is
  reported. Two exclusions, both because the product maps are built offline
  from the stripped bag: slam_toolbox's own message-filter drops, and warnings
  naming its map <-> odom output. See WINDOW_GATED for the rule and VS_BASELINE
  for why the old rate-against-a-baseline comparison is now advisory.

  COVERAGE. b2maps_e0's explorer declared exploration complete having never
  seen the outer boundary. "It reached the boundary" is only a criterion if it
  is a number decided before the run, so: robot K's TRUE trajectory must pass
  within the lidar's own maximum range of an outer boundary wall at least
  once. Both quantities are read from the files that define them, never
  retyped.

  Neither gate uses the SLAM map. They read the explorer's log and Gazebo's
  ground truth, so a run that maps badly cannot flatter itself on either.

Usage (ROS sourced only for --bag; --log needs nothing):
    python3 experiments/analysis/run_health.py --log <explore.log>
    python3 experiments/analysis/run_health.py --bag <bag dir> --robot 0
    python3 experiments/analysis/run_health.py --log L --bag B --robot 0 \
        [--baseline-log experiments/logs/b2maps/b2maps_e0_explore.log]
    python3 experiments/analysis/run_health.py --bag B --robot 0 --tf-rates \
        [--parked 1]
    python3 experiments/analysis/run_health.py --bag B --robot 0 \
        --save-map experiments/maps/<name>

Exit status:

    0  every gate that ran passed, and none was skipped
    1  a gate FAILED. Also what bag_overlap.die() exits with, so a bad input
       and a bad run share a code; the output says which.
    2  argparse rejected the command line
    3  nothing failed, but a gate could not run. NOT EVALUATED is not a pass:
       a bag too short to place the analysis window, or a log with no bag
       beside it, must not be read as green by anything chaining on the
       status.

A gate whose input was not given does not run at all and does not affect the
status -- that is different from a gate that was asked for and could not be
evaluated, which is the 3 above.
"""

from __future__ import annotations

import argparse
import bisect
import math
import re
import statistics
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

from bag_overlap import (  # noqa: E402
    NUM_ROBOTS,
    WORLD_SDF,
    die,
    dist_to_nearest_wall,
    resolve_spawn_poses,
    world_walls_named,
)

# The analysis window is check_run_bag.py's window, imported rather than
# rebuilt. first_nonzero_cmd defines what "commanded to move" means and
# sim_window defines how a budget of SIM seconds after that command maps back
# onto the bag's receive clock; C3 and C4 place their own windows with exactly
# these two, and the cut that produces the offline map is C4's --start /
# --duration. Reimplementing either here is how R4 would come to judge a
# different span from the one the map is built from.
#
# Safe at module scope: check_run_bag imports only yaml and bag_overlap up
# front and keeps rosbag2_py inside the functions that read a bag, so the log
# half of this script still runs without a sourced ROS.
from check_run_bag import (  # noqa: E402
    CMD_EPS_ANGULAR,
    CMD_EPS_LINEAR,
    _ClockTrace,
    first_nonzero_cmd,
    read_clock_series,
    read_prologue_series,
    sim_window,
)

BURGER_SDF = Path('/opt/ros/jazzy/share/turtlebot3_gazebo/models/'
                  'turtlebot3_burger/model.sdf')
LAUNCH_FILE = (REPO_ROOT / 'ros2_ws' / 'src' / 'nsk_swarm' / 'launch'
               / 'swarm_sim.launch.py')

# The four outer boundary models in knowledge_world.sdf. Named rather than
# selected by size: a geometric predicate ("the long ones") would silently
# pick up a maze wall if the world were ever edited, and this criterion is
# specifically about the perimeter the e0 explorer never reached.
BOUNDARY_WALLS = ('wall_north', 'wall_south', 'wall_east', 'wall_west')

# gz-sim 8's DiffDrive default, used when the model sets no
# <odom_publish_frequency> -- which the stock burger does not; see rate_facts().
# Corroborated by measurement, not taken on faith: b2maps_e0's /robot_0/odom
# runs at 50.0 Hz against its own /clock.
DIFFDRIVE_DEFAULT_HZ = 50.0

# How far above baseline a counted warning rate may go before the run fails.
# Set by the amendment that asked for this script; not a measured threshold.
FAIL_FACTOR = 2.0

# ── R2: how far a frame pair's transform rate may sit from its nominal ───────
# The band has to be wide enough for jitter and narrow enough to exclude the
# rate a SECOND publisher would produce, because that is the whole question R2
# answers. The nearest confusable case is two owners on one pair, which reads
# the SUM of their rates: 20 + 50 = 70 Hz. Against the parked robot's 50 Hz
# nominal, +-20% puts the limit at 60 Hz -- exactly halfway between the one
# legitimate rate and the two-owner sum. Against robot K's 20 Hz nominal the
# band is [16, 24] and every alternative is far outside it: 50 Hz if
# truth_odom_tf never started, 70 Hz if the DiffDrive tf_topic redirect in
# make_namespaced_burger_sdf failed to take.
TF_RATE_TOL = 0.20

# ── R4: the span of the run a map is actually built from ─────────────────────
# Offline mapping replays from the first nonzero command onward, for the number
# of SIM seconds the recording rule fixed. Before that command the robot is
# sitting at spawn; after the cut is simulation no map ever reads. So the
# window [t_cmd, t_cmd + MIN_SIM] in sim time is the only span whose transforms
# a map can be wrong about, and it is what R4 gates.
#
# 1200 s is the b2maps rule and the longest of its four cuts
# (experiments/runs/b2maps/README.md). --min-sim is the same knob as
# check_run_bag.py's, by the same name and with the same meaning, so the two
# scripts can be handed the same number and judge the same span. Only the
# DEFAULT differs: check_run_bag's is still its original 2400 and the b2maps
# README tells people to pass 1200 to it explicitly. This default IS that
# instruction, not a second knob.
#
# What this replaces, and why. R4 used to gate every recorded transform from
# the first one in the bag, forgiving 20 PosePublisher periods at each end. The
# leading edge is not a property of the run: it is the gap between rosbag2
# subscribing to /tf and to /model/robot_K/pose. b2maps_relay1's record log --
#
#     [1789943096.804587] Subscribed to topic '/tf'
#     [1789943096.947029] Subscribed to topic '/model/robot_0/pose'
#
# 142.4 ms, which at 20 Hz is 2.85 sample periods, and exactly 3 transforms in
# that bag carry stamps (75.250, 75.300, 75.350) earlier than the first
# recorded pose (75.400). The race swings both ways and its width is arbitrary:
# rehearsal5 -224 ms (pose first, no edge at all), e0t +44.6 ms, relay1
# +142.4 ms, b2maps_k0..k4 0, 20, 28, 32 and 34 periods, e0 +2824 ms. Gating on
# that gated the recorder. Worse, a pose hole anywhere in the series withheld
# the grace entirely, so k2 failed on a 200 ms hole at sim 61.850 -- 14 s
# before its first command, in a run whose every compared transform agreed
# to 0.000e+00 m and 2.5e-14 deg.
#
# Nothing is forgiven inside the window now, and nothing is gated outside it.
# The case R4 exists for -- a surviving DiffDrive broadcast -- publishes
# THROUGH the run, so it lands inside, where a transform on no truth stamp is
# still a hard failure. A publisher confined to the time before the first
# command cannot reach a map; and R2 catches one that ran the whole bag by
# reading the 70 Hz sum on the pair.
MIN_SIM_S = 1200.0

# A truth sample is MISSING, rather than merely jittered, when the step to the
# next one exceeds this many nominal periods. b2maps_relay1's pose series steps
# 50.000 ms every time -- median, min and max all equal -- so this only has to
# separate "one sample absent" (2.0) from "no sample absent" (1.0).
TRUTH_GAP_PERIODS = 1.5

# ── R7: the trinary thresholds nav2's map_saver uses ─────────────────────────
# Percent occupancy, matching experiments/slam/save_map.py's OCC_TH/FREE_TH
# (0.65/0.25 there, scaled by 100 at the comparison). Restated in this file
# rather than imported: save_map.py imports rclpy at module scope, so importing
# it would drag a sourced ROS into the pure half of this script. The pixel
# values are nav2's too, and PGM_* is what makes a map written from a bag
# interchangeable with one saved live.
OCC_TH_PCT, FREE_TH_PCT = 65, 25
PGM_OCC, PGM_FREE, PGM_UNKNOWN = 0, 254, 205

# ── what gets counted ────────────────────────────────────────────────────────
# Every pattern below was confirmed to exist in an installed library on this
# machine (strings -a over /opt/ros/jazzy/lib/*.so*), so none of them is a
# phrase that can never match:
#
#   'Timed out waiting for transform'     libnav2_costmap_2d_core.so, libtf2_ros.so
#   'Transform data too old'              libtf_help.so  (nav2_util, used by
#                                         controller_server -- the exact string
#                                         slam_robot0.yaml's transform_timeout
#                                         comment cites)
#   'Lookup would require extrapolation'  libtf2.so
#   'extrapolation into the'              libtf2.so
#   'Message Filter dropping message'     libtoolbox_common.so (slam_toolbox)
#                                         and liblayers.so (costmap)
#   'Failed to make progress'             libcontroller_server_core.so
#
# 'Message Filter dropping message' is emitted by BOTH slam_toolbox and the
# costmap layers, and the two mean entirely different things here, so the line
# has to say which. It is identified by the LOGGER -- [robot_3.slam_toolbox] --
# and not by the wording and not by the process tag:
#
#   [async_slam_toolbox_node-2] [INFO] [...] [robot_3.slam_toolbox]:
#       Message Filter dropping message: frame 'robot_3/base_scan' ...
#   [planner_server-5] [INFO] [...] [robot_3.global_costmap.global_costmap]:
#       Message Filter dropping message: frame 'robot_3/base_scan' ...
#
# Matching on the wording alone would put those in one bucket; matching on the
# substring 'slam_toolbox' anywhere in the line would also accept the process
# tag, which is a weaker claim about who dropped the message. SLAM_LOGGER_RE
# matches the logger field and nothing else -- [robot_3.slam_toolbox] and a
# bare [slam_toolbox], never [async_slam_toolbox_node-2].
SLAM_LOGGER_RE = re.compile(r'\[(?:[A-Za-z0-9_.]+\.)?slam_toolbox\]')

# Compared against the baseline run's rates, and REPORTED. Nothing here is
# gated on the comparison: which way it goes depends entirely on which log is
# handed to --baseline-log, and the b2maps corpus makes that concrete. relay1's
# 0.107 transform warnings/min is a FAIL against e0 (0 in 98.96 min, limit
# 0.000), a PASS against e0t (0.063/min, limit 0.126) and a FAIL again against
# rehearsal5 (0). One run, three verdicts, none of them about the run. The
# comparison is still worth printing -- "is this worse than the run it
# replaces?" is a real question -- so it stays, labelled advisory.
VS_BASELINE = ('nav2_transform', 'slam_drop', 'other_drop')

# Gated, and only inside the analysis window. See WINDOW_GATE_NOTE.
WINDOW_GATED = ('nav2_transform', 'other_drop')

COUNTS = {
    'nav2_transform': dict(
        label='Nav2/tf2 transform warnings',
        any_of=('Timed out waiting for transform', 'Transform data too old',
                'Lookup would require extrapolation', 'extrapolation into the'),
        require=None, exclude=None),
    'slam_drop': dict(
        label='message-filter drops, slam_toolbox',
        any_of=('Message Filter dropping message',),
        require=SLAM_LOGGER_RE, exclude=None),
    'other_drop': dict(
        label='message-filter drops, any other logger',
        any_of=('Message Filter dropping message',),
        require=None, exclude=SLAM_LOGGER_RE),
    'no_progress': dict(
        label='controller "Failed to make progress"',
        any_of=('Failed to make progress',),
        require=None, exclude=None),
    'loop_rate': dict(
        label='control/planner loop missed its rate',
        any_of=('missed its desired rate',),
        require=None, exclude=None),
}

# 'no_progress' is a navigation outcome rather than a timing fault, and
# 'loop_rate' is a load symptom worth seeing next to the others on a rig with
# this one's history. Neither is gated on any terms.
assert set(VS_BASELINE) <= set(COUNTS)
assert set(WINDOW_GATED) <= set(COUNTS)

# ── R9: which counted lines decide the run ───────────────────────────────────
# A counted line is GATED when all three hold:
#
#   1. its category is in WINDOW_GATED,
#   2. its stamp is inside the analysis window -- Part 1's window, the span the
#      map is built from, reused rather than recomputed, and
#   3. it does not name robot_K/map <-> robot_K/odom.
#
# Everything else is reported. The two exclusions are the same exclusion twice:
# that pair IS online slam_toolbox's output, and so is its message-filter
# queue, and no offline map reads either. The product maps are built by
# run_offline_maps.sh from the stripped bag, so a stale online map -> odom
# estimate cannot reach them.
#
# What that leaves gated is every consumer that could not get a transform it
# needed, inside the window, about any other link -- robot_K/odom ->
# robot_K/base_footprint above all, the transform truth_odom_tf publishes and
# this whole change is about. The corpus shows the distinction is real and that
# pooling destroyed it: all 8 'Transform data too old' lines in it (e0t's 7,
# k2's 1) name map <-> odom and were traced to slam_toolbox's estimate running
# ~0.55 s stale, while the one line naming the transform under test is relay1's
# collision_monitor extrapolation error at sim 463. One count for both let
# warnings about a transform this change does not touch set the budget for the
# one it does.
#
# A line whose pair cannot be parsed FAILS CLOSED: it gates, and it is printed
# in full so the phrasing can be read and, if it is a real format, added to
# PAIR_RES. Silently excusing an unrecognised warning is how a gate stops
# being one.
#
# That is the whole rule for a transform WARNING, which names the pair it could
# not get. A message-filter DROP names no pair -- it names a frame and an
# instant -- so the three conditions above make every drop a candidate and
# nothing more. What decides a drop is whether a link in the chain from the
# costmap's global_frame down to the dropped message's frame actually failed to
# supply a transform in time, which is a question for the bag and not the log:
# see drop_verdict() and experiments/analysis/attribute_tf_drop.py.
DROP_CATS = ('slam_drop', 'other_drop')
#
# The frame pair is taken from the line by precedence, because a single warning
# can name two pairs. relay1's, in full:
#
#   [collision_monitor-11] [ERROR] [...] [getTransform]: Failed to get
#   "robot_0/base_scan"->"robot_0/base_footprint" frame transform: Lookup would
#   require extrapolation into the future. Requested time 463.070000 but the
#   latest data is at time 463.050000, when looking up transform from frame
#   [robot_0/odom] to frame [robot_0/base_footprint]
#
# The quoted pair is what the node ASKED for; tf2's own trailing detail is the
# link whose lookup actually failed, and that is the one worth gating on --
# here robot_0/odom -> robot_0/base_footprint, truth_odom_tf's. So tf2's
# bracket form wins whenever it is present.
PAIR_RES = (
    # libtf2.so, tf2's extrapolation detail: the link that failed.
    re.compile(r'looking up transform from frame \[([^\]]+)\] to frame '
               r'\[([^\]]+)\]'),
    # libtf_help.so (nav2_util getTransform): the transform asked for.
    re.compile(r'"([^"]+)"->"([^"]+)" frame transform'),
    # libtf_help.so: 'Transform data too old when converting from A to B'.
    re.compile(r'converting from (\S+) to (\S+)'),
    # libnav2_costmap_2d_core.so: 'Timed out waiting for transform from A to B
    # to become available ...'.
    re.compile(r'waiting for transform from (\S+) to (\S+)'),
)

# <ns>/map and <ns>/odom, one common namespace, either direction. The namespace
# has to MATCH: robot_0/map <-> robot_1/odom is a cross-robot link and a real
# fault, and it must not be excused by a substring hit on 'map'. A pair with no
# namespace at all does not match either, and so gates -- every frame in this
# stack is namespaced (all 9 real lines, and slam_robot0.yaml), so an
# unnamespaced one is not a case that has been reasoned about.
MAP_ODOM_RE = re.compile(r'^(.+)/(map|odom)$')

# The frame and the instant a dropped message carried, from tf2_ros's one
# format string (libtf2_ros): "Message Filter dropping message: frame '%s' at
# time %.3f for reason '%s'".
DROP_RE = re.compile(r"dropping message: frame '([^']+)' at time "
                     r'([0-9]+\.[0-9]+)')

STAMP_RE = re.compile(r'\[(1[0-9]{9}\.[0-9]+)\]')
PROC_RE = re.compile(r'^\[([A-Za-z0-9_]+)-[0-9]+\]')


# ─────────────────────────── facts, read from source ─────────────────────────

def _grep_one(path: Path, pattern: str, what: str,
              required: bool = True) -> tuple[str, int]:
    """First capture of `pattern` in `path`, with its 1-based line number.

    Dies rather than defaulting: every number this script reports has to come
    from the file that defines it, so a template that stopped matching must be
    loud. A silent default is how a criterion drifts away from the world it
    claims to describe.

    required=False returns (None, None) instead, for the one case where
    ABSENCE is the finding rather than an error -- see rate_facts().
    """
    if not path.is_file():
        die(f'cannot read {what}: {path} does not exist')
    for n, line in enumerate(path.read_text().splitlines(), start=1):
        m = re.search(pattern, line)
        if m:
            return m.group(1), n
    if not required:
        return None, None
    die(f'no {what} found in {path} (pattern {pattern!r})')


def lidar_max_range() -> tuple[float, str]:
    """The lidar ceiling the sim ACTUALLY RUNS, in metres, and its source.

    NOT the stock model's <range><max>. swarm_sim.launch.py's
    make_namespaced_burger_sdf rewrites that element into every spawned robot
    (BURGER_STOCK_RANGE_MAX -> BURGER_RANGE_MAX), so the stock file states the
    range of a sensor no robot in this sim carries.

    Reading the stock file is exactly how this criterion would drift away from
    the world it claims to describe while still printing a number. _grep_one
    dies only when a pattern is ABSENT, and <max>3.5</max> is still there in
    the stock model -- untouched, findable, and wrong. COVERAGE would go on
    testing against 3.5 m while the robots saw 8 m, silently, and in the
    direction that makes the gate EASIER to pass: a criterion that got weaker
    and said nothing about it is worse than one that broke.

    So the effective value is read from the launch file that sets it, the way
    rate_facts() reads PosePublisher's update_frequency out of the same file.
    Three things are checked, because the constant alone does not prove the
    rewrite happens:

      1. the stock model still has a <range><max> to rewrite,
      2. BURGER_RANGE_MAX is defined, and
      3. it is WIRED INTO the replacement list -- the f-string that builds the
         new element must be there, or the constant is a number the spawn path
         never reads.

    The stock value travels in the returned source string beside the effective
    one. The pair is the evidence: a launch file that stopped rewriting the
    sensor shows up here as the two numbers being equal, which is legible,
    rather than as the effective number quietly becoming the stock one.
    """
    stock, stock_line = _grep_one(
        BURGER_SDF, r'<max>([0-9.]+)</max>', 'the stock lidar maximum range')
    effective, eff_line = _grep_one(
        LAUNCH_FILE, r'^BURGER_RANGE_MAX\s*=\s*([0-9.]+)',
        'the lidar maximum range the sim runs (BURGER_RANGE_MAX in the launch '
        'file)')
    _grep_one(
        LAUNCH_FILE, r"(f'<max>\{BURGER_RANGE_MAX\}</max>')",
        'the SDF rewrite that applies BURGER_RANGE_MAX to the spawned model. '
        'The constant is defined but nothing in make_namespaced_burger_sdf '
        'uses it, so the robots would still spawn with the stock sensor while '
        'this gate reported the new range')

    src = (f'{LAUNCH_FILE.relative_to(REPO_ROOT)}:{eff_line}; '
           f'stock {float(stock):.2f} m at {BURGER_SDF.name}:{stock_line}')
    return float(effective), src


def rate_facts() -> dict:
    """The two transform rates this change trades between.

    DiffDrive's odometry AND its TF are emitted from one UpdateOdometry under
    one period, so this is the rate of the transform being replaced, not merely
    of the /robot_N/odom topic.

    The number in the stock burger SDF is NOT that rate. It writes

        <odom_publisher_frequency>30</odom_publisher_frequency>   (line 394)

    and the element gz-sim's DiffDrive actually reads is
    <odom_publish_frequency> -- 'publish', not 'publisher'. Confirmed both
    ways on this machine: `strings` over
    libgz-sim8-diff-drive-system.so.8.15.0 contains odom_publish_frequency and
    does NOT contain odom_publisher_frequency. Unknown SDF elements are
    ignored silently, so the 30 has never taken effect and DiffDrive has always
    run at its own default of 50 Hz.

    Measured, not inferred: b2maps_e0 carries 290035 /robot_0/odom messages
    over 5801.1 s of sim time (from its own /clock), which is 50.0 Hz. Its
    116012 /model/robot_0/pose messages over the same span are 20.0 Hz.

    So the honest statement of what this change does to the transform rate is
    50 Hz -> 20 Hz, a factor of 2.5, not 30 -> 20. Both numbers are returned:
    'nominal' is what the file says, 'diffdrive_hz' is what runs.
    """
    # The element DiffDrive reads, if the model ever starts using it. Absence
    # is the finding here, not an error, so this one may come back empty.
    effective, _eff_line = _grep_one(
        BURGER_SDF,
        r'<odom_publish_frequency>([0-9.]+)</odom_publish_frequency>',
        'DiffDrive odom_publish_frequency', required=False)
    ignored = effective is None
    if ignored:
        effective = DIFFDRIVE_DEFAULT_HZ

    nominal, nom_line = _grep_one(
        BURGER_SDF, r'<odom_publisher_frequency>([0-9.]+)</odom_publisher',
        "the burger SDF's (misspelled, inert) odom_publisher_frequency")

    truth, truth_line = _grep_one(
        LAUNCH_FILE, r'<update_frequency>([0-9.]+)</update_frequency>',
        'PosePublisher ground-truth update frequency')
    return {
        'diffdrive_hz': float(effective),
        'diffdrive_nominal_hz': float(nominal),
        'diffdrive_ignored': ignored,
        'diffdrive_src': f'{BURGER_SDF}:{nom_line}',
        'truth_hz': float(truth),
        'truth_src': f'{LAUNCH_FILE.relative_to(REPO_ROOT)}:{truth_line}',
    }


def boundary_rects() -> list[tuple[float, float, float, float]]:
    """The four outer boundary walls, from knowledge_world.sdf."""
    by_name = dict(world_walls_named())
    missing = [n for n in BOUNDARY_WALLS if n not in by_name]
    if missing:
        die(f'{WORLD_SDF} has no model(s) {missing}; the outer boundary '
            'cannot be identified, so the coverage criterion has no target')
    return [by_name[n] for n in BOUNDARY_WALLS]


# ────────────────────────────── R9: log counting ─────────────────────────────

def stamp_of(line: str) -> float | None:
    """The ROS log timestamp on a line, or None.

    These are WALL-clock epoch seconds -- a log has no /clock -- so the rates
    below are per minute of wall time. That is the right denominator here:
    transform timeouts are a question of whether the transform arrived in time
    on the machine, not of how much simulation happened.
    """
    m = STAMP_RE.search(line)
    return float(m.group(1)) if m else None


def frame_pair(line: str) -> str | None:
    """'A -> B', the frame pair the line names, or None. Pure.

    By precedence over PAIR_RES -- see it for why a line can name two pairs and
    which one wins. None means no phrasing matched, which is not a shrug: the
    caller gates on it.
    """
    for rx in PAIR_RES:
        m = rx.search(line)
        if m:
            a, b = (g.strip().rstrip('.,') for g in m.groups())
            return f'{a} -> {b}'
    return None


def dropped_message(line: str):
    """(frame, stamp in sim seconds) of the message a filter dropped, or
    (None, None). Pure.

    tf2_ros's MessageFilter writes one format, from libtf2_ros:
        Message Filter dropping message: frame '%s' at time %.3f for reason ...
    so a drop names the frame it could not transform and the instant it needed.
    Attribution needs both; attribute_tf_drop.py takes them from here rather
    than parsing the line a second time.
    """
    m = DROP_RE.search(line)
    return (m.group(1), float(m.group(2))) if m else (None, None)


def is_slam_output_pair(pair: str | None) -> bool:
    """True for <ns>/map <-> <ns>/odom, in either direction. Pure.

    That transform is online slam_toolbox's output. A warning about it says the
    online SLAM estimate was stale, which the offline maps do not read -- the
    same reason slam_toolbox's own message-filter drops are advisory.
    """
    if pair is None:
        return False
    a, _, b = pair.partition(' -> ')
    ma, mb = MAP_ODOM_RE.match(a), MAP_ODOM_RE.match(b)
    if not (ma and mb):
        return False
    return (ma.group(1) == mb.group(1)
            and {ma.group(2), mb.group(2)} == {'map', 'odom'})


def gates(key: str, pair: str | None) -> bool:
    """Would a counted line of this category and pair gate, if in the window?

    Pure, and the whole rule in one place: category, then pair. See
    WINDOW_GATED's comment block for why these two exclusions and no others.
    """
    return key in WINDOW_GATED and not is_slam_output_pair(pair)


def drop_verdict(attribution) -> tuple[bool, str]:
    """(gates, why) for one attributed message-filter drop. Pure.

    The rule, and the whole of it:

        no attribution      GATES. Fail closed: a drop nobody could attribute
                            has not been cleared of anything. An empty
                            attribution counts as none, because a chain with no
                            dynamic link in it is not a chain this rule knows
                            how to excuse.
        no link late        advisory. Every link had a covering transform on
                            the wire before the node gave up.
        only map <-> odom   advisory. The same exclusion the pair rule makes
            late            for warnings: that link is online slam_toolbox's
                            output and no offline map reads it.
        any other link      GATES.

    'late' is attribute_tf_drop.late_links's definition and not a second one:
    the first transform stamped at or after the dropped message reached the
    wire after the line's own wall time, or never appeared at all. The
    neighbour lags that tool prints beside it are scale for a reader and are no
    part of this -- nothing here compares a lag against a number.

    Imported inside the function so run_health stays importable by the tool
    without a cycle: attribute_tf_drop's pure half imports nothing from here.
    """
    from attribute_tf_drop import late_links

    if not attribution:
        return True, 'not attributed: fail closed'
    late = late_links(attribution)
    if not late:
        return False, 'none late'
    rig = [ln for ln in late
           if not is_slam_output_pair(f'{ln[0]} -> {ln[1]}')]
    if not rig:
        return False, 'only map <-> odom late'
    return True, 'late: ' + ', '.join(f'{a} -> {b}' for a, b in rig)


def final_gate(candidates):
    """(gating, advisory), each a list of (hit, why). Pure.

    count_health's candidates are what the log can convict. A message-filter
    drop is then judged on the bag, by drop_verdict, and is demoted to advisory
    when the bag clears it; a transform warning keeps the pair rule the log
    already applied, unchanged. So this adds nothing to the population and only
    ever moves drops out of it.
    """
    gating, advisory = [], []
    for hit in candidates:
        if hit['cat'] in DROP_CATS:
            gates_, why = drop_verdict(hit.get('attribution'))
        else:
            gates_, why = True, f'pair {hit["pair"] or "not parsed"}'
        (gating if gates_ else advisory).append((hit, why))
    return gating, advisory


def attribute_drops(bag: Path, robot: int, candidates,
                    nav2_params: Path | None = None) -> None:
    """Attach the bag's attribution to every drop among `candidates`.

    One region read per drop, 4-8 s each on these bags, and nothing at all when
    a run has no drop to attribute -- which is four of the six b2maps runs. The
    attribution is attribute_tf_drop's, called through the same entry point its
    own CLI uses, so the tool a reader runs by hand cannot disagree with the
    gate about what it found.
    """
    drops = [h for h in candidates if h['cat'] in DROP_CATS]
    if not drops:
        return
    from attribute_tf_drop import (attribute_hit, costmap_facts,
                                   nav2_params_path, read_static_links)

    params = nav2_params or nav2_params_path(robot)
    facts = costmap_facts(params)
    static_links = read_static_links(bag)
    print(f'attributing {len(drops)} message-filter drop(s) against the bag, '
          f'global_frame from {params.name}')
    for hit in drops:
        row, why = attribute_hit(bag, robot, hit, facts, static_links)
        hit['attribution'] = None if row is None else row['attribution']
        hit['attr'] = row
        hit['attr_why'] = why
    print()


def in_window(stamp: float | None, window: dict | None) -> bool:
    """Is a log line's WALL stamp inside the analysis window? Pure.

    The bag's receive stamps and a ROS log's stamps are the same machine's
    epoch clock, which is what makes this comparison possible at all:
    window['recv_*'] comes from the bag, `stamp` from the log beside it.
    Checked on b2maps_relay1, where the bag's /clock reads sim 463.050 at wall
    1789943495.572 and the collision_monitor line at wall 1789943495.800 says
    it asked for sim 463.070.

    An unstamped line is not in the window -- it is nowhere, and reported as
    such rather than assumed into the span that decides the run.
    """
    return (window is not None and stamp is not None
            and window['recv_lo'] <= stamp <= window['recv_hi'])


def count_health(lines, window: dict | None = None) -> dict:
    """Counts per category, split by node and by frame pair, placed against the
    analysis window. Pure.

    Returns {'counts': {...}, 'by_proc': {cat: {proc: n}},
             'by_pair': {cat: {pair or None: n}},
             'placed': {cat: {'in': n, 'out': n, 'unstamped': n}},
             'candidates': [hit, ...], 'first', 'last', 'span_s', 'stamped'}.

    `candidates` is everything the LOG can convict: one entry per counted line
    that gates on the pair rule AND fell inside the window, each carrying the
    line itself so the report can print it in full. For a transform warning
    that is the verdict. For a message-filter drop it is only a candidacy --
    final_gate() judges those on the bag's attribution. Without a window it is
    empty and 'placed' is all 'unstamped'/'out': there is nothing to place
    against, which is why the caller reports NOT EVALUATED rather than a pass.
    """
    counts = {k: 0 for k in COUNTS}
    by_proc = {k: {} for k in COUNTS}
    by_pair = {k: {} for k in COUNTS}
    placed = {k: {'in': 0, 'out': 0, 'unstamped': 0} for k in COUNTS}
    candidates = []
    first = last = None
    stamped = 0

    for line in lines:
        t = stamp_of(line)
        if t is not None:
            stamped += 1
            if first is None or t < first:
                first = t
            if last is None or t > last:
                last = t
        proc = PROC_RE.match(line)
        proc = proc.group(1) if proc else '?'
        for key, spec in COUNTS.items():
            if spec['require'] and not spec['require'].search(line):
                continue
            if spec['exclude'] and spec['exclude'].search(line):
                continue
            if not any(p in line for p in spec['any_of']):
                continue

            counts[key] += 1
            by_proc[key][proc] = by_proc[key].get(proc, 0) + 1
            pair = frame_pair(line)
            by_pair[key][pair] = by_pair[key].get(pair, 0) + 1
            where = ('unstamped' if t is None
                     else 'in' if in_window(t, window) else 'out')
            placed[key][where] += 1
            if where == 'in' and gates(key, pair):
                frame, scan = dropped_message(line)
                candidates.append({'cat': key, 'stamp': t, 'proc': proc,
                                   'pair': pair, 'line': line.rstrip(),
                                   'frame': frame, 'scan': scan})

    span = (last - first) if (first is not None and last is not None) else 0.0
    return {'counts': counts, 'by_proc': by_proc, 'by_pair': by_pair,
            'placed': placed, 'candidates': candidates, 'first': first,
            'last': last, 'span_s': span, 'stamped': stamped}


def per_minute(counts: dict, span_s: float) -> dict:
    """Counts -> rate per minute. A zero span yields inf for any nonzero count
    rather than a division error: a burst inside an instant is not healthy.
    """
    if span_s <= 0.0:
        return {k: (0.0 if v == 0 else math.inf) for k, v in counts.items()}
    return {k: v * 60.0 / span_s for k, v in counts.items()}


def judge(rates: dict, baseline: dict, factor: float = FAIL_FACTOR) -> dict:
    """Per compared category: (within, limit). ADVISORY -- see VS_BASELINE.

    More than `factor` times the baseline rate is "over". A zero baseline makes
    the limit zero, so any occurrence is over it -- b2maps_e0 ran 98.96 minutes
    and logged not one transform warning, so zero is what this stack does when
    it is healthy.

    This used to be R9's verdict, and the 'ok' it returned is now 'within': the
    run is decided by where the warnings fell, not by how a second log's rates
    compare.
    """
    out = {}
    for key in VS_BASELINE:
        limit = baseline.get(key, 0.0) * factor
        out[key] = (rates.get(key, 0.0) <= limit, limit)
    return out


def read_log(path: Path):
    """Explore logs carry raw bytes from the child processes, so they are not
    reliably valid UTF-8 -- grep calls them binary. Decode permissively.
    """
    if not path.is_file():
        die(f'log not found: {path}')
    return path.read_text(errors='replace').splitlines()


# ── R10: is the rig transform itself on time? (REPORT ONLY, no gate) ─────────
# R9 is consumer-reported all the way down: a transform warning is a node
# saying it could not get one, and a message-filter drop is judged on whether a
# link answered before that node gave up. Both fire only if a consumer happened
# to look, and only after its own wait expired -- 0.5 s for the costmaps. So a
# rig transform late by less than that wait is invisible to R9 however slow the
# rest of the stack was, and one late by more is invisible unless somebody
# asked. That is the blind spot b2maps README criterion 2 now states.
#
# R10 measures the link directly instead, over the same window, and REPORTS. It
# is not a gate and contributes nothing to the exit status: the threshold is to
# be decided from the numbers it prints, not fitted here. Two quantities,
# because they are not the same question:
#
#   wire lag    per transform, in WALL seconds: its receive time minus the wall
#               instant of its own header stamp. "Did the publisher get it out
#               on time." A publisher-side diagnostic.
#   staleness   at each arrival, in SIM seconds: the sim time then, minus the
#               newest stamp already on the wire. "How old was the best answer
#               a consumer could have got."
#
# Staleness is the quantity a transform_tolerance governs, and the units are
# the reason: every node here runs use_sim_time, so tf2's "now" and every
# stamp are sim time, and a tolerance bounds sim-time age. The wire lag is
# wall, and the two are NOT interchangeable -- b2maps_k4's worst transform is
# 151.9 ms of wall late and only 139.6 ms of sim stale, because sim was
# advancing at about 0.4x wall through that stretch. Comparing the wall lag
# against a tolerance would be a unit error, so the counts below mark which
# column a tolerance can be read against.
#
# Staleness never reaches zero: it is the lag plus up to one publication
# period, so a healthy 20 Hz link sits near 50-60 ms.
#
# Both are measured against /clock by interpolation rather than by taking the
# nearest sample, because /clock is 100 Hz here and the numbers of interest are
# tens of milliseconds.
RECOVERY_RE = re.compile(
    r'^\[behavior_server-[0-9]+\].*?: Running (?!Nav2 LifecycleNode)(\S+)')


def _interp(xs, ys, x: float):
    """y at x, linearly between the bracketing samples. None outside. Pure.

    `xs` must be sorted. Used both ways round the /clock series: sim from a
    receive time, and a receive time from sim.
    """
    i = bisect.bisect_left(xs, x)
    if i < len(xs) and xs[i] == x:
        return ys[i]        # an exact sample, and the only use of xs[0]
    if i == 0 or i >= len(xs):
        return None
    x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
    if x1 == x0:
        return y0
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def rig_timeliness(tf_rows, clock, sim_lo: float, sim_hi: float) -> dict:
    """Wire lags and staleness for one link over a sim span. Pure.

    `tf_rows` is [(stamp, receive)] in publication order, `clock` is
    [(receive, sim)] in read order.

    Staleness is sampled at every arrival, which is where it peaks: between two
    arrivals nothing new is available, so the age of the newest stamp grows
    until the next one lands. One sample per arrival over a near-uniform 50 ms
    grid, so the p99 of the samples is the p99 of the continuous function to
    within a period.
    """
    recvs = [r for r, _s in clock]
    sims = [s for _r, s in clock]
    lags, stale, newest = [], [], None
    for stamp, recv in tf_rows:
        if newest is not None:
            now = _interp(recvs, sims, recv)
            if now is not None and sim_lo <= stamp <= sim_hi:
                stale.append((now, now - newest, recv))
        if sim_lo <= stamp <= sim_hi:
            w = _interp(sims, recvs, stamp)
            if w is not None:
                lags.append((stamp, recv, recv - w))
        newest = stamp if newest is None else max(newest, stamp)
    return {'lags': lags, 'stale': stale}


def quantiles(values) -> dict:
    """median, p99 and max of a list, or Nones. Pure."""
    if not values:
        return {'n': 0, 'median': None, 'p99': None, 'max': None}
    ordered = sorted(values)
    return {'n': len(ordered), 'median': statistics.median(ordered),
            'p99': ordered[int(0.99 * (len(ordered) - 1))],
            'max': ordered[-1]}


def recovery_starts(lines) -> list:
    """[(wall stamp, behaviour name)] for every recovery behavior_server ran.

    nav2_behaviors logs exactly one INFO line when a behaviour begins --
    "Running %s" at timed_behavior.hpp:213 -- and NONE when it ends:
    succeeded_current() reports through debug_msg
    (simple_action_server.hpp:507), which is below the log's level. So a
    recovery's start is on the record and its end is not, and R10 reports the
    nearest preceding start rather than claiming a span it cannot close.

    The lifecycle line "Running Nav2 LifecycleNode rcl preshutdown
    (behavior_server)" also begins with Running and is excluded by name.
    """
    out = []
    for line in lines:
        m = RECOVERY_RE.match(line)
        if m:
            t = stamp_of(line)
            if t is not None:
                out.append((t, m.group(1)))
    return out


def read_rig_tf(bag: Path, robot: int, window: dict):
    """([(stamp, receive)] for the rig link, [(receive, sim)] for /clock) over
    the window's receive span. seek()s to the window rather than reading the
    bag end to end.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    pair = (f'robot_{robot}/odom', f'robot_{robot}/base_footprint')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {t.name for t in reader.get_all_topics_and_types()}
    if '/tf' not in available or '/clock' not in available:
        die(f'{bag} needs /tf and /clock for R10')
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/clock']))
    reader.seek(int(window['recv_lo'] * 1e9))

    clock, tf = [], []
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        recv = recv_ns / 1e9
        if recv < window['recv_lo']:
            continue
        if recv > window['recv_hi']:
            break
        if topic == '/clock':
            c = deserialize_message(data, Clock).clock
            clock.append((recv, c.sec + c.nanosec / 1e9))
        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                if (tr.header.frame_id, tr.child_frame_id) == pair:
                    tf.append((tr.header.stamp.sec
                               + tr.header.stamp.nanosec / 1e9, recv))
    return tf, clock


def report_rig_timeliness(robot: int, window: dict, measured: dict,
                          tols: list, starts: list, truth_hz: float,
                          params: Path) -> None:
    """R10. Prints; decides nothing. See the comment block above."""
    print('R10 IS THE RIG TRANSFORM ITSELF ON TIME?  (REPORT ONLY -- not a '
          'gate, and no')
    print('    part of the exit status. The threshold is to be decided from '
          'these numbers.)')
    print(f'  link           robot_{robot}/odom -> robot_{robot}/'
          'base_footprint')
    print(f'  window         sim [{window["sim_lo"]:.3f}, '
          f'{window["sim_hi"]:.3f}]')

    lags = [lag for _s, _r, lag in measured['lags']]
    stale = [age for _now, age, _recv in measured['stale']]
    q_lag, q_stale = quantiles(lags), quantiles(stale)
    period_ms = 1000.0 / truth_hz
    print(f'  transforms     {q_lag["n"]}   nominal {truth_hz:g} Hz, one '
          f'period {period_ms:g} ms')
    if not lags:
        print('  no transform on this link inside the window, so there is '
              'nothing to measure')
        print('  -> REPORTED (nothing)\n')
        return
    print('  wire lag, WALL seconds  = receive time - the wall instant of the '
          'transform\'s own stamp')
    print(f'      median {q_lag["median"] * 1e3:7.1f} ms   '
          f'p99 {q_lag["p99"] * 1e3:7.1f} ms   '
          f'max {q_lag["max"] * 1e3:7.1f} ms')
    print('  staleness, SIM seconds  = at each arrival, sim now - the newest '
          'stamp already on the wire')
    print(f'      median {q_stale["median"] * 1e3:7.1f} ms   '
          f'p99 {q_stale["p99"] * 1e3:7.1f} ms   '
          f'max {q_stale["max"] * 1e3:7.1f} ms   '
          f'(floor is one period, {period_ms:g} ms)')

    print(f'  transform_tolerance declared in {params.name}, smallest first:')
    for decl in sorted(tols, key=lambda d: d['tol']):
        frames = ', '.join(f'{k}={v}' for k, v in decl['frames'].items())
        print(f'      {decl["tol"]:.2f} s  {decl["block"]:<32} '
              f'(:{decl["line"]})  {frames or "declares no frames"}')
    print(f'      exceeding each, of {q_lag["n"]} transforms and '
          f'{q_stale["n"]} staleness samples. Everything runs use_sim_time,')
    print('      so a tolerance bounds SIM age: the staleness column is the '
          'one it can be read')
    print('      against, and the wall column is there to show the publisher '
          'side.')
    for tol in sorted({d['tol'] for d in tols}):
        n_lag = sum(1 for v in lags if v > tol)
        n_stale = sum(1 for v in stale if v > tol)
        print(f'      > {tol:.2f} s   staleness (sim) {n_stale:>6}   '
              f'wire lag (wall) {n_lag:>6}')

    smallest = min(d['tol'] for d in tols)
    over = [(now, age, recv) for now, age, recv in measured['stale']
            if age > smallest]
    print(f'  staleness excursions over the smallest declared tolerance '
          f'({smallest:.2f} s): {len(over)}')
    for now, age, recv in over[:20]:
        print(f'      sim {now:9.3f}  wire {recv:.6f}  stale '
              f'{age * 1e3:6.1f} ms   behaviour: '
              f'{_recovery_context(recv, starts)}')
    if len(over) > 20:
        print(f'      ... and {len(over) - 20} more')
    n_lag_over = sum(1 for v in lags if v > smallest)
    if n_lag_over != len(over):
        print(f'      ({n_lag_over} transforms exceed {smallest:.2f} s of '
              'WALL lag, which is the')
        print('       other column and not the same set -- see the units '
              'note above)')
    if over and not starts:
        print('      behavior_server ran no behaviour at all in this log, so '
              'the one consumer')
        print(f'      declaring {smallest:.2f} s never looked up this link '
              'while any of these')
        print('      excursions was in force. That is a fact about this run, '
              'not a defence:')
        print('      the transform was late whether or not anything asked for '
              'it.')
    print('  -> REPORTED (not a gate)\n')


def _recovery_context(recv: float, starts: list) -> str:
    """What behavior_server was last seen starting before a wall instant.

    No end marker exists at INFO level -- see recovery_starts -- so this says
    how long ago a behaviour began and never that one was still running.
    """
    if not starts:
        return 'none ran in this log'
    before = [(t, name) for t, name in starts if t <= recv]
    if not before:
        nxt = min(t for t, _n in starts)
        return f'none yet; first one {nxt - recv:.1f} s later'
    t, name = before[-1]
    return f'{name} started {recv - t:.1f} s earlier (no end line is logged)'


# ─────────────────────── coverage: closest approach ──────────────────────────

def closest_approach(pts, rects) -> float:
    """Least distance from any point in `pts` to any rectangle in `rects`.

    Distance to the rectangle, which for these 0.1 m-thick walls is distance to
    the face. Built on bag_overlap.dist_to_nearest_wall so that "how far from a
    wall" means one thing across this project.
    """
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return math.inf
    return float(np.min(dist_to_nearest_wall(pts, rects)))


def _yaw(x, y, z, w):
    """Heading about Z. The one expression this project uses -- see
    check_run_bag.yaw_from_quaternion for why it is not 2*acos(w).
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _inv_compose(anchor, pose):
    """inv(A) . P, both (x, y, yaw). The node's composition and the offline
    rewriter's, restated here only so this check does not import either of the
    things it is checking.
    """
    ax, ay, at = anchor
    c, s = math.cos(-at), math.sin(-at)
    ix, iy = -(c * ax - s * ay), -(s * ax + c * ay)
    px, py, pt = pose
    c, s = math.cos(-at), math.sin(-at)
    return (ix + c * px - s * py, iy + s * px + c * py, pt - at)


def read_truth_tf_series(bag: Path, robot: int):
    """One pass for R4 and R5: the truth poses, the recorded odom->base
    transform, and the wheel odometry, keyed by header stamp in nanoseconds.
    """
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    pose_topic = f'/model/robot_{robot}/pose'
    odom_topic = f'/robot_{robot}/odom'
    parent, child = f'robot_{robot}/odom', f'robot_{robot}/base_footprint'

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {t.name for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (pose_topic, odom_topic, '/tf') if t in available]
    if pose_topic not in wanted or '/tf' not in wanted:
        die(f'{bag} needs both {pose_topic} and /tf for this check')
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    truth = {}            # stamp_ns -> (x, y, yaw)
    tf_pairs = []         # (stamp_ns, x, y, yaw)
    wheel = []            # (stamp_ns, yaw)
    other_publishers = 0

    while reader.has_next():
        topic, data, _recv = reader.read_next()
        if topic == pose_topic:
            m = deserialize_message(data, PoseStamped)
            q, p = m.pose.orientation, m.pose.position
            ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            truth[ns] = (p.x, p.y, _yaw(q.x, q.y, q.z, q.w))
        elif topic == odom_topic:
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            wheel.append((ns, _yaw(q.x, q.y, q.z, q.w)))
        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id != parent or tr.child_frame_id != child:
                    continue
                q, t = tr.transform.rotation, tr.transform.translation
                ns = tr.header.stamp.sec * 10**9 + tr.header.stamp.nanosec
                tf_pairs.append((ns, t.x, t.y, _yaw(q.x, q.y, q.z, q.w),
                                 t.z))
    return truth, tf_pairs, wheel, other_publishers


def window_from(sim_start: float, min_sim: float, t_cmd_recv: float,
                t_end_recv: float, sim_end: float, cmd_topic: str) -> dict:
    """The analysis window, in both of the clocks this file needs. Pure.

    SIM nanoseconds, because that is what a transform's header stamp carries
    and R4 compares stamps. RECEIVE seconds, because a ROS log line carries a
    wall-clock stamp off the same machine clock the recorder stamped messages
    with, and that is what R9 has to place a warning in.

    The two spans are not quite the same length. sim_hi is exactly
    t_cmd + min_sim; recv_hi is the receive time of the first /clock whose
    VALUE reached it, so the wall span can run up to one clock step (10 ms at
    these runs' 100 Hz) past the sim span. C4 already gates that overshoot
    against SIM_SPAN_SLACK_S; nothing here is decided at that resolution.

    round(), not int(): sim_start arrives as a float built from a Clock's
    sec + nanosec/1e9, and truncating it could land a transform stamped at
    exactly t_cmd outside its own window. A double holds sim 5000 s to about
    1e-12 s, so the rounded nanosecond is the one the clock published.
    """
    lo_ns = round(sim_start * 1e9)
    return {
        'sim_lo': sim_start,
        'sim_hi': sim_start + min_sim,
        'sim_lo_ns': lo_ns,
        'sim_hi_ns': lo_ns + round(min_sim * 1e9),
        'sim_end': sim_end,
        'recv_lo': t_cmd_recv,
        'recv_hi': t_end_recv,
        'min_sim': min_sim,
        'cmd_topic': cmd_topic,
        'src': (f'the first |v| > {CMD_EPS_LINEAR} m/s or '
                f'|w| > {CMD_EPS_ANGULAR} rad/s on {cmd_topic}'),
    }


def read_analysis_window(bag: Path, robot: int, min_sim: float):
    """(window, None) for one bag, or (None, why it could not be placed).

    Both halves are check_run_bag.py's -- see the import. _ClockTrace is what
    lets the failure say how much sim the bag DID hold after the command,
    which is the one number a short bag most needs reported, and it is the same
    use C4 puts it to.
    """
    cmd_topic = f'/robot_{robot}/cmd_vel'
    cmds, _poses = read_prologue_series(bag, robot)
    t_cmd = first_nonzero_cmd(cmds)
    if t_cmd is None:
        return None, (f'no command over {CMD_EPS_LINEAR} m/s or '
                      f'{CMD_EPS_ANGULAR} rad/s anywhere on {cmd_topic}, so '
                      'this run never began exploring and has no window')

    trace = _ClockTrace(read_clock_series(bag), t_cmd)
    found = sim_window(trace, t_cmd, min_sim)
    if found is None:
        have = trace.sim_available
        if have is None:
            return None, ('no /clock arrived at or before the first command, '
                          'so there is no sim time to place the window from')
        return None, (f'/clock never advances {min_sim:.0f} s of sim after '
                      f'the first command: the bag holds {have:.3f} s, short '
                      f'by {min_sim - have:.3f} s')

    sim_start, t_end, sim_end = found
    return window_from(sim_start, min_sim, t_cmd, t_end, sim_end,
                       cmd_topic), None


def split_at_window(tf_pairs, lo_ns: int, hi_ns: int):
    """Recorded transforms partitioned by stamp against the analysis window.

    Returns (before, inside, after), each a list of (stream index, row), where
    the stream index is the transform's position in the order it was recorded.
    Both bounds are inclusive: a transform stamped exactly at the first command
    is inside the span the map is built from.

    Pure. `tf_pairs` must be in recorded order, which read_truth_tf_series
    guarantees by appending as it reads.
    """
    before, inside, after = [], [], []
    for i, row in enumerate(tf_pairs):
        ns = row[0]
        bucket = before if ns < lo_ns else after if ns > hi_ns else inside
        bucket.append((i, row))
    return before, inside, after


def compare_window(rows, truth, anchor):
    """(matched, unmatched, worst |dxy| m, worst |dyaw| deg, worst |z| m) for
    one population of transforms against the truth poses. Pure.

    A transform whose stamp is not one the pose series carries is UNMATCHED and
    contributes to no maximum: there is nothing to compare it against. The node
    copies the pose's stamp verbatim, so inside the window that is the finding
    R4 exists for, and outside it is the recorder's attach order.

    One function for both populations so the numbers printed for the ungated
    one are the same arithmetic as the numbers the gate is decided on.
    """
    matched = unmatched = 0
    worst_xy = worst_yaw = worst_z = 0.0
    for _i, (ns, x, y, yaw, z) in rows:
        want = truth.get(ns)
        if want is None:
            unmatched += 1
            continue
        matched += 1
        ex, ey, eyaw = _inv_compose(anchor, want)
        worst_xy = max(worst_xy, abs(x - ex), abs(y - ey))
        worst_yaw = max(worst_yaw, abs(math.degrees(_wrap(yaw - eyaw))))
        worst_z = max(worst_z, abs(z))
    return matched, unmatched, worst_xy, worst_yaw, worst_z


def truth_gaps(truth, truth_hz: float):
    """Adjacent truth stamps more than TRUTH_GAP_PERIODS apart. Pure.

    Reported, never gated. R4 judges the transform, not the pose stream: a hole
    with no transform stranded in it is a question about the rate, and R2 owns
    rates. A hole that DOES strand transforms inside the analysis window fails
    R4 already, as transforms on no truth stamp -- which is the same rule
    everything else inside the window is held to, not a special case for gaps.
    """
    period_ns = 1e9 / truth_hz
    ks = sorted(truth)
    return [(a, b) for a, b in zip(ks, ks[1:])
            if (b - a) > TRUTH_GAP_PERIODS * period_ns]


def report_truth_tf(robot: int, anchor, truth, tf_pairs, wheel,
                    truth_hz: float, window: dict | None,
                    window_why: str | None = None) -> bool | None:
    """R4 and R5 together: the transform IS the truth, and the wheel odometry
    is still the wheel's.

    R4 compares each recorded robot_K/odom -> robot_K/base_footprint against
    inv(A) . P at ITS OWN header stamp. The node copies the pose's stamp
    verbatim, so a transform whose stamp is one the pose series carries must
    match it exactly, and one that falls BETWEEN two pose samples was published
    by something else -- a surviving DiffDrive broadcast being the case this
    guards. That comparison, and its 1e-6 tolerances, are unchanged.

    What changed (2026-09-28) is WHICH transforms it is asked of: the ones
    inside the analysis window, [t_cmd, t_cmd + --min-sim] in sim time, which
    is the span the offline map is built from. Inside it the rule is absolute:
    land on a truth stamp, match it to 1e-6, or fail, with no grace at all.
    Outside it every number is still computed and printed, and none of it is
    gated: no map reads that time. See MIN_SIM_S for what this replaces and for
    the measured evidence that the old edge rule was gating the recorder's
    subscription order rather than the transform.

    Poses carrying no transform are counted and printed but not gated: a
    transform that was never published is a question about the rate, and R2
    already answers that against /clock.

    Without a window -- no cmd_vel, no /clock, no nonzero command, or a bag too
    short to hold --min-sim of sim after the command -- R4 returns None, NOT
    EVALUATED, after printing the ungated comparison. It does not fall back to
    judging the whole bag: that is the rule this change removed. R5 needs no
    window, so a failing R5 still returns False and cannot hide behind a
    missing one.

    R5 is the C4 discriminator from rewrite_odom_from_truth, pointed the other
    way: if /robot_K/odom had quietly become a copy of the truth, the two yaw
    series would agree and this run would prove nothing about odometry.
    """
    print('R4 THE TRANSFORM IS THE TRUTH')
    print(f'  anchor A = spawn of robot_{robot}: ({anchor[0]:+.4f}, '
          f'{anchor[1]:+.4f}, {math.degrees(anchor[2]):+.4f} deg)')
    if window is None:
        print(f'  NO ANALYSIS WINDOW: {window_why}')
        print('  the numbers below are over the whole recording and are NOT '
              'gated')
    else:
        print(f'  analysis window  sim [{window["sim_lo"]:.3f}, '
              f'{window["sim_hi"]:.3f}]   (t_cmd + {window["min_sim"]:g} s, '
              '--min-sim)')
        print(f'      t_cmd        {window["sim_lo"]:.3f} s sim, bag receive '
              f'{window["recv_lo"]:.6f}')
        print(f'                   {window["src"]}')
        print('                   -- check_run_bag.first_nonzero_cmd, the '
              'command C3 and C4')
        print('                   place their own windows on, so this is the '
              'span the map is cut to')
        print(f'      wall span    [{window["recv_lo"]:.6f}, '
              f'{window["recv_hi"]:.6f}]  (the same window on the log\'s '
              'clock)')
    print(f'  truth poses {len(truth)}   recorded transforms {len(tf_pairs)}')
    if not tf_pairs:
        print(f'  no robot_{robot}/odom -> robot_{robot}/base_footprint in '
              '/tf at all')
        print('  -> FAIL\n')
        return False
    if not truth:
        print(f'  no /model/robot_{robot}/pose in the bag, so there is no '
              'truth to compare the transform against')
        print('  -> FAIL\n')
        return False

    period_ms = 1000.0 / truth_hz
    gaps = truth_gaps(truth, truth_hz)
    print(f'  pose series   [{min(truth) / 1e9:.3f}, {max(truth) / 1e9:.3f}] '
          f's sim, {"gapless" if not gaps else f"{len(gaps)} GAP(S)"} at '
          f'{truth_hz:g} Hz ({period_ms:g} ms)')
    for a, b in gaps[:5]:
        print(f'      {(b - a) / 1e6:.1f} ms with no pose, after '
              f'{a / 1e9:.3f} s sim{_where(a, window)}')

    if window is None:
        rows = list(enumerate(tf_pairs))
        matched, unmatched, worst_xy, worst_yaw, worst_z = compare_window(
            rows, truth, anchor)
        _print_comparison('  ', matched, unmatched, worst_xy, worst_yaw,
                          worst_z)
        ok = None
    else:
        before, inside, after = split_at_window(tf_pairs, window['sim_lo_ns'],
                                                window['sim_hi_ns'])
        matched, unmatched, worst_xy, worst_yaw, worst_z = compare_window(
            inside, truth, anchor)
        print('  INSIDE the window -- gated')
        _print_comparison('    ', matched, unmatched, worst_xy, worst_yaw,
                          worst_z)
        print('  OUTSIDE the window -- reported, never gated: no map is built '
              'from this time')
        for label, rows in (('before t_cmd    ', before),
                            ('after the window', after)):
            _m, um, wxy, wyaw, wz = compare_window(rows, truth, anchor)
            print(f'    {label}  {len(rows):>7} transforms, {um} on no truth '
                  f'stamp, worst |dxy| {wxy:.3e} m, |dyaw| {wyaw:.3e} deg, '
                  f'|z| {wz:.3e} m')
            if rows:
                stamps = [row[0] for _i, row in rows]
                print(f'                        {min(stamps) / 1e9:.3f} .. '
                      f'{max(stamps) / 1e9:.3f} s sim')
        ok = (unmatched == 0 and matched > 0 and worst_xy <= 1e-6
              and worst_yaw <= 1e-6 and worst_z == 0.0)

    orphans = sorted(set(truth) - {row[0] for row in tf_pairs})
    print(f'  poses with no transform       {len(orphans):>9}  '
          '(reported, not gated: a missing transform is R2\'s rate question)')
    if orphans and window is not None:
        n_in = sum(1 for ns in orphans
                   if window['sim_lo_ns'] <= ns <= window['sim_hi_ns'])
        print(f'      of those, inside the window {n_in}')

    print(f'  -> {_verdict(ok)}\n')

    print('R5 THE WHEEL ODOMETRY IS STILL THE WHEEL\'S')
    if not wheel:
        print(f'  /robot_{robot}/odom is absent -- the wheel stream this run '
              'has to preserve is not in the bag')
        print('  -> FAIL\n')
        return False
    diffs = []
    for ns, wyaw in wheel:
        want = truth.get(ns)
        if want is None:
            continue
        diffs.append(abs(math.degrees(
            _wrap(wyaw - _inv_compose(anchor, want)[2]))))
    print(f'  /robot_{robot}/odom messages   {len(wheel):>9}')
    print(f'  on a truth stamp              {len(diffs):>9}')
    if not diffs:
        print('  no shared stamp, so the two series cannot be compared')
        print('  -> FAIL\n')
        return False
    diffs.sort()
    median = diffs[len(diffs) // 2]
    print(f'  |dYaw| wheel vs truth: median {median:8.3f} deg, '
          f'max {diffs[-1]:8.3f} deg')
    print('  need median > 1 deg (else the wheel stream is not independent)')
    wheel_ok = median > 1.0
    print(f'  -> {"PASS" if wheel_ok else "FAIL"}\n')
    # ok is None when no window could be placed, and R5 does not need one. A
    # failing R5 is still a failure; otherwise the pair is NOT EVALUATED,
    # because R4 -- the gate this bag was read for -- has not been answered.
    if not wheel_ok:
        return False
    return ok


def _print_comparison(indent: str, matched: int, unmatched: int,
                      worst_xy: float, worst_yaw: float,
                      worst_z: float) -> None:
    """The five numbers R4 is decided on, in one shape wherever they appear."""
    print(f'{indent}compared      {matched:>9}  transforms')
    print(f'{indent}worst |dxy|   {worst_xy:.3e} m    (tolerance 1e-6)')
    print(f'{indent}worst |dyaw|  {worst_yaw:.3e} deg  (tolerance 1e-6)')
    print(f'{indent}worst |z|     {worst_z:.3e} m    (must be 0: planar '
          'transform)')
    print(f'{indent}on no truth stamp {unmatched:>5}  (must be 0: another '
          'publisher)')


def _where(stamp_ns: int, window: dict | None) -> str:
    """"  (before t_cmd)" and friends, for a sim stamp. '' without a window."""
    if window is None:
        return ''
    if stamp_ns < window['sim_lo_ns']:
        return '  (before t_cmd)'
    if stamp_ns > window['sim_hi_ns']:
        return '  (after the window)'
    return '  (INSIDE the window)'


def _verdict(ok: bool | None) -> str:
    return {True: 'PASS', False: 'FAIL', None: 'NOT EVALUATED'}[ok]


def _wrap(rad: float) -> float:
    return (rad + math.pi) % (2 * math.pi) - math.pi


# ───────────────── R2: one owner per frame pair, by its rate ─────────────────
# tf2 in ROS 2 cannot answer "who published this transform". A TFMessage
# carries no publisher identity, and `ros2 run tf2_ros tf2_monitor A B` reports
# the rate of ALL of /tf rather than of the pair it was asked about -- on this
# swarm it printed ~316 Hz for robot_0 and the same ~316 Hz for robot_1, which
# is the five-robot tree, not either robot's odom->base_footprint.
#
# The rate of the PAIR does discriminate, because the two candidate owners run
# at rates that come from different files and differ by 2.5x:
#
#   truth_odom_tf  20 Hz  one transform per /model/robot_K/pose, which the
#                         PosePublisher in swarm_sim.launch.py emits at
#                         <update_frequency> -- rate_facts()['truth_hz']
#   DiffDrive      50 Hz  the gz-sim plugin default, since the burger SDF's
#                         own frequency element is misspelled and inert --
#                         rate_facts()['diffdrive_hz']
#
# So robot K reading 20 Hz says DiffDrive's broadcast is gone, a parked robot
# still reading 50 Hz says the SDF edit reached only robot K, and either pair
# reading the 70 Hz sum says both are publishing. R4 closes the other
# direction: every transform on K's pair lands on a truth stamp, so the 20 Hz
# population is the truth node's rather than some third publisher's at the
# same rate.

def read_tf_pair_stamps(bag: Path, pairs):
    """Header stamps of each (parent, child) pair in /tf, plus the sim span.

    Returns ({pair: [stamp_ns, ...]}, sim_span_s or None, (messages,
    transforms)).

    Messages and transforms are counted separately because on this rig they
    are very different numbers: b2maps_rehearsal3 holds 77629 /tf messages
    carrying 57832 transforms, the other 19797 being EMPTY TFMessages
    published at a steady 100.0 Hz. Counting messages as transforms -- which
    is what a rate taken off `ros2 topic hz /tf` does -- would inflate every
    rate on this topic by a third.

    The stamps are sim time: everything in this stack runs use_sim_time, and
    the bridge carries Gazebo's clock into the header. The span comes from
    /clock for the same reason check_run_bag.py's C4 uses it -- bag timestamps
    are wall-clock receive times, and RTF varies within a run as well as
    between runs, so a rate per bag second is not a rate.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    stamps = {tuple(p): [] for p in pairs}

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {t.name for t in reader.get_all_topics_and_types()}
    if '/tf' not in available:
        die(f'{bag} carries no /tf, so no frame pair has a rate')
    wanted = ['/tf'] + (['/clock'] if '/clock' in available else [])
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    sim_first = sim_last = None
    messages = transforms = 0
    while reader.has_next():
        topic, data, _recv = reader.read_next()
        if topic == '/clock':
            c = deserialize_message(data, Clock).clock
            t = c.sec + c.nanosec / 1e9
            if sim_first is None:
                sim_first = t
            sim_last = t
            continue
        messages += 1
        for tr in deserialize_message(data, TFMessage).transforms:
            transforms += 1
            key = (tr.header.frame_id, tr.child_frame_id)
            if key in stamps:
                stamps[key].append(tr.header.stamp.sec * 10**9
                                   + tr.header.stamp.nanosec)

    span = (sim_last - sim_first) if sim_first is not None else None
    return stamps, span, (messages, transforms)


def tf_rate(stamps_ns, sim_span_s: float | None) -> dict:
    """Count and the two rates for one frame pair. Pure.

    'rate_sim' is the gated number: transforms per second of the SIMULATION
    the bag covers, so a publisher that stopped halfway through shows up as
    half its nominal rate.

    'rate_own' is the same count over the pair's OWN first-to-last stamp span,
    and exists to make a failure readable. Read together:

        own 20, sim 20   one owner at the truth rate
        own 20, sim 10   the truth node died (or started) mid-run
        own 70, sim 70   two owners on one pair

    'distinct' is how many different stamps the transforms carry. Reported
    only: a stamp repeated by two publishers is R4's question, and R4 answers
    it against the truth poses rather than by counting.
    """
    n = len(stamps_ns)
    out = {'n': n, 'distinct': len(set(stamps_ns)), 'first_ns': None,
           'last_ns': None, 'own_span_s': 0.0, 'rate_own': 0.0,
           'rate_sim': 0.0, 'sim_span_s': sim_span_s}
    if n:
        out['first_ns'] = min(stamps_ns)
        out['last_ns'] = max(stamps_ns)
        out['own_span_s'] = (out['last_ns'] - out['first_ns']) / 1e9
        if out['own_span_s'] > 0:
            out['rate_own'] = n / out['own_span_s']
    if sim_span_s and sim_span_s > 0:
        out['rate_sim'] = n / sim_span_s
    return out


def judge_rate(stats: dict, nominal_hz: float,
               tol: float = TF_RATE_TOL) -> tuple[bool, float, float]:
    """(ok, lo, hi) for one pair against its nominal rate. Pure.

    An empty pair fails: the transform the run depends on is simply absent,
    which is a louder finding than any rate.
    """
    lo, hi = nominal_hz * (1.0 - tol), nominal_hz * (1.0 + tol)
    ok = stats['n'] > 0 and lo <= stats['rate_sim'] <= hi
    return ok, lo, hi


def report_tf_rates(robot: int, parked: int, stats_k: dict, stats_p: dict,
                    facts: dict, sim_span: float | None,
                    seen: tuple[int, int]) -> bool | None:
    """R2: the frame pair's rate names its owner."""
    messages, transforms = seen
    print('R2 ONE OWNER PER FRAME PAIR')
    print(f'  /tf messages        {messages:>10}')
    print(f'  transforms in them  {transforms:>10}  (all pairs, all five '
          'robots)')
    if messages > transforms:
        print(f'  {messages - transforms} of those messages carry NO '
              'transform at all, so message count is not transform count')
    if not sim_span or sim_span <= 0:
        print('  the bag carries no /clock span, so a rate per SIM second '
              'cannot be formed')
        print('  -> NOT EVALUATED\n')
        return None
    print(f'  sim time in the bag {sim_span:>10.3f} s  (from /clock)')
    print()

    ok = True
    for who, k, stats, nominal, src in (
            ('explorer, truth_odom_tf', robot, stats_k, facts['truth_hz'],
             facts['truth_src']),
            ('parked control, DiffDrive', parked, stats_p,
             facts['diffdrive_hz'], 'gz-sim DiffDrive default')):
        passed, lo, hi = judge_rate(stats, nominal)
        ok = ok and passed
        print(f'  robot_{k}/odom -> robot_{k}/base_footprint   ({who})')
        print(f'    transforms        {stats["n"]:>10}   '
              f'distinct stamps {stats["distinct"]}')
        if stats['n'] == 0:
            print('    the pair is absent from /tf entirely')
            print(f'    -> {"ok" if passed else "FAIL"}')
            continue
        covered = 100.0 * stats['own_span_s'] / sim_span
        print(f'    stamp span        {stats["own_span_s"]:>10.3f} s  '
              f'({covered:.1f}% of the bag)')
        print(f'    rate over it      {stats["rate_own"]:>10.3f} Hz')
        print(f'    rate over the bag {stats["rate_sim"]:>10.3f} Hz  '
              f'need {nominal:g} +/-{TF_RATE_TOL * 100:.0f}% = '
              f'[{lo:.2f}, {hi:.2f}]  {"ok" if passed else "FAIL"}')
        print(f'    nominal from      {src}')
    print()
    print('  a second publisher on a pair reads the SUM of both rates '
          f'({facts["truth_hz"] + facts["diffdrive_hz"]:g} Hz), which no band '
          'here admits;')
    print('  R4 rules out the other direction, a third publisher at the '
          'right rate.')
    print(f'  -> {"PASS" if ok else "FAIL"}\n')
    return ok


def read_truth_xy(bag: Path, robot: int):
    """(N, 2) array of robot K's true positions, from /model/robot_K/pose.

    ROS imports are function-scoped, the convention bag_overlap.py and
    check_run_bag.py already follow, so the log half of this script runs
    without a sourced ROS.
    """
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message

    topic = f'/model/robot_{robot}/pose'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    if topic not in {t.name for t in reader.get_all_topics_and_types()}:
        die(f'{bag} carries no {topic}; ground truth is what this gate reads, '
            'so it cannot be evaluated on this bag')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    xy = []
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        p = deserialize_message(data, PoseStamped).pose.position
        xy.append((p.x, p.y))
    return np.asarray(xy, dtype=float).reshape(-1, 2)


# ──────────────────── R7: the map, out of the bag ────────────────────────────
# nav2's map_saver_cli failed on this rig with "Failed to spin map
# subscription", and it could only ever have worked while the run was still
# alive. The bag already holds every /robot_K/map message the run published,
# so the map it finished with is simply the last of them -- no live node, no
# QoS negotiation, and repeatable long after the run is over.

def read_last_map(bag: Path, robot: int):
    """The LAST /robot_K/map message in the bag, as a plain dict, or None.

    Only the newest serialized payload is kept while scanning: a 2400 s run
    publishes thousands of these at ~150 KB each, and all but one are
    superseded. One deserialisation happens after the loop.
    """
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message

    topic = f'/robot_{robot}/map'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    if topic not in {t.name for t in reader.get_all_topics_and_types()}:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    last, count = None, 0
    while reader.has_next():
        _topic, data, recv_ns = reader.read_next()
        last, count = (data, recv_ns), count + 1
    if last is None:
        return None

    data, recv_ns = last
    m = deserialize_message(data, OccupancyGrid)
    q = m.info.origin.orientation
    return {
        'count': count,
        'recv_s': recv_ns / 1e9,
        'stamp_s': m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
        'frame_id': m.header.frame_id,
        'w': m.info.width, 'h': m.info.height, 'res': m.info.resolution,
        'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
        'yaw': _yaw(q.x, q.y, q.z, q.w),
        'data': np.asarray(m.data, dtype=np.int16),
    }


def grid_to_trinary(data, w: int, h: int):
    """OccupancyGrid data -> nav2 trinary pixels, in IMAGE row order. Pure.

    The thresholds and the three values are save_map.py's, so a map written
    from a bag and one saved live off the same OccupancyGrid are the same
    file. Everything that is neither free nor occupied -- unknown (-1) and the
    26..64 band both -- becomes 205, exactly as that script's else branch
    does.

    The flip is not cosmetic. OccupancyGrid row 0 is the origin row, the
    LOWEST y; nav2's PGM row 0 is the highest. An unflipped image is a map
    mirrored about the x axis, which looks perfectly plausible in a viewer and
    is wrong everywhere.
    """
    g = np.asarray(data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), PGM_UNKNOWN, dtype=np.uint8)
    img[(g >= 0) & (g <= FREE_TH_PCT)] = PGM_FREE
    img[g >= OCC_TH_PCT] = PGM_OCC
    return img[::-1, :].copy()


def map_stats(data, w: int, h: int, res: float, ox: float, oy: float) -> dict:
    """Cell counts and the KNOWN extent in metres, off the grid. Pure.

    Known means occupied or free in the trinary sense -- the cells that are
    not 205 in the written PGM -- so this and pgm_extent.py agree about the
    same map. 'other' is the 26..64 band, which renders as unknown but is not
    unknown; counted separately so a map full of undecided cells cannot hide
    inside the unknown total.

    Computed on the grid rather than the image, so the extent does not depend
    on the flip above. Cell (r, c) spans x in [ox + c*res, ox + (c+1)*res] and
    y in [oy + r*res, oy + (r+1)*res] -- edges, not centres, which is
    pgm_extent.box()'s convention.
    """
    g = np.asarray(data, dtype=np.int16).reshape(h, w)
    occ_mask = g >= OCC_TH_PCT
    free_mask = (g >= 0) & (g <= FREE_TH_PCT)
    unknown = int((g == -1).sum())
    occ, free = int(occ_mask.sum()), int(free_mask.sum())
    out = {'total': w * h, 'occupied': occ, 'free': free, 'unknown': unknown,
           'other': w * h - occ - free - unknown, 'known_box': None}

    known = occ_mask | free_mask
    rows = np.where(known.any(axis=1))[0]
    cols = np.where(known.any(axis=0))[0]
    if len(rows):
        r0, r1, c0, c1 = rows[0], rows[-1], cols[0], cols[-1]
        out['known_box'] = {
            'cols': int(c1 - c0 + 1), 'rows': int(r1 - r0 + 1),
            'width_m': float((c1 - c0 + 1) * res),
            'height_m': float((r1 - r0 + 1) * res),
            'x0': float(ox + c0 * res), 'x1': float(ox + (c1 + 1) * res),
            'y0': float(oy + r0 * res), 'y1': float(oy + (r1 + 1) * res),
        }
    return out


def write_pgm(path: Path, img) -> None:
    """P5 trinary, byte-identical in form to save_map.py's output."""
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())


def write_png(path: Path, img) -> None:
    """The same array as the PGM, so the two cannot disagree.

    PIL is imported here rather than at module scope, the convention
    fit_world_transform.py follows, so the log half of this script does not
    depend on it.
    """
    from PIL import Image
    Image.fromarray(img, mode='L').save(path)


def write_map_yaml(path: Path, image_name: str, res: float, ox: float,
                   oy: float, yaw: float) -> None:
    """The .yaml nav2 needs beside the PGM, in save_map.py's exact format.

    Not asked for by R7, written anyway: a PGM alone carries no resolution and
    no origin, so without this file neither nav2 nor pgm_extent.py can say
    where the map is, and the extent printed below could not be checked
    against an independent reader.
    """
    with open(path, 'w') as f:
        f.write(f'image: {image_name}\nmode: trinary\nresolution: {res}\n')
        f.write(f'origin: [{ox}, {oy}, {yaw}]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def report_map(robot: int, found: dict | None, out_stem: Path) -> bool:
    """R7: write the run's final map out of the bag and describe it."""
    print('R7 THE FINAL MAP, OUT OF THE BAG')
    if found is None:
        print(f'  the bag carries no /robot_{robot}/map message, so the run '
              'produced no map to save')
        print('  -> FAIL\n')
        return False

    w, h, res = found['w'], found['h'], found['res']
    stats = map_stats(found['data'], w, h, res, found['ox'], found['oy'])
    img = grid_to_trinary(found['data'], w, h)

    out_stem.parent.mkdir(parents=True, exist_ok=True)
    # Appended, not with_suffix(): a stem such as b2maps_run1.2 is a run name,
    # not a file with a '.2' extension, and with_suffix() would eat it.
    pgm, png, yml = (Path(f'{out_stem}.pgm'), Path(f'{out_stem}.png'),
                     Path(f'{out_stem}.yaml'))
    write_pgm(pgm, img)
    write_png(png, img)
    write_map_yaml(yml, pgm.name, res, found['ox'], found['oy'], found['yaw'])

    print(f'  /robot_{robot}/map messages   {found["count"]:>8}  '
          f'(the LAST one is written)')
    print(f'  its header stamp             {found["stamp_s"]:>8.3f} s sim, '
          f'frame {found["frame_id"]}')
    print(f'  grid  {w}x{h} @ {res:g} m = {w * res:.2f} x {h * res:.2f} m')
    print(f'  origin ({found["ox"]:+.3f}, {found["oy"]:+.3f}) '
          f'yaw {math.degrees(found["yaw"]):+.3f} deg')
    if abs(found['yaw']) > 1e-9:
        print('  the origin is rotated, so the extents below are along the '
              'GRID axes, not the map frame axes')
    print(f'  cells  total {stats["total"]}   occupied {stats["occupied"]}   '
          f'free {stats["free"]}   unknown {stats["unknown"]}   '
          f'other {stats["other"]}')
    print(f'         "other" is the {FREE_TH_PCT + 1}..{OCC_TH_PCT - 1} band: '
          'drawn as unknown, but not unknown')

    box = stats['known_box']
    if box is None:
        print('  known extent   none -- every cell is unknown')
    else:
        print(f'  known extent   {box["width_m"]:.2f} x '
              f'{box["height_m"]:.2f} m  ({box["cols"]}x{box["rows"]} cells '
              f'of {stats["occupied"] + stats["free"]} known)')
        print(f'                 x [{box["x0"]:+.2f}, {box["x1"]:+.2f}]   '
              f'y [{box["y0"]:+.2f}, {box["y1"]:+.2f}]')
    print(f'  wrote  {pgm}')
    print(f'         {png}')
    print(f'         {yml}')
    print('  check it independently: python3 '
          f'experiments/analysis/pgm_extent.py {pgm}')
    print('  -> PASS\n')
    return True


# ──────────────────────────────── reporting ──────────────────────────────────

def report_health(found: dict, rates: dict, baseline_rates, verdicts,
                  window: dict | None, window_why: str | None = None):
    """R9: no consumer failed to get a transform inside the mapped window.

    The counts are split three ways -- by node, by the frame pair the line
    names, and by where the line fell against the window -- because the verdict
    turns on the last two and a reader has to be able to see it do so. Only
    final_gate()'s `gating` population decides anything; see WINDOW_GATED for
    which lines are candidates and drop_verdict() for how a drop leaves that
    population.
    """
    print('R9 TF TIMING HEALTH')
    if found['first'] is None:
        print('  no ROS timestamps in this log, so there is no runtime to '
              'divide by')
        print('  -> FAIL\n')
        return False

    print(f'  explorer runtime   {found["span_s"]:10.1f} s '
          f'({found["span_s"] / 60.0:.2f} min), '
          f'{found["stamped"]} timestamped lines')
    print(f'  first stamp        {found["first"]:.6f}')
    print(f'  last stamp         {found["last"]:.6f}')
    if window is None:
        print(f'  NO ANALYSIS WINDOW: {window_why}')
    else:
        print(f'  gated window       [{window["recv_lo"]:.6f}, '
              f'{window["recv_hi"]:.6f}] wall')
        print(f'                     = sim [{window["sim_lo"]:.3f}, '
              f'{window["sim_hi"]:.3f}], {window["min_sim"]:g} s of sim after '
              f'the first command on {window["cmd_topic"]}')
        print('                     (Part 1\'s window, read once from the bag '
              'and shared with R4)')
    print()
    head = f'  {"":<38}{"count":>8}{"per min":>10}'
    if baseline_rates is not None:
        head += f'{"limit":>10}{"":>10}'
    print(head)

    for key, spec in COUNTS.items():
        line = (f'  {spec["label"]:<38}{found["counts"][key]:>8}'
                f'{rates[key]:>10.3f}')
        if baseline_rates is not None:
            if key in VS_BASELINE:
                within, limit = verdicts[key]
                line += f'{limit:>10.3f}{"within" if within else "OVER":>10}'
            else:
                line += f'{"-":>10}{"":>10}'
        print(line)
        if not found['counts'][key]:
            continue
        for proc, n in sorted(found['by_proc'][key].items(),
                              key=lambda kv: -kv[1]):
            print(f'      node  {proc:<45}{n:>4}')
        # The pair split for every category the gate classifies, so that
        # "advisory" and "gated" are visible per line rather than inferred, and
        # for the two purely reported ones only if a pair was actually named.
        # 'Failed to make progress' names no frames and never will.
        if key in VS_BASELINE or any(found['by_pair'][key]):
            for pair, n in sorted(found['by_pair'][key].items(),
                                  key=lambda kv: -kv[1]):
                if key == 'slam_drop':
                    note = "advisory: slam_toolbox's own queue"
                elif is_slam_output_pair(pair):
                    note = "advisory: slam_toolbox's own map <-> odom"
                elif key in WINDOW_GATED:
                    note = ('gates when inside the window' if pair else
                            'no pair named: fail closed, gates when inside')
                else:
                    note = 'reported, never gated'
                print(f'      pair  {pair or "(none named)":<45}{n:>4}  '
                      f'{note}')
        p = found['placed'][key]
        print(f'      where inside the window {p["in"]}, outside it '
              f'{p["out"]}, unstamped {p["unstamped"]}')

    if baseline_rates is None:
        print('\n  no --baseline-log given, so no rate comparison. The gate '
              'below does not need one.')
    else:
        print(f'\n  the limit column is {FAIL_FACTOR:g}x the baseline rate, '
              'REPORTED and not gated: which way it')
        print('  goes depends on which log is the baseline (see VS_BASELINE), '
              'not on this run.')

    print('\n  gate, inside the window only:')
    print('    a TRANSFORM WARNING gates unless it names a same-namespace '
          'map <-> odom pair.')
    print('      A pair that cannot be parsed gates -- fail closed.')
    print('    a MESSAGE-FILTER DROP gates only if a link in the chain was '
          'LATE: its first')
    print('      transform covering the dropped message reached the wire '
          'after this line was')
    print('      logged, or never. map <-> odom being the only late link is '
          'advisory, on')
    print('      the same grounds as above; an unattributed drop gates -- '
          'fail closed. The')
    print('      neighbour lags below are scale for a reader, compared '
          'against nothing.')
    print('    slam_toolbox\'s own drops never gate.')
    if window is None:
        print('  a warning cannot be placed in a window that could not be '
              'read, so nothing is gated')
        print('  -> NOT EVALUATED\n')
        return None

    gating, advisory = final_gate(found['candidates'])
    for label, population in (('GATED   ', gating),
                              ('advisory', advisory)):
        for hit, why in population[:10]:
            print(f'  {label}  {hit["stamp"]:.6f}  {hit["proc"]}  {why}')
            _print_attribution(hit)
            print(f'            {hit["line"]}')
        if len(population) > 10:
            print(f'  ... and {len(population) - 10} more')
    if not gating:
        print('  nothing that gates fell inside the window')
    ok = not gating
    print(f'  -> {"PASS" if ok else "FAIL"}\n')
    return ok


def _print_attribution(hit: dict) -> None:
    """The per-link evidence behind a drop's verdict, when there is any."""
    row = hit.get('attr')
    if row is None:
        if hit['cat'] in DROP_CATS:
            print('            not attributed'
                  + (f': {hit["attr_why"]}' if hit.get('attr_why') else ''))
        return
    print(f'            chain {row["chain"][0][1]} up to '
          f'{row["chain"][-1][0]}, {len(row["links"])} dynamic link(s), '
          f'message stamped {row["sim"]:.3f} s sim')
    for link in row['links']:
        d = row['attribution'][link]
        base = row['base'][link]
        state = 'LATE' if d['late'] else 'ok  '
        if d['missing']:
            print(f'            {state} {link[0]} -> {link[1]}: no transform '
                  'stamped at or after it in the region read')
            continue
        spread = ('' if not base else
                  f', neighbours {statistics.median(base):+.3f} / '
                  f'{max(base):+.3f} s')
        print(f'            {state} {link[0]} -> {link[1]}: first covers '
              f'{d["stamp"]:.3f}, on the wire {d["vs_drop_s"]:+.3f} s vs this '
              f'line{spread}')


def report_coverage(bag: Path, robot: int, xy, rects, reach: float,
                    reach_src: str) -> bool:
    print('COVERAGE -- the true trajectory reaches the outer boundary')
    print(f'  lidar maximum range {reach:.2f} m  ({reach_src})')
    print(f'  boundary walls      {", ".join(BOUNDARY_WALLS)}  '
          f'({WORLD_SDF.name})')
    print(f'  truth samples       {len(xy)}  from /model/robot_{robot}/pose')

    if len(xy) == 0:
        print('  no truth samples, so closest approach is undefined')
        print('  -> FAIL\n')
        return False

    d = closest_approach(xy, rects)
    i = int(np.argmin(dist_to_nearest_wall(xy, rects)))
    ok = d <= reach
    print(f'  closest approach    {d:.3f} m  at ({xy[i][0]:+.3f}, '
          f'{xy[i][1]:+.3f})')
    print(f'  need                <= {reach:.2f} m at least once  '
          f'{"ok" if ok else "FAIL"}')
    if not ok:
        print(f'  the explorer never came within lidar range of the '
              f'perimeter: it missed by {d - reach:.3f} m.')
    print(f'  -> {"PASS" if ok else "FAIL"}\n')
    return ok


def summarise(results: dict) -> int:
    """The exit code for a set of gate verdicts. Pure.

    NOT EVALUATED is not a pass and not a failure: it is its own outcome with
    its own code, because a script chaining on the status must not read a bag
    that could not be judged as a green one. A FAIL outranks a skip -- if
    anything failed, 1 is the louder and truer answer.
    """
    if any(v is False for v in results.values()):
        return 1
    if any(v is None for v in results.values()):
        return 3
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--log', type=Path,
                    help='explore.launch.py console log to count warnings in')
    ap.add_argument('--baseline-log', type=Path,
                    help='log whose rates are printed beside this run\'s (the '
                         'run being replaced). ADVISORY: R9\'s verdict is the '
                         'window gate and does not need it.')
    ap.add_argument('--bag', type=Path,
                    help='run bag holding /model/robot_K/pose')
    ap.add_argument('--robot', type=int, default=0,
                    help='the explorer K (default %(default)s)')
    ap.add_argument('--factor', type=float, default=FAIL_FACTOR,
                    help='multiple of baseline allowed (default %(default)s)')
    ap.add_argument('--truth-tf', action='store_true',
                    help='also run R4/R5: the recorded odom->base_footprint '
                         'must BE inv(spawn).truth, and /robot_K/odom must '
                         'still be independent wheel odometry. Only '
                         'meaningful on a bag recorded with '
                         'truth_odom_robots:=[K].')
    ap.add_argument('--tf-rates', action='store_true',
                    help='also run R2: the sim-time rate of robot K\'s '
                         'odom->base_footprint in /tf must be the truth '
                         'node\'s, and a parked robot\'s must still be '
                         "DiffDrive's -- which is what names the owner, since "
                         'tf2 messages carry no publisher identity.')
    ap.add_argument('--parked', type=int,
                    help="R2's control robot, one that is still on DiffDrive "
                         '(default: the lowest robot id that is not --robot)')
    ap.add_argument('--save-map', type=Path, metavar='STEM',
                    help='also run R7: write the LAST /robot_K/map message in '
                         'the bag to STEM.pgm, STEM.png and STEM.yaml and '
                         'report its known extent and cell counts. Give the '
                         'path WITHOUT an extension.')
    ap.add_argument('--spawn-rev', default='08617b2',
                    help='git rev whose DOT_POSES gives the anchor '
                         '(default %(default)s, the b18/b2maps era)')
    ap.add_argument('--rig-lag', action='store_true',
                    help='also run R10: how long robot K\'s odom -> '
                         'base_footprint transforms take to reach the wire '
                         'inside the window, and how stale the newest one '
                         'ever got. REPORT ONLY -- it gates nothing and does '
                         'not affect the exit status.')
    ap.add_argument('--nav2-params', type=Path,
                    help="the run's nav2 params, whose costmap global_frame "
                         "R9 needs to attribute a message-filter drop "
                         '(default: experiments/nav/nav2_robotK.yaml, which '
                         "is explore.launch.py's own default)")
    ap.add_argument('--min-sim', type=float, default=MIN_SIM_S,
                    dest='min_sim',
                    help='SIM seconds after robot K\'s first nonzero command '
                         'that R4 gates: the span the offline map is built '
                         'from (default %(default)s, the b2maps cut). Same '
                         'name and meaning as check_run_bag.py\'s --min-sim, '
                         'whose own default is 2400.')
    args = ap.parse_args()

    if not args.log and not args.bag:
        ap.error('give --log, --bag, or both')
    if not 0 <= args.robot < NUM_ROBOTS:
        die(f'--robot {args.robot} is out of range 0..{NUM_ROBOTS - 1}')

    # R2's control has to be a robot that still owns its own DiffDrive
    # transform, so it cannot be the explorer.
    parked = args.parked
    if parked is None:
        parked = next(n for n in range(NUM_ROBOTS) if n != args.robot)
    if not 0 <= parked < NUM_ROBOTS:
        die(f'--parked {parked} is out of range 0..{NUM_ROBOTS - 1}')
    if parked == args.robot:
        die(f'--parked {parked} is the explorer itself; R2 needs a SECOND '
            'robot, one still on DiffDrive, as its control')
    if (args.tf_rates or args.save_map) and not args.bag:
        ap.error('--tf-rates and --save-map read the bag, so both need --bag')

    facts = rate_facts()
    print(f'transform rate being replaced: DiffDrive '
          f'{facts["diffdrive_hz"]:g} Hz')
    if facts['diffdrive_ignored']:
        print(f'    the burger SDF says {facts["diffdrive_nominal_hz"]:g} Hz '
              f'({facts["diffdrive_src"]}) but spells the element '
              '<odom_publisher_frequency>;')
        print('    DiffDrive reads <odom_publish_frequency>, so that line is '
              'inert and the plugin default stands')
        print('    (b2maps_e0 measured /robot_0/odom at 50.0 Hz against its '
              'own /clock)')
    print(f'transform rate replacing it:   truth_odom_tf '
          f'{facts["truth_hz"]:g} Hz, the PosePublisher rate '
          f'({facts["truth_src"]})')
    print(f'    a factor of '
          f'{facts["diffdrive_hz"] / facts["truth_hz"]:.2g} fewer transforms, '
          'which is what R9 is gating\n')

    results = {}

    # ONE window, read once, for both R4 and R9: the sim half places
    # transforms, the receive half places log lines, and they have to be the
    # same span or the two gates are judging different runs. Read before any of
    # the heavy passes -- it costs ~4 s (cmd_vel, then /clock only as far as
    # the window's end) against ~40 s for /tf.
    window = window_why = None
    if args.bag and (args.log or args.truth_tf or args.rig_lag):
        window, window_why = read_analysis_window(args.bag, args.robot,
                                                  args.min_sim)
    elif args.log:
        window_why = ('no --bag given, and the two things that place the '
                      'window -- the first nonzero command and /clock -- are '
                      'only in the bag')

    if args.log:
        found = count_health(read_log(args.log), window)
        # A drop is judged on the bag, so the attribution has to be read before
        # the verdict. Costs nothing on a run with no drop inside its window.
        if args.bag and window is not None:
            attribute_drops(args.bag, args.robot, found['candidates'],
                            args.nav2_params)
        rates = per_minute(found['counts'], found['span_s'])
        baseline_rates = verdicts = None
        if args.baseline_log:
            # The baseline is a log without a bag beside it here, so its counts
            # are over its whole runtime: a rate to compare, never a gate.
            base = count_health(read_log(args.baseline_log))
            baseline_rates = per_minute(base['counts'], base['span_s'])
            verdicts = judge(rates, baseline_rates, args.factor)
            print(f'baseline: {args.baseline_log} '
                  f'({base["span_s"] / 60.0:.2f} min)')
            for key in VS_BASELINE:
                print(f'  {COUNTS[key]["label"]:<38}'
                      f'{base["counts"][key]:>8}{baseline_rates[key]:>10.3f}'
                      ' per min')
            print()
        results['R9'] = report_health(found, rates, baseline_rates, verdicts,
                                      window, window_why)

    if args.bag and args.tf_rates:
        pair_k = (f'robot_{args.robot}/odom',
                  f'robot_{args.robot}/base_footprint')
        pair_p = (f'robot_{parked}/odom', f'robot_{parked}/base_footprint')
        stamps, sim_span, seen = read_tf_pair_stamps(args.bag,
                                                     [pair_k, pair_p])
        results['R2'] = report_tf_rates(
            args.robot, parked, tf_rate(stamps[pair_k], sim_span),
            tf_rate(stamps[pair_p], sim_span), facts, sim_span, seen)

    if args.bag and args.truth_tf:
        spawn = resolve_spawn_poses(args.spawn_rev)
        sx, sy = spawn[args.robot]
        truth, tf_pairs, wheel, _ = read_truth_tf_series(args.bag, args.robot)
        results['R4/R5'] = report_truth_tf(
            args.robot, (sx, sy, 0.0), truth, tf_pairs, wheel,
            facts['truth_hz'], window, window_why)

    # R10 is deliberately NOT in `results`: it has no verdict to contribute, so
    # it can neither pass nor be NOT EVALUATED. A reporting block that quietly
    # entered the summary as a pass would be the worst of both.
    if args.bag and args.rig_lag:
        from attribute_tf_drop import nav2_params_path, tolerance_facts
        params = args.nav2_params or nav2_params_path(args.robot)
        if window is None:
            print('R10 IS THE RIG TRANSFORM ITSELF ON TIME?')
            print(f'  no analysis window: {window_why}')
            print('  -> REPORTED (nothing)\n')
        else:
            tf_rows, clock = read_rig_tf(args.bag, args.robot, window)
            starts = recovery_starts(read_log(args.log)) if args.log else []
            report_rig_timeliness(
                args.robot, window,
                rig_timeliness(tf_rows, clock, window['sim_lo'],
                               window['sim_hi']),
                tolerance_facts(params), starts, facts['truth_hz'], params)

    if args.bag and args.save_map:
        results['R7'] = report_map(args.robot,
                                   read_last_map(args.bag, args.robot),
                                   args.save_map)

    if args.bag:
        reach, reach_src = lidar_max_range()
        results['COVERAGE'] = report_coverage(
            args.bag, args.robot, read_truth_xy(args.bag, args.robot),
            boundary_rects(), reach, reach_src)

    failed = [k for k, v in results.items() if v is False]
    skipped = [k for k, v in results.items() if v is None]
    print('=' * 70)
    for k, v in results.items():
        print(f'  {k:<10}{_verdict(v)}')
    if skipped and not failed:
        print(f'\nno gate failed, but {", ".join(skipped)} could not run -- '
              'the run is not verified, and this is not a pass')
    code = summarise(results)
    print(f'exit {code}')
    print('=' * 70)
    return code


if __name__ == '__main__':
    sys.exit(main())
