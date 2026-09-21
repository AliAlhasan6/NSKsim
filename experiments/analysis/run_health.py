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
  alone once collapsed /clock from 99 Hz to 0.5 Hz. So the run is gated on
  counted warnings per minute, against the same counts taken from the run
  being replaced.

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

Exit status is 0 when every gate that ran passed, 1 if any failed, 2 on an
input error. A gate whose input was not given does not run and does not
affect the status.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

from bag_overlap import (  # noqa: E402
    NUM_ROBOTS,
    WORLD_SDF,
    die,
    dist_to_nearest_wall,
    resolve_spawn_poses,
    world_walls_named,
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

# ── R4: how much of the recording edge is not a finding ──────────────────────
# A transform whose stamp lies OUTSIDE the recorded pose series is not evidence
# of a second publisher; it is usually evidence that rosbag2 attached to /tf
# and to /model/robot_K/pose at different moments. b2maps_relay1's record log:
#
#     [1789943096.804587] Subscribed to topic '/tf'
#     [1789943096.947029] Subscribed to topic '/model/robot_0/pose'
#
# 142.4 ms apart, which at the PosePublisher's 20 Hz is 2.85 sample periods --
# and exactly 3 transforms in that bag carry stamps (75.250, 75.300, 75.350)
# earlier than the first recorded pose (75.400). The order is a discovery race
# and swings both ways run to run: e0t +44.6 ms, e0 +2824 ms, and rehearsal5
# subscribed to the pose topic FIRST, at -224 ms, so it has no leading edge at
# all. Gating on it gates the recorder, not the transform.
#
# The grace is a count and a span, both in PosePublisher periods, at each end.
# 20 periods is 1.0 s at 20 Hz. Measured against the runs R4 actually applies
# to -- the truth_odom_tf ones -- that is about seven times the widest edge
# seen: relay1 3, e0t 1, rehearsal5 0.
#
# It is NOT seven times every attach gap this rig has produced. b2maps_e0 took
# 2824 ms to subscribe, which would be 57 periods; R4 does not run on it (e0 is
# the DiffDrive run being replaced, so it has no truth transform to check), but
# a truth run that attached that slowly WOULD fail here. That is the intended
# behaviour and not a number to raise on sight: an edge that long is no longer
# a negligible artefact, and the record log beside the bag says in two lines
# whether the recorder or a publisher produced it.
#
# The bound is deliberately belt-and-braces. Contiguity and an unbroken pose
# series already make a second publisher implausible here, and R2 catches one
# outright by reading the 70 Hz sum on the pair. What the bound adds is that
# "outside the pose series" cannot become an unexamined place to put
# transforms. The case R4 exists for puts them INSIDE the series, where the
# comparison is unchanged and unmatched is still a hard failure.
EDGE_GRACE_PERIODS = 20

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
# costmap layers, so the slam_toolbox count additionally requires the line to
# name slam_toolbox -- true of both the process tag
# ([async_slam_toolbox_node-1]) and the logger ([robot_0.slam_toolbox]).
GATED = ('nav2_transform', 'slam_drop')

COUNTS = {
    'nav2_transform': dict(
        label='Nav2/tf2 transform warnings',
        any_of=('Timed out waiting for transform', 'Transform data too old',
                'Lookup would require extrapolation', 'extrapolation into the'),
        require=None),
    'slam_drop': dict(
        label='slam_toolbox message-filter drops',
        any_of=('Message Filter dropping message',),
        require='slam_toolbox'),
    'no_progress': dict(
        label='controller "Failed to make progress"',
        any_of=('Failed to make progress',),
        require=None),
    'loop_rate': dict(
        label='control/planner loop missed its rate',
        any_of=('missed its desired rate',),
        require=None),
}

# Reported, never gated. The amendment gates the first two; 'no_progress' is a
# navigation outcome rather than a timing fault, and 'loop_rate' is a load
# symptom worth seeing next to the others on a rig with this one's history.
assert set(GATED) <= set(COUNTS)

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


def count_health(lines) -> dict:
    """Counts per category plus the log's timestamp span. Pure.

    Returns {'counts': {...}, 'by_proc': {cat: {proc: n}}, 'first', 'last',
             'span_s', 'stamped'}.
    """
    counts = {k: 0 for k in COUNTS}
    by_proc = {k: {} for k in COUNTS}
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
            if spec['require'] and spec['require'] not in line:
                continue
            if any(p in line for p in spec['any_of']):
                counts[key] += 1
                by_proc[key][proc] = by_proc[key].get(proc, 0) + 1

    span = (last - first) if (first is not None and last is not None) else 0.0
    return {'counts': counts, 'by_proc': by_proc, 'first': first,
            'last': last, 'span_s': span, 'stamped': stamped}


def per_minute(counts: dict, span_s: float) -> dict:
    """Counts -> rate per minute. A zero span yields inf for any nonzero count
    rather than a division error: a burst inside an instant is not healthy.
    """
    if span_s <= 0.0:
        return {k: (0.0 if v == 0 else math.inf) for k, v in counts.items()}
    return {k: v * 60.0 / span_s for k, v in counts.items()}


def judge(rates: dict, baseline: dict, factor: float = FAIL_FACTOR) -> dict:
    """Per gated category: (ok, limit). More than `factor` times baseline fails.

    A zero baseline makes the limit zero, so ANY occurrence fails. That is not
    a degenerate case to work around -- b2maps_e0 ran 98.96 minutes and logged
    not one transform warning, so zero is what this stack does when it is
    healthy, and the first one is the signal.
    """
    out = {}
    for key in GATED:
        limit = baseline.get(key, 0.0) * factor
        out[key] = (rates[key] <= limit, limit)
    return out


def read_log(path: Path):
    """Explore logs carry raw bytes from the child processes, so they are not
    reliably valid UTF-8 -- grep calls them binary. Decode permissively.
    """
    if not path.is_file():
        die(f'log not found: {path}')
    return path.read_text(errors='replace').splitlines()


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


def split_at_truth_span(tf_pairs, truth):
    """Recorded transforms partitioned by stamp against the truth series' span.

    Returns (before, inside, after), each a list of (stream index, row), where
    the stream index is the transform's position in the order it was recorded.
    The indices are what makes the contiguity test below possible: an edge is
    only an edge if it sits at an END of the stream.

    Pure. `tf_pairs` must be in recorded order, which read_truth_tf_series
    guarantees by appending as it reads.
    """
    if not truth or not tf_pairs:
        return [], [], []
    lo, hi = min(truth), max(truth)
    before, inside, after = [], [], []
    for i, row in enumerate(tf_pairs):
        ns = row[0]
        bucket = before if ns < lo else after if ns > hi else inside
        bucket.append((i, row))
    return before, inside, after


def truth_gaps(truth, truth_hz: float):
    """Adjacent truth stamps more than TRUTH_GAP_PERIODS apart. Pure.

    This is what separates the two readings of "the transform's stamp is
    outside the pose series". If the series is unbroken from its first sample
    to its last, then outside means the recorder had not attached yet. If the
    series has holes, poses went missing DURING the run and an unmatched
    transform can no longer be attributed to the edge -- so the edge grace
    below is withheld entirely rather than applied to a series that cannot
    support it.
    """
    period_ns = 1e9 / truth_hz
    ks = sorted(truth)
    return [(a, b) for a, b in zip(ks, ks[1:])
            if (b - a) > TRUTH_GAP_PERIODS * period_ns]


def judge_edge(edge, n_tf: int, truth_hz: float, leading: bool):
    """(ok, reason, span_periods) for one end's out-of-span transforms. Pure.

    Admissible only as a contiguous run at its own end of the stream, no longer
    than EDGE_GRACE_PERIODS in both count and span. A publisher that ran during
    the run cannot satisfy the first condition, and one that burst inside the
    attach window cannot satisfy the second.
    """
    if not edge:
        return True, 'none', 0.0
    idx = [i for i, _row in edge]
    want = (list(range(len(edge))) if leading
            else list(range(n_tf - len(edge), n_tf)))
    where = 'prefix' if leading else 'suffix'
    if idx != want:
        return (False,
                f'not a contiguous {where} of the recorded stream (indices '
                f'{idx[0]}..{idx[-1]} of {n_tf}), so these were published '
                'during the run, not before the recorder attached', 0.0)

    period_ns = 1e9 / truth_hz
    stamps = [row[0] for _i, row in edge]
    span = (max(stamps) - min(stamps)) / period_ns + 1.0
    if len(edge) > EDGE_GRACE_PERIODS:
        return (False, f'{len(edge)} transforms is more than the '
                       f'{EDGE_GRACE_PERIODS}-period grace', span)
    if span > EDGE_GRACE_PERIODS:
        return (False, f'spans {span:.1f} periods, more than the '
                       f'{EDGE_GRACE_PERIODS}-period grace', span)
    return True, 'within grace', span


def report_truth_tf(robot: int, anchor, truth, tf_pairs, wheel,
                    truth_hz: float) -> bool:
    """R4 and R5 together: the transform IS the truth, and the wheel odometry
    is still the wheel's.

    R4 compares each recorded robot_K/odom -> robot_K/base_footprint against
    inv(A) . P at ITS OWN header stamp. The node copies the pose's stamp
    verbatim, so a transform whose stamp is one the pose series carries must
    match it exactly, and one that falls BETWEEN two pose samples was published
    by something else -- a surviving DiffDrive broadcast being the case this
    guards. That comparison, and its 1e-6 tolerances, are unchanged.

    What changed (2026-09-21) is which transforms it is asked of. The rule used
    to require that EVERY recorded transform land on a pose stamp, and
    b2maps_relay1 failed it on 3 of 11271 while the other 11268 agreed to
    0.000 m and 2.5e-14 deg. Those 3 were the first three in the stream,
    contiguous, stamped 75.250/75.300/75.350 against a first recorded pose of
    75.400 -- the window in which the recorder held a /tf subscription and not
    yet a /model/robot_0/pose one. See EDGE_GRACE_PERIODS for the evidence.

    So the transforms are split at the pose series' span. INSIDE it the old
    rule stands unweakened: land on a stamp, match it to 1e-6, or fail. OUTSIDE
    it they are the recording edge, and are admitted only if they sit at an end
    of the stream, are short, and the pose series itself has no holes -- three
    conditions a second publisher cannot meet, because a publisher that ran
    during the run leaves transforms inside the span, where nothing here
    forgives them.

    Poses carrying no transform are counted and printed but not gated: a
    transform that was never published is a question about the rate, and R2
    already answers that against /clock.

    R5 is the C4 discriminator from rewrite_odom_from_truth, pointed the other
    way: if /robot_K/odom had quietly become a copy of the truth, the two yaw
    series would agree and this run would prove nothing about odometry.
    """
    print('R4 THE TRANSFORM IS THE TRUTH')
    print(f'  anchor A = spawn of robot_{robot}: ({anchor[0]:+.4f}, '
          f'{anchor[1]:+.4f}, {math.degrees(anchor[2]):+.4f} deg)')
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
              f'{a / 1e9:.3f} s sim')

    before, inside, after = split_at_truth_span(tf_pairs, truth)

    unmatched = 0
    worst_xy = worst_yaw = worst_z = 0.0
    for _i, (ns, x, y, yaw, z) in inside:
        want = truth.get(ns)
        if want is None:
            unmatched += 1
            continue
        ex, ey, eyaw = _inv_compose(anchor, want)
        worst_xy = max(worst_xy, abs(x - ex), abs(y - ey))
        worst_yaw = max(worst_yaw, abs(math.degrees(_wrap(yaw - eyaw))))
        worst_z = max(worst_z, abs(z))

    matched = len(inside) - unmatched
    print(f'  compared      {matched:>9}  transforms inside the pose series')
    print(f'  worst |dxy|   {worst_xy:.3e} m    (tolerance 1e-6)')
    print(f'  worst |dyaw|  {worst_yaw:.3e} deg  (tolerance 1e-6)')
    print(f'  worst |z|     {worst_z:.3e} m    (must be 0: planar transform)')
    print(f'  BETWEEN two pose samples      {unmatched:>9}  '
          '(must be 0: another publisher)')

    lead_ok, lead_why, lead_span = judge_edge(before, len(tf_pairs), truth_hz,
                                              leading=True)
    trail_ok, trail_why, trail_span = judge_edge(after, len(tf_pairs),
                                                 truth_hz, leading=False)
    print(f'  at the recording edge         '
          f'{len(before):>9} leading, {len(after)} trailing  '
          f'(grace {EDGE_GRACE_PERIODS} periods each)')
    for label, edge, why, span, passed in (
            ('leading ', before, lead_why, lead_span, lead_ok),
            ('trailing', after, trail_why, trail_span, trail_ok)):
        if not edge:
            continue
        stamps = [row[0] for _i, row in edge]
        print(f'      {label}  {min(stamps) / 1e9:.3f} .. '
              f'{max(stamps) / 1e9:.3f} s sim, {len(edge)} transforms over '
              f'{span:.1f} periods')
        print(f'                {"ok" if passed else "FAIL"}: {why}')
    if gaps and (before or after):
        print('      the pose series has holes, so an out-of-span transform '
              'cannot be charged to the recorder: grace WITHHELD')

    orphans = len(set(truth) - {row[0] for row in tf_pairs})
    print(f'  poses with no transform       {orphans:>9}  '
          '(reported, not gated: a missing transform is R2\'s rate question)')

    edges_ok = lead_ok and trail_ok and not (gaps and (before or after))
    ok = (unmatched == 0 and matched > 0 and worst_xy <= 1e-6
          and worst_yaw <= 1e-6 and worst_z == 0.0 and edges_ok)
    print(f'  -> {"PASS" if ok else "FAIL"}\n')

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
    return ok and wheel_ok


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

def report_health(found: dict, rates: dict, baseline_rates, verdicts) -> bool:
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
    print()
    head = f'  {"":<38}{"count":>8}{"per min":>10}'
    if baseline_rates is not None:
        head += f'{"limit":>10}{"":>8}'
    print(head)

    ok = True
    for key, spec in COUNTS.items():
        gated = key in GATED and baseline_rates is not None
        line = (f'  {spec["label"]:<38}{found["counts"][key]:>8}'
                f'{rates[key]:>10.3f}')
        if baseline_rates is not None:
            if gated:
                passed, limit = verdicts[key]
                line += f'{limit:>10.3f}{"ok" if passed else "FAIL":>11}'
                ok = ok and passed
            else:
                line += f'{"-":>10}{"ungated":>11}'
        print(line)
        for proc, n in sorted(found['by_proc'][key].items(),
                              key=lambda kv: -kv[1]):
            print(f'      {proc:<36}{n:>8}')

    if baseline_rates is None:
        print('\n  no --baseline-log given: counts reported, NOTHING GATED.')
        print('  -> NOT EVALUATED\n')
        return None

    print(f'\n  gate: more than {FAIL_FACTOR:g}x the baseline rate on '
          f'{" or ".join(GATED)} is a FAIL')
    print(f'  -> {"PASS" if ok else "FAIL"}\n')
    return ok


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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--log', type=Path,
                    help='explore.launch.py console log to count warnings in')
    ap.add_argument('--baseline-log', type=Path,
                    help='log whose rates set the limit (the run being '
                         'replaced). Without it --log reports but gates '
                         'nothing.')
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

    if args.log:
        found = count_health(read_log(args.log))
        rates = per_minute(found['counts'], found['span_s'])
        baseline_rates = verdicts = None
        if args.baseline_log:
            base = count_health(read_log(args.baseline_log))
            baseline_rates = per_minute(base['counts'], base['span_s'])
            verdicts = judge(rates, baseline_rates, args.factor)
            print(f'baseline: {args.baseline_log} '
                  f'({base["span_s"] / 60.0:.2f} min)')
            for key in GATED:
                print(f'  {COUNTS[key]["label"]:<38}'
                      f'{base["counts"][key]:>8}{baseline_rates[key]:>10.3f}'
                      ' per min')
            print()
        results['R9'] = report_health(found, rates, baseline_rates, verdicts)

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
            facts['truth_hz'])

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
        print(f'  {k:<10}'
              + {True: 'PASS', False: 'FAIL', None: 'NOT EVALUATED'}[v])
    if skipped and not failed:
        print(f'\nno gate failed, but {", ".join(skipped)} could not run -- '
              'the run is not verified')
    print('=' * 70)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
