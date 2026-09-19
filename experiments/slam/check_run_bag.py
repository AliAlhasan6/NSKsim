#!/usr/bin/env python3
"""Check a single-explorer run bag before anything is built from it.

Four checks, run against a bag recorded by record_run.sh with EXPLORER=K:

  C1 COMPLETE   every topic that mode records is present with a nonzero count
  C2 SPAWN      robot K's FIRST truth pose is its DOT_POSES spawn pose
  C3 PROLOGUE   the bag starts before robot K was ever commanded to move
  C4 BUDGET     it then holds enough exploring to map from, and says where
                to cut: the --start/--duration for
                strip_bag_for_offline_slam.py

C2 and C3 are the same requirement approached from two sides, and both exist
because b18_run2 failed it: recording there began 603 s of sim time after the
sim did, with robot_0 already 3.075 m and -60.9 deg from spawn. A bag like
that cannot anchor a map, and nothing downstream notices -- the fitter happily
reports a score for a map placed 3 m wrong.

All four run, each reports, and the exit status is nonzero if any FAILED. A
check whose inputs C1 found missing reports NOT EVALUATED rather than
crashing: one missing topic should not hide the state of the other two.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 experiments/slam/check_run_bag.py --bag <dir> --robot K \
        [--spawn-rev 08617b2] [--budget 2400]

Reading a bag needs the ROS environment, so rosbag2_py is imported inside the
function that uses it -- the convention bag_overlap.py and
fit_world_transform.py already follow.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

from bag_overlap import (  # noqa: E402
    NUM_ROBOTS,
    die,
    read_first_truth_poses,
    resolve_spawn_poses,
)

# ── C2/C3 tolerances ─────────────────────────────────────────────────────────
# Tighter than bag_overlap's SPAWN_TOL_M (0.05), deliberately. That one gates
# "is this the right DOT_POSES era?", where the eras are 3.15 m apart and 5 cm
# is ample. This one gates "had the robot moved yet?", where the measured
# agreement for a genuinely parked robot was 1.6 mm -- so 1 cm is loose enough
# to absorb that and tight enough to catch a robot that has started rolling.
SPAWN_TOL_M = 0.01
SPAWN_TOL_DEG = 0.5

# ── C3: what counts as "commanded to move" ───────────────────────────────────
# A Twist counts as nonzero when |linear.x| or |angular.z| exceeds this.
#
# 1e-3 sits two orders of magnitude below anything that moves this robot and
# comfortably above float residue. The burger's limits are 0.22 m/s and
# 1.0 rad/s (velocity_smoother max_velocity in the nav2 params), so 1e-3 rad/s
# is 0.1 % of full scale; over one 50 ms control period it is 5e-5 rad, which
# is 0.003 deg -- far below C3's own 0.5 deg gate, so a command at this
# threshold cannot be what moved the robot. Above it because the smoother
# ramps toward zero in floating point and deadband_velocity is [0, 0, 0], so
# exact zeros are published but near-zeros can appear while stopping.
#
# y is ignored: the robot is differential-drive and the bridge's Twist carries
# linear.x and angular.z only.
CMD_EPS_LINEAR = 1e-3    # m/s
CMD_EPS_ANGULAR = 1e-3   # rad/s

# ── C3: what a clean prologue looks like ─────────────────────────────────────
# Long enough that a late cmd_vel subscription cannot masquerade as a quiet
# start, and sampled densely enough that "it did not move" rests on a series
# rather than one lucky instant. Truth arrives at 16.57 Hz, so 50 samples is
# about 3 s of coverage inside the 5 s window.
PROLOGUE_MIN_S = 5.0
PROLOGUE_MIN_POSES = 50

# ── C4: how much exploring the bag has to contain ────────────────────────────
# Offline mapping replays from the first command onward; everything before it
# is the robot sitting at spawn. 2400 s is the exploring the B2 maps need per
# robot. --budget overrides it for a rehearsal, where the point is the
# procedure rather than the coverage.
BUDGET_S = 2400.0


def expected_topics(k: int) -> list[str]:
    """Exactly what record_run.sh records with EXPLORER=k. Keep in step."""
    topics = ['/clock', '/tf', '/tf_static', '/kg_share', '/nsk/convergence']
    for n in range(NUM_ROBOTS):
        topics += [f'/robot_{n}/odom', f'/robot_{n}/scan',
                   f'/model/robot_{n}/pose', f'/robot_{n}/joint_states']
    topics += [f'/robot_{k}/map', f'/robot_{k}/map_metadata',
               f'/robot_{k}/cmd_vel']
    return topics


def read_metadata(bag: Path) -> tuple[dict[str, int], float, float]:
    """(message count per topic, bag start, bag end), both in seconds."""
    meta_path = bag / 'metadata.yaml'
    if not meta_path.is_file():
        die(f'no metadata.yaml in {bag} -- not a rosbag2 directory')
    info = yaml.safe_load(meta_path.read_text())['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: int(t['message_count'])
              for t in info['topics_with_message_count']}
    start_ns = int(info['starting_time']['nanoseconds_since_epoch'])
    end_ns = start_ns + int(info['duration']['nanoseconds'])
    return counts, start_ns / 1e9, end_ns / 1e9


def strip_keep_topics(robots: int = NUM_ROBOTS) -> set[str]:
    """The topics strip_bag_for_offline_slam.py keeps (that script, lines 63-65).

    Mirrored rather than imported: that script imports rosbag2_py at module
    level, so importing it would drag the ROS environment into every use of
    this one. If its `keep` set changes, this must change with it -- C4's
    offset is only in the strip script's convention while the two agree.
    """
    keep = {'/clock', '/tf', '/tf_static'}
    for n in range(robots):
        keep |= {f'/robot_{n}/scan', f'/robot_{n}/odom'}
    return keep


def strip_reference_time(bag: Path) -> float | None:
    """Stamp of the first message strip_bag_for_offline_slam.py would keep.

    THE reference for C4's offset. That script's --start is "bag seconds from
    the first kept message" (its docstring, line 17, and --start's own help at
    line 51), and it sets bag_t0 from the first message off a reader already
    filtered to the keep set (lines 75, 103-104). So the reference is the
    first message on one of those 13 topics, NOT the bag's first message and
    NOT metadata's starting_time, which count all 28.

    They coincide whenever the bag opens on a kept topic -- /clock usually
    wins, and did on b2maps_rehearsal3 to 0.000000 s. They diverge if a bag
    ever opens on /model/robot_N/pose, /robot_N/joint_states, a map topic or
    cmd_vel, none of which the strip script keeps. main() reports the gap
    rather than assuming it away.
    """
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    available = {t.name for t in reader.get_all_topics_and_types()}
    wanted = sorted(strip_keep_topics() & available)
    if not wanted:
        return None
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    if not reader.has_next():
        return None
    _topic, _data, ns = reader.read_next()
    return ns / 1e9


def read_prologue_series(bag: Path, k: int) -> tuple[list, list]:
    """(cmd series, pose series) for robot k, on the bag's own clock.

    cmd  -> [(t_s, linear_x, angular_z), ...]
    pose -> [(t_s, x, y, yaw), ...]

    Timestamps are the bag's receive stamps for both topics, so the two series
    share one clock and are directly comparable to the bag start in
    read_metadata(). Header stamps would be sim time on one topic and
    whatever the bridge stamped on the other.
    """
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.serialization import deserialize_message

    cmd_topic = f'/robot_{k}/cmd_vel'
    pose_topic = f'/model/robot_{k}/pose'

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    available = {t.name for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (cmd_topic, pose_topic) if t in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    cmds, poses = [], []
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        t = recv_ns / 1e9
        if topic == cmd_topic:
            m = deserialize_message(data, Twist)
            cmds.append((t, m.linear.x, m.angular.z))
        else:
            m = deserialize_message(data, PoseStamped)
            p, q = m.pose.position, m.pose.orientation
            poses.append((t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))
    return cmds, poses


def first_nonzero_cmd(cmds) -> float | None:
    """Stamp of the first command that could have moved the robot."""
    for t, lin, ang in cmds:
        if abs(lin) > CMD_EPS_LINEAR or abs(ang) > CMD_EPS_ANGULAR:
            return t
    return None


def check_prologue(bag_start: float, cmds, poses, spawn_xy) -> tuple[bool, list]:
    """C3, as a pure function so it can be tested on synthetic series.

    The interval runs from the bag's first message to the first nonzero
    command. Over ALL of it, robot K must still be sitting on its spawn pose:
    a maximum over the window, not a reading at one instant. Yaw is checked as
    well as position because Nav2 can open with an in-place turn, which moves
    yaw and leaves position alone -- a position-only test would call that a
    quiet prologue and pass a bag whose cmd_vel subscription simply arrived
    late.
    """
    lines = []
    t_cmd = first_nonzero_cmd(cmds)
    if t_cmd is None:
        lines.append('  no nonzero command anywhere in the bag '
                     f'(threshold {CMD_EPS_LINEAR} m/s, {CMD_EPS_ANGULAR} rad/s)')
        lines.append('  robot K was never commanded to move -- this is not a '
                     'run that explored')
        return False, lines

    span = t_cmd - bag_start
    window = [p for p in poses if bag_start <= p[0] <= t_cmd]
    lines.append(f'  first nonzero command at bag t={span:.2f} s '
                 f'(threshold {CMD_EPS_LINEAR} m/s, {CMD_EPS_ANGULAR} rad/s)')

    long_enough = span >= PROLOGUE_MIN_S
    lines.append(f'  prologue length   {span:8.2f} s  (need >= '
                 f'{PROLOGUE_MIN_S:.1f})  {_verdict(long_enough)}')

    dense_enough = len(window) >= PROLOGUE_MIN_POSES
    lines.append(f'  pose samples      {len(window):8d}    (need >= '
                 f'{PROLOGUE_MIN_POSES})  {_verdict(dense_enough)}')

    if not window:
        lines.append('  no truth samples inside the prologue, so "did it move" '
                     'cannot be answered')
        return False, lines

    max_d = max(math.hypot(x - spawn_xy[0], y - spawn_xy[1])
                for _t, x, y, _yaw in window)
    max_yaw = max(abs(math.degrees(_wrap(yaw))) for _t, _x, _y, yaw in window)
    still_d = max_d <= SPAWN_TOL_M
    still_y = max_yaw <= SPAWN_TOL_DEG
    lines.append(f'  max displacement  {max_d * 100:8.2f} cm (need <= '
                 f'{SPAWN_TOL_M * 100:.0f})  {_verdict(still_d)}')
    lines.append(f'  max |yaw|         {max_yaw:8.3f} deg(need <= '
                 f'{SPAWN_TOL_DEG})  {_verdict(still_y)}')

    ok = long_enough and dense_enough and still_d and still_y
    if not ok:
        lines.append('  recording did not start before robot K moved. The '
                     'bag cannot anchor a map at spawn.')
    return ok, lines


def check_budget(strip_t0: float, bag_end: float, t_cmd: float | None,
                 budget: float) -> tuple[bool, list, float | None]:
    """C4, pure so it can be tested without a bag.

    Returns (ok, report lines, offset). The offset is in
    strip_bag_for_offline_slam.py's convention -- seconds from the first
    message that script would keep -- so it can be handed straight to its
    --start. Everything before it is the robot sitting at spawn, which is
    exactly what the offline replay should skip.
    """
    lines = []
    if t_cmd is None:
        lines.append('  no nonzero command in the bag, so there is nothing to '
                     'replay from')
        return False, lines, None

    offset = t_cmd - strip_t0
    remaining = bag_end - t_cmd
    ok = remaining >= budget

    lines.append(f'  first nonzero command at {offset:.2f} s '
                 '(strip convention: from the first kept message)')
    lines.append(f'  bag ends              {bag_end - strip_t0:.2f} s')
    lines.append(f'  usable after it       {remaining:8.2f} s  (need >= '
                 f'{budget:.0f})  {_verdict(ok)}')
    if ok:
        lines.append('  strip this segment with:')
        lines.append(f'    --start {offset:.2f} --duration {budget:.0f}')
    else:
        lines.append(f'  short by {budget - remaining:.2f} s. The run stopped '
                     'too early to map from; record a longer one rather than '
                     'lowering the budget.')
    return ok, lines, offset


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading about Z alone -- NOT the quaternion's total rotation angle.

    That distinction decides C2 and C3. A burger at rest on this world's floor
    sits at a pitch of about -0.71 deg (measured on b2maps_rehearsal3: the
    first /model/robot_0/pose carries y=-0.0062281, w=0.999980605), which is
    suspension settle, not heading. Its total rotation angle, 2*acos(w), is
    therefore 0.714 deg and would blow the 0.5 deg limit on tilt alone -- both
    checks would fail every bag, on every robot, for a robot that had not
    turned at all.

    This is the same expression bag_overlap.read_first_truth_poses() uses
    (bag_overlap.py:186-187), which is where C2 gets its yaw; keeping one
    formula is why C2 and C3 cannot disagree about what "turned" means.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(rad: float) -> float:
    return (rad + math.pi) % (2 * math.pi) - math.pi


def _verdict(ok: bool) -> str:
    return 'ok' if ok else 'FAIL'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', required=True, help='bag directory to check')
    ap.add_argument('--robot', type=int, required=True,
                    help='the explorer K this bag was recorded for')
    ap.add_argument('--spawn-rev', default='08617b2',
                    help='git rev whose DOT_POSES gives the spawn poses '
                         '(default %(default)s, the b18 era)')
    ap.add_argument('--budget', type=float, default=BUDGET_S,
                    help='seconds of exploring C4 requires after the first '
                         'command (default %(default)s)')
    args = ap.parse_args()

    if not 0 <= args.robot < NUM_ROBOTS:
        die(f'--robot {args.robot} is out of range 0..{NUM_ROBOTS - 1}')
    bag = Path(args.bag)
    k = args.robot

    print(f'checking {bag} as a single-explorer run, robot_{k}\n')
    counts, bag_start, bag_end = read_metadata(bag)
    results = {}

    cmd_topic = f'/robot_{k}/cmd_vel'
    pose_topic = f'/model/robot_{k}/pose'
    # One pass for C3 and C4, which want the same two series.
    cmds, poses = [], []
    if counts.get(cmd_topic, 0) or counts.get(pose_topic, 0):
        cmds, poses = read_prologue_series(bag, k)

    # ── C1 COMPLETE ──
    want = expected_topics(k)
    missing = [t for t in want if counts.get(t, 0) == 0]
    print(f'C1 COMPLETE -- all {len(want)} recorded topics present and nonempty')
    for t in want:
        n = counts.get(t, 0)
        print(f'  {n:9d}  {t}' + ('   MISSING' if n == 0 else ''))
    if missing:
        print(f'  {len(missing)} topic(s) missing or empty:')
        for t in missing:
            print(f'    {t}')
    results['C1'] = not missing
    print(f'  -> {"PASS" if not missing else "FAIL"}\n')

    # ── C2 SPAWN ──
    print(f'C2 SPAWN -- robot_{k} first truth pose within '
          f'{SPAWN_TOL_M * 100:.0f} cm and {SPAWN_TOL_DEG} deg of '
          f'DOT_POSES @ {args.spawn_rev}')
    if counts.get(pose_topic, 0) == 0:
        print(f'  {pose_topic} is missing (C1), so there is no first truth '
              'pose to test')
        print('  -> NOT EVALUATED\n')
        results['C2'] = None
    else:
        spawn = resolve_spawn_poses(args.spawn_rev)
        first_pose, _max_disp = read_first_truth_poses(bag)
        x, y, yaw = first_pose[k]
        # resolve_spawn_poses drops the DOT_POSES yaw and verifies the launch
        # file passes no '-Y' at that revision, so the spawn yaw IS zero and
        # the angular half of this check is "truth yaw within tolerance of 0".
        # It is a real check only because of that verification.
        d = math.hypot(x - spawn[k][0], y - spawn[k][1])
        dyaw = abs(math.degrees(_wrap(yaw)))
        ok = d <= SPAWN_TOL_M and dyaw <= SPAWN_TOL_DEG
        print(f'  truth     ({x:+.3f}, {y:+.3f}) yaw {math.degrees(yaw):+.2f} deg')
        print(f'  DOT_POSES ({spawn[k][0]:+.3f}, {spawn[k][1]:+.3f}) yaw +0.00 deg')
        print(f'  off by    {d:.3f} m ({d * 100:.2f} cm), {dyaw:.2f} deg')
        if not ok:
            print('  recording began after robot K had already moved.')
        results['C2'] = ok
        print(f'  -> {"PASS" if ok else "FAIL"}\n')

    # ── C3 PROLOGUE ──
    print(f'C3 PROLOGUE -- bag starts >= {PROLOGUE_MIN_S:.0f} s and '
          f'{PROLOGUE_MIN_POSES} pose samples before robot_{k} was commanded')
    if counts.get(cmd_topic, 0) == 0 or counts.get(pose_topic, 0) == 0:
        absent = [t for t in (cmd_topic, pose_topic) if counts.get(t, 0) == 0]
        print(f'  {", ".join(absent)} missing (C1), so the prologue cannot be '
              'measured')
        print('  -> NOT EVALUATED\n')
        results['C3'] = None
    else:
        spawn = resolve_spawn_poses(args.spawn_rev)
        ok, lines = check_prologue(bag_start, cmds, poses, spawn[k])
        for line in lines:
            print(line)
        results['C3'] = ok
        print(f'  -> {"PASS" if ok else "FAIL"}\n')

    # ── C4 BUDGET ──
    print(f'C4 BUDGET -- at least {args.budget:.0f} s of bag after robot_{k} '
          'was first commanded')
    if counts.get(cmd_topic, 0) == 0:
        print(f'  {cmd_topic} missing (C1), so the replay start is unknown')
        print('  -> NOT EVALUATED\n')
        results['C4'] = None
    else:
        strip_t0 = strip_reference_time(bag)
        if strip_t0 is None:
            print('  the bag carries none of the topics '
                  'strip_bag_for_offline_slam.py keeps, so --start has no '
                  'reference')
            print('  -> NOT EVALUATED\n')
            results['C4'] = None
        else:
            skew = strip_t0 - bag_start
            if abs(skew) > 1e-6:
                print(f'  note: the first kept message is {skew:+.3f} s from '
                      "the bag's first message, so this offset is NOT the "
                      "same as C3's prologue length")
            ok, lines, _offset = check_budget(
                strip_t0, bag_end, first_nonzero_cmd(cmds), args.budget)
            for line in lines:
                print(line)
            results['C4'] = ok
            print(f'  -> {"PASS" if ok else "FAIL"}\n')

    print('=' * 70)
    for name in ('C1', 'C2', 'C3', 'C4'):
        state = {True: 'PASS', False: 'FAIL', None: 'NOT EVALUATED'}[results[name]]
        print(f'  {name}  {state}')
    failed = [n for n in results if results[n] is False]
    skipped = [n for n in results if results[n] is None]
    if failed:
        print(f'\n{len(failed)} check(s) FAILED: {", ".join(sorted(failed))}')
    elif skipped:
        print(f'\nno check failed, but {", ".join(sorted(skipped))} could not '
              'run -- the bag is not verified')
    else:
        print(f'\nall {len(results)} checks passed')
    print('=' * 70)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
