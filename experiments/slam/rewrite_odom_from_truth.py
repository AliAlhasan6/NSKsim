#!/usr/bin/env python3
"""Rewrite a bag's odom->base_footprint TF from Gazebo ground-truth poses.

Why this exists (2026-09-18, the two-layer known-pose probe):
  b18_run2's robot_0 map, built on odometry, does not describe the world: it
  fits the known walls at 21.9%, which is noise level, and its map->odom ends
  4.76 m and +44.35 degrees from identity (arm A, 2026-09-19). Nobody has
  inspected that map's image, so how it fails -- rotated copies of the walls,
  or some other way -- is not known here. The duplicated walls are b16's
  finding, not this bag's.

  The hypothesis is that accumulated odometric drift feeds the scan matcher.
  This script builds the bag that tests it: the real scans, placed at ground
  truth instead of at odometry. Replayed through slam_toolbox with
  use_scan_matching:false, that is a deterministic known-pose mapper -- no
  matcher, no loop closure.

What it does:
  Removes every robot_N/odom -> robot_N/base_footprint transform from /tf and
  synthesises one per /model/robot_N/pose sample instead, so the bag's only
  authority on where the robot was is Gazebo.

The anchor:
  odom_T_base = inv(A) . P, with A the robot's SPAWN pose, read from DOT_POSES
  in the launch file at --spawn-rev. Anchoring on spawn -- rather than on the
  first recorded pose -- makes the rewritten odom frame BE the spawn frame,
  which is exactly what fit_world_transform.py assumes when it builds
  world_T_odom as a pure translation with spawn yaw 0. The fitted world pose
  and truth_anchor.txt are then two independently derived values that must
  agree, which is the check Part 3 rests on.

  On b18_run2 the first recorded pose is 3.075 m and -60.9 deg away from
  spawn: recording started 603 s of sim time after the sim did. Anchoring
  there would have put every map 3 m from where the robot actually started
  AND made C4 pass on the baked-in offset rather than on real drift.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 rewrite_odom_from_truth.py --in SRC --out DST --robot 0 \
        --spawn-rev 08617b2 [--dry-run]

Expected: one discovery pass, one rewrite pass, one verification pass;
roughly 10 min on b18_run2's 1.52 M messages.
"""
import argparse
import bisect
import math
import os
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

from bag_overlap import (  # noqa: E402
    BAG_ERA_REV,
    NUM_ROBOTS,
    check_spawn_against_truth,
    resolve_spawn_poses,
)

TF_TYPE = 'tf2_msgs/msg/TFMessage'
POSE_TYPE = 'geometry_msgs/msg/PoseStamped'


# ────────────────────────────── SE(2) helpers ───────────────────────────────

def yaw_of(q):
    """Quaternion -> yaw. Same form as analysis/tf_pair_at.py."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def se2_compose(a, b):
    """a . b, both (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def se2_inverse(a):
    ax, ay, at = a
    c, s = math.cos(-at), math.sin(-at)
    return (-(c * ax - s * ay), -(s * ax + c * ay), -at)


def wrap(a):
    """Wrap radians to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_se2(msg):
    p = msg.pose.position
    return (p.x, p.y, yaw_of(msg.pose.orientation))


def is_robot_odom_base(robot, parent, child):
    """Does this pair look like robot N's odom -> base_footprint?

    Matched on the frame-naming convention rather than on literal strings, so
    a renamed namespace is discovered rather than silently missed.
    """
    token = f'robot_{robot}'
    return (child.endswith('base_footprint') and parent.endswith('odom')
            and token in child and token in parent)


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# ───────────────────────────────── checks ───────────────────────────────────

def check_c1() -> bool:
    """C1: composition, synthetic, before any bag I/O.

    The expected value is hand-calculated, not produced by this code. The
    reversed composition gives (-2.190671, 0.448288, -45 deg), so an inverted
    operand order fails here rather than silently shifting every map.
    """
    A = (2.0, -1.0, math.radians(30.0))
    P = (3.0, 1.0, math.radians(75.0))
    want = (1.866025, 1.232051, math.radians(45.0))

    got = se2_compose(se2_inverse(A), P)
    err = max(abs(got[0] - want[0]), abs(got[1] - want[1]),
              abs(wrap(got[2] - want[2])))
    back = se2_compose(A, got)
    err_back = max(abs(back[0] - P[0]), abs(back[1] - P[1]),
                   abs(wrap(back[2] - P[2])))

    print('C1 composition (synthetic):')
    print(f'  inv(A) . P   = ({got[0]:.6f}, {got[1]:.6f}, {math.degrees(got[2]):.6f} deg)')
    print(f'  expected     = ({want[0]:.6f}, {want[1]:.6f}, {math.degrees(want[2]):.6f} deg)')
    print(f'  max error    = {err:.3e}   (tolerance 1e-6)')
    print(f'  A . result recovers P to {err_back:.3e}')
    if err > 1e-6 or err_back > 1e-6:
        print('  C1 FAILED', file=sys.stderr)
        return False
    print('  C1 ok')
    return True


def interp_yaw(samples_t, samples_y, t):
    """Shortest-arc interpolation of yaw at t. None outside the sample range."""
    if not samples_t or t < samples_t[0] or t > samples_t[-1]:
        return None
    i = bisect.bisect_left(samples_t, t)
    if i == 0:
        return samples_y[0]
    if samples_t[i - 1] == t:
        return samples_y[i - 1]
    t0, t1 = samples_t[i - 1], samples_t[i]
    y0, y1 = samples_y[i - 1], samples_y[i]
    if t1 == t0:
        return y0
    return y0 + wrap(y1 - y0) * (t - t0) / (t1 - t0)


# ──────────────────────────────── pass 1 ────────────────────────────────────

def discover(src, robot, progress):
    """One pass over the TF and pose topics.

    Returns everything the rewrite and the checks need: the frame pair, its
    carrier topic, per-pair transform counts, per-robot pose statistics, and
    the yaw series of both the recorded odometry and the ground truth.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    all_topics = reader.get_all_topics_and_types()

    tf_topics = [t.name for t in all_topics
                 if t.type == TF_TYPE and t.name != '/tf_static']
    pose_topics = {}
    for t in all_topics:
        if t.type != POSE_TYPE:
            continue
        for n in range(NUM_ROBOTS):
            if t.name == f'/model/robot_{n}/pose':
                pose_topics[n] = t.name
    if not tf_topics:
        print('source bag carries no TFMessage topic other than /tf_static',
              file=sys.stderr)
        return None
    if robot not in pose_topics:
        print(f'source bag has no /model/robot_{robot}/pose', file=sys.stderr)
        return None

    want = sorted(tf_topics + list(pose_topics.values()))
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))

    pairs = Counter()                 # (topic, parent, child) -> transforms
    series = {}                       # candidate pair -> [(stamp_s, yaw)]
    pose_n = Counter()                # robot -> pose messages
    first_pose = {}                   # robot -> (x, y, yaw)
    max_disp = {}                     # robot -> max distance from first pose
    pose_frame = {}                   # robot -> header.frame_id
    truth_t, truth_se2 = [], []       # robot N only
    total = 0
    t0 = time.monotonic()

    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        total += 1

        if topic in tf_topics:
            msg = deserialize_message(data, TFMessage)
            for tr in msg.transforms:
                parent, child = tr.header.frame_id, tr.child_frame_id
                pairs[(topic, parent, child)] += 1
                # Collected here so C4 needs no second pass over /tf.
                if is_robot_odom_base(robot, parent, child):
                    series.setdefault((topic, parent, child), []).append(
                        (stamp_seconds(tr.header), yaw_of(tr.transform.rotation)))
        else:
            n = next(k for k, v in pose_topics.items() if v == topic)
            msg = deserialize_message(data, PoseStamped)
            se2 = pose_se2(msg)
            pose_n[n] += 1
            if n not in first_pose:
                first_pose[n] = se2
                max_disp[n] = 0.0
                pose_frame[n] = msg.header.frame_id
            d = math.hypot(se2[0] - first_pose[n][0], se2[1] - first_pose[n][1])
            if d > max_disp[n]:
                max_disp[n] = d
            if n == robot:
                truth_t.append(stamp_seconds(msg.header))
                truth_se2.append(se2)

        if progress and total % progress == 0:
            print(f'  {total:>9} read  {time.monotonic() - t0:6.0f} s', flush=True)

    print(f'\ndiscovery: {total} messages read in {time.monotonic() - t0:.0f} s')
    return {
        'pairs': pairs,
        'series': series,
        'tf_topics': tf_topics,
        'pose_topics': pose_topics,
        'pose_n': pose_n,
        'first_pose': first_pose,
        'max_disp': max_disp,
        'pose_frame': pose_frame,
        'truth_t': truth_t,
        'truth_se2': truth_se2,
    }


def select_pair(pairs, robot):
    """Find robot N's odom -> base_footprint pair without hard-coding names."""
    hits = sorted({(topic, p, c) for (topic, p, c) in pairs
                   if is_robot_odom_base(robot, p, c)})
    print(f'\nframe pairs carrying transforms (robot_{robot} candidates marked *):')
    for (topic, p, c), n in sorted(pairs.items()):
        mark = '*' if (topic, p, c) in hits else ' '
        print(f' {mark} {topic:<8} {p:<26} -> {c:<28} {n:>9}')
    if not hits:
        print(f'\nno odom -> base_footprint transform for robot_{robot}',
              file=sys.stderr)
        return None
    if len(hits) > 1:
        print(f'\nambiguous: {len(hits)} distinct pairs match robot_{robot}: {hits}',
              file=sys.stderr)
        return None
    return hits[0]


def check_c6(found, spawn, spawn_rev, allow_unverified) -> bool:
    """C6: corroborate --spawn-rev against robots that never moved.

    A parked robot's truth pose IS its spawn pose, so it pins the revision.
    The comparison itself lives in bag_overlap so that this and
    fit_world_transform.py cannot drift apart; only the policy on an
    inconclusive bag is decided here.
    """
    print(f'\nC6 spawn-rev corroboration (DOT_POSES @ {spawn_rev}):')
    ok, lines, parked = check_spawn_against_truth(
        found['first_pose'], found['max_disp'], spawn, spawn_rev)
    for line in lines:
        print(line)
    if not ok:
        print('  C6 FAILED', file=sys.stderr)
        return False
    if not parked:
        msg = (f'  C6 INCONCLUSIVE: no robot was parked, so {spawn_rev} is '
               f'uncorroborated by this bag.')
        if not allow_unverified:
            print(msg + ' Pass --allow-unverified-spawn to proceed anyway.',
                  file=sys.stderr)
            return False
        print(msg + ' Proceeding on --allow-unverified-spawn.')
        return True
    print('  C6 ok')
    return True


def check_c4(found, anchor) -> bool:
    """C4: the rewrite has to change something.

    Compares the synthesised yaw against the odometry it replaces. With the
    spawn anchor both are measured from the same origin, so this is the odom
    yaw error itself -- not an artefact of where the anchor was placed.
    """
    odom_t, odom_y = found['odom_t'], found['odom_y']
    inv_A = se2_inverse(anchor)

    diffs = []
    outside = 0
    for t, se2 in zip(found['truth_t'], found['truth_se2']):
        y_odom = interp_yaw(odom_t, odom_y, t)
        if y_odom is None:
            outside += 1
            continue
        diffs.append(abs(math.degrees(wrap(se2_compose(inv_A, se2)[2] - y_odom))))

    print('\nC4 the rewrite changed something:')
    if not diffs:
        print('  C4 FAILED: no synthesised stamp falls inside the recorded '
              'odometry range', file=sys.stderr)
        return False
    med, mx = statistics.median(diffs), max(diffs)
    print(f'  |dYaw| truth vs odometry over {len(diffs)} stamps '
          f'({outside} outside the odometry range):')
    print(f'    median {med:8.3f} deg')
    print(f'    max    {mx:8.3f} deg   (must exceed 1 deg)')
    if mx <= 1.0:
        print('  C4: odometry already agreed with truth on this bag -- the '
              'test cannot discriminate.', file=sys.stderr)
        return False
    print('  C4 ok')
    return True


# ──────────────────────────────── pass 2 ────────────────────────────────────

def rewrite(src, dst, anchor, pair, pose_topic, progress):
    """Strip the recorded pair, synthesise one transform per truth pose."""
    topic_tf, parent, child = pair
    inv_A = se2_inverse(anchor)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = reader.get_all_topics_and_types()

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for t in topics:
        writer.create_topic(t)

    removed = synth = emptied = total_in = total_out = 0
    c3_checked = c3_worst_xy = c3_worst_yaw = 0
    t0 = time.monotonic()

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        total_in += 1

        if topic == topic_tf:
            msg = deserialize_message(data, TFMessage)
            keep = []
            changed = False
            for tr in msg.transforms:
                if tr.header.frame_id == parent and tr.child_frame_id == child:
                    removed += 1
                    changed = True
                else:
                    keep.append(tr)
            if changed:
                if not keep:
                    emptied += 1
                    continue                     # nothing left to write
                msg.transforms = keep
                data = serialize_message(msg)

        writer.write(topic, data, stamp)
        total_out += 1

        if topic == pose_topic:
            pose = deserialize_message(data, PoseStamped)
            se2 = pose_se2(pose)
            ox, oy, oyaw = se2_compose(inv_A, se2)

            tr = TransformStamped()
            tr.header.stamp = pose.header.stamp
            tr.header.frame_id = parent
            tr.child_frame_id = child
            tr.transform.translation.x = ox
            tr.transform.translation.y = oy
            tr.transform.translation.z = 0.0
            tr.transform.rotation.x = 0.0
            tr.transform.rotation.y = 0.0
            tr.transform.rotation.z = math.sin(oyaw / 2.0)
            tr.transform.rotation.w = math.cos(oyaw / 2.0)

            writer.write(topic_tf, serialize_message(TFMessage(transforms=[tr])), stamp)
            synth += 1
            total_out += 1

            # C3: round trip through the quaternion we just built.
            if synth % 100 == 1:
                t = tr.transform.translation
                back = se2_compose(anchor, (t.x, t.y, yaw_of(tr.transform.rotation)))
                c3_worst_xy = max(c3_worst_xy, abs(back[0] - se2[0]),
                                  abs(back[1] - se2[1]))
                c3_worst_yaw = max(c3_worst_yaw, abs(wrap(back[2] - se2[2])))
                c3_checked += 1

        if progress and total_in % progress == 0:
            print(f'  {total_in:>9} read  {time.monotonic() - t0:6.0f} s', flush=True)

    del writer                                   # flush and close the bag
    print(f'\nrewrite: {total_in} messages read in {time.monotonic() - t0:.0f} s '
          f'-> {dst}')
    return {
        'removed': removed, 'synth': synth, 'emptied': emptied,
        'total_in': total_in, 'total_out': total_out,
        'c3_checked': c3_checked,
        'c3_worst_xy': c3_worst_xy, 'c3_worst_yaw': c3_worst_yaw,
    }


def count_pair(bag, pair):
    """Pass 3: count the pair in a written bag."""
    topic_tf, parent, child = pair
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_tf]))
    n = 0
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                n += 1
    return n


# ───────────────────────────────── driver ───────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in', dest='src', required=True,
                    help='source bag directory (mcap)')
    ap.add_argument('--out', dest='dst',
                    help='destination bag directory; must not exist. '
                         'Not required with --dry-run')
    ap.add_argument('--robot', type=int, default=0)
    ap.add_argument('--spawn-rev', default=BAG_ERA_REV,
                    help='git rev whose DOT_POSES gives the spawn anchor. '
                         'b18-era bags need 08617b2, not the default')
    ap.add_argument('--dry-run', action='store_true',
                    help='discovery, C1, C4 and C6 only; writes nothing')
    ap.add_argument('--allow-unverified-spawn', action='store_true',
                    help='proceed when no parked robot can corroborate --spawn-rev')
    ap.add_argument('--progress', type=int, default=250_000,
                    help='print progress every N messages read')
    args = ap.parse_args()

    # Redirected, stdout block-buffers while stderr does not, which lands every
    # failure line above the output that explains it. Line buffering keeps a
    # tee'd log in the order the checks actually ran.
    sys.stdout.reconfigure(line_buffering=True)

    if not args.dry_run and not args.dst:
        ap.error('--out is required unless --dry-run is given')
    if not 0 <= args.robot < NUM_ROBOTS:
        ap.error(f'--robot must be in 0..{NUM_ROBOTS - 1}')

    # C1 runs before anything touches a bag: if the composition is wrong,
    # nothing downstream is worth computing.
    if not check_c1():
        return 1

    if args.dst and os.path.exists(args.dst):
        print(f'refusing to overwrite existing bag: {args.dst}', file=sys.stderr)
        return 2

    spawn = resolve_spawn_poses(args.spawn_rev)

    found = discover(args.src, args.robot, args.progress)
    if found is None:
        return 3

    pair = select_pair(found['pairs'], args.robot)
    if pair is None:
        return 4
    topic_tf, parent, child = pair
    census = found['pairs'][pair]
    pose_topic = found['pose_topics'][args.robot]
    pose_count = found['pose_n'][args.robot]

    print(f'\nrewriting on {topic_tf}: {parent} -> {child}  ({census} transforms)')
    print(f'ground truth: {pose_topic}  ({pose_count} messages, '
          f"header.frame_id {found['pose_frame'][args.robot]!r})")

    # The odometry yaw series C4 compares against, gathered during discovery.
    odom = sorted(found['series'][pair])
    found['odom_t'] = [s for s, _ in odom]
    found['odom_y'] = [y for _, y in odom]

    sx, sy = spawn[args.robot]
    anchor = (sx, sy, 0.0)
    print(f'\nanchor A = spawn of robot_{args.robot} @ {args.spawn_rev}: '
          f'x={anchor[0]:+.4f} y={anchor[1]:+.4f} yaw={math.degrees(anchor[2]):+.4f} deg')

    ok = check_c6(found, spawn, args.spawn_rev, args.allow_unverified_spawn)
    if not ok:
        return 5
    if not check_c4(found, anchor):
        return 2 if found['truth_t'] else 6

    if args.dry_run:
        print('\n--dry-run: nothing written. Predicted counts:')
        print(f'  transforms to remove      {census:>9}')
        print(f'  transforms to synthesise  {pose_count:>9}')
        return 0

    res = rewrite(args.src, args.dst, anchor, pair, pose_topic, args.progress)

    print('\nC2 counts:')
    print(f'  transforms removed        {res["removed"]:>9}  (census {census})')
    print(f'  transforms synthesised    {res["synth"]:>9}  (poses {pose_count})')
    print(f'  messages emptied, skipped {res["emptied"]:>9}')
    print(f'  messages in               {res["total_in"]:>9}')
    print(f'  messages out              {res["total_out"]:>9}  '
          f'(expected {res["total_in"] - res["emptied"] + res["synth"]})')
    c2 = (res['removed'] == census
          and res['synth'] == pose_count
          and res['total_out'] == res['total_in'] - res['emptied'] + res['synth'])
    print('  C2 ok' if c2 else '  C2 FAILED')

    print('\nC3 round trip on the bag:')
    print(f'  {res["c3_checked"]} sampled transforms, worst error '
          f'{res["c3_worst_xy"]:.3e} m, {res["c3_worst_yaw"]:.3e} rad '
          f'(tolerance 1e-6)')
    c3 = res['c3_worst_xy'] <= 1e-6 and res['c3_worst_yaw'] <= 1e-6
    print('  C3 ok' if c3 else '  C3 FAILED')

    print('\nC5 no leftovers:')
    out_n = count_pair(args.dst, pair)
    print(f'  {parent} -> {child} in the output bag: {out_n} '
          f'(must equal {pose_count})')
    c5 = out_n == pose_count
    print('  C5 ok' if c5 else '  C5 FAILED')

    anchor_txt = Path(args.dst) / 'truth_anchor.txt'
    anchor_txt.write_text(
        f'# truth-pose replay anchor for robot_{args.robot}\n'
        f'# written by rewrite_odom_from_truth.py from {args.src}\n'
        f'# odom_T_base = inv(A) . P, A below, world frame '
        f"{found['pose_frame'][args.robot]!r}\n"
        f'# A is the spawn pose: DOT_POSES[{args.robot}] @ {args.spawn_rev}\n'
        f'x_m {anchor[0]:.6f}\n'
        f'y_m {anchor[1]:.6f}\n'
        f'yaw_deg {math.degrees(anchor[2]):.6f}\n')
    print(f'\nwrote {anchor_txt}')

    if not (c2 and c3 and c5):
        return 7
    print('\nall checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
