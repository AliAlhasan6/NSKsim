#!/usr/bin/env python3
"""Verify the robots are ACTUALLY MOVING before a timed run starts.

Run by hand between bring-up and the run; there is no harness. Subscribes to
/robot_<id>/odom for a set of robots, samples for a window of SIM time, and
makes TWO INDEPENDENT ASSERTIONS per robot:

  msgs>0     odometry is reaching ROS at all. Zero means a bridge or namespace
             problem — the robot may well be moving in Gazebo, but nothing is
             arriving, so no measurement of it is possible.
  path>=floor  the robot is being driven. Plenty of messages with no motion
             means it is muted (its id is in swarm_sim.launch.py's nav_robots,
             which mutes the wander driver) or wander:=false, or it has no
             controller. The plumbing is fine; nothing is driving.

They are reported in SEPARATE COLUMNS and never collapsed into one verdict:
they indicate different faults, and which one fired is the only thing that says
where to go looking. Both worthless runs look identical from the outside — every
node healthy, coverage numbers produced for robots that never left their spawn
pose. That is what invalidated rung2f through rung2i.

The motion assertion is made against CUMULATIVE PATH LENGTH, not net
displacement: a robot bouncing between two walls ends where it started, so its
displacement is ~0 while it has plainly been moving. Net displacement is
reported alongside for context and is never asserted on.

Usage:
  preflight_motion.py --min-path 0.5                      # robots 0-4, 20 s
  preflight_motion.py --min-path 0.5 --robots 0,3         # just those two
  preflight_motion.py --min-path 0.5 --window 60          # longer window
  preflight_motion.py --min-path 0.0                      # CALIBRATION pass

--min-path is REQUIRED and has no default; see its help text. The last form is
how you calibrate it: every robot passes by construction and the path_m column
is the measurement a real floor gets derived from.

Exit codes:
  0  every robot passed both assertions
  1  at least one assertion failed (the summary names each robot and which)
  2  operational failure — no /clock, or the window never completed. An
     incomplete window is an INVALID MEASUREMENT, not a fail verdict, and is
     deliberately not reported as one.
"""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


# ── Geometry ─────────────────────────────────────────────────────────────────
# Pure functions over [(x, y), ...] — no rclpy, no node, nothing to mock. This
# is the part with a right answer, so it is the part under unit test
# (test/test_preflight_motion.py in the nsk_swarm package).

def path_length(points):
    """Cumulative distance travelled: the sum of inter-sample steps.

    NOT net displacement, and the difference is the whole point. A robot
    bouncing between two walls ends where it started: displacement ~0, path
    length large. This is the number the motion assertion is made against.

    Empty and single-point inputs sum to 0.0 without special-casing.
    """
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def net_displacement(points):
    """Straight-line distance from the first sample to the last.

    Reported for context — a robot with a long path and ~0 displacement is
    circling or wall-bouncing, one with path ~= displacement drove off in a
    line — but never asserted on.
    """
    return math.dist(points[0], points[-1]) if len(points) >= 2 else 0.0


# ── Sampling ─────────────────────────────────────────────────────────────────

def parse_robots(text):
    """'0,3,4' -> [0, 3, 4], rejecting anything that is not a robot id.

    Deduplicates while keeping the given order: a repeated id would otherwise
    get its own table row against the same track, reading as two robots.
    """
    ids = []
    for field in text.split(','):
        field = field.strip()
        if not field:
            continue
        try:
            value = int(field)
        except ValueError:
            raise argparse.ArgumentTypeError(
                f'not a robot id: {field!r} (expected comma-separated ints)')
        if value < 0:
            raise argparse.ArgumentTypeError(f'negative robot id: {value}')
        if value not in ids:
            ids.append(value)
    if not ids:
        raise argparse.ArgumentTypeError('no robot ids given')
    return ids


def sim_seconds(node):
    """Sim time in seconds. Reads 0.0 until /clock first publishes."""
    return node.get_clock().now().nanoseconds * 1e-9


def wait_for_clock(node, wall_timeout):
    """Spin until the sim clock ticks past zero. True if it did.

    With use_sim_time the clock sits at exactly 0.0 until /clock publishes, so
    windowing before that point measures against a frozen clock and returns
    immediately (or never). swarm_sim.launch.py starts the /clock bridge with no
    TimerAction precisely because everything else is stuck at t=0 without it —
    if this wait times out, that bridge (or the whole sim) is the thing to check.
    """
    deadline = time.monotonic() + wall_timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if sim_seconds(node) > 0.0:
            return True
    return False


def sample(node, tracks, window, wall_timeout):
    """Spin for `window` SIM seconds, filling `tracks`. True if it completed.

    Sim time, not wall clock: under a degraded RTF a 20 s wall window samples
    some much shorter slice of simulated motion, which is the wrong interval and
    silently understates every path length.

    `wall_timeout` is a hang guard only — a sim that stalls mid-window would
    otherwise spin here forever. Tripping it invalidates the measurement rather
    than failing the robots, so the caller exits 2 and reports no verdicts.
    """
    start = sim_seconds(node)
    deadline = time.monotonic() + wall_timeout
    while rclpy.ok() and sim_seconds(node) - start < window:
        if time.monotonic() >= deadline:
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return True


# ── Report ───────────────────────────────────────────────────────────────────

def report(results, floor):
    """Print the per-robot table; return the failures as (id, [assertion]).

    A robot with zero messages reports the floor check as 'n/a', not FAIL: with
    no samples the floor was never measured, and calling it a motion fault would
    name a fault nobody observed. It still fails, on msgs>0.
    """
    print(f'{"robot":>5} {"msgs":>7} {"path_m":>9} {"net_m":>9} '
          f'{"msgs>0":>8} {"path>=" + format(floor, ".2f"):>12}')

    failures = []
    for robot_id, points in results:
        travelled = path_length(points)
        displaced = net_displacement(points)

        got_messages = len(points) > 0
        # Evaluated only when there are samples to evaluate it on.
        moved = travelled >= floor if got_messages else None

        failed = []
        if not got_messages:
            failed.append('msgs>0')
        if moved is False:
            failed.append('path>=floor')
        if failed:
            failures.append((robot_id, failed))

        print(f'{robot_id:>5} {len(points):>7} {travelled:>9.3f} '
              f'{displaced:>9.3f} {"PASS" if got_messages else "FAIL":>8} '
              f'{("n/a" if moved is None else "PASS" if moved else "FAIL"):>12}')
    return failures


def main():
    ap = argparse.ArgumentParser(
        description='Verify robots are moving before a timed run starts.')
    ap.add_argument(
        '--robots', type=parse_robots, default='0,1,2,3,4',
        help='Comma-separated robot ids to check (default: 0,1,2,3,4 — the '
             'full swarm, NUM_ROBOTS=5).')
    ap.add_argument(
        '--window', type=float, default=20.0,
        help='Sampling window in SIM seconds (default: 20.0). Sim time, not '
             'wall clock: under a degraded RTF a wall-clock window samples a '
             'much shorter slice of simulated motion.')
    ap.add_argument(
        '--min-path', type=float, required=True,
        help='REQUIRED, no default. Cumulative path-length floor in metres, '
             'below which a robot counts as not moving. Deliberately has no '
             'default: it must be CALIBRATED against a real wandering robot '
             'before it can gate anything, and a default would harden a number '
             'nobody measured into the pass criterion. Run once with '
             '--min-path 0 and read the path_m column to get one.')
    ap.add_argument(
        '--wall-timeout', type=float, default=300.0,
        help='Wall-clock hang guard in seconds (default: 300.0). Caps the wait '
             'for /clock and the sampling window so a stalled sim cannot spin '
             'here forever. Tripping it exits 2 (measurement invalid), never 1.')
    args = ap.parse_args()

    # argparse applies `type` to a string default too, so this is a list of
    # ints whether or not --robots was passed.
    robots = args.robots

    rclpy.init()
    # use_sim_time from birth: the window is measured in sim seconds, so the
    # node's clock has to be the sim's from the first sample onward.
    node = rclpy.create_node(
        'nsk_preflight_motion',
        parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    tracks = {robot_id: [] for robot_id in robots}

    def record(msg: Odometry, robot_id: int):
        p = msg.pose.pose.position
        tracks[robot_id].append((p.x, p.y))

    for robot_id in robots:
        # qos_profile_sensor_data (BEST_EFFORT), matching robot_node.ODOM_QOS.
        # The ros_gz bridge publishes odom RELIABLE, and a reliable publisher
        # feeds a best-effort subscriber fine (only the reverse fails to
        # connect). Getting this backwards would report EVERY robot as
        # zero-message — this script would manufacture the exact fault it
        # exists to detect.
        node.create_subscription(
            Odometry, f'/robot_{robot_id}/odom',
            lambda msg, rid=robot_id: record(msg, rid),
            qos_profile_sensor_data)

    def shutdown():
        node.destroy_node()
        rclpy.shutdown()

    if not wait_for_clock(node, args.wall_timeout):
        print(f'ERROR: no /clock within {args.wall_timeout:g}s wall — the sim '
              f'is not running, or its clock bridge is missing. Nothing was '
              f'measured.', file=sys.stderr)
        shutdown()
        sys.exit(2)

    print(f'sampling {args.window:g} s (sim) on '
          f'{", ".join(f"/robot_{r}/odom" for r in robots)}')
    completed = sample(node, tracks, args.window, args.wall_timeout)

    # Snapshot before shutdown; the callbacks stop firing after this.
    results = [(robot_id, list(tracks[robot_id])) for robot_id in robots]
    shutdown()

    if not completed:
        print(f'ERROR: the {args.window:g} s (sim) window did not complete '
              f'within {args.wall_timeout:g}s wall — the sim stalled. The '
              f'samples taken are an incomplete interval, so no verdict is '
              f'reported.', file=sys.stderr)
        sys.exit(2)

    print(f'\nwindow {args.window:g} s (sim), floor {args.min_path:.2f} m\n')
    failures = report(results, args.min_path)

    if failures:
        named = ', '.join(f'robot {robot_id} ({", ".join(which)})'
                          for robot_id, which in failures)
        print(f'\nFAILED: {named}', file=sys.stderr)
        sys.exit(1)

    print(f'\nPASSED: {len(results)} robot(s) moving')


if __name__ == '__main__':
    main()
