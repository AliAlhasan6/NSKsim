"""Layer 1 — lidar obstacle steering in NSKRobotNode._update_motion_state.

Same choice as test_stuck_escape.py (which supplies make_robot_stub): the
real unbound _forward_arc_bins and _update_motion_state are bound to a plain
stub, so these exercise the production math with no rclpy init and no
wait_for_service block.

Why this layer exists at all: the near_wall guard tests pose against the
world boundary only, so it cannot see the interior walls at y=+/-3.0 and
x=+/-4.0, and the stuck detector cannot cover them either — a robot sliding
along a wall displaces well over stuck_epsilon_m per window, so _check_stuck
correctly returns False. Measured: robots pinned against interior walls
accumulated 2.1-2.7 m of path over 20 s and produced zero 'stuck at'
triggers in a 227 s run. Detection has to come from the scan.
"""

import math

import pytest
from sensor_msgs.msg import LaserScan

from nsk_swarm.robot_node import EXPLORE, FLOCK

from test_stuck_escape import make_robot_stub

# Stock TurtleBot3 burger hls_lfcd_lds, the lidar this sim actually runs
# (turtlebot3_gazebo/models/turtlebot3_burger/model.sdf): 360 samples,
# min_angle 0.0, max_angle 6.28, range 0.12 .. 3.5.
BURGER_SAMPLES    = 360
BURGER_ANGLE_MIN  = 0.0
BURGER_RANGE_MIN  = 0.12
BURGER_RANGE_MAX  = 3.5

OPEN = 3.0    # a clear return, comfortably past the 0.6 m stop distance
WALL = 0.3    # a wall well inside it


def wrap(angle: float) -> float:
    """Wrap to (-pi, pi], matching the production bearing convention."""
    return math.atan2(math.sin(angle), math.cos(angle))


def make_scan(range_at, samples=BURGER_SAMPLES, angle_min=BURGER_ANGLE_MIN,
              angle_increment=None, range_min=BURGER_RANGE_MIN,
              range_max=BURGER_RANGE_MAX):
    """A LaserScan whose i-th sample is range_at(bearing), bearing wrapped
    to (-pi, pi] so fixtures can be written in robot-relative terms.

    The defaults are the deployed burger geometry, sweeping 0..2*pi rather
    than -pi..pi. That is deliberate and load-bearing: the forward arc is
    then SPLIT across the two ends of the ranges array (indices 270-359 and
    0-89), so an implementation that sliced a contiguous index range would
    read only the robot's right-hand side. Every test here runs against that
    layout by default; test_arc_found_with_symmetric_angle_convention
    re-runs one against -pi..pi to show the code is convention-independent.
    """
    if angle_increment is None:
        angle_increment = 2.0 * math.pi / samples
    scan = LaserScan()
    scan.angle_min       = angle_min
    scan.angle_max       = angle_min + (samples - 1) * angle_increment
    scan.angle_increment = angle_increment
    scan.range_min       = range_min
    scan.range_max       = range_max
    scan.ranges = [float(range_at(wrap(angle_min + i * angle_increment)))
                   for i in range(samples)]
    return scan


def sectors(*spans, default=OPEN):
    """range_at built from (lo_deg, hi_deg, range) spans, degrees relative to
    straight ahead; bearings outside every span get `default`."""
    def range_at(bearing):
        deg = math.degrees(bearing)
        for lo, hi, value in spans:
            if lo <= deg <= hi:
                return value
        return default
    return range_at


def steer_stub(scan=None, **kwargs):
    """A stub with the scan installed. walk_turn_max defaults high so
    `diff * 2.0` never clamps and the *chosen bearing* stays recoverable as
    _current_angular_z / 2 — the difference between asserting 'it turned
    right' and 'it turned to this bearing, which is in the open sector'."""
    kwargs.setdefault('walk_turn_max', 10.0)
    stub = make_robot_stub(latest_scan=scan, **kwargs)
    return stub


def chosen_bearing_deg(stub) -> float:
    """The bearing the scan branch steered toward, in degrees relative to
    straight ahead. Valid only while walk_turn_max is high enough that the
    clamp is inactive (see steer_stub)."""
    return math.degrees(stub._current_angular_z / 2.0)


# ── a. wall dead ahead, and the corner case ──────────────────────────────────

def test_wall_dead_ahead_steers_away_and_not_into_the_second_wall():
    # Corner: wall dead ahead (+/-40 deg) AND a second wall to the left
    # (40..90 deg). The only way out is right. Steering merely 'away from
    # ahead' is not enough — turning left escapes the first wall straight
    # into the second.
    scan = make_scan(sectors((-40, 40, WALL), (40, 90, 0.5)))
    stub = steer_stub(scan, pos_x=0.0, pos_y=0.0, yaw=0.0)

    stub._update_motion_state()

    bearing = chosen_bearing_deg(stub)
    assert stub._current_angular_z < 0.0        # turned right
    # Landed in the open right-hand sector, not into either wall.
    assert -90.0 <= bearing <= -40.0
    # And the bearing it picked really is clear.
    bins = dict(stub._forward_arc_bins())
    picked = min(bins, key=lambda b: abs(math.degrees(b) - bearing))
    assert bins[picked] == pytest.approx(OPEN)


def test_wall_dead_ahead_with_symmetric_opening_still_turns():
    # Sanity floor for the case above: a wall straight ahead with clear air
    # on both flanks must still produce a turn, whichever way it breaks.
    scan = make_scan(sectors((-30, 30, WALL)))
    stub = steer_stub(scan, yaw=0.0)

    stub._update_motion_state()

    assert stub._current_angular_z != 0.0
    assert abs(chosen_bearing_deg(stub)) > 30.0   # off the blocked sector


# ── b. obstacles outside the forward arc are ignored ─────────────────────────

def test_obstacle_behind_is_ignored():
    # A wall squarely behind (|bearing| > 120 deg) is not in the way. The
    # forward arc is clear, so the scan branch must not fire at all — proven
    # by leaving FLOCK steering intact rather than preempting it.
    scan = make_scan(sectors((120, 180, WALL), (-180, -120, WALL)))
    stub = steer_stub(scan, pos_x=0.0, pos_y=0.0, yaw=0.0, state=FLOCK)
    stub.peer_positions = {1: (0.0, 2.0)}     # peer due north, in range

    stub._update_motion_state()

    # Every forward bin is clear.
    assert min(r for _, r in stub._forward_arc_bins()) == pytest.approx(OPEN)
    # FLOCK steering ran: turning toward the peer (north, positive), which
    # the scan branch would have overridden had it fired.
    assert stub._state == FLOCK
    assert stub._current_angular_z > 0.0


# ── c/h/i. invalid returns ───────────────────────────────────────────────────

def test_all_inf_scan_is_not_blocked():
    scan = make_scan(lambda b: math.inf)
    stub = steer_stub(scan, state=EXPLORE)
    assert stub._forward_arc_bins() == []


def test_zero_no_return_samples_are_not_treated_as_contact():
    # 0.0 is the common 'no return' encoding and sits below range_min.
    # Taken at face value it reads as an obstacle permanently touching the
    # robot, which would pin the scan branch on forever.
    scan = make_scan(lambda b: 0.0)
    stub = steer_stub(scan)
    assert stub._forward_arc_bins() == []


def test_returns_beyond_range_max_are_dropped():
    scan = make_scan(lambda b: BURGER_RANGE_MAX + 1.0)
    stub = steer_stub(scan)
    assert stub._forward_arc_bins() == []


def test_non_finite_samples_dropped_even_when_range_max_is_infinite():
    # Some LaserScan producers advertise range_max=inf, and then the
    # range-bounds test alone lets inf and nan straight through — only the
    # isfinite guard rejects them. Without it an inf return would read as
    # 'valid at infinite distance' and, worse, nan would enter the bins.
    scan = make_scan(lambda b: math.inf, range_max=math.inf)
    stub = steer_stub(scan)
    assert stub._forward_arc_bins() == []

    nan_scan = make_scan(lambda b: math.nan, range_max=math.inf)
    nan_stub = steer_stub(nan_scan)
    assert nan_stub._forward_arc_bins() == []


def test_valid_samples_survive_alongside_invalid_ones_in_the_same_bin():
    # A bin holding one real return and a spray of junk keeps the real one.
    def range_at(bearing):
        deg = math.degrees(bearing)
        if -90 <= deg <= 90:
            return WALL if -2.0 <= deg <= 2.0 else math.inf
        return OPEN
    stub = steer_stub(make_scan(range_at))
    bins = stub._forward_arc_bins()
    assert len(bins) == 1                       # only the centre bin survived
    assert bins[0][1] == pytest.approx(WALL)


# ── d. side blockage steers the other way ────────────────────────────────────

def test_blocked_on_left_steers_right():
    # The wall wraps from the left flank across straight ahead, so the only
    # open bearings are on the right. It has to: with a wall on the left
    # flank alone, holding course is a perfectly good answer, and asserting
    # a right turn would be asserting more than the code should promise.
    scan = make_scan(sectors((-10, 90, WALL)))
    stub = steer_stub(scan, yaw=0.0)
    stub._update_motion_state()
    assert stub._current_angular_z < 0.0
    assert chosen_bearing_deg(stub) < 0.0


def test_blocked_on_right_steers_left():
    scan = make_scan(sectors((-90, 10, WALL)))
    stub = steer_stub(scan, yaw=0.0)
    stub._update_motion_state()
    assert stub._current_angular_z > 0.0
    assert chosen_bearing_deg(stub) > 0.0


def test_side_blockage_verdicts_are_stable_across_the_tie_break():
    # Both scenes above pick at random among several open bins. The bins
    # differ, the SIDE must not — re-run enough to catch a scene that left
    # an opposite-sign or straight-ahead bin in the tied set.
    left_blocked  = steer_stub(make_scan(sectors((-10, 90, WALL))), yaw=0.0)
    right_blocked = steer_stub(make_scan(sectors((-90, 10, WALL))), yaw=0.0)
    for _ in range(60):
        left_blocked._update_motion_state()
        assert left_blocked._current_angular_z < 0.0
        right_blocked._update_motion_state()
        assert right_blocked._current_angular_z > 0.0


# ── tie-breaking: no systematic left/right bias ──────────────────────────────

def test_symmetric_opening_does_not_always_break_the_same_way():
    # max() returns the FIRST maximal element and _forward_arc_bins emits
    # right-to-left, so a deterministic pick sends every robot right out of
    # every symmetric opening. Every robot runs this same rule on the same
    # geometry, so that is a correlated fleet-wide drift, not a per-robot
    # quirk — it would bias the Levy walk the node exists to perform.
    scan = make_scan(sectors((-35, 35, WALL)))
    stub = steer_stub(scan, yaw=0.0)

    lefts = 0
    for _ in range(60):
        stub._update_motion_state()
        assert stub._current_angular_z != 0.0
        if stub._current_angular_z > 0.0:
            lefts += 1

    # Both outcomes occur, and neither is a rare accident: the scene is
    # symmetric, so this is Binom(60, 0.5) and the bounds sit past 5 sigma.
    assert 10 <= lefts <= 50


def test_tie_tolerance_absorbs_sensor_noise():
    # Real returns never land on identical floats — the burger lidar carries
    # gaussian noise, stddev 0.01 m in the SDF. With an exact-equality tie
    # test the draw collapses back to a deterministic winner, because one
    # side is always a few millimetres 'wider' than the other. Here the left
    # reads 0.02 m further open: inside _OBSTACLE_OPEN_TIE_M, so both sides
    # must still be reachable.
    def range_at(bearing):
        deg = math.degrees(bearing)
        if -35 <= deg <= 35:
            return WALL
        return OPEN + (0.02 if deg > 0 else 0.0)

    stub = steer_stub(make_scan(range_at), yaw=0.0)
    signs = set()
    for _ in range(60):
        stub._update_motion_state()
        signs.add(math.copysign(1.0, stub._current_angular_z))
    assert signs == {-1.0, 1.0}


def test_a_clearly_wider_opening_still_wins_every_time():
    # The tie-break must not turn a real preference into a coin flip. The
    # left gap is 2.0 m wider than the right — far outside the tolerance —
    # so it wins on every single call.
    scan = make_scan(sectors((-35, 35, WALL), (35, 90, OPEN), default=1.0))
    stub = steer_stub(scan, yaw=0.0)
    for _ in range(60):
        stub._update_motion_state()
        assert stub._current_angular_z > 0.0
        assert chosen_bearing_deg(stub) > 35.0


# ── e/f. the pose guard is an independent OR term ────────────────────────────

def test_pose_guard_still_fires_when_no_scan_has_ever_arrived():
    # A silently dead /scan must not disable the boundary guard.
    half = 20.0 / 2.0 - 0.5
    stub = make_robot_stub(pos_x=half + 0.5, pos_y=0.0, yaw=0.0,
                           walk_turn_max=1.0, world_size=20.0)
    assert stub._latest_scan is None

    stub._update_motion_state()

    # Centre is behind the robot (past the +x wall, facing +x): the guard
    # commands a turn rather than being bypassed.
    assert abs(stub._current_angular_z) == pytest.approx(stub.walk_turn_max)


def test_pose_guard_still_fires_when_the_scan_is_present_and_clear():
    # The mutation this pins: consulting the boundary test only when the
    # scan is missing or blocked. A live, clear scan must not shadow it.
    half = 20.0 / 2.0 - 0.5
    scan = make_scan(lambda b: OPEN)
    stub = make_robot_stub(pos_x=half + 0.5, pos_y=0.0, yaw=0.0,
                           walk_turn_max=1.0, world_size=20.0,
                           latest_scan=scan)

    stub._update_motion_state()

    assert abs(stub._current_angular_z) == pytest.approx(stub.walk_turn_max)


# ── g. the scan term fires well inside the boundary ──────────────────────────

def test_scan_fires_at_an_interior_wall_far_from_the_boundary():
    # The whole point: pinned against the interior wall at y=3.0, nowhere
    # near the |x|,|y| > 9.5 boundary band, so near_wall is False.
    scan = make_scan(sectors((-45, 45, WALL)))
    stub = steer_stub(scan, pos_x=0.0, pos_y=2.8, yaw=math.pi / 2)
    half = stub.world_size / 2.0 - 0.5
    assert not (abs(stub.pos_x) > half or abs(stub.pos_y) > half)

    stub._update_motion_state()

    assert stub._current_angular_z != 0.0


# ── j. angle convention independence ─────────────────────────────────────────

def test_arc_found_with_symmetric_angle_convention():
    # Same corner scene as test (a), published -pi..pi instead of 0..2*pi.
    # The verdict must not depend on the producer's convention.
    scene = sectors((-40, 40, WALL), (40, 90, 0.5))
    stub = steer_stub(make_scan(scene, angle_min=-math.pi), yaw=0.0)

    stub._update_motion_state()

    assert stub._current_angular_z < 0.0
    assert -90.0 <= chosen_bearing_deg(stub) <= -40.0


# ── k. the disable flag ──────────────────────────────────────────────────────

def test_obstacle_enabled_false_reproduces_prior_behaviour():
    # Wall dead ahead, well inside the boundary: with the flag off nothing
    # steers, exactly as before the scan existed.
    scan = make_scan(sectors((-45, 45, WALL)))
    stub = steer_stub(scan, pos_x=0.0, pos_y=0.0, yaw=0.0,
                      obstacle_enabled=False, state=FLOCK)
    stub.peer_positions = {1: (0.0, 2.0)}     # peer due north, in range

    stub._update_motion_state()

    # FLOCK steering ran untouched — the scan branch never fired.
    assert stub._state == FLOCK
    assert stub._current_angular_z > 0.0


def test_obstacle_stop_m_is_the_threshold():
    # A return just outside obstacle_stop_m does not block; just inside does.
    stop = 0.6
    clear = steer_stub(make_scan(sectors((-45, 45, stop + 0.05))),
                       obstacle_stop_m=stop, state=FLOCK)
    clear.peer_positions = {1: (0.0, 2.0)}
    clear._update_motion_state()
    assert clear._current_angular_z > 0.0      # FLOCK steering, not the scan

    blocked = steer_stub(make_scan(sectors((-45, 45, stop - 0.05))),
                         obstacle_stop_m=stop, state=FLOCK)
    blocked.peer_positions = {1: (0.0, 2.0)}
    blocked._update_motion_state()
    assert abs(chosen_bearing_deg(blocked)) > 45.0   # steered off the wall
