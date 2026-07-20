"""Layer 1 — stuck detector + recovery state machine in NSKRobotNode.

Choice (per test_geometry.py): the real unbound _record_motion_sample,
_check_stuck, _enter_recovery, _advance_recovery and _exit_recovery are
bound to a plain stub exposing only the attributes they touch. This
exercises the exact production math with no rclpy init and no
wait_for_service block (NSKRobotNode.__init__ blocks on the engine
services).
"""

import math
import time
from collections import deque
from types import SimpleNamespace
from unittest import mock

import pytest

from nsk_swarm.robot_node import (
    EXPLORE, FLOCK, NSKRobotNode, RECOVER_REVERSE, RECOVER_ROTATE,
    _RECOVERY_ROTATE_MAX_RAD, _RECOVERY_ROTATE_MIN_RAD,
    _RECOVERY_ROTATE_ESCALATE_MAX_RAD, _RECOVERY_ROTATE_ESCALATE_MIN_RAD,
    _RECOVERY_ROTATE_ESCALATE_EXCLUDE_HI_RAD,
    _RECOVERY_ROTATE_ESCALATE_EXCLUDE_LO_RAD,
)


def make_robot_stub(pos_x=0.0, pos_y=0.0, yaw=0.0, robot_id=0,
                    walk_speed=0.15, walk_turn_max=0.5,
                    stuck_window_sec=4.0, stuck_epsilon_m=0.05,
                    escape_reverse_m=0.9, escape_suppress_sec=4.0,
                    escape_repeat_radius_m=1.0, escape_repeat_window_sec=60.0,
                    num_robots=2, comm_range=3.0, world_size=20.0,
                    state=EXPLORE):
    stub = SimpleNamespace(
        robot_id=robot_id, pos_x=pos_x, pos_y=pos_y, yaw=yaw,
        walk_speed=walk_speed, walk_turn_max=walk_turn_max,
        stuck_window_sec=stuck_window_sec, stuck_epsilon_m=stuck_epsilon_m,
        escape_reverse_m=escape_reverse_m,
        escape_suppress_sec=escape_suppress_sec,
        escape_repeat_radius_m=escape_repeat_radius_m,
        escape_repeat_window_sec=escape_repeat_window_sec,
        num_robots=num_robots, comm_range=comm_range, world_size=world_size,
        _state=state,
        peer_positions={},
        _current_angular_z=0.0,
        _levy_steps_left=0,
        _levy_heading=0.0,
        _disperse_until=0.0,
        _motion_samples=deque(),
        _recovery_phase=None,
        _recovery_phase_start_time=0.0,
        _recovery_phase_start_pos=(0.0, 0.0),
        _recovery_target_yaw=0.0,
        _escape_reverse_target_m=escape_reverse_m,
        _escape_suppress_sec_current=escape_suppress_sec,
        _escape_escalated=False,
        _escape_suppress_until=0.0,
        _escape_heading=0.0,
        _last_escape_pos=None,
        _last_escape_time=0.0,
        _reverse_time_cap_sec=3.0 * (escape_reverse_m / walk_speed),
        _rotate_time_cap_sec=3.0 * (_RECOVERY_ROTATE_MAX_RAD / walk_turn_max),
        logger=mock.MagicMock())
    stub.get_logger = lambda: stub.logger
    for name in ('_record_motion_sample', '_check_stuck', '_enter_recovery',
                 '_advance_recovery', '_exit_recovery',
                 '_update_motion_state', '_peers_in_range', '_in_range'):
        setattr(stub, name, getattr(NSKRobotNode, name).__get__(stub))
    return stub


# ── _check_stuck ─────────────────────────────────────────────────────────────

def test_check_stuck_triggers_when_driven_but_not_moving():
    # Commanded speed nonzero throughout a full 4 s window; position barely
    # changes (wall-pinned) — this is exactly the detector's trigger case.
    stub = make_robot_stub(pos_x=1.0, pos_y=1.0)
    now = time.time()
    stub._motion_samples = deque([
        (now - 4.1, 1.00, 1.00, 0.15),
        (now - 3.0, 1.00, 1.01, 0.15),
        (now - 2.0, 1.01, 1.01, 0.15),
        (now - 1.0, 1.01, 1.00, 0.15),
        (now - 0.1, 1.00, 1.00, 0.15),
    ])
    assert stub._check_stuck() is True


def test_check_stuck_false_during_normal_motion():
    # Same nonzero-speed-throughout window, but the robot actually covered
    # ground — must not be flagged as stuck.
    stub = make_robot_stub(pos_x=1.5, pos_y=1.0)
    now = time.time()
    stub._motion_samples = deque([
        (now - 4.1, 1.00, 1.00, 0.15),
        (now - 3.0, 1.15, 1.00, 0.15),
        (now - 2.0, 1.30, 1.00, 0.15),
        (now - 1.0, 1.40, 1.00, 0.15),
        (now - 0.1, 1.50, 1.00, 0.15),
    ])
    assert stub._check_stuck() is False


def test_check_stuck_false_when_speed_was_zero_partway_through_window():
    # Motion was intentionally paused somewhere inside the window (not
    # driven for the *whole* window) — must not be mistaken for pinning.
    stub = make_robot_stub(pos_x=1.0, pos_y=1.0)
    now = time.time()
    stub._motion_samples = deque([
        (now - 4.1, 1.00, 1.00, 0.15),
        (now - 3.0, 1.00, 1.00, 0.0),
        (now - 2.0, 1.00, 1.00, 0.15),
        (now - 1.0, 1.00, 1.00, 0.15),
        (now - 0.1, 1.00, 1.00, 0.15),
    ])
    assert stub._check_stuck() is False


def test_check_stuck_false_before_window_fully_spanned():
    # Only ~1 s of history so far (robot just started, or just re-armed
    # after a previous recovery) — too early to judge.
    stub = make_robot_stub(pos_x=1.0, pos_y=1.0)
    now = time.time()
    stub._motion_samples = deque([
        (now - 1.0, 1.00, 1.00, 0.15),
        (now - 0.5, 1.00, 1.00, 0.15),
    ])
    assert stub._check_stuck() is False


def test_check_stuck_false_with_fewer_than_two_samples():
    stub = make_robot_stub(pos_x=1.0, pos_y=1.0)
    stub._motion_samples = deque([(time.time(), 1.0, 1.0, 0.15)])
    assert stub._check_stuck() is False


# ── _record_motion_sample ─────────────────────────────────────────────────────

def test_record_motion_sample_prunes_entries_outside_the_window():
    stub = make_robot_stub(pos_x=0.0, pos_y=0.0, stuck_window_sec=1.0)
    now = time.time()
    stub._motion_samples = deque([(now - 5.0, 0.0, 0.0, 0.15)])

    stub._record_motion_sample(0.15)

    assert len(stub._motion_samples) == 1   # the stale sample was dropped
    (t, x, y, speed), = stub._motion_samples
    assert speed == pytest.approx(0.15)


# ── _enter_recovery ────────────────────────────────────────────────────────────

def test_enter_recovery_logs_once_and_arms_reverse_phase():
    stub = make_robot_stub(pos_x=2.0, pos_y=-1.0, robot_id=3)
    stub._motion_samples = deque([(time.time(), 2.0, -1.0, 0.15)] * 3)

    stub._enter_recovery()

    stub.logger.info.assert_called_once()
    (msg,), _ = stub.logger.info.call_args
    assert msg == '[Robot 3] stuck at (2.00, -1.00) — escaping'
    assert stub._recovery_phase == RECOVER_REVERSE
    assert stub._recovery_phase_start_pos == (2.0, -1.0)
    assert len(stub._motion_samples) == 0   # cleared, ready to re-arm later


# ── _advance_recovery: full reverse -> rotate -> exit sequence ──────────────

def test_recovery_sequence_reverses_then_rotates_and_rearms(monkeypatch):
    # Fix the escape angle so the sequence is fully deterministic instead
    # of depending on an unmocked random draw.
    fixed_turn = 2.0   # radians, inside [90 deg, 270 deg] = [1.571, 4.712]
    calls = []

    def fake_uniform(a, b):
        calls.append((a, b))
        return fixed_turn

    monkeypatch.setattr('nsk_swarm.robot_node.random.uniform', fake_uniform)

    stub = make_robot_stub(pos_x=0.0, pos_y=0.0, yaw=0.0,
                           walk_speed=0.2, walk_turn_max=1.0)
    stub._motion_samples = deque([(time.time(), 0.0, 0.0, 0.2)] * 5)
    stub._enter_recovery()
    assert stub._recovery_phase == RECOVER_REVERSE

    dt = 0.01   # small relative to the rotate tolerance (3 deg), so the
                # sequence converges by angle rather than needing the
                # (real-wall-clock) time-cap fallback to end the test
    phases_seen = []
    for _ in range(3000):
        cmd = stub._advance_recovery()
        # Classify by the phase *after* the call: on the tick where reverse
        # completes, _advance_recovery transitions and falls through to
        # compute this same tick's rotate command in one call — so the
        # returned cmd already reflects whichever leg is now active.
        phase_after = stub._recovery_phase
        if phase_after == RECOVER_REVERSE:
            phases_seen.append('reverse')
            assert cmd.linear.x == pytest.approx(-stub.walk_speed)
            assert cmd.angular.z == 0.0
        elif phase_after == RECOVER_ROTATE:
            phases_seen.append('rotate')
            assert cmd.linear.x == 0.0
            assert abs(cmd.angular.z) == pytest.approx(stub.walk_turn_max)
        # Integrate one simulated tick from the commanded twist (a stand-in
        # for odom feedback in the real robot).
        stub.pos_x += cmd.linear.x * math.cos(stub.yaw) * dt
        stub.pos_y += cmd.linear.x * math.sin(stub.yaw) * dt
        stub.yaw   += cmd.angular.z * dt
        if phase_after is None:
            break
    else:
        pytest.fail('recovery did not complete within the simulated budget')

    # Reverse commands were issued strictly before any rotate commands, and
    # both legs actually ran (recovery didn't skip straight to exit).
    assert 'reverse' in phases_seen and 'rotate' in phases_seen
    assert phases_seen.index('reverse') < phases_seen.index('rotate')
    # The escape angle was drawn from the documented [90, 270] deg range.
    assert calls == [(_RECOVERY_ROTATE_MIN_RAD, _RECOVERY_ROTATE_MAX_RAD)]
    # Recovery re-arms cleanly: phase cleared, normal state machine
    # resumed, and the detector's window reset (no stale samples leak
    # through to the next check).
    assert stub._recovery_phase is None
    assert stub._state == EXPLORE
    assert len(stub._motion_samples) == 0


def test_advance_recovery_falls_back_to_time_cap_when_target_unreachable():
    # Position never moves at all (still pinned even while "reversing"):
    # the reverse leg must still end, via the time-cap fallback, instead of
    # commanding reverse forever.
    stub = make_robot_stub(pos_x=5.0, pos_y=5.0, walk_speed=0.2)
    stub._enter_recovery()
    assert stub._recovery_phase == RECOVER_REVERSE
    # _enter_recovery sizes the reverse cap to the target; override it to a
    # tiny value afterward so the fallback fires fast in this test.
    stub._reverse_time_cap_sec = 0.05

    time.sleep(0.06)   # exceed the cap without moving pos_x/pos_y at all
    stub._advance_recovery()

    # The time cap fired and advanced the leg — not stuck commanding
    # reverse forever. (Rotate can't complete in this same tick: the
    # minimum escape angle is 90 deg, far outside the rotate tolerance.)
    assert stub._recovery_phase == RECOVER_ROTATE


# ── (a) reverse uses the new default distance ────────────────────────────────

def test_reverse_uses_new_default_distance(monkeypatch):
    # The reverse leg must drive until ~escape_reverse_m (new default 0.9 m),
    # not the old fixed 0.3 m. Fix the escape angle so the draw is
    # deterministic; drive the reverse leg and record the distance covered at
    # the instant the phase flips off reverse.
    monkeypatch.setattr('nsk_swarm.robot_node.random.uniform',
                        lambda a, b: 2.0)
    stub = make_robot_stub(pos_x=0.0, pos_y=0.0, yaw=0.0,
                           walk_speed=0.2, walk_turn_max=1.0)
    stub._enter_recovery()
    assert stub._escape_reverse_target_m == pytest.approx(0.9)

    dt = 0.01
    dist_at_transition = None
    for _ in range(5000):
        was_reverse = stub._recovery_phase == RECOVER_REVERSE
        cmd = stub._advance_recovery()
        if was_reverse and stub._recovery_phase != RECOVER_REVERSE:
            # Reverse just completed on this tick — measure how far it went.
            dist_at_transition = math.hypot(stub.pos_x, stub.pos_y)
            break
        stub.pos_x += cmd.linear.x * math.cos(stub.yaw) * dt
        stub.pos_y += cmd.linear.x * math.sin(stub.yaw) * dt
        stub.yaw   += cmd.angular.z * dt
    else:
        pytest.fail('reverse leg never completed')

    # Reversed the new ~0.9 m (within a per-tick step), and unmistakably past
    # the old 0.3 m target.
    assert dist_at_transition == pytest.approx(0.9, abs=0.02)
    assert dist_at_transition > 0.3


# ── (b) FLOCK suppressed for the window after recovery, then resumes ─────────

def test_flock_steering_suppressed_after_escape_then_resumes():
    # A peer sits due north and in comm-range; the robot's escape heading
    # points due south. During the suppression window the robot must hold the
    # escape heading (turn south / negative), NOT chase the peer; after the
    # window FLOCK resumes and it turns toward the peer (north / positive).
    stub = make_robot_stub(pos_x=0.0, pos_y=0.0, yaw=0.0,
                           walk_turn_max=1.0, state=FLOCK)
    stub.peer_positions = {1: (0.0, 2.0)}     # peer due north, 2 m (in range)
    now = time.time()
    stub._escape_heading = -math.pi / 2       # hold heading = due south
    stub._escape_suppress_until = now + 4.0   # suppression active

    stub._update_motion_state()
    suppressed_cmd = stub._current_angular_z
    # Turning toward the escape heading (south), away from the peer.
    assert suppressed_cmd < 0.0

    # Window elapsed → FLOCK attraction resumes; now steer toward the peer.
    stub._escape_suppress_until = now - 1.0
    stub._update_motion_state()
    resumed_cmd = stub._current_angular_z
    assert resumed_cmd > 0.0
    assert stub._state == FLOCK


def test_near_wall_centre_steer_still_fires_during_suppression():
    # The boundary guard must remain live even while FLOCK is suppressed: a
    # robot past the near_wall band steers toward centre regardless of the
    # suppression window or the escape heading.
    half = 20.0 / 2.0 - 0.5
    stub = make_robot_stub(pos_x=half + 0.5, pos_y=0.0, yaw=0.0,
                           walk_turn_max=1.0, world_size=20.0, state=FLOCK)
    now = time.time()
    stub._escape_heading = 0.0                # would hold straight ahead...
    stub._escape_suppress_until = now + 4.0   # ...but suppression is active
    stub._update_motion_state()
    # Centre is behind the robot (it is past +x wall, facing +x): the guard
    # commands a turn (nonzero), i.e. it fired rather than being bypassed.
    assert abs(stub._current_angular_z) == pytest.approx(stub.walk_turn_max)


# ── (c) second trigger within radius + window escalates ─────────────────────

def test_second_trigger_within_radius_and_window_escalates():
    stub = make_robot_stub(pos_x=4.0, pos_y=0.0, walk_speed=0.2,
                           walk_turn_max=1.0)

    # First escape at (4.0, 0.0): not a repeat → base params + normal message.
    stub._enter_recovery()
    assert stub._escape_escalated is False
    assert stub._escape_reverse_target_m == pytest.approx(0.9)
    assert stub._escape_suppress_sec_current == pytest.approx(4.0)
    stub._exit_recovery()      # completes; records last-escape pos/time

    # Second escape 0.5 m away (< 1.0 m radius) and immediately (< 60 s
    # window) → escalate: doubled reverse + suppression, escalating message.
    stub.pos_x, stub.pos_y = 4.4, 0.3         # ~0.5 m from (4.0, 0.0)
    stub._enter_recovery()
    assert stub._escape_escalated is True
    assert stub._escape_reverse_target_m == pytest.approx(1.8)      # doubled
    assert stub._escape_suppress_sec_current == pytest.approx(8.0)  # doubled
    msgs = [c.args[0] for c in stub.logger.info.call_args_list]
    assert any('escalating escape' in m for m in msgs)

    # The escalated rotation must turn broadly away but never straight back
    # along the incoming axis: uniform over [90, 270] deg, excluding the
    # +/-20 deg band around 180. Sample many escalated draws (real RNG) and
    # check every one lands in [90, 270] and outside [160, 200] — a plain
    # uniform without the reject band would eventually land in the band.
    for _ in range(300):
        stub.pos_x, stub.pos_y, stub.yaw = 4.4, 0.3, 0.0
        stub._enter_recovery()              # stays escalated (same spot, recent)
        assert stub._escape_escalated is True
        stub._reverse_time_cap_sec = 0.0    # end the reverse leg on first tick
        stub._advance_recovery()            # reverse → rotate; draws the angle
        assert stub._recovery_phase == RECOVER_ROTATE
        turn = stub._recovery_target_yaw    # yaw == 0, so target == drawn turn
        assert _RECOVERY_ROTATE_ESCALATE_MIN_RAD <= turn \
            <= _RECOVERY_ROTATE_ESCALATE_MAX_RAD
        assert not (_RECOVERY_ROTATE_ESCALATE_EXCLUDE_LO_RAD <= turn
                    <= _RECOVERY_ROTATE_ESCALATE_EXCLUDE_HI_RAD)
        stub._recovery_phase = None         # reset for the next draw


# ── (d) trigger outside radius OR window does NOT escalate ───────────────────

def test_trigger_outside_radius_does_not_escalate():
    stub = make_robot_stub(pos_x=4.0, pos_y=0.0)
    stub._enter_recovery()
    stub._exit_recovery()
    # Second trigger 3 m away (> 1.0 m radius), still inside the time window
    # → no escalation: base reverse + suppression, no escalating message.
    stub.pos_x, stub.pos_y = 7.0, 0.0
    stub.logger.reset_mock()
    stub._enter_recovery()
    assert stub._escape_escalated is False
    assert stub._escape_reverse_target_m == pytest.approx(0.9)
    assert stub._escape_suppress_sec_current == pytest.approx(4.0)
    (msg,), _ = stub.logger.info.call_args
    assert 'escalating' not in msg


def test_trigger_outside_window_does_not_escalate():
    stub = make_robot_stub(pos_x=4.0, pos_y=0.0)
    stub._enter_recovery()
    stub._exit_recovery()
    # Age the previous escape past the repeat window so a spatially-close
    # trigger is not treated as a repeat.
    stub._last_escape_time -= (stub.escape_repeat_window_sec + 1.0)
    stub.pos_x, stub.pos_y = 4.2, 0.0         # only 0.2 m away, but stale
    stub.logger.reset_mock()
    stub._enter_recovery()
    assert stub._escape_escalated is False
    assert stub._escape_reverse_target_m == pytest.approx(0.9)
    assert stub._escape_suppress_sec_current == pytest.approx(4.0)
    (msg,), _ = stub.logger.info.call_args
    assert 'escalating' not in msg
