"""Layer 1 — the Nav2 readiness gate in FrontierExplorer.

`waitUntilNav2Active()` waits on bt_navigator only; controller_server and
planner_server can still be configuring when it returns, and every goal sent in
that window is rejected outright ("Action server is inactive. Rejecting the
goal.") and misread as a dead frontier. `_wait_for_nav2_servers_active` closes
that window by polling each server's lifecycle get_state until it reports
'active'.

Its happy path shows up in any successful run; its TIMEOUT path does not — by
construction it only fires when the stack is broken, which is exactly when you
need the diagnostic to be right. So it is driven here instead.

Per the stub pattern in test_robot_node.py / test_spawn_offsets.py, the real
unbound methods (`_wait_for_nav2_servers_active`, plus the real `_hb` and
`_sleep` they lean on) are bound to a plain SimpleNamespace — no rclpy.init, no
node, no BasicNavigator. The module's `time` and `rclpy` are swapped for fakes,
so the whole 120-second budget is exercised without sleeping for any of it and
every assertion about elapsed time is exact rather than flaky.
"""

import os
from types import SimpleNamespace

import pytest

# nav2_simple_commander (ros-jazzy-navigation2) is a hard, module-level
# dependency of frontier_explorer — FrontierExplorer SUBCLASSES BasicNavigator,
# so the import cannot be deferred into a function. Guarded in the same shape as
# the torch guard (see conftest.py), including why it is a skipif MARKER and not
# pytest.skip(allow_module_level): under this project's plugin set a
# module-level Skipped aborts the whole session.
#
# CI sets NSK_REQUIRE_NAV2=1 and installs Nav2 deliberately, so a miss there is
# fatal. These tests are the reason the gate is trusted; skipping them in CI
# would leave the readiness gate covered by nothing.
try:
    from nsk_swarm import frontier_explorer as fx
    from nsk_swarm.frontier_explorer import FrontierExplorer
except ImportError:
    if os.environ.get('NSK_REQUIRE_NAV2', '') not in ('', '0', 'false'):
        raise
    fx = FrontierExplorer = None

pytestmark = pytest.mark.skipif(
    fx is None,
    reason='nav2_simple_commander is not importable — install '
           'ros-jazzy-navigation2 to run the Nav2 readiness-gate tests')


class FakeClock:
    """Deterministic stand-in for the `time` module inside frontier_explorer.

    sleep() advances the clock instead of blocking. The 1e-9 floor guarantees
    forward progress: `_sleep` slices its nap into 0.1 s steps and the residual
    of that division can round to a value too small to move a float, which
    would spin forever on a fake clock (a real one keeps ticking regardless).
    """

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, 1e-9)


class FakeSensor:
    """Scripted stand-in for _SensorNode's lifecycle_state().

    `script` maps a server name to the sequence of labels successive calls
    return; the final entry repeats forever. None models "get_state not
    advertised, or no reply within the per-call budget".
    """

    def __init__(self, script):
        self.script = {name: list(labels) for name, labels in script.items()}
        self.calls = []

    def lifecycle_state(self, server, wait=None):
        self.calls.append(server)
        labels = self.script.get(server, [None])
        return labels.pop(0) if len(labels) > 1 else labels[0]


def make_explorer_stub(script, robot_id=0):
    stub = SimpleNamespace(
        robot_id=robot_id,
        sensor=FakeSensor(script),
        _hb_last={},
        logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.warn = lambda msg: stub.logged.append(('WARN', msg))
    stub.error = lambda msg: stub.logged.append(('ERROR', msg))
    for name in ('_wait_for_nav2_servers_active', '_hb', '_sleep'):
        setattr(stub, name, getattr(FrontierExplorer, name).__get__(stub))
    return stub


def messages(stub, level=None):
    return [msg for lvl, msg in stub.logged if level is None or lvl == level]


@pytest.fixture
def clock(monkeypatch):
    """Swap the module's `time` and `rclpy` for fakes, scoped to one test."""
    fake = FakeClock()
    monkeypatch.setattr(fx, 'time', fake)
    monkeypatch.setattr(fx, 'rclpy', SimpleNamespace(ok=lambda: True))
    return fake


# ── the gate opens ───────────────────────────────────────────────────────────

def test_both_servers_already_active_passes_immediately(clock):
    stub = make_explorer_stub({'controller_server': ['active'],
                               'planner_server': ['active']})
    assert stub._wait_for_nav2_servers_active() is True
    assert clock.now == 0.0            # no polling delay when the stack is up
    logs = messages(stub, 'INFO')
    assert any('controller_server is active' in m for m in logs)
    assert any('planner_server is active' in m for m in logs)
    assert any('readiness gate passed' in m for m in logs)


def test_a_late_server_is_waited_for_and_logged_when_it_flips(clock):
    # controller_server is up; planner_server spends three polls configuring.
    stub = make_explorer_stub({
        'controller_server': ['active'],
        'planner_server': ['unconfigured', 'unconfigured', 'inactive', 'active'],
    })
    assert stub._wait_for_nav2_servers_active() is True

    # Three NAV2_READY_POLL naps to get there — the gate is not a fixed delay,
    # it costs exactly as long as the stack takes.
    assert clock.now == pytest.approx(3 * fx.NAV2_READY_POLL, abs=0.01)

    activations = [m for m in messages(stub, 'INFO') if ' is active ' in m]
    assert len(activations) == 2
    assert 'controller_server' in activations[0]
    assert 'planner_server' in activations[1]


def test_an_active_server_is_not_polled_again(clock):
    # Once a server reports active it leaves the pending set, so a slow peer
    # cannot make the gate re-query it every second for two minutes.
    stub = make_explorer_stub({
        'controller_server': ['active'],
        'planner_server': ['unconfigured'] * 5 + ['active'],
    })
    assert stub._wait_for_nav2_servers_active() is True
    assert stub.sensor.calls.count('controller_server') == 1
    assert stub.sensor.calls.count('planner_server') == 6


# ── the gate stays shut ──────────────────────────────────────────────────────

@pytest.mark.parametrize('label', ['unconfigured', 'inactive', 'activating',
                                   'finalized'])
def test_no_label_other_than_active_opens_the_gate(clock, label):
    # 'activating' in particular: the node is mid-transition and its action
    # server still rejects goals. Only 'active' counts.
    stub = make_explorer_stub({'controller_server': ['active'],
                               'planner_server': [label]})
    assert stub._wait_for_nav2_servers_active() is False


def test_timeout_names_the_stuck_server_and_not_the_healthy_one(clock):
    stub = make_explorer_stub({'controller_server': ['active'],
                               'planner_server': ['unconfigured']})
    assert stub._wait_for_nav2_servers_active() is False
    assert clock.now >= fx.NAV2_READY_TIMEOUT

    errors = messages(stub, 'ERROR')
    assert len(errors) == 1
    assert 'TIMED OUT' in errors[0]
    # The diagnostic has to point at the server that failed, with the state it
    # was stuck in — that is the whole reason this path exists.
    assert 'planner_server' in errors[0]
    assert 'unconfigured' in errors[0]
    # ...and must not implicate the server that came up fine.
    assert 'controller_server' not in errors[0]


def test_timeout_reports_a_server_that_never_answered(clock):
    # No get_state service at all (node never launched): the gate must still
    # fail loudly rather than reporting a bare 'None' state.
    stub = make_explorer_stub({'controller_server': [None],
                               'planner_server': [None]})
    assert stub._wait_for_nav2_servers_active() is False

    errors = messages(stub, 'ERROR')
    assert 'no get_state response' in errors[0]
    assert 'controller_server' in errors[0] and 'planner_server' in errors[0]


def test_gate_emits_heartbeats_while_it_waits(clock):
    # A silent block is the failure mode this project keeps re-learning; the
    # wait must narrate itself.
    stub = make_explorer_stub({'controller_server': ['active'],
                               'planner_server': ['unconfigured']})
    stub._wait_for_nav2_servers_active()
    heartbeats = [m for m in messages(stub, 'INFO')
                  if 'waiting for lifecycle state active' in m]
    assert heartbeats, 'the gate must heartbeat while blocked'
    # Throttled by HEARTBEAT_PERIOD, not one line per 1 s poll.
    assert len(heartbeats) < fx.NAV2_READY_TIMEOUT / fx.NAV2_READY_POLL


def test_gate_gives_up_when_rclpy_shuts_down(monkeypatch):
    # SIGINT during bringup: the gate must return False, not spin on a dead
    # context until its 120 s budget expires.
    fake = FakeClock()
    monkeypatch.setattr(fx, 'time', fake)
    monkeypatch.setattr(fx, 'rclpy', SimpleNamespace(ok=lambda: False))
    stub = make_explorer_stub({'controller_server': ['active'],
                               'planner_server': ['active']})
    assert stub._wait_for_nav2_servers_active() is False
    assert fake.now == 0.0


# ── Layer 0 — the activation wait BELOW the readiness gate ───────────────────
#
# _wait_for_nav2_servers_active above is bounded in three independent places
# (NAV2_READY_TIMEOUT, NAV2_STATE_CALL_WAIT, and lifecycle_state dropping its
# own timed-out futures). The wait that runs BEFORE it was not bounded at all:
# waitUntilNav2Active -> BasicNavigator._waitForNodeToActivate spins on the
# get_state future with no timeout_sec, so one undelivered response parks the
# explorer forever and the readiness gate below is never reached to time out.
# That is the aborted_k0_gate_hang failure. FrontierExplorer overrides the
# method; these pin the override.


class WouldBlockForever(Exception):
    """Raised by the fake spin when asked to wait with no timeout.

    Models what rclpy.spin_until_future_complete(node, future) does with a
    future that never completes: it does not return. Raising instead of
    actually blocking keeps the regression test deterministic and instant,
    and still fails loudly on any code path that reaches an unbounded wait.
    """


class FakeFuture:
    def __init__(self, answerable, label):
        self.answerable = answerable
        self.label = label
        self._done = False
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        if not self._done:
            return None
        return SimpleNamespace(current_state=SimpleNamespace(label=self.label))

    def cancel(self):
        self.cancelled = True


class FakeStateClient:
    """A get_state client whose answers are scripted call by call.

    `answers` is one bool per call -- True the response arrives, False it is
    lost, exactly the bt_navigator "failed to send response ... (timeout)" in
    the evidence log. The last entry repeats.
    """

    def __init__(self, answers, label='active'):
        self.answers = list(answers)
        self.label = label
        self.futures = []
        self.removed = []

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, req):
        answerable = (self.answers.pop(0) if len(self.answers) > 1
                      else self.answers[0])
        future = FakeFuture(answerable, self.label)
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def make_activate_stub(answers, label='active'):
    client = FakeStateClient(answers, label)
    stub = SimpleNamespace(client=client, logged=[])
    stub.info = lambda msg: stub.logged.append(('INFO', msg))
    stub.debug = lambda msg: stub.logged.append(('DEBUG', msg))
    stub.create_client = lambda srv_type, name: client
    stub._waitForNodeToActivate = (
        FrontierExplorer._waitForNodeToActivate.__get__(stub))
    return stub


@pytest.fixture
def spin(monkeypatch):
    """Swap `time` and `rclpy` for fakes that model spin_until_future_complete.

    With a timeout_sec it returns when the response does not arrive, leaving
    the future not done -- which is what gives the override something to
    retry. Without one it never returns, which is upstream's bug.
    """
    fake = FakeClock()

    def spin_until_future_complete(node, future, timeout_sec=None):
        if future.answerable:
            future._done = True
            return
        if timeout_sec is None:
            raise WouldBlockForever(
                'spin_until_future_complete with no timeout_sec on a response '
                'that never arrives')
        fake.sleep(timeout_sec)

    fake_rclpy = SimpleNamespace(
        ok=lambda: True,
        spin_until_future_complete=spin_until_future_complete)
    monkeypatch.setattr(fx, 'time', fake)
    monkeypatch.setattr(fx, 'rclpy', fake_rclpy)
    # robot_navigator resolves `rclpy` and `time` from its OWN module globals
    # (its lines 41 and 18), so patching frontier_explorer's names alone leaves
    # upstream's method running against the real rclpy. Both modules get the
    # same fakes, so the two methods below differ only in themselves.
    import nav2_simple_commander.robot_navigator as rn
    monkeypatch.setattr(rn, 'time', fake)
    monkeypatch.setattr(rn, 'rclpy', fake_rclpy)
    return fake


def test_a_lost_get_state_response_is_retried_not_waited_on_forever(spin):
    # The evidence: first response lost, node active by the second ask.
    stub = make_activate_stub([False, True])
    stub._waitForNodeToActivate('bt_navigator')      # must return at all

    assert len(stub.client.futures) == 2, 'the lost call must be re-sent'
    lost, answered = stub.client.futures
    assert lost.cancelled and stub.client.removed == [lost], (
        'a timed-out request must be cancelled AND removed from the pending '
        'map, or a late reply sits there forever')
    assert not answered.cancelled

    retry = [m for m in messages(stub, 'INFO') if 'unanswered' in m]
    assert retry == ['get_state on bt_navigator unanswered after 5 s, '
                     'asking again (attempt 1)'], retry


def test_the_unpatched_upstream_wait_never_returns(spin):
    """The regression this fixes: upstream reaches an unbounded wait.

    Same stub, same lost response — but BasicNavigator's own method, which
    omits timeout_sec. In a real process this is the 23-minute silence in
    aborted_k0_gate_hang; here the fake refuses to pretend it returns.
    """
    from nav2_simple_commander.robot_navigator import BasicNavigator

    stub = make_activate_stub([False, True])
    stub._waitForNodeToActivate = (
        BasicNavigator._waitForNodeToActivate.__get__(stub))
    with pytest.raises(WouldBlockForever):
        stub._waitForNodeToActivate('bt_navigator')


def test_an_answering_stack_sees_no_change(spin):
    # One call, one answer, no retry logged, and upstream's trailing
    # time.sleep(2) still paid — the happy path is untouched.
    stub = make_activate_stub([True])
    stub._waitForNodeToActivate('bt_navigator')
    assert len(stub.client.futures) == 1
    assert stub.client.removed == []
    assert not [m for m in messages(stub, 'INFO') if 'unanswered' in m]
    assert spin.now == pytest.approx(2.0)


def test_it_keeps_asking_until_the_node_answers_active(spin):
    # Three lost responses in a row do not end the wait, and each is counted.
    stub = make_activate_stub([False, False, False, True])
    stub._waitForNodeToActivate('bt_navigator')
    assert len(stub.client.futures) == 4
    assert len(stub.client.removed) == 3
    retry = [m for m in messages(stub, 'INFO') if 'unanswered' in m]
    assert [m[-len('(attempt N)'):] for m in retry] == [
        '(attempt 1)', '(attempt 2)', '(attempt 3)']


def test_a_node_answering_not_active_is_polled_again(spin):
    # Answers arrive, but the label is not 'active' until the third — the
    # 'active' test itself is upstream's and must be unchanged.
    stub = make_activate_stub([True], label='inactive')

    labels = iter(['configuring', 'inactive', 'active'])
    original = stub.client.call_async

    def call_async(req):
        future = original(req)
        future.label = next(labels)
        return future

    stub.client.call_async = call_async
    stub._waitForNodeToActivate('bt_navigator')
    assert len(stub.client.futures) == 3
    assert stub.client.removed == []        # nothing timed out; nothing dropped


# ── the constant the launch file is pinned against ───────────────────────────

def test_required_servers_are_the_goal_executing_ones():
    # Not bt_navigator (waitUntilNav2Active already covers it) and not the
    # optional servers — these two are the ones that reject or abort a goal
    # when they are not up. test_launch_descriptions.py asserts the launch file
    # actually brings both of them up.
    assert fx.NAV2_REQUIRED_SERVERS == ('controller_server', 'planner_server')
