"""Layer 3 — real DDS round-trip: client call path vs a mock engine server.

Client-side choice (per task): NSKRobotNode.__init__ is too entangled to
instantiate here — it blocks in wait_for_service for BOTH /nsk/compress and
/nsk/merge and creates pubs/subs/timers — and there is no module-level call
helper. Instead a tiny harness node REUSES THE REAL NSKRobotNode._call_engine
method by assigning the unbound function in the harness class body, so the
exact production call path (call_async + sliced Event wait + timeout +
success gating + future discard) is under test, not a copy of it.

Both nodes are spun by one MultiThreadedExecutor in a background thread;
_call_engine is invoked from the pytest thread, mirroring how it blocks
inside a robot callback while another executor thread delivers the response.

The mock server's service uses a ReentrantCallbackGroup so the timeout
test's sleeping callback cannot serialise behind (or ahead of) another
test's request. The slow test runs last to keep total runtime low.
"""

import json
import threading
import time

import pytest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nsk_swarm_interfaces.srv import Compress

from nsk_swarm.robot_node import NSKRobotNode

GRAPH_DICT = {'x': [[1.0, 2.0]], 'edge_index': [[0], [0]],
              'edge_type': [0], 'num_nodes': 1, 'agent_id': 0}


class MockEngineServer(Node):
    """Offers /nsk/compress with the real srv type; per-test behaviour is
    selected via .mode ('ok' | 'fail') and .sleep_sec."""

    def __init__(self):
        super().__init__('mock_nsk_engine')
        self.mode = 'ok'
        self.sleep_sec = 0.0
        self.create_service(Compress, '/nsk/compress', self._handle,
                            callback_group=ReentrantCallbackGroup())

    def _handle(self, request, response):
        if self.sleep_sec:
            time.sleep(self.sleep_sec)
        if self.mode == 'ok':
            response.success = True
            response.graph_json = json.dumps(GRAPH_DICT)
            response.node_retention = 0.4
            response.bridge_nodes_kept = 2
        else:
            response.success = False
            response.message = 'engine exploded (mock)'
        return response


class EngineClientHarness(Node):
    # Production call helper, reused verbatim: it only touches
    # self.get_logger(), self.robot_id, self.service_timeout_sec and the
    # client object passed in — all provided by this harness.
    _call_engine = NSKRobotNode._call_engine

    def __init__(self):
        super().__init__('engine_client_harness')
        self.robot_id = 99
        self.service_timeout_sec = 2.0
        self.compress_cli = self.create_client(
            Compress, '/nsk/compress',
            callback_group=ReentrantCallbackGroup())


@pytest.fixture(scope='module')
def dds_ring(rclpy_context):
    server = MockEngineServer()
    client = EngineClientHarness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.compress_cli.wait_for_service(timeout_sec=10.0), \
        'mock /nsk/compress never became visible over DDS'
    yield server, client
    executor.shutdown()
    thread.join(timeout=10.0)
    server.destroy_node()
    client.destroy_node()


def make_request():
    return Compress.Request(agent_id=99, retention_ratio=0.4)


def test_happy_roundtrip(dds_ring):
    server, client = dds_ring
    server.mode, server.sleep_sec = 'ok', 0.0

    resp = client._call_engine(client.compress_cli, make_request())

    assert resp is not None
    assert resp.success is True
    graph = json.loads(resp.graph_json)
    assert graph == GRAPH_DICT
    assert resp.node_retention == pytest.approx(0.4)
    assert resp.bridge_nodes_kept == 2


def test_server_failure_returns_none_without_raising(dds_ring):
    server, client = dds_ring
    server.mode, server.sleep_sec = 'fail', 0.0

    resp = client._call_engine(client.compress_cli, make_request())

    # success=False is swallowed into None (with a logged warning), never
    # surfaced as an exception.
    assert resp is None


def test_timeout_returns_none_within_margin(dds_ring):
    server, client = dds_ring
    server.mode, server.sleep_sec = 'ok', 2.5   # sleeps past the timeout

    start = time.monotonic()
    resp = client._call_engine(client.compress_cli, make_request(),
                               timeout_sec=1.0)
    elapsed = time.monotonic() - start

    assert resp is None
    # Returned at the 1.0s timeout plus margin — well before the server's
    # 2.5s reply would have arrived.
    assert elapsed < 2.0
    # Let the sleeping server callback finish and its late reply drain
    # before module teardown shuts the executor down.
    time.sleep(server.sleep_sec - elapsed + 0.3)
