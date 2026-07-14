"""Layer 1 — adaptive retention in NSKRobotNode._share_timer_cb.

Choice (per test_geometry.py): the real unbound _share_timer_cb and
_in_range are bound to a plain stub, with _call_engine replaced by a fake
answering the similarity and compress requests in-process. This exercises
the exact production share path — similarity refresh, min-sim selection,
retention banding, broadcast — with no rclpy init.

Matrix convention under test (shared with convergence_monitor): rows and
columns follow the request's agent_ids order ([self] + peers_in_range),
so matrix[0][k+1] is the similarity between this robot and
peers_in_range[k].
"""

import json
from types import SimpleNamespace
from unittest import mock

import pytest

from nsk_swarm.robot_node import NSKRobotNode, retention_for_similarity

GRAPH_JSON = json.dumps({'x': [[1.0, 2.0]], 'edge_index': [[0], [0]],
                         'edge_type': [0], 'num_nodes': 1, 'agent_id': 0})


def make_share_stub(sim_matrix, peer_similarity=None):
    """Robot 0 with peers 1 and 2 in range (peer 3 out of range).

    sim_matrix=None makes the similarity call return None — what the real
    _call_engine yields on unavailability, timeout or success=False.
    """
    stub = SimpleNamespace(
        robot_id=0, num_robots=4, comm_range=3.0,
        pos_x=0.0, pos_y=0.0,
        peer_positions={1: (1.0, 0.0), 2: (0.0, 1.0), 3: (10.0, 10.0)},
        peer_similarity=peer_similarity if peer_similarity is not None else {},
        _sim_cli=object(), _compress_cli=object(),
        _state='flock', _disperse_until=0.0,
        kg_pub=mock.MagicMock(),
        compress_requests=[],
        logger=mock.MagicMock())
    stub.get_logger = lambda: stub.logger

    def fake_call_engine(client, request):
        if client is stub._sim_cli:
            if sim_matrix is None:
                return None
            assert list(request.agent_ids) == [0, 1, 2]
            return SimpleNamespace(success=True,
                                   matrix_json=json.dumps(sim_matrix),
                                   mean_sim=0.0)
        assert client is stub._compress_cli
        stub.compress_requests.append(request)
        return SimpleNamespace(success=True, graph_json=GRAPH_JSON)

    stub._call_engine = fake_call_engine
    stub._in_range = NSKRobotNode._in_range.__get__(stub)
    stub._share_timer_cb = NSKRobotNode._share_timer_cb.__get__(stub)
    return stub


def test_similarity_refresh_updates_peers_and_retention():
    # Peer 1 nearly converged (0.9), peer 2 very different (0.1): the
    # broadcast must carry the richest graph for the most-different peer.
    matrix = [[1.0, 0.9, 0.1],
              [0.9, 1.0, 0.3],
              [0.1, 0.3, 1.0]]
    stub = make_share_stub(matrix)

    stub._share_timer_cb()

    assert stub.peer_similarity == pytest.approx({1: 0.9, 2: 0.1})
    assert len(stub.compress_requests) == 1
    req = stub.compress_requests[0]
    assert req.agent_id == 0
    assert req.retention_ratio == pytest.approx(retention_for_similarity(0.1))
    assert req.retention_ratio == pytest.approx(0.65)
    stub.kg_pub.publish.assert_called_once()


def test_high_similarity_unpins_retention_from_default():
    # REGRESSION GUARD for the dead-adaptivity bug: peer_similarity was
    # never written, so min_sim stayed 0.0 and retention was pinned at
    # 0.65. With every in-range peer similar, the request must now sit in
    # the compress-hard band.
    matrix = [[1.0, 0.9, 0.8],
              [0.9, 1.0, 0.7],
              [0.8, 0.7, 1.0]]
    stub = make_share_stub(matrix)

    stub._share_timer_cb()

    assert stub.peer_similarity == pytest.approx({1: 0.9, 2: 0.8})
    assert stub.compress_requests[0].retention_ratio == pytest.approx(0.20)


def test_similarity_failure_keeps_cached_values():
    stub = make_share_stub(None, peer_similarity={1: 0.55, 2: 0.5})

    stub._share_timer_cb()

    # Cache untouched — neither cleared nor zeroed — and retention derives
    # from the cached minimum (0.5 -> middle band), not the 0.0 default.
    assert stub.peer_similarity == {1: 0.55, 2: 0.5}
    assert stub.compress_requests[0].retention_ratio == pytest.approx(0.40)
    stub.logger.warn.assert_called_once()
    stub.kg_pub.publish.assert_called_once()
