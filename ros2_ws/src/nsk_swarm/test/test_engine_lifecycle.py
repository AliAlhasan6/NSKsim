"""Layer 2 — NSKEngineNode lifecycle with a mocked AgentManager.

Mocking choice: a stub module is installed at
sys.modules['nsk_engine.agent_manager'] before configure is triggered.
on_configure imports AgentManager lazily (`from nsk_engine.agent_manager
import AgentManager` inside the callback), and Python's import machinery
short-circuits to sys.modules before touching the real module — so the real
agent_manager.py (whose construction needs the NSK checkpoint and
/home/lawlite/Desktop/NSK) is never even imported. This is cleaner than
monkeypatching an attribute on the real module.

Transition driving: verified on rclpy Jazzy that LifecycleNode's
trigger_configure()/trigger_activate()/trigger_deactivate() execute the
transition callbacks synchronously in the calling thread and return the
TransitionCallbackReturn — no executor, spinning, or service client needed.
"""

import json
import sys
import types
from types import SimpleNamespace
from unittest import mock

import pytest
import torch
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from nsk_swarm_interfaces.srv import Compress, Merge, SimilarityQuery

from nsk_engine.engine_server import NSKEngineNode

NSK_SERVICE_NAMES = {'/nsk/compress', '/nsk/merge', '/nsk/similarity_query'}


def service_names(node):
    # rclpy reports parameter services with their unexpanded relative names
    # ('nsk_engine/get_parameters') but lifecycle and /nsk/* services fully
    # qualified. Node namespace is '/', so prefixing normalises them all.
    return [s.srv_name if s.srv_name.startswith('/') else '/' + s.srv_name
            for s in node.services]


@pytest.fixture()
def engine_node(rclpy_context):
    node = NSKEngineNode()
    yield node
    node.destroy_node()


@pytest.fixture()
def fake_agent_manager(monkeypatch):
    """Stub module shadowing nsk_engine.agent_manager in sys.modules.
    Returns the AgentManager class mock (configure side_effect to fail)."""
    module = types.ModuleType('nsk_engine.agent_manager')
    module.AgentManager = mock.MagicMock(name='AgentManager')
    monkeypatch.setitem(sys.modules, 'nsk_engine.agent_manager', module)
    return module.AgentManager


@pytest.fixture()
def configured_params(engine_node, tmp_path):
    """Point the node at a throwaway config/base dir so on_configure's file
    I/O succeeds without /home/lawlite/Desktop/NSK. Restores sys.path, which
    on_configure extends with nsk_base_path."""
    config = tmp_path / 'base.yaml'
    config.write_text('model: {}\n')
    engine_node.set_parameters([
        Parameter('config_path', value=str(config)),
        Parameter('checkpoint_path', value=str(tmp_path / 'ckpt.pt')),
        Parameter('nsk_base_path', value=str(tmp_path)),
    ])
    saved_sys_path = list(sys.path)
    yield tmp_path
    sys.path[:] = saved_sys_path


# ── (a) construction / _services shadowing regression guard ─────────────────

def test_init_does_not_shadow_rclpy_service_registry(engine_node):
    # REGRESSION GUARD for the `_services` shadowing bug: rclpy.Node keeps
    # its service registry in `self._services`. NSKEngineNode.__init__ once
    # initialised its own list under that same name, wiping the registry and
    # silently detaching every already-created service (parameter +
    # lifecycle) from the executor. The registry must still be populated
    # right after __init__, and the node's own list must live elsewhere.
    names = service_names(engine_node)
    assert names, 'rclpy service registry is empty — _services was shadowed'
    node_name = engine_node.get_name()
    assert f'/{node_name}/get_parameters' in names
    assert f'/{node_name}/set_parameters' in names
    assert f'/{node_name}/change_state' in names       # lifecycle service
    assert engine_node._nsk_services == []             # internal registry


# ── (b) on_configure outcomes ────────────────────────────────────────────────

def test_on_configure_success_with_mocked_manager(
        engine_node, configured_params, fake_agent_manager):
    assert engine_node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert engine_node.manager is fake_agent_manager.return_value

    kwargs = fake_agent_manager.call_args.kwargs
    assert kwargs['config'] == {'model': {}}
    assert kwargs['checkpoint_path'] == str(configured_params / 'ckpt.pt')
    assert kwargs['nsk_base_path'] == str(configured_params)
    assert kwargs['dataset_indices'] == [0, 900, 1800, 2700, 3600]
    assert kwargs['device'] == 'cpu'


def test_on_configure_failure_when_manager_raises(
        engine_node, configured_params, fake_agent_manager):
    fake_agent_manager.side_effect = RuntimeError('checkpoint corrupt')
    assert engine_node.trigger_configure() == TransitionCallbackReturn.FAILURE
    assert engine_node.manager is None


def test_seed_applied_before_manager_construction(
        engine_node, configured_params, fake_agent_manager):
    # Ordering matters: the AgentManager's train DataLoader draws from
    # torch's global RNG at iteration time, so a seed applied after
    # construction would be useless. Attaching both mocks to one parent
    # records their calls in a single timeline.
    engine_node.set_parameters([Parameter('seed', value=123)])
    with mock.patch('torch.manual_seed') as manual_seed:
        order = mock.Mock()
        order.attach_mock(manual_seed, 'manual_seed')
        order.attach_mock(fake_agent_manager, 'AgentManager')
        assert engine_node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    manual_seed.assert_called_once_with(123)
    call_names = [c[0] for c in order.mock_calls]
    assert call_names.index('manual_seed') < call_names.index('AgentManager')


def test_default_seed_leaves_torch_unseeded(
        engine_node, configured_params, fake_agent_manager):
    with mock.patch('torch.manual_seed') as manual_seed:
        assert engine_node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    manual_seed.assert_not_called()


# ── (c) /nsk/* services across activate/deactivate ──────────────────────────

def test_nsk_services_created_on_activate_destroyed_on_deactivate(
        engine_node, configured_params, fake_agent_manager):
    assert engine_node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert not NSK_SERVICE_NAMES & set(service_names(engine_node))

    assert engine_node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert NSK_SERVICE_NAMES <= set(service_names(engine_node))

    assert engine_node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    remaining = set(service_names(engine_node))
    assert not NSK_SERVICE_NAMES & remaining
    # Second angle on the shadowing bug: deactivate must remove ONLY the
    # /nsk/* servers, not empty the whole rclpy registry.
    node_name = engine_node.get_name()
    assert f'/{node_name}/get_parameters' in remaining


# ── (d) service callback contracts with a mocked manager ────────────────────

def make_compress_manager(stats=None):
    agent = mock.MagicMock()
    agent.compressor.config = {'retention_ratio': 0.40}
    g_tilde = SimpleNamespace(
        x=torch.tensor([[1.0, 0.5], [0.0, 2.0]]),
        edge_index=torch.tensor([[0, 1], [1, 0]]),
        edge_type=torch.tensor([0, 1]),
        num_nodes=2)
    agent.compress_and_share.return_value = (
        g_tilde, stats or {'node_retention': 0.42, 'bridge_nodes_kept': 3})
    manager = mock.MagicMock()
    manager.agents = {0: agent}
    return manager, agent


def test_compress_happy_path(engine_node):
    engine_node.manager, agent = make_compress_manager()
    req = Compress.Request(agent_id=0, retention_ratio=0.3)
    resp = engine_node._compress(req, Compress.Response())

    assert resp.success is True
    assert resp.message == ''
    graph = json.loads(resp.graph_json)            # must json-parse
    assert graph['num_nodes'] == 2
    assert graph['agent_id'] == 0
    assert graph['x'] == [[1.0, 0.5], [0.0, 2.0]]
    assert graph['edge_index'] == [[0, 1], [1, 0]]
    assert graph['edge_type'] == [0, 1]
    assert resp.node_retention == pytest.approx(0.42)
    assert resp.bridge_nodes_kept == 3
    # The adaptive retention override must be restored after the call.
    assert agent.compressor.config['retention_ratio'] == 0.40


def test_compress_sentinel_retention_leaves_config_untouched(engine_node):
    engine_node.manager, agent = make_compress_manager()
    agent.compressor.config = {}       # any write would show up as a new key
    req = Compress.Request(agent_id=0, retention_ratio=-1.0)
    resp = engine_node._compress(req, Compress.Response())
    assert resp.success is True
    assert agent.compressor.config == {}


def test_compress_manager_exception(engine_node):
    engine_node.manager, agent = make_compress_manager()
    agent.compress_and_share.side_effect = RuntimeError('boom')
    resp = engine_node._compress(
        Compress.Request(agent_id=0, retention_ratio=-1.0),
        Compress.Response())
    assert resp.success is False
    assert 'compress_request failed' in resp.message
    assert 'boom' in resp.message


def test_compress_unknown_agent(engine_node):
    engine_node.manager = mock.MagicMock()
    engine_node.manager.agents = {}
    resp = engine_node._compress(
        Compress.Request(agent_id=7, retention_ratio=-1.0),
        Compress.Response())
    assert resp.success is False
    assert resp.message != ''


def test_merge_happy_path(engine_node):
    manager = mock.MagicMock()
    manager.embedding_level_merge.return_value = {
        'z_star': [0.1, -0.2, 0.3], 'gate': 0.5, 'z_norm': 1.0}
    engine_node.manager = manager

    graph_dict = {'x': [[0.1, 0.2]], 'edge_index': [[0], [0]],
                  'edge_type': [0], 'num_nodes': 1}
    req = Merge.Request(agent_id=1, sender_id=0,
                        graph_json=json.dumps(graph_dict))
    resp = engine_node._merge(req, Merge.Response())

    assert resp.success is True
    assert resp.message == ''
    assert list(resp.z_star) == pytest.approx([0.1, -0.2, 0.3])
    assert resp.gate == 0.5
    assert resp.z_norm == 1.0
    # The manager receives the parsed dict, not the JSON string.
    manager.embedding_level_merge.assert_called_once_with(1, graph_dict)


def test_merge_manager_exception(engine_node):
    manager = mock.MagicMock()
    manager.embedding_level_merge.side_effect = ValueError('shape mismatch')
    engine_node.manager = manager
    req = Merge.Request(agent_id=1, sender_id=0, graph_json='{}')
    resp = engine_node._merge(req, Merge.Response())
    assert resp.success is False
    assert 'merge_request failed' in resp.message
    assert 'shape mismatch' in resp.message


def test_merge_invalid_json_reports_failure(engine_node):
    engine_node.manager = mock.MagicMock()
    req = Merge.Request(agent_id=1, sender_id=0, graph_json='not json')
    resp = engine_node._merge(req, Merge.Response())
    assert resp.success is False
    assert 'merge_request failed' in resp.message


def test_similarity_happy_path(engine_node):
    manager = mock.MagicMock()
    matrix = [[1.0, 0.2], [0.2, 1.0]]
    manager.pairwise_similarity.return_value = (matrix, 0.2)
    engine_node.manager = manager

    req = SimilarityQuery.Request(agent_ids=[0, 1])
    resp = engine_node._similarity(req, SimilarityQuery.Response())

    assert resp.success is True
    assert resp.message == ''
    assert json.loads(resp.matrix_json) == matrix   # must json-parse
    assert resp.mean_sim == pytest.approx(0.2)
    manager.pairwise_similarity.assert_called_once_with([0, 1])


def test_similarity_manager_exception(engine_node):
    manager = mock.MagicMock()
    manager.pairwise_similarity.side_effect = RuntimeError('no agents')
    engine_node.manager = manager
    resp = engine_node._similarity(
        SimilarityQuery.Request(agent_ids=[0, 1]),
        SimilarityQuery.Response())
    assert resp.success is False
    assert 'similarity_query failed' in resp.message
    assert 'no agents' in resp.message
