"""Layer 1 — graph_to_dict / dict_to_graph round-trip (pure logic, no rclpy).

torch / torch_geometric come from the project venv's site-packages on
PYTHONPATH; no NSK checkpoint or dataset is needed for a synthetic Data.
"""

import json

import torch
from torch_geometric.data import Data

from nsk_swarm.graph_serialiser import dict_to_graph, graph_to_dict


def make_graph() -> Data:
    # 5 nodes but edges only among the first 4: num_nodes must survive the
    # round-trip explicitly, not be re-inferred from edge_index (which would
    # yield 4).
    x = torch.tensor([[1.0, 2.0, 3.0],
                      [4.0, 5.0, 6.0],
                      [7.0, 8.0, 9.0],
                      [0.5, -1.5, 2.5],
                      [0.0, 0.0, 0.0]], dtype=torch.float)
    edge_index = torch.tensor([[0, 1, 2, 3],
                               [1, 2, 3, 0]], dtype=torch.long)
    edge_type = torch.tensor([0, 3, 1, 2], dtype=torch.long)
    return Data(x=x, edge_index=edge_index, edge_type=edge_type, num_nodes=5)


def test_graph_to_dict_fields():
    g = make_graph()
    d = graph_to_dict(g)
    assert d['x'] == g.x.tolist()
    assert d['edge_index'] == g.edge_index.tolist()
    assert d['edge_type'] == g.edge_type.tolist()
    assert d['num_nodes'] == 5
    assert d['agent_id'] == -1     # graph carries no agent_id attribute


def test_graph_to_dict_preserves_agent_id():
    g = make_graph()
    g.agent_id = 3
    assert graph_to_dict(g)['agent_id'] == 3


def test_graph_dict_graph_roundtrip():
    g = make_graph()
    g2 = dict_to_graph(graph_to_dict(g))
    assert torch.equal(g2.x, g.x)
    assert torch.equal(g2.edge_index, g.edge_index)
    assert torch.equal(g2.edge_type, g.edge_type)
    assert g2.num_nodes == g.num_nodes == 5
    assert g2.x.dtype == torch.float
    assert g2.edge_index.dtype == torch.long
    assert g2.edge_type.dtype == torch.long


def test_dict_graph_dict_roundtrip():
    d = {
        'x':          [[0.25, -1.0], [3.5, 2.0], [0.0, 7.0]],
        'edge_index': [[0, 1], [1, 2]],
        'edge_type':  [4, 0],
        'num_nodes':  3,
        'agent_id':   2,
    }
    d2 = graph_to_dict(dict_to_graph(d))
    for key in ('x', 'edge_index', 'edge_type', 'num_nodes'):
        assert d2[key] == d[key]
    # dict_to_graph deliberately drops agent_id (engine-side field), so the
    # re-serialised dict reports the "absent" sentinel.
    assert d2['agent_id'] == -1


def test_serialised_dict_is_json_safe():
    # Both service payloads ship this dict through json.dumps — it must
    # survive a JSON round-trip unchanged.
    d = graph_to_dict(make_graph())
    assert json.loads(json.dumps(d)) == d
