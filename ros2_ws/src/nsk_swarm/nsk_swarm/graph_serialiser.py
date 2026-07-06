"""
graph_serialiser.py
Serialise/deserialise PyG Data objects to/from JSON-safe dicts.
- graph_to_dict: no PyTorch import — safe for ROS 2 (system Python) side
- dict_to_graph: imports torch/torch_geometric — call from engine side only
"""


def graph_to_dict(g) -> dict:
    """Serialise PyG Data → JSON-safe dict. Works without torch import."""
    return {
        'x':          g.x.tolist(),
        'edge_index': g.edge_index.tolist(),
        'edge_type':  g.edge_type.tolist(),
        'num_nodes':  int(g.num_nodes),
        'agent_id':   int(g.agent_id) if hasattr(g, 'agent_id') else -1,
    }


def dict_to_graph(d: dict):
    """Deserialise dict → PyG Data. Requires torch (call from engine side only)."""
    import torch
    from torch_geometric.data import Data
    return Data(
        x=torch.tensor(d['x'], dtype=torch.float),
        edge_index=torch.tensor(d['edge_index'], dtype=torch.long),
        edge_type=torch.tensor(d['edge_type'], dtype=torch.long),
        num_nodes=d['num_nodes'],
    )
