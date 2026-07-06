#!/usr/bin/env python3
"""
agent_manager.py — Manages all SwarmAgent instances for the NSK engine.

Runs in nsk_env. Imports PyTorch and PyG — do NOT import from ROS 2 nodes.
"""

import sys
import os

import torch
import yaml
from torch_geometric.data import Batch


class AgentManager:
    """
    Initialises and holds all SwarmAgent instances.
    Loads joint_best.pt once at startup.
    """

    def __init__(self, config: dict, checkpoint_path: str,
                 dataset_indices: list[int], nsk_base_path: str,
                 device: str = 'cpu'):
        self.device = torch.device(device)
        self.nsk_base_path = nsk_base_path

        # Add NSK src path
        src_path = os.path.join(nsk_base_path, 'src')
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        nsk_root = nsk_base_path
        if nsk_root not in sys.path:
            sys.path.insert(0, nsk_root)

        # Import NSK components (must be after path setup)
        from stage2_embedder.model import build_model
        from stage3_merger.merger import build_merger, KnowledgeMerger
        from multiagent.validate import SwarmAgent
        from stage1_compressor.compressor import GraphCompressor
        from utils.data_loader import get_dataloaders, load_config

        # Load dataset config
        nsk_config = load_config(os.path.join(nsk_base_path, 'configs/base.yaml'))

        # Load checkpoint
        ckpt = torch.load(
            checkpoint_path,
            map_location=self.device,
            weights_only=False)
        print(f'[AgentManager] Loaded checkpoint: epoch={ckpt["epoch"]}  '
              f'val_loss={ckpt["val_loss"]:.6f}')

        # Build embedder
        self.embedder = build_model(nsk_config)
        self.embedder.load_state_dict(ckpt['autoencoder_state'])
        self.embedder.to(self.device)
        self.embedder.eval()

        # Build merger
        merger = build_merger(nsk_config, num_relations=237)
        merger.load_state_dict(ckpt['merger_state'])
        merger.to(self.device)
        merger.eval()

        # Load dataset
        train_loader, val_loader, test_loader, dataset = get_dataloaders(
            nsk_config, root=nsk_base_path)
        # Collect all graphs
        all_graphs = []
        for batch in train_loader:
            # Unbatch
            for i in range(batch.num_graphs):
                mask = batch.batch == i
                g_x = batch.x[mask]
                # Re-extract edges belonging to this graph
                all_graphs.append(batch[i] if hasattr(batch, '__getitem__') else batch)
                if len(all_graphs) > max(dataset_indices) + 1:
                    break
            if len(all_graphs) > max(dataset_indices) + 1:
                break

        # Build compressor
        compressor = GraphCompressor(nsk_config)

        # Initialise agents
        self.agents: dict[int, SwarmAgent] = {}
        for idx, ds_idx in enumerate(dataset_indices):
            if ds_idx < len(all_graphs):
                local_graph = all_graphs[ds_idx]
            else:
                # Fallback: wrap around
                local_graph = all_graphs[ds_idx % len(all_graphs)]
            agent = SwarmAgent(
                agent_id=idx,
                local_graph=local_graph,
                embedder=self.embedder,
                compressor=compressor,
                merger=merger,
                device=self.device,
            )
            self.agents[idx] = agent
            print(f'[AgentManager] Initialised agent {idx} '
                  f'(dataset index {ds_idx}, '
                  f'{local_graph.num_nodes} nodes)')

        print(f'[AgentManager] {len(self.agents)} agents ready.')

    def get_z_stars(self, agent_ids: list) -> torch.Tensor:
        """Returns stacked [N, 32] tensor of z* for given agent_ids."""
        zs = [self.agents[i].z_star for i in agent_ids]
        return torch.cat(zs, dim=0)   # [N, 32]

    def pairwise_similarity(self, agent_ids: list) -> tuple[list, float]:
        """
        Returns (NxN cosine similarity matrix as list-of-lists, mean off-diagonal).
        """
        z = self.get_z_stars(agent_ids)  # [N, 32]
        # Normalise rows
        z_norm = z / z.norm(dim=1, keepdim=True).clamp(min=1e-8)
        sim_matrix = (z_norm @ z_norm.T).tolist()  # [N, N]
        n = len(agent_ids)
        off_diag = [sim_matrix[i][j]
                    for i in range(n) for j in range(n) if i != j]
        mean_sim = sum(off_diag) / len(off_diag) if off_diag else 0.0
        return sim_matrix, mean_sim

    def embedding_level_merge(self, receiver_id: int,
                               received_graph_dict: dict) -> dict:
        """
        Validated embedding-level merge (avoids OOD merger graph encoder issue).
        z* = L2-norm(0.7 * z_self + 0.3 * z_received)
        Returns dict with z_star, gate, z_norm.
        """
        from utils.data_loader import load_config  # noqa: F401
        # Import here to avoid circular at module level
        from torch_geometric.data import Data, Batch as TGBatch

        # Deserialise received graph
        g = Data(
            x=torch.tensor(received_graph_dict['x'], dtype=torch.float),
            edge_index=torch.tensor(received_graph_dict['edge_index'], dtype=torch.long),
            edge_type=torch.tensor(received_graph_dict['edge_type'], dtype=torch.long),
            num_nodes=received_graph_dict['num_nodes'],
        )
        g = g.to(self.device)
        batch = TGBatch.from_data_list([g])

        with torch.no_grad():
            z_received, _ = self.embedder(
                batch.x, batch.edge_index, batch.edge_type, batch.batch)
            # z_received: [1, hidden_dim]
            z_self = self.agents[receiver_id].z_star   # [1, hidden_dim]
            z_merged = 0.7 * z_self + 0.3 * z_received
            norm = z_merged.norm(dim=-1, keepdim=True).clamp(min=1e-8)
            z_star_new = z_merged / norm

        self.agents[receiver_id].z_star = z_star_new
        z_norm = float(z_star_new.norm().item())

        return {
            'z_star': z_star_new.squeeze(0).tolist(),
            'gate':   0.5,       # fixed for embedding-level merge
            'z_norm': z_norm,
        }
