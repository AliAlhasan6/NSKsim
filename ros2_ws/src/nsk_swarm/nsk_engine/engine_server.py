#!/usr/bin/env python3
"""
engine_server.py — NSK rclpy service node.

Run in nsk_env:
    CUDA_VISIBLE_DEVICES="" ros2 run nsk_swarm engine_server --ros-args \\
        -p num_robots:=5 \\
        -p config_path:=configs/base.yaml \\
        -p checkpoint_path:=experiments/checkpoints/joint_best.pt

Services:
    /nsk/compress          (nsk_swarm_interfaces/srv/Compress)
    /nsk/merge             (nsk_swarm_interfaces/srv/Merge)
    /nsk/similarity_query  (nsk_swarm_interfaces/srv/SimilarityQuery)
"""

import json
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nsk_swarm_interfaces.srv import Compress, Merge, SimilarityQuery


class NSKEngineNode(Node):

    def __init__(self):
        super().__init__('nsk_engine')

        self.declare_parameter('config_path', 'configs/base.yaml')
        self.declare_parameter('checkpoint_path',
                               'experiments/checkpoints/joint_best.pt')
        self.declare_parameter('dataset_indices', [0, 900, 1800, 2700, 3600])
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('nsk_base_path', '/home/lawlite/Desktop/NSK')

        config_path     = self.get_parameter('config_path').value
        checkpoint_path = self.get_parameter('checkpoint_path').value
        dataset_indices = [int(i) for i in
                           self.get_parameter('dataset_indices').value]
        num_robots      = int(self.get_parameter('num_robots').value)
        nsk_base        = os.path.abspath(
            self.get_parameter('nsk_base_path').value)

        checkpoint_path = (checkpoint_path if os.path.isabs(checkpoint_path)
                           else os.path.join(nsk_base, checkpoint_path))
        config_path     = (config_path if os.path.isabs(config_path)
                           else os.path.join(nsk_base, config_path))

        self.get_logger().info(f'NSK base:     {nsk_base}')
        self.get_logger().info(f'Checkpoint:   {checkpoint_path}')
        self.get_logger().info(f'Config:       {config_path}')
        self.get_logger().info(f'Num robots:   {num_robots}')
        self.get_logger().info(f'DS indices:   {dataset_indices}')

        # Add NSK paths
        for path in [nsk_base, os.path.join(nsk_base, 'src')]:
            if path not in sys.path:
                sys.path.insert(0, path)

        # Lazy import to give time for path setup
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Initialise AgentManager
        from nsk_engine.agent_manager import AgentManager
        self.manager = AgentManager(
            config=config,
            checkpoint_path=checkpoint_path,
            dataset_indices=dataset_indices[:num_robots],
            nsk_base_path=nsk_base,
            device='cpu',
        )

        self.create_service(Compress, '/nsk/compress', self._compress)
        self.create_service(Merge, '/nsk/merge', self._merge)
        self.create_service(SimilarityQuery, '/nsk/similarity_query',
                            self._similarity)

        self.get_logger().info('Ready.')

    def _compress(self, request, response):
        try:
            agent_id = int(request.agent_id)
            agent    = self.manager.agents[agent_id]
            # Apply adaptive retention ratio if provided (-1.0 = not provided)
            retention = (None if request.retention_ratio == -1.0
                         else float(request.retention_ratio))
            if retention is not None:
                original = agent.compressor.config.get('retention_ratio', 0.40)
                agent.compressor.config['retention_ratio'] = retention
            g_tilde, stats = agent.compress_and_share()
            if retention is not None:
                agent.compressor.config['retention_ratio'] = original
            # Serialise graph
            graph_dict = {
                'x':          g_tilde.x.tolist(),
                'edge_index': g_tilde.edge_index.tolist(),
                'edge_type':  g_tilde.edge_type.tolist(),
                'num_nodes':  int(g_tilde.num_nodes),
                'agent_id':   agent_id,
            }
            response.success           = True
            response.graph_json        = json.dumps(graph_dict)
            response.node_retention    = float(stats.get('node_retention', 0.0))
            response.bridge_nodes_kept = int(stats.get('bridge_nodes_kept', 0))
        except Exception as e:
            response.success = False
            response.message = f'compress_request failed: {e}'
            self.get_logger().error(response.message)
        return response

    def _merge(self, request, response):
        try:
            agent_id = int(request.agent_id)
            graph_d  = json.loads(request.graph_json)
            result   = self.manager.embedding_level_merge(agent_id, graph_d)
            response.success = True
            response.z_star  = [float(v) for v in result['z_star']]
            response.gate    = float(result['gate'])
            response.z_norm  = float(result['z_norm'])
        except Exception as e:
            response.success = False
            response.message = f'merge_request failed: {e}'
            self.get_logger().error(response.message)
        return response

    def _similarity(self, request, response):
        try:
            agent_ids = [int(i) for i in request.agent_ids]
            matrix, mean_sim = self.manager.pairwise_similarity(agent_ids)
            response.success     = True
            response.matrix_json = json.dumps(matrix)
            response.mean_sim    = float(mean_sim)
        except Exception as e:
            response.success = False
            response.message = f'similarity_query failed: {e}'
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NSKEngineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down.')
    finally:
        # Under `ros2 launch`, SIGINT may shut the context down at any moment,
        # even between an ok() check and the shutdown() call (check-then-act
        # race). Catch instead of check: a failed double-shutdown is a no-op.
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
