#!/usr/bin/env python3
"""
engine_server.py — NSK ZMQ REP server.

Run in nsk_env:
    CUDA_VISIBLE_DEVICES="" python -m nsk_swarm.nsk_engine.engine_server \\
        --num_robots 5 \\
        --endpoint ipc:///tmp/nsk_engine_0 \\
        --config configs/base.yaml \\
        --checkpoint experiments/checkpoints/joint_best.pt

Never import this from a ROS 2 node.
"""

import argparse
import json
import os
import sys

import zmq


def parse_args():
    p = argparse.ArgumentParser(description='NSK ZMQ engine server')
    p.add_argument('--num_robots',   type=int,   default=5)
    p.add_argument('--endpoint',     type=str,   default='ipc:///tmp/nsk_engine_0')
    p.add_argument('--config',       type=str,   default='configs/base.yaml')
    p.add_argument('--checkpoint',   type=str,
                   default='experiments/checkpoints/joint_best.pt')
    p.add_argument('--nsk_base',     type=str,   default=None,
                   help='Path to NSK root (default: auto-detect from script location)')
    p.add_argument('--dataset_indices', type=int, nargs='+',
                   default=[0, 900, 1800, 2700, 3600])
    return p.parse_args()


def main():
    args = parse_args()

    # Resolve NSK base path
    if args.nsk_base:
        nsk_base = os.path.abspath(args.nsk_base)
    else:
        # Assume engine_server.py lives in <NSK>/ros2_ws/src/nsk_swarm/nsk_engine/
        # or the user is running from <NSK>/
        nsk_base = os.path.abspath(os.getcwd())

    checkpoint_path = (args.checkpoint if os.path.isabs(args.checkpoint)
                       else os.path.join(nsk_base, args.checkpoint))
    config_path     = (args.config if os.path.isabs(args.config)
                       else os.path.join(nsk_base, args.config))

    print(f'[NSK Engine] NSK base:     {nsk_base}')
    print(f'[NSK Engine] Checkpoint:   {checkpoint_path}')
    print(f'[NSK Engine] Config:       {config_path}')
    print(f'[NSK Engine] Endpoint:     {args.endpoint}')
    print(f'[NSK Engine] Num robots:   {args.num_robots}')
    print(f'[NSK Engine] DS indices:   {args.dataset_indices}')

    # Add NSK paths
    for path in [nsk_base, os.path.join(nsk_base, 'src')]:
        if path not in sys.path:
            sys.path.insert(0, path)

    # Lazy import to give time for path setup
    import yaml
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Initialise AgentManager
    from nsk_swarm.nsk_engine.agent_manager import AgentManager
    manager = AgentManager(
        config=config,
        checkpoint_path=checkpoint_path,
        dataset_indices=args.dataset_indices[:args.num_robots],
        nsk_base_path=nsk_base,
        device='cpu',
    )

    # ── ZMQ REP socket ───────────────────────────────────────────────────────
    ctx    = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.bind(args.endpoint)
    print(f'[NSK Engine] Ready. Listening on {args.endpoint}')

    # ── Request loop ─────────────────────────────────────────────────────────
    while True:
        try:
            raw  = socket.recv_string()
            req  = json.loads(raw)
            resp = _handle(req, manager)
        except json.JSONDecodeError as e:
            resp = {'type': 'error', 'message': f'JSON decode error: {e}'}
        except KeyboardInterrupt:
            print('[NSK Engine] Shutting down.')
            break
        except Exception as e:
            resp = {'type': 'error', 'message': str(e)}
            print(f'[NSK Engine] Unhandled exception: {e}')

        try:
            socket.send_string(json.dumps(resp))
        except Exception as e:
            print(f'[NSK Engine] Failed to send response: {e}')

    socket.close()
    ctx.term()


def _handle(req: dict, manager) -> dict:
    """Dispatch a ZMQ request to the appropriate handler."""
    rtype = req.get('type', '')

    if rtype == 'compress_request':
        return _compress(req, manager)

    elif rtype == 'merge_request':
        return _merge(req, manager)

    elif rtype == 'similarity_query':
        return _similarity(req, manager)

    else:
        return {'type': 'error', 'message': f'Unknown request type: {rtype}'}


def _compress(req: dict, manager) -> dict:
    try:
        agent_id = int(req['agent_id'])
        agent    = manager.agents[agent_id]
        # Apply adaptive retention ratio if provided
        retention = req.get('retention_ratio', None)
        if retention is not None:
            original = agent.compressor.config.get('retention_ratio', 0.40)
            agent.compressor.config['retention_ratio'] = float(retention)
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
        return {
            'type':     'compressed_graph',
            'agent_id': agent_id,
            'graph':    graph_dict,
            'stats': {
                'node_retention':  float(stats.get('node_retention', 0.0)),
                'bridge_nodes_kept': int(stats.get('bridge_nodes_kept', 0)),
            },
        }
    except Exception as e:
        return {'type': 'error', 'message': f'compress_request failed: {e}'}


def _merge(req: dict, manager) -> dict:
    try:
        agent_id  = int(req['agent_id'])
        sender_id = int(req['sender_id'])
        graph_d   = req['graph']
        result    = manager.embedding_level_merge(agent_id, graph_d)
        return {
            'type':      'merge_done',
            'agent_id':  agent_id,
            'sender_id': sender_id,
            'z_star':    result['z_star'],
            'gate':      result['gate'],
            'z_norm':    result['z_norm'],
        }
    except Exception as e:
        return {'type': 'error', 'message': f'merge_request failed: {e}'}


def _similarity(req: dict, manager) -> dict:
    try:
        agent_ids  = [int(i) for i in req['agent_ids']]
        matrix, mean_sim = manager.pairwise_similarity(agent_ids)
        return {
            'type':     'similarity_response',
            'matrix':   matrix,
            'mean_sim': float(mean_sim),
        }
    except Exception as e:
        return {'type': 'error', 'message': f'similarity_query failed: {e}'}


if __name__ == '__main__':
    main()
