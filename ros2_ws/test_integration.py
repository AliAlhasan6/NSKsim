#!/usr/bin/env python3
"""
test_integration.py — NSK engine server end-to-end test over ZMQ.

Run in nsk_env (no ROS 2, no Gazebo required):
    cd ~/Desktop/NSK
    source nsk_env/bin/activate
    CUDA_VISIBLE_DEVICES="" python test_integration.py

Starts the engine server as a subprocess, runs three test suites, terminates.
"""

import json
import subprocess
import sys
import time

import zmq


def req(sock, payload: dict) -> dict:
    sock.send_string(json.dumps(payload))
    raw = sock.recv_string()
    return json.loads(raw)


def main():
    endpoint = 'ipc:///tmp/nsk_test'

    print('=' * 60)
    print('NSK Integration Test')
    print('=' * 60)

    # ── Start engine server as subprocess ────────────────────────────────────
    print('\n[1/4] Starting NSK engine server subprocess...')
    proc = subprocess.Popen(
        [
            sys.executable, '-m', 'nsk_swarm.nsk_engine.engine_server',
            '--num_robots', '5',
            '--endpoint',   endpoint,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    print('      Waiting 8 s for startup (model load + dataset)...')
    time.sleep(8)

    if proc.poll() is not None:
        print('[FAIL] Engine server exited prematurely!')
        out, _ = proc.communicate()
        print(out)
        sys.exit(1)

    # ── Connect ZMQ ──────────────────────────────────────────────────────────
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 10_000)   # 10 s receive timeout
    sock.connect(endpoint)

    all_passed = True

    try:
        # ── Test 1: Compress all 5 robots ────────────────────────────────────
        print('\n[2/4] Test 1: Compress all 5 robots')
        compressed = {}
        for i in range(5):
            resp = req(sock, {'type': 'compress_request', 'agent_id': i})
            assert resp['type'] == 'compressed_graph', \
                f'FAIL compress robot {i}: got {resp}'
            ret = resp['stats']['node_retention']
            assert ret < 0.99, \
                f'FAIL retention still 1.0 for robot {i} (got {ret:.4f})'
            compressed[i] = resp['graph']
            print(f'  PASS compress robot {i}: '
                  f'retention={ret:.3f}  '
                  f'bridge_nodes={resp["stats"]["bridge_nodes_kept"]}')

        # ── Test 2: Robot 0 merges from robots 1, 2, 3, 4 ───────────────────
        print('\n[3/4] Test 2: Robot 0 merges from robots 1, 2, 3, 4')
        for sender in [1, 2, 3, 4]:
            resp = req(sock, {
                'type':      'merge_request',
                'agent_id':  0,
                'graph':     compressed[sender],
                'sender_id': sender,
            })
            assert resp['type'] == 'merge_done', \
                f'FAIL merge from {sender}: got {resp}'
            z_norm = resp['z_norm']
            assert abs(z_norm - 1.0) < 1e-4, \
                f'FAIL z* not normalised for sender {sender}: z_norm={z_norm:.6f}'
            z_star = resp['z_star']
            nan_count = sum(1 for v in z_star if v != v)
            assert nan_count == 0, \
                f'FAIL NaN in z* after merge from {sender}: {nan_count} NaN values'
            print(f'  PASS merge robot 0 ← robot {sender}: '
                  f'z_norm={z_norm:.4f}  gate={resp["gate"]:.3f}')

        # ── Test 3: Similarity query ──────────────────────────────────────────
        print('\n[4/4] Test 3: Similarity query all 5 robots')
        resp = req(sock, {'type': 'similarity_query', 'agent_ids': [0, 1, 2, 3, 4]})
        assert resp['type'] == 'similarity_response', \
            f'FAIL similarity_query: got {resp}'
        mean_sim = resp['mean_sim']
        matrix   = resp['matrix']
        assert -1.0 <= mean_sim <= 1.0, \
            f'FAIL similarity out of range: {mean_sim}'
        assert len(matrix) == 5 and len(matrix[0]) == 5, \
            f'FAIL matrix shape: {len(matrix)}x{len(matrix[0])}'
        # Diagonal should be ≈ 1.0
        for i in range(5):
            diag = matrix[i][i]
            assert abs(diag - 1.0) < 1e-3, \
                f'FAIL diagonal [{i},{i}] = {diag:.4f} (expected ≈1.0)'
        print(f'  PASS similarity query: mean_sim={mean_sim:.4f}')
        print(f'  Similarity matrix (rounded):')
        for row in matrix:
            print('    ' + '  '.join(f'{v:+.3f}' for v in row))

        # ── Test 4: Error recovery ────────────────────────────────────────────
        print('\nBonus: Test unknown request type → error response')
        resp = req(sock, {'type': 'unknown_request_xyz'})
        assert resp['type'] == 'error', \
            f'FAIL: expected error response, got {resp}'
        print(f'  PASS error response: {resp["message"]}')

    except AssertionError as e:
        print(f'\n✗ ASSERTION FAILED: {e}')
        all_passed = False
    except zmq.ZMQError as e:
        print(f'\n✗ ZMQ ERROR: {e}')
        all_passed = False
    except Exception as e:
        print(f'\n✗ UNEXPECTED ERROR: {e}')
        all_passed = False
    finally:
        sock.close()
        ctx.term()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print('\n' + '=' * 60)
    if all_passed:
        print('✓  All integration tests PASSED.')
    else:
        print('✗  Some tests FAILED — see output above.')
    print('=' * 60)
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
