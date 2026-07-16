"""Layer 1 — per-cycle CSV export from ConvergenceMonitorNode._monitor_cb.

Choice (per test_share_adaptivity.py): the real unbound _monitor_cb,
_csv_open, _csv_append and _csv_close are bound to a plain stub, with
_call_engine replaced by a fake serving scripted mean_sim readings
in-process. This exercises the exact production export path — open on
init, one flushed row per cycle, error-once disable — with no rclpy init.

The row-content test reads the file while the writer still holds it open,
so it passes only because every row is flushed.
"""

import csv
import json
import time
from datetime import datetime
from types import SimpleNamespace
from unittest import mock

import pytest

from nsk_swarm.convergence_monitor import ConvergenceMonitorNode

NUM_ROBOTS = 3


def make_monitor_stub(csv_path, mean_sims):
    """Monitor over 3 robots; robots 0 and 1 have odom, robot 2 does not.
    mean_sims scripts the engine's reply for successive cycles."""
    stub = SimpleNamespace(
        num_robots=NUM_ROBOTS,
        csv_path=csv_path,
        comm_range=3.0,
        min_rise=0.03,
        stability_eps=0.005,
        merge_counts={},
        similarity_history=[],
        start_time=time.time(),
        robot_positions={0: (1.0, 2.0), 1: (-0.5, 4.25)},
        baseline_sim=None,
        _consec_stable=0,
        _converged=False,
        _sim_cli=SimpleNamespace(srv_name='/nsk/similarity_query'),
        conv_pub=mock.MagicMock(),
        marker_pub=mock.MagicMock(),
        logger=mock.MagicMock())
    stub.get_logger = lambda: stub.logger
    stub._check_duplicate_servers = lambda: None
    stub._make_markers = lambda matrix: []
    stub._log_final_report = lambda *args: None

    readings = iter(mean_sims)

    def fake_call_engine(client, request):
        assert client is stub._sim_cli
        n = len(request.agent_ids)
        matrix = [[1.0] * n for _ in range(n)]
        return SimpleNamespace(success=True, mean_sim=next(readings),
                               matrix_json=json.dumps(matrix))

    stub._call_engine = fake_call_engine
    for name in ('_monitor_cb', '_csv_open', '_csv_append', '_csv_close'):
        setattr(stub, name,
                getattr(ConvergenceMonitorNode, name).__get__(stub))
    stub._csv_open()   # what __init__ does when csv_path is set
    return stub


def test_two_cycles_write_header_plus_two_rows(tmp_path):
    path = tmp_path / 'run.csv'
    stub = make_monitor_stub(str(path), [0.50, 0.62])
    stub.merge_counts = {(0, 1): 2, (1, 2): 1}

    stub._monitor_cb()
    stub._monitor_cb()

    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    header, r1, r2 = rows   # exactly header + 2 rows
    assert header == ['timestamp', 'elapsed_sec', 'mean_sim', 'baseline',
                      'rise', 'converged',
                      'robot0_x', 'robot0_y', 'robot1_x', 'robot1_y',
                      'robot2_x', 'robot2_y', 'pair_counts']
    col = header.index

    datetime.fromisoformat(r1[col('timestamp')])   # ISO or this raises
    assert float(r1[col('mean_sim')]) == pytest.approx(0.50)
    assert float(r2[col('mean_sim')]) == pytest.approx(0.62)
    # Baseline is the first reading; rise follows from it.
    assert float(r1[col('baseline')]) == pytest.approx(0.50)
    assert float(r1[col('rise')]) == pytest.approx(0.0)
    assert float(r2[col('baseline')]) == pytest.approx(0.50)
    assert float(r2[col('rise')]) == pytest.approx(0.12)
    assert r1[col('converged')] == 'False'
    # Odom-fed positions; robot 2 published no odom yet.
    assert float(r1[col('robot0_x')]) == pytest.approx(1.0)
    assert float(r1[col('robot0_y')]) == pytest.approx(2.0)
    assert float(r1[col('robot1_x')]) == pytest.approx(-0.5)
    assert float(r1[col('robot1_y')]) == pytest.approx(4.25)
    assert r1[col('robot2_x')] == ''
    assert r1[col('robot2_y')] == ''
    assert r1[col('pair_counts')] == '0-1:2;1-2:1'
    stub.logger.error.assert_not_called()


def test_default_empty_path_creates_no_file_and_no_error(tmp_path):
    stub = make_monitor_stub('', [0.5, 0.6])

    stub._monitor_cb()
    stub._monitor_cb()

    assert list(tmp_path.iterdir()) == []
    assert stub._csv_file is None and stub._csv_writer is None
    stub.logger.error.assert_not_called()


def test_unwritable_path_logs_once_and_cycles_keep_running(tmp_path):
    # tmp_path itself is a directory: open(..., 'w') fails with an OSError.
    stub = make_monitor_stub(str(tmp_path), [0.5, 0.6])

    stub.logger.error.assert_called_once()
    assert stub._csv_file is None and stub._csv_writer is None

    stub._monitor_cb()
    stub._monitor_cb()

    # Cycles ran to completion without raising and without re-logging.
    assert stub.similarity_history == [0.5, 0.6]
    stub.logger.error.assert_called_once()
