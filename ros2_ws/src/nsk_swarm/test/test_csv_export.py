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
from nav_msgs.msg import Odometry

from nsk_swarm.convergence_monitor import ConvergenceMonitorNode

NUM_ROBOTS = 3


def make_monitor_stub(csv_path, mean_sims, spawn_xs=(), spawn_ys=()):
    """Monitor over 3 robots; robots 0 and 1 have odom, robot 2 does not.
    mean_sims scripts the engine's reply for successive cycles."""
    stub = SimpleNamespace(
        num_robots=NUM_ROBOTS,
        csv_path=csv_path,
        comm_range=3.0,
        spawn_xs=list(spawn_xs),
        spawn_ys=list(spawn_ys),
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
    for name in ('_monitor_cb', '_csv_open', '_csv_append', '_csv_close',
                 '_odom_cb'):
        setattr(stub, name,
                getattr(ConvergenceMonitorNode, name).__get__(stub))
    stub._csv_open()   # what __init__ does when csv_path is set
    return stub


def odom_msg(x: float, y: float) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    return msg


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


def test_spawn_offsets_applied_to_csv_positions(tmp_path):
    path = tmp_path / 'run.csv'
    stub = make_monitor_stub(str(path), [0.5],
                             spawn_xs=[4.0, 1.2, -3.2],
                             spawn_ys=[0.0, 3.8, 2.4])
    # Positions arrive through the real odom callback (which applies the
    # spawn offset), not the factory's preset dict.
    stub.robot_positions = {}
    stub._odom_cb(odom_msg(1.0, 2.0), 0)
    stub._odom_cb(odom_msg(-0.5, 0.45), 1)

    stub._monitor_cb()

    with open(path, newline='') as f:
        header, row = list(csv.reader(f))
    col = header.index
    assert float(row[col('robot0_x')]) == pytest.approx(5.0)
    assert float(row[col('robot0_y')]) == pytest.approx(2.0)
    assert float(row[col('robot1_x')]) == pytest.approx(0.7)
    assert float(row[col('robot1_y')]) == pytest.approx(4.25)
    # Robot 2 published no odom: its columns stay blank, offset or not.
    assert row[col('robot2_x')] == ''
    assert row[col('robot2_y')] == ''
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


# ── Regression: rows keep writing regardless of spawn-offset config ─────────
#
# Symptom this guards against: after the spawn_xs/spawn_ys offset patch, a
# live run wrote the CSV header and then zero data rows for the whole run,
# with no CSV-related log line at all — i.e. _csv_append's silent
# `if self._csv_writer is None: return` guard was firing every cycle despite
# _csv_open() having succeeded. Root cause: spawn_xs/spawn_ys were declared
# via a type-only `Parameter.Type.DOUBLE_ARRAY` with no default value, so on
# any launch that leaves them unset (exactly the "unset = zero offsets"
# backward-compatible case the code claims to support), get_parameter(...)
# .value raises rclpy.exceptions.ParameterUninitializedException instead of
# returning None — see test_spawn_array_param_unset_does_not_raise below for
# the direct regression test on that declaration. These two tests instead
# confirm the *_monitor_cb -> _csv_append* path itself writes one row per
# cycle end-to-end, with spawn offsets both configured and left at their
# backward-compatible default.

def test_rows_written_each_cycle_with_spawn_offsets_set(tmp_path):
    path = tmp_path / 'run.csv'
    stub = make_monitor_stub(str(path), [0.40, 0.50, 0.60],
                             spawn_xs=[4.0, 1.2, -3.2],
                             spawn_ys=[0.0, 3.8, 2.4])
    stub.robot_positions = {}
    stub._odom_cb(odom_msg(0.0, 0.0), 0)
    stub._odom_cb(odom_msg(0.0, 0.0), 1)

    for _ in range(3):
        stub._monitor_cb()

    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    assert len(rows) - 1 == 3   # header + one row per cycle, none dropped
    stub.logger.warn.assert_not_called()
    stub.logger.error.assert_not_called()


def test_rows_written_each_cycle_with_spawn_offsets_unset(tmp_path):
    # Backward-compatible dot-era usage: spawn_xs/spawn_ys never configured
    # at all (make_monitor_stub's () default -> []), so odom passes through
    # unshifted — this is the exact configuration that used to crash node
    # construction before the declare_parameter fix.
    path = tmp_path / 'run.csv'
    stub = make_monitor_stub(str(path), [0.40, 0.50, 0.60])
    stub.robot_positions = {}
    stub._odom_cb(odom_msg(1.5, -2.0), 0)
    stub._odom_cb(odom_msg(0.25, 0.75), 1)

    for _ in range(3):
        stub._monitor_cb()

    with open(path, newline='') as f:
        rows = list(csv.reader(f))
    assert len(rows) - 1 == 3
    col = rows[0].index
    assert float(rows[1][col('robot0_x')]) == pytest.approx(1.5)
    assert float(rows[1][col('robot0_y')]) == pytest.approx(-2.0)
    stub.logger.warn.assert_not_called()
    stub.logger.error.assert_not_called()


def test_forced_write_failure_logs_once_and_disables_cleanly(tmp_path):
    """The failure policy for a row-write exception: log exactly one
    warning with the exception text, then disable — never silently, and
    never raising into _monitor_cb."""
    path = tmp_path / 'run.csv'
    stub = make_monitor_stub(str(path), [0.5, 0.6, 0.7])

    # Force every subsequent writerow() to fail, simulating e.g. a full
    # disk. _csv_file is left as the real handle _csv_open() opened —
    # _csv_close() must still be able to close it cleanly.
    stub._csv_writer = mock.MagicMock()
    stub._csv_writer.writerow.side_effect = OSError('disk full')

    stub._monitor_cb()   # row-write raises internally; must not propagate

    stub.logger.warn.assert_called_once()
    (warn_msg,), _ = stub.logger.warn.call_args
    assert 'disk full' in warn_msg
    assert stub._csv_writer is None and stub._csv_file is None
    stub.logger.error.assert_not_called()

    # Export is cleanly disabled: later cycles run to completion and don't
    # raise or re-log.
    stub._monitor_cb()
    stub._monitor_cb()
    assert stub.similarity_history == [0.5, 0.6, 0.7]
    stub.logger.warn.assert_called_once()


# ── Regression: the real declare_parameter path (not the stub) ──────────────
#
# The stub-based tests above bind the real _csv_*/_monitor_cb methods to a
# plain object, bypassing declare_parameter/get_parameter entirely — they
# cannot catch a regression in the parameter *declaration* itself. This pair
# constructs a throwaway plain Node (not the full ConvergenceMonitorNode,
# whose __init__ blocks in wait_for_service for the engine — see
# test_geometry.py) and issues the exact declare_parameter('spawn_xs',
# [0.0]) call convergence_monitor.py makes, to pin down the actual root
# cause: a bare [] default is mis-inferred as BYTE_ARRAY, and the type-only
# Parameter.Type.DOUBLE_ARRAY declaration this replaced has no fallback
# value, so get_parameter().value raises ParameterUninitializedException
# instead of returning None whenever spawn_xs/spawn_ys are left unset.

def test_spawn_array_param_unset_does_not_raise(rclpy_context):
    from rclpy.node import Node
    node = Node('csv_export_param_regression_test_unset')
    try:
        node.declare_parameter('spawn_xs', [0.0])
        node.declare_parameter('spawn_ys', [0.0])
        # Must not raise ParameterUninitializedException, and the readout
        # logic (list(...value or [])) must land on an all-zero-offset
        # fallback usable for every robot id.
        assert list(node.get_parameter('spawn_xs').value or []) == [0.0]
        assert list(node.get_parameter('spawn_ys').value or []) == [0.0]
    finally:
        node.destroy_node()


def test_spawn_array_param_override_reads_back_correctly(rclpy_context):
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    overrides = [
        Parameter('spawn_xs', Parameter.Type.DOUBLE_ARRAY, [4.0, 1.2, -3.2]),
        Parameter('spawn_ys', Parameter.Type.DOUBLE_ARRAY, [0.0, 3.8, 2.4]),
    ]
    node = Node('csv_export_param_regression_test_override',
               parameter_overrides=overrides)
    try:
        node.declare_parameter('spawn_xs', [0.0])
        node.declare_parameter('spawn_ys', [0.0])
        assert list(node.get_parameter('spawn_xs').value) == [4.0, 1.2, -3.2]
        assert list(node.get_parameter('spawn_ys').value) == [0.0, 3.8, 2.4]
    finally:
        node.destroy_node()
