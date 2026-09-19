"""Unit tests for check_cut_reference.py -- pure logic only, no ROS, no bags.

Finds the module by walking up from this file to experiments/slam/, so it runs
from experiments/slam/ or from ros2_ws/src/nsk_swarm/test/ alike.
"""
import importlib.util
from pathlib import Path

import pytest


def _load():
    for parent in Path(__file__).resolve().parents:
        for cand in (parent / "check_cut_reference.py",
                     parent / "experiments" / "slam" / "check_cut_reference.py"):
            if cand.is_file():
                spec = importlib.util.spec_from_file_location("check_cut_reference", cand)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
    raise ImportError("check_cut_reference.py not found above this test")


ccr = _load()


def test_keep_set_has_13_topics():
    assert len(ccr.KEEP) == 13
    assert len(set(ccr.KEEP)) == 13
    assert "/model/robot_0/pose" not in ccr.KEEP


def test_first_read_wins_not_smallest_timestamp():
    msgs = [("/clock", 200), ("/tf", 100), ("/tf_static", 300)]
    t0, topic0, _ = ccr.first_kept(msgs, {"/clock", "/tf", "/tf_static"})
    assert (t0, topic0) == (200, "/clock")


def test_unkept_topics_are_skipped():
    msgs = [("/model/robot_0/pose", 50), ("/robot_0/cmd_vel", 60), ("/clock", 70)]
    t0, topic0, first = ccr.first_kept(msgs, {"/clock"})
    assert (t0, topic0) == (70, "/clock")
    assert first == {"/clock": 70}


def test_stops_once_every_kept_topic_is_seen():
    def gen():
        yield ("/clock", 1)
        yield ("/tf", 2)
        raise AssertionError("read past the point where all kept topics were seen")

    t0, _, first = ccr.first_kept(gen(), {"/clock", "/tf"})
    assert t0 == 1 and first == {"/clock": 1, "/tf": 2}


def test_no_kept_messages_gives_none():
    t0, topic0, first = ccr.first_kept([("/other", 5)], {"/clock"})
    assert t0 is None and topic0 is None and first == {}


def _r(t0, present=("/clock", "/tf"), topic0="/clock"):
    return {"t0": t0, "topic0": topic0, "present": set(present)}


def test_verdict_passes_on_identical_zero():
    ok, reasons = ccr.verdict(_r(1_000), _r(1_000))
    assert ok and reasons == []


def test_verdict_fails_on_one_nanosecond_by_default():
    ok, reasons = ccr.verdict(_r(1_000), _r(1_001))
    assert not ok and "differs by 1 ns" in reasons[0]


def test_verdict_tolerance_is_inclusive():
    assert ccr.verdict(_r(0), _r(5), tol_ns=5)[0]
    assert not ccr.verdict(_r(0), _r(6), tol_ns=5)[0]


def test_verdict_fails_when_kept_sets_differ():
    ok, reasons = ccr.verdict(_r(0), _r(0, present=("/clock",)))
    assert not ok and "kept-topic sets differ" in reasons[0]


def test_verdict_fails_when_a_bag_has_no_kept_messages():
    ok, reasons = ccr.verdict(_r(None), _r(0))
    assert not ok and "no kept messages" in reasons[0]


def test_read_metadata(tmp_path):
    (tmp_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  storage_identifier: mcap\n"
        "  starting_time:\n"
        "    nanoseconds_since_epoch: 123\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata: {name: /clock}\n"
        "      message_count: 7\n"
    )
    sid, start, counts = ccr.read_metadata(tmp_path)
    assert (sid, start, counts) == ("mcap", 123, {"/clock": 7})


def test_read_metadata_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        ccr.read_metadata(tmp_path)
