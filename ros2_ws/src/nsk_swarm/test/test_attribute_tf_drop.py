"""Unit tests for experiments/analysis/attribute_tf_drop.py -- pure logic.

The tool answers one question about a dropped scan: was any link in the chain
from the costmap's global_frame down to the scan frame unable to supply a
transform in time? It decides nothing, but R9 consumes its verdict, so the
verdict has to be right for the two reasons it can be wrong: calling a link
late when its transform was on the wire in good time, and calling a run clean
when a link had nothing at all.

Everything here is synthetic. The numbers are shaped like the real ones:
transforms at 20 Hz, scans at 5 Hz, wire times a few tens of milliseconds after
the stamp they carry.

Loaded by location, the convention test_run_health.py and
test_check_run_bag.py use for scripts under experiments/.
"""
import importlib.util
from pathlib import Path

import pytest


def _load():
    for parent in Path(__file__).resolve().parents:
        for cand in (parent / 'attribute_tf_drop.py',
                     parent / 'experiments' / 'analysis'
                     / 'attribute_tf_drop.py'):
            if cand.is_file():
                spec = importlib.util.spec_from_file_location(
                    'attribute_tf_drop', cand)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
    raise ImportError('attribute_tf_drop.py not found above this test')


at = _load()

MAP_ODOM = ('robot_0/map', 'robot_0/odom')
ODOM_BASE = ('robot_0/odom', 'robot_0/base_footprint')
BASE_LINK = ('robot_0/base_footprint', 'robot_0/base_link')
LINK_SCAN = ('robot_0/base_link', 'robot_0/base_scan')
STATIC = {BASE_LINK, LINK_SCAN}

# One scan, its wire time, and the instant the costmap logged the drop: the
# shape every real case has -- the scan reaches the wire, the filter waits its
# transform_tolerance, the line is written.
SCAN_SIM = 100.0
SCAN_WIRE = 1790000000.000
DROP_WALL = SCAN_WIRE + 0.5


def _link(first_sim, first_wire, n=40, step=0.05, wire_step=0.05):
    """A 20 Hz link: stamps from `first_sim`, wire times from `first_wire`."""
    return [(first_sim + i * step, first_wire + i * wire_step)
            for i in range(n)]


# ── first_covering: at or after, with the wire time that goes with it ───────

def test_a_transform_stamped_exactly_at_the_scan_covers_it():
    """The sample the filter needs is the one at the scan's own instant, so
    at-or-after and not strictly-after: an off-by-one here would call every
    on-time run late.
    """
    rows = _link(SCAN_SIM - 0.5, SCAN_WIRE - 0.5)
    stamp, recv = at.first_covering(rows, SCAN_SIM)

    assert stamp == pytest.approx(SCAN_SIM)
    assert recv == pytest.approx(SCAN_WIRE)


def test_a_link_that_never_reaches_the_scan_covers_nothing():
    rows = _link(SCAN_SIM - 2.0, SCAN_WIRE - 2.0, n=10)      # ends at 99.55
    assert at.first_covering(rows, SCAN_SIM) == (None, None)


# ── the two verdicts ─────────────────────────────────────────────────────────

def test_a_covering_transform_that_arrives_after_the_drop_names_that_link():
    """b2maps_k2's shape: the chain is fine except map -> odom, whose stamps
    froze and whose first covering sample reached the wire 171 ms after the
    costmap had given up.
    """
    tf = {
        ODOM_BASE: _link(SCAN_SIM - 1.0, SCAN_WIRE - 1.0),
        # Stalled: nothing stamped at or after the scan until 0.9 s later, and
        # that one lands on the wire after the drop.
        MAP_ODOM: ([(SCAN_SIM - 0.1, SCAN_WIRE - 0.1)]
                   + [(SCAN_SIM + 0.9, DROP_WALL + 0.171)]),
    }
    got = at.attribute_drop([MAP_ODOM, ODOM_BASE], tf, SCAN_SIM, DROP_WALL)

    assert at.late_links(got) == [MAP_ODOM]
    assert got[MAP_ODOM]['late'] is True
    assert got[MAP_ODOM]['missing'] is False
    assert got[MAP_ODOM]['vs_drop_s'] == pytest.approx(0.171)
    assert got[MAP_ODOM]['stamp_lag_ms'] == pytest.approx(900.0)
    assert got[ODOM_BASE]['late'] is False
    # The link that decided it is the one that arrived last.
    assert at.last_to_arrive(got) == MAP_ODOM


def test_every_covering_transform_before_the_drop_is_none_late():
    """b2maps_k3's and k1's shape: nothing was missing. The tool must not
    invent a transform fault, because R9 treats this as advisory and a false
    "late" here would fail a run for load.
    """
    tf = {
        ODOM_BASE: _link(SCAN_SIM - 1.0, SCAN_WIRE - 1.0),
        MAP_ODOM: _link(SCAN_SIM - 1.0, SCAN_WIRE - 1.4, step=0.1),
    }
    got = at.attribute_drop([MAP_ODOM, ODOM_BASE], tf, SCAN_SIM, DROP_WALL)

    assert at.late_links(got) == []
    assert all(not d['late'] and not d['missing'] for d in got.values())
    assert all(d['vs_drop_s'] < 0 for d in got.values())


def test_a_link_with_no_covering_transform_is_late_and_missing():
    """Fail closed. A link that supplied nothing at or after the scan cannot be
    shown to have done its job, and "we did not find one" must not read the
    same as "it was there".
    """
    tf = {ODOM_BASE: _link(SCAN_SIM - 2.0, SCAN_WIRE - 2.0, n=10),
          MAP_ODOM: _link(SCAN_SIM - 1.0, SCAN_WIRE - 1.0)}
    got = at.attribute_drop([MAP_ODOM, ODOM_BASE], tf, SCAN_SIM, DROP_WALL)

    assert at.late_links(got) == [ODOM_BASE]
    assert got[ODOM_BASE]['missing'] is True
    assert got[ODOM_BASE]['recv'] is None
    assert got[ODOM_BASE]['vs_drop_s'] is None


def test_a_link_absent_from_tf_entirely_is_late_too():
    """Not merely a link whose stamps stop short: one that published nothing at
    all in the region. Same verdict, and no KeyError.
    """
    got = at.attribute_drop([MAP_ODOM], {}, SCAN_SIM, DROP_WALL)

    assert at.late_links(got) == [MAP_ODOM]
    assert got[MAP_ODOM]['missing'] is True
    assert at.last_to_arrive(got) is None


# ── the chain, derived rather than listed ────────────────────────────────────

def test_the_chain_walks_up_from_the_scan_frame_to_the_global_frame():
    parent_of = {'robot_0/base_scan': 'robot_0/base_link',
                 'robot_0/base_link': 'robot_0/base_footprint',
                 'robot_0/base_footprint': 'robot_0/odom',
                 'robot_0/odom': 'robot_0/map'}

    chain = at.chain_from(parent_of, 'robot_0/base_scan', 'robot_0/map')
    assert chain == [LINK_SCAN, BASE_LINK, ODOM_BASE, MAP_ODOM]

    # The local costmap stops one link earlier, at odom.
    local = at.chain_from(parent_of, 'robot_0/base_scan', 'robot_0/odom')
    assert local == [LINK_SCAN, BASE_LINK, ODOM_BASE]


def test_a_target_that_is_not_an_ancestor_has_no_chain():
    """A broken or disconnected tree is not a late transform, and must not be
    reported as one.
    """
    parent_of = {'robot_0/base_scan': 'robot_0/base_link'}
    assert at.chain_from(parent_of, 'robot_0/base_scan', 'robot_0/map') is None


def test_a_cycle_terminates_instead_of_hanging():
    parent_of = {'a': 'b', 'b': 'a'}
    assert at.chain_from(parent_of, 'a', 'robot_0/map') is None


def test_the_dynamic_links_are_what_tf_static_does_not_carry():
    """Derived from the bag, not listed here: on these runs base_footprint ->
    base_link -> base_scan are static, so a four-link chain has two dynamic
    links and the local costmap's three-link chain has one.
    """
    chain = [LINK_SCAN, BASE_LINK, ODOM_BASE, MAP_ODOM]
    assert at.dynamic_links(chain, STATIC) == [ODOM_BASE, MAP_ODOM]
    assert at.dynamic_links([LINK_SCAN, BASE_LINK], STATIC) == []


# ── which costmap logged it, and where its global_frame is set ──────────────

def test_the_costmap_is_read_off_the_logger():
    assert at.costmap_of(
        "[controller_server-3] [INFO] [robot_3.local_costmap.local_costmap]: "
        'Message Filter dropping message') == 'local'
    assert at.costmap_of(
        '[planner_server-5] [INFO] [robot_3.global_costmap.global_costmap]: '
        'Message Filter dropping message') == 'global'
    assert at.costmap_of(
        '[collision_monitor-11] [ERROR] [getTransform]: Failed to get') is None


def test_the_params_line_comes_from_inside_the_right_block(tmp_path):
    """nav2_robot2.yaml sets `global_frame: robot_2/map` at :26 for
    bt_navigator and again at :222 for the global costmap. A file-wide search
    returns the first, which is a real line documenting the wrong node -- so
    the search is scoped to the block.
    """
    p = tmp_path / 'nav2_robotX.yaml'
    p.write_text(
        'bt_navigator:\n'
        '  ros__parameters:\n'
        '    global_frame: robot_2/map\n'          # line 3, the decoy
        'local_costmap:\n'
        '  local_costmap:\n'
        '    ros__parameters:\n'
        '      global_frame: robot_2/odom\n'       # line 7
        '      transform_tolerance: 0.5\n'         # line 8
        'global_costmap:\n'
        '  global_costmap:\n'
        '    ros__parameters:\n'
        '      global_frame: robot_2/map\n'        # line 12
        '      transform_tolerance: 0.5\n')        # line 13

    facts = at.costmap_facts(p)
    assert facts['local'] == {'global_frame': 'robot_2/odom',
                              'transform_tolerance': 0.5,
                              'frame_line': 7, 'tol_line': 8}
    assert facts['global']['global_frame'] == 'robot_2/map'
    assert facts['global']['frame_line'] == 12      # not 3
    assert facts['global']['tol_line'] == 13


def test_a_missing_params_file_is_fatal_rather_than_defaulted(tmp_path):
    with pytest.raises(SystemExit):
        at.costmap_facts(tmp_path / 'does_not_exist.yaml')


def test_the_default_params_path_is_the_launch_files_own(tmp_path):
    """explore.launch.py:128 hands nav2 experiments/nav/nav2_robotK.yaml unless
    nav2_params_file is passed, which the b2maps procedure does not do.
    """
    assert at.nav2_params_path(3).name == 'nav2_robot3.yaml'
    assert at.nav2_params_path(3).parent.name == 'nav'
    assert at.nav2_params_path(3).is_file()


# ── the neighbour baseline is a measurement, not a bound ────────────────────

def test_the_baseline_measures_each_scan_against_its_own_wire_time():
    """Against the DROP's wall time, a later scan would look progressively
    worse for no reason but being later. Against its own wire time the figure
    is the same quantity for every scan, which is what makes the dropped one
    comparable to its neighbours.
    """
    tf = {ODOM_BASE: _link(SCAN_SIM - 1.0, SCAN_WIRE - 1.0, n=200)}
    scans = [(SCAN_SIM + 0.2 * i, SCAN_WIRE + 0.2 * i) for i in range(-5, 6)]

    base = at.neighbour_baseline([ODOM_BASE], tf, scans, idx=5)
    assert len(base[ODOM_BASE]) == 10
    # Every scan sees the same offset, so the spread is flat.
    assert max(base[ODOM_BASE]) - min(base[ODOM_BASE]) == pytest.approx(
        0.0, abs=1e-9)

    own = at.covering_vs_scan([ODOM_BASE], tf, SCAN_SIM, SCAN_WIRE)
    assert own[ODOM_BASE] == pytest.approx(base[ODOM_BASE][0])
