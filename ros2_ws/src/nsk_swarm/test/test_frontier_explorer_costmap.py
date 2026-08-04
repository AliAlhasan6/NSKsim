"""Layer 1 — the costmap the explorer plans against but could not see.

rung2g halted with all 12 frontier candidates unplannable while a 0.5 m start
probe planned successfully from the same pose. Every occupancy number in that
log came from the SLAM map; every planner verdict came from navfn reading the
GLOBAL COSTMAP — a different grid, with a different origin, a different
resolution and an inflation layer in only one of them. Reconstructing the
costmap offline from the saved PGM could not reproduce what the planner saw,
so the two could not be compared and the run's own evidence did not settle it.

These tests cover the subscription that ends that: what a cost reads as, what
happens off the edge of the grid, what happens when no costmap ever arrives,
and — the one that motivated the whole shape of the code — that a costmap
whose geometry differs from the SLAM map's is read with ITS OWN geometry
rather than the map's.

Everything here is diagnostic. The last section pins that: selection reaches
the same verdicts with a costmap, without one, and with one that says the
robot is standing in a wall.

Per the stub pattern in test_frontier_explorer_precheck.py, the real unbound
methods are bound to a plain SimpleNamespace — no rclpy.init, no node, no
BasicNavigator.
"""

import math
import os
import threading
from types import SimpleNamespace

import pytest

# Same guard, and the same reasoning, as test_frontier_explorer_precheck.py:
# FrontierExplorer subclasses BasicNavigator, so the nav2_simple_commander
# import is module-level and cannot be deferred. CI sets NSK_REQUIRE_NAV2=1 and
# installs Nav2 deliberately, so a miss there is fatal rather than skipped.
try:
    from nsk_swarm import frontier_explorer as fx
    from nsk_swarm import reachability
    from nsk_swarm.frontier_explorer import (DEFER, OFF_COSTMAP,
                                             FrontierExplorer, _CostGrid,
                                             _SensorNode, cost_name)
except ImportError:
    if os.environ.get('NSK_REQUIRE_NAV2', '') not in ('', '0', 'false'):
        raise
    fx = FrontierExplorer = _CostGrid = _SensorNode = None
    DEFER = OFF_COSTMAP = cost_name = reachability = None

pytestmark = pytest.mark.skipif(
    fx is None,
    reason='nav2_simple_commander is not importable — install '
           'ros-jazzy-navigation2 to run the costmap diagnostic tests')


# ── message stand-ins ───────────────────────────────────────────────────────
#
# Two constructors exist because the two messages genuinely differ: nav2
# publishes the visualisation grid as nav_msgs/OccupancyGrid (info.width,
# info.height, int8 data on the translated -1..100 scale) and the costs navfn
# reads as nav2_msgs/Costmap (metadata.size_x, metadata.size_y, uint8 data on
# the raw 0..255 scale). A stub that flattened them would test a shape neither
# publisher sends.

def occupancy_msg(w, h, res=0.05, ox=0.0, oy=0.0, fill=0):
    return SimpleNamespace(
        info=SimpleNamespace(
            width=w, height=h, resolution=res,
            origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy))),
        data=[fill] * (w * h))


def costmap_msg(w, h, res=0.05, ox=0.0, oy=0.0, fill=0):
    return SimpleNamespace(
        metadata=SimpleNamespace(
            size_x=w, size_y=h, resolution=res,
            origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy))),
        data=[fill] * (w * h))


def grid_of(msg, topic=None, stamp=0.0, seq=1):
    """Snapshot `msg` through whichever constructor its shape calls for."""
    if hasattr(msg, 'metadata'):
        return _CostGrid.from_costmap(
            msg, topic or f'/robot_0/{fx.COSTMAP_RAW_TOPIC}', stamp, seq)
    return _CostGrid.from_occupancy_grid(
        msg, topic or f'/robot_0/{fx.COSTMAP_TOPIC}', stamp, seq)


def at(grid, x, y, value):
    """Write `value` into the cell containing world (x, y)."""
    cx, cy = grid.cell_of(x, y)
    grid.data[cy * grid.width + cx] = value
    return grid


# ── the cost lookup, and the int8 wrap ──────────────────────────────────────

def test_the_four_named_costs_and_the_unnamed_band():
    assert cost_name(0) == 'FREE'
    assert cost_name(253) == 'INSCRIBED_INFLATED'
    assert cost_name(254) == 'LETHAL'
    assert cost_name(255) == 'NO_INFORMATION'
    # nav2 names no value in 1..252 — it is the inflation gradient, and no
    # decision anywhere reads it. Naming one would invent a semantic.
    for cost in (1, 99, 100, 128, 252):
        assert cost_name(cost) == 'COST'


@pytest.mark.parametrize('raw,cost,name', [
    (-3, 253, 'INSCRIBED_INFLATED'),
    (-2, 254, 'LETHAL'),
    (-1, 255, 'NO_INFORMATION'),
])
def test_a_negative_int8_decodes_to_the_cost_nav2_meant(raw, cost, name):
    # THE conversion this file exists for. nav2's top three costs do not fit in
    # a signed byte, so on an int8 grid they arrive negative. Unguarded, LETHAL
    # reads as -2 (a small free-ish number) and NO_INFORMATION as -1 with
    # nothing to distinguish it from a cost — the readout would then say the
    # planner saw open space exactly where it saw a wall.
    grid = grid_of(occupancy_msg(4, 4, fill=raw))
    reading = grid.cost_at(0.02, 0.02)

    assert reading.cost == cost
    assert reading.name == name
    # The wire value survives the decode: when the two grids disagree, what was
    # actually on the wire is the thing to look at.
    assert reading.raw == raw


def test_an_unsigned_costmap_needs_no_unwrapping():
    # nav2_msgs/Costmap data is uint8, so 254 arrives as 254 and must pass
    # through untouched — the same code path, and it may not "correct" it.
    grid = grid_of(costmap_msg(4, 4, fill=254))
    reading = grid.cost_at(0.02, 0.02)

    assert (reading.cost, reading.name, reading.raw) == (254, 'LETHAL', 254)


def test_zero_and_the_gradient_are_read_as_published():
    grid = grid_of(occupancy_msg(4, 4, fill=0))
    assert grid.cost_at(0.02, 0.02) == (0, 'FREE', 0)
    grid = grid_of(occupancy_msg(4, 4, fill=98))
    assert grid.cost_at(0.02, 0.02) == (98, 'COST', 98)


def test_known_values_at_known_world_coordinates():
    # The end-to-end shape of the reviewer's own check: a synthetic grid with a
    # LETHAL cell and an INSCRIBED cell at coordinates chosen in metres, read
    # back by world coordinate rather than by index.
    msg = occupancy_msg(40, 40, res=0.05, ox=-1.0, oy=-1.0)
    grid = grid_of(msg)
    at(grid, 0.30, 0.42, -2)        # 254 LETHAL
    at(grid, -0.55, 0.10, -3)       # 253 INSCRIBED_INFLATED

    assert grid.cost_at(0.30, 0.42) == (254, 'LETHAL', -2)
    assert grid.cost_at(-0.55, 0.10) == (253, 'INSCRIBED_INFLATED', -3)
    # ...and a cell away from either is untouched free space, so the writes
    # landed where the world coordinates said and not somewhere convenient.
    assert grid.cost_at(0.30, 0.20).cost == 0
    assert grid.cost_at(0.35, 0.42).cost == 0


def test_a_cell_boundary_belongs_to_the_cell_it_starts():
    # res 0.05 from origin 0.0: [0.05, 0.10) is cell 1 in each axis.
    grid = grid_of(occupancy_msg(4, 4, res=0.05, ox=0.0, oy=0.0))
    assert grid.cell_of(0.0, 0.0) == (0, 0)
    assert grid.cell_of(0.0499, 0.0499) == (0, 0)
    assert grid.cell_of(0.05, 0.05) == (1, 1)
    assert grid.cell_of(0.1999, 0.1999) == (3, 3)


# ── off the edge of the grid ────────────────────────────────────────────────

def edged_grid():
    """4x4 over [0.0, 0.2) with every border cell marked LETHAL."""
    grid = grid_of(occupancy_msg(4, 4, res=0.05, ox=0.0, oy=0.0))
    for i in range(4):
        for cx, cy in ((i, 0), (i, 3), (0, i), (3, i)):
            grid.data[cy * grid.width + cx] = -2      # 254
    return grid


@pytest.mark.parametrize('x,y', [
    (0.25, 0.10),     # past +x
    (0.10, 0.25),     # past +y
    (-0.01, 0.10),    # left of the origin
    (0.10, -0.01),    # below the origin
    (-5.0, -5.0),     # nowhere near it
])
def test_a_point_outside_the_grid_returns_the_sentinel(x, y):
    # Never a raise (a diagnostic may not end a run) and never a clamp: an edge
    # cell substituted for a point the grid does not cover would read as an
    # ordinary cost and hide the finding — a pose the planner's own grid does
    # not even reach is itself the answer.
    reading = edged_grid().cost_at(x, y)

    assert reading.cost is OFF_COSTMAP
    assert reading.raw is OFF_COSTMAP
    assert reading.name == 'off-costmap'


def test_a_point_left_of_the_origin_is_not_folded_into_cell_zero():
    # int() truncates toward zero, so int((-0.01 - 0.0) / 0.05) is 0 and a
    # point outside the grid would read as cell 0. floor() is what keeps that
    # from being reported as a real cost.
    grid = edged_grid()
    assert grid.cell_of(-0.01, 0.10)[0] < 0
    assert grid.cost_at(-0.01, 0.10).cost is OFF_COSTMAP
    # Cell 0 itself is readable and LETHAL — so the sentinel above is not just
    # "everything reads off-grid".
    assert grid.cost_at(0.02, 0.10).cost == 254


def test_the_sentinel_reprs_as_something_readable_in_a_log():
    assert repr(OFF_COSTMAP) == 'off-costmap'


# ── partial updates ─────────────────────────────────────────────────────────

def test_an_in_window_update_lands_in_the_cached_grid():
    grid = grid_of(occupancy_msg(6, 6, res=0.05, ox=0.0, oy=0.0))
    # 2x2 window at cell (2, 2) -> world [0.10, 0.20)
    assert grid.apply_update(2, 2, 2, 2, [-2, -2, -2, -2], stamp=9.0) is True

    assert grid.cost_at(0.11, 0.11) == (254, 'LETHAL', -2)
    assert grid.cost_at(0.16, 0.16) == (254, 'LETHAL', -2)
    assert grid.cost_at(0.06, 0.06).cost == 0      # outside the window
    assert (grid.updates, grid.rejected) == (1, 0)
    assert grid.stamp == 9.0                       # receipt time moves with it


def test_an_update_that_runs_off_the_grid_is_rejected_and_counted():
    # This is what arrives when the costmap has been RESIZED and the cached
    # snapshot is the wrong shape. Dropping it leaves a region stale, so it is
    # counted — the readout prints the count — rather than silently absorbed.
    grid = grid_of(occupancy_msg(6, 6))
    before = list(grid.data)

    assert grid.apply_update(5, 5, 3, 3, [-2] * 9, stamp=9.0) is False
    assert grid.apply_update(-1, 0, 2, 2, [-2] * 4, stamp=9.0) is False

    assert grid.data == before
    assert (grid.updates, grid.rejected) == (0, 2)


def test_an_update_with_too_little_data_is_rejected():
    grid = grid_of(occupancy_msg(6, 6))
    before = list(grid.data)

    assert grid.apply_update(0, 0, 3, 3, [-2] * 4, stamp=9.0) is False

    assert grid.data == before
    assert grid.rejected == 1


def test_an_empty_window_is_a_no_op_not_a_rejection():
    grid = grid_of(occupancy_msg(6, 6))
    assert grid.apply_update(0, 0, 0, 0, [], stamp=9.0) is False
    assert (grid.updates, grid.rejected) == (0, 0)


def sensor_stub(costmap=None, raw=None):
    """The costmap intake of _SensorNode, without a node under it."""
    stub = SimpleNamespace(_cm_lock=threading.Lock(), _costmap=costmap,
                           _costmap_raw=raw, _costmap_orphans=0,
                           _orphan_logged=False, logged=[])
    stub.get_logger = lambda: SimpleNamespace(
        warn=lambda m: stub.logged.append(m))
    for name in ('_apply_update', '_on_costmap_update',
                 '_on_costmap_raw_update', 'get_costmaps'):
        setattr(stub, name, getattr(_SensorNode, name).__get__(stub))
    return stub


def test_each_update_message_is_unpacked_with_its_own_field_names():
    # map_msgs/OccupancyGridUpdate names the window width/height;
    # nav2_msgs/CostmapUpdate names it size_x/size_y. Duck-typing the two would
    # silently drop one of them.
    cm = grid_of(occupancy_msg(6, 6))
    raw = grid_of(costmap_msg(6, 6, fill=0))
    stub = sensor_stub(cm, raw)

    stub._on_costmap_update(SimpleNamespace(x=1, y=1, width=2, height=2,
                                            data=[-2] * 4))
    stub._on_costmap_raw_update(SimpleNamespace(x=1, y=1, size_x=2, size_y=2,
                                                data=[254] * 4))

    assert cm.cost_at(0.06, 0.06) == (254, 'LETHAL', -2)
    assert raw.cost_at(0.06, 0.06) == (254, 'LETHAL', 254)
    assert (cm.updates, raw.updates) == (1, 1)


def test_an_update_before_any_full_costmap_says_so_once():
    # Silence here would look exactly like a costmap full of free space.
    stub = sensor_stub()
    for _ in range(5):
        stub._on_costmap_update(SimpleNamespace(x=0, y=0, width=1, height=1,
                                                data=[-2]))

    assert stub._costmap_orphans == 5
    assert len(stub.logged) == 1
    assert 'before any full costmap' in stub.logged[0]


def test_get_costmaps_hands_back_both_grids():
    cm, raw = grid_of(occupancy_msg(2, 2)), grid_of(costmap_msg(2, 2))
    assert sensor_stub(cm, raw).get_costmaps() == (cm, raw)
    assert sensor_stub().get_costmaps() == (None, None)


# ── the explorer's readouts ─────────────────────────────────────────────────

def explorer_stub(costmaps=(None, None), raiser=False):
    stub = SimpleNamespace(robot_id=0, _costmap_missing_warned=False,
                           logged=[])
    stub.info = lambda m: stub.logged.append(('INFO', m))
    stub.warn = lambda m: stub.logged.append(('WARN', m))
    stub.error = lambda m: stub.logged.append(('ERROR', m))

    def get_costmaps():
        if raiser:
            raise RuntimeError('sensor is gone')
        return costmaps

    stub.sensor = SimpleNamespace(get_costmaps=get_costmaps)
    stub._cost_reading_note = FrontierExplorer._cost_reading_note
    for name in ('_costmaps_or_warn', '_cost_note', '_costmap_pose_readout',
                 '_pose_occupancy_readout', '_halt_verdict_note'):
        setattr(stub, name, getattr(FrontierExplorer, name).__get__(stub))
    return stub


def warns(stub):
    return [m for lvl, m in stub.logged if lvl == 'WARN']


def test_a_cost_note_names_both_grids_at_one_point():
    cm = at(grid_of(occupancy_msg(20, 20, ox=-0.5, oy=-0.5)), 0.1, 0.1, 100)
    raw = at(grid_of(costmap_msg(20, 20, ox=-0.5, oy=-0.5)), 0.1, 0.1, 254)
    stub = explorer_stub((cm, raw))

    note = stub._cost_note(cm, raw, 0.1, 0.1)

    # The translated number beside the raw one that produced it: nav2 maps 254
    # onto 100 for the visualisation grid, so '100' alone would read as a
    # mid-inflation cost and '254' alone would not match what rviz shows.
    assert note == 'costmap=100 COST raw=254 LETHAL'


def test_the_wire_value_is_printed_only_where_it_differs_from_the_cost():
    cm = at(grid_of(occupancy_msg(8, 8)), 0.1, 0.1, -2)
    stub = explorer_stub((cm, None))

    assert 'costmap=254 LETHAL int8=-2' in stub._cost_note(cm, None, 0.1, 0.1)
    assert 'int8=' not in stub._cost_note(cm, None, 0.05, 0.05)   # a 0 cell


def test_a_point_off_the_costmap_says_so_in_the_note():
    cm = grid_of(occupancy_msg(8, 8, ox=0.0, oy=0.0))
    stub = explorer_stub((cm, None))
    assert stub._cost_note(cm, None, 99.0, 99.0) == \
        'costmap=off-costmap raw=none-received'


# ── degradation when no costmap ever arrives ────────────────────────────────

def test_a_missing_costmap_warns_exactly_once_across_every_call_site():
    # Once — a per-candidate warning would flood the log this exists to make
    # readable. But never zero: a silent costmap readout is indistinguishable
    # from a costmap that says everything is free, which is the wrong-file /
    # empty-grep failure this change exists to prevent.
    stub = explorer_stub((None, None))
    for i in range(4):
        stub._costmaps_or_warn(f'precheck cand#{i}')
    stub._costmaps_or_warn('start probe 1/3')
    stub._costmaps_or_warn('termination readout')

    assert len(warns(stub)) == 1
    msg = warns(stub)[0]
    # It names the point that first asked, both dead topics, and what to check.
    assert 'precheck cand#0' in msg
    assert fx.COSTMAP_TOPIC in msg and fx.COSTMAP_RAW_TOPIC in msg
    assert 'transient-local' in msg


def test_only_the_missing_topic_is_named():
    cm = grid_of(occupancy_msg(4, 4))
    stub = explorer_stub((cm, None))
    stub._costmaps_or_warn('precheck cand#0')

    msg = warns(stub)[0]
    assert fx.COSTMAP_RAW_TOPIC in msg
    assert f'on {fx.COSTMAP_TOPIC} ' not in msg


def test_both_grids_present_warns_about_nothing():
    stub = explorer_stub((grid_of(occupancy_msg(4, 4)),
                          grid_of(costmap_msg(4, 4))))
    stub._costmaps_or_warn('precheck cand#0')
    assert warns(stub) == []


def test_a_sensor_that_raises_is_reported_not_propagated():
    # A diagnostic may never end a run differently, and may never go quiet
    # about failing either.
    stub = explorer_stub(raiser=True)

    assert stub._costmaps_or_warn('precheck cand#0') == (None, None)
    assert len(warns(stub)) == 1
    assert 'could not read the costmap snapshots' in warns(stub)[0]
    assert 'RuntimeError' in warns(stub)[0]
    # Still once, however many times it fails.
    stub._costmaps_or_warn('precheck cand#1')
    assert len(warns(stub)) == 1


def test_a_degraded_note_still_produces_a_greppable_line():
    stub = explorer_stub((None, None))
    assert stub._cost_note(None, None, 1.0, 2.0) == \
        'costmap=none-received raw=none-received'
    assert stub._costmap_pose_readout(None, 'raw', 1.0, 2.0, None) == \
        'costmap[raw] none received'


# ── the termination readout: two grids, two geometries ──────────────────────

def slam_grid(w, h, res=0.05, ox=0.0, oy=0.0, fill=0):
    """A SLAM OccupancyGrid stand-in for _pose_occupancy_readout."""
    return SimpleNamespace(
        info=SimpleNamespace(
            width=w, height=h, resolution=res,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=ox, y=oy),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[fill] * (w * h))


def test_the_readout_keeps_the_slam_grid_and_adds_the_costmaps():
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)
    cx, cy = 21, 24                      # (0.06, 0.21) with res 0.05
    g.data[cy * 40 + cx] = 100           # occupied under the robot
    cm = at(grid_of(occupancy_msg(40, 40, ox=-1.0, oy=-1.0)), 0.06, 0.21, -2)
    raw = at(grid_of(costmap_msg(40, 40, ox=-1.0, oy=-1.0)), 0.06, 0.21, 254)
    stub = explorer_stub((cm, raw))

    out = stub._pose_occupancy_readout(g, 7, 0.06, 0.21)

    # The pre-existing SLAM readout is untouched — it is half of the pair.
    assert f'cell ({cx}, {cy}) value=100' in out
    assert '3x3 north-row-first [0,0,0] [0,100,0] [0,0,0]' in out
    assert 'nearest occupied 0.0' in out
    assert 'map seq 7' in out
    # ...and each costmap now says the same three things for itself.
    assert 'costmap[translated]' in out and 'costmap[raw]' in out
    assert f'cell ({cx}, {cy}) cost=254 LETHAL' in out
    assert '3x3 north-row-first [0,0,0] [0,254,0] [0,0,0]' in out
    assert out.count('at nearest SLAM-occupied cost=254 LETHAL') == 2


def test_divergence_between_the_grids_is_what_the_pair_shows():
    # THE finding this change exists to make visible: the SLAM map says the
    # robot is standing in open space and the costmap says it is inside the
    # inflated envelope. Neither grid alone says that; the pair does.
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)          # free everywhere
    cm = at(grid_of(occupancy_msg(40, 40, ox=-1.0, oy=-1.0)), 0.06, 0.21, -3)
    stub = explorer_stub((cm, None))

    out = stub._pose_occupancy_readout(g, 7, 0.06, 0.21)

    assert 'value=0' in out                          # SLAM: free
    assert 'cost=253 INSCRIBED_INFLATED' in out      # costmap: unplannable
    assert 'costmap[raw] none received' in out


def test_each_grid_reports_its_own_cell_under_a_different_geometry():
    # The costmap is published by a different node with its own origin,
    # resolution and size. Reusing the SLAM map's cell index would read a cell
    # metres away from the pose and quietly report the wrong evidence.
    g = slam_grid(40, 40, res=0.05, ox=-1.0, oy=-1.0)
    cm = grid_of(occupancy_msg(40, 40, res=0.10, ox=-2.0, oy=-3.0))
    raw = grid_of(costmap_msg(60, 60, res=0.025, ox=0.0, oy=0.0))
    px, py = 0.33, 0.44                  # mid-cell in all three resolutions
    at(cm, px, py, -2)
    at(raw, px, py, 253)
    stub = explorer_stub((cm, raw))

    assert g.info.resolution != cm.resolution != raw.resolution
    assert cm.cell_of(px, py) == (23, 34)
    assert raw.cell_of(px, py) == (13, 17)
    out = stub._pose_occupancy_readout(g, 7, px, py)

    assert 'cell (26, 28) value=0' in out            # the SLAM map's own cell
    assert 'cell (23, 34) cost=254 LETHAL' in out
    assert 'cell (13, 17) cost=253 INSCRIBED_INFLATED' in out
    # Each clause carries the geometry it was read with, so the numbers can be
    # checked against the run afterwards rather than assumed to match.
    assert '40x40 res=0.100 origin=(-2.000, -3.000)' in out
    assert '60x60 res=0.025 origin=(0.000, 0.000)' in out


def test_a_pose_the_costmap_does_not_cover_reads_off_rather_than_wrong():
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)
    cm = grid_of(occupancy_msg(10, 10, ox=5.0, oy=5.0))   # elsewhere entirely
    stub = explorer_stub((cm, None))

    out = stub._pose_occupancy_readout(g, 7, 0.06, 0.21)

    assert 'cost=off-costmap' in out
    assert '3x3 north-row-first [off,off,off] [off,off,off] [off,off,off]' in out


def test_the_readout_reports_how_stale_the_costmap_is():
    # A costmap from before the robot got where it is proves nothing about
    # where it is now, and 'the costmap said FREE' is worth exactly as much as
    # the age beside it.
    g = slam_grid(20, 20, ox=-0.5, oy=-0.5)
    cm = grid_of(occupancy_msg(20, 20, ox=-0.5, oy=-0.5), seq=12)
    cm.stamp = fx.time.monotonic() - 4.0
    cm.updates, cm.rejected = 3, 1
    stub = explorer_stub((cm, None))

    out = stub._pose_occupancy_readout(g, 7, 0.0, 0.0)

    assert 'seq 12, 3 updates applied (1 rejected)' in out
    assert 's old' in out
    assert f'/robot_0/{fx.COSTMAP_TOPIC} ' in out


def test_each_clause_names_the_scale_and_the_topic_it_was_read_from():
    # Two grids on two scales, and the numbers in a clause mean nothing without
    # knowing which. The scale comes off the GRID, not off the caller's label,
    # so a clause can never be headed with a scale its numbers are not on.
    g = slam_grid(20, 20, ox=-0.5, oy=-0.5)
    cm = grid_of(occupancy_msg(20, 20, ox=-0.5, oy=-0.5))
    raw = grid_of(costmap_msg(20, 20, ox=-0.5, oy=-0.5))
    stub = explorer_stub((cm, raw))

    out = stub._pose_occupancy_readout(g, 7, 0.0, 0.0)

    assert f'costmap[translated] /robot_0/{fx.COSTMAP_TOPIC} seq' in out
    assert f'costmap[raw] /robot_0/{fx.COSTMAP_RAW_TOPIC} seq' in out
    # The label is only a fallback, used where there is no grid to ask.
    assert stub._costmap_pose_readout(None, 'translated', 0.0, 0.0, None) == \
        'costmap[translated] none received'


def test_a_missing_slam_map_still_reports_the_costmaps():
    # The costmap is a separate publisher, so its silence and SLAM's are
    # different failures — this arm used to report neither.
    cm = at(grid_of(occupancy_msg(20, 20, ox=-0.5, oy=-0.5)), 0.0, 0.0, -2)
    stub = explorer_stub((cm, None))
    stub.sensor.get_map = lambda: (None, 0)
    stub.sensor.robot_xy = lambda: (0.0, 0.0)
    stub._dump_termination_diagnostics = \
        FrontierExplorer._dump_termination_diagnostics.__get__(stub)

    stub._dump_termination_diagnostics(fx.EXIT_START_BLOCKED, (0.0, 0.0))

    msg = [m for m in warns(stub) if 'termination diagnostic' in m][0]
    assert 'no map has ever been received' in msg
    assert 'cost=254 LETHAL' in msg
    assert 'no occupied cell in the SLAM grid to compare' in msg


def test_a_malformed_grid_warns_instead_of_raising():
    # A diagnostic may never crash the node or move an exit path.
    stub = explorer_stub((object(), None))
    stub.sensor.get_map = lambda: (slam_grid(4, 4), 1)
    stub.sensor.robot_xy = lambda: (0.0, 0.0)
    stub._dump_termination_diagnostics = \
        FrontierExplorer._dump_termination_diagnostics.__get__(stub)

    stub._dump_termination_diagnostics(fx.EXIT_START_BLOCKED, (0.0, 0.0))

    assert any('termination diagnostic' in m and 'failed' in m
               for m in warns(stub)), stub.logged


# ── the halt verdict ────────────────────────────────────────────────────────
#
# What replaced a sentence that was wrong in both its premise and its advice:
#
#   "An occupied value at the robot's own cell means the believed pose is
#    inside a mapped wall (SLAM pose error), NOT that the robot is in a pocket
#    — do not add a standoff or back up on it."
#
# It was printed unconditionally, so it was printed at rung2h's halt too, where
# the SLAM cell read -1 (UNKNOWN, not occupied) and the raw costmap read 253
# INSCRIBED_INFLATED across the whole 3x3 with a 254 at 0.072 m. The robot was
# in an inflation pocket of peer-robot residue and backing out is what gets it
# free — the one action the sentence forbade. These tests pin one verdict per
# reading, the refusal to give one without the costmap, and the numbers that
# let each verdict be checked rather than believed.

REMOVED_CLAIMS = ('inside a mapped wall', 'do not add a standoff',
                  'NOT that the robot is in a pocket')


def raw_at(cost, w=40, h=40, ox=-1.0, oy=-1.0, fill=0, px=0.06, py=0.21):
    """A raw-scale costmap reading `cost` under (px, py)."""
    return at(grid_of(costmap_msg(w, h, ox=ox, oy=oy, fill=fill)), px, py, cost)


def rung2h_pocket():
    """The costmap as rung2h actually read it: 253 everywhere, one 254 corner."""
    raw = grid_of(costmap_msg(40, 40, ox=-1.0, oy=-1.0, fill=253))
    return at(raw, 0.06 + 0.05, 0.21 + 0.05, 254)


def test_an_occupied_slam_cell_makes_pose_error_the_verdict():
    v = fx._halt_verdict(100, raw_at(254), 0.06, 0.21)

    assert v.startswith('Verdict: pose error is plausible')
    assert 'inside mapped structure' in v
    # ...and it carries all three readings, so the claim can be checked against
    # its own evidence.
    assert 'SLAM value=100 (occupied)' in v
    assert 'raw cost=254 LETHAL' in v
    assert '1/9 of the 3x3 at 253-254' in v


def test_an_unoccupied_cell_inside_the_inflation_reads_as_a_pocket():
    # rung2h exactly: unknown under the robot, INSCRIBED_INFLATED under the
    # planner. The reading the removed sentence could not represent at all.
    v = fx._halt_verdict(-1, rung2h_pocket(), 0.06, 0.21)

    assert v.startswith('Verdict: inflation pocket')
    assert 'free or unknown space' in v
    assert "inside another obstacle's inflation radius" in v
    assert 'SLAM value=-1 (unknown)' in v
    assert 'raw cost=253 INSCRIBED_INFLATED' in v
    assert '9/9 of the 3x3 at 253-254' in v
    # The two things the run established about this state, and nothing further.
    assert 'Short-range plans may still succeed where long-range ones fail' in v
    assert 'displacement out of the inflated region is the indicated recovery' in v


def test_the_pocket_verdict_prescribes_no_distance_and_no_threshold():
    # rung2h's reach limit — successes 0.43-1.81 m, failures 0.66-6.34 m — is
    # one run, and the two bands overlap. A number taken from it and printed as
    # advice would be the removed sentence's mistake with a decimal point.
    v = fx._halt_verdict(-1, rung2h_pocket(), 0.06, 0.21)

    assert ' m ' not in v and ' m.' not in v
    for measured in ('0.43', '1.81', '0.66', '6.34', '0.072'):
        assert measured not in v


def test_a_free_cell_under_an_ordinary_cost_claims_neither():
    v = fx._halt_verdict(0, raw_at(0), 0.06, 0.21)

    assert v.startswith('Verdict: neither pose error nor inflation pocket')
    assert 'some other cause' in v
    assert 'SLAM value=0 (free)' in v and 'raw cost=0 FREE' in v
    assert '0/9 of the 3x3 at 253-254' in v


def test_a_mid_inflation_cost_is_not_a_pocket():
    # 1-252 is the inflation gradient, which navfn will happily expand through.
    v = fx._halt_verdict(-1, raw_at(196), 0.06, 0.21)

    assert v.startswith('Verdict: neither')
    assert 'raw cost=196 COST' in v


def test_no_information_under_the_robot_is_not_called_a_pocket():
    # 255 is >= 253 but is not inflation: it is a cell the costmap has no
    # evidence about. Twenty of rung2h's fifty precheck readings were 255, so
    # calling this a pocket would mislabel the commonest reading in the run.
    v = fx._halt_verdict(-1, raw_at(255, fill=255), 0.06, 0.21)

    assert v.startswith('Verdict: neither')
    assert 'raw cost=255 NO_INFORMATION' in v
    assert '0/9 of the 3x3 at 253-254' in v


def test_no_costmap_means_no_verdict_rather_than_a_slam_only_guess():
    # The SLAM grid alone is exactly what the removed sentence reasoned from.
    # With no costmap there is no second reading to check it against, so the
    # honest output is the absence.
    v = fx._halt_verdict(100, None, 0.06, 0.21)

    assert v.startswith('No verdict')
    assert 'nothing has ever been received on the raw costmap' in v
    assert 'SLAM value=100 (occupied)' in v
    # An occupied cell and no costmap is precisely the case the old sentence
    # ruled on. Nothing is ruled here.
    assert 'pose error is plausible' not in v
    assert 'inflation pocket' in v.split('cannot tell')[1]   # named, not claimed
    for claim in REMOVED_CLAIMS:
        assert claim not in v


def test_a_pose_the_costmap_does_not_cover_gets_no_verdict_either():
    elsewhere = grid_of(costmap_msg(10, 10, ox=5.0, oy=5.0))
    v = fx._halt_verdict(-1, elsewhere, 0.06, 0.21)

    assert v.startswith('No verdict')
    assert 'outside the raw costmap' in v
    assert 'SLAM value=-1 (unknown)' in v


def test_an_off_grid_pose_is_not_read_as_mapped_structure():
    # A pose the SLAM map does not cover is not evidence of a wall under it.
    g = slam_grid(4, 4, ox=0.0, oy=0.0)
    assert fx._slam_value_at(g, 99.0, 99.0) is None

    v = fx._halt_verdict(None, rung2h_pocket(), 0.06, 0.21)

    assert v.startswith('Verdict: inflation pocket')
    assert 'SLAM value=off-grid (off-grid)' in v


def test_the_ring_count_counts_the_pocket_costs_in_the_3x3_only():
    raw = grid_of(costmap_msg(40, 40, ox=-1.0, oy=-1.0))
    cx, cy = raw.cell_of(0.06, 0.21)
    for i, cost in enumerate((253, 254, 255, 100)):
        raw.data[(cy - 1 + i // 3) * 40 + (cx - 1 + i % 3)] = cost
    raw.data[cy * 40 + cx] = 253                 # the robot's own cell counts
    raw.data[(cy + 2) * 40 + cx] = 254           # a 254 just outside does not

    v = fx._halt_verdict(-1, raw, 0.06, 0.21)

    assert '3/9 of the 3x3 at 253-254' in v


@pytest.mark.parametrize('value,state', [
    (100, 'occupied'), (50, 'occupied'), (49, 'free'), (0, 'free'),
    (-1, 'unknown'), (None, 'off-grid'),
])
def test_the_verdict_names_the_slam_state_on_the_readouts_own_threshold(
        value, state):
    # Same OCC_THRESH the readout printed beside it: the verdict must never
    # call a cell occupied that the line above it calls free.
    assert fx._slam_state(value) == state
    assert f'({state})' in fx._halt_verdict(value, raw_at(0), 0.06, 0.21)


# ── the verdict where the dump emits it ─────────────────────────────────────

def dump_stub(costmaps, grid, tmp_path, monkeypatch, pose=(0.06, 0.21)):
    monkeypatch.setattr(fx, '_map_dump_dir', lambda: str(tmp_path))
    stub = explorer_stub(costmaps)
    stub.sensor.get_map = lambda: (grid, 7)
    stub.sensor.robot_xy = lambda: pose
    stub._dump_termination_diagnostics = \
        FrontierExplorer._dump_termination_diagnostics.__get__(stub)
    return stub


def dumped(stub):
    return [m for lvl, m in stub.logged if 'termination diagnostic' in m][0]


def boom(*args, **kwargs):
    raise RuntimeError('the readings went away')


def test_the_removed_sentence_is_gone_from_a_non_occupied_halt(tmp_path,
                                                               monkeypatch):
    # The regression this commit exists to prevent: rung2h's own readings, and
    # the log must no longer tell the operator that backing up is the wrong
    # move. Unknown under the robot, 253 under the planner.
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0, fill=-1)
    stub = dump_stub((None, rung2h_pocket()), g, tmp_path, monkeypatch)

    stub._dump_termination_diagnostics(fx.EXIT_DEFER_TRUNCATED, (0.0, 0.0))

    msg = dumped(stub)
    assert 'value=-1' in msg                       # the reading it ruled on
    for claim in REMOVED_CLAIMS:
        assert claim not in msg
    assert 'Verdict: inflation pocket' in msg
    assert 'displacement out of the inflated region' in msg


def test_an_occupied_halt_still_gets_the_pose_error_reading(tmp_path,
                                                            monkeypatch):
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)
    g.data[24 * 40 + 21] = 100                     # (0.06, 0.21)
    stub = dump_stub((None, raw_at(254)), g, tmp_path, monkeypatch)

    stub._dump_termination_diagnostics(fx.EXIT_START_BLOCKED, (0.0, 0.0))

    msg = dumped(stub)
    assert 'Verdict: pose error is plausible' in msg
    # Plausible, not proven, and with no instruction attached either way.
    for claim in REMOVED_CLAIMS:
        assert claim not in msg


def test_a_dump_with_no_costmap_states_the_absence_where_the_verdict_goes(
        tmp_path, monkeypatch):
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)
    g.data[24 * 40 + 21] = 100
    stub = dump_stub((None, None), g, tmp_path, monkeypatch)

    stub._dump_termination_diagnostics(fx.EXIT_DEFER_UNANSWERED, (0.0, 0.0))

    msg = dumped(stub)
    assert 'No verdict' in msg
    assert 'Verdict:' not in msg
    for claim in REMOVED_CLAIMS:
        assert claim not in msg
    # The readout it is appended to is unaffected, and the missing costmap is
    # still warned about exactly once.
    assert 'cell (21, 24) value=100' in msg
    assert len([m for m in warns(stub) if 'nothing has ever been received on '
                f'{fx.COSTMAP_TOPIC}' in m]) == 1


def test_a_clean_completion_is_given_no_verdict(tmp_path, monkeypatch):
    # Nothing halted, so there is nothing to explain; a verdict on the pose
    # would read as an accusation of a run in which nothing went wrong.
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0, fill=-1)
    stub = dump_stub((None, rung2h_pocket()), g, tmp_path, monkeypatch)

    stub._dump_termination_diagnostics(fx.EXIT_NO_FRONTIERS, (0.0, 0.0))

    msg = dumped(stub)
    assert 'Verdict' not in msg and 'verdict' not in msg
    assert 'cell (21, 24) value=-1' in msg          # the readout is unchanged


def test_a_verdict_that_cannot_be_formed_costs_only_the_verdict(tmp_path,
                                                                monkeypatch):
    # Diagnostic text may not take the readout down with it, and may not raise.
    g = slam_grid(40, 40, ox=-1.0, oy=-1.0)
    stub = dump_stub((None, raw_at(0)), g, tmp_path, monkeypatch)
    monkeypatch.setattr(fx, '_halt_verdict', boom)

    stub._dump_termination_diagnostics(fx.EXIT_START_BLOCKED, (0.0, 0.0))

    msg = dumped(stub)
    assert 'No verdict: its readings could not be taken' in msg
    assert 'cell (21, 24) value=0' in msg
    assert list(tmp_path.glob('*.pgm'))             # and the map still saved


# ── the probe log lines ─────────────────────────────────────────────────────

class FakePlanner:
    """Scripted stand-in for _SensorNode.plan_to() (see the precheck file)."""

    def __init__(self, replies=None):
        self.replies = replies or {}
        self.asked = []

    def plan_to(self, x, y, frame, timeout=None):
        self.asked.append((round(x, 3), round(y, 3)))
        return self.replies.get((round(x, 3), round(y, 3)),
                                (0, [(0.0, 0.0), (x, y)], 0.01, ''))


def probe_stub(planner, costmaps=(None, None), centroids=None,
               targets=((0.5, 0.0),)):
    stub = explorer_stub(costmaps)
    stub.map_frame = 'robot_0/map'
    stub.sensor.plan_to = planner.plan_to
    stub._precheck = True
    stub._blacklist = []
    stub._hb_last = {}
    stub._retirements = fx.RetirementLedger(fx.BLACKLIST_RADIUS,
                                            fx.RETIRE_TTL_MAPS)
    stub._frontier_centroids = lambda: list(centroids or [(0.0, 1.0, 10)])
    stub._free_targets_near = lambda rx, ry: list(targets)
    for name in ('_probe', '_start_probe', '_select_goal',
                 '_ordered_candidates', '_blacklisted', '_hb'):
        setattr(stub, name, getattr(FrontierExplorer, name).__get__(stub))
    return stub


def infos(stub):
    return [m for lvl, m in stub.logged if lvl == 'INFO']


def test_the_precheck_verdict_line_carries_the_cost_that_produced_it():
    # The line the whole run is read from. The planner's answer and the grid it
    # answered from have to be on ONE line — matching a verdict up with a
    # readout logged elsewhere is the reconstruction this change removes.
    cm = at(grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0)), 0.0, 1.0, 100)
    raw = at(grid_of(costmap_msg(80, 80, ox=-2.0, oy=-2.0)), 0.0, 1.0, 254)
    planner = FakePlanner({(0.0, 1.0): (208, [], 0.01, 'no valid path')})
    stub = probe_stub(planner, (cm, raw))

    verdict, code = stub._probe((0.0, 1.0), 0)

    assert (verdict, code) == (reachability.UNREACHABLE, 208)
    line = [m for m in infos(stub) if 'precheck cand#0' in m][0]
    assert 'error_code=208' in line
    assert 'costmap=100 COST raw=254 LETHAL' in line


def test_an_unanswered_precheck_still_reports_the_cost():
    # A candidate the planner never answered about is exactly where the grid is
    # the only evidence there is.
    cm = at(grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0)), 0.0, 1.0, -3)
    planner = FakePlanner({(0.0, 1.0): None})
    stub = probe_stub(planner, (cm, None))

    assert stub._probe((0.0, 1.0), 0) == (reachability.INCONCLUSIVE, None)
    line = [m for m in infos(stub) if 'precheck cand#0' in m][0]
    assert 'INCONCLUSIVE' in line
    assert 'costmap=253 INSCRIBED_INFLATED int8=-3' in line


def test_every_start_probe_target_reports_its_cost():
    # Targets come from the SLAM map, which has no inflation layer — so the
    # costmap's cost AT a target is what says whether "free cell" meant the
    # same thing to the planner that refused to reach it.
    cm = grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0))
    at(cm, 0.5, 0.0, -3)
    at(cm, 0.0, 0.5, 0)
    planner = FakePlanner({(0.5, 0.0): (208, [], 0.01, ''),
                           (0.0, 0.5): (0, [(0.0, 0.0), (0.0, 0.5)], 0.01, '')})
    stub = probe_stub(planner, (cm, None), targets=((0.5, 0.0), (0.0, 0.5)))

    assert stub._start_probe(0.0, 0.0) == reachability.START_OK

    lines = [m for m in infos(stub) if 'start probe' in m]
    assert len(lines) == 2
    assert 'costmap=253 INSCRIBED_INFLATED int8=-3' in lines[0]
    assert 'costmap=0 FREE' in lines[1]


def test_every_costmap_readout_is_findable_by_one_grep():
    # `grep -a 'costmap='` has to find every one of them; a readout that only
    # some lines carry is the empty-grep failure in a smaller form.
    cm = grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0))
    planner = FakePlanner({(0.0, 1.0): (208, [], 0.01, '')})
    stub = probe_stub(planner, (cm, None))
    stub._probe((0.0, 1.0), 0)
    stub._start_probe(0.0, 0.0)

    lines = [m for m in infos(stub)
             if 'precheck cand#' in m or 'start probe ' in m]
    assert lines
    assert all('costmap=' in m for m in lines), lines


# ── it is a diagnostic: selection may not move ──────────────────────────────

THREE = [(0.0, 3.0, 30), (0.0, 1.0, 10), (0.0, 2.0, 20)]


@pytest.mark.parametrize('costmaps,label', [
    ((None, None), 'no costmap at all'),
    ((grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0)), None), 'free costmap'),
    ((grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0, fill=-2)),
      grid_of(costmap_msg(80, 80, ox=-2.0, oy=-2.0, fill=254))), 'all lethal'),
])
def test_selection_reaches_the_same_goal_whatever_the_costmap_says(costmaps,
                                                                   label):
    # The pin on "pure addition". A costmap reporting LETHAL everywhere — which
    # would justify retiring the lot, if any of this fed selection — must pick
    # the same goal as no costmap at all. START_PROBE_CLEAR stays hardcoded
    # until this readout produces the data to replace it with.
    stub = probe_stub(FakePlanner(), costmaps, centroids=THREE)

    assert stub._select_goal(0.0, 0.0, 0) == ((0.0, 1.0), (0.0, 1.0), 10), label
    assert stub._retirements.active(0) == 0, label


def test_a_condemned_cycle_defers_the_same_way_with_a_lethal_costmap():
    condemned = {(0.0, y): (208, [], 0.01, '') for y in (1.0, 2.0, 3.0)}
    lethal = grid_of(occupancy_msg(80, 80, ox=-2.0, oy=-2.0, fill=-2))
    stub = probe_stub(FakePlanner(condemned), (lethal, None), centroids=THREE)

    assert stub._select_goal(0.0, 0.0, 0) is DEFER
    # Retirement keys on the planner's verdict, never on a cost.
    assert stub._retirements.active(0) == 3
    assert stub._last_defer_definitive is True


def test_the_costmap_never_changes_how_far_a_probe_target_is_looked_for():
    # _free_targets_near still reads the SLAM map only: its lack of an
    # inflation layer is what makes a failed probe informative (see
    # START_PROBE_CLEAR), and this change deliberately does not touch it.
    stub = SimpleNamespace(sensor=SimpleNamespace(
        get_map=lambda: (slam_grid(41, 41, ox=-1.0, oy=-1.0), 0),
        get_costmaps=lambda: (None, None)))
    stub._cell_has_clearance = FrontierExplorer._cell_has_clearance
    stub._free_targets_near = FrontierExplorer._free_targets_near.__get__(stub)

    targets = stub._free_targets_near(0.0, 0.0)

    assert len(targets) == fx.START_PROBE_MAX
    lo = fx.START_PROBE_DIST - fx.START_PROBE_RING
    hi = fx.START_PROBE_DIST + fx.START_PROBE_RING
    for tx, ty in targets:
        assert lo <= math.hypot(tx, ty) <= hi, (tx, ty)
