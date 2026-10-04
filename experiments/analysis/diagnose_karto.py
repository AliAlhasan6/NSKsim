#!/usr/bin/env python3
"""Rebuild Karto's per-cell hit and pass counts offline, and split the passes by
source. A DIAGNOSIS, and nothing else.

WHY. Part B found that four of the twenty b2maps grids hold real wall that the map
records as FREE (SPEC_b2_wall_predicate §12). diagnose_dwell showed the cells were
CLEARED rather than never seen (P3), but it counted HITS by source, and a cell is
cleared by PASSES: a cell hit while the robot moved can be cleared afterwards by
parked rays crossing it. Only a rebuild of Karto's own counters can separate the
sources, so this rebuilds them, exactly, and then removes one source at a time.

THE RULES, ALL COPIED FROM SOURCE, ALL CITED. Read at
~/src/slam_toolbox_2.8.5 (ros-jazzy-slam-toolbox 2.8.5, orig tarball SHA256
80af3955a610a81c320b4a75bff6b5e5ca99a6118455b7ca5ee1530e25608583); never copied
into this repo. Paths below are relative to that tree; Karto.h means
lib/karto_sdk/include/karto_sdk/Karto.h.

  line tracer     Karto.h:4874-4926, Grid<T>::TraceLine. Bresenham: swap to steep
                  if |dy|>|dx|, then swap ends if x0>x1. Per iteration the cell is
                  taken at the CURRENT cross coordinate, THEN error += deltaY and
                  the cross coordinate steps if 2*error >= deltaX. `x <= x1`, so
                  BOTH endpoints are traced. Cells outside the grid are skipped,
                  never clamped.
  pass vs hit     Karto.h:6202-6236, OccupancyGrid::RayTrace. TraceLine gives +1
                  PASS to every valid cell including the endpoint; then, only if
                  isEndPointValid, the endpoint cell gets ANOTHER +1 pass and +1
                  hit (:6226-6227). One valid ray therefore leaves its hit cell
                  with 2 passes and 1 hit.
  clipping        Karto.h:6148-6192, OccupancyGrid::AddScan.
                  isEndPointValid = r < rangeThreshold - KT_TOLERANCE (:6167,
                  KT_TOLERANCE 1e-06 at Math.h:41). r <= minRange, r >= maxRange
                  or NaN: ignored entirely, no pass and no hit (:6170). Otherwise
                  r >= rangeThreshold: the endpoint is rescaled by
                  rangeThreshold/r along the ray and traced FREE-ONLY (:6173).
                  rangeThreshold itself is clipped to [minRange, maxRange]
                  (Karto.h:3949).
  hit point       Karto.h:5640-5665, LocalizedRangeScan::Update.
                  angle = sensorYaw + minimumAngle + beam*angularResolution;
                  point = sensorPosition + r*(cos angle, sin angle). AddScan reads
                  the UNFILTERED readings, one per beam (Karto.h:6157).
  thresholds      Karto.h:6244-6256 UpdateCell, :6260-6277 Update, :4612
                  Grid::Clear, :4379-4382 GridStates. Update memsets to 0 =
                  Unknown, then: passes > MinPassThrough and hits/passes >
                  OccupancyThreshold -> Occupied; same pass test and ratio at or
                  below -> Free; passes <= MinPassThrough -> stays Unknown. Both
                  comparisons strict.
  values          src/slam_mapper.cpp:360-372 declares min_pass_through default 2
                  and occupancy_threshold default 0.1, and :67-69 passes them with
                  mapper_->GetAllProcessedScans() to CreateFromScans. The
                  replay config sets NEITHER, so those defaults built these maps.
  which scans     karto_sdk/src/Mapper.cpp:3147-3178 HasMovedEnough returns true
                  on the FIRST test, `timeInterval >= m_pMinimumTimeInterval`, and
                  the replay config sets minimum_time_interval 0.0 -- so a scan
                  from an identical pose IS integrated, and the travel tests are
                  never reached. Upstream, src/slam_toolbox_common.cpp:756-816
                  shouldProcessScan drops nothing here EXCEPT `if (scan_ctr < 5)
                  return false` (:795-797): the 2nd, 3rd and 4th scans of the run.
                  throttle_scans defaults to 1 (:366-370) and is not set.
  cell indexing   Karto.h:4421-4436 WorldToGrid: grid = Round((world - offset) *
                  scale). ROUND, not floor -- so in Karto's geometry a cell's
                  centre sits at offset + i*res. This file indexes cells that way,
                  which is the rule the rebuild must match.

THE HALF-CELL SHIFT, which this file reported and which is now FIXED upstream of
it. toNavMap (include/slam_toolbox/visualization_utils.hpp:108-129) publishes
map.info.origin = the Karto grid offset verbatim. ROS then reads cell i as
spanning [origin + i*res, origin + (i+1)*res) with its centre at
origin + (i+0.5)*res, while Karto placed that cell's centre at origin + i*res --
so every ROS reading of a slam_toolbox map sat half a cell (0.05 m here) further
+x and +y than Karto did. This file always indexed Karto's way, and that is now
the project's one convention: trinary_map.CELL_CENTRE_OFFSET = 0.0, which
world_to_grid and the cell-centre lookups below go through rather than restate.
Nothing here MOVED -- the arithmetic was already right -- so diagnose_karto.json's
numbers are unaffected, and the GATE below is index-based in any case.

WHAT THIS TOUCHES. It writes exactly one file,
experiments/logs/graph_walls/diagnose_karto.json. It opens no map JSON and moves
no threshold. Every pose, scan and sensor pose comes from graph_walls, so the
geometry is the same one Part B measured.

TRUTH. The grazing counterfactual (c) and the free-depth report (step 5) use the
world SDF, and are diagnosis only -- they stay in this file and out of every
<map>.json, as §6 requires.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/diagnose_karto.py --map b2maps_k1_cut1200_robot1
    python3 experiments/analysis/diagnose_karto.py

One map first, then the corpus. No simulator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import graph_walls as gw                                     # noqa: E402
from trinary_map import (  # noqa: E402
    CELL_CENTRE_OFFSET,
    cell_centre,
    cell_centre_provenance,
    cell_index,
)

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_karto.json'

SOURCE_TREE = Path.home() / 'src' / 'slam_toolbox_2.8.5'
SOURCE_SHA256 = ('80af3955a610a81c320b4a75bff6b5e5ca99a6118455b7ca5ee1530e256'
                 '08583')

# ── values copied from source, with their lines ──────────────────────────────
MIN_PASS_THROUGH = 2          # src/slam_mapper.cpp:360
OCCUPANCY_THRESHOLD = 0.1     # src/slam_mapper.cpp:367
KT_TOLERANCE = 1e-06          # lib/karto_sdk/include/karto_sdk/Math.h:41
# slam_toolbox_common.cpp:795-797, `if (scan_ctr < 5) return false` -- the first
# scan is let through by first_measurement_ (ctr 1), so ctr 2, 3 and 4 are lost.
STABILISATION_DROP = (2, 3, 4)

# Karto's three cell states, Karto.h:4379-4382.
K_UNKNOWN, K_OCCUPIED, K_FREE = 0, 100, 255

# The stationary test, verbatim from diagnose_dwell so a scan carries the same
# label in both diagnoses.
STILL_M = 0.01
STILL_DEG = 0.5

# Step 4(c). 75 deg from the nearest SDF face normal, as specified.
GRAZE_DEG = 75.0

# Step 3's pre-registered gate.
GATE_MIN_AGREEMENT = 0.995

HYPOTHESES = [
    ('H_relay', 'removing relay-filled rays recommits >= 50 % of the off-wall '
                'cells, on each of the four B1 failures'),
    ('H_graze', 'removing finite rays that cross at >= 75 deg from the nearest '
                'SDF face normal recommits >= 50 %, on each of the four'),
    ('H_dwell', 'removing stationary rays recommits >= 50 % on k1_cut1200 and '
                'k3_cut1200, and < 10 % on k0_cut240 and k4_cut120'),
]


# ─────────────────── the tracer: verbatim, then vectorised ───────────────────

def trace_line_reference(x0: int, y0: int, x1: int, y1: int) -> list:
    """Karto.h:4874-4926, transcribed statement for statement.

    Deliberately a slow, literal port with the same swaps, the same
    `error += deltaY` before the same `2 * error >= deltaX`, and the same
    `x <= x1`. It exists to be the thing the fast version is checked against;
    nothing calls it on real data.
    """
    steep = abs(y1 - y0) > abs(x1 - x0)
    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0

    delta_x = x1 - x0
    delta_y = abs(y1 - y0)
    error = 0
    ystep = 1 if y0 < y1 else -1
    y = y0

    out = []
    for x in range(x0, x1 + 1):
        if steep:
            point_x, point_y = y, x
        else:
            point_x, point_y = x, y
        error += delta_y
        if 2 * error >= delta_x:
            y += ystep
            error -= delta_x
        out.append((point_x, point_y))
    return out


def trace_rays(x0: np.ndarray, y0: np.ndarray, x1: np.ndarray, y1: np.ndarray
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Every cell of every ray, vectorised. Same cells as trace_line_reference.

    The closed form. After the two swaps the along coordinate runs x0..x1 and the
    cross coordinate at step i is `y0 + ystep * s_i`, where s_i is how many times
    the error has fired in steps 0..i-1. Karto's recurrence gives

        s_i = floor((2*i*deltaY + deltaX) / (2*deltaX))

    which test_trace_matches_karto pins against the transcribed loop on random
    rays rather than asserting.

    Returns (cell x, cell y, ray index for each cell).
    """
    steep = np.abs(y1 - y0) > np.abs(x1 - x0)
    ax0 = np.where(steep, y0, x0)
    ay0 = np.where(steep, x0, y0)
    ax1 = np.where(steep, y1, x1)
    ay1 = np.where(steep, x1, y1)
    flip = ax0 > ax1
    ax0, ax1 = np.where(flip, ax1, ax0), np.where(flip, ax0, ax1)
    ay0, ay1 = np.where(flip, ay1, ay0), np.where(flip, ay0, ay1)

    delta_x = ax1 - ax0
    delta_y = np.abs(ay1 - ay0)
    ystep = np.where(ay0 < ay1, 1, -1)

    n_steps = delta_x + 1
    ray = np.repeat(np.arange(n_steps.size), n_steps)
    # step index within each ray: 0, 1, .. delta_x
    starts = np.concatenate([[0], np.cumsum(n_steps)[:-1]])
    i = np.arange(ray.size) - starts[ray]

    dx_r = delta_x[ray]
    safe = np.where(dx_r > 0, dx_r, 1)
    s = np.where(dx_r > 0, (2 * i * delta_y[ray] + dx_r) // (2 * safe), 0)
    along = ax0[ray] + i
    cross = ay0[ray] + ystep[ray] * s

    st = steep[ray]
    cx = np.where(st, cross, along)
    cy = np.where(st, along, cross)
    return cx, cy, ray


# ───────────────────────────── Karto's geometry ──────────────────────────────

def world_to_grid(wx: np.ndarray, wy: np.ndarray, origin, rho: float) -> tuple:
    """Karto.h:4421-4436. Round, not floor; centres at origin + i*res.

    trinary_map.cell_index is that same arithmetic with the convention named:
    under CELL_CENTRE_OFFSET = 0.0 it is floor((w - origin)/rho + 0.5), which is
    Round for every value this sees. Kept as a named function because the Karto
    citation belongs on it, but it no longer carries its own copy of the offset.
    """
    assert CELL_CENTRE_OFFSET == 0.0, (
        'world_to_grid is Karto\'s WorldToGrid, which rounds about '
        'origin + i*res. A non-zero CELL_CENTRE_OFFSET would make cell_index '
        'something else and this citation false.')
    return (cell_index(wx, origin[0], rho), cell_index(wy, origin[1], rho))


def update_cell(passes: np.ndarray, hits: np.ndarray,
                min_pass_through: int = MIN_PASS_THROUGH,
                occupancy_threshold: float = OCCUPANCY_THRESHOLD
                ) -> np.ndarray:
    """Karto.h:6244-6256 UpdateCell, over a whole grid at once.

    Cells at or below the pass threshold keep the Unknown that Update's Clear
    memset left there (Karto.h:6260-6266, :4612).
    """
    out = np.full(passes.shape, K_UNKNOWN, dtype=np.uint8)
    enough = passes > min_pass_through
    with np.errstate(invalid='ignore', divide='ignore'):
        ratio = np.where(enough, hits / np.where(enough, passes, 1), 0.0)
    out[enough & (ratio > occupancy_threshold)] = K_OCCUPIED
    out[enough & ~(ratio > occupancy_threshold)] = K_FREE
    return out


def karto_to_trinary(state: np.ndarray) -> np.ndarray:
    """Karto state -> this project's PGM classes.

    toNavMap (visualization_utils.hpp:120-128) sends Unknown to -1, Occupied to
    100 and Free to 0; save_map.py then writes 205, 0 and 254 under
    occupied_thresh 0.65 / free_thresh 0.25, which those three values cannot
    straddle ambiguously.
    """
    out = np.full(state.shape, gw.UNKNOWN, dtype=np.uint8)
    out[state == K_OCCUPIED] = gw.OCC
    out[state == K_FREE] = gw.FREE
    return out


# ──────────────────────────── scans and sources ──────────────────────────────

def relay_fill(ranges: np.ndarray, range_max: float,
               range_threshold: float) -> tuple[np.ndarray, np.ndarray]:
    """The free_space_relay's rewrite, and which beams it rewrote.

    free_space_relay.py:71-97: +inf and finite readings AT OR ABOVE range_max are
    replaced by 0.5*(range_threshold + range_max); NaN is left alone. Everything
    else passes through untouched, which is why the raw topic can stand in for the
    scan_free the replay actually consumed.
    """
    fill = 0.5 * (range_threshold + range_max)
    filled = (~np.isnan(ranges)) & (np.isinf(ranges) | (ranges >= range_max))
    out = np.where(filled, fill, ranges)
    return out, filled


def stationary_scans(sx: np.ndarray, sy: np.ndarray, syaw: np.ndarray,
                     ok: np.ndarray) -> np.ndarray:
    """diagnose_dwell's definition, unchanged: under 0.01 m AND under 0.5 deg
    since the previous pose-valid scan, with the first counted as moving."""
    stat = np.zeros(sx.size, dtype=bool)
    idx = np.nonzero(ok)[0]
    if idx.size < 2:
        return stat
    step = np.hypot(np.diff(sx[idx]), np.diff(sy[idx]))
    turn = np.abs((np.diff(syaw[idx]) + math.pi) % (2.0 * math.pi) - math.pi)
    stat[idx[1:]] = (step < STILL_M) & (turn < math.radians(STILL_DEG))
    return stat


def reaching_grid(ok: np.ndarray) -> np.ndarray:
    """Which scans reached Karto's grid.

    Every pose-valid scan except the 2nd, 3rd and 4th of the sequence: Karto's
    HasMovedEnough passes all of them on the time test (Mapper.cpp:3155-3158 with
    minimum_time_interval 0.0), and slam_toolbox's shouldProcessScan drops only
    `scan_ctr < 5` (slam_toolbox_common.cpp:795-797). A scan with no pose never
    reached the node's message filter at all.
    """
    reach = ok.copy()
    idx = np.nonzero(ok)[0]
    for ctr in STABILISATION_DROP:
        if idx.size >= ctr:
            reach[idx[ctr - 1]] = False
    return reach


# ──────────────────────────── the world, for (c) and step 5 ──────────────────

def nearest_face_normals(shape, origin, rho: float) -> tuple:
    """Per cell, the outward normal of the nearest SDF face, and the distance.

    Truth, used only by the grazing counterfactual and the free-depth report.
    Cell centres are placed by KARTO's rule (origin + i*res), the same rule the
    rebuild indexes by, so the two cannot drift apart.
    """
    # Called at the real l_min so sdf_wall_faces' own pre-registered assertions
    # hold; `faces` is every face regardless, caps included, which is what the
    # nearest-normal lookup wants.
    world = gw.sdf_wall_faces(gw.L_MIN)
    h, w = shape
    ys, xs = np.mgrid[0:h, 0:w]
    cx = cell_centre(origin[0], xs, rho)
    cy = cell_centre(origin[1], ys, rho)
    best = np.full(shape, np.inf)
    nx = np.zeros(shape)
    ny = np.zeros(shape)
    for f in world['faces']:
        ax, ay = f['p0']
        bx, by = f['p1']
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        t = np.clip(((cx - ax) * vx + (cy - ay) * vy) / vv, 0.0, 1.0)
        d = np.hypot(cx - (ax + t * vx), cy - (ay + t * vy))
        closer = d < best
        best = np.where(closer, d, best)
        nx = np.where(closer, f['n'][0], nx)
        ny = np.where(closer, f['n'][1], ny)
    return nx, ny, best


# ───────────────────────────── the rebuild ───────────────────────────────────

def bag_path_for(stem: str, k: int, cut: int) -> Path:
    """The stripped bag whose scans built `stem`'s map.

    Through graph_walls.bag_variant_of, never from (k, cut) alone. The gate lives
    in the BAG -- minimum_travel_distance and _heading are 0.0 in the b2maps
    configs, so Karto gates nothing -- while an overlay build replays the
    recording of the run it is named after. So `_gated` belongs in this path and
    `_extfix` and `_ovl` must not: a stem of the old corpus resolves to exactly
    the path this function's caller built before any variant existed, and a
    `_gated_extfix` stem resolves to the gated bag instead of silently admitting
    scans its map never saw.

    Pure: it builds a name and opens nothing, which is what lets the mapping be
    tested where there are no bags.
    """
    return (gw.BAGS_DIR
            / f'{gw.RUN_PREFIX}_k{k}_cut{cut}{gw.bag_variant_of(stem)}_slamin')


def run_name_for(stem: str, k: int, cut: int) -> str:
    """The RUN this stem's run artefacts carry: replay config, map->odom log.

    Through graph_walls.run_variant_of, which keeps EVERY variant token -- the
    opposite of bag_path_for, and for the opposite reason. A run artefact is
    rendered per build, so an _extfix replay wrote its own config and its own
    map->odom capture and those are the ones that describe it; a bag is a
    recording that an overlay build replays unchanged, so _extfix must not
    appear in a bag name. A plain stem gives the plain run name, which is what
    the old corpus has always read.

    Pure: it builds a name and opens nothing.
    """
    return f'{gw.RUN_PREFIX}_k{k}_cut{cut}{gw.run_variant_of(stem)}'


def scan_inputs(stem: str, k: int, cut: int) -> dict:
    """Everything a ray needs, before anything is traced.

    Factored out of rebuild() so that "which rays built this map" has exactly
    ONE answer here. The scan admission, the poses, the sensor offset and the
    relay ceiling are assembled once, and both rebuild() and admitted_rays()
    take them from this function; a second copy of the sequence would be a
    second answer, and SPEC_b2_divergence_walls §4 turns on there being one.
    """
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    t0 = time.monotonic()
    graph, _el, _seg, grid = gw.graph_and_elements(stem)
    rho, origin = graph['resolution'], graph['origin']
    r_max, r_max_src = gw.read_replay_r_max(k, cut, gw.run_variant_of(stem))

    data = gw.read_bag(bag_path_for(stem, k, cut), k)
    geom = data['geom']
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    fwt.RUN = run_name_for(stem, k, cut)
    mto, _p = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, stem)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    returns = gw.scan_returns(data, poses, base_scan, mto, r_max)
    t_read = time.monotonic() - t0

    # Karto clips the threshold into the sensor's own range (Karto.h:3949).
    range_threshold = min(max(r_max, geom['range_min']), geom['range_max'])
    ranges, filled = relay_fill(data['ranges'], geom['range_max'],
                                range_threshold)
    stat_scan = stationary_scans(returns['sensor_x'], returns['sensor_y'],
                                 returns['sensor_yaw'], poses[3])
    reach = reaching_grid(poses[3])
    beam = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']

    return {'graph': graph, 'grid': grid, 'rho': rho, 'origin': origin,
            'geom': geom, 'data': data, 'poses': poses, 'returns': returns,
            'ranges': ranges, 'filled': filled, 'stat_scan': stat_scan,
            'reach': reach, 'range_threshold': range_threshold, 'beam': beam,
            'r_max': r_max, 'r_max_source': r_max_src, 't_read': t_read}


def admitted_rays(stem: str, k: int, cut: int) -> dict:
    """The rays that built this map: origin, unit direction, free length.

    SPEC_b2_divergence_walls §4's ray set, and built from scan_inputs so the
    admission is rebuild()'s rather than a second opinion. Out, in order: a scan
    with no pose (the first-scan drop, graph_walls.interpolate_poses), scans 2-4
    (reaching_grid), and a beam Karto ignores outright (Karto.h:6170). A beam at
    or above the threshold is kept but traced only to the threshold
    (Karto.h:6173), which is what the relay ceiling means for a no-return beam.

    `free_len` is therefore the length Karto CLEARS along the ray, which is the
    quantity T2's condition 3 compares against `s + eps`. `hit` is Karto's
    isEndPointValid: whether the beam ended on a cell it marked occupied.
    """
    si = scan_inputs(stem, k, cut)
    geom, beam, rt = si['geom'], si['beam'], si['range_threshold']
    returns = si['returns']
    sel = np.nonzero(si['reach'])[0]

    r = si['ranges'][sel]
    use = (~np.isnan(r)) & (r > geom['range_min']) & (r < geom['range_max'])
    sidx, bidx = np.nonzero(use)
    ang = returns['sensor_yaw'][sel][sidx] + beam[bidx]
    raw = r[sidx, bidx]
    return {
        'ox': returns['sensor_x'][sel][sidx],
        'oy': returns['sensor_y'][sel][sidx],
        'dx': np.cos(ang), 'dy': np.sin(ang),
        'free_len': np.minimum(raw, rt),
        'hit': raw < rt - KT_TOLERANCE,
        'scan': sel[sidx],
        'n_rays': int(raw.size),
        'range_threshold': rt,
        'rho': si['rho'], 'origin': si['origin'], 'shape': si['grid'].shape,
    }


def rebuild(stem: str, k: int, cut: int, chunk_rays: int = 40000) -> dict:
    """One map's hit and pass counts, per source, by Karto's rules.

    Passes go into four buckets -- finite or relay-filled, crossed while moving
    or while stationary -- plus a fifth that counts the finite passes crossing at
    grazing incidence. Hits go into two, and only finite rays can produce one.
    Every gate, break and counterfactual downstream is then arithmetic on these
    buckets: no ray is traced twice.
    """
    si = scan_inputs(stem, k, cut)
    graph, grid = si['graph'], si['grid']
    rho, origin = si['rho'], si['origin']
    h, w = grid.shape
    geom, data = si['geom'], si['data']
    poses, returns = si['poses'], si['returns']
    ranges, filled = si['ranges'], si['filled']
    stat_scan, reach = si['stat_scan'], si['reach']
    range_threshold, beam = si['range_threshold'], si['beam']
    r_max_src, t_read = si['r_max_source'], si['t_read']

    gx_n, gy_n, _d = nearest_face_normals(grid.shape, origin, rho)
    cos_graze = math.cos(math.radians(GRAZE_DEG))

    n_cells = h * w
    passes = {key: np.zeros(n_cells, dtype=np.int64)
              for key in ('finite_moving', 'finite_still',
                          'filled_moving', 'filled_still', 'finite_grazing')}
    hits = {key: np.zeros(n_cells, dtype=np.int64)
            for key in ('moving', 'still', 'grazing')}

    scan_idx = np.nonzero(reach)[0]
    t1 = time.monotonic()
    n_rays = 0
    for lo in range(0, scan_idx.size, max(1, chunk_rays // geom['n_beams'])):
        sel = scan_idx[lo:lo + max(1, chunk_rays // geom['n_beams'])]
        r = ranges[sel]                                   # (n, beams)
        ox = returns['sensor_x'][sel][:, None]
        oy = returns['sensor_y'][sel][:, None]
        ang = returns['sensor_yaw'][sel][:, None] + beam[None, :]
        px = ox + r * np.cos(ang)
        py = oy + r * np.sin(ang)

        # Karto.h:6170 -- these are ignored outright, no pass and no hit.
        use = (~np.isnan(r)) & (r > geom['range_min']) & (r < geom['range_max'])
        # Karto.h:6173 -- at or above the threshold: rescale, trace free-only.
        over = use & (r >= range_threshold)
        ratio = np.where(over, range_threshold / np.where(over, r, 1.0), 1.0)
        px = np.where(over, ox + ratio * (px - ox), px)
        py = np.where(over, oy + ratio * (py - oy), py)
        valid_end = use & (r < range_threshold - KT_TOLERANCE)

        si, bi = np.nonzero(use)
        if si.size == 0:
            continue
        n_rays += si.size
        gx0, gy0 = world_to_grid(np.broadcast_to(ox, r.shape)[si, bi],
                                 np.broadcast_to(oy, r.shape)[si, bi],
                                 origin, rho)
        gx1, gy1 = world_to_grid(px[si, bi], py[si, bi], origin, rho)
        is_filled = filled[sel][si, bi]
        is_still = stat_scan[sel][si]
        is_valid_end = valid_end[si, bi]
        dirx = np.cos(ang[si, bi])
        diry = np.sin(ang[si, bi])

        cx, cy, ray = trace_rays(gx0, gy0, gx1, gy1)
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        cx, cy, ray = cx[inside], cy[inside], ray[inside]
        flat = cy * w + cx

        # incidence at each traced cell: the angle between the ray coming BACK
        # toward the sensor and that cell's nearest face normal, so 0 deg is
        # head-on and 90 deg is grazing -- §5's own convention.
        cos_psi = (-dirx[ray]) * gx_n[cy, cx] + (-diry[ray]) * gy_n[cy, cx]
        grazing = cos_psi < cos_graze

        f_ray, s_ray = ~is_filled[ray], is_still[ray]
        for key, mask in (('finite_moving', f_ray & ~s_ray),
                          ('finite_still', f_ray & s_ray),
                          ('filled_moving', ~f_ray & ~s_ray),
                          ('filled_still', ~f_ray & s_ray),
                          ('finite_grazing', f_ray & grazing)):
            if mask.any():
                passes[key] += np.bincount(flat[mask], minlength=n_cells)

        # the endpoint's second pass and its hit (Karto.h:6219-6228)
        end_ok = is_valid_end & (gx1 >= 0) & (gx1 < w) & (gy1 >= 0) & (gy1 < h)
        if end_ok.any():
            eflat = gy1[end_ok] * w + gx1[end_ok]
            e_still = is_still[end_ok]
            e_graze = ((-dirx[end_ok]) * gx_n[gy1[end_ok], gx1[end_ok]]
                       + (-diry[end_ok]) * gy_n[gy1[end_ok], gx1[end_ok]]
                       ) < cos_graze
            for key, mask in (('finite_moving', ~e_still),
                              ('finite_still', e_still),
                              ('finite_grazing', e_graze)):
                if mask.any():
                    passes[key] += np.bincount(eflat[mask], minlength=n_cells)
            for key, mask in (('moving', ~e_still), ('still', e_still),
                              ('grazing', e_graze)):
                if mask.any():
                    hits[key] += np.bincount(eflat[mask], minlength=n_cells)
    t_trace = time.monotonic() - t1

    return {
        'stem': stem, 'robot': k, 'cut': cut,
        'graph': graph, 'grid': grid, 'rho': rho, 'origin': origin,
        'shape': grid.shape, 'returns': returns, 'poses': poses,
        'passes': {key: v.reshape(h, w) for key, v in passes.items()},
        'hits': {key: v.reshape(h, w) for key, v in hits.items()},
        'normals': (gx_n, gy_n),
        'range_threshold': range_threshold, 'r_max_source': r_max_src,
        'n_scans_read': int(data['stamps'].size),
        'n_scans_pose_valid': int(np.count_nonzero(poses[3])),
        'n_scans_reaching_grid': int(np.count_nonzero(reach)),
        'n_scans_stationary': int(np.count_nonzero(stat_scan & reach)),
        'n_rays_traced': int(n_rays),
        'seconds': {'read': t_read, 'trace': t_trace},
    }


def states_from(rb: dict, drop: tuple = (), threshold: float = None,
                min_pass: int = None) -> np.ndarray:
    """The rebuilt grid with some sources' passes (and hits) removed.

    `drop` names pass buckets to leave out; a dropped bucket's hits go with it,
    because removing a ray means removing everything it contributed.
    """
    threshold = OCCUPANCY_THRESHOLD if threshold is None else threshold
    min_pass = MIN_PASS_THROUGH if min_pass is None else min_pass
    keys = ('finite_moving', 'finite_still', 'filled_moving', 'filled_still')
    total = sum(rb['passes'][key] for key in keys if key not in drop)
    hit = np.zeros_like(total)
    if 'finite_moving' not in drop:
        hit = hit + rb['hits']['moving']
    if 'finite_still' not in drop:
        hit = hit + rb['hits']['still']
    if 'finite_grazing' in drop:
        total = total - rb['passes']['finite_grazing']
        hit = hit - rb['hits']['grazing']
    return update_cell(total, hit, min_pass, threshold)


def verify_tracer(n: int = 4000, span: int = 60, seed: int = 0) -> dict:
    """Pin the vectorised tracer against the transcribed one. Refuses on any miss.

    Runs before anything is rebuilt, so "copied exactly" is a result of the run
    rather than a claim in a docstring. Random rays include the degenerate cases
    that break naive Bresenham ports: single cells, axis-aligned rays, exact
    diagonals, and both orders of both swaps.
    """
    rng = np.random.default_rng(seed)
    x0 = rng.integers(-span, span, n)
    y0 = rng.integers(-span, span, n)
    x1 = rng.integers(-span, span, n)
    y1 = rng.integers(-span, span, n)
    # force the degenerate cases into the sample rather than hoping for them
    x0[:6] = [0, 0, 0, 5, -3, 7]
    y0[:6] = [0, 0, 5, 0, -3, 7]
    x1[:6] = [0, 9, 0, 5, 4, -7]
    y1[:6] = [0, 0, 9, 9, 4, -7]

    cx, cy, ray = trace_rays(x0, y0, x1, y1)
    got = {}
    for r, a, b in zip(ray.tolist(), cx.tolist(), cy.tolist()):
        got.setdefault(r, []).append((a, b))
    n_cells = 0
    for r in range(n):
        want = trace_line_reference(int(x0[r]), int(y0[r]), int(x1[r]),
                                    int(y1[r]))
        if got.get(r, []) != want:
            gw.die('the vectorised tracer disagrees with the transcribed '
                   f'Karto loop on ray {r}: '
                   f'({x0[r]},{y0[r]})->({x1[r]},{y1[r]}); '
                   f'got {got.get(r, [])[:6]} want {want[:6]}')
        n_cells += len(want)
    return {'rays': n, 'cells': n_cells, 'seed': seed,
            'reference': 'trace_line_reference, transcribed from Karto.h:'
                         '4874-4926',
            'agree': True}


def off_wall_cells(rb: dict) -> tuple[np.ndarray, dict]:
    """The cells B1 counts as off-wall, as a flat index array.

    B1's own machinery, imported from graph_walls, so this is the same set Part B
    and diagnose_dwell reported on -- not a third definition of it.
    """
    b1 = gw.on_occupied_share(rb['returns'], rb['grid'], rb['rho'],
                              rb['origin'])
    h, w = rb['shape']
    off = ~b1['hit']
    # a return's cell, by KARTO's rule, since that is what the rebuild indexes by
    cx, cy = world_to_grid(rb['returns']['px'], rb['returns']['py'],
                           rb['origin'], rb['rho'])
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    flat_all = (cy * w + cx)[inside]
    flat_off = (cy * w + cx)[off & inside]
    return np.unique(flat_off), {
        'b1_share': b1['share'],
        'n_returns_off_wall': int(np.count_nonzero(off)),
        'n_cells_with_any_return': int(np.unique(flat_all).size),
    }


def free_depth(rb: dict) -> dict:
    """Step 5, report only. How deep into a cell its hits sit.

    For every cell holding a return: the distance along the nearest SDF face
    normal from the cell's FREE-SIDE EDGE (Karto centre + rho/2 along the normal)
    to the median of the hit positions in that cell, projected on the same normal.
    Positive means the hits sit behind the free edge, i.e. inside the cell.
    Off-wall cells are compared against the cells the map did commit as wall.
    """
    h, w = rb['shape']
    nx, ny = rb['normals']
    rho = rb['rho']
    px, py = rb['returns']['px'], rb['returns']['py']
    cx, cy = world_to_grid(px, py, rb['origin'], rho)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    cx, cy, px, py = cx[inside], cy[inside], px[inside], py[inside]
    flat = cy * w + cx
    n_flat = nx[cy, cx] * px + ny[cy, cx] * py          # hit, projected on n
    centre = (nx[cy, cx] * (rb['origin'][0] + cx * rho)
              + ny[cy, cx] * (rb['origin'][1] + cy * rho))
    depth = (centre + rho / 2.0) - n_flat

    order = np.argsort(flat, kind='stable')
    flat_s, depth_s = flat[order], depth[order]
    edges = np.concatenate([[0], np.nonzero(np.diff(flat_s))[0] + 1,
                            [flat_s.size]])
    cells = flat_s[edges[:-1]]
    med = np.array([np.median(depth_s[a:b])
                    for a, b in zip(edges[:-1], edges[1:])])

    off, _stats = off_wall_cells(rb)
    saved = karto_state_of_saved(rb)
    is_off = np.isin(cells, off)
    is_wall = saved.reshape(-1)[cells] == K_OCCUPIED

    def summary(vals):
        if not vals.size:
            return None
        return {'n_cells': int(vals.size),
                'median_m': float(np.median(vals)),
                'p10_m': float(np.percentile(vals, 10)),
                'p90_m': float(np.percentile(vals, 90))}

    return {'definition': 'free-side cell edge (Karto centre + rho/2 along the '
                          'nearest SDF face normal) minus the median hit '
                          'position in that cell, along the same normal; '
                          'positive = hits sit inside the cell',
            'off_wall_cells': summary(med[is_off]),
            'committed_wall_cells': summary(med[is_wall & ~is_off]),
            'all_cells_with_a_return': summary(med)}


def karto_state_of_saved(rb: dict) -> np.ndarray:
    """The saved map's classes as Karto states, indexed [y][x] like the rebuild.

    graph_walls' grid is already bottom-up, so its (row, col) IS Karto's (y, x) --
    the same integers. Only the world position attached to them differs, by the
    half cell this file's docstring reports.
    """
    out = np.full(rb['grid'].shape, K_UNKNOWN, dtype=np.uint8)
    out[rb['grid'] == gw.OCC] = K_OCCUPIED
    out[rb['grid'] == gw.FREE] = K_FREE
    return out


def gate(rb: dict) -> dict:
    """Step 3. Does the rebuild reproduce the map?

    Agreement over OBSERVED cells: those the saved map committed, or the rebuild
    did. Index-based, so the half-cell shift between Karto's geometry and the ROS
    reading cannot flatter it.
    """
    saved = karto_state_of_saved(rb)
    rebuilt = states_from(rb)
    observed = (saved != K_UNKNOWN) | (rebuilt != K_UNKNOWN)
    n_obs = int(np.count_nonzero(observed))
    agree = int(np.count_nonzero((saved == rebuilt) & observed))
    conf = {}
    for a, an in ((K_UNKNOWN, 'unknown'), (K_FREE, 'free'),
                  (K_OCCUPIED, 'occupied')):
        for b, bn in ((K_UNKNOWN, 'unknown'), (K_FREE, 'free'),
                      (K_OCCUPIED, 'occupied')):
            n = int(np.count_nonzero((saved == a) & (rebuilt == b)))
            if n:
                conf[f'saved_{an}__rebuilt_{bn}'] = n
    return {'n_observed_cells': n_obs,
            'n_agree': agree,
            'agreement': (agree / n_obs) if n_obs else float('nan'),
            'limit': GATE_MIN_AGREEMENT,
            'holds': bool(n_obs and agree / n_obs >= GATE_MIN_AGREEMENT),
            'confusion': conf}


def gate_breaks(rb: dict) -> dict:
    """The two breaks step 3 names. Each must fail the gate.

    Arithmetic on the same buckets -- no ray is traced again -- so a break cannot
    differ from the real rebuild in any way other than the one thing it breaks.
    """
    saved = karto_state_of_saved(rb)

    def agreement(rebuilt):
        observed = (saved != K_UNKNOWN) | (rebuilt != K_UNKNOWN)
        n = int(np.count_nonzero(observed))
        return ((int(np.count_nonzero((saved == rebuilt) & observed)) / n)
                if n else float('nan'))

    no_relay = agreement(states_from(rb, drop=('filled_moving',
                                               'filled_still')))
    half = agreement(states_from(rb, threshold=0.5))
    return {
        'drop_relay_filled': {'agreement': no_relay,
                              'fails_gate': bool(no_relay
                                                 < GATE_MIN_AGREEMENT)},
        'occupancy_threshold_0p5': {'agreement': half,
                                    'fails_gate': bool(half
                                                       < GATE_MIN_AGREEMENT)},
    }


def counterfactuals(rb: dict) -> dict:
    """Step 4. Which removal recommits the off-wall cells."""
    off, stats = off_wall_cells(rb)
    base = states_from(rb).reshape(-1)
    out = {'n_off_wall_cells': int(off.size), **stats}
    if not off.size:
        return out
    was_not_occupied = base[off] != K_OCCUPIED
    out['n_off_wall_cells_not_occupied_in_rebuild'] = int(
        np.count_nonzero(was_not_occupied))
    for name, drop in (('a_relay_filled', ('filled_moving', 'filled_still')),
                       ('b_stationary', ('finite_still', 'filled_still')),
                       ('c_finite_grazing', ('finite_grazing',))):
        state = states_from(rb, drop=drop).reshape(-1)[off]
        recommitted = was_not_occupied & (state == K_OCCUPIED)
        out[name] = {
            'dropped_buckets': list(drop),
            'n_recommitted': int(np.count_nonzero(recommitted)),
            'share_recommitted': (float(np.count_nonzero(recommitted))
                                  / max(1, np.count_nonzero(was_not_occupied))),
        }
    return out


# ──────────────────────────────── main ───────────────────────────────────────

def summarise(rb: dict, g: dict, breaks: dict) -> dict:
    """Everything about one map that goes in the JSON. No numpy leaks through."""
    return {
        'map': rb['stem'], 'robot': rb['robot'], 'cut_s': rb['cut'],
        'grid': {'width': int(rb['shape'][1]), 'height': int(rb['shape'][0]),
                 'resolution': rb['rho'], 'origin': list(rb['origin']),
                 'indexing': 'Karto: cell i centred on origin + i*res '
                             '(Karto.h:4421-4436)'},
        'range_threshold': rb['range_threshold'],
        'r_max_source': rb['r_max_source'],
        'scans': {'read': rb['n_scans_read'],
                  'pose_valid': rb['n_scans_pose_valid'],
                  'reaching_grid': rb['n_scans_reaching_grid'],
                  'stationary_reaching_grid': rb['n_scans_stationary'],
                  'dropped_by_stabilisation': list(STABILISATION_DROP),
                  'rule': 'every pose-valid scan except the 2nd, 3rd and 4th '
                          '(slam_toolbox_common.cpp:795-797); Karto rejects '
                          'none, HasMovedEnough passing on the time test with '
                          'minimum_time_interval 0.0 (Mapper.cpp:3155-3158)'},
        'rays_traced': rb['n_rays_traced'],
        'passes_total': {k: int(v.sum()) for k, v in rb['passes'].items()},
        'hits_total': {k: int(v.sum()) for k, v in rb['hits'].items()},
        'gate': g,
        'gate_breaks': breaks,
        'seconds': rb['seconds'],
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps',
                    help='one map stem; repeatable. Default: all 20 b2maps cuts')
    args = ap.parse_args()
    stems = args.maps or gw.corpus_stems()

    print("rebuilding Karto's per-cell hit and pass counts, and splitting the "
          'passes by source')
    print(f'  rules read at {SOURCE_TREE}')
    print(f'  orig tarball SHA256 {SOURCE_SHA256}')
    print(f'  min_pass_through {MIN_PASS_THROUGH} (slam_mapper.cpp:360), '
          f'occupancy_threshold {OCCUPANCY_THRESHOLD} (:367) -- the replay '
          'config sets neither')
    print('  cells indexed by Karto: centre at origin + i*res '
          '(Karto.h:4421-4436)')
    print()
    print('PRE-REGISTERED, printed before any number is read:')
    print(f'  gate     cell-state agreement >= {GATE_MIN_AGREEMENT:.1%} of '
          'observed cells, on every map,')
    print('           and both named breaks must FAIL it')
    for name, text in HYPOTHESES:
        print(f'  {name}  {text}')
    print()

    tracer = verify_tracer()
    print(f'tracer: the vectorised version agrees with the transcribed Karto '
          f'loop on all {tracer["rays"]} random rays ({tracer["cells"]} cells)')
    print()

    rows, rebuilt = {}, {}
    print(f'{"map":<30}{"scans":>7}{"->grid":>8}{"rays":>10}{"agree":>9}'
          f'{"no-relay":>10}{"thr 0.5":>9}{"read s":>8}{"trace s":>9}')
    for stem in stems:
        k, cut = gw._k_and_cut(stem, 'the rebuild needs the bag and the '
                                     'replay config')
        rb = rebuild(stem, k, cut)
        g = gate(rb)
        br = gate_breaks(rb)
        rows[stem] = summarise(rb, g, br)
        rebuilt[stem] = rb
        print(f'{stem:<30}{rb["n_scans_read"]:>7}'
              f'{rb["n_scans_reaching_grid"]:>8}{rb["n_rays_traced"]:>10}'
              f'{g["agreement"]:>8.2%}'
              f'{br["drop_relay_filled"]["agreement"]:>10.2%}'
              f'{br["occupancy_threshold_0p5"]["agreement"]:>9.2%}'
              f'{rb["seconds"]["read"]:>8.1f}{rb["seconds"]["trace"]:>9.1f}'
              + ('' if g['holds'] else '  <- GATE FAILED'))
    print()

    failed = [s for s, r in rows.items() if not r['gate']['holds']]
    breaks_ok = all(r['gate_breaks']['drop_relay_filled']['fails_gate']
                    and r['gate_breaks']['occupancy_threshold_0p5']['fails_gate']
                    for r in rows.values())
    print(f'GATE: {len(rows) - len(failed)}/{len(rows)} maps at or above '
          f'{GATE_MIN_AGREEMENT:.1%}')
    print(f'      both breaks fail the gate on every map: '
          f'{"yes" if breaks_ok else "NO"}')

    out = {
        'question': "does a rebuild of Karto's own hit and pass counts show "
                    'which source cleared the wall cells',
        'provenance': {
            'source_tree': str(SOURCE_TREE),
            'package': 'ros-jazzy-slam-toolbox 2.8.5',
            'orig_tarball_sha256': SOURCE_SHA256,
            'copied_into_repo': False,
            'rules': {
                'line_tracer': 'Karto.h:4874-4926 Grid<T>::TraceLine',
                'pass_vs_hit': 'Karto.h:6202-6236 OccupancyGrid::RayTrace; the '
                               'endpoint of a valid ray gets a second pass and '
                               'a hit at :6226-6227',
                'clipping': 'Karto.h:6148-6192 OccupancyGrid::AddScan; '
                            'isEndPointValid at :6167; ignored readings at '
                            ':6170; free-only rescale at :6173; threshold clip '
                            'at Karto.h:3949; KT_TOLERANCE 1e-06 at Math.h:41',
                'hit_point': 'Karto.h:5640-5665 LocalizedRangeScan::Update; '
                             'AddScan reads the unfiltered readings at :6157',
                'thresholds': 'Karto.h:6244-6256 UpdateCell, :6260-6277 Update, '
                              ':4612 Grid::Clear, :4379-4382 GridStates',
                'values': 'src/slam_mapper.cpp:360-372 (min_pass_through 2, '
                          'occupancy_threshold 0.1), passed at :67-69 with '
                          'GetAllProcessedScans()',
                'which_scans': 'karto_sdk/src/Mapper.cpp:3147-3178 '
                               'HasMovedEnough (returns true on the time test); '
                               'src/slam_toolbox_common.cpp:756-816 '
                               'shouldProcessScan (scan_ctr < 5 at :795-797; '
                               'throttle_scans default 1 at :366-370)',
                'cell_indexing': 'Karto.h:4421-4436 WorldToGrid, Round not '
                                 'floor',
                'published_origin': 'include/slam_toolbox/'
                                    'visualization_utils.hpp:108-129 toNavMap '
                                    'copies the Karto grid offset verbatim',
            },
            'half_cell_shift': 'Karto centres cell i on origin + i*res; the '
                               'ROS convention reads it as origin + '
                               '(i+0.5)*res, and toNavMap publishes the offset '
                               'unchanged, so a ROS reading of a slam_toolbox '
                               'map sat half a cell (0.05 m here) further +x '
                               'and +y than Karto placed it. NOW FIXED: '
                               'trinary_map.CELL_CENTRE_OFFSET is 0.0 and this '
                               'file routes through it. Nothing here moved -- '
                               'it always indexed Karto\'s way -- and the gate '
                               'is index-based in any case',
            'cell_centre': cell_centre_provenance(),
            'values_used': {'min_pass_through': MIN_PASS_THROUGH,
                            'occupancy_threshold': OCCUPANCY_THRESHOLD,
                            'kt_tolerance': KT_TOLERANCE,
                            'graze_deg': GRAZE_DEG,
                            'stationary_test': {'m': STILL_M,
                                                'deg': STILL_DEG}},
        },
        'tracer_check': tracer,
        'gate': {'limit': GATE_MIN_AGREEMENT,
                 'maps_passing': len(rows) - len(failed),
                 'maps': len(rows),
                 'failed_maps': failed,
                 'both_breaks_fail_everywhere': breaks_ok},
        'hypotheses': {name: text for name, text in HYPOTHESES},
        'truth_used_for': 'the grazing counterfactual (c) and the free-depth '
                          'report only; both stay in this file',
        'per_map': rows,
    }

    if failed:
        print()
        print('The gate failed, so step 4 was NOT run: counterfactuals on a '
              'rebuild that does not')
        print('  reproduce the map would describe the rebuild, not the map.')
        OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
        print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
        return 2

    # ── step 4 and step 5, only once the gate holds ──────────────────────────
    print()
    print(f'{"map":<30}{"off cells":>11}{"not occ":>9}'
          f'{"(a) relay":>11}{"(b) still":>11}{"(c) graze":>11}')
    for stem, rb in rebuilt.items():
        cf = counterfactuals(rb)
        fd = free_depth(rb)
        rows[stem]['counterfactuals'] = cf
        rows[stem]['free_depth'] = fd
        if cf['n_off_wall_cells']:
            print(f'{stem:<30}{cf["n_off_wall_cells"]:>11}'
                  f'{cf["n_off_wall_cells_not_occupied_in_rebuild"]:>9}'
                  f'{cf["a_relay_filled"]["share_recommitted"]:>11.1%}'
                  f'{cf["b_stationary"]["share_recommitted"]:>11.1%}'
                  f'{cf["c_finite_grazing"]["share_recommitted"]:>11.1%}')
        else:
            print(f'{stem:<30}{0:>11}{"-":>9}{"-":>11}{"-":>11}{"-":>11}')
    print()

    b1_failures = [s for s, rb in rebuilt.items()
                   if rows[s]['counterfactuals'].get('b1_share', 1.0)
                   < gw.B1_MIN_SHARE]
    print('HYPOTHESES, read off the numbers above. Reported, never gated.')
    verdicts = {}
    for name, _text in HYPOTHESES:
        key = {'H_relay': 'a_relay_filled', 'H_graze': 'c_finite_grazing',
               'H_dwell': 'b_stationary'}[name]
        per = {}
        for s in b1_failures:
            share = rows[s]['counterfactuals'][key]['share_recommitted']
            if name == 'H_dwell':
                want = (share >= 0.5 if 'cut1200' in s else share < 0.10)
            else:
                want = share >= 0.5
            per[s] = {'share': share, 'holds': bool(want)}
        held = all(v['holds'] for v in per.values()) if per else False
        verdicts[name] = {'held': held, 'per_map': per}
        print(f'  {name}  {"HELD" if held else "FAILED"}')
        for s, v in per.items():
            print(f'        {s}: {v["share"]:.1%} recommitted -> '
                  f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    out['verdicts'] = verdicts
    out['b1_failures'] = b1_failures

    print()
    print('FREE DEPTH (step 5, report only) -- how far behind a cell\'s '
          'free-side edge its hits sit')
    for stem in rebuilt:
        fd = rows[stem]['free_depth']
        off, wall = fd['off_wall_cells'], fd['committed_wall_cells']
        if off and wall:
            print(f'  {stem:<30} off-wall median {off["median_m"]:+.4f} m '
                  f'({off["n_cells"]} cells)   committed wall '
                  f'{wall["median_m"]:+.4f} m ({wall["n_cells"]} cells)')
    print()

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
