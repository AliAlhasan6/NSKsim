#!/usr/bin/env python3
"""Does Karto's grid stop short of its own extent, and does that lose walls?

A DIAGNOSIS of the candidate SPEC_b2_wall_predicate §12.3 reclassified §12's four
B1 failures under. It writes one file,
experiments/logs/graph_walls/diagnose_extent.json, and touches no map, no map JSON
and no threshold.

THE HYPOTHESIS, in the terms the source puts it in. Every number below comes from
~/src/slam_toolbox_2.8.5 (ros-jazzy-slam-toolbox 2.8.5, orig tarball SHA256
80af3955...608583), never copied into this repo; Karto.h means
lib/karto_sdk/include/karto_sdk/Karto.h.

  the dimensions   OccupancyGrid::ComputeDimensions, Karto.h:6089-6113. It takes
                   the union of the processed scans' bounding boxes (:6097-6105),
                   then
                       rWidth  = Round(size.GetWidth()  * scale)   (:6111)
                       rHeight = Round(size.GetHeight() * scale)   (:6112)
                       rOffset = boundingBox.GetMinimum()          (:6113)
                   with scale = 1/resolution (:6108). math::Round is
                   `v >= 0 ? floor(v + 0.5) : ceil(v - 0.5)` (Math.h:87-90).
  NOTHING PADS IT  the static CreateFromScans (Karto.h:5947-5964) passes those
                   three straight to the OccupancyGrid constructor
                   (:5910-5930) -- no border, no margin, no rounding up.
                   Grid::CreateGrid (:4586-4594) takes width, height and
                   resolution and nothing else; Karto has no borderSize here.
                   The ONE thing that looks like padding is not addressable:
                   Grid::Resize (:4636-4664) sets
                   m_WidthStep = AlignValue(width, 8) (:4640, Math.h:234-237),
                   an 8-aligned ROW STRIDE for the data array, while m_Width
                   stays Round(size*scale) and IsValidGridIndex tests against
                   m_Width (:4671-4674). The extra columns exist in memory and
                   cannot be indexed.
  the shortfall    WorldToGrid rounds about the offset (Karto.h:4421-4436), so a
                   grid of W cells covers world x in
                       [O - res/2, O + (W - 0.5)*res)
                   while the scans span [O, O + size]. Writing u = size/res and
                   W = Round(u), the far-edge shortfall is
                       u - (W - 0.5) = u - Round(u) + 0.5   cells,
                   which lies in [0, 1) for every u: under its own indexing
                   Karto's grid ALWAYS stops short of its own bounding box, by
                   between nothing and a whole cell. A hit in that band rounds to
                   index W and is skipped by TraceLine (Karto.h:4874-4926, "cells
                   outside the grid are skipped"); the passes along the same ray
                   stop at W-1 and are kept. Passes without hits is Karto's FREE
                   (UpdateCell, :6244-6256).
                   The same class contradicts itself about which semantics the
                   offset has: CoordinateConverter::GetBoundingBox (:4536-4548)
                   reads it as the grid's minimum CORNER, maxX = minX + W*res.

WHAT IS MEASURED, and X1/X2 are printed before any of it.

  1  per map, the recomputed Karto extent against the saved one, and the far-edge
     shortfall in cells in x and in y. All 20 ungated and all 20 gated maps. The
     recomputation is predict_gated_maps.karto_extent, imported unchanged -- the
     function whose --check reproduced every saved lattice exactly -- and a map
     whose origin, width or height it cannot reproduce is REFUSED rather than
     reported, because then the shortfall is not this map's.
  2  per map, A9 recall split by boundary face: the inward faces of wall_east and
     wall_north (the max-x and max-y side) against those of wall_west and
     wall_south (the min side). Truth-derived, diagnosis only, exactly as A9 is.
  3  per map, per boundary face, the share of the returns that struck that face
     which fell off the grid.

Parts 2 and 3 need Part B's chain, so they resolve the gated bag, config and
map->odom log with LOCAL helpers (variant_* below). That is deliberately not the
same change as threading the variant through graph_walls.obs_for_map's three
builders, which is its own task; nothing here is imported by Part B and the relay
log (B3) is not read at all.

TRUTH. The SDF faces are truth, used for parts 2 and 3 and for X1. Diagnosis only:
§6 forbids any later tool from reading A8-A11, and the same rule is applied here --
none of this goes near a <map>.json.

    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/NSKsim/install/setup.bash
    python3 experiments/analysis/diagnose_extent.py --map b2maps_k1_cut1200_robot1
    python3 experiments/analysis/diagnose_extent.py

One map first, then the corpus. No simulator. Blocking.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import diagnose_karto as dk                                  # noqa: E402
import graph_walls as gw                                     # noqa: E402
import predict_gated_maps as pg                              # noqa: E402
from trinary_map import cell_centre_provenance, cell_index    # noqa: E402

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_extent.json'

# A return "struck" a face when it is within this of it, the same 2*rho the A8-A11
# pass uses for "on a wall" (TRUTH_TOL_CELLS). Not a new threshold.
STRIKE_TOL_CELLS = gw.TRUTH_TOL_CELLS

# The four boundary models, and which of their faces looks into the arena. Named,
# not derived: the hypothesis is specifically about the max-x and max-y sides.
MAX_SIDE = (('wall_east', 'west'), ('wall_north', 'south'))
MIN_SIDE = (('wall_west', 'east'), ('wall_south', 'north'))

PREDICTIONS = [
    ('X1', 'on every map, at least 95 % of off-grid returns lie within '
           f'{STRIKE_TOL_CELLS:g}*rho of an SDF face; and on the 1200 s maps, at '
           'least 95 % lie on the east or north boundary face'),
    ('X2', 'among maps where all four boundary faces were observed, the max-side '
           'mean A9 recall is lower than the min-side mean on at least 80 % of '
           'them'),
]
X1_MIN = 0.95
X2_MIN_SHARE = 0.80


# ────────────────────── the gated/ungated input paths ────────────────────────
# Local to this file. graph_walls derives these from (robot, cut) alone and
# refuses a _gated stem for exactly that reason; threading the variant through
# Part B is a separate change and is not made here.

def variant_of(stem: str) -> str:
    """'' for an ungated stem, '_gated' for a gated one."""
    return '_gated' if '_gated' in stem else ''


def variant_run(k: int, cut: int, variant: str) -> str:
    return f'{gw.RUN_PREFIX}_k{k}_cut{cut}{variant}'


def variant_bag(k: int, cut: int, variant: str) -> Path:
    path = gw.BAGS_DIR / f'{variant_run(k, cut, variant)}_slamin'
    if not path.is_dir():
        gw.die(f'replay bag not found: {path}')
    return path


def variant_r_max(k: int, cut: int, variant: str) -> tuple[float, str]:
    """`max_laser_range` from this variant's own replay config, read not assumed.

    graph_walls.read_replay_r_max with the variant in the name; the regex and the
    refusal are the same.
    """
    path = (gw.LOGS_DIR / f'offline_mapping_{variant_run(k, cut, variant)}'
            f'_robot_{k}.yaml')
    if not path.is_file():
        gw.die(f'replay config not found, so r_max cannot be read: {path}')
    for i, line in enumerate(path.read_text().splitlines(), 1):
        m = re.match(r'^\s*max_laser_range:\s*([0-9.eE+-]+)\s*$',
                     line.split('#', 1)[0])
        if m:
            return float(m.group(1)), f'{path.name}:{i}'
    gw.die(f'no max_laser_range in {path}')


def variant_map_to_odom(k: int, cut: int, variant: str):
    """map->odom for this variant, through the fitter's own parser.

    Refused unless identity, as robot_divergence does: a non-identity map_T_odom
    makes the placement ambiguous and this file would be guessing.
    """
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415
    fwt.RUN = variant_run(k, cut, variant)
    mto, path = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, variant_run(k, cut, variant))
    return mto, path


# ──────────────────────────────── part 1 ─────────────────────────────────────

def shortfall(stem: str) -> dict:
    """Karto's extent recomputed, against the saved one, with the shortfall.

    `karto_extent` is imported from predict_gated_maps unchanged: it is the
    function whose extent self-check reproduced every saved origin, width and
    height exactly, so it is the one thing here that does not need re-arguing.
    """
    k, cut = gw._k_and_cut(stem, 'the extent needs the robot and the cut')
    variant = variant_of(stem)
    m = gw.load_grid(stem)
    rho = m['rho']
    r_max, r_max_src = variant_r_max(k, cut, variant)
    data = gw.read_bag(variant_bag(k, cut, variant), k)
    geom = data['geom']
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    mto, mto_path = variant_map_to_odom(k, cut, variant)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    returns = gw.scan_returns(data, poses, base_scan, mto, r_max)
    sx, sy, syaw = (returns['sensor_x'], returns['sensor_y'],
                    returns['sensor_yaw'])
    range_threshold = min(max(r_max, geom['range_min']), geom['range_max'])
    ranges, _filled = dk.relay_fill(data['ranges'], geom['range_max'],
                                    range_threshold)
    beam = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']
    sel = dk.reaching_grid(poses[3])

    (ox, oy), w, h = pg.karto_extent(sx, sy, ranges, beam, syaw, sel, geom,
                                     rho, range_threshold)
    # karto_extent returns the bounding box MINIMUM and a ROUNDED size, which
    # throws away the fraction the shortfall is made of, so the far corner is
    # walked separately rather than changing an imported function the extent
    # self-check already validated.
    hi_x, hi_y = _far_corner(sx, sy, ranges, beam, syaw, sel, geom,
                             range_threshold)
    span_x = (hi_x - ox) / rho
    span_y = (hi_y - oy) / rho

    out = {
        'map': stem, 'robot': k, 'cut_s': cut,
        'variant': 'gated' if variant else 'ungated',
        'resolution': rho,
        'bag': str(variant_bag(k, cut, variant).relative_to(REPO_ROOT)),
        'r_max': {'value': r_max, 'source': r_max_src},
        'map_to_odom_log': str(Path(mto_path).relative_to(REPO_ROOT)),
        'n_scans_read': int(data['stamps'].size),
        'n_scans_reaching_grid': int(np.count_nonzero(sel)),
        'saved': {'origin': [m['origin'][0], m['origin'][1]],
                  'width': m['width'], 'height': m['height']},
        'recomputed': {'origin': [float(ox), float(oy)],
                       'width': int(w), 'height': int(h)},
        'origin_dx': float(ox - m['origin'][0]),
        'origin_dy': float(oy - m['origin'][1]),
        'span_cells': [float(span_x), float(span_y)],
        # u - Round(u) + 0.5, in [0, 1): how far the data runs past the outer
        # edge of the last cell, under Karto's own centre semantics.
        'shortfall_cells_x': float(span_x - (w - 0.5)),
        'shortfall_cells_y': float(span_y - (h - 0.5)),
        'shortfall_m_x': float((span_x - (w - 0.5)) * rho),
        'shortfall_m_y': float((span_y - (h - 0.5)) * rho),
    }
    out['lattice_reproduced'] = bool(
        w == m['width'] and h == m['height']
        and abs(out['origin_dx']) < 1e-9 and abs(out['origin_dy']) < 1e-9)
    out['_returns'] = returns
    out['_grid_shape'] = m['grid'].shape
    out['_origin'] = m['origin']
    out['_rho'] = rho
    return out


def _far_corner(sx, sy, ranges, beam, syaw, sel, geom, range_threshold):
    """The bounding box MAXIMUM, by ComputeDimensions' own rule.

    karto_extent returns the minimum and a rounded size, which throws away the
    fraction the shortfall is made of. This walks the same scans and the same
    filtered readings (minRange <= r <= rangeThreshold, Karto.h:5694-5700 with
    Math.h:172-175) and keeps the other corner.
    """
    hi_x = hi_y = -np.inf
    for i in np.nonzero(sel)[0]:
        r = ranges[i]
        keep = (~np.isnan(r)) & (r >= geom['range_min']) & (r <= range_threshold)
        hi_x = max(hi_x, sx[i])
        hi_y = max(hi_y, sy[i])
        if keep.any():
            ang = syaw[i] + beam[keep]
            hi_x = max(hi_x, float((sx[i] + r[keep] * np.cos(ang)).max()))
            hi_y = max(hi_y, float((sy[i] + r[keep] * np.sin(ang)).max()))
    return hi_x, hi_y


# ──────────────────────────────── part 2 ─────────────────────────────────────

def boundary_face_indices(world: dict) -> dict:
    """{(model, side): index into world['inward']} for the four boundary faces."""
    want = dict.fromkeys(MAX_SIDE + MIN_SIDE)
    out = {}
    for i, f in enumerate(world['inward']):
        key = (f['model'], f['side'])
        if key in want:
            out[key] = i
    missing = [k for k in want if k not in out]
    if missing:
        gw.die(f'these boundary faces are not among the inward faces: {missing}. '
               "A9's denominator is defined in terms of them (C6), so either the "
               'SDF or sdf_wall_faces has changed.')
    return out


def recall_by_face(graph: dict, spawn, world: dict, samples: dict,
                   rho: float) -> dict:
    """A9's covered-length share, per inward face.

    The covering test is graph_walls.validate_map's, element for element: a sample
    is covered when some segment's line passes within 2*rho of it AND that
    segment's normal agrees with the face's by more than A9_MATCH_DOT. Weighted by
    the metre of face each sample stands for, so this is a LENGTH share.
    """
    tol = gw.TRUTH_TOL_CELLS * rho
    seg = gw._segment_arrays(graph, spawn)
    covered = np.zeros(samples['x'].size, dtype=bool)
    for i in range(len(graph['segments'])):
        dot = samples['nx'] * seg['nx'][i] + samples['ny'] * seg['ny'][i]
        near = gw.point_segment_distance(samples['x'], samples['y'], seg, i) <= tol
        covered |= near & (dot > gw.A9_MATCH_DOT)
    out = {}
    for fi, f in enumerate(world['inward']):
        pick = samples['face'] == fi
        w = float(samples['w'][pick].sum())
        out[f'{f["model"]}.{f["side"]}'] = {
            'length_m': w,
            'covered_m': float(samples['w'][pick & covered].sum()),
            'recall': (float(samples['w'][pick & covered].sum()) / w) if w else None,
        }
    return out


# ──────────────────────────────── part 3 ─────────────────────────────────────

def face_strike_offgrid(returns: dict, origin, rho: float, shape,
                        spawn, world: dict) -> dict:
    """Per inward face: the returns that struck it, and how many fell off the grid.

    A return strikes the face whose surface is nearest to it, if that distance is
    within 2*rho. Nearest-face attribution so one return is counted once; the
    alternative (every face within tolerance) would double-count the returns at a
    corner and make the per-face shares unreadable.

    `off the grid` is the Karto cell index falling outside [0, W) x [0, H) -- the
    same test B1's denominator uses, and the same one TraceLine applies.
    """
    h, w = shape
    col = cell_index(returns['px'], origin[0], rho)
    row = cell_index(returns['py'], origin[1], rho)
    inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)

    wx = returns['px'] + spawn[0]
    wy = returns['py'] + spawn[1]
    best = np.full(wx.size, np.inf)
    owner = np.full(wx.size, -1, dtype=np.int64)
    for fi, f in enumerate(world['inward']):
        ax, ay = f['p0']
        bx, by = f['p1']
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        t = np.clip(((wx - ax) * vx + (wy - ay) * vy) / vv, 0.0, 1.0)
        d = np.hypot(wx - (ax + t * vx), wy - (ay + t * vy))
        closer = d < best
        best = np.where(closer, d, best)
        owner = np.where(closer, fi, owner)

    tol = STRIKE_TOL_CELLS * rho
    struck = best <= tol
    out = {}
    for fi, f in enumerate(world['inward']):
        pick = struck & (owner == fi)
        n = int(np.count_nonzero(pick))
        n_off = int(np.count_nonzero(pick & ~inside))
        out[f'{f["model"]}.{f["side"]}'] = {
            'n_returns_struck': n,
            'n_off_grid': n_off,
            'share_off_grid': (float(n_off) / n) if n else None,
            'observed': bool(n),
        }
    # X1's population: every off-grid return, wherever it landed.
    off = ~inside
    n_off_total = int(np.count_nonzero(off))
    bnd = boundary_face_indices(world)
    max_idx = [bnd[key] for key in MAX_SIDE]
    on_max = off & struck & np.isin(owner, max_idx)
    return {
        'per_face': out,
        'offgrid': {
            'n': n_off_total,
            'n_within_tol_of_a_face': int(np.count_nonzero(off & struck)),
            'share_within_tol_of_a_face':
                (float(np.count_nonzero(off & struck)) / n_off_total)
                if n_off_total else None,
            'n_on_max_side_boundary': int(np.count_nonzero(on_max)),
            'share_on_max_side_boundary':
                (float(np.count_nonzero(on_max)) / n_off_total)
                if n_off_total else None,
            'tol_m': tol,
        },
    }


# ──────────────────────────────── the run ────────────────────────────────────

def all_stems(maps: list[str] | None) -> list[str]:
    if maps:
        return maps
    out = []
    for stem in gw.corpus_stems():
        out.append(stem)
        k, cut = gw._k_and_cut(stem, 'the gated twin')
        out.append(f'{gw.RUN_PREFIX}_k{k}_cut{cut}_gated_robot{k}')
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps',
                    help='one map stem; repeatable. Default: the 20 ungated and '
                         'the 20 gated maps')
    ap.add_argument('--spawn-rev', default=gw.SPAWN_REV,
                    help='git rev of the launch file DOT_POSES are read at')
    ap.add_argument('--no-json', action='store_true',
                    help='print everything and write nothing')
    args = ap.parse_args()
    t0 = time.monotonic()

    print('=' * 100)
    print("does Karto's grid stop short of its own extent, and does that lose "
          'walls?')
    print('  the candidate SPEC_b2_wall_predicate §12.3 reclassified §12\'s four '
          'B1 failures under.')
    print()
    print('PRE-REGISTERED, stated before any number below is read:')
    for name, text in PREDICTIONS:
        print(f'  {name}  {text}')
    print()
    print('  X1 and X2 are judged on ALL the maps this run covers -- the 20 '
          'ungated AND the 20')
    print('  gated -- which is the strictest reading of "every map". The ungated '
          'and gated')
    print('  subsets are reported separately beside the whole, so neither can be '
          'picked after')
    print('  the fact. Both are DIAGNOSIS: truth-derived, reported, never gated, '
          'and §6 keeps')
    print('  them out of every <map>.json.')
    print()
    print('SOURCE, parts quoted in the module docstring: ComputeDimensions sets')
    print('  rWidth = Round(size.GetWidth()*scale) (Karto.h:6111), '
          'rHeight likewise (:6112),')
    print('  rOffset = boundingBox.GetMinimum() (:6113). NOTHING PADS THE GRID: '
          'CreateFromScans')
    print('  (:5947-5964) passes those three to the constructor (:5910-5930) with '
          'no border, and')
    print('  Grid::CreateGrid (:4586-4594) takes no borderSize. Grid::Resize '
          'pads only the row')
    print('  STRIDE, m_WidthStep = AlignValue(width, 8) (:4640, Math.h:234-237); '
          'm_Width stays')
    print('  Round(size*scale) and IsValidGridIndex tests m_Width (:4671-4674), '
          'so the extra')
    print('  columns are unaddressable.')
    print('=' * 100)
    print()

    from fit_world_transform import resolve_spawn_poses       # numpy/yaml only
    import robot_divergence as rd                             # noqa: PLC0415
    spawn = resolve_spawn_poses(args.spawn_rev)
    world = gw.sdf_wall_faces(gw.L_MIN)
    samples = gw.face_samples(world, 0.01)
    radius, where = gw.read_robot_radius()
    params = gw.Params(g=2.0 * radius)

    stems = all_stems(args.maps)
    rows: dict[str, dict] = {}

    print('1 -- the far-edge shortfall. span is the bounding box in cells; '
          'shortfall is')
    print('    span - (W - 0.5), which is u - Round(u) + 0.5 and lies in [0, 1) '
          'by construction.')
    print()
    print(f'{"map":<36}{"W":>5}{"H":>5}{"span x":>10}{"span y":>10}'
          f'{"short x":>9}{"short y":>9}{"m x":>7}{"m y":>7}  lattice')
    for stem in stems:
        row = shortfall(stem)
        rows[stem] = row
        print(f'{stem:<36}{row["recomputed"]["width"]:>5}'
              f'{row["recomputed"]["height"]:>5}'
              f'{row["span_cells"][0]:>10.3f}{row["span_cells"][1]:>10.3f}'
              f'{row["shortfall_cells_x"]:>9.3f}{row["shortfall_cells_y"]:>9.3f}'
              f'{row["shortfall_m_x"]:>7.3f}{row["shortfall_m_y"]:>7.3f}'
              f'  {"ok" if row["lattice_reproduced"] else "REFUSED"}')
    print()
    bad = [s for s, r in rows.items() if not r['lattice_reproduced']]
    if bad:
        gw.die(f'the recomputed lattice does not reproduce the saved one on '
               f'{len(bad)} map(s): {bad}. Then the shortfall reported for them '
               'is not their shortfall, and no part of this diagnosis about them '
               'means anything.')
    sfx = [r['shortfall_cells_x'] for r in rows.values()]
    sfy = [r['shortfall_cells_y'] for r in rows.values()]
    print(f'  shortfall in x over {len(rows)} maps: min {min(sfx):.3f}, '
          f'median {float(np.median(sfx)):.3f}, max {max(sfx):.3f} cells')
    print(f'  shortfall in y over {len(rows)} maps: min {min(sfy):.3f}, '
          f'median {float(np.median(sfy)):.3f}, max {max(sfy):.3f} cells')
    print(f'  every map reproduces its saved origin, width and height exactly '
          f'({len(rows)}/{len(rows)}).')
    print()

    print('2 -- A9 recall per boundary face. max side = wall_east.west + '
          'wall_north.south;')
    print('    min side = wall_west.east + wall_south.north. Truth, diagnosis '
          'only.')
    print()
    print(f'{"map":<36}{"E.west":>9}{"N.south":>9}{"W.east":>9}{"S.north":>9}'
          f'{"max mean":>10}{"min mean":>10}{"max<min":>9}')
    rd.MAPS_DIR = gw.MAPS_DIR_DEFAULT
    for stem in stems:
        k = rows[stem]['robot']
        graph = gw.graph_for_map(stem, params=params)
        rec = recall_by_face(graph, spawn[k], world, samples,
                             rows[stem]['_rho'])
        rows[stem]['recall_by_face'] = rec
        mx = [rec[f'{m}.{s}']['recall'] for m, s in MAX_SIDE]
        mn = [rec[f'{m}.{s}']['recall'] for m, s in MIN_SIDE]
        rows[stem]['max_side_mean_recall'] = float(np.mean(mx))
        rows[stem]['min_side_mean_recall'] = float(np.mean(mn))
        print(f'{stem:<36}'
              + ''.join(f'{100 * rec[f"{m}.{s}"]["recall"]:>8.2f}%'
                        for m, s in MAX_SIDE + MIN_SIDE)
              + f'{100 * np.mean(mx):>9.2f}%{100 * np.mean(mn):>9.2f}%'
              + f'{str(np.mean(mx) < np.mean(mn)):>9}')
    print()

    print('3 -- per boundary face, the share of the returns that struck it which '
          'fell off the')
    print(f'    grid (within {STRIKE_TOL_CELLS:g}*rho of the face, nearest face '
          'wins), and X1\'s population.')
    print()
    print(f'{"map":<36}{"E.west":>9}{"N.south":>9}{"W.east":>9}{"S.north":>9}'
          f'{"offgrid":>9}{"on a face":>11}{"on max":>9}')
    for stem in stems:
        r = rows[stem]
        fs = face_strike_offgrid(r['_returns'], r['_origin'], r['_rho'],
                                 r['_grid_shape'], spawn[r['robot']], world)
        r['face_strikes'] = fs
        pf = fs['per_face']
        og = fs['offgrid']
        print(f'{stem:<36}'
              + ''.join(
                  (f'{100 * pf[f"{m}.{s}"]["share_off_grid"]:>8.2f}%'
                   if pf[f'{m}.{s}']['share_off_grid'] is not None else
                   f'{"-":>9}')
                  for m, s in MAX_SIDE + MIN_SIDE)
              + f'{og["n"]:>9}'
              + (f'{100 * og["share_within_tol_of_a_face"]:>10.2f}%'
                 if og['share_within_tol_of_a_face'] is not None else
                 f'{"-":>11}')
              + (f'{100 * og["share_on_max_side_boundary"]:>8.2f}%'
                 if og['share_on_max_side_boundary'] is not None else
                 f'{"-":>9}'))
    print()

    # ── X1 ──────────────────────────────────────────────────────────────────
    def x1_of(sub: list[str]) -> dict:
        near = [s for s in sub
                if (rows[s]['face_strikes']['offgrid']
                    ['share_within_tol_of_a_face'] or 0.0) >= X1_MIN]
        no_off = [s for s in sub
                  if rows[s]['face_strikes']['offgrid']['n'] == 0]
        cut1200 = [s for s in sub if rows[s]['cut_s'] == 1200]
        on_max = [s for s in cut1200
                  if (rows[s]['face_strikes']['offgrid']
                      ['share_on_max_side_boundary'] or 0.0) >= X1_MIN]
        return {'n_maps': len(sub),
                'n_offgrid_near_a_face': len(near),
                'maps_with_no_offgrid_returns': no_off,
                'near_holds': len(near) == len(sub),
                'n_cut1200': len(cut1200),
                'n_cut1200_on_max_side': len(on_max),
                'max_side_holds': len(on_max) == len(cut1200),
                'failing_near': [s for s in sub if s not in near],
                'failing_max_side': [s for s in cut1200 if s not in on_max]}

    ung = [s for s in stems if rows[s]['variant'] == 'ungated']
    gat = [s for s in stems if rows[s]['variant'] == 'gated']
    x1 = {'all': x1_of(stems), 'ungated': x1_of(ung), 'gated': x1_of(gat),
          'limit': X1_MIN}
    x1['holds'] = bool(x1['all']['near_holds'] and x1['all']['max_side_holds'])

    # ── X2 ──────────────────────────────────────────────────────────────────
    def x2_of(sub: list[str]) -> dict:
        obs = [s for s in sub
               if all(rows[s]['face_strikes']['per_face']
                      [f'{m}.{sd}']['observed']
                      for m, sd in MAX_SIDE + MIN_SIDE)]
        lower = [s for s in obs
                 if rows[s]['max_side_mean_recall']
                 < rows[s]['min_side_mean_recall']]
        share = (len(lower) / len(obs)) if obs else None
        return {'n_all_four_observed': len(obs),
                'maps_all_four_observed': obs,
                'n_max_lower': len(lower),
                'share': share,
                'holds': bool(obs) and share >= X2_MIN_SHARE,
                'not_lower': [s for s in obs if s not in lower]}

    x2 = {'all': x2_of(stems), 'ungated': x2_of(ung), 'gated': x2_of(gat),
          'limit': X2_MIN_SHARE}
    x2['holds'] = bool(x2['all']['holds'])

    print('=' * 100)
    print('PRE-REGISTERED VERDICTS')
    a = x1['all']
    print(f'  X1  off-grid returns within {STRIKE_TOL_CELLS:g}*rho of an SDF '
          f'face on >= {X1_MIN:.0%}: '
          f'{"HELD" if a["near_holds"] else "FAILED"} '
          f'({a["n_offgrid_near_a_face"]}/{a["n_maps"]})')
    for s in a['failing_near']:
        og = rows[s]['face_strikes']['offgrid']
        print(f'        {s}: {og["n"]} off-grid, '
              + ('none, so the share is undefined and the map cannot hold it'
                 if og['n'] == 0 else
                 f'{og["share_within_tol_of_a_face"]:.2%} near a face'))
    print(f'      on the cut1200 maps, >= {X1_MIN:.0%} on the east or north '
          f'boundary face: '
          f'{"HELD" if a["max_side_holds"] else "FAILED"} '
          f'({a["n_cut1200_on_max_side"]}/{a["n_cut1200"]})')
    for s in a['failing_max_side']:
        og = rows[s]['face_strikes']['offgrid']
        print(f'        {s}: {og["share_on_max_side_boundary"]:.2%} on the max '
              f'side, {og["share_within_tol_of_a_face"]:.2%} on any face')
    print(f'      X1 overall: {"HELD" if x1["holds"] else "FAILED"}   '
          f'(ungated near {x1["ungated"]["n_offgrid_near_a_face"]}'
          f'/{x1["ungated"]["n_maps"]}, gated '
          f'{x1["gated"]["n_offgrid_near_a_face"]}/{x1["gated"]["n_maps"]})')
    b = x2['all']
    print(f'  X2  max-side mean recall below min-side on >= {X2_MIN_SHARE:.0%} of '
          f'the maps with all four faces observed: '
          f'{"HELD" if b["holds"] else "FAILED"} '
          + (f'({b["n_max_lower"]}/{b["n_all_four_observed"]} = '
             f'{b["share"]:.1%})' if b['share'] is not None else
             '(no map observed all four, so X2 is not evaluable)'))
    if b['not_lower']:
        print('        max side NOT lower on: '
              + ', '.join(b['not_lower'][:6])
              + (' ...' if len(b['not_lower']) > 6 else ''))
    print(f'      ungated '
          + (f'{x2["ungated"]["n_max_lower"]}/'
             f'{x2["ungated"]["n_all_four_observed"]}'
             if x2['ungated']['n_all_four_observed'] else 'n/a')
          + ', gated '
          + (f'{x2["gated"]["n_max_lower"]}/'
             f'{x2["gated"]["n_all_four_observed"]}'
             if x2['gated']['n_all_four_observed'] else 'n/a'))
    print('=' * 100)
    print()

    for r in rows.values():
        for key in ('_returns', '_grid_shape', '_origin', '_rho'):
            r.pop(key, None)

    elapsed = time.monotonic() - t0
    print(f'{len(rows)} maps in {elapsed:.1f} s')
    if args.no_json:
        print('--no-json: nothing written')
        return 0

    out = {
        'question': "does Karto's grid stop short of its own extent, and does "
                    'that lose the walls on the max-x and max-y sides',
        'predictions': {name: text for name, text in PREDICTIONS},
        'predictions_registered': 'by Ali, in the request that produced this '
                                  'file, before any number in it was read',
        'judged_on': 'all 20 ungated and all 20 gated maps; the two subsets are '
                     'reported separately so neither can be chosen afterwards',
        'truth_used': True,
        'truth_used_for': 'parts 2 and 3 and X1 -- the SDF faces. Diagnosis '
                          'only; §6 keeps it out of every <map>.json',
        'gated': False,
        'touches': 'writes this file only; opens no map JSON and moves no '
                   'threshold',
        'provenance': {
            'source_tree': str(dk.SOURCE_TREE),
            'package': 'ros-jazzy-slam-toolbox 2.8.5',
            'orig_tarball_sha256': dk.SOURCE_SHA256,
            'copied_into_repo': False,
            'dimensions': 'OccupancyGrid::ComputeDimensions, Karto.h:6089-6113: '
                          'rWidth = Round(size.GetWidth()*scale) (:6111), '
                          'rHeight likewise (:6112), rOffset = '
                          'boundingBox.GetMinimum() (:6113), scale = 1/resolution '
                          '(:6108); math::Round is floor(v+0.5) for v >= 0 '
                          '(Math.h:87-90)',
            'nothing_pads_the_grid': 'the static CreateFromScans '
                                     '(Karto.h:5947-5964) passes width, height '
                                     'and offset straight to the constructor '
                                     '(:5910-5930) with no border; '
                                     'Grid::CreateGrid (:4586-4594) takes no '
                                     'borderSize. Grid::Resize pads only the row '
                                     'STRIDE, m_WidthStep = AlignValue(width, 8) '
                                     '(:4640, Math.h:234-237), while m_Width '
                                     'stays Round(size*scale) and '
                                     'IsValidGridIndex tests m_Width '
                                     '(:4671-4674) -- the extra columns are '
                                     'unaddressable',
            'shortfall_rule': 'WorldToGrid rounds about the offset '
                              '(Karto.h:4421-4436), so W cells cover '
                              '[O - res/2, O + (W - 0.5)*res) while the scans '
                              'span [O, O + size]. With u = size/res and '
                              'W = Round(u) the far-edge shortfall is '
                              'u - Round(u) + 0.5 cells, in [0, 1) for every u',
            'offset_semantics_contradiction':
                'CoordinateConverter::GetBoundingBox (Karto.h:4536-4548) reads '
                'the same offset as the grid\'s minimum CORNER, maxX = minX + '
                'W*res, while WorldToGrid/GridToWorld treat it as cell 0\'s '
                'CENTRE. Both are in one class',
            'extent_recomputation': 'predict_gated_maps.karto_extent, imported '
                                    'unchanged; a map whose saved origin, width '
                                    'and height it cannot reproduce exactly is '
                                    'refused, not reported',
            'cell_centre': cell_centre_provenance(),
            'spawn_rev': args.spawn_rev,
            'robot_radius': f'{where} robot_radius {radius}',
            'strike_tol_cells': STRIKE_TOL_CELLS,
        },
        'definitions': {
            'shortfall_cells': 'span in cells minus (W - 0.5): how far the '
                               "bounding box runs past the last cell's outer "
                               "edge under Karto's own centre semantics",
            'max_side': [f'{m}.{s}' for m, s in MAX_SIDE],
            'min_side': [f'{m}.{s}' for m, s in MIN_SIDE],
            'recall': "A9's covered-length share restricted to one inward face, "
                      'by graph_walls.validate_map\'s own covering test',
            'struck': f'within {STRIKE_TOL_CELLS:g}*rho of the face surface, '
                      'nearest face wins so a return is counted once',
            'off_grid': 'the Karto cell index falls outside [0, W) x [0, H) -- '
                        "B1's denominator test, and TraceLine's",
        },
        'verdicts': {'X1': x1, 'X2': x2},
        'shortfall_summary': {
            'x': {'min': min(sfx), 'median': float(np.median(sfx)),
                  'max': max(sfx)},
            'y': {'min': min(sfy), 'median': float(np.median(sfy)),
                  'max': max(sfy)},
            'always_in_0_1': bool(all(0.0 <= v < 1.0 for v in sfx + sfy)),
            'lattice_reproduced': f'{len(rows)}/{len(rows)}',
        },
        'elapsed_s': elapsed,
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
