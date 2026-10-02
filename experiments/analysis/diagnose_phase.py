#!/usr/bin/env python3
"""Does a wall get cleared because of where Karto's cell edges happen to fall?

A DIAGNOSIS, and nothing else. Writes one file,
experiments/logs/graph_walls/diagnose_phase.json, and touches no map JSON, no
threshold and no reader.

WHERE THIS COMES FROM. diagnose_karto rebuilt Karto's counters exactly (20/20
maps) and showed that on the two long-dwell B1 failures the parked rays' PASSES
cleared the wall (H_dwell, 84 % and 69 % recommitted), while on k0_cut240 and
k4_cut120 -- which never dwell -- none of relay fill, dwell or grazing recommits
anything. Its step 5 then found that off-wall cells take their hits at a cell
EDGE (median 0.0045 m or 0.0957 m into a 0.1 m cell, tightly), while committed
wall cells take theirs inside the cell. That is the observation this file tests.

HYPOTHESIS. A wall is cleared when its surface lies near a Karto cell edge, and
where those edges fall is set separately for each map, not by the world.

THE PREMISE IS ALREADY CONFIRMED FROM SOURCE. OccupancyGrid::ComputeDimensions
(Karto.h:6090-6114, read at ~/src/slam_toolbox_2.8.5) sets
`rOffset = boundingBox.GetMinimum()` (:6113) -- the minimum corner of the union of
the processed scans' own bounding boxes (:6100-6106), with width and height
rounded from its size (:6111-6112). The offset is NOT snapped to a lattice and
owes nothing to the world: it is a raw float fixed by where that robot drove and
how far its rays reached. So every map has its own sub-cell phase, and two robots
mapping one wall can put a cell edge in different places on it.

WHAT IS MEASURED, per map:
  phase          ((Karto offset + spawn) mod rho) in x and y -- where that map's
                 cell lattice sits in the WORLD frame. Cell centres; the edge
                 lattice is the same thing shifted by rho/2, reported beside it.
  q, per face    the distance from each SDF face surface to the nearest Karto
                 cell edge, in [0, rho/2] = [0, 0.05] m.
  cleared share  per face, the share of its observed length that holds off-wall
                 returns rather than occupied cells (see face_cleared_share).

GEOMETRY. Cells are placed by KARTO's rule throughout -- centre at
origin + i*res, edges at origin + (i +/- 0.5)*res (Karto.h:4421-4436) -- the same
rule diagnose_karto's rebuild and its step 5 used. The ROS reading of these maps
sits half a cell away from that; diagnose_karto reports it and nothing here or
anywhere else has been corrected for it yet.

TRUTH. The SDF faces are truth, used for q and for the cleared share. Diagnosis
only: none of it goes near a map JSON.

    source /opt/ros/jazzy/setup.bash
    python3 experiments/analysis/diagnose_phase.py --map b2maps_k4_cut120_robot4
    python3 experiments/analysis/diagnose_phase.py

About two minutes for the corpus: the same 20 stripped bags Part B reads, and no
rebuild -- the phase question needs the returns and the saved grid, not the
counters.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'slam'))

import diagnose_karto as dk                                  # noqa: E402
import graph_walls as gw                                     # noqa: E402

OUT_PATH = gw.OUT_DIR_DEFAULT / 'diagnose_phase.json'

# The q bands the predictions are stated in.
Q_NEAR = 0.01           # m, "the surface sits on a cell edge"
Q_FAR = 0.02            # m, "it does not"
FACE_STEP = 0.01        # m, how finely a face is sampled along its length
NEAR_CELLS = 2.0        # the "within 2*rho" of the cleared-share definition
G2_MIN_SHARE = 0.80
G3_MIN_SPREAD = 0.02    # m

PREDICTIONS = [
    ('G1', 'on every map that has both kinds, faces with q < 0.01 m have a '
           'larger mean cleared share than faces with q >= 0.02 m'),
    ('G2', 'on k0_cut240 and k4_cut120 (no dwell), at least 80 % of off-wall '
           'cells lie on faces with q < 0.01 m'),
    ('G3', "at the same cut, the five robots' cell-edge phases in the world "
           'frame differ by more than 0.02 m in x or y'),
]


def sub_cell_phase(value: float, rho: float) -> float:
    """`value` modulo rho, in [0, rho)."""
    return float(value - math.floor(value / rho) * rho)


def distance_to_cell_edge(coord: float, origin_a: float, rho: float) -> float:
    """From a surface coordinate to the nearest Karto cell EDGE, in [0, rho/2].

    Karto centres cell i on origin + i*rho (Karto.h:4421-4436), so the edges lie
    halfway between, on the lattice origin + rho/2 + i*rho.
    """
    d = (coord - (origin_a + rho / 2.0)) / rho
    return float(abs(d - round(d)) * rho)


def map_frame_faces(faces: list, spawn) -> list:
    """The SDF faces in one robot's map frame.

    map = world - spawn: registered origin = spawn + YAML origin
    (robot_divergence.py:304), and these coordinates already carry the YAML
    origin, so the whole placement is a translation by the spawn.
    """
    out = []
    for i, f in enumerate(faces):
        out.append({
            'index': i, 'model': f['model'], 'side': f['side'],
            'length': f['length'],
            'p0': (f['p0'][0] - spawn[0], f['p0'][1] - spawn[1]),
            'p1': (f['p1'][0] - spawn[0], f['p1'][1] - spawn[1]),
            'n': f['n'],
        })
    return out


def face_q(face: dict, origin, rho: float) -> float:
    """q for one face: its surface lies along one axis, so q is that axis's."""
    if abs(face['n'][0]) > 0.5:                 # a face whose normal is +/-x
        return distance_to_cell_edge(face['p0'][0], origin[0], rho)
    return distance_to_cell_edge(face['p0'][1], origin[1], rho)


def face_cleared_share(face: dict, occupied: np.ndarray, off_count: np.ndarray,
                       origin, rho: float) -> dict:
    """What lies along one face: off-wall returns, or cells the map committed.

    The face is sampled every FACE_STEP along its length. Round each sample's
    neighbourhood -- every cell whose KARTO centre is within NEAR_CELLS*rho of
    the sample -- and ask two questions of it: does it hold an occupied cell, and
    does it hold a cell with an off-wall return in it.

      observed  either of those is true
      cleared   an off-wall return is there and NO occupied cell is

    `cleared_share` is cleared / observed, so a stretch of face the robot never
    saw lowers neither number; `n_observed` says how much of the face is being
    talked about. A sample with both is counted as observed and not cleared: the
    map did commit a wall cell there.
    """
    h, w = occupied.shape
    length = math.hypot(face['p1'][0] - face['p0'][0],
                        face['p1'][1] - face['p0'][1])
    n = max(1, int(round(length / FACE_STEP)))
    t = (np.arange(n) + 0.5) / n
    sx = face['p0'][0] + t * (face['p1'][0] - face['p0'][0])
    sy = face['p0'][1] + t * (face['p1'][1] - face['p0'][1])

    reach = int(math.ceil(NEAR_CELLS))
    di, dj = np.mgrid[-reach:reach + 1, -reach:reach + 1]
    di, dj = di.ravel()[None, :], dj.ravel()[None, :]
    ci = np.rint((sy - origin[1]) / rho).astype(np.int64)[:, None] + di
    cj = np.rint((sx - origin[0]) / rho).astype(np.int64)[:, None] + dj
    inside = (ci >= 0) & (ci < h) & (cj >= 0) & (cj < w)
    near = inside & (np.hypot(origin[0] + cj * rho - sx[:, None],
                              origin[1] + ci * rho - sy[:, None])
                     <= NEAR_CELLS * rho)
    cis, cjs = np.clip(ci, 0, h - 1), np.clip(cj, 0, w - 1)
    has_occ = (near & occupied[cis, cjs]).any(axis=1)
    has_off = (near & (off_count[cis, cjs] > 0)).any(axis=1)

    observed = has_occ | has_off
    cleared = has_off & ~has_occ
    n_obs = int(np.count_nonzero(observed))
    return {'n_samples': int(n), 'n_observed': n_obs,
            'n_cleared': int(np.count_nonzero(cleared)),
            'observed_m': n_obs * length / n,
            'cleared_share': (float(np.count_nonzero(cleared)) / n_obs)
            if n_obs else None}


def nearest_face_index(shape, origin, rho: float, faces: list) -> np.ndarray:
    """Per cell, which face of `faces` is nearest. Karto cell centres."""
    h, w = shape
    ys, xs = np.mgrid[0:h, 0:w]
    cx = origin[0] + xs * rho
    cy = origin[1] + ys * rho
    best = np.full(shape, np.inf)
    idx = np.full(shape, -1, dtype=np.int64)
    for i, f in enumerate(faces):
        ax, ay = f['p0']
        bx, by = f['p1']
        vx, vy = bx - ax, by - ay
        vv = vx * vx + vy * vy
        t = np.clip(((cx - ax) * vx + (cy - ay) * vy) / vv, 0.0, 1.0)
        d = np.hypot(cx - (ax + t * vx), cy - (ay + t * vy))
        closer = d < best
        best = np.where(closer, d, best)
        idx = np.where(closer, i, idx)
    return idx


def returns_for(stem: str, k: int, cut: int) -> dict:
    """The saved grid and Part B's kept returns. No rebuild, no counters.

    Exactly the chain graph_walls uses, so these are the same returns Part B and
    both earlier diagnoses measured.
    """
    import fit_world_transform as fwt                        # noqa: PLC0415
    import robot_divergence as rd                            # noqa: PLC0415

    m = gw.load_grid(stem)
    r_max, _src = gw.read_replay_r_max(k, cut)
    data = gw.read_bag(gw.BAGS_DIR / f'{gw.RUN_PREFIX}_k{k}_cut{cut}_slamin', k)
    base_scan, _hops = gw.compose_base_scan(data['static'], k)
    fwt.RUN = f'{gw.RUN_PREFIX}_k{k}_cut{cut}'
    mto, _p = fwt.load_map_to_odom(k)
    rd.check_map_to_odom(mto, stem)
    poses = gw.interpolate_poses(data['tf'], data['stamps'])
    returns = gw.scan_returns(data, poses, base_scan, mto, r_max)
    return {'grid': m['grid'], 'rho': m['rho'], 'origin': m['origin'],
            'shape': m['grid'].shape, 'returns': returns,
            'n_scans_read': int(data['stamps'].size)}


def diagnose(stem: str, k: int, cut: int, faces_world: list, spawn) -> dict:
    """One map: its phase, every face's q and cleared share, and its off-wall
    cells' q distribution."""
    rb = returns_for(stem, k, cut)
    rho, origin = rb['rho'], rb['origin']
    h, w = rb['shape']

    # B1's own off-wall set, from graph_walls, binned by KARTO's rule
    b1 = gw.on_occupied_share(rb['returns'], rb['grid'], rho, origin)
    off = ~b1['hit']
    cx, cy = dk.world_to_grid(rb['returns']['px'], rb['returns']['py'],
                              origin, rho)
    inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    off_count = np.zeros((h, w), dtype=np.int64)
    sel = off & inside
    if sel.any():
        np.add.at(off_count, (cy[sel], cx[sel]), 1)
    occupied = rb['grid'] == gw.OCC

    faces = map_frame_faces(faces_world, spawn)
    per_face = []
    for f in faces:
        q = face_q(f, origin, rho)
        share = face_cleared_share(f, occupied, off_count, origin, rho)
        per_face.append({'model': f['model'], 'side': f['side'],
                         'length_m': f['length'], 'q_m': q, **share})

    # G2: every off-wall cell's nearest face, and that face's q
    near_idx = nearest_face_index(rb['shape'], origin, rho, faces)
    off_cells = np.nonzero(off_count > 0)
    q_of = np.array([per_face[i]['q_m'] for i in near_idx[off_cells]]) \
        if off_cells[0].size else np.zeros(0)

    phase_x = sub_cell_phase(origin[0] + spawn[0], rho)
    phase_y = sub_cell_phase(origin[1] + spawn[1], rho)
    return {
        'map': stem, 'robot': k, 'cut_s': cut,
        'resolution': rho, 'karto_offset': list(origin), 'spawn': list(spawn),
        'phase': {
            'centre_x_m': phase_x, 'centre_y_m': phase_y,
            'edge_x_m': sub_cell_phase(phase_x + rho / 2.0, rho),
            'edge_y_m': sub_cell_phase(phase_y + rho / 2.0, rho),
            'definition': '(Karto offset + spawn) mod rho, in the world frame; '
                          'the edge lattice is the centre lattice + rho/2',
        },
        'b1_share': b1['share'],
        'n_off_wall_returns': int(np.count_nonzero(off)),
        'n_off_wall_cells': int(off_cells[0].size),
        'off_wall_cells_by_q': {
            'share_q_under_0p01': float(np.count_nonzero(q_of < Q_NEAR)
                                        / q_of.size) if q_of.size else None,
            'share_q_over_0p02': float(np.count_nonzero(q_of >= Q_FAR)
                                       / q_of.size) if q_of.size else None,
            'median_q_m': float(np.median(q_of)) if q_of.size else None,
        },
        'faces': per_face,
    }


def band_means(per_face: list) -> dict:
    """G1's two bands: the mean cleared share of observed faces in each."""
    near = [f['cleared_share'] for f in per_face
            if f['q_m'] < Q_NEAR and f['cleared_share'] is not None]
    far = [f['cleared_share'] for f in per_face
           if f['q_m'] >= Q_FAR and f['cleared_share'] is not None]
    return {'n_faces_q_under_0p01': len(near), 'n_faces_q_over_0p02': len(far),
            'mean_cleared_q_under_0p01': (float(np.mean(near)) if near
                                          else None),
            'mean_cleared_q_over_0p02': (float(np.mean(far)) if far else None),
            'has_both_kinds': bool(near and far)}


def circular_spread(values: list, rho: float) -> float:
    """The largest pairwise difference of phases, respecting the wrap at rho."""
    worst = 0.0
    for i, a in enumerate(values):
        for b in values[i + 1:]:
            d = abs(a - b) % rho
            worst = max(worst, min(d, rho - d))
    return float(worst)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', action='append', dest='maps',
                    help='one map stem; repeatable. Default: all 20 b2maps cuts')
    args = ap.parse_args()
    stems = args.maps or gw.corpus_stems()

    print('does a wall get cleared because of where Karto\'s cell edges fall?')
    print('  HYPOTHESIS: a wall is cleared when its surface lies near a Karto '
          'cell edge, and')
    print('  where those edges fall is set separately for each map, not by the '
          'world.')
    print()
    print('  THE PREMISE, already read from source: '
          'OccupancyGrid::ComputeDimensions')
    print('  (Karto.h:6090-6114) sets rOffset = boundingBox.GetMinimum() '
          '(:6113) -- the minimum')
    print('  corner of the union of the processed scans\' own bounding boxes. '
          'NOT snapped to a')
    print('  lattice, and owing nothing to the world: a raw float fixed by '
          'where that robot')
    print('  drove and how far its rays reached.')
    print()
    print('PRE-REGISTERED, printed before any number is read:')
    for name, text in PREDICTIONS:
        print(f'  {name}  {text}')
    print()
    print(f'q is the distance to the nearest Karto cell EDGE, in [0, '
          f'{0.05:.2f}] m. Cells are placed')
    print('  by Karto\'s rule (centre at origin + i*res), as in '
          'diagnose_karto. The half-cell')
    print('  shift against the ROS reading is reported there and corrected '
          'nowhere.')
    print()

    from fit_world_transform import resolve_spawn_poses       # noqa: PLC0415
    spawn = resolve_spawn_poses(gw.SPAWN_REV)
    faces_world = gw.sdf_wall_faces(gw.L_MIN)['faces']
    print(f'{len(faces_world)} SDF faces, spawn table at {gw.SPAWN_REV}')
    print()

    rows = {}
    print(f'{"map":<30}{"phase x":>9}{"phase y":>9}{"off cells":>11}'
          f'{"med q":>8}{"q<.01":>8}{"q>=.02":>8}'
          f'{"clr q<.01":>11}{"clr q>=.02":>12}')
    for stem in stems:
        k, cut = gw._k_and_cut(stem, 'the phase test needs the bag and the '
                                     'replay config')
        row = diagnose(stem, k, cut, faces_world, spawn[k])
        row['bands'] = band_means(row['faces'])
        rows[stem] = row
        b, q = row['bands'], row['off_wall_cells_by_q']

        def pct(v):
            return f'{v:.0%}' if v is not None else '-'

        def num(v):
            return f'{v:.3f}' if v is not None else '-'
        print(f'{stem:<30}{row["phase"]["centre_x_m"]:>9.4f}'
              f'{row["phase"]["centre_y_m"]:>9.4f}'
              f'{row["n_off_wall_cells"]:>11}{num(q["median_q_m"]):>8}'
              f'{pct(q["share_q_under_0p01"]):>8}'
              f'{pct(q["share_q_over_0p02"]):>8}'
              f'{num(b["mean_cleared_q_under_0p01"]):>11}'
              f'{num(b["mean_cleared_q_over_0p02"]):>12}')
    print()

    # ── G1 ───────────────────────────────────────────────────────────────────
    g1 = {}
    for stem, row in rows.items():
        b = row['bands']
        if not b['has_both_kinds']:
            continue
        g1[stem] = {'near': b['mean_cleared_q_under_0p01'],
                    'far': b['mean_cleared_q_over_0p02'],
                    'holds': bool(b['mean_cleared_q_under_0p01']
                                  > b['mean_cleared_q_over_0p02'])}
    # ── G2 ───────────────────────────────────────────────────────────────────
    g2 = {}
    for stem in ('b2maps_k0_cut240_robot0', 'b2maps_k4_cut120_robot4'):
        if stem not in rows:
            continue
        share = rows[stem]['off_wall_cells_by_q']['share_q_under_0p01']
        g2[stem] = {'share_q_under_0p01': share,
                    'holds': bool(share is not None
                                  and share >= G2_MIN_SHARE)}
    # ── G3 ───────────────────────────────────────────────────────────────────
    g3 = {}
    for cut in gw.CUTS:
        have = [rows[f'{gw.RUN_PREFIX}_k{k}_cut{cut}_robot{k}']
                for k in range(gw.NUM_ROBOTS)
                if f'{gw.RUN_PREFIX}_k{k}_cut{cut}_robot{k}' in rows]
        if len(have) < 2:
            continue
        rho = have[0]['resolution']
        sx = circular_spread([r['phase']['centre_x_m'] for r in have], rho)
        sy = circular_spread([r['phase']['centre_y_m'] for r in have], rho)
        g3[str(cut)] = {'n_robots': len(have), 'spread_x_m': sx,
                        'spread_y_m': sy,
                        'holds': bool(sx > G3_MIN_SPREAD
                                      or sy > G3_MIN_SPREAD)}

    def verdict(d):
        """HELD, FAILED, or NOT EVALUATED when there was nothing to read.

        The b2maps README's own rule: a criterion that could not be evaluated is
        not a passed one, and it is not a failed one either -- saying FAILED of an
        empty comparison would be inventing a result.
        """
        if not d:
            return 'NOT EVALUATED'
        return 'HELD' if all(v['holds'] for v in d.values()) else 'FAILED'

    print('PREDICTIONS, read off the numbers above. Reported, never gated.')
    print(f'  G1  {verdict(g1)}'
          f'  ({sum(v["holds"] for v in g1.values())}/{len(g1)} maps with both '
          'kinds of face)')
    for stem, v in g1.items():
        print(f'        {stem}: q<0.01 mean {v["near"]:.3f} vs q>=0.02 mean '
              f'{v["far"]:.3f} -> '
              f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    print(f'  G2  {verdict(g2)}')
    for stem, v in g2.items():
        s = v['share_q_under_0p01']
        shown = f'{100 * s:.1f}%' if s is not None else 'no off-wall cells'
        print(f'        {stem}: {shown} of off-wall cells on faces with '
              f'q < {Q_NEAR} m -> '
              f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    print(f'  G3  {verdict(g3)}')
    for cut, v in g3.items():
        print(f'        cut {cut}: phase spread x {v["spread_x_m"]:.4f} m, '
              f'y {v["spread_y_m"]:.4f} m over {v["n_robots"]} robots -> '
              f'{"as predicted" if v["holds"] else "NOT as predicted"}')
    print()

    out = {
        'question': 'is a wall cleared because of where Karto\'s cell edges '
                    'happen to fall, and is that phase per map rather than per '
                    'world',
        'provenance': {
            'source_tree': str(dk.SOURCE_TREE),
            'package': 'ros-jazzy-slam-toolbox 2.8.5',
            'orig_tarball_sha256': dk.SOURCE_SHA256,
            'copied_into_repo': False,
            'offset_rule': 'OccupancyGrid::ComputeDimensions, Karto.h:6090-6114: '
                           'rOffset = boundingBox.GetMinimum() (:6113) over the '
                           'union of the processed scans\' bounding boxes '
                           '(:6100-6106); width and height are Round(size*scale) '
                           '(:6111-6112). NOT snapped to a lattice',
            'cell_geometry': 'Karto: centre at origin + i*res, edges at '
                             'origin + (i +/- 0.5)*res (Karto.h:4421-4436). The '
                             'ROS reading sits half a cell away; see '
                             'diagnose_karto.json. Nothing corrected here',
            'spawn_rev': gw.SPAWN_REV,
        },
        'definitions': {
            'q': 'distance from an SDF face surface to the nearest Karto cell '
                 f'edge, in [0, {0.05}] m',
            'phase': '(Karto offset + spawn) mod rho, in the world frame',
            'cleared_share': 'per face, of the samples with an occupied cell or '
                             f'an off-wall return within {NEAR_CELLS}*rho, the '
                             'share that have an off-wall return and no '
                             f'occupied cell. Sampled every {FACE_STEP} m',
            'off_wall': "graph_walls.on_occupied_share's own B1 mask, binned by "
                        "Karto's rule",
        },
        'predictions': {name: text for name, text in PREDICTIONS},
        'truth_used_for': 'the SDF faces, for q and the cleared share. '
                          'Diagnosis only',
        'gated': False,
        'verdicts': {
            'G1': {'per_map': g1, 'evaluated': bool(g1),
                   'held': bool(g1 and all(v['holds'] for v in g1.values()))},
            'G2': {'per_map': g2, 'evaluated': bool(g2),
                   'held': bool(g2 and all(v['holds'] for v in g2.values()))},
            'G3': {'per_cut': g3, 'evaluated': bool(g3),
                   'held': bool(g3 and all(v['holds'] for v in g3.values()))},
        },
        'per_map': rows,
    }
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps(out, indent=2, sort_keys=False) + '\n')
    print(f'wrote {OUT_PATH.relative_to(REPO_ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
