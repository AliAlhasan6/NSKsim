#!/usr/bin/env python3
"""Which walls does each map's fit actually match, and how many cells?

The free-fit score in fit_world_transform.py is a FRACTION of occupied cells
within ON_WALL_TOL of a wall. That measure rewards small maps: a grid holding
one short wall segment and nothing else can score near 1.0 while constraining
almost nothing. robot_4 scores 97.2% on 250 occupied cells; robot_3 scores
41.9% on 453. This script asks whether that ranking survives being told WHICH
walls were matched and HOW MANY cells did the matching.

A rigid placement is only pinned down by structure in two independent
directions. Cells lying on a single straight segment leave translation along
that segment free -- the degenerate case for scan matching, and equally the
degenerate case for fitting a map to a wall model. If robot_4's matched cells
all fall on one rectangle, its 97.2% is agreement between two ways of sliding
along the same line, and the world-frame composition it confirmed is
unconfirmed.

Reuses fit_world_transform's loader, scorer and search unchanged. No second
wall model, no second distance function -- introducing either would break
comparability with every published baseline, which is the trap the cKDTree
scorer was rejected for.

Run from the venv, NOT a ROS shell:

    cd ~/Desktop/NSKsim
    python3 experiments/slam/wall_attribution.py

Writes experiments/logs/wall_attribution.csv
"""
import csv
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from fit_world_transform import (  # noqa: E402
    ON_WALL_TOL,
    THETA_STEP,
    build_wall_mask,
    cell_points,
    free_fit,
    load_map,
    place,
)
sys.path.insert(0, str(HERE.parent / 'analysis'))
from bag_overlap import dist_to_nearest_wall, world_walls  # noqa: E402

CONVENTION = 'A'

# A matched group is called "a wall" only if it holds at least this many cells.
# Below it, a handful of scattered hits on a far rectangle are noise, not
# structure a fit could have used. Reporting threshold, not a physical constant.
MIN_GROUP = 5


def attribute(pts, rects):
    """Nearest rectangle index per point, and the on-wall mask."""
    d_best = np.full(len(pts), np.inf)
    which = np.full(len(pts), -1, dtype=int)
    for i, (x0, x1, y0, y1) in enumerate(rects):
        dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0.0)
        dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0.0)
        d = np.hypot(dx, dy)
        closer = d < d_best
        d_best[closer] = d[closer]
        which[closer] = i
    return which, d_best < ON_WALL_TOL


def spread(pts):
    """Eigenvalues of the point scatter: lambda_min/lambda_max near zero means
    the matched cells lie along a single line and pin down only one axis."""
    if len(pts) < 3:
        return float('nan')
    c = pts - pts.mean(axis=0)
    ev = np.linalg.eigvalsh(c.T @ c / len(c))
    hi = float(ev[-1])
    return float(ev[0]) / hi if hi > 0 else float('nan')


def main():
    rects = world_walls()
    mask, wx0, wy0 = build_wall_mask(rects)

    # Same grid main() uses. free_fit takes the thetas as an argument rather
    # than owning them, so this must match or the peaks are not comparable.
    thetas = np.arange(-180.0, 180.0, THETA_STEP)

    rows = []
    for n in range(5):
        m = load_map(n)
        pts = cell_points(m, CONVENTION)
        c = pts.mean(axis=0)
        # free_fit returns (refined_peaks, bounds), sorted best-score first.
        peaks, _bounds = free_fit(pts, c, mask, wx0, wy0, rects, thetas)
        fit = peaks[0]

        # No placed cloud is returned -- the peak is (theta, dx, dy), so the
        # cells are re-placed with the fitter's own transform rather than a
        # reimplementation of it.
        placed = place(pts, c, fit['theta'], fit['dx'], fit['dy'])

        which, on = attribute(placed, rects)
        groups = {}
        for idx in np.unique(which[on]):
            k = int((which[on] == idx).sum())
            if k >= MIN_GROUP:
                groups[int(idx)] = k

        matched = placed[on]
        rows.append({
            'robot': n,
            'occupied_cells': len(pts),
            'score': fit.get('score', float('nan')),
            'matched_cells': int(on.sum()),
            'n_walls_matched': len(groups),
            'largest_wall_share': (max(groups.values()) / on.sum()
                                   if groups and on.sum() else float('nan')),
            'matched_spread': spread(matched),
            'walls': ';'.join(f'{k}:{v}' for k, v in sorted(groups.items())),
        })

    print(f'\n{"rob":>3} {"occ":>5} {"score":>7} {"matched":>8} {"walls":>6} '
          f'{"top_share":>10} {"spread":>8}  detail')
    for r in rows:
        print(f'{r["robot"]:>3} {r["occupied_cells"]:>5} {r["score"]:>7.3f} '
              f'{r["matched_cells"]:>8} {r["n_walls_matched"]:>6} '
              f'{r["largest_wall_share"]:>10.1%} {r["matched_spread"]:>8.4f}  '
              f'{r["walls"]}')

    out = pathlib.Path('experiments/logs/wall_attribution.csv')
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
