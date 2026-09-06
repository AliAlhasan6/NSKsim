#!/usr/bin/env python3
"""Per-robot motion and scan-constraint statistics, read directly from a bag.

Answers two questions without ROS, without replay, and without SLAM:

  1. Did the robot traverse, or circle?
     path length vs bounding-box diagonal. A ratio near 1 is traversal;
     4 or more is circling in one corner.

  2. Did its scans ever constrain translation in both axes?
     lambda_min/lambda_max of the scatter matrix of per-beam surface normals.
     Near zero means one direction is unconstrained -- the degenerate case for
     laser scan matching, and the mechanism identified in the 2026-08-01 report
     for the nineteen clustered TF failures.

Run from the venv, NOT a ROS shell:

    cd ~/Desktop/NSKsim
    source nsk_env/bin/activate
    python3 experiments/analysis/bag_trajectory_stats.py experiments/bags/phaseB_run1

Dependencies: numpy, rosbags (pip, pure python -- reads mcap without ROS).
"""
import argparse
import csv
import math
import pathlib
import sys

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

# A scan is called degenerate below this eigenvalue ratio. This is a reporting
# threshold, not a physical constant. The mean ratio is the primary number and
# is printed alongside so the threshold can be second-guessed from the output.
DEGENERATE_RATIO = 0.05

# Two consecutive beams belong to the same surface only if their endpoints are
# close. 0.15 m at 3.5 m range spans roughly 2.5 beam widths on a 360-beam scan.
MAX_NEIGHBOUR_GAP = 0.15


def yaw_of(q):
    """Yaw from a quaternion, z-axis only (planar robot)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_constraint(msg):
    """lambda_min/lambda_max of the normal scatter matrix, and the finite count.

    Returns (ratio, n_finite), or (None, n_finite) when there are too few usable
    points to form normals at all. +inf returns (nothing within range_max) and
    -inf returns (closer than range_min, i.e. wall-hug) are both dropped -- see
    the B1.5a handoff; neither is a fault.
    """
    r = np.asarray(msg.ranges, dtype=float)
    finite = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    n_finite = int(finite.sum())
    if n_finite < 8:
        return None, n_finite

    ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
    pts = np.column_stack([r * np.cos(ang), r * np.sin(ang)])[finite]

    # Segment between consecutive kept points; a normal is that segment rotated
    # 90 degrees. Segments spanning a range discontinuity are not a surface.
    d = np.diff(pts, axis=0)
    seg = np.linalg.norm(d, axis=1)
    keep = (seg > 1e-6) & (seg < MAX_NEIGHBOUR_GAP)
    if keep.sum() < 4:
        return None, n_finite

    d = d[keep] / seg[keep, None]
    normals = np.column_stack([-d[:, 1], d[:, 0]])

    m = normals.T @ normals / normals.shape[0]
    ev = np.linalg.eigvalsh(m)
    lo, hi = float(ev[0]), float(ev[-1])
    if hi <= 0.0:
        return None, n_finite
    return lo / hi, n_finite


def analyse(bag, robot):
    """One pass over one robot's odom and scan topics."""
    odom_topic = f'/robot_{robot}/odom'
    scan_topic = f'/robot_{robot}/scan'

    xy, yaws = [], []
    ratios, finites, n_unusable = [], [], 0

    with Reader(bag) as reader:
        wanted = {odom_topic, scan_topic}
        conns = [c for c in reader.connections if c.topic in wanted]
        missing = wanted - {c.topic for c in conns}
        if missing:
            raise SystemExit(f'bag has no {", ".join(sorted(missing))}')

        for conn, _ts, raw in reader.messages(connections=conns):
            msg = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == odom_topic:
                p = msg.pose.pose.position
                xy.append((p.x, p.y))
                yaws.append(yaw_of(msg.pose.pose.orientation))
            else:
                ratio, n_finite = scan_constraint(msg)
                finites.append(n_finite)
                if ratio is None:
                    n_unusable += 1
                else:
                    ratios.append(ratio)

    if len(xy) < 2:
        raise SystemExit(f'robot_{robot}: fewer than 2 odom samples')

    xy = np.asarray(xy)
    steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    path = float(steps.sum())
    w = float(np.ptp(xy[:, 0]))
    h = float(np.ptp(xy[:, 1]))
    diag = math.hypot(w, h)
    net = float(np.linalg.norm(xy[-1] - xy[0]))

    ratios = np.asarray(ratios) if ratios else np.zeros(0)
    return {
        'robot': robot,
        'n_odom': len(xy),
        'n_scan': len(finites),
        'path_m': path,
        'bbox_w_m': w,
        'bbox_h_m': h,
        'bbox_diag_m': diag,
        'net_disp_m': net,
        'path_over_diag': path / diag if diag > 1e-6 else float('nan'),
        'mean_finite_returns': float(np.mean(finites)) if finites else 0.0,
        'mean_lambda_ratio': float(ratios.mean()) if ratios.size else float('nan'),
        'frac_degenerate': (float((ratios < DEGENERATE_RATIO).mean())
                            if ratios.size else float('nan')),
        'n_scans_unusable': n_unusable,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', help='bag directory, e.g. experiments/bags/phaseB_run1')
    ap.add_argument('--robots', type=int, nargs='+', default=[0, 1, 2, 3, 4])
    ap.add_argument('--csv', default='experiments/logs/bag_trajectory_stats.csv',
                    help='output CSV (never /tmp -- wiped on reboot)')
    args = ap.parse_args()

    if not pathlib.Path(args.bag).is_dir():
        sys.exit(f'not a bag directory: {args.bag}')

    rows = [analyse(args.bag, n) for n in args.robots]

    print(f'\n{args.bag}\n')
    print(f'{"rob":>3} {"scans":>6} {"path":>8} {"bbox":>15} {"p/diag":>7} '
          f'{"returns":>8} {"lam_ratio":>10} {"frac_deg":>9}')
    for r in rows:
        print(f'{r["robot"]:>3} {r["n_scan"]:>6} {r["path_m"]:>7.2f}m '
              f'{r["bbox_w_m"]:>6.2f} x {r["bbox_h_m"]:>6.2f} '
              f'{r["path_over_diag"]:>7.2f} '
              f'{r["mean_finite_returns"]:>8.1f} '
              f'{r["mean_lambda_ratio"]:>10.4f} '
              f'{r["frac_degenerate"]:>8.1%}')

    n_bad = sum(r['n_scans_unusable'] for r in rows)
    if n_bad:
        print(f'\n{n_bad} scans had too few usable returns to form normals '
              f'(excluded from lam_ratio and frac_deg)')

    out = pathlib.Path(args.csv)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'\nwrote {out}')


if __name__ == '__main__':
    main()
