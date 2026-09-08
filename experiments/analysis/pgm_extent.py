#!/usr/bin/env python3
"""Extent of the KNOWN region of a nav2 trinary PGM, in metres.

The grid dimensions in the .yaml include unknown padding and say nothing
about how much of the world was mapped. This reports the bounding box of
non-unknown cells (and of free cells alone), which is what compares against
a trajectory bounding box such as tf_corrected_extents_v2's.

Width x height do not depend on PGM row order. The printed map-frame
coordinates assume row 0 is the TOP of the image (max y), which is the
nav2 map_saver convention; treat them as indicative until checked.

Usage:
    python3 pgm_extent.py experiments/maps/b16_slamin_robot1.pgm [more.pgm ...]
Reads <stem>.yaml beside each PGM for resolution and origin.
Expected: well under a second per map.
"""
import os
import sys

import numpy as np


def read_pgm(path):
    data = open(path, "rb").read()
    if data[:2] != b"P5":
        raise ValueError(f"{path}: not a binary PGM")
    i, fields = 2, []
    while len(fields) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1
    w, h, _maxval = fields
    px = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return px


def read_yaml(path):
    res, origin = None, None
    for line in open(path):
        line = line.strip()
        if line.startswith("resolution:"):
            res = float(line.split(":", 1)[1])
        elif line.startswith("origin:"):
            vals = line.split(":", 1)[1].strip().strip("[]").split(",")
            origin = [float(v) for v in vals]
    if res is None or origin is None:
        raise ValueError(f"{path}: missing resolution or origin")
    return res, origin


def box(mask, res, origin, h):
    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    if len(rows) == 0:
        return None
    r0, r1, c0, c1 = rows[0], rows[-1], cols[0], cols[-1]
    wc, hc = c1 - c0 + 1, r1 - r0 + 1
    x0 = origin[0] + c0 * res
    x1 = origin[0] + (c1 + 1) * res
    y0 = origin[1] + (h - 1 - r1) * res
    y1 = origin[1] + (h - r0) * res
    return wc, hc, wc * res, hc * res, x0, x1, y0, y1


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for pgm in sys.argv[1:]:
        yaml = os.path.splitext(pgm)[0] + ".yaml"
        px = read_pgm(pgm)
        res, origin = read_yaml(yaml)
        h, w = px.shape
        occ = int((px == 0).sum())
        free = int((px == 254).sum())
        unk = int((px == 205).sum())
        print(f"{pgm}")
        print(f"  grid {w}x{h} @ {res} m = {w*res:.2f} x {h*res:.2f} m; "
              f"origin ({origin[0]:.3f}, {origin[1]:.3f})")
        print(f"  cells: {occ} occupied, {free} free, {unk} unknown")
        for label, mask in (("known (occ+free)", px != 205),
                            ("free only", px == 254),
                            ("occupied only", px == 0)):
            b = box(mask, res, origin, h)
            if b is None:
                print(f"  {label:<18} none")
                continue
            wc, hc, wm, hm, x0, x1, y0, y1 = b
            print(f"  {label:<18} {wm:5.2f} x {hm:5.2f} m  ({wc}x{hc} cells)  "
                  f"x [{x0:7.2f}, {x1:7.2f}]  y [{y0:7.2f}, {y1:7.2f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
