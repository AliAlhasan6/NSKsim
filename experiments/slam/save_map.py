#!/usr/bin/env python3
"""Self-contained occupancy-map saver (nav2_map_server not installed here).

Subscribes to an OccupancyGrid topic (transient_local/latched, as slam_toolbox
publishes it), then writes <out>.pgm (P5, trinary) + <out>.yaml in the exact
nav2 map_saver format.

Usage:
  save_map.py --out experiments/maps/robot_0_run1                 # default /map
  save_map.py --topic /robot_3/map --out .../robot_3_run1         # namespaced
  save_map.py --topic /robot_0/map --out ... --timeout 30         # slow RTF

--topic defaults to /map, which predates namespacing; under the swarm every
robot publishes on /robot_<id>/map, so pass it explicitly.
"""
import argparse
import collections
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

OCC_TH, FREE_TH = 0.65, 0.25  # nav2 map_saver defaults


def main():
    ap = argparse.ArgumentParser(
        description='Save a latched OccupancyGrid to nav2 .pgm/.yaml.')
    ap.add_argument(
        '--topic', default='/map',
        help='OccupancyGrid topic to save (default: /map). Under namespacing '
             'the real topic is /robot_<id>/map — pass it explicitly.')
    ap.add_argument(
        '--out', required=True,
        help='Output path WITHOUT extension; writes <out>.pgm and <out>.yaml.')
    ap.add_argument(
        '--timeout', type=float, default=20.0,
        help='Seconds to wait for the latched map before giving up '
             '(default: 20.0). nav2 map_saver_cli defaults to 2 s, which is too '
             'short for a fresh SLAM map to arrive here — do not repeat that.')
    args = ap.parse_args()
    topic, out, timeout = args.topic, args.out, args.timeout

    rclpy.init()
    node = rclpy.create_node('nsk_map_saver')
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.history = QoSHistoryPolicy.KEEP_LAST
    got = {}
    node.create_subscription(OccupancyGrid, topic,
                             lambda m: got.setdefault('map', m), qos)
    t0 = time.time()
    while rclpy.ok() and 'map' not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'map' not in got:
        print(f"ERROR: no map on {topic} within {timeout:g}s", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    m = got['map']
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    q = m.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    d = m.data
    pix = bytearray(w * h)
    k = 0
    for y in range(h):
        base = (h - y - 1) * w          # flip vertically, nav2 convention
        for x in range(w):
            v = d[base + x]
            if 0 <= v <= FREE_TH * 100:
                pix[k] = 254
            elif v >= OCC_TH * 100:
                pix[k] = 0
            else:
                pix[k] = 205
            k += 1
    with open(out + '.pgm', 'wb') as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(bytes(pix))
    name = out.split('/')[-1]
    with open(out + '.yaml', 'w') as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {res}\n")
        f.write(f"origin: [{ox}, {oy}, {yaw}]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    c = collections.Counter(d)
    occ = sum(n for v, n in c.items() if v >= 65)
    free = sum(n for v, n in c.items() if 0 <= v <= 25)
    unk = c.get(-1, 0)
    print(f"MAP SAVED {out}.pgm/.yaml  {w}x{h} res={res} "
          f"origin=({ox:.3f},{oy:.3f},{yaw:.3f})")
    print(f"cells total={w*h} occupied={occ} free={free} unknown={unk} "
          f"other={w*h-occ-free-unk}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
