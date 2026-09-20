#!/usr/bin/env python3
"""free_space_relay.py — republish a scan so no-return beams clear free space.

Why this exists (2026-09-20):
  b2maps_e0t explored for 111 minutes on true poses with the matcher off and
  still mapped 63 m2 of a 391 m2 floor, never coming within 4.18 m of the
  perimeter. The cause is not the poses, the explorer, or the matcher: it is
  what Karto does with a beam that hits nothing.

  slam_toolbox 2.8.5 vendors Karto as karto_sdk, and OccupancyGrid::AddScan
  (Karto.h:6148-6191, inline in the installed header) opens with

      if (rangeReading <= minRange || rangeReading >= maxRange ||
          std::isnan(rangeReading)) {
        // ignore these readings
        continue;                      // <- no RayTrace, nothing cleared
      } else if (rangeReading >= rangeThreshold) {
        // trace up to range reading                 <- free space, no hit
      }

  maxRange is the scan's own range_max (3.5 m here) and rangeThreshold is the
  max_laser_range parameter, which slam_toolbox clamps to the sensor and Karto
  clips again to <= maxRange (Karto.h:3946-3949). Our runs set max_laser_range
  to 3.5, so rangeThreshold == maxRange and the SECOND branch is unreachable:
  every beam that hits nothing is dropped whole, and the floor it passed over
  stays unknown. 57.47% of b2maps_e0t's 11.7 million beams are +inf, so more
  than half the sensor's output cleared nothing at all.

What this node does:
  Republishes /robot_K/scan as /robot_K/scan_free with every no-return beam --
  +inf, and finite readings at or above range_max -- rewritten to a value
  strictly between the range threshold and range_max. Karto then takes the
  SECOND branch for those beams: it traces free space out to the threshold and
  marks no obstacle, because isEndPointValid (Karto.h:6167) is
  rangeReading < rangeThreshold - KT_TOLERANCE, which a filled beam fails.

  The two numbers must straddle: with max_laser_range 3.4 and range_max 3.5
  the fill is 3.45, which is >= rangeThreshold (so it is traced) and
  < maxRange (so it is not dropped). Filling at range_max - epsilon with
  max_laser_range left at 3.5 would instead make isEndPointValid TRUE and
  paint a phantom wall at 3.49 m on 57% of beams. The relay therefore derives
  the fill from the threshold it is told and the range_max the scan itself
  carries, and refuses to invent one if they do not straddle.

  NaN is left alone. In gz-sim a NaN is not "nothing within range", it is a
  reading the sensor could not produce; none appear in b2maps_e0t's 11.7
  million beams, and inventing free space from one would be inventing data.
  Genuine hits are copied through untouched, including readings below
  range_min, which Karto drops for its own reasons.

What consumes the output:
  slam_toolbox, and nothing else. Nav2's costmaps stay on the RAW scan: the
  fill value is a convention meaningful only against Karto's rangeThreshold,
  and an obstacle layer reading it as a return would place an obstacle there.
  Nav2 has its own knob for this question (inf_is_valid on the observation
  source), which is a separate decision with its own rehearsal.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# How often to log the running fraction. The fraction IS the finding -- 57% on
# b2maps_e0t -- so a run should carry it in its log rather than needing a bag
# replay to recover it.
LOG_EVERY = 200


def fill_value(range_threshold: float, range_max: float) -> float | None:
    """The value a no-return beam is rewritten to, or None if none exists.

    Strictly between the threshold and range_max, so Karto traces it as free
    space and does not mark a hit. The midpoint rather than either end: at
    exactly rangeThreshold the trace is the same but the float comparison is
    on a knife edge, and at range_max the reading is dropped outright.

    None when the two do not straddle -- range_threshold >= range_max leaves no
    room for a value that is both traced and not dropped, which is exactly the
    configuration this node exists to escape, so it must not paper over it.
    """
    if not (range_max > range_threshold):
        return None
    return 0.5 * (range_threshold + range_max)


def rewrite_ranges(ranges, range_max: float, fill: float):
    """(rewritten ranges, how many were rewritten). Pure.

    A no-return beam is +inf or a finite reading at or above range_max -- both
    of which Karto's `rangeReading >= maxRange` test drops. -inf is not a
    no-return (nothing produces it, and it is not "no obstacle within range"),
    NaN is not either; both pass through untouched.
    """
    out = list(ranges)
    n = 0
    for i, r in enumerate(out):
        if math.isnan(r):
            continue
        if r == math.inf or r >= range_max:
            out[i] = fill
            n += 1
    return out, n


class FreeSpaceRelay(Node):
    """One instance per robot whose scans feed a known-pose mapper."""

    def __init__(self):
        super().__init__('free_space_relay')

        self.declare_parameter('robot_id', 0)
        # The max_laser_range slam_toolbox will be given. Not read from
        # slam_robotN.yaml: this node would then depend on a file it does not
        # own, and the launch file that sets one sets the other.
        self.declare_parameter('range_threshold', 3.4)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.range_threshold = float(
            self.get_parameter('range_threshold').value)

        ns = f'robot_{self.robot_id}'
        self.in_topic = f'/{ns}/scan'
        self.out_topic = f'/{ns}/scan_free'

        # Sensor QoS both ways: best-effort/keep-last is what the gz bridge
        # offers and what slam_toolbox subscribes with, and a scan that
        # arrives late is worse than one that does not arrive.
        self.pub = self.create_publisher(
            LaserScan, self.out_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.in_topic, self._scan_cb, qos_profile_sensor_data)

        self._scans = 0
        self._beams = 0
        self._filled = 0
        self._complained = False

        self.get_logger().info(
            f'free-space relay for {ns}: {self.in_topic} -> {self.out_topic}, '
            f'no-return beams filled just above a threshold of '
            f'{self.range_threshold:.2f} m')

    def _scan_cb(self, msg: LaserScan):
        fill = fill_value(self.range_threshold, msg.range_max)
        if fill is None:
            if not self._complained:
                self._complained = True
                self.get_logger().error(
                    f'range_threshold {self.range_threshold:.3f} m is not '
                    f'below the scan range_max {msg.range_max:.3f} m, so a '
                    'filled beam would be dropped by Karto exactly as +inf '
                    'is. Passing scans through UNCHANGED -- lower '
                    'range_threshold (and slam_toolbox max_laser_range with '
                    'it) below range_max.')
            self.pub.publish(msg)
            return

        ranges, filled = rewrite_ranges(msg.ranges, msg.range_max, fill)
        # A new message rather than a mutated one: the incoming message is
        # shared with any other subscriber in this process, and the header,
        # angles and range limits must survive verbatim -- range_max above all,
        # since the fill is only correct BELOW it.
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities
        self.pub.publish(out)

        self._scans += 1
        self._beams += len(ranges)
        self._filled += filled
        if self._scans % LOG_EVERY == 0:
            self.get_logger().info(
                f'{self._scans} scans, {self._filled}/{self._beams} beams '
                f'filled ({100.0 * self._filled / max(self._beams, 1):.1f}%), '
                f'fill {fill:.3f} m')


def main(args=None):
    rclpy.init(args=args)
    node = FreeSpaceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'relayed {node._scans} scans, filled {node._filled} of '
            f'{node._beams} beams')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
