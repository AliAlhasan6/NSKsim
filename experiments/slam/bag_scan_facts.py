#!/usr/bin/env python3
"""Print ONE fact about /robot_<N>/scan in a bag, for a shell caller.

WHY THIS EXISTS
---------------
Two callers need a number off the scan topic before they can do their job, and
both are shell scripts blocking on the answer:

  run_offline_maps.sh, for DETERMINISTIC=true's scan_queue_size. slam_toolbox
  hands its scan subscription to a tf2_ros::MessageFilter whose queue holds
  scans that are waiting for odom->base_footprint, and scan_queue_size defaults
  to 1 -- so a scan still waiting when the next one arrives is DISCARDED
  ('discarding message because the queue is full'). In deterministic mode every
  scan is supposed to be integrated, and a dropped scan is a scan that never
  reached Karto at all. Sizing the queue at the segment's own scan count makes
  eviction impossible by construction rather than by a margin someone guessed.

  run_b2maps_cuts.sh, for its drop gate. One drop at the segment's FIRST scan is
  unavoidable -- there is no transform history behind it yet, so it is dropped
  either for an empty transform cache or for the queue -- and every later drop is
  a scan silently missing from the map. Telling those apart needs the first
  scan's stamp, and the log line to compare it against prints %.3f.

--count IS METADATA, --first-stamp IS NOT
-----------------------------------------
--count reads metadata.yaml's per-topic message_count, the same source
check_run_bag.read_metadata() uses. No deserialisation, and no ROS environment
either -- PyYAML alone -- so the queue size can be sized from a bag without
sourcing anything.

--first-stamp has to open the bag, because a LaserScan's header stamp is inside
the message. Only the first message is read: that is the one being asked about.

THE STAMP IS THE HEADER'S, NOT THE BAG'S
----------------------------------------
A bag carries two times per message: the recorder's receive time and the stamp
the publisher wrote. tf2_ros::MessageFilter logs its drops against the HEADER
stamp ('at time 95.800'), which in these runs is sim time, so that is what this
prints and what a caller may compare. The receive time would be a wall clock and
would never match.

free_space_relay copies the header through untouched (free_space_relay.py:166,
`out.header = msg.header`), so a bag's /robot_N/scan stamps are also the stamps
of the /robot_N/scan_free that slam_toolbox actually subscribes to under
FREE_SPACE_RELAY=true. One read answers for both topics.

Usage:
    bag_scan_facts.py <bag-dir> <robot-id> --count         # prints e.g. 6000
    bag_scan_facts.py <bag-dir> <robot-id> --first-stamp    # prints e.g. 106.000

Exit codes:
  0  the value is on stdout, bare, with no trailing text -- it is consumed by
     command substitution, so anything else on stdout corrupts the caller.
  1  the bag, its metadata, the topic, or a message on it could not be read.
     The reason is on STDERR, again so that a caller capturing stdout gets a
     clean empty string rather than a number with an error glued to it.
"""
import argparse
import sys
from pathlib import Path

import yaml


def die(message):
    print(f'{message}', file=sys.stderr)
    sys.exit(1)


def scan_count(bag: Path, robot: str) -> int:
    """message_count of /robot_<robot>/scan, out of metadata.yaml.

    Metadata rather than a read: the count is recorded, and counting 6000
    messages by deserialising them would cost seconds per invocation for a
    number the bag already states.
    """
    topic = f'/robot_{robot}/scan'
    meta = bag / 'metadata.yaml'
    if not meta.is_file():
        die(f'no metadata.yaml in {bag} -- not a rosbag2 directory')
    try:
        info = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
        counts = {t['topic_metadata']['name']: int(t['message_count'])
                  for t in info['topics_with_message_count']}
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        die(f'cannot read topic counts out of {meta}: {exc}')
    if topic not in counts:
        die(f'{bag} has no {topic}. Topics: '
            f'{", ".join(sorted(counts)) or "none"}')
    if counts[topic] == 0:
        die(f'{topic} is in {bag}\'s metadata with a count of 0, so there is '
            'nothing to replay for this robot')
    return counts[topic]


def first_stamp(bag: Path, robot: str) -> float:
    """header.stamp of the first message on /robot_<robot>/scan, in seconds."""
    # Imported here, not at module level: --count must work without the ROS
    # environment, and rosbag2_py is the only thing in this file that needs it.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topic = f'/robot_{robot}/scan'
    reader = rosbag2_py.SequentialReader()
    try:
        # storage_id empty so mcap and sqlite3 bags both open -- this repo has
        # both, and hardcoding one would refuse the other for no reason. Same
        # choice as bag_range_max.open_reader().
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
    except Exception as exc:                       # noqa: BLE001 -- rosbag2
        die(f'cannot open bag {bag}: {exc}')

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        die(f'{bag} has no {topic}. Topics: {", ".join(sorted(types)) or "none"}')

    msg_type = get_message(types[topic])
    storage_filter = rosbag2_py.StorageFilter()
    storage_filter.topics = [topic]
    reader.set_filter(storage_filter)

    if not reader.has_next():
        die(f'{bag} has {topic} in its metadata but not one message on it')
    _topic, data, _recv_ns = reader.read_next()
    stamp = deserialize_message(data, msg_type).header.stamp
    return stamp.sec + stamp.nanosec / 1e9


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', type=Path, help='bag directory')
    ap.add_argument('robot', help='robot id, as it appears in /robot_<id>/scan')
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument('--count', action='store_true',
                      help='message count on the topic, from metadata.yaml')
    what.add_argument('--first-stamp', action='store_true',
                      help="the first message's header stamp, in seconds")
    args = ap.parse_args()

    if args.count:
        print(f'{scan_count(args.bag, args.robot)}')
    else:
        # %.3f, matching the format tf2_ros::MessageFilter logs a dropped
        # message's time with, so a caller can compare the two directly.
        print(f'{first_stamp(args.bag, args.robot):.3f}')


if __name__ == '__main__':
    main()
