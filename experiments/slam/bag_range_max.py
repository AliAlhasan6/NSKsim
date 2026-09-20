#!/usr/bin/env python3
"""Print the LaserScan range_max recorded on /robot_<N>/scan in a bag.

WHY THIS EXISTS
---------------
run_offline_maps.sh replays recorded scans, so the sensor ceiling it must hand
slam_toolbox is a property of the BAG, not of the current launch files. There
are now two eras of bag in this repo -- every b16, b18 and b2maps bag is a
3.5 m LDS-01, and anything recorded after BURGER_RANGE_MAX moved is an 8 m
LDS-02 -- so a literal in that script would be wrong for one era whichever
value it took.

Neither way of being wrong announces itself:

  max_laser_range too HIGH is inert. Karto clips its rangeThreshold to the
  scan's own maxRange (Karto.h:3946-3949), so 8.0 against a 3.5 m bag simply
  behaves as 3.5 and nothing in the log says the number was ignored.

  A relay threshold too high is worse than inert. 7.9 against a 3.5 m bag
  leaves no value that is both traced by Karto and not dropped by it, so
  free_space_relay refuses to invent one (fill_value returns None), logs once,
  and passes every scan through UNCHANGED. The replay completes, writes a map,
  and that map is the no-relay map under a filename that claims otherwise.

Reading the number is cheap and removes the whole question.

ONE MESSAGE, NOT A SURVEY
-------------------------
Only the first LaserScan on the topic is deserialised. range_max is a property
of the sensor and is constant within a run; scanning 32524 messages to confirm
that would cost minutes per invocation on a 3 GB bag, and every caller here is
a shell script blocking on the answer. A bag whose range_max genuinely varies
mid-run is a broken bag, and --check is there for the one time that needs
proving rather than assuming.

Usage:
    bag_range_max.py <bag-dir> [robot-id]      # prints e.g. 3.5
    bag_range_max.py <bag-dir> [robot-id] --check

Exit codes:
  0  the value is on stdout, bare, with no trailing text -- it is consumed by
     command substitution, so anything else on stdout corrupts the caller.
  1  the bag, the topic, or a LaserScan on it could not be found, or --check
     found more than one distinct range_max. The reason is on STDERR, again so
     that a caller capturing stdout gets a clean empty string rather than a
     number with an error message glued to it.
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def die(message):
    print(f'{message}', file=sys.stderr)
    sys.exit(1)


def open_reader(bag):
    """A SequentialReader over `bag`, storage auto-detected.

    storage_id is left empty so mcap and sqlite3 bags both open: this repo has
    both, and hardcoding 'mcap' would refuse the older ones for no reason.
    """
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
    except Exception as exc:                       # noqa: BLE001 -- rosbag2
        die(f'cannot open bag {bag}: {exc}')
    return reader


def range_max(bag, robot, check=False):
    """The recorded range_max on /robot_<robot>/scan."""
    topic = f'/robot_{robot}/scan'
    reader = open_reader(bag)

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        die(f'{bag} has no {topic}. Topics: {", ".join(sorted(types)) or "none"}')

    msg_type = get_message(types[topic])
    storage_filter = rosbag2_py.StorageFilter()
    storage_filter.topics = [topic]
    reader.set_filter(storage_filter)

    seen = set()
    while reader.has_next():
        _topic, data, _stamp = reader.read_next()
        seen.add(deserialize_message(data, msg_type).range_max)
        if not check:
            break

    if not seen:
        die(f'{bag} has {topic} in its metadata but not one message on it')
    if len(seen) > 1:
        die(f'{topic} carries {len(seen)} distinct range_max values '
            f'({", ".join(f"{v:g}" for v in sorted(seen))}). The sensor '
            f'changed mid-recording, so no single ceiling describes this bag.')
    return seen.pop()


def main():
    args = [a for a in sys.argv[1:] if a != '--check']
    if not args:
        die(__doc__.strip().splitlines()[0] +
            '\nusage: bag_range_max.py <bag-dir> [robot-id] [--check]')

    bag = args[0]
    robot = args[1] if len(args) > 1 else '0'
    value = range_max(bag, robot, check='--check' in sys.argv[1:])

    # Bare, on stdout, no newline decoration beyond the one print adds: the
    # caller is `$(...)` in a shell script and compares the result against a
    # numeric regex.
    print(f'{value:g}')


if __name__ == '__main__':
    main()
