#!/usr/bin/env python3
"""attribute_tf_drop.py -- why a scan the costmap dropped could not be
transformed, per link, from the bag.

Triage for R9's gated message-filter drops. It decides nothing; it prints the
one distinction that tells you what to do about a FAIL:

  a LINK WAS LATE    the first transform on that link stamped at or after the
                     dropped scan reached the wire AFTER the node had already
                     given up. The transform was genuinely not available. On
                     b2maps_k2's drop at scan 629.400 that link was
                     robot_2/map -> robot_2/odom, whose stamp had frozen at
                     629.300 for 1.502 s of wall -- 5.7x its normal 0.26 s
                     dwell -- and then jumped to 630.300, skipping four
                     200 ms stamps.

  NONE LATE          every link in the chain had a covering transform on the
                     wire before the drop, within the spread of its own
                     neighbours. Nothing was missing; the node did not consume
                     what it had in time. Three of the four in-window b2maps
                     drops are this, and so is b2maps_k1's out-of-window one,
                     all four firing 0.492-0.500 s after the scan reached the
                     wire -- which is the costmap's own transform_tolerance.

WHY THE LOG CANNOT TELL YOU THIS. Every one of those drops is logged with the
same reason, 'the timestamp on the message is earlier than all the data in the
transform cache', and that sentence is not a diagnosis. In
/opt/ros/jazzy/include/tf2_ros/tf2_ros/message_filter.hpp:551-557 the reason is
assigned by a catch-all:

    FilterFailureReason error = filter_failure_reasons::Unknown;
    try {
      future.get();
    } catch (...) {
      transform_available = false;
      error = filter_failure_reasons::OutTheBack;
    }

`OutTheBack` is the enum whose string that is (lines 105-107). The future comes
from buffer_.waitForTransform(target, frame_id, stamp, buffer_timeout_, cb) at
line 440, so ANY failure or timeout of that asynchronous request is reported
with a sentence about the cache. None of the five drops in the b2maps corpus
was older than the cache: each was 0.49-0.64 s old against tf2's 10 s default.

WIRE TIME IS A PROXY. Every 'on the wire' time here is the RECORDER's receive
stamp for that message, which is the closest the bag can come to when the
costmap's own process saw it. The node's copy travels the same DDS publication
but lands in its buffer only once its executor runs the TransformListener
callback -- setTransform() -> testTransformableRequests() (tf2 BufferCore) ->
the filter's transformReadyCallback. So "on the wire before the drop" does NOT
prove the transform was in the node's buffer; it proves the transform existed
and was published in time, which is the half the bag can testify to. A NONE
LATE verdict is therefore a statement about the publisher, and points at the
consumer by elimination.

No thresholds. The +-NEIGHBOURS scan baseline is printed for scale and nothing
is compared against it: two of the four in-window drops have different causes,
so any bound fitted here would be fitted to k2.

Usage (ROS sourced):
    python3 experiments/analysis/attribute_tf_drop.py \
        --bag <bag dir> --robot K --log <explore.log> [--min-sim 1200]
        [--nav2-params experiments/nav/nav2_robotK.yaml]

Exit status is 0 whenever the attribution ran; this is a report, not a gate.
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

# explore.launch.py:72 and :128 -- the params file the run gets unless
# nav2_params_file is passed, which the b2maps procedure does not do.
EXP_NAV = REPO_ROOT / 'experiments' / 'nav'

# How much of the bag to read around a drop. The scan is at most ~1 s older
# than the drop and the neighbour baseline reaches +-4 s, so BEFORE is
# generous; AFTER has to be long enough that "no covering transform" means the
# link really stalled rather than that the read stopped. Both are printed with
# the result, because a link reported missing is missing WITHIN THIS REGION.
REGION_BEFORE_S = 60.0
REGION_AFTER_S = 120.0

# Scans either side of the dropped one, for scale. Reported, never compared.
NEIGHBOURS = 20

# 'frame '<id>' at time <sim>' out of a message-filter drop, and the requested
# time out of tf2's extrapolation error. The tf_help phrasing ('Transform data
# too old when converting from A to B') carries NO time at all, so a warning in
# that form cannot be attributed this way -- see main().
REQUESTED_TIME_RE = re.compile(r'Requested time ([0-9]+\.[0-9]+)')


# ───────────────────────────── the params, read ──────────────────────────────

def nav2_params_path(robot: int) -> Path:
    """The file explore.launch.py hands nav2 for robot K, by its default."""
    return EXP_NAV / f'nav2_robot{robot}.yaml'


def _line_in_block(lines, block: str, key: str):
    """The 1-based line of `key:` inside the top-level `block:`, or None.

    Block-scoped on purpose. The same setting appears in several nodes of these
    files -- nav2_robot2.yaml carries `global_frame: robot_2/map` at :26 for
    bt_navigator and again at :222 for the global costmap -- so a search over
    the whole file returns a real line that documents the wrong node.
    """
    depth = None
    for i, ln in enumerate(lines, start=1):
        stripped = ln.strip()
        indent = len(ln) - len(ln.lstrip())
        if stripped == f'{block}:' and depth is None:
            depth = indent
            continue
        if depth is None:
            continue
        if stripped and indent <= depth and not stripped.startswith('#'):
            return None                     # left the block without a hit
        if stripped.startswith(f'{key}:'):
            return i
    return None


def costmap_facts(path: Path) -> dict:
    """{'local'|'global': {'global_frame', 'transform_tolerance', lines}}.

    Values by yaml, provenance by locating each key inside its own costmap
    block, so a printed number can be checked against the line that sets it.
    """
    if not path.is_file():
        raise SystemExit(f'FATAL: no nav2 params at {path}')
    text = path.read_text()
    doc = yaml.safe_load(text)
    lines = text.splitlines()
    out = {}
    for which in ('local', 'global'):
        key = f'{which}_costmap'
        block = doc[key][key]['ros__parameters']
        out[which] = {
            'global_frame': block['global_frame'],
            'transform_tolerance': block['transform_tolerance'],
            'frame_line': _line_in_block(lines, key, 'global_frame'),
            'tol_line': _line_in_block(lines, key, 'transform_tolerance'),
        }
    return out


def costmap_of(line: str) -> str | None:
    """'local' or 'global', from the logger the drop was written under --
    [robot_3.local_costmap.local_costmap]. Which costmap it was decides which
    global_frame the chain starts at, and nothing else in the line says.
    """
    for which in ('local', 'global'):
        if f'{which}_costmap' in line:
            return which
    return None


# ────────────────────────────── the frame chain ──────────────────────────────

def chain_from(parent_of: dict, source: str, target: str):
    """The edges from `source` up to `target`, as [(parent, child), ...]. Pure.

    A tf tree gives every frame exactly one parent, so walking up from the
    scan frame is the whole search. Returns None if `target` is not an
    ancestor of `source`, which is a broken tree rather than a late transform.
    """
    edges, seen, node = [], {source}, source
    while node != target:
        parent = parent_of.get(node)
        if parent is None or parent in seen:
            return None
        edges.append((parent, node))
        seen.add(parent)
        node = parent
    return edges


def dynamic_links(chain, static_links):
    """The edges of `chain` that are not in /tf_static. Pure.

    Derived, not listed: on these bags base_footprint -> base_link ->
    base_scan are all static, so the dynamic remainder of
    map -> odom -> base_footprint -> base_link -> base_scan is exactly
    map -> odom and odom -> base_footprint. A world that moved a joint into
    /tf would change that, and this would follow it.
    """
    return [e for e in chain if e not in static_links]


# ─────────────────────────── the attribution itself ──────────────────────────

def first_covering(rows, sim: float):
    """(stamp, wire time) of the first transform stamped at or after `sim`, or
    (None, None). Pure.

    `rows` is in publication order, so the first match is the earliest such
    stamp. A transform stamped exactly at `sim` counts: that is the sample the
    filter needs.
    """
    for stamp, recv in rows:
        if stamp >= sim:
            return stamp, recv
    return None, None


def attribute_drop(links, tf: dict, sim: float, drop_wall: float) -> dict:
    """Per link: the first covering transform, and whether it was late. Pure.

    late means the transform reached the wire after the node had already given
    up, or never appeared at all within the region read. Those are the two
    ways a link can fail to answer for the dropped message; everything else is
    the link having done its job.
    """
    out = {}
    for link in links:
        stamp, recv = first_covering(tf.get(link, ()), sim)
        out[link] = {
            'stamp': stamp,
            'recv': recv,
            'stamp_lag_ms': None if stamp is None else (stamp - sim) * 1e3,
            'vs_drop_s': None if recv is None else recv - drop_wall,
            'missing': stamp is None,
            'late': stamp is None or recv > drop_wall,
        }
    return out


def late_links(attribution) -> list:
    """The links that did not answer in time, in chain order. Pure."""
    return [link for link, d in attribution.items() if d['late']]


def last_to_arrive(attribution):
    """The link whose covering transform reached the wire last. Pure.

    None if no link had one at all. This is the link that decided the outcome,
    late or not.
    """
    have = {k: d for k, d in attribution.items() if d['recv'] is not None}
    if not have:
        return None
    return max(have.items(), key=lambda kv: kv[1]['recv'])[0]


# ──────────────────────────────── bag reading ────────────────────────────────

def read_static_links(bag: Path) -> set:
    """Every (parent, child) in /tf_static."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    if '/tf_static' not in {t.name for t in reader.get_all_topics_and_types()}:
        return set()
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
    out = set()
    while reader.has_next():
        _t, data, _recv = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            out.add((tr.header.frame_id, tr.child_frame_id))
    return out


def read_region(bag: Path, scan_topic: str, lo: float, hi: float):
    """({(parent, child): [(stamp, wire), ...]}, [(stamp, wire), ...] scans).

    seek() first, so a 1 GB bag is not read end to end for a three-minute
    region. Every pair in /tf is collected rather than a chosen two: which
    links matter is decided by the chain, which is not known until /tf_static
    and the params have been read.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    available = {t.name for t in reader.get_all_topics_and_types()}
    topics = [t for t in ('/tf', scan_topic) if t in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    reader.seek(int(lo * 1e9))

    tf, scans = {}, []
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        recv = recv_ns / 1e9
        if recv < lo:
            continue
        if recv > hi:
            break
        if topic == scan_topic:
            m = deserialize_message(data, LaserScan)
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
                          recv))
        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                tf.setdefault(key, []).append(
                    (tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9,
                     recv))
    return tf, scans


def parent_map(tf: dict, static_links: set) -> dict:
    """child -> parent over both /tf and /tf_static."""
    out = {child: parent for parent, child in static_links}
    for parent, child in tf:
        out[child] = parent
    return out


# ───────────────────────────────── reporting ─────────────────────────────────

def covering_vs_scan(links, tf, stamp: float, wire: float) -> dict:
    """Per link: the covering transform's wire time minus the SCAN's own wire
    time, in seconds. Pure.

    This is the quantity the neighbour baseline is in, and the dropped scan's
    value is in the same one, so the two are comparable. Measuring the
    neighbours against the DROP's wall time instead would make later scans look
    progressively worse for no reason other than being later.

    Both terms are receive stamps off one clock, so no /clock is needed and no
    sim-to-wall conversion enters the comparison.
    """
    out = {}
    for link in links:
        _stamp, recv = first_covering(tf.get(link, ()), stamp)
        out[link] = None if recv is None else recv - wire
    return out


def neighbour_baseline(links, tf, scans, idx):
    """covering_vs_scan for the NEIGHBOURS scans either side of `idx`.

    Reported for scale, never compared against: two of the four in-window
    b2maps drops have different causes, so a bound fitted here would be fitted
    to one of them.
    """
    out = {link: [] for link in links}
    for j in range(max(0, idx - NEIGHBOURS),
                   min(len(scans), idx + NEIGHBOURS + 1)):
        if j == idx:
            continue
        stamp, wire = scans[j]
        for link, v in covering_vs_scan(links, tf, stamp, wire).items():
            if v is not None:
                out[link].append(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--robot', type=int, required=True)
    ap.add_argument('--log', type=Path, required=True,
                    help='explore log whose gated lines are attributed')
    ap.add_argument('--nav2-params', type=Path,
                    help='default: the file explore.launch.py would pass '
                         '(experiments/nav/nav2_robotK.yaml)')
    ap.add_argument('--min-sim', type=float, dest='min_sim',
                    help="R9's window length in sim seconds "
                         '(default: run_health.MIN_SIM_S)')
    args = ap.parse_args()

    # R9's own reader, window and classifier. Nothing here re-derives which
    # lines are gated: this tool explains that list, it does not recompute it.
    # Imported in the function so the pure half above stays importable BY
    # run_health without a cycle.
    from run_health import (MIN_SIM_S, count_health, is_slam_output_pair,
                            read_analysis_window, read_log)

    min_sim = MIN_SIM_S if args.min_sim is None else args.min_sim
    window, why = read_analysis_window(args.bag, args.robot, min_sim)
    if window is None:
        raise SystemExit(f'FATAL: no analysis window for {args.bag}: {why}')
    found = count_health(read_log(args.log), window)
    hits = found['gated']

    params = args.nav2_params or nav2_params_path(args.robot)
    facts = costmap_facts(params)
    static_links = read_static_links(args.bag)
    scan_topic = f'/robot_{args.robot}/scan'

    print(f'bag    {args.bag}')
    print(f'log    {args.log}')
    print(f'params {params.relative_to(REPO_ROOT)}')
    for which, f in facts.items():
        print(f'       {which:<6} global_frame {f["global_frame"]:<22} '
              f'(:{f["frame_line"]})   transform_tolerance '
              f'{f["transform_tolerance"]} (:{f["tol_line"]})')
    print(f'window sim [{window["sim_lo"]:.3f}, {window["sim_hi"]:.3f}], wall '
          f'[{window["recv_lo"]:.6f}, {window["recv_hi"]:.6f}]')
    print(f'gated  {len(hits)} line(s) inside it')
    print(f'static links in /tf_static: {len(static_links)}')
    print()

    rows = []
    for hit in hits:
        which = costmap_of(hit['line'])
        if which is not None:
            frame, sim = hit['frame'], hit['scan']
            target = facts[which]['global_frame']
            kind = f'{which} costmap'
        else:
            # A transform warning. Attributable only when the line states the
            # time it asked for: tf2's extrapolation error does, and the
            # tf_help 'Transform data too old' phrasing carries no time at
            # all, so that one is reported and skipped.
            m = REQUESTED_TIME_RE.search(hit['line'])
            if m is None or hit['pair'] is None:
                print(f'not attributable: {hit["cat"]} at {hit["stamp"]:.6f} '
                      'states no requested time, so there is no instant to '
                      'ask the bag about')
                print(f'    {hit["line"]}')
                continue
            frame, sim = hit['pair'].split(' -> ')[1], float(m.group(1))
            target = hit['pair'].split(' -> ')[0]
            kind = f'{hit["proc"]} warning'

        lo = hit['stamp'] - REGION_BEFORE_S
        hi = hit['stamp'] + REGION_AFTER_S
        tf, scans = read_region(args.bag, scan_topic, lo, hi)
        chain = chain_from(parent_map(tf, static_links), frame, target)
        if chain is None:
            print(f'no tf chain from {frame} up to {target} in this region')
            continue
        links = dynamic_links(chain, static_links)
        attribution = attribute_drop(links, tf, sim, hit['stamp'])

        wire = None
        idx = None
        if scans:
            idx = min(range(len(scans)), key=lambda i: abs(scans[i][0] - sim))
            if abs(scans[idx][0] - sim) < 5e-4:
                wire = scans[idx][1]
            else:
                idx = None
        base = (neighbour_baseline(links, tf, scans, idx)
                if idx is not None else {link: [] for link in links})
        own = (covering_vs_scan(links, tf, sim, wire) if wire is not None
               else {link: None for link in links})

        late = [ln for ln in late_links(attribution)]
        rig = [ln for ln in late
               if not is_slam_output_pair(f'{ln[0]} -> {ln[1]}')]
        rows.append(dict(hit=hit, kind=kind, sim=sim, wire=wire, links=links,
                         attribution=attribution, base=base, own=own,
                         late=late, rig=rig, last=last_to_arrive(attribution),
                         region=(lo, hi), chain=chain))

    def _s(v, unit=' s', fmt='+.3f'):
        return '-' if v is None else f'{v:{fmt}}{unit}'

    print('| logged (wall) | kind | chain | asked for | scan on wire | '
          'logged-wire | link | first tf >= asked | stamp lag | tf wire vs '
          'LOGGED | tf wire vs SCAN | neighbours med / max | last | verdict |')
    print('|' + '---|' * 14)
    for r in rows:
        hit = r['hit']
        verdict = ('none late' if not r['late'] else
                   ', '.join(f'{a.split("/")[-1]} -> {b.split("/")[-1]} LATE'
                             for a, b in r['late']))
        for link in r['links']:
            d = r['attribution'][link]
            b = r['base'][link]
            wire = '-' if r['wire'] is None else f'{r["wire"]:.3f}'
            gap = _s(None if r['wire'] is None
                     else hit['stamp'] - r['wire'])
            stamp = 'MISSING' if d['stamp'] is None else f'{d["stamp"]:.3f}'
            spread = ('-' if not b else
                      f'{statistics.median(b):+.3f} / {max(b):+.3f} s')
            print(f'| {hit["stamp"]:.3f} | {r["kind"]} '
                  f'| {r["chain"][0][1].split("/")[-1]} -> '
                  f'{r["chain"][-1][0].split("/")[-1]} '
                  f'| {r["sim"]:.3f} | {wire} | {gap} '
                  f'| {link[0].split("/")[-1]} -> {link[1].split("/")[-1]} '
                  f'| {stamp} | {_s(d["stamp_lag_ms"], " ms", "+.0f")} '
                  f'| {_s(d["vs_drop_s"])} | {_s(r["own"][link])} '
                  f'| {spread} | {"yes" if link == r["last"] else ""} '
                  f'| {verdict} |')
    print()

    for r in rows:
        hit = r['hit']
        print(f'{hit["stamp"]:.6f}  {r["kind"]}  region read '
              f'[{r["region"][0]:.1f}, {r["region"][1]:.1f}]')
        print('    chain  ' + ' -> '.join(
            [r['chain'][-1][0]] + [c for _p, c in reversed(r['chain'])]))
        print(f'    dynamic links  {len(r["links"])} of {len(r["chain"])}  '
              f'(the rest are in /tf_static)')
        if not r['late']:
            print('    NONE LATE: every link had a covering transform on the '
                  'wire before the line was logged.')
            print('    The transforms existed and were published in time; '
                  'what did not happen in time')
            print('    was the node consuming them. See WIRE TIME IS A PROXY.')
        else:
            for link in r['late']:
                d = r['attribution'][link]
                if d['missing']:
                    print(f'    LATE: {link[0]} -> {link[1]} has NO '
                          f'transform stamped at or after {r["sim"]:.3f} '
                          'anywhere in the region read')
                else:
                    print(f'    LATE: {link[0]} -> {link[1]} first covers '
                          f'{r["sim"]:.3f} at stamp {d["stamp"]:.3f}, which '
                          f'reached the wire {d["vs_drop_s"] * 1e3:+.0f} ms '
                          'after the line was logged')
            if not r['rig']:
                print('    every late link is a same-namespace map <-> odom '
                      'pair: slam_toolbox\'s own output')
        print(f'    {hit["line"]}')
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
