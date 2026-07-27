"""Layer 2 — the launch files build, and contain the nodes they promise.

swarm_sim.launch.py and explore.launch.py carry real logic (SDF/URDF rewriting,
RewrittenYaml re-keying, namespace pushes, lifecycle transition wiring) and had
zero test coverage: the only way a mis-edit surfaced was a lost sim boot —
Gazebo + SLAM + Nav2 spun up for minutes before the missing node showed itself.
These tests CONSTRUCT both descriptions and walk the resulting entity tree.

Nothing is executed here: no process spawns, no ROS graph, no rclpy. So this
catches exactly the class of mistake that used to cost a boot — an import or
substitution error, a node dropped from the tree, a package/executable renamed,
a node that quietly stopped being launched — and nothing that only appears once
the actions actually RUN (a wrong param value, a missing world file, a node
that starts and then dies). That is the trade these tests are here to make.

Walk semantics: `_walk` descends into GroupActions, TimerActions and resolved
IncludeLaunchDescriptions, and it IGNORES conditions — an entity guarded by an
IfCondition is still reported. So "present in the tree" means "declared", not
"guaranteed to run" (nav2's own use_composition branches are the case in point).
"""

import importlib.util
import os
import shutil
import tempfile

import pytest

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch_ros.actions import LifecycleNode, Node, PushROSNamespace, SetRemap

# Set to 1 by .github/workflows/ci.yml, which installs Nav2 on purpose: there,
# anything missing is a broken image, so every skip below becomes a hard
# failure. These tests exist to catch launch-description defects that review
# misses — a skipped one in CI is decoration. Local shells without Nav2 still
# skip cleanly. Only the tests that actually need Nav2 are affected; the
# swarm_sim ones require none of it and always run.
REQUIRE_NAV2 = os.environ.get('NSK_REQUIRE_NAV2', '') not in ('', '0', 'false')

# frontier_explorer imports nav2_simple_commander at module level (see
# test_frontier_explorer_gate.py). Only one test below needs this constant, so
# guard the import rather than skipping the whole module.
try:
    from nsk_swarm.frontier_explorer import NAV2_REQUIRED_SERVERS
except ImportError:
    if REQUIRE_NAV2:
        raise
    NAV2_REQUIRED_SERVERS = None


LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch')


@pytest.fixture(scope='module', autouse=True)
def scratch_tempdir():
    """Redirect tempfile to a scratch dir for the duration of this module.

    Both launch files write NamedTemporaryFile(delete=False) copies of stock
    assets they rewrite — swarm_sim's five namespaced burger SDFs, explore's
    ROBOT_NS-substituted RViz config. Both writes happen when the action that
    consumes them is PERFORMED, not when the description is built, so the tests
    below currently trigger neither: they build the tree and walk it, and the
    one that resolves an OpaqueFunction resolves the rviz:=false path that
    returns nothing. Kept as a cheap guard for the moment a test does perform
    one — pointing tempfile.tempdir at a directory we delete afterwards keeps
    /tmp clean without touching either launch file's behaviour.
    """
    scratch = tempfile.mkdtemp(prefix='nsk_launch_test_')
    previous = tempfile.tempdir
    tempfile.tempdir = scratch
    yield scratch
    tempfile.tempdir = previous
    shutil.rmtree(scratch, ignore_errors=True)


def load(filename):
    """Import a launch file from the source tree and return the module.

    Launch files aren't importable as package modules ('.launch.py' is not an
    identifier), so they're loaded by location — the same file `ros2 launch`
    reads. Importing only runs module-level code, and build() below touches no
    disk either: swarm_sim.launch.py reads the stock TurtleBot3 SDF/URDF behind
    a Substitution (LazyRobotAsset), so the read happens only once the spawn
    and robot_state_publisher actions are performed. That is what lets these
    tests run in a container without turtlebot3_gazebo installed.
    """
    path = os.path.join(LAUNCH_DIR, filename)
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build(filename):
    """Return the LaunchDescription `filename` generates."""
    try:
        return load(filename).generate_launch_description()
    except PackageNotFoundError as exc:
        # An unfindable package share dir means the environment isn't sourced
        # (or Nav2 isn't installed), not that the launch file is broken; make
        # the skip actionable. In CI every package IS installed, so the same
        # condition is a broken image and must fail the job, not vanish.
        if REQUIRE_NAV2:
            raise
        pytest.skip(f'{filename}: package {exc} not found — source the ROS 2 '
                    f'and workspace environments to run this test')


def resolved(launch_description, **arguments):
    """Return `launch_description` with its OpaqueFunctions expanded.

    An OpaqueFunction is a black box until a LaunchContext performs it, so a
    node it gates is absent from the raw tree whether the gate is working or
    not — asserting on the raw tree would pass vacuously. This applies the same
    sequence `ros2 launch` does: command-line arguments land in the context
    first, then each DeclareLaunchArgument fills in its default for whatever is
    still unset, then the functions run against that.

    `arguments` are the `name:=value` overrides, as the strings launch would
    hand over (rviz='true', not rviz=True).
    """
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    expanded = []
    for entity in launch_description.entities:
        if isinstance(entity, OpaqueFunction):
            expanded += entity.execute(context) or []
        else:
            expanded.append(entity)
    return LaunchDescription(expanded)


def declared_defaults(launch_description):
    """The launch arguments' declared defaults, as launch would resolve them."""
    context = LaunchContext()
    defaults = {}
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
            defaults[entity.name] = context.launch_configurations[entity.name]
    return defaults


def _walk(entity, seen=None):
    """Yield `entity` and every sub-entity reachable from it, once each.

    Covers the three containers these launch files use: LaunchDescription
    (.entities), GroupAction / IncludeLaunchDescription (.get_sub_entities(),
    which resolves an include's file without a launch context) and TimerAction
    (.describe_conditional_sub_entities(), where every delayed node lives).
    """
    seen = set() if seen is None else seen
    if id(entity) in seen:
        return
    seen.add(id(entity))
    yield entity

    children = []
    if isinstance(entity, LaunchDescription):
        children += list(entity.entities)
    try:
        children += list(entity.get_sub_entities())
    except Exception:                   # noqa: BLE001
        # An include that can't be resolved without a context contributes
        # nothing; tests that need one skip explicitly rather than fail here.
        pass
    try:
        for _description, actions in entity.describe_conditional_sub_entities():
            children += list(actions)
    except Exception:                   # noqa: BLE001
        pass

    for child in children:
        yield from _walk(child, seen)


def nodes(entity):
    """Every Node/LifecycleNode under `entity` as (package, executable) pairs.

    Both fields are plain strings in these launch files (and in nav2's), so no
    substitution needs performing; anything else reports as None, so a
    substitution creeping in shows up as a failed match rather than a crash.
    """
    def text(value):
        return value if isinstance(value, str) else None

    return [(text(e.node_package), text(e.node_executable))
            for e in _walk(entity) if isinstance(e, Node)]


def delayed(launch_description):
    """(package, executable) pairs that only run after a TimerAction fires."""
    pairs = []
    for entity in _walk(launch_description):
        if isinstance(entity, TimerAction):
            pairs += nodes(entity)
    return pairs


# ── explore.launch.py ────────────────────────────────────────────────────────

def test_explore_launches_slam_nav2_and_the_explorer():
    ld = build('explore.launch.py')
    present = nodes(ld)

    # SLAM must be a LifecycleNode, not a plain Node: launched plain it sits in
    # 'unconfigured' forever, never publishes map->odom, and Nav2's global
    # costmap then times out on a robot_<id>/map frame that never exists.
    slam = [e for e in _walk(ld)
            if isinstance(e, LifecycleNode) and e.node_package == 'slam_toolbox']
    assert len(slam) == 1, 'slam_toolbox must be launched exactly once'
    assert slam[0].node_executable == 'async_slam_toolbox_node'

    assert present.count(('nsk_swarm', 'frontier_explorer')) == 1


def test_explore_nav2_group_pushes_the_namespace_and_keeps_the_global_tf():
    # The Nav2 include is wrapped in a GroupAction that must do two things:
    # push robot_<id> (navigation_launch.py sets no namespace on its servers,
    # so without this they come up at root), and shadow the stock ('/tf','tf')
    # remap on BOTH tf topics so Nav2 stays on the shared global TF tree
    # instead of an isolated, empty /robot_<id>/tf.
    ld = build('explore.launch.py')

    nav2_groups = [list(e.get_sub_entities()) for e in ld.entities
                   if isinstance(e, GroupAction)
                   and any(isinstance(s, IncludeLaunchDescription)
                           for s in e.get_sub_entities())]
    assert len(nav2_groups) == 1, 'expected exactly one Nav2 include group'
    subs = nav2_groups[0]

    assert any(isinstance(s, PushROSNamespace) for s in subs)
    assert sum(isinstance(s, SetRemap) for s in subs) == 2


@pytest.mark.skipif(NAV2_REQUIRED_SERVERS is None,
                    reason='nav2_simple_commander is not importable — install '
                           'ros-jazzy-navigation2 to run this test')
def test_explore_nav2_include_brings_up_the_servers_the_gate_waits_on():
    # frontier_explorer's readiness gate blocks on the lifecycle state of
    # NAV2_REQUIRED_SERVERS, by name. If the include stopped launching one of
    # them (or nav2 renamed it), the gate would block until its timeout and the
    # run would abort with a confusing "never reached active" — so pin the two
    # together here, where the check costs a second instead of a boot.
    ld = build('explore.launch.py')
    executables = {executable for _package, executable in nodes(ld)}
    if 'bt_navigator' not in executables:
        # In CI nav2_bringup is installed, so an unresolvable include is a real
        # defect (or a nav2 layout change) and must fail rather than skip.
        assert not REQUIRE_NAV2, (
            'nav2_bringup navigation_launch.py did not resolve to its server '
            "nodes — the include or nav2's launch layout changed")
        pytest.skip('nav2_bringup navigation_launch.py could not be resolved '
                    'without a launch context on this install')
    for server in NAV2_REQUIRED_SERVERS:
        assert server in executables, (
            f'{server} is in frontier_explorer.NAV2_REQUIRED_SERVERS but is '
            f'not launched by explore.launch.py')


def test_explore_stages_nothing_on_a_timer():
    # Deliberate: readiness is gated on lifecycle STATE (frontier_explorer's
    # _wait_for_nav2_servers_active), never on a fixed delay. A TimerAction
    # here would be tuned to one machine's real-time factor and would silently
    # regress on any other — the exact failure mode the state gate replaced.
    ld = build('explore.launch.py')
    assert delayed(ld) == [], (
        'explore.launch.py must not stage bringup with TimerAction — gate on '
        'lifecycle state instead (see frontier_explorer.NAV2_REQUIRED_SERVERS)')


# ── swarm_sim.launch.py ──────────────────────────────────────────────────────

def test_swarm_sim_launches_one_robot_node_per_robot():
    ld = build('swarm_sim.launch.py')
    present = nodes(ld)
    num_robots = load('swarm_sim.launch.py').NUM_ROBOTS

    assert present.count(('nsk_swarm', 'robot_node')) == num_robots
    assert present.count(('ros_gz_sim', 'create')) == num_robots
    # One robot_state_publisher per robot supplies the static frame chain that
    # each robot's bridged dynamic odom->base_footprint transform hangs off.
    assert present.count(
        ('robot_state_publisher', 'robot_state_publisher')) == num_robots


def test_swarm_sim_launches_the_engine_and_the_monitor():
    ld = build('swarm_sim.launch.py')

    engine = [e for e in _walk(ld) if isinstance(e, LifecycleNode)
              and e.node_executable == 'nsk_engine']
    assert len(engine) == 1, 'the NSK engine must be a single LifecycleNode'
    assert ('nsk_swarm', 'convergence_monitor') in nodes(ld)


RVIZ = ('rviz2', 'rviz2')


def test_swarm_sim_does_not_launch_rviz_by_default():
    # RViz launched here unconditionally on a 7 s timer and silently corrupted
    # every RTF and coverage number this repo has recorded: rendering the
    # 5-robot scene took /clock from ~99 Hz to 0.500 Hz, and from there the
    # robot nodes timed out on /nsk/similarity_query and /nsk/compress and
    # bt_navigator could not answer its own get_state. A default-args boot must
    # contain no rviz2 process at all.
    ld = build('swarm_sim.launch.py')
    assert declared_defaults(ld)['rviz'] == 'false'
    assert RVIZ not in nodes(resolved(ld))


def test_swarm_sim_launches_rviz_on_request_after_the_seven_second_delay():
    # The opt-in path must still work, and still start last — after the robot
    # nodes and the monitor, so RViz subscribes to topics that already exist.
    ld = build('swarm_sim.launch.py')
    with_rviz = resolved(ld, rviz='true')
    assert RVIZ in nodes(with_rviz)
    assert RVIZ in delayed(with_rviz)


def test_explore_does_not_launch_rviz_by_default():
    # Same gate, same reason, in the launch file that already had it — the RTF
    # this rig measures is only meaningful if NEITHER file renders by default.
    ld = build('explore.launch.py')
    assert declared_defaults(ld)['rviz'] == 'false'
    assert RVIZ not in nodes(resolved(ld))


def test_swarm_sim_clock_bridge_starts_immediately():
    # Every use_sim_time node in the swarm stays frozen at t=0 until /clock
    # publishes, so the clock bridge must NOT sit behind a TimerAction the way
    # the topic bridge and the robot nodes do. (25 nodes once ran against a
    # frozen clock because this bridge was missing entirely.)
    ld = build('swarm_sim.launch.py')
    bridge = ('ros_gz_bridge', 'parameter_bridge')

    assert bridge in nodes(ld)
    assert bridge not in delayed(ld), (
        'the /clock bridge must start immediately, not on a timer')
