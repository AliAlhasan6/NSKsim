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
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.utilities import perform_substitutions
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
    from nsk_swarm.frontier_explorer import (ESCAPE_DISTANCE,
                                             NAV2_REQUIRED_SERVERS)
except ImportError:
    if REQUIRE_NAV2:
        raise
    ESCAPE_DISTANCE = NAV2_REQUIRED_SERVERS = None


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


def enabled(launch_description, **arguments):
    """The top-level entities whose condition passes, as launch evaluates it.

    `_walk` ignores conditions on purpose (see the module docstring), so a gated
    entity sits in the tree whether its gate works or not — asserting on the
    tree alone would pass for EVERY value of the flag. This asks each entity's
    condition instead, against a context filled the way `resolved` fills one:
    command-line `arguments` first, then each DeclareLaunchArgument supplies its
    default for whatever is still unset.

    Only the top level, which is where this file's conditions live. Unlike
    ParameterValue (see `explorer_parameter`), a Condition caches nothing — it
    re-performs its substitutions per call — so one built description can be
    probed with several argument sets.
    """
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return [entity for entity in launch_description.entities
            if entity.condition is None or entity.condition.evaluate(context)]


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


def test_the_nav2_flag_gates_nav2_and_the_explorer_but_never_slam():
    # nav2:=false is the mapper-only rung: slam_toolbox and its two lifecycle
    # transitions, nothing else. The explorer rides the SAME flag on purpose —
    # FrontierExplorer is a BasicNavigator, so with no stack under it there is
    # nothing it could do — which makes "Nav2 off, explorer on" a combination
    # that must not be reachable. SLAM switches with neither: gating it would
    # leave a launch that brings up nothing at all.
    ld = build('explore.launch.py')

    def surviving(**arguments):
        kept = enabled(ld, **arguments)
        return {
            'slam': sum(isinstance(e, LifecycleNode)
                        and e.node_package == 'slam_toolbox' for e in kept),
            'configure': sum(isinstance(e, EmitEvent) for e in kept),
            'activate': sum(isinstance(e, RegisterEventHandler) for e in kept),
            # The Nav2 group is the one holding the include (same predicate as
            # test_explore_nav2_group_pushes_the_namespace_and_keeps_the_global_tf).
            'nav2': sum(isinstance(e, GroupAction)
                        and any(isinstance(s, IncludeLaunchDescription)
                                for s in e.get_sub_entities())
                        for e in kept),
            'explorer': nodes(LaunchDescription(kept)).count(
                ('nsk_swarm', 'frontier_explorer')),
        }

    mapper_only = surviving(nav2='false')
    assert mapper_only == {'slam': 1, 'configure': 1, 'activate': 1,
                           'nav2': 0, 'explorer': 0}, (
        f'nav2:=false must leave SLAM alone and drop the rest, got {mapper_only}')

    whole_stack = {'slam': 1, 'configure': 1, 'activate': 1,
                   'nav2': 1, 'explorer': 1}
    # MUST hold for the bare default: every command that does not name nav2 has
    # to reproduce its previous rung exactly.
    assert surviving() == whole_stack, 'the default must bring up everything'
    assert surviving(nav2='true') == whole_stack


@pytest.mark.skipif(ESCAPE_DISTANCE is None, reason='needs nav2_simple_commander')
def test_the_escape_distance_default_matches_the_module_constant():
    # explore.launch.py declares this default as a LITERAL, on purpose: it
    # imports nothing from nsk_swarm, so that a launch description never drags
    # rclpy and nav2_simple_commander in. This is the price of that choice —
    # the two spellings of one number, held equal here rather than by hope.
    ld = build('explore.launch.py')
    declared = [e for e in _walk(ld)
                if isinstance(e, DeclareLaunchArgument)
                and e.name == 'escape_distance']

    assert len(declared) == 1, 'escape_distance must be a launch argument'
    assert float(declared[0].default_value[0].text) == ESCAPE_DISTANCE


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


def node_parameters(filename, executable, name, **arguments):
    """Evaluate one parameter on every Node running `executable`, as launch
    would. Returns a list, one entry per matching node, in tree order.

    Command-line `arguments` land in the context first and the declared defaults
    fill in the rest (same sequence as `resolved`), then the ParameterValue is
    evaluated against it — which is what applies its `value_type` cast.
    OpaqueFunctions are expanded against that SAME context, so nodes that only
    exist for some argument values (swarm_sim's truth_odom_tf) are reachable.

    Two launch_ros details this has to work around, both of which silently
    return a WRONG answer rather than raising:

    1. The parameter names in a Node's dict block are not strings. Node.__init__
       runs normalize_parameters() at construction, and normalize_parameter_dict
       rewrites every key through normalize_to_list_of_substitutions and stores
       it as a tuple of TextSubstitution. So `name in block` is false for every
       parameter on every Node; the keys have to be performed before comparing.

    2. ParameterValue.evaluate() caches into __evaluated_parameter_value, and
       the .value property then returns that cached SCALAR instead of the
       original substitution — so a second evaluate() on the same object
       re-evaluates a plain bool and hands back the FIRST answer whatever the
       new context says. Hence the build per call: each one gets untouched
       ParameterValue objects, which is also what a real `ros2 launch` does.
       Sharing one description across calls makes every spelling resolve to
       whatever the first one did.
    """
    launch_description = build(filename)

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

    found = []
    for node in _walk(LaunchDescription(expanded)):
        if not isinstance(node, Node) or node.node_executable != executable:
            continue
        for block in node._Node__parameters:
            if not isinstance(block, dict):
                continue
            for key, value in block.items():
                if perform_substitutions(context, list(key)) != name:
                    continue
                found.append(value.evaluate(context)
                             if hasattr(value, 'evaluate') else value)
    return found


def explorer_parameter(name, **arguments):
    """Evaluate one parameter on the frontier_explorer node, as launch would.

    Command-line `arguments` land in the context first and the declared defaults
    fill in the rest (same sequence as `resolved`), then the ParameterValue is
    evaluated against it — which is what applies its `value_type` cast.

    Two launch_ros details this has to work around, both of which silently
    return a WRONG answer rather than raising:

    1. The parameter names in a Node's dict block are not strings. Node.__init__
       runs normalize_parameters() at construction, and normalize_parameter_dict
       rewrites every key through normalize_to_list_of_substitutions and stores
       it as a tuple of TextSubstitution. So `name in block` is false for every
       parameter on every Node; the keys have to be performed before comparing.

    2. ParameterValue.evaluate() caches into __evaluated_parameter_value, and
       the .value property then returns that cached SCALAR instead of the
       original substitution — so a second evaluate() on the same object
       re-evaluates a plain bool and hands back the FIRST answer whatever the
       new context says. Hence the build per call: each one gets untouched
       ParameterValue objects, which is also what a real `ros2 launch` does.
       Sharing one description across calls makes every spelling resolve to
       whatever the first one did.
    """
    launch_description = build('explore.launch.py')

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    explorer = [e for e in _walk(launch_description)
                if isinstance(e, Node) and e.node_executable == 'frontier_explorer']
    assert len(explorer) == 1, f'expected one explorer node, got {len(explorer)}'
    for block in explorer[0]._Node__parameters:
        if not isinstance(block, dict):
            continue
        for key, value in block.items():
            if perform_substitutions(context, list(key)) == name:
                return value.evaluate(context)
    raise AssertionError(f'{name} is not a parameter on the explorer node')


def test_explore_declares_the_precheck_argument_defaulting_off():
    # The A/B variable. A run that does not name it must reproduce the previous
    # rung exactly, so the default is what makes the comparison a comparison.
    ld = build('explore.launch.py')
    assert declared_defaults(ld)['reachability_precheck'] == 'false'


def test_precheck_argument_reaches_the_explorer_as_a_bool():
    # Both spellings of the A/B switch must arrive as actual bools.
    default = explorer_parameter('reachability_precheck')
    assert default is False, f'default resolved to {default!r}, not False'

    off = explorer_parameter('reachability_precheck',
                             reachability_precheck='false')
    assert off is False, f'reachability_precheck:=false resolved to {off!r}'

    on = explorer_parameter('reachability_precheck',
                            reachability_precheck='true')
    assert on is True, f'reachability_precheck:=true resolved to {on!r}'


def test_numeric_precheck_spellings_stay_bools():
    # What value_type=bool actually buys. launch_ros' type inference reads the
    # word forms right on its own, but infers '1' as the INT 1 — and main()
    # declares reachability_precheck with a False default, so rclpy would reject
    # an int for it and the explorer would die at startup on a spelling of "on"
    # that looks entirely reasonable at the command line.
    for raw, expected in (('1', True), ('0', False)):
        got = explorer_parameter('reachability_precheck',
                                 reachability_precheck=raw)
        assert got is expected, f'precheck:={raw} resolved to {got!r} ({type(got).__name__})'


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


def robot_parameter(name, **arguments):
    """Evaluate one parameter on every robot node, as launch would.

    The swarm_sim counterpart of `explorer_parameter`, working around the same
    two launch_ros traps for the same reasons — performed keys, and the fresh
    build per call that keeps ParameterValue's evaluation cache from making
    every spelling resolve to whatever the first one did.

    One difference: swarm_sim launches NUM_ROBOTS robot nodes, and
    robot_params() builds a separate ParameterValue for each, so this returns
    the list rather than a single value. A plumbing mistake that wired only
    robot 0 then shows up as a disagreement instead of passing on first hit.
    Parameters that are plain Python values (robot_id, comm_range) are passed
    through unevaluated; only substitutions carry an .evaluate.
    """
    launch_description = build('swarm_sim.launch.py')

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)

    robots = [e for e in _walk(launch_description)
              if isinstance(e, Node) and e.node_executable == 'robot_node']
    assert robots, 'expected at least one robot node'

    values = []
    for robot in robots:
        for block in robot._Node__parameters:
            if not isinstance(block, dict):
                continue
            for key, value in block.items():
                if perform_substitutions(context, list(key)) == name:
                    values.append(value.evaluate(context)
                                  if hasattr(value, 'evaluate') else value)
    assert len(values) == len(robots), (
        f'{name} reached {len(values)} of {len(robots)} robot nodes')
    return values


def test_swarm_sim_declares_the_obstacle_stop_distance_defaulting_to_the_node():
    # robot_node's own PARAMS default. A command that does not set it has to
    # reproduce the previous run exactly, so this default is what keeps an
    # unset run comparable.
    ld = build('swarm_sim.launch.py')
    assert declared_defaults(ld)['obstacle_stop_m'] == '0.6'


def test_obstacle_stop_m_reaches_every_robot_node_as_a_float():
    # The default and an override both have to arrive as actual floats, on
    # every robot rather than just the first.
    default = robot_parameter('obstacle_stop_m')
    assert default == [0.6] * len(default), f'default resolved to {default!r}'
    assert all(isinstance(v, float) for v in default)

    passed = robot_parameter('obstacle_stop_m', obstacle_stop_m='1.25')
    assert passed == [1.25] * len(passed), f'override resolved to {passed!r}'


def test_integer_obstacle_stop_spellings_stay_floats():
    # What value_type=float actually buys, and the reason escape_distance is
    # pinned the same way in explore.launch.py: launch_ros infers a bare '0.6'
    # as a float on its own, but 'obstacle_stop_m:=1' infers as the INT 1 —
    # and robot_node declares this parameter with a float default, which rclpy
    # will not accept an int for. Without the cast a robot would die at
    # startup on a spelling that looks entirely reasonable at the command line.
    for raw in ('1', '2'):
        got = robot_parameter('obstacle_stop_m', obstacle_stop_m=raw)
        assert got == [float(raw)] * len(got), (
            f'obstacle_stop_m:={raw} resolved to {got!r}')
        assert all(isinstance(v, float) for v in got), (
            f'obstacle_stop_m:={raw} gave '
            f'{[type(v).__name__ for v in got]}, not floats')


# ── truth odometry (swarm_sim.launch.py truth_odom_robots) ───────────────────

TRUTH_ODOM = ('nsk_swarm', 'truth_odom_tf')

# make_namespaced_burger_sdf reads the stock TurtleBot3 model off disk. The
# tests that call it directly therefore need that package; the ones that only
# walk the tree do not, which is the whole point of LazyRobotAsset.
BURGER_SDF_PRESENT = os.path.isfile(
    '/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')
needs_burger = pytest.mark.skipif(
    not BURGER_SDF_PRESENT and not REQUIRE_NAV2,
    reason='turtlebot3_gazebo not installed — nothing to rewrite')


def test_swarm_sim_declares_truth_odom_robots_defaulting_empty():
    # The A/B variable for the whole known-pose change. Every run recorded
    # before it existed has to reproduce exactly, so a command that does not
    # name it must behave as though the flag were not there.
    ld = build('swarm_sim.launch.py')
    assert declared_defaults(ld)['truth_odom_robots'] == '[]'


def test_swarm_sim_launches_no_truth_odom_node_by_default():
    # Absent from the tree, not merely condition-suppressed — the same gate
    # shape as rviz, so "a default boot runs no such node" is a statement the
    # tree itself can be asked about.
    ld = build('swarm_sim.launch.py')
    assert TRUTH_ODOM not in nodes(resolved(ld))


def test_truth_odom_launches_one_node_per_listed_robot():
    ld = build('swarm_sim.launch.py')

    one = nodes(resolved(ld, truth_odom_robots='[0]'))
    assert one.count(TRUTH_ODOM) == 1

    two = nodes(resolved(ld, truth_odom_robots='[0, 3]'))
    assert two.count(TRUTH_ODOM) == 2


def test_the_truth_odom_node_is_staged_behind_a_timer():
    # It needs the bridge (3 s) to be forwarding /model/robot_N/pose before it
    # has an input at all; starting it with the description would give it a
    # subscription to a topic nobody publishes yet.
    ld = build('swarm_sim.launch.py')
    assert TRUTH_ODOM in delayed(resolved(ld, truth_odom_robots='[0]'))


def test_the_truth_odom_node_gets_its_own_robots_spawn_as_the_anchor():
    # The anchor decides where every map built from the run is placed, and a
    # wrong one does not crash: it shifts the map by a constant and the fitter
    # reports a score for a map metres from where the robot was. So the node
    # must receive ITS OWN spawn, not robot_0's and not (0, 0).
    #
    # DOT_POSES[3] is (-0.53, -0.73); with two robots listed the values must
    # arrive per node rather than one repeated pair.
    xs = node_parameters('swarm_sim.launch.py', 'truth_odom_tf', 'spawn_x',
                         truth_odom_robots='[0, 3]')
    ys = node_parameters('swarm_sim.launch.py', 'truth_odom_tf', 'spawn_y',
                         truth_odom_robots='[0, 3]')
    ids = node_parameters('swarm_sim.launch.py', 'truth_odom_tf', 'robot_id',
                          truth_odom_robots='[0, 3]')

    assert ids == [0, 3]
    assert xs == [pytest.approx(0.86), pytest.approx(-0.53)]
    assert ys == [pytest.approx(0.28), pytest.approx(-0.73)]


def test_the_truth_odom_node_runs_on_sim_time():
    # It restamps nothing — it copies the pose's own sim-time stamp — but every
    # consumer of the transform is a use_sim_time node, and a wall-clock node
    # in that chain is the kind of thing that only shows up as dropped scans.
    flags = node_parameters('swarm_sim.launch.py', 'truth_odom_tf',
                            'use_sim_time', truth_odom_robots='[0]')
    assert flags == [True]


@pytest.mark.parametrize('spelling, expected', [
    ('[]', []), ('[0]', [0]), ('[0, 3]', [0, 3]), ('0', [0]),
    ('[3, 0, 3]', [0, 3]), ('', []), ('  [1]  ', [1]),
])
def test_parse_robot_ids_accepts_the_reasonable_spellings(spelling, expected):
    module = load('swarm_sim.launch.py')
    assert module.parse_robot_ids(spelling, 'truth_odom_robots') == expected


@pytest.mark.parametrize('spelling', ['[5]', '[-1]', '[0', 'zero', "['0']"])
def test_parse_robot_ids_refuses_anything_else(spelling):
    # Loudly. Treating an unparseable value as "no robots" would turn a typo
    # into a full 40-minute run recorded in the wrong mode, discoverable only
    # afterwards from the bag.
    module = load('swarm_sim.launch.py')
    with pytest.raises(RuntimeError):
        module.parse_robot_ids(spelling, 'truth_odom_robots')


@needs_burger
def test_the_default_sdf_does_not_mention_the_truth_odom_change():
    # Byte-identity against HEAD is checked outside pytest (it needs git);
    # what this pins is that the default path runs the SAME replacement list it
    # always did, so nothing can drift into it unnoticed.
    module = load('swarm_sim.launch.py')
    for n in range(5):
        with open(module.make_namespaced_burger_sdf(n)) as f:
            sdf = f.read()
        assert '<tf_topic>/tf</tf_topic>' in sdf
        assert 'tf_wheel' not in sdf


@needs_burger
def test_the_flag_moves_diffdrives_transform_off_the_bridged_tf():
    # Exactly one line differs, and it is the one that stops DiffDrive
    # claiming robot_N/odom -> robot_N/base_footprint on the shared /tf.
    # /robot_N/odom must be untouched: the wheel odometry keeps its topic, its
    # rate and its meaning, which is what leaves record_run.sh, check_run_bag.py
    # and the strip keep set alone.
    module = load('swarm_sim.launch.py')
    with open(module.make_namespaced_burger_sdf(0)) as f:
        base = f.read().splitlines()
    with open(module.make_namespaced_burger_sdf(0, truth_odom=True)) as f:
        flagged = f.read().splitlines()

    assert len(base) == len(flagged)
    differing = [(a, b) for a, b in zip(base, flagged) if a != b]
    assert differing == [('      <tf_topic>/tf</tf_topic>',
                          '      <tf_topic>/robot_0/tf_wheel</tf_topic>')]
    assert '<odom_topic>/robot_0/odom</odom_topic>' in '\n'.join(flagged)


# ── online scan matching (explore.launch.py scan_matching) ───────────────────

def test_explore_declares_scan_matching_defaulting_true():
    # true is the value slam_robotN.yaml already carries, so the default
    # reproduces every earlier rung exactly.
    ld = build('explore.launch.py')
    assert declared_defaults(ld)['scan_matching'] == 'true'


def test_scan_matching_reaches_slam_toolbox_as_a_bool():
    got = node_parameters('explore.launch.py', 'async_slam_toolbox_node',
                          'use_scan_matching')
    assert got == [True], f'default resolved to {got!r}'

    off = node_parameters('explore.launch.py', 'async_slam_toolbox_node',
                          'use_scan_matching', scan_matching='false')
    assert off == [False], f'scan_matching:=false resolved to {off!r}'


def test_numeric_scan_matching_spellings_stay_bools():
    # Same cast, same reason as reachability_precheck: launch_ros infers '0'
    # as the INT 0, and slam_toolbox declares use_scan_matching as a bool, so
    # without value_type=bool the node would refuse to start on a spelling
    # that looks entirely reasonable at the command line.
    for raw, expected in (('1', True), ('0', False)):
        got = node_parameters('explore.launch.py', 'async_slam_toolbox_node',
                              'use_scan_matching', scan_matching=raw)
        assert got == [expected], f'scan_matching:={raw} resolved to {got!r}'
