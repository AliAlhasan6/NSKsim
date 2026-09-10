#!/usr/bin/env python3
"""
swarm_sim.launch.py — NSK Swarm Robotics 2D Simulation launch file.

Launches:
  1. Gazebo Harmonic with knowledge_world.sdf, then spawns 5 TurtleBot3
     burger models (topics rewritten into /robot_N namespaces, after 2 s)
  2. NSK engine lifecycle node, auto-driven configure → activate
     (services /nsk/compress, /nsk/merge, /nsk/similarity_query once active)
  3. ros_gz_bridge: per-robot cmd_vel + odom + scan, plus the shared
     gz→ROS /tf (dynamic odom→base_footprint for all robots)
  3b. Per-robot robot_state_publisher: static base_footprint→base_link→
     base_scan chain to /tf_static, from the namespaced URDF (after 4 s)
  4. 5 NSKRobotNode instances (after 5 s delay)
  5. ConvergenceMonitorNode (after 6 s delay)
  6. RViz2 with preconfigured layout (after 7 s delay) — ONLY with rviz:=true

RViz is OFF by default and MUST stay so for any measured run: rendering the
5-robot scene collapses /clock from ~99 Hz to 0.500 Hz and starves the whole
stack. Use rviz:=true for eyeballing only, never during a timed run.
"""

import os
import tempfile

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Substitution
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

# Single source of truth for the swarm size. Must not exceed the number of
# dataset_indices the engine is configured with (5 by default).
NUM_ROBOTS = 5

# colcon entry points run under the system interpreter; torch/PyG live in the
# venv, so its site-packages must be on PYTHONPATH for the Python nodes.
VENV_SITE_PACKAGES = '/home/lawlite/Desktop/NSKsim/venv/lib/python3.12/site-packages'

TURTLEBOT3_BURGER_SDF = ('/opt/ros/jazzy/share/turtlebot3_gazebo/models/'
                         'turtlebot3_burger/model.sdf')

# Stock burger URDF: a xacro template that prefixes every link/joint frame
# with ${namespace}. robot_state_publisher needs it substituted to robot_N/ so
# the static TF frames match the gz-side (robot_N/base_footprint, etc.).
TURTLEBOT3_BURGER_URDF = ('/opt/ros/jazzy/share/turtlebot3_description/urdf/'
                          'turtlebot3_burger.urdf')

# Spawn poses: (x, y, yaw) for robot_0..robot_4, a regular pentagon on a
# 0.9 m ring about the origin.
#
# These replace the ~3.8 m-radius ring inherited from the inline dot models
# that used to live in knowledge_world.sdf. On that ring NONE of the 10 pairs
# started inside the 3.0 m comm_range (nearest pair 4.20 m), and since frontier
# exploration only pushes robots further apart, sharing encounters were
# effectively impossible and EXP-01's reconciliation dynamics could never occur.
# Clustering the spawns puts every pair in range at t=0 and lets exploration
# diverge them from there.
#
# Verified properties of the pentagon below: all 10 pairwise distances are
# < 3.0 m (max 1.72 m, the non-adjacent pairs); min separation 1.06 m, which
# clears the burger's ~0.21 m footprint by a wide margin, so the spawn is
# collision-free. Nearest maze wall (maze_h1 at y=2.9) is ~2.0 m away.
#
# The yaw component is retained for the record but intentionally NOT applied at
# spawn: the burger's DiffDrive odometry is expressed in a frame oriented along
# the spawn yaw, so the pure-translation spawn_x/spawn_y correction in the robot
# nodes and monitor is exact only when every robot spawns with yaw 0. These
# poses are pure-translation offsets and hold all yaw at 0, so that correction
# stays exact.
DOT_POSES = [
    (0.86,  0.28, 0.0),
    (0.00,  0.90, 0.0),
    (-0.86, 0.28, 0.0),
    (-0.53, -0.73, 0.0),
    (0.53, -0.73, 0.0),
]


def make_namespaced_burger_sdf(robot_id: int) -> str:
    """Rewrite the stock burger SDF's plugin/sensor topics and frames into
    the robot_N namespace and return the path of a tempfile holding the
    result, plus append a PosePublisher for ground truth. Geometry, inertia,
    joints, and sensor parameters are untouched; each replaced string occurs
    exactly once in the stock model.

    Called at launch EXECUTE time via LazyRobotAsset, never while the launch
    description is being built — see that class.
    """
    with open(TURTLEBOT3_BURGER_SDF) as f:
        sdf = f.read()
    ns = f'robot_{robot_id}'
    for old, new in [
        ('<topic>cmd_vel</topic>', f'<topic>/{ns}/cmd_vel</topic>'),
        ('<odom_topic>odom</odom_topic>',
         f'<odom_topic>/{ns}/odom</odom_topic>'),
        ('<frame_id>odom</frame_id>', f'<frame_id>{ns}/odom</frame_id>'),
        ('<child_frame_id>base_footprint</child_frame_id>',
         f'<child_frame_id>{ns}/base_footprint</child_frame_id>'),
        ('<topic>scan</topic>', f'<topic>/{ns}/scan</topic>'),
        # SLAM looks up robot_N/base_scan, but the stock sensor advertises the
        # bare frame base_scan; prefix its gz_frame_id to match the TF tree.
        ('<gz_frame_id>base_scan</gz_frame_id>',
         f'<gz_frame_id>{ns}/base_scan</gz_frame_id>'),
        ('<topic>imu</topic>', f'<topic>/{ns}/imu</topic>'),
        ('<topic>joint_states</topic>',
         f'<topic>/{ns}/joint_states</topic>'),
        # B1.8 ground truth: Gazebo's per-world pose topics carry no entity
        # names (verified 2026-09-10 — every child_frame_id came through empty
        # on both /world/knowledge_world/pose/info and .../dynamic_pose/info),
        # so a pose there cannot be attributed to a robot. PosePublisher
        # publishes on a per-model topic instead, where the topic name IS the
        # attribution: the spawn passes -name robot_N, so this plugin lands on
        # /model/robot_N/pose. update_frequency is the recorded ground-truth
        # rate. Unlike every pair above this one APPENDS rather than rewrites;
        # it is anchored on the stock model's single closing tag (line 406).
        ('</model>',
         """    <plugin filename="gz-sim-pose-publisher-system"
            name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
      <update_frequency>20</update_frequency>
    </plugin>
  </model>"""),
    ]:
        sdf = sdf.replace(old, new)
    out = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'{ns}_burger_', suffix='.sdf', delete=False)
    out.write(sdf)
    out.close()
    return out.name


def make_namespaced_burger_urdf(robot_id: int) -> str:
    """Read the stock burger URDF template and substitute its ${namespace}
    token with 'robot_N/' (trailing slash), so every link/joint frame becomes
    robot_N/base_footprint, robot_N/base_scan, etc. — matching the gz-side TF
    prefix. Returns the substituted URDF as a string for RSP's
    robot_description. The stock template is read only, never modified.

    Called at launch EXECUTE time via LazyRobotAsset, never while the launch
    description is being built — see that class.
    """
    with open(TURTLEBOT3_BURGER_URDF) as f:
        urdf = f.read()
    return urdf.replace('${namespace}', f'robot_{robot_id}/')


class LazyRobotAsset(Substitution):
    """One of the make_namespaced_* helpers above, deferred to execute time.

    Both helpers read a stock TurtleBot3 file out of /opt/ros/jazzy/share, and
    both used to be CALLED while generate_launch_description() assembled the
    tree. That made merely BUILDING the description a filesystem operation:
    anywhere turtlebot3_gazebo / turtlebot3_description are absent — a CI
    container, a fresh clone, any machine that isn't this rig — the description
    could not be constructed at all, and the five launch-description tests died
    on FileNotFoundError before they could assert anything.

    Wrapping the call in a Substitution moves the read to the moment the action
    that consumes it runs: `ros2 launch` is unchanged (Node performs its
    arguments and parameters at execute time, when the packages must be present
    anyway for Gazebo to spawn anything), while constructing the description
    touches no disk. The Node objects stay statically in the tree, so the tests
    still see all five spawns and all five robot_state_publishers.

    A missing TurtleBot3 install therefore surfaces when the spawn timer fires
    rather than at description build; it is the same FileNotFoundError naming
    the same path, and a boot without those packages was never going to work.
    """

    def __init__(self, make, robot_id: int):
        self._make = make
        self._robot_id = robot_id

    def describe(self) -> str:
        return f'{self._make.__name__}({self._robot_id})'

    def perform(self, context) -> str:
        return self._make(self._robot_id)


def generate_launch_description():
    pkg_share = get_package_share_directory('nsk_swarm')
    world_file  = os.path.join(pkg_share, 'worlds', 'knowledge_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz',   'nsk_convergence.rviz')
    params_file = os.path.join(pkg_share, 'config', 'nsk_swarm_params.yaml')

    # The burger's DiffDrive plugin publishes odometry relative to each
    # robot's spawn pose (the dots' OdometryPublisher was world-frame), so
    # every consumer of odom needs the spawn offsets to reconstruct
    # world-frame positions for distance/comm-range math.
    spawn_xs = [float(x) for x, _y, _yaw in DOT_POSES[:NUM_ROBOTS]]
    spawn_ys = [float(y) for _x, y, _yaw in DOT_POSES[:NUM_ROBOTS]]

    # ── Common robot parameters ──────────────────────────────────────────────
    def robot_params(robot_id: int) -> dict:
        return {
            'robot_id':            robot_id,
            'num_robots':          NUM_ROBOTS,
            'comm_range':          3.0,
            'share_interval':      8.0,
            'world_size':          20.0,
            'walk_speed':          0.15,
            'walk_turn_max':       0.5,
            # Own spawn pose, plus the full per-robot list: peer odoms are
            # spawn-relative too, so peer positions need peer offsets.
            'spawn_x':             spawn_xs[robot_id],
            'spawn_y':             spawn_ys[robot_id],
            'spawn_xs':            spawn_xs,
            'spawn_ys':            spawn_ys,
            # True when this robot's id is in the nav_robots launch arg (a list
            # literal such as [0]); mutes its wander cmd_vel so Nav2 can drive
            # it. Evaluated at launch time via PythonExpression so CLI overrides
            # (nav_robots:=[0]) are honored, typed bool as seed/csv_path are.
            'nav_controlled': ParameterValue(
                PythonExpression([str(robot_id), ' in ',
                                  LaunchConfiguration('nav_robots')]),
                value_type=bool),
            # Whether the legacy wander driver may drive this robot at all —
            # a separate question from nav_controlled above. Off by default.
            # A bare LaunchConfiguration is yaml-parsed to the *string*
            # 'false', which is truthy on the node side; pin the type as
            # seed/csv_path do.
            'wander_enabled': ParameterValue(
                LaunchConfiguration('wander'), value_type=bool),
            # Lidar obstacle steering: on by default, set false to reproduce
            # runs recorded before it existed. Same yaml-parsing hazard as
            # wander above — pin the type or 'false' arrives as a truthy
            # string.
            'obstacle_enabled': ParameterValue(
                LaunchConfiguration('obstacle'), value_type=bool),
            # Same cast, same reason as escape_distance in explore.launch.py:
            # robot_node declares this one with a float default, and
            # 'obstacle_stop_m:=1' would otherwise infer as the INT 1 and be
            # refused at startup on a perfectly reasonable spelling.
            'obstacle_stop_m': ParameterValue(
                LaunchConfiguration('obstacle_stop_m'), value_type=float),
        }

    # ── Bridge topic specs ──────────────────────────────────────────────────
    bridge_topics = []
    for n in range(NUM_ROBOTS):
        bridge_topics += [
            f'/robot_{n}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/robot_{n}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Lidar is gz→ROS only: the burger's sensor publishes on the gz
            # side and ROS consumes it (Phase B). '[' is the input-only
            # delimiter, unlike the bidirectional '@...@' used above.
            f'/robot_{n}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # B1.8 ground truth, gz→ROS only. The gz name is kept verbatim on
            # the ROS side: parameter_bridge's '@' spec names ONE topic for
            # both ends, and /model/robot_N/pose is already unambiguous — it
            # is the attribution (see the PosePublisher pair in
            # make_namespaced_burger_sdf). use_pose_vector_msg is false there,
            # so the wire type is a single gz.msgs.Pose, not Pose_V; taking it
            # as PoseStamped keeps the sim timestamp that a bare Pose drops.
            f'/model/robot_{n}/pose'
            '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ]

    # All 5 DiffDrive plugins publish their dynamic odom→base_footprint
    # transform to one shared gz topic /tf (gz.msgs.Pose_V), with frames
    # already robot_N/-prefixed (see the SDF frame rewrites above). Bridge it
    # once, gz→ROS only ('['), into ROS /tf — no namespace remap needed since
    # both sides are the global /tf. RSP supplies the static links on
    # /tf_static; the two trees meet at each robot's base_footprint.
    bridge_topics += [
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]

    # ── NSK engine lifecycle node + auto-driven transitions ─────────────────
    # unconfigured → configure (emitted below; launch's lifecycle event
    # manager waits for the node's change_state service, so emitting at
    # launch time is race-free) → inactive → activate (via the
    # OnStateTransition handler) → active, at which point the engine's
    # service servers exist.
    engine_node = LifecycleNode(
        package='nsk_swarm',
        executable='nsk_engine',
        name='nsk_engine',
        namespace='',
        parameters=[{
            'num_robots': NUM_ROBOTS,
            # LaunchConfiguration resolves to a string; the engine declares
            # seed as int, so the type must be forced here.
            'seed': ParameterValue(LaunchConfiguration('seed'),
                                   value_type=int),
        }],
        output='screen',
    )

    engine_activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=engine_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(engine_node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE),
                )),
            ],
        )
    )

    engine_configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(engine_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    # ── RViz2 (opt-in; OFF by default) ──────────────────────────────────────
    # RViz used to launch here UNCONDITIONALLY on a 7 s timer, and it was
    # quietly wrecking every measurement this rig has ever taken: rendering the
    # 5-robot scene collapsed /clock from ~99 Hz to 0.500 Hz (200x slower than
    # real time), which starved everything downstream — all five robot nodes
    # timed out on /nsk/similarity_query and /nsk/compress, and bt_navigator
    # could not answer its own get_state service. Every RTF and coverage number
    # recorded before this gate was taken with that load on the machine.
    #
    # Gated the same way explore.launch.py gates its own RViz: an OpaqueFunction
    # returning NOTHING when the flag is false, so the node is absent from the
    # launch tree rather than merely condition-suppressed. (No string
    # substitution into the config is needed here — nsk_convergence.rviz is
    # namespace-agnostic — but keeping the two launch files structurally
    # identical is worth more than saving the indirection, and it is what makes
    # "no rviz2 node with default args" a testable statement.)
    def _rviz_actions(context):
        if LaunchConfiguration('rviz').perform(context).lower() not in ('true', '1'):
            return []
        # 7 s, unchanged from the original — RViz starts last, after the robot
        # nodes and the monitor, so it subscribes to topics that already exist.
        return [TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                ),
            ],
        )]

    rviz_group = OpaqueFunction(function=_rviz_actions)

    # Gazebo, gated the same way, and for a second reason: a substitution
    # cannot express "this argv element is absent". A PythonExpression that
    # evaluates to '' still passes an empty argument through to gz sim, which
    # would stop the default GUI path being the argv every earlier run used.
    # Building the list here keeps that path byte-identical to the pre-headless
    # one, so 'headless defaults off and reproduces exactly' stays true.
    def _gz_sim_actions(context):
        if LaunchConfiguration('headless').perform(context).lower() in ('true', '1'):
            return [ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world_file],
                                   output='screen')]
        return [ExecuteProcess(cmd=['gz', 'sim', '-r', world_file],
                               output='screen')]

    return LaunchDescription([

        # ── 0. Venv on PYTHONPATH for the Python nodes ───────────────────────
        DeclareLaunchArgument(
            'venv_site_packages',
            default_value=VENV_SITE_PACKAGES,
            description='site-packages dir prepended to PYTHONPATH so the '
                        'Python nodes can import torch/PyG',
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            [LaunchConfiguration('venv_site_packages'),
             os.pathsep + os.environ.get('PYTHONPATH', '')],
        ),
        # The spawned burger SDFs reference model://turtlebot3_common mesh
        # URIs, which Gazebo can only resolve with this directory on its
        # resource path (physics works without it, but the GUI shows mesh
        # errors).
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
            + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='-1',
            description='Engine RNG seed for reproducible runs '
                        '(-1 = unseeded)',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='',
            description='Per-cycle CSV export path for the convergence '
                        "monitor ('' = disabled)",
        ),
        DeclareLaunchArgument(
            'nav_robots',
            default_value='[]',
            description='List of robot IDs whose wander driver is muted so '
                        'Nav2 can drive them, e.g. [0]',
        ),
        DeclareLaunchArgument(
            'obstacle',
            default_value='true',
            description='Steer the wander driver away from whatever its lidar '
                        'sees in the forward arc. On by default: without it '
                        'the only wall check is a pose test against the outer '
                        'boundary, which cannot see the interior maze walls, '
                        'and robots grind along them indefinitely. Set false '
                        'to reproduce runs recorded before it existed.',
        ),
        DeclareLaunchArgument(
            'obstacle_stop_m',
            default_value='0.6',
            description='How close a forward lidar return has to be, in '
                        'metres, before the wander driver steers away from '
                        'it. Tunable because 0.6 is not a calibrated value: '
                        'at walk_speed 0.15 m/s and walk_turn_max 0.5 rad/s a '
                        'robot needs about 3 s to turn 90 deg and covers '
                        '~0.45 m doing it, so 0.6 m leaves only ~0.15 m of '
                        'margin against a perpendicular wall and NONE at an '
                        'oblique angle. Observed 2026-08-07: at 0.6 all five '
                        'robots still made wall contact, though they '
                        'travelled between walls rather than staying pinned. '
                        'Like escape_distance in explore.launch.py, an '
                        'observation to be re-measured, not a calibrated '
                        'value. The default is robot_node\'s own, so a '
                        'command that does not set it reproduces the previous '
                        'run exactly.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 with the convergence layout. MUST '
                        'default false: rendering the 5-robot scene CORRUPTS '
                        'the RTF measurements this rig exists to take — '
                        'measured at /clock 99 Hz -> 0.500 Hz (200x), with the '
                        'robot nodes timing out on /nsk/similarity_query and '
                        '/nsk/compress and bt_navigator unable to answer its '
                        'own get_state. Enable only for eyeballing, never '
                        'during a timed run.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only, no GUI. Off by default so '
                        'earlier runs reproduce exactly. The GUI costs roughly '
                        '25% of one core and about 7 points of load; ban it '
                        'for timed and recorded runs, as rviz already is.',
        ),
        DeclareLaunchArgument(
            'wander',
            default_value='false',
            description='Enable the legacy dot-era wander driver on every '
                        'robot node (opt-in; off by default, so robots hold '
                        'their spawn poses unless another controller drives '
                        'them)',
        ),

        # ── 1. Gazebo Harmonic ───────────────────────────────────────────────
        OpaqueFunction(function=_gz_sim_actions),

        # ── 1b. Spawn TurtleBot3 burgers at the recorded dot x/y, yaw 0 ─────
        # (2 s; zero yaw keeps the odom frame axis-aligned with the world —
        # see the DOT_POSES comment.)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name=f'spawn_robot_{n}',
                    arguments=[
                        '-world', 'knowledge_world',
                        '-file', LazyRobotAsset(make_namespaced_burger_sdf, n),
                        '-name', f'robot_{n}',
                        '-x', str(x), '-y', str(y), '-z', '0.01',
                    ],
                    output='screen',
                )
                for n, (x, y, _yaw) in enumerate(DOT_POSES[:NUM_ROBOTS])
            ],
        ),

        # ── 2. NSK engine (lifecycle: configure → inactive → activate) ──────
        engine_node,
        engine_activate_on_inactive,
        engine_configure,

        # ── 2b. Dedicated /clock bridge (gz→ROS) ────────────────────────────
        # Gazebo's sim clock must reach ROS as ABSOLUTE /clock or every
        # use_sim_time node stays frozen at zero. Kept as its own bridge,
        # SEPARATE from the main bridge below and OUTSIDE any namespace push so
        # the topic resolves to /clock (not /robot_N/clock). use_sim_time is
        # forced False here: a bridge that waited on the very clock it publishes
        # would deadlock. Started immediately (no timer) so the clock is live
        # before the sim-time nodes (RSP at 4 s, robots at 5 s) come up.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),

        # ── 3. ros_gz_bridge ────────────────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge']
                         + bridge_topics,
                    output='screen',
                ),
            ],
        ),

        # ── 3b. Per-robot robot_state_publisher (after 4 s) ─────────────────
        # Publishes the burger's static frame chain (base_footprint→base_link→
        # base_scan, plus wheel/imu/caster links) from the namespaced URDF.
        # 'tf'/'tf_static' are remapped to the global topics so all robots
        # share one TF tree, kept distinct by their robot_N/ frame prefix; this
        # meets the bridged dynamic odom→base_footprint at base_footprint.
        # (SLAM will supply map→odom next.)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace=f'/robot_{n}',
                    parameters=[{
                        'robot_description': ParameterValue(
                            LazyRobotAsset(make_namespaced_burger_urdf, n),
                            value_type=str),
                        'use_sim_time': True,
                    }],
                    remappings=[
                        (f'/robot_{n}/tf', '/tf'),
                        (f'/robot_{n}/tf_static', '/tf_static'),
                    ],
                    output='screen',
                )
                for n in range(NUM_ROBOTS)
            ],
        ),

        # ── 4. Robot nodes (after 5 s) ───────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nsk_swarm',
                    executable='robot_node',
                    name=f'robot_{n}',
                    parameters=[robot_params(n)],
                    output='screen',
                )
                for n in range(NUM_ROBOTS)
            ],
        ),

        # ── 5. Convergence monitor (after 6 s) ──────────────────────────────
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nsk_swarm',
                    executable='convergence_monitor',
                    name='convergence_monitor',
                    parameters=[{
                        'num_robots':            NUM_ROBOTS,
                        'monitor_interval':      10.0,
                        'comm_range':            3.0,
                        'convergence_threshold': 0.25,
                        'spawn_xs':              spawn_xs,
                        'spawn_ys':              spawn_ys,
                        # A bare LaunchConfiguration is yaml-parsed, so ''
                        # would not survive as a string; pin the type as
                        # the seed parameter does.
                        'csv_path': ParameterValue(
                            LaunchConfiguration('csv_path'), value_type=str),
                    }],
                    output='screen',
                ),
            ],
        ),

        # ── 6. RViz2 (opt-in; OFF by default, after 7 s) ─────────────────────
        rviz_group,
    ])
