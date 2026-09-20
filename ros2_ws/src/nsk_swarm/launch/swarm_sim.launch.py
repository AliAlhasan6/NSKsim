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
  4b. truth_odom_tf for each robot in truth_odom_robots (after 4 s) — ONLY
     with truth_odom_robots:=[K], which is empty by default
  5. ConvergenceMonitorNode (after 6 s delay)
  6. RViz2 with preconfigured layout (after 7 s delay) — ONLY with rviz:=true

RViz is OFF by default and MUST stay so for any measured run: rendering the
5-robot scene collapses /clock from ~99 Hz to 0.500 Hz and starves the whole
stack. Use rviz:=true for eyeballing only, never during a timed run.
"""

import ast
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

# WHICH SENSOR THIS SIM MODELS -- the canonical statement, quoted from here by
# slam_robot<id>.yaml, explore.launch.py, README.md and TECHNICAL.md:
#
#   The simulated lidar is a ROBOTIS LDS-02 -- detection distance 160 to
#   8000 mm per the ROBOTIS e-manual, the sensor that replaced the LDS-01 on
#   the TurtleBot3 Burger in 2022 -- not the 120 to 3500 mm LDS-01 that the
#   stock turtlebot3_gazebo model still ships; make_namespaced_burger_sdf
#   rewrites both bounds at spawn.
#
# BURGER_STOCK_RANGE_MIN/MAX are what the stock model ships (model.sdf:152 and
# :153, the only <min> and <max> in the file); BURGER_RANGE_MIN/MAX are what
# every spawned robot actually runs. Everything else about the sensor -- 360
# samples, 1 degree increment, 5 Hz, gaussian noise at 0.01 m -- is common to
# both and untouched.
#
# WHY, at length, because a fidelity claim is not a preference. The LDS-01 is a
# real 0.12-3.5 m sensor
# and it cannot see this world: b2maps_e0t's true trajectory bounds an
# 11.37 x 5.75 m box inside a 20 x 20 m room, and its closest approach to each
# outer wall is east 4.178, west 4.355, north 6.707, south 7.440 m. Below
# 7.44 m at least one boundary wall never enters a single scan, so the map's
# extent is set by where the robot went rather than by where the room ends --
# e0t mapped 63 m2 of a 391 m2 floor in a 12.05 x 8.00 m box. 8.0 m is 7.44
# rounded up, and lands exactly on a real sensor rather than an invented one,
# which is why the platform can still be named honestly: a TurtleBot3 Burger
# WITH AN LDS-02, the configuration ROBOTIS has shipped since 2022.
#
# This is NOT a substitute for free_space_relay and does not overlap with it.
# Karto sizes its occupancy grid from FILTERED readings only (Karto.h:5644,
# InRange(reading, minRange, rangeThreshold)), so a relay-filled beam can never
# enlarge the grid -- measured: b2maps_e0t_kp and _kpfree have identical
# 12.00 x 8.00 m boxes. Range sets the box; the relay fills it. Raising this
# alone still leaves 20.5% of beams clearing nothing at 8 m.
#
# The MINIMUM moves with it, for the same reason: the LDS-02's detection
# distance is 160-8000 mm (ROBOTIS e-manual), so keeping the LDS-01's 0.12 m
# floor beside an 8 m ceiling would model neither sensor. Measured cost on
# b2maps_e0t, which is none: 94 of 58542840 beams across all five robots fall
# in [0.12, 0.16) -- 0.00016%, and 49 of them are robot_0's. Replaying every
# robot_0 scan through _forward_arc_bins at both floors, 5 scans of 32524 hold
# such a beam, 2 change a forward bin's minimum, and NO bin is emptied and no
# blocked/open verdict flips. The robot's own body cannot enter the band
# either: at the sensor's z=0.171 the only same-model geometry is
# lidar_sensor_collision, whose surface is 0.039-0.063 m away, already below
# both floors.
#
# Anything that reports a number derived from the lidar ceiling must read
# BURGER_RANGE_MAX, not the stock file -- see run_health.py:lidar_max_range.
BURGER_STOCK_RANGE_MAX = 3.5
BURGER_RANGE_MAX = 8.0
BURGER_STOCK_RANGE_MIN = 0.12
BURGER_RANGE_MIN = 0.16

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


def make_namespaced_burger_sdf(robot_id: int, truth_odom: bool = False) -> str:
    """Rewrite the stock burger SDF's plugin/sensor topics and frames into
    the robot_N namespace and return the path of a tempfile holding the
    result, plus append a PosePublisher for ground truth. Geometry, inertia
    and joints are untouched; each replaced string occurs exactly once in the
    stock model.

    THE SENSOR'S RANGE BOUNDS ARE NOT UNTOUCHED: <range><min> and <range><max>
    move from BURGER_STOCK_RANGE_MIN/MAX to BURGER_RANGE_MIN/MAX, so the
    spawned robots carry an LDS-02 rather than the burger's stock LDS-01. The
    reasoning is on those constants. Everything else about the sensor -- 360
    samples, 1 degree increment, 5 Hz, gaussian noise at 0.01 m -- is the stock
    model's, and is shared by both sensors.

    truth_odom=True additionally moves this robot's DiffDrive transform OFF the
    bridged /tf, so truth_odom_tf can own the frame pair instead — see the
    truth_odom_robots launch argument. It changes nothing else: DiffDrive keeps
    publishing /robot_N/odom at the same rate, on the same topic, with the same
    meaning, so record_run.sh, check_run_bag.py, the strip script and every
    consumer of wheel odometry are untouched. False leaves the replacement list
    exactly as it was before the flag existed, so a default boot writes a
    byte-identical SDF.

    Called at launch EXECUTE time via LazyRobotAsset, never while the launch
    description is being built — see that class.
    """
    with open(TURTLEBOT3_BURGER_SDF) as f:
        sdf = f.read()
    ns = f'robot_{robot_id}'
    truth_odom_pairs = [
        # The stock model publishes DiffDrive's odom->base_footprint onto the
        # shared gz /tf (model.sdf line 396), which swarm_sim bridges into ROS
        # /tf for all five robots at once. tf2 allows ONE parent per frame, so
        # the truth transform cannot simply be added alongside it: both would
        # claim robot_N/odom -> robot_N/base_footprint and every consumer would
        # read a mixture. The bridge spec names /tf for the whole swarm and
        # cannot drop one robot's transforms, so the removal has to happen on
        # the gz side, here.
        #
        # /robot_N/tf_wheel is a valid gz topic that NOTHING bridges, so these
        # transforms stay inside Gazebo. Deliberately not the empty string:
        # DiffDrive runs an invalid tf_topic through gz::sim::validTopic() and
        # silently falls back to /model/<name>/tf, which would work by accident
        # rather than by contract.
        ('<tf_topic>/tf</tf_topic>', f'<tf_topic>/{ns}/tf_wheel</tf_topic>'),
    ] if truth_odom else []
    for old, new in truth_odom_pairs + [
        # The lidar's two range bounds. Applies to every robot regardless of
        # truth_odom: the sensor is the same sensor on all five. See
        # BURGER_RANGE_MAX.
        #
        # The two are formatted DIFFERENTLY because the stock file writes them
        # differently -- <min>0.120000</min> to six places, <max>3.5</max> to
        # one. These are exact-string replacements against a file this repo
        # does not own, so the formatting is part of the pattern, not a style
        # choice; str(0.12) would render '0.12' and match nothing. Each
        # literal occurs exactly once in the stock model, and
        # test_launch_descriptions.py asserts both counts so a stock-model
        # update that reformats either one fails loudly instead of silently
        # leaving the sensor where it was.
        (f'<min>{BURGER_STOCK_RANGE_MIN:.6f}</min>',
         f'<min>{BURGER_RANGE_MIN:.6f}</min>'),
        (f'<max>{BURGER_STOCK_RANGE_MAX}</max>',
         f'<max>{BURGER_RANGE_MAX}</max>'),
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


def parse_robot_ids(text: str, arg_name: str) -> list:
    """A list-literal launch argument such as '[0]' -> [0].

    Strict on purpose. The alternative — treating anything unparseable as "no
    robots" — turns a typo into a full run recorded in the wrong mode, which is
    only discoverable afterwards from the bag. A bare integer is accepted as
    well as a list, because 'truth_odom_robots:=0' has exactly one reading and
    refusing it would cost a boot to learn a punctuation rule.
    """
    text = (text or '').strip()
    if not text:
        return []
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(
            f'{arg_name}:={text!r} is not a list literal ({exc}). '
            f'Write it as {arg_name}:=[0] or {arg_name}:=[].') from exc

    ids = [value] if isinstance(value, int) else list(value)
    bad = [i for i in ids if not isinstance(i, int) or not 0 <= i < NUM_ROBOTS]
    if bad:
        raise RuntimeError(
            f'{arg_name}:={text!r} names {bad}, which is not a robot id in '
            f'0..{NUM_ROBOTS - 1}.')
    return sorted(set(ids))


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

    truth_odom_arg names a launch argument holding the list of robots whose
    odometry comes from ground truth; when given, this resolves it at the same
    execute time and passes truth_odom=True/False to the maker. Reading it here
    rather than at build time is what keeps the flag consistent with the rest
    of the class: the SDF is written once, when the spawn action runs, from
    whatever the command line actually said.
    """

    def __init__(self, make, robot_id: int, truth_odom_arg: str = None):
        self._make = make
        self._robot_id = robot_id
        self._truth_odom_arg = truth_odom_arg

    def describe(self) -> str:
        if self._truth_odom_arg is None:
            return f'{self._make.__name__}({self._robot_id})'
        return (f'{self._make.__name__}({self._robot_id}, '
                f'truth_odom=<{self._truth_odom_arg}>)')

    def perform(self, context) -> str:
        if self._truth_odom_arg is None:
            return self._make(self._robot_id)
        ids = parse_robot_ids(
            LaunchConfiguration(self._truth_odom_arg).perform(context),
            self._truth_odom_arg)
        return self._make(self._robot_id, truth_odom=self._robot_id in ids)


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
            # Wheel joint state, gz→ROS only. JointStatePublisher in the
            # burger SDF publishes gz.msgs.Model (a Model envelope carrying
            # per-joint axis1 position and velocity), already namespaced to
            # /robot_N/joint_states by make_namespaced_burger_sdf. The bridge
            # registers Model→JointState; verified live 2026-09-11, the bridge
            # logging "Creating GZ->ROS Bridge: [/robot_0/joint_states
            # (gz.msgs.Model) -> ... (sensor_msgs/msg/JointState)]". Needed to
            # separate wheel rotation from body rotation in the B1.8 analysis.
            f'/robot_{n}/joint_states'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
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

    # Ground-truth odometry for named robots, gated the same way and for the
    # same reason: with the list empty this returns NOTHING, so a default boot
    # has no such node in the tree at all rather than a suppressed one.
    #
    # 4 s, alongside robot_state_publisher. The bridge comes up at 3 s and must
    # already be forwarding /model/robot_N/pose for this node to have an input;
    # slam_toolbox and Nav2 arrive later still, from explore.launch.py, so the
    # transform is live before anything looks it up.
    #
    # A distinct node name per robot. Unlike frontier_explorer (see the note in
    # explore.launch.py) this process spins exactly ONE node, so the
    # process-wide `-r __node:=` that name= becomes renames the one node it is
    # meant to, and two truth-driven robots cannot collide.
    def _truth_odom_actions(context):
        ids = parse_robot_ids(
            LaunchConfiguration('truth_odom_robots').perform(context),
            'truth_odom_robots')
        if not ids:
            return []
        return [TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='nsk_swarm',
                    executable='truth_odom_tf',
                    name=f'truth_odom_tf_robot_{n}',
                    parameters=[{
                        'robot_id': n,
                        # The anchor. Same spawn_xs/spawn_ys the robot nodes
                        # and the monitor read, so there is one table and one
                        # place to get it wrong.
                        'spawn_x': spawn_xs[n],
                        'spawn_y': spawn_ys[n],
                        'use_sim_time': True,
                    }],
                    output='screen',
                )
                for n in ids
            ],
        )]

    truth_odom_group = OpaqueFunction(function=_truth_odom_actions)

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
            'truth_odom_robots',
            default_value='[]',
            description='List of robot IDs whose odom->base_footprint comes '
                        'from Gazebo ground truth instead of wheel odometry, '
                        'e.g. [0]. For each one, DiffDrive\'s transform is '
                        'moved off the bridged /tf and a truth_odom_tf node '
                        'publishes inv(spawn).truth in its place; the scans '
                        'are untouched, and /robot_N/odom still carries the '
                        'same wheel odometry on the same topic. Exists '
                        'because b2maps_e0\'s explorer steered by a map its '
                        'own odometry had destroyed — 138 deg median heading '
                        'error, walls in fans of rotated copies, exploration '
                        'declared complete having never seen the outer '
                        'boundary. Orthogonal to nav_robots, which only mutes '
                        'the wander driver; a truth-driven explorer needs '
                        'both. MUST default []: every run recorded before '
                        'this existed has to reproduce exactly, and with the '
                        'list empty the generated SDFs are byte-identical and '
                        'no extra node is launched.',
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
                        '-file', LazyRobotAsset(make_namespaced_burger_sdf, n,
                                                'truth_odom_robots'),
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

        # ── 5b. Ground-truth odometry (opt-in; OFF by default, after 4 s) ───
        truth_odom_group,

        # ── 6. RViz2 (opt-in; OFF by default, after 7 s) ─────────────────────
        rviz_group,
    ])
