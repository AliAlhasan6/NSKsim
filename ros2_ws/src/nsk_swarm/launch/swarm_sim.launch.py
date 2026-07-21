#!/usr/bin/env python3
"""
swarm_sim.launch.py — NSK Swarm Robotics 2D Simulation launch file.

Launches:
  1. Gazebo Harmonic with knowledge_world.sdf, then spawns 5 TurtleBot3
     burger models (topics rewritten into /robot_N namespaces, after 2 s)
  2. NSK engine lifecycle node, auto-driven configure → activate
     (services /nsk/compress, /nsk/merge, /nsk/similarity_query once active)
  3. ros_gz_bridge for all 5 robots (cmd_vel + odom + scan)
  4. 5 NSKRobotNode instances (after 5 s delay)
  5. ConvergenceMonitorNode (after 6 s delay)
  6. RViz2 with preconfigured layout (after 7 s delay)
"""

import os
import tempfile

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
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

# Spawn poses carried over from the inline dot models that used to live in
# knowledge_world.sdf: (x, y, yaw) for robot_0..robot_4. The yaw component
# is retained for the record but intentionally NOT applied at spawn: the
# burger's DiffDrive odometry is expressed in a frame oriented along the
# spawn yaw, so the pure-translation spawn_x/spawn_y correction in the
# robot nodes and monitor is exact only when every robot spawns with yaw 0.
DOT_POSES = [
    (3.0, 0.0, 0.0),
    (1.2, 3.8, 4.71238898),
    (-3.2, 2.4, 0.0),
    (-3.2, -2.4, 0.78539816),
    (1.2, -3.8, 1.57079633),
]


def make_namespaced_burger_sdf(robot_id: int) -> str:
    """Rewrite the stock burger SDF's plugin/sensor topics and frames into
    the robot_N namespace and return the path of a tempfile holding the
    result. Geometry, inertia, joints, and sensor parameters are untouched;
    each replaced string occurs exactly once in the stock model.
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
        ('<topic>imu</topic>', f'<topic>/{ns}/imu</topic>'),
        ('<topic>joint_states</topic>',
         f'<topic>/{ns}/joint_states</topic>'),
    ]:
        sdf = sdf.replace(old, new)
    out = tempfile.NamedTemporaryFile(
        mode='w', prefix=f'{ns}_burger_', suffix='.sdf', delete=False)
    out.write(sdf)
    out.close()
    return out.name


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

        # ── 1. Gazebo Harmonic ───────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

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
                        '-file', make_namespaced_burger_sdf(n),
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

        # ── 6. RViz2 (after 7 s) ────────────────────────────────────────────
        TimerAction(
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
        ),
    ])
