#!/usr/bin/env python3
"""
swarm_sim.launch.py — NSK Swarm Robotics 2D Simulation launch file.

Launches:
  1. Gazebo Harmonic with knowledge_world.sdf
  2. NSK engine lifecycle node, auto-driven configure → activate
     (services /nsk/compress, /nsk/merge, /nsk/similarity_query once active)
  3. ros_gz_bridge for all 5 robots (cmd_vel + odom)
  4. 5 NSKRobotNode instances (after 5 s delay)
  5. ConvergenceMonitorNode (after 6 s delay)
  6. RViz2 with preconfigured layout (after 7 s delay)
"""

import os

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


def generate_launch_description():
    pkg_share = get_package_share_directory('nsk_swarm')
    world_file  = os.path.join(pkg_share, 'worlds', 'knowledge_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz',   'nsk_convergence.rviz')
    params_file = os.path.join(pkg_share, 'config', 'nsk_swarm_params.yaml')

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
        }

    # ── Bridge topic specs ──────────────────────────────────────────────────
    bridge_topics = []
    for n in range(NUM_ROBOTS):
        bridge_topics += [
            f'/robot_{n}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/robot_{n}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
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
