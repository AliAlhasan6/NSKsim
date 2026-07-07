#!/usr/bin/env python3
"""
swarm_sim.launch.py — NSK Swarm Robotics 2D Simulation launch file.

Launches:
  1. Gazebo Harmonic with knowledge_world.sdf
  2. ros_gz_bridge for all 5 robots (cmd_vel + odom)
  3. 5 NSKRobotNode instances (after 5 s delay)
  4. ConvergenceMonitorNode (after 6 s delay)
  5. RViz2 with preconfigured layout (after 7 s delay)

NOTE: Start NSK engine SEPARATELY in nsk_env before launching:
    source ~/Desktop/NSK/nsk_env/bin/activate
    CUDA_VISIBLE_DEVICES="" python -m nsk_engine.engine_server
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nsk_swarm')
    world_file  = os.path.join(pkg_share, 'worlds', 'knowledge_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz',   'nsk_convergence.rviz')
    params_file = os.path.join(pkg_share, 'config', 'nsk_swarm_params.yaml')

    # ── Common robot parameters ──────────────────────────────────────────────
    def robot_params(robot_id: int) -> dict:
        return {
            'robot_id':            robot_id,
            'num_robots':          8,
            'comm_range':          3.0,
            'share_interval':      8.0,
            'zmq_endpoint':        'ipc:///tmp/nsk_engine_0',
            'world_size':          20.0,
            'walk_speed':          0.15,
            'walk_turn_max':       0.5,
            'zmq_timeout_ms':      2000,
        }

    # ── Bridge topic specs ──────────────────────────────────────────────────
    bridge_topics = []
    for n in range(5):
        bridge_topics += [
            f'/robot_{n}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/robot_{n}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ]

    return LaunchDescription([

        # ── 1. Gazebo Harmonic ───────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # ── 2. ros_gz_bridge ────────────────────────────────────────────────
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

        # ── 3. Engine reminder ───────────────────────────────────────────────
        LogInfo(
            msg=(
                '\n\n'
                '╔══════════════════════════════════════════════════════╗\n'
                '║  ACTION REQUIRED: Start NSK engine in Terminal 2     ║\n'
                '║                                                      ║\n'
                '║  cd ~/Desktop/NSK                                    ║\n'
                '║  source nsk_env/bin/activate                         ║\n'
                '║  CUDA_VISIBLE_DEVICES="" \\                            ║\n'
                '║    python -m nsk_engine.engine_server                ║\n'
                '╚══════════════════════════════════════════════════════╝\n'
            ),
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
                for n in range(5)
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
                        'num_robots':            8,
                        'monitor_interval':      10.0,
                        'comm_range':            3.0,
                        'convergence_threshold': 0.25,
                        'zmq_endpoint':          'ipc:///tmp/nsk_engine_0',
                        'zmq_timeout_ms':        2000,
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
