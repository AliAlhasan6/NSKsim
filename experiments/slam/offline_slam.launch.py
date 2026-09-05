#!/usr/bin/env python3
"""One slam_toolbox instance, unnamespaced, for OFFLINE mapping of a single
robot from ``experiments/bags/phaseB_run1``.

This is deliberately NOT explore.launch.py. That file brings up SLAM + Nav2 +
the frontier explorer for five robots against a live Gazebo. Here there is no
simulator, no Nav2, no explorer, and exactly one robot per invocation -- the bag
is replayed once per robot by run_offline_maps.sh.

Two mechanisms are copied from explore.launch.py because they were established
the hard way there:

  1. slam_toolbox is a LIFECYCLE node. Launched as a plain Node it comes up
     'unconfigured' and stays there forever -- it never publishes map->odom and
     never publishes an OccupancyGrid, so save_map.py would time out on a topic
     that exists but never fires. We emit CONFIGURE, then ACTIVATE on reaching
     'inactive'.
  2. use_lifecycle_manager:false tells the node not to wait on a bond with an
     external manager, since these launch-emitted transitions drive it instead.

What is deliberately different: no namespace and no RewrittenYaml. The node's
fully-qualified name is plain /slam_toolbox, so the params file uses a flat
'slam_toolbox:' key (see offline_mapping.yaml.template). Frames stay
robot_<id>/... because they are parameter values, not namespaces.

Usage (run_offline_maps.sh does this for you):

    ros2 launch experiments/slam/offline_slam.launch.py \
        params_file:=/tmp/offline_mapping_robot_3.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        description=(
            'Absolute path to a slam_toolbox params file with a FLAT '
            '"slam_toolbox:" key and the robot frames already substituted. '
            'run_offline_maps.sh generates one per robot from '
            'experiments/slam/offline_mapping.yaml.template.'
        ),
    )

    # async_slam_toolbox_node, matching explore.launch.py. The async node does
    # not block the executor on scan processing, which matters when the bag
    # replays faster than the scans were recorded.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[params_file],
        output='screen',
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    return LaunchDescription([
        declare_params_file,
        slam_node,
        slam_activate,   # registered before the CONFIGURE it reacts to
        slam_configure,
    ])
