#!/usr/bin/env python3
"""explore.launch.py — live SLAM + Nav2 + frontier explorer for ONE robot.

Brings up, all under the /robot_<id> namespace (default robot_0):
  1. slam_toolbox online_async node        -> live map (robot_<id>/map -> robot_<id>/odom on /tf)
  2. Nav2 stack (navigation_launch.py)     -> plans and drives /robot_<id>/cmd_vel
  3. frontier_explorer node                -> sends NavigateToPose goals at map frontiers

This proves the explore -> plan -> drive -> map loop for one robot and lets us
measure RTF before scaling to 5 robots.

PREREQUISITE: swarm_sim.launch.py must ALREADY be running with
nav_robots:=[<id>] so the target robot's wander driver is muted and its
/robot_<id>/cmd_vel is free for Nav2 to command. This launch adds only the SLAM,
Nav2 and explorer nodes on top of that running sim.

TF NOTE: tf2 uses the absolute /tf topic and the whole swarm shares one global
/tf tree keyed by robot_N/-prefixed frames. navigation_launch.py hardcodes a
('/tf','tf') remap that would isolate Nav2 onto an empty /robot_<id>/tf; the
GroupAction below prepends SetRemap('/tf','/tf') (and /tf_static) which — being a
global remap applied before the node's own rules, first-match-wins — keeps Nav2
listening on the global /tf. slam_toolbox needs no such remap (no isolating rule).

Parametrized by robot_id: the namespace and the slam/nav2 param filenames all
derive from it, so robots 1-4 reuse this launch by changing one arg (and
providing sibling slam_robotN.yaml / nav2_robotN.yaml).

Usage:
  ros2 launch nsk_swarm explore.launch.py                 # robot_0
  ros2 launch nsk_swarm explore.launch.py robot_id:=3     # robot_3 (needs *_robot3.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (LifecycleNode, Node, PushROSNamespace,
                                SetRemap)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml

# The nav/slam param files live in the repo's experiments/ tree, not inside the
# installed package share. swarm_sim.launch.py already hardcodes an absolute repo
# path (VENV_SITE_PACKAGES); follow that precedent here.
EXP_NAV = '/home/lawlite/Desktop/NSKsim/experiments/nav'


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    navigation_launch = os.path.join(
        nav2_bringup_share, 'launch', 'navigation_launch.py')

    robot_id = LaunchConfiguration('robot_id')
    # robot_<id> namespace as a substitution (concatenation of str + LaunchConfig).
    namespace = ['robot_', robot_id]

    slam_params = LaunchConfiguration('slam_params_file')
    nav2_params = LaunchConfiguration('nav2_params_file')

    declare_robot_id = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description='Robot to explore with; selects namespace robot_<id> and '
                    'the slam/nav2 param files. Requires swarm_sim.launch.py '
                    'running with nav_robots:=[<id>].')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=[os.path.join(EXP_NAV, 'slam_robot'), robot_id, '.yaml'],
        description='slam_toolbox online_async params (frames namespaced to '
                    'robot_<id>).')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=[os.path.join(EXP_NAV, 'nav2_robot'), robot_id, '.yaml'],
        description='Nav2 params (frames namespaced to robot_<id>, '
                    'enable_stamped_cmd_vel:false).')

    # ── 1. SLAM (online async) ──────────────────────────────────────────────
    # No /tf remap: launched under /robot_<id> it still broadcasts map->odom onto
    # the global /tf (tf2 uses the absolute /tf topic).
    #
    # PARAMS MUST BE RE-KEYED. slam_robot<id>.yaml keys its params under
    # 'slam_toolbox:', but this node's fully-qualified name is
    # /robot_<id>/slam_toolbox. A plain --params-file matches by FQN, so that
    # bare key NEVER matches and NONE of the YAML loads — scan_topic, map_frame,
    # odom_frame and base_frame all silently fall back to slam_toolbox's own
    # defaults (/scan, map, odom, base_footprint). The node then listens on the
    # nonexistent /scan (real topic is /robot_<id>/scan) so it registers zero
    # scans and never publishes map->odom, AND its frames are un-prefixed so
    # robot_<id>/map never exists. RewrittenYaml wraps the file under the
    # robot_<id> namespace key (robot_<id>: slam_toolbox: ros__parameters:) so it
    # matches the node — the same mechanism nav2's navigation_launch.py uses for
    # its own servers. Verified: with this the node loads scan_topic=/robot_<id>/scan
    # and map_frame=robot_<id>/map, subscribes to /robot_<id>/scan and publishes
    # robot_<id>/map -> robot_<id>/odom.
    namespaced_slam_params = RewrittenYaml(
        source_file=slam_params,
        root_key=namespace,
        param_rewrites={},
        convert_types=False,
    )

    # slam_toolbox is a LIFECYCLE node. Launched as a plain Node it comes up in
    # 'unconfigured' and stays there forever: nothing configures it (Nav2's
    # lifecycle_manager manages only the Nav2 servers, not slam_toolbox), so it
    # never publishes map->odom and Nav2's global_costmap times out on the
    # missing robot_<id>/map frame. So launch it as a LifecycleNode and drive it
    # through configure->activate here, copying the stock
    # online_async_launch.py mechanism adapted to the /robot_<id> namespace:
    # emit CONFIGURE, then on reaching 'inactive' (configuring done) emit
    # ACTIVATE. use_lifecycle_manager:false tells the node not to wait on a bond
    # with an external manager — these launch-emitted transitions drive it.
    #
    # map_name: the YAML doesn't set it, and slam_toolbox's default map_name is
    # the absolute '/map', so the OccupancyGrid (and '<map_name>_metadata') would
    # publish on bare /map, /map_metadata regardless of namespace. Point it at
    # /robot_<id>/map so the map lands under the robot's namespace for Nav2/RViz.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        parameters=[namespaced_slam_params,
                    {'use_lifecycle_manager': False,
                     'use_sim_time': True,
                     'map_name': ['/robot_', robot_id, '/map']}],
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

    # ── 2. Nav2 stack ───────────────────────────────────────────────────────
    # navigation_launch.py does NOT set namespace= on its server Nodes — its
    # `namespace` arg only feeds RewrittenYaml(root_key=...) and the composition
    # container name. So the include alone brings the servers up at ROOT
    # (/bt_navigator, /planner_server, ...). PushROSNamespace pushes robot_<id>
    # onto every node in this GroupAction (the canonical nav2 bringup_launch.py
    # pattern), so they resolve to /robot_<id>/bt_navigator etc. — including the
    # lifecycle_manager, which then manages its plain-named node list under
    # /robot_<id>/. The `namespace` arg is STILL passed to the include so
    # RewrittenYaml wraps the (plain-keyed) params under robot_<id>: to match.
    #
    # SetRemap('/tf','/tf') / ('/tf_static','/tf_static') are global remaps applied
    # to every node inside this GroupAction (including navigation_launch.py's).
    # They shadow the stock ('/tf','tf') remap (first-match-wins), keeping Nav2 on
    # the shared global /tf tree instead of an isolated /robot_<id>/tf.
    nav2_group = GroupAction([
        PushROSNamespace(namespace=namespace),
        SetRemap('/tf', '/tf'),
        SetRemap('/tf_static', '/tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params,
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])

    # ── 3. Frontier explorer ────────────────────────────────────────────────
    # No namespace= here: FrontierExplorer(BasicNavigator) applies the robot_<id>
    # namespace itself from the robot_id parameter, so its actions resolve to
    # /robot_<id>/navigate_to_pose. The name must stay 'frontier_explorer' so the
    # launch-delivered robot_id parameter reaches the bootstrap read in main().
    explorer_node = Node(
        package='nsk_swarm',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[{
            'robot_id': ParameterValue(robot_id, value_type=int),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_robot_id,
        declare_slam_params,
        declare_nav2_params,
        slam_node,
        slam_configure,
        slam_activate,
        nav2_group,
        explorer_node,
    ])
