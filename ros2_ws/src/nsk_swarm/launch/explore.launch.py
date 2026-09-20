#!/usr/bin/env python3
"""explore.launch.py — live SLAM + Nav2 + frontier explorer for ONE robot.

Brings up, all under the /robot_<id> namespace (default robot_0):
  1. slam_toolbox online_async node        -> live map (robot_<id>/map -> robot_<id>/odom on /tf)
  2. Nav2 stack (navigation_launch.py)     -> plans and drives /robot_<id>/cmd_vel
  3. frontier_explorer node                -> sends NavigateToPose goals at map frontiers

nav2:=false drops 2 and 3 and brings up 1 alone — SLAM only: a mapper-only
robot, which maps whatever it is driven past but neither plans nor drives.

This proves the explore -> plan -> drive -> map loop for one robot and lets us
measure RTF before scaling to 5 robots.

PREREQUISITE: swarm_sim.launch.py must ALREADY be running with
nav_robots:=[<id>] so the target robot's wander driver is muted and its
/robot_<id>/cmd_vel is free for Nav2 to command. This launch adds only the SLAM,
Nav2 and explorer nodes on top of that running sim.

A robot launched with nav2:=false must NOT appear in that nav_robots list.
nav_robots exists to MUTE the wander driver, and a mapper-only robot has no
other source of motion — muting it leaves the robot stationary, mapping one
static view of its spawn pose forever. That is the failure that invalidated
rung2f through rung2i.

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
  ros2 launch nsk_swarm explore.launch.py rviz:=true      # + RViz (eyeballing only; see note)

RViz is OFF by default and MUST stay so for timed runs: its rendering steals
cycles and corrupts the real-time-factor numbers this rig exists to measure.
rviz:=true opens a view preconfigured for the robot's namespace (Fixed Frame
robot_<id>/map, Map /robot_<id>/map, LaserScan /robot_<id>/scan).
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
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

# slam_toolbox's max_laser_range when free_space_relay is on, and the threshold
# the relay fills just above. It must sit strictly BELOW the scan's range_max or
# the relay's fill is dropped by Karto exactly as +inf is -- see
# free_space_relay.py.
#
# The simulated lidar is a ROBOTIS LDS-02 -- detection distance 160 to 8000 mm
# per the ROBOTIS e-manual, the sensor that replaced the LDS-01 on the
# TurtleBot3 Burger in 2022 -- not the 120 to 3500 mm LDS-01 that the stock
# turtlebot3_gazebo model still ships; make_namespaced_burger_sdf rewrites both
# bounds at spawn.
#
# So range_max is BURGER_RANGE_MAX in swarm_sim.launch.py, 8.0
# m; this is that minus the same 0.1 m of headroom the 3.5 m era used, so the
# fill lands at the midpoint 7.95 m, 50 mm clear of both ends against a 0.05 m
# map resolution. The invariant, not the literal, is what matters: the two files
# are checked against each other in test_launch_descriptions.py.
#
# The headroom costs the last 0.1 m of every genuine hit -- readings in
# [7.9, 8.0) are traced as free to 7.9 m with no obstacle marked, instead of
# marking one where they landed. At the old pair that band was 96114 readings:
# 1.93% of b2maps_e0t's 4979217 returns and 0.82% of all its beams. That figure
# is HISTORICAL, about a 3.5 m bag at a 3.4 m threshold, and does not carry over
# -- the [7.9, 8.0) band cannot be known until a run at 8 m produces it, and
# should be re-measured then rather than assumed to be the same fraction.
RELAY_RANGE_THRESHOLD = 7.9


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    navigation_launch = os.path.join(
        nav2_bringup_share, 'launch', 'navigation_launch.py')
    nsk_swarm_share = get_package_share_directory('nsk_swarm')

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

    declare_nav2 = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Bring up the Nav2 stack. Gates the frontier explorer as '
                    'well, not Nav2 alone: FrontierExplorer IS a '
                    'BasicNavigator and works only by sending NavigateToPose '
                    'goals, so without the stack under it there is nothing it '
                    'can do — a separate explorer flag would buy nothing but '
                    'the invalid combination (no Nav2, explorer on), an '
                    'explorer that comes up only to block forever on servers '
                    'nobody launched. false therefore leaves slam_toolbox '
                    'alone: a mapper-only robot. MUST default true: any '
                    'command that does not set it has to reproduce its '
                    'previous rung exactly.')

    declare_precheck = DeclareLaunchArgument(
        'reachability_precheck', default_value='false',
        description='Ask the planner whether a frontier is plannable BEFORE '
                    'dispatching a goal to it (ComputePathToPose per candidate, '
                    'walking the nearest-first list until one is reachable), and '
                    're-query it when a goal fails with a FollowPath 1xx code '
                    'that masked the planner\'s own verdict. MUST default false: '
                    'it is the A/B variable, so a run that does not set it '
                    'explicitly has to reproduce the previous rung exactly. Costs '
                    'at most PRECHECK_MAX_PROBES round trips or '
                    'PRECHECK_CYCLE_BUDGET wall seconds per selection cycle, '
                    'whichever binds first (see also PLAN_CALL_WAIT).')

    declare_escape_distance = DeclareLaunchArgument(
        # Literal, not an import of ESCAPE_DISTANCE: this file imports nothing
        # from nsk_swarm, and pulling frontier_explorer in would drag rclpy and
        # nav2_simple_commander into every launch-description evaluation. The
        # two are held equal by a test instead (test_launch_descriptions.py).
        'escape_distance', default_value='0.6',
        description='How far one inflation-pocket escape displaces the robot, '
                    'in metres. When the pre-check would defer and the halt '
                    'verdict reads INSCRIBED_INFLATED under a SLAM cell that is '
                    'free or unknown, the explorer dispatches a short goal in '
                    'the cheapest drivable direction instead of waiting for a '
                    'map refresh that cannot clear an inflation layer. The '
                    'default is the LOW END of the success band measured in ONE '
                    'run (rung2h: plans succeeded 0.43-1.81 m, failed 0.66-6.34 '
                    'm, n=1, overlapping) — an observation to be re-measured, '
                    'not a calibrated value, which is why it is a launch '
                    'argument. 0 disables the escape.')

    declare_scan_matching = DeclareLaunchArgument(
        'scan_matching', default_value='true',
        description='slam_toolbox use_scan_matching. Default true is the '
                    'value slam_robotN.yaml already carries, so a command '
                    'that does not set it reproduces its previous rung '
                    'exactly. false turns slam_toolbox into a known-pose '
                    'mapper: MatchScan is never called, each scan keeps the '
                    'pose it was handed, map->odom stays identity, and the '
                    'pose graph is never built — so do_loop_closing has no '
                    'effect whatever the YAML says, TryCloseLoop sitting '
                    'inside the same branch. ONLY meaningful when the poses '
                    'are already correct, i.e. with swarm_sim.launch.py '
                    'truth_odom_robots:=[<id>]; on wheel odometry it removes '
                    'the only thing correcting the drift. Measured on '
                    'b18_run2 robot_0 offline (HANDOFF_2026-09-19 §5): with '
                    'truth poses the matcher COSTS 20 points of world fit '
                    '(96.1% off against 76.1% on) and invents 0.98 m of '
                    'map->odom drift that the poses do not have. n=1.')

    declare_free_space_relay = DeclareLaunchArgument(
        'free_space_relay', default_value='false',
        description='Feed slam_toolbox a scan whose no-return beams clear '
                    'free space, instead of the raw one where they clear '
                    'nothing. Karto drops any reading at or above the scan '
                    'range_max, +inf included (Karto.h:6169), so on b2maps_e0t '
                    '57.47% of 11.7M beams cleared nothing and the run mapped '
                    '63 m2 of a 391 m2 floor. Still needed at 8 m: 20.5% of '
                    'beams return nothing there, and the relay is worth 209 -> '
                    '335 m2 of free space on e0t\'s own trajectory. It is NOT '
                    'a substitute for the range and the range is not a '
                    'substitute for it -- Karto sizes the grid from filtered '
                    'readings only (Karto.h:5644), so a filled beam can never '
                    'enlarge the map box. Range sets the box, the relay fills '
                    'it. true starts free_space_relay, '
                    'points scan_topic at /robot_<id>/scan_free, and lowers '
                    f'max_laser_range to {RELAY_RANGE_THRESHOLD} so the '
                    'relay\'s fill is traced as free space rather than '
                    'dropped or marked as a wall. Nav2 keeps the RAW scan: '
                    'the fill is a convention Karto reads and an obstacle '
                    'layer would not. Default false leaves the launch exactly '
                    'as it was, YAML included.')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch an RViz preconfigured for robot_<id> (Fixed Frame '
                    'robot_<id>/map, Map /robot_<id>/map, LaserScan '
                    '/robot_<id>/scan). MUST default false: RViz rendering '
                    'steals cycles and CORRUPTS the RTF measurements this rig '
                    'exists to take. Enable only for eyeballing, never during a '
                    'timed run.')

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
    def _relay_on(context) -> bool:
        return LaunchConfiguration('free_space_relay').perform(
            context).lower() in ('true', '1')

    def _slam_overrides(context) -> dict:
        """The free-space-relay overrides, or nothing at all.

        Empty by default, and that is the point: with the flag off
        slam_toolbox is handed exactly the parameter dict it was handed before
        this argument existed, and slam_robotN.yaml remains the single source
        for scan_topic and max_laser_range. Naming those two here
        unconditionally -- even with today's values -- would make the launch
        file a second source that goes stale the moment the YAML is edited.
        """
        if not _relay_on(context):
            return {}
        rid = LaunchConfiguration('robot_id').perform(context)
        return {
            # What the relay publishes; the raw scan is untouched and still
            # feeds Nav2's costmaps.
            'scan_topic': f'/robot_{rid}/scan_free',
            # Must be strictly BELOW the scan's range_max for the relay's fill
            # to be traced rather than dropped -- see free_space_relay.py.
            'max_laser_range': RELAY_RANGE_THRESHOLD,
        }

    def _slam_actions(context):
        """The slam_toolbox node and the two events that drive its lifecycle.

        Built at execute time so the overrides above can be ABSENT rather than
        present-with-the-old-value. The three are returned together because
        the events match the node by identity.
        """
        node = _make_slam_node(_slam_overrides(context))
        return [node, _slam_configure(node), _slam_activate(node)]

    def _make_slam_node(overrides: dict) -> LifecycleNode:
        return LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            parameters=[namespaced_slam_params,
                        {'use_lifecycle_manager': False,
                         'use_sim_time': True,
                         'map_name': ['/robot_', robot_id, '/map'],
                     # Overridden HERE rather than through RewrittenYaml's
                     # param_rewrites: a later dict wins over an earlier
                     # params file, so this needs no second rewrite pass and
                     # leaves slam_robotN.yaml on disk untouched — that file
                     # stays the run-of-record default, and the override is
                     # visible in the launch command instead of hidden in a
                     # rewritten temp file. value_type=bool for the same
                     # reason reachability_precheck needs it: without the
                     # cast 'scan_matching:=0' infers as the INT 0, which
                     # rclpy will not accept for a bool parameter, and the
                     # node would die at startup on a reasonable spelling.
                         'use_scan_matching': ParameterValue(
                             LaunchConfiguration('scan_matching'),
                             value_type=bool),
                         **overrides}],
            output='screen',
        )

    def _slam_configure(node) -> EmitEvent:
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
        )

    def _slam_activate(node) -> RegisterEventHandler:
        return RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
        )

    slam_group = OpaqueFunction(function=_slam_actions)

    # The relay itself, gated the same way and for the same reason as
    # swarm_sim's truth_odom_tf group: with the flag off this returns NOTHING,
    # so a default run has no such node in the tree rather than a suppressed
    # one. It reads the raw scan and publishes /robot_<id>/scan_free, which is
    # what the overrides above point slam_toolbox at.
    def _relay_actions(context):
        if not _relay_on(context):
            return []
        rid = LaunchConfiguration('robot_id').perform(context)
        return [Node(
            package='nsk_swarm',
            executable='free_space_relay',
            name=f'free_space_relay_robot_{rid}',
            parameters=[{'robot_id': int(rid),
                         'range_threshold': RELAY_RANGE_THRESHOLD}],
            output='screen',
        )]

    relay_group = OpaqueFunction(function=_relay_actions)

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
    ], condition=IfCondition(LaunchConfiguration('nav2')))

    # ── 3. Frontier explorer ────────────────────────────────────────────────
    # No namespace= here: FrontierExplorer(BasicNavigator) applies the robot_<id>
    # namespace itself from the robot_id parameter, so its actions resolve to
    # /robot_<id>/navigate_to_pose.
    #
    # NO name= either. This process spins TWO nodes concurrently: the sensor node
    # (code name frontier_explorer_sensors) and the navigator (code name
    # frontier_explorer). A launch name= becomes a bare `-r __node:=frontier_explorer`
    # remap, which is PROCESS-WIDE, not scoped to one node — so it renames BOTH,
    # collapsing them into two /robot_<id>/frontier_explorer entries and a rosout
    # "duplicate publisher" warning. Dropping name= lets the code-declared names
    # stand (…/frontier_explorer + …/frontier_explorer_sensors), no collision.
    # The robot_id parameter still arrives: with no namespace= set here the node
    # name isn't fully specified, so launch_ros writes the params under the /**
    # wildcard (see Node._create_params_file_from_dict) — they reach every node in
    # the process, including main()'s bootstrap read, regardless of node name.
    explorer_node = Node(
        package='nsk_swarm',
        executable='frontier_explorer',
        parameters=[{
            'robot_id': ParameterValue(robot_id, value_type=int),
            # value_type=bool pins the type instead of leaving it to launch_ros'
            # inference. Inference reads the word forms correctly ('false' ->
            # False), but 'reachability_precheck:=1' infers as the INT 1, and
            # main() declares this parameter with a False default — a bool — which
            # rclpy will not accept an int for, so the explorer would die at
            # startup on a spelling of "on" that looks perfectly reasonable. The
            # cast maps 0/1 onto False/True and makes every spelling work.
            'reachability_precheck': ParameterValue(
                LaunchConfiguration('reachability_precheck'), value_type=bool),
            # Same cast, same reason: main() declares this one with a float
            # default, and 'escape_distance:=1' would otherwise infer as the INT
            # 1 and be refused at startup on a perfectly reasonable spelling.
            'escape_distance': ParameterValue(
                LaunchConfiguration('escape_distance'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    # ── 4. RViz (opt-in; OFF by default) ─────────────────────────────────────
    # Ship one namespace-agnostic rviz/explore.rviz template with a ROBOT_NS
    # placeholder; resolve it to robot_<id> at launch time and hand rviz2 the
    # rewritten copy via -d. This has to be an OpaqueFunction because the fix is a
    # string substitution INSIDE the file's contents (frames + topics), which a
    # plain launch substitution can't do — the same reason SLAM's params go
    # through RewrittenYaml above. Gated on rviz:=true; returns nothing (so rviz
    # never starts) when false, the default, to keep RTF measurements clean.
    def _rviz_actions(context):
        if LaunchConfiguration('rviz').perform(context).lower() not in ('true', '1'):
            return []
        ns = 'robot_' + LaunchConfiguration('robot_id').perform(context)
        template = os.path.join(nsk_swarm_share, 'rviz', 'explore.rviz')
        with open(template) as f:
            cfg = f.read().replace('ROBOT_NS', ns)
        tmp = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'explore_{ns}_', suffix='.rviz', delete=False)
        tmp.write(cfg)
        tmp.close()
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=ns,
            arguments=['-d', tmp.name],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )]

    rviz_group = OpaqueFunction(function=_rviz_actions)

    return LaunchDescription([
        declare_robot_id,
        declare_slam_params,
        declare_nav2_params,
        declare_nav2,
        declare_precheck,
        declare_escape_distance,
        declare_scan_matching,
        declare_free_space_relay,
        declare_rviz,
        relay_group,
        slam_group,
        nav2_group,
        explorer_node,
        rviz_group,
    ])
