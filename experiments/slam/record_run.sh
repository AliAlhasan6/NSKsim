#!/usr/bin/env bash
# Record a five-robot run with ground truth. RUN=<name> (default b18_run1).
# Topic list is b16_run1's exact set plus per-robot ground-truth pose.
#
# EXPLORER=<K> switches to SINGLE-EXPLORER mode, the five sequential runs of
# HANDOFF_2026-09-19 section 7: robot K explores under Nav2 and the other four
# stay parked. Nothing extra is needed to park them -- swarm_sim.launch.py's
# `wander` defaults false, so a robot holds its spawn pose unless something
# drives it. Run order:
#
#   ros2 launch nsk_swarm swarm_sim.launch.py headless:=true nav_robots:=[K]
#   EXPLORER=K RUN=<name> experiments/slam/record_run.sh        # this script
#   ros2 launch nsk_swarm explore.launch.py robot_id:=K
#
# With EXPLORER unset this script behaves exactly as it did before the flag
# existed.
set -euo pipefail
RUN="${RUN:-b18_run1}"
EXPLORER="${EXPLORER:-}"
OUT="experiments/logs/${RUN%%_*}/${RUN}"

if [[ -z "$EXPLORER" ]]; then
  T=(/clock /tf /tf_static /kg_share /nsk/convergence)
  for N in 0 1 2 3 4; do
    T+=("/robot_$N/odom" "/robot_$N/scan" "/robot_$N/map" \
        "/robot_$N/map_metadata" "/model/robot_$N/pose")
  done
  # Every stack is expected to be up before recording starts, so every topic
  # is required now.
  REQUIRED=("${T[@]}")
  LATER=()
else
  [[ "$EXPLORER" =~ ^[0-4]$ ]] || {
    echo "EXPLORER must be a robot id 0..4, got '$EXPLORER'" >&2; exit 1; }
  K="$EXPLORER"
  # Required now: the sim is already running, so all five robots publish these
  # from the moment swarm_sim.launch.py came up. /model/robot_K/pose being in
  # this set is what makes the bag's first truth sample the spawn pose.
  T=(/clock /tf /tf_static /kg_share /nsk/convergence)
  for N in 0 1 2 3 4; do
    T+=("/robot_$N/odom" "/robot_$N/scan" "/model/robot_$N/pose" \
        "/robot_$N/joint_states")
  done
  REQUIRED=("${T[@]}")

  # Expected later: robot K's own stack, which only exists once
  # explore.launch.py has brought slam_toolbox and Nav2 up -- necessarily
  # AFTER this recorder is already running. Recorded, but not required at
  # startup; see the discovery note below.
  #
  # /robot_K/cmd_vel, not cmd_vel_nav and not cmd_vel_smoothed. Nav2 runs
  # three stages -- controller_server publishes cmd_vel_nav, velocity_smoother
  # turns that into cmd_vel_smoothed, collision_monitor turns that into
  # cmd_vel (cmd_vel_in_topic/cmd_vel_out_topic in
  # experiments/nav/nav2_robotN.yaml) -- and only the last one is bridged into
  # Gazebo, by swarm_sim.launch.py's
  # '/robot_N/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'. So it is the
  # only stage that actually moved the robot; a command can die at the
  # collision monitor without ever reaching the wheels. Unstamped Twist, not
  # TwistStamped: enable_stamped_cmd_vel is false on every producer in the
  # nav2 params, and the bridge spec pins the type independently.
  LATER=("/robot_$K/map" "/robot_$K/map_metadata" "/robot_$K/cmd_vel")
  T+=("${LATER[@]}")
fi

# A topic that is not advertised when the recorder starts is skipped AT
# STARTUP and then picked up by topic discovery, which is ON by default and
# polls every 100 ms (-p; --no-discovery turns it off, and its own help --
# "only topics present at startup will be recorded" -- is what documents the
# default). Verified on the installed rosbag2 0.26.11: a recorder started
# naming a topic nobody had advertised logged "Subscribed to topic" 5.2 s
# later and captured every message published to it.
#
# This replaces a b16-era note that read the startup half as permanent
# ("silently skips topics that are not yet advertised ... observed on
# b18_smoke: 20 of 30"). The half that is true is why the check below exists:
# it is a fail-fast on stacks that should ALREADY be up, not a rosbag2
# limitation. Topics in LATER are deliberately exempt from it.
#
# No --qos-profile-overrides-path, deliberately. /tf_static is published
# transient_local and latched once at startup; rosbag2 reads each publisher's
# offered QoS and adapts its subscription, which is why b18_run2 carries its 5
# /tf_static messages even though the recorder started well after the
# publishers did. Do not reach for --include-unpublished-topics to grab the
# LATER topics earlier either: it subscribes "with default QoS unless
# otherwise specified in a QoS overrides file", which would silently drop
# those latched transforms.
#
# The check RETRIES, because `ros2 topic list` is itself a discovery snapshot
# and not a reading of the graph. It starts a node, waits one discovery period
# and prints whatever has been announced by then; on this rig -- 5 robots,
# ~28 topics, every stack already up -- that snapshot is routinely partial.
# Rehearsal 4 aborted on one: 13 of 25 "missing", a random mix across robots
# and topic types including /model/robot_0/pose, which truth_odom_tf was
# receiving at that very moment (it had already logged its first transform
# from it). A second look answered differently.
#
# So a topic is only declared absent after PREFLIGHT_TRIES snapshots taken
# PREFLIGHT_WAIT_S apart, and each attempt re-checks ONLY what was still
# missing -- a topic seen once is settled. The gate itself is unchanged: a
# stack that never came up still aborts, and still aborts before any bag
# exists. What changed is that the gate no longer decides on one sample.
PREFLIGHT_TRIES="${PREFLIGHT_TRIES:-5}"     # snapshots, including the first
PREFLIGHT_WAIT_S="${PREFLIGHT_WAIT_S:-3}"   # seconds between them

missing=("${REQUIRED[@]}")
for (( attempt=1; attempt<=PREFLIGHT_TRIES; attempt++ )); do
  have="$(ros2 topic list)"
  still=()
  for t in "${missing[@]}"; do
    grep -qxF "$t" <<<"$have" || still+=("$t")
  done
  missing=("${still[@]+"${still[@]}"}")
  if (( ${#missing[@]} == 0 )); then
    echo "pre-flight: all ${#REQUIRED[@]} required topic(s) advertised," \
         "on attempt $attempt of $PREFLIGHT_TRIES"
    break
  fi
  if (( attempt < PREFLIGHT_TRIES )); then
    echo "pre-flight attempt $attempt of $PREFLIGHT_TRIES:" \
         "${#missing[@]} of ${#REQUIRED[@]} topic(s) not yet advertised;" \
         "re-checking those in ${PREFLIGHT_WAIT_S}s" >&2
    sleep "$PREFLIGHT_WAIT_S"
  fi
done
if (( ${#missing[@]} )); then
  echo "ABORT: ${#missing[@]} of ${#REQUIRED[@]} topic(s) not advertised" \
       "after $PREFLIGHT_TRIES attempt(s) ${PREFLIGHT_WAIT_S}s apart:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  echo "Bring up SLAM/Nav2 and wait for all stacks active, then re-run." >&2
  exit 1
fi

if (( ${#LATER[@]} )); then
  echo "single-explorer mode: robot_$EXPLORER explores, the other four are parked"
  echo "  ${#REQUIRED[@]} topic(s) advertised now; ${#LATER[@]} expected once explore.launch.py starts:"
  printf '    %s\n' "${LATER[@]}"
  echo "  start explore.launch.py only after this recorder is up."
fi

echo "recording ${#T[@]} topics -> $OUT"
exec ros2 bag record -o "$OUT" "${T[@]}"
