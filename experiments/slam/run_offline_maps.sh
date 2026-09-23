#!/usr/bin/env bash
# Offline per-robot SLAM over a replayed bag, one slam_toolbox per robot in
# sequence. BAG picks the bag directory (default experiments/bags/phaseB_run1)
# and RUN -- default: basename of BAG -- names every output, so maps, logs,
# params and tf records from different bags never overwrite each other. RATE
# scales replay speed; DUR_<N> truncates robot N's replay in seconds via
# --playback-duration (unset means -1: play everything). Bags that carry live
# slam_toolbox output, like b16, must first be stripped with
# experiments/slam/strip_bag_for_offline_slam.py -- it removes the recorded
# map->odom transforms that would fight the offline SLAM's -- producing e.g.
# experiments/logs/b16/b16_slamin, which is then the BAG for this script.
#
# SCAN_MATCHING=false turns slam_toolbox into a known-pose mapper; only use it
# on a bag whose odom->base_footprint is ground truth (rewrite_odom_from_truth.py).
#
#   source /opt/ros/jazzy/setup.bash
#   cd ~/Desktop/NSKsim
#   BAG=experiments/logs/b16/b16_slamin DUR_1=2143 RATE=2.0 bash experiments/slam/run_offline_maps.sh 1
#   BAG=experiments/logs/b18/b18r2_truth_slamin RUN=b18r2_truth RATE=2.0 \
#     SCAN_MATCHING=false bash experiments/slam/run_offline_maps.sh 0

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

BAG="${BAG:-$REPO_ROOT/experiments/bags/phaseB_run1}"
RUN="${RUN:-$(basename "$BAG")}"
TEMPLATE="$REPO_ROOT/experiments/slam/offline_mapping.yaml.template"
LAUNCH="$REPO_ROOT/experiments/slam/offline_slam.launch.py"
SAVER="$REPO_ROOT/experiments/slam/save_map.py"
MAPDIR="$REPO_ROOT/experiments/maps"
LOGDIR="$REPO_ROOT/experiments/logs"

# Replay rate. 1.0 is realtime. Lower it if the SLAM node cannot keep up --
# symptom is a map that covers only the start of the path.
RATE="${RATE:-1.0}"

# Seconds to wait after replay ends before saving. map_update_interval is 1.0 s,
# and the final loop-closure pass needs a moment beyond that.
SETTLE="${SETTLE:-8}"

# slam_toolbox's use_scan_matching. Default true -- the generated params are
# then byte-identical to every run before this switch existed.
#
# With it false slam_toolbox never calls MatchScan: each scan keeps the pose it
# was handed, map->odom stays identity, and the pose graph is never built, so
# do_loop_closing has no effect whatever it says. That makes this a
# deterministic known-pose mapper, which is only meaningful when the bag's
# odom->base_footprint IS ground truth -- see rewrite_odom_from_truth.py.
SCAN_MATCHING="${SCAN_MATCHING:-true}"

# FREE_SPACE_RELAY=true replays each scan through nsk_swarm's free_space_relay
# and points slam_toolbox at its output instead of the bag's raw topic.
#
# Karto drops every reading at or above the scan's range_max, +inf included
# (karto_sdk Karto.h:6169), so a beam that hits nothing clears nothing: on
# b2maps_e0t that is 57.47% of 11.7M beams, and the run mapped 63 m2 of a
# 391 m2 floor. The relay rewrites those beams to a value between
# RELAY_RANGE_THRESHOLD and range_max, which Karto traces as free space
# without marking an obstacle -- but only while max_laser_range is BELOW
# range_max, which is why the two move together here.
#
# The bag is untouched: the relay is a node in the replay, exactly as it is a
# node in the live run, so an online map and an offline map of the same run
# are built the same way. Default false renders a params file byte-identical
# to every one already in experiments/logs/.
FREE_SPACE_RELAY="${FREE_SPACE_RELAY:-false}"

# Both of these are DERIVED FROM THE BAG unless set, and that is the whole
# point: this script replays recorded scans, and a bag's range_max is whatever
# the sensor was when it was recorded. There are now two eras of bag -- every
# b16/b18/b2maps bag is a 3.5 m LDS-01, and anything recorded after
# BURGER_RANGE_MAX moved is an 8 m LDS-02 -- so a literal here would be wrong
# for one of them whichever value it took.
#
# Getting it wrong is quiet in one direction and loud in the other. Too HIGH a
# max_laser_range is inert: Karto clips rangeThreshold to the scan's own
# maxRange (Karto.h:3946-3949), so 8.0 against a 3.5 m bag just behaves as 3.5
# and nothing says so. Too high a RELAY THRESHOLD is not inert -- 7.9 against a
# 3.5 m bag leaves no value that is both traced and not dropped, so
# free_space_relay refuses to invent one, logs once, and passes every scan
# through UNCHANGED. The replay then completes, writes a map, and that map is
# silently the no-relay map under a name that says otherwise.
RELAY_HEADROOM="${RELAY_HEADROOM:-0.1}"

mkdir -p "$MAPDIR" "$LOGDIR"

die() { echo -e "\nFATAL: $*\n" >&2; exit 1; }

# ── children, and stopping every one of them ─────────────────────────────────
# Every process this script starts is registered here, and the trap below stops
# all of them on EVERY exit path: normal finish, die, and Ctrl-C.
#
# The PID bash hands back is not the process that matters. 'ros2 run' and
# 'ros2 launch' Popen the real executable and merely wait on it
# (ros2run/api/__init__.py), so $! is a wrapper:
#
#   run_offline_maps.sh
#   └─ ros2 run nsk_swarm free_space_relay          <- $RELAY_PID
#      └─ python3 .../lib/nsk_swarm/free_space_relay --ros-args ...
#
# 'kill $RELAY_PID' stops the wrapper and leaves the node reparented to init and
# still republishing /robot_N/scan_free. That is how the relay of
# b2maps_k1_cut1200 was alive a day after the script exited 0 -- on the SUCCESS
# path, not an error path -- while slam_toolbox, which had a pkill on its node
# name, was not. So every stop here takes the whole tree, and the node PIDs are
# registered too, once they exist, in case a wrapper dies first.
#
# Only registered PIDs and their descendants are ever signalled. Never
# 'pkill -f free_space_relay': a live sim on this machine runs relays with the
# same command line and those are not this script's to kill.
CHILD_PIDS=()
CHILD_CMDS=()
CHILDREN_STOPPED=0

# A PID's command line, empty if it is gone. The 2>/dev/null comes BEFORE the
# input redirection on purpose: redirections are applied left to right, and a
# '< /proc/<gone>/cmdline' that fails is reported by the shell on whatever stderr
# is current at that moment. Trailing, it printed 'No such file or directory' for
# every process this script had already stopped.
pid_cmd() {
  [[ -r "/proc/$1/cmdline" ]] || return 0
  tr '\0' ' ' 2>/dev/null < "/proc/$1/cmdline"
}

# NOT 'kill -0', which cannot tell a zombie from a running process. Our own
# background jobs are not the problem -- bash reaps those promptly -- but the
# NODES below are grandchildren that nothing in this shell ever waits on, so one
# that has exited while its wrapper has not yet collected it answers kill -0 as
# if it were alive. The escalation loop in stop_pids would then sit out its full
# budget on a corpse.
alive() { [[ -n "$(pid_cmd "$1")" ]]; }

register_child() {
  local pid="$1" cmd
  cmd="$(pid_cmd "$pid")"
  [[ -n "$cmd" ]] || return 0
  CHILD_PIDS+=("$pid")
  CHILD_CMDS+=("$cmd")
}

# Descendants of $1, DEEPEST FIRST: a wrapper must not be signalled before the
# node it holds, because once the parent is gone the child is reparented and
# pgrep -P can no longer find it.
descendants() {
  local kid
  for kid in $(pgrep -P "$1" 2>/dev/null); do
    descendants "$kid"
    echo "$kid"
  done
}

# Register what a wrapper has spawned by now. Call it once the child is known to
# be up -- after the lifecycle wait, after the relay's startup sleep -- so the
# node stays killable by PID even if its wrapper dies first.
register_spawned() {
  local kid
  while read -r kid; do register_child "$kid"; done < <(descendants "$1")
}

# PID numbers are recycled. Signal a registered PID only while it is still the
# process that was registered.
is_registered_process() {
  local pid="$1" i
  for (( i = 0; i < ${#CHILD_PIDS[@]}; i++ )); do
    if [[ "${CHILD_PIDS[i]}" == "$pid" ]]; then
      [[ "$(pid_cmd "$pid")" == "${CHILD_CMDS[i]}" ]]
      return $?
    fi
  done
  return 1
}

# Stop registered PIDs and everything below them.
#
# SIGTERM, NOT SIGINT, even though SIGINT is the signal this stack is written for
# (rclpy.spin catches KeyboardInterrupt, ros2 launch shuts its nodes down in
# order). When job control is off -- which it is in any script -- bash sets
# SIGINT and SIGQUIT to SIG_IGN in every command it starts with '&', and an
# ignored-on-entry signal cannot be re-enabled from inside. So every wrapper
# started here is deaf to SIGINT by construction: measured, 'ros2 bag play' sat
# through a SIGINT to its whole process group and only died on the SIGKILL below.
# SIGTERM is not special-cased that way and reaches all of them.
#
# SIGKILL 10 s later for anything that wanted longer. Nothing here needs a
# graceful exit: the map is already written by save_map.py before slam_toolbox is
# stopped, and the relay is a stateless republisher.
stop_pids() {
  local pid kid victims=() live=() tops=()
  # A node is both registered in its own right and reachable as a descendant of
  # its wrapper, so the list has to be deduplicated or it signals twice and says
  # so twice.
  local -A seen=()
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    is_registered_process "$pid" || continue
    tops+=("$pid")
    while read -r kid; do
      [[ -n "$kid" && -z "${seen[$kid]:-}" ]] || continue
      seen[$kid]=1
      victims+=("$kid")
    done < <(descendants "$pid")
    if [[ -z "${seen[$pid]:-}" ]]; then seen[$pid]=1; victims+=("$pid"); fi
  done
  (( ${#victims[@]} )) || return 0
  for pid in "${victims[@]}"; do
    # Skip what has already gone: on a Ctrl-C the terminal has signalled the
    # whole foreground group itself, and some of these are dead before we look.
    alive "$pid" || continue
    echo "      stopping $pid: $(pid_cmd "$pid" | cut -c1-70)"
    kill -TERM "$pid" 2>/dev/null
  done
  for _ in $(seq 1 20); do
    live=()
    for pid in "${victims[@]}"; do alive "$pid" && live+=("$pid"); done
    (( ${#live[@]} )) || break
    sleep 0.5
  done
  for pid in "${live[@]}"; do
    echo "      SIGKILL $pid: $(pid_cmd "$pid" | cut -c1-70)"
    kill -KILL "$pid" 2>/dev/null
  done
  # Reap our own background jobs, so a later 'is it gone?' is not answered by a
  # zombie that still looks like a process to kill -0.
  for pid in "${tops[@]}"; do wait "$pid" 2>/dev/null; done
  return 0
}

# The trap. Idempotent, because INT runs it and the exit that follows runs it
# again, and $?-preserving, so the EXIT path does not rewrite the script's own
# exit status. Reverse registration order: last started, first stopped.
stop_children() {
  local rc=$?
  [[ "$CHILDREN_STOPPED" == 1 ]] && return "$rc"
  CHILDREN_STOPPED=1
  local i rev=() n=0
  for (( i = ${#CHILD_PIDS[@]} - 1; i >= 0; i-- )); do
    rev+=("${CHILD_PIDS[i]}")
    is_registered_process "${CHILD_PIDS[i]}" && n=$((n + 1))
  done
  if (( n )); then
    echo -e "\n  cleanup: $n process(es) still running" >&2
    stop_pids "${rev[@]}"
  fi
  return "$rc"
}

trap stop_children EXIT
trap 'stop_children; exit 130' INT
trap 'stop_children; exit 143' TERM

# ── preconditions ───────────────────────────────────────────────────────────

[[ -d "$BAG"      ]] || die "bag not found: $BAG"
[[ -f "$TEMPLATE" ]] || die "params template not found: $TEMPLATE"
[[ -f "$LAUNCH"   ]] || die "launch file not found: $LAUNCH"
[[ -f "$SAVER"    ]] || die "save_map.py not found: $SAVER"

[[ "$SCAN_MATCHING" == "true" || "$SCAN_MATCHING" == "false" ]] || \
  die "SCAN_MATCHING must be exactly 'true' or 'false', got: $SCAN_MATCHING"

[[ "$FREE_SPACE_RELAY" == "true" || "$FREE_SPACE_RELAY" == "false" ]] || \
  die "FREE_SPACE_RELAY must be exactly 'true' or 'false', got: $FREE_SPACE_RELAY"

if [[ "$FREE_SPACE_RELAY" == "true" ]]; then
  SCAN_SUFFIX="scan_free"
  ros2 pkg executables nsk_swarm 2>/dev/null | grep -q free_space_relay || \
    die "free_space_relay is not in the sourced nsk_swarm. Build from the \
repo root (colcon build --packages-select nsk_swarm) and source \
install/setup.bash."
else
  SCAN_SUFFIX="scan"
fi

command -v ros2 >/dev/null 2>&1 || \
  die "ros2 is not on PATH. Run: source /opt/ros/jazzy/setup.bash"

ros2 pkg prefix slam_toolbox >/dev/null 2>&1 || \
  die "slam_toolbox is not installed or not sourced."

# The handoff's lesson: check for strays BEFORE launching, not after wondering
# why the map looks wrong. A leftover slam_toolbox from a previous attempt will
# happily publish onto /map and be saved as if it were this run's output.
if pgrep -af 'slam_toolbox|ros2 bag play' | grep -q .; then
  echo "processes already running:"
  pgrep -af 'slam_toolbox|ros2 bag play'
  die "kill these first -- they will contaminate /map."
fi

# Clock source is measured, not assumed. use_sim_time:=true needs a /clock:
# a stripped bag (b16_slamin) records its own, phaseB_run1 does not. --clock
# is added only when the bag lacks one -- playing both would hand slam_toolbox
# two competing clocks.
if ros2 bag info "$BAG" | grep -q 'Topic: /clock '; then
  CLOCK_ARGS=()
  echo "bag publishes /clock -- replaying without --clock"
else
  CLOCK_ARGS=(--clock)
  echo "bag has no /clock -- synthesising it with --clock"
fi

ROBOTS=("$@")
if [[ ${#ROBOTS[@]} -eq 0 ]]; then
  ROBOTS=(0 1 2 3 4)
fi

# ── the lidar ceiling, read off the bag ─────────────────────────────────────
# Measured, not assumed, for the reason on RELAY_HEADROOM above: this replays
# recorded scans, and the recorded range_max is a property of the bag rather
# than of the current launch files. Read from the FIRST requested robot; all
# five carry the same sensor, and a bag that disagrees between robots is a
# broken bag rather than something to average.
#
# BAG_RANGE_MAX=<value> overrides, for a hand-built or repaired bag.
if [[ -z "${BAG_RANGE_MAX:-}" ]]; then
  # stderr to a file, NOT folded into the value: rosbag2's storage plugins log
  # on stderr, and 2>&1 here would splice a log line into the number on any run
  # that merely warned. The regex below would then reject a perfectly good bag.
  RANGE_ERR="$(mktemp)"
  BAG_RANGE_MAX="$(python3 "$REPO_ROOT/experiments/slam/bag_range_max.py" \
                     "$BAG" "${ROBOTS[0]}" 2>"$RANGE_ERR")" || \
    die "could not read range_max from $BAG:
$(cat "$RANGE_ERR")"
  rm -f "$RANGE_ERR"
fi

[[ "$BAG_RANGE_MAX" =~ ^[0-9]+(\.[0-9]+)?$ ]] || \
  die "BAG_RANGE_MAX is not a number: '$BAG_RANGE_MAX'"

if [[ "$FREE_SPACE_RELAY" == "true" ]]; then
  RELAY_RANGE_THRESHOLD="${RELAY_RANGE_THRESHOLD:-$(awk -v m="$BAG_RANGE_MAX" \
      -v h="$RELAY_HEADROOM" 'BEGIN { printf "%.4g", m - h }')}"
  MAX_LASER_RANGE="$RELAY_RANGE_THRESHOLD"
  # The inequality free_space_relay rests on, checked HERE rather than
  # discovered 40 minutes into a replay that quietly produced the no-relay map.
  awk -v t="$RELAY_RANGE_THRESHOLD" -v m="$BAG_RANGE_MAX" \
      'BEGIN { exit !(t < m && t > 0) }' || \
    die "RELAY_RANGE_THRESHOLD $RELAY_RANGE_THRESHOLD is not strictly between \
0 and the bag's range_max $BAG_RANGE_MAX. The relay would have no value that is \
both traced by Karto and not dropped, so it would pass every scan through \
unchanged and this run would silently be a no-relay run."
  echo "bag range_max $BAG_RANGE_MAX -- relay threshold $RELAY_RANGE_THRESHOLD"
else
  MAX_LASER_RANGE="${MAX_LASER_RANGE:-$BAG_RANGE_MAX}"
  echo "bag range_max $BAG_RANGE_MAX -- max_laser_range $MAX_LASER_RANGE"
fi

# ── per-robot run ───────────────────────────────────────────────────────────

FAILED=()

for N in "${ROBOTS[@]}"; do
  echo
  echo "════════════════════════════════════════════════════════════"
  echo "  robot_$N"
  echo "════════════════════════════════════════════════════════════"

  PARAMS="$LOGDIR/offline_mapping_${RUN}_robot_$N.yaml"
  LOG="$LOGDIR/offline_slam_${RUN}_robot_$N.log"
  OUT="$MAPDIR/${RUN}_robot$N"

  # Params live in experiments/logs/, not /tmp -- /tmp is wiped on reboot and
  # these are the record of what produced each map.
  sed -e "s/__ROBOT__/$N/g" -e "s/__SCAN_MATCHING__/$SCAN_MATCHING/g" \
      -e "s/__SCAN_SUFFIX__/$SCAN_SUFFIX/g" \
      -e "s/__MAX_LASER_RANGE__/$MAX_LASER_RANGE/g" \
      "$TEMPLATE" > "$PARAMS"
  grep -q "robot_$N/odom" "$PARAMS" || die "substitution failed in $PARAMS"
  grep -q "use_scan_matching: $SCAN_MATCHING" "$PARAMS" || \
    die "scan-matching substitution failed in $PARAMS"
  grep -q "scan_topic: /robot_$N/$SCAN_SUFFIX" "$PARAMS" || \
    die "scan-topic substitution failed in $PARAMS"
  grep -q "max_laser_range: $MAX_LASER_RANGE" "$PARAMS" || \
    die "max-laser-range substitution failed in $PARAMS"
  # __UPPERCASE__, not bare '__': ros__parameters is not a placeholder.
  grep -qE '__[A-Z_]+__' "$PARAMS" && \
    die "unsubstituted placeholder left in $PARAMS"

  rm -f "$OUT.pgm" "$OUT.yaml"

  echo "[1/4] starting slam_toolbox (log: ${LOG#$REPO_ROOT/})"
  ros2 launch "$LAUNCH" params_file:="$PARAMS" > "$LOG" 2>&1 &
  SLAM_PID=$!
  register_child "$SLAM_PID"

  # Wait for the node to reach 'active'. Polling the lifecycle state is the
  # honest check -- a running process proves nothing, the node can sit in
  # 'unconfigured' indefinitely.
  echo -n "[2/4] waiting for lifecycle 'active' "
  ACTIVE=0
  for _ in $(seq 1 40); do
    sleep 1
    echo -n "."
    STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null || true)
    if [[ "$STATE" == *"active"* ]]; then ACTIVE=1; break; fi
  done
  echo
  # The launched node, not just the 'ros2 launch' wrapper holding it. Registered
  # whether or not it activated: a node stuck in 'unconfigured' is still a
  # process, and the failure path below has to take it with it.
  register_spawned "$SLAM_PID"
  if [[ $ACTIVE -ne 1 ]]; then
    echo "  node never reached 'active'. Last state: ${STATE:-<none>}"
    echo "  tail of $LOG:"; tail -20 "$LOG"
    stop_pids "$SLAM_PID"
    FAILED+=("robot_$N (never activated)")
    continue
  fi

  # Only this robot's inputs are replayed; the other four robots' topics stay
  # in the bag. START_<N> and DUR_<N> cut the replay so a bag can be windowed
  # per robot.
  DUR_VAR="DUR_$N"
  DUR="${!DUR_VAR:--1}"
  START_VAR="START_$N"
  START="${!START_VAR:-0}"
  # The relay must be subscribed before the first scan is replayed: it is a
  # plain republisher with no history, so anything it misses is simply not
  # mapped. Started per robot and killed with the replay, so two robots in one
  # invocation cannot leave each other's relay running.
  RELAY_PID=""
  if [[ "$FREE_SPACE_RELAY" == "true" ]]; then
    echo "[3/4] starting free_space_relay for robot_$N " \
         "(threshold $RELAY_RANGE_THRESHOLD m)"
    ros2 run nsk_swarm free_space_relay --ros-args \
        -p "robot_id:=$N" -p "range_threshold:=$RELAY_RANGE_THRESHOLD" \
        >> "$LOG" 2>&1 &
    RELAY_PID=$!
    register_child "$RELAY_PID"
    sleep 2
    # The relay node itself, not just the wrapper that Popen'd it.
    register_spawned "$RELAY_PID"
    # The die below is what leaves slam_toolbox running with nothing to stop it,
    # which is why it is worth saying that the trap now covers it: measured, the
    # FATAL here takes the whole slam tree with it.
    alive "$RELAY_PID" || die "free_space_relay died at startup; see $LOG"
  fi

  echo "[3/4] replaying $RUN at rate $RATE, start-offset $START, playback-duration $DUR"

  # START_<N> is kept for completeness but /tf_static sits at bag time 0 and
  # --start-offset skips it; cut segments with strip_bag_for_offline_slam.py
  # --start/--duration instead, which re-timestamps /tf_static.
  # Backgrounded and waited on rather than run in the foreground: bash defers a
  # trap until the foreground command returns, so a Ctrl-C mid-replay would
  # otherwise not be handled until the replay itself decided to stop -- and a
  # SIGTERM to this script would leave the replay running with nobody to kill
  # it. The replay's exit status is not consulted, exactly as before.
  ros2 bag play "$BAG" "${CLOCK_ARGS[@]}" --rate "$RATE" \
      --start-offset "$START" --playback-duration "$DUR" \
      --topics /clock /tf /tf_static "/robot_$N/scan" "/robot_$N/odom" \
      >> "$LOG" 2>&1 &
  PLAY_PID=$!
  register_child "$PLAY_PID"
  wait "$PLAY_PID"

  echo "      settling ${SETTLE}s for the final map publish"
  sleep "$SETTLE"

  if [[ -n "$RELAY_PID" ]]; then
    # After the settle, not before: the last scans are still being processed,
    # and a relay killed early truncates the map by however much is in flight.
    stop_pids "$RELAY_PID"
    grep -a 'beams filled' "$LOG" | tail -1
  fi

  echo "[4/4] saving map"
  
  # slam_toolbox's correction. NOT in the saved .yaml, but required to place the
  # map in world coordinates: world = spawn o (map->odom)^-1 o map_origin o cell.
  # Omitting it cost a session of wrong on-wall figures (2026-09-05).
  # 'timeout' signals the whole group it started, so tf2_echo cannot outlive it
  # on expiry -- but registering it covers the 5 s window in which this script
  # itself could be interrupted.
  timeout 5 ros2 run tf2_ros tf2_echo "robot_$N/map" "robot_$N/odom" \
      --ros-args -p use_sim_time:=true \
      > "$LOGDIR/map_to_odom_${RUN}_robot_$N.txt" 2>&1 &
  TF_PID=$!
  register_child "$TF_PID"
  wait "$TF_PID"
  head -8 "$LOGDIR/map_to_odom_${RUN}_robot_$N.txt"

  python3 "$SAVER" --topic /map --out "$OUT" --timeout 30 >> "$LOG" 2>&1 &
  SAVE_PID=$!
  register_child "$SAVE_PID"
  wait "$SAVE_PID"
  SAVE_RC=$?

  # The tree, which is the 'ros2 launch' wrapper plus the node registered above.
  # This replaces a 'pkill -f async_slam_toolbox_node' that was the only reason
  # slam_toolbox did not leak the way the relay did -- and that would have
  # reached outside this script's own children to do it.
  stop_pids "$SLAM_PID"

  # ── verification ──────────────────────────────────────────────────────────
  # A file existing is not the check. experiments/maps holds a dozen 27-byte
  # '*_startblocked.pgm' files that were "written successfully" and contain
  # nothing. Verify the grid actually has content.
  if [[ $SAVE_RC -ne 0 || ! -s "$OUT.pgm" ]]; then
    echo "  save failed (rc=$SAVE_RC). tail of $LOG:"; tail -20 "$LOG"
    FAILED+=("robot_$N (save failed)")
    continue
  fi

  python3 - "$OUT.pgm" <<'PYEOF'
import sys
# nav2 trinary PGM: 0 = occupied, 205 = unknown, 254 = free.
path = sys.argv[1]
data = open(path, 'rb').read()
# Skip the P5 header: magic, dims, maxval -- three whitespace-separated fields
# after 'P5', with '#' comment lines possible.
i, fields = 2, []
while len(fields) < 3:
    while i < len(data) and data[i:i+1].isspace():
        i += 1
    if data[i:i+1] == b'#':
        while i < len(data) and data[i:i+1] != b'\n':
            i += 1
        continue
    j = i
    while j < len(data) and not data[j:j+1].isspace():
        j += 1
    fields.append(data[i:j]); i = j
i += 1
w, h = int(fields[0]), int(fields[1])
px = data[i:]
occ = px.count(0)
free = px.count(254)
unk = px.count(205)
total = w * h
print(f'  grid {w}x{h} = {total} cells: '
      f'{occ} occupied, {free} free, {unk} unknown')
if occ < 50:
    print(f'  VERIFY FAILED: only {occ} occupied cells -- no walls mapped.')
    sys.exit(1)
if free < 200:
    print(f'  VERIFY FAILED: only {free} free cells -- no space carved.')
    sys.exit(1)
print('  VERIFY OK')
PYEOF

  if [[ $? -ne 0 ]]; then
    FAILED+=("robot_$N (empty or degenerate grid)")
    continue
  fi

  echo "  wrote ${OUT#$REPO_ROOT/}.pgm / .yaml"
done

# ── summary ─────────────────────────────────────────────────────────────────

echo
echo "════════════════════════════════════════════════════════════"
if [[ ${#FAILED[@]} -eq 0 ]]; then
  echo "  all requested robots mapped and verified"
  ls -la "$MAPDIR/${RUN}_robot"*.pgm
  exit 0
fi
echo "  FAILURES:"
printf '    %s\n' "${FAILED[@]}"
exit 1
