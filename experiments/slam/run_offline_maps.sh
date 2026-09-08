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
#   source /opt/ros/jazzy/setup.bash
#   cd ~/Desktop/NSKsim
#   BAG=experiments/logs/b16/b16_slamin DUR_1=2143 RATE=2.0 bash experiments/slam/run_offline_maps.sh 1

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

mkdir -p "$MAPDIR" "$LOGDIR"

die() { echo -e "\nFATAL: $*\n" >&2; exit 1; }

# ── preconditions ───────────────────────────────────────────────────────────

[[ -d "$BAG"      ]] || die "bag not found: $BAG"
[[ -f "$TEMPLATE" ]] || die "params template not found: $TEMPLATE"
[[ -f "$LAUNCH"   ]] || die "launch file not found: $LAUNCH"
[[ -f "$SAVER"    ]] || die "save_map.py not found: $SAVER"

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
  sed "s/__ROBOT__/$N/g" "$TEMPLATE" > "$PARAMS"
  grep -q "robot_$N/odom" "$PARAMS" || die "substitution failed in $PARAMS"

  rm -f "$OUT.pgm" "$OUT.yaml"

  echo "[1/4] starting slam_toolbox (log: ${LOG#$REPO_ROOT/})"
  ros2 launch "$LAUNCH" params_file:="$PARAMS" > "$LOG" 2>&1 &
  SLAM_PID=$!

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
  if [[ $ACTIVE -ne 1 ]]; then
    echo "  node never reached 'active'. Last state: ${STATE:-<none>}"
    echo "  tail of $LOG:"; tail -20 "$LOG"
    kill $SLAM_PID 2>/dev/null; wait $SLAM_PID 2>/dev/null
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
  echo "[3/4] replaying $RUN at rate $RATE, start-offset $START, playback-duration $DUR"

  # START_<N> is kept for completeness but /tf_static sits at bag time 0 and
  # --start-offset skips it; cut segments with strip_bag_for_offline_slam.py
  # --start/--duration instead, which re-timestamps /tf_static.
  ros2 bag play "$BAG" "${CLOCK_ARGS[@]}" --rate "$RATE" \
      --start-offset "$START" --playback-duration "$DUR" \
      --topics /clock /tf /tf_static "/robot_$N/scan" "/robot_$N/odom" \
      >> "$LOG" 2>&1

  echo "      settling ${SETTLE}s for the final map publish"
  sleep "$SETTLE"

  echo "[4/4] saving map"
  
  # slam_toolbox's correction. NOT in the saved .yaml, but required to place the
  # map in world coordinates: world = spawn o (map->odom)^-1 o map_origin o cell.
  # Omitting it cost a session of wrong on-wall figures (2026-09-05).
  timeout 5 ros2 run tf2_ros tf2_echo "robot_$N/map" "robot_$N/odom" \
      --ros-args -p use_sim_time:=true \
      > "$LOGDIR/map_to_odom_${RUN}_robot_$N.txt" 2>&1
  head -8 "$LOGDIR/map_to_odom_${RUN}_robot_$N.txt"
  
  
  
  python3 "$SAVER" --topic /map --out "$OUT" --timeout 30 >> "$LOG" 2>&1
  SAVE_RC=$?

  kill $SLAM_PID 2>/dev/null; wait $SLAM_PID 2>/dev/null
  pkill -f async_slam_toolbox_node 2>/dev/null

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
