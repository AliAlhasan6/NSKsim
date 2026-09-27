#!/usr/bin/env bash
# Batch driver for the offline known-pose maps of the five B2 runs.
#
# This automates experiments/runs/b2maps/README.md steps 3, 4 and 5 and NOTHING
# else. It does not record a run -- T1/T2/T3 are driven by hand, one run at a
# time -- it does not run run_health.py (step 2's gates), and it never attaches
# to a live stack. Every command below reads a bag that is already on disk.
#
# Per robot K and cut CUT, with RUN=b2maps_k$K and CUTRUN=${RUN}_cut${CUT}:
#
#   1  check_run_bag.py --min-sim CUT against the RAW bag. C1-C4 must all PASS,
#      and the --start/--duration it prints are the ONLY source for the strip --
#      nothing here recomputes them.
#   2  that same run's offset at --min-sim 1200 must be the same number.
#      --start is the offset of the first nonzero command, which does not move,
#      so per the README only --duration changes between cuts; a disagreement is
#      a fact about the bag rather than about the cut, and it stops the batch.
#   3  strip_bag_for_offline_slam.py -> run_offline_maps.sh (SCAN_MATCHING=false
#      FREE_SPACE_RELAY=true) -> fit_world_transform.py --run CUTRUN --bag the
#      RAW bag. Both env vars every time: with either one off, the online and
#      offline maps of one run answer different questions.
#   4  after each map, the dropped scans out of
#      experiments/logs/offline_slam_${CUTRUN}_robot_K.log -- GATED: at most one,
#      and only at the segment's first scan -- and a pgrep that must find no
#      free_space_relay and no slam_toolbox left behind.
#
# The run prints five stages rather than four, because 4 is run BETWEEN the map
# and the fit: a leaked relay or slam_toolbox has to stop the batch before the
# next cut replays onto the same /map, and the fit is a numpy job that cannot
# leave either behind.
#
# Usage -- both lists are required, deliberately. There is no default robot set
# and no default cut set: a bare invocation would otherwise be fifteen maps and
# several hours of replay.
#
#   source /opt/ros/jazzy/setup.bash
#   source ~/Desktop/NSKsim/install/setup.bash        # the repo-root install
#   bash experiments/slam/run_b2maps_cuts.sh --robots 0,1,2,3,4 --cuts 60,120,240
#   bash experiments/slam/run_b2maps_cuts.sh --robots 0,1 --cuts 1200 --force
#
# SOURCING. Run it from a shell that has sourced /opt/ros/jazzy and the
# REPO-ROOT install -- not ros2_ws/install, which is a build-time tree with no
# truth_odom_tf in it (README, "Once, before k0"). The preflight below prints
# the nsk_swarm prefix it resolved so the log records which tree produced the
# maps, and it aborts early if free_space_relay is not in that tree, rather than
# after a strip has already run.
#
# STOPS AT THE FIRST FAILURE, on purpose. Every cut replays through the same
# /map and the same node names, so one bag that does not check out or one leaked
# slam_toolbox makes every map after it unreadable.
#
# SKIPPING AND OVERWRITING. A cut whose map already exists is skipped unless
# --force. With --force the existing .pgm/.yaml and the newest matching
# world_fit JSON are copied into experiments/maps/superseded/ BEFORE anything is
# removed, and old vs new occupied/free/unknown is reported. The .pgm/.yaml
# copies carry this batch's timestamp so a second --force cannot overwrite what
# the first one preserved. The stripped ${CUTRUN}_slamin bag is NOT preserved:
# it is an intermediate that this script rebuilds from the raw bag on the same
# --start/--duration, so the copy would be byte-for-byte what step 3 writes
# again. Under --force an existing one is deleted, and only after checking it
# really is a rosbag2 directory.
#
# TEE. The README bans `tee` for T1-T3 because a terminal pumping tens of
# thousands of lines competes for exactly the time a live run measures. That
# reasoning does not reach here: these four commands print a few hundred lines
# per cut in total, and there is no live stack whose /clock can collapse. So
# everything lands in experiments/logs/b2maps/batch_<timestamp>.log as well as
# in the per-step logs, which keep the names the manual runs already used
# (${CUTRUN}_check.log, _strip.log, _offline.log, _fit.log).
#
# FOREGROUND. Run it in the foreground if you want Ctrl-C to work. `bash
# run_b2maps_cuts.sh &` makes the INT trap below silently nonexistent -- bash
# sets SIGINT to SIG_IGN in any command started with '&' when job control is
# off, and an ignored-on-entry signal cannot be re-enabled from inside.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Checked, because every path below is relative to it: a cd that failed would
# leave them resolving against whatever directory this was started from, and the
# preflight would then report missing scripts rather than the real fault.
cd "$REPO_ROOT" || { echo "FATAL: cannot cd to $REPO_ROOT" >&2; exit 1; }

# --spawn-rev is not a flag here. The script's own default is fc050c4, the b16
# era, and on a b18/b2maps bag it places every map 3.15 m out without saying so
# (README, step 5). There is one right answer for these five runs.
SPAWN_REV=08617b2

# Every B2 map is built with run_offline_maps.sh's deterministic variant, and
# that is not a flag here either. With the default gates one stripped segment
# replayed twice is not one map: measured on b2maps_k4_cut60, 2 of 7 replays of
# ONE segment produced 469 occupied cells instead of 353, with 411 cells (3.06 %
# of everything either map knew) in different classes. Under the variant all 10
# replays across the three cuts came out byte-identical.
#
# It is fixed rather than optional because a corpus is only comparable if every
# map in it was built the same way -- a cut built with the gates on cannot be
# read against one built with them off, and the difference (4.1 % of known cells
# at 60 s, 0.22 % at 1200 s) is larger than most of what these cuts are meant to
# show. run_offline_maps.sh's own default stays OFF, so old bags reproduce
# byte-identically; this is the B2 corpus's choice, not a change to that script.
DET_MODE=true

# Step 2's reference. 1200 s is the recording rule these runs were taken under,
# and it is the cut whose --start every other cut has to agree with.
REF_MIN_SIM=1200

LOGS=experiments/logs/b2maps          # bags, per-run logs, this batch log
SLAMLOGS=experiments/logs             # run_offline_maps.sh's per-robot slam log
MAPDIR=experiments/maps
SUPERSEDED=$MAPDIR/superseded

CHECK=experiments/slam/check_run_bag.py
STRIP=experiments/slam/strip_bag_for_offline_slam.py
OFFLINE=experiments/slam/run_offline_maps.sh
FIT=experiments/slam/fit_world_transform.py
SCAN_FACTS=experiments/slam/bag_scan_facts.py
VENV_PY=venv/bin/python

# The two processes run_offline_maps.sh starts and must not leave behind.
# Bracketed so pgrep cannot match the pgrep that is asking: pgrep -f reads full
# command lines, and the unbracketed spelling always finds itself.
LEFTOVER_RE='[f]ree_space_relay|[s]lam_toolbox'

usage() {
  cat <<'EOF'
usage: bash experiments/slam/run_b2maps_cuts.sh --robots K[,K...] --cuts S[,S...] [--force]

  --robots  explorer ids, 0..4, comma-separated. One raw bag per id:
            experiments/logs/b2maps/b2maps_k<K>
  --cuts    sim seconds per cut, comma-separated (the README's cuts are
            60, 120, 240 and 1200)
  --force   re-map cuts whose map already exists, preserving the old map and
            fit under experiments/maps/superseded/

Both lists are required; there is no default.
EOF
}

die() { echo -e "\nFATAL: $*\n" >&2; exit 1; }

# ── arguments ────────────────────────────────────────────────────────────────

ROBOTS_CSV=""
CUTS_CSV=""
FORCE=0

# Kept before the loop consumes them: the tee re-run below re-invokes this
# script with its own arguments, and by then "$@" has been shifted away.
ORIG_ARGS=("$@")

while (( $# )); do
  case "$1" in
    --robots)   ROBOTS_CSV="${2:-}"; shift 2 || die "--robots needs a value" ;;
    --robots=*) ROBOTS_CSV="${1#*=}"; shift ;;
    --cuts)     CUTS_CSV="${2:-}";   shift 2 || die "--cuts needs a value" ;;
    --cuts=*)   CUTS_CSV="${1#*=}"; shift ;;
    --force)    FORCE=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *)          usage >&2; die "unknown argument: $1" ;;
  esac
done

[[ -n "$ROBOTS_CSV" ]] || { usage >&2; die "--robots is required"; }
[[ -n "$CUTS_CSV"   ]] || { usage >&2; die "--cuts is required"; }

IFS=, read -r -a ROBOTS <<< "$ROBOTS_CSV"
IFS=, read -r -a CUTS   <<< "$CUTS_CSV"

# Duplicates are rejected rather than deduplicated: the second pass over one
# (robot, cut) would find the map its own first pass had just written and report
# it as "already exists", which reads as a fact about the repo and is not one.
dup_check() {
  local label="$1"; shift
  local -A seen=()
  local v
  for v in "$@"; do
    [[ -z "${seen[$v]:-}" ]] || die "$label lists $v twice"
    seen[$v]=1
  done
}

for K in "${ROBOTS[@]}"; do
  [[ "$K" =~ ^[0-4]$ ]] || die "--robots: '$K' is not a robot id 0..4"
done
for C in "${CUTS[@]}"; do
  [[ "$C" =~ ^[1-9][0-9]*$ ]] || die "--cuts: '$C' is not a positive integer of sim seconds"
done
dup_check --robots "${ROBOTS[@]}"
dup_check --cuts "${CUTS[@]}"

# ── tee everything, once ─────────────────────────────────────────────────────
# Re-run self through a pipe rather than exec'ing into a process substitution:
# the parent then waits on the pipeline, so tee has flushed before this script's
# status is reported, and ${PIPESTATUS[0]} keeps that status intact. The
# timestamp is chosen HERE and handed down, so both halves name one log file.
if [[ -z "${B2MAPS_BATCH_LOG:-}" ]]; then
  mkdir -p "$LOGS" || die "cannot create $LOGS"
  B2MAPS_BATCH_LOG="$LOGS/batch_$(date -u +%Y%m%dT%H%M%SZ).log" || \
    die 'date failed, so this batch has no log name'
  export B2MAPS_BATCH_LOG
  bash "$0" "${ORIG_ARGS[@]}" 2>&1 | tee -a "$B2MAPS_BATCH_LOG"
  exit "${PIPESTATUS[0]}"
fi
BATCH_LOG="$B2MAPS_BATCH_LOG"
STAMP="$(basename "$BATCH_LOG" .log)"; STAMP="${STAMP#batch_}"

# ── the summary table ────────────────────────────────────────────────────────
# Accumulated as rows are finished, printed once at the end -- including on the
# failure path, which is where it is worth the most.

ROW_FMT='  %-12s %5s  %-11s %-8s %9s %9s %9s %7s %7s %6s %10s\n'
ROWS=()

R_RUN=""; R_CUT=""
row_reset() { R_OCC='-'; R_FREE='-'; R_UNK='-'; R_FITA='-'; R_FITB='-'
              R_DROPS='-'; R_LEFT='-'; R_VARIANT='-'; }
row_reset

emit_row() {
  [[ -n "$R_RUN" ]] || return 0
  ROWS+=("$(printf "$ROW_FMT" "$R_RUN" "$R_CUT" "$1" "$R_VARIANT" "$R_OCC" \
                   "$R_FREE" "$R_UNK" "$R_FITA" "$R_FITB" "$R_DROPS" "$R_LEFT")")
}

# Which settings built a map that is already on disk, read off the params file
# run_offline_maps.sh keeps for every run. This is how a corpus half-built under
# the old gates is spotted: a skipped row reading 'base' is a map that is not
# reproducible and has to be re-made with --force.
map_variant() {          # $1 = CUTRUN, $2 = robot id
  local p="$SLAMLOGS/offline_mapping_$1_robot_$2.yaml" d h t q
  [[ -f "$p" ]] || { echo '?'; return 0; }
  d="$(sed -nE 's/^ *minimum_travel_distance: *([0-9.]+).*/\1/p' "$p" | tail -1)"
  h="$(sed -nE 's/^ *minimum_travel_heading: *([0-9.]+).*/\1/p' "$p" | tail -1)"
  t="$(sed -nE 's/^ *minimum_time_interval: *([0-9.]+).*/\1/p' "$p" | tail -1)"
  q="$(sed -nE 's/^ *scan_queue_size: *([0-9]+).*/\1/p' "$p" | tail -1)"
  # Empty means the file predates these knobs being recorded, which is not the
  # same as zero -- awk would read an empty string as 0 and call it 'det'.
  [[ -n "$d" && -n "$h" && -n "$t" ]] || { echo '?'; return 0; }
  if ! awk -v a="$d" -v b="$h" -v c="$t" \
           'BEGIN { exit !(a == 0 && b == 0 && c == 0) }'; then
    echo base
    return 0
  fi
  # Zeroed gates are not the whole variant. An ABSENT scan_queue_size leaves
  # slam_toolbox on its default of 1, which drops any scan still waiting for a
  # transform when the next one arrives -- so the map integrated whatever
  # survived that race rather than the segment, and it is not reproducible even
  # though every gate reads 0. b2maps_k0_cut1200 is exactly that map.
  if [[ -z "$q" ]]; then
    echo det/q1
  else
    echo det
  fi
}

print_table() {
  echo
  echo "════════════════════════════════════════════════════════════════════════════════════════════════"
  printf "$ROW_FMT" run cut status variant occupied free unknown 'fitA%' 'fitB%' \
         drops leftovers
  local row
  for row in "${ROWS[@]}"; do printf '%s\n' "$row"; done
  echo "════════════════════════════════════════════════════════════════════════════════════════════════"
  echo "  variant: det = built with DETERMINISTIC=true (replays byte-identical)."
  echo "           det/q1 = gates zeroed but no scan_queue_size, so slam_toolbox's"
  echo "           default of 1 was in force and mid-replay scans COULD be dropped."
  echo "           Read it with the drops column: 0 drops means nothing was lost"
  echo "           after all; any drop past the first scan means re-make it."
  echo "           base = the old gates, NOT reproducible -- re-make it with --force."
  echo "           '?' = no params file, so what built it is not recorded."
  echo "  occupied/free/unknown: cells of ${MAPDIR}/<run>_cut<cut>_robot<K>.pgm"
  echo "  fitA%/fitB%: fit_world_transform.py free fit, conventions A and B"
  echo "  drops: 'Message Filter dropping message' in the offline slam log."
  echo "         GATED for a map this batch built: at most one, and only at the"
  echo "         segment's first scan, which has no transform behind it yet."
  echo "         Any other drop is a scan missing from the map and fails the row."
  echo "         A skipped row's count is read off the old log and NOT gated --"
  echo "         this batch did not measure it."
  echo "  leftovers: free_space_relay / slam_toolbox processes still alive"
  echo "  batch log: $BATCH_LOG"
}

# Failure inside the loop: the row is closed as FAILED so the table shows where
# the batch stopped, and the reason is repeated after it.
fail() {
  echo -e "\nFAILED: $*" >&2
  emit_row FAILED
  print_table
  echo
  echo "stopped at the first failure: $*"
  exit 1
}

trap 'echo -e "\ninterrupted (SIGINT)" >&2; emit_row INTERRUPTED; print_table; exit 130' INT
trap 'echo -e "\nterminated (SIGTERM)" >&2; emit_row TERMINATED; print_table; exit 143' TERM

# ── helpers ──────────────────────────────────────────────────────────────────

# Run one step: to its own log AND to stdout, which the outer tee puts in the
# batch log. Returns the step's status, not tee's.
run_step() {
  local log="$1"; shift
  echo
  echo "  + $*"
  echo "    log: $log"
  "$@" 2>&1 | tee "$log"
  return "${PIPESTATUS[0]}"
}

# The drop gate. A dropped scan never reached Karto, so it is simply missing
# from the map -- and under DETERMINISTIC=true, where every scan that arrives is
# integrated, a drop is the only way the accepted set can still vary between two
# replays of one segment. So it is gated, not merely counted.
#
# EXACTLY ONE drop is allowed, and only at the segment's FIRST scan. That one is
# structural: the first scan has no transform behind it yet, so tf2's message
# filter discards it either for an empty transform cache ('the timestamp on the
# message is earlier than all the data in the transform cache', which
# b2maps_k4_cut1200 logs) or for the queue ('discarding message because the queue
# is full', which b2maps_k1_cut1200 and b2maps_k2_cut1200 log at 44.200 and
# 75.800 -- both of those ARE their segment's first scan). The reason does not
# matter; the position does.
#
# Any other drop fails the row. That is what b2maps_k0_cut1200 did on 2026-09-27:
# 6 drops at sim 773.0, 796.0, 806.0, 850.8, 1217.8 and 1272.8, with the
# segment's first scan at 106.000, every one of them 'queue is full' on a
# scan_queue_size that slam_toolbox defaults to 1. That map was accepted and
# tabulated as 'reported, not gated'.
#
# Compared NUMERICALLY with a 1 ms tolerance, not as strings: tf2 prints the
# dropped message's time as %.3f of a double and bag_scan_facts.py prints %.3f of
# its own reconstruction, and 1 ms is 1/200th of the 200 ms these bags space
# scans by -- loose enough to absorb any formatting difference, far too tight to
# mistake one scan for its neighbour.
DROP_REASON=""
check_drops() {           # $1 = CUTRUN, $2 = slam log, $3 = slamin bag, $4 = robot
  local cutrun="$1" slamlog="$2" slamin="$3" k="$4"
  local n first_scan times t
  n="$(grep -ac 'Message Filter dropping message' "$slamlog" || true)"
  n="${n:-0}"

  if (( n == 0 )); then
    DROP_REASON="no scan was dropped"
    return 0
  fi

  # Read the first scan stamp only when there IS a drop to place: it opens the
  # bag, and the common case has nothing to compare.
  local stamp_err
  stamp_err="$(mktemp)"
  first_scan="$(python3 "$SCAN_FACTS" "$slamin" "$k" --first-stamp 2>"$stamp_err")"
  if [[ ! "$first_scan" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    DROP_REASON="$n drop(s), and the segment's first scan stamp could not be
read out of $slamin to place them: $(cat "$stamp_err")"
    rm -f "$stamp_err"
    return 1
  fi
  rm -f "$stamp_err"

  # The 'at time <T>' of every drop, in log order.
  mapfile -t times < <(grep -a 'Message Filter dropping message' "$slamlog" \
                       | sed -nE "s/.* at time ([0-9]+\.[0-9]+) for reason .*/\1/p")
  if (( ${#times[@]} != n )); then
    DROP_REASON="$n drop line(s) but only ${#times[@]} carried a readable 'at
time' -- the log format is not what this gate reads"
    return 1
  fi

  local stray=()
  for t in "${times[@]}"; do
    awk -v a="$t" -v b="$first_scan" 'BEGIN { exit !((a - b < 0.001) && (b - a < 0.001)) }' \
      || stray+=("$t")
  done

  if (( ${#stray[@]} == 0 )) && (( n == 1 )); then
    DROP_REASON="1 drop, at the segment's first scan ($first_scan s) -- \
structural, allowed"
    return 0
  fi
  if (( ${#stray[@]} == 0 )); then
    # All of them at the first scan, but more than one of them. tf2 logs a given
    # message once, so this is not a shape the filter produces.
    DROP_REASON="$n drops all at the segment's first scan ($first_scan s), which
tf2 does not do -- at most one drop is allowed and only there"
    return 1
  fi
  DROP_REASON="$n drop(s), ${#stray[@]} of them NOT at the segment's first scan
($first_scan s): sim ${stray[*]}. A dropped scan never reached Karto, so it is
missing from this map, and which scans lose that race is a timing accident --
this map is not reproducible. Check scan_queue_size in
$SLAMLOGS/offline_mapping_${cutrun}_robot_${k}.yaml: unset means slam_toolbox's
default of 1, which drops any scan still waiting for a transform when the next
one arrives"
  return 1
}

# "occupied free unknown" of a nav2 trinary PGM, or nothing. One counter for the
# old map and the new one, so a --force comparison is not two implementations
# disagreeing. Same convention as run_offline_maps.sh's own verifier: 0 is
# occupied, 254 free, 205 unknown.
pgm_counts() {
  python3 - "$1" <<'PYEOF'
import sys
data = open(sys.argv[1], 'rb').read()
if data[:2] != b'P5':
    sys.exit(f'{sys.argv[1]}: not a binary PGM')
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
px = data[i:i + w * h]
print(px.count(0), px.count(254), px.count(205))
PYEOF
}

# ── preflight ────────────────────────────────────────────────────────────────

echo "════════════════════════════════════════════════════════════════════════════════════════════════"
echo "  b2maps offline cuts -- README steps 3-5, batched"
echo "════════════════════════════════════════════════════════════════════════════════════════════════"
echo "  started        $(date -u +%Y-%m-%dT%H:%M:%SZ) (UTC)"
echo "  robots         ${ROBOTS[*]}"
echo "  cuts           ${CUTS[*]} s of sim"
echo "  force          $((FORCE)) $( ((FORCE)) && echo '(existing maps re-made, old ones preserved)' || echo '(existing maps skipped)')"
echo "  spawn-rev      $SPAWN_REV (fixed; the fc050c4 default misplaces b2maps maps by 3.15 m)"
echo "  determinism    DETERMINISTIC=$DET_MODE (fixed) -- every scan integrated,"
echo "                 replay delayed; replays are byte-identical. A map built"
echo "                 without it is not reproducible: 2 of 7 base replays of"
echo "                 b2maps_k4_cut60 differed by 3.06 % of known cells."
echo "  batch log      $BATCH_LOG"
echo "  repo           $REPO_ROOT @ $(git rev-parse --short HEAD 2>/dev/null || echo '?')$(git diff --quiet 2>/dev/null || echo ' (dirty)')"

for f in "$CHECK" "$STRIP" "$OFFLINE" "$FIT" "$SCAN_FACTS"; do
  [[ -f "$f" ]] || die "missing script: $f"
done
[[ -x "$VENV_PY" ]] || die "$VENV_PY not found -- fit_world_transform.py runs in the venv"

command -v ros2 >/dev/null 2>&1 || \
  die "ros2 is not on PATH. Run: source /opt/ros/jazzy/setup.bash && source $REPO_ROOT/install/setup.bash"

# check_run_bag.py and strip_bag_for_offline_slam.py read bags, and
# fit_world_transform.py's --bag does too -- from inside the venv, which is the
# one flag of that script that needs the ROS environment. Checked here rather
# than discovered at the last step of a 25-minute cut.
python3 -c 'import rosbag2_py' 2>/dev/null || \
  die "python3 cannot import rosbag2_py -- source /opt/ros/jazzy/setup.bash first"
"$VENV_PY" -c 'import numpy, scipy, PIL, rosbag2_py' 2>/dev/null || \
  die "$VENV_PY cannot import one of numpy/scipy/PIL/rosbag2_py. The first three
are the venv's; rosbag2_py comes from the sourced ROS environment, which
fit_world_transform.py --bag needs."

# FREE_SPACE_RELAY=true is not optional here, so a tree without the relay is a
# preflight failure rather than a failure after the first strip.
ros2 pkg executables nsk_swarm 2>/dev/null | grep -q free_space_relay || \
  die "free_space_relay is not in the sourced nsk_swarm. Build from the repo root
(colcon build --packages-select nsk_swarm) and source install/setup.bash."
echo "  nsk_swarm      $(ros2 pkg prefix nsk_swarm 2>/dev/null || echo '?')"

for K in "${ROBOTS[@]}"; do
  [[ -d "$LOGS/b2maps_k$K" ]] || die "raw bag not found: $LOGS/b2maps_k$K"
  [[ -f "$LOGS/b2maps_k$K/metadata.yaml" ]] || \
    die "$LOGS/b2maps_k$K is not a rosbag2 directory (no metadata.yaml)"
done
echo "  bags           $(printf 'b2maps_k%s ' "${ROBOTS[@]}")-- all present"

LEFT="$(pgrep -af "$LEFTOVER_RE" || true)"
if [[ -n "$LEFT" ]]; then
  echo "$LEFT"
  die "free_space_relay or slam_toolbox is already running. These would
contaminate /map, and this script must not kill processes it did not start."
fi

BATCH_START=$SECONDS

# ── robot x cut ──────────────────────────────────────────────────────────────

for K in "${ROBOTS[@]}"; do
  RUN="b2maps_k$K"
  BAG="$LOGS/$RUN"

  # Step 2's reference, per robot: resolved on the first cut of this robot and
  # reused by the rest, since the bag does not change between cuts.
  REF_START=""
  REF_SRC=""

  for CUT in "${CUTS[@]}"; do
    CUTRUN="${RUN}_cut${CUT}"
    MAP="$MAPDIR/${CUTRUN}_robot$K"
    SLAMIN="$LOGS/${CUTRUN}_slamin"
    SLAMLOG="$SLAMLOGS/offline_slam_${CUTRUN}_robot_$K.log"
    CHECK_LOG="$LOGS/${CUTRUN}_check.log"
    STRIP_LOG="$LOGS/${CUTRUN}_strip.log"
    OFFLINE_LOG="$LOGS/${CUTRUN}_offline.log"
    FIT_LOG="$LOGS/${CUTRUN}_fit.log"
    CUT_START_SECONDS=$SECONDS

    R_RUN="$RUN"; R_CUT="$CUT"; row_reset

    echo
    echo "════════════════════════════════════════════════════════════════════════════════════════════════"
    echo "  robot_$K   cut ${CUT}s   ->  ${CUTRUN}_robot$K"
    echo "════════════════════════════════════════════════════════════════════════════════════════════════"

    # ── skip, or prepare to supersede ──
    SUPERSEDING=0
    OLD_COUNTS=""
    if [[ -f "$MAP.pgm" ]]; then
      if (( FORCE == 0 )); then
        echo "  map exists: $MAP.pgm"
        R_VARIANT="$(map_variant "$CUTRUN" "$K")"
        echo "  built with: $R_VARIANT settings"
        case "$R_VARIANT" in
          det) ;;
          det/q1)
            echo "  NOTE: DETERMINISTIC=true, but no scan_queue_size, so slam_toolbox's" \
                 "default of 1 was in force and any scan still waiting for a transform" \
                 "when the next one arrived COULD have been dropped. Read the drops" \
                 "count below: 0, or 1 at the segment's first scan, means nothing was" \
                 "lost and this map is what the segment contains. Anything else is a" \
                 "scan missing from it, and re-making it with --force is the fix." ;;
          *)
            echo "  NOTE: not built with DETERMINISTIC=true, so it is not reproducible" \
                 "and does not belong in the same corpus as the rest. Re-make it with --force." ;;
        esac
        OLD_COUNTS="$(pgm_counts "$MAP.pgm" || true)"
        if [[ -n "$OLD_COUNTS" ]]; then
          read -r R_OCC R_FREE R_UNK <<< "$OLD_COUNTS"
          echo "  existing grid: $R_OCC occupied, $R_FREE free, $R_UNK unknown"
        fi
        # The drops of the run that made it, if its log is still there. The
        # fit percentages are not read back: this script did not produce them
        # in this batch and will not put a number in the table it has not just
        # measured.
        if [[ -f "$SLAMLOG" ]]; then
          R_DROPS="$(grep -ac 'Message Filter dropping message' "$SLAMLOG" || true)"
          R_DROPS="${R_DROPS:-0}"
        fi
        echo "  SKIPPED -- pass --force to re-make it"
        emit_row skipped
        continue
      fi
      SUPERSEDING=1
      echo "  map exists and --force is set: it will be superseded"
    fi

    # ── step 1: C1-C4 at this cut, against the RAW bag ──
    echo
    echo "  [1/5] check_run_bag.py --min-sim $CUT"
    run_step "$CHECK_LOG" python3 "$CHECK" --bag "$BAG" --robot "$K" \
        --spawn-rev "$SPAWN_REV" --min-sim "$CUT"
    CHECK_RC=$?

    VERDICTS=""
    for c in C1 C2 C3 C4; do
      v="$(grep -a -E "^  $c  " "$CHECK_LOG" | tail -1 | sed -E "s/^  $c  //")"
      VERDICTS+="$c=${v:-<missing>} "
      [[ "$v" == "PASS" ]] || fail "$CUTRUN: $c is ${v:-<missing>}, not PASS ($CHECK_LOG)"
    done
    (( CHECK_RC == 0 )) || fail "$CUTRUN: check_run_bag.py exited $CHECK_RC ($CHECK_LOG)"
    echo "    $VERDICTS"

    # The cut's own numbers, taken from what it printed -- never recomputed.
    SEG="$(grep -a -m1 -E '^    --start ' "$CHECK_LOG")"
    [[ -n "$SEG" ]] || fail "$CUTRUN: C4 passed but printed no --start/--duration line"
    CUT_START="$(sed -E 's/.*--start +([0-9.]+).*/\1/' <<< "$SEG")"
    CUT_DUR="$(sed -E 's/.*--duration +([0-9.]+).*/\1/' <<< "$SEG")"
    [[ "$CUT_START" =~ ^[0-9]+\.[0-9]+$ && "$CUT_DUR" =~ ^[0-9]+\.[0-9]+$ ]] || \
      fail "$CUTRUN: could not read --start/--duration out of '$SEG'"
    echo "    --start $CUT_START --duration $CUT_DUR"

    # ── step 2: the same run's --start at --min-sim 1200 ──
    echo
    echo "  [2/5] consistency: --start must equal this run's offset at --min-sim $REF_MIN_SIM"
    if [[ -z "$REF_START" ]]; then
      if (( CUT == REF_MIN_SIM )); then
        # The cut IS the reference; re-running the same command to compare a
        # number with itself would prove nothing.
        REF_START="$CUT_START"
        REF_SRC="the same invocation ($CHECK_LOG)"
      else
        REF_CHECK_LOG="$LOGS/${RUN}_check_ref${REF_MIN_SIM}.log"
        # Its C4 verdict is NOT gated on. b2maps_relay1 holds 520 s and fails
        # at 1200 while still supplying 60/120/240 -- what is wanted here is the
        # offset of the first nonzero command, which C4 prints either way.
        run_step "$REF_CHECK_LOG" python3 "$CHECK" --bag "$BAG" --robot "$K" \
            --spawn-rev "$SPAWN_REV" --min-sim "$REF_MIN_SIM"
        REF_LINE="$(grep -a -m1 -E '^  first nonzero command at .* \(strip convention' \
                      "$REF_CHECK_LOG")"
        [[ -n "$REF_LINE" ]] || \
          fail "$RUN: --min-sim $REF_MIN_SIM printed no strip-convention offset, so the cut's --start cannot be corroborated ($REF_CHECK_LOG)"
        REF_START="$(sed -E 's/.*command at +([0-9.]+) s.*/\1/' <<< "$REF_LINE")"
        REF_SRC="$REF_CHECK_LOG"
        REF_C4="$(grep -a -E '^  C4  ' "$REF_CHECK_LOG" | tail -1 | sed -E 's/^  C4  //')"
        echo "    at --min-sim $REF_MIN_SIM: offset $REF_START, C4 ${REF_C4:-<missing>} (reported, not gated)"
      fi
    fi
    [[ "$CUT_START" == "$REF_START" ]] || \
      fail "$CUTRUN: --start $CUT_START but --min-sim $REF_MIN_SIM gives $REF_START. --start is the offset of the first nonzero command and must not move between cuts; something is wrong with the bag, not with the cut. Reference: $REF_SRC"
    echo "    $CUT_START == $REF_START  ok  (reference: $REF_SRC)"

    # ── supersede, now that the checks have passed ──
    if (( SUPERSEDING )); then
      echo
      echo "  --force: preserving the existing map and fit under $SUPERSEDED/"
      mkdir -p "$SUPERSEDED" || fail "cannot create $SUPERSEDED"
      OLD_COUNTS="$(pgm_counts "$MAP.pgm")" || \
        fail "$CUTRUN: cannot read the existing $MAP.pgm, so old vs new cannot be reported"
      for ext in pgm yaml; do
        [[ -f "$MAP.$ext" ]] || { echo "    no $MAP.$ext to preserve"; continue; }
        cp -p "$MAP.$ext" "$SUPERSEDED/${CUTRUN}_robot${K}_${STAMP}.$ext" || \
          fail "$CUTRUN: could not copy $MAP.$ext into $SUPERSEDED"
        echo "    $MAP.$ext -> $SUPERSEDED/${CUTRUN}_robot${K}_${STAMP}.$ext"
      done
      # The newest world_fit JSON for this cut: its name carries its own UTC
      # stamp, so sorting by name is sorting by time, and the newest is the one
      # that describes the map being replaced.
      OLD_FIT="$(ls -1 "$SLAMLOGS"/world_fit_"${CUTRUN}"_*.json 2>/dev/null | sort | tail -1)"
      if [[ -n "$OLD_FIT" ]]; then
        cp -p "$OLD_FIT" "$SUPERSEDED/$(basename "$OLD_FIT")" || \
          fail "$CUTRUN: could not copy $OLD_FIT into $SUPERSEDED"
        echo "    $OLD_FIT -> $SUPERSEDED/$(basename "$OLD_FIT")"
      else
        echo "    no world_fit_${CUTRUN}_*.json to preserve"
      fi
    fi

    # ── step 3: strip -> offline map ──
    echo
    echo "  [3/5] strip the RAW bag, then the offline known-pose map"
    # strip_bag_for_offline_slam.py refuses to overwrite, so an existing segment
    # has to go. It is an intermediate this script owns: rebuilt from the raw bag
    # on the same --start/--duration the check just printed, so the copy would be
    # what the strip below writes again. This is only reached when a map is about
    # to be written anyway -- an existing map without --force was skipped above --
    # which is also what makes a retry after a failed replay work without hand
    # cleanup. Deleted only after confirming it really is a rosbag2 directory.
    if [[ -e "$SLAMIN" ]]; then
      [[ -d "$SLAMIN" && -f "$SLAMIN/metadata.yaml" ]] || \
        fail "$SLAMIN exists and is not a rosbag2 directory -- refusing to delete it. Look at it and remove it by hand."
      echo "    removing the old stripped segment ($(du -sh "$SLAMIN" | cut -f1)); it is rebuilt below from the raw bag on the same --start/--duration"
      rm -rf "$SLAMIN" || fail "could not remove $SLAMIN"
    fi

    run_step "$STRIP_LOG" python3 "$STRIP" "$BAG" "$SLAMIN" \
        --start "$CUT_START" --duration "$CUT_DUR" \
      || fail "$CUTRUN: strip_bag_for_offline_slam.py failed ($STRIP_LOG)"
    [[ -f "$SLAMIN/metadata.yaml" ]] || fail "$CUTRUN: $SLAMIN is not a bag after the strip"

    # DETERMINISTIC passed explicitly rather than exported, so the command in
    # the log is the whole recipe for the map beside it.
    run_step "$OFFLINE_LOG" env BAG="$SLAMIN" RUN="$CUTRUN" \
        SCAN_MATCHING=false FREE_SPACE_RELAY=true DETERMINISTIC="$DET_MODE" \
        bash "$OFFLINE" "$K" \
      || fail "$CUTRUN: run_offline_maps.sh failed ($OFFLINE_LOG)"
    R_VARIANT="$(map_variant "$CUTRUN" "$K")"
    [[ "$R_VARIANT" == "det" ]] || \
      fail "$CUTRUN: built with DETERMINISTIC=$DET_MODE but its params file reads '$R_VARIANT', not 'det'"
    [[ -s "$MAP.pgm" && -s "$MAP.yaml" ]] || \
      fail "$CUTRUN: run_offline_maps.sh exited 0 but $MAP.pgm/.yaml is missing or empty"

    read -r R_OCC R_FREE R_UNK <<< "$(pgm_counts "$MAP.pgm")" || \
      fail "$CUTRUN: cannot read the new $MAP.pgm"
    echo
    echo "    new grid: $R_OCC occupied, $R_FREE free, $R_UNK unknown"
    if (( SUPERSEDING )); then
      read -r O_OCC O_FREE O_UNK <<< "$OLD_COUNTS"
      echo "    old grid: $O_OCC occupied, $O_FREE free, $O_UNK unknown  (superseded)"
      printf '    delta   : %+d occupied, %+d free, %+d unknown\n' \
             "$((R_OCC - O_OCC))" "$((R_FREE - O_FREE))" "$((R_UNK - O_UNK))"
    fi

    # ── step 4: the two checks that belong to the map ──
    # Before the fit: a leaked relay or slam_toolbox would contaminate the next
    # cut's /map, and the fit is a numpy job that cannot leave either behind.
    echo
    echo "  [4/5] dropped scans, and leftovers"
    [[ -f "$SLAMLOG" ]] || fail "$CUTRUN: no offline slam log at $SLAMLOG"
    R_DROPS="$(grep -ac 'Message Filter dropping message' "$SLAMLOG" || true)"
    R_DROPS="${R_DROPS:-0}"
    echo "    dropped scans: $R_DROPS  ($SLAMLOG)"
    if (( R_DROPS )); then
      grep -a 'Message Filter dropping message' "$SLAMLOG" | sed 's/^/      /'
    fi
    check_drops "$CUTRUN" "$SLAMLOG" "$SLAMIN" "$K" || \
      fail "$CUTRUN: $DROP_REASON ($SLAMLOG)"
    echo "    gate: $DROP_REASON"

    # Retried: stop_pids SIGKILLs after 10 s and reaps, but a process can still
    # be on its way out when this looks. A single sample would make that a
    # false leftover.
    LEFT=""
    for _ in 1 2 3 4 5; do
      LEFT="$(pgrep -af "$LEFTOVER_RE" || true)"
      [[ -n "$LEFT" ]] || break
      sleep 2
    done
    if [[ -n "$LEFT" ]]; then
      R_LEFT="$(wc -l <<< "$LEFT")"
      echo "$LEFT" | sed 's/^/      /'
      fail "$CUTRUN: $R_LEFT free_space_relay/slam_toolbox process(es) left behind. Stop them before mapping anything else -- they publish onto /map."
    fi
    R_LEFT=0
    echo "    leftovers: none (free_space_relay, slam_toolbox)"

    # ── step 5: place the map in the world ──
    # --run is the cut, --bag is the RAW bag: it is what corroborates the spawn
    # table against the robots that never moved, and there is no bag under the
    # cut's name.
    echo
    echo "  [5/5] fit_world_transform.py --run $CUTRUN"
    run_step "$FIT_LOG" "$VENV_PY" "$FIT" --run "$CUTRUN" --robot "$K" \
        --spawn-rev "$SPAWN_REV" --bag "$BAG" \
      || fail "$CUTRUN: fit_world_transform.py failed ($FIT_LOG)"

    for conv in A B; do
      pct="$(grep -a -m1 -E "^  convention $conv: free fit" "$FIT_LOG" \
             | sed -E 's/.*free fit +([0-9.]+)%.*/\1/')"
      [[ -n "$pct" ]] || fail "$CUTRUN: no 'convention $conv: free fit' line in $FIT_LOG"
      if [[ "$conv" == A ]]; then R_FITA="$pct"; else R_FITB="$pct"; fi
    done
    echo
    echo "    free fit: A $R_FITA%  B $R_FITB%"
    grep -a -m1 -E '^(RESOLVED|UNRESOLVED|AMBIGUOUS|NO VERDICT)' "$FIT_LOG" \
      | sed 's/^/    verdict: /'

    emit_row "$( ((SUPERSEDING)) && echo superseded || echo mapped )"
    printf '  done in %d min %d s\n' \
           $(( (SECONDS - CUT_START_SECONDS) / 60 )) $(( (SECONDS - CUT_START_SECONDS) % 60 ))
  done
done

R_RUN=""          # nothing half-finished to close
print_table
printf '\n  %d cut(s) over %d robot(s) in %d min %d s\n' \
       "${#ROWS[@]}" "${#ROBOTS[@]}" $(( (SECONDS - BATCH_START) / 60 )) \
       $(( (SECONDS - BATCH_START) % 60 ))
exit 0
