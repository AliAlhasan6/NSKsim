#!/usr/bin/env bash
# preflight_stub_check.sh -- record_run.sh's pre-flight, under a stub `ros2`.
#
# What it protects. record_run.sh retries its pre-flight because `ros2 topic
# list` is a discovery snapshot rather than a reading of the graph: rehearsal 4
# aborted on one that called 13 of 25 topics missing, including
# /model/robot_0/pose, which truth_odom_tf was receiving at that moment. The
# retry must fix exactly that and change nothing else -- above all it must not
# change WHAT GETS RECORDED, because that is the content of every run bag.
#
# So this runs the current record_run.sh and a baseline revision of it side by
# side under a stub `ros2` (prints a topic list, echoes what `bag record` was
# asked for) and a stub `sleep` (returns instantly, logs the interval), and
# checks four scenarios:
#
#   A  swarm mode, everything advertised    same 30 topics, both exit 0
#   B  EXPLORER=0, everything advertised    same 28 topics, both exit 0
#   C  a topic genuinely never comes up     both abort, nothing recorded
#   D  rehearsal 4's race: missing, then    baseline ABORTS, current records,
#      missing, then complete               and records what B records
#
# D is the change; A, B and C are what it must not disturb.
#
# Needs bash, git and grep. No ROS, no Gazebo, no bag: every scenario is a
# few milliseconds, and nothing is written outside a temp directory.
#
# Usage:
#     experiments/slam/preflight_stub_check.sh [BASELINE_REV]
#
# BASELINE_REV defaults to 1f213c8, the EXPLORER-mode commit -- the state of
# this script's pre-flight immediately before the retry was added. Exit status
# is 0 when all four scenarios behave as required, 1 otherwise.
set -uo pipefail

BASELINE_REV="${1:-1f213c8}"
REPO="$(git rev-parse --show-toplevel)" || {
  echo "not inside a git repository" >&2; exit 1; }
CURRENT="$REPO/experiments/slam/record_run.sh"
[[ -x "$CURRENT" ]] || { echo "not executable: $CURRENT" >&2; exit 1; }

W="$(mktemp -d)"
trap 'rm -rf "$W"' EXIT
mkdir -p "$W/bin"

git -C "$REPO" show "$BASELINE_REV:experiments/slam/record_run.sh" \
    > "$W/baseline.sh" || {
  echo "cannot read record_run.sh at $BASELINE_REV" >&2; exit 1; }
chmod +x "$W/baseline.sh"

# ── the stubs ────────────────────────────────────────────────────────────────
# `ros2 topic list` prints $TOPICS_FILE, or, when $TOPICS_DIR is set, the file
# named for the attempt number -- which is how a discovery race is expressed.
# `ros2 bag record` never records: it prints what it was asked to record, one
# token per line, which is the thing being compared.
cat > "$W/bin/ros2" <<'STUB'
#!/usr/bin/env bash
case "$1 $2" in
  "topic list")
    if [[ -n "${TOPICS_DIR:-}" ]]; then
      n=$(( $(cat "$ATTEMPT_FILE") + 1 )); echo "$n" > "$ATTEMPT_FILE"
      f="$TOPICS_DIR/attempt$n"; [[ -f "$f" ]] || f="$TOPICS_DIR/final"
      cat "$f"
    else
      cat "$TOPICS_FILE"
    fi
    ;;
  "bag record") shift 2; echo RECORD; printf '%s\n' "$@" ;;
  *) echo "stub ros2: unexpected '$*'" >&2; exit 99 ;;
esac
STUB
cat > "$W/bin/sleep" <<'STUB'
#!/usr/bin/env bash
echo "$1" >> "$SLEEP_LOG"
STUB
chmod +x "$W/bin/ros2" "$W/bin/sleep"
export PATH="$W/bin:$PATH"

# ── the topic universe ───────────────────────────────────────────────────────
{ printf '%s\n' /clock /tf /tf_static /kg_share /nsk/convergence
  for n in 0 1 2 3 4; do
    printf '/robot_%s/odom\n/robot_%s/scan\n/robot_%s/map\n' $n $n $n
    printf '/robot_%s/map_metadata\n/model/robot_%s/pose\n' $n $n
  done; } > "$W/all_swarm"

{ printf '%s\n' /clock /tf /tf_static /kg_share /nsk/convergence
  for n in 0 1 2 3 4; do
    printf '/robot_%s/odom\n/robot_%s/scan\n' $n $n
    printf '/model/robot_%s/pose\n/robot_%s/joint_states\n' $n $n
  done; } > "$W/all_explorer"

# Rehearsal 4's snapshot, verbatim: the 25 required minus the 13 it reported.
printf '%s\n' \
  /model/robot_0/pose /robot_0/joint_states /robot_1/scan /model/robot_1/pose \
  /robot_1/joint_states /robot_2/scan /model/robot_2/pose \
  /robot_2/joint_states /robot_3/scan /model/robot_3/pose /robot_4/scan \
  /model/robot_4/pose /robot_4/joint_states > "$W/rehearsal4_missing"
grep -vxF -f "$W/rehearsal4_missing" "$W/all_explorer" > "$W/rehearsal4_snap"

grep -vxF -e /robot_3/scan "$W/all_explorer" > "$W/permanently_short"

# ── running one version through one scenario ─────────────────────────────────
run() {  # $1 tag, $2 script, $3 EXPLORER ('' for swarm mode)
  local tag="$1" script="$2" explorer="$3"
  export SLEEP_LOG="$W/$tag.sleeps"; : > "$SLEEP_LOG"
  [[ -n "${TOPICS_DIR:-}" ]] && echo 0 > "$ATTEMPT_FILE"
  if [[ -n "$explorer" ]]; then
    ( cd "$REPO" && EXPLORER="$explorer" RUN=stubcheck "$script" ) \
      > "$W/$tag.out" 2> "$W/$tag.err"
  else
    ( cd "$REPO" && RUN=stubcheck "$script" ) > "$W/$tag.out" 2> "$W/$tag.err"
  fi
  echo $? > "$W/$tag.rc"
  # Everything from the RECORD marker on is what the bag would contain.
  sed -n '/^RECORD$/,$p' "$W/$tag.out" > "$W/$tag.rec"
}

FAILURES=0
check() {  # $1 description, $2 actual, $3 expected
  if [[ "$2" == "$3" ]]; then
    printf '    ok   %s\n' "$1"
  else
    printf '    FAIL %s\n         got      %s\n         expected %s\n' \
           "$1" "$2" "$3"
    FAILURES=$((FAILURES + 1))
  fi
}

topics_recorded() { grep -c '^/' "$W/$1.rec"; }

# ── A: swarm mode, everything advertised ─────────────────────────────────────
unset TOPICS_DIR; export TOPICS_FILE="$W/all_swarm"
echo "A  swarm mode, all 30 topics advertised"
run A_base "$W/baseline.sh" ''
run A_cur "$CURRENT" ''
check "baseline exits 0" "$(cat "$W/A_base.rc")" 0
check "current exits 0" "$(cat "$W/A_cur.rc")" 0
check "records 30 topics" "$(topics_recorded A_cur)" 30
check "records exactly what the baseline records" \
      "$(diff -q "$W/A_base.rec" "$W/A_cur.rec" >/dev/null && echo same)" same
check "does not sleep" "$(wc -l < "$W/A_cur.sleeps")" 0

# ── B: explorer mode, everything advertised ──────────────────────────────────
export TOPICS_FILE="$W/all_explorer"
echo "B  EXPLORER=0, all 25 required topics advertised"
run B_base "$W/baseline.sh" 0
run B_cur "$CURRENT" 0
check "baseline exits 0" "$(cat "$W/B_base.rc")" 0
check "current exits 0" "$(cat "$W/B_cur.rc")" 0
check "records 28 topics" "$(topics_recorded B_cur)" 28
check "records exactly what the baseline records" \
      "$(diff -q "$W/B_base.rec" "$W/B_cur.rec" >/dev/null && echo same)" same
check "reports the attempt it passed on" \
      "$(grep -c 'on attempt 1 of 5' "$W/B_cur.out")" 1

# ── C: a topic that genuinely never comes up ─────────────────────────────────
export TOPICS_FILE="$W/permanently_short"
echo "C  EXPLORER=0, /robot_3/scan never advertised"
run C_base "$W/baseline.sh" 0
run C_cur "$CURRENT" 0
check "baseline aborts" "$(cat "$W/C_base.rc")" 1
check "current aborts" "$(cat "$W/C_cur.rc")" 1
check "records nothing" "$(topics_recorded C_cur)" 0
check "tries 5 times" "$(grep -c 'pre-flight attempt' "$W/C_cur.err")" 4
check "waits 3 s between tries" "$(sort -u "$W/C_cur.sleeps" | tr -d '\n')" 3
check "names the topic in the abort" \
      "$(grep -c '^  /robot_3/scan$' "$W/C_cur.err")" 1

# ── D: rehearsal 4's discovery race ──────────────────────────────────────────
unset TOPICS_FILE
mkdir -p "$W/flaky"
cp "$W/rehearsal4_snap" "$W/flaky/attempt1"
cp "$W/rehearsal4_snap" "$W/flaky/attempt2"
cp "$W/all_explorer" "$W/flaky/final"
export TOPICS_DIR="$W/flaky" ATTEMPT_FILE="$W/attempt"
echo "D  EXPLORER=0, rehearsal 4's race: 13 missing, 13 missing, then complete"
run D_base "$W/baseline.sh" 0
run D_cur "$CURRENT" 0
check "baseline aborts on the first snapshot" "$(cat "$W/D_base.rc")" 1
check "baseline abort names 13 topics" \
      "$(grep -c 'ABORT: 13 of 25' "$W/D_base.err")" 1
check "current proceeds" "$(cat "$W/D_cur.rc")" 0
check "current passes on attempt 3" \
      "$(grep -c 'on attempt 3 of 5' "$W/D_cur.out")" 1
check "re-checks only what was missing" \
      "$(grep -c '13 of 25 topic(s) not yet advertised' "$W/D_cur.err")" 2
check "records what it records when nothing was ever missing" \
      "$(diff -q "$W/B_cur.rec" "$W/D_cur.rec" >/dev/null && echo same)" same

echo
if (( FAILURES )); then
  echo "FAIL: $FAILURES check(s) failed (baseline $BASELINE_REV)"
  exit 1
fi
echo "PASS: pre-flight retries the race and changes nothing else " \
     "(baseline $BASELINE_REV)"
