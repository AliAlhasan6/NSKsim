#!/usr/bin/env bash
# Re-fit the existing B2 cut maps into the world. FITS ONLY -- no SLAM.
#
# This is step 5 of experiments/slam/run_b2maps_cuts.sh and NOTHING else. It
# does not check a bag, does not strip one, does not replay anything into
# slam_toolbox and never writes a map. Every .pgm it reads is already on disk;
# the only thing it produces is a new world_fit JSON + PNG per map (new stamped
# names, so nothing is overwritten) and this batch's log.
#
# WHY IT EXISTS. fit_world_transform.py used to gate each analytic candidate
# against the free optimum of its OWN row convention, so a mirrored (B) reading
# was only ever asked "are you the best mirrored placement of this map". Three
# of the twenty cuts came out AMBIGUOUS that way -- b2maps_k1_cut1200,
# b2maps_k4_cut240 and b2maps_k4_cut1200, each a B candidate sitting ~19 pp
# under the A reading of the same map and passing on a few tenths of its own.
# The gate now reads the best free fit ACROSS conventions, and the whole corpus
# has to be re-read under it. The maps do not change, so there is nothing to
# re-map: only the verdict over them changes.
#
# Usage -- both lists are OPTIONAL here, unlike run_b2maps_cuts.sh:
#
#   source /opt/ros/jazzy/setup.bash
#   source ~/Desktop/NSKsim/install/setup.bash        # the repo-root install
#   bash experiments/slam/refit_b2maps_cuts.sh
#   bash experiments/slam/refit_b2maps_cuts.sh --robots 4 --cuts 240,1200
#
# The batch driver makes them mandatory because "a bare invocation would
# otherwise be fifteen maps and several hours of replay". That reasoning does
# not reach here: a bare invocation is the twenty maps of the corpus, ~14
# minutes of numpy, and it replays nothing and writes no map. So the corpus IS
# the default.
#
# WALL TIME: about 14 minutes for all twenty, single-threaded. Per map it runs
# from ~25 s for a 60 s cut to ~66 s for a 1200 s cut -- measured off the
# per-cut fit logs of the 2026-09-27 batches. The cost is the 360-theta FFT
# sweep over the map's occupied cells plus the fixed robot_4 self-check every
# invocation pays, so it scales with the map, not with the cut's duration.
#
# SOURCING. --bag is the one flag of fit_world_transform.py that does not run
# from a bare venv: it opens the RAW bag to corroborate the spawn table against
# the robots that never moved. So this needs /opt/ros/jazzy and the REPO-ROOT
# install sourced, exactly as the batch driver does, and the preflight checks
# that before the first fit rather than after thirteen minutes.
#
# DOES NOT STOP AT THE FIRST FAILURE, unlike run_b2maps_cuts.sh. That script
# must: every cut replays through the same /map and the same node names, so one
# bad map makes every map after it unreadable. Nothing here shares state --
# each fit is an independent numpy job reading files it never writes -- so a
# failed map is marked FAILED, the loop carries on, and the script exits
# nonzero at the end. One bad map must not hide the nineteen good ones.
#
# FOREGROUND. Run it in the foreground if you want Ctrl-C to work. `bash
# refit_b2maps_cuts.sh &` makes the INT trap below silently nonexistent -- bash
# sets SIGINT to SIG_IGN in any command started with '&' when job control is
# off, and an ignored-on-entry signal cannot be re-enabled from inside.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT" || { echo "FATAL: cannot cd to $REPO_ROOT" >&2; exit 1; }

# Not a flag, for the same reason it is not one in run_b2maps_cuts.sh: the
# script's own default is fc050c4, the b16 era, and on a b2maps bag it places
# every map 3.15 m out without saying so. There is one right answer for these
# five runs.
SPAWN_REV=08617b2

# The corpus: five explorers x four cuts. The _q1/_q2 maps of b2maps_k0 are
# deliberately absent -- they are the scan_queue_size diagnostics, not part of
# the corpus, and run_b2maps_cuts.sh never built them as cuts either.
DEFAULT_ROBOTS="0,1,2,3,4"
DEFAULT_CUTS="60,120,240,1200"

LOGS=experiments/logs/b2maps          # raw bags, per-run logs, this batch log
SLAMLOGS=experiments/logs             # where the fit writes its JSON and PNG
MAPDIR=experiments/maps

FIT=experiments/slam/fit_world_transform.py
VENV_PY=venv/bin/python

usage() {
  cat <<EOF
usage: bash experiments/slam/refit_b2maps_cuts.sh [--robots K[,K...]] [--cuts S[,S...]]

  --robots  explorer ids, 0..4          (default $DEFAULT_ROBOTS)
  --cuts    sim seconds per cut         (default $DEFAULT_CUTS)

Re-fits maps that already exist. Runs no SLAM and writes no map. About 14 min
for the full corpus of twenty; needs /opt/ros/jazzy and the repo-root install
sourced, because the fit's --bag flag reads the raw bag.
EOF
}

die() { echo -e "\nFATAL: $*\n" >&2; exit 1; }

# ── arguments ────────────────────────────────────────────────────────────────

ROBOTS_CSV="$DEFAULT_ROBOTS"
CUTS_CSV="$DEFAULT_CUTS"

# Kept before the loop consumes them: the tee re-run below re-invokes this
# script with its own arguments, and by then "$@" has been shifted away.
ORIG_ARGS=("$@")

while (( $# )); do
  case "$1" in
    --robots)   ROBOTS_CSV="${2:-}"; shift 2 || die "--robots needs a value" ;;
    --robots=*) ROBOTS_CSV="${1#*=}"; shift ;;
    --cuts)     CUTS_CSV="${2:-}";   shift 2 || die "--cuts needs a value" ;;
    --cuts=*)   CUTS_CSV="${1#*=}"; shift ;;
    -h|--help)  usage; exit 0 ;;
    *)          usage >&2; die "unknown argument: $1" ;;
  esac
done

[[ -n "$ROBOTS_CSV" ]] || die "--robots was given an empty value"
[[ -n "$CUTS_CSV"   ]] || die "--cuts was given an empty value"

IFS=, read -r -a ROBOTS <<< "$ROBOTS_CSV"
IFS=, read -r -a CUTS   <<< "$CUTS_CSV"

# Duplicates are rejected rather than deduplicated: the second pass over one
# (robot, cut) would fit the same map twice and put two rows in the table that
# read as two maps.
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
if [[ -z "${B2MAPS_REFIT_LOG:-}" ]]; then
  mkdir -p "$LOGS" || die "cannot create $LOGS"
  B2MAPS_REFIT_LOG="$LOGS/refit_$(date -u +%Y%m%dT%H%M%SZ).log" || \
    die 'date failed, so this batch has no log name'
  export B2MAPS_REFIT_LOG
  bash "$0" "${ORIG_ARGS[@]}" 2>&1 | tee -a "$B2MAPS_REFIT_LOG"
  exit "${PIPESTATUS[0]}"
fi
REFIT_LOG="$B2MAPS_REFIT_LOG"

# ── the summary table ────────────────────────────────────────────────────────
# Accumulated as rows are finished, printed once at the end -- including on the
# interrupt path, which is where it is worth the most.

ROW_FMT='  %-12s %5s  %-8s %8s %8s %9s  %s\n'
ROWS=()
FAILURES=()

R_RUN=""; R_CUT=""
row_reset() { R_FITA='-'; R_FITB='-'; R_MARGIN='-'; R_VERDICT='-'; }
row_reset

emit_row() {
  [[ -n "$R_RUN" ]] || return 0
  ROWS+=("$(printf "$ROW_FMT" "$R_RUN" "$R_CUT" "$1" "$R_FITA" "$R_FITB" \
                   "$R_MARGIN" "$R_VERDICT")")
}

print_table() {
  echo
  echo "════════════════════════════════════════════════════════════════════════════════════════════════"
  printf "$ROW_FMT" run cut status 'fitA%' 'fitB%' 'margin' verdict
  local row
  for row in "${ROWS[@]}"; do printf '%s\n' "$row"; done
  echo "════════════════════════════════════════════════════════════════════════════════════════════════"
  echo "  fitA%/fitB%: fit_world_transform.py free fit, conventions A and B,"
  echo "               read out of this run's own fit log."
  echo "  margin:      best A - best B, in percentage points, read out of the JSON"
  echo "               this fit wrote (verdict.convention_margin_pp). Reported by"
  echo "               the fitter, never gated on -- and never recomputed here."
  echo "  verdict:     RESOLVED / AMBIGUOUS / UNRESOLVED / NO VERDICT."
  echo "  batch log:   $REFIT_LOG"
  if (( ${#FAILURES[@]} )); then
    echo
    echo "  ${#FAILURES[@]} map(s) FAILED:"
    local f
    for f in "${FAILURES[@]}"; do echo "    $f"; done
  fi
}

trap 'echo -e "\ninterrupted (SIGINT)" >&2; emit_row INTERRUPTED; print_table; exit 130' INT
trap 'echo -e "\nterminated (SIGTERM)" >&2; emit_row TERMINATED; print_table; exit 143' TERM

# A map that could not be fitted. Unlike the batch driver this does NOT stop:
# the row is closed as FAILED, the reason is recorded, and the loop goes on.
FAILED=0
row_failed() {
  echo -e "\n  FAILED: $*" >&2
  FAILURES+=("${R_RUN}_cut${R_CUT}: $*")
  FAILED=1
  emit_row FAILED
}

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

# ── preflight ────────────────────────────────────────────────────────────────

echo "════════════════════════════════════════════════════════════════════════════════════════════════"
echo "  b2maps re-fit -- run_b2maps_cuts.sh step 5 only, no SLAM"
echo "════════════════════════════════════════════════════════════════════════════════════════════════"
echo "  started        $(date -u +%Y-%m-%dT%H:%M:%SZ) (UTC)"
echo "  robots         ${ROBOTS[*]}"
echo "  cuts           ${CUTS[*]} s of sim"
echo "  maps           $(( ${#ROBOTS[@]} * ${#CUTS[@]} )), read from $MAPDIR -- none is re-made"
echo "  spawn-rev      $SPAWN_REV (fixed; the fc050c4 default misplaces b2maps maps by 3.15 m)"
echo "  expect         ~25 s per 60 s cut, ~66 s per 1200 s cut; ~14 min for all twenty"
echo "  batch log      $REFIT_LOG"
echo "  repo           $REPO_ROOT @ $(git rev-parse --short HEAD 2>/dev/null || echo '?')$(git diff --quiet 2>/dev/null || echo ' (dirty)')"

[[ -f "$FIT" ]] || die "missing script: $FIT"
[[ -x "$VENV_PY" ]] || die "$VENV_PY not found -- fit_world_transform.py runs in the venv"

# --bag reads the raw bag from inside the venv, which is the one flag of that
# script that needs the ROS environment. Checked here rather than discovered at
# the last map.
command -v ros2 >/dev/null 2>&1 || \
  die "ros2 is not on PATH. Run: source /opt/ros/jazzy/setup.bash && source $REPO_ROOT/install/setup.bash"
"$VENV_PY" -c 'import numpy, scipy, PIL, rosbag2_py' 2>/dev/null || \
  die "$VENV_PY cannot import one of numpy/scipy/PIL/rosbag2_py. The first three
are the venv's; rosbag2_py comes from the sourced ROS environment, which
fit_world_transform.py --bag needs."

# Every input of every requested map, listed in one go. Thirteen minutes in is
# the wrong place to find out that map twenty was never built.
MISSING=()
for K in "${ROBOTS[@]}"; do
  [[ -d "$LOGS/b2maps_k$K" && -f "$LOGS/b2maps_k$K/metadata.yaml" ]] || \
    MISSING+=("raw bag $LOGS/b2maps_k$K (not a rosbag2 directory)")
  for C in "${CUTS[@]}"; do
    CR="b2maps_k${K}_cut${C}"
    for f in "$MAPDIR/${CR}_robot$K.pgm" "$MAPDIR/${CR}_robot$K.yaml" \
             "$SLAMLOGS/map_to_odom_${CR}_robot_$K.txt"; do
      [[ -f "$f" ]] || MISSING+=("$f")
    done
  done
done
if (( ${#MISSING[@]} )); then
  printf '  missing: %s\n' "${MISSING[@]}" >&2
  die "${#MISSING[@]} input(s) missing. This script re-fits maps that already
exist; it does not build them. Use run_b2maps_cuts.sh for any that are absent."
fi
echo "  inputs         all present (map, yaml, map->odom log, raw bag)"

BATCH_START=$SECONDS

# ── robot x cut ──────────────────────────────────────────────────────────────

for K in "${ROBOTS[@]}"; do
  RUN="b2maps_k$K"
  BAG="$LOGS/$RUN"

  for CUT in "${CUTS[@]}"; do
    CUTRUN="${RUN}_cut${CUT}"
    # _refit.log, never _fit.log: the batch driver's own per-cut fit logs are
    # the record of the run that BUILT the map, and this did not build it.
    REFIT_STEP_LOG="$LOGS/${CUTRUN}_refit.log"
    CUT_START_SECONDS=$SECONDS

    R_RUN="$RUN"; R_CUT="$CUT"; row_reset

    echo
    echo "════════════════════════════════════════════════════════════════════════════════════════════════"
    echo "  robot_$K   cut ${CUT}s   ->  re-fit ${CUTRUN}_robot$K"
    echo "════════════════════════════════════════════════════════════════════════════════════════════════"

    # The batch driver's step 5, invocation for invocation (run_b2maps_cuts.sh:722):
    # --run is the cut, --bag is the RAW bag, because it is what corroborates the
    # spawn table against the robots that never moved and there is no bag under
    # the cut's name.
    if ! run_step "$REFIT_STEP_LOG" "$VENV_PY" "$FIT" --run "$CUTRUN" --robot "$K" \
           --spawn-rev "$SPAWN_REV" --bag "$BAG"; then
      row_failed "fit_world_transform.py failed ($REFIT_STEP_LOG)"
      continue
    fi

    OK=1
    for conv in A B; do
      pct="$(grep -a -m1 -E "^  convention $conv: free fit" "$REFIT_STEP_LOG" \
             | sed -E 's/.*free fit +([0-9.]+)%.*/\1/')"
      if [[ -z "$pct" ]]; then
        row_failed "no 'convention $conv: free fit' line in $REFIT_STEP_LOG"
        OK=0
        break
      fi
      if [[ "$conv" == A ]]; then R_FITA="$pct"; else R_FITB="$pct"; fi
    done
    (( OK )) || continue

    R_VERDICT="$(grep -a -m1 -E '^(RESOLVED|UNRESOLVED|AMBIGUOUS|NO VERDICT)' \
                 "$REFIT_STEP_LOG" | cut -c1-60)"
    [[ -n "$R_VERDICT" ]] || { row_failed "no verdict line in $REFIT_STEP_LOG"; continue; }

    # The margin comes out of the JSON this fit just wrote, not out of a
    # subtraction of the two percentages above: the fitter defines it
    # (verdict.convention_margin_pp) and a second definition here could drift
    # from it. The fit prints the path it wrote, so there is no guessing which
    # of the stamped files is this one.
    JSON="$(grep -a -m1 -E "^wrote .*/world_fit_${CUTRUN}_[0-9TZ]+\.json$" \
            "$REFIT_STEP_LOG" | sed -E 's/^wrote +//')"
    if [[ -z "$JSON" || ! -f "$JSON" ]]; then
      row_failed "the fit printed no world_fit JSON path, so the margin cannot be read ($REFIT_STEP_LOG)"
      continue
    fi
    R_MARGIN="$("$VENV_PY" - "$JSON" "$K" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
m = d['verdict']['convention_margin_pp'][f'robot_{sys.argv[2]}']
print('-' if m is None else f'{m:+.1f}')
PYEOF
)" || { row_failed "cannot read verdict.convention_margin_pp out of $JSON"; continue; }

    echo
    echo "    free fit: A $R_FITA%  B $R_FITB%   A-B margin $R_MARGIN pp"
    echo "    verdict:  $R_VERDICT"
    echo "    json:     $JSON"

    emit_row refit
    printf '  done in %d min %d s\n' \
           $(( (SECONDS - CUT_START_SECONDS) / 60 )) $(( (SECONDS - CUT_START_SECONDS) % 60 ))
  done
done

R_RUN=""          # nothing half-finished to close
print_table
printf '\n  %d map(s) over %d robot(s) in %d min %d s\n' \
       "${#ROWS[@]}" "${#ROBOTS[@]}" $(( (SECONDS - BATCH_START) / 60 )) \
       $(( (SECONDS - BATCH_START) % 60 ))
exit "$FAILED"
