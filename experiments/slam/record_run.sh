#!/usr/bin/env bash
# Record a five-robot run with ground truth. RUN=<name> (default b18_run1).
# Topic list is b16_run1's exact set plus per-robot ground-truth pose.
set -euo pipefail
RUN="${RUN:-b18_run1}"
OUT="experiments/logs/${RUN%%_*}/${RUN}"
T=(/clock /tf /tf_static /kg_share /nsk/convergence)
for N in 0 1 2 3 4; do
  T+=("/robot_$N/odom" "/robot_$N/scan" "/robot_$N/map" \
      "/robot_$N/map_metadata" "/model/robot_$N/pose")
done
# ros2 bag record silently skips topics that are not yet advertised, so a
# recorder started before SLAM/Nav2 produces a bag missing /robot_N/map and
# /robot_N/map_metadata with no error (observed on b18_smoke: 20 of 30).
have="$(ros2 topic list)"
missing=()
for t in "${T[@]}"; do
  grep -qxF "$t" <<<"$have" || missing+=("$t")
done
if (( ${#missing[@]} )); then
  echo "ABORT: ${#missing[@]} of ${#T[@]} topic(s) not advertised:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  echo "Bring up SLAM/Nav2 and wait for all stacks active, then re-run." >&2
  exit 1
fi

echo "recording ${#T[@]} topics -> $OUT"
exec ros2 bag record -o "$OUT" "${T[@]}"
