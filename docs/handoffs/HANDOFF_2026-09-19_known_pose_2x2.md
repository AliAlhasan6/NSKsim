# HANDOFF — 2026-09-19 — Known-pose mapping: built, verified, and the 2×2

## 1. What this session was

It executed §5 of `HANDOFF_2026-09-18_two_layer_known_pose_mapping.md`.

- slam_toolbox source was read.
- A spec was written, and Claude Code built Parts 1–2 of it.
- Four commits were pushed.
- A 120 s runtime slice was run.
- Four offline arms were run on `b18_run2`, robot_0 only.

**The two-layer design now has its evidence.** With ground-truth poses and real
scans, robot_0's map fits the world at 96.1 %. The same scans on odometry fit at
21.9 %, which is noise level.

No simulator was run. Everything was offline replay of existing bags.

## 2. slam_toolbox with matching off is the locked layer

**Source.** `use_scan_matching` exists in every Jazzy release, from 2.8.0 through
HEAD `02afdde`. The installed version is 2.8.5, and the 2.8.5 tag was read
directly. Its `Mapper::Process()` is unchanged from 2.8.0. The only mapper
changes between those versions add `min_pass_through` and
`occupancy_threshold`, which affect how the grid is drawn, not how poses are
handled.

With the flag `false`:
- `MatchScan` is never called, so each scan keeps the pose it was given.
- The scan still goes into the sensor manager, so `GetAllProcessedScans()` →
  `CreateFromScans()` still draws it into the grid.
- The scan never enters the pose graph.
- `TryCloseLoop` sits inside the same `if`, so loop closure never runs,
  whatever `do_loop_closing` says.

This is the original open_karto structure.

**Runtime.** The 120 s slice confirmed all four checks:
- `ros2 param get` on the running node returned `False`.
- The map was non-empty: 197 occupied and 1617 free cells.
- map→odom was identity to the printed precision (under 0.5 mm and under 0.001°).
- There was one drop, the first scan at bag time 603.4 s.

**Consequence.** The standalone log-odds mapper proposed in the 09-18 handoff is
not needed. The locked layer is slam_toolbox with `use_scan_matching: false`,
replaying a bag whose `odom → base_footprint` has been rewritten from truth.

Offline maps are 0.10 m per cell.

## 3. What landed

Pushed as `a900953..6d67eab`. This also carried the three older commits,
`2b3d115`, `796a7ee` and `7402446`.

| Commit | Content |
|---|---|
| `7ece30e` | `bag_overlap.py`: `read_first_truth_poses()` and a pure `check_spawn_against_truth()`. `fit_world_transform.py --bag` runs the check before fitting and aborts on a mismatch. |
| `82bb567` | `rewrite_odom_from_truth.py`, with checks C1–C6. C6 is the parked-robot spawn check, shared with the fitter. |
| `684c444` | `SCAN_MATCHING` switch in `run_offline_maps.sh` via a `__SCAN_MATCHING__` placeholder. With the default, the output is byte-identical to before (md5 `eda84f4f…`, the same as the b16 run's params file). `false` changes exactly one line. |
| `6d67eab` | `docs/specs/SPEC_truth_pose_replay.md` |

Usage for the b18 era, per robot N:

```
rewrite_odom_from_truth.py --in <bag> --out <dir> --robot N --spawn-rev 08617b2
strip_bag_for_offline_slam.py <dir> <dir>_slamin
BAG=<dir>_slamin RUN=<name> RATE=2.0 SCAN_MATCHING=false run_offline_maps.sh N
fit_world_transform.py --run <name> --robot N --spawn-rev 08617b2 --bag <bag>
```

Wall time is about 23 min per robot on this machine, for a 2329 s bag at rate 2.0.

## 4. Traps found. Do not re-learn these.

**`--spawn-rev` pins the launch file, not the world.** It parses `DOT_POSES`
from `swarm_sim.launch.py` at that revision. `knowledge_world.sdf` is never
read, so a git diff on the world file proves nothing about spawn poses.

The default `fc050c4` predates the pentagon layout: it puts robot_0 at
(3.00, 0.00) instead of (0.86, 0.28). For the b18 era, always pass
`--spawn-rev 08617b2 --bag <bag with truth>`. In b18_run2, the parked robots
match `08617b2` to 1.6 mm and miss `fc050c4` by 3.15 m.

**A wrong `--spawn-rev` cannot move a score.** The spawn table reaches only
`analytic_candidates()`:
- The coarse search keeps the full correlation, uncropped.
- It searches thetas over the whole circle.
- Refinement is seeded by the coarse peak.

So a wrong revision misdirects the verdict instead: you get UNRESOLVED, which
blames a frame chain that was never broken. `--spawn-rev` stays optional,
because `phaseB_run1` genuinely needs `fc050c4`.

**The `--bag` gate needs ground truth.** Truth arrived with `662abe2` on
2026-09-10, so the gate cannot verify `phaseB_run1` or b16. It also needs at
least one parked robot as a witness (see §7).

**b18_run2 started recording late.** Recording began 603 s of sim time after
the sim started. The first truth pose is 3.075 m and −60.9° from spawn. By bag
start, robot_0 already carried 2.197 m and +121.47° of odometry error. That is a
single sample; do not cite it as a rate.

**The anchor is the spawn pose, not the first truth sample.** The first-sample
anchor would have let C4 pass on the 61° frame offset alone. Anchoring at spawn
makes the rewritten odom frame the spawn frame, which is exactly what the fitter
assumes: `world_T_odom` as a pure translation with spawn yaw 0.

**The AMBIGUOUS verdict in arm B is an artefact.** When map→odom is identity,
`map_T_odom` and its inverse are the same transform, so one candidate gets
counted twice. The printed explanation, world symmetry, is wrong in that case.
Not yet fixed.

**The rewriter's docstring claims too much.** It says b18_run2's map "shows the
same walls in two or three copies". That was b16's finding. Arm A shows
b18_run2's odometry map failing the fit, but nobody has checked its image for
copies. Not yet fixed.

## 5. Results — the 2×2 on b18_run2, robot_0

All four arms use the same scans and the same fitter, with
`--spawn-rev 08617b2 --bag b18_run2` corroborated by all four parked robots.
Fit scores are convention A.

| Arm | Poses | Matcher | Fit | Peaks 2–5 | map→odom | Occupied (grid) | Drops |
|---|---|---|---|---|---|---|---|
| B | truth | off | **96.1 %** | 96.1 67.8 67.8 62.7 | identity | 515 (106×97) | 1 |
| C | truth | on | 76.1 % | 76.1 51.1 51.1 46.0 | 0.98 m, +0.44° | 548 (105×94) | 1 |
| A | odometry | on | 21.9 % | 21.9 21.8 21.7 20.9 | 4.76 m, +44.35° | 2304 (218×161) | 0 |
| D | odometry | off | 14.4 % | 14.4 14.4 14.3 13.9 | identity | 6009 (189×203) | 0 |

In B, peaks 1 and 2 tie because the world's 180° twin fits equally well. The
truth anchor decides between them.

Logs are in `experiments/logs/b18/arm{A,B,C,D}_{console,fit}.txt`. Fit JSON and
PNG files are `experiments/logs/world_fit_b18r2_{truth,odom,truth_sm,odom_nm}_*`.

**Predictions against outcomes.** Each prediction was written before its run.

| Prediction | Outcome |
|---|---|
| 09-18: the duplicated walls collapse, and the fit clears the 80 % floor | **Held.** B scores 96.1 %. |
| Arm A, as control, fails the floor | **Held.** 21.9 %, with flat peaks. |
| Arm B's free fit lands within one cell and 0.5° of spawn | **MISSED.** Measured against the peak where the spawn placement lands (peak 2): dθ 1.00°, 0.12 m (dx 0.106, dy 0.054). |
| Arm D: map→odom identity, flat peaks below the floor, more than A's 2304 occupied cells | **Held.** 6009 occupied cells. |

About the miss: the spawn placement itself scores 96.1 %, level with the best
fit. The fitter's own self-check already treats a 1.5° difference on the plateau
as "reported, not gated", so the 0.5° tolerance was set without checking what
this fitter can resolve. The substitute criterion, "spawn scores level with the
best fit", was chosen **after** seeing the result and must be recorded as post
hoc. Rerunning B would not help: with matching off it is deterministic up to a
dropped scan, and the gap comes from the fitter, not the map.

**Reading.**
- **Odometry drift alone is enough to destroy the map.** Truth poses with the
  matcher off give 96.1 %.
- **The matcher is not neutral.** On perfect poses it costs 20 points (C against
  B) and drifts map→odom by 0.98 m. This is n = 1. It is also a second reason,
  beyond determinism, why the locked layer runs with matching off.
- **The odometry arms can't be ranked by score.** Below the 80 % floor, the
  fitter reports that it is mostly fitting the map's own noise. Occupied cells
  can rank them: 6009 in D and 2304 in A, against about 530 in the truth arms.
- **For Article 3's second condition** (HANDOFF_2026-09-18 §4): B against D is
  the odometry-only comparison, because it keeps the mapper that ships fixed. C
  against A is the same comparison under realistic SLAM. Both show that
  odometry error at this scale destroys a map rather than blurring it. This is
  one robot and one run.
- **B's remaining 3.9 % off-wall is unexplained.** One untested candidate is the
  four parked peers: they appear in the scans but not in the wall mask. That
  would be part of the open perception layer, not a defect. It is a guess, not
  a measurement.

## 6. Decisions made

- **Locked layer:** slam_toolbox with `use_scan_matching: false`, fed a bag whose
  `odom → base_footprint` has been rewritten from `/model/robot_N/pose` and
  anchored at spawn. It needs no matcher, no loop closure and no standalone
  mapper.
- **Anchor:** the spawn pose, from `DOT_POSES` at the revision that the parked
  robots corroborate.
- **`--spawn-rev` stays optional.** The `--bag` corroboration replaces the idea
  of making it required. A required flag would turn a silent error into a
  mandatory guess, and still couldn't tell a right guess from a wrong one.

## 7. First action next session — how to get five maps

This is the open problem of 09-18 §6, and it is now the blocker. B2 needs five
per-robot maps.
- b16 has no ground truth.
- In b18_run2, robots 1–4 were parked for the whole run, so their maps would be
  one viewpoint each. That is not usable for B2.
- Five Nav2 stacks are not possible on this machine; that attempt was abandoned
  at stack one.

**Candidate route: five sequential runs shaped like b18_run2.** In each run, one
robot explores with Nav2 and the other four are spawned and parked. Rotate the
explorer across the five runs, then map each explorer offline through §3.

What this keeps:
- The parked peers stay in the world as static obstacles, so part of the
  phantom-obstacle realism survives.
- The parked peers are the spawn witnesses that C6 and the fitter's `--bag`
  gate need.

What this costs:
- All moving robot-to-robot interaction.

That trade is a deliberate decision, not a default. Make it explicitly and
record the reason.

**Constraints on the recording, whatever the route:**
- **Start recording before any robot moves**, so the first truth sample is the
  spawn pose. That avoids b18_run2's 603 s gap and gives a second, independent
  spawn check.
- **Keep at least one robot parked.** A run with no parked robot leaves the
  `--bag` gate with no witness. It then prints "no robot was parked", and C6
  aborts unless `--allow-unverified-spawn` is passed.
- **Record `cmd_vel`.** It was missing from b18_run2, and the calibration design
  needs it (see the overview's decay entry).
- **Record `/model/robot_N/pose` for all five robots**, as `662abe2` already does.

Estimated cost per explorer: one recorded run of about 40 min, plus about
25 min offline (rewrite, strip, map, fit).

## 8. Small fixes owed (Claude Code; commit each separately)

1. Spec: record B's anchor miss and the post-hoc criterion; add arm D and the
   2×2 table.
2. `fit_world_transform.py`: when map→odom is identity within tolerance, merge
   the two candidate directions instead of reporting AMBIGUOUS.
3. `rewrite_odom_from_truth.py` docstring: remove the unverified "two or three
   copies" claim about b18_run2.

## 9. Carried forward unchanged

- The designed calibration run (`HANDOFF_2026-09-14` §6) is still owed. Every
  statistic referenced to odometry-versus-truth waits for it. The 2×2 fit
  scores are measured against world walls and are not in that category.
- `SPEC_b2_steady_gate.md` is **not on this machine**: `find ~` returned
  nothing. It probably exists only in an earlier chat. Recover it from there
  before landing it in `docs/specs/`.
- Three untracked files were left alone, and their origin was not established:
  `.vscode/` (probably belongs in `.gitignore`, as its own commit),
  `experiments/wall_affinity.py` and
  `ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`. Decide about them
  separately.
- The B2 schema work (§3.2–3.5 of `HANDOFF_phaseB2_schema_entry.md`) is
  untouched. Divergence is measured robot-to-robot and never references truth.
- Files on disk, all gitignored: `experiments/logs/b18/b18r2_truth/` (rewriter
  output, with `truth_anchor.txt`), `b18r2_truth_slamin/`, `b18r2_slamin/` and
  `slice120_console.txt`.
