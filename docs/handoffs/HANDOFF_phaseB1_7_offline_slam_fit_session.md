# HANDOFF — Phase B1.7: offline SLAM on b16, world fit, and why no map fits

Session: 2026-09-08 (continuous, ~05:00–22:30 local by file timestamps).
Previous handoff: `HANDOFF_phaseB1_6c_coverage_and_clock_stall_session.md`.
Repo `~/Desktop/NSKsim`, HEAD `f2638a4`. Push status at close: not checked — run `git status -sb`.

Replies in the next session: 120–130 words, one command block per turn, blocking/sim/expected-time header on every block, scripts as downloadable files, Ali runs terminals and commits, Claude Code edits repo files with manual approval. Nothing written to `/tmp`.

---

## 0. One-paragraph state

The offline pipeline for b16 is complete and proven end to end: a replay-ready bag with the live `map→odom` stripped, five per-robot slam_toolbox maps built at rate 2.0 (byte-identical to rate 1.0), and the world fitter run on them with run-parametric inputs. The result is a clean negative. **No b16 map clears the 80 % free-fit floor** (33–53 %). The fit PNGs show each map as two or three rotated copies of the same straight walls. Three explanations were tested and eliminated by measurement: wrong scored geometry, the CPU-starved five-stack period, and rotation faster than the scan matcher's search. The remaining candidates — odometry yaw error accumulated during heavy in-place spinning, or scan/pose timestamp skew — cannot be separated without ground-truth pose, which the bag does not contain. **B1.7 coverage and pairwise overlap are blocked until a map fits.** The requirement this places on the next simulation run is stated in §8.

---

## 1. Commits this session (all on `main`, none amended)

| hash | what |
|---|---|
| `bab60cc` | `experiments/analysis/tf_corrected_extents_v2.py` (fixed: robot0 stop 01:48:16→01:47:37, threshold labels `.3f`) and `tf_frame_census.py`. From the previous session's `~/Downloads/`; `b16_coverage.py` and `tf_corrected_extents.py` v1 left uncommitted there. |
| `ade6860` | `run_offline_maps.sh` bag-parametric (`BAG`, `RUN`, `RATE`, `DUR_<N>`, clock source measured from `ros2 bag info`, `--topics` limited to the target robot); `strip_bag_for_offline_slam.py`; `pgm_extent.py`; `tf_pair_at.py`. |
| `f2638a4` | `fit_world_transform.py --run/--spawn-rev`; `bag_overlap.resolve_spawn_poses(rev)` with a `-Y` refusal; `strip_bag_for_offline_slam.py --start/--duration` (segment cut with `/tf_static` re-timestamped); `run_offline_maps.sh START_<N>` (kept, but see §5); `odom_yaw_rate.py`. |

Untracked and deliberate, unchanged from before: `.vscode/`, `experiments/wall_affinity.py`, `ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`. `experiments/logs/` and `experiments/maps/` are gitignored.

---

## 2. The b16 bag, measured

`experiments/logs/b16/b16_run1/b16_run1_0.mcap`, 2.3 GiB, Jazzy, 7732.78 s, 23:57:31.759 → 02:06:24.544 on 6–7 Sep, 3 372 040 messages. Sim stamps in `/tf` run 759.1 → 3428.5 s (the sim clock had been running ~35 min before recording started).

Per-robot counts are near-identical across all five robots — `/scan` 12 969–12 982, `/odom` 129 653–129 789 — so **the Gazebo bridge kept publishing scans and odometry for robots 1–4 after their stacks were torn down at 00:33:15**; they carry ~94 min of stationary data each. `/tf` holds 2 353 556 messages, every one carrying exactly one transform: ~341 k `robot_N/map→robot_N/odom` per robot from the live SLAMs and ~129.7 k `odom→base_footprint` per robot. `/tf_static` is five messages at bag time 0. `/clock`: 265 603 messages.

Bag-second anchors used throughout (from bag start 23:57:31.759):
- robots 1–4 last goal END ≈ 00:32:44 → 2113 s; cut used **2143 s** (+30 s tail)
- teardown complete 00:33:15 → **2144 s** (segment start for the late run)
- robot0 last goal END 01:47:37 → 6606 s; cut used **6636 s**
- bag 2143 s ↔ sim stamp ≈ 1298; bag 6636 s ↔ sim ≈ 2942

---

## 3. Replay preparation — `b16_slamin`

`strip_bag_for_offline_slam.py b16_run1 b16_slamin` (97 s, one pass). Keeps `/clock`, `/tf_static`, `/robot_N/scan`, `/robot_N/odom`, and `/tf` minus every transform whose child ends in `/odom`. Result: 1 705 232 `/tf` messages emptied and skipped (341 148 / 341 466 / 340 894 / 341 080 / 340 644 for robots 0–4), 648 324 kept, all `odom→base_footprint`. Duration 7712.87 s — 20 s shorter than the source because only `map→odom` was publishing at the very end. Output bag 1 627 221 messages.

Two facts about replaying b16 that are not true of `phaseB_run1` and cost time to establish:
1. **Do not pass `--clock`.** The bag carries its own `/clock` and every header stamp is that sim time; `--clock` excludes the bag's clock and publishes wall-based time instead, breaking every TF lookup. The runner now greps `bag info` for `/clock` and decides.
2. **The live `map→odom` must be stripped before replay.** `ros2 bag play` excludes topics, not transforms; two authorities on one frame pair would corrupt the `tf2_echo` read at save time.

---

## 4. Offline maps — `experiments/maps/b16_slamin_robot{0..4}.{pgm,yaml}`

Runner: `BAG=$PWD/experiments/logs/b16/b16_slamin DUR_1..4=2143 DUR_0=6636 RATE=2.0`. Params per robot in `experiments/logs/offline_mapping_b16_slamin_robot_N.yaml` (template unchanged: res 0.1, `minimum_travel_distance` 0.1, `minimum_travel_heading` 0.1, `coarse_search_angle_offset` 0.349). Logs `experiments/logs/offline_slam_b16_slamin_robot_N.log`, `map→odom` in `experiments/logs/map_to_odom_b16_slamin_robot_N.txt`.

| robot | cut (bag s) | grid | occ / free / unk | known extent (m) | final `map→odom` (x, y, yaw°) at sim |
|---|---|---|---|---|---|
| 0 | 6636 | 142×110 | 1820 / 4418 / 9382 | 13.5 × 10.9 | +8.512, +7.990, +66.1 @ 2942 |
| 1 | 2143 | 96×119 | 1400 / 4075 / 5949 | 9.5 × 11.8 | +5.125, +10.469, −126.1 @ 1298 |
| 2 | 2143 | 106×103 | 1016 / 4111 / 5791 | 10.6 × 10.3 | −1.823, −7.581, +91.6 @ 1298 |
| 3 | 2143 | 112×105 | 833 / 5561 / 5366 | 11.1 × 10.5 | −16.671, −5.747, +125.2 @ 1298 |
| 4 | 2143 | 103×118 | 1011 / 3821 / 7322 | 10.3 × 11.7 | −3.500, −11.918, +25.9 @ 1298 |

**Rate 2.0 is verified, not assumed.** Robot_1 at rate 1.0 (`RUN=b16_slamin_rate1`) produced a byte-identical map and identical `map→odom`; slam_toolbox is deterministic over the same scan set, so nothing was dropped. Drop census over all logs (`grep -c 'dropping message'`, INFO level — a WARN/ERROR grep misses them): robots 1–4 and the rate-1.0 run 0; robot_0 exactly 1, the first scan at sim 758.6 before any transform existed. Rate 4.0 was not tried; the saving was under an hour against another comparison run.

Wrong claim retracted during the session: comparing a map's known extent with §3's corrected trajectory box is not valid — the offline map frame is the odom frame at bag start, the live map frame is the odom frame at live start, and the two differ by the live `map→odom` at stamp 759 (robot_1: 7 m, 132°); lidar reach adds up to 3.5 m per side besides.

Live-vs-offline `map→odom` comparison for robot_1, composed through that offset: live(759) ∘ offline(1298) predicts (−14.3, 3.1, +6°); the live SLAM published (−12.8, 9.8, −53°). Two estimates of one drift, 7 m and 58° apart. Recorded, not resolved; the fit below is the arbiter.

---

## 5. Segment replay and the `/tf_static` trap

Goal: map robot_0's post-teardown segment alone (bag 2144 → 6636, robot_0 driving with one stack at a healthier RTF).

- `START_0=2144` via `ros2 bag play --start-offset` → 609 "Message Filter dropping message" lines from the first scan, alternating *queue is full* / *timestamp earlier than all the data in the transform cache*, no map ever published, save failed. Cause: **`--start-offset` skips `/tf_static` at bag time 0**, so `base_scan→base_footprint` never exists and the scan filter can never resolve a scan.
- A one-second `--topics /tf_static --playback-duration 1` pre-play in the runner **did not deliver** (same 609 drops). Not diagnosed further; the block was to be removed from the runner and replaced by a comment (Claude Code spec issued at close — verify it landed, `git show f2638a4 -- experiments/slam/run_offline_maps.sh`).
- Working route: `strip_bag_for_offline_slam.py b16_slamin b16_r0late --start 2144 --duration 4492`, which skips 336 310 pre-start messages and writes the five `/tf_static` messages at the segment start. Replayed with no offset: **0 drops**, map saved.

`experiments/maps/b16_r0late_robot0`: 150×106, 1010 / 3631 / 11 259, final `map→odom` (−7.310, +7.797, −109.1°) at sim 2942.

---

## 6. World fit — `fit_world_transform.py --run b16_slamin --spawn-rev 08617b2`

Self-check (phaseB robot_4, convention A) still passes: 97.2 % measured vs 96.4 % expected. The fitter is sound.

| robot | conv A | conv B | peaks 2..5 (A) |
|---|---|---|---|
| 0 | 33.2 % @ θ +88.7 | 34.1 % @ +91.2 | 33.2 32.5 32.3 31.8 — flat |
| 1 | 24.9 % @ −53.0 | 21.6 % @ +29.9 | 24.9 23.4 23.4 22.2 — flat |
| 2 | **49.3 %** @ −162.4 | 38.1 % | 49.2 35.1 35.1 33.8 — one distinct peak |
| 3 | 44.2 % @ −159.7 | 37.2 % | 43.9 35.7 35.4 35.4 — one distinct peak |
| 4 | 43.5 % @ +26.5 | 32.8 % | 43.2 35.9 35.9 33.3 — one distinct peak |
| 0, late segment only | **53.1 %** @ +1.1 | 50.4 % | 53.0 49.3 49.3 48.8 — flat |

NO VERDICT: nothing clears `WELL_FIT_FLOOR` = 0.80; all analytic candidates disagree, as expected when the reference itself is unfit. Outputs: `experiments/logs/world_fit_b16_slamin_20260908T092813Z.json` + five PNGs; `world_fit_b16_r0late_robot0_20260908T190349Z.png`. For the segment run the analytic candidates mean nothing — its `map→odom` starts from identity at 2144 s.

**What the PNGs show** (robot_2 and robot_0 inspected): walls are straight, not smeared. Robot_2's map is the interior pinwheel drawn three times — one copy on the true walls, two more rotated ~10–15° each about a point near the centre. Robot_0's is an outer corner, doubled, with a third copy at ~40°. The 33–53 % figure is the fraction of occupied cells belonging to whichever copy the fitter chose.

---

## 7. Hypotheses tested and eliminated

1. **Wrong scored geometry / spawn poses.** `knowledge_world.sdf` is the world at both `fc050c4` (BAG_ERA_REV) and `08617b2`; `git diff` on `worlds/` between them is empty. The launch at `08617b2` spawns with `-x -y -z` only, yaw 0, same pentagon `DOT_POSES` as the phaseB table. HEAD sat on `08617b2` from 15:54 on the 6th until 03:27 on the 8th (after b16 ended), and the **installed** `knowledge_world.sdf` and `swarm_sim.launch.py` that Gazebo loaded are byte-identical to `08617b2`. Eliminated.
2. **CPU-starved five-stack period (RTF ~0.25) corrupting sensor/physics timing.** Robot_0's post-teardown segment alone, mapped cleanly with zero drops, fits 53.1 % with a flat profile. Eliminated as the cause (it may still contribute).
3. **Rotation faster than the correlation matcher's ±20° search.** `odom_yaw_rate.py` over the full bag: every robot's odom yaw rate caps at exactly 1.00 rad/s (Nav2's limit) = 11.5° per 0.2 s scan; `scan>search` = 0 intervals for all five. Eliminated.

| robot | tf samples | total rotation (°) | revolutions | p90 rad/s | max | % time > 1.0 |
|---|---|---|---|---|---|---|
| 0 | 129 665 | 42 891 | 119 | 1.00 | 1.00 | 4.6 |
| 1 | 129 667 | 14 090 | 39 | 0.37 | 1.00 | 1.5 |
| 2 | 129 665 | 19 551 | 54 | 1.00 | 1.00 | 1.4 |
| 3 | 129 666 | 17 359 | 48 | 0.68 | 1.00 | 2.5 |
| 4 | 129 661 | 15 322 | 43 | 0.37 | 1.00 | 1.7 |

Side finding from the same table: robots 1–4 completed 39–54 full revolutions each while barely translating (the previous handoff's zero median displacement). That is the explorer or Nav2 recovery behaviour spinning in place, and in-place spinning is where Gazebo wheel odometry accumulates yaw error fastest.

**Remaining candidates**, not separable from this bag:
- (a) odometry yaw error accumulated during the spinning, large enough per episode that the matcher re-anchors on a wrong heading when little structure is in view;
- (b) scan/pose timestamp skew in the Gazebo → ros_gz bridge under load, which rotates each scan relative to the pose it is attributed to.

Both are measured only against ground truth. The bag has no true pose.

---

## 8. What the next simulation run must record (proposed B1.8 requirement)

1. **Gazebo ground-truth pose per robot**, bridged and bagged alongside `/odom` and `/tf` — e.g. the model pose from `/world/<world>/pose/info` or a per-model `/model/robot_N/pose` bridge, at ≥10 Hz in sim time. This alone separates (a) from (b): odom yaw vs true yaw over time is (a); scan-vs-true-pose consistency is (b).
2. Keep everything else about the recording identical (same topics, `/clock`, `/tf_static`) so the strip → map → fit pipeline runs unchanged with `--run <name> --spawn-rev <rev>`.
3. Decide, before launch, whether the in-place spinning is behaviour to keep. If it is a recovery loop, it is both a coverage cost and the prime suspect for (a); limiting it is a separate change and should be its own commit and its own run.

Pipeline for the next bag, in order, all existing: `strip_bag_for_offline_slam.py` → `run_offline_maps.sh` (rate 2.0, per-robot `DUR_<N>`) → `fit_world_transform.py --run … --spawn-rev …` → coverage and overlap **only** over robots that clear 0.80.

---

## 9. Loose ends

- `tf_corrected_extents_v2.py --report-threshold` still defaults to 0.05; the sweep showed it changes nothing, so it was deliberately left.
- `run_offline_maps.sh START_<N>` remains as a flag but is a trap without a segment bag (§5). Confirm the pre-play block is gone.
- `experiments/logs/b16/` now holds two derived bags: `b16_slamin` (full, stripped) and `b16_r0late` (segment). Sizes not recorded; check before the disk fills.
- `~/Downloads/`: `b16_coverage.py` and `tf_corrected_extents.py` v1 — not committed, discard.
- The live-vs-offline `map→odom` disagreement (§4) is unexplained and may become relevant once ground truth exists.
- Nothing has been pushed this session unless Ali did so unprompted.

---

## 10. Retractions this session, so they are not re-argued

- "Compare the offline map extent with the §3 corrected box" — invalid, different frames (§4).
- "Rate 2.0 may have dropped scans in the late segment" — wrong; drop census showed one benign drop (§4).
- "`/tf_static` pre-play in the runner will fix `--start-offset`" — did not (§5).
- "The rotated copies come from the starved five-stack period" — eliminated (§7.2).
- "The copies come from turning faster than the matcher's search" — eliminated (§7.3).
