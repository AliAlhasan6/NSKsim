# HANDOFF — 2026-09-27 — Five B2 runs recorded, 20 deterministic maps built

## 1. What this session was

It started at §7 of `HANDOFF_2026-09-21_lds02_and_free_space_relay.md`:
decide the stopping budget, then record five per-robot runs.

It ends with:

- all five runs recorded and checked (`b2maps_k0` … `b2maps_k4`);
- the offline map pipeline made **deterministic**;
- all 20 known-pose maps (5 robots × cuts of 60 / 120 / 240 / 1200 s)
  built, gated and fitted in one batch
  (`experiments/logs/b2maps/batch_20260927T124741Z.log`, 170 min 48 s).

Along the way, three recording attempts were aborted and eight defects
were found and fixed (§7).

## 2. Decisions taken

- **Budget.** Record at least 1200 s of sim time after the first nonzero
  command. Cut the maps offline at 60 / 120 / 240 / 1200 s from that one bag.
  The 2400 s budget is superseded: with the LDS-02 and the relay, a single
  robot knows 99 % of its final floor 389–470 s after its first command
  (all five runs, §5), so 2400 s maps would be near-identical.
- **World kept.** `partial_maps.py` showed early maps are shaped by wall
  shadows, not only by sensor range, so no harder world was built.
- **Run names.** `b2maps_k0` … `b2maps_k4`, robot K explores, the other four
  stay parked. `b2maps_e0` and `b2maps_e0t` remain controls.
- **Knowledge layer left unseeded for B2** (§6.5). All five runs follow the
  same protocol; each run's per-agent node counts are in its sim log. A
  fixed seed is required before any knowledge-level experiment.
- **Every B2 offline map is built with `DETERMINISTIC=true`** (§6.4).

## 3. What landed

| Commit | Content |
|---|---|
| `c6aadac` | `analysis/map_saturation.py`: free area vs sim time from a map topic |
| `dcb553b` | `analysis/partial_maps.py`: occlusion index of early-cut maps |
| `abbe3ed` | `check_run_bag.py`: `--min-sim` (alias `--budget`), shortfall in sim seconds |
| `bb28c2b` | README: 1200 s recording, four offline cuts |
| `0f48865` | Explorer: bounded Nav2 start gate; one lost `get_state` no longer hangs |
| `a0760da` | README: run names `b2maps_kK`, gate retry note, recorder in teardown `pgrep` |
| `79dc4bf` | `run_offline_maps.sh`: stops every child on every exit path (relay leak) |
| `cc92c38` | README: what a surviving recorder actually does |
| `f3c5bbc` | `run_offline_maps.sh` + template: `DETERMINISTIC` mode |
| (between) | `analysis/pgm_agreement.py`; `slam/run_b2maps_cuts.sh` batch driver |
| `61140c6` | README: every B2 map built with `DETERMINISTIC=true` |
| (between) | `check_run_bag.py` C4 float fix + tests; scan queue in `DETERMINISTIC` mode + `bag_scan_facts.py`; driver drop gate |
| `d9db691` | README: scan queue and drop gate (HEAD at the final batch) |

**Verify first thing:** the last four commits' push. Their `git push` output
was not seen in chat. Run `git status` and `git log --oneline -6 origin/main`.

Untracked and unrelated, as before: `.vscode/`, `experiments/wall_affinity.py`,
`ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`.

## 4. The five runs

| Run | Spawn (x, y) | Bag | Sim at 1st cmd | C4 `--start` | RTF (1200 s) | R4 | R9 | Closest approach | kg_share |
|---|---|---|---|---|---|---|---|---|---|
| k0 | (+0.86, +0.28) | 1.33 GB | 105.850 | 54.591 | 0.987 | FAIL: 32 leading | PASS | 0.325 m | 1035 |
| k1 | (0.00, +0.90) | 997 MB | 44.320 | 23.246 | 0.992 | PASS | PASS | 0.300 m | 835 |
| k2 | (−0.86, +0.28) | 970 MB | 75.820 | 39.789 | 0.970 | FAIL: grace withheld | FAIL: 1 controller TF warning | 0.328 m | **29** |
| k3 | (−0.53, −0.73) | 1.08 GB | 96.330 | 53.083 | 0.978 | FAIL: 34 leading | FAIL: 5 online queue-full drops | 0.338 m | 951 |
| k4 | (+0.53, −0.73) | 988 MB | 95.810 | 68.319 | 0.946 | FAIL: 28 leading | PASS | 0.289 m | 970 |

All five pass C1–C4. R2, R5, R7 (full 20 × 20 m online map) and COVERAGE
pass on all five. Every R4/R9 failure lies outside the analysis window or
touches only the online map (§8.2).

C4 `--duration` per cut (the `--start` is constant per bag, as required):

| Run | 60 | 120 | 240 | 1200 |
|---|---|---|---|---|
| k0 | 67.389 | 128.479 | 250.019 | 1215.433 |
| k1 | 60.559 | 121.341 | 242.775 | 1209.178 |
| k2 | 68.351 | 136.218 | 263.198 | 1237.671 |
| k3 | 65.642 | 131.027 | 259.793 | 1226.843 |
| k4 | 100.833 | 174.142 | 298.735 | 1268.115 |

Aborted attempts are kept as evidence in `experiments/logs/b2maps/`:
`aborted_k0_gate_hang/`, `aborted_k1_clock_bridge_killed/`,
`aborted_k3_recorder_late/`.

Run notes:

- **k0.** R4's 32 leading transforms: the recorder subscribed `/tf` 2.34 s
  (wall) before `/model/robot_0/pose`. relay1's 3 matched its 0.14 s gap.
- **k1.** The explorer finished on its own after 21 goals (~10 min); the
  recorder was held to 30 min, as the README requires.
- **k2.** An unusually heavy knowledge-graph draw (17 212 nodes against
  k1's 4 918) caused 97 % `/nsk/compress` timeouts, collapsing `/kg_share`
  to 29. It also raised load (4.5 control-loop misses/min against 0.6).
  Mapping was unaffected. R4 withheld its grace for a 0.2 s pose gap at
  sim 61.85 s, before the first command.
- **k3.** The online SLAM dropped 11 scans mid-run: 5 "queue is full"
  (R9's five) and 6 "earlier than the transform cache". This affects the
  online map only.
- **k4.** Busiest run: 701 control-loop misses (21/min) and the lowest RTF.
  The final map was unaffected.

## 5. Results

### 5.1 Saturation (online maps, `map_saturation.py`)

| Run | 99 % of final free area, s after 1st cmd | Final free |
|---|---|---|
| k0 | 389 | 387.9 m² |
| k1 | 413 | 387.9 m² |
| k2 | 406 | 387.9 m² |
| k3 | 412 | 387.5 m² |
| k4 | 470 | 387.5 m² |

### 5.2 Early maps (online, `partial_maps.py`): % of final free / occlusion index

| Run | 60 s | 120 s | 240 s |
|---|---|---|---|
| relay1 | 40 / 0.589 | 53 / 0.706 | 63 / 0.718 |
| k0 | 14.8 / 0.229 | 41.8 / 0.501 | 76.1 / 0.819 |
| k1 | 15.1 / 0.224 | 43.3 / 0.523 | 64.0 / 0.685 |
| k2 | 27.6 / 0.382 | 45.1 / 0.602 | 77.1 / 0.849 |
| k3 | 26.7 / 0.399 | 45.7 / 0.613 | 74.3 / 0.819 |
| k4 | 27.0 / 0.394 | 44.1 / 0.568 | 62.0 / 0.694 |

The 120 s cut is tight across all five robots (42–46 %). The 60 s cut splits
into two groups. relay1 (a rehearsal) is the outlier; the reason is open.

### 5.3 The 20 offline maps (`DETERMINISTIC=true`, 0.1 m cells)

Free area = free cells × 0.01 m².

| Run | Cut | occ / free / unknown | Free m² | Fit A % | Fit B % | Verdict | Drops | md5 (12) |
|---|---|---|---|---|---|---|---|---|
| k0 | 60 | 436 / 5839 / 3526 | 58.4 | 98.2 | 67.4 | RESOLVED | 0 | c5b4b97c70e8 |
| k0 | 120 | 679 / 16675 / 5046 | 166.8 | 98.8 | 69.2 | RESOLVED | 0 | 237e43e7c65a |
| k0 | 240 | 975 / 29853 / 1172 | 298.5 | 99.2 | 71.4 | RESOLVED | 0 | 19031bc86937 |
| k0 | 1200 | 1565 / 38267 / 168 | 382.7 | 99.4 | 80.1 | RESOLVED | 0 | aefb6eea256d |
| k1 | 60 | 298 / 5982 / 3440 | 59.8 | 98.0 | 66.1 | RESOLVED | 1 | dcad3ee2a9c2 |
| k1 | 120 | 712 / 17076 / 4612 | 170.8 | 98.7 | 73.7 | RESOLVED | 1 | 54d17b9f1e10 |
| k1 | 240 | 1039 / 27769 / 3192 | 277.7 | 99.1 | 77.1 | RESOLVED | 1 | b052b896554b |
| k1 | 1200 | 1622 / 38215 / 163 | 382.2 | 99.6 | 81.3 | **AMBIGUOUS** | 1 | 969de699b867 |
| k2 | 60 | 521 / 13234 / 3045 | 132.3 | 97.7 | 66.6 | RESOLVED | 1 | 4898cc88f0c7 |
| k2 | 120 | 709 / 17867 / 3824 | 178.7 | 98.4 | 69.3 | RESOLVED | 1 | 6ae8f2451abf |
| k2 | 240 | 1169 / 30113 / 1518 | 301.1 | 99.1 | 75.9 | RESOLVED | 1 | 4ac428b5efaa |
| k2 | 1200 | 1622 / 38231 / 147 | 382.3 | 99.5 | 79.5 | RESOLVED | 1 | d7ed59860923 |
| k3 | 60 | 582 / 11578 / 7680 | 115.8 | 98.6 | 67.5 | RESOLVED | 0 | ad79b7308e4c |
| k3 | 120 | 932 / 20450 / 7738 | 204.5 | 99.0 | 79.9 | RESOLVED | 0 | 72d4b09db7a6 |
| k3 | 240 | 1330 / 29441 / 5629 | 294.4 | 99.5 | 81.0 | RESOLVED | 0 | 9c2c55b46164 |
| k3 | 1200 | 1639 / 38205 / 156 | 382.1 | 99.6 | 80.2 | RESOLVED | 0 | 030f02fe768f |
| k4 | 60 | 369 / 13325 / 3106 | 133.3 | 97.8 | 71.5 | RESOLVED | 1 | 8528343ae000 |
| k4 | 120 | 586 / 17576 / 4238 | 175.8 | 98.6 | 71.5 | RESOLVED | 1 | 4b146d4bf622 |
| k4 | 240 | 1126 / 25711 / 5163 | 257.1 | 99.3 | 80.8 | **AMBIGUOUS** | 1 | 98ad885c1570 |
| k4 | 1200 | 1753 / 38091 / 156 | 380.9 | 99.5 | 80.8 | **AMBIGUOUS** | 1 | 9e5294d5caf1 |

- Every placement is the robot's spawn, within about 1° and 0.2 m of the
  free optimum.
- Four parked robots corroborate `08617b2` in every fit.
- Every allowed drop is the segment's first scan.
- Zero leftover processes after every map.

**Determinism proof in the batch:** k0's rebuilt maps reproduce the proof
digests exactly: cut 60 `c5b4b97c70e89d69b141098427b75257`, cut 1200
`aefb6eea256df1023464d46e10f10515`. k0's short cuts and k4 cut 60 also
rebuilt with zero cell delta.

The superseded base-mode 1200 s maps are in `experiments/maps/superseded/`.
Deltas det vs base at 1200 s: k1 −85 occupied, k2 −73, k3 −11, k4 +45.

## 6. Findings

### 6.1 Saturation repeats across robots
99 % came 389–470 s after the first command in all five runs, so the 1200 s
budget has at least 2.5× margin.

### 6.2 Early coverage
Offline free area at 60 s is about 59 m² for k0 and k1, and 116–133 m² for
k2–k4. At 120 s the spread is 167–205 m², and at 1200 s all five sit at
381–383 m². Differences between robots are an early-time phenomenon.

### 6.3 The spawns are clustered
All five spawns lie within about 2 m of each other near the centre. Map
differences come mainly from explorer route choice, not viewpoint.

### 6.4 Offline replays were not reproducible — now fixed
Four mechanisms were involved:

1. **Chained scan acceptance.** `minimum_travel_distance` 0.1,
   `minimum_travel_heading` 0.1 and `minimum_time_interval` 0.2 are each
   measured from the previous accepted scan. Whichever first scan the
   startup race dropped re-anchored every later decision: k4 cut 60 base
   mode, 2 of 7 replays differed by 3.06 % of known cells.
2. **Scan queue of 1.** slam_toolbox's `scan_queue_size` defaulted to 1,
   so under load a scan waiting on its transform was evicted (k0 cut 1200:
   6 mid-replay queue-full drops).
3. **Startup race.** The first scan of each segment has no transform behind
   it yet, so it is dropped. With every scan integrated, that drop is inert.
4. **Floating-point residue in C4** (§7).

`DETERMINISTIC=true` sets all three gates to 0, `PLAY_DELAY` 3.0, and
`scan_queue_size` to the segment's own scan count. Replays are then
byte-identical (5/5 at cut 60, 3/3 at 240, 2/2 at 1200, plus the k0 batch
check).

- The variant shifts maps versus base by 4.1 % (60 s), 4.3 % (240 s) and
  0.22 % (1200 s) of cells, mostly unknown becoming known.
- Fit A is essentially unchanged.
- The default mode is byte-identical for old bags: the e0t digests
  reproduce, and `b2maps_e0t_kp_robot0.pgm` stays
  `e432e857148fabd7b28bc9d2a5d702f5`.

### 6.5 Knowledge graphs are drawn at random on every engine start
- The NSK repo's `src/utils/data_loader.py:245` uses `shuffle=True` with no
  generator, and NSKsim's `seed` launch argument defaults to −1 and has
  never been passed ("Seed: unseeded" in all 11 sim logs).
- Per-agent node counts range from single digits to ~6000.
- Agent 4's dataset index 3600 exceeds the 3,171-graph train split and
  wraps to position 429.
- Irrelevant to maps. It must be fixed (seed + in-range indices) before any
  knowledge-level experiment.

## 7. Traps found. Do not re-learn these.

- **Nav2 start gate could hang forever.** Jazzy `_waitForNodeToActivate`
  spins on `get_state` with no timeout; one lost response under load hangs
  it. Fixed in the explorer (`0f48865`); a retry line under load is normal.
- **Never kill the bridges while the sim runs.** `pkill -f parameter_bridge`
  also kills the `/clock` bridge and freezes sim time (lost k1 attempt 1).
  The teardown Block B now refuses to run while `gz sim` is alive.
- **Never start the explorer before the recorder shows 26 subscriptions.**
  The recorder joined 32 s after goal #1 (lost k3 attempt 1). The Terminal 3
  block now refuses to launch below 26.
- **`[r]osbag2` does not find the recorder.** `record_run.sh` execs
  `ros2 bag record`; use `[b]ag record` and `[r]ecord_run`.
- **Never move a bag folder while its recorder runs.** The open `.mcap`
  follows the move, but `metadata.yaml` is written to the old path at
  shutdown and is lost.
- **`run_offline_maps.sh` used to leak its relay.** It killed only the
  `ros2 run` wrapper PID (fixed `79dc4bf`). The Terminal 1 leftover-process
  guard caught it.
- **slam_toolbox logs to its own file.** Read
  `experiments/logs/offline_slam_<run>_robot_K.log`, not the runner output.
- **C4 float residue.** `104.32 − 44.32 = 59.99999999999999` failed a window
  C4 had just certified. Fixed with a 1 ns tolerance; 20/20 cuts PASS.
- **Every "Terminal N" is its own tab.** Only the Terminal 4 check block runs
  during a recording. Blocks A and B are for teardown only.

## 8. Open — first actions next session

### 8.1 Fitter convention gate (touches the results table)
Convention B clears the 80 % floor on full maps (79.5–81.3 %), because the
square outer boundary dominates the wall count and is symmetric. That
leaves three AMBIGUOUS verdicts, though A beats B by 18.3 pp or more in
every map. Options:

- (a) Gate the verdict on a margin, e.g. A − B ≥ 10 pp. All 20 maps would
  resolve, and the gate can still fail.
- (b) Drop B from gating, if it is established as the mirrored row order.

Decide, implement, re-run the fits only (fast), and re-tabulate.

### 8.2 R4 and R9 reshaping (before judging the five)
- **R4.** The fixed 20-period grace is the wrong shape; the leading edge
  depends on recorder subscription order (0.14–2.34 s). Candidates: gate R4
  on the analysis window [first command, +1200 s] and only report outside
  it, or bound the grace by the record log's `/tf`-vs-pose subscription gap.
  k2's "holes withhold grace" case lies before the first command.
- **R9.** Split by node and frame pair (plan:
  `~/.claude/plans/zazzy-forging-sonnet.md`). Treat online-SLAM drops as
  advisory, since the product maps are offline.

Re-run `run_health.py` on all five after the change.

### 8.3 README
Adopt the two guards from the chat blocks into the run procedure: the
Terminal 3 recorder-count check, and Block B's `gz sim` check.

### 8.4 Then the analysis
Robot-to-robot divergence on the 20 maps (B2 schema §3.2–3.5 of
`HANDOFF_phaseB2_schema_entry.md`). Divergence never references truth.

- Register the maps in the world frame by the spawn translation (the fits
  confirm it).
- Offline maps are 0.1 m with per-cut grids and origins; online maps are
  0.05 m.
- `pgm_agreement.py` is the cell-wise tool.

### 8.5 Cleanup (optional)
- Proof maps: `experiments/maps/b2maps_k0_cut1200_q{1,2}_robot0.*`,
  `b2maps_k0_cut60_q1_robot0.*`, `experiments/maps/replay_noise/`.
- Keep `experiments/maps/superseded/`.

## 9. Carried forward unchanged

- The designed calibration run (`HANDOFF_2026-09-14` §6) is still owed for
  any ground-truth-referenced statistic in Article 3.
- The `parameter_bridge`-as-`Node` fix was deferred until all five runs
  were recorded. They now are, so it is unblocked.
- The rewriter's `--from {truth,odom}` selector is deferred on the same
  terms.
- The knowledge-layer seed and in-range dataset indices (§6.5).
- Why relay1's early coverage differs from k0 at the same spawn.
