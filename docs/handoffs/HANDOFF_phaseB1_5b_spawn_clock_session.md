# HANDOFF — Phase B1.5b (clock bridge + retry ladder + spawn geometry)

**Session date:** 2026-07-24
**Repo:** `AliAlhasan6/NSKsim` — branch `main`
**Entry state:** B1.5a done; goal was to scale exploration to 5 robots (option 2→1)
**Exit state:** two critical latent bugs fixed and pushed; a large tested-but-uncommitted
pile awaiting rung re-validation; one new blocker (odom drift) diagnosed and parked.

---

## 1. WHERE WE STAND

### Pushed to remote (verified, safe)
| commit | what | how verified |
|--------|------|--------------|
| `c0e7f43` | **/clock bridge** from Gazebo | `/clock` publisher 1, ~76 Hz; `sim=` tracks `wall=` across 39 goals; `inf Hz` gone |
| `e864ae7` | **clustered spawn poses** (0.9 m pentagon) | Gazebo ground truth: all 5 within 0.75 m, 10/10 pairs in 3.0 m comm range |

### Built + tested + runtime-verified, **NOT committed** (biggest current risk — laptop-only)
All 116 unit tests green. Interrupted before full rung re-validation by the odom discovery.
- `goal_supervisor.py` — RTF-invariant goal supervision (sim-time progress + sim timeout)
- `outcome_classifier.py` — WORLD vs STACK failure classification; per-frontier retry ladder (3 retries); stack-wide health guard (5 distinct-frontier failures → stop)
- `frontier_explorer.py` — `_supervise_goal`, resend-guard defers to retry ladder, auditable termination block, window-displacement logging
- `nav2_robot0.yaml` — `wait_for_service_timeout` 1000 → 5000 ms
- `save_map.py` — argparse (`--topic`/`--out`/`--timeout`); was hardcoded to `/map`
- `explore.launch.py` — opt-in `rviz:=true` (default false; RViz corrupts RTF), node-name fix (dropped `name=` that renamed both nodes process-wide)
- `.gitignore` — un-ignore `experiments/slam/*.py` and `*.yaml`
- **Staged but reset earlier:** `.gitignore`, `experiments/slam/save_map.py`, `experiments/slam/robot_0_mapping.yaml` — need re-adding

### New blocker (diagnosed, parked)
**Odom drift on stationary burgers.** `/odom` diverges on robots commanded to hold still
(`cmd_vel` silent; Gazebo confirms them static at origin). Values drift wildly and
inconsistently between reads (r0: (1.38,3.28)→(6.73,5.82) in ~1 min).

---

## 2. WHAT HAPPENED THIS SESSION (chronological)

1. **Chose option 2→1** (measure at 2 robots, then all 5 live, full fidelity) — concurrency is the experiment, wall-clock is the only free currency, map quality is B2's raw material so no degradation.
2. **Clock bug (rung1/1b).** Supervisor logged `sim=0.0s` every goal. Root cause: Gazebo `/clock` never bridged into ROS → 25 nodes with `use_sim_time:=true` had `node.now()` frozen at 0; also caused `controller_server` `inf Hz` (1/dt, dt=0). Fixed via dedicated `ros_gz_bridge` node with `use_sim_time:=False`. → `c0e7f43`.
3. **Blacklist contamination (rung1b→1c→1d).** Explorer blacklisted frontiers on Nav2 `FAILED`, but many FAILEDs were `bt_navigator` ack timeouts (infrastructure, not geometry). Measured: rung1b terminated "complete" with 508 frontier cells still on the map (43% of boundary unexplored). Introduced outcome classifier + retry ladder + stack-wide guard. rung1d showed full ladders (1/3→2/3→3/3) and frontiers succeeding on retry that would previously have been discarded.
4. **World-size correction.** Assumed ~8×10 m maze all session. World file: **20×20 m (400 m²)**. Recomputed coverage: rung1b 5.7%, rung1c 7.4%, rung1d 19.7%. rung1d halted correctly on the stack-wide guard, not exhaustion.
5. **Comm-range / spawn geometry.** Realized 400 m² + 3.0 m comm range + frontier exploration (which pushes robots apart) = near-zero sharing encounters. Measured old spawns: **0/10 pairs in range**, nearest 4.20 m. Decision: keep short comm range (harsh comms-denied environment is the whole motivation; it's the constraint that makes the agreement problem non-trivial and keeps the work tied to Steels' local-interaction lineage). Fixed via clustered spawns (lever 1), not by inflating comm range (lever 2, reviewer-bait). → `e864ae7`.
6. **Odom drift found.** t=0 sharing test showed odom positions nowhere near spawns and drifting. Ruled out (via Gazebo ground truth + cmd_vel silence) spawn failure, offset-wiring, and flocking. Diagnosed as dot-era odom assumption vs burger reality. Parked as next session's entry point.

---

## 3. NEXT SESSION — DO THIS, IN ORDER

### Step 1 — Land the uncommitted pile (removes laptop-only risk)
- Re-run rung1 with the new clustered spawns to confirm the retry ladder end-to-end
  (`swarm_sim.launch.py nav_robots:=[0]` + `explore.launch.py robot_id:=0`).
  Note: single-robot exploration is unaffected by the odom-drift bug because Nav2/SLAM
  corrects pose; the drift only bites the monitor's raw-`/odom` range math.
- Verify: retry ladder reaches 2/3 and 3/3, termination audit honest, frontier share < 43%.
- Commit the classifier/supervisor/tooling pile. Re-add the reset `.gitignore` +
  `save_map.py` + `robot_0_mapping.yaml` first.

### Step 2 — Fix odom drift (the real gate to a valid 5-robot sharing run)
- **Root cause:** `convergence_monitor.py:129` comment ("dots' OdometryPublisher was
  already world-frame") records a dot-model assumption. Burgers publish spawn-relative
  diff-drive odom that integrates contact jitter → drifts on a stationary robot.
- **Likely fix:** monitor reads SLAM-corrected TF (`map` → `base_footprint`) instead of
  raw `/odom` for pair-distance / comm-range math. SLAM already corrects this drift.
- Everything downstream (comm gating, `/kg_share`, EXP-01 similarity) depends on it.

### Step 3 — EXP-01 t=0 sharing test (was about to run when odom bug surfaced)
- With trustworthy positions: launch `nav_robots:=[]`, confirm all pairs register
  in-range and sharing fires *before* releasing exploration, then release and watch
  divergence develop from the common origin.

---

## 4. ALSO PARKED (smaller, non-blocking)

- **Spawn-yaw / odom-frame coupling.** Spawn offsets are exact only at yaw 0 (all current
  spawns are yaw 0, so dormant). Outward-facing spawns would need this fixed first.
- **Min spawn separation 0.15 m actual** (Gazebo) vs 0.21 m footprint — tight. Nudge the
  ring radius out slightly. (Designed 1.06 m; robots settled closer.)
- **Health-guard diagnostic text is backwards.** Says "the frontiers are reachable, the
  STACK is not" — but rung1d's terminal failures were 1-second planner *goal rejections*
  (`GridBased plugin failed to plan`, 1.3 m goals), i.e. bad goals, not an unhealthy stack.
  Action was right, explanation is wrong. Reword.
- **Resend-guard log string misleads.** Says "re-selected without progress" but actually
  measures distance between consecutive goal *coordinates* (`_last_goal`, written at
  dispatch, never reset), not robot motion. Rename to "re-selected within RESEND_RADIUS".
- **`MIN_MOVE` (0.1) vs `PROGRESS_MIN_MOVE` (0.25)** — two displacement thresholds, two
  different questions (whole-goal success gate vs in-flight progress window). Both justified
  but must be documented before the thesis chapter.
- **`FUTILE` verdict legibility** — now prints `window=` alongside `net=` so goals like
  rung1c #26 (FUTILE, net=2.99 m, correct) are readable. Confirm it's landing in runs.

---

## 5. KEY NUMBERS (for reference)

- **World:** 20×20 m, 400 m², sparse (18 model/link tags — long wall segments, not a dense maze)
- **Comm range:** 3.0 m (unchanged; 3 consumers: `convergence_monitor`, `robot_node`, launch)
- **New spawns:** (0.86,0.28) (0.0,0.90) (-0.86,0.28) (-0.53,-0.73) (0.53,-0.73), all yaw 0
- **RTF:** ~1.0 baseline (physics + 5 lidars); ~0.90 with 1 Nav2 stack
- **Coverage reached:** rung1d ~19.7% of world in 110 goals before health-guard halt
- **Test count:** 116 (all green)

---

## 6. PROTOCOL REMINDERS (unchanged, worked well this session)

- Claude explains what/why before code; Claude Code edits + builds locally (accept-edits);
  Ali runs terminal commands and pastes output for diagnosis.
- Runtime verification before every commit — never on static checks alone. This session's
  two most important bugs (frozen clock, spawn geometry) were **invisible until measured**;
  green tests did not catch them.
- Explicit `git add <file>` only. Commands labeled with sim-state requirement + terminal.
- **`colcon build` must run from `ros2_ws/`, not the repo root** — root cwd caused two
  spurious "package failed" scares this session.
- `tee` truncates — use distinct log names per run (`explore_rung1X.log`).
