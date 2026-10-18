# NSKsim — TurtleBot3 Swap Session Handoff

**Date of handoff:** 2026-07-17
**Repo:** github.com/AliAlhasan6/NSKsim (public, MIT, at `e62feb4`, tagged **v1.0.0**, release published)
**Local:** `/home/lawlite/Desktop/NSKsim` · workspace `ros2_ws/` · venv at `venv/`
**Models:** `/home/lawlite/Desktop/NSK/experiments/checkpoints/joint_best.pt` (external repo)

---

## Gap session: COMPLETE (all 6 items)

1. **Retention adaptivity FIXED** (`825584a`). Root cause: `peer_similarity` declared,
   never written. Robots now call `/nsk/similarity_query` (same `_call_engine`
   pattern as Compress, same ReentrantCallbackGroup) with `[own_id]+peers_in_range`
   before each share cycle; matrix parsed per the monitor's convention (rows follow
   request order). On failure: warn once, keep cached values (stale beats zero).
   Live-verified: retention sweeps 0.20/0.40/0.65 tracking min_sim.
2. **Seed parameter** (`7112392`). Nondeterminism traced to train DataLoader
   `shuffle=True` in NSK's `src/utils/data_loader.py:245` — `all_graphs[ds_idx]`
   indexes a shuffled iteration order (split itself already seeded at :239;
   ego-graphs cached to disk, not re-sampled). Fix: `seed` param (default -1 =
   unseeded) seeds random/numpy/torch in on_configure before AgentManager.
   Verified: two seed:=42 runs → identical node counts (330/5999/890/859/751);
   unseeded run wildly different (182/773/509/21/30). Lévy-walk motion
   deliberately remains unseeded (physical variability wanted for EXP-01).
3. **CSV export** (`7296b50`). `csv_path` param (default '' = disabled) on the
   monitor; one row per `_monitor_cb` cycle: timestamp, elapsed_sec, mean_sim,
   baseline, rise, converged, robot{i}_x/y from odom, pair_counts as `a-b:n;…`
   (cumulative). **flush() per row** — verified live by reading the file mid-run.
   Write failure disables export, never raises into _monitor_cb.
4. **Pipeline diagram** regenerated (old PNG only existed in chat) with gap-session
   state: adaptive retention as live mechanism, seed param, CSV export, 63 tests.
   At `docs/nsksim_pipeline.png`, embedded in README §Architecture.
5. **Run screenshot** `docs/nsksim_run.png` (Gazebo world + RViz similarity graph,
   cropped and stitched side-by-side), embedded as README hero image under the badge.
6. **v1.0.0 tagged + GitHub release published** with Phase A notes.

**Test suite: 63** (55 → +3 adaptivity, +2 seed lifecycle, +3 CSV).

## OPEN FLAG (resolve first, one command)

- CI green on `e62feb4` was confirmed only via badge (GitHub REST API was in a
  503 incident all session; push/tag/release worked, runs API didn't). Run
  `gh run list --branch main --limit 3` once the API recovers and confirm the
  run on e62feb4 succeeded before building on it.

## Research ledger addition

- Seeded run seed:=42: from ~t=140s pair (0,2) similarity DECAYED monotonically
  (0.94 → −0.06) while merging every cycle, both robots parked at 2.41 m in
  FLOCK; adaptive retention responded correctly (0.20 → 0.65 as sim fell).
  Meanwhile pair (1,4) locked at 0.40 m, sim ≈0.82 stable. Similarity decay
  under repeated merging is a real phenomenon, now measurable via CSV —
  candidate EXP-01 observable, possibly merger-bypass/embedding-blend related.
  Not a bug; do not "fix". Note only.

## This session's scope: TurtleBot3 swap (decision already made)

- Stock TB3 burger SDF. Lidar plugin PRESENT but UNCONSUMED (nothing subscribes
  to /scan) — consuming it is Phase B, not this session.
- Expect behavior re-tuning: TB3 max 0.22 m/s vs dot velocities; comm_range and
  possibly world size.
- Ends with tag v1.1.0. Same commit rhythm: one commit per coherent step,
  runtime verification before each commit, CI green.

## Explicitly OUT of scope (unchanged from gap handoff)

- Perception / knowledge-from-environment — Phase B founding architecture.
- EXP-01 execution — instrumentation now exists; EXP-01 is its own session.
  Design doc: `experiments/designs/EXP-01_...md`.
- Merger reactivation (loaded but bypassed, OOD on compressed graphs) — ledger.
- FLOCK parking equilibria — controlled variable in EXP-01, not a bug.
- Cosmetics (rosout shutdown line, SDF ambient warning) — permanent "not worth it".

## Working protocol (unchanged)

- Explain WHAT and WHY before any code. Code → Claude Code; scaffolding, builds,
  verification, git → in chat with output pasted back.
- Diagnose before fixing. Commit only after runtime verification.
- One or two code blocks per message when Ali is executing.

## Hard-won context — additions from this session

- **`ros2 launch ... | grep` shows NOTHING** — pipe buffering. Redirect to a file
  (`> /tmp/run.log 2>&1`) and grep the file after Ctrl-C. AgentManager print()
  lines flush only at shutdown either way.
- **Launch args need type pinning**: bare LaunchConfiguration values get
  yaml-parsed as strings; use `ParameterValue(LaunchConfiguration(x),
  value_type=int/str)` (seed and csv_path both do this — copy the pattern).
- **`git add` on an unmodified file stages nothing, silently.** The README-embed
  commit initially shipped images without the README edit (grep found no embeds,
  commit showed 0 insertions). Verify with `grep` + `git status --short` before
  committing; `--amend` is fine while unpushed.
- **GitHub REST API can 503 while web infra works**: badge endpoint
  (`.../badge.svg?branch=main | grep -oE "passing|failing"`) and pushes/releases
  kept working through the outage. Badge reflects latest COMPLETED run — only
  trustworthy for a specific commit after enough time has passed for its run to
  be the completed one.
- Pre-existing carryovers still in force: env is PER TERMINAL ("Package
  'nsk_swarm' not found" = you forgot to source); pgrep before every launch;
  `ros2 run` bypasses venv; never shadow rclpy Node internals; call_async +
  ReentrantCallbackGroup pattern; catch-don't-check shutdown; extras_require
  not tests_require; gh over HTTPS (SSH blocked), older gh flags.

## Session opening move

1. Resolve the CI flag: `gh run list --branch main --limit 3`.
2. `pgrep -af "nsk_engine|robot_node|convergence"` — kill leftovers.
3. Diagnosis before code: locate the current robot SDF and spawn path —
   ```bash
   grep -rn "sdf\|spawn" ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py | head
   ls ros2_ws/src/nsk_swarm/worlds/ ros2_ws/src/nsk_swarm/models/ 2>/dev/null
   ```
   Read how robots are currently described/spawned, THEN plan the TB3 swap
   (SDF source, namespacing, velocity limits, comm_range) before any edits.
