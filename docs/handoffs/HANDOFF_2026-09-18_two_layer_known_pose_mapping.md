# HANDOFF — 2026-09-18 — Two-layer design: mapping with known poses

## 1. What this session was

Brainstorming. **No simulator was run, no code was written, no commit was
made.** The previous handoff (`2026-09-14`) left B2 unblocked and the designed
calibration run owed. This session asked a different question: the maps
themselves are defective, so what feeds B2 at all?

The output is a design decision and one cheap offline test that decides whether
it holds.

## 2. The map defect, stated correctly

Two restatements were needed this session, because the loose version leads to
the wrong fix.

**It is not drift between maps. It is drift inside one map.** A single robot's
b16 map shows the same wall in two or three copies rotated 10–40° about a common
point. No rigid placement scores more than one copy — best fit `robot_2` at
49.3 % against the 80 % `WELL_FIT_FLOOR`. Robot-to-robot matching is a later
problem; this one breaks a map on its own.

**The driver is accumulated rotation, not runtime.** The four parked robots in
`b18_run2` show exactly 0.00° of odometry yaw error over 96 508 samples. Wheels
that do not turn do not drift. Robot_0 in b16 turned ~119 revolutions; robot_1
turned 14 090°.

**It is not a bias you can divide out.** Summed |excess| over 10 s chunks is
3264.5° against a net of −629.0° in run2, and 796.4° against +41.0° in run3.
Sign-alternating. It is a random walk with a small drift, which is why the
copies sit at different angles rather than at one consistent skew.

Mechanism (from B1.8, closed): the wheels reach commanded speed ~420 ms before
the body does, and turn against a body that cannot follow. Per scan the error is
~11° at Nav2's 1.00 rad/s cap, far inside the ±20° search window, so no single
scan match ever fails. It compounds, and loop closure never absorbs it.

## 3. Options considered and rejected

Record these so they are not re-proposed.

**A learned map-matching model.** Rejected. Matching is not the failure. A
matcher finds correspondences; it cannot decide which of three rotated copies of
a wall is the true one. That needs a correct prior, and b16 has none — ground
truth was first recorded in b18. There is also a reviewer problem: a learned
corrector writes an assumption about the world into the quantity Article 3
measures.

**Widening the scan-match search window.** Rejected as stated, but with a
surviving variant. `coarse_search_angle_offset` is already measured as
non-binding: odom yaw rate caps at exactly 1.00 rad/s, ~11° per scan, and
`scan > search` came back zero for all five robots. Widening it costs search
volume and buys nothing. **The window that could bind is loop closure**, not
scan matching — `loop_search_maximum_distance` and the loop-closure angle
search. After 10–40° of accumulated drift a revisited corridor is displaced by
metres and never enters the candidate chain. That variant is testable offline on
the b16 bag, with the 80 % fit floor as the verdict; the risk is false closures,
which can make a map worse. Kept as a fallback, not the main path.

**A run with less spinning.** Rejected on realism. A real robot turns a lot.
Reducing rotation to make the measurement work would make the simulation less
like the thing being simulated.

**A fully pre-built map unlocked cell by cell as the robot explores.** Proposed
this session, rejected on research grounds rather than engineering ones. It is
defensible against §1.6, which already assumes perception correctness is not in
question — it would instantiate an existing assumption, not smuggle in a new
one. But it collapses §3.3 of the B2 schema: if every robot reveals cells of one
identical grid, two robots that saw the same wall agree bit-for-bit, divergence
reduces to coverage difference, and weighting corroboration by viewpoint
diversity has nothing left to weight. The subjectivity Article 3 measures would
be an artefact of who went where.

## 4. The decision — two layers

**Occupancy grid mapping with known poses.** Feed ground-truth pose to the
mapper instead of odometry; keep the real scans.

- **Locked layer:** the pose frame. Exact, no drift, no duplicated walls.
- **Open layer:** perception. Real occlusion, range noise, grazing incidence,
  limited field of view. Maps still differ where the viewing geometry differed.

The rule the choice follows, and the one to apply to every later question of
this kind: **lock the layer you are not studying; leave open the layer you are.**
The mapping substrate is not the contribution. Divergence from observation
geometry, and how a group reconciles it, is.

**The failure mode to watch:** locking one layer too many, until convergence is
guaranteed by construction and the result is a tautology. The pre-built-map
option in §3 is exactly that failure, one step further along the same road.

**A second condition falls out for free.** The same world, same scans, same
episodes, mapped once from truth and once from odometry, gives a controlled
comparison of what odometry error alone does to divergence. That is a result, not
a diagnostic.

## 5. First action next session — the offline test

Cheap, offline, no Gazebo. `b18_run2` has robot_0's map and all five
ground-truth poses at 16.6 Hz over 2329 s.

1. Rewrite the `odom → base_footprint` transform in the bag from
   `/model/robot_0/pose`.
2. Replay through the pipeline that is already proven:
   `strip_bag_for_offline_slam.py` → `run_offline_maps.sh` →
   `fit_world_transform.py --run … --spawn-rev …`.

**Predictions, written before the run:** the duplicated walls collapse, and the
fit clears the 80 % floor. If it does not, drift was never the whole story and
the two-layer design does not yet have its evidence.

**One caveat to check in source before relying on the result.** slam_toolbox
still runs its own scan matcher over whatever poses it is handed, so a pass does
not by itself prove the poses were used as given. Check whether this version
exposes `use_scan_matching` (and what it defaults to) rather than assuming it.
If the matcher cannot be disabled, the honest version of the locked layer is a
standalone log-odds occupancy mapper consuming scans plus true poses — no
matcher, no loop closure, deterministic, and small. Treat the slam_toolbox
replay as the cheap first probe and the standalone mapper as the layer that
ships.

## 6. The open problem this does not solve

**b16 cannot take this route.** It has no ground truth. Five per-robot maps are
what B2 needs, and producing them means a new five-robot run — which is not
available on this machine, since five Nav2 stacks were abandoned at stack one.

The escape is **sequential single-robot runs in the same world**: one robot at a
time, each recording truth, merged afterwards into a five-robot map set. It buys
the maps. It costs phantom obstacles and every robot-to-robot interaction
effect, both of which are real phenomena this project has previously insisted on
keeping.

That trade is a deliberate decision, not a default. Make it explicitly and
record the reason.

## 7. Carried forward unchanged

- Any statistic referenced to ground truth is deferred until the designed
  calibration run of `HANDOFF_2026-09-14` §6 exists. That run is owed, not
  optional.
- Repo hygiene, ten minutes: land `SPEC_b2_steady_gate.md` in `docs/specs/`,
  commit it, push the three commits sitting on `main` (`2b3d115`, `796a7ee`,
  `7402446`).
- The two decay logs exist in one place on this machine; `experiments/logs/` is
  gitignored.
- B2's schema work (§3.2–3.5 of `HANDOFF_phaseB2_schema_entry.md`) is untouched
  by any of this. Divergence is robot-to-robot and never references truth.

## 8. Paper logged

`2606.24489v1` — DRAN, Zhao et al., 23 June 2026: decentralized Riemannian
approximate-Newton pose graph optimization for object-based multi-robot SLAM.
**Not a fix for the map defect** — it is a back-end solver over loop closures
that already exist, and their real experiment relies on AprilTags for cross-robot
data association and motion capture for initial frames. Relevant instead to B2
and B4: objects as long-lived shared public variables against private
trajectories is close to the wall/opening/corner schema, and they deliberately
decouple the communication graph from the measurement graph, which is the
3 m `comm_range` choice. Note the project copy is a zip of page images with a
`.pdf` extension.
