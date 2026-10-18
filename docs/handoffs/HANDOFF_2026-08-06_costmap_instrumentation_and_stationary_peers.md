# HANDOFF — 2026-08-06
## The explorer now reads the planner's costmap; peers were stationary in every prior run

---

## 1. Session outcome in one paragraph

Three commits landed on `main` (unpushed, held at your instruction) replacing
reconstruction with direct measurement: the explorer subscribes to the global
costmap on both the translated and raw scales, derives its halt verdict from
the readings rather than a fixed sentence, and dispatches a displacement goal
when the verdict is an inflation pocket. All three were verified at runtime
before landing. The escape then fired four times in `rung3b` and cleared the
pocket every time (raw 253 → raw 0). Separately, and more consequentially,
`wander` was found to default to `false`: robots 1–4 held their spawn poses in
`rung2f`, `rung2g`, `rung2h` and `rung2i`. The peer-residue mechanism recorded
in the 2026-08-03 handoff cannot have operated in any of those runs, and their
convergence figures describe five robots sitting in a 0.9 m pentagon looking at
the same scene.

The day's method lesson: every diagnosis built on log-string greps or offline
reconstruction from a saved PGM failed. Every diagnosis built on a direct read
of a config file, a source file, or the costmap topic held.

---

## 2. Commits (main, NOT pushed)

| Hash | Title | What it does |
|---|---|---|
| `a3cf9b5` | feat: read the planner's costmap instead of reconstructing it | Four latched (transient-local, depth 1) subscriptions: `global_costmap/costmap` (`nav_msgs/OccupancyGrid`, translated −1..100), `costmap_raw` (`nav2_msgs/Costmap`, raw 0..255), and both update topics. Separate geometry per topic. `cost_at(x, y)` returns cost, name and raw; out-of-bounds returns an `OFF_COSTMAP` sentinel. Diagnostic only. |
| `193684e` | fix: derive the halt verdict from the readings, not a fixed sentence | `_halt_classify` → `HaltReading`, computed once and shared by the logged sentence and (later) the escape trigger. Four branches: pose error, inflation pocket, neither, and no-verdict when the raw costmap was never received. Readings print alongside the verdict. |
| `8ed0123` | feat: dispatch a displacement goal when the halt verdict is an inflation pocket | `_escape_target` samples 16 headings off the raw costmap, disqualifies a ray on a LETHAL or off-grid sample, scores the rest by summed cost, ties break to the lowest heading index. Fires on `VERDICT_POCKET` only, in the DEFER branch, ahead of the defer cap. `ESCAPE_DISTANCE = 0.6`, `MAX_CONSEC_ESCAPES = 3`, `EXIT_ESCAPE_EXHAUSTED`. |

Test count 309 → 334 across the explorer suites. All three mutation-checked
rather than only green.

Nine commits are now local and unpushed: these three plus the six from
2026-08-03.

---

## 3. THE BIG ONE — peers were stationary in every prior run

`swarm_sim.launch.py` line ~370:

```python
DeclareLaunchArgument(
    'wander',
    default_value='false',
    description='Enable the legacy dot-era wander driver on every '
                'robot node (opt-in; off by default, so robots hold '
                'their spawn poses unless another controller drives them)',
)
```

No launch command in `rung2f`, `rung2g`, `rung2h` or `rung2i` passed
`wander:=true`. Robots 1–4 held their spawn poses — a 0.9 m regular pentagon —
for the whole of each run.

**What this invalidates:**

- The peer-residue mechanism of the 2026-08-03 handoff §3 ("a peer wanders
  within lidar range, slam_toolbox writes it in, the peer moves away, the mark
  is never cleared"). A stationary robot is a legitimate static obstacle,
  correctly mapped and correctly persistent. Not a phantom.
- The reading of the blob trail as "one peer driving past, stamped at three
  successive scan integrations". Robot_0 drove past *stationary* peers; the
  trail is more likely its own view of one robot from successive angles, or
  SLAM inconsistency across viewpoints.
- The convergence numbers from those runs (mean pairwise similarity ~0.93).
  Five robots at 1 m spacing see nearly the same scene, so near-identical
  inputs produce near-identical graphs. That is not the viewpoint-dependent,
  occlusion-shaped divergence Phase B exists to demonstrate.

**What survives:** the inflation pocket itself. `rung2h`'s `raw=253` at the
robot's own cell was a direct measurement and stands. Its cause was robot_0
driving too close to a *stationary* peer, not residue from a moving one.

**Effect size:** with `wander:=true`, `rung3b` hit four pockets in about six
minutes. `rung2i` hit zero in 78 goals.

---

## 4. What the costmap subscription established

### The start was never lethal

`rung2h` halted with the robot's own cell at `raw=253 INSCRIBED_INFLATED`, not
254. Navfn expands from 253 — which is exactly why the start probe honestly
returned `START_OK` on every cycle while every distant candidate failed. The
2026-08-03 handoff §3 ("the start cell is lethal, every plan impossible") is
wrong.

### The translated scale cannot answer this

Same cell, same instant: translated `99 COST`, raw `253 INSCRIBED_INFLATED`.
Nav2 maps 254 → 100, 253 → 99, 255 → −1 for visualisation, so 99 is ambiguous
between "inscribed" and "ordinarily expensive". Subscribing to both was the
difference between a measurement and a shrug.

### Reach limit was pocket-specific, not a property of 253

`rung2h` measured plans succeeding at 0.43–1.81 m from the robot and failing
from 0.66–6.34 m (2 of 30 failures inside the success range, n=44 readings).
In `rung2i`, targets at `raw=253` planned successfully at `poses=352, 351, 254,
394`. So the reach limit described that one pocket's geometry, not inscribed
cells in general. `ESCAPE_DISTANCE` is parameterised for this reason and
carries a provenance comment saying so.

---

## 5. The escape, validated 4/4 in rung3b

| # | Pose before | Moved | Cost before → after |
|---|---|---|---|
| 1 | (0.05, −1.01) | 0.39 m | 253 → 0 FREE |
| 2 | (−0.06, −1.04) | 0.43 m | 253 → 0 FREE |
| 3 | ~(−0.5, −1.1) | 0.49 m | 253 → 0 FREE |
| 4 | (−5.08, −0.10) | 0.37 m | 253 → 0 FREE |

Every one `nav2=SUCCEEDED`, every one `escape #1` (so the counter reset
correctly on intervening successful goals).

Displacement clustered at 0.37–0.49 m against a 0.60 m target: Nav2's
`tolerance: 0.5` stops the robot short. The margin is doing real work — tuning
the distance down to the observed 0.4 m would put actual displacement near
0.25 m and might not clear.

Escape #1's ray data: `cheapest of 7/16 drivable directions`, samples
`253,234,208,189,169,0,0,0,...`. Nine rays disqualified, cost falling
monotonically along the chosen heading into free space.

Escapes 1–3 fired within 67 s near (0, −1.3): the robot escaped, re-approached
the same frontier, and re-entered the same pocket. The escape resolves the
symptom but does not stop the re-entry.

### The other branch fired correctly too

`rung3b`'s actual halt was **not** a pocket: SLAM `value=100` (occupied), raw
254 LETHAL, 9/9 of the 3×3 at 253–254, nearest obstacle 0.020 m.

> Verdict: pose error is plausible — the believed pose is inside mapped
> structure (SLAM value=100 (occupied), raw cost=254 LETHAL, 9/9 of the 3x3
> at 253-254).

The escape correctly did **not** fire. Displacement does not fix a pose error.
First real encounter with that branch; branch discrimination works.

---

## 6. Corrections to beliefs held during this session

Recorded because several were mine and were held confidently:

- **"The precheck starves the BT of dispatches, so recoveries never fire."**
  False. Recovery rates per goal were comparable (ON 0.375, OFF 0.44).
- **"`planning_time=0.000s` means navfn rejected before searching."** False.
  The *successful* start probe shows the same value; the field is never
  populated.
- **"The OFF control may be an unbacked number."** False. It exists as
  `sim_rung2e_off_explore.log`: 127 SUCCEEDED of 172, 73.8%. It was mislabelled
  "rung2f" in the previous handoff's run table.
- **"`ndimage.label` says the robot cell is blocked."** A script bug —
  `label` reserves 0 for background, so the "start truly blocked" branch was
  printed without ever running the test. It would have re-confirmed a
  conclusion already retracted.
- **"Goals on unknown cells are rejected outright."** False. Unknown-cell goals
  both succeeded (`poses=62`, `poses=36`) and failed (`poses=0`).
- **"The frontier goal is offset past the boundary."** False. Goal and frontier
  coordinates are identical in every log line.
- **"`rung2i` is in a permanent livelock."** Overcalled. It was a shuttle
  *phase*; the run broke out on its own once the map changed.

---

## 7. Verified environment facts (do not re-derive)

- `nav_controlled` only **mutes** a robot's wander output. It does **not**
  launch Nav2. Nav2 comes up with `explore.launch.py` — one stack per explorer.
  `swarm_sim.launch.py nav_robots:=[0,1,2,3,4]` brings up zero planner servers.
- Per-robot configs exist **only for robot_0**: `experiments/nav/` contains
  `nav2_robot0.yaml` and `slam_robot0.yaml` and nothing else. Launching an
  explorer for robot_1 fails immediately on missing `slam_robot1.yaml`.
- The launch file's `nav_robots` list parsing is correct —
  `PythonExpression([str(robot_id), ' in ', LaunchConfiguration('nav_robots')])`
  genuinely handles `[0,1,2,3,4]`.
- **4 cores.** Load average sat at 19–30 throughout, `%Cpu(s) 0.0 id`.
- Top consumer is **`nsk_engine` at ~106 % CPU and 1.9 GB RSS** — not Nav2
  (`bt_navigator` 18.8 %), not Gazebo (~31 % across two ruby processes). This
  explains the constant `/nsk/merge` and `/nsk/compress` timeouts.
- One Nav2 stack costs ~500 MB RAM and almost nothing on the GPU. GPU sat at
  ~600 MiB / 4096 and 11–35 % utilisation. **The 4 GB card is not the binding
  constraint; the CPU is.**
- Closing Chrome freed 2 GB RAM and dropped GPU to 11 % but did **not** reduce
  load average.
- The wander driver is **open-loop**. `robot_node.py` contains no reference to
  `scan`, `obstacle`, or `avoid`. Peers execute a blind random walk and collide
  with walls; the physics engine and the stuck detector sort out the rest.
- `robot_node`'s stuck detector fires on *commanded but not moving*, so it can
  never fire for a Nav2-driven robot whose explorer refuses to dispatch.
- `/global_costmap/costmap` publishes the **translated** −1..100 scale
  (`costmap_2d_publisher.hpp:162,180`), not raw. `nav_msgs/OccupancyGrid.data`
  deserialises as signed `array('b')`; `nav2_msgs/Costmap.data` as unsigned
  `array('B')`.
- `costmap_updates` is silent under the current config —
  `always_send_full_costmap: True`. The update path is implemented and unit
  tested but has never run against live data.
- Global costmap plugins: `["static_layer", "obstacle_layer",
  "inflation_layer"]`. Local: `["voxel_layer", "inflation_layer"]` — **no
  static layer**. `footprint_clearing_enabled` appears nowhere in the file, and
  the static layer has no footprint-clearing mechanism at all. Obstacle-layer
  fixes cannot touch a SLAM-map phantom.
- There is a dead `static_layer:` config block inside the local costmap section
  (line ~189) that is absent from its plugin list.

---

## 8. Open items, in priority order

1. **Per-robot nav/slam configs for robots 1–4.** Without them no robot but
   robot_0 can explore, and Phase B's five divergent maps cannot be produced.
   Probably parameterisable from the robot_0 versions rather than five hand
   written copies.
2. **Decide the multi-robot strategy.** Five simultaneous Nav2 stacks are
   plausible on RAM but questionable on 4 saturated cores. Options: sequential
   runs (loses live sharing during exploration); one navigating + four
   wandering, rotated across five runs (keeps live sharing, needs item 1);
   reduce `nsk_engine` load; or accept a degraded real-time factor — noting the
   TF tolerances were tuned against current timing.
3. **The truncated-STACK exit message is still wrong.** It advised raising
   `PRECHECK_MAX_PROBES` in `rung3b` while the actual fault was the pose. Open
   item 2 from the 2026-08-03 handoff, still unfixed.
4. **Pocket re-entry.** The escape clears the pocket but the robot
   re-approaches the same frontier and re-enters. Three times in 67 s in
   `rung3b`.
5. **The shuttle phase.** `rung2i` goals 47–59 alternated between (0.20, 3.34)
   and (3.93, −2.32), 6.3 m apart, ~50 s each, all SUCCEEDED,
   `blacklisted=False`, frontier shrinking 114 → 113 cells. Twelve goals for
   one cell of progress. Suspected cause: `tolerance: 0.5` lets Nav2 report
   success ~10 cells short of the frontier at 0.05 m resolution.
6. **`explore()` returns `True` for both "nothing left" and "nothing
   reachable".** Carried over; the escape's placement ahead of the defer cap
   mitigates one path into it but does not fix it.
7. **Uncharacterised: `INCONCLUSIVE` with `error_code=0` and successful
   plans.** `triage()` is documented to return it for 203/205; this arrives by
   some other path.
8. **Uncharacterised: a candidate at `raw=254 LETHAL` returned `REACHABLE`
   with `poses=394`.** Planning to a lethal goal cell should not succeed;
   `tolerance: 0.5` may be terminating the path near the goal rather than on it.
9. **The `/nsk/merge` and `/nsk/compress` timeouts.** Convergence moves in
   discrete steps (0.0000 for stretches, then +0.0050) because merges land only
   when the engine keeps up. Characterise before any convergence figure goes
   into a paper.

---

## 9. Explicitly NOT verified

The previous handoff's confident claims cost most of a day. This section exists
so the next one costs less.

- **The escape has never fired on a genuine peer-residue phantom.** All four
  `rung3b` pockets formed while peers were moving, but whether the inflation
  came from a peer, a wall, or both was never established per-escape.
- **Five simultaneous Nav2 stacks have never been launched.** Everything in §7
  about hardware headroom is extrapolation from a single stack.
- **The blob analysis from 2026-08-03 has not been redone** against a run with
  moving peers. The twenty small components in that PGM are still unexplained
  under the corrected understanding of §3.
- **`costmap_updates` row-splice path has never run against live data.**
- **No multi-robot map divergence has been measured since 22 July**
  (`phaseB_run1`, mean off-diagonal Jaccard 0.039), and that measurement was
  taken with peers stationary.
- **The `rung2i` shuttle's tolerance hypothesis is untested.** Nobody has
  compared where the robot actually stopped against the frontier coordinate.
- **Convergence figures from `rung3b` (0.9297 → 0.8219) are one run**, one
  navigating robot, four blind walkers.

---

## 10. Run history

| Run | Peers | Result |
|---|---|---|
| rung2e (OFF control, 1 Aug) | stationary | 172 goals, 127 SUCCEEDED (73.8 %), 4 × 208. Mislabelled "rung2f" in the previous handoff. |
| rung2f (ON, pre-commits) | stationary | Died at 13 goals. 43 × 208 from a frozen pose. No map saved. |
| rung2g (ON) | stationary | Died at 8 goals. Map saved. Start probe cleared the pose on every 208. |
| rung2h (ON, costmap subscription) | stationary | Halted at 14 goals. **`raw=253 INSCRIBED_INFLATED` at the robot's own cell** — the measurement the whole session was after. |
| rung2i (ON, escape committed) | stationary | Killed manually at 78 goals, 57 SUCCEEDED (73 %). No pocket formed, escape never fired. Shuttle phase at goals 47–59. |
| rung3b (ON, `wander:=true`) | **moving** | Halted at 21 goals. **4 escapes, all 253 → 0 FREE.** Halt was a pose-error verdict, correctly not escaped. Explorer process died, exit code 1. |

Archives: `experiments/runs/rung2h/`, `experiments/runs/rung2i/`,
`experiments/runs/rung3b/`, each with a README.

---

## 11. Working practice notes

- **Read the config, not the log.** Every diagnosis from log-string greps or
  offline PGM reconstruction failed today. Every diagnosis from a direct read
  of a YAML, a source file, or the costmap topic held.
- **An empty grep is not evidence.** Wrong file, wrong topic name, QoS
  mismatch, and "the thing genuinely never happened" all present identically as
  silence. Three separate hours were lost to this. Always run a positive
  control against a case known to be present.
- **nav2's output goes to the explore log**, not the sim log — Nav2 launches
  with `explore.launch.py`.
- **Normalise before comparing.** Raw recovery counts across runs of different
  length were meaningless; so were counts inflated by startup chatter
  ("Creating behavior plugin spin" is not a firing).
- Terminal convention: 1 = simulation, 2 = nodes, 3 and 4 = analysis.
- `colcon build` runs from `ros2_ws/`, never the repo root.
- Heredocs pasted into the terminal sometimes lose the closing `EOF`; verify
  with `tail` and `wc -l` after writing a README this way.
