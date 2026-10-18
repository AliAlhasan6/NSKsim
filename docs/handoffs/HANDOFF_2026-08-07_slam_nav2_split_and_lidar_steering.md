# HANDOFF — 2026-08-07
## SLAM and Nav2 split apart; wandering peers now steer off lidar and stop grinding into walls

---

## 1. Session outcome in one paragraph

Seven commits landed and were pushed (`bb3a88e..d0d45d2`, then `d0d45d2..4d7ee22`),
clearing a four-day backlog of unpushed work along with them. The architectural
result: SLAM and Nav2 are separable, and `explore.launch.py nav2:=false` now
brings up a mapper-only robot — validated at runtime, four `async_slam_toolbox_node`
processes and no planner. The empirical result: five SLAM instances cost roughly
5 % CPU and 62–70 MB RSS each, an order of magnitude under one Nav2 stack, so the
five-mapper configuration is comfortably affordable on this hardware. The
behavioural result: peer robots were grinding along interior maze walls, invisible
to both the `near_wall` pose guard and the stuck detector, and now steer off lidar
with `obstacle_stop_m:=1.2` — verified visually with all five robots clear of every
wall. A motion pre-flight was built, calibrated against real runs, and proven to
discriminate in both directions. **But a second look three minutes later showed
the robots clustered together again**, which is a separate and more consequential
problem than wall contact, and which blocks the divergence run. See §8a.

The day's method lesson, again: `ros2 param set` reports success and reads back the
new value while changing nothing. Four separate defects this session were checks
that looked load-bearing and were not.

---

## 2. Commits (main, PUSHED)

| Hash | Title | What it does |
|---|---|---|
| `363bb6a` | feat: add slam_robot1..4.yaml so robots 1-4 can map | Derived from `slam_robot0.yaml` by `s/robot_0/robot_N/g`. Only four keys are robot-specific: `odom_frame`, `map_frame`, `base_frame`, `scan_topic`. Body byte-identical, verified by index-normalised diff. Each file carries a provenance header. |
| `c1424a5` | feat: gate Nav2 and the explorer behind a nav2 flag for mapper-only peers | `nav2:=false` → slam_toolbox alone. Gates the explorer too (FrontierExplorer is a BasicNavigator). SLAM ungated — it reaches `active` via this launch's own lifecycle events, not Nav2's manager. Default `true`. 334 → 335. |
| `ae33f9a` | chore: archive rung2h, rung2i and rung3b run artifacts | 4.4 MB, 13 files. Was untracked and single-copy. |
| `3a54abd` | feat: add a motion pre-flight so a stationary-peer run cannot start unnoticed | `experiments/preflight_motion.py`. 335 → 342. |
| `d0d45d2` | feat: give --min-path its measured default of 0.30 m | Calibration recorded in help text and comment. |
| *(hash not captured)* | feat: steer the wander driver away from what its lidar sees | Scan subscription, `_forward_arc_bins()`, three PARAMS entries, `obstacle` launch argument, `sensor_msgs` in package.xml. 335 → 362. **Hash was never read back — recover with `git log --oneline`.** |
| `4d7ee22` | feat: expose obstacle_stop_m as a launch argument | Default 0.6 (robot_node's own). 362 → 365. |

Test count 334 → 365. CI green on the first push (`31129272565`); the second
push's run was queued and **not yet checked**.

---

## 3. SLAM and Nav2 are separable — confirmed and shipped

`explore.launch.py` returns three independent top-level entities: the SLAM
`LifecycleNode` plus its two transition events, the Nav2 `GroupAction`, and the
explorer `Node`. Nothing passes between them. The load-bearing fact is at lines
161–170 of that file: Nav2's `lifecycle_manager` manages only the Nav2 servers,
**not** slam_toolbox, which is why this launch drives `configure`→`activate`
itself. Dropping the Nav2 block leaves a fully functional mapper.

Also established: `nav2_params` is read in exactly one place, inside the gated
include. **A mapper-only robot needs no `nav2_robotN.yaml`.** Open item 1 from the
2026-08-06 handoff is therefore satisfied for the sizing run by `363bb6a` alone.

**Runtime validation:** four `explore.launch.py robot_id:=N nav2:=false` launches
produced four `async_slam_toolbox_node` processes and nothing else — no
`bt_navigator`, no `planner_server`, no `controller_server`, no
`frontier_explorer`.

---

## 4. Hardware sizing — the CPU picture is not what §7 of the last handoff implied

Measured 2026-08-07 from `ps aux` and `top` on four mappers:

- **slam_toolbox: ~5 % CPU, 62–70 MB RSS per instance.** Against Nav2's
  `bt_navigator` alone at 18.8 % and ~500 MB per stack.
- **Four mappers cost ~9 percentage points of total CPU** (18.0 % user with, 8.7 %
  without, same desktop background).
- **Load average 1.89 with four mappers up**, against 19–30 with one Nav2 stack.
- **Total RAM is 23.9 GB, not a constrained figure.** 10 GB free with everything
  running. RAM was never the binding constraint.

**Caveat, important:** these readings were taken with the **sim down**, so
slam_toolbox was idling on no incoming scans. Under live lidar it will cost more.
Treat ~9 points as a floor. The order-of-magnitude gap against Nav2 is what
settles the architecture question, not the absolute number.

`nsk_engine` at ~106 % remains the top consumer and the binding constraint.
`/nsk/merge` timeouts appeared on four of five robots within two seconds of a
wander-only run — **no SLAM, no Nav2, no load**. Whatever is wrong there is not
contention.

---

## 5. The motion pre-flight

`experiments/preflight_motion.py`, run by hand between bring-up and a timed run.
There is no harness — every rung in this project has been launched by hand, which
was confirmed by searching the repo for scripts.

Two independent assertions per robot, reported separately because they indicate
different faults: message count above zero (bridge/namespace fault) and cumulative
path length above a floor (muted via `nav_robots`, `wander` false, or no
controller). Path length is summed inter-sample distance, not net displacement.
Sim time. Exit 1 on verdict failure, **exit 2 on wall-timeout** so "measurement
invalid" is distinguishable from "robots aren't moving".

**Calibration, 20 s sim windows, five Burgers:**

| Configuration | msgs | path_m |
|---|---|---|
| wander on | ~1016 | 1.175–1.505, and 2.809–3.084 in other windows |
| wander off | ~985 | **exactly 0.000** |

Stationary robots record exact zeros because Gazebo's `DiffDrive` publishes no
odometry noise for an uncommanded robot. The populations do not overlap; `0.30 m`
sits an order of magnitude from each.

**Verified in both directions at the operative floor:** wander on → five PASS,
exit 0. Wander off → five FAIL on `path>=floor` with `msgs>0` still passing,
exit 1. Also demonstrated discriminating *within* one swarm: with
`nav_robots:=[0]`, robot_0 read 0.000 while robots 1–4 read 2.1–2.7 m.

**Its known limitation, discovered the hard way:** wedged robots PASS. A robot
grinding against a wall accumulates path length because `DiffDrive` integrates
commanded wheel rotation. `path_m` 2.1–2.7 m looks identical for a travelling
robot and a pinned one. `net_m` is the discriminating column and is not asserted
on. **Do not treat a green pre-flight as evidence the robots are exploring.**

---

## 6. THE BIG ONE — peers were grinding along interior walls, and nothing could see it

Screenshot at 01:26 showed four of five robots pressed against walls.

**Why the stuck detector never fired.** `_check_stuck` returns true only when
world-frame displacement over 4 s is under `stuck_epsilon_m` = 0.05 m. A robot
*sliding* along a wall displaces far more than that. Measured: pinned robots
accumulated 2.1–2.7 m of path over 20 s, and a 227 s run produced **zero** `stuck at`
triggers. The detector is not broken — it detects *pinned and immobile*, and the
actual failure is *pinned and sliding*. **No odometric test can distinguish
sliding-along-a-wall from travelling.**

**Why the `near_wall` guard never fired.** `half = world_size/2 - 0.5` tests the
**outer boundary only**. `knowledge_world.sdf` puts the interior walls at y=±3.0
and x=±4.0, nowhere near it.

Consequence: the entire `_enter_recovery` / `_advance_recovery` / `_exit_recovery`
subsystem — reverse, rotate, repeat-escape escalation, FLOCK suppression — **has
never executed**. It is well-built dead code.

**The fix:** a scan term OR'd with the pose term in `_update_motion_state`. Not a
fallback — an independent OR, so a silently dead `/scan` cannot restore the old
behaviour unnoticed.

---

## 7. Lidar steering — implementation notes worth not rediscovering

- **The burger lidar sweeps 0→2π, not −π→π** (`model.sdf:135`, `min_angle 0.0`,
  `max_angle 6.28`). The ±90° forward arc is therefore **split across indices
  270–359 and 0–89**, not contiguous. Bearings are derived per sample and wrapped;
  a slicing implementation would read only the robot's right-hand side and still
  pass a naive "wall ahead" test.
- **Ties in bin openness break randomly.** Bins run right-to-left, so `max()`
  first-wins would turn every robot right out of every symmetric opening — and
  since every robot runs the same rule, that is a **correlated fleet-wide drift**,
  not a per-robot quirk. Tie tolerance `_OBSTACLE_OPEN_TIE_M` = 0.05 m, sized off
  the SDF's stated 0.01 m noise stddev — **a spec number, not a measurement**. If
  robots look indecisive in open space, that is the dial.
- `_OBSTACLE_ARC_BINS = 9` over ±90° → 20° bins → ~20 samples each at the burger's
  1° resolution.
- `scan_blocked` fires on **any** bin in the ±90° arc being under threshold. A
  robot travelling parallel to a wall has it in arc permanently. **Untested
  alternative:** test blockage over a narrow forward cone (±30°) while choosing
  the escape bearing from the full ±90°. First thing to try if coverage looks poor.
- Linear speed is **not** reduced when blocked — deliberate. Slowing keeps a robot
  near the wall longer and could push it under `stuck_epsilon_m`, firing
  recoveries spuriously. `obstacle_stop_m` is the intended dial instead.

---

## 8. `obstacle_stop_m` — 0.6 was too tight, 1.2 works

Geometry: at `walk_speed` 0.15 m/s and `walk_turn_max` 0.5 rad/s a robot needs
~3 s to turn 90° and covers ~0.45 m doing it. At 0.6 m it arrives with ~0.15 m
spare against a perpendicular wall and **none at an oblique angle**.

Observed progression, three screenshots:

| Time | Configuration | Result |
|---|---|---|
| 01:26 | no scan steering | four of five pinned where they first hit |
| 02:50 | `obstacle_stop_m` 0.6 | still touching, but distributed across different walls |
| 03:28 | `obstacle_stop_m:=1.2` | all five clear of every wall, spread across the maze |
| 03:31 | same run, 3 min later | **robots clustered together again — see §8a** |

Parameter confirmed at 1.2 on all five nodes at launch.

**The 03:28 frame was a transient, not a steady state.** Wall-grinding does appear
solved; clustering is not. Read §8a before treating this configuration as working.

**Open question:** whether 1.2 m of standoff costs map quality. A Burger sees
3.5 m, so an over-cautious robot in open space maps little. This is a question for
the maps, not for tuning blind.

---

## 8a. THE OTHER BIG ONE — the robots cluster, and FLOCK is the suspect

Two screenshots at 03:31, three minutes after the clean 03:28 frame, show the five
robots pulled together: in one, four of them within roughly a robot-length of each
other; in the other, two adjacent to the left interior wall with a third close by.

**Why this matters more than wall-grinding.** Clustered robots observe the same
scene, so they build near-identical maps. That is exactly the artifact that
invalidated `rung2f` through `rung2i` — five robots in a 0.9 m pentagon — now
arriving by a different route. Fixing locomotion does not help if the robots then
converge on each other. The convergence monitor reading mean pairwise similarity
0.9818 and rising at t=37 s is consistent with this.

**Mechanism — DISPERSE is too weak to break a flock, by a factor of two.**

The first hypothesis, that nothing ever enters `DISPERSE`, is **FALSIFIED**. It is
entered at line ~797 of `robot_node.py`, immediately after a successful broadcast
in `_share_timer_cb`:

```python
self._state          = DISPERSE
self._disperse_until = time.time() + 10.0
```

But the arithmetic does not work:

- DISPERSE lasts **10.0 s** (a hardcoded literal, not a parameter)
- `walk_speed` is **0.15 m/s** → **1.5 m of travel, maximum, and only if straight**
- `comm_range` is **3.0 m**

A robot therefore disperses at most **half** the distance it needs to leave the
flock, hits the timeout at line ~448, returns to `EXPLORE`, finds its peers still
in range, and re-enters `FLOCK` immediately. The cycle cannot terminate. Clustering
is not a failure of dispersal to fire — it is dispersal firing correctly and being
insufficient by design.

**Candidate fixes** (none implemented, none tested): raise the 10.0 s literal to
~20 s+ so dispersal outlasts the range; reduce `comm_range`; or end dispersal on
achieved distance rather than a timer. `comm_range` is the interesting one for the
thesis, since it also determines who can share with whom — changing it changes the
sharing topology, not just the locomotion.

**A second finding in the same subsystem — sharing requires proximity.**
`wander_diag2.log`, a 227 s run: **28 broadcasts**, against a naive ceiling of ~140
for five robots at `share_interval` 8.0 s. The cause is the first guard in
`_share_timer_cb` (line ~751):

```python
peers_in_range = [j for j in range(self.num_robots)
                  if j != self.robot_id and self._in_range(j)]
if not peers_in_range:
    return
```

**A robot with no peer within `comm_range` never shares.** The timer fires and
returns immediately. 28 broadcasts is therefore not a defect — it is the fraction
of the run robots spent close enough to have something to share.

Two earlier guesses about this are **FALSIFIED**: the merge timeouts are in
`_on_kg_share` (receive path) and cannot block anything, and `_call_engine` does
not starve the timer — its docstring records a `ReentrantCallbackGroup` under a
`MultiThreadedExecutor`, so a blocked callback does not serialise the next tick.

**What this implies for the thesis, and it is not a bug.** The design is a
deliberate cycle: approach → share → disperse → wander → approach. Sharing is
*gated on* proximity, and dispersal is the *consequence* of sharing. So
`comm_range` sets both how far apart robots get and how often they can agree.
Disperse them enough for genuinely divergent maps and they share less; keep them
close enough to share often and their maps converge on geometry alone. **Divergence
and agreement are coupled through a single constant.** That tension is arguably the
phase-2 research question showing up as a parameter, not something to tune away.
Decide deliberately before changing `comm_range`, the 10 s literal, or
`share_interval`.

**NOTE for whoever reads `robot_node.py` next:** the lidar work shifted line
numbers by ~150. Any line reference in this or the previous handoff predating
§7 is stale. Locate functions with `grep -n 'def _name'`. Current positions:
`_share_timer_cb` 748, `_call_engine` 827, timer registration ~336–342.

**Do not run the five-mapper divergence run until this is understood.** A run whose
robots spend it huddled together reproduces the exact defect the run exists to
escape, and it would take an hour to discover that from the maps.

---

## 9. Corrections to beliefs held during this session

- **"The stuck detector must be broken."** Wrong twice over. First I suspected the
  `any(speed <= 0.0)` guard via FLOCK — but `_publish_cmd_vel` never zeroes
  `linear.x` outside recovery. Then I suspected `_check_stuck` was never called —
  but it is called unconditionally at the top of `_update_motion_state`. The
  detector works correctly and is measuring the wrong quantity.
- **"`experiments/runs/` might be a regenerable working directory."** No — unlike
  `experiments/logs/` and `experiments/maps/`, it is a curated archive.
- **"Nothing ever enters `DISPERSE`, so FLOCK is a one-way ratchet."** False. It is
  entered after every successful broadcast. The real problem is that 10 s of
  dispersal at 0.15 m/s covers 1.5 m against a 3.0 m `comm_range` — half what is
  needed. Written into the handoff as a hypothesis before being checked; corrected
  within the same session.
- **"The merge timeouts block the DISPERSE transition."** False. Those are in
  `_on_kg_share`, the receive path. Only the compress-call early return skips the
  transition, and that fired twice in 227 s.
- **"`_call_engine` starves the share timer, so broadcasts are throttled by engine
  latency."** False. `_cb_group` is a `ReentrantCallbackGroup` under a
  `MultiThreadedExecutor`; a blocked callback does not serialise the next tick. The
  28 broadcasts are explained by the `if not peers_in_range: return` guard — a
  robot alone never shares.
- **Line numbers from before the lidar work are stale by ~150 lines.** Cost one
  wasted read that returned `_check_stuck` instead of `_share_timer_cb`.
- **`grep -c` returning 0 on `rung4a_sim.log`.** The file did not exist; the `tee`
  was never issued. The zero meant "no file", not "no triggers". Caught only by
  `ls`-ing first on the retry.
- **`ros2 topic echo` "does not appear to be published yet"** was discovery lag,
  not silence. The topic was publishing at 10 Hz.
- **`Publisher count: 2` on `/robot_N/cmd_vel`** is `ros_gz_bridge`'s expected
  bidirectional registration, not a competing controller. Confirmed with
  `--verbose`.

---

## 10. Verified environment facts (do not re-derive)

- **`ros2 param set` on `obstacle_stop_m` reports success and reads back the new
  value while changing nothing.** `__init__` copies parameters into instance
  attributes and there is no `add_on_set_parameters_callback`. Tuning requires a
  relaunch. This is a general property of every parameter in `robot_node.py`.
- `slam_robot0.yaml` differs from its siblings in exactly four keys.
- `pytest ros2_ws/src/nsk_swarm/test/` **requires the workspace sourced** or it
  dies at collection on `ModuleNotFoundError: nsk_swarm_interfaces`.
- 20 tests skip in the sourced suite; that count has been constant all session and
  predates today.
- `qos_profile_sensor_data` works for both `/odom` and `/scan` — reliable
  publisher to best-effort subscriber connects.
- The wander driver's forward speed schedule appears **identical across robots**:
  in one 20 s window four robots read `path_m` 3.081 and the fifth 3.084, while
  `net_m` spread 1.785–2.773. Only the angular component differs. Relevant to how
  much trajectory diversity wander can generate.
- `git --no-pager` is needed or `diff`/`log` output pages and truncates on paste.

---

## 11. Open items, in priority order

1. **Clustering (§8a) — blocks everything below it.** Mechanism identified:
   DISPERSE lasts 10 s, which at 0.15 m/s buys 1.5 m against a 3.0 m `comm_range`,
   so a robot re-enters FLOCK the moment dispersal expires. The fix is arithmetic,
   but the choice is not: sharing is gated on proximity, so `comm_range` sets both
   how divergent the maps get and how often robots can agree. **Treat this as a
   research decision, not a tuning task** — it is the phase-2 tension appearing as
   a constant. Robots that huddle produce identical maps, so the divergence run is
   void until it is settled.
2. **The five-mapper divergence run**, after item 1. Everything else it needs now
   exists. Terminal 1:
   `swarm_sim.launch.py wander:=true obstacle_stop_m:=1.2 nav_robots:=[0]`;
   pre-flight gate (robot_0 will correctly FAIL, peers must PASS); then
   `explore.launch.py robot_id:=0` and `robot_id:=1..4 nav2:=false`, one at a
   time watching load. **This produces the first five maps in the project's
   history from robots that both moved and mapped.**
3. **`nsk_engine`.** Top consumer at ~106 %, and `/nsk/merge` timeouts fire within
   two seconds on a wander-only sim. Open item 9 from the previous handoff, now
   with evidence that it is not a contention problem.
4. **Convergence figures remain unpublishable.** The monitor reported mean pairwise
   similarity 0.9818 and rising at t=37 s with robots ~2 m apart — a measure of
   geometry, not agreement dynamics. Compounded by the merge timeouts and now by
   §8a.
5. **Does 1.2 m standoff cost map coverage?** Answer from the maps.
6. **Pre-flight does not catch wedging.** Consider asserting on `net_m`, or on a
   net/path ratio. Note it would not catch clustering either — a huddle of robots
   circling each other accumulates path and net displacement quite happily.
7. **The narrow-cone alternative** for `scan_blocked` (§7).
8. **Recovery subsystem has still never executed.** Now by design — the scan layer
   should prevent contact — but it means `escape_reverse_m` and friends remain
   entirely untested against live behaviour.
9. Carried forward, all still open: truncated-STACK exit message; pocket re-entry;
   the `rung2i` shuttle phase; `explore()` returning `True` for two conditions;
   `INCONCLUSIVE` with `error_code=0`; `raw=254 LETHAL` returning `REACHABLE`.
10. **`phaseB_run1`'s Jaccard 0.039** was measured with peers stationary. Low, not
   high — the opposite direction from `rung2f–2i`'s 0.93. Different quantities, so
   not necessarily contradictory, but unexplained. Worth a look once a real
   divergence measurement exists to compare against.

---

## 12. Explicitly NOT verified

- **Five simultaneous SLAM instances have never run against a live sim.** The
  sizing numbers come from four mappers with the sim **down**.
- **No map has been built by a robot using lidar steering.** The 03:28 screenshot
  shows correct locomotion; nothing has been mapped with it yet.
- **No multi-robot map divergence has been measured since 22 July**, and that
  measurement had stationary peers. This is unchanged from the last handoff.
- **CI has not been checked for the second push** (`d0d45d2..4d7ee22`).
- **The obstacle-steering commit hash was never read back.**
- **The 0.05 m tie tolerance is a spec figure**, from the SDF's noise stddev, not
  a measurement against real Gazebo returns.
- **1.2 m is one observation**, from one screenshot after one two-minute run — and
  a second look three minutes later showed clustering (§8a), so even the wall
  result rests on a single frame.
- **The DISPERSE arithmetic (§8a) is derived from constants, not observed.** 10 s ×
  0.15 m/s = 1.5 m against `comm_range` 3.0 m is sound on paper, and both numbers
  were read from source, but nobody has logged a state transition or watched
  `_state` during a run to confirm the flock/disperse cycle behaves as the
  arithmetic predicts. **This is the one remaining unverified link in §8a** — the
  guards and callback groups around it have now all been read directly.
- **TF delay under five-mapper load has never been measured.** The tolerances were
  tuned against one-navigator timing; `tf2_monitor` during the run.

---

## 13. Working practice notes

- **The screenshot was the decisive instrument four times today.** Topic rates, QoS
  introspection, parameter read-backs and odometry tables all reported health while
  robots were wedged against walls; and it was a screenshot three minutes after a
  clean one that revealed the clustering. When the question is "is the robot doing
  the right thing", look at the simulator.
- **One frame is not a result.** The 03:28 screenshot was taken as confirmation
  that the run worked; 03:31 showed otherwise. Sample the simulator more than once
  before concluding, and prefer a frame late in a run over an early one.
- **A check that only ever fires one way looks like it works.** Four instances
  today: `_walk` ignoring conditions so the launch test passed for both values of
  the flag; a `math.inf` sentinel shadowing an `isfinite` guard so deleting the
  guard changed nothing; `test_blocked_on_left_steers_right` asserting a turn
  where holding course was correct, passing only because the implementation shared
  its bias; and 15 launch tests skipping silently on a missing ament index. **Run
  the mutation. Assert the non-empty case.**
- **Read the artifact, not the report.** Claude Code's summaries were accurate
  every time this session, and reading the diffs still caught the rightward tie-break
  bias, which nothing else would have.
- **`ls` the file before grepping it.** An absent file and an absent match are the
  same zero.
- Terminal convention: 1 = simulation, 2 = pre-flight/analysis, 3+ = inspection.
  4–7 = per-robot explore launches.
- Python pasted into bash produces a `>` continuation prompt. Ctrl+C, then verify
  with `git status` that nothing was written.
