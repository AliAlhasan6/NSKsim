# B2 maps — the five truth-driven explorer runs, k0 … k4

One run per robot, one at a time, never two at once. In run **k\<K\>** robot K
explores under Nav2 with its `odom → base_footprint` built from Gazebo ground
truth and slam_toolbox in known-pose mode; the other four hold their spawn
poses. K is the explorer id, 0 … 4, and it appears in the launch arguments, the
run name, the bag, and every offline command below.

This procedure is what rehearsal 5 actually ran. It supersedes
`experiments/logs/msgs/REHEARSAL_R1_R9.md` — see the last section for what
changed and why.

**Why these runs exist.** `b2maps_e0` explored on wheel odometry with the
matcher on: 138° median heading error, walls mapped as fans of rotated copies,
and the explorer declared exploration complete having never seen the outer
boundary. The pose layer is now known and the scans are not, so a bad map can
no longer be blamed on the odometry.

---

## Success criteria — fixed before k0 is started

A run counts when all five hold. Nothing here is judged by eye; each is a
number a script prints.

1. **COVERAGE ≤ 8.00 m.** The *true* trajectory passes within the lidar's
   maximum range of an outer boundary wall at least once. `run_health.py`
   reads the range from `BURGER_RANGE_MAX` in `swarm_sim.launch.py`, not from
   the stock model, so this threshold follows the sensor automatically.

   **This criterion is now weak, and criterion 5 carries the weight.** It was
   written against a 3.50 m ceiling, where e0's 3.897 m failed it and e0t's
   4.178 m would have too. At 8.00 m both pass, so COVERAGE no longer separates
   the runs that motivated it. It is kept because a trajectory that never comes
   within lidar range of the perimeter is still definitively bad — but passing
   it is no longer evidence that the map reached the boundary, only that the
   robot could in principle have seen it.
2. **R9's two gated counts are 0** — Nav2/tf2 transform warnings and
   slam_toolbox message-filter drops. e0's baseline is 0.000/min over
   98.96 min, so the limit is 0.000 and the first warning fails the run.
3. **R4 exact, inside the pose series**: 0 transforms falling *between* two
   pose samples, worst |dxy| ≤ 1e-6 m, worst |dyaw| ≤ 1e-6 deg, worst |z| = 0.

   The qualifier is the rule `e7e2abb` settled, and it is not a loosening.
   Transforms whose stamps land outside the recorded pose series are the
   **recording edge**, not a second publisher: rosbag2 subscribes to `/tf` and
   to `/model/robot_K/pose` at different moments, and whichever it gets first
   is a discovery race. `b2maps_relay1` recorded `/tf` 142.4 ms early and so
   carried 3 transforms ahead of its first pose — with worst |dxy| 0.000 m
   across the other 11 268. `b2maps_rehearsal5` won the race the other way and
   has none. Those transforms are admitted only as a contiguous prefix or
   suffix of the stream, within `EDGE_GRACE_PERIODS` (20) in both count and
   span, and only while the pose series itself is gapless — conditions a
   publisher that ran *during* the run cannot meet, because it would leave
   transforms inside the series where nothing is forgiven. Poses carrying no
   transform are printed and not gated; that is a rate question and R2 owns it.
4. **R5 median wheel-vs-truth |dYaw| > 1°** — `/robot_K/odom` must still be
   independent wheel odometry, or arm D cannot be built from these bags.
5. **The offline truth map has occupied cells on the outer boundary.**

R2 (one owner per frame pair), R3 (identity at spawn), R6 (`map → odom`
identity) and C1–C4 are preconditions, not criteria: if any of them fails the
run is not a run, and the four above cannot be read from it.

---

## Once, before k0 — build and sourcing

There are two install trees and they have different jobs. **`ros2_ws/install`
is a build-time dependency only** — it is the one place `nsk_swarm_interfaces`
is built, so the build needs it on the path. **`~/Desktop/NSKsim/install` is
the tree you run from**, and it is the only one sourced in the three terminals
below.

```bash
cd ~/Desktop/NSKsim

# BUILD — ros2_ws/install is sourced here and nowhere else, for
# nsk_swarm_interfaces. Use a throwaway shell so it cannot leak into a run.
bash -c 'source /opt/ros/jazzy/setup.bash
         source ros2_ws/install/setup.bash
         colcon build --packages-select nsk_swarm'      # from the REPO ROOT

# RUN — every terminal below starts with exactly these two lines.
source /opt/ros/jazzy/setup.bash
source ~/Desktop/NSKsim/install/setup.bash              # the repo-root install
```

**Do not source `ros2_ws/install/setup.bash` in a terminal you then launch
from.** That tree was built on 2026-09-10 and has no `truth_odom_tf` in it; the
repo-root install does. The failure is not always loud — sourcing the stale
tree puts an older `nsk_swarm` ahead of the one you just built, and the launch
either dies at 4 s with *executable not found* or runs the wrong node set.
Sourcing the repo-root install chains `nsk_swarm_interfaces` in on its own, so
a run never needs `ros2_ws/install` named a second time. Confirm before
launching:

```bash
ls install/nsk_swarm/lib/nsk_swarm/     # must list truth_odom_tf and
                                        # free_space_relay
```

The install holds **copies**, not symlinks (`--symlink-install` is not used),
so every edit to a launch file or a node needs that `colcon build` again before
`ros2 launch` can see it.

---

## The run — three terminals

Set the explorer id once per terminal:

```bash
K=0                                   # 0, then 1, 2, 3, 4 in later runs
RUN=b2maps_k${K}                      # NOT b2maps_e${K}t — e0/e0t are the controls
LOGS=experiments/logs/b2maps
```

Each command writes to a **log file only** — no `tee`. The logs are the record,
and a terminal pumping tens of thousands of lines competes for exactly the
time this run measures; on this rig RViz alone once collapsed `/clock` from
99 Hz to 0.5 Hz. Read the logs with `grep -a` (they carry raw bytes from child
processes and grep calls them binary without it).

### T1 — the simulator

```bash
ros2 launch nsk_swarm swarm_sim.launch.py headless:=true \
    nav_robots:=[$K] truth_odom_robots:=[$K] \
    > $LOGS/${RUN}_sim.log 2>&1
```

Both arguments, every run: `nav_robots:=[K]` mutes the wander driver for the
robot Nav2 will drive, `truth_odom_robots:=[K]` is what makes its transform the
truth. They are orthogonal — a truth-driven explorer needs both.

Wait ~15 s, then check **R3** from the log:

```bash
grep -a 'truth odometry for\|first transform' $LOGS/${RUN}_sim.log
```

Expect the anchor A to be robot K's own spawn, and the first transform to read
identity while it is still parked — rehearsal 5 printed `x=-0.0007`, well
inside `check_run_bag`'s 1 cm / 0.5°.

### T2 — the recorder

```bash
EXPLORER=$K RUN=$RUN experiments/slam/record_run.sh \
    > $LOGS/${RUN}_record.log 2>&1
```

Expect `pre-flight: all 25 required topic(s) advertised, on attempt N of 5`
(rehearsal 5: attempt 1). An abort after 5 attempts means a stack really is
down — fix it, do not retry blindly.

**Do not start T3 until the recorder has 26 subscriptions:**

```bash
grep -ac 'Subscribed to topic' $LOGS/${RUN}_record.log     # must print 26
```

26 is the 25 required topics plus `/robot_K/cmd_vel`, whose publisher
`robot_node` creates unconditionally. The last two — `/robot_K/map` and
`/robot_K/map_metadata` — only exist once the explorer brings slam_toolbox up,
and rosbag2's topic discovery (on by default, 100 ms) picks them up then,
taking the bag to 28.

### T3 — the explorer

```bash
ros2 launch nsk_swarm explore.launch.py robot_id:=$K scan_matching:=false \
    free_space_relay:=true \
    > $LOGS/${RUN}_explore.log 2>&1
```

`scan_matching:=false` is what makes this a known-pose run: slam_toolbox never
calls `MatchScan`, every scan keeps the pose it was handed, and `map → odom`
stays identity. It is only sound because T1 supplied truth poses.

`free_space_relay:=true` (`explore.launch.py:196`) is what makes the 8 m sensor
worth having. Karto's `OccupancyGrid::AddScan` (`Karto.h:6169`) skips any
reading at or above the maximum range — `+inf` included — with no ray trace, so
a beam that returns nothing clears no floor; 57.4 % of `b2maps_e0t`'s 11.7 M
beams were `+inf`. The relay republishes `/robot_K/scan` as
`/robot_K/scan_free` with those beams filled just above the range threshold, so
Karto traces them as free space. Range alone sets how far the box can reach;
the relay decides whether open floor inside it becomes known. Neither works
without the other, and `b2maps_relay1` is the first run with both.

Nav2's costmaps stay on the raw `/robot_K/scan`. The fill value means
something only against Karto's `rangeThreshold`; an obstacle layer would read
it as a return and put an obstacle there. Nav2 has its own knob for the same
question — `inf_is_valid` on the observation source — and it is a separate
decision with its own rehearsal.

**Record at least 1200 s of SIM time after the first nonzero command**, capped
at **2 h wall time**. This replaces "record until the explorer exits" — see
*Why 1200 s, and why four cuts* below for what changed and why.

```bash
grep -a 'exploration complete' $LOGS/${RUN}_explore.log
```

That line now tells you the explorer is done, **not** that you may stop. **If
the explorer finishes early, leave the recorder running until the 1200 s is in
the bag.** All four offline cuts come out of one recording, so a bag that stops
when the explorer does cannot supply the 1200 s cut and the run has to be
redone.

1200 s of SIM, not wall. At the ~0.97 RTF these runs hold, it is roughly 21 min
of wall time — use that as the on-the-day proxy and leave margin, because RTF
varies within a run and nothing here may attach to the running stack to read
`/clock` directly. C4 measures it exactly, afterwards, off the bag; if C4 fails
at `--min-sim 1200` the recording was short and the run must be repeated.

If the 2 h cap comes first, stop anyway and note the time in the run log; the
bag is still usable for whichever cuts C4 passes on.

**A retry line at the Nav2 start gate is expected, not a fault.** Under load
the explorer may log

```
get_state on bt_navigator unanswered after 5 s, asking again (attempt N)
```

bt_navigator can drop a `get_state` response while it is still Configuring, and
since `0f48865` the explorer bounds that wait and re-asks instead of blocking on
it forever. Seeing the line means the gate is working. Let it run; it clears as
soon as bt_navigator answers.

**A real stall looks different: no `frontier_explorer` line at all after
`waiting for Nav2 to activate...` for 2 minutes, AND no retry lines either.**
Retry lines say the wait is alive; silence with none of them says it is not.
Tear the run down and report it — that is a new failure, not the one `0f48865`
fixed.

```bash
grep -a 'frontier_explorer' $LOGS/${RUN}_explore.log | tail -5
```

**Attach nothing to the running stack.** No `tf2_monitor` — it cannot answer
R2 anyway (tf2 messages carry no publisher identity, and its rate is the rate
of all of `/tf`: it read ~316 Hz for robot_0 and the same for robot_1). No
live `map_saver_cli` — it failed here with *Failed to spin map subscription*,
and R7 comes out of the bag afterwards. Every check below is offline.

---

## Teardown — in this order, waiting for each

1. **T2, the recorder, first.** Ctrl-C, then wait for `Recording stopped` in
   its log; the line before it, `Writing remaining messages from cache`, is the
   flush that decides whether the bag's last seconds exist.
2. **T3, the explorer.** Ctrl-C, wait for the process to exit.
3. **T1, the simulator.** Ctrl-C, wait for `process has finished cleanly`.
4. **The bridge, by name** — it does not always go with the sim:
   ```bash
   pkill -INT -f parameter_bridge
   ```
5. Then this must print `clean`:
   ```bash
   pgrep -fa '[g]z sim|[p]arameter_bridge|[s]lam_toolbox|[n]sk_swarm|[r]osbag2|[b]ag record|[r]ecord_run' \
       || echo clean
   ```
   The brackets are not decoration. `pgrep -f` matches full command lines,
   including the line that is running `pgrep` itself, so the unbracketed
   spelling always finds one process and never says `clean`. `[g]z` matches
   `gz` in a real process and not the literal `[g]z` in this command.

   **`[r]osbag2` alone does not find the recorder.** `record_run.sh:142` ends
   in `exec ros2 bag record -o …`, and `exec` means the surviving command line
   is exactly that — it contains `ros2` and `bag record`, and the string
   `rosbag2` appears nowhere in it. `rosbag2_recorder` is the ROS *node* name,
   which is what the log shows and what `pgrep -f` never sees. The aborted k0
   run printed `clean` with a recorder still writing. `[b]ag record` catches
   the recorder itself and `[r]ecord_run` catches the wrapper script if it is
   ever run without `exec`.

A surviving bridge or gz server poisons the next run quietly: the next boot
finds topics already advertised and robots already spawned, and the run that
results is neither this one nor a clean one. A surviving recorder fails
differently: the next run is still captured in full by its own recorder, but
the old one keeps its own bag open and appends the new run's messages there as
well. The data lands in both, the previous finished bag is corrupted by a run
it is not a record of, and the second recorder competes for exactly the time
this run measures.

---

## Afterwards — the bag is the record

### 1. Is the bag usable at all?

```bash
python3 experiments/slam/check_run_bag.py \
    --bag $LOGS/$RUN --robot $K --spawn-rev 08617b2 --min-sim 1200
```

**`--min-sim 1200` on this first pass** — it is the recording rule, so this is
the check that the bag holds what it was supposed to hold.

The flag's own default is still 2400 s, which these runs deliberately no longer
use; *Why 1200 s, and why four cuts* below is the reasoning. `--budget` is an
accepted alias for the same knob, so older notes quoting it still work.

* **C1** all 28 recorded topics present and non-empty, `/robot_K/odom` among
  them (that is the check that the wheel stream survived).
* **C2** robot K's first truth pose on its spawn, ≤ 1 cm and ≤ 0.5°.
* **C3** ≥ 5 s and ≥ 50 pose samples before the first nonzero command.
* **C4** at `--min-sim 1200`, PASS means the recording is long enough to cut.
  A FAIL names the shortfall in sim seconds — `sim available after it … short
  by … s` — which is how much longer the next recording has to run. C4 is then
  re-run per cut in step 3 to get each cut's numbers.

Rehearsal 5, at the rehearsal's `--budget 300`: C1 28/28, C2 0.16 cm / 0.00°,
C3 44.67 s with 814 samples, C4 PASS at RTF 0.970.

### 2. The gates: R9, R4/R5, R2, R7, coverage

```bash
python3 experiments/analysis/run_health.py \
    --log $LOGS/${RUN}_explore.log \
    --baseline-log $LOGS/b2maps_e0_explore.log \
    --bag $LOGS/$RUN --robot $K \
    --truth-tf --tf-rates \
    --save-map experiments/maps/${RUN}_online_robot$K
```

The baseline log is the run being replaced, and without it R9 reports but gates
nothing.

| gate | what it must say | rehearsal 5 |
|---|---|---|
| R9 | 0 transform warnings, 0 slam drops | 0 and 0 over 12.14 min |
| R2 | robot K ≈ 20 Hz, parked control ≈ 50 Hz | 20.001 / 50.001 Hz |
| R4 | 0 transforms between two pose samples; any recording edge a short contiguous prefix/suffix | 14471 compared, 0 between, 0 leading and 0 trailing, 16 poses with no transform, worst dyaw 2.5e-14° |
| R5 | median \|dYaw\| > 1° | 81.1° |
| R7 | writes the final online map | 11.95 × 8.00 m, 779 occupied |
| COVERAGE | ≤ 8.00 m | rehearsal 5 read 5.621 m, which FAILED the 3.50 m gate it ran under and PASSES the 8.00 m one. The number is the rehearsal's; the verdict is not comparable across the change |

`--parked` defaults to the lowest robot id that is not K; pass it explicitly
only if that robot was not parked. R7 writes `.pgm`, `.png` and `.yaml`;
`experiments/analysis/pgm_extent.py` reads the PGM back independently if you
want a second opinion on the extent.

### 3. Strip the RAW bag with C4's numbers — once per cut

**Four cuts per run: 60, 120, 240 and the full 1200 s.** Each is a separate
strip and a separate map, all taken from the one recording. Steps 4 and 5 are
unchanged except for the names they are given.

```bash
CUT=240                          # repeat for 60, 120, 240, 1200
CUTRUN=${RUN}_cut${CUT}

# 1. C4 for this cut -- prints --start / --duration spanning exactly CUT sim s
python3 experiments/slam/check_run_bag.py \
    --bag $LOGS/$RUN --robot $K --spawn-rev 08617b2 --min-sim $CUT

# 2. strip that span out of the RAW bag
python3 experiments/slam/strip_bag_for_offline_slam.py \
    $LOGS/$RUN $LOGS/${CUTRUN}_slamin \
    --start <C4's --start> --duration <C4's --duration>

# 3. step 4 and step 5 below, against this cut
BAG=$LOGS/${CUTRUN}_slamin RUN=$CUTRUN SCAN_MATCHING=false FREE_SPACE_RELAY=true \
    bash experiments/slam/run_offline_maps.sh $K

venv/bin/python experiments/slam/fit_world_transform.py \
    --run $CUTRUN --robot $K --spawn-rev 08617b2 --bag $LOGS/$RUN
```

**`--bag` stays `$LOGS/$RUN`, the raw bag, in every cut.** It is what
corroborates the spawn table against parked robots; `$LOGS/$CUTRUN` is not a
bag and there is no bag under that name.

`--start` is the same number for all four cuts — it is the offset of the first
nonzero command, which does not move — and only `--duration` changes. On
`b2maps_relay1` the three cuts it can supply read `--start 46.786` with
`--duration` 61.307, 122.031 and 244.103. If a cut's `--start` differs from the
others, something is wrong with the bag, not with the cut.

Take the cuts a run's C4 passes. `b2maps_relay1` holds 520.220 s of sim after
its first command, so it supplies 60/120/240 and **fails at 1200** (`short by
679.780 s`); it predates this rule. Runs recorded under the rule above supply
all four.

**The raw bag, with no rewrite step.** `rewrite_odom_from_truth.py` exists to
put truth into a bag that was recorded on wheel odometry; these bags carry the
truth transform already. `check_cut_reference.py` therefore has nothing to
compare either — it checks that a rewritten bag kept the raw bag's time zero,
and there is no rewritten bag.

### 4. Offline map, known-pose

Once per cut, with `CUT` and `CUTRUN` set as in step 3:

```bash
BAG=$LOGS/${CUTRUN}_slamin RUN=$CUTRUN SCAN_MATCHING=false FREE_SPACE_RELAY=true \
    bash experiments/slam/run_offline_maps.sh $K
```

Writes `experiments/maps/${CUTRUN}_robot$K.pgm` / `.yaml`. `SCAN_MATCHING=false`
for the same reason as in T3, and it is sound for the same reason: the bag's
`odom → base_footprint` is ground truth.

`FREE_SPACE_RELAY=true` (`run_offline_maps.sh:68`) replays each scan through
the same `free_space_relay` node T3 ran online, so the offline map is built on
the same beams as the online one. Both must be set, or the two maps of one run
answer different questions. The script derives the fill from the range the bag
itself carries (`bag_range_max.py`) and aborts if the fill and the threshold do
not straddle, so old 3.5 m bags replay correctly at 3.4 / 3.45 rather than
against a literal.

### 5. Place the map in the world

Once per cut:

```bash
venv/bin/python experiments/slam/fit_world_transform.py \
    --run $CUTRUN --robot $K --spawn-rev 08617b2 --bag $LOGS/$RUN
```

`--run` is the cut, `--bag` is the raw bag. The 80 % on-wall floor is a gate on
the **1200 s** map; the three short cuts are read for the shape of the map as it
grew, and a short cut scoring below the floor is a fact about how little had
been mapped yet, not a failed run.

**`--spawn-rev 08617b2` every time.** The default is `fc050c4`, the b16 era,
and on a b18/b2maps bag it silently places every map 3.15 m out. `--bag`
corroborates the spawn table against robots that never moved, so the table is
checked rather than trusted. Criterion 5 above (occupied cells on the outer
boundary) is read off the **1200 s** map, the same one the on-wall floor gates
— not off a short cut, which has not had time to reach the boundary.

---

## Why 1200 s, and why four cuts

**The map is finished long before 2400 s.** With the LDS-02 at 8 m and the
free-space relay, `b2maps_relay1` reached 99 % of its final free area about
**430 s after its first nonzero command** (99 % at sim 546.0 s; the command is
at sim 118.6 s), and its last 60 s added 0.39 m² to a 387.7 m² map — 0.1 %. A
2400 s map and a 1200 s map of this world would be near-identical, so the extra
1200 s buys wall-clock cost and no coverage. 1200 s is still ~2.8× the observed
saturation time, which is the margin for a run that explores less efficiently
than this one did. `experiments/analysis/map_saturation.py` is what measures
this, and re-measures it per run:

```bash
python3 experiments/analysis/map_saturation.py --bag $LOGS/$RUN --topic /robot_$K/map
```

**The cuts are there because early maps are shaped by the world, not only by
sensor range.** `experiments/analysis/partial_maps.py` on `b2maps_relay1`
reports an occlusion index — the fraction of straight-line-reachable floor that
is actually known — of **0.59 / 0.71 / 0.72** at 60 / 120 / 240 s. At 60 s,
41 % of what the robot could have seen if nothing blocked it was still unknown.
That shortfall is not the sensor running out of reach: stray (known free
outside the 8 m envelope) is ≤ 0.09 % at every cut, with **zero** cells beyond
range, so everything known is inside the envelope and what is missing is inside
it too — behind walls. The cuts are what make that visible; a single final map
cannot show it.

Read the three cut values, not the final one. `partial_maps.py` defines reach
from the final map's free cells, so the final map scores 1.000 by construction
and is not evidence of anything.

Both numbers above are `b2maps_relay1`'s, which is one run of one world with
one sensor. They are the basis for choosing 1200 s, not a property of the
configuration — re-read them on the first run recorded under this rule before
treating 1200 s as settled.

---

## What a finished run leaves behind

| path | what |
|---|---|
| `experiments/logs/b2maps/b2maps_k<K>/` | the raw bag, 28 topics |
| `experiments/logs/b2maps/b2maps_k<K>_sim.log` | R3's evidence |
| `experiments/logs/b2maps/b2maps_k<K>_record.log` | pre-flight attempt, 26 → 28 subscriptions, clean flush |
| `experiments/logs/b2maps/b2maps_k<K>_explore.log` | R9's input; exploration-complete line |
| `experiments/logs/b2maps/b2maps_k<K>_cut<CUT>_slamin/` | the stripped segment, one per cut (60, 120, 240, 1200) |
| `experiments/maps/b2maps_k<K>_online_robot<K>.*` | R7, the map the run steered by |
| `experiments/maps/b2maps_k<K>_cut<CUT>_robot<K>.*` | the offline known-pose map, one per cut |

---

## What changed since `REHEARSAL_R1_R9.md`

That document was written before rehearsal 4 and is wrong in ways that cost
runs. It is kept for its reasoning, not its commands.

* **Sourcing and build.** It builds inside `ros2_ws` and then keeps
  `ros2_ws/install/setup.bash` sourced for the run. That install has no
  `truth_odom_tf` and shadows the repo-root one. Build from the repo root —
  `ros2_ws/install` belongs in the build shell only, for
  `nsk_swarm_interfaces` — and run from `~/Desktop/NSKsim/install`.
* **Launch arguments.** It shows `nav_robots:=[0]` alone in places; both
  `nav_robots:=[K]` and `truth_odom_robots:=[K]` are required.
* **`tee`.** It pipes all three processes through `tee`. Logs only.
* **R2 by `tf2_monitor`.** It cannot answer the question: tf2 messages carry no
  publisher authority and the rate reported is all of `/tf` (~316 Hz for both
  robots). R2 now comes from the bag, via `run_health.py --tf-rates`.
* **R7 by live `map_saver_cli`.** It failed with *Failed to spin map
  subscription*. R7 now comes from the bag, via `--save-map`.
* **The 26-subscription gate** before starting the explorer is new; rehearsal 4
  had no such check and rehearsal 5 confirmed the count.
* **Teardown.** The old order (explorer, recorder, sim) risks losing the
  recorder's cache flush, and it never mentions `parameter_bridge`, which
  survives the sim.
* **Pre-flight aborts.** Rehearsal 4 aborted on a `ros2 topic list` discovery
  race, 13 of 25 topics "missing" while they were being received.
  `record_run.sh` now retries 5 times, 3 s apart, re-checking only what was
  missing; `experiments/slam/preflight_stub_check.sh` is the standalone proof
  that recorded content is unchanged on every path.
