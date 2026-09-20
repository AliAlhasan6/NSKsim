# B2 maps — the five truth-driven explorer runs, e0t … e4t

One run per robot, one at a time, never two at once. In run **e\<K\>t** robot K
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

## Success criteria — fixed before e0t is started

A run counts when all five hold. Nothing here is judged by eye; each is a
number a script prints.

1. **COVERAGE ≤ 3.50 m.** The *true* trajectory passes within the burger's own
   lidar maximum range of an outer boundary wall at least once. e0 managed
   3.897 m and stopped.
2. **R9's two gated counts are 0** — Nav2/tf2 transform warnings and
   slam_toolbox message-filter drops. e0's baseline is 0.000/min over
   98.96 min, so the limit is 0.000 and the first warning fails the run.
3. **R4 exact**: 0 transforms off a truth stamp, worst |dxy| ≤ 1e-6 m, worst
   |dyaw| ≤ 1e-6 deg, worst |z| = 0.
4. **R5 median wheel-vs-truth |dYaw| > 1°** — `/robot_K/odom` must still be
   independent wheel odometry, or arm D cannot be built from these bags.
5. **The offline truth map has occupied cells on the outer boundary.**

R2 (one owner per frame pair), R3 (identity at spawn), R6 (`map → odom`
identity) and C1–C4 are preconditions, not criteria: if any of them fails the
run is not a run, and the four above cannot be read from it.

---

## Once, before e0t — build and sourcing

```bash
cd ~/Desktop/NSKsim
colcon build --packages-select nsk_swarm          # from the REPO ROOT
source /opt/ros/jazzy/setup.bash
source ~/Desktop/NSKsim/install/setup.bash        # the repo-root install
```

Every terminal below starts with those two `source` lines.

**Never source `ros2_ws/install/setup.bash`.** That tree was built on
2026-09-10 and has no `truth_odom_tf` in it; the repo-root install does. The
failure is not always loud — sourcing the stale tree first puts an older
`nsk_swarm` ahead of the one you just built, and the launch either dies at 4 s
with *executable not found* or runs the wrong node set. Confirm before
launching:

```bash
ls install/nsk_swarm/lib/nsk_swarm/     # must list truth_odom_tf
```

`nsk_swarm_interfaces` still resolves out of `ros2_ws/install`; sourcing the
repo-root install chains it in, which is why the repo-root one is the only one
you source.

The install holds **copies**, not symlinks (`--symlink-install` is not used),
so every edit to a launch file or a node needs that `colcon build` again before
`ros2 launch` can see it.

---

## The run — three terminals

Set the explorer id once per terminal:

```bash
K=0                                   # 0, then 1, 2, 3, 4 in later runs
RUN=b2maps_e${K}t
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
    > $LOGS/${RUN}_explore.log 2>&1
```

`scan_matching:=false` is what makes this a known-pose run: slam_toolbox never
calls `MatchScan`, every scan keeps the pose it was handed, and `map → odom`
stays identity. It is only sound because T1 supplied truth poses.

**Record until the explorer exits**, capped at **2 h wall time**:

```bash
grep -a 'exploration complete' $LOGS/${RUN}_explore.log
```

If the cap comes first, stop anyway and note the time in the run log; the bag
is still usable as long as C4 passes on it.

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
   pgrep -fa '[g]z sim|[p]arameter_bridge|[s]lam_toolbox|[n]sk_swarm|[r]osbag2' \
       || echo clean
   ```
   The brackets are not decoration. `pgrep -f` matches full command lines,
   including the line that is running `pgrep` itself, so the unbracketed
   spelling always finds one process and never says `clean`. `[g]z` matches
   `gz` in a real process and not the literal `[g]z` in this command.

A surviving bridge or gz server poisons the next run quietly: the next boot
finds topics already advertised and robots already spawned, and the run that
results is neither this one nor a clean one.

---

## Afterwards — the bag is the record

### 1. Is the bag usable at all?

```bash
python3 experiments/slam/check_run_bag.py \
    --bag $LOGS/$RUN --robot $K --spawn-rev 08617b2
```

The **default 2400 s budget** applies to these runs — do not pass `--budget`.

* **C1** all 28 recorded topics present and non-empty, `/robot_K/odom` among
  them (that is the check that the wheel stream survived).
* **C2** robot K's first truth pose on its spawn, ≤ 1 cm and ≤ 0.5°.
* **C3** ≥ 5 s and ≥ 50 pose samples before the first nonzero command.
* **C4** prints the `--start` / `--duration` step 3 needs.

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
| R4 | 0 transforms off a truth stamp | 14471 transforms, 0 off, worst dyaw 2.5e-14° |
| R5 | median \|dYaw\| > 1° | 81.1° |
| R7 | writes the final online map | 11.95 × 8.00 m, 779 occupied |
| COVERAGE | ≤ 3.50 m | FAIL at 5.621 m — expected on 12 min, **not** on a real run |

`--parked` defaults to the lowest robot id that is not K; pass it explicitly
only if that robot was not parked. R7 writes `.pgm`, `.png` and `.yaml`;
`experiments/analysis/pgm_extent.py` reads the PGM back independently if you
want a second opinion on the extent.

### 3. Strip the RAW bag with C4's numbers

```bash
python3 experiments/slam/strip_bag_for_offline_slam.py \
    $LOGS/$RUN $LOGS/${RUN}_slamin \
    --start <C4's --start> --duration <C4's --duration>
```

**The raw bag, with no rewrite step.** `rewrite_odom_from_truth.py` exists to
put truth into a bag that was recorded on wheel odometry; these bags carry the
truth transform already. `check_cut_reference.py` therefore has nothing to
compare either — it checks that a rewritten bag kept the raw bag's time zero,
and there is no rewritten bag.

### 4. Offline map, known-pose

```bash
BAG=$LOGS/${RUN}_slamin RUN=$RUN SCAN_MATCHING=false \
    bash experiments/slam/run_offline_maps.sh $K
```

Writes `experiments/maps/${RUN}_robot$K.pgm` / `.yaml`. `SCAN_MATCHING=false`
for the same reason as in T3, and it is sound for the same reason: the bag's
`odom → base_footprint` is ground truth.

### 5. Place the map in the world

```bash
venv/bin/python experiments/slam/fit_world_transform.py \
    --run $RUN --robot $K --spawn-rev 08617b2 --bag $LOGS/$RUN
```

**`--spawn-rev 08617b2` every time.** The default is `fc050c4`, the b16 era,
and on a b18/b2maps bag it silently places every map 3.15 m out. `--bag`
corroborates the spawn table against robots that never moved, so the table is
checked rather than trusted. The 80 % on-wall floor is the criterion here;
criterion 5 above (occupied cells on the outer boundary) is read off this map.

---

## What a finished run leaves behind

| path | what |
|---|---|
| `experiments/logs/b2maps/b2maps_e<K>t/` | the raw bag, 28 topics |
| `experiments/logs/b2maps/b2maps_e<K>t_sim.log` | R3's evidence |
| `experiments/logs/b2maps/b2maps_e<K>t_record.log` | pre-flight attempt, 26 → 28 subscriptions, clean flush |
| `experiments/logs/b2maps/b2maps_e<K>t_explore.log` | R9's input; exploration-complete line |
| `experiments/logs/b2maps/b2maps_e<K>t_slamin/` | the stripped segment |
| `experiments/maps/b2maps_e<K>t_online_robot<K>.*` | R7, the map the run steered by |
| `experiments/maps/b2maps_e<K>t_robot<K>.*` | the offline known-pose map |

---

## What changed since `REHEARSAL_R1_R9.md`

That document was written before rehearsal 4 and is wrong in ways that cost
runs. It is kept for its reasoning, not its commands.

* **Sourcing and build.** It builds inside `ros2_ws` and sources
  `ros2_ws/install/setup.bash`. That install has no `truth_odom_tf` and
  shadows the repo-root one. Build from the repo root, source
  `~/Desktop/NSKsim/install/setup.bash`.
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
