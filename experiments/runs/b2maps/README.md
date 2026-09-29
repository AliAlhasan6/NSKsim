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

A run counts when all six hold. Nothing here is judged by eye; each is a
number a script prints.

**A criterion that could not be evaluated is not a passed one.** `run_health.py`
exits 0 only when every gate that ran passed *and* none was skipped; a gate that
was asked for and could not be judged — most often a bag too short to place the
analysis window below — prints `NOT EVALUATED` and exits **3**, distinct from the
1 a failure exits with. So nothing chaining on the exit status can read a short
or incomplete bag as green.

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
2. **R9 gates nothing inside the analysis window.** Zero Nav2/tf2 transform
   warnings and zero message-filter drops inside `[t_cmd, t_cmd + 1200 s]` —
   the same window as criterion 3, read once from the bag and shared between
   the two gates — with exactly two exclusions:

   * warnings naming `robot_K/map ↔ robot_K/odom`, in either direction, and
   * message-filter drops whose **logger** is slam_toolbox's
     (`[robot_K.slam_toolbox]`, not the `[async_slam_toolbox_node-N]` process
     tag).

   Both exclusions are the same exclusion twice: that pair *is* online
   slam_toolbox's output, and so is its message-filter queue, and the product
   maps are built offline from the stripped bag, so neither can reach them. They
   are reported by node and by pair, never gated.

   **A drop is then judged on the bag, not on the log.** A message-filter drop
   names a frame and an instant, not a pair, so being inside the window only
   makes it a *candidate*. `attribute_tf_drop.py` reads the chain from that
   costmap's `global_frame` down to the dropped message's frame — links derived
   from `/tf_static`, `global_frame` read from `nav2_robotK.yaml` — and asks of
   each dynamic link whether its first transform covering that instant reached
   the wire before the line was logged. The drop **fails the run** only if some
   link other than `map ↔ odom` was late, or supplied nothing at all (fail
   closed, as does a drop that cannot be attributed). "None late" and "only
   `map ↔ odom` late" are advisory, printed with the per-link evidence. The
   neighbour lags printed beside it are scale for a reader and are compared
   against nothing.

   Why: of the four in-window drops in this corpus, one (k2's global costmap at
   scan 629.400) is a real transform stall — `robot_2/map → robot_2/odom` froze
   at stamp 629.300 for 1.502 s of wall, 5.7× its normal dwell, then skipped to
   630.300 — and three had every link on the wire 0.5–0.9 s *before* the line.
   Those three are the node not consuming in time what it already had, which is
   load, not the rig. Failing a run for it would fail k2 and k3 for something no
   map can be wrong about.

   **The blind spot, stated.** A drop is only logged after the filter's own wait
   expires — `buffer_timeout`, which nav2 sets from `transform_tolerance`, 0.5 s
   in both costmaps here (`nav2_robotK.yaml:163` and `:236`). So the comparison
   is against a deadline 0.5 s later than the instant the transform was needed:
   a rig transform late by **less than that wait**, on a node slow enough to
   have missed it anyway, reaches the wire before the line is logged and reads
   "none late". This rule cannot see such a case, and neither could the old
   count — it is the reason criterion 2 is a floor and not a proof of
   timeliness. A "none late" drop means *no link missed the 0.5 s deadline*,
   not *every transform was on time*.

   **R10 closes it by measuring the link directly** (`--rig-lag`), and it is a
   gate: see criterion 6.

   **Every other transform warning inside the window fails the run**, above all
   one naming `robot_K/odom → robot_K/base_footprint` — the transform
   `truth_odom_tf` publishes. A warning whose frame pair cannot be parsed also
   fails, and is printed in full: an unrecognised phrasing is not an excused
   one. The same warning **outside** the window is counted and printed, not
   gated.

   The split matters because pooling hid it. All 8 `Transform data too old`
   lines in this corpus (e0t's 7, k2's 1) name `map ↔ odom` and trace to
   slam_toolbox's estimate running ~0.55 s stale; the one line naming the
   transform under test is `b2maps_relay1`'s `collision_monitor`
   extrapolation error at sim 463 — six minutes into driving, not a bring-up
   artefact. Counted together, warnings about a transform this work does not
   touch set the budget for the one it does.

   `--baseline-log` still prints e0's rates beside this run's, and that
   comparison is now **advisory**: relay1's 0.107 warnings/min is over e0's
   limit, under e0t's and over rehearsal5's — one run, three verdicts, none of
   them about the run.
3. **R4 exact, inside the analysis window**: 0 transforms falling on no truth
   stamp, worst |dxy| ≤ 1e-6 m, worst |dyaw| ≤ 1e-6 deg, worst |z| = 0.

   The window is `[t_cmd, t_cmd + 1200 s]` in **sim** time, where `t_cmd` is the
   first nonzero command on `/robot_K/cmd_vel` — the same command C3 and C4
   place their windows on, computed by importing `check_run_bag.py`'s own
   `first_nonzero_cmd` and `sim_window`, so R4 gates exactly the span step 3
   cuts the map from. `--min-sim` sets the length and defaults to 1200; pass the
   same number to both scripts.

   **Inside the window nothing is forgiven** — every transform must land on a
   truth stamp and match `inv(spawn) · truth` to 1e-6. That is the rule the gate
   exists for: a surviving DiffDrive broadcast publishes *through* the run, so
   its transforms land inside. **Outside the window nothing is gated**, because
   no map is built from that time; the counts, the unmatched and the worst error
   are printed for both ends anyway.

   This replaces the `EDGE_GRACE_PERIODS` rule (`e7e2abb`), which gated the
   recorder rather than the transform. rosbag2 subscribes to `/tf` and to
   `/model/robot_K/pose` at different moments and whichever wins is a discovery
   race: relay1 +142.4 ms (3 transforms), rehearsal5 −224 ms (none), k0…k4 32,
   0, 20, 34 and 28 periods. Every one of those edges ended *before* the first
   command — k0's at sim 57.150 against a command at 105.850 — and a pose hole
   anywhere in the series withheld the grace outright, so k2 failed on a 200 ms
   hole 14 s before its own first command while every compared transform agreed
   to 0.000 m and 2.5e-14°.

   Poses carrying no transform are printed and not gated; that is a rate
   question and R2 owns it. A pose hole is likewise not itself a failure — it
   fails only if it strands transforms inside the window, which is the same rule
   as everything else there, not a special case.
4. **R5 median wheel-vs-truth |dYaw| > 1°** — `/robot_K/odom` must still be
   independent wheel odometry, or arm D cannot be built from these bags.
5. **The offline truth map has occupied cells on the outer boundary.**
6. **R10: the rig transform is on time for the consumers that read it.** Inside
   the same analysis window, the *consumer-seen staleness* of
   `robot_K/odom → base_footprint` — at every arrival, sim now minus the newest
   stamp already on the wire — must never exceed **the smallest
   `transform_tolerance` declared by a consumer that is always asking**. That
   value is read by `tolerance_facts()` from the run's own `nav2_robotK.yaml`
   and never typed into the script; today it is `collision_monitor`'s **0.2 s**,
   and the verdict prints the block and line it came from.

   Staleness and not the wall-clock wire lag: every node runs `use_sim_time`, so
   a tolerance bounds **sim** age. The two are not interchangeable and not even
   ordered the same way — k4's worst transform is 151.9 ms of wall lag but only
   139.6 ms of sim staleness, while k3's worst is 90.9 ms of lag and 149.4 ms of
   staleness. Staleness floors at one publication period (50 ms at 20 Hz), so a
   healthy link reads ~57 ms.

   **`behavior_server` is excluded, though it declares the tightest tolerance of
   all (0.1 s).** It only looks up this link while a behaviour is running, and
   across k0–k4 it logged not one `Running <plugin>` line — in five full runs it
   never asked for this transform, while k2, k3 and k4 each carry staleness
   excursions over its 0.1 s. Gating on a deadline nothing was waiting on would
   fail three runs for a transform no consumer was reading. The exclusion is by
   name and is the only one: a block appearing in these files later is included
   by default, which can only tighten the gate, whereas excluding one loosens it
   and has to be argued for in the source.

   **Reported, never gated**: every in-window excursion over `behavior_server`'s
   0.1 s that is within the gate, each with how long before it — in sim seconds
   — behavior_server was last seen starting a behaviour, or "none started";
   everything outside the window, on the same grounds as criteria 2 and 3; and
   the wall-lag distribution beside the staleness one. `b2maps_relay1` is why
   the 0.1 s figure stays on the page: it ran 11 behaviours, and a `wait` had
   started 1.3 s before its 346.9 ms excursion, so there the tightest declared
   deadline really was in force.

   No start-up exemption. k4's excursions are not confined to bring-up anyway —
   sim 97.8–140.2, then 644.8 and 694.4, against a window opening at 95.8.

R2 (one owner per frame pair), R3 (identity at spawn), R6 (`map → odom`
identity) and C1–C4 are preconditions, not criteria: if any of them fails the
run is not a run, and the criteria above cannot be read from it.

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

**Do not start T3 until the recorder has 26 subscriptions.** The T3 block below
checks that itself and refuses to launch, so it is not a step to remember.

26 is the 25 required topics plus `/robot_K/cmd_vel`, whose publisher
`robot_node` creates unconditionally. The last two — `/robot_K/map` and
`/robot_K/map_metadata` — only exist once the explorer brings slam_toolbox up,
and rosbag2's topic discovery (on by default, 100 ms) picks them up then,
taking the bag to 28.

### T3 — the explorer

**Guarded: it refuses to launch below 26 recorder subscriptions.** k3 attempt 1
was lost to launching without that check — the recorder joined 32 s after goal
#1, so the bag opened on a robot that had already been commanded to move and
could anchor nothing.

```bash
N=$(grep -ac 'Subscribed to topic' $LOGS/${RUN}_record.log 2>/dev/null); N=${N:-0}
if [ "$N" -lt 26 ]; then
  echo "STOP: recorder shows $N subscriptions, need 26. Wait 10 s and re-run this block."
else
  date '+%T' | tee $LOGS/${RUN}_explorer_start.txt
  ros2 launch nsk_swarm explore.launch.py robot_id:=$K scan_matching:=false \
      free_space_relay:=true \
      > $LOGS/${RUN}_explore.log 2>&1
fi
```

It expects `K`, `RUN` and `LOGS` from the block at the top of this section, like
every other terminal here. A tab where they are unset finds no log, reads
`N=0` and stops, which is the direction this should fail in.

`date '+%T' | tee` leaves the explorer's start time in
`${RUN}_explorer_start.txt`. Nothing gates on it; it is the one record of when
T3 began that does not depend on a log surviving.

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
4. **Block B — the bridge, and the check that nothing survived.** Only after
   steps 1–3. **Guarded: it refuses to run while gz sim is alive.** k1 attempt 1
   was lost to running it early — `pkill -f parameter_bridge` also kills the
   `/clock` bridge, and with the sim still up that freezes sim time under a run
   that looks alive.

   ```bash
   if pgrep -f '[g]z sim' > /dev/null; then
     echo "STOP: the sim is still running. Block B is only for after teardown."
   else
     pkill -INT -f parameter_bridge; sleep 5
     pgrep -fa '[g]z sim|[p]arameter_bridge|[s]lam_toolbox|[n]sk_swarm|[b]ag record|[r]ecord_run|[c]ontroller_server|[l]ifecycle_manager|[b]t_navigator|[r]obot_node|[n]sk_engine' \
       || echo clean
     ls -la $LOGS/$RUN/
   fi
   ```

   It must print `clean`, and then the bag's own files. The `sleep 5` is the
   bridge's own shutdown: without it the `pgrep` can catch a bridge that is
   already on its way out and report the teardown as unclean.

   The brackets are not decoration. `pgrep -f` matches full command lines,
   including the line that is running `pgrep` itself, so the unbracketed
   spelling always finds one process and never says `clean`. `[g]z` matches
   `gz` in a real process and not the literal `[g]z` in this command — which is
   also why the guard above can ask about `[g]z sim` without answering itself.

   **The nav2 nodes are named one by one** — `[c]ontroller_server`,
   `[l]ifecycle_manager`, `[b]t_navigator` — because nothing else in the
   pattern reaches them: they are installed under `/opt/ros/jazzy/lib/nav2_*`
   and their command lines contain no `nsk_swarm`. `[r]obot_node` and
   `[n]sk_engine` are this package's own, so `[n]sk_swarm` already matches
   their paths; they are listed because the runs listed them.

   **`[r]osbag2` was in this pattern and has been removed, because it never
   matched the recorder.** `record_run.sh:142` ends in
   `exec ros2 bag record -o …`, and `exec` means the surviving command line is
   exactly that — it contains `ros2` and `bag record`, and the string `rosbag2`
   appears nowhere in it. `rosbag2_recorder` is the ROS *node* name, which is
   what the log shows and what `pgrep -f` never sees. The aborted k0 run
   printed `clean` with a recorder still writing, against a pattern that had
   `[r]osbag2` and nothing else that could find it. `[b]ag record` is what
   catches the recorder now, and `[r]ecord_run` the wrapper script if it is
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
    --truth-tf --tf-rates --min-sim 1200 --rig-lag \
    --save-map experiments/maps/${RUN}_online_robot$K
```

`--rig-lag` adds R10 (criterion 6). It reads the whole bag rather than the
window, since it reports what happened outside as well, and costs ~30 s.

`--min-sim 1200` is the same number step 1 gave `check_run_bag.py`, and for the
same reason: it is the span the map is cut from, so it is the span R4 gates.
It is also this script's default, and is written out here because the number is
the recording rule rather than a setting.

The baseline log is the run being replaced. Its rates are printed beside this
run's and are **advisory** (criterion 2); R9's verdict is the window gate, which
needs `--bag` rather than a baseline. Without `--bag` there is no window, so R9
reports its counts and exits NOT EVALUATED.

| gate | what it must say | rehearsal 5 |
|---|---|---|
| R9 | nothing gated inside the window: no transform warning on a pair other than `map ↔ odom`, and no message-filter drop whose attribution names a late link other than `map ↔ odom` | rehearsal 5 logged 0 and 0 over 12.14 min. k0–k4 all pass: k2's and k3's two in-window costmap drops each are advisory once attributed — k2's on `map ↔ odom`, the other three with every link on the wire before the line. relay1 FAILS, on a `collision_monitor` warning naming `robot_0/odom → base_footprint` |
| R2 | robot K ≈ 20 Hz, parked control ≈ 50 Hz | 20.001 / 50.001 Hz |
| R4 | inside `[t_cmd, t_cmd + 1200 s]` sim: 0 transforms on no truth stamp, worst \|dxy\| ≤ 1e-6 m, \|dyaw\| ≤ 1e-6°, \|z\| = 0. Outside it: reported, never gated | k2, the awkward one: 24000 compared inside the window with 0 unmatched and worst dyaw 2.5e-14°; before `t_cmd`, 721 transforms of which 20 on no truth stamp, and a 200 ms pose hole at sim 61.850 — all of it outside the window, none of it gated |
| R5 | median \|dYaw\| > 1° | 81.1° |
| R10 | in-window consumer-seen staleness of `odom → base_footprint` ≤ the tightest always-asking `transform_tolerance` (0.2 s, `collision_monitor`) | k0–k4 pass: worst staleness 94–164 ms against the 0.2 s gate, and the 0.1 s excursions k2/k3/k4 carry are reported with "none started" beside them. relay1 FAILS: 4 samples over 0.2 s, worst 346.9 ms at sim 463, with a `wait` 1.3 s earlier |
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
    DETERMINISTIC=true \
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
    DETERMINISTIC=true \
    bash experiments/slam/run_offline_maps.sh $K
```

Writes `experiments/maps/${CUTRUN}_robot$K.pgm` / `.yaml`. `SCAN_MATCHING=false`
for the same reason as in T3, and it is sound for the same reason: the bag's
`odom → base_footprint` is ground truth.

**`DETERMINISTIC=true` on every B2 map, no exceptions.** Without it, replaying
one stripped segment twice does not give one map. Measured on
`b2maps_k4_cut60`, seven replays of a single segment — same 300 scans, and
`free_space_relay`'s 200-scan checkpoint byte-identical at 15681/72000 beams
filled — produced **two different maps**: five at 353 occupied cells and two at
469, with 411 cells (3.06 % of everything either map knew) in different classes.
It is not a rare corner: 2 in 7.

The cause is that scan acceptance is a *chain*. `minimum_travel_distance` (0.1 m),
`minimum_travel_heading` (0.1 rad) and `minimum_time_interval` (0.2 s) each
measure from the last **accepted** scan, so losing one scan re-anchors every
decision after it — and `minimum_time_interval` is the sharpest of the three,
because these bags deliver scans at exactly 5 Hz, i.e. 0.2 s apart, so the
default sits exactly on its own threshold. What is lost varies run to run with
DDS discovery at the start of the replay: across those replays slam_toolbox
logged `95.800 'queue is full'`, `95.800 'earlier than all the data in the
transform cache'`, `97.000 'earlier than …'`, and in two replays no drop at all.

*Which* scan is lost is what matters, not that one is. Losing the first scan
(95.800) is harmless and provably so — the replay that dropped it is
byte-identical to the two that dropped nothing — because at sim 95.81 the robot
has only just been commanded and consecutive scans are the same view from the
same place. The divergent maps are the two that lost **97.000**, six scans later,
after motion had begun.

`DETERMINISTIC=true` sets all three gates to 0, so every scan is integrated and
there is no chain to re-anchor, and adds `ros2 bag play --delay 3.0` so DDS
discovery finishes before the first scan is published. Under it, all ten replays
(5 at 60 s, 3 at 240 s, 2 at 1200 s) came out **byte-identical** within their cut.
The gates alone are what fix it — five replays with the gates zeroed and no delay
were also byte-identical while the drop still varied between them — and the delay
is kept because it makes the input deterministic too, at 3 s per replay.

It changes the map, which is the reason it must be all or nothing: integrating
every scan maps more of the world, so occupied cells go 353 → 369 at 60 s,
1039 → 1126 at 240 s and 1708 → 1753 at 1200 s (4.10 %, 4.26 % and 0.22 % of
known cells differ from the old settings). A cut built with the gates on
therefore cannot be read against one built with them off, and **the five
`b2maps_k<K>_cut1200` maps built before this rule have to be re-made.**

It changes how much is mapped, not where the map goes. Fitted with
`fit_world_transform.py` on the same segments, convention A — the one the
verdict picks — reads **98.0 → 97.8 %** on-wall at 60 s, **99.1 → 99.3 %** at
240 s and **99.4 → 99.5 %** at 1200 s. (Convention B is the wrong row
convention and is not the answer; it moves further, 58.9 → 71.5 % at 60 s.) The
verdict itself is unchanged in kind: `AMBIGUOUS` appears for the 1200 s map
either way — it is the 180° world-symmetry case `fit_world_transform.py`
documents, and it was already `b2maps_k4_cut1200`'s verdict before this change.

What built a map is recorded beside it, in
`experiments/logs/offline_mapping_${CUTRUN}_robot_$K.yaml`: all three of
`minimum_travel_distance`, `minimum_travel_heading` and `minimum_time_interval`
at 0 is this variant, and anything else is not.

The numbers above come from `experiments/maps/replay_noise/` (25 maps),
measured with `experiments/analysis/pgm_agreement.py`, which compares classes
per cell in world coordinates — necessary because slam_toolbox recomputes a
*continuous* grid origin per save, and two replays of one segment sat 0.479 and
0.243 cells apart, so comparing by array index would report a sub-cell shift as
wholesale disagreement.

**Zeroing the gates is necessary but not sufficient: a scan that never arrives
cannot be integrated.** slam_toolbox subscribes through a
`tf2_ros::MessageFilter` that parks scans until `odom → base_footprint` is
available, and `scan_queue_size` — the depth of that queue — **defaults to 1**
(measured on the live node, slam_toolbox 2.8.5). A scan still parked when the
next one arrives is discarded, logged as `discarding message because the queue
is full`, and simply missing from the map. The k4 replays above never showed it
past the first scan, so the rule above was written without it; `b2maps_k0_cut1200`
then dropped **6** scans that way under `DETERMINISTIC=true` — at sim 773.0,
796.0, 806.0, 850.8, 1217.8 and 1272.8 — and was tabulated as reproducible with
nothing guaranteeing that it was.

Those particular six turned out not to matter. Replayed **twice** with the queue
sized to the segment — 20 min 53 s and 20 min 43 s, **0 drops each**, against 6
before — both maps are byte-identical to each other *and* to the map built
without the parameter: md5 `aefb6eea256df1023464d46e10f10515`, 1565 occupied,
0 differing cells by `pgm_agreement.py`, identical `.yaml` origin. At 5 Hz a
dropped scan is nearly the view its neighbours already carried, and by sim 773
that part of the floor was mapped. So the corpus map stands — what changed is
that it is now *guaranteed* rather than lucky, and a different draw of the race
had no reason to be as kind.

`DETERMINISTIC=true` now also sets `scan_queue_size` to **the segment's own scan
count**, read off the bag per robot (`bag_scan_facts.py --count`; 6000 for a
1200 s cut). A queue that can hold every scan the replay will ever publish cannot
evict one, so this is sufficient by construction rather than by a margin. It
costs nothing when unused — the filter appends to a list, it does not preallocate
a buffer.

**One drop survives, and it is the one that is harmless.** The segment's first
scan has no transform behind it yet, so it is discarded either for the queue or
for an empty transform cache; which of the two is logged varies and does not
matter. `run_b2maps_cuts.sh` gates on exactly that shape — **at most one drop,
and only at the segment's first scan** — and fails the row on any other, because
a drop elsewhere is a scan missing from the map and *which* scan loses that race
is a timing accident. It is the same distinction the k4 measurement above already
drew between losing 95.800 (harmless, byte-identical) and losing 97.000
(divergent), now enforced instead of described.

A map whose log shows no drop past the first scan is **unaffected** by the queue
size, because the integrated set is the same either way. Confirmed on
`b2maps_k0_cut60` (0 drops): rebuilt with `scan_queue_size: 300`, its `.pgm` is
byte-identical to the one built without the parameter, md5
`c5b4b97c70e89d69b141098427b75257`, sidecar origin included.

`experiments/slam/run_b2maps_cuts.sh` batches steps 3–5 and sets
`DETERMINISTIC=true` itself, so the rule cannot be forgotten on one cut of
fifteen; it prints the variant in its log header, and in its final table it
reports what built every map it skipped, so a corpus half-built under the old
gates is visible rather than silent.

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
