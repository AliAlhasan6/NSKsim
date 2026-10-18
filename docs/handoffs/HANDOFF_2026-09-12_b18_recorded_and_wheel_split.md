# HANDOFF — B1.8 recorded twice, the wheel/body split is done, and the two runs disagree by an order of magnitude

Session: 2026-09-11 00:14–02:00 local, resumed 2026-09-12 14:30–15:20.
Previous handoff: `HANDOFF_2026-09-11_analysis_script_and_headless.md`.
Repo `~/Desktop/NSKsim`, HEAD still `840b0ba` on `main`.

**The working tree is NOT clean.** One deliberate, built, verified, *uncommitted*
edit to `ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py` — the
`joint_states` bridge entry (§3). Commit it first thing; the analysis in §6
depends on it and `b18_run3` cannot be reproduced without it. Suggested message:

```
launch: bridge joint_states gz->ROS for wheel-level odometry analysis
```

The three previously untracked items (`.vscode/`, `experiments/wall_affinity.py`,
`ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`) are unchanged.

Conventions unchanged: replies 120–130 words, one command block per turn,
blocking/sim/expected-time header on every block, every terminal sources
`/opt/ros/jazzy/setup.bash` then the **repo-root** `install/setup.bash`, any spec
for Claude Code introduced with a **(for Claude Code)** heading, scripts as
downloadable files, Ali runs terminals and commits, Claude Code edits repo files
with manual approval.

---

## 0. One-paragraph state

B1.8 is recorded — twice — and the analysis script works. The blocker the last
three handoffs described is gone. `b18_run2` (2329 s, 22 topics) and `b18_run3`
(574 s, 27 topics, joint states included) are both on disk with clean
`metadata.yaml`. **Stamp skew is ruled out.** The wheel-versus-body split, which
was the point of `run3`, came back decisive: the DiffDrive plugin's odometry
integration is exact to +0.002 %, and physical slip is −0.26 %. But `run2`
measured odometry over-reporting rotation by **+2.12 %** against ground truth,
and `run3` measured **−0.26 %** — same quantity, same code path, an order of
magnitude apart and opposite in sign. `run3`'s figure is the corroborated one.
**Reconciling the two runs is the next session's whole job**, and both bags are
already on disk, so it needs no simulator.

---

## 1. What was recorded

| | `b18_run2` | `b18_run3` |
|---|---|---|
| date | 2026-09-11 00:47–01:26 | 2026-09-12 15:04–15:13 |
| duration | 2329.3 s | 573.8 s |
| topics | 22 | 27 |
| messages | 1 519 736 | 753 346 |
| size | 741.6 MiB | 258.5 MiB |
| ground truth | 38 604–38 605 @ 16.6 Hz | 10 917–10 924 @ 19.0 Hz |
| joint states | — | 54 586–54 615 @ ~95 Hz |
| robot_0 true rotation | 51 215 ° | 14 762 ° |
| `Failed to make progress` | 35 | 12 in the first 7 min |
| `/clock` | 86.9 → 77.2 Hz | 92.8 Hz |
| load | ~30–33 | ~31 |

Both under `nav_robots:=[0] headless:=true`, `wander` at its default `false`, so
robots 1–4 held spawn poses as static obstacles and published ground truth
regardless. Both live in `experiments/logs/b18/`, which is **gitignored** — these
bags are not tracked and not backed up anywhere.

`/clock` at 92.8 Hz in `run3` against 86.9 in `run2` means RTF was *better* in
`run3` at the same load. Do not assume the two runs were under equal pressure.

### The pre-run checks that passed

- five `/model/robot_N/pose` advertised, 19.83–20.01 Hz against 20 configured
- three robots bit-identical to sixteen digits across 40 s
- all five on the `DOT_POSES` pentagon (`swarm_sim.launch.py:78`), −1.6 mm x offset
- stack reached `Managed nodes are active`, readiness gate passed
- base load headless with `nav_robots:=[0]`: **13.01 / 80.8 % us**, below the
  15.98 measured the night before

### `record_run.sh` aborts on a single-mover run

Its topic list unconditionally demands all five `/robot_N/map` and
`/robot_N/map_metadata`. With one stack, eight are unadvertised and it exits 1 —
correctly, by its own design. Both runs were recorded with an inline topic list
instead. **The script needs a `--nav-robots` or equivalent before it can be used
for single-mover runs**, and until then every such run bypasses its guard.

---

## 2. Ruled out: stamp skew, and three of my own hypotheses

`yaw_truth_audit.py` ran **first time, EXIT=0, in 54 s**, writing all three CSVs
and five PNGs. It needed no fixes despite never having touched a real message.
The previous handoff's budget for repairing it was not spent.

**(b1) scan-to-pose stamp skew is ruled out.** Pose mean −0.02 ms, median 0.00,
against a (b2) sweep where even ±25 ms would inject 0.66 °/scan. The 7.99 ms
*odom* mean is 50 Hz sampling geometry, not skew. This also survived the lag scan
in §5, which found the net error flat to 0.016 % across ±100 ms.

**The four parked robots show exactly 0.00 ° odometry yaw error over 96 508
samples each**, and `run3`'s parked robot_1 integrates to **0.0000 °** of wheel
rotation. Parked wheels do not drift. The control is perfect and should be kept
in every future run — it costs nothing and it is the only thing that would catch
a systematically broken ground-truth path.

Falsified this session, all mine:

| hypothesis | how it died |
|---|---|
| yaw unwrap bug (the `180.00` max) | distribution is smooth — p50 74°, p90 152°, p99 178°, only 329 of 96 508 above 179.5°. The 180.00 was rounding. |
| wheel separation mismatch | SDF wheels at ±0.08 m = 0.160 m configured; `wheel_radius` 0.033 matches collision geometry at lines 184/246. Confirmed again live from the joint message. |
| truth interpolation error | interpolating the *fast* series instead gave the same bins and the same net to three decimals. |
| a fixed lag | RMS falls monotonically to the scan edge — no minimum, so no lag fits. Net error flat at +2.10 % across ±100 ms. |

---

## 3. The `joint_states` bridge — built, verified, **uncommitted**

`/robot_N/joint_states` was advertised on both the gz and ROS sides but silent in
ROS, because no bridge entry existed. The gz topic carries **`gz.msgs.Model`** —
that *is* what `JointStatePublisher` publishes; the Model envelope holds a
`joint` block per wheel with `axis1 { position, velocity }`. `ros_gz_bridge`
registers `Model → sensor_msgs/JointState`; the symbols are in
`libros_gz_bridge*.so` and the bridge logs the conversion on creation.

Ten lines added inside the per-robot loop in `bridge_topics`, immediately after
the `PosePublisher` entry it is modelled on:

```python
f'/robot_{n}/joint_states'
'@sensor_msgs/msg/JointState[gz.msgs.Model',
```

**Verified live**, all five: `Creating GZ->ROS Bridge: [/robot_N/joint_states
(gz.msgs.Model) -> /robot_N/joint_states (sensor_msgs/msg/JointState)]`, and
`b18_run3` contains ~54 600 messages per robot with names, positions and
velocities populated. The bridges are `Lazy 0` — they forward only once something
subscribes, so their creation line alone is not proof that data flows.

A steady-spin sanity check before the run: velocities ∓1.93939 symmetric to nine
digits, giving ω = (1.9394 × 2) × 0.033 / 0.160 = **0.8000 rad/s**, exactly the
commanded value.

---

## 4. §6 of the previous handoff, inverted: my verification was the thing that lied

The previous session recorded Claude Code reporting edits as applied that were
not. This session Claude Code applied the edit correctly on the first attempt and
produced the diff on request. **The failures this session were all in my own
checks.** Five of them, and they cost more time than the work:

- A ground-truth stationarity gate that compared **empty against empty** and
  printed `IDENTICAL — nothing is driving them`. It asserted nothing. A gate that
  cannot fail is worse than no gate.
- `ros2 topic hz … | head -1` took the "not published yet" warning and discarded
  the rate that followed it — reported three of five publishers dead when all
  five were at 20 Hz.
- `head -20` on the gz joint message cut off the `axis1` blocks and produced a
  confident wrong conclusion about what the publisher emits.
- `grep -c "joint_states@sensor_msgs"` on the installed launch file returned **0**
  while the bridge was demonstrably working — the topic and type are on separate
  source lines, so the string never appears contiguously.
- A CSV column filter on `'err'` found nothing, because the column is `e_deg`.
  The header was printed on the line above.

**Rule for the next session: a check that passes must be capable of failing, and
must assert an expected count or value, not merely run.** Where output is
filtered, the filter is part of the claim and has to be justified.

---

## 5. The measurements, in the order they were made

Each of these is reproducible from the bags with no simulator.

**`run2`, from the audit CSV, wrapped:** `e_deg` swings sign (−151, +16, −48,
−126, +76, +146) — this is error already wrapped into ±180 ° and is *not*
interpretable as accumulated error.

**`run2`, unwrapped increments:** −629 ° over 51 215 ° of true rotation.

**`run2`, yaw read directly from the bag:** odom net −30 485.1 ° against truth
−29 852.8 °, i.e. odom **over**-rotates by 632.3 ° = **+2.12 %**. Two independent
paths agreeing to ~3 ° in 30 000. My earlier "under-reports by 1.23 %" was a sign
error from reading the error column without accounting for the net turn being
negative.

**`run2`, failure-window correlation** (35 failures, 32 mapped into sim time via
`/clock`'s wall→sim pairs; 3 fell before recording began at sim 602.5 s):

| window | truth | odom | error |
|---|---|---|---|
| within 10 s of a failure | −17 747.8 ° | −17 714.5 ° | **−0.19 %** |
| elsewhere | −11 779.1 ° | −12 333.8 ° | **+4.71 %** |

**Degraded navigation did not contaminate the measurement** — the error lives
almost entirely *outside* the failure windows. Treating `Failed to make progress`
as data rather than as an abort was the right call, and the standing abort
criterion would have discarded a good bag. That criterion is still formally
unamended.

**`run2`, by turn rate** (signed, truth-rate accumulation, excess = odom − truth):

| rate deg/s | time s | turned ° | excess ° | exc/sec | exc/deg |
|---|---|---|---|---|---|
| 0–2 | 260.9 | 79.6 | 63.3 | 0.243 | 0.795 |
| 2–5 | 167.8 | 536.5 | 19.7 | 0.117 | 0.037 |
| 5–10 | 223.5 | 1840.7 | −57.8 | −0.258 | −0.031 |
| 10–20 | 268.3 | 4006.2 | −300.7 | −1.121 | −0.075 |
| 20–30 | 237.2 | 5786.6 | −275.9 | −1.163 | −0.048 |
| 30–45 | 212.1 | 7795.4 | −157.8 | −0.744 | −0.020 |
| 45–55 | 114.9 | 5688.7 | 49.1 | 0.427 | 0.009 |
| 55–200 | 445.2 | 25 481.5 | 31.1 | 0.070 | 0.001 |

**The excess changes sign across the bins.** Additive and multiplicative errors
both keep one sign, so neither describes this. At the 45–60 deg/s cap — where
26 000 ° of the 30 000 ° sits — odom tracks truth to −0.31 %. Caution: `turned`
is absolute and `excess` is signed in this table, against a negative net
rotation; the column labels are misleading and should be rewritten before reuse.

---

## 6. The `run3` result — Option B, answered

Net rotation, robot_0, over 574 s and 14 762 ° of true rotation:

| source | net rotation | against |
|---|---|---|
| wheels, via ω = R(ω_R − ω_L)/S | −14 723.2 ° | — |
| odometry, as published | −14 723.5 ° | **+0.002 %** vs wheels |
| ground truth | −14 761.8 ° | wheels are **−0.262 %** vs truth |
| | | odom vs truth: **−0.260 %** |

Parked robot_1, same integration: **0.0000 °**.

Three things follow.

1. **There is no integration bug.** The DiffDrive plugin's published odometry
   reproduces its own wheel kinematics to 0.3 ° in 14 723 — i.e. exactly.
2. **Physical slip is −0.26 %** and that is the whole of the wheel-to-body
   discrepancy in this run. Small, and the right sign for a robot whose wheels
   turn slightly more than its body does.
3. **`run2`'s +2.12 % is not a property of the simulator.** `run3` measures the
   same quantity, through the same code, at −0.26 %.

`run3`'s figure is the corroborated one: the wheels independently confirm it.
`run2`'s has two agreeing derivations but no third source. **Do not cite either
number until §7 is resolved.**

---

## 7. Next session — reconcile `run2` against `run3`

The whole job, and it needs no simulator: both bags are on disk.

Two candidate explanations, both testable:

- **Scale of accumulation.** `run2` covers 51 215 ° over 2329 s; `run3` covers
  14 762 ° over 574 s. If the excess is something that compounds, `run3` may
  simply be too short to show it. **Test:** slice `run2` into 574 s windows and
  measure each independently. If early windows read ≈ −0.26 % and late ones read
  ≫ 0, it accumulates. If every window reads ≈ +2 %, it does not, and the
  difference is not duration.
- **Motion profile.** `run2` spent heavy time in Nav2 spin recoveries; the rate
  table above shows its rotation concentrated at the 45–60 deg/s cap. **Test:**
  build `run3`'s equivalent rate table and compare the two distributions
  bin-for-bin. If `run3` never populated the bins where `run2`'s excess lives, the
  runs are not measuring the same motion.

Run both before proposing a mechanism. A third possibility that neither test
addresses — that one of the two runs has a defect not yet found — stays open
until one of them explains the other.

`run3` also carries joint states for **all five** robots, so the wheel-level
analysis can be repeated per robot, with the four parked ones as a null.

### Also queued, unchanged

- **Commit the bridge edit** (§3) — highest priority, it is uncommitted work.
- **`experiments/runs/b18_run1/README.md`** still contains the GitHub profile
  README instead of the run procedure. `--show-args` regenerates the command
  lines; the launch arguments are now confirmed (§8).
- **`experiments/runs/` still has no README for b16.**
- **The `headless` launch argument's help text is now false.** It claims the GUI
  costs "roughly 25 % of one core and about 7 points of load"; the measured base-
  state saving is 1.6. Fix in the same commit as the b18_run1 README.
- **`yaw_truth_audit.py` imports matplotlib unconditionally** at line 55, so a
  machine without it would die at import before writing any CSV — contrary to the
  previous handoff's description of a graceful-degradation path. Moot here
  (3.6.3 present, `Agg` correctly set) but a real gap.
- **`record_run.sh` cannot record a single-mover run** (§1).
- **Profile `nsk_engine`.** Untouched, still the floor, still never measured.
- **Does the gz *server* render lidar in software?** The `libEGL … driver (null)`
  warnings appear with the GUI off, from `gz-1` — so the server's rendering path
  demonstrably fails to bind the GTX 1050 Ti. Whether five lidars are being
  rasterised on the CPU is still untested and is the strongest remaining
  headroom lead.

---

## 8. Operational facts worth not rediscovering

**Launch arguments, confirmed from `--show-args`:**

- `wander` already defaults to **`false`** — nothing to pass for a single-mover
  run; robots hold spawn poses unless another controller drives them.
- `nav_robots:=[0]` is required by `explore.launch.py`'s own help, not merely a
  wander mute.
- `use_sim_time:=true` is **not** the default on the explore side and must be
  passed.
- `obstacle` defaults true, `obstacle_stop_m` 0.6, `rviz` false both sides,
  `nav2` true, `reachability_precheck` false, `escape_distance` 0.6.

**Teardown.** `pkill -f "gz sim"` does not match `robot_node`, `nsk_engine`,
`parameter_bridge`, or the Nav2 servers. The working form:

```bash
pkill -f "gz sim"; pkill -f robot_node; pkill -f nsk_engine; pkill -f parameter_bridge
sleep 4; pkill -9 -f "gz sim"; sleep 2
ps -eo pid,args | grep -E "gz sim|robot_node|nsk_engine|parameter_bridge|slam_toolbox|controller_server" | grep -v grep || echo clean
```

**A Nav2 stack can outlive its simulator.** After Gazebo was torn down,
`slam_toolbox` and `controller_server` survived — both had `use_sim_time:=true`
and were probably blocked on a `/clock` that would never tick again. They needed
`kill -INT` by PID. The pre-run check must grep for Nav2 servers, not just `gz sim`.

**Ctrl-C does not always orphan the `gz sim` server.** One launch this session
was ended by a misdirected Ctrl-C and every process, `gz-1` included, logged
`process has finished cleanly`. The standing §9 claim is at least not universal.
Teardown before relaunch is still mandatory.

**Under saturation the ROS CLI lies in specific ways:**

- `ros2 topic hz` prints `does not appear to be published yet` **and then reports
  a rate anyway** — read the **last** line, never the first.
- `ros2 topic echo --once` loses discovery races, and once failed with
  `failed to create timer: the given context is not valid` — a client-side
  failure, not evidence about the publisher.
- `top -b -n1` quantizes its first iteration; use `top -b -n2 -d 5` and read the
  second.

**The recorder.** `ros2 bag record` prints on subscription and then stays silent
while writing — silence is healthy, not a stall. `metadata.yaml` is written only
on clean shutdown; if a Ctrl-C misses the window, `kill -INT <pid>` on the
recorder PID is exact. Never `kill -9`, which skips the finalizer. MCAP is
self-describing, so the data survives a hard kill and `ros2 bag reindex -s mcap`
regenerates the sidecar.

**Clipboard, X11 session:** append `| tee /dev/tty | xclip -sel clip` to any
block to capture its output without selecting text. `wl-copy` is also installed
but this session is X11.

**Terminal numbering in these handoffs is Claude's, not Ali's.** Windows do not
persist between sessions and are not labelled. Identify by what is running:

```bash
pgrep -af "swarm_sim.launch|explore.launch|bag record"
```

---

## 9. Loose ends carried forward unchanged

- The **failure mode where Gazebo starts, dies immediately, and all five spawn
  clients wait forever** remains unexplained and open. It did not recur.
- The live-vs-offline `map→odom` disagreement from B1.7 §4 — still unexplained,
  and **now checkable**, since ground truth exists in both bags.
- Robot_0's `explore_robot0.log` from b16 shows `controller_server` loading its
  critics four times. Unexplained.
- Two install trees (`~/Desktop/NSKsim/install` and
  `~/Desktop/NSKsim/ros2_ws/install`). The repo-root one is live. Still a hazard.
- `experiments/logs/` is gitignored, so **both B1.8 bags are untracked and exist
  in exactly one place.**
