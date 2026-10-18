# HANDOFF — B1.8 step one: ground truth built and verified; the run is blocked on machine load

Session: 2026-09-10 (late, ~00:50–02:45 local).
Previous handoff: `HANDOFF_phaseB1_7_offline_slam_fit_session.md`.
Repo `~/Desktop/NSKsim`, HEAD `19267b6` on `main`, **pushed**. Working tree clean
apart from the three deliberate untracked items (`.vscode/`,
`experiments/wall_affinity.py`,
`ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`).

Replies in the next session: 120–130 words, one command block per turn,
blocking/sim/expected-time header on every block, **every terminal sources
`/opt/ros/jazzy/setup.bash` then the install setup — including blocks that only
run `gz`/`ros2` queries**, any spec for Claude Code introduced with a
**(for Claude Code)** heading, scripts as downloadable files, Ali runs terminals
and commits, Claude Code edits repo files with manual approval. Nothing written
to `/tmp` by hand.

---

## 0. One-paragraph state

§8 of the B1.7 handoff asked for per-robot Gazebo ground-truth pose bridged and
bagged alongside `/odom` and `/tf`. **That is built, verified at runtime, and
pushed.** Gazebo's per-world pose topics turned out to be unusable — they carry
no entity names — so the route is a `PosePublisher` system injected into each
robot's SDF, publishing on `/model/robot_N/pose` where the topic name itself is
the attribution. Five topics, both sides of the bridge, 19.95–20.00 Hz against a
configured 20, sim-time stamps, each pose matching its own `DOT_POSES` entry.
A recorder (`experiments/slam/record_run.sh`) records b16's exact 25 topics plus
the five pose topics and refuses to start if any is unadvertised. **The
recording run itself did not happen**: the machine was at load 33 on 4 cores
with one Nav2 stack, and recording under starvation would produce a bag whose
odometry error is dominated by candidate (b) — the very thing the run exists to
separate from (a). The run needs a fresh boot. Procedure is written up at
`experiments/runs/b18_run1/README.md`; read that, not this, when running.

---

## 1. Commits this session (all on `main`, all pushed)

| hash | what |
|---|---|
| `c51e190` | Public capability-overview figure (light/dark SVG + 2× PNG in `docs/`, with its generator `make_public_overview.py`), and a README restructure that leads with it and states plainly which layer is shipped (knowledge sharing, v1.1.0, still FB15k-237) and which is under construction (perception). |
| `662abe2` | `PosePublisher` pair appended to `make_namespaced_burger_sdf`'s substitution list; `/model/robot_N/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose` added to the per-robot bridge loop; docstring updated; root-level `build/`, `install/`, `log/` gitignored. |
| `9be57ac` | `experiments/slam/record_run.sh` — b16's topic set plus ground truth, with an unadvertised-topic pre-flight. |
| `19267b6` | `experiments/runs/b18_run1/README.md` — the run procedure. |

Push history: `0a08a5a..c51e190`, then `c51e190..9be57ac`, then
`9be57ac..19267b6`. The three B1.7 commits (`bab60cc`, `ade6860`, `f2638a4`)
went out with the first of those; nothing is local.

---

## 2. Ground truth — how it works and why this route

### The two routes that failed

`/world/knowledge_world/pose/info` and `.../dynamic_pose/info` both exist and
both bridge cleanly. `dynamic_pose` runs at ~46 Hz, 3.21 KB/msg, ~155 KB/s.
Both are **useless**: bridged to `TFMessage`, every `child_frame_id` comes
through **empty** (40 entries on `dynamic_pose`, 89 on `pose/info`). The gz→ROS
converter fills that field from the Pose_V entry's name, and Gazebo publishes
these identified by numeric entity id only. A pose that cannot be attributed to
a robot is not ground truth. Do not revisit these.

### The route that works

`gz::sim::systems::PosePublisher` in the robot SDF. It publishes on a per-model
topic, so **the topic name is the attribution** and the empty-name problem never
arises. The spawn already passes `-name robot_{n}`
(`swarm_sim.launch.py:470`), which is what the plugin reads at Configure time.

Injected as one more `(old, new)` pair in `make_namespaced_burger_sdf`,
anchored on `</model>` — the one and only occurrence in the stock burger model
(`/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf:406`),
which preserves the function's documented "each replaced string occurs exactly
once" contract. `str.replace` is unbounded; a second occurrence would silently
inject a second plugin. Note the pair **appends** rather than rewrites, unlike
the eight above it.

Config: `publish_model_pose` true, every other `publish_*` false,
`use_pose_vector_msg` false, `static_publisher` false, `update_frequency` 20.

Bridged gz→ROS only as `PoseStamped`, not `Pose`: with `use_pose_vector_msg`
false the wire type is a single `gz.msgs.Pose`, and taking it as `PoseStamped`
preserves the sim timestamp a bare `Pose` would drop. The conversion is declared
in the installed headers
(`ros_gz_bridge/convert/geometry_msgs.hpp:143-144`).

### Verification (runtime, not static)

Five distinct topics on `gz topic -l` and `ros2 topic list`. `hz` on
`/model/robot_0/pose`: 19.94–20.00 across 166 samples. One echoed message:
`frame_id: knowledge_world`, `stamp` advancing on sim time.

| robot | ground truth (x, y) | `DOT_POSES` |
|---|---|---|
| 0 | 0.8584, 0.2800 | 0.86, 0.28 |
| 1 | −0.0016, 0.9000 | 0.00, 0.90 |
| 2 | −0.8616, 0.2800 | −0.86, 0.28 |
| 3 | −0.5316, −0.7300 | −0.53, −0.73 |
| 4 | 0.5284, −0.7300 | 0.53, −0.73 |

y matches to 8 decimal places; x sits ~1.6 mm off on **all five, same sign and
magnitude** — a uniform constant that cancels in any drift measurement, not a
per-robot error. Worth not re-deriving; worth flagging if it ever becomes
per-robot.

---

## 3. The recorder

`experiments/slam/record_run.sh`, `RUN=<name>` (default `b18_run1`), output
`experiments/logs/<prefix>/<RUN>`.

30 topics: `/clock /tf /tf_static /kg_share /nsk/convergence`, then per robot
`/odom /scan /map /map_metadata /model/robot_N/pose`. The first 25 are b16's
exact set, read out of `b16_run1`'s own metadata rather than reconstructed —
**there was no record command anywhere in the repo**, b16 was captured by hand.

`/robot_N/map` and `/map_metadata` are kept deliberately. The offline pipeline
rebuilds maps from scans and ignores them, and they are most of the bag's size,
but dropping them breaks §8's identical-recording requirement and removes the
live-vs-offline `map→odom` comparison B1.7 left unexplained.

Ground-truth cost is negligible: five `PoseStamped` at 20 Hz ≈ 6 KB/s, against
`dynamic_pose`'s 155 KB/s.

**The pre-flight matters.** `ros2 bag record` skips topics that are not yet
advertised, silently, with exit 0. A 60 s smoke test recorded 20 of 30 — the
missing ten were exactly `/robot_N/map` and `/map_metadata`, because SLAM had
never been launched. The script now compares the requested list against
`ros2 topic list` once and aborts naming what is missing.

Known limit, deliberately not fixed: the guard proves topics are **advertised**,
not **publishing**. A `--once` echo per map topic would close that, but it
blocks until a map arrives, which on paced bringup is minutes — a second silent
wait in front of a multi-hour recording. If a thin bag ever appears, the fix
belongs in the run procedure, not the recorder.

---

## 4. The build defect — read this before blaming code

A `log/COLCON_IGNORE` file, created during the 8 September build, made
`colcon build --packages-select nsk_swarm` print `Finished <<< nsk_swarm` while
**installing nothing**. Separately, `install/nsk_swarm/share` was a plain copy
dated **7 August**, not a symlink, so `--symlink-install` had never applied to
it.

Combined effect: for over a month, `ros2 launch` loaded 7 August launch code
regardless of what was edited. Deleting the stale install tree removed the
crutch and produced "package not found"; deleting `log/COLCON_IGNORE` made the
build work immediately — launch, worlds, config and rviz all symlinked in.

**The audit is done and came back clean.** No commit since 7 August touched an
installed `share/` asset:

- `swarm_sim.launch.py` — only `4d7ee22` (itself dated 7 August, so it *is* the
  installed version) before this session's `662abe2`
- `explore.launch.py`, `experiments/nav/nav2_robot0.yaml`,
  `slam_robot0.yaml`, `worlds/knowledge_world.sdf`,
  `config/nsk_swarm_params.yaml` — unchanged since 8 August

So **b16 ran the code its handoffs describe, and B1.6/B1.7 conclusions stand
unqualified.** The stale tree was a latent trap that would have silently
invalidated the *next* run, not a past one. `generate_robot_params.py --check`
also passes 8/8 identical.

Colcon invoked from the repo root writes `build/`, `install/` and `log/` there
rather than into `ros2_ws/`. Both trees now exist; the repo-root one is what the
working sim used tonight, so **source `install/setup.bash` from the repo root**.
All three are gitignored as of `662abe2`.

Symptoms to recognise later: a source edit has no runtime effect; no new
directory under `log/` after a build; `install/nsk_swarm/share` missing or
old-dated. Recovery is in §5 of the run README.

---

## 5. Why the run did not happen

Load average on the 4-core machine, 1h53m uptime with Chrome resident:

- knowledge layer alone (nsk_engine 119 %, five robot_node ~20–26 % each,
  convergence monitor, two Gazebo ruby threads): **~20**
- plus one Nav2 + SLAM stack on robot_0: **33**

`stack_robot0.log` showed repeated `Failed to make progress` (controller_server)
and `Failed to get result for follow_path in node halt!` (bt_navigator) — the
signature of the 20 Hz control loop being starved, not a navigation defect.

**The asymmetry is the useful finding.** Under that load SLAM stayed healthy:
`/robot_0/map` at its configured 0.5 Hz, drop census 1 (the benign first scan
before any transform existed — same signature as b16). Healthy SLAM is
therefore **not** evidence a run is sound. The controller degrades first.

This is why the run was stopped rather than recorded. A bag taken at load 33
would have odometry error dominated by CPU starvation, which is candidate (b),
and the run's entire purpose is to separate (b) from (a).

B1.6 recorded the same class of thing — Chrome was the fifth-stack gate there
too.

---

## 6. Settled this session, so it is not re-argued

**The in-place spinning question is closed.** B1.7 §7 flagged odometry yaw error
accumulated during heavy spinning as candidate (a), and asked whether the
spinning is behaviour to keep or a recovery loop to limit. Measured from the b16
explorer logs — `behavior_server` "Running spin", 1.57 rad each — against §7's
odom totals:

| robot | spins | recovery ° | odom total ° | share |
|---|---|---|---|---|
| 0 | 80 (79 completed) | 7 200 | 42 891 | 17 % |
| 1 | 37 | 3 330 | 14 090 | 24 % |
| 2 | 11 (9 completed) | 990 | 19 551 | 5 % |
| 3 | 20 (19 completed) | 1 800 | 17 359 | 10 % |
| 4 | 4 | 360 | 15 322 | 2 % |

Recovery is 2–24 %. Three-quarters or more of every robot's rotation is ordinary
navigation — DWB goal-heading rotation and path following. Limiting recovery
would remove at most a fifth of the yaw error, so it is **not** the lever on (a),
not worth its own commit or its own run, and **behaviour goes into B1.8
unchanged**.

Two side observations from the same data, not pursued:

- Robot_2 completed 9 of 11 spins, robot_0 79 of 80. An interrupted rotation is
  where odometry yaw error lands hardest — small numbers, but the sharpest
  offline evidence for (a) that exists before ground truth.
- Robot_0's `explore_robot0.log` shows `controller_server` loading its critics
  **four times**, i.e. four restarts during b16. Not in the B1.6 account of that
  run. Unexplained.

This also weakens B1.7 §7's side-finding: robots 1–4 spinning 39–54 revolutions
"while barely translating" was attributed to recovery or explorer behaviour;
recovery explains only a small part of it. Carry as corrected, not settled.

---

## 7. Launch facts established (these cost time; do not re-derive)

- **`explore.launch.py` is per-robot.** `robot_id` selects one namespace and one
  param-file pair. There is no five-robot argument. "Paced bringup" means five
  separate invocations, one at a time, watching load between each.
- **`swarm_sim.launch.py nav_robots` defaults to `[]`** — no robot's wander
  driver is muted, so Nav2 has authority over none of them. It is fixed at
  launch and cannot be changed afterwards. A recorded run needs
  `nav_robots:=[0,1,2,3,4]`.
- **`explore.launch.py use_sim_time` defaults to `false`.** Every recorded run
  needs it true or stamps are wall time and the offline pipeline cannot work.
- `rviz` defaults false and must stay false during a timed run — its rendering
  corrupts the RTF measurements.
- `worlds/knowledge_world.sdf` lives at
  `ros2_ws/src/nsk_swarm/worlds/knowledge_world.sdf`.

---

## 8. Next session

1. **Fresh boot, no browser.** Then work from
   `experiments/runs/b18_run1/README.md` — preconditions table first, then §1.
   That file, not this one, is the operating document.
2. Record `b18_run1`.
3. Pipeline unchanged: `strip_bag_for_offline_slam.py` → `run_offline_maps.sh`
   (rate 2.0, per-robot `DUR_<N>` from the bag's own anchors) →
   `fit_world_transform.py --run … --spawn-rev …`. Coverage and pairwise overlap
   **only** over robots clearing 0.80.
4. Then the measurement the run exists for: odom yaw vs true yaw over time is
   (a); scan-vs-true-pose consistency is (b).

**Non-sim work available if the machine is busy or the reboot is not happening:**
the analysis script for step 4 does not exist. Nothing in `experiments/analysis/`
compares odom yaw against true yaw, or scan timestamps against true pose.
Writing it against b16's structure now means the bag gets analysed in the same
session it is recorded.

---

## 9. Loose ends

- Disk: 49 G free. `b16_run1` 2.4 G, `b16_slamin` 794 M, `b16_r0late` 487 M. The
  two derived bags are reproducible from `b16_run1` in ~100 s if space is needed.
- `experiments/runs/` has a README for `rung3b` and now `b18_run1`; b16 has none,
  which is why its launch sequence had to be reconstructed this session.
- Root-level `build/` and `install/` are the trees tonight's sim used. Two
  install trees now exist (`~/Desktop/NSKsim/install` and
  `~/Desktop/NSKsim/ros2_ws/install`); why colcon was ever invoked from the repo
  root is not established, and having both is a hazard of exactly the kind §4
  describes.
- The `b18_smoke` bag was deleted. `experiments/logs/b18/` holds only
  `bringup.log` and `stack_robot0.log` from the abandoned attempt.
- The live-vs-offline `map→odom` disagreement from B1.7 §4 is still unexplained
  and becomes checkable once ground truth exists.

---

## 10. Retractions this session, so they are not re-argued

- "The `</model>` substitution never fired" — **wrong**. I read the newest
  tempfile, which was from a pre-build launch. The 11493-byte file (556 bytes
  larger, i.e. the plugin) proved it did.
- "`nav2_robot0.yaml` has no recovery section, so the spinning cannot be Nav2
  recovery" — **wrong**. `behavior_server` is at line 261 with `Spin` and
  `BackUp` loaded; my grep pattern missed it. Spin recovery did fire, on robots
  0–3.
- "`data_files` globs resolve against the build directory, so nothing installs" —
  **wrong twice over**: the command log shows `cwd` is the package source, and
  the log being read was from the 8 September build, not that night's.
  `latest_build` had not moved.
- "The 20-of-30 smoke bag means the recorder is defective" — **wrong**. SLAM was
  never launched in that test; the ten missing topics did not exist.
- "Gazebo's per-world pose topic is the cheap route to ground truth" — **wrong**,
  and the reason is worth keeping: no entity names, so no attribution (§2).
