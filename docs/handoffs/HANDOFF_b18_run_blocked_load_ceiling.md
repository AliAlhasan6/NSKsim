# HANDOFF — B1.8 run attempted on a fresh boot; blocked at stack one; the run README is lost

Session: 2026-09-10 (early, ~04:30–05:05 local).
Previous handoff: `HANDOFF_b18_ground_truth_session.md`.
Repo `~/Desktop/NSKsim`, HEAD `19267b6` on `main`, pushed. Working tree clean apart
from the three deliberate untracked items (`.vscode/`,
`experiments/wall_affinity.py`,
`ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`).

**No commits this session.** Everything below is diagnosis and log artefacts.

Conventions unchanged: replies 120–130 words, one command block per turn,
blocking/sim/expected-time header on every block, every terminal sources
`/opt/ros/jazzy/setup.bash` then the **repo-root** `install/setup.bash`, any spec
for Claude Code introduced with a **(for Claude Code)** heading, scripts as
downloadable files, Ali runs terminals and commits, Claude Code edits repo files
with manual approval.

---

## 0. One-paragraph state

The B1.8 recording run did not happen, on a machine booted two minutes earlier
with no browser open. That was the condition the previous session said the run
needed. Load reached **29.3 with a single Nav2 stack**, against 33 last night,
and `controller_server` logged **`Failed to make progress` twice while only
robot_0 was under Nav2** — the agreed abort criterion, fired at stack one. Five
stacks are not available on this hardware in this configuration. Separately,
`experiments/runs/b18_run1/README.md` — the operating document the previous
handoff pointed to — **contains the GitHub profile README, not the run
procedure**, and there is no other version in history, so the procedure is gone.
Ground truth was re-verified live and is sound. One new and unexplained failure
mode appeared: a launch in which the Gazebo server started and died immediately,
leaving all five spawn clients waiting forever.

---

## 1. The run procedure is unrecoverable

`git show 19267b6 --stat` shows one file, 112 insertions, at
`experiments/runs/b18_run1/README.md`. All 112 lines are the GitHub profile
README (`<h1 align="center">Ali Alhasan</h1>`, ORCID badge, NSK pipeline table,
project grid). `git log --all -- experiments/runs/b18_run1/` returns that single
commit, so there is nothing earlier to restore. The wrong content was staged;
the file was not corrupted afterwards.

The commit message describes what was intended:

> Records the exact commands, the two arguments that default wrong for a
> recorded run (nav_robots, use_sim_time), the abort criteria under CPU
> starvation, and the colcon install-tree failure mode.

Everything in that list is recoverable from the previous handoff except the
exact command lines, and `--show-args` regenerates those. **Open item:** rewrite
that file and replace the committed content.

**Check separately whether the profile README reached its own repo**
(`AliAlhasan6/AliAlhasan6`). If it was written to the NSKsim path by mistake, it
may be missing where it belongs.

### Launch arguments confirmed this session (from `--show-args`)

- `swarm_sim.launch.py`: `venv_site_packages`, `seed`, `csv_path`,
  `nav_robots` (default `[]`), `obstacle` (default true), `obstacle_stop_m`
  (0.6), `rviz` (false), `wander` (false). **There is no headless argument.**
- `explore.launch.py`: `robot_id`, `slam_params_file`, `nav2_params_file`,
  `nav2` (true), `reachability_precheck` (false), `escape_distance` (0.6),
  `rviz` (false), `namespace`, `use_sim_time` (**false**), `params_file`,
  `autostart`, `use_composition`, `container_name`, `use_respawn`, `log_level`.

Working recorded-run invocations:

```
ros2 launch nsk_swarm swarm_sim.launch.py nav_robots:=[0,1,2,3,4]
ros2 launch nsk_swarm explore.launch.py robot_id:=<N> use_sim_time:=true
```

---

## 2. The load ceiling — the finding that ends the session

Measured on a 4-core machine, 30 minutes after boot, no browser:

| state | load |
|---|---|
| knowledge layer only (5 robots, no Nav2) | 17.6 |
| plus one SLAM + Nav2 stack on robot_0, GUI running | **29.3** |
| same, after killing `gz sim gui` | 24.4 |

For comparison, the previous session measured ~20 and 33 for the first two rows,
with Chrome resident. **The fresh boot bought about four points of load, not the
headroom the run needs.**

`controller_server` logged `Failed to make progress` **twice with one stack
active**. Last night the same string appeared under five-stack load. It now
appears at stack one.

Top consumers at that moment: `nsk_engine` **105–119 %** (the single largest, by
a wide margin), `gz sim server` 30 %, `gz sim gui` 25 %, each `robot_node`
21–24 %.

SLAM was healthy throughout — `/robot_0/map` at exactly 0.500 Hz, resolution
0.05. **This is the asymmetry the previous session established: healthy SLAM is
not evidence that a run is sound.** The controller degrades first. It held again
tonight.

### The GUI is the strongest lead for tomorrow

`gz sim gui` was consuming 25 % and rendering the five-robot scene. Killing it
returned ~7 points of load with no other change. It is not mentioned anywhere in
the B1.6/B1.7 accounts, and `swarm_sim.launch.py --show-args` lists **no headless
option**, which means the GUI has been running unconditionally on every run
including b16 — the same cost class `rviz` is banned for during timed runs.

**Proposed work item (non-sim):** add a `headless` argument to
`swarm_sim.launch.py` defaulting to false so previous runs reproduce exactly,
passing `-s` (server-only) to `gz sim` when true. This is the one identified
saving that does not change robot behaviour or the recorded topic set. Whether
7 points is enough for five stacks is unknown — it is not, on its own, but it is
free.

---

## 3. Ground truth re-verified — no action needed

Run `1c` came up clean: five entities in `gz model --list`, five
`/model/robot_N/pose` topics on both sides of the bridge.

| robot | ground truth (x, y, z) | `DOT_POSES` |
|---|---|---|
| 0 | 0.8584227076830406, 0.2800000050593877, 0.010008360534952663 | 0.86, 0.28 |
| 1 | −0.00157729231695838, 0.900000005059387, 0.010008360534952635 | 0.00, 0.90 |
| 2 | −0.861577292316959, 0.28000000505938766, 0.010008360534952654 | −0.86, 0.28 |
| 3 | −0.531577292316959, −0.729999994940612, 0.010008360534952637 | −0.53, −0.73 |
| 4 | 0.5284227076830413, −0.729999994940611, 0.010008360534952635 | 0.53, −0.73 |

Two samples 42 s apart were **identical to all sixteen digits** on all five, as
required with `wander` false and no explorer running. `z` is the spawn `-z 0.01`.
The x offset is **−1.5772923 mm, byte-identical across all five robots** — the
same uniform constant measured last night, so still not a per-robot error. Rate
19.37–19.57 Hz against a configured 20.

This is now the standard post-spawn check: identical poses across two samples
proves nothing is driving the robots, and matching `DOT_POSES` proves attribution.

---

## 4. New failure mode: server starts and dies, spawns wait forever

Sequence, `bringup_run1b.log`:

1. `[INFO] [gz-1]: process started with pid [10109]` at 04:42.
2. Clock bridge, all 30 per-robot bridges and the `/tf` bridge come up normally,
   including all five `/model/robot_N/pose` lines.
3. All five `create` clients then log
   `Waiting for service [/world/knowledge_world/create] to become available ...`
   every 5 s, **for over 5 minutes, forever**.
4. At the time of diagnosis `ps` showed **no `gz sim` process at all**, while all
   five `create` processes were alive in state `Sl+`.
5. `gz service --list` → `Service call to [/gazebo/worlds] timed out`.
   `/clock` → nothing.
6. `~/.gz/sim/log/` had **no directory for this launch**; the newest were from
   the previous run.

So the server died before it could open its own log or advertise anything. No
segfault, no OOM in `dmesg`, no `died`/`exit code` line in the launch log.

**This is unexplained and should be treated as open.** The obvious symptom — the
bridges and robot nodes come up looking healthy regardless — is a trap: their
log lines say nothing about whether the world exists. The reliable check is
`Entity creation successful` × 5, or `/clock` producing.

### What is ruled out

Everything downstream of the server, checked in isolation with no ROS involved:

- Bare `gz sim -v 4 -r -s` on `worlds/knowledge_world.sdf` loads fine — 8692
  bytes of log, physics, all services advertised.
- `/world/knowledge_world/create` is **up within 5 s**, so the 5-minute wait was
  never the service being slow.
- Empty world and `shapes.sdf` both run to 200 iterations, exit 0.
- Manual `ros2 run ros_gz_sim create` with **run1b's own `/tmp` SDF**, same name
  and coordinates, against a healthy server: `Entity creation successful`,
  **instant**, exit 0, `/model/robot_0/pose` advertised. `grep -c PosePublisher`
  on that SDF returns 1, and it is 11493 bytes — the plugin is present.

So: not the world file, not the SDF, not the plugin, not the create service, not
the GPU (nvidia-smi clean, 244 MiB used).

### What was proposed and NOT confirmed

I suggested a second server from run1 was holding the world name. **That does
not fit**: two live servers would both appear in `ps`, and during the wait there
was no gz process at all. Do not carry the two-server story forward as
established.

### Related, and confirmed: Ctrl-C orphans the server

After run1, `ps` showed `gz sim server` **still alive at 305 s elapsed** with
`robot_3` still in the world, five minutes after the launch was interrupted. The
gz log confirms it only exited on signal 15 from the kill block.

**Teardown discipline is therefore mandatory before every relaunch.** The
working sequence:

```bash
pkill -f "gz sim"; sleep 4; pkill -9 -f "gz sim"; sleep 2
ps -eo pid,args | grep -E "gz sim|create|parameter_bridge|robot_node|nsk_engine" | grep -v grep || echo clean
gz service --list 2>&1 | grep -c knowledge_world   # must be 0
ros2 node list | wc -l                              # must be 0
```

This may also explain the "only 4 robots spawned, the 5th was glitching"
observation from run1 — the world was not necessarily in the state the launch
intended.

---

## 5. Retractions this session, so they are not re-argued

- **"The Gazebo server is silent even with `-v 4`, so it dies before parsing"** —
  **wrong**. `tee | tail -40` buffers; only `Terminated` reached the screen. The
  log on disk had 8692 bytes of normal startup. Any future solo-server test uses
  `stdbuf -oL -eL` and redirects to a file, then reads the file.
- **"The orphaned server explains run1b"** — **not established**, see §4.
- **"The Gazebo server failed in run1"** — **wrong**. Its own log shows four
  minutes of healthy operation, five entities, `PosePublisher` loaded for entity
  42, lidar advertised, exiting only on our signal 15.

---

## 6. Next session

The previous handoff's plan assumed a fresh boot would deliver headroom. It did
not, so the first decision is which of these to take.

1. **Headless GUI argument** (non-sim, ~7 points of load). §2. Cheapest real
   change available, and it needs no simulator to write.
2. **The analysis script**, still missing and still the bottleneck between a
   recorded bag and a result: odom yaw vs true yaw is candidate (a), scan
   timestamps vs true pose is candidate (b). Nothing in `experiments/analysis/`
   does either. Written against b16's structure, it means the bag gets analysed
   in the session it is recorded. This was the previous handoff's own
   recommendation for a busy machine and it is still right.
3. **Rewrite `experiments/runs/b18_run1/README.md`** (§1), and check the profile
   README reached its own repo.
4. **A reduced-robot run.** Ground truth is per-model, so a two- or three-robot
   recording still separates (a) from (b) for the robots that move. It is not
   b16's configuration, so it is a different run and must be labelled as one —
   not a b16 repeat.
5. **Investigate `nsk_engine` at 105–119 %.** It is the single largest consumer
   and it sets the floor that everything else stacks on top of. No profiling has
   ever been done on it. If the knowledge layer alone costs 17.6 load, no amount
   of Nav2 tuning reaches five stacks.

Ali's stated plan is to reboot and debug tomorrow. Worth saying plainly: tonight
**was** the fresh-boot condition, so a further reboot is unlikely to change the
ceiling on its own. The load numbers in §2 are the fresh-boot numbers.

---

## 7. Artefacts on disk (uncommitted)

In `experiments/logs/b18/`:

| file | what |
|---|---|
| `bringup_run1.log` | first launch; 5 entities spawned, later Ctrl-C'd, server orphaned |
| `bringup_run1b.log` | the failure of §4 — server dies, 5 clients wait forever |
| `bringup_run1c.log` | clean launch, the one ground truth was verified against |
| `gz_solo.log` | bare server on knowledge_world, healthy, 8692 bytes |
| `gz_solo2.log` | same with `stdbuf`, create service up in <5 s |
| `gz_empty.out` / `.err` | empty world, 200 iterations, exit 0 |
| `stack_robot0_run1.log` | robot_0 SLAM+Nav2, clean 12 s bringup, then 2× `Failed to make progress` |

Decide whether any of these are worth keeping before the next run overwrites the
directory. `bringup_run1b.log` and `stack_robot0_run1.log` are the two with
evidence in them.

Disk: 49 G free, unchanged. `b16_run1` 2.4 G, `b16_slamin` 794 M, `b16_r0late`
487 M; the two derived bags are regenerable from `b16_run1` in ~100 s.

---

## 8. Loose ends carried forward unchanged

- The live-vs-offline `map→odom` disagreement from B1.7 §4 is still unexplained
  and becomes checkable once ground truth is recorded.
- Robot_0's `explore_robot0.log` from b16 shows `controller_server` loading its
  critics four times — four restarts during that run, not in the B1.6 account.
  Unexplained.
- Two install trees exist (`~/Desktop/NSKsim/install` and
  `~/Desktop/NSKsim/ros2_ws/install`). The repo-root one is the live one and its
  symlink chain was verified this session: `install/…/launch/*.launch.py` →
  `build/nsk_swarm/launch/*` → `ros2_ws/src/nsk_swarm/launch/*`. Edits reach the
  runtime. Having both trees remains a hazard.
- `experiments/runs/` still has no README for b16.
