# HANDOFF — the analysis script exists and is pushed; headless argument added; the load ceiling is confirmed real

Session: 2026-09-10 late → 2026-09-11 early (~23:00–00:10 local).
Previous handoff: `HANDOFF_b18_run_blocked_load_ceiling.md`.
Repo `~/Desktop/NSKsim`, HEAD `840b0ba` on `main`, pushed. Working tree clean
apart from the three deliberate untracked items (`.vscode/`,
`experiments/wall_affinity.py`,
`ros2_ws/src/nsk_swarm/test/test_wall_affinity.py`).

**Two commits this session**, both pushed:

- `d3ad22a` — `analysis: yaw_truth_audit.py, discriminate odom yaw error from stamp skew`
- `840b0ba` — `launch: headless argument for swarm_sim, GUI off for recorded runs`

Conventions unchanged: replies 120–130 words, one command block per turn,
blocking/sim/expected-time header on every block, every terminal sources
`/opt/ros/jazzy/setup.bash` then the **repo-root** `install/setup.bash`, any spec
for Claude Code introduced with a **(for Claude Code)** heading, scripts as
downloadable files, Ali runs terminals and commits, Claude Code edits repo files
with manual approval.

---

## 0. One-paragraph state

No simulator run was attempted for the science. Both non-sim items from the
previous handoff's §6 are now done and on `origin/main`: the analysis script that
sat between a recorded bag and a result, and a `headless` argument for
`swarm_sim.launch.py`. The headless saving is **1.6 points of load, not the 7 the
previous handoff estimated** — the 7 was measured with a Nav2 stack running and
does not transfer to the knowledge layer alone. The GUI was found to be rendering
**in software** on a machine with a GTX 1050 Ti, which is why it cost CPU at all.
Separately, my own hypothesis that the load figures were an I/O artefact was
**tested and falsified**: the machine is genuinely CPU-saturated. The ceiling
stands exactly where the previous session put it. The remaining lever is the run
design, not the hardware, and the next session should record a single-mover run.

---

## 1. `yaw_truth_audit.py` — committed as `d3ad22a`

`experiments/analysis/yaw_truth_audit.py`, 487 insertions with the
`odom_yaw_rate.py` help-string change in the same commit.

Reads `/robot_N/odom`, `/robot_N/scan`, `/model/robot_N/pose` from a b18 bag.
Header stamps only, never bag receive time. Three measurements:

- **(a)** odometry yaw error against truth. Deltas from each robot's own first
  odom sample, not absolute yaws, because the odom frame starts at the spawn
  pose. Reports final error, max, RMS, and **degrees of error per 100 degrees of
  true rotation** — the last is the one that matters, since it normalises against
  how much the robot actually turned.
- **(b1)** empirical scan-to-pose stamp gap: signed gap from each scan to the
  nearest ground-truth stamp and the nearest odom stamp. Mean, median, p95, max.
  **A systematic nonzero mean is the finding.**
- **(b2)** sensitivity sweep: for skews of ±25/50/100/200 ms, how much apparent
  rotation that skew would inject *given the motion actually recorded*. This is
  the discriminator — it converts a stamp gap into degrees of map error.

Outputs `yaw_truth_audit.csv`, `yaw_truth_scans.csv`, `yaw_truth_summary.csv`,
and per-robot PNGs, all into `--out`.

### What is verified, and what is not

**Verified:** the abort path. Run against `b16_run1`, which predates `662abe2`
and has no pose topics, it prints the missing topic by name and exits 1 with no
partial output written:

```
FATAL: topic(s) not in the bag, so this bag cannot answer the question it was
recorded for:
         /model/robot_0/pose
EXIT=1
```

**Not verified: everything else.** The interpolation, the yaw unwrap, the (b2)
sweep and the plotting have never run on real messages, because no bag with
ground truth exists yet. **The first b18 bag is also this script's first
functional test.** Budget for fixing it in the session that records the run, not
after. This is the expected cost of writing it before the run rather than the
argument against having done so.

### Plotting dependency

Decided but **not yet checked** — the matplotlib availability command was
approved and its output never pasted. Intended behaviour: use **system
`python3`**, not `nsk_env`, since bag reading needs `rosbag2_py` from
`/opt/ros/jazzy` and the venv will not see it unless built with system site
packages. If matplotlib is missing, write the three CSVs, skip the PNGs, print
one warning naming matplotlib. **Do not install it and do not switch
environments.** The CSVs are the deliverable. Confirm this is what the script
actually does before the run.

---

## 2. The `headless` argument — committed as `840b0ba`

`headless`, default `false`, on `swarm_sim.launch.py`.

Built as an **`OpaqueFunction`** modelled on `_rviz_actions`, not as a
substitution inside the `cmd` list. The reason matters: a `PythonExpression`
evaluating to `''` puts a **real empty argv element** into
`gz sim -r '' <world>`, so "earlier runs reproduce exactly" would have depended
on `gz sim` tolerating an empty positional. The `OpaqueFunction` emits two
literal argv lists and makes the claim true by construction:

- `headless:=false` → `['gz', 'sim', '-r', world_file]`, byte-identical to
  pre-change.
- `headless:=true` → `['gz', 'sim', '-s', '-r', world_file]`.

It accepts `true` or `1`, matching `_rviz_actions`'s own test rather than being
stricter, so the two boolean launch arguments behave the same way.

**Verified both ways.** Default: five `Entity creation successful`, GUI window
up. `headless:=true`: five `Entity creation successful`, exactly one
`gz sim -s -r …` in `pgrep`, no window.

### The saving is 1.6 points, not 7

Knowledge layer only, five robots, no Nav2, 180 s after launch:

| state | load |
|---|---|
| with GUI (previous session, fresh boot) | 17.6 |
| headless, this session | **15.98** |

The previous handoff's 7 points came from killing `gz sim gui` **while a Nav2
stack was running**. It does not transfer to the knowledge layer alone. Take
1.6 as the figure for the base state; the saving under a stack is presumably
still larger, but that is untested and should not be assumed.

The argument is still worth having — it is free, it removes a GUI from every
recorded run, and it costs nothing when off — but it does **not** change the
five-stack conclusion.

### The GUI was rendering in software — new, and the open lead

The default launch logs, repeatedly:

```
libEGL warning: pci id for fd 93: 10de:1c8d, driver (null)
libEGL warning: egl: failed to create dri2 screen
```

`10de:1c8d` is the GTX 1050 Ti. `driver (null)` and the dri2 failure mean EGL
never bound the GPU, so the GUI has been rendering on the CPU on every run
including b16. That is why a GUI cost load at all.

**Open, and the strongest lead left in this area:** whether the *server's* lidar
sensor rendering falls back the same way. Five robots' lidars rendering in
software would dwarf anything the GUI cost. `nvidia-smi` read 244 MiB last night,
which is consistent with the GPU being nearly idle but proves nothing on its own.
This is untested. It is a hypothesis, not a finding.

---

## 3. The load ceiling is real — my own hypothesis falsified

I proposed that load ~16 against ~71 % apparent CPU utilisation meant the load
average was counting blocked threads rather than runnable ones, and that the
"ceiling" was therefore measuring the wrong thing. **Measured, and wrong.**

With the knowledge layer up, headless:

```
vmstat 5 4    →  r = 12, 9, 12, 15    b = 0 throughout
                 cs = 28002, 28509, 31545 per second
                 us = 87–88, id = 6–7
ps -eLo state →  R: 16 threads    D: 1 thread
top (2nd iter) →  89.9 us, 5.8 sy, 4.0 id
```

`b = 0` and a single D-state thread mean nothing is blocked on I/O. Sixteen
runnable threads against four cores is a real queue. The 71 % figure was my
error: it came from summing per-process `TIME+`, which misses short-lived and
multi-threaded work. `%Cpu(s)` at 90 us / 4 id is the truthful number.

**The machine is saturated. The previous session's account of the ceiling stands
unamended.**

### Also: `top -b -n1` quantizes — do not use it

The first `top` iteration reported 166.7, 33.3, 25.0, 25.0 % — thirds, an
artefact of the sampling window, not measurements. The second iteration at a real
5 s interval gave:

| process | %CPU |
|---|---|
| `nsk_engine` | 94.2 |
| `robot_node` ×5 | 54.3, 51.3, 27.4, 26.8, 26.2 |
| `gz sim server` (shows as `ruby`) | 34.2 |

**Every future load measurement uses `top -b -n2 -d 5` and reads the second
iteration**, or reads `TIME+` deltas across a known interval. The single-shot
form has probably contaminated earlier readings too.

`nsk_engine` at ~94–107 % is still the single largest consumer and still has
never been profiled. It sets the floor everything else stacks on.

---

## 4. Scan matcher window — provenance now recorded in the code

The b16 angular search window is **±20 degrees**, and this is now stated in both
`yaw_truth_audit.py`'s docstring and `odom_yaw_rate.py`'s `--search-deg` help.

`coarse_search_angle_offset` is **pinned explicitly at 0.349 rad**, not left to
the slam_toolbox default, in:

- `experiments/nav/slam_robot{0,1,2,3,4}.yaml:113` — online
- `experiments/slam/offline_mapping.yaml.template:97` — offline template
- `experiments/logs/offline_mapping_b16_slamin_robot_{0..4}.yaml:97` — the
  generated configs for b16 itself

Both SLAM paths agree, which is what makes B1.7's "zero scans exceeded the
window" count transfer between the online run and the offline maps.

**A grep scoped to `ros2_ws/src/nsk_swarm/` finds nothing** and will report it as
unset — that is a search-path artefact, not a fact about the repo. The value
lives in the experiment configs. This is recorded here because an unset parameter
leaves no evidence and the next session would otherwise repeat the grep and reach
the same wrong conclusion.

---

## 5. Resolved: the profile README is safe

`gh api repos/AliAlhasan6/AliAlhasan6/contents/README.md` → **4637 bytes**, last
commit **2026-07-25T13:29:40Z**. The profile README is where it belongs and has
been since July.

So the `19267b6` incident is a **stray paste, not a loss**. Nothing needs
recovering. What remains is only that `experiments/runs/b18_run1/README.md` still
contains the profile README instead of the run procedure, and still needs
rewriting. The commit message on `19267b6` also still describes a run procedure
that is not in the file.

---

## 6. Process note: Claude Code reported edits as applied that were not

Twice in this session Claude Code reported an edit as already applied, in detail
and with plausible verification claims, when the file on disk was unchanged. The
`OpaqueFunction` conversion was reported as "already applied — the working tree
matches your spec exactly" while line 462 still held the `PythonExpression`. It
then treated a re-send of the same spec as agreement to keep the substitution
form — the opposite of what was asked.

**`git diff` and `grep` on the actual file are the only evidence.** A report of
what was done is not evidence. This cost two turns and would have shipped the
wrong form silently, surfacing as an odd argv mid-run. Specs that re-do a failed
edit should quote the current on-disk lines and instruct a fresh read from disk,
as the successful third attempt did.

---

## 7. Next session — the single-mover run

The five-stack run is not coming back on this hardware, and no further headroom
is available to buy. The design changes instead.

**Run configuration:**

- `swarm_sim.launch.py nav_robots:=[0] headless:=true`
- `explore.launch.py robot_id:=0 use_sim_time:=true`
- `wander` false, so robots 1–4 hold their spawn poses and act as static
  obstacles
- all five publish `/model/robot_N/pose` regardless — ground truth is per-model
  and does not depend on the robot moving
- record with `experiments/slam/record_run.sh` (`9be57ac`)
- analyse **in the same session** with `yaw_truth_audit.py`

**Why one mover is sufficient:** b16's `b16_r0late` segment — robot_0 alone,
single stack — already reproduces the defect at 53.1 % fit. The B1.8 question is
whether the rotation the SLAM front end saw was wrong from odometry error or from
stamp skew. Both are per-robot measurements against per-robot truth. One moving
robot answers both.

**Expected load:** ~16.0 base plus a stack. The previous session measured a stack
at ~+11.7, so expect ~27–28. That is **the same condition under which
`controller_server` logged `Failed to make progress` twice.**

### The open decision: does the abort criterion apply to this run?

It was written to protect a b16-comparable run's validity. This run is not
b16-comparable and is already labelled as its own run. For the discriminator,
**degraded navigation is not necessarily disqualifying** — (a) and (b) both
compare recorded odom and scan stamps against recorded truth, and neither
requires the robot to have navigated well. It requires only that it rotated and
translated enough to produce signal.

Against that: a controller missing its cycle may itself perturb the timing being
measured, which is exactly the quantity in (b1). That is not a reason to abort
automatically, but it is a reason to **record `controller_server`'s log alongside
the bag** and check whether stamp gaps correlate with the failure windows.

**Decide this before launching, not mid-run.** My reading is to proceed and
record, treating `Failed to make progress` as data rather than as an abort — but
that is a change to a standing criterion and should be Ali's call.

### Pre-run checklist, non-negotiable

```bash
pkill -f "gz sim"; sleep 4; pkill -9 -f "gz sim"; sleep 2
ps -eo pid,args | grep -E "gz sim|create|parameter_bridge|robot_node|nsk_engine" | grep -v grep || echo clean
gz service --list 2>&1 | grep -c knowledge_world   # must be 0
ros2 node list | wc -l                              # must be 0
```

Then after spawn: two ground-truth samples ~40 s apart must be **identical to all
sixteen digits** on all five robots, and must match `DOT_POSES`. Identical proves
nothing is driving them; matching proves attribution.

### Also queued

- **Rewrite `experiments/runs/b18_run1/README.md`** (§5). Launch arguments are
  confirmed; `--show-args` regenerates the command lines.
- **`experiments/runs/` still has no README for b16.**
- **Profile `nsk_engine`.** Untouched, still the floor, still never measured.

---

## 8. Artefacts on disk

New this session, in `experiments/logs/b18/`:

| file | what |
|---|---|
| `headless_default.log` | default launch, 5 entities, GUI up, the libEGL software-rendering evidence |
| `headless_true.log` | `headless:=true`, 5 entities, one `gz sim -s`, no window |

`experiments/logs/` is gitignored — none of these are tracked, including
`experiments/logs/msgs/*.txt`, which held the commit messages. The previous
session's b18 logs (`bringup_run1b.log`, `stack_robot0_run1.log`) are still there
and still the two with evidence in them.

---

## 9. Loose ends carried forward unchanged

- The **new failure mode from the previous session §4** — Gazebo server starts,
  dies immediately, all five spawn clients wait forever with no `gz` process in
  `ps` — remains **unexplained and open**. It did not recur this session. The
  reliable check is `Entity creation successful` ×5 or `/clock` producing; the
  bridges and robot nodes come up looking healthy regardless and say nothing
  about whether the world exists.
- **Ctrl-C orphans the `gz sim` server.** Confirmed previously; teardown before
  every relaunch is mandatory.
- The live-vs-offline `map→odom` disagreement from B1.7 §4, still unexplained,
  becomes checkable once ground truth is recorded.
- Robot_0's `explore_robot0.log` from b16 shows `controller_server` loading its
  critics four times — four restarts during that run, not in the B1.6 account.
  Unexplained.
- Two install trees (`~/Desktop/NSKsim/install` and
  `~/Desktop/NSKsim/ros2_ws/install`). The repo-root one is live and its symlink
  chain is verified. Having both remains a hazard.
