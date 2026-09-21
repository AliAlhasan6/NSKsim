# HANDOFF — 2026-09-21 — The coverage fix: an LDS-02 and a free-space relay

## 1. What this session was

It started at §7 of `HANDOFF_2026-09-19_known_pose_2x2.md`: get five per-robot
maps. It ends with the machinery finished and none of the five recorded.

Three things happened, in order.

1. `b2maps_e0` was recorded, mapped and fitted — and then superseded, because
   its explorer steered by a map its own odometry had destroyed.
2. `b2maps_e0t` was recorded with the explorer's online poses driven from
   ground truth. The map came out clean and the coverage did not improve.
3. The real cause was found in slam_toolbox's vendored Karto, fixed with two
   changes, and rehearsed. `b2maps_relay1` is the first map in this project
   that covers the whole world.

## 2. The finding

**Karto discards a beam that returns nothing.** `OccupancyGrid::AddScan`
(`Karto.h:6169`) skips any reading `>= maxRange`, `+inf` included, with no ray
trace, so it clears no free space. 57.4 % of `b2maps_e0t`'s 11.7 M beams are
`+inf`.

**A filled beam can never enlarge the map.** The grid's extent comes from
`m_PointReadings` (`Karto.h:5644`), the readings below the range threshold, and
a free-space-only ray is not among them.

So the two changes are complementary and neither works alone:

- **range sets how far the box can reach**
- **the beam policy decides whether open floor inside it becomes known**

Measured on `b2maps_e0t`'s own frozen trajectory: the relay alone at 3.5 m took
free area from 53.56 to 75.73 m² and left the box at exactly 12.00 × 8.00 m.

## 3. What landed

Pushed, HEAD `b449bb2`:

| Commit | Content |
|---|---|
| `92c66d8` | `fit_world_transform.py` judges the analytic candidate on its own score against the best free fit; angle and offset reported, not gated. Seeds the candidate into the peak list before the NMS that had deleted the 180° twin. |
| `cef124b` | The burger carries an LDS-02: `<max>` 3.5 → 8.0, `<min>` 0.12 → 0.16. |
| `2e76fc7` | `max_laser_range` follows the sensor; Nav2's costmap ranges deliberately do not. |
| `ee7c590` | `free_space_relay`: `/robot_K/scan` → `/robot_K/scan_free`, no-return beams filled just above the range threshold. |
| `a8276f1` | `run_offline_maps.sh` reads the lidar ceiling off the bag (`bag_range_max.py`), never from a literal. |
| `9171179` | The COVERAGE gate reads the range the robots actually have. |
| `5a337a4` | `experiments/runs/b2maps/REHEARSAL_free_space_relay.md` |
| `19a68c4`, `a32bc6c`, `b449bb2` | Three CI fixes, all the same shape: a test assumed a package CI does not install. |

Earlier in the session, from the 19th: `record_run.sh` pre-flight retry,
`check_run_bag.py` C4 in sim time, `check_cut_reference.py`, `run_health.py`,
`truth_odom_tf`, `truth_odom_robots` and `scan_matching`.

The R4 edge rule (see §5) is the last commit of the session.

### Sensor and relay numbers

- Sensor: 0.16–8.0 m, per the ROBOTIS e-manual. The LDS-02 replaced the LDS-01
  on the TB3 Burger in 2022, so this is a real sensor, not a convenience.
- Online: `max_laser_range` 7.9, fill 7.95, from `free_space_relay:=true`.
- Offline: derived from the bag. Old 3.5 m bags replay at 3.4 / 3.45.
- Old bags reproduce byte-identically: `b2maps_e0t_inert_robot0.pgm` is
  md5-identical to `b2maps_e0t_kp_robot0.pgm` (`e432e857148fabd7b28bc9d2a5d702f5`).
- Nav2 stays on the raw scan. The fill is a convention only Karto reads.

## 4. Results

| Run | What it shows |
|---|---|
| `b2maps_e0` | Online map fanned into rotated copies; median odometry-vs-truth heading gap 138°. Known-pose cut fits 97.0 %, 5244 free cells, central walls only. **Control.** |
| `b2maps_e0t` | Truth-driven explorer, 111 min, 296 goals, HALTED with 129 frontier cells left. Known-pose cut fits 95.5 %, 5356 free cells — the same coverage. Closest approach to the perimeter 4.178 m. **Control.** |
| `b2maps_relay1` | 9.3 min. Online map 20.00 × 20.00 m, 3139 occupied, no phantom ring, closest approach 0.456 m. The world, correctly mapped. |

In the `e0t` fit, the off-wall cells are the four parked peers — which settles
the open question from the 09-19 handoff, where that was a guess.

## 5. Checks, and what they caught

`check_run_bag.py` C1–C4 and `run_health.py` R2/R4/R5/R7/R9 + COVERAGE ran on
every run. Both `e0t` and `relay1` passed C1–C4, R2, R5 and R7.

- **R4** flagged 3 of 11 271 transforms on no truth stamp in `relay1`, with the
  worst error still 0. It was a recording edge effect: the transforms lead the
  pose series. The rule now splits at the pose series' span — a stamp *between*
  two pose samples is still a hard FAIL, while a short contiguous prefix or
  suffix is admitted within a bounded grace. Do not widen the tolerance
  instead.
- **R9** FAILED on both. `e0t`'s 7 warnings were one event (map→odom arriving
  ~0.55 s old against Nav2's 0.5 s tolerance) at sim 4250–5540 s, after the
  2400 s cut. `relay1`'s single warning came from `collision_monitor`, a
  different node. **Open:** the gate counts all nodes together, so it will keep
  firing across five runs for unrelated reasons. The split-by-frame-pair
  proposal is in `~/.claude/plans/zazzy-forging-sonnet.md`.
- **COVERAGE** is now weak: at 8 m it passes almost anything, and both `e0`
  and `e0t` would pass it. Criterion 5 — occupied cells on the outer boundary —
  carries the weight now.

## 6. Traps found. Do not re-learn these.

- **The live install tree is `~/Desktop/NSKsim/install`, not
  `ros2_ws/install`.** Source `ros2_ws/install` *for the build only*, because
  `nsk_swarm_interfaces` exists nowhere else. Check before every run that the
  installed launch files match the source; a stale tree rehearsed the old
  system once this session.
- **`make_namespaced_burger_sdf` returns a temp file path, not SDF text.** Two
  checks read nothing and reported a false zero before this was noticed.
- **ROS command-line tools miss things under load.** `ros2 topic list` failed a
  recorder pre-flight on 13 of 25 topics that were all live (hence the retry);
  `tf2_monitor` cannot attribute a transform to a publisher at all, so R2 is
  measured from the bag; `map_saver_cli` timed out, so R7 is taken from the
  bag; `ros2 topic hz | tail` loses buffered output when killed. Treat an empty
  result as "retry", never as evidence.
- **Teardown order:** recorder first (so `metadata.yaml` is written), then the
  explorer, then the sim, then `parameter_bridge` by name. Ctrl-C on the launch
  file does not reach the bridge.
- **The recorder's log is the gate.** 26 subscriptions before the explorer
  starts, 28 after. A 0 means the pre-flight aborted; re-run it, and do not
  start the explorer.

## 7. First action next session

**Decide the stopping budget, then record the five runs.**

The budget question is now live because the explorer no longer finishes: with
the 3.5 m sensor it stopped after 71 goals (`e0`) and halted after 296
(`e0t`), and at 8 m it will see more still. 2400 s of sim time per run was
chosen when a partial, differing view per robot was the goal — which is still
the argument for keeping it. Decide it explicitly and record the reason.

Then, per robot K = 0…4, by `experiments/runs/b2maps/README.md`:

```
ros2 launch nsk_swarm swarm_sim.launch.py headless:=true \
    nav_robots:=[K] truth_odom_robots:=[K]
EXPLORER=K RUN=<name> ./experiments/slam/record_run.sh
ros2 launch nsk_swarm explore.launch.py robot_id:=K \
    scan_matching:=false free_space_relay:=true
```

then `check_run_bag.py` → `run_health.py` → strip at C4's printed
`--start/--duration` → `run_offline_maps.sh` with `SCAN_MATCHING=false` and
`FREE_SPACE_RELAY=true` → `fit_world_transform.py --spawn-rev 08617b2`.

Names: pick one and keep it. `b2maps_e0` and `b2maps_e0t` are taken, and both
are controls worth keeping.

Cost: about 2 h of recording plus 25 min offline per robot.

## 8. Carried forward unchanged

- **The designed calibration run** (`HANDOFF_2026-09-14` §6) is still owed for
  any ground-truth-referenced statistic in Article 3.
- **The `parameter_bridge` fix** (launch it as a `Node`, like the clock bridge)
  is deferred until all five runs are recorded, so the teardown stays the same
  across them.
- **The rewriter's `--from {truth,odom}` selector**, which rebuilds Article 3's
  odometry-only arm from these bags, is deferred for the same reason.
- **The B2 schema work** (§3.2–3.5 of `HANDOFF_phaseB2_schema_entry.md`) is
  untouched. Divergence is measured robot-to-robot and never references truth.
- `.vscode/`, `experiments/wall_affinity.py` and `test_wall_affinity.py` are
  still untracked and unrelated. Their 36 tests should not be counted in a
  suite total.
