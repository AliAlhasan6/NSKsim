# SPEC — truth-pose replay (known-pose mapping probe)

Entry point: `HANDOFF_2026-09-18_two_layer_known_pose_mapping.md` §5.
Goal: map robot_0 of `b18_run2` from real scans placed at ground-truth poses,
and decide whether that removes the rotated wall copies.

Status 2026-09-18: Parts 1 and 2 built. Part 3 is a run, not a change, and is
not done here.

---

## 0. Source facts behind this spec

Read from `SteveMacenski/slam_toolbox`, branch `jazzy`: tag `2.8.0` (first
Jazzy release) and HEAD `02afdde` (17 Aug 2026). Identical in both.

- `src/slam_mapper.cpp` declares `use_scan_matching`, default `true`.
- `lib/karto_sdk/src/Mapper.cpp`, `Mapper::Process()`, with it `false`:
  - `MatchScan` is never called, so each scan keeps its odometric pose.
    The "last correction" carried forward is identity from the first scan on.
  - The scan is still added to the sensor manager, so it is in
    `GetAllProcessedScans()`, which is what `CreateFromScans()` builds the
    grid from.
  - The scan is NOT added to the pose graph, and `TryCloseLoop` sits inside
    the same `if`, so loop closure never runs whatever `do_loop_closing` says.
- Scans are still gated by `minimum_travel_distance` / `minimum_travel_heading`.

Consequence: slam_toolbox with `use_scan_matching: false` is a deterministic
known-pose mapper with no matcher and no loop closure. `map → odom` stays
identity. The standalone log-odds mapper of the handoff is only needed if the
checks in Part 3 fail.

**What was corroborated locally (2026-09-18).** The installed build is
**2.8.5**, not the 2.8.0 that was read, so the two differ by five patch
releases. `use_scan_matching` is declared in
`/opt/ros/jazzy/lib/libtoolbox_common.so` and appears `true` in all five stock
configs under `/opt/ros/jazzy/share/slam_toolbox/config/`. Upstream documents
the intended use at `/opt/ros/jazzy/include/karto_sdk/Mapper.h:1781`:

> When set to true, the mapper will use a scan matching algorithm. [...] In
> some simulator environments where the simulated scan and odometry data are
> very accurate, the scan matching algorithm can produce worse results. In
> those cases set to false to improve results.

**What is NOT corroborated locally.** No karto `.cpp` ships with the binary
install — only headers — so the claim that a scan still reaches
`GetAllProcessedScans()` while skipping the pose graph rests on the upstream
read alone. If it is wrong the map comes out empty. Part 3 therefore runs a
short slice before committing to a full replay.

---

## 1. Two corrections found before implementing

Both were found by checking the spec against the bag rather than against the
source, and both would have produced a wrong answer that still looked like a
result.

### 1.1 `--spawn-rev` pins the launch file, not the world

The spec proposed confirming `08617b2` with
`git diff 08617b2 HEAD -- <knowledge_world.sdf>`, expecting no output. That
diff is indeed empty, and it proves nothing: `--spawn-rev` is consumed by
`bag_overlap.resolve_spawn_poses()` (`experiments/analysis/bag_overlap.py:43,71`),
which parses `DOT_POSES` out of
`ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py` **at that revision**. The
world SDF is never read. The launch file did change between `08617b2` and
`HEAD` (66 insertions), and the two candidate revisions disagree outright:

| rev | robot_0 spawn |
|---|---|
| `fc050c4` — the **default**, `BAG_ERA_REV` | `(3.00, 0.00)` |
| `08617b2` / `HEAD` | `(0.86, 0.28)` |

**The spec's choice of `08617b2` is right; its justification is not.** The
decisive evidence is in the bag. Robots 1–4 are parked — 0.00 m of motion
across all 2329 s — so each one's first truth pose *is* its spawn pose:

| robot | first truth | vs `fc050c4` | vs `08617b2` |
|---|---|---|---|
| 1 | `(−0.002, +0.900)` | 3139.1 mm | **1.6 mm** |
| 2 | `(−0.862, +0.280)` | 3156.4 mm | **1.6 mm** |
| 3 | `(−0.532, −0.730)` | 3147.9 mm | **1.6 mm** |
| 4 | `(+0.528, −0.730)` | 3142.6 mm | **1.6 mm** |

This is now check **C6**, which runs on every invocation. The `git diff`
against the world SDF is retired; it answers a question nobody asked.

**What a wrong revision actually breaks (checked 2026-09-18).** Not the score.
`fit_world_transform.py` neither seeds nor bounds its search at spawn: `spawn`
reaches exactly one function, `analytic_candidates()` (line 485), and never
touches `coarse_search`, `refine` or `free_fit`. The coarse stage keeps the
**full** FFT correlation — its docstring records that a ±5 m box anchored at
the YAML origin would exclude robot_1's and robot_3's own candidates, robot_3
by 11.6 m — and `thetas` is `arange(-180, 180)`, the whole circle. So the
free-fit score, the 80 % floor and the gating/diagnostic split are all
independent of `--spawn-rev`.

What moves is the analytic candidates. The two eras are 3.15 m apart against
an `AGREE_XY_M` of 0.25 m — 12.6× the tolerance — so every candidate misses
its own free fit and the run prints **UNRESOLVED**, whose text invites you to
go looking for "a further frame in the chain". A false `RESOLVED` is not
reachable at that displacement; the cost is misdirection, not a wrong number.
That is still worth a gate, because the misdirection is expensive and silent.

**Both eras are live in this repo**, which is why the default cannot simply be
repointed: `phaseB_run1` (2026-07-22) predates `e864ae7`'s pentagon and needs
`fc050c4`, while b16 and b18 need `08617b2`. Corroboration is the only fix
that distinguishes them.

### 1.2 The anchor is the spawn pose, not the first sample

The spec defined `A` as the first `/model/robot_N/pose` sample. On `b18_run2`
that is `(3.664, −0.981, −60.90°)` — **3.075 m and −60.9° away from spawn**,
because recording began 603 s of sim time after the sim did (the first
`odom → base_footprint` header stamp is 603.330).

Anchoring there has two consequences, neither acceptable:

- `fit_world_transform.py` builds `world_T_odom` as a **pure translation with
  spawn yaw 0**. An odom frame parked at an arbitrary mid-run pose violates
  that, so its analytic candidates no longer describe the map.
- **C4 would pass vacuously.** Its whole purpose is to prove the rewrite
  changed something; with a −60.9° constant offset baked into the anchor,
  `max |Δyaw| > 1°` is satisfied by the offset alone and the check stops
  discriminating.

**`A` is therefore `(DOT_POSES[N].x, DOT_POSES[N].y, 0)` at `--spawn-rev`.**
The rewritten `odom` frame then *is* the spawn frame, which is exactly what
the fitter assumes, and `truth_anchor.txt` and the fitter's analytic candidate
become two independently derived values that must agree. Spawn yaw is 0 and
dropped, on the same grounds the fitter drops it: the launch file passes only
`-x -y -z`, and every `DOT_POSES` yaw is `0.0` at this revision.

**A result worth keeping.** Composing the first truth pose with the inverse of
the first recorded `odom → base_footprint` puts the odom origin at
`(0.567, −1.897, 121.47°)` against a true spawn of `(0.86, 0.28, 0°)` —
robot_0 carried **2.197 m and +121.47° of odometry error before the recording
even started**. That is the defect under study, measured, and it is why arm A
is expected to fail.

---

## 2. Part 1 — `experiments/slam/rewrite_odom_from_truth.py`

Sits beside `strip_bag_for_offline_slam.py` and mirrors its bag I/O exactly:
`rosbag2_py` reader/writer, `storage_id="mcap"`, `ConverterOptions("", "")`,
reader `TopicMetadata` passed straight to `writer.create_topic` (which
preserves `/tf`'s two offered QoS profiles), `TFMessage` deserialised only on
the carrier topic, raw bytes everywhere else, `del writer` to close.
`yaw_of(q)` is the form used in `experiments/analysis/tf_pair_at.py`.

**Arguments:** `--in`, `--out`, `--robot N` (default 0), `--spawn-rev`
(default `BAG_ERA_REV`; b18-era bags need `08617b2`), `--dry-run`,
`--allow-unverified-spawn`, `--progress`. An existing `--out` is refused
(exit 2), as in the sibling.

**Discovery, before any write. Frame names are not hard-coded.**
The pair is selected by convention — child ends `base_footprint`, parent ends
`odom`, both naming robot N — over every `TFMessage` topic except
`/tf_static`. Zero matches or more than one distinct pair aborts nonzero. All
pairs found are printed, with the candidates marked. The
`header.frame_id` of `/model/robot_N/pose` is printed.

**Rewrite.** Every transform with that pair is removed from the carrier topic;
a message is dropped only once it is empty. For each pose `P` a one-transform
`TFMessage` is emitted with `odom_T_base = inv(A) ∘ P`, `header.stamp` the
pose's header stamp, z = 0, roll = pitch = 0, written at the pose's bag
receive time. `/tf_static`, scans, `/clock` and the pose topic itself pass
through untouched. Reads and writes both follow bag order, so the output stays
timestamp-ordered. The anchor is written to `<out>/truth_anchor.txt`.

**Checks.** Each asserts a value and can fail.

- **C1, composition (synthetic, before any bag I/O).** `A = (2.0, −1.0, 30°)`,
  `P = (3.0, 1.0, 75°)`, expected `inv(A) ∘ P = (1.866025, 1.232051, 45°)`
  hard-coded from the hand calculation, asserted to 1e-6, plus
  `A ∘ result == P`. The reversed composition gives
  `(−2.190671, 0.448288, −45°)`, so an inverted operand order fails here
  rather than shifting every map. Measured error 4.04e-7.
- **C2, counts.** Removed == census for the pair; synthesised == pose-message
  count; output total == input − emptied + synthesised. All four printed.
  Measured: removed 96,522, synthesised 38,604, emptied 96,522,
  in 1,519,736, out **1,461,818**.
- **C3, round trip on the bag.** Every 100th synthesised transform is decoded
  back out of the quaternion actually written and composed with `A`; it must
  reproduce the pose's SE(2) to 1e-6 m and 1e-6 rad. Going through the built
  message, not the intermediate tuple, is what makes this test the encoding.
  Measured: 387 sampled transforms, worst error 4.44e-16.
- **C4, the rewrite changed something.** Original odometry yaw is interpolated
  (shortest arc) at each synthesised stamp; median and max wrapped `|Δyaw|`
  are reported and max must exceed 1°, else exit 2. Under the spawn anchor
  both series share an origin, so this is the odom yaw error itself. Measured
  on `b18_run2` robot_0 over all 38,604 stamps, none outside the odometry
  range: **median 103.557°, max 179.989°.** Robot_0's recorded heading is
  effectively decorrelated from truth.
- **C5, no leftovers.** The output bag is re-read; transforms with the pair
  must equal the pose count exactly.
- **C6, spawn-rev corroboration.** Every robot that never moved (< 1 cm total
  displacement) must sit within 5 cm and 0.5° of its `DOT_POSES` entry.
  Abort if any mismatches, or if none is parked — `--allow-unverified-spawn`
  overrides the latter only. This is §1.1 turned into a gate.

Wall time is one discovery pass, one rewrite pass and one verification pass:
~34 s, ~35 s and ~25 s on `b18_run2`, well under the 5–10 min estimated.

**Run 2026-09-18, all checks passed.** The rewritten bag is
`experiments/logs/b18/b18r2_truth` (732 MiB, 1,461,818 messages, same start,
end and duration as the source; `/tf` 541,077 = 598,995 − 96,522 + 38,604,
every other topic count unchanged). Its `truth_anchor.txt` reads
`x_m 0.860000`, `y_m 0.280000`, `yaw_deg 0.000000` — the value the fitter must
recover in Part 3. Stripped for replay as
`experiments/logs/b18/b18r2_truth_slamin`.

---

## 3. Part 2 — `SCAN_MATCHING` in `run_offline_maps.sh`

`SCAN_MATCHING` (default `true`) substitutes `__SCAN_MATCHING__` in
`offline_mapping.yaml.template`, which previously held a literal
`use_scan_matching: true` at line 87. A second `sed -e` does the substitution
and a second `grep -q … || die` proves it landed, following the existing
`__ROBOT__` precedent. Values other than exactly `true` or `false` are
rejected before anything launches.

Nothing was added to the template but the placeholder itself: a comment there
would propagate into every generated params file and break the byte-for-byte
guarantee below, so the rationale lives in `run_offline_maps.sh` instead.

**Verified both ways** — an empty diff alone would prove nothing:

- With `SCAN_MATCHING` unset the generated YAML is byte-identical to the
  pre-change output, md5 `eda84f4fc222910026a1f8bcf90a9992`, which is also the
  md5 of `experiments/logs/offline_mapping_b16_slamin_robot_0.yaml` — the
  params file from the real b16 run.
- `SCAN_MATCHING=false` changes exactly one line, `87c87`.

`do_loop_closing` is untouched; it has no effect when matching is off.

**`strip_bag_for_offline_slam.py` needs no change.** Its keep set is
`{/clock, /tf, /tf_static}` ∪ `{robot_N/scan, robot_N/odom}`; `b18_run2`
carries all of them, so the exit-3 missing-topic check passes, and the extra
b18 topics are dropped silently by the reader filter. Use `--robots 1` for
robot_0 alone. The 25-topic figure in the spec was b16's recorded set;
`b18_run2` has 22.

Confirmed on the rewritten bag: 840,383 messages in 26 s, keeping `/clock`
(193,131), `/robot_0/odom` (96,518), `/robot_0/scan` (9,652), `/tf_static` (5)
and `/tf` (541,077 → 424,694). It dropped the 116,383 live
`robot_0/map → robot_0/odom` and kept `robot_0/odom → robot_0/base_footprint`
at exactly **38,604** — the synthesised truth transforms and nothing else.

**Order matters.** `strip` drops `/model/robot_N/pose`, so the rewrite must
run **first**. Arms B and C are
`rewrite → strip → run_offline_maps.sh → fit_world_transform.py`.

---

## 3b. The shared spawn-rev gate

The parked-robot comparison lives in `bag_overlap.py` as two functions, so the
rewriter and the fitter cannot drift apart:

- `read_first_truth_poses(bag)` → first pose and furthest excursion per robot.
  ROS imports are function-scoped, the pattern `read_bag()` already uses, so
  importing `bag_overlap` from a bare venv still costs nothing.
- `check_spawn_against_truth(first_pose, max_disp, spawn, rev)` → pure, no
  I/O. Returns `(ok, lines, n_parked)`; `ok` is False only on a genuine
  mismatch, leaving the policy on an uncorroborated bag to the caller.

`rewrite_odom_from_truth.py` calls it as C6 and refuses to run uncorroborated
unless given `--allow-unverified-spawn`. `fit_world_transform.py` calls it
behind a new optional `--bag` and aborts on mismatch before any fitting; the
result is recorded in the output JSON as `spawn_rev_corroborated`.

`--spawn-rev` was deliberately **not** made required. The default is correct
for the phaseB corpus, requiring it would turn a detectable error into a
mandatory guess, and a required flag still cannot tell a right guess from a
wrong one. Corroboration can.

**Limitation.** The gate needs ground truth, which arrived with `662abe2`
(2026-09-10). On `phaseB_run1` and b16 bags `--bag` dies with a message saying
so rather than pretending to verify. Those runs still rest on the recorded
reasoning in `bag_overlap.py:45-54`.

Without `--bag` the fitter prints `spawn table UNVERIFIED` and proceeds, so
every existing invocation behaves exactly as before — confirmed against the
stored `b16_slamin` robot_0 reference: convention A 33.2 % at θ +88.70,
dx −4.450, dy +0.442, and convention B 34.1 % at θ +91.20, unchanged.

---

## 4. Part 3 — three arms on `b18_run2`, robot_0 only (Ali runs)

Pass `--spawn-rev 08617b2` to both `rewrite_odom_from_truth.py` and
`fit_world_transform.py`, and `--bag` to the fitter so the table is
corroborated rather than assumed. Do not use the `fc050c4` default (§1.1).

Per arm, check the slam_toolbox log's drop count ("queue is full" /
"earlier than all the data in the transform cache"). Allowed: one benign
first-scan drop. Truth TF arrives at 16.57 Hz against odometry's ≈41 Hz, so
this matters more than usual — though a 60 ms period still sits well inside
`transform_timeout: 0.2`.

| Arm | Bag | Matching | Prediction |
|---|---|---|---|
| A control | original | on | Fails the 80 % floor, with rotated copies |
| B locked layer | truth | off | Clears 80 %; fitted world pose equals `truth_anchor.txt` within one map cell and 0.5° |
| C matcher on truth | truth | on | Optional. Shows whether the matcher disturbs correct poses |

Run a ~120 s slice of arm B first (`DUR_0`) and confirm a non-empty PGM before
committing to a full replay — that is the cheap test of the §0 claim that
cannot be checked locally. During arm B, `ros2 param get /slam_toolbox
use_scan_matching` must report `False`.

Approximate wall time: 20 min per replay at rate 2.0 (2329 s bag), ~1 min per
fit.

**Reading the result.**
- A fails, B passes with the anchor recovered → the two-layer design has its
  evidence; slam_toolbox with matching off is the locked layer.
- A fails, B fails → drift was not the whole story (handoff §5).
- B clears 80 % but misses the anchor → poses were not used as given.
  Investigate before trusting any score.
- A passes → `b18_run2` does not show the defect and cannot test the fix.
  Read the peak profile, not only the score. This branch is now unlikely:
  robot_0 carries 121° of odometry error at bag t=0 (§1.2).

---

## 5. Bag facts (`experiments/logs/b18/b18_run2`)

mcap, 742 MB, 22 topics, **1,519,736** messages, 2329.316 s.
`/model/robot_0/pose`: **38,604** messages at 16.57 Hz,
`header.frame_id` `'knowledge_world'`. `/tf`: **598,995** messages carrying
exactly one transform each —

| pair | count |
|---|---|
| `robot_0/map → robot_0/odom` | 116,383 |
| `robot_0/odom → robot_0/base_footprint` | **96,522** |
| `robot_{1,4}/odom → …/base_footprint` | 96,522 each |
| `robot_{2,3}/odom → …/base_footprint` | 96,523 each |

The target pair is unique, so discovery cannot be ambiguous. Because every
`/tf` message carries one transform, each removal empties its message, so the
expected C2 counts are: removed 96,522, synthesised 38,604, emptied 96,522,
output 1,461,818.

Robot_0's net first→last displacement is **6.43 m** and its furthest
excursion from the first recorded pose is **7.77 m**, against the 4.7 m from
spawn quoted in the handoff. The map should not be as small as feared.

Discovery reads the TF and pose topics only — 792,017 messages in ~34 s.

---

## 6. Known defects and carried forward

- `experiments/runs/b18_run1/README.md` was committed in `19267b6` as the run
  procedure, but its contents are the author's GitHub profile README. The
  b18_run1 procedure was never committed.
- Two comments in `offline_mapping.yaml.template` are stale for b18: it states
  the bag has no `/clock` (`b18_run2` has 193,131) and cites "~4340 msgs each"
  for `odom → base_footprint` (96,522 here). Cosmetic; left alone so the
  byte-for-byte guarantee of §3 stays checkable against the b16-era files.
- `SPEC_b2_steady_gate.md` is still owed to `docs/specs/` (handoff §7). This
  document is the first file in that directory.
