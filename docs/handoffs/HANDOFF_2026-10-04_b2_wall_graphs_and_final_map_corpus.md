# HANDOFF — B2 wall graphs and the final map corpus (29 Sep – 4 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-09-29_gates_and_grid_divergence.md`.

---

## 0. In one paragraph

The wall predicate is built and validated (Parts A and B of
`docs/specs/SPEC_b2_wall_predicate.md`). Checking it against the maps exposed three
defects in how the B2 maps were made and read: a parked robot outvoting the rest of
its map, every ROS reader placing slam_toolbox cells half a cell off, and Karto's
grid stopping short of its own data so that the east and north walls were lost. All
three are fixed. The final B2 corpus is 20 maps,
`b2maps_k{K}_cut{C}_gated_extfix_robot{K}`, built by slam_toolbox 2.8.5 with a
one-line extent patch from travel-gated bags, each with its wall graph and viewing
directions. On that corpus every pre-registered wall check holds, including the
original B1. Ali chose to build the divergence measure on walls next, using the
"same world" shape accepted this session (§5).

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only .vscode/, experiments/wall_affinity.py, test_wall_affinity.py
git log --oneline -2 origin/main         # expect this handoff's commit, then cdba182
ls experiments/maps/b2maps_k*_cut*_gated_extfix_robot*.pgm | wc -l            # expect 20
ls experiments/logs/graph_walls/b2maps_k*_cut*_gated_extfix_robot*.json | wc -l  # expect 20
```

Note on `cdba182`: it was made at 2026-10-04 05:48:33 +0300, before Ali's commit
block ran, and holds exactly the five intended files. Unless Ali committed it at that
time, Claude Code did, which breaks the rule that Ali makes every commit. Remind
Claude Code of the rule at the start of the next session.

---

## 2. Commits this session (all pushed)

| Commit | What |
|---|---|
| `06aac42` | graph_walls: `_gated` stems. **Also first added `graph_walls.py` (Parts A and B) and `test_graph_walls.py`**; its message understates this |
| `e96b7f0` | offline replay: travel gate, default off |
| `83a9a6c` | gated maps: cell-level prediction and comparison |
| `0a574a9` | spec line. **Also first added the whole spec**; message understates this |
| `b3f8cb5` | B1 diagnosis instruments (dwell, Karto rebuild, phase, edge), with the history correction for the two commits above |
| `f14747f` | one cell-centre convention for slam_toolbox maps, and B1 in-grid (19 files) |
| `0e44006` | extent fix: the Karto patch file and its prediction |
| `9cf0ae2` | overlay stage A: the `SLAM_TOOLBOX_OVERLAY` hook, stem tokens, the R1–R3 tool |
| `9035dd4` | overlay stages C and D recorded, and the `--extfix` driver |
| `cdba182` | stage E: the 20 patched maps, E3, Part B on a variant corpus (see §1) |

Untracked and unrelated: `.vscode/`, `experiments/wall_affinity.py` and its test (the
August locomotion tool). Decide later whether to commit or delete.

---

## 3. The final B2 corpus

**Maps:** `experiments/maps/b2maps_k{K}_cut{C}_gated_extfix_robot{K}.pgm/.yaml`,
K = 0–4, C = 60, 120, 240, 1200 s of sim after the first command.

**How they were built:**
- Known-pose offline mapping, `DETERMINISTIC=true`, free-space relay, from the bags
  `experiments/logs/b2maps/b2maps_k{K}_cut{C}_gated_slamin`.
- Travel gate: a scan is admitted only if the sensor moved at least 0.01 m or turned
  at least 0.5° since the last admitted scan. Short cuts drop 1.7–3.3 % of scans;
  1200 s cuts drop 47.5–61.6 % (the parked tail after exploration ends).
- Binary: slam_toolbox 2.8.5 with `docs/patches/karto_extent.patch`
  (Karto.h:6111-6112, width and height `Round(size/res) + 1`), overlay
  `~/src/slam_toolbox_ws_extfix/install`.
- Batch `experiments/logs/b2maps/batch_20261003T224251Z.log`, 162 min 50 s, HEAD
  `9035dd4`. Every row: at most one drop, at the first scan; no leftovers; fit A
  98.0–99.7 %, all RESOLVED convention A.

**Checks on the corpus:**

| Check | Result |
|---|---|
| R1: unpatched overlay reproduces the installed binary | 3/3 byte-identical (k0, k1, k2 cut60); `/proc/<pid>/maps` showed the overlay's own libraries loaded |
| R2: patched map = unpatched gated map on the overlap, +1 per axis, origin unchanged | 20/20 |
| R3: added row and column agree with the extent prediction (≥ 99.5 %) | 19/20; k2_cut60 fails by one cell of 105 (map occupied, prediction free). Recorded, not chased |
| E3: max-side (east, north) recall within 2 pp of min-side, cut1200 | 5/5, worst 1.06 pp (unpatched gaps were up to 21.4 pp) |
| Off-grid kept returns | 0 on 20/20 |
| Each cut1200 map vs its gated twin | +401 occupied cells, free and unknown unchanged: the boundary walls |

**Wall graphs:** `experiments/logs/graph_walls/b2maps_k*_cut*_gated_extfix_robot*.json`.

| Check | Result |
|---|---|
| F1: B1 counting all kept returns ≥ 90 % | 20/20; B1_offgrid 0 everywhere; in-grid worst 99.98 % |
| F2: B2 side contradictions ≤ 1 % | 20/20, worst 0.249 % |
| F3: A8 ≥ 95 %, A10 | 20/20 and 20/20; A9 rises to 99.18–99.34 % at cut1200 |
| B3 | 19/19, plus k4_cut1200 NOT JUDGED: its relay was SIGKILLed at teardown before printing its summary. The map itself is unaffected (R2 held) |

**Superseded map sets, kept but not for B2:** the ungated deterministic maps
(`b2maps_k*_cut*_robot*`, 27 Sep), the gated maps (`*_gated_robot*`, 1 Oct), and the
overlay controls (`*_gated_ovl_robot*`, k0, k1, k2 cut60).

**Binaries and source:**

| Path | libtoolbox_common / libkartoSlamToolbox sha256 (16) |
|---|---|
| `/opt/ros/jazzy` (deb) | `991de233df7137bc` / `343a02caafcd3942` |
| `~/src/slam_toolbox_ws/install` (unpatched overlay) | `5718fbe8c08b0e73` / `8ecb141cbf39f47c` |
| `~/src/slam_toolbox_ws_extfix/install` (patched) | `f7171cb029990f75` / `b2261dc254de0f58` |
| `~/src/slam_toolbox_2.8.5/` (pristine source, citation authority) | orig tarball `80af3955…608583` |

---

## 4. What was learned

### 4.1 The wall predicate (spec v2)
- A wall node is one observed **face** (decision D1), not a physical slab.
- A face element is an oriented boundary between an occupied cell and a free cell,
  with a normal into free space. A one-cell wall free on both sides gives two
  elements, one per side.
- Resolution is 0.1 m (not 0.05). Thresholds: `eps = rho(1 + √2/2) = 0.1707 m`;
  band `b = rho·√2/2`; `g = 2·robot_radius = 0.22 m` (read from nav2 params);
  `l_min = 0.5 m`; bearing histograms of 72 × 5°. Observation count is `n_scans`.
- A8 cannot discriminate the half-cell convention, because the world's wall surfaces
  sit at 0.0 and 0.05 mod rho. That pre-registration was a design flaw, recorded.

### 4.2 Karto and slam_toolbox, from the 2.8.5 source
- A scan from an identical pose **is** integrated: `HasMovedEnough` returns true on
  `minimum_time_interval = 0` (Mapper.cpp:3147-3178).
- slam_toolbox discards scans 2, 3 and 4 of every run (`scan_ctr < 5`,
  slam_toolbox_common.cpp:795-797).
- Defaults when the config sets neither: `min_pass_through 2`,
  `occupancy_threshold 0.1` (slam_mapper.cpp:360-372).
- One valid ray leaves its hit cell with 2 passes and 1 hit (Karto.h:6202-6236).
- The published map origin is Karto's offset verbatim (visualization_utils.hpp:108-129),
  and Karto centres cell i at `origin + i·res` (Karto.h:4421-4436). Every ROS reading
  (`origin + (i + 0.5)·res`) was 0.05 m off in +x and +y. Fixed once, in
  `trinary_map.CELL_CENTRE_OFFSET = 0.0`, with every reader routed through it.
- The grid width is `Round(size/res)` with the offset at the bounding-box minimum, so
  the grid stops 0–1 cell short of its own far extent, and hits there are dropped.
  That removed part of the east and north boundary walls from every map. Fixed by the
  patch.

### 4.3 The B1 diagnosis chain
On the 27 September maps, B1 failed on 4/20 and B2 on 1/20. Four diagnoses ran, each
with predictions printed first:
1. **dwell:** the cells were cleared, not unseen (P3 held 20/20).
2. **exact Karto rebuild** (`diagnose_karto.py`, reproduces the maps 18/20 exactly,
   2/20 within 5 cells): dwell passes recommit 84 % and 69 % of the off-wall cells on
   k1 and k3 cut1200; relay fill and grazing were ruled out.
3. **grid phase:** ruled out (G1, G2 failed). Grid phases converge at cut1200,
   because the bounding boxes coincide once coverage is complete.
4. **range-edge clipping:** ruled out. The stopping rule then closed the residual
   (k0_cut240, k4_cut120) as unexplained.

The residual was explained afterwards, by accident: correcting the cell-centre
convention showed 92–97 % of the off-wall returns falling **off the grid**, which led
to the extent defect. The travel gate fixed the dwell maps; the patch fixed the rest.
On the final corpus the original B1 holds 20/20.

Also recorded:
- B3 compared scans received by the relay, not scans reaching the grid (6001 read,
  6000 pose-valid, 5997 to the grid).
- The spin-smear idea in spec §12 stays UNTESTED.

### 4.4 Operational traps found this session
- `run_offline_maps.sh` runs with `set -u`. colcon's `setup.bash` reads unset
  variables, so the hook clears nounset only around the source line.
- The runner's preflight refuses any process whose argv contains `slam_toolbox`.
  That includes `timeout`, `nohup` and `bash -c` wrappers, and any argv holding an
  overlay path. Never wrap the runner or the driver. Pass the overlay as an exported
  variable, and `unset SLAM_TOOLBOX_OVERLAY TRAVEL_GATE` afterwards.
- Calling `run_offline_maps.sh` directly needs the driver's settings:
  `SCAN_MATCHING=false FREE_SPACE_RELAY=true DETERMINISTIC=true`. A params-file diff
  (R0) catches any omission before a replay is trusted.
- An overlay build must match the deb's flags: in the workspace directory, export
  `CXXFLAGS="$(dpkg-buildflags --get CXXFLAGS) -DNDEBUG"`,
  `CPPFLAGS="$(dpkg-buildflags --get CPPFLAGS)"` and `LDFLAGS=`, with only
  `/opt/ros/jazzy` sourced; `--cmake-args -DBUILD_TESTING=OFF`. The package forces
  Release. A build takes 6–8 min.
- The library named in a log line is the file on disk, not proof of what was loaded.
  Read `/proc/<pid>/maps` during a replay.
- Several tools overwrite a single result file (`comparison.json`,
  `diagnose_extent.json`). Copy the file before re-running.
- Claude Code's command blocks arrive truncated at terminal width. Chat re-issues
  them cleaned, and Ali runs those.
- Edits are all committed before a batch starts, and nothing is edited during one.

### 4.5 Not fixed, deliberately
`frontier_explorer.py` reads `/map` with the ROS half-cell convention, and the online
slam_toolbox (the deb) has the same extent truncation. Both affect exploration
behaviour, not B2 measurement. Decide before B4 whether online runs should use the
patched overlay.

---

## 5. Next: the divergence measure on walls

**Accepted shape (Ali, this session).** Place every robot's wall faces in one frame
using the spawn registration, which the robots are given by design (DOT_POSES at
`08617b2`, translation only, yaw 0); no truth enters the measure. For an ordered pair
(A, B), sort each face of A by B's evidence:

- **corroborated:** B has a matching face;
- **unobserved:** B never observed that place;
- **contradicted:** B observed open space there.

A contradiction must rest on **B's own scans having crossed the place without a
return**, not merely on B's map saying FREE. Same world means nothing is
contradicted. Divergence is the mix of the three. Convergence is the unobserved share
shrinking while contradictions stay at zero.

**Inputs available:** the 20 graph JSONs (segments, normals, endpoint states, gap
states, bearing and incidence histograms, `n_scans`), the 20 maps, and the gated
bags. `diagnose_karto.py` already has an exact, vectorised Karto ray tracer that a
"B's scans crossed this place" test can reuse.

**Design questions for the spec (none decided yet):**
1. Face matching between robots: normal tolerance, line distance (`eps`?), and
   overlap. Faces may split differently in two maps, so match by length along the
   line, not by segment identity.
2. Weighting: by face length; how viewpoint diversity from `bearing_hist` enters
   (§3.3 of the schema handoff); how to handle faces with empty histograms (4 of 220
   on the old corpus).
3. "Unobserved" versus "contradicted" when B's evidence is mixed along one face.
4. Per pair, per cut: the three shares, a symmetric summary, and the series over cuts.
5. The five runs differ in one physical respect: in run K, robot K has left its spawn
   while the other four stay parked. Parked robots are unclassified, not faces, but
   they can occlude walls. State how the measure treats that.
6. Pre-registrations, to be written before any number. Candidates: contradicted share
   near zero at cut1200 for every pair; unobserved share non-increasing with cut.
   Truth is used only to validate that corroborated faces are real walls.

---

## 6. Open items carried forward

- Remind Claude Code that Ali makes every commit (§1, `cdba182`).
- R3 on k2_cut60 (one cell) and k2's earlier C2 discrepancies (3, 1 and 1 cells):
  recorded, not chased.
- B3 not judged on k4_cut1200_gated_extfix: the relay outlived its teardown grace
  and was SIGKILLed. Check the stopper's grace before the next batch.
- The spin-smear line in spec §12: UNTESTED.
- Online explorer and online mapper defects (§4.5): decide before B4.
- The stage F pre-registration text sits in a gitignored file
  (`experiments/logs/graph_walls/stage_f_preregistration.txt`); its content is in
  spec §12.9.
- `HANDOFF_phaseB2_schema_entry.md` and `HANDOFF_2026-09-29_gates_and_grid_divergence.md`
  should be added to `docs/handoffs/` (spec v2, change C1), if not already done.
- Check the GitHub Actions results for the commits since `0dee108`. CI had not seen
  graph_walls or its tests before this session.
- `wall_affinity.py` and its test: commit or delete.

---

## 7. Working protocol (unchanged, with this session's additions)

- Replies are 120–130 words, code excluded, in plain English.
- Claude designs and reviews. Claude Code edits, runs analyses when asked, and stops
  for review at each stop point. Ali runs the simulation, batches and builds, and
  makes every commit. Commit messages go through files in `experiments/logs/msgs/`
  and `git commit -F`.
- Every command block states terminal, sim status, blocking status and wall time, and
  sources `/opt/ros/jazzy/setup.bash` and `~/Desktop/NSKsim/install/setup.bash`. The
  one exception is an overlay build, which sources only `/opt/ros/jazzy`.
- Any prompt for Claude Code is introduced with a "(for Claude Code)" heading.
- Predictions are pre-registered and printed before the numbers, are never reworded
  afterwards, and stay printed when they fail. Every check is shown failing under a
  named break. A threshold set after seeing data is labelled CALIBRATED or post hoc.
- A diagnosis chain gets a stopping rule before its last candidate is tested.
- Typical wall times: Part A of graph_walls ~20 s for 20 maps; Part B ~90 s; Karto
  rebuild ~3 min; a 20-map deterministic batch 2.6–2.8 h.

---

## 8. The operating lesson

A check that reads a map through a reader inherits the reader's geometry. Four
diagnoses chased cleared walls through a reader that was half a cell off, against a
grid that stopped short of its own data. The cause surfaced only when the reader was
corrected for a different reason. Before diagnosing what a tool produced, confirm the
reader against the producer's source. And an exact reimplementation of the producer
pays for itself: it turned slam_toolbox from a black box into arithmetic, where every
counterfactual is a subtraction and every fix can be predicted cell by cell before it
is built.
