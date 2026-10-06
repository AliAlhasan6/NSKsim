# HANDOFF — B2 viewpoint weight on walls (6 – 7 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-06_b2_divergence_on_walls.md`.

---

## 0. In one paragraph

O1 is closed. Each wall element of A that B corroborates now carries a viewpoint
weight: how far apart A's and B's viewing directions on that face are, from 0 (the
same directions) to 1 (half a turn apart). The weight was specified, gated,
registered, tested and recorded, each in its own commit, and no weighted number was
seen before its predictions were registered. All three hold: a map against itself
scores exactly 0 (G3, 20/20), the same wall scores lower than a different wall facing
the same way (P7, 19/20), and B's score against A's final map falls as B explores
(P8, 20/20). The finding is the level: at full coverage, B's views of a wall sit on
average 5–11° from A's, so corroboration rests on closely aligned viewpoints and is
largely correlated evidence.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only .vscode/
git log --oneline -1 origin/main         # expect this handoff's commit, or 10ae987
grep -m1 -o "v0\.10" docs/specs/SPEC_b2_divergence_walls.md     # expect v0.10
ls experiments/logs/divergence/s5_viewpoint_20261006T212758Z.json  # expect the file
```

Note: the previous handoff's §1 sourced `~/Desktop/NSKsim/install/setup.bash`. The
workspace builds into `ros2_ws/install/`; the block above is corrected.

---

## 2. Commits this session (all pushed)

| Commit | What |
|---|---|
| `8ca8166` | spec v0.8: section "Viewpoint weight (O1)"; `circular_emd_deg`, `corroborate_matches`, `viewpoint_weights`; 14 tests |
| `25ba70e` | G2 stage: surveys the segment guard on the corpus without reading a histogram. Also changes the guard's endpoint test from per-axis to Euclidean (same 1e-6 m tolerance), which alters behaviour committed in `8ca8166`; the message says so |
| `d5b03b8` | spec v0.9: registers G3, P7 and P8, including the definition of a segment's direction. Spec alone, committed before the code that tests it |
| `2161665` | S5 stage: tests G3, P7 and P8; refuses on an edited tree |
| `10ae987` | spec v0.10: records the S5 results, two cautions, O1 closed, O6 opened |

Tests: 60 divergence; full listed suite 733 passed, 33 skipped (project venv).

---

## 3. The weight as it stands (spec v0.10)

- **What is stored.** Each wall segment (one face) keeps `bearing_hist`: 72 bins of
  5°, counting the returns that hit it. Bearings are sensor-to-hit directions in each
  robot's own map frame (`graph_walls.py:1544`). A return counts only if it came from
  the free side, so one face's histogram covers at most half the circle.
- **Opposite sides are not in O1.** A robot behind a wall hits the back face, a
  separate segment. Pairing front and back faces is the schema's back-to-back edge,
  part of the rest of the schema, not the divergence measure.
- **The weight.** For a corroborated element `e` of A on segment `S_A`, matched by T1
  to a B element on `S_B`: `w(e) = W1(p, q) / 180°`, where `p`, `q` are the two
  histograms normalised and `W1` is the earth-mover distance on the circle.
- **Which B element.** T1 records only that a match exists, so `corroborate_matches`
  picks one: nearest across, then nearest along, then lowest B segment id.
- **Histograms come from the saved wall-graph JSON**, joined to the recomputed
  segments by index. `segment_guard` checks the join (same count, every endpoint
  within 1e-6 m, Euclidean); `load_side` refuses on a bad verdict.
- **Unweighable** elements (an empty histogram, or no B segment) get no weight and
  are reported as `unweighable_A` / `unweighable_B`.
- **Reported per row:** `W` (weighted share, a lower bound, never above `C`),
  `w_mean`, `w_p10`, `w_p50`, `w_p90`, both unweighable shares.
- **Known limit, accepted:** the histogram counts returns, not scans, so a close pass
  or a long stop weighs more than a distant one. Per-scan counts are not stored.
- `W` sits beside `C` and never replaces it. P3, P4 and P6 stay on `C`.

---

## 4. Results

### 4.1 G2, the join guard (`g2_guard_20261006T211250Z.json`, at `25ba70e`)

20/20 pass; every largest endpoint difference exactly 0.0. `graph_walls.py` last
changed in `cdba182` (4 Oct 2026, 05:48 +0300); the zeros show the saved files match
it.

### 4.2 S5, registered at v0.9 (`s5_viewpoint_20261006T212758Z.json`, at `2161665`)

| Prediction | Result |
|---|---|
| G3: a map against itself gives C = 1 and w = 0 exactly | **HOLD**, 20/20 |
| P7: at cut1200, matched faces score below B's other same-direction faces, in ≥ 18 of 20 pairs | **HOLD**, 19/20 |
| P8: with A at cut1200, `w_mean` at B's cut1200 is below B's cut60, in ≥ 16 of 20 pairs | **HOLD**, 20/20 |

- **The P7 failure.** A = k1, B = k3: mean `w` 0.058794 against mean `w_null`
  0.057730, a difference of −0.191°. The results make no claim about whether it is a
  sampling artefact (see caution 2).
- **P8.** `w_mean` at B = cut1200: 0.0295–0.0588 (5.32°–10.58°). At B = cut60:
  0.0842–0.1540 (15.16°–27.71°). Smallest ratio 1.861 (A = k3, B = k1).
- **Reported, cut1200 same-cut.** Mean `w_null` 0.0522–0.0704 (9.39°–12.68°); `W`
  0.0289–0.0579; `w_p10` 0.0075–0.0227; `w_p50` 0.0185–0.0478; `w_p90`
  0.0399–0.1136; both unweighable shares 0.0 on every row; |E7| 1286–1298.
- **O2 recount.** One segment with an empty histogram in the whole corpus
  (k3_cut1200), against 4 of 220 on the old corpus. No weight ever needed it.

### 4.3 Two cautions, recorded in the spec

1. **Mirror pairs.** `w_mean(A,B)` and `w_mean(B,A)` agree to 0.000016 (0.003°) at
   most. The 20 ordered pairs are ten near-duplicate pairs; within a mirror, only
   `w_null` can move a P7 verdict.
2. **The sample.** `w` is constant within a matched segment pair, so the effective
   sample per pair is the number of distinct (S_A, S_B) pairs, not |E7|. The maps hold
   12–13 segments, so at most 169 pairs; in practice about a dozen. The exact count is
   not in the result file (O6).

---

## 5. Housekeeping done

- **CI.** All 17 runs from `0dee108` through `d3baa68` are green. The three
  failures on 19–20 Sep predate that window, each followed by a green run. This
  session's pushes are unchecked.
- **`wall_affinity.py` and its test** (8 Aug; whether the Lévy-walk obstacle rule
  traps robots along walls) moved to `~/Desktop/NSKsim_attic/`, not deleted. No
  handoff quoted a result from them; their 36 tests passed before the move.
- **The unattributed `1 failed, 850 passed`.** One whole-directory run (system
  `python3`): 871 passed, 20 skipped, none failed. Closed as unreproduced under its
  stopping rule.

---

## 6. What was learned

- **A definition a registered prediction needs belongs in the registered text.**
  P7's "same direction" was undefined; Claude Code chose a reading and disclosed it in
  a docstring. Since registration is the commit, the definition went into the text
  before the commit, with a tie rule and tests.
- **Hold back anything that bears on a measure until it is registered**, including a
  count of empty inputs. G2 checked the join without reading one histogram.
- **Gate an index join before trusting it.** The histograms come from saved files and
  the segments from today's code; G2 confirmed they line up on all 20 maps before any
  weight depended on it.
- **Count the sample at the level the quantity varies.** The weight changes only
  between faces, so element counts overstate the sample by a factor of about 100.
- **A results section cannot argue against its own caution.** A clause using |E7| =
  1291 to rule out a small-sample explanation was removed, because caution 2 says
  |E7| is the wrong count.
- **The level was not predicted, and that was right.** Stops can dominate hit counts,
  so no threshold could be placed honestly; the tests were ordinal (P7, P8) and the
  level is reported.

---

## 7. Corrections and records

- **Chat was wrong** that the listed suite did not collect `test_wall_affinity.py`;
  its 36 tests were in the 743. Today's 733 is after their removal and the new tests.
- **Chat's date prediction for `graph_walls.py`** ("before 4 Oct") was imprecise: it
  last changed on 4 Oct, in `cdba182`.
- **Skip counts depend on the interpreter.** Under system `python3`,
  `test_graph_serialiser.py` skips 5 more (torch not importable). Suite counts are
  taken under the project venv.
- **Result files are untracked.** `.gitignore` ignores `experiments/logs/`, so the
  spec carries the S5 file's path and sha256 rather than the file.

---

## 8. Working protocol (changes this session)

- Long Claude Code prompts arrive from chat as downloadable `.md` files under a
  "(for Claude Code)" heading. Registered wording inside them is pasted verbatim.
- A commit block that precedes a stage which refuses an edited tree also checks,
  before pushing, that no tracked file is modified. A block making two commits gates
  each commit's staged file count.
- Numbers quoted in a spec's results are read from the result file by script, never
  retyped from the terminal.
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **O3, a symmetric summary** of A → B and B → A, and the series over cuts. The
  weight is already symmetric to 0.003°; the open question is corroboration, which
  can differ between directions at early cuts, where one map is much smaller.
- **O4, parked-robot shadows.** Whether to report the not-corroborated length that
  parked robots shadow.
- **O5, the deep tail.** Unexplained.
- **O6, distinct segment pairs.** S5's output should record the distinct (S_A, S_B)
  pairs per row, so caution 2 gets an exact count at the next rerun.
- GitHub Actions for `8ca8166` through this handoff's commit: unchecked.
- B3 not judged on k4_cut1200_gated_extfix (relay SIGKILLed); check the stopper's
  grace before the next batch.
- Online explorer and mapper defects (half-cell reading, extent truncation): decide
  before B4.
- Superseded message files in `experiments/logs/msgs/` are scratch and can go.

---

## 10. Next

O3, with O4 and O6 alongside it. Then the rest of the schema (openings, corners, and
the back-to-back face pairing where opposite-side evidence enters), then B3, in the
planned order of Article 3.

---

## 11. The operating lesson

Every weighted number in this session was unseen until its predictions were on
GitHub, and the one ambiguity in the registered text was found before the commit
that fixed it in place. What the measure then showed was not what the design was
built to find: the robots do not see the world from different angles. They see each
wall from nearly the same directions, so their agreement, however complete, is
largely the same evidence counted twice. That is a result Article 3 has to carry,
and the weight is what makes it visible.
