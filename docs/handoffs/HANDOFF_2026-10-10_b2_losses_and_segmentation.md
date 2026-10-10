# HANDOFF — B2 cut matrix, losses and segmentation (8 – 10 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-07_b2_viewpoint_weight.md`.

---

## 0. In one paragraph

O3's form is decided, and the full 320-row cut matrix is computed. Then a chain of four
registered predictions followed one question: does B's evidence about A's walls only
grow as B explores? In net terms, almost always. Element by element, no. With A held
at an early cut, the not-corroborated share rose in 20 of 60 series (P9 failed). Every
one of P3's 20 held series also loses elements (P10 held); the losses were hidden by
much larger gains. The 213 lost elements were then traced:

- **129 segmentation.** In 129 of them, B's two cells around the matched wall are
  unchanged at the later cut, but the face element between them is no longer assigned
  to a wall segment.
- **The rest:** 31 off grid, 29 front cell closed, 21 wall cell freed, and 3 geometry.

Most losses therefore come from our own wall-segment extraction, not from B's map
changing (P11 failed, P12 held). The losses are small: in P3's series, at most
7 elements are lost in a step, against at least 160 gained. The chain was stopped
here by decision. Why segmentation drops these elements is open (O8).

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
grep -m1 -o "v0\.15" docs/specs/SPEC_b2_divergence_walls.md     # expect v0.15
sha256sum experiments/logs/divergence/s9_segmentation_20261009T013501Z.json
# expect 42c7505992ff8047d069cb7092dcdeaf3e8644193ff36484f3892425d307deab
```

---

## 2. Commits this session

| Commit | What |
|---|---|
| `533fa52` | spec v0.11: O3 symmetric form; P9 registered (A held at early cuts) |
| `ce8dd6e` | S6 stage: cut matrix, G4 against S4/S5, tests P9 |
| `9541ea1` | spec v0.12: S6 results; P10 registered (losses under P3) |
| `1cef039` | S7 stage: gains and losses per step, G5 against S6, tests P10 |
| `df7ae16` | spec v0.13: S7 results; P11 registered (wall cells freed) |
| `e81de80` | S8 stage: cell states under losses, G6 against S7, tests P11 |
| `4698880` | spec v0.14: S8 results, two errata; P12 registered (segmentation) |
| `808c4eb` | S9 stage: segmentation or geometry, G7 against S8, tests P12 |
| `1faee18` | spec v0.15: S9 results; O7 closed; O8 opened |
| this commit | docs/handoffs: this handoff |

- **Tests:** 143 divergence tests; the full listed suite gives 816 passed and
  33 skipped (project venv).
- **S6's synthetic output** is byte-identical across every stage added since:
  `e3d8568b…`, 293,309 bytes.
- **CI** is green for every push from `533fa52` through `1faee18`, checked through
  the GitHub Actions API before this handoff was committed. This handoff's own
  commit is unchecked.

---

## 3. What the code now does

- **Stages.** `divergence_walls.py --stage s6 | s7 | s8 | s9`. Each stage:
  - refuses to run on an edited tree, before loading anything;
  - requires its prediction's registering commit to be an ancestor of HEAD;
  - pins the previous stage's result file by sha256;
  - reproduces that stage's stored values exactly before computing anything new.
- **Shared code.**
  - `s6_classify` was extracted from `s6_row`. It is a pure extraction.
  - `within_t1_tolerances` is now the one place T1's inclusive tolerances live (with
    the 1e-9 m slack). Both S7's `within_t1` flag and S9's geometry classes use it.
- **T1 matches assigned elements only.** It compares A's elements with B's elements
  assigned to a wall segment: `face_elements_only(b['el'], b['seg_of'])`. `seg_of` runs
  parallel to B's full element arrays, index for index.
- **`nearest_same_dir`** scans the set it is given. In S7 that set is B's assigned
  elements, so "none within 1 m" means no assigned element within 1 m. The docstring
  now says so (`808c4eb`).
- **Cell lookup.** S8 and S9 find cells by position with `trinary_map.cell_index`.
  - They never use `trinary_map.sample`. It fills positions outside the map with
    UNKNOWN, which would move off-grid elements into "wall cell freed", the class P11
    predicted.
  - S9 finds e′ by `(row, col, dir)`. That is exact: each occupied cell has at most one
    element per direction.

---

## 4. Results

### 4.1 O3's form decided (v0.11)

- **The symmetric form.** The two directions of a pair are reported side by side with
  their difference. No mean and no minimum is reported: when one map is much smaller,
  the two directions measure different things.
- **Convergence** is presented as fixed-A series. The same-cut series is reported, but
  it is not a convergence curve.
- **The cut matrix** has 320 rows. S4 had computed 140 of them (A at cut1200, or A and
  B at the same cut). The other 180 were unseen until S6.
- **Still open under O3:** whether the viewpoint weight's mirror agreement holds at
  mixed cuts (§9).

### 4.2 S6 (`s6_cutmatrix_20261008T225604Z.json`, sha `a3d584f3…`, at `ce8dd6e`)

- **G4: HOLD, 820/820** (S4 420/420, S5 400/400). S4 ran from an uncommitted script, so
  this is the first reproduction of S4's values from committed code.
- **P9: FAIL, 40/60 series.**
  - 21 steps in 20 series rise.
  - Each rise is 1, 2, 3 or 5 elements (0.1–0.5 m). The largest is 0.8 % of A's walls.
- **The level.** Against B at cut1200, 0.72–5.76 % of A's early-cut walls are not
  corroborated.

Not-corroborated share, min–max over the 20 ordered pairs (`*` = unseen before S6):

| A's cut | B at 60 | B at 120 | B at 240 | B at 1200 |
|---|---|---|---|---|
| 60 | 0.0040–0.4401 | 0.0040–0.4378 `*` | 0.0081–0.4332 `*` | 0.0081–0.0576 `*` |
| 120 | 0.3169–0.6217 `*` | 0.0036–0.5779 | 0.0072–0.4705 `*` | 0.0072–0.0174 `*` |
| 240 | 0.4982–0.7540 `*` | 0.2999–0.6724 `*` | 0.0114–0.5416 | 0.0073–0.0323 `*` |
| 1200 | 0.6805–0.8186 | 0.5081–0.5970 | 0.2587–0.3903 | 0.0054–0.0409 |

### 4.3 S7 (`s7_gainloss_20261009T000606Z.json`, sha `b8b7cb6c…`, at `1cef039`)

- **G5: HOLD** (shares 320/320; the 21 failing steps 21/21).
- **P10: HOLD, 20/20** of P3's series have a loss (10 required). Per step, P3's series
  gain 160–472 elements and lose 0–7.
- **P9's series by A's cut:**

  | A's cut | Gains | Losses | Series with a loss |
  |---|---|---|---|
  | 60 | 1,902 | 31 | 17/20 |
  | 120 | 6,181 | 41 | 17/20 |
  | 240 | 12,341 | 63 | 20/20 |

- **Losses come with B's step.** For example, B = k2 at step 240 → 1200 loses elements
  against all four As: 7, 4, 7 and 4. The count varies with A.
- **213 lost elements** (78 in P3's series, 135 in P9's), in three distance bins:
  - **102 one-cell holes.** The nearest same-direction B element at k′ is within eps
    across and beyond rho/2 along; the median along distance is 0.0993 m, one cell,
    i.e. the neighbouring element on the same wall line.
  - **5 shifted.** Beyond eps across, within 1 m.
  - **106 with nothing within 1 m** (Euclidean).

  None lies within both of T1's tolerances.

### 4.4 S8 (`s8_cellstates_20261009T010602Z.json`, sha `f71f4597…`, at `e81de80`)

- **G6: HOLD** (the lost set 213/213; b's cells at k 213/213 as required).
- **P11: FAIL, 21/213** wall cells freed, 107 required.

| Class | One-cell hole | Shifted | None within 1 m | Total |
|---|---|---|---|---|
| off grid | 0 | 0 | 31 | 31 |
| wall cell freed | 15 | 1 | 5 | 21 |
| front cell closed | 11 | 2 | 16 | 29 |
| unchanged | 76 | 2 | 54 | 132 |

- **Wall cells freed:** 12 now free, 9 now unknown.
- **Off grid.** All 31 are off by their front cell, all at step cut60 → cut120, and all
  with B = k0 or k1. k0's image top edge shrinks from 3.8793 m at cut60 to 3.8163 m at
  cut120, so a map's extent box is not nested across cuts.
- **Lattice offsets between cuts,** over 15 steps: up to 0.4138 cells in x and 0.3699
  in y.

### 4.5 S9 (`s9_segmentation_20261009T013501Z.json`, sha `42c75059…`, at `808c4eb`)

- **G7: HOLD, all four parts.** Part 4 checks the carve-out instead of assuming it; it
  is stricter than the spec.
- **P12: HOLD, 75/78** segmentation, 39 required. The 78 are the unchanged losses
  outside the carve-out (76 one-cell holes and 2 shifted). They are not P3's 78 losses.
  The carve-out is 54/54 segmentation.

| Class | One-cell hole | Shifted | None within 1 m | Total |
|---|---|---|---|---|
| segmentation | 75 | 0 | 54 | 129 |
| geometry, across | 0 | 2 | 0 | 2 |
| geometry, along | 1 | 0 | 0 | 1 |

The three geometry elements are all at step cut60 → cut120, with A at an early cut:

| A | B | A's cut | Across | Along | Class | Lattice offset (cells) |
|---|---|---|---|---|---|---|
| k3 | k0 | 60 | 0.0675 m | 0.0785 m | along | (+0.0000, +0.3699) |
| k4 | k2 | 120 | 0.1850 m | 0.0247 m | across | (+0.0000, −0.1641) |
| k4 | k3 | 120 | 0.1748 m | 0.0056 m | across | (−0.1576, −0.1679) |

All three fall inside the bounds Claude Code derived: across at most eps + rho
(0.2707 m), along at most rho (0.1 m).

### 4.6 All 213 losses

| Cause | Count |
|---|---|
| Segmentation: cells unchanged, e′ not assigned at k′ | 129 |
| Off grid at k′ | 31 |
| Front cell closed | 29 |
| Wall cell freed | 21 |
| Geometry | 3 |

---

## 5. Gates and predictions to date (B2 walls)

| Item | Stage, commit | Result |
|---|---|---|
| G0: maps rebuilt from admitted rays | S3 | HOLD 20/20 |
| G1: parked robots at spawn | S3 | HOLD 80/80 |
| X1, X6: controls built to fail | S3 | failed, as intended |
| C0: contradiction self-test | v0.1–v0.3 | failed; T2 removed (v0.4) |
| P3: A at cut1200, not-corroborated share non-increasing | S4 `9eb3f17` | HOLD 20/20 |
| P4: corroborated walls are real walls | S4 | HOLD 80/80 |
| P6: ≥ 90 % corroborated at full coverage | S4 | HOLD 16/16 |
| G2: histogram join | `25ba70e` | HOLD 20/20 |
| G3: a map against itself | S5 `2161665` | HOLD 20/20 |
| P7: matched faces below other same-direction faces | S5 | HOLD 19/20 |
| P8: weight falls as B explores | S5 | HOLD 20/20 |
| G4: S6 reproduces S4, S5 | S6 `ce8dd6e` | HOLD 820/820 |
| P9: A at early cuts, non-increasing | S6 | **FAIL 40/60** |
| G5: S7 reproduces S6 | S7 `1cef039` | HOLD |
| P10: losses in ≥ 10 of P3's 20 series | S7 | HOLD 20/20 |
| G6: S8 reproduces S7 | S8 `e81de80` | HOLD |
| P11: ≥ half of wall cells freed | S8 | **FAIL 21/213** |
| G7: S9 reproduces S8 | S9 `808c4eb` | HOLD |
| P12: ≥ half segmentation | S9 | HOLD 75/78 |

---

## 6. What was learned

- **A net change hides cancellation, again.** P3 held because gains swamped losses.
  Splitting each step into gains and losses showed losses in every series. This is the
  lesson of B1.8 (sum the size of the error, not the net), learned a second time in a
  different measure.
- **A median over a mixture means nothing.** S7's median across distance of 0.17 m
  described neither the one-cell holes nor the far losses. Binning first showed the
  three groups.
- **Count in the set the measure counts in.** The share counts only assigned elements.
  The gain and loss counts had to use the same set, and the test that proves it (L13)
  was mutation-checked: it fails when the restriction is removed.
- **A class list in a registration must be exhaustive.** A structure-only audit (YAML
  headers, stored positions, no cell states) found that elements can fall off the later
  grid. The off-grid class went into P11 before the commit.
- **A default that fills missing data can bias a test.** `trinary_map.sample` fills
  positions outside the map with UNKNOWN. Used in S8, it would have pushed off-grid
  elements into the class P11 predicted.
- **Check a carve-out; don't assume it.** The 54 "by construction" elements were
  checked in G7 part 4.
- **Each stage reproduces the last one before computing anything new.** G4 to G7 form a
  chain, so every new number rests on numbers already on the record.
- **A failed prediction is still a step.** P9 and P11 failed and stand as registered.
  Each failure shaped the next prediction.

---

## 7. Corrections and records

- **Chat was wrong twice about the losses.**
  - It called the 107 nearby losses "shifted walls". 102 of them are one-cell holes
    along the same wall line; only 5 are shifted.
  - It said a lattice shift alone could push an element past T1's along limit. It
    cannot: on a continuous wall line whose elements are all assigned, some element
    lies within rho/2 along. An along loss also needs a neighbour that is missing or
    not assigned.
- **The v0.13 commit message has two wrong numbers.** Both are corrected in v0.14's
  errata. The spec stated neither, so no registered text changed.
  - The off-grid proxy is 7 of 213 with a worst margin of 2.44 cm, not 6 and 3.3 cm.
  - The lattice offsets reach 0.4138 cells in x and 0.3699 in y, not 0.175.
- **The v0.11 draft** said the viewpoint weight at mixed cuts was unseen. It was false:
  S5's P8 computed weights at A@1200 against B@60. The sentence was amended before the
  commit.
- **S4's code is not in the repo's history.** It ran from an uncommitted script. G4 now
  reproduces its values from committed code.
- **Chat's wall-time estimates were too high.** S6 took 17.4 s against an estimate of
  0.5–1.5 min.
- **P11's text repeats a clause:** "off-grid elements count against P11" appears twice.
  The two are consistent, and the text is registered, so it stays.
- **Wrong when built, refuted by the real run.** Claude Code found "unchanged" (S8)
  unreachable on a shared lattice and the geometry classes (S9) not reachable
  end-to-end on a synthetic corpus. The real run, with offset lattices, gave 132
  unchanged and 3 geometry.
- **v0.15, fixed before its commit.** O8's draft called b "the same face". b is a face
  element, so O8 now calls it e′'s counterpart at the earlier cut. The draft commit
  message had dropped every apostrophe; they were restored.

---

## 8. Working protocol (changes this session)

- **Every stage prompt forbids computing any unseen quantity on the real maps** until
  the stage is committed. Tests use synthetic inputs only.
- **On a gate failure, a stage computes nothing unseen.** Tests prove this with a spy.
- **Each stage pins the previous result file** by sha256, and requires the registering
  commit to be an ancestor of HEAD.
- **Before registering, a structure-only audit** of real data is allowed: headers,
  stored positions, counts, never the predicted quantity. Its purpose is to find
  missing classes.
- **Commit blocks for a spec:**
  - check today's date against the registration header's date;
  - grep for the registration header;
  - when an amendment is pending, grep that the new phrases are present and the old
    ones are absent.
- **Spec result numbers** are spliced in by a script that reads the result file and
  exits rather than print a false statement.
- **Splice scripts are kept** in `experiments/logs/splice/` (gitignored), not in `/tmp`,
  which is wiped on reboot. v0.15's is `splice_spec_v0_15.py`.
- **Commit messages keep their apostrophes.** Inside printf's single quotes, write each
  as `'\''`.
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **O3.** Form decided (v0.11). Open: whether the viewpoint weight's mirror agreement
  holds at mixed cuts. On the 180 rows that were unseen before S6, the weight is still
  unseen, so a prediction about it can still be registered.
- **O4, parked-robot shadows.** Whether to report the not-corroborated length that
  parked robots shadow.
- **O5, the deep tail.** Unexplained.
- **O6, distinct segment pairs.** S5's output should record the distinct (S_A, S_B)
  pairs per row.
- **O7, losses.** Closed in v0.15 (`1faee18`) with §4.6's breakdown.
- **O8, segmentation across cuts.** Why `graph_walls` leaves unassigned, at k′, a face
  element whose two cells are unchanged and whose counterpart b was assigned at k.
  Open; not pursued. The chain stopped at S9 by decision on 10 Oct 2026.
- **CI** for this handoff's commit is unchecked.
- **Extent boxes are not nested across cuts** (k0's top edge, §4.4). This bears on the
  open mapper defect, extent truncation.
- **B3** was not judged on k4_cut1200_gated_extfix (relay SIGKILLed). Check the
  stopper's grace before the next batch.
- **Online explorer and mapper defects** (half-cell reading, extent truncation): decide
  before B4.
- **Superseded message files** in `experiments/logs/msgs/` are scratch and can go.

---

## 10. Next

1. **Check CI for this handoff's commit.**
2. **Decide whether O8 matters for Article 3.** Its net effect is small: P9's largest
   rise is 0.8 % of A's walls. But it means corroboration of single wall elements is
   not stable across cuts, for a reason inside our pipeline. At minimum, Article 3
   reports it as a limitation.
3. **Then O4 and O6.**
4. **Then the rest of the schema:** openings, corners, and the back-to-back face
   pairing.
5. **Then B3,** in the planned order of Article 3.

---

## 11. The operating lesson

P3 holding looked like clean evidence that each robot's view of the others' walls
only improves. It was a net change. Asking what the net hid led, in four registered
steps, out of the robots' maps and into our own segmentation code. Two of those four
predictions failed, and both failures were needed to get there. Every number in the
chain was registered before it was seen. Every stage reproduced the one before it
exactly. The answer is not the one the design hoped for, but it is one the article
can defend.
