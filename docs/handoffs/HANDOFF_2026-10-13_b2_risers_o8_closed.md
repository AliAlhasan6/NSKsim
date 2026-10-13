# HANDOFF — B2 segmentation losses traced to risers; O8 closed (10 – 13 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-10_b2_losses_and_segmentation.md`.

---

## 0. In one paragraph

The previous session ended with O8 open: 129 of the 213 lost wall elements had both
cells unchanged at the later cut, yet the face element between them was no longer
assigned to a wall segment. This session reopened O8 and followed it in two registered
stages to a single cause.

- **S10** asked whether the segment behind each loss changed between cuts. Mostly it
  did not: 15 of the 19 segments are intact, and in the other 4 the nearest change is
  1.30–4.74 m from the lost element.
- **S11** traced the extraction and named the rule that drops each element. It is not
  the line's distance (P15 failed, 1 of 22). It is facing (R4, 21 of 22).
- **A post-hoc reading** of the stored trace shows why. All 22 elements are risers:
  faces perpendicular to the wall's line. `graph_walls` admits a riser only when the
  refitted line tilts toward it, because FACING_MIN is 1e-9. The tilt is small
  (0.06–2.01°), and its sign reversed between the two cuts in all 22.

O8 is closed in v0.18 as a limitation of the wall measure. O9 is open: whether segment
ends, for openings and corners, should ignore risers.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
grep -m1 -o "v0\.18" docs/specs/SPEC_b2_divergence_walls.md     # expect v0.18
sha256sum experiments/logs/divergence/s11_trace_20261012T233219Z.json
# expect 006a7f8fc2774cf1324b613d42e70d8e4d103bd8e8298624a8630522c58663dd
```

---

## 2. Commits this session

| Commit | What |
|---|---|
| `e15c94b` | spec v0.16: O8 reopened; P13a, P13b, P14 registered (G8, S10) |
| `ed2a618` | S10 stage: segments behind the unassigned elements, G8 against S9 |
| `dc754de` | spec v0.17: S10 results; P15, P16 registered (G9, S11) |
| `dc5e2a3` | S11 stage: trace of the rule that drops each e′, G9 against S10 |
| `387f332` | spec v0.18: S11 results; riser reading; O8 closed, O9 opened |
| this commit | docs/handoffs: this handoff |

- **Tests at `dc5e2a3`:** 213 divergence tests, 63 graph_walls tests; the full listed
  suite gives 916 passed and 33 skipped (project venv).
- **S6's synthetic output** is still byte-identical: `e3d8568b…`, 293,309 bytes.
- **CI** is green for `a22d450`, `dc754de` and `dc5e2a3`. `e15c94b`'s run was in
  progress when checked and was not rechecked. `387f332` and this handoff's commit are
  unchecked.

---

## 3. What the code now does

- **Stages.** `divergence_walls.py --stage s10 | s11`, in the S6–S9 pattern: refuse an
  edited tree before loading anything; require the registering commit to be an
  ancestor of HEAD; pin the previous result file by sha256; reproduce the previous
  stage exactly before computing anything new.
- **Shared code.**
  - `recompute_unchanged` was lifted out of S9 (a pure extraction). S10 reproduces S9
    through it.
  - `s10_derive` was lifted out of S10. S10 and S11 both call it, so S11 reproduces
    S10 without writing a second S10 result file.
- **S10.** Rebuilds B's elements at k, finds the segment S_b that held b, and looks up
  every member's two cells at k′ by position (`trinary_map.cell_index`, never
  `sample`). A segment is *changed* if any member's wall cell is no longer occupied,
  any front cell no longer free, or any cell is off grid; otherwise *intact*. It also
  tests flanking and reports a base rate over all of B's segments.
- **The tracer** (`graph_walls.py`). An optional argument, default None, threaded
  through `graph_and_elements` and `extract_with_elements`. It records every peak
  iteration on every exit path, copies what it records, and never touches the tie
  break. With None, nothing changes; the one line in the loop that is not a hook,
  `cand_first = cand`, only keeps a reference.
- **S11.** Traces B at k and k′, with the tracer off and on, and requires identical
  `seg_of` and segments (G9 check 2). It finds the home iteration H (the one that
  assigned the most counterparts of S_b's members) and classes each e′:
  - **No home:** H does not exist;
  - **R3:** e′ is farther than eps from H's refitted line;
  - **R4:** e′ does not face H's refitted line;
  - **R5+R6:** e′ is a candidate of H in a run that was not accepted;
  - **Other:** none of these.
- **What the S11 file does not store:** the directions of H's members, and e′'s
  position along H's span. Any question needing them needs a new stage.

---

## 4. Results

### 4.1 The audit (10 Oct, code only)

Seven ways an element can end unassigned (`seg_of` = −1):

| Rule | What it does | Reach |
|---|---|---|
| R1, vote floor | no live window holding it reaches vote_min = 3.5355 | whole map |
| R2, one-shot bins | its seed bin was spent by an earlier iteration | whole map, greedy order |
| R3, eps band | farther than eps = 0.1707 m from the iteration's refitted line | the whole infinite line |
| R4, facing | does not face the line (dot ≤ FACING_MIN = 1e-9) | its own cells and the line |
| R5+R6, gaps and length | its run, cut at gaps ≥ g = 0.22 m, is shorter than l_min = 0.5 m | the run |
| R7, abort guards | end an iteration early; they feed R2 | whole map |

`MIN_GAP_FACTOR` (0.25) only labels bridged gaps in a segment's report; it cannot
change membership.

### 4.2 S10 (`s10_segments_20261011T134922Z.json`, sha `fa750aa9…`, at `ed2a618`)

- **G8: HOLD,** all three checks.
- **P13a: FAIL.** 2 of the 5 distinct S_b behind the 54 "none within 1 m" losses are
  changed; 3 required.
- **P13b: HOLD.** 12 of the 14 distinct S_b behind the 75 one-cell holes are intact;
  7 required.
- **P14: HOLD.** 0 of the 22 distinct e′ are flanked.
- The 129 rows are 22 distinct events, each repeated once per A that matched it.
  Judged per row, every verdict would have been the same.
- 15 of the 19 S_b are intact. In the 4 changed ones, the nearest changed member is
  1.30–4.74 m from e′.
- P13a's reason (the minimum length) is refuted: the 5 S_b behind the 54 are
  3.85–7.15 m long, and 3 are intact.
- Base rate, not registered: 44 of 102 segments change across the 13 steps (0.43);
  behind the losses, 4 of 19 (0.21).

### 4.3 S11 (`s11_trace_20261012T233219Z.json`, sha `006a7f8f…`, at `dc5e2a3`)

- **G9: HOLD,** all three checks.
- **P15: FAIL.** 1 of the 22 is in R3; 11 required.
- **P16: HOLD.** 0 of the 22 are in Other.
- **R4: 21.** No home 0, R5+R6 0. All 19 S_b hold an R4 element; one (k4,
  cut60 → cut120) also holds the R3 element.

### 4.4 The riser reading (post hoc, not registered)

Read from the stored S11 file only, after the verdicts.

- S11's reported "angle" lies between the refitted line and the element's own face
  tangent, folded into 0–90°. So 90° minus the angle is the line's tilt.
- All 22 e′ are perpendicular to H's refitted line, within 1.13°.
- In all 22, b faced the line that assigned it at k (dot > 1e-9), and e′ does not face
  H's line at k′ (dot ≤ 1e-9). In size, each dot is the sine of the line's tilt: at
  most 0.0351 at k and 0.0197 at k′.
- In all 22, the tilt, measured from the same axis, has opposite signs at k and k′.
  The tilts are 0.06–2.01°.
- The R3 element (k4, e′ 83) is a riser too. It is R3 only because R3 is tested
  first: it lies 1.167 eps from H's line.
- Where b's place in its run is stored (13 of 22), b is the first, second, last or
  second-to-last element, so where it is known, b sat at an end of its run.

### 4.5 What it means (v0.18)

- **Mechanism.** A riser is a face perpendicular to its wall's line. W4 admits it only
  if the dot of the normals exceeds 1e-9, and for a riser that dot is the sine of the
  line's tilt. The refitted line is fitted to everything along the infinite line, so
  its tilt can change sign between cuts while the wall's own cells do not. When it
  does, the riser leaves the segment.
- **For the measure.** Whether B corroborates one of A's risers depends on the sign of
  a small tilt in B's line, not only on B's evidence. The effect is small: P9's
  largest rise is 0.8 % of A's walls, and no step in P3's series loses more than 7
  elements while every step gains at least 160. Article 3 will report it as a
  limitation of the wall measure.

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
| G8: S10 reproduces S9 | S10 `ed2a618` | HOLD |
| P13a: ≥ half of S_b behind the 54 changed | S10 | **FAIL 2/5** |
| P13b: ≥ half of S_b behind the 75 intact | S10 | HOLD 12/14 |
| P14: no e′ flanked | S10 | HOLD 0/22 |
| G9: S11 reproduces S10; tracer changes nothing | S11 `dc5e2a3` | HOLD |
| P15: ≥ 11 of 22 excluded by the line (R3) | S11 | **FAIL 1/22** |
| P16: no e′ in Other | S11 | HOLD 0/22 |

---

## 6. What was learned

- **A local check cannot clear a global rule.** S9 looked at e′'s own two cells, but
  segmentation reads the whole map. S10 had to look at the whole segment, and S11 at
  the extraction itself.
- **Count events, not rows.** The 129 losses were 22 events, each repeated once per A.
  The unit was fixed before anything at k′ was computed.
- **A prediction's reason can fail while its direction holds.** P13b held, but P13a's
  reason (length) was wrong. P15 failed because its reason (distance) was wrong; the
  line was the cause, through facing.
- **"Same direction" hid the geometry.** Risers share b's direction but not the wall's.
  P16's first draft reasoned the other way and would have failed at 21 of 22. The
  review before registration caught it.
- **A floor written for exact arithmetic became a sign test.** FACING_MIN = 1e-9 was set
  so that cos(90°) counts as zero. After a refit, any tilt at all decides whether a
  perpendicular face belongs to the wall.
- **Check the words, not only the numbers.** The splice script refused "sub-degree"
  because two tilts exceed 1°. Bounds are now floored and ranges rounded.
- **A post-hoc reading is labelled and carries no verdict.** It reads only stored
  results and stops when a quantity is not stored.

---

## 7. Corrections and records

- **Chat's v0.16 draft counted per loss** (thresholds 27 and 38; P14 "0 of 129").
  Claude Code's structure audit showed the 129 rows are 22 events. The unit was changed
  to distinct S_b and distinct e′ before registration.
- **v0.16's G8 check 1 named 213 classes**, but S9 holds 132 rows. Fixed before
  registration. "Never uses `trinary_map.sample`" was narrowed to the cell lookup.
- **v0.16 was committed before chat's check block ran**, so that block's diff checks
  passed on nothing. It was verified afterwards: committed 10 Oct 19:52 +0300, matching
  the registration date; message identical to its file; the five expected lines
  removed. Who ran the commit was not recorded.
- **v0.17's "at least 1.30 m"** was true only after rounding (1.299314 m). It became
  1.29 m before commit.
- **v0.17's Part B had five flaws**, found by Claude Code before commit: "No home" was
  unreachable, "refitted line" named two lines, the trace covered k′ only, G9 check 1
  would have written a second S10 file, and P16's reason was wrong. All were fixed
  before commit.
- **Chat's grep for "at least 11 of the 22"** was lowercase; P15 begins with "At".
- **P15 was chat's prediction, and it failed.**
- **v0.18's first draft was refused by the splice**: "sub-degree" was false
  (k2 e′ 1022, −2.0110° at k; k3 e′ 571, +1.0104° at k′). The second draft fixed six
  things, listed in `387f332`'s commit message.
- **A stale prompt.** On 13 Oct the v0.15 prompt from 10 Oct was given to a fresh Claude
  Code session as if it were the handoff. Its own Step 1 audit (HEAD = `808c4eb`)
  stopped it; nothing changed.
- **Two copies of the 10 Oct handoff.** The one in `~/Downloads/` (`0335e094…`, 16,902
  bytes) is an earlier draft. The committed one (`be04648e…`, 17,730 bytes) is final.

---

## 8. Working protocol (changes this session)

- **Every Claude Code prompt opens with an audit of HEAD and the tree**, so a stale
  prompt stops itself.
- **Counting units are fixed before any unseen quantity is computed.**
- **Post-hoc readings** use stored result files only, are labelled "post hoc, not
  registered", and stop rather than recompute from the maps.
- **Splice scripts check prose as well as numbers.** A bound ("at least", "at most",
  "within") is floored or ceiled so that it is true of the file; a range is rounded.
  Adjectives are checked. A cited commit message is read. Any false statement stops
  the write.
- **Drafts from chat** arrive as files, are copied to `experiments/logs/splice/` and
  are pinned by sha256 in the prompt.
- **Commit blocks** confirm the file is still uncommitted before committing, and
  compare the committed message with its file (`MSG_SAME`).
- **Claude Code never lists commit commands** (reaffirmed).
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **O3.** Whether the viewpoint weight's mirror agreement holds at mixed cuts. Still
  registrable: S10 and S11 computed no viewpoint weight. Decide (register or drop on
  the record) before O6, because O6 will run S5's code on the mixed-cut rows.
- **O4, parked-robot shadows.** Whether to report the not-corroborated length that
  parked robots shadow.
- **O5, the deep tail.** Unexplained.
- **O6, distinct segment pairs.** S5's output should record the distinct (S_A, S_B)
  pairs per row.
- **O8.** Closed in v0.18.
- **O9, risers.** (1) Whether the wall measure should count perpendicular elements at
  all. Not proposed now: it would change the registered measure. (2) Whether segment
  ends, for openings and corners, are read only from elements in the direction of the
  segment's normal axis, so a riser leaving cannot move an end. Proposed: yes, to be
  decided in the schema spec.
- **Not checked, because S11 does not store them:** the directions of H's members, and
  e′'s position along H's span.
- **Carried over:** extent boxes are not nested across cuts; B3 was not judged on
  k4_cut1200_gated_extfix (check the stopper's grace before the next batch); the online
  explorer and mapper defects must be decided before B4.
- **Claude Code's auto mode** was set up on 11 Oct (`~/.claude/settings.json`). Check
  that `claude auto-mode config` keeps git commit, git push and simulation launches out
  of its allow list.
- **Scratch to remove:** `experiments/logs/splice/spec_v0_18_preview.diff` (the refused
  first draft's diff) and the old handoff in `~/Downloads/`.

---

## 10. Next

1. **Check CI** for `387f332` and this handoff's commit.
2. **Decide O3:** register a prediction on the mixed-cut viewpoint weight, or drop it on
   the record.
3. **Then O4 and O6.**
4. **Then the schema spec:** openings, corners and the back-to-back face pairing,
   starting with O9 (2).
5. **Then B3,** in the planned order of Article 3.

---

## 11. The operating lesson

Over two sessions, nine predictions followed one question, and four failed: P9, P11,
P13a and P15. Each failure narrowed the cause: from B's map, to our segmentation, to
one rule, to one threshold acting on one kind of element. The answer is a constant
written so that cos(90°) would count as zero. After a refit, it turns a tilt as small
as 0.06° into a yes-or-no on whether a perpendicular face belongs to the wall. No single check would have found it. A chain of checks, each registered before
it was run, did.
