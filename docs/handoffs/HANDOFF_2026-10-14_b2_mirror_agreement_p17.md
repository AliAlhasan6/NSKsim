# HANDOFF — B2 viewpoint weight at mixed cuts; P17 holds, O3 and O6 closed (13 – 14 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-13_b2_risers_o8_closed.md`.

---

## 0. In one paragraph

The previous handoff left O3 open: does the viewpoint weight agree in both directions
when the two maps come from different cuts? Weights on those rows had never been
computed, so a prediction could still be registered. This session registered one and
tested it.

- **v0.19** registered P17: on the 60 mixed-cut mirror pairs with both cuts in
  {60, 120, 240}, the two directions' w_mean differ by at most 1° in at least 54. It
  also registered G10, the gate for the stage that tests P17.
- **S12** computed the weight on all 320 rows of the cut matrix. Before it computed
  any unseen weight, it reproduced S6's C and S5's weights exactly.
- **P17 held, 60 of 60.** The largest difference is 0.056°. In 58 of the 60 pairs it
  is exactly 0.
- **A post-hoc reading** of the stored file shows why. In all 160 mirror pairs, the
  two rows hold the same matched segment pairs. In 156 of them, every matched pair
  also carries the same number of elements from both sides. In the other 4, one
  matched pair has one element more on one side.

So w_mean belongs to the pair of maps, not to the direction, while C does not. O3 and
O6 are closed in v0.20.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
grep -m1 -o "v0\.20" docs/specs/SPEC_b2_divergence_walls.md     # expect v0.20
sha256sum experiments/logs/divergence/s12_mirror_20261013T101707Z.json
# expect 247faaa851e9c29cbd5fdd29f4b03e8f27a9b3faef2968ed721be9405486f1ba
```

---

## 2. Commits this session

| Commit | What |
|---|---|
| `82e6462` | spec v0.19: P17 and G10 registered (O3 mirror agreement, S12) |
| `4f86b8f` | S12 stage: viewpoint weight at mixed cuts (P17), G10 against S5 and S6 |
| `4540c37` | spec v0.20: S12 results (G10 holds, P17 holds 60/60); O3 and O6 closed |
| this commit | docs/handoffs: this handoff |

- **Tests at `4f86b8f`:** 257 divergence tests (213 before) and 63 graph_walls tests.
  The full listed suite gives 960 passed and 33 skipped (project venv).
- **S6's synthetic output** is still byte-identical: `e3d8568b…`, 293,309 bytes.
- **CI** is green for `387f332`, `30f07cf` and `e15c94b` (the previous handoff left
  these unchecked), and for `82e6462` and `4f86b8f`. `4540c37` and this handoff's
  commit are unchecked.

---

## 3. What the code now does

- **The stage.** `divergence_walls.py --stage s12` follows the S6–S11 pattern. Before
  loading anything it:
  - refuses an edited tree;
  - requires `82e6462` to be an ancestor of HEAD, and the spec at HEAD to carry P17
    and G10;
  - reads the registered paragraphs from the spec by script;
  - pins S5's and S6's result files by sha256.
- **Nothing is reimplemented.** C comes from `cut_matrix`, S6's call. The weights come
  from `_side_classify`, S5's call, with S5's arguments. No argument was added to any
  existing function. S_A is `res['seg_of']` and S_B is `res['weights']['seg_b_of']`,
  both already returned per element.
- **The door.** Every weight call goes through `S12Weights`. Its `row()` raises
  `S12GateClosed` on any row S5 does not hold, until G10 checks 1 and 2 and check 3
  on S5's 140 rows have all held. A test proves that it raises. The unseen rows are
  protected by the call itself, not only by the order of the stage.
- **The order:**
  1. C on all 320 rows (check 1);
  2. weights on S5's 140 rows and G3 on the 20 maps (check 2);
  3. check 3 on those 140 rows;
  4. the door opens, and the other 180 rows are computed;
  5. check 3 on all 320 rows;
  6. the mirror pairs, then P17.
- **Never computed:** w_null and E7, anywhere. A prediction about them at mixed cuts
  can still be registered.
- **The record** (`s12_mirror_<UTC>.json`):
  - per row: the seven weight quantities, the n and length of A's weighable elements,
    and the distinct (S_A, S_B) pairs, each with w, n and length;
  - per mirror pair (160): both w_mean, d, and the set it belongs to;
  - P17's verdict, with every failing pair joined on (S_X, S_Y);
  - the reported summaries, and the check on w's symmetry.
- **On a G10 failure,** the stage writes `s12_g10_fail_<UTC>.json` and exits 2. The
  payload is first walked for any verdict, pair record or weight field.
- **The run log** is not written by the code. It is the terminal output, captured with
  `tee`.

---

## 4. Results

### 4.1 S12 (`s12_mirror_20261013T101707Z.json`, sha `247faaa8…`, at `4f86b8f`)

Run time 29.9 s, 340 weight calls. Run log `s12_run_20261013T101637Z.log`
(`7c2effaf…`).

- **G10: HOLD,** all three checks.
  - Check 1: C equals S6's on 320 of 320 rows. Beside it, not gating: 1,280 of 1,280.
  - Check 2: 1,440 of 1,440 weight values equal S5's, over 6 locations. G3 holds on
    20 of 20 maps. Beside them, not gating: 600 of 600.
  - Check 3: 320 of 320 rows within 1e-12. The largest difference is 5.6e-17.
- **P17: HOLD, 60 of 60,** with 54 required.
  - The largest d is 0.000313, or 0.0563°, at k3@cut60 ↔ k4@cut120.
  - d is exactly 0 in 58 of the 60.
- **Reported, not judged:**
  - M1200: d is exactly 0 in 60 of 60.
  - Same cut: exactly 0 in 38 of 40. The other two are S5's cut1200 values, as
    disclosed at registration.
  - w is identical from both sides on all 1,264 matched segment pairs.

### 4.2 The reading (post hoc, not registered)

Read from the stored S12 file only (`s12_reading.txt`, `097d0044…`).

- **The same matched segment pairs.** In 160 of 160 mirror pairs, the two rows hold
  the same matched segment pairs. In 156, the element counts are equal too. In the
  other 4, one matched pair carries one element more on one side: two M60 pairs, both
  on k4@cut120's segment 3, and the two cut1200 pairs.
- **The two rows are opposite directions.** C differs in all 120 mixed-cut pairs
  (|ΔC| 0.118 to 0.809), and L_A does too (11.5 m to 109.7 m). So d = 0 is not one row
  read twice.
- **The effective sample.** A row has 4 to 22 distinct (S_A, S_B) pairs, median 7.
  With both maps at cut1200 it has 12 to 22. That compares with Caution 2's bound of
  169 and with about 1,290 elements.
- **Unweighable elements.** One appears in each direction of k2@cut240 ↔ k3@cut1200.
  Both fall on k3@cut1200's side, where the corpus's only segment with an empty
  histogram lies (O2).
- **The weight at every pair of cuts** is now seen. For the fixed-A series, w_mean is
  non-increasing as B explores:
  - in 19 of 20 series with A at cut1200;
  - in 0, 2 and 1 of 20 with A at cut60, cut120 and cut240.

### 4.3 What it means (v0.20)

- **For Article 3.** w_mean is the same in both directions, but C is not. So the
  convergence series can give one viewpoint weight per pair of maps, and must give C in
  both directions.
- **Agreement is not convergence.** With A held at an early cut, 57 of the 60 series
  rise at some step as B explores. Nothing about these series was predicted beyond
  P8's end-to-end comparison at A = cut1200. They are now seen, so no prediction on
  them can be registered.

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
| G10: S12 reproduces S6 and S5; the pairs give back w_mean | S12 `4f86b8f` | HOLD |
| P17: mirror pairs at mixed cuts, d ≤ 1° in ≥ 54 of 60 | S12 | HOLD 60/60 |

---

## 6. What was learned

- **The margin was the finding.** P17's bound was 1°. The largest d was 0.056°, and 58
  of 60 were exactly 0. The verdict alone says little; the zeros say that the two
  directions count the same wall, element for element.
- **The better guide was in the data, read in the right order.** Chat expected tenths
  of a degree at mixed cuts. The same-cut zeros at cut60, cut120 and cut240, read
  after the bound was fixed, pointed to exact agreement. Fixing the bound first was
  right. The expectation was wrong.
- **When a result is all zeros, rule out the trivial cause.** d = 0 could have meant
  one row read twice. C and L_A, which differ in all 120 pairs, ruled that out.
- **Agreement is not convergence.** A measure can be symmetric and still not settle.
- **Count the effective sample.** At cut1200 a row rests on 12 to 22 matched segment
  pairs, not on about 1,290 elements.
- **A gate built into the call beats a gate in the order of calls.** The door raises
  on an unseen row whatever calls it.
- **Read the words against the registrations.** "Nothing about these series was
  registered" was false for A = cut1200, because P8 compared their end values.

---

## 7. Corrections and records

- **`82e6462`'s commit output was lost** when the terminal was cleared. Rerunning the
  commit block stopped at its first check (HEAD was no longer `30f07cf`), as designed.
  The commit was then checked read-only: parent `30f07cf`, committed spec
  `5127d712…`, MSG_SAME, pushed, CI green, dated 13 Oct 05:23 +0300, which matches
  P17's registration date.
- **v0.19's first draft** stated as a fact that each element "is matched to the
  other's segment". Claude Code showed that each direction picks its own nearest
  element. The wording was changed before commit. The disclosure's "at most 0.000°"
  became exact zeros, and its 163-column line was rewrapped.
- **Claude Code's v0.19 report** gave the spec as 1,419 lines; it was 1,418. This was
  a reporting error only, because the file was pinned by hash.
- **Chat expected d in tenths of a degree** at mixed cuts. That expectation was never
  registered, and it was wrong.
- **v0.20's first draft** had two sentences that were false or unsupported. The first
  was "Nothing about these series was registered", which is wrong because of P8. The
  second, "S12 asserted on every row", rested on asserts being live and was replaced
  with evidence from the file. Claude Code found both, and both were fixed before
  commit.
- **Claude Code's splice runs stopped twice on its own checks:** a path pattern, and a
  count of 19 predictions where there are 18. Nothing was written either time.
- **A latent flaw in S11's fail path, not fixed.** `_no_class_in` bans the key `P15`,
  but the real meta carries `predictions_verbatim.P15`. On a real G9 failure, S11
  would raise an AssertionError instead of writing its fail file. S9 and S10 look the
  same by reading. Their gates held, so no result is affected.

---

## 8. Working protocol (changes this session)

- **A stage that must not compute an unseen quantity guards the call itself,** and a
  test proves the guard raises.
- **A surprising result is checked against independent fields** before it is read.
- **Results text cites what the stored file shows,** not what the code asserted.
- **If a commit block's output is lost,** verify the commit read-only: its parent, the
  committed file's hash, MSG_SAME, origin, and CI. Do not rerun the block.
- **Claude Code may add checks beyond the brief.** When one of its own checks stops a
  run, it reports this, fixes the check, and reruns. Nothing is written in between.
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **O3.** Closed in v0.20.
- **O6.** Closed in v0.20.
- **O2, empty histograms.** Treatment still undecided. S12 found the one unweighable
  element pair, at k2@cut240 ↔ k3@cut1200.
- **O4, parked-robot shadows.** Whether to report the not-corroborated length that
  parked robots shadow. Next.
- **O5, the deep tail.** Unexplained.
- **O9, risers.** (1) Whether the wall measure should count perpendicular elements.
  (2) Whether segment ends, for openings and corners, are read only from elements in
  the direction of the segment's normal axis. Proposed: yes, to be decided in the
  schema spec.
- **w_null and E7 at mixed cuts** are still unseen, so a prediction about them can
  still be registered.
- **The fixed-A weight series** are seen. How Article 3 shows and reads them is open.
- **The fail paths of S9–S11** (§7). Fix them in their own commit before any of those
  stages is rerun.
- **Carried over:**
  - extent boxes are not nested across cuts;
  - B3 was not judged on k4_cut1200_gated_extfix (check the stopper's grace before
    the next batch);
  - the online explorer and mapper defects must be decided before B4;
  - check that Claude Code's auto mode keeps git commit, git push and simulation
    launches out of its allow list.
- **Scratch to remove:**
  - `experiments/logs/splice/spec_v0_18_preview.diff`;
  - the old handoff draft in `~/Downloads/`.

---

## 10. Next

1. **Check CI** for `4540c37` and this handoff's commit.
2. **Decide O4.**
3. **Then the schema spec:** openings, corners and the back-to-back face pairing,
   starting with O9 (2).
4. **Then B3,** in the planned order of Article 3.

---

## 11. The operating lesson

P17 held by a wide margin, and the margin carried the information. The bound was set
for edge noise that, in 156 of 160 pairs, never occurred. A verdict tells you whether
a bound was crossed. Only the values tell you how far away it was, and why. Here they
show that two robots' maps, whatever their sizes, count the same wall, element for
element, from either side. They also show something the bound could not: the weight
agrees in both directions without settling as the robots explore.
