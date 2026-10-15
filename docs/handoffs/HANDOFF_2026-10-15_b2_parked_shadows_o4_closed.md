# HANDOFF — B2 parked-robot shadows; P18 and P19 hold, O4 closed (14 – 15 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-14_b2_mirror_agreement_p17.md`.

---

## 0. In one paragraph

The previous handoff left O4 next. In each run one robot explores and four stay parked
at their spawns. A parked robot can stop B's beams, so some of A's walls may be not
corroborated only because B's view was blocked, not because the maps disagree. This
session measured that share, Δ.

- **v0.21** (`2a88223`) registered P18 and P19, with G11, the gate for stage S13, and
  X7, a decoy disk. A structure audit and five checks of the drafts came before it;
  six drafts were needed. The thresholds and the two prediction sentences never
  changed; most of what the checks found was in the reasons.
- **S13** (`58b654b`) extends each blocked beam past the parked robot and marks the
  first of A's walls it would have reached. Δ is the share of A's walls that are
  marked and not corroborated.
- **G11 held, and P18 and P19 held 20 of 20.** With A at cut1200 and B at cut60, the
  largest Δ is 0.0023, against a bound of 0.05. With both maps at cut1200, Δ is 0 in
  all 20 pairs.
- **v0.22** (`0c4aff7`) records the results and closes O4.

So parked robots do not explain P3's series or P6's residual, as far as beams with a
hit can show.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
grep -m1 -o "v0\.22" docs/specs/SPEC_b2_divergence_walls.md     # expect v0.22
sha256sum experiments/logs/divergence/s13_shadows_20261014T095408Z.json
# expect cd1ada604646b786157a87b3a4adf1f1701aaf2ff613de6c9eb2822081a08023
```

---

## 2. Commits this session

| Commit | What | Committed |
|---|---|---|
| `2a88223` | spec v0.21: O4 shadows; P18, P19, G11 and X7 registered (S13); O10 recorded | 14 Oct 12:00 +0300 |
| `58b654b` | S13 stage: parked-robot shadows (P18, P19), G11 and X7 | 14 Oct 12:45 +0300 |
| `0c4aff7` | spec v0.22: S13 results (G11 holds, P18 and P19 hold 20/20); O4 closed | 15 Oct 23:55 +0300 |
| this commit | docs/handoffs: this handoff | |

- **Tests at `58b654b`:** 309 divergence tests (257 before) and 63 graph_walls tests.
  The full listed suite gives 1,012 passed and 33 skipped (project venv). S13 added 52
  cases in 36 test functions.
- **Mutation check.** Claude Code applied 15 deliberate bugs, one at a time. The tests
  caught 14. The fifteenth loosens the 7.9 m check on the return point, which changes
  no mark, because the check on every cell entered stops the walk at the same place.
- **S6's synthetic output** is still byte-identical: `e3d8568b…`, 293,309 bytes.
- **CI** is green for `4540c37` and `8dbc3c1` (the previous handoff left these
  unchecked), and for `2a88223` and `58b654b`. `0c4aff7` and this handoff's commit are
  unchecked.

---

## 3. The measure and the code

### 3.1 The measure (v0.21)

- **Spawn disk D_j:** centre s_j, radius r = 0.176 m. The parked disks of B's run are
  the four with j ≠ B.
- **Blocked beam:** one of B's admitted rays with a hit whose return lies inside a
  parked disk (at most r + 1e-9 m from its centre). A ray whose origin lies inside a
  disk is excluded.
- **Mark:** walk A's grid along the beam's extension, in A's frame:
  - cell membership by `trinary_map.cell_index`; at a corner, the x-step first;
  - the starting cell and every spawn-disk cell skipped; free and unknown cells passed;
  - stop at the first occupied cell entered below 7.9 m from the ray's origin, and mark
    its entry face if it is one of A's measured elements; nothing is marked at 7.9 m
    or off A's grid.
- **Shadowed:** marked, and not corroborated by B. **Δ** is shadowed length over A's
  measured length.
- **What Δ bounds:** for beams with a hit, Δ is an upper bound. It cannot count a
  shadow behind a reading at or below `range_min` (0.16 m), which has no hit.
- **G11:** (1) reproduce S6's C and A's length on the 80 rows with A at cut1200;
  (2) reproduce S3's G0 `disagreeing_cells` on the 20 maps; (3) every parked disk holds
  a blocked-beam return, 80 of 80; (4) X7, a decoy disk at the spawns' centroid, holds
  no hit return; (5) S6's synthetic output byte-identical.
- **P18:** with A at cut1200 and B at cut60, Δ ≤ 0.05 in at least 18 of 20 pairs.
  **P19:** with both at cut1200, Δ ≤ 0.01 in at least 18 of 20.

### 3.2 The stage

`divergence_walls.py --stage s13` follows the S6–S12 pattern.

- **Before loading anything** it refuses an edited tree, requires `2a88223` to be an
  ancestor of HEAD, and requires the spec at HEAD to carry P18, P19 and G11. It stores
  the registered text verbatim. It finds every number it applies in that text and
  refuses on any mismatch: r, both 1e-9 m tolerances, 7.9 m, 0.3 m, 5 %, 1 % and 18 of
  20. It pins the S6 file (`a3d584f3…`) and the S3 file (`dd0b3796…`) by sha256.
- **Reused, not reimplemented:** `cut_matrix`, `s6_classify` and `shares` (S6);
  `diagnose_karto.admitted_rays`, `rebuild` and `gate` (G0); `load_side`;
  `trinary_map.cell_index`; `to_frame_a` (G1's translation).
- **Added:** `blocked_beams`, `walk_mark` and `delta_row`, with small helpers; the door,
  `S13Shadows`, whose method raises `S13GateClosed` until all five G11 checks hold;
  `run_s13` and its fail-file guard.
- **Order:** check 5, check 1, check 2, checks 3 and 4, then the door opens, Δ on all
  320 rows, the verdicts, and the reported items. A failed check runs nothing after it.
- **Verdicts in integers:** P18 holds for a pair when `100 * shadowed_n <= 5 *
  measured_n`; P19 when `<= 1 * measured_n`.
- **The fail path** is tested with the meta that `run_s13` builds from the real spec,
  and writes its file rather than raising. This covers the S9–S11 flaw for S13 only.
- **Where the spec was silent,** Claude Code chose:
  - a pair with no measured element counts against the prediction;
  - `disagreeing_cells` is G0's observed cells minus agreeing cells (checked against
    the stored S3 file before commit, §4.2);
  - S6's synthetic corpus is restated in the module, and a test proves it equal to the
    test file's;
  - the origin rule also applies to the decoy disk;
  - a walk that reaches 7.9 m and leaves the grid at once reports the 7.9 m end.

---

## 4. Results

### 4.1 Before registration: the audit and the checks

None of these counted a return inside any disk, or computed a mark or Δ. Files are in
`experiments/logs/splice/`: `o4_audit_20261013T225430Z.txt` (`6a71de9b…`),
`o4_draft4_check_tilt.txt` (`b0388289…`), `o4_draft5_check_pitch.txt` (`ce086949…`).

- **Laser:** `range_min` 0.16 m and `range_max` 8.0 m in every scan header of all five
  runs; Karto's `max_laser_range` is 7.9 m in all 20 replay configs.
- **Spawns:** a pentagon 0.900–0.904 m from (0, 0); neighbours 1.06 m apart,
  non-neighbours 1.71–1.72 m.
- **Clearance:** in all 20 maps, every parked spawn's occupied cells lie within
  0.115 m of it, none lies between 0.176 m and 0.5 m, and the centroid is at least
  0.791 m from any occupied cell.
- **B's closest approach** to a parked spawn is 0.18890 m (k0's run, s_1).
- **Parked robots** sit 0.0016 m from their spawns by truth. G1's 0.066 m is the worst
  distance between a parked robot as mapped and its spawn, not the robot's offset.
- **The scan plane is pitched** about 0.71° nose-up: 0.7102–0.7169° over all 14,063
  admitted scans; parked robots sit at 0.7137°. The likely cause is the rear caster
  (Claude Code's inference from the model, which gives 0.707°). The plane passes over
  a parked robot in some scans and cuts its housing or its body in others. Every part
  it can cut lies within about 0.108 m of the spawn (rendered model; 0.127 m by the
  collision box). The measure does not depend on this: a blocked beam is defined by
  where its return lands.
- **O10:** the offline ray set turns `-inf` readings into 7.9 m of free space, while the
  live relay leaves them for Karto to ignore. This adds 313 rays to k3_cut1200 (G0's
  worst map, 4 cells) and one ray to each of the four k0 maps.

### 4.2 G11 check 2, checked before commit

S3's driver is not in the tree. From the stored S3 file only, each `G0[<stem>]` holds
`agreement`, `disagreeing_cells`, `holds` and `n_observed_cells`. In 20 of 20,
`agreement` equals `(n_observed_cells − disagreeing_cells) / n_observed_cells` exactly,
which is S13's reading. S13 does not compare `holds`: S3's means at most 5 cells,
`gate`'s means agreement of at least 0.995.

### 4.3 S13 (`s13_shadows_20261014T095408Z.json`, sha `cd1ada60…`, at `58b654b`)

Run time 471.7 s. Run log `s13_run_20261014T094616Z.log` (`5178c8e0…`).

- **G11: HOLD,** all five checks: 160 of 160 values; 20 of 20 maps; 80 of 80 parked
  disks; X7 holds no hit return in 20 of 20 maps; S6's output byte-identical.
- **P18: HOLD, 20 of 20,** with 18 required. The largest Δ is 0.00231.
- **P19: HOLD, 20 of 20,** with 18 required. Δ is 0 in all 20 pairs.
- **Reported, not judged:** the largest Δ over the 20 pairs at each pair of cuts. At
  every 0.0000 entry, Δ is exactly 0 in all 20 pairs.

  | cut_A \ cut_B | 60 | 120 | 240 | 1200 |
  |---|---|---|---|---|
  | 60 | 0.0121 | 0.0040 | 0.0040 | 0.0040 |
  | 120 | 0.0052 | 0.0000 | 0.0000 | 0.0000 |
  | 240 | 0.0036 | 0.0000 | 0.0000 | 0.0000 |
  | 1200 | 0.0023 | 0.0000 | 0.0000 | 0.0000 |

- The largest Δ anywhere is 0.0121, with both maps at cut60 (A = k1, B = k0).
- **Readings at or below `range_min`:** 18 in each of k0's four maps, all in scans
  within 0.3 m of a parked spawn; 374 in k3_cut1200 and 1 in k4_cut1200, none in such
  a scan; none elsewhere.
- **Post hoc, not registered:** only 28 of the 320 rows have any shadowed element, and
  all 28 have one of the two maps at cut60.

### 4.4 What it means (v0.22)

- With A at cut1200, shadows account for less than 0.25 points of the first point of
  P3's series, against a typical first step of 22 points. At full coverage they
  account for none.
- So neither P3's series nor P6's residual is a product of parked robots blocking B's
  view, as far as beams with a hit can show.
- The only readings that can have met a parked robot without a hit are the 18 in k0's
  run.

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
| G11: S13 reproduces S6 and S3; detector and decoy | S13 `58b654b` | HOLD |
| X7: decoy disk at the spawns' centroid | S13 | no hit return 20/20, as intended |
| P18: A at cut1200, B at cut60, Δ ≤ 0.05 in ≥ 18 of 20 | S13 | HOLD 20/20 |
| P19: both at cut1200, Δ ≤ 0.01 in ≥ 18 of 20 | S13 | HOLD 20/20 |

---

## 6. What was learned

- **The thresholds were right; the reasons were not.** P18's and P19's sentences were
  set in draft 1 and never moved. The audit and four checks found false statements,
  none in a threshold, most in the reasons: a robot's width, its distance from B, a
  level scan plane, a pitch "at every scan", what a beam with no hit means. The rest
  were in definitions and records (a corner rule, an off-grid count, roundings).
- **The measure was safe from those errors.** A blocked beam is defined by where its
  return lands, so the robot's shape and the plane's pitch never entered Δ. They only
  entered the arguments for why Δ would be small.
- **An audit can be wrong too.** The audit assumed a level plane (item F) and miscounted
  rows where a walk could leave A's grid (item I.12). Each later check caught the
  audit's own errors as well as the draft's.
- **Rounding is a claim.** "1.72 m" was 1.714 m in two pairs; "0.71° at every scan"
  came from a median, and 130 scans round to 0.72°.
- **A gate check that rests on code not in the tree must be tested first.** S3's
  driver was never committed. Reading the stored S3 file before the run showed that
  S13 counts disagreeing cells the same way.
- **The margin carried the information again.** P18's bound was 5 %; the largest value
  was 0.23 %. With both maps fully explored, there was no shadow at all.

---

## 7. Corrections and records

- **Chat's errors in the v0.21 drafts:**
  - draft 1: a parked robot "about 0.2 m wide", and an argument that B stays 1–1.7 m
    from it (B's sensor passes 0.19–0.42 m from a parked spawn in every run); r's
    reason cited G1's 0.066 m as the robot's offset;
  - draft 2, among others: a collision width of 0.078 m, which appears nowhere; shadow
    widths built on it; "B sees the same walls just before and after" stated as fact;
  - draft 3: "a parked robot that close returns no hit";
  - draft 4: "only a beam nearer than `range_min` returns no hit" (following Claude
    Code's own loose wording);
  - drafts 1–4: "B starts inside the parked cluster"; B starts on the same ring;
  - draft 5: "0.71° at every admitted scan".

  Draft 6's §8 and §9 list them; all six drafts are pinned in
  `experiments/logs/splice/`.
- **Claude Code's errors,** found by its own later checks: the level scan plane (audit
  item F); the off-grid count (audit item I.12); 1.72 m in its audit summary; 0.71°
  given from a median in its draft 4 check.
- **v0.22's draft 1** had one false bullet: it left out "at or below `range_min`".
  Claude Code found it before the splice.
- **Trailers.** The v0.21, S13 and v0.22 commits end with a `Co-Authored-By` trailer;
  the v0.19 and v0.20 spec commits do not.
- **Three points in the 13 Oct handoff, raised at the start of this session and not yet
  checked:** §4.4's "b sat at an end of its run" (second and second-to-last are not
  ends); §4.4's largest dot at k′ (0.0197, a tilt near 1.13°) against §6's two tilts
  above 1° and §7's +1.0104° at k′; and CI for `ed2a618`, never recorded.

---

## 8. Working protocol (changes this session)

- **Commit and run blocks are script files,** run with `bash`, under `set -euo
  pipefail` with a `stop` function. A commit script pins every reviewed file by sha256,
  checks HEAD, origin, the tree and the diff's numstat, checks that new text is present
  and old text absent, stages by name, gates the staged set, and ends with `MSG_SAME`
  and `PUSHED_SAME`.
- **A run script** refuses unless HEAD is the pushed stage commit on its expected
  parent and the tree is clean. It captures the full output with `tee` and names what
  the exit code means.
- **Prediction reasons are short and labelled as expectations.** A draft states as fact
  only what an audit, a check or a stored file shows.
- **After a draft passes a check,** the next check diffs against it and re-checks only
  the changed lines.
- **A gate check that rests on code not in the tree** is tested against its stored file
  before the stage runs.
- **A stage reads every number it applies from the registered text** and refuses on a
  mismatch (new in S13).
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **O4.** Closed in v0.22.
- **O2, empty histograms.** Treatment still undecided.
- **O5, the deep tail.** Unexplained.
- **O9, risers.** (1) Whether the wall measure should count perpendicular elements.
  (2) Whether segment ends, for openings and corners, are read only from elements in the
  direction of the segment's normal axis. Proposed: yes, to be decided in the schema
  spec.
- **O10, `-inf` rays.** Whether they are linked to G0's differences is untested. Not
  checked: whether k3_cut1200's 374 readings at or below `range_min` include its 313
  `-inf` readings.
- **w_null and E7 at mixed cuts** are still unseen, so a prediction about them can
  still be registered.
- **The fixed-A weight series** are seen. How Article 3 shows and reads them is open.
- **The fail paths of S9–S11.** Fix them in their own commit before any of those stages
  is rerun.
- **The pitched scan plane** belongs in Article 3's description of the sensor.
- **The three points from the 13 Oct handoff** (§7).
- **Carried over:**
  - extent boxes are not nested across cuts;
  - B3 was not judged on k4_cut1200_gated_extfix (check the stopper's grace before the
    next batch);
  - the online explorer and mapper defects must be decided before B4;
  - check that Claude Code's auto mode keeps git commit, git push and simulation
    launches out of its allow list.
- **Scratch to remove:** `experiments/logs/splice/spec_v0_18_preview.diff`; the old
  handoff draft in `~/Downloads/`; this session's drafts, prompts and scripts in
  `~/Downloads/` (the pinned copies are in `experiments/logs/splice/`).

---

## 10. Next

1. **Check CI** for `0c4aff7` and this handoff's commit.
2. **The schema spec:** openings, corners and the back-to-back face pairing, starting
   with O9 (2).
3. **Then B3,** in the planned order of Article 3.

---

## 11. The operating lesson

P18 and P19 had the right bounds from their first draft, and the bounds were never the
problem. Six drafts went into the text around them, and most of what the checks found
were reasons that claimed more than had been measured. The measure itself was immune,
because it was defined by where returns land, not by what the robots look like. A
reason is a claim like any other. When it cannot be checked against what has been measured, call it an
expectation and keep it short. The result, 0.23 % against a 5 % bound, needed no
argument at all.
