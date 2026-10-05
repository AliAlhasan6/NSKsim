# HANDOFF — B2 divergence on walls (4 – 6 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-04_b2_wall_graphs_and_final_map_corpus.md`.

---

## 0. In one paragraph

The divergence measure on walls is specified, built, tested and run. Its spec,
`docs/specs/SPEC_b2_divergence_walls.md`, is at v0.7. Every one of A's 0.1 m wall
face elements is classed by robot B's evidence as corroborated or not corroborated.
A third class, contradicted (B's own rays crossing A's wall without a return), was
built and then removed: its self-test, which runs it on each robot's own rays
against its own walls, failed twice, and the fallback written before the second
run took it out of the measure. Three predictions were registered before any share
was computed, and all three hold on the final corpus: B's evidence about A's final
walls only grows with B's exploration (P3), every unseen pair corroborates at least
90 % of A's walls at full coverage (P6), and every corroborated wall is a real wall
(P4). Same world is argued from corroboration, not from the absence of
contradiction.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only .vscode/, experiments/wall_affinity.py, test_wall_affinity.py
git log --oneline -1 origin/main         # expect this handoff's commit, or 78eb800 if not yet committed
grep -m1 -o "v0\.7" docs/specs/SPEC_b2_divergence_walls.md      # expect v0.7
ls experiments/logs/divergence/s4_predictions_20261005T230035Z.json  # expect the file
```

---

## 2. Commits this session (all pushed)

| Commit | What |
|---|---|
| `710f075` | spec v0.1: the divergence spec with the S1 answers folded in |
| `37d9920` | divergence_walls §3–§6 on synthetic data. **Also** diagnose_karto's variant-aware inputs (`bag_path_for`, `run_name_for`, `scan_inputs`, `admitted_rays`); its message covers only divergence_walls (see §7) |
| `3cf18e2` | empty record commit: 37d9920 also moved rebuild()'s inputs onto variant paths, with the regression evidence |
| `9f48d55` | spec v0.2 and v0.3: T2 conditions 3 and 4, and C0 still failing 16 of 20. Its message has one wrong sentence (see §7) |
| `f2b4406` | spec v0.4: T2 leaves the measure under §7's fallback; T2 kept in code behind a flag, default off |
| `a9b14e5` | spec v0.5: T1's distance test split into across (≤ eps) and along (≤ rho/2) |
| `3dd70f3` | spec v0.6: P3, P4 and P6 registered, before any share was computed |
| `9eb3f17` | a pair may take A and B at different cuts; every row carries cut_A and cut_B |
| `78eb800` | spec v0.7: erratum to P6's quoted seen values; no registered wording changed |

---

## 3. The measure as it stands (spec v0.7)

- **Unit.** One of A's face elements: the oriented 0.1 m boundary between an
  occupied and a free cell, with its normal into free space. Elements are recomputed
  with `graph_walls.graph_and_elements(stem)`, with `g` taken from the JSON's
  `thresholds.g` and the map's sha256 checked against the JSON. Element normals are
  axis-aligned by construction.
- **Common frame.** Spawn registration from `DOT_POSES` at `08617b2`, translation
  only: `x_A = x_B + s_B − s_A`. No truth enters the measure.
- **T1, corroborated.** B has a face element with the same dir, within
  `eps = 0.1707 m` across the face and `rho/2 = 0.05 m` along it (inclusive, 1e-9 m).
- **Not corroborated.** Everything else.
- **T2.** Kept in the code and spec as a record, behind a flag that defaults to off.
  With the flag off, no ray set is built, so a pair needs no bag.
- **Pairs.** Ordered (A, B); A and B may sit at different cuts.
- **Tool and tests.** `experiments/analysis/divergence_walls.py`,
  `ros2_ws/src/nsk_swarm/test/test_divergence_walls.py` (34 tests; full suite
  743 passed, 33 skipped). Results in `experiments/logs/divergence/`, one new dated
  file per run.

---

## 4. Results

### 4.1 Gates (S3, file `s3_gates_20261004T201944Z.json`)

| Check | Result |
|---|---|
| G0: each map rebuilt from its admitted rays matches within 5 cells | 20/20; eighteen at 0, one at 1, worst 4 (k3_cut1200) |
| X6: k1_cut1200 rebuilt from the ungated bag fails G0 | fails, 132 cells (gated bag: 0) |
| G1: parked robots land within 0.15 m of their spawns in A's frame | 80/80, worst 0.066 m (20 distinct measurements; the error does not depend on A) |
| X1: frame sign flipped | 0/80 hold, worst 3.493 m |

### 4.2 Registered predictions (S4 at `9eb3f17`, file `s4_predictions_20261005T230035Z.json`, 0.3 min)

| Prediction | Result |
|---|---|
| P3: with A held at cut1200, the not-corroborated share is non-increasing as B's cut grows, every ordered pair | **HOLD**, 20/20, no rise at any step; typically 0.81 → 0.59 → 0.34 → 0.02 |
| P6: corroborated share ≥ 90 % at cut1200 for the 16 pairs with A ≠ k0 | **HOLD**, 16/16, 0.9591–0.9946. The 4 seen pairs (A = k0): 0.9780–0.9833 |
| P4: ≥ 95 % of corroborated length within eps of a world wall, 80 same-cut rows | **HOLD**, 80/80, every row 1.0000; furthest element 0.1262 m |

The same-cut series (A and B at the same cut) is reported, not tested. It dips at
the middle cuts (for example A = k0, B = k4: 0.4422 at cut120, 0.5094 at cut240,
0.9788 at cut1200), because A's wall set grows under the measure. It is not a
convergence curve, which is why P3 holds A fixed.

---

## 5. The contradiction test: what was tried, and why it left

The accepted shape on 4 Oct had three classes. The third, contradicted, required B's
own rays to cross A's wall without a return. Its floor was measured by **C0**: the
test alone, run on each robot's own rays against its own walls, with a stop rule of
1 % of A's face length.

1. **v0.1** (one ray; run-on of eps measured along the ray; rays at least 30° off
   the face line). C0 failed on 20/20, 4.5–68.6 %, and 2.5–39.7 % even at 45°.
   Contradicted elements sat a median 3.1 m from face ends, so not an edge effect.
2. **Depth diagnostic** (k0_cut60, predictions printed first). A: every element sits
   exactly on the lattice (286/286), so the geometry is right. B: crossing depths
   behind the face line have medians of about one cell but a p99 of 3.9–4.9 m in
   three of four directions. C: 25.94 % of contradicting crossings were relay-filled
   no-return beams. D: every crossing deeper than 0.12 m sat on a cell with a
   hit/pass ratio under 0.5 (median 0.25).
3. **v0.2**: condition 3 measured as depth across the wall, not distance along the
   ray. Post hoc, for a physical reason: A's own wall surface can lie a cell or more
   behind its face line.
4. **Deep-tail chain**, with a pre-stated stop. S (scans taken while turning): the
   deep rate rose 0.99× and 2.36× from the lowest to the highest quartile of yaw
   rate, against ≥ 5× required. J (rays slipping through joints between wall
   boxes): 10.7 % and 26.0 % of deep crossings near a box end, against ≥ 80 %. Both
   failed; the chain stopped. The deep tail stays **unexplained (O5)**: crossings
   are either within 0.2 m of the face or metres beyond it, almost nothing between,
   on 3–10 % of crossings.
5. **v0.3**: condition 4, weight of evidence (at least 2 rays run on, and they are
   a strict majority of the rays crossing). Post hoc, threshold on principle. C0:
   4/20 within 1 %, 0.62–3.63 %. X5 (the contradicted share falls as the angle
   threshold rises) was violated on 8/20 maps, because a majority ratio is not
   monotone in that threshold.
6. **v0.4**: §7's fallback, written before the v0.3 run, applied. T2 left the
   measure.

---

## 6. What was learned

- **A check inside a pasted block cannot stop the next line.** On 4 Oct, a commit
  block ran before its message file existed: the first commit failed, the second
  `git add` stacked onto the first, and `37d9920` took four files under a message
  that describes two. Commit blocks are now chained with `&&`, gated on the staged
  file count, and end with a push and an `ls-remote` check.
- **A contradiction test needs a self-test.** C0 caught two design flaws on first
  contact with real data, before any cross-robot number existed.
- **In Karto, occupancy is a ratio.** Real wall cells take many passes from rays that
  cross the front part of the cell and hit the wall just behind it. Passing the cell
  is not passing the wall.
- **A regression baseline must come from HEAD's code**, not from a result file of
  unknown vintage. `diagnose_karto.json` predated the 3 Oct cell-centre fix, so the
  refactor check found 195 differences against it; HEAD's code against the edited
  tree gave 0 over 1,292 fields.
- **A diagnostic that computes the measure is seeing the measure.** `t1_across`
  produced the v0.5 corroborated share for the four A = k0 pairs. That is why P6
  excludes them.
- **A number quoted in registered text must come from committed, tested code.**
  `t1_across` came from a scratch script with a wrong denominator, and P6 quotes its
  numbers (§7).
- **Registration is the commit.** After it, not even typography changes a registered
  wording; a backtick edit made after `3dd70f3` was discarded. Corrections are
  appended as errata beside the prediction.

---

## 7. Corrections and records

- **cdba182 was Ali's commit.** The previous handoff's §1 suspected Claude Code; its
  session had no git calls, and the commit landed 29 minutes after the hand-over.
- **Claude Code's September commits.** Claude Code ran 32 commits between 14 and
  21 Sep 2026, under the task-scoped instructions then in force. It pushed three
  itself (`19a68c4`, `a32bc6c`, `b449bb2`, each confirmed as "update by push" in the
  reflog) and amended one (`b7bb299` into `6724221`, identical tree, trailers only).
  None since 22 Sep. The rule is now blanket and in Claude Code's own memory.
- **9f48d55's message** says Karto's occupied cell sits a cell or more behind the
  face line. The face line is the occupied cell's front edge. It is the wall surface
  that can lie a cell or more behind the face line, inside the occupied cell or
  beyond it (depth diagnostic, `depth_diagnostic_20261004T*.json`). Spec v0.2's
  condition-3 text states this correctly. The commit is public and was not amended.
- **37d9920's message** covers only divergence_walls; `3cf18e2` records the
  diagnose_karto half.
- **diagnose_karto.json is stale.** It predates `f14747f`. The 84 % and 69 % dwell
  figures in the previous handoff's §4.3 were computed before the cell-centre fix;
  do not cite them without regenerating. Copies made this session sit beside it:
  `diagnose_karto_20261004_before_scan_inputs.json`, `..._before_s3.json`.
- **t1_across's numbers are wrong.** Its scratch script divided by all 1,377 of A's
  elements instead of the 1,320 assigned to faces, and 57 of the 77 "none" entries
  were phantom array slots. The seen shares were 97.80–98.33 %, not 93.8–94.3 %.
  Spec v0.7 carries the erratum beside P6. The across-tolerance decision is
  unaffected: with the right denominator, the 0.25–0.5 m band is at most 0.38 %.

---

## 8. Working protocol (changes this session)

- **Commit blocks come only from chat**, after review. Claude Code writes the message
  file under `experiments/logs/msgs/`, lists the files to stage, and stops
  (`memory/ali-makes-every-commit.md`, updated 6 Oct).
- **Chat sends a commit block only after Claude Code confirms** the step it depends on.
- **Every commit block** is chained with `&&`, gates on the staged file count, and ends
  with `git push` and an `ls-remote` comparison against HEAD.
- **Before registered numbers are computed**, the code that computes them is
  committed and pushed, and its hash is recorded in the result file.
- Everything else in the previous handoff's §7 is unchanged: 120–130-word replies,
  "(for Claude Code)" headings, predictions printed before numbers and never reworded,
  a stopping rule for each diagnosis chain, nothing edited during a run.

---

## 9. Open items carried forward

- **O1, viewpoint weighting.** How `bearing_hist` enters (schema handoff §3.3:
  corroboration is weighted by viewpoint diversity, never by observer count).
- **O2, faces with empty bearing histograms.** Recount on the extfix corpus.
- **O3, a symmetric summary** of A → B and B → A, and the series over cuts.
- **O4, parked-robot shadows.** Whether to report the not-corroborated length that
  parked robots shadow.
- **O5, the deep tail.** Unexplained. The turning test (S) bears on, but does not
  settle, the spin-smear line in the wall-predicate spec §12.
- An unreproduced `1 failed, 850 passed` from one whole-directory pytest run, before
  the collectable-file list was regenerated; not attributed.
- GitHub Actions results for every push since `0dee108` are still unchecked.
- `wall_affinity.py` and its test: commit or delete.
- B3 not judged on k4_cut1200_gated_extfix (relay SIGKILLed); check the stopper's
  grace before the next batch.
- Online explorer and mapper defects (half-cell reading, extent truncation): decide
  before B4.
- `HANDOFF_phaseB2_schema_entry.md` and `HANDOFF_2026-09-29_gates_and_grid_divergence.md`
  into `docs/handoffs/`, if not already done.
- Superseded message files in `experiments/logs/msgs/` are scratch and can go.

---

## 10. Next

B2's wall divergence now has its dependent variable and three held predictions.
The remaining B2 work is O1 and O3, then the rest of the schema (openings and
corners), then B3, in the planned order of Article 3.

---

## 11. The operating lesson

The contradiction test was the most natural part of the design, and it did not
survive contact with the robots' own data. What kept that from becoming a false
result was a control written before the first number: run the test on each robot's
own rays against its own walls, and stop if it fires. It fired, the diagnosis chains
had stopping rules, and the fallback was on the page before the last attempt. The
measure that remains is smaller, and every number in it was registered before it
was seen.
