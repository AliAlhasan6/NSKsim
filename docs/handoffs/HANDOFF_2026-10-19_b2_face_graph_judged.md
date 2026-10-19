# HANDOFF — B3 inventory, the edges spec, and the face graph built and judged (18 – 19 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-18_b2_schema_run1.md`.

---

## 0. In one paragraph

The previous handoff named B3 as next. A read-only inventory showed it could not start:
no code turned the schema's elements into a graph, and the 20 maps come from one world.
So this session finished B2 instead. It wrote and registered an edges spec
(`SPEC_b2_edges.md` v0.2, `358650a`) whose truth answer, F\*, was predicted by hand and
then matched exactly. It built the face graph in two commits (`9cd7839`, `e2ae698`) and
ran the stage once on the 20 `gated_extfix` maps. Gate G-E1 held, and all four
registered predictions hold with no margin used. On every cut1200 map: one component,
12 of 12 F\* pairs, nothing wrong, nothing unmatched. Twelve old handoffs and one
revision were also brought into the repo. **The B3 corpus problem is now sharp:** the
five full maps give the same graph.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
( source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim || exit 1
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
sha256sum experiments/logs/edges/judge_e2ae698_20261019T084358Z/edges_judge_20261019T084358Z.json
#   expect 348a5a4bba1f7a035e47e59d47f2ab2fd38ddb7a05f37d103fa32220d89747e7
gh run list --branch main --limit 7 )    # CI for the commits in §2
```

---

## 2. Commits

| Commit | What | CI |
|---|---|---|
| `20343e5` | docs/handoffs: twelve handoffs from ~/Downloads, 17 Jul to 4 Oct | unchecked |
| `cfde00b` | docs/handoffs: the 07-10 handoff, revised (CI checked, where O3 starts) | unchecked |
| `358650a` | docs/specs: SPEC_b2_edges v0.2, registered | unchecked |
| `9cd7839` | edges_walls: the face graph of one map, and its per-map file | unchecked |
| `e2ae698` | edges_judge: the stage, gate G-E1 and P-E1 to P-E4 | unchecked |
| this commit | docs/handoffs: this handoff | unchecked |

- **`9cd7839`:** 39 tests. Listed suite, venv, ROS-sourced: 1189 passed, 0 skipped.
- **`e2ae698`:** 61 tests. Listed suite, same shell: 1250 passed, 0 skipped, in 41 s.
- CI for `cd48836` (the previous handoff) was green, checked 18 Oct.

---

## 3. The run

**Location.** `experiments/logs/edges/judge_e2ae698_20261019T084358Z/` (gitignored).
It ran in 0.78 s. It read the 20 schema files of schema run 1 (checked against
`84ffdb5a…`) and the 20 graph_walls files, and wrote 20 per-map files and one results
file.

| File | sha256 |
|---|---|
| edges_judge_20261019T084358Z.json | `348a5a4b…` |

**Gate G-E1: HOLD.**
- **Item 2 (truth under v0.2):** 12 pairs, all equal to F\* in whole samples; the four
  pairs beside a slab end each 5 samples short of v0.1, as §5.1 said. 0 samples through
  a slab, 20 blocked by caps, 0 no hit, 0 ties. Components 1 with corners, 2 without.
- **Item 3 (decoy):** 50 samples, all blocked, 0 facing edges; every other edge and
  tally unchanged.
- **Item 1:** corner and slab edges equal to the schema file on 20 of 20 maps.
- Before the gate, the stage reproduced the v0.1 truth file to its printed decimals.

| | Result | Bound |
|---|---|---|
| P-E1 | 1 component on 5 of 5 cut1200 maps | ≥ 4 of 5 |
| P-E2 | 12 of 12 F\* pairs on 5 of 5 maps | ≥ 11 of 12, on ≥ 4 of 5 |
| P-E3 | 0 maps with a wrong facing edge | ≤ 1 |
| P-E4 | 0 maps with an unmatched facing edge | ≤ 1 |

**Reported, not judged** (totals over 5 maps per cut):

| Cut | Nodes | Excluded | Corner | Slab | Caps | Facing |
|---|---|---|---|---|---|---|
| 60 | 27 | 1 | 0 | 3 | 4 | 16 |
| 120 | 36 | 0 | 4 | 6 | 11 | 26 |
| 240 | 51 | 1 | 9 | 14 | 24 | 44 |
| 1200 | 60 | 3 | 20 | 20 | 40 | 60 |

At cut1200 the 60 facing edges are exactly 12 per map: no map formed a pair outside F\*.
Widths, gaps, no-hit counts and earlier-cut components are in the results file and
have not been read yet.

---

## 4. Decisions this session

- **Sequential, not parallel.** The explorer and mapper defect work waits until B3 is
  done.
- **B3 waits for B2.** B2 was not finished: no graph existed. The edges spec and
  builder came first; the corpus decision comes after.
- **The edges spec (D-E1 to D-E6).** Nodes are faces with axis ends. Inside corners and
  slab pairs are edges. A new `facing` edge gives the plan's "relative bearing" a
  definition. Connectivity and containment are out of v1. The knowledge/claim switch
  moves to B4. All geometry stays in each robot's own map frame.
- **The build plan's 34 proposals** were accepted as proposed, including Q1 (a face
  never occludes its own rays), Q7 (a strict join), Q13 (a decoy outside the arena) and
  Q20 (extra record keys).
- **E24 room** was corrected after its first run (§6).
- **Handoffs import.** Twelve old handoffs went into the repo unchanged, and the 7 Oct
  handoff was replaced by its uncommitted revision. Twenty more stay out: three are
  personal throughout; seventeen carry personal or third-party passages. They remain in
  ~/Downloads and the HDD backup.

**The planned order, restated here** because the September handoffs that hold it stay
out of the repo:
1. **B2** turns each robot's map into a structured description: walls (wall predicate),
   their divergence between robots, the schema (corners, slabs, slab ends, openings as
   a control), and now the face graph. All of it is done for the 20 maps.
2. **B3** builds a corpus of environment graphs and retrains the embedder (GATv2) on
   it, so its embeddings are in distribution for maps rather than for FB15k-237.
3. **B4** replaces the FB15k-237 `dataset_indices` feed with live environment graphs,
   runs the sharing loop on what the robots perceive, and measures convergence. It
   needs new sim runs, so the explorer and mapper defects are fixed first.
4. All three stay in Article 3.

**The merger, restated.** The September handoffs left the trained merger out of NSKsim:
its encoder gives one constant output for every graph, and fifteen repair
configurations failed. NSKsim's sharing rule is a fixed 0.7 / 0.3 average, and the
merger is loaded but never called. AgentManager still requires `merger_state` in the
checkpoint (B3 inventory §5). This is recorded here so the repo holds it.

---

## 5. Files in splice this session

| File | sha256 |
|---|---|
| b3_inventory.md | `af8a2c05…` |
| handoffs_import.md | `fcc07dca…` |
| handoff_1007_revision.md | `740b0e50…` |
| edge_spec_inventory.md | `d187ece2…` |
| edges_truth_fstar.py / _out.txt | `f1db4cec…` / `6dbc3354…` |
| edges_truth_fstar_report.md | `7452ffa3…` |
| spec_edges_v0_2_register.md | `d4bc7248…` |
| edges_build_plan.md (rulings in header; E24 line) | `2015120f…` |
| edges_walls_build_report.md (E24 line) | `8ada11dc…` |
| e24_room_diag.py / .md | `faf51da2…` / `376cb1bc…` |
| e24_fix_note.md | `05f79dc0…` |
| edges_judge_build_report.md | `c84c0be9…` |

---

## 6. What was learned

- **Predict before running, in full.** The truth script's output was written down by
  hand before Ali ran it, and every number matched. That made the run a test rather than
  a source of numbers, and it is why F\* could be pinned with confidence.
- **A literal spec can sink its own predictions.** Read literally, §3.3 let a face
  block its own rays: on k0 at least five faces would have lost every sample. The build
  plan caught it (Q1) before any code ran.
- **Risers move cells between faces.** At an inside corner, a slightly tilted face takes
  the perpendicular wall's end cells (W4, W5), so that wall gets shorter. The hand
  derivation behind E24 missed this; the first run caught it. A failing test was
  diagnosed, not loosened, and the counts pinned afterwards are labelled as found.
- **The fresh-session guard works.** It stopped two prompts sent to the wrong session.
  New-session prompts now carry a banner inside the copy block.
- **The B3 corpus problem is structural, not a detail.** One world gives one graph.

---

## 7. Corrections and records

- **Claude Code's slips:**
  - the truth run block sourced `~/Desktop/NSKsim/install/setup.bash`; chat corrected
    it to `ros2_ws/install/` before the run;
  - the E24 hand derivation (riser transfer missed; it also reasoned about a 3.8 m face
    where the failing face was 2.8 m);
  - in the commit-2 session, a read-only `python3 -I -c` printed a JSON's keys during a
    turn asked to run nothing; it wrote nothing and is recorded in the build report;
  - one session counted the guard's catches as three; it was one at that point.
- **Chat's slips:** none recorded beyond asking for "four handoffs" when the import
  found more.
- **The plan's E24 row and the commit-1 build report §6** are superseded by
  `e24_room_diag.md`, each with a dated line.
- **Untested until the run, now exercised once:** the F\* file parse and check,
  D-E1's 5-of-179 corpus check, and `run_edges` past its refusals.

---

## 8. Open items

- **The B3 corpus.** All five cut1200 maps give the same graph, against FB15k's 4,530.
  Options: new maze worlds with new sim runs, or synthetic partial maps from world files.
  To decide next.
- **The embedder's input** (B3 inventory §3): `input_dim` 2 with an entity-id feature
  that maps lack; 237 relation ids against 3 edge types here; the Stage 2 loader
  hard-codes FB15k-237; NSKsim loads `joint_best.pt` and needs `merger_state`.
- **Freeze B2 formally.** `SPEC_b2_schema.md` and `SPEC_b2_divergence_walls.md` still
  call themselves drafts. The plan said to freeze B2 before B3.
- **Edges spec:** E-O1 (absolute position in node features), E-O2 (feature vector and
  relation ids), E-O3 (unclassified cells do not occlude), E-O4 (w_min), E-O5 (misaligned
  and unobserved slab ends), E-O6 (a slab end without two points leaves its slab open).
- **E-O7 (new):** riser transfer at inside corners shortens boundary widths and adds
  no-hit counts. Reported, never gated.
- **Read the reported numbers:** widths and gaps against truth, no-hit counts, and
  components at cuts 60–240, from `348a5a4b…`.
- **Held handoffs.** The schema entry handoff, which `SPEC_b2_wall_predicate.md:41`
  quotes, stays out of the repo; §4 restates its plan. The 10-06 open item is half
  closed: 10-04 is committed, 09-29 is held.
- **CI** for the six commits in §2.
- **Carried over from 18 Oct:** S-O5, S-O6, O9 (2) to note in the divergence spec, the
  explorer and mapper defects (after B3), O2, O5, O9 (1), O10, and the rest of that
  handoff's §8 list.

---

## 9. Next

1. **Check CI** for the commits in §2.
2. **Start a new chat session** from this handoff.
3. **Decide the B3 corpus source** in chat before any Claude Code work. The deciding
   questions: how many graphs B3 needs, whether synthetic partial maps would be in the
   same distribution as real ones, and what new worlds would cost in sim time on this
   machine.
4. Then a **fresh Claude Code session** for whatever the decision needs first.

---

## 10. The operating lesson

B2's last piece went from a gap found by an inventory to a registered verdict in about a
day: inventory, decisions, spec, truth predicted and matched, plan, two commits, one
run. Twice, a hand derivation was the check that mattered. The first one matched and
made F\* trustworthy. The second one was wrong, and the first run said so plainly.
