# HANDOFF — B3 corpus decided, the edges numbers read, and B2 frozen (19 – 20 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-19_b2_face_graph_judged.md`.

---

## 0. In one paragraph

The previous handoff left the B3 corpus source to decide before any Claude Code work.
Ali decided it: a large corpus, trained on synthetic maps and tested on real ones, with
as many real recordings as needed. One world gives one graph at full coverage, so the
corpus needs a **world generator**, and real runs are recorded in a sample of the
generated worlds as the test set. Two steps came first, and both are done.

- **The edges run's reported numbers were read** by a stdlib-only script with a
  1,541-check self-check (0 mismatched). The reading is **post hoc, not registered**.
  Gaps are short of truth by a steady amount at every cut. Widths are within 0.2 m at
  cut1200 except the outer pairs. Early cuts are dominated by partial visibility.
- **B2 is frozen** (`a4ad8e1`). No spec text was edited, because three of the specs
  are read by their own stages. A separate record, `docs/specs/B2_FREEZE.md`, holds
  every verdict, 33 errata, and a disposition for each of the 28 open items.

Next is the B3 spec, in a new session.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
( source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim || exit 1
git fetch -q origin
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
sha256sum docs/specs/B2_FREEZE.md
#   expect cb3064ced80e78f2c1ca825955f7af1fdf3849080dd3a0061933e442faebe1b8
sha256sum experiments/logs/edges/judge_e2ae698_20261019T084358Z/edges_reported_read_out.txt
#   expect 796b98d3ecd2ca4d8d1b5a7f7336d0f4df6d128348ee46c87c993c490eb79640
gh run list --branch main --limit 3 )    # CI for §2
```

---

## 2. Commits

| Commit | What | CI |
|---|---|---|
| `a4ad8e1` | docs/specs: B2 freeze record (specs unedited; verdicts, errata, dispositions) | green |
| this commit | docs/handoffs: this handoff | unchecked |

- **`a4ad8e1`** was verified read-only after its output was lost (§7): parent
  `8de8dc5`, numstat `662 0 docs/specs/B2_FREEZE.md`, committed file `cb3064ce…`,
  MSG_SAME, `git ls-remote` equal to HEAD, CI green.
- **CI for the previous handoff's six commits** is green: `20343e5`, `cfde00b`,
  `358650a`, `9cd7839` and `e2ae698` (checked 19 Oct), and `8de8dc5` (checked 20 Oct).

---

## 3. Results

### 3.1 The edges run's reported numbers, read back (post hoc, not registered)

- **Script:** `experiments/logs/splice/edges_reported_read.py` (`2c5fabb0…`), standard
  library only, run with the project venv (Python 3.12.3). It refuses unless the
  results file is `348a5a4b…` and each per-map file matches its `per_map_files` hash.
- **Output:** `edges_reported_read_out.txt` in the run folder (`796b98d3…`). Exit 0.
- **Self-check:** 1,541 checks, 0 mismatched. It recounted the 19 Oct handoff's §3
  totals from the per-map files and recounted P-E1 to P-E4: all match.
- **Truth used:** v0.2, as the stage stored the differences. Four pairs (E–maze_v1+x,
  N–maze_h2+y, S–maze_h1-y, W–maze_v2-x) are 0.25 m narrower under v0.2 than in the
  F\* table's v0.1.

**Gaps (map minus truth).** Every facing gap is short of truth, and the shortfall is
steady across cuts. Median per cut: −0.098 (60), −0.141 (120), −0.142 (240), −0.142
(1200); largest |diff| 0.206. At cut1200, by pair type:

| Pair type | Range at cut1200 |
|---|---|
| outer–outer (E–W, N–S) | −0.155 to −0.206 |
| outer–maze | −0.080 to −0.157 |
| maze–maze | −0.073 to −0.120 |

**A hypothesis, untested:** outer faces sit about 0.10 m, and maze faces about 0.05 m,
inside their true planes (0.20, 0.15 and 0.10 m summed by pair type). The per-map node
positions can confirm or refute it.

**Widths (map minus truth) at cut1200:**
- maze–maze pairs: +0.100 on all 10;
- the four pairs beside a slab end: −0.200, except k3's E–maze_v1+x at −0.100;
- the other four outer–maze pairs: 0.000;
- outer pairs: E–W −0.70 to −1.05, N–S −0.65 to −1.05. This is E-O7, riser transfer at
  inside corners.

Median per cut: +0.100 (60), 0.000 (120), −0.025 (240), −0.050 (1200).

**Early cuts.** Width errors reach 5.6 m (k1, cut240, N–maze_h1+y) where faces were
only partly seen.

**No-hit samples.** 709, 945, 742 and 26 at cuts 60, 120, 240 and 1200. At cut1200 all
26 fall on 9 faces that are an end of a corner edge, which fits E-O7. The E-O7 flag only
marks corner adjacency; where along a face the no-hits fall is not stored. So at cuts
120 and 240 it cannot separate riser transfer from partners not yet seen.

**Components** (truth: 1 with corner edges, 2 without, 2 facing):
- cut60: 2 on four maps, 3 on k2 (no corner edges exist yet);
- cut120: k0 2; k1–k4 1;
- cut240: 1, except k1 at 2 (3 without corners, 4 facing), where maze_h1+y appears as
  two faces (ids 9 and 10).

**What it means for B3.** The large errors come from partial visibility: truncated
faces, missing partners, a split face. Per-face noise is centimetres. So the
generator's hard part is simulating what a robot sees along its path, not adding noise.

### 3.2 The B2 freeze record (`a4ad8e1`)

`docs/specs/B2_FREEZE.md`, 662 lines, `cb3064ce…`. Written by Claude Code from Ali's
rulings and reviewed in chat. Its input was the read-only inventory
`b2_freeze_inventory.md` (`1162c336…`).

- **Frozen at `8de8dc5`.** No spec text is edited. What pins each spec:
  - `schema_judge` compares §1–§10 of the schema spec byte for byte;
  - `edges_judge` compares §1–§5 of the edges spec and leaves §6 out;
  - S13 reads the divergence spec at HEAD;
  - no pinning file was found for the wall-predicate, steady-gate and truth-pose-replay
    specs (INFERRED).
- **Six specs** listed with their sha256 and last commit. `SPEC_truth_pose_replay.md`
  is included as an input spec. Its last commit is `03d666c`, from `git log`; a guess
  from reflog subjects had said `00e76e5`.
- **Verdict register:** 98 rows. 64 hold or pass, 15 fail (V5's fail is INFERRED: the
  14 Sep handoff records its wiring, not an exit code on the bags), 6 void, 3 controls
  that behaved as intended, 8 with no recorded verdict, 2 reported only. It includes
  the verdicts the specs never held: S3/S4, G2, G-S1, P-S1 to P-S8, G-E1, P-E1 to
  P-E4, V1 to V5 and S0 to S3. The S4 file was hashed at the freeze (`4f173f4c…`).
- **Errata:** 33 stale statements, each with what is true now and its source. Among
  them: both draft status lines, O9 (2) still open in D, E-O7 missing from E, E §3.3
  against ruling Q1, the steady gate's missing V5 and interval mask, drifted code line
  citations, and the wall predicate's old setup path. D11 (deep tail 4–10 % against
  3–10 %) stays unresolved.
- **Change log:** any later change to a B2 spec is a new spec version, logged in the
  record with a dated line.

---

## 4. Decisions (Ali, 19 – 20 Oct 2026)

**B3 corpus.**
- A large number of graphs, aiming for accuracy.
- Trained on synthetic maps, tested on real ones.
- Ali will record as many real runs as needed.
- Consequence: a world generator. Synthetic maps come from many generated worlds; real
  runs are recorded in a sample of them as the test set.

**Order.**
1. Read the edges run's reported numbers. Done.
2. Freeze B2. Done.
3. Write the B3 spec: the generator, the synthetic maps, the test-set size, and a gate
   that synthetic copies of our 20 maps reproduce their real graphs.
4. Fix the explorer and mapper defects, then record.

Step 4 moves the defect work ahead of B3's test recordings; the 19 Oct handoff had it
after B3. Work stays sequential, one problem at a time.

**The freeze.**
- No spec is edited, for any of the six; one record file instead.
- Dispositions:
  - **closed:** O9 (2) by D-S1; S-O2, reported as the spec says; O9 (1), not changed,
    since that would change a registered measure;
  - **moved to B3:** E-O1, E-O2, E-O5, O2, and S-O5, whose "in view" rule the
    generator's visibility model supplies;
  - **frozen as registered, Article 3 limitations:** E-O3, E-O4, E-O6, E-O7, S-O6, O5,
    O10, and the wall predicate's spin smear;
  - **void:** the wall predicate's T1–T3, whose corpus was superseded.
- **w_null and E7 at mixed cuts are dropped on the record:** never computed, never
  registered. Half the comparison (w) was already seen at mixed cuts, and early maps
  have 4–8 segments. Listed as a candidate prediction for B4, on fresh data.
- **The Article 3 rule is narrowed:** "The designed run is owed for any claim that
  compares odometry against ground truth. B2's maps were built on known poses, so B2's
  truth validations (P4, A8–A10, the schema matching, the edges judging) do not need
  it."
- **Destinations for carried items:** see §9.

**Outside this thread.** The Izvestia LETI article is in review (20 Oct). It needs no
slot before B3 for now.

---

## 5. Files in splice this session

| File | sha256 |
|---|---|
| edges_reported_read.py | `2c5fabb0…` |
| b2_freeze_inventory.md | `1162c336…` |
| B2_FREEZE_draft1.md | `051e34e9…` |
| B2_FREEZE_draft2.md | `f692585c…` |

Also: `edges_reported_read_out.txt` in the edges run folder (`796b98d3…`), and the
commit message `experiments/logs/msgs/b2_freeze.msg` (`59e7c8ed…`).

---

## 6. What was learned

- **Find what reads a spec before you edit it.** The freeze looked like two status
  lines. The inventory showed three specs are read by their own stages, so editing them
  would have blocked reruns. A record beside them keeps both reproducibility and
  correction.
- **Commit subjects are not a file history.** The truth-pose-replay spec's last commit
  was guessed from reflog subjects and was wrong. `git log -- <file>` settled it.
- **A verdict is what was stored, not how it was wired.** V5's handoff records its exit
  code wiring, not an exit on the bags, so its fail stays INFERRED.
- **An unseen quantity expires at a freeze.** w_null and E7 were registrable for a week.
  Freezing forced the choice, and it was made on the record.
- **Partial visibility, not noise, is the gap between real and ideal maps.** That shapes
  what the B3 generator must simulate.
- **A pasted script runs in your own shell.** See §7.

---

## 7. Corrections and records

- **Chat's slips:**
  - The first Claude Code prompt (the read-back script) lacked the HEAD and tree audit,
    and both fresh-session prompts lacked the guard line ("If you can see earlier
    turns, stop and say so"). Both carried the banner. Nothing went wrong, but the rule
    was not followed.
  - **The commit script was given as a code block to save, and was pasted into the
    terminal instead.** It ran in the live shell under `set -euo pipefail`, passed every
    check, committed `a4ad8e1` and pushed it; then the terminal closed and the output was
    lost. The commit was verified read-only afterwards (§2). A follow-up block that
    wrote the script through a pasted heredoc came through garbled (36 of 40 lines,
    lines run together); the file was deleted unrun.
- **Claude Code's slips:**
  - Draft 1 inferred the truth-pose-replay spec's last commit as `00e76e5`; `git log`
    gave `03d666c`.
  - The inventory's citations for O9 (2) and S12's refusal were corrected in draft 1
    (D:1727; D:1342–1344).
  - Its limits (no directory listing, no `git log`) came from the prompt's command list,
    and were closed by Ali's blocks.
- **Records:** CI is green for every commit up to `a4ad8e1`.

---

## 8. Working protocol (changes this session)

- **Script files are written by Claude Code, never pasted.** A commit or run script is
  written by Claude Code into `experiments/logs/msgs/` or `experiments/logs/splice/`,
  pinned by sha256, reviewed in chat, and run as `bash <file>` from a `( … )` block.
  Chat never hands over a script body to paste, and never a heredoc that creates one.
- **Every Claude Code prompt opens with the HEAD and tree audit.** Every fresh-session
  prompt also carries the banner and the guard line, inside the copy block.
- **Read-only Claude Code tasks list the exact commands they may run.** When a fact
  needs a command outside that list (`git log`, `ls`), chat asks Ali for a block, and the
  answer turns INFERRED into fact.
- **Lost output:** verify read-only, never rerun (reaffirmed; it worked).
- **Post-hoc readings** are labelled "post hoc, not registered" wherever they are
  quoted.
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **The B3 spec (next):**
  - the world generator: what a generated world may contain, and how close it stays to
    the knowledge world;
  - the synthetic maps: simulated lidar along explorer-like paths (§3.1);
  - the test-set size: how many worlds get real recordings;
  - the gate: synthetic copies of the 20 maps reproduce their real graphs;
  - the items moved to B3: E-O1, E-O2, E-O5, O2, S-O5.
- **The embedder and the merger** (B3 inventory `b3_inventory.md`, `af8a2c05…`, §3 and
  §5; previous handoff §4 and §8):
  - `input_dim` 2 with an entity-id feature that maps lack;
  - 237 relation ids against 3 edge types here;
  - the Stage 2 loader hard-codes FB15k-237;
  - NSKsim loads `joint_best.pt`, and AgentManager requires `merger_state`;
  - the trained merger gives one constant output and is never called; NSKsim's sharing
    rule is a fixed 0.7 / 0.3 average.
- **The face-offset hypothesis** (§3.1): check it from the node positions before the
  generator's noise model is fixed.
- **Before the B3 test recordings:** the explorer and mapper defects; extent boxes not
  nested across cuts; the wall predicate's B3 check on `k4_cut1200_gated_extfix`
  (check the stopper's grace).
- **Before any rerun:** the S9–S11 fail paths.
- **For Article 3:** the limitations in §4; the pitched scan plane; how to show the
  fixed-A weight series; the three unchecked points of the 13 Oct handoff, checked
  before Article 3 cites them; the designed run, owed only for odometry-against-truth
  claims.
- **At the next fresh Claude Code session:** check that auto mode keeps git commit, git
  push and simulation launches out of its allow list.
- **Held handoffs:** the schema-entry handoff stays out of the repo; 09-29 is still held.
- **Scratch to remove:** drafts and prompts in `~/Downloads/`;
  `experiments/logs/splice/spec_v0_18_preview.diff`.
- **CI** for this handoff's commit.

---

## 10. Next

1. **Check CI** for this handoff's commit.
2. **Start a new chat session** from this handoff.
3. **The B3 spec, in chat first.** Start from `b3_inventory.md` and §9's list. Decide the
   generator's scope and the gate before any code.
4. Then a **fresh Claude Code session** (banner, guard line, audit) for whatever the
   spec needs read first. Check the auto-mode allow list at its start.

---

## 11. The operating lesson

The freeze looked like changing two words in two status lines. The inventory showed
that three specs are read by their own stages, so the honest freeze was a record beside
them, not an edit. The same session's one accident had the same shape: a script did
exactly what it was written to do, in a shell it was never meant for. Before changing
or handing over anything, find what reads it and where it will run.
