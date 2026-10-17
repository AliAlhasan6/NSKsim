# HANDOFF — B2 schema v0.2: matching to truth and eight predictions (17 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-17_b2_schema_v0_1.md`.

---

## 0. In one paragraph

The previous handoff left S-O3 (matching schema elements to truth) and S-O4 (which
predictions to register) before any schema element is computed on a real map. This
session settled both and registered v0.2 of `docs/specs/SPEC_b2_schema.md` (`5bf3ee2`).

- **Matching goes through faces.** A segment matches a world face when the face's
  normal is its axis, the midpoint of its axis ends lies within `tau_m = 2*rho`
  (A8's and A9's tolerance) of the face's line, and the extents overlap. Corners,
  slabs and slab ends are true when their segments match the right faces. No element
  is matched by a radius around a point.
- **Spurious has two kinds, both judged.** *Wrong*: every segment matches a face, but
  not the true ones. *Unmatched-part*: a segment matches no face.
- **Eight predictions and a gate.** Found elements are judged on the five cut1200
  maps; wrong and unmatched-part elements are judged on all 20.

No schema element has been computed on a real map. Only stored A8/A9 values and the
segment count per map were read.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
( source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim || exit 1
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
sha256sum docs/specs/SPEC_b2_schema.md )
# expect 8dd6da2ddfac39093b0669772e4465b257b8de90a51f37d0e3a38479a1b4eef6
```

---

## 2. Commits this session

| Commit | What |
|---|---|
| `5bf3ee2` | spec: B2 schema v0.2, matching to truth (S-O3); eight predictions registered (S-O4) |
| this commit | docs/handoffs: this handoff |

- **CI** is green for `0c4aff7`, `a3f1760`, `86fdc8b` and `3da353b`. It is unchecked
  for `5bf3ee2` and this handoff's commit.
- No code changed. All checks ran on synthetic grids and emulated maps in gitignored
  scratch scripts.

---

## 3. The spec (v0.2)

§1–§7 are byte-identical to v0.1.

**Decisions, all confirmed by Ali on 17 Oct 2026.**
- **D-S5, truth after the fact.** Truth enters only in the stage that judges §10,
  never in the per-map files.
- **D-S6, matched through faces,** with one tolerance, `tau_m = 2*rho`.
- **D-S7, rows.** Found elements are judged on cut1200 only; wrong and unmatched-part
  elements are judged on all 20 maps, each kind apart.

**§9, matching (M0–M6).**
- **M0:** frame `p + s_K`, `DOT_POSES` at `08617b2`.
- **M1:** face match.
- **M2–M4:** inside corners, slabs, slab ends.
  - `tau_end = tau_m + rho/2` (0.25 m).
  - A slab end of a true pair beyond `tau_end` is *wrong* (Claude Code's choice,
    accepted).
  - Outside corners follow their slab end.
- **M5:** every opening is spurious.
- **M6:** misses are reported with reasons, never judged.

**§10, gate G-S1** (in order, a failure stops the rest):
1. The synthetic counts reproduce.
2. A9 reproduces from each map's segments within 1e-9.
3. Decoy X-S1, the truth shifted (+0.5, +0.5) m, matches no segment.

**Predictions:**

| | Rows | Holds when |
|---|---|---|
| P-S1 | cut1200 | ≥ 18 of 20 inside corners found |
| P-S2 | all 20 | ≤ 1 map with a wrong inside corner |
| P-S3 | cut1200 | ≥ 18 of 20 slabs found |
| P-S4 | all 20 | ≤ 1 map with a wrong pair or slab end |
| P-S5 | cut1200 | ≥ 36 of 40 slab ends found |
| P-S6 | cut1200 | ≤ 1 map with an opening |
| P-S7 | all 20 | ≤ 1 map with an unmatched-part inside corner |
| P-S8 | all 20 | ≤ 1 map with an unmatched-part pair or slab end |

**The stage** follows S13's pattern:
- It refuses an edited tree.
- It requires `5bf3ee2` as an ancestor of HEAD.
- It stores the registered text and finds every number it applies in that text.

---

## 4. Findings

All files are in `experiments/logs/splice/` (gitignored).

| File | sha256 |
|---|---|
| SPEC_b2_schema_draft1_v0_2.md | `81a6d22a…` |
| SPEC_b2_schema_draft2_v0_2.md | `b8c76708…` |
| SPEC_b2_schema_draft3_v0_2.md | `db0a8fc7…` |
| SPEC_b2_schema_draft4_v0_2.md | `30db98ba…` |
| schema_check_v0_2_draft1.md | `d6c07162…` |
| schema_v0_2_match.py / _out.txt | `f4c19286…` / `4b83bf14…` |
| schema_v0_2_partial.py / _out.txt | `2b5b501a…` / `70676c01…` |
| schema_check_v0_2_draft2.md | `609a1703…` |
| schema_v0_2_match_d2.py / _out.txt | `d3c76487…` / `943f7a92…` |
| schema_v0_2_partial_d2.py / _out.txt | `88abcb6a…` / `f86e4362…` |

Both draft-1 checks reproduced byte for byte on Ali's machine (about 10 min);
recorded in chat as BOTH_SAME from Ali's terminal, not in the repo.

- **640 synthetic grids** (8 variants × 5 spawns × 16 phases, in each robot's frame):
  - Outside blur2, every inside corner, slab and slab end was found, with nothing
    wrong, unmatched or duplicated.
  - **blur2** (a face two cells thicker) lost 160 inside corners as missed and gave
    160 unmatched-part ones.
- **Margin.** The largest midpoint distance was 0.19098039 m (thickE). `tau_m` exceeds
  it by 0.0090196 m, at both grid and map `rho`.
- **jog2.** A corner's crossing point lay 0.351 m from the true corner, yet the corner
  matched through its faces. A radius as wide as `tau_c` would have missed it.
- **Decoy.** Every corner became unmatched-part, and none was wrong.
- **80 emulated partial maps** (Karto-like counting, four cuts):
  - nothing wrong or unmatched-part at any cut;
  - 6 openings on 6 maps, 5 at the first cut, each a gap on a real slab face, none
    at full path;
  - one face split across two `lines[]` entries on 2 of 80.

---

## 5. Gates and predictions

G-S1 and P-S1 to P-S8 are registered at `5bf3ee2`. None is computed yet. The B2
walls table is unchanged.

---

## 6. What was learned

- **Match through the parts, not by distance to a point.** Under a jog, the corner
  point moved 0.351 m while both faces still matched.
- **A split meant to diagnose can empty a prediction.** Judging only "wrong" left
  P-S2 and P-S4 nearly unable to fail. The decoy showed it: a total misplacement gives
  unmatched-part findings, never wrong ones. Split for diagnosis, and judge both kinds.
- **State a thin margin unrounded.** 0.191 m against 0.2 m reads as safe until it is
  written as 0.009 m.
- **Run the decoy against every new category,** to show each prediction can fail.

---

## 7. Corrections and records

- **Chat's error:** in draft 2, chat asked to judge only wrong findings. That left
  P-S2 and P-S4 nearly empty and a phantom corner unjudged. Fixed in draft 3 with
  P-S7 and P-S8.
- **Chat's earlier point** that 3ρ would cut the 0.5 m decoy's margin from 0.109 m
  to about 9 mm stands. It was one reason for keeping 2ρ.
- **Fresh session at the start:** chat said one was due. The Claude
  Code session was already fresh from its first turn; its remark that
  none of the four signals held was about a further switch after the
  work. There was no disagreement.
- **Claude Code reran draft 2's checks** from `_d2` copies, because draft 1's check
  report pins the originals by sha256.
- **The title keeps "(v0.2, draft)"** like the divergence spec. The stage stores the
  text verbatim, so it is not to be edited.

---

## 8. Working protocol

Unchanged from the previous handoff's §8.

---

## 9. Open items

- **S-O5, earlier cuts.** Judging found elements before cut1200 needs an "in view"
  rule not built from the schema itself.
- **S-O6, blur beyond A8's tolerance.** P-S7, P-S8, M6 and the distances would show
  it. G-S1 cannot.
- **O9 (2)** is decided by the schema spec's D-S1. The divergence spec still lists it
  as open; note it there at its next version.
- **Carried over:** O2, O5, O9 (1), O10; w_null and E7 at mixed cuts; how Article 3
  shows the fixed-A weight series; the S9–S11 fail paths; the pitched scan plane in
  Article 3; the three unchecked points from the 13 Oct handoff; extent boxes not
  nested; B3 not judged on k4_cut1200_gated_extfix; the explorer and mapper defects
  before B4; Claude Code's auto-mode allow list.
- **Scratch to remove** from `~/Downloads/`, if any remains from this session.

---

## 10. Next

1. **Check CI** for `5bf3ee2` and this handoff's commit.
2. **Start a fresh Claude Code session** with `claude` in a new terminal.
3. **Build the schema module** (§8 per-map files) and the judging stage, with G-S1
   first.
4. **Run the stage** and record P-S1 to P-S8.
5. **Then B3,** in Article 3's planned order.

---

## 11. The operating lesson

The draft's first split made the false-finding predictions safe by making them nearly
unable to fail. The decoy, built to find nothing, showed where false findings actually
land. Every prediction now has a check that shows it can fail. The margin is written
to seven decimals, so the next reader sees how thin it is.
