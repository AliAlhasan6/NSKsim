# HANDOFF — B2 schema: module, stage and first run (17 – 18 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-17_b2_schema_v0_2_registered.md`.

---

## 0. In one paragraph

The previous handoff left the schema module and its judging stage to build. This
session built both (`266429b`, `5a72769`) and ran the stage once on the 20
`gated_extfix` maps. Gate G-S1 held, and all eight registered predictions hold. At
cut1200, every inside corner, slab and slab end was found; on all 20 maps, nothing
was wrong or unmatched-part, and no map at any cut showed an opening.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
( source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim || exit 1
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
sha256sum experiments/logs/schema/judge_5a72769_20261018T103953Z/schema_judge_20261018T104734Z.json )
# expect 84ffdb5a5d6a75ee9e50bb27658df1878353ea0227dfe6eac98ecc3ae5379f18
```

---

## 2. Commits

| Commit | What | CI |
|---|---|---|
| `266429b` | schema_walls: §2–§7 on one map, and its per-map file | success |
| `5a72769` | schema_judge: the stage, gate G-S1 and P-S1 to P-S8 | success |
| this commit | docs/handoffs: this handoff | unchecked |

- **Commit 1:** 31 tests. The listed suite gave 1043 passed and 33 skipped
  (unsourced).
- **Commit 2:** 74 tests. The listed suite gave 1150 passed and 0 skipped in a
  ROS-sourced shell, which is 1117 plus the 33 tests that skip unsourced.

---

## 3. The run

**Location.** `experiments/logs/schema/judge_5a72769_20261018T103953Z/` (gitignored).
It ran in 461 s, 445.3 s of which was check 1.

| File | sha256 |
|---|---|
| schema_judge_20261018T104734Z.json | `84ffdb5a…` |
| schema_judge_run.log | `cb5ad018…` |
| schema_maps_20261018T104009Z/ | 20 per-map files |

**Gate G-S1: HOLD.**
- **Check 1:** every pinned count reproduced.
- **Check 2:** A9 was recomputed through `validate_map`. It is bit-equal to the stored
  value on 20 of 20 maps.
- **Check 3:** the decoy matched no segment on 20 of 20 maps.

| | Result | Bound |
|---|---|---|
| P-S1 | 20 of 20 inside corners found | ≥ 18 |
| P-S2 | 0 maps with a wrong inside corner | ≤ 1 |
| P-S3 | 20 of 20 slabs found | ≥ 18 |
| P-S4 | 0 maps with a wrong pair or slab end | ≤ 1 |
| P-S5 | 40 of 40 slab ends found | ≥ 36 |
| P-S6 | 0 of 5 cut1200 maps with an opening | ≤ 1 |
| P-S7 | 0 maps with an unmatched-part inside corner | ≤ 1 |
| P-S8 | 0 maps with an unmatched-part pair or slab end | ≤ 1 |

**Reported, not judged** (`experiments/logs/splice/schema_run1_summary.md`,
`ea8c33aa…`):
- **Margin.** The largest m(S) distance is 0.11041341484547296 m (k4 cut1200, face
  N). That leaves a margin to `tau_m` of 0.0896 m, against 0.0090 m on the synthetic
  grids.
- **Clean everywhere.** Openings, duplicates and unmatched segments are zero at every
  cut. 5 segments have no axis ends (one at cut60, one at cut240, three at cut1200);
  they take no part and are counted.
- **Misses** (corners / slabs / slab ends): cut60 20/17/36, cut120 16/14/29, cut240
  11/6/16, cut1200 none.
  - Two corners were missed with both faces matched: k0 cut120 ES and k4 cut240 WS.
  - Every missed slab had one long face matched.
  - Seven slab ends were missed on true pairs: six misaligned and one unobserved.
- **S-O2.** One face is split across two `lines[]` entries: maze_h2-y on k2 cut60.
- **Emulation versus the corpus.** The emulated partial maps showed openings on 6 of
  80 emulated maps (5 of 20 near cut60, 1 of 20 near cut240); the corpus showed none
  at any cut.

---

## 4. Files in splice this session

| File | sha256 |
|---|---|
| schema_build_plan.md (Ali's four rulings in its header) | `b93adf76…` |
| schema_v0_2_match_d3.py / _out.txt | `15c763e5…` / `7cdf0930…` |
| schema_check_v0_2_d3.md (ruling 3 holds) | `420cfef5…` |
| schema_judge_build_report.md (written afresh; see §6) | `9bcc6631…` |
| schema_gs1_check1_repro.py / .txt | `069d93dc…` / `289d9072…` |
| schema_run1_summary.md | `ea8c33aa…` |

---

## 5. What was learned

- **Measure a spec-versus-scratch gap before building on it.** d3 showed that the 12
  faces give the same counts as the scratch's 20, so the gate was never at risk of
  failing on a difference between the scratch and the spec.
- **A suspicious timing needs a code read, not trust.** A check that prints 0.0 s
  looked as if it had been skipped. It wasn't: the timer starts after the
  extraction.
- **Shell and interpreter change the counts.** `python3` instead of `venv`, or a
  sourced shell instead of an unsourced one, moves the skip count. Baselines name
  their shell.
- **Save a report to disk before pasting it.** The commit-2 report was garbled in the
  paste and lost with its session.

---

## 6. Corrections and records

- **The commit-2 build report is lost.** The session that built commit 2 ended before
  saving it, and the paste was garbled. A later session wrote a report afresh from
  the code, the diff and the message file, labelled as such. Check 1 was rerun by Ali
  on 18 Oct, and its output is pinned.
- **Prompts landed in the wrong session three times.** The guard line caught each
  one.
- **A safety classifier interrupted a Claude Code run once.** After that, long runs
  went to Ali as scripts; Claude Code still ran read-only checks (a setup-only call
  of check 1, the room-fixture check, pytest --collect-only).
- **Claude Code's slips:**
  - its d3 diff prediction was off by one line;
  - the d3 run block, in the commit-1 session, used `python3` from inside
    `splice/`;
  - read-only commands ran during turns asked to run nothing: a grep in the
    commit-1 session; ls, git queries and python one-liners reading four result
    JSONs before the run block;
  - a message rewrite briefly had five dots.
- **Chat's slip:** chat put pytest inside the ROS-sourced shell, so commit 2's suite
  count (1150, 0 skipped) is not directly comparable to commit 1's (1043, 33 skipped).
  The block Ali ran was chat's, which sourced ROS before pytest.
- **The room fixture** in `test_schema_judge.py` was moved to (20, 20). At the origin
  it sat 0.05 m from two decoy faces. It matches no true face in either position.
- **Positive matches on extracted segments** are tested only by gate check 1b, which
  runs before every stage run.

---

## 7. Working protocol

Unchanged. New: **every report a later step needs is saved to splice before it is
pasted to chat.**

---

## 8. Open items

- **S-O5:** an "in view" rule for judging found elements before cut1200.
- **S-O6:** blur beyond A8's tolerance. Not seen on real maps; margin 0.0896 m.
- **O9 (2)** is decided by the schema spec's D-S1. Note this in the divergence spec at
  its next version.
- **The sim has not run for about ten sessions.** B4 needs the explorer and mapper
  defects fixed first. Ali asked whether that work should start in parallel; it is
  not yet decided.
- **Carried over:** O2, O5, O9 (1), O10; w_null and E7 at mixed cuts; how Article 3
  shows the fixed-A weight series; the S9–S11 fail paths; the pitched scan plane in
  Article 3; the three unchecked points from the 13 Oct handoff; extent boxes not
  nested; B3 not judged on k4_cut1200_gated_extfix; Claude Code's auto-mode allow
  list; scratch to remove from ~/Downloads/.

---

## 9. Next

1. **Check CI** for this handoff's commit.
2. **Start a fresh Claude Code session.**
3. **B3,** in Article 3's planned order. Decide whether the explorer and mapper defect
   work runs in parallel.

---

## 10. The operating lesson

The schema went from registered text to a verdict in two commits and one run, and the
run changed nothing it was registered to judge. Two things almost went wrong:
- a check that looked skipped;
- a lost report.

A code read caught the first. For the second, a fresh write-up labelled honestly, plus
a rerun on Ali's machine, replaced the lost text.
