# HANDOFF — B2 schema v0.1: axis ends, openings, corners and slabs (16 – 17 Oct 2026)

Destination: `docs/handoffs/` and the claude.ai project knowledge.
Previous handoff: `HANDOFF_2026-10-15_b2_parked_shadows_o4_closed.md`.

---

## 0. In one paragraph

The previous handoff left the schema spec next, starting with O9 (2): whether wall
ends, for openings and corners, should ignore risers. This session wrote the schema's
definitions and committed them as `docs/specs/SPEC_b2_schema.md` v0.1 (`86fdc8b`).

- **Wall ends feed registered results.** The existing ends (`p0`, `p1`, `length`)
  decide which returns count toward each wall's `bearing_hist`, the input to the
  viewpoint weight behind G3, P7, P8, G10 and P17. So they stay untouched. Openings
  and corners read a second pair, **axis ends**, built only from elements that face
  the way the wall does.
- **The world has no doorways.** Its 4 maze walls are free-standing 0.2 m slabs. It
  has 4 inside corners (on the boundary), 16 outside corners (at slab ends), 4 slabs
  and no openings.
- **Ali's two decisions (16 Oct):** openings are kept as a control, so every opening
  a map shows is an artifact; outside corners are read from slab ends, because a
  slab's 0.2 m end face never becomes a segment.
- **Six drafts and five checks,** with 160 synthetic grids built with `graph_walls`'
  own extraction. The final rules found all 640 true inside corners, every slab and
  every slab end, with no false finding.

v0.1 registers no prediction and computed nothing on real maps.

---

## 1. Verify first

```bash
# Terminal 1 — no sim — blocking — under 0.1 min
source /opt/ros/jazzy/setup.bash && source ~/Desktop/NSKsim/ros2_ws/install/setup.bash
cd ~/Desktop/NSKsim
git status --short                       # expect only ?? .vscode/
git log --oneline -1 origin/main         # expect the commit that added this handoff
sha256sum docs/specs/SPEC_b2_schema.md
# expect 654745f8d50a5b5c801fc3ffecb5e27beb5e1e80abf32d27b01b6079babd9cc1
```

---

## 2. Commits this session

| Commit | What | Committed |
|---|---|---|
| `86fdc8b` | spec: B2 schema v0.1, axis ends, openings, inside corners and slabs (definitions only) | 17 Oct 07:42 +0300 |
| this commit | docs/handoffs: this handoff | |

- No code changed. No test was run against the repo's suites; all testing was on
  synthetic grids in gitignored scratch scripts.
- **CI** is unchecked for `0c4aff7`, `a3f1760`, `86fdc8b` and this handoff's commit.

---

## 3. The spec (v0.1)

**Decisions.**
- **D-S1, axis ends (O9 (2)).** Openings and corners read ends only from axis
  members. `p0`, `p1`, `length` and all their readers stay as they are.
- **D-S2, openings as a control.** The true number of openings is 0.
- **D-S3, outside corners through slabs.** Two perpendicular segments meeting give an
  inside corner or nothing.
- **D-S4, words.** "Segment" is the map object; "world face" is an SDF box face.

**Definitions.**
- **S1, axis ends.** A segment's axis is the grid direction nearest its normal (no
  axis on an exact tie). Axis members are the members with that direction; the rest
  are risers. `q0`, `q1` are the axis members' extreme projections, each extended by
  half a cell. A segment with no axis ends takes no part, and is counted.
- **S2, axis end state.** Def 5 at the axis ends, counting as the segment's own only
  its axis members' cells.
- **S3, openings.** A `free` gap wider than `g` (0.22 m) between consecutive segments
  on one `lines[]` entry, read between their axis ends.
- **S4, inside corners.** Two perpendicular segments whose nearer axis ends (on a
  tie, `q0`) both lie within `tau_c = eps + rho` (0.2707 m) of their lines' crossing,
  each running away from it along the other's axis.
- **S5, slabs.** Two opposite-facing segments, each behind the other, at most
  `s_max = 0.5 m` apart, overlapping, each the other's nearest partner.
- **S6, slab ends.** At each end of a slab, the two faces' axis ends aligned within
  `tau_e = rho` and both `free`; each slab end gives two outside corners.
- **§8.** A new module will compute these per map into its own files. Nothing
  existing changes.

---

## 4. Findings

All files are in `experiments/logs/splice/` (gitignored).

### 4.1 Audits (16 Oct)

- **Step 0, segment ends** (`schema_audit_step0_segment_ends.md`, `137b3325…`).
  `graph_walls` sets ends from every member, risers included. Registered quantities
  read them: `bearing_hist` through `attribute`, and `segment_guard` in `load_side`.
- **Step 1, axis ends** (`schema_audit_step1_axis_ends.md`, `e05ada89…`). Axis ends
  can be computed after extraction from `graph_and_elements`, changing nothing. A
  segment can lack an axis member (a tilted comb); a 45° fit is a tie.
- **Step 2, the world** (`schema_audit_step2_world.md`, `6cb6d1a3…`; script
  `schema_audit_step2_world.py`). The inventory above. Every wall is axis-aligned.

### 4.2 Synthetic grids

- **Draft-1 check** (`schema_check_draft1.md`, `6b1c719f…`; `schema_v0_1_synth.py`).
  The first tolerances failed:
  - `tau_c = rho` found only about half the inside corners. At a corner, the through
    segment takes the perpendicular segment's last elements as risers, so that
    segment's axis end stops 0.17–0.20 m short.
  - `s_max = 0.3 m` sat exactly on the lattice value; one cell of blur broke it.
  - `tau_e = rho` held.
  - A world face bowed 0.3 m split across two `lines[]` entries and gave an
    18.30 m false opening.
- **Draft-2 retest** (`schema_check_draft2.md`, `ed8bdb27…`; `schema_v0_1_synth2.py`,
  `schema_v0_1_adversarial.py`). With `tau_c = eps + rho` and `s_max = 0.5 m`: 640 of
  640 inside corners, every slab, every slab end aligned and `free`, no false
  finding. Limit: two slabs stopping 0.25 m apart are called a corner. In the world
  the nearest such ends are 1.9 m from their crossing.
- **Unrounded bounds** (`schema_v0_1_raw_bounds.py`, `551465fc…`). Used from draft 4
  on, because printed values had been rounded down (0.0270 was 0.0270012).

---

## 5. Gates and predictions

None new. v0.1 is definitions only. The B2 walls table in the previous handoff's §5 is
unchanged.

---

## 6. What was learned

- **Find every reader before changing a quantity.** Wall ends looked like geometry.
  They also decide which returns count toward the viewpoint weight.
- **Inventory the world before defining categories.** The maze has no doorways.
  Without the world audit, openings would have been defined against nothing.
- **A tolerance from first principles still needs grids.** `tau_c = rho` looked right
  and found half the corners; the through segment's riser rule explained why.
- **Printed rounding hides bounds.** Check claims against unrounded values.
- **A list called complete keeps failing.** The reader list failed twice; "include"
  holds.
- **Fresh sessions need a guard.** Prompts kept landing in the old session. A first
  line ("If you can see earlier turns, stop and say so") caught it, and `claude` in a
  new terminal gave a clean one.

---

## 7. Corrections and records

- **Chat's errors in the drafts:**
  - draft 1: `tau_c = rho`, `s_max = 0.3 m` with "blur" as its reason, "a short run
    makes the gap `mixed`", S-O2's direction;
  - draft 2: "another junction" and "takes no part", each contradicting another
    rule; a bound derivation stated without its conditions;
  - drafts 3–5: an incomplete reader list, an unresolved tie in the corner rule, and
    "an element" where an axis member was meant.
- **Claude Code's errors, found in its own later checks:** "every stage from S4"
  (there is no S4 stage); a tilt range taken from one world only; three values
  rounded down; "the diagnose scripts read only SDF faces" (`diagnose_extent` reads
  segments); "for A8–A11" (only A9 reads the ends).
- **Reports and sessions.**
  - The old Claude Code session compacted on 15 Oct. Its three audits, its draft-1
    check and its draft-3 check were extracted from its transcript. The draft-3 check
    (`c86d0039…`) matched the copy Ali pasted to chat byte for byte, apart from the
    final newline.
  - In the new session, the replies holding the draft-4 and draft-5 checks were
    stopped by a safety classifier, so it could not write them out again. The draft-4
    check was filed from chat's copy (`schema_check_draft4.md`, `d4b30303…`). The
    draft-5 check exists only in chat.
  - The new session fixed Claude Code's memory index (`fresh-session-signals.md` was
    missing from it; the commit rule's index line was wrong).
- **Drafts** 1–6 are pinned in `experiments/logs/splice/`; the spec is draft 6 less
  its draft note.

---

## 8. Working protocol (changes this session)

- **Every command goes in chat as a code block,** wrapped in `( … )`, so a failed
  check stops the block without closing the terminal.
- **Chat says when a fresh Claude Code session is due:** a chain closes, the session
  compacts or can't quote its earlier reports, it repeats a fixed mistake, or a large
  build starts. Claude Code holds the same rules in `fresh-session-signals.md`.
- **Before switching,** every report the next step needs is saved to
  `experiments/logs/splice/` and pinned by sha256.
- **A prompt for a fresh session opens with a guard,** and is self-contained: it pins
  and lists every file to read.
- **Start a fresh session with `claude` in a new terminal.**
- **Bounds are checked against unrounded values.**
- Everything else in the previous handoff's §8 is unchanged.

---

## 9. Open items

- **Schema, before v0.2:**
  - **S-O2.** Whether real maps bow enough to split one world face across two
    `lines[]` entries. Not to be computed before v0.2 registers.
  - **S-O3.** How a schema element is matched to the world's truth in each robot's
    frame. On the grids, inside-corner axis ends lay up to 0.389 m from the true
    corner and outside corners up to 0.216 m.
  - **S-O4.** Which predictions v0.2 registers, and on which rows.
- **O9 (2)** is decided by the schema spec's D-S1. The divergence spec still lists it
  as open; note it there at its next version.
- **Carried over:** O2, O5, O9 (1), O10; w_null and E7 at mixed cuts; how Article 3
  shows the fixed-A weight series; the S9–S11 fail paths; the pitched scan plane in
  Article 3; the three unchecked points from the 13 Oct handoff; extent boxes not
  nested; B3 not judged on k4_cut1200_gated_extfix; the explorer and mapper defects
  before B4; Claude Code's auto-mode allow list.
- **Scratch to remove** from `~/Downloads/`: this session's drafts and prompts (the
  pinned copies are in `experiments/logs/splice/`).

---

## 10. Next

1. **Check CI** for `0c4aff7`, `a3f1760`, `86fdc8b` and this handoff's commit.
2. **Start a fresh Claude Code session.**
3. **Schema v0.2:** settle S-O3 (matching to truth), then register predictions
   (S-O4) before any schema element is computed on a real map.
4. **Then the schema module,** then B3, in the planned order of Article 3.

---

## 11. The operating lesson

The question looked like geometry: where does a wall end? The audit showed that the
answer was already wired into five registered results, and the world file showed
that one of the three categories had no true instance at all. Both facts were cheap
to find and expensive to miss. The tolerances took five checks, and each check found
something real. The rules that came out find every true corner and slab on the test
grids, and nothing else.
