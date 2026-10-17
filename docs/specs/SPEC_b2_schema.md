# SPEC — B2 schema: axis ends, openings, corners and slabs (v0.1, draft)

Status: draft for review, 16 Oct 2026. v0.1 gives definitions only: it registers no
prediction and computes nothing on real maps. It rests on three read-only audits by
Claude Code on 16 Oct 2026 (segment ends, axis ends, the world SDF) and on two checks
of its drafts with 160 synthetic grids, the second of which confirmed `tau_c`, `s_max`
and `tau_e`. The audits, checks and scripts are in `experiments/logs/splice/`:
`schema_audit_step0_segment_ends.md`, `schema_audit_step1_axis_ends.md`,
`schema_audit_step2_world.md`, `schema_check_draft1.md`, `schema_check_draft2.md`,
`schema_v0_1_synth.py`, `schema_v0_1_synth2.py` and `schema_v0_1_adversarial.py`.

Depends on: `docs/specs/SPEC_b2_wall_predicate.md` (v2) for face element (Def 1),
segment (Def 2), band (Def 3), gap state (Def 4), endpoint state (Def 5), `rho`,
`eps` and `g`; `docs/specs/SPEC_b2_divergence_walls.md` (v0.22) for the common frame
and the riser finding (O9).

---

## 0. Decisions recorded here

- **D-S1, axis ends (O9 (2)).** Openings and corners read a segment's ends only from
  its axis members (§2). The existing `p0`, `p1` and `length` stay exactly as they
  are, and so does every reader of them. The readers include:
  - `attribute`, which assigns returns to segments, from which `observations` builds
    `bearing_hist`, the input to the weight behind G3, P7, P8, G10 and P17;
  - `segment_guard` in `load_side`, which every stage from S5 to S13 calls (S4 ran
    from an uncommitted script);
  - S10's `length_m`, and `contradicted_detail`;
  - within `graph_walls` itself: `_finish`'s W3 check and `print_b_predictions`,
    which read `length`; `summary_line`; `_segment_arrays`, which reads `p0` and `p1`
    for `validate_map`'s A9 and for `diagnose_extent.recall_by_face`; and
    `validate_map`'s A11, which reads `length`.

  Axis ends are computed after extraction and are never written into the per-map
  files.
- **D-S2, openings as a control.** The world has no true opening (§1). Openings keep
  their definition (§4). Every opening a map yields is therefore an artifact of the
  map, and the schema reports it as such.
- **D-S3, outside corners through slabs.** A maze slab's end face is 0.2 m wide,
  shorter than `l_min` = 0.5 m, so it never becomes a segment (this held in all 160
  synthetic grids). An outside corner therefore cannot be two segments meeting.
  Outside corners are read from slab ends (§7). Two perpendicular segments meeting
  give an inside corner or nothing (§5).
- **D-S4, words.** "Segment" is the map object of Def 2. "World face" is a face of a
  box in the SDF. The wall-predicate spec's D1 reads "A wall node is one observed
  **face**, not a physical slab"; its "face" is a segment in this spec's words.

---

## 1. The world (`knowledge_world.sdf`, sha256 `f5fea67f…`, unchanged since `c39ae5b`)

- **Boxes:** 4 boundary walls, 0.1 m thick, overlapping at the corners; 4 maze slabs,
  0.2 m thick:
  - `maze_h1`: x −6.0…1.0, y 2.9…3.1;
  - `maze_h2`: x −1.0…6.0, y −3.1…−2.9;
  - `maze_v1`: x −4.1…−3.9, y −4.0…1.0;
  - `maze_v2`: x 3.9…4.1, y −1.0…4.0.

  All at yaw 0. No two slabs touch; the closest pairs are 1.9 m apart.
- **Observable world faces:** 20. The 4 boundary inner faces (19.9 m each) and all 16
  slab faces.
- **Openings:** none. No two observable world faces share a line and a facing.
- **Inside corners:** 4, at (±9.95, ±9.95), on the boundary.
- **Outside corners:** 16, at the slabs' corners. Each is also a free end of a long
  world face of a slab.
- **Slabs:** 4 back-to-back pairs of long world faces, 0.2 m apart, both observable.
  The boundary walls form no observable pair.
- **Frames:** every spawn is at yaw 0. With `(x_k, y_k)` robot k's spawn from
  `DOT_POSES` at `08617b2`, a world point (x, y) sits at (x − x_k, y − y_k) in robot
  k's map frame.

---

## 2. Axis ends (Def S1)

For a segment S with unit normal `n`:

- **Axis.** `a(S)` is the direction in `graph_walls.DIRS` with the largest dot with
  `n`. If the two largest dots differ by at most 1e-9, S has no axis. A fit near 45°
  has an axis, which can flip with rounding.
- **Axis members.** The members of S (elements with `seg_of` = S) whose `dir` is
  `a(S)`. W4 rules out a member facing `−a(S)`, so every other member is a riser:
  its `dir` is perpendicular to `a(S)`. Risers occur even in noise-free grids,
  because a fitted line tilts slightly (0.017–4.05° across the 160 synthetic grids).
- **Axis ends.** `q0` and `q1` are the smallest and largest projections of the axis
  members' positions on S's reported line, each extended by `rho/2`, as Def 2 extends
  `p0` and `p1`.
- **No axis ends.** A segment with no axis, or with no axis member, has no axis ends.
  It has no axis end state and is in no gap, corner, pair or slab end. In §4 it still
  counts as a neighbour, so no gap is formed beside it. Such segments are counted.
- **Source.** Computed from `graph_walls.graph_and_elements(stem)`, which returns the
  graph, the element table, `seg_of` and the grid: each segment's `theta`, `normal`
  and `p0`; each element's `x`, `y`, `dir`, `row` and `col`; and the cells. No
  existing function or output field changes.

---

## 3. Axis end state (Def S2)

Def 5, read at axis ends: the bands `[t(q0) − g, t(q0)]` and `[t(q1), t(q1) + g]`
along S's reported line. A band cell counts as S's own only if it is the occupied cell
of an axis member of S. Precedence is Def 5's: `occupied`, then `unknown`, then
`free`.

Why the own-cell rule: in the synthetic grids, counting all members' cells as S's own
gave `unknown` at 1–16 axis ends per variant near true inside corners, where this rule
gives `occupied`.

---

## 4. Openings (Def S3)

- **Gap.** Take one entry of `lines[]` and order its segments by `t(p0)` along that
  entry's line (`theta`, `d`). For two consecutive segments S and S′, both with axis
  ends, the gap is the interval from `t(q1(S))` to `t(q0(S′))`, measured on that line.
  If either neighbour has no axis ends, no gap is formed there. The gap's width is the
  interval's length; its state is Def 4's label over the band on that interval:
  `free`, `unknown` or `mixed`.
- **Opening.** A gap whose state is `free` and whose width is greater than `g`
  (0.22 m), as the ontology says ("wider than the robot's footprint"). `split_runs`
  already guarantees at least `g` between runs, and axis ends only widen it, so the
  width test excludes little.
- **What the band can miss.** The band reaches only `rho*sqrt(2)/2` (about 0.07 m)
  from the line. A short run one cell behind the line, or a recess deeper than that,
  does not show in the band, which then reads `free` if its other cells are free.
- **What the line can invent.** One true world face can sit on two entries of
  `lines[]`. In a separate synthetic case, not one of the 160 grids, a world face
  bowed by 0.3 m gave its two end pieces one entry and its middle another, and §4
  read an 18.30 m `free` gap between the end pieces, so an opening. Such gaps are
  artifacts, as D-S2 expects.
- **In this world** the true number of openings is 0 (§1).

---

## 5. Inside corners (Def S4)

- **Corner.** Take two segments S and S′ with axis ends and perpendicular axes, whose
  reported lines meet at a point c. For each, take its axis end nearer c (on a tie,
  `q0`). They form a corner if both of these nearer axis ends lie within `tau_c` of
  c, and if each segment, from its nearer axis end toward its other axis end, runs in
  the direction of the other segment's axis. Two perpendicular segments that fail any
  of these conditions form no corner.
- **Tolerance.** `tau_c = eps + rho` (0.2707 m). At an inside corner, the through
  segment can take, as risers, the perpendicular segment's elements that lie within
  `eps` of the line its candidates were chosen against. It does so when that line
  tilts toward them and the through segment is extracted first. If the perpendicular
  segment has an axis member in the cell just beyond `eps`, its first remaining axis
  member is then at most `eps + rho` from that line, so its axis end stops at most
  `eps + rho/2` (0.2207 m) short of it. The remaining half cell covers the gap
  between that line and the reported line, on which c lies.
- **When the bound fails.** If that cell holds no axis member of the perpendicular
  segment, its axis end is pushed farther from c, and the corner can then be missed.
  That cell can be a hole in the segment (W2 lets a segment hold holes narrower than
  `g`), or a cell whose only element in the segment is a riser.
- **On the synthetic grids,** §5 found every true inside corner (640 of 640) and no
  false one. Of the two corner axis ends, the farther stopped at most 0.197 m short of
  c, and the nearer lay within 0.031 m of c.
- **Limit.** §5 cannot tell a corner from two perpendicular walls that both stop
  within `tau_c` of their crossing. In an adversarial grid, two 0.2 m slabs stopping
  0.25 m apart were called an inside corner at all 16 offsets. In the world, the
  nearest such ends, other than at the 4 inside corners, are 1.9 m from their
  crossing.
- **Reported, not judged.** The two axis end states at each corner. In the synthetic
  grids, 634 of the 640 found corners read `occupied` at both ends; requiring that
  would have rejected the other 6.
- **In this world** the true inside corners are the 4 on the boundary (§1).

---

## 6. Slabs: back-to-back pairs (Def S5)

For two segments S and S′ with axis ends and opposite axes (`a(S′) = −a(S)`):

- **Midpoints.** `m(S)` is the midpoint of S's axis ends; likewise `m(S′)`.
- **Behind.** `m(S′)` lies on the occupied side of S's reported line, and `m(S)` on
  the occupied side of S′'s.
- **Separation.** The larger of the distance from `m(S′)` to S's line and from `m(S)`
  to S′'s line. It must be at most `s_max`.
- **Overlap.** The intervals `[t(q0), t(q1)]` of S and of S′, both projected on S's
  line direction, overlap.
- **Pair.** S and S′ form a pair if all of the above hold and each is the other's
  nearest qualifying partner by separation (on a tie, by the larger overlap). Each
  segment is in at most one pair.
- **Tolerance.** `s_max = 0.5 m`: the slabs' 0.2 m, one cell from the lattice (a slab
  whose two world faces fall inside cells reads 0.3 m with no blur), and two cells of
  blur.
- **On the synthetic grids,** every slab was found in every grid, with separations up
  to 0.395 m (one cell of blur). Segments from different boxes met "behind" and
  "overlap" only 6.2–8.3 m apart, far beyond `s_max`.
- **In this world** the true slabs are the 4 maze boxes (§1).

---

## 7. Slab ends and outside corners (Def S6)

At each end of a pair (S, S′), take the axis end of S and of S′ on that side.

- **Aligned.** Their positions along S's line direction differ by at most `tau_e`
  (`tau_e = rho`, 0.1 m).
- **State,** in Def 5's precedence:
  - `blocked` if either axis end state is `occupied`;
  - `unobserved` if either is `unknown`;
  - `free` otherwise.
- **Slab end.** An aligned end whose state is `free`. It gives two outside corners,
  one at each of its two axis ends.
- **Recorded otherwise.** An end that is not aligned is recorded as `misaligned`, with
  its state. An aligned end that is `blocked` or `unobserved` is recorded with that
  state. Neither gives outside corners.
- **On the synthetic grids,** every slab end was aligned, with differences of at most
  0.028 m, and `free`.
- **In this world** the true slab ends are 8, and the true outside corners 16 (§1).

---

## 8. What is computed, and where

- A new module, separate from `graph_walls.py` and `divergence_walls.py`, computes
  §2–§7 for one map at a time and writes its own file per map. No existing file,
  function or output field changes, and no per-map file that a stage pins is
  rewritten.
- Every record carries the map's stem and sha256, the `graph_walls` commit, and the
  thresholds used (`rho`, `eps`, `g`, `tau_c`, `s_max`, `tau_e`).

---

## 9. Open, before v0.2

- **S-O1.** Closed in v0.1: `tau_c`, `s_max` and `tau_e` were confirmed on the 160
  synthetic grids before commit.
- **S-O2.** Whether real maps bow enough to split one world face across two entries
  of `lines[]`. Not to be computed before v0.2 registers its predictions.
- **S-O3.** How a schema element is matched to the world's truth in each robot's frame
  (§1), for counting found, missed and spurious elements. In the synthetic grids, axis
  ends at found inside corners lay up to 0.389 m from the true corner, and outside
  corners up to 0.216 m.
- **S-O4.** Which predictions v0.2 registers, and on which rows, before any schema
  element is computed on a real map.
