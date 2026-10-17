# SPEC — B2 schema: axis ends, openings, corners and slabs (v0.2, draft)

Status: registered, 17 Oct 2026. v0.2 adds how a schema element is matched to
the world's truth (§9, settling S-O3) and registers eight predictions with a gate
(§10, settling S-O4). §1–§7 are unchanged; §0 gains three decisions and §8 one
paragraph. No schema element has been computed on a real map.

v0.1 rests on three read-only audits by Claude Code on 16 Oct 2026 (segment ends,
axis ends, the world SDF) and on two checks of its drafts with 160 synthetic grids,
the second of which confirmed `tau_c`, `s_max` and `tau_e`. v0.2 adds two checks by
Claude Code on 17 Oct 2026: §9 on 640 synthetic grids built in each robot's frame,
and §2–§7 with §9 on 80 emulated partial maps. The audits, checks and scripts are in
`experiments/logs/splice/`: `schema_audit_step0_segment_ends.md`,
`schema_audit_step1_axis_ends.md`, `schema_audit_step2_world.md`,
`schema_check_draft1.md`, `schema_check_draft2.md`, `schema_v0_1_synth.py`,
`schema_v0_1_synth2.py`, `schema_v0_1_adversarial.py`, `schema_v0_2_match_d2.py`,
`schema_v0_2_match_d2_out.txt`, `schema_v0_2_partial_d2.py` and
`schema_v0_2_partial_d2_out.txt`.

Depends on: `docs/specs/SPEC_b2_wall_predicate.md` (v2) for face element (Def 1),
segment (Def 2), band (Def 3), gap state (Def 4), endpoint state (Def 5), `rho`,
`eps`, `g`, and A8's and A9's tolerance (§6); `docs/specs/SPEC_b2_divergence_walls.md`
(v0.22) for the common frame, the 20 maps (its §2) and the riser finding (O9).

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
- **D-S5, truth after the fact** (confirmed by Ali, 17 Oct 2026). §2–§7 read no
  truth, and the per-map files of §8 hold none. §9 matches their records to truth
  only inside the stage that judges §10, as the divergence spec uses truth only to
  validate (its §1 and §8).
- **D-S6, matched through faces** (confirmed by Ali, 17 Oct 2026). Every schema
  element is matched to truth through the world faces its segments match (§9), with
  one tolerance, `tau_m`, which is A8's and A9's. No element is matched by a radius
  around a point.
- **D-S7, rows** (confirmed by Ali, 17 Oct 2026). Found elements are judged only on
  the five cut1200 maps; wrong and unmatched-part elements are judged on all 20, each
  kind apart (§9, §10).

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
- §9 is computed only by the stage that judges §10 (D-S5), in memory, from the
  per-map files and the SDF. Its results go into that stage's results file, never
  into the per-map files.

---

## 9. Matching to truth (Def S7, S-O3)

- **M0, frame.** A point p in robot K's map frame lies at p + s_K in the world, with
  s_K from `DOT_POSES` at `08617b2`: translation only, yaw 0. This is how A8–A10 place
  segments (`graph_walls._segment_arrays`) and how the divergence spec's §8 places
  them.
- **Truth (§1),** from `knowledge_world.sdf` through `graph_walls.sdf_wall_faces`,
  which stops if the SDF's face totals change:
  - the 12 long observable world faces, each with its outward normal, its line and
    its exposed extent: for a boundary face, the 19.9 m between the two inside
    corners on it; for a slab face, the whole face;
  - the 4 inside corners, each with its two boundary faces;
  - the 4 slabs, each with its two long faces and its two ends; an end is the
    coordinate of an end cap along the slab's long axis;
  - the 16 outside corners, two at each slab end.

  Same-facing long faces are at least 6.0 m apart (`maze_h1` and `maze_h2`, +y faces
  and −y faces alike).

### M1, face match

A segment S with axis ends matches world face W if:

- W's outward normal is `a(S)`;
- the midpoint `m(S)` of S's axis ends (§6) lies within `tau_m` of W's line;
- S's interval `[t(q0), t(q1)]` and W's exposed extent, both projected on W's
  direction, overlap.

If more than one face qualifies, S matches the one whose line is nearest `m(S)`. In
this world that cannot happen: two qualifying faces would face the same way and lie
at most `2*tau_m` apart. A segment with axis ends that matches no face is
**unmatched**, and counted.

- **Tolerance.** `tau_m = 2*rho` (0.2 m), `graph_walls.TRUTH_TOL_CELLS`. On the 20
  maps, A8 places every element of every segment within `tau_m` of a wall surface:
  share 1.0 on 20 of 20 maps (`experiments/logs/graph_walls/
  _validation_20261004T021555Z.json`, sha256 `555ef95f…`).
- **Why the midpoint.** A fitted line tilts, so its axis ends can lie farther from W
  than its midpoint. On the emulated partial maps, axis ends lay up to 0.208 m from
  their face and midpoints up to 0.114 m.
- **Margin.** On the 640 grids, the largest `m(S)` distance was
  0.19098039120664012 m (thickE: the east boundary face one cell thicker on its free
  side; jog2 reached 0.19097824360901683 m). `tau_m` minus it is **0.0090196 m**,
  with `rho` = 0.1 as the grids use it and with the maps' `rho` =
  0.10000000149011612 alike. With no blur the largest was 0.09098039120664403 m
  (base). On the emulated partial maps the largest was 0.11383175706494342 m, a
  margin of 0.0861682 m.
- **When the bound fails.** A face two cells thicker on its free side puts its
  elements 0.2–0.3 m from the surface, at or beyond A8's tolerance. On such grids
  (blur2), the east boundary face matched nothing, so both inside corners on it were
  missed and both were counted unmatched-part: 160 of each over 80 grids.

### Spurious: wrong and unmatched-part

A finding of M2–M4 that is not true is **spurious**, of one of two kinds:

- **wrong:** every segment in it matches a face, but not the true faces of any
  element of its kind; for a slab end only, also a slab end whose faces are right
  but which lies beyond `tau_end` of the end (M4);
- **unmatched-part:** one of its segments matches no face.

A wrong finding is §5–§7 combining faces that the world does not combine. An
unmatched-part finding rests on a segment that M1 could not place on any world face
(S-O6). Both kinds are judged on all 20 maps, apart: wrong findings by P-S2 and
P-S4, unmatched-part findings by P-S7 and P-S8 (§10).
Openings are not split: every opening is spurious (D-S2, M5).

### M2, inside corners

- A found corner (§5) is **true** for inside corner K if its two segments match K's
  two faces, one each. Otherwise it is spurious: **wrong** if both its segments match
  faces, **unmatched-part** if one of them matches none.
- K is **found** if some corner is true for it. A further corner true for K is a
  **duplicate**, counted apart.
- **Reported:** the distance from c to K. On the grids it reached 0.351 m (jog2)
  while both faces still matched. A radius around K as wide as `tau_c` would have
  missed those corners.

### M3, slabs

- A pair (§6) is **true** for slab B if its two segments match B's two long faces, one
  each. Otherwise it is spurious: **wrong** if both its segments match faces,
  **unmatched-part** if one of them matches none.
- B is **found** if some pair is true for it. A further true pair is a duplicate.

### M4, slab ends and outside corners

Take a pair true for slab B. Assign each of its two ends to the end of B nearer to
the mean, along B's long axis, of that end's two axis ends.

- A slab end (§7) is **true** for its assigned end E if that mean lies within
  `tau_end` of E. E is **found** if some slab end is true for it. A further one is a
  duplicate.
- Otherwise a slab end is spurious. It is **wrong** if its pair is true but the mean
  lies beyond `tau_end` of E (both segments match the slab's faces, but the end is
  not at E), or if its pair is wrong. It is **unmatched-part** if its pair is
  unmatched-part.
- **Tolerance.** `tau_end = tau_m + rho/2` (0.25 m). If an axis member beyond the
  end cap lies within `tau_m` of the end cap's world face, as A8 found for every
  element, the axis end lies within `tau_end` of E, because §2 extends an axis end
  half a cell beyond its last axis member. On the grids the mean
  lay at most 0.190 m from E with both end caps one cell thicker (endcap), and at
  most 0.090 m otherwise.
- **Outside corners** go with their slab end: a true slab end finds its two outside
  corners. They are reported, as twice the slab ends, and not counted apart.
  **Reported:** each outside corner's distance to its box corner (at most 0.237 m on
  the grids, jogS).

### M5, openings

Every opening is spurious (D-S2). Each is reported with its width, its state, and the
faces its two segments match.

### M6, missed, and why

A true element that is not found is **missed**. Reported, never judged, for each:

- an inside corner or a slab: how many of its faces some segment matches; for an
  inside corner with both faces matched, for each face, the distance from the
  corner to the nearest axis end of any segment matching that face;
- a slab end: whether its slab has a true pair, and if so that end's record from §7
  (`misaligned`, `blocked`, `unobserved`, or `free` beyond `tau_end`).

Also reported, per map: for each world face, how many segments match it and on how
many `lines[]` entries they lie (S-O2).

**"Both faces matched" does not mean "in view."** On the emulated partial maps, 4,
10 and 24 inside corners at the first three cuts were missed although both their
faces were matched; the corner region itself had not been seen. M6 explains a miss.
It does not excuse one.

### On the checks

- **Synthetic grids** (640: the five v0.1 variants and three new ones, each at 5
  spawns × 16 lattice phases, built in robot K's frame and matched through M0). The
  new variants are blur2 (east boundary face two cells thicker), endcap (both end
  caps of `maze_h1` one cell thicker) and jog2 (half the east boundary face two cells
  thicker, a 0.2 m jog). On every grid outside blur2: every true inside corner, slab
  and slab end found; nothing wrong, unmatched-part, unmatched or duplicated; no
  opening. On blur2: 160 inside corners missed and 160 unmatched-part, none wrong;
  every slab and slab end found.
- **The same grids against a decoy truth,** the world shifted by (+0.5, +0.5) m: no
  segment matched a face, so every corner the grids hold was unmatched-part (320 per
  variant) and none was wrong.
- **Emulated partial maps** (80: 5 spawns × 2 paths × 2 lattice phases × 4 cuts).
  Karto's hit and pass counting along an exploration path (`min_pass_through` 2 and
  `occupancy_threshold` 0.1, as `slam_robot*.yaml` sets them; the offline template
  does not set them), with the burger's beam pattern and range noise, cut after 5, 12
  and 22 m of travel and at the end of the path. Their A9-like recall was
  0.24–0.37, 0.38–0.48, 0.54–0.64 and 0.998–0.999; the corpus's A9 is 0.19–0.32,
  0.42–0.50, 0.63–0.74 and 0.992–0.993 at its four cuts. Results:
  - no wrong, unmatched-part or unmatched element at any cut, and no duplicate;
  - 6 openings on 6 maps (5 at the first cut, 1 at the third), each a free gap
    between two segments matching the same slab face; none at the end of the path;
  - one face on two `lines[]` entries, on 2 of the 80 maps.

  The emulation is not the corpus. Its only noise is 0.01 m of range noise, and it
  samples cells along each beam rather than tracing them as Karto does.

---

## 10. Predictions (S-O4)

**Rows.** The 20 maps of the divergence spec's §2,
`b2maps_k{K}_cut{C}_gated_extfix_robot{K}`, K from 0 to 4 and C in {60, 120, 240,
1200}, with their graph JSONs. A map whose sha256 differs from its JSON's
`map_sha256` stops the stage.

**Why found elements are judged only at cut1200** (D-S7). At cut1200, A9 covers
0.9970–0.9981 of the reachable inward face length on all five maps; at earlier cuts
A9's recall is 0.19–0.74 (the stored file above). Early on, a true element can be
missed because the robot never looked at it, and M6 cannot separate that case (§9).
Wrong and unmatched-part elements need no such denominator.

**Read before registration.** From real maps, only A8 and A9 per map (the stored
file above) and the number of segments per map in the graph JSONs: 4–12 before
cut1200 and 12–13 at cut1200, against 12 long observable world faces. Nothing of
§2–§7 or §9 was computed on a real map.

### Gate G-S1

Three checks, in this order, before any count. A failed check runs nothing after it.

1. **The synthetic grids.** The stage's §2–§7 give the counts of
   `schema_check_draft2.md` on its 160 grids (640 of 640 inside corners, every slab
   and slab end, no false finding). Its §2–§7 and §9 give the counts of
   `schema_v0_2_match_d2_out.txt` on its 640 grids, blur2's included.
2. **Frame and truth source.** A9's recall, recomputed from each map's segments with
   M0's frame and `graph_walls.sdf_wall_faces`, equals the stored value within 1e-9
   on 20 of 20 maps.
3. **X-S1, a decoy truth.** Against the world shifted by (+0.5, +0.5) m, no segment
   matches a face, on 20 of 20 maps. Like X7, it is built to find nothing.

### The predictions

The counts of P-S1 to P-S6 were confirmed by Ali on 17 Oct 2026; P-S7 and P-S8 were
added by Ali the same day.

- **P-S1, inside corners found.** On the five cut1200 maps, at least 18 of the 20
  true inside corners are found.
- **P-S2, no wrong inside corner.** On at least 19 of the 20 maps, no inside corner is
  wrong.
- **P-S3, slabs found.** On the five cut1200 maps, at least 18 of the 20 true slabs
  are found.
- **P-S4, no wrong slab.** On at least 19 of the 20 maps, no pair and no slab end is
  wrong.
- **P-S5, slab ends found.** On the five cut1200 maps, at least 36 of the 40 true
  slab ends are found.
- **P-S6, openings at full coverage.** On at least 4 of the 5 cut1200 maps, there is
  no opening.
- **P-S7, no unmatched-part inside corner.** On at least 19 of the 20 maps, no inside
  corner is unmatched-part.
- **P-S8, no unmatched-part slab.** On at least 19 of the 20 maps, no pair and no slab
  end is unmatched-part.

**Expectations behind them** (not facts about the corpus):

- **P-S1, P-S3, P-S5.** Every true element was found on every synthetic grid within
  A8's tolerance, and on every emulated full map. The bounds allow one miss in ten
  for what no check modelled.
- **P-S2, P-S4.** In this world, perpendicular faces end near their crossing only at
  the 4 inside corners (§5's limit), and opposite-facing faces lie within `s_max` of
  each other only across the 4 slabs (§6). A wrong element therefore needs a segment
  that runs far past its face's end, or a pair across two boxes. A segment that
  matches no face gives an unmatched-part element instead, which P-S7 and P-S8 judge.
  No wrong element occurred on any of the 640 grids, blur2's included, or on any of
  the 80 emulated maps, partial ones included.
- **P-S6.** No complete grid and no emulated full map showed an opening. Earlier cuts
  are not judged: on the emulated maps nearest cut60, 5 of 20 showed one.
- **P-S7, P-S8.** An unmatched-part finding needs a segment that M1 cannot place on
  any world face. A8 places every element of every segment within `tau_m` of a wall
  surface, on 20 of 20 maps (§9 M1). On the grids, only blur2, whose east face lies
  beyond `tau_m`, gave unmatched-part findings: 160 inside corners over its 80 grids,
  and no pair or slab end. The 80 emulated maps gave none.
- **The decoy makes every corner unmatched-part.** Against X-S1's shifted truth, all
  320 inside corners of each grid variant were unmatched-part and none was wrong.
  Since no segment matched, every pair and slab end is unmatched-part too (M3, M4).
  So P-S7 and P-S8 are not empty: a frame or truth that placed no segment would fail
  both.

**Verdicts in integers.** P-S1 and P-S3 hold when found ≥ 18; P-S5 when found ≥ 36;
P-S2 and P-S4 when at most 1 map has a wrong element of that kind; P-S6 when at most
1 cut1200 map has an opening; P-S7 and P-S8 when at most 1 map has an unmatched-part
element of that kind. Duplicates count neither for nor against.

**Reported, not judged.** Every count at every cut, with M6's reasons; duplicates;
unmatched segments; openings at every cut, with M5's faces; S-O2's per-face counts;
every distance named in §9; the axis end states at found corners (§5); the record of
every slab end (§7).

**The stage** follows S13's pattern. Before loading anything it refuses an edited
tree, requires the commit that registers v0.2 to be an ancestor of HEAD, and stores
the registered text verbatim. It finds in that text every number it applies (`rho`,
`eps`, `g`, `tau_c`, `s_max`, `tau_e`, `tau_m`, `tau_end`, the decoy's shift, 1e-9,
18 of 20 for P-S1 and P-S3, 19 of 20 for P-S2, P-S4, P-S7 and P-S8, 36 of 40 for
P-S5, 4 of 5 for P-S6) and refuses on a mismatch. It writes §8's per-map files and
one results file.

---

## 11. Open, before the stage

- **S-O1.** Closed in v0.1.
- **S-O2.** Reported per map through M6, not judged (confirmed by Ali, 17 Oct 2026).
  No synthetic grid put one face on two `lines[]` entries, not even with a 0.2 m jog
  (jog2). The emulated partial maps did so on 2 of 80.
- **S-O3.** Closed by §9.
- **S-O4.** Closed by §10.
- **S-O5, earlier cuts.** Judging found elements before cut1200 needs a rule for "in
  view" that is not built from the schema itself. M6's face counts are not such a
  rule (§9).
- **S-O6, blur beyond A8's tolerance.** A real face blurred past `tau_m` would be
  unmatched. Its corners would count as unmatched-part, against P-S7, and as missed,
  against P-S1 (blur2). A blurred slab face would count against P-S8 and P-S3 the
  same way. G-S1 cannot catch this; P-S7, P-S8, M6 and the reported distances would
  show it.
