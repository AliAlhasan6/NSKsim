# SPEC — B2 divergence on walls (v0.8, draft)

Status: draft for review, 4 Oct 2026. v0.1 folds in the S1 answers. v0.2 changes
T2's condition 3, after C0 failed; v0.3 adds condition 4, after the deep-tail
diagnostic. v0.4: T2 leaves the measure under §7's fallback (C0 failed under
v0.3); v0.5 splits T1's distance test across and along the face. Sections 1–8
are decided. v0.6: P3, P4 and P6 registered on 6 Oct 2026, before any share was
computed, except the four A = k0 cut1200 values seen in t1_across. v0.7: erratum
to P6's quoted seen values; no registered wording changed. v0.8 adds the
viewpoint weight on corroborated elements (O1); it changes no class, no
existing field and no registered wording. No code is written until the
stop-point questions in §11 are answered.

Depends on: `docs/specs/SPEC_b2_wall_predicate.md` (v2) for face, face element,
`rho`, `eps`, `l_min` and decision D1 (a wall node is one observed face).
Origin: `HANDOFF_2026-10-04_b2_wall_graphs_and_final_map_corpus.md` §5.

---

## 1. Purpose

For an ordered pair of robots (A, B) and a cut C, sort every wall face that A
observed by B's evidence:

- **corroborated:** B has a matching face;
- **unobserved:** B never observed that place;
- **contradicted:** B's own scans crossed the place without a return.

Same world is argued from corroboration: at full coverage, B corroborates nearly
all of A's walls. Contradiction is not measured (v0.4).

The measure therefore has two classes, corroborated and not corroborated. The
three-way sort above is what v0.1 to v0.3 attempted; `unobserved` and
`contradicted` are now one class, because only T1 separates them and T1 cannot.

The pair is ordered: A → B judges A's walls by B's evidence, and B → A is a separate
computation. How the two combine is open (§10).

The measure uses no ground truth beyond the poses that built the maps. Truth is used
only to validate the result (§8).

## 2. Inputs

For robot K in 0–4 and cut C in {60, 120, 240, 1200} s:

| Input | Path |
|---|---|
| Map | `experiments/maps/b2maps_k{K}_cut{C}_gated_extfix_robot{K}.pgm/.yaml` |
| Wall graph | `experiments/logs/graph_walls/b2maps_k{K}_cut{C}_gated_extfix_robot{K}.json` |
| Rays | robot K's scans in `experiments/logs/b2maps/b2maps_k{K}_cut{C}_gated_slamin` |

Robot K's map comes from run K, in which robot K explores and the other four stay
parked. A pair (A, B) therefore compares the explorers of two different runs. A
pair may take A and B at different cuts; P3 uses A at cut1200.

**Common frame.** Spawn registration from `DOT_POSES` at `08617b2`: translation only,
yaw 0, given by design and never fitted. With each map frame anchored at its robot's
spawn `s_K`, a point in B's frame maps into A's frame as `x_A = x_B + s_B − s_A`.
The transform is applied in continuous coordinates. No grid is resampled, because the
two lattices are generally offset by a fraction of a cell.

## 3. The unit

The unit is one face element of A, as defined in the wall-predicate spec: an oriented
boundary between an occupied cell `o` and a free cell `f`, length `rho = 0.1 m`,
normal `n` into free space, midpoint `m`.

The JSON stores segments only, so elements are always recomputed with
`graph_walls.graph_and_elements(stem)` (`graph_walls.py:1226`), with `g` passed
explicitly from the JSON's `thresholds.g` and never re-derived from the nav2 params.
The map's sha256 must equal the JSON's `map_sha256`, or the tool stops. `seg_of` gives
each element's face.

Only elements that belong to a face of A's wall graph are classified. Elements
outside every face (fragments under `l_min`, parked robots, anything unclassified) are
not, and their total length is reported (§6) so that the excluded length stays
visible.

Every element has length `rho`, so shares by length equal shares by count. How two
maps split a face into segments does not matter, and mixed evidence along one face
becomes a mix of shares on that face.

## 4. Classification

The tests run in order, and an element takes the first class that applies.

**T1 — corroborated.** B has a face element `e'`, from B's wall graph and moved into
A's frame, with

- the same `dir` as `e`. Element normals are axis-aligned by construction
  (`graph_walls.py:308-313`) and the transform is a pure translation, so this is
  exact. Both normals come from `face_elements()`, never from `segments[].normal`,
  which is a fitted value,
- `|(m' − m)·n| ≤ eps = rho(1 + √2/2) = 0.1707 m` across the face, and
  `|(m' − m)·t| ≤ rho/2 = 0.05 m` along it, where `t = (-n_y, n_x)`. Both
  inclusive, with a 1e-9 m tolerance. Changed in v0.5: a Euclidean `eps`
  reaches the NEXT element along a face, `rho = 0.1 m` away, so corroboration
  ran one element past the end of B's coverage. The across tolerance is
  unchanged, and the `t1_across` diagnostic
  (`experiments/logs/divergence/t1_across_20261005T214716Z.json`) held its
  printed prediction on all four pairs: a near mode under 0.25 m, a far mode
  beyond 0.5 m, and 0.07-0.36 % between them against a 1 % bound.

Only elements of B's faces count, not every boundary in B's map. Corroboration is
face to face (D1), and B's unclassified elements include its own parked robots.

**Viewpoint weight (O1).** A corroborated element says B has a face there. It does
not say B saw it from anywhere A did not. The weight below is how far apart the two
robots' viewing directions were, and it is reported beside the corroborated share,
never in place of it.

For a corroborated element `e` of A, let `S_A` be the segment `e` belongs to and
`S_B` the segment of the B element that T1 matched to `e`. Let `p` and `q` be the
`bearing_hist` of `S_A` and `S_B`, each divided by its own sum (72 bins of 5°,
`graph_walls.BEARING_BINS`). Bearings are sensor-to-hit directions in each
robot's own map frame (`graph_walls.py:1544`); registration is translation only,
so A's and B's bearings compare without rotation. Then

    w(e) = W1(p, q) / 180°

where `W1` is the earth-mover distance on the circle: `D_k = sum_{j<=k} (p_j - q_j)`
for `k = 0..71`, and `W1 = 5° * sum_k |D_k - a|` with `a` a median of `{D_k}`. `w`
lies in [0, 1]: 0 when A and B saw the face from the same directions, rising with
the angle between their views, 1 when they are 180° apart.

T1 records only THAT a match exists, not which element matched. The B element taken
here is the one with the smallest across distance; ties go to the smallest along
distance, then to the lowest B segment id.

**Unweighable.** `e` is corroborated but `p` or `q` sums to zero, or the matched B
element has no segment. Such an `e` gets no weight. Its length is reported as
`unweighable_A` (A's histogram empty) or `unweighable_B` (B's histogram empty, or
no B segment).

**The weighted share.** `W` is the sum of `w(e) * rho` over weighable corroborated
`e`, divided by the corroborated share's own denominator, A's elements assigned to
faces. So `W <= C`, unweighable elements contribute 0, and `W` is a lower bound.

Per row, also reported: `w_mean`, the length-weighted mean of `w` over weighable
corroborated elements; `w_p10`, `w_p50`, `w_p90`; and `unweighable_A` and
`unweighable_B`, both as shares of the same denominator.

**Known limit, accepted.** `bearing_hist` counts returns, not scans, so a close pass
weighs more than a distant one. Per-scan counts are not stored.

`W` is reported beside `C` and never replaces it. P3, P4 and P6 stay on `C`.
Predictions on `W` are registered in a separate commit, before any corpus run.

**T2 (removed from the measure in v0.4; kept as a record).** B's admitted rays
satisfy all four:

1. **Crossing.** The ray crosses the element's segment (closed, length `rho`) while
   travelling against the normal (`d · n < 0`), that is, from the free side that A
   observed.
2. **Not grazing.** The angle between the ray and the face line is at least
   `theta_g = 30°`, that is, `|d · n| ≥ sin theta_g`.
3. **Runs on.** If the crossing is at range `s`, the ray reaches a depth of at
   least `eps` behind the face line with no hit before it: `(R - s)·|d · n| ≥ eps`
   for a beam with a return at range `R`, and `(L - s)·|d · n| ≥ eps` for a
   relay-filled beam, whose free length `L` is the one Karto clears
   (`max_laser_range`). Changed in v0.2, after C0 failed: v0.1 measured `eps` along
   the ray, but A's own wall surface can lie a cell or more behind the face line, so
   a ray at 30° could travel 0.2 m past the face and still hit A's own wall. Post
   hoc, with that physical reason.
4. **Weight of evidence.** At least 2 of B's rays meet conditions 1-3 at the
   element, and they are more than half of B's rays that meet conditions 1 and 2
   there. Added in v0.3, after the deep-tail diagnostic: on A's own maps, 4-10 % of
   crossings run on by metres for a reason not yet found (turning and wall-box
   joints were tested and ruled out), so one ray is not evidence that B saw open
   space. The majority threshold is chosen on principle, not calibrated. Post hoc.

**T3 — not corroborated.** Everything else. Under v0.4 this is simply the
complement of T1, and it absorbs what T2 would have called contradicted.

Why corroboration goes first: if B saw the same face, a ray slipping past an
element's edge or through a corner is a discretisation artefact, not evidence that the
wall is absent.

Why only the free side counts: the face is what A saw from its free side. A ray that
reaches it from behind is evidence about the opposite face, if A has one.

**The rays.** The exact ray set that built B's map at cut C: the same scan admission
(the first-scan drop, slam_toolbox's discard of scans 2–4, the travel gate), the same
poses, the same sensor offset and the same relay ceiling. They come from the Karto
tracer in `diagnose_karto.py`, not from a re-implementation, and never from B's map.
The tracer supplies each ray's origin, direction, free length and hit. Crossings are
then tested against A's elements in continuous geometry.
`diagnose_karto.rebuild()` reads the ungated bag today (`diagnose_karto.py:388`). It
must read through `bag_variant_of`, so that a `_gated_extfix` stem reads
`_gated_slamin`, while stems of the old corpus read exactly as before.

**The crossing walk.** Each of B's rays, moved into A's frame, is walked through A's
lattice, not B's, because the two lattices are offset by a fraction of a cell. The
walk must visit every cell the ray touches (a supercover traversal, such as
Amanatides-Woo): Karto's stepping can pass a corner diagonally, and a skipped cell is
a missed crossing. Elements are keyed by their free cell `f`, since a ray from the
free side is in `f` just before it crosses. Candidates then take the exact tests 1-3
in continuous geometry. Because the walk depends on A's lattice, it is redone per
pair, using `rebuild()`'s chunking. C0 runs this same code with B = A.

Cuts are prefixes of one bag, so each robot's rays can be traced once at cut1200 and
sliced by scan time for the shorter cuts.

## 5. Parked robots

No special rule. Parked robots are unclassified in graph_walls, so they are neither
A's faces nor B's corroborating faces. A parked robot in B's run stops B's rays where
it stands, so a wall it shadows gets no crossings and falls to unobserved, never to
contradicted.

## 6. Outputs

Tool: `experiments/analysis/divergence_walls.py`. Tests: `test_divergence_walls.py`,
next to `test_graph_walls.py`. Results: `experiments/logs/divergence/`.

Per ordered pair (A, B) and cut C (20 pairs × 4 cuts = 80 rows):

- `L_A`, the total length of A's classified elements, and `L_excl`, A's element
  length outside faces;
- the two shares, corroborated and not corroborated, summing to 1;
- per face: face id and the two lengths, so that a gap in coverage can be located.

The T2 outputs are produced only with the T2 flag on, which is off by default
(v0.4). They are, unchanged from v0.3: per face the three lengths; per
contradicted element the number of rays meeting conditions 1 and 2 and the number
meeting 1-3, the crossing angle, and the distance to the nearest face end; per
pair the deep share, crossings meeting conditions 1-3 over crossings meeting 1 and
2; and the contradicted share at `theta_g` = 15, 20, 25, 30, 35, 40 and 45°.

Each tool that writes a result file writes a new, dated file. Nothing overwrites an
earlier result.

## 7. Gates, control and named breaks

All three below are printed before any cross-robot number.

**G0 — ray set.** Rebuild each map from the tracer's admitted rays, with the patched
extent (`Round(size/res) + 1` per axis) and `trinary_map.CELL_CENTRE_OFFSET`. It must
match the extfix map exactly, or within 5 cells, which is the standard
`diagnose_karto.py` met on the old corpus. Any map beyond 5 cells stops the run.

**G1 — frame.** Every parked robot visible in B's map, moved into A's frame, must
land within 0.15 m of its spawn as predicted by `s_X − s_A`. This fails if the
transform's sign or anchor is wrong.

**C0 — the contradiction test's floor.** Apply T2 alone (T1 switched off) to A's own
rays against A's own faces. This measures how often the test fires on a map against
the rays that built it. Reported per map. If C0 exceeds 1 % of `L_A` on any map at
`theta_g = 30°`, stop for review before any cross-robot number is read. If C0 fails
under v0.3, T2 leaves the measure: elements are reported as corroborated or not, and
the deep tail as a stated limitation.

**Named breaks.** Each must be shown failing, as a test:

| Break | Expected effect |
|---|---|
| X1: frame sign flipped (`x_B − s_B + s_A`) | G1 fails; corroborated share collapses on overlapping pairs |
| X2: a synthetic 1.0 m face (10 elements) added to A in a strip that is free in both maps and at least 1 m from any occupied cell | at least 90 % of its length contradicted |
| X3: B given no rays | contradicted share exactly 0; unobserved = 1 − corroborated |
| X4: B given no faces | corroborated share exactly 0 |
| X5: `theta_g` swept 0–45° | contradicted share non-increasing in `theta_g` |
| X6: b2maps_k1_cut1200 rebuilt from the ungated bag | fails G0 |

Synthetic unit cases, on small hand-built grids:

- A and B see the same wall → corroborated;
- a ray passes through a wall that only A has → contradicted;
- the same ray at 20° to the face line → unobserved;
- a ray stopped by an occluder before the wall → unobserved;
- a ray whose hit lies within `eps` beyond the face → unobserved;
- a one-cell wall seen from opposite sides by A and B → each side unobserved by the
  other, never contradicted;
- a ray at 45° crossing an element within 0.01 m of its end → contradicted
  (the walk must find it);
- the cases above with B's lattice offset from A's by (0.03, 0.07) m → the same
  classes as with no offset.

## 8. Validation against truth

This is the only use of truth, and it is diagnosis, not part of the measure. In the
world frame (`s_A` added), the share of corroborated length lying within `eps` of a
wall surface in `knowledge_world.sdf` at `08617b2` is reported. Each contradicted
element is located against truth as well: on a real wall (B's evidence is wrong) or in
open space (A's face is spurious).

## 9. Pre-registrations

P3, P4 and P6 are REGISTERED: accepted and committed on 6 Oct 2026, and never
reworded afterwards. P1, P2 and P5 are void, kept as a record of what was tried,
not as live predictions.

- **P1. VOID (v0.4).** C0 is at most 1 % of `L_A` on every map at `theta_g = 30°`.
  Void because it was tested and failed: 0/20 under v0.1, 4/20 under v0.3. It is
  the failure that removed T2, so it cannot also be a prediction about it.
- **P2. VOID (v0.4).** Same world at full coverage: contradicted share at most 1 %
  for every ordered pair at cut1200. Void because the contradicted share is no
  longer measured.
- **P3. REGISTERED 6 Oct 2026.** Convergence, A held at cut1200: for every ordered
  pair, the not-corroborated share of A's cut1200 walls is non-increasing as B's
  cut goes 60 → 120 → 240 → 1200. A rise of any size at any step counts as a
  failure and is shown. For A = k0, the value at B's cut1200 was seen. The
  same-cut series (A and B at the same cut) is reported, not tested.
- **P4. REGISTERED 6 Oct 2026.** Truth check: for every pair at every cut, at
  least 95 % of corroborated length lies within eps of a wall surface in
  knowledge_world.sdf at 08617b2.
- **P5. VOID (v0.4).** At cut1200, the contradicted share changes by at most
  0.5 pp across `theta_g` from 15° to 45°, for every pair, so the result does not
  hinge on 30°. Void on two counts: the contradicted share is no longer measured,
  and the sweep it is stated over is not even monotone under v0.3 (8/20 maps).
- **P6. REGISTERED 6 Oct 2026.** Same world at full coverage: for each of the 16
  ordered pairs with A ≠ k0, the corroborated share at cut1200 (A and B both at
  cut1200) is at least 90 %. The four pairs with A = k0 were seen in the
  t1_across diagnostic (93.8–94.3 %); they are reported, labelled seen, and not
  part of the test. The 90 % is informed by them.

Erratum to P6 (6 Oct 2026; P6's registered wording is unchanged). The four seen
values P6 quotes came from the t1_across diagnostic, whose scratch script divided
by all 1,377 of A's elements instead of the 1,320 assigned to faces. The v0.5
corroborated shares actually seen were 97.80-98.33 %, not 93.8-94.3 %, so the
90 % threshold sat about 8 points below them, not 4. P6 tests only the 16 pairs
with A ≠ k0, so its verdict does not depend on this. Found by the cross-check in
s4_predictions_20261005T230035Z.json.

## 10. Open, not decided

- **O1.** Viewpoint weighting. Corroboration cannot be weighted by the number of
  observers (schema handoff §3.3); decided in v0.8, see §4 "Viewpoint weight
  (O1)".
- **O2.** Faces with empty bearing histograms: 4 of 220 on the old corpus. Recount on
  the extfix corpus; treatment undecided.
- **O3.** A symmetric summary of A → B and B → A, and the series over cuts.
- **O4.** Whether to report separately the unobserved length that parked robots
  shadow.
- **O5.** The deep tail, unexplained. On A's own maps 4-10 % of crossings that meet
  conditions 1 and 2 run on well past the face, and the distribution is bimodal:
  under 0.2 m or over 1 m, with almost nothing between. Turning (yaw rate at the
  scan) and wall-box joints were each pre-registered and each ruled out. Condition 4
  is a guard against it, not an explanation of it.

## 11. Stop points for Claude Code

**S1 — read-only questions, answered before any code:** Answered 4 Oct 2026; folded
into v0.1.

1. Does each graph JSON store the per-face elements (midpoint, normal, cell)? If not,
   which graph_walls function produces them, and does reusing it reproduce the JSON's
   face set exactly?
2. Are element normals axis-aligned by construction?
3. Does the tracer in `diagnose_karto.py` use `CELL_CENTRE_OFFSET` and the patched
   extent? Where does it apply the travel gate, the scan 2–4 discard and the
   first-scan drop?
4. Is each robot's map frame anchored at its spawn, so that `x_world = x_map + s_K`?
   Cite the line.
5. Estimate the wall time for T2 on one pair at cut1200 (about 1 M rays against a few
   thousand elements; brute force will not do).

**S2.** The unit cases and X1–X5 pass on synthetic data.

**S3.** G0, G1 and C0 printed on the real corpus, and nothing else.

**S4.** Cross-robot numbers only after the pre-registrations are committed.
