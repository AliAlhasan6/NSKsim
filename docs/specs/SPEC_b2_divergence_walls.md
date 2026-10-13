# SPEC — B2 divergence on walls (v0.20, draft)

Status: draft for review, 4 Oct 2026. v0.1 folds in the S1 answers. v0.2 changes
T2's condition 3, after C0 failed; v0.3 adds condition 4, after the deep-tail
diagnostic. v0.4: T2 leaves the measure under §7's fallback (C0 failed under
v0.3); v0.5 splits T1's distance test across and along the face. Sections 1–8
are decided. v0.6: P3, P4 and P6 registered on 6 Oct 2026, before any share was
computed, except the four A = k0 cut1200 values seen in t1_across. v0.7: erratum
to P6's quoted seen values; no registered wording changed. v0.8 adds the
viewpoint weight on corroborated elements (O1); it changes no class, no
existing field and no registered wording. v0.9 registers G3, P7 and P8 on the
viewpoint weight, before any of W, w, w_null or an unweighable share was
computed on a real map. v0.10 records the S5 results: G3, P7 and P8 all hold,
and O1 is closed. v0.11 decides O3's symmetric form and the convergence series,
and registers P9, before any row P9 tests was computed. v0.12 records the S6
results — G4 holds, 820/820, and P9 fails, 40 of 60 series — and registers P10 on
losses between B's cuts, before any gain or loss was computed. v0.13 records the
S7 results — G5 holds and P10 holds, 20 of 20 series losing an element — and
registers P11 on the cell states behind those losses, before any cell state at
B's later cut was read for a lost element. v0.14 records the S8 results — G6
holds and P11 fails, 21 of 213 wall cells freed — and registers P12 on whether
the 132 unchanged losses are segmentation or geometry, before any segment
assignment was read for an unchanged element. v0.15 records the S9 results —
G7 holds on all four parts and P12 holds, 75 of 78 unchanged losses in
segmentation — and closes O7 and opens O8. It registers no prediction: the
chain stopped at S9 by decision on 10 Oct 2026. v0.16 reverses that decision,
reopens O8, and registers P13a, P13b and P14 on whether the segment b belonged
to at k is intact at k', before any member of that segment was read at k'.
v0.17 records the S10 results — G8 holds on all three checks, P13a fails, 2 of
5 segments changed, and P13b holds, 12 of 14 intact, and P14 holds, 0 of 22
flanked — and registers P15 and P16 on which rule of the extraction drops each
e', before any class was computed for any e'. v0.18 records the S11 results —
G9 holds, P15 fails, 1 of 22 in R3, and P16 holds, 0 of 22 in Other — and a
post-hoc reading that all 22 elements are risers with respect to H's refitted
line. It closes O8, opens O9 and registers no prediction. v0.19 registers P17,
mirror agreement of the viewpoint weight at mixed cuts (O3), and G10, the gate
for stage S12. S12 will also record the distinct (S_A, S_B) pairs per row (O6).
v0.20 records the S12 results — G10 holds, P17 holds 60 of 60 — and a post-hoc
reading that the two rows of every mirror pair hold the same matched segment
pairs. It closes O3 and O6 and registers no prediction. No code is written until
the stop-point questions in §11 are answered.

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

### Registered at v0.9: the O1 viewpoint weight

Registered in a commit that precedes the commit of the code that tests them, and
before W, w, w_null or any unweighable share was computed on any real map.
Disclosure: Step 0 of the O1 work observed that all 13 segments of k0_cut1200 have
non-empty bearing histograms. No histogram content was seen beyond that.

Terms are those of the section "Viewpoint weight (O1)". Pairs are the 20 ordered
pairs (A, B) of distinct robots k0..k4.

**G3 (gate).** For each of the 20 maps, the map paired with itself gives C = 1, and
w = 0 exactly on every weighable corroborated element. If G3 fails on any map, P7
and P8 are not evaluated in that run.

**P7.** A and B both at cut1200. For a weighable corroborated element e of A, let
w_null(e) be the mean of W1(p, q_S) / 180° over every B segment S that has the same
direction as S_B, is not S_B, and has a non-empty histogram. A segment's direction
is the T1 element dir of the elements assigned to it, one of the four axis-aligned
normals; if they differ, the commonest, ties going to the dir nearest the segment's
fitted normal. E7 is the set of such e for which at least one such S exists. Over
E7, the length-weighted mean of w is strictly lower than the length-weighted mean
of w_null. P7 holds if this is true for at least 18 of the 20 pairs. A pair with E7
empty counts against P7.

**P8.** A at cut1200. w_mean with B at cut1200 is strictly lower than w_mean with B
at cut60. P8 holds if this is true for at least 16 of the 20 pairs. A pair with no
weighable corroborated element at either cut counts against P8.

**Reported, not tested.** For every row with A at cut1200 and B at each cut, and for
the same-cut series: W, w_mean, w_p10, w_p50, w_p90, unweighable_A, unweighable_B
and the length of E7; and for each map, the number of segments with an empty
histogram (O2). No level of w is predicted: bearing_hist counts returns, so a robot
standing still can dominate a face's histogram.

### Results: G3, P7, P8

Source: `experiments/logs/divergence/s5_viewpoint_20261006T212758Z.json`, sha256
`f99e001d517734ea4b8d737436f0e9243122c0edd9d30f65a241dde39e89bb67`. The file is
not tracked — `experiments/logs/` is gitignored — so the hash stands in for it.
Produced at HEAD `2161665`, the commit that added the stage, against a clean tree.
Every number here was read from that file by script.

**G3 — HOLD, 20/20.** Every map against itself gives `C` exactly 1.0 and
`max |w|` exactly 0.0, on all 20. Not approximately: the two histograms are the
same object, so the transport cost between them is a subtraction that yields zero.

**P7 — HOLD, 19/20, 18 required.** The one failure is A = k1, B = k3: mean `w`
0.058794 against mean `w_null` 0.057730, a difference of −0.001064, which is
10.583° against 10.391°, or −0.191°. On that pair the null is simply as close as
the match.

**P8 — HOLD, 20/20, 16 required.** `w_mean` with B at cut1200 runs 0.0295 to
0.0588 (5.32° to 10.58°); with B at cut60, 0.0842 to 0.1540 (15.16° to 27.71°).
The smallest ratio of the two columns is 1.861, at A = k3, B = k1 — so even the
weakest pair halves its viewpoint gap between cut60 and cut1200.

**Reported, not tested.** At cut1200, same cut, over the 20 ordered pairs: mean
`w` 0.0295 to 0.0588 (5.32° to 10.58°); mean `w_null` 0.0522 to 0.0704 (9.39° to
12.68°); `W` 0.0289 to 0.0579; `w_p10` 0.0075 to 0.0227; `w_p50` 0.0185 to 0.0478;
`w_p90` 0.0399 to 0.1136; `unweighable_A` and `unweighable_B` both 0.0 on every
row; |E7| 1286 to 1298. Per map (O2), segments with an empty histogram: 1 in the
whole corpus, on `b2maps_k3_cut1200_gated_extfix_robot3`, and 0 on the other 19.
Segment counts run 4–7 at cut60, 7–8 at cut120, 9–12 at cut240 and 12–13 at
cut1200.

**Caution 1 — mirror pairs.** The 20 ordered pairs are not 20 independent tests.
Over the 10 unordered pairs at cut1200 the largest `|w_mean(A,B) − w_mean(B,A)|`
is 0.000016, which is 0.003°. `w` is symmetric in its two histograms, so a mirror
can differ only through which segments were matched; here it barely does. Within a
mirror, P7's verdict can differ only through `w_null`, which is the asymmetric
half. Read the 20 rows as 10 pairs of near-duplicates.

**Caution 2 — the sample.** `w` is constant within a matched segment pair: every
element of A's face S_A that matched B's face S_B gets the same weight. The
effective sample is therefore the number of distinct `(S_A, S_B)` pairs in E7, not
|E7|. That count is NOT recorded in the result file, so it cannot be read from it;
what the file does bound is its size. The cut1200 maps carry 12 or 13 segments
each, so there are at most 169 distinct pairs against an |E7| of 1286 to 1298 — at
least a sevenfold overstatement if |E7| is read as a sample size, and in practice
far more, since most of A's faces match one B face. Recording the pairs is the
first thing a rerun of S5 should add.

**Reading.** At full coverage, B's views of a wall sit on average within 5.32° to
10.58° of A's, and other same-facing walls only slightly further at 9.39° to
12.68°, so corroboration rests on closely aligned viewpoints.

## Symmetric summary and convergence series (O3)

**Rows already seen.** The full cut matrix has 320 rows: 20 ordered pairs × 4 cuts
for A × 4 cuts for B. S4 (`s4_predictions_20261005T230035Z.json`, at `9eb3f17`)
computed C for 140 of them:
- A at cut1200 against every B cut (P3, 80 rows);
- every same-cut row (80 rows, 20 ordered pairs at each cut).
The two sets share the 20 rows where both maps are at cut1200. No result file in
`experiments/logs/divergence/` holds a row with A at 60, 120 or 240 and B at a
different cut. Those 180 rows are unseen.

**The symmetric form.** Take an unordered pair {X, Y} and cuts a and b. The two
directions, X@a → Y@b and Y@b → X@a, are reported side by side with their
difference. No mean and no minimum is reported, because when one map is much smaller
the two directions measure different things:
- the larger map against the smaller measures the smaller map's coverage;
- the smaller map against the larger measures whether the smaller map's walls hold up.

**The convergence series.** Convergence is presented as fixed-A series: A is held at
one cut while B's cut grows 60 → 120 → 240 → 1200. The same-cut series is still
reported, but it is not a convergence curve, because A's wall set grows under the
measure.

**What is computed.** Stage S6 computes C for all 320 rows of the cut matrix, with
the measure P3 tested, unchanged.
- **G4.** S6 recomputes the 140 seen rows and requires each to equal S4's value
  exactly, and S5's value wherever S5 holds that row. On any difference it stops
  before writing a verdict.
- **Each row** records A, B, cut_A, cut_B, L_A_m, corroborated, not_corroborated and
  seen.
- **The symmetric table** records, for each unordered pair and each (a, b), both
  directions and their difference.
- S6 refuses to run on an edited tree.

**What is not computed.** S6 computes no viewpoint weight. Every weight S5 computed
lies in the 140 seen rows or on a map against itself, so on the 180 unseen rows the
weight is unseen too, and a prediction about its mirror agreement at mixed cuts can
still be registered.

**P9. REGISTERED 8 Oct 2026.** Convergence, A held at an early cut: for every ordered
pair and for each of A's cuts 60, 120 and 240, the not-corroborated share of A's walls
at that cut is non-increasing as B's cut goes 60 → 120 → 240 → 1200. That is 60
series of four values. A rise of any size at any step counts as a failure and is
shown. With A fixed, the denominator is constant, so a rise is a rise in
not-corroborated length. A series whose A walls have zero length is undefined and
counts as a failure. In each series, the value with B at A's own cut was seen (S4's
same-cut rows); the other three were not, and together they are the 180 unseen rows.
The level is reported, not predicted.

### S6 results

Source: `experiments/logs/divergence/s6_cutmatrix_20261008T225604Z.json`, sha256
`a3d584f310e509c33bcca09c2fe641d4914dd4f768146f10c1ebf340f58b2f13`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `ce8dd6e`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

**G4 — HOLD, 820/820.** Every stored seen value equals S6's exactly, with no
tolerance: S4 420/420, S5 400/400. The comparisons name 140 distinct rows, which is
every one of the 140 seen rows, so G4 gates the whole seen half of the matrix rather
than a sample of it. S4 ran from an uncommitted script, so G4 is the first
reproduction of S4's values from committed code.

**P9 — FAIL, 40/60 series.** 20 of the 60 series rise at some step, over 21 failing
steps in all. No series was undefined.

| A | B | cut_A | B's step | at the earlier cut | at the later cut | rise | rise (m) |
|---|---|---|---|---|---|---|---|
| k0 | k1 | 60 | 60 → 120 | 0.024096 | 0.028112 | 0.004016 | 0.1000 |
| k0 | k2 | 120 | 120 → 240 | 0.003610 | 0.007220 | 0.003610 | 0.2000 |
| k0 | k3 | 120 | 240 → 1200 | 0.007220 | 0.009025 | 0.001805 | 0.1000 |
| k0 | k4 | 60 | 120 → 240 | 0.012048 | 0.020080 | 0.008032 | 0.2000 |
| k1 | k2 | 120 | 120 → 240 | 0.007286 | 0.009107 | 0.001821 | 0.1000 |
| k1 | k2 | 120 | 240 → 1200 | 0.009107 | 0.010929 | 0.001821 | 0.1000 |
| k1 | k2 | 240 | 240 → 1200 | 0.011403 | 0.017104 | 0.005701 | 0.5000 |
| k1 | k3 | 120 | 240 → 1200 | 0.010929 | 0.012750 | 0.001821 | 0.1000 |
| k1 | k4 | 60 | 120 → 240 | 0.004032 | 0.008065 | 0.004032 | 0.1000 |
| k2 | k0 | 60 | 120 → 240 | 0.031088 | 0.033679 | 0.002591 | 0.1000 |
| k2 | k0 | 120 | 240 → 1200 | 0.013889 | 0.015625 | 0.001736 | 0.1000 |
| k2 | k1 | 60 | 120 → 240 | 0.028497 | 0.031088 | 0.002591 | 0.1000 |
| k2 | k1 | 120 | 240 → 1200 | 0.012153 | 0.013889 | 0.001736 | 0.1000 |
| k2 | k3 | 60 | 240 → 1200 | 0.028497 | 0.031088 | 0.002591 | 0.1000 |
| k2 | k3 | 120 | 240 → 1200 | 0.015625 | 0.017361 | 0.001736 | 0.1000 |
| k3 | k1 | 60 | 60 → 120 | 0.417062 | 0.419431 | 0.002370 | 0.1000 |
| k3 | k2 | 60 | 60 → 120 | 0.417062 | 0.419431 | 0.002370 | 0.1000 |
| k4 | k0 | 120 | 60 → 120 | 0.574394 | 0.576125 | 0.001730 | 0.1000 |
| k4 | k1 | 60 | 60 → 120 | 0.430876 | 0.437788 | 0.006912 | 0.3000 |
| k4 | k1 | 120 | 60 → 120 | 0.572664 | 0.577855 | 0.005190 | 0.3000 |
| k4 | k2 | 120 | 60 → 120 | 0.576125 | 0.577855 | 0.001730 | 0.1000 |

Every rise is a whole number of 0.1 m elements — 1, 2, 3 or 5 of them. The rise in
metres divides by the element length to an integer within 1e-9 m on all 21 steps, and
that integer equals the drop in the corroborated count between the two rows, which is
exact integer arithmetic rather than a tolerance. Verified by script.

**Not-corroborated share, min–max over the 20 ordered pairs.** A cell marked `*` was
unseen before S6.

| A's cut | B at 60 | B at 120 | B at 240 | B at 1200 |
|---|---|---|---|---|
| 60 | 0.0040–0.4401 | 0.0040–0.4378 `*` | 0.0081–0.4332 `*` | 0.0081–0.0576 `*` |
| 120 | 0.3169–0.6217 `*` | 0.0036–0.5779 | 0.0072–0.4705 `*` | 0.0072–0.0174 `*` |
| 240 | 0.4982–0.7540 `*` | 0.2999–0.6724 `*` | 0.0114–0.5416 | 0.0073–0.0323 `*` |
| 1200 | 0.6805–0.8186 | 0.5081–0.5970 | 0.2587–0.3903 | 0.0054–0.0409 |

The verdict stands as registered.

## Gains and losses (P10)

**Why.** P9 failed: 20 of its 60 series rise at some step. With A fixed, a rise in
the not-corroborated share means at least one of A's elements corroborated by B at
the earlier cut is not corroborated by B at the later cut. P3's 20 series held, but
a step's net change can hide losses behind larger gains. P10 asks whether it did.

**Definitions.** Take one series, A fixed at one cut and B's cut going
60 → 120 → 240 → 1200, and one step from B's cut k to the next cut k'. An element of
A is
- a **gain** if B at k does not corroborate it and B at k' does;
- a **loss** if B at k corroborates it and B at k' does not.

**What is computed.** Stage S7 takes the 80 fixed-A series (P3's 20, with A at
cut1200, and P9's 60). For each element of A, it computes the element's
corroboration at every B cut, using the element masks that the measure itself
computes, not a reimplementation. From those it counts gains and losses per step, as
element counts and in metres.
- **G5.** S7 recomputes the corroborated share of all 320 rows from its element masks
  and requires each to equal S6's value exactly. On any difference it stops before
  writing a verdict, and writes no gain or loss.
- **Reported, not tested.** For each lost element, the across and along distances to
  the nearest same-direction element of B at k'.
- S7 computes no viewpoint weight and refuses to run on an edited tree.

**P10. REGISTERED 9 Oct 2026.** Losses under P3's held series: in at least 10 of P3's
20 series (A at cut1200), at least one step contains at least one loss. A loss of one
element counts. The threshold is informed by P9's result, which showed net rises in
20 of its 60 series. The size of the losses is reported, not predicted.

### S7 results

Source: `experiments/logs/divergence/s7_gainloss_20261009T000606Z.json`, sha256
`b8b7cb6c4dc57e8df48ea9baa975dc2e85b8eddbeed7e75c71fe52a271461c21`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `1cef039`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

**G5 — HOLD.** The corroborated share recomputed from each element mask equals S6's
exactly on all 320/320 rows, and losses minus gains equals S6's rise in elements on
all 21/21 of S6's failing P9 steps.

**P10 — HOLD, 20 of 20 series** with at least one loss, 10 required. Every one of
P3's series loses at least one element at some step.

**P3's 20 series, A at cut1200.** Gains and losses in elements, per step.

| A | B | B: cut60 → cut120 | B: cut120 → cut240 | B: cut240 → cut1200 |
|---|---|---|---|---|
| k0 | k1 | +299 / −2 | +323 / −0 | +427 / −1 |
| k0 | k2 | +193 / −1 | +390 / −0 | +341 / −7 |
| k0 | k3 | +224 / −0 | +324 / −0 | +337 / −2 |
| k0 | k4 | +161 / −2 | +254 / −3 | +472 / −0 |
| k1 | k0 | +308 / −2 | +405 / −0 | +338 / −1 |
| k1 | k2 | +194 / −1 | +386 / −1 | +337 / −4 |
| k1 | k3 | +224 / −0 | +326 / −0 | +329 / −3 |
| k1 | k4 | +160 / −2 | +254 / −2 | +472 / −0 |
| k2 | k0 | +308 / −3 | +400 / −1 | +343 / −0 |
| k2 | k1 | +299 / −2 | +320 / −1 | +425 / −1 |
| k2 | k3 | +224 / −0 | +323 / −0 | +333 / −2 |
| k2 | k4 | +161 / −1 | +252 / −4 | +470 / −1 |
| k3 | k0 | +307 / −2 | +403 / −0 | +345 / −0 |
| k3 | k1 | +298 / −2 | +321 / −0 | +428 / −0 |
| k3 | k2 | +193 / −1 | +389 / −0 | +343 / −7 |
| k3 | k4 | +161 / −2 | +254 / −3 | +470 / −0 |
| k4 | k0 | +308 / −2 | +404 / −0 | +338 / −0 |
| k4 | k1 | +299 / −2 | +322 / −0 | +427 / −0 |
| k4 | k2 | +193 / −1 | +386 / −0 | +337 / −4 |
| k4 | k3 | +224 / −0 | +324 / −0 | +329 / −2 |

**P9's 60 series, by A's cut.** Totals over the 20 ordered pairs.

| A's cut | gains | losses | series with a loss |
|---|---|---|---|
| 60 | 1902 | 31 | 17/20 |
| 120 | 6181 | 41 | 17/20 |
| 240 | 12341 | 63 | 20/20 |

**Lost elements: 213** — 78 in P3's series and 135 in P9's. They fall into three bins
by how far the nearest same-direction element of B at k' lies from the lost element,
across and along the face, in the common frame T1 works in. Distance is Euclidean, as
the stage measures it, and the bound is 1 m. The bins are disjoint and cover all 213.

| bin | count |
|---|---|
| within eps across (0.1707 m), beyond rho/2 along (0.0500 m) | 102 |
| beyond eps across, within 1 m | 5 |
| none within 1 m | 106 |

Quartiles within bins 1 and 2, in metres.

| bin | axis | Q1 | median | Q3 |
|---|---|---|---|---|
| 1: within eps across | across | 0.0029 | 0.0066 | 0.0156 |
| 1: within eps across | along | 0.0944 | 0.0992 | 0.1036 |
| 2: beyond eps across | across | 0.1850 | 0.2961 | 0.2983 |
| 2: beyond eps across | along | 0.0004 | 0.0006 | 0.0056 |

Every lost element has a same-direction element of B at k' somewhere — 0 do not — and
none has one inside both of T1's tolerances at k' (0), which is the self-check: an
element inside both would have been corroborated.

The verdict stands as registered.

## Cell states under losses (P11)

**Why.** S7 found 213 lost elements: each was corroborated by B at cut k and not at
the next cut k'. B's maps are deterministic replays, so B's map at k' integrates
every scan its map at k integrated, and more. In Karto, a cell's state rests on its
share of hits among the rays that reach it, so a cell occupied at k can be free at k'
once later rays pass through it. B's grid origins also differ between cuts by
fractions of a cell, so a wall cell can be freed by the lattice moving as well as by
later rays. P11 tests the state, not which cause freed it.

**Definitions.** For a lost element e of A at step k → k', let b be the element of B
at k that corroborates e, chosen as `corroborate_matches` chooses: nearest across,
then nearest along, then lowest B segment id. Element b separates two cells of B's
grid at k: its wall cell (occupied) and its front cell (free). Each cell is named by
its centre's position in B's map frame, which is the same at every cut of B. Its
state at k' is the state of the cell of B's grid at k' that contains that position,
found by position, never by index, with the classification `graph_walls` uses to
build face elements. Each lost element falls in one class:
- **off grid**: either cell's position lies outside B's grid at k';
- **wall cell freed**: on grid, and b's wall cell is not occupied at k';
- **front cell closed**: on grid, b's wall cell is still occupied at k' and its front
  cell is not free;
- **unchanged**: on grid, and both cells hold their states at k'.

**What is computed.** Stage S8 takes every lost element in S7's result file
(`s7_gainloss_20261009T000606Z.json`) and computes b, its two cells, their states at
k', and the class.
- **G6.** S8 requires its set of lost elements to equal S7's exactly, and, for every
  one, b's wall cell to be occupied and its front cell free in B's grid at k, read the
  same way. On any difference it stops before writing a verdict, and reads no grid at
  k'.
- **Reported, not tested.** The four classes split by S7's distance bins; for wall
  cells freed, whether the cell is free or unknown at k'; every unchanged element,
  listed; and for every step, the offset between B's lattices at k and k', in cells.
- S8 refuses to run on an edited tree.

**P11. REGISTERED 9 Oct 2026.** Wall cells freed: in at least half of the 213 lost
elements, b's wall cell lies on B's grid at k' and is not occupied there; off-grid
elements count against P11. The threshold is informed by Karto's ratio rule and by
the depth diagnostic, which found wall cells crossed by many rays that hit just
behind them. The split among the other three classes (off grid, front cell
closed, unchanged) is reported, not predicted; off-grid elements still count
against P11.

### S8 results

Source: `experiments/logs/divergence/s8_cellstates_20261009T010602Z.json`, sha256
`f71f4597d3e050417d65d092295b9ccf1ec0337bbf54af39868418da9c6c31c4`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `e81de80`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

**G6 — HOLD.** S8's lost-element set equals S7's exactly, 213/213, and for every one
of the 213/213 b's wall cell is occupied and its front cell free in B's grid at k,
read by position the same way the k' states are read.

**P11 — FAIL, 21 of 213** wall cells freed, 107 required.

**The four classes by S7's distance bin.** Off-grid elements count against P11, and
are in the denominator.

| class | within eps across | beyond eps across | none within 1 m | total |
|---|---|---|---|---|
| off grid | 0 | 0 | 31 | 31 |
| wall cell freed | 15 | 1 | 5 | 21 |
| front cell closed | 11 | 2 | 16 | 29 |
| unchanged | 76 | 2 | 54 | 132 |
| **total** | 102 | 5 | 106 | 213 |

**Wall cells freed: 21.** At k' the wall cell is 12 free, 9 unknown. **Unchanged:
132.** **Off grid: 31.**

Three facts about the 31 off-grid elements, each verified by script: every one is off
the grid by its **front** cell, never its wall cell; every one is at step **cut60 →
cut120**; and every one has **B = k0** or **B = k1**.

**Lattice offsets** over all 15 steps of all five B maps, in cells: |x| from 0.0000
to 0.4138, |y| from 0.0000 to 0.3699. Zero would mean the two lattices coincide at
that step.

The verdict stands as registered.

Erratum to the v0.13 commit message (9 Oct 2026; no registered wording changes).
Its off-grid proxy is given as 6 of 213 lost elements outside B's image box at k',
with a worst margin of 3.3 cm. That box was the ROS reading, `[origin,
origin + n*res]`; under the Karto span this project uses, cell i covering
`[origin + (i − 0.5)*res, origin + (i + 0.5)*res)`, it is 7 of 213 and 2.44 cm. The
spec stated neither figure, so no registered text changes. S8 then measured the
class directly: 31 off grid, all by the front cell.

Erratum to the v0.13 commit message (9 Oct 2026; no registered wording changes). It
gives the lattice offsets between B's cuts as up to 0.175 cells, which came from the
YAML origins alone. S8 measured them over all 15 steps: up to 0.4138 cells in x and
0.3699 in y. The spec stated no figure, so no registered text changes.

## Unchanged losses: segmentation or geometry (P12)

**Why.** S8 found 132 lost elements whose two cells around b hold their states at
k'. So B's grid at k' still has a face element between those two cells, in b's
direction. T1 compares A's elements only with B's elements that are assigned to a
wall segment. An unchanged element is therefore lost in one of two ways: its face
element at k' is not assigned to a segment, or it is assigned but lies outside T1's
tolerances from A's element. On a continuous wall line whose elements are all
assigned, some element lies within half a cell along of A's element, so a geometric
loss along the wall also needs a neighbouring element of that line to be missing or
unassigned. A lattice shift across the wall can, on its own, carry the face beyond
eps.

**Definitions.** For an unchanged lost element e of A at step k → k', let e' be B's
face element at k' whose wall cell and front cell are the two cells S8 read at k'.
Each unchanged element falls in one class:
- **segmentation**: e' is not assigned to a wall segment at k';
- **geometry, across**: e' is assigned and lies beyond eps across from e;
- **geometry, along**: e' is assigned, lies within eps across from e, and lies beyond
  rho/2 along.
Distances are measured in the common frame, as T1 measures them.

**What is computed.** Stage S9 takes S8's 132 unchanged elements
(`s8_cellstates_20261009T010602Z.json`). For each, it finds e' among B's face
elements at k', as `graph_walls` produces them, reads e''s segment assignment, and
computes e''s across and along distances from e.
- **G7.** S9 requires:
  - its set of unchanged elements to equal S8's exactly;
  - e' to exist for every one, in b's direction;
  - no e' to be both assigned and within both of T1's tolerances from e, since that
    would contradict S7's mask.

  On any failure it stops before writing a verdict.
- **Reported, not tested.** The three classes split by S7's distance bin, and, for
  geometric losses, the lattice offset of their step.
- S9 refuses to run on an edited tree.

**P12. REGISTERED 9 Oct 2026.** Segmentation, not geometry: of the 78 unchanged
elements outside S7's "none within 1 m" bin, at least half fall in segmentation. The
54 unchanged elements in that bin are segmentation by construction, because S7's
nearest-element search covers only B's assigned elements. They are reported,
labelled seen, and not part of the test. The rest of the split is reported, not
predicted.

### S9 results

Source: `experiments/logs/divergence/s9_segmentation_20261009T013501Z.json`, sha256
`42c7505992ff8047d069cb7092dcdeaf3e8644193ff36484f3892425d307deab`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `808c4eb`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

**G7 — HOLD, all four parts.** S9's set of unchanged elements equals S8's exactly,
132/132, with both of b's cell states equal on every one; e' exists for all 132/132,
in b's direction; no e' is both assigned and within both of T1's tolerances from e
(0); and no e' in the carve-out is assigned (0). **Part 4 checks the carve-out,**
which is stricter than the registration asked for: the spec calls the 54 "none within
1 m" elements segmentation by construction and carves them out without asking anyone
to verify it. S7's nearest-element search covers only B's assigned elements, so an
assigned e' within a metre would have been found there; an assigned e' in the
carve-out would mean that reasoning is wrong. None is assigned.

**P12 — HOLD, 75 of 78** unchanged losses in segmentation, 39 required.

**The 78 are not P3's 78 losses.** The two counts coincide and name different sets.
P3's 20 series account for 78 of the 213 losses (S7, v0.13); these 78 are the
unchanged losses outside the carve-out — 76 in S7's "within eps across" bin, the
one-cell hole along the wall, and 2 in "beyond eps across", shifted across it.

**The carve-out: 54 of 54** in segmentation, reported and not tested, as registered.

**The three classes by S7's distance bin.**

| class | within eps across | beyond eps across | none within 1 m | total |
|---|---|---|---|---|
| segmentation | 75 | 0 | 54 | 129 |
| geometry, across | 0 | 2 | 0 | 2 |
| geometry, along | 1 | 0 | 0 | 1 |
| **total** | 76 | 2 | 54 | 132 |

The column totals are S8's own unchanged row from v0.14, 76, 2 and 54, which S9 was
not made to reproduce.

**The three geometric losses**, with the lattice offset of their step. All three are
at step **cut60 → cut120**.

| A | B | A's cut | across | along | class | lattice offset (cells) |
|---|---|---|---|---|---|---|
| k3 | k0 | 60 | 0.0675 m | 0.0785 m | along | (+0.0000, +0.3699) |
| k4 | k2 | 120 | 0.1850 m | 0.0247 m | across | (+0.0000, −0.1641) |
| k4 | k3 | 120 | 0.1748 m | 0.0056 m | across | (−0.1576, −0.1679) |

All three sit within `eps + rho = 0.2707 m` across and `rho = 0.1000 m` along: e' is
displaced from e by no more than a cell plus T1's own across tolerance, which is what
a lattice shift of a fraction of a cell can do on its own. None is a wall that moved.

The verdict stands as registered.

## Segments behind the unassigned elements (P13, P14)

**Why.** v0.15 stopped the chain at S9 by decision and left O8 open but not pursued.
That decision is reversed on the same day, for two reasons.

1. A read-only audit of `graph_walls.py` (the audit below) shows that segment
   membership is decided by a greedy pass over the whole map. Whether an element is
   assigned depends on elements far from it. S8 and S9 checked only e''s own two
   cells. So S9's segmentation class does not yet show that these 129 losses happen
   without B's map changing. The map may have changed elsewhere on b's segment.
2. The rest of the schema (openings, corners, back-to-back face pairing) will likely
   read segment ends. If membership can change between cuts while B's map does not,
   openings can appear or vanish for reasons inside our pipeline.

Record: the claim that most losses come from our own segmentation, and not from B's
map changing (§10, O7), rests on e''s two cells only. It is withdrawn until S10
reports.

**Audit of `graph_walls.py`** (read-only, no real data). Path:
`graph_and_elements` → `extract_with_elements` → face elements, Hough peaks, line
candidates, refit, run splitting, minimum length. `seg_of` is −1 for an unassigned
element. An element is assigned only as a member of an accepted run in some peak
iteration. An assigned element is never unassigned.

Rules by which an element ends at −1:

| Rule | What it does | Inputs reach |
|---|---|---|
| R1, vote floor | No live Hough window containing it reaches vote_min = 3.5355 | Whole map |
| R2, one-shot bins | Its seed bin was spent by an earlier iteration | Whole map, greedy order |
| R3, eps band | It lies more than eps = 0.1707 m from the iteration's refitted line | Every candidate along the infinite line |
| R4, facing | Its normal does not face the line's normal | Its own cells |
| R5 + R6, gaps and length | Its run, cut at physical gaps ≥ g = 0.22 m, is shorter than l_min = 0.5 m | The run |
| R7, abort guards | End an iteration early; they feed R2 | Whole map |

R5 and R6 are local to the run and do not depend on the map's origin. R1, R2 and the
line that R3 uses depend on the whole map: one global bin phase per map, the map's
extent, and the greedy order.

One consequence is used by P14. Take an unassigned element e' on the same face line,
with the same direction, strictly between two members of one segment. Its distance
to that segment's line lies between theirs, so it is within eps. It faces the line
as they do. It is alive, since it was never assigned. And it falls inside their
run, with no gap of g or more. So it would have been a member. Under the audit, no
such e' exists.

**Definitions.** The units are S9's 129 segmentation losses (75 one-cell holes, 54
with no assigned element within 1 m), read from
`s9_segmentation_20261009T013501Z.json`. For each, with B's earlier cut k and later
cut k':

- **S_b** is the segment b belongs to at k.
- Every member m of S_b has a wall cell and a front cell. S10 finds each at k' by
  position with `trinary_map.cell_index`, exactly as S8 finds b's cells. For that
  lookup it never uses `trinary_map.sample`. Loading a map with `load_grid` is
  unaffected.
- **Changed:** at k', some member's wall cell is not occupied, or some member's front
  cell is not free, or some member's cell is off grid.
- **Intact:** otherwise. Every element of S_b at k then still exists at k'.
- **Face line:** elements with the same direction and the same across coordinate,
  within 1e-9 m.
- **Flanked:** at k', e' lies strictly between two elements on its face line that
  belong to the same segment.

Counting units. P13a and P13b count distinct (B, k, k', S_b). P14 counts distinct
(B, k, k', e'). Intact, changed and flanked do not depend on A, and S9's 129 rows
repeat 22 distinct events 2 to 14 times, once per A pairing, so a count per loss
would weight each event by how many As it was matched against. Per-loss counts are
reported beside, not judged. An S_b behind losses of both shapes counts in both P13a
and P13b.

**What is computed.**
- **G8 (S10 against S9).** S10 refuses an edited tree, requires this registration's
  commit to be an ancestor of HEAD, and pins S9's result file by sha256
  `42c7505992ff8047d069cb7092dcdeaf3e8644193ff36484f3892425d307deab`. Before
  computing anything new, it checks:
  1. S9's stored values reproduce exactly: all 132 rows and their classes, with the
     129 segmentation losses split 75 and 54, and 22 distinct (B, k, k', e') among
     them (14 and 8).
  2. Every b has seg_of ≥ 0 at k, and every e' has seg_of = −1 at k': 129/129 each.
  3. Every S_b is non-empty and contains b.

  On a gate failure, S10 computes nothing unseen.
- **Reported, not tested.**
  - For each changed loss: which of wall cell freed, front cell closed and off grid
    occurred (more than one can), how many members changed, and the distance along
    the line from e' to the nearest changed member.
  - For every loss: the number of members of S_b at k, and its length.
  - **Base rate.** At each B step (B, k → k') that has at least one of the 129, the
    share of all B's segments at k that are changed at k'. Without it, "changed"
    could simply be what most segments do between cuts.

S10 computes no viewpoint weight, so O3's mixed-cut prediction can still be
registered.

Registered: 10 Oct 2026, before any quantity below was computed on the real maps.

**P13a. REGISTERED 10 Oct 2026.** Of the distinct S_b behind the 54 losses with no
assigned element within 1 m, at least half (rounded up) are changed. Reason: the
audit's most direct rule for this shape is the minimum length. A segment that met
l_min at k can fail it at k' only by losing members.

**P13b. REGISTERED 10 Oct 2026.** Of the distinct S_b behind the 75 one-cell holes,
at least half (rounded up) are intact. Reason: the audit's only exclusion rule for
this shape is R3. Its line is refitted to candidates along the whole infinite line,
so it can move with no member of S_b changing.

**P14. REGISTERED 10 Oct 2026.** None of the 22 distinct e' is flanked; P14 holds
only at 0 of 22. The 8 behind the 54 hold by construction, since a flanking pair
would put an assigned element within 1 m. S10 checks them rather than assuming it.
P14 tests the audit, not the maps: a flanked e' means the audit missed a rule.

P13a, P13b and P14 are judged separately.

### S10 results

Source: `experiments/logs/divergence/s10_segments_20261011T134922Z.json`, sha256
`fa750aa97fc08c5a1f177b78019832c3c5794ef42a8898d9d8f6d1d7127407e2`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `ed2a618`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

- **G8: HOLD,** all three checks.
- **P13a: FAIL.** 2 of the 5 distinct S_b behind the 54 losses with no assigned
  element within 1 m are changed; 3 required.
- **P13b: HOLD.** 12 of the 14 distinct S_b behind the 75 one-cell holes are intact;
  7 required.
- **P14: HOLD.** 0 of the 22 distinct e' are flanked.
- No S_b lies behind both shapes. Of the 19 distinct S_b, 4 are changed and 15
  intact.
- Per loss, reported and not judged: one-cell holes 20 changed, 55 intact; no
  assigned element within 1 m, 11 changed, 43 intact; 0 flanked. Judged per loss,
  every verdict would be the same.
- Of the 22 distinct e', 18 sit on intact S_b and 4 on changed ones.

**Table 1 — distinct S_b by loss shape and state at k'.** An S_b behind losses of
both shapes would count in both rows, as P13a and P13b are registered; none does.

| loss shape | changed | intact | total |
|---|---|---|---|
| one-cell hole | 2 | 12 | 14 |
| none within 1 m | 2 | 3 | 5 |
| of those, behind both shapes | 0 | 0 | 0 |
| **all distinct S_b** | 4 | 15 | 19 |

**Table 2 — the 22 distinct e'.** S_b is named by its segment id in B's map at k, and
`b` by its element index there; `members` and `length` are S_b's. The last two
columns are empty where S_b is intact.

| B | step | e' | b | S_b | members | length | shape | state | flanked | routes | nearest changed member |
|---|---|---|---|---|---|---|---|---|---|---|---|
| k0 | cut60 → cut120 | 436 | 198 | 3 | 52 | 5.05 m | none within 1 m | changed | no | wall cell freed | 120, 4.74 m |
| k0 | cut120 → cut240 | 760 | 511 | 5 | 53 | 5.15 m | one-cell hole | intact | no | — | — |
| k0 | cut240 → cut1200 | 86 | 14 | 6 | 72 | 7.15 m | one-cell hole | intact | no | — | — |
| k1 | cut60 → cut120 | 431 | 198 | 2 | 53 | 5.15 m | none within 1 m | intact | no | — | — |
| k1 | cut60 → cut120 | 432 | 199 | 2 | 53 | 5.15 m | none within 1 m | intact | no | — | — |
| k1 | cut120 → cut240 | 735 | 511 | 6 | 53 | 5.15 m | one-cell hole | intact | no | — | — |
| k1 | cut240 → cut1200 | 85 | 14 | 5 | 72 | 7.15 m | one-cell hole | intact | no | — | — |
| k1 | cut240 → cut1200 | 232 | 105 | 11 | 15 | 1.55 m | one-cell hole | intact | no | — | — |
| k2 | cut60 → cut120 | 137 | 88 | 5 | 40 | 3.85 m | none within 1 m | changed | no | front cell closed | 80, 2.50 m |
| k2 | cut120 → cut240 | 17 | 14 | 3 | 74 | 7.15 m | one-cell hole | intact | no | — | — |
| k2 | cut120 → cut240 | 799 | 543 | 5 | 53 | 5.15 m | one-cell hole | intact | no | — | — |
| k2 | cut240 → cut1200 | 227 | 106 | 5 | 73 | 7.15 m | one-cell hole | intact | no | — | — |
| k2 | cut240 → cut1200 | 1017 | 716 | 8 | 55 | 5.15 m | one-cell hole | changed | no | wall cell freed, front cell closed | 652, 1.30 m |
| k2 | cut240 → cut1200 | 1022 | 722 | 9 | 54 | 5.16 m | one-cell hole | intact | no | — | — |
| k2 | cut240 → cut1200 | 1099 | 801 | 7 | 56 | 5.15 m | one-cell hole | changed | no | wall cell freed | 809, 2.70 m |
| k3 | cut240 → cut1200 | 571 | 518 | 5 | 73 | 7.15 m | one-cell hole | intact | no | — | — |
| k3 | cut240 → cut1200 | 1118 | 947 | 6 | 53 | 5.15 m | none within 1 m | intact | no | — | — |
| k3 | cut240 → cut1200 | 1119 | 948 | 6 | 53 | 5.15 m | none within 1 m | intact | no | — | — |
| k4 | cut60 → cut120 | 81 | 57 | 2 | 73 | 7.15 m | none within 1 m | intact | no | — | — |
| k4 | cut60 → cut120 | 83 | 58 | 2 | 73 | 7.15 m | none within 1 m | intact | no | — | — |
| k4 | cut120 → cut240 | 551 | 308 | 5 | 53 | 5.15 m | one-cell hole | intact | no | — | — |
| k4 | cut240 → cut1200 | 543 | 358 | 5 | 72 | 7.15 m | one-cell hole | intact | no | — | — |

**Table 3 — the base rate per step,** over every segment of B at k and not only the
S_b behind a loss.

| B | step | segments | changed | share |
|---|---|---|---|---|
| k0 | cut60 → cut120 | 4 | 4 | 1.0000 |
| k0 | cut120 → cut240 | 7 | 1 | 0.1429 |
| k0 | cut240 → cut1200 | 10 | 4 | 0.4000 |
| k1 | cut60 → cut120 | 4 | 2 | 0.5000 |
| k1 | cut120 → cut240 | 7 | 2 | 0.2857 |
| k1 | cut240 → cut1200 | 12 | 5 | 0.4167 |
| k2 | cut60 → cut120 | 7 | 3 | 0.4286 |
| k2 | cut120 → cut240 | 7 | 2 | 0.2857 |
| k2 | cut240 → cut1200 | 10 | 7 | 0.7000 |
| k3 | cut240 → cut1200 | 11 | 3 | 0.2727 |
| k4 | cut60 → cut120 | 7 | 3 | 0.4286 |
| k4 | cut120 → cut240 | 7 | 4 | 0.5714 |
| k4 | cut240 → cut1200 | 9 | 4 | 0.4444 |
| **total** | 13 steps | 102 | 44 | 0.4314 |

What S10 shows:

1. **P13a's reason is refuted.** It rested on the minimum length: a segment that met
   l_min at k can fail it at k' only by losing members. The 5 S_b behind the 54
   losses are 3.85–7.15 m long, and 3 of them are intact.
2. **Where S_b is changed, the change is not near e'.** In all 4 changed S_b, the
   nearest changed member lies 1.30–4.74 m along the line from e'.
3. **Base rate,** reported and not tested. Over the 13 steps that carry a loss, 44 of
   102 segments are changed (0.43). Behind the losses, 4 of 19 are (0.21). This
   comparison was not registered and carries no verdict.

**Record.** v0.16 withdrew the claim that most losses come from our segmentation and
not from B's map changing, until S10 reported. S10 restores it in a narrower form. Of
the 19 segments behind the 129 losses, 15 are intact at k', and in the other 4 the
nearest change is at least 1.29 m from e'. So these losses do not come from changes
to the walls they sit on. They come from the extraction, whose rules read the whole
map and its grid (v0.16 audit).

## The rule that drops each e' (P15, P16)

**Why.** S10 shows the losses sit on walls that are intact at k', or changed far from
e'. The v0.16 audit names the rules by which an element can end unassigned. Which
rule acts decides what a fix must change, and whether openings and corners can be
read from segment ends.

**Definitions.**

- **Units.** The 22 distinct (B, k, k', e') of S10: 18 on intact S_b, 4 on changed
  ones. The 22 are not independent: 3 S_b each carry two e'. Counts per distinct S_b
  are reported beside, not judged.
- **Trace.** `graph_walls`' extraction on B at k', and on B at k for the line
  quantities at k, rerun with a read-only tracer. For every peak iteration it
  records:
  - the seed (theta index and d bin);
  - whether e' voted in the seed window;
  - e''s distance to the iteration's first-pass line and to its refitted line;
  - whether e' was a candidate of the refitted line;
  - which run held e', the run's length, and whether the run was accepted.

  The tracer changes no output (G9, check 2).
- **Counterpart** of a member m of S_b at k': the element at k' with m's direction,
  in the cell found at m's wall-cell position with `trinary_map.cell_index`, if one
  exists.
- **Home iteration H:** the iteration at k' that assigned the most counterparts of
  S_b's members. Ties go to the earliest. If no counterpart is assigned at k', e' has
  no home.
- **Refitted line:** the line of the iteration's own refit, the one its second
  candidate pass tests against; not the line `_segment` may report for a segment.
- **Class.** If H does not exist, e' is in **No home**. Otherwise e' takes the first
  class that applies at H:
  - **R3, line:** e' is farther than eps from H's refitted line.
  - **R4, facing:** e' does not face H's refitted line.
  - **R5+R6, short run:** e' is a candidate of H, in a run rejected as shorter than
    l_min.
  - **Other:** none of the above.

**What is computed.**
- **G9 (S11 against S10).** S11 refuses an edited tree, requires this registration's
  commit to be an ancestor of HEAD, and pins S10's result file by sha256
  `fa750aa97fc08c5a1f177b78019832c3c5794ef42a8898d9d8f6d1d7127407e2`. Checks, in
  order:
  1. S10's stored values reproduce exactly through the functions S10's stage calls,
     without writing an S10 result file: the verdicts, the 19 S_b and their states,
     the 22 e' rows, and the base rate.
  2. **Non-interference:** for every map traced, `seg_of` and the segment list with
     the tracer on equal those with it off, exactly.
  3. Every e' is unassigned in the traced run, and lands in exactly one class.

  On a gate failure, S11 classifies nothing and prints no class.
- **Reported, not tested.**
  - Classes by loss shape, and by intact or changed S_b.
  - At H: the angle between H's refitted line and the wall's axis, in degrees; e''s
    distance to the first-pass and refitted lines, against eps; and the along-extent
    of H's candidates against S_b's span, which shows whether H's line was fitted to
    elements beyond the wall.
  - The same line quantities at k, for the iteration that assigned b.
  - For R5+R6: whether the gap that cut e''s run lies where H's candidates were
    thinned by R3.
  - After H, why no later iteration took e': never in a seeded window again (R1), its
    seed bin exhausted (R2), a later rejected run (R5+R6), or an aborted iteration
    (R7).
  - For no home: whether R1 or R2 kept S_b's counterparts unassigned.
  - Whether e''s nearest assigned same-direction element at k' is on e''s own face
    line.

S11 computes no viewpoint weight, so O3's mixed-cut prediction can still be
registered.

Registered: 11 Oct 2026, before any quantity below was computed on the real maps.

**P15. REGISTERED 11 Oct 2026.** At least 11 of the 22 distinct e' are in R3. Reason:
the audit's only exclusion rule for a live element beside members of the same face is
the line. H's line is refitted to every candidate along the infinite line, so it can
move while the wall's own cells do not.

**P16. REGISTERED 11 Oct 2026.** None of the 22 is in Other. P16 holds only at 0 of
22. A candidate in an accepted run is assigned, so an e' in Other would mean the
audit or the trace missed a rule. R4 is not excluded: a segment can hold faces of
more than one direction, so R4 can act, and its count is reported. P16 tests the
audit and the trace, not the maps.

P15 and P16 are judged separately.

### S11 results

Source: `experiments/logs/divergence/s11_trace_20261012T233219Z.json`, sha256
`006a7f8fc2774cf1324b613d42e70d8e4d103bd8e8298624a8630522c58663dd`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `dc5e2a3`, the commit that added the stage, against a clean tree. Every
number here was read from that file by script.

- **G9: HOLD,** all three checks.
- **P15: FAIL.** 1 of the 22 distinct e' is in R3; 11 required.
- **P16: HOLD.** 0 of the 22 are in Other.
- The other 21 are in R4: e' does not face H's refitted line. None is in No home or
  R5+R6.
- Per distinct S_b, reported and not judged: all 19 hold an R4 element, and 1 of them
  (k4, cut60 → cut120) also holds the R3 element.

**Table 1 — classes by loss shape.**

| class | one-cell hole | none within 1 m | total |
|---|---|---|---|
| No home | 0 | 0 | 0 |
| R3 | 0 | 1 | 1 |
| R4 | 14 | 7 | 21 |
| R5+R6 | 0 | 0 | 0 |
| Other | 0 | 0 | 0 |
| **all distinct e'** | 14 | 8 | 22 |

**Table 2 — classes by the state of S_b at k'.**

| class | intact | changed | total |
|---|---|---|---|
| No home | 0 | 0 | 0 |
| R3 | 1 | 0 | 1 |
| R4 | 17 | 4 | 21 |
| R5+R6 | 0 | 0 | 0 |
| Other | 0 | 0 | 0 |
| **all distinct e'** | 18 | 4 | 22 |

**The riser reading (post hoc, not registered).** Read from the S11 file by script,
after the verdicts above. None of it was registered, and it carries no verdict.

- **The angle.** The angle S11 reports lies between the refitted line's direction and
  the element's own face tangent, folded into 0–90°. So 90° minus the angle is the
  line's unsigned tilt from its nearest axis.
- **Perpendicular.** All 22 e' are perpendicular to the axis nearest H's refitted
  normal. The file does not store the directions of H's members, so this axis stands
  in for their majority direction.
- **Facing.** In all 22, b faced the line that assigned it at k (dot > 1e-9), and e'
  does not face H's refitted line at k' (dot ≤ 1e-9). In size, each dot equals the
  sine of that line's tilt: at most 0.0351 at k and 0.0197 at k'.
- **Tilt sign.** In all 22, the line's tilt, measured from the same axis with the
  same sign convention, has opposite signs at k and k'.
- **The R3 element** (k4, e' 83) is a riser too, with a negative dot at k'. It is in
  R3 because R3 is tested first: it lies 1.167 eps from H's refitted line.
- **b's place in its run.** In 13 of the 22, b's run is the first run of its
  candidates, so b's place in it is known: in all 13, b is the first, second, last or
  second-to-last element. For the other 9 it is not stored.
- **Not stored, so not checked:** the directions of H's members, and e''s position
  along H's span at k'.

**Table 3 — the 22 distinct e', one row each.** `H` is the home iteration at k';
`first-pass` and `refit` are e''s distances to H's first-pass and refitted lines,
against eps = 0.1707 m; `e' dir` is e''s direction and `axis` the axis nearest H's
refitted normal. The dot is that of the element's normal with the refitted normal: e'
with H's at k', b with the line that assigned it at k. The tilt is the refitted
line's, from its nearest axis, counter-clockwise positive; the nearest axis is the
same at k and k' in every row.

| B | step | e' | S_b | shape | state | class | H | first-pass (m) | refit (m) | e' dir | axis | dot at k' | tilt at k' | dot at k | tilt at k | sign differs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| k0 | cut60 → cut120 | 436 | 3 | none within 1 m | changed | R4 | 6 | 0.1850 | 0.0518 | +y | +x | −0.001070 | −0.0613° | +0.003771 | +0.2161° | yes |
| k0 | cut120 → cut240 | 760 | 5 | one-cell hole | intact | R4 | 7 | 0.1825 | 0.0643 | −y | −x | −0.008411 | −0.4819° | +0.008411 | +0.4819° | yes |
| k0 | cut240 → cut1200 | 86 | 6 | one-cell hole | intact | R4 | 4 | 0.0651 | 0.0589 | +x | +y | −0.004979 | +0.2852° | +0.006890 | −0.3948° | yes |
| k1 | cut60 → cut120 | 431 | 2 | none within 1 m | intact | R4 | 5 | 0.2355 | 0.1643 | +y | +x | −0.008411 | −0.4819° | +0.003990 | +0.2286° | yes |
| k1 | cut60 → cut120 | 432 | 2 | none within 1 m | intact | R4 | 5 | 0.1356 | 0.0643 | +y | +x | −0.008411 | −0.4819° | +0.003990 | +0.2286° | yes |
| k1 | cut120 → cut240 | 735 | 6 | one-cell hole | intact | R4 | 6 | 0.1825 | 0.0643 | −y | −x | −0.008411 | −0.4819° | +0.003990 | +0.2286° | yes |
| k1 | cut240 → cut1200 | 85 | 5 | one-cell hole | intact | R4 | 4 | 0.0640 | 0.0509 | +x | +y | −0.002386 | +0.1367° | +0.008346 | −0.4782° | yes |
| k1 | cut240 → cut1200 | 232 | 11 | one-cell hole | intact | R4 | 6 | 0.1061 | 0.0747 | +x | +y | −0.006668 | +0.3821° | +0.004619 | −0.2646° | yes |
| k2 | cut60 → cut120 | 137 | 5 | none within 1 m | changed | R4 | 2 | 0.1680 | 0.1659 | −x | −y | −0.006239 | +0.3575° | +0.002913 | −0.1669° | yes |
| k2 | cut120 → cut240 | 17 | 3 | one-cell hole | intact | R4 | 4 | 0.0591 | 0.0598 | +x | +y | −0.006164 | +0.3532° | +0.013583 | −0.7783° | yes |
| k2 | cut120 → cut240 | 799 | 5 | one-cell hole | intact | R4 | 8 | 0.0988 | 0.0640 | −y | −x | −0.011209 | −0.6422° | +0.003990 | +0.2286° | yes |
| k2 | cut240 → cut1200 | 227 | 5 | one-cell hole | intact | R4 | 5 | 0.0639 | 0.0579 | +x | −y | −0.003646 | −0.2089° | +0.002371 | +0.1358° | yes |
| k2 | cut240 → cut1200 | 1017 | 8 | one-cell hole | changed | R4 | 10 | 0.1356 | 0.0643 | +y | −x | −0.008411 | +0.4819° | +0.011209 | −0.6422° | yes |
| k2 | cut240 → cut1200 | 1022 | 9 | one-cell hole | intact | R4 | 11 | 0.1136 | 0.0951 | −y | +x | −0.019673 | +1.1273° | +0.035092 | −2.0110° | yes |
| k2 | cut240 → cut1200 | 1099 | 7 | one-cell hole | changed | R4 | 8 | 0.0878 | 0.0733 | −y | +x | −0.010962 | +0.6281° | +0.001444 | −0.0827° | yes |
| k3 | cut240 → cut1200 | 571 | 5 | one-cell hole | intact | R4 | 5 | 0.1719 | 0.0999 | −x | −y | −0.017633 | +1.0104° | +0.014097 | −0.8077° | yes |
| k3 | cut240 → cut1200 | 1118 | 6 | none within 1 m | intact | R4 | 9 | 0.0771 | 0.1338 | −y | +x | −0.012238 | +0.7012° | +0.003990 | −0.2286° | yes |
| k3 | cut240 → cut1200 | 1119 | 6 | none within 1 m | intact | R4 | 9 | 0.0229 | 0.0338 | −y | +x | −0.012238 | +0.7012° | +0.003990 | −0.2286° | yes |
| k4 | cut60 → cut120 | 81 | 2 | none within 1 m | intact | R4 | 3 | 0.1749 | 0.0993 | +x | −y | −0.012582 | −0.7209° | +0.002140 | +0.1226° | yes |
| k4 | cut60 → cut120 | 83 | 2 | none within 1 m | intact | R3 | 3 | 0.2749 | 0.1992 | +x | −y | −0.012582 | −0.7209° | +0.002140 | +0.1226° | yes |
| k4 | cut120 → cut240 | 551 | 5 | one-cell hole | intact | R4 | 7 | 0.1796 | 0.0724 | +y | +x | −0.013594 | −0.7789° | +0.008411 | +0.4819° | yes |
| k4 | cut240 → cut1200 | 543 | 5 | one-cell hole | intact | R4 | 7 | 0.0795 | 0.0592 | −x | −y | −0.003546 | +0.2032° | +0.006890 | −0.3948° | yes |

What S11 shows:

**Risers.** A riser, here, is a face element perpendicular to the line of the wall
segment in question. In a grid map such faces arise where a wall's cells step
sideways or end; where the 22 sit along their walls is not stored. `graph_walls`
admits an element to a line only if the element faces the line: the dot of their
normals must exceed FACING_MIN = 1e-9 (W4). For a riser, that dot is the sine of the
line's tilt from the axis. So a riser joins a wall when the refitted line tilts
toward it, by any amount, and leaves it when the line tilts away.

**The mechanism.** The refitted line is fitted to every candidate along the infinite
line (v0.16 audit), so its tilt can change sign between two cuts while the wall's own
cells do not. S11 and the reading above show this for all 22 distinct e': each is a
riser with respect to H's refitted line (perpendicular to it within 1.13°), b faced
its line at k, and e' faces away from H's line at k', with the tilt's sign reversed.
21 fail the facing test (R4). One also lies beyond eps and is counted as R3.

**Corrections.**

- P15's reason assumed that e' lies on the same face as H's members, so that only the
  line's distance could exclude it. It does not: all 22 are risers with respect to
  H's line. The line's tilt is the cause, but it acts through facing, not distance.
- P16's first draft reasoned that an element of b's direction faces any line that b's
  segment faces. The review before registration found this wrong (recorded in the
  commit message of `dc754de`), and P16 was amended to allow R4. Unamended, P16 would
  have failed at 21 of 22.

**For the measure.** Whether B corroborates one of A's risers depends on the sign of
a small tilt in B's line, 0.06–2.01° across the 22, not only on B's evidence. The
effect is small: P9's largest rise is 0.8 % of A's walls, and in P3's series no step
loses more than 7 elements, while every step gains at least 160. Article 3 will
report it as a limitation of the wall measure.

**Record.** v0.17's record said the losses come from the extraction, whose rules read
the whole map and its grid. v0.18 narrows it: the 129 segmentation losses, 22
distinct elements, are risers with respect to H's refitted line, and their membership
in a wall segment follows the sign of that line's tilt.

## Viewpoint weight at mixed cuts (P17)

Registered in a commit that precedes the commit of the code that tests it (S12), and
before any weight was computed on a row with cut_A ≠ cut_B other than S5's rows with
A at cut1200.

**Mirror pairs.** A mirror pair is two rows that swap roles: X@a → Y@b and
Y@b → X@a, for distinct robots X and Y. The 240 rows with a ≠ b form 120 mirror
pairs, in two sets:
- **M60 (tested):** a and b both in {60, 120, 240}. 60 pairs: 10 unordered robot
  pairs × 3 unordered cut pairs × 2 ways to assign the cuts. No weight has been
  computed on either row of any M60 pair.
- **M1200 (reported):** one of a and b is 1200. 60 pairs. In each, the row with A at
  cut1200 is one of S5's reported rows; its mirror's weight has not been computed.

The 40 same-cut mirror pairs (10 at each cut) are all in S5's file.

**Why the directions should agree.** w is symmetric in its two histograms (Caution 1)
and constant within a matched segment pair (Caution 2). So w_mean(A, B) is a mean of
the matched pairs' w, each weighted by the length of A's weighable corroborated
elements in that pair, and w_mean(B, A) is a mean of the same values, weighted by B's
length. Corroboration is expected to be mutual: where both maps hold a wall element at
the same place, each corroborates the other and is normally matched to the other's
segment. The code does not enforce this. Each direction picks its own nearest element
under T1, and the across tolerance (eps = 0.1707 m) is wider than one element
(rho = 0.1 m). A match to a neighbour changes nothing unless the neighbour lies in
another segment, which happens near segment ends or where two same-facing segments
lie within eps of each other. So a matched pair carries about the same length from
both sides. The sides can differ only through such cases: lattice offsets between the
two maps, segment ends and risers (S11), same-facing segments within eps of each
other, and elements that are corroborated but unweighable. At cut1200 the mirror
difference was at most 0.003° (Caution 1). In M60 the smaller map has 4–8 segments,
against 12–13 at cut1200, so an edge element is a larger share of its pair. The
weights are also larger at early cuts (P8: w_mean 15.16° to 27.71° with B at cut60),
so the matched pairs' weights can lie further apart. The same few edge elements can
therefore move a mean by more. The bound below allows for that; a systematic
asymmetry would exceed it in many pairs.

**Disclosure.** The bound and the count were chosen before any of the following was
read, and were not changed after. Before registration, the same-cut mirror
differences in S5's file were read. At cut60, cut120 and cut240, d is exactly 0 in all
30 pairs: the two directions' w_mean are identical. At cut1200, d is exactly 0 in 8 of
the 10 pairs and at most 0.003° in the other 2. None exceeds 1°. No weight on any M60
row, or on the unseen row of any M1200 pair, was read or computed.

**P17. REGISTERED 13 Oct 2026.** For a mirror pair, let
d = |w_mean(X@a → Y@b) − w_mean(Y@b → X@a)|, with w_mean as in "Viewpoint weight
(O1)". P17 holds if d ≤ 1/180 (1°) for at least 54 of the 60 pairs of M60. A pair in
which either row has no weighable corroborated element counts against P17. Every pair
with d over 1° is shown with its matched pairs: for each (S_A, S_B), its w and the
length it carries from each side.

**G10 (gate).** Before it computes any weight on a row S5 does not hold, S12:
1. recomputes C on all 320 rows and requires each to equal S6's value exactly;
2. recomputes w_mean, W, w_p10, w_p50, w_p90, unweighable_A and unweighable_B on every
   row where S5 holds them, and G3 on the 20 maps against themselves, and requires
   each to equal S5's value exactly.

On any difference it stops before writing a verdict. A third check runs on every row
S12 computes: the length-weighted mean of w over the row's recorded (S_A, S_B) pairs
equals w_mean within 1e-12. If it fails on any row, S12 writes no verdict.

**What S12 computes and records.** For each of the 320 rows: w_mean, W, w_p10, w_p50,
w_p90, unweighable_A, unweighable_B, the length of A's weighable corroborated
elements, and the distinct (S_A, S_B) pairs, each with its w and the length it
carries. For each of the 160 mirror pairs (120 mixed-cut, 40 same-cut): both rows'
w_mean and d. M1200 and the same-cut pairs are reported, not judged. S12 refuses an
edited tree, requires this registration's commit to be an ancestor of HEAD, and pins
S5's and S6's result files by sha256.

**What is not computed.** w_null and E7 at mixed cuts. S12 does not compute them, so
a prediction about them can still be registered.

**The pair record.** The distinct (S_A, S_B) pairs per row are what Caution 2 says a
rerun of S5 should add first, and what O6 asks for. S12 records them on all 320 rows.

### S12 results

Source: `experiments/logs/divergence/s12_mirror_20261013T101707Z.json`, sha256
`247faaa851e9c29cbd5fdd29f4b03e8f27a9b3faef2968ed721be9405486f1ba`. The file is not
tracked — `experiments/logs/` is gitignored — so the hash stands in for it. Produced
at HEAD `4f86b8f`, the commit that added the stage, against a clean tree, in 29.9 s
and 340 weight calls. Every number here was read from that file by script.

**G10 — HOLD, all three checks.**
- Check 1: C equals S6's value exactly on 320 of 320 rows. The other four fields S6
  stores per row were compared beside it, not gating: 1,280 of 1,280 equal.
- Check 2: 1,440 of 1,440 weight values equal S5's exactly, over the six locations in
  S5's file that store them, and G3 holds on 20 of 20 maps. The fields S5 stores
  beside them were compared, not gating: 600 of 600 equal.
- Check 3: on all 320 rows the recorded pairs give back w_mean within 1e-12; the
  largest difference is 5.6e-17. S12 also read check 3 on S5's 140 rows before it
  computed a weight on any other row, which is stricter than registered.

**P17 — HOLD, 60/60, 54 required.** The largest d is 0.000313 (0.0563°), at
k3@cut60 ↔ k4@cut120. d is exactly 0 in 58 of the 60 pairs. No row lacks a
weighable element.

**Reported, not judged.**
- M1200: d is exactly 0 in 60 of 60 pairs.
- Same cut: d is exactly 0 in 38 of 40. The other two are the cut1200 pairs
  k0 ↔ k2 and k0 ↔ k3, as disclosed at registration.
- w from the two sides of one matched segment pair is identical in all 1,264 segment
  pairs held by both rows of a mirror pair. No segment pair is held by one row only.

**The two rows are opposite directions.** In all 120 mixed-cut mirror pairs the two
rows' C differ (|ΔC| 0.118 to 0.809, median 0.501), and so do their L_A (11.5 m to
109.7 m). Each row's C equals S6's (check 1), and on all 320 rows the weight call's
corroborated count, weighable plus unweighable, equals S6's n_corroborated. So d = 0
is not one row read twice.

**Distinct (S_A, S_B) pairs (O6, Caution 2).** Over the 320 rows a row has 4 to 22
distinct pairs, median 7. With both maps at cut1200 it has 12 to 22, against Caution
2's bound of 169, behind 1,286 to 1,298 weighable elements: 58.5 to 108.2 elements
per pair. The effective sample of a row is its pair count.

**Post hoc, not registered.** Read from the stored file only, after the verdict.
- In all 160 mirror pairs, the two rows hold the same set of matched segment pairs
  (S_X, S_Y).
- In 156, every matched segment pair also carries the same number of elements from
  both sides, and d is exactly 0.
- In the other 4, one matched segment pair carries one element more on one side:

| Mirror pair | S_X, S_Y | elements X → Y / Y → X | d |
|---|---|---|---|
| k2@cut60 ↔ k4@cut120 | 2, 3 | 71 / 72 | 0.0112° |
| k3@cut60 ↔ k4@cut120 | 2, 3 | 72 / 73 | 0.0563° |
| k0@cut1200 ↔ k2@cut1200 | 0, 0 | 204 / 205 | 0.0028° |
| k0@cut1200 ↔ k3@cut1200 | 0, 0 | 207 / 206 | 0.0025° |

- The registered reason allowed for edge elements to move the mean further at early
  cuts. In 58 of the 60 M60 pairs, no element differed between the two sides.
- Two rows carry one unweighable element each: k2@cut240 → k3@cut1200 and its mirror,
  which S5 holds. It is unweighable_A in the row from k3 and unweighable_B in the row
  to it, so both fall on k3@cut1200's side, where the corpus's one segment with an
  empty histogram lies (O2). d for that pair is 0.

**Newly seen, reported, not judged: w_mean at every pair of cuts.** Min–max over the
20 ordered pairs, in degrees. A cell marked `*` had no weight computed before S12.

| A's cut | B at 60 | B at 120 | B at 240 | B at 1200 |
|---|---|---|---|---|
| 60 | 5.82–24.63 | 7.45–25.44 `*` | 9.86–28.48 `*` | 15.16–27.71 `*` |
| 120 | 7.45–25.44 `*` | 9.36–29.82 | 10.19–32.90 `*` | 11.62–20.64 `*` |
| 240 | 9.86–28.48 `*` | 10.19–32.90 `*` | 8.42–20.00 | 9.31–14.59 `*` |
| 1200 | 15.16–27.71 | 11.62–20.64 | 9.31–14.59 | 5.32–10.58 |

Held at one cut of A, w_mean is non-increasing as B's cut goes 60 → 120 → 240 → 1200
in 19 of the 20 series with A at cut1200, and in 0, 2 and 1 of 20 with A at cut60,
cut120 and cut240. No prediction was registered on whether these series fall. P8
compared only the first and last values of the series with A at cut1200 (B at cut60
against B at cut1200). The series are now seen, so no such prediction can be
registered.

**Reading.** At every pair of cuts, w_mean is the same in both directions — exactly
in 156 of the 160 mirror pairs and within 0.06° in the other 4 — while C is not (S6).
So the convergence series can give one viewpoint weight per pair of maps, and must
give C in both directions. Mirror agreement is not convergence: with A held at cut60,
cut120 or cut240, 57 of the 60 series rise at some step as B explores.

## 10. Open, not decided

- **O1.** Viewpoint weighting. Corroboration cannot be weighted by the number of
  observers (schema handoff §3.3); decided in v0.8, see §4 "Viewpoint weight
  (O1)". CLOSED in v0.10: the weight is defined, registered and measured, and
  G3, P7 and P8 all hold — see "Results: G3, P7, P8" in §9.
- **O2.** Faces with empty bearing histograms: 4 of 220 on the old corpus.
  Recounted on the extfix corpus in v0.10 — 1 segment in the whole corpus, on
  `b2maps_k3_cut1200_gated_extfix_robot3`, per-map counts in "Results: G3, P7,
  P8" in §9. Treatment still undecided, though `unweighable_B` was 0.0 on every
  cut1200 row, so nothing was lost to it there. S12 finds one unweighable
  element in each direction of k2@cut240 ↔ k3@cut1200, the only two rows of 320
  with any.
- **O3. CLOSED in v0.20.** Form decided (v0.11). P9 failed 40/60 (S6). P17 held
  60/60 (S12): w_mean is the same in both directions at every pair of cuts, while C
  is not.
- **O4.** Whether to report separately the unobserved length that parked robots
  shadow.
- **O5.** The deep tail, unexplained. On A's own maps 4-10 % of crossings that meet
  conditions 1 and 2 run on well past the face, and the distribution is bimodal:
  under 0.2 m or over 1 m, with almost nothing between. Turning (yaw rate at the
  scan) and wall-box joints were each pre-registered and each ruled out. Condition 4
  is a guard against it, not an explanation of it.
- **O6. CLOSED in v0.20.** S12 records the distinct `(S_A, S_B)` pairs on all 320
  rows: 4 to 22 per row, and 12 to 22 with both maps at cut1200 ("S12 results").
- **O7. Losses. CLOSED in v0.15.** P10 held (S7); P11 failed (S8), 21 of 213 wall
  cells freed; P12 held (S9), 75 of the 78 tested unchanged losses in segmentation
  and 54 of 54 in the carve-out. All 213 losses now carry a cause:

  | cause | count |
  |---|---|
  | segmentation: cells unchanged, e' not assigned at k' | 129 |
  | off grid at k' | 31 |
  | front cell closed | 29 |
  | wall cell freed | 21 |
  | geometry | 3 |
  | **total** | 213 |

  Segmentation is the majority cause, 129 of 213, and geometry accounts for 3. What
  is left is why the segmenter leaves those 129 unassigned — O8.
- **O8. Unassigned face elements at k'. CLOSED in v0.18.** For 129 of the 213
  losses b's two cells hold their states at k', so B's grid at k' still carries the
  face element e' between them, in b's direction — and yet `graph_walls` does not
  assign e' to a wall segment, while b, its counterpart at the earlier cut k, was
  assigned. Why the segmenter drops an element whose two cells are unchanged, and
  what changed about it between B's two cuts, is unexplained. Not pursued: the chain
  stopped at S9 by decision on 10 Oct 2026. Reopened in v0.16. S10 tests P13a, P13b
  and P14. S10 (v0.17): the losses sit on walls intact at k', or changed
  at least 1.29 m from e'. P13a failed; P13b and P14 held. S11 traces which rule
  drops each e' (P15, P16). S11 (v0.18): the 129 segmentation losses are 22 distinct
  elements, all risers with respect to H's refitted line. `graph_walls` admits a
  riser to a wall only when the refitted line tilts toward it, and the tilt's sign
  changed between the two cuts in all 22. Reported as a limitation of the wall
  measure; see O9.
- **O9. Risers. OPEN.** Two questions for the rest of the schema. (1) Whether the
  wall measure should count elements perpendicular to their segment's line at all.
  Changing this would change the registered measure, and it is not proposed now. (2)
  Whether segment ends, for openings and corners, are read only from elements in the
  direction of the segment's normal axis, so that a riser leaving a segment cannot
  move its end. Proposed: yes, to be decided when the schema spec is written.

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
