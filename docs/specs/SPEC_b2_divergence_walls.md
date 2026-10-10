# SPEC — B2 divergence on walls (v0.15, draft)

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
chain stopped at S9 by decision on 10 Oct 2026. No code is written until the
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

## 10. Open, not decided

- **O1.** Viewpoint weighting. Corroboration cannot be weighted by the number of
  observers (schema handoff §3.3); decided in v0.8, see §4 "Viewpoint weight
  (O1)". CLOSED in v0.10: the weight is defined, registered and measured, and
  G3, P7 and P8 all hold — see "Results: G3, P7, P8" in §9.
- **O2.** Faces with empty bearing histograms: 4 of 220 on the old corpus.
  Recounted on the extfix corpus in v0.10 — 1 segment in the whole corpus, on
  `b2maps_k3_cut1200_gated_extfix_robot3`, per-map counts in "Results: G3, P7,
  P8" in §9. Treatment still undecided, though `unweighable_B` was 0.0 on every
  cut1200 row, so nothing was lost to it there.
- **O3.** Form decided; P9 registered (v0.11) and failed 40/60 (S6, v0.12). Open:
  whether the viewpoint weight's mirror agreement holds at mixed cuts. On the 180
  rows unseen before S6, the weight is still unseen.
- **O4.** Whether to report separately the unobserved length that parked robots
  shadow.
- **O5.** The deep tail, unexplained. On A's own maps 4-10 % of crossings that meet
  conditions 1 and 2 run on well past the face, and the distribution is bimodal:
  under 0.2 m or over 1 m, with almost nothing between. Turning (yaw rate at the
  scan) and wall-box joints were each pre-registered and each ruled out. Condition 4
  is a guard against it, not an explanation of it.
- **O6.** S5's output records the distinct `(S_A, S_B)` pairs per row, so caution
  2 gets an exact count at the next rerun.
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
- **O8. Unassigned face elements at k'. OPEN, not pursued.** For 129 of the 213
  losses b's two cells hold their states at k', so B's grid at k' still carries the
  face element e' between them, in b's direction — and yet `graph_walls` does not
  assign e' to a wall segment, while b, its counterpart at the earlier cut k,
  was assigned. Why the segmenter drops an element whose two cells are unchanged,
  and what changed about it between B's two cuts, is unexplained. Not pursued: the
  chain stopped at S9 by decision on 10 Oct 2026.

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
