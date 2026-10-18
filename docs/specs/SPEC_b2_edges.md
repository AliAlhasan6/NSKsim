# SPEC_b2_edges — the face graph (v0.2, 18 Oct 2026)

Status: **v0.2, for registration.** No map data has been read under this spec. Truth
was computed under v0.1 (`edges_truth_fstar_out.txt`, sha256 `6dbc3354…`), and it
matched the hand prediction made before the run exactly.
Builds on `SPEC_b2_wall_predicate.md` (v2) and `SPEC_b2_schema.md` (v0.2).
Inputs: the per-map schema file and graph_walls file, joined by segment `id` and `map_sha256`.

Changes from v0.1:
- §3.3 condition 2 is opposite axes, not exactly opposite normals (A6).
- Slab ends are added as occluders (A5).
- The numeric conventions found while computing truth (A1–A4, A7–A11) are fixed in §3.4.
- `offset_m` and `bearing_deg` are stored once per edge (A7).
- F* is pinned (§5.1), and the prediction bounds are set (§5.3).
- P-E2 and P-E3 match through the judge's face match M1, not M0.

---

## 0. Purpose

Turn one map's schema output into a typed graph that an embedder can read. This finishes
the B2 item "edge types" (schema-entry handoff, :133–135) and closes the gap the B3
inventory found: no code turns schema elements into a graph.

## 1. Decisions (agreed 18 Oct 2026)

- **D-E1. Nodes are faces.** A node is one segment that has axis ends. Segments with
  `no_axis_member` (5 of 179 on the 20 maps) are not nodes. They are counted, and they
  still occlude in §3.3.
- **D-E2. Corners and slabs are edges.** Each inside corner becomes a `corner` edge
  between its two faces. Each pair becomes a `slab` edge between its two faces. Slab
  ends are attributes of the slab edge, not edges of their own.
- **D-E3. A new edge type, `facing`.** It links two faces that see each other across
  free space (§3.3). It is the plan's "relative bearing", given a definition.
- **D-E4. Connectivity and containment are out of v1.** There are no openings (openings
  are a control, D-S2) and no region nodes.
- **D-E5. The knowledge/claim switch moves to B4.** It governs sharing between robots,
  not one map's graph.
- **D-E6. Frame.** All geometry stays in robot K's map frame. No world coordinate enters
  the graph (as D-S5). World coordinates appear only in the gate and judging (§5).

## 2. Nodes

For a face S, the node record holds:

| field | source |
|---|---|
| `id`, `line` | schema `segments[]` |
| axis a(S) ∈ {+x, −x, +y, −y}; unit axis vector n̂(S) | schema `axis` |
| fitted normal | schema `normal` (kept, not used by §3.3) |
| axis ends q0(S), q1(S); midpoint m(S) | schema `axis_ends` |
| axis length L(S) = tq1 − tq0 | schema `axis_ends.t` |
| axis-end states | schema `axis_ends` |
| `n_scans`, `n_hits`, `n_contradictions`, `t_first`, `t_last` | graph_walls `obs` |
| `bearing_hist` (72), `incidence_hist` (9) | graph_walls `obs` |

The record keeps every field. Which fields become the embedder's feature vector is
decided in the B3 builder (E-O2).

## 3. Edges

All edges are undirected. Each carries a `type` and its attributes. A pair of faces may
carry more than one type.

### 3.1 `corner`

For each inside corner C with `segments` [a, b]: an edge a–b.
Attributes: `c`, `near_end`, `near_m`, `states`.

### 3.2 `slab`

For each pair P with `segments` [k, j]: an edge k–j.
Attributes: `separation_m`, `overlap_m`, and for each of the two ends its `record`,
`diff_m`, `aligned` and `state`.

### 3.3 `facing`

**Occluders.** O is the union of:
- every segment of the map, nodes and excluded segments alike, each as its closed
  segment p0–p1;
- every slab end that carries two points, as the closed segment between them (its two
  outside corners). On truth these are the 8 slab end caps.

**Samples.** For a face S_i, let u_i = (n̂_y, −n̂_x), and sample its axis interval at
p_k = q0(S_i) + (k + ½)·ρ·u_i, for k = 0 … ⌊(L(S_i) + 10⁻⁹)/ρ⌋ − 1.

**Rays.** From each p_k cast r_k(λ) = p_k + λ·n̂(S_i), λ > 0. The ray runs along the
face's axis vector, not its fitted normal. Let H(p_k) be the occluder the ray crosses
first (smallest λ), and λ_k that distance.

Sample p_k **sees** face S_j when:
1. H(p_k) = S_j, and
2. a(S_j) = −a(S_i) (opposite axes), and
3. the crossing lies within S_j's axis interval [q0(S_j), q1(S_j)].

Let V_ij be the samples of S_i that see S_j. The visible width is

  w_ij = ρ · (|V_ij| + |V_ji|) / 2.

**A `facing` edge joins S_i and S_j iff |V_ij| + |V_ji| ≥ 2·w_min/ρ**, that is, ≥ 4
samples at ρ = 0.1, w_min = 0.2.

**Attributes**, computed once per edge from the face with the lower `id` as S_i
(on truth, the lower name):
- `gap_m`: the median of λ_k over V_ij ∪ V_ji;
- `width_m`: w_ij;
- `offset_m`: the signed distance along u_i from m(S_i) to the projection of m(S_j);
- `bearing_deg`: atan2(`offset_m`, `gap_m`).

For opposite faces u_j = −u_i, so the reverse direction gives the same two numbers.

### 3.4 Numeric conventions

- **C1.** Occluders and axis intervals are closed, with a slack of 10⁻⁹ m at each end.
- **C2.** An occluder parallel to the ray (|d × e| ≤ 10⁻¹² |e|) is never crossed.
- **C3.** A tie in the first crossing goes to the occluder with the lower `id` (on
  truth, the lower name; slab-end occluders after all segments).
- **C4.** A ray with no crossing is "no hit". A first crossing that fails condition 2
  or 3 is "blocked". Both are counted per face and reported.
- **C5.** A partial last cell at the q1 end is not sampled.
- **C6.** `gap_m` with an even count is the mean of the two middle values.
- **C7.** On truth, a boundary face's extent is the judge's exposed 19.9 m.

Notes:
- The two faces of a slab never face each other: each lies behind the other, so their
  rays point apart. `slab` and `facing` cannot both join one pair.
- Unclassified occupied cells do not occlude (E-O3).

## 4. Output file, one per map

```
{ "frame": "robot_K map",
  "provenance": { map_sha256, schema_file_sha256, graph_walls_commit, builder_commit },
  "thresholds": { rho, w_min },
  "nodes": [ ... §2 ... ],
  "excluded": [ { id, reason } ],
  "edges": [ { type, i, j, attrs } ],
  "diagnostics": { per-face seen / blocked / no hit / ties },
  "counts": { nodes, excluded, corner, slab, facing, components } }
```

`components` is the number of connected components over all edge types, nodes only.

## 5. Gate and predictions

### 5.1 F*, pinned

Under v0.1, §3.3 on the judge's 12 true faces gave 12 facing pairs
(`edges_truth_fstar_out.txt`, sha256 `6dbc3354…`):

| pair | width under v0.1 (m) |
|---|---|
| E – W | 11.80 |
| E – maze_v1+x | 3.00 |
| E – maze_v2+x | 5.05 |
| N – S | 7.80 |
| N – maze_h1+y | 7.05 |
| N – maze_h2+y | 5.00 |
| S – maze_h1-y | 5.00 |
| S – maze_h2-y | 7.05 |
| W – maze_v1-x | 5.05 |
| W – maze_v2-x | 3.00 |
| maze_h1-y – maze_h2+y | 2.00 |
| maze_v1+x – maze_v2-x | 2.00 |

**F* is these 12 pairs.** Widths are not pinned. Under v0.2, the slab-end occluders
remove the 20 samples that ran through a slab, so four widths drop by 0.25 m:
E – maze_v1+x, N – maze_h2+y, S – maze_h1-y and W – maze_v2-x. No pair is created or
lost.

With truth's slab and corner edges added, the 12 faces form one component. Without the
4 inside corners they form two: the six y-normal faces and the six x-normal faces.

### 5.2 Gate G-E1

Must hold before the predictions are judged:
1. On 20 of 20 maps, `corner` edges = inside corners and `slab` edges = pairs, as in
   the schema file.
2. Under v0.2, truth gives exactly the 12 pairs of F*, the four widths reduced by
   0.25 m, and 0 samples that run through a slab.
3. A decoy face placed far from every wall gets no `facing` edge.

### 5.3 Predictions, at cut1200 (5 maps)

A map face is matched to a true face by the judge's face match, M1 (`match_faces`).

- **P-E1. One component.** Each map's graph is one connected component.
  Bound: ≥ 4 of 5 maps.
- **P-E2. F* recovered.** The number of F* pairs whose two true faces are matched by
  two map faces joined by a `facing` edge. Bound: ≥ 11 of 12, on ≥ 4 of 5 maps.
- **P-E3. No wrong facing edges.** A wrong edge joins two map faces that match two true
  faces not paired in F*. Bound: wrong edges on ≤ 1 map.
- **P-E4. No unmatched facing edges.** A `facing` edge with at least one face that M1
  leaves unmatched. Bound: on ≤ 1 map.

**Reported, not judged:** components, facing counts and F* recovery at cuts 60, 120 and
240 (judging found elements before cut1200 waits on S-O5); widths and gaps against
truth's; per-face blocked and no-hit counts.

## 6. Open items

- **E-O1.** Whether absolute position in K's frame enters the node features. It differs
  between robots by s_K, so it could let the embedder tell robots apart by spawn alone.
- **E-O2.** The embedder's feature vector and relation ids (input_dim 2 and 237
  relations today). Belongs to the B3 builder.
- **E-O3.** Occlusion by unclassified occupied mass is ignored.
- **E-O4.** The value of w_min.
- **E-O5.** How the 6 misaligned and 1 unobserved slab ends are presented to the
  embedder.
- **E-O6.** A slab end that does not carry two points leaves its slab open to rays.
