# SPEC — B2 wall predicate and viewing directions, v2

**Status:** v2, after stop point 1. Part A may start once Ali has read §V2.
**Builds on:** `trinary_map.py` (`0dee108`), `robot_divergence.py` (`c57854a`),
`bag_overlap.world_walls_named()`.
**Scope:** walls only. Corners, openings, edges, the "same world" statement and the
divergence measure are later specs. This one records what they will need: endpoint
states (corners), gap states (openings), viewing directions (corroboration).

Claude Code edits and stops for review at each stop point in §9. Ali runs every
command and commits.

---

## V2. What changed since v1, and why

| # | Change | Cause |
|---|---|---|
| C1 | The two schema requirements this spec relies on are now quoted in §0.1, so no outside file is needed to check it. The handoffs live in the claude.ai project; Ali adds them to `docs/handoffs/` separately | stop-1 item ① |
| C2 | A face element is now an **oriented occupied–free cell boundary**, not a cell (Def 1). A segment accepts only elements whose normal agrees with its own (W4). Every segment is one-sided by construction; Def 3's `±` label is gone. A one-cell wall free on both sides gives two elements, one per side, with no either/or. Faces sit on the true surface, so a maze wall's two faces are 0.20 m apart, not 0.10 | stop-1 item ②: D1 was unreachable under v1 |
| C3 | `eps = rho*(1 + sqrt(2)/2)`, matching its stated reason | value and reason disagreed |
| C4 | W2 measures the physical gap, `t_(k+1) − t_(k) − rho`, so "a gap a robot could pass is never bridged" holds as stated | v1 bridged gaps up to one cell wider than `g` |
| C5 | Gap and endpoint states use a narrow band `b = rho*sqrt(2)/2` (cells touching the face), separate from `eps`, so they never read the wall's own back row | follows from C2 and C3 |
| C6 | A9's denominator is the 128.80 m of faces visible from inside the arena; the four outward faces of the boundary walls are excluded. Decided from geometry, before any recall data | stop-1 item ③ |
| C7 | `base_T_scan` is composed from the two-hop `/tf_static` chain | stop-1 item ④ |
| C8 | `r_max = 7.9`, the `max_laser_range` the replay gave Karto, read from the replay config | stop-1 smaller points |
| C9 | A4 runs on a 0.02 m synthetic grid as well as at 0.1 m | robot radius is 1.1 cells at 0.1 m |
| C10 | Poses from `/tf` only, never from `/robot_K/odom` | stop-1 smaller points |

---

## 0. Decisions (settled)

| # | Decision | Value |
|---|---|---|
| D1 | A wall node is one observed **face**, not a physical slab. Back-to-back faces are paired later by an edge | face |
| D2 | Minimum wall length `l_min` | 0.5 m, pre-registered |
| D3 | Largest physical gap bridged inside a wall, `g` | `2*robot_radius` = 0.22 m, read from the nav2 params |
| D4 | Viewing directions stored as 5° bearing histograms | 72 bins |

### 0.1 Requirements quoted from `HANDOFF_phaseB2_schema_entry.md`

> §3.2 Node attributes: position in the robot's own frame, extent, observation count,
> observation geometry (the viewing directions from which it was seen).

> §3.3 [...] corroboration **cannot** be weighted by the number of observers. The
> robots share a coordinate frame, an odometry model, and a rangefinder model, so
> observations from similar directions carry correlated errors. Weighting must be by
> the geometric diversity of viewpoints.
>
> The schema must therefore record, per node, the set of directions from which it
> was observed — not merely a count. This is a schema requirement, not a later
> addition.

The schema does not define "observation count". This spec defines it as `n_scans`
(§5).

---

## 1. Facts confirmed at stop point 1

| Fact | Value | Where |
|---|---|---|
| Per-cell state | `classify(...)`; `OCC, FREE, UNKNOWN = 0, 254, 205` | `trinary_map.py:154`, `:69` |
| Map placement | `robot_divergence` refuses `map_T_odom` unless identity within 1e-6; origin = spawn + YAML origin; convention A inside `sample()` | `robot_divergence.py:248`, `:290`, `:304` |
| `rho` | 0.10000000149011612, all 20 maps | YAMLs |
| `robot_radius` | 0.11, all five robots, local and global | `nav2_robot0.yaml:168`, `:237`; robots 1–4 `:169`, `:238` |
| Replay bags | `experiments/logs/b2maps/b2maps_k{K}_cut{C}_slamin`, all 20 present | `run_b2maps_cuts.sh:108,503,665` |
| Raw scan topic | `/robot_{K}/scan` | `run_offline_maps.sh:631` |
| Replay `max_laser_range` | 7.9, all 20 | `offline_mapping_b2maps_k*_robot_*.yaml:53` |
| Scan geometry | 360 beams, 1.0023°, `range_min` 0.16, `range_max` 8.0, frame `robot_K/base_scan` | bag |
| SDF reader | `bag_overlap.world_walls_named()` | `bag_overlap.py:472` |
| SDF faces | 32 faces, 212.00 m; 16 end caps below `l_min`; 4 outward boundary faces unseeable | §6, A9 |

Import all of these. Recompute none.

---

## 2. Notation

- Cells `c`, resolution `rho`, state `s(c) ∈ {F, O, U}` from `trinary_map`.
- Cell centre `x(c) ∈ R²` in **the robot's own map frame** (YAML origin, convention A).
- `N4(c)`: the four edge neighbours of `c`.
- Oriented line, `theta ∈ [0°, 360°)`: direction `u = (cos theta, sin theta)`,
  normal `n = (−sin theta, cos theta)`, line `L(theta, d) = { x : x·n = d }`.
  The normal points to the free side.
- For a point `x`: distance `delta(x, L) = |x·n − d|`, position along `t(x) = x·u`.

---

## 3. Definitions (Part A: grid only)

**Def 1 — face element.** A pair `e = (c, c')` with `s(c) = O`, `c' ∈ N4(c)`,
`s(c') = F`. Its position is the shared cell boundary, `x(e) = (x(c) + x(c'))/2`.
Its normal is `n_e = (x(c') − x(c)) / rho`, a unit vector into free space. One
occupied cell may give up to four elements. An occupied–unknown boundary gives none:
nothing was seen from that side.

**Def 2 — wall segment.** A set `S` of face elements with oriented line `L` is a
wall segment when:

- **W1 collinear:** `delta(x(e), L) <= eps` for every `e` in `S`.
- **W2 continuous:** with positions sorted, `t_(1) <= … <= t_(m)`, every
  physical gap `t_(k+1) − t_(k) − rho` is `< g`.
- **W3 long:** `l(S) = t_(m) − t_(1) + rho >= l_min`.
- **W4 facing:** `n_e · n > 0` for every `e` in `S`.
- **W5 disjoint:** no element belongs to two segments. An occupied cell may give
  elements to several segments (a corner cell, a one-cell wall seen from both sides).

Extraction (§4) is greedy and deterministic. Segments are maximal with respect to
the extraction order only; no global optimum is claimed.

The reported line of `S` is the total-least-squares fit to its element positions,
keeping the orientation that satisfies W4. Endpoints `p0`, `p1` are the extreme
projections on that line, each extended by `rho/2`; `p0` has the smaller `t`.

**Def 3 — band.** For a line `L` and an interval of `t`, the band is every cell
whose centre lies within `b = rho*sqrt(2)/2` of `L` with projection in the interval:
the cells that touch the face, on both sides of it.

**Def 4 — gap state.** For each physical gap along `L` (inside a segment, bridged,
or between runs on the same line, splitting), take the band over the gap. Record
the counts of `F`, `O`, `U` on the occupied side (`(x(c) − x)·n < 0`) and on the
free side separately. The label is `free` if every band cell is `F`, `unknown` if
any is `U`, `mixed` otherwise. Bridged gaps are recorded on the segment, splitting
gaps on the line. The opening spec reads the splitting ones.

**Def 5 — endpoint state.** Beyond each endpoint, take the band over a length `g`.
In this order of precedence:

1. `occupied` if any band cell is `O` and gives no element to `S` (the face runs
   into other occupied mass: a corner or junction candidate);
2. `unknown` if any band cell is `U` (the robot stopped seeing here; this end is a
   limit of observation, not of the wall);
3. `free` otherwise (the wall visibly ends).

**Def 6 — unclassified occupied.** Occupied cells that give no element to any
segment. Reported as a count and as 8-connected components with their extents. Not
a category. Parked robots should land here.

---

## 4. Extraction procedure (Part A)

1. Compute all face elements.
2. Oriented Hough accumulator: `theta` in steps of `dtheta = 0.5°` over
   `[0°, 360°)`, `d` in bins of `rho`. Each element votes only for `theta` with
   `n(theta) · n_e > 0`. Peak detection may sum a `d` window of width `2*eps`; state
   what was chosen.
3. Take the highest unexhausted peak. Ties: smallest `theta`, then smallest `d`.
   Stop when the peak's votes fall below `l_min / (sqrt(2) * rho)`.
4. Candidates: unassigned elements within `eps` of the peak line and satisfying W4.
   Refit by total least squares; recompute the candidates once against the refit
   line.
5. Split the candidates into runs wherever the physical gap is `g` or more. Each
   run with `l >= l_min` becomes a segment: assign its elements and remove their
   votes. Shorter runs stay unassigned. Record the splitting gaps, with their Def 4
   states, on a line record.
6. Mark that bin exhausted, whatever step 5 produced, so the loop always ends. Go
   to 3.
7. For each segment compute its bridged gaps and endpoint states.

Any change to this procedure is reported before it is made.

---

## 5. Viewing directions (Part B)

**Scans.** From the cut's replay bag, read `/robot_K/scan`. Keep finite returns
with `range_min <= r < r_max`, `r_max = 7.9` read from the replay config. Returns at
or above `r_max`, and `inf` or `NaN`, never marked a cell occupied, so they are not
observations of a wall.

**Pose at each scan.**
`T_map_scan(t) = map_T_odom · odom_T_base(t) · base_T_scan`:

- `odom_T_base(t)` from the replay bag's `/tf`, never `/robot_K/odom`, interpolated
  at the scan header stamp (linear in position, shortest arc in yaw);
- `base_T_scan` composed from `/tf_static`:
  `base_footprint → base_link → base_scan`;
- `map_T_odom` exactly as `robot_divergence` reads and checks it.

A scan whose stamp falls outside the transform span is dropped and counted.

**Returns.** For beam angle `a` and range `r`: hit point
`p = T_map_scan · (r cos a, r sin a)`, sensor origin `o = T_map_scan · 0`,
bearing `beta = atan2(p − o)` in `[0°, 360°)`, map frame.

**Attribution.** A return belongs to segment `S` when
`delta(p, L_S) <= eps + rho/2`, `t(p)` lies within `[t(p0) − rho, t(p1) + rho]`,
and the ray came from the free side, `n_S · (o − p) > 0`. Ties go to the nearer
line, then the lower segment id. A return that meets the first two conditions for
some segment but the third for none is a **side contradiction** and is counted.

**Incidence.** `psi ∈ [0°, 90°]`, the angle between `o − p` and `n_S`.

**Per segment:** `n_scans` (distinct scans with at least one attributed return;
this is the observation count), `n_hits`, `bearing_hist` (72 bins of 5°),
`incidence_hist` (9 bins of 10°), `t_first` and `t_last` (sim seconds). Returns
within one scan are not independent, so `n_hits` is never the observation count.

---

## 6. Checks

Every check is shown failing with its named break before it is trusted. A
pre-registered prediction stays printed when it fails. Any threshold set after
seeing data is labelled CALIBRATED.

### Part A, synthetic (test suite, no corpus)

| # | Check | Break that must fail it |
|---|---|---|
| A1 | Four axis-aligned faces and one at 17°, 10 % of cells jittered one cell: every segment found, endpoints within `eps + rho`, angle within 1° | `eps = 0.4*rho` |
| A2 | A1's grid rotated 30° (nearest neighbour): angles shift 30° ± 1°, lengths within `2*rho`, same count. The only test of arbitrary angles, since the world's walls are axis aligned | restrict `theta` to multiples of 90° |
| A3 | Physical gaps: free `0.5g` (bridged, `free`), unknown `0.5g` (bridged, `unknown`), free `1.5g` (splits, line gap `free`) | `g` doubled |
| A4 | Disc of radius `robot_radius` (occupied rim, unknown inside, free outside), at 0.02 m and at 0.1 m: no segment, one unclassified component | `l_min = 0.1` |
| A5 | Face ending at free, at unknown, at a perpendicular face: `free`, `unknown`, `occupied` | swap precedence 1 and 2 |
| A6 | Two-cell wall free on both sides: two segments, opposite normals, lines 0.2 m apart. One-cell wall free on both sides: two segments, opposite normals, the cell giving one element to each | drop W4 |
| A7 | Same map twice: byte-identical JSON | seed-dependent tie-break |

### Part A, corpus (skip when `NSK_TRINARY_MAPS_DIR` is absent)

A8–A10 are **validation of the extractor only**. They are the one place world truth
enters this tool, and no later tool may read their outputs. The divergence measure
never references truth.

| # | Check | Prediction |
|---|---|---|
| A8 | Segments placed in the world by `robot_divergence`'s registration: share of element positions within `2*rho` of an SDF face | ≥ 95 % on all 20 maps, pre-registered |
| A9 | Recall: share of the 128.80 m of inward-facing SDF faces of length ≥ `l_min` lying within `2*rho` of a segment with a matching normal. Excluded faces are listed | reported per map; pre-registered: non-decreasing with cut for each robot, 5/5 |
| A10 | No segment element within `robot_radius + 2*rho` of a parked robot's spawn | 20/20, pre-registered |
| A11 | Each segment's angle residual to the nearest multiple of 90°, world frame | reported only; states what this corpus does not test |

### Part B

| # | Check | Prediction / break |
|---|---|---|
| B1 | Share of kept returns landing within one cell of any occupied cell | ≥ 90 % per map, pre-registered. Break: place with convention B |
| B2 | Side contradictions as a share of all kept returns | ≤ 1 %, pre-registered. Break: flip every normal |
| B3 | Scans read equal the replay's integrated count, within one | already 0–1 per cut at stop 1 |
| B4 | Synthetic scans of a known face from known poses give the known bearing histogram exactly | Break: omit `base_T_scan` (x offset −0.032 m) |

---

## 7. Output

One file per map: `experiments/logs/graph_walls/<map>.json`.

```json
{
  "map": "b2maps_k0_cut60_robot0",
  "map_sha256": "…",
  "frame": "robot_0 map",
  "resolution": 0.1,
  "thresholds": {
    "eps":   {"value": 0.1707, "unit": "m", "source": "pre-registered"},
    "b":     {"value": 0.0707, "unit": "m", "source": "pre-registered"},
    "g":     {"value": 0.22,   "unit": "m", "source": "nav2_robot0.yaml:168"},
    "l_min": {"value": 0.5,    "unit": "m", "source": "pre-registered"}
  },
  "lines":    [{"theta": 0.0, "d": 0.0, "segments": [0, 1],
                "gaps": [{"t0": 0.0, "t1": 0.0, "width": 0.0, "state": "free",
                          "occ_side": {"F": 0, "O": 0, "U": 0},
                          "free_side": {"F": 0, "O": 0, "U": 0}}]}],
  "segments": [{"id": 0, "p0": [0, 0], "p1": [0, 0], "theta": 0.0,
                "normal": [0, 1], "length": 0.0, "n_elements": 0,
                "bridged_gaps": [], "end_state": ["unknown", "occupied"],
                "obs": null}],
  "unclassified": {"n_cells": 0, "components": []},
  "checks": {}
}
```

Numeric values are placeholders except the thresholds. Part A writes
`"obs": null`; Part B fills it. Print a one-line summary per map: segment count,
total length, unclassified cells, endpoint-state counts.

---

## 8. Thresholds

| Name | Value | Source |
|---|---|---|
| `rho` | 0.1 m | map YAML, read |
| `eps` | `rho*(1 + sqrt(2)/2)` = 0.1707 m | pre-registered: a rasterised face at any angle stays within `rho*sqrt(2)/2` of its true line, plus one cell of rasterisation offset |
| `b` | `rho*sqrt(2)/2` = 0.0707 m | pre-registered: the cells that touch the face |
| `dtheta` | 0.5° over 360° | pre-registered |
| `g` | `2*robot_radius` = 0.22 m | nav2 params, read |
| `l_min` | 0.5 m | pre-registered (D2) |
| `r_max` | 7.9 m | replay config, read |
| bearing bins | 72 × 5° | D4 |

---

## 9. Commits and stop points

1. ~~§1 report.~~ Done.
2. Part A: `experiments/analysis/graph_walls.py` (no ROS import at module scope;
   bag reading imports inside functions, as the fitter does), tests A1–A7, then the
   corpus run A8–A11. Show every break failing. **Stop for review.**
3. Part B: viewing directions and B1–B4. **Stop for review.**

Every command block proposed states terminal, sim or not, blocking or not, and
expected wall time, and sources `/opt/ros/jazzy/setup.bash` and
`~/Desktop/NSKsim/install/setup.bash`. Tests run against scratch copies, never real
run artefacts.

---

## 10. Out of scope

Corners, openings, edges (adjacency, connectivity, containment, relative bearing,
back-to-back face pairing), the "same world" statement, the divergence measure, and
the knowledge/claim switch. The viewpoint-diversity weighting quoted in §0.1 is
defined in the divergence spec from `bearing_hist`; this spec only stores it.

---

## 11. Choices made while writing Part A

§4 requires that any change to the procedure is reported before it is made, and
§4 step 2 asks explicitly for the peak-detection window to be stated. These are
the choices Part A made, all reported to Ali and agreed before the code was
written. The first two are inside the latitude §4 grants; the third is a
deviation from A3's stated widths; the last two are decisions §6 and §7 left
open.

| # | Choice | Why |
|---|---|---|
| P1 | Peak detection sums a **3-bin** `d` window: `3*rho` = 0.30 m, inside the `2*eps` = 0.3414 m §4 allows | An axis-aligned face's `d` is `origin + k*rho`, so a map whose origin sits near a half-cell would split one wall's votes across two bins on float noise alone. The window also makes the stop threshold `l_min/(sqrt(2)*rho)` = 3.54 votes a statement about a neighbourhood rather than about one bin |
| P2 | The peak line's `d` is the **mean `d` of the live votes in the window**, not the bin centre | A closer seed for step 4's candidate selection. Step 4's TLS refit then fixes the reported line, so the final geometry does not depend on this choice |
| P3 | A3 uses hole widths of **1, 2 and 3 cells** (0.10 m = 0.45 `g`, 0.20 m = 0.91 `g`, 0.30 m = 1.36 `g`) rather than `0.5g` and `1.5g` | Physical gaps on a `rho` = 0.1 m lattice are whole cells, so 0.11 m and 0.33 m are not realizable. The realizable pair still straddles `g`: 1 and 2 cells bridge (free, and unknown), 3 cells splits. The named break — `g` doubled to 0.44 m — still turns the 3-cell split into a bridge and still fails the check. 2 cells is the tightest bridged case and is asserted as well |
| P4 | A8–A11 are written to **`experiments/logs/graph_walls/_validation_<stamp>.json`**, never into `<map>.json`. `<map>.json`'s `checks` holds truth-free self-checks only | §6 says no later tool may read their outputs. A number that is not in the per-map file cannot be read from it by accident, which is a stronger guarantee than a comment saying not to |
| P5 | A9's denominator stays the pre-registered **128.80 m**. The run prints, separately, that 1.2 m of it is unreachable: each inward boundary face runs 0.15 m into the perpendicular slab at each of its two corners (8 × 0.15 m), so that length is not an exposed surface of the union of the eight boxes | A9's pre-registered prediction is that recall is non-decreasing in the cut, which a constant floor cannot affect. Moving the denominator after the geometry was inspected would be exactly the tuning C6 fixed the number to prevent, so the floor is reported instead |

The next three were forced by what the corpus did, and each is reported with the
measurement that forced it.

| # | Choice | Why |
|---|---|---|
| P6 | A segment reports the TLS fit to its own elements (Def 2) **only when W1, W2 and W4 still hold under it**; otherwise it reports the line its run was cut on, which satisfies all three by construction. `line_source` says which, per segment; on the corpus it is 176 own fits and 44 fallbacks out of 220 | Def 2 asks for the fit to S's own elements, and that fit is usually better -- a 5.15 m wall cut on a peak 2.18° off the axis refits to 0.48°. But a short, thick run has no direction to find: candidates may lie anywhere within `eps` = 1.71 cells of the line, so four elements spanning 0.55 m form a blob whose thinnest axis is not the wall. On `b2maps_k0_cut1200_robot0` one such run was cut at 146.1° and refitted to 179.1°, and against that line its own elements were 1.30·`eps` off it with a 1.13·`g` gap between two of them. W1 and W2 are what make S a wall segment; a segment that fails them under its own reported line cannot be the output. With the fallback, W1 ≤ 0.999·`eps` and W2 ≤ 0.909·`g` on all 20 maps |
| P7 | Def 3's band takes a cell whose **centre** is within `b` of the line (unchanged, and what C5 is about) and whose centre is within the `t` interval **or whose own projected extent contains the whole interval** | A gap can be narrower than one cell -- see P8 -- and then no cell centre lies inside it. Under a centre-only rule Def 4 gets an empty band and reports nothing about a gap that is plainly inside one known cell. A cell's extent along `u` is `rho*(|ux| + |uy|)`, at most 0.141 m, so for any interval as wide as Def 5's `g` this clause can never fire |
| P8 | A gap is recorded only when it is at least **a quarter of a cell** wide | Two scales meet, three orders of magnitude apart. A line fitted 0.21° off the axis shortens every step to `rho*cos(0.21°)` and leaves 0.4 mm of "gap" between face cells that touch; recording those would hand the opening spec thousands of gaps that are nothing but the projection of a tilt. The smallest REAL gap is half a cell, for the reason in the note below. A quarter of a cell sits between the two with a factor of 60 below and a factor of 2 above, and no fraction of a cell can be a missing cell |

**A consequence of W4 as the spec defines it, reported rather than changed.**
W4 is `n_e · n > 0`. For an exactly axis-aligned line a perpendicular face --
a riser at a corner, a wall's end cap -- has dot exactly 0 and is correctly
excluded (in floating point cos(90°) is 6.1e-17, so the implementation excludes
anything below 1e-9, which restores the exact arithmetic rather than departing
from it). But a line fitted a hundredth of a degree off the axis gives the
risers on **one** side dot = +0.002, and W4 admits them. Two consequences, both
pinned by tests so that a later change to W4 shows up there:

* the gap in a FACE is up to half a cell narrower than the hole in the WALL,
  because the riser on the near side of the hole belongs to the face. Which side
  contributes it depends on the sign of a 0.002 tilt. The opening spec will have
  to say which of the two lengths it means;
* at a corner, the perpendicular face's riser is absorbed into the through
  face, whose endpoint then overshoots the corner by half a cell, and Def 5's
  band starts beyond the corner cell. Against a one-cell perpendicular wall
  that is enough to turn an `occupied` endpoint into an `unknown` one. The
  corner spec needs to know this; it is not corrected here, because the same
  looseness in W4 is what lets a 17° face's staircase risers belong to the face
  they are part of (A1).

The same looseness is why A11 is worth reading: at `l_min` = 0.5 m, four or five
leftover elements strung diagonally across a corner satisfy W1-W5 exactly as a
wall does. 38 of the corpus's 220 segments are under 1 m, and every angle
residual above 2.18° belongs to one of them.

---

## 12. What Part B measured, and the two predictions that failed

**B3 held on all 20 maps, exactly (+0 every time).** The reference is
free_space_relay's own `relayed N scans` line: the relay stood between the bag
and Karto, so its count *is* the integrated count, and the raw `/robot_K/scan`
this tool reads is its input. k1's cut1200 relayed 6001 where the others relayed
6000, which is the off-by-one the tolerance was written for.

**B1 failed on 4 of 20 maps** (77.16 %, 80.92 %, 82.05 %, 86.62 % against the
pre-registered 90 %) and **B2 failed on 1 of 20** (1.377 % against 1 %). Neither
threshold has been moved. Both failures have the same cause, and it is not in
this tool:

* on every one of the four B1 failures, **100 % of the off-wall returns lie
  within 2·rho of a real SDF wall, at a median distance of 0.000 m** — inside
  the wall. A return that misses every occupied cell of the map while sitting on
  a real wall says the MAP has no wall there; a misplaced return would miss the
  real wall too. On `b2maps_k1_cut1200_robot1` the robot parked 0.495 m from the
  east boundary wall for the run's second half and swept it with ~150 beams a
  scan: every one of those returns lands at x = 9.95 ± 0.01, the wall's true
  surface, and the map's column there holds 89 occupied cells out of 200.
* B2's contradictions concentrate the same way, read from the other end: 90 % of
  `b2maps_k2_cut1200_robot2`'s belong to three 0.70 m segments with no
  back-to-back twin. The robot saw those walls from the side whose face the map
  never recorded, so the rays match a face's line and extent and no face's side.

Why those cells are FREE rather than never seen is diagnosed in `experiments/logs/graph_walls/diagnose_dwell.json` (P3 held, P1 and P2 failed: dwell explains two of the four failures, not all four), then in `diagnose_karto.json` (Karto's counters rebuilt exactly, 20/20 maps; measured by PASSES, dwell recommits 84 % and 69 % of the off-wall cells on k1 and k3 cut1200) and `diagnose_phase.json` (the grid-phase hypothesis, not supported).

**What B3 actually compared.** B3 measured the stripped bag's scan count against
free_space_relay's `relayed N scans`, and the relay sits *upstream* of
slam_toolbox — so B3 compared **scans received**, not scans that reached the
grid. Its recorded verdict stands as it is; what it is evidence about is
narrower than it looks. The chain, measured on `b2maps_k1_cut1200_robot1`:

| | scans | why the loss |
|---|---|---|
| read from the bag | 6001 | what the relay delivered, which is what B3 checked |
| pose-valid | 6000 | one scan's stamp fell outside the `/tf` span, so no pose could be interpolated for it; Part B counts it as `n_scans_dropped_no_pose` |
| reaching Karto's grid | 5997 | `SlamToolbox::shouldProcessScan` drops the 2nd, 3rd and 4th scans of the run — `if (scan_ctr < 5) return false`, slam_toolbox_common.cpp:795-797 |

Karto itself rejects none of them: `Mapper::HasMovedEnough` returns true on its
first test, `timeInterval >= m_pMinimumTimeInterval` (Mapper.cpp:3155-3158), and
the replay config sets `minimum_time_interval: 0.0` — so a scan from an identical
pose **is** integrated, and the travel and heading tests are never reached.

**The grid phase converges by cut1200.** Karto's grid offset is
`boundingBox.GetMinimum()` over the processed scans (Karto.h:6113), not a lattice
snap, so each map's cell edges sit at their own sub-cell phase. Across the five
robots that phase differs by 0.025–0.045 m at cuts 60/120/240 — but by only
0.0067 m in x and 0.0040 m in y at cut1200, because once every robot has covered
the whole arena every bounding box is the same rectangle. **A phase artefact at
cut1200 would therefore be common to all five maps rather than independent across
them**, which matters for anything that reads agreement between robots as
evidence. Measured in `diagnose_phase.json`.

**The residual: 2 of the 20 maps are unexplained, and the diagnosis ends here.**
Four candidates were tested against the off-wall cells, each by removing one
source's passes from an exact rebuild of Karto's counters:

| map | B1 | off-wall cells | (a) relay fill | (b) dwell | (c) grazing | (d) range edge | best |
|---|---|---|---|---|---|---|---|
| k0_cut240 | 80.9 % | 188 | 2.1 % | 0.0 % | 0.0 % | 0.0 % | **2.1 %** |
| k1_cut1200 | 77.2 % | 112 | 10.7 % | **83.9 %** | 38.4 % | 0.0 % | 83.9 % |
| k3_cut1200 | 86.6 % | 78 | 17.9 % | **69.2 %** | 0.0 % | 0.0 % | 69.2 % |
| k4_cut120 | 82.1 % | 131 | 0.8 % | 0.0 % | 0.0 % | 0.8 % | **0.8 %** |

Dwell accounts for the two long-dwell failures. **k0_cut240 and k4_cut120 are
accounted for by nothing**: no single source recommits more than 2.1 % of their
off-wall cells, and the grid-phase hypothesis failed on them too (0.0 % of their
off-wall cells lie on faces within 0.01 m of a cell edge). What is measured about
them is the arithmetic rather than the cause: those cells are **passed through 17
to 28 times more often than they are struck** — 1845 hits against 78,470 passes
on k0_cut240, 1139 against 31,406 on k4_cut120 — giving hit ratios of 0.023 and
0.035 against the 0.1 `occupancy_threshold`. The passes are ordinary finite rays
at ordinary ranges (median 4.8 m and 6.3 m from the sensor), with only 0.6–1.1 %
of them in the 7.9 m clipping band. By the stopping rule agreed before the last
run, this is recorded as unexplained and no fifth candidate is pursued. One
candidate is named and **UNTESTED, not to be tested now**: possible smear of long-range returns during fast spins (pose/scan timing); k0_cut240 and k4_cut120 are early cuts. Across
all 20 maps the residual is 938 off-wall cells; `diagnose_edge.json` carries the
per-map figures.

**Why break (i) of the rebuild gate does not fail on four maps.** Dropping the
relay-filled rays fails the gate on 16 of 20 maps (agreement 64–88 %) but not on
k0/k1/k2/k4 cut1200 (99.95–99.99 %). On a long-dwell map the parked pose has
inflated the counters so far that removing any single source moves almost no cell
across the ratio — which is H_relay's own finding (0.8–17.9 % recommitted), not a
weakness in the rebuild. The rebuild reproduces every map at 100.00 % (one at
99.99 %), and break (ii) fails the gate on 20/20.

So the wall predicate can only find faces the map committed, and these grids do
not contain every real wall face. That is a finding about the corpus — and the
same shortfall bounds A9's recall — not a reason to lower a pre-registered
number. The validation file carries the diagnosis per map as
`B1_off_wall_vs_truth`; it uses truth, so it never enters a `<map>.json`.

| # | Choice | Why |
|---|---|---|
| P9 | Each map gains an **`observations`** block: the bag, the scan topic and why it is the raw one, `r_max` with its source, the scan geometry, the pose source, the `/tf` span and sample count, `base_T_scan` with both hops, `map_T_odom`, and the return accounting (read, dropped for want of a pose, kept, at-or-over `r_max`, infinite, attributed, contradictory, off-wall, outside the map) | §7 fixes what a segment carries and says nothing about the map level. Part B's numbers are meaningless without the provenance of the bag and the chain that placed them, and the divergence spec will need to know a map's returns were read at all |
| P10 | Each segment's `obs` gains **`n_contradictions`**: contradictions whose nearest line is that face, by §5's own tie rule (nearer line, then lower id) | §5 counts contradictions and does not attribute them. Counting them per face costs no new threshold and is what turned "B2 = 1.377 %" into "three short faces whose opposite side the map never recorded" |
| P11 | `--break no-base-scan` is kept even though it does **not** break B1 (98.92 % against 99.97 %) | That is the finding. 0.032 m is a third of a cell and B1 forgives a whole one, so the corpus cannot see the error at all. It is also why B4 is a synthetic check asserting hit positions to 1e-9 rather than a corpus one — and why B4's own assertion is about positions: `p - o = R_map_scan · (r cos a, r sin a)`, so a pure sensor translation cannot rotate a ray and the bearing histogram is invariant to it |

**Four faces out of 220 have no viewing directions at all** (`n_scans` 0):
0.70–0.90 m fragments of 3–8 elements, all reported on the fallback line, and
all but one with `occupied` at *both* endpoints — wedged between other occupied
mass, so the returns that struck those cells went to the longer crossing faces
by §5's nearer-line rule. The same population A11 flags. The divergence spec
must therefore handle a node whose bearing histogram is empty: it has an extent
and a position but nothing to weight corroboration with.

**A note on `--break convention-b`.** It fires — B1 drops 99.97 % → 85.40 % and
fails — but not catastrophically, and the reason is worth recording. In
`b2maps_k4_cut1200_robot4` the two long boundary walls sit at map y = +10.70 and
y = −9.10, and a vertical flip maps each to within 0.16 m of the other, so the
walls that carry most of the returns land nearly on top of themselves. This is
the same effect `robot_divergence.py:174-206` documents for its swap gate:
in an axis-aligned, nearly symmetric arena a wrong placement can leave long
walls overlapping themselves, and an absolute threshold on the result silently
assumes it away.
