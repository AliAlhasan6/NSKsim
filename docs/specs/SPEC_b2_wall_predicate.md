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

### 12.1 The gated batch, reported only

`compare_gated_maps.py` on the travel-gated batch: **C1 20/20, C2 5/5 at 100 %**,
and the `--break ungated-prediction` run failed C2 on 5/5 cut1200 maps as
pre-registered. `experiments/logs/b2maps_gated/comparison.json` carries it.

Reported and **not to be chased**: k2's short cuts are the only maps where C2
comes out under 100 %, by 3, 1 and 1 cells at cut60, cut120 and cut240. The short
cuts are the ones C2 was written not to judge — the gated and ungated predictions
differ there by a handful of cells, so a single cell is a whole percentage point —
and three cells on one robot is not a finding.

### 12.2 The half-cell fix, and what it did to the numbers in §12

`slam_toolbox` publishes Karto's grid offset verbatim as the map origin — `toNavMap`
assigns `map.info.origin.position = occ_grid->GetCoordinateConverter()->GetOffset()`
(`include/slam_toolbox/visualization_utils.hpp:108-129`, the assignment at
:123-124) — and Karto centres cell *i* on that offset: `GridToWorld` returns
`m_Offset + rGrid/m_Scale` and `WorldToGrid` inverts it with `math::Round`
(`karto_sdk/include/karto_sdk/Karto.h:4421-4436`, `:4444-4458`). Every
ROS-convention reading, `origin + (i+0.5)*res`, therefore placed a cell 0.05 m too
far in +x and +y. That is now one constant, `trinary_map.CELL_CENTRE_OFFSET = 0.0`,
with three helpers (`cell_centre`, `cell_low_edge`, `cell_index`); every
index-to-position site in the project routes through it and the one hand
compensation — `compare_gated_maps`' `-rho/2` on each origin — is gone.

**What did not move, and why.** `robot_divergence`'s whole output is byte-identical,
as is `compare_gated_maps`' `comparison.json` apart from its new `provenance`
field, and the fitter's on-wall scores are identical to three decimals on 5/5
cut1200 maps in both conventions. The reason is arithmetic, and it is worth
stating because it bounds what any of these checks can ever prove:
`trinary_map.sample` asks for `x = x0 + (i + c)*res` and answers with
`floor((x - origin)/res + 0.5 - c)`, so changing `c` moves the query lattice and
the lookup together and cancels, and a shift applied to `x0` and `origin` alike
cancels too. **Any measurement whose every lattice is named relative to another
lattice is invariant to this convention.** That is why the hand compensation in
`compare_gated_maps` was inert: removing it could not change `comparison.json`,
and the pre-registered break that reinstates it on BOTH origins does nothing. A
live break had to be added, `--break half-cell-source-only`, which shifts the
source origin alone: it fails the resampler self-check on 20/20 and moves C1 from
100 % to 90.5–95.2 %.

**Part A's extraction barely moves, and the one place it does is named.** A uniform
translation of every element leaves the Hough accumulator almost alone — each
element's `d` shifts by `t · n(theta)`, which varies with theta, so the per-theta
bin phase is not quite preserved. Measured over the 20 ungated maps: **total
segment length within 1 % on 20/20** (worst 0.626 %, 14 of 20 under 0.002 %), and
**segment count unchanged on 19/20**. The exception is `b2maps_k2_cut240_robot2`,
11 → 10 segments and −0.5955 m, and it is one segment: a **0.6002 m, 7-element
fragment on the fallback `extraction_line`**, whose elements were absorbed into the
7.15 m walls beside it. That is the l_min-floor corner population A11 already
flags, not a wall. Element count identical (1031), iterations 4567 → 4614. By the
letter of the pre-registration — segment count within 1 % on 20/20 — this **FAILS
on 1 of 20**, since one segment out of 11 is 9 %; the threshold has not been moved.

The convention bites in exactly one place: where a **world coordinate** meets a
**cell index**. Three such sites, and all three moved.

* **the fitter.** `cell_points` lifts occupied cells to points, so the fitted
  translation moves by exactly **+0.0500 in dx and +0.0500 in dy** on 5/5 cut1200
  maps in both conventions, with the score, theta, the resolved convention and the
  A–B margin all unchanged. The half cell, recovered and visible.
* **A8**, which cannot adjudicate this convention on this world, and the
  pre-registered prediction that its median would fall on 20/20 **FAILED: it falls
  on 15/20 and rises on 5/20**, and the threshold has not been moved. The reason is
  measured: this world's wall surfaces lie at x and y phases **0.0 and 0.05 modulo
  rho** (inner faces at ±9.95/±4.1/±3.9/±1.0/±6.0, outer at ±10.05/±10.1), and a
  half-cell translation maps the set {0.0, 0.05} onto itself. A8's
  `share_all` is byte-identical on 20/20 in consequence. Over all 14 693 assigned
  elements the mean distance to a wall improves slightly (0.0762 → 0.0714 m) while
  the share sitting exactly on a surface drops (4.02 % → 1.40 %); the median hops
  between discrete distance levels and goes both ways. A8 is a test of whether the
  cell edges happen to coincide with the wall surfaces, not of where the cells are.
* **B1**, which bites hardest, and which turns up the explanation §12 stopped
  looking for.

**B1 and the far edge of Karto's own grid.** Under the corrected binning B1 falls
on 20/20 maps, by up to 43 pp, and 16/20 holding becomes 9/20. It is not the fix
being wrong. `OccupancyGrid::ComputeDimensions` (`Karto.h:6090-6114`) sets
`rOffset = boundingBox.GetMinimum()` and `rWidth = Round(size*scale)`; under the
centre semantics `WorldToGrid` actually uses, a grid of W cells covers
`[O - res/2, O + (W - 0.5)*res)`, while the scans span `[O, O + size]`. On
`b2maps_k2_cut1200_robot2` that span is 200.117 cells against W = 200, so **the
last 0.6 cell of Karto's own bounding box has no cell to put a return in** — the
same class is internally inconsistent about this, since `CoordinateConverter::
GetBoundingBox` (`:4536-4548`) treats the offset as the grid's minimum CORNER.
Measured consequences:

| | k0_cut240 | k4_cut120 | k1_cut1200 | k3_cut1200 |
|---|---|---|---|---|
| B1 as §12 recorded it | 80.9 % | 82.1 % | 77.2 % | 86.6 % |
| B1, corrected binning | 76.1 % | 81.1 % | 47.8 % | 58.2 % |
| returns rounding OFF the grid | 23.4 % | 18.3 % | 50.8 % | 40.7 % |
| …of those, in the last row or column | 100.0 % | 100.0 % | 100.0 % | 100.0 % |
| **B1 among returns that reached the grid** | **99.37 %** | **99.20 %** | **97.17 %** | **98.15 %** |
| §12's off-wall returns that round off the grid | 97.3 % | 96.2 % | 94.0 % | 91.9 % |

B1 restricted to returns that reached the grid is **97.17–100.00 % on 20/20 maps**,
and the four B1 failures of §12 are not among them. 91.9–97.3 % of the very
returns §12 called off-wall — **including both maps §12 recorded as unexplained** —
are returns whose Karto cell index equals the grid width. A hit there is skipped
by `TraceLine`; the passes along the same ray stop at W-1 and are kept; passes
without hits is precisely Karto's FREE. `diagnose_karto`'s rebuild reproduced
every map at 100.00 % while dropping exactly those endpoints, which is why nothing
in the three diagnosis files could see it: all of them inherited the drop.

It arrived as a by-product of the half-cell fix rather than from a pre-registered
test, so §12.3 records what has been ruled on it and what has not.

The 20 regenerated `<map>.json` files carry B1 under the corrected binning, each
with a `provenance.cell_centre` block naming the convention. The previous run's
values are the ones quoted in §12 above.

### 12.3 Rulings on §12.2, by Ali

**B1 is redefined, POST HOC, and the label travels with it.** A return that falls
outside the map's grid cannot be checked against the map: there is no cell where it
landed to be occupied or free. So

> **B1 = the share of IN-GRID kept returns within one cell of an occupied cell.**

The **threshold stays at 90 %**, unmoved. Every map also carries `B1_offgrid`, the
share of kept returns that round off the grid, and
`B1_on_occupied_share_all_returns`, the original all-returns figure — so nothing is
hidden and the old number is recoverable from any JSON. `B1_redefined` in each
`checks` block says POST HOC in those words, and `obs_summary_line` prints all
three. The change is a change of DENOMINATOR made after seeing data, for the stated
reason above, and it is not evidence for anything by itself.

Two consequences inside the tool, both deliberate:

* `n_returns_off_wall` and the `off_wall_sample` the A8–A11 pass diagnoses are now
  **in-grid misses only**. The whole point of that diagnosis is to separate "the
  map has no wall where a wall is" from "the return is misplaced", and an off-grid
  return is neither. `n_returns_off_wall_all` keeps the old count.
* `--break convention-b` still bites: a row flip moves the walls, not a return's
  grid membership, so the denominator is untouched and only the hits fall. Pinned
  by test.

**Measured, on the 20 ungated maps regenerated under the new definition: in-grid B1
HOLDS 20/20, worst 97.17 %.** The all-returns figure would hold on 9/20, and the
worst off-grid share is 50.78 %. Every map's `checks` carries `B1_definition`,
`B1_redefined` (the words POST HOC), `B1_on_occupied_share_all_returns`,
`B1_offgrid`, and the three counts behind them; `obs_summary_line` prints all three
shares. B2 and B3 are unaffected except by the half-cell fix itself: B2 still fails
on 1 of 20, `b2maps_k2_cut1200_robot2` at 1.352 % against the 1.377 % §12 records,
and B3 is +0 on 20/20.

**A8 under K3: FAILED and uninformative.** The median falls on 15/20 and rises on
5/20 against a pre-registered 20/20. This world's wall surfaces sit at phases 0.0
and 0.05 modulo rho, so a half-cell shift maps the set onto itself and A8's
`share_all` is byte-identical on 20/20. **That is a flaw in the test design, not a
finding about the maps or the extractor.** A8 remains a valid check that elements
land on walls at all; it is not a check of where a cell sits, and no future change
of that kind should be judged by it. K3's count miss stands as recorded:
`b2maps_k2_cut240_robot2`, 11 → 10 segments, one 0.6002 m l_min-floor fragment.

**`frontier_explorer`'s `/map` reading stays as it is** — `origin + (i+0.5)*res` at
`frontier_explorer.py:1450`, `:1524`, `:1528`, `:2456`, `:2459`. It is the robot's
own control loop, so half a cell there is **behaviour, not measurement**, and the
one definition in `trinary_map` does not reach into it by design.

**§12's residual is reclassified.** The four B1 failures —
`b2maps_k0_cut240_robot0`, `b2maps_k4_cut120_robot4`, `b2maps_k1_cut1200_robot1`,
`b2maps_k3_cut1200_robot3` — are **no longer unexplained**. The candidate cause is
**the grid extent**: Karto's width and height are `Round(size*scale)` with
centre-semantics indexing, so the grid stops short of its own bounding box and hits
on the max-x and max-y walls are discarded while the passes along the same rays are
kept. 91.9–97.3 % of the returns §12 called off-wall are returns whose cell index
equals the grid width, and in-grid B1 on those four maps is 99.37 / 99.20 / 97.17 /
98.15 %. The two maps §12 recorded as accounted for by nothing — `k0_cut240` and
`k4_cut120` — are included. §12's stopping rule ("no fifth candidate is pursued")
is satisfied rather than broken: this candidate was not found by searching for one.

The spin-smear line of §12 **stays, and stays marked UNTESTED**: possible smear of
long-range returns during fast spins (pose/scan timing). It is not retracted and it
is not tested.

§12.4 is the pre-registered test of the grid-extent candidate.

### 12.4 The grid-extent candidate, pre-registered and tested

`experiments/analysis/diagnose_extent.py` → `experiments/logs/graph_walls/diagnose_extent.json`.
Truth-derived, diagnosis only, reported and never gated, like A8–A11. 40 maps in
214 s. X1 and X2 were printed before any number was read, and were judged on **all
20 ungated and all 20 gated maps** — the strictest reading of "every map" — with the
two subsets reported separately so neither could be chosen afterwards.

**Both held.**

**1. How the dimensions are set, and nothing pads them.**
`OccupancyGrid::ComputeDimensions` (`Karto.h:6089-6113`) takes the union of the
processed scans' bounding boxes (`:6097-6105`) and sets
`rWidth = Round(size.GetWidth()*scale)` (`:6111`), `rHeight` likewise (`:6112`),
`rOffset = boundingBox.GetMinimum()` (`:6113`), with `scale = 1/resolution`
(`:6108`); `math::Round` is `v >= 0 ? floor(v+0.5) : ceil(v-0.5)` (`Math.h:87-90`).
The static `CreateFromScans` (`:5947-5964`) passes those three straight to the
constructor (`:5910-5930`) — **no border, no margin, no rounding up** — and
`Grid::CreateGrid` (`:4586-4594`) takes no `borderSize`. The one thing that looks
like padding is not addressable: `Grid::Resize` (`:4636-4664`) sets
`m_WidthStep = AlignValue(width, 8)` (`:4640`, `Math.h:234-237`), an 8-aligned row
**stride** for the data array, while `m_Width` stays `Round(size*scale)` and
`IsValidGridIndex` tests `m_Width` (`:4671-4674`).

So with `u = size/res` and `W = Round(u)`, and `WorldToGrid` rounding about the
offset, a grid of `W` cells covers `[O − res/2, O + (W − 0.5)·res)` while the scans
span `[O, O + size]`. The far-edge shortfall is `u − Round(u) + 0.5` cells, **which
lies in [0, 1) for every u**: under its own indexing Karto's grid always stops short
of its own bounding box.

Measured, and it is not a corner case — **every one of the 40 maps has a shortfall**:

| | min | median | max |
|---|---|---|---|
| shortfall in x, cells | 0.045 | 0.675 | 0.984 |
| shortfall in y, cells | 0.006 | 0.703 | 0.973 |

all in [0, 1) as derived, and 0.001–0.098 m. The recomputation reproduced each
map's **saved origin, width and height exactly, 40/40** (`karto_extent` imported
from `predict_gated_maps` unchanged; a map it could not reproduce would have been
refused, not reported), so each shortfall is that map's own. Gating changes the
bounding box on only two maps and only in y — `k2_cut60` 0.629 → 0.601 cells,
`k4_cut1200` 0.6915 → 0.6894 — because the gate drops scans from poses the robot
had already occupied, and a duplicate pose cannot extend a bounding box.

**2. A9 recall splits by side, hard.** Max side = `wall_east.west` +
`wall_north.south`; min side = `wall_west.east` + `wall_south.north`. On the ten
maps where all four boundary faces were observed — which is exactly the ten cut1200
maps, and that is X2's reach — the max-side mean recall is below the min-side mean
on **10/10**:

| map | max side | min side |
|---|---|---|
| k0_cut1200 | 62.33 % | 99.51 % |
| k1_cut1200 | 73.14 % | 99.63 % |
| k2_cut1200 | 70.52 % | 99.26 % |
| k3_cut1200 | 81.51 % | 99.11 % |
| k4_cut1200 | 97.85 % | 99.53 % |
| k0_cut1200 gated | 77.52 % | 98.91 % |
| k1_cut1200 gated | 97.03 % | 99.51 % |
| k2_cut1200 gated | 81.11 % | 98.64 % |
| k3_cut1200 gated | 97.97 % | 99.11 % |
| k4_cut1200 gated | 98.34 % | 99.03 % |

**3. The mechanism, three orders of magnitude apart.** Share of the returns that
struck each boundary face which fell off the grid, over the maps where that face
was seen at all:

| inward face | maps | min | median | max |
|---|---|---|---|---|
| `wall_east.west` (max x) | 24 | 17.46 % | 77.54 % | 98.84 % |
| `wall_north.south` (max y) | 22 | 55.46 % | 79.62 % | 97.73 % |
| `wall_west.east` (min x) | 20 | 0.0095 % | 0.0330 % | 0.4179 % |
| `wall_south.north` (min y) | 30 | 0.0000 % | 0.0261 % | 0.0623 % |

**X1 — HELD.** On **40/40** maps, the share of off-grid returns lying within 2·rho
of an SDF face is **exactly 1.000000** — not merely above 0.95, but every off-grid
return on every map is on a real wall. On the 10 cut1200 maps, the share lying on
the east or north boundary face is **99.71–99.99 %**, holding 10/10. Ungated 20/20,
gated 20/20.

**X2 — HELD.** Max-side mean recall below min-side mean on **10/10** (100 %) of the
maps where all four boundary faces were observed, against the pre-registered 80 %.
Ungated 5/5, gated 5/5. **Limit on its reach, stated rather than glossed:** only the
cut1200 maps observed all four boundary walls, so X2 is a statement about five
robots at one cut and its replication on their gated twins, not about 40 independent
cases.

**What this closes, and what it does not.** The hypothesis is confirmed as stated:
Karto's grid stops short of its own extent on every map; the hits on the max-x and
max-y walls are discarded there while the passes along the same rays are kept; and
those walls are measurably missing from the map, by A9 recall and by B1 alike. §12's
four B1 failures and its two "unexplained" maps are accounted for by this. It does
**not** establish that nothing else contributes — the max-side recall is 62–98 %
rather than 0 %, so the far-edge band removes part of each wall and not all of it —
and the spin-smear candidate of §12 remains recorded and **UNTESTED**.

One result bears directly on §12.5: the travel gate **raises** max-side recall on
every cut1200 map — 62.33 → 77.52 %, 73.14 → 97.03 %, 70.52 → 81.11 %,
81.51 → 97.97 %, 97.85 → 98.34 % — because dropping the parked duplicate scans
removes passes from the far-edge cells without removing the hits that did land at
W−1, which moves those cells back across the occupancy ratio.

### 12.5 Task 2, re-registered under the new B1 before it runs

Pre-registered now, after §12.3's redefinition and §12.4's result and before any
gated Part B number exists. The earlier registration (B1 ≥ 90 % on k1 and k3
cut1200, < 90 % on k0_cut240 and k4_cut120, and graph_walls within 0.5 pp of
`predict_gated_maps`) was written against the all-returns denominator and is
**superseded** — it is not carried forward and not re-tested.

| # | Prediction |
|---|---|
| **T1** | In-grid B1 ≥ 90 % on 20/20 gated maps |
| **T2** | On k1 and k3 cut1200, gated in-grid B1 ≥ ungated in-grid B1 (97.17 % and 98.15 %) |
| **T3** | `predict_gated_maps`' B1 recomputed under the new definition agrees with `graph_walls`' within 0.5 pp on 20/20 |

**T3 is doable, and what it does and does not test.** `predict_gated_maps` does not
carry its own binning: it calls `graph_walls.on_occupied_share` directly
(`predict_gated_maps.py:316`), so re-running it picks up the in-grid denominator
with no code change and writes `b1_predicted` under the new definition. The two
numbers are therefore never a test OF the binning — one function computes both.
What T3 tests is the prediction: `predict_gated_maps` measures B1 on the grid it
predicted at the origin it predicted, from the admitted scans' returns, while
`graph_walls` measures it on the saved gated grid at the saved origin from the gated
bag's returns. Agreement within 0.5 pp says those are the same map.

T1 and T2 need Part B on a `_gated` stem, which `obs_for_map` still refuses: it
derives the bag, the replay config and the relay log from (robot, cut) alone and
would read the ungated ones. §12.4's tool resolved the first two with local helpers
(`diagnose_extent.variant_bag`, `variant_r_max`, `variant_map_to_odom`) and needs no
relay log; Part B needs all three, so that plumbing is Task 2's first step and is
not done here.

### 12.6 The extent fix, predicted. Nothing patched, nothing built

**1. The patch, specified and not applied.** `docs/patches/karto_extent.patch`, which
adds `+ 1` to each of `ComputeDimensions`' two dimensions (`Karto.h:6111-6112`) and
changes nothing else. Verified to apply with `patch -p1` at zero fuzz against a
scratch COPY; `~/src/slam_toolbox_2.8.5` is untouched and still reads
`Round(size.GetWidth() * scale)` at `:6111`, so every line number cited anywhere in
`docs/` still resolves.

Why +1 is **sufficient**: with `O` the offset, `u = (x − O)/res` and
`U = size/res`, `WorldToGrid` gives index `Round(u)`; `u` runs over `[0, U]`; so the
largest index any bounding-box point can produce is `Round(U)`. Valid indices are
`[0, W)`, and `W = Round(U) + 1` makes `Round(U)` the last valid one.

Why it is **not more than needed**: index `Round(U)` IS attained — by the extreme
point that set the bounding box — so the added row and column are not dead. `+2`
would add a row and column that no endpoint and no traced pass could reach, and they
would stay Unknown for the life of the grid. The offset is untouched, so no existing
cell moves: cell `i` keeps its centre at `O + i·res`.

**What the patch does NOT fix, stated so the claim is not over-read.** A reading at
or above `rangeThreshold` is rescaled to exactly `rangeThreshold` along its own beam
(`AddScan`, `:6173-6179`) and that synthesised endpoint is **not a bounding-box
point**: the box is built from the sensor pose and the FILTERED readings only
(`:5696-5700` over `m_PointReadings`, filtered at `:5662` by
`InRange(r, minRange, rangeThreshold)`), while `AddScan` traces
`GetPointReadings(false)` = the unfiltered ones (`:5613-5623`). No change to
`ComputeDimensions` can bound those. They carry `isEndPointValid = false` (`:6167`),
so losing their tails costs free-space marking and never a wall — which is why E1,
about *kept* returns, can hold while this remains true.

**2. The prediction.** `predict_gated_maps.py --extent-fix`, default off. With the
flag absent all 40 PGM/YAML files in `b2maps_gated/predicted/` are reproduced
**byte-identically** (`cmp` on all 40). Its `_summary.json` gains three reported
fields — `b1_predicted_all_returns`, `b1_offgrid` and two counts — which follow from
§12.3's B1 and are not a prediction change. With the flag on it writes to
`experiments/logs/b2maps_extent/predicted/`, which cannot collide with the
predictions C1 and C2 were judged against; `--extent-fix` and `--check` are refused
together, since `--check` asks the UNPATCHED extent to reproduce maps the unpatched
Karto built.

**3. All three pre-registered predictions HELD.** Printed before the numbers, in both
tools.

| | verdict | |
|---|---|---|
| **E1** off-grid kept returns = 0 | **HELD 20/20** | every map exactly 0.000 % |
| **E2** overlap identical to the gated prediction | **HELD 20/20** | 0 differing cells, origin bit-identical, exactly +1 per axis |
| **E3** max-side A9 recall within 2 pp of min-side, cut1200 | **HELD 5/5** | worst gap 1.06 pp |

Reported without a threshold, as asked: in-grid B1 and all-returns B1 are **equal on
20/20**, which is what E1 implies, and the worst of them is **99.98 %** against the
97.17 % the unpatched maps give.

**The added row and column are not empty, which E2 alone would not have shown.** On
each of the five cut1200 maps **all 402** added cells come out OCCUPIED — the
boundary walls, appearing in full, with not one free or unknown cell among them. The
short cuts behave as they should: a robot that never reached a boundary wall adds
mostly unknown.

| | occupied | free | unknown |
|---|---|---|---|
| cut60, 5 maps | 98 | 291 | 828 |
| cut120 | 410 | 224 | 918 |
| cut240 | 1327 | 117 | 392 |
| cut1200 | **2010** | **0** | **0** |

And E3 against §12.4's measurement of the same quantity on the unpatched maps:

| map | max side, unpatched | max side, patched | min side, patched |
|---|---|---|---|
| k0_cut1200 | 62.33 % | 99.48 % | 98.42 % |
| k1_cut1200 | 73.14 % | 98.99 % | 98.61 % |
| k2_cut1200 | 70.52 % | 99.03 % | 98.64 % |
| k3_cut1200 | 81.51 % | 98.96 % | 98.61 % |
| k4_cut1200 | 97.85 % | 98.34 % | 99.03 % |

The gap between the sides closes from 1.7–37.2 pp to 0.35–1.06 pp. **This is a
prediction, not a measurement of slam_toolbox**: no slam_toolbox was compiled or
run. `experiments/logs/graph_walls/diagnose_extent_fix.json` carries E2, E3 and the
B1 pair; `b2maps_extent/predicted/_summary.json` carries E1.

### 12.7 Option A: a patched slam_toolbox in an overlay. Plan only, nothing built

**The source tree stays pristine.** `~/src/slam_toolbox_2.8.5` is the citation
authority for every `Karto.h:` line number in this document, so the overlay patches
a COPY and never that tree:

```
~/src/slam_toolbox_ws/                     # outside the repo; nothing here is committed
  src/slam_toolbox/                        # cp -a of ros-jazzy-slam-toolbox-2.8.5
  build/ install/ log/                     # colcon output
```

```
mkdir -p ~/src/slam_toolbox_ws/src
cp -a ~/src/slam_toolbox_2.8.5/ros-jazzy-slam-toolbox-2.8.5 \
      ~/src/slam_toolbox_ws/src/slam_toolbox
cd ~/src/slam_toolbox_ws/src/slam_toolbox
patch -p1 --dry-run < ~/Desktop/NSKsim/docs/patches/karto_extent.patch   # expect no fuzz
patch -p1          < ~/Desktop/NSKsim/docs/patches/karto_extent.patch
cd ~/src/slam_toolbox_ws && colcon build --packages-select slam_toolbox \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The patch lives in the repo at **`docs/patches/karto_extent.patch`**, so the build is
reproducible from a clean checkout plus the orig tarball, and the diff is reviewable
without a build. It carries its own rationale, the SHA256 of the tarball it is
against, and the note that it is not applied upstream of itself.

**Dependencies: nothing to install.** `rosdep install --from-paths
~/src/slam_toolbox_2.8.5/ros-jazzy-slam-toolbox-2.8.5 --ignore-src --rosdistro jazzy
--simulate -y` prints **nothing and exits 0** — every dependency is already
satisfied. Forcing the full list with `--reinstall --simulate` resolves 11 apt keys,
all system libraries (`libeigen3-dev`, `libboost-all-dev`, `libsuitesparse-dev`,
`liblapack-dev`, `libceres-dev`, `libtbb-dev`, `libqt5core5t64`,
`libqt5widgets5t64`, `libqt5gui5t64`, `libqt5opengl5t64`, `qtbase5-dev`), and
`dpkg -s` confirms **all seven -dev packages are present**. The ROS dependencies are
satisfied by the installed `ros-jazzy-slam-toolbox 2.8.5-1noble` chain.

**Build time on this machine: 8–15 minutes, expect ~10.** 4 cores (`nproc`), 23 GB
RAM, 25 GB free on `/`. 20 translation units outside the tests, of which
`lib/karto_sdk/src/Mapper.cpp` and `Karto.cpp` are the heavy ones, plus a Qt rviz
plugin (`SlamToolboxPlugin`) and 6 node executables with 6 libraries — a dozen
targets over 4 jobs. `ccache` is on PATH; priming it costs nothing the first time but
makes a second build after a one-line `Karto.h` edit much cheaper. Note the patch
touches a HEADER, so every target that includes `Karto.h` recompiles on each change;
budget a near-full rebuild per iteration.

**How `run_offline_maps.sh` would select it, default unset and byte-identical.** The
script already resolves slam_toolbox through the environment and nothing else —
`ros2 pkg prefix slam_toolbox` at `:413-414` and `ros2 launch "$LAUNCH"` at `:562`,
with `LAUNCH` fixed at `:30` — so the overlay needs no change to how the node is
started, only an opt-in prepend to `AMENT_PREFIX_PATH`. The hook, to go beside the
existing preflight at `:410-414`:

```sh
# Opt-in overlay for a patched slam_toolbox. UNSET BY DEFAULT: with it unset this
# block does nothing and the run is byte-identical to every run before it. Set it
# only for an offline replay; the online simulation never reads it, because
# swarm_sim.launch.py does not look at it and nothing here exports it.
if [[ -n "${SLAM_TOOLBOX_OVERLAY:-}" ]]; then
  [[ -f "$SLAM_TOOLBOX_OVERLAY/setup.bash" ]] || \
    die "SLAM_TOOLBOX_OVERLAY=$SLAM_TOOLBOX_OVERLAY has no setup.bash"
  source "$SLAM_TOOLBOX_OVERLAY/setup.bash"
fi
# Provenance, printed WHETHER OR NOT the overlay is set, so every run log records
# which slam_toolbox built the map rather than leaving it to be inferred.
echo "slam_toolbox prefix: $(ros2 pkg prefix slam_toolbox)"
```

Used as `SLAM_TOOLBOX_OVERLAY=~/src/slam_toolbox_ws/install bash
experiments/slam/run_offline_maps.sh ...`. Three properties, each deliberate:

* **default unset is byte-identical.** The block is skipped entirely; the only added
  output is the prefix line, which is new information in the log and not a change to
  the run.
* **the online simulation never sees it.** The variable is read only here, never
  exported by this script, and `swarm_sim.launch.py` does not mention it — so a
  patched Karto cannot leak into a live 5-robot run, where it would change the maps
  the explorer plans against.
* **every run says which binary built its map.** Without the prefix line, two map
  sets built weeks apart would be indistinguishable in the logs, which is exactly the
  confusion the `--spawn-rev` gate exists to prevent for the spawn table.

Before any patched map is compared with an unpatched one: the overlay must first
reproduce an UNPATCHED map, by building the copy **without** the patch and checking a
replay against an existing map — otherwise a difference cannot be attributed to the
patch rather than to the overlay's compiler flags.
