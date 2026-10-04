"""Layer 1 -- the wall predicate in experiments/analysis/graph_walls.py.

A1-A7 of docs/specs/SPEC_b2_wall_predicate.md §6, on synthetic grids only: no
corpus, no bag, no world SDF, no git. A8-A11 are the corpus run's job and are
deliberately NOT here -- they are world-truth validation, they are reported
rather than gated, and nothing in CI has the maps.

EVERY CHECK IS SHOWN FAILING WITH ITS NAMED BREAK. That is the whole shape of
this file: for each check there is a second test that runs the SAME extractor
with one parameter broken and asserts the check goes red. A check nobody has
seen fail is a check that might be asserting nothing, and the breaks are named
in the spec rather than invented here:

    A1  eps = 0.4*rho                 A5  swap endpoint precedence 1 and 2
    A2  theta restricted to 90°        A6  drop W4
    A3  g doubled                      A7  seed-dependent tie-break
    A4  l_min = 0.1

The breaks are values in graph_walls.Params, so no test edits code to break it.

WHAT THE FIXTURES ARE. `paint_face` marches along a straight surface and paints
`thickness` cells of OCCUPIED behind it and `free_depth` cells of FREE in front,
free first so occupied wins any overlap. The background stays UNKNOWN, which is
what makes each painted wall expose exactly ONE face: the far side of the slab
borders unknown, and an occupied-unknown boundary gives no element (Def 1). A
wall free on BOTH sides is then a deliberate construction, and A6 is the test of
it.

The module is loaded by location, the way test_robot_divergence.py:47 loads its
script: experiments/ is not an importable package. Importing it costs numpy and
PyYAML only -- no scipy, no PIL, no ROS -- so this file must keep collecting in
CI's ros:jazzy container, where a module-level import of any of those is a
COLLECTION error that takes the whole nsk_swarm suite down.
"""

import importlib.util
import json
import math
import os

import numpy as np
import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'graph_walls.py')


def load_module():
    spec = importlib.util.spec_from_file_location('graph_walls', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gw = load_module()

OCC, FREE, UNK = gw.OCC, gw.FREE, gw.UNKNOWN
RHO = 0.1
ROBOT_RADIUS = 0.11         # §1, nav2_robot*.yaml; g = 2*robot_radius = 0.22


# ─────────────────────────────── fixtures ────────────────────────────────────

def blank(h, w, state=UNK):
    return np.full((h, w), state, dtype=np.uint8)


def cell_of(x, y, rho, origin):
    """The cell containing (x, y), with trinary_map.sample's own float slack.

    The 1e-9 is not decoration. 8.2 / 0.1 is 81.99999999999999 in IEEE 754, so a
    bare floor puts x = 8.2 in cell 81 and a three-cell hole comes out four cells
    wide -- which is exactly how the first draft of A3 built a 3-cell hole where
    it meant a 2-cell one. trinary_map.sample:247 carries the same EPS for the
    same reason.
    """
    return (int(math.floor((y - origin[1]) / rho + 1e-9)),
            int(math.floor((x - origin[0]) / rho + 1e-9)))


def paint_face(grid, rho, origin, p0, p1, normal, thickness=3, free_depth=5,
               jitter=None):
    """One straight exposed face: occupied behind the surface, free in front.

    `jitter` maps a cell index along the face to an offset in whole cells along
    the normal -- A1's "10 % of cells jittered one cell". Painting free first and
    occupied second means a neighbour's slab wins over a jittered sample's free
    cell, which is exactly how a one-cell jitter looks in a real map: the face is
    the outermost exposed boundary, with a notch where a neighbour sticks out.
    """
    jitter = jitter or {}
    (x0, y0), (x1, y1) = p0, p1
    length = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    nn = math.hypot(*normal)
    nx, ny = normal[0] / nn, normal[1] / nn

    free_pts, occ_pts = [], []
    for i in range(int(round(length / (rho / 3.0))) + 1):
        s = min(i * rho / 3.0, length)
        off = rho * jitter.get(int(s / rho), 0)
        bx, by = x0 + s * ux + off * nx, y0 + s * uy + off * ny
        for k in range(free_depth):
            free_pts.append((bx + (k + 0.5) * rho * nx,
                             by + (k + 0.5) * rho * ny))
        for k in range(thickness):
            occ_pts.append((bx - (k + 0.5) * rho * nx,
                            by - (k + 0.5) * rho * ny))

    h, w = grid.shape
    for pts, state in ((free_pts, FREE), (occ_pts, OCC)):
        for x, y in pts:
            r, c = cell_of(x, y, rho, origin)
            if 0 <= r < h and 0 <= c < w:
                grid[r, c] = state
    return grid


def paint_block(grid, rho, origin, x0, y0, x1, y1, state):
    """A rectangle of one state, in metres. Free space round a wall end."""
    r0, c0 = cell_of(x0, y0, rho, origin)
    r1, c1 = cell_of(x1, y1, rho, origin)
    h, w = grid.shape
    grid[max(0, r0):min(h, r1 + 1), max(0, c0):min(w, c1 + 1)] = state
    return grid


def jitter_every(n_cells, every=10):
    """A deterministic one-in-`every` jitter, plus the two cells at each end.

    No RNG anywhere: A7 asserts that two runs over one map agree byte for byte,
    and a fixture that rolled dice would make that assertion about the fixture.
    The END cells are jittered on purpose -- they are what makes A1's
    "endpoints within eps + rho" bite, since a too-small eps drops them and pulls
    the endpoint in by two whole cells.
    """
    idx = {0: +1, 1: -1, n_cells - 2: +1, n_cells - 1: -1}
    for i in range(every, n_cells - 2, every):
        idx[i] = +1 if (i // every) % 2 else -1
    return idx


FIVE_FACES = (
    # (name, p0, p1, outward normal). Four axis aligned and one at 17°.
    #
    # The 17° face sits in the empty upper right, and WHERE it sits is not
    # arbitrary. Placed at (10.0, 6.0) its line, extended backwards, crossed
    # y = 3.2 at x = 1.98 -- the left end of the `south` face. Eight of that
    # face's elements are then collinear with all 63 of the diagonal's, so a peak
    # at 19.5° collects 75 votes, beats the true 71-vote faces, and takes the
    # first eight cells of `south` away as a 0.8 m segment of its own. The
    # extractor is behaving exactly as §4 specifies -- greedy, and "maximal with
    # respect to the extraction order only" -- on a fixture that accidentally put
    # two faces on one line. From (11.0, 12.5) the same line leaves the grid
    # before reaching any other face.
    ('south', (2.0, 3.2), (9.0, 3.2), (0.0, 1.0)),
    ('north', (2.0, 16.0), (9.0, 16.0), (0.0, -1.0)),
    ('west', (2.2, 6.0), (2.2, 11.0), (1.0, 0.0)),
    ('east', (16.0, 6.0), (16.0, 11.0), (-1.0, 0.0)),
    ('diag17', (11.0, 12.5),
     (11.0 + 5.0 * math.cos(math.radians(17.0)),
      12.5 + 5.0 * math.sin(math.radians(17.0))),
     (-math.sin(math.radians(17.0)), math.cos(math.radians(17.0)))),
)


def five_face_grid(rho=RHO, jitter=True):
    """A1's grid: four axis-aligned faces and one at 17°, each exposed on one
    side only, 10 % of cells jittered one cell."""
    origin = (0.0, 0.0)
    grid = blank(int(round(20.0 / rho)), int(round(20.0 / rho)))
    for _name, p0, p1, n in FIVE_FACES:
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n_cells = int(round(length / rho))
        paint_face(grid, rho, origin, p0, p1, n,
                   jitter=jitter_every(n_cells) if jitter else None)
    return grid, rho, origin


def expected_theta(normal):
    """The oriented theta whose normal is `normal` (§2: n = (-sin, cos))."""
    return (math.degrees(math.atan2(normal[1], normal[0])) - 90.0) % 360.0


def rotate_nn(grid, deg):
    """The same grid rotated about its centre, nearest neighbour, unknown outside.

    A2's only job is arbitrary angles, and nearest neighbour is what the spec
    asks for: it is the crude resampling a rotated map really suffers, jagged
    surface and all.
    """
    h, w = grid.shape
    cy, cx = (h - 1) / 2.0, (w - 1) / 2.0
    rr, cc = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    rad = math.radians(deg)
    # Inverse rotation: where does this output cell come from. The content turns
    # by +deg, the same direction rotate_point() turns a point -- getting these
    # two out of step makes every A2 expectation 60° wrong and nothing else.
    sy, sx = rr - cy, cc - cx
    src_r = np.rint(cy + math.cos(rad) * sy - math.sin(rad) * sx).astype(int)
    src_c = np.rint(cx + math.sin(rad) * sy + math.cos(rad) * sx).astype(int)
    ok = (src_r >= 0) & (src_r < h) & (src_c >= 0) & (src_c < w)
    out = blank(h, w)
    out[ok] = grid[src_r[ok], src_c[ok]]
    return out


def rotate_point(p, deg, grid, rho, origin):
    h, w = grid.shape
    cx = origin[0] + ((w - 1) / 2.0 + 0.5) * rho
    cy = origin[1] + ((h - 1) / 2.0 + 0.5) * rho
    rad = math.radians(deg)
    dx, dy = p[0] - cx, p[1] - cy
    return (cx + math.cos(rad) * dx - math.sin(rad) * dy,
            cy + math.sin(rad) * dx + math.cos(rad) * dy)


def disc_grid(rho, radius=ROBOT_RADIUS, pad_cells=6):
    """A4's grid: occupied rim, unknown inside, free outside. A parked robot."""
    half = radius + pad_cells * rho
    n = int(math.ceil(2 * half / rho))
    origin = (-half, -half)
    xs = origin[0] + (np.arange(n) + 0.5) * rho
    ys = origin[1] + (np.arange(n) + 0.5) * rho
    gx, gy = np.meshgrid(xs, ys)
    r = np.hypot(gx, gy)
    grid = blank(n, n)
    grid[r > radius + rho / 2.0] = FREE
    grid[np.abs(r - radius) <= rho / 2.0] = OCC
    return grid, rho, origin


def write_map(tmp_path, stem, grid, rho, origin):
    """A scratch PGM + YAML, so A7 exercises the real per-map path.

    §9: tests run against scratch copies, never real run artefacts. The bytes and
    the thresholds are the ones all three writers in this project emit -- 0 / 254
    / 205 with occupied_thresh 0.65 and free_thresh 0.25, the pair that does NOT
    recover 205 and is exactly why trinary_map overrides it.
    """
    pgm = tmp_path / f'{stem}.pgm'
    h, w = grid.shape
    with open(pgm, 'wb') as fh:
        fh.write(f'P5\n{w} {h}\n255\n'.encode())
        fh.write(grid[::-1, :].tobytes())      # convention A: row 0 = max y
    (tmp_path / f'{stem}.yaml').write_text(
        f'image: {stem}.pgm\nmode: trinary\nresolution: {rho!r}\n'
        f'origin: [{origin[0]!r}, {origin[1]!r}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return pgm


def params(**kw):
    """Default Params with g = 2*robot_radius, as the corpus run reads it."""
    kw.setdefault('g', 2.0 * ROBOT_RADIUS)
    return gw.Params(**kw)


def pair_endpoints(got, want):
    """Match two extracted endpoints to two expected ones, nearest first.

    Not by sorting the coordinates: a segment fitted a hundredth of a degree off
    vertical has two endpoints whose x differs in the third decimal, and sorting
    tuples then pairs the top of one with the bottom of the other.
    """
    straight = (math.dist(got[0], want[0]) + math.dist(got[1], want[1]))
    crossed = (math.dist(got[0], want[1]) + math.dist(got[1], want[0]))
    return list(zip(got, want)) if straight <= crossed \
        else list(zip(got, want[::-1]))


def find_segment(graph, theta, normal, tol_deg=1.0):
    """The segment whose angle is within `tol_deg` of `theta` and which faces
    the same way. Returns None when there is none."""
    best = None
    for s in graph['segments']:
        if abs(((s['theta'] - theta) + 180.0) % 360.0 - 180.0) > tol_deg:
            continue
        if s['normal'][0] * normal[0] + s['normal'][1] * normal[1] <= 0:
            continue
        if best is None or s['length'] > best['length']:
            best = s
    return best


# ───────────────────────────────── A1 ────────────────────────────────────────

def test_a1_five_faces_with_one_cell_jitter():
    """Four axis-aligned faces and one at 17°: all five found, endpoints within
    eps + rho of the truth, angle within 1°."""
    grid, rho, origin = five_face_grid()
    p = params()
    graph = gw.extract(grid, rho, origin, p)
    tol = p.eps(rho) + rho

    for name, p0, p1, n in FIVE_FACES:
        theta = expected_theta(n)
        s = find_segment(graph, theta, n)
        assert s is not None, f'{name}: no segment within 1° of {theta:.2f}°'
        for g_pt, w_pt in pair_endpoints([tuple(s['p0']), tuple(s['p1'])],
                                         [p0, p1]):
            assert math.dist(g_pt, w_pt) <= tol, \
                f'{name}: endpoint {g_pt} is more than {tol:.4f} m from {w_pt}'
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        assert abs(s['length'] - length) <= tol, \
            f'{name}: length {s["length"]:.3f} != {length:.3f} within {tol:.3f}'


def test_a1_break_eps_too_small():
    """eps = 0.4*rho. A jitter of one whole cell is then outside the band, so the
    jittered end cells drop out and the endpoints pull in by two cells -- further
    than the (now tighter) eps + rho tolerance."""
    grid, rho, origin = five_face_grid()
    p = params(eps_factor=0.4)
    graph = gw.extract(grid, rho, origin, p)
    tol = p.eps(rho) + rho

    failures = []
    for name, p0, p1, n in FIVE_FACES:
        s = find_segment(graph, expected_theta(n), n)
        if s is None:
            failures.append(f'{name}: not found')
            continue
        for g_pt, w_pt in pair_endpoints([tuple(s['p0']), tuple(s['p1'])],
                                         [p0, p1]):
            if math.dist(g_pt, w_pt) > tol:
                failures.append(f'{name}: endpoint off by more than {tol:.3f} m')
    assert failures, 'eps = 0.4*rho must break A1, and it did not'


# ───────────────────────────────── A2 ────────────────────────────────────────

def test_a2_grid_rotated_30_degrees():
    """The same grid rotated 30°: every angle shifts 30° ± 1°, lengths hold
    within 2*rho, same segment count. The only test of arbitrary angles, since
    every wall in the corpus world is axis aligned.

    Rotated WITHOUT A1's jitter, and that is a decision worth stating. Nearest
    neighbour resampling of a one-cell-jittered tip erodes it: rotating the
    jittered grid left the `south` face 0.38 m short at one end, four cells lost
    as a sub-l_min run, which is the fixture's two artefacts compounding rather
    than anything about the extractor. A2 is the arbitrary-angle check; A1 is the
    jitter check.
    """
    grid, rho, origin = five_face_grid(jitter=False)
    upright = gw.extract(grid, rho, origin, params())
    rotated = gw.extract(rotate_nn(grid, 30.0), rho, origin, params())

    assert len(rotated['segments']) == len(upright['segments']), (
        f'{len(upright["segments"])} segments upright, '
        f'{len(rotated["segments"])} rotated')

    for name, p0, p1, n in FIVE_FACES:
        rad = math.radians(30.0)
        rn = (n[0] * math.cos(rad) - n[1] * math.sin(rad),
              n[0] * math.sin(rad) + n[1] * math.cos(rad))
        want_theta = (expected_theta(n) + 30.0) % 360.0
        s = find_segment(rotated, want_theta, rn)
        assert s is not None, \
            f'{name}: no segment within 1° of {want_theta:.2f}° after rotating'
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        assert abs(s['length'] - length) <= 2 * rho, \
            f'{name}: length {s["length"]:.3f} != {length:.3f} within {2 * rho}'


def test_a2_break_theta_restricted_to_90():
    """theta on multiples of 90° only, and A2's assertions go red.

    Note what does NOT save it: a peak at 90° still gathers whatever lies within
    eps of that line, and the TLS refit is free to turn the result to the true
    angle. So the faces are still found NEAR the right angle -- in pieces. A
    30° face crosses the 0.34 m of d that one peak can reach in 0.68 m of face,
    so the count explodes and no piece is anywhere near its true length. The
    check A2 states is count AND length, and that is what has to fail.
    """
    grid, rho, origin = five_face_grid(jitter=False)
    rotated = rotate_nn(grid, 30.0)
    broken = gw.extract(rotated, rho, origin, params(dtheta_deg=90.0))
    upright = gw.extract(grid, rho, origin, params())

    failures = []
    if len(broken['segments']) != len(upright['segments']):
        failures.append(f'{len(broken["segments"])} segments, not '
                        f'{len(upright["segments"])}')
    for name, p0, p1, n in FIVE_FACES:
        rad = math.radians(30.0)
        rn = (n[0] * math.cos(rad) - n[1] * math.sin(rad),
              n[0] * math.sin(rad) + n[1] * math.cos(rad))
        s = find_segment(broken, (expected_theta(n) + 30.0) % 360.0, rn)
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if s is None:
            failures.append(f'{name}: not found')
        elif abs(s['length'] - length) > 2 * rho:
            failures.append(f'{name}: {s["length"]:.2f} m, not {length:.2f} m')
    assert failures, 'theta on multiples of 90° must break A2, and it did not'


# ───────────────────────────────── A3 ────────────────────────────────────────
# The spec asks for physical gaps of 0.5g and 1.5g. Neither is realizable: a
# physical gap on a rho = 0.1 m lattice is a whole number of missing cells, and
# g = 0.22 m, so the realizable neighbours are 1 cell (0.10 m = 0.45 g), 2 cells
# (0.20 m = 0.91 g, the tightest gap that still bridges) and 3 cells (0.30 m =
# 1.36 g, which splits). The pair still straddles g, and g doubled to 0.44 m
# still turns the 3-cell split into a bridge. See §11 P3.

def gap_grid():
    """One 8 m face with four holes: 1 cell free, 1 cell unknown, 3 cells free
    (which must split it), and 2 cells free in the far run."""
    rho, origin = RHO, (0.0, 0.0)
    grid = blank(60, 110)
    paint_face(grid, rho, origin, (1.0, 3.0), (9.0, 3.0), (0.0, 1.0))
    # A hole has to go through the WHOLE slab: leaving a lower occupied row
    # would expose a second face one cell down, inside eps of the first, and
    # there would be no gap at all.
    for x0, x1, state in ((4.0, 4.1, FREE),      # 1 cell, free      -> bridged
                          (6.0, 6.1, UNK),       # 1 cell, unknown   -> bridged
                          (7.0, 7.3, FREE),      # 3 cells, free     -> splits
                          (8.2, 8.4, FREE)):     # 2 cells, free     -> bridged
        paint_block(grid, rho, origin, x0, 2.6, x1 - 1e-9, 3.0 - 1e-9, state)
        if state is UNK:
            paint_block(grid, rho, origin, x0, 3.0, x1 - 1e-9, 3.1 - 1e-9, UNK)
    return grid, rho, origin


def test_a3_gaps_bridged_and_splitting():
    """Holes of 1, 1 and 2 cells bridge; the 3-cell hole splits.

    The widths are asserted as bands, `[MIN_GAP, g)` for a bridge and `>= g` for
    a split, rather than as exact numbers. A hole in the WALL and the gap in the
    FACE are not the same length: the riser on the near side of a hole satisfies
    W4 (`dot > 0`) against the line's hundredth-of-a-degree tilt and joins the
    segment, so the face gap is up to half a cell shorter than the hole. Which
    side contributes the riser depends on the sign of that tilt --
    test_a3_risers_at_a_hole_edge_join_the_face pins the behaviour, and the
    opening spec will have to decide which of the two lengths it wants.
    """
    grid, rho, origin = gap_grid()
    p = params()
    graph = gw.extract(grid, rho, origin, p)

    assert len(graph['segments']) == 2, (
        'the 3-cell gap (0.30 m, 1.36 g) must split the face in two, giving '
        f'2 segments, not {len(graph["segments"])}')
    near, far = sorted(graph['segments'], key=lambda s: s['length'],
                       reverse=True)

    floor_m = gw.MIN_GAP_FACTOR * rho
    for s, n_gaps, why in ((near, 2, 'the 1-cell free and 1-cell unknown holes'),
                           (far, 1, 'the 2-cell hole, 0.91 g')):
        assert len(s['bridged_gaps']) == n_gaps, (
            f'{why} must bridge: {[q["width"] for q in s["bridged_gaps"]]}')
        for q in s['bridged_gaps']:
            assert floor_m <= q['width'] < p.g, q

    assert sorted(q['state'] for q in near['bridged_gaps']) == \
        ['free', 'unknown'], (
        'a 1-cell hole whose band is free must read free and one whose band is '
        f'unknown must read unknown, got '
        f'{[q["state"] for q in near["bridged_gaps"]]}')

    lines = [ln for ln in graph['lines'] if ln['gaps']]
    assert len(lines) == 1, f'{len(lines)} lines carry a splitting gap'
    split = lines[0]['gaps']
    assert len(split) == 1 and split[0]['width'] >= p.g, split
    assert split[0]['state'] == 'free', split[0]
    assert sorted(x for x in split[0]['between'] if x is not None) == [0, 1], (
        'a splitting gap records which segments it separates: '
        f'{split[0]["between"]}')


def test_a3_risers_at_a_hole_edge_join_the_face():
    """W4 is `dot > 0`, and that has a consequence worth pinning.

    A hole in a wall exposes the wall's SIDE inside the hole. Those riser
    elements are perpendicular to the face, so their dot with an exactly
    axis-aligned line is exactly 0 and FACING_MIN excludes them -- but a line
    fitted a hundredth of a degree off the axis gives the risers on one side
    dot = +0.002, and W4 admits them. They then extend the run half a cell into
    the hole.

    This is the spec's W4, not a departure from it: the same looseness is what
    lets a 17° face's staircase risers belong to the face they are part of (A1).
    It is pinned here so that a later change to W4 shows up as this test
    changing, rather than as gap widths quietly moving by half a cell.
    """
    grid, rho, origin = gap_grid()
    graph = gw.extract(grid, rho, origin, params())
    el = gw.face_elements(grid, rho, origin)
    long_seg = max(graph['segments'], key=lambda s: s['length'])
    ux, uy = gw.dir_of(long_seg['theta'])
    t0 = min(long_seg['p0'][0] * ux + long_seg['p0'][1] * uy,
             long_seg['p1'][0] * ux + long_seg['p1'][1] * uy)
    t1 = max(long_seg['p0'][0] * ux + long_seg['p0'][1] * uy,
             long_seg['p1'][0] * ux + long_seg['p1'][1] * uy)
    t = el['x'] * ux + el['y'] * uy
    inside = (t >= t0) & (t <= t1)
    perpendicular = np.abs(el['nx']) > 0.5          # a +/-x normal
    assert np.count_nonzero(inside & perpendicular) > 0, (
        'the hole edges must expose risers inside the long run; if this fails '
        'the fixture no longer has free-sided holes')
    assert 0.0 < long_seg['theta'] % 90.0 < 1.0 \
        or 89.0 < long_seg['theta'] % 90.0 < 90.0, (
        'the risers tilt the fit off the axis by a fraction of a degree, which '
        f'is what admits them: theta = {long_seg["theta"]}')


def test_a3_break_g_doubled():
    """g = 0.44 m bridges the 0.30 m gap, so the face is not split and the
    splitting gap the opening spec would read is gone."""
    grid, rho, origin = gap_grid()
    graph = gw.extract(grid, rho, origin, params(g=4.0 * ROBOT_RADIUS))
    assert len(graph['segments']) == 1, (
        'g doubled must bridge the 0.30 m gap and leave one segment, '
        f'got {len(graph["segments"])}')
    assert not any(ln['gaps'] for ln in graph['lines']), \
        'with g doubled there is no splitting gap left to record'


# ───────────────────────────────── A4 ────────────────────────────────────────

@pytest.mark.parametrize('rho', [0.02, 0.1])
def test_a4_disc_is_never_a_wall(rho):
    """A parked robot: occupied rim, unknown inside, free outside. No segment at
    either resolution, and exactly one unclassified component (Def 6)."""
    grid, rho, origin = disc_grid(rho)
    graph = gw.extract(grid, rho, origin, params())
    assert graph['segments'] == [], (
        f'a disc of radius {ROBOT_RADIUS} m is not a wall, got '
        f'{len(graph["segments"])} segments at rho = {rho}')
    unc = graph['unclassified']
    assert unc['n_cells'] > 0 and len(unc['components']) == 1, (
        f'the rim must be one unclassified component, got '
        f'{len(unc["components"])} of {unc["n_cells"]} cells')


def test_a4_break_l_min_0p1():
    """l_min = 0.1 m is one cell, so any two rim cells in a row become a wall."""
    grid, rho, origin = disc_grid(0.02)
    graph = gw.extract(grid, rho, origin, params(l_min=0.1))
    assert graph['segments'], 'l_min = 0.1 must turn the rim into segments'


# ───────────────────────────────── A5 ────────────────────────────────────────

def end_state_grid():
    """Three faces: one ending in free space, one ending at unknown, one running
    into a perpendicular face with unknown behind it.

    The unknown BEHIND the perpendicular face is the point: without it, swapping
    Def 5's precedence 1 and 2 would change nothing and the named break would
    not be a break. A real corner has exactly this -- the robot sees the corner
    and not what is behind it.
    """
    rho, origin = RHO, (0.0, 0.0)
    grid = blank(120, 200)

    # 1. ends free: free space wraps round the end, on both sides of the line.
    paint_face(grid, rho, origin, (1.0, 2.0), (3.0, 2.0), (0.0, 1.0))
    paint_block(grid, rho, origin, 3.0, 1.5, 3.5, 2.5, FREE)

    # 2. ends unknown: nothing painted beyond the end, so the band reads U.
    paint_face(grid, rho, origin, (6.0, 2.0), (8.0, 2.0), (0.0, 1.0))

    # 3. ends occupied: a perpendicular face across the end, unknown behind it.
    #    TWO cells thick, as this world's maze walls are, and the thickness is
    #    load-bearing at both ends of the range. Three cells fill the whole
    #    0.22 m band and the unknown behind is out of reach, so precedence has
    #    nothing to choose between and the named break cannot fire. One cell is
    #    not enough either: the corner's own riser satisfies W4 against the
    #    through face's slightly tilted line and gets absorbed into it, pushing
    #    that face's endpoint half a cell PAST the corner, and its cell is then
    #    the segment's own and excluded from precedence 1 -- the corner reads
    #    `unknown`. At two cells the second column is occupied, is not the
    #    segment's own, and the unknown behind it is still inside the band.
    paint_face(grid, rho, origin, (11.0, 2.0), (13.0, 2.0), (0.0, 1.0))
    paint_face(grid, rho, origin, (13.0, 2.0), (13.0, 4.0), (-1.0, 0.0),
               thickness=2)
    return grid, rho, origin


def _ends(graph, rho):
    """The end_state of each of A5's three faces, keyed by which one it is.

    `normal[1] > 0.9` rather than `== 1`: the face that runs into a perpendicular
    one picks up that corner's riser (W4 is `dot > 0`), which tilts its fit by
    about a degree. Demanding an exact normal here would silently drop the very
    segment this check is about.
    """
    out = {}
    for s in graph['segments']:
        if s['normal'][1] < 0.9:
            continue
        x0, x1 = sorted([s['p0'][0], s['p1'][0]])
        for name, want in (('free', 1.0), ('unknown', 6.0), ('occupied', 11.0)):
            if abs(x0 - want) < 0.5:
                # end_state is [p0 end, p1 end] and p0 has the smaller t; for
                # theta 0 that is the smaller x, so the far end is index 1.
                far = s['end_state'][1] if s['p1'][0] > s['p0'][0] \
                    else s['end_state'][0]
                out[name] = far
    return out


def test_a5_endpoint_states():
    grid, rho, origin = end_state_grid()
    ends = _ends(gw.extract(grid, rho, origin, params()), rho)
    assert ends == {'free': 'free', 'unknown': 'unknown',
                    'occupied': 'occupied'}, ends


def test_a5_break_precedence_swapped():
    """Unknown ahead of occupied: the corner then reports the unknown BEHIND the
    perpendicular face instead of the face itself, and stops being a corner
    candidate."""
    grid, rho, origin = end_state_grid()
    ends = _ends(gw.extract(grid, rho, origin,
                            params(endpoint_precedence=('unknown',
                                                        'occupied'))), rho)
    assert ends.get('occupied') == 'unknown', (
        'swapping precedence 1 and 2 must relabel the corner, got '
        f'{ends.get("occupied")!r}')


# ───────────────────────────────── A6 ────────────────────────────────────────

def two_sided_grid(thickness_cells):
    """A wall free on BOTH sides: two faces, two segments, opposite normals."""
    rho, origin = RHO, (0.0, 0.0)
    grid = blank(80, 120)
    y = 4.0
    paint_face(grid, rho, origin, (2.0, y), (8.0, y), (0.0, 1.0),
               thickness=thickness_cells, free_depth=5)
    paint_face(grid, rho, origin, (2.0, y - thickness_cells * rho),
               (8.0, y - thickness_cells * rho), (0.0, -1.0),
               thickness=thickness_cells, free_depth=5)
    return grid, rho, origin


@pytest.mark.parametrize('cells,apart', [(2, 0.2), (1, 0.1)])
def test_a6_wall_free_on_both_sides(cells, apart):
    """Two segments with opposite normals, their lines `apart` metres apart.

    C2's consequence: faces sit on the true surface, so a 0.2 m wall's two faces
    are 0.20 m apart, not 0.10. With one cell of wall the SAME cell gives one
    element to each face, which is why W5 forbids sharing an element and not
    sharing a cell.
    """
    grid, rho, origin = two_sided_grid(cells)
    graph = gw.extract(grid, rho, origin, params())
    assert len(graph['segments']) == 2, (
        f'a {cells}-cell wall free on both sides is two faces, got '
        f'{len(graph["segments"])}')
    up = find_segment(graph, 0.0, (0.0, 1.0))
    down = find_segment(graph, 180.0, (0.0, -1.0))
    assert up is not None and down is not None, [
        (s['theta'], s['normal']) for s in graph['segments']]
    assert abs(up['normal'][1] + down['normal'][1]) < 1e-6, 'normals oppose'
    assert abs(abs(up['p0'][1] - down['p0'][1]) - apart) < 1e-6, (
        f'the two lines must be {apart} m apart, got '
        f'{abs(up["p0"][1] - down["p0"][1]):.4f}')
    if cells == 1:
        assert up['n_elements'] == down['n_elements'], (
            'one cell of wall gives one element to each side: '
            f'{up["n_elements"]} vs {down["n_elements"]}')


def test_a6_break_w4_dropped():
    """Without W4 the one-cell wall's two faces, 0.10 m apart and inside
    eps = 0.17 m, collapse into a single segment facing neither way."""
    grid, rho, origin = two_sided_grid(1)
    graph = gw.extract(grid, rho, origin, params(enforce_facing=False))
    assert len(graph['segments']) == 1, (
        'dropping W4 must merge the two faces of a one-cell wall, got '
        f'{len(graph["segments"])} segments')


# ───────────────────────────────── A7 ────────────────────────────────────────

def test_a7_same_map_twice_is_byte_identical(tmp_path):
    """The whole per-map path, twice, on a map with tied peaks.

    Two of the four axis-aligned faces are 7.0 m and two are 5.0 m, so the
    accumulator has ties and the tie-break is what decides the order -- which is
    what makes this a test of determinism rather than of arithmetic.
    """
    grid, rho, origin = five_face_grid(jitter=False)
    write_map(tmp_path, 'scratch', grid, rho, origin)
    one = gw.graph_for_map('scratch', tmp_path, params(), 'test')
    two = gw.graph_for_map('scratch', tmp_path, params(), 'test')
    assert json.dumps(one, indent=2) == json.dumps(two, indent=2)


def test_a7_break_seeded_tie_break(tmp_path):
    """A seed-dependent tie-break, and two runs of one map stop agreeing."""
    grid, rho, origin = five_face_grid(jitter=False)
    write_map(tmp_path, 'scratch', grid, rho, origin)
    one = gw.graph_for_map('scratch', tmp_path, params(tie_break_seed=1), 'x')
    two = gw.graph_for_map('scratch', tmp_path, params(tie_break_seed=2), 'x')
    assert json.dumps(one, indent=2) != json.dumps(two, indent=2), \
        'a seeded tie-break must make two runs of one map disagree'


# ─────────────────── the pieces the checks above rest on ─────────────────────

def test_face_elements_ignore_the_unknown_side():
    """Def 1: an occupied-unknown boundary gives no element.

    The single most consequential line in the extractor. Taking unknown for free
    is what once reported all 20 offline grids as fully known.
    """
    # Written the way it looks on screen, top row first, then flipped: the grid
    # this file works in is bottom-up, +y with the row index.
    grid = np.array([[UNK, UNK, UNK],
                     [UNK, OCC, FREE],
                     [FREE, FREE, FREE]], dtype=np.uint8)[::-1].copy()
    el = gw.face_elements(grid, 1.0, (0.0, 0.0))
    got = sorted(zip(el['nx'].tolist(), el['ny'].tolist()))
    assert got == [(0.0, -1.0), (1.0, 0.0)], got


def test_band_reads_outside_the_map_as_unknown():
    """A band cell off the grid is UNKNOWN, as trinary_map.sample fills it.

    Without this a wall running to the edge of the PGM would have an empty band
    beyond its endpoint and Def 5 would fall through to `free` -- reporting that
    the wall visibly ends at the one place the map cannot say anything.
    """
    grid = np.full((3, 3), FREE, dtype=np.uint8)
    bd = gw.band(grid, 1.0, (0.0, 0.0), 0.0, 1.0, 5.0, 7.0, 0.707)
    assert bd['states'].size and np.all(bd['states'] == UNK)
    assert not bd['inside'].any()


def test_w2_measures_the_physical_gap():
    """C4: the gap is t[k+1] - t[k] - rho, not t[k+1] - t[k].

    Under the difference alone a gap of one whole cell more than g would still
    bridge, which is what v1 of the spec did.
    """
    t = np.array([0.0, 0.1, 0.4])       # one 0.2 m physical gap at the end
    assert gw.split_runs(t, 0.1, 0.22) == [(0, 3)]
    assert gw.split_runs(t, 0.1, 0.19) == [(0, 2), (2, 3)]


def test_a_gated_stem_parses_to_the_same_robot_and_cut():
    """The travel-gated maps carry `_gated`, and must parse to the same (K, cut).

    That is the whole point of widening the parser: a gated map and the ungated
    map it is compared against are two maps of ONE cut, so (K, cut) has to come
    out the same from both names. A stem that names neither still has to be
    refused, or a typo would be read as a corpus map.
    """
    plain = gw._k_and_cut('b2maps_k3_cut240_robot3', 'why')
    gated = gw._k_and_cut('b2maps_k3_cut240_gated_robot3', 'why')
    assert plain == (3, 240)
    assert gated == plain

    for bad in ('b2maps_k3_cut240_robot4',          # run and map disagree
                'b2maps_k3_cut240_gated_robot4',
                'b2maps_k3_cut240',                 # no robot
                'b2maps_k3_cutX_robot3',            # no cut
                'b2maps_k3_cut240_other_robot3'):   # an unknown variant
        with pytest.raises(SystemExit):
            gw._k_and_cut(bad, 'why')


def test_part_b_reads_the_run_variants_config_and_the_bag_variants_bag(
        tmp_path, monkeypatch):
    """Part B's four per-cut inputs, each under the rule that belongs to it.

    Three name the RUN -- the replay config, the relay log, the map->odom capture
    -- and the fourth is the stripped BAG, which carries the gate ONLY: an
    overlay build replays the recording of the run it is named after, so
    b2maps_k0_cut60_gated_extfix_robot0 must read ..._gated_slamin. Reaching for
    ..._gated_extfix_slamin would look for a bag nobody ever made, and this is
    the test that stops it: obs_for_map used to refuse every variant stem rather
    than get this right, and nothing failed when that refusal was removed.

    read_bag is stubbed, so no ROS and no corpus are needed.
    """
    stem = 'b2maps_k0_cut60_gated_extfix_robot0'
    assert gw.run_variant_of(stem) == '_gated_extfix'
    assert gw.bag_variant_of(stem) == '_gated'

    monkeypatch.setattr(gw, 'LOGS_DIR', tmp_path)
    (tmp_path / 'offline_mapping_b2maps_k0_cut60_gated_extfix_robot_0.yaml'
     ).write_text('    max_laser_range: 7.9\n')
    (tmp_path / 'offline_slam_b2maps_k0_cut60_gated_extfix_robot_0.log'
     ).write_text('[relay] relayed 290 scans\n')

    seen = {}

    def stub_read_bag(bag, k):
        seen['bag'] = bag
        raise SystemExit(99)        # stop before rosbag2_py is needed

    monkeypatch.setattr(gw, 'read_bag', stub_read_bag)
    with pytest.raises(SystemExit):
        gw.obs_for_map({'resolution': 0.1}, stem, 0, 60, np.zeros((3, 3), int),
                       params(), bags_dir=tmp_path)
    assert seen['bag'].name == 'b2maps_k0_cut60_gated_slamin'

    # And the run-variant readers refuse the plain names, which are not there.
    assert gw.read_replay_r_max(0, 60, '_gated_extfix')[0] == 7.9
    assert gw.read_relayed_scan_count(0, 60, '_gated_extfix')[0] == 290
    for call in (lambda: gw.read_replay_r_max(0, 60),
                 lambda: gw.read_relayed_scan_count(0, 60)):
        with pytest.raises(SystemExit):
            call()


def test_the_frame_is_read_from_a_gated_stem_too():
    """graph_walls takes the frame from `robot(\\d+)$`, which `_gated` leaves
    alone -- checked rather than assumed, since it is a second parser."""
    import re
    for stem, want in (('b2maps_k2_cut60_robot2', '2'),
                       ('b2maps_k2_cut60_gated_robot2', '2')):
        assert re.search(r'robot(\d+)$', stem).group(1) == want


def test_params_refuses_a_nonsense_configuration():
    for kw in ({'window_bins': 2}, {'dtheta_deg': 0.0}, {'l_min': -1.0},
               {'endpoint_precedence': ('free', 'unknown')}):
        with pytest.raises(ValueError):
            params(**kw)


# ══════════════════════ Part B -- viewing directions (§5) ════════════════════
# B1-B3 are corpus checks: they need a bag, and the script reports them per map
# with `--break convention-b` / `--break flip-normals` to show them failing. B4
# is synthetic and lives here, together with the three pieces of §5 that carry
# the geometry: the pose interpolation, the return construction and the
# attribution.

BASE_T_SCAN = (-0.032, 0.0, 0.0)     # the two-hop /tf_static chain, §1 and C7
SCAN_GEOM = {'frame': 'robot_0/base_scan', 'n_beams': 360,
             'angle_min': 0.0, 'angle_increment': 2.0 * math.pi / 360.0,
             'range_min': 0.16, 'range_max': 8.0}
R_MAX = 7.9                           # the replay config's max_laser_range


def synthetic_scan(face_y, poses, geom=None, range_max_fill=None):
    """Scans of a horizontal face at `face_y`, ranged from the SENSOR.

    Every beam that would hit the face gets its true sensor-to-face range; the
    rest are +inf, as a real scan has them. The ranges therefore already carry
    the 0.032 m sensor offset, which is what makes B4's break observable: omit
    base_T_scan when reconstructing and the hit points land off the face by
    exactly that much.
    """
    geom = geom or SCAN_GEOM
    beams = geom['angle_min'] + np.arange(geom['n_beams']) \
        * geom['angle_increment']
    ranges = np.full((len(poses), geom['n_beams']), np.inf)
    stamps = np.zeros(len(poses))
    for i, (t, x, y, yaw) in enumerate(poses):
        stamps[i] = t
        sy = y + math.sin(yaw) * BASE_T_SCAN[0] + math.cos(yaw) * BASE_T_SCAN[1]
        for j, b in enumerate(beams):
            ang = yaw + b
            sin_a = math.sin(ang)
            if abs(sin_a) < 1e-9:
                continue
            r = (face_y - sy) / sin_a
            if geom['range_min'] <= r < (range_max_fill or R_MAX):
                ranges[i, j] = r
    return {'geom': geom, 'stamps': stamps, 'ranges': ranges,
            'tf': None, 'tf_max_tilt': 0.0, 'static': {}}


# The poses B4 uses, and why the yaw is 92.5° rather than either 0 or 90.
#
# NOT 0: base_T_scan is (-0.032, 0) in the ROBOT's frame, so at yaw 0 it runs
# parallel to a horizontal face. Omitting it would slide every hit ALONG the face
# without moving it off, and the named break would be invisible. Near 90° the
# same offset is perpendicular to the face and dropping it lands every hit 3.2 cm
# past it.
#
# NOT 90 either: the beams are at whole degrees and the histogram bins are 5°
# wide, so at yaw 90 every bearing falls exactly ON a bin edge and which side of
# it a bearing lands on is decided by the last bit of a float. B4 asks for the
# histogram EXACTLY, so the geometry must not put the answer on a knife edge:
# 92.5 puts every bearing 2.5° from the nearest edge, and then the two ways of
# computing it -- `yaw + beam` and `atan2(p - o)` -- cannot disagree.
B4_FACE_Y = 3.0
B4_YAW = math.radians(92.5)
B4_POSES = [(10.0, 0.0, 1.0, B4_YAW), (10.2, 0.5, 1.0, B4_YAW)]


def pose_arrays(poses):
    return (np.array([p[1] for p in poses]), np.array([p[2] for p in poses]),
            np.array([p[3] for p in poses]),
            np.ones(len(poses), dtype=bool))


def test_b4_synthetic_scans_give_the_known_bearing_histogram():
    """B4. Two poses looking at a known face: the histogram is exactly known.

    Every ray leaves the sensor along `yaw + beam`, so each return's bearing in
    the map frame IS that angle -- no fitting, no tolerance, a number computable
    by hand. The histogram is those angles counted into 5° bins.
    """
    data = synthetic_scan(B4_FACE_Y, B4_POSES)
    returns = gw.scan_returns(data, pose_arrays(B4_POSES), BASE_T_SCAN,
                              (0.0, 0.0, 0.0), R_MAX)

    # every hit lands ON the face, to the last bit
    assert np.allclose(returns['py'], B4_FACE_Y, atol=1e-9), \
        f'worst |py - face_y| = {np.max(np.abs(returns["py"] - B4_FACE_Y))}'

    si, bi = np.nonzero(np.isfinite(data['ranges']))
    want_bearing = np.degrees(
        np.array([B4_POSES[i][3] for i in si])
        + data['geom']['angle_min']
        + bi * data['geom']['angle_increment']) % 360.0
    assert np.allclose(np.sort(returns['bearing']), np.sort(want_bearing),
                       atol=1e-9)

    # the face is seen from BELOW, so its free side -- and its normal -- is -y
    # The synthetic face is unbounded, and the sensor sits at x = 0, so hits
    # reach +/-7.6 m along it: the segment has to be at least that wide or §5's
    # extent condition rejects the far ones for being past its end.
    seg = [{'id': 0, 'p0': [-20.0, B4_FACE_Y], 'p1': [20.0, B4_FACE_Y],
            'theta': 180.0, 'normal': [0.0, -1.0]}]
    attrib = gw.attribute(returns, seg, RHO, gw.EPS_FACTOR * RHO)
    assert np.all(attrib['seg'] == 0), 'every return is on the one face'
    obs = gw.observations(returns, attrib, seg)[0]
    want = np.bincount((want_bearing / 5.0).astype(int),
                       minlength=gw.BEARING_BINS).tolist()
    assert obs['bearing_hist'] == want
    assert obs['n_scans'] == 2, 'two poses are two observations'
    assert obs['n_hits'] == int(np.isfinite(data['ranges']).sum())
    assert obs['n_hits'] > obs['n_scans'], \
        'n_hits is not the observation count and must not be mistaken for it'


def test_b4_break_omitting_base_t_scan():
    """The named break: no base_T_scan, so every hit lands 0.032 m off the face.

    The BEARING histogram is untouched, and that is worth stating rather than
    discovering: `p - o = R_map_scan . (r cos a, r sin a)`, and base_T_scan is a
    pure translation with zero yaw, so it cannot rotate a ray. What it moves is
    where the ray ENDS -- which is what the attribution and B1 read, and B1 is
    the check that catches it on a real map.
    """
    data = synthetic_scan(B4_FACE_Y, B4_POSES)
    good = gw.scan_returns(data, pose_arrays(B4_POSES), BASE_T_SCAN,
                           (0.0, 0.0, 0.0), R_MAX)
    broken = gw.scan_returns(data, pose_arrays(B4_POSES), (0.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0), R_MAX)
    off = np.abs(broken['py'] - B4_FACE_Y)
    want_off = abs(BASE_T_SCAN[0] * math.sin(B4_YAW))   # the perpendicular part
    assert np.allclose(off, want_off, atol=1e-9), \
        (f'every hit must leave the face by the perpendicular part of the '
         f'dropped offset, {want_off:.5f} m; worst {off.max():.5f}')
    assert np.allclose(np.hypot(broken['px'] - good['px'],
                                broken['py'] - good['py']),
                       abs(BASE_T_SCAN[0]), atol=1e-9), \
        'and every one must move by exactly the whole offset that was dropped'
    assert np.allclose(np.sort(broken['bearing']), np.sort(good['bearing']),
                       atol=1e-9), \
        'a pure sensor translation cannot change a bearing'


def test_scan_returns_drop_what_never_marked_a_cell_occupied():
    """r >= r_max, +inf and NaN are not observations of a wall.

    The relay filled exactly those at 7.95 m so Karto would trace them as FREE
    space, so a return at or above r_max cleared a cell and never occupied one.
    Keeping them would put a wall observation wherever the sensor saw nothing.
    """
    geom = dict(SCAN_GEOM, n_beams=6, angle_increment=math.pi / 3.0)
    data = {'geom': geom, 'stamps': np.array([1.0]),
            'ranges': np.array([[0.1, 0.5, 7.89, 7.9, np.inf, np.nan]])}
    poses = (np.zeros(1), np.zeros(1), np.zeros(1), np.ones(1, dtype=bool))
    out = gw.scan_returns(data, poses, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), R_MAX)
    assert out['n_kept'] == 2, 'only 0.5 and 7.89 are inside [range_min, r_max)'
    assert sorted(round(float(r), 2) for r in out['range']) == [0.5, 7.89]
    assert out['n_at_or_over_r_max'] == 1 and out['n_infinite'] == 2


def test_interpolate_poses_uses_the_shortest_arc_and_drops_the_outside():
    """Linear in position, shortest arc in yaw, and no extrapolation.

    The wrap case is the one that matters: a robot going from +179° to -179°
    turned 2°, and interpolating the wrapped values would report it spinning
    358° the other way -- through every bearing bin on the way.
    """
    tf = np.array([[10.0, 0.0, 0.0, math.radians(179.0)],
                   [11.0, 2.0, 4.0, math.radians(-179.0)]])
    x, y, yaw, ok = gw.interpolate_poses(
        tf, np.array([9.5, 10.0, 10.5, 11.0, 11.5]))
    assert ok.tolist() == [False, True, True, True, False], \
        'a stamp outside the transform span is dropped, never extrapolated'
    assert (x[2], y[2]) == (1.0, 2.0), 'position is linear'
    assert abs(math.degrees(yaw[2]) - 180.0) < 1e-9, \
        f'the midpoint of +179 and -179 is 180, got {math.degrees(yaw[2])}'


def test_attribution_needs_the_ray_to_come_from_the_free_side():
    """§5's third condition, and the side contradiction it defines.

    Two returns on one face's line, one seen from the free side and one from
    behind it. The first is attributed; the second satisfies the distance and
    extent conditions and no segment's side condition, which is exactly what a
    side contradiction is -- the map says this face was seen from one side and
    the ray came from the other.
    """
    returns = {'px': np.array([1.0, 2.0]), 'py': np.array([0.0, 0.0]),
               'ox': np.array([1.0, 2.0]), 'oy': np.array([-1.0, 1.0]),
               'bearing': np.array([90.0, 270.0]),
               'scan': np.array([0, 0]), 'stamp': np.array([1.0, 1.0])}
    seg = [{'id': 0, 'p0': [0.0, 0.0], 'p1': [5.0, 0.0], 'theta': 0.0,
            'normal': [0.0, -1.0]}]         # face looks DOWN, free side is -y
    attrib = gw.attribute(returns, seg, RHO, gw.EPS_FACTOR * RHO)
    assert attrib['seg'].tolist() == [0, -1]
    assert attrib['contradiction'].tolist() == [False, True]
    # incidence: straight on from the free side is 0 deg
    assert abs(attrib['incidence'][0]) < 1e-9

    flipped = gw.attribute(returns, seg, RHO, gw.EPS_FACTOR * RHO,
                           flip_normals=True)
    assert flipped['seg'].tolist() == [-1, 0], \
        'B2\'s break: flipping the normals swaps which side is contradictory'


def test_observations_counts_scans_not_hits():
    """`n_scans` is §5's observation count; `n_hits` never is.

    Returns within one scan are not independent -- one pose, one rangefinder, one
    instant -- so a face swept by 200 beams of a single scan has been observed
    once. The schema requirement in §0.1 is about the DIRECTIONS, and this is the
    count that goes beside them.
    """
    n = 5
    returns = {'px': np.full(n, 1.0), 'py': np.zeros(n),
               'ox': np.full(n, 1.0), 'oy': np.full(n, -1.0),
               'bearing': np.array([1.0, 6.0, 91.0, 91.0, 359.0]),
               'scan': np.array([0, 0, 0, 1, 1]),
               'stamp': np.array([1.0, 1.0, 1.0, 2.0, 2.0])}
    seg = [{'id': 0, 'p0': [0.0, 0.0], 'p1': [5.0, 0.0], 'theta': 0.0,
            'normal': [0.0, -1.0]}]
    attrib = gw.attribute(returns, seg, RHO, gw.EPS_FACTOR * RHO)
    obs = gw.observations(returns, attrib, seg)[0]
    assert obs['n_hits'] == 5 and obs['n_scans'] == 2
    assert obs['t_first'] == 1.0 and obs['t_last'] == 2.0
    assert sum(obs['bearing_hist']) == 5
    # 5° bins: 1° -> 0, 6° -> 1, 91° -> 18, 359° -> 71
    assert obs['bearing_hist'][0] == 1 and obs['bearing_hist'][1] == 1, \
        '1 deg and 6 deg are not the same bin'
    assert obs['bearing_hist'][18] == 2 and obs['bearing_hist'][71] == 1
    assert len(obs['bearing_hist']) == 72 and len(obs['incidence_hist']) == 9


def test_compose_base_scan_walks_the_two_hop_chain():
    """C7: composed from /tf_static, not assumed, and it refuses a missing hop."""
    static = {('robot_0/base_footprint', 'robot_0/base_link'):
              (0.0, 0.0, 0.0, 0.0, 0.0),
              ('robot_0/base_link', 'robot_0/base_scan'):
              (-0.032, 0.0, 0.0, 0.0, 0.0)}
    (x, y, yaw), hops = gw.compose_base_scan(static, 0)
    assert (round(x, 6), round(y, 6), round(yaw, 6)) == (-0.032, 0.0, 0.0)
    assert [h['child'] for h in hops] == ['robot_0/base_link',
                                          'robot_0/base_scan']
    with pytest.raises(SystemExit):
        gw.compose_base_scan({}, 0)


def test_b1_counts_a_return_within_one_cell_of_occupied():
    """B1's measure, and its named break, on a grid small enough to count by hand.

    The lattice is Karto's, trinary_map.CELL_CENTRE_OFFSET = 0.0: cell (r, c) is
    centred on origin + (c, r)*rho, so cell (3, 2) sits at (0.20, 0.30) and NOT at
    the (0.25, 0.35) a ROS reading would give it. The three returns are the cell's
    own centre, one cell along, and two cells along -- which is what B1's one-cell
    dilation has to separate.

    Convention B is the same PGM read without the row flip. The returns keep
    their place and the walls move, so a correct pose chain scores near nothing --
    which is the point: B1 is the check that would catch a placement error.
    """
    grid = blank(5, 5, FREE)
    grid[3, 2] = OCC                       # occupied cell centred at (0.20, 0.30)
    rho, origin = 0.1, (0.0, 0.0)
    returns = {'px': np.array([0.20, 0.30, 0.40, 9.0]),
               'py': np.array([0.30, 0.30, 0.30, 9.0])}
    good = gw.on_occupied_share(returns, grid, rho, origin)
    assert good['n_on_occupied'] == 2, \
        'the cell itself and one cell away count; two cells away does not'
    assert good['n_outside_map'] == 1 and good['convention'] == 'A'

    # The POST HOC denominator: in-grid returns only. Three of the four are on
    # the grid, two of those are on a wall.
    assert good['n_returns'] == 4 and good['n_in_grid'] == 3
    assert good['share'] == pytest.approx(2 / 3)
    assert good['share_all'] == pytest.approx(2 / 4), \
        'the original all-returns figure is kept, so nothing is hidden'
    assert good['share_offgrid'] == pytest.approx(1 / 4)
    assert 'POST HOC' in good['definition']
    broken = gw.on_occupied_share(returns, grid, rho, origin,
                                  convention_b=True)
    assert broken['n_on_occupied'] < good['n_on_occupied'], \
        'convention B must move the wall away from the returns'
    assert broken['n_in_grid'] == good['n_in_grid'], \
        'the break moves walls, not grid membership, so it cannot pass by ' \
        'shrinking the denominator'
    assert broken['share'] < good['share']


def test_the_off_wall_sample_excludes_returns_that_fell_off_the_grid():
    """The B1 diagnosis population is in-grid misses, matching the denominator.

    An off-grid return is not evidence that the map has no wall where a wall is:
    there is no cell where it landed. Sampling it would put Karto's grid-extent
    shortfall back into the one diagnosis meant to separate a missing wall from a
    misplaced return.
    """
    grid = blank(5, 5, FREE)
    grid[3, 2] = OCC
    rho, origin = 0.1, (0.0, 0.0)
    returns = {'px': np.array([0.20, 0.40, 9.0]),
               'py': np.array([0.30, 0.30, 9.0])}
    b1 = gw.on_occupied_share(returns, grid, rho, origin)
    assert (b1['n_returns'], b1['n_in_grid'], b1['n_on_occupied']) == (3, 2, 1)
    pts = gw.off_wall_sample(returns, b1['hit'], b1['inside'])
    assert pts.shape == (1, 2), 'the one in-grid miss, not the off-grid return'
    assert pts[0].tolist() == pytest.approx([0.40, 0.30])
