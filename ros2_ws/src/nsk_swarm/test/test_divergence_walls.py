"""Layer 2 -- B2 divergence on walls, experiments/analysis/divergence_walls.py.

§7 of docs/specs/SPEC_b2_divergence_walls.md v0.1, on synthetic grids only: no
corpus, no bag, no map, no graph JSON, no world SDF, no git. The corpus gates G0
and G1-on-real-maps are the run's job and are not here -- nothing in CI has the
maps, and S3 is where they are printed.

THE TABLE BELOW WAS WRITTEN BEFORE THE CODE WAS RUN. It is the pre-registration
the task asked for: each row is the class or share the case must produce, fixed
in advance, so that a test which merely reports whatever the code does would be
visibly not this.

  id   case                                                 expected
  ───────────────────────────────────────────────────────────────────────────
  U1   A and B see the same wall                            corroborated 12/12
  U2   a ray through a wall only A has                      1 contradicted,
                                                            11 unobserved
  U3   the same ray at 20 deg to the face line              0 contradicted at
                                                            theta_g 30; 1 at 15
  U4   a ray stopped by an occluder before the wall         0 contradicted;
                                                            1 without occluder
  U5   a ray whose hit lies within eps beyond the face      0 contradicted;
                                                            1 at s + eps
  U6   a one-cell wall from opposite sides, both ways       0 contradicted each
                                                            way, unobserved 1.0
  U7   45 deg crossing 0.005 m from an element's end        element 5
                                                            contradicted, 1 in
                                                            total
  U8   U1, U2, U6 and U7 with B offset (0.03, 0.07) m       the same classes
  X1   frame sign flipped                                   G1 fails;
                                                            corroborated
                                                            1.000 -> 0.000
  X2   a synthetic 1.0 m face (10 elements) in free space   >= 90 % of its
                                                            1.0 m contradicted
  X3   B given no rays                                      contradicted
                                                            exactly 0.0;
                                                            unobserved =
                                                            1 - corroborated
  X4   B given no faces                                     corroborated
                                                            exactly 0.0
  X5   theta_g swept 0-45 deg (§7's range, wider than §6's) contradicted
                                                            non-increasing,
                                                            and it does fall
  S1   supercover of a 45 deg segment through two corners   exactly the 7 cells
                                                            the two corners
                                                            touch
  S2   supercover of an axis-aligned segment                exactly 4 cells
  S3   31 deg ray whose element Karto's line steps past     contradicted here,
                                                            and the Bresenham
                                                            cell set lacks it

WHAT THE FIXTURES ARE. `wall_grid` paints one row of OCCUPIED with FREE on one
side and leaves the rest UNKNOWN, so the slab exposes exactly ONE face per
column: an occupied-unknown boundary gives no element (Def 1, graph_walls
face_elements). That is what makes the element count predictable -- 12 columns,
12 elements, all with dir '-y' -- and it is also why U6 can show the same cell
from two sides without the two faces ever matching.

`seg_of` is supplied by the test rather than by the extractor: these grids are
too small for the Hough extractor's l_min, and §3's unit is the element, so the
tests assign elements to faces directly and keep the extractor out of the
question being asked.

THE OFFSET CASES (U8) move B by a spawn difference that is not a whole number of
cells, which is the realistic case: two robots' maps of one wall, discretised on
lattices offset by a fraction of a cell. (0.03, 0.07) is 0.076 m, inside
eps = 0.171 m, so T1 must still match.

The module is loaded by location, the way test_robot_divergence.py:47 loads its
script: experiments/ is not an importable package. Importing divergence_walls
costs numpy, PyYAML and graph_walls -- no scipy, no PIL, no ROS, and no file
reads at module scope -- so this file keeps collecting in CI's ros:jazzy
container, where a module-level import of any of those is a COLLECTION error
that takes the whole nsk_swarm suite down.
"""

import importlib.util
import math
import os

import numpy as np
import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
ANALYSIS = os.path.join(REPO_ROOT, 'experiments', 'analysis')


def load_module(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ANALYSIS, f'{name}.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dw = load_module('divergence_walls')
gw = dw.gw

OCC, FREE, UNK = gw.OCC, gw.FREE, gw.UNKNOWN
RHO = 0.1
ORIGIN = (0.0, 0.0)
EPS = gw.EPS_FACTOR * RHO            # 0.1707 m
OFFSET = (0.03, 0.07)                # U8's lattice offset, 0.076 m


# ─────────────────────────────── fixtures ────────────────────────────────────

def blank(h, w, state=UNK):
    return np.full((h, w), state, dtype=np.uint8)


def wall_grid(h=12, w=12, wall_row=6):
    """One occupied row with FREE below it and UNKNOWN above.

    Exactly `w` elements, all dir '-y', at y = wall_row*rho - rho/2 and
    x = col*rho. Element i is column i: every element comes from the fourth
    DIRS pass, which takes its cells in row-major order.
    """
    g = blank(h, w)
    g[:wall_row, :] = FREE
    g[wall_row, :] = OCC
    return g


def side(grid, seg_of=None, origin=ORIGIN, rho=RHO):
    """An A-side: elements from the real extractor, faces assigned by the test."""
    el = gw.face_elements(grid, rho, origin)
    if seg_of is None:
        seg_of = np.zeros(el['n'], dtype=np.int64)
    return {'el': el, 'seg_of': seg_of, 'grid': grid, 'rho': rho,
            'origin': origin, 'shape': grid.shape}


def no_faces():
    return {'n': 0}


def b_faces(grid, offset=(0.0, 0.0), origin=ORIGIN, rho=RHO):
    """B's elements, in A's frame, with B displaced by `offset`."""
    el = gw.face_elements(grid, rho, origin)
    return dw.translate_elements(el, offset[0], offset[1])


def rays(*spec, offset=(0.0, 0.0)):
    """Rays as (ox, oy, heading_deg, free_len), already in A's frame."""
    if not spec:
        z = np.zeros(0)
        return {'ox': z, 'oy': z, 'dx': z, 'dy': z, 'free_len': z}
    a = np.array([[s[0], s[1], math.radians(s[2]), s[3]] for s in spec])
    return {'ox': a[:, 0] + offset[0], 'oy': a[:, 1] + offset[1],
            'dx': np.cos(a[:, 2]), 'dy': np.sin(a[:, 2]), 'free_len': a[:, 3]}


def run(a, el_b, rs, theta_g=dw.THETA_G_DEG, k=dw.K_PRIMARY, skip_t1=False):
    return dw.classify(a['el'], a['seg_of'], el_b, rs, a['rho'], a['origin'],
                       a['shape'], theta_g_deg=theta_g, k=k, skip_t1=skip_t1)


def counts(result):
    """(corroborated, contradicted, unobserved) element counts."""
    cls = result['cls'][result['assigned']]
    return (int((cls == dw.CORROBORATED).sum()),
            int((cls == dw.CONTRADICTED).sum()),
            int((cls == dw.UNOBSERVED).sum()))


# ───────────────────────────── the unit cases ────────────────────────────────

@pytest.mark.parametrize('offset', [(0.0, 0.0), OFFSET], ids=['aligned', 'offset'])
def test_u1_same_wall_is_corroborated(offset):
    """U1 (and U8): B's matching face corroborates every element of A."""
    a = side(wall_grid())
    el_b = b_faces(wall_grid(), offset=offset)
    corr, contra, unobs = counts(run(a, el_b, rays()))
    assert (corr, contra, unobs) == (12, 0, 0)
    assert dw.shares(run(a, el_b, rays()))['corroborated'] == 1.0


@pytest.mark.parametrize('offset', [(0.0, 0.0), OFFSET], ids=['aligned', 'offset'])
def test_u2_ray_through_a_only_wall_is_contradicted(offset):
    """U2 (and U8): B crossed the place and came back with nothing."""
    a = side(wall_grid())
    r = rays((0.0, 0.1, 90.0, 1.0), offset=offset)
    res = run(a, no_faces(), r)
    corr, contra, unobs = counts(res)
    assert (corr, contra) == (0, 1)
    assert unobs == 11
    assert res['contradicted'][0]          # the element at x = 0.0


def test_u3_grazing_ray_is_unobserved():
    """U3: 20 deg to the face line is under theta_g = 30 and must not count.

    The same ray at theta_g = 15 DOES contradict, which is what makes this a
    test of condition 2 rather than a ray that happened to miss.
    """
    a = side(wall_grid(h=12, w=24))
    r = rays((0.0, 0.1, 20.0, 2.0))
    assert counts(run(a, no_faces(), r, theta_g=30.0))[1] == 0
    assert counts(run(a, no_faces(), r, theta_g=15.0))[1] == 1


def test_u4_occluded_ray_is_unobserved():
    """U4: the ray stops short, so it never crossed the place at all."""
    a = side(wall_grid())
    occluded = rays((0.0, 0.1, 90.0, 0.2))        # s = 0.45, stops at 0.2
    clear = rays((0.0, 0.1, 90.0, 1.0))
    assert counts(run(a, no_faces(), occluded))[1] == 0
    assert counts(run(a, no_faces(), clear))[1] == 1


def test_u5_hit_within_eps_beyond_the_face_is_unobserved():
    """U5: condition 3 wants the ray to RUN ON, by eps, not merely to arrive."""
    a = side(wall_grid())
    s = 0.45
    short = rays((0.0, 0.1, 90.0, s + 0.5 * EPS))
    past = rays((0.0, 0.1, 90.0, s + EPS + 0.01))
    assert counts(run(a, no_faces(), short))[1] == 0
    assert counts(run(a, no_faces(), past))[1] == 1


@pytest.mark.parametrize('offset', [(0.0, 0.0), OFFSET], ids=['aligned', 'offset'])
def test_u6_one_cell_wall_from_opposite_sides(offset):
    """U6 (and U8): each side is unobserved by the other, never contradicted.

    A's face looks down into free space and B's looks up. The dirs differ, so T1
    cannot match; and B's ray arrives from behind A's face, so `d . n > 0` and
    condition 1 refuses it. A face is what its own robot saw from its own free
    side, and evidence about the other side is evidence about the other face.
    """
    a_grid, b_grid = blank(12, 12), blank(12, 12)
    a_grid[6, 6], a_grid[5, 6] = OCC, FREE        # A: free BELOW
    b_grid[6, 6], b_grid[7, 6] = OCC, FREE        # B: free ABOVE

    a = side(a_grid)
    from_above = rays((0.6, 1.1, 270.0, 0.4), offset=offset)
    res = run(a, b_faces(b_grid, offset=offset), from_above)
    assert counts(res) == (0, 0, 1)

    b = side(b_grid)
    from_below = rays((0.6, 0.1, 90.0, 0.4), offset=(-offset[0], -offset[1]))
    res = run(b, b_faces(a_grid, offset=(-offset[0], -offset[1])), from_below)
    assert counts(res) == (0, 0, 1)


@pytest.mark.parametrize('offset', [(0.0, 0.0), OFFSET], ids=['aligned', 'offset'])
def test_u7_crossing_near_an_element_end_is_found(offset):
    """U7 (and U8): a crossing 0.005 m from an element's end still counts.

    The 45 deg ray crosses y = 0.55 at x = 0.545, inside element 5's closed
    segment [0.45, 0.55] by 5 mm. This is the test of condition 1's CLOSED
    `|w| <= rho/2`: a midpoint-only or half-open test loses the crossing.

    It is not the test of the walk -- Karto's Bresenham happens to visit this
    element's free cell too. S3 is the case where the walk decides the class.
    """
    a = side(wall_grid())
    r = rays((0.055, 0.06, 45.0, 1.2), offset=offset)
    res = run(a, no_faces(), r)
    assert counts(res)[1] == 1
    assert res['contradicted'][5]


# ──────────────────────────── the named breaks ───────────────────────────────

def test_x1_frame_sign_flip_fails_g1_and_collapses_corroboration():
    """X1: the wrong sign puts B's evidence twice the spawn gap away."""
    spawn_a, spawn_b = (0.0, 0.0), (1.0, 0.0)
    parked = np.array([[0.5, 0.5]])
    expected = np.array([[0.5 + 1.0, 0.5]])
    assert dw.g1_frame_check(parked, spawn_b, spawn_a, expected)['holds']
    assert not dw.g1_frame_check(parked, spawn_b, spawn_a, expected,
                                 flip=True)['holds']

    a = side(wall_grid())
    own = dw.translate_elements(gw.face_elements(wall_grid(), RHO, ORIGIN),
                                -1.0, 0.0)
    for flip, expect in ((False, 1.0), (True, 0.0)):
        bx, by = dw.to_frame_a(own['x'], own['y'], spawn_b, spawn_a, flip=flip)
        el_b = dict(own, x=bx, y=by)
        assert dw.shares(run(a, el_b, rays()))['corroborated'] == expect


def test_x2_synthetic_face_in_free_space_is_contradicted():
    """X2: a face nobody's wall explains, in a strip both maps call free.

    The bar sits 2.3 m from the only real wall, well past §7's 1 m, and B's ten
    rays cross it head-on from the free side below.
    """
    g = blank(30, 12)
    g[0:2, :] = FREE
    g[2, 1:11] = OCC                       # the synthetic 1.0 m face
    g[20:25, :] = FREE
    g[25, :] = OCC                         # the real wall, 2.3 m away
    seg_of = np.concatenate([np.zeros(10, np.int64), np.ones(12, np.int64)])
    a = side(g, seg_of=seg_of)
    assert a['el']['n'] == 22

    r = rays(*[(0.1 * c, 0.05, 90.0, 1.0) for c in range(1, 11)])
    res = run(a, no_faces(), r)
    face0 = dw.per_face(res, 2)[0]
    assert face0['length'] == pytest.approx(1.0)
    assert face0['contradicted_m'] >= 0.9


def test_x3_no_rays_gives_no_contradiction():
    """X3: with nothing crossing, the split is corroborated against unobserved."""
    a = side(wall_grid())
    el_b = b_faces(wall_grid(h=12, w=6))     # B saw half the wall
    s = dw.shares(run(a, el_b, rays()))
    assert s['contradicted'] == 0.0
    assert s['unobserved'] == pytest.approx(1.0 - s['corroborated'])
    assert 0.0 < s['corroborated'] < 1.0     # the case is not degenerate


def test_x4_no_faces_gives_no_corroboration():
    """X4: T1 has nothing to match, so every element falls to T2 or T3."""
    a = side(wall_grid())
    s = dw.shares(run(a, no_faces(), rays((0.0, 0.1, 90.0, 1.0))))
    assert s['corroborated'] == 0.0
    assert s['contradicted'] + s['unobserved'] == pytest.approx(1.0)


def test_x5_contradicted_share_is_non_increasing_in_theta_g():
    """X5: raising theta_g only ever removes crossings.

    Swept 0-45 deg as §7 states, which is wider than §6's reported 15-45: at 0
    nothing is grazing and both rays count, and the 20 deg ray drops out once
    theta_g passes it.
    """
    a = side(wall_grid(h=12, w=24))
    r = rays((0.0, 0.1, 20.0, 2.0), (1.5, 0.1, 90.0, 1.0))
    sweep = dw.theta_sweep(a['el'], a['seg_of'], no_faces(), r, a['rho'],
                           a['origin'], a['shape'],
                           sweep=tuple(float(t) for t in range(0, 50, 5)))
    shares = [row['contradicted'] for row in sweep]
    assert all(x >= y for x, y in zip(shares, shares[1:]))
    assert shares[0] > shares[-1]            # and it does fall


# ────────────────────────────── the walk itself ──────────────────────────────

def cells(flat, w):
    return {(int(f) % w, int(f) // w) for f in flat}


def test_s1_supercover_takes_both_cells_at_a_corner():
    """S1: a 45 deg segment through two lattice corners touches seven cells.

    The segment runs corner to corner, so at each corner all four cells meeting
    there are touched. Karto's Bresenham steps diagonally and reports four;
    anything this file does with the missing three would be wrong.
    """
    flat, _ray = dw.supercover(np.array([0.0]), np.array([0.0]),
                               np.array([math.sqrt(0.5)]),
                               np.array([math.sqrt(0.5)]),
                               np.array([0.2 * math.sqrt(2.0)]),
                               RHO, ORIGIN, (6, 6))
    assert cells(flat, 6) == {(0, 0), (1, 0), (0, 1), (1, 1),
                              (2, 1), (1, 2), (2, 2)}


def test_s2_supercover_of_an_axis_aligned_segment():
    """S2: straight along +x, four cells and no diagonal neighbours."""
    flat, _ray = dw.supercover(np.array([0.0]), np.array([0.0]),
                               np.array([1.0]), np.array([0.0]),
                               np.array([0.3]), RHO, ORIGIN, (6, 6))
    assert cells(flat, 6) == {(0, 0), (1, 0), (2, 0), (3, 0)}


def test_s3_bresenham_would_lose_a_crossing_the_supercover_keeps():
    """S3: the walk decides the class, and Karto's line gets it wrong.

    A 31 deg ray from (0.0, 0.06) crosses y = 0.55 at x = 0.8155, which is
    inside element 8 by 15 mm and at |d . n| = 0.515, clear of theta_g = 30.
    Karto's TraceLine never enters that element's free cell -- it steps past the
    corner -- so a walk borrowed from diagnose_karto would classify the element
    unobserved. The supercover visits it and the element is contradicted.

    This is §4's reason for not reusing trace_rays here, as a difference in a
    class rather than a difference in a cell list.
    """
    dk = load_module('diagnose_karto')
    a = side(wall_grid(h=12, w=14))
    ox, oy, ang, flen = 0.0, 0.06, 31.0, 1.4

    res = run(a, no_faces(), rays((ox, oy, ang, flen)))
    assert counts(res)[1] == 1
    assert res['contradicted'][8]

    rad = math.radians(ang)
    g0 = dk.world_to_grid(np.array([ox]), np.array([oy]), ORIGIN, RHO)
    g1 = dk.world_to_grid(np.array([ox + flen * math.cos(rad)]),
                          np.array([oy + flen * math.sin(rad)]), ORIGIN, RHO)
    cx, cy, _r = dk.trace_rays(g0[0], g0[1], g1[0], g1[1])
    assert (8, 5) not in {(int(x), int(y)) for x, y in zip(cx, cy)}


def test_c0_floor_is_zero_on_a_map_against_its_own_rays():
    """§7 C0 on a synthetic map: A's own ray stops AT its own wall.

    A ray that ends on the face it built cannot run on by eps past it, so
    condition 3 refuses it and the floor is zero. This is the shape C0 measures
    on the corpus; the number there is the run's business.
    """
    a = side(wall_grid())
    own = rays((0.0, 0.1, 90.0, 0.45))       # ends exactly at the face
    c0 = dw.c0_floor(a['el'], a['seg_of'], own, a['rho'], a['origin'],
                     a['shape'])
    assert c0['share'] == 0.0
    assert c0['holds']
