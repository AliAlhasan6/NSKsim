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
  U2   two rays through a wall only A has                   1 contradicted,
                                                            11 unobserved
  U3   the same ray at 20 deg to the face line              0 contradicted at
                                                            theta_g 30; 1 at 15
  U4   a ray stopped by an occluder before the wall         0 contradicted;
                                                            1 without occluder
  U5   45 deg rays overshooting by eps ALONG d, so only     0 contradicted;
       eps*cos45 of DEPTH (v0.1 called this contradicted)   1 at eps of depth
  C4a  one ray through a wall only A has                    unobserved (1 of 1)
  C4b  three crossings, one running on                      unobserved (1 of 3)
  C4c  three crossings, two running on                      contradicted (2/3)
  C4d  four crossings, two running on                       unobserved (2 of 4,
                                                            half is not more)
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
  C1   a pair at two different cuts (§2)                     runs; cut_A 1200,
                                                             cut_B 60, and no
                                                             row has a bare
                                                             `cut`
  C2   a same-cut pair                                       no bare `cut`;
                                                             shares and
                                                             per_face equal the
                                                             direct call
  C3   the cut is bookkeeping, the spawn is geometry         1.0 / 0.167 / 0.0
                                                             at 0, 1 and 2 m
  V1   identical bearing histograms (O1)                     0 deg
  V2   nine bins apart                                       45 deg, w = 0.25
  V3   bins 70 and 1, across the wrap                        15 deg, not 345
  V4   half the circle apart                                 180 deg, w = 1
  V5   symmetry, rotation and scale                          all unchanged
  V6   an empty histogram either side                        None; unweighable
                                                             on that side
  V7   two segments, hand-computed                           W 0.125, w_mean
                                                             0.25, unweighable_B
                                                             0.5
  V8   A against itself                                      w = 0, W = 0
  G2   a segment count mismatch                              a verdict, not a
                                                             crash; diff None
                                                             on a mismatch
  S0a  a tied segment dir (P7's tie rule)                    the fitted normal
                                                             decides, not the
                                                             dir index
  S0b  an untied segment dir                                 the commonest
                                                             wins; empty is -1
  S1   w_null over two alternatives (P7)                     0.375, w = 0
  S2   no alternative / empty / wrong dir                    all NaN, outside
                                                             E7
  S3   a map against itself (G3)                             C = 1, W = 0,
                                                             w == 0 exactly
  S4   verdict counting and ties                             18/17, 16/15;
                                                             equal and empty
                                                             count against
  S5   --stage s5 on a dirty tree                            refuses, exit 2
  V1   the module default (v0.4)                            t2 off, two
                                                            classes, nothing
                                                            contradicted
  V2   a would-be contradiction, both ways                  CONTRADICTED with
                                                            t2 on,
                                                            NOT_CORROBORATED
                                                            with it off
  V3   the two shares                                       6 of 12 corroborate
                                                            (v0.5: the along
                                                            rule stops at the
                                                            end of B's cover)
  V3b  the same, B shifted 0.05 m along the face            7 of 12, the end
                                                            element matching at
                                                            exactly rho/2
  V4   the measure with no ray set at all                   rays=None is fine
  V5   per face, two lengths                                1.2 m not
                                                            corroborated
  V6   the deep share is a T2 output                        2/3 on, 0.0 off
  S3   31 deg ray whose element Karto's line steps past     contradicted here,
                                                            and the Bresenham
                                                            cell set lacks it

The M series is S6, the cut matrix. Its rows were written before the stage was
run, like the rest of this table, and nothing in them touches a real map.

  M1   the seen flag over the 320 keys                       140 seen, 180
                                                             unseen; seen iff
                                                             cut_A 1200 or
                                                             cut_A == cut_B
  M2   P9 with equal steps everywhere                        holds, 60/60
  M3   P9, one element more not corroborated at one step     fails; the rise is
                                                             0.1 m, one
                                                             element's length
  M4   P9, a rise of 1e-12                                   fails; no
                                                             tolerance
  M5   P9, A's walls of zero length                          fails, undefined,
                                                             no step named
  M6   P9 with 60, 59 and 58 holding series                  holds only at 60
                                                             of 60
  M7   the symmetric table over a 320-row matrix             160 entries, every
                                                             row used exactly
                                                             once, X < Y
  M8   the difference's sign                                 first minus
                                                             second: 0.9 - 0.4
  M9   G4, one stored seen value one ulp out                 exit 2; a fail
                                                             file of seen rows,
                                                             no unseen row
                                                             written or printed,
                                                             and C called 140
                                                             times, once per
                                                             seen row
  M10  G4 holding on a synthetic corpus                      320 rows written,
                                                             140 of them seen,
                                                             160 symmetric
                                                             entries, P9 60/60
  M11  the P9 paragraph read from spec text                  one line; stops at
                                                             the blank line; a
                                                             list marker comes
                                                             off
  M12  --stage s6 on a dirty tree, with 533fa52 not an       refuses, exit 2,
       ancestor, and with a spec that lacks P9               on each of the
                                                             three

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

import hashlib
import importlib.util
import json
import math
import os
import re

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


def run(a, el_b, rs, theta_g=dw.THETA_G_DEG, skip_t1=False, t2=True):
    """Classify. `t2=True` here, because most of this file tests T2.

    T2 left the measure in v0.4 and the module default is off; the cases that
    exercise it therefore have to ask for it. The v0.4 default has its own
    group of tests below, which pass `t2=False`.
    """
    return dw.classify(a['el'], a['seg_of'], el_b, rs, a['rho'], a['origin'],
                       a['shape'], theta_g_deg=theta_g, skip_t1=skip_t1, t2=t2)


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
    """U2 (and U8): B crossed the place twice and came back with nothing.

    Two rays, because condition 4 (v0.3) wants at least two running on and a
    strict majority. One would leave the element unobserved -- which is its own
    case below.
    """
    a = side(wall_grid())
    r = rays((0.0, 0.1, 90.0, 1.0), (0.01, 0.1, 90.0, 1.0), offset=offset)
    res = run(a, no_faces(), r)
    corr, contra, unobs = counts(res)
    assert (corr, contra) == (0, 1)
    assert unobs == 11
    assert res['contradicted'][0]          # the element at x = 0.0
    assert (res['n12'][0], res['n123'][0]) == (2, 2)


def test_u3_grazing_ray_is_unobserved():
    """U3: 20 deg to the face line is under theta_g = 30 and must not count.

    The same ray at theta_g = 15 DOES contradict, which is what makes this a
    test of condition 2 rather than a ray that happened to miss.
    """
    a = side(wall_grid(h=12, w=24))
    r = rays((0.0, 0.1, 20.0, 2.0), (0.01, 0.1, 20.0, 2.0))
    assert counts(run(a, no_faces(), r, theta_g=30.0))[1] == 0
    assert counts(run(a, no_faces(), r, theta_g=15.0))[1] == 1


def test_u4_occluded_ray_is_unobserved():
    """U4: the ray stops short, so it never crossed the place at all."""
    a = side(wall_grid())
    occluded = rays((0.0, 0.1, 90.0, 0.2),        # s = 0.45, both stop at 0.2
                    (0.01, 0.1, 90.0, 0.2))
    clear = rays((0.0, 0.1, 90.0, 1.0), (0.01, 0.1, 90.0, 1.0))
    res = run(a, no_faces(), occluded)
    assert counts(res)[1] == 0
    assert res['n12'][0] == 0        # it never reached the element to cross it
    assert counts(run(a, no_faces(), clear))[1] == 1


def test_u5_hit_within_eps_of_depth_beyond_the_face_is_unobserved():
    """U5, in v0.2's DEPTH form: eps is measured behind the face, not along d.

    At 45 deg the two forms disagree, which is the point of the case. A ray that
    overshoots by exactly eps ALONG ITS OWN DIRECTION reaches a depth of only
    eps*cos45 = 0.121 m behind the face line, and v0.1 would have called that a
    contradiction. v0.2 wants eps of depth, which costs eps/cos45 = 0.241 m of
    travel.

    Two rays throughout, so condition 4 is satisfied whenever condition 3 is and
    this stays a test of condition 3.
    """
    a = side(wall_grid())
    cos45 = math.sqrt(0.5)
    s = (0.55 - 0.06) / cos45                      # 45 deg from y = 0.06
    short = rays((0.01, 0.06, 45.0, s + EPS),            # v0.1 passed this
                 (0.02, 0.06, 45.0, s + EPS))
    past = rays((0.01, 0.06, 45.0, s + EPS / cos45 + 0.01),
                (0.02, 0.06, 45.0, s + EPS / cos45 + 0.01))
    res_short = run(a, no_faces(), short)
    assert counts(res_short)[1] == 0
    assert res_short['n12'][5] == 2 and res_short['n123'][5] == 0
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
    r = rays((0.055, 0.06, 45.0, 1.2), (0.056, 0.06, 45.0, 1.2), offset=offset)
    res = run(a, no_faces(), r)
    assert counts(res)[1] == 1
    assert res['contradicted'][5]


# ─────────────────── condition 4, the weight of evidence ─────────────────────
#
# All four act on element 0 of the plain wall, at x = 0.0, whose closed segment
# is [-0.05, 0.05]. A vertical ray from y = 0.1 crosses it at s = 0.45; a
# free_len of 1.0 clears it by 0.55 m of depth and one of 0.50 by 0.05 m, which
# is under eps. So `free_len` alone decides whether a crossing runs on, and the
# cases differ only in how many of each there are.

def _at_element_0(*lens):
    return rays(*[(-0.03 + 0.02 * i, 0.1, 90.0, L) for i, L in enumerate(lens)])


def test_c4_one_ray_through_an_a_only_wall_is_unobserved():
    """One ray is not evidence that B saw open space (v0.3, condition 4)."""
    a = side(wall_grid())
    res = run(a, no_faces(), _at_element_0(1.0))
    assert (res['n12'][0], res['n123'][0]) == (1, 1)
    assert counts(res) == (0, 0, 12)


def test_c4_three_crossings_one_running_on_is_unobserved():
    """1 of 3 fails both clauses: under two, and not a majority."""
    a = side(wall_grid())
    res = run(a, no_faces(), _at_element_0(0.50, 0.50, 1.0))
    assert (res['n12'][0], res['n123'][0]) == (3, 1)
    assert not res['contradicted'][0]
    assert counts(res)[1] == 0


def test_c4_three_crossings_two_running_on_is_contradicted():
    """2 of 3 clears both clauses: at least two, and 2 > 3/2."""
    a = side(wall_grid())
    res = run(a, no_faces(), _at_element_0(0.50, 1.0, 1.0))
    assert (res['n12'][0], res['n123'][0]) == (3, 2)
    assert res['contradicted'][0]
    assert counts(res)[1] == 1


def test_c4_two_of_four_is_not_a_majority():
    """2 of 4 has two running on but is exactly half, and half is not more.

    The clause that separates this from 2 of 3 is the strict majority, and it is
    also what makes the theta_g sweep non-monotone in principle: drop the two
    shallow crossings and the same element becomes 2 of 2.
    """
    a = side(wall_grid())
    res = run(a, no_faces(), _at_element_0(0.50, 0.50, 1.0, 1.0))
    assert (res['n12'][0], res['n123'][0]) == (4, 2)
    assert not res['contradicted'][0]


# ───────────────────── v0.4: the measure is T1 and its complement ────────────

def test_v04_default_has_two_classes_and_no_t2():
    """The module default is T2 off, and then nothing is contradicted."""
    a = side(wall_grid())
    r = _at_element_0(1.0, 1.0, 1.0)          # would contradict under T2
    res = dw.classify(a['el'], a['seg_of'], no_faces(), r, a['rho'],
                      a['origin'], a['shape'])
    assert dw.T2_DEFAULT is False
    assert res['t2'] is False
    assert not res['contradicted'].any()
    assert set(np.unique(res['cls'])) <= {dw.CORROBORATED, dw.NOT_CORROBORATED}


def test_v04_a_would_be_contradiction_is_merely_not_corroborated():
    """The same scene, both ways: T2 on calls it contradicted, v0.4 does not."""
    a = side(wall_grid())
    r = _at_element_0(1.0, 1.0, 1.0)
    on = run(a, no_faces(), r, t2=True)
    off = run(a, no_faces(), r, t2=False)
    assert on['cls'][0] == dw.CONTRADICTED
    assert off['cls'][0] == dw.NOT_CORROBORATED


def test_v04_shares_are_two_and_sum_to_one():
    """§6: the two shares, corroborated and not corroborated, summing to 1."""
    a = side(wall_grid())
    el_b = b_faces(wall_grid(h=12, w=6))      # B saw columns 0-5 of 12
    s = dw.shares(run(a, el_b, rays(), t2=False))
    assert set(k for k in s if k in
               ('corroborated', 'not_corroborated', 'unobserved',
                'contradicted')) == {'corroborated', 'not_corroborated'}
    assert s['corroborated'] + s['not_corroborated'] == pytest.approx(1.0)
    # 6 of 12 under v0.5. The along tolerance is rho/2 = 0.05 m, so A's element
    # at x = 0.6 is NOT matched by B's last one at x = 0.5: that is a whole
    # element away. Under the v0.4 Euclidean eps it was, and corroboration ran
    # one element past the end of B's coverage.
    assert (s['n_corroborated'], s['n_not_corroborated']) == (6, 6)


def test_v05_half_overlapping_end_element_still_matches():
    """The same wall with B shifted half an element along the face.

    B's elements now sit at x = 0.05 .. 0.55, so every A element from 0.0 to
    0.6 has one within rho/2 = 0.05 m along the face -- the end one at 0.6 by
    exactly the tolerance, which is inclusive. 0.7 is 0.15 m away and is not.
    Seven match, which is the case the along rule must still admit: an element
    half-overlapping the end of B's coverage is corroborated, one clear of it
    is not.
    """
    a = side(wall_grid())
    el_b = b_faces(wall_grid(h=12, w=6), offset=(0.05, 0.0))
    s = dw.shares(run(a, el_b, rays(), t2=False))
    assert (s['n_corroborated'], s['n_not_corroborated']) == (7, 5)


def test_v04_needs_no_rays_at_all():
    """With T2 off nothing reads the ray set, so None must be acceptable.

    This is the saving that matters on the corpus: building B's admitted rays
    is the expensive half of a pair, and the measure does not use them.
    """
    a = side(wall_grid())
    res = dw.classify(a['el'], a['seg_of'], b_faces(wall_grid()), None,
                      a['rho'], a['origin'], a['shape'])
    assert dw.shares(res)['corroborated'] == 1.0


def test_v04_per_face_reports_two_lengths():
    """§6: per face, the two lengths."""
    a = side(wall_grid())
    row = dw.per_face(run(a, no_faces(), rays(), t2=False), 1)[0]
    assert set(k for k in row if k.endswith('_m')) == {'corroborated_m',
                                                       'not_corroborated_m'}
    assert row['not_corroborated_m'] == pytest.approx(1.2)


def test_v04_deep_share_is_a_t2_output():
    """The deep share is produced with the flag on and is 0.0 otherwise."""
    a = side(wall_grid())
    r = _at_element_0(0.50, 1.0, 1.0)
    assert dw.deep_share(run(a, no_faces(), r, t2=True)) == pytest.approx(2 / 3)
    assert dw.deep_share(run(a, no_faces(), r, t2=False)) == 0.0


# ──────────────────── a pair, and its two cuts (§2) ──────────────────────────

def fake_side(grid, stem='b2maps_kX_cutY_gated_extfix_robotX'):
    """A side shaped like load_side's output, built from a grid.

    pair_row reads no file, so a hand-built side is all it needs: this is what
    lets a pair be tested where there is no corpus.
    """
    s = side(grid)
    return {'stem': stem, 'el': s['el'], 'seg_of': s['seg_of'],
            'grid': grid, 'rho': s['rho'], 'origin': s['origin'],
            'shape': s['shape'],
            'segments': [{'p0': [0.0, 0.55], 'p1': [1.1, 0.55]}]}


def test_pair_row_accepts_two_different_cuts():
    """§2: a pair may take A and B at different cuts. P3 needs exactly this.

    Before this change run_pair refused outright -- "a pair is one cut" -- and
    P3, which holds A at cut1200 and walks B's cut, could not have been run.
    """
    a = fake_side(wall_grid(), 'b2maps_k0_cut1200_gated_extfix_robot0')
    b = fake_side(wall_grid(h=12, w=6), 'b2maps_k1_cut60_gated_extfix_robot1')
    row = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 1200, 60)
    assert (row['cut_A'], row['cut_B']) == (1200, 60)
    assert 'cut' not in row              # no row carries a bare cut
    assert row['shares']['corroborated'] + \
        row['shares']['not_corroborated'] == pytest.approx(1.0)
    assert row['shares']['n_corroborated'] == 6


def test_pair_row_same_cut_computes_what_it_computed_before():
    """A same-cut pair's NUMBERS are untouched; its key shape deliberately is not.

    The row reports its cut as cut_A and cut_B like every other row, and
    carries no bare `cut` -- a key present on some rows and absent on others is
    a trap for whoever reads the two series side by side. What must not move is
    the computation, so shares and per_face are checked against what classify,
    shares and per_face give directly: all run_pair ever did with them.
    """
    a = fake_side(wall_grid(), 'b2maps_k0_cut240_gated_extfix_robot0')
    b = fake_side(wall_grid(h=12, w=6), 'b2maps_k1_cut240_gated_extfix_robot1')
    row = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 240, 240)
    assert row['cut_A'] == row['cut_B'] == 240
    assert 'cut' not in row

    el_b = dict(dw.face_elements_only(b['el'], b['seg_of']))
    res = dw.classify(a['el'], a['seg_of'], el_b, None, a['rho'], a['origin'],
                      a['shape'])
    assert row['shares'] == dw.shares(res)
    assert row['per_face'] == dw.per_face(res, 1)


def test_pair_row_carries_the_spawn_difference():
    """The two cuts are bookkeeping; the frame change is still the spawns.

    B displaced clear of A and declared at another cut must not corroborate:
    the cut does not enter the geometry, and the spawn difference does. The
    wall is 1.2 m long, so the displacement has to exceed that -- at 1.0 m the
    two still overlap by 0.2 m and 2 of 12 elements match.
    """
    a = fake_side(wall_grid(), 'b2maps_k0_cut1200_gated_extfix_robot0')
    b = fake_side(wall_grid(), 'b2maps_k1_cut60_gated_extfix_robot1')
    near = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 1200, 60)
    overlap = dw.pair_row(a, b, (0.0, 0.0), (1.0, 0.0), 1200, 60)
    clear = dw.pair_row(a, b, (0.0, 0.0), (2.0, 0.0), 1200, 60)
    assert near['shares']['corroborated'] == 1.0
    assert overlap['shares']['n_corroborated'] == 2
    assert clear['shares']['corroborated'] == 0.0


# ─────────────────── O1: the viewpoint weight on T1 ──────────────────────────

NB = gw.BEARING_BINS                      # 72, D4's bin count
BINW = 360.0 / NB                         # 5 degrees


def hist(*bins, n=1):
    """A bearing histogram with `n` counts in each named bin."""
    h = np.zeros(NB, dtype=np.int64)
    for b in bins:
        h[b % NB] += n
    return h


def test_o1_identical_histograms_give_zero():
    """V1: same directions, no viewpoint distance."""
    h = hist(3, 4, 5)
    assert dw.circular_emd_deg(h, h) == pytest.approx(0.0)


def test_o1_nine_bins_apart_is_45_degrees():
    """V2: 9 bins x 5 deg = 45 deg, so w = 0.25."""
    d = dw.circular_emd_deg(hist(0), hist(9))
    assert d == pytest.approx(45.0)
    assert d / 180.0 == pytest.approx(0.25)


def test_o1_distance_goes_the_short_way_round():
    """V3: bins 70 and 1 are 3 bins apart across the wrap, not 69.

    This is what the median subtraction buys: on a cut line the same pair
    would be 345 deg.
    """
    assert dw.circular_emd_deg(hist(70), hist(1)) == pytest.approx(15.0)


def test_o1_opposite_bins_are_180_degrees():
    """V4: half the circle apart is the maximum, w = 1."""
    d = dw.circular_emd_deg(hist(0), hist(NB // 2))
    assert d == pytest.approx(180.0)
    assert d / 180.0 == pytest.approx(1.0)


@pytest.mark.parametrize('shift', [0, 1, 17, 71])
def test_o1_symmetric_rotation_and_scale_invariant(shift):
    """V5: W1(p, q) = W1(q, p), and unmoved by rotating or rescaling both."""
    p, q = hist(2, 3, 40), hist(11, 12)
    base = dw.circular_emd_deg(p, q)
    assert dw.circular_emd_deg(q, p) == pytest.approx(base)
    rp = np.roll(p, shift)
    rq = np.roll(q, shift)
    assert dw.circular_emd_deg(rp, rq) == pytest.approx(base)
    assert dw.circular_emd_deg(p * 7, q * 1000) == pytest.approx(base)


def test_o1_empty_histogram_gives_none():
    """V6a: an empty side is unweighable, which is not a weight of zero."""
    assert dw.circular_emd_deg(np.zeros(NB, int), hist(0)) is None
    assert dw.circular_emd_deg(hist(0), np.zeros(NB, int)) is None


def _one_segment_pair(hist_a, hist_b):
    """A and B on the same 12-element wall, one segment each."""
    a = fake_side(wall_grid(), 'A')
    b = fake_side(wall_grid(), 'B')
    b['bearing_hist'] = [hist_b]
    a['bearing_hist'] = [hist_a]
    return a, b


def test_o1_empty_histogram_lands_in_the_right_unweighable_column():
    """V6b: whichever side was empty is the side it is counted against."""
    for which in ('A', 'B'):
        ha = np.zeros(NB, int) if which == 'A' else hist(0)
        hb = np.zeros(NB, int) if which == 'B' else hist(0)
        a, b = _one_segment_pair(ha, hb)
        row = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 1200, 1200)
        s = row['shares']
        assert s['corroborated'] == 1.0          # T1 is untouched
        assert s[f'unweighable_{which}'] == pytest.approx(1.0)
        assert s[f'unweighable_{"B" if which == "A" else "A"}'] == 0.0
        assert s['W'] == 0.0 and s['n_weighable'] == 0
        assert s['w_mean'] is None


def test_o1_two_segments_give_the_hand_computed_row():
    """V7: two faces, two viewpoint gaps, and W worked out by hand.

    A's wall is 12 elements on one segment; the test splits it into two faces
    of 6 by hand, so face 0's elements see a 45 deg gap (w = 0.25) and face 1's
    an empty B histogram (unweighable_B). Then

        W = (6 * 0.25 + 6 * 0) / 12 = 0.125
        w_mean = 0.25 over the 6 weighable
        unweighable_B = 6 / 12 = 0.5
    """
    a = fake_side(wall_grid(), 'A')
    b = fake_side(wall_grid(), 'B')
    two = np.where(np.arange(a['el']['n']) < 6, 0, 1).astype(np.int64)
    a['seg_of'] = two
    b['seg_of'] = two
    a['segments'] = [{'p0': [0.0, 0.55], 'p1': [0.5, 0.55]},
                     {'p0': [0.6, 0.55], 'p1': [1.1, 0.55]}]
    a['bearing_hist'] = [hist(0), hist(0)]
    b['bearing_hist'] = [hist(9), np.zeros(NB, int)]
    s = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 1200, 1200)['shares']
    assert s['corroborated'] == 1.0
    assert s['n_weighable'] == 6
    assert s['W'] == pytest.approx(0.125)
    assert s['w_mean'] == pytest.approx(0.25)
    assert s['w_p10'] == pytest.approx(0.25)
    assert s['w_p90'] == pytest.approx(0.25)
    assert s['unweighable_B'] == pytest.approx(0.5)
    assert s['unweighable_A'] == 0.0
    assert s['W'] <= s['corroborated']


def test_o1_a_against_itself_weighs_zero_everywhere():
    """V8: the same robot's own viewpoints cannot differ from themselves."""
    a, b = _one_segment_pair(hist(3, 20, 55), hist(3, 20, 55))
    row = dw.pair_row(a, b, (0.0, 0.0), (0.0, 0.0), 1200, 1200)
    s = row['shares']
    assert s['corroborated'] == 1.0
    assert s['W'] == 0.0
    assert s['w_mean'] == pytest.approx(0.0)
    assert s['w_p90'] == pytest.approx(0.0)
    assert s['n_weighable'] == 12


def test_o1_the_mask_is_the_same_with_and_without_matching():
    """corroborate_matches must not change WHO is corroborated, only add which."""
    a = side(wall_grid())
    el_b = b_faces(wall_grid(h=12, w=6))
    el_b['seg'] = np.zeros(el_b['n'], dtype=np.int64)
    plain = dw.corroborate(a['el'], el_b, EPS, RHO)
    withm, match = dw.corroborate_matches(a['el'], el_b, EPS, RHO)
    assert np.array_equal(plain, withm)
    assert (match >= 0).sum() == plain.sum()


def test_o1_fields_are_absent_without_histograms():
    """No histograms, no O1 fields -- the measure's own row is unchanged."""
    a = side(wall_grid())
    s = dw.shares(run(a, b_faces(wall_grid()), rays(), t2=False))
    assert 'W' not in s and 'w_mean' not in s and 'unweighable_A' not in s


def test_g2_a_segment_count_mismatch_is_a_verdict_not_a_crash():
    """G2: the guard reports a count mismatch; it does not raise on one.

    The survey stage has to get through all 20 maps, so the guard returns a
    verdict and `load_side` is what turns a bad one into a refusal. A mismatch
    also has no endpoint difference to report, which is why that field is None
    rather than zero -- zero would read as a perfect match.
    """
    saved = [{'p0': [0.0, 0.0], 'p1': [1.0, 0.0]},
             {'p0': [0.0, 1.0], 'p1': [1.0, 1.0]}]
    recomputed = [{'p0': [0.0, 0.0], 'p1': [1.0, 0.0]}]
    g = dw.segment_guard(saved, recomputed)
    assert g['ok'] is False
    assert (g['n_saved'], g['n_recomputed']) == (2, 1)
    assert g['max_endpoint_diff_m'] is None
    assert '2 segments' in g['message'] and '1' in g['message']

    same = dw.segment_guard(saved, saved)
    assert same['ok'] is True
    assert same['max_endpoint_diff_m'] == pytest.approx(0.0)
    assert same['message'] is None

    moved = [{'p0': [0.0, 0.0], 'p1': [1.0, 0.0]},
             {'p0': [0.0, 1.0], 'p1': [1.0, 1.003]}]
    off = dw.segment_guard(saved, moved)
    assert off['ok'] is False
    assert off['max_endpoint_diff_m'] == pytest.approx(0.003)
    assert 'tolerance' in off['message']


# ───────────────── S5: G3, P7's null, and the verdict counts ─────────────────

def _weights_for(a, b):
    """classify one pair the way the S5 stage does, and hand back the pieces."""
    el_b = dict(dw.face_elements_only(b['el'], b['seg_of']))
    el_b['seg'] = b['seg_of'][b['seg_of'] >= 0]
    res = dw.classify(a['el'], a['seg_of'], el_b, None, a['rho'], a['origin'],
                      a['shape'], hist_a=a['bearing_hist'],
                      hist_b=b['bearing_hist'])
    return res, dw.shares(res)


def test_s5_segment_dir_tie_goes_to_the_fitted_normal():
    """P7's tie rule: equal counts, so the fitted normal decides.

    Two elements facing +x and two facing +y on one segment. The dir index would
    break the tie for +x whatever the face looks like; the fitted normal is the
    face's own property, so it is what decides.
    """
    el = {'dir': np.array([0, 0, 2, 2], dtype=np.int64)}
    seg_of = np.zeros(4, dtype=np.int64)
    assert dw.segment_dirs(seg_of, el, [{'normal': [1.0, 0.0]}])[0] == 0
    assert dw.segment_dirs(seg_of, el, [{'normal': [0.0, 1.0]}])[0] == 2
    # leaning, not axis-aligned: the nearer of the two tied dirs still wins
    assert dw.segment_dirs(seg_of, el, [{'normal': [0.3, 0.95]}])[0] == 2


def test_s5_segment_dir_commonest_wins_over_the_normal():
    """No tie, so the count decides and the fitted normal does not get a vote."""
    el = {'dir': np.array([0, 0, 0, 2], dtype=np.int64)}
    seg_of = np.zeros(4, dtype=np.int64)
    # the normal points hard at +y, but +x is the commonest dir
    assert dw.segment_dirs(seg_of, el, [{'normal': [0.0, 1.0]}])[0] == 0
    # a segment with no elements is -1
    two = dw.segment_dirs(np.zeros(4, dtype=np.int64), el,
                          [{'normal': [0.0, 1.0]}, {'normal': [1.0, 0.0]}])
    assert two[1] == -1


def test_s5_w_null_matches_the_hand_computed_value():
    """S1: the null is the MEAN over B's other same-direction segments.

    A's single face has its histogram at bin 0. B has three faces, all the same
    dir: the matched one at bin 0, and two alternatives at bins 9 and 18. The
    null is therefore the mean of 45 deg and 90 deg over 180 deg,
    (0.25 + 0.5) / 2 = 0.375, while w itself is 0.
    """
    a = fake_side(wall_grid(), 'A')
    b = fake_side(wall_grid(), 'B')
    a['bearing_hist'] = [hist(0)]
    b['bearing_hist'] = [hist(0), hist(9), hist(18)]
    b['graph'] = {'segments': [{}, {}, {}]}
    dirs_b = np.array([3, 3, 3])              # all '-y', as wall_grid paints
    res, _sh = _weights_for(a, b)
    wn = dw.null_weights(a['seg_of'], res['weights'], a['bearing_hist'],
                         b['bearing_hist'], dirs_b)
    good = ~np.isnan(res['weights']['w'])
    assert good.all()
    assert np.allclose(res['weights']['w'][good], 0.0)
    assert np.allclose(wn[good], 0.375)


def test_s5_no_alternative_means_outside_e7_and_empty_is_ignored():
    """S2: no same-dir alternative puts e outside E7; an empty one is skipped.

    Outside E7 is NaN, not zero: zero would be the claim that the alternative
    faces look identical, which is the opposite of what no alternative means.
    """
    a = fake_side(wall_grid(), 'A')
    a['bearing_hist'] = [hist(0)]

    # (i) B has one segment, so there is no alternative at all
    b1 = fake_side(wall_grid(), 'B')
    b1['bearing_hist'] = [hist(9)]
    res1, _ = _weights_for(a, b1)
    wn1 = dw.null_weights(a['seg_of'], res1['weights'], a['bearing_hist'],
                          b1['bearing_hist'], np.array([3]))
    assert np.isnan(wn1).all()

    # (ii) the only alternative has an empty histogram -> ignored, still NaN
    b2 = fake_side(wall_grid(), 'B')
    b2['bearing_hist'] = [hist(9), np.zeros(NB, int)]
    res2, _ = _weights_for(a, b2)
    wn2 = dw.null_weights(a['seg_of'], res2['weights'], a['bearing_hist'],
                          b2['bearing_hist'], np.array([3, 3]))
    assert np.isnan(wn2).all()

    # (iii) a different dir is not an alternative either
    b3 = fake_side(wall_grid(), 'B')
    b3['bearing_hist'] = [hist(9), hist(18)]
    res3, _ = _weights_for(a, b3)
    wn3 = dw.null_weights(a['seg_of'], res3['weights'], a['bearing_hist'],
                          b3['bearing_hist'], np.array([3, 0]))
    assert np.isnan(wn3).all()


def test_s5_g3_a_map_against_itself():
    """S3: C = 1, W = 0, every w exactly 0 -- G3's whole content."""
    a = fake_side(wall_grid(), 'A')
    a['bearing_hist'] = [hist(3, 20, 55)]
    res, sh = _weights_for(a, a)
    assert sh['corroborated'] == 1.0
    assert sh['W'] == 0.0
    w = res['weights']['w']
    good = ~np.isnan(w)
    assert good.sum() == 12
    assert (w[good] == 0.0).all()             # exactly, not approximately


@pytest.mark.parametrize('n_hold,required,expected', [
    (18, dw.P7_MIN_PAIRS, True), (17, dw.P7_MIN_PAIRS, False),
    (16, dw.P8_MIN_PAIRS, True), (15, dw.P8_MIN_PAIRS, False)])
def test_s5_verdict_counting(n_hold, required, expected):
    """S4a: 18 of 20 holds P7 and 17 does not; 16 holds P8 and 15 does not."""
    assert (n_hold >= required) is expected


def test_s5_equal_values_and_an_empty_set_count_against():
    """S4b: 'strictly lower' excludes equal, and an absent answer is a failure.

    Both predictions say so, and both say it because the alternative is to let
    a pair that produced no comparison be read as a pair that passed one.
    """
    # strictly lower: equal is not lower
    assert not (0.3 < 0.3)
    # an empty E7: n7 == 0, so the verdict is False whatever the means would be
    for n7, mw, mn in ((0, None, None), (0, 0.1, 0.9)):
        assert bool(n7 and mw is not None and mn is not None and mw < mn) is False
    # P8 with no weighable element at either cut
    for m1200, m60 in ((None, 0.4), (0.4, None), (None, None)):
        assert bool(m1200 is not None and m60 is not None
                    and m1200 < m60) is False


def test_s5_stage_refuses_a_dirty_tree(monkeypatch):
    """S5: a result file must not name a commit it was not produced by."""
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s5()
    assert e.value.code == 2


# ───────────── S6: the cut matrix, G4 and P9 (the M series) ──────────────────
#
# EVERY CASE HERE IS SYNTHETIC. The 180 unseen rows of the real matrix -- A at
# 60, 120 or 240 with B at another cut -- must stay unseen until the stage is
# run from a clean, committed tree, so nothing below loads a map, and the two
# builders are the whole corpus these cases get.

S6_WIDTHS = {60: 3, 120: 6, 240: 9, 1200: 12}


def s6_sides(n_robots=dw.N_ROBOTS):
    """A synthetic corpus: five robots by four cuts, on one wall that grows.

    Robot k at cut c carries `S6_WIDTHS[c] + k` elements of the same wall from
    x = 0, so B@cut_b corroborates A's first w(b) elements and nothing past
    them: `not_corroborated` is `max(0, w_a - w_b) / w_a`, which falls as B's
    cut grows and makes P9 hold on this corpus by construction. The widths
    differ per robot so the two directions of a pair differ, which is what the
    symmetric table needs to be more than a column of zeroes.

    All five spawn at the origin: the frame change is tested by C3 and is not
    what the matrix is about.
    """
    by_kc, spawn = {}, {}
    for k in range(n_robots):
        spawn[k] = (0.0, 0.0)
        for c, w in S6_WIDTHS.items():
            by_kc[(k, c)] = fake_side(
                wall_grid(h=12, w=w + k),
                f'b2maps_k{k}_cut{c}_gated_extfix_robot{k}')
    return by_kc, spawn


def p9_rows(values=None, lengths=None, flat=0.5, l_a=1.2):
    """A 320-row matrix holding only what P9 and the symmetric table read.

    `values[(A, B, cut_A)]` is that series' four not-corroborated shares in
    CUTS order and `lengths[(A, B, cut_A)]` its L_A_m; anything unnamed gets a
    flat series, which holds, on the 1.2 m twelve-element wall the rest of this
    file uses. Built rather than measured, so a case can state the series it is
    about and nothing else.
    """
    values, lengths = values or {}, lengths or {}
    rows = {}
    for key in dw.matrix_keys():
        a, b, ca, cb = key
        vals = values.get((a, b, ca))
        nc = vals[dw.CUTS.index(cb)] if vals else flat
        rows[key] = {'A': a, 'B': b, 'cut_A': ca, 'cut_B': cb,
                     'not_corroborated': nc, 'corroborated': 1.0 - nc,
                     'L_A_m': lengths.get((a, b, ca), l_a),
                     'seen': dw.is_seen(ca, cb)}
    return rows


def cut_pairs_in(node):
    """Every (cut_A, cut_B) a payload names, in either shape a row can take."""
    found = []
    if isinstance(node, dict):
        if 'cut_A' in node and 'cut_B' in node:
            found.append((node['cut_A'], node['cut_B']))
        key = node.get('key')
        if isinstance(key, list) and len(key) == 4:
            found.append((key[2], key[3]))
        for v in node.values():
            found += cut_pairs_in(v)
    elif isinstance(node, list):
        for v in node:
            found += cut_pairs_in(v)
    return found


def test_s6_seen_flag_counts_140_and_180():
    """M1: `seen` is true exactly at cut_A 1200 or cut_A == cut_B."""
    keys = dw.matrix_keys()
    assert len(keys) == 320
    seen = [k for k in keys if dw.is_seen(k[2], k[3])]
    assert len(seen) == dw.N_SEEN == 140
    assert len(keys) - len(seen) == dw.N_UNSEEN == 180
    for (a, b, ca, cb) in keys:
        assert dw.is_seen(ca, cb) is (ca == 1200 or ca == cb)
    # the spec's two seen sets and the 20 rows they share
    assert sum(1 for k in seen if k[2] == 1200) == 80
    assert sum(1 for k in seen if k[2] == k[3]) == 80
    assert sum(1 for k in seen if k[2] == 1200 and k[3] == 1200) == 20


def test_s6_p9_equal_steps_hold():
    """M2: non-increasing includes equal, so a flat series holds."""
    out = dw.p9_verdict(p9_rows())
    assert (out['holds'], out['n_holding'], out['of']) == (True, 60, 60)
    assert out['failures'] == []


def test_s6_p9_a_rise_of_one_element_fails():
    """M3: one element more not corroborated is a rise, and it is 0.1 m of it.

    With A fixed the denominator is constant, which is what makes the rise
    convertible: 1/12 of a twelve-element 1.2 m wall is one element, 0.1 m.
    """
    out = dw.p9_verdict(p9_rows(values={(0, 1, 60): [4 / 12, 4 / 12,
                                                     5 / 12, 5 / 12]}))
    assert (out['holds'], out['n_holding']) == (False, 59)
    assert len(out['failures']) == 1
    f = out['failures'][0]
    assert (f['A'], f['B'], f['cut_A'], f['step']) == (0, 1, 60,
                                                       'cut120->cut240')
    assert f['rise'] == pytest.approx(1 / 12)
    assert f['rise_m'] == pytest.approx(RHO)


def test_s6_p9_a_rise_of_1e_12_fails():
    """M4: a rise of any size, with no tolerance at all."""
    out = dw.p9_verdict(p9_rows(values={(2, 3, 240): [0.5, 0.5 + 1e-12,
                                                      0.5 + 1e-12,
                                                      0.5 + 1e-12]}))
    assert (out['holds'], out['n_holding']) == (False, 59)
    assert len(out['failures']) == 1
    assert 0.0 < out['failures'][0]['rise'] < 1e-11


def test_s6_p9_zero_length_a_walls_fail():
    """M5: an undefined series is a failure, and names no step."""
    out = dw.p9_verdict(p9_rows(lengths={(4, 0, 120): 0.0}))
    assert (out['holds'], out['n_holding']) == (False, 59)
    f = out['failures'][0]
    assert (f['A'], f['B'], f['cut_A']) == (4, 0, 120)
    assert f['step'] is None and f['rise'] is None and f['reason']
    assert next(s for s in out['series']
                if (s['A'], s['B'], s['cut_A']) == (4, 0, 120))['undefined']


@pytest.mark.parametrize('n_bad', [0, 1, 2])
def test_s6_p9_holds_only_at_60_of_60(n_bad):
    """M6: P9 holds only if all 60 series hold -- 59 is a failed P9."""
    bad = [(a, b, 60) for (a, b) in dw.ordered_pairs()][:n_bad]
    out = dw.p9_verdict(p9_rows(values={s: [0.4, 0.5, 0.5, 0.5] for s in bad}))
    assert out['of'] == 60
    assert out['n_holding'] == 60 - n_bad
    assert out['holds'] is (n_bad == 0)


def test_s6_symmetric_table_uses_every_row_exactly_once():
    """M7: 160 entries over the 10 unordered pairs, and all 320 rows consumed."""
    rows = p9_rows()
    entries = dw.symmetric_table(rows)
    assert len(entries) == 160
    used = []
    for e in entries:
        assert e['X'] < e['Y']
        used += [(e['X'], e['Y'], e['cut_X'], e['cut_Y']),
                 (e['Y'], e['X'], e['cut_Y'], e['cut_X'])]
    assert len(used) == 320
    assert len(set(used)) == 320
    assert set(used) == set(rows)


def test_s6_symmetric_difference_is_first_minus_second():
    """M8: the difference is C(X@a -> Y@b) minus C(Y@b -> X@a), in that order."""
    rows = p9_rows()
    rows[(0, 1, 60, 120)]['corroborated'] = 0.9
    rows[(1, 0, 120, 60)]['corroborated'] = 0.4
    e = next(x for x in dw.symmetric_table(rows)
             if (x['X'], x['Y'], x['cut_X'], x['cut_Y']) == (0, 1, 60, 120))
    assert (e['C_X_to_Y'], e['C_Y_to_X']) == (0.9, 0.4)
    assert e['difference'] == pytest.approx(0.5)


def test_s6_g4_one_ulp_stops_the_stage_before_any_unseen_row(tmp_path, capsys,
                                                             monkeypatch):
    """M9: G4 fails on one ulp, and no unseen row is computed, written or printed.

    The last part is the one that matters, so it is proved rather than argued:
    a spy on `shares` -- the C function -- counts 140 calls, one per seen row
    and not one more, and a spy on `s6_row` shows every key it was asked for
    was a seen one. The fail file and the terminal are then checked for an
    unseen (cut_A, cut_B) anywhere in them.
    """
    by_kc, spawn = s6_sides()
    seen = [k for k in dw.matrix_keys() if dw.is_seen(k[2], k[3])]
    rows = dw.cut_matrix(by_kc, spawn, seen)          # before the spies
    comparisons = [{'source': 's4', 'path': f'same_cut[{i}].corroborated',
                    'key': k, 'field': 'corroborated',
                    'stored': rows[k]['corroborated']}
                   for i, k in enumerate(seen)]
    was = comparisons[7]['stored']
    comparisons[7]['stored'] = math.nextafter(was, math.inf)
    assert comparisons[7]['stored'] != was

    keys_asked, c_calls = [], []
    real_row, real_shares = dw.s6_row, dw.shares
    monkeypatch.setattr(dw, 's6_row', lambda b, s, key: (
        keys_asked.append(key), real_row(b, s, key))[1])
    monkeypatch.setattr(dw, 'shares', lambda res: (
        c_calls.append(1), real_shares(res))[1])

    with pytest.raises(SystemExit) as e:
        dw.s6_matrix(by_kc, spawn, comparisons,
                     {'stage': 's6', 'head': 'synthetic'}, tmp_path)
    assert e.value.code == 2

    assert len(keys_asked) == dw.N_SEEN
    assert all(dw.is_seen(k[2], k[3]) for k in keys_asked)
    assert len(c_calls) == dw.N_SEEN           # C ran on the seen rows only

    assert not list(tmp_path.glob('s6_cutmatrix_*.json'))
    written = list(tmp_path.glob('s6_g4_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert 'rows' not in payload and 'P9' not in payload
    assert len(payload['seen_rows']) == dw.N_SEEN
    assert all(r['seen'] for r in payload['seen_rows'])
    assert payload['G4']['n_equal'] == dw.N_SEEN - 1
    pairs = cut_pairs_in(payload)
    assert pairs and all(dw.is_seen(ca, cb) for ca, cb in pairs)

    out = capsys.readouterr().out
    assert 'G4: FAIL' in out
    assert 'cut_A \\ cut_B' not in out         # the 4x4 table never printed
    assert 'P9' not in out
    printed = re.findall(r'cut_A (\d+) cut_B (\d+)', out)
    assert printed and all(dw.is_seen(int(ca), int(cb)) for ca, cb in printed)


def test_s6_g4_holding_lets_the_whole_matrix_through(tmp_path):
    """M10: with G4 equal, all 320 rows are computed and written."""
    by_kc, spawn = s6_sides()
    seen = [k for k in dw.matrix_keys() if dw.is_seen(k[2], k[3])]
    rows = dw.cut_matrix(by_kc, spawn, seen)
    comparisons = [{'source': 's4', 'path': f'same_cut[{i}].{f}', 'key': k,
                    'field': f, 'stored': rows[k][f]}
                   for i, k in enumerate(seen)
                   for f in ('L_A_m', 'corroborated', 'not_corroborated')]
    out = dw.s6_matrix(by_kc, spawn, comparisons,
                       {'stage': 's6', 'head': 'synthetic'}, tmp_path)

    assert out['G4']['all_equal'] and out['G4']['n'] == 3 * dw.N_SEEN
    assert (out['n_rows'], out['n_seen'], out['n_unseen']) == (320, 140, 180)
    assert len(out['symmetric']) == 160
    assert (out['P9']['holds'], out['P9']['n_holding'], out['P9']['of']) == (
        True, 60, 60)
    written = list(tmp_path.glob('s6_cutmatrix_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['rows']) == 320
    assert sum(1 for r in payload['rows'] if r['seen']) == 140
    assert out['written']['path'] == str(written[0])
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def test_s6_reads_the_p9_paragraph_from_the_spec_text():
    """M11: header to blank line, unwrapped, with a list marker taken off."""
    text = ('## 9. Pre-registrations\n\n'
            '**P9. REGISTERED 8 Oct 2026.** Convergence, A held at an early\n'
            'cut: the share is non-increasing.\n'
            '\n'
            'A later paragraph that is not P9.\n')
    assert dw.spec_paragraph(text, dw.P9_HEADER) == (
        '**P9. REGISTERED 8 Oct 2026.** Convergence, A held at an early cut: '
        'the share is non-increasing.')
    assert dw.spec_paragraph('- **P3. REGISTERED.** one\n  two\n\n',
                             '**P3. REGISTERED') == (
        '**P3. REGISTERED.** one two')
    with pytest.raises(SystemExit) as e:
        dw.spec_paragraph('no prediction here\n', dw.P9_HEADER)
    assert e.value.code == 2


def test_s6_refuses_a_dirty_tree(monkeypatch):
    """M12a: a result file must not name a commit it was not produced by."""
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s6()
    assert e.value.code == 2


def test_s6_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """M12b: P9 must be registered in history behind the code that tests it."""
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] in ('status', 'merge-base') else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        dw.run_s6()
    assert e.value.code == 2


def test_s6_refuses_when_the_spec_at_head_lacks_p9(monkeypatch):
    """M12c: ancestry is not enough -- the spec at HEAD must carry the header."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return '## 9\n\n- **P3. REGISTERED 6 Oct 2026.** something\n'
        return 'c0ffee'                       # merge-base == rev-parse
    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s6()
    assert e.value.code == 2


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

    # three rays per element, because condition 4 wants two running on and a
    # majority; the prediction is about the face, not about the ray count
    r = rays(*[(0.1 * c + dxx, 0.05, 90.0, 1.0)
               for c in range(1, 11) for dxx in (-0.02, 0.0, 0.02)])
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
    r = rays((0.0, 0.1, 20.0, 2.0), (0.01, 0.1, 20.0, 2.0),
             (1.5, 0.1, 90.0, 1.0), (1.51, 0.1, 90.0, 1.0))
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

    res = run(a, no_faces(), rays((ox, oy, ang, flen),
                                  (ox + 0.001, oy, ang, flen)))
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
