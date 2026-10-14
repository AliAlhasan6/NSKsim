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

The L series is S7, gains and losses. Its rows were written before the stage was
run, and nothing in them touches a real map.

  L1   an element corroborated at both cuts                 neither a gain nor
                                                             a loss
  L2   the direction convention                             not-then-yes is a
                                                             gain, yes-then-not
                                                             a loss, 0.1 m each
  L3   gains minus losses                                   equals the change in
                                                             the corroborated
                                                             count, every case
  L4   an unassigned element that changes state             counted as neither
  L5   P10 at 0, 9, 10 and 20 of P3's 20 series             holds only from 10;
                                                             P9's 60 never count
  L6   G5, one ulp on one stored S6 share                   exit 2; the share
                                                             comparisons only,
                                                             and no gain, loss
                                                             or distance at all
  L7   G5, one element on one failing step's rise           exit 2; no reported
                                                             gain, loss or
                                                             distance, and the
                                                             only counting is
                                                             G5's own, on S6's
                                                             failing steps
  L8   G5 holding on a synthetic corpus                     80 series, 320 rows,
                                                             P10 20/20, a file
  L9   a reordered A element list                           the alignment
                                                             assertion fires
  L10  the nearest same-direction B element                 across first, then
                                                             along, then the
                                                             lowest index; None
                                                             when B faces no
                                                             such way
  L11  the split that exposes the mask                      S6's row is the
                                                             split plus shares,
                                                             and c/n off the
                                                             mask is the share
  L12  --stage s7 on a dirty tree, without 9541ea1, with    refuses, exit 2, on
       a spec lacking P10, and on a wrong S6 sha256          each of the four
  L13  an unassigned element flipping at every step          counted nowhere,
                                                             against the
                                                             SHARE's own count,
                                                             and no lost-element
                                                             record

The C series is S8, the cell states under losses. Written before the stage ran,
and no real grid is read at a lost element's cells anywhere in this file.

  C1   the two cell centres, all four directions             always the
                                                             occupied cell and
                                                             its free neighbour
  C2   a whole slab, sub-cell origin                         every element lands
                                                             wall OCC, front
                                                             FREE
  C3   the lookup, off-lattice origin and bottom-up rows     the containing
                                                             cell, anywhere
                                                             inside it
  C4   both sides of all four edges                          lower inclusive,
                                                             upper exclusive;
                                                             off grid gives None
                                                             and never UNKNOWN
  C5   the four classes and their order                      off grid first and
                                                             it wins; freed to
                                                             free and to
                                                             unknown; front
                                                             closed; unchanged
  C6   P11 at 0, 105, 106, 107 and 213 of 213                holds only from
                                                             107; off grid stays
                                                             in the denominator
  C7   the lattice offset in cells                           [-0.5, 0.5); a
                                                             whole cell reads 0
  C8   G6 holding, end to end                                freed (both
                                                             states), front
                                                             closed and off grid
                                                             all reached; a file
  C9   a lost set short by one                               exit 2, and NO grid
                                                             opened at all
  C10  a wrong wall state at k                               exit 2; two reads
                                                             per element, each
                                                             on its own k map,
                                                             and no k' read
  C11  --stage s8 on a dirty tree, without df7ae16, with     refuses, exit 2, on
       a spec lacking P11, and on a wrong S7 sha256           each of the four

The Q series is S9, segmentation or geometry. Q, not P, because P3 to P12 are
the predictions. Written before the stage ran; no segment assignment and no e'
distance is read on a real map anywhere in this file.

  Q1   e' by (row, col, dir), all four directions            the element of
                                                             that cell pair
  Q2   a sub-cell lattice offset at k', walked across a      e' found at every
       whole cell                                             offset
  Q3   a cell with no element, and the wrong direction       None for each
  Q4   seg_of against the full array and the subset          the two indexings
                                                             differ; only the
                                                             full read sees an
                                                             unassigned e'
  Q5   the three classes and their order                     segmentation
                                                             first, then
                                                             across, then along
  Q6   the boundaries at eps and rho/2                       inclusive to 1e-9,
                                                             T1's own, through
                                                             one predicate
  Q7   P12 at 0, 38, 39 and 78 of 78                         holds only from
                                                             39; the 54 seen
                                                             stay out of both
                                                             numerator and
                                                             denominator
  Q8   G7 holding, end to end                                56 unchanged, all
                                                             segmentation, both
                                                             tested bins, a file
  Q9   an unchanged set short by one                         part 1 fails
  Q10  a cell state S9 reads differently from S8             part 1 fails --
                                                             stricter than the
                                                             spec
  Q11  no e', and an e' in the wrong direction               part 2 fails
  Q12  an e' assigned and inside T1                          part 3 fails
  Q13  a carve-out element whose e' is assigned              part 4 fails --
                                                             stricter than the
                                                             spec
  Q14  --stage s9 on a dirty tree, without 4698880, with     refuses, exit 2,
       a spec lacking P12, and on a wrong S8 sha256           on each of the
                                                             four
  Q15  the nearest_same_dir docstring                        names the caller's
                                                             set; behaviour
                                                             unchanged

The R series is S10, the segments behind the unassigned elements. Written
before the stage ran; no cell of a member of S_b is read on a real map
anywhere in this file, and no changed, intact or flanked is computed on one.

  R1   S_b's members at k', every route and intact            one route per
                                                              member, in P11's
                                                              order; two routes
                                                              at once when two
                                                              members differ
  R2   a member whose cells left the map                      off grid, and
                                                              NEVER wall cell
                                                              freed
  R3   the lookup with trinary_map.sample made to raise       never called
  R4   an unassigned element between two members of one       flanked
       segment, on its face line
  R5   two segments either side; past a segment's end;        not flanked in
       off the face line by 2e-9 m; and at exactly 1e-9       each; 1e-9 is
                                                              inclusive
  R7   one event's rows, once per A pairing                   one event, one
                                                              unit, rows kept
  R8   an S_b behind losses of both shapes                    in both
                                                              denominators;
                                                              per-loss counting
                                                              flips P13b, so the
                                                              dedupe is checked
  R9   ceil(n/2) at odd and even n, and in a verdict          the larger half;
                                                              3 needs 2, 4
                                                              needs 2; P14 is
                                                              0 or nothing
  R10  the base rate on a toy map of two segments             1 of 2, share 0.5
  R11  G8 holding, end to end                                 108 losses, 27
                                                              events, 2 units:
                                                              the hole one
                                                              intact, the
                                                              carve-out one
                                                              changed; P13a,
                                                              P13b, P14 all
                                                              hold; a file
  R12  a failure at each of G8's three checks                 exit 2; segment_
                                                              change and
                                                              is_flanked called
                                                              0 times; no later
                                                              check run
  R13  --stage s10 on a dirty tree, without e15c94b, with     refuses, exit 2,
       a spec missing any of the three registrations, and     and loads nothing
       on a wrong S9 sha256                                   in each
  R14  S9's part-1 recompute, after the extraction            S9 calls it; its
                                                              statements are
                                                              gone from S9

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

import collections
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


# ────────── S7: gains and losses, G5 and P10 (the L series) ─────────────────
#
# SYNTHETIC ONLY, like the M series. No element mask is computed on a real map
# anywhere in this file.

# One occupied run per cut, placed rather than merely grown: the run SHIFTS
# right by one column at cut240, so A's column 0 is corroborated by B at cut120
# and not by B at cut240. That is a loss, and it is what gives this corpus
# failing P9 steps for G5's second check to compare and losses for P10 to count.
S7_SPAN = {60: (0, 2), 120: (0, 5), 240: (1, 9), 1200: (0, 12)}
S7_GRID_W = 18


def wall_span(lo, hi, w_total=S7_GRID_W, h=12, wall_row=6):
    """One occupied run from column `lo` to `hi` inclusive, FREE above it.

    Exactly hi-lo+1 elements, all the same dir, at x = col*rho -- the shape
    `wall_grid` gives, but placed. The cells beside the run stay UNKNOWN, so
    the run's ends yield no element of their own (Def 1: an occupied-unknown
    boundary is not a face), and A's and B's elements line up column for
    column.
    """
    g = blank(h, w_total)
    g[:wall_row, :] = FREE
    g[wall_row, lo:hi + 1] = OCC
    return g


def s7_sides(n_robots=dw.N_ROBOTS):
    """The synthetic corpus S7's cases run on: five robots by four cuts."""
    by_kc, spawn = {}, {}
    for k in range(n_robots):
        spawn[k] = (0.0, 0.0)
        for c, (lo, hi) in S7_SPAN.items():
            by_kc[(k, c)] = fake_side(
                wall_span(lo, hi + k),
                f'b2maps_k{k}_cut{c}_gated_extfix_robot{k}')
    return by_kc, spawn


def s7_fake_s6(by_kc, spawn):
    """An S6 payload for the synthetic corpus: the 320 rows and P9's failures.

    Built with S6's own code, so S7's G5 agrees with it by construction and a
    case can then break exactly one number and watch the stage stop.
    """
    keys = dw.matrix_keys()
    rows = dw.cut_matrix(by_kc, spawn, keys)
    return {'rows': [rows[k] for k in keys], 'P9': dw.p9_verdict(rows)}


def gl_masks(ck, ckp, assigned=None, rho=RHO):
    """Two one-row masks carrying only what `gain_loss_step` reads."""
    ck, ckp = np.array(ck, bool), np.array(ckp, bool)
    keep = (np.ones(ck.size, bool) if assigned is None
            else np.array(assigned, bool))
    return ({'key': (0, 1, 1200, 60), 'corr': ck, 'assigned': keep,
             'rho': rho, 'n_corroborated': int((ck & keep).sum())},
            {'key': (0, 1, 1200, 120), 'corr': ckp, 'assigned': keep,
             'rho': rho, 'n_corroborated': int((ckp & keep).sum())})


def b_at(el_a, offsets):
    """B elements placed by hand at (across, along) from A's element 0.

    Across is along A's own normal and along is along its tangent, which is
    how `nearest_same_dir` measures, so a case can state the answer it wants.
    """
    nx, ny = float(el_a['nx'][0]), float(el_a['ny'][0])
    tx, ty = -ny, nx
    xs = [float(el_a['x'][0]) + ac * nx + al * tx for ac, al in offsets]
    ys = [float(el_a['y'][0]) + ac * ny + al * ty for ac, al in offsets]
    n = len(offsets)
    return {'n': n, 'x': np.array(xs), 'y': np.array(ys),
            'nx': np.full(n, nx), 'ny': np.full(n, ny),
            'dir': np.full(n, int(el_a['dir'][0]), dtype=np.int64)}


def banned_words_in(node, path='$'):
    """Every path in a payload whose key names a gain, loss or distance."""
    bad, words = [], ('gain', 'loss', 'lost', 'across_m', 'along_m', 'dist_m')
    if isinstance(node, dict):
        for k, v in node.items():
            if any(w in str(k).lower() for w in words):
                bad.append(f'{path}.{k}')
            bad += banned_words_in(v, f'{path}.{k}')
    elif isinstance(node, list):
        for v in node:
            bad += banned_words_in(v, f'{path}[]')
    return bad


def test_s7_corroborated_at_both_cuts_is_neither():
    """L1: a state that does not change is not a gain and not a loss."""
    gl = dw.gain_loss_step(*gl_masks([1, 1, 1], [1, 1, 1]))
    assert (gl['n_gained'], gl['n_lost']) == (0, 0)
    gl = dw.gain_loss_step(*gl_masks([0, 0, 0], [0, 0, 0]))
    assert (gl['n_gained'], gl['n_lost']) == (0, 0)


def test_s7_the_gain_and_loss_direction_convention():
    """L2: not-then-yes is a gain; yes-then-not is a loss. Never the reverse."""
    gl = dw.gain_loss_step(*gl_masks([0, 1], [1, 0]))
    assert (gl['n_gained'], gl['n_lost']) == (1, 1)
    assert list(gl['gained_idx']) == [0]
    assert list(gl['lost_idx']) == [1]
    assert gl['gained_m'] == pytest.approx(RHO)
    assert gl['lost_m'] == pytest.approx(RHO)


@pytest.mark.parametrize('ck,ckp', [
    ([0, 0, 1, 1], [1, 1, 1, 0]), ([1, 1, 1, 1], [0, 0, 0, 0]),
    ([0, 0, 0, 0], [1, 1, 1, 1]), ([1, 0, 1, 0], [1, 0, 1, 0])])
def test_s7_gains_minus_losses_is_the_change_in_count(ck, ckp):
    """L3: every element that changed state is a gain or a loss, so the net is
    the change in the corroborated count. The identity S7 asserts at run time."""
    mk, mkp = gl_masks(ck, ckp)
    gl = dw.gain_loss_step(mk, mkp)
    assert (gl['n_gained'] - gl['n_lost']
            == mkp['n_corroborated'] - mk['n_corroborated'])


def test_s7_an_unassigned_element_is_neither():
    """L4: §3 leaves unassigned elements as excluded length, out of every share."""
    gl = dw.gain_loss_step(*gl_masks([0, 0], [1, 1], assigned=[True, False]))
    assert (gl['n_gained'], gl['n_lost']) == (1, 0)
    gl = dw.gain_loss_step(*gl_masks([1, 1], [0, 0], assigned=[True, False]))
    assert (gl['n_gained'], gl['n_lost']) == (0, 1)


def unassigned_side(grid, unassigned, stem):
    """A side with chosen elements left off a face, so `assigned` is not all True.

    `fake_side` assigns every element to face 0, which is what the M and L
    corpora want and also why neither of them ever exercises §3's excluded
    length. This is the fixture that does.
    """
    s = side(grid)
    seg_of = s['seg_of'].copy()
    seg_of[list(unassigned)] = -1
    return {'stem': stem, 'el': s['el'], 'seg_of': seg_of, 'grid': grid,
            'rho': s['rho'], 'origin': s['origin'], 'shape': s['shape'],
            'segments': [{'p0': [0.0, 0.55], 'p1': [1.1, 0.55]}]}


def test_s7_an_unassigned_element_flipping_never_counts():
    """L13: an unassigned element changes state at every step and counts nowhere.

    A carries six elements with the last left off a face. B's run covers all
    six at cut120 and cut1200 and only the first five at cut60 and cut240, so
    A's element 5 flips at all three steps -- a gain, then a loss, then a gain
    if anything counted it. Nothing may: §3 leaves it as excluded length, so
    `shares` never sees it and neither may a gain or a loss.

    This is the case that ties the restriction to the SHARE's own count rather
    than to a hand-built one: `change_in_corroborated` here comes from
    `shares`, through `s7_mask`. Drop the `keep &` in `gain_loss_step` and the
    run-time identity in `gain_loss_series` fires, because gains minus losses
    would be -1 or +1 against a change of 0.
    """
    by_kc, spawn = {}, {0: (0.0, 0.0), 1: (0.0, 0.0)}
    by_kc[(0, 1200)] = unassigned_side(wall_span(0, 5), [5], 'A_unassigned_5')
    covers = {60: 4, 120: 5, 240: 4, 1200: 5}
    for c, hi in covers.items():
        by_kc[(1, c)] = fake_side(wall_span(0, hi), f'B_cut{c}_to_{hi}')

    keys = [(0, 1, 1200, cb) for cb in dw.CUTS]
    masks = {k: dw.s7_mask(by_kc, spawn, k) for k in keys}

    # five assigned elements, all five corroborated at every cut; the sixth is
    # excluded length and is not in the denominator at all
    for k in keys:
        assert masks[k]['n_elements'] == 5
        assert masks[k]['n_corroborated'] == 5
        assert masks[k]['corroborated'] == 1.0
        assert masks[k]['assigned'].sum() == 5
        assert masks[k]['el_a']['n'] == 6
    # and it really does flip: corroborated at cut120 and cut1200, not at the others
    assert [bool(masks[(0, 1, 1200, cb)]['corr'][5]) for cb in dw.CUTS] == [
        False, True, False, True]

    s = dw.gain_loss_series(masks, 0, 1, 1200)
    assert s['n_elements'] == 5
    assert s['n_corroborated_by_B_cut'] == [5, 5, 5, 5]
    for st in s['steps']:
        assert (st['n_gained'], st['n_lost']) == (0, 0)
        assert st['change_in_corroborated'] == 0
        assert (st['gained_m'], st['lost_m']) == (0.0, 0.0)
    assert s['any_loss'] is False
    assert (s['total_gained'], s['total_lost']) == (0, 0)

    # and no lost-element record is written for it
    assert dw.lost_elements(masks, [s]) == []


@pytest.mark.parametrize('n_with_loss,expected', [
    (0, False), (9, False), (10, True), (20, True)])
def test_s7_p10_threshold_is_10_of_p3s_20(n_with_loss, expected):
    """L5: 9 of 20 fails and 10 of 20 holds, and P9's 60 series never count."""
    series = []
    for i, (a, b) in enumerate(dw.ordered_pairs()):
        n = 1 if i < n_with_loss else 0
        series.append({'A': a, 'B': b, 'cut_A': 1200, 'any_loss': bool(n),
                       'total_lost': n, 'steps': [{'n_lost': n, 'n_gained': 0}]})
        for ca in (60, 120, 240):          # P9's, all with losses, all ignored
            series.append({'A': a, 'B': b, 'cut_A': ca, 'any_loss': True,
                           'total_lost': 9,
                           'steps': [{'n_lost': 9, 'n_gained': 0}]})
    out = dw.p10_verdict(series)
    assert (out['of'], out['min_required']) == (20, 10)
    assert out['n_with_loss'] == n_with_loss
    assert out['holds'] is expected


def _s7_spies(monkeypatch):
    """Spies on the reported sweep, the distances, and the step counting."""
    seen = {'series': 0, 'near': 0, 'steps': []}
    real_series = dw.gain_loss_series
    real_near = dw.nearest_same_dir
    real_step = dw.gain_loss_step

    def series(*a, **k):
        seen['series'] += 1
        return real_series(*a, **k)

    def near(*a, **k):
        seen['near'] += 1
        return real_near(*a, **k)

    def step(mk, mkp):
        seen['steps'].append((tuple(mk['key']), tuple(mkp['key'])))
        return real_step(mk, mkp)

    monkeypatch.setattr(dw, 'gain_loss_series', series)
    monkeypatch.setattr(dw, 'nearest_same_dir', near)
    monkeypatch.setattr(dw, 'gain_loss_step', step)
    return seen


def _assert_g5_fail_file(tmp_path, capsys):
    """The fail file and the terminal carry no gain, loss or distance."""
    assert not list(tmp_path.glob('s7_gainloss_*.json'))
    written = list(tmp_path.glob('s7_g5_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert banned_words_in(payload) == []
    assert 'series' not in payload and 'P10' not in payload
    out = capsys.readouterr().out
    for word in ('lost elements', 'P10', 'gains', 'losses', 'across'):
        assert word not in out
    return payload


def test_s7_g5_one_ulp_share_stops_before_any_counting(tmp_path, capsys,
                                                       monkeypatch):
    """L6: one ulp on one of S6's 320 shares, and nothing is ever counted.

    The share check runs first, so on this failure not even G5's own step
    check gets to call the counter: the spy sees no call at all.
    """
    by_kc, spawn = s7_sides()
    s6 = s7_fake_s6(by_kc, spawn)
    was = s6['rows'][11]['corroborated']
    s6['rows'][11]['corroborated'] = math.nextafter(was, math.inf)
    assert s6['rows'][11]['corroborated'] != was

    seen = _s7_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        dw.s7_gainloss(by_kc, spawn, s6,
                       {'stage': 's7', 'head': 'synthetic'}, tmp_path)
    assert e.value.code == 2
    assert (seen['series'], seen['near'], seen['steps']) == (0, 0, [])
    payload = _assert_g5_fail_file(tmp_path, capsys)
    assert payload['G5']['steps'] is None
    assert payload['G5']['shares']['n'] == 320
    assert len(payload['G5']['shares']['differing']) == 1


def test_s7_g5_one_element_on_a_rise_stops_before_the_sweep(tmp_path, capsys,
                                                            monkeypatch):
    """L7: one element on one failing step's rise stops the stage.

    G5's own second check cannot reach its verdict without counting on S6's
    failing steps -- that is what it checks -- so the claim proved here is the
    one that matters: the REPORTED sweep never runs and no distance is ever
    measured, and the only counting that happened was G5's own, bounded to
    S6's failing steps.
    """
    by_kc, spawn = s7_sides()
    s6 = s7_fake_s6(by_kc, spawn)
    fails = s6['P9']['failures']
    assert fails, 'the synthetic corpus must have failing P9 steps'
    s6['P9']['failures'][0]['rise_m'] += RHO        # one element out

    seen = _s7_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        dw.s7_gainloss(by_kc, spawn, s6,
                       {'stage': 's7', 'head': 'synthetic'}, tmp_path)
    assert e.value.code == 2
    assert (seen['series'], seen['near']) == (0, 0)
    assert len(seen['steps']) == len(fails)
    s6_steps = {((f['A'], f['B'], f['cut_A'], f['from_cut_B']),
                 (f['A'], f['B'], f['cut_A'], f['to_cut_B'])) for f in fails}
    assert set(seen['steps']) == s6_steps

    payload = _assert_g5_fail_file(tmp_path, capsys)
    assert payload['G5']['shares']['all_equal']
    assert len(payload['G5']['steps']['differing']) == 1


def test_s7_g5_holding_lets_the_whole_sweep_through(tmp_path):
    """L8: with G5 equal, all 80 series are counted and written."""
    by_kc, spawn = s7_sides()
    s6 = s7_fake_s6(by_kc, spawn)
    out = dw.s7_gainloss(by_kc, spawn, s6,
                         {'stage': 's7', 'head': 'synthetic'}, tmp_path)

    assert out['G5']['all_equal']
    assert out['G5']['shares']['n'] == 320
    assert out['G5']['steps']['n'] == len(s6['P9']['failures'])
    assert out['n_series'] == 80 == dw.N_SERIES
    assert (out['P10']['of'], out['P10']['min_required']) == (20, 10)
    # the corpus shifts B's run right at cut240, so A's column 0 is lost at the
    # 120 -> 240 step of every series
    assert out['P10']['n_with_loss'] == 20
    assert out['P10']['holds']
    assert out['lost_elements']
    for s in out['series']:
        assert len(s['steps']) == 3
        for st in s['steps']:
            assert (st['n_gained'] - st['n_lost']
                    == st['change_in_corroborated'])
    written = list(tmp_path.glob('s7_gainloss_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['series']) == 80
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def test_s7_alignment_fires_on_a_reordered_a_element_list():
    """L9: index-wise counting is wrong the moment A's order moves."""
    by_kc, spawn = s7_sides()
    keys = [(0, 1, 1200, cb) for cb in dw.CUTS]
    masks = {k: dw.s7_mask(by_kc, spawn, k) for k in keys}
    dw.assert_aligned(masks, keys)                  # the real series aligns

    m = masks[keys[2]]
    el = dict(m['el_a'])
    order = np.arange(el['n'])[::-1]
    for f in ('x', 'y', 'dir'):
        el[f] = el[f][order]
    masks[keys[2]] = {**m, 'el_a': el}
    with pytest.raises(AssertionError):
        dw.assert_aligned(masks, keys)


def test_s7_nearest_same_dir_takes_across_first_then_along_then_index():
    """L10: the search's whole convention, and the null case.

    `b_at` places B by (across, along) from A's element 0, so each case states
    the answer it expects. The third element is nearer in a straight line and
    further across, which is exactly the case the convention decides.
    """
    a = side(wall_grid())
    el_a = a['el']

    # across decides, even when another element is closer euclidean
    el_b = b_at(el_a, [(0.30, 0.00), (0.05, 0.50)])
    near = dw.nearest_same_dir(el_a, 0, el_b)
    assert near['b_index'] == 1
    assert near['across_m'] == pytest.approx(0.05)
    assert near['along_m'] == pytest.approx(0.50)
    assert near['dist_m'] == pytest.approx(math.hypot(0.05, 0.50))

    # equal across: the smaller along wins
    el_b = b_at(el_a, [(0.20, 0.40), (0.20, 0.10)])
    assert dw.nearest_same_dir(el_a, 0, el_b)['b_index'] == 1

    # equal across and equal along: the lowest index wins
    el_b = b_at(el_a, [(0.20, 0.10), (0.20, -0.10)])
    near = dw.nearest_same_dir(el_a, 0, el_b)
    assert near['b_index'] == 0
    assert near['across_m'] == pytest.approx(0.20)
    assert near['along_m'] == pytest.approx(0.10)

    # no element facing A's way at all
    el_b = b_at(el_a, [(0.20, 0.00)])
    el_b['dir'] = el_b['dir'] + 1
    assert dw.nearest_same_dir(el_a, 0, el_b) is None
    assert dw.nearest_same_dir(el_a, 0, {'n': 0, 'x': np.empty(0),
                                         'y': np.empty(0), 'nx': np.empty(0),
                                         'ny': np.empty(0),
                                         'dir': np.empty(0, dtype=np.int64)}
                               ) is None


def test_s7_s6_row_is_the_split_plus_shares():
    """L11: exposing the mask left S6's row exactly where it was.

    `s6_row` is `s6_classify` plus `shares`, and the mask the split exposes is
    the array the share was counted from -- `corroborated & assigned` over
    `assigned` IS the corroborated share, which is what makes G5 an equality.
    """
    by_kc, spawn = s6_sides()
    for key in ((0, 1, 1200, 60), (2, 3, 240, 240), (4, 0, 60, 1200)):
        row = dw.s6_row(by_kc, spawn, key)
        res, el_b, a, b = dw.s6_classify(by_kc, spawn, key)
        sh = dw.shares(res)
        assert row['L_A_m'] == sh['L_A']
        assert row['corroborated'] == sh['corroborated']
        assert row['not_corroborated'] == sh['not_corroborated']
        assert row['n_corroborated'] == sh['n_corroborated']
        assert row['n_elements'] == sh['n_elements']
        assert (row['A_stem'], row['B_stem']) == (a['stem'], b['stem'])

        n = int(res['assigned'].sum())
        c = int((res['corroborated'] & res['assigned']).sum())
        assert (c, n) == (sh['n_corroborated'], sh['n_elements'])
        assert (c / n) == sh['corroborated']


def test_s7_refuses_a_dirty_tree(monkeypatch):
    """L12a: a result file must not name a commit it was not produced by."""
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s7()
    assert e.value.code == 2


def test_s7_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """L12b: P10 must be registered in history behind the code that tests it."""
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] in ('status', 'merge-base') else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        dw.run_s7()
    assert e.value.code == 2


def test_s7_refuses_when_the_spec_at_head_lacks_p10(monkeypatch):
    """L12c: ancestry is not enough; the spec at HEAD must carry the header."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return '## 9\n\n**P9. REGISTERED 8 Oct 2026.** something\n'
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s7()
    assert e.value.code == 2


def test_s7_refuses_a_wrong_s6_sha256(tmp_path, monkeypatch):
    """L12d: G5 is an equality against ONE S6 run, so that run is pinned.

    The refusal lands before the corpus is loaded, so no map is read here.
    """
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return ('## Gains and losses (P10)\n\n'
                    '**P10. REGISTERED 9 Oct 2026.** Losses under P3.\n')
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    (tmp_path / dw.S6_RESULT).write_text('{"rows": [], "P9": {"failures": []}}')
    with pytest.raises(SystemExit) as e:
        dw.run_s7(tmp_path)
    assert e.value.code == 2


# ────────── S8: cell states under losses, G6 and P11 (the C series) ─────────
#
# SYNTHETIC GRIDS ONLY. No real grid is read at a lost element's cells here or
# anywhere else in this file.

from trinary_map import cell_index as _cell_index           # noqa: E402


def wall_row_grid(spec, w_total=S7_GRID_W, h=12, wall_row=6):
    """One row of cells painted from `spec`, with FREE above the wall row.

    `spec` maps a column to its state in `wall_row`; columns not named stay
    UNKNOWN. A column that is OCC with FREE below it in grid terms -- which is
    row `wall_row - 1`, since the grid is bottom-up -- yields one element with
    dir '-y', exactly as `wall_grid` does.
    """
    g = blank(h, w_total)
    g[:wall_row, :] = FREE
    for col, state in spec.items():
        g[wall_row, col] = state
    return g


def run_of(lo, hi, **kw):
    """`wall_row_grid` for one OCC run from `lo` to `hi` inclusive."""
    return wall_row_grid({c: OCC for c in range(lo, hi + 1)}, **kw)


def sided(grid, stem, origin=ORIGIN):
    """A side with its own origin, which is what a lattice shift needs."""
    s = side(grid, origin=origin)
    return {'stem': stem, 'el': s['el'], 'seg_of': s['seg_of'], 'grid': grid,
            'rho': s['rho'], 'origin': origin, 'shape': grid.shape,
            'segments': [{'p0': [0.0, 0.55], 'p1': [1.1, 0.55]}]}


def st(state, on_grid=True, row=0, col=0):
    """A `state_at` result, built by hand."""
    return {'map': 'synthetic', 'row': row, 'col': col, 'on_grid': on_grid,
            'state': state if on_grid else None}


def test_s8_cell_centres_for_all_four_directions():
    """C1: the wall and front centres are the midpoint stepped by its normal.

    One occupied cell ringed by free cells fires all four passes of DIRS, so
    this covers every direction at once, and the two centres are checked by
    landing them back on the cells they must name.
    """
    rho, origin = RHO, (0.37, -0.21)        # a deliberately awkward origin
    g = np.full((7, 7), FREE, dtype=np.uint8)
    g[3, 3] = OCC
    el = gw.face_elements(g, rho, origin)
    assert el['n'] == 4
    seen = {}
    for i in range(el['n']):
        wall, front = dw.cell_centres_of(el, i, rho)
        wr = int(_cell_index(wall[1], origin[1], rho))
        wc = int(_cell_index(wall[0], origin[0], rho))
        fr = int(_cell_index(front[1], origin[1], rho))
        fc = int(_cell_index(front[0], origin[0], rho))
        drow, dcol = gw.DIRS[int(el['dir'][i])][:2]
        assert (wr, wc) == (3, 3)                    # always the occupied cell
        assert (fr, fc) == (3 + drow, 3 + dcol)      # always the free neighbour
        assert g[wr, wc] == OCC and g[fr, fc] == FREE
        seen[gw.DIRS[int(el['dir'][i])][4]] = (wall, front)
    assert set(seen) == {'+x', '-x', '+y', '-y'}
    # DIRS makes the normal the same pair as (dcol, drow), which is why one
    # formula covers all four
    for d in gw.DIRS:
        assert (d[2], d[3]) == (float(d[1]), float(d[0]))


def test_s8_cell_centres_round_trip_a_whole_synthetic_wall():
    """C2: every element of a slab lands wall=OCC, front=FREE, sub-cell origin."""
    rho, origin = RHO, (0.37, -0.21)
    g = blank(20, 20, FREE)
    g[10, 2:15] = OCC
    el = gw.face_elements(g, rho, origin)
    assert el['n'] > 20
    for i in range(el['n']):
        wall, front = dw.cell_centres_of(el, i, rho)
        wr = int(_cell_index(wall[1], origin[1], rho))
        wc = int(_cell_index(wall[0], origin[0], rho))
        fr = int(_cell_index(front[1], origin[1], rho))
        fc = int(_cell_index(front[0], origin[0], rho))
        assert g[wr, wc] == OCC
        assert g[fr, fc] == FREE


def test_s8_state_at_reads_by_position_through_the_row_order():
    """C3: the lookup carries the origin, the resolution and the bottom-up order.

    The grid `load_grid` hands on is bottom-up, so row grows with y. A map
    whose origin is not on a whole cell is the case that would catch an
    index-based shortcut, so the origin here is deliberately off-lattice.
    """
    origin = (0.37, -0.21)
    g = blank(6, 5, FREE)
    g[0, 0] = OCC                       # minimum y, minimum x
    g[5, 4] = UNK                       # maximum y, maximum x
    s = sided(g, 'synthetic', origin=origin)
    # cell (0, 0)'s centre is the origin itself under CELL_CENTRE_OFFSET 0.0
    assert dw.state_at(s, origin)['state'] == OCC
    assert dw.state_at(s, origin)['on_grid'] is True
    assert dw.state_at(s, (origin[0], origin[1]))['row'] == 0
    top = (origin[0] + 4 * RHO, origin[1] + 5 * RHO)
    assert dw.state_at(s, top) == {'map': 'synthetic', 'row': 5, 'col': 4,
                                   'on_grid': True, 'state': UNK}
    # anywhere inside a cell reads that cell, not just its centre
    for dx in (-0.049, 0.0, 0.049):
        for dy in (-0.049, 0.0, 0.049):
            assert dw.state_at(s, (origin[0] + dx,
                                   origin[1] + dy))['state'] == OCC
    assert dw.state_name(OCC) == 'occupied'
    assert dw.state_name(FREE) == 'free'
    assert dw.state_name(UNK) == 'unknown'
    assert dw.state_name(None) is None


def test_s8_off_grid_on_both_sides_of_every_edge():
    """C4: cell i spans [origin + (i-0.5)rho, origin + (i+0.5)rho).

    So the grid covers [origin - rho/2, origin + (n - 0.5)rho) on each axis:
    the lower edge is inclusive and the upper exclusive, which is `cell_index`
    flooring. Each edge is probed just inside and just outside.
    """
    origin = (0.37, -0.21)
    h, w = 6, 5
    s = sided(blank(h, w, FREE), 'synthetic', origin=origin)
    half = 0.5 * RHO
    x_lo, x_hi = origin[0] - half, origin[0] + (w - 0.5) * RHO
    y_lo, y_hi = origin[1] - half, origin[1] + (h - 0.5) * RHO
    inside = 1e-6

    assert dw.state_at(s, (x_lo + inside, origin[1]))['on_grid'] is True
    assert dw.state_at(s, (x_lo - inside, origin[1]))['on_grid'] is False
    assert dw.state_at(s, (x_hi - inside, origin[1]))['on_grid'] is True
    assert dw.state_at(s, (x_hi + inside, origin[1]))['on_grid'] is False
    assert dw.state_at(s, (origin[0], y_lo + inside))['on_grid'] is True
    assert dw.state_at(s, (origin[0], y_lo - inside))['on_grid'] is False
    assert dw.state_at(s, (origin[0], y_hi - inside))['on_grid'] is True
    assert dw.state_at(s, (origin[0], y_hi + inside))['on_grid'] is False
    # off the grid the state is None, never UNKNOWN -- P11's first class turns
    # on telling those two apart
    assert dw.state_at(s, (x_lo - 1.0, origin[1]))['state'] is None


def test_s8_classification_order_and_every_class():
    """C5: the four classes in the spec's order, each reached.

    Off grid is decided FIRST, so a wall cell that is both off the grid and
    not occupied is off grid and is never counted as freed -- which matters,
    because off-grid elements count against P11.
    """
    # off grid, by the wall cell and by the front cell, and both
    assert dw.classify_cells(st(None, on_grid=False), st(FREE)) == dw.OFF_GRID
    assert dw.classify_cells(st(OCC), st(None, on_grid=False)) == dw.OFF_GRID
    assert dw.classify_cells(st(None, on_grid=False),
                             st(None, on_grid=False)) == dw.OFF_GRID
    # and off grid wins over everything else
    for front in (st(FREE), st(OCC), st(UNK)):
        assert dw.classify_cells(st(None, on_grid=False),
                                 front) == dw.OFF_GRID

    assert dw.classify_cells(st(FREE), st(FREE)) == dw.WALL_FREED
    assert dw.classify_cells(st(UNK), st(FREE)) == dw.WALL_FREED
    assert dw.classify_cells(st(FREE), st(OCC)) == dw.WALL_FREED   # freed wins
    assert dw.classify_cells(st(OCC), st(OCC)) == dw.FRONT_CLOSED
    assert dw.classify_cells(st(OCC), st(UNK)) == dw.FRONT_CLOSED
    assert dw.classify_cells(st(OCC), st(FREE)) == dw.UNCHANGED
    assert dw.P11_CLASSES == (dw.OFF_GRID, dw.WALL_FREED, dw.FRONT_CLOSED,
                              dw.UNCHANGED)


@pytest.mark.parametrize('n_freed,expected', [
    (0, False), (105, False), (106, False), (107, True), (213, True)])
def test_s8_p11_threshold_is_107_of_213(n_freed, expected):
    """C6: at least half of 213 is 107, so 106 fails. Off grid stays in."""
    records = ([{'class': dw.WALL_FREED}] * n_freed
               + [{'class': dw.OFF_GRID}] * (213 - n_freed))
    out = dw.p11_verdict(records)
    assert (out['of'], out['min_required']) == (213, 107)
    assert out['n_freed'] == n_freed
    assert out['holds'] is expected
    # the denominator is the whole set, off-grid elements included
    assert sum(out['by_class'].values()) == 213


def test_s8_p11_denominator_is_asserted():
    """C6b: a set that is not 213 long is a bug, not a smaller denominator."""
    with pytest.raises(AssertionError):
        dw.p11_verdict([{'class': dw.WALL_FREED}] * 212)


def test_s8_lattice_offsets_map_to_half_open_half_cell():
    """C7: the fractional origin difference in cells, mapped to [-0.5, 0.5)."""
    by_kc = {}
    for k, shifts in enumerate(([0.0, 0.0, 0.0, 0.0], [0.0, 0.03, -0.07, 0.4])):
        for c, s in zip(dw.CUTS, shifts):
            by_kc[(k, c)] = sided(run_of(0, 5), f'k{k}_cut{c}',
                                  origin=(s, 0.0))
    out = dw.lattice_offsets(by_kc, dw.CUTS, n_robots=2)
    assert len(out) == 2 * (len(dw.CUTS) - 1)
    flat = {(o['B'], o['step']): (o['offset_x_cells'], o['offset_y_cells'])
            for o in out}
    # k0 never moves
    for step in ('cut60->cut120', 'cut120->cut240', 'cut240->cut1200'):
        assert flat[(0, step)] == pytest.approx((0.0, 0.0))
    # k1: +0.03 m is +0.3 cells; -0.10 m is a whole cell, so 0.0; +0.47 m is
    # 4.7 cells, whose fractional part 0.7 maps to -0.3
    assert flat[(1, 'cut60->cut120')][0] == pytest.approx(0.3)
    assert flat[(1, 'cut120->cut240')][0] == pytest.approx(0.0)
    assert flat[(1, 'cut240->cut1200')][0] == pytest.approx(-0.3)
    for v in flat.values():
        assert -0.5 <= v[0] < 0.5 and -0.5 <= v[1] < 0.5


# ── the end-to-end synthetic corpus ─────────────────────────────────────────
#
# Two robots, four cuts. A (k0) carries a 13-column wall at every cut, so its
# element set never moves. B (k1) is painted per cut to put a loss in each
# class that a shared lattice can reach:
#
#   cut60   OCC 0..5                    A's 0..5 corroborated
#   cut120  OCC 0..5                    unchanged, so no loss on this step
#   cut240  col 0 FREE, col 1 UNKNOWN,  cols 0 and 1 lost: wall cell freed, to
#           OCC 2..9, row5 col3 UNKNOWN free and to unknown. Col 3 lost: its
#                                       wall cell is still OCC and its front
#                                       cell is now UNKNOWN -> front closed
#   cut1200 OCC 2..9, origin +0.9 m     a 9-cell shift, so A's columns run off
#                                       B's grid -> off grid
#
# "Unchanged" is NOT reachable here, and the reason is worth stating: the
# looked-up cell being OCC with a FREE front means B HAS a face there, within
# half a cell along of b, so A's element would be corroborated and would not be
# a loss at all. It needs A's and B's lattices offset so that b sits near T1's
# along limit, which this corpus does not do. C5 covers the class directly.

def s8_sides():
    """The corpus above, as a `by_kc` and a spawn table."""
    spec240 = {0: FREE, 1: UNK}
    spec240.update({c: OCC for c in range(2, 10)})
    g240 = wall_row_grid(spec240)
    g240[5, 3] = UNK                      # close col 3's front cell
    grids = {60: (run_of(0, 5), (0.0, 0.0)),
             120: (run_of(0, 5), (0.0, 0.0)),
             240: (g240, (0.0, 0.0)),
             1200: (run_of(2, 9), (0.9, 0.0))}
    by_kc, spawn = {}, {0: (0.0, 0.0), 1: (0.0, 0.0)}
    for c in dw.CUTS:
        by_kc[(0, c)] = sided(run_of(0, 12), f'A_k0_cut{c}')
        g, o = grids[c]
        by_kc[(1, c)] = sided(g, f'B_k1_cut{c}', origin=o)
    return by_kc, spawn


def s8_fake_s7(by_kc, spawn, n_robots=2):
    """An S7 payload for the synthetic corpus, built with S7's own code."""
    keys = dw.matrix_keys(dw.CUTS, n_robots)
    masks = {k: dw.s7_mask(by_kc, spawn, k) for k in keys}
    series = [dw.gain_loss_series(masks, a, b, ca, dw.CUTS)
              for (a, b, ca) in dw.series_keys(dw.CUTS, n_robots)]
    lost = dw.lost_elements(masks, series, dw.CUTS)
    return {'lost_elements': lost, 'rho_m': RHO, 'series': series}


def test_s8_g6_holding_reaches_three_classes_end_to_end(tmp_path):
    """C8: the whole pipeline on the corpus above, and the classes it reaches."""
    by_kc, spawn = s8_sides()
    s7 = s8_fake_s7(by_kc, spawn)
    n = len(s7['lost_elements'])
    assert n > 0
    out = dw.s8_cellstates(by_kc, spawn, s7,
                           {'stage': 's8', 'head': 'synthetic'}, tmp_path,
                           n_robots=2, n_expected=n)
    assert out['G6']['all_ok']
    assert out['G6']['lost_set']['equal']
    assert out['G6']['states_at_k']['n_ok'] == n
    assert out['n_lost'] == n
    classes = {r['class'] for r in out['elements']}
    assert dw.WALL_FREED in classes
    assert dw.FRONT_CLOSED in classes
    assert dw.OFF_GRID in classes
    # a freed wall cell is reached both as free and as unknown
    freed = {r['wall_at_kp'] for r in out['elements']
             if r['class'] == dw.WALL_FREED}
    assert {'free', 'unknown'} <= freed
    # off grid records which cell left
    for r in out['elements']:
        if r['class'] == dw.OFF_GRID:
            assert r['off_grid_cell'] in ('wall', 'front', 'both')
        else:
            assert r['off_grid_cell'] is None
        assert r['wall_at_k'] == 'occupied' and r['front_at_k'] == 'free'
        assert r['s7_bin'] in dw.S7_BINS or r['s7_bin'] == dw.BIN_WITHIN_T1
    assert out['P11']['of'] == n
    written = list(tmp_path.glob('s8_cellstates_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['elements']) == n
    assert len(payload['reported']['lattice_offsets']) == 2 * 3
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def _s8_spy(monkeypatch):
    """A spy on the grid reader: which maps were read, in order."""
    seen = []
    real = dw.state_at

    def spy(side, pos):
        seen.append(side['stem'])
        return real(side, pos)

    monkeypatch.setattr(dw, 'state_at', spy)
    return seen


def _expected_k_reads(by_kc, s7):
    """Exactly which map each state read must come from, and how many times.

    A map CANNOT be ruled out by its stem alone: B at cut240 is the k' map of
    the cut120 -> cut240 step and the k map of cut240 -> cut1200, so banning
    the stem would ban a legitimate k read. What pins the claim is the count:
    two reads per lost element -- its wall cell and its front cell -- each on
    THAT element's own k map, and nothing else. Any k' read at all breaks it.
    """
    want = collections.Counter()
    for r in s7['lost_elements']:
        want[by_kc[(r['B'], r['from_cut_B'])]['stem']] += 2
    return want


def test_s8_g6_a_lost_set_short_by_one_stops_the_stage(tmp_path, capsys,
                                                       monkeypatch):
    """C9: G6 part 1 fails, and no grid is opened at all -- not even at k."""
    by_kc, spawn = s8_sides()
    s7 = s8_fake_s7(by_kc, spawn)
    n = len(s7['lost_elements'])
    dropped = s7['lost_elements'].pop()
    seen = _s8_spy(monkeypatch)

    with pytest.raises(SystemExit) as e:
        dw.s8_cellstates(by_kc, spawn, s7,
                         {'stage': 's8', 'head': 'synthetic'}, tmp_path,
                         n_robots=2, n_expected=n)
    assert e.value.code == 2
    assert seen == []                        # part 1 fails before any lookup
    payload = _assert_s8_fail_file(tmp_path, capsys, by_kc, s7, dropped)
    assert payload['G6']['states_at_k'] is None
    assert payload['G6']['lost_set']['only_in_s8']


def test_s8_g6_a_wrong_state_at_k_stops_the_stage(tmp_path, capsys,
                                                  monkeypatch):
    """C10: G6 part 2 fails, and no grid at k' is opened.

    The wall cell of one lost element's b is repainted free in B's grid AT K,
    so the measure's own precondition -- b separates an occupied cell from a
    free one -- no longer holds and the stage must stop before looking at k'.
    """
    by_kc, spawn = s8_sides()
    s7 = s8_fake_s7(by_kc, spawn)
    n = len(s7['lost_elements'])
    rec = s7['lost_elements'][0]
    # b is at k, so repaint B's grid at k under the whole wall row
    by_kc[(rec['B'], rec['from_cut_B'])]['grid'][6, :] = FREE

    seen = _s8_spy(monkeypatch)
    with pytest.raises(SystemExit) as e:
        dw.s8_cellstates(by_kc, spawn, s7,
                         {'stage': 's8', 'head': 'synthetic'}, tmp_path,
                         n_robots=2, n_expected=n)
    assert e.value.code == 2
    assert seen, 'part 2 reads B at k, which is the point of it'
    assert collections.Counter(seen) == _expected_k_reads(by_kc, s7)
    payload = _assert_s8_fail_file(tmp_path, capsys, by_kc, s7, None)
    assert payload['G6']['states_at_k'] is not None
    assert payload['G6']['states_at_k']['differing']


def _assert_s8_fail_file(tmp_path, capsys, by_kc, s7, _dropped):
    """The fail file holds the G6 comparisons and names no k' map."""
    assert not list(tmp_path.glob('s8_cellstates_*.json'))
    written = list(tmp_path.glob('s8_g6_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert set(payload) >= {'G6', 'stage', 'seconds'}
    assert 'elements' not in payload and 'P11' not in payload
    assert 'reported' not in payload
    out = capsys.readouterr().out
    assert 'G6' in out and 'FAIL' in out
    for word in ('P11', 'wall cells freed', 'classes by S7 distance bin',
                 'lattice offsets'):
        assert word not in out
    return payload


def test_s8_refuses_a_dirty_tree(monkeypatch):
    """C11a: a result file must not name a commit it was not produced by."""
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s8()
    assert e.value.code == 2


def test_s8_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """C11b: P11 must be registered in history behind the code that tests it."""
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] in ('status', 'merge-base') else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        dw.run_s8()
    assert e.value.code == 2


def test_s8_refuses_when_the_spec_at_head_lacks_p11(monkeypatch):
    """C11c: ancestry is not enough; the spec at HEAD must carry the header."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return '## 9\n\n**P10. REGISTERED 9 Oct 2026.** something\n'
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s8()
    assert e.value.code == 2


def test_s8_refuses_a_wrong_s7_sha256(tmp_path, monkeypatch):
    """C11d: G6 is an equality against ONE S7 run, so that run is pinned."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return ('## Cell states under losses (P11)\n\n'
                    '**P11. REGISTERED 9 Oct 2026.** Wall cells freed.\n')
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    (tmp_path / dw.S7_RESULT).write_text('{"lost_elements": [], "rho_m": 0.1}')
    with pytest.raises(SystemExit) as e:
        dw.run_s8(tmp_path)
    assert e.value.code == 2


# ───────── S9: segmentation or geometry, G7 and P12 (the Q series) ──────────
#
# SYNTHETIC ONLY. No segment assignment and no e' distance is read on a real
# map here or anywhere else in this file.

def seg_side(grid, stem, assign, origin=ORIGIN):
    """A side whose elements are assigned by a predicate on (row, col, dir).

    `fake_side` assigns every element and `unassigned_side` names indices;
    neither is much use when the question IS which elements carry a segment, so
    this takes `assign(row, col, dir) -> bool` and builds `seg_of` from it.
    Segment 0 for the assigned, -1 for the rest, which is what
    `face_elements_only` and S9 both read.
    """
    el = gw.face_elements(grid, RHO, origin)
    seg_of = np.full(el['n'], -1, dtype=np.int64)
    for i in range(el['n']):
        if assign(int(el['row'][i]), int(el['col'][i]), int(el['dir'][i])):
            seg_of[i] = 0
    return {'stem': stem, 'el': el, 'seg_of': seg_of, 'grid': grid,
            'rho': RHO, 'origin': origin, 'shape': grid.shape,
            'segments': [{'p0': [0.0, 0.55], 'p1': [1.1, 0.55]}]}


def two_runs(lo_hi_by_row, w_total=S7_GRID_W, h=12):
    """FREE below each occupied run, so every run exposes one '-y' face line."""
    g = blank(h, w_total)
    g[:max(lo_hi_by_row) + 1, :] = FREE
    for row, (lo, hi) in lo_hi_by_row.items():
        g[row, lo:hi + 1] = OCC
    return g


def test_s9_find_e_prime_for_all_four_directions():
    """Q1: e' is named by (row, col, dir), which the element arrays carry."""
    rho, origin = RHO, (0.37, -0.21)
    g = np.full((9, 9), FREE, dtype=np.uint8)
    g[4, 4] = OCC
    el = gw.face_elements(g, rho, origin)
    assert el['n'] == 4
    for i in range(el['n']):
        d = int(el['dir'][i])
        j = dw.find_e_prime(el, (4, 4), d)
        assert j == i
        # and the front cell it implies is the free neighbour
        drow, dcol = gw.DIRS[d][:2]
        assert g[4 + drow, 4 + dcol] == FREE
    # the triple really is a key: two occupied cells do not collide
    g[4, 6] = OCC
    el2 = gw.face_elements(g, rho, origin)
    seen = {(int(el2['row'][i]), int(el2['col'][i]), int(el2['dir'][i]))
            for i in range(el2['n'])}
    assert len(seen) == el2['n']


def test_s9_find_e_prime_under_a_sub_cell_lattice_offset():
    """Q2: the k' cell is found by position, so a shifted origin still lands.

    b's wall-cell centre comes from k's lattice; the cell that CONTAINS it at
    k' is a different index when the origins differ by part of a cell, and e'
    is the element of that cell. This walks the offset across a whole cell.
    """
    g = two_runs({6: (0, 12)})
    for shift in (0.0, 0.01, 0.04, 0.06, 0.09):
        side_k = seg_side(g, 'k', lambda r, c, d: True, origin=(0.0, 0.0))
        side_kp = seg_side(g, 'kp', lambda r, c, d: True,
                           origin=(shift, 0.0))
        i = int(np.nonzero(side_k['el']['col'] == 5)[0][0])
        wall, _front = dw.cell_centres_of(side_k['el'], i, RHO)
        rc = dw.state_at(side_kp, wall)
        j = dw.find_e_prime(side_kp['el'], (rc['row'], rc['col']),
                            int(side_k['el']['dir'][i]))
        assert j is not None, shift
        # the element found is the one whose own wall cell is that cell
        assert int(side_kp['el']['row'][j]) == rc['row']
        assert int(side_kp['el']['col'][j]) == rc['col']


def test_s9_find_e_prime_fails_cleanly():
    """Q3: None when the cell pair carries no element, or none in b's direction."""
    g = two_runs({6: (0, 12)})
    s = seg_side(g, 'kp', lambda r, c, d: True)
    row6 = int(np.nonzero(s['el']['col'] == 3)[0][0])
    d = int(s['el']['dir'][row6])
    assert dw.find_e_prime(s['el'], (6, 3), d) is not None
    # a cell with no element at all
    assert dw.find_e_prime(s['el'], (0, 0), d) is None
    # the right cell, the wrong direction
    other = next(k for k in range(4) if k != d)
    assert dw.find_e_prime(s['el'], (6, 3), other) is None


def test_s9_assignment_indexes_the_full_array_not_the_subset():
    """Q4: seg_of is parallel to the FULL element array.

    `extract_with_elements` builds `seg_of = np.full(el['n'], -1)` off
    `face_elements`, so index i of `seg_of` is index i of the full arrays --
    which is why `seg_of[e'] >= 0` is e''s assignment. Reading it off
    `face_elements_only`'s subset would silently name a different element, and
    this is the case that would catch that.
    """
    g = two_runs({2: (0, 12), 6: (0, 12)})
    s = seg_side(g, 'kp', lambda r, c, d: r == 2)      # only row 2 assigned
    full, sub = s['el'], dw.face_elements_only(s['el'], s['seg_of'])
    assert sub['n'] < full['n']
    for i in range(full['n']):
        assert bool(s['seg_of'][i] >= 0) == (int(full['row'][i]) == 2)
    # the two indexings are NOT the same map: the subset skips the unassigned,
    # so beyond the first gap a subset index names a different element
    assigned = np.nonzero(s['seg_of'] >= 0)[0]
    assert not np.array_equal(assigned, np.arange(assigned.size))
    for sub_i, full_i in enumerate(assigned):
        assert sub['x'][sub_i] == full['x'][full_i]
    # a row-6 element exists in the full array and is unassigned; it has no
    # index at all in the subset, so only the full-array read can see it
    j = dw.find_e_prime(full, (6, 4), 3)
    assert j is not None
    assert s['seg_of'][j] < 0
    assert j not in set(assigned.tolist())


@pytest.mark.parametrize('across,along,assigned,expected', [
    (0.0, 0.0, False, dw.SEGMENTATION),          # unassigned wins outright
    (9.9, 9.9, False, dw.SEGMENTATION),
    (0.5, 0.0, True, dw.GEOM_ACROSS),
    (0.0, 0.5, True, dw.GEOM_ALONG),
])
def test_s9_classification_order(across, along, assigned, expected):
    """Q5: segmentation first, then across, then along."""
    assert dw.classify_unchanged(assigned, across, along, EPS, RHO) == expected


def test_s9_classification_boundaries_are_t1s_own():
    """Q6: eps and rho/2 inclusive to 1e-9 m, exactly as T1 has them.

    `within_t1_tolerances` is the one statement of that boundary and both S7's
    `within_t1` flag and these classes go through it, so eps can be at the
    limit here without the two stages disagreeing about which side it is on.
    """
    tol = dw.T1_TOL_M
    # across: at eps and at eps + tol it is NOT beyond; past that it is
    assert dw.classify_unchanged(True, EPS, 0.0, EPS, RHO) == dw.GEOM_ALONG
    assert dw.classify_unchanged(True, EPS + tol, 0.0, EPS,
                                 RHO) == dw.GEOM_ALONG
    assert dw.classify_unchanged(True, EPS + 2 * tol, 0.0, EPS,
                                 RHO) == dw.GEOM_ACROSS
    # and the predicate agrees on both tolerances
    half = 0.5 * RHO
    assert dw.within_t1_tolerances(EPS, half, EPS, RHO)
    assert dw.within_t1_tolerances(EPS + tol, half + tol, EPS, RHO)
    assert not dw.within_t1_tolerances(EPS + 2 * tol, half, EPS, RHO)
    assert not dw.within_t1_tolerances(EPS, half + 2 * tol, EPS, RHO)


@pytest.mark.parametrize('n_seg,expected', [
    (0, False), (38, False), (39, True), (78, True)])
def test_s9_p12_threshold_is_39_of_78(n_seg, expected):
    """Q7: at least half of 78 is 39, so 38 fails. The 54 seen stay out."""
    recs = ([{'class': dw.SEGMENTATION, 's7_bin': dw.BIN_WITHIN_EPS}] * n_seg
            + [{'class': dw.GEOM_ACROSS, 's7_bin': dw.BIN_WITHIN_EPS}]
            * (78 - n_seg)
            + [{'class': dw.SEGMENTATION, 's7_bin': dw.BIN_FAR}] * 54)
    out = dw.p12_verdict(recs)
    assert (out['of'], out['min_required']) == (78, 39)
    assert out['n_segmentation'] == n_seg
    assert out['holds'] is expected
    # the carve-out is reported beside the test, never inside it
    assert out['seen']['of'] == 54
    assert out['seen']['n_segmentation'] == 54
    assert sum(out['by_class'].values()) == 78


def test_s9_p12_denominators_are_asserted():
    """Q7b: 78 tested and 54 seen are the registered numbers, not whatever came."""
    with pytest.raises(AssertionError):
        dw.p12_verdict([{'class': dw.SEGMENTATION,
                         's7_bin': dw.BIN_WITHIN_EPS}] * 77
                       + [{'class': dw.SEGMENTATION,
                           's7_bin': dw.BIN_FAR}] * 54)


# ── the end-to-end synthetic corpus ─────────────────────────────────────────
#
# Two robots, four cuts, and A's elements lost to SEGMENTATION, which is the
# one class a corpus on a shared lattice can reach end to end:
#
#   A (k0), every cut   row 6 OCC cols 0..12, all assigned
#   B (k1) at cut60     the same, all assigned -> A fully corroborated
#   B (k1) at 120, 240  row 6 OCC and row 2 OCC, and ONLY row 2 assigned
#   and 1200
#
# So at the 60 -> 120 step every one of A's 13 elements loses its only
# corroborator; b's two cells still read occupied and free at k', so the loss
# is UNCHANGED; and e' -- the row-6 element at that cell pair -- exists and is
# unassigned, which is segmentation. The row-2 line is there to put the
# elements in a TESTED bin: it is assigned and 0.40 m across, which is beyond
# eps and inside a metre, so S7's bin is `beyond_eps_across`, not the carve-out.
#
# THE GEOMETRY CLASSES ARE NOT REACHABLE HERE, and the reason is structural. e'
# is the element of the cell CONTAINING b's wall-cell centre, so its midpoint
# sits within half a cell of b's in each axis; with b itself inside T1 at k,
# e' lands within eps + rho across and rho along of e. Reaching a geometric
# class therefore needs A's and B's lattices offset so that b sits near T1's
# limit at k and the k' lattice carries e' just past it -- which the real
# corpus can do and this one does not. P5 and P6 cover the classes directly.

def s9_sides():
    """The corpus above, as a `by_kc` and a spawn table."""
    a_grid = two_runs({6: (0, 12)})
    b_early = two_runs({6: (0, 12)})
    b_late = two_runs({2: (0, 12), 6: (0, 12)})
    by_kc, spawn = {}, {0: (0.0, 0.0), 1: (0.0, 0.0)}
    for c in dw.CUTS:
        by_kc[(0, c)] = seg_side(a_grid, f'A_k0_cut{c}', lambda r, c_, d: True)
        if c == 60:
            by_kc[(1, c)] = seg_side(b_early, f'B_k1_cut{c}',
                                     lambda r, c_, d: True)
        else:
            by_kc[(1, c)] = seg_side(b_late, f'B_k1_cut{c}',
                                     lambda r, c_, d: r == 2)
    return by_kc, spawn


def s9_fake_s8(by_kc, spawn, n_robots=2):
    """An S8 payload for the synthetic corpus, built with S8's own code."""
    keys = dw.matrix_keys(dw.CUTS, n_robots)
    masks = {k: dw.s7_mask(by_kc, spawn, k) for k in keys}
    series = [dw.gain_loss_series(masks, a, b, ca, dw.CUTS)
              for (a, b, ca) in dw.series_keys(dw.CUTS, n_robots)]
    lost = dw.lost_elements(masks, series, dw.CUTS)
    keys_k = sorted({(r['A'], r['B'], r['cut_A'], r['from_cut_B'])
                     for r in lost})
    matches = dw.b_matches(by_kc, spawn, keys_k)
    at_k = dw.g6_states_at_k(by_kc, matches, lost)
    rho = next(iter(by_kc.values()))['rho']
    eps = gw.EPS_FACTOR * rho
    bin_of = {(r['A'], r['B'], r['cut_A'], r['step'], r['element']):
              dw.s7_distance_bin(r, eps, 0.5 * rho) for r in lost}
    out = []
    for row in at_k['rows']:
        side_kp = by_kc[(row['B'], dw._to_cut(row['step']))]
        sw = dw.state_at(side_kp, tuple(row['wall_cell']))
        sf = dw.state_at(side_kp, tuple(row['front_cell']))
        rec = dict(row)
        rec.pop('equal', None)
        rec.update({'map_at_kp': side_kp['stem'],
                    'wall_at_kp': dw.state_name(sw['state']),
                    'front_at_kp': dw.state_name(sf['state']),
                    'wall_rc_at_kp': [sw['row'], sw['col']],
                    'front_rc_at_kp': [sf['row'], sf['col']],
                    'wall_on_grid_at_kp': sw['on_grid'],
                    'front_on_grid_at_kp': sf['on_grid'],
                    'class': dw.classify_cells(sw, sf),
                    'off_grid_cell': None,
                    's7_bin': bin_of[(row['A'], row['B'], row['cut_A'],
                                      row['step'], row['element'])]})
        out.append(rec)
    return {'elements': out, 'rho_m': rho,
            'reported': {'lattice_offsets': dw.lattice_offsets(
                by_kc, dw.CUTS, n_robots)}}


def _s9_counts(s8):
    """The unchanged set this corpus produced, and its tested/seen split."""
    u = [r for r in s8['elements'] if r['class'] == dw.UNCHANGED]
    seen = [r for r in u if r['s7_bin'] == dw.BIN_FAR]
    return len(u), len(u) - len(seen), len(seen)


def test_s9_g7_holding_classifies_every_unchanged_element(tmp_path):
    """Q8: the whole pipeline, and the class the corpus is built to reach."""
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    n_u, n_t, n_s = _s9_counts(s8)
    assert n_u > 0 and n_t > 0
    out = dw.s9_segmentation(by_kc, spawn, s8,
                             {'stage': 's9', 'head': 'synthetic'}, tmp_path,
                             n_robots=2, n_unchanged=n_u, n_tested=n_t,
                             n_seen=n_s)
    assert out['G7']['all_ok']
    assert all(out['G7']['parts'].values())
    assert out['n_unchanged'] == n_u
    assert {r['class'] for r in out['elements']} == {dw.SEGMENTATION}
    for r in out['elements']:
        assert r['assigned'] is False
        assert r['e_prime_seg'] == -1
        assert r['s7_bin'] in (dw.BIN_WITHIN_EPS, dw.BIN_BEYOND_EPS)
    # the corpus reaches both tested bins: the '-y' line's nearest assigned
    # same-dir element is 0.40 m ACROSS, and the one '+x' element's is 0.40 m
    # ALONG, so the two land in different bins
    assert {r['s7_bin'] for r in out['elements']} == {dw.BIN_WITHIN_EPS,
                                                     dw.BIN_BEYOND_EPS}
    assert out['P12']['holds']
    assert out['P12']['n_segmentation'] == n_t
    written = list(tmp_path.glob('s9_segmentation_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['elements']) == n_u
    assert payload['reported']['geometry'] == []
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def _assert_s9_fail_file(tmp_path, capsys):
    """The fail file holds the G7 comparisons and no class count or verdict."""
    assert not list(tmp_path.glob('s9_segmentation_*.json'))
    written = list(tmp_path.glob('s9_g7_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert set(payload) >= {'G7', 'stage', 'seconds'}
    assert 'elements' not in payload and 'P12' not in payload
    assert 'reported' not in payload
    # no class count and no verdict anywhere, by exact key name
    def keys_of(node):
        if isinstance(node, dict):
            return set(node) | set().union(*(keys_of(v) for v in node.values())
                                           or [set()])
        if isinstance(node, list):
            return set().union(*(keys_of(v) for v in node) or [set()])
        return set()
    assert not (keys_of(payload) & {'P12', 'class', 'by_class',
                                    'n_segmentation', 'elements', 'reported'})
    out = capsys.readouterr().out
    assert 'G7' in out and 'FAIL' in out
    for word in ('P12', 'segmentation', 'geometry elements',
                 'classes by S7 distance bin'):
        assert word not in out
    return payload


def _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path, counts=None):
    n_u, n_t, n_s = counts or _s9_counts(s8)
    with pytest.raises(SystemExit) as e:
        dw.s9_segmentation(by_kc, spawn, s8,
                           {'stage': 's9', 'head': 'synthetic'}, tmp_path,
                           n_robots=2, n_unchanged=n_u, n_tested=n_t,
                           n_seen=n_s)
    assert e.value.code == 2


def test_s9_g7_part1_an_unchanged_set_short_by_one(tmp_path, capsys):
    """Q9: part 1 fails on a set that differs by one element."""
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    counts = _s9_counts(s8)
    dropped = next(i for i, r in enumerate(s8['elements'])
                   if r['class'] == dw.UNCHANGED)
    s8['elements'] = [r for i, r in enumerate(s8['elements'])
                      if i != dropped]
    _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path, counts)
    payload = _assert_s9_fail_file(tmp_path, capsys)
    assert payload['G7']['parts']['1_set_and_states'] is False
    assert payload['G7']['unchanged_set']['only_in_s9']


def test_s9_g7_part1_a_differing_cell_state(tmp_path, capsys):
    """Q10: part 1 also fails on a state S9 reads differently from S8.

    Stricter than the spec, which asks only that the SETS agree. An unchanged
    element whose states S9 reads differently is not the element S8
    classified, and the set alone would not notice.
    """
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    counts = _s9_counts(s8)
    for r in s8['elements']:
        if r['class'] == dw.UNCHANGED:
            r['front_at_kp'] = 'unknown'          # S9 will read 'free'
            break
    _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path, counts)
    payload = _assert_s9_fail_file(tmp_path, capsys)
    assert payload['G7']['unchanged_set']['differing_states']


@pytest.mark.parametrize('part,patch', [
    ('2_e_prime_exists', 'find_e_prime'),
    ('2_e_prime_exists', 'e_prime_distances'),
])
def test_s9_g7_part2_a_missing_or_misdirected_e_prime(part, patch, tmp_path,
                                                      capsys, monkeypatch):
    """Q11: no e', and an e' in the wrong direction, both stop the stage.

    DRIVEN BY PATCHING, and deliberately so: on a consistent corpus neither
    case can arise -- an unchanged element's cells ARE an occupied/free pair,
    so the element is there, in b's direction. Part 2 is a consistency check
    between S8 and `graph_walls`, and the only way to exercise its handling is
    to make the finder answer as it would if that consistency broke.
    """
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    counts = _s9_counts(s8)
    calls = {'n': 0}
    real = getattr(dw, patch)

    def once(*a, **k):
        calls['n'] += 1
        return None if calls['n'] == 1 else real(*a, **k)

    monkeypatch.setattr(dw, patch, once)
    _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path, counts)
    payload = _assert_s9_fail_file(tmp_path, capsys)
    assert payload['G7']['parts'][part] is False
    assert payload['G7']['missing_e_prime'] or payload['G7']['wrong_direction']


def _point_e_prime_at_an_assigned_element(monkeypatch, by_kc, inside=False):
    """Make the finder name an ASSIGNED element at k', optionally inside T1.

    The corpus cannot be mutated into this shape: assigning e' at k' would
    make T1 corroborate the element, so it would not be a loss and part 1
    would fail first on a different unchanged set. Patching the finder is the
    only way in, and it is the honest one -- parts 3 and 4 are cross-stage
    consistency checks that a CONSISTENT corpus cannot reach.
    """
    first = {'done': False}
    real_find, real_dist = dw.find_e_prime, dw.e_prime_distances

    def find(el, wall_rc, dir_b):
        if not first['done']:
            hit = np.nonzero(el['dir'] == dir_b)[0]
            for j in hit:                       # the assigned row-2 line
                if int(el['row'][j]) == 2:
                    first['done'] = True
                    return int(j)
        return real_find(el, wall_rc, dir_b)

    def dist(a_side, i, b_side, j, sa, sb):
        out = real_dist(a_side, i, b_side, j, sa, sb)
        if inside and out is not None and int(b_side['el']['row'][j]) == 2:
            out = dict(out, across_m=0.0, along_m=0.0, dist_m=0.0)
        return out

    monkeypatch.setattr(dw, 'find_e_prime', find)
    monkeypatch.setattr(dw, 'e_prime_distances', dist)


def test_s9_g7_part3_an_e_prime_assigned_and_within_t1(tmp_path, capsys,
                                                       monkeypatch):
    """Q12: an e' both assigned and inside T1 contradicts S7's mask.

    If such an e' existed T1 would have corroborated the element and it would
    not be a loss at all, so part 3 is what would catch S7 and S9 disagreeing.
    """
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    counts = _s9_counts(s8)
    _point_e_prime_at_an_assigned_element(monkeypatch, by_kc, inside=True)
    _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path, counts)
    payload = _assert_s9_fail_file(tmp_path, capsys)
    assert payload['G7']['parts']['3_none_assigned_and_within_t1'] is False
    assert payload['G7']['assigned_and_within_t1']


def test_s9_g7_part4_a_carve_out_element_that_is_assigned(tmp_path, capsys,
                                                          monkeypatch):
    """Q13: part 4, which is stricter than the spec, and is checked not assumed.

    The spec calls the "none within 1 m" elements segmentation BY
    CONSTRUCTION. Part 4 checks it. Driven by relabelling one element's bin to
    the carve-out while its e' reads assigned-but-outside-T1, which is the
    shape a broken carve-out would take.
    """
    by_kc, spawn = s9_sides()
    s8 = s9_fake_s8(by_kc, spawn)
    n_u, n_t, n_s = _s9_counts(s8)
    target = next(r for r in s8['elements'] if r['class'] == dw.UNCHANGED)
    target['s7_bin'] = dw.BIN_FAR          # not a field part 1 compares
    # an assigned e', left at its real distance of 0.40 m across, which is
    # outside T1 -- so part 3 holds and part 4 is the one that fires
    _point_e_prime_at_an_assigned_element(monkeypatch, by_kc, inside=False)
    _s9_run_expecting_failure(by_kc, spawn, s8, tmp_path,
                              (n_u, n_t - 1, n_s + 1))
    payload = _assert_s9_fail_file(tmp_path, capsys)
    assert payload['G7']['parts']['4_carve_out_is_segmentation'] is False
    assert payload['G7']['carve_out_assigned']


def test_s9_refuses_a_dirty_tree(monkeypatch):
    """Q14a: a result file must not name a commit it was not produced by."""
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s9()
    assert e.value.code == 2


def test_s9_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """Q14b: P12 must be registered in history behind the code that tests it."""
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] in ('status', 'merge-base') else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        dw.run_s9()
    assert e.value.code == 2


def test_s9_refuses_when_the_spec_at_head_lacks_p12(monkeypatch):
    """P14c: ancestry is not enough; the spec at HEAD must carry the header."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return '## 9\n\n**P11. REGISTERED 9 Oct 2026.** something\n'
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s9()
    assert e.value.code == 2


def test_s9_refuses_a_wrong_s8_sha256(tmp_path, monkeypatch):
    """P14d: G7 is an equality against ONE S8 run, so that run is pinned."""
    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return ('## Unchanged losses (P12)\n\n'
                    '**P12. REGISTERED 9 Oct 2026.** Segmentation.\n')
        return 'c0ffee'
    monkeypatch.setattr(dw, '_git', fake)
    (tmp_path / dw.S8_RESULT).write_text('{"elements": [], "rho_m": 0.1}')
    with pytest.raises(SystemExit) as e:
        dw.run_s9(tmp_path)
    assert e.value.code == 2


def test_s9_nearest_same_dir_docstring_names_the_callers_set():
    """Q15: the docstring fix, and that it is a wording change only.

    The behaviour it describes is pinned elsewhere; what this holds is that the
    docstring now says whose set is scanned, because P12's carve-out rests on
    that set being B's ASSIGNED elements and the old wording read as all of
    them.
    """
    doc = dw.nearest_same_dir.__doc__
    assert 'face_elements_only' in doc
    assert 'ASSIGNED TO A WALL SEGMENT' in doc
    assert "the caller's choice" in doc.lower()
    assert 'scans every same-direction element of B' not in doc
    # and the behaviour is still the behaviour: handed a set, it scans that set
    a = side(wall_grid())
    el_b = b_at(a['el'], [(0.2, 0.0)])
    assert dw.nearest_same_dir(a['el'], 0, el_b)['b_index'] == 0
    assert dw.nearest_same_dir(a['el'], 0, {
        'n': 0, 'x': np.empty(0), 'y': np.empty(0), 'nx': np.empty(0),
        'ny': np.empty(0), 'dir': np.empty(0, dtype=np.int64)}) is None


# ───── S10: the segments behind the unassigned elements (the R series) ──────
#
# SYNTHETIC ONLY. No cell of a member of S_b is read on a real map here or
# anywhere else in this file, and no `changed`, `intact` or `flanked` is
# computed on one.

def s10_side(grid, stem, assign, origin=ORIGIN, theta=0.0):
    """A side whose `segments` list really agrees with its `seg_of`.

    `seg_side` carries one placeholder segment, which is enough for S9 and not
    for S10: check 3 compares a segment's `id` and `n_elements` against the
    members `seg_of` gives, the base rate walks `segments` to reach every
    segment of the map, and the along distance is measured on a segment's own
    `theta`. So the entry is built from `seg_of` here, the way
    `extract_with_elements` builds it -- id by position in the list, members by
    `seg_of == id`.

    `assign(row, col, dir)` returns a segment id or None. `theta` is the line's
    angle for every segment, which on these one-row walls is 0: only |d.u| is
    ever read off it, so the sign the real extractor would pick does not
    matter.
    """
    el = gw.face_elements(grid, RHO, origin)
    seg_of = np.full(el['n'], -1, dtype=np.int64)
    for i in range(el['n']):
        s = assign(int(el['row'][i]), int(el['col'][i]), int(el['dir'][i]))
        if s is not None:
            seg_of[i] = s
    ux, uy = gw.dir_of(theta)
    segments = []
    for s in range(int(seg_of.max()) + 1 if seg_of.size and seg_of.max() >= 0
                   else 0):
        m = np.nonzero(seg_of == s)[0]
        t = el['x'][m] * ux + el['y'][m] * uy
        t0, t1 = float(t.min()) - RHO / 2, float(t.max()) + RHO / 2
        segments.append({
            'id': s, 'theta': theta, 'n_elements': int(m.size),
            'length': t1 - t0, 'normal': list(gw.normal_of(theta)),
            'p0': [t0 * ux, t0 * uy], 'p1': [t1 * ux, t1 * uy]})
    return {'stem': stem, 'el': el, 'seg_of': seg_of, 'grid': grid,
            'rho': RHO, 'origin': origin, 'shape': grid.shape,
            'segments': segments}


def face_line_el(points, dir_i=3):
    """An element table placed by hand, for the face-line cases.

    DIRS[3] is '-y', whose normal is (0, -1), so for these elements ACROSS is
    -y and ALONG is x: a case can put an element a stated distance off the
    line and at a stated place along it, with no grid to paint.
    """
    nx, ny = gw.DIRS[dir_i][2], gw.DIRS[dir_i][3]
    n = len(points)
    return {'n': n,
            'x': np.array([p[0] for p in points], float),
            'y': np.array([p[1] for p in points], float),
            'nx': np.full(n, nx), 'ny': np.full(n, ny),
            'dir': np.full(n, dir_i, dtype=np.int64),
            'row': np.zeros(n, dtype=np.int64),
            'col': np.arange(n, dtype=np.int64)}


def test_s10_members_take_one_route_each_and_intact_is_the_negation():
    """R1: every route of change, per member, and an unchanged S_b.

    The four cases are built on one segment of three members at k and four
    different k' grids, so what differs between them is only B's later map.
    """
    g_k = wall_span(2, 4)                          # three '-y' elements
    side_k = s10_side(g_k, 'B_k', lambda r, c, d: 0)
    assert side_k['segments'][0]['n_elements'] == 3

    intact = s10_side(wall_span(2, 4), 'B_kp', lambda r, c, d: 0)
    ch = dw.segment_change(side_k, intact, 0)
    assert (ch['state'], ch['changed'], ch['routes']) == (dw.INTACT, False, [])
    assert ch['n_members'] == 3 and ch['n_changed_members'] == 0

    freed = wall_span(2, 4)
    freed[6, 3] = FREE                             # the middle wall cell goes
    ch = dw.segment_change(side_k, s10_side(freed, 'B_kp',
                                            lambda r, c, d: 0), 0)
    assert (ch['state'], ch['routes']) == (dw.CHANGED, [dw.WALL_FREED])
    assert ch['n_changed_members'] == 1
    assert [r['member'] for r in ch['changed_members']] == [1]

    closed = wall_span(2, 4)
    closed[5, 2] = UNK                             # a front cell closes
    ch = dw.segment_change(side_k, s10_side(closed, 'B_kp',
                                            lambda r, c, d: 0), 0)
    assert (ch['state'], ch['routes']) == (dw.CHANGED, [dw.FRONT_CLOSED])

    # more than one route at once, which the spec says can happen
    both = wall_span(2, 4)
    both[6, 2] = FREE
    both[5, 4] = UNK
    ch = dw.segment_change(side_k, s10_side(both, 'B_kp',
                                            lambda r, c, d: 0), 0)
    assert ch['routes'] == [dw.WALL_FREED, dw.FRONT_CLOSED]
    assert ch['n_changed_members'] == 2


def test_s10_an_off_grid_member_is_off_grid_and_never_freed():
    """R2: off grid is its own route, and it wins -- it is not a freed wall.

    `trinary_map.sample` fills outside the map with UNKNOWN, which would read
    as a wall cell no longer occupied: the class P13a predicts. The lookup is
    `state_at`, which answers None off the grid, and `classify_cells` puts off
    grid first, so a member whose cells left the map takes that route and only
    that one.
    """
    side_k = s10_side(wall_span(2, 4), 'B_k', lambda r, c, d: 0)
    # k' is the same wall, on a lattice 9 cells to the right: every member's
    # cells fall off the left edge
    away = s10_side(wall_span(2, 4), 'B_kp', lambda r, c, d: 0,
                    origin=(0.9, 0.0))
    ch = dw.segment_change(side_k, away, 0)
    assert ch['routes'] == [dw.OFF_GRID]
    assert dw.WALL_FREED not in ch['routes']
    assert ch['n_changed_members'] == 3
    for r in ch['changed_members']:
        assert r['route'] == dw.OFF_GRID
        assert r['wall_at_kp'] is None and not r['wall_on_grid_at_kp']


def test_s10_the_cell_lookup_never_calls_trinary_map_sample(monkeypatch):
    """R3: a spy on `sample` that raises, over the whole lookup phase.

    The lookup phase only: `load_grid` verifies its own row order with
    `sample` on a map's own lattice and is unaffected, which is why this
    patches the module attribute and runs the two functions that read k'
    rather than the stage.
    """
    tm = importlib.import_module('trinary_map')
    calls = []

    def boom(*a, **k):
        calls.append(a)
        raise AssertionError('trinary_map.sample was called for a lookup')

    monkeypatch.setattr(tm, 'sample', boom)
    monkeypatch.setattr(gw, 'sample', boom)              # its other binding
    assert not hasattr(dw, 'sample')                     # and not a third

    side_k = s10_side(wall_span(2, 4), 'B_k', lambda r, c, d: 0)
    off = s10_side(wall_span(2, 4), 'B_kp', lambda r, c, d: 0,
                   origin=(0.9, 0.0))
    assert dw.segment_change(side_k, side_k, 0)['state'] == dw.INTACT
    assert dw.segment_change(side_k, off, 0)['routes'] == [dw.OFF_GRID]
    assert dw.is_flanked(side_k['el'], side_k['seg_of'], 0)['flanked'] is False
    assert calls == []


def test_s10_flanked_between_two_members_of_one_segment():
    """R4: strictly between two members of ONE segment, on its face line."""
    g = wall_span(0, 12)
    s = s10_side(g, 'B_kp', lambda r, c, d: None if c == 5 else 0)
    i = int(np.nonzero((s['el']['col'] == 5) & (s['el']['dir'] == 3))[0][0])
    assert s['seg_of'][i] == -1
    out = dw.is_flanked(s['el'], s['seg_of'], i)
    assert out['flanked'] is True
    assert out['flanking_segments'] == [0]
    assert out['n_on_face_line'] == 13          # the whole '-y' line, with i


def test_s10_not_flanked_by_two_segments_or_past_an_end():
    """R5a: two segments either side is not flanked, and nor is past the end."""
    g = wall_span(0, 12)
    two = s10_side(g, 'B_kp',
                   lambda r, c, d: None if c == 5 else (0 if c < 5 else 1))
    i = int(np.nonzero((two['el']['col'] == 5) & (two['el']['dir'] == 3))[0][0])
    out = dw.is_flanked(two['el'], two['seg_of'], i)
    assert out['flanked'] is False
    assert out['segments_on_face_line'] == [0, 1]    # both are there, and
    assert out['flanking_segments'] == []            # neither is on both sides

    beyond = s10_side(g, 'B_kp', lambda r, c, d: 0 if c <= 4 else None)
    j = int(np.nonzero((beyond['el']['col'] == 7)
                       & (beyond['el']['dir'] == 3))[0][0])
    assert dw.is_flanked(beyond['el'], beyond['seg_of'], j)['flanked'] is False


def test_s10_not_flanked_off_the_face_line_and_the_tolerance_is_inclusive():
    """R5b: "the same across coordinate, within 1e-9 m", inclusive at 1e-9.

    ACROSS is the projection on the element's own normal, which for a '-y'
    element is -y, so a pair moved in y is a pair off the line. At exactly the
    tolerance the pair still flanks; at twice it, it does not.
    """
    tol = dw.FACE_LINE_TOL_M
    seg_of = np.array([0, -1, 0], dtype=np.int64)
    on = face_line_el([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)])
    assert dw.is_flanked(on, seg_of, 1)['flanked'] is True

    at_tol = face_line_el([(0.0, tol), (0.5, 0.0), (1.0, -tol)])
    assert dw.is_flanked(at_tol, seg_of, 1)['flanked'] is True

    past = face_line_el([(0.0, 2 * tol), (0.5, 0.0), (1.0, -2 * tol)])
    out = dw.is_flanked(past, seg_of, 1)
    assert out['flanked'] is False
    assert out['n_on_face_line'] == 1            # only the element itself
    # one side on the line is still not two sides
    half = face_line_el([(0.0, 0.0), (0.5, 0.0), (1.0, -2 * tol)])
    assert dw.is_flanked(half, seg_of, 1)['flanked'] is False


def _r_rows(spec):
    """Loss rows from `(B, k, kp, e_prime, seg_b, shape, A, cut_A)` tuples."""
    return [{'A': a, 'B': b, 'cut_A': ca, 'step': f'cut{k}->cut{kp}',
             'element': 0, 'b_index': 0, 'b_dir': 3, 'e_prime': e,
             's7_bin': dw.BIN_FAR if shape == dw.SHAPE_FAR
             else dw.BIN_WITHIN_EPS, 'k': k, 'kp': kp, 'seg_b': s,
             'shape': shape}
            for (b, k, kp, e, s, shape, a, ca) in spec]


def test_s10_repeated_rows_of_one_event_count_once():
    """R7: S9's rows repeat an event per A pairing; the unit is the event."""
    rows = _r_rows([(1, 60, 120, 40, 0, dw.SHAPE_HOLE, a, ca)
                    for a in (0, 2) for ca in (60, 120, 240)])
    assert len(rows) == 6
    events = dw.distinct_groups(rows, dw.EVENT_FIELDS)
    assert len(events) == 1
    assert events[0]['key'] == [1, 60, 120, 40]
    assert len(events[0]['rows']) == 6               # the rows are kept
    units = dw.distinct_groups(rows, dw.UNIT_FIELDS)
    assert len(units) == 1 and units[0]['key'] == [1, 60, 120, 0]
    # a second e' on the same segment is a second event and the same unit
    rows += _r_rows([(1, 60, 120, 41, 0, dw.SHAPE_HOLE, 0, 60)])
    assert len(dw.distinct_groups(rows, dw.EVENT_FIELDS)) == 2
    assert len(dw.distinct_groups(rows, dw.UNIT_FIELDS)) == 1


def _r_units(rows, changed_keys):
    """The units `rows` make, each with the state the case wants."""
    out = []
    for u in dw.distinct_groups(rows, dw.UNIT_FIELDS):
        key = tuple(u['key'])
        state = dw.CHANGED if key in changed_keys else dw.INTACT
        out.append({'key': list(key),
                    'shapes': [s for s in dw.S10_SHAPES
                               if any(r['shape'] == s for r in u['rows'])],
                    'state': state, 'changed': state == dw.CHANGED,
                    'n_losses': len(u['rows'])})
    return out


def test_s10_an_s_b_behind_both_shapes_counts_in_both():
    """R8: it is in both denominators, and the dedupe is load-bearing.

    Two segments: S_b 0 carries three one-cell holes (one per A pairing) and
    one carve-out loss, and is changed; S_b 1 carries one hole and is intact.
    Deduped, P13b is 1 intact of 2 and holds. Counted per loss -- the
    mutation -- the shared S_b weighs three times and P13b fails, which is
    what makes this test catch the dedupe going away.
    """
    rows = _r_rows(
        [(1, 60, 120, 40, 0, dw.SHAPE_HOLE, a, 60) for a in (0, 2, 3)]
        + [(1, 60, 120, 41, 0, dw.SHAPE_FAR, 0, 60),
           (1, 60, 120, 50, 1, dw.SHAPE_HOLE, 0, 60)])
    units = _r_units(rows, changed_keys={(1, 60, 120, 0)})
    assert len(units) == 2
    assert units[0]['shapes'] == [dw.SHAPE_HOLE, dw.SHAPE_FAR]

    p13a = dw.p13_verdict(units, dw.SHAPE_FAR, dw.CHANGED)
    p13b = dw.p13_verdict(units, dw.SHAPE_HOLE, dw.INTACT)
    assert (p13a['of'], p13a['n'], p13a['holds']) == (1, 1, True)
    assert (p13b['of'], p13b['n'], p13b['holds']) == (2, 1, True)
    assert [1, 60, 120, 0] in p13a['units'] and [1, 60, 120, 0] in p13b['units']

    # MUTATION: one unit per loss row instead of per distinct S_b
    per_loss = [{'key': [r['B'], r['k'], r['kp'], r['seg_b']],
                 'shapes': [r['shape']], 'state': u['state'],
                 'changed': u['changed'], 'n_losses': 1}
                for r in rows
                for u in units if u['key'][3] == r['seg_b']]
    mutated = dw.p13_verdict(per_loss, dw.SHAPE_HOLE, dw.INTACT)
    assert (mutated['of'], mutated['n']) == (4, 1)
    assert mutated['holds'] is False, 'the dedupe is what makes P13b hold here'


@pytest.mark.parametrize('n,want', [(0, 0), (1, 1), (2, 1), (3, 2), (4, 2),
                                    (21, 11), (22, 11), (75, 38), (54, 27)])
def test_s10_at_least_half_is_ceil(n, want):
    """R9: ceil(n/2) at odd and even n -- at an odd n, the larger half."""
    assert dw.at_least_half(n) == want
    assert dw.at_least_half(n) >= n / 2


def test_s10_the_verdict_threshold_at_an_odd_and_an_even_denominator():
    """R9b: the same rounding inside P13, and P14's own all-or-nothing form."""
    def units(n, n_intact, shape=dw.SHAPE_HOLE):
        return [{'key': [1, 60, 120, i], 'shapes': [shape],
                 'state': dw.INTACT if i < n_intact else dw.CHANGED,
                 'changed': i >= n_intact} for i in range(n)]

    assert dw.p13_verdict(units(3, 2), dw.SHAPE_HOLE, dw.INTACT)['holds']
    assert dw.p13_verdict(units(3, 1), dw.SHAPE_HOLE,
                          dw.INTACT)['holds'] is False
    assert dw.p13_verdict(units(3, 1), dw.SHAPE_HOLE,
                          dw.INTACT)['min_required'] == 2
    assert dw.p13_verdict(units(4, 2), dw.SHAPE_HOLE, dw.INTACT)['holds']
    assert dw.p13_verdict(units(4, 1), dw.SHAPE_HOLE,
                          dw.INTACT)['holds'] is False
    with pytest.raises(AssertionError):                 # 0 of 0 never "holds"
        dw.p13_verdict(units(2, 2, dw.SHAPE_FAR), dw.SHAPE_HOLE, dw.INTACT)

    events = [{'key': [1, 60, 120, i], 'shape': dw.SHAPE_HOLE,
               'flanked': False, 'flanking_segments': []} for i in range(5)]
    out = dw.p14_verdict(events, n_expected=5)
    assert (out['holds'], out['n_flanked'], out['of']) == (True, 0, 5)
    events[2].update(flanked=True, flanking_segments=[7])
    out = dw.p14_verdict(events, n_expected=5)
    assert (out['holds'], out['n_flanked']) == (False, 1)
    assert out['flanked'][0]['flanking_segments'] == [7]
    with pytest.raises(AssertionError):                 # 22 is registered
        dw.p14_verdict(events)


def test_s10_the_base_rate_on_a_toy_map():
    """R10: every segment of B at k, not only the S_b behind a loss."""
    g_k = blank(12, S7_GRID_W)
    g_k[:7, :] = FREE
    g_k[6, 0:3] = OCC                                  # segment 0
    g_k[6, 8:11] = OCC                                 # segment 1
    side_k = s10_side(g_k, 'B_k1_cut60',
                      lambda r, c, d: 0 if c < 4 else 1)
    # 4 and 5: each run's three '-y' faces, plus the '+x' face at its right
    # end, plus the '-x' face at the second run's left end
    assert [s['n_elements'] for s in side_k['segments']] == [4, 5]

    g_kp = g_k.copy()
    g_kp[6, 9] = FREE                                  # one member of 1 goes
    side_kp = s10_side(g_kp, 'B_k1_cut120',
                       lambda r, c, d: 0 if c < 4 else 1)

    rates = dw.base_rate({(1, 60): side_k, (1, 120): side_kp},
                         [(1, 60, 120)])
    assert len(rates) == 1
    r = rates[0]
    assert (r['n_segments'], r['n_changed']) == (2, 1)
    assert r['share_changed'] == pytest.approx(0.5)
    assert r['step'] == 'cut60->cut120'
    assert r['by_route'] == {dw.OFF_GRID: 0, dw.WALL_FREED: 1,
                             dw.FRONT_CLOSED: 0}
    assert set(r['by_route']) == {dw.OFF_GRID, dw.WALL_FREED, dw.FRONT_CLOSED}


# ── the end-to-end synthetic corpus ─────────────────────────────────────────
#
# Two robots, four cuts, and two loss steps -- one of each shape, with the
# state P13 predicts for it:
#
#   A (k0), every cut    row 6 OCC cols 0..12, all assigned
#   B (k1) at cut60      the same, all assigned        -> A corroborated
#   B (k1) at cut120     row 6 and row 2 OCC, and ONLY row 2 assigned
#   B (k1) at cut240     row 6 OCC cols 0..12, all assigned again
#   B (k1) at cut1200    row 6 OCC cols 1..12, NOTHING assigned
#
# At 60 -> 120 every one of A's 14 elements loses its corroborator; b's two
# cells still read occupied and free, so the loss is unchanged; e' is the
# row-6 element, which is unassigned -- segmentation. The assigned row-2 line
# sits 0.40 m away, so these are inside a metre: the ONE-CELL HOLE shape. S_b
# is B's segment 0 at cut60, whose every member still has its two cells at
# cut120: INTACT, which is P13b's prediction.
#
# At 240 -> 1200 the same losses recur, with nothing assigned at k' -- so
# nothing within a metre, the CARVE-OUT shape -- and column 0's wall cell is
# now free. A's own column-0 element is therefore a freed wall cell and not an
# unchanged loss; the other 13 are unchanged, and their S_b has lost a member:
# CHANGED, which is P13a's prediction.
#
# Each loss recurs once per A cut, four times, which is what the distinct
# counting has to collapse: 108 rows, 27 events, 2 segment units.

S10_N_ROWS, S10_N_EVENTS = 108, 27
S10_N_HOLE_ROWS, S10_N_FAR_ROWS = 56, 52
S10_N_HOLE_EVENTS, S10_N_FAR_EVENTS = 14, 13


def s10_sides():
    """The corpus above, as a `by_kc` and a spawn table."""
    a_grid = two_runs({6: (0, 12)})
    by_kc, spawn = {}, {0: (0.0, 0.0), 1: (0.0, 0.0)}
    for c in dw.CUTS:
        by_kc[(0, c)] = s10_side(a_grid, f'A_k0_cut{c}',
                                 lambda r, c_, d: 0)
    by_kc[(1, 60)] = s10_side(two_runs({6: (0, 12)}), 'B_k1_cut60',
                              lambda r, c_, d: 0)
    by_kc[(1, 120)] = s10_side(two_runs({2: (0, 12), 6: (0, 12)}),
                               'B_k1_cut120',
                               lambda r, c_, d: 0 if r == 2 else None)
    by_kc[(1, 240)] = s10_side(two_runs({6: (0, 12)}), 'B_k1_cut240',
                               lambda r, c_, d: 0)
    by_kc[(1, 1200)] = s10_side(two_runs({6: (1, 12)}), 'B_k1_cut1200',
                                lambda r, c_, d: None)
    return by_kc, spawn


def s10_fake_s9(by_kc, spawn, n_robots=2):
    """An S9 payload for the synthetic corpus, built with S9's own code."""
    rho = next(iter(by_kc.values()))['rho']
    rows = dw.recompute_s9_rows(by_kc, spawn, rho, dw.CUTS, n_robots)
    return {'elements': rows, 'rho_m': rho, 'head': 'synthetic',
            'reported': {'class_by_bin': [
                {'class': c, 's7_bin': b,
                 'n': sum(1 for r in rows
                          if r['class'] == c and r['s7_bin'] == b)}
                for c in dw.P12_CLASSES for b in dw.S7_BINS]}}


def _s10_counts(s9):
    """The counts this corpus produced, as `s10_segments`'s own arguments."""
    seg = [r for r in s9['elements'] if r['class'] == dw.SEGMENTATION]
    far = [r for r in seg if r['s7_bin'] == dw.BIN_FAR]
    hole = [r for r in seg if r['s7_bin'] != dw.BIN_FAR]

    def n_distinct(rows):
        return len({(r['B'], r['step'], r['e_prime']) for r in rows})

    return {'n_unchanged': len(s9['elements']),
            'n_segmentation': len(seg), 'n_hole': len(hole),
            'n_far': len(far), 'n_events': n_distinct(seg),
            'n_events_hole': n_distinct(hole),
            'n_events_far': n_distinct(far)}


def test_s10_the_corpus_has_the_shape_the_cases_assume():
    """R11a: the counts above, before any of them is a gate's denominator."""
    by_kc, spawn = s10_sides()
    assert by_kc[(1, 60)]['el']['n'] == 14            # 13 '-y' and one '+x'
    assert by_kc[(1, 60)]['segments'][0]['n_elements'] == 14
    assert by_kc[(1, 1200)]['segments'] == []         # nothing assigned at k'
    s9 = s10_fake_s9(by_kc, spawn)
    counts = _s10_counts(s9)
    assert {r['class'] for r in s9['elements']} == {dw.SEGMENTATION}
    assert counts == {'n_unchanged': S10_N_ROWS,
                      'n_segmentation': S10_N_ROWS,
                      'n_hole': S10_N_HOLE_ROWS, 'n_far': S10_N_FAR_ROWS,
                      'n_events': S10_N_EVENTS,
                      'n_events_hole': S10_N_HOLE_EVENTS,
                      'n_events_far': S10_N_FAR_EVENTS}


def _s10_spies(monkeypatch):
    """Spies on the two functions that compute an unseen quantity."""
    seen = {'segment_change': 0, 'is_flanked': 0}
    real_change, real_flank = dw.segment_change, dw.is_flanked

    def change(*a, **k):
        seen['segment_change'] += 1
        return real_change(*a, **k)

    def flank(*a, **k):
        seen['is_flanked'] += 1
        return real_flank(*a, **k)

    monkeypatch.setattr(dw, 'segment_change', change)
    monkeypatch.setattr(dw, 'is_flanked', flank)
    return seen


def test_s10_g8_holding_judges_all_three_predictions(tmp_path, monkeypatch):
    """R11: the whole pipeline, both shapes, and a file.

    The corpus is built to reach one unit of each shape with the state its
    prediction names, so a verdict that read the wrong unit or the wrong state
    would flip here rather than merely move a count.

    The spies are the POSITIVE CONTROL for R12's zeroes: with G8 holding,
    `segment_change` runs twice for the two units and twice more for the base
    rate's two steps -- not 108 times, once per loss -- and `is_flanked` once
    per distinct e'.
    """
    by_kc, spawn = s10_sides()
    s9 = s10_fake_s9(by_kc, spawn)
    seen = _s10_spies(monkeypatch)
    out = dw.s10_segments(by_kc, spawn, s9,
                          {'stage': 's10', 'head': 'synthetic'}, tmp_path,
                          n_robots=2, **_s10_counts(s9))
    assert seen == {'segment_change': 2 + 2, 'is_flanked': S10_N_EVENTS}
    assert out['G8']['all_ok'] and all(out['G8']['parts'].values())
    assert out['G8']['1_s9_rows_reproduce']['counts_equal']
    assert out['G8']['1_s9_rows_reproduce']['class_by_bin_equal']
    assert out['G8']['2_b_assigned_e_prime_unassigned']['n_ok'] == S10_N_ROWS
    assert out['G8']['3_segment_holds_b']['n_ok'] == S10_N_ROWS

    assert (out['n_losses'], out['n_events']) == (S10_N_ROWS, S10_N_EVENTS)
    assert out['n_segment_units'] == 2
    by_key = {tuple(u['key']): u for u in out['segment_units']}
    hole, far = by_key[(1, 60, 120, 0)], by_key[(1, 240, 1200, 0)]
    assert hole['shapes'] == [dw.SHAPE_HOLE]
    assert (hole['state'], hole['routes']) == (dw.INTACT, [])
    assert (hole['n_members'], hole['n_losses']) == (14, S10_N_HOLE_ROWS)
    assert far['shapes'] == [dw.SHAPE_FAR]
    assert (far['state'], far['routes']) == (dw.CHANGED, [dw.WALL_FREED])
    assert (far['n_members'], far['n_changed_members']) == (14, 1)

    assert (out['P13a']['of'], out['P13a']['n']) == (1, 1)
    assert out['P13a']['holds'] and out['P13a']['min_required'] == 1
    assert (out['P13b']['of'], out['P13b']['n']) == (1, 1)
    assert out['P13b']['holds']
    assert (out['P14']['of'], out['P14']['n_flanked']) == (S10_N_EVENTS, 0)
    assert out['P14']['holds']
    assert out['P14']['by_shape'][dw.SHAPE_FAR]['of'] == S10_N_FAR_EVENTS

    # reported, not tested
    per_loss = out['reported']['per_loss']
    assert per_loss['of'] == S10_N_ROWS
    assert per_loss['by_shape'][dw.SHAPE_HOLE][dw.INTACT] == S10_N_HOLE_ROWS
    assert per_loss['by_shape'][dw.SHAPE_FAR][dw.CHANGED] == S10_N_FAR_ROWS
    assert len(out['reported']['changed_losses']) == S10_N_FAR_EVENTS
    # the one changed member is column 0's '-y' element, at x = 0, and the
    # along distance to it is each e''s own x: columns 1 to 12 for the '-y'
    # elements and 1.25 m for the '+x' face at the right-hand end
    along = sorted(e['nearest_changed_member']['along_m']
                   for e in out['reported']['changed_losses'])
    assert along == pytest.approx([RHO * c for c in range(1, 13)] + [1.25])
    for e in out['reported']['changed_losses']:
        near = e['nearest_changed_member']
        assert (near['route'], near['member']) == (dw.WALL_FREED, 1)
        assert e['routes'] == [dw.WALL_FREED] and e['n_changed_members'] == 1
    assert {r['step'] for r in out['reported']['base_rate']} == {
        'cut60->cut120', 'cut240->cut1200'}
    assert [r['share_changed'] for r in out['reported']['base_rate']] == [
        0.0, 1.0]
    assert len(out['reported']['segment_sizes']) == 2

    written = list(tmp_path.glob('s10_segments_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['losses']) == S10_N_ROWS
    assert len(payload['events']) == S10_N_EVENTS
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def _assert_s10_fail_file(tmp_path, capsys):
    """The fail file holds the G8 comparisons and no unseen quantity."""
    assert not list(tmp_path.glob('s10_segments_*.json'))
    written = list(tmp_path.glob('s10_g8_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert set(payload) >= {'G8', 'stage', 'seconds'}
    for key in ('events', 'segment_units', 'losses', 'reported',
                'P13a', 'P13b', 'P14'):
        assert key not in payload

    def keys_of(node):
        if isinstance(node, dict):
            return set(node) | set().union(*(keys_of(v) for v in node.values())
                                           or [set()])
        if isinstance(node, list):
            return set().union(*(keys_of(v) for v in node) or [set()])
        return set()

    assert not (keys_of(payload) & {'changed', 'intact', 'flanked', 'state',
                                    'routes', 'n_changed_members',
                                    'base_rate', 'share_changed', 'events',
                                    'segment_units', 'P13a', 'P13b', 'P14'})
    out = capsys.readouterr().out
    assert 'G8' in out and 'FAIL' in out
    for word in ('P13a', 'P13b', 'P14', 'changed', 'intact', 'flanked',
                 'base rate', 'distinct S_b'):
        assert word not in out
    return payload


@pytest.mark.parametrize('check', ['1_s9_rows_reproduce',
                                   '2_b_assigned_e_prime_unassigned',
                                   '3_segment_holds_b'])
def test_s10_a_failure_at_any_g8_check_computes_nothing(check, tmp_path,
                                                        capsys, monkeypatch):
    """R12: exit 2, and no changed, intact or flanked anywhere.

    CHECKS 2 AND 3 ARE DRIVEN BY A PATCH OR A MUTATED SIDE, deliberately: on a
    corpus S9 would accept neither can arise -- an assigned e' at k' would
    have been corroborated, and `seg_of` and the segment list come out of one
    pass of `extract_with_elements` -- so the only way to exercise the
    handling is to make one of them answer as it would if that broke.
    """
    by_kc, spawn = s10_sides()
    s9 = s10_fake_s9(by_kc, spawn)
    counts = _s10_counts(s9)

    if check == '1_s9_rows_reproduce':
        # one stored class changed: the rows no longer reproduce
        next(r for r in s9['elements']
             if r['s7_bin'] == dw.BIN_FAR)['class'] = dw.GEOM_ACROSS
    elif check == '2_b_assigned_e_prime_unassigned':
        # e' pointed at an ASSIGNED element of B at k', after check 1 held
        real = dw.loss_rows

        def tampered(stored):
            rows = real(stored)
            kp = by_kc[(1, 120)]
            assigned = int(np.nonzero(kp['seg_of'] >= 0)[0][0])
            for r in rows:
                if r['kp'] == 120:
                    r['e_prime'] = assigned
                    break
            return rows

        monkeypatch.setattr(dw, 'loss_rows', tampered)
    else:
        # the segment list disagrees with `seg_of` about its own membership
        by_kc[(1, 60)]['segments'][0]['n_elements'] += 1

    seen = _s10_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        dw.s10_segments(by_kc, spawn, s9,
                        {'stage': 's10', 'head': 'synthetic'}, tmp_path,
                        n_robots=2, **counts)
    assert e.value.code == 2
    assert seen == {'segment_change': 0, 'is_flanked': 0}
    payload = _assert_s10_fail_file(tmp_path, capsys)
    assert payload['G8']['parts'][check] is False
    later = [p for p in ('1_s9_rows_reproduce',
                         '2_b_assigned_e_prime_unassigned',
                         '3_segment_holds_b')
             if p > check]
    for p in later:                          # a failed check stops the chain
        assert payload['G8'][p] is None


def _s10_refusal_spies(monkeypatch):
    """Spies on everything `run_s10` could read: the corpus and the S9 file."""
    seen = {'load_side': 0, 'sha256': 0}
    real = dw._sha256

    def load_side(*a, **k):
        seen['load_side'] += 1
        raise AssertionError('run_s10 loaded a map past a refusal')

    def sha256(path):
        seen['sha256'] += 1
        return real(path)

    monkeypatch.setattr(dw, 'load_side', load_side)
    monkeypatch.setattr(dw, '_sha256', sha256)
    return seen


def test_s10_refuses_a_dirty_tree(monkeypatch):
    """R13a: a result file must not name a commit it was not produced by."""
    seen = _s10_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s10()
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'sha256': 0}


def test_s10_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """R13b: P13a, P13b and P14 must be registered behind the code."""
    seen = _s10_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] in ('status', 'merge-base') else 'deadbeef'))
    with pytest.raises(SystemExit) as e:
        dw.run_s10()
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'sha256': 0}


@pytest.mark.parametrize('present', ['', 'P13a', 'P13a+P13b'])
def test_s10_refuses_when_the_spec_at_head_lacks_a_registration(present,
                                                                monkeypatch):
    """R13c: all three headers, not one of them -- each is judged separately."""
    seen = _s10_refusal_spies(monkeypatch)
    text = '## 9\n\n**P12. REGISTERED 9 Oct 2026.** something\n'
    for name in [p for p in present.split('+') if p]:
        text += f'\n**{name}. REGISTERED 10 Oct 2026.** something\n'

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return text
        return 'c0ffee'                       # merge-base == rev-parse

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s10()
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'sha256': 0}


def test_s10_refuses_a_wrong_s9_sha256(tmp_path, monkeypatch):
    """R13d: G8 check 1 is an equality against ONE S9 run, so it is pinned."""
    seen = _s10_refusal_spies(monkeypatch)

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return ('## Segments behind the unassigned elements\n\n'
                    '**P13a. REGISTERED 10 Oct 2026.** Changed.\n\n'
                    '**P13b. REGISTERED 10 Oct 2026.** Intact.\n\n'
                    '**P14. REGISTERED 10 Oct 2026.** None flanked.\n')
        return 'c0ffee'

    monkeypatch.setattr(dw, '_git', fake)
    (tmp_path / dw.S9_RESULT).write_text('{"elements": [], "rho_m": 0.1}')
    with pytest.raises(SystemExit) as e:
        dw.run_s10(tmp_path)
    assert e.value.code == 2
    assert seen['load_side'] == 0            # the pin fires before the corpus
    assert seen['sha256'] == 1               # and it did hash the named file


def test_s10_the_unchanged_recompute_is_the_one_s9_committed():
    """R14: the extraction behind G8 check 1 is S9's part 1, not a copy.

    `recompute_unchanged` was lifted out of `s9_segmentation`; S10 reproduces
    S9's rows through it, so S9 has to be the caller of it rather than hold a
    second copy of those statements.
    """
    import inspect
    src = inspect.getsource(dw.s9_segmentation)
    assert 'recompute_unchanged(' in src
    for gone in ('s7_mask(by_kc', 'gain_loss_series(masks',
                 'g6_states_at_k(by_kc'):
        assert gone not in src
    doc = dw.recompute_unchanged.__doc__
    assert 'pure extraction' in doc


# ─────── S11: the rule that drops each e' (the T series) ────────────────────
#
# SYNTHETIC ONLY. Nothing here traces or classifies an element of a real map,
# and nothing reads experiments/logs/.

def hand_el(points, dirs=3):
    """Elements placed by hand: `(x, y)` each, with one direction or one each.

    DIRS[3] is '-y', normal (0, -1): such an element faces the line at theta
    180 and sits at d = -y on it, with along = -x. DIRS[0] is '+x', normal
    (1, 0), which is at right angles to that line and does not face it.
    """
    n = len(points)
    d = np.full(n, dirs, dtype=np.int64) if np.isscalar(dirs) else \
        np.array(dirs, dtype=np.int64)
    return {'n': n,
            'x': np.array([p[0] for p in points], float),
            'y': np.array([p[1] for p in points], float),
            'nx': np.array([gw.DIRS[k][2] for k in d], float),
            'ny': np.array([gw.DIRS[k][3] for k in d], float),
            'dir': d, 'row': np.zeros(n, dtype=np.int64),
            'col': np.arange(n, dtype=np.int64)}


def hand_trace(el, d_peak, theta_line, d_line, theta_peak=180.0):
    """One iteration, replayed through the tracer's own calls on chosen lines.

    The loop body's own steps, in its order and through its functions --
    `line_candidates` twice, the sort along the refitted line, `split_runs`,
    the l_min test -- with the first-pass line and the refit GIVEN rather than
    found, so a case can put an element a stated distance from each.
    """
    p = gw.Params()
    eps = p.eps(RHO)
    hough = gw.Hough(el, RHO, p)
    alive = np.ones(el['n'], dtype=bool)
    tracer = gw.Tracer()
    tracer.begin(el, hough, alive, p, RHO, eps, p.l_min / (math.sqrt(2) * RHO))
    j = int(np.nonzero(hough.thetas == theta_peak)[0][0])
    in_window = alive & hough.vote[:, j]
    cand_first = gw.line_candidates(el, alive, theta_peak, d_peak, eps, True)
    cand = gw.line_candidates(el, alive, theta_line, d_line, eps, True)
    ux, uy = gw.dir_of(theta_line)
    t_all = el['x'] * ux + el['y'] * uy
    idx = np.nonzero(cand)[0]
    idx = idx[np.argsort(t_all[idx], kind='stable')]
    t_sorted = t_all[idx]
    runs = gw.split_runs(t_sorted, RHO, p.g)
    accepted = [(lo, hi, idx[lo:hi]) for lo, hi in runs
                if float(t_sorted[hi - 1] - t_sorted[lo] + RHO) >= p.l_min]
    tracer.iteration(
        gw.TRACE_ACCEPTED if accepted else gw.TRACE_NO_ACCEPTED_RUN, j, 0,
        int(in_window.sum()), alive, in_window, d_peak=d_peak,
        cand_first=cand_first, theta_line=theta_line, d_line=d_line,
        degenerate=False, cand=cand, idx=idx, t_sorted=t_sorted, runs=runs,
        accepted=accepted)
    seg_of = np.full(el['n'], -1, dtype=np.int64)
    if accepted:
        ids = {lo: k for k, (lo, _hi, _m) in enumerate(accepted)}
        for lo, _hi, m in accepted:
            seg_of[m] = ids[lo]
        tracer.removed(hough, ids)
    tracer.end(hough, seg_of, 1)
    return tracer


def wall_points(cols, y=0.0):
    return [(RHO * c, y) for c in cols]


def at_home(within=True, faces=True, candidate=True, accepted=False,
            run=True):
    """The facts `classify_drop` reads at H, stated outright."""
    return {'within_eps_of_refit': within, 'faces_refit': faces,
            'candidate': candidate,
            'run': {'accepted': accepted} if run else None}


def test_s11_every_class_and_the_first_that_applies_wins():
    """T1: No home, R3, R4, R5+R6 and Other, in the registered order."""
    # no home is decided before anything at H is read at all
    assert dw.classify_drop(None, None) == dw.NO_HOME
    assert dw.classify_drop(None, at_home(within=False)) == dw.NO_HOME

    assert dw.classify_drop(0, at_home(within=False, candidate=False,
                                       run=False)) == dw.R3_LINE
    assert dw.classify_drop(0, at_home(faces=False, candidate=False,
                                       run=False)) == dw.R4_FACING
    assert dw.classify_drop(0, at_home()) == dw.R56_SHORT_RUN
    # a candidate in an ACCEPTED run would have been assigned: Other
    assert dw.classify_drop(0, at_home(accepted=True)) == dw.OTHER
    # within eps, facing, and yet not a candidate: Other too
    assert dw.classify_drop(0, at_home(candidate=False,
                                       run=False)) == dw.OTHER

    # the first that applies wins: R3 before R4, R4 before R5+R6
    assert dw.classify_drop(0, at_home(within=False, faces=False,
                                       candidate=False,
                                       run=False)) == dw.R3_LINE
    assert dw.classify_drop(0, at_home(within=False)) == dw.R3_LINE
    assert dw.classify_drop(0, at_home(faces=False)) == dw.R4_FACING
    # H = 0 is an iteration, not "no home"
    assert dw.classify_drop(0, at_home()) != dw.NO_HOME
    assert set(dw.DROP_CLASSES) == {dw.NO_HOME, dw.R3_LINE, dw.R4_FACING,
                                    dw.R56_SHORT_RUN, dw.OTHER}


def test_s11_r3_is_read_against_the_refit_not_the_first_pass_line():
    """T2: outside eps of the first-pass line, inside eps of the refit.

    A six-cell wall at y = 0 and one more '-y' element 0.25 m below it. The
    first-pass line is the wall itself, d = 0, and the element is 0.25 m off
    it, past eps = 0.1707 m. The refit sits at d = 0.12, so the element is
    0.13 m off THAT and a candidate: not R3. It is two metres along from the
    wall, alone in its run, so the run is short: R5+R6.

    And the mirror: inside eps of the first-pass line, outside eps of the
    refit, is R3.
    """
    el = hand_el(wall_points(range(6)) + [(2.6, -0.25)])
    e = 6
    tr = hand_trace(el, d_peak=0.0, theta_line=180.0, d_line=0.12)
    at = dw.element_at(tr, 0, e)
    assert at['dist_first_pass_m'] == pytest.approx(0.25)
    assert at['dist_first_pass_m'] > EPS and not at['candidate_first_pass']
    assert at['dist_refit_m'] == pytest.approx(0.13)
    assert at['within_eps_of_refit'] and at['candidate']
    assert at['run']['accepted'] is False and at['run']['n_elements'] == 1
    assert dw.classify_drop(0, at) == dw.R56_SHORT_RUN

    tr = hand_trace(el, d_peak=0.12, theta_line=180.0, d_line=0.0)
    at = dw.element_at(tr, 0, e)
    assert at['dist_first_pass_m'] == pytest.approx(0.13)
    assert at['candidate_first_pass'] is True
    assert at['dist_refit_m'] == pytest.approx(0.25)
    assert not at['within_eps_of_refit'] and at['run'] is None
    assert dw.classify_drop(0, at) == dw.R3_LINE


def test_s11_the_class_uses_the_iterations_refit_not_the_reported_line(
        monkeypatch):
    """T3: the refit and `_segment`'s reported line differ, and R3 follows
    the refit. Mutation-checked.

    A six-cell wall at y = 0, and e' 0.10 m to its far side. The iteration
    cuts on a refit at d = 0.10: the wall is 0.10 m off it, within eps, and
    its run is accepted; e' is 0.20 m off it, beyond eps, and is R3.
    `_segment` then refits the accepted run to its own six elements and
    reports the wall itself, d = 0 -- a line e' is only 0.10 m from.

    THE MUTATION: reading the reported line where the refit belongs puts e'
    within eps, and the class stops being R3.
    """
    el = hand_el(wall_points(range(6)) + [(0.25, 0.10)])
    e = 6
    tr = hand_trace(el, d_peak=0.0, theta_line=180.0, d_line=0.10)
    seg = gw._segment(blank(4, 8, FREE), RHO, ORIGIN, el,
                      tr.iterations[0]['members'][0], 0, 180.0, 0.10,
                      gw.Params().b(RHO), gw.Params())
    assert seg['line_source'] == 'own_tls'
    reported_d = float(el['x'][0] * seg['normal'][0]
                       + el['y'][0] * seg['normal'][1])
    assert reported_d == pytest.approx(0.0, abs=1e-9)       # the wall itself
    assert dw.refit_line_of(tr.iterations[0]) == (180.0, 0.10)

    at = dw.element_at(tr, 0, e)
    assert at['dist_refit_m'] == pytest.approx(0.20) and at['faces_refit']
    assert dw.classify_drop(0, at) == dw.R3_LINE

    monkeypatch.setattr(dw, 'refit_line_of',
                        lambda rec: (seg['theta'], reported_d))
    mutated = dw.element_at(tr, 0, e)
    assert mutated['dist_refit_m'] == pytest.approx(0.10)
    assert mutated['within_eps_of_refit']
    assert dw.classify_drop(0, mutated) != dw.R3_LINE, \
        'the reported line must not be what R3 is read against'


def test_s11_r4_is_the_facing_test_alone():
    """T4: on the line and not facing it is R4; off it as well is R3."""
    el = hand_el(wall_points(range(6)) + [(0.25, 0.0), (0.25, -0.30)],
                 dirs=[3] * 6 + [0, 0])
    tr = hand_trace(el, d_peak=0.0, theta_line=180.0, d_line=0.0)
    on_line = dw.element_at(tr, 0, 6)
    assert on_line['within_eps_of_refit'] and not on_line['faces_refit']
    assert on_line['facing_dot_refit'] == pytest.approx(0.0, abs=1e-12)
    assert on_line['candidate'] is False and on_line['run'] is None
    assert dw.classify_drop(0, on_line) == dw.R4_FACING
    off_line = dw.element_at(tr, 0, 7)
    assert not off_line['within_eps_of_refit'] and not off_line['faces_refit']
    assert dw.classify_drop(0, off_line) == dw.R3_LINE


def test_s11_home_is_the_iteration_with_most_counterparts_ties_earliest():
    """T5: most counterparts wins, ties go to the earliest, none is no home."""
    def cps(*js):
        return [{'member': m, 'counterpart': j} for m, j in enumerate(js)]

    #                    element:  0  1  2  3  4  5   6
    assigned_in = np.array([7, 7, 3, 3, 3, -1, 9], dtype=np.int64)
    h = dw.home_iteration(assigned_in, cps(0, 1, 2, 3, 4, 5))
    assert h['H'] == 3 and h['n_counterparts_assigned_by_H'] == 3
    assert h['counterparts_by_iteration'] == [[3, 3], [7, 2]]
    assert (h['n_members'], h['n_counterparts'],
            h['n_counterparts_assigned']) == (6, 6, 5)

    # a tie, 2 and 2: the earlier iteration, whichever is met first
    assert dw.home_iteration(assigned_in, cps(0, 1, 2, 3))['H'] == 3
    assert dw.home_iteration(assigned_in, cps(2, 3, 0, 1))['H'] == 3
    # a later iteration with more beats an earlier one with fewer
    assert dw.home_iteration(assigned_in, cps(0, 1, 2))['H'] == 7
    # a member with no counterpart, and an unassigned counterpart, count
    # for nothing
    h = dw.home_iteration(assigned_in, cps(None, 5, 6))
    assert h['H'] == 9 and h['n_counterparts'] == 2
    # no counterpart assigned at all: no home
    for none in (cps(5), cps(None, None), []):
        assert dw.home_iteration(assigned_in, none)['H'] is None
    assert dw.classify_drop(dw.home_iteration(assigned_in, cps(5))['H'],
                            None) == dw.NO_HOME


def s11_side(grid, stem, params=None):
    """A side straight from the real extractor, so a trace of it is its own."""
    graph, el, seg_of = gw.extract_with_elements(
        grid, RHO, ORIGIN, gw.Params() if params is None else params)
    return {'stem': stem, 'el': el, 'seg_of': seg_of, 'grid': grid,
            'rho': RHO, 'origin': ORIGIN, 'shape': grid.shape,
            'segments': graph['segments']}


def split_wall():
    """Row 6, cols 0..3 and 8..12, with cols 4..7 freed: 0.4 m and 0.5 m."""
    g = two_runs({6: (0, 12)})
    g[6, 4:8] = FREE
    return g


def test_s11_counterparts_are_found_by_position_and_direction():
    """T6: a member's counterpart is the element in ITS cell, facing ITS way."""
    side_k = s11_side(two_runs({6: (0, 12)}), 'B_k')
    assert side_k['segments'][0]['n_elements'] == 14       # 13 '-y', one '+x'
    side_kp = s11_side(split_wall(), 'B_kp')
    cps = dw.counterparts(side_k, side_kp, 0)
    assert len(cps) == 14
    for c in cps:
        m, j = c['member'], c['counterpart']
        col, d = int(side_k['el']['col'][m]), int(side_k['el']['dir'][m])
        if d == 3 and 4 <= col <= 7:
            assert j is None                       # the wall cell is gone
        else:
            assert int(side_kp['el']['col'][j]) == col
            assert int(side_kp['el']['row'][j]) == 6
            assert int(side_kp['el']['dir'][j]) == d
    # the same wall on a lattice five metres to the right: every cell off grid
    away = dict(side_kp, origin=(5.0, 0.0))
    assert all(c['counterpart'] is None
               for c in dw.counterparts(side_k, away, 0))
    # a half-cell lattice shift still lands each centre in one cell
    shifted = dict(side_kp, origin=(0.04, 0.03))
    assert sum(c['counterpart'] is not None
               for c in dw.counterparts(side_k, shifted, 0)) == 10


def test_s11_the_counterpart_lookup_never_calls_trinary_map_sample(
        monkeypatch):
    """T7: a spy on `sample` that raises, over the counterpart lookup."""
    side_k = s11_side(two_runs({6: (0, 12)}), 'B_k')
    side_kp = s11_side(split_wall(), 'B_kp')
    tm = importlib.import_module('trinary_map')
    calls = []

    def boom(*a, **k):
        calls.append(a)
        raise AssertionError('trinary_map.sample was called for a lookup')

    monkeypatch.setattr(tm, 'sample', boom)
    monkeypatch.setattr(gw, 'sample', boom)
    assert not hasattr(dw, 'sample')
    index_calls = []
    real_index = dw.cell_index
    monkeypatch.setattr(dw, 'cell_index', lambda *a: (
        index_calls.append(a), real_index(*a))[1])

    cps = dw.counterparts(side_k, side_kp, 0)
    assert sum(c['counterpart'] is not None for c in cps) == 10
    assert calls == []
    assert len(index_calls) == 2 * len(cps)        # x and y, once per member


def _classed(*classes):
    return [{'key': [1, 60, 120, i], 'class': c}
            for i, c in enumerate(classes)]


@pytest.mark.parametrize('n_r3,holds', [(0, False), (10, False), (11, True),
                                        (12, True), (22, True)])
def test_s11_p15_holds_from_11_of_22(n_r3, holds):
    """T8: "at least 11 of the 22 distinct e' are in R3", at its edge."""
    ev = _classed(*([dw.R3_LINE] * n_r3
                    + [dw.R56_SHORT_RUN] * (22 - n_r3)))
    out = dw.p15_verdict(ev)
    assert (out['holds'], out['n_R3'], out['of']) == (holds, n_r3, 22)
    assert out['min_required'] == 11 == dw.P15_MIN_R3
    assert sum(out['by_class'].values()) == 22
    with pytest.raises(AssertionError):                  # 22 is registered
        dw.p15_verdict(ev[:21])


@pytest.mark.parametrize('n_other,holds', [(0, True), (1, False),
                                           (22, False)])
def test_s11_p16_holds_only_at_0_of_22(n_other, holds):
    """T9: none in Other; and R4 and No home are not Other."""
    ev = _classed(*([dw.OTHER] * n_other + [dw.R3_LINE] * (22 - n_other)))
    out = dw.p16_verdict(ev)
    assert (out['holds'], out['n_other'], out['of']) == (holds, n_other, 22)
    assert len(out['other']) == n_other
    with pytest.raises(AssertionError):
        dw.p16_verdict(ev[:21])

    spared = _classed(*([dw.R4_FACING] * 5 + [dw.NO_HOME] * 6
                        + [dw.R56_SHORT_RUN] * 11))
    out = dw.p16_verdict(spared)
    assert out['holds'] and out['n_R4'] == 5
    assert dw.p15_verdict(spared)['holds'] is False      # judged separately


def test_s11_exactly_one_class_is_a_count_and_a_label():
    """T10: G9 check 3's second half."""
    ok = [dw.R3_LINE, dw.NO_HOME, dw.OTHER]
    assert dw.g9_one_class(ok, 3)['ok']
    assert not dw.g9_one_class(ok, 4)['ok']               # one e' short
    assert not dw.g9_one_class(ok + [None], 4)['ok']      # one with no class
    assert not dw.g9_one_class(ok + ['R3'], 4)['ok']      # not a label


def test_s11_the_line_quantities_at_an_iteration():
    """T11: the angle to the wall's axis, and candidates past the wall's span.

    A refit at 183 degrees over a '-y' wall is 3 degrees off its axis, and
    over a '+x' face 87. With the span set to the first three cells, the
    other three candidates lie beyond it, 0.3 m past its end.
    """
    el = hand_el(wall_points(range(6)) + [(0.25, 0.0)], dirs=[3] * 6 + [0])
    tr = hand_trace(el, d_peak=0.0, theta_line=183.0, d_line=0.0)
    span = slice(0, 3)
    q = dw.line_quantities(tr, 0, 0, el['x'][span], el['y'][span])
    assert q['angle_to_face_axis_deg'] == pytest.approx(3.0)
    assert q['theta_refit_deg'] == 183.0 and q['theta_first_pass_deg'] == 180.0
    assert q['span']['n'] == 3
    assert q['first_pass_candidates']['n'] == 6
    assert q['first_pass_candidates']['n_beyond_span'] == 3
    assert max(q['first_pass_candidates']['overhang_below_m'],
               q['first_pass_candidates']['overhang_above_m']
               ) == pytest.approx(0.3 * math.cos(math.radians(3.0)))
    assert dw.line_quantities(tr, 0, 6, el['x'][span], el['y'][span])[
        'angle_to_face_axis_deg'] == pytest.approx(87.0)
    whole = dw.line_quantities(tr, 0, 0, el['x'][:6], el['y'][:6])
    assert whole['first_pass_candidates']['n_beyond_span'] == 0


def test_s11_a_run_cut_where_the_refit_thinned_the_candidates():
    """T12: R5+R6, and the gap that cut the run is R3's doing.

    Cols 0..2 and 6..12 of a wall at y = 0, and cols 3..5 at y = +0.15. The
    first-pass line, d = 0, takes all thirteen: the three are 0.15 m off it.
    The refit, d = 0.05, leaves them 0.20 m off and drops them, which opens a
    0.3 m gap -- past g -- and strands cols 0..2 as a 0.3 m run.
    """
    el = hand_el(wall_points(range(0, 3)) + wall_points(range(3, 6), 0.15)
                 + wall_points(range(6, 13)))
    tr = hand_trace(el, d_peak=0.0, theta_line=180.0, d_line=0.05)
    rec = tr.iterations[0]
    assert rec['cand_first'].all() and not rec['cand'][3:6].any()
    at = dw.element_at(tr, 0, 1)
    assert dw.classify_drop(0, at) == dw.R56_SHORT_RUN
    assert at['run']['length_m'] == pytest.approx(0.3)
    assert dw.classify_drop(0, dw.element_at(tr, 0, 4)) == dw.R3_LINE

    gaps = {g['end']: g for g in dw.run_gaps(tr, 0, 1)}
    # along is -x on a line at 180 degrees, so the far run is "below"
    cut = gaps['below']
    assert gaps['above']['neighbour'] is None
    assert cut['neighbour'] == 6
    assert cut['gap_m'] == pytest.approx(0.3) and cut['gap_m'] >= cut['g_m']
    assert cut['n_first_pass_candidates_beyond_eps_of_refit'] == 3
    assert cut['thinned_by_r3'] is True

    # the same gap with nothing in it is a gap in the map, not R3's
    bare = hand_el(wall_points(range(0, 3)) + wall_points(range(6, 13)))
    tr = hand_trace(bare, d_peak=0.0, theta_line=180.0, d_line=0.05)
    cut = {g['end']: g for g in dw.run_gaps(tr, 0, 1)}['below']
    assert cut['gap_m'] == pytest.approx(0.3)
    assert cut['thinned_by_r3'] is False
    assert dw.run_gaps(tr, 0, 3) == [] or dw.element_at(tr, 0, 3)['run']


def test_s11_the_nearest_assigned_same_direction_element():
    """T13: nearest by distance, ties to the lowest index, and its face line."""
    el = hand_el([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (0.5, 0.3), (0.5, -0.1)],
                 dirs=[3, 3, 3, 3, 0])
    seg_of = np.array([0, -1, 0, -1, 2], dtype=np.int64)
    near = dw.nearest_assigned_same_dir(el, seg_of, 1)
    assert (near['index'], near['segment']) == (0, 0)     # 0 and 2 tie at 0.5
    assert near['dist_m'] == pytest.approx(0.5)
    assert near['on_e_prime_face_line'] is True
    # the '+x' element 0.1 m away is nearer and is not the same direction
    off = dw.nearest_assigned_same_dir(el, np.array([-1, -1, -1, 1, 2]), 1)
    assert off['index'] == 3 and off['on_e_prime_face_line'] is False
    assert dw.nearest_assigned_same_dir(
        el, np.full(5, -1, dtype=np.int64), 1) is None


# ── G9 check 2, and what a tracer that touches its arguments does to it ─────

def two_line_side():
    """Two walls, so the pass accepts in two iterations and order matters."""
    return s11_side(two_runs({2: (0, 12), 6: (0, 12)}), 'B_k1_cut120')


class _KillsAliveAtBegin(gw.Tracer):
    def begin(self, el, hough, alive, *a):
        super().begin(el, hough, alive, *a)
        alive[:] = False


class _KillsAliveInAnIteration(gw.Tracer):
    def iteration(self, exit_, j, bin_idx, votes, alive, in_window, **kw):
        super().iteration(exit_, j, bin_idx, votes, alive, in_window, **kw)
        alive[:] = False


class _DropsAnAcceptedRun(gw.Tracer):
    def iteration(self, exit_, j, bin_idx, votes, alive, in_window, **kw):
        super().iteration(exit_, j, bin_idx, votes, alive, in_window, **kw)
        if kw.get('accepted'):
            del kw['accepted'][:]


class _RewritesTheMembers(gw.Tracer):
    def iteration(self, exit_, j, bin_idx, votes, alive, in_window, **kw):
        super().iteration(exit_, j, bin_idx, votes, alive, in_window, **kw)
        if kw.get('idx') is not None:
            kw['idx'][:] = kw['idx'][0]         # the runs' members are views


class _EmptiesTheAccumulator(gw.Tracer):
    def removed(self, hough, run_seg_id):
        super().removed(hough, run_seg_id)
        hough.A[:] = 0


def test_s11_g9_check_2_holds_on_an_honest_tracer():
    """T14a: off equals on, and the off run is the loaded side."""
    side = two_line_side()
    assert len(side['segments']) >= 2
    assert len({s['theta'] for s in side['segments']}) >= 2    # two lines
    by_kc = {(1, 120): side}
    c2 = dw.g9_non_interference(by_kc, {(1, 120): dw.trace_grid(side)})
    assert c2['ok'] and c2['n'] == c2['n_ok'] == 1
    row = c2['rows'][0]
    assert row['seg_of_equal'] and row['segments_equal']
    assert row['whole_graph_equal'] and row['off_is_the_loaded_side']
    assert row['trace_self_check']['ok']
    assert not dw.g9_non_interference(by_kc, {})['ok']     # nothing traced


@pytest.mark.parametrize('tracer', [_KillsAliveAtBegin,
                                    _KillsAliveInAnIteration,
                                    _DropsAnAcceptedRun, _RewritesTheMembers,
                                    _EmptiesTheAccumulator])
def test_s11_a_tracer_that_alters_what_it_is_handed_fails_g9_check_2(tracer):
    """T14b: MUTATION. Each of these writes to one argument and nothing else.

    `alive`, before the loop and inside it; the list of accepted runs; the
    index array the runs' members are views of; and the accumulator after a
    removal. Every one changes `seg_of` or the segment list, and check 2 --
    the registered comparison, not the trace's self-check -- says so.
    """
    side = two_line_side()
    t = dw.trace_grid(side, tracer_factory=tracer)
    c2 = dw.g9_non_interference({(1, 120): side}, {(1, 120): t})
    assert not c2['ok']
    row = c2['rows'][0]
    assert not (row['seg_of_equal'] and row['segments_equal'])
    assert row['off_is_the_loaded_side']         # the off run is untouched
    assert c2['not_ok'] == [row]


def test_s11_g9_check_2_needs_the_off_run_to_be_the_loaded_side():
    """T14c: a trace of another extraction is not a trace of this side."""
    side = two_line_side()
    t = dw.trace_grid(side, params=gw.Params(l_min=2.0))   # nothing is long
    c2 = dw.g9_non_interference({(1, 120): side}, {(1, 120): t})
    row = c2['rows'][0]
    assert row['seg_of_equal'] and row['segments_equal']   # off == on, and
    assert not row['off_is_the_loaded_side'] and not c2['ok']   # still no


# ── the end-to-end synthetic corpus ─────────────────────────────────────────
#
# Two robots, four cuts, EVERY side straight from the real extractor, so that
# the trace of a map is the trace of the run that made its `seg_of`:
#
#   A (k0), every cut    row 6 OCC cols 0..12, one segment of 14
#   B (k1) at cut60      the same                       -> A corroborated
#   B (k1) at cut120     cols 0..3 and 8..12, cols 4..7 freed
#   B (k1) at cut240     row 6 OCC cols 0..12 again
#   B (k1) at cut1200    cols 0..3 only
#
# At 60 -> 120 the extractor's one accepting iteration takes cols 8..12 (0.5
# m) and leaves cols 0..3 (0.4 m, short of l_min) unassigned. Their cells are
# unchanged, so those four are segmentation losses, inside a metre of an
# assigned element. H is that iteration -- it assigned six counterparts of
# S_b -- and at H each e' is on the line, facing it, a candidate, in the
# rejected run: R5+R6.
#
# At 240 -> 1200 nothing is long enough and nothing is assigned: the same
# four cells are losses with no assigned element within a metre, no
# counterpart of S_b is assigned, and each e' has NO HOME.

S11_N_EVENTS = 8


def s11_sides():
    full = two_runs({6: (0, 12)})
    by_kc, spawn = {}, {0: (0.0, 0.0), 1: (0.0, 0.0)}
    for c in dw.CUTS:
        by_kc[(0, c)] = s11_side(full, f'A_k0_cut{c}')
    by_kc[(1, 60)] = s11_side(full, 'B_k1_cut60')
    by_kc[(1, 120)] = s11_side(split_wall(), 'B_k1_cut120')
    by_kc[(1, 240)] = s11_side(full, 'B_k1_cut240')
    by_kc[(1, 1200)] = s11_side(two_runs({6: (0, 3)}), 'B_k1_cut1200')
    return by_kc, spawn


def s11_fake_s10(by_kc, spawn, tmp_path, capsys):
    """An S10 result for the corpus, written and read back as S10's file is."""
    s9 = s10_fake_s9(by_kc, spawn)
    out = dw.s10_segments(by_kc, spawn, s9, {'stage': 's10',
                                             'head': 'synthetic'},
                          tmp_path / 's10', n_robots=2, **_s10_counts(s9))
    capsys.readouterr()                         # S10's own prints, dropped
    return json.loads(open(out['written']['path']).read())


def _s11_spies(monkeypatch):
    """Spies on everything that reads a class or a step towards one."""
    seen = {'counterparts': 0, 'home_iteration': 0, 'element_at': 0,
            'classify_drop': 0}

    def spy(name):
        real = getattr(dw, name)

        def wrapped(*a, **k):
            seen[name] += 1
            return real(*a, **k)

        monkeypatch.setattr(dw, name, wrapped)

    for name in list(seen):
        spy(name)
    return seen


def _s11_run(by_kc, spawn, s10, out_dir, **kw):
    kw.setdefault('trace_of', dw.trace_grid)
    return dw.s11_trace(by_kc, spawn, s10, {'stage': 's11',
                                            'head': 'synthetic'}, out_dir,
                        n_robots=2, n_events=S11_N_EVENTS, **kw)


def test_s11_g9_holding_classifies_every_e_prime(tmp_path, capsys,
                                                 monkeypatch):
    """T15: the whole pipeline on real extractions, both classes, one file."""
    by_kc, spawn = s11_sides()
    s10 = s11_fake_s10(by_kc, spawn, tmp_path, capsys)
    assert s10['n_events'] == S11_N_EVENTS
    assert {e['shape'] for e in s10['events']} == set(dw.S10_SHAPES)

    seen = _s11_spies(monkeypatch)
    traced = []
    out = _s11_run(by_kc, spawn, s10, tmp_path / 's11', min_r3=4,
                   trace_of=lambda side: (traced.append(side['stem']),
                                          dw.trace_grid(side))[1])
    # each map of each step once: B at k' and at k, never A
    assert sorted(traced) == ['B_k1_cut120', 'B_k1_cut1200', 'B_k1_cut240',
                              'B_k1_cut60']
    assert out['n_maps_traced'] == 4
    # the positive control for T16's zeroes: one class per distinct e', and
    # counterparts once per distinct S_b
    assert seen['classify_drop'] == S11_N_EVENTS
    assert seen['home_iteration'] == S11_N_EVENTS
    assert seen['counterparts'] == 2

    g9 = out['G9']
    assert g9['all_ok'] and all(g9['parts'].values())
    assert g9['1_s10_reproduces']['sections_differing'] == []
    assert [(r['section'], r['equal'])
            for r in g9['1_s10_reproduces']['sections_compared']] == [
        (name, True) for name in dw.S10_COMPARED]
    assert g9['2_non_interference']['n_ok'] == 4
    c3 = g9['3_e_prime_unassigned_in_one_class']
    assert c3['unassigned']['n_ok'] == S11_N_EVENTS and c3['one_class']['ok']

    by_step = {}
    for e in out['events']:
        by_step.setdefault(e['step'], []).append(e)
    hole, far = by_step['cut60->cut120'], by_step['cut240->cut1200']
    assert len(hole) == len(far) == 4

    for e in hole:
        assert e['class'] == dw.R56_SHORT_RUN and e['shape'] == dw.SHAPE_HOLE
        assert e['home']['H'] == 0
        assert (e['home']['n_members'], e['home']['n_counterparts'],
                e['home']['n_counterparts_assigned_by_H']) == (14, 10, 6)
        at = e['at_home']
        assert at['voted_in_seed_window'] and at['candidate']
        assert at['dist_refit_m'] <= e['eps_m'] and at['faces_refit']
        assert at['run']['accepted'] is False
        assert at['run']['length_m'] < e['l_min_m'] == 0.5
        # the gap that cut the run is the freed cells, not R3
        cut = [g for g in e['run_gaps'] if g['neighbour'] is not None]
        assert len(cut) == 1 and cut[0]['gap_m'] >= cut[0]['g_m']
        assert cut[0]['thinned_by_r3'] is False
        # after H its windows went on being seeded, and each run was short
        after = e['after_home']
        assert after['R1_never_in_a_seeded_window_again'] is False
        assert (after['later_by_outcome'][dw.LATER_SHORT_RUN]
                == after['n_later_seeded_windows'] > 0)
        assert after['later_accepting'] == []
        w = after['windows']
        assert w['exhausted_after'] == after['n_later_seeded_windows']
        assert sum(v for k, v in w.items() if k != 'n_windows') == w[
            'n_windows']
        assert e['no_home'] is None
        assert e['nearest_assigned_same_dir']['on_e_prime_face_line'] is True
        assert e['line_at_home']['angle_to_face_axis_deg'] < 1.0
        assert e['line_at_home']['first_pass_candidates'][
            'n_beyond_span'] == 0
        # the registered record of e' at H is in the file's trace
        assert [r['iteration'] for r in e['trace']] == [0]
        assert e['trace'][0] == at
        assert (e['trace_tally']['n_iterations']
                == sum(e['trace_tally']['iterations_by_exit'].values()))

    for e in far:
        assert e['class'] == dw.NO_HOME and e['shape'] == dw.SHAPE_FAR
        assert e['home']['H'] is None and e['at_home'] is None
        assert e['line_at_home'] is None and e['after_home'] is None
        assert e['run_gaps'] is None
        nh = e['no_home']
        assert (nh['n_counterparts'], nh['n_without_counterpart']) == (4, 10)
        assert (nh['n_R1_never_in_a_seeded_window'],
                nh['n_R2_seed_bins_spent']) == (0, 4)
        assert e['nearest_assigned_same_dir'] is None
        assert e['trace'] == []                 # no H, no accepting iteration

    # b at k, in every event: assigned by iteration 0, a candidate of it
    for e in out['events']:
        k = e['line_at_k']
        assert k['iteration'] == 0 and k['b']['run']['accepted'] is True
        assert k['b']['dist_refit_m'] <= k['eps_m']
        assert k['span']['n'] == 14

    assert (out['P15']['n_R3'], out['P15']['of']) == (0, S11_N_EVENTS)
    assert out['P15']['holds'] is False and out['P15']['min_required'] == 4
    assert out['P15']['by_class'] == {
        dw.NO_HOME: 4, dw.R3_LINE: 0, dw.R4_FACING: 0, dw.R56_SHORT_RUN: 4,
        dw.OTHER: 0}
    assert out['P16']['holds'] and out['P16']['n_other'] == 0

    rep = out['reported']
    assert rep['classes_by_shape'][dw.SHAPE_HOLE][dw.R56_SHORT_RUN] == 4
    assert rep['classes_by_shape'][dw.SHAPE_FAR][dw.NO_HOME] == 4
    assert rep['classes_by_state'][dw.CHANGED] == out['P15']['by_class']
    assert sum(rep['classes_by_state'][dw.INTACT].values()) == 0
    per = rep['per_distinct_s_b']
    assert (per['n_units'], per['n_units_with_two_classes']) == (2, 0)
    assert per['n_units_by_class'][dw.R56_SHORT_RUN] == 1
    assert per['n_units_by_class'][dw.NO_HOME] == 1

    printed = capsys.readouterr().out
    assert 'G9: check 1 HOLD  check 2 HOLD  check 3 HOLD   HOLD' in printed
    assert 'P15: FAIL  0/8' in printed and 'P16: HOLD  0/8' in printed

    # one new dated S11 file, and no second S10 file anywhere
    written = list((tmp_path / 's11').glob('*.json'))
    assert [p.name[:10] for p in written] == ['s11_trace_']
    assert len(list((tmp_path / 's10').glob('*.json'))) == 1
    payload = json.loads(written[0].read_text())
    assert len(payload['events']) == S11_N_EVENTS
    assert payload['G9']['all_ok'] is True
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()


def _assert_s11_fail_file(out_dir, capsys):
    """The fail file holds the G9 comparisons and no class; nor does stdout."""
    assert not list(out_dir.glob('s11_trace_*.json'))
    written = list(out_dir.glob('s11_g9_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert set(payload) >= {'G9', 'stage', 'seconds'}

    def keys_of(node):
        if isinstance(node, dict):
            return set(node) | set().union(*(keys_of(v) for v in node.values())
                                           or [set()])
        if isinstance(node, list):
            return set().union(*(keys_of(v) for v in node) or [set()])
        return set()

    assert not (keys_of(payload) & {'class', 'classes', 'H', 'home', 'at_home',
                                    'events', 'P15', 'P16', 'by_class',
                                    'reported', 'trace', 'after_home',
                                    'no_home', 'run_gaps'})
    text = written[0].read_text()
    out = capsys.readouterr().out
    assert 'G9' in out and 'FAIL' in out
    for word in ('P15', 'P16', 'no home') + tuple(dw.DROP_CLASSES):
        assert word not in out, word
    for word in dw.DROP_CLASSES:
        assert f'"{word}"' not in text, word
    return payload


@pytest.mark.parametrize('check', ['1_s10_reproduces', '2_non_interference',
                                   '3_e_prime_unassigned_in_one_class'])
def test_s11_a_failure_at_any_g9_check_classifies_nothing(check, tmp_path,
                                                          capsys, monkeypatch):
    """T16: exit 2, and no counterpart, no H and no class anywhere.

    CHECK 3 IS DRIVEN BY A PATCH, deliberately: with checks 1 and 2 holding,
    an e' assigned in the traced run cannot arise -- check 1 reproduced G8's
    "e' unassigned at k'" on the loaded side, and check 2 held the traced run
    equal to it -- so the tracer is made to answer as it would if that broke.
    """
    by_kc, spawn = s11_sides()
    s10 = s11_fake_s10(by_kc, spawn, tmp_path, capsys)
    trace_of, traced = dw.trace_grid, []

    if check == '1_s10_reproduces':
        # one stored state flipped: S10 no longer reproduces
        s10['events'][0]['state'] = dw.INTACT
    elif check == '2_non_interference':
        # at `begin`, because this corpus's maps accept in one iteration at
        # most and a change made after it would reach no output
        def trace_of(side):                               # noqa: F811
            return dw.trace_grid(side, tracer_factory=_KillsAliveAtBegin)
    else:
        e_prime = s10['events'][0]['e_prime']

        def trace_of(side):                               # noqa: F811
            t = dw.trace_grid(side)
            if side['stem'] == 'B_k1_cut120':
                said = t['tracer'].assigned_in()
                said[e_prime] = 0
                t['tracer'].assigned_in = lambda: said
            return t

    def counting(side):
        traced.append(side['stem'])
        return trace_of(side)

    seen = _s11_spies(monkeypatch)
    out_dir = tmp_path / 's11'
    with pytest.raises(SystemExit) as e:
        _s11_run(by_kc, spawn, s10, out_dir, trace_of=counting)
    assert e.value.code == 2
    assert seen == {'counterparts': 0, 'home_iteration': 0, 'element_at': 0,
                    'classify_drop': 0}
    payload = _assert_s11_fail_file(out_dir, capsys)
    parts = payload['G9']['parts']
    assert parts[check] is False
    order = ['1_s10_reproduces', '2_non_interference',
             '3_e_prime_unassigned_in_one_class']
    for earlier in order[:order.index(check)]:
        assert parts[earlier] is True
    if check == '1_s10_reproduces':
        assert traced == []                     # no map was traced at all
        assert payload['G9']['2_non_interference'] is None
        assert payload['G9']['1_s10_reproduces']['sections_differing'] == [
            'events']
    if check != '3_e_prime_unassigned_in_one_class':
        assert payload['G9']['3_e_prime_unassigned_in_one_class'][
            'unassigned'] is None


def test_s11_an_e_prime_outside_the_classes_prints_and_writes_none(
        tmp_path, capsys, monkeypatch):
    """T17: check 3's second half fails AFTER classifying, and shows nothing."""
    by_kc, spawn = s11_sides()
    s10 = s11_fake_s10(by_kc, spawn, tmp_path, capsys)
    real = dw.classify_drop
    calls = []

    def sometimes_nothing(home, at):
        calls.append(home)
        return None if len(calls) == 3 else real(home, at)

    monkeypatch.setattr(dw, 'classify_drop', sometimes_nothing)
    out_dir = tmp_path / 's11'
    with pytest.raises(SystemExit) as e:
        _s11_run(by_kc, spawn, s10, out_dir)
    assert e.value.code == 2
    assert len(calls) == S11_N_EVENTS
    payload = _assert_s11_fail_file(out_dir, capsys)
    c3 = payload['G9']['3_e_prime_unassigned_in_one_class']
    assert c3['unassigned']['ok'] is True
    assert c3['one_class'] == {'n': 8, 'n_expected': 8,
                               'n_not_a_registered_label': 1, 'ok': False}
    assert payload['G9']['parts'] == {
        '1_s10_reproduces': True, '2_non_interference': True,
        '3_e_prime_unassigned_in_one_class': False}


def test_s11_check_1_ignores_row_order_and_nothing_else(tmp_path, capsys):
    """T18: S10's rows in another order reproduce; one value moved does not."""
    by_kc, spawn = s11_sides()
    s10 = s11_fake_s10(by_kc, spawn, tmp_path, capsys)
    check, derived = dw.g9_s10_reproduces(by_kc, spawn, s10, n_robots=2,
                                          n_events=S11_N_EVENTS)
    assert check['ok'] and derived is not None
    assert check['wrote_an_s10_file'] is False

    shuffled = json.loads(json.dumps(s10))
    shuffled['losses'].reverse()
    shuffled['events'][0]['losses'].reverse()
    shuffled['G8']['2_b_assigned_e_prime_unassigned']['rows'].reverse()
    assert dw.g9_s10_reproduces(by_kc, spawn, shuffled, n_robots=2,
                                n_events=S11_N_EVENTS)[0]['ok']

    for path, value in ((('P13a', 'n'), 99), (('losses', 0, 'seg_b'), 7),
                        (('reported', 'base_rate', 0, 'n_changed'), 99),
                        (('segment_units', 0, 'length_m'), math.nextafter(
                            s10['segment_units'][0]['length_m'], math.inf))):
        moved = json.loads(json.dumps(s10))
        node = moved
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
        check = dw.g9_s10_reproduces(by_kc, spawn, moved, n_robots=2,
                                     n_events=S11_N_EVENTS)[0]
        assert not check['ok'] and check['sections_differing'] == [path[0]]

    # a row dropped is a row missing, whatever the order
    short = json.loads(json.dumps(s10))
    short['losses'].pop()
    assert not dw.g9_s10_reproduces(by_kc, spawn, short, n_robots=2,
                                    n_events=S11_N_EVENTS)[0]['ok']
    # the S9 hash S10's file names is part of check 1 when one is expected
    named = json.loads(json.dumps(s10))
    named['sources'] = {'s9': {'sha256': 'abc'}}
    assert dw.g9_s10_reproduces(by_kc, spawn, named, n_robots=2,
                                n_events=S11_N_EVENTS, s9_sha='abc')[0]['ok']
    assert not dw.g9_s10_reproduces(by_kc, spawn, named, n_robots=2,
                                    n_events=S11_N_EVENTS,
                                    s9_sha=dw.S9_SHA256)[0]['ok']


def test_s11_reproduces_s10_through_s10s_own_statements():
    """T19: `s10_derive` is what S10 runs, not a copy S11 keeps beside it."""
    import inspect
    s10_src = inspect.getsource(dw.s10_segments)
    assert 's10_derive(' in s10_src
    for gone in ('segment_change(', 'is_flanked(', 'base_rate(',
                 'p13_verdict(', 'p14_verdict('):
        assert gone not in s10_src
    g9_src = inspect.getsource(dw.g9_s10_reproduces)
    for used in ('recompute_s9_rows(', 'loss_rows(', 'g8_assignments(',
                 'g8_segments(', 's10_derive('):
        assert used in g9_src
    for src in (g9_src, inspect.getsource(dw.s11_trace),
                inspect.getsource(dw.s10_derive)):
        assert '_write_s10(' not in src and 's10_segments(' not in src


def _s11_refusal_spies(monkeypatch):
    """Spies on everything `run_s11` could read: the corpus and the S10 file."""
    seen = {'load_side': 0, 'trace_stem': 0, 'sha256': 0}
    real = dw._sha256

    def load_side(*a, **k):
        seen['load_side'] += 1
        raise AssertionError('run_s11 loaded a map past a refusal')

    def trace_stem(*a, **k):
        seen['trace_stem'] += 1
        raise AssertionError('run_s11 traced a map past a refusal')

    def sha256(path):
        seen['sha256'] += 1
        return real(path)

    monkeypatch.setattr(dw, 'load_side', load_side)
    monkeypatch.setattr(dw, 'trace_stem', trace_stem)
    monkeypatch.setattr(dw, '_sha256', sha256)
    return seen


S11_SPEC_TEXT = ('## The rule that drops each e\'\n\n'
                 '**P15. REGISTERED 11 Oct 2026.** At least 11.\n\n'
                 '**P16. REGISTERED 11 Oct 2026.** None in Other.\n')


def test_s11_refuses_a_dirty_tree(monkeypatch):
    """T20a: a result file must not name a commit it was not produced by."""
    seen = _s11_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/graph_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s11()
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'trace_stem': 0, 'sha256': 0}


def test_s11_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """T20b: dc754de registered P15 and P16 and must be behind the code."""
    seen = _s11_refusal_spies(monkeypatch)
    asked = []

    def fake(*a):
        asked.append(a)
        return '' if a[0] in ('status', 'merge-base') else 'deadbeef'

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s11()
    assert e.value.code == 2
    assert ('merge-base', 'dc754de', 'HEAD') in asked
    assert dw.S11_REGISTRATION == 'dc754de'
    assert seen == {'load_side': 0, 'trace_stem': 0, 'sha256': 0}


@pytest.mark.parametrize('present', ['', 'P15', 'P16'])
def test_s11_refuses_when_the_spec_at_head_lacks_a_registration(present,
                                                                monkeypatch):
    """T20c: both headers, not one of them -- each is judged separately."""
    seen = _s11_refusal_spies(monkeypatch)
    text = '## 9\n\n**P14. REGISTERED 10 Oct 2026.** something\n'
    if present:
        text += f'\n**{present}. REGISTERED 11 Oct 2026.** something\n'

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return text
        return 'c0ffee'                       # merge-base == rev-parse

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s11()
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'trace_stem': 0, 'sha256': 0}


def test_s11_refuses_a_missing_or_wrong_s10_file(tmp_path, monkeypatch):
    """T20d: G9 check 1 is an equality against ONE S10 run, so it is pinned."""
    seen = _s11_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] == 'status' else S11_SPEC_TEXT if a[0] == 'show'
        else 'c0ffee'))
    with pytest.raises(SystemExit) as e:
        dw.run_s11(tmp_path)                    # no S10 file there at all
    assert e.value.code == 2
    assert seen == {'load_side': 0, 'trace_stem': 0, 'sha256': 0}

    (tmp_path / dw.S10_RESULT).write_text('{"events": [], "rho_m": 0.1}')
    with pytest.raises(SystemExit) as e:
        dw.run_s11(tmp_path)
    assert e.value.code == 2
    assert seen['load_side'] == 0 and seen['trace_stem'] == 0
    assert seen['sha256'] == 1               # and it did hash the named file
    assert dw.S10_RESULT == 's10_segments_20261011T134922Z.json'
    assert dw.S10_SHA256 == ('fa750aa97fc08c5a1f177b78019832c3c5794ef42a8898d9'
                             'd8f6d1d7127407e2')


# ───── S12: the viewpoint weight at mixed cuts, G10 and P17 (the Y series) ───
#
# SYNTHETIC ONLY. No weight is computed on a real map anywhere in this file,
# at any pair of cuts, and nothing here reads S5's or S6's result file.
#
#   Y1   the mirror enumeration                    60, 60 and 40; M60 and
#                                                  M1200 disjoint; every row
#                                                  one direction of one pair
#   Y2   6 and 7 failing pairs of the 60           54 holds P17, 53 fails it
#   Y3   d of exactly 1/180, and one ulp over      holds; fails
#   Y4   a row with no weighable element           no d; counts against P17
#   Y5   the pairs of a row                        n and length sum to the
#                                                  row's; w one number a pair
#   Y6   two weights inside one pair               exit 2
#   Y7   check 3 on two pairs of different w       holds; a pair's w moved by
#                                                  1e-9 fails it
#   Y8   the door, before G10                      S5's rows and the self
#                                                  pairs pass; any other row
#                                                  raises; a failed check
#                                                  never opens it
#   Y9   the door against S5's call                the same result, the same
#                                                  arguments
#   Y10  S5's file, location by location           1440 values over 6
#                                                  locations, on 140 rows
#   Y11  one ulp on one of S6's 320 C              exit 2; no weight call at
#                                                  all; no verdict
#   Y12  one ulp on one of S5's weights, and on    exit 2; 160 weight calls,
#        one of G3's stored values                 none on an unseen row
#   Y13  check 3 failing on a row S5 holds         exit 2; 160 weight calls
#   Y14  check 3 failing on a row S5 does not      exit 2; no verdict written
#        hold                                      or printed, and no weight
#                                                  in the fail file
#   Y15  a symmetric corpus, end to end            P17 holds 60/60; 320 rows,
#                                                  160 pairs, one file
#   Y16  one double wall, end to end               8 pairs with d not 0: 7
#                                                  over the bound and shown
#                                                  with the pair only one
#                                                  side holds, 1 inside it
#                                                  and holding; 53/60, P17
#                                                  fails
#   Y17  null_weights and segment_dirs raising     never called
#   Y18  d over a set; w from both sides           min, median, max, zeroes
#   Y19  --stage s12 on a dirty tree, without      refuses, exit 2, and loads
#        82e6462, with a spec missing P17 or G10,  nothing in each
#        and on a wrong S5 or S6 sha256
#   Y20  the registered paragraphs                 read from the spec text

S12_SPLIT = 1                # a single wall's columns from here on are seg 1
S12_DOUBLE_AT = (4, 60)      # in the double corpus, k4 at cut60 has two walls


def double_wall(w, h=12):
    """Two '-y' faces per column, 0.2 m apart: OCC at rows 4 and 6.

    Row 4 sits on FREE and under FREE, so it also yields a '+y' face; row 6
    sits on FREE and under UNKNOWN. Three elements a column, and `s12_side`
    leaves the '+y' ones unassigned.
    """
    g = blank(h, w)
    g[:4, :] = FREE
    g[4, :] = OCC
    g[5, :] = FREE
    g[6, :] = OCC
    return g


def s12_side(grid, stem, k, ci):
    """A side with two segments and a bearing histogram on each.

    A single wall is split by column at `S12_SPLIT`; a double wall is one
    segment per wall. The histograms are one bin each, placed by robot and
    cut so that no two segments in the corpus share one: w between any two is
    then a whole number of bins and differs from pair to pair.
    """
    s = fake_side(grid, stem)
    el = s['el']
    minus_y = el['dir'] == 3
    if (~minus_y).any():                       # the double wall
        seg_of = np.where(minus_y, (el['row'] == 6).astype(np.int64), -1)
    else:
        seg_of = (el['col'] >= S12_SPLIT).astype(np.int64)
    s['seg_of'] = seg_of
    s['segments'] = [{'p0': [0.0, 0.0], 'p1': [1.0, 0.0]}] * 2
    s['bearing_hist'] = [hist(2 * k + ci), hist(7 * k + 3 * ci + 20)]
    return s


def s12_sides(double=False):
    """The synthetic corpus S12's cases run on: five robots by four cuts.

    S6's growing wall, on row 5, so `C` behaves as it does there. Every map
    is lattice-aligned with every other, so T1's matching is one to one, the
    two directions of a mirror pair weigh the same elements and d is exactly
    0 on every pair.

    `double=True` gives k4 at cut60 two walls instead, 0.1 m either side of
    where the others have one, and lifts k4's spawn by 0.02 m so the nearer
    of the two is nearer by a clear 0.04 m. Another robot's wall then matches
    k4's lower wall only, while BOTH of k4's walls match that robot's one
    wall: the pair (S_X, upper) exists from k4's side alone, which is exactly
    the asymmetry P17 is about.
    """
    by_kc, spawn = {}, {}
    for k in range(dw.N_ROBOTS):
        spawn[k] = (0.0, 0.02) if (double and k == S12_DOUBLE_AT[0]) \
            else (0.0, 0.0)
        for ci, c in enumerate(dw.CUTS):
            w = S6_WIDTHS[c] + k
            grid = (double_wall(w) if double and (k, c) == S12_DOUBLE_AT
                    else wall_grid(h=12, w=w, wall_row=5))
            by_kc[(k, c)] = s12_side(
                grid, f'b2maps_k{k}_cut{c}_gated_extfix_robot{k}', k, ci)
    return by_kc, spawn


def s12_fake_s6(by_kc, spawn):
    """An S6 payload for the corpus, built with S6's own code."""
    keys = dw.matrix_keys()
    rows = dw.cut_matrix(by_kc, spawn, keys)
    return json.loads(json.dumps({'head': 'synthetic',
                                  'rows': [rows[k] for k in keys]}))


def s12_fake_s5(by_kc, spawn):
    """An S5 payload for the corpus, in the shape `run_s5` writes.

    Built with S5's own call and `_reported`, row for row as `run_s5` builds
    it, and passed through JSON as the real file is. P7's E7 fields are
    placeholders: S12 never reads them, and Y10 says so.
    """
    def rep(a, b, ca, cb):
        _res, sh = dw._side_classify(by_kc[(a, ca)], by_kc[(b, cb)],
                                     spawn[a], spawn[b])
        return dw._reported(sh)

    g3 = []
    for kc in sorted(by_kc):
        s = by_kc[kc]
        res, sh = dw._side_classify(s, s, (0.0, 0.0), (0.0, 0.0))
        w = res['weights']['w']
        good = ~np.isnan(w)
        g3.append({'map': s['stem'], 'C': sh['corroborated'],
                   'max_abs_w': float(np.abs(w[good]).max()),
                   'n_weighable': int(good.sum()), 'holds': True})
    pairs = dw.ordered_pairs()
    p7 = [{'A': a, 'B': b, 'n_E7': 0, 'mean_w': None, 'mean_w_null': None,
           'holds': False, 'reported': {**rep(a, b, 1200, 1200), 'n_E7': 0}}
          for (a, b) in pairs]
    p8 = [{'A': a, 'B': b, 'holds': True,
           'w_mean_B1200': rep(a, b, 1200, 1200)['w_mean'],
           'w_mean_B60': rep(a, b, 1200, 60)['w_mean'],
           'reported_B60': rep(a, b, 1200, 60)} for (a, b) in pairs]
    reported = {
        'A_cut1200_by_B_cut': [
            {'A': a, 'B': b, 'cut_A': 1200, 'cut_B': c, **rep(a, b, 1200, c)}
            for (a, b) in pairs for c in dw.CUTS],
        'same_cut': [
            {'A': a, 'B': b, 'cut_A': c, 'cut_B': c, **rep(a, b, c, c)}
            for (a, b) in pairs for c in dw.CUTS]}
    return json.loads(json.dumps({
        'stage': 's5', 'head': 'synthetic',
        'G3': {'rows': g3, 'holds': True, 'failures': []},
        'P7': {'rows': p7}, 'P8': {'rows': p8}, 'reported': reported}))


S12_META = {'stage': 's12', 'head': 'synthetic',
            'predictions_verbatim': {'P17': 'the registered wording',
                                     'G10': 'the registered wording'},
            'sources': {'s5': {'sha256': 'x'}, 's6': {'sha256': 'y'}}}


def _s12_run(by_kc, spawn, s5, s6, out_dir, **kw):
    return dw.s12_mirror(by_kc, spawn, s5, s6, dict(S12_META), out_dir, **kw)


def _kc_of(stem):
    m = re.match(r'b2maps_k(\d)_cut(\d+)_', stem)
    return int(m.group(1)), int(m.group(2))


def _s12_spies(monkeypatch):
    """Spies on the weight call, on what computes w, and on P7's null.

    `weight_keys` is every (A, B, cut_A, cut_B) S5's call was made on, read
    off the two sides it was handed. `null_weights` and `segment_dirs` are
    how w_null and E7 are computed, and they raise: S12 calls neither.
    """
    seen = {'weight_keys': [], 'viewpoint_weights': 0, 'cut_matrix_keys': []}
    real_sc, real_vw, real_cm = (dw._side_classify, dw.viewpoint_weights,
                                 dw.cut_matrix)

    def side_classify(a, b, spawn_a, spawn_b):
        (ka, ca), (kb, cb) = _kc_of(a['stem']), _kc_of(b['stem'])
        seen['weight_keys'].append((ka, kb, ca, cb))
        return real_sc(a, b, spawn_a, spawn_b)

    def viewpoint_weights(*a, **k):
        seen['viewpoint_weights'] += 1
        return real_vw(*a, **k)

    def cut_matrix(by_kc, spawn, keys):
        seen['cut_matrix_keys'] += list(keys)
        return real_cm(by_kc, spawn, keys)

    def never(*a, **k):
        raise AssertionError('S12 computed w_null or E7')

    monkeypatch.setattr(dw, '_side_classify', side_classify)
    monkeypatch.setattr(dw, 'viewpoint_weights', viewpoint_weights)
    monkeypatch.setattr(dw, 'cut_matrix', cut_matrix)
    monkeypatch.setattr(dw, 'null_weights', never)
    monkeypatch.setattr(dw, 'segment_dirs', never)
    return seen


def _keys_naming_the_null(node, path='$'):
    """Every path in a payload whose KEY names w_null or E7."""
    bad = []
    if isinstance(node, dict):
        for k, v in node.items():
            if 'null' in str(k).lower() or 'e7' in str(k).lower():
                bad.append(f'{path}.{k}')
            bad += _keys_naming_the_null(v, f'{path}.{k}')
    elif isinstance(node, list):
        for v in node:
            bad += _keys_naming_the_null(v, f'{path}[]')
    return bad


def _unseen_weight_keys(keys):
    return [k for k in keys if k[0] != k[1] and not dw.is_seen(k[2], k[3])]


def _assert_s12_fail_file(tmp_path, capsys):
    """One G10 fail file, no result file, and no verdict in either or printed."""
    assert not list(tmp_path.glob('s12_mirror_*.json'))
    written = list(tmp_path.glob('s12_g10_fail_*.json'))
    assert len(written) == 1
    payload = json.loads(written[0].read_text())
    assert payload['stage'] == 's12_g10_fail'
    for key in ('P17', 'rows', 'mirror_pairs', 'reported'):
        assert key not in payload
    dw._no_verdict_in_s12(payload)
    out = capsys.readouterr().out
    assert 'G10: ' in out and 'FAIL' in out
    for word in ('P17', 'M60', 'M1200', 'd by set', 'largest d'):
        assert word not in out
    return payload, out


def s12_mirrors(d=None, n_weighable=None):
    """160 hand-built mirror pairs carrying only what the verdict reads.

    `d[(X, Y, cut_X, cut_Y)]` sets a pair's d and `n_weighable[...]` its two
    counts; anything unnamed has d = 0 and weighable elements on both sides.
    """
    d, n_weighable = d or {}, n_weighable or {}
    out = []
    for x in range(dw.N_ROBOTS):
        for y in range(x + 1, dw.N_ROBOTS):
            for a in dw.CUTS:
                for b in dw.CUTS:
                    nf, nr = n_weighable.get((x, y, a, b), (3, 3))
                    dd = None if 0 in (nf, nr) else d.get((x, y, a, b), 0.0)
                    out.append({'X': x, 'Y': y, 'cut_X': a, 'cut_Y': b,
                                'set': dw.mirror_set(a, b), 'd': dd,
                                'n_weighable_X_to_Y': nf,
                                'n_weighable_Y_to_X': nr})
    return out


def _m60_keys():
    return [(m['X'], m['Y'], m['cut_X'], m['cut_Y']) for m in s12_mirrors()
            if m['set'] == dw.M60]


def test_s12_mirror_enumeration_is_60_60_and_40():
    """Y1: 160 pairs in three disjoint sets, and all 320 rows used once."""
    by_kc, spawn = s12_sides()
    rows = {k: {'w_mean': 0.1, 'n_weighable': 3, 'weighable_length_m': 0.3,
                'seen_by_S5': dw.is_seen(k[2], k[3])}
            for k in dw.matrix_keys()}
    mirrors = dw.mirror_pairs(rows)
    assert len(mirrors) == 160
    sets = collections.Counter(m['set'] for m in mirrors)
    assert (sets[dw.M60], sets[dw.M1200], sets[dw.SAME_CUT]) == (
        60, 60, 40) == (dw.N_M60, dw.N_M1200, dw.N_SAME_CUT)

    used = collections.Counter()
    for m in mirrors:
        assert m['X'] < m['Y']
        used[(m['X'], m['Y'], m['cut_X'], m['cut_Y'])] += 1
        used[(m['Y'], m['X'], m['cut_Y'], m['cut_X'])] += 1
        early = m['cut_X'] in dw.M60_CUTS and m['cut_Y'] in dw.M60_CUTS
        full = 1200 in (m['cut_X'], m['cut_Y'])
        if m['set'] == dw.M60:
            assert m['cut_X'] != m['cut_Y'] and early and not full
            # no weight was seen on either row of an M60 pair
            assert not m['seen_by_S5_X_to_Y'] and not m['seen_by_S5_Y_to_X']
        elif m['set'] == dw.M1200:
            assert m['cut_X'] != m['cut_Y'] and full and not early
            # exactly one row of an M1200 pair is S5's: the one with A at 1200
            assert m['seen_by_S5_X_to_Y'] != m['seen_by_S5_Y_to_X']
        else:
            assert m['cut_X'] == m['cut_Y']
            assert m['seen_by_S5_X_to_Y'] and m['seen_by_S5_Y_to_X']
    assert set(used) == set(rows) and set(used.values()) == {1}
    # 10 unordered robot pairs x 3 unordered cut pairs x 2 ways round
    assert len({(m['X'], m['Y']) for m in mirrors}) == 10
    assert by_kc and spawn                       # the corpus builds


@pytest.mark.parametrize('n_bad,expected', [(0, True), (6, True),
                                            (7, False), (60, False)])
def test_s12_p17_holds_at_54_of_60_and_not_at_53(n_bad, expected):
    """Y2: at least 54 of the 60 M60 pairs; 53 is a failed P17."""
    bad = _m60_keys()[:n_bad]
    out = dw.p17_verdict(s12_mirrors(d={k: 0.01 for k in bad}))
    assert (out['of'], out['min_required']) == (60, 54)
    assert out['n_holding'] == 60 - n_bad
    assert out['holds'] is expected
    assert len(out['failing']) == n_bad == out['n_over_the_bound']
    assert out['bound'] == 1.0 / 180.0


def test_s12_p17_judges_m60_and_nothing_else():
    """Y2b: a d of 1.0 on every M1200 and same-cut pair moves nothing."""
    loud = {(m['X'], m['Y'], m['cut_X'], m['cut_Y']): 1.0
            for m in s12_mirrors() if m['set'] != dw.M60}
    assert len(loud) == 100
    out = dw.p17_verdict(s12_mirrors(d=loud))
    assert (out['holds'], out['n_holding'], out['max_d']) == (True, 60, 0.0)


def test_s12_p17_the_bound_is_inclusive_and_exact():
    """Y3: d of exactly 1/180 holds; one ulp over does not; no tolerance."""
    k0, k1 = _m60_keys()[:2]
    bound = 1.0 / 180.0
    over = math.nextafter(bound, math.inf)
    out = dw.p17_verdict(s12_mirrors(d={k0: bound, k1: over}))
    assert out['n_holding'] == 59
    assert [(f['X'], f['Y'], f['cut_X'], f['cut_Y'])
            for f in out['failing']] == [k1]
    assert out['failing'][0]['reason'] == 'd is over the bound'
    assert out['max_d'] == over and out['max_d_deg'] == over * 180.0


def test_s12_p17_a_row_with_no_weighable_element_counts_against():
    """Y4: no weighable element on either side is no d, and a failure."""
    keys = _m60_keys()
    out = dw.p17_verdict(s12_mirrors(
        n_weighable={keys[0]: (0, 5), keys[1]: (5, 0), keys[2]: (0, 0)}))
    assert (out['n_holding'], out['n_undefined'], out['n_over_the_bound']) \
        == (57, 3, 0)
    assert all(f['d'] is None and 'no weighable' in f['reason']
               for f in out['failing'])
    assert out['holds']                          # 57 of 60 still holds it
    assert out['d_summary']['n_undefined'] == 3
    seven = dw.p17_verdict(s12_mirrors(
        n_weighable={k: (0, 5) for k in keys[:7]}))
    assert (seven['n_holding'], seven['holds']) == (53, False)


def _s12_one_row(by_kc, spawn, key):
    c_row = dw.s6_row(by_kc, spawn, key)
    door = dw.S12Weights(by_kc, spawn)
    door.open_after({'ok': True}, {'ok': True}, {'ok': True})
    res, sh = door.row(key)
    return dw.s12_row(key, c_row, res, sh), res, sh


@pytest.mark.parametrize('key', [(0, 1, 60, 120), (1, 0, 120, 60),
                                 (0, 4, 120, 60), (4, 0, 60, 120),
                                 (2, 3, 1200, 1200)])
def test_s12_pairs_partition_the_weighable_elements(key):
    """Y5: the pairs' n and length sum to the row's, and w is one a pair."""
    by_kc, spawn = s12_sides(double=True)
    row, res, sh = _s12_one_row(by_kc, spawn, key)
    pairs = row['pairs']
    assert pairs and row['n_pairs'] == len(pairs)
    assert sum(p['n'] for p in pairs) == row['n_weighable'] == sh['n_weighable']
    assert sum(p['length_m'] for p in pairs) == pytest.approx(
        row['weighable_length_m'])
    assert row['weighable_length_m'] == pytest.approx(RHO * row['n_weighable'])
    assert len({(p['S_A'], p['S_B']) for p in pairs}) == len(pairs)

    w, seg_b = res['weights']['w'], res['weights']['seg_b_of']
    for p in pairs:
        mine = ((res['seg_of'] == p['S_A']) & (seg_b == p['S_B'])
                & ~np.isnan(w))
        assert mine.sum() == p['n']
        assert (w[mine] == p['w']).all()         # exactly, not approximately
    for f in dw.WEIGHT_FIELDS:
        assert row[f] == sh[f]
    assert row['seen_by_S5'] is dw.is_seen(key[2], key[3])


def test_s12_two_weights_inside_one_pair_stop_the_stage():
    """Y6: Caution 2 is required, not assumed."""
    by_kc, spawn = s12_sides()
    _row, res, _sh = _s12_one_row(by_kc, spawn, (0, 1, 1200, 1200))
    assert len(dw.s12_pairs(res)) == 2
    i = int(np.nonzero(~np.isnan(res['weights']['w']))[0][-1])
    res['weights']['w'][i] = math.nextafter(res['weights']['w'][i], math.inf)
    with pytest.raises(SystemExit) as e:
        dw.s12_pairs(res)
    assert e.value.code == 2


def test_s12_check_3_holds_on_two_pairs_of_different_w():
    """Y7: the length-weighted mean of the pairs is `w_mean`, and is checked.

    k0 at cut120 against k1 at cut240: column 0 is the pair (0, 0) and
    columns 1 to 5 the pair (1, 1), with weights 3 and 10 bins apart, so the
    mean is neither pair's w and only the weighting gives it back.
    """
    by_kc, spawn = s12_sides()
    row, _res, _sh = _s12_one_row(by_kc, spawn, (0, 1, 120, 240))
    pairs = {(p['S_A'], p['S_B']): p for p in row['pairs']}
    assert set(pairs) == {(0, 0), (1, 1)}
    assert (pairs[(0, 0)]['n'], pairs[(1, 1)]['n']) == (1, 5)
    assert pairs[(0, 0)]['w'] == pytest.approx(3 * BINW / 180.0)
    assert pairs[(1, 1)]['w'] == pytest.approx(10 * BINW / 180.0)
    assert row['w_mean'] == pytest.approx((3 + 5 * 10) / 6 * BINW / 180.0)

    rec = dw.g10_pairs_mean(row)
    assert rec['ok'] and rec['abs_diff'] <= 1e-12
    assert set(rec) == {'key', 'abs_diff', 'ok'}     # and never the mean

    moved = json.loads(json.dumps(row))
    moved['pairs'][0]['w'] += 1e-9
    bad = dw.g10_pairs_mean(moved)
    assert not bad['ok'] and bad['abs_diff'] > 1e-12
    swapped = json.loads(json.dumps(row))            # lengths matter too
    swapped['pairs'][0]['length_m'], swapped['pairs'][1]['length_m'] = (
        swapped['pairs'][1]['length_m'], swapped['pairs'][0]['length_m'])
    assert not dw.g10_pairs_mean(swapped)['ok']

    over = dw.g10_pairs_means([row, moved])
    assert (over['ok'], over['n_rows'], over['n_ok']) == (False, 2, 1)
    assert over['not_ok'] == [bad]


def test_s12_check_3_on_a_row_with_no_weighable_element():
    """Y7b: no pair and no `w_mean` holds; one without the other does not."""
    base = {'A': 0, 'B': 1, 'cut_A': 60, 'cut_B': 120}
    assert dw.g10_pairs_mean({**base, 'pairs': [], 'w_mean': None})['ok']
    assert not dw.g10_pairs_mean({**base, 'pairs': [], 'w_mean': 0.1})['ok']
    pair = {'S_A': 0, 'S_B': 0, 'w': 0.1, 'n': 1, 'length_m': 0.1}
    assert not dw.g10_pairs_mean({**base, 'pairs': [pair],
                                  'w_mean': None})['ok']


def test_s12_the_door_raises_on_an_unseen_row_until_g10_passes(monkeypatch):
    """Y8: a weight on a row S5 does not hold cannot be asked for early.

    The proof is on the call itself: before `open_after` every one of the 180
    unseen keys raises and S5's call is never reached, while the 140 rows S5
    holds and the 20 maps against themselves go through.
    """
    by_kc, spawn = s12_sides()
    seen = _s12_spies(monkeypatch)
    door = dw.S12Weights(by_kc, spawn)
    keys = dw.matrix_keys()
    unseen = [k for k in keys if not dw.is_seen(k[2], k[3])]
    assert len(unseen) == 180 and not door.is_open

    for k in unseen:
        with pytest.raises(dw.S12GateClosed):
            door.row(k)
    assert seen['weight_keys'] == [] and seen['viewpoint_weights'] == 0
    assert door.asked == []

    for k in keys:
        if dw.is_seen(k[2], k[3]):
            door.row(k)
    for (k, c) in by_kc:
        door.itself(k, c)
    assert len(seen['weight_keys']) == 160 == seen['viewpoint_weights']
    assert _unseen_weight_keys(seen['weight_keys']) == []
    with pytest.raises(ValueError):
        door.row((2, 2, 60, 60))                 # a self pair is not a row

    # a failed, missing or merely truthy check never opens it
    ok, bad = {'ok': True}, {'ok': False}
    for checks in ((bad, ok, ok), (ok, bad, ok), (ok, ok, bad),
                   (ok, None, None), (ok, ok, None), (ok, ok, {'ok': 1}),
                   (ok, ok, {})):
        assert door.open_after(*checks) is False
        with pytest.raises(dw.S12GateClosed):
            door.row(unseen[0])
    assert _unseen_weight_keys(seen['weight_keys']) == []

    assert door.open_after(ok, ok, ok) is True and door.is_open
    door.row(unseen[0])
    assert seen['weight_keys'][-1] == unseen[0]
    # and a later failing check shuts it again
    assert door.open_after(ok, ok, bad) is False
    with pytest.raises(dw.S12GateClosed):
        door.row(unseen[1])


def test_s12_the_door_is_s5s_call_with_s5s_arguments(monkeypatch):
    """Y9: the door adds nothing to S5's call and takes nothing from it.

    No argument was added to `_side_classify`, `classify`, `viewpoint_weights`
    or `shares` for S12: S_A is `res['seg_of']` and S_B is
    `res['weights']['seg_b_of']`, both already returned. So the claim to hold
    is that the door's result IS S5's, element for element, and that
    `classify` is handed what `run_s5` hands it.
    """
    by_kc, spawn = s12_sides(double=True)
    key = (0, 4, 1200, 60)                       # a row S5 holds
    a, b = by_kc[(0, 1200)], by_kc[(4, 60)]
    direct_res, direct_sh = dw._side_classify(a, b, spawn[0], spawn[4])

    calls = []
    real = dw.classify
    monkeypatch.setattr(dw, 'classify', lambda *p, **k: (
        calls.append((p, k)), real(*p, **k))[1])
    res, sh = dw.S12Weights(by_kc, spawn).row(key)

    assert sh == direct_sh
    assert set(res) == set(direct_res)
    for name in ('assigned', 'seg_of', 'corroborated', 'cls'):
        assert np.array_equal(res[name], direct_res[name])
    assert set(res['weights']) == {'w', 'seg_b_of', 'unweighable_A',
                                   'unweighable_B'}
    for name, arr in res['weights'].items():
        assert np.array_equal(arr, direct_res['weights'][name],
                              equal_nan=True)

    (p, k), = calls
    assert p[0] is a['el'] and p[1] is a['seg_of'] and p[3] is None
    assert set(k) == {'hist_a', 'hist_b'}
    assert k['hist_a'] is a['bearing_hist']
    assert k['hist_b'] is b['bearing_hist']
    el_b = p[2]                                  # B's FACE elements, in A's
    keep = b['seg_of'] >= 0                      # frame, with their segments
    assert el_b['n'] == int(keep.sum()) < b['el']['n']
    assert np.array_equal(el_b['seg'], b['seg_of'][keep])
    assert np.allclose(el_b['y'], b['el']['y'][keep] + 0.02)
    assert np.array_equal(el_b['x'], b['el']['x'][keep])


def test_s12_names_every_location_in_s5s_file():
    """Y10: 1440 weight values over six locations, on exactly S5's 140 rows."""
    by_kc, spawn = s12_sides()
    s5 = s12_fake_s5(by_kc, spawn)
    registered, beside = dw.s5_weight_comparisons(s5)
    locations = collections.Counter(c['location'] for c in registered)
    assert locations == {'P7.rows[].reported': 140,
                         'P8.rows[].reported_B60': 140,
                         'P8.rows[].w_mean_B1200': 20,
                         'P8.rows[].w_mean_B60': 20,
                         'reported.A_cut1200_by_B_cut[]': 560,
                         'reported.same_cut[]': 560}
    assert len(registered) == 1440
    assert {c['field'] for c in registered} == set(dw.WEIGHT_FIELDS) == {
        'w_mean', 'W', 'w_p10', 'w_p50', 'w_p90', 'unweighable_A',
        'unweighable_B'}
    rows = {c['key'] for c in registered}
    assert rows == {k for k in dw.matrix_keys() if dw.is_seen(k[2], k[3])}
    assert len(rows) == 140
    # P7 is both at cut1200; P8's two columns are B at 1200 and B at 60
    assert {c['key'][2:] for c in registered
            if c['location'] == 'P7.rows[].reported'} == {(1200, 1200)}
    assert {c['key'][2:] for c in registered
            if c['location'] == 'P8.rows[].w_mean_B60'} == {(1200, 60)}
    assert {c['key'][2:] for c in registered
            if c['location'] == 'P8.rows[].w_mean_B1200'} == {(1200, 1200)}
    # nothing of E7's or w_null's is compared, beside the gate or in it
    for c in registered + beside:
        assert 'E7' not in c['path'] and 'null' not in c['path']
        assert not c['path'].endswith('.mean_w')
    assert {c['field'] for c in beside} == set(dw.S5_BESIDE_FIELDS)

    reg6, beside6 = dw.s6_c_comparisons(s12_fake_s6(by_kc, spawn))
    assert len(reg6) == 320 and {c['field'] for c in reg6} == {'corroborated'}
    assert {c['key'] for c in reg6} == set(dw.matrix_keys())
    assert len(beside6) == 320 * len(dw.S6_BESIDE_FIELDS)


def test_s12_g10_check_1_one_ulp_stops_before_any_weight(tmp_path, capsys,
                                                         monkeypatch):
    """Y11: one ulp on one of S6's 320 C, and S5's call is never made."""
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    was = s6['rows'][201]['corroborated']
    s6['rows'][201]['corroborated'] = math.nextafter(was, 0.0)
    assert s6['rows'][201]['corroborated'] != was

    seen = _s12_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert seen['weight_keys'] == [] and seen['viewpoint_weights'] == 0
    assert sorted(seen['cut_matrix_keys']) == sorted(dw.matrix_keys())

    payload, out = _assert_s12_fail_file(tmp_path, capsys)
    g10 = payload['G10']
    assert g10['parts'] == {'1_C_equals_S6': False,
                            '2_weights_and_G3_equal_S5': False,
                            '3_pairs_give_back_w_mean': False}
    c1 = g10['1_C_equals_S6']
    assert (c1['n'], c1['n_equal'], len(c1['differing'])) == (320, 319, 1)
    assert c1['differing'][0]['path'] == 'rows[201].corroborated'
    assert g10['2_weights_and_G3_equal_S5'] is None
    assert g10['3_pairs_give_back_w_mean'] is None
    assert payload['n_weight_calls'] == 0
    assert 'rows[201].corroborated' in out


def test_s12_g10_check_1_needs_every_row_of_the_matrix(tmp_path, capsys,
                                                       monkeypatch):
    """Y11b: an S6 file a row short fails check 1 though every C is equal."""
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    del s6['rows'][17]
    seen = _s12_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert seen['weight_keys'] == []
    payload, _out = _assert_s12_fail_file(tmp_path, capsys)
    c1 = payload['G10']['1_C_equals_S6']
    assert (c1['covers_the_matrix'], c1['n'], c1['n_equal']) == (
        False, 319, 319)


@pytest.mark.parametrize('where', ['w_p50', 'W', 'unweighable_B',
                                   'P8.w_mean_B60', 'G3.max_abs_w', 'G3.C'])
def test_s12_g10_check_2_one_ulp_stops_before_any_unseen_weight(
        where, tmp_path, capsys, monkeypatch):
    """Y12: one stored value of S5's off by an ulp, and no unseen row is weighed.

    The weight call ran 160 times -- the 140 rows S5 holds and the 20 maps
    against themselves -- and not once on any of the other 180.
    """
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    if where.startswith('G3.'):
        node, field = s5['G3']['rows'][9], where[3:]
    elif where.startswith('P8.'):
        node, field = s5['P8']['rows'][4], where[3:]
    else:
        node, field = s5['reported']['same_cut'][33], where
    was = node[field]
    node[field] = math.nextafter(was, math.inf)
    assert node[field] != was

    seen = _s12_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert len(seen['weight_keys']) == 160 == seen['viewpoint_weights']
    assert _unseen_weight_keys(seen['weight_keys']) == []
    assert sum(1 for k in seen['weight_keys'] if k[0] == k[1]) == 20

    payload, _out = _assert_s12_fail_file(tmp_path, capsys)
    g10 = payload['G10']
    assert g10['parts'] == {'1_C_equals_S6': True,
                            '2_weights_and_G3_equal_S5': False,
                            '3_pairs_give_back_w_mean': False}
    c2 = g10['2_weights_and_G3_equal_S5']
    assert c2['weights']['n'] == 1440 and c2['G3']['n_maps'] == 20
    if where.startswith('G3.'):
        assert c2['weights']['all_equal'] and not c2['G3']['ok']
        assert [r['path'] for r in c2['G3']['not_ok']] == ['G3.rows[9]']
        assert c2['G3']['not_ok'][0]['g3'] is True       # G3 itself held;
        assert c2['G3']['not_ok'][0]['equal_to_S5'] is False   # S5 differs
    else:
        assert c2['G3']['ok'] and c2['weights']['n_equal'] == 1439
        assert len(c2['weights']['differing']) == 1
        assert c2['weights']['differing'][0]['path'].endswith(field)
    assert g10['3_pairs_give_back_w_mean'] is None
    assert payload['n_weight_calls'] == 160
    assert all(dw.is_seen(c['key'][2], c['key'][3])
               for c in c2['weights']['comparisons'])


def test_s12_g10_g3_itself_failing_stops_the_stage(tmp_path, capsys,
                                                   monkeypatch):
    """Y12b: S5 agreed with, and G3 still required: C = 1 and max |w| = 0.

    The weight call is skewed by 0.25 on one map against itself, and the S5
    payload is built through the same skewed call. So the stage reproduces
    S5's stored row exactly, `equal_to_S5` holds, and it is G3's own half --
    max |w| is 0.0 exactly -- that stops the stage.
    """
    by_kc, spawn = s12_sides()
    s6 = s12_fake_s6(by_kc, spawn)
    real = dw._side_classify

    def skewed(a, b, spawn_a, spawn_b):
        res, sh = real(a, b, spawn_a, spawn_b)
        if a is b and a['stem'] == by_kc[(2, 240)]['stem']:
            res['weights']['w'] = res['weights']['w'] + 0.25
        return res, sh

    monkeypatch.setattr(dw, '_side_classify', skewed)
    s5 = s12_fake_s5(by_kc, spawn)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    payload, _out = _assert_s12_fail_file(tmp_path, capsys)
    g3 = payload['G10']['2_weights_and_G3_equal_S5']['G3']
    assert (g3['ok'], g3['n_ok']) == (False, 19)
    bad, = g3['not_ok']
    assert bad['equal_to_S5'] is True and bad['g3'] is False
    assert bad['max_abs_w'] == 0.25


def test_s12_g10_check_3_on_a_row_s5_holds_stops_before_any_unseen_weight(
        tmp_path, capsys, monkeypatch):
    """Y13: the pairs are checked on S5's rows first, and a failure ends it.

    Stricter than the registered order asks: check 3 is read on the 140 rows
    before the door opens, so a fault in the pair record is found on rows
    whose weights are already published.
    """
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    seen = _s12_spies(monkeypatch)
    real = dw.s12_pairs

    def one_pair_wrong(res):
        pairs = real(res)
        if one_pair_wrong.n == 57:               # one of S5's 140 rows
            pairs[0]['w'] += 1e-6
        one_pair_wrong.n += 1
        return pairs

    one_pair_wrong.n = 0
    monkeypatch.setattr(dw, 's12_pairs', one_pair_wrong)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert len(seen['weight_keys']) == 160
    assert _unseen_weight_keys(seen['weight_keys']) == []
    assert one_pair_wrong.n == 140               # S5's rows, and no other

    payload, _out = _assert_s12_fail_file(tmp_path, capsys)
    g10 = payload['G10']
    assert g10['parts'] == {'1_C_equals_S6': True,
                            '2_weights_and_G3_equal_S5': True,
                            '3_pairs_give_back_w_mean': False}
    c3 = g10['3_pairs_give_back_w_mean']
    assert (c3['n_rows'], c3['n_ok'], len(c3['not_ok'])) == (140, 139, 1)
    assert dw.is_seen(*c3['not_ok'][0]['key'][2:])


def test_s12_g10_check_3_on_an_unseen_row_writes_no_verdict(tmp_path, capsys,
                                                            monkeypatch):
    """Y14: all 320 rows weighed, one pair record wrong, and no verdict.

    The fail file names the row and the difference. It holds no weight of an
    unseen row, no mirror pair and no P17, and none is printed.
    """
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    seen = _s12_spies(monkeypatch)
    real = dw.s12_pairs

    def one_pair_wrong(res):
        pairs = real(res)
        if one_pair_wrong.n == 250:              # past S5's 140: an unseen row
            pairs[0]['w'] += 1e-6
        one_pair_wrong.n += 1
        return pairs

    one_pair_wrong.n = 0
    monkeypatch.setattr(dw, 's12_pairs', one_pair_wrong)
    p17_calls = []
    real_p17 = dw.p17_verdict
    monkeypatch.setattr(dw, 'p17_verdict', lambda *a, **k: (
        p17_calls.append(1), real_p17(*a, **k))[1])

    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert len(seen['weight_keys']) == 340       # the gate had passed
    assert p17_calls == []                       # and the verdict never ran

    payload, _out = _assert_s12_fail_file(tmp_path, capsys)
    c3 = payload['G10']['3_pairs_give_back_w_mean']
    assert (c3['n_rows'], c3['n_ok'], len(c3['not_ok'])) == (320, 319, 1)
    bad, = c3['not_ok']
    assert not dw.is_seen(*bad['key'][2:])
    assert set(bad) == {'key', 'abs_diff', 'ok'} and bad['abs_diff'] > 1e-12
    assert all(set(r) == {'key', 'abs_diff', 'ok'} for r in c3['rows'])
    assert payload['n_weight_calls'] == 340


def test_s12_g10_holding_on_a_symmetric_corpus_end_to_end(tmp_path, capsys,
                                                          monkeypatch):
    """Y15: lattice-aligned maps, one-to-one matching, and d exactly 0."""
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    seen = _s12_spies(monkeypatch)
    out = _s12_run(by_kc, spawn, s5, s6, tmp_path)

    # C through S6's call on all 320; S5's call once per row and per map; and
    # the order: every unseen row comes after every row S5 holds
    assert sorted(seen['cut_matrix_keys']) == sorted(dw.matrix_keys())
    wk = seen['weight_keys']
    assert len(wk) == 340 == out['n_weight_calls'] == seen['viewpoint_weights']
    assert len(set(wk)) == 340
    first_unseen = min(i for i, k in enumerate(wk)
                       if k[0] != k[1] and not dw.is_seen(k[2], k[3]))
    assert first_unseen == 160
    assert len(_unseen_weight_keys(wk)) == 180

    g10 = out['G10']
    assert g10['all_ok'] and all(g10['parts'].values())
    c1, c2, c3 = (g10['1_C_equals_S6'], g10['2_weights_and_G3_equal_S5'],
                  g10['3_pairs_give_back_w_mean'])
    assert (c1['n'], c1['n_equal']) == (320, 320)
    assert c1['beside']['n'] == c1['beside']['n_equal'] == 1280
    assert (c2['weights']['n'], c2['weights']['n_equal']) == (1440, 1440)
    assert len(c2['weights']['by_location']) == 6
    assert (c2['G3']['n_maps'], c2['G3']['n_ok']) == (20, 20)
    assert c2['beside']['n'] == c2['beside']['n_equal'] == 600
    assert c2['not_compared']['locations'] == list(dw.S5_NOT_COMPARED)
    assert (c3['n_rows'], c3['n_ok']) == (320, 320)
    assert c3['max_abs_diff'] <= 1e-12

    assert (out['n_rows'], out['n_seen_by_S5'], out['n_unseen_before_S12'],
            out['n_mirror_pairs']) == (320, 140, 180, 160)
    assert sum(1 for r in out['rows'] if r['seen_by_S5']) == 140
    for r in out['rows']:
        assert r['n_weighable'] > 0 and r['pairs']
        assert set(dw.WEIGHT_FIELDS) <= set(r)
        assert sum(p['n'] for p in r['pairs']) == r['n_weighable']
    assert any(len({p['w'] for p in r['pairs']}) > 1 for r in out['rows'])

    p17 = out['P17']
    assert (p17['holds'], p17['n_holding'], p17['of'], p17['min_required']) \
        == (True, 60, 60, 54)
    assert p17['max_d'] == 0.0 and p17['failing'] == []
    assert p17['d_summary']['n_exactly_0'] == 60
    rep = out['reported']
    assert (rep[dw.M1200]['n'], rep[dw.SAME_CUT]['n']) == (60, 40)
    assert rep[dw.M1200]['n_exactly_0'] == 60
    assert rep[dw.SAME_CUT]['max'] == 0.0
    assert rep['w_symmetry']['n_segment_pairs_in_both_rows'] > 160
    assert rep['w_symmetry']['max_abs_diff'] <= 1e-15

    written = list(tmp_path.glob('s12_mirror_*.json'))
    assert len(written) == 1 and not list(tmp_path.glob('s12_g10_fail_*'))
    payload = json.loads(written[0].read_text())
    assert out['written']['sha256'] == hashlib.sha256(
        written[0].read_bytes()).hexdigest()
    assert payload['head'] == 'synthetic'
    assert payload['sources'] == S12_META['sources']
    assert len(payload['rows']) == 320 and len(payload['mirror_pairs']) == 160
    printed = capsys.readouterr().out
    assert 'G10: check 1 HOLD  check 2 HOLD  check 3 HOLD   HOLD' in printed
    assert 'P17: HOLD  60/60 M60 pairs' in printed
    assert _keys_naming_the_null(payload) == []


def test_s12_a_double_wall_moves_eight_pairs_and_shows_the_join(tmp_path,
                                                                capsys):
    """Y16: one map with two walls, and the asymmetry P17 is about.

    k4 at cut60 has a wall either side of everyone else's. Another robot's
    wall matches k4's lower one only; both of k4's match that robot's wall.
    So for X in k0..k3 at cut120 or cut240 against k4 at cut60 -- 8 of M60's
    60 -- the pair (S_X, upper) carries length from k4's side and none from
    X's, and the two means differ.

    Seven of the eight differ by more than 1/180 and fail. The eighth, k1 at
    cut120, differs by 1/1260 -- 137/7 against 272/14 bins -- which is not 0
    and is inside the bound, so it HOLDS: the verdict is on d, not on whether
    the two rows weighed the same pairs. 53 of 60, and P17 fails.
    """
    by_kc, spawn = s12_sides(double=True)
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    out = _s12_run(by_kc, spawn, s5, s6, tmp_path)

    assert out['G10']['all_ok']
    p17 = out['P17']
    moved = {(x, 4, a, 60) for x in range(4) for a in (120, 240)}
    m60 = {(m['X'], m['Y'], m['cut_X'], m['cut_Y']): m
           for m in out['mirror_pairs'] if m['set'] == dw.M60}
    assert {k for k, m in m60.items() if m['d'] != 0.0} == moved
    inside = m60[(1, 4, 120, 60)]
    assert inside['d'] == pytest.approx(BINW / 180.0 / 7.0)
    assert 0.0 < inside['d'] <= 1.0 / 180.0

    assert (p17['holds'], p17['n_holding'], p17['of']) == (False, 53, 60)
    assert (p17['n_over_the_bound'], p17['n_undefined']) == (7, 0)
    assert {(f['X'], f['Y'], f['cut_X'], f['cut_Y']) for f in p17['failing']} \
        == moved - {(1, 4, 120, 60)}
    assert p17['max_d'] == max(f['d'] for f in p17['failing'])
    assert all(f['d'] > 1.0 / 180.0 for f in p17['failing'])
    assert p17['d_summary']['n_exactly_0'] == 52

    rows = {(r['A'], r['B'], r['cut_A'], r['cut_B']): r for r in out['rows']}
    for f in p17['failing']:
        kf = (f['X'], f['Y'], f['cut_X'], f['cut_Y'])
        kr = (f['Y'], f['X'], f['cut_Y'], f['cut_X'])
        joined = f['matched_pairs']
        assert {(j['S_X'], j['S_Y']) for j in joined} == {
            (0, 0), (1, 0), (0, 1), (1, 1)}
        for j in joined:
            if j['S_Y'] == 0:                    # k4's lower wall: both sides
                assert j['w_X_to_Y'] == pytest.approx(j['w_Y_to_X'])
                assert j['n_X_to_Y'] == j['n_Y_to_X'] > 0
                assert j['length_X_to_Y_m'] == j['length_Y_to_X_m'] > 0.0
            else:                                # its upper wall: k4's only
                assert j['w_X_to_Y'] is None and j['w_Y_to_X'] is not None
                assert (j['n_X_to_Y'], j['length_X_to_Y_m']) == (0, 0.0)
                assert j['length_Y_to_X_m'] > 0.0
        # the join carries both rows whole: lengths and the two means
        for side_, key in (('X_to_Y', kf), ('Y_to_X', kr)):
            assert sum(j[f'n_{side_}'] for j in joined) \
                == rows[key]['n_weighable']
            mean = (sum(j[f'w_{side_}'] * j[f'n_{side_}'] for j in joined
                        if j[f'n_{side_}'])
                    / rows[key]['n_weighable'])
            assert mean == pytest.approx(rows[key]['w_mean'], abs=1e-12)
        assert f['d'] == abs(rows[kf]['w_mean'] - rows[kr]['w_mean'])

    # the hand-computed one: k0 at cut120 against k4 at cut60, in bins of 5 deg
    f = next(f for f in p17['failing']
             if (f['X'], f['cut_X']) == (0, 120))
    assert f['w_mean_X_to_Y'] == pytest.approx((7 + 5 * 15) / 6 * BINW / 180)
    assert f['w_mean_Y_to_X'] == pytest.approx(
        (7 + 5 * 15 + 25 + 5 * 25) / 12 * BINW / 180)

    # the same double wall also moves pairs that are reported, not judged
    rep = out['reported']
    assert rep[dw.M1200]['max'] > 1.0 / 180.0
    assert rep[dw.SAME_CUT]['max'] > 1.0 / 180.0
    assert rep[dw.M1200]['n_exactly_0'] < 60
    printed = capsys.readouterr().out
    assert 'P17: FAIL  53/60 M60 pairs' in printed
    assert 'k1@cut120 <-> k4@cut60' not in printed   # it holds; not shown
    assert 'k0@cut120 <-> k4@cut60' in printed and 'S_X  1  S_Y  1' in printed
    assert len(list(tmp_path.glob('s12_mirror_*.json'))) == 1


def test_s12_reported_summaries_and_w_symmetry():
    """Y18: min, median, max and zeroes over a set; w from both sides."""
    entries = [{'d': v} for v in (0.0, 0.0, 0.002, None, 0.01)]
    s = dw.d_summary(entries)
    assert (s['n'], s['n_undefined'], s['n_exactly_0']) == (5, 1, 2)
    assert (s['min'], s['median'], s['max']) == (0.0, 0.001, 0.01)
    assert s['max_deg'] == pytest.approx(1.8)
    empty = dw.d_summary([{'d': None}])
    assert (empty['n'], empty['n_undefined'], empty['min'], empty['median'],
            empty['max'], empty['max_deg']) == (1, 1, None, None, None, None)

    def row(*pairs):
        return {'pairs': [{'S_A': a, 'S_B': b, 'w': w, 'n': n,
                           'length_m': n * RHO} for a, b, w, n in pairs]}

    xy = row((0, 0, 0.10, 2), (1, 0, 0.30, 1))
    yx = row((0, 0, 0.10, 2), (0, 1, 0.30 + 1e-9, 1), (2, 1, 0.5, 4))
    joined = dw.joined_pairs(xy, yx)
    assert [(j['S_X'], j['S_Y']) for j in joined] == [(0, 0), (1, 0), (1, 2)]
    assert joined[2] == {'S_X': 1, 'S_Y': 2, 'w_X_to_Y': None,
                         'w_Y_to_X': 0.5, 'n_X_to_Y': 0, 'n_Y_to_X': 4,
                         'length_X_to_Y_m': 0.0,
                         'length_Y_to_X_m': pytest.approx(0.4)}
    m = {'X': 0, 'Y': 1, 'cut_X': 60, 'cut_Y': 120}
    sym = dw.w_symmetry({(0, 1, 60, 120): xy, (1, 0, 120, 60): yx}, [m])
    assert (sym['n_segment_pairs_in_both_rows'], sym['n_exactly_0']) == (2, 1)
    assert sym['max_abs_diff'] == pytest.approx(1e-9)
    assert sym['at'] == {**m, 'S_X': 1, 'S_Y': 0}


def test_s12_the_fail_payload_guard_catches_a_verdict():
    """Y14b: the guard on a fail file fires on each thing it must not hold."""
    ok = {'stage': 's12_g10_fail', 'predictions_verbatim': {'P17': 'text'},
          'G10': {'1_C_equals_S6': {'comparisons': [
                      {'key': [0, 1, 60, 120], 'field': 'corroborated',
                       'stored': 0.5, 'computed': 0.5}]},
                  '2_weights_and_G3_equal_S5': {'weights': {'comparisons': [
                      {'key': [0, 1, 1200, 60], 'field': 'w_mean',
                       'stored': 0.1, 'computed': 0.1}]}},
                  '3_pairs_give_back_w_mean': {'rows': [
                      {'key': [0, 1, 60, 120], 'abs_diff': 0.0, 'ok': True}]}}}
    dw._no_verdict_in_s12(ok)                    # what a fail file does hold
    for extra in ({'P17': {}}, {'rows': []}, {'mirror_pairs': []},
                  {'reported': {}}, {'x': [{'w_mean': 0.1}]},
                  {'x': {'pairs': []}}, {'x': {'d': 0.0}}):
        with pytest.raises(AssertionError):
            dw._no_verdict_in_s12({**ok, **extra})
    unseen = json.loads(json.dumps(ok))
    unseen['G10']['2_weights_and_G3_equal_S5']['weights']['comparisons'][0][
        'key'] = [0, 1, 60, 120]
    with pytest.raises(AssertionError):
        dw._no_verdict_in_s12(unseen)


def test_s12_the_fail_payload_is_walked_before_it_is_written(tmp_path,
                                                            monkeypatch):
    """Y14c: the stage calls the guard, and a guard that fires writes nothing."""
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    s6['rows'][5]['corroborated'] = math.nextafter(
        s6['rows'][5]['corroborated'], 0.0)
    walked = []

    def guard(payload):
        walked.append(payload['stage'])
        raise AssertionError('a verdict in a fail payload')

    monkeypatch.setattr(dw, '_no_verdict_in_s12', guard)
    with pytest.raises(AssertionError, match='a verdict in a fail payload'):
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert walked == ['s12_g10_fail']
    assert not list(tmp_path.iterdir())


def test_s12_needs_bearing_histograms_on_every_side(tmp_path, monkeypatch):
    """A side with no histograms cannot be weighed, and stops before any C."""
    by_kc, spawn = s12_sides()
    s5, s6 = s12_fake_s5(by_kc, spawn), s12_fake_s6(by_kc, spawn)
    by_kc[(3, 120)]['bearing_hist'] = None
    seen = _s12_spies(monkeypatch)
    with pytest.raises(SystemExit) as e:
        _s12_run(by_kc, spawn, s5, s6, tmp_path)
    assert e.value.code == 2
    assert seen['cut_matrix_keys'] == [] and seen['weight_keys'] == []
    assert not list(tmp_path.iterdir())


S12_SPEC_TEXT = (
    '## Viewpoint weight at mixed cuts (P17)\n\n'
    '**P17. REGISTERED 13 Oct 2026.** For a mirror pair, let\n'
    'd = |w_mean(X@a → Y@b) − w_mean(Y@b → X@a)|. P17 holds if d ≤ 1/180.\n\n'
    '**G10 (gate).** Before it computes any weight on a row S5 does not '
    'hold, S12:\n'
    '1. recomputes C on all 320 rows;\n'
    '2. recomputes w_mean on every\n   row where S5 holds them.\n\n'
    'On any difference it stops before writing a verdict. A third check '
    'runs on every row\nS12 computes.\n\n'
    '**What S12 computes and records.** For each of the 320 rows.\n')


def test_s12_reads_the_registered_paragraphs_from_the_spec_text():
    """Y20: P17, G10 with its two numbered checks, and the third check."""
    assert dw.spec_paragraph(S12_SPEC_TEXT, dw.P17_HEADER) == (
        '**P17. REGISTERED 13 Oct 2026.** For a mirror pair, let '
        'd = |w_mean(X@a → Y@b) − w_mean(Y@b → X@a)|. P17 holds if '
        'd ≤ 1/180.')
    assert dw.spec_paragraph(S12_SPEC_TEXT, dw.G10_HEADER) == (
        '**G10 (gate).** Before it computes any weight on a row S5 does not '
        'hold, S12: 1. recomputes C on all 320 rows; 2. recomputes w_mean on '
        'every row where S5 holds them.')
    assert dw.spec_paragraph(S12_SPEC_TEXT, dw.G10_THIRD_HEADER) == (
        'On any difference it stops before writing a verdict. A third check '
        'runs on every row S12 computes.')
    assert dw.P17_GATE_TEXT in S12_SPEC_TEXT
    assert dw.G10_GATE_TEXT in S12_SPEC_TEXT


def _s12_refusal_spies(monkeypatch):
    """Spies on everything `run_s12` could read: the corpus and two files."""
    seen = {'load_side': 0, 'sha256': 0, 's12_mirror': 0}
    real = dw._sha256

    def load_side(*a, **k):
        seen['load_side'] += 1
        raise AssertionError('run_s12 loaded a map past a refusal')

    def sha256(path):
        seen['sha256'] += 1
        return real(path)

    def s12_mirror(*a, **k):
        seen['s12_mirror'] += 1
        raise AssertionError('run_s12 reached the stage past a refusal')

    monkeypatch.setattr(dw, 'load_side', load_side)
    monkeypatch.setattr(dw, '_sha256', sha256)
    monkeypatch.setattr(dw, 's12_mirror', s12_mirror)
    return seen


S12_NOTHING = {'load_side': 0, 'sha256': 0, 's12_mirror': 0}


def test_s12_refuses_a_dirty_tree(monkeypatch):
    """Y19a: a result file must not name a commit it was not produced by."""
    seen = _s12_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s12()
    assert e.value.code == 2
    assert seen == S12_NOTHING


def test_s12_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """Y19b: 82e6462 registered P17 and G10 and must be behind the code."""
    seen = _s12_refusal_spies(monkeypatch)
    asked = []

    def fake(*a):
        asked.append(a)
        return '' if a[0] in ('status', 'merge-base') else 'deadbeef'

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s12()
    assert e.value.code == 2
    assert ('merge-base', '82e6462', 'HEAD') in asked
    assert dw.S12_REGISTRATION == '82e6462'
    assert seen == S12_NOTHING


@pytest.mark.parametrize('present', ['', 'P17', 'G10'])
def test_s12_refuses_when_the_spec_at_head_lacks_p17_or_g10(present,
                                                            monkeypatch):
    """Y19c: the prediction AND its gate, not one of them."""
    seen = _s12_refusal_spies(monkeypatch)
    text = '## 9\n\n**P16. REGISTERED 11 Oct 2026.** something\n'
    if present == 'P17':
        text += '\n**P17. REGISTERED 13 Oct 2026.** something\n'
    if present == 'G10':
        text += '\n**G10 (gate).** something\n'

    def fake(*a):
        if a[0] == 'status':
            return ''
        if a[0] == 'show':
            return text
        return 'c0ffee'                       # merge-base == rev-parse

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s12()
    assert e.value.code == 2
    assert seen == S12_NOTHING


def test_s12_refuses_a_missing_or_wrong_s5_or_s6_file(tmp_path, monkeypatch):
    """Y19d: G10 is an equality against ONE S5 run and ONE S6 run.

    Each file is refused when missing and when its hash is not the pinned
    one, S5 first, and no map is loaded in any of the four.
    """
    seen = _s12_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] == 'status' else S12_SPEC_TEXT if a[0] == 'show'
        else 'c0ffee'))

    def refused():
        with pytest.raises(SystemExit) as e:
            dw.run_s12(tmp_path)
        assert e.value.code == 2
        assert seen['load_side'] == 0 and seen['s12_mirror'] == 0

    refused()                                   # neither file is there
    assert seen['sha256'] == 0

    s5_path = tmp_path / dw.S5_RESULT
    s5_path.write_text('{"G3": {"rows": []}}')
    refused()                                   # S5 there, and not the pinned
    assert seen['sha256'] == 1

    monkeypatch.setattr(dw, 'S5_SHA256',
                        hashlib.sha256(s5_path.read_bytes()).hexdigest())
    refused()                                   # S5 passes; S6 is missing
    assert seen['sha256'] == 2

    (tmp_path / dw.S6_RESULT).write_text('{"rows": []}')
    refused()                                   # S6 there, and not the pinned
    assert seen['sha256'] == 4

    monkeypatch.undo()
    assert dw.S5_RESULT == 's5_viewpoint_20261006T212758Z.json'
    assert dw.S5_SHA256 == ('f99e001d517734ea4b8d737436f0e9243122c0edd9d30f65'
                            'a241dde39e89bb67')
    assert dw.S6_RESULT == 's6_cutmatrix_20261008T225604Z.json'
    assert dw.S6_SHA256 == ('a3d584f310e509c33bcca09c2fe641d4914dd4f768146f10'
                            'c1ebf340f58b2f13')


def test_s12_is_a_stage_of_the_command_line(monkeypatch):
    """`--stage s12` reaches `run_s12` and nothing else."""
    called = []
    monkeypatch.setattr(dw, 'run_s12', lambda out_dir: called.append(out_dir))
    monkeypatch.setattr(dw.sys, 'argv', ['divergence_walls.py', '--stage',
                                         's12'])
    assert dw.main() == 0
    assert called == [dw.OUT_DIR_DEFAULT]


# ───── S13: parked-robot shadows, G11, X7, P18 and P19 (the Z series) ─────────
#
# SYNTHETIC ONLY. No ray of a real bag is read, no blocked beam, mark or Δ is
# computed on a real map, and nothing here reads S3's or S6's result file. The
# one real file read is the spec, for the registered text.
#
#   Z1   a corner, heading +x +y                 the x-step first
#   Z2   a corner the exact order would take y    the x-step first, whatever
#        first, and that cell_index puts in the   cell cell_index gives the
#        y-neighbour                              corner point
#   Z3   the corner tolerance, along the ray      0.9e-9 m: x first; 1.1e-9 m:
#                                                 the exact order
#   Z4   a ray along a grid line                  the row cell_index gives it
#   Z5   cell_index's slack at a lower edge       the walk starts in the upper
#                                                 cell; a floor would not
#   Z6   the starting cell occupied               skipped
#   Z7   an occupied cell inside a spawn disk     skipped; just outside, not
#   Z8   free and unknown cells                   passed; a face entered from
#                                                 unknown marks nothing
#   Z9   entered below 7.9 m, at 7.9 m, beyond    marked; nothing; nothing
#   Z10  leaving the grid                         nothing marked
#   Z11  an unassigned face                       nothing marked
#   Z12  blocked beams: hit false; a return at    never; counts; does not;
#        exactly r + 1e-9, one ulp beyond; an     excluded and counted
#        origin inside a disk
#   Z13  shadowed and Δ on a hand count           marked 4, shadowed 2, Δ 2/12,
#                                                 2/6; one mark per element
#   Z14  the frame                                s_j - s_A; B's beam moved by
#                                                 s_B - s_A
#   Z15  the door                                 raises before G11, and after
#                                                 any failed or missing check;
#                                                 opens on five
#   Z16  G11 checks 1 to 5, each failing          exit 2, a fail file, and no
#                                                 mark or Δ (spies); a check
#                                                 that failed runs nothing
#                                                 after it
#   Z17  the verdict's integers                   5 % and 1 % hold, one more
#                                                 fails; 18 of 20 holds, 17
#                                                 fails; no measured element
#                                                 counts against
#   Z18  the fail path, with a real meta          the file is written: no
#                                                 AssertionError from the
#                                                 meta's predictions_verbatim
#   Z19  a corpus holding G11, end to end         320 rows, P18 and P19 20/20,
#                                                 one file
#   Z20  readings at or below range_min           -inf and range_min count;
#                                                 NaN, +inf and non-admitted
#                                                 scans do not; 0.3 m inclusive
#   Z21  S6's synthetic corpus and check 5        the test's corpus; e3d8568b
#   Z22  the registered text                      read from the spec; a changed
#                                                 number refuses
#   Z23  --stage s13 refusals                     dirty tree, no 2a88223, a
#                                                 spec without P18, P19 or G11,
#                                                 a missing or wrong S6 or S3:
#                                                 exit 2, nothing loaded
#   Z24  --stage s13 on the command line          reaches run_s13

SPEC_PATH = os.path.join(REPO_ROOT, 'docs', 'specs',
                         'SPEC_b2_divergence_walls.md')
NO_DISKS = {'centres': np.zeros((0, 2)), 'cells': frozenset(), 'r': 0.176,
            'tol': 1e-9}


def z_side(grid, origin=ORIGIN, seg_of=None):
    """A grid for the walk, its elements and their lookup."""
    el = gw.face_elements(grid, RHO, origin)
    if seg_of is None:
        seg_of = np.zeros(el['n'], dtype=np.int64)
    return ({'grid': grid, 'origin': origin, 'rho': RHO},
            dw.face_lookup(el, seg_of), el)


def z_element(el, row, col, d):
    hits = [i for i in range(el['n'])
            if (el['row'][i], el['col'][i], el['dir'][i]) == (row, col, d)]
    assert len(hits) == 1, (row, col, d, hits)
    return hits[0]


def z_ray(px, py, heading_deg=None, d=None, back=0.3):
    """A ray whose return point is (px, py), its origin `back` m behind it."""
    if d is None:
        d = (math.cos(math.radians(heading_deg)),
             math.sin(math.radians(heading_deg)))
    return {'ox': px - back * d[0], 'oy': py - back * d[1], 'dx': d[0],
            'dy': d[1], 'free_len': back}


DIR_PX, DIR_MX, DIR_PY, DIR_MY = 0, 1, 2, 3          # gw.DIRS: +x, -x, +y, -y


def test_s13_entry_dirs_name_the_face_a_step_crosses():
    """A +x step crosses the cell's -x face, whose free neighbour it came from."""
    assert [gw.DIRS[k][4] for k in (DIR_PX, DIR_MX, DIR_PY, DIR_MY)] == [
        '+x', '-x', '+y', '-y']
    assert dw.ENTRY_DIR == {('x', 1): DIR_MX, ('x', -1): DIR_PX,
                            ('y', 1): DIR_MY, ('y', -1): DIR_PY}


def test_s13_walk_corner_takes_the_x_step_first():
    """Z1: through the corner of cells (0,0), (0,1), (1,0), (1,1), x first."""
    g = blank(4, 4, FREE)
    g[0, 1] = g[1, 0] = g[1, 1] = OCC
    side_, lookup, el = z_side(g)
    e, why = dw.walk_mark(z_ray(0.0, 0.0, 45.0), side_, lookup, NO_DISKS)
    assert why == dw.WALK_MARKED
    assert e == z_element(el, 0, 1, DIR_MX)       # not (1, 0)'s -y face


def test_s13_walk_corner_overrides_cell_index_and_the_exact_order():
    """Z2: heading -x +y, cell_index gives the corner point to the y-neighbour,
    and the y-edge is crossed first by 2.8e-10 m; the x-step still comes first."""
    g = blank(4, 4, FREE)
    g[0, 0] = g[1, 1] = OCC
    side_, lookup, el = z_side(g)
    ray = z_ray(0.1, 0.0, 135.0)
    e, why = dw.walk_mark(ray, side_, lookup, NO_DISKS)
    assert why == dw.WALK_MARKED
    assert e == z_element(el, 0, 0, DIR_PX)       # x first: (0, 0), not (1, 1)
    # the exact order, with no corner rule, would have taken the y-step
    assert dw.walk_mark(ray, side_, lookup, NO_DISKS, corner_tol=0.0) == (
        z_element(el, 1, 1, DIR_MY), dw.WALK_MARKED)


@pytest.mark.parametrize('gap,expect', [(0.9e-9, (0, 1, DIR_MX)),
                                        (1.1e-9, (1, 0, DIR_MY))])
def test_s13_walk_corner_tolerance_is_along_the_ray(gap, expect):
    """Z3: the y-edge crossed first by `gap` m along the ray, at 45 degrees."""
    g = blank(4, 4, FREE)
    g[0, 1] = g[1, 0] = OCC
    side_, lookup, el = z_side(g)
    shift = gap / math.sqrt(2.0)                  # gap along the ray, at 45 deg
    e, why = dw.walk_mark(z_ray(-shift, 0.0, 45.0), side_, lookup, NO_DISKS)
    assert why == dw.WALK_MARKED and e == z_element(el, *expect)


def test_s13_walk_along_a_grid_line_stays_in_cell_indexs_row():
    """Z4: y exactly on the edge between rows 0 and 1: row 1, by cell_index."""
    g = blank(3, 8, FREE)
    g[0, 2] = OCC                                 # nearer, in the other row
    g[1, 4] = OCC
    side_, lookup, el = z_side(g)
    assert int(dw.cell_index(0.05, 0.0, RHO)) == 1
    e, why = dw.walk_mark(z_ray(0.0, 0.05, d=(1.0, 0.0)), side_, lookup,
                          NO_DISKS)
    assert why == dw.WALK_MARKED and e == z_element(el, 1, 4, DIR_MX)


def test_s13_walk_uses_cell_indexs_slack_at_a_lower_edge():
    """Z5: 5e-11 m below cell 1's edge is cell 1 by cell_index, 0 by a floor."""
    g = blank(1, 3, FREE)
    g[0, 0] = OCC
    side_, lookup, el = z_side(g)
    x = 0.05 - 5e-11
    assert int(dw.cell_index(x, 0.0, RHO)) == 1
    assert math.floor((x - 0.0) / RHO + 0.5) == 0          # supercover's floor
    e, why = dw.walk_mark(z_ray(x, 0.0, d=(-1.0, 0.0)), side_, lookup,
                          NO_DISKS)
    assert why == dw.WALK_MARKED and e == z_element(el, 0, 0, DIR_PX)


def test_s13_walk_skips_its_starting_cell_even_when_occupied():
    """Z6: the return point lies in an occupied cell; the walk goes on."""
    g = blank(1, 6, FREE)
    g[0, 2] = g[0, 4] = OCC
    side_, lookup, el = z_side(g)
    e, why = dw.walk_mark(z_ray(0.2, 0.0, d=(1.0, 0.0)), side_, lookup,
                          NO_DISKS)
    assert why == dw.WALK_MARKED and e == z_element(el, 0, 4, DIR_MX)


@pytest.mark.parametrize('offset,expect_col', [(0.176, 4), (0.176 + 1e-9, 4),
                                               (0.176 + 2e-9, 2)])
def test_s13_walk_skips_cells_inside_a_spawn_disk(offset, expect_col):
    """Z7: a cell whose centre is at most r + 1e-9 from a spawn is skipped."""
    g = blank(1, 6, FREE)
    g[0, 2] = g[0, 4] = OCC
    side_, lookup, el = z_side(g)
    disks = dw.spawn_disks_on(side_, [(0.2, -offset)])
    assert ((0, 2) in disks['cells']) == (expect_col == 4)
    e, why = dw.walk_mark(z_ray(0.0, 0.0, d=(1.0, 0.0)), side_, lookup, disks)
    assert why == dw.WALK_MARKED
    assert e == z_element(el, 0, expect_col, DIR_MX)


def test_s13_walk_passes_free_and_unknown_cells():
    """Z8: free and unknown do not stop it; a face entered from unknown is no
    element, so nothing is marked there."""
    g = blank(2, 8, UNK)
    g[0, 1:4] = FREE
    g[0, 5] = OCC                                 # entered from (0, 4), unknown
    g[1, :] = FREE
    g[1, 6] = OCC                                 # entered from (1, 5), free
    side_, lookup, el = z_side(g)
    e, why = dw.walk_mark(z_ray(0.0, 0.0, d=(1.0, 0.0)), side_, lookup,
                          NO_DISKS)
    assert (e, why) == (None, dw.WALK_NOT_A_FACE)
    e, why = dw.walk_mark(z_ray(0.0, 0.1, d=(1.0, 0.0)), side_, lookup,
                          NO_DISKS)
    assert why == dw.WALK_MARKED and e == z_element(el, 1, 6, DIR_MX)


@pytest.mark.parametrize('entry,expect', [(7.9 - 1e-6, dw.WALK_MARKED),
                                          (7.9 + 1e-9, dw.WALK_END),
                                          (7.9 + 1e-6, dw.WALK_END)])
def test_s13_walk_stops_only_at_a_cell_entered_below_7_9_m(entry, expect):
    """Z9: the occupied cell's edge at `entry` m from the ray's origin.

    An entry at exactly 7.9 m is not built here, because the walk's arithmetic
    cannot place an edge there to the last bit; the next case holds the bound
    exactly, on a return point at 7.9 m.
    """
    g = blank(1, 90, FREE)
    g[0, 85] = OCC
    side_, lookup, el = z_side(g)
    edge = (85 - 0.5 - 1e-9) * RHO                # where cell_index enters 85
    ray = {'ox': edge - entry, 'oy': 0.0, 'dx': 1.0, 'dy': 0.0,
           'free_len': 0.5}
    e, why = dw.walk_mark(ray, side_, lookup, NO_DISKS)
    assert why == expect
    assert (e is not None) == (expect == dw.WALK_MARKED)


def test_s13_walk_returns_nothing_at_7_9_m_or_off_the_grid():
    """Z9, Z10: a return at 7.9 m walks nowhere; leaving the grid marks nothing."""
    g = blank(1, 6, FREE)
    side_, lookup, _el = z_side(g)
    ray = {'ox': -7.9, 'oy': 0.0, 'dx': 1.0, 'dy': 0.0, 'free_len': 7.9}
    assert dw.walk_mark(ray, side_, lookup, NO_DISKS) == (None, dw.WALK_END)
    assert dw.walk_mark(z_ray(0.2, 0.0, d=(1.0, 0.0)), side_, lookup,
                        NO_DISKS) == (None, dw.WALK_OFF_GRID)
    assert dw.walk_mark(z_ray(-0.3, 0.0, d=(1.0, 0.0)), side_, lookup,
                        NO_DISKS) == (None, dw.WALK_OFF_GRID)


def test_s13_walk_marks_only_a_measured_face():
    """Z11: the same face, assigned and unassigned."""
    g = blank(1, 6, FREE)
    g[0, 4] = OCC
    side_, lookup, el = z_side(g)
    i = z_element(el, 0, 4, DIR_MX)
    ray = z_ray(0.0, 0.0, d=(1.0, 0.0))
    assert dw.walk_mark(ray, side_, lookup, NO_DISKS) == (i, dw.WALK_MARKED)
    seg_of = np.zeros(el['n'], dtype=np.int64)
    seg_of[i] = -1
    _s, lookup2, _e = z_side(g, seg_of=seg_of)
    assert dw.walk_mark(ray, side_, lookup2, NO_DISKS) == (
        None, dw.WALK_NOT_MEASURED)


def z_rays(*spec):
    """Rays as (ox, oy, dx, dy, free_len, hit), admitted_rays' shape."""
    a = np.array([s[:5] for s in spec], dtype=float).reshape(-1, 5)
    return {'ox': a[:, 0], 'oy': a[:, 1], 'dx': a[:, 2], 'dy': a[:, 3],
            'free_len': a[:, 4],
            'hit': np.array([bool(s[5]) for s in spec], dtype=bool),
            'scan': np.zeros(len(spec), dtype=np.int64),
            'range_threshold': 7.9}


def test_s13_blocked_beams_need_a_hit_and_a_return_inside():
    """Z12: hit false never counts; r + 1e-9 counts; one ulp beyond does not."""
    edge = dw.SHADOW_R_M + dw.INSIDE_TOL_M
    beyond = math.nextafter(edge, math.inf)
    rays_ = z_rays((edge, -1.0, 0.0, 1.0, 1.0, True),      # return (edge, 0)
                   (beyond, -1.0, 0.0, 1.0, 1.0, True),    # one ulp outside
                   (0.0, -1.0, 0.0, 1.0, 1.0, False),      # no hit, at centre
                   (0.0, -1.0, 0.0, 1.0, 1.0, True))       # hit, at centre
    out = dw.blocked_beams(rays_, [(0.0, 0.0)])
    assert list(out['index']) == [0, 3]
    assert out['n_per_disk'] == [2]
    assert out['n_excluded_origin_inside'] == 0


def test_s13_blocked_beams_exclude_an_origin_inside_any_disk():
    """Z12: the origin in one disk, the return in another: excluded, counted."""
    rays_ = z_rays((0.0, 0.0, 1.0, 0.0, 1.0, True),        # from disk 0 to 1
                   (0.5, 0.0, 1.0, 0.0, 0.5, True))        # from outside to 1
    out = dw.blocked_beams(rays_, [(0.0, 0.0), (1.0, 0.0)])
    assert list(out['index']) == [1] and out['n_per_disk'] == [0, 1]
    assert out['n_excluded_origin_inside'] == 1


def z_hand_corpus(spawn_b=(0.0, 0.0)):
    """A at cut1200 with a 12-element wall, B at cut60 holding its first 6."""
    a = side(wall_grid())
    b = side(wall_grid(w=6))
    by_kc = {(0, 1200): dict(a, stem='b2maps_k0_cut1200_gated_extfix_robot0'),
             (1, 60): dict(b, stem='b2maps_k1_cut60_gated_extfix_robot1')}
    return by_kc, {0: (0.0, 0.0), 1: spawn_b}


def z_up_beams(cols, x_shift=0.0):
    """Blocked beams heading +y, returns at (col * rho - x_shift, 0.2)."""
    xs = np.array([c * RHO - x_shift for c in cols], dtype=float)
    n = len(cols)
    return {'blocked': {'ox': xs, 'oy': np.full(n, -1.0), 'dx': np.zeros(n),
                        'dy': np.ones(n), 'free_len': np.full(n, 1.2),
                        'disk': [0] * n},
            'blocked_per_parked_disk': {'k0': n},
            'readings_at_or_below_range_min': {'n': 0}}


def test_s13_shadowed_needs_marked_and_not_corroborated_hand_count():
    """Z13: B corroborates columns 0-5. Beams mark 2, 3, 7, 8 (7 twice)."""
    by_kc, spawn = z_hand_corpus()
    row = dw.delta_row(by_kc, spawn, (0, 1, 1200, 60),
                       z_up_beams([2, 3, 7, 7, 8]), n_robots=2)
    assert row['measured_n'] == 12 and row['not_corroborated_n'] == 6
    assert row['walk_ends'][dw.WALK_MARKED] == 5
    assert row['n_marked'] == 4 and row['marked_elements'] == [2, 3, 7, 8]
    assert row['shadowed_n'] == 2
    assert row['delta'] == 2 / 12
    assert row['shadowed_over_not_corroborated'] == 2 / 6
    assert row['shadowed_by_segment'] == {'0': 2}
    assert row['not_corroborated'] == 0.5 and row['L_A_m'] == 12 * RHO


def test_s13_the_frame_is_g1s_translation():
    """Z14: s_j - s_A; and B's beams land in A's frame shifted by s_B - s_A."""
    spawn = {0: (0.86, 0.28), 1: (0.0, 0.9), 2: (-0.53, -0.73)}
    for j in spawn:
        for a in spawn:
            assert dw.spawn_in_frame(spawn, j, a) == (
                spawn[j][0] - spawn[a][0], spawn[j][1] - spawn[a][1])
    by_kc, spawn = z_hand_corpus(spawn_b=(0.3, 0.0))
    row = dw.delta_row(by_kc, spawn, (0, 1, 1200, 60), z_up_beams([4]),
                       n_robots=2)
    assert row['marked_elements'] == [7]          # B's x 0.4 is A's 0.7


Z_SPAWN = {0: (0.86, 0.28), 1: (0.0, 0.9), 2: (-0.86, 0.28),
           3: (-0.53, -0.73), 4: (0.53, -0.73)}
Z_HALF = 3.0                                     # the room, -3 to 3 m


def z_corpus():
    """Twenty maps of one square room, each in its robot's own frame."""
    n = int(round(2 * Z_HALF / RHO)) + 1
    g = blank(n, n, FREE)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCC
    by_kc = {}
    for k, s in Z_SPAWN.items():
        origin = (-Z_HALF - s[0], -Z_HALF - s[1])
        el = gw.face_elements(g, RHO, origin)
        for c in dw.CUTS:
            by_kc[(k, c)] = {'stem': f'b2maps_k{k}_cut{c}_gated_extfix_robot{k}',
                             'el': el, 'seg_of': np.zeros(el['n'], np.int64),
                             'grid': g, 'rho': RHO, 'origin': origin,
                             'shape': g.shape}
    return by_kc, dict(Z_SPAWN)


def z_s6(by_kc, spawn):
    keys = [k for k in dw.matrix_keys() if k[2] == 1200]
    rows = dw.cut_matrix(by_kc, spawn, keys)
    return {'rows': [dict(rows[k], A=k[0], B=k[1], cut_A=k[2], cut_B=k[3])
                     for k in keys]}


def z_g0(stem):
    return {'n_observed_cells': 1000, 'agreement': 0.996,
            'disagreeing_cells': 4 if 'k3_cut1200' in stem else 0}


def z_s3(by_kc):
    return {'G0': {s['stem']: z_g0(s['stem']) for s in by_kc.values()}}


def z_b_rays(k, spawn, drop=None, decoy=False):
    """B's rays from its spawn: a hit at every parked spawn, a miss beside it."""
    spec = []
    for j in spawn:
        if j == k or j == drop:
            continue
        x, y = dw.spawn_in_frame(spawn, j, k)
        d = math.hypot(x, y)
        spec.append((0.0, 0.0, x / d, y / d, d, True))
        spec.append((0.0, 0.0, -y / d, x / d, 1.5, True))   # sideways, a wall
    if decoy:
        cx, cy = np.mean([dw.spawn_in_frame(spawn, j, k) for j in spawn], 0)
        d = math.hypot(cx, cy)
        spec.append((0.0, 0.0, cx / d, cy / d, d, True))
    return z_rays(*spec)


def z_inputs_of(spawn, drop_at=None, decoy_at=None):
    """`inputs_of` for the stage: every map's rays, one of them edited."""
    def inputs_of(kc):
        k, _c = kc
        rays_ = z_b_rays(k, spawn, drop=(drop_at[1] if drop_at and
                                         drop_at[0] == kc else None),
                         decoy=decoy_at == kc)
        readings = dw.readings_at_or_below_range_min(
            np.array([[1.0, -np.inf]]), np.array([True]), np.array([0.0]),
            np.array([0.0]), 0.16, [dw.spawn_in_frame(spawn, j, k)
                                    for j in spawn if j != k])
        return dw.shadow_inputs(rays_, readings, spawn, k)
    return inputs_of


def z_meta():
    """The meta run_s13 builds, from the registered text of the real spec."""
    with open(SPEC_PATH, encoding='utf-8') as f:
        verbatim = dw.s13_verbatim(f.read())
    return dw.s13_meta('synthetic', 'synthetic', ['?? .vscode/'], 'synthetic',
                       verbatim, dw.s13_constants_from(verbatim),
                       {'s6': {'sha256': 'x'}, 's3': {'sha256': 'y'}})


def z_spies(monkeypatch):
    """Counts of every mark, Δ, G0 rebuild and ray read the stage makes."""
    seen = {'delta_row': 0, 'walk_mark': 0}
    real_delta, real_walk = dw.delta_row, dw.walk_mark

    def delta_row(*a, **k):
        seen['delta_row'] += 1
        return real_delta(*a, **k)

    def walk_mark(*a, **k):
        seen['walk_mark'] += 1
        return real_walk(*a, **k)

    monkeypatch.setattr(dw, 'delta_row', delta_row)
    monkeypatch.setattr(dw, 'walk_mark', walk_mark)
    return seen


def z_run(tmp_path, s6=None, s3=None, g0=z_g0, inputs_of=None, calls=None):
    by_kc, spawn = z_corpus()
    calls = {} if calls is None else calls
    calls.setdefault('g0', [])
    calls.setdefault('inputs', [])
    inputs_of = inputs_of or z_inputs_of(spawn)

    def g0_of(stem):
        calls['g0'].append(stem)
        return g0(stem)

    def counted_inputs(kc):
        calls['inputs'].append(kc)
        return inputs_of(kc)

    return dw.s13_shadows(by_kc, spawn, s6 or z_s6(by_kc, spawn),
                          s3 or z_s3(by_kc), z_meta(), g0_of, counted_inputs,
                          tmp_path)


def z_failed(tmp_path, capsys):
    """The stage stopped at exit 2 with one fail file and no verdict."""
    written = list(tmp_path.glob('s13_g11_fail_*.json'))
    assert len(written) == 1
    assert not list(tmp_path.glob('s13_shadows_*.json'))
    payload = json.loads(written[0].read_text())
    for key in ('rows', 'verdicts', 'reported', 'P18', 'P19'):
        assert key not in payload
    assert set(payload['predictions_verbatim']) >= {'P18', 'P19', 'G11'}
    out = capsys.readouterr().out
    assert 'P18:' not in out and 'P19:' not in out
    return payload


def test_s13_the_door_raises_until_g11_has_passed(monkeypatch):
    """Z15: before G11, and after any failed or missing check, it raises."""
    by_kc, spawn = z_hand_corpus()
    door = dw.S13Shadows(by_kc, spawn, {(1, 60): z_up_beams([2])}, n_robots=2)
    seen = z_spies(monkeypatch)
    key = (0, 1, 1200, 60)
    with pytest.raises(dw.S13GateClosed):
        door.row(key)
    ok = {'ok': True}
    for checks in ((ok, ok, ok, ok, {'ok': False}), (ok, ok, ok, ok, None),
                   (ok, ok, ok, ok), (ok, ok, ok, ok, {'ok': 1})):
        assert not door.open_after(*checks)
        with pytest.raises(dw.S13GateClosed):
            door.row(key)
    assert seen == {'delta_row': 0, 'walk_mark': 0} and door.asked == []
    assert door.open_after(ok, ok, ok, ok, ok)
    door.row(key)
    assert seen['delta_row'] == 1 and door.asked == [key]


def test_s13_g11_check_5_failing_runs_nothing_after_it(tmp_path, capsys,
                                                      monkeypatch):
    """Z16: S6's synthetic output not the pinned bytes: exit 2, nothing else."""
    monkeypatch.setattr(dw, 'S6_SYNTH_SHA256', '0' * 64)
    seen, calls = z_spies(monkeypatch), {}
    reproduced = []
    monkeypatch.setattr(dw, 'g11_reproduce_s6',
                        lambda *a: reproduced.append(1))
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, calls=calls)
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    assert payload['G11']['parts']['5_S6_synthetic'] is False
    assert payload['G11']['1_reproduce_S6'] is None
    assert reproduced == []
    assert calls == {'g0': [], 'inputs': []}
    assert seen == {'delta_row': 0, 'walk_mark': 0}


def test_s13_g11_check_1_one_ulp_stops_before_g0_and_any_mark(tmp_path, capsys,
                                                              monkeypatch):
    """Z16: one ulp on one stored C of the 80 rows."""
    by_kc, spawn = z_corpus()
    s6 = z_s6(by_kc, spawn)
    s6['rows'][17]['corroborated'] = math.nextafter(
        s6['rows'][17]['corroborated'], 0.0)
    seen, calls = z_spies(monkeypatch), {}
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, s6=s6, calls=calls)
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    c1 = payload['G11']['1_reproduce_S6']
    assert c1['n'] == 160 and c1['n_equal'] == 159 and not c1['ok']
    assert payload['G11']['2_G0_cells'] is None
    assert calls == {'g0': [], 'inputs': []}
    assert seen == {'delta_row': 0, 'walk_mark': 0}


def test_s13_g11_check_1_needs_each_of_the_80_rows(tmp_path, capsys,
                                                   monkeypatch):
    """Z16: S6's file missing one row with A at cut1200 fails check 1."""
    by_kc, spawn = z_corpus()
    s6 = z_s6(by_kc, spawn)
    del s6['rows'][40]
    seen, calls = z_spies(monkeypatch), {}
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, s6=s6, calls=calls)
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    assert payload['G11']['1_reproduce_S6']['covers_the_80_rows'] is False
    assert calls == {'g0': [], 'inputs': []} and seen['delta_row'] == 0


def test_s13_g11_check_2_one_cell_stops_before_any_ray_is_read(
        tmp_path, capsys, monkeypatch):
    """Z16: one map's disagreeing_cells one off S3's."""
    by_kc, _spawn = z_corpus()
    s3 = z_s3(by_kc)
    s3['G0']['b2maps_k2_cut240_gated_extfix_robot2']['disagreeing_cells'] = 1
    seen, calls = z_spies(monkeypatch), {}
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, s3=s3, calls=calls)
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    c2 = payload['G11']['2_G0_cells']
    assert c2['n'] == 20 and c2['n_equal'] == 19
    assert [m['map'] for m in c2['differing']] == [
        'b2maps_k2_cut240_gated_extfix_robot2']
    assert len(calls['g0']) == 20 and calls['inputs'] == []
    assert seen == {'delta_row': 0, 'walk_mark': 0}


def test_s13_g11_check_3_one_empty_parked_disk_stops_before_any_mark(
        tmp_path, capsys, monkeypatch):
    """Z16: map k1 at cut60 loses its beam at parked robot 4: 79 of 80."""
    by_kc, spawn = z_corpus()
    seen, calls = z_spies(monkeypatch), {}
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, calls=calls,
              inputs_of=z_inputs_of(spawn, drop_at=((1, 60), 4)))
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    c3 = payload['G11']['3_detector']
    assert (c3['n_disks'], c3['n_disks_with_a_return']) == (80, 79)
    assert payload['G11']['parts']['4_decoy_X7'] is True
    assert len(calls['inputs']) == 20
    assert seen == {'delta_row': 0, 'walk_mark': 0}


def test_s13_g11_check_4_a_return_in_the_decoy_stops_before_any_mark(
        tmp_path, capsys, monkeypatch):
    """Z16, X7: one hit return in the centroid disk of map k3 at cut1200."""
    by_kc, spawn = z_corpus()
    seen, calls = z_spies(monkeypatch), {}
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, calls=calls,
              inputs_of=z_inputs_of(spawn, decoy_at=(3, 1200)))
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    c4 = payload['G11']['4_decoy_X7']
    assert (c4['n_maps'], c4['n_maps_clean']) == (20, 19)
    assert payload['G11']['parts']['3_detector'] is True
    assert seen == {'delta_row': 0, 'walk_mark': 0}


@pytest.mark.parametrize('measured,shadowed,percent,holds', [
    (100, 5, 5, True), (100, 6, 5, False), (2000, 100, 5, True),
    (2000, 101, 5, False), (100, 1, 1, True), (100, 2, 1, False),
    (1000, 10, 1, True), (1000, 11, 1, False), (0, 0, 5, False)])
def test_s13_verdict_compares_in_integers(measured, shadowed, percent, holds):
    """Z17: a pair holds when 100 * shadowed <= percent * measured."""
    rows = [{'A': a, 'B': b, 'cut_A': 1200, 'cut_B': 60,
             'shadowed_n': shadowed if (a, b) == (0, 1) else 0,
             'measured_n': measured if (a, b) == (0, 1) else 100,
             'delta': None} for a, b in dw.ordered_pairs()]
    v = dw.s13_verdict(rows, 60, percent)
    assert v['n_holding'] == (20 if holds else 19)
    assert [(f['A'], f['B']) for f in v['failing']] == ([] if holds
                                                       else [(0, 1)])
    if not holds:
        assert (v['failing'][0]['shadowed_n'],
                v['failing'][0]['measured_n']) == (shadowed, measured)


@pytest.mark.parametrize('n_bad,holds', [(2, True), (3, False)])
def test_s13_verdict_needs_18_of_20(n_bad, holds):
    """Z17: at least 18 of the 20 pairs."""
    pairs = dw.ordered_pairs()
    rows = [{'A': a, 'B': b, 'cut_A': 1200, 'cut_B': 1200,
             'shadowed_n': 2 if i < n_bad else 1, 'measured_n': 100,
             'delta': None} for i, (a, b) in enumerate(pairs)]
    v = dw.s13_verdict(rows, 1200, 1)
    assert (v['holds'], v['n_holding'], v['of']) == (holds, 20 - n_bad, 20)


def test_s13_the_fail_guard_allows_the_real_meta_and_catches_a_verdict():
    """Z18: predictions_verbatim.P18 is allowed; a P18 anywhere else is not."""
    meta = z_meta()
    assert {'P18', 'P19', 'G11'} <= set(meta['predictions_verbatim'])
    ok = dict(meta, stage='s13_g11_fail',
              G11={'3_detector': {'maps': [{'blocked_per_parked_disk':
                                            {'k1': 0}}]}})
    dw._no_verdict_in_s13(ok)
    for extra in ({'P18': {}}, {'rows': []}, {'verdicts': {}},
                  {'x': [{'delta': 0.0}]}, {'x': {'marked_elements': []}},
                  {'x': {'shadowed_n': 1}}, {'reported': {}}):
        with pytest.raises(AssertionError):
            dw._no_verdict_in_s13({**ok, **extra})


def test_s13_the_fail_path_writes_its_file_with_a_real_meta(tmp_path, capsys,
                                                           monkeypatch):
    """Z18: the S9-S11 flaw: a real meta must reach the file, not an assert."""
    by_kc, spawn = z_corpus()
    s6 = z_s6(by_kc, spawn)
    s6['rows'][3]['L_A_m'] = math.nextafter(s6['rows'][3]['L_A_m'], math.inf)
    with pytest.raises(SystemExit) as e:
        z_run(tmp_path, s6=s6)
    assert e.value.code == 2
    payload = z_failed(tmp_path, capsys)
    assert payload['stage'] == 's13_g11_fail'
    assert payload['predictions_verbatim']['P18'].startswith(
        '**P18. REGISTERED 14 Oct 2026.** With A at cut1200 and B at cut60')


def test_s13_holding_corpus_end_to_end(tmp_path, capsys, monkeypatch):
    """Z19: G11 holds; 320 rows through the door; P18 and P19 hold 20/20."""
    seen, calls = z_spies(monkeypatch), {}
    out = z_run(tmp_path, calls=calls)
    assert out['G11']['all_ok']
    assert out['n_rows'] == 320 and out['n_door_calls'] == 320
    assert seen['delta_row'] == 320
    assert seen['walk_mark'] == 320 * 4           # the blocked beams only
    assert len(calls['g0']) == 20 and len(calls['inputs']) == 20
    rows = out['rows']
    # every parked beam runs on to the room's wall, which B holds too
    assert all(r['walk_ends'][dw.WALK_MARKED] == 4 for r in rows)
    assert all(r['shadowed_n'] == 0 and r['delta'] == 0.0 for r in rows)
    for name in ('P18', 'P19'):
        v = out['verdicts'][name]
        assert (v['holds'], v['n_holding'], v['of']) == (True, 20, 20)
    assert len(out['reported']['fixed_A_series']) == 80
    written = list(tmp_path.glob('s13_shadows_*.json'))
    assert len(written) == 1 and out['written']['path'] == str(written[0])
    assert 'P18: HOLD' in capsys.readouterr().out


def test_s13_readings_at_or_below_range_min():
    """Z20: -inf and range_min count; NaN, +inf, above and unadmitted do not."""
    rmin = 0.16
    ranges = np.array([[-np.inf, rmin, np.nan, np.inf, 0.17],
                       [-np.inf, -np.inf, 1.0, 1.0, 1.0],
                       [0.1, 1.0, 1.0, 1.0, 1.0]])
    reach = np.array([True, True, False])
    sx = np.array([0.3, 2.0, 0.0])
    sy = np.zeros(3)
    out = dw.readings_at_or_below_range_min(ranges, reach, sx, sy, rmin,
                                            [(0.0, 0.0)])
    assert out == {'n': 4, 'n_in_scans_within_0.3_m_of_a_parked_spawn': 2,
                   'n_admitted_scans_within_0.3_m': 1}


def test_s13_s6_synthetic_corpus_is_m10s_and_check_5_holds():
    """Z21: the stage's copy of the corpus is the test's, and gives e3d8568b."""
    mine, spawn = dw._s6_synthetic_corpus()
    theirs, spawn_t = s6_sides()
    assert spawn == spawn_t and sorted(mine) == sorted(theirs)
    for kc in mine:
        a, b = mine[kc], theirs[kc]
        assert a['stem'] == b['stem'] and a['rho'] == b['rho']
        assert a['origin'] == b['origin'] and a['shape'] == b['shape']
        assert np.array_equal(a['grid'], b['grid'])
        assert np.array_equal(a['seg_of'], b['seg_of'])
        assert a['segments'] == b['segments']
        for f in ('x', 'y', 'nx', 'ny', 'row', 'col', 'dir'):
            assert np.array_equal(a['el'][f], b['el'][f])
    c5 = dw.s6_synthetic_check()
    assert c5['ok'] and c5['bytes'] == 293309
    assert c5['sha256'] == ('e3d8568bebeea2b621629ebb8a011cf11839f92f6ed21ce4'
                            'ac85d2868ffea82b')


def test_s13_reads_the_registered_text_and_its_numbers():
    """Z22: P18, P19, G11 and the definitions, read from the spec."""
    with open(SPEC_PATH, encoding='utf-8') as f:
        text = f.read()
    v = dw.s13_verbatim(text)
    assert v['P18'] == (
        '**P18. REGISTERED 14 Oct 2026.** With A at cut1200 and B at cut60, '
        'Δ ≤ 0.05 in at least 18 of the 20 ordered pairs. Reason, an '
        'expectation that has not been tested: B\'s scan plane is pitched '
        'about 0.71° nose-up, so in part of B\'s scans it passes over a parked '
        'robot, which then blocks nothing. Where a robot does block B\'s view, '
        'we expect B to see most of the same walls from nearby places soon '
        'after, and a wall B sees clearly to be corroborated. So Δ should '
        'stay well under 0.05.')
    assert v['P19'].startswith('**P19. REGISTERED 14 Oct 2026.** With A at '
                               'cut1200 and B at cut1200, Δ ≤ 0.01 in at least '
                               '18 of the 20 ordered pairs.')
    assert v['G11'].startswith('**G11 (gate).** S13 computes no Δ')
    assert '5. **Nothing else moves.**' in v['G11']
    assert v['G11_failure_rule'].startswith('On a G11 failure, S13 writes')
    assert v['definitions'].startswith('**Definitions.**')
    assert '- **Mark.** Walk A\'s grid' in v['definitions']
    assert v['definitions'].rstrip().endswith('one row per pair.')
    c = dw.s13_constants_from(v)
    assert c['predictions'] == {
        'p18': {'cut_A': 1200, 'cut_B': 60, 'percent': 5, 'min_pairs': 18,
                'of': 20},
        'p19': {'cut_A': 1200, 'cut_B': 1200, 'percent': 1, 'min_pairs': 18,
                'of': 20}}
    for old, new in (('Δ ≤ 0.05 in at least', 'Δ ≤ 0.06 in at least'),
                     ('radius r = 0.176 m', 'radius r = 0.2 m'),
                     ('less than 7.9 m from', 'less than 8.0 m from')):
        assert old in text
        with pytest.raises(SystemExit):
            dw.s13_constants_from(dw.s13_verbatim(text.replace(old, new, 1)))


def _s13_refusal_spies(monkeypatch):
    seen = {'load_side': 0, 'sha256': 0, 's13_shadows': 0}
    real = dw._sha256

    def load_side(*a, **k):
        seen['load_side'] += 1
        raise AssertionError('run_s13 loaded a map past a refusal')

    def sha256(path):
        seen['sha256'] += 1
        return real(path)

    def s13_shadows(*a, **k):
        seen['s13_shadows'] += 1
        raise AssertionError('run_s13 reached the stage past a refusal')

    monkeypatch.setattr(dw, 'load_side', load_side)
    monkeypatch.setattr(dw, '_sha256', sha256)
    monkeypatch.setattr(dw, 's13_shadows', s13_shadows)
    return seen


S13_NOTHING = {'load_side': 0, 'sha256': 0, 's13_shadows': 0}


def _s13_spec_text():
    with open(SPEC_PATH, encoding='utf-8') as f:
        return f.read()


def test_s13_refuses_a_dirty_tree(monkeypatch):
    """Z23a: a result file must not name a commit it was not produced by."""
    seen = _s13_refusal_spies(monkeypatch)
    monkeypatch.setattr(dw, '_git',
                        lambda *a: ' M experiments/analysis/divergence_walls.py'
                        if a[0] == 'status' else 'deadbeef')
    with pytest.raises(SystemExit) as e:
        dw.run_s13()
    assert e.value.code == 2
    assert seen == S13_NOTHING


def test_s13_refuses_when_the_registration_is_not_an_ancestor(monkeypatch):
    """Z23b: 2a88223 registered P18, P19, G11 and X7."""
    seen = _s13_refusal_spies(monkeypatch)
    asked = []

    def fake(*a):
        asked.append(a)
        return '' if a[0] in ('status', 'merge-base') else 'deadbeef'

    monkeypatch.setattr(dw, '_git', fake)
    with pytest.raises(SystemExit) as e:
        dw.run_s13()
    assert e.value.code == 2
    assert ('merge-base', '2a88223', 'HEAD') in asked
    assert dw.S13_REGISTRATION == '2a88223'
    assert seen == S13_NOTHING


@pytest.mark.parametrize('missing', ['P18. REGISTERED', 'P19. REGISTERED',
                                     'G11 (gate).'])
def test_s13_refuses_a_spec_without_p18_p19_or_g11(missing, monkeypatch):
    """Z23c: the spec at HEAD must carry all three."""
    seen = _s13_refusal_spies(monkeypatch)
    text = _s13_spec_text().replace(missing, 'NOT THERE')
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] == 'status' else text if a[0] == 'show' else 'c0ffee'))
    with pytest.raises(SystemExit) as e:
        dw.run_s13()
    assert e.value.code == 2
    assert seen == S13_NOTHING


def test_s13_refuses_a_missing_or_wrong_s6_or_s3_file(tmp_path, monkeypatch):
    """Z23d: G11 is an equality against ONE S6 run and ONE S3 run."""
    seen = _s13_refusal_spies(monkeypatch)
    text = _s13_spec_text()
    monkeypatch.setattr(dw, '_git', lambda *a: (
        '' if a[0] == 'status' else text if a[0] == 'show' else 'c0ffee'))

    def refused():
        with pytest.raises(SystemExit) as e:
            dw.run_s13(tmp_path)
        assert e.value.code == 2
        assert seen['load_side'] == 0 and seen['s13_shadows'] == 0

    refused()                                   # neither file is there
    assert seen['sha256'] == 0
    s6_path = tmp_path / dw.S6_RESULT
    s6_path.write_text('{"rows": []}')
    refused()                                   # S6 there, and not the pinned
    assert seen['sha256'] == 1
    monkeypatch.setattr(dw, 'S6_SHA256',
                        hashlib.sha256(s6_path.read_bytes()).hexdigest())
    refused()                                   # S6 passes; S3 is missing
    assert seen['sha256'] == 2
    (tmp_path / dw.S3_RESULT).write_text('{"G0": {}}')
    refused()                                   # S3 there, and not the pinned
    assert seen['sha256'] == 4
    monkeypatch.undo()
    assert dw.S3_RESULT == 's3_gates_20261004T201944Z.json'
    assert dw.S3_SHA256 == ('dd0b37969e50f2ee302fac8391ae73291e285b9ed7ae81e6'
                            '5446428c3af4dfc6')
    assert dw.S6_SHA256 == ('a3d584f310e509c33bcca09c2fe641d4914dd4f768146f10'
                            'c1ebf340f58b2f13')


def test_s13_is_a_stage_of_the_command_line(monkeypatch):
    """Z24: `--stage s13` reaches `run_s13` and nothing else."""
    called = []
    monkeypatch.setattr(dw, 'run_s13', lambda out_dir: called.append(out_dir))
    monkeypatch.setattr(dw.sys, 'argv', ['divergence_walls.py', '--stage',
                                         's13'])
    assert dw.main() == 0
    assert called == [dw.OUT_DIR_DEFAULT]


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
