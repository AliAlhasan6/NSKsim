"""Layer 1 -- the measure in experiments/analysis/robot_divergence.py.

That script answers how much five robots' own maps of one world agree when laid
side by side, at four map-time cuts, without ever referencing truth. Everything
it reports rests on four pieces of arithmetic, and these tests pin all four:

  classify()      three states out of a trinary PGM. The interesting case is the
                  one that cost a whole wrong run: the YAML's own thresholds put
                  the reserved unknown byte 205 in the FREE band (shade 0.196 vs
                  free_thresh 0.25), because save_map.py writes 205 for "neither"
                  and then hardcodes 0.25 beside it. Applied literally the
                  thresholds reported all 20 maps as fully known and Jaccard
                  silently became extent overlap. The reserved byte must win, and
                  a test here would have caught it.
  registration    registered origin = spawn + YAML origin, and the guard that
                  refuses a map_T_odom that is not identity.
  pair_metrics()  coverage and conflict, including that unknown never
                  contradicts anything, and the one-cell tolerance.
  the gates       three predicates that must be able to FAIL, so each is shown
                  failing here on doctored input, not just passing on good input.

The script lives in experiments/, which is not an importable package, so it is
loaded by location the way test_fit_world_transform.py loads the fitter.
Importing it costs numpy, yaml, pgm_agreement and fit_world_transform -- which
defers scipy, PIL and rosbag2_py into the functions that need them -- so there is
no ROS environment and no scipy here. That matters: CI's ros:jazzy container has
neither, and a module-level import of one is a COLLECTION error that takes the
whole nsk_swarm suite down rather than just these tests.

The one test that reaches PIL (classify against the fitter's own occupied set)
is marked accordingly, and the handful that read real maps skip when those files
are absent. Everything else is numpy over small hand-built arrays and MUST keep
running in CI -- those are the tests that catch the defects this file exists for.
"""

import importlib.util
import math
import os

import numpy as np
import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'robot_divergence.py')
MAPS_DIR = os.path.join(REPO_ROOT, 'experiments', 'maps')


def load_module():
    spec = importlib.util.spec_from_file_location('robot_divergence', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rd = load_module()

OCC, FREE, UNK = rd.OCC, rd.FREE, rd.UNKNOWN

needs_pil = pytest.mark.skipif(
    importlib.util.find_spec('PIL') is None,
    reason='PIL is deferred in fit_world_transform on purpose; absent here')


def grid(rows):
    """A three-state grid from characters: o occupied, f free, ? unknown."""
    table = {'o': OCC, 'f': FREE, '?': UNK}
    return np.array([[table[ch] for ch in row] for row in rows], dtype=np.uint8)


def have_maps(*stems):
    return all(os.path.isfile(os.path.join(MAPS_DIR, f'{s}.pgm'))
               and os.path.isfile(os.path.join(MAPS_DIR, f'{s}.yaml'))
               for s in stems)


# ───────────────────────────── classify() ────────────────────────────────────

def test_classify_recovers_the_three_reserved_bytes():
    px = np.array([[0, 254, 205]], dtype=np.uint8)
    got = rd.classify(px, negate=0, occupied_thresh=0.65, free_thresh=0.25)
    assert got.tolist() == [[OCC, FREE, UNK]]


def test_classify_does_not_let_the_thresholds_call_unknown_free():
    """The defect this file exists for, stated as a test.

    Under these very thresholds -- the ones save_map.py hardcodes into every one
    of these YAMLs -- byte 205 has shade 50/255 = 0.196, which is below
    free_thresh 0.25. A classifier that consults only the thresholds therefore
    calls every unknown cell free, reports the map as fully known, and turns
    Jaccard into extent overlap. Asserted on the numbers, so the test says WHY.
    """
    assert rd.shade(205.0, 0) == pytest.approx(50 / 255)
    assert rd.shade(205.0, 0) < 0.25            # the trap: inside the free band
    px = np.full((4, 5), 205, dtype=np.uint8)
    got = rd.classify(px, negate=0, occupied_thresh=0.65, free_thresh=0.25)
    assert (got == UNK).all()
    assert not (got == FREE).any()


def test_classify_honours_negate():
    px = np.array([[0, 254]], dtype=np.uint8)
    got = rd.classify(px, negate=1, occupied_thresh=0.65, free_thresh=0.25)
    assert got.tolist() == [[FREE, OCC]]        # the ends swap


def test_classify_puts_the_threshold_band_in_unknown():
    """A byte inside [free_thresh, occupied_thresh] is unknown on its own merits.

    Nothing in these 20 maps carries such a byte -- check_thresholds refuses a
    fourth value outright -- but the thresholds are still what decides for
    anything that is not a reserved byte, and this pins that.
    """
    px = np.array([[100, 150, 191]], dtype=np.uint8)   # shades 0.61, 0.41, 0.25
    got = rd.classify(px, negate=0, occupied_thresh=0.65, free_thresh=0.25)
    assert (got == UNK).all()


@needs_pil
@pytest.mark.skipif(not have_maps('b2maps_k0_cut1200_robot0'),
                    reason='experiments/maps/b2maps_k0_cut1200_robot0.* absent')
def test_classify_occupied_set_matches_the_fitter_on_a_real_map():
    """classify's occupied cells are the fitter's, cell for cell.

    The p_occ line is shared with fit_world_transform.load_map:276 by
    construction; this is what stops the two drifting apart. Only OCCUPIED is
    compared, because that is the only class load_map computes -- it never needs
    the free/unknown split classify adds.
    """
    # robot_divergence's own module-level sys.path inserts already put
    # experiments/analysis and experiments/slam on the path.
    from pathlib import Path

    import fit_world_transform as fwt
    from pgm_agreement import load

    m = fwt.load_map(0, stem='b2maps_k0_cut1200_robot')
    raw = load(Path(os.path.join(MAPS_DIR, 'b2maps_k0_cut1200_robot0.pgm')))
    mine = rd.classify(raw['px'], m['negate'], m['occupied_thresh'], 0.25)

    theirs = np.zeros(mine.shape, dtype=bool)
    theirs[m['rows'], m['cols']] = True
    assert np.array_equal(mine == OCC, theirs)
    assert int(theirs.sum()) == m['n_occupied']


# ──────────────────────────── check_thresholds() ─────────────────────────────

def test_check_thresholds_reports_that_205_is_not_recovered():
    info = rd.check_thresholds(0, 0.65, 0.25, other=0, where='t')
    assert info['unknown_byte_recovered_by_thresholds'] is False
    assert info['unknown_byte_band_under_thresholds'] == 'free'
    assert info['threshold_unknown_band_bytes'] == [90, 191]


def test_check_thresholds_dies_on_a_yaml_that_does_not_describe_its_pgm():
    # negate=1 makes byte 0 read as free, i.e. the reserved occupied byte is not
    # occupied, i.e. this YAML belongs to some other image.
    with pytest.raises(SystemExit) as exc:
        rd.check_thresholds(1, 0.65, 0.25, other=0, where='t')
    assert exc.value.code == 2


def test_check_thresholds_dies_on_a_fourth_pixel_value():
    with pytest.raises(SystemExit) as exc:
        rd.check_thresholds(0, 0.65, 0.25, other=7, where='t')
    assert exc.value.code == 2


def test_check_thresholds_dies_on_unordered_thresholds():
    with pytest.raises(SystemExit) as exc:
        rd.check_thresholds(0, 0.25, 0.65, other=0, where='t')
    assert exc.value.code == 2


# ──────────────────────────── coverage arithmetic ────────────────────────────

def test_jaccard_and_only_counts():
    #        both known      A only      B only      neither
    a = grid(['of', 'f?'])
    b = grid(['of', '?f'])
    met = rd.pair_metrics(a, b)
    assert met['known_a'] == 3 and met['known_b'] == 3
    assert met['known_both'] == 2                      # the top row
    assert met['known_union'] == 4
    assert met['jaccard'] == pytest.approx(2 / 4)
    assert met['only_a'] == 1 and met['only_b'] == 1


def test_identical_grids_agree_perfectly():
    a = grid(['oof', 'f?f', 'ooo'])
    met = rd.pair_metrics(a, a.copy())
    assert met['jaccard'] == 1.0
    assert met['conflict_strict'] == 0
    assert met['conflict_tolerant'] == 0
    assert met['only_a'] == 0 and met['only_b'] == 0


def test_unknown_never_contradicts_and_never_enters_both():
    """A cell one robot never committed is a coverage gap, not a disagreement.

    The two findings are reported separately and must not leak into each other:
    charging a contradiction for a cell nobody claimed would make every partial
    map look wrong rather than incomplete.
    """
    a = grid(['o?f'])
    b = grid(['?o?'])
    met = rd.pair_metrics(a, b)
    assert met['known_both'] == 0
    assert met['conflict_strict'] == 0
    assert met['conflict_tolerant'] == 0
    assert math.isnan(met['conflict_strict_pct'])      # no denominator to speak of


def test_strict_conflict_counts_both_directions():
    a = grid(['of'])
    b = grid(['fo'])
    met = rd.pair_metrics(a, b)
    assert met['known_both'] == 2
    assert met['conflict_strict'] == 2
    assert met['a_occ_b_free'] == 1
    assert met['a_free_b_occ'] == 1
    assert met['conflict_strict_pct'] == pytest.approx(100.0)


def test_free_on_free_and_occupied_on_occupied_are_not_conflicts():
    a = grid(['offo'])
    met = rd.pair_metrics(a, a.copy())
    assert met['known_both'] == 4
    assert met['conflict_strict'] == 0


# ───────────────────────────── dilate() ──────────────────────────────────────

def test_dilate_is_the_3x3_block():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    got = rd.dilate(mask, 1)
    assert int(got.sum()) == 9
    assert got[1:4, 1:4].all()


def test_dilate_does_not_wrap_around_any_edge():
    """A shift must move cells OFF the array, not around it.

    np.roll would pass every other test in this file and silently forgive a
    conflict on the far side of the map.
    """
    for corner in ((0, 0), (0, 4), (4, 0), (4, 4)):
        mask = np.zeros((5, 5), dtype=bool)
        mask[corner] = True
        got = rd.dilate(mask, 1)
        assert int(got.sum()) == 4          # a corner has 3 neighbours, not 8
        assert not got[2, 2]                # nothing appeared across the array


def test_dilate_by_zero_is_the_identity():
    mask = np.array([[True, False], [False, True]])
    assert np.array_equal(rd.dilate(mask, 0), mask)


# ───────────────────── the one-cell tolerance ────────────────────────────────

def test_tolerance_forgives_a_wall_one_cell_over_in_both_directions():
    """Both robots found the wall; they rasterised it one cell apart.

    Checked in both directions, because a tolerance applied to only one map's
    occupied cells would pass a one-sided test and then report a conflict that
    depends on the order the pair is listed in.
    """
    a = grid(['ffof', 'ffof'])
    b = grid(['foff', 'foff'])
    met = rd.pair_metrics(a, b)
    assert met['conflict_strict'] == 4          # 2 rows x 2 disagreeing cells
    assert met['conflict_tolerant'] == 0
    assert met['forgiven'] == 4

    swapped = rd.pair_metrics(b, a)
    assert swapped['conflict_strict'] == met['conflict_strict']
    assert swapped['conflict_tolerant'] == 0


def test_tolerance_does_not_forgive_two_cells():
    a = grid(['ffofff'])
    b = grid(['offfff'])
    met = rd.pair_metrics(a, b)
    assert met['conflict_strict'] == 2
    assert met['conflict_tolerant'] == 2       # nothing within one cell
    assert met['forgiven'] == 0


def test_tolerance_does_not_forgive_a_wall_that_is_simply_absent():
    """One robot says wall, the other says open floor and found no wall nearby.

    This is the disagreement the measure exists to report, and the tolerance must
    leave it alone -- otherwise "tolerant conflict stays small" is vacuous.
    """
    a = grid(['fffof', 'fffff'])
    b = grid(['fffff', 'fffff'])
    met = rd.pair_metrics(a, b)
    assert met['conflict_strict'] == 1
    assert met['conflict_tolerant'] == 1
    assert met['forgiven'] == 0


def test_tolerant_never_exceeds_strict():
    rng = np.random.default_rng(20260929)
    for _ in range(40):
        a = rng.choice([OCC, FREE, UNK], size=(12, 14)).astype(np.uint8)
        b = rng.choice([OCC, FREE, UNK], size=(12, 14)).astype(np.uint8)
        met = rd.pair_metrics(a, b)
        assert met['conflict_tolerant'] <= met['conflict_strict']
        assert met['forgiven'] == (met['conflict_strict']
                                   - met['conflict_tolerant'])


def test_a_rigid_one_cell_shift_is_entirely_forgiven():
    """The shift gate's prediction, on a grid small enough to read by eye.

    Every occupied cell of the shifted copy sits one cell from the original's, so
    a correct tolerance forgives all of it -- which is why the gate on real maps
    demands exactly 0 rather than "a few".
    """
    a = grid(['oofoo', 'ofofo', 'ooooo'])
    b = np.roll(a, 1, axis=1)
    b[:, 0] = a[:, 0]                       # no wraparound: hold the new column
    met = rd.pair_metrics(a, b)
    assert met['conflict_strict'] > 0
    assert met['conflict_tolerant'] == 0


# ─────────────────────────── registration ────────────────────────────────────

def test_registered_origin_is_spawn_plus_yaml_origin():
    m = {'yaml_origin': [-6.886, -4.321], 'origin': [0.0, 0.0],
         'stem': 'm', 'resolution': 0.1}
    got = rd.reregistered(m, (0.86, 0.28))
    assert got['origin'][0] == pytest.approx(0.86 - 6.886)
    assert got['origin'][1] == pytest.approx(0.28 - 4.321)


def test_swapping_two_spawns_moves_the_maps_in_opposite_directions():
    """The swap gate's premise: the pair separates by TWICE the spawn gap.

    Robots 0 and 2, whose spawns differ by 1.72 m in x and not at all in y --
    which is also why their swap is the axis-aligned case the pre-registered pp
    criterion could not describe.
    """
    s_i, s_j = (0.86, 0.28), (-0.86, 0.28)
    mi = {'yaml_origin': [0.0, 0.0], 'stem': 'i', 'resolution': 0.1}
    mj = {'yaml_origin': [0.0, 0.0], 'stem': 'j', 'resolution': 0.1}
    correct_gap = (rd.reregistered(mi, s_i)['origin'][0]
                   - rd.reregistered(mj, s_j)['origin'][0])
    swapped_gap = (rd.reregistered(mi, s_j)['origin'][0]
                   - rd.reregistered(mj, s_i)['origin'][0])
    assert correct_gap == pytest.approx(1.72)
    assert swapped_gap == pytest.approx(-1.72)          # the gap reverses
    assert abs(swapped_gap - correct_gap) == pytest.approx(2 * abs(s_i[0] - s_j[0]))
    assert abs(swapped_gap - correct_gap) == pytest.approx(3.44)
    assert rd.axis_aligned(2 * (s_j[0] - s_i[0]), 2 * (s_j[1] - s_i[1]))


def test_shifted_moves_the_origin_by_whole_cells():
    m = {'origin': [1.0, 2.0], 'resolution': 0.1, 'stem': 'm'}
    got = rd.shifted(m, dx_cells=1, dy_cells=-2)
    assert got['origin'][0] == pytest.approx(1.1)
    assert got['origin'][1] == pytest.approx(1.8)


def test_map_to_odom_guard_accepts_identity_and_refuses_anything_else():
    rd.check_map_to_odom((0.0, -0.0, 0.0), 'ok')          # returns, no exit
    for bad in ((0.5, 0.0, 0.0), (0.0, 0.5, 0.0), (0.0, 0.0, 0.01)):
        with pytest.raises(SystemExit) as exc:
            rd.check_map_to_odom(bad, 'bad')
        assert exc.value.code == 2


def test_axis_aligned_flags_exactly_the_displacements_along_a_wall():
    assert rd.axis_aligned(-3.44, 0.0)          # pair 0-2's swap
    assert rd.axis_aligned(2.12, 0.0)           # pair 3-4's swap
    assert rd.axis_aligned(0.0, 1.5)
    assert not rd.axis_aligned(-1.72, 1.24)     # pair 0-1's swap
    assert not rd.axis_aligned(-2.78, -2.02)


# ─────────────────────────── the common grid ─────────────────────────────────

def test_common_grid_is_anchored_on_the_world_origin():
    """A cell boundary falls exactly on x = 0 and y = 0, whatever the map set.

    That is what makes the grid a property of the world: adding a map extends the
    extent without moving a single existing sample.
    """
    res = 0.1
    maps = [{'origin': [-6.0262, -4.0407], 'width': 121, 'height': 81,
             'resolution': res, 'stem': 'a'},
            {'origin': [-9.9937, -4.4822], 'width': 160, 'height': 124,
             'resolution': res, 'stem': 'b'}]
    g = rd.common_grid(maps)
    assert (g['x0'] / res) == pytest.approx(round(g['x0'] / res))
    assert (g['y0'] / res) == pytest.approx(round(g['y0'] / res))
    assert g['x0'] <= min(m['origin'][0] for m in maps)
    assert g['y0'] <= min(m['origin'][1] for m in maps)
    # and it reaches the far corner of every map
    assert g['x0'] + g['w'] * res >= max(m['origin'][0] + m['width'] * res
                                         for m in maps) - 1e-9
    assert g['y0'] + g['h'] * res >= max(m['origin'][1] + m['height'] * res
                                         for m in maps) - 1e-9


def test_common_grid_refuses_a_mismatched_resolution():
    maps = [{'origin': [0.0, 0.0], 'width': 4, 'height': 4, 'resolution': 0.1,
             'stem': 'a'},
            {'origin': [0.0, 0.0], 'width': 4, 'height': 4, 'resolution': 0.05,
             'stem': 'b'}]
    with pytest.raises(SystemExit) as exc:
        rd.common_grid(maps)
    assert exc.value.code == 2


def test_on_grid_places_a_synthetic_map_where_the_registration_says():
    """One round trip through the imported resampler, end to end.

    Convention A lives in pgm_agreement.sample's single row flip, so this also
    pins the vertical orientation: the occupied cell is written in PGM ROW 0,
    which is MAXIMUM y, and must come back at the TOP of the bottom-up grid.
    """
    res = 0.1
    px = np.array([[OCC, FREE],
                   [FREE, UNK]], dtype=np.uint8)
    m = {'origin': [0.0, 0.0], 'width': 2, 'height': 2, 'resolution': res,
         'px': px, 'stem': 'm'}
    g = {'res': res, 'x0': -0.1, 'y0': -0.1, 'w': 4, 'h': 4}
    out = rd.on_grid(m, g)

    assert out.shape == (4, 4)
    # Row 0 of the output is minimum y; the map occupies output rows 1..2.
    assert out[2, 1] == OCC          # PGM row 0 col 0 -> top-left of the map
    assert out[2, 2] == FREE
    assert out[1, 1] == FREE
    assert out[1, 2] == UNK
    # Everything outside the map is unknown, by sample()'s own fill.
    assert (out[0, :] == UNK).all() and (out[3, :] == UNK).all()
    assert (out[:, 0] == UNK).all() and (out[:, 3] == UNK).all()


def test_subcell_offset_is_zero_on_the_lattice_and_half_at_worst():
    g = {'res': 0.1, 'x0': 0.0, 'y0': 0.0, 'w': 10, 'h': 10}
    assert rd.subcell_offset({'origin': [0.5, 0.3]}, g) == pytest.approx(0.0)
    assert rd.subcell_offset({'origin': [0.55, 0.3]}, g) == pytest.approx(0.5,
                                                                          abs=1e-6)
    assert rd.subcell_offset({'origin': [0.53, 0.3]}, g) == pytest.approx(0.3,
                                                                          abs=1e-6)


# ─────────────────── the gates, each shown able to fail ──────────────────────

def test_self_gate_passes_only_on_a_perfect_match():
    good = rd.pair_metrics(grid(['of?', 'foo']), grid(['of?', 'foo']))
    assert rd.self_gate(good)['pass'] is True


@pytest.mark.parametrize('doctored,why', [
    ({'jaccard': 0.999, 'conflict_strict': 0, 'conflict_tolerant': 0},
     'a thousandth of a Jaccard short is still not the same map'),
    ({'jaccard': 1.0, 'conflict_strict': 1, 'conflict_tolerant': 0},
     'one strict conflict against itself'),
    ({'jaccard': 1.0, 'conflict_strict': 0, 'conflict_tolerant': 1},
     'one tolerant conflict against itself'),
])
def test_self_gate_fails_on_doctored_input(doctored, why):
    assert rd.self_gate(doctored)['pass'] is False, why


def test_shift_gate_needs_both_halves():
    assert rd.shift_gate({'conflict_strict': 300,
                          'conflict_tolerant': 0})['pass'] is True
    # a tolerance that forgives everything, including a real disagreement
    assert rd.shift_gate({'conflict_strict': 0,
                          'conflict_tolerant': 0})['pass'] is False
    # a tolerance that forgives nothing, i.e. is not being applied
    assert rd.shift_gate({'conflict_strict': 300,
                          'conflict_tolerant': 1})['pass'] is False


def test_swap_gate_reports_both_criteria_whichever_is_gated():
    """The pre-registered pp verdict is always present, whatever sets the exit.

    Pair 0-2's measured numbers: 0.329% strict correctly registered, 4.128% with
    the two spawns exchanged. That is a rise of 3.80 pp -- under the
    pre-registered 5 pp floor -- and 12.5x. A run gated on the ratio must still
    report that the pp criterion failed, or the record of what was predicted
    first is lost.

    The arithmetic is asserted against the inputs rather than against numbers
    typed in here, so the test pins the gate's behaviour and cannot fail on a
    stale hand computation of its own.
    """
    correct, swapped = 0.329, 4.128

    g = rd.swap_gate(correct, swapped, criterion='ratio')
    assert g['rise_pp'] == pytest.approx(swapped - correct)
    assert g['rise_ratio'] == pytest.approx(swapped / correct)
    assert g['rise_pp'] < rd.SWAP_MIN_RISE_PP          # the pre-registered miss
    assert g['rise_ratio'] >= rd.SWAP_MIN_RISE_RATIO
    assert g['pp_pass'] is False
    assert g['ratio_pass'] is True
    assert g['pass'] is True

    on_pp = rd.swap_gate(correct, swapped, criterion='pp')
    assert on_pp['pass'] is False
    assert on_pp['ratio_pass'] is True          # still reported

    both = rd.swap_gate(correct, swapped, criterion='both')
    assert both['pass'] is False


def test_swap_gate_fails_when_nothing_moved():
    """The --break-gate swap case: spawns not exchanged, so no rise at all."""
    for criterion in rd.SWAP_CRITERIA:
        g = rd.swap_gate(0.583, 0.583, criterion=criterion)
        assert g['pass'] is False
        assert g['rise_pp'] == pytest.approx(0.0)
        assert g['rise_ratio'] == pytest.approx(1.0)


def test_swap_gate_rejects_an_unknown_criterion():
    with pytest.raises(ValueError):
        rd.swap_gate(1.0, 9.0, criterion='whatever')


def test_swap_gate_ratio_is_infinite_rather_than_dividing_by_zero():
    g = rd.swap_gate(0.0, 4.0, criterion='ratio')
    assert math.isinf(g['rise_ratio'])
    assert g['pass'] is True


# ───────────────── prediction evaluators, able to say no ─────────────────────

def test_rises_accepts_non_decreasing_and_rejects_a_dip():
    assert rd.rises([0.40, 0.48, 0.76, 1.0]) is True
    assert rd.rises([1.0, 1.0, 1.0, 1.0]) is True          # flat is not a fall
    # pair 0-4's real Jaccard trend: it dips at 120 s and the detector must say so
    assert rd.rises([0.5758, 0.2727, 0.6000, 1.0]) is False


def test_rises_is_not_vacuously_true_on_a_nan():
    """A NaN means a pair had no known cells in common, not that it improved."""
    assert rd.rises([0.4, float('nan'), 0.9, 1.0]) is False


def test_stays_small_is_an_upper_bound_and_a_nan_is_not_small():
    assert rd.stays_small([0.05, 0.36, 0.82, 0.31], limit=1.0) is True
    assert rd.stays_small([0.10, 1.52, 1.53, 0.24], limit=1.0) is False
    assert rd.stays_small([1.0, 1.0], limit=1.0) is True           # inclusive
    assert rd.stays_small([0.1, float('nan')], limit=1.0) is False


def test_preregistered_constants_are_the_ones_ali_fixed():
    """A guard on the numbers themselves, so a later edit has to be deliberate.

    SMALL_TOLERANT_PCT and the all-ten-pairs swap scope were chosen before the
    script was written or run; SWAP_MIN_RISE_PP is the pre-registered swap
    criterion that failed and is kept anyway. If any of these move, the claim
    that they were not tuned toward an output stops being true, and this test is
    where that has to be argued rather than noticed later.
    """
    assert rd.SMALL_TOLERANT_PCT == 1.0
    assert rd.SWAP_MIN_RISE_PP == 5.0
    assert rd.TOL_CELLS == 1
    assert rd.SHIFT_TOLERANT_MAX == 0
    assert len(rd.pairs()) == 10
    assert rd.CUTS == (60, 120, 240, 1200)
