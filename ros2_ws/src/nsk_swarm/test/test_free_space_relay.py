"""Unit tests for nsk_swarm/free_space_relay.py — the two pure functions.

The relay exists because Karto ignores a beam that hits nothing, so the floor
that beam passed over never becomes known. Its whole correctness is one
inequality: the value a no-return beam is rewritten to must sit STRICTLY
between slam_toolbox's max_laser_range (Karto's rangeThreshold) and the scan's
range_max (Karto's maxRange). Karto.h:6148-6191 is what makes that the
criterion:

    rangeReading >= maxRange        -> dropped whole, nothing cleared
    rangeThreshold <= r < maxRange  -> traced as free space, no obstacle
    r < rangeThreshold - 1e-6       -> traced, and the endpoint IS an obstacle

Land on the wrong side of either bound and the relay either does nothing at
all (>= maxRange) or paints a phantom wall one fill-value out across every beam
that hit nothing. Both failures look like a working relay from the outside,
which is why they are tested here rather than eyeballed on a map.

TWO PAIRS OF BOUNDS APPEAR BELOW, and the distinction is deliberate.
fill_value and rewrite_ranges take both bounds as arguments and hardcode
neither, so they are range-agnostic and the tests are free to pick either pair.

  DEPLOYED_*  what the sim runs now: an LDS-02-class 8.0 m range_max against
              explore.launch.py's RELAY_RANGE_THRESHOLD of 7.9.
  RANGE_MAX / THRESHOLD
              3.5 / 3.4 -- the burger's stock LDS-01 as b2maps_e0t RECORDED
              it. Kept, and kept as the default fixture, because several tests
              below assert facts about that bag (4006 readings at exactly
              range_max; 57.47% no-returns out of 11708640 beams). Those are
              historical measurements of a file on disk: they do not move when
              the sensor moves, and rewriting them to 8.0 would turn a checked
              fact into a fabricated one.
"""
import math

import pytest

from nsk_swarm.free_space_relay import fill_value, rewrite_ranges

# The burger's stock LDS-01 as b2maps_e0t records it, and the threshold the
# launch files paired with it at the time. See the module docstring.
RANGE_MAX = 3.5
THRESHOLD = 3.4

# What swarm_sim.launch.py (BURGER_RANGE_MAX) and explore.launch.py
# (RELAY_RANGE_THRESHOLD) set today. test_launch_descriptions.py is what ties
# these literals to those files; here they only have to be a valid pair.
DEPLOYED_RANGE_MAX = 8.0
DEPLOYED_THRESHOLD = 7.9


# ── the fill value ───────────────────────────────────────────────────────────

def test_the_fill_is_the_midpoint_of_the_two_bounds():
    assert fill_value(THRESHOLD, RANGE_MAX) == pytest.approx(3.45)


def test_the_fill_is_strictly_inside_both_bounds():
    """The inequality the whole node rests on, asserted as an inequality."""
    fill = fill_value(THRESHOLD, RANGE_MAX)
    assert fill >= THRESHOLD          # traced at all
    assert fill < RANGE_MAX           # not dropped as a no-return
    assert fill > THRESHOLD - 1e-6    # and NOT marked as an obstacle


def test_equal_bounds_leave_nowhere_to_put_the_fill():
    """max_laser_range 3.5 against range_max 3.5 -- the configuration every
    run before this one used, and the reason its no-return beams cleared
    nothing. There is no value that is both traced and not dropped, so the
    relay must say so rather than pick one.
    """
    assert fill_value(3.5, 3.5) is None


def test_a_threshold_above_range_max_is_refused_too():
    assert fill_value(4.0, 3.5) is None


def test_a_wider_gap_still_lands_between_the_bounds():
    fill = fill_value(2.0, 10.0)
    assert 2.0 <= fill < 10.0


def test_the_deployed_pair_lands_where_karto_traces_it():
    """The bounds the sim actually runs: 7.9 against an 8.0 m range_max.

    The same inequality as the 3.4/3.5 pair, at the range that is live. Worth
    its own test rather than a reparametrised one: this is the pair a run uses,
    and the fill it produces (7.95 m) is the number R2 of the rehearsal greps
    for in the relay's log line.
    """
    fill = fill_value(DEPLOYED_THRESHOLD, DEPLOYED_RANGE_MAX)

    assert fill == pytest.approx(7.95)
    assert fill >= DEPLOYED_THRESHOLD              # traced at all
    assert fill < DEPLOYED_RANGE_MAX               # not dropped as a no-return
    assert fill > DEPLOYED_THRESHOLD - 1e-6        # and NOT marked an obstacle


def test_the_deployed_range_max_still_refuses_the_old_threshold_as_equal():
    """8.0 against 8.0 is the same dead configuration 3.5 against 3.5 was.

    Raising the sensor does not make the relay unnecessary and does not make
    the equal-bounds case safe: at 8 m, 20.5% of e0t-trajectory beams still
    return nothing, and with the threshold left at range_max every one of them
    would clear nothing exactly as before.
    """
    assert fill_value(DEPLOYED_RANGE_MAX, DEPLOYED_RANGE_MAX) is None


def test_a_beam_that_reaches_the_old_ceiling_is_now_a_genuine_hit():
    """3.5 m used to be a no-return; at an 8 m range_max it is a wall.

    The regression this guards is a fill value or a range_max left behind at
    3.5 after the sensor moved: every reading between 3.5 and 8.0 would be
    rewritten, and 58% of the map's real walls would be erased into free space.
    """
    out, n = rewrite_ranges([3.5, 5.0, 7.89], DEPLOYED_RANGE_MAX, 7.95)

    assert out == [3.5, 5.0, 7.89]
    assert n == 0


# ── the rewrite ──────────────────────────────────────────────────────────────

FILL = 3.45


def test_infinite_beams_are_filled():
    out, n = rewrite_ranges([math.inf, math.inf], RANGE_MAX, FILL)
    assert out == [FILL, FILL]
    assert n == 2


def test_a_reading_exactly_at_range_max_is_filled():
    """4006 of b2maps_e0t's beams come through as exactly 3.5, and Karto's
    test is `>=`, so they are dropped alongside the infinities.
    """
    out, n = rewrite_ranges([RANGE_MAX], RANGE_MAX, FILL)
    assert out == [FILL] and n == 1


def test_a_reading_above_range_max_is_filled():
    out, n = rewrite_ranges([3.6], RANGE_MAX, FILL)
    assert out == [FILL] and n == 1


def test_nan_is_left_alone():
    """A NaN is not "nothing within range", it is a reading the sensor could
    not produce. None appear in 11.7M beams of b2maps_e0t; inventing free
    space from one would be inventing data.
    """
    out, n = rewrite_ranges([math.nan], RANGE_MAX, FILL)
    assert math.isnan(out[0])
    assert n == 0


def test_genuine_hits_pass_through_untouched():
    hits = [0.5, 1.0, 2.75, 3.39, 3.4999]
    out, n = rewrite_ranges(hits, RANGE_MAX, FILL)
    assert out == hits
    assert n == 0


def test_a_reading_below_range_min_is_not_a_no_return():
    """Karto drops it for its own reason (<= minRange). It is a reading, not
    an absence of one, and the relay does not touch it.
    """
    out, n = rewrite_ranges([0.05], RANGE_MAX, FILL)
    assert out == [0.05] and n == 0


def test_a_mixed_scan_is_counted_correctly():
    scan = [math.inf, 1.2, math.nan, RANGE_MAX, 3.0, math.inf]
    out, n = rewrite_ranges(scan, RANGE_MAX, FILL)

    assert n == 3                                   # two inf, one at range_max
    assert out[0] == FILL and out[5] == FILL
    assert out[3] == FILL
    assert out[1] == 1.2 and out[4] == 3.0
    assert math.isnan(out[2])
    assert len(out) == len(scan)


def test_the_input_list_is_not_mutated():
    """The incoming message is shared with every other subscriber in the
    process; rewriting in place would change what they see.
    """
    scan = [math.inf, 1.0]
    out, _ = rewrite_ranges(scan, RANGE_MAX, FILL)
    assert scan == [math.inf, 1.0]
    assert out is not scan


def test_the_e0t_fraction_comes_out_of_the_rule():
    """57.47% of b2maps_e0t's beams are no-returns: 6725417 infinite and 4006
    at exactly range_max, out of 11708640. The rule must count both, or the
    number the whole change is justified by is not the number it produces.
    """
    sample = ([math.inf] * 6725 + [RANGE_MAX] * 4 + [1.5] * 4979)
    _out, n = rewrite_ranges(sample, RANGE_MAX, FILL)
    assert n / len(sample) == pytest.approx(0.5747, abs=5e-4)
