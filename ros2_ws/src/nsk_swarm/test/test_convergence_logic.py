"""Layer 1 — the monitor's convergence decision as pure math (no rclpy init).

Exercises nsk_swarm.convergence_monitor.convergence_step, which was
extracted from the _monitor_cb timer callback so the decision is testable
without a node, a service, or a spinning executor.
"""

import pytest

from nsk_swarm.convergence_monitor import convergence_step

# Defaults declared by ConvergenceMonitorNode
MIN_RISE = 0.03
STAB_EPS = 0.005


def run_readings(readings, has_merges):
    """Feed a sequence of mean_sim readings through convergence_step with
    the same bookkeeping _monitor_cb does (baseline = first reading, delta
    from the previous reading). Returns True once any cycle converges."""
    baseline = None
    history = []
    consec_stable = 0
    converged = False
    for mean_sim in readings:
        if baseline is None:
            baseline = mean_sim
        history.append(mean_sim)
        has_prev = len(history) > 1
        delta = mean_sim - history[-2] if has_prev else 0.0
        _, consec_stable, newly = convergence_step(
            mean_sim, baseline, delta, has_prev, consec_stable,
            converged, has_merges, MIN_RISE, STAB_EPS)
        if newly:
            converged = True
    return converged


# ── Sequence-level scenarios ─────────────────────────────────────────────────

def test_high_baseline_no_merges_never_converges():
    # A baseline well above the old absolute convergence_threshold (0.25),
    # perfectly stable, but with zero merges observed: must NOT converge.
    assert run_readings([0.9] * 10, has_merges=False) is False


def test_high_baseline_stable_with_merges_but_no_rise_never_converges():
    # Same high stable readings WITH merges: rise over baseline is 0.0,
    # below min_rise, so still not converged (baseline varies per run — an
    # absolute threshold could be met at t=0).
    assert run_readings([0.9] * 10, has_merges=True) is False


def test_rise_plus_three_stable_cycles_plus_merges_converges():
    # Rise of 0.05 >= min_rise, then a stable tail: deltas 0.001 < eps for
    # 3 consecutive cycles, merges present -> converged.
    readings = [0.10, 0.15, 0.151, 0.152, 0.153]
    assert run_readings(readings, has_merges=True) is True


def test_rise_without_stability_does_not_converge():
    # Keeps rising by 0.05 per cycle: rise is large and merges are present,
    # but every delta exceeds stability_eps so the stable counter never
    # reaches 3.
    readings = [0.10 + 0.05 * i for i in range(10)]
    assert run_readings(readings, has_merges=True) is False


def test_stability_without_merges_does_not_converge():
    # The exact sequence that converges with merges must not converge
    # without them.
    readings = [0.10, 0.15, 0.151, 0.152, 0.153]
    assert run_readings(readings, has_merges=False) is False


# ── Single-cycle edge cases ──────────────────────────────────────────────────

def test_rise_exactly_min_rise_counts():
    rise, consec, newly = convergence_step(
        mean_sim=0.53, baseline_sim=0.50, delta=0.001, has_prev=True,
        consec_stable=2, already_converged=False, has_merges=True,
        min_rise=0.03, stability_eps=0.005)
    assert rise == pytest.approx(0.03)
    assert consec == 3
    assert newly is True


def test_unstable_delta_resets_stable_counter():
    _, consec, newly = convergence_step(
        mean_sim=0.60, baseline_sim=0.50, delta=0.02, has_prev=True,
        consec_stable=2, already_converged=False, has_merges=True,
        min_rise=0.03, stability_eps=0.005)
    assert consec == 0
    assert newly is False


def test_first_reading_has_no_stability_credit():
    # has_prev=False (first cycle): counter resets regardless of delta.
    _, consec, newly = convergence_step(
        mean_sim=0.50, baseline_sim=0.50, delta=0.0, has_prev=False,
        consec_stable=5, already_converged=False, has_merges=True,
        min_rise=0.03, stability_eps=0.005)
    assert consec == 0
    assert newly is False


def test_already_converged_never_retriggers():
    _, _, newly = convergence_step(
        mean_sim=0.60, baseline_sim=0.50, delta=0.001, has_prev=True,
        consec_stable=3, already_converged=True, has_merges=True,
        min_rise=0.03, stability_eps=0.005)
    assert newly is False
