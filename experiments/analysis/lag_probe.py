#!/usr/bin/env python3
"""lag_probe.py -- separate a timing offset from a real scale error.

Reads the *_samples.csv files written by rate_profile.py. No bag, no
simulator, runs in seconds.

THE MODEL

  If the odometry stream is offset from the ground-truth stream by tau
  seconds, then over an interval the difference between the two yaw
  increments is

      excess_i  ~=  -tau * (omega_{i+1} - omega_{i-1})/2

  i.e. it is proportional to the CHANGE in turn rate, not to the rotation
  itself. Such a term telescopes: summed over a whole run it is close to
  zero, which is why a net-error lag scan cannot see it. Inside a rate or
  speed bin it does not cancel, which is why the per-bin percentages are
  enormous and alternate in sign.

  A real scale error -- wheel slip, wrong wheel radius, wrong separation --
  is instead proportional to the rotation itself:

      excess_i  ~=  b * d_truth_i

  So fit both at once:

      excess_i  =  a * dOmega_i  +  b * d_truth_i,   tau = -a

  b is the quantity b18 set out to measure. a is the artefact standing in
  front of it.

WHAT WOULD FALSIFY THIS

  A poor fit. If R^2 is small, or if the two runs return different a, the
  timing hypothesis is wrong and the residue has another cause. The script
  prints R^2 for the joint fit and for each single-parameter fit, so a
  timing term that explains nothing cannot hide behind a slip term that
  does.

  This linear model is a first-order approximation, valid while tau is
  small against the timescale on which omega changes. If it fits, confirm
  it by re-reading the bag with the truth series shifted by the fitted tau
  and checking that the summed |excess| collapses -- the approximation is
  evidence, not proof.

Usage:
  python3 lag_probe.py experiments/logs/b18/analysis/run2_robot0_samples.csv \
                       experiments/logs/b18/analysis/run3_robot0_samples.csv
"""

from __future__ import annotations

import os
import sys

import numpy as np


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    need = ("t_sim", "dt", "d_truth_deg", "d_odom_deg", "excess_deg")
    missing = [c for c in need if c not in d.dtype.names]
    if missing:
        sys.exit("ABORT: %s lacks columns %s" % (path, ", ".join(missing)))
    return d


def r2(y, resid):
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def chunk_abs_excess(t, e, chunk_s=10.0):
    """Summed |excess| over fixed chunks -- the statistic that exposed this."""
    idx = np.floor((t - t[0]) / chunk_s).astype(int)
    return float(np.sum(np.abs(np.bincount(idx, weights=e))))


def exact_scan(t_mid, dt, d_truth, d_odom, taus, chunk_s=10.0, guard=2.0):
    """Shift the truth series by tau and re-difference it. Exact, not
    first-order: the cumulative yaw is rebuilt and resampled at the shifted
    times. A guard band is dropped at both ends, where np.interp clamps."""
    left = t_mid - 0.5 * dt
    edges = np.concatenate([left, [t_mid[-1] + 0.5 * dt[-1]]])
    cum_t = np.concatenate([[0.0], np.cumsum(d_truth)])
    out = []
    for tau in taus:
        shifted = np.interp(edges - tau, edges, cum_t)
        d_t = np.diff(shifted)
        exc = d_odom - d_t
        m = ((t_mid > edges[0] + guard + abs(tau)) &
             (t_mid < edges[-1] - guard - abs(tau)))
        # least-squares scale on the shifted series: the reconciled number
        b = float(np.sum(exc[m] * d_t[m]) / np.sum(d_t[m] ** 2))
        out.append((float(tau), chunk_abs_excess(t_mid[m], exc[m], chunk_s),
                    float(np.sum(exc[m])), 100.0 * b))
    return out


def fit(path, chunk_s=10.0, trim=0.01):
    d = load(path)
    t_full = d["t_sim"]
    dt = d["dt"]
    d_truth = d["d_truth_deg"]
    d_odom = d["d_odom_deg"]
    excess = d["excess_deg"]

    omega = d_truth / dt                      # signed deg/s

    # The regressor is the change in turn rate across an interval. Truth is
    # sampled at 20 Hz and odom at 50 Hz, so a one-sample difference is
    # mostly interpolation staircase; that measurement error attenuates the
    # fitted slope. Sweep the stencil width instead of picking one. On
    # synthetic data with an injected 60 ms offset and -0.300 % scale, k=1
    # returned 43.6 ms and k=4-5 returned 59-61 ms, while the scale estimate
    # held at -0.32 % throughout. Stability of the estimates across k is
    # itself part of the evidence.
    def stencil(k):
        p = np.zeros_like(omega)
        p[k:-k] = (omega[2 * k:] - omega[:-2 * k]) / (2.0 * k)
        s = slice(k, -k)
        A_k = np.column_stack([p[s], d_truth[s]])
        y_k = excess[s]
        c, *_ = np.linalg.lstsq(A_k, y_k, rcond=None)
        res = y_k - A_k @ c
        return A_k, y_k, c, res, s

    print("\n" + "=" * 70)
    print(os.path.basename(path))
    print("=" * 70)
    print("STENCIL SWEEP   (k = half-width of the rate difference, samples)")
    print("%4s %10s %10s %9s" % ("k", "tau ms", "scale %", "R2"))
    widths = (1, 2, 3, 4, 5, 8, 12, 20, 30, 50, 75, 100)
    swept = {}
    for k in widths:
        A_k, y_k, c, res, _ = stencil(k)
        swept[k] = (float(-c[0] * 1000.0), float(100.0 * c[1]),
                    r2(y_k, res))
        print("%4d %10.1f %10.3f %9.4f" % (k, swept[k][0], swept[k][1],
                                           swept[k][2]))
    best_k = max(widths, key=lambda k: swept[k][2])
    print("  best fit at k = %d" % best_k)

    A, y, coef, resid, interior = stencil(best_k)
    t = t_full[interior]
    a, b = float(coef[0]), float(coef[1])

    # single-parameter fits, so neither term can hide behind the other
    a_only = float(np.sum(y * A[:, 0]) / np.sum(A[:, 0] ** 2))
    b_only = float(np.sum(y * A[:, 1]) / np.sum(A[:, 1] ** 2))
    r2_a = r2(y, y - a_only * A[:, 0])
    r2_b = r2(y, y - b_only * A[:, 1])
    r2_ab = r2(y, resid)

    # trimmed refit: drop the largest |excess| samples
    if trim > 0:
        k = max(1, int(trim * y.size))
        keep = np.argsort(np.abs(y))[:-k]
        coef_t, *_ = np.linalg.lstsq(A[keep], y[keep], rcond=None)
    else:
        coef_t = coef

    name = os.path.basename(path)
    print("\nsamples %d, sim span %.1f s" % (y.size, t_full[-1] - t_full[0]))
    print("net excess            %10.2f deg" % float(np.sum(excess)))
    print("summed |excess| /%gs  %10.2f deg"
          % (chunk_s, chunk_abs_excess(t_full, excess, chunk_s)))
    print("absolute true turn    %10.1f deg" % float(np.sum(np.abs(d_truth))))

    print("\nJOINT FIT   excess = a*dOmega + b*d_truth")
    print("  a  = %+.6f deg per (deg/s)   ->  tau = %+.1f ms"
          % (a, -a * 1000.0))
    print("  b  = %+.6f                   ->  scale error %+.3f %%"
          % (b, 100.0 * b))
    print("  R^2 joint %.4f   (timing alone %.4f, scale alone %.4f)"
          % (r2_ab, r2_a, r2_b))
    print("  scale-only robustness, top %.0f%% |excess| dropped: scale "
          "%+.3f %%  (its tau is meaningless -- trimming removes the very "
          "samples that carry the timing signal)"
          % (100 * trim, 100.0 * coef_t[1]))

    net_timing = float(np.sum(a * A[:, 0]))
    net_scale = float(np.sum(b * A[:, 1]))
    net_resid = float(np.sum(resid))
    print("\nDECOMPOSITION of the net excess")
    print("  timing term  %10.2f deg   (telescopes, expected near zero)"
          % net_timing)
    print("  scale term   %10.2f deg" % net_scale)
    print("  residual     %10.2f deg" % net_resid)
    print("  summed |excess| /%gs after removing the timing term: %.2f deg"
          % (chunk_s, chunk_abs_excess(t, y - a * A[:, 0], chunk_s)))
    print("  (scale here is a least-squares slope on the shifted series, "
          "not net/absolute -- those are different normalisations)")

    print("\nEXACT SHIFT SCAN -- truth series shifted and re-differenced,")
    print("no linear approximation. tau > 0 means odom LAGS truth.")
    print("%10s %16s %12s %10s" % ("tau ms", "summed |excess|", "net excess",
                                   "scale %"))
    taus = np.arange(-2.0, 2.0001, 0.02)
    scan = exact_scan(t_full, dt, d_truth, d_odom, taus, chunk_s)
    best = min(scan, key=lambda r: r[1])
    for tau, summed, net, sc in scan:
        if abs(round(tau * 1000.0) % 100) > 1:
            continue                       # print every 100 ms, scan every 20
        print("%10.0f %16.1f %12.1f %9.3f%%" % (tau * 1000.0, summed, net, sc))
    print("  minimum at tau = %.0f ms: summed |excess| %.1f deg (from %.1f at "
          "zero), net excess %.1f deg, scale %+.3f %%"
          % (best[0] * 1000.0, best[1],
             [r[1] for r in scan if abs(r[0]) < 1e-9][0], best[2],
             best[3]))
    interior_min = abs(best[0]) < taus[-1] - 1e-9
    print("  %s"
          % ("interior minimum -- a fixed delay of this size fits"
             if interior_min else
             "minimum sits at the edge of the scan -- a fixed delay does NOT "
             "fit, the model is wrong or tau is outside +/-2 s"))

    verdict = ("timing term explains most of it" if r2_a > 0.5 else
               "timing term does NOT explain it -- hypothesis dead"
               if r2_a < 0.1 else "timing term partial, do not conclude")
    print("\n  VERDICT: R^2 for timing alone = %.4f -- %s" % (r2_a, verdict))
    return {"file": name, "tau_ms": -a * 1000.0, "scale_pct": 100.0 * b,
            "r2_timing": r2_a, "r2_joint": r2_ab,
            "best_tau_ms": best[0] * 1000.0,
            "best_scale_pct": best[3],
            "interior": interior_min}


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = [fit(p) for p in sys.argv[1:]]
    if len(out) > 1:
        print("\n" + "=" * 70)
        print("COMPARISON -- the reconciliation test")
        print("=" * 70)
        print("%-30s %9s %9s %9s %9s %6s"
              % ("file", "fit tau", "fit scale", "scan tau", "scan scale",
                 "int?"))
        for o in out:
            print("%-30s %9.1f %9.3f %9.0f %9.3f %6s"
                  % (o["file"], o["tau_ms"], o["scale_pct"],
                     o["best_tau_ms"], o["best_scale_pct"],
                     "yes" if o["interior"] else "NO"))
        taus = [o["tau_ms"] for o in out]
        scales = [o["scale_pct"] for o in out]
        print("\n  tau spread   %.1f ms" % (max(taus) - min(taus)))
        print("  scale spread %.3f points" % (max(scales) - min(scales)))
        print("  The runs are reconciled only if the scale figures agree "
              "after the timing term is removed. Equal tau across runs is "
              "what a systemic offset predicts; unequal tau is not.")
    print("\nEXIT=0")


if __name__ == "__main__":
    main()
