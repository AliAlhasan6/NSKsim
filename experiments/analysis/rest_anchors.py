#!/usr/bin/env python3
"""rest_anchors.py -- measure the rotational scale error in a way a timing
offset cannot corrupt.

WHY

  Comparing yaw increments sample by sample mixes two things that cannot be
  separated from these series: a real scale error, proportional to rotation,
  and a timing offset, proportional to the change in turn rate. In b18 the
  timing term dominates -- it explains ~60 % of the per-sample error against
  0.2 % for scale -- and the fitted scale swings by tens of percent as the
  assumed offset moves, so the number B1.8 wanted is not identifiable that
  way.

  But a timing offset contributes EXACTLY ZERO while the robot is at rest.
  If yaw is constant in both streams over an interval, shifting either
  stream in time changes nothing. So: find the instants where the robot is
  stationary in ground truth, and compare the yaw CHANGE between one rest
  instant and the next. Each such segment is anchored at both ends by a
  quantity no shift can alter.

  The scale error is then the slope of odom yaw change against true yaw
  change across all segments. The script proves the immunity rather than
  asserting it: it repeats the whole measurement with the truth series
  deliberately shifted by up to +/-300 ms and prints how little the answer
  moves. If the answer DOES move, the anchors are not resting and the
  measurement is void.

WHAT WOULD FALSIFY IT

  Too few anchors, too little of the run covered, or a scale that drifts
  with the injected shift. All three are printed. Validated against a
  synthetic series carrying an injected 300 ms offset and -0.300 % slip.

Usage:
  python3 rest_anchors.py run2_robot0_samples.csv run3_robot0_samples.csv
"""

from __future__ import annotations

import os
import sys

import numpy as np

OMEGA_REST_DEG_S = 1.0      # true turn rate below this counts as still
SPEED_REST_M_S = 0.005      # true linear speed below this counts as still
MIN_REST_S = 0.40           # a rest must last this long to anchor
MIN_SEGMENT_DEG = 5.0       # ignore segments with less rotation than this


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    need = ("t_sim", "dt", "d_truth_deg", "d_odom_deg", "speed_m_s")
    missing = [c for c in need if c not in d.dtype.names]
    if missing:
        sys.exit("ABORT: %s lacks columns %s" % (path, ", ".join(missing)))
    return d


def rest_anchors(t, omega, speed, omega_thr, speed_thr, min_hold):
    """Index of the middle sample of every window that qualifies as an anchor.

    The timing term over a segment is -tau * (omega_end - omega_start), so it
    vanishes wherever the two ends share a turn rate -- not only at a full
    stop. Two anchor definitions follow from that:

      STOPPED   |omega| and speed both near zero, held for min_hold. The
                strictest reading, and the one with the fewest anchors.
      ZERO-RATE |omega| near zero alone, including the instant the robot
                reverses its turn direction while still translating. Far
                more anchors; the residual timing term is bounded by
                tau * 2 * omega_thr per segment and alternates in sign.

    Neither is assumed correct. The immunity check measures which one is."""
    still = np.abs(omega) < omega_thr
    if speed_thr is not None:
        still = still & (speed < speed_thr)
    anchors, start = [], None
    for i in range(still.size + 1):
        if i < still.size and still[i]:
            if start is None:
                start = i
        elif start is not None:
            if t[i - 1] - t[start] >= min_hold:
                anchors.append((start + i - 1) // 2)
            start = None
    return np.asarray(anchors, dtype=int)


def shifted_truth(t, dt, d_truth, tau):
    """Cumulative true yaw, resampled as if the truth stream were shifted."""
    left = t - 0.5 * dt
    edges = np.concatenate([left, [t[-1] + 0.5 * dt[-1]]])
    cum = np.concatenate([[0.0], np.cumsum(d_truth)])
    if tau == 0.0:
        return cum
    return np.interp(edges - tau, edges, cum)


def measure(t, dt, d_truth, d_odom, anchors, tau=0.0, speed=None,
            min_seg=MIN_SEGMENT_DEG):
    cum_tr = shifted_truth(t, dt, d_truth, tau)
    cum_od = np.concatenate([[0.0], np.cumsum(d_odom)])
    a = anchors
    d_tr = cum_tr[a[1:]] - cum_tr[a[:-1]]
    d_od = cum_od[a[1:]] - cum_od[a[:-1]]
    keep = np.abs(d_tr) >= min_seg
    d_tr, d_od = d_tr[keep], d_od[keep]
    i0, i1 = a[:-1][keep], a[1:][keep]
    if d_tr.size < 3:
        return None
    slope = float(np.sum(d_od * d_tr) / np.sum(d_tr ** 2))
    resid = d_od - slope * d_tr
    ss_tot = float(np.sum((d_od - d_od.mean()) ** 2))
    return {
        "n": int(d_tr.size),
        "abs_turn": float(np.sum(np.abs(d_tr))),
        "net_truth": float(np.sum(d_tr)),
        "net_odom": float(np.sum(d_od)),
        "scale_slope_pct": 100.0 * (slope - 1.0),
        "scale_aggregate_pct": 100.0 * (np.sum(d_od) - np.sum(d_tr))
                               / np.sum(np.abs(d_tr)),
        "r2": 1.0 - float(np.sum(resid ** 2)) / ss_tot if ss_tot > 0 else
              float("nan"),
        "resid_rms": float(np.sqrt(np.mean(resid ** 2))),
        "d_tr": d_tr, "d_od": d_od, "resid": resid, "slope": slope,
        "t0": t[i0], "t1": t[i1], "dur": t[i1] - t[i0],
        "mean_speed": (np.array([speed[x:y].mean() for x, y in zip(i0, i1)])
                       if speed is not None else np.zeros_like(d_tr)),
    }


def bootstrap_ci(d_tr, d_od, n_boot=4000, seed=0):
    """Confidence interval for the slope, resampling whole segments.

    Segments are the independent units here: each is anchored at both ends
    and shares no data with its neighbours. A point estimate without this
    interval cannot say whether two runs differ."""
    rng = np.random.default_rng(seed)
    n = d_tr.size
    idx = rng.integers(0, n, size=(n_boot, n))
    num = np.einsum("ij,ij->i", d_od[idx], d_tr[idx])
    den = np.einsum("ij,ij->i", d_tr[idx], d_tr[idx])
    sl = 100.0 * (num / den - 1.0)
    return float(np.percentile(sl, 2.5)), float(np.percentile(sl, 97.5))


def sensitivity(path, t, dt, d_truth, d_odom, speed, omega):
    """Does the answer survive the choices I made in defining an anchor?"""
    total_abs = float(np.sum(np.abs(d_truth)))
    print("\n--- SENSITIVITY to the anchor definition")
    print("%-10s %7s %8s %6s %7s %9s %19s %8s"
          % ("mode", "omega", "min seg", "segs", "cover%", "slope %",
             "95% CI", "immune"))
    for label, use_speed, hold in (("STOPPED", True, MIN_REST_S),
                                   ("ZERO-RATE", False, 0.0)):
        for o_thr in (0.5, 1.0, 2.0):
            a = rest_anchors(t, omega, speed, o_thr,
                             SPEED_REST_M_S if use_speed else None, hold)
            if a.size < 4:
                print("%-10s %7.1f %8s %6s %7s %9s %19s %8s"
                      % (label, o_thr, "-", 0, "-", "-", "-", "-"))
                continue
            for min_seg in (5.0, 20.0, 50.0, 100.0):
                m = measure(t, dt, d_truth, d_odom, a, speed=speed,
                            min_seg=min_seg)
                if m is None:
                    continue
                lo, hi = bootstrap_ci(m["d_tr"], m["d_od"])
                sp = max(abs(measure(t, dt, d_truth, d_odom, a, tau / 1000.0,
                                     speed, min_seg)["scale_slope_pct"]
                             - m["scale_slope_pct"])
                         for tau in (-300.0, 300.0))
                print("%-10s %7.1f %8.0f %6d %7.1f %9.3f   [%+7.3f, %+7.3f] %8s"
                      % (label, o_thr, min_seg, m["n"],
                         100.0 * m["abs_turn"] / total_abs,
                         m["scale_slope_pct"], lo, hi,
                         "yes" if sp < 0.2 else "NO"))


def report_segments(m):
    """Is the excess a property of every segment, or of a few of them?"""
    d_tr, d_od, resid = m["d_tr"], m["d_od"], m["resid"]
    per = 100.0 * (d_od / d_tr - 1.0)          # per-segment scale, %
    w = np.abs(d_tr)

    order = np.argsort(per)
    cw = np.cumsum(w[order])
    wmed = float(per[order][np.searchsorted(cw, 0.5 * cw[-1])])

    k = max(1, int(0.10 * d_tr.size))
    keep = np.argsort(np.abs(resid))[:-k]
    trimmed = float(np.sum(d_od[keep] * d_tr[keep])
                    / np.sum(d_tr[keep] ** 2) - 1.0) * 100.0
    share = 100.0 * float(np.sum(np.abs(resid)[np.argsort(np.abs(resid))[-k:]])
                          / np.sum(np.abs(resid)))

    print("\n  PER-SEGMENT DISTRIBUTION (%d segments)" % d_tr.size)
    print("    weighted median scale   %+8.3f %%   (slope %+.3f %%)"
          % (wmed, m["scale_slope_pct"]))
    print("    slope after dropping the worst %d segments  %+8.3f %%"
          % (k, trimmed))
    print("    those %d carry %.1f %% of the total |residual|" % (k, share))
    print("    per-segment scale: p10 %+.2f, p50 %+.2f, p90 %+.2f %%, "
          "range %+.2f to %+.2f"
          % (np.percentile(per, 10), np.percentile(per, 50),
             np.percentile(per, 90), per.min(), per.max()))
    big = w > 20.0
    if big.sum() > 4:
        for label, x in (("mean speed", m["mean_speed"]),
                         ("duration", m["dur"]),
                         ("|rotation|", w)):
            c = float(np.corrcoef(per[big], x[big])[0, 1])
            print("    correlation of segment scale with %-11s %+.3f"
                  % (label, c))
    print("\n    %9s %9s %11s %11s %9s %9s %9s"
          % ("t0 s", "dur s", "truth deg", "odom deg", "scale %",
             "resid deg", "speed"))
    for i in np.argsort(-np.abs(resid))[:12]:
        print("    %9.1f %9.1f %11.1f %11.1f %+9.2f %+9.1f %9.4f"
              % (m["t0"][i], m["dur"][i], d_tr[i], d_od[i], per[i],
                 resid[i], m["mean_speed"][i]))
    print("    A uniform scale error puts the weighted median on the slope "
          "and leaves the trimmed slope unchanged. A few stuck episodes "
          "pull the slope away from the median and collapse it when "
          "trimmed.")


def run(path):
    d = load(path)
    t, dt = d["t_sim"], d["dt"]
    d_truth, d_odom, speed = d["d_truth_deg"], d["d_odom_deg"], d["speed_m_s"]
    omega = d_truth / dt

    print("\n" + "=" * 70)
    print(os.path.basename(path))
    print("=" * 70)
    results = []
    for label, o_thr, v_thr, hold in (
            ("STOPPED", OMEGA_REST_DEG_S, SPEED_REST_M_S, MIN_REST_S),
            ("ZERO-RATE", OMEGA_REST_DEG_S, None, 0.0)):
        r = variant(label, path, t, dt, d_truth, d_odom, speed, omega,
                    o_thr, v_thr, hold)
        if r:
            results.append(r)
    sensitivity(path, t, dt, d_truth, d_odom, speed, omega)
    if not results:
        return None
    # prefer whichever definition the immunity check actually passed
    passed = [r for r in results if r["spread"] < 0.2]
    return (passed[0] if passed else results[0])


def variant(label, path, t, dt, d_truth, d_odom, speed, omega,
            o_thr, v_thr, hold):
    anchors = rest_anchors(t, omega, speed, o_thr, v_thr, hold)
    print("\n--- %s anchors: |omega| < %g deg/s%s%s" %
          (label, o_thr,
           ", speed < %g m/s" % v_thr if v_thr is not None else "",
           ", held %g s" % hold if hold else ""))
    print("anchors found: %d over %.1f s" % (anchors.size, t[-1] - t[0]))
    if anchors.size < 4:
        print("  too few anchors, nothing to measure with this definition")
        return None

    base = measure(t, dt, d_truth, d_odom, anchors, speed=speed)
    if base is None:
        print("  fewer than 3 usable segments with this definition")
        return None

    total_abs = float(np.sum(np.abs(d_truth)))
    print("segments used: %d, covering %.1f deg of %.1f deg absolute "
          "rotation (%.1f %%)"
          % (base["n"], base["abs_turn"], total_abs,
             100.0 * base["abs_turn"] / total_abs))
    print("\n  net true turn over segments  %10.1f deg" % base["net_truth"])
    print("  net odom turn over segments  %10.1f deg" % base["net_odom"])
    print("  SCALE, least-squares slope   %+10.3f %%" % base["scale_slope_pct"])
    print("  SCALE, aggregate             %+10.3f %%"
          % base["scale_aggregate_pct"])
    lo, hi = bootstrap_ci(base["d_tr"], base["d_od"])
    print("  95 %% CI on the slope, resampling segments  [%+.3f, %+.3f] %%"
          % (lo, hi))
    print("  R^2 %.5f, residual RMS %.3f deg per segment"
          % (base["r2"], base["resid_rms"]))
    print("  anchors span sim %.1f to %.1f s, %.1f %% of the run's duration"
          % (base["t0"][0], base["t1"][-1],
             100.0 * (base["t1"][-1] - base["t0"][0]) / (t[-1] - t[0])))
    report_segments(base)

    print("\n  IMMUNITY CHECK -- same measurement with the truth stream "
          "deliberately shifted")
    print("  %10s %12s %12s" % ("shift ms", "scale %", "change"))
    for tau_ms in (-300, -150, -50, 0, 50, 150, 300):
        m = measure(t, dt, d_truth, d_odom, anchors, tau_ms / 1000.0)
        print("  %10d %12.3f %12.3f"
              % (tau_ms, m["scale_slope_pct"],
                 m["scale_slope_pct"] - base["scale_slope_pct"]))
    spread = max(abs(measure(t, dt, d_truth, d_odom, anchors, s / 1000.0)
                     ["scale_slope_pct"] - base["scale_slope_pct"])
                 for s in (-300, 300))
    print("  worst change across +/-300 ms: %.3f points -- %s"
          % (spread, "immune, the measurement stands" if spread < 0.2 else
             "NOT immune, the anchors are not at rest and this is void"))
    return {"file": "%s [%s]" % (os.path.basename(path), label),
            "ci": (lo, hi),
            "scale": base["scale_slope_pct"],
            "agg": base["scale_aggregate_pct"], "n": base["n"],
            "cov": 100.0 * base["abs_turn"] / total_abs, "spread": spread}


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = [r for r in (run(p) for p in sys.argv[1:]) if r]
    if len(out) > 1:
        print("\n" + "=" * 70)
        print("COMPARISON")
        print("=" * 70)
        print("%-32s %9s %19s %6s %7s"
              % ("file", "scale %", "95% CI", "segs", "cover %"))
        for o in out:
            print("%-32s %9.3f   [%+7.3f, %+7.3f] %6d %7.1f"
                  % (o["file"], o["scale"], o["ci"][0], o["ci"][1],
                     o["n"], o["cov"]))
        sc = [o["scale"] for o in out]
        print("\n  spread %.3f points" % (max(sc) - min(sc)))
        if len(out) == 2:
            a, b = out
            gap = (a["ci"][0] > b["ci"][1]) or (b["ci"][0] > a["ci"][1])
            print("  intervals %s -- the runs %s distinguishable on this "
                  "evidence"
                  % ("do not overlap" if gap else "OVERLAP",
                     "are" if gap else "are NOT"))
    print("\nEXIT=0")


if __name__ == "__main__":
    main()
