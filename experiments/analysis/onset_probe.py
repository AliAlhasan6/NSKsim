#!/usr/bin/env python3
"""onset_probe.py -- measure the offset between the ground-truth and odometry
streams without fitting anything.

WHY NOT ANOTHER FIT

  The regression and the shift scan both estimate the offset from the bulk
  of the data, through a model. They agreed on roughly -300 ms, but a model
  that is wrong in some other way can produce a confident number. This does
  not use a model. It finds the moments when the robot starts turning after
  standing still, and asks, in each stream separately, when the turn began.
  The difference of those two instants is the offset, read directly off the
  clock.

  Sign: delta = t_truth_onset - t_odom_onset. POSITIVE means ground truth
  reports the motion LATER than odometry does, i.e. the truth stream is the
  delayed one.

DELAY OR SMOOTHING

  The threshold sweep separates the two cases, which no previous test here
  could:

    a pure transport delay   -- every stream is the same signal shifted, so
                                delta is the SAME at every onset threshold.
    a filter or smoothing    -- the slower stream also rises more gradually,
                                so delta GROWS with the threshold.

  This matters for what gets fixed. A delay is a stamping or transport
  problem. A filter is a publisher doing interpolation or averaging.

WHAT WOULD FALSIFY IT

  Too few transitions, or a delta whose scatter swamps its median. Both are
  printed. Validated against a synthetic series with a known pure 300 ms
  offset, where it returns -300 ms at every threshold.

Usage:
  python3 onset_probe.py run2_robot0_samples.csv run3_robot0_samples.csv
"""

from __future__ import annotations

import os
import sys

import numpy as np

REST_OMEGA = 1.0        # deg/s, below this the stream counts as still
MIN_REST_S = 0.50       # a rest must last this long to start a transition
SEARCH_S = 5.0          # look this far past the rest for an onset
HOLD_SAMPLES = 3        # crossing must persist, so one spike is not an onset


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    for c in ("t_sim", "dt", "d_truth_deg", "d_odom_deg"):
        if c not in d.dtype.names:
            sys.exit("ABORT: %s lacks column %s" % (path, c))
    return d


def rest_windows(t, om_a, om_b):
    """Windows where BOTH streams are still, long enough to anchor an onset."""
    still = (np.abs(om_a) < REST_OMEGA) & (np.abs(om_b) < REST_OMEGA)
    out, start = [], None
    for i in range(still.size + 1):
        if i < still.size and still[i]:
            if start is None:
                start = i
        elif start is not None:
            if t[i - 1] - t[start] >= MIN_REST_S:
                out.append(i - 1)          # last still sample
            start = None
    return out


def first_crossing(t, om, i0, thresh):
    """Index of the first sustained crossing of thresh at or after i0."""
    n = om.size
    end = np.searchsorted(t, t[i0] + SEARCH_S)
    hit = np.abs(om[i0:end]) > thresh
    if hit.size < HOLD_SAMPLES:
        return None
    run = np.convolve(hit.astype(int), np.ones(HOLD_SAMPLES, int), "valid")
    idx = np.flatnonzero(run == HOLD_SAMPLES)
    return i0 + int(idx[0]) if idx.size else None


def onsets(t, om_tr, om_od, starts, thresh):
    deltas = []
    for i0 in starts:
        a = first_crossing(t, om_tr, i0, thresh)
        b = first_crossing(t, om_od, i0, thresh)
        if a is None or b is None:
            continue
        deltas.append(t[a] - t[b])
    return np.asarray(deltas)


def rise_times(t, om_tr, om_od, starts, lo=2.0, hi=40.0):
    """How long each stream takes to climb from lo to hi after a rest.

    This separates two very different stories. If odometry STEPS -- rise
    near zero -- the plugin applies the commanded wheel velocity at once and
    the wheels are slipping against a body that cannot follow. If both
    streams ramp at similar rates and are merely offset, the command itself
    is being ramped and the difference is transport, not slip."""
    out = {"truth": [], "odom": []}
    for i0 in starts:
        pair = {}
        for name, om in (("truth", om_tr), ("odom", om_od)):
            a = first_crossing(t, om, i0, lo)
            b = first_crossing(t, om, i0, hi)
            if a is None or b is None or b < a:
                break
            pair[name] = t[b] - t[a]
        if len(pair) == 2:              # keep only transitions both streams made
            out["truth"].append(pair["truth"])
            out["odom"].append(pair["odom"])
    return {k: np.asarray(v) * 1000.0 for k, v in out.items()}


def run(path):
    d = load(path)
    t, dt = d["t_sim"], d["dt"]
    om_tr = d["d_truth_deg"] / dt
    om_od = d["d_odom_deg"] / dt

    starts = rest_windows(t, om_tr, om_od)
    print("\n" + "=" * 70)
    print(os.path.basename(path))
    print("=" * 70)
    print("rest windows where both streams are still for %.1f s: %d"
          % (MIN_REST_S, len(starts)))
    if len(starts) < 5:
        print("  too few transitions to measure -- nothing claimed")
        return None

    print("\n%9s %7s %10s %10s %10s %10s"
          % ("thresh", "n", "median ms", "mean ms", "IQR ms", "p10-p90"))
    table = []
    for thresh in (2.0, 5.0, 10.0, 20.0, 40.0):
        dl = onsets(t, om_tr, om_od, starts, thresh) * 1000.0
        if dl.size < 5:
            print("%9.0f %7d %10s %10s %10s %10s"
                  % (thresh, dl.size, "-", "-", "-", "-"))
            continue
        q1, q3 = np.percentile(dl, [25, 75])
        print("%9.0f %7d %10.0f %10.0f %10.0f %5.0f to %-5.0f"
              % (thresh, dl.size, np.median(dl), dl.mean(), q3 - q1,
                 np.percentile(dl, 10), np.percentile(dl, 90)))
        table.append((thresh, float(np.median(dl)), int(dl.size),
                      float(q3 - q1)))

    if len(table) < 3:
        print("\n  too few usable thresholds to distinguish delay from filter")
        return None

    meds = [r[1] for r in table]
    drift = max(meds) - min(meds)
    print("\n  median across thresholds: %s ms"
          % ", ".join("%.0f" % m for m in meds))
    print("  drift with threshold: %.0f ms -- %s"
          % (drift,
             "flat, consistent with a pure transport delay" if drift < 40 else
             "grows, consistent with smoothing in the slower stream rather "
             "than a plain delay"))
    med = float(np.median(meds))
    if drift < 40:
        print("  OFFSET: %.0f ms, %s stream is the later one"
              % (med, "ground-truth" if med > 0 else "odometry"))
        print("  scatter check: IQR is %.0f ms against a median of %.0f -- %s"
              % (table[len(table) // 2][3], med,
                 "the median is meaningful"
                 if abs(med) > table[len(table) // 2][3] else
                 "scatter swamps the median, treat this as unresolved"))
    else:
        print("  no single offset is printed: with this much drift the "
              "streams differ in SHAPE, not by a shift, and any one number "
              "would just be the threshold I happened to pick")

    rt = rise_times(t, om_tr, om_od, starts)
    print("\n  RISE TIME from 2 to 40 deg/s after a rest")
    print("  %-8s %6s %10s %10s %12s" % ("stream", "n", "median ms",
                                         "mean ms", "IQR ms"))
    for name in ("truth", "odom"):
        v = rt[name]
        if v.size < 5:
            print("  %-8s %6d %10s %10s %12s" % (name, v.size, "-", "-", "-"))
            continue
        q1, q3 = np.percentile(v, [25, 75])
        print("  %-8s %6d %10.0f %10.0f %12.0f"
              % (name, v.size, np.median(v), v.mean(), q3 - q1))
    if rt["truth"].size >= 5:
        # Transitions differ enormously from each other, so compare the two
        # streams WITHIN each transition. The paired difference cancels that
        # variation; the medians above do not.
        diff = rt["truth"] - rt["odom"]
        q1, q3 = np.percentile(diff, [25, 75])
        wins = float(np.mean(diff > 0))
        print("\n  PAIRED, within each transition: truth rise minus odom rise")
        print("    n %d, median %+.0f ms, IQR %.0f ms, p10 %+.0f, p90 %+.0f"
              % (diff.size, np.median(diff), q3 - q1,
                 np.percentile(diff, 10), np.percentile(diff, 90)))
        print("    truth is the slower stream in %.0f %% of transitions"
              % (100.0 * wins))
        # sign test: how surprising is that count if the two were equivalent?
        k, n = int(np.sum(diff > 0)), diff.size
        print("    sign test: %d of %d. Under no systematic difference this "
              "would be %d +/- %.1f" % (k, n, n // 2, 0.5 * np.sqrt(n)))
        if wins > 0.8 and np.median(diff) > 100:
            print("    The wheels reach speed first in almost every "
                  "transition. Equal shaping would put this near 50 %% and "
                  "the median near zero, as it does on the synthetic.")
    return {"file": os.path.basename(path), "offset_ms": med, "drift": drift,
            "n": table[0][2]}


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = [r for r in (run(p) for p in sys.argv[1:]) if r]
    if len(out) > 1:
        print("\n" + "=" * 70)
        print("COMPARISON")
        print("=" * 70)
        print("%-34s %12s %12s" % ("file", "offset ms", "drift ms"))
        for o in out:
            print("%-34s %12.0f %12.0f"
                  % (o["file"], o["offset_ms"], o["drift"]))
        sp = max(o["offset_ms"] for o in out) - min(o["offset_ms"]
                                                    for o in out)
        print("\n  spread between runs: %.0f ms -- %s" % (sp,
              "one systemic offset" if abs(sp) < 60 else
              "not one number, the offset differs between runs"))
    print("\nEXIT=0")


if __name__ == "__main__":
    main()
