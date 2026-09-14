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

DECAY MODE

  --decay measures the mirror image: motion -> rest rather than rest ->
  motion. Same threshold ladder, same pairing within transitions, same
  persistence rule. It is a separate path; nothing above is touched, and the
  rise result reproduces unchanged.

  It exists because the rise says nothing about the TAIL. B1.8 measured
  onsets and only onsets, so how long truth takes to FINISH catching up was
  never measured -- and that is the number that sizes the rest interval in
  section 7 of HANDOFF 2026-09-13, and the number a steady-motion guard band
  has to clear.

  THE SETTLE is the headline: from odometry's last crossing below 2 deg/s
  until ground truth stays below it. For a first-order lag decaying from
  rate w it is tau*ln(w/2), so it GROWS with the rate the episode span down
  from -- which is why the settle is also reported against peak rate.

  That identity assumes odometry stops like a STEP, so that truth's own fall
  is the whole of the gap. It holds on the synthetic, where odom falls in 60
  ms, and D1 checks it there. It does NOT hold wherever odometry ramps down
  too: then both streams are still falling together, the settle is much
  shorter than tau*ln(w/2), and it is a measurement of the gap and not of
  any tau. The settle is always the two crossing times subtracted, which is
  the number section 7 needs either way; only the model behind it lapses.

  Sign is unchanged: positive means ground truth is the later stream.

Usage:
  python3 onset_probe.py run2_robot0_samples.csv run3_robot0_samples.csv
  python3 onset_probe.py --decay --synthetic
  python3 onset_probe.py --decay experiments/logs/b18/analysis/run2_robot0_samples.csv
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

REST_OMEGA = 1.0        # deg/s, below this the stream counts as still
MIN_REST_S = 0.50       # a rest must last this long to start a transition
SEARCH_S = 5.0          # look this far past the rest for an onset
HOLD_SAMPLES = 3        # crossing must persist, so one spike is not an onset

# --- decay mode only; the constants above keep the rise path untouched -----
DECAY_HI = 40.0         # deg/s, a transition must have been turning this fast
SETTLE_OMEGA = 2.0      # deg/s, "at rest" for the settle, the ladder's floor
LADDER = (2.0, 5.0, 10.0, 20.0, 40.0)

# Verdicts for the synthetic. A first-order lag reaches threshold th from
# rest at -tau*ln(1 - th/w) and falls to it at +tau*ln(w/th); for w = 57 and
# the 2->40 ladder that is 0.51 s up against 1.41 s down. If the probe does
# not see the decay come out the longer of the two on data built that way,
# the probe is wrong and the bags prove nothing.
D0_MIN_RATIO = 2.0      # truth's fall must be this many times its rise
D1_TOL = 0.30           # settle within +/- 30 % of tau*ln(w/2)


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


# --------------------------------------------------------------- decay mode

def motion_rest_windows(t, om_a, om_b):
    """Indices where both streams are turning fast and then come to rest.

    The mirror of rest_windows(). That one anchors on the last still sample
    before motion; this one anchors on the last FAST sample before a rest.
    Both streams must qualify, exactly as they must there, so a transition
    one stream missed is not counted in the other.

    A transition needs |omega| above DECAY_HI and then below SETTLE_OMEGA,
    held for MIN_REST_S. "Staying there" is the whole point: a stream that
    dips below 2 deg/s and comes back has not settled, it has passed through
    zero on its way to turning the other way.

    Returns (i0, i_end) pairs: the last fast sample, and the last sample of
    the rest that followed. The decay is measured strictly between them.
    That bound is not cosmetic. The rise path can afford a fixed SEARCH_S
    because it wants the FIRST crossing and the window only has to be long
    enough; the decay wants the LAST one, so a window that runs on into the
    next episode finds that episode's spin-up and reports no decay at all.
    """
    fast = (np.abs(om_a) > DECAY_HI) & (np.abs(om_b) > DECAY_HI)
    still = (np.abs(om_a) < SETTLE_OMEGA) & (np.abs(om_b) < SETTLE_OMEGA)
    out, start = [], None
    for i in range(still.size + 1):
        if i < still.size and still[i]:
            if start is None:
                start = i
        elif start is not None:
            if t[i - 1] - t[start] >= MIN_REST_S:
                lo = np.searchsorted(t, t[start] - SEARCH_S)
                seg = np.flatnonzero(fast[lo:start])
                if seg.size:
                    out.append((lo + int(seg[-1]), i - 1))
            start = None
    return out


def last_crossing(t, om, i0, i_end, thresh):
    """Index of the first sample in [i0, i_end] below thresh that STAYS
    below it through i_end.

    The mirror of first_crossing(): that one wants the first sustained
    crossing upward, this one the last crossing downward. The same
    HOLD_SAMPLES persistence applies, here as a tail long enough that a
    spike at the close of the window is not read as "never settled" --
    without it the answer would depend on where the window happened to end.
    """
    seg = np.abs(om[i0:i_end + 1])
    if seg.size < HOLD_SAMPLES + 1:
        return None
    above = np.flatnonzero(seg > thresh)
    if above.size == 0:
        return i0                      # already below and stayed below
    last = int(above[-1])
    if (seg.size - 1 - last) < HOLD_SAMPLES:
        return None                    # still above when the rest ended
    return i0 + last + 1


def decays(t, om_tr, om_od, starts, thresh):
    """Paired difference of the two streams' downward crossings of thresh.

    Mirror of onsets(). Sign is the same convention: positive means truth
    crossed LATER, i.e. truth is the stream still moving.
    """
    deltas = []
    for i0, i_end in starts:
        a = last_crossing(t, om_tr, i0, i_end, thresh)
        b = last_crossing(t, om_od, i0, i_end, thresh)
        if a is None or b is None:
            continue
        deltas.append(t[a] - t[b])
    return np.asarray(deltas)


def fall_times(t, om_tr, om_od, starts, hi=DECAY_HI, lo=SETTLE_OMEGA):
    """How long each stream takes to fall from hi to lo. Mirror of
    rise_times(), and compared against it directly on the synthetic."""
    out = {"truth": [], "odom": []}
    for i0, i_end in starts:
        pair = {}
        for name, om in (("truth", om_tr), ("odom", om_od)):
            b = last_crossing(t, om, i0, i_end, hi)
            a = last_crossing(t, om, i0, i_end, lo)
            if a is None or b is None or a < b:
                break
            pair[name] = t[a] - t[b]
        if len(pair) == 2:
            out["truth"].append(pair["truth"])
            out["odom"].append(pair["odom"])
    return {k: np.asarray(v) * 1000.0 for k, v in out.items()}


def episode_peak(t, om_tr, om_od, still, i0):
    """The rate the episode ending at i0 was actually turning at.

    Read BACKWARD from i0, over the contiguous stretch of motion that led
    into the rest. Reading forward would be meaningless: i0 is by
    construction the LAST sample above DECAY_HI, so everything after it is
    the tail, and every transition would come back at just over 40 deg/s no
    matter what plateau it spun down from -- collapsing the rate buckets
    below into one. The walk stops at the previous rest, and in any case
    within SEARCH_S, so a long motion phase cannot reach back into an
    earlier episode's plateau.
    """
    lo = int(np.searchsorted(t, t[i0] - SEARCH_S))
    j = i0
    while j > lo and not still[j - 1]:
        j -= 1
    return float(max(np.max(np.abs(om_tr[j:i0 + 1])),
                     np.max(np.abs(om_od[j:i0 + 1]))))


def settles(t, om_tr, om_od, starts):
    """The headline number, per transition, with the rate it decayed from.

    From odometry's last crossing below SETTLE_OMEGA until truth stays below
    it. Returned with each transition's peak rate, because for a lag the
    settle is tau*ln(w/2) and so grows with the rate -- sizing a rest
    interval off the median would under-size it for the fast episodes.

    Also returns the span from the last fast sample until truth is done,
    which is the same tail measured from the start of the ramp-down rather
    than from odometry's finish.
    """
    still = (np.abs(om_tr) < SETTLE_OMEGA) & (np.abs(om_od) < SETTLE_OMEGA)
    out = []
    for i0, i_end in starts:
        a = last_crossing(t, om_tr, i0, i_end, SETTLE_OMEGA)
        b = last_crossing(t, om_od, i0, i_end, SETTLE_OMEGA)
        if a is None or b is None:
            continue
        out.append((t[a] - t[b], t[a] - t[i0],
                    episode_peak(t, om_tr, om_od, still, i0)))
    if not out:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    arr = np.asarray(out)
    return arr[:, 0] * 1000.0, arr[:, 1] * 1000.0, arr[:, 2]


def describe(label, v, unit="ms"):
    if v.size < 5:
        print("  %-26s %6d %10s %10s %10s %10s"
              % (label, v.size, "-", "-", "-", "-"))
        return None
    q1, q3 = np.percentile(v, [25, 75])
    print("  %-26s %6d %10.0f %10.0f %10.0f %5.0f to %-5.0f"
          % (label, v.size, np.median(v), v.mean(), q3 - q1,
             np.percentile(v, 10), np.percentile(v, 90)))
    return float(np.median(v))


def run_decay(d, label, tau=None):
    """The motion -> rest path. Nothing here touches the rise path."""
    t, dt = d["t_sim"], d["dt"]
    om_tr = d["d_truth_deg"] / dt
    om_od = d["d_odom_deg"] / dt

    print("\n" + "=" * 70)
    print("%s  --  DECAY (motion -> rest)" % label)
    print("=" * 70)
    starts = motion_rest_windows(t, om_tr, om_od)
    print("transitions with both streams above %g deg/s then below %g for "
          "%.1f s: %d" % (DECAY_HI, SETTLE_OMEGA, MIN_REST_S, len(starts)))
    if len(starts) < 5:
        print("  too few transitions to measure -- nothing claimed")
        return None

    print("\n  FALL of each stream across the ladder, truth minus odom, "
          "paired within transitions")
    print("  %-26s %6s %10s %10s %10s %10s"
          % ("threshold", "n", "median ms", "mean ms", "IQR ms", "p10-p90"))
    ladder = []
    for thresh in LADDER:
        dl = decays(t, om_tr, om_od, starts, thresh) * 1000.0
        med = describe("down through %g deg/s" % thresh, dl)
        if med is not None:
            ladder.append((thresh, med))
    if len(ladder) >= 3:
        meds = [m for _, m in ladder]
        print("\n  On the RISE the gap grew with the threshold. An "
              "exponential tail is")
        print("  furthest behind at the BOTTOM, so on the decay the gap "
              "should grow as the")
        print("  threshold FALLS -- the same statement about shape, read "
              "from the other end.")
        print("    %s ms at %s deg/s"
              % (", ".join("%.0f" % m for m in meds),
                 ", ".join("%g" % th for th, _ in ladder)))
        # Checked, not asserted. A ladder that is not monotone is not a
        # single lag unwinding, and saying "and it does" over the top of one
        # that is not would be the probe deciding the answer in advance.
        if all(a >= b for a, b in zip(meds, meds[1:])):
            print("    and it does: monotone down the ladder, one tail.")
        else:
            print("    and it does NOT: the gap is not monotone, so this "
                  "tail is not one")
            print("    lag unwinding. The SETTLE below is still a "
                  "measurement -- it is two")
            print("    crossing times subtracted -- but do not read this "
                  "ladder as a shape.")

    ft = fall_times(t, om_tr, om_od, starts)
    print("\n  FALL TIME from %g to %g deg/s, per stream"
          % (DECAY_HI, SETTLE_OMEGA))
    print("  %-26s %6s %10s %10s %10s %10s"
          % ("stream", "n", "median ms", "mean ms", "IQR ms", "p10-p90"))
    fall_tr = describe("truth", ft["truth"])
    describe("odom", ft["odom"])

    # Anchor the rise on the rests this mode just found, not on
    # rest_windows(). That function requires both streams under REST_OMEGA
    # (1 deg/s) for MIN_REST_S, and a lagged truth needs tau*ln(w/1) to get
    # there -- 1.7 s from 57 deg/s -- so on a series whose rests are shorter
    # than that it anchors nothing and the comparison has no rise to make.
    # The rest END of each decay transition is the same instant rest_windows
    # would return, found at the floor the rest of this mode uses. The rise
    # is then measured by the rise path's own unmodified functions.
    starts_rise = [i_end for _, i_end in starts]
    rt = rise_times(t, om_tr, om_od, starts_rise)
    print("\n  RISE TIME from %g to %g deg/s, the same transitions, for scale"
          % (SETTLE_OMEGA, DECAY_HI))
    print("  %-26s %6s %10s %10s %10s %10s"
          % ("stream", "n", "median ms", "mean ms", "IQR ms", "p10-p90"))
    rise_tr = describe("truth", rt["truth"])
    describe("odom", rt["odom"])

    ratio = (fall_tr / rise_tr if fall_tr and rise_tr and rise_tr > 0
             else float("nan"))
    if not math.isnan(ratio):
        print("\n  truth falls %.1fx slower than it rises (%.0f ms against "
              "%.0f ms)" % (ratio, fall_tr, rise_tr))

    st, span, peak = settles(t, om_tr, om_od, starts)
    print("\n  SETTLE -- odom's last crossing below %g deg/s until truth "
          "stays below it" % SETTLE_OMEGA)
    print("  %-26s %6s %10s %10s %10s %10s"
          % ("", "n", "median ms", "mean ms", "IQR ms", "p10-p90"))
    st_med = describe("settle", st)
    describe("truth still moving for", span)
    if st.size >= 5:
        print("\n  by the rate the transition decayed from:")
        print("  %-26s %6s %10s %10s" % ("peak rate deg/s", "n", "median ms",
                                         "max ms"))
        for lo, hi in ((DECAY_HI, 60.0), (60.0, 90.0), (90.0, 1e9)):
            sel = (peak >= lo) & (peak < hi)
            if not np.any(sel):
                continue
            hi_s = "inf" if hi > 1e8 else "%g" % hi
            print("  %-26s %6d %10.0f %10.0f"
                  % ("%g-%s" % (lo, hi_s), int(sel.sum()),
                     np.median(st[sel]), np.max(st[sel])))
        print("\n  THIS IS THE NUMBER THAT SIZES SECTION 7's REST INTERVAL.")
        print("  p90 %.0f ms, max %.0f ms. A rest shorter than the max "
              "leaves the next" % (np.percentile(st, 90), np.max(st)))
        print("  episode starting while truth is still unwinding the last "
              "one, which is")
        print("  exactly the contamination the designed run exists to "
              "avoid.")

    verdicts = {}
    if tau is not None and st.size >= 5:
        pred = tau * np.log(np.maximum(peak, SETTLE_OMEGA * 1.01)
                            / SETTLE_OMEGA) * 1000.0
        got, want = float(np.median(st)), float(np.median(pred))
        verdicts["d1"] = (abs(got - want) <= D1_TOL * want, got, want)
    verdicts["ratio"] = ratio
    verdicts["settle_med"] = st_med
    return verdicts


def synthetic_samples(rate_lo, rate_hi, seed=3):
    """The transient steady_gate.py already validated, in this format.

    Reused rather than rebuilt: steady_gate's S0 checks that same generator
    against every onset threshold b18_run2 was measured at, so the decay
    probe is being pointed at a transient whose RISE is known to match the
    data. That is the only thing that makes its verdict on the FALL mean
    anything.
    """
    import rate_profile
    import steady_gate

    od, tr, _ = steady_gate.synth(False, rate_lo, rate_hi, seed)
    t_mid, dt, d_truth, d_odom = rate_profile.increments(
        tr["t"], np.radians(tr["yaw"]), od["t"], np.radians(od["yaw"]))
    return ({"t_sim": t_mid, "dt": dt, "d_truth_deg": d_truth,
             "d_odom_deg": d_odom}, steady_gate.SYN_TAU)


def run_synthetic():
    """Validate the decay probe before it is pointed at a bag."""
    print("=" * 70)
    print("SYNTHETIC -- decay probe against a known first-order lag")
    print("=" * 70)
    print("Injected tau is the only transient. For a lag decaying from w,")
    print("rise to 40 deg/s is -tau*ln(1-40/w) and fall to 2 is tau*ln(w/2).")
    print("At w = 57 that is 0.51 s up against 1.41 s down. If this probe "
          "does not")
    print("see that, the probe is wrong -- not the data it is later pointed "
          "at.")

    d, tau = synthetic_samples(50.0, 65.0)
    print("\ninjected tau %.0f ms, episodes pinned to the 50-65 deg/s Nav2 "
          "spin band" % (tau * 1000))
    v = run_decay(d, "synthetic, 50-65 deg/s", tau=tau)

    print("\n" + "=" * 70)
    print("SYNTHETIC VERDICTS")
    print("=" * 70)
    if v is None:
        print("  D0 DECAY-LONGER  FAIL  no transitions detected at all -- "
              "the probe cannot see a decay it built itself")
        return False
    ok0 = (not math.isnan(v["ratio"])) and v["ratio"] >= D0_MIN_RATIO
    print("  %-17s %-4s  expected truth falls >= %gx slower than it rises   "
          "got %.1fx"
          % ("D0 DECAY-LONGER", "PASS" if ok0 else "FAIL", D0_MIN_RATIO,
             v["ratio"]))
    ok1 = True
    if "d1" in v:
        ok1, got, want = v["d1"]
        print("  %-17s %-4s  expected tau*ln(w/2) = %.0f ms +/- %.0f %%      "
              "got %.0f ms"
              % ("D1 SETTLE-MODEL", "PASS" if ok1 else "FAIL", want,
                 100 * D1_TOL, got))
    ok = ok0 and ok1
    if not ok:
        print("\n  The probe does not reproduce a decay it was handed by "
              "construction.")
        print("  Do NOT read the run2 table below as a measurement of the "
              "robot.")
    else:
        print("\n  The probe recovers the injected decay. It may be pointed "
              "at a bag.")
    return ok


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
                  "transition. Equal shaping would put this near 50 % and "
                  "the median near zero, as it does on the synthetic.")
    return {"file": os.path.basename(path), "offset_ms": med, "drift": drift,
            "n": table[0][2]}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="per-sample CSVs from "
                                            "rate_profile.py")
    ap.add_argument("--decay", action="store_true",
                    help="motion -> rest instead of rest -> motion")
    ap.add_argument("--synthetic", action="store_true",
                    help="validate against an injected first-order lag; "
                         "reads no bag and no CSV")
    args = ap.parse_args()

    if args.synthetic:
        if not args.decay:
            ap.error("--synthetic is implemented for --decay only; the rise "
                     "path's synthetic control is described in the header "
                     "and lives in rest_anchors.py")
        return 0 if run_synthetic() else 1

    if not args.paths:
        ap.error("give at least one per-sample CSV, or --synthetic")

    if args.decay:
        seen = 0
        for p in args.paths:
            if run_decay(load(p), os.path.basename(p)) is not None:
                seen += 1
        print("\nEXIT=%d" % (0 if seen else 1))
        return 0 if seen else 1

    out = [r for r in (run(p) for p in args.paths) if r]
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
