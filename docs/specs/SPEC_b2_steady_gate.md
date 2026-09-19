# SPEC — steady-rotation gate for B2 ground-truth comparison

Target: new script `experiments/analysis/steady_gate.py`.
Status of the question it serves: B1.8 is closed; this does **not** reopen it
and must not produce an odometry scale figure for NSKsim.

## 1. Purpose

B1.8 established that the odometry/truth gap is transient slip during
rotational acceleration: the wheels reach commanded speed ~420 ms before the
body, odometry over-reports through that window, and the sign reverses on
deceleration. Any B2 divergence statistic computed against ground truth over
raw samples inherits that error.

This script builds a **mask of steady-motion samples** and restricts every
ground-truth comparison to them. It is an exclusion, not a correction. No
transient model is fitted and no per-run scale factor is reported.

A second, equally important output is the **retention** figure. If the gate
keeps too little of either bag, that is the evidence-backed case for the
designed single-robot transient run proposed in §7 of the 2026-09-13 handoff.

## 2. Inputs

- The per-sample CSVs already exported by `rate_profile.py` for `b18_run2`
  and `b18_run3`, in `experiments/logs/b18/analysis/`.
- If those CSVs carry yaw but not planar position, extend `rate_profile.py`'s
  export to include truth and odom `x`, `y`. Do not recompute the series in a
  second place — one exporter, one definition.

Time base: odom is 50 Hz, truth 16.6–20 Hz. Resample by linear interpolation
of **unwrapped** yaw onto the truth stamps. Never interpolate wrapped angles.
Note that interpolation error is itself concentrated in transients, which is a
further reason the guard band in §4 is generous rather than tight.

## 3. Gating signal — odometry only

The mask is computed from the **odometry** stream alone. Truth must not enter
the mask in any form.

Reason: truth is the reference the gated statistic is measured against. A mask
built partly from truth selects on the quantity being measured and the result
is unfalsifiable. Odometry is also the stream that would be available online,
so the same gate is implementable outside analysis.

## 4. Algorithm

1. `w_odom` = central difference of unwrapped odom yaw, then a **median**
   filter over 5 samples (100 ms at 50 Hz). Median, not mean — a mean smears
   step edges and would hide the thing being excluded.
2. `alpha` = central difference of `w_odom`, same median filter.
   `v`, `a_lin` = the same treatment applied to planar position.
3. `raw_ok = (abs(alpha) <= ALPHA_MAX) & (abs(a_lin) <= A_LIN_MAX)`.
4. **Erode** `raw_ok` by `T_GUARD` on both sides: a sample is accepted only if
   every sample in `[t - T_GUARD, t + T_GUARD]` is in `raw_ok`. Equivalently,
   dilate the rejected set. `T_GUARD = 0.60 s` — the measured median rise was
   420 ms and p90 was 532 ms, so 600 ms clears the paired distribution.
5. Drop accepted runs shorter than `T_MIN = 1.2 s` (2 × `T_GUARD`). Short
   islands are dominated by their own edges.
6. Accumulate increments **only** between consecutive sample pairs whose two
   endpoints lie in the same accepted window. Never bridge a gap. A gap
   spanned is exactly the transient the gate exists to remove.

Per accepted window `i`: `d_odom_i`, `d_truth_i`, `excess_i = d_odom_i -
d_truth_i`. Form `scale_i = d_odom_i / d_truth_i` only where
`abs(d_truth_i) > 5 deg`.

## 5. Outputs

One table per run, written to `experiments/logs/b18/analysis/`:

- **Retention**: accepted duration / total; accepted absolute rotation /
  total absolute rotation; accepted path length / total. Rotation retention is
  the headline number.
- **Primary diagnostic**: summed `abs(excess)` per 1000 deg turned, gated
  against ungated. Per the standing method rule this is the statistic, not the
  net — the net is a cancellation residue.
- Gated point estimate: pooled slope (sum `d_odom` over sum `d_truth` across
  accepted windows), weighted median across windows, and a bootstrap 95 %
  interval over whole windows.
- Breakdown by the existing rate bins, so it is visible whether the exclusion
  falls where B1.8 said the divergence lives (5–45 deg/s, driving and turning
  at once) rather than in the 55–200 deg/s spin-recovery rows that already
  agreed across runs.
- Sweep table over `ALPHA_MAX` and `A_LIN_MAX`.

## 6. Verdicts

Each prints PASS or FAIL against a stated expected value, and each must be
capable of failing on the real data.

- **V1 RETENTION** — accepted absolute rotation is at least 20 % of total in
  *each* run. FAIL means the gate starves the measurement; that is the
  argument for the designed run, and it is a legitimate outcome, not a bug to
  tune away.
- **V2 REDUCTION** — gated `abs(excess)` per 1000 deg is below 50 % of the
  ungated value in each run. FAIL means the mask is not removing the
  transient and the whole approach is wrong.
- **V3 AGREEMENT** — gated `run2` and `run3` point estimates each fall inside
  the other's bootstrap interval, and both intervals are narrower than the
  ungated `[-0.28, +4.33]` and `[-0.82, +5.21]`. FAIL means something other
  than the transient separates the two bags.
- **V4 FLATNESS** — the gated estimate moves less than 0.5 points across the
  middle half of the `ALPHA_MAX` sweep. FAIL means the answer is a function of
  the threshold rather than of the data.

V3 and V4 are informational for B2 — they test whether the gate is sound. Only
V1 and V2 decide whether B2 can proceed on these terms.

## 7. Synthetic validation — before either bag is opened

Build a synthetic series and run the whole estimator on it first. This is the
standing rule on this question and three wrong answers were produced by
skipping it.

Construct truth yaw from a commanded profile: rest → ramp → constant rate →
ramp → rest, roughly 50 episodes spanning 5–200 deg/s, in two variants, one
pure rotation and one with simultaneous translation.

Construct odometry as:
- a known steady slip applied everywhere (use `-0.26 %`), plus
- a transient: let truth be a first-order lag of the odom rate with time
  constant 420 ms. This reproduces the measured shape difference — odom leads,
  the excess is positive on acceleration and negative on deceleration — without
  injecting a fixed time offset, which §3.5 of the handoff ruled out.

Requirements to pass:
- The **ungated** estimate must come out wrong, and must change when the
  episode mix changes. If it does not, the synthetic is not reproducing the
  problem and the test proves nothing.
- The **gated** estimate must recover `-0.26 % ± 0.10` in both mixes.

If the gate cannot do this on data with known injected values, it must not be
pointed at the bags.

## 8. How B2 consumes this

The accepted mask becomes a boolean column carried alongside each robot's
divergence series. Every divergence-versus-truth statistic reported in
Article 3 is computed over accepted samples only, and the retention figure is
reported next to it. Robot-to-robot comparisons of map or graph state do not
involve ground truth and are unaffected.

## 9. Running it

Terminal 1, no simulator required, non-blocking.

- Synthetic self-test: ~1 min.
- One bag: ~2 min. Both bags plus the threshold sweep: ~5 min.

Exit nonzero, naming the missing column or file, if an input CSV lacks a
required series — same convention as `yaw_truth_audit.py`.

## 10. Non-goals

- No correction of odometry, and no transient model fitted for use.
- No single odometry scale figure for NSKsim, gated or otherwise.
- No reuse of the retracted ~300 ms stream offset. There is no clock offset;
  that reading was this same shape difference misread.
