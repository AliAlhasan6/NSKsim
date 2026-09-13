# HANDOFF -- 2026-09-13 -- B1.8 closed: the odometry/truth gap is transient slip

## 1. Where this stands

B1.8 was set up to produce one number: how much wheel odometry misreports
rotation against ground truth. **That question is closed as malformed.**
There is no scale factor to measure. Neither `+2.12 %` (b18_run2) nor
`-0.26 %` (b18_run3) is a property of the simulator, and neither should be
cited anywhere.

No simulator was run this session. Everything below comes from the two bags
already on disk.

## 2. Repo state

- `8c8b3aa` -- the `joint_states` bridge entry in
  `ros2_ws/src/nsk_swarm/launch/swarm_sim.launch.py`, committed at the start
  of this session. `b18_run3` cannot be reproduced without it.
- Four analysis scripts in `experiments/analysis/`, committed with this
  document.
- `experiments/logs/` is gitignored. Both bags and every analysis output
  exist in exactly one place on this machine.

## 3. What was measured, in order of dependency

### 3.1 Denominators were the first error

`run2`'s `+2.12 %` was taken against **net** rotation (29 853 deg), while its
**absolute** rotation is 51 215 deg. On absolute it is `-1.23 %`. `run3` is not
monotone either -- net/absolute 0.75, absolute 19 823 deg against net 14 762 deg --
giving `+0.21 %`. Compare runs on absolute rotation only. Net and absolute
differ by 1.7x in one run and 1.3x in the other.

### 3.2 The net was never the signal

Per 10 s chunk, `run2` accumulates **3264.5 deg** of unsigned error to leave a
net of **-629.0 deg**; `run3`, **796.4 deg** to leave **+41.0 deg**. Ratios 0.19 and
0.05. The worst decile of chunks holds only 27 % and 29 % of the error, so it
is not a few episodes either. The error is everywhere and alternates sign;
both published figures are cancellation residues.

### 3.3 Both hypotheses from the previous handoff failed

- **No accumulation.** Cut into four equal-rotation windows, `run2` reads
  `-3.07, -0.72, -2.01, +3.27 %`. Its own quarters disagree with each other
  by more than `run2` disagrees with `run3`.
- **Motion profile explains about half.** `run2`'s per-bin errors applied to
  `run3`'s distribution give `-0.59 %` against `run2`'s actual `-1.23 %`.
  The 55-200 deg/s rows -- pure Nav2 spin recovery, half of `run2`'s rotation
  and 71 % of `run3`'s -- **agree** at `+0.12 %` and `+0.09 %`. The divergence
  lives in the 5-45 deg/s rows: driving and turning at once.

### 3.4 The two runs were never distinguishable

Anchoring segments on moments of rest makes the scale measurement immune to
any timing difference by construction: a shift changes nothing while yaw is
constant in both streams. Point estimates, stable across every anchor
threshold swept: `run2` `+2.86 %`, `run3` `-0.48 %`.

Bootstrapping over whole segments: **`[-0.28, +4.33]` and `[-0.82, +5.21]`.
They overlap.** The order-of-magnitude disagreement chased across three
handoffs is segment scatter -- residual RMS 47.7 deg per segment in `run2`,
13.4 deg in `run3`.

`run2` had 36 stopped anchors spanning 95 % of its duration. `run3` had none
at all; it never stood still for 0.4 s, so its figure rests on zero-rate
crossings, and its slope, weighted median and trimmed slope give `-0.48`,
`-0.03` and `+1.16 %` -- three answers. The previous handoff had this exactly
backwards: `run3` was the figure being trusted.

### 3.5 The mechanism

Model-free onset timing on `run2`'s 38 rest->motion transitions:

| stream | median rise, 2->40 deg/s |
|---|---|
| odometry | 180 ms |
| ground truth | 620 ms |

Paired **within** each transition, truth is slower by **+420 ms**, IQR 80 ms,
p10 +348, p90 +532, in **38 of 38** transitions. The unpaired IQRs of ~600 ms
collapse to 80 ms when paired: the variation was between transitions, not
within them.

The onset gap **grows with threshold** -- 40, 80, 120, 230, 470 ms at 2, 5,
10, 20, 40 deg/s -- which rules out a transport delay and identifies a
difference in *shape*. The synthetic control carrying a pure 300 ms offset
returns -300 ms flat across all thresholds and a paired difference of +20 ms,
one sample at 50 Hz, the quantisation floor.

**Reading.** The wheels reach commanded speed about 420 ms before the body
does. Through that window the wheels turn against a body that cannot follow
-- slip at the contact -- and odometry over-reports rotation. On deceleration
the sign reverses. A run's net error is the residue of these transients, and
its sign and magnitude depend on the mix of accelerations in that run. That
is why `run2`'s own quarters disagree, and why two runs of the same system
give different answers.

Odometry is *not* at fault as an integrator: `run3`'s joint states put
published odometry at `+0.002 %` against wheel kinematics. The discrepancy is
entirely wheel-against-ground.

**The earlier timing-offset reading was this same effect misread.** A shape
difference fits as a delay whose apparent size depends on the frequency band
sampled, which is exactly why the fitted tau grew monotonically with stencil
width (-208 ms at k=1 to -396 ms at k=30) and why the shift scan's minimum
sat near the edge of its range. There is no clock offset to fix.

## 4. What this means for the thesis

B1.7's rotated map copies need no precise figure: anything in 0-4 % over 142
revolutions accounts for them.

The citable claim is the mechanism, not a rate. In a Nav2-driven simulation,
odometry yaw error is dominated by transient slip during rotational
acceleration; a single-number "odometry drift rate" is an artefact of the
motion mix in whichever run produced it. **Do not report one odometry scale
figure for NSKsim.**

## 5. Method rules earned here

- Compare on absolute rotation, never net.
- The diagnostic statistic is summed |excess| over fixed chunks, never the
  net. The net is insensitive to any telescoping term by construction --
  which is why the earlier +/-100 ms lag scan found nothing.
- Anchor on rest whenever a timing difference between streams is possible.
- Pair within transitions when unit-to-unit variation is large.
- Never sum |odom increments| over ~10^5 samples; the noise floor has
  produced three wrong answers on this question already.
- Every estimator was validated against a synthetic series with injected
  values *before* being pointed at the data, and every check prints a verdict
  it is capable of failing. Two failed in use and were discarded: `run2`'s
  zero-rate anchors (immunity 0.392 points) and `run3`'s stopped anchors
  (zero found).
- A point estimate without an interval settled nothing for three sessions.

## 6. Tooling

All in `experiments/analysis/`, outputs in `experiments/logs/b18/analysis/`
and `experiments/logs/b18/*.txt`.

- `rate_profile.py` -- rate and speed tables, ratexspeed cross-tab,
  equal-rotation windows, 10 s episode chunks, per-sample CSV export.
  Carries a reproduction check against the published `run2` totals.
- `lag_probe.py` -- two-parameter fit (`excess = a*domega + b*d_truth`) with a
  stencil-width sweep, plus an exact shift scan.
- `rest_anchors.py` -- rest-anchored scale with an immunity check,
  per-segment distribution, bootstrap CI, threshold sensitivity sweep.
- `onset_probe.py` -- model-free onset timing, threshold sweep, rise times,
  paired within-transition comparison.

Known defect: `onset_probe.py` prints a literal `50 %%` in one message.

## 7. Next session

**B1.8 is closed. Nothing further is owed to it.**

Quantifying the transient properly is optional and needs a *designed* run,
not more analysis of these bags: one robot, no Nav2, no SLAM, headless;
rest 2 s -> spin at a commanded rate for N seconds -> rest 2 s, repeated ~50
times across the rate range, with and without translation; record truth,
odometry and joint states. That turns 33 noisy segments into ~50 clean ones
and maps the transient against commanded acceleration. It is light enough
for this machine -- one robot, no navigation stack.

Otherwise go to B2. One thing carries forward: **any B2 divergence
measurement against ground truth must exclude acceleration transients**, or
it inherits this error. At 0.22 m/s that is 6.6 cm of position, and at the
spin cap the heading gap through a transient reaches tens of degrees.
