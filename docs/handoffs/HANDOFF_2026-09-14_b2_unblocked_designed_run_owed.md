# HANDOFF — 2026-09-14 — B2 unblocked; the designed run is now owed

## 1. Where this stands

B1.8 closed last session with the mechanism identified as transient wheel slip
during rotational acceleration. This session asked one question: can B2's
comparisons against ground truth be rescued by *excluding* the transients,
rather than by recording a new calibration run?

**They cannot.** The exclusion gate works, and what it removes is the entire
contested band. What survives is the band that already agreed.

The consequence is narrower than it sounds. Only truth-referenced statistics
are affected. B2's actual object — robot-to-robot divergence between
independently built maps and graphs — never touches ground truth and is
untouched. **B2 proceeds. The designed run of §6 below is now owed for any
truth-referenced claim in Article 3, not optional.**

No simulator was run this session. Everything comes from the two b18 bags and
their exported CSVs.

## 2. Repo state

Three commits on `main`, none pushed:

- `2b3d115` — `export_grids` in `rate_profile.py`, consumed by `steady_gate.py`.
- `796a7ee` — `steady_gate.py`.
- `7402446` — `--decay` mode in `onset_probe.py`.

**Outstanding:** the spec document `SPEC_b2_steady_gate.md` was written in chat
and never landed on disk, so `steady_gate.py` was committed citing sections of
a file the repo does not contain. Drop it into `docs/specs/` and commit. Note
when you do that V5 and the interval-emitting mask **post-date** the spec —
the script is ahead of it, deliberately, and commit `796a7ee` records why.

`experiments/logs/` is gitignored. The two decay logs
(`onset_decay_synthetic.txt`, `onset_decay_run2.txt`) exist in one place on
this machine, which is why their numbers are reproduced in full below.

Unrelated and still untracked: `wall_affinity.py`, its test, `.vscode/`.

## 3. The gate, and why it fails

| | run2 | run3 |
|---|---|---|
| rotation retention | 23.8 % | 49.7 % |
| path-length retention | 1.1 % | 3.7 % |
| accepted rotation from 55–200 deg/s | 99.1 % | 99.9 % |
| retention in contested 5–45 deg/s | 0.2 % | 0.0 % |
| V2 ratio | 3.7 % | 0.3 % |

V1 (retention) and V2 (excess reduction) both pass. **They pass hollowly.**
The gate removes precisely the band B1.8 located the divergence in and keeps
precisely the band that already agreed across runs at `+0.12 %` and `+0.09 %`.
Accepted windows are monotone in-place spin, net/absolute 1.000. The `|excess|`
falls because the contested rotation was removed, not because a transient was
removed from it.

A verdict that passes because the measurement collapsed onto an uncontested
subset is a failure, and it has to be recordable as one. **V5 CONTESTED-BAND
was added and wired as deciding.** Exit code 2; the script refuses to hand the
mask to B2.

The same gate retains **69 %** of the contested band on the synthetic. This is
a property of Nav2-driven motion, not of the gate — Nav2 never holds a steady
turn while also translating — and no threshold recovers it from these bags.
That is the evidence for §6.

V3 (agreement) still fails, but the gated intervals are `[+0.064, +0.503]` and
`[−0.014, +0.003]`: under half a point apart, against the published ungated
`[−0.28, +4.33]` and `[−0.82, +5.21]`. Record that the gap narrowed by an
order of magnitude on pure spin; do not loosen the criterion. V4 (flatness)
passes at 0.020 and 0.000 points, and only prints when the sweep runs.

**S3 fails on the synthetic** at the specified `T_GUARD = 0.60 s`, missing the
injected `−0.26 %` by 1.638 points. A τ = 0.42 s first-order lag's tail far
outlasts its rise, and 0.60 s was sized on B1.8's measured *rise* distribution.
Those are different quantities and the tail was never measured. Raising the
guard to ~2.4 s would pass the check against an unvalidated model and take
retention to zero. **The default stays 0.60 s with the caveat printed.** S0,
S1 and S2 pass: the injected transient reproduces all five measured onset
thresholds, and the ungated estimate is both wrong and mix-dependent.

## 4. The decay measurement

Run to size the rest interval in §6; §7 of the previous handoff guessed 2 s.

`onset_probe.py --decay` mirrors the rise path over motion→rest transitions.
Three bugs in the working-tree skeleton were fixed: a docstring closed early
leaving six lines of prose as bare Python; `settles()` calling `last_crossing`
with 4 args against a 5-arg signature and iterating `for i0 in starts` where
`starts` holds pairs; and, exposed by the arity fix, a forward peak-rate scan
that — since `i0` is by construction the last sample above 40 deg/s — bucketed
every transition as 40–60 regardless of the plateau it spun down from. Peak is
now read by walking back through the motion run, bounded by the previous rest.
The 38-transition rise result reproduces byte-identical apart from the `50 %%`
fix.

Synthetic passes D0 (truth falls 2.5× slower than it rises) and D1 (settle
1360 ms against the closed form's 1417 ms).

**run2, 33 transitions: settle median 160 ms, p90 660, max 760.**

Two caveats on that number, both load-bearing:

- The fall ladder is **not monotone** — 160, 280, 260, 140, 140 ms at 2, 5,
  10, 20, 40 deg/s. Odometry ramps down over 1120 ms on the bag against 60 ms
  on the synthetic, so it is not one lag unwinding, `tau*ln(w/2)` does not
  apply, and the settle is a **gap measurement only**, not a decay constant.
- **All 33 transitions decayed from 40–60 deg/s.** Nothing above 60 ever
  reaches rest in run2. This sizes the Nav2 spin band and nothing else.

"Truth still moving for", measured from the last fast sample rather than from
odometry's crossing, is 1220 ms median with p90 4296 ms. The two columns
differ by an order of magnitude, so **which end the rest interval is timed
from changes the answer** — hence the condition-based rule in §6 rather than a
constant.

The `cmd_vel` split that would have separated clean stops from transitions
re-commanded mid-decay is **unavailable**: `cmd_vel` was never recorded in
`b18_run2`. Not pending — unavailable.

## 5. The asymmetry — open, and the reason §6 sweeps both directions

Rise gap `+420 ms` (IQR 80), fall gap `160 ms` (IQR 480).

Pairing within transitions collapsed the rise IQR from ~600 ms to 80 ms. It
does **not** collapse the fall. That says the decelerations are not one
phenomenon, and the 160 ms median may be a mixture rather than a shape.

B1.8's "the sign reverses on deceleration" was inference from the mechanism,
never measured. This is the first evidence that it may not be symmetric in
magnitude. **It stays provisional, and it cannot be resolved from these bags** —
without `cmd_vel` the mixture cannot be split.

## 6. §7 amended — the designed run

Unchanged from the previous handoff: one robot, no Nav2, no SLAM, headless;
spin episodes across the rate range, with and without translation, ~50 of
them; record truth, odometry and joint states. Light enough for this machine.

Three amendments earned this session:

1. **Record `cmd_vel`.** Without it the designed run lands exactly where §4
   landed — unable to tell a clean stop from a re-commanded one.
2. **Rest interval is condition-based, not a constant.** Hold until truth
   stays under 2 deg/s for a fixed hold; floor 2 s, cap 6 s; log the actual
   wait per episode. A constant sized off Nav2 data inherits Nav2's behaviour,
   and §4's settle covers only 40–60 deg/s anyway.
3. **Sweep decelerations as well as accelerations.** §5 is the reason. Do not
   assume one cancels the other.

## 7. What these bags cannot answer

Do not spend another session on them.

- Whether the fall-gap spread is a mixture or a shape — needs `cmd_vel`.
- Whether the sign reverses symmetrically in magnitude.
- Any truth-referenced statistic in the 5–45 deg/s band: Nav2 never holds
  steady motion there, so there is nothing to gate to.

## 8. Method rules earned here

- A verdict that passes because the measurement collapsed onto an uncontested
  subset is a failure. Make it recordable — that is what V5 is.
- Never raise a threshold until a check passes when the check's model is
  itself unvalidated. Measure the model instead, or print the FAIL with the
  reason attached.
- Retention is a first-class output, not a footnote. A gate that keeps
  nothing is a finding.
- Build the mask from the stream that is not the reference, or the gate
  selects on the quantity being measured.
- Emit masks as time intervals on the source grid, not per-index flags — they
  then survive a change of grid.
- Check what the bag actually recorded before designing an analysis around a
  topic.

## 9. Next session

**B2.** Divergence is robot-to-robot and nothing here blocks it. Carry forward
only this: any statistic referenced to ground truth is deferred until the
designed run exists.
