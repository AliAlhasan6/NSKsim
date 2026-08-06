# rung2i — 2026-08-05

First run with the pocket-escape commit (8ed0123). Longest ON run to date.

Final: 57 SUCCEEDED, 11 FAILED, 10 FUTILE_NO_PROGRESS (78 goals, 73% success).
Killed manually, not a halt. Matches the OFF baseline rate (73.8%, n=172).
Prior ON runs died at 13 (rung2f), 8 (rung2g), 14 (rung2h).

## The escape never fired

No `escape #` line, no `Verdict:` line, no pocket formed. The new code is
still untested in anger. It cannot be credited for this run's length.

## Reach limit was pocket-specific, not a property of 253

rung2h measured plans succeeding at 0.43-1.81 m and failing beyond.
Here, targets at raw=253 planned successfully at poses=352, 351, 254, 394.
So 0.43-1.81 m described that one pocket's geometry, not inscribed cells
in general. Confirms ESCAPE_DISTANCE=0.6 was right to leave parameterised.

## Shuttle phase (goals ~47-59)

Goals alternated between (0.20, 3.34) and (3.93, -2.32), 6.3 m apart,
~50 s each, both marked SUCCEEDED, blacklisted=False throughout.
Frontier cluster shrank 114 -> 113 cells across ~6 visits.

Twelve goals for one cell of progress. NOT a permanent livelock: the run
broke out on its own once the map changed (frontiers moved to 77, 57, 18).
A shuttle phase, not a terminal state. Still a real inefficiency, and the
success counter reads as healthy throughout, which is the dangerous part.

Suspected cause: planner tolerance 0.5 lets Nav2 report success ~10 cells
short of the frontier at 0.05 m resolution, so arriving barely consumes it.

## Anomalies for later

- `INCONCLUSIVE` appearing with error_code=0 and successful plans. Per the
  handoff, triage() returns INCONCLUSIVE for 203/205; this arrives by some
  other path, uncharacterised.
- One candidate at raw=254 LETHAL returned REACHABLE with poses=394.
  Planning to a lethal goal cell should not succeed; tolerance 0.5 may be
  terminating the path near the goal rather than on it.
