# EXP-01: Convergence fragmentation — topology vs. swarm size

**Status:** designed, not run
**Origin:** 2026-07-09 convergence run (`experiments/logs/2026-07-09_convergence_run_5robots.log`)

## Observation
5-robot run: mean pairwise similarity rose 0.174 → 0.510 (t≈130s), then
declined monotonically to 0.281 (t=303s). Cause visible in merge-pair counts:
swarm fragmented into two static pairs (1↔4 @ 0.40m, 0↔2 @ 2.41m) plus an
isolated robot (3). Within-pair merging continued; clusters drifted apart.

## Competing hypotheses
- **H-topology:** decline is caused by the communication graph freezing into
  disconnected components. Consensus theory: global agreement requires the
  union of communication graphs over time to be connected. Predicts: decline
  persists at larger N if pairing/parking behavior is unchanged (more, bigger
  clusters, slower decline).
- **H-size:** decline is an artifact of small N (one parked pair removes 40%
  of a 5-robot population; no third robot bridges clusters). Predicts: decline
  vanishes or strongly attenuates at larger N via natural re-mixing.

## Experiments
1. **N-sweep:** N ∈ {3, 5, 8}, same world, same duration (~6 min), 3 seeds each.
   Metric: mean-sim trajectory; time-to-peak; post-peak slope.
   (N=8 is the expected CPU ceiling — all agents on one core.)
2. **Mixing intervention (key experiment):** N=5 fixed; force periodic
   re-mixing (e.g., trigger EXPLORE every 60s regardless of peers).
   If decline disappears without changing N → topology isolated as cause.
   Stronger claim, confound held fixed.

## Confounds / notes
- Merge weighting (0.7 self / 0.3 received) interacts with both hypotheses;
  full design would be a grid (N × mixing), budget permitting.
- Frozen dist values suggest FLOCK drives pairs into stable parking equilibria —
  behavior-side issue, worth a look independent of NSK.
- Requires per-cycle CSV export from the monitor (currently log-parse only) —
  small instrumentation task before running.

## Relevance
- Thesis: empirical evidence that embedding-level knowledge sharing produces
  cluster consensus, not global consensus, without graph mixing.
- Candidate experiment for ВАК paper #2 alongside compressor sensitivity
  (retention ratio and mixing rate are both knowledge-flow knobs).
