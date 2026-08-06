# rung3b — 2026-08-06

First run with `wander:=true`. Peers actually move. This is the
configuration Phase B has always assumed and, as far as the logs show,
had never been run.

Result: 21 goals, 4 escapes, halted defer-truncated at pose (-4.640, -0.337).
Explorer process then died (exit code 1); sim itself stayed up.

## The escape mechanism validated, 4/4

Every escape: nav2=SUCCEEDED, raw 253 INSCRIBED_INFLATED -> raw 0 FREE.
Displacement 0.37-0.49 m against a 0.60 m target (Nav2 tolerance 0.5
stops short). All were `escape #1`, so the counter reset correctly on
intervening successful goals.

Three fired within 67 s near (0, -1.3) — the robot escaped, re-approached
the same frontier, re-entered the same pocket. The fourth came 278 s later
at (-5.08, -0.10). Pockets are not one bad location.

Ray data from escape #1: cheapest of 7/16 drivable directions, samples
253,234,208,189,169,0,0,0... — cost falling monotonically into free space.

## The other verdict branch also fired, correctly

The halt was NOT a pocket. SLAM value=100 (occupied), raw=254 LETHAL,
9/9 of the 3x3 at 253-254, nearest obstacle 0.020 m.
Verdict: "pose error is plausible — the believed pose is inside mapped
structure". The escape correctly did not fire: displacement does not fix
a pose error. First real encounter with that branch, handled right.

## Peers were stationary in ALL prior runs

`wander` defaults to false. rung2f/2g/2h/2i all ran with robots 1-4 held
at their spawn poses. Consequences:
- The peer-residue mechanism in the 2026-08-03 handoff cannot have
  operated in those runs. Stationary robots are legitimate static
  obstacles, correctly mapped and correctly persistent.
- Their convergence numbers (~0.93) reflect five robots in a 0.9 m
  pentagon seeing nearly the same scene. Not meaningful divergence.

With peers moving: 4 pockets in ~6 minutes vs 0 in rung2i's 78 goals.

## Convergence became non-trivial

0.9297 (stationary) -> 0.8935 -> 0.8513 -> 0.8219, declining through the
run as maps diverged. Deltas are stepwise (0.0000 for stretches, then
+0.0050), consistent with the /nsk/merge timeouts: merges land only when
the engine keeps up.

## Hardware

4 cores, load average ~20-30 throughout. Top consumer is nsk_engine at
106% CPU / 1.9 GB RSS — not Nav2, not Gazebo. Five simultaneous Nav2
stacks were never launched; only explore.launch.py brings up a stack, and
per-robot configs exist only for robot_0 (nav2_robot0.yaml,
slam_robot0.yaml).

The wander driver is open-loop: robot_node.py contains no reference to
scan, obstacle, or avoidance. Peers execute a blind random walk and
collide with walls.

## Known-wrong output still present

The defer-truncated exit message advised raising PRECHECK_MAX_PROBES while
the actual fault was the pose. Open item 2 from the 2026-08-03 handoff,
still unfixed.

Halt was honest: 1064 frontier cells remaining, 27 retired-unexpired,
"NOT complete".
