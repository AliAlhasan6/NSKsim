# rung2h — 2026-08-04

First run with the global costmap subscription (translated + raw).

Result: halted at 14 goals, defer-truncated, pose (-2.047, -2.877).

Robot's own cell: raw=253 INSCRIBED_INFLATED (translated 99 COST).
3x3 neighbourhood all 253, one 254 corner.
Nearest SLAM-occupied 0.072 m away, raw=254 LETHAL.

Key finding: the start is INSCRIBED, not LETHAL. Navfn can expand from
253, which is why the start probe returned START_OK on every cycle.
Handoff 2026-08-03 section 3 ("start cell lethal, every plan impossible")
is therefore incorrect.

Cost distribution over 50 precheck readings:
  20 raw=255 NO_INFORMATION
  18 raw=0   FREE
   7 raw=253 INSCRIBED_INFLATED
   5 raw=193/196 COST

Note: goals on unknown (255) cells both succeeded (poses=62, poses=36)
and failed (poses=0), so goal-cell cost is NOT the discriminator.

Defect found: the termination diagnostic's closing sentence claims an
occupied own-cell means SLAM pose error and advises against a standoff.
Own cell read value=-1 (unknown), not occupied. Advice is inverted.

Gazebo screenshot at halt: five robots in open floor, well separated,
no physical entrapment.
