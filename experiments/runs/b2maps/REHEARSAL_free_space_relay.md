# Rehearsal — free_space_relay, online

A short run that answers one question: does feeding slam_toolbox a scan whose
no-return beams clear free space change the map, and nothing else?

Everything here is robot_0 and about 20 minutes of attention: ~5 min of
setup, a ~12 min sim, ~5 min of checks. The procedure is the one in
[README.md](README.md) — same terminals, same logs-only rule, same teardown.
**One line differs**, T3:

```bash
ros2 launch nsk_swarm explore.launch.py robot_id:=$K scan_matching:=false \
    free_space_relay:=true \
    > $LOGS/${RUN}_explore.log 2>&1
```

Use `RUN=b2maps_relay_rehearsal1`.

**What this is for.** b2maps_e0t ran 111 minutes on true poses and mapped
63 m² of a 391 m² floor, never getting within 4.18 m of the perimeter. Karto
ignores any reading at or above the scan's `range_max`, `+inf` included
(`karto_sdk/Karto.h:6169`), so the 57.47% of e0t's 11.7M beams that hit
nothing cleared nothing. The relay rewrites those beams to 7.95 m, between
slam_toolbox's `max_laser_range` (7.9 with the flag on) and the scan's
`range_max` (8.0), which is the one band Karto traces as free space without
marking an obstacle.

**The relay is half of a two-part change, and it is the half that cannot
work alone.** Karto sizes its occupancy grid from FILTERED readings only —
`LocalizedRangeScan::Update()` (`Karto.h:5644`) builds `m_BoundingBox` from
readings that pass `InRange(reading, minRange, rangeThreshold)`, and a filled
beam fails that test by construction. So a filled beam can never enlarge the
map box; it can only fill in the box that genuine returns already defined.
Measured on e0t at 3.5 m: the relay took free area from 53.56 → 75.73 m² and
left the known box at exactly 12.00 × 8.00 m, unchanged. **Range sets the box,
the relay fills it.** That is why `BURGER_RANGE_MAX` moved to 8.0 m in the same
change — e0t's trajectory is 7.44 m from the south wall at its closest, so
below that at least one boundary wall never enters a single scan.

Expected together, on e0t's own frozen trajectory: box 20.00 × 20.00 m, free
area ~335 m², ~84.6% of the floor. Those are **lower bounds** — that trajectory
was itself shaped by a 3.5 m sensor and never left an 11.37 × 5.75 m band — so
a live run should beat them, and failing to reach them means something other
than range is limiting the map.

---

## R0. Build, and the one trap in it

```bash
cd ~/Desktop/NSKsim
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash        # for nsk_swarm_interfaces ONLY
colcon build --packages-select nsk_swarm
source ~/Desktop/NSKsim/install/setup.bash
ros2 pkg executables nsk_swarm | grep free_space_relay
```

**Expect** `nsk_swarm free_space_relay`.

`ros2_ws/install` has to be sourced **for the build** and not for the run:
`nsk_swarm_interfaces` was built there on 2026-09-10 and exists nowhere else,
so without it colcon fails with *Failed to find .../nsk_swarm_interfaces/
share/nsk_swarm_interfaces/package.sh*. The README's warning still stands for
everything after the build — that tree's `nsk_swarm` is stale and must not
shadow the one you just built, which the `ls` in R0 of the README checks.

## R1. The default is untouched — no simulator needed

```bash
python3 - <<'PY'
import importlib.util
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
s = importlib.util.spec_from_file_location(
    'ex', 'ros2_ws/src/nsk_swarm/launch/explore.launch.py')
m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
ld = m.generate_launch_description()
def resolve(**a):
    c = LaunchContext(); c.launch_configurations.update(a)
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument): e.execute(c)
    out = []
    for e in ld.entities:
        out += (e.execute(c) or []) if isinstance(e, OpaqueFunction) else [e]
    return [getattr(x, 'node_executable', None) for x in out]
print('default :', [x for x in resolve() if x])
print('relay on:', [x for x in resolve(free_space_relay='true') if x])
PY
```

**Expect** `['async_slam_toolbox_node', 'frontier_explorer']` by default and
`['free_space_relay', 'async_slam_toolbox_node', 'frontier_explorer']` with
the flag. The default must not merely suppress the relay — it must not be in
the tree, and slam_toolbox must be handed the same four parameter keys it was
handed before this flag existed, so `slam_robot0.yaml` stays the one source
of `scan_topic` and `max_laser_range`.

And the offline half of the same switch:

```bash
md5sum experiments/logs/offline_mapping_b2maps_e0t_kp_robot_0.yaml
sed -e s/__ROBOT__/0/g -e s/__SCAN_MATCHING__/false/g \
    -e s/__SCAN_SUFFIX__/scan/g -e s/__MAX_LASER_RANGE__/3.5/g \
    experiments/slam/offline_mapping.yaml.template | md5sum
```

**Expect** `b06c960b4e9824e7884db4ceca909b17` twice — the template now carries
placeholders where those two values were, and a default render must still be
the byte-identical file that built every existing map.

`3.5`, not `8.0`, and that is the point: this reproduces the params of a bag
recorded on the old sensor, so it substitutes what *that bag* carries.
`run_offline_maps.sh` reads the value off the bag rather than from a literal,
which is what lets this digest survive the sensor change unchanged. The relay
render of the same bag is checked the same way:

```bash
md5sum experiments/logs/offline_mapping_b2maps_e0t_kpfree_robot_0.yaml
sed -e s/__ROBOT__/0/g -e s/__SCAN_MATCHING__/false/g \
    -e s/__SCAN_SUFFIX__/scan_free/g -e s/__MAX_LASER_RANGE__/3.4/g \
    experiments/slam/offline_mapping.yaml.template | md5sum
```

**Expect** `7ecb52b38f2f7509a5b08a7f24b885bf` twice.

## R2. The relay is up, and filling

Once T3 is running:

```bash
grep -a 'free-space relay\|beams filled' $LOGS/${RUN}_explore.log | head -3
```

**Expect** the startup line naming `/robot_0/scan -> /robot_0/scan_free`, then
a line every 200 scans:

```
200 scans, 14958/72000 beams filled (20.8%), fill 7.950 m
```

**The fill must read 7.950 m.** Anything else means the threshold and the
scan's `range_max` are not the pair this rests on — and the specific failure to
suspect is a threshold left behind at 3.4, which would rewrite every genuine
return between 3.5 and 8.0 m and erase real walls into free space.

**Roughly 15–30% filled is the expected band, not the 57% of the 3.5 m era.**
Raising the sensor converts no-returns into returns: on e0t's trajectory the
fraction of beams that come back rises from 41.7% at 3.5 m to 79.5% at 8 m, so
only about a fifth of beams still need filling. A reading near 57% here means
the scans are still 3.5 m ones — check `ros2 topic echo --once
/robot_0/scan --field range_max` before going further. It runs higher than 20%
while the robot is in open floor, and much higher near the middle of the room.

*Can it fail?* Launch without `free_space_relay:=true`: no relay, no lines,
and `/robot_0/scan_free` does not exist.

## R3. slam_toolbox is reading the relay, and only slam_toolbox

```bash
ros2 param get /robot_0/slam_toolbox scan_topic        # /robot_0/scan_free
ros2 param get /robot_0/slam_toolbox max_laser_range   # 7.9
ros2 topic echo --once /robot_0/scan --field range_max # 8.0
ros2 topic info /robot_0/scan_free --verbose | grep -c 'Node name'
ros2 topic info /robot_0/scan --verbose | grep 'Node name'
```

**Expect** `scan_topic` `/robot_0/scan_free`, `max_laser_range` `7.9`, a scan
`range_max` of `8.0`, exactly **two** endpoints on `/robot_0/scan_free` (the
relay publishing, slam_toolbox subscribing), and on the raw `/robot_0/scan` the
relay plus **both Nav2 costmaps**.

The `range_max` check is not redundant with the other two. It is the only one
that proves the SENSOR moved rather than just the parameters: a stale build, or
a Gazebo that reused a cached model, gives 7.9 / `scan_free` on a 3.5 m scan,
and the relay then refuses to fill (no value is both traced and not dropped),
logs once, and passes everything through — a run that looks configured and
produces the no-relay map.

Nav2 stays on the raw scan deliberately. The fill is a convention that means
something only against Karto's range threshold; an obstacle layer reading
7.95 m would place an obstacle there. Today's `obstacle_max_range: 2.5`
happens to be below the fill — as it was below the old 3.45 m one — so it would
be ignored anyway, but that safety is a coincidence of two unrelated numbers,
and raising that range later would turn every no-return beam into a phantom
ring. If the costmap should clear on no-return beams, Nav2's own
`inf_is_valid: true` is the knob, and it is a separate change with its own
rehearsal.

## R4. The map is not ringed with phantom walls

```bash
ros2 run nav2_map_server map_saver_cli -t /robot_0/map -f /tmp/relay_check  \
  || python3 experiments/analysis/run_health.py --bag ... --save-map ...   # after the run
```

Eyeball it, once, in the PNG R7 writes afterwards. **Expect** walls where the
world has walls. A band of occupied cells at a constant 7.95 m from the
trajectory is the failure this check exists for: it would mean the fill landed
below the range threshold and Karto marked it as a hit.

Numerically, after the run: occupied cells should RISE, but only in proportion
to how much more boundary is now visible — the ray-cast estimate on e0t's own
trajectory is ~1650 at 8 m against its recorded 1340 at 3.5 m. A multiple of
that, or anything near 360 × the number of scans, is the phantom ring.

## R5. map -> odom stays identity

```bash
ros2 run tf2_ros tf2_echo robot_0/map robot_0/odom
```

**Expect** translation `[0.000, 0.000, 0.000]` throughout — `scan_matching` is
still false, so nothing moves it. The relay changes what is mapped, not where
the scans are placed.

## R6. More floor becomes known than e0t ever managed

The point of the whole change. During the run:

```bash
grep -a 'frontiers\|exploration complete' $LOGS/${RUN}_explore.log | tail -5
```

and afterwards, from the bag:

```bash
python3 experiments/analysis/run_health.py --bag $LOGS/$RUN --robot 0 \
    --save-map experiments/maps/${RUN}_online_robot0
```

**Expect** a known extent clearly past e0t's 12.05 × 8.00 m. A 12-minute
rehearsal will not reach the perimeter — coverage stays a criterion for the
real run — but its map should already be wider than a 12-minute run had any
business being, because the box now grows with the 8 m sensor rather than
stopping 3.5 m from the path.

The two anchors, and what each actually shows:

| | free area | known box |
|---|---|---|
| e0t offline, 3.5 m, no relay (`b2maps_e0t_kp`) | 53.56 m² | 12.00 × 8.00 m |
| e0t offline, 3.5 m, relay (`b2maps_e0t_kpfree`) | 75.73 m² | 12.00 × 8.00 m |

The relay is worth **+41%** of free area on the same bag, and **zero** box —
the two maps have identical extents. (An earlier draft of this document said
the relay "more than doubled the free area"; the artifacts above say +41%, and
they are what the check should be read against.) That unchanged box is the
whole reason the range moved with it: it is not a shortfall in the relay, it is
`Karto.h:5644` sizing the grid from filtered readings, which a filled beam is
not. Expect the range to move the box and the relay to fill it, and if the box
grows while the free area does not, the relay is not running.

## R7. The stack is no unhappier than before

```bash
python3 experiments/analysis/run_health.py \
    --log $LOGS/${RUN}_explore.log \
    --baseline-log $LOGS/b2maps_e0_explore.log \
    --bag $LOGS/$RUN --robot 0 --truth-tf --tf-rates
```

**Expect** R9's two gated counts still **0**, R2 at 20.001/50.001 Hz and R4
exact, exactly as rehearsal 5 had them. The relay adds a node and a topic to a
rig with a history of starving under load, so this is not a formality: if
transform warnings appear here they are the relay's, and the run is not clean.

**Read a failure here carefully, because the baseline is no longer like-for-
like.** `b2maps_e0_explore.log` was recorded on a 3.5 m sensor, where the map
stayed at 63 m². At 8 m with the relay it is several times that, so the
explorer scans several times the cells per cycle, the costmaps carry more, and
`PRECHECK_MAX_PROBES 12` / `PRECHECK_CYCLE_BUDGET 10.0 s` bind much harder.
R9's 0.000/min stays the right criterion — a transform warning is still a
transform warning — but a first failure against it is at least as likely to be
the new load as a defect in the relay. Distinguish them by re-running the
rehearsal with `free_space_relay:=false` at 8 m: if the warnings persist
without the relay, they are the range's, and the relay is not the thing to fix.

COVERAGE will now pass on almost anything; see the note on criterion 1 in
[README.md](README.md). Criterion 5 — occupied cells on the outer boundary —
is the one that means something here.

---

## Then, and only then

If R1–R7 hold, re-run **e0t** in full with `free_space_relay:=true`, by the
README's procedure, and compare against the e0t already recorded: same world,
same poses, same matcher setting, **two** differences now — the relay and the
sensor. Keep both bags. The old one is the control, and it is the only record
of what the 3.5 m ceiling did.

Because there are two differences, a straight A/B no longer attributes
anything. If the new run needs to be explained rather than merely accepted,
the attribution is already available without a simulator: `b2maps_e0t_kp` and
`b2maps_e0t_kpfree` isolate the relay at a fixed 3.5 m (+41% free area, no box
change), and the ray-cast table in the plan isolates the range at a fixed
trajectory. Between them each half is accounted for.

The offline half of every run already recorded can be rebuilt through the
relay without touching the simulator:

```bash
BAG=experiments/logs/b2maps/<run>_slamin RUN=<run>_kpfree \
    SCAN_MATCHING=false FREE_SPACE_RELAY=true RATE=2.0 \
    bash experiments/slam/run_offline_maps.sh 0
```

which is how the change was checked before any of this was run online.

**Those replays stay at 3.5 m, and that is correct.** A bag's `range_max` is
whatever the sensor was when it was recorded, so `run_offline_maps.sh` reads it
off the bag (`experiments/slam/bag_range_max.py`) and derives `max_laser_range`
and the relay threshold from that — 3.4/3.45 for every existing bag, 7.9/7.95
for anything recorded after the change. It is not a setting to override. Doing
so is silent in one direction and quietly destructive in the other: too high a
`max_laser_range` is clipped by Karto to the scan's own `range_max` and does
nothing, while too high a relay threshold leaves no valid fill, so the relay
passes every scan through unchanged and writes the no-relay map under a
`_kpfree` name. The script refuses that pairing rather than running it.
