# NSKsim — Technical Documentation

**Repository:** github.com/AliAlhasan6/NSKsim · **License:** MIT · **Status:** Phase A complete (hardening finished, CI green)
**Stack:** ROS 2 Jazzy · Gazebo Harmonic · rclpy · PyTorch (CPU) · PyTorch Geometric · Ubuntu 24.04

This document is the internal engineering reference for NSKsim. It records what the system is, how it is put together, why each significant design decision was taken, and what is knowingly left open. The public README covers the pitch and quick start; this document covers the reasoning, and it is written so that the project can be picked up cold — by a future phase of this work or by another engineer — without re-deriving any of it.

---

## 1. Purpose and context

NSKsim is a multi-robot simulation built for the purpose of studying distributed knowledge sharing under bandwidth constraints, and it is the simulation arm of the NSK (neuro-symbolic knowledge) research line. Five differential-drive robots move in a Gazebo world; each carries a local knowledge graph (an FB15k-237 ego-graph assigned at startup), and when robots come within communication range, one compresses its graph, broadcasts it, and peers merge the received knowledge into their own embedding-level state. The scientific question underneath is what a "language for machines" should transmit when it cannot transmit everything; the engineering question this repository answers is how to host that experiment on a production-grade ROS 2 architecture rather than a research script.

The system began as a prototype with hand-rolled ZMQ transport and was hardened across four steps: (1) baseline inspection, (2) migration of all transport to typed DDS services, (3) launch, QoS, concurrency, and lifecycle robustness, and (4) tests, CI, and publication. That history matters because several design decisions below are best understood as corrections of specific prototype failures, and those failures are documented alongside the fixes.

---

## 2. System architecture

```
swarm_sim.launch.py            (single entry point; num_robots = 5, single source of truth)
│
├── nsk_engine                 rclpy LifecycleNode — hosts all PyTorch models
│     on_configure : load config + checkpoint, build AgentManager (~12 s)
│     on_activate  : create the three /nsk/* service servers
│     services     : /nsk/compress · /nsk/merge · /nsk/similarity_query
│
├── robot_0 … robot_4          Gazebo-embodied agents (FLOCK / EXPLORE behaviour)
│     clients of /nsk/compress and /nsk/merge
│     pub /kg_share (graph broadcast) · pub cmd_vel · sub own + peer odom
│
├── convergence_monitor        client of /nsk/similarity_query (10 s cycle)
│     sub /kg_share (merge-event counting) · sub all odom (range gating)
│
├── ros_gz_bridge              odom / cmd_vel bridging to Gazebo Harmonic
├── gz sim                     physics + world (worlds/knowledge_world.sdf, self-contained)
└── rviz2                      visualisation (rviz/nsk_convergence.rviz)
```

**One knowledge-sharing cycle.** A robot calls `/nsk/compress`; the engine compresses that agent's graph (retention ratio selected by the robot) and returns it as `graph_json`. The robot broadcasts the payload on `/kg_share`. Every peer within `comm_range` (3.0 m, 2D Euclidean on odom positions) that hears the broadcast calls `/nsk/merge` with the received graph; the engine embeds it with the GATv2 encoder and fuses it into the receiver's persistent embedding state. The convergence monitor, on a 10-second timer, queries `/nsk/similarity_query` for the full pairwise cosine-similarity matrix and tracks swarm-wide convergence.

**The NSK pipeline inside the engine** is a three-stage architecture: a heuristic graph compressor (structural and surprise scoring with bridge-node preservation), a trained GATv2 embedder producing 32-dimensional graph embeddings (trained on FB15k-237 ego-graphs; checkpoint `joint_best.pt`), and a merger. Note honestly: the live `/nsk/merge` path uses a validated *embedding-level* blend — `z* = L2-normalise(0.7·z_self + 0.3·z_received)` with a constant reported gate of 0.5 — because the trained merger's graph encoder is out-of-distribution on compressed graphs (recorded in `agent_manager.py`'s own docstring). The trained merger is loaded but bypassed.

---

## 3. Package layout

```
NSKsim/
├── .github/workflows/ci.yml        CI: build + test in ros:jazzy container
├── README.md · LICENSE · TECHNICAL.md (this file) · requirements.txt
├── experiments/
│   ├── designs/EXP-01_fragmentation_vs_swarm_size.md
│   └── logs/2026-07-09_convergence_run_5robots.log
└── ros2_ws/
    └── src/
        ├── nsk_swarm_interfaces/   ament_cmake — service definitions only
        │   └── srv/  Compress.srv · Merge.srv · SimilarityQuery.srv
        └── nsk_swarm/              ament_python — all behaviour
            ├── nsk_engine/         engine_server.py · agent_manager.py
            ├── nsk_swarm/          robot_node.py · convergence_monitor.py · graph_serialiser.py
            ├── launch/  config/  worlds/  rviz/
            └── test/               55-test pytest suite (see §9)
```

The split into two packages is deliberate: interface definitions live in their own `ament_cmake` package because `rosidl` generates code from them at build time, while `nsk_swarm` is pure `ament_python` (the prototype's mixed CMakeLists+setup.py build was removed — a mixed build is ambiguous to colcon and broke tooling). Entry points: `ros2 run nsk_swarm nsk_engine | robot_node | convergence_monitor`.

---

## 4. Service interfaces

The contract between the engine and its clients consists of three services. The design convention throughout: typed scalar fields for identifiers and metrics; JSON-encoded strings for variable-shape tensors (graphs, matrices), since their dimensions vary per call and full typing would buy little at the cost of interface rigidity; and a uniform `success`/`message` pair replacing the prototype's ad-hoc error dictionaries, such that callbacks never raise across the service boundary.

| Service | Request | Response |
|---|---|---|
| `/nsk/compress` (`Compress`) | `int32 agent_id`, `float32 retention_ratio` (−1.0 = compressor default) | `bool success`, `string message`, `string graph_json`, `float32 node_retention`, `int32 bridge_nodes_kept` |
| `/nsk/merge` (`Merge`) | `int32 agent_id`, `int32 sender_id`, `string graph_json` | `bool success`, `string message`, `float64[] z_star`, `float64 gate`, `float64 z_norm` |
| `/nsk/similarity_query` (`SimilarityQuery`) | `int32[] agent_ids` | `bool success`, `string message`, `string matrix_json`, `float64 mean_sim` |

`graph_json` encodes `{x, edge_index, edge_type, num_nodes, agent_id}`; `graph_serialiser.py` provides the round-trip (`graph_to_dict`/`dict_to_graph`), which is covered by tests in both directions.

**Why services and not topics:** compress/merge/similarity are inherently request–response interactions, and the prototype implemented them as ZMQ REQ/REP — a pattern whose lockstep failure mode (one lost message wedges the socket) forced manual reconnect logic everywhere. DDS services give delivery, discovery, and error propagation for free, and the migration deleted every line of that reconnect code.

---

## 5. The engine as a lifecycle node

The engine's startup is dominated by a ~12-second model load, and in the prototype this interval was invisible: the process existed, the services did not, and every client blind-retried. The engine is therefore a `rclpy.lifecycle.LifecycleNode`, whose state machine makes readiness explicit and queryable:

- `__init__` — parameter declarations only; nothing expensive, nothing that can fail.
- `on_configure` — resolve paths, load the checkpoint, construct the `AgentManager` (all agents initialised); returns `FAILURE` on any exception, which launch surfaces immediately instead of hanging.
- `on_activate` — create the three service servers; ends with the `Ready.` log line.
- `on_deactivate` — destroy the services. `on_cleanup` drops the manager reference, so a cleanup→configure cycle could reload a new checkpoint without killing the process.

The launch file auto-drives the transitions (`EmitEvent(ChangeState → configure)` at startup; an `OnStateTransition` handler emits `activate` on reaching `inactive`), so `ros2 launch` remains a one-command start. Clients required no change: services simply do not exist until activation, and the existing wait-for-service loops now wait for something semantically meaningful.

**The `_services` shadowing incident (regression-guarded).** During this conversion the engine kept a private bookkeeping list named `self._services`, which silently clobbered rclpy `Node`'s *internal service registry* of the same name. The consequences were subtle and instructive: the six parameter services lost their only Python reference and were garbage-collected off the DDS graph, while the five lifecycle services — C-level entities owned by the rcl state machine — remained *visible* on the graph but were never dispatched to Python, because the executor discovers a node's services exclusively through that list. The node therefore looked alive (it spun; Ctrl+C worked) while answering nothing. The fix is a rename to `_nsk_services`; the lesson — a subclass attribute can shadow base-class internals with no warning — is encoded as a permanent regression test that asserts the parameter services survive `__init__`.

---

## 6. Client nodes

**robot_node** (×5). Each robot runs a simple two-state behaviour (FLOCK toward peers in range, EXPLORE otherwise), publishes `cmd_vel`, tracks its own and peers' odometry, and executes the sharing cycle described in §2. The compress retention ratio is selected by `retention_for_similarity` (bands at similarity 0.4 and 0.7 → retention 0.65 / 0.4 / 0.2 — the intuition being that a very different peer deserves a richer graph). Note the ledger item in §11: the similarity input to this function is currently never written, so the live system always compresses at 0.65.

**convergence_monitor.** On a 10-second timer it queries the full similarity matrix and evaluates convergence with a baseline-relative criterion, extracted into the pure function `convergence_step()` for testability. Convergence requires all of: at least one merge event observed, a rise of at least `min_rise` (0.03) over the *first* reading, and `|Δ|` < `stability_eps` (0.005) for three consecutive cycles — and monitoring continues after convergence is declared. This replaced an absolute threshold (0.25) that failed in both directions across runs, because the baseline similarity varies per launch (the agents' ego-graphs are sampled differently each time; observed baselines ranged 0.14–0.35), so a fixed bar can be met at t=0 with zero merges or never met at all. The principle: measure change from where the run started, not distance to a constant. The monitor also counts merge events by listening to `/kg_share` and gating on pairwise range, and it warns every cycle if it detects more than one `/nsk/similarity_query` server on the graph (see the duplicate-engine incident, §11).

**Concurrency model (both clients).** Service calls are made from inside timer and subscription callbacks, which under a default single-threaded executor is a guaranteed deadlock: the callback blocks awaiting a response that only the (occupied) executor thread can deliver. The shipped pattern is `call_async()` plus a bounded wait on the future, with the service clients and all engine-touching callbacks in a single `ReentrantCallbackGroup`, spun by a `MultiThreadedExecutor` — the reentrant group is what makes the blocking wait safe, since another thread can always deliver the response. The wait itself (`_call_engine`) is a monotonic-deadline loop in ≤0.2 s slices that aborts silently when `rclpy.ok()` goes false, so a Ctrl+C during in-flight calls tears down in under two seconds instead of serving out a full timeout; the timeout is the ROS parameter `service_timeout_sec` (5.0 s default). Cancelled or failed futures have their exception retrieved on every exit path, which suppresses rclpy's "exception was never retrieved" noise at shutdown.

**Shutdown discipline (all three mains).** Under `ros2 launch`, the SIGINT handler may shut the rclpy context down at any moment — including between an `if rclpy.ok():` check and the `rclpy.shutdown()` call it guards. The prototype's check-then-act guard therefore failed probabilistically (some runs clean, some crashed). The shipped form catches instead of checking: `destroy_node()` and `shutdown()` are each wrapped in `try/except Exception: pass`, which is deliberate — this is final cleanup of a dying process, and a failed double-shutdown is a no-op. The acceptance test for this fix was three consecutive clean launch/interrupt cycles, because probabilistic bugs demand repeated trials.

---

## 7. QoS design

Every publisher and subscription declares an explicit profile; no bare depth integers remain. Explicit QoS documents each channel's delivery assumptions and protects against the classic silent failure: a BEST_EFFORT publisher and a RELIABLE subscriber simply never connect, with no error anywhere. (The compatible mixed pairing — RELIABLE publisher, BEST_EFFORT subscriber — is exactly what the odom channels use against the gz bridge.)

| Channel | Profile | Reasoning |
|---|---|---|
| `/kg_share` (pub + subs) | RELIABLE · VOLATILE · KEEP_LAST 10 | A lost broadcast is a lost merge, i.e. lost knowledge; the rate is low (~1 msg / 8 s / robot) so reliability costs nothing. VOLATILE because merging a stale graph on late join would be semantically wrong. |
| odom subscriptions | `qos_profile_sensor_data` (BEST_EFFORT · VOLATILE · depth 5) | Latest-pose-only semantics — a stale position is worse than a skipped one; matches sensor convention and the bridge pairing above. |
| `cmd_vel`, monitor publishers | RELIABLE · VOLATILE · KEEP_LAST 10 | Command/report streams where each message matters; low rate. |

Because QoS breakage manifests as silence rather than errors, the acceptance test for any QoS change is behavioural: the system must run identically, verified live alongside `ros2 topic info --verbose` inspection of the negotiated profiles.

---

## 8. Parameters, launch, and environment

**Parameters** (declared per node; `config/nsk_swarm_params.yaml` documents them, the launch file is authoritative for the robot count):

| Parameter | Node(s) | Default | Meaning |
|---|---|---|---|
| `num_robots` | all | 5 (launch-file variable — single source of truth) | swarm size; must match the engine's agent count |
| `comm_range` | robots, monitor | 3.0 m | 2D Euclidean broadcast/merge gating |
| `service_timeout_sec` | robots, monitor | 5.0 | engine call timeout |
| `min_rise` / `stability_eps` | monitor | 0.03 / 0.005 | convergence criterion (§6) |
| `config_path`, `checkpoint_path`, `dataset_indices`, `nsk_base_path` | engine | checkpoint `experiments/checkpoints/joint_best.pt` under `nsk_base_path` (default `/home/lawlite/Desktop/NSK`) | model loading; relative checkpoint resolves against the base |
| `venv_site_packages` | launch argument | `<repo>/venv/lib/python3.12/site-packages` | see environment note below |

The `num_robots` = 5 single-sourcing corrects a prototype defect where the count was declared in three places and disagreed (8 vs 5), which produced a `KeyError: 5` in every similarity query — the monitor asked for agents the engine never created.

**Environment.** The models require torch/PyG from a project venv created with `--system-site-packages` (so rclpy from `/opt/ros` and torch from pip share one interpreter), while colcon-generated entry points carry a `#!/usr/bin/python3` shebang that bypasses any activated venv. The launch file therefore prepends the venv's site-packages to `PYTHONPATH` via `SetEnvironmentVariable` (overridable through the `venv_site_packages` launch argument), and hence `ros2 launch nsk_swarm swarm_sim.launch.py` works from any correctly-sourced terminal with no manual exports. Manual `ros2 run` of the engine still requires the export. Standard sourcing order per terminal: `/opt/ros/jazzy/setup.bash`, then the workspace `install/setup.bash`.

**Operational habit:** run `pgrep -af "nsk_engine|robot_node|convergence"` before every launch. Stray processes are state — see §11.

---

## 9. Testing

The suite (55 tests, ~6–8 s, `ros2_ws/src/nsk_swarm/test/`) is designed around one constraint: it must run with **no Gazebo, no display, and no checkpoint**, because CI has none of them. Three layers:

1. **Pure logic** — retention bands (boundary-probed around 0.4/0.7 with epsilon cases), graph serialiser round-trips (tensor equality both directions, JSON-safety), the `convergence_step` decision table, proximity geometry.
2. **Node behaviour** — engine lifecycle transitions with a mocked `AgentManager` (configure success/FAILURE, services appear on activate and vanish on deactivate), service callback contracts (well-formed `success=True` fields; `success=False` + message on manager exceptions; JSON fields parse), and the `_services`-shadowing regression guard.
3. **DDS integration** — real client↔engine round-trips over the wire in-process, including the timeout path and the `success=False` path, under a 20 s budget.

Entry points: `python3 -m pytest test/ -v` (venv on `PYTHONPATH` for torch) or `colcon test` from the workspace. One packaging landmine is recorded here because it silently defeats the whole suite: modern setuptools (≥72) drops the `tests_require` kwarg, upon which colcon falls back to the unittest runner and reports success while running **zero** tests. The supported spelling is `extras_require={'test': ['pytest']}`, which is what `setup.py` uses. Deliberate scope exclusions: Gazebo physics, robot motion, and convergence *dynamics* are not CI-tested — they remain manual/experimental, as the cost of simulating physics in CI exceeds its verification value at this stage. Ament linters are likewise deliberately omitted.

---

## 10. CI and delivery

`.github/workflows/ci.yml`: on push/PR to `main`, a single job in the `ros:jazzy` container installs pip (absent from the image) with PEP 668 handling (`--break-system-packages`), installs torch-CPU + PyG (pip download cache keyed on the workflow file; ~1.5–2 min warm), then `colcon build` and `colcon test` with **`colcon test-result --verbose` as the pass/fail gate** — `colcon test` itself exits 0 on test failures. Container steps are not login shells, so every ROS step sources its environment explicitly. Verified in production: the first runs completed green in ~1.5 min with the log showing `Summary: 55 tests, 0 errors, 0 failures, 0 skipped`. In CI the venv split does not exist — torch installs into the container's system Python, the same interpreter ROS uses — so the `PYTHONPATH` machinery of §8 is a local-machine concern only.

---

## 11. Known limitations and open ledger

Recorded honestly, with severity. None of these corrupts committed results; all are tracked rather than hidden.

**Research-logic defects (fix before any adaptive-compression claims):**
- *Dead retention adaptivity.* `retention_for_similarity` exists and is tested, but its input (`peer_similarity`) is declared and never written in `robot_node.py`, so every live compress runs at retention 0.65 (`min_sim=0.000` in every log line). Discovered while writing the honest README — reading one's own system to describe it truthfully is itself a debugging technique.
- *Trained merger bypassed.* Live merges are the embedding-level blend (§2) with a constant gate of 0.5; the trained gated merger is loaded but not exercised, due to its graph encoder being OOD on compressed graphs.

**Instrumentation gaps (prerequisites for EXP-01):**
- No per-cycle CSV export from the monitor (results currently require log parsing).
- No seed parameter — agent ego-graph sampling differs per launch, so runs are not reproducible (this is also *why* the convergence criterion had to be baseline-relative).

**Behavioural observations (research questions, not bugs):**
- *Pair-cluster fragmentation* (the headline finding): in the archived 5-minute run, mean pairwise similarity rose 0.174 → 0.510 (t≈130 s) and then declined monotonically to 0.281, as the swarm froze into two static merge pairs (robots 1↔4 at 0.40 m, 0↔2 at 2.41 m) with robot 3 isolated — within-pair consensus, between-cluster drift. Consensus theory says global agreement requires the union of communication graphs over time to be connected; ours disconnected. EXP-01 (`experiments/designs/`) is the designed experiment separating the topology hypothesis from the swarm-size hypothesis, with the mixing intervention (forced periodic EXPLORE at fixed N) as the discriminating test.
- FLOCK appears to drive pairs into stable parking equilibria (frozen inter-robot distances) — a behaviour-side question independent of NSK.

**Cosmetic (deferred by explicit decision — they get no cheaper by being fixed early):**
- `Failed to publish log to rosout` line at teardown; Gazebo SDF `ambient` warning.

**Incident record (both fixed, both regression-relevant):**
- *Duplicate-engine state divergence.* Two engine processes once served the same `/nsk/*` names simultaneously; DDS load-balanced requests between them, so merges persisted in one engine's agents while the monitor read the other's — a frozen monitor with no error anywhere. Root cause of the monitor guard and the pre-launch `pgrep` habit.
- *`Node._services` shadowing* — §5.

---

## 12. Design-decision log (summary)

| Decision | Alternative rejected | Reason |
|---|---|---|
| DDS services for compress/merge/similarity | keep ZMQ REQ/REP | request–response semantics; delete lockstep failure mode and reconnect code |
| JSON strings for tensors in .srv | fully typed nested messages | variable shapes; typing adds rigidity, buys little |
| Interfaces in separate ament_cmake package | mixed package | rosidl codegen; standard professional layout |
| Lifecycle node for the engine | plain node + retries | 12 s load made explicit, queryable, and launch-sequenced |
| `call_async` + reentrant group + MT executor | done-callback chaining | preserves linear control flow; reentrancy makes the bounded wait deadlock-free |
| Catch-don't-check shutdown | `if rclpy.ok()` guard | check-then-act race under launch's SIGINT; proven by 3× repeated trials |
| Baseline-relative convergence | absolute threshold | per-run baseline variance (0.14–0.35); absolute bars false-fire or never fire |
| Explicit QoS everywhere | defaults | documents assumptions; guards the silent-incompatibility failure |
| CPU-only torch | CUDA build | engine runs `device='cpu'`; saves ~2.5 GB; GTX 1050 Ti reserved for training work |
| Linters omitted, physics untested | full ament lint / sim-in-CI | cost exceeds verification value; stated openly rather than half-configured |

---

## 13. Operations quick reference

```bash
# Build
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Run everything (after: source install/setup.bash)
pgrep -af "nsk_engine|robot_node|convergence" || ros2 launch nsk_swarm swarm_sim.launch.py

# Inspect
ros2 lifecycle get /nsk_engine                      # expect: active [3]
ros2 service call /nsk/similarity_query nsk_swarm_interfaces/srv/SimilarityQuery "{agent_ids: [0, 1]}"
ros2 topic info /kg_share --verbose                 # negotiated QoS

# Test
python3 -m pytest ros2_ws/src/nsk_swarm/test/ -v    # venv site-packages on PYTHONPATH
cd ros2_ws && colcon test --packages-select nsk_swarm && colcon test-result --verbose
```

Without the external checkpoint (`joint_best.pt`, not included in the repo): the full test suite and the build run; a live `on_configure` returns FAILURE and the robots wait on `/nsk/*` indefinitely.

---

*Phase A closed at commit `0f055dc`. Next: EXP-01 instrumentation (seed, CSV export, retention fix) on the research track, or Phase B (multi-robot SLAM/Nav2 with NSK as the bandwidth-aware knowledge filter) on the portfolio track.*
