# NSK Swarm Robotics 2D Simulation
## ROS 2 Jazzy + Gazebo Harmonic

A 2D multi-robot simulation where **5 differential-drive robots** navigate a
flat Gazebo Harmonic world using random-walk motion. Each robot shares
compressed FB15k-237 knowledge graphs over ZMQ. The NSK pipeline fuses
received graphs into each robot's z* embedding, and convergence is
monitored in real time via RViz2.

---

## Architecture Overview

```
┌─────────────────────────────────┐      ZMQ ipc://       ┌──────────────────────┐
│  ROS 2 Node  (system Python)    │ ◄──────────────────► │  NSK Engine (nsk_env) │
│                                 │                        │                       │
│  /robot_N/odom  → (x, y, yaw)  │  compress_request     │  SwarmAgent           │
│  /robot_N/cmd_vel ← Twist       │  ──────────────────►  │  GraphCompressor      │
│  /kg_share  ↔ String JSON       │  compressed_graph     │  KnowledgeMerger      │
│  /nsk/similarity_markers ←      │  ◄──────────────────  │  joint_best.pt        │
└─────────────────────────────────┘                        └──────────────────────┘
```

**Key design decisions:**
- All NSK/PyTorch inference in `nsk_env` (separate process) — never in ROS 2 nodes
- Embedding-level sharing: re-embed received G̃ with trained embedder
- Weighted merge: `z* = L2-norm(0.7 × z_self + 0.3 × z_received)`
- VelocityControl + OdometryPublisher plugins (no wheel joints needed for cylinders)
- Communication range: 3.0 m (2D Euclidean)

---

## Prerequisites

```bash
# ROS 2 Jazzy (system-wide)
source /opt/ros/jazzy/setup.bash

# Gazebo Harmonic
sudo apt install gz-harmonic

# ros_gz_bridge
sudo apt install ros-jazzy-ros-gz-bridge

# Python ZMQ (system Python for ROS 2 nodes)
sudo apt install python3-zmq

# nsk_env already has: torch, torch_geometric, zmq, yaml
```

---

## One-Time Setup

```bash
# Copy this folder into your NSK workspace
cp -r NSKsim/ros2_ws ~/Desktop/NSK/

# Build the ROS 2 package
cd ~/Desktop/NSK/ros2_ws
colcon build --packages-select nsk_swarm
source install/setup.bash
```

---

## Running the Simulation

Open **three terminals**.

### Terminal 1 — Gazebo + ROS 2 nodes

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/NSK/ros2_ws/install/setup.bash
ros2 launch nsk_swarm swarm_sim.launch.py
```

This starts (with staggered delays):
1. Gazebo Harmonic with `knowledge_world.sdf`
2. `ros_gz_bridge` (10 topics: 5 robots × cmd_vel + odom)
3. 5 `NSKRobotNode` instances (t=5 s)
4. `ConvergenceMonitorNode` (t=6 s)
5. RViz2 with top-down view (t=7 s)

### Terminal 2 — NSK Engine (nsk_env)

```bash
cd ~/Desktop/NSK
source nsk_env/bin/activate
CUDA_VISIBLE_DEVICES="" python -m nsk_swarm.nsk_engine.engine_server \
    --num_robots 5 \
    --endpoint ipc:///tmp/nsk_engine_0 \
    --config configs/base.yaml \
    --checkpoint experiments/checkpoints/joint_best.pt
```

Start this **before** or within a few seconds of Terminal 1. The robot nodes
will retry ZMQ connections automatically.

### Terminal 3 — Monitor (optional, already started by launch file)

```bash
ros2 topic echo /nsk/convergence
```

---

## Quick Sanity Check (Before Launching Gazebo)

Run the integration test to verify the engine works end-to-end:

```bash
cd ~/Desktop/NSK
source nsk_env/bin/activate
CUDA_VISIBLE_DEVICES="" python ros2_ws/test_integration.py
```

Expected output:
```
✓  All integration tests PASSED.
```

Expected values (from Phase 6 validation):
| Metric | Expected |
|--------|----------|
| `node_retention` per robot | 0.41–0.45 |
| `bridge_nodes_kept` | 16–107 |
| `z_norm` after every merge | exactly 1.0000 |
| No NaN in z* | always |
| Baseline mean pairwise sim | ~0.399 |

---

## Convergence Experiment Protocol

1. Robots start at pentagon spawn points and begin 2D random walk
2. `t=0`: baseline similarity logged (~0.40)
3. Every 8 s per robot: compress local graph → broadcast if peers in range
4. Every 10 s: monitor queries engine, logs mean pairwise similarity
5. **Stop condition:** mean sim > 0.25 for 3 consecutive readings, OR t > 600 s
6. Final report printed automatically with:
   - Total merge events per robot pair
   - Final mean pairwise similarity
   - Whether convergence threshold was reached and when

---

## RViz2 Visualisation

| Element | Description |
|---------|-------------|
| Arrows (R/G/B/O/P) | Robot positions from odometry |
| Coloured cylinders | z* convergence state (red→green) |
| Blue lines | Active communication links |
| Yellow HUD text | Current mean pairwise similarity |
| Per-robot labels | Robot ID + mean similarity |

**Fixed frame:** `world`  
**View:** Top-down orthographic, 25 m × 25 m window

---

## Package Structure

```
ros2_ws/
  src/nsk_swarm/
    nsk_swarm/
      __init__.py
      robot_node.py           # ROS 2 node (system Python, no PyTorch)
      convergence_monitor.py  # aggregates z* similarity, publishes markers
      graph_serialiser.py     # graph ↔ JSON (no PyTorch dependency)
    nsk_engine/
      __init__.py
      engine_server.py        # ZMQ REP server (runs in nsk_env)
      agent_manager.py        # manages 5 SwarmAgent instances
    launch/
      swarm_sim.launch.py     # launches everything
    worlds/
      knowledge_world.sdf     # flat 2D world, 5 inline cylinder robots
    config/
      nsk_swarm_params.yaml   # all parameters
    rviz/
      nsk_convergence.rviz    # top-down RViz2 layout
    package.xml
    setup.py
    CMakeLists.txt
  test_integration.py         # standalone ZMQ test
  README_GAZEBO.md
```

---

## Troubleshooting

### Robots not moving
```bash
# Check topics exist
ros2 topic list | grep cmd_vel

# Check odometry is publishing
ros2 topic echo /robot_0/odom

# Check Gazebo side
gz topic -l | grep robot

# Check bridge is running
ros2 node list | grep parameter_bridge
```

### `node_retention = 1.000` in integration test
The expanding semantic closure bug has been re-introduced. Check `compressor.py`:
```bash
grep -n "verify_semantic_closure" ~/Desktop/NSK/src/stage1_compressor/compressor.py
# Should return set(kept_nodes), NOT expand iteratively
```

### ZMQ timeout in robot nodes
```bash
# Verify engine is running
ps aux | grep engine_server

# Verify endpoint matches
# In params yaml: zmq_endpoint: "ipc:///tmp/nsk_engine_0"
# In engine startup: --endpoint ipc:///tmp/nsk_engine_0
```

### Mean similarity stays at ~−0.002
- Verify dataset indices are `[0, 900, 1800, 2700, 3600]` (widely spaced)
- Verify `CUDA_VISIBLE_DEVICES=""` is set for engine server
- Confirm `joint_best.pt` is loading (not `embedder_best.pt`)

### NaN in z*
- L2 normalisation is missing or the norm is zero
- Engine logs the z_norm; should always be 1.0000

### Gazebo starts but robots invisible
- Ensure SDF version is 1.9 (`<sdf version="1.9">`)
- Cylinder z=0.005 places it just above the ground plane (correct)

---

## Reference: Phase 6 Validated Numbers

| Metric | Expected value |
|--------|---------------|
| Baseline mean pairwise sim | ~0.399 |
| After 1 merge per agent | ~−0.079 (temporary — normal) |
| After 10+ sharing rounds | trending toward +0.24 |
| Gate value per merge | 0.50 (fixed for embedding-level merge) |
| z* norm after merge | exactly 1.0 |
| Node retention on real graphs | 41–45% |
| Bridge nodes kept | 16–107 |

---

## Hardware Notes

Target machine: Lenovo Y520, GTX 1050 4 GB, i5-7300HQ, 16 GB RAM, Ubuntu 24.04.

- GPU is used by Gazebo rendering — all NSK inference runs on CPU (`CUDA_VISIBLE_DEVICES=""`)
- 2D simulation is deliberately lightweight: no meshes, no sensors, no URDF
- Gazebo should start in under 5 seconds
- Full simulation runs comfortably on 2 CPU cores
