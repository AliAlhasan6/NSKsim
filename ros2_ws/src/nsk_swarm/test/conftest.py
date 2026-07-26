"""Shared pytest fixtures for the nsk_swarm test suite.

Run from the package root:
    cd ros2_ws/src/nsk_swarm && python3 -m pytest test/ -v
with the ROS 2 Jazzy + workspace environments sourced (for rclpy and
nsk_swarm_interfaces) and the project venv's site-packages on PYTHONPATH
(for torch / torch_geometric).

Missing torch: skipped locally, fatal in CI
-------------------------------------------
torch and torch_geometric live ONLY in the project venv, so a bare
`python3 -m pytest` under system Python cannot import them. That used to take
down far more than the tests that needed them: a module-level `import torch` in
one file is a COLLECTION error, and pytest aborts the whole session on one
(exit code 2, "Interrupted: 1 error during collection"). The other 140-odd
tests — none of which touch torch — never ran, and reported nothing.

So the two torch-dependent modules (test_engine_lifecycle.py,
test_graph_serialiser.py) skip themselves at module level when torch is absent.
That is right for a local shell without the venv and WRONG for CI, where torch
is installed deliberately and a missing one means the install broke: skipping
there would shrink the suite silently and still report green. Hence the
NSK_REQUIRE_TORCH escape hatch — set to 1 by .github/workflows/ci.yml, it turns
the skip back into the ImportError that fails the job.
"""

import os
import sys

# Isolate test DDS traffic from any concurrently running sim on this machine
# (a live /nsk/compress server in the same domain would corrupt the
# integration tests). An explicitly exported ROS_DOMAIN_ID is respected.
# Must happen before rclpy.init() reads the environment.
os.environ.setdefault('ROS_DOMAIN_ID', '77')

# Guarantee the source tree (not the colcon-installed copy on PYTHONPATH)
# is what the tests import, regardless of how pytest was invoked.
PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

import pytest
import rclpy


@pytest.fixture(scope='session')
def rclpy_context():
    """Session-scoped rclpy init/shutdown.

    Session scope: one DDS participant context per pytest process. Repeated
    init/shutdown cycles per module pay discovery cost each time and are a
    known source of teardown races in DDS middlewares; nothing in this suite
    needs a fresh context. Layer-1 (pure logic) tests simply never request
    this fixture, so they run without any rclpy state.
    """
    rclpy.init()
    yield
    rclpy.shutdown()
