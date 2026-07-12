"""Shared pytest fixtures for the nsk_swarm test suite.

Run from the package root:
    cd ros2_ws/src/nsk_swarm && python3 -m pytest test/ -v
with the ROS 2 Jazzy + workspace environments sourced (for rclpy and
nsk_swarm_interfaces) and the project venv's site-packages on PYTHONPATH
(for torch / torch_geometric).
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
