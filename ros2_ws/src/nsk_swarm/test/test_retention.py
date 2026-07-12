"""Layer 1 — retention_for_similarity thresholds (pure logic, no rclpy init).

Bands in robot_node.retention_for_similarity:
    sim > 0.7          -> 0.20  (already similar — compress hard)
    0.4 < sim <= 0.7   -> 0.40  (default)
    sim <= 0.4         -> 0.65  (very different — send rich graph)
Both comparisons are strict '>', so the boundary values 0.7 and 0.4 fall
into the band below them.
"""

import pytest

from nsk_swarm.robot_node import retention_for_similarity


@pytest.mark.parametrize('sim, expected', [
    # Boundary at 0.7: strict '>', so exactly 0.7 gets the middle band
    (0.7, 0.40),
    (0.7 + 1e-9, 0.20),
    (0.7 - 1e-9, 0.40),
    # Boundary at 0.4: strict '>', so exactly 0.4 gets the low band
    (0.4, 0.65),
    (0.4 + 1e-9, 0.40),
    (0.4 - 1e-9, 0.65),
    # Representative points in each band
    (1.0, 0.20),
    (0.9, 0.20),
    (0.55, 0.40),
    (0.5, 0.40),
    (0.2, 0.65),
    (0.0, 0.65),
    (-0.3, 0.65),   # cosine similarity can be negative
    (-1.0, 0.65),
])
def test_retention_bands(sim, expected):
    assert retention_for_similarity(sim) == expected
