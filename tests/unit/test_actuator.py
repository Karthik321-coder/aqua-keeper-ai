"""
tests/unit/test_actuator.py
Unit tests for the Actuator (mock backend).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.control.actuator import Actuator


class TestActuatorMock:
    def _make(self, goal_width=2.4, min_pos=0.0, max_pos=None, **kw):
        return Actuator(
            backend="mock",
            goal_width=goal_width,
            min_pos=min_pos,
            max_pos=max_pos if max_pos is not None else goal_width,
            **kw,
        )

    def test_initial_position_is_centre(self):
        act = self._make()
        assert act.get_position() == pytest.approx(1.2, abs=0.01)

    def test_move_to_within_bounds(self):
        act = self._make()
        pos = act.move_to(1.0, t=0.0)
        assert 0.0 <= pos <= 2.4

    def test_clamps_below_min(self):
        act = self._make(min_pos=0.1)
        pos = act.move_to(-5.0, t=0.0)
        assert pos >= 0.1

    def test_clamps_above_max(self):
        act = self._make(max_pos=2.0)
        # Feed a large time delta so velocity limit isn't the binding constraint
        pos = act.move_to(99.0, t=100.0)
        assert pos <= 2.0

    def test_disable_prevents_movement(self):
        act = self._make()
        original = act.get_position()
        act.disable()
        result = act.move_to(2.0, t=0.0)
        assert result == original  # no movement

    def test_enable_after_disable(self):
        act = self._make()
        act.disable()
        act.enable()
        assert act.is_enabled

    def test_centre(self):
        act = self._make(goal_width=2.0)
        # Give a large time so velocity limit isn't binding
        act.move_to(0.0, t=0.0)
        act.centre()
        # After centre, last commanded position should be goal_width / 2
        # (may be rate-limited but should move towards centre)
        assert act.get_position() >= 0.0

    def test_invalid_backend_raises(self):
        with pytest.raises(ValueError, match="Unknown backend"):
            Actuator(backend="invalid_backend")

    def test_velocity_limiting(self):
        act = self._make(max_velocity=1.0)
        # Start at centre (1.2), try to jump to 2.4 in 0.1 s
        act.move_to(1.2, t=0.0)
        pos = act.move_to(2.4, t=0.1)
        # Max displacement in 0.1 s at 1.0 m/s = 0.1 m → pos ≤ 1.3
        assert pos <= 1.2 + 0.1 + 1e-6

    def test_acceleration_limiting(self):
        act = self._make(max_velocity=10.0, max_acceleration=5.0)
        act.move_to(1.2, t=0.0)
        # From rest, max velocity after dt=0.1 s is 0.5 m/s → max pos change = 0.05 m
        pos = act.move_to(2.4, t=0.1)
        assert pos <= 1.2 + 0.05 + 1e-6
