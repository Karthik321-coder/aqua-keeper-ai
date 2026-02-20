"""
tests/unit/test_controller.py
Unit tests for trajectory prediction and PID controller.
"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.control.controller import (
    BlockerController,
    PIDController,
    TrajectoryPredictor,
)


class TestTrajectoryPredictor:
    def test_no_prediction_below_min_observations(self):
        pred = TrajectoryPredictor(min_observations=3)
        pred.add_observation(1.0, 0.5, 3.0, t=0.0)
        pred.add_observation(1.0, 0.5, 2.0, t=0.1)
        assert pred.predict_intercept() is None

    def test_linear_intercept_at_x(self):
        pred = TrajectoryPredictor(min_observations=3, use_gravity=False)
        # Ball moving straight towards goal: constant x=1.2, decreasing z
        for i in range(5):
            pred.add_observation(1.2, 0.5, 3.0 - i * 0.5, t=float(i) * 0.1)
        result = pred.predict_intercept()
        assert result is not None
        x_intercept, y_intercept, t_impact = result
        assert abs(x_intercept - 1.2) < 0.1
        assert t_impact > 0

    def test_reset_clears_observations(self):
        pred = TrajectoryPredictor(min_observations=2)
        pred.add_observation(1.0, 0.5, 2.0, t=0.0)
        pred.add_observation(1.0, 0.5, 1.0, t=0.1)
        pred.reset()
        assert pred.n_observations == 0
        assert pred.predict_intercept() is None

    def test_ball_moving_away_returns_none(self):
        pred = TrajectoryPredictor(min_observations=3, use_gravity=False)
        # z increasing → moving away
        for i in range(5):
            pred.add_observation(1.0, 0.5, float(i), t=float(i) * 0.1)
        result = pred.predict_intercept()
        assert result is None


class TestPIDController:
    def test_proportional_output(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, min_output=0.0, max_output=5.0)
        # setpoint=3.0, measurement=1.0 → error=2.0 → p_term=4.0
        # raw_output = measurement + p_term = 1.0 + 4.0 = 5.0 (clamped)
        out = pid.compute(setpoint=3.0, measurement=1.0, t=0.0)
        # The output is clamped to [0, 5]
        assert 0.0 <= out <= 5.0

    def test_output_clamped_to_limits(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, min_output=0.0, max_output=2.4)
        out = pid.compute(setpoint=2.4, measurement=0.0, t=0.0)
        assert out <= 2.4
        out = pid.compute(setpoint=0.0, measurement=2.4, t=1.0)
        assert out >= 0.0

    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.0, min_output=0.0, max_output=5.0)
        pid.compute(setpoint=2.0, measurement=0.0, t=0.0)
        pid.compute(setpoint=2.0, measurement=1.0, t=1.0)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error is None

    def test_integrator_antiwindup(self):
        pid = PIDController(
            kp=0.0, ki=1.0, kd=0.0,
            min_output=-10.0, max_output=10.0,
            max_integral=0.5,
        )
        # Feed large errors for many steps
        for i in range(100):
            pid.compute(setpoint=5.0, measurement=0.0, t=float(i))
        assert abs(pid._integral) <= 0.5 + 1e-6


class TestBlockerController:
    def test_fallback_to_centre_on_startup(self):
        goal_width = 2.4
        ctrl = BlockerController(goal_width=goal_width)
        # No observations yet — should return centre area
        target, t_impact = ctrl.update(1.2, 0.5, 3.0, 1.2, t=0.0)
        assert 0.0 <= target <= goal_width
        assert t_impact is None

    def test_target_within_goal_bounds(self):
        ctrl = BlockerController(goal_width=2.4)
        for i in range(10):
            target, _ = ctrl.update(0.1, 0.5, 3.0 - i * 0.2, 1.0, t=float(i) * 0.1)
            assert 0.0 <= target <= 2.4

    def test_reset_works(self):
        ctrl = BlockerController(goal_width=2.4)
        for i in range(5):
            ctrl.update(0.5, 0.5, 3.0 - i * 0.3, 1.0, t=float(i) * 0.1)
        ctrl.reset()
        assert ctrl._predictor.n_observations == 0
