"""
tests/integration/test_pipeline_integration.py
Integration tests for the vision → controller → actuator pipeline.

These tests exercise the data flow between subsystems using mock
backends and synthetic data — no camera or model weights required.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.control.actuator import Actuator
from src.control.controller import BlockerController
from src.control.coverage import CoverageStrategy
from src.vision.calibration import GoalMapper
from src.vision.tracker import KalmanBallTracker


GOAL_WIDTH = 2.4
GOAL_HEIGHT = 0.9


# ------------------------------------------------------------------ #
# Vision → Controller integration
# ------------------------------------------------------------------ #


class TestVisionToController:
    """Mock detections fed through the controller produce valid commands."""

    def _make_controller(self) -> BlockerController:
        return BlockerController(
            goal_width=GOAL_WIDTH,
            goal_height=GOAL_HEIGHT,
            pid_kp=5.0,
            pid_ki=0.1,
            pid_kd=0.05,
            min_observations=3,
        )

    def test_mock_detections_produce_valid_intercept(self):
        ctrl = self._make_controller()
        # Simulate ball approaching goal: constant x, decreasing z
        for i in range(10):
            target, t_impact = ctrl.update(
                ball_x=1.0,
                ball_y=0.5,
                ball_z=3.0 - i * 0.3,
                blocker_pos=GOAL_WIDTH / 2.0,
                t=float(i) * 0.033,
            )
            assert 0.0 <= target <= GOAL_WIDTH

        # After enough observations, t_impact should be available
        assert t_impact is not None
        assert t_impact >= 0.0

    def test_controller_handles_noisy_detections(self):
        ctrl = self._make_controller()
        rng = np.random.default_rng(42)
        for i in range(20):
            noise = rng.normal(0, 0.05)
            target, _ = ctrl.update(
                ball_x=1.2 + noise,
                ball_y=0.45,
                ball_z=3.0 - i * 0.15,
                blocker_pos=GOAL_WIDTH / 2.0,
                t=float(i) * 0.033,
            )
            assert 0.0 <= target <= GOAL_WIDTH


# ------------------------------------------------------------------ #
# Controller → Actuator integration
# ------------------------------------------------------------------ #


class TestControllerToActuator:
    """Controller commands forwarded to actuator respect all safety limits."""

    def test_commands_stay_within_limits(self):
        ctrl = BlockerController(goal_width=GOAL_WIDTH)
        act = Actuator(
            backend="mock",
            goal_width=GOAL_WIDTH,
            max_velocity=4.0,
            max_acceleration=20.0,
        )

        for i in range(30):
            t = float(i) * 0.033
            ball_z = 3.0 - i * 0.1
            if ball_z < 0:
                ball_z = 0.01
            blocker_pos = act.get_position()
            target, _ = ctrl.update(0.3, 0.5, ball_z, blocker_pos, t)
            actual = act.move_to(target, t)
            assert 0.0 <= actual <= GOAL_WIDTH

    def test_out_of_range_commands_clipped(self):
        act = Actuator(
            backend="mock",
            goal_width=GOAL_WIDTH,
            min_pos=0.0,
            max_pos=GOAL_WIDTH,
        )
        # Direct extreme commands should be clipped
        pos = act.move_to(-10.0, t=0.0)
        assert pos >= 0.0
        pos = act.move_to(100.0, t=100.0)
        assert pos <= GOAL_WIDTH


# ------------------------------------------------------------------ #
# Full loop — synthetic frames through all stages
# ------------------------------------------------------------------ #


class TestFullSyntheticLoop:
    """End-to-end loop with synthetic ball trajectory; no exceptions."""

    def test_100_iterations_no_exceptions(self):
        ctrl = BlockerController(goal_width=GOAL_WIDTH)
        act = Actuator(backend="mock", goal_width=GOAL_WIDTH)

        for i in range(100):
            t = float(i) * 0.033
            z = max(3.0 - i * 0.03, 0.01)
            blocker_pos = act.get_position()
            target, _ = ctrl.update(1.2, 0.5, z, blocker_pos, t)
            actual = act.move_to(target, t)
            assert 0.0 <= actual <= GOAL_WIDTH

    def test_tracker_feeds_controller(self):
        """Kalman tracker output → GoalMapper → Controller → Actuator."""
        tracker = KalmanBallTracker(process_noise=1.0, measurement_noise=5.0)
        ctrl = BlockerController(goal_width=GOAL_WIDTH)
        act = Actuator(backend="mock", goal_width=GOAL_WIDTH)

        # Fake a simple homography (identity-ish for 640×360 → goal)
        image_corners = np.array(
            [[0, 360], [640, 360], [640, 0], [0, 0]], dtype=np.float32
        )
        mapper = GoalMapper.from_correspondences(image_corners, GOAL_WIDTH, GOAL_HEIGHT)

        for i in range(50):
            t = float(i) * 0.033
            # Simulate a ball at pixel (300, 200) drifting right
            px = 300.0 + i * 2.0
            py = 200.0
            tracker.update(px, py, t)

            if tracker.is_tracking:
                est_x, est_y = tracker.position
                gx, gy = mapper.pixel_to_goal(est_x, est_y)
                blocker_pos = act.get_position()
                target, _ = ctrl.update(gx, gy, 1.0, blocker_pos, t)
                actual = act.move_to(target, t)
                assert 0.0 <= actual <= GOAL_WIDTH

    def test_coverage_strategy_idle_position(self):
        """CoverageStrategy provides valid idle positions."""
        cov = CoverageStrategy(
            goal_width=GOAL_WIDTH,
            goal_height=GOAL_HEIGHT,
            zone_cols=5,
            zone_rows=3,
        )
        idle = cov.idle_position
        assert 0.0 <= idle <= GOAL_WIDTH
        # With prediction
        rec = cov.recommend_position(predicted_x=0.5)
        assert rec == pytest.approx(0.5)
        # Without prediction
        rec = cov.recommend_position(predicted_x=None)
        assert rec == idle
