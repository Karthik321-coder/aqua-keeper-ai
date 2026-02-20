"""
tests/unit/test_tracker.py
Unit tests for the Kalman ball tracker.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.vision.tracker import KalmanBallTracker


class TestKalmanBallTracker:
    def test_initial_state_not_tracking(self):
        tracker = KalmanBallTracker()
        assert not tracker.is_tracking

    def test_first_update_initialises(self):
        tracker = KalmanBallTracker()
        tracker.update(100.0, 200.0, t=0.0)
        assert tracker.is_tracking
        x, y = tracker.position
        assert x == pytest.approx(100.0)
        assert y == pytest.approx(200.0)

    def test_velocity_estimate_after_updates(self):
        tracker = KalmanBallTracker(process_noise=1.0, measurement_noise=1.0)
        # Ball moving right at 100 px/s
        for i in range(20):
            t = float(i) * 0.033
            tracker.update(100.0 + i * 3.3, 200.0, t)
        vx, vy = tracker.velocity
        assert vx > 0  # should detect rightward motion
        assert abs(vy) < abs(vx)  # minimal vertical motion

    def test_coast_count_increases_on_predict_only(self):
        tracker = KalmanBallTracker(max_coast_frames=5)
        tracker.update(100.0, 200.0, t=0.0)
        assert tracker.coast_count == 0
        tracker.predict(t=0.033)
        assert tracker.coast_count == 1
        tracker.predict(t=0.066)
        assert tracker.coast_count == 2

    def test_track_lost_after_max_coast(self):
        tracker = KalmanBallTracker(max_coast_frames=3)
        tracker.update(100.0, 200.0, t=0.0)
        for i in range(1, 5):
            tracker.predict(t=float(i) * 0.033)
        assert not tracker.is_tracking

    def test_update_resets_coast_count(self):
        tracker = KalmanBallTracker(max_coast_frames=10)
        tracker.update(100.0, 200.0, t=0.0)
        tracker.predict(t=0.033)
        tracker.predict(t=0.066)
        assert tracker.coast_count == 2
        tracker.update(105.0, 200.0, t=0.1)
        assert tracker.coast_count == 0

    def test_reset_clears_state(self):
        tracker = KalmanBallTracker()
        tracker.update(100.0, 200.0, t=0.0)
        tracker.reset()
        assert not tracker.is_tracking
        assert tracker.coast_count == 0

    def test_state_shape(self):
        tracker = KalmanBallTracker()
        tracker.update(100.0, 200.0, t=0.0)
        state = tracker.state
        assert state.shape == (6, 1)

    def test_stationary_ball_low_velocity(self):
        tracker = KalmanBallTracker(process_noise=0.1, measurement_noise=1.0)
        for i in range(30):
            tracker.update(320.0, 240.0, t=float(i) * 0.033)
        vx, vy = tracker.velocity
        assert abs(vx) < 5.0
        assert abs(vy) < 5.0
