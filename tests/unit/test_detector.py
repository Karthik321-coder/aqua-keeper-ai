"""
tests/unit/test_detector.py
Unit tests for BallDetector helper classes (no model weights required).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.vision.detector import Detection, ExponentialSmoother, BallDetector


class TestDetection:
    def test_radius(self):
        d = Detection(cx=100.0, cy=200.0, w=20.0, h=30.0, confidence=0.9, timestamp=0.0)
        assert d.radius() == pytest.approx(12.5)

    def test_attributes(self):
        d = Detection(cx=1.0, cy=2.0, w=10.0, h=10.0, confidence=0.85, timestamp=1.5)
        assert d.cx == 1.0
        assert d.cy == 2.0
        assert d.confidence == 0.85
        assert d.timestamp == 1.5


class TestExponentialSmoother:
    def test_first_update_equals_measurement(self):
        s = ExponentialSmoother(alpha=0.5)
        val = s.update(10.0)
        assert val == pytest.approx(10.0)

    def test_smoothing_moves_toward_measurement(self):
        s = ExponentialSmoother(alpha=0.5)
        s.update(0.0)
        val = s.update(10.0)
        # 0.5 * 10 + 0.5 * 0 = 5.0
        assert val == pytest.approx(5.0)

    def test_alpha_one_tracks_exactly(self):
        s = ExponentialSmoother(alpha=1.0)
        s.update(5.0)
        val = s.update(99.0)
        assert val == pytest.approx(99.0)

    def test_reset_clears_value(self):
        s = ExponentialSmoother(alpha=0.5)
        s.update(10.0)
        s.reset()
        assert s.value is None

    def test_invalid_alpha_raises(self):
        with pytest.raises(ValueError):
            ExponentialSmoother(alpha=0.0)
        with pytest.raises(ValueError):
            ExponentialSmoother(alpha=1.5)

    def test_multiple_updates_converge(self):
        s = ExponentialSmoother(alpha=0.8)
        target = 100.0
        val = 0.0
        for _ in range(20):
            val = s.update(target)
        assert val == pytest.approx(target, abs=1.0)


class TestBallDetectorHelpers:
    def test_history_bounded_by_maxlen(self):
        det = BallDetector(history_len=5)
        # Manually inject detections into history
        for i in range(10):
            d = Detection(float(i), float(i), 10.0, 10.0, 0.9, float(i))
            det.history.append(d)
        assert len(det.get_history()) == 5

    def test_get_history_returns_list(self):
        det = BallDetector()
        assert isinstance(det.get_history(), list)

    def test_is_loaded_false_before_load(self):
        det = BallDetector()
        assert not det.is_loaded()

    def test_detect_raises_before_load(self):
        det = BallDetector()
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        with pytest.raises(RuntimeError, match="not loaded"):
            det.detect(dummy)
