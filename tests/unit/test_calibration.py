"""
tests/unit/test_calibration.py
Unit tests for GoalMapper and calibration utilities.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.vision.calibration import GoalMapper, save_goal_mapper, load_goal_mapper


GOAL_WIDTH = 2.4
GOAL_HEIGHT = 0.9

# Image corners: bottom-left, bottom-right, top-right, top-left
# (in a 640×360 image the goal fills the frame)
_IMAGE_CORNERS = np.array(
    [[0, 360], [640, 360], [640, 0], [0, 0]], dtype=np.float32
)


@pytest.fixture
def mapper():
    return GoalMapper.from_correspondences(_IMAGE_CORNERS, GOAL_WIDTH, GOAL_HEIGHT)


class TestGoalMapper:
    def test_bottom_left_maps_to_origin(self, mapper):
        x, y = mapper.pixel_to_goal(0.0, 360.0)
        assert x == pytest.approx(0.0, abs=0.01)
        assert y == pytest.approx(0.0, abs=0.01)

    def test_bottom_right_maps_to_right_post(self, mapper):
        x, y = mapper.pixel_to_goal(640.0, 360.0)
        assert x == pytest.approx(GOAL_WIDTH, abs=0.01)
        assert y == pytest.approx(0.0, abs=0.01)

    def test_top_left_maps_to_top_left(self, mapper):
        x, y = mapper.pixel_to_goal(0.0, 0.0)
        assert x == pytest.approx(0.0, abs=0.01)
        assert y == pytest.approx(GOAL_HEIGHT, abs=0.01)

    def test_centre_pixel_maps_to_goal_centre(self, mapper):
        x, y = mapper.pixel_to_goal(320.0, 180.0)
        assert x == pytest.approx(GOAL_WIDTH / 2, abs=0.05)
        assert y == pytest.approx(GOAL_HEIGHT / 2, abs=0.05)

    def test_roundtrip_goal_to_pixel_to_goal(self, mapper):
        for gx, gy in [(0.5, 0.3), (1.2, 0.45), (2.0, 0.8)]:
            u, v = mapper.goal_to_pixel(gx, gy)
            x, y = mapper.pixel_to_goal(u, v)
            assert x == pytest.approx(gx, abs=0.01)
            assert y == pytest.approx(gy, abs=0.01)

    def test_serialise_deserialise(self, mapper, tmp_path):
        path = str(tmp_path / "hom.yaml")
        save_goal_mapper(path, mapper)
        loaded = load_goal_mapper(path)
        # Check a mapping is preserved
        x1, y1 = mapper.pixel_to_goal(320.0, 180.0)
        x2, y2 = loaded.pixel_to_goal(320.0, 180.0)
        assert x1 == pytest.approx(x2, abs=1e-4)
        assert y1 == pytest.approx(y2, abs=1e-4)

    def test_goal_dimensions_preserved(self, mapper):
        assert mapper.goal_width == GOAL_WIDTH
        assert mapper.goal_height == GOAL_HEIGHT


class TestGoalMapperFromDict:
    def test_from_dict_roundtrip(self, mapper):
        d = mapper.to_dict()
        restored = GoalMapper.from_dict(d)
        x1, y1 = mapper.pixel_to_goal(100.0, 200.0)
        x2, y2 = restored.pixel_to_goal(100.0, 200.0)
        assert x1 == pytest.approx(x2, abs=1e-4)
        assert y1 == pytest.approx(y2, abs=1e-4)
