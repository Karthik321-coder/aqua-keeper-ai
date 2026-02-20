"""
tests/unit/test_coverage.py
Unit tests for the multi-zone coverage strategy.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.control.coverage import Zone, build_zone_grid, CoverageStrategy


GOAL_WIDTH = 2.4
GOAL_HEIGHT = 0.9


class TestZone:
    def test_centre(self):
        z = Zone(name="test", x_min=0.0, x_max=1.0, y_min=0.0, y_max=0.5)
        assert z.centre_x == pytest.approx(0.5)
        assert z.centre_y == pytest.approx(0.25)

    def test_dimensions(self):
        z = Zone(name="test", x_min=0.0, x_max=1.2, y_min=0.0, y_max=0.3)
        assert z.width == pytest.approx(1.2)
        assert z.height == pytest.approx(0.3)

    def test_contains(self):
        z = Zone(name="test", x_min=0.0, x_max=1.0, y_min=0.0, y_max=1.0)
        assert z.contains(0.5, 0.5)
        assert z.contains(0.0, 0.0)  # boundary inclusive
        assert not z.contains(1.1, 0.5)
        assert not z.contains(0.5, -0.1)


class TestBuildZoneGrid:
    def test_correct_count(self):
        zones = build_zone_grid(GOAL_WIDTH, GOAL_HEIGHT, cols=5, rows=3)
        assert len(zones) == 15

    def test_zones_cover_full_goal(self):
        zones = build_zone_grid(GOAL_WIDTH, GOAL_HEIGHT, cols=4, rows=2)
        # Every point in the goal should belong to at least one zone
        for x in [0.0, 0.6, 1.2, 1.8, 2.4]:
            for y in [0.0, 0.45, 0.9]:
                found = any(z.contains(x, y) for z in zones)
                assert found, f"({x}, {y}) not covered"

    def test_invalid_cols_raises(self):
        with pytest.raises(ValueError):
            build_zone_grid(GOAL_WIDTH, GOAL_HEIGHT, cols=0, rows=1)

    def test_single_zone(self):
        zones = build_zone_grid(GOAL_WIDTH, GOAL_HEIGHT, cols=1, rows=1)
        assert len(zones) == 1
        assert zones[0].width == pytest.approx(GOAL_WIDTH)
        assert zones[0].height == pytest.approx(GOAL_HEIGHT)


class TestCoverageStrategy:
    def test_idle_position_within_goal(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH, goal_height=GOAL_HEIGHT)
        assert 0.0 <= cov.idle_position <= GOAL_WIDTH

    def test_idle_position_is_centre_for_symmetric_grid(self):
        cov = CoverageStrategy(
            goal_width=GOAL_WIDTH,
            goal_height=GOAL_HEIGHT,
            zone_cols=5,
            zone_rows=3,
        )
        # For a symmetric grid, idle should be near goal centre
        assert cov.idle_position == pytest.approx(GOAL_WIDTH / 2.0, abs=0.01)

    def test_recommend_position_with_prediction(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH)
        pos = cov.recommend_position(predicted_x=0.8)
        assert pos == pytest.approx(0.8)

    def test_recommend_position_clamps(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH)
        assert cov.recommend_position(predicted_x=-1.0) == pytest.approx(0.0)
        assert cov.recommend_position(predicted_x=10.0) == pytest.approx(GOAL_WIDTH)

    def test_recommend_position_without_prediction(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH)
        assert cov.recommend_position(predicted_x=None) == cov.idle_position

    def test_worst_case_reach_time(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH, blocker_speed=4.0)
        t = cov.worst_case_reach_time(GOAL_WIDTH / 2.0)
        # From centre, max distance to any zone centre should be < goal_width/2
        assert t > 0.0
        assert t < GOAL_WIDTH / 4.0  # roughly

    def test_coverage_report_structure(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH, zone_cols=3, zone_rows=2)
        report = cov.coverage_report(GOAL_WIDTH / 2.0)
        assert len(report) == 6
        for entry in report:
            assert "zone" in entry
            assert "reach_time_s" in entry
            assert "reachable_200ms" in entry

    def test_zone_for_point(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH, goal_height=GOAL_HEIGHT)
        z = cov.zone_for_point(1.2, 0.45)
        assert z is not None
        assert z.contains(1.2, 0.45)

    def test_zone_for_point_outside_returns_none(self):
        cov = CoverageStrategy(goal_width=GOAL_WIDTH, goal_height=GOAL_HEIGHT)
        z = cov.zone_for_point(-1.0, 0.5)
        assert z is None
