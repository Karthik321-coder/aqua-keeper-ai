"""
tests/unit/test_benchmark.py
Unit tests for the latency profiler.
"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.utils.benchmark import LatencyProfiler


class TestLatencyProfiler:
    def test_measure_records_time(self):
        prof = LatencyProfiler()
        with prof.measure("test_stage"):
            time.sleep(0.01)  # ~10 ms
        stats = prof.stage_stats("test_stage")
        assert stats["count"] == 1
        assert stats["mean"] > 0.0

    def test_empty_stage_stats(self):
        prof = LatencyProfiler()
        stats = prof.stage_stats("nonexistent")
        assert stats["count"] == 0

    def test_multiple_stages(self):
        prof = LatencyProfiler()
        with prof.measure("a"):
            pass
        with prof.measure("b"):
            pass
        summary = prof.summary()
        assert "a" in summary
        assert "b" in summary
        assert "_loop" in summary

    def test_tick_records_loop_time(self):
        prof = LatencyProfiler()
        prof.tick()
        time.sleep(0.01)
        prof.tick()
        stats = prof.loop_stats()
        assert stats["count"] == 1
        assert stats["mean"] > 0.0

    def test_reset_clears_all(self):
        prof = LatencyProfiler()
        with prof.measure("x"):
            pass
        prof.tick()
        prof.reset()
        assert prof.stage_stats("x")["count"] == 0
        assert prof.loop_stats()["count"] == 0

    def test_window_size_limits_data(self):
        prof = LatencyProfiler(window_size=5)
        for _ in range(10):
            with prof.measure("s"):
                pass
        stats = prof.stage_stats("s")
        assert stats["count"] == 5

    def test_summary_includes_all_stages(self):
        prof = LatencyProfiler()
        for name in ["capture", "inference", "control"]:
            with prof.measure(name):
                pass
        summary = prof.summary()
        assert "capture" in summary
        assert "inference" in summary
        assert "control" in summary
