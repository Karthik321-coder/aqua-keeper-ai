"""
src/utils/benchmark.py
Performance benchmarking utilities for latency profiling.

Provides a lightweight profiler that tracks per-stage latencies
(capture, inference, control, actuation) and reports summary statistics
suitable for optimising the real-time pipeline.
"""

from __future__ import annotations

import statistics
import time
from collections import defaultdict
from typing import Optional


class LatencyProfiler:
    """
    Accumulates per-stage timing measurements and produces summary
    statistics.

    Usage::

        prof = LatencyProfiler()
        with prof.measure("inference"):
            result = model.predict(frame)
        with prof.measure("control"):
            cmd = controller.update(...)
        prof.tick()          # mark end of one pipeline iteration
        print(prof.summary())

    Parameters
    ----------
    window_size : int
        Number of most-recent measurements to keep per stage.
    """

    def __init__(self, window_size: int = 500) -> None:
        self._window_size = window_size
        self._stages: dict[str, list[float]] = defaultdict(list)
        self._loop_times: list[float] = []
        self._tick_start: Optional[float] = None

    class _Timer:
        """Context manager that records elapsed time into a list."""

        def __init__(self, store: list[float], max_len: int) -> None:
            self._store = store
            self._max_len = max_len
            self._start: float = 0.0

        def __enter__(self) -> "_Timer":
            self._start = time.monotonic()
            return self

        def __exit__(self, *_: object) -> None:
            elapsed_ms = (time.monotonic() - self._start) * 1000.0
            self._store.append(elapsed_ms)
            if len(self._store) > self._max_len:
                self._store.pop(0)

    def measure(self, stage: str) -> _Timer:
        """Return a context manager that times *stage* in milliseconds."""
        return self._Timer(self._stages[stage], self._window_size)

    def tick(self) -> None:
        """Mark the end of one complete loop iteration."""
        now = time.monotonic()
        if self._tick_start is not None:
            dt_ms = (now - self._tick_start) * 1000.0
            self._loop_times.append(dt_ms)
            if len(self._loop_times) > self._window_size:
                self._loop_times.pop(0)
        self._tick_start = now

    def stage_stats(self, stage: str) -> dict[str, float]:
        """Return mean / p50 / p95 / p99 / max for a single stage."""
        data = self._stages.get(stage, [])
        if not data:
            return {"count": 0, "mean": 0.0, "p50": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
        sorted_data = sorted(data)
        n = len(sorted_data)
        return {
            "count": n,
            "mean": round(statistics.mean(sorted_data), 3),
            "p50": round(sorted_data[int(n * 0.50)], 3),
            "p95": round(sorted_data[min(int(n * 0.95), n - 1)], 3),
            "p99": round(sorted_data[min(int(n * 0.99), n - 1)], 3),
            "max": round(sorted_data[-1], 3),
        }

    def loop_stats(self) -> dict[str, float]:
        """Return summary statistics for full-loop iteration times."""
        return self._compute_stats(self._loop_times)

    def summary(self) -> dict[str, dict[str, float]]:
        """Return a dict of stats for every registered stage plus the loop."""
        result: dict[str, dict[str, float]] = {}
        for stage in sorted(self._stages):
            result[stage] = self.stage_stats(stage)
        result["_loop"] = self.loop_stats()
        return result

    def reset(self) -> None:
        """Clear all accumulated measurements."""
        self._stages.clear()
        self._loop_times.clear()
        self._tick_start = None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_stats(data: list[float]) -> dict[str, float]:
        if not data:
            return {"count": 0, "mean": 0.0, "p50": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
        sorted_data = sorted(data)
        n = len(sorted_data)
        return {
            "count": n,
            "mean": round(statistics.mean(sorted_data), 3),
            "p50": round(sorted_data[int(n * 0.50)], 3),
            "p95": round(sorted_data[min(int(n * 0.95), n - 1)], 3),
            "p99": round(sorted_data[min(int(n * 0.99), n - 1)], 3),
            "max": round(sorted_data[-1], 3),
        }
