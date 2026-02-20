"""
src/control/coverage.py
Multi-zone coverage strategy for full-goal protection.

Divides the goal mouth into configurable zones and computes optimal
blocker positioning to minimise worst-case intercept time, taking into
account the blocker's physical speed limitations.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Zone:
    """A rectangular region of the goal face."""

    name: str
    x_min: float  # metres from left post
    x_max: float
    y_min: float  # metres from ground
    y_max: float

    @property
    def centre_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def centre_y(self) -> float:
        return (self.y_min + self.y_max) / 2.0

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def build_zone_grid(
    goal_width: float,
    goal_height: float,
    cols: int = 5,
    rows: int = 3,
) -> list[Zone]:
    """
    Partition the goal face into a grid of *cols* × *rows* zones.

    Returns a flat list of :class:`Zone` objects ordered left-to-right,
    bottom-to-top.
    """
    if cols < 1 or rows < 1:
        raise ValueError("cols and rows must be >= 1")

    dw = goal_width / cols
    dh = goal_height / rows
    zones: list[Zone] = []
    for r in range(rows):
        for c in range(cols):
            name = f"R{r}C{c}"
            zones.append(
                Zone(
                    name=name,
                    x_min=round(c * dw, 6),
                    x_max=round((c + 1) * dw, 6),
                    y_min=round(r * dh, 6),
                    y_max=round((r + 1) * dh, 6),
                )
            )
    return zones


class CoverageStrategy:
    """
    Computes an optimal default blocker position that minimises the
    worst-case travel distance to any zone centre.

    When a ball trajectory prediction is available, the strategy defers
    to the prediction.  When idle (no prediction), it places the blocker
    at the minimax-optimal position.

    Parameters
    ----------
    goal_width : float
        Goal width in metres.
    goal_height : float
        Goal height in metres.
    zone_cols : int
        Number of horizontal zones.
    zone_rows : int
        Number of vertical zones.
    blocker_speed : float
        Maximum blocker speed in m/s.
    """

    def __init__(
        self,
        goal_width: float = 2.4,
        goal_height: float = 0.9,
        zone_cols: int = 5,
        zone_rows: int = 3,
        blocker_speed: float = 4.0,
    ) -> None:
        self.goal_width = goal_width
        self.goal_height = goal_height
        self.blocker_speed = blocker_speed
        self.zones = build_zone_grid(goal_width, goal_height, zone_cols, zone_rows)
        self._idle_pos = self._compute_minimax_position()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _compute_minimax_position(self) -> float:
        """
        Find the x position that minimises the maximum distance to any
        zone centre (1-D minimax on x-axis).
        """
        centres = [z.centre_x for z in self.zones]
        x_min_c = min(centres)
        x_max_c = max(centres)
        # Minimax solution is the midpoint of the range
        return (x_min_c + x_max_c) / 2.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def idle_position(self) -> float:
        """Optimal x-position when no ball is being tracked."""
        return self._idle_pos

    def worst_case_reach_time(self, from_x: float) -> float:
        """
        Maximum time (seconds) to reach any zone centre from *from_x*.
        """
        max_dist = max(abs(z.centre_x - from_x) for z in self.zones)
        if self.blocker_speed <= 0:
            return float("inf")
        return max_dist / self.blocker_speed

    def zone_for_point(self, x: float, y: float) -> Optional[Zone]:
        """Return the zone containing (x, y), or None if outside the goal."""
        for z in self.zones:
            if z.contains(x, y):
                return z
        return None

    def recommend_position(
        self,
        predicted_x: Optional[float] = None,
    ) -> float:
        """
        Return the recommended blocker x-position.

        If *predicted_x* is given (from trajectory prediction), clamp it to
        [0, goal_width] and return it.  Otherwise, return the idle minimax
        position.
        """
        if predicted_x is not None:
            return float(np.clip(predicted_x, 0.0, self.goal_width))
        return self._idle_pos

    def coverage_report(self, from_x: float) -> list[dict]:
        """
        Return per-zone reachability information from blocker position *from_x*.
        """
        report = []
        for z in self.zones:
            dist = abs(z.centre_x - from_x)
            t = dist / self.blocker_speed if self.blocker_speed > 0 else float("inf")
            report.append(
                {
                    "zone": z.name,
                    "centre_x": z.centre_x,
                    "centre_y": z.centre_y,
                    "distance_m": round(dist, 4),
                    "reach_time_s": round(t, 4),
                    "reachable_200ms": t <= 0.200,
                }
            )
        return report
