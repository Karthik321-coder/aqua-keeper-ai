"""
src/control/controller.py
Trajectory prediction and PID-based blocker position control.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Trajectory prediction
# ---------------------------------------------------------------------------


class TrajectoryPredictor:
    """
    Fits a linear (or optional parabolic) trajectory to recent ball observations
    in 3-D goal coordinates and estimates the intercept point on the goal plane
    (z = 0).

    Observations are (x, y, z, t) tuples where:
      x — horizontal position (metres from left post)
      y — vertical position (metres from ground)
      z — depth from camera / distance to goal plane (metres)
      t — monotonic timestamp (seconds)
    """

    def __init__(
        self,
        min_observations: int = 3,
        use_gravity: bool = True,
        g: float = 9.81,
    ) -> None:
        self.min_observations = min_observations
        self.use_gravity = use_gravity
        self.g = g
        self._obs: list[tuple[float, float, float, float]] = []

    def reset(self) -> None:
        self._obs.clear()

    def add_observation(self, x: float, y: float, z: float, t: Optional[float] = None) -> None:
        if t is None:
            t = time.monotonic()
        self._obs.append((x, y, z, t))
        # Keep only the last 10 observations
        if len(self._obs) > 10:
            self._obs.pop(0)

    def predict_intercept(self) -> Optional[tuple[float, float, float]]:
        """
        Returns (x_intercept, y_intercept, t_impact) or None if insufficient data.

        t_impact is the estimated time (seconds) until the ball reaches z=0.
        """
        if len(self._obs) < self.min_observations:
            return None

        xs = np.array([o[0] for o in self._obs])
        ys = np.array([o[1] for o in self._obs])
        zs = np.array([o[2] for o in self._obs])
        ts = np.array([o[3] for o in self._obs])
        # Normalise time
        t0 = ts[0]
        ts_norm = ts - t0

        # Fit linear models for x(t) and z(t)
        vx = float(np.polyfit(ts_norm, xs, 1)[0])
        vz = float(np.polyfit(ts_norm, zs, 1)[0])

        x0 = float(xs[-1])
        z0 = float(zs[-1])
        t_now = ts[-1] - t0

        if abs(vz) < 1e-6:
            return None  # ball not moving towards goal

        t_impact = -z0 / vz  # time from now until z=0
        if t_impact < 0:
            return None  # ball moving away from goal

        x_intercept = x0 + vx * t_impact

        # Vertical prediction
        if self.use_gravity:
            vy0, y0_fit = np.polyfit(ts_norm, ys, 1)
            y0 = float(y0_fit + vy0 * t_now)
            vy = float(vy0)
            y_intercept = y0 + vy * t_impact - 0.5 * self.g * t_impact**2
        else:
            vy = float(np.polyfit(ts_norm, ys, 1)[0])
            y0 = float(ys[-1])
            y_intercept = y0 + vy * t_impact

        return x_intercept, y_intercept, t_impact

    @property
    def n_observations(self) -> int:
        return len(self._obs)


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------


class PIDController:
    """
    Discrete-time PID position controller for the blocker.

    All positions are in metres (goal frame x-axis, 0 … goal_width).
    """

    def __init__(
        self,
        kp: float = 5.0,
        ki: float = 0.1,
        kd: float = 0.05,
        min_output: float = 0.0,
        max_output: float = 1.0,
        max_integral: float = 0.3,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.max_integral = max_integral

        self._integral: float = 0.0
        self._prev_error: Optional[float] = None
        self._prev_time: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def compute(
        self,
        setpoint: float,
        measurement: float,
        t: Optional[float] = None,
    ) -> float:
        """
        Compute the PID output.

        Parameters
        ----------
        setpoint    : desired blocker position (metres)
        measurement : current blocker position (metres)
        t           : current time (monotonic seconds); uses time.monotonic() if None

        Returns
        -------
        output : commanded position (metres), clamped to [min_output, max_output]
        """
        if t is None:
            t = time.monotonic()

        error = setpoint - measurement

        dt = (t - self._prev_time) if self._prev_time is not None else 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        if dt > 0:
            self._integral += error * dt
        self._integral = float(np.clip(self._integral, -self.max_integral, self.max_integral))
        i_term = self.ki * self._integral

        # Derivative (on measurement to avoid derivative kick)
        d_term = 0.0
        if self._prev_error is not None and dt > 0:
            d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = t

        raw_output = measurement + p_term + i_term + d_term
        return float(np.clip(raw_output, self.min_output, self.max_output))


# ---------------------------------------------------------------------------
# High-level blocker controller
# ---------------------------------------------------------------------------


class BlockerController:
    """
    Combines the trajectory predictor and PID to produce a target position
    command for the blocker actuator.

    Falls back to goal-centre when no prediction is available.
    """

    def __init__(
        self,
        goal_width: float = 2.4,
        goal_height: float = 0.9,
        pid_kp: float = 5.0,
        pid_ki: float = 0.1,
        pid_kd: float = 0.05,
        min_observations: int = 3,
    ) -> None:
        self.goal_width = goal_width
        self.goal_height = goal_height
        self._predictor = TrajectoryPredictor(min_observations=min_observations)
        self._pid = PIDController(
            kp=pid_kp,
            ki=pid_ki,
            kd=pid_kd,
            min_output=0.0,
            max_output=goal_width,
            max_integral=goal_width * 0.15,
        )
        self._last_setpoint: float = goal_width / 2.0

    def update(
        self,
        ball_x: float,
        ball_y: float,
        ball_z: float,
        blocker_pos: float,
        t: Optional[float] = None,
    ) -> tuple[float, Optional[float]]:
        """
        Feed a new ball observation and return a blocker position command.

        Parameters
        ----------
        ball_x, ball_y, ball_z : ball position in goal frame (metres)
        blocker_pos            : current blocker position (metres)
        t                      : timestamp (uses time.monotonic() if None)

        Returns
        -------
        (target_pos, t_impact)
          target_pos : commanded blocker position (metres)
          t_impact   : estimated seconds until ball reaches goal (or None)
        """
        if t is None:
            t = time.monotonic()

        self._predictor.add_observation(ball_x, ball_y, ball_z, t)

        prediction = self._predictor.predict_intercept()
        if prediction is not None:
            x_intercept, _y_intercept, t_impact = prediction
            # Clamp to goal width
            x_intercept = float(np.clip(x_intercept, 0.0, self.goal_width))
            self._last_setpoint = x_intercept
        else:
            # Fall back to last known setpoint (or centre on first frame)
            t_impact = None

        target_pos = self._pid.compute(self._last_setpoint, blocker_pos, t)
        return target_pos, prediction[2] if prediction is not None else None

    def reset(self) -> None:
        self._predictor.reset()
        self._pid.reset()
        self._last_setpoint = self.goal_width / 2.0
