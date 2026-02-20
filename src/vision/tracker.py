"""
src/vision/tracker.py
Advanced Kalman-filter-based ball tracker for robust multi-frame tracking.

Provides position, velocity, and acceleration estimates with uncertainty
bounds, enabling the controller to make confident intercept predictions
even through brief occlusions and noisy detections.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np


class KalmanBallTracker:
    """
    2-D (pixel-space) constant-acceleration Kalman filter for ball tracking.

    State vector (6×1):
        [x, y, vx, vy, ax, ay]

    Measurement vector (2×1):
        [x, y]  (detected bounding-box centre)

    The filter predicts between detections to provide smooth position
    estimates and velocity/acceleration for downstream trajectory
    prediction.

    Parameters
    ----------
    process_noise : float
        Scalar multiplier for process-noise covariance Q.
    measurement_noise : float
        Scalar multiplier for measurement-noise covariance R.
    max_coast_frames : int
        Number of consecutive frames the filter will coast (predict-only)
        before declaring the track lost.
    """

    STATE_DIM = 6
    MEAS_DIM = 2

    def __init__(
        self,
        process_noise: float = 1.0,
        measurement_noise: float = 10.0,
        max_coast_frames: int = 10,
    ) -> None:
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.max_coast_frames = max_coast_frames

        # State: [x, y, vx, vy, ax, ay]
        self._x = np.zeros((self.STATE_DIM, 1), dtype=np.float64)
        self._P = np.eye(self.STATE_DIM, dtype=np.float64) * 500.0

        # Measurement matrix: we observe [x, y] only
        self._H = np.zeros((self.MEAS_DIM, self.STATE_DIM), dtype=np.float64)
        self._H[0, 0] = 1.0
        self._H[1, 1] = 1.0

        # Measurement noise
        self._R = np.eye(self.MEAS_DIM, dtype=np.float64) * measurement_noise

        self._initialised = False
        self._coast_count = 0
        self._last_time: Optional[float] = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition_matrix(self, dt: float) -> np.ndarray:
        """Build the state-transition matrix F for a given time step."""
        F = np.eye(self.STATE_DIM, dtype=np.float64)
        F[0, 2] = dt
        F[0, 4] = 0.5 * dt * dt
        F[1, 3] = dt
        F[1, 5] = 0.5 * dt * dt
        F[2, 4] = dt
        F[3, 5] = dt
        return F

    def _process_noise_matrix(self, dt: float) -> np.ndarray:
        """Discrete white-noise acceleration model."""
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        q = self.process_noise
        Q = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=np.float64)
        # Position block
        Q[0, 0] = dt4 / 4.0
        Q[0, 2] = dt3 / 2.0
        Q[0, 4] = dt2 / 2.0
        Q[1, 1] = dt4 / 4.0
        Q[1, 3] = dt3 / 2.0
        Q[1, 5] = dt2 / 2.0
        # Velocity block
        Q[2, 0] = dt3 / 2.0
        Q[2, 2] = dt2
        Q[2, 4] = dt
        Q[3, 1] = dt3 / 2.0
        Q[3, 3] = dt2
        Q[3, 5] = dt
        # Acceleration block
        Q[4, 0] = dt2 / 2.0
        Q[4, 2] = dt
        Q[4, 4] = 1.0
        Q[5, 1] = dt2 / 2.0
        Q[5, 3] = dt
        Q[5, 5] = 1.0
        return Q * q

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(self, t: Optional[float] = None) -> np.ndarray:
        """
        Predict the state forward to time *t*.

        Returns the predicted state (6×1) **without** consuming a measurement.
        Call this once per frame before :meth:`update`.
        """
        if t is None:
            t = time.monotonic()
        if self._last_time is None:
            self._last_time = t
        dt = max(t - self._last_time, 1e-6)
        self._last_time = t

        F = self._transition_matrix(dt)
        Q = self._process_noise_matrix(dt)

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        self._coast_count += 1
        return self._x.copy()

    def update(self, cx: float, cy: float, t: Optional[float] = None) -> np.ndarray:
        """
        Incorporate a new measurement (detected ball centre).

        If the filter has not been initialised, this also sets the initial
        state.

        Returns the updated state (6×1).
        """
        if t is None:
            t = time.monotonic()

        if not self._initialised:
            self._x[0, 0] = cx
            self._x[1, 0] = cy
            self._last_time = t
            self._initialised = True
            self._coast_count = 0
            return self._x.copy()

        # Predict to current time first
        self.predict(t)

        z = np.array([[cx], [cy]], dtype=np.float64)
        y = z - self._H @ self._x                    # innovation
        S = self._H @ self._P @ self._H.T + self._R  # innovation covariance
        K = self._P @ self._H.T @ np.linalg.inv(S)   # Kalman gain

        self._x = self._x + K @ y
        I = np.eye(self.STATE_DIM, dtype=np.float64)
        self._P = (I - K @ self._H) @ self._P

        self._coast_count = 0
        return self._x.copy()

    def reset(self) -> None:
        """Reset the filter to an uninitialised state."""
        self._x = np.zeros((self.STATE_DIM, 1), dtype=np.float64)
        self._P = np.eye(self.STATE_DIM, dtype=np.float64) * 500.0
        self._initialised = False
        self._coast_count = 0
        self._last_time = None

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def position(self) -> tuple[float, float]:
        """Current estimated position (x, y)."""
        return float(self._x[0, 0]), float(self._x[1, 0])

    @property
    def velocity(self) -> tuple[float, float]:
        """Current estimated velocity (vx, vy)."""
        return float(self._x[2, 0]), float(self._x[3, 0])

    @property
    def acceleration(self) -> tuple[float, float]:
        """Current estimated acceleration (ax, ay)."""
        return float(self._x[4, 0]), float(self._x[5, 0])

    @property
    def is_tracking(self) -> bool:
        """True if the filter is initialised and not coasting beyond limit."""
        return self._initialised and self._coast_count <= self.max_coast_frames

    @property
    def coast_count(self) -> int:
        """Number of consecutive frames without a measurement."""
        return self._coast_count

    @property
    def state(self) -> np.ndarray:
        """Full state vector (6×1) copy."""
        return self._x.copy()
