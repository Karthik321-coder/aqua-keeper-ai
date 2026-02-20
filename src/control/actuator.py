"""
src/control/actuator.py
Motor interface abstraction with safety interlocks.

Supports two backends:
  - "servo"  : PCA9685 PWM hat via adafruit-circuitpython-pca9685 (I²C)
  - "stepper": DRV8825 via RPi.GPIO step/dir pulses
  - "mock"   : software-only for testing (default when no hardware is present)
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_SERVO_MIN_US = 1000   # µs pulse width → left limit
_SERVO_MAX_US = 2000   # µs pulse width → right limit
_PCA9685_FREQ = 50     # Hz


# ---------------------------------------------------------------------------
# Backend implementations
# ---------------------------------------------------------------------------


class _MockBackend:
    """Software-only motor backend for testing without hardware."""

    def __init__(self) -> None:
        self.position_m: float = 0.0          # current simulated position

    def send_position(self, position_m: float, goal_width: float) -> None:
        # Instant teleport in mock mode
        self.position_m = position_m

    def get_position(self) -> float:
        return self.position_m

    def disable(self) -> None:
        pass

    def enable(self) -> None:
        pass


class _ServoBackend:
    """PCA9685 servo driver (I²C)."""

    def __init__(self, channel: int = 0, i2c_address: int = 0x40) -> None:
        try:
            import board  # type: ignore
            import busio  # type: ignore
            from adafruit_pca9685 import PCA9685  # type: ignore
            from adafruit_motor import servo as adafruit_servo  # type: ignore

            i2c = busio.I2C(board.SCL, board.SDA)
            pca = PCA9685(i2c, address=i2c_address)
            pca.frequency = _PCA9685_FREQ
            self._servo = adafruit_servo.Servo(
                pca.channels[channel],
                min_pulse=_SERVO_MIN_US,
                max_pulse=_SERVO_MAX_US,
            )
        except (ImportError, Exception) as exc:
            raise RuntimeError(
                "PCA9685/adafruit libraries not available. "
                "Install: pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor"
            ) from exc

    def send_position(self, position_m: float, goal_width: float) -> None:
        # Map [0, goal_width] → [0°, 180°]
        angle = (position_m / goal_width) * 180.0
        angle = float(np.clip(angle, 0.0, 180.0))
        self._servo.angle = angle

    def get_position(self) -> float:
        # Servo position is open-loop; return last commanded
        return getattr(self, "_last_pos", 0.0)

    def disable(self) -> None:
        self._servo.angle = None  # release

    def enable(self) -> None:
        pass


class _StepperBackend:
    """DRV8825 stepper driver via GPIO step/dir pulses."""

    def __init__(
        self,
        step_pin: int = 17,
        dir_pin: int = 27,
        en_pin: int = 22,
        steps_per_metre: float = 3200.0,
    ) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore

            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(step_pin, GPIO.OUT)
            GPIO.setup(dir_pin, GPIO.OUT)
            GPIO.setup(en_pin, GPIO.OUT)
            GPIO.output(en_pin, GPIO.LOW)  # active-low enable
        except ImportError as exc:
            raise RuntimeError(
                "RPi.GPIO not available. Install: pip install RPi.GPIO"
            ) from exc

        self._step_pin = step_pin
        self._dir_pin = dir_pin
        self._en_pin = en_pin
        self._steps_per_metre = steps_per_metre
        self._current_steps: int = 0

    def send_position(self, position_m: float, goal_width: float) -> None:
        target_steps = int(round(position_m * self._steps_per_metre))
        delta = target_steps - self._current_steps
        if delta == 0:
            return
        direction = 1 if delta > 0 else 0
        self._GPIO.output(self._dir_pin, direction)
        step_delay = 0.0001  # 100 µs → ~5 000 steps/s
        for _ in range(abs(delta)):
            self._GPIO.output(self._step_pin, self._GPIO.HIGH)
            time.sleep(step_delay)
            self._GPIO.output(self._step_pin, self._GPIO.LOW)
            time.sleep(step_delay)
        self._current_steps = target_steps

    def get_position(self) -> float:
        return self._current_steps / self._steps_per_metre

    def disable(self) -> None:
        self._GPIO.output(self._en_pin, self._GPIO.HIGH)  # active-low

    def enable(self) -> None:
        self._GPIO.output(self._en_pin, self._GPIO.LOW)


# ---------------------------------------------------------------------------
# Public Actuator class
# ---------------------------------------------------------------------------


class Actuator:
    """
    High-level actuator interface with safety interlocks.

    Parameters
    ----------
    backend : "mock" | "servo" | "stepper"
    goal_width : float
        Physical goal width in metres (used for position clamping).
    min_pos, max_pos : float
        Software hard limits (metres). Defaults to [0, goal_width].
    max_velocity : float
        Maximum permitted velocity (m/s).
    max_acceleration : float
        Maximum permitted acceleration (m/s²).
    estop_pin : int or None
        BCM GPIO pin for emergency-stop button (active-low). None = disabled.
    **backend_kwargs
        Additional keyword arguments forwarded to the backend constructor.
    """

    def __init__(
        self,
        backend: str = "mock",
        goal_width: float = 2.4,
        min_pos: float = 0.0,
        max_pos: Optional[float] = None,
        max_velocity: float = 4.0,
        max_acceleration: float = 20.0,
        estop_pin: Optional[int] = None,
        **backend_kwargs,
    ) -> None:
        self.goal_width = goal_width
        self.min_pos = min_pos
        self.max_pos = max_pos if max_pos is not None else goal_width
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self._enabled: bool = True
        self._estopped: bool = False
        self._last_pos: float = goal_width / 2.0
        self._last_vel: float = 0.0
        self._last_cmd_time: Optional[float] = None

        # Initialise backend
        if backend == "mock":
            self._backend: _MockBackend | _ServoBackend | _StepperBackend = _MockBackend()
        elif backend == "servo":
            self._backend = _ServoBackend(**backend_kwargs)
        elif backend == "stepper":
            self._backend = _StepperBackend(**backend_kwargs)
        else:
            raise ValueError(f"Unknown backend: {backend!r}. Choose 'mock', 'servo', or 'stepper'.")

        # E-stop GPIO monitoring
        if estop_pin is not None:
            self._setup_estop(estop_pin)

    # ------------------------------------------------------------------
    # E-stop
    # ------------------------------------------------------------------

    def _setup_estop(self, pin: int) -> None:
        try:
            import RPi.GPIO as GPIO  # type: ignore

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                pin, GPIO.FALLING, callback=self._estop_callback, bouncetime=50
            )
            logger.info("E-stop monitoring enabled on GPIO %d", pin)
        except ImportError:
            logger.warning("RPi.GPIO not available; e-stop GPIO monitoring disabled.")

    def _estop_callback(self, channel: int) -> None:
        logger.critical("E-STOP triggered on GPIO %d! Disabling actuator.", channel)
        self._estopped = True
        self._enabled = False
        self._backend.disable()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def move_to(self, position: float, t: Optional[float] = None) -> float:
        """
        Command the blocker to a target position with rate limiting and safety checks.

        Parameters
        ----------
        position : float
            Target position in metres [0, goal_width].
        t : float, optional
            Current timestamp; uses time.monotonic() if None.

        Returns
        -------
        actual_pos : float
            The position after clamping (what was actually commanded).
        """
        if not self._enabled:
            logger.warning("Actuator disabled; ignoring move_to(%0.3f m)", position)
            return self._last_pos

        if t is None:
            t = time.monotonic()

        dt = (t - self._last_cmd_time) if self._last_cmd_time is not None else 0.0
        self._last_cmd_time = t

        # 1. Clamp to software position limits
        position = float(np.clip(position, self.min_pos, self.max_pos))

        # 2. Rate-limit velocity
        if dt > 0:
            requested_vel = (position - self._last_pos) / dt
            clamped_vel = float(np.clip(requested_vel, -self.max_velocity, self.max_velocity))
            # Rate-limit acceleration
            dv = clamped_vel - self._last_vel
            max_dv = self.max_acceleration * dt
            clamped_vel = self._last_vel + float(np.clip(dv, -max_dv, max_dv))
            position = self._last_pos + clamped_vel * dt
            position = float(np.clip(position, self.min_pos, self.max_pos))
            self._last_vel = clamped_vel
        else:
            self._last_vel = 0.0

        self._last_pos = position
        self._backend.send_position(position, self.goal_width)
        return position

    def get_position(self) -> float:
        """Return current (estimated) blocker position in metres."""
        return self._last_pos

    def centre(self) -> None:
        """Move blocker to goal centre."""
        self.move_to(self.goal_width / 2.0)

    def enable(self) -> None:
        """Re-enable the actuator (clears software disable, not e-stop)."""
        if self._estopped:
            logger.error("Cannot enable — hardware e-stop is active.")
            return
        self._enabled = True
        self._backend.enable()

    def disable(self) -> None:
        """Software-disable the actuator (does not clear e-stop)."""
        self._enabled = False
        self._backend.disable()

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    @property
    def is_estopped(self) -> bool:
        return self._estopped
