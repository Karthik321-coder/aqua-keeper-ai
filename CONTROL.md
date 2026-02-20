# Control System

## Blocker Kinematics

The blocker is a carriage sliding along a horizontal rail driven by a GT2 belt.

```
        [Servo/Stepper] ──── [Pulley] ────── GT2 belt ────── [Idler]
                                                  │
                                          [Blocker carriage]
                                         x=0 ──────────── x=W
```

- **W** = goal width (metres), configurable.
- **x** = blocker centre position from left post (0 … W).
- **pulley_radius** = 0.012 m (20T GT2 pulley, 2 mm pitch).
- **steps_per_rev** = 200 (1.8° stepper) × microstep (default ×16 = 3 200).
- **metres_per_step** = (2π × r) / steps_per_rev_microstepped.

For a servo: position is commanded as a PWM pulse width (1000–2000 µs) mapped linearly to 0 … W.

---

## Servo / Stepper Sizing

### Minimum Speed Requirement

A ball travelling at **10 m/s** from 2 m away gives ~200 ms to intercept.
The blocker must traverse worst-case W/2 = 0.5 m in < 150 ms (leaving 50 ms for latency).

```
v_min = 0.5 m / 0.15 s ≈ 3.3 m/s
```

### Servo Selection

| Parameter | Value |
|---|---|
| Required speed | 3.3 m/s belt linear |
| Belt circumference (20T, 2mm) | 40 mm |
| Required shaft rpm | 3.3 / 0.04 × 60 ≈ 4 950 rpm |
| Hitec HS-7980TH no-load speed | ~720° / 0.14 s → ~5 142 rpm equiv. |

The HS-7980TH meets the requirement. Alternatively use a high-KV brushless motor with ESC.

### Stepper Selection

NEMA-17 at 1/16 microstepping and 24 V supply can achieve ~4 000 steps/s (limited by back-EMF). Check that `motor.max_speed_steps_per_s` in `default.yaml` does not exceed the driver capability.

---

## PID Tuning Guide

The controller uses a **position PID** loop where the setpoint is the predicted intercept x-coordinate.

```
error(t) = x_intercept - x_blocker(t)
u(t) = Kp·e + Ki·∫e dt + Kd·ė
```

### Initial Gains (start here)

| Gain | Initial Value | Notes |
|------|--------------|-------|
| Kp | 5.0 | Increase until oscillation, then halve |
| Ki | 0.1 | Integrator for steady-state error |
| Kd | 0.05 | Derivative damps overshoot |

### Tuning Procedure

1. Set Ki=0, Kd=0. Increase Kp until the blocker oscillates around the setpoint.
2. Halve Kp. This is your P-only baseline.
3. Add Kd gradually to reduce overshoot.
4. Add Ki to eliminate residual steady-state error.
5. Verify with `scripts/test_loop.py --scenario sine`.

### Anti-windup

The integrator is clamped to `±motor.max_integral` (default ±0.3 m) to prevent wind-up when the blocker is at a position limit.

---

## Motion Planning — Time-to-Impact Estimation

`controller.py` uses the last N ball positions (default N=5) to fit a 2D trajectory.

### Linear Model (fast balls, < 0.2 s to impact)

```
x(t) = x0 + vx·t
z(t) = z0 + vz·t   (depth towards goal)
t_impact = -z0 / vz   (when z=0, ball at goal line)
x_intercept = x0 + vx·t_impact
```

### Parabolic Model (lobbed shots, > 0.5 s to impact)

```
x(t) = x0 + vx·t
y(t) = y0 + vy·t - 0.5·g·t²
z(t) = z0 + vz·t
t_impact = (-vz - sqrt(vz²- 2·g·z0)) / (-g)   [if gravity component in z]
```

Switch between models based on `|vz|`: use linear for `|vz| > threshold`, parabolic otherwise.

---

## Saturation and Safety Limits

All limits are enforced in `actuator.py` before any signal reaches the motor driver.

| Limit | Default | Config key |
|-------|---------|------------|
| Min position | 0.0 m | `motor.min_pos` |
| Max position | goal_width m | `motor.max_pos` |
| Max velocity | 4.0 m/s | `motor.max_velocity` |
| Max acceleration | 20 m/s² | `motor.max_acceleration` |
| Max PID integral | 0.3 m | `motor.max_integral` |
| E-stop GPIO | 26 | `motor.estop_pin` |

Position commands outside `[min_pos, max_pos]` are clipped. Velocity commands exceeding `max_velocity` are scaled down. A rate limiter enforces `max_acceleration` between consecutive commands.
