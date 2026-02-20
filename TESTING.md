# Testing Plan

## Test Levels

### 1. Unit Tests (`tests/unit/`)

Run with: `pytest tests/unit/ -v`

| Test file | What it tests |
|-----------|--------------|
| `test_config.py` | Config loading, defaults, type checking |
| `test_detector.py` | Detection output schema, smoothing filter |
| `test_calibration.py` | Homography computation, pixel↔goal mapping |
| `test_controller.py` | Trajectory fitting, intercept calculation, PID output |
| `test_actuator.py` | Position clamping, velocity limiting, e-stop logic |
| `test_tracker.py` | Kalman filter initialisation, tracking, coasting, reset |
| `test_coverage.py` | Zone grid construction, idle position, coverage report |
| `test_benchmark.py` | Latency profiler timing, window management, summary |

### 2. Integration Tests (`tests/integration/`)

Run with: `pytest tests/integration/ -v --timeout=30`

| Test class | Scenario | Expected behaviour |
|---|---|---|
| `TestVisionToController` | Mock detections → controller | Valid intercept commands within goal bounds |
| `TestVisionToController` | Noisy detections → controller | Commands remain bounded despite noise |
| `TestControllerToActuator` | Controller → actuator | Commands within limits are forwarded; out-of-range clipped |
| `TestFullSyntheticLoop` | 100-frame synthetic trajectory | No exceptions over 100 iterations |
| `TestFullSyntheticLoop` | Kalman tracker → mapper → controller → actuator | Full pipeline produces valid positions |
| `TestFullSyntheticLoop` | Coverage strategy idle fallback | Valid idle position within goal width |

### 3. System Tests (physical)

Run `scripts/test_loop.py` scenarios (see below).

---

## Bench Tests (no water)

1. **Camera test** – `python -c "import cv2; cap=cv2.VideoCapture(0); print(cap.read()[0])"` — must print `True`.
2. **Servo centre** – Run `python scripts/test_loop.py --scenario hold_centre` — blocker should move to centre and hold.
3. **Full traverse** – Run `--scenario sweep` — blocker should traverse left→right→centre in < 300 ms.
4. **E-stop** – Press the e-stop button mid-traverse — blocker must halt within 10 ms.

---

## Dry-Run Tests (no ball, no water)

1. Launch `python src/pipeline/main.py` with a pre-recorded video (`camera.source: test_videos/dry_run.mp4`).
2. Verify logs show inference latency < 30 ms per frame.
3. Verify actuator commands are within `[min_pos, max_pos]`.
4. Inspect `logs/run_*.jsonl` for any ERROR entries.

---

## Water Tests

> **Safety:** Power off electronics before placing in pool area. Use GFCI protection.

1. **Splash robustness** – Splash water in front of camera; verify detection does not produce false positives > 2/min.
2. **Partial occlusion** – Hold ball half-submerged; verify bounding box covers visible arc.
3. **Full intercept** – Throw ball at various speeds and angles; measure intercept rate over 20 trials.
4. **Continuous operation** – Run for 30 minutes; verify no thermal throttle, no memory leak (monitor with `htop`).

---

## Latency Tests

```bash
python scripts/test_loop.py --scenario latency_benchmark --frames 500
```

Reports:
- Mean inference latency
- P95 inference latency
- Mean end-to-end (capture → command) latency
- Frame drop rate

Target: P95 end-to-end ≤ 80 ms.

---

## Accuracy Tests

```bash
python scripts/test_loop.py --scenario accuracy --scenario-file test_scenarios/shots_30.json
```

`shots_30.json` defines 30 synthetic ball trajectories. Reports:
- Intercept success rate (blocker within paddle_radius of ball at goal line)
- Mean position error at intercept
- Worst-case position error

Target: ≥ 85 % intercept rate on test set.

---

## Logging

All runs write structured JSON-lines logs to `logs/run_<timestamp>.jsonl`:

```json
{"ts": 1700000000.123, "level": "INFO", "module": "detector", "msg": "detection", "cx": 320, "cy": 240, "conf": 0.91}
{"ts": 1700000000.145, "level": "INFO", "module": "controller", "msg": "intercept", "x_intercept": 0.43, "t_impact": 0.18}
{"ts": 1700000000.147, "level": "INFO", "module": "actuator", "msg": "command", "position": 0.43, "velocity": 2.1}
```

Replay a log:

```bash
python -c "
import json, sys
for line in open(sys.argv[1]):
    print(json.loads(line))
" logs/run_20240101_120000.jsonl | head -20
```
