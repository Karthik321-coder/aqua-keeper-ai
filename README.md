# Aqua Keeper AI 🏊‍♂️🤖

An automated, computer-vision-powered water-pool goalkeeper that detects incoming balls, predicts their trajectory, and drives a motorised blocker to intercept them in real time.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Quick Start](#quick-start)
4. [Hardware Bill of Materials](#hardware-bill-of-materials)
5. [Wiring Overview](#wiring-overview)
6. [Assembly Steps](#assembly-steps)
7. [Calibration](#calibration)
8. [Software Setup](#software-setup)
9. [Running the System](#running-the-system)
10. [Advanced Usage](#advanced-usage)
11. [Safety Considerations](#safety-considerations)
12. [Documentation Index](#documentation-index)
13. [Project Structure](#project-structure)

---

## Overview

Aqua Keeper AI combines a wide-angle underwater-safe camera with a lightweight deep-learning detector and a fast-response motorised blocker mounted across the goal mouth. A Raspberry Pi 4 (or Jetson Nano) reads frames from the camera, runs inference to locate the ball, predicts where it will cross the goal line, and commands the actuator to move there — all within a target latency budget of **< 80 ms** end-to-end.

### How It Works

```
Camera → YOLOv8n Detection → Kalman Tracking → Trajectory Prediction
    → PID Control → Motor Driver → Blocker intercepts ball
```

1. **Detect** — A YOLOv8n model detects the ball in each camera frame at 30 fps.
2. **Track** — A Kalman filter smooths noisy detections and estimates velocity.
3. **Predict** — A trajectory predictor extrapolates where the ball will cross the goal plane.
4. **Control** — A PID controller computes the blocker's target position.
5. **Actuate** — A high-speed servo or stepper drives the blocker to intercept the ball.

---

## Features

- **High-speed motorised blocker** – belt-driven or linear servo covering the full goal width in < 200 ms.
- **Full-goal coverage** – multi-zone coverage strategy optimises idle position for minimal worst-case reach time.
- **CV-based ball detection** – YOLOv8n (or RT-DETR-L) running at 30 fps on Raspberry Pi 4 with NCNN/TFLite export or on Jetson Nano with TensorRT.
- **Kalman filter tracking** – 6-state constant-acceleration filter for robust position and velocity estimation through occlusions.
- **Predictive interception** – parabolic / linear trajectory extrapolation gives the blocker time to reach the intercept point before the ball arrives.
- **PID motion control** – smooth, jerk-limited position commands with hardware safety limits.
- **Latency profiling** – built-in per-stage benchmarking (capture, inference, control, actuation).
- **Hardware diagnostics** – pre-flight health check for camera, GPIO, model weights, and system resources.
- **Modular configuration** – single YAML file controls all tunable parameters.
- **Comprehensive test suite** – 89+ unit and integration tests; no hardware required.

---

## Quick Start

```bash
# 1. Clone and set up
git clone https://github.com/your-org/aqua-keeper-ai.git
cd aqua-keeper-ai
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 2. Run diagnostics
python scripts/diagnostics.py

# 3. Run unit tests (no hardware needed)
python -m pytest tests/ -v

# 4. Run a simulation (no camera or model needed)
python scripts/test_loop.py --scenario straight
python scripts/test_loop.py --scenario accuracy

# 5. (With hardware) Calibrate and run
python src/vision/calibration.py --mode intrinsic --board 9x6 --square 0.025 --output configs/camera_intrinsics.yaml
python src/vision/calibration.py --mode extrinsic --intrinsics configs/camera_intrinsics.yaml --output configs/goal_homography.yaml
python src/pipeline/main.py --config configs/default.yaml
```

---

## Hardware Bill of Materials

| # | Component | Qty | Notes / Link |
|---|-----------|-----|--------------|
| 1 | Raspberry Pi 4 Model B (4 GB) | 1 | [raspberrypi.com](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) |
| 2 | Arducam IMX477 wide-angle CSI camera (IP67-rated housing) | 1 | Use any USB UVC cam as fallback |
| 3 | Waterproof camera enclosure | 1 | 3D-print files in `cad/` (placeholder) |
| 4 | High-torque servo (e.g. Hitec HS-7980TH) **or** NEMA-17 stepper | 1 | Servo preferred for speed |
| 5 | PCA9685 16-channel PWM servo hat | 1 | I²C address 0x40 |
| 6 | DRV8825 stepper driver (if using stepper) | 1 | — |
| 7 | GT2 timing belt + 20T pulley + idler | 1 set | 2 m belt for 1 m goal |
| 8 | Aluminium extrusion 2020 rail, 1.1 m | 1 | Blocker track |
| 9 | Blocker paddle (foam/ABS, 30 cm × goal height) | 1 | 3D-print placeholder |
| 10 | 12 V / 10 A power supply (IP65) | 1 | Outdoor rated |
| 11 | 5 V / 3 A DC-DC buck converter | 1 | For Pi |
| 12 | Waterproof junction box (200×150×75 mm) | 1 | Houses Pi + electronics |
| 13 | M3/M4 hardware, cable glands, heat-shrink | — | — |

> **Note:** For Jetson Nano substitute the PCA9685 hat with a PWM breakout on the Nano's GPIO header.

---

## Wiring Overview

```
12V PSU
  ├─── Buck converter → 5V → Raspberry Pi 4 (USB-C)
  ├─── Servo hat VCC (6V tap or 12V depending on servo)
  └─── DRV8825 VMOT (if stepper)

Raspberry Pi 4
  ├─── CSI ribbon → Camera
  ├─── I²C (SDA/SCL, GPIO 2/3) → PCA9685 servo hat
  │       └─── Channel 0 → Servo signal wire
  └─── GPIO 17/27/22 (STEP/DIR/EN) → DRV8825 (stepper mode)

PCA9685
  └─── Channel 0 PWM out → Servo signal (orange wire)
       Servo power → 6V from dedicated BEC/buck
       Servo ground → common GND
```

Full pin-by-pin table: see `docs/wiring_detail.md` (placeholder — add your specific GPIO assignments here).

---

## Assembly Steps

1. **Frame** – Cut 2020 extrusion to goal width + 100 mm. Mount end brackets.
2. **Drive** – Attach pulley to servo/stepper shaft. Route GT2 belt around idler. Fix blocker carriage to belt with clamp plates.
3. **Camera** – Mount inside waterproof housing at goal centre, angled to cover the full goal face. Seal with silicone.
4. **Electronics box** – Mount Pi, servo hat, and buck converter on DIN rail inside junction box. Run cable glands for servo cable, camera ribbon extension, and power input.
5. **Power** – Wire 12 V supply → buck converter → Pi; separate 6 V BEC → servo hat power rail.
6. **Cable management** – Zip-tie all wiring to extrusion. Use flexible drag chain for moving blocker cable.
7. **Waterproofing** – Confirm all connectors outside the box are IP67+. Apply conformal coating to PCBs.

---

## Calibration

### Camera Intrinsics

```bash
python src/vision/calibration.py --mode intrinsic --board 9x6 --square 0.025 --output configs/camera_intrinsics.yaml
```

Print a 9×6 checkerboard and capture 20+ images from different angles.

### Goal-Coordinate Mapping

```bash
python src/vision/calibration.py --mode extrinsic --intrinsics configs/camera_intrinsics.yaml --output configs/goal_homography.yaml
```

Mark the four corners of the goal mouth in the calibration UI; the script computes the homography from image pixels to goal coordinates (metres from left post).

### Blocker Zero-Position

Edit `configs/default.yaml` → `motor.zero_offset_steps` until the blocker parks at the physical centre when commanded to position 0.0.

---

## Software Setup

### Requirements

- Python 3.9+
- See `requirements.txt`

### Install

```bash
git clone https://github.com/your-org/aqua-keeper-ai.git
cd aqua-keeper-ai
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Model Weights

Download or train YOLOv8n weights (see `VISION.md`) and place them at the path specified in `configs/default.yaml` → `model.weights_path`. A placeholder path `models/yolov8n_ball.pt` is used by default.

```bash
mkdir -p models
# Copy your weights:
cp /path/to/yolov8n_ball.pt models/
```

---

## Running the System

### Main Loop (live camera)

```bash
python src/pipeline/main.py --config configs/default.yaml
```

### Simulation / Dry Run

```bash
python scripts/test_loop.py --config configs/default.yaml --scenario straight
```

### Training Stub

```bash
python scripts/train.py --data data/ball_dataset --epochs 50 --output models/
```

### Unit Tests

```bash
pytest tests/ -v
```

---

## Advanced Usage

### Kalman Tracker

The Kalman ball tracker (`src/vision/tracker.py`) provides 6-state estimation (position, velocity, acceleration) for robust tracking through brief occlusions:

```python
from src.vision.tracker import KalmanBallTracker

tracker = KalmanBallTracker(process_noise=1.0, measurement_noise=10.0, max_coast_frames=10)
tracker.update(cx=320.0, cy=240.0, t=0.0)
print(tracker.position)     # (x, y)
print(tracker.velocity)     # (vx, vy)
print(tracker.is_tracking)  # True
```

### Coverage Strategy

The multi-zone coverage strategy (`src/control/coverage.py`) divides the goal into zones and computes the optimal idle position:

```python
from src.control.coverage import CoverageStrategy

cov = CoverageStrategy(goal_width=2.4, goal_height=0.9, zone_cols=5, zone_rows=3)
print(f"Idle position: {cov.idle_position:.2f} m")
print(f"Worst-case reach: {cov.worst_case_reach_time(cov.idle_position):.3f} s")

# Per-zone reachability report
for entry in cov.coverage_report(cov.idle_position):
    print(f"  {entry['zone']}: {entry['reach_time_s']:.3f} s {'✅' if entry['reachable_200ms'] else '⚠️'}")
```

### Latency Profiling

The benchmark profiler (`src/utils/benchmark.py`) tracks per-stage latencies:

```python
from src.utils.benchmark import LatencyProfiler

prof = LatencyProfiler()
with prof.measure("inference"):
    # ... run inference ...
    pass
prof.tick()
print(prof.summary())
```

### System Diagnostics

Run a comprehensive pre-flight check before deployment:

```bash
python scripts/diagnostics.py --config configs/default.yaml
```

This checks Python version, dependencies, camera access, model weights, disk space, and GPIO availability.

---

## Safety Considerations

1. **Emergency stop** – Connect a hardware e-stop button to GPIO pin 26. The software monitors this pin; `actuator.py` will disable motor output immediately.
2. **Position limits** – Software hard-limits prevent the blocker from over-running the track ends (configurable in `configs/default.yaml` → `motor.min_pos` / `motor.max_pos`).
3. **Waterproofing** – Never power electronics while the junction box is open near water. Verify all IP ratings before deployment.
4. **Pinch points** – Keep spectators clear of the belt drive during operation. Add physical end-stops as a mechanical backup.
5. **Electrical safety** – Use a fused supply. Ground the metal extrusion frame.
6. **Children** – The blocker moves rapidly. Install a mesh guard in front of the mechanism.
7. **Failsafe mode** – If vision or control threads crash, the actuator holds its last safe position (centre) rather than driving to a limit.

---

## Documentation Index

| Document | Description |
|---|---|
| [README.md](README.md) | Project overview, quick start, and usage |
| [ARCHITECTURE.md](ARCHITECTURE.md) | System design, data flow, state machine, deployment diagrams |
| [VISION.md](VISION.md) | Model selection, dataset, training, augmentation, deployment |
| [CONTROL.md](CONTROL.md) | Trajectory prediction, PID tuning, servo/stepper sizing |
| [HARDWARE.md](HARDWARE.md) | Full BOM, 3D print specs, wiring, step-by-step assembly |
| [OPERATIONS.md](OPERATIONS.md) | Deployment, monitoring, maintenance, troubleshooting |
| [TESTING.md](TESTING.md) | Unit, integration, system, and latency test procedures |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Development setup, coding standards, PR process |

---

## Project Structure

```
aqua-keeper-ai/
├── configs/
│   └── default.yaml          # All tuneable parameters
├── diagrams/
│   ├── architecture.png       # (placeholder — see ARCHITECTURE.md mermaid)
│   └── control-loop.png       # (placeholder — see ARCHITECTURE.md mermaid)
├── models/                    # Place model weights here (gitignored)
├── scripts/
│   ├── diagnostics.py         # Hardware and software diagnostics
│   ├── train.py               # Training stub
│   └── test_loop.py           # Simulation test loop
├── src/
│   ├── control/
│   │   ├── actuator.py        # Motor interface + safety interlocks
│   │   ├── controller.py      # Trajectory prediction + PID
│   │   └── coverage.py        # Multi-zone coverage strategy
│   ├── pipeline/
│   │   └── main.py            # End-to-end loop
│   ├── utils/
│   │   ├── benchmark.py       # Latency profiler
│   │   ├── config.py          # Config loader
│   │   └── logger.py          # Structured logger
│   └── vision/
│       ├── calibration.py     # Camera & field calibration
│       ├── detector.py        # Ball detection + tracking
│       └── tracker.py         # Kalman filter ball tracker
├── tests/
│   ├── unit/                  # Unit tests (89 tests)
│   └── integration/           # Integration tests
├── ARCHITECTURE.md            # System design and diagrams
├── CONTRIBUTING.md            # Development and contribution guide
├── CONTROL.md                 # Control system details
├── HARDWARE.md                # BOM, assembly, wiring guide
├── OPERATIONS.md              # Deployment and monitoring
├── TESTING.md                 # Testing strategy
├── VISION.md                  # Vision pipeline and training
├── pytest.ini
└── requirements.txt
```
