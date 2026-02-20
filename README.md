# Aqua Keeper AI 🏊‍♂️🤖

An automated, computer-vision-powered water-pool goalkeeper that detects incoming balls, predicts their trajectory, and drives a motorised blocker to intercept them in real time.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Hardware Bill of Materials](#hardware-bill-of-materials)
4. [Wiring Overview](#wiring-overview)
5. [Assembly Steps](#assembly-steps)
6. [Calibration](#calibration)
7. [Software Setup](#software-setup)
8. [Running the System](#running-the-system)
9. [Safety Considerations](#safety-considerations)
10. [Project Structure](#project-structure)

---

## Overview

Aqua Keeper AI combines a wide-angle underwater-safe camera with a lightweight deep-learning detector and a fast-response motorised blocker mounted across the goal mouth. A Raspberry Pi 4 (or Jetson Nano) reads frames from the camera, runs inference to locate the ball, predicts where it will cross the goal line, and commands the actuator to move there — all within a target latency budget of **< 80 ms** end-to-end.

---

## Features

- **High-speed motorised blocker** – belt-driven or linear servo covering the full goal width in < 200 ms.
- **Full-goal coverage** – blocker paddle spans the goal height; horizontal position is the controlled axis.
- **CV-based ball detection** – YOLOv8n (or RT-DETR-L) running at 30 fps on Raspberry Pi 4 with NCNN/TFLite export or on Jetson Nano with TensorRT.
- **Predictive interception** – parabolic / linear trajectory extrapolation gives the blocker time to reach the intercept point before the ball arrives.
- **PID motion control** – smooth, jerk-limited position commands with hardware safety limits.
- **Modular configuration** – single YAML file controls all tunable parameters.

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

## Safety Considerations

1. **Emergency stop** – Connect a hardware e-stop button to GPIO pin 26. The software monitors this pin; `actuator.py` will disable motor output immediately.
2. **Position limits** – Software hard-limits prevent the blocker from over-running the track ends (configurable in `configs/default.yaml` → `motor.min_pos` / `motor.max_pos`).
3. **Waterproofing** – Never power electronics while the junction box is open near water. Verify all IP ratings before deployment.
4. **Pinch points** – Keep spectators clear of the belt drive during operation. Add physical end-stops as a mechanical backup.
5. **Electrical safety** – Use a fused supply. Ground the metal extrusion frame.
6. **Children** – The blocker moves rapidly. Install a mesh guard in front of the mechanism.
7. **Failsafe mode** – If vision or control threads crash, the actuator holds its last safe position (centre) rather than driving to a limit.

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
│   ├── train.py               # Training stub
│   └── test_loop.py           # Simulation test loop
├── src/
│   ├── control/
│   │   ├── actuator.py        # Motor interface + safety interlocks
│   │   └── controller.py      # Trajectory prediction + PID
│   ├── pipeline/
│   │   └── main.py            # End-to-end loop
│   ├── utils/
│   │   ├── config.py          # Config loader
│   │   └── logger.py          # Structured logger
│   └── vision/
│       ├── calibration.py     # Camera & field calibration
│       └── detector.py        # Ball detection + tracking
├── tests/
│   └── unit/                  # pytest unit tests
├── ARCHITECTURE.md
├── CONTROL.md
├── OPERATIONS.md
├── TESTING.md
├── VISION.md
├── pytest.ini
└── requirements.txt
```
