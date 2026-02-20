# Hardware Guide

Detailed bill of materials, mechanical assembly, 3D-printable enclosure references, and wiring instructions for building the Aqua Keeper AI water-pool goalkeeper.

---

## Table of Contents

1. [Compatible Hardware Platforms](#compatible-hardware-platforms)
2. [Full Bill of Materials](#full-bill-of-materials)
3. [3D-Printable Components](#3d-printable-components)
4. [Wiring Diagram](#wiring-diagram)
5. [Step-by-Step Assembly](#step-by-step-assembly)
6. [Waterproofing Guide](#waterproofing-guide)
7. [Servo and Motor Sizing](#servo-and-motor-sizing)
8. [Power Budget](#power-budget)
9. [Mechanical Dimensions](#mechanical-dimensions)

---

## Compatible Hardware Platforms

| Platform | CPU | GPU / Accelerator | Inference (YOLOv8n) | Recommended |
|---|---|---|---|---|
| Raspberry Pi 4B (4 GB) | BCM2711 quad-core A72 | — (NCNN on CPU) | ~25 ms @ 640px | ✅ Primary |
| Raspberry Pi 5 (8 GB) | BCM2712 quad-core A76 | — (NCNN on CPU) | ~15 ms @ 640px | ✅ Upgrade |
| NVIDIA Jetson Nano (4 GB) | quad-core A57 | 128-core Maxwell | ~12 ms (TensorRT FP16) | ✅ Performance |
| NVIDIA Jetson Orin Nano | 6-core A78AE | 1024-core Ampere | ~4 ms (TensorRT FP16) | ⭐ Best |
| Intel NUC (i5) | Core i5 | — (OpenVINO) | ~18 ms | ✔️ Alternative |

> **Note:** The software is platform-agnostic. Any Linux SBC running Python 3.9+ with a USB or CSI camera will work. Adjust `configs/default.yaml` → `model.backend` and `model.device` accordingly.

---

## Full Bill of Materials

### Compute and Electronics

| # | Component | Qty | Approx. Cost | Notes |
|---|---|---|---|---|
| 1 | Raspberry Pi 4 Model B (4 GB) | 1 | $55 | Or Jetson Nano |
| 2 | MicroSD card 32 GB (Class 10) | 1 | $10 | Raspberry Pi OS Lite |
| 3 | USB-C power supply 5 V / 3 A | 1 | $12 | For Pi (if not using buck) |
| 4 | PCA9685 16-ch PWM servo hat | 1 | $8 | I²C address 0x40 |
| 5 | DRV8825 stepper driver module | 1 | $5 | Optional (stepper mode) |
| 6 | 12 V / 10 A enclosed PSU (IP65) | 1 | $25 | Outdoor rated |
| 7 | 5 V / 3 A DC-DC buck converter | 1 | $6 | Powers the Pi |
| 8 | 6 V BEC (battery eliminator circuit) | 1 | $8 | Servo power |
| 9 | E-stop button (NO, IP67) | 1 | $10 | Safety interlock |
| 10 | 10 kΩ pull-up resistor | 1 | $0.10 | E-stop GPIO pull-up |

### Vision

| # | Component | Qty | Approx. Cost | Notes |
|---|---|---|---|---|
| 11 | Arducam IMX477 wide-angle CSI camera | 1 | $30 | 160° FOV, 12 MP |
| 12 | CSI ribbon cable (30 cm) | 1 | $4 | Standard 15-pin FFC |
| 13 | Waterproof camera housing (IP68) | 1 | $15 | 3D-print or buy |

### Mechanical

| # | Component | Qty | Approx. Cost | Notes |
|---|---|---|---|---|
| 14 | Aluminium 2020 V-slot extrusion, 1.1 m | 1 | $12 | Blocker rail |
| 15 | 2020 end brackets | 2 | $4 | Rail mounting |
| 16 | V-slot gantry plate (linear carriage) | 1 | $8 | Blocker mount |
| 17 | GT2 timing belt (2 mm pitch), 2.5 m | 1 | $5 | Belt drive |
| 18 | GT2 20-tooth pulley (5 mm bore) | 1 | $3 | Drive pulley |
| 19 | GT2 idler bearing (20T, 5 mm bore) | 1 | $3 | Belt tensioner |
| 20 | High-torque servo (Hitec HS-7980TH) | 1 | $80 | Or NEMA-17 stepper |
| 21 | NEMA-17 stepper motor (alternative) | 1 | $15 | 1.8°/step |
| 22 | Blocker paddle (ABS / HDPE), 30 cm tall | 1 | $10 | 3D-print |

### Enclosures and Fasteners

| # | Component | Qty | Approx. Cost | Notes |
|---|---|---|---|---|
| 23 | Waterproof junction box 200×150×75 mm | 1 | $15 | Houses electronics |
| 24 | PG9 cable glands (IP68) | 4 | $4 | Cable pass-through |
| 25 | M3×10 stainless socket-head bolts | 20 | $3 | Assembly |
| 26 | M4×12 stainless socket-head bolts | 10 | $3 | Rail mounting |
| 27 | M3 T-nuts for 2020 extrusion | 20 | $3 | Slot mounting |
| 28 | Flexible cable drag chain (10×10 mm) | 0.5 m | $5 | Moving cable guide |
| 29 | Heat-shrink tubing assortment | 1 | $4 | Weatherproofing |
| 30 | Silica gel desiccant packets | 5 | $2 | Condensation control |

**Estimated total cost: ~$360 (servo variant) / ~$290 (stepper variant)**

---

## 3D-Printable Components

The following parts are designed for FDM 3D printing in PETG or ASA (outdoor UV-resistant).

| Part | Print Material | Infill | Supports | Notes |
|---|---|---|---|---|
| Camera housing | PETG | 40% | Yes | O-ring groove for IP68 seal |
| Blocker paddle | ABS/PETG | 30% | No | Flat panel, easy print |
| Servo mount bracket | PETG | 50% | Yes | Bolts to extrusion end |
| Belt clamp plates (×2) | PETG | 60% | No | Secures belt to carriage |
| Cable drag-chain mount | PETG | 40% | No | Clips onto extrusion |

> **STL files:** Place your STL files in a `cad/` directory at the repository root. The `.gitignore` does not exclude `cad/` by default.

### Recommended print settings

- **Layer height:** 0.2 mm
- **Nozzle:** 0.4 mm
- **Wall count:** 4 (for waterproof parts)
- **Top/bottom layers:** 5
- **Temperature:** 240°C nozzle / 80°C bed (PETG)

---

## Wiring Diagram

```
                     ┌──────────────────────┐
    ┌─── 12V PSU ──▸│ Junction Box          │
    │    (IP65)      │                       │
    │                │  ┌─────────────┐      │
    ├────────────────┤▸ │ Buck 5V/3A  │──▸ Pi USB-C power
    │                │  └─────────────┘      │
    │                │  ┌─────────────┐      │
    ├────────────────┤▸ │ BEC 6V/5A   │──▸ PCA9685 V+ rail
    │                │  └─────────────┘      │
    │                │                       │
    │                │  ┌─────────────┐      │
    │                │  │ Raspberry Pi│      │
    │                │  │    4B       │      │
    │                │  │  ┌───────┐  │      │
    │                │  │  │GPIO   │  │      │
    │                │  │  │ 2(SDA)├──┤──▸ PCA9685 SDA
    │                │  │  │ 3(SCL)├──┤──▸ PCA9685 SCL
    │                │  │  │26(EST)├──┤──▸ E-stop button (NO)
    │                │  │  │  GND  ├──┤──▸ Common ground bus
    │                │  │  └───────┘  │      │
    │                │  │  CSI port ──┤──▸ Camera ribbon
    │                │  └─────────────┘      │
    │                │                       │
    │                │  ┌─────────────┐      │
    │                │  │ PCA9685 hat │      │
    │                │  │  CH0 PWM  ──┤──▸ Servo signal (orange)
    │                │  │  V+ rail  ──┤──◂ 6V BEC
    │                │  │  GND      ──┤──▸ Common GND
    │                │  └─────────────┘      │
    │                └──────────────────────┘
    │
    │    Stepper variant (replace servo):
    │    ┌─────────────┐
    ├───▸│ DRV8825     │
    │    │ VMOT (12V)  │
    │    │ STEP ◂── GPIO 17
    │    │ DIR  ◂── GPIO 27
    │    │ EN   ◂── GPIO 22
    │    │ OUT1/OUT2 ──▸ NEMA-17
    │    └─────────────┘
    │
    └──▸ Common GND bus
```

### Pin Assignment Table

| GPIO (BCM) | Function | Wire Colour | Notes |
|---|---|---|---|
| 2 (SDA1) | I²C data | Blue | PCA9685 SDA |
| 3 (SCL1) | I²C clock | Yellow | PCA9685 SCL |
| 17 | Stepper STEP | Green | DRV8825 (stepper only) |
| 27 | Stepper DIR | White | DRV8825 (stepper only) |
| 22 | Stepper EN | Orange | DRV8825 active-low |
| 26 | E-stop input | Red | Pull-up 10 kΩ to 3.3 V |
| GND | Common ground | Black | All devices share GND |

---

## Step-by-Step Assembly

### Phase 1: Frame and Rail (30 min)

1. Cut the 2020 aluminium extrusion to **goal width + 100 mm** (e.g., 2500 mm for a 2.4 m goal).
2. Attach end brackets at both ends using M4 bolts and T-nuts.
3. Mount the V-slot gantry plate (linear carriage) onto the rail — it should slide freely.
4. Mount the GT2 drive pulley on the servo/stepper shaft.
5. Route the GT2 belt around the drive pulley, along the rail, around the idler at the far end, and back.
6. Clamp the belt to the gantry plate using the 3D-printed belt-clamp plates and M3 bolts.
7. Tension the belt by adjusting the idler position — there should be no slack but the belt should not be overly tight.

### Phase 2: Blocker Paddle (15 min)

1. Attach the blocker paddle to the gantry plate with M3 bolts.
2. The paddle should be oriented vertically, covering the full goal height (0.9 m).
3. Verify that the paddle clears the rail at both end-stops.

### Phase 3: Camera Mount (20 min)

1. Mount the IMX477 camera inside the waterproof housing.
2. Seal the housing with silicone sealant around the lens opening.
3. Route the CSI ribbon cable through a cable gland.
4. Mount the camera housing at goal centre, facing outward to cover the full goal face.
5. Angle the camera so the entire goal is visible in the field of view.

### Phase 4: Electronics Box (30 min)

1. Mount the Raspberry Pi on standoffs or DIN rail inside the junction box.
2. Mount the PCA9685 servo hat on top of the Pi GPIO header.
3. Install the 5 V buck converter and 6 V BEC on the DIN rail.
4. Wire the 12 V input through a cable gland to the buck converter and BEC.
5. Wire the 5 V output to the Pi USB-C power input.
6. Wire the 6 V BEC output to the PCA9685 V+ power rail.
7. Connect the servo signal cable from PCA9685 CH0 through a cable gland.
8. Connect the e-stop button through a cable gland to GPIO 26 and GND.
9. Add a 10 kΩ pull-up resistor between GPIO 26 and 3.3 V.

### Phase 5: Cable Management (15 min)

1. Zip-tie all cables to the extrusion.
2. Install the drag chain for the moving blocker cable (if the servo is on the carriage).
3. Apply heat-shrink to all exposed solder joints.
4. Verify all cable glands are tight and sealed.

### Phase 6: Software Setup (20 min)

1. Flash Raspberry Pi OS Lite to the MicroSD card.
2. Connect to Wi-Fi and enable SSH.
3. Clone the repository and install dependencies:
   ```bash
   git clone https://github.com/your-org/aqua-keeper-ai.git
   cd aqua-keeper-ai
   python -m venv .venv && source .venv/bin/activate
   pip install -r requirements.txt
   ```
4. Run diagnostics: `python scripts/diagnostics.py`
5. Calibrate the camera: see [README.md — Calibration](README.md#calibration).
6. Run the test loop: `python scripts/test_loop.py --scenario straight`

### Phase 7: Waterproofing Verification (15 min)

1. Close the junction box and tighten all cable glands.
2. Verify all connectors outside the box are IP67+.
3. Place a silica gel packet inside the junction box.
4. Apply conformal coating to all exposed PCB areas (optional but recommended).
5. **Do not power on near water until all seals are verified.**

---

## Waterproofing Guide

| Component | Protection Level | Method |
|---|---|---|
| Junction box | IP66 | Factory-sealed enclosure |
| Cable glands | IP68 | PG9 nylon glands, hand-tight + 1/4 turn |
| Camera housing | IP68 | O-ring seal + silicone sealant |
| CSI ribbon cable | Splash-proof | Heat-shrink at both ends |
| Servo/stepper | Splash-proof | Mount under extrusion overhang |
| Belt and pulleys | Water-resistant | Stainless/anodised components |
| Power supply | IP65 | Factory rating |

> **Important:** Never power electronics while the junction box is open near water. Always use a GFCI/RCD-protected outlet.

---

## Servo and Motor Sizing

### Speed Requirement

```
Ball speed:       10 m/s
Distance to goal: 2 m
Time budget:      200 ms (travel) + 80 ms (compute) = 280 ms total
Worst-case traverse: goal_width / 2 = 1.2 m
Required speed:   1.2 m / 0.15 s ≈ 8.0 m/s (burst)
```

### Recommended Servos

| Servo | Speed | Torque | Voltage | Suitability |
|---|---|---|---|---|
| Hitec HS-7980TH | 0.14 s/60° @ 7.4 V | 36 kg·cm | 6–7.4 V | ✅ Good |
| Savox SV-1270TG | 0.14 s/60° @ 7.4 V | 35 kg·cm | 6–7.4 V | ✅ Good |
| JX CLS-12V7346 | 0.07 s/60° @ 12 V | 46 kg·cm | 12 V | ⭐ Best |

### Stepper Alternative

| Motor | Steps/rev | Holding Torque | Max Speed (24 V) |
|---|---|---|---|
| NEMA-17 (42BYGH40) | 200 | 40 N·cm | ~1000 RPM |

> With 1/16 micro-stepping and a 20T GT2 pulley, the stepper achieves ~2.5 m/s linear belt speed — adequate for slower pool games.

---

## Power Budget

| Component | Voltage | Current (max) | Power |
|---|---|---|---|
| Raspberry Pi 4B | 5 V | 3 A | 15 W |
| PCA9685 hat | 3.3 V (logic) | 10 mA | 0.03 W |
| Servo (under load) | 6 V | 5 A | 30 W |
| Camera | 3.3 V (from Pi) | 0.3 A | 1 W |
| **Total** | | | **~46 W** |

A 12 V / 10 A (120 W) supply provides ample headroom.

---

## Mechanical Dimensions

```
Goal frame (standard water polo):
  Width:  2.4 m  (between inner faces of posts)
  Height: 0.9 m  (above water surface)

Extrusion rail:
  Length: 2.5 m  (goal width + 100 mm overhang)
  Profile: 20×20 mm V-slot

Blocker paddle:
  Width:  0.30 m  (covers ±0.15 m from centre)
  Height: 0.90 m  (full goal height)

Gantry plate:
  Size: 44×44 mm (standard V-slot plate)

Belt run:
  Total length: ~5.2 m  (2× rail length + pulley circumferences)
  Pitch: 2 mm GT2
```
