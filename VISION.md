# Vision System

## Model Options

| Model | Framework | mAP50 (COCO ball) | Latency Pi 4 | Latency Jetson Nano |
|-------|-----------|-------------------|--------------|---------------------|
| YOLOv8n | Ultralytics / NCNN | ~45 | ~25 ms | ~12 ms |
| YOLOv8s | Ultralytics / TFLite | ~50 | ~55 ms | ~22 ms |
| RT-DETR-L | PyTorch / TensorRT | ~53 | N/A | ~18 ms (TRT FP16) |

**Recommendation:** YOLOv8n with NCNN export for Raspberry Pi 4; TensorRT FP16 on Jetson Nano.

---

## Dataset Requirements

- **Minimum:** 2 000 labelled images of balls in water/pool environments.
- **Recommended:** 5 000+ images with varied lighting, splash effects, and partial occlusion.
- **Label format:** YOLO-format `.txt` (one file per image: `class cx cy w h` normalised).
- **Classes:** `0 = ball` (single-class detector).

### Directory Layout

```
data/ball_dataset/
├── images/
│   ├── train/   # 80 %
│   ├── val/     # 10 %
│   └── test/    # 10 %
├── labels/
│   ├── train/
│   ├── val/
│   └── test/
└── dataset.yaml
```

### dataset.yaml

```yaml
path: data/ball_dataset
train: images/train
val:   images/val
test:  images/test
nc: 1
names: ['ball']
```

---

## Labelling Guide

1. Use [Label Studio](https://labelstud.io/) or [Roboflow](https://roboflow.com/) for annotation.
2. Draw tight bounding boxes around the visible portion of the ball (exclude splash).
3. For partially submerged balls, label the visible arc only.
4. Export in YOLO format.
5. Augment before training (see below).

---

## Training Pipeline

```bash
# Install Ultralytics
pip install ultralytics

# Fine-tune from COCO pretrained weights
python scripts/train.py \
    --data data/ball_dataset/dataset.yaml \
    --model yolov8n.pt \
    --epochs 100 \
    --imgsz 640 \
    --batch 16 \
    --output models/
```

The training stub in `scripts/train.py` wraps the Ultralytics API. On first run it will download `yolov8n.pt` from the Ultralytics CDN.

---

## Augmentation

Recommended augmentations (configured inside `scripts/train.py` or via `ultralytics` hyp file):

| Augmentation | Value |
|---|---|
| Horizontal flip | 0.5 |
| Random crop | scale 0.5–1.0 |
| HSV hue jitter | ±0.015 |
| HSV saturation | ±0.7 |
| HSV value | ±0.4 |
| Mosaic | 1.0 |
| Blur | 0.01 |
| JPEG compression | 0.01 |

Aqua-specific:
- **Caustic overlay** – overlay random caustic-light pattern at low alpha.
- **Water ripple** – apply sinusoidal distortion to simulate refraction.
- **Splash mask** – paste random splash textures around ball region.

---

## Evaluation Metrics

| Metric | Target |
|--------|--------|
| mAP@50 | ≥ 0.85 |
| mAP@50-95 | ≥ 0.60 |
| Recall @ conf=0.3 | ≥ 0.90 |
| Inference latency (P95) | ≤ 25 ms (Pi 4 NCNN) |
| False positives / min | < 2 |

Run evaluation:

```bash
python -c "
from ultralytics import YOLO
model = YOLO('models/yolov8n_ball.pt')
metrics = model.val(data='data/ball_dataset/dataset.yaml', split='test')
print(metrics.results_dict)
"
```

---

## Deployment Notes

### Raspberry Pi 4 — NCNN Export

```bash
yolo export model=models/yolov8n_ball.pt format=ncnn imgsz=640
# Outputs: models/yolov8n_ball_ncnn_model/
```

Update `configs/default.yaml`:

```yaml
model:
  weights_path: models/yolov8n_ball_ncnn_model
  backend: ncnn
```

### Jetson Nano — TensorRT Export

```bash
yolo export model=models/yolov8n_ball.pt format=engine half=True imgsz=640
# Outputs: models/yolov8n_ball.engine
```

Update `configs/default.yaml`:

```yaml
model:
  weights_path: models/yolov8n_ball.engine
  backend: tensorrt
```

### detector.py integration

`src/vision/detector.py` accepts any backend via the `backend` config key. When `backend=pytorch` (default for development), it loads the `.pt` file with Ultralytics. When `backend=ncnn` or `backend=tensorrt`, it passes the path directly to `YOLO()` which auto-selects the correct runtime.
