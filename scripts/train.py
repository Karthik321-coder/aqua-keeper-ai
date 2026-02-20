"""
scripts/train.py
Training stub for a custom ball-detection model using Ultralytics YOLOv8.

Usage
-----
    python scripts/train.py \
        --data data/ball_dataset/dataset.yaml \
        --model yolov8n.pt \
        --epochs 100 \
        --imgsz 640 \
        --batch 16 \
        --output models/

Instructions
------------
1.  Prepare your dataset in YOLO format (see VISION.md for layout).
2.  Install dependencies: pip install ultralytics
3.  Run this script.  On first run it downloads the base YOLOv8n weights (~6 MB).
4.  Trained weights are saved to <output>/train/weights/best.pt.
5.  Export for deployment:
      yolo export model=models/train/weights/best.pt format=ncnn imgsz=640
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aqua Keeper AI — YOLOv8 training stub")
    p.add_argument("--data", required=True, help="Path to dataset.yaml")
    p.add_argument("--model", default="yolov8n.pt", help="Base weights or model name")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--output", default="models/", help="Directory to save results")
    p.add_argument("--device", default="cpu", help="Training device: cpu, 0, 0,1, ...")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--patience", type=int, default=20, help="Early-stopping patience")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = parse_args(argv)

    data_path = Path(args.data)
    if not data_path.exists():
        logger.error("Dataset YAML not found: %s", data_path)
        logger.error("Create it following the layout in VISION.md.")
        return 1

    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError:
        logger.error("ultralytics is not installed. Run: pip install ultralytics")
        return 1

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("Loading base model: %s", args.model)
    model = YOLO(args.model)

    logger.info(
        "Starting training — data=%s  epochs=%d  imgsz=%d  batch=%d  device=%s",
        args.data,
        args.epochs,
        args.imgsz,
        args.batch,
        args.device,
    )

    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        patience=args.patience,
        project=str(output_dir),
        name="train",
        exist_ok=True,
    )

    best_weights = output_dir / "train" / "weights" / "best.pt"
    logger.info("Training complete.")
    if best_weights.exists():
        logger.info("Best weights saved to: %s", best_weights)
    else:
        logger.warning("Expected weights not found at %s — check training output.", best_weights)

    # Print validation metrics
    metrics = model.val(data=str(data_path), split="test")
    logger.info("Test set metrics: %s", metrics.results_dict)

    return 0


if __name__ == "__main__":
    sys.exit(main())
