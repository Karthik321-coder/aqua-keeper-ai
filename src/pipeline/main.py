"""
src/pipeline/main.py
End-to-end pipeline: vision → controller → actuator.

Usage
-----
    python src/pipeline/main.py --config configs/default.yaml [--loglevel DEBUG]
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path
from typing import Optional

# Ensure repo root is on the path when running as a script
_REPO_ROOT = str(Path(__file__).resolve().parents[2])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from src.control.actuator import Actuator
from src.control.controller import BlockerController
from src.utils.config import load_config
from src.utils.logger import get_logger
from src.vision.calibration import GoalMapper, load_goal_mapper
from src.vision.detector import BallDetector

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Graceful shutdown
# ---------------------------------------------------------------------------

_shutdown_requested = False


def _handle_signal(signum: int, frame: object) -> None:
    global _shutdown_requested
    logger.info("Shutdown signal received (%d).", signum)
    _shutdown_requested = True


signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run(config_path: str) -> None:
    cfg = load_config(config_path)
    run_logger = get_logger("pipeline", cfg.get("logging", {}))

    # ------------------------------------------------------------------
    # Initialise subsystems
    # ------------------------------------------------------------------
    goal_width: float = cfg["geometry"]["goal_width"]
    goal_height: float = cfg["geometry"]["goal_height"]

    detector = BallDetector(
        weights_path=cfg["model"]["weights_path"],
        confidence_threshold=cfg["model"]["confidence_threshold"],
        smoothing_alpha=cfg["model"]["smoothing_alpha"],
        history_len=cfg["model"]["history_len"],
        imgsz=cfg["model"]["imgsz"],
        device=cfg["model"]["device"],
    )

    controller = BlockerController(
        goal_width=goal_width,
        goal_height=goal_height,
        pid_kp=cfg["control"]["kp"],
        pid_ki=cfg["control"]["ki"],
        pid_kd=cfg["control"]["kd"],
        min_observations=cfg["control"]["min_observations"],
    )

    actuator = Actuator(
        backend=cfg["motor"]["backend"],
        goal_width=goal_width,
        min_pos=cfg["motor"]["min_pos"],
        max_pos=cfg["motor"]["max_pos"],
        max_velocity=cfg["motor"]["max_velocity"],
        max_acceleration=cfg["motor"]["max_acceleration"],
        estop_pin=cfg["motor"].get("estop_pin"),
    )

    # Goal mapper (optional — skipped if file not present)
    mapper: Optional[GoalMapper] = None
    homography_path: str = cfg.get("camera", {}).get("homography_path", "")
    if homography_path and Path(homography_path).exists():
        mapper = load_goal_mapper(homography_path)
        logger.info("Goal homography loaded from %s", homography_path)
    else:
        logger.warning(
            "No goal homography found at %r. Using raw pixel coordinates scaled to goal dimensions.",
            homography_path,
        )

    # Load model
    logger.info("Loading detector model from %s …", cfg["model"]["weights_path"])
    try:
        detector.load()
    except Exception as exc:
        logger.error("Failed to load model: %s", exc)
        logger.error("Running without a detector — no ball tracking will occur.")

    # Open camera
    camera_source = cfg["camera"]["source"]
    try:
        cap = BallDetector.open_camera(
            camera_source,
            width=cfg["camera"]["width"],
            height=cfg["camera"]["height"],
        )
    except OSError as exc:
        logger.error("Cannot open camera: %s", exc)
        return

    # Centre the blocker
    actuator.centre()
    logger.info("System ready. Starting main loop…")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    frame_count = 0
    inference_times: list[float] = []

    try:
        while not _shutdown_requested:
            t_loop_start = time.monotonic()

            ret, frame = cap.read()
            if not ret:
                logger.warning("Empty frame from camera. Retrying…")
                time.sleep(0.05)
                continue

            # --- Perception ---
            t_infer_start = time.monotonic()
            detection = None
            if detector.is_loaded():
                try:
                    detection = detector.detect(frame)
                except Exception as exc:
                    logger.error("Detector error: %s", exc)

            t_infer_end = time.monotonic()
            infer_ms = (t_infer_end - t_infer_start) * 1000.0
            inference_times.append(infer_ms)

            if detection is not None:
                run_logger.info(
                    "detection",
                    cx=round(detection.cx, 1),
                    cy=round(detection.cy, 1),
                    conf=round(detection.confidence, 3),
                    infer_ms=round(infer_ms, 1),
                )

                # --- Map pixels → goal coordinates ---
                if mapper is not None:
                    ball_x, ball_y = mapper.pixel_to_goal(detection.cx, detection.cy)
                else:
                    # Fallback: scale pixels proportionally to goal dimensions
                    cam_w = cfg["camera"]["width"]
                    cam_h = cfg["camera"]["height"]
                    ball_x = (detection.cx / cam_w) * goal_width
                    ball_y = (1.0 - detection.cy / cam_h) * goal_height

                # Approximate depth (z) — assume ball at fixed depth for now
                ball_z = cfg["geometry"].get("default_ball_depth", 1.0)

                # --- Control ---
                blocker_pos = actuator.get_position()
                target_pos, t_impact = controller.update(
                    ball_x, ball_y, ball_z, blocker_pos
                )

                # --- Actuation ---
                actual_pos = actuator.move_to(target_pos)

                run_logger.info(
                    "command",
                    target=round(target_pos, 3),
                    actual=round(actual_pos, 3),
                    t_impact=round(t_impact, 3) if t_impact is not None else None,
                )

            frame_count += 1

            # Periodic stats
            if frame_count % 100 == 0:
                mean_infer = sum(inference_times[-100:]) / len(inference_times[-100:])
                logger.info(
                    "Frame %d | mean inference %.1f ms | blocker %.3f m",
                    frame_count,
                    mean_infer,
                    actuator.get_position(),
                )

    finally:
        logger.info("Shutting down…")
        actuator.centre()
        time.sleep(0.2)
        actuator.disable()
        cap.release()
        logger.info("Done. Processed %d frames.", frame_count)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aqua Keeper AI — main pipeline")
    p.add_argument("--config", default="configs/default.yaml")
    p.add_argument(
        "--loglevel",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.loglevel),
        format="%(asctime)s %(levelname)s %(name)s — %(message)s",
    )
    run(args.config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
