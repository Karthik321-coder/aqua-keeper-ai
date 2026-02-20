"""
src/vision/detector.py
Ball detection and tracking using a lightweight YOLO model.

Supports backends: pytorch (.pt), ncnn, tensorrt (.engine).
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional

import cv2
import numpy as np


class Detection:
    """Single ball detection result."""

    __slots__ = ("cx", "cy", "w", "h", "confidence", "timestamp")

    def __init__(
        self,
        cx: float,
        cy: float,
        w: float,
        h: float,
        confidence: float,
        timestamp: float,
    ) -> None:
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.confidence = confidence
        self.timestamp = timestamp

    def radius(self) -> float:
        """Return approximate ball radius in pixels."""
        return (self.w + self.h) / 4.0


class ExponentialSmoother:
    """1-D exponential moving average for noisy detections."""

    def __init__(self, alpha: float = 0.4) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self._value: Optional[float] = None

    def update(self, measurement: float) -> float:
        if self._value is None:
            self._value = measurement
        else:
            self._value = self.alpha * measurement + (1.0 - self.alpha) * self._value
        return self._value

    def reset(self) -> None:
        self._value = None

    @property
    def value(self) -> Optional[float]:
        return self._value


class BallDetector:
    """
    Wraps a YOLO-family model for real-time ball detection.

    Parameters
    ----------
    weights_path : str
        Path to model weights (.pt, ncnn directory, or .engine).
    confidence_threshold : float
        Minimum detection confidence to accept.
    smoothing_alpha : float
        EMA alpha for x/y smoothing (0 < alpha ≤ 1; higher = less smoothing).
    history_len : int
        Number of past detections kept for trajectory estimation.
    imgsz : int
        Inference image size.
    device : str
        Torch device string ('cpu', 'cuda:0', etc.).
    """

    def __init__(
        self,
        weights_path: str = "models/yolov8n_ball.pt",
        confidence_threshold: float = 0.40,
        smoothing_alpha: float = 0.40,
        history_len: int = 10,
        imgsz: int = 640,
        device: str = "cpu",
    ) -> None:
        self.weights_path = weights_path
        self.confidence_threshold = confidence_threshold
        self.imgsz = imgsz
        self.device = device
        self.history: deque[Detection] = deque(maxlen=history_len)

        self._smoother_cx = ExponentialSmoother(smoothing_alpha)
        self._smoother_cy = ExponentialSmoother(smoothing_alpha)
        self._model = None
        self._frames_since_detection = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def load(self) -> None:
        """Load the YOLO model.  Call once before the inference loop."""
        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(self.weights_path)
            # Warm-up
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self._model(dummy, verbose=False, device=self.device)
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required for BallDetector. "
                "Install it with: pip install ultralytics"
            ) from exc

    def is_loaded(self) -> bool:
        return self._model is not None

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> Optional[Detection]:
        """
        Run inference on a BGR frame and return the highest-confidence ball
        detection (or None if no ball is found above threshold).

        The raw bounding-box centre is smoothed with EMA before returning.
        """
        if self._model is None:
            raise RuntimeError("Model not loaded. Call BallDetector.load() first.")

        ts = time.monotonic()
        results = self._model(
            frame,
            verbose=False,
            conf=self.confidence_threshold,
            device=self.device,
            imgsz=self.imgsz,
        )

        best: Optional[Detection] = None
        best_conf = 0.0

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                conf = float(box.conf[0])
                if conf < self.confidence_threshold or conf <= best_conf:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                raw_cx = (x1 + x2) / 2.0
                raw_cy = (y1 + y2) / 2.0
                cx = self._smoother_cx.update(raw_cx)
                cy = self._smoother_cy.update(raw_cy)
                best = Detection(
                    cx=cx,
                    cy=cy,
                    w=x2 - x1,
                    h=y2 - y1,
                    confidence=conf,
                    timestamp=ts,
                )
                best_conf = conf

        if best is not None:
            self.history.append(best)
            self._frames_since_detection = 0
        else:
            self._frames_since_detection += 1
            if self._frames_since_detection > 5:
                self._smoother_cx.reset()
                self._smoother_cy.reset()

        return best

    def get_history(self) -> list[Detection]:
        """Return a copy of the detection history (oldest first)."""
        return list(self.history)

    # ------------------------------------------------------------------
    # Camera helpers
    # ------------------------------------------------------------------

    @staticmethod
    def open_camera(source: int | str, width: int = 1280, height: int = 720) -> cv2.VideoCapture:
        """Open a camera or video file and configure resolution."""
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise OSError(f"Cannot open camera/video source: {source!r}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimise latency
        return cap

    @staticmethod
    def draw_detection(frame: np.ndarray, det: Detection) -> np.ndarray:
        """Overlay bounding circle and confidence on frame (in-place)."""
        r = int(det.radius())
        cx, cy = int(det.cx), int(det.cy)
        cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
        cv2.putText(
            frame,
            f"{det.confidence:.2f}",
            (cx - r, cy - r - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        return frame
