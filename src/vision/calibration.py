"""
src/vision/calibration.py
Camera intrinsic calibration (checkerboard) and goal-coordinate homography.

Usage
-----
# Intrinsic calibration
python src/vision/calibration.py --mode intrinsic --board 9x6 --square 0.025 \
    --output configs/camera_intrinsics.yaml

# Extrinsic / homography calibration
python src/vision/calibration.py --mode extrinsic \
    --intrinsics configs/camera_intrinsics.yaml \
    --goal-width 2.4 --goal-height 0.9 \
    --output configs/goal_homography.yaml
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Intrinsic calibration
# ---------------------------------------------------------------------------


def calibrate_intrinsics(
    images_or_capture: list[np.ndarray] | cv2.VideoCapture,
    board_size: tuple[int, int] = (9, 6),
    square_size_m: float = 0.025,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute camera matrix and distortion coefficients from checkerboard images.

    Parameters
    ----------
    images_or_capture : list of BGR frames or an open VideoCapture
    board_size : inner corners (cols, rows)
    square_size_m : side length of one square in metres

    Returns
    -------
    camera_matrix, dist_coeffs, rvecs, tvecs
    """
    cols, rows = board_size
    obj_point = np.zeros((rows * cols, 3), np.float32)
    obj_point[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m

    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    img_shape: Optional[tuple[int, int]] = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    frames: list[np.ndarray]
    if isinstance(images_or_capture, cv2.VideoCapture):
        frames = _collect_frames_interactive(images_or_capture)
    else:
        frames = images_or_capture

    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]  # (w, h)
        ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(obj_point)
            img_points.append(corners_refined)
            logger.info("Checkerboard found (%d total)", len(obj_points))
        else:
            logger.debug("Checkerboard NOT found in frame")

    if len(obj_points) < 5:
        raise RuntimeError(
            f"Only {len(obj_points)} usable frames; need at least 5 for reliable calibration."
        )

    assert img_shape is not None
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None
    )
    rms = ret
    logger.info("Calibration RMS reprojection error: %.4f px", rms)
    return camera_matrix, dist_coeffs, rvecs, tvecs


def _collect_frames_interactive(cap: cv2.VideoCapture, target: int = 20) -> list[np.ndarray]:
    """Show live feed; user presses SPACE to capture, Q to quit."""
    frames: list[np.ndarray] = []
    print(f"Press SPACE to capture (need {target}), Q to quit.")
    while len(frames) < target:
        ret, frame = cap.read()
        if not ret:
            break
        display = frame.copy()
        cv2.putText(
            display,
            f"Captured: {len(frames)}/{target}  SPACE=capture  Q=quit",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 200, 0),
            2,
        )
        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            frames.append(frame.copy())
            print(f"  Captured frame {len(frames)}")
        elif key == ord("q"):
            break
    cv2.destroyAllWindows()
    return frames


# ---------------------------------------------------------------------------
# Goal homography
# ---------------------------------------------------------------------------


class GoalMapper:
    """
    Maps image pixel coordinates to goal physical coordinates (metres).

    The homography H is computed from 4 user-clicked goal-corner correspondences.

    Goal coordinate system:
      (0, 0) = bottom-left post
      (goal_width, 0) = bottom-right post
      (0, goal_height) = top-left bar
    """

    def __init__(self, H: np.ndarray, goal_width: float, goal_height: float) -> None:
        self._H = H
        self._H_inv = np.linalg.inv(H)
        self.goal_width = goal_width
        self.goal_height = goal_height

    @classmethod
    def from_correspondences(
        cls,
        image_pts: np.ndarray,
        goal_width: float,
        goal_height: float,
    ) -> "GoalMapper":
        """
        Build a GoalMapper from 4 image-space points corresponding to the
        four goal corners.

        image_pts : shape (4, 2) in pixel (u, v)
        Order: bottom-left, bottom-right, top-right, top-left
        """
        w, h = goal_width, goal_height
        goal_pts = np.array(
            [[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32
        )
        image_pts_f = np.array(image_pts, dtype=np.float32)
        H, _ = cv2.findHomography(image_pts_f, goal_pts)
        if H is None:
            raise RuntimeError("findHomography failed — check your corner points.")
        return cls(H, goal_width, goal_height)

    def pixel_to_goal(self, u: float, v: float) -> tuple[float, float]:
        """Convert pixel (u, v) → goal (x, y) in metres."""
        pt = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, self._H)
        return float(dst[0, 0, 0]), float(dst[0, 0, 1])

    def goal_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """Convert goal (x, y) in metres → pixel (u, v)."""
        pt = np.array([[[x, y]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, self._H_inv)
        return float(dst[0, 0, 0]), float(dst[0, 0, 1])

    def to_dict(self) -> dict:
        return {
            "H": self._H.tolist(),
            "goal_width": self.goal_width,
            "goal_height": self.goal_height,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "GoalMapper":
        H = np.array(d["H"], dtype=np.float64)
        return cls(H, d["goal_width"], d["goal_height"])


def interactive_goal_corners(cap: cv2.VideoCapture) -> np.ndarray:
    """Let user click the 4 goal corners on a live frame. Returns (4,2) array."""
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Cannot read from camera for corner selection.")

    pts: list[tuple[int, int]] = []
    labels = ["bottom-left", "bottom-right", "top-right", "top-left"]
    clone = frame.copy()

    def _on_mouse(event: int, x: int, y: int, flags: int, param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))
            cv2.circle(clone, (x, y), 6, (0, 0, 255), -1)
            cv2.imshow("Goal corners", clone)

    cv2.imshow("Goal corners", clone)
    cv2.setMouseCallback("Goal corners", _on_mouse)
    print("Click the 4 goal corners in order:", labels)

    while len(pts) < 4:
        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            raise RuntimeError("Corner selection aborted by user.")

    cv2.destroyAllWindows()
    return np.array(pts, dtype=np.float32)


# ---------------------------------------------------------------------------
# Persistence helpers
# ---------------------------------------------------------------------------


def save_intrinsics(path: str, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> None:
    data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
    }
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(data, f)
    logger.info("Saved intrinsics to %s", path)


def load_intrinsics(path: str) -> tuple[np.ndarray, np.ndarray]:
    with open(path) as f:
        data = yaml.safe_load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def save_goal_mapper(path: str, mapper: GoalMapper) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(mapper.to_dict(), f)
    logger.info("Saved goal homography to %s", path)


def load_goal_mapper(path: str) -> GoalMapper:
    with open(path) as f:
        data = yaml.safe_load(f)
    return GoalMapper.from_dict(data)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aqua Keeper AI — camera calibration")
    p.add_argument("--mode", choices=["intrinsic", "extrinsic"], required=True)
    p.add_argument("--board", default="9x6", help="Inner corners e.g. 9x6")
    p.add_argument("--square", type=float, default=0.025, help="Square side in metres")
    p.add_argument("--camera", type=int, default=0, help="Camera device index")
    p.add_argument("--intrinsics", default="configs/camera_intrinsics.yaml")
    p.add_argument("--goal-width", type=float, default=2.4)
    p.add_argument("--goal-height", type=float, default=0.9)
    p.add_argument("--output", required=True)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = _parse_args(argv)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        logger.error("Cannot open camera %d", args.camera)
        return 1

    try:
        if args.mode == "intrinsic":
            cols, rows = (int(x) for x in args.board.split("x"))
            frames = _collect_frames_interactive(cap)
            K, D, _, _ = calibrate_intrinsics(frames, (cols, rows), args.square)
            save_intrinsics(args.output, K, D)

        elif args.mode == "extrinsic":
            corners = interactive_goal_corners(cap)
            mapper = GoalMapper.from_correspondences(
                corners, args.goal_width, args.goal_height
            )
            save_goal_mapper(args.output, mapper)

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
