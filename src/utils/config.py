"""
src/utils/config.py
Configuration loader — reads YAML and returns a plain dict.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


_DEFAULTS: dict[str, Any] = {
    "camera": {
        "source": 0,
        "width": 1280,
        "height": 720,
        "homography_path": "configs/goal_homography.yaml",
    },
    "model": {
        "weights_path": "models/yolov8n_ball.pt",
        "backend": "pytorch",
        "confidence_threshold": 0.40,
        "smoothing_alpha": 0.40,
        "history_len": 10,
        "imgsz": 640,
        "device": "cpu",
    },
    "geometry": {
        "goal_width": 2.4,
        "goal_height": 0.9,
        "default_ball_depth": 1.0,
    },
    "control": {
        "kp": 5.0,
        "ki": 0.1,
        "kd": 0.05,
        "min_observations": 3,
    },
    "motor": {
        "backend": "mock",
        "min_pos": 0.0,
        "max_pos": 2.4,
        "max_velocity": 4.0,
        "max_acceleration": 20.0,
        "estop_pin": None,
        "zero_offset_steps": 0,
        "steps_per_metre": 3200.0,
        "servo_channel": 0,
    },
    "logging": {
        "log_dir": "logs",
        "level": "INFO",
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into a copy of *base*."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_config(path: str | None = None) -> dict[str, Any]:
    """
    Load configuration from a YAML file, merging with built-in defaults.

    Parameters
    ----------
    path : str or None
        Path to a YAML config file. If None, returns the defaults.

    Returns
    -------
    dict with fully populated configuration.
    """
    cfg = copy.deepcopy(_DEFAULTS)

    if path is not None:
        config_file = Path(path)
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {path!r}")
        with open(config_file) as f:
            user_cfg = yaml.safe_load(f) or {}
        cfg = _deep_merge(cfg, user_cfg)

    return cfg
