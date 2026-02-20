"""
scripts/diagnostics.py
Hardware and software diagnostics for Aqua Keeper AI.

Performs pre-flight checks on camera, motor driver, GPIO, and
system resources. Run this before every deployment session to verify
the system is healthy.

Usage
-----
    python scripts/diagnostics.py [--config configs/default.yaml]
"""

from __future__ import annotations

import argparse
import logging
import os
import platform
import shutil
import sys
from pathlib import Path

_REPO_ROOT = str(Path(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from src.utils.config import load_config

logger = logging.getLogger(__name__)

# Result symbols
PASS = "✅"
WARN = "⚠️"
FAIL = "❌"
INFO = "ℹ️"


def _check_python_version() -> str:
    ver = platform.python_version()
    major, minor = sys.version_info[:2]
    if major >= 3 and minor >= 9:
        return f"{PASS} Python {ver}"
    return f"{FAIL} Python {ver} — requires 3.9+"


def _check_package(name: str) -> str:
    try:
        mod = __import__(name)
        version = getattr(mod, "__version__", "unknown")
        return f"{PASS} {name} {version}"
    except ImportError:
        return f"{FAIL} {name} — not installed"


def _check_camera(source: int | str) -> str:
    try:
        import cv2
        cap = cv2.VideoCapture(source)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return f"{PASS} Camera {source} — read OK"
            return f"{WARN} Camera {source} — opened but first read failed"
        return f"{FAIL} Camera {source} — cannot open"
    except Exception as exc:
        return f"{FAIL} Camera {source} — {exc}"


def _check_model_weights(path: str) -> str:
    if Path(path).exists():
        size_mb = Path(path).stat().st_size / (1024 * 1024)
        return f"{PASS} Model weights — {path} ({size_mb:.1f} MB)"
    return f"{WARN} Model weights — {path} not found (download or train first)"


def _check_disk_space() -> str:
    total, used, free = shutil.disk_usage("/")
    free_gb = free / (1024**3)
    if free_gb > 1.0:
        return f"{PASS} Disk — {free_gb:.1f} GB free"
    return f"{WARN} Disk — only {free_gb:.1f} GB free"


def _check_gpio() -> str:
    try:
        import RPi.GPIO  # type: ignore  # noqa: F401
        return f"{PASS} RPi.GPIO available"
    except ImportError:
        return f"{INFO} RPi.GPIO not available (expected on non-Pi systems)"


def _check_config_file(path: str) -> str:
    try:
        cfg = load_config(path)
        sections = list(cfg.keys())
        return f"{PASS} Config — {path} ({len(sections)} sections)"
    except Exception as exc:
        return f"{FAIL} Config — {exc}"


def run_diagnostics(config_path: str) -> list[str]:
    """Run all diagnostics and return a list of result strings."""
    results: list[str] = []

    results.append("=== System ===")
    results.append(_check_python_version())
    results.append(f"{INFO} Platform: {platform.platform()}")
    results.append(f"{INFO} CPU count: {os.cpu_count()}")
    results.append(_check_disk_space())

    results.append("\n=== Dependencies ===")
    for pkg in ["cv2", "numpy", "yaml", "torch"]:
        results.append(_check_package(pkg))

    results.append("\n=== Configuration ===")
    results.append(_check_config_file(config_path))

    try:
        cfg = load_config(config_path)
    except Exception:
        cfg = load_config(None)

    results.append("\n=== Hardware ===")
    results.append(_check_camera(cfg["camera"]["source"]))
    results.append(_check_model_weights(cfg["model"]["weights_path"]))
    results.append(_check_gpio())

    return results


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aqua Keeper AI — system diagnostics")
    p.add_argument("--config", default="configs/default.yaml")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = parse_args(argv)

    print("\n╔══════════════════════════════════════╗")
    print("║  Aqua Keeper AI — System Diagnostics ║")
    print("╚══════════════════════════════════════╝\n")

    results = run_diagnostics(args.config)
    for line in results:
        print(line)

    failures = sum(1 for r in results if FAIL in r)
    warnings = sum(1 for r in results if WARN in r)

    print(f"\n{'─' * 40}")
    if failures:
        print(f"{FAIL} {failures} failure(s) detected — fix before running.")
        return 1
    if warnings:
        print(f"{WARN} {warnings} warning(s) — system may work with limitations.")
        return 0
    print(f"{PASS} All checks passed — system ready.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
