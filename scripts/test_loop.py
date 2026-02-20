"""
scripts/test_loop.py
Simulated end-to-end test loop — exercises the controller and actuator
with synthetic ball trajectories (no camera or model required).

Usage
-----
    # Run a straight-shot scenario
    python scripts/test_loop.py --scenario straight

    # Run a curved (sine) shot
    python scripts/test_loop.py --scenario sine

    # Latency benchmark
    python scripts/test_loop.py --scenario latency_benchmark --frames 500

    # Accuracy benchmark (built-in trajectories)
    python scripts/test_loop.py --scenario accuracy

    # Replay a recorded JSONL log
    python scripts/test_loop.py --replay logs/run_20240101_120000.jsonl
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Optional

_REPO_ROOT = str(Path(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from src.control.actuator import Actuator
from src.control.controller import BlockerController
from src.utils.config import load_config

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Trajectory generators
# ---------------------------------------------------------------------------


def _gen_straight(
    goal_width: float,
    start_x: Optional[float] = None,
    n_frames: int = 60,
    depth_start: float = 3.0,
    speed: float = 10.0,
) -> list[tuple[float, float, float]]:
    """Ball travelling in a straight line toward the goal centre."""
    if start_x is None:
        start_x = goal_width * 0.7  # off-centre shot
    depth_per_frame = depth_start / n_frames
    trajectory = []
    for i in range(n_frames):
        z = depth_start - i * depth_per_frame
        x = start_x  # constant lateral position (straight shot)
        y = 0.5
        trajectory.append((x, y, z))
    return trajectory


def _gen_sine(
    goal_width: float,
    n_frames: int = 60,
    depth_start: float = 3.0,
) -> list[tuple[float, float, float]]:
    """Ball following a sinusoidal lateral path."""
    trajectory = []
    for i in range(n_frames):
        z = depth_start * (1.0 - i / n_frames)
        x = goal_width / 2.0 + (goal_width / 2.5) * math.sin(2 * math.pi * i / n_frames)
        y = 0.5
        trajectory.append((x, y, z))
    return trajectory


def _gen_lob(
    goal_width: float,
    n_frames: int = 80,
    depth_start: float = 4.0,
) -> list[tuple[float, float, float]]:
    """Lobbed ball with parabolic vertical component."""
    trajectory = []
    for i in range(n_frames):
        t = i / n_frames
        z = depth_start * (1.0 - t)
        x = goal_width * 0.3 + goal_width * 0.4 * t
        y = 0.9 * (4 * t * (1.0 - t))  # parabola, max height at t=0.5
        trajectory.append((x, y, z))
    return trajectory


_SCENARIO_GENERATORS = {
    "straight": _gen_straight,
    "sine": _gen_sine,
    "lob": _gen_lob,
}

# Built-in accuracy test trajectories (start_x, expected_intercept_x)
_ACCURACY_SHOTS: list[tuple[float, float]] = [
    (0.2, 0.2),
    (0.5, 0.5),
    (1.0, 1.0),
    (1.5, 1.5),
    (2.0, 2.0),
    (2.2, 2.2),
    (0.1, 0.1),
    (1.8, 1.8),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _simulate_scenario(
    trajectory: list[tuple[float, float, float]],
    controller: BlockerController,
    actuator: Actuator,
    dt: float = 0.033,
) -> dict:
    """Step through a trajectory and return stats."""
    t = time.monotonic()
    latencies: list[float] = []
    final_blocker_pos: float = actuator.get_position()

    for ball_x, ball_y, ball_z in trajectory:
        t0 = time.monotonic()
        blocker_pos = actuator.get_position()
        target, t_impact = controller.update(ball_x, ball_y, ball_z, blocker_pos, t)
        actual = actuator.move_to(target, t)
        t1 = time.monotonic()
        latencies.append((t1 - t0) * 1000.0)
        t += dt
        final_blocker_pos = actual

    ball_x_final = trajectory[-1][0]
    error_m = abs(final_blocker_pos - ball_x_final)
    return {
        "mean_latency_ms": sum(latencies) / max(len(latencies), 1),
        "p95_latency_ms": sorted(latencies)[int(0.95 * len(latencies))],
        "final_blocker_pos": final_blocker_pos,
        "final_ball_x": ball_x_final,
        "error_m": error_m,
    }


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------


def run_scenario(scenario: str, cfg: dict, n_frames: int) -> None:
    goal_width = cfg["geometry"]["goal_width"]
    controller = BlockerController(
        goal_width=goal_width,
        goal_height=cfg["geometry"]["goal_height"],
        pid_kp=cfg["control"]["kp"],
        pid_ki=cfg["control"]["ki"],
        pid_kd=cfg["control"]["kd"],
    )
    actuator = Actuator(
        backend="mock",
        goal_width=goal_width,
        min_pos=cfg["motor"]["min_pos"],
        max_pos=cfg["motor"]["max_pos"],
        max_velocity=cfg["motor"]["max_velocity"],
        max_acceleration=cfg["motor"]["max_acceleration"],
    )

    if scenario not in _SCENARIO_GENERATORS:
        logger.error("Unknown scenario: %s. Choose from: %s", scenario, list(_SCENARIO_GENERATORS))
        return

    gen = _SCENARIO_GENERATORS[scenario]
    trajectory = gen(goal_width, n_frames=n_frames)
    stats = _simulate_scenario(trajectory, controller, actuator)

    print(f"\n=== Scenario: {scenario} ({n_frames} frames) ===")
    print(f"  Mean latency:       {stats['mean_latency_ms']:.3f} ms")
    print(f"  P95 latency:        {stats['p95_latency_ms']:.3f} ms")
    print(f"  Final blocker pos:  {stats['final_blocker_pos']:.3f} m")
    print(f"  Final ball x:       {stats['final_ball_x']:.3f} m")
    print(f"  Position error:     {stats['error_m']:.3f} m")


def run_latency_benchmark(cfg: dict, n_frames: int) -> None:
    goal_width = cfg["geometry"]["goal_width"]
    controller = BlockerController(goal_width=goal_width)
    actuator = Actuator(backend="mock", goal_width=goal_width)

    trajectory = _gen_straight(goal_width, n_frames=n_frames)
    stats = _simulate_scenario(trajectory, controller, actuator)

    print(f"\n=== Latency Benchmark ({n_frames} frames) ===")
    print(f"  Mean latency:  {stats['mean_latency_ms']:.4f} ms")
    print(f"  P95 latency:   {stats['p95_latency_ms']:.4f} ms")
    print(f"  Target P95:    < 5.0 ms (control only, no inference)")


def run_accuracy(cfg: dict) -> None:
    goal_width = cfg["geometry"]["goal_width"]
    successes = 0
    errors: list[float] = []
    paddle_radius = 0.10  # 10 cm

    for start_x, expected_x in _ACCURACY_SHOTS:
        controller = BlockerController(goal_width=goal_width)
        actuator = Actuator(backend="mock", goal_width=goal_width)

        trajectory = _gen_straight(goal_width, start_x=start_x, n_frames=60)
        stats = _simulate_scenario(trajectory, controller, actuator)
        err = abs(stats["final_blocker_pos"] - expected_x)
        errors.append(err)
        if err <= paddle_radius:
            successes += 1

    rate = 100.0 * successes / len(_ACCURACY_SHOTS)
    mean_err = sum(errors) / len(errors)
    print(f"\n=== Accuracy Test ({len(_ACCURACY_SHOTS)} shots) ===")
    print(f"  Intercept rate: {rate:.1f}%  (target ≥ 85%)")
    print(f"  Mean error:     {mean_err:.3f} m")
    print(f"  Max error:      {max(errors):.3f} m")


def run_replay(replay_path: str, cfg: dict) -> None:
    goal_width = cfg["geometry"]["goal_width"]
    controller = BlockerController(goal_width=goal_width)
    actuator = Actuator(backend="mock", goal_width=goal_width)

    log_file = Path(replay_path)
    if not log_file.exists():
        logger.error("Log file not found: %s", replay_path)
        return

    print(f"\n=== Replaying: {replay_path} ===")
    n = 0
    with open(log_file) as f:
        for line in f:
            record = json.loads(line)
            if record.get("msg") != "detection":
                continue
            # Mock z depth (not stored in log)
            ball_x = record.get("cx", 0.0)  # raw pixels in log; scale
            cam_w = cfg["camera"]["width"]
            ball_x_m = (ball_x / cam_w) * goal_width
            blocker_pos = actuator.get_position()
            target, _ = controller.update(ball_x_m, 0.5, 1.0, blocker_pos)
            actuator.move_to(target)
            n += 1

    print(f"  Replayed {n} detection events.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Aqua Keeper AI — simulation test loop")
    p.add_argument("--config", default="configs/default.yaml")
    p.add_argument(
        "--scenario",
        choices=list(_SCENARIO_GENERATORS.keys()) + ["latency_benchmark", "accuracy"],
        default="straight",
    )
    p.add_argument("--frames", type=int, default=60)
    p.add_argument("--replay", default=None, help="Path to a JSONL log file to replay")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = parse_args(argv)
    cfg = load_config(args.config)

    if args.replay:
        run_replay(args.replay, cfg)
    elif args.scenario == "latency_benchmark":
        run_latency_benchmark(cfg, args.frames)
    elif args.scenario == "accuracy":
        run_accuracy(cfg)
    else:
        run_scenario(args.scenario, cfg, args.frames)

    return 0


if __name__ == "__main__":
    sys.exit(main())
