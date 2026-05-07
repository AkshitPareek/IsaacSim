# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# VLA Calibration Experiment for Franka Pick-Place
#
# Purpose: Run the scripted pick-place controller UNMODIFIED.
# At each step, also query the OpenVLA server and log:
#   - phase, step
#   - camera image (saved to disk)
#   - scripted end-effector goal (ground truth)
#   - scripted action applied
#   - VLA raw output
#   - cube position, target position
#
# This data is used to compute the correct action normalization
# for this Isaac Sim Franka setup, before any VLA control is applied.
#
# Usage:
#   Set OPENVLA_SERVER_URL env var (default: http://localhost:8000/act)
#   Set OPENVLA_LOG_DIR env var (default: ./vla_calib_logs)
#   Set OPENVLA_TARGET_LABELS env var (default: target)
#   Set OPENVLA_TARGET_LABEL_MODE env var (default: random; random, cycle, balanced)
#   Set OPENVLA_RUNS_PER_LABEL env var (balanced mode only)
#   Set OPENVLA_INSTRUCTION_TEMPLATE env var (default: pick up the blue cube and place it on the {target_label})
#   Set OPENVLA_QUERY_EVERY env var (default: 10, query VLA every N steps)
#   Set OPENVLA_ENABLED_PHASES env var (optional comma-separated VLA query phase allowlist)
#   Set OPENVLA_ENABLED env var (default: 1)
#   Set OPENVLA_TIMEOUT env var (default: 15 seconds)
#   Set OPENVLA_SAVE_IMAGES env var (default: 1)
#   Set OPENVLA_SAVE_IMAGE_EVERY env var (default: 1, save every sampled frame)
#   Set OPENVLA_MAX_IMAGE_SAVES env var (default: 0, unlimited)
#   Set OPENVLA_CAMERA_RESOLUTION env var (default: 320,320)
#   Set ISAACSIM_HEADLESS or OPENVLA_HEADLESS env var (default: 0)
#   Set OPENVLA_DRY_RUN env var (default: 0, save/log frames but skip HTTP)
#   Set OPENVLA_ADAPTER_CONFIG env var to an affine adapter JSON for dry-run goal logging
#   Set OPENVLA_ADAPTER_DRY_RUN env var (default: 1 when adapter config is supplied)
#   Set OPENVLA_ADAPTER_MAX_DELTA env var (default: 0.08 meters, reject larger dry-run deltas)
#   Set OPENVLA_ADAPTER_ENABLED_PHASES env var (optional comma-separated phase allowlist)
#
#   .\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares

from __future__ import annotations

import argparse
import json
import os

parser = argparse.ArgumentParser()
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="damped-least-squares",
)
parser.add_argument("--runs", type=int, default=3, help="Number of scripted runs to collect calibration data from")
parser.add_argument("--seed", type=int, default=None, help="Seed for randomized cube/target sampling")
parser.add_argument(
    "--headless",
    type=int,
    choices=[0, 1],
    default=None,
    help="Run Isaac Sim headless; defaults to ISAACSIM_HEADLESS/OPENVLA_HEADLESS or 0",
)
parser.add_argument("--cube-x-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--cube-y-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--target-x-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--target-y-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--openvla-enabled", type=int, choices=[0, 1], default=None, help="Enable OpenVLA HTTP queries")
parser.add_argument("--openvla-timeout", type=float, default=None, help="OpenVLA HTTP timeout in seconds")
parser.add_argument("--openvla-save-images", type=int, choices=[0, 1], default=None, help="Save calibration camera frames")
parser.add_argument("--openvla-save-image-every", type=int, default=None, help="Save every Nth sampled frame")
parser.add_argument("--openvla-max-image-saves", type=int, default=None, help="Maximum sampled frames to save; 0 means unlimited")
parser.add_argument("--camera-resolution", type=int, nargs=2, default=None, metavar=("WIDTH", "HEIGHT"))
parser.add_argument("--openvla-dry-run", type=int, choices=[0, 1], default=None, help="Log state/images but skip OpenVLA HTTP")
parser.add_argument(
    "--openvla-enabled-phases",
    type=str,
    default=None,
    help="Optional comma-separated phase allowlist for VLA image sampling/HTTP queries",
)
parser.add_argument("--target-labels", type=str, default=None, help="Comma-separated language labels to sample per run")
parser.add_argument(
    "--target-label-mode",
    type=str,
    choices=["random", "cycle", "balanced"],
    default=None,
    help="How to assign target labels across runs; default is random",
)
parser.add_argument(
    "--runs-per-label",
    type=int,
    default=None,
    help="For balanced mode, collect this many runs for each target label",
)
parser.add_argument(
    "--instruction-template",
    type=str,
    default=None,
    help="Instruction template; supports {target_label} and {target}",
)
parser.add_argument(
    "--min-cube-target-distance",
    type=float,
    default=None,
    help="Minimum XY distance between randomized cube and target positions",
)
parser.add_argument(
    "--adapter-config",
    type=str,
    default=None,
    help="Optional affine adapter config JSON for observer-only dry-run goal logging",
)
parser.add_argument(
    "--adapter-dry-run",
    type=int,
    choices=[0, 1],
    default=None,
    help="Compute/log adapter goals from VLA actions without controlling the robot",
)
parser.add_argument(
    "--adapter-max-delta",
    type=float,
    default=None,
    help="Maximum affine adapter delta norm accepted for dry-run safety logging",
)
parser.add_argument(
    "--adapter-enabled-phases",
    type=str,
    default=None,
    help="Optional comma-separated phase allowlist for adapter dry-run readiness/candidate logging",
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

def parse_bool_env(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "y", "on"}


def configured_headless() -> bool:
    if args.headless is not None:
        return bool(args.headless)
    return parse_bool_env("ISAACSIM_HEADLESS", parse_bool_env("OPENVLA_HEADLESS", False))


EXPERIENCE = os.environ.get("ISAACSIM_PYTHON_EXPERIENCE", "")
HEADLESS = configured_headless()
if EXPERIENCE:
    print(f"Using experience: {EXPERIENCE}")
    simulation_app = SimulationApp({"headless": HEADLESS}, experience=EXPERIENCE)
else:
    simulation_app = SimulationApp({"headless": HEADLESS})

import csv
import io
import time
from pathlib import Path

import numpy as np
import omni.timeline
import requests
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.manipulators.examples.franka import FrankaPickPlace
from isaacsim.sensors.camera import Camera

# ── Config from environment ──────────────────────────────────────────────────
VLA_SERVER_URL  = os.environ.get("OPENVLA_SERVER_URL", "http://localhost:8000/act")
VLA_QUERY_EVERY = int(os.environ.get("OPENVLA_QUERY_EVERY", "10"))
LOG_DIR         = Path(os.environ.get("OPENVLA_LOG_DIR", "./vla_calib_logs"))
LOG_DIR.mkdir(parents=True, exist_ok=True)

CSV_PATH   = LOG_DIR / "calibration.csv"
IMAGE_DIR  = LOG_DIR / "images"
IMAGE_DIR.mkdir(parents=True, exist_ok=True)

DEFAULT_CUBE_X_RANGE = (0.40, 0.60)
DEFAULT_CUBE_Y_RANGE = (-0.18, 0.18)
DEFAULT_TARGET_X_RANGE = (-0.42, -0.20)
DEFAULT_TARGET_Y_RANGE = (-0.42, -0.18)
DEFAULT_MIN_CUBE_TARGET_DISTANCE = 0.35
DEFAULT_TARGET_LABELS = ("target",)
DEFAULT_INSTRUCTION_TEMPLATE = "pick up the blue cube and place it on the {target_label}"
DEFAULT_TARGET_LABEL_MODE = "random"
DEFAULT_SCRIPTED_STEPS_PER_RUN = 280
DEFAULT_CAMERA_RESOLUTION = (320, 320)

CSV_FIELDS = [
    "run_id", "phase", "phase_step", "global_step",
    "target_label", "instruction",
    "scripted_ee_goal_x", "scripted_ee_goal_y", "scripted_ee_goal_z",
    "ee_pos_x", "ee_pos_y", "ee_pos_z",
    "cube_x", "cube_y", "cube_z",
    "target_x", "target_y", "target_z",
    "vla_queried",
    "vla_latency_ms",
    "vla_dx", "vla_dy", "vla_dz", "vla_droll", "vla_dpitch", "vla_dyaw", "vla_gripper",
    "vla_error",
    "adapter_ready",
    "adapter_accepted",
    "adapter_rejected_reason",
    "adapter_goal_x", "adapter_goal_y", "adapter_goal_z",
    "adapter_delta_x", "adapter_delta_y", "adapter_delta_z",
    "adapter_delta_norm",
    "adapter_error_to_scripted",
    "image_path",
]

# ── CSV writer ────────────────────────────────────────────────────────────────
def open_csv():
    write_header = not CSV_PATH.exists() or CSV_PATH.stat().st_size == 0
    f = open(CSV_PATH, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    if write_header:
        writer.writeheader()
    return f, writer


# ── Camera setup ─────────────────────────────────────────────────────────────
def setup_camera(resolution: tuple[int, int]):
    """Add a fixed overhead camera looking at the workspace."""
    cam = Camera(
        prim_path="/World/CalibCamera",
        position=np.array([0.5, 0.0, 1.2]),
        orientation=np.array([0.0, 0.7071068, 0.7071068, 0.0]),  # looking down
        resolution=resolution,
    )
    try:
        cam.initialize()
        print(f"Calibration camera initialized at {resolution[0]}x{resolution[1]}")
    except Exception as e:
        print(f"Camera init warning (may resolve after first frame): {e}")
    return cam


# ── Scene randomization ──────────────────────────────────────────────────────
def env_float_pair(name: str, default: tuple[float, float]) -> tuple[float, float]:
    raw = os.environ.get(name)
    if not raw:
        return default
    parts = [part.strip() for part in raw.replace(",", " ").split() if part.strip()]
    if len(parts) != 2:
        raise ValueError(f"{name} must contain exactly two float values")
    return validate_range(name, (float(parts[0]), float(parts[1])))


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else float(raw)


def env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in {"1", "true", "yes", "y", "on"}


def configured_flag(arg_value: int | None, env_name: str, default: bool) -> bool:
    if arg_value is not None:
        return bool(arg_value)
    return env_flag(env_name, default)


def configured_timeout() -> float:
    timeout = args.openvla_timeout if args.openvla_timeout is not None else env_float("OPENVLA_TIMEOUT", 15.0)
    if timeout <= 0:
        raise ValueError("OPENVLA_TIMEOUT must be positive")
    return timeout


def env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    return default if raw is None or raw == "" else int(raw)


def env_int_pair(name: str, default: tuple[int, int]) -> tuple[int, int]:
    raw = os.environ.get(name)
    if not raw:
        return default
    parts = [part.strip() for part in raw.replace(",", " ").split() if part.strip()]
    if len(parts) != 2:
        raise ValueError(f"{name} must contain exactly two integer values")
    return int(parts[0]), int(parts[1])


def configured_positive_int(arg_value: int | None, env_name: str, default: int) -> int:
    value = arg_value if arg_value is not None else env_int(env_name, default)
    if value <= 0:
        raise ValueError(f"{env_name} must be positive")
    return value


def configured_non_negative_int(arg_value: int | None, env_name: str, default: int) -> int:
    value = arg_value if arg_value is not None else env_int(env_name, default)
    if value < 0:
        raise ValueError(f"{env_name} must be non-negative")
    return value


def configured_camera_resolution() -> tuple[int, int]:
    if args.camera_resolution is not None:
        resolution = (int(args.camera_resolution[0]), int(args.camera_resolution[1]))
    else:
        resolution = env_int_pair("OPENVLA_CAMERA_RESOLUTION", DEFAULT_CAMERA_RESOLUTION)
    if resolution[0] <= 0 or resolution[1] <= 0:
        raise ValueError("OPENVLA_CAMERA_RESOLUTION values must be positive")
    return resolution


def configured_adapter_config_path() -> Path | None:
    raw = args.adapter_config if args.adapter_config is not None else os.environ.get("OPENVLA_ADAPTER_CONFIG", "")
    if raw is None or raw.strip() == "":
        return None
    return Path(raw)


def configured_adapter_max_delta() -> float:
    max_delta = (
        args.adapter_max_delta
        if args.adapter_max_delta is not None
        else env_float("OPENVLA_ADAPTER_MAX_DELTA", 0.08)
    )
    if max_delta <= 0:
        raise ValueError("OPENVLA_ADAPTER_MAX_DELTA must be positive")
    return max_delta


def parse_adapter_enabled_phases(raw: str | None) -> set[int] | None:
    if raw is None or raw.strip() == "":
        return None
    phases = set()
    for part in raw.replace(",", " ").split():
        phase = int(part)
        if phase < 0:
            raise ValueError("Adapter enabled phases must be non-negative integers")
        phases.add(phase)
    if not phases:
        raise ValueError("Adapter enabled phases must include at least one phase")
    return phases


def configured_adapter_enabled_phases() -> set[int] | None:
    raw = (
        args.adapter_enabled_phases
        if args.adapter_enabled_phases is not None
        else os.environ.get("OPENVLA_ADAPTER_ENABLED_PHASES")
    )
    return parse_adapter_enabled_phases(raw)


def configured_vla_enabled_phases() -> set[int] | None:
    raw = (
        args.openvla_enabled_phases
        if args.openvla_enabled_phases is not None
        else os.environ.get("OPENVLA_ENABLED_PHASES", os.environ.get("OPENVLA_QUERY_PHASES", ""))
    )
    return parse_adapter_enabled_phases(raw)


def validate_range(name: str, values: tuple[float, float]) -> tuple[float, float]:
    low, high = values
    if not np.isfinite(low) or not np.isfinite(high):
        raise ValueError(f"{name} values must be finite")
    if low > high:
        raise ValueError(f"{name} min must be <= max")
    return low, high


def configured_range(arg_value, env_name: str, default: tuple[float, float]) -> tuple[float, float]:
    if arg_value is not None:
        return validate_range(env_name, (float(arg_value[0]), float(arg_value[1])))
    return env_float_pair(env_name, default)


def configured_seed() -> int | None:
    if args.seed is not None:
        return args.seed
    raw = os.environ.get("OPENVLA_RANDOM_SEED")
    return None if raw is None or raw == "" else int(raw)


def parse_labels(raw: str | None, default: tuple[str, ...]) -> tuple[str, ...]:
    if raw is None or raw.strip() == "":
        return default
    labels = tuple(label.strip() for label in raw.split(",") if label.strip())
    if not labels:
        raise ValueError("Target labels must include at least one non-empty label")
    return labels


def configured_target_labels() -> tuple[str, ...]:
    raw = args.target_labels if args.target_labels is not None else os.environ.get("OPENVLA_TARGET_LABELS")
    return parse_labels(raw, DEFAULT_TARGET_LABELS)


def configured_target_label_mode() -> str:
    mode = args.target_label_mode or os.environ.get("OPENVLA_TARGET_LABEL_MODE", DEFAULT_TARGET_LABEL_MODE)
    if mode not in {"random", "cycle", "balanced"}:
        raise ValueError("Target label mode must be one of: random, cycle, balanced")
    return mode


def configured_runs_per_label() -> int | None:
    if args.runs_per_label is not None:
        value = args.runs_per_label
    else:
        raw = os.environ.get("OPENVLA_RUNS_PER_LABEL")
        value = None if raw is None or raw == "" else int(raw)
    if value is not None and value <= 0:
        raise ValueError("Runs per label must be positive")
    return value


def configured_instruction_template() -> str:
    if args.instruction_template is not None:
        template = args.instruction_template
    else:
        template = os.environ.get(
            "OPENVLA_INSTRUCTION_TEMPLATE",
            os.environ.get("OPENVLA_INSTRUCTION", DEFAULT_INSTRUCTION_TEMPLATE),
        )
    if template.strip() == "":
        raise ValueError("Instruction template must be non-empty")
    return template


def build_instruction(template: str, target_label: str) -> str:
    try:
        return template.format(target_label=target_label, target=target_label)
    except KeyError as e:
        raise ValueError(f"Unsupported instruction template field: {e}") from e


def load_adapter_config(path: Path | None) -> dict[str, object] | None:
    if path is None:
        return None
    with path.open(encoding="utf-8") as file:
        config = json.load(file)
    if not isinstance(config, dict) or not isinstance(config.get("phases"), list):
        raise ValueError(f"Adapter config must contain a phases list: {path}")
    print(f"Loaded OpenVLA adapter config: {path}")
    return config


def adapter_phase_config(config: dict[str, object], phase: int) -> dict[str, object] | None:
    phases = config.get("phases", [])
    if not isinstance(phases, list):
        return None
    phase_key = str(phase)
    for phase_config in phases:
        if isinstance(phase_config, dict) and str(phase_config.get("phase")) == phase_key:
            return phase_config
    return None


def compute_adapter_delta(phase_config: dict[str, object], vla_action: np.ndarray) -> np.ndarray | None:
    if not bool(phase_config.get("ready_for_control", False)):
        return None
    coefficients = phase_config.get("coefficients", {})
    if not isinstance(coefficients, dict):
        raise ValueError("Adapter phase config is missing coefficients")

    features = {
        "bias": 1.0,
        "vla_dx": float(vla_action[0]),
        "vla_dy": float(vla_action[1]),
        "vla_dz": float(vla_action[2]),
    }
    delta = []
    for axis in ("x", "y", "z"):
        axis_coefficients = coefficients.get(axis)
        if not isinstance(axis_coefficients, dict):
            raise ValueError(f"Adapter phase config is missing {axis} coefficients")
        delta.append(
            sum(float(axis_coefficients.get(name, 0.0)) * value for name, value in features.items())
        )
    return np.array(delta, dtype=np.float32)


def sample_target_label(rng: np.random.Generator, labels: tuple[str, ...]) -> str:
    return str(rng.choice(labels))


def build_target_label_plan(
    labels: tuple[str, ...],
    mode: str,
    runs: int,
    runs_per_label: int | None,
) -> tuple[str | None, ...]:
    if runs <= 0:
        raise ValueError("Number of runs must be positive")
    if mode == "random":
        return tuple(None for _ in range(runs))

    if mode == "balanced" and runs_per_label is not None:
        return tuple(label for label in labels for _ in range(runs_per_label))

    return tuple(labels[i % len(labels)] for i in range(runs))


def estimate_vla_samples(total_runs: int) -> int:
    return int(np.ceil((total_runs * DEFAULT_SCRIPTED_STEPS_PER_RUN) / max(VLA_QUERY_EVERY, 1)))


def estimate_vla_samples_for_phases(total_runs: int, enabled_phases: set[int] | None) -> int:
    if enabled_phases is None:
        return estimate_vla_samples(total_runs)
    per_run_steps = sum(
        steps
        for phase, steps in enumerate((60, 40, 20, 40, 80, 20, 20))
        if phase in enabled_phases
    )
    return int(np.ceil((total_runs * per_run_steps) / max(VLA_QUERY_EVERY, 1)))


def should_save_sampled_image(sample_index: int, save_every: int, max_saves: int, saved_count: int) -> bool:
    if (sample_index - 1) % save_every != 0:
        return False
    return max_saves == 0 or saved_count < max_saves


def sample_xy(rng: np.random.Generator, x_range: tuple[float, float], y_range: tuple[float, float]) -> np.ndarray:
    return np.array(
        [
            rng.uniform(x_range[0], x_range[1]),
            rng.uniform(y_range[0], y_range[1]),
        ],
        dtype=np.float32,
    )


def sample_scene_positions(
    rng: np.random.Generator,
    cube_x_range: tuple[float, float],
    cube_y_range: tuple[float, float],
    target_x_range: tuple[float, float],
    target_y_range: tuple[float, float],
    min_distance: float,
    target_z: float,
) -> tuple[np.ndarray, np.ndarray]:
    for _ in range(100):
        cube_xy = sample_xy(rng, cube_x_range, cube_y_range)
        target_xy = sample_xy(rng, target_x_range, target_y_range)
        if np.linalg.norm(cube_xy - target_xy) >= min_distance:
            cube_position = np.array([cube_xy[0], cube_xy[1], 0.0258], dtype=np.float32)
            target_position = np.array([target_xy[0], target_xy[1], target_z], dtype=np.float32)
            return cube_position, target_position
    raise RuntimeError(
        "Could not sample cube/target positions with the configured minimum distance; "
        "widen the ranges or reduce OPENVLA_MIN_CUBE_TARGET_DISTANCE."
    )


# ── VLA query ─────────────────────────────────────────────────────────────────
def query_vla(rgb_image: np.ndarray, instruction: str, timeout: float):
    """
    Send image to OpenVLA server. Returns (action_array, error_str, latency_ms).
    action_array is [dx, dy, dz, droll, dpitch, dyaw, gripper] or None.
    """
    start_time = time.perf_counter()
    try:
        import cv2
        ok, buf = cv2.imencode(".png", cv2.cvtColor(rgb_image.astype(np.uint8), cv2.COLOR_RGB2BGR))
        if not ok:
            return None, "cv2 encode failed", ""
        response = requests.post(
            VLA_SERVER_URL,
            data={"instruction": instruction},
            files={"image": ("camera.png", io.BytesIO(buf.tobytes()), "image/png")},
            timeout=timeout,
        )
        latency_ms = (time.perf_counter() - start_time) * 1000.0
        response.raise_for_status()
        payload = response.json()
        if "action" not in payload:
            return None, "response missing action field", f"{latency_ms:.3f}"
        action = np.array(payload["action"], dtype=np.float32).reshape(-1)
        if action.size != 7:
            return None, f"expected 7D action, got {action.size}D", f"{latency_ms:.3f}"
        return action, "", f"{latency_ms:.3f}"
    except Exception as e:
        latency_ms = (time.perf_counter() - start_time) * 1000.0
        return None, str(e), f"{latency_ms:.3f}"


def as_vec3(value) -> np.ndarray:
    """Normalize Isaac pose outputs to a flat xyz vector."""
    arr = np.asarray(value, dtype=np.float32)
    if arr.size < 3:
        return np.zeros(3, dtype=np.float32)
    return arr.reshape(-1)[:3]


# ── Scripted goal extraction ──────────────────────────────────────────────────
def get_scripted_goal(pick_place: FrankaPickPlace) -> np.ndarray:
    """
    Replicate the scripted controller's goal position for the current phase.
    This is ground truth: what position the robot is being commanded to.
    """
    event = pick_place._event
    cube_pos = pick_place.cube.get_world_poses()[0].numpy()

    if event == 0:
        return np.array([cube_pos[0, 0], cube_pos[0, 1], cube_pos[0, 2] + 0.2])
    elif event == 1:
        return cube_pos[0] + np.array([0.0, 0.0, 0.1])
    elif event == 2:
        _, current_position, _ = pick_place.robot.get_current_state()
        return as_vec3(current_position)  # gripper close, EE stays put
    elif event == 3:
        _, current_position, _ = pick_place.robot.get_current_state()
        return as_vec3(current_position) + np.array([0.0, 0.0, 0.2])
    elif event == 4:
        return pick_place.target_position.copy()
    elif event == 5:
        _, current_position, _ = pick_place.robot.get_current_state()
        return as_vec3(current_position)  # gripper open, EE stays put
    elif event == 6:
        cube_pos2 = pick_place.cube.get_world_poses()[0].numpy()
        return cube_pos2[0] + np.array([0.0, 0.0, 0.3])
    else:
        return np.zeros(3)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    if VLA_QUERY_EVERY <= 0:
        raise ValueError("OPENVLA_QUERY_EVERY must be positive")
    seed = configured_seed()
    rng = np.random.default_rng(seed)
    vla_enabled = configured_flag(args.openvla_enabled, "OPENVLA_ENABLED", True)
    vla_save_images = configured_flag(args.openvla_save_images, "OPENVLA_SAVE_IMAGES", True)
    vla_save_image_every = configured_positive_int(
        args.openvla_save_image_every,
        "OPENVLA_SAVE_IMAGE_EVERY",
        1,
    )
    vla_max_image_saves = configured_non_negative_int(
        args.openvla_max_image_saves,
        "OPENVLA_MAX_IMAGE_SAVES",
        0,
    )
    vla_dry_run = configured_flag(args.openvla_dry_run, "OPENVLA_DRY_RUN", False)
    vla_timeout = configured_timeout()
    vla_enabled_phases = configured_vla_enabled_phases()
    camera_resolution = configured_camera_resolution()
    adapter_config = load_adapter_config(configured_adapter_config_path())
    adapter_dry_run = configured_flag(args.adapter_dry_run, "OPENVLA_ADAPTER_DRY_RUN", adapter_config is not None)
    adapter_max_delta = configured_adapter_max_delta()
    adapter_enabled_phases = configured_adapter_enabled_phases()
    cube_x_range = configured_range(args.cube_x_range, "OPENVLA_CUBE_X_RANGE", DEFAULT_CUBE_X_RANGE)
    cube_y_range = configured_range(args.cube_y_range, "OPENVLA_CUBE_Y_RANGE", DEFAULT_CUBE_Y_RANGE)
    target_x_range = configured_range(args.target_x_range, "OPENVLA_TARGET_X_RANGE", DEFAULT_TARGET_X_RANGE)
    target_y_range = configured_range(args.target_y_range, "OPENVLA_TARGET_Y_RANGE", DEFAULT_TARGET_Y_RANGE)
    target_labels = configured_target_labels()
    target_label_mode = configured_target_label_mode()
    runs_per_label = configured_runs_per_label()
    target_label_plan = build_target_label_plan(target_labels, target_label_mode, args.runs, runs_per_label)
    total_runs = len(target_label_plan)
    instruction_template = configured_instruction_template()
    min_distance = (
        args.min_cube_target_distance
        if args.min_cube_target_distance is not None
        else env_float("OPENVLA_MIN_CUBE_TARGET_DISTANCE", DEFAULT_MIN_CUBE_TARGET_DISTANCE)
    )
    if min_distance < 0:
        raise ValueError("Minimum cube-target distance must be non-negative")

    estimated_vla_samples = estimate_vla_samples_for_phases(total_runs, vla_enabled_phases)
    estimated_http_calls = 0 if (not vla_enabled or vla_dry_run) else estimated_vla_samples
    if adapter_dry_run and adapter_config is None:
        print("Adapter dry-run requested but no adapter config was supplied; adapter fields will be blank.")
    elif adapter_config is not None and not adapter_dry_run:
        print("Adapter config supplied but adapter dry-run is disabled; adapter fields will be blank.")

    print(f"VLA Calibration Experiment")
    print(f"  Headless:    {int(HEADLESS)}")
    print(f"  Runs:        {total_runs}")
    print(f"  VLA enabled: {int(vla_enabled)}")
    print(f"  VLA server:  {VLA_SERVER_URL}")
    print(f"  Query every: {VLA_QUERY_EVERY} steps")
    print(
        "  VLA phases:  "
        + (
            "all"
            if vla_enabled_phases is None
            else ", ".join(str(phase) for phase in sorted(vla_enabled_phases))
        )
    )
    print(f"  Est. samples:{estimated_vla_samples}")
    print(f"  Est. HTTP:   {estimated_http_calls}")
    print(f"  Timeout:     {vla_timeout:g} seconds")
    print(f"  Save images: {int(vla_save_images)}")
    print(f"  Save every:  {vla_save_image_every} sampled frame(s)")
    print(f"  Max images:  {vla_max_image_saves if vla_max_image_saves else 'unlimited'}")
    print(f"  Camera res:  {camera_resolution[0]}x{camera_resolution[1]}")
    print(f"  Dry run:     {int(vla_dry_run)}")
    print(f"  Adapter dry: {int(adapter_dry_run)}")
    print(f"  Adapter max delta: {adapter_max_delta:g} m")
    print(
        "  Adapter phases: "
        + (
            "all ready phases"
            if adapter_enabled_phases is None
            else ", ".join(str(phase) for phase in sorted(adapter_enabled_phases))
        )
    )
    print(f"  Log dir:     {LOG_DIR}")
    print(f"  Random seed: {seed if seed is not None else 'system entropy'}")
    print(f"  Cube x/y:    {cube_x_range} / {cube_y_range}")
    print(f"  Target x/y:  {target_x_range} / {target_y_range}")
    print(f"  Min distance:{min_distance}")
    print(f"  Labels:      {', '.join(target_labels)}")
    print(f"  Label mode:  {target_label_mode}")
    if runs_per_label is not None:
        print(f"  Runs/label:  {runs_per_label}")
    print(f"  Template:    {instruction_template}")
    if target_label_mode != "random":
        planned_labels = ", ".join(
            f"run{i + 1:03d}={label}" for i, label in enumerate(target_label_plan)
        )
        print(f"  Label plan:  {planned_labels}")

    SimulationManager.set_physics_sim_device(args.device)
    simulation_app.update()

    pick_place = FrankaPickPlace()
    pick_place.setup_scene()

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    camera = setup_camera(camera_resolution)
    simulation_app.update()

    csv_file, writer = open_csv()

    global_step   = 0
    session_id    = str(int(time.time() * 1000))
    run_id        = f"{session_id}_run000"
    current_run   = 0
    reset_needed  = True
    current_target_label = ""
    current_instruction = build_instruction(instruction_template, target_labels[0])
    sampled_vla_frames = 0
    http_queries = 0
    images_saved = 0
    vla_errors = 0

    print(f"\nStarting calibration data collection (run 1 of {total_runs})...")

    while simulation_app.is_running():
        if not SimulationManager.is_simulating():
            simulation_app.update()
            continue

        # ── Reset ────────────────────────────────────────────────────────────
        if reset_needed:
            cube_position, target_position = sample_scene_positions(
                rng,
                cube_x_range,
                cube_y_range,
                target_x_range,
                target_y_range,
                min_distance,
                float(pick_place.target_position[2]),
            )
            pick_place.target_position = target_position
            pick_place.reset(cube_position=cube_position)
            simulation_app.update()
            planned_label = target_label_plan[current_run]
            current_target_label = (
                sample_target_label(rng, target_labels)
                if planned_label is None
                else planned_label
            )
            current_instruction = build_instruction(instruction_template, current_target_label)
            try:
                cube_readback = as_vec3(pick_place.cube.get_world_poses()[0].numpy()[0])
            except Exception:
                cube_readback = np.zeros(3, dtype=np.float32)
            reset_needed = False
            run_id = f"{session_id}_run{current_run + 1:03d}"
            print(f"\n[Run {current_run + 1}/{total_runs}] id={run_id}")
            print(f"  sampled cube:   {np.round(cube_position, 4)}")
            print(f"  readback cube:  {np.round(cube_readback, 4)}")
            print(f"  sampled target: {np.round(target_position, 4)}")
            print(f"  target label:   {current_target_label}")
            print(f"  instruction:    {current_instruction}")

        # ── Record state BEFORE scripted step ────────────────────────────────
        phase      = pick_place._event
        phase_step = pick_place._step

        try:
            _, ee_pos, _ = pick_place.robot.get_current_state()
            ee_pos = as_vec3(ee_pos)
        except Exception:
            ee_pos = np.zeros(3)

        try:
            cube_pos = pick_place.cube.get_world_poses()[0].numpy()[0]
        except Exception:
            cube_pos = np.zeros(3)

        scripted_goal = as_vec3(get_scripted_goal(pick_place))

        # ── Decide whether to query VLA this step ────────────────────────────
        should_sample_vla = (
            vla_enabled
            and (global_step % VLA_QUERY_EVERY == 0)
            and (phase < len(pick_place.events_dt))
            and (vla_enabled_phases is None or phase in vla_enabled_phases)
        )
        do_vla_query = should_sample_vla and not vla_dry_run
        vla_action   = None
        vla_error    = ""
        vla_latency_ms = ""
        image_path   = ""
        adapter_ready = ""
        adapter_accepted = ""
        adapter_rejected_reason = ""
        adapter_goal = None
        adapter_delta = None
        adapter_delta_norm = ""
        adapter_error_to_scripted = ""

        if should_sample_vla:
            sampled_vla_frames += 1
            try:
                rgb = camera.get_rgb()
                if rgb is not None and rgb.size > 0:
                    if vla_save_images and should_save_sampled_image(
                        sampled_vla_frames,
                        vla_save_image_every,
                        vla_max_image_saves,
                        images_saved,
                    ):
                        fname = IMAGE_DIR / f"run{run_id}_ph{phase}_s{phase_step:04d}.png"
                        import cv2
                        cv2.imwrite(str(fname), cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2BGR))
                        image_path = str(fname)
                        images_saved += 1

                    if vla_dry_run:
                        print(f"  [ph{phase} s{phase_step:3d}] VLA dry run: HTTP skipped")
                    else:
                        http_queries += 1
                        vla_action, vla_error, vla_latency_ms = query_vla(rgb, current_instruction, vla_timeout)
                        if vla_action is not None:
                            print(
                                f"  [ph{phase} s{phase_step:3d}] VLA "
                                f"({vla_latency_ms} ms): {np.round(vla_action, 4)}"
                            )
                        else:
                            print(
                                f"  [ph{phase} s{phase_step:3d}] VLA error "
                                f"({vla_latency_ms} ms): {vla_error[:60]}"
                            )
                            vla_errors += 1
                else:
                    vla_error = "empty camera frame"
                    vla_errors += 1
            except Exception as e:
                vla_error = str(e)
                vla_errors += 1

        if adapter_dry_run and adapter_config is not None and vla_action is not None:
            phase_config = adapter_phase_config(adapter_config, phase)
            if phase_config is not None:
                phase_enabled = adapter_enabled_phases is None or phase in adapter_enabled_phases
                adapter_ready = int(phase_enabled and bool(phase_config.get("ready_for_control", False)))
                if not phase_enabled:
                    adapter_rejected_reason = "phase disabled by adapter phase allowlist"
                elif not adapter_ready:
                    adapter_rejected_reason = "phase not ready for control"
                else:
                    adapter_delta = compute_adapter_delta(phase_config, vla_action)
                if adapter_delta is not None:
                    adapter_delta_norm = float(np.linalg.norm(adapter_delta))
                    adapter_goal = ee_pos + adapter_delta
                    adapter_error_to_scripted = float(np.linalg.norm(adapter_goal - scripted_goal))
                    adapter_accepted = int(adapter_delta_norm <= adapter_max_delta)
                    if not adapter_accepted:
                        adapter_rejected_reason = (
                            f"delta_norm {adapter_delta_norm:.4f} > max {adapter_max_delta:.4f}"
                        )
                    print(
                        f"  [ph{phase} s{phase_step:3d}] adapter dry-run goal "
                        f"{np.round(adapter_goal, 4)} "
                        f"(error to scripted {adapter_error_to_scripted:.4f} m, "
                        f"accepted={adapter_accepted})"
                    )

        # ── Log row ───────────────────────────────────────────────────────────
        row = {
            "run_id":            run_id,
            "phase":             phase,
            "phase_step":        phase_step,
            "global_step":       global_step,
            "target_label":      current_target_label,
            "instruction":       current_instruction,
            "scripted_ee_goal_x": scripted_goal[0],
            "scripted_ee_goal_y": scripted_goal[1],
            "scripted_ee_goal_z": scripted_goal[2],
            "ee_pos_x":          ee_pos[0],
            "ee_pos_y":          ee_pos[1],
            "ee_pos_z":          ee_pos[2],
            "cube_x":            cube_pos[0],
            "cube_y":            cube_pos[1],
            "cube_z":            cube_pos[2],
            "target_x":          pick_place.target_position[0],
            "target_y":          pick_place.target_position[1],
            "target_z":          pick_place.target_position[2],
            "vla_queried":       int(do_vla_query),
            "vla_latency_ms":    vla_latency_ms,
            "vla_dx":            vla_action[0] if vla_action is not None else "",
            "vla_dy":            vla_action[1] if vla_action is not None else "",
            "vla_dz":            vla_action[2] if vla_action is not None else "",
            "vla_droll":         vla_action[3] if vla_action is not None else "",
            "vla_dpitch":        vla_action[4] if vla_action is not None else "",
            "vla_dyaw":          vla_action[5] if vla_action is not None else "",
            "vla_gripper":       vla_action[6] if vla_action is not None else "",
            "vla_error":         vla_error,
            "adapter_ready":      adapter_ready,
            "adapter_accepted":   adapter_accepted,
            "adapter_rejected_reason": adapter_rejected_reason,
            "adapter_goal_x":     adapter_goal[0] if adapter_goal is not None else "",
            "adapter_goal_y":     adapter_goal[1] if adapter_goal is not None else "",
            "adapter_goal_z":     adapter_goal[2] if adapter_goal is not None else "",
            "adapter_delta_x":    adapter_delta[0] if adapter_delta is not None else "",
            "adapter_delta_y":    adapter_delta[1] if adapter_delta is not None else "",
            "adapter_delta_z":    adapter_delta[2] if adapter_delta is not None else "",
            "adapter_delta_norm": adapter_delta_norm,
            "adapter_error_to_scripted": adapter_error_to_scripted,
            "image_path":        image_path,
        }
        writer.writerow(row)
        csv_file.flush()

        # ── Execute scripted step ─────────────────────────────────────────────
        pick_place.forward(args.ik_method)
        global_step += 1

        # ── Check completion ──────────────────────────────────────────────────
        if pick_place.is_done():
            current_run += 1
            print(f"Run {current_run} complete.")
            if current_run >= total_runs:
                print(f"\nAll {total_runs} calibration runs complete.")
                print("Run summary:")
                print(f"  Sampled frames: {sampled_vla_frames}")
                print(f"  HTTP queries:   {http_queries}")
                print(f"  VLA errors:     {vla_errors}")
                print(f"  Images saved:   {images_saved}")
                print(f"CSV saved to: {CSV_PATH}")
                print(f"Images saved to: {IMAGE_DIR}")
                break
            reset_needed = True

        simulation_app.update()

    csv_file.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        simulation_app.close()
