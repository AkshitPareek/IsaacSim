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
#   Set OPENVLA_INSTRUCTION env var (default: pick up the blue cube and place it on the target)
#   Set OPENVLA_QUERY_EVERY env var (default: 10, query VLA every N steps)
#
#   .\python.bat standalone_examples\api\isaacsim.robot.manipulators\franka\vla_pick_place.py --device cuda --ik-method damped-least-squares

from __future__ import annotations

import argparse
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
parser.add_argument("--cube-x-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--cube-y-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--target-x-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument("--target-y-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"))
parser.add_argument(
    "--min-cube-target-distance",
    type=float,
    default=None,
    help="Minimum XY distance between randomized cube and target positions",
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

EXPERIENCE = os.environ.get("ISAACSIM_PYTHON_EXPERIENCE", "")
if EXPERIENCE:
    print(f"Using experience: {EXPERIENCE}")
    simulation_app = SimulationApp({"headless": False}, experience=EXPERIENCE)
else:
    simulation_app = SimulationApp({"headless": False})

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
VLA_INSTRUCTION = os.environ.get("OPENVLA_INSTRUCTION", "pick up the blue cube and place it on the target")
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

CSV_FIELDS = [
    "run_id", "phase", "phase_step", "global_step",
    "scripted_ee_goal_x", "scripted_ee_goal_y", "scripted_ee_goal_z",
    "ee_pos_x", "ee_pos_y", "ee_pos_z",
    "cube_x", "cube_y", "cube_z",
    "target_x", "target_y", "target_z",
    "vla_queried",
    "vla_dx", "vla_dy", "vla_dz", "vla_droll", "vla_dpitch", "vla_dyaw", "vla_gripper",
    "vla_error",
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
def setup_camera():
    """Add a fixed overhead camera looking at the workspace."""
    cam = Camera(
        prim_path="/World/CalibCamera",
        position=np.array([0.5, 0.0, 1.2]),
        orientation=np.array([0.0, 0.7071068, 0.7071068, 0.0]),  # looking down
        resolution=(320, 320),
    )
    try:
        cam.initialize()
        print("Calibration camera initialized at 320x320")
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
def query_vla(rgb_image: np.ndarray, instruction: str):
    """
    Send image to OpenVLA server. Returns (action_array, error_str).
    action_array is [dx, dy, dz, droll, dpitch, dyaw, gripper] or None.
    """
    try:
        import cv2
        ok, buf = cv2.imencode(".png", cv2.cvtColor(rgb_image.astype(np.uint8), cv2.COLOR_RGB2BGR))
        if not ok:
            return None, "cv2 encode failed"
        response = requests.post(
            VLA_SERVER_URL,
            data={"instruction": instruction},
            files={"image": ("camera.png", io.BytesIO(buf.tobytes()), "image/png")},
            timeout=15,
        )
        response.raise_for_status()
        action = np.array(response.json()["action"], dtype=np.float32)
        return action, ""
    except Exception as e:
        return None, str(e)


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
    seed = configured_seed()
    rng = np.random.default_rng(seed)
    cube_x_range = configured_range(args.cube_x_range, "OPENVLA_CUBE_X_RANGE", DEFAULT_CUBE_X_RANGE)
    cube_y_range = configured_range(args.cube_y_range, "OPENVLA_CUBE_Y_RANGE", DEFAULT_CUBE_Y_RANGE)
    target_x_range = configured_range(args.target_x_range, "OPENVLA_TARGET_X_RANGE", DEFAULT_TARGET_X_RANGE)
    target_y_range = configured_range(args.target_y_range, "OPENVLA_TARGET_Y_RANGE", DEFAULT_TARGET_Y_RANGE)
    min_distance = (
        args.min_cube_target_distance
        if args.min_cube_target_distance is not None
        else env_float("OPENVLA_MIN_CUBE_TARGET_DISTANCE", DEFAULT_MIN_CUBE_TARGET_DISTANCE)
    )
    if min_distance < 0:
        raise ValueError("Minimum cube-target distance must be non-negative")

    print(f"VLA Calibration Experiment")
    print(f"  Runs:        {args.runs}")
    print(f"  VLA server:  {VLA_SERVER_URL}")
    print(f"  Query every: {VLA_QUERY_EVERY} steps")
    print(f"  Log dir:     {LOG_DIR}")
    print(f"  Random seed: {seed if seed is not None else 'system entropy'}")
    print(f"  Cube x/y:    {cube_x_range} / {cube_y_range}")
    print(f"  Target x/y:  {target_x_range} / {target_y_range}")
    print(f"  Min distance:{min_distance}")

    SimulationManager.set_physics_sim_device(args.device)
    simulation_app.update()

    pick_place = FrankaPickPlace()
    pick_place.setup_scene()

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    camera = setup_camera()
    simulation_app.update()

    csv_file, writer = open_csv()

    global_step   = 0
    session_id    = str(int(time.time() * 1000))
    run_id        = f"{session_id}_run000"
    current_run   = 0
    reset_needed  = True

    print(f"\nStarting calibration data collection (run 1 of {args.runs})...")

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
            try:
                cube_readback = as_vec3(pick_place.cube.get_world_poses()[0].numpy()[0])
            except Exception:
                cube_readback = np.zeros(3, dtype=np.float32)
            reset_needed = False
            run_id = f"{session_id}_run{current_run + 1:03d}"
            print(f"\n[Run {current_run + 1}/{args.runs}] id={run_id}")
            print(f"  sampled cube:   {np.round(cube_position, 4)}")
            print(f"  readback cube:  {np.round(cube_readback, 4)}")
            print(f"  sampled target: {np.round(target_position, 4)}")

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
        do_vla_query = (global_step % VLA_QUERY_EVERY == 0) and (phase < len(pick_place.events_dt))
        vla_action   = None
        vla_error    = ""
        image_path   = ""

        if do_vla_query:
            try:
                rgb = camera.get_rgb()
                if rgb is not None and rgb.size > 0:
                    # Save image
                    fname = IMAGE_DIR / f"run{run_id}_ph{phase}_s{phase_step:04d}.png"
                    import cv2
                    cv2.imwrite(str(fname), cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2BGR))
                    image_path = str(fname)

                    # Query VLA
                    vla_action, vla_error = query_vla(rgb, VLA_INSTRUCTION)
                    if vla_action is not None:
                        print(f"  [ph{phase} s{phase_step:3d}] VLA: {np.round(vla_action, 4)}")
                    else:
                        print(f"  [ph{phase} s{phase_step:3d}] VLA error: {vla_error[:60]}")
                else:
                    vla_error = "empty camera frame"
            except Exception as e:
                vla_error = str(e)

        # ── Log row ───────────────────────────────────────────────────────────
        row = {
            "run_id":            run_id,
            "phase":             phase,
            "phase_step":        phase_step,
            "global_step":       global_step,
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
            "vla_dx":            vla_action[0] if vla_action is not None else "",
            "vla_dy":            vla_action[1] if vla_action is not None else "",
            "vla_dz":            vla_action[2] if vla_action is not None else "",
            "vla_droll":         vla_action[3] if vla_action is not None else "",
            "vla_dpitch":        vla_action[4] if vla_action is not None else "",
            "vla_dyaw":          vla_action[5] if vla_action is not None else "",
            "vla_gripper":       vla_action[6] if vla_action is not None else "",
            "vla_error":         vla_error,
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
            if current_run >= args.runs:
                print(f"\nAll {args.runs} calibration runs complete.")
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
