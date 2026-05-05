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

parser = argparse.ArgumentParser()
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="damped-least-squares",
)
parser.add_argument("--runs", type=int, default=3, help="Number of scripted runs to collect calibration data from")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import csv
import io
import os
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
        return current_position  # gripper close, EE stays put
    elif event == 3:
        _, current_position, _ = pick_place.robot.get_current_state()
        return current_position + np.array([0.0, 0.0, 0.2])
    elif event == 4:
        return pick_place.target_position.copy()
    elif event == 5:
        _, current_position, _ = pick_place.robot.get_current_state()
        return current_position  # gripper open, EE stays put
    elif event == 6:
        cube_pos2 = pick_place.cube.get_world_poses()[0].numpy()
        return cube_pos2[0] + np.array([0.0, 0.0, 0.3])
    else:
        return np.zeros(3)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"VLA Calibration Experiment")
    print(f"  Runs:        {args.runs}")
    print(f"  VLA server:  {VLA_SERVER_URL}")
    print(f"  Query every: {VLA_QUERY_EVERY} steps")
    print(f"  Log dir:     {LOG_DIR}")

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
    run_id        = int(time.time())
    current_run   = 0
    reset_needed  = True

    print(f"\nStarting calibration data collection (run 1 of {args.runs})...")

    while simulation_app.is_running():
        if not SimulationManager.is_simulating():
            simulation_app.update()
            continue

        # ── Reset ────────────────────────────────────────────────────────────
        if reset_needed:
            pick_place.reset()
            reset_needed = False
            run_id = int(time.time())
            print(f"\n[Run {current_run + 1}/{args.runs}] id={run_id}")

        # ── Record state BEFORE scripted step ────────────────────────────────
        phase      = pick_place._event
        phase_step = pick_place._step

        try:
            _, ee_pos, _ = pick_place.robot.get_current_state()
        except Exception:
            ee_pos = np.zeros(3)

        try:
            cube_pos = pick_place.cube.get_world_poses()[0].numpy()[0]
        except Exception:
            cube_pos = np.zeros(3)

        scripted_goal = get_scripted_goal(pick_place)

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