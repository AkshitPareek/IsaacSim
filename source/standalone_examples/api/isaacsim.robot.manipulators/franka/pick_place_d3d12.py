# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="damped-least-squares",
    help="Differential inverse kinematics method",
)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

d3d12_experience = os.path.join(os.environ["EXP_PATH"], "isaacsim.exp.base.python.d3d12.kit")
simulation_app = SimulationApp({"headless": False}, experience=d3d12_experience)

import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.manipulators.examples.franka import FrankaPickPlace


def main():
    print("Starting Simple Franka Pick-and-Place Demo (D3D12)")
    print(f"Using experience: {d3d12_experience}")
    SimulationManager.set_physics_sim_device(args.device)
    simulation_app.update()

    pick_place = FrankaPickPlace()
    pick_place.setup_scene()

    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    reset_needed = True
    task_completed = False

    print("Starting pick-and-place execution")
    while simulation_app.is_running():
        if SimulationManager.is_simulating() and not task_completed:
            if reset_needed:
                pick_place.reset()
                reset_needed = False

            pick_place.forward(args.ik_method)

        if pick_place.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True

        simulation_app.update()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        simulation_app.close()
