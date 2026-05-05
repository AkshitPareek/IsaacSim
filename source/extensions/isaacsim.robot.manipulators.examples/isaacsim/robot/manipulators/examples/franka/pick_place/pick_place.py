# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
import os
import time
from typing import List, Optional

import cv2
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import requests
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.robot.manipulators.examples.franka.franka_experimental import FrankaExperimental
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path


OPENVLA_SERVER_URL = os.getenv("OPENVLA_SERVER_URL", "http://localhost:8000/act")
OPENVLA_INSTRUCTION = os.getenv("OPENVLA_INSTRUCTION", "pick up the blue cube and place it on the target")
OPENVLA_LOG_ENABLED = os.getenv("OPENVLA_LOG_ENABLED", "1") != "0"
OPENVLA_LOG_PATH = os.getenv("OPENVLA_LOG_PATH", "openvla_pick_place_log.csv")
OPENVLA_GRASP_ASSIST_ENABLED = os.getenv("OPENVLA_GRASP_ASSIST_ENABLED", "1") != "0"


class FrankaPickPlace:
    """Simple, direct Franka pick-and-place controller.

    No complex inheritance, no RL concepts, no task wrappers.
    Just straightforward robot control that's easy to understand and modify.
    """

    def __init__(self, events_dt: Optional[List[float]] = None):
        """Initialize the FrankaPickPlace controller.

        Sets up initial state variables for the state machine.

        Args:
            events_dt: List of step counts for each phase. If None, uses default values.
        """
        self.cube = None
        self.robot = None
        self.camera = None
        self._last_vla_action = None
        self._last_vla_event = None
        self._last_vla_step = -1
        self._vla_query_interval = 15
        self._vla_enabled = os.getenv("OPENVLA_ENABLED", "1") != "0"
        self._run_id = str(int(time.time() * 1000))
        self._log_header_written = False
        self._grasp_assist_offset = None

        # Define step counts for each phase
        self.events_dt = events_dt
        if self.events_dt is None:
            # Phase durations: [(x,y) positioning, approach, grasp, lift, move, release, retract]
            self.events_dt = [
                60,  # Phase 0: Move to x,y position above cube
                40,  # Phase 1: Approach down to cube
                20,  # Phase 2: Close gripper to grasp
                40,  # Phase 3: Lift cube upward
                140,  # Phase 4: Slowly carry cube above target location
                60,  # Phase 5: Lower cube, then open gripper to release
                20,  # Phase 6: Move up and away
            ]
        self._event = 0
        self._step = 0

    def setup_scene(
        self,
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        cube_size: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        """Set up the scene with robot and cube.

        Creates and adds a Franka robot and a dynamic cube to the simulation world.
        Sets default values for positions and sizes if not provided.

        Args:
            cube_initial_position: Initial cube position [x, y, z]. Defaults to [0.5, 0.0, 0.0258].
            cube_initial_orientation: Initial cube orientation as quaternion [w, x, y, z]. Defaults to [1, 0, 0, 0].
            cube_size: Cube dimensions [w, h, d]. Defaults to [0.0515, 0.0515, 0.0515].
            target_position: Target position for placing [x, y, z]. Defaults to [-0.3, -0.3, cube_height/2].
            offset: Additional offset to apply to target position [x, y, z]. Defaults to [0, 0, 0].
        """
        self.cube_initial_position = cube_initial_position
        self.cube_initial_orientation = cube_initial_orientation
        self.target_position = target_position
        self.cube_size = cube_size
        self.offset = offset

        if self.cube_size is None:
            self.cube_size = np.array([0.0515, 0.0515, 0.0515])
        if self.cube_initial_position is None:
            self.cube_initial_position = np.array([0.5, 0.0, 0.0258])
        if self.cube_initial_orientation is None:
            self.cube_initial_orientation = np.array([1, 0, 0, 0])
        if self.target_position is None:
            self.target_position = np.array([-0.3, -0.3, 0.12])
        if self.offset is None:
            self.offset = np.array([0.0, 0.0, 0.0])
        self.target_position = self.target_position + self.offset

        # Create a new USD stage with default sunlight lighting
        stage_utils.create_new_stage(template="sunlight")

        # Create the Franka robot controller (inherits from Articulation)
        self.robot = FrankaExperimental(robot_path="/World/robot", create_robot=True)
        self.end_effector_link = self.robot.end_effector_link

        # Add ground plane environment for physics simulation
        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        # Create blue visual material for the cube
        visual_material = PreviewSurfaceMaterial("/Visual_materials/blue")
        visual_material.set_input_values("diffuseColor", [0.0, 0.0, 1.0])

        cube_shape = Cube(
            paths="/World/Cube",
            positions=self.cube_initial_position,
            orientations=self.cube_initial_orientation,
            sizes=[1.0],
            scales=self.cube_size,
            reset_xform_op_properties=True,
        )

        GeomPrim(paths=cube_shape.paths, apply_collision_apis=True)
        self.cube = RigidPrim(paths=cube_shape.paths)
        cube_shape.apply_visual_materials(visual_material)

        self.camera = Camera(
            prim_path="/World/OpenVLACamera",
            position=np.array([0.25, 0.0, 1.35]),
            frequency=10,
            resolution=(320, 320),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
        )
        try:
            self.camera.initialize()
        except Exception as exc:
            print(f"OpenVLA camera initialization failed, using scripted reach: {exc}")
            self.camera = None

    def _get_scripted_reach_goal(self) -> np.ndarray:
        """Return the safe scripted Phase 0 reach target above the cube."""
        cube_pos = self.cube.get_world_poses()[0].numpy()
        return np.array([cube_pos[0, 0], cube_pos[0, 1], cube_pos[0, 2] + 0.2])

    def _get_scripted_place_goal(self) -> np.ndarray:
        """Return the safe scripted Phase 4 carry/place target."""
        return self.target_position + np.array([0.0, 0.0, 0.22])

    def _log_vla_guided_goal(self, phase: int, source: str, goal_position: np.ndarray) -> None:
        """Log concise VLA phase debug details at the existing VLA cadence."""
        if self._step != 0 and self._step % self._vla_query_interval != 0:
            return

        formatted_goal = np.round(np.asarray(goal_position).reshape(-1), 4)
        print(f"Phase {phase}: using {source}; end-effector goal target: {formatted_goal}")
        self._write_run_log(phase, source, goal_position)

    def _format_array_for_log(self, value: Optional[np.ndarray]) -> str:
        """Format vectors compactly for CSV cells."""
        if value is None:
            return ""
        return ";".join(f"{x:.6f}" for x in np.asarray(value).reshape(-1))

    def _get_log_cube_position(self) -> Optional[np.ndarray]:
        """Return the current cube position for run logging."""
        if self.cube is None:
            return None
        try:
            return self.cube.get_world_poses()[0].numpy().reshape(-1)[:3]
        except Exception:
            return None

    def _write_run_log(self, phase: int, source: str, goal_position: np.ndarray) -> None:
        """Append one VLA controller decision row to the CSV run log."""
        if not OPENVLA_LOG_ENABLED:
            return

        log_dir = os.path.dirname(OPENVLA_LOG_PATH)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        file_exists = os.path.exists(OPENVLA_LOG_PATH)
        write_header = not file_exists or os.path.getsize(OPENVLA_LOG_PATH) == 0
        row = {
            "run_id": self._run_id,
            "timestamp": f"{time.time():.6f}",
            "phase": phase,
            "phase_step": self._step,
            "goal_source": source,
            "vla_enabled": int(self._vla_enabled),
            "instruction": OPENVLA_INSTRUCTION,
            "vla_action": self._format_array_for_log(self._last_vla_action),
            "ee_goal": self._format_array_for_log(goal_position),
            "cube_position": self._format_array_for_log(self._get_log_cube_position()),
            "target_position": self._format_array_for_log(self.target_position),
        }

        with open(OPENVLA_LOG_PATH, mode="a", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
            self._log_header_written = True
            writer.writerow(row)

    def _get_vla_rgb_image(self) -> Optional[np.ndarray]:
        """Read the current RGB camera image for the VLA server."""
        if self.camera is None:
            return None

        try:
            rgb_image = self.camera.get_rgb(device="cpu")
        except TypeError:
            rgb_image = self.camera.get_rgb()
        except Exception as exc:
            print(f"OpenVLA camera read failed, using scripted reach: {exc}")
            return None

        if rgb_image is None or rgb_image.size == 0:
            return None
        return np.asarray(rgb_image, dtype=np.uint8)

    def _call_openvla(self, rgb_image: np.ndarray) -> Optional[np.ndarray]:
        """Send an Isaac camera frame to the OpenVLA server and return its 7D action."""
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        ok, encoded_image = cv2.imencode(".png", bgr_image)
        if not ok:
            print("OpenVLA image encoding failed, using scripted reach")
            return None

        try:
            response = requests.post(
                OPENVLA_SERVER_URL,
                data={"instruction": OPENVLA_INSTRUCTION},
                files={"image": ("camera.png", encoded_image.tobytes(), "image/png")},
                timeout=30,
            )
            response.raise_for_status()
            action = np.asarray(response.json()["action"], dtype=np.float32)
        except Exception as exc:
            print(f"OpenVLA request failed, using scripted reach: {exc}")
            return None

        if action.shape[0] < 7:
            print(f"OpenVLA returned malformed action {action}, using scripted reach")
            return None
        return action

    def _get_openvla_action(self) -> Optional[np.ndarray]:
        """Return a cached OpenVLA action, refreshing every few simulation steps."""
        if not self._vla_enabled:
            return None

        should_refresh = (
            self._last_vla_action is None
            or self._last_vla_event != self._event
            or self._step - self._last_vla_step >= self._vla_query_interval
        )
        if not should_refresh:
            return self._last_vla_action

        rgb_image = self._get_vla_rgb_image()
        if rgb_image is None:
            return self._last_vla_action

        action = self._call_openvla(rgb_image)
        if action is not None:
            self._last_vla_action = action
            self._last_vla_event = self._event
            self._last_vla_step = self._step
            print(f"OpenVLA action for phase {self._event}: {np.round(action, 4)}")
        return self._last_vla_action

    def _get_vla_reach_goal(self) -> Optional[np.ndarray]:
        """Convert the VLA translational delta into a clamped end-effector target."""
        action = self._get_openvla_action()
        if action is None:
            return None

        _, current_position, _ = self.robot.get_current_state()
        delta_position = np.clip(action[:3], -0.03, 0.03)
        goal_position = current_position[0] + delta_position

        # Keep Phase 0 high enough to avoid early table/cube contact.
        cube_pos = self.cube.get_world_poses()[0].numpy()
        goal_position[2] = max(goal_position[2], cube_pos[0, 2] + 0.12)
        return goal_position

    def _get_vla_descend_goal(self, cube_pos: np.ndarray) -> Optional[np.ndarray]:
        """Return a guarded OpenVLA-assisted Phase 1 descend target above the cube."""
        action = self._get_openvla_action()
        if action is None:
            return None

        _, current_position, _ = self.robot.get_current_state()
        cube_center = cube_pos[0]
        scripted_goal = cube_center + np.array([0.0, 0.0, 0.1])

        # Use only the VLA translation delta; gripper/action tail never controls grasping.
        vla_delta = np.clip(action[:3], -0.02, 0.02)
        vla_goal = current_position[0] + vla_delta

        # Bias strongly back to the known safe target so descent stays predictable.
        goal_position = 0.35 * vla_goal + 0.65 * scripted_goal
        xy_margin = 0.05
        goal_position[:2] = np.clip(goal_position[:2], cube_center[:2] - xy_margin, cube_center[:2] + xy_margin)
        goal_position[2] = max(goal_position[2], cube_center[2] + 0.08)
        return goal_position

    def _get_vla_place_goal(self) -> Optional[np.ndarray]:
        """Convert the VLA translational delta into a guarded Phase 4 place target."""
        action = self._get_openvla_action()
        if action is None:
            return None

        _, current_position, _ = self.robot.get_current_state()
        current_position = current_position[0]
        vla_delta = np.clip(action[:3], -0.02, 0.02)
        safe_carry_height = self.target_position[2] + 0.22
        target_above_place = self.target_position + np.array([0.0, 0.0, safe_carry_height - self.target_position[2]])
        scripted_delta = target_above_place - current_position

        # Strongly bias toward the known place target; VLA only nudges the carry path.
        if np.linalg.norm(scripted_delta) > 0.025:
            scripted_delta = 0.025 * scripted_delta / np.linalg.norm(scripted_delta)
        goal_position = current_position + 0.1 * vla_delta + 0.9 * scripted_delta

        lower_bounds = self.target_position + np.array([-0.1, -0.1, safe_carry_height])
        upper_bounds = self.target_position + np.array([0.35, 0.35, safe_carry_height])
        goal_position[:2] = np.clip(goal_position[:2], lower_bounds[:2], upper_bounds[:2])
        goal_position[2] = safe_carry_height
        return goal_position

    def _capture_grasp_assist_offset(self) -> None:
        """Measure the cube offset from the hand for prototype grasp stabilization."""
        if not OPENVLA_GRASP_ASSIST_ENABLED or self._grasp_assist_offset is not None:
            return

        _, end_effector_position, _ = self.robot.get_current_state()
        cube_position = self.cube.get_world_poses()[0].numpy()
        self._grasp_assist_offset = cube_position[0] - end_effector_position[0]
        print(f"Grasp assist captured cube offset: {np.round(self._grasp_assist_offset, 4)}")

    def _apply_grasp_assist(self) -> None:
        """Keep the grasped cube under the hand until the scripted release phase."""
        if not OPENVLA_GRASP_ASSIST_ENABLED or self._grasp_assist_offset is None:
            return

        _, end_effector_position, _ = self.robot.get_current_state()
        _, cube_orientation = self.cube.get_world_poses()
        assisted_position = end_effector_position[0] + self._grasp_assist_offset
        self.cube.set_world_poses(
            positions=assisted_position.reshape(1, -1),
            orientations=cube_orientation.numpy(),
        )

    def _release_grasp_assist(self) -> None:
        """Disable prototype cube attachment after release."""
        if self._grasp_assist_offset is not None:
            print("Grasp assist released")
        self._grasp_assist_offset = None

    def forward(self, ik_method: str = "damped-least-squares") -> bool:
        """Execute one step of the pick-and-place operation using the specified IK method.

        Args:
            ik_method: The inverse kinematics method to use. Defaults to "damped-least-squares".

        Returns:
            True if a step was executed, False if the sequence is complete.
        """
        if self.is_done():
            return False

        # Get downward-facing orientation for end-effector
        goal_orientation = self.robot.get_downward_orientation()

        # Phase 0: Move to x,y position above cube
        if self._event == 0:
            if self._step == 0:
                print("Phase 0: Moving to x,y position above cube with OpenVLA guidance...")

            goal_position = self._get_vla_reach_goal()
            if goal_position is None:
                goal_position = self._get_scripted_reach_goal()
                goal_source = "scripted fallback"
            else:
                goal_source = "VLA"
            self._log_vla_guided_goal(0, goal_source, goal_position)

            # Use the new high-level method that combines position and orientation
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[0]:
                self._event += 1
                self._step = 0

        # Phase 1: Approach down to the cube
        elif self._event == 1:
            if self._step == 0:
                print("Phase 1: Approaching cube with guarded OpenVLA guidance...")

            # Goal is above and slightly behind the cube for proper approach
            cube_pos = self.cube.get_world_poses()[0].numpy()
            goal_position = self._get_vla_descend_goal(cube_pos)
            if goal_position is None:
                goal_position = cube_pos + np.array([0.0, 0.0, 0.1])  # Approach from above with safe distance
                goal_source = "scripted fallback"
            else:
                goal_source = "VLA"
            self._log_vla_guided_goal(1, goal_source, goal_position)

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[1]:
                self._event += 1
                self._step = 0

        # Phase 2: Close gripper to grasp the cube
        elif self._event == 2:
            if self._step == 0:
                print("Phase 2: Closing gripper...")
                self._capture_grasp_assist_offset()

            # Close gripper
            self.robot.close_gripper()

            self._step += 1
            if self._step >= self.events_dt[2]:
                self._event += 1
                self._step = 0

        # Phase 3: Lift the cube
        elif self._event == 3:
            if self._step == 0:
                print("Phase 3: Lifting cube...")

            self.robot.close_gripper()

            # Get current end effector position and lift up
            _, current_position, _ = self.robot.get_current_state()
            goal_position = current_position + np.array([0.0, 0.0, 0.2])

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)
            self._apply_grasp_assist()

            self._step += 1
            if self._step >= self.events_dt[3]:
                self._event += 1
                self._step = 0

        # Phase 4: Move cube to target location
        elif self._event == 4:
            if self._step == 0:
                print("Phase 4: Moving cube with OpenVLA guidance...")

            self.robot.close_gripper()

            goal_position = self._get_vla_place_goal()
            if goal_position is None:
                goal_position = self._get_scripted_place_goal()
                goal_source = "scripted fallback"
            else:
                goal_source = "VLA"
            self._log_vla_guided_goal(4, goal_source, goal_position)

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)
            self._apply_grasp_assist()

            self._step += 1
            if self._step >= self.events_dt[4]:
                self._event += 1
                self._step = 0

        # Phase 5: Open gripper to release cube
        elif self._event == 5:
            if self._step == 0:
                print("Phase 5: Lowering cube, then opening gripper...")

            release_position = self.target_position + np.array([0.0, 0.0, 0.06])
            if self._step < int(0.75 * self.events_dt[5]):
                self.robot.close_gripper()
                self.robot.set_end_effector_pose(
                    position=release_position, orientation=goal_orientation, ik_method=ik_method
                )
                self._apply_grasp_assist()
            else:
                self._release_grasp_assist()
                self.robot.open_gripper()

            self._step += 1
            if self._step >= self.events_dt[5]:
                self._event += 1
                self._step = 0

        # Phase 6: Move up
        elif self._event == 6:
            if self._step == 0:
                print("Phase 6: Moving up...")

            # Goal is to lift up
            cube_pos = self.cube.get_world_poses()[0].numpy()
            goal_position = cube_pos + np.array([0.0, 0.0, 0.3])  # Move above the cube

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[6]:
                self._event += 1
                self._step = 0

        return True

    def is_done(self) -> bool:
        """Check if the pick-and-place sequence is complete.

        Returns:
            True if the state machine reached the last phase. Otherwise False.
        """
        if self._event >= len(self.events_dt):
            return True
        else:
            return False

    def reset(self, cube_position: Optional[np.ndarray] = None, cube_orientation: Optional[np.ndarray] = None):
        """Reset the entire pick-and-place system to initial state.

        This is the main reset function that resets both robot and cube.
        Use this for complete system reset.

        Args:
            cube_position: Optional new position for the cube. If None, uses initial position.
            cube_orientation: Optional new orientation for the cube. If None, uses initial orientation.
        """
        print("Resetting pick-and-place system...")
        self.reset_robot()
        self.reset_cube(position=cube_position, orientation=cube_orientation)
        print("Pick-and-place system reset complete")

    def reset_robot(self):
        """Reset the robot to its default state.

        Resets the robot's joint positions to the default configuration
        and resets the state machine to the beginning.
        """
        if self.robot is not None:
            # Reset robot using the controller
            self.robot.reset_to_default_pose()

            # Reset state machine
            self._event = 0
            self._step = 0
            self._last_vla_action = None
            self._last_vla_event = None
            self._last_vla_step = -1
            self._grasp_assist_offset = None

            print("Robot reset to default state")
        else:
            print("Warning: Franka controller not initialized, cannot reset")

    def reset_cube(self, position: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None):
        """Reset the cube to its initial position and orientation.

        Args:
            position: Optional new position for the cube. If None, uses initial position.
            orientation: Optional new orientation for the cube. If None, uses initial orientation.
        """
        if self.cube is not None:
            # Use provided position/orientation or fall back to initial values
            reset_position = position if position is not None else self.cube_initial_position
            reset_orientation = orientation if orientation is not None else self.cube_initial_orientation

            # Reset cube position and orientation
            self.cube.set_world_poses(
                positions=reset_position.reshape(1, -1), orientations=reset_orientation.reshape(1, -1)
            )

            print(f"Cube reset to position: {reset_position}")
        else:
            print("Warning: Cube not initialized, cannot reset")
