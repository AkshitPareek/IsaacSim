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

import os
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

        # Define step counts for each phase
        self.events_dt = events_dt
        if self.events_dt is None:
            # Phase durations: [(x,y) positioning, approach, grasp, lift, move, release, retract]
            self.events_dt = [
                60,  # Phase 0: Move to x,y position above cube
                40,  # Phase 1: Approach down to cube
                20,  # Phase 2: Close gripper to grasp
                40,  # Phase 3: Lift cube upward
                80,  # Phase 4: Move cube to target location
                20,  # Phase 5: Open gripper to release
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
            resolution=(224, 224),
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
        return self.target_position

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

    def _get_vla_place_goal(self) -> Optional[np.ndarray]:
        """Convert the VLA translational delta into a guarded Phase 4 place target."""
        action = self._get_openvla_action()
        if action is None:
            return None

        _, current_position, _ = self.robot.get_current_state()
        vla_delta = np.clip(action[:3], -0.03, 0.03)
        scripted_delta = self.target_position - current_position[0]

        # Blend toward the known target so the VLA can guide but not drift away.
        if np.linalg.norm(scripted_delta) > 0.03:
            scripted_delta = 0.03 * scripted_delta / np.linalg.norm(scripted_delta)
        goal_position = current_position[0] + 0.5 * vla_delta + 0.5 * scripted_delta

        lower_bounds = self.target_position + np.array([-0.25, -0.25, 0.0])
        upper_bounds = self.target_position + np.array([0.25, 0.25, 0.25])
        goal_position = np.clip(goal_position, lower_bounds, upper_bounds)
        goal_position[2] = max(goal_position[2], self.target_position[2])
        return goal_position

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

            # Use the new high-level method that combines position and orientation
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[0]:
                self._event += 1
                self._step = 0

        # Phase 1: Approach down to the cube
        elif self._event == 1:
            if self._step == 0:
                print("Phase 1: Approaching cube...")

            # Goal is above and slightly behind the cube for proper approach
            cube_pos = self.cube.get_world_poses()[0].numpy()
            goal_position = cube_pos + np.array([0.0, 0.0, 0.1])  # Approach from above with safe distance

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

            # Get current end effector position and lift up
            _, current_position, _ = self.robot.get_current_state()
            goal_position = current_position + np.array([0.0, 0.0, 0.2])

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[3]:
                self._event += 1
                self._step = 0

        # Phase 4: Move cube to target location
        elif self._event == 4:
            if self._step == 0:
                print("Phase 4: Moving cube with OpenVLA guidance...")

            goal_position = self._get_vla_place_goal()
            if goal_position is None:
                goal_position = self._get_scripted_place_goal()

            # Move to position using the controller
            self.robot.set_end_effector_pose(position=goal_position, orientation=goal_orientation, ik_method=ik_method)

            self._step += 1
            if self._step >= self.events_dt[4]:
                self._event += 1
                self._step = 0

        # Phase 5: Open gripper to release cube
        elif self._event == 5:
            if self._step == 0:
                print("Phase 5: Opening gripper...")

            # Open gripper
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
