#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demo for agentic pick-and-place with PickPlaceModule.

This demo sets up the complete grasping pipeline:
- Camera for perception
- Object detection and scene registration
- GraspGen (Docker) for neural network grasp generation
- PickPlaceModule for agent-facing pick/place skills
- LLM agent for natural language interaction

Example interaction:
    User: Can you see the cup?
    Agent: Yes, I can see a cup on the table.
    User: Pick it up.
    Agent: [picks up cup] Successfully picked the cup.
"""

from pathlib import Path

from dimos.agents.agent import llm_agent
from dimos.agents.cli.human import human_input
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense import realsense_camera
from dimos.manipulation.grasping import graspgen, pickplace_module
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.object_scene_registration import object_scene_registration_module
from dimos.robot.foxglove_bridge import foxglove_bridge

camera_module = realsense_camera(enable_pointcloud=False)

# Agent Mode Demo: Only pick/place/stop skills exposed to agent
demo_grasping = autoconnect(
    camera_module,
    object_scene_registration_module(
        target_frame="camera_color_optical_frame", prompt_mode=YoloePromptMode.PROMPT
    ),
    # GraspGen Docker module for neural network grasp generation
    graspgen(
        docker_file_path=Path(__file__).parent / "docker_context" / "Dockerfile",
        docker_build_context=Path(__file__).parent.parent.parent.parent,  # repo root
        gripper_type="robotiq_2f_140",
        num_grasps=400,
        topk_num_grasps=100,
        filter_collisions=False,
        save_visualization_data=False,
        docker_volumes=[("/tmp", "/tmp", "rw")],
    ),
    # PickPlace module: agent-facing pick/place skills with BT orchestration
    # Gripper control routes through ManipulationModule -> ControlCoordinator
    pickplace_module(
        lift_height=0.10,  # 10cm lift after grasp
    ),
    foxglove_bridge(),
    human_input(),
    llm_agent(),
).global_config(viewer_backend="foxglove")
