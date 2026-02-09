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

"""Blueprints for agentic grasping with PickPlaceModule.

Provides two modes:
1. Agent Mode: Only high-level skills (pick, place, stop) exposed to LLM agent
2. Debug Mode: Full low-level access for development and debugging

The VLM/agent only calls high-level skills in Agent Mode. All low-level operations
(graspgen, plan, execute, gripper) are internal to PickPlaceModule.

Usage:
    # Agent Mode (production)
    from dimos.manipulation.grasping.grasping_blueprints import grasping_agent_mode
    bp = grasping_agent_mode.build()
    bp.loop()

    # Debug Mode (development)
    from dimos.manipulation.grasping.grasping_blueprints import grasping_debug_mode
    bp = grasping_debug_mode.build()
    bp.loop()
"""

from pathlib import Path

from dimos.agents.agent import llm_agent
from dimos.agents.cli.human import human_input
from dimos.control.blueprints import coordinator_xarm6
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense import realsense_camera
from dimos.manipulation.grasping import graspgen, pickplace_module
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.object_scene_registration import object_scene_registration_module
from dimos.robot.foxglove_bridge import foxglove_bridge

# =============================================================================
# Shared Components
# =============================================================================


def _graspgen_component():
    """GraspGen Docker module for neural network grasp generation."""
    return graspgen(
        docker_file_path=Path(__file__).parent / "docker_context" / "Dockerfile",
        docker_build_context=Path(__file__).parent.parent.parent.parent,  # repo root
        gripper_type="robotiq_2f_140",
        num_grasps=400,
        topk_num_grasps=100,
        filter_collisions=False,
        save_visualization_data=False,
        docker_volumes=[("/tmp", "/tmp", "rw")],
    )


def _pickplace_component(robot_name: str | None = None):
    """PickPlace module for agent-facing pick/place skills."""
    return pickplace_module(
        robot_name=robot_name,
        lift_height=0.10,
    )


def _perception_components():
    """Camera and object detection components."""
    return [
        realsense_camera(enable_pointcloud=False),
        object_scene_registration_module(
            target_frame="camera_color_optical_frame",
            prompt_mode=YoloePromptMode.PROMPT,
        ),
    ]


# =============================================================================
# Agent Mode Blueprint
# =============================================================================

# Agent Mode: Only pick/place/stop skills exposed
# The LLM agent can only call high-level skills, not low-level operations
grasping_agent_mode = autoconnect(
    *_perception_components(),
    _graspgen_component(),
    _pickplace_component(),
    foxglove_bridge(),
    human_input(),
    llm_agent(),
).global_config(viewer_backend="foxglove")


# =============================================================================
# Debug Mode Blueprint
# =============================================================================

# Debug Mode: Full low-level access for development
# Includes ManipulationModule and ControlCoordinator for direct access
grasping_debug_mode = autoconnect(
    *_perception_components(),
    _graspgen_component(),
    _pickplace_component(),
    # Additional debug components
    manipulation_module(
        planning_timeout=10.0,
        enable_viz=True,
    ),
    foxglove_bridge(),
    human_input(),
    llm_agent(),
).global_config(viewer_backend="foxglove")


# =============================================================================
# Hardware-Specific Blueprints
# =============================================================================


def xarm6_grasping_agent():
    """XArm6 grasping with PickPlaceModule in Agent Mode."""
    return autoconnect(
        *_perception_components(),
        _graspgen_component(),
        _pickplace_component(robot_name="xarm6"),
        coordinator_xarm6,  # Hardware coordinator
        foxglove_bridge(),
        human_input(),
        llm_agent(),
    ).global_config(viewer_backend="foxglove")


__all__ = [
    "grasping_agent_mode",
    "grasping_debug_mode",
    "xarm6_grasping_agent",
]
