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
from pathlib import Path
from dimos.agents.agent import llm_agent
from dimos.agents.cli.human import human_input
from dimos.agents.skills.grasping import grasping_skill
from dimos.core.blueprints import autoconnect
from dimos.grasping import graspgen
from dimos.grasping.temp_graspgen_testing import grasp_pipeline
from dimos.hardware.sensors.camera.realsense import realsense_camera
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.object_scene_registration import object_scene_registration_module
from dimos.robot.foxglove_bridge import foxglove_bridge

demo_perception_grasping = autoconnect(
    camera_module,
    object_scene_registration_module(target_frame=target_frame, prompt_mode=YoloePromptMode.PROMPT),
    graspgen(
        docker_file_path=Path(__file__).parent.parent / "grasping" / "docker_context" / "Dockerfile",
        docker_build_context=Path(__file__).parent.parent.parent,  # repo root
        gripper_type="robotiq_2f_140",
        save_visualization_data=False,
        num_grasps=400,
        topk_num_grasps=100,
        filter_collisions=False,
        docker_volumes=[("/tmp", "/tmp", "rw")],
    ),
    grasp_pipeline(object_name="object"),
    grasping_skill(),
    foxglove_bridge(),
    human_input(),
    llm_agent(),
).global_config(viewer_backend="foxglove")
