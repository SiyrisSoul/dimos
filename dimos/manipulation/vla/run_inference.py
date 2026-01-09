# Copyright 2026 Dimensional Inc.
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

""" 
Given a hosted policy server tuned for a particular robot, this script will get observations from camera and joint positions, 
query the policy server for actions, and send the actions to the robot.
"""

import os
import threading
import time
import argparse

import numpy as np
from openpi_client import websocket_client_policy

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image, JointCommand, JointState
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat
from dimos.msgs.sensor_msgs import JointCommand
from dimos.manipulation.vla.utils import get_camera_image, get_joint_positions, get_joint_velocities

ACTION_HORIZON = 15
ACTION_CHUNK = None
GRIPPER_CHUNK = None

# xArm7 joint limits in degrees (lower, upper)
XARM7_JOINT_LIMITS_DEG = [
    (-360.0, 360.0),
    (-118.0, 118.0),
    (-360.0, 360.0),
    (-233.0, 11.0),
    (-360.0, 360.0),
    (-97.0, 180.0),
    (-360.0, 360.0),
]


def get_observation(wrist_cam_topic, exterior_cam_topic, joint_state_topic):
    return {
        "observation/exterior_image_1_left": get_camera_image(exterior_cam_topic),  # ADD SECOND CAMERA IN BLUEPRINT DEFINED WITH SERIAL NUMBER
        "observation/wrist_image_left": get_camera_image(wrist_cam_topic),
        "observation/joint_position": get_joint_positions(joint_state_topic),
        "observation/gripper_position": 0.0,
        "prompt": "move the arm slightly to the left",
    }


def run_inference(wrist_cam_topic: str, exterior_cam_topic: str, joint_state_topic: str, joint_cmd_topic: str):
    """
    Run inference loop until user interrupts
    """
    actions_from_chunk_completed = 0
    joint_cmd_pub = LCMTransport(joint_cmd_topic, JointCommand)

    while True:
        if actions_from_chunk_completed == 0 or actions_from_chunk_completed >= ACTION_HORIZON:
            actions_from_chunk_completed = 0
            observation = get_observation(wrist_cam_topic, exterior_cam_topic, joint_state_topic)

            result = policy.infer(observation)
            action_chunk = result["actions"]  # Shape: (15, 8) - these are VELOCITY COMMANDS

            dt = 1.0 / 15.0
            action_chunk = action_chunk.copy()
            action_chunk[:, :-1] *= dt
            action_chunk[:, :-1] = np.cumsum(
                action_chunk[:, :-1], axis=0
            )  # integrate to get delta position in radians, franka
            current_joint_positions = np.array(get_joint_positions(joint_state_topic))
            action_chunk[:, :-1] += current_joint_positions
            action_chunk[:, :-1] *= 360 / (2 * np.pi)  # convert to degrees
            ACTION_CHUNK = action_chunk[:, :-1]
            GRIPPER_CHUNK = action_chunk[:, 7]
            GRIPPER_CHUNK = np.where(GRIPPER_CHUNK > 0.5, 0.0, GRIPPER_CHUNK)

        action = ACTION_CHUNK[actions_from_chunk_completed]
        limits = np.array(XARM7_JOINT_LIMITS_DEG[: len(action)])
        lower = limits[:, 0]
        upper = limits[:, 1]
        action = np.clip(action, lower, upper)
        gripper_xarm = (1.0 - GRIPPER_CHUNK[actions_from_chunk_completed]) * 850
        actions_from_chunk_completed += 1

        print(f"Setting joint positions: {action} and gripper position: {gripper_xarm}")
        joint_positions_rad = np.deg2rad(action).tolist()
        # joint_cmd_pub.broadcast(None, JointCommand(positions=joint_positions_rad))

        time.sleep(0.2)


if __name__ == "__main__":

    # connect to policy server
    policy = websocket_client_policy.WebsocketClientPolicy(
        host="localhost",
        port=8000,
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--wrist-cam-topic", type=str, default="/wrist/color_image")
    parser.add_argument("--exterior-cam-topic", type=str, default="/exterior/color_image")
    parser.add_argument("--joint-state-topic", type=str, default="/xarm/joint_states")
    parser.add_argument("--joint-cmd-topic", type=str, default="/xarm/joint_position_command")
    args = parser.parse_args()

    wrist_cam_topic = args.wrist_cam_topic
    exterior_cam_topic = args.exterior_cam_topic
    joint_state_topic = args.joint_state_topic
    joint_cmd_topic = args.joint_cmd_topic

    print("Starting inference loop...")
    run_inference(wrist_cam_topic, exterior_cam_topic, joint_state_topic, joint_cmd_topic)
