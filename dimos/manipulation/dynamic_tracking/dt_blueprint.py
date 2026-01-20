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

import cv2

from dimos.control.blueprints import orchestrator_xarm6_cartesian
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.dynamic_tracking.aruco_tracker import aruco_tracker
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image

# =============================================================================
# ArUco Tracker with RealSense Camera
# =============================================================================
# Combines:
#   - RealSenseCamera: RGB-D camera with hardware interface
#   - ArucoTracker: Detects ArUco markers and computes transforms
#
# Data flow:
#   RealSenseCamera.color_image ──► ArucoTracker.color_image (marker detection)
#   RealSenseCamera.camera_info ──► ArucoTracker.camera_info (intrinsics)
# =============================================================================

aruco_tracker_realsense = (
    autoconnect(
        RealSenseCamera.blueprint(
            width=848,
            height=480,
            fps=15,
            camera_name="camera",
            base_frame_id="ee_link",
            base_transform=Transform(
                translation=Vector3(0.06693724, -0.0309563, 0.00691482),
                rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
            ),
            enable_depth=True,
            align_depth_to_color=False,
        ),
        aruco_tracker(
            marker_size=0.027,  # 27mm markers (default)
            aruco_dict=cv2.aruco.DICT_4X4_50,
            camera_frame_id="camera_color_optical_frame",
            rate=15,
            max_loops=10000,
            move_robot_to_aruco=False,
            robot_connected=False,  # No robot, use dummy EE transform
        ),
    )
    .transports(
        {
            # Camera color image for ArUco detection
            ("color_image", Image): LCMTransport("/camera/color", Image),
            # Camera info for pose estimation
            ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
            # Annotated image with ArUco axes
            ("annotated_image", Image): LCMTransport("/aruco/annotated", Image),
        }
    )
    .global_config(viewer_backend="rerun-native")
)

# =============================================================================
# ArUco Tracker with RealSense Camera + XArm6 Control Orchestrator
# =============================================================================
# Extends aruco_tracker_realsense with XArm6 control orchestrator.
# =============================================================================

aruco_tracker_realsense_xarm6 = (
    autoconnect(
        orchestrator_xarm6_cartesian,
        RealSenseCamera.blueprint(
            width=848,
            height=480,
            fps=15,
            camera_name="camera",
            base_frame_id="ee_link",
            base_transform=Transform(
                translation=Vector3(0.06693724, -0.0309563, 0.00691482),
                rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
            ),
            enable_depth=True,
            align_depth_to_color=False,
        ),
        aruco_tracker(
            marker_size=0.027,  # 27mm markers (default)
            aruco_dict=cv2.aruco.DICT_4X4_50,
            camera_frame_id="camera_color_optical_frame",
            rate=15,
            max_loops=10000,
            move_robot_to_aruco=True,
            robot_connected=True,
        ),
    )
    .transports(
        {
            # Camera color image for ArUco detection
            ("color_image", Image): LCMTransport("/camera/color", Image),
            # Camera info for pose estimation
            ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
            # Annotated image with ArUco axes
            ("annotated_image", Image): LCMTransport("/aruco/annotated", Image),
        }
    )
    .global_config(viewer_backend="rerun-native")
)


__all__ = ["aruco_tracker_realsense", "aruco_tracker_realsense_xarm6"]
