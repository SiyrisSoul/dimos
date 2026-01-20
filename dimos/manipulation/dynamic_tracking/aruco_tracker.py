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

from dataclasses import dataclass
import os
from threading import Event, Thread
import time
from typing import Any

import cv2
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.core.rpc_client import RpcCall
from dimos.dashboard.rerun_init import connect_rerun
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger()


@dataclass
class ArucoTrackerConfig(ModuleConfig):
    """Configuration for ArUco tracker."""

    marker_size: float = 0.1
    aruco_dict: int = cv2.aruco.DICT_4X4_50
    camera_frame_id: str = "camera_color_optical_frame"  # Frame ID for the camera
    rate: float = 1  # Rate in Hz - defines speed of execution (process loop then sleep)
    max_loops: int = 5  # Maximum number of loops to process
    move_robot_to_aruco: bool = True  # Whether to move the robot to the ArUco marker
    safety_max_delta: float = 0.10  # Max allowed 3D distance (meters) between commands
    hardware_id: str = "arm"  # Hardware ID for ControlOrchestrator EE pose lookup
    robot_connected: bool = True  # Whether robot is connected (False = use dummy EE transform)


class ArucoTracker(Module[ArucoTrackerConfig]):
    """
    ArUco marker tracker that detects markers in camera images and computes their transforms.

    Subscribes to camera images and camera info, detects ArUco markers,
    and publishes their transforms relative to the camera frame.
    """

    # Transport ports
    color_image: In[Image]
    camera_info: In[CameraInfo]
    annotated_image: Out[Image]

    config: ArucoTrackerConfig
    default_config = ArucoTrackerConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # ArUco detector
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(self.config.aruco_dict)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        # params from callback
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._latest_image: Image | None = None

        # Threading for processing loop
        self._stop_event = Event()
        self._processing_thread: Thread | None = None
        self._loop_count = 0

        # RPC calls to ControlOrchestrator
        self._get_ee_positions_rpc: RpcCall | None = None
        self._move_to_cartesian_pose_rpc: RpcCall | None = None

        # Safety: track last commanded position for delta check
        self._last_commanded_pos: tuple[float, float, float] | None = None

    @rpc
    def start(self) -> None:
        """Start the ArUco tracker by subscribing to camera streams."""
        super().start()

        connect_rerun()
        self._disposables.add(self.camera_info.observable().subscribe(self._update_camera_info))
        self._disposables.add(self.color_image.observable().subscribe(self._store_latest_image))

        self._loop_count = 0
        self._stop_event.clear()
        self._processing_thread = Thread(
            target=self._processing_loop, daemon=True, name="ArucoTracker"
        )
        self._processing_thread.start()

    @rpc
    def stop(self) -> None:
        """Stop the ArUco tracker."""
        self._stop_event.set()

        if self._processing_thread is not None and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)
        super().stop()

    def _store_latest_image(self, image: Image) -> None:
        """Store the latest image for processing."""
        self._latest_image = image

    def _update_camera_info(self, camera_info: CameraInfo) -> None:
        """Update camera intrinsics from CameraInfo message."""
        if len(camera_info.K) == 9:
            fx, _, cx, _, fy, cy, _, _, _ = camera_info.K
            self._camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
            self._dist_coeffs = (
                np.array(camera_info.D, dtype=np.float32) if camera_info.D else np.zeros(5)
            )

    @rpc
    def set_ControlOrchestrator_get_ee_positions(self, rpc_call: RpcCall) -> None:
        """Wire get_ee_positions RPC from ControlOrchestrator."""
        self._get_ee_positions_rpc = rpc_call
        self._get_ee_positions_rpc.set_rpc(self.rpc)

    @rpc
    def set_ControlOrchestrator_move_to_cartesian_pose(self, rpc_call: RpcCall) -> None:
        """Wire move_to_cartesian_pose RPC from ControlOrchestrator."""
        self._move_to_cartesian_pose_rpc = rpc_call
        self._move_to_cartesian_pose_rpc.set_rpc(self.rpc)

    def _log_transform_to_rerun(self, transform: Transform) -> None:
        """Log a transform to Rerun without named frame references (avoids duplicate entries)."""
        rr.log(
            f"world/tf/{transform.child_frame_id}",
            rr.Transform3D(
                translation=[
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                rotation=rr.Quaternion(
                    xyzw=[
                        transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w,
                    ]
                ),
            ),
        )

    def _check_safety_delta(self, x: float, y: float, z: float) -> bool:
        """Check if the new position is within safety limits of the last commanded position."""
        if self._last_commanded_pos is None:
            return True

        # Calculate 3D distance from last successful command
        dx = x - self._last_commanded_pos[0]
        dy = y - self._last_commanded_pos[1]
        dz = z - self._last_commanded_pos[2]
        delta = (dx**2 + dy**2 + dz**2) ** 0.5

        if delta > self.config.safety_max_delta:
            logger.warning(
                f"Safety check failed: delta {delta:.3f}m exceeds limit {self.config.safety_max_delta:.3f}m"
            )
            return False
        return True

    def _processing_loop(self) -> None:
        """Processing loop that runs at the configured rate."""
        period = 1.0 / self.config.rate
        logger.info(f"ArUco processing loop started at {self.config.rate}Hz")

        while not self._stop_event.is_set() and self._loop_count < self.config.max_loops:
            loop_start = time.time()
            try:
                if self._latest_image is None:
                    time.sleep(period)
                    continue

                self._process_image(self._latest_image)
                self._loop_count += 1
                logger.debug(f"Processed image {self._loop_count}/{self.config.max_loops}")

            except Exception as e:
                logger.error(f"Error in processing loop: {e}")

            # Sleep for the remainder of the period
            elapsed = time.time() - loop_start
            logger.debug(f"Processing loop took {elapsed:.3f}s")
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        logger.info(f"ArUco processing loop completed after {self._loop_count} iterations")

    def _process_image(self, image: Image) -> None:
        """Process image to detect ArUco markers."""
        if self._camera_matrix is None or self._dist_coeffs is None:
            return  # Skip if camera info not ready yet

        # Convert image for visualization (keep original format)
        if image.format.name == "RGB":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_RGB2GRAY)
        elif image.format.name == "BGR":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_BGR2GRAY)
        else:
            display_image = image.data.copy()
            gray = image.data

        # Detect ArUco markers
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            logger.debug("No ArUco markers detected")
            return

        if len(ids) > 1:
            logger.error(f"Multiple ArUco markers detected ({len(ids)}), expected exactly one")
            return

        marker_id = int(ids[0][0])
        marker_corners = corners[0]

        # Estimate pose for the single marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners], self.config.marker_size, self._camera_matrix, self._dist_coeffs
        )
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        # Convert rotation vector to quaternion
        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

        # Create and publish transforms (ArUco marker and EE)
        transform = self._set_transforms(marker_id, tvec, quat, image.ts)
        if transform is None:
            logger.error("Failed to create transform")
            return

        # Print TF tree
        # print(self.tf.graph())

        aruco_wrt_robot_base = self.tf.get("base_link", f"aruco_{marker_id}")
        if aruco_wrt_robot_base is not None:
            aruco_rpy = aruco_wrt_robot_base.rotation.to_euler()
            logger.debug(
                f"ArUco wrt base_link - pos: ({aruco_wrt_robot_base.translation.x:.3f}, "
                f"{aruco_wrt_robot_base.translation.y:.3f}, {aruco_wrt_robot_base.translation.z:.3f}), "
                f"RPY: ({aruco_rpy.x:.3f}, {aruco_rpy.y:.3f}, {aruco_rpy.z:.3f})"
            )

        if aruco_wrt_robot_base is not None and self.config.move_robot_to_aruco:
            self._move_to_aruco(aruco_wrt_robot_base)

        # Draw markers on display image
        self._draw_markers(
            display_image,
            [marker_corners],
            np.array([[marker_id]]),
            rvecs,
            tvecs,
            [transform],
            image.format.name,
        )

    def _move_to_aruco(self, aruco_wrt_robot_base: Transform) -> None:
        """Move the robot to the ArUco marker position with offset."""
        # Calculate reach pose: position offset + orientation
        reach_x = aruco_wrt_robot_base.translation.x - 0.05  # 5cm offset in x
        reach_y = aruco_wrt_robot_base.translation.y
        reach_z = aruco_wrt_robot_base.translation.z + 0.20  # 20cm offset in z

        # Orientation: roll=-3.13, pitch=0, yaw=0 (in radians)
        reach_roll = -3.13
        reach_pitch = 0.0
        reach_yaw = 0.0

        logger.debug(
            f"Computed reach pose: x={reach_x:.3f}, y={reach_y:.3f}, z={reach_z:.3f}, "
            f"roll={reach_roll:.3f}, pitch={reach_pitch:.3f}, yaw={reach_yaw:.3f}"
        )

        # Safety check: ensure delta is within limits
        if not self._check_safety_delta(reach_x, reach_y, reach_z):
            logger.warning("Skipping move command due to safety check failure")
            return

        if self._move_to_cartesian_pose_rpc is None:
            logger.warning("move_to_cartesian_pose RPC not available")
            return

        try:
            success = self._move_to_cartesian_pose_rpc(
                hardware_id=self.config.hardware_id,
                x=reach_x,
                y=reach_y,
                z=reach_z,
                roll=reach_roll,
                pitch=reach_pitch,
                yaw=reach_yaw,
            )
            if not success:
                logger.warning("Failed to move to pose, stopping processing loop")
                self._loop_count = self.config.max_loops
            else:
                self._last_commanded_pos = (reach_x, reach_y, reach_z)
                logger.debug("Successfully commanded move to pose")
        except Exception as e:
            logger.error(f"Error calling move_to_cartesian_pose: {e}")
            self._loop_count = self.config.max_loops

    def _set_transforms(
        self, marker_id: int, tvec: np.ndarray, quat: np.ndarray, timestamp: float
    ) -> Transform | None:
        # Create ArUco marker transform (camera_optical -> aruco_{marker_id})
        aruco_transform = Transform(
            translation=Vector3(float(tvec[0]), float(tvec[1]), float(tvec[2])),
            rotation=Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            frame_id="camera_color_optical_frame",
            child_frame_id=f"aruco_{marker_id}",
            ts=timestamp,
        )

        # Publish ArUco marker transform to TF buffer (for graph lookups)
        self.tf.publish(aruco_transform)
        self._log_transform_to_rerun(aruco_transform)

        # Publish world -> base_link transform (static, but republish for TF polling)
        robot_base_to_world_transform = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id="base_link",
            ts=timestamp,
        )
        self.tf.publish(robot_base_to_world_transform)
        self._log_transform_to_rerun(robot_base_to_world_transform)

        if self.config.robot_connected and self._get_ee_positions_rpc is not None:
            # Get EE pose from ControlOrchestrator via RPC
            try:
                ee_positions = self._get_ee_positions_rpc()
                if ee_positions is not None:
                    hw_pose = ee_positions.get(self.config.hardware_id)
                    if hw_pose is not None:
                        x, y, z = hw_pose["x"], hw_pose["y"], hw_pose["z"]
                        roll, pitch, yaw = hw_pose["roll"], hw_pose["pitch"], hw_pose["yaw"]
                        orientation = euler_to_quaternion(Vector3(roll, pitch, yaw))
                        ee_transform = Transform(
                            translation=Vector3(float(x), float(y), float(z)),
                            rotation=orientation,
                            frame_id="base_link",
                            child_frame_id="ee_link",
                            ts=timestamp,
                        )
                        self.tf.publish(ee_transform)
                        self._log_transform_to_rerun(ee_transform)
                    else:
                        logger.warning(f"No EE pose for hardware_id '{self.config.hardware_id}'")
            except Exception as e:
                logger.error(f"Error getting EE pose from ControlOrchestrator: {e}")
        else:
            import math

            orientation = euler_to_quaternion(Vector3(math.pi, 0.0, 0.0))
            ee_transform = Transform(
                translation=Vector3(0.4, 0.0, 0.4),
                rotation=orientation,
                frame_id="base_link",
                child_frame_id="ee_link",
                ts=timestamp,
            )
            self.tf.publish(ee_transform)
            self._log_transform_to_rerun(ee_transform)

        return aruco_transform

    def _draw_markers(
        self,
        display_image: np.ndarray,
        corners: list[np.ndarray],
        ids: np.ndarray,
        rvecs: np.ndarray,
        tvecs: np.ndarray,
        transforms: list[Transform],
        image_format: str,
    ) -> None:
        # Draw all detected markers
        cv2.aruco.drawDetectedMarkers(display_image, corners, ids)

        # Draw axes and text for each marker
        for i, (_marker_id, _transform) in enumerate(zip(ids.flatten(), transforms, strict=False)):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            axis_length = self.config.marker_size * 0.5
            cv2.drawFrameAxes(
                display_image,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec,
                axis_length,
            )

        # Publish annotated image (for Foxglove or other subscribers)
        from dimos.msgs.sensor_msgs.Image import ImageFormat

        if image_format == "BGR":
            publish_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        else:
            publish_image = display_image

        annotated_msg = Image(
            data=publish_image,
            format=ImageFormat.RGB,
            frame_id=self.config.camera_frame_id,
            ts=time.time(),
        )
        self.annotated_image.publish(annotated_msg)
        rr.log("aruco/annotated", rr.Image(publish_image))


aruco_tracker = ArucoTracker.blueprint

__all__ = ["ArucoTracker", "ArucoTrackerConfig", "aruco_tracker"]
