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

import threading
import time

import numpy as np

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PersonFollowSkillContainer(SkillModule):
    """Skill container for following a person using visual servoing with EdgeTAM.

    This skill uses:
    - A VL model (QwenVlModel) to initially detect a person from a text description
    - EdgeTAM for continuous tracking across frames
    - Visual servoing to control robot movement towards the person
    """

    rpc_calls: list[str] = ["GO2Connection.move"]

    color_image: In[Image]

    # Camera calibration information for image undistortion
    camera_info: CameraInfo | None = None

    # Visual servoing parameters
    _target_distance: float = 1.5  # meters - target distance to maintain from person
    _min_distance: float = 0.8  # meters - stop if closer than this
    _max_linear_speed: float = 0.5  # m/s - maximum forward/backward speed
    _max_angular_speed: float = 0.8  # rad/s - maximum turning speed

    # Size-based distance estimation parameters
    # Assumed average adult shoulder width in meters for distance estimation
    # Using width instead of height because ground robot can't see full person height up close
    _ASSUMED_PERSON_WIDTH: float = 0.45  # meters (average shoulder width)

    def __init__(self, camera_info: CameraInfo, global_config: GlobalConfig) -> None:
        super().__init__()
        self._global_config: GlobalConfig = global_config
        self._latest_image: Image | None = None
        self._vl_model: QwenVlModel | None = None
        self._tracker: EdgeTAMProcessor | None = None
        self._following: bool = False
        self._stop_event = threading.Event()
        self._follow_thread: threading.Thread | None = None
        self.camera_info = camera_info

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.color_image.subscribe(self._on_color_image))  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        self._stop_following()
        if self._tracker is not None:
            self._tracker.stop()
            self._tracker = None
        if self._vl_model is not None:
            self._vl_model.stop()
            self._vl_model = None
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image

    def _get_normalized_x(self, pixel_x: float) -> float | None:
        """Convert pixel x coordinate to normalized camera coordinate.

        Uses inverse K matrix: x_norm = (pixel_x - cx) / fx

        Args:
            pixel_x: x coordinate in pixels

        Returns:
            Normalized x coordinate (tan of angle), or None if no camera info
        """
        if self.camera_info is None:
            return None
        fx = self.camera_info.K[0]  # focal length x
        cx = self.camera_info.K[2]  # optical center x
        return (pixel_x - cx) / fx

    def _ensure_models_loaded(self) -> None:
        """Lazy load models when first needed."""
        if self._vl_model is None:
            logger.info("Loading VL model for initial detection...")
            self._vl_model = QwenVlModel()
        if self._tracker is None:
            logger.info("Loading EdgeTAM tracker...")
            self._tracker = EdgeTAMProcessor()

    def _estimate_distance(self, bbox: tuple[float, float, float, float]) -> float:
        """Estimate distance to person based on bounding box size and camera intrinsics.

        Uses the pinhole camera model:
            pixel_width / fx = real_width / distance
            distance = (real_width * fx) / pixel_width

        Uses bbox width instead of height because ground robot can't see full
        person height when close. Width (shoulders) is more consistently visible.
        """
        bbox_width = bbox[2] - bbox[0]  # x2 - x1

        if bbox_width <= 0:
            return float("inf")

        if self.camera_info is None:
            # Fallback: rough estimation assuming fx ~= image_width
            # This will be inaccurate but prevents crash
            logger.warning("No camera intrinsics available, using rough distance estimate")
            estimated_distance = (self._ASSUMED_PERSON_WIDTH * 800) / bbox_width
        else:
            # Pinhole camera model: distance = (real_width * fx) / pixel_width
            fx = self.camera_info.K[0]  # focal length x in pixels
            estimated_distance = (self._ASSUMED_PERSON_WIDTH * fx) / bbox_width

        logger.info(
            f"BBox width: {bbox_width:.0f}px, estimated distance: {estimated_distance:.2f}m"
        )
        return estimated_distance

    def _compute_visual_servo_twist(
        self,
        bbox: tuple[float, float, float, float],
        image_width: int,
    ) -> Twist:
        """Compute twist command to servo towards the person.

        Args:
            bbox: Bounding box (x1, y1, x2, y2) in pixels
            image_width: Width of the image

        Returns:
            Twist command for the robot
        """
        x1, _, x2, _ = bbox
        bbox_center_x = (x1 + x2) / 2.0

        # Get normalized x coordinate using inverse K matrix
        # Positive = person is to the right of optical center
        x_norm = self._get_normalized_x(bbox_center_x)
        if x_norm is None:
            # Fallback: assume cx at image center, fx ~= image_width (rough approx)
            x_norm = (bbox_center_x - image_width / 2.0) / image_width

        # Estimate distance based on bbox size
        estimated_distance = self._estimate_distance(bbox)

        # Calculate distance error (positive = too far, need to move forward)
        distance_error = estimated_distance - self._target_distance

        # Compute angular velocity (turn towards person)
        # Negative because positive angular.z is counter-clockwise (left turn)
        angular_z = -x_norm * 1.0  # Proportional gain (reduced from 2.0 to avoid oscillation)
        angular_z = float(np.clip(angular_z, -self._max_angular_speed, self._max_angular_speed))

        # Compute linear velocity - ALWAYS move forward/backward based on distance
        # Reduce forward speed when turning sharply to maintain stability
        turn_factor = 1.0 - min(abs(x_norm) * 2.0, 0.7)  # Range: [0.3, 1.0]

        if estimated_distance < self._min_distance:
            # Too close, back up (don't reduce speed for backing up)
            linear_x = -self._max_linear_speed * 0.6
        else:
            # Move forward based on distance error with proportional gain
            # Gain of 0.8 means at 1m error, we'd want 0.8 m/s (before clamping)
            linear_x = distance_error * 0.8 * turn_factor
            linear_x = float(np.clip(linear_x, -self._max_linear_speed, self._max_linear_speed))

        logger.info(
            f"Servo: dist_err={distance_error:.2f}m, x_norm={x_norm:.3f}, "
            f"linear={linear_x:.2f}m/s, angular={angular_z:.2f}rad/s"
        )

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),  # type: ignore[call-arg]
            angular=Vector3(0.0, 0.0, angular_z),  # type: ignore[call-arg]
        )

    def _follow_loop(self, query: str) -> str:
        """Main following loop."""
        self._ensure_models_loaded()

        if self._latest_image is None:
            return "No image available to detect person."

        # Get initial bounding box using VL model
        logger.info(f"Looking for: {query}")
        initial_bbox = get_object_bbox_from_image(
            self._vl_model,
            self._latest_image,
            query,  # type: ignore[arg-type]
        )

        if initial_bbox is None:
            return f"Could not find '{query}' in the current view."

        logger.info(f"Found {query} at bbox: {initial_bbox}")

        # Initialize EdgeTAM tracker with the initial bounding box
        x1, y1, x2, y2 = initial_bbox
        box = np.array([x1, y1, x2, y2], dtype=np.float32)

        logger.info("Initializing EdgeTAM tracker...")
        initial_detections = self._tracker.init_track(  # type: ignore[union-attr]
            image=self._latest_image,
            box=box,
            obj_id=1,
        )

        if len(initial_detections) == 0:
            return f"EdgeTAM failed to segment '{query}'."

        logger.info(f"EdgeTAM initialized with {len(initial_detections)} detections")

        try:
            move_rpc = self.get_rpc_calls("GO2Connection.move")
        except Exception:
            logger.error("GO2Connection not connected properly")
            return "Failed to connect to GO2Connection."

        # Main following loop
        lost_count = 0
        max_lost_frames = 15  # Number of frames to wait before declaring person lost

        while not self._stop_event.is_set():
            if self._latest_image is None:
                time.sleep(0.05)
                continue

            # Track person in current frame using EdgeTAM
            logger.info(
                f"Image size: {self._latest_image.width}x{self._latest_image.height}, frame_count: {self._tracker.frame_count}"
            )
            start = time.perf_counter()
            detections = self._tracker.process_image(self._latest_image)  # type: ignore[union-attr]
            end = time.perf_counter()
            logger.info(f"EdgeTAM processing time: {(end - start) * 1000:.1f} ms")

            if len(detections) == 0:
                lost_count += 1
                if lost_count > max_lost_frames:
                    # Stop the robot and exit
                    move_rpc(Twist.zero())
                    return f"Lost track of '{query}'. Stopping."
                # Keep last velocity for a short time
                time.sleep(0.05)
                continue

            lost_count = 0

            # Get the largest detection (most likely to be the target)
            best_detection = max(detections.detections, key=lambda d: d.bbox_2d_volume())

            # Compute servo command
            twist = self._compute_visual_servo_twist(
                best_detection.bbox,
                self._latest_image.width,
            )

            # Send movement command
            move_rpc(twist)

            # Small delay to avoid overloading
            time.sleep(0.05)

        # Stopped by user
        move_rpc(Twist.zero())
        return f"Stopped following '{query}'."

    def _stop_following(self) -> None:
        """Stop the following loop."""
        self._stop_event.set()
        if self._follow_thread is not None and self._follow_thread.is_alive():
            self._follow_thread.join(timeout=2.0)
        self._follow_thread = None
        self._following = False

    @skill()
    def follow_person(self, query: str) -> str:
        """Follow a person matching the given description using visual servoing.

        The robot will continuously track and follow the person, maintaining
        a safe distance while keeping them centered in the camera view.

        Uses EdgeTAM for robust video object tracking and segmentation.

        Args:
            query: Description of the person to follow (e.g., "man with blue shirt",
                   "woman in red dress", "person wearing glasses")

        Returns:
            Status message indicating the result of the following action.

        Example:
            follow_person("man with blue shirt")
            follow_person("person in the doorway")
        """
        if self._following:
            return "Already following someone. Call stop_following first."

        self._following = True
        self._stop_event.clear()

        # Run the follow loop in this thread (blocking skill)
        result = self._follow_loop(query)
        self._following = False

        return result

    @skill()
    def stop_following(self) -> str:
        """Stop following the current person.

        Returns:
            Confirmation message.
        """
        if not self._following:
            return "Not currently following anyone."

        self._stop_following()

        # Send stop command
        try:
            move_rpc = self.get_rpc_calls("GO2Connection.move")
            move_rpc(Twist.zero())
        except Exception:
            pass

        return "Stopped following."


person_follow_skill = PersonFollowSkillContainer.blueprint

__all__ = ["PersonFollowSkillContainer", "person_follow_skill"]
