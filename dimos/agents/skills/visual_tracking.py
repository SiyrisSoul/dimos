# Copyright 2025 Dimensional Inc.
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

# cv_bridge removed for numpy 2.x compatibility
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage as ROSCompressedImage, Image as ROSImage
from std_msgs.msg import Bool as ROSBool
from vision_msgs.msg import (
    BoundingBox2D as ROSBoundingBox2D,
    Detection2D as ROSDetection2D,
    Detection2DArray as ROSDetection2DArray,
    ObjectHypothesisWithPose as ROSObjectHypothesisWithPose,
    Point2D as ROSPoint2D,
    Pose2D as ROSPose2D,
)

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.models.vl.moondream_hosted import MoondreamHostedVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


class VisualTrackingSkillContainer(SkillModule):
    _latest_image: Image | None = None
    _moondream: MoondreamHostedVlModel | None = None
    _node: Node | None = None
    _tracking_status: bool | None = None
    _spin_thread: threading.Thread | None = None
    _running: bool = False

    def __init__(self) -> None:
        super().__init__()
        self._moondream = MoondreamHostedVlModel()

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("visual_tracking_skill")

        self._image_sub = self._node.create_subscription(
            ROSCompressedImage, "/camera/color/image_raw/compressed", self._on_compressed_image, 10
        )
        self._bbox_pub = self._node.create_publisher(ROSDetection2DArray, "/track_3d/init_bbox", 10)
        self._tracking_sub = self._node.create_subscription(
            ROSBool, "/track_3d/is_tracking", self._on_tracking_status, 10
        )

        self._spin_thread = threading.Thread(
            target=self._spin_node, daemon=True, name="VisualTrackingSpinThread"
        )
        self._spin_thread.start()

    def _spin_node(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        if self._node:
            self._node.destroy_node()
        super().stop()

    def _on_compressed_image(self, msg: ROSCompressedImage) -> None:
        import cv2
        import numpy as np

        compressed_data = np.frombuffer(msg.data, dtype=np.uint8)
        bgr_image = cv2.imdecode(compressed_data, cv2.IMREAD_COLOR)
        cv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        self._latest_image = Image.from_numpy(cv_image)

    def _on_tracking_status(self, msg: ROSBool) -> None:
        self._tracking_status = msg.data

    @skill()
    def track_object(self, query: str) -> str:
        """Track an object using visual detection. Provide a description of what to track.

        Example:
            track_object("person")
            track_object("red cup")
            track_object("xbox controller")
        """
        image = self._latest_image

        logger.info(f"Detecting '{query}' in image")
        detections = self._moondream.query_detections(image, query)

        if not detections.detections:
            return f"No '{query}' found in current camera view."

        logger.info(f"Found {len(detections.detections)} detections for '{query}'")

        detection_array = ROSDetection2DArray()

        for det in detections.detections:
            x1, y1, x2, y2 = det.bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            size_x = x2 - x1
            size_y = y2 - y1

            ros_detection = ROSDetection2D()
            ros_detection.bbox = ROSBoundingBox2D()
            ros_detection.bbox.center = ROSPose2D()
            ros_detection.bbox.center.position = ROSPoint2D()
            ros_detection.bbox.center.position.x = center_x
            ros_detection.bbox.center.position.y = center_y
            ros_detection.bbox.center.theta = 0.0
            ros_detection.bbox.size_x = size_x
            ros_detection.bbox.size_y = size_y

            detection_array.detections.append(ros_detection)

        self._bbox_pub.publish(detection_array)
        logger.info(f"Published {len(detection_array.detections)} bboxes to /track_3d/init_bbox")

        return f"Initiated tracking for {len(detections.detections)} {query} object(s)."


visual_tracking_skill = VisualTrackingSkillContainer.blueprint


__all__ = ["VisualTrackingSkillContainer", "visual_tracking_skill"]
