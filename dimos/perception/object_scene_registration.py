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
import time
from queue import Queue

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import (
    CameraInfo as ROSCameraInfo,
    CompressedImage as ROSCompressedImage,
    Image as ROSImage,
    PointCloud2 as ROSPointCloud2,
)
from std_msgs.msg import Header as ROSHeader
from vision_msgs.msg import (
    Detection2DArray as ROSDetection2DArray,
    Detection3DArray as ROSDetection3DArray,
)

from dimos.core import Module, rpc
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ObjectSceneRegistrationModule(Module):
    """Module for detecting objects in camera images using YOLO-E with 2D and 3D detection."""

    _detector: Yoloe2DDetector | None = None
    _node: Node | None = None
    _bridge: CvBridge | None = None
    _spin_thread: threading.Thread | None = None
    _processing_thread: threading.Thread | None = None
    _processing_queue: Queue | None = None
    _running: bool = False
    _camera_info: CameraInfo | None = None

    def __init__(
        self,
        image_topic: str = "/camera/color/image_raw/compressed",
        depth_topic: str = "/camera/aligned_depth_to_color/image_raw/compressedDepth",
        camera_info_topic: str = "/camera/color/camera_info",
        detections_2d_topic: str = "/object_detections/bbox",
        detections_3d_topic: str = "/object_detections/bbox3d",
        overlay_topic: str = "/object_detections/overlay",
        pointcloud_topic: str = "/object_detections/pointcloud",
    ) -> None:
        super().__init__()
        self._image_topic = image_topic
        self._depth_topic = depth_topic
        self._camera_info_topic = camera_info_topic
        self._detections_2d_topic = detections_2d_topic
        self._detections_3d_topic = detections_3d_topic
        self._overlay_topic = overlay_topic
        self._pointcloud_topic = pointcloud_topic
        self._bridge = CvBridge()

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True

        # Initialize detector (uses yoloe-11l-seg-pf.pt for LRPC mode by default)
        self._detector = Yoloe2DDetector(
            prompt_mode=YoloePromptMode.LRPC,
        )
        logger.info("Initialized YOLO-E detector in LRPC (prompt-free) mode")

        # Initialize ROS
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("object_scene_registration")

        # Wait for camera info once (it rarely changes)
        logger.info(f"Waiting for camera_info on {self._camera_info_topic}...")
        success, camera_info_msg = wait_for_message(
            ROSCameraInfo,
            self._node,
            self._camera_info_topic,
            time_to_wait=10.0,
        )
        if success and camera_info_msg:
            self._camera_info = CameraInfo.from_ros_msg(camera_info_msg)
            logger.info(f"Received camera_info: {camera_info_msg.width}x{camera_info_msg.height}")
        else:
            raise RuntimeError(f"Timeout waiting for camera_info on {self._camera_info_topic}")

        # Publishers
        self._detections_2d_pub = self._node.create_publisher(
            ROSDetection2DArray,
            self._detections_2d_topic,
            10,
        )
        self._detections_3d_pub = self._node.create_publisher(
            ROSDetection3DArray,
            self._detections_3d_topic,
            10,
        )
        self._overlay_pub = self._node.create_publisher(
            ROSImage,
            self._overlay_topic,
            10,
        )
        self._pointcloud_pub = self._node.create_publisher(
            ROSPointCloud2,
            self._pointcloud_topic,
            10,
        )

        # Set up synchronized subscribers for color and depth
        self._color_sub = message_filters.Subscriber(
            self._node,
            ROSCompressedImage,
            self._image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._depth_sub = message_filters.Subscriber(
            self._node,
            ROSCompressedImage,
            self._depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        # Synchronize color and depth with approximate time sync
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=10,
            slop=0.1,  # 100ms tolerance
        )
        self._sync.registerCallback(self._on_synced_images)

        logger.info("Synchronized subscribers:")
        logger.info(f"  Color: {self._image_topic}")
        logger.info(f"  Depth: {self._depth_topic}")
        logger.info("Publishing:")
        logger.info(f"  2D detections: {self._detections_2d_topic}")
        logger.info(f"  3D detections: {self._detections_3d_topic}")
        logger.info(f"  Overlay: {self._overlay_topic}")
        logger.info(f"  Pointcloud: {self._pointcloud_topic}")

        # Start spin thread
        self._spin_thread = threading.Thread(
            target=self._spin_node,
            daemon=True,
            name="ObjectSceneRegistrationSpinThread",
        )
        self._spin_thread.start()

        # Start processing thread (keeps callback fast, processes in background)
        self._processing_queue = Queue(maxsize=1)
        self._processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True,
            name="ObjectSceneRegistrationProcessingThread",
        )
        self._processing_thread.start()

    def _spin_node(self) -> None:
        """Spin the ROS node."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _processing_loop(self) -> None:
        """Background thread for heavy image processing."""
        while self._running:
            try:
                # Block until data is available (with timeout to check _running)
                color_msg, depth_msg = self._processing_queue.get(timeout=0.5)
                self._process_images(color_msg, depth_msg)
            except Exception:
                # Queue.get timeout - just continue to check _running
                continue

    @rpc
    def stop(self) -> None:
        """Stop the module and clean up resources."""
        self._running = False

        if self._processing_thread and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=1.0)

        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)

        if self._detector:
            self._detector.stop()
            self._detector = None

        if self._node:
            self._node.destroy_node()
            self._node = None

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    def _convert_compressed_depth_image(self, msg: ROSCompressedImage) -> Image | None:
        """Convert ROS compressedDepth image to internal Image type."""
        if not self._bridge:
            return None

        try:
            import cv2

            # compressedDepth format has a 12-byte header followed by PNG data
            # cv_bridge doesn't handle this format correctly
            if "compressedDepth" in msg.format:
                depth_header_size = 12
                raw_data = np.frombuffer(msg.data, dtype=np.uint8)

                if len(raw_data) <= depth_header_size:
                    logger.warning("compressedDepth data too small")
                    return None

                # Decode the PNG portion (after header)
                png_data = raw_data[depth_header_size:]
                depth_cv = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)

                if depth_cv is None:
                    logger.warning("Failed to decode compressedDepth PNG data")
                    return None
            else:
                # Regular compressed image
                depth_cv = self._bridge.compressed_imgmsg_to_cv2(msg)

            # Convert to meters if uint16 (assuming mm depth)
            if depth_cv.dtype == np.uint16:
                depth_cv = depth_cv.astype(np.float32) / 1000.0
            elif depth_cv.dtype != np.float32:
                depth_cv = depth_cv.astype(np.float32)

            depth_image = Image.from_numpy(depth_cv)
            depth_image.from_ros_header(msg.header)
            return depth_image
        except Exception as e:
            logger.warning(f"Failed to convert compressed depth image: {e}")
            return None

    def _on_synced_images(
        self, color_msg: ROSCompressedImage, depth_msg: ROSCompressedImage
    ) -> None:
        """Queue synchronized images for processing (fast callback)."""
        if not self._processing_queue:
            return

        # Drop old data if queue is full (we always want the latest)
        if self._processing_queue.full():
            try:
                self._processing_queue.get_nowait()
            except Exception:
                pass

        self._processing_queue.put((color_msg, depth_msg))

    def _process_images(
        self, color_msg: ROSCompressedImage, depth_msg: ROSCompressedImage
    ) -> None:
        """Process synchronized color and depth images (runs in background thread)."""
        if not self._detector or not self._bridge or not self._camera_info:
            return

        # Convert color image
        cv_image = self._bridge.compressed_imgmsg_to_cv2(color_msg, "rgb8")
        color_image = Image.from_numpy(cv_image)
        color_image.from_ros_header(color_msg.header)

        # Convert compressed depth image
        depth_image = self._convert_compressed_depth_image(depth_msg)
        if depth_image is None:
            return

        # Run 2D detection
        detections_2d: ImageDetections2D = self._detector.process_image(color_image)

        ros_detections_2d = detections_2d.to_ros_detection2d_array()
        self._detections_2d_pub.publish(ros_detections_2d)

        # Publish overlay image
        overlay_image = detections_2d.overlay()
        overlay_msg = self._bridge.cv2_to_imgmsg(overlay_image.to_opencv(), encoding="rgb8")
        overlay_msg.header = color_msg.header
        self._overlay_pub.publish(overlay_msg)

        # Process 3D detections
        self._process_3d_detections(detections_2d, color_image, depth_image, color_msg.header)

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D,
        color_image: Image,
        depth_image: Image,
        header: ROSHeader,
    ) -> None:
        """Convert 2D detections to 3D and publish."""
        if self._camera_info is None:
            return

        # Convert 2D segmentation to 3D pointcloud detections
        detections_3d = Detection3DPC.from_2d_depth(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
        )

        if not detections_3d or len(detections_3d.detections) == 0:
            return

        # Publish 3D detection array
        ros_detections_3d = detections_3d.to_ros_detection3d_array()
        self._detections_3d_pub.publish(ros_detections_3d)

        # Publish aggregated pointcloud
        aggregated_pc = detections_3d.get_aggregated_objects_pointcloud()
        if aggregated_pc is not None:
            ros_pc = aggregated_pc.to_ros_msg()
            ros_pc.header = header
            self._pointcloud_pub.publish(ros_pc)


object_scene_registration_module = ObjectSceneRegistrationModule.blueprint

__all__ = ["ObjectSceneRegistrationModule", "object_scene_registration_module"]
