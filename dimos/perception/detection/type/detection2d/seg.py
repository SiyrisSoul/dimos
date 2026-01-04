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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
import torch

from dimos_lcm.foxglove_msgs.ImageAnnotations import PointsAnnotation
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.types.timestamped import to_ros_stamp

if TYPE_CHECKING:
    pass


@dataclass
class Detection2DSeg(Detection2DBBox):
    """Represents a detection with a segmentation mask."""

    mask: np.ndarray  # Binary mask [H, W], uint8 0 or 255

    @classmethod
    def from_sam2_result(
        cls,
        mask: np.ndarray | torch.Tensor,
        obj_id: int,
        image: Image,
        class_id: int = 0,
        name: str = "object",
        confidence: float = 1.0,
    ) -> Detection2DSeg:
        """Create Detection2DSeg from SAM output (single object).

        Args:
            mask: Segmentation mask (logits or binary). Shape [H, W] or [1, H, W].
            obj_id: Tracking ID of the object.
            image: Source image.
            class_id: Class ID (default 0).
            name: Class name (default "object").
            confidence: Confidence score (default 1.0).

        Returns:
            Detection2DSeg instance.
        """
        # Convert mask to numpy if tensor
        if isinstance(mask, torch.Tensor):
            mask = mask.detach().cpu().numpy()

        # Handle dimensions (EdgeTAM might return [1, H, W] or [H, W])
        if mask.ndim == 3:
            mask = mask.squeeze()

        # Binarize if it's logits (usually < 0 is background, > 0 is foreground)
        # or if it's boolean
        if mask.dtype == bool:
            mask = mask.astype(np.uint8) * 255
        elif np.issubdtype(mask.dtype, np.floating):
            mask = (mask > 0.0).astype(np.uint8) * 255

        # Calculate bbox
        y_indices, x_indices = np.where(mask > 0)
        if len(x_indices) > 0:
            x1, y1 = np.min(x_indices), np.min(y_indices)
            x2, y2 = np.max(x_indices), np.max(y_indices)
        else:
            x1, y1, x2, y2 = 0.0, 0.0, 0.0, 0.0

        bbox = (float(x1), float(y1), float(x2), float(y2))

        return cls(
            bbox=bbox,
            track_id=obj_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
            mask=mask,
        )

    def to_points_annotation(self) -> list[PointsAnnotation]:
        """Override to include mask outline."""
        annotations = super().to_points_annotation()

        # Find contours
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Simplify contour to reduce points
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            points = []
            for pt in approx:
                points.append(Point2(x=float(pt[0][0]), y=float(pt[0][1])))

            if len(points) < 3:
                continue

            annotations.append(
                PointsAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    outline_color=Color.from_string(self.name, alpha=1.0, brightness=1.25),
                    fill_color=Color.from_string(self.name, alpha=0.4),
                    thickness=1.0,
                    points_length=len(points),
                    points=points,
                    type=PointsAnnotation.LINE_LOOP,
                )
            )

        return annotations

