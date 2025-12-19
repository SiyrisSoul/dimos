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
import functools
import time
from collections import defaultdict
from typing import Dict, Generator, List, Optional, Tuple, TypedDict

import numpy as np
from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,
)
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection2D as ROSDetection2D
from reactivex import operators as ops

from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.protocol.skill import skill
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Output, Reducer, Stream

LabelDB = Dict[str, "DetectionLabel"]


class DetectionLabel:
    name: str
    instances: List[Detection3D]

    def __init__(self, name: str):
        self.name = name
        self.instances = []

    def add_detection(self, detection: Detection3D):
        self.instances.append(detection)


class DetectionDBModule(Detection3DModule):
    labels: LabelDB

    def __init__(
        self,
    ):
        super().__init__(self)
        self.labels = {}

    @rpc
    def start(self):
        super().start()
        self.pointcloud_stream().subscribe(self.add_detections)

    def add_detections(self, detections: ImageDetections3D):
        for det in detections:
            self.add_detection(det)

    def add_detection(self, detection: Detection3D):
        if detection.name not in self.labels:
            self.labels[detection.name] = DetectionLabel(detection.name)
        self.labels[detection.name].add_detection(detection)

    @skill
    def list_object_labels(self) -> List[str]:
        """List all detected object labels (e.g., "person", "car", "bottle")."""

    @skill
    def goto_object(self, label_or_id: str):
        """
        Navigate to a specific object by its label or id.
        If there are multiple objects with the same label, you will get a list of object details and their ids
        so you can call goto_object again with a specific id.
        """

    @skill
    def get_object_picture(self, label_or_id: str) -> Image:
        """Get the cropped image of a specific object by its unique identity."""

    @skill(stream=Stream.passive, reducer=Reducer.accumulate_dict)
    def detected_object_stream(self) -> Generator[Detection3D]:
        """Stream of all detected objects between agent invocations."""
