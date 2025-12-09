#!/usr/bin/env python3

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

import math
import time
import traceback
from typing import Callable, Optional

import reactivex as rx
from plum import dispatch
from reactivex import operators as ops

from dimos.core import TF, In, Module, Out, rpc

# from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.msgs.geometry_msgs import (
    Pose,
    PoseLike,
    PoseStamped,
    Transform,
    Twist,
    Vector3,
    VectorLike,
    to_pose,
)
from dimos.msgs.nav_msgs import Path
from dimos.msgs.tf2_msgs import TFMessage
from dimos.types.costmap import Costmap
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.local_planner")


def vector_to_twist(vector: Vector3) -> Twist:
    """Convert a Vector3 to a Twist message."""
    twist = Twist()
    twist.linear.x = vector.x
    twist.linear.y = vector.y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = vector.z
    return twist


class SimplePlanner(Module):
    path: In[Path] = None
    movecmd: Out[Twist] = None
    speed: float = 0.3

    tf: TF

    def __init__(self):
        Module.__init__(self)
        self.tf = TF()

    def move_stream(self, frequency: float = 20.0) -> rx.Observable:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            ops.filter(lambda _: self.tf.get("world", "base_link") and self.goal),
            ops.map(lambda base_link: self.goal @ base_link),
            ops.map(lambda direction: direction.normalize() * self.speed),
        )

    @rpc
    def start(self):
        self.path.subscribe(self.set_goal)
        self.move_stream(frequency=20.0).subscribe(self.movecmd.publish)
