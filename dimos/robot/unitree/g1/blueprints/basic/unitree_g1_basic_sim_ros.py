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

"""Basic G1 sim stack with ROS nav: sim connection and ROS navigation stack."""

import threading
import time

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation.rosnav_docker import ros_nav
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_cam import (
    unitree_g1_primitive_no_cam,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Test waypoints for the sim in the world frame (x, y, yaw_deg)
_TEST_WAYPOINTS = [
    (5.0, 0.0),
    (5.0, 5.0),
    (0.0, 5.0),
    (0.0, 0.0),
]
_GOAL_INTERVAL_S = 30  # seconds between waypoints
_STARTUP_DELAY_S = 90  # wait for nav stack before first goal


class GoalTestPublisher(Module):
    """Publishes a cycling sequence of test goals for the sim nav stack.

    Waits for the nav stack to initialise before publishing the first goal,
    then cycles through _TEST_WAYPOINTS every _GOAL_INTERVAL_S seconds.
    Remove this module (or replace with a real goal source) for production.
    """

    goal_request: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        threading.Thread(target=self._publish_loop, daemon=True, name="GoalTestThread").start()

    def _publish_loop(self) -> None:
        logger.info(f"[GoalTestPublisher] waiting {_STARTUP_DELAY_S}s for nav stack…")
        time.sleep(_STARTUP_DELAY_S)
        idx = 0
        while True:
            x, y = _TEST_WAYPOINTS[idx % len(_TEST_WAYPOINTS)]
            goal = PoseStamped(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
            )
            logger.info(f"[GoalTestPublisher] publishing goal #{idx}: ({x}, {y})")
            self.goal_request.publish(goal)
            idx += 1
            time.sleep(_GOAL_INTERVAL_S)


unitree_g1_basic_sim_ros = autoconnect(
    unitree_g1_primitive_no_cam,
    ros_nav(mode="simulation"),
    # GoalTestPublisher.blueprint(),
)

__all__ = ["unitree_g1_basic_sim_ros"]
