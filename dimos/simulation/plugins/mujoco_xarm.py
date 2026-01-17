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

"""MuJoCo engine registration with xArm robot specs."""

from __future__ import annotations

import math

from dimos.hardware.manipulators.spec import JointLimits
from dimos.simulation.engines.base import RobotSpec
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.registry import registry


def _xarm_limits(dof: int) -> JointLimits:
    if dof == 7:
        lower_deg = [-360, -118, -360, -233, -360, -97, -360]
        upper_deg = [360, 118, 360, 11, 360, 180, 360]
    elif dof == 6:
        lower_deg = [-360, -118, -225, -11, -360, -97]
        upper_deg = [360, 118, 11, 225, 360, 180]
    else:
        lower_deg = [-360, -118, -225, -97, -360]
        upper_deg = [360, 118, 11, 180, 360]

    lower_rad = [math.radians(d) for d in lower_deg[:dof]]
    upper_rad = [math.radians(d) for d in upper_deg[:dof]]
    max_vel_rad = math.radians(180.0)
    return JointLimits(
        position_lower=lower_rad,
        position_upper=upper_rad,
        velocity_max=[max_vel_rad] * dof,
    )


def register() -> None:
    registry.register_engine("mujoco", MujocoEngine)

    registry.register_robot(
        "xarm7",
        RobotSpec(
            name="xarm7",
            engine="mujoco",
            asset="xarm7_mj_description",
            dof=7,
            vendor="UFACTORY",
            model="xArm7",
            limits=_xarm_limits(7),
        ),
    )
    registry.register_robot(
        "xarm6",
        RobotSpec(
            name="xarm6",
            engine="mujoco",
            asset="xarm6_mj_description",
            dof=6,
            vendor="UFACTORY",
            model="xArm6",
            limits=_xarm_limits(6),
        ),
    )
    registry.register_robot(
        "xarm5",
        RobotSpec(
            name="xarm5",
            engine="mujoco",
            asset="xarm5_mj_description",
            dof=5,
            vendor="UFACTORY",
            model="xArm5",
            limits=_xarm_limits(5),
        ),
    )


__all__ = [
    "register",
]
