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

"""Grasping and pick-place manipulation.

Modules:
- GraspGenModule: Docker-based neural network grasp generation
- PickPlaceModule: Unified pick/place orchestration with BT-based execution

The PickPlaceModule is the agent-facing interface for grasping operations.
It internally calls GraspGenModule for grasp generation.
"""

from dimos.manipulation.grasping.graspgen_module import (
    GraspGenConfig,
    GraspGenModule,
    graspgen,
)
from dimos.manipulation.grasping.pickplace_module import (
    FailureCode,
    PickPlaceModule,
    PickPlaceModuleConfig,
    PickResult,
    PickStatus,
    pickplace_module,
)

__all__ = [
    # GraspGen (Docker neural net)
    "GraspGenConfig",
    "GraspGenModule",
    "graspgen",
    # PickPlace (agent-facing orchestration)
    "FailureCode",
    "PickPlaceModule",
    "PickPlaceModuleConfig",
    "PickResult",
    "PickStatus",
    "pickplace_module",
]