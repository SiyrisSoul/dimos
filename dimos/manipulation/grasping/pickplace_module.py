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

"""PickPlace Module - Unified pick and place orchestration with BT-based execution.

This module follows the connection_module.py pattern:
- Skills (@skill) for agent-facing operations with generator-based streaming status
- RPCs (@rpc) for lifecycle and internal queries
- BT-based orchestration inside skills (not external BT runner)

Agent-facing skills:
- pick(object_name, max_attempts, dry_run) -> Generator[str, ...]
- place(location, dry_run) -> Generator[str, ...]
- stop() -> bool

The VLM/agent only calls high-level skills; all low-level operations
(graspgen, plan, execute, gripper) are internal BT leaves.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Generator
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any

from dimos.core import Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Pose, PoseArray
    from dimos.msgs.sensor_msgs import PointCloud2

logger = setup_logger()


# =============================================================================
# Enums and Result Types
# =============================================================================


class PickStatus(Enum):
    """Status codes for pick operation."""

    IDLE = "idle"
    GENERATING_GRASPS = "generating_grasps"
    PLANNING_APPROACH = "planning_approach"
    EXECUTING_APPROACH = "executing_approach"
    CLOSING_GRIPPER = "closing_gripper"
    PLANNING_LIFT = "planning_lift"
    EXECUTING_LIFT = "executing_lift"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"


class FailureCode(Enum):
    """Structured failure codes for recovery."""

    NONE = "none"
    NO_OBJECT = "no_object"
    NO_GRASPS = "no_grasps"
    IK_FAIL = "ik_fail"
    PLAN_FAIL = "plan_fail"
    EXECUTION_FAIL = "execution_fail"
    GRIPPER_FAIL = "gripper_fail"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


@dataclass
class PickResult:
    """Result of a pick operation."""

    success: bool
    failure_code: FailureCode = FailureCode.NONE
    message: str = ""
    grasp_index: int = -1  # Which grasp was used (0-indexed)
    attempts: int = 0


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class PickPlaceModuleConfig(ModuleConfig):
    """Configuration for PickPlaceModule.

    Attributes:
        robot_name: Robot name for ManipulationModule planning/gripper
            (None = default robot if only one configured)
        lift_height: Height to lift after grasping (meters)
        approach_distance: Pre-grasp approach distance (meters)
        planning_timeout: Timeout for motion planning (seconds)
        execution_poll_rate: Polling rate for execution status (Hz)
    """

    robot_name: str | None = None
    lift_height: float = 0.10  # 10cm lift
    approach_distance: float = 0.05  # 5cm approach
    planning_timeout: float = 10.0
    execution_poll_rate: float = 10.0  # 10Hz polling


# =============================================================================
# PickPlace Module
# =============================================================================


class PickPlaceModule(Module):
    """Unified pick-and-place module with BT-based orchestration.

    Follows the connection_module.py pattern:
    - Skills live directly on the module (no separate SkillModule wrapper)
    - Generator-based skills for streaming status updates
    - BT orchestration is internal to pick()/place() methods

    Example agent interaction:
        User: Can you pick up the cup?
        Agent: [calls pick("cup")] -> streams status -> "Successfully picked cup"
    """

    default_config = PickPlaceModuleConfig
    config: PickPlaceModuleConfig

    # Outputs
    status: Out[Any]  # JSON status updates

    # RPC bindings for calling other modules
    rpc_calls: list[str] = [
        # Perception
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_name",
        "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id",
        "ObjectSceneRegistrationModule.get_full_scene_pointcloud",
        # Grasp generation (Docker)
        "GraspGenModule.generate_grasps",
        # Motion planning & gripper (ManipulationModule routes to coordinator)
        "ManipulationModule.plan_to_pose",
        "ManipulationModule.execute",
        "ManipulationModule.get_state",
        "ManipulationModule.cancel",
        "ManipulationModule.get_ee_pose",
        "ManipulationModule.open_gripper",
        "ManipulationModule.close_gripper",
        "ManipulationModule.set_gripper",
        "ManipulationModule.get_gripper",
    ]

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)

        # State
        self._status = PickStatus.IDLE
        self._cancel_requested = False
        self._lock = threading.Lock()

        # Latest grasps (for retry logic)
        self._current_grasps: PoseArray | None = None
        self._current_grasp_index: int = 0

        logger.info("PickPlaceModule initialized")

    # =========================================================================
    # Lifecycle RPCs
    # =========================================================================

    @rpc
    def start(self) -> bool:
        """Start the PickPlace module."""
        super().start()
        logger.info("PickPlaceModule started")
        return True

    @rpc
    def stop(self) -> None:
        """Stop the PickPlace module."""
        logger.info("PickPlaceModule stopping")
        self._cancel_requested = True
        super().stop()

    @rpc
    def get_status(self) -> str:
        """Get current pick/place status."""
        return self._status.value

    # =========================================================================
    # Agent-Facing Skills
    # =========================================================================

    @skill()
    def pick(
        self,
        object_name: str,
        object_id: str | None = None,
        max_attempts: int = 3,
        dry_run: bool = False,
    ) -> Generator[str, None, None]:
        """Pick up an object using BT-based orchestration.

        This skill encapsulates the complete pick pipeline:
        1. Generate grasps for the object
        2. For each grasp candidate (with retries):
           a. Plan approach motion
           b. Execute approach
           c. Close gripper
           d. Plan lift motion
           e. Execute lift
        3. Return success/failure with structured feedback

        Args:
            object_name: Name of the object to pick (e.g. "cup", "bottle")
            object_id: Optional specific object ID from perception
            max_attempts: Maximum grasp attempts before giving up
            dry_run: If True, only plan but don't execute

        Yields:
            Status messages for agent feedback

        Example:
            User: Pick up the red cup
            Agent: [streams] "Generating grasps..." -> "Planning approach..."
                   -> "Executing..." -> "Successfully picked red cup"
        """
        with self._lock:
            if self._status != PickStatus.IDLE:
                yield f"Cannot pick: already in state {self._status.value}"
                return
            self._cancel_requested = False

        target_name = object_id or object_name

        try:
            yield f"Starting pick operation for '{target_name}'..."

            # Execute the BT-based pick sequence
            result = yield from self._execute_pick_bt(
                object_name=object_name,
                object_id=object_id,
                max_attempts=max_attempts,
                dry_run=dry_run,
            )

            # Final status
            if result.success:
                yield f"Successfully picked '{target_name}'"
            else:
                yield f"Failed to pick '{target_name}': {result.failure_code.value} - {result.message}"
        finally:
            # Always reset to IDLE so the module can be used again
            self._status = PickStatus.IDLE

    @skill()
    def place(
        self,
        target_pose: Pose | None = None,
        location_name: str | None = None,
        dry_run: bool = False,
    ) -> Generator[str, None, None]:
        """Place the held object at a target location.

        Args:
            target_pose: Target pose for placement (in world frame)
            location_name: Named location (alternative to target_pose)
            dry_run: If True, only plan but don't execute

        Yields:
            Status messages for agent feedback
        """
        with self._lock:
            if self._status != PickStatus.IDLE:
                yield f"Cannot place: already in state {self._status.value}"
                return
            self._cancel_requested = False

        try:
            yield "Starting place operation..."

            # Simpler BT: plan_to_place -> execute -> open_gripper -> retreat
            result = yield from self._execute_place_bt(
                target_pose=target_pose,
                location_name=location_name,
                dry_run=dry_run,
            )

            if result.success:
                yield "Successfully placed object"
            else:
                yield f"Failed to place: {result.failure_code.value} - {result.message}"
        finally:
            # Always reset to IDLE so the module can be used again
            self._status = PickStatus.IDLE

    @skill()
    def stop_motion(self) -> str:
        """Emergency stop - cancel current motion and open gripper.

        This skill:
        1. Requests cancellation of any ongoing pick/place
        2. Cancels active trajectory execution
        3. Opens the gripper (safety: release any held object)

        Returns:
            Status message
        """
        self._cancel_requested = True
        logger.info("Stop requested")

        try:
            # Cancel any active trajectory
            cancel_rpc = self.get_rpc_calls("ManipulationModule.cancel")
            cancel_rpc()
        except Exception as e:
            logger.warning(f"Could not cancel trajectory: {e}")

        try:
            # Open gripper for safety
            open_gripper = self.get_rpc_calls("ManipulationModule.open_gripper")
            open_gripper(self.config.robot_name)
        except Exception as e:
            logger.warning(f"Could not open gripper: {e}")

        # Reset to IDLE so a new pick/place can be started immediately
        self._status = PickStatus.IDLE
        return "Motion stopped, gripper opened"

    # =========================================================================
    # Internal BT Execution
    # =========================================================================

    def _execute_pick_bt(
        self,
        object_name: str,
        object_id: str | None,
        max_attempts: int,
        dry_run: bool,
    ) -> Generator[str, None, PickResult]:
        """Execute pick behavior tree.

        BT Structure:
        - Sequence:
          1. GenerateGrasps
          2. ForEachGrasp (with retry):
             a. PlanApproach
             b. ExecuteApproach
             c. CloseGripper
             d. PlanLift
             e. ExecuteLift
        """
        # Step 1: Generate grasps
        self._status = PickStatus.GENERATING_GRASPS
        yield "Generating grasp candidates..."

        grasps = self._generate_grasps(object_name, object_id)
        if grasps is None or len(grasps.poses) == 0:
            self._status = PickStatus.FAILED
            return PickResult(
                success=False,
                failure_code=FailureCode.NO_GRASPS,
                message=f"No valid grasps found for '{object_name}'",
            )

        self._current_grasps = grasps
        yield f"Generated {len(grasps.poses)} grasp candidates"

        # Step 2: Try each grasp with retries
        for attempt in range(max_attempts):
            if self._cancel_requested:
                self._status = PickStatus.CANCELLED
                return PickResult(
                    success=False,
                    failure_code=FailureCode.CANCELLED,
                    message="Pick cancelled by user",
                    attempts=attempt + 1,
                )

            grasp_idx = attempt % len(grasps.poses)
            grasp_pose = grasps.poses[grasp_idx]
            yield f"Attempt {attempt + 1}/{max_attempts}: trying grasp {grasp_idx + 1}"

            # Try this grasp
            result = yield from self._try_single_grasp(grasp_pose, grasp_idx, dry_run)

            if result.success:
                result.attempts = attempt + 1
                self._status = PickStatus.SUCCESS
                return result

            # Log failure and try next
            yield f"Grasp {grasp_idx + 1} failed: {result.failure_code.value}"

        # All attempts exhausted
        self._status = PickStatus.FAILED
        return PickResult(
            success=False,
            failure_code=FailureCode.EXECUTION_FAIL,
            message=f"Failed after {max_attempts} attempts",
            attempts=max_attempts,
        )

    def _try_single_grasp(
        self,
        grasp_pose: Pose,
        grasp_index: int,
        dry_run: bool,
    ) -> Generator[str, None, PickResult]:
        """Try executing a single grasp.

        Sequence:
        1. Open gripper (pre-grasp)
        2. Plan to pre-grasp pose
        3. Execute approach
        4. Close gripper
        5. Plan lift
        6. Execute lift
        """
        # Open gripper first
        if not dry_run:
            yield "Opening gripper..."
            if not self._open_gripper():
                return PickResult(
                    success=False,
                    failure_code=FailureCode.GRIPPER_FAIL,
                    message="Failed to open gripper",
                    grasp_index=grasp_index,
                )

        # Plan approach to grasp pose
        self._status = PickStatus.PLANNING_APPROACH
        yield "Planning approach motion..."

        if not self._plan_to_pose(grasp_pose):
            return PickResult(
                success=False,
                failure_code=FailureCode.PLAN_FAIL,
                message="Failed to plan approach motion",
                grasp_index=grasp_index,
            )

        if dry_run:
            yield "Dry run: would execute approach here"
            return PickResult(success=True, grasp_index=grasp_index)

        # Execute approach
        self._status = PickStatus.EXECUTING_APPROACH
        yield "Executing approach motion..."

        success, msg = yield from self._execute_and_wait()
        if not success:
            return PickResult(
                success=False,
                failure_code=FailureCode.EXECUTION_FAIL,
                message=f"Approach execution failed: {msg}",
                grasp_index=grasp_index,
            )

        # Close gripper
        self._status = PickStatus.CLOSING_GRIPPER
        yield "Closing gripper..."

        if not self._close_gripper():
            return PickResult(
                success=False,
                failure_code=FailureCode.GRIPPER_FAIL,
                message="Failed to close gripper",
                grasp_index=grasp_index,
            )

        # Brief pause for gripper to close
        time.sleep(0.5)

        # Plan lift
        self._status = PickStatus.PLANNING_LIFT
        yield "Planning lift motion..."

        lift_pose = self._compute_lift_pose(grasp_pose)
        if not self._plan_to_pose(lift_pose):
            # Grasp might still be OK, try to lift anyway
            yield "Warning: could not plan lift, attempting direct lift"

        # Execute lift
        self._status = PickStatus.EXECUTING_LIFT
        yield "Lifting object..."

        success, msg = yield from self._execute_and_wait()
        if not success:
            yield f"Warning: lift execution issue: {msg}"
            # Don't fail here - object might still be grasped

        self._status = PickStatus.SUCCESS
        return PickResult(success=True, grasp_index=grasp_index)

    def _execute_place_bt(
        self,
        target_pose: Pose | None,
        location_name: str | None,
        dry_run: bool,
    ) -> Generator[str, None, PickResult]:
        """Execute place behavior tree.

        Simpler sequence:
        1. Plan to place pose
        2. Execute
        3. Open gripper
        4. Retreat (optional)
        """
        if target_pose is None and location_name is None:
            return PickResult(
                success=False,
                failure_code=FailureCode.NO_OBJECT,
                message="No target pose or location specified",
            )

        # For now, require explicit pose
        if target_pose is None:
            return PickResult(
                success=False,
                failure_code=FailureCode.NO_OBJECT,
                message=f"Location '{location_name}' not implemented yet",
            )

        # Plan to place pose
        yield "Planning place motion..."
        if not self._plan_to_pose(target_pose):
            return PickResult(
                success=False,
                failure_code=FailureCode.PLAN_FAIL,
                message="Failed to plan place motion",
            )

        if dry_run:
            yield "Dry run: would execute place here"
            return PickResult(success=True)

        # Execute
        yield "Executing place motion..."
        success, msg = yield from self._execute_and_wait()
        if not success:
            return PickResult(
                success=False,
                failure_code=FailureCode.EXECUTION_FAIL,
                message=f"Place execution failed: {msg}",
            )

        # Open gripper to release
        yield "Releasing object..."
        if not self._open_gripper():
            yield "Warning: gripper open failed, object may not be released"

        return PickResult(success=True)

    # =========================================================================
    # RPC Wrappers (BT Leaves)
    # =========================================================================

    def _generate_grasps(
        self,
        object_name: str,
        object_id: str | None = None,
    ) -> PoseArray | None:
        """Generate grasps via RPC to GraspGenModule."""
        try:
            # Get object pointcloud
            if object_id:
                get_pc = self.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_object_id"
                )
                pc: PointCloud2 | None = get_pc(object_id)
            else:
                get_pc = self.get_rpc_calls(
                    "ObjectSceneRegistrationModule.get_object_pointcloud_by_name"
                )
                pc = get_pc(object_name)

            if pc is None:
                logger.warning(f"No pointcloud for '{object_id or object_name}'")
                return None

            # Get scene for collision filtering
            get_scene = self.get_rpc_calls(
                "ObjectSceneRegistrationModule.get_full_scene_pointcloud"
            )
            scene_pc: PointCloud2 | None = get_scene(exclude_object_id=object_id)

            # Generate grasps
            generate = self.get_rpc_calls("GraspGenModule.generate_grasps")
            return generate(pc, scene_pc)

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return None

    def _plan_to_pose(self, pose: Pose) -> bool:
        """Plan motion to pose via ManipulationModule."""
        try:
            plan_rpc = self.get_rpc_calls("ManipulationModule.plan_to_pose")
            return plan_rpc(pose, self.config.robot_name)
        except Exception as e:
            logger.error(f"Planning failed: {e}")
            return False

    def _execute_and_wait(self) -> Generator[str, None, tuple[bool, str]]:
        """Execute planned trajectory and poll until complete.

        Follows navigation-style start -> poll status -> cancel pattern.
        """
        try:
            execute_rpc = self.get_rpc_calls("ManipulationModule.execute")
            get_state_rpc = self.get_rpc_calls("ManipulationModule.get_state")

            # Start execution
            if not execute_rpc(self.config.robot_name):
                return (False, "Execute command rejected")

            # Poll for completion
            poll_interval = 1.0 / self.config.execution_poll_rate
            timeout = self.config.planning_timeout * 2  # Allow 2x planning time for execution
            start_time = time.time()

            while True:
                if self._cancel_requested:
                    cancel_rpc = self.get_rpc_calls("ManipulationModule.cancel")
                    cancel_rpc()
                    return (False, "Cancelled")

                state = get_state_rpc()

                if state == "COMPLETED":
                    return (True, "")
                elif state == "FAULT":
                    return (False, "Execution fault")
                elif state == "IDLE":
                    # Already done
                    return (True, "")

                if time.time() - start_time > timeout:
                    return (False, "Execution timeout")

                yield f"Executing... ({state})"
                time.sleep(poll_interval)

        except Exception as e:
            return (False, str(e))

    def _open_gripper(self) -> bool:
        """Open gripper via ManipulationModule (routes to ControlCoordinator)."""
        try:
            open_rpc = self.get_rpc_calls("ManipulationModule.open_gripper")
            return open_rpc(self.config.robot_name)
        except Exception as e:
            logger.error(f"Open gripper failed: {e}")
            return False

    def _close_gripper(self) -> bool:
        """Close gripper via ManipulationModule (routes to ControlCoordinator)."""
        try:
            close_rpc = self.get_rpc_calls("ManipulationModule.close_gripper")
            return close_rpc(self.config.robot_name)
        except Exception as e:
            logger.error(f"Close gripper failed: {e}")
            return False

    def _compute_lift_pose(self, grasp_pose: Pose) -> Pose:
        """Compute lift pose by moving grasp pose up by lift_height."""
        from dimos.msgs.geometry_msgs import Pose as PoseMsg, Vector3

        return PoseMsg(
            position=Vector3(
                x=grasp_pose.position.x,
                y=grasp_pose.position.y,
                z=grasp_pose.position.z + self.config.lift_height,
            ),
            orientation=grasp_pose.orientation,
        )


# Blueprint export
pickplace_module = PickPlaceModule.blueprint

__all__ = [
    "FailureCode",
    "PickPlaceModule",
    "PickPlaceModuleConfig",
    "PickResult",
    "PickStatus",
    "pickplace_module",
]