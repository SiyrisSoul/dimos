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

"""Piper manipulator driver.

This driver:
- Uses PiperBackend by default (injectable for testing)
- Has full control over threading and timing
- Can be customized without affecting other arms
"""

from dataclasses import dataclass, field
import threading
import time

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.hardware.manipulators.piper.backend import PiperBackend
from dimos.hardware.manipulators.spec import (
    ControlMode,
    DriverStatus,
    JointLimits,
    ManipulatorBackend,
    ManipulatorInfo,
    default_base_transform,
)
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState

# ============================================================================
# CONFIGURATION
# ============================================================================


@dataclass
class PiperConfig(ModuleConfig):
    """Piper-specific configuration.

    Type-safe config with Piper-specific fields.
    """

    can_port: str = "can0"
    dof: int = 6  # Piper is always 6-DOF
    control_rate: float = 100.0  # Hz
    monitor_rate: float = 10.0  # Hz
    has_gripper: bool = False
    base_frame_id: str = "base_link"
    base_transform: Transform | None = field(default_factory=default_base_transform)


# ============================================================================
# DRIVER
# ============================================================================


class Piper(Module[PiperConfig]):
    """Piper manipulator driver.

    Features:
    - Injectable backend (defaults to PiperBackend)
    - 100Hz control loop for joint state
    - 10Hz monitor loop for robot state
    - Full RPC interface for control

    Example:
        >>> arm = Piper(can_port="can0")
        >>> arm.start()
        >>> arm.enable_servos()
        >>> arm.move_joint([0, 0, 0, 0, 0, 0])

    Testing:
        >>> from dimos.hardware.manipulators.mock import MockBackend
        >>> arm = Piper(backend=MockBackend())
        >>> arm.start()  # No hardware needed!
    """

    # Input topics (commands from controllers)
    joint_position_command: In[JointCommand]
    joint_velocity_command: In[JointCommand]

    # Output topics
    joint_state: Out[JointState]
    robot_state: Out[RobotState]

    # Config
    config: PiperConfig
    default_config = PiperConfig

    def __init__(
        self,
        backend: ManipulatorBackend | None = None,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)

        # Backend is injectable for testing (pass config to backend __init__)
        self.backend: ManipulatorBackend = backend or PiperBackend(
            can_port=self.config.can_port,
            dof=self.config.dof,
        )

        # Threading state
        self._running = False
        self._control_thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None

        # Cached info
        self._dof: int = self.config.dof
        self._joint_names: list[str] = [f"joint{i + 1}" for i in range(self._dof)]

        # Auto-connect on initialization
        self._auto_start()

    def _auto_start(self) -> None:
        """Auto-connect to hardware on initialization."""
        if not self.backend.connect():
            print(f"WARNING: Failed to connect to Piper on {self.config.can_port}")
            return

        # Update DOF from backend (in case it differs)
        self._dof = self.backend.get_dof()
        self._joint_names = [f"joint{i + 1}" for i in range(self._dof)]

        # Start threads
        self._running = True

        self._control_thread = threading.Thread(
            target=self._control_loop,
            name="Piper-Control",
            daemon=True,
        )
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="Piper-Monitor",
            daemon=True,
        )

        self._control_thread.start()
        self._monitor_thread.start()

        print(f"Piper connected on {self.config.can_port}, DOF={self._dof}")

    def _subscribe_to_commands(self) -> None:
        """Subscribe to command input topics."""
        try:
            if self.joint_position_command:
                self.joint_position_command.subscribe(self._on_joint_position_command)
                print("Subscribed to joint_position_command")
        except Exception as e:
            print(f"Could not subscribe to joint_position_command: {e}")

        try:
            if self.joint_velocity_command:
                self.joint_velocity_command.subscribe(self._on_joint_velocity_command)
                print("Subscribed to joint_velocity_command")
        except Exception as e:
            print(f"Could not subscribe to joint_velocity_command: {e}")

    def _on_joint_position_command(self, cmd: JointCommand) -> None:
        """Handle incoming joint position command."""
        if not self._running:
            return

        positions = list(cmd.positions) if cmd.positions else []
        if len(positions) != self._dof:
            print(f"WARNING: Position command has {len(positions)} joints, expected {self._dof}")
            return

        self.backend.write_joint_positions(positions)

    def _on_joint_velocity_command(self, cmd: JointCommand) -> None:
        """Handle incoming joint velocity command."""
        if not self._running:
            return

        # JointCommand uses 'positions' field for velocity commands too
        velocities = list(cmd.positions) if cmd.positions else []
        if len(velocities) != self._dof:
            print(f"WARNING: Velocity command has {len(velocities)} joints, expected {self._dof}")
            return

        self.backend.write_joint_velocities(velocities)

    # =========================================================================
    # Lifecycle
    # =========================================================================

    @rpc
    def start(self) -> DriverStatus:
        """Connect to Piper and start control loops (if not already running)."""
        super().start()  # Important: sets up transports

        if self._running:
            # Already connected, just subscribe to commands
            self._subscribe_to_commands()
            return DriverStatus.CONNECTED

        self._auto_start()
        self._subscribe_to_commands()
        return DriverStatus.CONNECTED if self._running else DriverStatus.ERROR

    @rpc
    def stop(self) -> None:
        """Stop Piper and disconnect."""
        self._running = False

        # Stop motion first
        try:
            self.backend.write_stop()
        except Exception:
            pass

        # Wait for threads
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)

        # Disconnect
        try:
            self.backend.disconnect()
        except Exception:
            pass

        super().stop()

    # =========================================================================
    # Control Loop (100Hz)
    # =========================================================================

    def _control_loop(self) -> None:
        """High-frequency loop for joint state publishing."""
        period = 1.0 / self.config.control_rate

        while self._running:
            start = time.perf_counter()

            try:
                self._publish_joint_state()
            except Exception as e:
                print(f"Piper control loop error: {e}")

            # Rate control
            elapsed = time.perf_counter() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _publish_joint_state(self) -> None:
        """Read and publish joint state."""
        positions = self.backend.read_joint_positions()
        velocities = self.backend.read_joint_velocities()
        efforts = self.backend.read_joint_efforts()

        msg = JointState(
            ts=time.time(),
            frame_id="joint_state",
            name=self._joint_names,
            position=positions,
            velocity=velocities,
            effort=efforts,
        )
        self.joint_state.publish(msg)

    # =========================================================================
    # Monitor Loop (10Hz)
    # =========================================================================

    def _monitor_loop(self) -> None:
        """Low-frequency loop for robot state monitoring."""
        period = 1.0 / self.config.monitor_rate

        while self._running:
            start = time.perf_counter()

            try:
                self._publish_robot_state()
            except Exception as e:
                print(f"Piper monitor loop error: {e}")

            # Rate control
            elapsed = time.perf_counter() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _publish_robot_state(self) -> None:
        """Read and publish robot state."""
        state = self.backend.read_state()
        error_code, _ = self.backend.read_error()

        msg = RobotState(
            state=state.get("state", 0),
            mode=state.get("mode", 0),
            error_code=error_code,
            warn_code=0,
        )
        self.robot_state.publish(msg)

    # =========================================================================
    # RPC Methods - Servo Control
    # =========================================================================

    @rpc
    def enable_servos(self) -> bool:
        """Enable motor control."""
        return self.backend.write_enable(True)

    @rpc
    def disable_servos(self) -> bool:
        """Disable motor control."""
        return self.backend.write_enable(False)

    @rpc
    def clear_errors(self) -> bool:
        """Clear error state."""
        return self.backend.write_clear_errors()

    # =========================================================================
    # RPC Methods - Mode Control
    # =========================================================================

    @rpc
    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode (position, velocity, etc)."""
        return self.backend.set_control_mode(mode)

    @rpc
    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self.backend.get_control_mode()

    # =========================================================================
    # RPC Methods - Joint Space Motion
    # =========================================================================

    @rpc
    def move_joint(self, positions: list[float], velocity: float = 0.5) -> bool:
        """Move to joint positions (radians)."""
        return self.backend.write_joint_positions(positions, velocity)

    @rpc
    def move_velocity(self, velocities: list[float]) -> bool:
        """Move with joint velocities (rad/s).

        Note: Piper doesn't support native velocity control.
        This will return False.
        """
        return self.backend.write_joint_velocities(velocities)

    @rpc
    def stop_motion(self) -> bool:
        """Stop all motion."""
        return self.backend.write_stop()

    # =========================================================================
    # RPC Methods - Cartesian Motion
    # =========================================================================

    @rpc
    def get_cartesian_position(self) -> dict[str, float] | None:
        """Get end-effector pose (x, y, z in meters; roll, pitch, yaw in radians)."""
        return self.backend.read_cartesian_position()

    @rpc
    def move_cartesian(
        self,
        pose: dict[str, float],
        velocity: float = 0.5,
    ) -> bool:
        """Move to cartesian pose.

        Args:
            pose: Dict with x, y, z (meters), roll, pitch, yaw (radians)
            velocity: Speed as fraction of max (0-1)

        Note: Piper may not support direct cartesian control.
        """
        return self.backend.write_cartesian_position(pose, velocity)

    # =========================================================================
    # RPC Methods - Info
    # =========================================================================

    @rpc
    def get_info(self) -> ManipulatorInfo:
        """Get manipulator info."""
        return self.backend.get_info()

    @rpc
    def get_limits(self) -> JointLimits:
        """Get joint limits."""
        return self.backend.get_limits()

    # =========================================================================
    # RPC Methods - Optional Features (Gripper)
    # =========================================================================

    @rpc
    def get_gripper_position(self) -> float | None:
        """Get gripper position (meters). None if no gripper."""
        if not self.config.has_gripper:
            return None
        return self.backend.read_gripper_position()

    @rpc
    def set_gripper_position(self, position: float) -> bool:
        """Set gripper position (meters)."""
        if not self.config.has_gripper:
            return False
        return self.backend.write_gripper_position(position)


# ============================================================================
# EXPORTS
# ============================================================================

piper = Piper.blueprint

__all__ = ["Piper", "PiperConfig", "piper"]
