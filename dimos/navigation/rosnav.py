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

"""
NavBot class for navigation-related functionality.
Encapsulates ROS transport and topic remapping for Unitree robots.
"""

from dataclasses import dataclass, field
import logging
from pathlib import Path
import shutil
import threading
import time
from typing import Any

from reactivex import operators as ops
from reactivex.subject import Subject

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core import DimosCluster, In, LCMTransport, Module, Out, rpc
from dimos.core.docker_runner import DockerModuleConfig
from dimos.core.transport import ROSTransport
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    TwistStamped,
    Vector3,
)
from dimos.msgs.nav_msgs import Path as NavPath
from dimos.msgs.sensor_msgs import Joy, PointCloud2
from dimos.msgs.std_msgs import Bool, Int8
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger(level=logging.INFO)


@dataclass
class ROSNavConfig(DockerModuleConfig):
    # --- Docker settings ---
    docker_image: str = "dimos_autonomy_stack:humble"
    docker_shm_size: str = "8g"
    docker_entrypoint: str = "/usr/local/bin/dimos_module_entrypoint.sh"
    docker_file: Path = Path(__file__).parent.parent.parent / "docker" / "navigation" / "Dockerfile"
    docker_build_context: Path = Path(__file__).parent.parent.parent
    # No --gpus flag; Jetson runtime added via _nvidia_extra_args() when nvcc is present.
    docker_gpus: str | None = None
    docker_extra_args: list = field(
        default_factory=lambda: ["--runtime=nvidia"] if shutil.which("nvcc") else []
    )
    docker_env: dict = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "42",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_ws/config/fastdds.xml",
            "START_ROS_NAV": "true",
            "START_UNITY_SIM": "false",
            # "ROBOT_CONFIG_PATH": "unitree/unitree_g1",
        }
    )
    docker_volumes: list = field(default_factory=lambda: [])

    # --- Simulation settings ---
    unity_simulation: bool = False

    # --- Module settings ---
    local_pointcloud_freq: float = 2.0
    global_map_freq: float = 1.0
    sensor_to_base_link_transform: Transform = field(
        default_factory=lambda: Transform(frame_id="sensor", child_frame_id="base_link")
    )

    def __post_init__(self) -> None:
        self.docker_env["START_UNITY_SIM"] = "true" if self.unity_simulation else "false"
        self.docker_volumes += [
            # Mount live dimos source so the module is always up-to-date
            (str(Path(__file__).parent.parent.parent), "/workspace/dimos", "rw"),
            # Mount entrypoint script so changes don't require a rebuild
            (
                str(Path(__file__).parent / "dimos_module_entrypoint.sh"),
                "/usr/local/bin/dimos_module_entrypoint.sh",
                "ro",
            ),
        ]


class ROSNav(
    Module, NavigationInterface, spec.Nav, spec.GlobalPointcloud, spec.Pointcloud, spec.LocalPlanner
):
    config: ROSNavConfig
    default_config = ROSNavConfig

    @classproperty
    def blueprint(self) -> Any:  # type: ignore[override]
        # super().blueprint doesn't work with classproperty descriptors (Python passes the
        # metaclass as `cls` rather than ROSNav).  Blueprint.create(cls, ...) is the exact
        # equivalent of what Module.blueprint.fget(cls) returns a partial of.
        from dimos.core.blueprints import Blueprint

        def factory(*args: Any, **kwargs: Any) -> Any:
            return Blueprint.create(self, *args, **kwargs).transports(
                {
                    ("ros_goal_reached", Bool): ROSTransport("/goal_reached", Bool),
                    ("ros_cmd_vel", TwistStamped): ROSTransport("/cmd_vel", TwistStamped),
                    ("ros_way_point", PoseStamped): ROSTransport("/way_point", PoseStamped),
                    ("ros_registered_scan", PointCloud2): ROSTransport(
                        "/registered_scan", PointCloud2
                    ),
                    ("ros_global_map", PointCloud2): ROSTransport("/terrain_map_ext", PointCloud2),
                    ("ros_path", NavPath): ROSTransport("/path", NavPath),
                    ("ros_tf", TFMessage): ROSTransport("/tf", TFMessage),
                    ("ros_goal_pose", PoseStamped): ROSTransport("/goal_pose", PoseStamped),
                    ("ros_cancel_goal", Bool): ROSTransport("/cancel_goal", Bool),
                    ("ros_soft_stop", Int8): ROSTransport("/stop", Int8),
                    ("ros_joy", Joy): ROSTransport("/joy", Joy),
                }
            )

        return factory

    # Existing ports (default LCM/pSHM transport)
    goal_req: In[PoseStamped]

    pointcloud: Out[PointCloud2]
    global_map: Out[PointCloud2]

    goal_active: Out[PoseStamped]
    path_active: Out[NavPath]
    cmd_vel: Out[Twist]

    # ROS In ports (receiving from ROS topics via ROSTransport)
    ros_goal_reached: In[Bool]
    ros_cmd_vel: In[TwistStamped]
    ros_way_point: In[PoseStamped]
    ros_registered_scan: In[PointCloud2]
    ros_global_map: In[PointCloud2]
    ros_path: In[NavPath]
    ros_tf: In[TFMessage]

    # ROS Out ports (publishing to ROS topics via ROSTransport)
    ros_goal_pose: Out[PoseStamped]
    ros_cancel_goal: Out[Bool]
    ros_soft_stop: Out[Int8]
    ros_joy: Out[Joy]

    # Using RxPY Subjects for reactive data flow instead of storing state
    _local_pointcloud_subject: Subject  # type: ignore[type-arg]
    _global_map_subject: Subject  # type: ignore[type-arg]

    _current_position_running: bool = False
    _goal_reach: bool | None = None

    # Navigation state tracking for NavigationInterface
    _navigation_state: NavigationState = NavigationState.IDLE
    _state_lock: threading.Lock
    _navigation_thread: threading.Thread | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Initialize RxPY Subjects for streaming data
        self._local_pointcloud_subject = Subject()
        self._global_map_subject = Subject()

        # Initialize state tracking
        self._state_lock = threading.Lock()
        self._navigation_state = NavigationState.IDLE
        self._goal_reached = False

        logger.info("NavigationModule initialized")

    @rpc
    def start(self) -> None:
        self._running = True
        # ROS transport setup (rclpy node + topic discovery) can take several
        # seconds, which exceeds RPC_READY_TIMEOUT (3 s).  Do it in a background
        # thread so start() returns immediately and the host considers the module
        # ready without timing out.
        threading.Thread(target=self._start_ros, daemon=True, name="ROSNavStartThread").start()

    def _start_ros(self) -> None:
        self._disposables.add(
            self._local_pointcloud_subject.pipe(
                ops.sample(1.0 / self.config.local_pointcloud_freq),
            ).subscribe(
                on_next=self.pointcloud.publish,
                on_error=lambda e: logger.error(f"Lidar stream error: {e}"),
            )
        )

        self._disposables.add(
            self._global_map_subject.pipe(
                ops.sample(1.0 / self.config.global_map_freq),
            ).subscribe(
                on_next=self.global_map.publish,
                on_error=lambda e: logger.error(f"Map stream error: {e}"),
            )
        )

        # Subscribe to ROS In ports
        self.ros_goal_reached.subscribe(self._on_ros_goal_reached)
        self.ros_cmd_vel.subscribe(self._on_ros_cmd_vel)
        self.ros_way_point.subscribe(self._on_ros_goal_waypoint)
        self.ros_registered_scan.subscribe(self._on_ros_registered_scan)
        self.ros_global_map.subscribe(self._on_ros_global_map)
        self.ros_path.subscribe(self._on_ros_path)
        self.ros_tf.subscribe(self._on_ros_tf)

        self.goal_req.subscribe(self._on_goal_pose)
        logger.info("NavigationModule started with ROS transport and RxPY streams")

    def _on_ros_goal_reached(self, msg: Bool) -> None:
        self._goal_reach = msg.data
        if msg.data:
            with self._state_lock:
                self._goal_reached = True
                self._navigation_state = NavigationState.IDLE

    def _on_ros_goal_waypoint(self, msg: PoseStamped) -> None:
        self.goal_active.publish(msg)

    def _on_ros_cmd_vel(self, msg: TwistStamped) -> None:
        self.cmd_vel.publish(Twist(linear=msg.linear, angular=msg.angular))

    def _on_ros_registered_scan(self, msg: PointCloud2) -> None:
        self._local_pointcloud_subject.on_next(msg)

    def _on_ros_global_map(self, msg: PointCloud2) -> None:
        self._global_map_subject.on_next(msg)

    def _on_ros_path(self, msg: NavPath) -> None:
        msg.frame_id = "base_link"
        self.path_active.publish(msg)

    def _on_ros_tf(self, msg: TFMessage) -> None:
        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=euler_to_quaternion(Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        self.tf.publish(
            self.config.sensor_to_base_link_transform.now(),
            map_to_world_tf,
            *msg.transforms,
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool) -> None:
        if msg.data:
            self.stop()

    def _set_autonomy_mode(self) -> None:
        joy_msg = Joy(
            axes=[0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0],
            buttons=[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        )
        self.ros_joy.publish(joy_msg)
        logger.info("Setting autonomy mode via Joy message")

    @skill
    def goto(self, x: float, y: float) -> str:
        """
        move the robot in relative coordinates
        x is forward, y is left

        goto(1, 0) will move the robot forward by 1 meter
        """
        pose_to = PoseStamped(
            position=Vector3(x, y, 0),
            orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
            frame_id="base_link",
            ts=time.time(),
        )

        self.navigate_to(pose_to)
        return "arrived"

    @skill
    def goto_global(self, x: float, y: float) -> str:
        """
        go to coordinates x,y in the map frame
        0,0 is your starting position
        """
        target = PoseStamped(
            ts=time.time(),
            frame_id="map",
            position=Vector3(x, y, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
        )

        self.navigate_to(target)

        return f"arrived to {x:.2f}, {y:.2f}"

    @rpc
    def navigate_to(self, pose: PoseStamped, timeout: float = 60.0) -> bool:
        """
        Navigate to a target pose by publishing to ROS topics.

        Args:
            pose: Target pose to navigate to
            timeout: Maximum time to wait for goal (seconds)

        Returns:
            True if navigation was successful
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f} @ {pose.frame_id})"
        )

        self._goal_reach = None
        self._set_autonomy_mode()

        # Enable soft stop (0 = enable)
        self.ros_soft_stop.publish(Int8(data=0))
        self.ros_goal_pose.publish(pose)

        # Wait for goal to be reached
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._goal_reach is not None:
                self.ros_soft_stop.publish(Int8(data=2))
                return self._goal_reach
            time.sleep(0.1)

        self.stop_navigation()
        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    @rpc
    def stop_navigation(self) -> bool:
        """
        Stop current navigation by publishing to ROS topics.

        Returns:
            True if stop command was sent successfully
        """
        logger.info("Stopping navigation")

        self.ros_cancel_goal.publish(Bool(data=True))
        self.ros_soft_stop.publish(Int8(data=2))

        with self._state_lock:
            self._navigation_state = NavigationState.IDLE
            self._current_goal = None
            self._goal_reached = False

        return True

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        """Set a new navigation goal (non-blocking)."""
        with self._state_lock:
            self._current_goal = goal
            self._goal_reached = False
            self._navigation_state = NavigationState.FOLLOWING_PATH

        # Start navigation in a separate thread to make it non-blocking
        if self._navigation_thread and self._navigation_thread.is_alive():
            logger.warning("Previous navigation still running, cancelling")
            self.stop_navigation()
            self._navigation_thread.join(timeout=1.0)

        self._navigation_thread = threading.Thread(
            target=self._navigate_to_goal_async,
            args=(goal,),
            daemon=True,
            name="ROSNavNavigationThread",
        )
        self._navigation_thread.start()

        return True

    def _navigate_to_goal_async(self, goal: PoseStamped) -> None:
        """Internal method to handle navigation in a separate thread."""
        try:
            result = self.navigate_to(goal, timeout=60.0)
            with self._state_lock:
                self._goal_reached = result
                self._navigation_state = NavigationState.IDLE
        except Exception as e:
            logger.error(f"Navigation failed: {e}")
            with self._state_lock:
                self._goal_reached = False
                self._navigation_state = NavigationState.IDLE

    @rpc
    def get_state(self) -> NavigationState:
        """Get the current state of the navigator."""
        with self._state_lock:
            return self._navigation_state

    @rpc
    def is_goal_reached(self) -> bool:
        """Check if the current goal has been reached."""
        with self._state_lock:
            return self._goal_reached

    @rpc
    def cancel_goal(self) -> bool:
        """Cancel the current navigation goal."""

        with self._state_lock:
            had_goal = self._current_goal is not None

        if had_goal:
            self.stop_navigation()

        return had_goal

    @rpc
    def stop(self) -> None:
        """Stop the navigation module and clean up resources."""
        self.stop_navigation()
        try:
            self._running = False

            self._local_pointcloud_subject.on_completed()
            self._global_map_subject.on_completed()

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        finally:
            super().stop()


ros_nav = ROSNav.blueprint


def deploy(dimos: DimosCluster) -> Any:  # type: ignore[no-untyped-def]
    """
    Deploy ROSNav as a DockerModule.

    The container runs both the ROS navigation stack (in the background via
    dimos_module_entrypoint.sh) and the ROSNav DimOS module.  ROS transports
    are wired up inside the container by ROSNav.start(); only the external
    LCM ports are configured here from the host side.
    """
    nav = dimos.deploy(ROSNav)

    # External (LCM) ports - configured from host, bridged into the container
    nav.set_transport("pointcloud", LCMTransport("/lidar", PointCloud2))
    nav.set_transport("global_map", LCMTransport("/map", PointCloud2))
    nav.set_transport("goal_req", LCMTransport("/goal_req", PoseStamped))
    nav.set_transport("goal_active", LCMTransport("/goal_active", PoseStamped))
    nav.set_transport("path_active", LCMTransport("/path_active", NavPath))
    nav.set_transport("cmd_vel", LCMTransport("/cmd_vel", Twist))

    # ROS transports are pre-configured at blueprint level via ros_nav() - no host-side setup needed.

    nav.start()
    return nav


__all__ = ["ROSNav", "ROSNavConfig", "deploy", "ros_nav"]
