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

"""
ArduPilot LCM Module - Wrapper around pymavlink for ArduPilot communication
Publishes sensor data and mission information via LCM topics
"""

import time
import math
import threading
import logging
import os
from typing import Dict, Any, Optional, List
import numpy as np

from reactivex import interval
from reactivex import operators as ops

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
    logging.warning("pymavlink not found. Please install pymavlink to use ArduPilot functionality.")

from dimos.core import Module, Out, In, rpc
from dimos.utils.logging_config import setup_logger

# Import standard LCM message types (always available)
from dimos_lcm.sensor_msgs import (
    NavSatFix,
    NavSatStatus,
    Imu,
    MagneticField,
    FluidPressure,
    Temperature,
    BatteryState,
    NavSatStatus,
)
from dimos_lcm.nav_msgs import Odometry, Path
from dimos_lcm.geometry_msgs import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
    Vector3,
    PoseWithCovariance,
    TwistWithCovariance,
    AccelStamped,
    Accel,
    TwistStamped,
)
from dimos_lcm.std_msgs import Header, Time, Float64, String, UInt16, UInt8, Bool
from dimos_lcm.diagnostic_msgs import DiagnosticArray
from dimos.protocol.tf import TF
from dimos.msgs.geometry_msgs import Transform


# Set msg_name for all geometry_msgs types
PoseStamped.msg_name = "geometry_msgs.PoseStamped"
Pose.msg_name = "geometry_msgs.Pose"
Point.msg_name = "geometry_msgs.Point"
Quaternion.msg_name = "geometry_msgs.Quaternion"
Twist.msg_name = "geometry_msgs.Twist"
Vector3.msg_name = "geometry_msgs.Vector3"
PoseWithCovariance.msg_name = "geometry_msgs.PoseWithCovariance"
TwistWithCovariance.msg_name = "geometry_msgs.TwistWithCovariance"
AccelStamped.msg_name = "geometry_msgs.AccelStamped"
Accel.msg_name = "geometry_msgs.Accel"
TwistStamped.msg_name = "geometry_msgs.TwistStamped"

# Try to import MAVROS-specific message types (conditional functionality)
MAVROS_MSGS_AVAILABLE = False
try:
    from dimos_lcm.mavros_msgs import (
        ActuatorControl,
        ExtendedState,
        State,
        ManualControl,
        OverrideRCIn,
        RCIn,
        RCOut,
        StatusEvent,
        StatusText,
        Altitude,
        WindEstimation,
    )

    MAVROS_MSGS_AVAILABLE = True
    print("MAVROS message types available - full functionality enabled")
except ImportError:
    print("MAVROS message types not available - using standard ROS messages only")

    # Define dummy classes for MAVROS messages to prevent errors
    class ActuatorControl:
        pass

    class ExtendedState:
        pass

    class State:
        pass

    class ManualControl:
        pass

    class OverrideRCIn:
        pass

    class RCIn:
        pass

    class RCOut:
        pass

    class StatusEvent:
        pass

    class StatusText:
        pass

    class Altitude:
        pass

    class WindEstimation:
        pass


logger = setup_logger(__name__)


class ArduPilotInterface:
    """Low-level ArduPilot interface using pymavlink."""

    def __init__(self, connection_string: str = "/dev/ttyACM0", baudrate: int = 57600):
        """
        Initialize ArduPilot connection.

        Args:
            connection_string: MAVLink connection string
                Examples:
                - 'udp:127.0.0.1:14550' for SITL
                - '/dev/ttyACM0' for serial connection
                - 'tcp:127.0.0.1:5760' for TCP connection
            baudrate: Serial baudrate (if using serial connection)
        """
        if mavutil is None:
            raise ImportError("pymavlink not installed. Please install pymavlink package.")

        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.is_connected = False

        # Data storage
        self.global_position = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "relative_alt": 0.0}
        self.gps_status = {"fix_type": 0, "satellites": 0, "hdop": 0.0, "vdop": 0.0}
        self.local_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.angular_velocity = {"rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
        self.pose_covariance = [0.0] * 36  # 6x6 covariance matrix
        self.twist_covariance = [0.0] * 36  # 6x6 covariance matrix
        self.waypoints = []

        # Additional sensor data
        self.imu_data = {
            "linear_acceleration": [0.0, 0.0, 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
        }
        self.magnetic_field = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.pressure_data = {"pressure": 0.0, "altitude": 0.0, "temperature": 0.0}
        self.battery_data = {"voltage": 0.0, "current": 0.0, "remaining": 0.0}
        self.rc_data = {"channels": [0] * 18}
        self.vehicle_state = {"armed": False, "connected": False, "guided": False, "mode": ""}
        self.extended_state = {"vtol_state": 0, "landed_state": 0}

        # Additional data structures for other message types
        self.vfr_hud_data = {
            "airspeed": 0.0,
            "groundspeed": 0.0,
            "heading": 0,
            "throttle": 0,
            "alt": 0.0,
            "climb": 0.0,
        }

        # Timestamps
        self.last_global_position_time = 0
        self.last_local_position_time = 0
        self.last_attitude_time = 0

    def connect(self) -> bool:
        """Connect to ArduPilot."""
        try:
            print(f"Connecting to ArduPilot on {self.connection_string}...")

            # Handle serial connections with baudrate
            if self.connection_string.startswith("/dev/"):
                self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            else:
                self.master = mavutil.mavlink_connection(self.connection_string)

            # Wait for the first heartbeat with timeout
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)

            print(
                f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}"
            )

            # Request data streams
            self.request_data_streams()

            self.is_connected = True
            print("ArduPilot connected successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to ArduPilot: {e}")
            return False

    def request_data_streams(self):
        """Request specific data streams from ArduPilot."""
        if not self.master:
            return

        try:
            # Request position, attitude, and GPS data at 10Hz
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,
                1,
            )

            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                10,
                1,
            )

            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                10,
                1,
            )

            print("Data streams requested")

        except Exception as e:
            logger.error(f"Error requesting data streams: {e}")

    def send_heartbeat(self):
        """Send heartbeat to ArduPilot."""
        if not self.master or not self.is_connected:
            return

        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception as e:
            logger.error(f"Error sending heartbeat: {e}")
            # Mark connection as invalid on heartbeat failure
            if "Bad file descriptor" in str(e) or "write failed" in str(e):
                self.is_connected = False
                logger.warning("Connection marked as invalid due to heartbeat error")

    def update_telemetry(self):
        """Update telemetry data from incoming MAVLink messages."""
        if not self.master:
            return

        try:
            # Check if connection is still valid
            if not self.is_connection_valid():
                logger.warning("Connection lost, attempting to reconnect...")
                if not self.reconnect():
                    return

            # Process all available messages
            while True:
                try:
                    msg = self.master.recv_match(blocking=False)
                    if msg is None:
                        break

                    msg_type = msg.get_type()
                    if msg_type == "GLOBAL_POSITION_INT":
                        # Global position data - match MAVROS scaling exactly
                        # Latitude and longitude: degrees (already scaled by 1E7 in MAVROS)
                        self.global_position["lat"] = msg.lat / 1e7  # deg
                        self.global_position["lon"] = msg.lon / 1e7  # deg
                        self.global_position["alt"] = msg.alt / 1000.0  # MSL altitude in meters
                        self.global_position["relative_alt"] = (
                            msg.relative_alt / 1000.0
                        )  # relative altitude in meters

                        # Velocity: cm/s to m/s (match MAVROS)
                        self.velocity["vx"] = msg.vx / 100.0  # cm/s to m/s
                        self.velocity["vy"] = msg.vy / 100.0  # cm/s to m/s
                        self.velocity["vz"] = msg.vz / 100.0  # cm/s to m/s
                        self.last_global_position_time = time.time()

                    elif msg_type == "LOCAL_POSITION_NED":
                        # Convert NED to ENU frame when receiving data
                        # Position: NED -> ENU (x_ned -> y_enu, y_ned -> x_enu, z_ned -> -z_enu)
                        self.local_position["x"] = msg.y  # NED y -> ENU x
                        self.local_position["y"] = msg.x  # NED x -> ENU y
                        self.local_position["z"] = -msg.z  # NED z -> ENU z (negated)

                        # Velocity: NED -> ENU (vx_ned -> vy_enu, vy_ned -> vx_enu, vz_ned -> -vz_enu)
                        self.velocity["vx"] = msg.vy  # NED vy -> ENU vx
                        self.velocity["vy"] = msg.vx  # NED vx -> ENU vy
                        self.velocity["vz"] = -msg.vz  # NED vz -> ENU vz (negated)
                        self.last_local_position_time = time.time()

                    elif msg_type == "ATTITUDE":
                        # Convert NED to ENU frame when receiving data
                        # Orientation: NED -> ENU (roll_ned -> -pitch_enu, pitch_ned -> -roll_enu, yaw_ned -> yaw_enu)
                        self.orientation["roll"] = msg.roll  # NED pitch -> ENU roll (negated)
                        self.orientation["pitch"] = -msg.pitch  # NED roll -> ENU pitch (negated)
                        self.orientation["yaw"] = np.pi / 2 - msg.yaw  # NED yaw -> ENU yaw (same)

                        # Angular velocity: NED -> ENU
                        self.angular_velocity["rollspeed"] = (
                            msg.rollspeed
                        )  # NED pitch -> ENU roll (negated)
                        self.angular_velocity[
                            "pitchspeed"
                        ] = -msg.pitchspeed  # NED roll -> ENU pitch (negated)
                        self.angular_velocity[
                            "yawspeed"
                        ] = -msg.yawspeed  # NED yaw -> ENU yaw (same)
                        self.last_attitude_time = time.time()

                    elif msg_type == "GPS_RAW_INT":
                        # GPS status data - match MAVROS scaling exactly
                        self.gps_status["fix_type"] = msg.fix_type
                        self.gps_status["satellites"] = msg.satellites_visible
                        # EPH and EPV: cm to m (match MAVROS)
                        self.gps_status["hdop"] = (
                            msg.eph / 100.0 if msg.eph != 65535 else 0.0
                        )  # cm to m
                        self.gps_status["vdop"] = (
                            msg.epv / 100.0 if msg.epv != 65535 else 0.0
                        )  # cm to m

                    elif msg_type == "LOCAL_POSITION_NED_COV":
                        # ArduPilot provides covariance data for local position - match MAVROS exactly
                        # Note: This message provides position in NED frame, but we convert to ENU
                        # Position: NED -> ENU (x_ned -> y_enu, y_ned -> x_enu, z_ned -> -z_enu)
                        self.local_position["x"] = msg.y  # NED y -> ENU x
                        self.local_position["y"] = msg.x  # NED x -> ENU y
                        self.local_position["z"] = -msg.z  # NED z -> ENU z (negated)

                        # Velocity: NED -> ENU (vx_ned -> vy_enu, vy_ned -> vx_enu, vz_ned -> -vz_enu)
                        self.velocity["vx"] = msg.vy  # NED vy -> ENU vx
                        self.velocity["vy"] = msg.vx  # NED vx -> ENU vy
                        self.velocity["vz"] = -msg.vz  # NED vz -> ENU vz (negated)

                        # Extract pose covariance (position and orientation) - match MAVROS mapping
                        self.pose_covariance = list(msg.covariance)
                        self.last_local_position_time = time.time()

                    elif msg_type == "RAW_IMU":
                        # Raw IMU data - convert from aircraft frame (FRD) to base_link frame (FLU)
                        # This matches MAVROS ftf::transform_frame_aircraft_baselink()
                        # +π rotation around X-axis: FRD -> FLU

                        # Scaling constants (same as MAVROS)
                        MILLIRS_TO_RADSEC = 1.0e-3  # millirad/s to rad/s
                        MILLIG_TO_MS2 = 9.80665 / 1000.0  # milliG to m/s² (ArduPilot)
                        MILLIT_TO_TESLA = 1000.0  # milliTesla to Tesla

                        # Linear acceleration: FRD -> FLU (same as MAVROS)
                        self.imu_data["linear_acceleration"] = [
                            msg.xacc * MILLIG_TO_MS2,  # FRD x -> FLU x (same)
                            -msg.yacc * MILLIG_TO_MS2,  # FRD y -> FLU y (negated)
                            -msg.zacc * MILLIG_TO_MS2,  # FRD z -> FLU z (negated)
                        ]  # milliG to m/s²

                        # Angular velocity: FRD -> FLU (same as MAVROS)
                        self.imu_data["angular_velocity"] = [
                            msg.xgyro * MILLIRS_TO_RADSEC,  # FRD x -> FLU x (same)
                            -msg.ygyro * MILLIRS_TO_RADSEC,  # FRD y -> FLU y (negated)
                            -msg.zgyro * MILLIRS_TO_RADSEC,  # FRD z -> FLU z (negated)
                        ]  # millirad/s to rad/s

                        # Magnetic field: FRD -> FLU (same as MAVROS)
                        self.magnetic_field["x"] = (
                            msg.xmag / MILLIT_TO_TESLA
                        )  # FRD x -> FLU x (same)
                        self.magnetic_field["y"] = (
                            -msg.ymag / MILLIT_TO_TESLA
                        )  # FRD y -> FLU y (negated)
                        self.magnetic_field["z"] = (
                            -msg.zmag / MILLIT_TO_TESLA
                        )  # FRD z -> FLU z (negated)

                        # Debug logging

                    elif msg_type == "SCALED_PRESSURE":
                        # Pressure and temperature data - match MAVROS scaling exactly
                        # Pressure: hPa to Pa (match MAVROS)
                        self.pressure_data["pressure"] = msg.press_abs * 100.0  # hPa to Pa
                        self.pressure_data["altitude"] = msg.press_diff  # differential pressure
                        # Temperature: cdegC to degC (match MAVROS)
                        self.pressure_data["temperature"] = msg.temperature / 100.0  # cdegC to degC

                    elif msg_type == "SYS_STATUS":
                        # Battery and system status - match MAVROS scaling exactly
                        # Voltage: mV to V (match MAVROS)
                        self.battery_data["voltage"] = msg.voltage_battery / 1000.0  # mV to V
                        # Current: 10 mA to A (match MAVROS)
                        self.battery_data["current"] = (
                            msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                        )  # 10 mA to A
                        # Remaining: % to ratio (match MAVROS)
                        self.battery_data["remaining"] = (
                            msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else 0.0
                        )  # % to ratio

                    elif msg_type == "RC_CHANNELS":
                        # RC channel data
                        self.rc_data["channels"] = [
                            msg.chan1_raw,
                            msg.chan2_raw,
                            msg.chan3_raw,
                            msg.chan4_raw,
                            msg.chan5_raw,
                            msg.chan6_raw,
                            msg.chan7_raw,
                            msg.chan8_raw,
                            msg.chan9_raw,
                            msg.chan10_raw,
                            msg.chan11_raw,
                            msg.chan12_raw,
                            msg.chan13_raw,
                            msg.chan14_raw,
                            msg.chan15_raw,
                            msg.chan16_raw,
                            msg.chan17_raw,
                            msg.chan18_raw,
                        ]

                    elif msg_type == "HEARTBEAT":
                        # Vehicle state from heartbeat
                        self.vehicle_state["armed"] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        self.vehicle_state["mode"] = (
                            mavutil.mavlink.enums["MAV_MODE"][msg.base_mode].name
                            if msg.base_mode in mavutil.mavlink.enums["MAV_MODE"]
                            else "UNKNOWN"
                        )

                    elif msg_type == "EXTENDED_SYS_STATE":
                        # Extended state information
                        self.extended_state["vtol_state"] = msg.vtol_state
                        self.extended_state["landed_state"] = msg.landed_state

                    elif msg_type == "STATUSTEXT":
                        # Status text messages
                        print(f"ArduPilot Status: {msg.text}")

                    elif msg_type == "MISSION_COUNT":
                        # Request all waypoints when mission count is received
                        self.request_waypoints(msg.count)

                    elif msg_type == "MISSION_ITEM_INT":
                        # Store waypoint - match MAVROS scaling exactly
                        if len(self.waypoints) <= msg.seq:
                            self.waypoints.extend([None] * (msg.seq + 1 - len(self.waypoints)))

                        # Scaling factors based on frame (match MAVROS encode_factor)
                        if msg.frame in [0, 3, 4, 5, 6]:  # GLOBAL frames
                            encode_factor = 10000000.0  # lat/lon scaling
                        elif msg.frame in [1, 2, 7, 8, 9, 10, 11, 12, 13, 14]:  # LOCAL frames
                            encode_factor = 10000.0  # local position scaling
                        else:
                            encode_factor = 1.0  # mission frame

                        self.waypoints[msg.seq] = {
                            "seq": msg.seq,
                            "frame": msg.frame,
                            "command": msg.command,
                            "lat": msg.x / encode_factor,  # Apply scaling factor
                            "lon": msg.y / encode_factor,  # Apply scaling factor
                            "alt": msg.z,
                            "param1": msg.param1,
                            "param2": msg.param2,
                            "param3": msg.param3,
                            "param4": msg.param4,
                        }

                    elif msg_type == "VFR_HUD":
                        # VFR_HUD message - contains airspeed, groundspeed, heading, etc.
                        if self.vfr_hud_data is not None:
                            try:
                                self.vfr_hud_data["airspeed"] = msg.airspeed
                                self.vfr_hud_data["groundspeed"] = msg.groundspeed
                                self.vfr_hud_data["heading"] = msg.heading
                                self.vfr_hud_data["throttle"] = msg.throttle
                                self.vfr_hud_data["alt"] = msg.alt
                                self.vfr_hud_data["climb"] = msg.climb
                            except (KeyError, TypeError) as e:
                                # Re-initialize vfr_hud_data if it's corrupted
                                logger.warning(f"vfr_hud_data corrupted, re-initializing: {e}")
                                self.vfr_hud_data = {
                                    "airspeed": 0.0,
                                    "groundspeed": 0.0,
                                    "heading": 0,
                                    "throttle": 0,
                                    "alt": 0.0,
                                    "climb": 0.0,
                                }
                                # Try again
                                self.vfr_hud_data["airspeed"] = msg.airspeed
                                self.vfr_hud_data["groundspeed"] = msg.groundspeed
                                self.vfr_hud_data["heading"] = msg.heading
                                self.vfr_hud_data["throttle"] = msg.throttle
                                self.vfr_hud_data["alt"] = msg.alt
                                self.vfr_hud_data["climb"] = msg.climb
                        else:
                            logger.warning("vfr_hud_data not initialized, skipping VFR_HUD message")

                    elif msg_type == "SERVO_OUTPUT_RAW":
                        # Servo output data - not needed for sensor publishing
                        pass

                    elif msg_type == "COMMAND_ACK":
                        # Command acknowledgment - not needed for sensor publishing
                        pass

                    # Ignore other message types silently
                    else:
                        # Only log unknown message types that aren't common noise
                        pass

                except Exception as e:
                    # Handle individual message processing errors
                    logger.warning(
                        f"Error processing message {msg_type if 'msg_type' in locals() else 'unknown'}: {e}"
                    )
                    continue

        except Exception as e:
            logger.error(f"Error updating telemetry: {e}")
            # If it's a file descriptor error, mark connection as invalid
            if "Bad file descriptor" in str(e) or "read failed" in str(e):
                self.is_connected = False
                logger.warning("Connection marked as invalid due to read error")

    def is_connection_valid(self) -> bool:
        """Check if the connection is still valid."""
        if not self.master or not self.is_connected:
            return False

        try:
            # Try to get file descriptor status
            if hasattr(self.master, "fd") and self.master.fd:
                import os

                try:
                    os.fstat(self.master.fd)
                    return True
                except OSError:
                    return False
            return True
        except:
            return False

    def reconnect(self) -> bool:
        """Attempt to reconnect to ArduPilot."""
        try:
            print("Attempting to reconnect to ArduPilot...")

            # Close existing connection
            if self.master:
                try:
                    self.master.close()
                except:
                    pass
                self.master = None

            self.is_connected = False

            # Wait a bit before reconnecting
            time.sleep(2)

            # Attempt to reconnect
            return self.connect()

        except Exception as e:
            logger.error(f"Reconnection failed: {e}")
            return False

    def request_waypoints(self, count: int):
        """Request all waypoints from ArduPilot."""
        if not self.master:
            return

        try:
            for i in range(count):
                self.master.mav.mission_request_int_send(
                    self.master.target_system, self.master.target_component, i
                )
        except Exception as e:
            logger.error(f"Error requesting waypoints: {e}")

    def request_mission_list(self):
        """Request mission list from ArduPilot."""
        if not self.master:
            return

        try:
            self.master.mav.mission_request_list_send(
                self.master.target_system, self.master.target_component
            )
        except Exception as e:
            logger.error(f"Error requesting mission list: {e}")

    def get_global_position_data(self) -> Dict[str, Any]:
        """Get global position data for NavSatFix message."""
        return {
            "latitude": self.global_position["lat"],
            "longitude": self.global_position["lon"],
            "altitude": self.global_position["alt"],
            "status": self.gps_status["fix_type"],
            "satellites": self.gps_status["satellites"],
            "position_covariance": [
                self.gps_status["hdop"] ** 2,
                0,
                0,
                0,
                self.gps_status["hdop"] ** 2,
                0,
                0,
                0,
                self.gps_status["vdop"] ** 2,
            ],
            "timestamp": self.last_global_position_time,
        }

    def get_odometry_data(self) -> Dict[str, Any]:
        """Get odometry data for Odometry message."""
        # Convert Euler angles to quaternion (already in ENU frame)
        roll, pitch, yaw = (
            self.orientation["roll"],
            self.orientation["pitch"],
            self.orientation["yaw"],
        )

        # Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        return {
            "position": [
                self.local_position["x"],
                self.local_position["y"],
                self.local_position["z"],
            ],
            "orientation": [q_x, q_y, q_z, q_w],
            "linear_velocity": [self.velocity["vx"], self.velocity["vy"], self.velocity["vz"]],
            "angular_velocity": [
                self.angular_velocity["rollspeed"],
                self.angular_velocity["pitchspeed"],
                self.angular_velocity["yawspeed"],
            ],
            "pose_covariance": self.pose_covariance,
            "twist_covariance": self.twist_covariance,
            "timestamp": max(self.last_local_position_time, self.last_attitude_time),
        }

    def get_waypoints_data(self) -> List[Dict[str, Any]]:
        """Get waypoints data for Path message."""
        valid_waypoints = [wp for wp in self.waypoints if wp is not None]
        return valid_waypoints

    def disconnect(self):
        """Disconnect from ArduPilot."""
        if self.master:
            self.master.close()
            self.master = None
        self.is_connected = False
        print("ArduPilot disconnected")


class ArduPilotModule(Module):
    """
    DIMOS module for ArduPilot that publishes sensor data via LCM.

    Publishes:
        - /mavros/global_position/global: GPS position data
        - /mavros/local_position/odom: Local position and velocity
        - /mavros/mission/waypoints: Mission waypoints as path
    """

    # Define LCM outputs (ArduPilot → LCM)
    global_position: Out[NavSatFix] = None
    local_odom: Out[Odometry] = None
    mission_waypoints: Out[Path] = None
    gps_satellites: Out[UInt16] = None

    # Sensor data outputs
    imu_data: Out[Imu] = None
    imu_data_raw: Out[Imu] = None
    magnetic_field: Out[MagneticField] = None
    static_pressure: Out[FluidPressure] = None
    differential_pressure: Out[FluidPressure] = None
    temperature_imu: Out[Temperature] = None
    temperature_baro: Out[Temperature] = None
    battery: Out[BatteryState] = None

    # Position outputs (standard ROS messages)
    local_pose: Out[PoseStamped] = None
    local_pose_cov: Out[PoseStamped] = None
    local_velocity: Out[TwistStamped] = None
    local_velocity_body: Out[TwistStamped] = None
    local_velocity_body_cov: Out[TwistStamped] = None
    local_accel: Out[AccelStamped] = None

    # Status outputs (standard ROS messages)
    diagnostics: Out[DiagnosticArray] = None

    # Setpoint inputs
    setpoint_position_local: In[PoseStamped] = None
    setpoint_position_global: In[PoseStamped] = None
    setpoint_velocity_cmd_vel: In[TwistStamped] = None
    setpoint_accel: In[AccelStamped] = None
    setpoint_attitude_thrust: In[Float64] = None
    setpoint_attitude_cmd_vel: In[TwistStamped] = None

    def __init__(
        self,
        connection_string: str = "/dev/ttyACM0",
        baudrate: int = 921600,
        publish_rate: float = 10.0,
        heartbeat_rate: float = 1.0,
        frame_id: str = "base_link",
        local_frame_id: str = "odom",
        global_frame_id: str = "map",
        **kwargs,
    ):
        """
        Initialize ArduPilot Module.

        Args:
            connection_string: MAVLink connection string
            baudrate: Serial baudrate for serial connections
            publish_rate: Rate to publish messages (Hz)
            heartbeat_rate: Rate to send heartbeats (Hz)
            frame_id: TF frame ID for local position messages
            global_frame_id: TF frame ID for global position messages
        """
        super().__init__(**kwargs)

        self.connection_string = connection_string
        self.baudrate = baudrate
        self.publish_rate = publish_rate
        self.heartbeat_rate = heartbeat_rate
        self.frame_id = frame_id
        self.local_frame_id = local_frame_id
        self.global_frame_id = global_frame_id
        self.tf_broadcaster = TF()

        # Internal state
        self.ardupilot = None
        self._running = False
        self._publish_subscription = None
        self._heartbeat_subscription = None
        self._sequence = 0

        print(f"ArduPilotModule initialized for {connection_string}")

        # Add MAVROS-specific ports if available
        if MAVROS_MSGS_AVAILABLE:
            self._add_mavros_ports()

    def _add_mavros_ports(self):
        """Add MAVROS-specific input/output ports if MAVROS messages are available."""
        # Vehicle state outputs (MAVROS-specific)
        self.state: Out[State] = None
        self.extended_state: Out[ExtendedState] = None
        self.rc_in: Out[RCIn] = None
        self.rc_out: Out[RCOut] = None
        self.altitude: Out[Altitude] = None
        self.status_text: Out[StatusText] = None
        self.wind_estimation: Out[WindEstimation] = None

        # MAVROS input ports (LCM → ArduPilot)
        self.actuator_control: In[ActuatorControl] = None
        self.manual_control_send: In[ManualControl] = None
        self.rc_override: In[OverrideRCIn] = None

    @rpc
    def start(self):
        """Start the ArduPilot module and begin publishing data."""
        if self._running:
            logger.warning("ArduPilot module already running")
            return

        try:
            # Initialize ArduPilot interface
            self.ardupilot = ArduPilotInterface(
                connection_string=self.connection_string, baudrate=self.baudrate
            )
            # Connect to ArduPilot
            if not self.ardupilot.connect():
                print("Failed to connect to ArduPilot")
                return

            # Request initial mission list
            self.ardupilot.request_mission_list()

            # Start periodic publishing
            self._running = True

            # Publishing subscription
            publish_interval = 1.0 / self.publish_rate
            self._publish_subscription = interval(publish_interval).subscribe(
                lambda _: self._update_and_publish()
            )

            # Heartbeat subscription
            heartbeat_interval = 1.0 / self.heartbeat_rate
            self._heartbeat_subscription = interval(heartbeat_interval).subscribe(
                lambda _: self._send_heartbeat()
            )

            print(f"ArduPilot module started, publishing at {self.publish_rate} Hz")

        except Exception as e:
            logger.error(f"Error starting ArduPilot module: {e}")
            self._running = False

    @rpc
    def stop(self):
        """Stop the ArduPilot module."""
        if not self._running:
            return

        self._running = False

        # Stop subscriptions
        if self._publish_subscription:
            self._publish_subscription.dispose()
            self._publish_subscription = None

        if self._heartbeat_subscription:
            self._heartbeat_subscription.dispose()
            self._heartbeat_subscription = None

        # Disconnect from ArduPilot
        if self.ardupilot:
            self.ardupilot.disconnect()
            self.ardupilot = None

        print("ArduPilot module stopped")

    def _setup_command_subscribers(self):
        """Set up subscribers for command inputs (MAVROS-specific)."""
        if not MAVROS_MSGS_AVAILABLE:
            return

        try:
            # Subscribe to MAVROS command inputs
            if hasattr(self, "actuator_control") and self.actuator_control:
                self.actuator_control.subscribe(self._handle_actuator_control)
            if hasattr(self, "manual_control_send") and self.manual_control_send:
                self.manual_control_send.subscribe(self._handle_manual_control)
            if hasattr(self, "rc_override") and self.rc_override:
                self.rc_override.subscribe(self._handle_rc_override)

            # Standard setpoint inputs (always available)
            if self.setpoint_position_local:
                self.setpoint_position_local.subscribe(self._handle_setpoint_position_local)
            if self.setpoint_velocity_cmd_vel:
                self.setpoint_velocity_cmd_vel.subscribe(self._handle_setpoint_velocity)

            print("Command subscribers configured")
        except Exception as e:
            print(f"Error setting up command subscribers: {e}")

    def _handle_actuator_control(self, msg: ActuatorControl):
        """Forward actuator control commands to ArduPilot."""
        if not self.ardupilot or not self.ardupilot.master:
            return
        try:
            self.ardupilot.master.mav.set_actuator_control_target_send(
                0,  # time_boot_ms
                msg.group,
                self.ardupilot.master.target_system,
                self.ardupilot.master.target_component,
                msg.controls,
            )
        except Exception as e:
            print(f"Error sending actuator control: {e}")

    def _handle_manual_control(self, msg: ManualControl):
        """Forward manual control commands to ArduPilot."""
        if not self.ardupilot or not self.ardupilot.master:
            return
        try:
            self.ardupilot.master.mav.manual_control_send(
                self.ardupilot.master.target_system,
                int(msg.x * 1000),  # -1000 to 1000
                int(msg.y * 1000),
                int(msg.z * 1000),
                int(msg.r * 1000),
                msg.buttons,
            )
        except Exception as e:
            print(f"Error sending manual control: {e}")

    def _handle_rc_override(self, msg: OverrideRCIn):
        """Forward RC override commands to ArduPilot."""
        if not self.ardupilot or not self.ardupilot.master:
            return
        try:
            self.ardupilot.master.mav.rc_channels_override_send(
                self.ardupilot.master.target_system,
                self.ardupilot.master.target_component,
                msg.channels[0] if len(msg.channels) > 0 else 0,
                msg.channels[1] if len(msg.channels) > 1 else 0,
                msg.channels[2] if len(msg.channels) > 2 else 0,
                msg.channels[3] if len(msg.channels) > 3 else 0,
                msg.channels[4] if len(msg.channels) > 4 else 0,
                msg.channels[5] if len(msg.channels) > 5 else 0,
                msg.channels[6] if len(msg.channels) > 6 else 0,
                msg.channels[7] if len(msg.channels) > 7 else 0,
            )
        except Exception as e:
            print(f"Error sending RC override: {e}")

    def _handle_setpoint_position_local(self, msg: PoseStamped):
        """Forward local position setpoint to ArduPilot."""
        if not self.ardupilot or not self.ardupilot.master:
            return
        try:
            self.ardupilot.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.ardupilot.master.target_system,
                self.ardupilot.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # type_mask (ignore velocity and acceleration)
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                0,
                0,
                0,  # velocity
                0,
                0,
                0,  # acceleration
                0,
                0,  # yaw, yaw_rate
            )
        except Exception as e:
            print(f"Error sending position setpoint: {e}")

    def _handle_setpoint_velocity(self, msg: TwistStamped):
        """Forward velocity setpoint to ArduPilot."""
        if not self.ardupilot or not self.ardupilot.master:
            return
        try:
            self.ardupilot.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.ardupilot.master.target_system,
                self.ardupilot.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # type_mask (ignore position and acceleration)
                0,
                0,
                0,  # position
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
                0,
                0,
                0,  # acceleration
                0,
                msg.twist.angular.z,  # yaw, yaw_rate
            )
        except Exception as e:
            print(f"Error sending velocity setpoint: {e}")

    def _send_heartbeat(self):
        """Send heartbeat to ArduPilot."""
        if not self._running or not self.ardupilot:
            return

        try:
            self.ardupilot.send_heartbeat()
        except Exception as e:
            print("Error sending heartbeat: {e}")

    def _update_and_publish(self):
        """Update telemetry and publish all data."""
        if not self._running or not self.ardupilot:
            return

        try:
            # Update telemetry data
            self.ardupilot.update_telemetry()

            # Get timestamp
            timestamp_ns = time.time_ns()
            timestamp = Time(sec=timestamp_ns // 1_000_000_000, nsec=timestamp_ns % 1_000_000_000)

            # Publish global position
            self._publish_global_position(timestamp)

            # Publish local odometry
            self._publish_local_odometry(timestamp)

            # Publish sensor data
            self._publish_imu_data(timestamp)
            self._publish_magnetic_field(timestamp)
            self._publish_pressure_data(timestamp)
            self._publish_temperature_data(timestamp)
            self._publish_battery_data(timestamp)

            # Publish position data
            self._publish_local_pose(timestamp)
            self._publish_local_velocity(timestamp)
            self._publish_local_accel(timestamp)

            # Publish GPS satellites
            self._publish_gps_satellites(timestamp)

            # Publish diagnostics
            self._publish_diagnostics(timestamp)

            # Publish waypoints (less frequently)
            if self._sequence % 50 == 0:  # Every 5 seconds at 10Hz
                self._publish_waypoints(timestamp)

            self._sequence += 1

        except Exception as e:
            logger.error(f"Error in update and publish: {e}")

    def _publish_global_position(self, timestamp: Time):
        """Publish global position as NavSatFix message."""
        try:
            data = self.ardupilot.get_global_position_data()

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.global_frame_id)

            # Create NavSatStatus - match MAVROS exactly
            status = NavSatStatus(
                status=data["status"],
                service=1,  # SERVICE_GPS (match MAVROS)
            )

            # Create NavSatFix message - match MAVROS exactly
            msg = NavSatFix(
                header=header,
                status=status,
                latitude=data["latitude"],
                longitude=data["longitude"],
                altitude=data["altitude"],
                position_covariance=data["position_covariance"],
                position_covariance_type=2,  # COVARIANCE_TYPE_DIAGONAL_KNOWN (match MAVROS)
            )

            self.global_position.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing global position: {e}")

    def _publish_local_odometry(self, timestamp: Time):
        """Publish local odometry as Odometry message."""
        try:
            data = self.ardupilot.get_odometry_data()
            self._publish_tf(timestamp, data)
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.local_frame_id)

            # Create pose with covariance
            pose = Pose(
                position=Point(x=data["position"][0], y=data["position"][1], z=data["position"][2]),
                orientation=Quaternion(
                    x=data["orientation"][0],
                    y=data["orientation"][1],
                    z=data["orientation"][2],
                    w=data["orientation"][3],
                ),
            )

            # Use actual covariance from ArduPilot
            pose_with_cov = PoseWithCovariance(pose=pose, covariance=data["pose_covariance"])

            # Create twist with covariance
            twist = Twist(
                linear=Vector3(
                    x=data["linear_velocity"][0],
                    y=data["linear_velocity"][1],
                    z=data["linear_velocity"][2],
                ),
                angular=Vector3(
                    x=data["angular_velocity"][0],
                    y=data["angular_velocity"][1],
                    z=data["angular_velocity"][2],
                ),
            )

            # Use actual twist covariance from ArduPilot
            twist_with_cov = TwistWithCovariance(twist=twist, covariance=data["twist_covariance"])

            # Create Odometry message
            msg = Odometry(
                header=header,
                child_frame_id=self.frame_id,
                pose=pose_with_cov,
                twist=twist_with_cov,
            )
            self.local_odom.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing local odometry: {e}")

    def _publish_tf(self, timestamp: Time, data: dict):
        """Publish transform from local frame to base_link frame."""
        try:
            map_to_baselink = Transform(
                translation=Vector3(data["position"][0], data["position"][1], data["position"][2]),
                rotation=Quaternion(
                    data["orientation"][0],
                    data["orientation"][1],
                    data["orientation"][2],
                    data["orientation"][3],
                ),  # Identity rotation
                frame_id=self.global_frame_id,
                child_frame_id=self.frame_id,
                ts=timestamp.sec + timestamp.nsec * 1e-9,
            )

            # Broadcast the transform
            self.tf_broadcaster.publish(map_to_baselink)

        except Exception as e:
            # use traceback to print the error
            import traceback

            logger.error(f"Error publishing TF: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")

    def _publish_waypoints(self, timestamp: Time):
        """Publish waypoints as Path message."""
        try:
            waypoints = self.ardupilot.get_waypoints_data()

            if not waypoints:
                return

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.global_frame_id)

            # Create path with poses - match MAVROS coordinate handling
            poses = []
            for wp in waypoints:
                if wp["command"] == 16:  # NAV_WAYPOINT
                    # Create pose for waypoint
                    pose_header = Header(
                        seq=wp["seq"], stamp=timestamp, frame_id=self.global_frame_id
                    )

                    # For global waypoints, use lat/lon as coordinates (match MAVROS)
                    # For local waypoints, these would be local coordinates
                    pose = Pose(
                        position=Point(x=wp["lat"], y=wp["lon"], z=wp["alt"]),
                        orientation=Quaternion(x=0, y=0, z=0, w=1),  # No orientation for waypoints
                    )

                    pose_stamped = PoseStamped(header=pose_header, pose=pose)
                    poses.append(pose_stamped)

            # Create Path message
            msg = Path(header=header, poses_length=len(poses), poses=poses)

            self.mission_waypoints.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing waypoints: {e}")

    def _publish_imu_data(self, timestamp: Time):
        """Publish IMU data."""
        try:
            # Check if IMU outputs are available
            if not self.imu_data:
                logger.warning("imu_data output not available, skipping")
                return
            if not self.imu_data_raw:
                logger.warning("imu_data_raw output not available, skipping")
                return

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get IMU data from ArduPilot interface
            imu_data = self.ardupilot.imu_data
            orientation = self.ardupilot.orientation
            angular_velocity = self.ardupilot.angular_velocity

            # Debug logging
            logger.debug(f"IMU Data - Linear Accel: {imu_data['linear_acceleration']}")
            logger.debug(f"IMU Data - Angular Vel: {imu_data['angular_velocity']}")
            logger.debug(
                f"Orientation - Roll: {orientation['roll']:.3f}, Pitch: {orientation['pitch']:.3f}, Yaw: {orientation['yaw']:.3f}"
            )

            # Convert Euler angles to quaternion (already in ENU frame)
            roll, pitch, yaw = orientation["roll"], orientation["pitch"], orientation["yaw"]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            q_w = cr * cp * cy + sr * sp * sy
            q_x = sr * cp * cy - cr * sp * sy
            q_y = cr * sp * cy + sr * cp * sy
            q_z = cr * cp * sy - sr * sp * cy

            logger.debug(f"Quaternion - x: {q_x:.3f}, y: {q_y:.3f}, z: {q_z:.3f}, w: {q_w:.3f}")

            # Create IMU message (already in ENU frame)
            msg = Imu(
                header=header,
                orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w),
                orientation_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                angular_velocity=Vector3(
                    x=imu_data["angular_velocity"][0],
                    y=imu_data["angular_velocity"][1],
                    z=imu_data["angular_velocity"][2],
                ),
                angular_velocity_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                linear_acceleration=Vector3(
                    x=imu_data["linear_acceleration"][0],
                    y=imu_data["linear_acceleration"][1],
                    z=imu_data["linear_acceleration"][2],
                ),
                linear_acceleration_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            )

            self.imu_data.publish(msg)

            # Also publish raw IMU data (already in ENU frame)
            raw_msg = Imu(
                header=header,
                orientation=Quaternion(x=0, y=0, z=0, w=1),  # No orientation for raw data
                orientation_covariance=[-1, 0, 0, 0, 0, 0, 0, 0, 0],  # -1 means no orientation
                angular_velocity=Vector3(
                    x=imu_data["angular_velocity"][0],
                    y=imu_data["angular_velocity"][1],
                    z=imu_data["angular_velocity"][2],
                ),
                angular_velocity_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                linear_acceleration=Vector3(
                    x=imu_data["linear_acceleration"][0],
                    y=imu_data["linear_acceleration"][1],
                    z=imu_data["linear_acceleration"][2],
                ),
                linear_acceleration_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            )

            self.imu_data_raw.publish(raw_msg)

        except Exception as e:
            logger.error(f"Error publishing IMU data: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def _publish_magnetic_field(self, timestamp: Time):
        """Publish magnetic field data."""
        try:
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get magnetic field data from ArduPilot interface
            mag_data = self.ardupilot.magnetic_field

            # Create MagneticField message
            msg = MagneticField(
                header=header,
                magnetic_field=Vector3(x=mag_data["x"], y=mag_data["y"], z=mag_data["z"]),
                magnetic_field_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
            )

            self.magnetic_field.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing magnetic field: {e}")

    def _publish_pressure_data(self, timestamp: Time):
        """Publish pressure data."""
        try:
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get pressure data from ArduPilot interface
            pressure_data = self.ardupilot.pressure_data

            # Create static pressure message
            static_msg = FluidPressure(
                header=header,
                fluid_pressure=pressure_data["pressure"],
                variance=0.01,  # Pressure variance
            )

            self.static_pressure.publish(static_msg)

            # Create differential pressure message
            diff_msg = FluidPressure(
                header=header,
                fluid_pressure=pressure_data["altitude"],  # Using altitude as differential pressure
                variance=0.01,
            )

            self.differential_pressure.publish(diff_msg)

        except Exception as e:
            logger.error(f"Error publishing pressure data: {e}")

    def _publish_temperature_data(self, timestamp: Time):
        """Publish temperature data."""
        try:
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get temperature data from ArduPilot interface
            pressure_data = self.ardupilot.pressure_data

            # Create IMU temperature message (using baro temperature as approximation)
            imu_temp_msg = Temperature(
                header=header,
                temperature=pressure_data["temperature"],
                variance=0.1,  # Temperature variance
            )

            self.temperature_imu.publish(imu_temp_msg)

            # Create baro temperature message
            baro_temp_msg = Temperature(
                header=header, temperature=pressure_data["temperature"], variance=0.1
            )

            self.temperature_baro.publish(baro_temp_msg)

        except Exception as e:
            logger.error(f"Error publishing temperature data: {e}")

    def _publish_battery_data(self, timestamp: Time):
        """Publish battery data."""
        try:
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get battery data from ArduPilot interface
            battery_data = self.ardupilot.battery_data

            # Create BatteryState message - match MAVROS exactly
            msg = BatteryState(
                header=header,
                voltage=battery_data["voltage"],
                current=-battery_data["current"],  # MAVROS negates current
                charge=float("nan"),  # MAVROS sets to NAN
                capacity=float("nan"),  # MAVROS sets to NAN
                design_capacity=float("nan"),  # MAVROS sets to NAN
                percentage=battery_data["remaining"] * 100.0,  # Convert ratio to percentage
                power_supply_status=1,  # POWER_SUPPLY_STATUS_DISCHARGING (match MAVROS)
                power_supply_health=0,  # POWER_SUPPLY_HEALTH_UNKNOWN (match MAVROS)
                power_supply_technology=0,  # POWER_SUPPLY_TECHNOLOGY_UNKNOWN (match MAVROS)
                present=True,
                cell_voltage=[],  # MAVROS clears this
                location="",
                serial_number="",
            )

            self.battery.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing battery data: {e}")

    def _publish_local_pose(self, timestamp: Time):
        """Publish local pose data."""
        try:
            # Check if pose outputs are available
            if not self.local_pose:
                logger.warning("local_pose output not available, skipping")
                return
            if not self.local_pose_cov:
                logger.warning("local_pose_cov output not available, skipping")
                return

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.local_frame_id)

            # Get position and orientation data (already in ENU frame)
            local_pos = self.ardupilot.local_position
            orientation = self.ardupilot.orientation

            # Convert Euler angles to quaternion (already in ENU frame)
            roll, pitch, yaw = orientation["roll"], orientation["pitch"], orientation["yaw"]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            q_w = cr * cp * cy + sr * sp * sy
            q_x = sr * cp * cy - cr * sp * sy
            q_y = cr * sp * cy + sr * cp * sy
            q_z = cr * cp * sy - sr * sp * cy

            # Create pose message (already in ENU frame)
            pose = Pose(
                position=Point(x=local_pos["x"], y=local_pos["y"], z=local_pos["z"]),
                orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w),
            )

            msg = PoseStamped(header=header, pose=pose)
            self.local_pose.publish(msg)

            # Also publish pose with covariance
            self.local_pose_cov.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing local pose: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def _publish_local_velocity(self, timestamp: Time):
        """Publish local velocity data."""
        try:
            # Check if velocity outputs are available
            if not self.local_velocity:
                logger.warning("local_velocity output not available, skipping")
                return
            if not self.local_velocity_body:
                logger.warning("local_velocity_body output not available, skipping")
                return
            if not self.local_velocity_body_cov:
                logger.warning("local_velocity_body_cov output not available, skipping")
                return

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.local_frame_id)

            # Get velocity data
            velocity = self.ardupilot.velocity
            angular_velocity = self.ardupilot.angular_velocity

            # Create twist message (already in ENU frame)
            twist = Twist(
                linear=Vector3(x=velocity["vx"], y=velocity["vy"], z=velocity["vz"]),
                angular=Vector3(
                    x=angular_velocity["rollspeed"],
                    y=angular_velocity["pitchspeed"],
                    z=angular_velocity["yawspeed"],
                ),
            )

            msg = TwistStamped(header=header, twist=twist)

            # Publish with error handling for each output
            try:
                self.local_velocity.publish(msg)
            except Exception as e:
                if "Resource temporarily unavailable" in str(e):
                    logger.debug(f"Temporary resource issue with local_velocity: {e}")
                else:
                    logger.error(f"Error publishing local_velocity: {e}")

            try:
                self.local_velocity_body.publish(msg)
            except Exception as e:
                if "Resource temporarily unavailable" in str(e):
                    logger.debug(f"Temporary resource issue with local_velocity_body: {e}")
                else:
                    logger.error(f"Error publishing local_velocity_body: {e}")

            try:
                self.local_velocity_body_cov.publish(msg)
            except Exception as e:
                if "Resource temporarily unavailable" in str(e):
                    logger.debug(f"Temporary resource issue with local_velocity_body_cov: {e}")
                else:
                    logger.error(f"Error publishing local_velocity_body_cov: {e}")

        except Exception as e:
            logger.error(f"Error publishing local velocity: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def _publish_local_accel(self, timestamp: Time):
        """Publish local acceleration data."""
        try:
            # Check if local_accel output is available
            if not self.local_accel:
                logger.warning("local_accel output not available, skipping")
                return

            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get acceleration data from IMU
            imu_data = self.ardupilot.imu_data
            angular_velocity = self.ardupilot.angular_velocity

            # Create acceleration message with both linear and angular acceleration
            accel = Accel(
                linear=Vector3(
                    x=imu_data["linear_acceleration"][0],
                    y=imu_data["linear_acceleration"][1],
                    z=imu_data["linear_acceleration"][2],
                ),
                angular=Vector3(
                    x=angular_velocity["rollspeed"],
                    y=angular_velocity["pitchspeed"],
                    z=angular_velocity["yawspeed"],
                ),
            )

            msg = AccelStamped(header=header, accel=accel)
            self.local_accel.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing local acceleration: {e}")
            import traceback

            logger.error(f"Traceback: {traceback.format_exc()}")

    def _publish_gps_satellites(self, timestamp: Time):
        """Publish GPS satellites count."""
        try:
            # Check if GPS satellites output is available
            if not self.gps_satellites:
                logger.warning("gps_satellites output not available, skipping")
                return

            # Get GPS status from ArduPilot interface
            gps_status = self.ardupilot.gps_status

            # Create UInt16 message with satellite count
            msg = UInt16(data=gps_status["satellites"])

            self.gps_satellites.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing GPS satellites: {e}")

    def _publish_diagnostics(self, timestamp: Time):
        """Publish diagnostics data."""
        try:
            # Create header
            header = Header(seq=self._sequence, stamp=timestamp, frame_id=self.frame_id)

            # Get vehicle state
            vehicle_state = self.ardupilot.vehicle_state
            gps_status = self.ardupilot.gps_status

            # Create diagnostic status
            from dimos_lcm.diagnostic_msgs import DiagnosticStatus, KeyValue

            # GPS diagnostic
            gps_status_msg = DiagnosticStatus(
                name="GPS",
                message=f"GPS Fix: {gps_status['fix_type']}, Satellites: {gps_status['satellites']}",
                hardware_id="ArduPilot GPS",
                level=0 if gps_status["fix_type"] >= 3 else 1,  # 0=OK, 1=WARN
                values=[
                    KeyValue(key="Fix Type", value=str(gps_status["fix_type"])),
                    KeyValue(key="Satellites", value=str(gps_status["satellites"])),
                    KeyValue(key="HDOP", value=f"{gps_status['hdop']:.2f}"),
                    KeyValue(key="VDOP", value=f"{gps_status['vdop']:.2f}"),
                ],
            )

            # Vehicle state diagnostic
            vehicle_status_msg = DiagnosticStatus(
                name="Vehicle State",
                message=f"Mode: {vehicle_state['mode']}, Armed: {vehicle_state['armed']}",
                hardware_id="ArduPilot Vehicle",
                level=0,  # OK
                values=[
                    KeyValue(key="Mode", value=vehicle_state["mode"]),
                    KeyValue(key="Armed", value=str(vehicle_state["armed"])),
                    KeyValue(key="Connected", value=str(vehicle_state["connected"])),
                    KeyValue(key="Guided", value=str(vehicle_state["guided"])),
                ],
            )

            # Create diagnostic array
            msg = DiagnosticArray(header=header, status=[gps_status_msg, vehicle_status_msg])

            self.diagnostics.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing diagnostics: {e}")

    @rpc
    def get_connection_status(self) -> Dict[str, Any]:
        """Get connection status information."""
        if self.ardupilot:
            return {
                "connected": self.ardupilot.is_connected,
                "connection_string": self.ardupilot.connection_string,
                "target_system": self.ardupilot.master.target_system
                if self.ardupilot.master
                else 0,
                "target_component": self.ardupilot.master.target_component
                if self.ardupilot.master
                else 0,
            }
        return {"connected": False}

    @rpc
    def request_mission_update(self):
        """Request updated mission from ArduPilot."""
        if self.ardupilot:
            self.ardupilot.request_mission_list()

    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()


def main():
    """
    Simple main function to run ArduPilot module for 60 seconds.
    Publishes all non-MAVROS messages to demonstrate functionality.
    """
    import time
    import sys

    # Default connection string - can be overridden via command line
    connection_string = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

    print(f"Starting ArduPilot module with connection: {connection_string}")
    print("Will run for 60 seconds and publish all non-MAVROS messages...")

    try:
        # Initialize DIMOS core
        from dimos import core
        from dimos.robot.foxglove_bridge import FoxgloveBridge

        dimos = core.start(1)
        dimos.deploy(FoxgloveBridge)
        print("✓ DIMOS core started successfully")

        # Deploy ArduPilot module
        module = dimos.deploy(
            ArduPilotModule,
            connection_string=connection_string,
            baudrate=921600,
            publish_rate=10.0,
            heartbeat_rate=1.0,
            frame_id="base_link",
            global_frame_id="map",
        )
        print("✓ ArduPilotModule deployed successfully")

        # Configure LCM transports for all non-MAVROS outputs
        print("Configuring LCM transports...")

        # Standard sensor and position outputs
        module.global_position.transport = core.LCMTransport(
            "/mavros/global_position/global/raw/fix", NavSatFix
        )
        module.local_odom.transport = core.LCMTransport("/mavros/local_position/odom", Odometry)
        module.mission_waypoints.transport = core.LCMTransport("/mavros/mission/waypoints", Path)
        module.gps_satellites.transport = core.LCMTransport(
            "/mavros/global_position/global/raw/satellites", UInt16
        )

        # Sensor data outputs
        module.imu_data.transport = core.LCMTransport("/mavros/imu/data", Imu)
        module.imu_data_raw.transport = core.LCMTransport("/mavros/imu/data_raw", Imu)
        module.magnetic_field.transport = core.LCMTransport("/mavros/imu/mag", MagneticField)
        module.static_pressure.transport = core.LCMTransport(
            "/mavros/imu/static_pressure", FluidPressure
        )
        module.differential_pressure.transport = core.LCMTransport(
            "/mavros/imu/diff_pressure", FluidPressure
        )
        module.temperature_imu.transport = core.LCMTransport(
            "/mavros/imu/temperature_imu", Temperature
        )
        module.temperature_baro.transport = core.LCMTransport(
            "/mavros/imu/temperature_baro", Temperature
        )
        module.battery.transport = core.LCMTransport("/mavros/battery", BatteryState)

        # Position outputs
        module.local_pose.transport = core.LCMTransport("/mavros/local_position/pose", PoseStamped)
        module.local_pose_cov.transport = core.LCMTransport(
            "/mavros/local_position/pose_cov", PoseStamped
        )
        module.local_velocity.transport = core.LCMTransport(
            "/mavros/local_position/velocity", TwistStamped
        )
        module.local_velocity_body.transport = core.LCMTransport(
            "/mavros/local_position/vel_body", TwistStamped
        )
        module.local_velocity_body_cov.transport = core.LCMTransport(
            "/mavros/local_position/vel_body_cov", TwistStamped
        )
        module.local_accel.transport = core.LCMTransport(
            "/mavros/local_position/accel", AccelStamped
        )

        # Status outputs
        module.diagnostics.transport = core.LCMTransport("/mavros/diagnostics", DiagnosticArray)

        print("✓ All LCM transports configured successfully")

        # Start the module
        print("Starting ArduPilot module...")
        module.start()

        # Run for 60 seconds
        print("Running for 60 seconds...")
        start_time = time.time()

        while time.time() - start_time < 600:
            time.sleep(1)
            elapsed = int(time.time() - start_time)
            remaining = 60 - elapsed
            print(f"Running... {elapsed}s elapsed, {remaining}s remaining")

            # Check connection status every 10 seconds
            if elapsed % 10 == 0:
                status = module.get_connection_status()
                print(f"Connection status: {status}")

        print("60 seconds completed, stopping module...")

    except KeyboardInterrupt:
        print("\nInterrupted by user, stopping...")
    except Exception as e:
        print(f"Error during execution: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Stop and cleanup
        if "module" in locals():
            module.stop()
            module.cleanup()
        if "dimos" in locals():
            dimos.stop()
        print("ArduPilot module stopped and cleaned up")


if __name__ == "__main__":
    main()
