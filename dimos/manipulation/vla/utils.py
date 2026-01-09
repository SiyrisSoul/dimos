import threading

import numpy as np
import os
import time
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image, JointCommand, JointState
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat
from dimos.msgs.sensor_msgs import JointCommand

def get_camera_image(camera_topic: str = "/camera/color", timeout: float = 5.0) -> np.ndarray:
    event = threading.Event()
    image_data: dict[str, np.ndarray] = {}

    def on_img(msg: Image) -> None:
        if event.is_set():
            return
        image_data["image"] = msg.to_rgb().to_opencv()
        os.makedirs("captures", exist_ok=True)
        filename = f"camera_color_{time.time()}.png"
        Image.from_numpy(image_data["image"], format=ImageFormat.RGB).save(
            os.path.join("captures", filename)
        )
        event.set()

    transport = LCMTransport(camera_topic, Image)
    transport.subscribe(on_img)

    if not event.wait(timeout=timeout):
        raise TimeoutError(f"No image received on {camera_topic} within {timeout} seconds.")

    return image_data["image"]

def get_joint_positions(joint_state_topic: str = "/xarm/joint_states", timeout: float = 5.0):
    event = threading.Event()
    joint_positions: dict[str, np.ndarray] = {}

    def on_joint_state(msg: JointState) -> None:
        if event.is_set():
            return
        joint_positions["joint_positions"] = msg.position
        event.set()

    transport = LCMTransport(joint_state_topic, JointState)
    transport.subscribe(on_joint_state)

    if not event.wait(timeout=timeout):
        raise TimeoutError(f"No joint states received on {joint_state_topic} within {timeout} seconds.")

    return joint_positions["joint_positions"]

def get_joint_velocities(joint_velocity_topic: str = "/xarm/joint_velocity", timeout: float = 5.0):
    event = threading.Event()
    joint_velocities: dict[str, np.ndarray] = {}

    def on_joint_velocities(msg: JointState) -> None:
        if event.is_set():
            return
        joint_velocities["joint_velocities"] = msg.velocity
        event.set()

    transport = LCMTransport(joint_velocity_topic, JointCommand)
    transport.subscribe(on_joint_velocities)

    if not event.wait(timeout=timeout):
        raise TimeoutError(f"No joint velocities received on {joint_velocity_topic} within {timeout} seconds.")

    return joint_velocities["joint_velocities"]
    