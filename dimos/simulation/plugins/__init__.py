"""Simulation engine and robot registration plugins."""

from dimos.simulation.plugins.mujoco_xarm import register as register_mujoco_xarm


def register_all() -> None:
    register_mujoco_xarm()


__all__ = [
    "register_all",
]
