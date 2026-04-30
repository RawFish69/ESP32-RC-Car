"""LD06 lidar library for TA Bot.

Public API
----------
SerialReader   - background thread that reads the controller's JSON scan stream
ScanSnapshot   - frozen dataclass with the latest scan + IMU data
LidarViewer    - full viser-based 2D viewer (used standalone or as a base)
create_scene   - build the viser scene (rings, spokes, robot frame)
SceneHandles   - returned by create_scene
MadgwickFilter - 6-DoF IMU orientation filter
"""

__version__ = "0.1.0"

from .serial_reader import SerialReader, ScanSnapshot
from .viewer import LidarViewer, main
from .scene import create_scene, SceneHandles
from .imu_fusion import MadgwickFilter

__all__ = [
    "SerialReader", "ScanSnapshot",
    "LidarViewer", "main",
    "create_scene", "SceneHandles",
    "MadgwickFilter",
]
