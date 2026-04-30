"""Viser scene: range rings, spokes, and robot frame for the lidar viewer."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import viser

from . import config


@dataclass
class SceneHandles:
    robot: viser.FrameHandle
    rings: list[object] = field(default_factory=list)


def _circle_points(radius_m: float, n: int) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * math.pi, n + 1)
    x = radius_m * np.cos(angles)
    y = radius_m * np.sin(angles)
    return np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32)


def create_scene(server: viser.ViserServer) -> SceneHandles:
    server.scene.add_frame("/origin", axes_length=0.12, axes_radius=0.003, show_axes=False)

    rings = []
    for r in config.GRID_RING_RADII_M:
        pts = _circle_points(r, config.GRID_RING_SEGMENTS)
        rings.append(server.scene.add_spline_catmull_rom(
            f"/grid/ring_{r:.0f}m", positions=pts, line_width=1.2, color=(30, 80, 90), closed=True,
        ))

    max_r = config.GRID_RING_RADII_M[-1]
    for deg in range(0, 360, 30):
        rad = math.radians(deg)
        pts = np.array(
            [[0.0, 0.0, 0.0], [max_r * math.cos(rad), max_r * math.sin(rad), 0.0]],
            dtype=np.float32,
        )
        server.scene.add_spline_catmull_rom(
            f"/grid/spoke_{deg}", positions=pts, line_width=0.7, color=(25, 60, 70),
        )

    robot = server.scene.add_frame("/robot", axes_length=0.28, axes_radius=0.007, show_axes=True)

    server.scene.add_spline_catmull_rom(
        "/robot/ring",
        positions=_circle_points(config.ROBOT_RING_RADIUS_M, 48),
        line_width=2.0,
        color=(0, 220, 130),
        closed=True,
    )

    return SceneHandles(robot=robot, rings=rings)
