"""Shared helpers for visualizing robot motion intent in the lidar viewers."""

from __future__ import annotations

import math

import numpy as np


def motion_label(mode: str, direction: str, dodging: bool, wall_side: str) -> str:
    if direction == "x":
        return "stopped"

    base = {
        "w": "forward",
        "s": "reverse",
        "a": "arc_left",
        "d": "arc_right",
        "q": "spin_left",
        "e": "spin_right",
    }.get(direction, "moving")

    if dodging:
        return f"dodge_{base}"
    return base


def motion_color(dodging: bool) -> tuple[int, int, int]:
    return (255, 90, 90) if dodging else (40, 235, 180)


def draw_motion_indicator(
    scene,
    name_prefix: str,
    direction: str,
    dodging: bool,
) -> None:
    scene.remove_by_name(f"{name_prefix}/motion/shaft")
    scene.remove_by_name(f"{name_prefix}/motion/head_left")
    scene.remove_by_name(f"{name_prefix}/motion/head_right")
    scene.remove_by_name(f"{name_prefix}/motion/spin")

    if direction == "x":
        return

    color = motion_color(dodging)
    line_width = 3.5 if dodging else 2.6

    if direction in {"q", "e"}:
        clockwise = direction == "e"
        angles = np.linspace(
            math.radians(150 if clockwise else -30),
            math.radians(-110 if clockwise else 230),
            24,
        )
        radius = 0.28
        points = np.stack(
            [radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles)],
            axis=1,
        ).astype(np.float32)
        scene.add_spline_catmull_rom(
            f"{name_prefix}/motion/spin",
            positions=points,
            line_width=line_width,
            color=color,
        )
        return

    tip = {
        "w": np.array([0.42, 0.0, 0.0], dtype=np.float32),
        "s": np.array([-0.30, 0.0, 0.0], dtype=np.float32),
        "a": np.array([0.34, 0.20, 0.0], dtype=np.float32),
        "d": np.array([0.34, -0.20, 0.0], dtype=np.float32),
    }.get(direction)
    if tip is None:
        return

    origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    direction_vec = tip[:2] - origin[:2]
    direction_norm = np.linalg.norm(direction_vec)
    if direction_norm < 1.0e-6:
        return

    forward = direction_vec / direction_norm
    left = np.array([-forward[1], forward[0]], dtype=np.float32)
    head_back = 0.11
    head_span = 0.07

    head_left = np.array(
        [
            tip[0] - head_back * forward[0] + head_span * left[0],
            tip[1] - head_back * forward[1] + head_span * left[1],
            0.0,
        ],
        dtype=np.float32,
    )
    head_right = np.array(
        [
            tip[0] - head_back * forward[0] - head_span * left[0],
            tip[1] - head_back * forward[1] - head_span * left[1],
            0.0,
        ],
        dtype=np.float32,
    )

    scene.add_spline_catmull_rom(
        f"{name_prefix}/motion/shaft",
        positions=np.stack([origin, tip], axis=0),
        line_width=line_width,
        color=color,
    )
    scene.add_spline_catmull_rom(
        f"{name_prefix}/motion/head_left",
        positions=np.stack([tip, head_left], axis=0),
        line_width=line_width,
        color=color,
    )
    scene.add_spline_catmull_rom(
        f"{name_prefix}/motion/head_right",
        positions=np.stack([tip, head_right], axis=0),
        line_width=line_width,
        color=color,
    )
