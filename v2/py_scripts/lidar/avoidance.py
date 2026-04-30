"""Shared scan-based avoidance highlighting for the TA Bot viewers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


MIN_VALID_DIST_M = 0.04
CONTACT_MIN_FORWARD_M = 0.0
CONTACT_HALF_WIDTH_M = 0.10
CONTACT_EMERGENCY_M = 0.085
FRONT_MIN_FORWARD_M = 0.035
FRONT_HALF_WIDTH_M = 0.13
SIDE_MIN_FORWARD_M = 0.0
SIDE_MIN_LATERAL_M = 0.08

MODE_EMERGENCY_FRONT_M = {
    "2": 0.06,
    "3": 0.11,
    "4": 0.28,
    "5": 0.32,
}

MODE_CAUTION_FRONT_M = {
    "3": 0.18,
    "4": 0.50,
    "5": 0.65,
}

COLOR_CONTACT = np.array([255, 40, 40], dtype=np.uint8)
COLOR_FRONT_CAUTION = np.array([255, 180, 40], dtype=np.uint8)
COLOR_FRONT_EMERGENCY = np.array([255, 80, 60], dtype=np.uint8)
@dataclass(frozen=True)
class AvoidanceAnalysis:
    contact_min_m: float | None
    front_min_m: float | None
    left_min_m: float | None
    right_min_m: float | None
    contact_mask: np.ndarray
    front_caution_mask: np.ndarray
    front_emergency_mask: np.ndarray
    left_threat_mask: np.ndarray
    right_threat_mask: np.ndarray
    summary: str

    @classmethod
    def empty(cls, point_count: int = 0, summary: str = "Clear") -> "AvoidanceAnalysis":
        mask = np.zeros(point_count, dtype=bool)
        return cls(
            contact_min_m=None,
            front_min_m=None,
            left_min_m=None,
            right_min_m=None,
            contact_mask=mask.copy(),
            front_caution_mask=mask.copy(),
            front_emergency_mask=mask.copy(),
            left_threat_mask=mask.copy(),
            right_threat_mask=mask.copy(),
            summary=summary,
        )


def _masked_min(distance_m: np.ndarray, mask: np.ndarray) -> float | None:
    if not np.any(mask):
        return None
    return float(distance_m[mask].min())


def analyze_avoidance(
    x_m: np.ndarray,
    y_m: np.ndarray,
    distance_m: np.ndarray,
    mode: str,
    prev_left_m: float | None,
    prev_right_m: float | None,
) -> AvoidanceAnalysis:
    point_count = len(distance_m)
    if point_count == 0:
        return AvoidanceAnalysis.empty()

    valid_mask = distance_m >= MIN_VALID_DIST_M
    contact_sector = valid_mask & (x_m >= CONTACT_MIN_FORWARD_M) & (np.abs(y_m) <= CONTACT_HALF_WIDTH_M)
    front_sector = valid_mask & (x_m >= FRONT_MIN_FORWARD_M) & (np.abs(y_m) <= FRONT_HALF_WIDTH_M)
    left_sector = valid_mask & (x_m >= SIDE_MIN_FORWARD_M) & (y_m >= SIDE_MIN_LATERAL_M)
    right_sector = valid_mask & (x_m >= SIDE_MIN_FORWARD_M) & (y_m <= -SIDE_MIN_LATERAL_M)

    contact_min_m = _masked_min(distance_m, contact_sector)
    front_min_m = _masked_min(distance_m, front_sector)
    left_min_m = _masked_min(distance_m, left_sector)
    right_min_m = _masked_min(distance_m, right_sector)

    contact_mask = np.zeros(point_count, dtype=bool)
    front_caution_mask = np.zeros(point_count, dtype=bool)
    front_emergency_mask = np.zeros(point_count, dtype=bool)
    left_threat_mask = np.zeros(point_count, dtype=bool)
    right_threat_mask = np.zeros(point_count, dtype=bool)
    summary_parts: list[str] = []

    emergency_front_m = MODE_EMERGENCY_FRONT_M.get(mode)
    caution_front_m = MODE_CAUTION_FRONT_M.get(mode)

    if contact_min_m is not None and front_min_m is None:
        contact_mask = contact_sector & (distance_m <= CONTACT_EMERGENCY_M)
        if np.any(contact_mask):
            summary_parts.append(f"contact {contact_min_m:.2f} m")

    if emergency_front_m is not None and front_min_m is not None:
        front_emergency_mask = front_sector & (distance_m <= emergency_front_m)
        if np.any(front_emergency_mask):
            summary_parts.append(f"front emergency {front_min_m:.2f} m")
        elif caution_front_m is not None:
            front_caution_mask = front_sector & (distance_m <= caution_front_m)
            if np.any(front_caution_mask):
                summary_parts.append(f"front caution {front_min_m:.2f} m")

    summary = " | ".join(summary_parts) if summary_parts else "Clear"
    return AvoidanceAnalysis(
        contact_min_m=contact_min_m,
        front_min_m=front_min_m,
        left_min_m=left_min_m,
        right_min_m=right_min_m,
        contact_mask=contact_mask,
        front_caution_mask=front_caution_mask,
        front_emergency_mask=front_emergency_mask,
        left_threat_mask=left_threat_mask,
        right_threat_mask=right_threat_mask,
        summary=summary,
    )


def overlay_avoidance_colors(base_colors: np.ndarray, analysis: AvoidanceAnalysis) -> np.ndarray:
    if base_colors.size == 0:
        return base_colors

    colors = base_colors.copy()
    if np.any(analysis.contact_mask):
        colors[analysis.contact_mask] = COLOR_CONTACT
    if np.any(analysis.front_caution_mask):
        colors[analysis.front_caution_mask] = COLOR_FRONT_CAUTION
    if np.any(analysis.front_emergency_mask):
        colors[analysis.front_emergency_mask] = COLOR_FRONT_EMERGENCY
    return colors


class AvoidanceTracker:
    """Tracks previous scan sectors so viewers can mirror bot avoidance logic."""

    def __init__(self) -> None:
        self._last_scan_count = -1
        self._last_mode: str | None = None
        self._prev_left_m: float | None = None
        self._prev_right_m: float | None = None
        self._last_analysis = AvoidanceAnalysis.empty()

    def reset(self) -> None:
        self._last_scan_count = -1
        self._last_mode = None
        self._prev_left_m = None
        self._prev_right_m = None
        self._last_analysis = AvoidanceAnalysis.empty()

    def analyze(
        self,
        scan_count: int,
        x_m: np.ndarray,
        y_m: np.ndarray,
        distance_m: np.ndarray,
        mode: str,
    ) -> AvoidanceAnalysis:
        if mode != self._last_mode:
            self._prev_left_m = None
            self._prev_right_m = None

        if scan_count == self._last_scan_count and mode == self._last_mode:
            return self._last_analysis

        analysis = analyze_avoidance(
            x_m=x_m,
            y_m=y_m,
            distance_m=distance_m,
            mode=mode,
            prev_left_m=self._prev_left_m,
            prev_right_m=self._prev_right_m,
        )
        self._prev_left_m = analysis.left_min_m
        self._prev_right_m = analysis.right_min_m
        self._last_scan_count = scan_count
        self._last_mode = mode
        self._last_analysis = analysis
        return analysis
