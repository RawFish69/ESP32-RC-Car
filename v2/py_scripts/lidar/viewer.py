"""LD06 lidar 2D top-down viewer with optional map accumulation."""

from __future__ import annotations

import argparse
import logging
import math
import time
from collections import deque

import numpy as np
import viser

from . import config
from .avoidance import AvoidanceAnalysis, AvoidanceTracker, overlay_avoidance_colors
from .motion import draw_motion_indicator, motion_label
from .scene import create_scene
from .serial_reader import SerialReader, ScanSnapshot

logger = logging.getLogger("lidar")


def _colors_radar(intensity: np.ndarray) -> np.ndarray:
    t = intensity.astype(np.float32) / 255.0
    r = np.zeros(len(t), dtype=np.uint8)
    g = (t * 190 + 30).astype(np.uint8)
    b = (t * 195 + 60).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_distance(distance_m: np.ndarray) -> np.ndarray:
    t = np.clip(
        (distance_m - config.MIN_DIST_M) / (config.MAX_DIST_M - config.MIN_DIST_M), 0.0, 1.0,
    ).astype(np.float32)
    r = (t * 255).astype(np.uint8)
    g = np.zeros(len(t), dtype=np.uint8)
    b = ((1.0 - t) * 255).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def _colors_intensity(intensity: np.ndarray) -> np.ndarray:
    return intensity[:, np.newaxis].repeat(3, axis=1)


def _get_colors(distance_m: np.ndarray, intensity: np.ndarray, mode: str) -> np.ndarray:
    if mode == "Intensity":
        return _colors_intensity(intensity)
    if mode == "Distance":
        return _colors_distance(distance_m)
    return _colors_radar(intensity)


def _theta_to_wxyz(theta_rad: float) -> tuple[float, float, float, float]:
    h = theta_rad * 0.5
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _rotate_scan_to_robot_axes(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return y, -x


class LidarViewer:
    def __init__(self, port: str, baud: int = config.BAUD_RATE) -> None:
        self.serial_reader = SerialReader(port, baud)
        self.scene = None
        self._avoidance = AvoidanceTracker()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._map: deque[tuple[np.ndarray, np.ndarray, np.ndarray]] = deque()
        self._last_scan_count = -1

    def _setup_gui(self, server: viser.ViserServer) -> None:
        with server.gui.add_folder("Sensor"):
            self.status_text   = server.gui.add_text("Status",    initial_value="Waiting…")
            self.motion_text = server.gui.add_text("Motion", initial_value="stopped")
            self.avoidance_text = server.gui.add_text("Avoidance", initial_value="Clear")
            self.scan_fps_text = server.gui.add_text("Scan FPS",  initial_value="0.0")
            self.imu_text      = server.gui.add_text("IMU",       initial_value="Not detected")
            self.imu_fps_text  = server.gui.add_text("IMU FPS",   initial_value="0.0")
            self.points_text   = server.gui.add_text("Points",    initial_value="0")
            self.range_text    = server.gui.add_text("Range",     initial_value="--")

        with server.gui.add_folder("View"):
            self.color_mode = server.gui.add_dropdown(
                "Color Mode", options=["Radar", "Distance", "Intensity"], initial_value="Radar",
            )
            self.avoidance_mode = server.gui.add_dropdown(
                "Avoidance Profile",
                options=["Off", "Easy", "Normal", "Aggressive", "Crazy"],
                initial_value="Normal",
            )
            self.point_size_slider = server.gui.add_slider(
                "Point Size", min=0.005, max=0.15, step=0.005, initial_value=0.02,
            )
            self.max_dist_slider = server.gui.add_slider(
                "Max Distance (m)", min=1.0, max=config.MAX_DIST_M, step=0.5,
                initial_value=config.MAX_DIST_M,
            )
            self.apply_imu_checkbox = server.gui.add_checkbox(
                "Apply IMU Rotation", initial_value=True,
            )

        with server.gui.add_folder("Map"):
            self.accumulate_checkbox = server.gui.add_checkbox("Accumulate Scans", initial_value=False)
            self.max_scans_slider = server.gui.add_slider(
                "Max Scans", min=10, max=config.MAX_MAP_SCANS, step=10, initial_value=100,
            )
            self.map_point_size_slider = server.gui.add_slider(
                "Map Point Size", min=0.005, max=0.1, step=0.005, initial_value=0.025,
            )
            self.clear_map_button = server.gui.add_button("Clear Map")

            @self.clear_map_button.on_click
            def _clear(_) -> None:
                self._map.clear()
                server.scene.remove_by_name("/map/points")

    def _maybe_accumulate(self, snapshot: ScanSnapshot) -> None:
        if not self.accumulate_checkbox.value or snapshot.scan_count == self._last_scan_count:
            return
        if snapshot.x_m.size == 0:
            return
        self._last_scan_count = snapshot.scan_count
        c = math.cos(self._robot_theta)
        s = math.sin(self._robot_theta)
        local_x, local_y = _rotate_scan_to_robot_axes(snapshot.x_m, snapshot.y_m)
        wx = self._robot_x + c * local_x - s * local_y
        wy = self._robot_y + s * local_x + c * local_y
        self._map.append((wx, wy, snapshot.intensity.copy()))
        max_scans = int(self.max_scans_slider.value)
        while len(self._map) > max_scans:
            self._map.popleft()

    def _draw_map(self, server: viser.ViserServer) -> None:
        if not self._map:
            return
        all_wx = np.concatenate([e[0] for e in self._map])
        all_wy = np.concatenate([e[1] for e in self._map])
        all_i  = np.concatenate([e[2] for e in self._map])
        pts = np.stack([all_wx, all_wy, np.zeros_like(all_wx)], axis=1).astype(np.float32)
        t = all_i.astype(np.float32) / 255.0
        colors = np.stack([
            (t * 140 + 80).astype(np.uint8),
            (t * 100 + 60).astype(np.uint8),
            np.full(len(t), 20, dtype=np.uint8),
        ], axis=1)
        server.scene.add_point_cloud(
            "/map/points", points=pts, colors=colors,
            point_size=float(self.map_point_size_slider.value), point_shape="circle",
        )

    def _process_frame(self, server: viser.ViserServer) -> None:
        snapshot = self.serial_reader.get_snapshot()
        motion_desc = motion_label(
            mode=snapshot.motion_mode,
            direction=snapshot.motion_dir,
            dodging=snapshot.dodging,
            wall_side=snapshot.wall_side,
        )

        self.status_text.value   = "Connected" if snapshot.connected else "Disconnected"
        self.motion_text.value = motion_desc
        self.scan_fps_text.value = f"{snapshot.scan_fps:.1f}"
        self.imu_text.value      = "Connected" if snapshot.imu_connected else "Not detected"
        self.imu_fps_text.value  = f"{snapshot.imu_fps:.1f}"
        self.points_text.value   = str(snapshot.x_m.size)

        self.scene.robot.position = (self._robot_x, self._robot_y, 0.0)
        if self.apply_imu_checkbox.value and snapshot.imu_connected:
            self.scene.robot.wxyz = tuple(snapshot.quaternion)
        else:
            self.scene.robot.wxyz = _theta_to_wxyz(self._robot_theta)

        self._maybe_accumulate(snapshot)
        if self.accumulate_checkbox.value and self._map:
            self._draw_map(server)
        elif not self.accumulate_checkbox.value:
            server.scene.remove_by_name("/map/points")

        if not snapshot.connected or snapshot.x_m.size == 0:
            self.range_text.value = "--"
            self.motion_text.value = "stopped"
            self.avoidance_text.value = "Clear"
            self._avoidance.reset()
            server.scene.remove_by_name("/robot/scan/points")
            draw_motion_indicator(server.scene, "/robot", "x", False)
            return

        draw_motion_indicator(server.scene, "/robot", snapshot.motion_dir, snapshot.dodging)

        max_dist = float(self.max_dist_slider.value)
        mask = snapshot.distance_m <= max_dist
        if not np.any(mask):
            self.range_text.value = "No points in range"
            self.avoidance_text.value = "Clear"
            server.scene.remove_by_name("/robot/scan/points")
            return

        profile_to_mode = {
            "Off": "x",
            "Easy": "2",
            "Normal": "3",
            "Aggressive": "4",
            "Crazy": "5",
        }
        analysis = self._avoidance.analyze(
            scan_count=snapshot.scan_count,
            x_m=snapshot.x_m,
            y_m=snapshot.y_m,
            distance_m=snapshot.distance_m,
            mode=snapshot.motion_mode if snapshot.motion_mode in {"2", "3", "4", "5"} else profile_to_mode[str(self.avoidance_mode.value)],
        )

        x, y = _rotate_scan_to_robot_axes(snapshot.x_m[mask], snapshot.y_m[mask])
        dist  = snapshot.distance_m[mask]
        inten = snapshot.intensity[mask]
        colors = _get_colors(dist, inten, str(self.color_mode.value))
        masked_analysis = AvoidanceAnalysis(
            contact_min_m=analysis.contact_min_m,
            front_min_m=analysis.front_min_m,
            left_min_m=analysis.left_min_m,
            right_min_m=analysis.right_min_m,
            contact_mask=analysis.contact_mask[mask],
            front_caution_mask=analysis.front_caution_mask[mask],
            front_emergency_mask=analysis.front_emergency_mask[mask],
            left_threat_mask=analysis.left_threat_mask[mask],
            right_threat_mask=analysis.right_threat_mask[mask],
            summary=analysis.summary,
        )
        self.avoidance_text.value = analysis.summary

        server.scene.add_point_cloud(
            "/robot/scan/points",
            points=np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32),
            colors=overlay_avoidance_colors(colors, masked_analysis),
            point_size=float(self.point_size_slider.value),
            point_shape="circle",
        )
        self.range_text.value = f"{float(dist.min()):.2f}–{float(dist.max()):.2f} m"

    def run(self, host: str = "0.0.0.0", port: int = 8080) -> None:
        self.serial_reader.connect()
        self.serial_reader.start()

        server = viser.ViserServer(host=host, port=port)
        self.scene = create_scene(server)
        self._setup_gui(server)

        @server.on_client_connect
        def _on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, 0.0, 7.0)
            client.camera.look_at  = (0.0, 0.0, 0.0)
            client.camera.up       = (0.0, 1.0, 0.0)
            client.camera.fov      = 0.9

        logger.info("Viewer running at http://localhost:%d", port)

        try:
            while True:
                t0 = time.time()
                self._process_frame(server)
                elapsed = time.time() - t0
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)
        except KeyboardInterrupt:
            pass
        finally:
            self.serial_reader.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="LD06 Lidar Viewer for TA Bot")
    parser.add_argument("--port",  "-p", required=True, help="Serial port (e.g. COM3)")
    parser.add_argument("--baud",  "-b", type=int, default=config.BAUD_RATE)
    parser.add_argument("--host",        default="0.0.0.0")
    parser.add_argument("--web-port",    type=int, default=8080)
    parser.add_argument("--debug",       action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    LidarViewer(port=args.port, baud=args.baud).run(host=args.host, port=args.web_port)
