"""Background serial reader for the LD06 JSON stream from the controller."""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass

import numpy as np
import serial

from . import config
from .imu_fusion import MadgwickFilter

logger = logging.getLogger("lidar.serial")


@dataclass(frozen=True)
class ScanSnapshot:
    x_m: np.ndarray
    y_m: np.ndarray
    distance_m: np.ndarray
    intensity: np.ndarray
    quaternion: np.ndarray
    connected: bool
    imu_connected: bool
    scan_fps: float
    imu_fps: float
    status_text: str
    scan_count: int
    motion_mode: str
    motion_dir: str
    dodging: bool
    wall_side: str


class SerialReader:
    def __init__(self, port: str, baud: int = config.BAUD_RATE) -> None:
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        self.filter = MadgwickFilter(beta=config.MADGWICK_BETA)

        self._x_m = np.zeros(0, dtype=np.float32)
        self._y_m = np.zeros(0, dtype=np.float32)
        self._distance_m = np.zeros(0, dtype=np.float32)
        self._intensity = np.zeros(0, dtype=np.uint8)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._status_text = "waiting_for_serial"
        self._motion_mode = "x"
        self._motion_dir = "x"
        self._dodging = False
        self._wall_side = "left"
        self._last_scan_wall = 0.0
        self._last_imu_wall = 0.0
        self._last_imu_ts_us: int | None = None

        self._scan_frame_count = 0
        self._scan_fps = 0.0
        self._scan_fps_window_start = time.time()
        self._imu_frame_count = 0
        self._imu_fps = 0.0
        self._imu_fps_window_start = time.time()
        self._scan_count = 0

    def connect(self) -> None:
        logger.info("Connecting to %s at %d baud", self.port, self.baud)
        self.serial = serial.Serial(self.port, self.baud, timeout=config.SERIAL_TIMEOUT_S)
        time.sleep(2.0)
        self.serial.reset_input_buffer()

    def start(self) -> None:
        if self._thread is not None:
            return
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_snapshot(self) -> ScanSnapshot:
        with self._lock:
            now = time.time()
            connected = (now - self._last_scan_wall) < config.SCAN_TIMEOUT_S
            imu_connected = (now - self._last_imu_wall) < config.IMU_TIMEOUT_S
            quaternion = self._quaternion.copy() if imu_connected else np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float32
            )
            return ScanSnapshot(
                x_m=self._x_m.copy(),
                y_m=self._y_m.copy(),
                distance_m=self._distance_m.copy(),
                intensity=self._intensity.copy(),
                quaternion=quaternion,
                connected=connected,
                imu_connected=imu_connected,
                scan_fps=self._scan_fps,
                imu_fps=self._imu_fps,
                status_text=self._status_text,
                scan_count=self._scan_count,
                motion_mode=self._motion_mode,
                motion_dir=self._motion_dir,
                dodging=self._dodging,
                wall_side=self._wall_side,
            )

    def _reset_runtime_state(self) -> None:
        with self._lock:
            self.filter.reset()
            self._quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            self._last_imu_ts_us = None
            self._last_scan_wall = 0.0
            self._last_imu_wall = 0.0
            self._status_text = "reconnecting"
            self._motion_mode = "x"
            self._motion_dir = "x"
            self._dodging = False
            self._wall_side = "left"

    def _reconnect(self) -> bool:
        self._reset_runtime_state()
        try:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
            self.serial = serial.Serial(self.port, self.baud, timeout=config.SERIAL_TIMEOUT_S)
            time.sleep(2.0)
            self.serial.reset_input_buffer()
            return True
        except (serial.SerialException, OSError) as exc:
            logger.debug("Reconnection failed: %s", exc)
            return False

    def _update_fps(self, kind: str) -> None:
        now = time.time()
        with self._lock:
            if kind == "scan":
                self._scan_frame_count += 1
                elapsed = now - self._scan_fps_window_start
                if elapsed >= 1.0:
                    self._scan_fps = self._scan_frame_count / elapsed
                    self._scan_frame_count = 0
                    self._scan_fps_window_start = now
            else:
                self._imu_frame_count += 1
                elapsed = now - self._imu_fps_window_start
                if elapsed >= 1.0:
                    self._imu_fps = self._imu_frame_count / elapsed
                    self._imu_frame_count = 0
                    self._imu_fps_window_start = now

    def _handle_scan(self, data: dict) -> None:
        x_raw = data.get("x")
        y_raw = data.get("y")
        i_raw = data.get("i")
        if not (isinstance(x_raw, list) and isinstance(y_raw, list) and isinstance(i_raw, list)):
            return
        if not (len(x_raw) == len(y_raw) == len(i_raw)) or len(x_raw) == 0:
            return

        x_m = np.asarray(x_raw, dtype=np.float32) / 1000.0
        y_m = np.asarray(y_raw, dtype=np.float32) / 1000.0
        dist_m = np.hypot(x_m, y_m)
        intensity = np.asarray(i_raw, dtype=np.uint8)

        with self._lock:
            self._x_m = x_m
            self._y_m = y_m
            self._distance_m = dist_m
            self._intensity = intensity
            self._last_scan_wall = time.time()
            self._scan_count += 1

        self._update_fps("scan")

    def _handle_imu(self, data: dict) -> None:
        accel = data.get("accel_mps2")
        gyro = data.get("gyro_rads")
        ts_us = data.get("ts_us")
        if not (isinstance(accel, list) and isinstance(gyro, list)):
            return
        if len(accel) != 3 or len(gyro) != 3:
            return
        if not isinstance(ts_us, int):
            return

        accel_np = np.asarray(accel, dtype=np.float32)
        gyro_np = np.asarray(gyro, dtype=np.float32)

        with self._lock:
            if self._last_imu_ts_us is None or ts_us <= self._last_imu_ts_us:
                dt = config.DEFAULT_IMU_DT_S
            else:
                dt = (ts_us - self._last_imu_ts_us) / 1_000_000.0
                dt = min(max(dt, 1.0e-4), 0.1)
            quat = self.filter.update(gyro_np, accel_np, dt)
            self._quaternion = quat.astype(np.float32)
            self._last_imu_ts_us = ts_us
            self._last_imu_wall = time.time()

        self._update_fps("imu")

    def _handle_status(self, data: dict) -> None:
        stage = str(data.get("stage", ""))
        detail = str(data.get("detail", ""))
        with self._lock:
            self._status_text = f"{stage}: {detail}".strip(": ")

    def _handle_motion(self, data: dict) -> None:
        mode = str(data.get("mode", "x"))[:1] or "x"
        direction = str(data.get("dir", "x"))[:1] or "x"
        wall_side = str(data.get("wall_side", "left"))
        with self._lock:
            self._motion_mode = mode
            self._motion_dir = direction
            self._dodging = bool(data.get("dodging", False))
            self._wall_side = wall_side if wall_side in {"left", "right"} else "left"

    def _handle_packet(self, data: dict) -> None:
        t = data.get("t")
        if t == "scan":
            self._handle_scan(data)
        elif t == "imu":
            self._handle_imu(data)
        elif t == "status":
            self._handle_status(data)
        elif t == "motion":
            self._handle_motion(data)

    def _read_loop(self) -> None:
        while self.running:
            try:
                if self.serial is None or not self.serial.is_open:
                    time.sleep(0.1)
                    continue

                raw = self.serial.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line.startswith("{"):
                    continue

                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if isinstance(packet, dict):
                    self._handle_packet(packet)

            except (serial.SerialException, OSError) as exc:
                if not self.running:
                    break
                logger.warning("Serial connection lost: %s", exc)
                while self.running:
                    if self._reconnect():
                        break
                    time.sleep(1.0)
