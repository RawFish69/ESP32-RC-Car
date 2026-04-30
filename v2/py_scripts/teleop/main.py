"""Keyboard teleop + web lidar viewer for TA Bot."""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
import serial
import viser

from lidar import create_scene
from lidar.avoidance import AvoidanceAnalysis, AvoidanceTracker, overlay_avoidance_colors
from lidar.motion import draw_motion_indicator, motion_label


MODE_KEYS = {"1", "2", "3", "4", "5", "x"}
RAMP_KEYS = {"w", "a", "s", "d", "q", "e", "x", " "}
DIRECT_MOTOR_PREFIXES = {"L", "R"}

SCAN_CONNECTED_TIMEOUT_S = 2.0
KEY_STEP_PWM = 24
SPIN_STEP_PWM = 36
RAMP_STEP_PWM = 10
MAX_PWM = 220
SEND_INTERVAL_S = 0.08
PANEL_REFRESH_S = 0.1
LOOP_SLEEP_S = 0.02
PANEL_HEARTBEAT_S = 1.0


def _rotate_scan_to_robot_axes(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return y, -x


@dataclass
class TelemetryState:
    scan_count: int = 0
    last_scan_wall: float = 0.0
    last_imu_wall: float = 0.0
    last_status: str = "waiting_for_serial"
    imu_ok: bool = False
    yaw_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    motion_mode: str = "x"
    motion_dir: str = "x"
    dodging: bool = False
    wall_side: str = "left"


@dataclass
class DriveState:
    target_left: int = 0
    target_right: int = 0
    current_left: int = 0
    current_right: int = 0
    last_sent_left: int | None = None
    last_sent_right: int | None = None
    last_send_wall: float = 0.0


@dataclass(frozen=True)
class ScanSnapshot:
    x_m: np.ndarray
    y_m: np.ndarray
    distance_m: np.ndarray
    intensity: np.ndarray
    connected: bool
    imu_connected: bool
    imu_ok: bool
    yaw_deg: float
    yaw_rate_dps: float
    status_text: str
    scan_count: int
    motion_mode: str
    motion_dir: str
    dodging: bool
    wall_side: str


class ControllerBridge:
    """Reads scan JSON from the controller and sends drive commands back."""

    def __init__(self, port: str, baud: int = 460800) -> None:
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False
        self._reader_thread: threading.Thread | None = None
        self._log_queue: queue.Queue[str] = queue.Queue()
        self._state_lock = threading.Lock()

        self.telemetry = TelemetryState()
        self.drive = DriveState()
        self.current_mode = "x"

        self._x_m = np.zeros(0, dtype=np.float32)
        self._y_m = np.zeros(0, dtype=np.float32)
        self._distance_m = np.zeros(0, dtype=np.float32)
        self._intensity = np.zeros(0, dtype=np.uint8)

    def connect(self) -> None:
        self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)
        self.serial.reset_input_buffer()

    def start(self) -> None:
        if self.serial is None:
            raise RuntimeError("Not connected")
        if self._reader_thread is not None:
            return
        self.running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        self.running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        if self.serial is not None:
            try:
                self.serial.close()
            except serial.SerialException:
                pass
            self.serial = None

    def send_line(self, payload: str) -> None:
        if self.serial is None:
            raise RuntimeError("Not connected")
        self.serial.write(payload.encode("utf-8"))
        self.serial.flush()

    def send_mode(self, mode: str) -> None:
        self.current_mode = mode
        self.send_line(mode)
        self._log_queue.put(f"mode -> {mode}")
        if mode != "1":
            self.set_targets(0, 0)
            self.drive.current_left = 0
            self.drive.current_right = 0
            self.drive.last_sent_left = 0
            self.drive.last_sent_right = 0

    def set_targets(self, left: int, right: int) -> None:
        self.drive.target_left = max(-MAX_PWM, min(MAX_PWM, left))
        self.drive.target_right = max(-MAX_PWM, min(MAX_PWM, right))

    def nudge_targets(self, key: str) -> None:
        if key in {" ", "x"}:
            self.set_targets(0, 0)
            return
        if self.current_mode != "1":
            self.send_mode("1")

        left = self.drive.target_left
        right = self.drive.target_right
        if key == "w":
            left += KEY_STEP_PWM; right += KEY_STEP_PWM
        elif key == "s":
            left -= KEY_STEP_PWM; right -= KEY_STEP_PWM
        elif key == "a":
            dl, dr = self._steer_deltas("a"); left += dl; right += dr
        elif key == "d":
            dl, dr = self._steer_deltas("d"); left += dl; right += dr
        elif key == "q":
            left -= SPIN_STEP_PWM; right += SPIN_STEP_PWM
        elif key == "e":
            left += SPIN_STEP_PWM; right -= SPIN_STEP_PWM

        self.set_targets(left, right)
        self._log_queue.put(f"target -> L {self.drive.target_left:+d}  R {self.drive.target_right:+d}")

    def _steer_deltas(self, key: str) -> tuple[int, int]:
        reversing = self._is_reversing()
        if key == "a":
            return (KEY_STEP_PWM, -KEY_STEP_PWM) if reversing else (-KEY_STEP_PWM, KEY_STEP_PWM)
        return (-KEY_STEP_PWM, KEY_STEP_PWM) if reversing else (KEY_STEP_PWM, -KEY_STEP_PWM)

    def send_direct_motor(self, command: str) -> None:
        self.send_line(command + "\n")
        self._log_queue.put(f"motor -> {command}")

    def tick(self) -> None:
        if self.current_mode != "1":
            return

        self.drive.current_left  = _ramp_toward(self.drive.current_left,  self.drive.target_left,  RAMP_STEP_PWM)
        self.drive.current_right = _ramp_toward(self.drive.current_right, self.drive.target_right, RAMP_STEP_PWM)

        now = time.time()
        changed = (
            self.drive.current_left  != self.drive.last_sent_left or
            self.drive.current_right != self.drive.last_sent_right
        )
        keepalive = not self._is_fully_stopped() and (now - self.drive.last_send_wall) >= SEND_INTERVAL_S
        if changed or keepalive:
            self._send_wheel_speeds(force=changed)

    def _send_wheel_speeds(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self.drive.last_send_wall) < SEND_INTERVAL_S:
            return
        self.send_line(_format_motor_cmd("L", self.drive.current_left)  + "\n")
        self.send_line(_format_motor_cmd("R", self.drive.current_right) + "\n")
        self.drive.last_sent_left  = self.drive.current_left
        self.drive.last_sent_right = self.drive.current_right
        self.drive.last_send_wall  = now

    def _is_fully_stopped(self) -> bool:
        d = self.drive
        return d.target_left == 0 and d.target_right == 0 and d.current_left == 0 and d.current_right == 0

    def _is_reversing(self) -> bool:
        ta = self.drive.target_left + self.drive.target_right
        ca = self.drive.current_left + self.drive.current_right
        return ta < 0 or (ta == 0 and ca < 0)

    def get_snapshot(self) -> ScanSnapshot:
        with self._state_lock:
            now = time.time()
            return ScanSnapshot(
                x_m=self._x_m.copy(), y_m=self._y_m.copy(),
                distance_m=self._distance_m.copy(), intensity=self._intensity.copy(),
                connected=(now - self.telemetry.last_scan_wall) < SCAN_CONNECTED_TIMEOUT_S,
                imu_connected=(now - self.telemetry.last_imu_wall) < SCAN_CONNECTED_TIMEOUT_S,
                imu_ok=self.telemetry.imu_ok,
                yaw_deg=self.telemetry.yaw_deg, yaw_rate_dps=self.telemetry.yaw_rate_dps,
                status_text=self.telemetry.last_status, scan_count=self.telemetry.scan_count,
                motion_mode=self.telemetry.motion_mode, motion_dir=self.telemetry.motion_dir,
                dodging=self.telemetry.dodging, wall_side=self.telemetry.wall_side,
            )

    def drain_logs(self) -> list[str]:
        msgs: list[str] = []
        while True:
            try:
                msgs.append(self._log_queue.get_nowait())
            except queue.Empty:
                return msgs

    def _handle_scan(self, data: dict) -> None:
        x_raw, y_raw, i_raw = data.get("x"), data.get("y"), data.get("i")
        if not (isinstance(x_raw, list) and isinstance(y_raw, list) and isinstance(i_raw, list)):
            return
        if not (len(x_raw) == len(y_raw) == len(i_raw)) or not x_raw:
            return
        x_m = np.asarray(x_raw, dtype=np.float32) / 1000.0
        y_m = np.asarray(y_raw, dtype=np.float32) / 1000.0
        with self._state_lock:
            self._x_m = x_m
            self._y_m = y_m
            self._distance_m = np.hypot(x_m, y_m)
            self._intensity = np.asarray(i_raw, dtype=np.uint8)
            self.telemetry.scan_count += 1
            self.telemetry.last_scan_wall = time.time()

    def _handle_status(self, data: dict) -> None:
        stage  = str(data.get("stage", ""))
        detail = str(data.get("detail", ""))
        status = f"{stage}: {detail}".strip(": ")
        with self._state_lock:
            self.telemetry.last_status = status
        self._log_queue.put(f"status -> {status}")

    def _handle_imu(self, data: dict) -> None:
        with self._state_lock:
            self.telemetry.last_imu_wall = time.time()
            self.telemetry.imu_ok        = bool(data.get("imu_ok", False))
            self.telemetry.yaw_deg       = float(data.get("yaw_deg", 0.0))
            self.telemetry.yaw_rate_dps  = float(data.get("yaw_rate_dps", 0.0))

    def _handle_motion(self, data: dict) -> None:
        with self._state_lock:
            self.telemetry.motion_mode = str(data.get("mode", "x"))[:1] or "x"
            self.telemetry.motion_dir = str(data.get("dir", "x"))[:1] or "x"
            self.telemetry.dodging = bool(data.get("dodging", False))
            wall_side = str(data.get("wall_side", "left"))
            self.telemetry.wall_side = wall_side if wall_side in {"left", "right"} else "left"
            self.current_mode = self.telemetry.motion_mode

    def _read_loop(self) -> None:
        assert self.serial is not None
        while self.running:
            try:
                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if not line.startswith("{"):
                    self._log_queue.put(line)
                    continue
                try:
                    packet = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(packet, dict):
                    continue
                t = packet.get("t")
                if t == "scan":   self._handle_scan(packet)
                elif t == "status": self._handle_status(packet)
                elif t == "imu":    self._handle_imu(packet)
                elif t == "motion": self._handle_motion(packet)
            except (serial.SerialException, OSError) as exc:
                self._log_queue.put(f"serial error: {exc}")
                self.running = False


class WebViewer:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self.server = viser.ViserServer(host=host, port=port)
        self.robot_frame = create_scene(self.server).robot
        self._avoidance = AvoidanceTracker()
        self._add_gui()

        @self.server.on_client_connect
        def _on_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, 0.0, 7.0)
            client.camera.look_at  = (0.0, 0.0, 0.0)
            client.camera.up       = (0.0, 1.0, 0.0)
            client.camera.fov      = 0.9

    @property
    def url(self) -> str:
        host = "localhost" if self._host in {"0.0.0.0", "::"} else self._host
        return f"http://{host}:{self._port}"

    def _add_gui(self) -> None:
        with self.server.gui.add_folder("TA Bot"):
            self.status_text     = self.server.gui.add_text("Status",        initial_value="Waiting…")
            self.mode_text       = self.server.gui.add_text("Mode",          initial_value="x")
            self.motion_text     = self.server.gui.add_text("Motion",        initial_value="stopped")
            self.avoid_text      = self.server.gui.add_text("Avoidance",     initial_value="Clear")
            self.imu_text        = self.server.gui.add_text("IMU",           initial_value="No data")
            self.heading_text    = self.server.gui.add_text("Heading (deg)", initial_value="0.0")
            self.turn_rate_text  = self.server.gui.add_text("Turn Rate (dps)", initial_value="0.0")
            self.scan_count_text = self.server.gui.add_text("Scan Count",    initial_value="0")
            self.points_text     = self.server.gui.add_text("Points",        initial_value="0")
            self.range_text      = self.server.gui.add_text("Range",         initial_value="--")

        with self.server.gui.add_folder("View"):
            self.color_mode = self.server.gui.add_dropdown(
                "Color Mode", options=["Radar", "Distance", "Intensity"], initial_value="Radar",
            )
            self.max_dist_slider  = self.server.gui.add_slider("Max Distance (m)", min=1.0, max=12.0, step=0.5, initial_value=12.0)
            self.point_size_slider = self.server.gui.add_slider("Point Size", min=0.005, max=0.15, step=0.005, initial_value=0.02)

    def render(self, snapshot: ScanSnapshot, mode: str) -> None:
        bot_mode = snapshot.motion_mode if snapshot.motion_mode in MODE_KEYS else mode
        motion_desc = motion_label(
            mode=bot_mode,
            direction=snapshot.motion_dir,
            dodging=snapshot.dodging,
            wall_side=snapshot.wall_side,
        )
        self.status_text.value     = f"{'Connected' if snapshot.connected else 'Disconnected'} ({snapshot.status_text})"
        self.mode_text.value       = bot_mode
        self.motion_text.value     = motion_desc
        self.scan_count_text.value = str(snapshot.scan_count)
        self.points_text.value     = str(snapshot.x_m.size)
        self.imu_text.value        = ("Healthy" if snapshot.imu_ok else "Connected (degraded)") if snapshot.imu_connected else "No data"
        self.heading_text.value    = f"{snapshot.yaw_deg:.1f}"
        self.turn_rate_text.value  = f"{snapshot.yaw_rate_dps:.1f}"

        if not snapshot.connected or snapshot.x_m.size == 0:
            self.range_text.value = "--"
            self.motion_text.value = "stopped"
            self.avoid_text.value = "Clear"
            self._avoidance.reset()
            self.server.scene.remove_by_name("/robot/scan/points")
            draw_motion_indicator(self.server.scene, "/robot", "x", False)
            return

        draw_motion_indicator(self.server.scene, "/robot", snapshot.motion_dir, snapshot.dodging)

        mask = snapshot.distance_m <= float(self.max_dist_slider.value)
        if not np.any(mask):
            self.range_text.value = "No points in range"
            self.avoid_text.value = "Clear"
            self.server.scene.remove_by_name("/robot/scan/points")
            return

        analysis = self._avoidance.analyze(
            scan_count=snapshot.scan_count,
            x_m=snapshot.x_m,
            y_m=snapshot.y_m,
            distance_m=snapshot.distance_m,
            mode=bot_mode,
        )

        x, y = _rotate_scan_to_robot_axes(snapshot.x_m[mask], snapshot.y_m[mask])
        dist = snapshot.distance_m[mask]
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
        self.avoid_text.value = analysis.summary

        self.server.scene.add_point_cloud(
            "/robot/scan/points",
            points=np.stack([x, y, np.zeros_like(x)], axis=1).astype(np.float32),
            colors=overlay_avoidance_colors(colors, masked_analysis),
            point_size=float(self.point_size_slider.value),
            point_shape="circle",
        )
        self.range_text.value = f"{float(dist.min()):.2f}–{float(dist.max()):.2f} m"


# ── colour helpers ────────────────────────────────────────────────────────────

def _get_colors(distance_m: np.ndarray, intensity: np.ndarray, mode: str) -> np.ndarray:
    if mode == "Intensity":
        return intensity[:, np.newaxis].repeat(3, axis=1)
    if mode == "Distance":
        t = np.clip((distance_m - 0.02) / (12.0 - 0.02), 0.0, 1.0).astype(np.float32)
        return np.stack([(t * 255).astype(np.uint8), np.zeros(len(t), np.uint8), ((1 - t) * 255).astype(np.uint8)], axis=1)
    t = intensity.astype(np.float32) / 255.0
    return np.stack([np.zeros(len(t), np.uint8), (t * 190 + 30).astype(np.uint8), (t * 195 + 60).astype(np.uint8)], axis=1)


# ── keyboard / terminal ───────────────────────────────────────────────────────

def _read_key_windows() -> str | None:
    import msvcrt
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getwch()
    if ch in ("\x00", "\xe0"):
        _ = msvcrt.getwch()
        return None
    return ch


class _PosixKeyboard:
    def __init__(self) -> None:
        import termios, tty
        self._termios = termios
        self._tty = tty
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self) -> "_PosixKeyboard":
        self._tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_) -> None:
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    @staticmethod
    def read_key() -> str | None:
        import select
        readable, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if readable else None


def _direction_label(left: int, right: int) -> str:
    if left == 0 and right == 0:         return "stopped"
    if left > 0 and right > 0:
        if abs(left - right) <= 12:      return "forward"
        return "arc_left" if right > left else "arc_right"
    if left < 0 and right < 0:
        if abs(left - right) <= 12:      return "reverse"
        return "reverse_left" if right < left else "reverse_right"
    if left < 0 < right:                 return "spin_left"
    if right < 0 < left:                 return "spin_right"
    if left == 0:                        return "pivot_left" if right > 0 else "pivot_right"
    if right == 0:                       return "pivot_right" if left > 0 else "pivot_left"
    return "mixed"


def _display_direction(bridge: ControllerBridge, snapshot: ScanSnapshot) -> str:
    if bridge.current_mode == "1":
        return _direction_label(bridge.drive.current_left, bridge.drive.current_right)
    return motion_label(
        mode=snapshot.motion_mode,
        direction=snapshot.motion_dir,
        dodging=snapshot.dodging,
        wall_side=snapshot.wall_side,
    )


def _render_panel(bridge: ControllerBridge, snapshot: ScanSnapshot, log_lines: list[str], url: str) -> str:
    d = bridge.drive
    direction_text = _display_direction(bridge, snapshot)
    lines = [
        "TA Bot — Teleop",
        "",
        f"port       : {bridge.port} @ {bridge.baud}",
        f"web viewer : {url}",
        f"mode       : {bridge.current_mode}",
        f"direction  : {direction_text}",
        f"lidar      : {'connected' if snapshot.connected else 'not_connected'}",
        f"imu        : {'ok' if snapshot.imu_ok and snapshot.imu_connected else ('connected' if snapshot.imu_connected else 'no_data')}",
        f"heading    : {snapshot.yaw_deg:+.1f} deg",
        f"turn_rate  : {snapshot.yaw_rate_dps:+.1f} dps",
        f"scan_count : {snapshot.scan_count}",
        f"status     : {snapshot.status_text}",
        "",
        "Wheel State",
        f"  left   current={d.current_left:+4d}  target={d.target_left:+4d}",
        f"  right  current={d.current_right:+4d}  target={d.target_right:+4d}",
        "",
        "Controls",
        "  1 teleop   2 easy   3 normal   4 aggressive   5 crazy   x/space stop",
        "  w/s forward/reverse   a/d steer   q/e spin",
        "  m direct motor command   p summary   esc quit",
        "",
        "Recent Serial",
    ]
    recent = log_lines[-8:] if log_lines else ["(no recent serial lines)"]
    lines.extend(f"  {l}" for l in recent)
    return "\x1b[2J\x1b[H" + "\n".join(lines)


def _panel_signature(bridge: ControllerBridge, snapshot: ScanSnapshot, log_lines: list[str], url: str) -> tuple:
    d = bridge.drive
    return (
        bridge.port, bridge.baud, url, bridge.current_mode,
        _display_direction(bridge, snapshot),
        snapshot.connected, snapshot.imu_connected, snapshot.imu_ok,
        round(snapshot.yaw_deg, 1), round(snapshot.yaw_rate_dps, 1),
        d.current_left, d.target_left, d.current_right, d.target_right,
        snapshot.status_text, tuple(log_lines[-8:]),
    )


# ── helpers ───────────────────────────────────────────────────────────────────

def _ramp_toward(current: int, target: int, step: int) -> int:
    if current < target: return min(current + step, target)
    if current > target: return max(current - step, target)
    return current


def _format_motor_cmd(motor: str, value: int) -> str:
    if value > 0:  return f"{motor}f{value}"
    if value < 0:  return f"{motor}b{abs(value)}"
    return f"{motor}s"


# ── main loop ─────────────────────────────────────────────────────────────────

def _run_loop(bridge: ControllerBridge, viewer: WebViewer) -> None:
    print(f"TA Bot teleop ready. Viewer: {viewer.url}")

    log_lines: list[str] = []
    last_panel_wall = 0.0
    last_sig: tuple | None = None
    is_windows = os.name == "nt"
    posix_kb = None

    if not is_windows:
        posix_kb = _PosixKeyboard()
        posix_kb.__enter__()

    try:
        while bridge.running:
            key = _read_key_windows() if is_windows else _PosixKeyboard.read_key()
            if key is not None:
                key = key.lower()
                if key == "\x1b":
                    break
                if key in MODE_KEYS:
                    bridge.send_mode(key)
                elif key in RAMP_KEYS:
                    bridge.nudge_targets(key)
                elif key == "m":
                    if posix_kb: posix_kb.__exit__(None, None, None)
                    print("\nDirect motor command (e.g. Lf200, Rs): ", end="", flush=True)
                    cmd = input().strip()
                    if cmd and cmd[0] in DIRECT_MOTOR_PREFIXES:
                        bridge.send_direct_motor(cmd)
                    if posix_kb: posix_kb.__enter__()
                elif key == "p":
                    snap = bridge.get_snapshot()
                    d = bridge.drive
                    print(
                        f"[summary] mode={bridge.current_mode} "
                        f"dir={_display_direction(bridge, snap)} "
                        f"L={d.current_left:+d}/{d.target_left:+d} "
                        f"R={d.current_right:+d}/{d.target_right:+d} "
                        f"scans={snap.scan_count} lidar={'ok' if snap.connected else 'no'} "
                        f"imu={'ok' if snap.imu_ok and snap.imu_connected else 'no'} "
                        f"heading={snap.yaw_deg:+.1f}deg"
                    )

            bridge.tick()
            snapshot = bridge.get_snapshot()
            viewer.render(snapshot, bridge.current_mode)

            for msg in bridge.drain_logs():
                if not msg.startswith("status -> ") or "controller: ready" not in msg:
                    if not log_lines or log_lines[-1] != msg:
                        log_lines.append(msg)

            now = time.time()
            if now - last_panel_wall >= PANEL_REFRESH_S:
                last_panel_wall = now
                sig = _panel_signature(bridge, snapshot, log_lines, viewer.url)
                if sig != last_sig:
                    print(_render_panel(bridge, snapshot, log_lines, viewer.url), end="", flush=True)
                    last_sig = sig

            time.sleep(LOOP_SLEEP_S)
    finally:
        if posix_kb:
            posix_kb.__exit__(None, None, None)


def main() -> None:
    parser = argparse.ArgumentParser(description="TA Bot keyboard teleop + web lidar viewer")
    parser.add_argument("--port",     "-p", required=True,        help="Controller serial port (e.g. COM3)")
    parser.add_argument("--baud",     "-b", type=int, default=460800)
    parser.add_argument("--host",           default="0.0.0.0",    help="Web viewer bind host")
    parser.add_argument("--web-port",       type=int, default=8080)
    args = parser.parse_args()

    bridge = ControllerBridge(port=args.port, baud=args.baud)
    viewer = WebViewer(host=args.host, port=args.web_port)

    try:
        bridge.connect()
        bridge.start()
        _run_loop(bridge, viewer)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
