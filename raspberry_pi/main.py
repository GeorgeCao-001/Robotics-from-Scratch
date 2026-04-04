from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass
from math import isfinite
from typing import Any

from raspberry_pi.hardware import SerialComm, SerialConfig
from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import VisionTarget


@dataclass(frozen=True)
class RuntimeConfig:
    port: str
    baudrate: int = 115200
    camera_id: int = 0
    control_hz: float = 20.0
    detection_stale_s: float = 0.2
    status_interval_s: float = 0.0


class SharedVisionState:
    def __init__(self):
        self._lock = threading.Lock()
        self._target: VisionTarget | None = None
        self._last_seen_s: float = 0.0
        self._vision_failed = False
        self._vision_error: str | None = None

    def update(self, target: VisionTarget, now_s: float) -> None:
        with self._lock:
            self._target = target
            self._last_seen_s = now_s

    def get(self, now_s: float, stale_s: float) -> VisionTarget | None:
        with self._lock:
            if self._target is None:
                return None
            if now_s - self._last_seen_s > stale_s:
                return None
            return self._target

    def set_vision_failed(self, error: Exception) -> None:
        with self._lock:
            self._vision_failed = True
            self._vision_error = str(error)

    def vision_failed(self) -> tuple[bool, str | None]:
        with self._lock:
            return self._vision_failed, self._vision_error


def _to_vision_target(pose_info: dict[str, Any]) -> VisionTarget | None:
    required = [
        "x_error_norm",
        "y_error_norm",
        "height_norm",
        "width_norm",
    ]
    if any(key not in pose_info for key in required):
        return None

    try:
        x_error_norm = float(pose_info["x_error_norm"])
        y_error_norm = float(pose_info["y_error_norm"])
        height_norm = float(pose_info["height_norm"])
        width_norm = float(pose_info["width_norm"])
        confidence = float(pose_info.get("confidence", 1.0))
    except (TypeError, ValueError):
        return None

    values = [x_error_norm, y_error_norm, height_norm, width_norm, confidence]
    if not all(isfinite(v) for v in values):
        return None

    if not (-1.0 <= x_error_norm <= 1.0 and -1.0 <= y_error_norm <= 1.0):
        return None
    if not (0.0 <= height_norm <= 1.0 and 0.0 <= width_norm <= 1.0):
        return None
    confidence = min(max(confidence, 0.0), 1.0)

    return VisionTarget(
        x_error_norm=x_error_norm,
        y_error_norm=y_error_norm,
        height_norm=height_norm,
        width_norm=width_norm,
        confidence=confidence,
    )


def _run_control_loop(
    planner: Planner,
    comm: SerialComm,
    shared: SharedVisionState,
    runtime: RuntimeConfig,
    vision_alive: callable,
    now_fn: callable = time.monotonic,
    sleep_fn: callable = time.sleep,
) -> None:
    period_s = 1.0 / max(runtime.control_hz, 1.0)
    last_t = now_fn()
    last_status_t = last_t

    while vision_alive():
        failed, error = shared.vision_failed()
        if failed:
            if error:
                print(f"[MAIN] vision failed: {error}")
            comm.send_stop()
            return

        now_t = now_fn()
        dt_s = max(0.0, now_t - last_t)
        last_t = now_t

        target = shared.get(now_t, runtime.detection_stale_s)
        cmds = planner.update(target, dt_s)
        for cmd in cmds:
            comm.send_message(cmd)

        if (
            runtime.status_interval_s > 0
            and now_t - last_status_t >= runtime.status_interval_s
        ):
            last_status_t = now_t
            try:
                comm.request_status()
            except Exception as exc:  # pragma: no cover
                print(f"[MAIN] status request failed: {exc}")

        sleep_fn(period_s)


def run(runtime: RuntimeConfig) -> None:
    planner = Planner(PlanningConfig())
    shared = SharedVisionState()
    comm = SerialComm(SerialConfig(port=runtime.port, baudrate=runtime.baudrate))

    def on_detected(pose_info: dict[str, Any]) -> None:
        target = _to_vision_target(pose_info)
        if target is None:
            return
        shared.update(target, time.monotonic())

    def vision_worker() -> None:
        try:
            from raspberry_pi.vision.pose_landmarker import (
                run_pose_landmarker_on_camera,
            )

            run_pose_landmarker_on_camera(runtime.camera_id, on_detected=on_detected)
        except Exception as exc:  # pragma: no cover
            shared.set_vision_failed(exc)

    vision_thread = threading.Thread(target=vision_worker, daemon=True)

    try:
        comm.open()
        comm.send_stop()
        vision_thread.start()
        _run_control_loop(
            planner=planner,
            comm=comm,
            shared=shared,
            runtime=runtime,
            vision_alive=vision_thread.is_alive,
        )
    finally:
        try:
            comm.send_stop()
        except Exception:
            pass
        comm.close()


def _parse_args() -> RuntimeConfig:
    parser = argparse.ArgumentParser(description="Pose follow main pipeline")
    parser.add_argument(
        "--port", required=True, help="Serial port, e.g. /dev/cu.usbmodemxxxx"
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="UART baudrate")
    parser.add_argument("--camera-id", type=int, default=0, help="OpenCV camera id")
    parser.add_argument(
        "--control-hz", type=float, default=20.0, help="Control loop frequency"
    )
    parser.add_argument(
        "--detection-stale-s",
        type=float,
        default=0.2,
        help="Treat vision target as lost after this age",
    )
    parser.add_argument(
        "--status-interval-s",
        type=float,
        default=0.0,
        help="Optional status polling period; <=0 disables",
    )
    args = parser.parse_args()
    return RuntimeConfig(
        port=args.port,
        baudrate=args.baudrate,
        camera_id=args.camera_id,
        control_hz=args.control_hz,
        detection_stale_s=args.detection_stale_s,
        status_interval_s=args.status_interval_s,
    )


if __name__ == "__main__":
    run(_parse_args())
