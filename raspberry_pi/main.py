from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass
from math import isfinite
from typing import Any, Callable

from raspberry_pi.hardware import SerialComm, SerialConfig
from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import VisionTarget


@dataclass(frozen=True)
class RuntimeConfig:
    port: str
    baudrate: int = 9600
    camera_id: int = 0
    show_window: bool = False
    control_hz: float = 10.0
    detection_stale_s: float = 0.2
    status_interval_s: float = 0.0
    # optional, status query interval. no status query if <= 0
    # default: no status query


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
    vision_alive: Callable[[], bool],
    on_fatal_error: Callable[[Exception], None],
    now_fn: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    if runtime.control_hz <= 0.0:
        raise ValueError("control_hz must be > 0")

    period_s = 1.0 / runtime.control_hz
    last_t = now_fn()
    last_status_t = last_t

    while vision_alive():
        loop_start_t = now_fn()
        failed, error = shared.vision_failed()
        if failed:
            if error:
                print(f"[MAIN] vision failed: {error}")
            return

        now_t = now_fn()
        dt_s = max(0.0, now_t - last_t)
        last_t = now_t

        target = shared.get(now_t, runtime.detection_stale_s)
        cmds = planner.update(target, dt_s)
        try:
            for cmd in cmds:
                comm.send_message(cmd)
        except Exception as exc:
            print(f"[MAIN] send failed: {exc}")
            on_fatal_error(exc)
            try:
                comm.send_stop()
            except Exception:
                pass
            return

        if (
            runtime.status_interval_s > 0
            and now_t - last_status_t >= runtime.status_interval_s
        ):
            last_status_t = now_t
            try:
                comm.request_status()
            except Exception as exc:  # pragma: no cover
                print(f"[MAIN] status request failed: {exc}")

        elapsed = now_fn() - loop_start_t
        sleep_fn(max(0.0, period_s - elapsed))
        # nothing dealing with frame drops if it happens
        # but not that urgent


def run(runtime: RuntimeConfig) -> None:
    planner = Planner(PlanningConfig())
    shared = SharedVisionState()
    comm = SerialComm(SerialConfig(port=runtime.port, baudrate=runtime.baudrate))
    stop_event = threading.Event()

    def on_detected(pose_info: dict[str, Any]) -> None:
        if stop_event.is_set():
            raise RuntimeError("control loop stopped")
        target = _to_vision_target(pose_info)
        if target is None:
            return
        shared.update(target, time.monotonic())

    def control_worker() -> None:
        _run_control_loop(
            planner=planner,
            comm=comm,
            shared=shared,
            runtime=runtime,
            vision_alive=lambda: not stop_event.is_set(),
            on_fatal_error=lambda exc: (
                shared.set_vision_failed(exc),
                stop_event.set(),
            ),
        )

    control_thread = threading.Thread(target=control_worker, daemon=True)
    control_started = False

    try:
        from raspberry_pi.vision.pose_landmarker import run_pose_landmarker_on_camera

        comm.open()
        time.sleep(1.5)
        comm.send_stop()
        control_thread.start()
        control_started = True
        run_pose_landmarker_on_camera(
            runtime.camera_id,
            on_detected=on_detected,
            show_window=runtime.show_window,
        )
    except Exception as exc:  # pragma: no cover
        shared.set_vision_failed(exc)
    finally:
        stop_event.set()
        if control_started:
            control_thread.join(timeout=1.0)
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
    parser.add_argument("--baudrate", type=int, default=9600, help="UART baudrate")
    parser.add_argument("--camera-id", type=int, default=0, help="OpenCV camera id")
    parser.add_argument(
        "--show_window",
        action="store_true",
        help="Show OpenCV visualization window",
    )
    parser.add_argument(
        "--control-hz", type=float, default=10.0, help="Control loop frequency"
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
        show_window=args.show_window,
        control_hz=args.control_hz,
        detection_stale_s=args.detection_stale_s,
        status_interval_s=args.status_interval_s,
    )


if __name__ == "__main__":
    run(_parse_args())
