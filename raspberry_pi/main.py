from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass
from math import isfinite
from typing import Any, Callable

_MIN_TARGET_CONFIDENCE = 0.5

from raspberry_pi.hardware import GimbalConfig, GimbalHardware, SerialComm, SerialConfig
from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import GimbalOutput, VisionTarget


@dataclass(frozen=True)
class RuntimeConfig:
    port: str
    baudrate: int = 115200
    camera_id: int = 0
    show_window: bool = False
    frame_width: int = 640
    frame_height: int = 480
    camera_fps: int = 15
    num_poses: int = 1
    control_hz: float = 10.0
    detection_stale_s: float = 0.2
    status_interval_s: float = 0.0
    debug_vision: bool = False
    debug_control: bool = False
    debug_gimbal: bool = False


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

    if confidence < _MIN_TARGET_CONFIDENCE:
        return None

    return VisionTarget(
        x_error_norm=x_error_norm,
        y_error_norm=y_error_norm,
        height_norm=height_norm,
        width_norm=width_norm,
        confidence=confidence,
    )


def _dispatch_gimbal(
    gimbal: GimbalOutput,
    hw: GimbalHardware,
) -> None:
    try:
        hw.write(gimbal.pan_abs, gimbal.tilt_abs)
    except Exception as exc:
        print(f"[MAIN] gimbal write failed: {exc}")
        raise


def _run_control_loop(
    planner: Planner,
    comm: SerialComm,
    gimbal_hw: GimbalHardware,
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
    last_debug_t = last_t

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
        move_cmd, gimbal = planner.update(target, dt_s)
        if runtime.debug_control and now_t - last_debug_t >= 0.5:
            last_debug_t = now_t
            target_state = "yes" if target is not None else "no"
            x_err = "None" if target is None else f"{target.x_error_norm:.3f}"
            y_err = "None" if target is None else f"{target.y_error_norm:.3f}"
            print(
                "[CONTROL] "
                f"target={target_state} "
                f"x_err={x_err} "
                f"y_err={y_err} "
                f"move(v={move_cmd['v']:.3f},w={move_cmd['w']:.3f}) "
                f"gimbal(pan_abs={gimbal.pan_abs:.3f},"
                f"tilt_abs={gimbal.tilt_abs:.3f},"
                f"pan_delta={gimbal.pan_delta:.3f},"
                f"tilt_delta={gimbal.tilt_delta:.3f})"
            )
        try:
            comm.send_message(move_cmd)
        except Exception as exc:
            print(f"[MAIN] send failed: {exc}")
            on_fatal_error(exc)
            try:
                comm.send_stop()
            except Exception:
                pass
            return

        try:
            _dispatch_gimbal(gimbal, gimbal_hw)
        except Exception as exc:
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


def run(runtime: RuntimeConfig) -> None:
    planning_config = PlanningConfig()
    planner = Planner(planning_config)
    shared = SharedVisionState()
    comm = SerialComm(SerialConfig(port=runtime.port, baudrate=runtime.baudrate))
    gimbal_hw = GimbalHardware(
        GimbalConfig(
            pan_pin=planning_config.gimbal_pan_pin,
            tilt_pin=planning_config.gimbal_tilt_pin,
            initial_pan_angle=planning_config.pan_center,
            initial_tilt_angle=planning_config.tilt_center,
            debug=runtime.debug_gimbal,
        )
    )
    stop_event = threading.Event()

    def on_detected(pose_info: dict[str, Any]) -> None:
        if stop_event.is_set():
            raise RuntimeError("control loop stopped")
        target = _to_vision_target(pose_info)
        if target is None:
            if runtime.debug_vision:
                print(f"[MAIN] rejected pose_info keys={sorted(pose_info.keys())}")
            return
        if runtime.debug_vision:
            print(
                "[MAIN] target accepted "
                f"x_error={target.x_error_norm:.3f} "
                f"y_error={target.y_error_norm:.3f} "
                f"height={target.height_norm:.3f} "
                f"width={target.width_norm:.3f} "
                f"conf={target.confidence:.3f}"
            )
        shared.update(target, time.monotonic())

    def control_worker() -> None:
        _run_control_loop(
            planner=planner,
            comm=comm,
            gimbal_hw=gimbal_hw,
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
        from raspberry_pi.vision.pose_landmarker import (
            run_pose_landmarker_on_rpicam,
        )

        comm.open()
        time.sleep(1.5)
        comm.enter_rpi_auto_mode()
        time.sleep(0.2)
        comm.send_stop()
        time.sleep(0.5)
        control_thread.start()
        control_started = True

        vision_kwargs = {
            "on_detected": on_detected,
            "show_window": runtime.show_window,
            "frame_width": runtime.frame_width,
            "frame_height": runtime.frame_height,
            "camera_fps": runtime.camera_fps,
            "num_poses": runtime.num_poses,
            "debug_vision": runtime.debug_vision,
        }
        run_pose_landmarker_on_rpicam(
            runtime.camera_id,
            **vision_kwargs,
        )
    except Exception as exc:  # pragma: no cover
        print(f"[MAIN] vision runtime failed: {exc}")
        shared.set_vision_failed(exc)
    finally:
        stop_event.set()
        if control_started:
            control_thread.join(timeout=1.0)
        try:
            comm.send_stop()
        except Exception:
            pass
        try:
            comm.send_message({"cmd": "mode", "mode": "remote"})
        except Exception:
            pass
        try:
            gimbal_hw.cleanup()
        except Exception:
            pass
        comm.close()


def _parse_args() -> RuntimeConfig:
    parser = argparse.ArgumentParser(description="Pose follow main pipeline")
    parser.add_argument(
        "--port", required=True, help="Serial port, e.g. /dev/ttyACM0 (Linux) or /dev/cu.usbmodemxxxx (macOS)"
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="UART baudrate")
    parser.add_argument("--camera-id", type=int, default=0, help="rpicam camera ID")
    parser.add_argument(
        "--show_window",
        action="store_true",
        help="Show OpenCV visualization window",
    )
    parser.add_argument(
        "--frame-width", type=int, default=640, help="Camera frame width"
    )
    parser.add_argument(
        "--frame-height", type=int, default=480, help="Camera frame height"
    )
    parser.add_argument(
        "--camera-fps", type=int, default=15, help="Camera frame rate"
    )
    parser.add_argument(
        "--num-poses", type=int, default=1, help="Number of poses for MediaPipe"
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
    parser.add_argument(
        "--debug-vision",
        action="store_true",
        help="Print camera frame and pose detection diagnostics",
    )
    parser.add_argument(
        "--debug-control",
        action="store_true",
        help="Print control loop diagnostics",
    )
    parser.add_argument(
        "--debug-gimbal",
        action="store_true",
        help="Print GPIO gimbal PWM diagnostics",
    )
    args = parser.parse_args()
    return RuntimeConfig(
        port=args.port,
        baudrate=args.baudrate,
        camera_id=args.camera_id,
        show_window=args.show_window,
        frame_width=args.frame_width,
        frame_height=args.frame_height,
        camera_fps=args.camera_fps,
        num_poses=args.num_poses,
        control_hz=args.control_hz,
        detection_stale_s=args.detection_stale_s,
        status_interval_s=args.status_interval_s,
        debug_vision=args.debug_vision,
        debug_control=args.debug_control,
        debug_gimbal=args.debug_gimbal,
    )


if __name__ == "__main__":
    run(_parse_args())
