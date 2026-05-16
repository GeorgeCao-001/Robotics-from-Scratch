from __future__ import annotations

import argparse
import os
import select
import subprocess
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import vision

from raspberry_pi.hardware import GimbalConfig, GimbalHardware, SerialComm, SerialConfig
from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.planner import Planner
from raspberry_pi.planning.types import VisionTarget
from raspberry_pi.vision.face_landmarker import (
    _face_options,
    draw_face_landmarks,
    select_largest_face,
)


def _to_vision_target(face_info: dict) -> VisionTarget:
    return VisionTarget(
        x_error_norm=float(face_info["x_error_norm"]),
        y_error_norm=float(face_info["y_error_norm"]),
        height_norm=float(face_info["height_norm"]),
        width_norm=float(face_info["width_norm"]),
        confidence=float(face_info.get("confidence", 1.0)),
    )


def _start_rpicam(args: argparse.Namespace) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "rpicam-vid",
            "--camera",
            str(args.camera_id),
            "--timeout",
            "0",
            "--nopreview",
            "--codec",
            "mjpeg",
            "--width",
            str(args.frame_width),
            "--height",
            str(args.frame_height),
            "--framerate",
            str(args.camera_fps),
            "--verbose",
            "0",
            "-o",
            "-",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )


def _drain_stderr(proc: subprocess.Popen, lines: list[str]) -> None:
    if proc.stderr is None:
        return
    for line in proc.stderr:
        lines.append(line.decode(errors="replace").rstrip())
        del lines[:-20]


def _handle_frame(
    frame,
    landmarker,
    timestamp_ms: int,
    planner: Planner,
    comm: SerialComm,
    gimbal_hw: GimbalHardware,
    last_control_s: float,
    show_window: bool,
    debug: bool,
    last_debug_s: float,
) -> tuple[float, float, bool]:
    mp_image = mp.Image(
        image_format=mp.ImageFormat.SRGB,
        data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
    )
    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    face_info = None
    selected_face = None
    if result.face_landmarks:
        h, w = frame.shape[:2]
        selected_face, face_info = select_largest_face(result.face_landmarks, w, h)

    target = _to_vision_target(face_info) if face_info is not None else None
    now_s = time.monotonic()
    dt_s = max(0.0, now_s - last_control_s)
    move_cmd, gimbal = planner.update(target, dt_s)

    comm.send_message(move_cmd)
    gimbal_hw.write(gimbal.pan_abs, gimbal.tilt_abs)

    if debug and now_s - last_debug_s >= 0.5:
        last_debug_s = now_s
        if target is None:
            print(
                "[FACE-FOLLOW] target=no "
                f"move(v={move_cmd['v']:.3f},w={move_cmd['w']:.3f}) "
                f"gimbal(pan_abs={gimbal.pan_abs:.3f},tilt_abs={gimbal.tilt_abs:.3f})"
            )
        else:
            print(
                "[FACE-FOLLOW] target=yes "
                f"x_err={target.x_error_norm:.3f} "
                f"y_err={target.y_error_norm:.3f} "
                f"face_w={target.width_norm:.3f} "
                f"move(v={move_cmd['v']:.3f},w={move_cmd['w']:.3f}) "
                f"gimbal(pan_abs={gimbal.pan_abs:.3f},tilt_abs={gimbal.tilt_abs:.3f})"
            )

    should_quit = False
    if show_window:
        if selected_face is not None:
            annotated = draw_face_landmarks(frame, [selected_face])
        else:
            annotated = frame
        cv2.imshow("Face Follow Test", annotated)
        should_quit = cv2.waitKey(1) & 0xFF == ord("q")

    return now_s, last_debug_s, should_quit


def run(args: argparse.Namespace) -> None:
    planning_config = PlanningConfig()
    planner = Planner(planning_config)
    comm = SerialComm(SerialConfig(port=args.port, baudrate=args.baudrate))
    gimbal_hw = GimbalHardware(
        GimbalConfig(
            pan_pin=planning_config.gimbal_pan_pin,
            tilt_pin=planning_config.gimbal_tilt_pin,
            initial_pan_angle=planning_config.pan_center,
            initial_tilt_angle=planning_config.tilt_center,
            debug=args.debug_gimbal,
        )
    )

    proc = _start_rpicam(args)
    if proc.stdout is None:
        raise RuntimeError("failed to open rpicam-vid stdout")
    stderr_lines: list[str] = []
    threading.Thread(target=_drain_stderr, args=(proc, stderr_lines), daemon=True).start()

    buf = bytearray()
    stdout_fd = proc.stdout.fileno()
    start_s = time.monotonic()
    last_control_s = start_s
    last_frame_s = start_s
    last_debug_s = 0.0
    last_timestamp_ms = 0
    frame_timeout_s = max(5.0, 3.0 / max(args.camera_fps, 1))
    max_buffer_bytes = max(args.frame_width * args.frame_height * 4, 10 * 1024 * 1024)

    try:
        comm.open()
        time.sleep(1.5)
        comm.send_stop()
        gimbal_hw.setup()

        with vision.FaceLandmarker.create_from_options(_face_options()) as landmarker:
            while True:
                if proc.poll() is not None:
                    details = "\n".join(stderr_lines[-10:])
                    raise RuntimeError(
                        f"rpicam-vid exited with code {proc.returncode}: {details}"
                    )

                readable, _, _ = select.select([proc.stdout], [], [], 0.2)
                if not readable:
                    if time.monotonic() - last_frame_s > frame_timeout_s:
                        details = "\n".join(stderr_lines[-10:])
                        raise RuntimeError(
                            "timed out waiting for rpicam-vid frame"
                            + (f": {details}" if details else "")
                        )
                    continue

                chunk = os.read(stdout_fd, 4096)
                if not chunk:
                    details = "\n".join(stderr_lines[-10:])
                    raise RuntimeError(
                        "rpicam-vid ended before a complete frame was received"
                        + (f": {details}" if details else "")
                    )
                buf.extend(chunk)
                if len(buf) > max_buffer_bytes:
                    raise RuntimeError("rpicam-vid MJPEG buffer exceeded limit")

                while True:
                    soi = buf.find(b"\xff\xd8")
                    if soi < 0:
                        break
                    if soi > 0:
                        del buf[:soi]
                    eoi = buf.find(b"\xff\xd9")
                    if eoi < 0:
                        break

                    jpeg_bytes = bytes(buf[: eoi + 2])
                    del buf[: eoi + 2]

                    frame = cv2.imdecode(
                        np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is None:
                        continue

                    last_frame_s = time.monotonic()
                    timestamp_ms = int((last_frame_s - start_s) * 1000)
                    timestamp_ms = max(timestamp_ms, last_timestamp_ms + 1)
                    last_timestamp_ms = timestamp_ms

                    last_control_s, last_debug_s, should_quit = _handle_frame(
                        frame,
                        landmarker,
                        timestamp_ms,
                        planner,
                        comm,
                        gimbal_hw,
                        last_control_s,
                        args.show_window,
                        args.debug,
                        last_debug_s,
                    )
                    if should_quit:
                        return
    finally:
        try:
            comm.send_stop()
        except Exception:
            pass
        try:
            gimbal_hw.cleanup()
        except Exception:
            pass
        comm.close()
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        if args.show_window:
            cv2.destroyAllWindows()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Face-based follow test using rpicam")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-gimbal", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    run(_parse_args())
