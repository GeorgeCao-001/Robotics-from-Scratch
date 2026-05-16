from __future__ import annotations

import argparse
import time

import cv2
import mediapipe as mp

from mediapipe.tasks.python import vision

from raspberry_pi.hardware.gimbal import GimbalConfig, GimbalHardware
from raspberry_pi.planning.config import PlanningConfig
from raspberry_pi.planning.gimbal_controller import GimbalController
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


def _run(args: argparse.Namespace) -> None:
    planning_config = PlanningConfig()
    controller = GimbalController(planning_config)
    gimbal_hw = None
    if not args.dry_run:
        gimbal_hw = GimbalHardware(
            GimbalConfig(
                pan_pin=planning_config.gimbal_pan_pin,
                tilt_pin=planning_config.gimbal_tilt_pin,
                initial_pan_angle=planning_config.pan_center,
                initial_tilt_angle=planning_config.tilt_center,
                debug=args.debug_gimbal,
            )
        )

    cap = cv2.VideoCapture(args.camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    cap.set(cv2.CAP_PROP_FPS, args.camera_fps)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera_id}")

    start_monotonic = time.monotonic()
    last_timestamp_ms = 0
    last_log_s = 0.0

    try:
        with vision.FaceLandmarker.create_from_options(_face_options()) as landmarker:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                timestamp_ms = int((time.monotonic() - start_monotonic) * 1000)
                timestamp_ms = max(timestamp_ms, last_timestamp_ms + 1)
                last_timestamp_ms = timestamp_ms

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                )
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                face_info = None
                selected_face = None
                if result.face_landmarks:
                    h, w = frame.shape[:2]
                    selected_face, face_info = select_largest_face(
                        result.face_landmarks, w, h
                    )

                if face_info is not None:
                    target = _to_vision_target(face_info)
                    gimbal = controller.compute(target)
                    if gimbal_hw is not None:
                        gimbal_hw.write(gimbal.pan_abs, gimbal.tilt_abs)

                    now_s = time.monotonic()
                    if args.debug and now_s - last_log_s >= 0.5:
                        last_log_s = now_s
                        mode = "dry-run" if args.dry_run else "gpio"
                        print(
                            "[FACE-GIMBAL] "
                            f"mode={mode} "
                            f"x_err={target.x_error_norm:.3f} "
                            f"y_err={target.y_error_norm:.3f} "
                            f"face_w={target.width_norm:.3f} "
                            f"pan_delta={gimbal.pan_delta:.3f} "
                            f"pan_abs={gimbal.pan_abs:.3f} "
                            f"tilt_delta={gimbal.tilt_delta:.3f} "
                            f"tilt_abs={gimbal.tilt_abs:.3f}"
                        )

                if args.show_window:
                    if selected_face is not None:
                        annotated = draw_face_landmarks(frame, [selected_face])
                    else:
                        annotated = frame
                    cv2.imshow("Face Gimbal Test", annotated)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    finally:
        cap.release()
        if args.show_window:
            cv2.destroyAllWindows()
        if gimbal_hw is not None:
            gimbal_hw.cleanup()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Face-based gimbal test")
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=15)
    parser.add_argument("--show-window", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-gimbal", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    _run(_parse_args())
