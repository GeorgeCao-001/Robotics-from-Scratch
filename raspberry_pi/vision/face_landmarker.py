import argparse
import subprocess
import threading
import time

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import cv2
import numpy as np
import os

model_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "face_landmarker.task"
)


def to_pixel(x_norm: float, y_norm: float, w: int, h: int) -> tuple[int, int]:
    # keep to [0,1] to avoid occasional out-of-range artifacts
    x = min(max(x_norm, 0.0), 1.0)
    y = min(max(y_norm, 0.0), 1.0)
    return int(x * w), int(y * h)


def draw_face_landmarks(
    image_bgr: np.ndarray,
    face_landmarks_list,
    draw_points=False,
    draw_contours=True,
    point_radius=2,
    point_thickness=-1,
    contour_thickness=1,
):
    annotated = image_bgr.copy()
    h, w = annotated.shape[:2]

    for face_landmarks in face_landmarks_list:
        pts = [to_pixel(lm.x, lm.y, w, h) for lm in face_landmarks]
        num_landmarks = len(pts)

        if draw_contours and num_landmarks >= 468:
            # Face oval contour (468 points version)
            face_oval_indices = [
                10,
                338,
                297,
                332,
                284,
                251,
                389,
                356,
                454,
                323,
                361,
                288,
                397,
                365,
                379,
                378,
                400,
                377,
                152,
                148,
                176,
                149,
                150,
                136,
                172,
                58,
                132,
                93,
                234,
                127,
                162,
                21,
                54,
                103,
                67,
                109,
            ]
            if max(face_oval_indices) < num_landmarks:
                oval_pts = np.array([pts[i] for i in face_oval_indices], dtype=np.int32)
                cv2.polylines(
                    annotated, [oval_pts], True, (0, 255, 0), contour_thickness
                )

            # Left eye
            left_eye_indices = [
                33,
                7,
                163,
                144,
                145,
                153,
                154,
                155,
                133,
                173,
                157,
                158,
                159,
                160,
                161,
                246,
            ]
            if max(left_eye_indices) < num_landmarks:
                left_eye_pts = np.array(
                    [pts[i] for i in left_eye_indices], dtype=np.int32
                )
                cv2.polylines(
                    annotated, [left_eye_pts], True, (255, 0, 0), contour_thickness
                )

            # Right eye
            right_eye_indices = [
                362,
                382,
                381,
                380,
                374,
                373,
                390,
                249,
                263,
                466,
                388,
                387,
                386,
                385,
                384,
                398,
            ]
            if max(right_eye_indices) < num_landmarks:
                right_eye_pts = np.array(
                    [pts[i] for i in right_eye_indices], dtype=np.int32
                )
                cv2.polylines(
                    annotated, [right_eye_pts], True, (255, 0, 0), contour_thickness
                )

            # Lips - outer
            outer_lips_indices = [
                61,
                185,
                40,
                39,
                37,
                0,
                267,
                269,
                270,
                409,
                291,
                375,
                321,
                405,
                314,
                17,
                84,
                181,
                91,
                146,
            ]
            if max(outer_lips_indices) < num_landmarks:
                outer_lips_pts = np.array(
                    [pts[i] for i in outer_lips_indices], dtype=np.int32
                )
                cv2.polylines(
                    annotated, [outer_lips_pts], True, (0, 0, 255), contour_thickness
                )

            # Lips - inner
            inner_lips_indices = [
                78,
                191,
                80,
                81,
                82,
                13,
                312,
                311,
                310,
                415,
                308,
                324,
                318,
                402,
                317,
                14,
                87,
                178,
                88,
                95,
            ]
            if max(inner_lips_indices) < num_landmarks:
                inner_lips_pts = np.array(
                    [pts[i] for i in inner_lips_indices], dtype=np.int32
                )
                cv2.polylines(
                    annotated, [inner_lips_pts], True, (0, 0, 255), contour_thickness
                )

        if draw_points:
            for x, y in pts:
                cv2.circle(
                    annotated, (x, y), point_radius, (255, 255, 0), point_thickness
                )

    return annotated


def extract_face_info(face_landmarks, image_width: int, image_height: int) -> dict:
    x_coords = []
    y_coords = []

    for lm in face_landmarks:
        x_coords.append(lm.x * image_width)
        y_coords.append(lm.y * image_height)

    xmin, xmax = min(x_coords), max(x_coords)
    ymin, ymax = min(y_coords), max(y_coords)

    center_x = int((xmin + xmax) / 2)
    center_y = int((ymin + ymax) / 2)
    face_width = int(xmax - xmin)
    face_height = int(ymax - ymin)

    target_x_norm = min(max(center_x / image_width, 0.0), 1.0)
    target_y_norm = min(max(center_y / image_height, 0.0), 1.0)
    x_error_norm = target_x_norm * 2.0 - 1.0
    y_error_norm = target_y_norm * 2.0 - 1.0
    height_norm = min(max(face_height / image_height, 0.0), 1.0)
    width_norm = min(max(face_width / image_width, 0.0), 1.0)

    return {
        "target_x": center_x,
        "target_y": center_y,
        "height": face_height,
        "width": face_width,
        "target_x_norm": target_x_norm,
        "target_y_norm": target_y_norm,
        "x_error_norm": x_error_norm,
        "y_error_norm": y_error_norm,
        "height_norm": height_norm,
        "width_norm": width_norm,
        "confidence": 1.0,
    }


def select_largest_face(
    face_landmarks_list, image_width: int, image_height: int
) -> tuple:
    """
    Select the largest face from multiple detections.

    Args:
        face_landmarks_list: List of face landmarks
        image_width: Image width in pixels
        image_height: Image height in pixels

    Returns:
        tuple: (selected_face_landmarks, face_info) or (None, None) if no faces
    """
    if not face_landmarks_list:
        return None, None

    # Find largest face by width
    largest_face = None
    largest_info = None
    max_width = 0

    for face_landmarks in face_landmarks_list:
        info = extract_face_info(face_landmarks, image_width, image_height)
        if info["width"] > max_width:
            max_width = info["width"]
            largest_face = face_landmarks
            largest_info = info

    return largest_face, largest_info


def _print_face_info(info: dict) -> None:
    print(
        f"[FACE] x_err={info['x_error_norm']:+.3f} "
        f"y_err={info['y_error_norm']:+.3f} "
        f"w_norm={info['width_norm']:.3f} "
        f"h_norm={info['height_norm']:.3f}"
    )


def _face_options():
    base_options = python.BaseOptions(model_asset_path=model_path)
    return vision.FaceLandmarkerOptions(
        base_options=base_options,
        num_faces=1,
        min_face_detection_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO,
    )


def _process_face_frame(frame, landmarker, timestamp_ms, show_window):
    mp_image = mp.Image(
        image_format=mp.ImageFormat.SRGB,
        data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
    )
    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    if result.face_landmarks:
        h, w = frame.shape[:2]
        sel, info = select_largest_face(result.face_landmarks, w, h)
        if info:
            _print_face_info(info)
        annotated = draw_face_landmarks(frame, [sel] if sel else result.face_landmarks)
    else:
        annotated = frame
        info = None

    if show_window:
        cv2.imshow("Face Test", annotated)
        return cv2.waitKey(1) & 0xFF == ord("q"), info
    return False, info


def _run_face_opencv(args):
    cap = cv2.VideoCapture(args.camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.frame_height)
    if not cap.isOpened():
        print(f"Cannot open camera {args.camera_id}")
        return

    ts = 0
    with vision.FaceLandmarker.create_from_options(_face_options()) as lm:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            ts += 33
            should_quit, _info = _process_face_frame(frame, lm, ts, args.show_window)
            if should_quit:
                break
    cap.release()
    if args.show_window:
        cv2.destroyAllWindows()


def _run_face_rpicam(args):
    proc = subprocess.Popen(
        [
            "rpicam-vid", "--camera", str(args.camera_id),
            "--timeout", "0", "--nopreview", "--codec", "mjpeg",
            "--width", str(args.frame_width),
            "--height", str(args.frame_height),
            "--framerate", "15", "--verbose", "0", "-o", "-",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    if proc.stdout is None:
        raise RuntimeError("rpicam-vid stdout failed")

    def _drain():
        if proc.stderr:
            for _ in proc.stderr:
                pass

    threading.Thread(target=_drain, daemon=True).start()

    buf = bytearray()
    ts = 0
    with vision.FaceLandmarker.create_from_options(_face_options()) as lm:
        while True:
            if proc.poll() is not None:
                print(f"rpicam-vid exited {proc.returncode}")
                break
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                break
            buf.extend(chunk)
            while True:
                soi = buf.find(b"\xff\xd8")
                if soi < 0:
                    break
                if soi > 0:
                    del buf[:soi]
                eoi = buf.find(b"\xff\xd9")
                if eoi < 0:
                    break
                jpg = bytes(buf[: eoi + 2])
                del buf[: eoi + 2]
                frame = cv2.imdecode(
                    np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR
                )
                if frame is None:
                    continue
                ts += 33
                should_quit, _info = _process_face_frame(frame, lm, ts, args.show_window)
                if should_quit:
                    proc.terminate()
                    proc.wait()
                    if args.show_window:
                        cv2.destroyAllWindows()
                    return
    proc.terminate()
    proc.wait()
    if args.show_window:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Face landmarker test")
    parser.add_argument("--camera-backend", choices=["opencv", "rpicam"], default="opencv")
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--show-window", action="store_true")
    args = parser.parse_args()

    print("Face landmarker test. Press q in window to quit.")
    if args.camera_backend == "rpicam":
        _run_face_rpicam(args)
    else:
        _run_face_opencv(args)
