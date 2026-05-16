import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import cv2
import numpy as np
import os
import select
import subprocess
import threading
import time

model_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pose_landmarker.task"
)


def _create_pose_landmarker_options(num_poses: int = 1):
    base_options = python.BaseOptions(model_asset_path=model_path)
    return vision.PoseLandmarkerOptions(
        base_options=base_options,
        num_poses=num_poses,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO,
    )


def to_pixel(x_norm: float, y_norm: float, w: int, h: int) -> tuple[int, int]:
    # keep to [0,1] to avoid occasional out-of-range artifacts
    x = min(max(x_norm, 0.0), 1.0)
    y = min(max(y_norm, 0.0), 1.0)
    return int(x * w), int(y * h)


POSE_CONNECTIONS = [
    # 躯干
    (11, 12),  # 左肩 - 右肩
    (11, 23),  # 左肩 - 左髋
    (12, 24),  # 右肩 - 右髋
    (23, 24),  # 左髋 - 右髋
    # 左臂
    (11, 13),  # 左肩 - 左肘
    (13, 15),  # 左肘 - 左腕
    # 右臂
    (12, 14),  # 右肩 - 右肘
    (14, 16),  # 右肘 - 右腕
    # 左腿
    (23, 25),  # 左髋 - 左膝
    (25, 27),  # 左膝 - 左踝
    # 右腿
    (24, 26),  # 右髋 - 右膝
    (26, 28),  # 右膝 - 右踝
    # 面部
    (0, 11),  # 鼻子 - 左肩
    (0, 12),  # 鼻子 - 右肩
]


def draw_pose_landmarks(
    image_bgr: np.ndarray,
    pose_landmarks_list,
    draw_points=True,
    draw_connections=True,
    draw_center=True,
    draw_bbox=True,
    point_radius=3,
    point_thickness=-1,
    line_thickness=2,
):
    annotated = image_bgr.copy()
    h, w = annotated.shape[:2]

    for pose_landmarks in pose_landmarks_list:
        pts = [to_pixel(lm.x, lm.y, w, h) for lm in pose_landmarks]
        num_landmarks = len(pts)

        if draw_connections and num_landmarks >= 33:
            for a, b in POSE_CONNECTIONS:
                if a < num_landmarks and b < num_landmarks:
                    cv2.line(annotated, pts[a], pts[b], (0, 255, 0), line_thickness)

        if draw_points:
            for x, y in pts:
                cv2.circle(
                    annotated, (x, y), point_radius, (255, 255, 0), point_thickness
                )

        # 使用 extract_pose_info 获取与实际输出一致的数据
        info = extract_pose_info(pose_landmarks, w, h)
        if info:
            center_x = info["target_x"]
            center_y = info["target_y"]
            body_height = info["height"]
            body_width = info["width"]

            if draw_center:
                # 绘制胸口中心（使用 info 中的精确坐标）
                cv2.circle(annotated, (center_x, center_y), 8, (0, 255, 255), -1)
                cv2.circle(annotated, (center_x, center_y), 8, (0, 0, 0), 2)

            if draw_bbox:
                # 使用与实际输出一致的 height/width 绘制边界框
                # 以胸口中心为基准
                x1 = center_x - body_width // 2
                y1 = center_y - body_height // 2
                x2 = x1 + body_width
                y2 = y1 + body_height

                # 确保在画面内
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)

                # 绘制边界框
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)

                # 绘制与实际输出一致的尺寸标签
                label = f"C:{body_height} W:{body_width}"
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                )

    return annotated


def extract_pose_info(pose_landmarks, image_width: int, image_height: int) -> dict:
    """
    Extract pose detection info from landmarks.

    Args:
        pose_landmarks: MediaPipe pose landmarks object (33 points)
        image_width: Image width in pixels
        image_height: Image height in pixels

    Returns:
        dict: {
            "target_x": int,   # Chest center x coordinate (mid-shoulder)
            "target_y": int,   # Chest center y coordinate (mid-shoulder)
            "height": int,     # Body height (nose to knee)
            "width": int,      # Body width (shoulder to shoulder)
            "target_x_norm": float, # Chest center x in [0,1]
            "target_y_norm": float, # Chest center y in [0,1]
            "x_error_norm": float,  # Horizontal error in [-1,1]
            "y_error_norm": float,  # Vertical error in [-1,1]
            "height_norm": float,   # Body height normalized by image height
            "width_norm": float,    # Body width normalized by image width
            "confidence": float      # Detection confidence
        }
    """
    num_landmarks = len(pose_landmarks)
    if num_landmarks < 33:
        return {}

    # 胸口中心 (11: 左肩, 12: 右肩 的中点)
    shoulder_left_x = pose_landmarks[11].x * image_width
    shoulder_left_y = pose_landmarks[11].y * image_height
    shoulder_right_x = pose_landmarks[12].x * image_width
    shoulder_right_y = pose_landmarks[12].y * image_height

    center_x = int((shoulder_left_x + shoulder_right_x) / 2)
    center_y = int((shoulder_left_y + shoulder_right_y) / 2)

    # 高度: 鼻尖(0) 到 膝盖较低者 (25: 左膝, 26: 右膝)
    nose_y = pose_landmarks[0].y * image_height
    knee_left_y = pose_landmarks[25].y * image_height
    knee_right_y = pose_landmarks[26].y * image_height
    knee_y = max(knee_left_y, knee_right_y)  # 较低者 y 值更大
    body_height = int(abs(knee_y - nose_y))

    # 宽度: 左肩(11) 到 右肩(12)
    body_width = int(abs(shoulder_right_x - shoulder_left_x))

    # 置信度: 使用双肩(11, 12)和双髋(23, 24)的 visibility 平均值
    key_points = [11, 12, 23, 24]
    confidence = sum(pose_landmarks[i].visibility for i in key_points) / len(key_points)

    # 归一化信息（供 planning 使用，降低对分辨率变化的敏感性）
    target_x_norm = min(max(center_x / image_width, 0.0), 1.0)
    target_y_norm = min(max(center_y / image_height, 0.0), 1.0)
    x_error_norm = target_x_norm * 2.0 - 1.0
    y_error_norm = target_y_norm * 2.0 - 1.0
    height_norm = min(max(body_height / image_height, 0.0), 1.0)
    width_norm = min(max(body_width / image_width, 0.0), 1.0)

    return {
        "target_x": center_x,
        "target_y": center_y,
        "height": body_height,
        "width": body_width,
        "target_x_norm": target_x_norm,
        "target_y_norm": target_y_norm,
        "x_error_norm": x_error_norm,
        "y_error_norm": y_error_norm,
        "height_norm": height_norm,
        "width_norm": width_norm,
        "confidence": confidence,
    }


def select_largest_person(
    pose_landmarks_list, image_width: int, image_height: int
) -> tuple:
    """
    Select the largest person from multiple detections.

    Args:
        pose_landmarks_list: List of pose landmarks
        image_width: Image width in pixels
        image_height: Image height in pixels

    Returns:
        tuple: (selected_pose_landmarks, pose_info) or (None, None) if no poses
    """
    if not pose_landmarks_list:
        return None, None

    # Find largest person by height
    largest_pose = None
    largest_info = None
    max_height = 0

    for pose_landmarks in pose_landmarks_list:
        info = extract_pose_info(pose_landmarks, image_width, image_height)
        if info and info.get("height", 0) > max_height:
            max_height = info["height"]
            largest_pose = pose_landmarks
            largest_info = info

    return largest_pose, largest_info


def _process_pose_frame(
    frame_bgr,
    landmarker,
    start_monotonic,
    last_timestamp_ms,
    on_detected,
    show_window,
    debug_vision=False,
    debug_state=None,
):
    rgb_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    timestamp_ms = int((time.monotonic() - start_monotonic) * 1000)
    timestamp_ms = max(timestamp_ms, last_timestamp_ms + 1)
    last_timestamp_ms = timestamp_ms

    result = landmarker.detect_for_video(mp_image, timestamp_ms)

    pose_info = None
    if result.pose_landmarks:
        h, w = frame_bgr.shape[:2]
        selected_pose, pose_info = select_largest_person(
            result.pose_landmarks, w, h
        )
        if on_detected and pose_info:
            on_detected(pose_info)
        if selected_pose:
            annotated = draw_pose_landmarks(frame_bgr, [selected_pose])
        else:
            annotated = draw_pose_landmarks(frame_bgr, result.pose_landmarks)
    else:
        annotated = frame_bgr

    if debug_vision and debug_state is not None:
        now_s = time.monotonic()
        debug_state["frames"] += 1
        if pose_info:
            debug_state["detections"] += 1
            if now_s - debug_state["last_pose_log_s"] >= 0.5:
                debug_state["last_pose_log_s"] = now_s
                print(
                    "[VISION] pose "
                    f"x_error={pose_info['x_error_norm']:.3f} "
                    f"y_error={pose_info['y_error_norm']:.3f} "
                    f"height={pose_info['height_norm']:.3f} "
                    f"width={pose_info['width_norm']:.3f} "
                    f"conf={pose_info['confidence']:.3f}"
                )

        if now_s - debug_state["last_stats_log_s"] >= 1.0:
            elapsed_s = max(1e-6, now_s - debug_state["stats_start_s"])
            fps = debug_state["frames"] / elapsed_s
            print(
                "[VISION] stats "
                f"frames={debug_state['frames']} "
                f"fps={fps:.1f} "
                f"detections={debug_state['detections']}"
            )
            debug_state["frames"] = 0
            debug_state["detections"] = 0
            debug_state["stats_start_s"] = now_s
            debug_state["last_stats_log_s"] = now_s

    should_quit = False
    if show_window:
        cv2.imshow("Pose Landmarker", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            should_quit = True

    return last_timestamp_ms, annotated, should_quit


def run_pose_landmarker_on_camera(
    camera_id: int = 0,
    on_detected=None,
    show_window: bool = False,
    frame_width: int = 640,
    frame_height: int = 480,
    camera_fps: int = 15,
    num_poses: int = 1,
    debug_vision: bool = False,
):
    options = _create_pose_landmarker_options(num_poses=num_poses)

    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, camera_fps)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_id}")
        return
    if debug_vision:
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(
            "[VISION] camera opened "
            f"backend=opencv id={camera_id} "
            f"width={actual_width:.0f} height={actual_height:.0f} fps={actual_fps:.1f}"
        )

    start_monotonic = time.monotonic()
    last_timestamp_ms = 0
    debug_state = {
        "frames": 0,
        "detections": 0,
        "stats_start_s": start_monotonic,
        "last_stats_log_s": start_monotonic,
        "last_pose_log_s": 0.0,
    }
    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            last_timestamp_ms, _annotated, should_quit = _process_pose_frame(
                frame, landmarker, start_monotonic, last_timestamp_ms,
                on_detected, show_window, debug_vision, debug_state,
            )
            if should_quit:
                break

    cap.release()
    if show_window:
        cv2.destroyAllWindows()


def run_pose_landmarker_on_rpicam(
    camera_id: int = 0,
    on_detected=None,
    show_window: bool = False,
    frame_width: int = 640,
    frame_height: int = 480,
    camera_fps: int = 15,
    num_poses: int = 1,
    debug_vision: bool = False,
):
    options = _create_pose_landmarker_options(num_poses=num_poses)

    cmd = [
        "rpicam-vid",
        "--camera", str(camera_id),
        "--timeout", "0",
        "--nopreview",
        "--codec", "mjpeg",
        "--width", str(frame_width),
        "--height", str(frame_height),
        "--framerate", str(camera_fps),
        "--verbose", "0",
        "-o", "-",
    ]
    if debug_vision:
        print(
            "[VISION] camera opening "
            f"backend=rpicam id={camera_id} "
            f"width={frame_width} height={frame_height} fps={camera_fps}"
        )
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    if proc.stdout is None:
        raise RuntimeError("failed to open rpicam-vid stdout")
    stdout_fd = proc.stdout.fileno()

    stderr_lines = []

    def drain_stderr():
        if proc.stderr is None:
            return
        for line in proc.stderr:
            stderr_lines.append(line.decode(errors="replace").rstrip())
            del stderr_lines[:-20]

    threading.Thread(target=drain_stderr, daemon=True).start()

    buf = bytearray()
    start_monotonic = time.monotonic()
    last_timestamp_ms = 0
    last_frame_time = start_monotonic
    debug_state = {
        "frames": 0,
        "detections": 0,
        "stats_start_s": start_monotonic,
        "last_stats_log_s": start_monotonic,
        "last_pose_log_s": 0.0,
    }
    stream_debug = {
        "bytes": 0,
        "chunks": 0,
        "soi": 0,
        "eoi": 0,
        "decoded": 0,
        "decode_failed": 0,
        "last_log_s": start_monotonic,
        "first_frame_logged": False,
    }
    frame_timeout_s = max(5.0, 3.0 / max(camera_fps, 1))
    max_buffer_bytes = max(frame_width * frame_height * 4, 10 * 1024 * 1024)

    try:
        with vision.PoseLandmarker.create_from_options(options) as landmarker:
            while True:
                if proc.poll() is not None:
                    details = "\n".join(stderr_lines[-10:])
                    raise RuntimeError(
                        f"rpicam-vid exited with code {proc.returncode}: {details}"
                    )

                readable, _, _ = select.select([proc.stdout], [], [], 0.2)
                if not readable:
                    if debug_vision:
                        now_s = time.monotonic()
                        if now_s - stream_debug["last_log_s"] >= 1.0:
                            print(
                                "[VISION] rpicam stream "
                                f"bytes={stream_debug['bytes']} "
                                f"chunks={stream_debug['chunks']} "
                                f"buffer={len(buf)} "
                                f"soi={stream_debug['soi']} "
                                f"eoi={stream_debug['eoi']} "
                                f"decoded={stream_debug['decoded']} "
                                f"decode_failed={stream_debug['decode_failed']}"
                            )
                            stream_debug.update(
                                {
                                    "bytes": 0,
                                    "chunks": 0,
                                    "soi": 0,
                                    "eoi": 0,
                                    "decoded": 0,
                                    "decode_failed": 0,
                                    "last_log_s": now_s,
                                }
                            )
                    if time.monotonic() - last_frame_time > frame_timeout_s:
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
                if debug_vision:
                    stream_debug["bytes"] += len(chunk)
                    stream_debug["chunks"] += 1
                    stream_debug["soi"] += chunk.count(b"\xff\xd8")
                    stream_debug["eoi"] += chunk.count(b"\xff\xd9")
                buf.extend(chunk)
                if len(buf) > max_buffer_bytes:
                    details = "\n".join(stderr_lines[-10:])
                    raise RuntimeError(
                        "rpicam-vid MJPEG buffer exceeded limit; "
                        "check codec/output format"
                        + (f": {details}" if details else "")
                    )

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
                        if debug_vision:
                            stream_debug["decode_failed"] += 1
                        continue
                    if debug_vision:
                        stream_debug["decoded"] += 1
                        if not stream_debug["first_frame_logged"]:
                            stream_debug["first_frame_logged"] = True
                            h, w = frame.shape[:2]
                            print(
                                "[VISION] rpicam first frame decoded "
                                f"width={w} height={h}"
                            )
                    last_frame_time = time.monotonic()

                    if debug_vision:
                        now_s = time.monotonic()
                        if now_s - stream_debug["last_log_s"] >= 1.0:
                            print(
                                "[VISION] rpicam stream "
                                f"bytes={stream_debug['bytes']} "
                                f"chunks={stream_debug['chunks']} "
                                f"buffer={len(buf)} "
                                f"soi={stream_debug['soi']} "
                                f"eoi={stream_debug['eoi']} "
                                f"decoded={stream_debug['decoded']} "
                                f"decode_failed={stream_debug['decode_failed']}"
                            )
                            stream_debug.update(
                                {
                                    "bytes": 0,
                                    "chunks": 0,
                                    "soi": 0,
                                    "eoi": 0,
                                    "decoded": 0,
                                    "decode_failed": 0,
                                    "last_log_s": now_s,
                                }
                            )

                    last_timestamp_ms, _annotated, should_quit = (
                        _process_pose_frame(
                            frame,
                            landmarker,
                            start_monotonic,
                            last_timestamp_ms,
                            on_detected,
                            show_window,
                            debug_vision,
                            debug_state,
                        )
                    )
                    if should_quit:
                        return
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        if show_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    run_pose_landmarker_on_camera(0)
