import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import cv2
import numpy as np
import os
import time

model_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pose_landmarker.task"
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
                # 绘制髋关节中心（使用 info 中的精确坐标）
                cv2.circle(annotated, (center_x, center_y), 8, (0, 255, 255), -1)
                cv2.circle(annotated, (center_x, center_y), 8, (0, 0, 0), 2)

            if draw_bbox:
                # 使用与实际输出一致的 height/width 绘制边界框
                # 以髋关节中心为基准
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
                label = f"H:{body_height} W:{body_width}"
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
            "target_x": int,   # Hip center x coordinate
            "target_y": int,   # Hip center y coordinate
            "height": int,     # Body height (nose to knee)
            "width": int,      # Body width (shoulder to shoulder)
            "target_x_norm": float, # Hip center x in [0,1]
            "target_y_norm": float, # Hip center y in [0,1]
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

    # 髋关节中心 (23: 左髋, 24: 右髋)
    hip_left_x = pose_landmarks[23].x * image_width
    hip_left_y = pose_landmarks[23].y * image_height
    hip_right_x = pose_landmarks[24].x * image_width
    hip_right_y = pose_landmarks[24].y * image_height

    center_x = int((hip_left_x + hip_right_x) / 2)
    center_y = int((hip_left_y + hip_right_y) / 2)

    # 高度: 鼻尖(0) 到 膝盖较低者 (25: 左膝, 26: 右膝)
    nose_y = pose_landmarks[0].y * image_height
    knee_left_y = pose_landmarks[25].y * image_height
    knee_right_y = pose_landmarks[26].y * image_height
    knee_y = max(knee_left_y, knee_right_y)  # 较低者 y 值更大
    body_height = int(abs(knee_y - nose_y))

    # 宽度: 左肩(11) 到 右肩(12)
    shoulder_left_x = pose_landmarks[11].x * image_width
    shoulder_right_x = pose_landmarks[12].x * image_width
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


def run_pose_landmarker_on_camera(camera_id: int = 0, on_detected=None):
    """
    Run pose landmarker on camera with optional callback for detected poses.

    Args:
        camera_id: Camera device ID (default: 0)
        on_detected: Optional callback function(pose_info_dict) called when pose detected
                    pose_info format: {
                        "target_x": int,
                        "target_y": int,
                        "height": int,
                        "width": int,
                        "target_x_norm": float,
                        "target_y_norm": float,
                        "x_error_norm": float,
                        "y_error_norm": float,
                        "height_norm": float,
                        "width_norm": float,
                        "confidence": float
                    }
    """
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        num_poses=3,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        running_mode=vision.RunningMode.VIDEO,
    )

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_id}")
        return

    # 使用单调时钟计算真实时间戳，保证单调递增且不受系统时间回拨影响
    start_monotonic = time.monotonic()
    last_timestamp_ms = 0
    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # 计算基于真实经过时间的毫秒时间戳（VIDEO 模式要求严格单调递增）
            timestamp_ms = int((time.monotonic() - start_monotonic) * 1000)
            # 确保严格单调递增（防止同一毫秒重复调用）
            timestamp_ms = max(timestamp_ms, last_timestamp_ms + 1)
            last_timestamp_ms = timestamp_ms

            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            if result.pose_landmarks:
                # Get image dimensions
                h, w = frame.shape[:2]

                # Select largest person and extract info
                selected_pose, pose_info = select_largest_person(
                    result.pose_landmarks, w, h
                )

                # Call callback if provided and pose detected
                if on_detected and pose_info:
                    on_detected(pose_info)

                # Draw landmarks (only for the selected pose if multiple)
                if selected_pose:
                    annotated = draw_pose_landmarks(frame, [selected_pose])
                else:
                    annotated = draw_pose_landmarks(frame, result.pose_landmarks)
            else:
                annotated = frame

            cv2.imshow("Pose Landmarker", annotated)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_pose_landmarker_on_camera(0)
