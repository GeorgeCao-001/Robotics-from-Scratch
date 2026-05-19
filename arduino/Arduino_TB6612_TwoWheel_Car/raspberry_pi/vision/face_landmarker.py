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
    """
    Extract face detection info from landmarks.

    Args:
        face_landmarks: MediaPipe face landmarks object
        image_width: Image width in pixels
        image_height: Image height in pixels

    Returns:
        dict: {
            "target_x": int,  # Face center x coordinate
            "target_y": int,  # Face center y coordinate
            "width": int      # Face width in pixels
        }
    """
    # Get all landmark coordinates
    x_coords = []
    y_coords = []

    for lm in face_landmarks:
        x_coords.append(lm.x * image_width)
        y_coords.append(lm.y * image_height)

    # Calculate bounding box
    xmin, xmax = min(x_coords), max(x_coords)
    ymin, ymax = min(y_coords), max(y_coords)

    # Calculate center and width
    center_x = int((xmin + xmax) / 2)
    center_y = int((ymin + ymax) / 2)
    face_width = int(xmax - xmin)

    return {"target_x": center_x, "target_y": center_y, "width": face_width}


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


def run_face_landmarker_on_camera(camera_id: int = 0, on_detected=None):
    """
    Run face landmarker on camera with optional callback for detected faces.

    Args:
        camera_id: Camera device ID (default: 0)
        on_detected: Optional callback function(face_info_dict) called when face detected
                    face_info format: {"target_x": int, "target_y": int, "width": int}
    """
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        num_faces=3,
        min_face_detection_confidence=0.5,
        min_face_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        output_face_blendshapes=False,
        running_mode=vision.RunningMode.VIDEO,
    )

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_id}")
        return

    timestamp_ms = 0
    with vision.FaceLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            timestamp_ms += 33  # ~30 FPS
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            if result.face_landmarks:
                # Get image dimensions
                h, w = frame.shape[:2]

                # Select largest face and extract info
                selected_face, face_info = select_largest_face(
                    result.face_landmarks, w, h
                )

                # Call callback if provided and face detected
                if on_detected and face_info:
                    on_detected(face_info)

                # Draw landmarks (only for the selected face if multiple)
                if selected_face:
                    annotated = draw_face_landmarks(frame, [selected_face])
                else:
                    annotated = draw_face_landmarks(frame, result.face_landmarks)
            else:
                annotated = frame

            cv2.imshow("Face Landmarker", annotated)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_face_landmarker_on_camera(0)
