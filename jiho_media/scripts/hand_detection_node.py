#!/usr/bin/env python3
"""Hand detection node — MediaPipe HandLandmarker → ROS 2 topics."""

import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from jiho_media.msg import HandLandmarks, Landmark


_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
)
_MODEL_CACHE = Path.home() / ".cache" / "jiho_media" / "hand_landmarker.task"

# MediaPipe 21-point hand skeleton connections
_HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),           # index
    (0, 9), (9, 10), (10, 11), (11, 12),      # middle
    (0, 13), (13, 14), (14, 15), (15, 16),    # ring
    (0, 17), (17, 18), (18, 19), (19, 20),    # pinky
    (5, 9), (9, 13), (13, 17),                # palm
]


def _open_camera(preferred_index: int) -> tuple[cv2.VideoCapture, int]:
    """Try preferred_index first; fall back to scanning /dev/video* on failure."""
    def _try(idx: int):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened() and cap.read()[0]:
            return cap
        cap.release()
        return None

    cap = _try(preferred_index)
    if cap:
        return cap, preferred_index
    for idx in range(8):
        if idx == preferred_index:
            continue
        cap = _try(idx)
        if cap:
            return cap, idx
    raise RuntimeError("No accessible camera found (checked indices 0-7)")


def _ensure_model(custom_path: str) -> str:
    if custom_path:
        return custom_path
    if not _MODEL_CACHE.exists():
        _MODEL_CACHE.parent.mkdir(parents=True, exist_ok=True)
        print(f"[jiho_media] Downloading model → {_MODEL_CACHE}")
        urllib.request.urlretrieve(_MODEL_URL, _MODEL_CACHE)
        print("[jiho_media] Download complete.")
    return str(_MODEL_CACHE)


class HandDetectionNode(Node):

    def __init__(self):
        super().__init__("hand_detection_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("model_path", "")
        self.declare_parameter("max_num_hands", 2)
        self.declare_parameter("min_detection_confidence", 0.7)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("publish_rate", 30.0)

        camera_idx = self.get_parameter("camera_index").value
        model_path = _ensure_model(self.get_parameter("model_path").value)
        max_hands = self.get_parameter("max_num_hands").value
        min_det = self.get_parameter("min_detection_confidence").value
        min_track = self.get_parameter("min_tracking_confidence").value
        rate_hz = self.get_parameter("publish_rate").value

        # MediaPipe HandLandmarker — VIDEO mode (synchronous, timestamp-driven)
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=min_det,
            min_tracking_confidence=min_track,
        )
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
        self._frame_ts_ms: int = 0

        self._cap, actual_idx = _open_camera(camera_idx)

        self._pub_lm = self.create_publisher(HandLandmarks, "hand_landmarks", 10)
        self._pub_img = self.create_publisher(CompressedImage, "hand_image/compressed", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"HandDetectionNode started — camera=/dev/video{actual_idx}, rate={rate_hz:.0f} Hz"
        )

    # ── main loop ────────────────────────────────────────────────────────────

    def _tick(self):
        ok, frame_bgr = self._cap.read()
        if not ok:
            self.get_logger().warn("Failed to capture frame", throttle_duration_sec=2.0)
            return

        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        detection = self._landmarker.detect_for_video(mp_img, self._frame_ts_ms)
        self._frame_ts_ms += 33  # 33 ms ≈ 30 fps step

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id="camera")

        lm_msg = HandLandmarks()
        lm_msg.header = header
        lm_msg.num_hands = len(detection.hand_landmarks)

        annotated = frame_bgr.copy()

        for i, hand_lms in enumerate(detection.hand_landmarks):
            # Handedness
            if detection.handedness and i < len(detection.handedness):
                side = detection.handedness[i][0].category_name
            else:
                side = "Unknown"
            lm_msg.handedness.append(side)

            # Landmark data (normalized)
            for lm in hand_lms:
                lm_msg.landmarks.append(Landmark(x=lm.x, y=lm.y, z=lm.z))

            # Annotate frame
            _SKIP = set(range(13, 21))  # 약지(13~16), 소지(17~20)
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand_lms]
            for a, b in _HAND_CONNECTIONS:
                if a in _SKIP or b in _SKIP:
                    continue
                cv2.line(annotated, pts[a], pts[b], (200, 200, 200), 2)
            for idx, pt in enumerate(pts):
                if idx in _SKIP:
                    continue
                c = (0, 0, 255) if idx == 9 else (0, 255, 0)
                cv2.circle(annotated, pt, 5, c, -1)
            cv2.putText(
                annotated, side, pts[0],
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2,
            )

        self._pub_lm.publish(lm_msg)
        ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            img_msg = CompressedImage(header=header, format="bgr8; jpeg compressed")
            img_msg.data = buf.tobytes()
            self._pub_img.publish(img_msg)

    # ── cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._cap.release()
        self._landmarker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
