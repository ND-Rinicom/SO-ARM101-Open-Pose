import time
import json
import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
from datetime import datetime, timezone
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ---------------- MQTT CONFIG ----------------
MQTT_HOST = "192.168.1.107"
MQTT_PORT = 1883
MQTT_TOPIC = "watchman_robotarm/SO-ARM101"

# ---------------- POSE CONFIG ----------------
MODEL_PATH = "pose_landmarker_heavy.task"

# Because webcam is mirrored, we track left arm for right arm control
LEFT_SHOULDER = 11
LEFT_ELBOW = 13
LEFT_WRIST = 15
LEFT_HAND = 19  # left index fingertip

# ---------------- MQTT SETUP ----------------
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()

# ---------------- MEDIAPIPE SETUP ----------------
base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,
    num_poses=1,
    min_pose_detection_confidence=0.5,
    min_pose_presence_confidence=0.5,
    min_tracking_confidence=0.5,
)

landmarker = vision.PoseLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam (try changing VideoCapture(0) to 1).")

# Ensure the window is created with proper size
cv2.namedWindow("Left arm → MQTT (ESC to quit)", cv2.WINDOW_NORMAL)

t0 = time.monotonic()

def joint_xyz(lm):
    """Return dict with x,y,z (z fixed to 0 for now)."""
    return {
        "x": float(lm.x)*1.7,
        "y": -(float(lm.y)*1.7),
        "z": 0.0
    }

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)  # Flip horizontally
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.monotonic() - t0) * 1000)

        result = landmarker.detect_for_video(mp_image, timestamp_ms)

        if result.pose_landmarks:
            lm = result.pose_landmarks[0]

            payload = {
                "method": "set_joints_from_arm_pose",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "params": {
                    "joints": {
                        "shoulder": joint_xyz(lm[LEFT_SHOULDER]),
                        "elbow": joint_xyz(lm[LEFT_ELBOW]),
                        "wrist": joint_xyz(lm[LEFT_WRIST]),
                        "hand": joint_xyz(lm[LEFT_HAND]),
                    }
                }
            }

            mqtt_client.publish(
                MQTT_TOPIC,
                json.dumps(payload),
                qos=0,
                retain=False
            )

            # Optional visual debug
            h, w, _ = frame.shape
            for idx in [LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST, LEFT_HAND]:
                x = int(lm[idx].x * w)
                y = int(lm[idx].y * h)
                cv2.circle(frame, (x, y), 6, (0, 255, 0), -1)

        cv2.imshow("Left arm → MQTT (ESC to quit)", frame)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break
        
        # time.sleep(0.033)  # Limit to ~30 updates per second
except KeyboardInterrupt:
    pass
finally:
    cap.release()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    cv2.destroyAllWindows()
    landmarker.close()
