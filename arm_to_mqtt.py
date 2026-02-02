import time
import json
import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
import argparse
from datetime import datetime, timezone
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# ---------------- MQTT CONFIG ----------------
MQTT_HOST = "192.168.1.107"
MQTT_PORT = 1883
MQTT_TOPIC = "watchman_robotarm/SO-ARM101"

# ---------------- POSE CONFIG ----------------
MODEL_PATH = "pose_landmarker_heavy.task"

# ---------------- ARGUMENT PARSING ----------------
parser = argparse.ArgumentParser(description="Track arm pose and send to MQTT")
parser.add_argument(
    "--video",
    type=str,
    default="0",
    help="Video source: webcam index (0, 1, ...) or path to video file (default: 0)"
)
parser.add_argument(
    "--width",
    type=int,
    default=1280,
    help="Webcam resolution width (default: 1280)"
)
parser.add_argument(
    "--height",
    type=int,
    default=720,
    help="Webcam resolution height (default: 720)"
)
args = parser.parse_args()

# Determine video source
try:
    video_source = int(args.video)  # Try as webcam index
    source_name = f"Webcam {video_source}"
except ValueError:
    video_source = args.video  # Use as file path
    source_name = args.video

# Because webcam is mirrored, we track left arm for right arm control
LEFT_SHOULDER = 11
LEFT_ELBOW = 13
LEFT_WRIST = 15
LEFT_HAND = 19  # left index fingertip

# ---------------- MQTT SETUP ----------------
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()

# ---------------- SMOOTHING CONFIG ----------------
SMOOTHING_FACTOR = 0.5  # Lower = smoother but more lag (0.1-0.5 recommended)
DEAD_ZONE = 0.01  # Ignore changes smaller than this (in normalized coords)

# Store previous joint positions for smoothing
prev_joints = None

def smooth_landmark(prev_lm, curr_lm, alpha=SMOOTHING_FACTOR):
    """Apply exponential moving average to landmark coordinates."""
    if prev_lm is None:
        return {"x": curr_lm.x, "y": curr_lm.y, "z": curr_lm.z}
    
    # Check if change is significant enough (dead-zone filtering)
    dx = abs(curr_lm.x - prev_lm["x"])
    dy = abs(curr_lm.y - prev_lm["y"])
    
    if dx < DEAD_ZONE and dy < DEAD_ZONE:
        return prev_lm  # Keep previous position
    
    # Apply EMA smoothing: new_value = alpha * current + (1-alpha) * previous
    return {
        "x": alpha * curr_lm.x + (1 - alpha) * prev_lm["x"],
        "y": alpha * curr_lm.y + (1 - alpha) * prev_lm["y"],
        "z": alpha * curr_lm.z + (1 - alpha) * prev_lm["z"]
    }

# ---------------- MEDIAPIPE SETUP ----------------
base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,
    num_poses=1,
    min_pose_detection_confidence=0.7,  # Increased from 0.5
    min_pose_presence_confidence=0.7,   # Increased from 0.5
    min_tracking_confidence=0.7,        # Increased from 0.5
)

landmarker = vision.PoseLandmarker.create_from_options(options)

cap = cv2.VideoCapture(video_source)
if not cap.isOpened():
    raise RuntimeError(f"Could not open video source: {source_name}")

# Set resolution for webcam (not needed for video files)
if isinstance(video_source, int):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Using video source: {source_name} at {actual_width}x{actual_height}")
else:
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Using video source: {source_name} at {actual_width}x{actual_height}")

# Ensure the window is created with proper size
window_title = f"Left arm → MQTT (ESC to quit) - {source_name}"
cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

t0 = time.monotonic()

def joint_xyz(lm):
    """Return dict with x,y,z from smoothed landmark (z fixed to 0 for now)."""
    return {
        "x": float(lm["x"])*2,
        "y": -(float(lm["y"])*2),
        "z": 0.0
    }
def extend_hand_position(wrist, hand, extension_factor=1.7):
    """Extend hand position along wrist-to-hand direction."""
    # Calculate direction vector from wrist to hand
    dx = hand["x"] - wrist["x"]
    dy = hand["y"] - wrist["y"]
    dz = hand["z"] - wrist["z"]
    
    # Extend the hand position by multiplying the direction vector
    return {
        "x": wrist["x"] + dx * extension_factor,
        "y": wrist["y"] + dy * extension_factor,
        "z": wrist["z"] + dz * extension_factor,
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

            # Initialize or update smoothed joint positions
            if prev_joints is None:
                prev_joints = {
                    "shoulder": {"x": lm[LEFT_SHOULDER].x, "y": lm[LEFT_SHOULDER].y, "z": lm[LEFT_SHOULDER].z},
                    "elbow": {"x": lm[LEFT_ELBOW].x, "y": lm[LEFT_ELBOW].y, "z": lm[LEFT_ELBOW].z},
                    "wrist": {"x": lm[LEFT_WRIST].x, "y": lm[LEFT_WRIST].y, "z": lm[LEFT_WRIST].z},
                    "hand": {"x": lm[LEFT_HAND].x, "y": lm[LEFT_HAND].y, "z": lm[LEFT_HAND].z},
                }
            else:
                # Apply smoothing to each joint
                prev_joints["shoulder"] = smooth_landmark(prev_joints["shoulder"], lm[LEFT_SHOULDER])
                prev_joints["elbow"] = smooth_landmark(prev_joints["elbow"], lm[LEFT_ELBOW])
                prev_joints["wrist"] = smooth_landmark(prev_joints["wrist"], lm[LEFT_WRIST])
                prev_joints["hand"] = smooth_landmark(prev_joints["hand"], lm[LEFT_HAND])

            # Extend hand position to fingertips
            extended_hand = extend_hand_position(prev_joints["wrist"], prev_joints["hand"], extension_factor=2.0)
            
            payload = {
                "method": "set_joints_from_arm_pose",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "params": {
                    "joints": {
                        "shoulder": joint_xyz(prev_joints["shoulder"]),
                        "elbow": joint_xyz(prev_joints["elbow"]),
                        "wrist": joint_xyz(prev_joints["wrist"]),
                        "hand": joint_xyz(extended_hand),
                    }
                }
            }

            mqtt_client.publish(
                MQTT_TOPIC,
                json.dumps(payload),
                qos=0,
                retain=False
            )

            # Visual debug with colored lines and spheres
            h, w, _ = frame.shape
            
            # Get pixel coordinates for each joint (using smoothed positions and extended hand)
            shoulder_px = (int(prev_joints["shoulder"]["x"] * w), int(prev_joints["shoulder"]["y"] * h))
            elbow_px = (int(prev_joints["elbow"]["x"] * w), int(prev_joints["elbow"]["y"] * h))
            wrist_px = (int(prev_joints["wrist"]["x"] * w), int(prev_joints["wrist"]["y"] * h))
            hand_px = (int(extended_hand["x"] * w), int(extended_hand["y"] * h))
            
            # Draw lines with matching colors (BGR format in OpenCV)
            # Green: shoulder to elbow
            cv2.line(frame, shoulder_px, elbow_px, (0, 255, 0), 3)
            # Red: elbow to wrist
            cv2.line(frame, elbow_px, wrist_px, (0, 0, 255), 3)
            # Blue: wrist to hand
            cv2.line(frame, wrist_px, hand_px, (255, 0, 0), 3)
            
            # Draw colored circles at keypoints
            cv2.circle(frame, shoulder_px, 8, (255, 255, 255), -1)  # White for shoulder (base)
            cv2.circle(frame, elbow_px, 8, (0, 255, 0), -1)          # Green for elbow
            cv2.circle(frame, wrist_px, 8, (0, 0, 255), -1)          # Red for wrist
            cv2.circle(frame, hand_px, 8, (255, 0, 0), -1)           # Blue for hand

        cv2.imshow(window_title, frame)
        
        # For video files, use normal playback speed; for webcam, minimal delay
        wait_time = 30 if isinstance(video_source, str) else 1
        if (cv2.waitKey(wait_time) & 0xFF) == 27:
            break
except KeyboardInterrupt:
    pass
finally:
    cap.release()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    cv2.destroyAllWindows()
    landmarker.close()
