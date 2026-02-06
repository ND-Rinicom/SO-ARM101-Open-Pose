# SO-ARM101-Open-Pose
Controls the SO-ARM101 robot arm using arm movements captured by a camera with pose estimation.

## Overview
This project combines human pose detection with a 3D web-based robot arm visualization and control system. The system captures human arm movements via camera and uses inverse kinematics to control a virtual robot arm model.

## Prerequisites
- Python 3.8 or higher
- Node.js (for web server, if needed)
- MQTT broker (Mosquitto recommended)

## Setup Instructions
1) Install Python 3.11 (Fedora)
```bash
sudo dnf install -y python3.11 python3.11-devel
```
2) Create a venv with Python 3.11
```bash
rm -rf .venv
python3.11 -m venv .venv
source .venv/bin/activate
```
3) Install dependencies 
```bash
python -m pip install --upgrade pip
python -m pip install mediapipe==0.10.32 opencv-python paho-mqtt

curl -L -o pose_landmarker_lite.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_lite.task
```

## Project Structure
```
SO-ARM101-Open-Pose/
├── README.md              # This file
├── requirements.txt       # Python dependencies
├── Models/               # 3D models (.glb files)
├── Mqtt/                 # MQTT JavaScript libraries
└── Web-3D-rig-loader stuff/
    ├── 3Dmodels.js       # Main 3D rendering and IK logic
    ├── ik_solver_final.js # Inverse kinematics solver
    ├── ik_debug.js       # IK debugging visualization
    └── openPose.html     # Main web interface
```


{
  "method": "set_joint_angles",
  "timestamp": "2026-01-28T14:32:15Z",
  "params": {
    "units": "degrees",
    "mode": "follower",
    "joints": {
      "base_rotation": { "y": 44.8 },
      "shoulder_lift": { "x": 29.9 },
      "elbow_flex": { "x": -16.1 },
      "wrist_flex": { "x": 59.7 },
      "wrist_roll": { "y": 89.3 },
      "gripper": { "z": 49.8 }
    }
  }
}


## MQTT Message Format

### Set Joint Angles
Send joint angles directly to the robot arm: 

mosquitto_pub -h 192.168.1.107 -p 1883 -t 'watchman_robotarm/SO-ARM101-new' -m '{
  "method": "set_joint_angles",
  "timestamp": "2026-01-14T15:27:05Z",
  "params": {
    "units": "degrees",
    "joints": {
      "base": { "y": 0 },
      "shoulder_pan": { "y": 0 },
      "shoulder_lift": { "x": 0 },
      "elbow_flex": { "x": 0 },
      "wrist_flex": { "x": 0 },
      "wrist_roll": { "y": 0 },
      "gripper": { "z": 0 },
    }
  }
}'
```

mosquitto_pub -h 192.168.1.107 -p 1883 -t 'watchman_robotarm/SO-ARM101-new' -m '{
  "method": "set_joints_from_arm_pose",
  "timestamp": "2026-01-28T14:32:15Z",
  "params": {
    "joints":{
      "shoulder":{"x":0,"y":0,"z":0},
      "elbow":{"x":-0.2,"y":0.8,"z":0},
      "wrist":{"x":1,"y":0.8,"z":0},
      "hand":{"x":1.1,"y":0.7,"z":0}    
    }
  }
}'


## Debugging

### Check MQTT Connection
```bash
# Subscribe to all robot arm topics:
mosquitto_sub -h 192.168.1.107 -p 1883 -t 'watchman_robotarm/#' -v
```

### View Browser Console
Open browser developer tools (F12) to see:
- Model loading status
- IK solver output
- MQTT connection status
- Joint angle updates

## Troubleshooting

### Model Not Loading
- Ensure the model file exists in the `Models/` directory
- Check browser console for 404 errors
- Verify the model filename matches the URL parameter

### MQTT Not Connecting
- Verify MQTT broker is running: `sudo systemctl status mosquitto`
- Check WebSocket port (default: 9000) is accessible
- Ensure firewall allows connections on port 9000

### Python Dependencies Issues
```bash
# If pip install fails, try upgrading pip first:
pip install --upgrade pip
pip install -r requirements.txt
```

## License
See individual component licenses (OpenPose has its own license).

## Contributors
See [AUTHORS.md](AUTHORS.md) for contributors.
