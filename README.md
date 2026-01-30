# SO-ARM101-Open-Pose
Controls the SO-ARM101 robot arm using arm movements captured by a camera with OpenPose pose estimation.

## Overview
This project combines OpenPose for human pose detection with a 3D web-based robot arm visualization and control system. The system captures human arm movements via camera, processes them through OpenPose, and uses inverse kinematics to control a virtual robot arm model.

## Prerequisites
- Python 3.8 or higher
- Node.js (for web server, if needed)
- MQTT broker (Mosquitto recommended)
- CMake (for building OpenPose)
- CUDA-capable GPU (recommended for OpenPose performance)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd SO-ARM101-Open-Pose
```

### 2. Set Up Python Environment
```bash
# Create virtual environment
python -m venv .venv

# Activate virtual environment
# On Linux/Mac:
source .venv/bin/activate
# On Windows:
# .venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Build OpenPose (Optional)
If you need to build OpenPose from source:
```bash
cd openpose
mkdir build && cd build
cmake ..
make -j`nproc`
```
Refer to [OpenPose installation guide](openpose/doc/installation/README.md) for detailed instructions.

### 4. Install MQTT Broker
```bash
# On Ubuntu/Debian:
sudo apt-get install mosquitto mosquitto-clients

# On Mac:
brew install mosquitto

# Start the broker:
sudo systemctl start mosquitto  # Linux
brew services start mosquitto   # Mac
```

### 5. Set Up Web Interface
The web interface uses Three.js loaded via CDN, so no additional installation is needed. Simply serve the files:
```bash
# Using Python's built-in HTTP server:
python -m http.server 8000

# Or use any other web server of your choice
```

## Running the Project

### 1. Start the MQTT Broker
Ensure your MQTT broker is running on the default port (1883).

### 2. Open the Web Interface
Navigate to:
```
http://localhost:8000/Web-3D-rig-loader%20stuff/openPose.html
```

Optional URL parameters:
- `model` - 3D model filename (default: SO-ARM101.glb)
- `xPos`, `yPos`, `zPos` - Model position (default: 0,0,0)
- `cameraZPos` - Camera Z position (default: 1)
- `wireframe` - Render mode: 0=ghost, 1=wireframe (default: 0)

Example:
```
http://localhost:8000/Web-3D-rig-loader%20stuff/openPose.html?model=SO-ARM101.glb&cameraZPos=7&wireframe=0
```

### 3. Run OpenPose
Follow OpenPose documentation to capture pose data and publish to MQTT.

## Project Structure
```
SO-ARM101-Open-Pose/
├── README.md              # This file
├── requirements.txt       # Python dependencies
├── so101.urdf            # Robot arm URDF model
├── Models/               # 3D models (.glb files)
├── Mqtt/                 # MQTT JavaScript libraries
├── openpose/             # OpenPose library
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
  "timestamp": "2026-01-28T14:32:15Z",
  "params": {
    "units": "degrees",
    "mode": "follower",
    "joints": {
      "Base_Rotation": { "y": 0 },
      "Shoulder_Lift": { "x": 0 },
      "Elbow_Flex": { "x": 0 },
      "Wrist_Flex": { "x": 0 },
      "Wrist_Roll": { "y": 0 },
      "Gripper": { "z": 0 }
    }
  }
}'
```

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


mosquitto_pub -h 192.168.1.107 -p 1883 -t 'watchman_robotarm/SO-ARM101-new' -m '{"method":"set_openpose_joints","timestamp":"2026-01-14T15:27:05Z","params":{"units":"degrees","joints":{"shoulder":{"x":0,"y":0,"z":0},"elbow":{"x":1,"y":1,"z":0},"wrist":{"x":-0.5,"y":1.5,"z":0},"hand":{"x":-0.7,"y":1.3,"z":0}}}}'
