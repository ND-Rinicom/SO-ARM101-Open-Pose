# SO-ARM101-Open-Pose
Controls the SO-ARM101 using arm movments captured by a camera using control net and open Pose


# set up

python -m venv .venv
source .venv/bin/activate

pip install pin yourdfpy numpy


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


## DUBUGING 

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


mosquitto_pub -h 192.168.1.107 -p 1883 -t 'watchman_robotarm/SO-ARM101-new' -m '{"method":"set_openpose_joints","timestamp":"2026-01-14T15:27:05Z","params":{"units":"degrees","joints":{"shoulder":{"x":0,"y":0,"z":0},"elbow":{"x":1,"y":1,"z":0},"wrist":{"x":-0.5,"y":1.5,"z":0},"hand":{"x":-0.7,"y":1.3,"z":0}}}}'
