#!/usr/bin/env python3
"""
MQTT IK Bridge

This service:
1. Subscribes to MQTT topic 'watchman_robotarm/SO-ARM101'
2. Listens for messages with method 'set_openpose_joints'
3. Computes IK using the IK solver
4. Publishes 'set_joint_angles' back to the same topic

Connects to existing mosquitto broker on localhost:9000
"""
from __future__ import annotations

import json
import numpy as np
import paho.mqtt.client as mqtt

from ik_solver import SoArmIk, q_for_joints, build_q_from_joint_values

# Configuration
URDF_PATH = "so101.urdf"
EE_FRAME = "gripper_frame_link"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MQTT_TOPIC = "watchman_robotarm/SO-ARM101"

# Joint offsets: URDF_angle = Frontend_angle + offset (in degrees)
JOINT_OFFSETS_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 90.0,
    "elbow_flex": -169.0,
    "wrist_flex": -76.8,
    "wrist_roll": 0.0,
}

# Initialize IK solver once
print("Loading URDF and initializing IK solver...")
ik_solver = SoArmIk(URDF_PATH, EE_FRAME)
offsets_rad = np.array([JOINT_OFFSETS_DEG[j] * np.pi / 180.0 for j in ARM_JOINTS])
print("IK solver ready!")


def solve_ik(target_xyz: np.ndarray, current_frontend_angles=None) -> dict:
    """
    Solve IK for target position and return frontend angles.
    
    Args:
        target_xyz: [x, y, z] target position in meters
        current_frontend_angles: Optional list of current frontend angles in radians for seeding
    
    Returns:
        Dictionary with joint names and angles in degrees
    """
    # Build q0 if current angles provided
    q0 = None
    if current_frontend_angles is not None:
        frontend_angles = np.array(current_frontend_angles, dtype=float)
        urdf_angles = frontend_angles + offsets_rad
        q0 = build_q_from_joint_values(ik_solver.model, ARM_JOINTS, urdf_angles)
    
    # Solve IK
    q_full = ik_solver.solve_xyz(
        target_xyz=target_xyz,
        q0=q0,
        max_iters=250,
        tol_m=1e-4,
        damping=1e-3,
        step_scale=0.7,
    )
    
    # Extract joint angles and convert to frontend coordinates
    urdf_angles = q_for_joints(ik_solver.model, q_full, ARM_JOINTS)
    frontend_angles = urdf_angles - offsets_rad
    frontend_angles_deg = frontend_angles * 180.0 / np.pi
    
    # Build result as joint name -> angle dictionary
    result = {
        joint_name: float(angle_deg)
        for joint_name, angle_deg in zip(ARM_JOINTS, frontend_angles_deg)
    }
    
    return result


def on_connect(client, userdata, flags, rc, properties=None):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        print(f"[Bridge] Connected to MQTT broker")
        print(f"[Bridge] Subscribing to topic: {MQTT_TOPIC}")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"[Bridge] Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Callback when a message is received"""
    try:
        message = json.loads(msg.payload.decode())
        
        # Check if this is a set_openpose_joints method call
        if message.get("method") == "set_openpose_joints":
            params = message.get("params", {})
            joints_data = params.get("joints", {})
            
            # Extract hand position as target
            hand = joints_data.get("hand", {})
            if not hand:
                print("[Bridge] No 'hand' joint found in message")
                return
            
            # Get x, y, z coordinates from hand
            target_xyz = np.array([
                float(hand.get("x", 0.18)),
                float(hand.get("y", 0.05)),
                float(hand.get("z", 0.22))
            ], dtype=float)
            
            print(f"[Bridge] Received OpenPose hand position: {target_xyz.tolist()}")
            
            # Solve IK
            joint_angles = solve_ik(target_xyz)
            
            print(f"[Bridge] Computed joint angles: {joint_angles}")
            
            # Create set_joint_angles message
            response = {
                "jsonrpc": "2.0",
                "method": "set_joint_angles",
                "params": {
                    "joints": joint_angles
                }
            }
            
            # Publish back to the same topic
            client.publish(MQTT_TOPIC, json.dumps(response))
            print(f"[Bridge] Published set_joint_angles to {MQTT_TOPIC}")
            
    except json.JSONDecodeError:
        print(f"[Bridge] Invalid JSON received")
    except Exception as e:
        print(f"[Bridge] Error processing message: {e}")
        import traceback
        traceback.print_exc()


def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    """Callback when subscription is confirmed"""
    print(f"[Bridge] Successfully subscribed to {MQTT_TOPIC}")


def on_disconnect(client, userdata, rc, properties=None):
    """Callback when disconnected"""
    if rc != 0:
        print(f"[Bridge] Unexpected disconnection. Reconnecting...")


def main():
    """Start the MQTT client"""
    import sys
    
    # MQTT broker settings
    broker = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 1883  # Standard MQTT port
    
    print(f"\n{'='*60}")
    print(f"MQTT IK Bridge")
    print(f"{'='*60}")
    print(f"Connecting to MQTT broker: {broker}:{port}")
    print(f"Topic: {MQTT_TOPIC}")
    print(f"\nPress Ctrl+C to stop\n")
    
    # Create MQTT client
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ik_bridge")
    
    # Set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    
    try:
        # Connect to broker
        client.connect(broker, port, 60)
        
        # Start the loop
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")
        client.disconnect()
    except Exception as e:
        print(f"[Bridge] Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
