# main.py
from __future__ import annotations

import numpy as np
from ik_solver import SoArmIk, q_for_joints, build_q_from_joint_values

URDF_PATH = "so101.urdf"
EE_FRAME = "gripper_frame_link"

# Joints you want to command (arm only)
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# Joint offsets: URDF_angle = Frontend_angle + offset (in degrees)
# To convert from your frontend model to URDF coordinates
JOINT_OFFSETS_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 90.0,
    "elbow_flex": -169.0,
    "wrist_flex": -76.8,
    "wrist_roll": 0.0,
}


def main() -> None:
    ik = SoArmIk(URDF_PATH, EE_FRAME)

    # Example target (meters) in the base/world frame of the URDF
    target_xyz = np.array([0.18, 0.05, 0.22], dtype=float)

    # OPTIONAL: seed IK with current arm joint angles for smooth output.
    # If you don't have current angles, leave q0=None.
    #
    # Example current angles from your frontend (radians) in ARM_JOINTS order:
    frontend_arm_angles = np.array([0.0, 0.3, 0.6, -0.4, 0.0], dtype=float)
    
    # Convert frontend angles to URDF angles by adding offsets
    offsets_rad = np.array([JOINT_OFFSETS_DEG[j] * np.pi / 180.0 for j in ARM_JOINTS])
    urdf_arm_angles = frontend_arm_angles + offsets_rad
    q0 = build_q_from_joint_values(ik.model, ARM_JOINTS, urdf_arm_angles)

    q_full = ik.solve_xyz(
        target_xyz=target_xyz,
        q0=q0,            # set to None if you don't want seeding
        max_iters=250,
        tol_m=1e-4,
        damping=1e-3,
        step_scale=0.7,
    )

    urdf_arm_angles = q_for_joints(ik.model, q_full, ARM_JOINTS)
    
    # Convert URDF angles back to frontend angles by subtracting offsets
    frontend_arm_angles = urdf_arm_angles - offsets_rad

    print("Target xyz (m):", target_xyz.tolist())
    print("Arm joints:", ARM_JOINTS)
    print("\nURDF angles (rad):", urdf_arm_angles.tolist())
    print("URDF angles (deg):", (urdf_arm_angles * 180.0 / np.pi).tolist())
    print("\nFrontend angles (rad):", frontend_arm_angles.tolist())
    print("Frontend angles (deg):", (frontend_arm_angles * 180.0 / np.pi).tolist())


if __name__ == "__main__":
    main()
