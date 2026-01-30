# ik/ik_solver.py
from __future__ import annotations

import numpy as np
import pinocchio as pin


class SoArmIk:
    """
    Position-only IK (x,y,z) using damped least squares (DLS).
    Loads the URDF with Pinocchio, uses the EE frame Jacobian, iteratively solves.

    Notes:
    - target_xyz is in meters, in the URDF/world base frame.
    - Use q0=current joint state for smooth results (seeded IK).
    """

    def __init__(self, urdf_path: str, ee_frame_name: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # End-effector frame id
        try:
            self.ee_fid = self.model.getFrameId(ee_frame_name)
        except Exception as e:
            raise ValueError(
                f"End-effector frame '{ee_frame_name}' not found. "
                f"Check URDF frame/link names."
            ) from e

        # Joint limits (may be +/-inf for continuous joints)
        self.q_min = self.model.lowerPositionLimit.copy()
        self.q_max = self.model.upperPositionLimit.copy()

    def solve_xyz(
        self,
        target_xyz: np.ndarray,
        q0: np.ndarray | None = None,
        max_iters: int = 200,
        tol_m: float = 1e-4,
        damping: float = 1e-3,
        step_scale: float = 0.6,
        clamp_limits: bool = True,
    ) -> np.ndarray:
        """
        target_xyz: (3,) in meters
        q0: (model.nq,) initial guess. If None, uses neutral pose.

        returns q: (model.nq,) full configuration vector
        """
        target_xyz = np.asarray(target_xyz, dtype=float).reshape(3)

        q = pin.neutral(self.model) if q0 is None else np.array(q0, dtype=float).copy()
        if q.shape[0] != self.model.nq:
            raise ValueError(f"q0 must have shape ({self.model.nq},), got {q.shape}")

        for _ in range(max_iters):
            # Forward kinematics (frames)
            pin.framesForwardKinematics(self.model, self.data, q)

            # Current EE translation in world frame
            oMf = self.data.oMf[self.ee_fid]
            current_xyz = oMf.translation

            err = target_xyz - current_xyz
            if float(np.linalg.norm(err)) < tol_m:
                break

            # 6xN Jacobian (LOCAL_WORLD_ALIGNED makes translation rows align with world axes)
            J6 = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_fid,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            J = J6[:3, :]  # translation part => 3xN

            # Damped least squares: dq = J^T (J J^T + λ^2 I)^-1 * err
            JJt = J @ J.T
            lam2I = (damping**2) * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt + lam2I, err)

            # Integrate on the manifold (handles revolute joints properly)
            q = pin.integrate(self.model, q, step_scale * dq)

            if clamp_limits:
                q = np.minimum(np.maximum(q, self.q_min), self.q_max)

        return q


def q_for_joints(model: pin.Model, q: np.ndarray, joint_names: list[str]) -> np.ndarray:
    """
    Extract 1-DoF joint values from full q in the order of joint_names.
    """
    q = np.asarray(q, dtype=float)
    out: list[float] = []
    for name in joint_names:
        jid = model.getJointId(name)
        if jid == 0:
            raise ValueError(f"Joint '{name}' not found in model.")
        j = model.joints[jid]
        if j.nq != 1:
            raise ValueError(f"Joint '{name}' has nq={j.nq}, expected 1.")
        out.append(float(q[j.idx_q]))
    return np.array(out, dtype=float)


def build_q_from_joint_values(
    model: pin.Model,
    joint_names: list[str],
    joint_values: list[float] | np.ndarray,
    q_ref: np.ndarray | None = None,
) -> np.ndarray:
    """
    Build a full q vector from a subset of joint values (by name).
    Useful for seeding IK with your robot's current joint angles.

    joint_names: names corresponding to joint_values, 1-DoF each.
    joint_values: list/array of radians.
    q_ref: optional reference full q; if None uses neutral pose.
    """
    q = pin.neutral(model) if q_ref is None else np.array(q_ref, dtype=float).copy()
    joint_values = np.asarray(joint_values, dtype=float).reshape(-1)

    if len(joint_names) != joint_values.shape[0]:
        raise ValueError("joint_names and joint_values must have same length")

    for name, val in zip(joint_names, joint_values):
        jid = model.getJointId(name)
        if jid == 0:
            raise ValueError(f"Joint '{name}' not found in model.")
        j = model.joints[jid]
        if j.nq != 1:
            raise ValueError(f"Joint '{name}' has nq={j.nq}, expected 1.")
        q[j.idx_q] = float(val)

    return q
