import * as THREE from "three";

// ===== 3D Jacobian DLS IK for Three.js =====

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

const IK_CHAIN = [
  { key: "Base_Rotation", axis: "y", objName: "Base_Rotation" },
  { key: "Shoulder_Lift", axis: "x", objName: "Shoulder_Lift" },
  { key: "Elbow_Flex", axis: "x", objName: "Elbow_Flex" },
  { key: "Wrist_Flex", axis: "x", objName: "Wrist_Flex" },
  { key: "Wrist_Roll", axis: "y", objName: "Wrist_Roll" },
];

// Helper: build timestamp like your example
function isoTimestampNow() {
  return new Date().toISOString();
}

// Helper: axis unit vector in local joint space
function axisVectorLocal(axisChar) {
  if (axisChar === "x") return new THREE.Vector3(1, 0, 0);
  if (axisChar === "y") return new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3(0, 0, 1);
}

// Helper: world-space axis for a joint
function getWorldAxis(obj, axisChar) {
  const axis = axisVectorLocal(axisChar);
  const q = new THREE.Quaternion();
  obj.getWorldQuaternion(q);
  return axis.applyQuaternion(q).normalize();
}

// Helper: world-space position
function getWorldPos(obj) {
  const p = new THREE.Vector3();
  obj.getWorldPosition(p);
  return p;
}

// Solve Δq with DLS using only a 3×3 inversion
function solveDLS(Jcols, errorVec3, lambda = 0.15) {
  // Build A = J*J^T + λ² I
  // Jcols are Vector3 columns
  let a00=0,a01=0,a02=0,a11=0,a12=0,a22=0;

  for (const c of Jcols) {
    a00 += c.x*c.x; a01 += c.x*c.y; a02 += c.x*c.z;
    a11 += c.y*c.y; a12 += c.y*c.z;
    a22 += c.z*c.z;
  }

  const l2 = lambda * lambda;
  a00 += l2; a11 += l2; a22 += l2;

  // Invert symmetric 3x3
  const b00 = a11*a22 - a12*a12;
  const b01 = a02*a12 - a01*a22;
  const b02 = a01*a12 - a02*a11;
  const b11 = a00*a22 - a02*a02;
  const b12 = a02*a01 - a00*a12;
  const b22 = a00*a11 - a01*a01;

  const det = a00*b00 + a01*b01 + a02*b02;
  if (Math.abs(det) < 1e-10) return new Array(Jcols.length).fill(0);

  const invDet = 1 / det;

  // v = inv(A) * error
  const ex = errorVec3.x, ey = errorVec3.y, ez = errorVec3.z;
  const vx = (b00*ex + b01*ey + b02*ez) * invDet;
  const vy = (b01*ex + b11*ey + b12*ez) * invDet;
  const vz = (b02*ex + b12*ey + b22*ez) * invDet;

  const v = new THREE.Vector3(vx, vy, vz);

  // Δq = J^T * v  => Δq_i = col_i · v
  return Jcols.map(c => c.dot(v));
}

// Optional: clamp function
function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

/**
 * Compute IK joint angles to reach a world-space target position.
 *
 * @param {THREE.Scene | THREE.Object3D} root - scene or robot root that contains joint nodes
 * @param {THREE.Vector3} targetWorldPos - desired end-effector position in world space
 * @param {object} options
 * @returns JSON in your "set_joint_angles" format (degrees)
 */
function solveIK3DToJson(root, targetWorldPos, options = {}) {
  const {
    endEffectorName = "wrist_roll", // CHANGE if you have a tool-tip node
    iterations = 30,
    tolerance = 0.002,      // meters-ish (depends on your scale)
    lambda = 0.15,          // damping (higher = more stable, less accurate)
    stepScale = 0.6,        // overall step size
    maxStepRad = 0.25,      // per-joint per-iteration clamp
    // Optional joint limits (degrees). Put null to disable.
    limitsDeg = {
      base_rotation: null,
      shoulder_lift: null,
      elbow_flex: null,
      wrist_flex: null,
      wrist_roll: null,
    },
    // Initial guess (degrees). If not provided, uses current scene rotations.
    initialGuessDeg = null,
  } = options;

  // 1) Resolve objects
  const ee = root.getObjectByName(endEffectorName);
  if (!ee) throw new Error(`End effector node "${endEffectorName}" not found`);

  const joints = IK_CHAIN.map(j => {
    const obj = root.getObjectByName(j.objName);
    if (!obj) throw new Error(`Joint node "${j.objName}" not found`);
    return { ...j, obj };
  });

  // 2) Initialize q from either provided guess or current node rotations
  const q = new Array(joints.length).fill(0);

  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    if (initialGuessDeg && initialGuessDeg[j.key] != null) {
      q[i] = initialGuessDeg[j.key] * DEG2RAD;
      j.obj.rotation[j.axis] = q[i];
    } else {
      q[i] = j.obj.rotation[j.axis];
    }
  }

  root.updateMatrixWorld(true);

  // 3) Iterate
  for (let iter = 0; iter < iterations; iter++) {
    root.updateMatrixWorld(true);

    const pe = getWorldPos(ee);
    const error = targetWorldPos.clone().sub(pe);

    if (iter === 0 || iter === iterations - 1) {
      console.log(`IK Iteration ${iter}: EE pos=${pe.x.toFixed(3)},${pe.y.toFixed(3)},${pe.z.toFixed(3)}, error=${error.length().toFixed(4)}`);
    }

    if (error.length() < tolerance) {
      console.log(`IK converged at iteration ${iter}, error=${error.length().toFixed(6)}`);
      break;
    }

    // Build Jacobian columns
    const Jcols = [];
    for (const j of joints) {
      const pi = getWorldPos(j.obj);
      const w = getWorldAxis(j.obj, j.axis);
      const col = w.clone().cross(pe.clone().sub(pi)); // ω × (pe - pi)
      Jcols.push(col);
    }

    // Solve
    const dqs = solveDLS(Jcols, error, lambda);

    if (iter === 0) {
      console.log("First iteration dqs (rad):", dqs);
    }

    // Apply update
    for (let i = 0; i < joints.length; i++) {
      const j = joints[i];

      let dq = dqs[i] * stepScale;
      dq = clamp(dq, -maxStepRad, maxStepRad);

      let next = q[i] + dq;

      // Apply joint limits if provided
      const lim = limitsDeg?.[j.key];
      if (lim && lim.length === 2) {
        const lo = lim[0] * DEG2RAD;
        const hi = lim[1] * DEG2RAD;
        const beforeClamp = next;
        next = clamp(next, lo, hi);
        
        if (iter === 0 && beforeClamp !== next) {
          console.log(`Joint ${j.key} clamped from ${(beforeClamp * RAD2DEG).toFixed(1)}° to ${(next * RAD2DEG).toFixed(1)}° (limits: [${lim[0]}, ${lim[1]}])`);
        }
      }

      q[i] = next;
      j.obj.rotation[j.axis] = next;
    }
  }

  // 4) Return JSON result (degrees)
  const jointsOut = {};
  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    const deg = q[i] * RAD2DEG;
    jointsOut[j.key] = { [j.axis]: +deg.toFixed(3) };
  }

  return {
    units: "degrees",
    joints: jointsOut,
  };
}

export { solveIK3DToJson };
