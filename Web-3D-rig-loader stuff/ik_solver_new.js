import * as THREE from "three";

// ===== 6D (position + orientation) Jacobian DLS IK for Three.js =====

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

const IK_CHAIN = [
  { key: "Base_Rotation", axis: "y", objName: "Base_Rotation" },
  { key: "Shoulder_Lift", axis: "x", objName: "Shoulder_Lift" },
  { key: "Elbow_Flex", axis: "x", objName: "Elbow_Flex" },
];

function isoTimestampNow() {
  return new Date().toISOString();
}

function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

function axisVectorLocal(axisChar) {
  if (axisChar === "x") return new THREE.Vector3(1, 0, 0);
  if (axisChar === "y") return new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3(0, 0, 1);
}

function getWorldAxis(obj, axisChar) {
  const axis = axisVectorLocal(axisChar);
  const q = new THREE.Quaternion();
  obj.getWorldQuaternion(q);
  return axis.applyQuaternion(q).normalize();
}

function getWorldPos(obj) {
  const p = new THREE.Vector3();
  obj.getWorldPosition(p);
  return p;
}

function getWorldQuat(obj) {
  const q = new THREE.Quaternion();
  obj.getWorldQuaternion(q);
  return q;
}

// Orientation error as a 3-vector (axis * angle) in world space
function orientationErrorVec(targetWorldQuat, currentWorldQuat) {
  // qErr = qTarget * inverse(qCurrent)
  const qErr = targetWorldQuat.clone().multiply(currentWorldQuat.clone().invert());

  // Ensure shortest rotation (optional but helps)
  if (qErr.w < 0) {
    qErr.x *= -1;
    qErr.y *= -1;
    qErr.z *= -1;
    qErr.w *= -1;
  }

  const w = THREE.MathUtils.clamp(qErr.w, -1, 1);
  const angle = 2 * Math.acos(w);

  const s = Math.sqrt(1 - w * w);
  if (s < 1e-8 || angle < 1e-8) return new THREE.Vector3(0, 0, 0);

  const axis = new THREE.Vector3(qErr.x / s, qErr.y / s, qErr.z / s);
  return axis.multiplyScalar(angle);
}

// Solve Δq = J^T (J J^T + λ² I)^-1 e
// Here e is 6x1, J is 6xN, so we invert a 6x6 matrix.
// We'll do a small generic 6x6 Gauss-Jordan inversion.
function invertMatrix(mat, n) {
  // mat: Float64Array length n*n (row-major)
  const a = new Float64Array(mat); // copy
  const inv = new Float64Array(n * n);
  for (let i = 0; i < n; i++) inv[i * n + i] = 1;

  for (let col = 0; col < n; col++) {
    // Pivot
    let pivotRow = col;
    let pivotVal = Math.abs(a[col * n + col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(a[r * n + col]);
      if (v > pivotVal) { pivotVal = v; pivotRow = r; }
    }
    if (pivotVal < 1e-12) return null; // singular

    // Swap rows
    if (pivotRow !== col) {
      for (let c = 0; c < n; c++) {
        const t1 = a[col * n + c];
        a[col * n + c] = a[pivotRow * n + c];
        a[pivotRow * n + c] = t1;

        const t2 = inv[col * n + c];
        inv[col * n + c] = inv[pivotRow * n + c];
        inv[pivotRow * n + c] = t2;
      }
    }

    // Normalize pivot row
    const pivot = a[col * n + col];
    for (let c = 0; c < n; c++) {
      a[col * n + c] /= pivot;
      inv[col * n + c] /= pivot;
    }

    // Eliminate other rows
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = a[r * n + col];
      if (Math.abs(factor) < 1e-15) continue;
      for (let c = 0; c < n; c++) {
        a[r * n + c] -= factor * a[col * n + c];
        inv[r * n + c] -= factor * inv[col * n + c];
      }
    }
  }

  return inv;
}

function solveDLS6D(Jcols6, e6, lambda = 0.2) {
  // Jcols6: array of { jp: Vector3, jr: Vector3 } columns
  // Build A = J J^T + λ² I (6x6)
  const n = 6;
  const A = new Float64Array(n * n);

  for (const col of Jcols6) {
    const v = [
      col.jp.x, col.jp.y, col.jp.z,
      col.jr.x, col.jr.y, col.jr.z
    ];
    // A += v * v^T
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        A[r * n + c] += v[r] * v[c];
      }
    }
  }

  const l2 = lambda * lambda;
  for (let i = 0; i < n; i++) A[i * n + i] += l2;

  const invA = invertMatrix(A, n);
  if (!invA) return new Array(Jcols6.length).fill(0);

  // v = invA * e6
  const v6 = new Float64Array(6);
  for (let r = 0; r < n; r++) {
    let sum = 0;
    for (let c = 0; c < n; c++) sum += invA[r * n + c] * e6[c];
    v6[r] = sum;
  }

  // Δq = J^T * v
  const dqs = new Array(Jcols6.length).fill(0);
  for (let i = 0; i < Jcols6.length; i++) {
    const col = Jcols6[i];
    dqs[i] =
      col.jp.x * v6[0] + col.jp.y * v6[1] + col.jp.z * v6[2] +
      col.jr.x * v6[3] + col.jr.y * v6[4] + col.jr.z * v6[5];
  }
  return dqs;
}

/**
 * 6D IK: reach target position + orientation, return joint angles JSON (degrees).
 *
 * Units: your scene is 1 unit = 1mm, so tolerancePos is in mm.
 */
function solveIK6DToJson(root, targetWorldPos, targetWorldQuat, options = {}) {
  const {
    endEffectorName = "EndEffector", // set this to your EE empty
    iterations = 50,
    tolerancePos = 2.0,           // mm
    toleranceRot = 0.02,          // radians (~1.1 deg)
    lambda = 0.25,
    stepScale = 0.5,
    maxStepRad = 0.2,

    // Weights: trade off position vs orientation importance
    wPos = 1.0,
    wRot = 0.1,

    limitsDeg = null, // optionally same shape as earlier
  } = options;

  const ee = root.getObjectByName(endEffectorName);
  if (!ee) throw new Error(`End effector node "${endEffectorName}" not found`);

  const joints = IK_CHAIN.map(j => {
    const obj = root.getObjectByName(j.objName);
    if (!obj) throw new Error(`Joint node "${j.objName}" not found`);
    return { ...j, obj };
  });

  // q init from scene
  const q = joints.map(j => j.obj.rotation[j.axis]);

  for (let iter = 0; iter < iterations; iter++) {
    root.updateMatrixWorld(true);

    const pe = getWorldPos(ee);
    const qe = getWorldQuat(ee);

    const posErr = targetWorldPos.clone().sub(pe);           // (mm)
    const rotErr = orientationErrorVec(targetWorldQuat, qe); // (rad)

    if (posErr.length() < tolerancePos && rotErr.length() < toleranceRot) break;

    // Weight the error (and implicitly the Jacobian rows)
    const e6 = new Float64Array(6);
    e6[0] = posErr.x * wPos; e6[1] = posErr.y * wPos; e6[2] = posErr.z * wPos;
    e6[3] = rotErr.x * wRot; e6[4] = rotErr.y * wRot; e6[5] = rotErr.z * wRot;

    // Build 6D Jacobian columns
    const Jcols6 = [];
    for (const j of joints) {
      const pi = getWorldPos(j.obj);
      const w = getWorldAxis(j.obj, j.axis); // world axis (unit)

      const jp = w.clone().cross(pe.clone().sub(pi)).multiplyScalar(wPos); // position row weight
      const jr = w.clone().multiplyScalar(wRot);                           // orientation row weight

      Jcols6.push({ jp, jr });
    }

    const dqs = solveDLS6D(Jcols6, e6, lambda);

    for (let i = 0; i < joints.length; i++) {
      const j = joints[i];

      let dq = dqs[i] * stepScale;
      dq = clamp(dq, -maxStepRad, maxStepRad);

      let next = q[i] + dq;

      if (limitsDeg?.[j.key]) {
        const [loDeg, hiDeg] = limitsDeg[j.key];
        next = clamp(next, loDeg * DEG2RAD, hiDeg * DEG2RAD);
      }

      q[i] = next;
      j.obj.rotation[j.axis] = next;
    }
  }

  // Output JSON
  const jointsOut = {};
  for (let i = 0; i < joints.length; i++) {
    const j = joints[i];
    jointsOut[j.key] = { [j.axis]: +(q[i] * RAD2DEG).toFixed(3) };
  }

  return {
    method: "set_joint_angles",
    timestamp: isoTimestampNow(),
    params: {
      units: "degrees",
      mode: "follower",
      joints: jointsOut,
    },
  };
}

export { solveIK6DToJson };