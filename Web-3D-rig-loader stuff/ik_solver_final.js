import * as THREE from "three";
// ===== DROP-IN: Coupled multi-target IK (3 points, position-only) =====
// Scene units in meters. Joint names capitalized as per your GLB.
// Solves in one system so upstream joints don't sabotage downstream joints.

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

function axisVectorLocal(axisChar) {
  if (axisChar === "x") return new THREE.Vector3(1, 0, 0);
  if (axisChar === "y") return new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3(0, 0, 1);
}

function getWorldPos(obj) {
  const v = new THREE.Vector3();
  obj.getWorldPosition(v);
  return v;
}

function getWorldAxis(obj, axisChar) {
  const axis = axisVectorLocal(axisChar);
  const q = new THREE.Quaternion();
  obj.getWorldQuaternion(q);
  return axis.applyQuaternion(q).normalize();
}

function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

// Generic small matrix inversion (Gauss-Jordan), good for 9x9.
function invertMatrix(mat, n) {
  const a = new Float64Array(mat); // copy
  const inv = new Float64Array(n * n);
  for (let i = 0; i < n; i++) inv[i * n + i] = 1;

  for (let col = 0; col < n; col++) {
    // pivot
    let pivotRow = col;
    let pivotVal = Math.abs(a[col * n + col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(a[r * n + col]);
      if (v > pivotVal) { pivotVal = v; pivotRow = r; }
    }
    if (pivotVal < 1e-12) return null;

    // swap rows
    if (pivotRow !== col) {
      for (let c = 0; c < n; c++) {
        let t = a[col * n + c];
        a[col * n + c] = a[pivotRow * n + c];
        a[pivotRow * n + c] = t;

        t = inv[col * n + c];
        inv[col * n + c] = inv[pivotRow * n + c];
        inv[pivotRow * n + c] = t;
      }
    }

    // normalize pivot row
    const piv = a[col * n + col];
    for (let c = 0; c < n; c++) {
      a[col * n + c] /= piv;
      inv[col * n + c] /= piv;
    }

    // eliminate
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = a[r * n + col];
      if (Math.abs(f) < 1e-15) continue;
      for (let c = 0; c < n; c++) {
        a[r * n + c] -= f * a[col * n + c];
        inv[r * n + c] -= f * inv[col * n + c];
      }
    }
  }
  return inv;
}

/**
 * Main solver:
 * - joints: Base_Rotation(y), Shoulder_Lift(x), Elbow_Flex(x), Wrist_Flex(x), Wrist_Roll(y optional)
 * - points: elbowPoint, wristPoint, eePoint
 * - targets: targetA, targetB, targetC (world Vector3)
 *
 * Returns a JSON command in your format.
 */
function solveCoupled3TargetIKToJson(root, targets, options = {}) {
  const {
    // Joint definitions (GLB names + axis)
    chain = [
      { glb: "Base_Rotation", axis: "y", json: "base_rotation" },
      { glb: "Shoulder_Lift", axis: "x", json: "shoulder_lift" },
      { glb: "Elbow_Flex", axis: "x", json: "elbow_flex" },
      { glb: "Wrist_Flex", axis: "x", json: "wrist_flex" },
    ],

    elbowPointName = "Elbow_Point", // recommend empties; can set to "Elbow_Flex"
    wristPointName = "Wrist_Point", // or "Wrist_Flex"
    eePointName = "EndEffector",

    // weights: prioritize downstream if needed
    wA = 1.4, // elbow target weight
    wB = 1.3, // wrist target weight
    wC = 0.5, // EE target weight

    iterations = 40,
    tolerance = 0.005,  // meters
    lambda = 0.2,       // damping
    stepScale = 0.7,
    maxStepRad = 0.15,
    
    // Per-joint max steps (base can take bigger steps to reach targets)
    maxStepRadPerJoint = {
      Base_Rotation: 2, // This will need to be ajusted for 3d   
      Shoulder_Lift: 0.15,
      Elbow_Flex: 0.15,
      Wrist_Flex: 0.12,
    },

    // optional limits in degrees by glb name
    limitsDeg = {
      Base_Rotation: [-109, 109],
      Shoulder_Lift: [0, 190],
      Elbow_Flex: [-180, 0],
      Wrist_Flex: [-170, 0],
    },
  } = options;

  const { targetA, targetB, targetC } = targets;
  if (!targetA || !targetB || !targetC) {
    throw new Error("targets must be {targetA, targetB, targetC} as THREE.Vector3 (world)");
  }

  // Resolve points
  const pAObj = root.getObjectByName(elbowPointName) || root.getObjectByName("Elbow_Flex");
  const pBObj = root.getObjectByName(wristPointName) || root.getObjectByName("Wrist_Flex");
  const pCObj = root.getObjectByName(eePointName);
  if (!pAObj || !pBObj || !pCObj) throw new Error("Missing point nodes (elbow/wrist/ee)");

  // Resolve joints
  const joints = chain.map(j => {
    const obj = root.getObjectByName(j.glb);
    if (!obj) throw new Error(`Missing joint node "${j.glb}"`);
    return { ...j, obj };
  });

  const limRad = (glb) => {
    const d = limitsDeg?.[glb];
    if (!d) return null;
    return [d[0] * DEG2RAD, d[1] * DEG2RAD];
  };

  // Iterate
  for (let it = 0; it < iterations; it++) {
    root.updateMatrixWorld(true);

    const pA = getWorldPos(pAObj);
    const pB = getWorldPos(pBObj);
    const pC = getWorldPos(pCObj);

    const eA = targetA.clone().sub(pA).multiplyScalar(wA);
    const eB = targetB.clone().sub(pB).multiplyScalar(wB);
    const eC = targetC.clone().sub(pC).multiplyScalar(wC);

    const err = Math.max(eA.length() / Math.max(wA, 1e-9), eB.length() / Math.max(wB, 1e-9), eC.length() / Math.max(wC, 1e-9));
    if (err < tolerance) break;

    // Debug logging
    if (it % 5 === 0) {
      console.log(
        `it=${it}`,
        "errA", targetA.distanceTo(pA).toFixed(4),
        "errB", targetB.distanceTo(pB).toFixed(4),
        "errC", targetC.distanceTo(pC).toFixed(4)
      );
    }

    // Build J columns for each joint:
    // For each joint i:
    //   J_Ai = ω × (pA - p_i)
    //   J_Bi = ω × (pB - p_i)
    //   J_Ci = ω × (pC - p_i)
    // Stack into 9xN column vector.
    const cols = [];
    for (const j of joints) {
      const pi = getWorldPos(j.obj);
      const w = getWorldAxis(j.obj, j.axis);

      const JA = w.clone().cross(pA.clone().sub(pi)).multiplyScalar(wA);
      const JB = w.clone().cross(pB.clone().sub(pi)).multiplyScalar(wB);
      const JC = w.clone().cross(pC.clone().sub(pi)).multiplyScalar(wC);

      cols.push({ JA, JB, JC });
    }

    // DLS: dq = J^T (J J^T + λ² I)^-1 e
    // Here e is 9x1, J is 9xN, so invert 9x9.
    const nE = 9;
    const A = new Float64Array(nE * nE);

    // A = sum_i col_i * col_i^T + λ² I
    for (const c of cols) {
      const v = [
        c.JA.x, c.JA.y, c.JA.z,
        c.JB.x, c.JB.y, c.JB.z,
        c.JC.x, c.JC.y, c.JC.z,
      ];
      for (let r = 0; r < nE; r++) {
        for (let k = 0; k < nE; k++) {
          A[r * nE + k] += v[r] * v[k];
        }
      }
    }
    const l2 = lambda * lambda;
    for (let d = 0; d < nE; d++) A[d * nE + d] += l2;

    const invA = invertMatrix(A, nE);
    if (!invA) break;

    const e9 = new Float64Array([
      eA.x, eA.y, eA.z,
      eB.x, eB.y, eB.z,
      eC.x, eC.y, eC.z,
    ]);

    // v = invA * e
    const v9 = new Float64Array(nE);
    for (let r = 0; r < nE; r++) {
      let sum = 0;
      for (let k = 0; k < nE; k++) sum += invA[r * nE + k] * e9[k];
      v9[r] = sum;
    }

    // dq_i = col_i · v
    for (let i = 0; i < joints.length; i++) {
      const j = joints[i];
      const c = cols[i];

      let dq =
        c.JA.x * v9[0] + c.JA.y * v9[1] + c.JA.z * v9[2] +
        c.JB.x * v9[3] + c.JB.y * v9[4] + c.JB.z * v9[5] +
        c.JC.x * v9[6] + c.JC.y * v9[7] + c.JC.z * v9[8];

      dq *= stepScale;
      
      // Invert base rotation - robot arm rotates away from target
      if (j.glb === "Base_Rotation") {
        dq = -dq;
      }
      
      // Use per-joint max step if defined, otherwise fallback to global maxStepRad
      const jointMaxStep = maxStepRadPerJoint[j.glb] || maxStepRad;
      dq = clamp(dq, -jointMaxStep, jointMaxStep);

      let next = j.obj.rotation[j.axis] + dq;

      const lim = limRad(j.glb);
      if (lim) next = clamp(next, lim[0], lim[1]);

      j.obj.rotation[j.axis] = next;
    }
  }

  // Output JSON (degrees)
  const outJoints = {};
  for (const j of joints) {
    outJoints[j.json] = { [j.axis]: +(j.obj.rotation[j.axis] * RAD2DEG).toFixed(3) };
  }
  outJoints.gripper = { z: 0 };

  return {
    method: "set_joint_angles",
    timestamp: new Date().toISOString(),
    params: {
      units: "degrees",
      mode: "follower",
      joints: outJoints,
    },
  };
}

export { solveCoupled3TargetIKToJson };
