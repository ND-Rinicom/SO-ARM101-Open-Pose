// ===== DROP-IN: Multi-target "best effort" solver (meter-scale) =====
// Assumes your scene units are meters (1 unit = 1m).
// Your GLB joint names are capitalized (e.g., "Base_Rotation").
// What it does (per iteration):
// 1) Base_Rotation(y) + Shoulder_Lift(x) try to move Elbow_Flex point -> targetA
// 2) Elbow_Flex(x) tries to move Wrist_Flex point -> targetB
// 3) Wrist_Flex(x) tries to move End_Effector point -> targetC
//
// Then call jointsToCommandJson(robotRoot) to get your JSON payload.

import * as THREE from "three";

// ---- helpers ----
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

// ---- one-joint step: rotate joint a bit to move a point toward a target ----
function stepOneJointToward(root, jointObj, jointAxisChar, pointObj, targetWorld, {
  gain = 0.6,        // meters-scale: start here; adjust if too slow/fast
  maxStepRad = 0.12, // per-iteration clamp (radians)
  limitRad = null,   // [min,max] radians or null
} = {}) {
  root.updateMatrixWorld(true);

  const pj = getWorldPos(jointObj); // joint pivot
  const p  = getWorldPos(pointObj); // point we want to move
  const e  = targetWorld.clone().sub(p);

  const w = getWorldAxis(jointObj, jointAxisChar); // world rotation axis
  const dp_dtheta = w.clone().cross(p.clone().sub(pj)); // ω × (p - pj)

  // Jacobian-transpose 1D step:
  // dtheta ≈ gain * (dp/dθ · error)
  let dtheta = gain * dp_dtheta.dot(e);

  dtheta = clamp(dtheta, -maxStepRad, maxStepRad);

  let next = jointObj.rotation[jointAxisChar] + dtheta;
  if (limitRad) next = clamp(next, limitRad[0], limitRad[1]);

  jointObj.rotation[jointAxisChar] = next;
  root.updateMatrixWorld(true);
}

/**
 * Multi-target solver:
 * targets = { targetA: Vector3, targetB: Vector3, targetC: Vector3 } (WORLD SPACE, meters)
 */
function solveMultiTargets(root, targets, options = {}) {
  const {
    // Joint node names (match your GLB)
    baseName = "Base_Rotation",
    shoulderName = "Shoulder_Lift",
    elbowName = "Elbow_Flex",
    wristFlexName = "Wrist_Flex",
    wristRollName = "Wrist_Roll", // optional for output only

    // "Points" to drive toward targets (often just the joint nodes themselves)
    elbowPointName = "Elbow_Flex",
    wristPointName = "Wrist_Flex",
    eePointName = "EndEffector",

    // Iteration + tuning
    iterations = 25,
    gainA = 0.6, gainB = 0.6, gainC = 0.6,
    maxStepRad = 0.12,

    // Optional joint limits in degrees (set null to disable)
    limitsDeg = {
      Base_Rotation: [-180, 180],
      Shoulder_Lift: [-120, 120],
      Elbow_Flex: [-150, 150],
      Wrist_Flex: [-180, 180],
      Wrist_Roll: [-180, 180],
    },
  } = options;

  const base = root.getObjectByName(baseName);
  const shoulder = root.getObjectByName(shoulderName);
  const elbow = root.getObjectByName(elbowName);
  const wristFlex = root.getObjectByName(wristFlexName);

  const elbowPoint = root.getObjectByName(elbowPointName);
  const wristPoint = root.getObjectByName(wristPointName);
  const eePoint = root.getObjectByName(eePointName);

  if (!base || !shoulder || !elbow || !wristFlex) {
    throw new Error("solveMultiTargets: missing one or more joint nodes");
  }
  if (!elbowPoint || !wristPoint || !eePoint) {
    throw new Error("solveMultiTargets: missing one or more point nodes (elbow/wrist/ee)");
  }

  const lim = (key) => {
    const d = limitsDeg?.[key];
    if (!d) return null;
    return [d[0] * DEG2RAD, d[1] * DEG2RAD];
  };

  const { targetA, targetB, targetC } = targets;
  if (!targetA || !targetB || !targetC) {
    throw new Error("solveMultiTargets: targets must be {targetA, targetB, targetC} as THREE.Vector3 (world, meters)");
  }

  for (let it = 0; it < iterations; it++) {
    // Task A: base + shoulder move elbowPoint toward targetA
    stepOneJointToward(root, base, "y", elbowPoint, targetA, {
      gain: gainA, maxStepRad, limitRad: lim("Base_Rotation"),
    });
    stepOneJointToward(root, shoulder, "x", elbowPoint, targetA, {
      gain: gainA, maxStepRad, limitRad: lim("Shoulder_Lift"),
    });

    // Task B: elbow moves wristPoint toward targetB
    stepOneJointToward(root, elbow, "x", wristPoint, targetB, {
      gain: gainB, maxStepRad, limitRad: lim("Elbow_Flex"),
    });

    // Task C: wrist_flex moves eePoint toward targetC
    stepOneJointToward(root, wristFlex, "x", eePoint, targetC, {
      gain: gainC, maxStepRad, limitRad: lim("Wrist_Flex"),
    });
  }
}

/**
 * Pack current joint rotations into your JSON command format (degrees).
 * NOTE: gripper is set to 0 by default; adjust if you want.
 */
function jointsToCommandJson(root, options = {}) {
  const {
    // GLB names:
    baseName = "Base_Rotation",
    shoulderName = "Shoulder_Lift",
    elbowName = "Elbow_Flex",
    wristFlexName = "Wrist_Flex",
    wristRollName = "Wrist_Roll",

    // JSON keys you want (lowercase snake_case like your original):
    jsonKeys = {
      Base_Rotation: ["Base_Rotation", "y"],
      Shoulder_Lift: ["Shoulder_Lift", "x"],
      Elbow_Flex: ["Elbow_Flex", "x"],
      Wrist_Flex: ["Wrist_Flex", "x"],
      Wrist_Roll: ["Wrist_Roll", "y"],
    },
  } = options;

  const joints = {};

  const nameList = [baseName, shoulderName, elbowName, wristFlexName, wristRollName];
  for (const glbName of nameList) {
    const obj = root.getObjectByName(glbName);
    if (!obj) continue;

    const mapping = jsonKeys[glbName];
    if (!mapping) continue;

    const [jsonName, axis] = mapping;
    joints[jsonName] = { [axis]: +(obj.rotation[axis] * RAD2DEG).toFixed(3) };
  }

  return {
    method: "set_joint_angles",
    timestamp: new Date().toISOString(),
    params: {
      units: "degrees",
      mode: "follower",
      joints,
    },
  };
}

// ===== Example usage =====
// const targets = {
//   targetA: new THREE.Vector3( 0.05, 0.30, 0.10), // elbow goal (world, meters)
//   targetB: new THREE.Vector3( 0.20, 0.25, 0.15), // wrist goal (world, meters)
//   targetC: new THREE.Vector3( 0.30, 0.20, 0.20), // EE goal (world, meters)
// };
// solveMultiTargets(robotRoot, targets, { eePointName: "End_Effector" });
// const cmd = jointsToCommandJson(robotRoot);
// console.log(cmd);


export { solveMultiTargets, jointsToCommandJson };