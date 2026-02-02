import * as THREE from "three";

// Direct geometric calculation of joint angles from pose keypoints
// No IK needed - we already know where all the joints are!

const RAD2DEG = 180 / Math.PI;

/**
 * Calculate joint angles directly from pose keypoints
 * @param {Object} keypoints - Object with shoulder, elbow, wrist, hand as THREE.Vector3
 * @returns {Object|null} - Joint command object or null if invalid
 */
function calculateJointAnglesFromPose(keypoints, options = {}) {
  const {
    // Joint limits in degrees
    limitsDeg = {
      Base_Rotation: [-109, 109],
      Shoulder_Lift: [0, 190],
      Elbow_Flex: [-180, 0],
      Wrist_Flex: [-170, 0],
    },
  } = options;

  const { shoulder, elbow, wrist, hand } = keypoints;
  
  if (!shoulder || !elbow || !wrist || !hand) {
    console.warn("Missing keypoints for angle calculation");
    return null;
  }

  // Calculate arm segment vectors
  const shoulderToElbow = new THREE.Vector3().subVectors(elbow, shoulder);
  const elbowToWrist = new THREE.Vector3().subVectors(wrist, elbow);
  const wristToHand = new THREE.Vector3().subVectors(hand, wrist);

  // --- BASE ROTATION (Y-axis) ---
  // Rotation in the XZ plane from shoulder to elbow (negate X to fix mirroring)
  const baseAngleRad = Math.atan2(-shoulderToElbow.x, shoulderToElbow.z);
  let baseAngleDeg = baseAngleRad * RAD2DEG;
  
  // Apply limits
  baseAngleDeg = clampAngle(baseAngleDeg, limitsDeg.Base_Rotation);

  // --- SHOULDER LIFT (X-axis) ---
  // Angle from horizontal plane to shoulder-elbow vector
  // Project onto YZ plane relative to base rotation
  const horizontalDist = Math.sqrt(shoulderToElbow.x * shoulderToElbow.x + shoulderToElbow.z * shoulderToElbow.z);
  const shoulderLiftRad = Math.atan2(shoulderToElbow.y, horizontalDist);
  let shoulderLiftDeg = shoulderLiftRad * RAD2DEG;
  
  // Apply limits
  shoulderLiftDeg = clampAngle(shoulderLiftDeg, limitsDeg.Shoulder_Lift);

  // --- ELBOW FLEX (X-axis) ---
  // Angle between shoulder-elbow and elbow-wrist vectors
  // This is the supplement of the interior angle at the elbow
  const elbowAngleRad = Math.PI - shoulderToElbow.angleTo(elbowToWrist);
  let elbowFlexDeg = elbowAngleRad * RAD2DEG;
  
  // Elbow flex is negative (folding inward)
  elbowFlexDeg = -elbowFlexDeg;
  
  // Apply limits
  elbowFlexDeg = clampAngle(elbowFlexDeg, limitsDeg.Elbow_Flex);

  // --- WRIST FLEX (X-axis) ---
  // Angle between elbow-wrist and wrist-hand vectors
  const wristAngleRad = Math.PI - elbowToWrist.angleTo(wristToHand);
  let wristFlexDeg = wristAngleRad * RAD2DEG;
  
  // Wrist flex is negative (folding inward)
  wristFlexDeg = -wristFlexDeg;
  
  // Offset so that straight wrist is at -85 degrees (neutral position)
  wristFlexDeg = wristFlexDeg + 95; // Shifts -180 to -85
  
  // Apply limits
  wristFlexDeg = clampAngle(wristFlexDeg, limitsDeg.Wrist_Flex);

  // Log calculated angles
  console.log(`Calculated angles: Base=${baseAngleDeg.toFixed(1)}°, Shoulder=${shoulderLiftDeg.toFixed(1)}°, Elbow=${elbowFlexDeg.toFixed(1)}°, Wrist=${wristFlexDeg.toFixed(1)}°`);

  // Return in same format as IK solver
  return {
    method: "set_joint_angles",
    timestamp: new Date().toISOString(),
    params: {
      units: "degrees",
      mode: "follower",
      joints: {
        Base_Rotation: { y: +baseAngleDeg.toFixed(3) },
        Shoulder_Lift: { x: +shoulderLiftDeg.toFixed(3) },
        Elbow_Flex: { x: +elbowFlexDeg.toFixed(3) },
        Wrist_Flex: { x: +wristFlexDeg.toFixed(3) },
      },
    },
  };
}

/**
 * Clamp an angle to within limits
 */
function clampAngle(angle, limits) {
  if (!limits) return angle;
  return Math.max(limits[0], Math.min(limits[1], angle));
}

export { calculateJointAnglesFromPose };
