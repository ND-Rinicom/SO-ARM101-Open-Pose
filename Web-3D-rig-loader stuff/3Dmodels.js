import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { solveCoupled3TargetIKToJson } from "./ik_solver_final.js";
import { createIKDebugRig, updateIKDebugRig } from "./ik_debug.js";

// --- BASIC SETUP ---
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);

const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// --- GLOBALS ---
let model;
let wireframeMode = false;

const bonesByName = new Map();
const objectsByName = new Map(); // For models without bones

let debugRig = null; // IK debug rig

let modelPositionOffset = { x: 0, y: 0, z: 0 }; // Configurable model position offset

// Skeleton lines for OpenPose visualization
let shoulderToElbowLine = null;  // Green
let elbowToWristLine = null;     // Red
let wristToHandLine = null;      // Blue

let lastIKTimestamp = null; // Last timestamp when IK was run
const IK_UPDATE_INTERVAL_MS = 150; // Run IK every 150ms

let lastSuccessfulIKTime = null; // Last time IK was successfully applied (not rejected)

const outlineMap = new WeakMap(); // mesh -> lineSegments

const OUTLINE_COLOR = 0x000000;
const EDGE_THRESHOLD_ANGLE = 15; // degrees; increase to show fewer edges

const outlineMaterial = new THREE.LineBasicMaterial({
  color: OUTLINE_COLOR,
  transparent: true,
  opacity: 1,
});

const wireframeOutlineMaterial = new THREE.LineBasicMaterial({
  color: 0x00ff00,
  transparent: true,
  opacity: 1,
});

// --- LOADING MODELS ---

// Add the outlines for our ghost mesh
function addOutlineForMesh(mesh, material = outlineMaterial, edgeThresholdAngle = EDGE_THRESHOLD_ANGLE) {
  // Extract edges from the mesh where adjacent faces meet at an angle > EDGE_THRESHOLD_ANGLE
  // This creates a cartoon-style outline by only drawing significant edges (not every triangle edge)
  const edgesGeom = new THREE.EdgesGeometry(mesh.geometry, edgeThresholdAngle);
  const lines = new THREE.LineSegments(edgesGeom, material);

  // Make it follow the mesh (including skinning transforms) by parenting it to the mesh
  mesh.add(lines);

  // Push it slightly outward to reduce z-fighting.
  lines.scale.setScalar(1.001);

  // draw lines after the ghost surface
  lines.renderOrder = 10;

  outlineMap.set(mesh, lines);
}

function getMaterial() {
  if (!wireframeMode) {
    return new THREE.MeshStandardMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.15, // CHANGE BACK TO 0.8

      // Stop objects behind the model showing through
      blending: THREE.NoBlending,
    });
  } else {
    return new THREE.MeshBasicMaterial({
      colorWrite: false,
    });
  }
}

function loadModel(modelPath) {
  const loader = new GLTFLoader();
  console.log("Loading model:", modelPath);

  return new Promise((resolve, _reject) => {
    // Check if model exists first
    fetch(modelPath, { method: 'HEAD' })
      .then(response => {
        if (!response.ok) {
          console.warn(`Model not found: ${modelPath} (status: ${response.status})`);
          resolve(false);
          return;
        }

        // Model exists, proceed with loading
        loader.load(
          modelPath,
          (gltf) => {
            model = gltf.scene;
            
            // Scale down the model by 100
            model.scale.set(0.01, 0.01, 0.01);
            
            // Apply custom position offset
            model.position.set(modelPositionOffset.x, modelPositionOffset.y, modelPositionOffset.z);

            // Get material (wireframe or ghost)
            const material = getMaterial();
            material.side = THREE.DoubleSide; // Ensure both sides are rendered

            // Apply materials + collect bones
            model.traverse((child) => {
              console.log("Model child:", child.name, child.type);
              if (child.isMesh) {
                child.material = material;

                // Add outlines (green for wireframe mode, black for ghost mode)
                if (wireframeMode) {
                  addOutlineForMesh(child, wireframeOutlineMaterial);
                } else {
                  addOutlineForMesh(child);
                }
              }

              if (child.isBone) {
                bonesByName.set(child.name, child);
              } else if (child.name) {
                // Store all named objects as potential joints (for non-skeletal models)
                objectsByName.set(child.name, child);
              }
            });

            scene.add(model);
            
            resolve(true);
            console.log("Loaded model:", model);

            // Create IK debug rig
            debugRig = createIKDebugRig(scene, model, {
              elbowPointName: "Elbow_Flex",
              wristPointName: "Wrist_Flex",
              eePointName: "EndEffector",
              jointNames: ["Base_Rotation","Shoulder_Lift","Elbow_Flex","Wrist_Flex"],
            });
          },
          undefined,
          (error) => {
            console.error(`Error loading model: ${modelPath}`, error);
            resolve(false);
          }
        );
      })
      .catch(error => {
        console.warn(`Failed to check model existence: ${modelPath}`, error);
        resolve(false);
      });
  });
}

// --- CONFIGURE LOADED MODELS ---

// Set all given joint angles
function setJointAngles(jointAngles, units = "degrees") {
  for (const jointName in jointAngles) {
    const axes = jointAngles[jointName];
    for (const axis in axes) {
      setRotation(jointName, axis, axes[axis], units);
      console.log(`Set joint ${jointName} axis ${axis} to ${axes[axis]} ${units}`);
    }
  }
  
  // Render the scene after updating joint angles
  renderer.render(scene, camera);
}

// Set rotation of a named bone
function setRotation(jointName, axis, value, units = "degrees") {
  // Try to find bone first, then fall back to regular object
  const bone = bonesByName.get(jointName) || objectsByName.get(jointName);
  if (!bone) {
    console.warn("No bone or object found for jointName:", jointName, "Known bones:", [...bonesByName.keys()], "Known objects:", [...objectsByName.keys()]);
    return;
  }

  if (axis !== "x" && axis !== "y" && axis !== "z") return;

  // Convert to radians only if units are in degrees
  const valueRad = units === "radians" ? value : THREE.MathUtils.degToRad(value);
  bone.rotation[axis] = valueRad;
  bone.updateMatrixWorld(true);
}

// Set render mode to wireframe
function setRenderMode(wireframe = false) {
  if(wireframe)
  {
    console.log("Setting render mode to wireframe");
    wireframeMode = true;
  }
}

// Set model position offset (call before loadModel)
function setModelPosition(x = 0, y = 0, z = 0) {
  modelPositionOffset = { x, y, z };
  
  // If model already loaded, update its position
  if (model) {
    model.position.set(x, y, z);
  }
}

// --- SKELETON LINES (OpenPose) ---

// Create the three skeleton lines with colors
function createSkeletonLines() {
  // Remove existing lines if they exist
  if (shoulderToElbowLine) scene.remove(shoulderToElbowLine);
  if (elbowToWristLine) scene.remove(elbowToWristLine);
  if (wristToHandLine) scene.remove(wristToHandLine);

  // Create line materials
  const greenMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 3 });
  const redMaterial = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 3 });
  const blueMaterial = new THREE.LineBasicMaterial({ color: 0x0000ff, linewidth: 3 });

  // Create geometries with two points each
  const geometry1 = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, 0)
  ]);
  const geometry2 = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, 0)
  ]);
  const geometry3 = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(0, 0, 0)
  ]);

  // Create lines
  shoulderToElbowLine = new THREE.Line(geometry1, greenMaterial);
  elbowToWristLine = new THREE.Line(geometry2, redMaterial);
  wristToHandLine = new THREE.Line(geometry3, blueMaterial);

  // Set render order to ensure lines render on top of the model
  shoulderToElbowLine.renderOrder = 100;
  elbowToWristLine.renderOrder = 100;
  wristToHandLine.renderOrder = 100;

  // Disable depth test so lines always render on top
  greenMaterial.depthTest = false;
  redMaterial.depthTest = false;
  blueMaterial.depthTest = false;

  // Add to scene
  scene.add(shoulderToElbowLine);
  scene.add(elbowToWristLine);
  scene.add(wristToHandLine);

  console.log("Skeleton lines created");


}

// Update skeleton lines with new keypoint data
// keypointData should be: { shoulder: {x, y, z}, elbow: {x, y, z}, wrist: {x, y, z}, hand: {x, y, z} }
// timestamp: ISO 8601 timestamp string from the message
function updateSkeletonLines(keypointData, timestamp = null) {
  // Create lines if they don't exist
  if (!shoulderToElbowLine || !elbowToWristLine || !wristToHandLine) {
    createSkeletonLines();
  }

  if (!keypointData.shoulder) return;

  // Get the Base object from the scene
  const baseObject = bonesByName.get("Base") || objectsByName.get("Base");
  if (!baseObject) {
    console.warn("Base object not found in scene");
    return;
  }

  // Get Base world position
  const baseWorldPos = new THREE.Vector3();
  baseObject.getWorldPosition(baseWorldPos);

  baseWorldPos.y += 0.5; // Offset base position 0.5 units higher

  // Calculate offset to move shoulder to base position
  const offsetX = keypointData.shoulder.x - baseWorldPos.x;
  const offsetY = keypointData.shoulder.y - baseWorldPos.y;
  const offsetZ = keypointData.shoulder.z - baseWorldPos.z;

  // Update base to elbow (green) - starts from base
  if (keypointData.elbow) {
    const positions1 = shoulderToElbowLine.geometry.attributes.position;
    positions1.setXYZ(0, baseWorldPos.x, baseWorldPos.y, baseWorldPos.z);
    positions1.setXYZ(1, 
      keypointData.elbow.x - offsetX, 
      keypointData.elbow.y - offsetY, 
      keypointData.elbow.z - offsetZ
    );
    positions1.needsUpdate = true;
  }

  // Update elbow to wrist (red) - starts from elbow with offset applied
  if (keypointData.elbow && keypointData.wrist) {
    const positions2 = elbowToWristLine.geometry.attributes.position;
    positions2.setXYZ(0, 
      keypointData.elbow.x - offsetX, 
      keypointData.elbow.y - offsetY, 
      keypointData.elbow.z - offsetZ
    );
    positions2.setXYZ(1, 
      keypointData.wrist.x - offsetX, 
      keypointData.wrist.y - offsetY, 
      keypointData.wrist.z - offsetZ
    );
    positions2.needsUpdate = true;
  }

  // Update wrist to hand (blue) - starts from wrist with offset applied
  if (keypointData.wrist && keypointData.hand) {
    const positions3 = wristToHandLine.geometry.attributes.position;
    positions3.setXYZ(0, 
      keypointData.wrist.x - offsetX, 
      keypointData.wrist.y - offsetY, 
      keypointData.wrist.z - offsetZ
    );
    positions3.setXYZ(1, 
      keypointData.hand.x - offsetX, 
      keypointData.hand.y - offsetY, 
      keypointData.hand.z - offsetZ
    );
    positions3.needsUpdate = true;
  }

  // Call IK solver with offset-adjusted targets (only if enough time has passed)
  let shouldRunIK = false;
  let timeSinceLastSuccessfulIK = 0;
  
  if (timestamp) {
    const currentTime = new Date(timestamp).getTime();
    
    // Calculate time since last successful IK (not just pose update)
    if (lastSuccessfulIKTime) {
      timeSinceLastSuccessfulIK = currentTime - lastSuccessfulIKTime;
    }
    
    // Check if we should run IK based on interval OR if there's been a long gap
    const timeSinceLastIK = lastIKTimestamp ? currentTime - lastIKTimestamp : Infinity;
    
    if (timeSinceLastIK >= IK_UPDATE_INTERVAL_MS) {
      // Normal interval check
      shouldRunIK = true;
      lastIKTimestamp = currentTime;
    }
  } else {
    // Fallback: always run if no timestamp provided
    shouldRunIK = true;
  }

  if (shouldRunIK && keypointData.elbow && keypointData.wrist && keypointData.hand) {
    // Adjust IK parameters based on time since last successful IK
    let ikOptions = {
      elbowPointName: "Elbow_Flex",
      wristPointName: "Wrist_Flex",
      eePointName: "EndEffector",
    };
    
    // If it's been more than 500ms since last successful IK, increase iterations
    if (timeSinceLastSuccessfulIK > 500) {
      ikOptions.iterations = 60; // More iterations for big jumps
      ikOptions.tolerance = 0.01; // Slightly relaxed tolerance
      console.log(`Long gap since successful IK (${timeSinceLastSuccessfulIK}ms), using ${ikOptions.iterations} iterations`);
    } else if (timeSinceLastSuccessfulIK > 200) {
      ikOptions.iterations = 50; // Moderate increase
    }
    
    goToTargets({
      elbow: {
        x: keypointData.elbow.x - offsetX,
        y: keypointData.elbow.y - offsetY,
        z: keypointData.elbow.z - offsetZ
      },
      wrist: {
        x: keypointData.wrist.x - offsetX,
        y: keypointData.wrist.y - offsetY,
        z: keypointData.wrist.z - offsetZ
      },
      hand: {
        x: keypointData.hand.x - offsetX,
        y: keypointData.hand.y - offsetY,
        z: keypointData.hand.z - offsetZ
      }
    }, ikOptions, timestamp);
  }

  // Render the scene
  renderer.render(scene, camera);
}

// --- CAMERA AND LIGHTING ---

// Lighting setup
const light = new THREE.DirectionalLight(0xffffff, 3);
light.position.set(-1, 2, 4);
scene.add(light);

// Configure lighting (color in hex, intensity, and position)
function setLighting(color = 0xffffff, intensity = 3, x = -1, y = 2, z = 4) {
  light.color.setHex(color);
  light.intensity = intensity;
  light.position.set(x, y, z);
}

// Set camera Z position
function setCameraPosition(zPos = 1) {
  camera.position.z = zPos;
}

// --- INVERSE KINEMATICS SOLVER ---

const MAX_IK_ERROR = 0.6; // Maximum acceptable error in meters (60cm)

// Sets arm joints to reach target position
function goToTargets(targets, ikOptions = {}, timestamp = null){
  console.log("Solving IK to reach target:", targets);

  // Save current joint state in case we need to reject the IK solution
  const savedJointAngles = {};
  const jointNames = ["Base_Rotation", "Shoulder_Lift", "Elbow_Flex", "Wrist_Flex"];
  for (const name of jointNames) {
    const obj = objectsByName.get(name);
    if (obj) {
      savedJointAngles[name] = {
        x: obj.rotation.x,
        y: obj.rotation.y,
        z: obj.rotation.z
      };
    }
  }

  const my_targets = {
  elbowTargetWorld: new THREE.Vector3( targets.elbow.x, targets.elbow.y, targets.elbow.z),
  wristTargetWorld: new THREE.Vector3( targets.wrist.x, targets.wrist.y, targets.wrist.z),
  eeTargetWorld: new THREE.Vector3( targets.hand.x, targets.hand.y, targets.hand.z), // EE goal (world, mm)
  }

  const cmd = solveCoupled3TargetIKToJson(model, {
    targetA: my_targets.elbowTargetWorld,
    targetB: my_targets.wristTargetWorld,
    targetC: my_targets.eeTargetWorld,
  }, {
    elbowPointName: "Elbow_Flex",
    wristPointName: "Wrist_Flex",
    eePointName: "EndEffector",
    ...ikOptions, // Merge in any custom options (iterations, tolerance, etc.)
  });

  // Validate the IK solution by checking final errors
  model.updateMatrixWorld(true);
  
  const elbowObj = objectsByName.get("Elbow_Flex");
  const wristObj = objectsByName.get("Wrist_Flex");
  const eeObj = objectsByName.get("EndEffector");
  
  if (elbowObj && wristObj && eeObj) {
    const elbowPos = new THREE.Vector3();
    const wristPos = new THREE.Vector3();
    const eePos = new THREE.Vector3();
    
    elbowObj.getWorldPosition(elbowPos);
    wristObj.getWorldPosition(wristPos);
    eeObj.getWorldPosition(eePos);
    
    const elbowError = elbowPos.distanceTo(my_targets.elbowTargetWorld);
    const wristError = wristPos.distanceTo(my_targets.wristTargetWorld);
    const eeError = eePos.distanceTo(my_targets.eeTargetWorld);
    
    const maxError = Math.max(elbowError, wristError, eeError);
    
    if (maxError > MAX_IK_ERROR) {
      console.warn(`IK solution rejected: max error ${maxError.toFixed(3)}m exceeds threshold ${MAX_IK_ERROR}m`);
      console.warn(`  Elbow error: ${elbowError.toFixed(3)}m, Wrist error: ${wristError.toFixed(3)}m, EE error: ${eeError.toFixed(3)}m`);
      
      // Restore previous joint angles
      for (const name of jointNames) {
        const obj = objectsByName.get(name);
        if (obj && savedJointAngles[name]) {
          obj.rotation.x = savedJointAngles[name].x;
          obj.rotation.y = savedJointAngles[name].y;
          obj.rotation.z = savedJointAngles[name].z;
        }
      }
      model.updateMatrixWorld(true);
      renderer.render(scene, camera);
      return; // Don't apply the solution
    }
    
    console.log(`IK solution accepted: max error ${maxError.toFixed(3)}m (elbow: ${elbowError.toFixed(3)}m, wrist: ${wristError.toFixed(3)}m, ee: ${eeError.toFixed(3)}m)`);
    
    // Update successful IK time
    if (timestamp) {
      lastSuccessfulIKTime = new Date(timestamp).getTime();
    }
  }

  console.log(cmd);
  setJointAngles(cmd.joints, cmd.units);

  // Update IK debug rig
  updateIKDebugRig(debugRig, {
    targetA: my_targets.elbowTargetWorld,
    targetB: my_targets.wristTargetWorld,
    targetC: my_targets.eeTargetWorld,
  });
  
}

// Export what your HTML needs
export { loadModel, setJointAngles, setRenderMode, setCameraPosition, setModelPosition, setLighting, updateSkeletonLines, goToTargets };
