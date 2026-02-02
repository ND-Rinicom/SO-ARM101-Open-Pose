import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { calculateJointAnglesFromPose } from "./geometric_solver.js";
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

// Skeleton spheres for keypoints (elbow, wrist, hand)
let elbowSphere = null;  // Green
let wristSphere = null;  // Red
let handSphere = null;   // Blue

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

  // Remove existing spheres if they exist
  if (elbowSphere) scene.remove(elbowSphere);
  if (wristSphere) scene.remove(wristSphere);
  if (handSphere) scene.remove(handSphere);

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

  // Create spheres for keypoints (radius 0.012 meters)
  const sphereGeometry = new THREE.SphereGeometry(0.012, 30, 30);
  
  const greenSphereMaterial = new THREE.MeshBasicMaterial({ 
    color: 0x00ff00, 
    depthTest: false 
  });
  const redSphereMaterial = new THREE.MeshBasicMaterial({ 
    color: 0xff0000, 
    depthTest: false 
  });
  const blueSphereMaterial = new THREE.MeshBasicMaterial({ 
    color: 0x0000ff, 
    depthTest: false 
  });

  elbowSphere = new THREE.Mesh(sphereGeometry, greenSphereMaterial);
  wristSphere = new THREE.Mesh(sphereGeometry, redSphereMaterial);
  handSphere = new THREE.Mesh(sphereGeometry, blueSphereMaterial);

  // Set render order to ensure spheres render on top
  elbowSphere.renderOrder = 999;
  wristSphere.renderOrder = 999;
  handSphere.renderOrder = 999;

  // Add spheres to scene
  scene.add(elbowSphere);
  scene.add(wristSphere);
  scene.add(handSphere);

  console.log("Skeleton lines and spheres created");
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

  // Update sphere positions
  if (keypointData.elbow) {
    elbowSphere.position.set(
      keypointData.elbow.x - offsetX,
      keypointData.elbow.y - offsetY,
      keypointData.elbow.z - offsetZ
    );
  }
  if (keypointData.wrist) {
    wristSphere.position.set(
      keypointData.wrist.x - offsetX,
      keypointData.wrist.y - offsetY,
      keypointData.wrist.z - offsetZ
    );
  }
  if (keypointData.hand) {
    handSphere.position.set(
      keypointData.hand.x - offsetX,
      keypointData.hand.y - offsetY,
      keypointData.hand.z - offsetZ
    );
  }

  // Calculate joint angles directly from pose (no IK needed)
  if (keypointData.elbow && keypointData.wrist && keypointData.hand) {
    calculateAnglesFromPose({
      shoulder: new THREE.Vector3(baseWorldPos.x, baseWorldPos.y, baseWorldPos.z),
      elbow: new THREE.Vector3(
        keypointData.elbow.x - offsetX,
        keypointData.elbow.y - offsetY,
        keypointData.elbow.z - offsetZ
      ),
      wrist: new THREE.Vector3(
        keypointData.wrist.x - offsetX,
        keypointData.wrist.y - offsetY,
        keypointData.wrist.z - offsetZ
      ),
      hand: new THREE.Vector3(
        keypointData.hand.x - offsetX,
        keypointData.hand.y - offsetY,
        keypointData.hand.z - offsetZ
      )
    });
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

// --- GEOMETRIC ANGLE CALCULATION ---

// Calculate joint angles directly from pose keypoints (no IK needed)
function calculateAnglesFromPose(keypoints) {
  console.log("Calculating joint angles from pose:", keypoints);

  const cmd = calculateJointAnglesFromPose(keypoints);

  console.log("Calculated joint angles command:", cmd);

  // If calculation failed (invalid keypoints), don't apply
  if (!cmd) {
    console.warn("Angle calculation failed, keeping previous pose");
    return;
  }

  console.log(cmd);
  setJointAngles(cmd.params.joints, cmd.params.units);

  // Update debug rig with the keypoints
  updateIKDebugRig(debugRig, {
    targetA: keypoints.elbow,
    targetB: keypoints.wrist,
    targetC: keypoints.hand,
  });
}

// Export what your HTML needs
export { loadModel, setJointAngles, setRenderMode, setCameraPosition, setModelPosition, setLighting, updateSkeletonLines };
