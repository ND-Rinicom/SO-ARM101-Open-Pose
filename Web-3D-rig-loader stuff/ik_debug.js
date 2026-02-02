import * as THREE from "three";

// ===== DEBUG VISUALS =====

// Helper function to get world position from an object
function getWorldPos(obj) {
  if (!obj) {
    console.warn('getWorldPos called with undefined object');
    return new THREE.Vector3();
  }
  const pos = new THREE.Vector3();
  obj.getWorldPosition(pos);
  return pos;
}

function makeDebugSphere(radius = 0.01, color = 0xffffff) { // meters
  const g = new THREE.SphereGeometry(radius, 30, 30);
  const m = new THREE.MeshBasicMaterial({ 
    color: color,
    depthTest: false // Render on top of everything
  });
  const mesh = new THREE.Mesh(g, m);
  mesh.renderOrder = 999; // Ensure it renders last (on top)
  return mesh;
}

function makeDebugLine() {
  const geom = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(), new THREE.Vector3()
  ]);
  const mat = new THREE.LineBasicMaterial();
  return new THREE.Line(geom, mat);
}

function updateLine(line, a, b) {
  const arr = line.geometry.attributes.position.array;
  arr[0] = a.x; arr[1] = a.y; arr[2] = a.z;
  arr[3] = b.x; arr[4] = b.y; arr[5] = b.z;
  line.geometry.attributes.position.needsUpdate = true;
  line.geometry.computeBoundingSphere();
}

// Draw world-axis arrows at a joint (x=red-ish, y=green-ish, z=blue-ish but default material)
function makeAxisHelpers(length = 0.06) {
  return {
    x: new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(), length),
    y: new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(), length),
    z: new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(), length),
  };
}

function updateAxisHelpers(axisHelpers, origin, qWorld, length = 0.06) {
  const x = new THREE.Vector3(1,0,0).applyQuaternion(qWorld).normalize();
  const y = new THREE.Vector3(0,1,0).applyQuaternion(qWorld).normalize();
  const z = new THREE.Vector3(0,0,1).applyQuaternion(qWorld).normalize();

  axisHelpers.x.position.copy(origin); axisHelpers.x.setDirection(x); axisHelpers.x.setLength(length);
  axisHelpers.y.position.copy(origin); axisHelpers.y.setDirection(y); axisHelpers.y.setLength(length);
  axisHelpers.z.position.copy(origin); axisHelpers.z.setDirection(z); axisHelpers.z.setLength(length);
}

// Create a debug rig once
function createIKDebugRig(scene, root, names, opts = {}) {
  const {
    pointRadius = 0.012,     // meters
    axisLength = 0.08,
  } = opts;

  const rig = {
    // error lines
    elbowLine: makeDebugLine(),
    wristLine: makeDebugLine(),
    eeLine: makeDebugLine(),

    // axis helpers per joint
    axes: {},
  };

  // add to scene (world space)
  scene.add(
    rig.elbowLine, rig.wristLine, rig.eeLine,
  );

  // axis helpers at each joint name you care about
  for (const jName of names.jointNames) {
    const h = makeAxisHelpers(axisLength);
    rig.axes[jName] = h;
    scene.add(h.x, h.y, h.z);
  }

  // store names for updates
  rig.names = names;
  rig.root = root;

  return rig;
}

// Update the debug rig each frame (after you run IK)
function updateIKDebugRig(rig, targets) {
  const root = rig.root;
  root.updateMatrixWorld(true);

  const elbowObj = root.getObjectByName(rig.names.elbowPointName);
  const wristObj = root.getObjectByName(rig.names.wristPointName);
  const eeObj    = root.getObjectByName(rig.names.eePointName);

  const pElbow = getWorldPos(elbowObj);
  const pWrist = getWorldPos(wristObj);
  const pEE    = getWorldPos(eeObj);

  updateLine(rig.elbowLine, pElbow, targets.targetA);
  updateLine(rig.wristLine, pWrist, targets.targetB);
  updateLine(rig.eeLine,    pEE,    targets.targetC);

  // update joint axes visuals
  for (const jName of rig.names.jointNames) {
    const jObj = root.getObjectByName(jName);
    if (!jObj) continue;
    const o = getWorldPos(jObj);
    const q = new THREE.Quaternion(); jObj.getWorldQuaternion(q);
    updateAxisHelpers(rig.axes[jName], o, q);
  }
}

export { createIKDebugRig, updateIKDebugRig };