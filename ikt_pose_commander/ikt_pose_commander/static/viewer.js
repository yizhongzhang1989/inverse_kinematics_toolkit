"use strict";
// 3D viewer for the ikt_pose_commander dashboard.
//
// Renders the robot from /robot_description meshes at the live /joint_states
// configuration (per-link FK computed server-side, mirrored from the
// ikt_inverse_kinematics dashboard) PLUS a triad + sphere at the *commanded
// target pose* (whatever is currently on <ns>/target_pose — the dashboard's own
// jog/send OR the spacemouse_servo teleop bridge). This is the visual check for
// "is the SpaceMouse sending the right command?".
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const $ = (id) => document.getElementById(id);

// ---- scene --------------------------------------------------------------
const canvas = $("viewer");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);
const vw = () => canvas.clientWidth || (innerWidth - 360);
const vh = () => canvas.clientHeight || innerHeight;
const camera = new THREE.PerspectiveCamera(50, vw() / vh(), 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(1.4, -1.4, 1.1);
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(vw(), vh(), false);
renderer.setPixelRatio(devicePixelRatio);
renderer.outputColorSpace = THREE.SRGBColorSpace;
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.1;
controls.target.set(0, 0, 0.4); controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x404040, 0.85));
const d1 = new THREE.DirectionalLight(0xffffff, 1.2); d1.position.set(3, 5, 4); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.4); d2.position.set(-2, 3, -1); scene.add(d2);
const grid = new THREE.GridHelper(3, 30, 0x445, 0x334); grid.rotation.x = Math.PI / 2; scene.add(grid);
scene.add(new THREE.AxesHelper(0.25));   // base-frame triad at the origin

// ---- target marker (sphere + triad), driven by the live target ----------
const targetGroup = new THREE.Group();
targetGroup.visible = false; targetGroup.matrixAutoUpdate = false; scene.add(targetGroup);
const targetBall = new THREE.Mesh(new THREE.SphereGeometry(0.02, 16, 16),
  new THREE.MeshBasicMaterial({ color: 0x39d353 }));
targetGroup.add(targetBall);
targetGroup.add(new THREE.AxesHelper(0.18));   // larger than base: the target's orientation

const solidMat = new THREE.MeshStandardMaterial({ color: 0x9fb4c4, metalness: 0.25, roughness: 0.6 });
const stlLoader = new STLLoader();
const geomCache = {};   // url -> {geom, waiting:[cb]}
const meshItems = {};   // key(link#i) -> {link, local, solid}
let didFit = false;

function getGeom(url, cb) {
  const c = geomCache[url];
  if (c && c.geom) { cb(c.geom); return; }
  if (c) { c.waiting.push(cb); return; }
  geomCache[url] = { geom: null, waiting: [cb] };
  stlLoader.load(url, (g) => {
    g.computeVertexNormals();
    geomCache[url].geom = g;
    geomCache[url].waiting.forEach((f) => f(g));
    geomCache[url].waiting = [];
  }, undefined, () => { /* load error: skeleton still shows */ });
}
function localMatrix(xyz, rpy, scale) {
  const m = new THREE.Matrix4();
  m.makeRotationFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
  m.setPosition(xyz[0], xyz[1], xyz[2]);
  if (scale) m.scale(new THREE.Vector3(scale[0], scale[1], scale[2]));
  return m;
}
function rosMat(a) {
  return new THREE.Matrix4().set(
    a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3],
    a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]);
}
function ensureMeshes(visuals) {
  visuals.forEach((v, i) => {
    const key = v.link + "#" + i;
    if (meshItems[key] !== undefined) return;
    const item = { link: v.link, local: localMatrix(v.xyz, v.rpy, v.scale), solid: null };
    meshItems[key] = item;
    getGeom(v.url, (geom) => {
      const s = new THREE.Mesh(geom, solidMat); s.matrixAutoUpdate = false;
      item.solid = s; scene.add(s);
    });
  });
}
function placeCurrent(linkTf) {
  for (const key in meshItems) {
    const it = meshItems[key]; if (!it.solid) continue;
    const lm = linkTf[it.link];
    if (!lm) { it.solid.visible = false; continue; }
    it.solid.visible = true;
    it.solid.matrix.copy(rosMat(lm).multiply(it.local));
  }
}

// skeleton fallback (no meshes)
let skelPts = null;
function updateSkeleton(linkTf) {
  if (Object.keys(meshItems).length) return;
  const pts = [];
  for (const k in linkTf) { const m = linkTf[k]; pts.push(m[0][3], m[1][3], m[2][3]); }
  if (!skelPts) {
    skelPts = new THREE.Points(new THREE.BufferGeometry(),
      new THREE.PointsMaterial({ color: 0x34c3ff, size: 0.03 }));
    scene.add(skelPts);
  }
  skelPts.geometry.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  skelPts.geometry.computeBoundingSphere();
}

function frameBox(linkTf) {
  const box = new THREE.Box3(); let any = false;
  for (const k in (linkTf || {})) {
    box.expandByPoint(new THREE.Vector3(linkTf[k][0][3], linkTf[k][1][3], linkTf[k][2][3])); any = true;
  }
  return any ? box : null;
}
function resetView(linkTf) {
  const box = frameBox(linkTf || window.__lastLinkTf);
  if (!box) return;
  const c = box.getCenter(new THREE.Vector3());
  const sz = box.getSize(new THREE.Vector3()).length() || 1.0;
  controls.target.copy(c);
  camera.position.set(c.x + sz, c.y - sz, c.z + sz * 0.7); controls.update();
}
function fitView(linkTf) {
  if (didFit) return;
  if (!frameBox(linkTf)) return;
  resetView(linkTf); didFit = true;
}

// ---- live target --------------------------------------------------------
function updateTarget(t) {
  const read = $("tgt-read");
  if (!t || !Array.isArray(t.xyz)) {
    targetGroup.visible = false;
    if (read) { read.textContent = "no target yet"; read.className = "muted"; }
    return;
  }
  const [x, y, z] = t.xyz, q = t.quat;   // quat is [w, x, y, z]
  const m = new THREE.Matrix4().makeRotationFromQuaternion(
    new THREE.Quaternion(q[1], q[2], q[3], q[0]));
  m.setPosition(x, y, z);
  targetGroup.matrix.copy(m); targetGroup.matrixWorldNeedsUpdate = true;
  targetGroup.visible = true;
  targetBall.material.color.setHex(t.fresh ? 0x39d353 : 0x8a6d1f);
  if (read) {
    const xf = t.transformed_from ? ` ⟵${t.transformed_from}` : "";
    read.textContent = `target [${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}]`
      + (t.fresh ? "  · live" : `  · stale ${t.age}s`) + xf;
    read.className = t.fresh ? "tgt-live" : "tgt-stale";
  }
}

// ---- poll ---------------------------------------------------------------
async function poll() {
  let s;
  try { s = await (await fetch("/api/state")).json(); } catch { return; }
  if (s.has_model_viz) {
    const ld = $("loading"); if (ld) ld.style.display = "none";
    if (s.has_meshes) { ensureMeshes(s.visuals || []); placeCurrent(s.link_tf || {}); }
    else updateSkeleton(s.link_tf || {});
    fitView(s.link_tf);
    window.__lastLinkTf = s.link_tf;
  }
  updateTarget(s.target);
}
if ($("fit")) $("fit").onclick = () => resetView();
poll(); setInterval(poll, 120);

// ---- render loop --------------------------------------------------------
function resizeToDisplay() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const pr = renderer.getPixelRatio();
  if (canvas.width !== Math.round(w * pr) || canvas.height !== Math.round(h * pr)) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
}
(function animate() {
  requestAnimationFrame(animate);
  resizeToDisplay(); controls.update(); renderer.render(scene, camera);
})();
