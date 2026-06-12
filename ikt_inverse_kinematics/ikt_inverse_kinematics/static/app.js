"use strict";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;

// ---- scene --------------------------------------------------------------
const canvas = $("viewer");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);
function vw() { return canvas.clientWidth || (innerWidth - 340); }
function vh() { return canvas.clientHeight || innerHeight; }
const camera = new THREE.PerspectiveCamera(50, vw() / vh(), 0.01, 100);
camera.up.set(0, 0, 1);
camera.position.set(1.4, -1.4, 1.1);
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(vw(), vh(), false);
renderer.setPixelRatio(devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.1;
controls.target.set(0, 0, 0.4); controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.65));
scene.add(new THREE.HemisphereLight(0xb0d4f1, 0x404040, 0.85));
const d1 = new THREE.DirectionalLight(0xffffff, 1.3);
d1.position.set(3, 5, 4); d1.castShadow = true; d1.shadow.mapSize.set(2048, 2048); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.45); d2.position.set(-2, 3, -1); scene.add(d2);
const grid = new THREE.GridHelper(3, 30, 0x445, 0x334); grid.rotation.x = Math.PI / 2; scene.add(grid);
scene.add(new THREE.AxesHelper(0.25));
const ground = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), new THREE.ShadowMaterial({ opacity: 0.28 }));
ground.receiveShadow = true; scene.add(ground);

// target marker (sphere + triad) and operated-frame triad
const targetGroup = new THREE.Group(); targetGroup.visible = false; scene.add(targetGroup);
const targetBall = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 16),
  new THREE.MeshBasicMaterial({ color: 0xffd23f }));
targetGroup.add(targetBall);
const targetAxes = new THREE.AxesHelper(0.13); targetGroup.add(targetAxes);
targetGroup.matrixAutoUpdate = false;

const solidMat = new THREE.MeshStandardMaterial({ color: 0x9fb4c4, metalness: 0.25, roughness: 0.6 });
const ghostMat = new THREE.MeshStandardMaterial({ color: 0x2f81f7, transparent: true, opacity: 0.32,
  metalness: 0.0, roughness: 0.9, depthWrite: false });

const stlLoader = new STLLoader();
const geomCache = {};   // url -> {geom, waiting:[cb]}  (load each STL once)
const meshItems = {};   // key(link#i) -> {link, local, solid, ghost}
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
    const item = { link: v.link, local: localMatrix(v.xyz, v.rpy, v.scale), solid: null, ghost: null };
    meshItems[key] = item;
    // each visual gets its OWN mesh instances (two arms share STL files, so we
    // must NOT key by url — only the geometry is cached/reused per url).
    getGeom(v.url, (geom) => {
      const s = new THREE.Mesh(geom, solidMat); s.castShadow = true; s.matrixAutoUpdate = false;
      const g = new THREE.Mesh(geom, ghostMat); g.matrixAutoUpdate = false; g.visible = false; g.renderOrder = 2;
      item.solid = s; item.ghost = g; scene.add(s); scene.add(g);
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
function placeGhost(linkTf) {
  const show = !!linkTf && $("ghost").checked;
  for (const key in meshItems) {
    const it = meshItems[key]; if (!it.ghost) continue;
    if (!show) { it.ghost.visible = false; continue; }
    const lm = linkTf[it.link];
    if (!lm) { it.ghost.visible = false; continue; }
    it.ghost.visible = true;
    it.ghost.matrix.copy(rosMat(lm).multiply(it.local));
  }
}

// skeleton fallback (no meshes)
let skelLine = null;
function updateSkeleton(linkTf) {
  if (Object.keys(meshItems).length) return;  // have meshes
  const pts = [];
  for (const k in linkTf) { const m = linkTf[k]; pts.push(m[0][3], m[1][3], m[2][3]); }
  if (!skelLine) {
    skelLine = new THREE.Points(new THREE.BufferGeometry(),
      new THREE.PointsMaterial({ color: 0x34c3ff, size: 0.03 }));
    scene.add(skelLine);
  }
  skelLine.geometry.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  skelLine.geometry.computeBoundingSphere();
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
  resetView(linkTf);
  didFit = true;
}

// ---- target marker ------------------------------------------------------
function updateTargetMarker() {
  if (!$("liveMarker").checked) { targetGroup.visible = false; return; }
  const x = +$("tx").value, y = +$("ty").value, z = +$("tz").value;
  if ([x, y, z].some(Number.isNaN)) { targetGroup.visible = false; return; }
  const q = new THREE.Quaternion().setFromEuler(
    new THREE.Euler((+$("rr").value || 0) * DEG, (+$("rp").value || 0) * DEG, (+$("ryaw").value || 0) * DEG, "ZYX"));
  const m = new THREE.Matrix4().makeRotationFromQuaternion(q); m.setPosition(x, y, z);
  targetGroup.matrix.copy(m); targetGroup.matrixWorldNeedsUpdate = true; targetGroup.visible = true;
}

// quat (wxyz) for the request, from rpy inputs
function targetQuatWXYZ() {
  const q = new THREE.Quaternion().setFromEuler(
    new THREE.Euler((+$("rr").value || 0) * DEG, (+$("rp").value || 0) * DEG, (+$("ryaw").value || 0) * DEG, "ZYX"));
  return [q.w, q.x, q.y, q.z];
}

// ---- control panel state ------------------------------------------------
const STIFF = ["x", "y", "z", "rx", "ry", "rz"];
let stiffVals = [1, 1, 1, 1, 1, 1];
let lastLinks = [], lastJoints = [];

function buildStiff() {
  const host = $("stiff"); host.innerHTML = "";
  STIFF.forEach((lab, i) => {
    const cell = document.createElement("div"); cell.className = "scell";
    cell.innerHTML = `${lab}<span class="sv" id="sv${i}">${stiffVals[i].toFixed(2)}</span>` +
      `<input type="range" min="0" max="1" step="0.05" value="${stiffVals[i]}" id="sl${i}">`;
    host.appendChild(cell);
  });
  STIFF.forEach((_, i) => {
    $("sl" + i).oninput = (e) => { stiffVals[i] = +e.target.value; $("sv" + i).textContent = stiffVals[i].toFixed(2); };
  });
}
function applyPreset(p) {
  if (p === "pose") stiffVals = [1, 1, 1, 1, 1, 1];
  else if (p === "point") stiffVals = [1, 1, 1, 0, 0, 0];
  else if (p === "posyaw") stiffVals = [1, 1, 1, 0, 0, 1];
  buildStiff();
}

function prefixOf(name) {
  // strip a trailing _Link<n> / _link<n> / _joint<n> to get the arm prefix
  const m = name.match(/^(.*?)_(?:Link|link|joint)\d+$/);
  if (m) return m[1];
  const i = name.indexOf("_base_link");
  return i > 0 ? name.slice(0, i) : "";
}

function fillDropdowns(s) {
  const links = s.links || [], joints = s.joints || [];
  if (JSON.stringify(links) === JSON.stringify(lastLinks) &&
      JSON.stringify(joints) === JSON.stringify(lastJoints)) return;
  lastLinks = links; lastJoints = joints;
  const opt = (v) => { const o = document.createElement("option"); o.value = v; o.textContent = v; return o; };
  for (const sel of [$("frame"), $("vfParent"), $("relA"), $("relB")]) {
    const cur = sel.value; sel.innerHTML = "";
    links.forEach((l) => sel.appendChild(opt(l)));
    if (cur && links.includes(cur)) sel.value = cur;
  }
  // active-joints: all + per-prefix groups
  const groups = new Set();
  joints.forEach((j) => { const p = prefixOf(j); if (p) groups.add(p); });
  const act = $("active"); const curA = act.value;
  act.innerHTML = '<option value="">all joints</option>';
  [...groups].sort().forEach((g) => act.appendChild(opt(g)));
  if (curA) act.value = curA;
  // default operated frame: a tip link (prefer *Link7), not the world root
  const cur = $("frame").value;
  if ((!cur || cur === "world" || cur === "base_link") && links.length) {
    const tip = links.find((l) => /Link7$/.test(l)) || links[links.length - 1];
    $("frame").value = tip;
    onFrameChange();
  }
  syncVfOption();
}

function activeJointsForRequest() {
  const v = $("active").value;
  if (!v) return null;
  return lastJoints.filter((j) => prefixOf(j) === v);
}

// Keep a synthetic option for the virtual tool frame in the frame dropdown so
// the R2 tool frame can actually be commanded (selected as the operated frame).
function syncVfOption() {
  const name = $("vfName").value.trim();
  const sel = $("frame");
  [...sel.options].forEach((o) => { if (o.dataset.vf) o.remove(); });
  if (name && !(lastLinks || []).includes(name)) {
    const o = document.createElement("option");
    o.value = name; o.textContent = name + "  (tool)"; o.dataset.vf = "1";
    sel.appendChild(o);
  }
}

function onFrameChange() {
  // auto-pick the matching active-joint group for the chosen frame; a tool
  // frame inherits its parent link's arm.
  let f = $("frame").value;
  const vfName = $("vfName").value.trim();
  if (vfName && f === vfName) f = $("vfParent").value;
  const p = prefixOf(f);
  if (p) { const opt = [...$("active").options].find((o) => o.value === p); if (opt) $("active").value = p; }
}

// ---- polling current state ---------------------------------------------
let haveModel = false;
async function poll() {
  let s;
  try { s = await (await fetch("/api/state")).json(); }
  catch { $("conn").textContent = "disconnected"; $("conn").className = "pill pill-bad"; return; }
  $("conn").textContent = "connected"; $("conn").className = "pill pill-good";
  if (!s.has_model) { $("modelInfo").textContent = "no /robot_description"; return; }
  haveModel = true;
  $("loading").style.display = "none";
  $("modelInfo").textContent = `${(s.joints || []).length} DOF · ${(s.links || []).length} links · ` +
    (s.has_meshes ? `${s.visuals.length} meshes` : "skeleton");
  fillDropdowns(s);
  if (s.has_meshes) { ensureMeshes(s.visuals); placeCurrent(s.link_tf); }
  else updateSkeleton(s.link_tf);
  fitView(s.link_tf);
  // arm-angle readout from ik status
  const st = s.ik_status || {};
  const aa = st.arm_angles_now || {};
  const keys = Object.keys(aa);
  $("armangles").textContent = keys.length
    ? keys.map((k) => `${k}: ψ=${aa[k].toFixed(3)}`).join("  ") : "no srs_chains on ik_node";
  window.__lastLinkTf = s.link_tf;
}

// ---- solve --------------------------------------------------------------
function relRequest() {
  if (!$("relOn").checked) return null;
  const a = $("relA").value, b = $("relB").value;
  const tf = window.__lastLinkTf || {};
  if (!tf[a] || !tf[b]) return null;
  // rel = inv(Xb) * Xa
  const Xa = rosMat(tf[a]), Xb = rosMat(tf[b]);
  const rel = new THREE.Matrix4().copy(Xb).invert().multiply(Xa);
  const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
  rel.decompose(pos, quat, scl);
  return { frame_a: a, frame_b: b, xyz: [pos.x, pos.y, pos.z],
    quat: [quat.w, quat.x, quat.y, quat.z], stiffness: [1, 1, 1, 1, 1, 1] };
}

function buildRequest() {
  const req = { tasks: [{ frame: $("frame").value, xyz: [+$("tx").value, +$("ty").value, +$("tz").value],
    quat: targetQuatWXYZ(), stiffness: stiffVals.slice() }] };
  const aj = activeJointsForRequest(); if (aj) req.active_joints = aj;
  const vf = $("vfName").value.trim();
  if (vf) req.virtual_frames = [{ name: vf, parent: $("vfParent").value,
    xyz: [+$("vfx").value, +$("vfy").value, +$("vfz").value], rpy: [0, 0, 0] }];
  if ($("aaOn").checked && $("aaChain").value.trim())
    req.arm_angles = [{ chain: $("aaChain").value.trim(), psi: +$("aaPsi").value, stiffness: +$("aaStiff").value }];
  const rel = relRequest(); if (rel) req.relative = rel;
  return req;
}

function pill(el, ok, t, f) { el.textContent = ok ? (t ?? "yes") : (f ?? "no"); el.className = "v pill " + (ok ? "pill-good" : "pill-bad"); }

async function solve() {
  if (!haveModel) { $("msg").textContent = "no model yet"; return; }
  $("solve").disabled = true; $("msg").textContent = "solving…";
  let r;
  try { r = await (await fetch("/api/solve", { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(buildRequest()) })).json(); }
  catch (e) { $("msg").textContent = "solve error: " + e; $("solve").disabled = false; return; }
  $("solve").disabled = false;
  if (!r.ok) { $("msg").textContent = "FAILED: " + (r.error || r.reason || "?"); pill($("reachable"), false, "", "no"); return; }
  pill($("reachable"), !!r.reachable, "reachable", "blocked");
  $("reason").textContent = r.reason ?? "—";
  $("posErr").textContent = (r.max_pos_err != null ? (r.max_pos_err * 1000).toFixed(2) + " mm" : "—");
  $("oriErr").textContent = (r.max_ori_err != null ? r.max_ori_err.toFixed(4) + " rad" : "—");
  $("iters").textContent = r.iters ?? "—";
  $("manip").textContent = r.manipulability != null ? r.manipulability.toFixed(4) : "—";
  $("sigma").textContent = r.sigma_min != null ? r.sigma_min.toFixed(4) : "—";
  $("dq").textContent = r.delta_norm != null ? r.delta_norm.toFixed(3) : "—";
  const bj = (r.blocking_joints || []).length ? " · blocking: " + r.blocking_joints.join(", ") : "";
  const aa = (r.arm_angles && Object.keys(r.arm_angles).length)
    ? " · ψ " + Object.entries(r.arm_angles).map(([k, v]) => `${k}=${Number(v).toFixed(3)}`).join(" ") : "";
  $("msg").textContent = (r.reachable ? "solved" : "not reachable") + bj + aa;
  placeGhost(r.solution_link_tf || null);
}

async function capture() {
  const f = $("frame").value; if (!f) return;
  const vfName = $("vfName").value.trim();
  // Tool frame: compute its current pose locally as parent_tf * offset (the
  // dashboard model has no virtual frame, but the parent link does).
  if (vfName && f === vfName) {
    const tf = window.__lastLinkTf || {}; const parent = $("vfParent").value;
    if (!tf[parent]) { $("msg").textContent = "capture: parent '" + parent + "' has no transform yet"; return; }
    const M = rosMat(tf[parent]).multiply(
      localMatrix([+$("vfx").value, +$("vfy").value, +$("vfz").value], [0, 0, 0], null));
    const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
    M.decompose(pos, quat, scl);
    $("tx").value = pos.x.toFixed(4); $("ty").value = pos.y.toFixed(4); $("tz").value = pos.z.toFixed(4);
    const e = new THREE.Euler().setFromQuaternion(quat, "ZYX");
    $("rr").value = (e.x / DEG).toFixed(1); $("rp").value = (e.y / DEG).toFixed(1); $("ryaw").value = (e.z / DEG).toFixed(1);
    updateTargetMarker();
    $("msg").textContent = "captured tool frame " + vfName + " (via " + parent + ")";
    return;
  }
  try {
    const r = await (await fetch("/api/fk", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ frame: f }) })).json();
    if (!r.ok) { $("msg").textContent = "capture failed: " + (r.error || "?"); return; }
    $("tx").value = r.xyz[0].toFixed(4); $("ty").value = r.xyz[1].toFixed(4); $("tz").value = r.xyz[2].toFixed(4);
    const e = new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(r.quat[1], r.quat[2], r.quat[3], r.quat[0]), "ZYX");
    $("rr").value = (e.x / DEG).toFixed(1); $("rp").value = (e.y / DEG).toFixed(1); $("ryaw").value = (e.z / DEG).toFixed(1);
    updateTargetMarker();
    $("msg").textContent = "captured current pose of " + f;
  } catch (e) { $("msg").textContent = "capture error: " + e; }
}

// ---- wire up ------------------------------------------------------------
buildStiff();
$("solve").onclick = solve;
$("capture").onclick = capture;
$("fit").onclick = () => resetView();
$("frame").onchange = onFrameChange;
$("ghost").onchange = () => placeGhost(null);  // hide until next solve
$("vfName").addEventListener("input", syncVfOption);
document.querySelectorAll("[data-preset]").forEach((b) => b.onclick = () => applyPreset(b.dataset.preset));
["tx", "ty", "tz", "rr", "rp", "ryaw", "liveMarker"].forEach((id) => $(id).addEventListener("input", updateTargetMarker));

poll(); setInterval(poll, 300);
// Drive sizing from the actual canvas box each frame: robust to window resizes
// and panel layout, and avoids the canvas intrinsic-size feedback loop.
function resizeToDisplay() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const pr = renderer.getPixelRatio();
  if (canvas.width !== Math.round(w * pr) || canvas.height !== Math.round(h * pr)) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
}
(function animate() { requestAnimationFrame(animate); resizeToDisplay(); controls.update(); renderer.render(scene, camera); })();
