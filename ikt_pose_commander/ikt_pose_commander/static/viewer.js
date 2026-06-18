"use strict";
// 3D viewer for the ikt_pose_commander dashboard.
//
// Renders the robot from /robot_description meshes at the live /joint_states
// configuration (per-link FK computed server-side, mirrored from the
// ikt_inverse_kinematics dashboard) PLUS a triad + sphere at the *commanded
// target pose* (whatever is currently on <ns>/target_pose — the dashboard's own
// jog/send OR the spacemouse_servo teleop bridge).
//
// On top of the basic robot + target view it adds (referring to the other
// toolkit dashboards):
//   * a "mesh" toggle (solid meshes vs. link-frame skeleton);
//   * per-link name labels (HTML overlay, toggle);
//   * per-link coordinate frames (toggle);
//   * highlights the controlled link (selected from the panel dropdown);
//   * a live joint-angle panel and a controlled-frame TCP-pose panel.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;

// Per-joint color palette for the on-canvas joint bars (matches the 6-axis
// convention used by the other toolkit dashboards: J1=blue, J2=green,
// J3=orange, J4=red, J5=purple, J6=cyan; extra axes cycle).
const JOINT_COLORS = ["#42a5f5", "#66bb6a", "#ffa726",
                      "#ef5350", "#ab47bc", "#26c6da"];

// ---- scene --------------------------------------------------------------
const canvas = $("viewer");
const labelsEl = $("labels");
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
const highlightMat = new THREE.MeshStandardMaterial({ color: 0xffb454, emissive: 0x6e3d00,
  emissiveIntensity: 0.6, metalness: 0.2, roughness: 0.5 });
const stlLoader = new STLLoader();
const geomCache = {};   // url -> {geom, waiting:[cb]}
const meshItems = {};   // key(link#i) -> {link, local, solid}
const frameAxes = {};   // link -> AxesHelper
const labelPool = [];   // reusable label divs (clustered, not per-link)
let allLinks = [];      // link names from the snapshot
let jointTree = [];     // [{parent, child, type}] for the skeleton lines
let didFit = false;

// ---- view options (checkboxes) ------------------------------------------
const opt = { mesh: true, labels: true, frames: false };
function readOpts() {
  if ($("show-mesh")) opt.mesh = $("show-mesh").checked;
  if ($("show-labels")) opt.labels = $("show-labels").checked;
  if ($("show-frames")) opt.frames = $("show-frames").checked;
}
["show-mesh", "show-labels", "show-frames"].forEach((id) => {
  const el = $(id);
  if (el) el.addEventListener("change", () => { readOpts(); refreshStatic(); });
});

// ---- selection (synced with the controlled-link dropdown) ---------------
let selectedLink = "";
function dropdown() { return $("link-select"); }
function setSelected(link) {
  selectedLink = link || "";
  if ($("sel-link")) $("sel-link").textContent = selectedLink || "\u2014";
}

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
      s.userData.link = v.link;          // for raycast → link lookup
      item.solid = s; scene.add(s);
    });
  });
}
function placeCurrent(linkTf) {
  for (const key in meshItems) {
    const it = meshItems[key]; if (!it.solid) continue;
    const lm = linkTf[it.link];
    if (!lm || !opt.mesh) { it.solid.visible = false; continue; }
    it.solid.visible = true;
    it.solid.material = (it.link === selectedLink) ? highlightMat : solidMat;
    it.solid.matrix.copy(rosMat(lm).multiply(it.local));
  }
}

// ---- per-link coordinate frames (toggle) --------------------------------
function ensureFrames(linkTf) {
  for (const link in linkTf) {
    if (frameAxes[link]) continue;
    const ax = new THREE.AxesHelper(0.07);
    ax.matrixAutoUpdate = false; ax.visible = false;
    frameAxes[link] = ax; scene.add(ax);
  }
}
function placeFrames(linkTf) {
  for (const link in frameAxes) {
    const ax = frameAxes[link]; const lm = linkTf[link];
    if (!lm || !opt.frames) { ax.visible = false; continue; }
    ax.visible = true; ax.matrix.copy(rosMat(lm));
  }
}

// ---- per-link name labels (HTML overlay, clustered) ---------------------
// Links whose screen positions overlap — e.g. ft_sensor_link and
// compliance_link mounted at the link_6 flange with zero offset — are MERGED
// into a single comma-separated label so they don't stack illegibly on top of
// each other (mirrors the reference cartesian_controller_dashboard). Uses a
// reusable pool of divs, re-clustered every frame. Clicking a merged label
// cycles the selection through the links it covers.
const MERGE_PX = 18;     // screen-space merge radius (CSS px)
function getLabelDiv(i) {
  if (labelPool[i]) return labelPool[i];
  const d = document.createElement("div");
  d.className = "lbl";
  labelPool[i] = d; if (labelsEl) labelsEl.appendChild(d);
  return d;
}
const _lv = new THREE.Vector3();
function placeLabels(linkTf) {
  if (!labelsEl) return;
  if (!opt.labels) { for (const d of labelPool) d.style.display = "none"; return; }
  const w = canvas.clientWidth, h = canvas.clientHeight;
  // project every visible link origin to screen
  const hits = [];
  for (const link of allLinks) {
    const lm = linkTf[link]; if (!lm) continue;
    _lv.set(lm[0][3], lm[1][3], lm[2][3]).project(camera);
    if (_lv.z > 1) continue;   // behind camera
    hits.push({ link, x: (_lv.x * 0.5 + 0.5) * w, y: (-_lv.y * 0.5 + 0.5) * h });
  }
  // greedy cluster by screen distance — overlapping links share one label
  const clusters = [];
  for (const hit of hits) {
    let merged = false;
    for (const c of clusters) {
      if (Math.hypot(hit.x - c.x, hit.y - c.y) < MERGE_PX) {
        c.links.push(hit.link); merged = true; break;
      }
    }
    if (!merged) clusters.push({ x: hit.x, y: hit.y, links: [hit.link] });
  }
  // render one div per cluster, reusing the pool
  let i = 0;
  for (; i < clusters.length; i++) {
    const c = clusters[i]; const d = getLabelDiv(i);
    d.textContent = c.links.join(", ");
    d.style.display = "block";
    d.style.left = c.x.toFixed(0) + "px";
    d.style.top = c.y.toFixed(0) + "px";
    d.classList.toggle("sel", c.links.includes(selectedLink));
  }
  for (; i < labelPool.length; i++) labelPool[i].style.display = "none";
}

// ---- skeleton (joint-connectivity lines + link dots) --------------------
// A proper kinematic skeleton — a line segment between each joint's parent and
// child link origins, plus a dot at every link origin — shown when meshes are
// off (or the model has no meshes). Mirrors the reference dashboard, which is
// skeleton-only.
let skelLines = null, skelDots = null;
function ensureSkeleton() {
  if (!skelLines) {
    skelLines = new THREE.LineSegments(new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x34c3ff }));
    skelLines.frustumCulled = false; scene.add(skelLines);
  }
  if (!skelDots) {
    skelDots = new THREE.Points(new THREE.BufferGeometry(),
      new THREE.PointsMaterial({ color: 0xe6e6e6, size: 0.022 }));
    skelDots.frustumCulled = false; scene.add(skelDots);
  }
}
function updateSkeleton(linkTf, show) {
  ensureSkeleton();
  skelLines.visible = show; skelDots.visible = show;
  if (!show) return;
  // lines: parent origin -> child origin for each joint in the tree
  const segs = [];
  for (const j of jointTree) {
    const a = linkTf[j.parent], b = linkTf[j.child];
    if (!a || !b) continue;
    segs.push(a[0][3], a[1][3], a[2][3], b[0][3], b[1][3], b[2][3]);
  }
  skelLines.geometry.setAttribute("position",
    new THREE.Float32BufferAttribute(segs, 3));
  skelLines.geometry.computeBoundingSphere();
  // dots at every link origin
  const pts = [];
  for (const k in linkTf) { const m = linkTf[k]; pts.push(m[0][3], m[1][3], m[2][3]); }
  skelDots.geometry.setAttribute("position",
    new THREE.Float32BufferAttribute(pts, 3));
  skelDots.geometry.computeBoundingSphere();
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

// ---- target frame + drag gizmo (the goal pose for the controlled link) ----
// A Three.js TransformControls handle on a VISIBLE "target proxy" frame. Drag it
// (move / rotate) to set the goal pose; it does NOT follow the link or command
// the robot on its own. "Snap target -> link" resets it onto the controlled
// link's current pose. The robot is commanded explicitly from the Engage panel:
//   * Snap robot  -> configure JTC + enable + ONE move to the target frame.
//   * Track robot -> configure FPC + enable + live-stream the target frame
//                    (dragging then drives the robot continuously). Toggle off /
//                    Stop disengages.
const targetProxy = new THREE.Object3D();
targetProxy.add(new THREE.AxesHelper(0.16));   // the visible target frame
scene.add(targetProxy);
const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.setSize(0.9);
// LOCAL space: the move/rotate handles align with the target frame's own axes
// (which equal the controlled link's axes after a snap), so rotating the target
// matches the link instead of the world axes.
gizmo.setSpace("local");
gizmo.attach(targetProxy);
scene.add(gizmo);

let tracking = false;        // FPC live-align active (Track robot)
let controlledFrame = "";    // commander's current controlled_frame
let rootFrame = "";          // model root frame name (target frame_id)
let _proxyInit = false;      // target frame placed at least once
let _lastStreamT = 0;
const STREAM_MIN_MS = 40;    // throttle live FPC streaming to ~25 Hz

gizmo.addEventListener("dragging-changed", (e) => {
  controls.enabled = !e.value;                 // don't orbit while dragging a handle
  if (!e.value && tracking) sendProxyTarget(true);   // final pose on release while tracking
});
gizmo.addEventListener("objectChange", () => {
  if (!tracking) return;                        // dragging commands the robot only while tracking
  const now = performance.now();
  if (now - _lastStreamT >= STREAM_MIN_MS) { _lastStreamT = now; sendProxyTarget(true); }
});

function setProxyFromMat(m4) {
  rosMat(m4).decompose(targetProxy.position, targetProxy.quaternion, targetProxy.scale);
  targetProxy.scale.set(1, 1, 1);
  targetProxy.updateMatrixWorld(true);
  _proxyInit = true;
}

async function api(url, body) {
  try {
    const r = await fetch(url, body !== undefined
      ? { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }
      : undefined);
    return await r.json();
  } catch (e) { return { ok: false, message: String(e) }; }
}
const gizmoPost = (url, body) => api(url, body || {});
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
function actionMsg(t) { const am = $("action-msg"); if (am) am.textContent = t || ""; }

const _gp = new THREE.Vector3(), _gq = new THREE.Quaternion(), _gscl = new THREE.Vector3();
function targetPoseBody() {
  targetProxy.updateMatrixWorld(true);
  targetProxy.matrixWorld.decompose(_gp, _gq, _gscl);
  // three Quaternion is (x,y,z,w); the commander wants (w,x,y,z)
  return { xyz: [_gp.x, _gp.y, _gp.z], quat: [_gq.w, _gq.x, _gq.y, _gq.z], frame_id: rootFrame || "" };
}
function sendProxyTarget(stream) {
  api(stream ? "/api/target" : "/api/send", targetPoseBody()).then((out) => {
    if (out && out.ok === false) actionMsg("target failed: " + (out.message || ""));
  });
}

// "Snap target -> link": move the target frame onto the selected control link.
function snapTargetToLink() {
  const link = ($("link-select") && $("link-select").value) || controlledFrame;
  const tf = window.__lastLinkTf;
  if (!link || !tf || !tf[link]) { actionMsg("no live pose for '" + (link || "?") + "' yet"); return; }
  setProxyFromMat(tf[link]);
  actionMsg("target frame snapped to " + link);
}

// Ensure the commander is configured for the selected link + mode and enabled.
// Passes the current controller names so no re-discovery is needed (fast).
async function engage(mode) {
  const link = ($("link-select") && $("link-select").value) || controlledFrame;
  const base = ($("base-select") && $("base-select").value) || "";
  if (!link) { actionMsg("pick a controlled link first"); return false; }
  let snap = await api("/api/state");
  let s = snap && snap.status;
  const needCfg = !s || !s.configured || s.controlled_frame !== link || s.mode !== mode;
  if (needCfg) {
    if (s && s.enabled) await gizmoPost("/api/disable");
    const cfg = { controlled_frame: link, base_frame: base, command_mode: mode };
    if (s && s.jtc_controller) cfg.jtc_controller = s.jtc_controller;
    if (s && s.fpc_controller) cfg.fpc_controller = s.fpc_controller;
    await gizmoPost("/api/configure", cfg);
    let ok = false;
    for (let i = 0; i < 40; i++) {
      await sleep(100);
      snap = await api("/api/state"); s = snap && snap.status;
      if (s && s.configured && s.mode === mode && s.controlled_frame === link) { ok = true; break; }
    }
    if (!ok) { actionMsg("configure to " + mode + " failed: " + (s ? s.last_message : "")); return false; }
  }
  if (snap && snap.root_frame) rootFrame = snap.root_frame;
  if (!s.enabled) { const r = await gizmoPost("/api/enable"); if (!r.ok) { actionMsg("enable failed: " + (r.message || "")); return false; } }
  return true;
}

async function snapRobot() {
  actionMsg("Snap: configuring JTC + enabling\u2026");
  if (!(await engage("jtc"))) return;
  sendProxyTarget(false);   // one /api/send -> JTC move to the target frame
  actionMsg("Snap: JTC move to the target frame sent");
}

function updateTrackBtn() {
  const b = $("btn-track-robot"); if (!b) return;
  b.textContent = tracking ? "Stop tracking" : "Track robot (fpc)";
  b.classList.toggle("btn-stop", tracking);
  b.classList.toggle("btn-go", !tracking);
}

async function trackRobot() {
  if (tracking) { await stopRobot(); return; }
  actionMsg("Track: configuring FPC + enabling\u2026");
  if (!(await engage("fpc"))) return;
  tracking = true; updateTrackBtn();
  sendProxyTarget(true);    // initial setpoint
  actionMsg("Tracking: drag the target frame — the robot follows live (FPC)");
}

async function stopRobot() {
  tracking = false; updateTrackBtn();
  await gizmoPost("/api/disable");
  actionMsg("Stopped / disengaged (holding pose)");
}

function setGizmoMode(mode) {
  gizmo.setMode(mode);
  const mv = $("gizmo-move"), ro = $("gizmo-rotate");
  if (mv) mv.classList.toggle("sel", mode === "translate");
  if (ro) ro.classList.toggle("sel", mode === "rotate");
}

if ($("gizmo-move")) $("gizmo-move").onclick = () => setGizmoMode("translate");
if ($("gizmo-rotate")) $("gizmo-rotate").onclick = () => setGizmoMode("rotate");
if ($("btn-snap-target")) $("btn-snap-target").onclick = snapTargetToLink;
if ($("btn-snap-robot")) $("btn-snap-robot").onclick = snapRobot;
if ($("btn-track-robot")) $("btn-track-robot").onclick = trackRobot;
if ($("btn-disable")) $("btn-disable").onclick = stopRobot;
// When the controlled link changes in the panel dropdown, re-place the target
// frame onto that link so it starts ALIGNED with it (the link is then driven to
// "match" the target). Skipped while live-tracking so an active FPC stream isn't
// yanked to a new pose mid-motion.
if ($("link-select")) $("link-select").addEventListener("change", () => {
  if (!tracking && window.__lastLinkTf) snapTargetToLink();
});

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

// ---- joint-angle panel (colored bars, centre-line fill) -----------------
const jointRowEls = {};   // jn -> {bar, val}
function buildJointRows(joints) {
  const host = $("joint-bars"); if (!host) return;
  if (host.dataset.n === String(joints.length)) return;   // already built
  host.dataset.n = String(joints.length);
  host.innerHTML = "";
  for (const k in jointRowEls) delete jointRowEls[k];
  joints.forEach((jn, i) => {
    const color = JOINT_COLORS[i % JOINT_COLORS.length];
    const row = document.createElement("div"); row.className = "jbrow";
    row.innerHTML =
      `<span class="jblabel" style="color:${color}" title="${jn}">J${i + 1}</span>`
      + `<span class="jbbg"><span class="jbcenter"></span>`
      + `<span class="jbbar"></span></span>`
      + `<span class="jbval">—</span>`;
    row.addEventListener("pointerdown", () => {
      // selecting the joint highlights the child link it actuates (best-effort)
    });
    host.appendChild(row);
    jointRowEls[jn] = { bar: row.querySelector(".jbbar"),
                        val: row.querySelector(".jbval"), color };
  });
}
function updateJointPanel(joints, values, limits) {
  const host = $("joint-bars"); if (!host) return;
  if (!joints || !joints.length || !values) {
    host.innerHTML = '<div class="muted sm">no joints</div>'; host.dataset.n = "";
    return;
  }
  buildJointRows(joints);
  for (const jn of joints) {
    const row = jointRowEls[jn]; if (!row) continue;
    const rad = Number(values[jn] ?? 0);
    const lim = (limits && limits[jn]) || [null, null];
    // symmetric display range from the limits (fallback ±π)
    let span = Math.PI;
    if (lim[0] != null && lim[1] != null) span = Math.max(Math.abs(lim[0]), Math.abs(lim[1])) || Math.PI;
    const frac = Math.max(-1, Math.min(1, rad / span));   // -1..1
    const pct = Math.abs(frac) * 50;
    const b = row.bar;
    b.style.background = row.color;
    if (frac >= 0) { b.classList.add("positive"); b.classList.remove("negative"); b.style.left = "50%"; b.style.right = ""; }
    else { b.classList.add("negative"); b.classList.remove("positive"); b.style.right = "50%"; b.style.left = ""; }
    b.style.width = pct.toFixed(1) + "%";
    row.val.textContent = (rad * DEG).toFixed(1) + "°";
  }
}

// ---- TCP-pose panel (controlled frame) ----------------------------------
const _p = new THREE.Vector3(), _q = new THREE.Quaternion(), _s = new THREE.Vector3();
function updateTcp(linkTf, frame) {
  const host = $("tcp-pose"); if (!host) return;
  const fl = $("tcp-frame");
  if (!frame || !linkTf || !linkTf[frame]) {
    if (fl) fl.textContent = "";
    host.innerHTML = '<div class="muted sm">no controlled frame (Configure a link)</div>'; return;
  }
  if (fl) fl.textContent = "· " + frame;
  rosMat(linkTf[frame]).decompose(_p, _q, _s);
  const e = new THREE.Euler().setFromQuaternion(_q, "ZYX");
  host.innerHTML =
    `<div class="trow"><span class="k">xyz m</span><span class="v">`
    + `${_p.x.toFixed(4)}, ${_p.y.toFixed(4)}, ${_p.z.toFixed(4)}</span></div>`
    + `<div class="trow"><span class="k">rpy °</span><span class="v">`
    + `${(e.x * DEG).toFixed(1)}, ${(e.y * DEG).toFixed(1)}, ${(e.z * DEG).toFixed(1)}</span></div>`
    + `<div class="trow"><span class="k">quat</span><span class="v">`
    + `${_q.w.toFixed(3)}, ${_q.x.toFixed(3)}, ${_q.y.toFixed(3)}, ${_q.z.toFixed(3)}</span></div>`;
}

// Re-apply visibility/highlight to the static scene after a toggle change,
// using the last link transforms (no need to wait for the next poll).
function refreshStatic() {
  const tf = window.__lastLinkTf;
  if (!tf) return;
  placeCurrent(tf); placeFrames(tf); placeLabels(tf);
  updateSkeleton(tf, !opt.mesh || !window.__hasMeshes);
}

// ---- poll ---------------------------------------------------------------
async function poll() {
  let s;
  try { s = await (await fetch("/api/state")).json(); } catch { return; }
  readOpts();
  // keep selection in sync if the user changed the dropdown directly
  const dd = dropdown();
  if (dd && dd.value && dd.value !== selectedLink) setSelected(dd.value);

  if (s.has_model_viz) {
    const ld = $("loading"); if (ld) ld.style.display = "none";
    const tf = s.link_tf || {};
    window.__hasMeshes = !!s.has_meshes;
    allLinks = s.links || Object.keys(tf);
    jointTree = s.joint_tree || [];
    if (s.has_meshes) ensureMeshes(s.visuals || []);
    ensureFrames(tf);
    placeCurrent(tf); placeFrames(tf); placeLabels(tf);
    updateSkeleton(tf, !opt.mesh || !s.has_meshes);
    fitView(tf);
    window.__lastLinkTf = tf;
    // floating-overlay counts + model pill
    if ($("n-links")) $("n-links").textContent = (s.links || []).length || "—";
    if ($("n-joints")) $("n-joints").textContent = (s.joints || []).length || "—";
    if ($("model-pill")) {
      $("model-pill").textContent = s.has_meshes ? "meshes" : "skeleton";
      $("model-pill").className = "pill pill-good";
    }
    updateJointPanel(s.joints, s.joint_values, s.joint_limits);
    updateTcp(tf, s.controlled_frame);
    // --- target frame: track names; place the frame ONCE (no auto-follow) ---
    controlledFrame = s.controlled_frame || "";
    rootFrame = s.root_frame || rootFrame;
    const gf = $("grab-frame");
    if (gf) gf.textContent = controlledFrame ? ("\u00b7 " + controlledFrame) : "";
    // Place the target frame on the controlled/selected link the FIRST time we
    // have a pose; afterwards it stays where the user drags it (use
    // "Snap target -> link" to reset it onto the link again).
    if (!_proxyInit) {
      const initLink = controlledFrame || ($("link-select") && $("link-select").value);
      if (initLink && tf[initLink]) setProxyFromMat(tf[initLink]);
    }
    // default the selection to the controlled frame the first time we see it
    if (!selectedLink && s.controlled_frame) setSelected(s.controlled_frame);
  }
  updateTarget(s.target);
}
if ($("fit")) $("fit").onclick = () => resetView();
// collapse / expand the floating overlay
if ($("vf-collapse")) $("vf-collapse").onclick = () => {
  const f = $("view-float"); const b = $("vf-collapse");
  const collapsed = f.classList.toggle("collapsed");
  b.textContent = collapsed ? "+" : "−";
  b.setAttribute("aria-expanded", String(!collapsed));
};
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
  // labels track the camera every frame (cheap; ~9 divs)
  if (window.__lastLinkTf) placeLabels(window.__lastLinkTf);
})();
