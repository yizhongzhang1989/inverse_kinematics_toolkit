"use strict";

const $ = (id) => document.getElementById(id);

async function postJSON(url, body) {
  try {
    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return await r.json();
  } catch (e) {
    return { ok: false, message: String(e) };
  }
}

function setMsg(text) {
  $("action-msg").textContent = text || "";
}

// ---- poll + render --------------------------------------------------------
async function poll() {
  let snap;
  try {
    const r = await fetch("/api/state");
    snap = await r.json();
  } catch (e) {
    $("conn").textContent = "disconnected";
    $("conn").className = "pill pill-bad";
    return;
  }

  const s = snap.status;
  $("conn").textContent = snap.fresh ? "connected" : "no commander status";
  $("conn").className = "pill " + (snap.fresh ? "pill-good" : "pill-bad");
  if (!s) return;

  // populate the link dropdown from the live URDF (once / when it changes)
  const links = s.available_links || [];
  const sel = $("link-select");
  if (links.length && sel.dataset.count != String(links.length)) {
    const cur = sel.value;
    sel.innerHTML = "";
    for (const l of links) {
      const o = document.createElement("option");
      o.value = l; o.textContent = l; sel.appendChild(o);
    }
    sel.dataset.count = String(links.length);
    // preselect the currently controlled frame if any; otherwise pick a
    // sensible robot-agnostic default — a gripper tip (prefer *Link7, then
    // common end-effector names), never the root "world"/"base_link" which
    // have no movable joints and make Configure fail.
    if (s.controlled_frame) {
      sel.value = s.controlled_frame;
    } else if (cur && links.includes(cur)) {
      sel.value = cur;
    } else {
      const tip = links.find((l) => /Link7$/.test(l))
        || links.find((l) => /(tool|tcp|_ee$|hand|gripper|flange)/i.test(l))
        || links[links.length - 1];
      if (tip) sel.value = tip;
    }
  }

  // populate the base-link dropdown (target reference frame): all links plus a
  // "(robot root)" = empty default. Runtime-changeable like the controlled link.
  const bsel = $("base-select");
  if (links.length && bsel.dataset.count != String(links.length)) {
    const cur = bsel.value;
    bsel.innerHTML = "";
    const root = document.createElement("option");
    root.value = ""; root.textContent = "(robot root)"; bsel.appendChild(root);
    for (const l of links) {
      const o = document.createElement("option");
      o.value = l; o.textContent = l; bsel.appendChild(o);
    }
    bsel.dataset.count = String(links.length);
    const b = s.base_frame;
    bsel.value = (b && b !== "(model root)" && links.includes(b)) ? b
      : (cur && links.includes(cur)) ? cur : "";
  }

  // Per-joint "fixed" checkboxes: one row per movable joint in the live URDF.
  // Checked = held OUT of the IK (e.g. a lifter). Rebuild only when the joint
  // set changes (so the 400 ms refresh never fights a click). Each refresh
  // re-syncs the checked state to the commander's current fixed_joints unless
  // the user is mid-edit (a pending apply).
  const joints = s.available_joints || [];
  const fjHost = $("fixed-joints-list");
  if (fjHost && joints.length && fjHost.dataset.count != String(joints.length)) {
    fjHost.innerHTML = "";
    joints.forEach((j, i) => {
      const id = "fj-" + i;
      const row = document.createElement("label");
      row.className = "fj-row";
      row.innerHTML =
        `<input type="checkbox" id="${id}" value="${j}">`
        + `<span class="fj-name">${j}</span>`
        + `<span class="fj-tag" hidden>FIXED</span>`;
      const cb = row.querySelector("input");
      cb.addEventListener("change", () => onFixedToggle());
      fjHost.appendChild(row);
    });
    fjHost.dataset.count = String(joints.length);
  }
  if (fjHost && !fjHost.dataset.editing) {
    const fixed = new Set(s.fixed_joints || []);
    fjHost.querySelectorAll("input[type=checkbox]").forEach((cb) => {
      cb.checked = fixed.has(cb.value);
      const tag = cb.parentElement.querySelector(".fj-tag");
      if (tag) tag.hidden = !cb.checked;
      cb.parentElement.classList.toggle("is-fixed", cb.checked);
    });
  }

  initParams(s);

  // Reflect the commander's target_mode on the buttons (once, then leave the
  // user's clicks alone — like the parameter inputs).
  if (!_tmodeInit && s.target_mode) {
    updateTargetModeUI(s.target_mode);
    _tmodeInit = true;
  }
}

function fixedJointsSelected() {
  const fjHost = $("fixed-joints-list");
  if (!fjHost) return [];
  return Array.from(fjHost.querySelectorAll("input[type=checkbox]:checked"))
    .map((cb) => cb.value);
}

// Toggle a joint's fixed state immediately (fixed_joints is structural -> the
// commander applies it while DISABLED and refuses while enabled). We send the
// full set and let the next poll re-sync the checkboxes to the real state.
async function onFixedToggle() {
  const fjHost = $("fixed-joints-list");
  if (fjHost) fjHost.dataset.editing = "1";   // pause refresh-resync briefly
  // reflect the tag immediately for snappy feedback
  fjHost.querySelectorAll("input[type=checkbox]").forEach((cb) => {
    const tag = cb.parentElement.querySelector(".fj-tag");
    if (tag) tag.hidden = !cb.checked;
    cb.parentElement.classList.toggle("is-fixed", cb.checked);
  });
  const fixed = fixedJointsSelected();
  const out = await postJSON("/api/configure", { fixed_joints: fixed });
  setMsg((out.ok ? "fixed joints: " : "fixed joints FAILED: ")
    + (fixed.length ? fixed.join(", ") : "none")
    + (out.message ? " — " + out.message : ""));
  setTimeout(() => { if (fjHost) delete fjHost.dataset.editing; }, 800);
  poll();
}

// ---- Configure: apply the picked controlled + base link ------------------
async function doConfigure() {
  const link = $("link-select").value;
  if (!link) { setMsg("pick a controlled link first"); return; }
  const out = await postJSON("/api/configure",
    { controlled_frame: link, base_frame: $("base-select").value,
      fixed_joints: fixedJointsSelected() });
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

// ---- Snap target -> current pose (server-side) ---------------------------
// Tells the commander to snap its internal target onto the controlled frame's
// CURRENT pose (FK of the measured joints) — seeds the goal before delta
// jogging, or re-centres it with no jump.
async function doSnapCurrent() {
  const out = await postJSON("/api/snap_target", {});
  setMsg((out.ok ? "snapped: " : "snap failed: ") + (out.message || ""));
}

// ---- Target mode (absolute | delta) --------------------------------------
let _tmodeInit = false;
function updateTargetModeUI(mode) {
  const ab = $("btn-tmode-absolute"), de = $("btn-tmode-delta");
  if (ab) ab.classList.toggle("sel", mode !== "delta");
  if (de) de.classList.toggle("sel", mode === "delta");
}
async function setTargetMode(mode) {
  updateTargetModeUI(mode);
  const out = await postJSON("/api/configure", { target_mode: mode });
  setMsg(out.ok ? ("target_mode = " + mode)
                : ("set target_mode failed: " + (out.message || "")));
}

// ---- IK / motion parameters (live: each change is applied immediately) ----
// Every input in the Parameters card carries data-key=<commander param>. Read
// its value by type and relay just that key to ~/configure (a _LIVE_KEY, so it
// applies even while enabled).
function paramValue(el) {
  if (el.type === "checkbox") return el.checked;
  if (el.dataset.vec) {
    const p = (el.value || "").trim().split(/[\s,]+/).map(Number);
    return (p.length === 6 && p.every((x) => !Number.isNaN(x))) ? p : null;
  }
  if (el.tagName === "SELECT") return el.value;
  if (el.type === "number") {
    const v = el.dataset.int ? parseInt(el.value, 10) : parseFloat(el.value);
    return Number.isNaN(v) ? null : v;
  }
  return el.value;
}

async function onParamChange(el) {
  const key = el.dataset.key;
  const val = paramValue(el);
  if (val === null) { setMsg("invalid value for " + key); return; }
  const out = await postJSON("/api/configure", { [key]: val });
  setMsg(out.ok ? ("set " + key) : ("set " + key + " failed: " + (out.message || "")));
}

// ---- per-DOF stiffness sliders -> one default_stiffness 6-vector ----------
// Six range sliders (X Y Z RX RY RZ) are assembled into the default_stiffness
// vector and sent together. 0 = that DOF floats free, 1 = fully constrained.
function setStiffSlider(i, val) {
  const el = $("p-stiff-" + i);
  const out = $("v-stiff-" + i);
  if (el) el.value = val;
  if (out) out.textContent = Number(val).toFixed(2);
}
function readStiffness() {
  const arr = [];
  for (let i = 0; i < 6; i++) {
    const el = $("p-stiff-" + i);
    const v = el ? parseFloat(el.value) : NaN;
    if (Number.isNaN(v)) return null;
    arr.push(v);
  }
  return arr;
}
async function sendStiffness() {
  const arr = readStiffness();
  if (!arr) return;
  const out = await postJSON("/api/configure", { default_stiffness: arr });
  setMsg(out.ok ? "set default_stiffness"
                : ("set default_stiffness failed: " + (out.message || "")));
}
// While dragging a slider: update its readout live and STREAM the vector
// throttled (~20 Hz) so the change drives the robot smoothly without flooding
// /api/configure; a final apply fires on release (the change event).
let _stiffTimer = null, _stiffLast = 0;
function onStiffInput(el) {
  const out = $(el.dataset.out);
  if (out) out.textContent = parseFloat(el.value).toFixed(2);
  const now = performance.now();
  if (now - _stiffLast >= 50) { _stiffLast = now; sendStiffness(); }
  else {
    clearTimeout(_stiffTimer);
    _stiffTimer = setTimeout(() => { _stiffLast = performance.now(); sendStiffness(); }, 50);
  }
}

// Populate the parameter inputs ONCE from the live status, then leave them to
// the user (the 400 ms refresh must never wipe what is being typed).
let _paramsInit = false;
function initParams(s) {
  if (_paramsInit || !s) return;
  let any = false;
  document.querySelectorAll("[data-key]").forEach((el) => {
    const v = s[el.dataset.key];
    if (v === undefined || v === null) return;
    if (el.type === "checkbox") el.checked = !!v;
    else if (el.dataset.vec) el.value = Array.isArray(v) ? v.join(" ") : String(v);
    else el.value = v;
    any = true;
  });
  // per-DOF stiffness sliders (assembled into one vector, not data-key driven)
  const st = s.default_stiffness;
  if (Array.isArray(st) && st.length === 6) {
    for (let i = 0; i < 6; i++) setStiffSlider(i, st[i]);
    any = true;
  }
  if (any) _paramsInit = true;
}

// ---- wire up --------------------------------------------------------------
// The control link is chosen ONLY in the panel; Snap/Track (viewer.js)
// auto-configure + enable, so Configure is just an explicit pre-configure.
$("btn-configure").onclick = doConfigure;
if ($("btn-snap-current")) $("btn-snap-current").onclick = doSnapCurrent;
if ($("btn-tmode-absolute")) $("btn-tmode-absolute").onclick = () => setTargetMode("absolute");
if ($("btn-tmode-delta")) $("btn-tmode-delta").onclick = () => setTargetMode("delta");

// IK / motion parameter inputs: apply each one live on change.
document.querySelectorAll("[data-key]").forEach((el) => {
  el.addEventListener("change", () => onParamChange(el));
});

// Per-DOF stiffness sliders: live readout + throttled streaming while dragging,
// plus a final apply on release.
document.querySelectorAll(".stiff-dof").forEach((el) => {
  el.addEventListener("input", () => onStiffInput(el));
  el.addEventListener("change", sendStiffness);
});

// kick off polling: connection pill + link/base dropdowns every 400 ms (the 3D
// viewer polls the scene separately).
poll();
setInterval(poll, 400);
