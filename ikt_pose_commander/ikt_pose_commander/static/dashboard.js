"use strict";

const $ = (id) => document.getElementById(id);

function pill(el, ok, textTrue, textFalse) {
  el.textContent = ok ? (textTrue ?? "yes") : (textFalse ?? "no");
  el.className = "v pill " + (ok ? "pill-good" : "pill-bad");
}

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

  const fresh = snap.fresh;
  const s = snap.status;
  $("conn").textContent = fresh ? "connected" : "no commander status";
  $("conn").className = "pill " + (fresh ? "pill-good" : "pill-bad");

  if (!s) {
    $("base").textContent = snap.base_frame || "—";
    return;
  }

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

  pill($("configured"), !!s.configured, "configured", "not configured");
  $("cfg-joints").textContent = (s.joints && s.joints.length)
    ? (s.joints.length + " (" + s.joints.join(", ") + ")") : "—";
  $("cfg-jtc").textContent = s.jtc_controller || "—";
  $("cfg-fpc").textContent = s.fpc_controller || "—";

  pill($("enabled"), !!s.enabled, "ENABLED", "disabled");
  $("enabled").className = "v pill " + (s.enabled ? "pill-warn" : "");
  $("mode").textContent = s.mode ?? "—";
  pill($("have_model"), !!s.have_model);
  pill($("js_fresh"), !!s.joint_states_fresh, "fresh", "stale");
  $("frame").textContent = s.controlled_frame ?? "—";
  $("base").textContent = s.base_frame ?? snap.base_frame ?? "—";
  $("last_message").textContent = s.last_message ?? "—";
  $("max_step").textContent = (s.max_step_rad ?? 0).toFixed(3) + " rad";
  $("step").textContent = (s.last_step_rad ?? 0).toFixed(4) + " rad";

  const sv = s.last_solve;
  if (sv) {
    pill($("reachable"), !!sv.reachable, "reachable", "blocked");
    $("reason").textContent = sv.reason ?? "—";
    $("pos_err").textContent = (sv.max_pos_err * 1000).toFixed(2) + " mm";
    $("ori_err").textContent = (sv.max_ori_err).toFixed(4) + " rad";
  }

  // ---- Tracking (Phase 0/1/2/3) -----------------------------------------
  $("solve_frame").textContent = s.solve_frame ?? "\u2014";
  $("preset_v").textContent = s.stiffness_preset ?? "\u2014";
  $("ee_disp").textContent = (s.ee_displacement ?? 0).toFixed(3) + " m";
  $("safety_v").textContent = (s.safety_radius_m ?? 0).toFixed(2) + " m";
  $("clamp_v").textContent = (s.clamp_scale ?? 1).toFixed(2);
  pill($("best_effort"), !!s.best_effort, "best-effort", "no");
  $("best_effort").className = "v pill " + (s.best_effort ? "pill-warn" : "");
  $("control_v").textContent = (s.control_rate_hz ?? 0).toFixed(0) + " Hz";
  const tox = s.tool_offset_xyz;
  $("tool_v").textContent = tox
    ? ("[" + tox.map((v) => v.toFixed(3)).join(", ") + "]") : "none";

  // Populate the motion inputs ONCE from the live status, then never overwrite
  // them (so a refresh tick never wipes what the user is typing).
  if (!window._motionInit && s.configured) {
    const setIf = (id, val) => {
      if (val !== undefined && val !== null && $(id)) $(id).value = val;
    };
    setIf("safety-radius", s.safety_radius_m);
    setIf("control-rate", s.control_rate_hz);
    setIf("reach-gain", s.reach_gain);
    setIf("max-speed", s.max_joint_speed);
    setIf("min-time", s.min_move_time);
    setIf("max-step-in", s.max_step_rad);
    if (s.stiffness_preset) $("preset-select").value = s.stiffness_preset;
    $("allow-unreach").checked = !!s.allow_unreachable;
    window._motionInit = true;
  }

// ---- actions --------------------------------------------------------------
async function doTrigger(enable) {
  const out = await postJSON(enable ? "/api/enable" : "/api/disable", {});
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

async function doConfigure() {
  const link = $("link-select").value;
  const mode = $("mode-select").value;
  const base = $("base-select").value;
  if (!link) { setMsg("pick a controlled link first"); return; }
  const out = await postJSON("/api/configure",
    { controlled_frame: link, command_mode: mode, base_frame: base });
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

async function doCapture() {
  const out = await postJSON("/api/capture", {});
  if (!out.ok) { setMsg("capture failed: " + (out.message || "")); return; }
  $("tx").value = out.xyz[0].toFixed(4);
  $("ty").value = out.xyz[1].toFixed(4);
  $("tz").value = out.xyz[2].toFixed(4);
  $("qw").value = out.quat[0].toFixed(4);
  $("qx").value = out.quat[1].toFixed(4);
  $("qy").value = out.quat[2].toFixed(4);
  $("qz").value = out.quat[3].toFixed(4);
  $("fid").value = out.frame_id || "";
  setMsg("captured current pose of controlled frame");
}

async function doSend() {
  const body = {
    xyz: [parseFloat($("tx").value), parseFloat($("ty").value), parseFloat($("tz").value)],
    quat: [parseFloat($("qw").value), parseFloat($("qx").value),
           parseFloat($("qy").value), parseFloat($("qz").value)],
    frame_id: $("fid").value.trim(),
  };
  if (body.xyz.some(Number.isNaN)) { setMsg("enter x/y/z first (or Capture)"); return; }
  const out = await postJSON("/api/send", body);
  setMsg(out.ok ? ("sent target → " + JSON.stringify(out.xyz)) : ("send failed: " + out.message));
}

async function doJog(axis, sign) {
  const step = parseFloat($("jog-step").value) || 0.01;
  const out = await postJSON("/api/jog", { axis, delta: sign * step });
  setMsg(out.ok ? (out.message || "jogged") : ("jog failed: " + (out.message || "")));
}

async function doApplyMotion() {
  const cfg = {
    stiffness_preset: $("preset-select").value,
    allow_unreachable: $("allow-unreach").checked,
  };
  const numIf = (id, key) => {
    const v = parseFloat($(id).value);
    if (!Number.isNaN(v)) cfg[key] = v;
  };
  numIf("safety-radius", "safety_radius_m");
  numIf("control-rate", "control_rate_hz");
  numIf("reach-gain", "reach_gain");
  numIf("max-speed", "max_joint_speed");
  numIf("min-time", "min_move_time");
  numIf("max-step-in", "max_step_rad");
  if ($("preset-select").value === "custom") {
    const parts = ($("stiff6").value || "").trim().split(/[\s,]+/).map(Number);
    if (parts.length === 6 && parts.every((x) => !Number.isNaN(x))) {
      cfg.default_stiffness = parts;
    }
  }
  const out = await postJSON("/api/configure", cfg);
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

async function doApplyTool() {
  const xyz = [parseFloat($("tool-x").value), parseFloat($("tool-y").value),
               parseFloat($("tool-z").value)];
  const rpy = [parseFloat($("tool-r").value), parseFloat($("tool-p").value),
               parseFloat($("tool-yw").value)];
  if (xyz.some(Number.isNaN) || rpy.some(Number.isNaN)) {
    setMsg("enter tool xyz/rpy (use 0 to clear)"); return;
  }
  const out = await postJSON("/api/configure",
    { tool_offset_xyz: xyz, tool_offset_rpy: rpy });
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

async function doReturn() {
  setMsg("returning to start\u2026");
  const out = await postJSON("/api/return_to_start", {});
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

// ---- wire up --------------------------------------------------------------
$("btn-configure").onclick = doConfigure;
$("btn-enable").onclick = () => doTrigger(true);
$("btn-disable").onclick = () => doTrigger(false);
$("btn-return").onclick = doReturn;
$("btn-apply-motion").onclick = doApplyMotion;
$("btn-apply-tool").onclick = doApplyTool;
$("btn-capture").onclick = doCapture;
$("btn-send").onclick = doSend;
document.querySelectorAll(".jog .btn").forEach((b) => {
  b.onclick = () => doJog(b.dataset.axis, parseFloat(b.dataset.sign));
});

poll();
setInterval(poll, 400);
