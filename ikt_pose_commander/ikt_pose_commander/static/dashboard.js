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
}

// ---- Configure: apply the picked controlled + base link ------------------
async function doConfigure() {
  const link = $("link-select").value;
  if (!link) { setMsg("pick a controlled link first"); return; }
  const out = await postJSON("/api/configure",
    { controlled_frame: link, base_frame: $("base-select").value });
  setMsg((out.ok ? "OK: " : "FAILED: ") + (out.message || ""));
  poll();
}

// ---- wire up --------------------------------------------------------------
// The control link is chosen ONLY in the panel; Snap/Track (viewer.js)
// auto-configure + enable, so Configure is just an explicit pre-configure.
$("btn-configure").onclick = doConfigure;

// kick off polling: connection pill + link/base dropdowns every 400 ms (the 3D
// viewer polls the scene separately).
poll();
setInterval(poll, 400);
