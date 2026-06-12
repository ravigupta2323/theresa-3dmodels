/* Fidget Clicker Tool - UI + Three.js preview */
"use strict";

const $ = (id) => document.getElementById(id);
const clog = (...a) => console.log("[fidget]", ...a);

clog("main.js v2 loaded");
clog("THREE present:", typeof THREE !== "undefined",
     typeof THREE !== "undefined" ? "r" + THREE.REVISION : "");
clog("STLLoader present:", typeof THREE !== "undefined" && !!THREE.STLLoader);
clog("OrbitControls present:", typeof THREE !== "undefined" && !!THREE.OrbitControls);
const PARAM_IDS = [
  "cap_height_mm", "size_mode", "look", "rest_float_mm",
  "manual_scale_factor", "bottom_solid_mm", "output_prefix",
];

/* ---------- parameter persistence ---------- */
function loadParams() {
  try {
    const saved = JSON.parse(localStorage.getItem("fidget_params_v2") || "{}");
    for (const id of PARAM_IDS) if (saved[id] !== undefined) $(id).value = saved[id];
  } catch (e) { /* ignore corrupt storage */ }
}
function collectParams() {
  const p = {};
  for (const id of PARAM_IDS) p[id] = $(id).value;
  localStorage.setItem("fidget_params_v2", JSON.stringify(p));
  const autoOr = (raw) => {
    const t = (raw || "").trim().toLowerCase();
    const n = parseFloat(t);
    return t === "" || t === "auto" || !isFinite(n) ? "auto" : n;
  };
  return {
    cap_height_mm: autoOr(p.cap_height_mm),
    rest_float_mm: autoOr(p.rest_float_mm),
    look: p.look,
    size_mode: p.size_mode,
    manual_scale_factor: parseFloat(p.manual_scale_factor) || 1.0,
    bottom_solid_mm: parseFloat(p.bottom_solid_mm),
    output_prefix: p.output_prefix || "fidget",
  };
}

/* ---------- log panel ---------- */
function renderLog(entries, append) {
  const el = $("log");
  if (!append) el.innerHTML = "";
  for (const e of entries || []) {
    const line = document.createElement("div");
    line.className = "log-" + e.level;
    line.textContent = `[${e.level}] ${e.msg}`;
    el.appendChild(line);
  }
  el.scrollTop = el.scrollHeight;
}

/* ---------- three.js scene ---------- */
let scene, camera, renderer, controls, modelGroup;

function init3d() {
  const vp = $("viewport");
  clog("init3d: viewport", vp ? vp.clientWidth + "x" + vp.clientHeight : "MISSING");
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1d2127);

  camera = new THREE.PerspectiveCamera(45, vp.clientWidth / vp.clientHeight, 0.1, 5000);
  camera.position.set(120, 90, 120);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  clog("init3d: WebGL renderer created, capabilities:",
       renderer.capabilities && renderer.capabilities.isWebGL2 ? "WebGL2" : "WebGL1");
  renderer.setSize(vp.clientWidth, vp.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  vp.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.HemisphereLight(0xe8f0ff, 0x303840, 0.9));
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(80, 140, 60);
  scene.add(dir);

  const grid = new THREE.GridHelper(200, 20, 0x39424e, 0x2a313a);
  scene.add(grid);

  // pipeline output is Z-up; rotate the group so Z-up maps to scene Y-up
  modelGroup = new THREE.Group();
  modelGroup.rotation.x = -Math.PI / 2;
  scene.add(modelGroup);

  // the stylesheet may not be applied yet when this runs, leaving the
  // viewport 0 wide - track its real size continuously
  const syncSize = () => {
    const w = vp.clientWidth, h = vp.clientHeight;
    if (w > 0 && h > 0 &&
        (renderer.domElement.width !== Math.round(w * window.devicePixelRatio) ||
         renderer.domElement.height !== Math.round(h * window.devicePixelRatio))) {
      clog("resize canvas to", w + "x" + h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    }
  };
  new ResizeObserver(syncSize).observe(vp);
  window.addEventListener("resize", syncSize);

  (function animate() {
    requestAnimationFrame(animate);
    syncSize();
    controls.update();
    renderer.render(scene, camera);
  })();
}

function clearGroup() {
  while (modelGroup.children.length) {
    const c = modelGroup.children.pop();
    if (c.geometry) c.geometry.dispose();
    if (c.material) c.material.dispose();
  }
}

async function addPreviewMesh(name, color, opacity) {
  try {
    clog("preview", name, "fetching…");
    const res = await fetch("/api/preview/" + name);
    if (!res.ok) {
      renderLog([{ level: "WARN", msg: `preview ${name}: HTTP ${res.status}` }], true);
      return;
    }
    const buf = await res.arrayBuffer();
    clog("preview", name, "got", buf.byteLength, "bytes, parsing…");
    const t0 = performance.now();
    const geo = new THREE.STLLoader().parse(buf);
    geo.computeVertexNormals();
    clog("preview", name, "parsed:",
         geo.attributes.position.count / 3, "triangles in",
         Math.round(performance.now() - t0), "ms");
    const mat = new THREE.MeshPhongMaterial({
      color, shininess: 28,
      transparent: opacity < 1, opacity,
      depthWrite: opacity >= 1,
    });
    modelGroup.add(new THREE.Mesh(geo, mat));
  } catch (e) {
    renderLog([{ level: "ERROR", msg: `preview ${name} failed: ${e}` }], true);
  }
}

function setupCutTuner(st) {
  const tuner = $("cut_tuner");
  if (st.mode !== "single_piece" || !st.cap_height_final_mm) {
    tuner.style.display = "none";
    return;
  }
  const h = st.model_size_mm[2];
  const slider = $("cap_slider");
  slider.min = 1;
  slider.max = Math.max(2, h - 5).toFixed(1);
  slider.step = 0.5;
  slider.value = st.cap_height_final_mm;
  $("cap_slider_val").textContent =
    `cap is the top ${st.cap_height_final_mm} mm (clicker ${h} mm tall)`;
  tuner.style.display = "flex";
}

function setDims(st) {
  const fmt = (a) => a.map((v) => v.toFixed(1)).join(" x ");
  $("dims").textContent =
    `clicker  ${fmt(st.model_size_mm)} mm\n` +
    `cap      ${fmt(st.cap_size_mm)} mm\n` +
    `holder   ${fmt(st.holder_size_mm)} mm\n` +
    `look: ${st.look}  travel: ${st.travel_mm} mm`;
}

async function loadPreview(stats) {
  $("status").textContent = "Loading 3D preview…";
  clearGroup();
  // solid parts first, ghost "shadow" overlays after (housing + keycap stem)
  await addPreviewMesh("bottom", 0x9fb0c0, 1.0);
  await addPreviewMesh("top", 0xcdd8e2, 1.0);
  await addPreviewMesh("housing", 0x33cc66, 0.4);
  await addPreviewMesh("keycap", 0xff9933, 0.4);

  // semi-transparent cut plane sized to the model bbox
  const [lo, hi] = stats.bbox;
  const w = (hi[0] - lo[0]) * 1.25, d = (hi[1] - lo[1]) * 1.25;
  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(w, d),
    new THREE.MeshBasicMaterial({
      color: 0x4488ff, transparent: true, opacity: 0.16,
      side: THREE.DoubleSide, depthWrite: false,
    })
  );
  plane.position.set((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, stats.z_cut);
  modelGroup.add(plane);

  // fit the camera
  const box = new THREE.Box3().setFromObject(modelGroup);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  clog("camera fit: children", modelGroup.children.length,
       "center", center.toArray().map(v => v.toFixed(1)),
       "size", size.toFixed(1));
  controls.target.copy(center);
  camera.position.copy(center.clone().add(new THREE.Vector3(size * 0.8, size * 0.55, size * 0.8)));
  camera.near = size / 100;
  camera.far = size * 10;
  camera.updateProjectionMatrix();
  $("status").textContent = "Preview ready.";
}

/* ---------- actions ---------- */
async function run() {
  const file = $("file").files[0];
  if (!file) {
    renderLog([{ level: "ERROR", msg: "Pick a .stl or .obj model file first." }]);
    return;
  }
  const params = collectParams();
  $("run").disabled = true;
  $("status").textContent = "Running pipeline… (big models take a few seconds)";
  try {
    const fd = new FormData();
    fd.append("model", file);
    fd.append("params", JSON.stringify(params));
    const res = await fetch("/api/run", { method: "POST", body: fd });
    const data = await res.json();
    renderLog(data.log);
    if (data.ok) {
      const st = data.stats;
      $("status").textContent =
        `Done (${st.mode === "two_piece" ? "two-piece model" : "single piece"}). ` +
        `Scale x${st.scale_applied}.`;
      setDims(st);
      setupCutTuner(st);
      await loadPreview(st);
      $("export").disabled = false;
    } else {
      $("status").textContent = "Failed - see log.";
    }
  } catch (e) {
    renderLog([{ level: "ERROR", msg: "Request failed: " + e }], true);
    $("status").textContent = "Failed - see log.";
  } finally {
    $("run").disabled = false;
  }
}

async function exportStl() {
  $("export").disabled = true;
  $("status").textContent = "Exporting STL files…";
  try {
    const res = await fetch("/api/export", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ output_prefix: collectParams().output_prefix }),
    });
    const data = await res.json();
    renderLog(data.log, true);
    $("status").textContent = data.ok
      ? "Exported: " + Object.values(data.paths).join("  |  ")
      : "Export failed - see log.";
  } finally {
    $("export").disabled = false;
  }
}

window.addEventListener("error", (e) => {
  renderLog([{ level: "ERROR", msg: "JS error: " + e.message }], true);
});

async function restoreLastResult() {
  try {
    const res = await fetch("/api/health");
    const d = await res.json();
    clog("health:", JSON.stringify({ has_result: d.has_result }));
    if (d.has_result && d.stats) {
      renderLog([{ level: "INFO", msg: "Restored the last run's preview." }]);
      setDims(d.stats);
      setupCutTuner(d.stats);
      await loadPreview(d.stats);
      $("export").disabled = false;
    }
  } catch (e) {
    clog("restore failed:", e);
    renderLog([{ level: "WARN", msg: "Could not restore last result: " + e }], true);
  }
}

loadParams();
init3d();
$("run").addEventListener("click", run);
$("export").addEventListener("click", exportStl);
$("cap_slider").addEventListener("input", () => {
  $("cap_height_mm").value = $("cap_slider").value;
  $("cap_slider_val").textContent =
    `cap = top ${$("cap_slider").value} mm - click Run to regenerate`;
});
$("cap_auto").addEventListener("click", () => {
  $("cap_height_mm").value = "auto";
  $("cap_slider_val").textContent = "auto seam - click Run to regenerate";
});
restoreLastResult();
