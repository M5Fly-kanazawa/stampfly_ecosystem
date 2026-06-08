// SPDX-License-Identifier: MIT — StampFly Ecosystem SIL GUI frontend.
// Scenario builder + parameter panel + interactive Plotly graphs + live three.js 3D.
// シナリオ作成・パラメータ・グラフ・ライブ3D。バックエンドは simulator/sil/gui/server.py。
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// ============================================================================ state
const S = {
  scenarios: [], params: [], paramOverrides: {},   // name -> value
  events: [],                                       // builder event list
  currentName: null, currentMode: 'saved',
  traj: null, frame: 0, playing: false, lastWall: 0,
};

// ============================================================================ api
const api = {
  async get(p) { return (await fetch(p)).json(); },
  async post(p, body) {
    return (await fetch(p, { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body) })).json();
  },
};
const $ = (id) => document.getElementById(id);
function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2200);
}

// ============================================================================ boot
async function boot() {
  S.scenarios = await api.get('/api/scenarios');
  S.params = await api.get('/api/params');
  const sel = $('scnSelect');
  sel.innerHTML = '<option value="">— 新規（空から作る）—</option>' +
    S.scenarios.map(s => `<option value="${s.name}">${s.name}</option>`).join('');
  sel.onchange = () => loadScenario(sel.value);
  buildParamPanel();
  wireUI();
  init3D();
  // Start on the capstone scenario if present, else the first.
  const start = S.scenarios.find(s => s.name === 'crash_refly') || S.scenarios[0];
  if (start) { sel.value = start.name; await loadScenario(start.name); }
}

// ============================================================================ scenario load + builder
async function loadScenario(name) {
  if (!name) { S.events = []; S.currentName = null; S.currentMode = 'custom'; renderEvents(); return; }
  const d = await api.get('/api/scenario?name=' + encodeURIComponent(name));
  if (d.error) { toast('読込失敗: ' + d.error); return; }
  S.events = d.events; S.currentName = name; S.currentMode = 'saved';
  $('durSec').value = Math.round((d.duration_us || 25e6) / 1e6);
  renderEvents();
}

// Field schema per channel — drives the builder form. 各チャネルの編集フィールド定義。
const FIELDS = {
  rc: [['thr', 'ｽﾛｯﾄﾙ', 2048], ['roll', 'ﾛｰﾙ', 2048], ['pitch', 'ﾋﾟｯﾁ', 2048], ['yaw', 'ﾖｰ', 2048],
       ['arm', 'arm', 0], ['hold_ms', '保持ms', 1000], ['rate_hz', 'Hz', 50],
       ['alt', 'ALT', 0], ['acro', 'ACRO', 0], ['pos', 'POS', 0]],
  rc_ramp: [['field', '軸', 'throttle'], ['from', 'from', 2048], ['to', 'to', 3000],
       ['step', 'step', 10], ['rate_hz', 'Hz', 50], ['arm', 'arm', 1], ['alt', 'ALT', 0], ['acro', 'ACRO', 0]],
  wind: [['fx', 'Fx[N]', 0], ['fy', 'Fy[N]', 0], ['fz', 'Fz↓[N]', 0], ['dur_ms', '突風ms', 0]],
  fault: [['motor', 'モータ0-3', 0], ['gain', 'ゲイン', 1.0]],
  bias: [['ax', 'ax', 0], ['ay', 'ay', 0], ['az', 'az', 0], ['gx', 'gx', 0], ['gy', 'gy', 0], ['gz', 'gz', 0]],
  handle: [['carry_alt', '高さm', 0.4], ['px', '置x', 0], ['py', '置y', 0],
       ['lift_ms', '持上ms', 1200], ['carry_ms', '運搬ms', 1200], ['place_ms', '設置ms', 1200]],
};
const CH_LABEL = { rc: 'rc スティック', rc_ramp: 'rc_ramp 掃引', wind: 'wind 外乱風',
  fault: 'fault モータ故障', bias: 'bias IMUバイアス', handle: 'handle 拾い上げ' };

function renderEvents() {
  const list = $('eventList');
  list.innerHTML = '';
  S.events.forEach((e, i) => list.appendChild(eventCard(e, i)));
}
function eventCard(e, i) {
  const div = document.createElement('div');
  div.className = 'event';
  const fields = FIELDS[e.ch] || [];
  div.innerHTML = `<div class="event-h">
      <span class="grip" title="ドラッグで並べ替え" draggable="true">⠿</span>
      <input class="at" value="${e.at ?? '+'}" title="時刻: 0=絶対ms / +=直前の後 / +500=後500ms"/>
      <span class="ch">${CH_LABEL[e.ch] || e.ch}</span>
      <button class="del" title="削除">✕</button>
    </div>
    <div class="event-fields">${fields.map(([k, lbl, dv]) =>
      `<label>${lbl}<input data-k="${k}" value="${e[k] ?? dv}"/></label>`).join('')}</div>
    <input class="comment" data-k="comment" placeholder="メモ" value="${e.comment || ''}"/>`;
  div.querySelector('.at').oninput = (ev) => { e.at = ev.target.value; };
  div.querySelectorAll('input[data-k]').forEach(inp => {
    inp.oninput = (ev) => {
      const k = inp.dataset.k, v = ev.target.value;
      e[k] = (k === 'comment' || k === 'field') ? v : (v.includes('.') ? parseFloat(v) : parseInt(v || 0));
    };
  });
  div.querySelector('.del').onclick = () => { S.events.splice(i, 1); renderEvents(); };
  // drag reorder
  const grip = div.querySelector('.grip');
  grip.ondragstart = (ev) => ev.dataTransfer.setData('text/plain', i);
  div.ondragover = (ev) => ev.preventDefault();
  div.ondrop = (ev) => {
    ev.preventDefault();
    const from = +ev.dataTransfer.getData('text/plain');
    if (from === i) return;
    const [m] = S.events.splice(from, 1); S.events.splice(i, 0, m); renderEvents();
  };
  return div;
}

function newEvent(ch) {
  const e = { ch, at: '+', comment: '' };
  (FIELDS[ch] || []).forEach(([k, , dv]) => e[k] = dv);
  return e;
}

// ============================================================================ params panel
function buildParamPanel() {
  const groups = {};
  S.params.forEach(p => (groups[p.group] = groups[p.group] || []).push(p));
  const wrap = $('paramList'); wrap.innerHTML = '';
  Object.entries(groups).forEach(([g, ps]) => {
    const det = document.createElement('details'); det.className = 'pgroup'; det.open = false;
    det.innerHTML = `<summary>${g} (${ps.length})</summary>`;
    ps.forEach(p => det.appendChild(paramRow(p)));
    wrap.appendChild(det);
  });
}
function paramRow(p) {
  const div = document.createElement('div');
  div.className = 'param'; div.dataset.name = p.name;
  const isBool = p.type === 'BOOL';
  const cur = S.paramOverrides[p.name] ?? p.default;
  div.innerHTML = `<div><div class="pname">${p.name}</div>
      <div class="pmeta">${p.type} 既定 ${fmt(p.default)} · [${fmt(p.min)}, ${fmt(p.max)}]</div></div>`;
  let input;
  if (isBool) {
    input = document.createElement('input'); input.type = 'checkbox'; input.className = 'bool';
    input.checked = cur != 0;
    input.onchange = () => setOverride(p, input.checked ? 1 : 0, div);
  } else {
    input = document.createElement('input'); input.type = 'number'; input.value = cur;
    input.step = (p.max - p.min) / 100 || 0.01; input.min = p.min; input.max = p.max;
    input.onchange = () => setOverride(p, parseFloat(input.value), div);
  }
  div.appendChild(input);
  if (S.paramOverrides[p.name] !== undefined) div.classList.add('changed');
  return div;
}
function setOverride(p, val, div) {
  if (val === p.default) { delete S.paramOverrides[p.name]; div.classList.remove('changed'); }
  else { S.paramOverrides[p.name] = val; div.classList.add('changed'); }
}
function fmt(v) { return Math.abs(v) < 1e-3 && v !== 0 ? v.toExponential(2) : String(+(+v).toFixed(4)); }

// ============================================================================ run
async function run() {
  const btn = $('runBtn'); btn.disabled = true;
  setVerdict('run', '実行中…');
  $('scene-msg').textContent = '実行中… 物理シミュレーションを走らせています';
  $('scene-msg').style.display = 'flex';
  const sel = $('scnSelect').value;
  // If the events were edited (or it's a new scenario), run as custom; else run the saved file.
  const mode = (S.currentMode === 'saved' && sel) ? 'saved' : 'custom';
  const req = {
    mode, name: sel || null, events: S.events,
    target: 'vehicle_new',
    duration_us: Math.round(parseFloat($('durSec').value) * 1e6),
    noise: $('noiseSel').value, seed: 12345, battery: $('battChk').checked,
    params: S.paramOverrides,
  };
  let r;
  try { r = await api.post('/api/run', req); }
  catch (err) { setVerdict('bad', 'エラー'); toast('通信エラー: ' + err); btn.disabled = false; return; }
  btn.disabled = false;
  if (r.error) { setVerdict('bad', '失敗'); toast(r.error); $('scene-msg').textContent = r.error; return; }
  renderResults(r);
  loadTrajectory(r.trajectory, r.timeline);
}
function setVerdict(kind, text) {
  const v = $('verdict'); v.className = 'verdict ' + kind; v.textContent = text;
}
function renderResults(r) {
  const checks = (r.results && r.results.checks) || [];
  const passN = checks.filter(c => c.pass && !c.skipped).length;
  const total = checks.filter(c => !c.skipped).length;
  const pass = r.results && r.results.pass;
  setVerdict(pass ? 'ok' : 'bad', pass ? `✅ ${passN}/${total}` : (r.ok ? `❌ ${passN}/${total}` : '❌ 失敗'));
  $('gateSummary').textContent = total ? `${passN}/${total} PASS` : '（.expect 無し → exit code 判定）';
  $('checks').innerHTML = checks.map(c => {
    const cls = c.skipped ? 'skip' : (c.pass ? 'pass' : 'fail');
    const badge = c.skipped ? 'SKIP' : (c.pass ? 'PASS' : 'FAIL');
    return `<div class="check ${cls}"><span class="badge">${badge}</span>
      <span>${c.name}</span><span class="detail">${c.detail || ''}</span></div>`;
  }).join('') || '<div class="muted">チェックなし</div>';
  $('cliTail').textContent = r.cli_tail || '';
}

// ============================================================================ Plotly graphs
const PLOT_LAYOUT = (title) => ({
  title: { text: title, font: { size: 11, color: '#8aa0bd' }, x: 0, xanchor: 'left' },
  margin: { l: 42, r: 8, t: 20, b: 22 }, height: 160,
  paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
  font: { color: '#8aa0bd', size: 9 }, showlegend: true,
  legend: { orientation: 'h', y: 1.25, x: 1, xanchor: 'right', font: { size: 9 } },
  xaxis: { gridcolor: '#1a2333', zeroline: false },
  yaxis: { gridcolor: '#1a2333', zeroline: false },
  shapes: [], hovermode: 'x unified',
});
const PLOT_CFG = { displayModeBar: false, responsive: true };

function drawGraphs(tr) {
  const t = tr.data.t, d = tr.data;
  const line = (y, name, color, dash) => ({ x: t, y, name, mode: 'lines',
    line: { color, width: 1.6, dash: dash || 'solid' }, hovertemplate: '%{y:.3f}' });
  Plotly.react('graphAlt', [
    line(d.alt, '高度 真値', '#22d3ee'), line(d.alt_est, '推定', '#a78bfa', 'dot'),
  ], PLOT_LAYOUT('高度 [m]'), PLOT_CFG);
  Plotly.react('graphAtt', [
    line(d.roll, 'roll 真', '#22d3ee'), line(d.roll_est, 'roll 推', '#0ea5b7', 'dot'),
    line(d.pitch, 'pitch 真', '#fbbf24'), line(d.pitch_est, 'pitch 推', '#b8860b', 'dot'),
  ], PLOT_LAYOUT('姿勢 roll/pitch [deg]'), PLOT_CFG);
  Plotly.react('graphMotor', [
    line(d.m0, 'M1', '#34d399'), line(d.m1, 'M2', '#22d3ee'),
    line(d.m2, 'M3', '#a78bfa'), line(d.m3, 'M4', '#f87171'),
  ], PLOT_LAYOUT('モータ duty [0-1]'), PLOT_CFG);
}
let _cursorThrottle = 0;
function updateCursor(tsec) {
  if (performance.now() - _cursorThrottle < 60) return;   // ~16 fps cursor update
  _cursorThrottle = performance.now();
  const shape = [{ type: 'line', x0: tsec, x1: tsec, yref: 'paper', y0: 0, y1: 1,
    line: { color: '#e6edf6', width: 1, dash: 'dot' } }];
  ['graphAlt', 'graphAtt', 'graphMotor'].forEach(g => {
    if ($(g).data) Plotly.relayout(g, { shapes: shape });
  });
}

// ============================================================================ three.js live 3D
let renderer, scene, camera, controls, worldGroup, drone, props = [], trailLine, trailGeo;
const MODEL_SCALE = 3.0;   // visual only — positions stay in real metres / 見やすさのため見た目だけ拡大

// Dolly the camera toward/away from the orbit target by `factor` (<1 = zoom in), clamped to
// a sensible distance for the metre-scale flight volume. OrbitControls.update() recomputes
// the spherical radius from the live camera offset, so a direct position change persists.
// カメラを target に対し factor 倍ドリー（<1で寄る）。距離はクランプ。
function dolly(factor) {
  const dir = camera.position.clone().sub(controls.target);
  const dist = Math.max(0.15, Math.min(50, dir.length() * factor));
  camera.position.copy(controls.target).add(dir.setLength(dist));
}

// Detect the OS so the zoom can be tuned per platform (the user asked for Mac/Win/Linux
// support). Low-entropy hint first (Chromium), then the legacy navigator.platform.
// OS 検出（Mac/Win/Linux 個別対応）。Chromium の低エントロピーヒント→旧 navigator.platform。
const OS = (() => {
  const p = ((navigator.userAgentData && navigator.userAgentData.platform) ||
             navigator.platform || '').toLowerCase();
  if (p.includes('mac')) return 'mac';
  if (p.includes('win')) return 'win';
  if (p.includes('linux')) return 'linux';
  return 'other';
})();

// Per-OS zoom gains [zoom-exponent per normalized pixel]. `scroll` = two-finger / mouse
// wheel; `pinch` = a trackpad/precision-touchpad pinch (ctrl+wheel), which sends much
// smaller steps so it needs a larger gain. The deltaMode normalization below (lines/pages →
// px) is what actually makes a Firefox line-mode mouse and a Mac trackpad behave the same;
// these per-OS numbers only fine-tune the feel. OS ごとのズーム感度。pinch は刻みが小さい
// ので gain 大。実際の機種差吸収は下の deltaMode 正規化が担い、ここは感触の微調整。
const ZOOM_TUNE = {
  mac:   { scroll: 0.0020, pinch: 0.013 },   // trackpad: small, smooth, momentum deltas
  win:   { scroll: 0.0016, pinch: 0.012 },   // mouse notch ≈ 100 px + precision touchpad
  linux: { scroll: 0.0018, pinch: 0.012 },   // mix of mouse (often line-mode) and touchpad
  other: { scroll: 0.0018, pinch: 0.012 },
};

// Cross-platform wheel zoom. Two-finger scroll AND pinch arrive as `wheel` (pinch = ctrlKey,
// finer steps); Safari pinch also fires gesture* events (handled below). We normalize for
// deltaMode (a mouse may report lines/pages, a trackpad reports pixels) and clamp the
// per-event factor so one big mouse notch or a fast flick can't teleport the zoom. All
// platforms preventDefault so the browser neither page-zooms (ctrl+wheel) nor scrolls the
// surrounding panel. Mac/Win/Linux 共通のホイールズーム。deltaMode を px に正規化し、
// 1イベントの倍率をクランプ。全 OS で preventDefault（ページズーム/親スクロール阻止）。
function installTrackpadZoom(canvas) {
  const tune = ZOOM_TUNE[OS] || ZOOM_TUNE.other;
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    let px = e.deltaY;
    if (e.deltaMode === 1) px *= 16;            // DOM_DELTA_LINE  → ~16 px/line (Firefox, some mice)
    else if (e.deltaMode === 2) px *= 100;      // DOM_DELTA_PAGE  → rough px
    const gain = e.ctrlKey ? tune.pinch : tune.scroll;
    let f = Math.exp(px * gain);                // deltaY>0 → zoom out, <0 → zoom in
    f = Math.min(2.0, Math.max(0.5, f));        // clamp per-event so a big notch can't jump
    dolly(f);
  }, { passive: false });
  // Safari (mac) pinch comes as gesture events; e.scale is cumulative since gesturestart.
  // Other browsers never fire these, so the handler is inert there. Safari のピンチ用。
  let gscale = 1;
  canvas.addEventListener('gesturestart', (e) => { e.preventDefault(); gscale = e.scale; },
    { passive: false });
  canvas.addEventListener('gesturechange', (e) => {
    e.preventDefault();
    dolly(gscale / e.scale);                    // scale grows (pinch out) → zoom in
    gscale = e.scale;
  }, { passive: false });
}

function init3D() {
  const canvas = $('scene');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 200);
  camera.position.set(2.2, 1.6, 2.2);
  controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  // We zoom ourselves (below) instead of OrbitControls' built-in wheel zoom: on a Mac
  // trackpad the built-in handling is unreliable — a pinch arrives as ctrl+wheel that the
  // browser eats for PAGE zoom, and a two-finger scroll gets stolen by the surrounding
  // scrollable panel. Our handler preventDefault()s both and dollies the camera directly.
  // Mac トラックパッドのため自前ズーム。ピンチ(ctrl+wheel)のページズーム化と親パネルへの
  // スクロール奪取を preventDefault で止め、カメラを直接ドリーする。
  controls.enableZoom = false;
  installTrackpadZoom(canvas);
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dl = new THREE.DirectionalLight(0xffffff, 0.8); dl.position.set(3, 6, 4); scene.add(dl);

  // worldGroup maps ENU local coords (x=East, y=North, z=Up) to three.js Y-up display:
  // rotating -90° about X sends a local (x,y,z) to world (x, z, -y) = (E, U, -N). Inside it
  // we use ENU coordinates directly. worldGroup が ENU→three(Y上)を担う(-90°X)。
  worldGroup = new THREE.Group(); worldGroup.rotation.x = -Math.PI / 2; scene.add(worldGroup);

  // Ground grid in the ENU x-y plane (z=0). GridHelper lies in three's x-z plane, so rotate
  // it into x-y. 地面グリッドを ENU 水平面(z=0)に。
  const grid = new THREE.GridHelper(8, 32, 0x2a3a55, 0x182336); grid.rotation.x = Math.PI / 2;
  worldGroup.add(grid);
  // ENU axes helper (E=red, N=green, U=blue). 座標軸(東赤/北緑/上青)。
  worldGroup.add(new THREE.AxesHelper(0.4));

  drone = buildDrone(); worldGroup.add(drone);
  trailGeo = new THREE.BufferGeometry();
  trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x22d3ee }));
  worldGroup.add(trailLine);

  window.addEventListener('resize', resize3D); resize3D();
  animate();
}
function resize3D() {
  const w = $('canvasWrap').clientWidth, h = $('canvasWrap').clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false); renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  camera.aspect = w / h; camera.updateProjectionMatrix();
}

// The drone is the REAL StampFly: the same STL meshes the MuJoCo model uses, loaded in the
// BODY FLU frame (x=forward, y=left, z=up). The MJCF imports each STL with scale 0.001
// (mm→m) and quat (0.5,0.5,0.5,0.5), which rotates the STL frame (X=left,Y=up,Z=fwd) onto
// FLU — we apply the identical transform so the model sits in the body frame and the
// per-frame attitude quaternion orients it correctly (props point down when it flips).
// 機体は本物の StampFly（MuJoCo と同じ STL）。MJCF と同じ scale 0.001 + quat(0.5,0.5,0.5,0.5)
// で STL(X左/Y上/Z前)を FLU に向け、毎フレームの姿勢 quat が正しく回す（反転でプロペラが下向き）。
const STL_PARTS = ['frame', 'pcb', 'm5stamps3', 'battery', 'battery_adapter',
                   'motor_fl', 'motor_fr', 'motor_rl', 'motor_rr'];
const PART_COLOR = { frame: 0x2b3442, pcb: 0x176f4f, m5stamps3: 0xe06a2a, battery: 0x8a6a3a,
  battery_adapter: 0x394150, motor_fl: 0x6b7480, motor_fr: 0x6b7480, motor_rl: 0x6b7480, motor_rr: 0x6b7480 };
// Rotor sites (FLU) + spin direction (CCW=+1 about +Z up) + which motor duty drives it.
// MJCF: M1 FR CCW, M2 RR CW, M3 RL CCW, M4 FL CW. ロータFLU位置＋回転方向＋対応モータ。
const ROTORS = [
  { pos: [0.023, -0.023, 0.006], dir: +1, motor: 0 },   // M1 FR CCW
  { pos: [-0.023, -0.023, 0.006], dir: -1, motor: 1 },  // M2 RR CW
  { pos: [-0.023, 0.023, 0.006], dir: +1, motor: 2 },   // M3 RL CCW
  { pos: [0.023, 0.023, 0.006], dir: -1, motor: 3 },    // M4 FL CW
];

// A 3-blade prop in the FLU frame, spinning about body +Z (up). Blades lie flat in the
// body x-y plane (wide chord, thin vertically). 機体FLUで3枚羽根、z軸回り。羽根は水平面に平ら。
function makeProp(color) {
  const prop = new THREE.Group();
  const r = 0.019;   // ~prop radius [m]
  // hub — a short cylinder whose axis (CylinderGeometry's local Y) is turned to body +Z (up)
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.005, 12),
    new THREE.MeshStandardMaterial({ color: 0x2a313c }));
  hub.rotation.x = Math.PI / 2;
  prop.add(hub);
  // 3 flat blades at 120°, radial in the x-y plane
  const bladeMat = new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.92,
    side: THREE.DoubleSide, metalness: 0.1, roughness: 0.7 });
  for (let i = 0; i < 3; i++) {
    const a = i * 2 * Math.PI / 3;
    const blade = new THREE.Mesh(new THREE.BoxGeometry(r, 0.005, 0.0008), bladeMat);  // len × chord × thin
    blade.position.set(Math.cos(a) * r / 2, Math.sin(a) * r / 2, 0.001);
    blade.rotation.z = a;
    prop.add(blade);
  }
  // faint translucent swept-area disk (flat in x-y)
  const disk = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.0006, 24),
    new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.10, side: THREE.DoubleSide }));
  disk.rotation.x = Math.PI / 2;
  prop.add(disk);
  return prop;
}

function buildDrone() {
  const g = new THREE.Group(); g.scale.setScalar(MODEL_SCALE);
  // small center marker so something shows before/if the STL parts load (fallback)
  g.add(new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.012, 0.006),
    new THREE.MeshStandardMaterial({ color: 0x9fb3c8 })));
  // Load the StampFly STL parts (async) into the FLU body frame.
  const loader = new STLLoader();
  const qStlToFlu = new THREE.Quaternion(0.5, 0.5, 0.5, 0.5);   // (x,y,z,w) = MJCF geom quat
  STL_PARTS.forEach(name => {
    loader.load(`/mesh/${name}.stl`, (geo) => {
      const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        color: PART_COLOR[name] || 0x6b7480, metalness: 0.25, roughness: 0.65 }));
      m.scale.setScalar(0.001);          // mm → m (MJCF mesh scale)
      m.quaternion.copy(qStlToFlu);      // STL frame → body FLU (MJCF geom quat)
      g.add(m);
    }, undefined, (err) => console.warn('[3D] STL load failed:', name, err));
  });
  // Props at the rotor sites; front cyan, rear violet for heading readability.
  ROTORS.forEach((rt, i) => {
    const prop = makeProp(rt.motor === 0 || rt.motor === 3 ? 0x22d3ee : 0xa78bfa);
    prop.position.set(rt.pos[0], rt.pos[1], rt.pos[2]);
    g.add(prop);
    props.push({ pivot: prop, dir: rt.dir, motor: rt.motor });
  });
  return g;
}

function loadTrajectory(tr, timeline) {
  if (!tr || !tr.data || !tr.data.t || !tr.data.t.length) {
    $('scene-msg').textContent = 'この走行には軌跡がありません'; $('scene-msg').style.display = 'flex';
    return;
  }
  S.traj = tr; S.frame = 0; S.playing = true; S.lastWall = performance.now();
  $('scene-msg').style.display = 'none';
  $('timeline').max = tr.data.t.length - 1; $('timeline').value = 0;
  drawGraphs(tr);
  buildTrail(tr);
  $('playBtn').textContent = '⏸';
}
function buildTrail(tr) {
  const d = tr.data, n = d.t.length;
  const pos = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { pos[i * 3] = d.px[i]; pos[i * 3 + 1] = d.py[i]; pos[i * 3 + 2] = d.pz[i]; }
  trailGeo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  trailGeo.setDrawRange(0, 0);
}

function setFrame(i) {
  if (!S.traj) return;
  const d = S.traj.data, n = d.t.length;
  S.frame = Math.max(0, Math.min(n - 1, i | 0));
  const f = S.frame;
  // ENU position + framequat (FLU→ENU). three.js Quaternion is (x,y,z,w).
  drone.position.set(d.px[f], d.py[f], d.pz[f]);
  drone.quaternion.set(d.qx[f], d.qy[f], d.qz[f], d.qw[f]);
  trailGeo.setDrawRange(0, $('trailChk').checked ? f + 1 : 0);
  // smooth chase: keep the controls target near the drone (user can still orbit/zoom)
  const tgt = new THREE.Vector3(d.px[f], d.pz[f], -d.py[f]);   // ENU→three for the target point
  controls.target.lerp(tgt, 0.12);
  $('timeline').value = f;
  $('clock').textContent = d.t[f].toFixed(2) + ' s';
  updateCursor(d.t[f]);
}

let _lastW = 0, _lastH = 0;
function animate() {
  requestAnimationFrame(animate);
  // Re-fit the renderer if the canvas wrapper changed size (initial layout, panel resize).
  // 初期レイアウト/リサイズに追従して描画バッファを合わせる。
  const w = $('canvasWrap').clientWidth, h = $('canvasWrap').clientHeight;
  if (w && h && (w !== _lastW || h !== _lastH)) { _lastW = w; _lastH = h; resize3D(); }
  const now = performance.now();
  if (S.traj && S.playing) {
    const d = S.traj.data, n = d.t.length;
    const dt = (now - S.lastWall) / 1000; S.lastWall = now;
    // advance by real time mapped onto the trajectory clock (≈ realtime playback)
    let tsec = d.t[S.frame] + dt;
    if (tsec >= d.t[n - 1]) { tsec = d.t[0]; }   // loop
    // find nearest frame for tsec
    let j = S.frame;
    while (j < n - 1 && d.t[j] < tsec) j++;
    if (tsec < d.t[S.frame]) j = 0;
    setFrame(j);
  } else { S.lastWall = now; }
  // Spin each prop at a rate set by ITS motor's duty at the current frame, in its real
  // turn direction (CCW/CW). Visual rate (not physical RPM — real props blur), but a higher
  // duty visibly spins faster, so you can read the mixer's per-motor effort. 各プロペラを
  // 現在フレームのそのモータ duty に比例した速さ・実際の回転方向で回す（視覚用レート）。
  const d = S.traj && S.traj.data;
  props.forEach(p => {
    const duty = d ? (d['m' + p.motor][S.frame] || 0) : 0;
    p.pivot.rotation.z += p.dir * (0.05 + duty * 1.6);   // idle creep + duty-proportional spin
  });
  controls.update();
  renderer.render(scene, camera);
}

// ============================================================================ wire UI
function wireUI() {
  $('runBtn').onclick = run;
  document.querySelectorAll('.tab').forEach(t => t.onclick = () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
    document.querySelectorAll('.tabpane').forEach(x => x.classList.remove('active'));
    t.classList.add('active'); $('tab-' + t.dataset.tab).classList.add('active');
  });
  $('addEventBtn').onclick = () => {
    S.events.push(newEvent($('addType').value));
    S.currentMode = 'custom';            // edited → run as custom
    renderEvents();
  };
  // any edit to events marks the scenario as custom (so the run uses the edited list)
  $('eventList').addEventListener('input', () => { S.currentMode = 'custom'; }, true);
  $('saveBtn').onclick = saveScenario;
  $('showScnBtn').onclick = async () => {
    const pre = $('scnPreview');
    if (!pre.hidden) { pre.hidden = true; return; }
    const r = await api.post('/api/save', { name: '__preview__', events: S.events, dry: true });
    // /api/save writes; for preview we just generate client-side via a no-op run path:
    pre.textContent = r.text || '(プレビュー生成不可)'; pre.hidden = false;
  };
  $('paramSearch').oninput = (e) => filterParams(e.target.value.toLowerCase());
  $('resetParamsBtn').onclick = () => { S.paramOverrides = {}; buildParamPanel(); toast('パラメータ変更をクリア'); };
  $('playBtn').onclick = () => {
    S.playing = !S.playing; S.lastWall = performance.now();
    $('playBtn').textContent = S.playing ? '⏸' : '▶';
  };
  $('timeline').oninput = (e) => { S.playing = false; $('playBtn').textContent = '▶'; setFrame(+e.target.value); };
  $('trailChk').onchange = () => setFrame(S.frame);
}
function filterParams(q) {
  document.querySelectorAll('.pgroup').forEach(det => {
    let any = false;
    det.querySelectorAll('.param').forEach(row => {
      const show = row.dataset.name.toLowerCase().includes(q);
      row.style.display = show ? '' : 'none'; any = any || show;
    });
    det.style.display = any ? '' : 'none'; if (q && any) det.open = true;
  });
}
async function saveScenario() {
  const name = $('saveName').value.trim();
  if (!name) { toast('保存名を入れてください'); return; }
  const r = await api.post('/api/save', { name, events: S.events,
    header: `${name}.scn — built with the SIL GUI` });
  if (r.error) { toast('保存失敗: ' + r.error); return; }
  toast('保存しました: ' + r.path);
  S.scenarios = await api.get('/api/scenarios');
  const sel = $('scnSelect');
  sel.innerHTML = '<option value="">— 新規（空から作る）—</option>' +
    S.scenarios.map(s => `<option value="${s.name}">${s.name}</option>`).join('');
  sel.value = name; S.currentName = name; S.currentMode = 'saved';
}

boot();
