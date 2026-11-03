// sf log video - replay view (three.js r160, vendored under ./vendor/three).
// sf log video の再生ビュー（three.js r160、./vendor/three に同梱）。
//
// CONVENTIONS (same words as replay_data.py) / 規約（replay_data.py と同じ言葉）
//  * attitude quaternion (w,x,y,z) rotates BODY FRD vectors into NED (R body->NED).
//    姿勢の四元数 (w,x,y,z) は機体 FRD のベクトルを NED へ回す（R 機体->NED）。
//  * position is NED in metres; altitude h = -pos_z.
//    位置は NED [m]、高度 h = -pos_z。
//  * motor duty order FR, RR, RL, FL (= M1..M4).
//    モータ duty の並びは FR, RR, RL, FL（= M1..M4）。
//  * flight mode 0 ACRO, 1 STABILIZE, 2 ALT_HOLD, 3 POS_HOLD.
//    飛行モード 0 ACRO, 1 STABILIZE, 2 ALT_HOLD, 3 POS_HOLD。
//  * roll/pitch/yaw shown in the numbers panel are the true (not exaggerated) ZYX angles.
//    数値欄の roll/pitch/yaw は誇張なしの ZYX 角。
//
// Scene world (three.js): X = East, Y = up, Z = -North.
// 描画の世界座標（three.js）: X = 東、Y = 上、Z = -北。
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// ------------------------------------------------------------------ settings
const Q = new URLSearchParams(location.search);
const OPT = {
  data: Q.get('data') || 'data.json',
  view: Q.get('view') || 'oblique',
  tilt: parseFloat(Q.get('tilt') ?? '3'),
  vscale: parseFloat(Q.get('vscale') ?? '2'),
  zoom: parseFloat(Q.get('zoom') ?? '0.6'),
  record: Q.get('record') === '1',
};
const W = window.innerWidth, H = window.innerHeight;
const MAIN_W = Math.round(W * 0.7);
document.documentElement.style.setProperty('--u', (H / 1080) + 'px');

const COL = { board: 0x25302b, panel: 0x161d19, chalk: 0xf2efe6, yellow: 0xecdf7a, cyan: 0x8fd3f4 };
const FOV_DEG = 36, CAM_TAU_S = 0.5, OBLIQUE_ELEV_DEG = 33;
const TRAIL_RADIUS = 0.00225, TRAIL_MIN_ALPHA = 0.18, TRAIL_FADE_S = 12, TRAIL_MAX_POINTS = 2500;
const SPIN_IDLE = 3, SPIN_GAIN = 60;               // rad per video second / 動画の1秒あたりのラジアン
const VIEW_HALF = 0.06;                             // half size of the small views [m] / 小画面の半幅 [m]
const VOLT_MIN = 3.3, VOLT_MAX = 4.2, VOLT_LOW = 3.4, VOLT_CAUTION = 3.6;

const MODE_JA = ['角速度操縦（ACRO）', '水平安定（STABILIZE）', '高度保持（ALT_HOLD）', '位置保持（POS_HOLD）'];
const STATE_JA = ['起動中', '地上で待機', '手で保持', '離陸準備完了', '離陸中', '飛行中', '着陸中', '宙返り中'];
const MOTOR_CELLS = [  // airframe layout: FL FR on top, RL RR below / 機体と同じ並び
  { idx: 3, name: 'FL', m: 'M4', dir: 'CW（時計回り）' },
  { idx: 0, name: 'FR', m: 'M1', dir: 'CCW（反時計）' },
  { idx: 2, name: 'RL', m: 'M3', dir: 'CCW（反時計）' },
  { idx: 1, name: 'RR', m: 'M2', dir: 'CW（時計回り）' },
];

const $ = (id) => document.getElementById(id);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));

// ----------------------------------------------------------------- renderer
const canvas = $('gl');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(1);
renderer.setSize(W, H, false);
const scene = new THREE.Scene();
const L_MAIN = 0, L_VIEW = { front: 1, side: 2, top: 3 };

// Lights are visible on every layer. 光源は全レイヤーで有効にする。
const key = new THREE.DirectionalLight(0xffffff, 2.6); key.position.set(0.4, 1, 0.5);
const fill = new THREE.DirectionalLight(0xbfd4ff, 1.0); fill.position.set(-0.6, 0.5, -0.4);
for (const light of [new THREE.HemisphereLight(0xffffff, 0x556058, 2.2), key, fill]) {
  light.layers.enableAll();
  scene.add(light);
}

// ------------------------------------------------------------------ vehicle
// STL parts and materials follow simulator/sils/gui/static/app.js.
// STL 部品と材質は simulator/sils/gui/static/app.js に合わせる。
const PART_NAMES = ['frame', 'pcb', 'm5stamps3', 'battery', 'battery_adapter', 'motor_fl', 'motor_fr', 'motor_rl', 'motor_rr'];
const metal = { color: 0xb9c1cb, roughness: 0.35, metalness: 0.85 };
const PART_MAT = {
  frame: new THREE.MeshStandardMaterial({ color: 0xccd2db, roughness: 0.5, metalness: 0.1 }),
  pcb: new THREE.MeshStandardMaterial({ color: 0x0e1014, roughness: 0.55, metalness: 0.2 }),
  m5stamps3: new THREE.MeshStandardMaterial({ color: 0xff6a00, roughness: 0.4, metalness: 0.1 }),
  battery: new THREE.MeshStandardMaterial({ color: 0x2a1d14, roughness: 0.6, metalness: 0.1 }),
  battery_adapter: new THREE.MeshStandardMaterial({ color: 0x33312d, roughness: 0.6, metalness: 0.1 }),
  motor_fl: new THREE.MeshStandardMaterial(metal), motor_fr: new THREE.MeshStandardMaterial(metal),
  motor_rl: new THREE.MeshStandardMaterial(metal), motor_rr: new THREE.MeshStandardMaterial(metal),
};
const bladeMat = new THREE.MeshStandardMaterial({ color: 0xff2b3d, transparent: true, opacity: 0.62, roughness: 0.18,
  emissive: 0x4a0008, emissiveIntensity: 0.35, side: THREE.DoubleSide });
const hubMat = new THREE.MeshStandardMaterial({ color: 0x8e0512, roughness: 0.3, metalness: 0.15 });

// Prop hubs in the STL native frame (mm; X left, Y up, Z front), the motor index in data
// order (FR, RR, RL, FL) that drives each, and its turn direction about body up
// (+1 CCW, -1 CW).  STL 元座標（mm; X 左, Y 上, Z 前）でのプロペラ位置、駆動するモータの
// データ上の番号（FR, RR, RL, FL）、機体の上向き軸まわりの回転方向（+1 反時計, -1 時計）。
const PROP_HUBS = [
  { pos: [22.80, 7.81, 22.80], motor: 3, dir: -1 },    // FL = M4 CW
  { pos: [-22.80, 7.81, 22.80], motor: 0, dir: +1 },   // FR = M1 CCW
  { pos: [22.80, 7.81, -22.81], motor: 2, dir: +1 },   // RL = M3 CCW
  { pos: [-22.80, 7.81, -22.81], motor: 1, dir: -1 },  // RR = M2 CW
];
const PROP_RADIUS = 14.99, HUB_R = PROP_RADIUS * 0.22;

// One flat paddle blade with a linear twist. ねじれ付きの平たい羽根 1 枚。
function makeBladeGeometry(len, width, thick, twistRoot, twistTip) {
  const s = new THREE.Shape();
  s.moveTo(0, 0.20 * width);
  s.bezierCurveTo(0.40 * len, 0.50 * width, 0.65 * len, 0.50 * width, 0.86 * len, 0.30 * width);
  s.quadraticCurveTo(len, 0.16 * width, len, 0);
  s.quadraticCurveTo(len, -0.12 * width, 0.86 * len, -0.20 * width);
  s.bezierCurveTo(0.65 * len, -0.58 * width, 0.40 * len, -0.58 * width, 0, -0.24 * width);
  s.closePath();
  const geo = new THREE.ExtrudeGeometry(s, { depth: thick, bevelEnabled: false });
  geo.translate(0, 0, -thick / 2);
  const p = geo.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
    const a = twistRoot + (twistTip - twistRoot) * clamp(x / len, 0, 1);
    p.setXYZ(i, x, y * Math.cos(a) - z * Math.sin(a), y * Math.sin(a) + z * Math.cos(a));
  }
  geo.rotateX(Math.PI / 2);
  geo.computeVertexNormals();
  return geo;
}
const bladeGeo = makeBladeGeometry(PROP_RADIUS - HUB_R * 0.6, (PROP_RADIUS - HUB_R * 0.6) / 2.7, 0.6, 0.38, 0.10);
const hubGeo = new THREE.CylinderGeometry(HUB_R, HUB_R * 0.9, 2.6, 24);
const domeGeo = new THREE.SphereGeometry(HUB_R * 0.85, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2);

// A three-blade propeller that spins about its local Y (= body up). 3 枚羽根のプロペラ（局所 Y 軸まわりに回る）。
function makeProp() {
  const prop = new THREE.Group();
  prop.add(new THREE.Mesh(hubGeo, hubMat));
  const dome = new THREE.Mesh(domeGeo, hubMat); dome.position.y = 1.0; prop.add(dome);
  for (let i = 0; i < 3; i++) {
    const blade = new THREE.Mesh(bladeGeo, bladeMat);
    blade.rotation.y = i * Math.PI * 2 / 3; blade.translateX(HUB_R * 0.6); prop.add(blade);
  }
  return prop;
}

const partGeometry = {};
// Load the STL parts once; every vehicle clone shares the geometry. STL 部品を 1 回だけ読み、全クローンで共有する。
async function loadParts() {
  const loader = new STLLoader();
  await Promise.all(PART_NAMES.map(async (name) => {
    const geo = await loader.loadAsync(`meshes/${name}.stl`);
    geo.computeVertexNormals();
    partGeometry[name] = geo;
  }));
}

// Build one StampFly on `layer`. The STL native frame is turned into body FLU
// (X forward, Y left, Z up) by scale 0.001 and quaternion (0.5,0.5,0.5,0.5).
// `layer` 上に StampFly を 1 機組み立てる。STL 元座標は scale 0.001 と
// 四元数 (0.5,0.5,0.5,0.5) で機体 FLU（X 前, Y 左, Z 上）にする。
function buildVehicle(layer, scale) {
  const pivot = new THREE.Group();
  const flu = new THREE.Group(); flu.scale.setScalar(scale);
  const native = new THREE.Group();
  native.scale.setScalar(0.001);
  native.quaternion.set(0.5, 0.5, 0.5, 0.5);
  flu.add(native); pivot.add(flu);
  const props = PROP_HUBS.map((h) => {
    const prop = makeProp();
    prop.position.set(h.pos[0], h.pos[1] - 1.4, h.pos[2]);
    native.add(prop);
    return { node: prop, dir: h.dir, motor: h.motor };
  });
  for (const name of PART_NAMES) native.add(new THREE.Mesh(partGeometry[name], PART_MAT[name]));
  pivot.traverse((o) => o.layers.set(layer));
  scene.add(pivot);
  return { pivot, props };
}

// --------------------------------------------------------------- attitude
// Fixed matrices: NED -> scene world, and body FLU -> body FRD.
// 固定行列: NED -> 描画の世界座標、機体 FLU -> 機体 FRD。
const M_NED_TO_SCENE = new THREE.Matrix4().set(0, 1, 0, 0,  0, 0, -1, 0,  -1, 0, 0, 0,  0, 0, 0, 1);
const M_FLU_TO_FRD = new THREE.Matrix4().makeScale(1, -1, -1);
const NED_DOWN = new THREE.Vector3(0, 0, 1);
const _m = new THREE.Matrix4(), _q = new THREE.Quaternion();

// Yaw [rad] (ZYX) of a body->NED quaternion. body->NED 四元数の ZYX ヨー角 [rad]。
function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

// Tilt exaggeration: the angle th between body-down and NED-down is mapped to
// th' = pi * (1 - (1 - th/pi)^k). It is k times th for small tilts, never exceeds
// pi, and stays continuous through an upside-down attitude. k = 1 is the identity.
// 傾きの強調: 機体の下向き軸と鉛直下向きのなす角 th を th' = pi*(1-(1-th/pi)^k) に写す。
// 小さい傾きでは k 倍、pi を超えず、上下反転でも連続。k = 1 なら恒等。
function mapTilt(th, k) {
  if (k === 1) return th;
  return Math.PI * (1 - Math.pow(1 - th / Math.PI, k));
}

// q' = qtilt(th') * (qtilt(th)^-1 * q): the tilt of the thrust axis is exaggerated, the turn about it (heading) is kept.
// 推力軸の傾きだけを強調し、その軸まわりの回転（機首の向き）は変えない。
function exaggerate(q, k, out) {
  const zb = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
  const th = Math.acos(clamp(zb.dot(NED_DOWN), -1, 1));
  if (th < 1e-6 || k === 1) return out.copy(q);
  const axis = new THREE.Vector3().crossVectors(NED_DOWN, zb).normalize();
  const qt = new THREE.Quaternion().setFromAxisAngle(axis, th);
  const qtk = new THREE.Quaternion().setFromAxisAngle(axis, mapTilt(th, k));
  const resid = qt.clone().invert().multiply(q);
  return out.copy(qtk).multiply(resid);
}

// Scene orientation of a body-FLU object whose attitude is `qNed` (body FRD -> NED).
// 機体 FLU の物体の描画上の向き（姿勢 `qNed` は機体 FRD -> NED）。
function sceneQuaternion(qNed, out) {
  _m.makeRotationFromQuaternion(qNed);
  _m.premultiply(M_NED_TO_SCENE).multiply(M_FLU_TO_FRD);
  return out.setFromRotationMatrix(_m);
}

// ------------------------------------------------------------------ state
let D = null;                      // replay data / 再生データ
let floorH = 0, halfH = 0, extent = 0.5, spN = 0, spE = 0;
let main = null; const views = {};
let trail = null;
const spin = [0, 0, 0, 0];
const rig = { tx: 0, ty: 0, tz: 0, init: false };
let lastStepped = -1, curFrame = 0;
const camera = new THREE.PerspectiveCamera(FOV_DEG, MAIN_W / H, 0.02, 80);
camera.layers.set(L_MAIN);

function posToScene(i, lift) {
  const p = D.pos[i];
  return new THREE.Vector3(p[1], -p[2] - floorH + lift, -p[0]);
}

// ------------------------------------------------------------------ scene
function quadLine(pos, idx, x0, z0, x1, z1, w, y) {
  const dx = x1 - x0, dz = z1 - z0, len = Math.hypot(dx, dz) || 1, nx = -dz / len * w / 2, nz = dx / len * w / 2;
  const b = pos.length / 3;
  pos.push(x0 + nx, y, z0 + nz, x0 - nx, y, z0 - nz, x1 - nx, y, z1 - nz, x1 + nx, y, z1 + nz);
  idx.push(b, b + 1, b + 2, b, b + 2, b + 3);
}

// Floor grid every 10 cm (every 50 cm thicker) as thin quads. 10 cm 格子（50 cm ごとに太く）。
function buildFloor() {
  const normal = { pos: [], idx: [] }, major = { pos: [], idx: [] };
  const n = Math.round(extent / 0.1);
  for (let i = -n; i <= n; i++) {
    const g = (i % 5 === 0) ? major : normal, w = (i % 5 === 0) ? 0.0034 : 0.0016, v = i * 0.1;
    quadLine(g.pos, g.idx, spE - extent, -spN + v, spE + extent, -spN + v, w, 0.0004);
    quadLine(g.pos, g.idx, spE + v, -spN - extent, spE + v, -spN + extent, w, 0.0004);
  }
  for (const [g, alpha] of [[normal, 0.26], [major, 0.55]]) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(g.pos, 3));
    geo.setIndex(g.idx);
    scene.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: COL.chalk, transparent: true, opacity: alpha,
      side: THREE.DoubleSide, depthWrite: false })));
  }
}

// Yellow ring and cross at the setpoint. 目標点の黄色い輪と十字。
function buildSetpointMark() {
  const mat = new THREE.MeshBasicMaterial({ color: COL.yellow, side: THREE.DoubleSide });
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.052, 0.062, 40), mat);
  ring.rotation.x = -Math.PI / 2; ring.position.set(spE, 0.0012, -spN); scene.add(ring);
  for (const r of [0, Math.PI / 2]) {
    const bar = new THREE.Mesh(new THREE.PlaneGeometry(0.15, 0.007), mat);
    bar.rotation.x = -Math.PI / 2; bar.rotation.z = r; bar.position.set(spE, 0.0012, -spN); scene.add(bar);
  }
}

// North / East letters just outside the floor. 床のすぐ外の「北」「東」の文字。
function buildCompass() {
  const label = (text, x, z) => {
    const cv = document.createElement('canvas'); cv.width = 256; cv.height = 128;
    const g = cv.getContext('2d');
    g.font = '700 84px "Hiragino Sans","Noto Sans CJK JP","Yu Gothic","Meiryo",sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillStyle = '#f2efe6'; g.fillText(text, 128, 66);
    const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false }));
    sp.scale.set(0.2, 0.1, 1); sp.position.set(x, 0.05, z); scene.add(sp);
  };
  label('北', spE, -spN - extent - 0.1);
  label('東', spE + extent + 0.1, -spN);
}

let shadow, stalk;
function buildShadowAndStalk() {
  shadow = new THREE.Mesh(new THREE.CircleGeometry(0.05 * OPT.vscale, 28),
    new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.5, depthWrite: false }));
  shadow.rotation.x = -Math.PI / 2; scene.add(shadow);
  stalk = new THREE.Mesh(new THREE.CylinderGeometry(0.0016, 0.0016, 1, 6),
    new THREE.MeshBasicMaterial({ color: COL.chalk, transparent: true, opacity: 0.35 }));
  scene.add(stalk);
}

// Trail: a thin 5-sided tube through the (decimated) positions; the colour fades
// toward the background with age, recoloured every frame. It is drawn first and
// writes no depth, so the vehicle is always drawn over it.
// 軌跡: 間引いた位置を通る細い 5 角形の管。古いほど背景色へ薄れる（毎コマ色を付け直す）。
// 最初に描き、奥行きを書き込まないので、機体はいつも軌跡の上に描かれる。
function buildTrail() {
  const stride = Math.max(1, Math.ceil(D.t.length / TRAIL_MAX_POINTS)), SIDES = 5;
  const frames = []; for (let i = 0; i < D.t.length; i += stride) frames.push(i);
  const pts = frames.map((i) => posToScene(i, halfH));
  const pos = new Float32Array(pts.length * SIDES * 3), idx = [];
  const ref = new THREE.Vector3(0, 1, 0), t = new THREE.Vector3(), u = new THREE.Vector3(), v = new THREE.Vector3();
  pts.forEach((p, i) => {
    t.subVectors(pts[Math.min(i + 1, pts.length - 1)], pts[Math.max(i - 1, 0)]);
    if (t.lengthSq() < 1e-12) t.set(1, 0, 0);
    t.normalize();
    u.crossVectors(t, Math.abs(t.y) > 0.99 ? new THREE.Vector3(1, 0, 0) : ref).normalize();
    v.crossVectors(t, u);
    for (let s = 0; s < SIDES; s++) {
      const a = s / SIDES * Math.PI * 2, o = (i * SIDES + s) * 3;
      pos[o] = p.x + TRAIL_RADIUS * (Math.cos(a) * u.x + Math.sin(a) * v.x);
      pos[o + 1] = p.y + TRAIL_RADIUS * (Math.cos(a) * u.y + Math.sin(a) * v.y);
      pos[o + 2] = p.z + TRAIL_RADIUS * (Math.cos(a) * u.z + Math.sin(a) * v.z);
    }
    if (i + 1 < pts.length) for (let s = 0; s < SIDES; s++) {
      const a = i * SIDES + s, b = i * SIDES + (s + 1) % SIDES, c = a + SIDES, d = b + SIDES;
      idx.push(a, b, c, b, d, c);
    }
  });
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(pos.length), 3));
  geo.setIndex(idx);
  const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide, depthWrite: false }));
  mesh.renderOrder = -1;   // behind the vehicle / 機体より後ろ
  mesh.frustumCulled = false; scene.add(mesh);
  trail = { mesh, frames, SIDES };
}

const cyanColor = new THREE.Color(COL.cyan), boardColor = new THREE.Color(COL.board);
function updateTrail(k) {
  let n = 0; while (n < trail.frames.length && trail.frames[n] <= k) n++;
  const col = trail.mesh.geometry.attributes.color, c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    const ageS = (k - trail.frames[i]) / D.meta.fps;
    const a = Math.max(TRAIL_MIN_ALPHA, 1 - ageS / TRAIL_FADE_S);
    c.copy(boardColor).lerp(cyanColor, a);
    for (let s = 0; s < trail.SIDES; s++) col.setXYZ(i * trail.SIDES + s, c.r, c.g, c.b);
  }
  col.needsUpdate = true;
  trail.mesh.geometry.setDrawRange(0, Math.max(0, n - 1) * trail.SIDES * 6);
}

// Small views: one vehicle clone per view (own layer), orthographic cameras, reference line, heading arrow.
// Each view looks from the negative side of a body axis toward its positive side: roll from
// behind (-X to +X), pitch from the left (-Y to +Y), yaw from above (-Z to +Z, FRD).
// Scene axes: +X east, +Y up, -Z north; with the heading removed the nose points north.
// 小画面: 画面ごとに機体のクローンを 1 機（専用レイヤー）、正射影カメラ、水平の基準線、機首の矢印。
// 各画面は機体軸の負の側から正の向きに見る: ロールは後ろから（-X → +X）、ピッチは左から
// （-Y → +Y）、ヨーは上から（-Z → +Z、FRD）。シーンの軸は +X 東・+Y 上・-Z 北で、機首の向きを除くと機首は北を向く。
const VIEW_DEFS = {
  front: { layer: L_VIEW.front, el: 'view-front', pos: [0, 0, 1], up: [0, 1, 0], heading: true },   // from behind / 後ろから
  side: { layer: L_VIEW.side, el: 'view-side', pos: [-1, 0, 0], up: [0, 1, 0], heading: true },    // from the left / 左から
  top: { layer: L_VIEW.top, el: 'view-top', pos: [0, 1, 0], up: [0, 0, -1], heading: false },
};
function buildViews() {
  for (const [name, d] of Object.entries(VIEW_DEFS)) {
    const cam = new THREE.OrthographicCamera(-VIEW_HALF, VIEW_HALF, VIEW_HALF, -VIEW_HALF, 0.1, 5);
    cam.position.set(...d.pos); cam.up.set(...d.up); cam.lookAt(0, 0, 0); cam.layers.set(d.layer);
    const veh = buildVehicle(d.layer, 1);
    const extra = new THREE.Group();
    if (d.heading) {   // horizontal reference line behind the vehicle / 機体の後ろの水平な基準線
      const line = new THREE.Mesh(new THREE.PlaneGeometry(1, 0.0014),
        new THREE.MeshBasicMaterial({ color: 0xb8c4bb, transparent: true, opacity: 0.8 }));
      const back = new THREE.Vector3(...d.pos).multiplyScalar(-0.5);
      line.position.copy(back);
      line.lookAt(new THREE.Vector3(...d.pos));
      extra.add(line);
    } else {           // heading arrow lying over the vehicle / 機体の上に置く機首方向の矢印
      const s = new THREE.Shape();
      s.moveTo(0, 0.056); s.lineTo(-0.014, 0.036); s.lineTo(-0.0045, 0.036); s.lineTo(-0.0045, 0.006);
      s.lineTo(0.0045, 0.006); s.lineTo(0.0045, 0.036); s.lineTo(0.014, 0.036); s.closePath();
      const arrow = new THREE.Mesh(new THREE.ShapeGeometry(s),
        new THREE.MeshBasicMaterial({ color: COL.yellow, depthTest: false, transparent: true, side: THREE.DoubleSide }));
      arrow.rotation.x = -Math.PI / 2;         // shape +Y -> scene -Z (north) / 形の +Y -> 北（-Z）
      arrow.renderOrder = 10;
      const holder = new THREE.Group(); holder.add(arrow); holder.position.y = 0.04; extra.add(holder);
      extra.userData.holder = holder;
    }
    extra.traverse((o) => o.layers.set(d.layer)); scene.add(extra);
    views[name] = { cam, veh, extra, def: d };
  }
}

// ----------------------------------------------------------------- per frame
// Advance the camera follow and the propeller phase by one video frame (dt = 1/fps).
// カメラの追従とプロペラの位相を動画 1 コマ（dt = 1/fps）進める。
function stepState(k) {
  const dt = 1 / D.meta.fps;
  for (const h of PROP_HUBS) {
    const duty = D.motor[k][h.motor];
    spin[h.motor] += h.dir * (SPIN_IDLE + SPIN_GAIN * duty) * dt;
  }
  const p = posToScene(k, 0);
  const follow = OPT.view !== 'top';
  const alt = p.y >= 0 ? Math.max(0.05, p.y * 0.6) : p.y;   // follow below the floor too / 床の下も追う
  const goal = follow ? [p.x, alt, p.z] : [spE, 0.18, -spN];
  const kk = rig.init ? 1 - Math.exp(-dt / CAM_TAU_S) : 1;
  rig.tx += (goal[0] - rig.tx) * kk; rig.ty += (goal[1] - rig.ty) * kk; rig.tz += (goal[2] - rig.tz) * kk;
  rig.init = true;
}

function placeCamera() {
  const el = { oblique: OBLIQUE_ELEV_DEG, top: 89, side: 6 }[OPT.view] * Math.PI / 180;
  const zoom = OPT.zoom * (OPT.view === 'side' ? 1.15 : 1);
  const fovTan = Math.tan(FOV_DEG * Math.PI / 360);
  const dist = (extent * 1.25) / (fovTan * Math.min(MAIN_W / H, 1.9)) / zoom * (OPT.view === 'top' ? 0.82 : 1);
  camera.position.set(rig.tx, rig.ty + dist * Math.sin(el), rig.tz + dist * Math.cos(el));
  camera.up.set(0, 1, 0); camera.lookAt(rig.tx, rig.ty, rig.tz);
}

const qRaw = new THREE.Quaternion(), qEx = new THREE.Quaternion(), qScene = new THREE.Quaternion(), qYaw = new THREE.Quaternion();
const NED_Z = new THREE.Vector3(0, 0, 1);

function applySpin(veh) { for (const p of veh.props) p.node.rotation.y = spin[p.motor]; }

function poseFrame(k) {
  const q = D.quat[k];
  qRaw.set(q[1], q[2], q[3], q[0]);                 // three.js order (x,y,z,w) / three.js の順
  // main vehicle / 主画面の機体
  exaggerate(qRaw, OPT.tilt, qEx);
  main.pivot.quaternion.copy(sceneQuaternion(qEx, qScene));
  main.pivot.position.copy(posToScene(k, halfH * OPT.vscale));
  applySpin(main);
  const base = posToScene(k, 0);
  shadow.position.set(base.x, 0.0008, base.z);
  // stalk between the floor and the vehicle, also below the floor / 床と機体を結ぶ線（床の下でも）
  const stalkH = Math.max(0.001, Math.abs(base.y));
  stalk.scale.set(1, stalkH, 1); stalk.position.set(base.x, base.y / 2, base.z);
  // small views / 小画面
  const yaw = yawOf(qRaw);
  for (const v of Object.values(views)) {
    let qv = qRaw;
    if (v.def.heading) {                            // remove the heading so roll / pitch read directly / 機首の向きを除く
      qYaw.setFromAxisAngle(NED_Z, -yaw); qv = qYaw.clone().multiply(qRaw);
    }
    exaggerate(qv, OPT.tilt, qEx);
    v.veh.pivot.quaternion.copy(sceneQuaternion(qEx, qScene));
    applySpin(v.veh);
    if (v.extra.userData.holder) v.extra.userData.holder.rotation.y = -yaw;   // arrow follows the heading / 矢印は機首の向きに追従
  }
}

const f = (x, d, sign) => (sign && x >= 0 ? '+' : '') + x.toFixed(d).replace('-', '−');

function voltageColor(v) {
  if (v < VOLT_LOW) return '#f4a6a6';
  if (v < VOLT_CAUTION) return '#ecdf7a';
  return '#8fd3f4';
}

let voltStart = null;
function updateHud(k) {
  const t = D.t[k], q = D.quat[k], p = D.pos[k];
  const speed = D.meta.speed;
  $('hud-tl').textContent = `t = ${t.toFixed(1)} s　実時間の ×${+speed.toPrecision(3)}`;
  $('hud-bl').textContent = `傾き ×${OPT.tilt} で表示　機体 ×${OPT.vscale}`;
  const mode = D.mode[k];
  const modeText = MODE_JA[mode] || '不明';
  $('hud-tr-1').textContent = D.meta.source;
  $('hud-tr-1b').textContent = modeText;
  $('hud-tr-2').textContent = '位置・姿勢は機体の推定値（ESKF）';
  $('hud-br').textContent = `sf log video ${D.meta.sf_version || ''}`;
  // numbers / 数値
  const dist = Math.hypot(p[0] - spN, p[1] - spE) * 100;
  $('n-time').textContent = f(t, 1);
  $('n-h').textContent = f(D.h[k], 3);
  $('n-north').textContent = f(p[0], 3, true);
  $('n-east').textContent = f(p[1], 3, true);
  $('n-dist-label').textContent = D.setpoint.source === 'ctrl_ref' ? '目標からの距離' : '離陸点からの距離';
  $('n-dist').textContent = f(dist, 1);
  $('n-roll').textContent = f(D.roll[k], 1, true);
  $('n-pitch').textContent = f(D.pitch[k], 1, true);
  $('n-yaw').textContent = f(D.yaw[k], 1, true);
  $('n-mode').textContent = modeText;
  $('n-state').textContent = STATE_JA[D.flight_state[k]] || '不明';
  const volt = D.voltage[k];
  $('n-volt').textContent = volt === null ? '—' : volt.toFixed(2);
  const m = D.motor[k], d3 = (i) => m[i].toFixed(3);
  $('n-duty-front').textContent = `FL ${d3(3)}  FR ${d3(0)}`;
  $('n-duty-rear').textContent = `RL ${d3(2)}  RR ${d3(1)}`;
  // Duty bars / Duty の棒
  const mean = (m[0] + m[1] + m[2] + m[3]) / 4;
  MOTOR_CELLS.forEach((c, i) => {
    const cell = $('bar-grid').children[i];
    cell.querySelector('.bar-fill').style.height = (clamp(m[c.idx], 0, 1) * 100) + '%';
    cell.querySelector('.bar-mean').style.bottom = (clamp(mean, 0, 1) * 100) + '%';
    cell.querySelector('.bar-val').textContent = d3(c.idx);
  });
  // battery voltage bar / 電池電圧の棒
  const frac = (x) => clamp((x - VOLT_MIN) / (VOLT_MAX - VOLT_MIN), 0, 1) * 100 + '%';
  const fill = $('volt-fill');
  if (volt === null) { fill.style.height = '0'; $('volt-val').textContent = '— V'; $('volt-val').style.color = ''; }
  else {
    fill.style.height = frac(volt); fill.style.background = voltageColor(volt);
    $('volt-val').textContent = `${volt.toFixed(2)} V`;
    $('volt-val').style.color = volt < VOLT_LOW ? '#f4a6a6' : (volt < VOLT_CAUTION ? '#ecdf7a' : '');
  }
}

function setupBars() {
  const grid = $('bar-grid');
  for (const c of MOTOR_CELLS) {
    const cell = document.createElement('div'); cell.className = 'bar-cell';
    cell.innerHTML = `<div class="bar-track"><div class="bar-fill"></div><div class="bar-mean"></div></div>
      <div class="bar-text"><div class="bar-name">${c.name}<span class="bar-dir"> ${c.m}</span></div>
      <div class="bar-dir">${c.dir}</div><div class="bar-val"></div></div>`;
    grid.appendChild(cell);
  }
  const frac = (x) => clamp((x - VOLT_MIN) / (VOLT_MAX - VOLT_MIN), 0, 1) * 100 + '%';
  $('volt-low').style.bottom = frac(VOLT_LOW);
  const first = D.voltage.find((x) => x !== null);
  voltStart = first === undefined ? null : first;
  if (voltStart === null) $('volt-start').style.display = 'none'; else $('volt-start').style.bottom = frac(voltStart);
  document.querySelectorAll('.tiltnote').forEach((e) => { e.textContent = `傾き ×${OPT.tilt}`; });
}

// ----------------------------------------------------------------- drawing
function rectOf(el) { const r = el.getBoundingClientRect(); return [Math.round(r.left), Math.round(H - r.bottom), Math.round(r.width), Math.round(r.height)]; }
function drawRegion(rect, clearColor, cam) {
  renderer.setViewport(...rect); renderer.setScissor(...rect);
  renderer.setClearColor(clearColor, 1);
  renderer.clear();
  if (cam) renderer.render(scene, cam);
}
function renderAll() {
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, W, H); renderer.setClearColor(0x1b2420, 1); renderer.clear();
  renderer.setScissorTest(true);
  drawRegion([0, 0, MAIN_W, H], COL.board, camera);
  for (const v of Object.values(views)) drawRegion(rectOf($(v.def.el)), COL.panel, v.cam);
}

// Set the replay to frame k (state is replayed from frame 0 when k is not the next frame,
// so the result never depends on call order) and draw exactly once.
// 再生をコマ k にして 1 回だけ描く（k が次のコマでなければ 0 から状態を再計算するので、
// 呼び出し順に結果が依存しない）。
function showFrame(k) {
  k = clamp(Math.round(k), 0, D.t.length - 1);
  if (k !== lastStepped + 1) {
    spin.fill(0); rig.init = false; lastStepped = -1;
    for (let j = 0; j <= k; j++) stepState(j);
  } else stepState(k);
  lastStepped = k; curFrame = k;
  poseFrame(k); updateTrail(k); placeCamera(); updateHud(k);
  renderAll();
}

// ------------------------------------------------------------------- boot
const rafQueue = [];
if (OPT.record) window.requestAnimationFrame = (cb) => { rafQueue.push(cb); return rafQueue.length; };

async function boot() {
  D = await (await fetch(OPT.data)).json();
  await loadParts();
  // vehicle half height along body up (unscaled) / 機体の上下方向の半分の高さ（拡大前）
  const box = new THREE.Box3();
  for (const name of PART_NAMES) { partGeometry[name].computeBoundingBox(); box.union(partGeometry[name].boundingBox); }
  halfH = (box.max.y - box.min.y) / 2 * 0.001;           // STL Y = body up, mm -> m
  // The floor is fixed at h = 0 (the estimator's ground). A negative estimate is drawn below
  // the floor on purpose, so an estimation fault shows up in the video.
  // 床は h = 0（推定の地面）に固定する。推定がマイナスなら機体を床の下に描く（推定の異常が動画で見えるように）。
  floorH = 0;
  spN = D.setpoint.ned[0]; spE = D.setpoint.ned[1];
  let ext = 0; for (const p of D.pos) ext = Math.max(ext, Math.abs(p[0] - spN), Math.abs(p[1] - spE));
  extent = clamp(Math.ceil((ext + 0.12) / 0.1) * 0.1, 0.5, 3.0);
  buildFloor(); buildSetpointMark(); buildCompass(); buildShadowAndStalk(); buildTrail();
  main = buildVehicle(L_MAIN, OPT.vscale); buildViews(); setupBars();
  showFrame(0);
  window.__frame = (k) => {
    showFrame(k);
    const q = rafQueue.splice(0); for (const cb of q) cb(k * 1000 / D.meta.fps);
  };
  window.__nframes = D.t.length;
  if (!OPT.record) startPlayback();
  window.__ready = true;
}

// Interactive preview (not used when recording): play in real time.
// 画面での確認用（録画時は使わない）: 実時間で再生する。
function startPlayback() {
  let t0 = performance.now();
  const loop = (now) => {
    const k = Math.floor((now - t0) / 1000 * D.meta.fps) % D.t.length;
    if (k !== curFrame) showFrame(k);
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

boot().catch((e) => { console.error('replay boot failed', e); window.__error = String(e); });
