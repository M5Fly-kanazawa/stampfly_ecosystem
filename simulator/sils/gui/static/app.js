// SPDX-License-Identifier: MIT — StampFly Ecosystem SILS GUI frontend.
// Scenario builder + parameter panel + interactive Plotly graphs + live three.js 3D.
// シナリオ作成・パラメータ・グラフ・ライブ3D。バックエンドは simulator/sils/gui/server.py。
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

// ============================================================================ i18n
// UI strings in 中文 / English / 日本語. Static HTML carries data-i18n* keys; dynamic
// strings go through L(key). The choice is kept per browser (localStorage); default
// follows the browser language. A missing key falls back to Japanese, then the key.
// UI 文字列（中文 / English / 日本語）。静的 HTML は data-i18n* キー、動的文字列は L(key)
// 経由。選択はブラウザごとに保存（localStorage）、既定はブラウザ言語。未定義キーは日本語→キー名へ。
const I18N = {
  zh: {
    language: '语言', scenario: '剧本', scenario_title: '已保存的剧本', duration: '时长[s]',
    noise: '噪声', battery: '电池模型', run: '▶ 运行',
    tab_builder: '编排剧本', tab_params: '参数',
    builder_hint: '在时间轴上添加事件来编排飞机的动作。<code>+</code> = 紧接上一个事件之后。',
    ev_rc: 'rc（摇杆）', ev_rc_ramp: 'rc_ramp（扫描）', ev_wind: 'wind（阵风扰动）',
    ev_fault: 'fault（电机故障）', ev_bias: 'bias（IMU 零偏）', ev_handle: 'handle（拿起放下）',
    add_event: '＋ 添加事件', save_name_ph: '保存名（例如 my_test）', save_scn: '💾 保存 .scn',
    show_scn: '查看 .scn', params_hint: '只有改过的参数会用于本次运行（无需重新编译）。',
    clear_changes: '清除修改', param_search_ph: '🔍 按参数名筛选',
    anim_title: '飞行动画', anim_hint: '（拖动旋转视角 / 双指缩放）',
    scene_idle: '选择剧本后点"▶ 运行"即可回放', trail: '轨迹',
    exag_title: '机体实际尺寸 82mm，相对飞行范围很小，可以放大显示', airframe: '机体', actual_size: '实际尺寸',
    graphs_title: '图表', graphs_hint: '（可缩放 · 悬停看数值 · 时间光标同步）',
    checks_title: '判定结果', run_log: '运行日志（末尾）',
    err_3d: '3D 错误：', new_scenario: '— 新建（从空白开始）—', load_failed: '读取失败：',
    f_thr: '油门', f_roll: '横滚', f_pitch: '俯仰', f_yaw: '偏航', f_hold_ms: '保持ms', f_axis: '轴',
    f_gust_ms: '阵风ms', f_motor: '电机0-3', f_gain: '增益', f_height: '高度m', f_place_x: '放置x',
    f_place_y: '放置y', f_lift_ms: '拿起ms', f_carry_ms: '搬运ms', f_place_ms: '放下ms',
    ch_rc: 'rc 摇杆', ch_rc_ramp: 'rc_ramp 扫描', ch_wind: 'wind 阵风', ch_fault: 'fault 电机故障',
    ch_bias: 'bias IMU 零偏', ch_handle: 'handle 拿起放下',
    drag_reorder: '拖动排序', at_title: '时间：0=绝对毫秒 / +=紧接上一事件 / +500=上一事件后 500ms',
    delete: '删除', note: '备注', default: '默认',
    pg_rate: '角速度控制', pg_attitude: '姿态控制', pg_altitude: '高度控制', pg_position: '位置控制',
    pg_eskf: '估计器 ESKF', pg_safety: '安全', pg_calibration: '校准', pg_estimator: '估计器选择', pg_other: '其他',
    running: '运行中…', running_msg: '运行中… 正在进行物理仿真', error: '错误', request_error: '通信错误：',
    failed: '失败', no_expect: '（没有 .expect → 按退出码判定）', no_checks: '无检查项',
    alt_true: '高度 真值', estimate: '估计', roll_true: 'roll 真值', roll_est: 'roll 估计',
    pitch_true: 'pitch 真值', pitch_est: 'pitch 估计',
    plot_alt: '高度 [m]', plot_att: '姿态 roll/pitch [deg]', plot_motor: '电机 duty [0-1]',
    no_trajectory: '这次运行没有轨迹数据', preview_unavailable: '（无法生成预览）',
    params_cleared: '已清除参数修改', enter_name: '请输入保存名', save_failed: '保存失败：', saved: '已保存：',
  },
  en: {
    language: 'Language', scenario: 'Scenario', scenario_title: 'Saved scenarios', duration: 'Duration [s]',
    noise: 'Noise', battery: 'Battery model', run: '▶ Run',
    tab_builder: 'Build scenario', tab_params: 'Parameters',
    builder_hint: 'Add events to the timeline to script the flight. <code>+</code> = right after the previous event.',
    ev_rc: 'rc (sticks)', ev_rc_ramp: 'rc_ramp (sweep)', ev_wind: 'wind (gust)',
    ev_fault: 'fault (motor failure)', ev_bias: 'bias (IMU bias)', ev_handle: 'handle (pick up & place)',
    add_event: '+ Add event', save_name_ph: 'Save as (e.g. my_test)', save_scn: '💾 Save .scn',
    show_scn: 'Show .scn', params_hint: 'Only changed parameters apply to the run (no rebuild needed).',
    clear_changes: 'Clear changes', param_search_ph: '🔍 Filter by parameter name',
    anim_title: 'Flight animation', anim_hint: '(drag to orbit / pinch to zoom)',
    scene_idle: 'Pick a scenario and press "▶ Run" to play it', trail: 'Trail',
    exag_title: 'The airframe is 82 mm, small next to the flight; you can enlarge it', airframe: 'Airframe',
    actual_size: 'True size',
    graphs_title: 'Charts', graphs_hint: '(zoom · hover for values · time cursor synced)',
    checks_title: 'Checks', run_log: 'Run log (tail)',
    err_3d: '3D error: ', new_scenario: '— New (blank) —', load_failed: 'Load failed: ',
    f_thr: 'Throttle', f_roll: 'Roll', f_pitch: 'Pitch', f_yaw: 'Yaw', f_hold_ms: 'hold ms', f_axis: 'axis',
    f_gust_ms: 'gust ms', f_motor: 'motor 0-3', f_gain: 'gain', f_height: 'height m', f_place_x: 'place x',
    f_place_y: 'place y', f_lift_ms: 'lift ms', f_carry_ms: 'carry ms', f_place_ms: 'place ms',
    ch_rc: 'rc sticks', ch_rc_ramp: 'rc_ramp sweep', ch_wind: 'wind gust', ch_fault: 'fault motor failure',
    ch_bias: 'bias IMU bias', ch_handle: 'handle pick & place',
    drag_reorder: 'Drag to reorder', at_title: 'Time: 0 = absolute ms / + = right after previous / +500 = 500 ms after previous',
    delete: 'Delete', note: 'Note', default: 'default',
    pg_rate: 'Rate control', pg_attitude: 'Attitude control', pg_altitude: 'Altitude control',
    pg_position: 'Position control', pg_eskf: 'Estimator ESKF', pg_safety: 'Safety',
    pg_calibration: 'Calibration', pg_estimator: 'Estimator select', pg_other: 'Other',
    running: 'Running…', running_msg: 'Running… physics simulation in progress', error: 'Error',
    request_error: 'Request error: ', failed: 'Failed', no_expect: '(no .expect → judged by exit code)',
    no_checks: 'No checks',
    alt_true: 'alt true', estimate: 'estimate', roll_true: 'roll true', roll_est: 'roll est',
    pitch_true: 'pitch true', pitch_est: 'pitch est',
    plot_alt: 'Altitude [m]', plot_att: 'Attitude roll/pitch [deg]', plot_motor: 'Motor duty [0-1]',
    no_trajectory: 'This run has no trajectory', preview_unavailable: '(preview unavailable)',
    params_cleared: 'Parameter changes cleared', enter_name: 'Please enter a name', save_failed: 'Save failed: ',
    saved: 'Saved: ',
  },
  ja: {
    language: '言語', scenario: 'シナリオ', scenario_title: '保存済みシナリオ', duration: '時間[s]',
    noise: 'ノイズ', battery: '電池モデル', run: '▶ 実行',
    tab_builder: 'シナリオ作成', tab_params: 'パラメータ',
    builder_hint: 'タイムラインにイベントを足して機体の動きを作ります。<code>+</code>＝直前イベントの後。',
    ev_rc: 'rc（スティック）', ev_rc_ramp: 'rc_ramp（掃引）', ev_wind: 'wind（外乱風）',
    ev_fault: 'fault（モータ故障）', ev_bias: 'bias（IMUバイアス）', ev_handle: 'handle（拾い上げ）',
    add_event: '＋ イベント追加', save_name_ph: '保存名 (例 my_test)', save_scn: '💾 .scn 保存',
    show_scn: '.scn を表示', params_hint: '変更したパラメータだけが走行に反映されます（再ビルド不要）。',
    clear_changes: '変更をクリア', param_search_ph: '🔍 パラメータ名で絞り込み',
    anim_title: '飛行アニメーション', anim_hint: '（ドラッグで視点回転 / 2本指でズーム）',
    scene_idle: 'シナリオを選んで「▶ 実行」を押すと再生されます', trail: '軌跡',
    exag_title: '機体は実寸82mm。飛行に対し小さいので拡大表示も選べます', airframe: '機体', actual_size: '実寸',
    graphs_title: 'グラフ', graphs_hint: '（拡大縮小・ホバーで値表示・時刻カーソル同期）',
    checks_title: '合否判定', run_log: '実行ログ（末尾）',
    err_3d: '3D エラー: ', new_scenario: '— 新規（空から作る）—', load_failed: '読込失敗: ',
    f_thr: 'ｽﾛｯﾄﾙ', f_roll: 'ﾛｰﾙ', f_pitch: 'ﾋﾟｯﾁ', f_yaw: 'ﾖｰ', f_hold_ms: '保持ms', f_axis: '軸',
    f_gust_ms: '突風ms', f_motor: 'モータ0-3', f_gain: 'ゲイン', f_height: '高さm', f_place_x: '置x',
    f_place_y: '置y', f_lift_ms: '持上ms', f_carry_ms: '運搬ms', f_place_ms: '設置ms',
    ch_rc: 'rc スティック', ch_rc_ramp: 'rc_ramp 掃引', ch_wind: 'wind 外乱風', ch_fault: 'fault モータ故障',
    ch_bias: 'bias IMUバイアス', ch_handle: 'handle 拾い上げ',
    drag_reorder: 'ドラッグで並べ替え', at_title: '時刻: 0=絶対ms / +=直前の後 / +500=後500ms',
    delete: '削除', note: 'メモ', default: '既定',
    pg_rate: 'レート制御', pg_attitude: '姿勢制御', pg_altitude: '高度制御', pg_position: '位置制御',
    pg_eskf: '推定器 ESKF', pg_safety: '安全', pg_calibration: '校正', pg_estimator: '推定器選択', pg_other: 'その他',
    running: '実行中…', running_msg: '実行中… 物理シミュレーションを動かしています', error: 'エラー',
    request_error: '通信エラー: ', failed: '失敗', no_expect: '（.expect 無し → exit code 判定）',
    no_checks: 'チェックなし',
    alt_true: '高度 真値', estimate: '推定', roll_true: 'roll 真', roll_est: 'roll 推',
    pitch_true: 'pitch 真', pitch_est: 'pitch 推',
    plot_alt: '高度 [m]', plot_att: '姿勢 roll/pitch [deg]', plot_motor: 'モータ duty [0-1]',
    no_trajectory: 'この走行には軌跡がありません', preview_unavailable: '(プレビュー生成不可)',
    params_cleared: 'パラメータ変更をクリア', enter_name: '保存名を入れてください', save_failed: '保存失敗: ',
    saved: '保存しました: ',
  },
};
const LANG_NAMES = { zh: '中文', en: 'English', ja: '日本語' };
const LANG_STORE_KEY = 'sils_gui_lang';

// Pick the UI language: saved choice → browser language → English.
// UI 言語を決める: 保存済みの選択 → ブラウザ言語 → 英語。
function detectLang() {
  try {
    const saved = localStorage.getItem(LANG_STORE_KEY);
    if (saved && I18N[saved]) return saved;
  } catch (e) { /* storage blocked → fall through / 保存領域が使えない → 次へ */ }
  const nav = (navigator.language || '').toLowerCase();
  if (nav.startsWith('zh')) return 'zh';
  if (nav.startsWith('ja')) return 'ja';
  return 'en';
}
const LANG = detectLang();

// Translate one key (fallback: Japanese, then the key itself).
// キーを1つ翻訳する（未定義なら日本語、それも無ければキー名）。
function L(key) { return I18N[LANG][key] ?? I18N.ja[key] ?? key; }

// Apply translations to the static HTML and build the language selector.
// 静的 HTML に翻訳を適用し、言語セレクタを組み立てる。
function applyStaticI18n() {
  document.documentElement.lang = { zh: 'zh-CN', en: 'en', ja: 'ja' }[LANG];
  document.querySelectorAll('[data-i18n]').forEach(el => { el.textContent = L(el.dataset.i18n); });
  document.querySelectorAll('[data-i18n-html]').forEach(el => { el.innerHTML = L(el.dataset.i18nHtml); });
  document.querySelectorAll('[data-i18n-placeholder]').forEach(el => { el.placeholder = L(el.dataset.i18nPlaceholder); });
  document.querySelectorAll('[data-i18n-title]').forEach(el => { el.title = L(el.dataset.i18nTitle); });
  const sel = $('langSel');
  sel.innerHTML = Object.entries(LANG_NAMES)
    .map(([code, name]) => `<option value="${code}">${name}</option>`).join('');
  sel.value = LANG;
  sel.onchange = () => {
    try { localStorage.setItem(LANG_STORE_KEY, sel.value); } catch (e) { /* not persisted / 保存不可 */ }
    location.reload();   // simplest full re-render / 最も単純な全再描画
  };
}

function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2200);
}
// Surface a JS error to the 3D message overlay (and console) instead of failing silently.
// JS エラーを無視せず 3D メッセージ欄とコンソールに出す。
window.addEventListener('error', (e) => {
  console.error('PAGEERR', e.message, (e.filename || '') + ':' + (e.lineno || ''));
  const m = $('scene-msg'); if (m) { m.style.display = 'flex'; m.textContent = L('err_3d') + e.message; }
});

// ============================================================================ boot
async function boot() {
  applyStaticI18n();
  S.scenarios = await api.get('/api/scenarios');
  S.params = await api.get('/api/params');
  const sel = $('scnSelect');
  sel.innerHTML = `<option value="">${L('new_scenario')}</option>` +
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
  if (d.error) { toast(L('load_failed') + d.error); return; }
  S.events = d.events; S.currentName = name; S.currentMode = 'saved';
  $('durSec').value = Math.round((d.duration_us || 25e6) / 1e6);
  renderEvents();
}

// Field schema per channel — drives the builder form. 各チャネルの編集フィールド定義。
const FIELDS = {
  rc: [['thr', 'f_thr', 2048], ['roll', 'f_roll', 2048], ['pitch', 'f_pitch', 2048], ['yaw', 'f_yaw', 2048],
       ['arm', 'arm', 0], ['hold_ms', 'f_hold_ms', 1000], ['rate_hz', 'Hz', 50],
       ['alt', 'ALT', 0], ['acro', 'ACRO', 0], ['pos', 'POS', 0]],
  rc_ramp: [['field', 'f_axis', 'throttle'], ['from', 'from', 2048], ['to', 'to', 3000],
       ['step', 'step', 10], ['rate_hz', 'Hz', 50], ['arm', 'arm', 1], ['alt', 'ALT', 0], ['acro', 'ACRO', 0]],
  wind: [['fx', 'Fx[N]', 0], ['fy', 'Fy[N]', 0], ['fz', 'Fz↓[N]', 0], ['dur_ms', 'f_gust_ms', 0]],
  fault: [['motor', 'f_motor', 0], ['gain', 'f_gain', 1.0]],
  bias: [['ax', 'ax', 0], ['ay', 'ay', 0], ['az', 'az', 0], ['gx', 'gx', 0], ['gy', 'gy', 0], ['gz', 'gz', 0]],
  handle: [['carry_alt', 'f_height', 0.4], ['px', 'f_place_x', 0], ['py', 'f_place_y', 0],
       ['lift_ms', 'f_lift_ms', 1200], ['carry_ms', 'f_carry_ms', 1200], ['place_ms', 'f_place_ms', 1200]],
};
// Field labels above are i18n keys (plain tokens like 'arm'/'Hz' pass through L() unchanged).
// 上のフィールド表示名は i18n キー（'arm'/'Hz' 等の素の語は L() をそのまま素通り）。
const chLabel = (ch) => { const k = 'ch_' + ch, v = L(k); return v === k ? ch : v; };

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
      <span class="grip" title="${L('drag_reorder')}" draggable="true">⠿</span>
      <input class="at" value="${e.at ?? '+'}" title="${L('at_title')}"/>
      <span class="ch">${chLabel(e.ch)}</span>
      <button class="del" title="${L('delete')}">✕</button>
    </div>
    <div class="event-fields">${fields.map(([k, lbl, dv]) =>
      `<label>${L(lbl)}<input data-k="${k}" value="${e[k] ?? dv}"/></label>`).join('')}</div>
    <input class="comment" data-k="comment" placeholder="${L('note')}" value="${e.comment || ''}"/>`;
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
    det.innerHTML = `<summary>${groupLabel(g, ps)} (${ps.length})</summary>`;
    ps.forEach(p => det.appendChild(paramRow(p)));
    wrap.appendChild(det);
  });
}
// Group title by the params' name prefix (e.g. 'rate.'), falling back to the server title.
// グループ名は param 名の接頭辞（例 'rate.'）で翻訳し、無ければサーバの表示名を使う。
function groupLabel(serverTitle, ps) {
  const key = 'pg_' + String(ps[0].name).split('.')[0], v = L(key);
  return v === key ? (serverTitle.includes('/') ? L('pg_other') : serverTitle) : v;
}
function paramRow(p) {
  const div = document.createElement('div');
  div.className = 'param'; div.dataset.name = p.name;
  const isBool = p.type === 'BOOL';
  const cur = S.paramOverrides[p.name] ?? p.default;
  div.innerHTML = `<div><div class="pname">${p.name}</div>
      <div class="pmeta">${p.type} ${L('default')} ${fmt(p.default)} · [${fmt(p.min)}, ${fmt(p.max)}]</div></div>`;
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
  setVerdict('run', L('running'));
  $('scene-msg').textContent = L('running_msg');
  $('scene-msg').style.display = 'flex';
  const sel = $('scnSelect').value;
  // If the events were edited (or it's a new scenario), run as custom; else run the saved file.
  const mode = (S.currentMode === 'saved' && sel) ? 'saved' : 'custom';
  const req = {
    mode, name: sel || null, events: S.events,
    target: 'vehicle',
    duration_us: Math.round(parseFloat($('durSec').value) * 1e6),
    noise: $('noiseSel').value, seed: 12345, battery: $('battChk').checked,
    params: S.paramOverrides,
  };
  let r;
  try { r = await api.post('/api/run', req); }
  catch (err) { setVerdict('bad', L('error')); toast(L('request_error') + err); btn.disabled = false; return; }
  btn.disabled = false;
  if (r.error) { setVerdict('bad', L('failed')); toast(r.error); $('scene-msg').textContent = r.error; return; }
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
  setVerdict(pass ? 'ok' : 'bad', pass ? `✅ ${passN}/${total}` : (r.ok ? `❌ ${passN}/${total}` : '❌ ' + L('failed')));
  $('gateSummary').textContent = total ? `${passN}/${total} PASS` : L('no_expect');
  $('checks').innerHTML = checks.map(c => {
    const cls = c.skipped ? 'skip' : (c.pass ? 'pass' : 'fail');
    const badge = c.skipped ? 'SKIP' : (c.pass ? 'PASS' : 'FAIL');
    return `<div class="check ${cls}"><span class="badge">${badge}</span>
      <span>${c.name}</span><span class="detail">${c.detail || ''}</span></div>`;
  }).join('') || `<div class="muted">${L('no_checks')}</div>`;
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
    line(d.alt, L('alt_true'), '#22d3ee'), line(d.alt_est, L('estimate'), '#a78bfa', 'dot'),
  ], PLOT_LAYOUT(L('plot_alt')), PLOT_CFG);
  Plotly.react('graphAtt', [
    line(d.roll, L('roll_true'), '#22d3ee'), line(d.roll_est, L('roll_est'), '#0ea5b7', 'dot'),
    line(d.pitch, L('pitch_true'), '#fbbf24'), line(d.pitch_est, L('pitch_est'), '#b8860b', 'dot'),
  ], PLOT_LAYOUT(L('plot_att')), PLOT_CFG);
  Plotly.react('graphMotor', [
    line(d.m0, 'M1', '#34d399'), line(d.m1, 'M2', '#22d3ee'),
    line(d.m2, 'M3', '#a78bfa'), line(d.m3, 'M4', '#f87171'),
  ], PLOT_LAYOUT(L('plot_motor')), PLOT_CFG);
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
// Drone display scale. DEFAULT = 1.0 = TRUE SCALE: the StampFly is drawn at its real ~82 mm
// size relative to the flight, so altitudes/drifts read honestly (the chase cam + zoom keep
// the small craft in view). The toolbar "機体" selector can EXAGGERATE it (×3…×20) to make
// attitude/props easier to read — that is an optional aid, not the default. 既定は実寸(×1)。
// 飛行に対し正直なスケール。ツールバーの選択で拡大は任意に。
let modelScale = 1.0;

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
  // Close default so the true-scale (~82 mm) craft is clearly visible at the origin; the
  // chase target follows it in flight and the user can zoom out to see the whole path.
  // 実寸機体が原点で見える近距離。飛行中はチェイス追従、ズームで全体も見られる。
  camera.position.set(0.34, 0.26, 0.34);
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
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  // Lighting ported from the landing page so the StampFly materials read the same way.
  // landing page と同じライティングで StampFly の材質を同じ見え方に。
  scene.add(new THREE.AmbientLight(0x8895b5, 0.9));
  const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(4, 8, 5); scene.add(key);
  const fill = new THREE.DirectionalLight(0xc9d4ff, 0.6); fill.position.set(-4, 3, -4); scene.add(fill);
  // Subtle cyan/violet accents (kept low so the frame still reads near its true light-grey).
  // フレームが本来の薄灰に見えるようアクセント光は控えめに。
  const cyan = new THREE.PointLight(0x22d3ee, 12, 14); cyan.position.set(-2, 1.5, 2); scene.add(cyan);
  const violet = new THREE.PointLight(0xa855f7, 12, 14); violet.position.set(2, 1, -2); scene.add(violet);

  // worldGroup maps ENU local coords (x=East, y=North, z=Up) to three.js Y-up display:
  // rotating -90° about X sends a local (x,y,z) to world (x, z, -y) = (E, U, -N). Inside it
  // we use ENU coordinates directly. worldGroup が ENU→three(Y上)を担う(-90°X)。
  worldGroup = new THREE.Group(); worldGroup.rotation.x = -Math.PI / 2; scene.add(worldGroup);

  // Ground grid in the ENU x-y plane (z=0). GridHelper lies in three's x-z plane, so rotate
  // it into x-y. 地面グリッドを ENU 水平面(z=0)に。
  const grid = new THREE.GridHelper(8, 32, 0x2a3a55, 0x182336); grid.rotation.x = Math.PI / 2;
  worldGroup.add(grid);
  // ENU axes helper (E=red, N=green, U=blue), ~5 cm so it does not dwarf the true-scale craft.
  // 座標軸(東赤/北緑/上青)。実寸機体を圧迫しないよう約5cm。
  worldGroup.add(new THREE.AxesHelper(0.05));

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

// =============================================================================
// StampFly model — ported from the landing page (landing/index.html), which was built
// from photos of the real aircraft: the actual STL body parts + true-to-life 3-blade
// propellers (paddle outline + linear twist). Everything is authored in the landing/STL
// NATIVE frame (millimetres, X=left, Y=up, Z=fwd); we then drop the whole model into the
// BODY FLU group with scale 0.001 (mm→m) + quat (0.5,0.5,0.5,0.5), the identical transform
// the MuJoCo MJCF uses, so it sits in the body frame and the per-frame attitude quaternion
// orients it (props point down when it flips). landing page と同一の実機準拠モデルを移植
// （実STLパーツ＋実機そっくりの3枚羽根）。landing/STL native(mm,X左/Y上/Z前)で作り、MJCF と
// 同じ scale0.001+quat(0.5,0.5,0.5,0.5)で機体FLUへ。姿勢quatが正しく回す。
// -----------------------------------------------------------------------------
const PART_MAT = {
  frame:           new THREE.MeshStandardMaterial({ color: 0xccd2db, roughness: 0.5,  metalness: 0.1 }),
  pcb:             new THREE.MeshStandardMaterial({ color: 0x0e1014, roughness: 0.55, metalness: 0.2 }),
  m5stamps3:       new THREE.MeshStandardMaterial({ color: 0xff6a00, roughness: 0.4,  metalness: 0.1 }),
  battery:         new THREE.MeshStandardMaterial({ color: 0x2a1d14, roughness: 0.6,  metalness: 0.1 }),
  battery_adapter: new THREE.MeshStandardMaterial({ color: 0x33312d, roughness: 0.6,  metalness: 0.1 }),
  motor_fl:        new THREE.MeshStandardMaterial({ color: 0xb9c1cb, roughness: 0.35, metalness: 0.85 }),
  motor_fr:        new THREE.MeshStandardMaterial({ color: 0xb9c1cb, roughness: 0.35, metalness: 0.85 }),
  motor_rl:        new THREE.MeshStandardMaterial({ color: 0xb9c1cb, roughness: 0.35, metalness: 0.85 }),
  motor_rr:        new THREE.MeshStandardMaterial({ color: 0xb9c1cb, roughness: 0.35, metalness: 0.85 }),
};
const bladeMat = new THREE.MeshStandardMaterial({ color: 0xff2b3d, transparent: true, opacity: 0.62,
  roughness: 0.18, metalness: 0.0, emissive: 0x4a0008, emissiveIntensity: 0.35, side: THREE.DoubleSide });
const hubMat = new THREE.MeshStandardMaterial({ color: 0x8e0512, roughness: 0.3, metalness: 0.15 });

// makeBlade: paddle outline with a linear twist root→tip (landing page, verbatim).
// パドル形状＋線形ねじれの羽根（landing page と同一）。
function makeBlade(L, Wm, thick, twistRoot, twistTip) {
  const s = new THREE.Shape();
  s.moveTo(0, 0.20 * Wm);
  s.bezierCurveTo(0.40 * L, 0.50 * Wm, 0.65 * L, 0.50 * Wm, 0.86 * L, 0.30 * Wm);
  s.quadraticCurveTo(L, 0.16 * Wm, L, 0.0);
  s.quadraticCurveTo(L, -0.12 * Wm, 0.86 * L, -0.20 * Wm);
  s.bezierCurveTo(0.65 * L, -0.58 * Wm, 0.40 * L, -0.58 * Wm, 0, -0.24 * Wm);
  s.closePath();
  const geo = new THREE.ExtrudeGeometry(s, { depth: thick, bevelEnabled: false });
  geo.translate(0, 0, -thick / 2);
  const p = geo.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
    const a = twistRoot + (twistTip - twistRoot) * Math.min(1, Math.max(0, x / L));
    const c = Math.cos(a), sn = Math.sin(a);
    p.setXYZ(i, x, y * c - z * sn, y * sn + z * c);
  }
  geo.rotateX(Math.PI / 2);     // lay the blade flat → the prop spins about its local Y
  geo.computeVertexNormals();
  return geo;
}
const PROP_RADIUS = 14.99;      // mm (landing page)
const HUB_R = PROP_RADIUS * 0.22;
const bladeGeo = makeBlade(PROP_RADIUS - HUB_R * 0.6, (PROP_RADIUS - HUB_R * 0.6) / 2.7, 0.6, 0.38, 0.10);
const hubGeo = new THREE.CylinderGeometry(HUB_R, HUB_R * 0.9, 2.6, 24);
const domeGeo = new THREE.SphereGeometry(HUB_R * 0.85, 20, 12, 0, Math.PI * 2, 0, Math.PI / 2);

// A true-to-life 3-blade prop (landing page makeProp). Spins about its local Y (= body up
// once the model is rotated into FLU). 実機準拠の3枚羽根（local Y 軸回りに回転）。
function makeProp() {
  const prop = new THREE.Group();
  prop.add(new THREE.Mesh(hubGeo, hubMat));
  const dome = new THREE.Mesh(domeGeo, hubMat); dome.position.y = 1.0; prop.add(dome);
  for (let i = 0; i < 3; i++) {
    const blade = new THREE.Mesh(bladeGeo, bladeMat);
    blade.rotation.y = i * (Math.PI * 2 / 3);
    blade.translateX(HUB_R * 0.6);
    prop.add(blade);
  }
  return prop;
}

// Prop hubs in the landing/STL NATIVE frame (mm, X=left, Y=up, Z=fwd) + the motor duty
// (m0..m3 = M1 FR/M2 RR/M3 RL/M4 FL) that drives each, in its real turn direction
// (CCW=+1 / CW=-1 about body up; MJCF: M1 FR & M3 RL CCW, M2 RR & M4 FL CW).
// プロペラハブ（landing native mm）＋駆動モータ＋実回転方向。
const PROP_HUBS = [
  { pos: [ 22.80, 7.81,  22.80], motor: 3, dir: -1 },  // FL = M4  CW
  { pos: [-22.80, 7.81,  22.80], motor: 0, dir: +1 },  // FR = M1  CCW
  { pos: [ 22.80, 7.81, -22.81], motor: 2, dir: +1 },  // RL = M3  CCW
  { pos: [-22.80, 7.81, -22.81], motor: 1, dir: -1 },  // RR = M2  CW
];
const BODY_PARTS = ['frame', 'pcb', 'm5stamps3', 'battery', 'battery_adapter',
                    'motor_fl', 'motor_fr', 'motor_rl', 'motor_rr'];

function buildDrone() {
  const g = new THREE.Group(); g.scale.setScalar(modelScale);
  // `model` holds the StampFly in the landing/STL native frame (mm); the scale+quat put it
  // into the body FLU frame at real metric. model に landing native(mm)で作り FLU へ変換。
  const model = new THREE.Group();
  model.scale.setScalar(0.001);                  // mm → m (MJCF mesh scale)
  model.quaternion.set(0.5, 0.5, 0.5, 0.5);      // native (X左/Y上/Z前) → body FLU (MJCF geom quat)
  g.add(model);
  // Propellers (procedural, synchronous). Spin about local Y (= body up after the transform).
  PROP_HUBS.forEach(h => {
    const prop = makeProp();
    prop.position.set(h.pos[0], h.pos[1] - 1.4, h.pos[2]);
    model.add(prop);
    props.push({ pivot: prop, dir: h.dir, motor: h.motor });
  });
  // STL body parts (async — pop in as they arrive). geo.computeVertexNormals so the lit
  // material is not black. STL本体（非同期）。法線生成で黒化を防ぐ。
  const loader = new STLLoader();
  BODY_PARTS.forEach(name => loader.load(`/mesh/${name}.stl`, (geo) => {
    geo.computeVertexNormals();
    model.add(new THREE.Mesh(geo, PART_MAT[name] || PART_MAT.frame));
  }, undefined, (err) => console.warn('[3D] STL load failed:', name, err)));
  return g;
}

function loadTrajectory(tr, timeline) {
  if (!tr || !tr.data || !tr.data.t || !tr.data.t.length) {
    $('scene-msg').textContent = L('no_trajectory'); $('scene-msg').style.display = 'flex';
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
    p.pivot.rotation.y += p.dir * (0.05 + duty * 1.6);   // spin about local Y (=body up); idle creep + duty
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
    pre.textContent = r.text || L('preview_unavailable'); pre.hidden = false;
  };
  $('paramSearch').oninput = (e) => filterParams(e.target.value.toLowerCase());
  $('resetParamsBtn').onclick = () => { S.paramOverrides = {}; buildParamPanel(); toast(L('params_cleared')); };
  $('playBtn').onclick = () => {
    S.playing = !S.playing; S.lastWall = performance.now();
    $('playBtn').textContent = S.playing ? '⏸' : '▶';
  };
  $('timeline').oninput = (e) => { S.playing = false; $('playBtn').textContent = '▶'; setFrame(+e.target.value); };
  $('trailChk').onchange = () => setFrame(S.frame);
  // Drone display scale (default ×1 = true scale). Rescale the model live and dolly the
  // camera by the same ratio so the craft keeps its apparent size when you switch. 機体の
  // 表示倍率（既定×1=実寸）。倍率変更時はカメラも同率ドリーして見た目の大きさを保つ。
  $('exagSel').onchange = (e) => {
    const next = parseFloat(e.target.value) || 1;
    if (drone) drone.scale.setScalar(next);
    dolly(next / modelScale);
    modelScale = next;
  };
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
  if (!name) { toast(L('enter_name')); return; }
  const r = await api.post('/api/save', { name, events: S.events,
    header: `${name}.scn — built with the SILS GUI` });
  if (r.error) { toast(L('save_failed') + r.error); return; }
  toast(L('saved') + r.path);
  S.scenarios = await api.get('/api/scenarios');
  const sel = $('scnSelect');
  sel.innerHTML = `<option value="">${L('new_scenario')}</option>` +
    S.scenarios.map(s => `<option value="${s.name}">${s.name}</option>`).join('');
  sel.value = name; S.currentName = name; S.currentMode = 'saved';
}

boot();
