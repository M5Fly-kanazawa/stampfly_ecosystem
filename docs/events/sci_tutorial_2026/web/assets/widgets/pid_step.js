/**
 * pid_step.js — rate-loop step-response widget for the SCI/SICE tutorial deck.
 * レートループのステップ応答ウィジェット（SCI/SICEチュートリアル用）。
 *
 * Plant (deck "プラントモデル"/"実測パラメータ" frames,
 * docs/events/sci_tutorial_2026/slides/chapters/sci_s4_pid.tex:81-128):
 *   G_p(s) = K / (s (tau_m s + 1))   -- duty -> body rate, first-order motor
 *                                        lag (tau_m) cascaded with the
 *                                        rigid-body integrator (1/(I s))
 *   K = Km / I  [rad/s^2 per unit duty], tau_m = 0.02 s (measured 0.0163 s)
 *   Roll K=102, Pitch K=70, Yaw K=8.0        (sci_s4_pid.tex:105-109)
 * A generic transport delay L is added ahead of the plant to represent the
 * loop's dead time (sensor + control compute + PWM update), per the deck's
 * "無駄時間の影響" frame: tau_d ~= 5 ms (sci_s4_pid.tex:198-201) and the
 * "自動チューニング" frame's G_p(s)=b*e^{-Ls}/(s(Ts+1)) ETFE form
 * (sci_s4_pid.tex:541-545), with b=K and T=tau_m from the table above.
 * 一般的なむだ時間 L をプラントの手前に加え、ループの遅れ（センサ+制御演算+PWM
 * 更新）を表現する（「無駄時間の影響」フレーム: tau_d ≈ 5ms、
 * sci_s4_pid.tex:198-201、および「自動チューニング」フレームの
 * G_p(s)=b*e^{-Ls}/(s(Ts+1)) というETFE形式、sci_s4_pid.tex:541-545。
 * b=K, T=tau_m は上表の値）。
 *
 * Controller: engineering-form PID, u = Kp*(e + (1/Ti)*Integral(e) + Td*de/dt),
 * with the derivative filtered by a first-order low-pass of time constant
 * eta*Td (deck "PID 工学形式パラメータ" frame, sci_s4_pid.tex:377-405; eta in
 * 0.1-0.2). Output is saturated to +-1 (firmware/vehicle/components/
 * sf_controller_pid/include/pid.hpp:53 output_limit default, and the deck's
 * PID-block frame). Integral uses trapezoidal (Tustin-style) accumulation
 * with conditional-integration anti-windup, adapted from pid.hpp's algorithm
 * (pid.hpp:96-125) but with a simple Euler/low-pass derivative instead of
 * pid.hpp's exact bilinear (Tustin) derivative filter -- close enough for a
 * teaching widget, and noted here rather than silently glossed over.
 * 制御器: 工学形式PID、u = Kp*(e + (1/Ti)*∫e + Td*de/dt)。微分項は時定数
 * eta*Td の一次遅れフィルタを通す（「PID 工学形式パラメータ」フレーム、
 * eta=0.1〜0.2）。出力は±1に制限（pid.hpp:53 の output_limit 既定値、および
 * デッキのPIDブロック図）。積分は台形則（Tustin風）＋条件付き積分アンチワインド
 * アップ（pid.hpp:96-125 のアルゴリズムを踏襲）だが、微分フィルタは pid.hpp の
 * 厳密な双一次変換ではなく単純な一次遅れ（オイラー近似）で代用している —
 * 教材用ウィジェットとしては十分だが、黙って一致させたことにはしない。
 *
 * D-on-error (as in the engineering-form textbook formula above) creates a
 * derivative "kick" the instant the setpoint steps -- the exact problem the
 * vehicle firmware avoids by differentiating the MEASUREMENT instead (D-on-M,
 * pid.hpp:20-30, deck "D項とアンチワインドアップ" frame, sci_s4_pid.tex:
 * 408-441). The "D-on-M (vehicle実装)" checkbox switches to that mode so the
 * kick can be shown and then made to disappear.
 * D-on-error（上の教科書的な工学形式の式どおり）は、目標値が階段状に変わった
 * 瞬間に微分キックを生む — これこそ vehicle ファームが「測定値」を微分する
 * D-on-M で回避している問題（pid.hpp:20-30、「D項とアンチワインドアップ」
 * フレーム）。「D-on-M (vehicle実装)」チェックボックスでこのモードに切り替え、
 * キックが出る様子・消える様子の両方を見せられる。
 *
 * Lesson 5 / Lesson 8 preset gains (direct Kp/Ki/Kd -> engineering-form
 * Ti=Kp/Ki, Td=Kd/Kp, per sci_s4_pid.tex:395):
 *   Lesson 5 (P only):  Kp_roll=Kp_pitch=0.5, Kp_yaw=2.0   (sci_s4_pid.tex:245)
 *   Lesson 8 (PID):     Roll  (Kp,Ki,Kd)=(0.25,0.3,0.005)
 *                        Pitch (Kp,Ki,Kd)=(0.36,0.3,0.005)
 *                        Yaw   (Kp,Ki,Kd)=(2.0, 0.5,0.01)   (sci_s4_pid.tex:400)
 *
 * API: window.SfWidgets.pidStep(container, options) -> instance
 *   options: { axis: 'roll'|'pitch'|'yaw' (default 'roll'),
 *              preset: 'lesson5'|'lesson8' (default 'lesson8'),
 *              demo: boolean (cycles axis/preset for headless verification) }
 *   instance: { dispose(), setAxis(axis), setPreset(name) }
 */
(function (global) {
  "use strict";

  // ---- Deck constants (see file header for source lines) -----------------
  var AXES = {
    roll:  { label: "Roll",  K: 102, I: 9.16e-6, Km: 9.3e-4 },
    pitch: { label: "Pitch", K: 70,  I: 13.3e-6, Km: 9.3e-4 },
    yaw:   { label: "Yaw",   K: 8.0, I: 20.4e-6, Km: 1.6e-4 },
  };
  var TAU_M = 0.02;        // s, common motor time constant (design value; measured 0.0163s)
  var L_DELAY = 0.005;     // s, loop dead time (sci_s4_pid.tex:201)
  var DT = 1 / 400;        // s, 400 Hz control loop (sci_s4_pid.tex:444-464)
  var OUTPUT_LIMIT = 1.0;  // pid.hpp:53 default
  var ETA_DEFAULT = 0.125; // pid.hpp:52 default (deck range 0.1-0.2)
  var T_SPAN = 1.5;        // s, plot window per spec

  function gains(axis, presetName) {
    if (presetName === "lesson5") {
      var kp5 = axis === "yaw" ? 2.0 : 0.5;
      return { kp: kp5, ti: 0, td: 0, useI: false, useD: false };
    }
    // lesson8: direct (Kp,Ki,Kd) -> engineering form (Kp,Ti,Td)
    var direct = {
      roll:  { kp: 0.25, ki: 0.3, kd: 0.005 },
      pitch: { kp: 0.36, ki: 0.3, kd: 0.005 },
      yaw:   { kp: 2.0,  ki: 0.5, kd: 0.01 },
    }[axis];
    return {
      kp: direct.kp,
      ti: direct.kp / direct.ki,
      td: direct.kd / direct.kp,
      useI: true,
      useD: true,
    };
  }

  function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

  // ---- Simulation ----------------------------------------------------------
  // Sampled-data (400 Hz) step response of the delayed plant under the
  // engineering-form PID above. Euler integration for the plant, matching the
  // "simple Euler + delay line" spec.
  // 遅延プラントの400Hzサンプル値ステップ応答（オイラー積分＋遅延バッファ）。
  function simulate(cfg) {
    var K = cfg.K, kp = cfg.kp, ti = cfg.ti, td = cfg.td;
    var useI = cfg.useI, useD = cfg.useD, dOnM = cfg.dOnM, eta = cfg.eta;
    var n = Math.round(T_SPAN / DT) + 1;
    var nDelay = Math.max(1, Math.round(L_DELAY / DT));
    var uBuf = new Float64Array(nDelay);
    var bufIdx = 0;
    var m = 0, y = 0, integral = 0, prevError = 0, prevMeas = 0, dFilt = 0;
    var t = new Float64Array(n), yArr = new Float64Array(n), uArr = new Float64Array(n);

    for (var k = 0; k < n; k++) {
      t[k] = k * DT;
      yArr[k] = y; // measurement sampled before this tick's control action

      var error = 1 - y;

      // Derivative term (D-on-E or D-on-M), eta*Td first-order filter.
      var dTerm = 0;
      if (useD && td > 0) {
        var src = dOnM ? y : error;
        var prevSrc = dOnM ? prevMeas : prevError;
        var raw = (src - prevSrc) / DT;
        var tauF = Math.max(eta * td, DT * 0.1);
        dFilt += (DT / (tauF + DT)) * (raw - dFilt);
        dTerm = kp * td * dFilt;
      }
      prevMeas = y;

      var pTerm = kp * error;

      var iTerm = integral;
      if (useI && ti > 0) {
        var iNext = integral + (kp / ti) * (error + prevError) * DT * 0.5;
        var outTest = pTerm + iNext + dTerm;
        var pushHigh = outTest > OUTPUT_LIMIT && error > 0;
        var pushLow = outTest < -OUTPUT_LIMIT && error < 0;
        if (!pushHigh && !pushLow) integral = iNext;
        integral = clamp(integral, -OUTPUT_LIMIT, OUTPUT_LIMIT);
        iTerm = integral;
      }
      prevError = error;

      var u = clamp(pTerm + iTerm + dTerm, -OUTPUT_LIMIT, OUTPUT_LIMIT);
      uArr[k] = u;

      var uDelayed = uBuf[bufIdx];
      uBuf[bufIdx] = u;
      bufIdx = (bufIdx + 1) % nDelay;
      m += (DT / TAU_M) * (uDelayed - m);
      y += DT * K * m;
    }
    return { t: t, y: yArr, u: uArr };
  }

  function analyzeResponse(t, y) {
    var peak = -Infinity, peakIdx = 0, trough = Infinity;
    for (var k = 0; k < y.length; k++) {
      if (y[k] > peak) { peak = y[k]; peakIdx = k; }
      if (y[k] < trough) trough = y[k];
    }
    var band = 0.02;
    var settleIdx = -1;
    for (var j = y.length - 1; j >= 0; j--) {
      if (Math.abs(y[j] - 1) > band) { settleIdx = j + 1; break; }
    }
    return {
      peak: peak,
      trough: trough,
      peakTime: t[peakIdx],
      overshootPct: Math.max(0, (peak - 1) * 100),
      settleTime: settleIdx >= 0 && settleIdx < t.length ? t[settleIdx] : null,
    };
  }

  // ---- DOM helpers -----------------------------------------------------
  function el(tag, cls, html) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (html != null) e.innerHTML = html;
    return e;
  }

  function makeSlider(labelText, id, min, max, step, value, fmt) {
    var wrap = el("div", "sfw-control");
    var labelRow = el("div", "sfw-control-label");
    var lab = el("span", null, labelText);
    var val = el("span", "sfw-val", fmt(value));
    labelRow.appendChild(lab);
    labelRow.appendChild(val);
    var input = document.createElement("input");
    input.type = "range";
    input.id = id;
    input.min = String(min);
    input.max = String(max);
    input.step = String(step);
    input.value = String(value);
    // Keep arrow-key adjustments local to the slider -- don't let them
    // bubble to the deck's document-level keydown handler (arrow = page nav).
    // 矢印キーでの調整はスライダー内で完結させ、デッキ側のページ送り
    // （矢印キー）に伝播させない。
    input.addEventListener("keydown", function (ev) { ev.stopPropagation(); });
    wrap.appendChild(labelRow);
    wrap.appendChild(input);
    return { wrap: wrap, input: input, valEl: val };
  }

  // `onResize` is called after every resize (including the very first one)
  // so the caller can redraw -- a container can legitimately change size
  // after mount (browser resize, slide re-layout), and simply resizing the
  // backing buffer without redrawing leaves a blank canvas.
  // `onResize` は最初の1回も含め毎回のリサイズ後に呼ばれる。コンテナは
  // マウント後にもサイズが変わりうる（ブラウザのリサイズ、スライドの
  // 再レイアウト）ため、バッファをリサイズするだけで再描画しないと
  // キャンバスが白紙のまま残ってしまう。
  function setupCanvas(wrap, onResize) {
    var canvas = document.createElement("canvas");
    wrap.appendChild(canvas);
    var ctx = canvas.getContext("2d");
    function resize() {
      var rect = wrap.getBoundingClientRect();
      var dpr = global.devicePixelRatio || 1;
      var w = Math.max(1, Math.floor(rect.width));
      var h = Math.max(1, Math.floor(rect.height));
      canvas.width = Math.floor(w * dpr);
      canvas.height = Math.floor(h * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      canvas._cssW = w;
      canvas._cssH = h;
      if (onResize) onResize();
    }
    resize();
    var ro = null;
    if (typeof ResizeObserver !== "undefined") {
      ro = new ResizeObserver(resize);
      ro.observe(wrap);
    } else {
      global.addEventListener("resize", resize);
    }
    return { canvas: canvas, ctx: ctx, resize: resize, disconnect: function () {
      if (ro) ro.disconnect(); else global.removeEventListener("resize", resize);
    } };
  }

  // ---- Widget ----------------------------------------------------------
  function mount(container, options) {
    options = options || {};
    var axis = options.axis && AXES[options.axis] ? options.axis : "roll";
    var presetName = options.preset === "lesson5" ? "lesson5" : "lesson8";
    var g0 = gains(axis, presetName);

    var state = {
      axis: axis,
      kp: g0.kp, ti: g0.ti || 0.8, td: g0.td || 0.02,
      useI: g0.useI, useD: g0.useD,
      dOnM: false,
      eta: ETA_DEFAULT,
      showPOnly: true,
    };

    var root = el("div", "sfw sfw-pid-step");
    // Attach to the live document immediately -- setupCanvas() below needs
    // getBoundingClientRect() to return real (non-zero) numbers, which only
    // works once the element is actually connected to the DOM.
    // setupCanvas() は getBoundingClientRect() が実サイズを返すことに依存する
    // ため、実DOMに接続済みでなければならない。先に接続してから中身を積む。
    container.appendChild(root);

    var head = el("div", "sfw-head");
    var title = el("div", "sfw-title", "レート制御ステップ応答");
    var seg = el("div", "sfw-seg");
    ["roll", "pitch", "yaw"].forEach(function (a) {
      var b = document.createElement("button");
      b.type = "button";
      b.textContent = AXES[a].label;
      b.className = a === state.axis ? "active" : "";
      b.addEventListener("click", function () { setAxis(a); });
      seg.appendChild(b);
    });
    head.appendChild(title);
    head.appendChild(seg);
    root.appendChild(head);

    var canvasWrap = el("div", "sfw-canvas-wrap");
    root.appendChild(canvasWrap);
    var redraw = function () {}; // reassigned to draw() once it exists below
    var cv = setupCanvas(canvasWrap, function () { redraw(); });

    var controls = el("div", "sfw-controls");
    var sKp = makeSlider("Kp", "kp", -2, 1, 0.01, Math.log10(state.kp),
      function () { return state.kp.toFixed(3); });
    var sTi = makeSlider("Ti [s]", "ti", -1.3, 0.9, 0.01, Math.log10(state.ti),
      function () { return state.ti.toFixed(3) + " s"; });
    var sTd = makeSlider("Td [s]", "td", -3, -0.3, 0.01, Math.log10(Math.max(state.td, 1e-3)),
      function () { return (state.td * 1000).toFixed(1) + " ms"; });
    var sEta = makeSlider("η (微分フィルタ)", "eta", 0.05, 0.3, 0.005, state.eta,
      function () { return state.eta.toFixed(3); });
    sEta.input.step = "0.005";
    sEta.input.min = "0.05";
    sEta.input.max = "0.3";
    sEta.input.value = String(state.eta);
    controls.appendChild(sKp.wrap);
    controls.appendChild(sTi.wrap);
    controls.appendChild(sTd.wrap);
    controls.appendChild(sEta.wrap);
    root.appendChild(controls);

    var checks = el("div", "sfw-btnrow");
    function makeCheck(label, checked) {
      var wrap = el("label", "sfw-check");
      var cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = checked;
      cb.addEventListener("keydown", function (ev) { ev.stopPropagation(); });
      wrap.appendChild(cb);
      wrap.appendChild(document.createTextNode(label));
      checks.appendChild(wrap);
      return cb;
    }
    var cbI = makeCheck("I項を使う", state.useI);
    var cbD = makeCheck("D項を使う", state.useD);
    var cbDoM = makeCheck("D-on-M (vehicle実装)", state.dOnM);
    var cbPOnly = makeCheck("Pのみを重ねる", state.showPOnly);
    root.appendChild(checks);

    var btnrow = el("div", "sfw-btnrow");
    var btn5 = el("button", "sfw-btn secondary", "実習 5 (Pのみ)");
    btn5.type = "button";
    var btn8 = el("button", "sfw-btn", "実習 8 (PID)");
    btn8.type = "button";
    btnrow.appendChild(btn5);
    btnrow.appendChild(btn8);
    root.appendChild(btnrow);

    var readouts = el("div", "sfw-readouts");
    root.appendChild(readouts);

    var note = el("div", "sfw-note",
      "G_p(s)=K/(s(τ_m s+1))・τ_m=0.02s・むだ時間 L=5ms（実測パラメータ／無駄時間の影響）。" +
      "出力は±1に飽和（pid.hpp output_limit）");
    root.appendChild(note);

    function syncSlidersFromState() {
      sKp.input.value = String(Math.log10(state.kp));
      sKp.valEl.textContent = state.kp.toFixed(3);
      sTi.input.value = String(Math.log10(Math.max(state.ti, 0.05)));
      sTi.valEl.textContent = state.ti.toFixed(3) + " s";
      sTd.input.value = String(Math.log10(Math.max(state.td, 1e-3)));
      sTd.valEl.textContent = (state.td * 1000).toFixed(1) + " ms";
      sEta.input.value = String(state.eta);
      sEta.valEl.textContent = state.eta.toFixed(3);
      cbI.checked = state.useI;
      cbD.checked = state.useD;
      cbDoM.checked = state.dOnM;
      sTi.wrap.style.opacity = state.useI ? "1" : "0.4";
      sTd.wrap.style.opacity = state.useD ? "1" : "0.4";
      sEta.wrap.style.opacity = state.useD ? "1" : "0.4";
    }

    function draw() {
      var K = AXES[state.axis].K;
      var main = simulate({
        K: K, kp: state.kp, ti: state.ti, td: state.td,
        useI: state.useI, useD: state.useD, dOnM: state.dOnM, eta: state.eta,
      });
      var pOnly = state.showPOnly
        ? simulate({ K: K, kp: state.kp, ti: 0, td: 0, useI: false, useD: false, dOnM: false, eta: state.eta })
        : null;

      var stat = analyzeResponse(main.t, main.y);
      var omegan = Math.sqrt(state.kp * K / TAU_M);
      var zeta = 1 / (2 * Math.sqrt(state.kp * K * TAU_M));
      var mp = zeta < 1 ? Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) * 100 : 0;

      var ctx = cv.ctx, w = cv.canvas._cssW, h = cv.canvas._cssH;
      ctx.clearRect(0, 0, w, h);
      var padL = 34, padR = 10, padT = 12, padB = 22;
      var plotW = w - padL - padR, plotH = h - padT - padB;
      var yMax = Math.max(1.3, stat.peak * 1.15);
      var yMin = Math.min(-0.1, stat.trough * 1.15);
      function xOf(t) { return padL + (t / T_SPAN) * plotW; }
      function yOf(v) { return padT + plotH - ((v - yMin) / (yMax - yMin)) * plotH; }

      // Grid / 補助線
      ctx.strokeStyle = "#e3edf5";
      ctx.lineWidth = 1;
      ctx.font = "11px 'Noto Sans JP', sans-serif";
      ctx.fillStyle = "#7a828a";
      for (var gx = 0; gx <= T_SPAN + 1e-6; gx += 0.25) {
        var px = xOf(gx);
        ctx.beginPath(); ctx.moveTo(px, padT); ctx.lineTo(px, padT + plotH); ctx.stroke();
        ctx.fillText(gx.toFixed(2), px - 8, h - 6);
      }
      var yStep = (yMax - yMin) > 2 ? 0.5 : 0.25;
      for (var gy = Math.ceil(yMin / yStep) * yStep; gy <= yMax; gy += yStep) {
        var py = yOf(gy);
        ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(padL + plotW, py); ctx.stroke();
        ctx.fillText(gy.toFixed(2), 2, py + 3);
      }

      // Target / 目標値
      ctx.strokeStyle = "#9aa6b0";
      ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.moveTo(padL, yOf(1)); ctx.lineTo(padL + plotW, yOf(1)); ctx.stroke();
      ctx.setLineDash([]);

      function plotCurve(t, y, color, dash) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        if (dash) ctx.setLineDash(dash); else ctx.setLineDash([]);
        ctx.beginPath();
        for (var k = 0; k < t.length; k++) {
          var X = xOf(t[k]), Y = yOf(y[k]);
          if (k === 0) ctx.moveTo(X, Y); else ctx.lineTo(X, Y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }
      if (pOnly) plotCurve(pOnly.t, pOnly.y, "rgb(80,80,80)", [5, 4]);
      plotCurve(main.t, main.y, "rgb(0,120,200)", null);

      readouts.innerHTML =
        "<span>目標=1・K=<b>" + K.toFixed(1) + "</b></span>" +
        "<span>ζ(P換算)=<b>" + zeta.toFixed(2) + "</b></span>" +
        "<span>ω_n(P換算)=<b>" + omegan.toFixed(1) + "</b> rad/s</span>" +
        "<span>理論オーバーシュート(P)=<b>" + mp.toFixed(1) + "</b>%</span>" +
        "<span>実測ピーク=<b>" + stat.peak.toFixed(2) + "</b> (t=" + stat.peakTime.toFixed(2) + "s)</span>" +
        "<span>実測オーバーシュート=<b>" + stat.overshootPct.toFixed(1) + "</b>%</span>" +
        "<span>整定(±2%)=<b>" + (stat.settleTime != null ? stat.settleTime.toFixed(2) + "s" : "—") + "</b></span>";
    }

    function setAxis(a) {
      if (!AXES[a]) return;
      state.axis = a;
      Array.prototype.forEach.call(seg.children, function (b, i) {
        b.className = ["roll", "pitch", "yaw"][i] === a ? "active" : "";
      });
      draw();
    }

    function setPreset(name) {
      var gg = gains(state.axis, name);
      state.kp = gg.kp;
      state.ti = gg.ti || state.ti || 0.8;
      state.td = gg.td || state.td || 0.02;
      state.useI = gg.useI;
      state.useD = gg.useD;
      syncSlidersFromState();
      draw();
    }

    sKp.input.addEventListener("input", function () {
      state.kp = Math.pow(10, parseFloat(sKp.input.value));
      sKp.valEl.textContent = state.kp.toFixed(3);
      draw();
    });
    sTi.input.addEventListener("input", function () {
      state.ti = Math.pow(10, parseFloat(sTi.input.value));
      sTi.valEl.textContent = state.ti.toFixed(3) + " s";
      draw();
    });
    sTd.input.addEventListener("input", function () {
      state.td = Math.pow(10, parseFloat(sTd.input.value));
      sTd.valEl.textContent = (state.td * 1000).toFixed(1) + " ms";
      draw();
    });
    sEta.input.addEventListener("input", function () {
      state.eta = parseFloat(sEta.input.value);
      sEta.valEl.textContent = state.eta.toFixed(3);
      draw();
    });
    cbI.addEventListener("change", function () {
      state.useI = cbI.checked;
      sTi.wrap.style.opacity = state.useI ? "1" : "0.4";
      draw();
    });
    cbD.addEventListener("change", function () {
      state.useD = cbD.checked;
      sTd.wrap.style.opacity = state.useD ? "1" : "0.4";
      sEta.wrap.style.opacity = state.useD ? "1" : "0.4";
      draw();
    });
    cbDoM.addEventListener("change", function () { state.dOnM = cbDoM.checked; draw(); });
    cbPOnly.addEventListener("change", function () { state.showPOnly = cbPOnly.checked; draw(); });
    btn5.addEventListener("click", function () { setPreset("lesson5"); });
    btn8.addEventListener("click", function () { setPreset("lesson8"); });

    redraw = draw;
    syncSlidersFromState();
    draw();

    var demoTimer = null;
    if (options.demo) {
      var demoStates = [
        { axis: "roll", preset: "lesson5" },
        { axis: "roll", preset: "lesson8" },
        { axis: "pitch", preset: "lesson8" },
        { axis: "yaw", preset: "lesson8" },
      ];
      var di = 0;
      function applyDemo() {
        var s = demoStates[di % demoStates.length];
        di++;
        setAxis(s.axis);
        setPreset(s.preset);
      }
      applyDemo();
      demoTimer = setInterval(applyDemo, 1800);
    }

    return {
      dispose: function () {
        cv.disconnect();
        if (demoTimer) clearInterval(demoTimer);
        if (container.contains(root)) container.removeChild(root);
      },
      setAxis: setAxis,
      setPreset: setPreset,
    };
  }

  if (!global.SfWidgets) global.SfWidgets = {};
  global.SfWidgets.pidStep = mount;
})(window);
