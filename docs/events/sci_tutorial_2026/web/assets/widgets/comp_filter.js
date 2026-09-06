/**
 * comp_filter.js — complementary-filter widget for the SCI/SICE tutorial deck.
 * 相補フィルタ・ウィジェット（SCI/SICEチュートリアル用）。
 *
 * Filter equation (deck "相補フィルタと STABILIZE" / 実習9 frames,
 * sci_s4_pid.tex:467-537, matching firmware/workshop/lessons/
 * lesson_09_estimation/solution.cpp:38-42):
 *   theta_hat[k] = alpha*(theta_hat[k-1] + omega*dt) + (1-alpha)*theta_accel
 *   alpha = 0.98 (gyro 98% + accel 2%), dt = 1/400 s
 * フィルタ式（「相補フィルタとSTABILIZE」フレーム、lesson_09 solution.cpp と同一）:
 *   theta_hat[k] = alpha*(theta_hat[k-1] + omega*dt) + (1-alpha)*theta_accel
 *   alpha = 0.98（ジャイロ98% + 加速度2%）、dt = 1/400 s
 *
 * The ground-truth roll angle, gyro measurement, and accelerometer-angle
 * measurement are all SYNTHETIC (a slow sinusoid + additive bias/noise/
 * vibration bumps) -- this part is a teaching signal generator, not sourced
 * from the deck or flight data; only the filter equation, alpha value and
 * 400 Hz sample time are the deck's real numbers.
 * 真値ロール角・ジャイロ計測・加速度角計測はすべて合成信号（緩やかな正弦波＋
 * バイアス／ノイズ／振動バンプ）— 教材用の信号生成であり、デッキやフライト
 * ログの実測値ではない。デッキの実数値なのはフィルタ式・alpha・400Hzのみ。
 *
 * API: window.SfWidgets.compFilter(container, options) -> instance
 *   options: { alpha: 0.90-0.999 (default 0.98), demo: boolean }
 *   instance: { dispose(), setAlpha(a), play(bool) }
 */
(function (global) {
  "use strict";

  var DT = 1 / 400;          // s, sci_s4_pid.tex:444-464 (400 Hz loop)
  var ALPHA_DEFAULT = 0.98;  // sci_s4_pid.tex:526, lesson_09 solution.cpp:40
  var TRUTH_AMP = 20 * Math.PI / 180;   // rad, synthetic slow-roll amplitude
  var TRUTH_FREQ = 0.15;                // Hz, synthetic slow oscillation
  var GYRO_BIAS = 0.03;                 // rad/s, synthetic constant bias
  var GYRO_NOISE_STD = 0.01;            // rad/s
  var ACCEL_NOISE_STD = 4 * Math.PI / 180; // rad
  var BUMP_PERIOD = 1.4;                // s, mean spacing of vibration bumps
  var BUMP_AMP = 10 * Math.PI / 180;    // rad
  var BUMP_DECAY = 0.06;                // s
  var HISTORY_SEC = 6;                  // s, rolling plot window

  // Box-Muller gaussian noise (deterministic seed via mulberry32 so repeated
  // mounts/demo cycles are reproducible for screenshot verification).
  // Box-Muller正規乱数（mulberry32で決定論的にシード、スクリーンショット
  // 確認の再現性を確保）。
  function mulberry32(seed) {
    return function () {
      seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
      var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function makeGaussian(rng) {
    var spare = null;
    return function (std) {
      if (spare !== null) { var s = spare; spare = null; return s * std; }
      var u1 = Math.max(rng(), 1e-9), u2 = rng();
      var r = Math.sqrt(-2 * Math.log(u1));
      spare = r * Math.sin(2 * Math.PI * u2);
      return r * Math.cos(2 * Math.PI * u2) * std;
    };
  }

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
    input.addEventListener("keydown", function (ev) { ev.stopPropagation(); });
    wrap.appendChild(labelRow);
    wrap.appendChild(input);
    return { wrap: wrap, input: input, valEl: val };
  }

  // `onResize` fires after every resize (including the first) so the caller
  // can redraw -- if playback is paused, a later container resize would
  // otherwise leave a stale/blank canvas until the next simulation step.
  // `onResize` は最初の1回も含め毎回のリサイズ後に呼ばれる。一時停止中に
  // コンテナがリサイズされても、再描画しないとキャンバスが白紙のまま残る。
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

  function mount(container, options) {
    options = options || {};

    var rng = mulberry32(12345);
    var gauss = makeGaussian(rng);

    var state = {
      alpha: options.alpha != null ? options.alpha : ALPHA_DEFAULT,
      playing: options.autoplay !== false,
      t: 0,
      nextBumpT: BUMP_PERIOD * (0.5 + rng()),
      bumpAmp: 0,
      bumpDecayT: 0,
      truth: 0,
      cfEst: 0,
      gyroEst: 0,
      sumSqErrGyro: 0, sumSqErrAccel: 0, sumSqErrCf: 0, nRms: 0,
      history: [], // {t, truth, gyroEst, accelAngle, cfEst}
    };

    var root = el("div", "sfw sfw-comp-filter");
    // Connect to the live document immediately so getBoundingClientRect()
    // (used by setupCanvas below) returns real numbers.
    // setupCanvas が使う getBoundingClientRect() が実サイズを返すよう、
    // 先にDOMへ接続してから中身を積む。
    container.appendChild(root);

    var head = el("div", "sfw-head");
    head.appendChild(el("div", "sfw-title", "相補フィルタ"));
    var btnPlay = el("button", "sfw-btn play", "一時停止");
    btnPlay.type = "button";
    head.appendChild(btnPlay);
    root.appendChild(head);

    var canvasWrap = el("div", "sfw-canvas-wrap");
    root.appendChild(canvasWrap);
    var redraw = function () {}; // reassigned to draw() once it exists below
    var cv = setupCanvas(canvasWrap, function () { redraw(); });

    var controls = el("div", "sfw-controls");
    var sAlpha = makeSlider("α", "alpha", 0.90, 0.999, 0.001, state.alpha, function (v) { return v.toFixed(3); });
    controls.appendChild(sAlpha.wrap);
    root.appendChild(controls);

    var btnrow = el("div", "sfw-btnrow");
    var btnReset = el("button", "sfw-btn secondary", "リセット");
    btnReset.type = "button";
    btnrow.appendChild(btnReset);
    root.appendChild(btnrow);

    var legend = el("div", "sfw-legend");
    legend.innerHTML =
      '<span><span class="dot" style="background:rgb(0,60,100)"></span>真値</span>' +
      '<span><span class="dot" style="background:rgb(220,50,30)"></span>ジャイロ積分のみ（ドリフト）</span>' +
      '<span><span class="dot" style="background:rgb(80,80,80)"></span>加速度のみ（ノイズ）</span>' +
      '<span><span class="dot" style="background:rgb(0,120,200)"></span>相補フィルタ推定値</span>';
    root.appendChild(legend);

    var readouts = el("div", "sfw-readouts");
    root.appendChild(readouts);

    var note = el("div", "sfw-note",
      "θ̂_k=α(θ̂_{k-1}+ωΔt)+(1-α)θ_accel・Δt=1/400s（実習9・lesson_09 solution.cpp:38-42）。" +
      "真値・ジャイロ・加速度は教材用の合成信号");
    root.appendChild(note);

    function step() {
      var t = state.t;
      var truth = TRUTH_AMP * Math.sin(2 * Math.PI * TRUTH_FREQ * t);
      var truthRate = TRUTH_AMP * 2 * Math.PI * TRUTH_FREQ * Math.cos(2 * Math.PI * TRUTH_FREQ * t);

      // Occasional vibration bump on the accel-angle channel.
      // 加速度チャンネルに時々のる振動バンプ
      if (t >= state.nextBumpT) {
        state.bumpAmp = BUMP_AMP * (rng() * 2 - 1);
        state.bumpDecayT = t;
        state.nextBumpT = t + BUMP_PERIOD * (0.5 + rng());
      }
      var bump = state.bumpAmp * Math.exp(-(t - state.bumpDecayT) / BUMP_DECAY);

      var gyroMeas = truthRate + GYRO_BIAS + gauss(GYRO_NOISE_STD);
      var accelAngle = truth + gauss(ACCEL_NOISE_STD) + bump;

      state.gyroEst += gyroMeas * DT;
      state.cfEst = state.alpha * (state.cfEst + gyroMeas * DT) + (1 - state.alpha) * accelAngle;

      state.sumSqErrGyro += (state.gyroEst - truth) * (state.gyroEst - truth);
      state.sumSqErrAccel += (accelAngle - truth) * (accelAngle - truth);
      state.sumSqErrCf += (state.cfEst - truth) * (state.cfEst - truth);
      state.nRms++;

      state.history.push({ t: t, truth: truth, gyroEst: state.gyroEst, accelAngle: accelAngle, cfEst: state.cfEst });
      var cutoff = t - HISTORY_SEC;
      while (state.history.length && state.history[0].t < cutoff) state.history.shift();

      state.t += DT;
    }

    function fastForward(seconds) {
      var nSteps = Math.round(seconds / DT);
      for (var i = 0; i < nSteps; i++) step();
    }

    function draw() {
      var ctx = cv.ctx, w = cv.canvas._cssW, h = cv.canvas._cssH;
      ctx.clearRect(0, 0, w, h);
      if (state.history.length < 2) return;

      var tMax = state.history[state.history.length - 1].t;
      var tMin = tMax - HISTORY_SEC;
      var padL = 40, padR = 10, padT = 12, padB = 20;
      var plotW = w - padL - padR, plotH = h - padT - padB;

      var degMax = 40;
      function xOf(t) { return padL + ((t - tMin) / HISTORY_SEC) * plotW; }
      function yOf(deg) { return padT + plotH / 2 - (deg / degMax) * (plotH / 2); }

      ctx.strokeStyle = "#e3edf5";
      ctx.lineWidth = 1;
      ctx.font = "11px 'Noto Sans JP', sans-serif";
      ctx.fillStyle = "#7a828a";
      for (var gy = -degMax; gy <= degMax; gy += 20) {
        var py = yOf(gy);
        ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(padL + plotW, py); ctx.stroke();
        ctx.fillText(gy + "°", 2, py + 3);
      }

      function plot(key, color, lw) {
        ctx.strokeStyle = color;
        ctx.lineWidth = lw || 2;
        ctx.beginPath();
        state.history.forEach(function (p, i) {
          var X = xOf(p.t), Y = yOf(p[key] * 180 / Math.PI);
          if (i === 0) ctx.moveTo(X, Y); else ctx.lineTo(X, Y);
        });
        ctx.stroke();
      }
      plot("accelAngle", "rgb(80,80,80)", 1);
      plot("gyroEst", "rgb(220,50,30)", 1.5);
      plot("truth", "rgb(0,60,100)", 2);
      plot("cfEst", "rgb(0,120,200)", 2);

      var n = Math.max(1, state.nRms);
      var rmsGyro = Math.sqrt(state.sumSqErrGyro / n) * 180 / Math.PI;
      var rmsAccel = Math.sqrt(state.sumSqErrAccel / n) * 180 / Math.PI;
      var rmsCf = Math.sqrt(state.sumSqErrCf / n) * 180 / Math.PI;
      readouts.innerHTML =
        "<span>α=<b>" + state.alpha.toFixed(3) + "</b></span>" +
        "<span>RMS誤差 ジャイロのみ=<b>" + rmsGyro.toFixed(1) + "</b>°</span>" +
        "<span>RMS誤差 加速度のみ=<b>" + rmsAccel.toFixed(1) + "</b>°</span>" +
        "<span>RMS誤差 相補フィルタ=<b>" + rmsCf.toFixed(1) + "</b>°</span>";
    }

    var rafId = null;
    var lastFrameMs = null;
    var accSec = 0;
    function frame(nowMs) {
      if (!state.playing) { rafId = null; return; }
      if (lastFrameMs == null) lastFrameMs = nowMs;
      var deltaSec = Math.min(0.1, (nowMs - lastFrameMs) / 1000);
      lastFrameMs = nowMs;
      accSec += deltaSec;
      var nSteps = Math.floor(accSec / DT);
      nSteps = Math.min(nSteps, 40); // guard against huge catch-up after a stall
      for (var i = 0; i < nSteps; i++) step();
      accSec -= nSteps * DT;
      draw();
      rafId = requestAnimationFrame(frame);
    }
    function startLoop() {
      if (rafId == null) {
        lastFrameMs = null;
        rafId = requestAnimationFrame(frame);
      }
    }
    function stopLoop() {
      if (rafId != null) { cancelAnimationFrame(rafId); rafId = null; }
    }

    function setAlpha(a) {
      state.alpha = Math.max(0.90, Math.min(0.999, a));
      sAlpha.input.value = String(state.alpha);
      sAlpha.valEl.textContent = state.alpha.toFixed(3);
    }
    function play(on) {
      state.playing = on;
      btnPlay.textContent = on ? "一時停止" : "再生";
      if (on) startLoop(); else { stopLoop(); draw(); }
    }
    function reset() {
      state.t = 0; state.gyroEst = 0; state.cfEst = 0;
      state.sumSqErrGyro = 0; state.sumSqErrAccel = 0; state.sumSqErrCf = 0; state.nRms = 0;
      state.history = [];
      state.bumpAmp = 0; state.nextBumpT = BUMP_PERIOD * 0.5;
      draw();
    }

    sAlpha.input.addEventListener("input", function () { setAlpha(parseFloat(sAlpha.input.value)); });
    btnPlay.addEventListener("click", function () { play(!state.playing); });
    btnReset.addEventListener("click", reset);

    redraw = draw;
    var demoTimer = null;
    if (options.demo) {
      fastForward(4);
      play(true);
      demoTimer = setInterval(function () {
        setAlpha(state.alpha > 0.95 ? 0.90 : 0.995);
      }, 2200);
    } else {
      draw();
      if (state.playing) startLoop();
    }

    return {
      dispose: function () {
        stopLoop();
        cv.disconnect();
        if (demoTimer) clearInterval(demoTimer);
        if (container.contains(root)) container.removeChild(root);
      },
      setAlpha: setAlpha,
      play: play,
    };
  }

  if (!global.SfWidgets) global.SfWidgets = {};
  global.SfWidgets.compFilter = mount;
})(window);
