/**
 * mixer.js — X-quad motor mixer widget for the SCI/SICE tutorial deck.
 * X型クアッドのモータミキサ・ウィジェット（SCI/SICEチュートリアル用）。
 *
 * Reproduces ws::motor_mixer bit-for-bit
 * (firmware/workshop/main/ws_internal.hpp:142-151, resolve_motor_request()):
 *   out[0] = T + 0.25*(-R + P + Y) / 3.7   // M1 FR (CCW)
 *   out[1] = T + 0.25*(-R - P - Y) / 3.7   // M2 RR (CW)
 *   out[2] = T + 0.25*( R - P + Y) / 3.7   // M3 RL (CCW)
 *   out[3] = T + 0.25*( R + P - Y) / 3.7   // M4 FL (CW)
 *   duty clamped to [0, 1] per motor
 * 3.7 and 0.25 are legacy tuning constants carried over bit-for-bit from
 * vehicle_old (ws_internal.hpp:136-141) -- not physical quantities.
 * 3.7 と 0.25 は vehicle_old 由来の旧チューニング定数で物理量ではない
 * （ws_internal.hpp:136-141 のコメント参照）。
 *
 * Layout (front up) and CW/CCW colour convention match
 * docs/events/_shared/tikz/motor_layout.tex: M1 front-right / M2 rear-right
 * / M3 rear-left / M4 front-left; CCW motors (M1, M3) drawn in the same
 * red-orange family as motor_layout.tex's `ccwcolor`, CW motors (M2, M4) in
 * `cwcolor` = the deck's own sfblue.
 * レイアウト（前方が上）とCW/CCWの色分けは motor_layout.tex に合わせる
 * （M1前右／M2後右／M3後左／M4前左。CCW=M1,M3は ccwcolor 系、CW=M2,M4は
 * cwcolor＝デッキの sfblue）。
 *
 * API: window.SfWidgets.mixer(container, options) -> instance
 *   options: { T,R,P,Y: initial slider values, demo: boolean }
 *   instance: { dispose(), setValues({T,R,P,Y}) }
 */
(function (global) {
  "use strict";

  var MOTORS = [
    { name: "M1", pos: "FR", spin: "CCW", corner: "tr", terms: { R: -1, P: 1, Y: 1 } },
    { name: "M2", pos: "RR", spin: "CW",  corner: "br", terms: { R: -1, P: -1, Y: -1 } },
    { name: "M3", pos: "RL", spin: "CCW", corner: "bl", terms: { R: 1, P: -1, Y: 1 } },
    { name: "M4", pos: "FL", spin: "CW",  corner: "tl", terms: { R: 1, P: 1, Y: -1 } },
  ];
  var CW_COLOR = "rgb(0,120,200)";     // motor_layout.tex cwcolor == deck sfblue
  var CCW_COLOR = "rgb(220,60,20)";    // motor_layout.tex ccwcolor

  function clamp01(v) { return Math.max(0, Math.min(1, v)); }

  // Bit-for-bit port of ws::resolve_motor_request's Mixer branch.
  // ws::resolve_motor_request の Mixer 分岐を数値まで一致させて移植。
  function motorMixer(T, R, P, Y) {
    var raw = MOTORS.map(function (m) {
      return T + 0.25 * (m.terms.R * R + m.terms.P * P + m.terms.Y * Y) / 3.7;
    });
    return raw.map(clamp01);
  }
  function contribution(m, R, P, Y) {
    return 0.25 * (m.terms.R * R + m.terms.P * P + m.terms.Y * Y) / 3.7;
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
  // can redraw -- a container can resize after mount, and resizing the
  // backing buffer without redrawing leaves a blank canvas.
  // `onResize` は最初の1回も含め毎回のリサイズ後に呼ばれる。再描画しないと
  // キャンバスが白紙のまま残る。
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
    var state = {
      T: options.T != null ? options.T : 0.5,
      R: options.R != null ? options.R : 0,
      P: options.P != null ? options.P : 0,
      Y: options.Y != null ? options.Y : 0,
    };

    var root = el("div", "sfw sfw-mixer");
    // Connect to the live document immediately so getBoundingClientRect()
    // (used by setupCanvas below) returns real numbers.
    // setupCanvas が使う getBoundingClientRect() が実サイズを返すよう、
    // 先にDOMへ接続してから中身を積む。
    container.appendChild(root);

    var head = el("div", "sfw-head");
    head.appendChild(el("div", "sfw-title", "モータミキサ（X型クアッド）"));
    root.appendChild(head);

    var body = el("div", "sfw-canvas-wrap");
    body.style.display = "flex";
    root.appendChild(body);
    var redraw = function () {}; // reassigned to draw() once it exists below
    var cv = setupCanvas(body, function () { redraw(); });

    var controls = el("div", "sfw-controls");
    var sT = makeSlider("T (スロットル)", "T", 0, 1, 0.01, state.T, function (v) { return v.toFixed(2); });
    var sR = makeSlider("R (ロール)", "R", -1, 1, 0.01, state.R, function (v) { return v.toFixed(2); });
    var sP = makeSlider("P (ピッチ)", "P", -1, 1, 0.01, state.P, function (v) { return v.toFixed(2); });
    var sY = makeSlider("Y (ヨー)", "Y", -1, 1, 0.01, state.Y, function (v) { return v.toFixed(2); });
    [sT, sR, sP, sY].forEach(function (s) { controls.appendChild(s.wrap); });
    root.appendChild(controls);

    var signTable = el("div", "sfw-note");
    signTable.innerHTML =
      "符号パターン（ws::motor_mixer, ws_internal.hpp:146-149）: " +
      "M1 FR(-R+P+Y) ｜ M2 RR(-R-P-Y) ｜ M3 RL(+R-P+Y) ｜ M4 FL(+R+P-Y)";
    root.appendChild(signTable);

    var legend = el("div", "sfw-legend");
    legend.innerHTML =
      '<span><span class="dot" style="background:' + CCW_COLOR + '"></span>CCW (M1,M3)</span>' +
      '<span><span class="dot" style="background:' + CW_COLOR + '"></span>CW (M2,M4)</span>' +
      '<span><span class="dot" style="background:rgb(40,160,40)"></span>寄与 + （このR/P/Yで増える）</span>' +
      '<span><span class="dot" style="background:rgb(220,50,30)"></span>寄与 − （このR/P/Yで減る）</span>';
    root.appendChild(legend);

    function draw() {
      var duties = motorMixer(state.T, state.R, state.P, state.Y);
      var ctx = cv.ctx, w = cv.canvas._cssW, h = cv.canvas._cssH;
      ctx.clearRect(0, 0, w, h);

      var cx = w / 2, cy = h / 2;
      var armLen = Math.min(w, h) * 0.32;
      var bodyR = Math.min(w, h) * 0.06;
      var barW = Math.min(w, h) * 0.16;
      var barMaxH = Math.min(w, h) * 0.34;

      // Body cross / 機体十字
      ctx.strokeStyle = "rgb(80,80,80)";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(cx - armLen, cy - armLen); ctx.lineTo(cx + armLen, cy + armLen);
      ctx.moveTo(cx + armLen, cy - armLen); ctx.lineTo(cx - armLen, cy + armLen);
      ctx.stroke();
      ctx.fillStyle = "rgb(80,80,80)";
      ctx.beginPath(); ctx.arc(cx, cy, bodyR, 0, Math.PI * 2); ctx.fill();

      // Front arrow / 前方を示す矢印
      ctx.strokeStyle = "rgb(0,60,100)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy - armLen * 0.55); ctx.lineTo(cx, cy - armLen * 1.05);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx, cy - armLen * 1.2);
      ctx.lineTo(cx - 5, cy - armLen * 1.05);
      ctx.lineTo(cx + 5, cy - armLen * 1.05);
      ctx.closePath();
      ctx.fillStyle = "rgb(0,60,100)";
      ctx.fill();
      ctx.font = "11px 'Noto Sans JP', sans-serif";
      ctx.fillText("Front", cx - 14, cy - armLen * 1.28);

      var cornerPos = {
        tr: { x: cx + armLen, y: cy - armLen },
        br: { x: cx + armLen, y: cy + armLen },
        bl: { x: cx - armLen, y: cy + armLen },
        tl: { x: cx - armLen, y: cy - armLen },
      };

      MOTORS.forEach(function (m, idx) {
        var p = cornerPos[m.corner];
        var duty = duties[idx];
        var contrib = contribution(m, state.R, state.P, state.Y);
        var color = m.spin === "CW" ? CW_COLOR : CCW_COLOR;

        // Bar gauge (0..1 duty) anchored at the motor corner.
        // duty[0,1]を表す棒グラフ（モータ位置に配置）
        var bx = p.x - barW / 2;
        var by = p.y - barMaxH / 2;
        ctx.fillStyle = "#eef3f7";
        ctx.fillRect(bx, by, barW, barMaxH);
        var fillH = barMaxH * duty;
        ctx.fillStyle = color;
        ctx.fillRect(bx, by + (barMaxH - fillH), barW, fillH);

        // Highlight ring: green if this motor's R/P/Y contribution is
        // currently positive, red if negative.
        // 寄与の符号でリングをハイライト（正=緑／負=赤）
        var eps = 1e-4;
        if (Math.abs(contrib) > eps) {
          ctx.strokeStyle = contrib > 0 ? "rgb(40,160,40)" : "rgb(220,50,30)";
          ctx.lineWidth = 3;
          ctx.strokeRect(bx - 3, by - 3, barW + 6, barMaxH + 6);
        } else {
          ctx.strokeStyle = "#c7d3dc";
          ctx.lineWidth = 1;
          ctx.strokeRect(bx - 3, by - 3, barW + 6, barMaxH + 6);
        }

        ctx.fillStyle = color;
        ctx.font = "bold 12px 'Noto Sans JP', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(m.name + " " + m.pos, p.x, by - 8);
        ctx.font = "11px 'Noto Sans JP', sans-serif";
        ctx.fillStyle = "rgb(0,60,100)";
        ctx.fillText(duty.toFixed(2), p.x, by + barMaxH + 16);
        ctx.textAlign = "left";
      });
    }

    function setValues(v) {
      if (v.T != null) { state.T = v.T; sT.input.value = String(v.T); sT.valEl.textContent = v.T.toFixed(2); }
      if (v.R != null) { state.R = v.R; sR.input.value = String(v.R); sR.valEl.textContent = v.R.toFixed(2); }
      if (v.P != null) { state.P = v.P; sP.input.value = String(v.P); sP.valEl.textContent = v.P.toFixed(2); }
      if (v.Y != null) { state.Y = v.Y; sY.input.value = String(v.Y); sY.valEl.textContent = v.Y.toFixed(2); }
      draw();
    }

    sT.input.addEventListener("input", function () { setValues({ T: parseFloat(sT.input.value) }); });
    sR.input.addEventListener("input", function () { setValues({ R: parseFloat(sR.input.value) }); });
    sP.input.addEventListener("input", function () { setValues({ P: parseFloat(sP.input.value) }); });
    sY.input.addEventListener("input", function () { setValues({ Y: parseFloat(sY.input.value) }); });

    redraw = draw;
    draw();

    var demoTimer = null;
    if (options.demo) {
      var demoStates = [
        { T: 0.5, R: 0, P: 0, Y: 0 },
        { T: 0.6, R: 0.4, P: -0.2, Y: 0.15 },
        { T: 0.5, R: -0.3, P: 0.3, Y: -0.25 },
      ];
      var di = 0;
      function applyDemo() { setValues(demoStates[di % demoStates.length]); di++; }
      applyDemo();
      demoTimer = setInterval(applyDemo, 1800);
    }

    return {
      dispose: function () {
        cv.disconnect();
        if (demoTimer) clearInterval(demoTimer);
        if (container.contains(root)) container.removeChild(root);
      },
      setValues: setValues,
    };
  }

  if (!global.SfWidgets) global.SfWidgets = {};
  global.SfWidgets.mixer = mount;
})(window);
