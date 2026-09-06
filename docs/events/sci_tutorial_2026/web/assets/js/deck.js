// deck.js — navigation, scaling, and chrome behaviour for the StampFly
// SCI/SICE tutorial HTML slide deck.
// StampFly SCI/SICEチュートリアルHTMLスライドデッキのナビゲーション・
// 拡大縮小・操作系。
(function () {
  "use strict";

  // Surface otherwise-silent errors (including inside the fonts.ready
  // promise chain) into the DOM, so a headless --dump-dom pass run with
  // ?check=1 can reveal a broken check pass instead of just omitting
  // <pre id="check-report">.
  // 通常は見えないエラー（fonts.readyのPromiseチェーン内も含む）をDOMに
  // 出す。headless --dump-domで?check=1を実行した際、<pre id="check-report">
  // が単に欠落するのではなく、壊れた原因がわかるようにするため。
  function reportJsError(label, detail) {
    var pre = document.getElementById("js-error-report");
    if (!pre) {
      pre = document.createElement("pre");
      pre.id = "js-error-report";
      document.body.appendChild(pre);
    }
    pre.textContent += label + ": " + detail + "\n";
  }
  window.addEventListener("error", function (ev) {
    reportJsError("error", ev.message + " @ " + ev.filename + ":" + ev.lineno);
  });
  window.addEventListener("unhandledrejection", function (ev) {
    reportJsError("unhandledrejection", ev.reason && ev.reason.stack ? ev.reason.stack : String(ev.reason));
  });

  var slides = Array.prototype.slice.call(document.querySelectorAll(".slide"));
  var total = slides.length;
  var current = 1; // 1-based, matches the ?s=N URL convention

  // ---- stage scaling: fit the fixed 1280x720 canvas into the viewport,
  // letterboxed on whichever axis is shorter.
  // ステージの拡大縮小: 1280x720の固定キャンバスを、短い方の軸に合わせて
  // レターボックス状に収める。
  function fitStage() {
    var stage = document.getElementById("stage");
    var scale = Math.min(window.innerWidth / 1280, window.innerHeight / 720);
    stage.style.transform = "scale(" + scale + ")";
  }

  function clamp(n) {
    return Math.max(1, Math.min(total, n));
  }

  function showSlide(n, opts) {
    opts = opts || {};
    n = clamp(n);
    slides.forEach(function (el, idx) {
      el.hidden = idx !== n - 1;
    });
    current = n;
    updateProgress();
    updateOutlineHighlight();
    updateUrl(opts.replace);
    renderMath();
    mountWidgetsForSlide(slides[n - 1]);
    disposeWidgetsExceptSlide(slides[n - 1]);
  }

  // ---- web-only widgets (`% @web: widget=...` markers, see
  // tools/slides/beamer_to_html.py apply_web_markers/render_web_widget) --
  // Each `.web-only.widget[data-widget]` div is an empty mount point in the
  // static HTML. Mount only the CURRENT slide's widgets (so a canvas/
  // rAF-driven widget is never running for a slide the viewer navigated
  // away from) and dispose() any widget whose slide is no longer current.
  // `% @web: widget=...`マーカー由来のウィジェット差し込み先
  // （`.web-only.widget[data-widget]`、静的HTML上は空div）。現在
  // 表示中のスライドの分だけマウントし（canvas/rAFループを他スライドで
  // 動かし続けない）、表示から外れたスライドのウィジェットはdispose()する。
  // ------------------------------------------------------------------------
  function numOrUndef(v) { return v === undefined ? undefined : Number(v); }
  var WIDGET_FACTORY = {
    pid_step: function (el, data) {
      return window.SfWidgets.pidStep(el, { axis: data.axis, preset: data.preset });
    },
    mixer: function (el, data) {
      return window.SfWidgets.mixer(el, {
        T: numOrUndef(data.t), R: numOrUndef(data.r), P: numOrUndef(data.p), Y: numOrUndef(data.y),
      });
    },
    comp_filter: function (el, data) {
      return window.SfWidgets.compFilter(el, { alpha: numOrUndef(data.alpha) });
    },
  };
  var mountedWidgets = new Map(); // <div class="web-only widget"> -> instance

  function mountWidgetsForSlide(slide) {
    if (!slide || typeof window.SfWidgets === "undefined") return;
    slide.querySelectorAll(".web-only.widget[data-widget]").forEach(function (el) {
      if (mountedWidgets.has(el)) return;
      var factory = WIDGET_FACTORY[el.dataset.widget];
      if (!factory) return;
      mountedWidgets.set(el, factory(el, el.dataset));
    });
  }

  function disposeWidgetsExceptSlide(currentSlide) {
    mountedWidgets.forEach(function (inst, el) {
      if (el.closest(".slide") !== currentSlide) {
        inst.dispose();
        mountedWidgets.delete(el);
      }
    });
  }

  function updateProgress() {
    var bar = document.getElementById("progress-bar");
    bar.style.width = (current / total * 100) + "%";
  }

  function updateUrl(replace) {
    var url = new URL(window.location.href);
    url.searchParams.set("s", String(current));
    if (replace) {
      window.history.replaceState({ s: current }, "", url);
    } else {
      window.history.pushState({ s: current }, "", url);
    }
  }

  function next() { showSlide(current + 1); }
  function prev() { showSlide(current - 1); }

  // ---- KaTeX auto-render: run once per slide switch is wasteful but the
  // deck is short-lived per view; simplest correct behaviour is to render
  // the whole stage once on first paint (all slides), which auto-render
  // will just skip on subsequent calls since it marks nodes as done.
  // 初回に全スライドへ一括適用する（auto-renderは処理済みノードを
  // 再処理しないため、以降の呼び出しは実質no-op）。
  var mathRendered = false;
  function renderMath() {
    if (mathRendered || typeof renderMathInElement === "undefined") return;
    renderMathInElement(document.getElementById("stage"), {
      delimiters: [
        { left: "\\(", right: "\\)", display: false },
        { left: "\\[", right: "\\]", display: true },
      ],
      throwOnError: false,
    });
    mathRendered = true;
  }

  // ---- outline panel ----------------------------------------------------
  function toggleOutline(force) {
    var panel = document.getElementById("outline");
    var show = force !== undefined ? force : panel.hidden;
    panel.hidden = !show;
    if (show) updateOutlineHighlight();
  }

  function updateOutlineHighlight() {
    var links = document.querySelectorAll("#outline a[data-slide]");
    links.forEach(function (a) {
      a.classList.toggle("current", Number(a.dataset.slide) === current);
    });
  }

  // ---- help overlay -------------------------------------------------------
  function toggleHelp(force) {
    var el = document.getElementById("help");
    var show = force !== undefined ? force : el.hidden;
    el.hidden = !show;
  }

  // ---- fullscreen -----------------------------------------------------------
  function toggleFullscreen() {
    if (document.fullscreenElement) {
      document.exitFullscreen();
    } else {
      document.documentElement.requestFullscreen().catch(function () {});
    }
  }

  // ---- keyboard ---------------------------------------------------------------
  document.addEventListener("keydown", function (ev) {
    if (ev.key === "ArrowRight" || ev.key === " " || ev.key === "PageDown" || ev.key === "n") {
      ev.preventDefault(); next();
    } else if (ev.key === "ArrowLeft" || ev.key === "PageUp" || ev.key === "p") {
      ev.preventDefault(); prev();
    } else if (ev.key === "Home") {
      showSlide(1);
    } else if (ev.key === "End") {
      showSlide(total);
    } else if (ev.key === "f") {
      toggleFullscreen();
    } else if (ev.key === "t") {
      toggleOutline();
    } else if (ev.key === "?") {
      toggleHelp();
    } else if (ev.key === "Escape") {
      toggleOutline(false);
      toggleHelp(false);
    }
  });

  // ---- click zones: left third = prev, right third = next -------------------
  document.getElementById("viewport").addEventListener("click", function (ev) {
    if (ev.target.closest("#controls")) return;
    var x = ev.clientX / window.innerWidth;
    if (x > 0.66) next();
    else if (x < 0.34) prev();
  });

  // ---- touch swipe --------------------------------------------------------
  var touchStartX = null;
  document.addEventListener("touchstart", function (ev) {
    touchStartX = ev.changedTouches[0].clientX;
  }, { passive: true });
  document.addEventListener("touchend", function (ev) {
    if (touchStartX === null) return;
    var dx = ev.changedTouches[0].clientX - touchStartX;
    if (Math.abs(dx) > 50) {
      if (dx < 0) next(); else prev();
    }
    touchStartX = null;
  }, { passive: true });

  // ---- outline links + buttons ----------------------------------------------
  document.querySelectorAll("#outline a[data-slide]").forEach(function (a) {
    a.addEventListener("click", function (ev) {
      ev.preventDefault();
      showSlide(Number(a.dataset.slide));
      toggleOutline(false);
    });
  });
  document.getElementById("btn-outline").addEventListener("click", function () { toggleOutline(); });
  document.getElementById("btn-fullscreen").addEventListener("click", toggleFullscreen);
  document.getElementById("btn-help").addEventListener("click", function () { toggleHelp(); });
  document.getElementById("help-close").addEventListener("click", function () { toggleHelp(false); });
  document.getElementById("help").addEventListener("click", function (ev) {
    if (ev.target.id === "help") toggleHelp(false);
  });

  window.addEventListener("popstate", function (ev) {
    var s = ev.state && ev.state.s ? ev.state.s : readSlideFromUrl();
    showSlide(s, { replace: true });
  });
  window.addEventListener("resize", fitStage);

  function readSlideFromUrl() {
    var params = new URLSearchParams(window.location.search);
    var n = parseInt(params.get("s"), 10);
    return isNaN(n) ? 1 : n;
  }

  // ---- fit-inner wrapping: give every .slide-body a single flex child that
  // grows to fill spare room (so .figure/.columns keep flex-growing exactly
  // as if they were direct children) but never shrinks below its own
  // natural size, so overflow can be measured and, only if real, scaled
  // down as a last resort. See slide.css .fit-inner for the mechanics.
  // フィットラッパーの生成: 各.slide-bodyの中身を1つのflex子要素にまとめる。
  // 余白があれば伸びて.figure/.columnsのflex-growを維持しつつ、自然な高さ
  // より縮むことはない。詳しい仕組みはslide.cssの.fit-innerコメント参照。
  // ------------------------------------------------------------------------
  var FIT_MIN_SCALE = 0.7;
  var fitReport = []; // [{slide: 1-based index, scale}], only entries < 1

  function wrapFitInner() {
    slides.forEach(function (slide) {
      var body = slide.querySelector(".slide-body");
      if (!body || body.querySelector(":scope > .fit-inner")) return;
      var inner = document.createElement("div");
      inner.className = "fit-inner";
      while (body.firstChild) inner.appendChild(body.firstChild);
      body.appendChild(inner);
    });
  }

  // Measure one slide's .fit-inner against its .slide-body box and, if the
  // content is genuinely taller than the slide (not just temporarily
  // shrunk by flex), scale .fit-inner down (min 0.7x) so it fits without
  // being clipped. Returns the scale actually applied (1 = no shrink
  // needed). Caller is responsible for un-hiding the slide first — a
  // [hidden] (display:none) element lays out at 0x0, so every measurement
  // here would read zero.
  // 1枚のスライドについて .fit-inner を .slide-body の箱と比較し、内容が
  // 本当にスライドより高い場合だけ縮小（最小0.7倍）して収める。呼び出し側
  // が事前にスライドの hidden を解除しておくこと（[hidden]はdisplay:none
  // になり寸法が0になるため）。
  // Returns {scale, clamped, residualPx, naturalH, availableH}. `scale` is
  // what got applied (1 = no shrink needed). `clamped` means even the
  // floor (0.7x) scale wasn't enough — residualPx is how much genuinely
  // still overflows in that case (a CSS transform changes paint, not
  // layout, so .slide-body's own scrollHeight can't be used to check this
  // afterwards — see runOverflowCheck).
  // 戻り値は {scale, clamped, residualPx, naturalH, availableH}。scaleは
  // 実際に適用した倍率（1なら縮小不要）。clampedは下限(0.7倍)まで縮めても
  // 収まらなかったことを示し、その場合の残存はみ出し量がresidualPx
  // （transformはレイアウトを変えず見た目だけなので、後から
  // .slide-body の scrollHeight では判定できない — runOverflowCheck参照）。
  function computeFit(slide) {
    var result = { scale: 1, clamped: false, residualPx: 0, naturalH: 0, availableH: 0 };
    var body = slide.querySelector(".slide-body");
    if (!body) return result; // title/divider slides have no .slide-body
    var inner = body.querySelector(":scope > .fit-inner");
    if (!inner) return result;
    inner.style.transform = "none";
    // offsetHeight, not getBoundingClientRect().height: GBCR reports the
    // PAINTED size, which includes the #stage viewport-fit scale (deck.js
    // fitStage() — 1x only when the window happens to be exactly
    // 1280x720). offsetHeight/clientHeight are layout metrics that CSS
    // transforms never touch, so they stay in the same fixed design-
    // canvas units as availableH (body.clientHeight) regardless of the
    // actual browser window size. Comparing a GBCR height (viewport-
    // scaled) against a clientHeight (design-canvas units) silently
    // shrunk slides that didn't need it whenever the capture window
    // wasn't exactly 1280x720.
    // getBoundingClientRect().height ではなく offsetHeight を使う:
    // GBCRは実際に描画されたサイズを返すため、deck.jsのfitStage()による
    // #stageの表示倍率（ウィンドウがちょうど1280x720の時だけ1倍）を
    // 含んでしまう。offsetHeight/clientHeightはCSSのtransformの影響を
    // 受けないレイアウト値なので、ブラウザウィンドウの実サイズに関係なく
    // availableH（body.clientHeight）と同じ固定デザインキャンバス単位に
    // 揃う。GBCR（表示倍率込み）とclientHeight（デザイン単位）を比較すると、
    // キャプチャ時のウィンドウがちょうど1280x720でない限り、縮小不要な
    // スライドまで無言で縮小してしまっていた。
    var naturalH = inner.offsetHeight;
    // body.clientHeight is the PADDING-BOX height (content + padding), but
    // .fit-inner only occupies the CONTENT box inside that padding — the
    // space it can actually grow into is clientHeight minus .slide-body's
    // own top+bottom padding (see slide.css .slide-body). Using
    // clientHeight directly overstated the available room by exactly that
    // padding, under-shrinking every scaled slide by the same amount and
    // letting their last line sit that far into the footer bar.
    // body.clientHeightは「内容+パディング」を含む値だが、.fit-innerが
    // 実際に伸びられるのはそのパディングの内側（コンテンツ領域）だけ
    // （slide.cssの.slide-body参照）。clientHeightをそのまま使うと
    // .slide-body自身の上下パディング分だけ余白を過大評価してしまい、
    // 縮小がその分だけ足りず、最終行がフッターバーにその分だけ食い込んで
    // いた。
    var bodyStyle = getComputedStyle(body);
    var availableH = body.clientHeight -
      parseFloat(bodyStyle.paddingTop) -
      parseFloat(bodyStyle.paddingBottom);
    result.naturalH = Math.round(naturalH);
    result.availableH = availableH;
    if (naturalH <= availableH + 1 || naturalH === 0) return result;
    var rawScale = availableH / naturalH;
    var scale = Math.max(FIT_MIN_SCALE, rawScale);
    inner.style.transform = "scale(" + scale.toFixed(4) + ")";
    result.scale = scale;
    if (rawScale < FIT_MIN_SCALE) {
      result.clamped = true;
      result.residualPx = Math.round(naturalH * scale - availableH);
    }
    return result;
  }

  function computeAllFits() {
    fitReport = [];
    slides.forEach(function (slide, idx) {
      var wasHidden = slide.hidden;
      slide.hidden = false;
      var r = computeFit(slide);
      if (r.scale < 0.999) {
        fitReport.push({
          slide: idx + 1,
          scale: Math.round(r.scale * 1000) / 1000,
          clamped: r.clamped,
          residualPx: r.residualPx,
        });
      }
      slide.hidden = wasHidden;
    });
  }

  // ---- ?check=1 overflow audit --------------------------------------------
  // Measures every slide against the fixed 1280x720 design box and writes a
  // JSON report into <pre id="check-report">, for a headless-Chrome
  // --dump-dom pass to parse. Not part of normal deck use.
  //
  // Vertical overflow is read from the fit pass (computeAllFits), not from
  // .slide-body's own scrollHeight: a CSS `transform: scale()` changes
  // paint only, never layout, so once the fit pass shrinks an overflowing
  // .fit-inner, .slide-body.scrollHeight *still* reports the pre-scale
  // (unscaled) height forever — reading it here would flag every fit-
  // scaled slide as "still overflowing" even though it visually isn't.
  // Genuine, unresolved vertical overflow only remains when the fit scale
  // was clamped at its 0.7x floor (fitReport's `clamped`/`residualPx`).
  // Horizontal overflow is checked directly on the two containers that
  // scroll instead of shrinking (.table-scroll, .code-block pre): a
  // transform scale shrinks them proportionally along with everything
  // else, but an unbreakable long token (URL, code) can still force one
  // wider than its box.
  // ?check=1 のはみ出し監査: 全スライドを1280x720の固定デザイン領域と
  // 比較し、JSONレポートを<pre id="check-report">に書き出す
  // （headless Chromeの--dump-domで解析する開発用機能）。
  //
  // 縦方向のはみ出しは.slide-body自身のscrollHeightではなくフィット結果
  // (computeAllFits)から判定する: CSSのtransform:scale()は見た目だけを
  // 変えレイアウトは変えないため、フィットで縮小した後も
  // .slide-body.scrollHeightは縮小前の高さを返し続け、見た目には収まって
  // いるスライドを毎回「まだはみ出している」と誤検出してしまう。本当に
  // 未解決のはみ出しが残るのは、フィット倍率が下限0.7倍で頭打ちに
  // なった場合だけ（fitReportのclamped/residualPx）。横方向は、縮小では
  // なくスクロールで逃げている2種類のコンテナ（.table-scroll、
  // .code-block pre）を直接調べる: transformによる縮小は他と一緒に
  // 比例して効くが、改行できない長いトークン（URLやコード）は依然として
  // 箱より幅を超えることがあるため。
  function runOverflowCheck() {
    var report = [];
    slides.forEach(function (slide, idx) {
      var wasHidden = slide.hidden;
      slide.hidden = false;

      var slideRect = slide.getBoundingClientRect();
      var issues = [];
      var fit = fitReport.filter(function (f) { return f.slide === idx + 1; })[0];
      if (fit && fit.clamped) {
        issues.push({ type: "body-overflow-y-after-fit-floor", px: fit.residualPx });
      }

      slide.querySelectorAll(".table-scroll, .code-block pre").forEach(function (m) {
        var dx = m.scrollWidth - m.clientWidth;
        if (dx > 2) {
          issues.push({ type: "horizontal-scroll-overflow", tag: m.className || m.tagName.toLowerCase(), px: Math.round(dx) });
        }
      });

      slide.querySelectorAll("img, svg, table, pre").forEach(function (m) {
        var r = m.getBoundingClientRect();
        if (r.width === 0 && r.height === 0) return; // not laid out (e.g. empty)
        var over = {
          top: slideRect.top - r.top,
          left: slideRect.left - r.left,
          right: r.right - slideRect.right,
          bottom: r.bottom - slideRect.bottom,
        };
        var worst = Math.max(over.top, over.left, over.right, over.bottom);
        if (worst > 1) {
          var edge = over.bottom === worst ? "bottom" : over.right === worst ? "right" :
            over.top === worst ? "top" : "left";
          var cls = m.className && m.className.baseVal !== undefined ? m.className.baseVal : m.className;
          issues.push({ type: "media-overflow", tag: m.tagName.toLowerCase(), cls: cls,
            overflowPx: Math.round(worst), edge: edge });
        }
      });

      report.push({
        slide: idx + 1,
        title: slide.dataset.title || "",
        issues: issues,
        fitScale: fit ? fit.scale : 1,
      });

      slide.hidden = wasHidden;
    });

    var summary = {
      total: report.length,
      overflowing: report.filter(function (r) { return r.issues.length > 0; }).length,
      fitApplied: report.filter(function (r) { return r.fitScale < 0.999; }).length,
      fitClampedUnresolved: report.filter(function (r) {
        return r.issues.some(function (i) { return i.type === "body-overflow-y-after-fit-floor"; });
      }).length,
    };

    var pre = document.getElementById("check-report");
    if (!pre) {
      pre = document.createElement("pre");
      pre.id = "check-report";
      document.body.appendChild(pre);
    }
    pre.textContent = JSON.stringify({ summary: summary, slides: report });
  }

  // ---- boot -------------------------------------------------------------------
  wrapFitInner();
  fitStage();
  showSlide(readSlideFromUrl(), { replace: true });
  var params = new URLSearchParams(window.location.search);
  if (params.get("outline") === "1") toggleOutline(true);

  // Fit computation (and the overflow audit, if requested) waits for web
  // fonts to finish loading: the Noto Sans JP swap can reflow text after
  // the very first paint, which would otherwise make every measurement
  // above stale.
  // フィット計算（および依頼時の監査）はWebフォント読み込み完了を待つ。
  // Noto Sans JPの差し替えで初回描画後にテキストが再フローするため、
  // それより前に測ると値が古くなってしまう。
  // Race against a fixed timeout rather than relying purely on
  // document.fonts.ready settling — bounds the wait even in an
  // environment with no network access (the Google Fonts request can
  // never resolve there) or any other reason the promise might stall,
  // and avoids requestAnimationFrame (which virtual-time-budget headless
  // runs do not reliably pump).
  // document.fonts.ready の解決だけに頼らず、固定タイムアウトとの
  // レースにする — ネットワーク不通（Google Fontsが解決できない環境）や
  // 他の理由でPromiseが止まっても待ち時間の上限を保証する。また
  // requestAnimationFrame は virtual-time-budget 付きheadless実行では
  // 確実に発火するとは限らないため使わない。
  var fontsReady = (document.fonts && document.fonts.ready) ? document.fonts.ready : Promise.resolve();
  var afterFonts = new Promise(function (resolve) {
    var done = false;
    var finish = function () { if (!done) { done = true; resolve(); } };
    fontsReady.then(finish, finish);
    setTimeout(finish, 1500);
  });
  afterFonts.then(function () {
    setTimeout(function () {
      computeAllFits();
      if (params.get("check") === "1") runOverflowCheck();
    }, 50);
  });
})();
