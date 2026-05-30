#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# M7 log review GUI — a human visually accepts (OK) or rejects (NG) each flight
# log as a verification "ruler", judging from three TRUSTWORTHY signals only:
#   1. bottom ToF raw distance — ground-truth height, does NOT pass the ESKF
#   2. the logged attitude estimate (quat -> roll/pitch) overlaid on the
#      raw-accel tilt — so the human sees whether the estimate matches raw physics
#   3. raw IMU (gyro_raw / accel_raw) — what the sensors actually measured
# Verdicts (OK / NG / hold) persist to docs/logmap/verdicts.json.
#
# Built on matplotlib's own Button widgets (no extra deps — no streamlit needed).
# Just run it; it opens a window with the plots and OK/NG/hold buttons.
#
# M7 ログ選別GUI — 人間が各フライトログを検証の「物差し」として OK/NG 判定する。
# 信頼できる3信号だけで判断: ①底面ToF生(対地距離=真値、ESKF非依存) ②姿勢推定
# (quat→roll/pitch)を生accel由来の傾きに重畳(推定が生の物理と整合か目視) ③IMU生値
# (gyro_raw/accel_raw)。判定(OK/NG/保留)は docs/logmap/verdicts.json に保存。
# matplotlib 標準のボタンウィジェットで実装(追加依存なし=streamlit不要)。
#
# Run / 起動:
#   cd simulator/sil
#   python3 m7_log_review_gui.py
#
# @design project_sil_architecture (memory) — log quality gate before M7 ruler use

import json
import os
import glob
import math
import sys
from datetime import datetime, timezone

import numpy as np
import matplotlib

# Pick an interactive backend (MacOSX on mac, else TkAgg/QtAgg). Headless -> Agg.
# 対話バックエンドを選ぶ(macはMacOSX、他はTkAgg/QtAgg)。ヘッドレスはAgg。
_BACKEND = None
for _b in ("MacOSX", "TkAgg", "QtAgg"):
    try:
        matplotlib.use(_b, force=True)
        _BACKEND = _b
        break
    except Exception:
        continue
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "logs"))
VERDICT_PATH = os.path.join(HERE, "docs", "logmap", "verdicts.json")

TOF_MAX_VALID = 4.0   # ToF sensor ceiling [m] / ToF測距上限
AIRBORNE_M = 0.15     # raw ToF above this = off the ground / これ超で浮上


# -----------------------------------------------------------------------------
# Data loading — reads ONLY the three trusted streams (+ quat for the estimate).
# データ読み込み — 信頼3ストリーム(+推定用quat)のみ読む。
# -----------------------------------------------------------------------------
def load_log(path):
    t_imu = []
    gx, gy, gz = [], [], []          # raw gyro [rad/s]
    ax_, ay_, az_ = [], [], []       # raw accel [m/s^2]
    er, ep = [], []                  # estimate euler roll/pitch [deg]
    rr, rp = [], []                  # raw-accel tilt roll/pitch [deg]
    t_tof, tof_d = [], []            # bottom ToF raw distance [m]
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            mid = o.get("id")
            ts = o.get("ts")
            if ts is None:
                continue
            if mid == "imu":
                g = o.get("gyro_raw") or o.get("gyro") or [0, 0, 0]
                a = o.get("accel_raw") or o.get("accel") or [0, 0, 0]
                t_imu.append(ts)
                gx.append(g[0]); gy.append(g[1]); gz.append(g[2])
                ax_.append(a[0]); ay_.append(a[1]); az_.append(a[2])
                q = o.get("quat", [1, 0, 0, 0])
                w, x, y, z = q
                roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
                er.append(math.degrees(roll)); ep.append(math.degrees(pitch))
                rr.append(math.degrees(math.atan2(-a[1], -a[2])))
                rp.append(math.degrees(math.atan2(a[0],
                          math.hypot(a[1], a[2]) + 1e-9)))
            elif mid == "tof_b":
                t_tof.append(ts)
                tof_d.append(o.get("distance", 0.0))
    t0 = t_imu[0] if t_imu else 0
    return dict(
        ti=(np.array(t_imu, float) - t0) / 1e6,
        gx=np.array(gx), gy=np.array(gy), gz=np.array(gz),
        ax=np.array(ax_), ay=np.array(ay_), az=np.array(az_),
        er=np.array(er), ep=np.array(ep), rr=np.array(rr), rp=np.array(rp),
        tt=(np.array(t_tof, float) - t0) / 1e6 if t_tof else np.array([]),
        tof=np.array(tof_d),
    )


def quick_facts(d):
    gyro_n = np.sqrt(d["gx"] ** 2 + d["gy"] ** 2 + d["gz"] ** 2) if len(d["gx"]) else np.array([])
    tof_valid = d["tof"][(d["tof"] > 0) & (d["tof"] < TOF_MAX_VALID)] \
        if len(d["tof"]) else np.array([])
    return dict(
        dur=float(d["ti"][-1]) if len(d["ti"]) else 0.0,
        has_tof=len(d["tt"]) > 0,
        tof_max=float(tof_valid.max()) if len(tof_valid) else 0.0,
        airborne=bool((tof_valid > AIRBORNE_M).any()) if len(tof_valid) else False,
        gyro_max=float(gyro_n.max()) if len(gyro_n) else 0.0,
        att_mismatch=float(np.sqrt(np.mean(
            (d["er"] - d["rr"]) ** 2 + (d["ep"] - d["rp"]) ** 2)))
        if len(d["er"]) else 0.0,
    )


# -----------------------------------------------------------------------------
# Verdict persistence
# 判定の永続化
# -----------------------------------------------------------------------------
def load_verdicts():
    if os.path.exists(VERDICT_PATH):
        with open(VERDICT_PATH) as f:
            return json.load(f)
    return {}


def save_verdict(name, verdict):
    os.makedirs(os.path.dirname(VERDICT_PATH), exist_ok=True)
    v = load_verdicts()
    v[name] = {"verdict": verdict,
               "ts": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    with open(VERDICT_PATH, "w") as f:
        json.dump(v, f, ensure_ascii=False, indent=2, sort_keys=True)
    return v


# -----------------------------------------------------------------------------
# Interactive reviewer (matplotlib Buttons)
# 対話レビューア(matplotlibボタン)
# -----------------------------------------------------------------------------
class Reviewer:
    def __init__(self, logs, names):
        self.logs = logs
        self.names = names
        self.idx = 0
        self.verdicts = load_verdicts()
        # start at the first unjudged log / 最初の未判定から開始
        first = next((i for i, n in enumerate(names) if n not in self.verdicts), 0)
        self.idx = first

        self.fig = plt.figure(figsize=(12, 9))
        # 4 stacked plot axes + a bottom strip for buttons
        # 4段のプロット軸 + 下部にボタン帯
        self.ax = [self.fig.add_axes([0.09, 0.74 - i * 0.165, 0.88, 0.15])
                   for i in range(4)]
        self._make_buttons()
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.draw()

    def _make_buttons(self):
        # row of action buttons along the very bottom
        # 最下部のアクションボタン列
        specs = [
            ("◀ 前 (←)", 0.09, self.prev, "0.85"),
            ("次 ▶ (→)", 0.20, self.next, "0.85"),
            ("✅ OK (o)", 0.34, lambda e: self.judge("OK"), "#bdf5bd"),
            ("❌ NG (n)", 0.47, lambda e: self.judge("NG"), "#f5bdbd"),
            ("❓ 保留 (h)", 0.60, lambda e: self.judge("unsure"), "#f5e6bd"),
            ("⏭ 次の未判定 (u)", 0.73, self.next_unjudged, "0.85"),
            ("💾 終了 (q)", 0.88, self.quit, "0.8"),
        ]
        self.buttons = []
        for label, x, cb, color in specs:
            w = 0.10 if "未判定" in label else 0.085
            bax = self.fig.add_axes([x, 0.02, w, 0.045])
            b = Button(bax, label, color=color, hovercolor="#ddd")
            b.label.set_fontsize(9)
            b.on_clicked(cb)
            self.buttons.append(b)

    # ---- navigation ----
    def prev(self, _e=None):
        self.idx = max(0, self.idx - 1); self.draw()

    def next(self, _e=None):
        self.idx = min(len(self.logs) - 1, self.idx + 1); self.draw()

    def next_unjudged(self, _e=None):
        nxt = next((j for j in range(self.idx + 1, len(self.names))
                    if self.names[j] not in self.verdicts), None)
        if nxt is None:
            nxt = next((j for j in range(len(self.names))
                        if self.names[j] not in self.verdicts), None)
        if nxt is not None:
            self.idx = nxt; self.draw()
        else:
            print("[review] 未判定はもうありません（全件判定済み）")

    def judge(self, verdict):
        name = self.names[self.idx]
        self.verdicts = save_verdict(name, verdict)
        print(f"[review] {name} -> {verdict}")
        # auto-advance to next unjudged
        self.next_unjudged()
        self.draw()

    def quit(self, _e=None):
        n_ok = sum(1 for x in self.verdicts.values() if x["verdict"] == "OK")
        print(f"\n[review] 保存先: {VERDICT_PATH}")
        print(f"[review] OK={n_ok} 件。OK一覧:")
        for k, x in sorted(self.verdicts.items()):
            if x["verdict"] == "OK":
                print(f"  {k}")
        plt.close(self.fig)

    def on_key(self, event):
        k = (event.key or "").lower()
        if k in ("left", "backspace"):
            self.prev()
        elif k in ("right", " "):
            self.next()
        elif k == "o":
            self.judge("OK")
        elif k == "n":
            self.judge("NG")
        elif k == "h":
            self.judge("unsure")
        elif k == "u":
            self.next_unjudged()
        elif k == "q":
            self.quit()

    # ---- drawing ----
    def draw(self):
        for a in self.ax:
            a.clear()
        name = self.names[self.idx]
        d = load_log(self.logs[self.idx])
        f = quick_facts(d)
        ax = self.ax

        # 1) bottom ToF RAW distance — circular-free ground-truth height
        if len(d["tt"]):
            tof = np.where((d["tof"] > 0) & (d["tof"] < TOF_MAX_VALID), d["tof"], np.nan)
            ax[0].plot(d["tt"], tof, "k.", ms=2, label="bottom ToF (RAW=truth)")
            ax[0].axhline(AIRBORNE_M, color="C3", ls="--", lw=1,
                          label=f"airborne {AIRBORNE_M:.2f}m")
        else:
            ax[0].text(0.5, 0.5, "NO bottom ToF — no independent height evidence",
                       ha="center", va="center", transform=ax[0].transAxes,
                       color="C3", fontsize=11)
        ax[0].set_ylabel("ToF [m]")
        ax[0].legend(fontsize=7, loc="upper right")

        # 2) attitude estimate (quat) vs RAW-accel tilt
        if len(d["er"]):
            ax[1].plot(d["ti"], d["er"], "C0-", lw=0.8, label="roll est")
            ax[1].plot(d["ti"], d["ep"], "C1-", lw=0.8, label="pitch est")
            ax[1].plot(d["ti"], d["rr"], "C0:", lw=0.7, alpha=0.6, label="roll raw")
            ax[1].plot(d["ti"], d["rp"], "C1:", lw=0.7, alpha=0.6, label="pitch raw")
        ax[1].set_ylabel("att [deg]")
        ax[1].legend(fontsize=7, loc="upper right", ncol=2)

        # 3) raw gyro
        ax[2].plot(d["ti"], d["gx"], "C0-", lw=0.5, label="gx")
        ax[2].plot(d["ti"], d["gy"], "C1-", lw=0.5, label="gy")
        ax[2].plot(d["ti"], d["gz"], "C2-", lw=0.5, label="gz")
        ax[2].set_ylabel("gyro raw\n[rad/s]")
        ax[2].legend(fontsize=7, loc="upper right", ncol=3)

        # 4) raw accel
        ax[3].plot(d["ti"], d["ax"], "C0-", lw=0.5, label="ax")
        ax[3].plot(d["ti"], d["ay"], "C1-", lw=0.5, label="ay")
        ax[3].plot(d["ti"], d["az"], "C2-", lw=0.5, label="az")
        ax[3].set_ylabel("accel raw\n[m/s²]")
        ax[3].set_xlabel("time [s]")
        ax[3].legend(fontsize=7, loc="upper right", ncol=3)

        for a in ax:
            a.grid(True, alpha=0.3)

        cur = self.verdicts.get(name, {}).get("verdict", "未判定")
        n_done = len(self.verdicts)
        n_ok = sum(1 for x in self.verdicts.values() if x["verdict"] == "OK")
        n_ng = sum(1 for x in self.verdicts.values() if x["verdict"] == "NG")
        warn = ""
        if not f["has_tof"]:
            warn = "  ⚠ ToFなし=高度の独立証拠なし(循環)→NG候補"
        elif f["att_mismatch"] > 10:
            warn = f"  ⚠ 姿勢の推定-生 乖離 {f['att_mismatch']:.0f}°(推定が生IMUと不一致)"
        ax[0].set_title(
            f"[{self.idx+1}/{len(self.logs)}] {name}   現判定: {cur}   "
            f"(進捗 {n_done}/{len(self.logs)}: OK{n_ok}/NG{n_ng})\n"
            f"dur={f['dur']:.0f}s  ToF={'あり' if f['has_tof'] else 'なし'}  "
            f"浮上={'はい' if f['airborne'] else 'いいえ'}({f['tof_max']:.2f}m)  "
            f"gyro_max={f['gyro_max']:.1f}  姿勢乖離={f['att_mismatch']:.1f}°{warn}",
            fontsize=9, loc="left")
        self.fig.canvas.draw_idle()


def main():
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "stampfly_udp_*.jsonl")))
    names = [os.path.basename(p).replace("stampfly_udp_", "").replace(".jsonl", "")
             for p in logs]
    if not logs:
        print(f"ログが見つかりません: {LOG_DIR}", file=sys.stderr)
        return
    if _BACKEND in (None, "Agg"):
        print("対話バックエンドがありません（ヘッドレス環境）。"
              "デスクトップで実行してください。", file=sys.stderr)
        return
    print(f"[review] backend={_BACKEND}  logs={len(logs)}  "
          f"保存先={VERDICT_PATH}")
    print("[review] 操作: ←/→ 移動, o=OK, n=NG, h=保留, u=次の未判定, q=終了")
    Reviewer(logs, names)
    plt.show()


if __name__ == "__main__":
    main()
