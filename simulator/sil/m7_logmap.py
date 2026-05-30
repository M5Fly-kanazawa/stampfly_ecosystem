#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# M7 log map — survey EVERY flight log and emit (1) a one-row-per-log fact table
# and (2) a per-log overview figure, so a human (and a review sub-agent) can SEE
# what each log actually was — ground-only, a clean flight to landing, a crash
# (flip), or an estimator-only blow-up — BEFORE any RMSE verification.
#
# This is deliberately a MAP, not a verdict: it reports observed facts (flight-
# state timeline, throttle-on span, max tilt from gravity, voltage, whether the
# logged estimate blew up) and draws them. Classification is done by eye on the
# figures, not by thresholds here.
#
# M7 ログ地図 — 全フライトログを俯瞰し、(1)1ログ=1行の事実テーブルと(2)ログ毎の
# 俯瞰図を出す。RMSE検証の前に、各ログが実際に何だったか(地上のみ/着陸まで完遂した
# 正常飛行/墜落=反転/推定だけ暴走)を人間(とレビューsub-agent)が「見て」判断できる
# ようにする。これは判定でなく地図: 観測事実(飛行状態列・スロットルON区間・重力
# からの最大傾き・電圧・ログ推定が暴走したか)を並べて描くだけ。分類は閾値でなく
# 図を目で見て行う。

import json
import glob
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SIL = os.path.dirname(os.path.abspath(__file__))
LOGS = sorted(glob.glob(os.path.join(SIL, "../../logs/stampfly_udp_*.jsonl")))
OUTDIR = os.path.join(SIL, "docs", "logmap")

# flight_state enum (firmware/.../stampfly_state.hpp), index = recorded int
# 飛行状態 enum（記録される整数 = この並びのインデックス）
FLIGHT_STATE = ["INIT", "CALIB", "IDLE", "ARMED", "FLYING", "LANDING", "ERROR"]
GRAVITY = 9.81


def read_log(path):
    """Stream a log into per-stream time series we need for the map.
    地図に必要なストリームだけ時系列に読み込む。"""
    t_imu, acc_z, tilt, qnorm = [], [], [], []
    acc_n, gyro_n = [], []
    t_pv, pos_h, pos_z, spd = [], [], [], []
    t_st, st, volt = [], [], []
    t_cr, thrust = [], []
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
                a = o.get("accel", [0, 0, 0])
                g = o.get("gyro", [0, 0, 0])
                n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
                t_imu.append(ts)
                acc_z.append(a[2])
                acc_n.append(n)
                gyro_n.append(math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]))
                # tilt from gravity: accel_z is ~ -g level, flips sign when inverted
                # 重力からの傾き: 水平で accel_z≈-g、反転で符号反転
                cz = max(-1.0, min(1.0, -a[2] / n)) if n > 1e-3 else 1.0
                tilt.append(math.degrees(math.acos(cz)))
                q = o.get("quat", [1, 0, 0, 0])
                qnorm.append(math.sqrt(sum(x * x for x in q)))
            elif mid == "posvel":
                p = o.get("pos", [0, 0, 0])
                v = o.get("vel", [0, 0, 0])
                t_pv.append(ts)
                pos_h.append(math.hypot(p[0], p[1]))
                pos_z.append(p[2])
                spd.append(math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
            elif mid == "status":
                t_st.append(ts)
                st.append(o.get("flight_state"))
                volt.append(o.get("voltage"))
            elif mid == "ctrl_ref":
                t_cr.append(ts)
                thrust.append(o.get("total_thrust", 0.0))
    return dict(
        t_imu=np.array(t_imu, float), acc_z=np.array(acc_z, float),
        acc_n=np.array(acc_n, float), gyro_n=np.array(gyro_n, float),
        tilt=np.array(tilt, float), qnorm=np.array(qnorm, float),
        t_pv=np.array(t_pv, float), pos_h=np.array(pos_h, float),
        pos_z=np.array(pos_z, float), spd=np.array(spd, float),
        t_st=np.array(t_st, float), st=st, volt=np.array(volt, float),
        t_cr=np.array(t_cr, float), thrust=np.array(thrust, float),
    )


def state_seq(st):
    """Compress the 1 Hz flight-state list into 'IDLE->ARMED->FLYING...'.
    1Hz の飛行状態列を 'IDLE->ARMED->FLYING...' に圧縮。"""
    seq = []
    for s in st:
        name = FLIGHT_STATE[s] if isinstance(s, int) and 0 <= s < len(FLIGHT_STATE) else str(s)
        if not seq or seq[-1] != name:
            seq.append(name)
    return "->".join(seq) if seq else "(no status)"


# Motion / inversion detection constants. The logged control outputs
# (total_thrust, motor_duty, throttle) are ALL zero in this corpus, so flight is
# detected from the IMU motion signature, not from thrust.
# 運動・反転検出の定数。本コーパスでは制御出力(total_thrust/motor_duty/throttle)が
# 全て0なので、飛行は推力でなくIMUの運動シグネチャから検出する。
GYRO_MOVE = 2.0      # |gyro| above this [rad/s] = actively moving (normal flight ~1-3)
ACC_STD_MOVE = 0.3   # accel-norm std above this [m/s^2] = vibration => moving
INVERT_DEG = 90.0    # tilt beyond this = inverted
INVERT_HOLD_S = 0.5  # inversion must persist this long to count (reject spikes)
IMU_DT = 0.0025      # 400 Hz


def detect_motion(d):
    """Classify stationary vs moved, and find sustained inversion + its timing.
    Returns a dict of observed facts (no verdict).
    静止/運動を分類し、持続反転とその時刻を見つける。観測事実のみ返す。"""
    g = d["gyro_n"]
    a = d["acc_n"]
    n = len(g)
    if n < 10:
        return dict(moved=False, gyro_max=0.0, acc_std=0.0,
                    inverted=False, invert_frac=None, move_frac=0.0)
    gyro_max = float(g.max())
    acc_std = float(np.std(a))
    moving = (g > GYRO_MOVE)
    move_frac = float(moving.mean())
    moved = gyro_max > GYRO_MOVE * 2 or acc_std > ACC_STD_MOVE

    # sustained inversion: tilt > INVERT_DEG for >= INVERT_HOLD_S continuously
    # 持続反転: 傾き>INVERT_DEG が INVERT_HOLD_S 以上連続
    inv = d["tilt"] > INVERT_DEG
    need = int(INVERT_HOLD_S / IMU_DT)
    inverted = False
    invert_frac = None
    run = 0
    for k in range(len(inv)):
        run = run + 1 if inv[k] else 0
        if run >= need:
            inverted = True
            invert_frac = k / len(inv)  # 0=start .. 1=end of record
            break
    return dict(moved=moved, gyro_max=gyro_max, acc_std=acc_std,
                inverted=inverted, invert_frac=invert_frac, move_frac=move_frac)


def heuristic_category(s, mo):
    """Rough auto-label to ORDER the table — MUST be confirmed by eye on figures.
    表を並べるための粗いラベル — 必ず図を目で見て確認すること。"""
    if not mo["moved"]:
        return "static?"            # never really moved => stationary on ground
    if mo["inverted"]:
        # inverted late in the record but estimate stayed sane => post-land flip
        # 記録後半で反転かつ推定は健全 => 着地後の手持ち反転の疑い
        if mo["invert_frac"] is not None and mo["invert_frac"] > 0.8 \
                and s["ref_pos_h_max"] < 5 and s["ref_pos_z_absmax"] < 5:
            return "flew+landflip?"
        return "crash?"             # sustained inversion during the record
    if s["ref_pos_h_max"] > 30 or s["ref_pos_z_absmax"] > 30:
        return "est-blowup?"        # moved, not inverted, but logged est went huge
    return "flew?"                  # moved, upright, estimate bounded


def summarize(path):
    name = os.path.basename(path).replace("stampfly_udp_", "").replace(".jsonl", "")
    d = read_log(path)
    if len(d["t_imu"]) < 10:
        return {"name": name, "ok": False, "note": "too-short"}

    t0 = d["t_imu"][0]
    dur = (d["t_imu"][-1] - t0) / 1e6

    # logged-estimate blow-up: the firmware's OWN pos/vel went non-physical
    # ログ推定の暴走: ファーム自身の pos/vel が非物理値に飛んだ
    ref_pos_h_max = float(d["pos_h"].max()) if len(d["pos_h"]) else 0.0
    ref_pos_z_absmax = float(np.abs(d["pos_z"]).max()) if len(d["pos_z"]) else 0.0

    s = {
        "name": name, "ok": True, "dur_s": dur,
        "has_status": len(d["t_st"]) > 0,
        "state_seq": state_seq(d["st"]),
        "ref_pos_h_max": ref_pos_h_max,
        "ref_pos_z_absmax": ref_pos_z_absmax,
        "v_min": float(d["volt"][d["volt"] > 0].min()) if (d["volt"] > 0).any() else 0.0,
        "_d": d, "_t0": t0,
    }
    mo = detect_motion(d)
    s.update(gyro_max=mo["gyro_max"], acc_std=mo["acc_std"],
             inverted=mo["inverted"], invert_frac=mo["invert_frac"],
             cat=heuristic_category(s, mo))
    return s


def plot_log(s):
    """One overview figure per log: tilt, height (ToF-ish via -pos_z), horizontal
    travel, speed, thrust, flight-state band. Drawn on the IMU time base.
    ログ毎の俯瞰図1枚: 傾き・高度(-pos_z)・水平移動・速度・推力・飛行状態帯。"""
    d, t0 = s["_d"], s["_t0"]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    ti = (d["t_imu"] - t0) / 1e6
    ax[0].plot(ti, d["tilt"], "C3-", lw=0.6)
    ax[0].axhline(90, color="0.6", ls="--", lw=0.8, label="90° (inverted)")
    ax[0].set_ylabel("tilt [deg]")
    ax[0].legend(fontsize=7, loc="upper right")
    inv_txt = "none"
    if s.get("inverted") and s.get("invert_frac") is not None:
        inv_txt = f"@{s['invert_frac']*100:.0f}% of record"
    ax[0].set_title(
        f"{s['name']}  dur={s['dur_s']:.0f}s  [{s.get('cat','?')}]  "
        f"gyro_max={s.get('gyro_max',0):.1f}rad/s  inversion={inv_txt}\n"
        f"status={'yes' if s['has_status'] else 'NO'}  states: {s['state_seq']}",
        fontsize=8)

    if len(d["t_pv"]):
        tp = (d["t_pv"] - t0) / 1e6
        ax[1].plot(tp, -d["pos_z"], "C0-", lw=0.7, label="-pos_z (height)")
        ax[1].set_ylabel("height [m]")
        ax[1].legend(fontsize=7, loc="upper right")
        ax[2].plot(tp, d["pos_h"], "C2-", lw=0.7, label="horizontal |pos|")
        ax[2].plot(tp, d["spd"], "C1-", lw=0.5, alpha=0.7, label="speed |vel|")
        ax[2].set_ylabel("pos_h / speed")
        ax[2].legend(fontsize=7, loc="upper right")

    if len(d["t_cr"]):
        tc = (d["t_cr"] - t0) / 1e6
        ax[3].plot(tc, d["thrust"], "C4-", lw=0.7, label="total_thrust")
    ax[3].set_ylabel("thrust")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(fontsize=7, loc="upper right")

    # flight-state band as colored vspans on the top axis
    # 飛行状態を上段に色帯で重ねる
    if len(d["t_st"]):
        colors = {"IDLE": "0.85", "ARMED": "#cfe", "FLYING": "#cfc",
                  "LANDING": "#fec", "ERROR": "#fcc", "INIT": "0.9", "CALIB": "0.9"}
        tst = (d["t_st"] - t0) / 1e6
        for k in range(len(tst)):
            s_i = d["st"][k]
            nm = FLIGHT_STATE[s_i] if isinstance(s_i, int) and 0 <= s_i < len(FLIGHT_STATE) else "?"
            t_a = tst[k]
            t_b = tst[k + 1] if k + 1 < len(tst) else ti[-1]
            ax[0].axvspan(t_a, t_b, color=colors.get(nm, "white"), alpha=0.4, zorder=0)

    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    os.makedirs(OUTDIR, exist_ok=True)
    fig.savefig(os.path.join(OUTDIR, f"{s['name']}.png"), dpi=85,
                bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    rows = []
    for i, log in enumerate(LOGS):
        s = summarize(log)
        if s["ok"]:
            plot_log(s)
            s.pop("_d"); s.pop("_t0")
        rows.append(s)
        print(f"[{i+1}/{len(LOGS)}] {s['name']} "
              f"{'(too-short)' if not s['ok'] else ''}", file=sys.stderr)

    ok = [r for r in rows if r["ok"]]

    # group counts for the heuristic categories (to be confirmed by eye)
    # ヒューリスティック分類の集計（図で要確認）
    from collections import Counter
    catc = Counter(r["cat"] for r in ok)

    md = ["# M7 ログ地図 — 全フライトログの事実一覧（判定でなく観測事実）",
          "",
          "> 各ログが実際に何だったかを俯瞰するための事実テーブル。**仮分類列は粗い"
          "ヒューリスティックで、必ず `docs/logmap/<name>.png` の俯瞰図を目で見て"
          "確定すること**（末尾 `?` は暫定の意）。",
          "",
          "> 注: 本コーパスは制御出力（total_thrust/motor_duty/throttle）が全ログ0で"
          "記録されており飛行検出に使えないため、運動は IMU の gyro/accel から検出した。",
          "",
          "**仮分類の内訳（要目視確認）:** "
          + " / ".join(f"{k}={v}" for k, v in catc.most_common()),
          "",
          "| log | 仮分類 | 記録秒 | status | 飛行状態の遷移 | gyro最大[rad/s] "
          "| accel-std | 持続反転 | ログ水平最大[m] | ログ\\|pos_z\\|最大[m] | 最低電圧 |",
          "|-----|:--:|------:|:--:|------|------:|------:|:--:|------:|------:|------:|"]
    # order: by category then by duration, so similar logs sit together
    cat_order = {"flew?": 0, "flew+landflip?": 1, "crash?": 2,
                 "est-blowup?": 3, "static?": 4}
    for r in sorted(ok, key=lambda r: (cat_order.get(r["cat"], 9), -r["dur_s"])):
        inv = "—"
        if r["inverted"] and r["invert_frac"] is not None:
            inv = f"@{r['invert_frac']*100:.0f}%"
        md.append(
            f"| {r['name']} | {r['cat']} | {r['dur_s']:.0f} | "
            f"{'✓' if r['has_status'] else '—'} | {r['state_seq']} "
            f"| {r['gyro_max']:.1f} | {r['acc_std']:.2f} | {inv} "
            f"| {r['ref_pos_h_max']:.1f} | {r['ref_pos_z_absmax']:.1f} "
            f"| {r['v_min']:.2f} |")
    bad = [r for r in rows if not r["ok"]]
    if bad:
        md += ["", f"**使用不可（短すぎ等）: {len(bad)}本** — "
               + ", ".join(r["name"] for r in bad)]
    md += ["", f"合計 {len(rows)} 本（解析可 {len(ok)}・使用不可 {len(bad)}）。"
           f"status 記録あり {sum(1 for r in ok if r['has_status'])} 本。",
           "",
           "**仮分類の意味:** flew?=動いた・直立・推定有界 / flew+landflip?=飛行後"
           "記録末尾で反転(着地後手持ち疑い) / crash?=記録中に持続反転 / "
           "est-blowup?=動いたが推定が巨大値へ / static?=ほぼ静止（飛んでいない疑い）"]

    with open(os.path.join(OUTDIR, "logmap.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print(f"\n[m7_logmap] table -> {OUTDIR}/logmap.md", file=sys.stderr)
    print(f"[m7_logmap] {len(ok)} per-log figures -> {OUTDIR}/<name>.png", file=sys.stderr)


if __name__ == "__main__":
    main()
