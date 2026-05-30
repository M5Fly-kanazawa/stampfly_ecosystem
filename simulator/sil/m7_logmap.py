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
    t_tof, tof_d = [], []   # bottom ToF: RAW ground distance, NOT through the ESKF
                            # 底面ToF: 生の対地距離（ESKFを通さない独立証拠）
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
            elif mid == "tof_b":
                # status 0 = valid range reading; keep all but mark for filtering
                # status 0 = 有効測距; 全て保持しフィルタは後段で
                t_tof.append(ts)
                tof_d.append(o.get("distance", 0.0))
    return dict(
        t_imu=np.array(t_imu, float), acc_z=np.array(acc_z, float),
        acc_n=np.array(acc_n, float), gyro_n=np.array(gyro_n, float),
        tilt=np.array(tilt, float), qnorm=np.array(qnorm, float),
        t_pv=np.array(t_pv, float), pos_h=np.array(pos_h, float),
        pos_z=np.array(pos_z, float), spd=np.array(spd, float),
        t_st=np.array(t_st, float), st=st, volt=np.array(volt, float),
        t_cr=np.array(t_cr, float), thrust=np.array(thrust, float),
        t_tof=np.array(t_tof, float), tof_d=np.array(tof_d, float),
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


# Airborne / agreement constants — based on the RAW bottom-ToF, which does NOT
# pass through the ESKF, so using it to judge "did it fly" is NOT circular.
# 浮上・一致の定数 — 生の底面ToF基準。ToFはESKFを通らないので「飛んだか」の判定に
# 使っても循環しない。
AIRBORNE_M = 0.15      # raw ToF above this = off the ground
AIRBORNE_HOLD_S = 1.0  # must persist this long to count as a real lift-off
TOF_MAX_VALID = 4.0    # ToF sensor range ceiling [m]


def tof_airborne(d):
    """Did the RAW ToF show a sustained lift-off? Returns (airborne, tof_max).
    生ToFが持続的な浮上を示したか。(浮上したか, ToF最大) を返す。"""
    td = d["tof_d"]
    if len(td) < 5:
        return False, 0.0
    valid = td[(td > 0.0) & (td < TOF_MAX_VALID)]
    tof_max = float(valid.max()) if len(valid) else 0.0
    # sustained run above AIRBORNE_M (ToF ~30 Hz => samples for 1 s ≈ 30)
    # AIRBORNE_M 超えの持続（ToF約30Hz => 1秒≈30サンプル）
    need = max(10, int(AIRBORNE_HOLD_S * 30))
    air = (td > AIRBORNE_M) & (td < TOF_MAX_VALID)
    run = 0
    for v in air:
        run = run + 1 if v else 0
        if run >= need:
            return True, tof_max
    return False, tof_max


def classify(s, mo, has_tof, airborne, tof_max):
    """Circular-free category using RAW ToF as ground truth for height.
    The ESKF position is the THING UNDER TEST, so it is NOT used to decide
    whether the drone flew — only to flag estimator blow-up against the ToF.
    生ToFを高度の真値とする循環なし分類。ESKF位置は検証対象なので飛行判定には
    使わず、ToFとの乖離で推定暴走を検出するためだけに見る。
    末尾 ? は暫定（図で要確認）。"""
    if not has_tof:
        return "no-tof?"            # no independent height evidence => unusable ruler
    if airborne:
        # flew for real. Does the ESKF height roughly track the real ToF height?
        # 実際に飛んだ。ESKF高度が実ToF高度を概ね追えているか？
        if s["ref_pos_z_absmax"] <= tof_max * 2.0 + 1.0 and s["ref_pos_h_max"] < 10:
            return "flew-clean?"    # flew AND estimate tracks => good ruler
        return "flew-divergent?"    # flew BUT estimate diverged => estimator suspect
    # NOT airborne (ToF says on the ground) ...
    if s["ref_pos_z_absmax"] > 2.0 or s["ref_pos_h_max"] > 5.0:
        return "est-blowup?"        # on the ground per ToF, yet ESKF huge => bug (no circularity)
    if mo["moved"]:
        return "ground-motor?"      # moved/vibrated but never lifted off
    return "static?"                # truly still


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
    has_tof = len(d["t_tof"]) > 0
    airborne, tof_max = tof_airborne(d)
    s.update(gyro_max=mo["gyro_max"], acc_std=mo["acc_std"],
             inverted=mo["inverted"], invert_frac=mo["invert_frac"],
             has_tof=has_tof, airborne=airborne, tof_max=tof_max,
             cat=classify(s, mo, has_tof, airborne, tof_max))
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
        ax[1].plot(tp, -d["pos_z"], "C0-", lw=0.7, label="-pos_z (ESKF est.)")
    # RAW ToF height (independent of the ESKF) — the circular-free ground truth.
    # 生ToF高度（ESKF非依存）— 循環しない高度の真値。
    if len(d["t_tof"]):
        tt = (d["t_tof"] - t0) / 1e6
        td = np.where((d["tof_d"] > 0) & (d["tof_d"] < TOF_MAX_VALID), d["tof_d"], np.nan)
        ax[1].plot(tt, td, "k.", ms=1.5, label="ToF dist (RAW, truth)")
        ax[1].axhline(AIRBORNE_M, color="0.6", ls="--", lw=0.7,
                      label=f"airborne {AIRBORNE_M:.2f}m")
    ax[1].set_ylabel("height [m]")
    ax[1].legend(fontsize=7, loc="upper right")

    # horizontal pos + speed (ESKF-derived; shown for divergence visibility)
    # 水平位置・速度(ESKF由来; 発散の可視化用)
    if len(d["t_pv"]):
        tp = (d["t_pv"] - t0) / 1e6
        ax[2].plot(tp, d["pos_h"], "C2-", lw=0.7, label="horizontal |pos|")
        ax[2].plot(tp, d["spd"], "C1-", lw=0.5, alpha=0.7, label="speed |vel|")
        ax[2].legend(fontsize=7, loc="upper right")
    ax[2].set_ylabel("pos_h / speed")

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
    from collections import Counter

    # Split by the independent height evidence: only ToF-bearing logs can serve
    # as a verification ruler (their "did it fly" answer is not circular).
    # 独立した高度証拠で二分: ToFを持つログだけが検証の物差しになりうる
    # （飛行判定が循環しないため）。
    tof_logs = [r for r in ok if r["has_tof"]]
    notof_logs = [r for r in ok if not r["has_tof"]]
    catc = Counter(r["cat"] for r in tof_logs)

    def row(r):
        inv = "—"
        if r["inverted"] and r["invert_frac"] is not None:
            inv = f"@{r['invert_frac']*100:.0f}%"
        air = "浮上" if r["airborne"] else "—"
        return (f"| {r['name']} | {r['cat']} | {r['dur_s']:.0f} | "
                f"{'✓' if r['has_status'] else '—'} | {air} | {r['tof_max']:.2f} "
                f"| {r['ref_pos_z_absmax']:.1f} | {r['ref_pos_h_max']:.1f} "
                f"| {r['gyro_max']:.1f} | {inv} | {r['v_min']:.2f} |")

    md = ["# M7 ログ地図 — 全フライトログの事実一覧（循環しない高度証拠で分類）",
          "",
          "> **設計の核心:** 「飛んだか」は生の底面ToF（`tof_b.distance`, ESKF を通らない"
          "対地距離）で判定する。ESKF の pos/height は**検証対象**なので飛行判定には使わ"
          "ない（使うと循環する）。ToF と ESKF 高度の**乖離**こそが推定器バグの証拠になる。",
          "",
          "> 制御出力（total_thrust/motor_duty/throttle）は全ログ 0 で記録され飛行検出に"
          "使えない（ログ品質の欠陥）。運動は生 gyro/accel から、高度は生 ToF から判定した。",
          "",
          "> 仮分類（末尾 `?`）は粗い自動判定で、必ず `docs/logmap/<name>.png` を目で"
          "見て確定すること。",
          "",
          f"## A. 物差し候補 — 生ToF を持つ {len(tof_logs)} 本（飛行判定が循環しない）",
          "",
          "**仮分類の内訳:** " + " / ".join(f"{k}={v}" for k, v in catc.most_common()),
          "",
          "| log | 仮分類 | 記録秒 | status | 浮上(ToF) | ToF最大[m] "
          "| ESKF\\|pos_z\\|最大[m] | ESKF水平最大[m] | gyro最大 | 反転 | 最低電圧 |",
          "|-----|:--:|------:|:--:|:--:|------:|------:|------:|------:|:--:|------:|"]
    cat_order = {"flew-clean?": 0, "flew-divergent?": 1, "est-blowup?": 2,
                 "ground-motor?": 3, "static?": 4}
    for r in sorted(tof_logs, key=lambda r: (cat_order.get(r["cat"], 9), -r["dur_s"])):
        md.append(row(r))

    md += ["",
           f"## B. 物差しに使えない — 生ToF なしの {len(notof_logs)} 本（4月前半）",
           "",
           "> これらは高度の独立証拠がなく、「飛んだか」を ESKF 出力でしか言えない"
           "（循環）。**物差しには使わない。** 参考に IMU 由来の事実だけ載せる。",
           "",
           "| log | 記録秒 | gyro最大[rad/s] | accel-std | 反転 | ESKF\\|pos_z\\|最大[m] | ESKF水平最大[m] |",
           "|-----|------:|------:|------:|:--:|------:|------:|"]
    for r in sorted(notof_logs, key=lambda r: -r["dur_s"]):
        inv = "—"
        if r["inverted"] and r["invert_frac"] is not None:
            inv = f"@{r['invert_frac']*100:.0f}%"
        md.append(f"| {r['name']} | {r['dur_s']:.0f} | {r['gyro_max']:.1f} "
                  f"| {r['acc_std']:.2f} | {inv} | {r['ref_pos_z_absmax']:.1f} "
                  f"| {r['ref_pos_h_max']:.1f} |")

    bad = [r for r in rows if not r["ok"]]
    if bad:
        md += ["", f"**使用不可（短すぎ等）: {len(bad)}本** — "
               + ", ".join(r["name"] for r in bad)]
    md += ["",
           f"合計 {len(rows)} 本 = 物差し候補(ToFあり) {len(tof_logs)} + "
           f"ToFなし {len(notof_logs)} + 使用不可 {len(bad)}。",
           "",
           "**仮分類の意味（ToF基準・循環なし）:** "
           "flew-clean?=浮上しESKF高度がToFを追えている（良い物差し） / "
           "flew-divergent?=浮上したがESKF高度がToFと乖離（推定器が怪しい） / "
           "est-blowup?=ToFは地上なのにESKFが巨大値（推定器バグ＝循環なしで断定） / "
           "ground-motor?=動いたが浮上せず / static?=静止"]

    with open(os.path.join(OUTDIR, "logmap.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print(f"\n[m7_logmap] table -> {OUTDIR}/logmap.md", file=sys.stderr)
    print(f"[m7_logmap] {len(ok)} per-log figures -> {OUTDIR}/<name>.png", file=sys.stderr)


if __name__ == "__main__":
    main()
