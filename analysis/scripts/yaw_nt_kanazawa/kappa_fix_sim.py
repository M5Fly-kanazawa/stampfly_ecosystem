"""
kappa_fix_sim.py
================
κ実測値更新（2026-07-17: ミキサー KAPPA 9.71e-3 → 6.12e-3、rate.yaw.kp 等価再スケール、
rate.yaw.max_torque=1.83e-3 既定）の効果を、NT金沢 2026-06-27 実飛行ログの実測外乱で
「真の物理単位系」で検証する閉ループ・ヨー軸シミュレーション。

yaw_cm_sim.py（旧解析）との違い — 単位系の是正:
    旧解析は τ_realized をファームウェア鏡写しの κ=9.71e-3 で再構成しており、
    「実現トルク」「外乱 τ_d」「対策効果」がすべて旧κ単位系（真値の混合）だった。
    真の物理では κ_true=6.12e-3（2026-07-15 コーストダウン実測）なので、
      - 実飛行の真の実現トルク = κ_true·ΔT(duty)/1.12   （旧解析の 0.63 倍）
      - 真の外乱 τ_d_true = I_z·ṙ − κ_true·ΔT/1.12       （旧 τ_d とは別物: I_z·ṙ は
        物理量のため、単純な 0.63 倍スケールにはならない）
    本スクリプトは torque_budget の KAPPA を実測値へ差し替えて全量を真単位系で再構成し、
    新旧ファームウェア構成を同一の τ_d_true で比較する。

ファームウェア指令 → 真の物理トルクの写像（検証対象の物理）:
    ミキサー: ΔT_cmd = τ_cmd/κ_mixer（actuator.cpp mixerCompute、指令空間）
    モータ曲線チェーンは飛行ログ較正済み（hover corr 1.12: 指令推力の 1/1.12 が実推力）
    → τ_true = κ_true·ΔT_cmd/1.12 = τ_cmd·(κ_true/κ_mixer)/1.12
      旧FW (κ_mixer=9.71e-3): τ_true = 0.5628·τ_cmd  ← ヨーは指令の 56% しか出ていなかった
      新FW (κ_mixer=6.12e-3): τ_true = 0.8929·τ_cmd  ← κ 誤差解消、残りは較正済み 1.12 のみ

比較シナリオ（全て同一 τ_d_true 駆動、ti=0.8/td=0.01/η=0.125/ヘディングホールド共通）:
    T0: 旧FW 飛行構成      kp=1.901691e-3, cap=2.2e-3  （NT金沢で回された構成、再現検証用）
    T1: 旧FW + 旧治療      kp=1.901691e-3, cap=2.9e-3  （前回セッションの S1 相当）
    T2: 新FW 既定（本コミット） kp=1.198594e-3, cap=1.83e-3 （κ修正+等価kp+治療キャップ）
    T3: 新FW 等価キャップ    kp=1.198594e-3, cap=1.387e-3（κ修正のみ・飛行実績完全等価 —
                                                        キャップ寄与の分離用、期待値 ≈ T0）
    理論予測: T2 ≈ T1（物理等価、丸め差 0.1%）、T3 ≈ T0（完全等価）。
    これ自体が κ 修正の「等価換算の正しさ」の数値検証になっている。

出典:
    κ_true=6.12e-3: tools/sysid/defaults.py（2026-07-15 実測、Cq=4.10e-11/Ct=6.7e-9）
    新ゲイン・キャップ: firmware/vehicle/components/sf_core/params.cpp（2026-07-17）
    その他の定数・再構成手法: torque_budget.py の出典コメント参照
"""
import sys
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import torque_budget as tb

# --- override the firmware-mirror kappa with the MEASURED value BEFORE any use ---
# torque_budget の KAPPA をモジュール変数として実測値へ差し替える（全再構成が真単位系になる）
KAPPA_TRUE = 6.12e-3   # measured 2026-07-15 (tools/sysid/defaults.py)
KAPPA_OLD = 9.71e-3    # firmware mixer value until 2026-07-17
tb.KAPPA = KAPPA_TRUE

from yaw_cm_sim import (PID, heading_hold_cmd, find_safe_preroll_start,
                        interp_with_gap_mask, DT_SIM, RECOVERY_THRESH_DEG,
                        PID_ETA, YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
import yawlib

R2D = 180.0 / np.pi

IZZ = tb.IZZ
CORR = tb.HOVER_THRUST_CORR
G_OLD = (KAPPA_TRUE / KAPPA_OLD) / CORR   # commanded->true torque, old firmware = 0.5628
G_NEW = 1.0 / CORR                        # commanded->true torque, new firmware = 0.8929

RATE_YAW_TI = 0.8
RATE_YAW_TD = 0.01

CONFIGS = {
    "T0": dict(label="old FW flight (kp=1.902e-3, cap=2.2e-3)",
               kp=1.901691e-3, cap=2.2e-3, g=G_OLD),
    "T1": dict(label="old FW treated (cap=2.9e-3)",
               kp=1.901691e-3, cap=2.9e-3, g=G_OLD),
    "T2": dict(label="NEW FW default (kp=1.199e-3, cap=1.83e-3)",
               kp=1.198594e-3, cap=1.83e-3, g=G_NEW),
    "T3": dict(label="NEW FW equiv-cap (cap=1.387e-3, isolates kappa fix)",
               kp=1.198594e-3, cap=1.387e-3, g=G_NEW),
}


def build_openloop_grid(ev, preroll_s=10.0):
    """Preroll+event grid of measured yaw rate r and onboard rate_ref (for PID seeding
    and open-loop validation). Mirrors yaw_cm_sim.openloop_validate's grid part."""
    data = tb.load_log(ev["file"])
    imu = data["imu"]
    rr = data["rate_ref"]
    t0, t1 = ev["t0"], ev["t1"]
    ts_imu = imu["ts"]
    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
    grid_start = find_safe_preroll_start(ts_imu, t0, preroll_s=preroll_s)
    n = int(round((t1 - grid_start) / DT_SIM)) + 1
    t_grid = grid_start + np.arange(n) * DT_SIM
    r_grid, _ = interp_with_gap_mask(t_grid, ts_imu, r_full, gap_thresh_s=0.02)
    rr_grid, _ = interp_with_gap_mask(t_grid, rr["ts"], rr["rate_ref"][:, 2], gap_thresh_s=0.02)
    return dict(t_grid=t_grid, r_grid=r_grid, rr_grid=rr_grid)


def seed_pid(cfg, ol, t_query):
    """Replay THIS config's PID over the logged (rate_ref, r) preroll up to t_query.
    For the flight config this reproduces the flight PID state; for a rescaled kp the
    integrator scales with kp, which is exactly the equivalent state (see module doc)."""
    pid = PID(cfg["kp"], RATE_YAW_TI, RATE_YAW_TD, PID_ETA, output_limit=cfg["cap"])
    t_grid = ol["t_grid"]
    idx = int(round((t_query - t_grid[0]) / DT_SIM))
    idx = min(max(idx, 0), len(t_grid) - 1)
    for i in range(idx + 1):
        pid.compute(ol["rr_grid"][i], ol["r_grid"][i], DT_SIM)
    return pid.snapshot()


def simulate(cfg, seed, psi0, r0, tau_d_true, t_sim, envelope_true):
    """Closed-loop yaw axis in TRUE physical units: I_z·ṙ = τ_true + τ_d_true."""
    n = len(t_sim)
    pid = PID(cfg["kp"], RATE_YAW_TI, RATE_YAW_TD, PID_ETA, output_limit=cfg["cap"])
    pid.restore(seed)
    psi = np.zeros(n)
    r = np.zeros(n)
    psi[0], r[0] = psi0, r0
    psi_tgt = psi0
    sat_samples = 0
    for i in range(1, n):
        dt = t_sim[i] - t_sim[i - 1]
        rate_sp = heading_hold_cmd(psi_tgt, psi[i - 1], YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
        tau_cmd = pid.compute(rate_sp, r[i - 1], dt)           # commanded [Nm], capped by cfg.cap
        tau_true = np.clip(tau_cmd * cfg["g"], -envelope_true, envelope_true)
        if abs(tau_cmd) >= 0.999 * cfg["cap"] or abs(tau_true) >= 0.999 * envelope_true:
            sat_samples += 1
        drdt = (tau_true + tau_d_true[i - 1]) / IZZ
        r[i] = r[i - 1] + drdt * dt
        psi[i] = psi[i - 1] + 0.5 * (r[i - 1] + r[i]) * dt
    dev_deg = (psi - psi_tgt) * R2D
    k = int(np.argmax(np.abs(dev_deg)))
    max_dev = float(np.abs(dev_deg[k]))
    recovery = None
    for i in range(k, n):
        if abs(dev_deg[i]) < RECOVERY_THRESH_DEG:
            recovery = float(t_sim[i] - t_sim[0])
            break
    return dict(dev_deg=dev_deg, max_dev_deg=max_dev,
                recovery_time_s=recovery, sat_fraction=sat_samples / max(n - 1, 1))


def run_event(ev):
    data = tb.load_log(ev["file"])
    imu = data["imu"]
    t0, t1 = ev["t0"], ev["t1"]
    ts_imu = imu["ts"]
    eff_t0 = max(t0, float(ts_imu.min()))

    res = tb.analyze_event(ev)   # ALL quantities now in TRUE units (tb.KAPPA overridden)

    half_width = (res["ur_max_possible"] - res["ur_min_possible"]) / 2.0
    envelope_true = float(np.nanmedian(half_width))

    t_cr = res["t"]
    tau_d_raw = res["tau_d"]
    nan_mask = np.isnan(tau_d_raw)
    tau_d_filled = (np.interp(t_cr, t_cr[~nan_mask], tau_d_raw[~nan_mask])
                    if np.any(~nan_mask) else np.nan_to_num(tau_d_raw))
    tau_d_peak = float(np.nanmax(np.abs(tau_d_raw)))

    n = int(round((t1 - eff_t0) / DT_SIM)) + 1
    t_sim = eff_t0 + np.arange(n) * DT_SIM
    tau_d_sim = np.interp(t_sim, t_cr, tau_d_filled)

    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
    r0 = float(np.interp(eff_t0, ts_imu, r_full))
    psi_unwrapped = np.unwrap(yawlib.quat_to_yaw(imu["quat"]))
    psi0 = float(np.interp(eff_t0, ts_imu, psi_unwrapped))
    m_plot = (ts_imu >= eff_t0) & (ts_imu <= t1)
    t_real = ts_imu[m_plot]
    psi_real_deg = np.degrees(psi_unwrapped[m_plot] - psi0)

    ol = build_openloop_grid(ev)

    # Open-loop validation of the flight-config model IN TRUE UNITS:
    # replay old-FW PID on logged (rate_ref, r), map to true torque, compare with the
    # duty-reconstructed true realized torque (independent source).
    pid = PID(CONFIGS["T0"]["kp"], RATE_YAW_TI, RATE_YAW_TD, PID_ETA,
              output_limit=CONFIGS["T0"]["cap"])
    tau_model_true = np.zeros(len(ol["t_grid"]))
    for i in range(len(ol["t_grid"])):
        tau_model_true[i] = pid.compute(ol["rr_grid"][i], ol["r_grid"][i], DT_SIM) * G_OLD
    model_at_cr = np.interp(t_cr, ol["t_grid"], tau_model_true)
    valid = ~np.isnan(res["tau_realized"])
    ol_corr = float(np.corrcoef(model_at_cr[valid], res["tau_realized"][valid])[0, 1]) \
        if np.sum(valid) > 2 else float("nan")

    scenarios = {}
    for key, cfg in CONFIGS.items():
        seed = seed_pid(cfg, ol, eff_t0)
        scenarios[key] = simulate(cfg, seed, psi0, r0, tau_d_sim, t_sim, envelope_true)

    # honesty check: T0 reproduction of the real trajectory
    sim0 = np.interp(t_real, t_sim, scenarios["T0"]["dev_deg"])
    repro_rms = float(np.sqrt(np.mean((sim0 - psi_real_deg) ** 2)))

    return dict(name=ev["name"], file=ev["file"], eff_t0=eff_t0, t1=t1,
                envelope_true_mNm=envelope_true * 1e3,
                tau_d_true_peak_mNm=tau_d_peak * 1e3,
                openloop_corr_true=ol_corr,
                real_max_dev_deg=float(np.max(np.abs(psi_real_deg))),
                repro_rms_deg=repro_rms,
                t_sim=t_sim, t_real=t_real, psi_real_deg=psi_real_deg,
                scenarios=scenarios)


def main():
    out_png_dir = os.environ.get("KFS_PNG_DIR", HERE)
    print("=" * 78)
    print("kappa_fix_sim.py — TRUE-unit closed-loop yaw sim (kappa fix verification)")
    print(f"KAPPA_TRUE={KAPPA_TRUE}  KAPPA_OLD={KAPPA_OLD}  "
          f"G_OLD={G_OLD:.4f}  G_NEW={G_NEW:.4f}  IZZ={IZZ}")
    print("Authority (true, at cap, before duty envelope):")
    for key, cfg in CONFIGS.items():
        print(f"  {key}: cap {cfg['cap']*1e3:.3f} mNm cmd -> {cfg['cap']*cfg['g']*1e3:.3f} mNm true")

    results = {}
    for ev in tb.EVENTS:
        er = run_event(ev)
        results[ev["name"]] = er
        print(f"\n--- {er['name']} ({er['file']}) ---")
        print(f"  true duty-saturation envelope (median) = {er['envelope_true_mNm']:.3f} mNm")
        print(f"  true disturbance peak |tau_d|          = {er['tau_d_true_peak_mNm']:.3f} mNm")
        print(f"  open-loop model corr (true units)      = {er['openloop_corr_true']:.4f}")
        print(f"  real max|dev| = {er['real_max_dev_deg']:.1f} deg   "
              f"T0 repro RMS = {er['repro_rms_deg']:.1f} deg")
        for key, sc in er["scenarios"].items():
            rt = sc["recovery_time_s"]
            rt_s = f"{rt:.1f}s" if rt is not None else "N/R"
            print(f"  {key}: max|dev|={sc['max_dev_deg']:6.1f} deg  recovery={rt_s:>6s}  "
                  f"sat={sc['sat_fraction']*100:4.1f}%   ({CONFIGS[key]['label']})")

        # plot
        fig, ax = plt.subplots(figsize=(10, 5.5))
        t_rel = er["t_sim"] - er["eff_t0"]
        ax.plot(er["t_real"] - er["eff_t0"], er["psi_real_deg"], "k-", lw=1.6, label="real")
        for key, color in [("T0", "tab:orange"), ("T1", "tab:blue"),
                           ("T2", "tab:green"), ("T3", "tab:purple")]:
            ax.plot(t_rel, er["scenarios"][key]["dev_deg"], color=color, lw=1.1,
                    label=f"{key}: {CONFIGS[key]['label']}")
        ax.axhline(0, color="gray", lw=0.6)
        ax.set_xlabel("t [s]"); ax.set_ylabel("psi - psi_tgt [deg]")
        ax.set_title(f"{er['name']} true-unit yaw sim (kappa fix)")
        ax.legend(fontsize=7); ax.grid(alpha=0.3)
        fig.tight_layout()
        p = os.path.join(out_png_dir, f"kappa_fix_sim_{er['name']}.png")
        fig.savefig(p, dpi=140); plt.close(fig)
        print(f"  saved {p}")

    # summary: reductions
    print("\n" + "=" * 78)
    print("Summary — max|dev| [deg] and reduction vs T0 (flight config)")
    print(f"{'event':6s}{'T0':>8s}{'T1':>8s}{'T2':>8s}{'T3':>8s}"
          f"{'T2 vs T0':>12s}{'T2 vs T1':>12s}{'T3 vs T0':>12s}")
    for name, er in results.items():
        d = {k: er["scenarios"][k]["max_dev_deg"] for k in CONFIGS}
        red = lambda a, b: f"{(1 - d[a] / d[b]) * 100:+.1f}%" if d[b] > 0 else "n/a"
        print(f"{name:6s}{d['T0']:8.1f}{d['T1']:8.1f}{d['T2']:8.1f}{d['T3']:8.1f}"
              f"{red('T2','T0'):>12s}{red('T2','T1'):>12s}{red('T3','T0'):>12s}")

    def strip(d):
        out = {}
        for k, v in d.items():
            if isinstance(v, np.ndarray):
                continue
            if isinstance(v, dict):
                out[k] = strip(v)
            elif isinstance(v, (np.floating,)):
                out[k] = float(v)
            else:
                out[k] = v
        return out

    payload = dict(
        constants=dict(KAPPA_TRUE=KAPPA_TRUE, KAPPA_OLD=KAPPA_OLD, G_OLD=G_OLD, G_NEW=G_NEW,
                       IZZ=IZZ, CORR=CORR, RATE_YAW_TI=RATE_YAW_TI, RATE_YAW_TD=RATE_YAW_TD,
                       YAW_HOLD_KP=YAW_HOLD_KP, YAW_HOLD_RATE_MAX=YAW_HOLD_RATE_MAX),
        configs={k: dict(kp=c["kp"], cap=c["cap"], g=c["g"], label=c["label"])
                 for k, c in CONFIGS.items()},
        events={k: strip(v) for k, v in results.items()},
    )
    out_json = os.path.join(HERE, "kappa_fix_sim_results.json")
    with open(out_json, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nSaved -> {out_json}")


if __name__ == "__main__":
    main()
