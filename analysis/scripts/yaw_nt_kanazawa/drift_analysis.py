"""
drift_analysis.py
==================
「遅いヨードリフト」機構解明スクリプト。

対象:
  - stampfly_udp_20260627T165713.jsonl  飛行区間2 [56.3, 196.5]s (ドリフト223.8°)
  - stampfly_udp_20260627T164611.jsonl  飛行区間1 [33.1, 199.6]s (ドリフト84.8°)
  - stampfly_udp_20260627T020050.jsonl  E0 -88°イベント事後確認

手順は task 指示の 1-6 に対応。yawlib.py の load_jsonl/quat_to_yaw/unwrap_deg を再利用。
"""
import sys
import json
import numpy as np

sys.path.insert(0, "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad")
from yawlib import load_jsonl, quat_to_yaw, unwrap_deg

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"
OUT_DIR = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"

KP_YAWHOLD = 3.0          # attitude.yawhold.kp [1/s]
RATE_MAX = 2.0             # attitude.yawhold.rate_max [rad/s]
RATE_MAX_MARGIN = 1.9      # 「クランプ未満」判定に使う閾値 [rad/s]  (task指定)
STICK_NEUTRAL_THRESH = 0.1 # ctrl.yaw 中立判定閾値 (task指定)
QUIET_GYRO_THRESH_DEGS = 20.0  # 静穏区間判定: |gyro_z| < 20 deg/s
QUIET_MIN_DUR_S = 5.0
GAP_BREAK_S = 0.2          # これ以上のtsギャップは区間分断とみなす
JUMP_THRESH_DEG = 10.0     # implied_target ジャンプ検出閾値


def nearest_interp(t_query, t_src, v_src, max_gap=None):
    """t_src (昇順) 上の v_src を t_query へ最近傍で対応付ける。
    max_gap 指定時は |t_query - 最近傍t_src| > max_gap のとき NaN。"""
    idx = np.searchsorted(t_src, t_query)
    idx = np.clip(idx, 1, len(t_src) - 1)
    left = t_src[idx - 1]
    right = t_src[idx]
    choose_right = (t_query - left) > (right - t_query)
    idx_final = np.where(choose_right, idx, idx - 1)
    out = v_src[idx_final]
    if max_gap is not None:
        dt = np.abs(t_query - t_src[idx_final])
        out = np.where(dt <= max_gap, out, np.nan)
    return out


def find_quiet_segments(t, gyro_z_degs, thresh=QUIET_GYRO_THRESH_DEGS,
                         min_dur=QUIET_MIN_DUR_S, gap_break=GAP_BREAK_S):
    """|gyro_z| < thresh が連続する区間 (>= min_dur) を検出して [(t0,t1),...] を返す。
    ts のギャップが gap_break を超えたらそこで区間を分断する（ギャップをまたがない）。"""
    n = len(t)
    quiet = np.abs(gyro_z_degs) < thresh
    dt = np.diff(t, prepend=t[0])
    continuous = dt <= gap_break
    segs = []
    seg_start = None
    for i in range(n):
        ok = quiet[i] and (continuous[i] or i == 0)
        if ok and seg_start is None:
            seg_start = i
        elif not ok and seg_start is not None:
            if t[i - 1] - t[seg_start] >= min_dur:
                segs.append((seg_start, i - 1))
            seg_start = None
    if seg_start is not None and t[n - 1] - t[seg_start] >= min_dur:
        segs.append((seg_start, n - 1))
    return segs


def analyze_flight(jsonl_name, seg_bounds, label, fast_events):
    path = f"{LOG_DIR}/{jsonl_name}"
    data = load_jsonl(path)

    t_imu = data["imu"]["ts"]
    psi_rad = quat_to_yaw(data["imu"]["quat"])
    gyro_z_degs = np.degrees(data["imu"]["gyro"][:, 2])

    t_rate = data["rate_ref"]["ts"]
    rate_ref_z = data["rate_ref"]["rate_ref"][:, 2]  # rad/s, 同じtickでimuと同時刻のはず

    t_ctrl = data["ctrl"]["ts"]
    ctrl_yaw = data["ctrl"]["yaw"]
    ctrl_throttle = data["ctrl"]["throttle"]

    t_ctrl_ref = data["ctrl_ref"]["ts"]
    mode = data["ctrl_ref"]["mode"]

    t_pos = data["posvel"]["ts"]
    altitude = -data["posvel"]["pos"][:, 2]  # NED z -> 高度[m]

    t_status = data["status"]["ts"]
    flight_state = data["status"]["flight_state"]

    seg_t0, seg_t1 = seg_bounds
    seg_mask_imu = (t_imu >= seg_t0) & (t_imu <= seg_t1)

    t_seg = t_imu[seg_mask_imu]
    psi_seg_rad = psi_rad[seg_mask_imu]
    psi_seg_deg = unwrap_deg(psi_seg_rad)
    gyro_z_seg = gyro_z_degs[seg_mask_imu]

    net_drift_deg = psi_seg_deg[-1] - psi_seg_deg[0]     # 始点-終点の正味変化
    peak_to_peak_deg = psi_seg_deg.max() - psi_seg_deg.min()  # 最大-最小(task記載の223.8/84.8度はこちら)
    cumulative_abs_variation_deg = float(np.sum(np.abs(np.diff(psi_seg_deg))))
    duration_s = t_seg[-1] - t_seg[0]

    print(f"\n=== {label} ({jsonl_name}) segment [{seg_t0},{seg_t1}]s ===")
    print(f"  n_imu_samples={len(t_seg)}  duration={duration_s:.1f}s")
    print(f"  psi_start={psi_seg_deg[0]:.1f}deg  psi_end={psi_seg_deg[-1]:.1f}deg  "
          f"NET drift(終-始)={net_drift_deg:.1f}deg")
    print(f"  psi_min={psi_seg_deg.min():.1f}deg(t={t_seg[np.argmin(psi_seg_deg)]:.1f}s)  "
          f"psi_max={psi_seg_deg.max():.1f}deg(t={t_seg[np.argmax(psi_seg_deg)]:.1f}s)  "
          f"PEAK-TO-PEAK={peak_to_peak_deg:.1f}deg  <- task記載の223.8/84.8degはこの値")
    print(f"  累積絶対変位(total path length)={cumulative_abs_variation_deg:.1f}deg")

    # ---- Step 1: 静穏区間検出 & ドリフトレート ----
    quiet_idx_segs = find_quiet_segments(t_seg, gyro_z_seg)
    quiet_results = []
    for i0, i1 in quiet_idx_segs:
        tt = t_seg[i0:i1 + 1]
        pp = psi_seg_deg[i0:i1 + 1]
        # 線形回帰でドリフトレート [deg/s]
        A = np.vstack([tt - tt[0], np.ones_like(tt)]).T
        slope, intercept = np.linalg.lstsq(A, pp, rcond=None)[0]
        quiet_results.append({
            "t0": float(tt[0]), "t1": float(tt[-1]), "dur": float(tt[-1] - tt[0]),
            "psi0": float(pp[0]), "psi1": float(pp[-1]),
            "delta_deg": float(pp[-1] - pp[0]),
            "rate_degs": float(slope),
        })
    print(f"  静穏区間(|gyro_z|<{QUIET_GYRO_THRESH_DEGS}deg/s, >= {QUIET_MIN_DUR_S}s) 数: {len(quiet_results)}")
    for q in quiet_results:
        print(f"    t=[{q['t0']:.1f},{q['t1']:.1f}] dur={q['dur']:.1f}s "
              f"delta={q['delta_deg']:+.2f}deg rate={q['rate_degs']:+.3f}deg/s")
    if quiet_results:
        rates = np.array([q["rate_degs"] for q in quiet_results])
        durs = np.array([q["dur"] for q in quiet_results])
        weighted_rate = np.sum(rates * durs) / np.sum(durs)
        print(f"  静穏区間の継続時間加重平均ドリフトレート = {weighted_rate:+.4f} deg/s "
              f"(total quiet duration={np.sum(durs):.1f}s / flight duration={duration_s:.1f}s = "
              f"{100*np.sum(durs)/duration_s:.0f}%)")
    else:
        weighted_rate = np.nan

    # 連続ランプ vs イベント階段の分類指標:
    # 静穏区間だけでの合計変化 vs フライト全体の変化の比率
    quiet_total_delta = sum(q["delta_deg"] for q in quiet_results) if quiet_results else 0.0
    print(f"  静穏区間だけでの累積変化 = {quiet_total_delta:+.2f}deg "
          f"(peak-to-peak{peak_to_peak_deg:.1f}degの{100*quiet_total_delta/peak_to_peak_deg:.0f}%)")

    # ---- Step 2: implied target 逆算 ----
    # rate_ref のtsに対して psi(unwrap, deg単位で連続) を対応付ける
    # rate_ref と imu は同一tick発行なので直接補間(線形)でよい
    psi_full_unwrap_rad = np.unwrap(psi_rad)  # ログ全体でunwrap (セグメント外も含めて連続性維持)
    psi_at_rate = np.interp(t_rate, t_imu, psi_full_unwrap_rad)  # rad

    seg_mask_rate = (t_rate >= seg_t0) & (t_rate <= seg_t1)
    t_rate_seg = t_rate[seg_mask_rate]
    rate_ref_z_seg = rate_ref_z[seg_mask_rate]
    psi_at_rate_seg = psi_at_rate[seg_mask_rate]

    ctrl_yaw_near = nearest_interp(t_rate_seg, t_ctrl, ctrl_yaw, max_gap=0.2)
    ctrl_throttle_near = nearest_interp(t_rate_seg, t_ctrl, ctrl_throttle, max_gap=0.2)
    mode_near = nearest_interp(t_rate_seg, t_ctrl_ref, mode, max_gap=0.2)
    altitude_near = np.interp(t_rate_seg, t_pos, altitude)

    valid = (
        (np.abs(ctrl_yaw_near) < STICK_NEUTRAL_THRESH) &
        (np.abs(rate_ref_z_seg) < RATE_MAX_MARGIN) &
        (mode_near == 3) &
        ~np.isnan(ctrl_yaw_near)
    )

    implied_target_rad = psi_at_rate_seg + rate_ref_z_seg / KP_YAWHOLD
    implied_target_deg = np.degrees(implied_target_rad)
    implied_target_deg_masked = np.where(valid, implied_target_deg, np.nan)

    n_valid = int(np.sum(valid))
    print(f"  implied_target 有効サンプル数 = {n_valid}/{len(valid)} "
          f"({100*n_valid/len(valid):.1f}%)")

    # implied_target のジャンプ検出（有効サンプル列上で連続差分）
    valid_idx = np.where(valid)[0]
    jumps = []
    if len(valid_idx) > 1:
        vt = t_rate_seg[valid_idx]
        vv = implied_target_deg[valid_idx]
        dv = np.diff(vv)
        dt_v = np.diff(vt)
        for k in range(len(dv)):
            if abs(dv[k]) > JUMP_THRESH_DEG:
                jt = vt[k + 1]
                # 照合: ctrl.yaw, mode, throttle stick, altitude (直近値)
                ctrl_yaw_at = float(nearest_interp(np.array([jt]), t_ctrl, ctrl_yaw, max_gap=0.5)[0])
                thr_at = float(nearest_interp(np.array([jt]), t_ctrl, ctrl_throttle, max_gap=0.5)[0])
                alt_at = float(np.interp(jt, t_pos, altitude))
                fs_at = float(nearest_interp(np.array([jt]), t_status, flight_state, max_gap=2.0))
                jumps.append({
                    "t": float(jt), "delta_deg": float(dv[k]), "dt_gap_s": float(dt_v[k]),
                    "ctrl_yaw": ctrl_yaw_at, "throttle_stick": thr_at,
                    "altitude_m": alt_at, "flight_state": fs_at,
                })
    print(f"  implied_target ジャンプ(>{JUMP_THRESH_DEG}deg) 検出数 = {len(jumps)}")
    for j in jumps:
        print(f"    t={j['t']:.2f}s delta={j['delta_deg']:+.1f}deg (gap={j['dt_gap_s']:.2f}s) "
              f"ctrl.yaw={j['ctrl_yaw']:+.3f} throttle_stick={j['throttle_stick']:.3f} "
              f"alt={j['altitude_m']:.2f}m flight_state={j['flight_state']:.0f}")

    # implied_targetがψに滑らかに追随しているかの指標:
    # 有効区間内でのimplied_targetの標準偏差 vs psiの標準偏差、および相関係数
    if n_valid > 10:
        vv_all = implied_target_deg[valid]
        pp_all = np.degrees(psi_at_rate_seg[valid])
        corr = np.corrcoef(vv_all, pp_all)[0, 1]
        target_range = float(np.nanmax(vv_all) - np.nanmin(vv_all))
        psi_range = float(np.nanmax(pp_all) - np.nanmin(pp_all))
        print(f"  implied_target range={target_range:.1f}deg vs psi range={psi_range:.1f}deg "
              f"(比={100*target_range/psi_range:.0f}%)  相関係数={corr:.3f}")
    else:
        corr = np.nan
        target_range = np.nan
        psi_range = np.nan

    # ---- Step 4: ラチェット検定（速いイベント前後でimplied_targetが更新されたか）----
    ratchet_results = []
    for (e0, e1) in fast_events:
        # イベント直前・直後の有効implied_targetサンプルを探す(±3秒window内)
        pre_mask = valid & (t_rate_seg < e0) & (t_rate_seg > e0 - 5.0)
        post_mask = valid & (t_rate_seg > e1) & (t_rate_seg < e1 + 5.0)
        pre_val = float(np.nanmean(implied_target_deg[pre_mask])) if np.any(pre_mask) else np.nan
        post_val = float(np.nanmean(implied_target_deg[post_mask])) if np.any(post_mask) else np.nan
        # 実際のpsi変化(イベントによる正味回転)
        psi_pre = float(np.interp(e0, t_imu, psi_full_unwrap_rad)) * 180 / np.pi
        psi_post = float(np.interp(e1, t_imu, psi_full_unwrap_rad)) * 180 / np.pi
        event_rotation = psi_post - psi_pre
        target_shift = post_val - pre_val if (not np.isnan(pre_val) and not np.isnan(post_val)) else np.nan
        ratchet_results.append({
            "event": [e0, e1], "target_pre_deg": pre_val, "target_post_deg": post_val,
            "target_shift_deg": target_shift, "psi_event_rotation_deg": event_rotation,
        })
        print(f"  ラチェット検定 event=[{e0},{e1}]s: psi回転={event_rotation:+.1f}deg, "
              f"target_pre={pre_val:.1f}deg target_post={post_val:.1f}deg "
              f"target_shift={target_shift:+.1f}deg")

    # ---- プロット ----
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    axes[0].plot(t_seg, psi_seg_deg, lw=0.8, color="tab:blue", label="psi_quat (unwrap)")
    axes[0].set_ylabel("psi_quat [deg]")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)
    for i0, i1 in quiet_idx_segs:
        axes[0].axvspan(t_seg[i0], t_seg[i1], color="green", alpha=0.1)

    axes[1].plot(t_rate_seg, implied_target_deg_masked, ".", ms=2, color="tab:red",
                 label="implied_target (valid samples)")
    axes[1].plot(t_seg, psi_seg_deg, lw=0.5, color="tab:blue", alpha=0.4, label="psi_quat (ref)")
    for j in jumps:
        axes[1].axvline(j["t"], color="k", lw=0.7, ls="--", alpha=0.6)
    axes[1].set_ylabel("implied target [deg]")
    axes[1].legend(loc="upper right")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t_ctrl, ctrl_yaw, lw=0.8, color="tab:purple", label="ctrl.yaw (stick)")
    axes[2].axhline(STICK_NEUTRAL_THRESH, color="gray", lw=0.5, ls=":")
    axes[2].axhline(-STICK_NEUTRAL_THRESH, color="gray", lw=0.5, ls=":")
    axes[2].set_ylabel("stick yaw")
    axes[2].legend(loc="upper right")
    axes[2].grid(alpha=0.3)

    axes[3].plot(t_pos, altitude, lw=0.8, color="tab:green", label="altitude (-pos_z)")
    axes[3].set_ylabel("altitude [m]")
    axes[3].set_xlabel("t [s]")
    axes[3].legend(loc="upper right")
    axes[3].grid(alpha=0.3)
    axes[3].set_xlim(seg_t0, seg_t1)

    for ev in fast_events:
        for ax in axes:
            ax.axvspan(ev[0], ev[1], color="orange", alpha=0.15)

    fig.suptitle(f"{label}: {jsonl_name} segment [{seg_t0},{seg_t1}]s net_drift={net_drift_deg:.1f}deg peak_to_peak={peak_to_peak_deg:.1f}deg")
    fig.tight_layout()
    short = jsonl_name.replace("stampfly_udp_", "").replace(".jsonl", "")
    out_png = f"{OUT_DIR}/drift_{short}.png"
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"  plot saved: {out_png}")

    return {
        "label": label, "file": jsonl_name, "seg": seg_bounds,
        "net_drift_deg": net_drift_deg, "peak_to_peak_deg": peak_to_peak_deg,
        "cumulative_abs_variation_deg": cumulative_abs_variation_deg, "duration_s": duration_s,
        "quiet_results": quiet_results, "weighted_quiet_rate_degs": weighted_rate,
        "quiet_total_delta_deg": quiet_total_delta,
        "n_valid_implied": n_valid, "n_total_rate": len(valid),
        "jumps": jumps, "implied_vs_psi_corr": corr,
        "implied_range_deg": target_range, "psi_range_deg": psi_range,
        "ratchet_results": ratchet_results,
        "out_png": out_png,
    }


def analyze_e0(jsonl_name="stampfly_udp_20260627T020050.jsonl"):
    path = f"{LOG_DIR}/{jsonl_name}"
    data = load_jsonl(path)
    t_imu = data["imu"]["ts"]
    psi_rad = quat_to_yaw(data["imu"]["quat"])
    psi_deg = unwrap_deg(psi_rad)

    t_pos = data["posvel"]["ts"]
    altitude = -data["posvel"]["pos"][:, 2]

    t_ctrl_ref = data["ctrl_ref"]["ts"]
    motor_duty = data["ctrl_ref"]["motor_duty"]
    duty_mean = np.nanmean(motor_duty, axis=1)

    # 着陸時刻をハードコードせず、高度・モータデューティから実測で検出する
    # (task記載の44.9sは推定値だったため、alt<0.05m かつ duty<0.03 の最初の時刻で上書き)
    idx_alt = np.where((t_pos > 40) & (altitude < 0.05))[0]
    idx_duty = np.where((t_ctrl_ref > 40) & (duty_mean < 0.03))[0]
    t_land_alt = float(t_pos[idx_alt[0]]) if len(idx_alt) else np.nan
    t_land_duty = float(t_ctrl_ref[idx_duty[0]]) if len(idx_duty) else np.nan
    t_landing = np.nanmax([t_land_alt, t_land_duty])  # 遅い方(完全着地)を採用
    print(f"\n=== E0 aftermath ({jsonl_name}) ===")
    print(f"  実測着陸時刻: alt<0.05m at t={t_land_alt:.2f}s, duty<0.03 at t={t_land_duty:.2f}s "
          f"-> 採用 t_landing={t_landing:.2f}s (task記載の44.9sは概算値だったため実測で置換)")

    # イベント前(t<38.9)の基準ヘディング、イベント後(t>40)〜着陸(実測)の推移
    pre_mask = (t_imu > 33.0) & (t_imu < 38.9)
    psi_pre = float(np.mean(psi_deg[pre_mask])) if np.any(pre_mask) else np.nan
    psi_at_40 = float(np.interp(40.0, t_imu, psi_deg))
    psi_at_land = float(np.interp(t_landing, t_imu, psi_deg))
    psi_plateau_post = float(np.mean(psi_deg[(t_imu > t_landing + 1.0) & (t_imu < t_landing + 5.0)])) \
        if np.any((t_imu > t_landing + 1.0) & (t_imu < t_landing + 5.0)) else np.nan
    print(f"  psi(pre-event mean, 33-38.9s) = {psi_pre:.1f}deg")
    print(f"  psi(t=40.0s, event end)       = {psi_at_40:.1f}deg  (event delta = {psi_at_40-psi_pre:+.1f}deg)")
    print(f"  psi(t={t_landing:.1f}s, landing[実測]) = {psi_at_land:.1f}deg  "
          f"(post-event recovery = {psi_at_land-psi_at_40:+.1f}deg, "
          f"remaining offset from pre = {psi_at_land-psi_pre:+.1f}deg)")
    print(f"  psi(着陸後+1〜5s 平均, 静止プラトー) = {psi_plateau_post:.1f}deg "
          f"(pre基準からの最終オフセット = {psi_plateau_post-psi_pre:+.1f}deg)")

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(t_imu, psi_deg, lw=1.0, color="tab:blue")
    axes[0].axvspan(38.9, 40.0, color="orange", alpha=0.2, label="event -88deg")
    axes[0].axvline(t_landing, color="red", ls="--", label=f"landing(実測 t={t_landing:.1f}s)")
    axes[0].set_ylabel("psi_quat [deg]")
    axes[0].set_xlim(25, 55)
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_pos, altitude, lw=1.0, color="tab:green")
    axes[1].axvline(t_landing, color="red", ls="--")
    axes[1].set_xlabel("t [s]")
    axes[1].set_ylabel("altitude [m]")
    axes[1].set_xlim(25, 55)
    axes[1].grid(alpha=0.3)
    fig.suptitle("E0 (020050) aftermath: heading after -88deg event, before landing")
    fig.tight_layout()
    out_png = f"{OUT_DIR}/drift_E0_aftermath.png"
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"  plot saved: {out_png}")

    return {
        "t_landing_measured_s": t_landing,
        "psi_pre_event_deg": psi_pre, "psi_event_end_deg": psi_at_40,
        "psi_landing_deg": psi_at_land,
        "psi_plateau_post_landing_deg": psi_plateau_post,
        "event_delta_deg": psi_at_40 - psi_pre,
        "post_event_recovery_deg": psi_at_land - psi_at_40,
        "net_offset_from_pre_deg": psi_at_land - psi_pre,
        "final_offset_from_pre_deg": psi_plateau_post - psi_pre,
        "out_png": out_png,
    }


if __name__ == "__main__":
    results = {}

    # 165713 飛行区間2
    results["165713"] = analyze_flight(
        "stampfly_udp_20260627T165713.jsonl",
        (56.3, 196.5),
        "165713 segment2",
        fast_events=[(92.2, 103.8), (173.5, 185.5), (29, 50)],
    )

    # 164611 飛行区間1
    results["164611"] = analyze_flight(
        "stampfly_udp_20260627T164611.jsonl",
        (33.1, 199.6),
        "164611 segment1",
        fast_events=[(114.1, 124.8), (182.2, 194.9)],
    )

    results["E0"] = analyze_e0()

    # JSON化して保存（後で参照しやすいように）
    def _sanitize(o):
        if isinstance(o, dict):
            return {k: _sanitize(v) for k, v in o.items()}
        if isinstance(o, list):
            return [_sanitize(v) for v in o]
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        return o

    with open(f"{OUT_DIR}/drift_analysis_results.json", "w") as f:
        json.dump(_sanitize(results), f, indent=2, ensure_ascii=False)
    print("\nAll results saved to drift_analysis_results.json")
