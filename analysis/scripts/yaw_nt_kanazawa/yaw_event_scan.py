"""
yaw_event_scan.py
==================
全ログを走査し、飛行区間内でヨー角が閾値以上回転したイベントを機械的に抽出する。
結論の先取りをせず閾値ベースで検出するのみ。yawlib.py の関数を使用する。

出力: yaw_events.json (events, flights, sign_check, notes)
"""

import json
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yawlib import load_jsonl, quat_to_yaw, unwrap_deg, detect_flight_segments, mode_str

LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"
FILES = [
    "stampfly_udp_20260627T020050.jsonl",
    "stampfly_udp_20260627T020137.jsonl",
    "stampfly_udp_20260627T164536.jsonl",
    "stampfly_udp_20260627T164611.jsonl",
    "stampfly_udp_20260627T165645.jsonl",
    "stampfly_udp_20260627T165713.jsonl",
    "stampfly_udp_20260629T145357.jsonl",
    "stampfly_udp_20260629T145526.jsonl",
]

WINDOW_S = 8.0
THRESHOLD_DEG = 45.0
MAX_EVENTS_BEFORE_RAISE = 30
RAISED_THRESHOLD_DEG = 60.0


def nearest_index(sorted_t, t):
    """sorted_t (昇順) の中で t に最も近いインデックスを返す（二分探索）。"""
    i = np.searchsorted(sorted_t, t)
    if i <= 0:
        return 0
    if i >= len(sorted_t):
        return len(sorted_t) - 1
    before = sorted_t[i - 1]
    after = sorted_t[i]
    return i - 1 if (t - before) <= (after - t) else i


def find_events_in_segment(t_imu_seg, psi_deg_seg, window_s, threshold_deg):
    """
    セグメント内でスライディングウィンドウ的に |Δψ| >= threshold_deg となる
    区間を検出する。実装: 各サンプルを起点に window_s 以内で最大/最小偏差を
    調べ、閾値を超えるペアをイベント候補とし、重なる候補をマージ、
    境界を局所極値まで拡張する。
    """
    n = len(t_imu_seg)
    if n < 2:
        return []

    # まず粗く: 各点 i について、t_imu_seg[i]+window_s 以内の最大|psi[j]-psi[i]|を調べる
    candidate_intervals = []  # (i0, i1) インデックスペア（i0<i1）
    j_end = 0
    for i in range(n):
        t_limit = t_imu_seg[i] + window_s
        if j_end < i:
            j_end = i
        while j_end + 1 < n and t_imu_seg[j_end + 1] <= t_limit:
            j_end += 1
        if j_end <= i:
            continue
        # ウィンドウ内 [i, j_end] で psi の最大・最小を取るインデックス
        seg = psi_deg_seg[i:j_end + 1]
        if seg.size == 0:
            continue
        local_max_idx = i + int(np.argmax(seg))
        local_min_idx = i + int(np.argmin(seg))
        dpsi_up = psi_deg_seg[local_max_idx] - psi_deg_seg[i]
        dpsi_down = psi_deg_seg[local_min_idx] - psi_deg_seg[i]
        if abs(dpsi_up) >= threshold_deg:
            lo, hi = sorted((i, local_max_idx))
            candidate_intervals.append((lo, hi))
        if abs(dpsi_down) >= threshold_deg:
            lo, hi = sorted((i, local_min_idx))
            candidate_intervals.append((lo, hi))

    if not candidate_intervals:
        return []

    # インデックス区間をマージ（重なるものを結合）
    candidate_intervals.sort()
    merged = [list(candidate_intervals[0])]
    for lo, hi in candidate_intervals[1:]:
        if lo <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])

    # 境界を局所極値まで拡張: 各マージ区間について、区間内の psi 最大・最小の
    # 位置を実際のイベント境界とする（オーバーシュートを含めた実回転量を測るため）
    events = []
    for lo, hi in merged:
        # 探索範囲を少し広げて局所極値を探す(前後0.5秒)
        pad = 0.5
        t_lo = t_imu_seg[lo] - pad
        t_hi = t_imu_seg[hi] + pad
        i_search_lo = nearest_index(t_imu_seg, t_lo)
        i_search_hi = nearest_index(t_imu_seg, t_hi)
        i_search_lo = max(0, i_search_lo)
        i_search_hi = min(n - 1, i_search_hi)
        seg = psi_deg_seg[i_search_lo:i_search_hi + 1]
        local_argmax = i_search_lo + int(np.argmax(seg))
        local_argmin = i_search_lo + int(np.argmin(seg))
        # 実際の開始・終了は元のマージ区間の端点と局所極値のうち、より外側のものを使う
        i0 = min(lo, local_argmin, local_argmax)
        i1 = max(hi, local_argmin, local_argmax)
        i0 = max(0, i0)
        i1 = min(n - 1, i1)
        dpsi = psi_deg_seg[i1] - psi_deg_seg[i0]
        events.append((i0, i1, dpsi))

    return events


def main():
    all_events = []
    all_flights = []
    sign_check_lines = []
    notes_lines = []

    threshold_deg = THRESHOLD_DEG
    raised = False

    # 一旦全ファイルでイベントを集めてから、件数を見て閾値を上げるかもしれないので
    # 2パスで処理する
    for pass_num in (1, 2):
        all_events = []
        all_flights = []
        for fname in FILES:
            path = os.path.join(LOG_DIR, fname)
            if not os.path.exists(path):
                notes_lines.append(f"{fname}: ファイルが存在しない")
                continue

            data = load_jsonl(path)

            if "imu" not in data or "ctrl_ref" not in data:
                notes_lines.append(f"{fname}: imu または ctrl_ref レコードが存在しない")
                continue

            imu = data["imu"]
            ctrl_ref = data["ctrl_ref"]

            # ts 単調性・ラップ検査 (uint32境界 ~4294.967296秒 相当のマイクロ秒ラップ)
            ts_us_raw = imu["ts_us_raw"]
            dt_raw = np.diff(ts_us_raw)
            n_reversal = int(np.sum(dt_raw < 0))
            n_duplicate = int(np.sum(dt_raw == 0))
            n_large_jump = int(np.sum(dt_raw > 1_000_000))  # 1秒超のジャンプ
            if pass_num == 1:
                if n_reversal > 0:
                    notes_lines.append(
                        f"{fname}: imu.ts が逆行する箇所 {n_reversal} 件"
                        f"（uint32ラップ等の疑い、要個別確認）"
                    )
                if n_duplicate > 0:
                    notes_lines.append(
                        f"{fname}: imu.ts が重複（dt=0）する箇所 {n_duplicate} 件"
                        f"（逆行ではなく同一tsの重複レコード。ラップは未検出）"
                    )
                if n_large_jump > 0:
                    notes_lines.append(
                        f"{fname}: imu.ts に1秒超のギャップ {n_large_jump} 件"
                    )

            t_imu = imu["ts"]
            psi_rad_full = quat_to_yaw(imu["quat"])

            if "motor_duty" not in ctrl_ref:
                notes_lines.append(f"{fname}: ctrl_ref.motor_duty が存在しない")
                continue

            segments = detect_flight_segments(
                ctrl_ref["ts"], ctrl_ref["motor_duty"],
                min_duration_s=2.0, merge_gap_s=1.0,
            )

            if not segments:
                # 飛行区間なしでもファイル自体は flights に記録しない
                # (仕様: 飛行区間が全く検出されない場合は該当なしとして notes に記す)
                notes_lines.append(f"{fname}: 飛行区間（motor_duty>0.05が2秒以上）が検出されなかった")
                continue

            status = data.get("status", None)
            gyro_yaw_full = imu["gyro"][:, 2]  # rad/s

            for (t0, t1) in segments:
                i0 = nearest_index(t_imu, t0)
                i1 = nearest_index(t_imu, t1)
                if i1 - i0 < 5:
                    continue

                t_seg = t_imu[i0:i1 + 1]
                psi_rad_seg = psi_rad_full[i0:i1 + 1]
                psi_deg_seg_unwrapped = unwrap_deg(psi_rad_seg)
                gyro_yaw_seg = gyro_yaw_full[i0:i1 + 1]  # rad/s, FRD想定

                # ---- 反証: dpsi/dt (quat由来) と gyro[2] の相関 ----
                if len(t_seg) > 10:
                    dt = np.diff(t_seg)
                    dt[dt <= 0] = np.nan
                    dpsi_dt = np.diff(psi_deg_seg_unwrapped) / dt / 180.0 * np.pi  # rad/s
                    gyro_mid = 0.5 * (gyro_yaw_seg[:-1] + gyro_yaw_seg[1:])
                    valid = np.isfinite(dpsi_dt) & np.isfinite(gyro_mid)
                    if np.sum(valid) > 10 and np.std(dpsi_dt[valid]) > 1e-6 and np.std(gyro_mid[valid]) > 1e-6:
                        corr = np.corrcoef(dpsi_dt[valid], gyro_mid[valid])[0, 1]
                    else:
                        corr = float("nan")
                else:
                    corr = float("nan")

                max_abs_dpsi = float(np.max(psi_deg_seg_unwrapped) - np.min(psi_deg_seg_unwrapped))

                # 区間内で出現した mode の一覧（ctrl_ref から）
                ctrl_t = ctrl_ref["ts"]
                m0 = nearest_index(ctrl_t, t0)
                m1 = nearest_index(ctrl_t, t1)
                modes_in_seg = ctrl_ref["mode"][m0:m1 + 1]
                modes_in_seg = modes_in_seg[np.isfinite(modes_in_seg)]
                unique_modes = sorted(set(int(m) for m in modes_in_seg))
                modes_str = ",".join(mode_str(m) for m in unique_modes) if unique_modes else "unknown"

                all_flights.append({
                    "file": fname,
                    "t0": float(t0),
                    "t1": float(t1),
                    "max_abs_dpsi_deg": max_abs_dpsi,
                    "modes": modes_str,
                    "_corr": corr,  # 内部利用、後でsign_checkへ集約
                })

                # ---- イベント抽出 ----
                found = find_events_in_segment(t_seg, psi_deg_seg_unwrapped, WINDOW_S, threshold_deg)
                for (ei0, ei1, dpsi) in found:
                    et0 = float(t_seg[ei0])
                    et1 = float(t_seg[ei1])
                    # peak_rate: このイベント区間内の gyro[2] 最大絶対値 [deg/s]
                    gyro_seg_event = gyro_yaw_seg[ei0:ei1 + 1]
                    if gyro_seg_event.size > 0 and np.any(np.isfinite(gyro_seg_event)):
                        peak_rate_dps = float(np.nanmax(np.abs(gyro_seg_event)) * 180.0 / np.pi)
                    else:
                        peak_rate_dps = float("nan")

                    # mode: イベント窓内の最頻値
                    em0 = nearest_index(ctrl_t, et0)
                    em1 = nearest_index(ctrl_t, et1)
                    ev_modes = ctrl_ref["mode"][em0:em1 + 1]
                    ev_modes = ev_modes[np.isfinite(ev_modes)]
                    if ev_modes.size > 0:
                        vals, counts = np.unique(ev_modes.astype(int), return_counts=True)
                        mode_mode = int(vals[np.argmax(counts)])
                        mode_name = mode_str(mode_mode)
                    else:
                        mode_name = "unknown"

                    # voltage: 直近 status
                    voltage_val = float("nan")
                    if status is not None and "voltage" in status and status["ts"].size > 0:
                        st = status["ts"]
                        idx = np.searchsorted(st, et0)
                        idx = min(max(idx - 1, 0), len(st) - 1) if st[min(idx, len(st)-1)] > et0 else min(idx, len(st) - 1)
                        idx = min(idx, len(st) - 1)
                        voltage_val = float(status["voltage"][idx])

                    all_events.append({
                        "file": fname,
                        "t0": et0,
                        "t1": et1,
                        "dpsi_deg": float(dpsi),
                        "peak_rate_dps": peak_rate_dps,
                        "mode": mode_name,
                        "voltage": voltage_val,
                    })

        if len(all_events) > MAX_EVENTS_BEFORE_RAISE and not raised:
            threshold_deg = RAISED_THRESHOLD_DEG
            raised = True
            notes_lines.append(
                f"検出イベント数が {MAX_EVENTS_BEFORE_RAISE} を超えたため、"
                f"閾値を {THRESHOLD_DEG}度から{RAISED_THRESHOLD_DEG}度に引き上げて再抽出した"
            )
            continue  # re-run pass 2 with raised threshold
        else:
            break

    # ---- sign_check の集約 ----
    corrs = [f["_corr"] for f in all_flights if np.isfinite(f["_corr"])]
    for f in all_flights:
        f.pop("_corr", None)

    if corrs:
        mean_corr = float(np.mean(corrs))
        min_corr = float(np.min(corrs))
        max_corr = float(np.max(corrs))
        sign_check = (
            f"dpsi/dt(quatアンラップ由来)とgyro[2]の相関係数: "
            f"平均={mean_corr:.4f}, 最小={min_corr:.4f}, 最大={max_corr:.4f} "
            f"({len(corrs)}区間で計算)。"
        )
        if mean_corr < 0:
            sign_check += " 平均が負であり、gyro[2]とquat由来ヨーレートの符号規約が食い違っている可能性がある。"
        elif mean_corr > 0.5:
            sign_check += " 正の強い相関であり、gyro[2]とquat由来ヨーレートの符号は整合している。"
        else:
            sign_check += " 相関が弱い区間を含む（機体がほぼ静止/低角速度の区間ではノイズにより相関が下がりうる）。"
    else:
        sign_check = "相関係数を計算できる区間がなかった（全区間でstd不足またはサンプル不足）。"

    result = {
        "events": all_events,
        "flights": all_flights,
        "sign_check": sign_check,
        "notes": " / ".join(notes_lines) if notes_lines else "特記事項なし",
    }

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yaw_events.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"events: {len(all_events)}, flights: {len(all_flights)}")
    print(f"threshold_deg used: {threshold_deg}")
    print(f"sign_check: {sign_check}")
    print(f"notes: {result['notes']}")
    print(f"written to {out_path}")


if __name__ == "__main__":
    main()
