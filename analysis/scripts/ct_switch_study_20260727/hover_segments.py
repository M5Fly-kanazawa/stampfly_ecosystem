#!/usr/bin/env python3
"""
Extract steady-hover, stick-neutral segments from a merged flight-log
DataFrame (see parse_log.py) and compute the implied MOTOR_CT-chain
correction factor for each segment.

"Steady hover" is defined MECHANICALLY (falsifiable, not assumed):
  - flight_mode in {ALT_HOLD(2), POS_HOLD(3)}
  - |altitude rolling std| over a 1.5s centered window < ALT_STD_MAX
  - |vertical velocity| (ESKF, NED->up) stays below VELZ_MAX over that window
  - all three attitude sticks (roll/pitch/yaw) stay below STICK_MAX over
    that window (pilot not maneuvering)
  - duty is armed and not saturated (DUTY_MIN < duty < DUTY_SAT)
Contiguous True regions of at least MIN_SEG_SAMPLES are kept as segments;
each segment reports its mean duty per motor, mean battery voltage, mean
altitude, and duration — plus the implied_corr for both motor chains.

飛行ログの結合 DataFrame から「定常ホバー・スティック中立」区間を機械的に抽出し
（高度変化率・鉛直速度・スティック偏差・duty のしきい値判定、恣意的な区間選びを排除）、
区間ごとに新旧モータチェーンの implied_corr（実測 duty から逆算した推力 ÷ 必要ホバー
推力）を計算する。
"""
import numpy as np
import pandas as pd

from motor_chains import OLD_CHAIN, NEW_CHAIN, HOVER_THRUST_N

# Thresholds — tuned once against a real ALT_HOLD log (stampfly_udp_20260621T232227)
# to give a non-trivial (~5-10%) hover fraction without admitting takeoff/landing
# transients; held fixed across all sessions (no per-log retuning) so the
# extraction criterion cannot be cherry-picked into supporting the hypothesis.
WINDOW_SAMPLES = 75      # 1.5s at 50Hz ctrl_ref rate
ALT_STD_MAX = 0.02       # [m]
VELZ_MAX = 0.10          # [m/s]
STICK_MAX = 0.05         # [-1..1] roll/pitch/yaw stick deflection
DUTY_MIN = 0.05          # armed / producing thrust
DUTY_SAT = 0.95          # not saturated
MIN_SEG_SAMPLES = 50     # >=1.0s contiguous to count as a segment


def find_hover_segments(df: pd.DataFrame) -> pd.DataFrame:
    m = df[df["mode"].isin([2, 3])].reset_index(drop=True)
    if len(m) < WINDOW_SAMPLES:
        return pd.DataFrame()

    alt_std = m["altitude"].rolling(WINDOW_SAMPLES, center=True).std()
    velz_absmax = m["velz_up"].abs().rolling(WINDOW_SAMPLES, center=True).max()
    stick_absmax = (
        m[["stick_roll", "stick_pitch", "stick_yaw"]].abs().max(axis=1)
        .rolling(WINDOW_SAMPLES, center=True).max()
    )
    duty_min = m["duty_mean"].rolling(WINDOW_SAMPLES, center=True).min()
    duty_max = m["duty_mean"].rolling(WINDOW_SAMPLES, center=True).max()

    cand = (
        (alt_std < ALT_STD_MAX)
        & (velz_absmax < VELZ_MAX)
        & (stick_absmax < STICK_MAX)
        & (duty_min > DUTY_MIN)
        & (duty_max < DUTY_SAT)
    ).fillna(False).to_numpy()

    # Group contiguous True runs into segments.
    segments = []
    i = 0
    n = len(cand)
    while i < n:
        if not cand[i]:
            i += 1
            continue
        j = i
        while j < n and cand[j]:
            j += 1
        if (j - i) >= MIN_SEG_SAMPLES:
            segments.append((i, j))
        i = j

    rows = []
    for (i, j) in segments:
        seg = m.iloc[i:j]
        duty4 = seg[["duty_m1", "duty_m2", "duty_m3", "duty_m4"]].to_numpy()
        vbat = seg["voltage"].to_numpy()
        valid = np.isfinite(vbat) & np.all(np.isfinite(duty4), axis=1)
        duty4 = duty4[valid]
        vbat = vbat[valid]
        if len(vbat) < MIN_SEG_SAMPLES // 2:
            continue

        # Per-sample, per-motor inversion, then sum across 4 motors -> total
        # implied thrust for that instant; then average over the segment.
        thrust_old = np.stack(
            [OLD_CHAIN.thrust_of_duty(duty4[:, k], vbat) for k in range(4)], axis=1
        ).sum(axis=1)
        thrust_new = np.stack(
            [NEW_CHAIN.thrust_of_duty(duty4[:, k], vbat) for k in range(4)], axis=1
        ).sum(axis=1)

        implied_corr_old = thrust_old / HOVER_THRUST_N
        implied_corr_new = thrust_new / HOVER_THRUST_N

        # Direct read of the firmware's own instantaneous belief (hover_thrust_
        # + PI correction, learned online) for cross-reference — NOT used as
        # the implied_corr metric itself (that comes from motor_duty inversion
        # above), just reported alongside for context.
        total_thrust_logged = seg["total_thrust"].to_numpy()
        firmware_corr_logged = total_thrust_logged[np.isfinite(total_thrust_logged)]
        firmware_corr_logged = (
            firmware_corr_logged / HOVER_THRUST_N if len(firmware_corr_logged) else np.array([np.nan])
        )

        rows.append({
            "t_start_s": seg["t_s"].iloc[0],
            "t_end_s": seg["t_s"].iloc[-1],
            "duration_s": seg["t_s"].iloc[-1] - seg["t_s"].iloc[0],
            "n_samples": len(seg),
            "mode": int(seg["mode"].mode().iloc[0]),
            "altitude_mean": seg["altitude"].mean(),
            "voltage_mean": float(np.mean(vbat)),
            "duty_m1_mean": float(np.mean(duty4[:, 0])),
            "duty_m2_mean": float(np.mean(duty4[:, 1])),
            "duty_m3_mean": float(np.mean(duty4[:, 2])),
            "duty_m4_mean": float(np.mean(duty4[:, 3])),
            "duty_mean": float(np.mean(duty4)),
            "duty_spread_motors": float(np.std(duty4.mean(axis=0))),
            "implied_corr_old_mean": float(np.mean(implied_corr_old)),
            "implied_corr_old_std": float(np.std(implied_corr_old)),
            "implied_corr_new_mean": float(np.mean(implied_corr_new)),
            "implied_corr_new_std": float(np.std(implied_corr_new)),
            "firmware_corr_logged_mean": float(np.mean(firmware_corr_logged)),
        })

    return pd.DataFrame(rows)
