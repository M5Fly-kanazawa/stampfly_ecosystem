#!/usr/bin/env python3
"""
flight_analysis.py - Flight-log v1 bundle analysis (backend of `sf log analyze`)
flight_analysis.py - フライトログ v1 一式の解析（`sf log analyze` のバックエンド）

Reads a StampFly flight-log v1 bundle (`.sflog.zip` or an extracted
directory; see docs/plans/flight-log-format-plan.md section 2 and
lib/sflog/bundle.py) and prints a plain-text gyro/attitude/position/pilot
report, plus an optional 6-panel PNG. The FFT/PSD sample series is always
the raw `imu` stream de-duplicated on `seq` and sorted by `seq` -- never
resampled, interpolated, or held (the plan's "no filled values in the
primary record" rule; see section 7 for why `imu.csv` legitimately repeats
a timestamp on ~13% of rows).

StampFly フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ。計画書
2節・lib/sflog/bundle.py 参照）を読み、ジャイロ・姿勢・位置・操縦入力の
プレーンテキストレポートと、任意で6パネルの PNG を出力する。FFT/PSD の
標本列は常に生の `imu` ストリームを `seq` で重複除去・整列したものであり、
再サンプリング・補間・保持は一切行わない（一次記録に埋め値を持たない
という計画書の方針。`imu.csv` の約13%の行が正当にタイムスタンプを
再利用する理由は計画書7節を参照）。

Usage:
    .venv/bin/python3 tools/log_analyzer/flight_analysis.py <bundle> [--save FILE]
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

import sflog

# =============================================================================
# Named constants (no magic numbers -- CLAUDE.md convention)
# 名前付き定数（マジックナンバー禁止 -- CLAUDE.md の規約）
# =============================================================================

# Radian gyro columns of imu.csv, keyed by the short axis label used
# throughout this module's stats dicts and report sections.
# imu.csv のラジアン角速度列。本モジュールの統計 dict / レポート節全体で
# 使う短い軸ラベルをキーにする。
GYRO_COLUMNS = {'x': 'gyro_x', 'y': 'gyro_y', 'z': 'gyro_z'}
AXIS_LABELS = {'x': 'Roll', 'y': 'Pitch', 'z': 'Yaw'}

# Duty columns shared by motor.csv (400 Hz) and ctrl_ref.csv (50 Hz) -- see
# _find_hover_window().
# motor.csv（400Hz）と ctrl_ref.csv（50Hz）が共通して持つ duty 列 --
# _find_hover_window() 参照。
DUTY_COLUMNS = ['duty_FR', 'duty_RR', 'duty_RL', 'duty_FL']

# Hover-window detection (same idea as motor_health.py's _hover_window: a
# sustained 4-motor duty sum above hover thrust, skipping the climb
# transient and the pre-landing descent).
# ホバー区間の判定（motor_health.py の _hover_window と同じ考え方: 4モータ
# duty 合計がホバー推力を持続して上回る区間から、上昇の過渡・着陸前の
# 降下を除いたもの）。
HOVER_DUTY_SUM = 2.0        # duty sum indicating powered flight (~4 x 0.5)
HOVER_SKIP_START_S = 1.5    # skip the climb transient after hover onset
HOVER_SKIP_END_S = 1.0      # skip the descent before hover ends
MIN_HOVER_SAMPLES = 50      # minimum over-threshold samples to trust the window
MIN_HOVER_S = 10.0          # minimum window duration to report stationary stats

# Welch PSD / peak search (oscillation analysis).
# Welch PSD・ピーク探索（振動解析）。
WELCH_NPERSEG = 4096
PEAK_BAND_LO_HZ = 0.5
PEAK_BAND_HI_HZ = 50.0
N_DOMINANT_PEAKS = 3

# Time-segment table and tuning-observation thresholds (unchanged from the
# pre-bundle flight_analysis.py).
# 時間区分表・チューニング所見の閾値（旧 flight_analysis.py から変更なし）。
TIME_SEGMENT_S = 5.0
STABLE_GYRO_STD_DEG_S = 5.0
HIGH_GYRO_STD_DEG_S = 15.0
HIGH_PILOT_STD = 0.1
HIGH_GYRO_MEAN_DEG_S = 2.0

# Figure layout.
# 図のレイアウト。
FIG_SIZE = (14, 16)
FIG_DPI = 150
LINE_WIDTH = 0.6

REPORT_RULE = "=" * 72
BUNDLE_SUFFIX = ".sflog.zip"


# =============================================================================
# Small numeric helpers
# 小さな数値ヘルパー
# =============================================================================


def _quat_to_euler_rad(qw, qx, qy, qz):
    """Convert a unit quaternion (w,x,y,z) to roll/pitch/yaw in RADIANS
    (vectorized: accepts numpy arrays or pandas Series).

    Same convention as visualize_stream._quat_to_euler_deg() (body-to-world
    per the repo's quaternion convention), so attitude computed here agrees
    with attitude plotted elsewhere for the same flight -- but this version
    returns radians (converted to degrees by the caller) rather than
    degrees, per this module's own API contract.

    単位クォータニオン(w,x,y,z)を roll/pitch/yaw[ラジアン]に変換する
    （ベクトル化: numpy 配列・pandas Series を受け付ける）。

    visualize_stream._quat_to_euler_deg() と同じ規約（本リポジトリの
    クォータニオン規約で body-to-world）を使うため、同一フライトを他所で
    描画した姿勢と食い違わない -- ただし本モジュールの API 契約により、
    度ではなくラジアンを返す（度への変換は呼び出し側が行う）。
    """
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _dedup_and_sort_by_seq(imu):
    """De-duplicate imu rows on `seq` (defensive against a re-sent packet)
    and sort by `seq` -- this is the "sample series" every FFT/PSD/rate
    computation in this module uses. Never resampled or interpolated.

    `seq` で重複除去し（再送パケットへの防御）、`seq` で並べ替える -- 本
    モジュールの FFT/PSD/レート計算が使う「標本列」はこれ。再サンプリング・
    補間は行わない。
    """
    return imu.drop_duplicates(subset='seq', keep='first').sort_values('seq').reset_index(drop=True)


def _series_stats(values):
    """mean/std/min/max of a 1-D array-like, as plain Python floats.
    1次元配列の平均・標準偏差・最小・最大を素の float で返す。
    """
    s = pd.Series(np.asarray(values, dtype=float))
    return {'mean': float(s.mean()), 'std': float(s.std()), 'min': float(s.min()), 'max': float(s.max())}


def _attitude_stats(attitude_df):
    """Mean/std/min/max roll/pitch/yaw in degrees from the attitude
    quaternion, or None if the stream is absent/empty.
    姿勢クォータニオンから求めた roll/pitch/yaw[deg]の平均・標準偏差・
    最小・最大（ストリームが無ければ None）。
    """
    if attitude_df is None or len(attitude_df) == 0:
        return None
    roll, pitch, yaw = _quat_to_euler_rad(
        attitude_df['quat_w'], attitude_df['quat_x'], attitude_df['quat_y'], attitude_df['quat_z'])
    return {
        'roll': _series_stats(np.degrees(roll)),
        'pitch': _series_stats(np.degrees(pitch)),
        'yaw': _series_stats(np.degrees(yaw)),
    }


def _position_stats(posvel):
    """Position spread (std, peak-to-peak) and horizontal drift from the
    window mean, in meters, or None if posvel is absent/empty.
    位置のばらつき（標準偏差・peak-to-peak）と窓平均からの水平ドリフトを
    [m] で返す（posvel が無ければ None）。
    """
    if posvel is None or len(posvel) == 0:
        return None
    axes = ('pos_x', 'pos_y', 'pos_z')
    std = tuple(float(posvel[c].std()) for c in axes)
    ptp = tuple(float(posvel[c].max() - posvel[c].min()) for c in axes)
    dx = posvel['pos_x'] - posvel['pos_x'].mean()
    dy = posvel['pos_y'] - posvel['pos_y'].mean()
    drift_max = float(np.sqrt(dx * dx + dy * dy).max())
    return {'std': std, 'ptp': ptp, 'drift_max': drift_max}


def _pilot_stats(pilot):
    """Mean/std/min/max for each pilot stick axis, or None if absent/empty.
    各操縦スティック軸の平均・標準偏差・最小・最大（無ければ None）。
    """
    if pilot is None or len(pilot) == 0:
        return None
    return {axis: _series_stats(pilot[axis]) for axis in ('throttle', 'roll', 'pitch', 'yaw')}


def _find_hover_window(duty_df, first_ts):
    """Locate a sustained powered-flight window from the 4-motor duty sum
    (same idea as motor_health.py's _hover_window): skip the climb
    transient and the pre-landing descent, and require a minimum sustained
    duration. Returns (t_lo_us, t_hi_us) or None.

    4モータの duty 合計から、安定した動力飛行区間を探す（motor_health.py
    の _hover_window と同じ考え方）: 上昇の過渡・着陸前の降下を除外し、
    最短継続時間を要求する。(t_lo_us, t_hi_us) または None を返す。
    """
    if duty_df is None or len(duty_df) == 0:
        return None
    duty_sum = duty_df[DUTY_COLUMNS].sum(axis=1)
    powered = duty_df.loc[duty_sum > HOVER_DUTY_SUM, 'timestamp_us']
    if len(powered) < MIN_HOVER_SAMPLES:
        return None
    t_lo = powered.iloc[0] + HOVER_SKIP_START_S * 1e6
    t_hi = powered.iloc[-1] - HOVER_SKIP_END_S * 1e6
    if t_hi <= t_lo or (t_hi - t_lo) / 1e6 < MIN_HOVER_S:
        return None
    return (t_lo, t_hi)


def _welch_psd_deg(gyro_rad, fs):
    """Welch PSD of one gyro axis, converting to deg/s first so the
    returned PSD is in (deg/s)^2/Hz.
    ジャイロ1軸の Welch PSD。先に deg/s へ変換するため、返す PSD の単位は
    (deg/s)^2/Hz になる。
    """
    gyro_deg = np.degrees(np.asarray(gyro_rad, dtype=float))
    nperseg = min(WELCH_NPERSEG, len(gyro_deg))
    freqs, psd = signal.welch(gyro_deg, fs=fs, nperseg=nperseg, detrend='constant')
    return freqs, psd


def _dominant_peaks(freqs, psd):
    """Top N_DOMINANT_PEAKS PSD peaks within [PEAK_BAND_LO_HZ,
    PEAK_BAND_HI_HZ], sorted by height (highest first).
    [PEAK_BAND_LO_HZ, PEAK_BAND_HI_HZ] 帯域内の PSD ピークを上位
    N_DOMINANT_PEAKS 個、高さの降順で返す。
    """
    band = (freqs >= PEAK_BAND_LO_HZ) & (freqs <= PEAK_BAND_HI_HZ)
    band_freqs, band_psd = freqs[band], psd[band]
    idx, _ = signal.find_peaks(band_psd)
    order = np.argsort(band_psd[idx])[::-1][:N_DOMINANT_PEAKS]
    return [(float(band_freqs[i]), float(band_psd[i])) for i in idx[order]]


# =============================================================================
# Compute phase: every number analyze_flight() reports, no printing/plotting
# 計算段階: analyze_flight() が報告する数値全て（表示・作図はしない）
# =============================================================================


def _compute(log):
    """Compute every number analyze_flight() needs, without printing or
    plotting. Returns a dict of intermediates (deduped imu DataFrame,
    per-stream stats, hover window, PSD) that analyze_flight() both prints
    from and returns as its result dict.

    analyze_flight() に必要な数値を全て計算する（表示・作図はしない）。
    重複除去済み imu DataFrame・各ストリームの統計・ホバー区間・PSD を
    まとめた dict を返し、analyze_flight() はそこから表示も戻り値の
    組み立ても行う。
    """
    imu_raw = log.streams['imu']
    imu = _dedup_and_sort_by_seq(imu_raw)
    n = len(imu)
    first_ts = int(imu['timestamp_us'].iloc[0])
    duration_s = (int(imu['timestamp_us'].iloc[-1]) - first_ts) / 1e6
    fs = (n - 1) / duration_s if duration_s > 0 else float('nan')
    repeated_timestamps = int((imu['timestamp_us'] == imu['timestamp_us'].shift(1)).sum())

    gyro_deg = {axis: np.degrees(imu[col]) for axis, col in GYRO_COLUMNS.items()}
    gyro = {axis: _series_stats(series) for axis, series in gyro_deg.items()}

    attitude_df = log.streams.get('attitude')
    posvel_df = log.streams.get('posvel')
    pilot_df = log.streams.get('pilot')
    duty_df = log.streams.get('motor', log.streams.get('ctrl_ref'))

    psd, dominant_hz = {}, {}
    for axis, col in GYRO_COLUMNS.items():
        freqs, pxx = _welch_psd_deg(imu[col], fs)
        psd[axis] = (freqs, pxx)
        dominant_hz[axis] = _dominant_peaks(freqs, pxx)

    return {
        'imu': imu, 'rows': len(imu_raw), 'n': n, 'first_ts': first_ts,
        'duration_s': duration_s, 'fs': fs, 'repeated_timestamps': repeated_timestamps,
        'gyro_deg': gyro_deg, 'gyro': gyro,
        'attitude_df': attitude_df, 'attitude': _attitude_stats(attitude_df),
        'posvel_df': posvel_df, 'position': _position_stats(posvel_df),
        'pilot_df': pilot_df, 'pilot': _pilot_stats(pilot_df),
        'window_us': _find_hover_window(duty_df, first_ts),
        'psd': psd, 'dominant_hz': dominant_hz,
    }


# =============================================================================
# Report sections (plain text)
# レポート各節（プレーンテキスト）
# =============================================================================


def _print_overview(name, source, rows, n, duration_s, fs, repeated_timestamps):
    """Overview: bundle name, source, row/duration/rate, repeated timestamps.
    概要: 一式の名前・取得元・行数/時間/レート・重複タイムスタンプ数。
    """
    pct = 100.0 * repeated_timestamps / n if n else 0.0
    print(REPORT_RULE)
    print(f"Flight Analysis: {name}")
    print(REPORT_RULE)
    print(f"Source: {source or 'unknown'}")
    print(f"IMU rows: {rows}")
    print(f"Duration: {duration_s:.2f} s")
    print(f"Measured sample rate: {fs:.2f} Hz")
    print(f"Repeated timestamps: {repeated_timestamps} ({pct:.1f}%)")
    print(f"  {repeated_timestamps} control cycles ({pct:.1f}%) reused the previous IMU "
          "sample; the spectrum treats rows as uniformly spaced at the measured rate.")


def _print_gyro_stats(gyro):
    """Gyro statistics per axis, in deg/s.
    軸ごとのジャイロ統計 [deg/s]。
    """
    print()
    print(REPORT_RULE)
    print("Gyro statistics (deg/s)")
    print(REPORT_RULE)
    for axis, label in AXIS_LABELS.items():
        st = gyro[axis]
        print(f"  {label:<6} mean={st['mean']:+.3f} std={st['std']:.3f} "
              f"min={st['min']:.2f} max={st['max']:.2f}")


def _print_attitude_stats(attitude):
    """Attitude (roll/pitch/yaw) statistics, in deg.
    姿勢（roll/pitch/yaw）統計 [deg]。
    """
    print()
    print(REPORT_RULE)
    print("Attitude statistics (deg)")
    print(REPORT_RULE)
    for axis in ('roll', 'pitch', 'yaw'):
        st = attitude[axis]
        print(f"  {axis.capitalize():<6} mean={st['mean']:+.3f} std={st['std']:.3f} "
              f"min={st['min']:.2f} max={st['max']:.2f}")


def _print_position_stats(position):
    """Position hold: std/ptp per axis and horizontal drift, in meters.
    位置保持: 軸ごとの標準偏差/peak-to-peak と水平ドリフト [m]。
    """
    print()
    print(REPORT_RULE)
    print("Position hold (posvel, m)")
    print(REPORT_RULE)
    for i, axis in enumerate(('x', 'y', 'z')):
        print(f"  pos_{axis}: std={position['std'][i]:.4f}  ptp={position['ptp'][i]:.4f}")
    print(f"  horizontal drift (max from window mean): {position['drift_max']:.4f} m")


def _print_pilot_stats(pilot):
    """Pilot input statistics per stick axis.
    スティック軸ごとの操縦入力統計。
    """
    print()
    print(REPORT_RULE)
    print("Pilot input statistics")
    print(REPORT_RULE)
    for axis in ('throttle', 'roll', 'pitch', 'yaw'):
        st = pilot[axis]
        print(f"  {axis:<9} mean={st['mean']:+.4f} std={st['std']:.4f} "
              f"min={st['min']:.3f} max={st['max']:.3f}")


def _print_hover_window(window_us, first_ts, imu, gyro_whole, posvel_df, position_whole):
    """Hover window bounds, and whole-log vs in-window gyro/position spread
    (a stationary hover should show a tighter spread inside the window).
    ホバー区間の範囲と、全体区間・区間内でのジャイロ／位置ばらつきの比較
    （静止ホバーなら区間内の方がばらつきが小さいはず）。
    """
    print()
    print(REPORT_RULE)
    print("Hover window")
    print(REPORT_RULE)
    if window_us is None:
        print(f"  not found (needs a sustained duty-sum > {HOVER_DUTY_SUM:.1f} segment "
              f">= {MIN_HOVER_S:.0f} s after trimming climb/descent)")
        return

    t_lo, t_hi = window_us
    lo_s, hi_s = (t_lo - first_ts) / 1e6, (t_hi - first_ts) / 1e6
    print(f"  window: {lo_s:.2f} - {hi_s:.2f} s  ({hi_s - lo_s:.2f} s)")

    mask = (imu['timestamp_us'] >= t_lo) & (imu['timestamp_us'] <= t_hi)
    gyro_win = {axis: _series_stats(np.degrees(imu.loc[mask, col]))
                for axis, col in GYRO_COLUMNS.items()}
    print("  gyro std [deg/s]        whole-log   hover-window")
    for axis, label in AXIS_LABELS.items():
        print(f"    {label:<6}               {gyro_whole[axis]['std']:>9.3f}   {gyro_win[axis]['std']:>11.3f}")

    if posvel_df is None or position_whole is None:
        return
    pmask = (posvel_df['timestamp_us'] >= t_lo) & (posvel_df['timestamp_us'] <= t_hi)
    pos_win = _position_stats(posvel_df[pmask])
    print("  position std [m]        whole-log   hover-window")
    for i, axis in enumerate(('x', 'y', 'z')):
        print(f"    pos_{axis:<3}               {position_whole['std'][i]:>9.4f}   {pos_win['std'][i]:>11.4f}")


def _print_oscillation(dominant_hz):
    """Top dominant frequencies per gyro axis within the search band.
    探索帯域内の、ジャイロ軸ごとの卓越周波数上位。
    """
    print()
    print(REPORT_RULE)
    print(f"Oscillation analysis (Welch PSD, {PEAK_BAND_LO_HZ:.1f}-{PEAK_BAND_HI_HZ:.0f} Hz band)")
    print(REPORT_RULE)
    for axis, label in AXIS_LABELS.items():
        print(f"  {label} dominant frequencies:")
        peaks = dominant_hz[axis]
        if not peaks:
            print("    (no peak found in band)")
            continue
        for i, (f_hz, psd_val) in enumerate(peaks, start=1):
            print(f"    {i}. {f_hz:.2f} Hz  (PSD={psd_val:.4g} (deg/s)^2/Hz)")


def _print_time_segments(t_s, gyro_deg, duration_s, pilot_df, first_ts):
    """5 s time-segment table: roll/pitch/yaw sigma, throttle mean (if
    pilot present), STABLE/UNSTABLE verdict.
    5秒ごとの時間区分表: roll/pitch/yaw の標準偏差、スロットル平均
    （pilot があれば）、STABLE/UNSTABLE 判定。
    """
    print()
    print(REPORT_RULE)
    print(f"Time-segment analysis ({TIME_SEGMENT_S:.0f} s windows)")
    print(REPORT_RULE)
    roll_dps, pitch_dps, yaw_dps = gyro_deg['x'], gyro_deg['y'], gyro_deg['z']
    pilot_t_s = (pilot_df['timestamp_us'] - first_ts) / 1e6 if pilot_df is not None else None

    start = 0.0
    while start < duration_s:
        end = min(start + TIME_SEGMENT_S, duration_s)
        mask = (t_s >= start) & (t_s < end)
        roll_std, pitch_std, yaw_std = roll_dps[mask].std(), pitch_dps[mask].std(), yaw_dps[mask].std()
        stable = "STABLE" if roll_std < STABLE_GYRO_STD_DEG_S and pitch_std < STABLE_GYRO_STD_DEG_S else "UNSTABLE"

        throttle_txt = "n/a"
        if pilot_df is not None:
            pmask = (pilot_t_s >= start) & (pilot_t_s < end)
            if pmask.any():
                throttle_txt = f"{pilot_df.loc[pmask, 'throttle'].mean():.2f}"

        print(f"  {start:5.1f}-{end:5.1f}s: Roll s={roll_std:5.2f} deg/s, "
              f"Pitch s={pitch_std:5.2f} deg/s, Yaw s={yaw_std:5.2f} deg/s, "
              f"Throttle={throttle_txt} [{stable}]")
        start += TIME_SEGMENT_S


def _print_tuning_observations(gyro, pilot):
    """Same heuristics as the pre-bundle flight_analysis.py (high gyro
    variance, high pilot correction activity, non-zero gyro mean), reworded
    to "WARNING:" instead of an emoji per repo convention.
    旧 flight_analysis.py と同じヒューリスティック（高いジャイロ分散、
    高い操縦補正活動、非ゼロのジャイロ平均）を、本リポジトリの記法に
    合わせて絵文字ではなく "WARNING:" で言い換えたもの。
    """
    print()
    print(REPORT_RULE)
    print("Tuning observations")
    print(REPORT_RULE)
    printed_any = False

    roll_std, pitch_std = gyro['x']['std'], gyro['y']['std']
    if roll_std > HIGH_GYRO_STD_DEG_S or pitch_std > HIGH_GYRO_STD_DEG_S:
        printed_any = True
        print("WARNING: High gyro variance detected - possible oscillation")
        print("  Recommendations:")
        print("  - Reduce P gain if oscillations are fast (>5Hz)")
        print("  - Reduce D gain if oscillations are slow (<2Hz) with overshoot")
        print("  - Check for mechanical issues (loose props, motor vibration)")

    if pilot is not None:
        roll_in_std, pitch_in_std = pilot['roll']['std'], pilot['pitch']['std']
        if roll_in_std > HIGH_PILOT_STD or pitch_in_std > HIGH_PILOT_STD:
            printed_any = True
            print("\nWARNING: High pilot correction activity detected")
            print(f"  Roll input σ={roll_in_std:.3f}, Pitch input σ={pitch_in_std:.3f}")
            print("  This indicates the drone is not self-stabilizing well")

    roll_mean, pitch_mean = gyro['x']['mean'], gyro['y']['mean']
    if abs(roll_mean) > HIGH_GYRO_MEAN_DEG_S or abs(pitch_mean) > HIGH_GYRO_MEAN_DEG_S:
        printed_any = True
        print("\nWARNING: Non-zero gyro mean during flight")
        print(f"  Roll mean={roll_mean:.2f}°/s, Pitch mean={pitch_mean:.2f}°/s")
        print("  Possible I-term accumulation or persistent external force")

    if not printed_any:
        print("  none")


# =============================================================================
# PNG (6 stacked panels; panel 3 does NOT share the time x-axis)
# PNG（6パネル縦積み。パネル3だけ時間 x軸を共有しない）
# =============================================================================


def _plot_gyro_roll_pitch(ax, t_s, gyro_deg):
    """Panel 1: gyro roll+pitch [deg/s].
    パネル1: ジャイロ roll・pitch [deg/s]。
    """
    ax.plot(t_s, gyro_deg['x'], linewidth=LINE_WIDTH, label='roll')
    ax.plot(t_s, gyro_deg['y'], linewidth=LINE_WIDTH, label='pitch')
    ax.set_ylabel('Gyro roll/pitch [deg/s]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _plot_gyro_yaw(ax, t_s, gyro_deg):
    """Panel 2: gyro yaw [deg/s].
    パネル2: ジャイロ yaw [deg/s]。
    """
    ax.plot(t_s, gyro_deg['z'], linewidth=LINE_WIDTH, color='C2', label='yaw')
    ax.set_ylabel('Gyro yaw [deg/s]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _plot_psd(ax, psd_results):
    """Panel 3: Welch PSD of the 3 gyro axes, semilogy, own x-axis
    (frequency, not time) -- this is the one panel that does not share x.
    パネル3: ジャイロ3軸の Welch PSD（semilogy）。このパネルだけ x軸が
    周波数であり、時間軸を共有しない。
    """
    for axis, label in AXIS_LABELS.items():
        freqs, psd_val = psd_results[axis]
        ax.semilogy(freqs, psd_val, label=label)
    ax.set_xlim(0, PEAK_BAND_HI_HZ)
    ax.set_xlabel('Frequency [Hz]')
    ax.set_ylabel('PSD [(deg/s)^2/Hz]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3, which='both')


def _plot_attitude_or_accel(ax, t_s, imu, attitude_df):
    """Panel 4: attitude roll/pitch/yaw [deg] if available, else accel_z.
    パネル4: 姿勢 roll/pitch/yaw [deg]（無ければ accel_z）。
    """
    if attitude_df is not None and len(attitude_df) > 0:
        t_att = (attitude_df['timestamp_us'] - imu['timestamp_us'].iloc[0]) / 1e6
        roll, pitch, yaw = _quat_to_euler_rad(
            attitude_df['quat_w'], attitude_df['quat_x'], attitude_df['quat_y'], attitude_df['quat_z'])
        ax.plot(t_att, np.degrees(roll), linewidth=LINE_WIDTH, label='roll')
        ax.plot(t_att, np.degrees(pitch), linewidth=LINE_WIDTH, label='pitch')
        ax.plot(t_att, np.degrees(yaw), linewidth=LINE_WIDTH, label='yaw')
        ax.set_ylabel('Attitude [deg]')
    else:
        ax.plot(t_s, imu['accel_z'], linewidth=LINE_WIDTH, color='purple', label='accel_z')
        ax.set_ylabel('Accel Z [m/s^2]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _plot_pilot_or_note(ax, pilot_df, first_ts):
    """Panel 5: pilot stick inputs if available, else a note (no axes).
    パネル5: 操縦スティック入力（無ければ注記のみ）。
    """
    if pilot_df is not None and len(pilot_df) > 0:
        t_pilot = (pilot_df['timestamp_us'] - first_ts) / 1e6
        for axis in ('throttle', 'roll', 'pitch', 'yaw'):
            ax.plot(t_pilot, pilot_df[axis], linewidth=LINE_WIDTH, label=axis)
        ax.set_ylabel('Pilot stick')
        ax.legend(loc='upper right', ncol=4, fontsize='small')
        ax.grid(True, alpha=0.3)
    else:
        ax.text(0.5, 0.5, 'no pilot stream', ha='center', va='center', transform=ax.transAxes)


def _plot_posvel_or_accel(ax, t_s, imu, posvel_df, first_ts):
    """Panel 6: posvel pos_x/y/z [m] if available, else accel_z.
    パネル6: 位置 pos_x/y/z [m]（無ければ accel_z）。
    """
    if posvel_df is not None and len(posvel_df) > 0:
        t_pos = (posvel_df['timestamp_us'] - first_ts) / 1e6
        for axis in ('pos_x', 'pos_y', 'pos_z'):
            ax.plot(t_pos, posvel_df[axis], linewidth=LINE_WIDTH, label=axis)
        ax.set_ylabel('Position [m]')
    else:
        ax.plot(t_s, imu['accel_z'], linewidth=LINE_WIDTH, color='purple', label='accel_z')
        ax.set_ylabel('Accel Z [m/s^2]')
    ax.set_xlabel('Time [s]')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)


def _build_and_save_png(png_path, t_s, gyro_deg, psd_results, imu, attitude_df, pilot_df, posvel_df, first_ts):
    """Render the 6-panel figure and save it to `png_path` (never calls
    plt.show()).
    6パネルの図を作成し `png_path` へ保存する（plt.show() は呼ばない）。
    """
    fig = plt.figure(figsize=FIG_SIZE)
    ax1 = fig.add_subplot(6, 1, 1)
    ax2 = fig.add_subplot(6, 1, 2, sharex=ax1)
    ax3 = fig.add_subplot(6, 1, 3)  # own x-axis: frequency, not time
    ax4 = fig.add_subplot(6, 1, 4, sharex=ax1)
    ax5 = fig.add_subplot(6, 1, 5, sharex=ax1)
    ax6 = fig.add_subplot(6, 1, 6, sharex=ax1)

    _plot_gyro_roll_pitch(ax1, t_s, gyro_deg)
    _plot_gyro_yaw(ax2, t_s, gyro_deg)
    _plot_psd(ax3, psd_results)
    _plot_attitude_or_accel(ax4, t_s, imu, attitude_df)
    _plot_pilot_or_note(ax5, pilot_df, first_ts)
    _plot_posvel_or_accel(ax6, t_s, imu, posvel_df, first_ts)

    fig.tight_layout()
    fig.savefig(png_path, dpi=FIG_DPI)
    plt.close(fig)
    print(f"Saved: {png_path}")


# =============================================================================
# Public API
# 公開 API
# =============================================================================


def analyze_flight(log: sflog.FlightLog, name: str, png_path=None) -> dict:
    """Analyze one flight-log v1 bundle and print a plain-text report.

    Requires an `imu` stream (raises ValueError otherwise); uses
    attitude/posvel/pilot/ctrl_ref/motor/status when present. Saves a
    6-panel PNG when `png_path` is given (never calls plt.show()). Returns
    the computed numbers as a dict so callers (tests, other tools) can
    assert on them without re-parsing stdout.

    1つのフライトログ v1 一式を解析し、プレーンテキストのレポートを表示する。

    `imu` ストリームを必須とし（無ければ ValueError）、
    attitude/posvel/pilot/ctrl_ref/motor/status はあれば使う。`png_path` が
    与えられれば6パネルの PNG を保存する（plt.show() は呼ばない）。
    計算した数値を dict で返すので、呼び出し側（テスト・他ツール）は
    標準出力を再パースせずに検証できる。

    Raises:
        ValueError: the bundle has no `imu` stream.
    """
    if 'imu' not in log.streams:
        raise ValueError("bundle has no imu stream")

    c = _compute(log)
    t_s = (c['imu']['timestamp_us'] - c['first_ts']) / 1e6
    hover_window_s = None
    if c['window_us'] is not None:
        hover_window_s = (
            (c['window_us'][0] - c['first_ts']) / 1e6,
            (c['window_us'][1] - c['first_ts']) / 1e6,
        )

    _print_overview(name, log.meta.get('source'), c['rows'], c['n'], c['duration_s'], c['fs'],
                    c['repeated_timestamps'])
    _print_gyro_stats(c['gyro'])
    if c['attitude'] is not None:
        _print_attitude_stats(c['attitude'])
    if c['position'] is not None:
        _print_position_stats(c['position'])
    if c['pilot'] is not None:
        _print_pilot_stats(c['pilot'])
    _print_hover_window(c['window_us'], c['first_ts'], c['imu'], c['gyro'], c['posvel_df'], c['position'])
    _print_oscillation(c['dominant_hz'])
    _print_time_segments(t_s, c['gyro_deg'], c['duration_s'], c['pilot_df'], c['first_ts'])
    _print_tuning_observations(c['gyro'], c['pilot'])

    if png_path is not None:
        _build_and_save_png(png_path, t_s, c['gyro_deg'], c['psd'], c['imu'],
                            c['attitude_df'], c['pilot_df'], c['posvel_df'], c['first_ts'])

    return {
        'name': name,
        'source': log.meta.get('source'),
        'rows': c['rows'],
        'duration_s': c['duration_s'],
        'sample_rate_hz': c['fs'],
        'repeated_timestamps': c['repeated_timestamps'],
        'gyro': c['gyro'],
        'attitude': c['attitude'],
        'position': c['position'],
        'pilot': c['pilot'],
        'hover_window_s': hover_window_s,
        'dominant_hz': c['dominant_hz'],
        'png': str(png_path) if png_path is not None else None,
    }


def _bundle_stem(bundle_path: Path) -> str:
    """File/directory name with a trailing `.sflog.zip` removed (Path.stem
    alone strips only the last suffix, leaving ".sflog" behind). Mirrors
    lib/sfcli/commands/log.py's `_bundle_stem()` for the same reason.

    末尾の `.sflog.zip` を除いたファイル/フォルダ名（Path.stem だけでは
    最後の拡張子しか外れず ".sflog" が残る）。同じ理由で
    lib/sfcli/commands/log.py の `_bundle_stem()` と同じ動作にしてある。
    """
    name = bundle_path.name
    return name[: -len(BUNDLE_SUFFIX)] if name.endswith(BUNDLE_SUFFIX) else name


def main():
    """CLI entry point: `sf log analyze`'s backend.
    CLI エントリポイント: `sf log analyze` のバックエンド。
    """
    parser = argparse.ArgumentParser(
        description="Analyze a StampFly flight-log v1 bundle (.sflog.zip or an extracted directory).")
    parser.add_argument('bundle', help="Path to a .sflog.zip file or an extracted bundle directory.")
    parser.add_argument('--save', metavar='FILE',
                        help="PNG output path (default: <bundle-stem>_analysis.png next to the bundle).")
    args = parser.parse_args()

    bundle_path = Path(args.bundle)
    log = sflog.load(bundle_path)
    save_path = Path(args.save) if args.save else bundle_path.parent / f"{_bundle_stem(bundle_path)}_analysis.png"

    analyze_flight(log, bundle_path.name, png_path=str(save_path))


if __name__ == "__main__":
    main()
