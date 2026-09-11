#!/usr/bin/env python3
"""
visualize_interactive.py - Interactive Flight-Log Dashboard
visualize_interactive.py - 対話型フライトログダッシュボード

Generates a self-contained HTML dashboard (signal selection, overlay
support, configurable layout, no server required) from a StampFly
flight-log v1 bundle (`.sflog.zip` or an extracted directory -- see
docs/plans/flight-log-format-plan.md and protocol/spec/flight_log.yaml,
the format's Single Source of Truth). This is the backend of
`sf log viz -i`.

StampFly フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ --
形式の正本は docs/plans/flight-log-format-plan.md と
protocol/spec/flight_log.yaml）から、信号選択・重ね描き・レイアウト変更が
可能な自己完結型 HTML ダッシュボード（サーバー不要）を生成する。
`sf log viz -i` のバックエンド実装。

Usage:
    python visualize_interactive.py <bundle.sflog.zip> [-o out.html]
    sf log viz <bundle.sflog.zip> -i [options]
"""

import argparse
import json
import math
import sys
import tempfile
import urllib.request
import webbrowser
from pathlib import Path

import numpy as np

import sflog


# =============================================================================
# Plotly.js local cache management
# Plotly.js ローカルキャッシュ管理
# =============================================================================

PLOTLY_CDN_URL = 'https://cdn.plot.ly/plotly-2.35.2.min.js'
PLOTLY_CACHE_DIR = Path(__file__).parent / 'vendor'
PLOTLY_CACHE_FILE = PLOTLY_CACHE_DIR / 'plotly.min.js'


def get_plotly_js() -> str:
    """Load Plotly.js from local cache, downloading if needed.
    ローカルキャッシュから Plotly.js を読み込み、なければダウンロード。

    Returns the full JavaScript source code as a string.
    """
    # Try local cache first
    # まずローカルキャッシュを試す
    if PLOTLY_CACHE_FILE.exists():
        return PLOTLY_CACHE_FILE.read_text(encoding='utf-8')

    # Download and cache
    # ダウンロードしてキャッシュ
    print(f"Downloading Plotly.js from CDN (first time only)...")
    try:
        PLOTLY_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(PLOTLY_CDN_URL, str(PLOTLY_CACHE_FILE))
        print(f"Cached: {PLOTLY_CACHE_FILE} ({PLOTLY_CACHE_FILE.stat().st_size // 1024} KB)")
        return PLOTLY_CACHE_FILE.read_text(encoding='utf-8')
    except Exception as e:
        print(f"ERROR: Failed to download Plotly.js: {e}", file=sys.stderr)
        print(f"Please download manually:", file=sys.stderr)
        print(f"  curl -L {PLOTLY_CDN_URL} -o {PLOTLY_CACHE_FILE}", file=sys.stderr)
        sys.exit(1)


# =============================================================================
# Bundle -> flat signal dict (column name -> display signal key)
# 一式 -> 平坦な信号辞書（列名 -> 表示用信号キー）
#
# Unlisted columns of a listed stream, and every column of a stream with no
# entry here at all, default to f"{stream}_{column}" (see load_bundle()).
# 表に無い列（表に載っているストリームの中の未記載列も、表に無いストリーム
# の全列も）は f"{stream}_{column}" になる（load_bundle() 参照）。
# =============================================================================

STREAM_SIGNALS = {
    'imu': {
        'gyro_x': 'gyro_x', 'gyro_y': 'gyro_y', 'gyro_z': 'gyro_z',
        'accel_x': 'accel_x', 'accel_y': 'accel_y', 'accel_z': 'accel_z',
        'gyro_raw_x': 'gyro_raw_x', 'gyro_raw_y': 'gyro_raw_y', 'gyro_raw_z': 'gyro_raw_z',
        'accel_raw_x': 'accel_raw_x', 'accel_raw_y': 'accel_raw_y', 'accel_raw_z': 'accel_raw_z',
    },
    'attitude': {
        'quat_w': 'quat_w', 'quat_x': 'quat_x', 'quat_y': 'quat_y', 'quat_z': 'quat_z',
        'gyro_bias_x': 'gyro_bias_x', 'gyro_bias_y': 'gyro_bias_y', 'gyro_bias_z': 'gyro_bias_z',
        'accel_bias_x': 'accel_bias_x', 'accel_bias_y': 'accel_bias_y', 'accel_bias_z': 'accel_bias_z',
    },
    'posvel': {
        'pos_x': 'pos_x', 'pos_y': 'pos_y', 'pos_z': 'pos_z',
        'vel_x': 'vel_x', 'vel_y': 'vel_y', 'vel_z': 'vel_z',
    },
    'rate_ref': {
        'rate_ref_roll': 'rate_ref_roll',
        'rate_ref_pitch': 'rate_ref_pitch',
        'rate_ref_yaw': 'rate_ref_yaw',
    },
    'motor': {
        'duty_FR': 'duty_FR', 'duty_RR': 'duty_RR', 'duty_RL': 'duty_RL', 'duty_FL': 'duty_FL',
    },
    'ctrl_output': {
        # PRE-MIXER commanded thrust/torque -- the real controller output.
        # ミキサー手前のコントローラ指令 -- 実測のコントローラ出力そのもの。
        'thrust': 'ctrl_thrust',
        'torque_roll': 'torque_roll',
        'torque_pitch': 'torque_pitch',
        'torque_yaw': 'torque_yaw',
    },
    'pilot': {
        'throttle': 'pilot_throttle', 'roll': 'pilot_roll',
        'pitch': 'pilot_pitch', 'yaw': 'pilot_yaw',
    },
    'ctrl_ref': {
        'flight_mode': 'flight_mode',
        'angle_ref_roll': 'angle_ref_roll', 'angle_ref_pitch': 'angle_ref_pitch',
        'total_thrust': 'total_thrust',
        # 50 Hz duty echo from the CtrlRef packet -- prefer motor.csv
        # (duty_FR etc., 400 Hz measured) when it is present; kept
        # distinct here (ctrl_ref_ prefix) so the two never collide.
        # CtrlRef パケット由来の 50Hz duty エコー -- motor.csv（duty_FR 等、
        # 400Hz 実測）がある場合はそちらを優先する。両者が衝突しないよう
        # ここでは ctrl_ref_ 接頭辞を付けて区別する。
        'duty_FR': 'ctrl_ref_duty_FR', 'duty_RR': 'ctrl_ref_duty_RR',
        'duty_RL': 'ctrl_ref_duty_RL', 'duty_FL': 'ctrl_ref_duty_FL',
        'alt_setpoint': 'alt_setpoint', 'alt_vel_target': 'alt_vel_target',
        'climb_rate_cmd': 'climb_rate_cmd',
        'pos_setpoint_x': 'pos_setpoint_x', 'pos_setpoint_y': 'pos_setpoint_y',
    },
    'baro': {
        'altitude': 'baro_altitude', 'pressure': 'baro_pressure',
    },
    'tof_bottom': {
        'distance': 'tof_bottom', 'status': 'tof_bottom_status',
    },
    'tof_front': {
        'distance': 'tof_front', 'status': 'tof_front_status',
    },
    'flow': {
        'dx': 'flow_x', 'dy': 'flow_y', 'quality': 'flow_quality',
    },
    'mag': {
        'x': 'mag_x', 'y': 'mag_y', 'z': 'mag_z',
    },
    'status': {
        'voltage': 'battery_voltage', 'current_ma': 'battery_current_ma',
        'flight_state': 'flight_state', 'sensor_health': 'sensor_health',
        'eskf_status': 'eskf_status', 'reset_reason': 'reset_reason',
        'pid_roll_kp': 'pid_roll_kp', 'pid_roll_ti': 'pid_roll_ti', 'pid_roll_td': 'pid_roll_td',
        'pid_pitch_kp': 'pid_pitch_kp', 'pid_pitch_ti': 'pid_pitch_ti', 'pid_pitch_td': 'pid_pitch_td',
        'pid_yaw_kp': 'pid_yaw_kp', 'pid_yaw_ti': 'pid_yaw_ti', 'pid_yaw_td': 'pid_yaw_td',
    },
    'eskf_cov': {
        # Diagonal of the ESKF covariance matrix P -- already named p_* in
        # the bundle, kept as-is (no eskf_cov_ prefix).
        # ESKF 共分散行列 P の対角成分 -- 一式の時点で既に p_* と命名され
        # ているのでそのまま使う（eskf_cov_ 接頭辞を付けない）。
        'p_pos_x': 'p_pos_x', 'p_pos_y': 'p_pos_y', 'p_pos_z': 'p_pos_z',
        'p_vel_x': 'p_vel_x', 'p_vel_y': 'p_vel_y', 'p_vel_z': 'p_vel_z',
        'p_att_x': 'p_att_x', 'p_att_y': 'p_att_y', 'p_att_z': 'p_att_z',
        'p_bg_x': 'p_bg_x', 'p_bg_y': 'p_bg_y', 'p_bg_z': 'p_bg_z',
        'p_ba_x': 'p_ba_x', 'p_ba_y': 'p_ba_y', 'p_ba_z': 'p_ba_z',
    },
    'truth': {
        'pos_x': 'truth_pos_x', 'pos_y': 'truth_pos_y', 'pos_z': 'truth_pos_z',
        'quat_w': 'truth_quat_w', 'quat_x': 'truth_quat_x',
        'quat_y': 'truth_quat_y', 'quat_z': 'truth_quat_z',
        'vel_x': 'truth_vel_x', 'vel_y': 'truth_vel_y', 'vel_z': 'truth_vel_z',
        'rate_x': 'truth_rate_x', 'rate_y': 'truth_rate_y', 'rate_z': 'truth_rate_z',
    },
}


# =============================================================================
# Signal definitions with categories (sidebar grouping)
# 信号定義とカテゴリ（サイドバーのグルーピング）
# =============================================================================

SIGNAL_CATEGORIES = {
    'IMU - Gyro (LPF)': [
        ('gyro_x', 'Gyro X [rad/s]'),
        ('gyro_y', 'Gyro Y [rad/s]'),
        ('gyro_z', 'Gyro Z [rad/s]'),
    ],
    'IMU - Accel (LPF)': [
        ('accel_x', 'Accel X [m/s²]'),
        ('accel_y', 'Accel Y [m/s²]'),
        ('accel_z', 'Accel Z [m/s²]'),
    ],
    'IMU - Gyro (Raw)': [
        ('gyro_raw_x', 'Gyro Raw X [rad/s]'),
        ('gyro_raw_y', 'Gyro Raw Y [rad/s]'),
        ('gyro_raw_z', 'Gyro Raw Z [rad/s]'),
    ],
    'IMU - Accel (Raw)': [
        ('accel_raw_x', 'Accel Raw X [m/s²]'),
        ('accel_raw_y', 'Accel Raw Y [m/s²]'),
        ('accel_raw_z', 'Accel Raw Z [m/s²]'),
    ],
    'ESKF - Attitude': [
        ('attitude_roll_deg', 'Roll [deg]'),
        ('attitude_pitch_deg', 'Pitch [deg]'),
        ('attitude_yaw_deg', 'Yaw [deg]'),
    ],
    'ESKF - Position': [
        ('pos_x', 'Pos North [m]'),
        ('pos_y', 'Pos East [m]'),
        ('pos_z', 'Pos Down [m]'),
    ],
    'ESKF - Velocity': [
        ('vel_x', 'Vel North [m/s]'),
        ('vel_y', 'Vel East [m/s]'),
        ('vel_z', 'Vel Down [m/s]'),
    ],
    'ESKF - Gyro Bias': [
        ('gyro_bias_x', 'Gyro Bias X [rad/s]'),
        ('gyro_bias_y', 'Gyro Bias Y [rad/s]'),
        ('gyro_bias_z', 'Gyro Bias Z [rad/s]'),
    ],
    'ESKF - Accel Bias': [
        ('accel_bias_x', 'Accel Bias X [m/s²]'),
        ('accel_bias_y', 'Accel Bias Y [m/s²]'),
        ('accel_bias_z', 'Accel Bias Z [m/s²]'),
    ],
    'Control Output': [
        ('ctrl_thrust', 'Ctrl Thrust [N] (pre-mixer)'),
        ('torque_roll', 'Ctrl Torque Roll [N·m]'),
        ('torque_pitch', 'Ctrl Torque Pitch [N·m]'),
        ('torque_yaw', 'Ctrl Torque Yaw [N·m]'),
    ],
    'Control - Angle Reference': [
        ('angle_ref_roll_deg', 'Angle Ref Roll [deg]'),
        ('angle_ref_pitch_deg', 'Angle Ref Pitch [deg]'),
        ('flight_mode', 'Flight Mode'),
    ],
    'Control - Rate Reference': [
        ('rate_ref_roll', 'Rate Ref Roll [rad/s]'),
        ('rate_ref_pitch', 'Rate Ref Pitch [rad/s]'),
        ('rate_ref_yaw', 'Rate Ref Yaw [rad/s]'),
    ],
    'Motor Duty (400 Hz)': [
        ('duty_FR', 'FR(M1) Duty'),
        ('duty_RR', 'RR(M2) Duty'),
        ('duty_RL', 'RL(M3) Duty'),
        ('duty_FL', 'FL(M4) Duty'),
    ],
    'Control - Reference (50 Hz)': [
        ('ctrl_ref_duty_FR', 'FR(M1) Duty (ref, 50Hz)'),
        ('ctrl_ref_duty_RR', 'RR(M2) Duty (ref, 50Hz)'),
        ('ctrl_ref_duty_RL', 'RL(M3) Duty (ref, 50Hz)'),
        ('ctrl_ref_duty_FL', 'FL(M4) Duty (ref, 50Hz)'),
        ('total_thrust', 'Total Thrust [N]'),
        ('total_duty', 'Total Duty (mean of 4)'),
        ('alt_setpoint', 'Alt Setpoint [m]'),
        ('alt_vel_target', 'Alt Vel Target [m/s]'),
        ('climb_rate_cmd', 'Climb Rate Cmd [m/s]'),
        ('pos_setpoint_x', 'Pos Setpoint X [m]'),
        ('pos_setpoint_y', 'Pos Setpoint Y [m]'),
    ],
    'Pilot': [
        ('pilot_throttle', 'Pilot Throttle'),
        ('pilot_roll', 'Pilot Roll'),
        ('pilot_pitch', 'Pilot Pitch'),
        ('pilot_yaw', 'Pilot Yaw'),
    ],
    'Sensors - Height': [
        ('baro_altitude', 'Baro Alt [m]'),
        ('baro_pressure', 'Baro Press [Pa]'),
        ('tof_bottom', 'ToF Bottom [m]'),
        ('tof_front', 'ToF Front [m]'),
        ('tof_bottom_status', 'ToF Bot Status'),
        ('tof_front_status', 'ToF Frt Status'),
    ],
    'Sensors - Flow': [
        ('flow_x', 'Flow X [counts]'),
        ('flow_y', 'Flow Y [counts]'),
        ('flow_quality', 'Flow Quality'),
    ],
    'Sensors - Mag': [
        ('mag_x', 'Mag X [uT]'),
        ('mag_y', 'Mag Y [uT]'),
        ('mag_z', 'Mag Z [uT]'),
    ],
    'Battery': [
        ('battery_voltage', 'Voltage [V]'),
        ('battery_current_ma', 'Current [mA]'),
    ],
    'Status': [
        ('flight_state', 'Flight State'),
        ('eskf_status', 'ESKF Status'),
        ('sensor_health', 'Sensor Health'),
    ],
    'PID gains': [
        ('pid_roll_kp', 'Roll Kp'),
        ('pid_roll_ti', 'Roll Ti [s]'),
        ('pid_roll_td', 'Roll Td [s]'),
        ('pid_pitch_kp', 'Pitch Kp'),
        ('pid_pitch_ti', 'Pitch Ti [s]'),
        ('pid_pitch_td', 'Pitch Td [s]'),
        ('pid_yaw_kp', 'Yaw Kp'),
        ('pid_yaw_ti', 'Yaw Ti [s]'),
        ('pid_yaw_td', 'Yaw Td [s]'),
    ],
    'Timing - IMU Interval': [
        ('imu_interval_us', 'IMU Interval [μs] (should be ~2500)'),
    ],
    'Truth (SILS/sim)': [
        ('truth_pos_x', 'Truth Pos North [m]'),
        ('truth_pos_y', 'Truth Pos East [m]'),
        ('truth_pos_z', 'Truth Pos Down [m]'),
        ('truth_quat_w', 'Truth Quat W'),
        ('truth_quat_x', 'Truth Quat X'),
        ('truth_quat_y', 'Truth Quat Y'),
        ('truth_quat_z', 'Truth Quat Z'),
        ('truth_vel_x', 'Truth Vel North [m/s]'),
        ('truth_vel_y', 'Truth Vel East [m/s]'),
        ('truth_vel_z', 'Truth Vel Down [m/s]'),
        ('truth_rate_x', 'Truth Rate X [rad/s]'),
        ('truth_rate_y', 'Truth Rate Y [rad/s]'),
        ('truth_rate_z', 'Truth Rate Z [rad/s]'),
    ],
}

COLORS = [
    '#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
    '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990',
    '#dcbeff', '#9A6324', '#800000', '#aaffc3', '#808000',
    '#000075', '#a9a9a9',
]


def load_bundle(path) -> dict:
    """Load a StampFly flight-log v1 bundle and flatten it into the
    dict-of-lists `generate_html()` consumes.

    Every present stream gets its own time axis `_time_<stream>` (seconds,
    relative to the earliest first-sample timestamp across all present
    streams). `time_s` aliases `_time_imu` (falling back to the first
    available `_time_*` when the imu stream itself is absent -- it is
    `required: true` in the schema, so this only matters for a hand-built
    or truncated bundle). Every data column (except `timestamp_us` and
    `seq`) is exported under the signal key STREAM_SIGNALS declares for it,
    defaulting to f"{stream}_{column}" for anything not listed there.
    `_signal_axis` maps every signal key to its owning `_time_<stream>`
    key, so `generate_html()` never has to guess a signal's time axis from
    array length -- two streams can share a row count at the same nominal
    rate (pilot and ctrl_ref are both 50 Hz), which length-matching alone
    cannot tell apart.

    StampFly フライトログ v1 一式を読み込み、`generate_html()` が消費する
    信号名→リストの辞書へ平坦化する。

    実在する各ストリームは自分の時間軸 `_time_<stream>`（秒、全ストリーム
    の最初の標本のうち最も早い時刻を基準）を持つ。`time_s` は `_time_imu`
    の別名（imu ストリームが無い場合は最初に見つかった `_time_*` -- imu は
    スキーマ上 `required: true` なので、これは手組みや欠損した一式でしか
    起こらない）。各データ列（`timestamp_us` と `seq` を除く）は
    STREAM_SIGNALS が宣言する信号キーでエクスポートし、表に無ければ
    f"{stream}_{column}" を使う。`_signal_axis` は各信号キーを対応する
    `_time_<stream>` キーへ写像し、`generate_html()` が配列長から信号の
    時間軸を推測しなくて済むようにする -- pilot と ctrl_ref はどちらも
    50Hz で行数が同じになり得るため、長さ一致だけでは区別できない。
    """
    log = sflog.load(path)
    data: dict = {}
    signal_axis: dict = {}

    # t0 = earliest first-sample timestamp across all present streams.
    # t0 = 全ストリームの最初の標本のうち最も早い時刻。
    first_timestamps = [
        int(df['timestamp_us'].iloc[0])
        for df in log.streams.values()
        if len(df) > 0 and 'timestamp_us' in df.columns
    ]
    if not first_timestamps:
        return data
    t0 = min(first_timestamps)

    for stream_name, df in log.streams.items():
        if len(df) == 0 or 'timestamp_us' not in df.columns:
            continue
        time_key = f'_time_{stream_name}'
        data[time_key] = ((df['timestamp_us'] - t0) / 1e6).tolist()

        column_map = STREAM_SIGNALS.get(stream_name, {})
        for column in df.columns:
            if column in ('timestamp_us', 'seq'):
                continue
            signal_key = column_map.get(column, f'{stream_name}_{column}')
            data[signal_key] = df[column].tolist()
            signal_axis[signal_key] = time_key

    # time_s: prefer the IMU time axis (highest rate), fall back to the
    # first available stream's time axis.
    # time_s: IMU の時間軸を優先（最高レート）、無ければ最初に見つかった
    # ストリームの時間軸。
    if '_time_imu' in data:
        data['time_s'] = data['_time_imu']
    else:
        for key in data:
            if key.startswith('_time_'):
                data['time_s'] = data[key]
                break

    _add_derived_signals(data, signal_axis, log)
    data['_signal_axis'] = signal_axis
    return data


def _add_derived_signals(data: dict, signal_axis: dict, log: 'sflog.FlightLog') -> None:
    """Compute signals not directly present in any stream (attitude angles
    from the quaternion, reference angles in degrees, the 4-motor duty
    average, IMU sample-interval jitter), registering each one in
    `signal_axis` under the same time axis as its source stream/signal.

    どのストリームにも直接無い信号（クォータニオンからの姿勢角、度単位の
    目標角、モータ4基の duty 平均、IMU サンプル間隔のジッタ）を計算し、
    それぞれを元ストリーム/信号と同じ時間軸で `signal_axis` に登録する。
    """
    # Attitude angles from the quaternion (roll/pitch/yaw), same formulas
    # the legacy CSV/JSONL loaders used.
    # クォータニオンからの姿勢角（ロール・ピッチ・ヨー）。旧CSV/JSONL
    # 読み込み処理と同じ式。
    if all(k in data for k in ('quat_w', 'quat_x', 'quat_y', 'quat_z')):
        w, x, y, z = data['quat_w'], data['quat_x'], data['quat_y'], data['quat_z']
        n = len(w)
        roll = [0.0] * n
        pitch = [0.0] * n
        yaw = [0.0] * n
        for i in range(n):
            roll[i] = math.degrees(math.atan2(
                2 * (w[i] * x[i] + y[i] * z[i]), 1 - 2 * (x[i] * x[i] + y[i] * y[i])))
            pitch[i] = math.degrees(math.asin(
                max(-1.0, min(1.0, 2 * (w[i] * y[i] - z[i] * x[i])))))
            yaw[i] = math.degrees(math.atan2(
                2 * (w[i] * z[i] + x[i] * y[i]), 1 - 2 * (y[i] * y[i] + z[i] * z[i])))
        data['attitude_roll_deg'], data['attitude_pitch_deg'], data['attitude_yaw_deg'] = roll, pitch, yaw
        for key in ('attitude_roll_deg', 'attitude_pitch_deg', 'attitude_yaw_deg'):
            signal_axis[key] = signal_axis['quat_w']

    # Attitude-reference angles in degrees (rad -> deg), for display
    # alongside attitude_roll_deg/attitude_pitch_deg.
    # 姿勢目標角を度に変換（rad -> deg）。attitude_roll_deg/attitude_pitch_deg と並べて
    # 表示するため。
    if 'angle_ref_roll' in data:
        data['angle_ref_roll_deg'] = [math.degrees(v) for v in data['angle_ref_roll']]
        data['angle_ref_pitch_deg'] = [math.degrees(v) for v in data['angle_ref_pitch']]
        signal_axis['angle_ref_roll_deg'] = signal_axis['angle_ref_roll']
        signal_axis['angle_ref_pitch_deg'] = signal_axis['angle_ref_pitch']

    # total_duty: mean of the 4 motor duties. Prefer the real 400 Hz
    # motor.csv values; fall back to the 50 Hz ctrl_ref duty echo when
    # motor.csv is absent (older captures, or a bundle that never records
    # it).
    # total_duty: モータ4基の duty 平均。実測 400Hz の motor.csv を優先し、
    # 無ければ 50Hz の ctrl_ref 側の duty で代用する（motor.csv が無い旧
    # 取得や、そもそも記録しない一式向け）。
    for keys in (
        ('duty_FR', 'duty_RR', 'duty_RL', 'duty_FL'),
        ('ctrl_ref_duty_FR', 'ctrl_ref_duty_RR', 'ctrl_ref_duty_RL', 'ctrl_ref_duty_FL'),
    ):
        if all(k in data for k in keys):
            n = len(data[keys[0]])
            data['total_duty'] = [sum(data[k][i] for k in keys) / 4.0 for i in range(n)]
            signal_axis['total_duty'] = signal_axis[keys[0]]
            break

    # imu_interval_us: consecutive-sample spacing of the IMU timestamp
    # (microseconds), first value repeated -- shows the firmware's
    # reused-IMU-sample cycles (plan section 7).
    # imu_interval_us: IMU タイムスタンプの連続標本間隔（マイクロ秒）、
    # 先頭値は複製。ファームが IMU 標本を使い回した周期を可視化する
    # （計画書7節）。
    imu_df = log.streams.get('imu')
    if imu_df is not None and len(imu_df) > 1:
        ts = imu_df['timestamp_us'].to_numpy()
        interval = np.diff(ts).astype(float)
        data['imu_interval_us'] = np.concatenate([interval[:1], interval]).tolist()
        signal_axis['imu_interval_us'] = '_time_imu'


def generate_html(data: dict, title: str, plotly_js: str = '') -> str:
    """Generate self-contained HTML dashboard.
    自己完結型 HTML ダッシュボードを生成する。
    """

    # Filter categories to only include signals present in data
    # 実在する信号だけが残るようカテゴリを絞り込む
    categories = {}
    for cat, signals in SIGNAL_CATEGORIES.items():
        available = [(key, label) for key, label in signals if key in data]
        if available:
            categories[cat] = available

    # Serialize data to JSON (only signals that exist + time axes)
    # データを JSON 化（実在する信号 + 時間軸のみ）
    all_keys = set()
    for sigs in categories.values():
        for key, _ in sigs:
            all_keys.add(key)
    all_keys.add('time_s')

    # Add internal time axes (never bookkeeping keys like `_signal_axis` --
    # only keys that ARE a time axis).
    # 内部時間軸を追加（`_signal_axis` のような管理用キーは含めない --
    # 時間軸そのものであるキーのみ）。
    for k in data:
        if k.startswith('_time_'):
            all_keys.add(k)

    export_data = {k: data[k] for k in all_keys if k in data}

    # Build signal-to-time-axis mapping. Prefer the explicit map
    # `load_bundle()` computed (`_signal_axis`) -- multiple streams can
    # share a row count at the same nominal rate (pilot and ctrl_ref are
    # both 50 Hz), so length-matching alone would put one stream's signals
    # on another stream's time axis. Fall back to length-matching only for
    # a signal missing from the explicit map (e.g. a hand-built `data`
    # dict, as in the unit tests).
    # 信号→時間軸のマッピングを構築する。`load_bundle()` が計算した明示
    # マップ（`_signal_axis`）を優先する -- 複数ストリームが同じ公称レート
    # で同じ行数を持ち得るため（pilot と ctrl_ref はどちらも50Hz）、長さ
    # 一致だけでは別ストリームの時間軸に誤って対応付いてしまう。明示マップ
    # に無い信号（単体テストの手組み辞書等）のときだけ長さ一致に
    # フォールバックする。
    signal_axis_map = data.get('_signal_axis', {})
    signal_time_map = {}

    sensor_time_axes = {}  # {'imu': ('_time_imu', 3928), ...}
    for k, v in data.items():
        if k.startswith('_time_'):
            sensor_name = k[6:]  # '_time_imu' → 'imu'
            sensor_time_axes[sensor_name] = (k, len(v))

    for sig, sig_data in data.items():
        # Skip time axes, the plain time_s alias, and any other internal
        # bookkeeping key (starts with '_', e.g. `_signal_axis`) -- never a
        # plottable signal.
        # 時間軸・素の time_s・その他の内部管理用キー（'_' で始まる、例:
        # `_signal_axis`）はスキップする -- プロット対象の信号ではない。
        if sig == 'time_s' or sig.startswith('_'):
            continue

        if sig in signal_axis_map:
            signal_time_map[sig] = {'time': signal_axis_map[sig], 'data': sig}
            continue

        sig_len = len(sig_data)

        # Find a _time_* axis with matching length
        # 長さが一致する _time_* 軸を探す
        matched = False
        for sensor_name, (time_key, time_len) in sensor_time_axes.items():
            if sig_len == time_len:
                signal_time_map[sig] = {'time': time_key, 'data': sig}
                matched = True
                break

        # Fallback: a hand-built dedup key, kept for callers that still
        # construct one by hand (the v1 bundle loader never produces one).
        # フォールバック: dedup キー。手組みで用意する呼び出し元向けに残す
        # （v1 バンドル読み込みでは生成しない）。
        if not matched:
            dedup_key = f'{sig}_dedup'
            if dedup_key in data:
                for sensor_name, (time_key, time_len) in sensor_time_axes.items():
                    if len(data[dedup_key]) == time_len:
                        signal_time_map[sig] = {'time': time_key, 'data': dedup_key}
                        break

    data_json = json.dumps(export_data, separators=(',', ':'))
    categories_json = json.dumps(categories, ensure_ascii=False)
    colors_json = json.dumps(COLORS)
    signal_time_map_json = json.dumps(signal_time_map)

    n_samples = len(data.get('time_s', []))
    duration = data['time_s'][-1] if 'time_s' in data and n_samples > 0 else 0
    rate = n_samples / duration if duration > 0 else 0

    # Round up duration to nearest integer for clean X-axis range
    # X軸範囲を整数秒に切り上げ
    duration_ceil = math.ceil(duration) if duration > 0 else 10

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>{title}</title>
<script>{plotly_js}</script>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; display: flex; height: 100vh; background: #f5f5f5; }}

/* Sidebar */
#sidebar {{
    width: 260px; min-width: 260px; background: #fff; border-right: 1px solid #ddd;
    overflow-y: auto; padding: 12px; font-size: 13px;
}}
#sidebar h2 {{ font-size: 15px; margin-bottom: 8px; color: #333; }}
.info {{ font-size: 11px; color: #888; margin-bottom: 12px; }}
.cat-title {{
    font-weight: 600; font-size: 12px; color: #555; margin: 10px 0 4px 0;
    cursor: pointer; user-select: none;
}}
.cat-title:hover {{ color: #000; }}
.signal-item {{
    display: flex; align-items: center; padding: 2px 0 2px 12px;
    cursor: pointer; border-radius: 3px;
}}
.signal-item:hover {{ background: #f0f0f0; }}
.signal-item input {{ margin-right: 6px; cursor: pointer; }}
.signal-item label {{ cursor: pointer; font-size: 12px; white-space: nowrap; }}

/* Plot controls */
#controls {{
    padding: 10px 0; border-top: 1px solid #eee; margin-top: 10px;
}}
#controls label {{ font-size: 12px; color: #555; }}
#controls select, #controls input {{ font-size: 12px; margin: 2px 0; }}
button {{
    padding: 6px 12px; font-size: 12px; cursor: pointer; border: 1px solid #ccc;
    border-radius: 4px; background: #fff; margin: 2px;
}}
button:hover {{ background: #e8e8e8; }}
button.primary {{ background: #4363d8; color: #fff; border-color: #4363d8; }}
button.primary:hover {{ background: #3252b5; }}
button.danger {{ color: #e6194b; border-color: #e6194b; }}

/* Main area */
#main {{ flex: 1; display: flex; flex-direction: column; overflow: hidden; }}
#toolbar {{
    background: #fff; border-bottom: 1px solid #ddd; padding: 8px 16px;
    display: flex; align-items: center; gap: 12px; font-size: 13px;
}}
#plot-area {{ flex: 1; overflow-y: auto; padding: 8px; }}
.plot-container {{
    background: #fff; border: 1px solid #ddd; border-radius: 6px;
    margin-bottom: 8px; position: relative;
}}
.plot-header {{
    display: flex; justify-content: space-between; align-items: center;
    padding: 4px 10px; border-bottom: 1px solid #eee; font-size: 12px;
    color: #555; background: #fafafa; border-radius: 6px 6px 0 0;
    cursor: pointer;
}}
.plot-header span {{ font-weight: 600; }}
.plot-container.active {{ border: 2px solid #4363d8; }}
.plot-container.active .plot-header {{ background: #e8ecf8; }}
.plot-div {{ width: 100%; }}
</style>
</head>
<body>

<div id="sidebar">
    <h2>StampFly Telemetry</h2>
    <div class="info">{n_samples} samples, {duration:.1f}s, {rate:.0f} Hz</div>

    <div id="signal-list"></div>

    <div id="controls">
        <button class="primary" onclick="addPlot()">+ Add Plot</button>
        <button onclick="addPreset('imu')">IMU</button>
        <button onclick="addPreset('eskf')">ESKF</button>
        <button onclick="addPreset('bias')">Bias</button>
        <button onclick="addPreset('flight')">Flight</button>
        <button onclick="addPreset('sensors')">Sensors</button>
        <button onclick="clearAll()">Clear All</button>
        <div style="margin-top:6px">
            <label><input type="checkbox" id="sync-x" checked> Sync time axis</label>
        </div>
        <div style="margin-top:4px">
            <label><input type="checkbox" id="hide-invalid" checked onchange="toggleInvalid()"> Hide invalid data</label>
        </div>
        <div style="margin-top:6px">
            <label>Draw mode:</label>
            <select id="draw-mode" style="width:100%" onchange="applyDrawMode()">
                <option value="lines" selected>Lines</option>
                <option value="markers">Points</option>
                <option value="lines+markers">Lines + Points</option>
            </select>
        </div>
        <div style="margin-top:8px">
            <label>Target plot:</label>
            <select id="target-plot" style="width:100%" onchange="selectPlot(this.value)"></select>
        </div>
    </div>
</div>

<div id="main">
    <div id="toolbar">
        <span style="font-weight:600">{title}</span>
        <span style="color:#888; font-size:12px">
            Check signals → click "Add to Plot" or double-click signal.
            Zoom: drag | Pan: shift+drag | Reset: double-click plot
        </span>
    </div>
    <div id="plot-area"></div>
</div>

<script>
const DATA = {data_json};
const CATEGORIES = {categories_json};
const SIGNAL_TIME_MAP = {signal_time_map_json};
const DURATION = {duration_ceil};
const COLORS = {colors_json};

let plots = [];  // [{{ id, div, traces: [{{key, label}}] }}]
let plotCounter = 0;

// Invalid data masking rules: signal → {{statusKey, validFn}}
// 無効データマスクルール: 信号名 → {{状態キー, 有効判定関数}}
const INVALID_RULES = {{
    'flow_x':     {{ status: 'flow_quality', valid: v => v > 0 }},
    'flow_y':     {{ status: 'flow_quality', valid: v => v > 0 }},
    'tof_bottom':  {{ status: 'tof_bottom_status', valid: v => v === 0 }},
    'tof_front':   {{ status: 'tof_front_status', valid: v => v === 0 }},
}};

function applyMask(yData, key) {{
    const rule = INVALID_RULES[key];
    if (!rule || !DATA[rule.status]) return yData;

    const hideInvalid = document.getElementById('hide-invalid').checked;
    if (!hideInvalid) return yData;

    const statusArr = DATA[rule.status];
    return yData.map((v, i) => {{
        const si = i < statusArr.length ? statusArr[i] : 0;
        return rule.valid(si) ? v : null;  // null = gap in Plotly
    }});
}}
let colorIndex = 0;
let _syncBusy = false;  // Guard against recursive relayout sync

function syncXRange(sourceId, eventData) {{
    if (_syncBusy) return;
    if (!document.getElementById('sync-x').checked) return;

    // Only respond to explicit x-axis range changes (zoom/pan/reset)
    // Ignore other relayout events (e.g., hover spike lines, resize)
    // 明示的な X 軸範囲変更（ズーム・パン・リセット）のみ応答
    let xr = null;
    let autorange = false;
    if ('xaxis.range[0]' in eventData && 'xaxis.range[1]' in eventData) {{
        xr = [eventData['xaxis.range[0]'], eventData['xaxis.range[1]']];
    }} else if ('xaxis.autorange' in eventData) {{
        autorange = true;
    }} else {{
        return;  // Not a user-initiated x-axis change
    }}

    _syncBusy = true;
    plots.forEach(p => {{
        if (p.id === sourceId) return;
        if (autorange) {{
            Plotly.relayout(p.id, {{'xaxis.autorange': true}});
        }} else {{
            Plotly.relayout(p.id, {{'xaxis.range': xr}});
        }}
    }});
    // Keep guard up until next event loop tick to block cascading events
    // 連鎖イベントをブロックするため次のイベントループまでガードを維持
    setTimeout(() => {{ _syncBusy = false; }}, 0);
}}

function nextColor() {{
    const c = COLORS[colorIndex % COLORS.length];
    colorIndex++;
    return c;
}}

// Build signal list sidebar
function buildSidebar() {{
    const container = document.getElementById('signal-list');
    let html = '';
    for (const [cat, signals] of Object.entries(CATEGORIES)) {{
        html += `<div class="cat-title" onclick="toggleCat(this)">▸ ${{cat}}</div>`;
        html += `<div class="cat-signals" style="display:none">`;
        for (const [key, label] of signals) {{
            html += `<div class="signal-item" ondblclick="quickAdd('${{key}}','${{label}}')">`;
            html += `<input type="checkbox" id="cb_${{key}}" value="${{key}}" data-label="${{label}}">`;
            html += `<label for="cb_${{key}}">${{label}}</label>`;
            html += `</div>`;
        }}
        html += `<div style="padding:2px 12px"><button onclick="addCheckedFromCat(this)" style="font-size:11px">Add checked to plot</button></div>`;
        html += `</div>`;
    }}
    container.innerHTML = html;
}}

function toggleCat(el) {{
    const sigs = el.nextElementSibling;
    if (sigs.style.display === 'none') {{
        sigs.style.display = 'block';
        el.textContent = el.textContent.replace('▸', '▾');
    }} else {{
        sigs.style.display = 'none';
        el.textContent = el.textContent.replace('▾', '▸');
    }}
}}

function addCheckedFromCat(btn) {{
    const catDiv = btn.parentElement.parentElement;
    const checkboxes = catDiv.querySelectorAll('input[type=checkbox]:checked');
    if (checkboxes.length === 0) return;

    let plot = getTargetPlot();
    if (!plot) plot = addPlot();

    checkboxes.forEach(cb => {{
        addSignalToPlot(plot, cb.value, cb.dataset.label);
        cb.checked = false;
    }});
}}

function quickAdd(key, label) {{
    let plot = getTargetPlot();
    if (!plot) plot = addPlot();
    addSignalToPlot(plot, key, label);
}}

function getTargetPlot() {{
    const sel = document.getElementById('target-plot');
    if (!sel.value) return null;
    return plots.find(p => p.id === sel.value);
}}

function updateTargetSelect() {{
    const sel = document.getElementById('target-plot');
    const current = sel.value;
    sel.innerHTML = '';
    plots.forEach(p => {{
        const opt = document.createElement('option');
        opt.value = p.id;
        opt.textContent = p.title;
        sel.appendChild(opt);
    }});
    if (current && plots.find(p => p.id === current)) {{
        sel.value = current;
    }} else if (plots.length > 0) {{
        sel.value = plots[plots.length - 1].id;
    }}
}}

function toggleInvalid() {{
    // Rebuild all plots with updated mask state
    // マスク状態を更新して全プロットを再構築
    const savedPlots = plots.map(p => ({{ id: p.id, traces: [...p.traces] }}));
    // Remove all traces and re-add with new mask
    savedPlots.forEach(sp => {{
        const plot = plots.find(p => p.id === sp.id);
        if (!plot) return;
        // Delete all traces
        const nTraces = plot.traces.length;
        if (nTraces > 0) {{
            Plotly.deleteTraces(plot.id, Array.from({{length: nTraces}}, (_, i) => 0));
        }}
        plot.traces = [];
        // Re-add with updated mask
        sp.traces.forEach(t => addSignalToPlot(plot, t.key, t.label));
    }});
}}

function applyDrawMode() {{
    const mode = document.getElementById('draw-mode').value;
    plots.forEach(p => {{
        const nTraces = p.traces.length;
        if (nTraces === 0) return;
        // Save current X range before restyle
        const plotDiv = document.getElementById(p.id);
        const layout = plotDiv._fullLayout;
        const xRange = layout && layout.xaxis ? [layout.xaxis.range[0], layout.xaxis.range[1]] : null;
        const update = {{ mode: Array(nTraces).fill(mode) }};
        Plotly.restyle(p.id, update);
        // Restore X range after restyle
        if (xRange) {{
            Plotly.relayout(p.id, {{'xaxis.range': xRange}});
        }}
    }});
}}

function selectPlot(id) {{
    // Set target plot and highlight
    // ターゲットプロットを設定してハイライト
    const sel = document.getElementById('target-plot');
    sel.value = id;
    // Update visual highlight
    document.querySelectorAll('.plot-container').forEach(el => el.classList.remove('active'));
    const container = document.getElementById(`container_${{id}}`);
    if (container) container.classList.add('active');
}}

function addPlot() {{
    plotCounter++;
    const id = `plot_${{plotCounter}}`;
    const title = `Plot ${{plotCounter}}`;

    const area = document.getElementById('plot-area');
    const container = document.createElement('div');
    container.className = 'plot-container';
    container.id = `container_${{id}}`;
    container.innerHTML = `
        <div class="plot-header" onclick="selectPlot('${{id}}')">
            <span style="color:#4363d8;font-size:11px;margin-right:6px">[${{plotCounter}}]</span>
            <span id="title_${{id}}">${{title}}</span>
            <span id="cursor_${{id}}" style="font-family:monospace;font-size:11px;color:#888;margin-left:auto;margin-right:12px"></span>
            <button class="danger" onclick="event.stopPropagation();removePlot('${{id}}')" style="font-size:11px;padding:2px 8px">✕ Remove</button>
        </div>
        <div id="${{id}}" class="plot-div"></div>
    `;
    area.appendChild(container);

    // Click anywhere on plot area to select as target
    // プロットエリアのクリックでターゲットに選択
    container.addEventListener('click', function(e) {{
        // Don't select if clicking remove button
        if (e.target.tagName === 'BUTTON') return;
        selectPlot(id);
    }});

    const layout = {{
        height: 280,
        margin: {{ l: 60, r: 20, t: 10, b: 40 }},
        xaxis: {{
            title: 'Time [s]',
            range: [0, DURATION],
            showspikes: true,
            spikemode: 'across',
            spikesnap: 'cursor',
            spikethickness: 1,
            spikecolor: '#999',
            spikedash: 'dot',
        }},
        yaxis: {{
            title: '',
            showspikes: true,
            spikemode: 'across',
            spikesnap: 'cursor',
            spikethickness: 1,
            spikecolor: '#999',
            spikedash: 'dot',
        }},
        hovermode: 'x unified',
        template: 'plotly_white',
        legend: {{ orientation: 'h', y: 1.12 }},
    }};

    Plotly.newPlot(id, [], layout, {{
        responsive: true,
        displayModeBar: true,
        modeBarButtonsToRemove: ['lasso2d', 'select2d'],
    }});

    // Show cursor position (t, y_axis) in plot header using mouse events
    // マウスイベントでカーソルの軸上の座標 (t, y) をヘッダーに表示
    // plotly_hover returns data values, not cursor position on axis.
    // Use mousemove + Plotly axis mapping to get the actual cursor y-coordinate.
    const plotDiv = document.getElementById(id);

    // Sync X axis on zoom/pan
    // ズーム・パン時に X 軸を同期
    plotDiv.on('plotly_relayout', function(ed) {{ syncXRange(id, ed); }});

    plotDiv.addEventListener('mousemove', function(evt) {{
        const cursorEl = document.getElementById(`cursor_${{id}}`);
        if (!cursorEl) return;
        const bb = plotDiv.getBoundingClientRect();
        const layout = plotDiv._fullLayout;
        if (!layout || !layout.xaxis || !layout.yaxis) return;
        const xa = layout.xaxis;
        const ya = layout.yaxis;
        // Mouse position relative to plot area
        const mouseX = evt.clientX - bb.left;
        const mouseY = evt.clientY - bb.top;
        // Check if inside plot area
        if (mouseX < xa._offset || mouseX > xa._offset + xa._length ||
            mouseY < ya._offset || mouseY > ya._offset + ya._length) {{
            cursorEl.textContent = '';
            return;
        }}
        // Convert pixel to data coordinates
        const tVal = xa.p2d(mouseX - xa._offset);
        const yVal = ya.p2d(mouseY - ya._offset);
        cursorEl.textContent = `t=${{tVal.toFixed(3)}}s  y=${{yVal.toFixed(5)}}`;
    }});
    plotDiv.addEventListener('mouseleave', function() {{
        const cursorEl = document.getElementById(`cursor_${{id}}`);
        if (cursorEl) cursorEl.textContent = '';
    }});

    const plot = {{ id, title, div: plotDiv, traces: [] }};
    plots.push(plot);
    updateTargetSelect();
    selectPlot(id);  // Auto-select new plot as target
    return plot;
}}

function removePlot(id) {{
    const idx = plots.findIndex(p => p.id === id);
    if (idx === -1) return;
    plots.splice(idx, 1);
    const container = document.getElementById(`container_${{id}}`);
    if (container) container.remove();
    updateTargetSelect();
}}

function addSignalToPlot(plot, key, label) {{
    if (!DATA[key] || !DATA.time_s) return;
    // Check if already added
    if (plot.traces.find(t => t.key === key)) return;

    // Use sensor-specific time axis and deduped data if available
    // 利用可能なら各センサー固有の時間軸と重複除去済みデータを使用
    let xData = DATA.time_s;
    let yData = DATA[key];
    const mapping = SIGNAL_TIME_MAP[key];
    if (mapping && DATA[mapping.time] && DATA[mapping.data]) {{
        xData = DATA[mapping.time];
        yData = DATA[mapping.data];
    }}

    // Apply invalid data mask if applicable
    // 無効データマスクを適用（該当する場合）
    yData = applyMask(yData, key);

    const drawMode = document.getElementById('draw-mode').value;
    const color = nextColor();
    const trace = {{
        x: xData,
        y: yData,
        name: label,
        type: 'scattergl',
        mode: drawMode,
        line: {{ color: color, width: 1 }},
        marker: {{ color: color, size: 3 }},
        hovertemplate: `${{label}}: %{{y:.5f}}<extra></extra>`,
    }};

    Plotly.addTraces(plot.id, trace);
    plot.traces.push({{ key, label }});

    // Update title
    const titleEl = document.getElementById(`title_${{plot.id}}`);
    titleEl.textContent = plot.traces.map(t => t.label).join(', ');
}}

function addPreset(name) {{
    if (name === 'imu') {{
        const p1 = addPlot();
        ['gyro_x', 'gyro_y', 'gyro_z'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['accel_x', 'accel_y', 'accel_z'].forEach(k => addSignalToPlot(p2, k, k));
    }} else if (name === 'eskf') {{
        const p1 = addPlot();
        ['attitude_roll_deg', 'attitude_pitch_deg', 'attitude_yaw_deg'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['pos_x', 'pos_y', 'pos_z'].forEach(k => addSignalToPlot(p2, k, k));
        const p3 = addPlot();
        ['vel_x', 'vel_y', 'vel_z'].forEach(k => addSignalToPlot(p3, k, k));
    }} else if (name === 'bias') {{
        const p1 = addPlot();
        ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['accel_bias_x', 'accel_bias_y', 'accel_bias_z'].forEach(k => addSignalToPlot(p2, k, k));
    }} else if (name === 'flight') {{
        const p1 = addPlot();
        ['attitude_roll_deg', 'attitude_pitch_deg'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['pilot_throttle'].forEach(k => addSignalToPlot(p2, k, k));
        const p3 = addPlot();
        ['tof_bottom'].forEach(k => addSignalToPlot(p3, k, k));
        const p4 = addPlot();
        ['gyro_x', 'gyro_y', 'gyro_z'].forEach(k => addSignalToPlot(p4, k, k));
        const p5 = addPlot();
        ['accel_x', 'accel_y', 'accel_z'].forEach(k => addSignalToPlot(p5, k, k));
    }} else if (name === 'sensors') {{
        const p1 = addPlot();
        ['tof_bottom', 'baro_altitude'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['flow_x', 'flow_y'].forEach(k => addSignalToPlot(p2, k, k));
        const p3 = addPlot();
        ['mag_x', 'mag_y', 'mag_z'].forEach(k => addSignalToPlot(p3, k, k));
    }}
}}

function clearAll() {{
    plots.forEach(p => {{
        const container = document.getElementById(`container_${{p.id}}`);
        if (container) container.remove();
    }});
    plots = [];
    colorIndex = 0;
    updateTargetSelect();
}}

// Initialize
buildSidebar();
</script>
</body>
</html>"""

    return html


def visualize(filepath: str, output=None, title=None):
    """Load a StampFly flight-log v1 bundle and generate/open the dashboard.
    StampFly フライトログ v1 一式を読み込み、ダッシュボードを生成・表示する。
    """
    print(f"Loading: {filepath}")
    data = load_bundle(filepath)

    n = len(data.get('time_s', []))
    dur = data['time_s'][-1] if 'time_s' in data and n > 0 else 0
    rate = n / dur if dur > 0 else 0
    print(f"Samples: {n}, Duration: {dur:.1f}s, Rate: {rate:.0f} Hz")

    if title is None:
        title = f"StampFly — {Path(filepath).name}"

    plotly_js = get_plotly_js()
    html = generate_html(data, title, plotly_js)

    if output:
        with open(output, 'w') as f:
            f.write(html)
        print(f"Saved: {output}")
    else:
        tmp = tempfile.NamedTemporaryFile(suffix='.html', delete=False, prefix='stampfly_viz_')
        tmp.write(html.encode('utf-8'))
        tmp.close()
        print(f"Opening in browser: {tmp.name}")
        webbrowser.open(f'file://{tmp.name}')


def main():
    parser = argparse.ArgumentParser(
        description="Interactive flight-log dashboard (StampFly v1 bundle)")
    parser.add_argument('file', help="Flight-log bundle (.sflog.zip or an extracted directory)")
    parser.add_argument('-o', '--output', help="Save HTML to file")
    args = parser.parse_args()
    visualize(args.file, output=args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
