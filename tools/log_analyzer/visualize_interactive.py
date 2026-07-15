#!/usr/bin/env python3
"""
visualize_interactive.py - Interactive Telemetry Dashboard

Generates a self-contained HTML dashboard with signal selection,
overlay support, and configurable layout. No server required.

ブラウザ上で信号選択・重ね描き・レイアウト変更が可能な
自己完結型 HTML ダッシュボードを生成。サーバー不要。

Usage:
    python visualize_interactive.py <csv_file> [options]
    sf log viz <csv_file> -i [options]

Features:
    - Select signals from sidebar checkboxes
    - Overlay multiple signals on the same plot
    - Add/remove subplots dynamically
    - Zoom, pan, hover on all axes
    - Configurable grid layout
"""

import argparse
import csv
import json
import math
import os
import sys
import tempfile
import urllib.request
import webbrowser
from pathlib import Path

import numpy as np


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
# Signal definitions with categories
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
    'IMU - Corrected Gyro': [
        ('gyro_corrected_x', 'Gyro Corr X [rad/s]'),
        ('gyro_corrected_y', 'Gyro Corr Y [rad/s]'),
        ('gyro_corrected_z', 'Gyro Corr Z [rad/s]'),
    ],
    'IMU - Corrected Accel': [
        ('accel_corrected_x', 'Accel Corr X [m/s²]'),
        ('accel_corrected_y', 'Accel Corr Y [m/s²]'),
        ('accel_corrected_z', 'Accel Corr Z [m/s²]'),
    ],
    'ESKF - Attitude': [
        ('roll_deg', 'Roll [deg]'),
        ('pitch_deg', 'Pitch [deg]'),
        ('yaw_deg', 'Yaw [deg]'),
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
    'Control': [
        ('ctrl_throttle', 'Throttle'),
        ('ctrl_roll', 'Roll Cmd'),
        ('ctrl_pitch', 'Pitch Cmd'),
        ('ctrl_yaw', 'Yaw Cmd'),
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
    'Computed - PID Output': [
        ('pid_out_roll', 'PID Roll [Nm]'),
        ('pid_out_pitch', 'PID Pitch [Nm]'),
        ('pid_out_yaw', 'PID Yaw [Nm]'),
    ],
    'Computed - Motor Thrust': [
        ('motor_thrust_FR', 'FR(M1) Thrust [N]'),
        ('motor_thrust_RR', 'RR(M2) Thrust [N]'),
        ('motor_thrust_RL', 'RL(M3) Thrust [N]'),
        ('motor_thrust_FL', 'FL(M4) Thrust [N]'),
    ],
    'Computed - Motor Duty': [
        ('motor_duty_FR', 'FR(M1) Duty'),
        ('motor_duty_RR', 'RR(M2) Duty'),
        ('motor_duty_RL', 'RL(M3) Duty'),
        ('motor_duty_FL', 'FL(M4) Duty'),
        ('motor_saturated', 'Saturated (any)'),
    ],
    'Actual Motor Duty': [
        ('actual_duty_FR', 'FR(M1) Actual Duty'),
        ('actual_duty_RR', 'RR(M2) Actual Duty'),
        ('actual_duty_RL', 'RL(M3) Actual Duty'),
        ('actual_duty_FL', 'FL(M4) Actual Duty'),
    ],
    'Thrust Command': [
        ('total_thrust', 'Total Thrust [N]'),
        ('total_duty', 'Total Duty (thrust/max)'),
    ],
    'Sensors - Height': [
        ('baro_altitude', 'Baro Alt [m]'),
        ('baro_pressure', 'Baro Press [hPa]'),
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
    ],
    'Timing - Internal Timestamps': [
        ('imu_timestamp_us', 'IMU Timestamp [μs]'),
        ('baro_timestamp_us', 'Baro Timestamp [μs]'),
        ('tof_timestamp_us', 'ToF Timestamp [μs]'),
        ('mag_timestamp_us', 'Mag Timestamp [μs]'),
        ('flow_timestamp_us', 'Flow Timestamp [μs]'),
    ],
    'Timing - IMU Interval': [
        ('imu_interval_us', 'IMU Interval [μs] (should be ~2500)'),
        ('telemetry_interval_us', 'Telemetry Interval [μs]'),
    ],
    'Timing - Telemetry Delay': [
        ('telemetry_delay_us', 'Telemetry Delay [μs] (WiFi pipeline)'),
    ],
    'Computed - Gyro Int (raw)': [
        ('gyro_int_roll', 'Gyro Int Roll [deg]'),
        ('gyro_int_pitch', 'Gyro Int Pitch [deg]'),
        ('gyro_int_yaw', 'Gyro Int Yaw [deg]'),
    ],
    'Computed - Gyro Int (corrected)': [
        ('gyro_corr_int_roll', 'Gyro Corr Int Roll [deg]'),
        ('gyro_corr_int_pitch', 'Gyro Corr Int Pitch [deg]'),
        ('gyro_corr_int_yaw', 'Gyro Corr Int Yaw [deg]'),
    ],
    'Computed - Accel Att (raw)': [
        ('accel_roll', 'Accel Roll [deg]'),
        ('accel_pitch', 'Accel Pitch [deg]'),
    ],
    'Computed - Accel Att (corrected)': [
        ('accel_corr_roll', 'Accel Corr Roll [deg]'),
        ('accel_corr_pitch', 'Accel Corr Pitch [deg]'),
    ],
    'Computed - Mag Heading': [
        ('mag_yaw', 'Mag Yaw [deg]'),
    ],
}

COLORS = [
    '#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
    '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990',
    '#dcbeff', '#9A6324', '#800000', '#aaffc3', '#808000',
    '#000075', '#a9a9a9',
]


def load_jsonl(filepath: str) -> dict:
    """Load JSONLines telemetry file and return dict of lists.
    JSONLines テレメトリファイルを読み込み、信号別リストの辞書を返す。

    Each sensor type gets its own time axis (_time_<sensor>) and data arrays.
    各センサ型は固有の時間軸と数値配列を持つ。
    """
    import json as _json

    # JSONLines id → (flat field name prefix, field extraction rules)
    # Each rule: (key_in_json, output_names, is_array)
    EXTRACT_RULES = {
        'imu': [
            ('gyro', ['gyro_x', 'gyro_y', 'gyro_z'], True),
            ('accel', ['accel_x', 'accel_y', 'accel_z'], True),
            ('gyro_raw', ['gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z'], True),
            ('accel_raw', ['accel_raw_x', 'accel_raw_y', 'accel_raw_z'], True),
            ('quat', ['quat_w', 'quat_x', 'quat_y', 'quat_z'], True),
            ('gyro_bias', ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z'], True),
            ('accel_bias', ['accel_bias_x', 'accel_bias_y', 'accel_bias_z'], True),
        ],
        'posvel': [
            ('pos', ['pos_x', 'pos_y', 'pos_z'], True),
            ('vel', ['vel_x', 'vel_y', 'vel_z'], True),
        ],
        'ctrl': [
            ('throttle', ['ctrl_throttle'], False),
            ('roll', ['ctrl_roll'], False),
            ('pitch', ['ctrl_pitch'], False),
            ('yaw', ['ctrl_yaw'], False),
        ],
        'flow': [
            ('dx', ['flow_x'], False),
            ('dy', ['flow_y'], False),
            ('quality', ['flow_quality'], False),
        ],
        'tof_b': [
            ('distance', ['tof_bottom'], False),
            ('status', ['tof_bottom_status'], False),
        ],
        'tof_f': [
            ('distance', ['tof_front'], False),
            ('status', ['tof_front_status'], False),
        ],
        'baro': [
            ('altitude', ['baro_altitude'], False),
            ('pressure', ['baro_pressure'], False),
        ],
        'mag': [
            ('x', ['mag_x'], False),
            ('y', ['mag_y'], False),
            ('z', ['mag_z'], False),
        ],
        'ctrl_ref': [
            ('angle_ref', ['angle_ref_roll', 'angle_ref_pitch'], True),
            ('mode', ['flight_mode'], False),
            ('total_thrust', ['total_thrust'], False),
            ('motor_duty', ['actual_duty_FR', 'actual_duty_RR', 'actual_duty_RL', 'actual_duty_FL'], True),
        ],
        'rate_ref': [
            ('rate_ref', ['rate_ref_roll', 'rate_ref_pitch', 'rate_ref_yaw'], True),
        ],
        'status': [
            ('voltage', ['battery_voltage'], False),
            ('pid_roll', ['pid_roll_kp', 'pid_roll_ti', 'pid_roll_td'], True),
            ('pid_pitch', ['pid_pitch_kp', 'pid_pitch_ti', 'pid_pitch_td'], True),
            ('pid_yaw', ['pid_yaw_kp', 'pid_yaw_ti', 'pid_yaw_td'], True),
        ],
    }

    # Collect per-sensor time and data arrays
    # センサ別の時間と数値配列を収集
    sensor_times = {}   # {sensor_id: [ts, ts, ...]}
    sensor_data = {}    # {field_name: [val, val, ...]}

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = _json.loads(line)
            sid = obj.get('id', '')
            ts = obj.get('ts', 0)

            if sid not in EXTRACT_RULES:
                continue

            if sid not in sensor_times:
                sensor_times[sid] = []
            sensor_times[sid].append(ts)

            for json_key, out_names, is_array in EXTRACT_RULES[sid]:
                val = obj.get(json_key)
                if val is None:
                    continue
                if is_array:
                    for i, name in enumerate(out_names):
                        if name not in sensor_data:
                            sensor_data[name] = []
                        sensor_data[name].append(val[i] if i < len(val) else 0.0)
                else:
                    name = out_names[0]
                    if name not in sensor_data:
                        sensor_data[name] = []
                    sensor_data[name].append(float(val))

    # Build output data dict
    # 出力データ辞書を構築
    data = {}

    # Use IMU's first timestamp as t0 (highest rate, most reliable)
    # IMU の最初のタイムスタンプを t0 として使用（最高レート、最も信頼性が高い）
    if 'imu' in sensor_times and sensor_times['imu']:
        t0 = sensor_times['imu'][0]
    else:
        all_first_ts = [ts[0] for ts in sensor_times.values() if ts]
        t0 = min(all_first_ts) if all_first_ts else 0

    # Create per-sensor time axes and assign data
    # センサ別時間軸を作成しデータを割り当て
    SENSOR_TIME_KEY = {
        'imu': '_time_imu',
        'posvel': '_time_posvel',
        'ctrl': '_time_ctrl',
        'flow': '_time_flow',
        'tof_b': '_time_tof_b',
        'tof_f': '_time_tof_f',
        'baro': '_time_baro',
        'mag': '_time_mag',
        'ctrl_ref': '_time_ctrl_ref',
        'rate_ref': '_time_rate_ref',
        'status': '_time_status',
    }

    for sid, times in sensor_times.items():
        time_key = SENSOR_TIME_KEY.get(sid, f'_time_{sid}')
        data[time_key] = [(t - t0) / 1e6 for t in times]

    # Use IMU time as the primary time_s (highest rate)
    # IMU 時間を主要 time_s として使用（最高レート）
    if '_time_imu' in data:
        data['time_s'] = data['_time_imu']
    elif '_time_posvel' in data:
        data['time_s'] = data['_time_posvel']
    else:
        # Fallback: use first available sensor time
        for key in data:
            if key.startswith('_time_'):
                data['time_s'] = data[key]
                break

    # Add all signal data
    data.update(sensor_data)

    # NOTE: Invalid sensor data is NOT masked here.
    # Raw data + quality/status are passed to HTML so JavaScript can
    # toggle invalid data visibility via checkbox at display time.
    # 注意: 無効センサデータはここではマスクしない。
    # 生データ + quality/status を HTML に渡し、JavaScript のチェックボックスで
    # 表示時に無効データの表示/非表示を切り替える。

    # Compute corrected IMU (gyro - bias, accel - bias)
    # バイアス補正済み IMU を計算（gyro - bias, accel - bias）
    if all(k in data for k in ['gyro_x', 'gyro_bias_x']):
        n = len(data['gyro_x'])
        for axis in ['x', 'y', 'z']:
            g = data[f'gyro_{axis}']
            b = data[f'gyro_bias_{axis}']
            data[f'gyro_corrected_{axis}'] = [g[i] - b[i] for i in range(n)]

    if all(k in data for k in ['accel_x', 'accel_bias_x']):
        n = len(data['accel_x'])
        for axis in ['x', 'y', 'z']:
            a = data[f'accel_{axis}']
            b = data[f'accel_bias_{axis}']
            data[f'accel_corrected_{axis}'] = [a[i] - b[i] for i in range(n)]

    # Average actual motor duty (from telemetry, includes Vbat effect)
    # 実測モータDuty平均（テレメトリから、電圧補正済み）
    if all(k in data for k in ['actual_duty_FR', 'actual_duty_RR', 'actual_duty_RL', 'actual_duty_FL']):
        n = len(data['actual_duty_FR'])
        data['total_duty'] = [(data['actual_duty_FR'][i] + data['actual_duty_RR'][i] +
                               data['actual_duty_RL'][i] + data['actual_duty_FL'][i]) / 4.0
                              for i in range(n)]

    # Compute derived signals from quaternion (same as load_csv)
    # クォータニオンから派生信号を計算（load_csv と同じ）
    if all(k in data for k in ['quat_w', 'quat_x', 'quat_y', 'quat_z']):
        n = len(data['quat_w'])
        data['roll_deg'] = [0.0] * n
        data['pitch_deg'] = [0.0] * n
        data['yaw_deg'] = [0.0] * n
        for i in range(n):
            w, x, y, z = data['quat_w'][i], data['quat_x'][i], data['quat_y'][i], data['quat_z'][i]
            data['roll_deg'][i] = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
            data['pitch_deg'][i] = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
            data['yaw_deg'][i] = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

    # Convert angle_ref from rad to deg for display
    # 表示用に angle_ref を rad → deg 変換
    if 'angle_ref_roll' in data:
        data['angle_ref_roll_deg'] = [math.degrees(v) for v in data['angle_ref_roll']]
        data['angle_ref_pitch_deg'] = [math.degrees(v) for v in data['angle_ref_pitch']]

    # =========================================================================
    # Computed: PID output + Motor duty reconstruction
    # PID出力 + モータDuty再構成
    # Requires: rate_ref (400Hz) + gyro_corrected (400Hz) + ctrl_throttle (50Hz)
    # =========================================================================
    if all(k in data for k in ['rate_ref_roll', 'gyro_corrected_x', 'ctrl_throttle']):
        n_imu = len(data['gyro_corrected_x'])
        n_rate = len(data['rate_ref_roll'])
        n = min(n_imu, n_rate)
        dt = 2.5e-3  # 400Hz

        # Resample throttle (50Hz ctrl) to 400Hz via ZOH
        # スロットルを50Hz→400Hzにリサンプル
        t_imu = data.get('_time_imu', list(range(n)))
        t_ctrl = data.get('_time_ctrl', t_imu)
        n_ctrl = len(data['ctrl_throttle'])
        if n_ctrl < n:
            import numpy as _np
            thr_400 = _np.interp(
                t_imu[:n] if len(t_imu) >= n else list(range(n)),
                t_ctrl[:n_ctrl] if len(t_ctrl) >= n_ctrl else list(range(n_ctrl)),
                data['ctrl_throttle'][:n_ctrl]
            ).tolist()
        else:
            thr_400 = data['ctrl_throttle'][:n]

        # PID parameters (physical units mode)
        # PIDパラメータ（物理単位モード）
        # Use PID gains from log if available (status packet), else fallback to hardcoded
        # ログにPIDゲインがあれば使用（statusパケット）、なければフォールバック
        pid_cfg_default = [
            {'Kp': 1.365e-3, 'Ti': 0.7, 'Td': 0.01, 'eta': 0.125, 'lim': 5.2e-3},  # Roll
            {'Kp': 1.995e-3, 'Ti': 0.7, 'Td': 0.01, 'eta': 0.125, 'lim': 5.2e-3},  # Pitch
            {'Kp': 5.31e-3,  'Ti': 1.6, 'Td': 0.01, 'eta': 0.125, 'lim': 2.2e-3},  # Yaw
        ]
        pid_cfg = pid_cfg_default
        if 'pid_roll_kp' in data and data['pid_roll_kp']:
            # Take first valid PID gains from status packets
            pid_cfg = [
                {'Kp': data['pid_roll_kp'][0],  'Ti': data['pid_roll_ti'][0],  'Td': data['pid_roll_td'][0],  'eta': 0.125, 'lim': 5.2e-3},
                {'Kp': data['pid_pitch_kp'][0], 'Ti': data['pid_pitch_ti'][0], 'Td': data['pid_pitch_td'][0], 'eta': 0.125, 'lim': 5.2e-3},
                {'Kp': data['pid_yaw_kp'][0],   'Ti': data['pid_yaw_ti'][0],  'Td': data['pid_yaw_td'][0],   'eta': 0.125, 'lim': 2.2e-3},
            ]
        rate_ref_keys = ['rate_ref_roll', 'rate_ref_pitch', 'rate_ref_yaw']
        gyro_keys = ['gyro_corrected_x', 'gyro_corrected_y', 'gyro_corrected_z']
        pid_names = ['pid_out_roll', 'pid_out_pitch', 'pid_out_yaw']

        # Reconstruct PID outputs (matching firmware pid.cpp exactly)
        # PID出力を再構成（ファームウェア pid.cpp を忠実に再現）
        # - Trapezoidal integration (bilinear transform)
        # - Derivative-on-Measurement (D-on-M)
        # - Bilinear transform derivative filter
        # - Back-calculation anti-windup (Tt = sqrt(Ti*Td))
        pid_out = [[0.0]*n for _ in range(3)]
        for axis in range(3):
            p = pid_cfg[axis]
            Kp, Ti, Td, eta, lim = p['Kp'], p['Ti'], p['Td'], p['eta'], p['lim']
            Tt = math.sqrt(Ti * Td) if Ti > 0 and Td > 0 else Ti
            ref = data[rate_ref_keys[axis]]
            act = data[gyro_keys[axis]]

            integral = 0.0
            d_filt = 0.0
            prev_error = 0.0
            prev_d_input = -act[0]  # D-on-M: derivative of -measurement

            for i in range(1, n):
                e = ref[i] - act[i]

                # P term
                P = Kp * e

                # I term: trapezoidal integration
                integral += (dt / (2.0 * Ti)) * (e + prev_error)
                I = Kp * integral

                # D term: D-on-Measurement with bilinear transform filter
                d_input = -act[i]  # D-on-M
                alpha_d = 2.0 * eta * Td / dt
                deriv_a = (alpha_d - 1.0) / (alpha_d + 1.0)
                deriv_b = 2.0 * Td / ((alpha_d + 1.0) * dt)
                d_filt = deriv_a * d_filt + deriv_b * (d_input - prev_d_input)
                D = Kp * d_filt

                # Unlimited output
                out_unlimited = P + I + D

                # Clamp
                out = max(-lim, min(lim, out_unlimited))

                # Anti-windup: back-calculation
                saturation = out - out_unlimited
                if saturation != 0.0:
                    integral += saturation * (dt / Tt) / Kp

                prev_error = e
                prev_d_input = d_input
                pid_out[axis][i] = out
            data[pid_names[axis]] = pid_out[axis]

        # NOTE(2026-07-15): 実測物理値は Ct=6.7e-9, Cq=4.10e-11, κ=6.12e-3, Jmp=1.375e-8
        #  （multicopter_introduction qa_log Q4-9..13）。以下はファームウェア実装の
        #  鏡写しが目的のため、ファーム更新まで旧値を維持する。
        # Mixer matrix B⁻¹ (from ControlAllocator)
        # ミキサー行列 B⁻¹（ControlAllocatorと同一）
        d_arm = 0.023   # arm length [m]
        kappa = 9.71e-3  # Cq/Ct ratio
        inv_d = 1.0 / d_arm
        inv_k = 1.0 / kappa
        #              Thrust    Roll      Pitch     Yaw
        B_inv = [
            [ 0.25, -0.25*inv_d,  0.25*inv_d,  0.25*inv_k],  # FR M1
            [ 0.25, -0.25*inv_d, -0.25*inv_d, -0.25*inv_k],  # RR M2
            [ 0.25,  0.25*inv_d, -0.25*inv_d,  0.25*inv_k],  # RL M3
            [ 0.25,  0.25*inv_d,  0.25*inv_d, -0.25*inv_k],  # FL M4
        ]

        # Motor model parameters (from motor_model.cpp DEFAULT_MOTOR_PARAMS)
        # モータモデルパラメータ
        Ct = 1.0e-8
        Cq = 9.71e-11
        Rm = 0.34
        Km = 6.125e-4
        Dm = 3.69e-8
        Qf = 2.76e-5
        Vbat = 3.7
        max_thrust = 0.168  # duty≤0.95 (5% margin for control)
        MAX_TOTAL_THRUST = 4 * max_thrust

        def thrust_to_duty(thrust):
            if thrust <= 0:
                return 0.0
            omega = math.sqrt(thrust / Ct)
            viscous = (Dm + Km*Km/Rm) * omega
            aero = Cq * omega * omega
            voltage = Rm * (viscous + aero + Qf) / Km
            duty = voltage / Vbat
            return max(0.0, min(1.0, duty))

        # Resample total_thrust from ctrl_ref (50Hz) to 400Hz if available
        # total_thrust をテレメトリから取得（50Hz→400Hz リサンプル）
        # Falls back to throttle × MAX_TOTAL_THRUST for legacy logs
        import numpy as _np2
        if 'total_thrust' in data and len(data['total_thrust']) > 0:
            t_ctrlref = data.get('_time_ctrl_ref', list(range(len(data['total_thrust']))))
            thrust_400 = _np2.interp(
                t_imu[:n] if len(t_imu) >= n else list(range(n)),
                t_ctrlref[:len(data['total_thrust'])],
                data['total_thrust'][:len(t_ctrlref)]
            ).tolist()
        else:
            thrust_400 = [thr_400[i] * MAX_TOTAL_THRUST for i in range(n)]

        # Compute motor thrusts and duties
        # モータ推力とDutyを計算
        motor_names = ['FR', 'RR', 'RL', 'FL']
        for m in range(4):
            data[f'motor_thrust_{motor_names[m]}'] = [0.0] * n
            data[f'motor_duty_{motor_names[m]}'] = [0.0] * n
        data['motor_saturated'] = [0.0] * n

        for i in range(n):
            u = [thrust_400[i], pid_out[0][i], pid_out[1][i], pid_out[2][i]]

            saturated = 0
            for m in range(4):
                thrust = sum(B_inv[m][j] * u[j] for j in range(4))
                if thrust < 0:
                    thrust = 0.0
                    saturated = 1
                elif thrust > max_thrust:
                    thrust = max_thrust
                    saturated = 1
                data[f'motor_thrust_{motor_names[m]}'][i] = thrust
                data[f'motor_duty_{motor_names[m]}'][i] = thrust_to_duty(thrust)
            data['motor_saturated'][i] = float(saturated)

    return data


def load_csv(filepath: str) -> dict:
    """Load CSV and return dict of lists (JSON-serializable)"""
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        columns = reader.fieldnames
        rows = list(reader)

    data = {}
    for col in columns:
        vals = []
        valid_count = 0
        for r in rows:
            v = r.get(col, '')
            if v == '' or v is None:
                vals.append(float('nan'))
            else:
                try:
                    vals.append(float(v))
                    valid_count += 1
                except ValueError:
                    vals.append(float('nan'))
        # Only include columns that have at least some valid data
        # 有効なデータが1つ以上ある列のみ含める
        if valid_count > 0:
            data[col] = vals

    # Compute time in seconds (telemetry capture time)
    if 'timestamp_us' in data:
        t0 = data['timestamp_us'][0]
        data['time_s'] = [(t - t0) / 1e6 for t in data['timestamp_us']]
    elif 'timestamp_ms' in data:
        t0 = data['timestamp_ms'][0]
        data['time_s'] = [(t - t0) / 1e3 for t in data['timestamp_ms']]

    # Compute sensor-specific time axes (using internal timestamps)
    # 各センサー固有の時間軸を生成（内部タイムスタンプ使用）
    # Use the first available timestamp as t0 reference
    if 'imu_timestamp_us' in data:
        t0_internal = data['imu_timestamp_us'][0]
    elif 'timestamp_us' in data:
        t0_internal = data['timestamp_us'][0]
    else:
        t0_internal = 0

    # IMU time axis
    if 'imu_timestamp_us' in data:
        data['_time_imu'] = [(t - t0_internal) / 1e6 for t in data['imu_timestamp_us']]

    # Sensor time axes with deduplication
    # 重複除去付きセンサー時間軸
    # For sensors that update slower than 400Hz, the same timestamp repeats.
    # We create deduplicated time+value arrays (suffix _dedup).
    sensor_ts_map = {
        'baro': ('baro_timestamp_us', ['baro_altitude', 'baro_pressure']),
        'tof_b': ('tof_timestamp_us', ['tof_bottom', 'tof_bottom_status']),
        'tof_f': ('tof_timestamp_us', ['tof_front', 'tof_front_status']),
        'mag': ('mag_timestamp_us', ['mag_x', 'mag_y', 'mag_z']),
        'flow': ('flow_timestamp_us', ['flow_x', 'flow_y', 'flow_quality']),
        'status': ('status_timestamp_us', ['battery_voltage']),
    }

    for sensor_name, (ts_key, signal_keys) in sensor_ts_map.items():
        if ts_key not in data:
            continue
        timestamps = data[ts_key]
        n = len(timestamps)
        if n == 0:
            continue

        # Find indices where timestamp changes (= new sensor reading)
        # タイムスタンプが変わったインデックス（= 新しいセンサー読み取り）
        unique_indices = [0]
        for i in range(1, n):
            if timestamps[i] != timestamps[i - 1]:
                unique_indices.append(i)

        # Create deduplicated time axis
        time_key = f'_time_{sensor_name}'
        data[time_key] = [(timestamps[i] - t0_internal) / 1e6 for i in unique_indices]

        # Create deduplicated signal arrays
        for sig_key in signal_keys:
            if sig_key in data:
                dedup_key = f'{sig_key}_dedup'
                data[dedup_key] = [data[sig_key][i] for i in unique_indices]

    # Compute attitude from quaternion (ESKF output)
    # クォータニオンから姿勢角を計算（ESKF出力）
    if all(k in data for k in ['quat_w', 'quat_x', 'quat_y', 'quat_z']):
        n = len(data['quat_w'])
        data['roll_deg'] = [0.0] * n
        data['pitch_deg'] = [0.0] * n
        data['yaw_deg'] = [0.0] * n
        for i in range(n):
            w, x, y, z = data['quat_w'][i], data['quat_x'][i], data['quat_y'][i], data['quat_z'][i]
            data['roll_deg'][i] = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
            data['pitch_deg'][i] = math.degrees(math.asin(max(-1, min(1, 2*(w*y - z*x)))))
            data['yaw_deg'][i] = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

    # =====================================================================
    # Computed sensor-only attitude estimates (for comparison with ESKF)
    # センサー単独の姿勢推定（ESKF出力との比較用）
    # =====================================================================

    n = len(data.get('time_s', []))

    # Compute IMU interval (jitter analysis)
    # IMU インターバル計算（ジッター解析）
    if 'imu_timestamp_us' in data and n > 1:
        data['imu_interval_us'] = [0.0] + [
            data['imu_timestamp_us'][i] - data['imu_timestamp_us'][i - 1]
            for i in range(1, n)
        ]

    # Compute telemetry interval
    if 'timestamp_us' in data and n > 1:
        data['telemetry_interval_us'] = [0.0] + [
            data['timestamp_us'][i] - data['timestamp_us'][i - 1]
            for i in range(1, n)
        ]

    # Compute telemetry delay (WiFi pipeline latency)
    # テレメトリ遅延（WiFi パイプラインのレイテンシ）
    # delay = telemetry capture time - IMU internal time
    if 'timestamp_us' in data and 'imu_timestamp_us' in data and n > 0:
        data['telemetry_delay_us'] = [
            data['timestamp_us'][i] - data['imu_timestamp_us'][i]
            for i in range(n)
        ]

    # Choose best time axis for gyro integration
    # ジャイロ積分に使う時間軸を選択（IMU内部時計が利用可能ならそちらを使用）
    gyro_time = data.get('_time_imu', data.get('time_s'))

    # 1a. Gyro integration (raw): cumulative integration of raw angular rate
    #     ジャイロ積分（生値）: 生の角速度を積分して角度を算出
    if gyro_time and all(k in data for k in ['gyro_x', 'gyro_y', 'gyro_z']) and n > 1:
        data['gyro_int_roll'] = [0.0] * n
        data['gyro_int_pitch'] = [0.0] * n
        data['gyro_int_yaw'] = [0.0] * n
        roll, pitch, yaw = 0.0, 0.0, 0.0
        for i in range(1, n):
            dt = gyro_time[i] - gyro_time[i - 1]
            if dt <= 0 or dt > 0.1:
                dt = 0.0025  # fallback to 400Hz
            roll += data['gyro_x'][i] * dt
            pitch += data['gyro_y'][i] * dt
            yaw += data['gyro_z'][i] * dt
            data['gyro_int_roll'][i] = math.degrees(roll)
            data['gyro_int_pitch'][i] = math.degrees(pitch)
            data['gyro_int_yaw'][i] = math.degrees(yaw)

    # 1b. Gyro integration (corrected): cumulative integration of bias-corrected gyro
    #     ジャイロ積分（補正値）: バイアス補正済み角速度を積分して角度を算出
    if gyro_time and all(k in data for k in ['gyro_corrected_x', 'gyro_corrected_y', 'gyro_corrected_z']) and n > 1:
        data['gyro_corr_int_roll'] = [0.0] * n
        data['gyro_corr_int_pitch'] = [0.0] * n
        data['gyro_corr_int_yaw'] = [0.0] * n
        roll, pitch, yaw = 0.0, 0.0, 0.0
        for i in range(1, n):
            dt = gyro_time[i] - gyro_time[i - 1]
            if dt <= 0 or dt > 0.1:
                dt = 0.0025
            roll += data['gyro_corrected_x'][i] * dt
            pitch += data['gyro_corrected_y'][i] * dt
            yaw += data['gyro_corrected_z'][i] * dt
            data['gyro_corr_int_roll'][i] = math.degrees(roll)
            data['gyro_corr_int_pitch'][i] = math.degrees(pitch)
            data['gyro_corr_int_yaw'][i] = math.degrees(yaw)

    # 2a. Accel-based attitude (raw): roll/pitch from raw accel gravity direction
    #     加速度ベースの姿勢（生値）: 生の加速度の重力方向からロール・ピッチ
    #     NED body frame: static accel ≈ [0, 0, -g]
    #     roll = atan2(-ay, -az), pitch = atan2(ax, sqrt(ay² + az²))
    if all(k in data for k in ['accel_x', 'accel_y', 'accel_z']) and n > 0:
        data['accel_roll'] = [0.0] * n
        data['accel_pitch'] = [0.0] * n
        for i in range(n):
            ax = data['accel_x'][i]
            ay = data['accel_y'][i]
            az = data['accel_z'][i]
            data['accel_roll'][i] = math.degrees(math.atan2(-ay, -az))
            data['accel_pitch'][i] = math.degrees(
                math.atan2(ax, math.sqrt(ay * ay + az * az)))

    # 2b. Accel-based attitude (corrected): roll/pitch from bias-corrected accel
    #     加速度ベースの姿勢（補正値）: バイアス補正済み加速度からロール・ピッチ
    if all(k in data for k in ['accel_corrected_x', 'accel_corrected_y', 'accel_corrected_z']) and n > 0:
        data['accel_corr_roll'] = [0.0] * n
        data['accel_corr_pitch'] = [0.0] * n
        for i in range(n):
            ax = data['accel_corrected_x'][i]
            ay = data['accel_corrected_y'][i]
            az = data['accel_corrected_z'][i]
            data['accel_corr_roll'][i] = math.degrees(math.atan2(-ay, -az))
            data['accel_corr_pitch'][i] = math.degrees(
                math.atan2(ax, math.sqrt(ay * ay + az * az)))

    # 3. Mag-based heading: yaw from magnetometer (tilt-compensated)
    #    地磁気ベースのヨー角: 傾き補正済み地磁気方位
    if all(k in data for k in ['mag_x', 'mag_y', 'mag_z']) and n > 0:
        data['mag_yaw'] = [0.0] * n
        for i in range(n):
            mx = data['mag_x'][i]
            my = data['mag_y'][i]
            # Tilt compensation using accel-derived roll/pitch if available
            # 加速度由来のロール・ピッチで傾き補正
            if 'accel_roll' in data and 'accel_pitch' in data:
                roll_r = math.radians(data['accel_roll'][i])
                pitch_r = math.radians(data['accel_pitch'][i])
                mz = data['mag_z'][i]
                # Rotate mag into horizontal plane
                # 地磁気を水平面に回転
                cos_r, sin_r = math.cos(roll_r), math.sin(roll_r)
                cos_p, sin_p = math.cos(pitch_r), math.sin(pitch_r)
                mx_h = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p
                my_h = my * cos_r - mz * sin_r
                data['mag_yaw'][i] = math.degrees(math.atan2(-my_h, mx_h))
            else:
                # No tilt compensation (2D heading)
                # 傾き補正なし（2D方位）
                data['mag_yaw'][i] = math.degrees(math.atan2(-my, mx))

    return data


def generate_html(data: dict, title: str, plotly_js: str = '') -> str:
    """Generate self-contained HTML dashboard"""

    # Filter categories to only include signals present in data
    categories = {}
    for cat, signals in SIGNAL_CATEGORIES.items():
        available = [(key, label) for key, label in signals if key in data]
        if available:
            categories[cat] = available

    # Serialize data to JSON (only signals that exist + time axes)
    all_keys = set()
    for sigs in categories.values():
        for key, _ in sigs:
            all_keys.add(key)
    all_keys.add('time_s')

    # Add internal time axes and dedup signals
    # 内部時間軸と重複除去済み信号を追加
    for k in list(data.keys()):
        if k.startswith('_time_') or k.endswith('_dedup'):
            all_keys.add(k)

    export_data = {k: data[k] for k in all_keys if k in data}

    # Build signal-to-time-axis mapping
    # 信号→時間軸のマッピングを構築
    # Each signal maps to its sensor-specific time axis.
    # 各信号はセンサ固有の時間軸にマッピングされる。
    signal_time_map = {}

    # Mapping: signal name → sensor time axis key
    # 信号名 → センサ時間軸キー のマッピング
    # Auto-build signal→sensor mapping from data lengths
    # データ長からシグナル→センサのマッピングを自動構築
    # Any signal whose length matches a _time_* axis gets mapped to it.
    # 長さが _time_* 軸と一致する信号は自動的にマッピングされる。
    sensor_time_axes = {}  # {'imu': ('_time_imu', 3928), ...}
    for k, v in data.items():
        if k.startswith('_time_'):
            sensor_name = k[6:]  # '_time_imu' → 'imu'
            sensor_time_axes[sensor_name] = (k, len(v))

    for sig, sig_data in data.items():
        if sig.startswith('_time_') or sig == 'time_s':
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

        # Fallback: check for dedup version (old CSV format)
        # フォールバック: dedup バージョンを確認（旧CSVフォーマット）
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
    import math as _math
    duration_ceil = _math.ceil(duration) if duration > 0 else 10

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
        ['roll_deg', 'pitch_deg', 'yaw_deg'].forEach(k => addSignalToPlot(p1, k, k));
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
        ['roll_deg', 'pitch_deg'].forEach(k => addSignalToPlot(p1, k, k));
        const p2 = addPlot();
        ['ctrl_throttle'].forEach(k => addSignalToPlot(p2, k, k));
        const p3 = addPlot();
        ['tof_bottom'].forEach(k => addSignalToPlot(p3, k, k));
        const p4 = addPlot();
        ['gyro_corrected_x', 'gyro_corrected_y', 'gyro_corrected_z'].forEach(k => addSignalToPlot(p4, k, k));
        const p5 = addPlot();
        ['accel_corrected_x', 'accel_corrected_y', 'accel_corrected_z'].forEach(k => addSignalToPlot(p5, k, k));
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


def load_file(filepath: str) -> dict:
    """Load telemetry data file (CSV or JSONLines).
    テレメトリデータファイルを読み込み（CSV または JSONLines）。
    """
    if filepath.endswith('.jsonl'):
        return load_jsonl(filepath)
    else:
        return load_csv(filepath)


def visualize(filepath: str, groups=None, layout=None, output=None, title=None):
    """Main entry point"""
    print(f"Loading: {filepath}")
    data = load_file(filepath)

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
    parser = argparse.ArgumentParser(description="Interactive telemetry dashboard")
    parser.add_argument('file', help="CSV telemetry file")
    parser.add_argument('-o', '--output', help="Save HTML to file")
    # These args are accepted for CLI compatibility but the browser UI handles selection
    parser.add_argument('--layout', help="(ignored, use browser UI)")
    parser.add_argument('--groups', nargs='+', help="(ignored, use browser UI)")
    args = parser.parse_args()
    visualize(args.file, output=args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
