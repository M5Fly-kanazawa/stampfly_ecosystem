"""
conftest.py - shared pytest fixtures for tools/log_analyzer tests
tools/log_analyzer のテスト共通フィクスチャ

Two inputs every log-analyzer test needs:
  * a synthetic StampFly flight-log v1 bundle (lib/sflog; docs/plans/
    flight-log-format-plan.md) with EVERY stream the renderers/analyzers
    read, at their native multi-rate cadence, so a test never has to
    hand-build DataFrames again;
  * the real-vehicle reference bundle under analysis/datasets/flightlog/
    (a fresh clone has no logs/ directory -- this fixture is the only real
    data CI can read).
ログ解析系のテストが共通して必要とする2つの入力:
  * 描画/解析側が読む全ストリームを原レートで持つ合成のフライトログ
    v1 一式（lib/sflog; 計画書参照）。各テストが DataFrame を手組みしなくて
    済むようにする。
  * analysis/datasets/flightlog/ にある実機由来の基準一式（clone 直後には
    logs/ が無いので、CI が読める実データはこれだけ）。

`build_synthetic_log()` is a plain function (not a fixture) so a test can
also build the log in memory without touching disk.
`build_synthetic_log()` はフィクスチャではなく素の関数なので、ディスクに
書かずメモリ上で組み立てたいテストからも呼べる。
"""

import math
from pathlib import Path

import pandas as pd
import pytest

import sflog

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# Real-vehicle reference bundle (30 s indoor hover, 2026-09-08). See
# analysis/datasets/flightlog/README.md for provenance.
# 実機由来の基準一式（2026-09-08 の屋内ホバー 30 秒）。由来は
# analysis/datasets/flightlog/README.md 参照。
REFERENCE_BUNDLE = (
    _REPO_ROOT / "analysis" / "datasets" / "flightlog"
    / "vehicle_hover_20260908T121243.sflog.zip"
)

# Synthetic flight: 5 s at the firmware's 400 Hz control rate, with the
# slower streams decimated by their nominal-rate stride.
# 合成飛行: 400 Hz 制御周期で 5 秒。低速ストリームは公称レート比で間引く。
SAMPLE_RATE_HZ = 400
DURATION_S = 5
N_SAMPLES = SAMPLE_RATE_HZ * DURATION_S
DT_US = 1_000_000 // SAMPLE_RATE_HZ
FIRST_TIMESTAMP_US = 1_000_000

STRIDE_50HZ = 8     # pilot / ctrl_ref / baro           (400/50)
STRIDE_100HZ = 4    # flow                              (400/100)
STRIDE_30HZ = 13    # tof (400/30 rounded)              (400/30)
STRIDE_25HZ = 16    # mag                               (400/25)
STRIDE_1HZ = 400    # status                            (400/1)

# Roll rate_ref step at t = 2.5 s: 0 -> STEP_RATE_RAD_S, so a rate panel
# and a sysid fit have something to look at.
# t = 2.5 s でロール rate_ref をステップ（0 -> STEP_RATE_RAD_S）。レート
# パネルや同定が見るべき変化を持たせる。
STEP_RATE_RAD_S = 0.1
GYRO_SINE_AMPLITUDE_RAD_S = 0.1
GYRO_SINE_PERIOD_SAMPLES = 50
HOVER_DUTY = 0.5
HOVER_THRUST_N = 0.5
GRAVITY_M_S2 = 9.81
HOVER_ALT_M = 0.5
BATTERY_V = 3.9
BARO_PRESSURE_PA = 101_325.0


def build_synthetic_log(duration_s: float = DURATION_S) -> sflog.FlightLog:
    """Build the synthetic bundle in memory (every stream, native rates).
    `duration_s` lengthens the flight (default DURATION_S = 5 s) for tests
    that need a hover longer than motor_health.MIN_HOVER_S.
    合成一式をメモリ上に組み立てる（全ストリーム、原レート）。`duration_s`
    で飛行を延ばせる（既定 DURATION_S = 5 秒）。motor_health.MIN_HOVER_S より
    長いホバーが要るテスト向け。"""
    n_samples = int(SAMPLE_RATE_HZ * duration_s)
    ts = [FIRST_TIMESTAMP_US + i * DT_US for i in range(n_samples)]
    seq = list(range(n_samples))
    gyro_x = [GYRO_SINE_AMPLITUDE_RAD_S * math.sin(i / GYRO_SINE_PERIOD_SAMPLES)
              for i in range(n_samples)]
    rate_ref_roll = [STEP_RATE_RAD_S if i >= n_samples // 2 else 0.0
                     for i in range(n_samples)]

    imu = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'gyro_x': gyro_x, 'gyro_y': 0.0, 'gyro_z': 0.0,
        'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': -GRAVITY_M_S2,
        'gyro_raw_x': gyro_x, 'gyro_raw_y': 0.0, 'gyro_raw_z': 0.0,
        'accel_raw_x': 0.0, 'accel_raw_y': 0.0, 'accel_raw_z': -GRAVITY_M_S2,
    })
    attitude = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0,
        'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
        'accel_bias_x': 0.0, 'accel_bias_y': 0.0, 'accel_bias_z': 0.0,
    })
    posvel = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'pos_x': 0.0, 'pos_y': 0.0, 'pos_z': -HOVER_ALT_M,
        'vel_x': 0.0, 'vel_y': 0.0, 'vel_z': 0.0,
    })
    rate_ref = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'rate_ref_roll': rate_ref_roll, 'rate_ref_pitch': 0.0, 'rate_ref_yaw': 0.0,
    })
    motor = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'duty_FR': HOVER_DUTY, 'duty_RR': HOVER_DUTY,
        'duty_RL': HOVER_DUTY, 'duty_FL': HOVER_DUTY,
    })
    ctrl_output = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'thrust': HOVER_THRUST_N,
        'torque_roll': 0.0, 'torque_pitch': 0.0, 'torque_yaw': 0.0,
    })
    ts_50 = ts[::STRIDE_50HZ]
    pilot = pd.DataFrame({
        'timestamp_us': ts_50,
        'throttle': HOVER_DUTY, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    })
    ctrl_ref = pd.DataFrame({
        'timestamp_us': ts_50,
        'flight_mode': 1,
        'angle_ref_roll': 0.0, 'angle_ref_pitch': 0.0,
        'total_thrust': HOVER_THRUST_N,
        'duty_FR': HOVER_DUTY, 'duty_RR': HOVER_DUTY,
        'duty_RL': HOVER_DUTY, 'duty_FL': HOVER_DUTY,
        'alt_setpoint': HOVER_ALT_M, 'alt_vel_target': 0.0, 'climb_rate_cmd': 0.0,
        'pos_setpoint_x': 0.0, 'pos_setpoint_y': 0.0,
    })
    baro = pd.DataFrame({
        'timestamp_us': ts_50,
        'altitude': HOVER_ALT_M, 'pressure': BARO_PRESSURE_PA,
    })
    ts_30 = ts[::STRIDE_30HZ]
    tof_bottom = pd.DataFrame({
        'timestamp_us': ts_30, 'distance': HOVER_ALT_M, 'status': 0,
    })
    tof_front = pd.DataFrame({
        'timestamp_us': ts_30, 'distance': 1.0, 'status': 0,
    })
    flow = pd.DataFrame({
        'timestamp_us': ts[::STRIDE_100HZ], 'dx': 0.0, 'dy': 0.0, 'quality': 100,
    })
    mag = pd.DataFrame({
        'timestamp_us': ts[::STRIDE_25HZ], 'x': 20.0, 'y': 0.0, 'z': -40.0,
    })
    ts_1 = ts[::STRIDE_1HZ]
    status = pd.DataFrame({
        'timestamp_us': ts_1,
        'uptime_ms': [t // 1000 for t in ts_1],
        'voltage': BATTERY_V, 'current_ma': 500.0,
        'flight_state': 2, 'sensor_health': 0, 'eskf_status': 1, 'reset_reason': 0,
        'pid_roll_kp': 1.0e-3, 'pid_roll_ti': 0.5, 'pid_roll_td': 0.002,
        'pid_pitch_kp': 1.0e-3, 'pid_pitch_ti': 0.5, 'pid_pitch_td': 0.002,
        'pid_yaw_kp': 1.2e-3, 'pid_yaw_ti': 1.0, 'pid_yaw_td': 0.0,
    })

    streams = {
        'imu': imu, 'attitude': attitude, 'posvel': posvel, 'rate_ref': rate_ref,
        'motor': motor, 'ctrl_output': ctrl_output, 'pilot': pilot,
        'ctrl_ref': ctrl_ref, 'baro': baro, 'tof_bottom': tof_bottom,
        'tof_front': tof_front, 'flow': flow, 'mag': mag, 'status': status,
    }
    meta = sflog.make_meta(
        source='sim', tool_name='tools/log_analyzer/conftest', tool_version='0.0.0',
        streams=streams,
    )
    return sflog.FlightLog(
        meta=meta, schema=sflog.schema.schema_for(streams.keys()), streams=streams,
    )


def write_synthetic_bundle(path: Path) -> Path:
    """Write the synthetic bundle to `path` (a `.sflog.zip`) and return it.
    合成一式を `path`（`.sflog.zip`）へ書き出し、そのパスを返す。"""
    build_synthetic_log().save(path)
    return path


@pytest.fixture
def synthetic_bundle(tmp_path) -> Path:
    """Path to a freshly written synthetic `flight_synthetic.sflog.zip`.
    書きたての合成一式 `flight_synthetic.sflog.zip` のパス。"""
    return write_synthetic_bundle(tmp_path / "flight_synthetic.sflog.zip")


@pytest.fixture
def reference_bundle() -> Path:
    """Path to the real-vehicle reference bundle (skips if missing).
    実機由来の基準一式のパス（無ければ skip）。"""
    if not REFERENCE_BUNDLE.exists():
        pytest.skip(f"reference bundle missing: {REFERENCE_BUNDLE}")
    return REFERENCE_BUNDLE
