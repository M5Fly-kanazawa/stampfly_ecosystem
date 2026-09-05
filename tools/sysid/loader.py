"""
Flight-log CSV loader for the sysid toolkit
sysid ツールキット向けフライトログ CSV ローダー

This is a small, self-contained replacement for the old tools/eskf_sim/loader.py,
which was deleted in commit 55074ad8 (old-SIL apparatus cleanup) while
tools/sysid/{noise,motor,drag,inertia}.py still imported `load_csv` from it.
It only implements the subset of the old LogData/SensorSample API that those
four modules actually use (see their `s.<attr>` / `log_data.<attr>` accesses).
これは削除された tools/eskf_sim/loader.py（コミット 55074ad8 で旧SIL一式と共に
削除）の代替。tools/sysid/{noise,motor,drag,inertia}.py が引き続き参照している
`load_csv` の API のうち、実際に使われているサブセットのみを再実装する。

Two CSV formats are produced by the current `sf log` toolchain and are
auto-detected from the header (see detect_csv_format):

  "stream"  -- `sf log wifi -o *.csv` (400Hz Data Stream). Written by
               tools/log_analyzer/udp_capture.py UDPTelemetryCapture.save_stream_csv().
               Columns: timestamp_us, gyro_x/y/z [rad/s], accel_x/y/z [m/s^2],
               quat_w/x/y/z, gyro_bias_x/y/z [rad/s], accel_bias_x/y/z [m/s^2],
               rate_ref_roll/pitch/yaw [rad/s], angle_ref_roll/pitch [rad],
               total_thrust, motor_duty_FR/RR/RL/FL, flight_mode.
               No baro/tof/flow/ctrl_* columns.
  "convert" -- `sf log convert` (USB binary log -> CSV). Written by
               tools/log_capture/log_capture.py convert_to_csv(), reading the
               128-byte LogPacket defined in
               firmware/vehicle_old/components/sf_svc_logger/include/logger.hpp.
               Columns: timestamp_ms, accel_x/y/z [m/s^2], gyro_x/y/z [rad/s],
               mag_x/y/z [uT], pressure [Pa], baro_alt [m], tof_bottom/front [m],
               flow_dx/dy [raw delta], flow_squal, pos_x/y/z [m] NED,
               vel_x/y/z [m/s] NED, roll/pitch/yaw [rad], gyro_bias_z [rad/s],
               accel_bias_x/y [m/s^2], eskf_status. No ctrl_* columns.

現行の `sf log` 系ツールが生成する CSV は2種類あり、ヘッダから自動判別する
（detect_csv_format 参照）:

  "stream"  -- `sf log wifi -o *.csv`（400Hz Data Stream）。上記の列を持つ。
               baro/tof/flow/ctrl_* 列は無い。
  "convert" -- `sf log convert`（USB バイナリログ→CSV）。上記の列を持つ。
               ctrl_* 列は無い。
"""

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# Timestamp unit conversion / タイムスタンプの単位変換
_MS_TO_US = 1000

# Fallback sample rate used only when a file has a single row (duration = 0)
# so callers get a finite, documented value instead of a divide-by-zero.
# 1行しか無くdurationが0のときのみ使う既定サンプルレート（ゼロ割り回避）。
_FALLBACK_SAMPLE_RATE_HZ = 400.0


@dataclass
class SensorSample:
    """Single sensor epoch. Only the fields tools/sysid/*.py reads are kept.
    センサの1エポック分。tools/sysid/*.py が参照するフィールドのみ保持する。
    """
    timestamp_us: int                              # Timestamp [us]
    gyro: np.ndarray                                # [rad/s] body frame
    accel: np.ndarray                               # [m/s^2] body frame
    gyro_corrected: Optional[np.ndarray] = None     # [rad/s], if distinct from gyro
    eskf_velocity: Optional[np.ndarray] = None      # [m/s] NED
    baro_altitude: Optional[float] = None           # [m]
    tof_distance: Optional[float] = None            # [m]
    flow_dx: Optional[int] = None                   # raw delta
    flow_dy: Optional[int] = None                   # raw delta
    ctrl_throttle: Optional[float] = None           # normalized stick
    ctrl_roll: Optional[float] = None                # normalized stick
    ctrl_pitch: Optional[float] = None               # normalized stick
    ctrl_yaw: Optional[float] = None                 # normalized stick


@dataclass
class LogData:
    """Whole log, as loaded by load_csv().
    load_csv() が返すログ全体。
    """
    samples: List[SensorSample]
    sample_rate_hz: float
    format: str   # "stream" or "convert"

    def __len__(self) -> int:
        return len(self.samples)


def detect_csv_format(columns: Optional[List[str]]) -> str:
    """Detect which of the two current `sf log` CSV schemas a header uses.
    ヘッダから現行2種類の `sf log` CSV 形式のどちらかを判別する。

    Raises:
        ValueError: neither schema's required columns are present.
    """
    cols = set(columns or [])

    if "timestamp_us" in cols and {"gyro_x", "gyro_y", "gyro_z"} <= cols:
        return "stream"
    if "timestamp_ms" in cols and {"gyro_x", "gyro_y", "gyro_z"} <= cols and "roll" in cols:
        return "convert"

    raise ValueError(
        "CSV format not recognized -- need either the `sf log wifi -o *.csv` "
        "Data Stream columns (timestamp_us + gyro_x/y/z) or the `sf log "
        f"convert` columns (timestamp_ms + gyro_x/y/z + roll). Header columns "
        f"found: {sorted(cols)}"
    )


def _optional_float(row: Dict[str, str], key: str) -> Optional[float]:
    """Parse a CSV field to float, returning None if absent/blank/invalid.
    CSV の値を float に変換する。列が無い・空・不正な場合は None を返す。
    """
    value = row.get(key)
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _parse_stream_row(row: Dict[str, str]) -> SensorSample:
    """Parse one row of the `sf log wifi -o *.csv` Data Stream format.
    `sf log wifi -o *.csv` の Data Stream 形式の1行を解析する。
    """
    return SensorSample(
        timestamp_us=int(float(row["timestamp_us"])),
        gyro=np.array([float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])]),
        accel=np.array([float(row["accel_x"]), float(row["accel_y"]), float(row["accel_z"])]),
    )


def _parse_convert_row(row: Dict[str, str]) -> SensorSample:
    """Parse one row of the `sf log convert` binary-log CSV format.
    `sf log convert` のバイナリログ CSV 形式の1行を解析する。
    """
    sample = SensorSample(
        timestamp_us=int(round(float(row["timestamp_ms"]) * _MS_TO_US)),
        gyro=np.array([float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])]),
        accel=np.array([float(row["accel_x"]), float(row["accel_y"]), float(row["accel_z"])]),
        baro_altitude=_optional_float(row, "baro_alt"),
        tof_distance=_optional_float(row, "tof_bottom"),
    )

    flow_dx = _optional_float(row, "flow_dx")
    flow_dy = _optional_float(row, "flow_dy")
    if flow_dx is not None and flow_dy is not None:
        sample.flow_dx = int(flow_dx)
        sample.flow_dy = int(flow_dy)

    vel_x = _optional_float(row, "vel_x")
    vel_y = _optional_float(row, "vel_y")
    vel_z = _optional_float(row, "vel_z")
    if vel_x is not None and vel_y is not None and vel_z is not None:
        sample.eskf_velocity = np.array([vel_x, vel_y, vel_z])

    return sample


def load_csv(filepath: str | Path) -> LogData:
    """Load a `sf log` CSV file (either "stream" or "convert" schema).
    `sf log` の CSV ファイルを読み込む（"stream"/"convert" いずれかの形式）。

    Args:
        filepath: Path to CSV file / CSV ファイルへのパス

    Returns:
        LogData with parsed samples, auto-detected sample rate and format.

    Raises:
        FileNotFoundError: file does not exist.
        ValueError: header format not recognized, or no valid rows parsed.
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Log file not found: {filepath}")

    with open(filepath, "r", newline="") as f:
        reader = csv.DictReader(f)
        fmt = detect_csv_format(reader.fieldnames)
        parse_row = _parse_stream_row if fmt == "stream" else _parse_convert_row

        samples: List[SensorSample] = []
        for row in reader:
            try:
                samples.append(parse_row(row))
            except (ValueError, KeyError):
                continue  # Skip malformed rows / 不正な行はスキップ

    if not samples:
        raise ValueError(f"No valid samples in file: {filepath}")

    duration_s = (samples[-1].timestamp_us - samples[0].timestamp_us) / 1e6
    sample_rate_hz = len(samples) / duration_s if duration_s > 0 else _FALLBACK_SAMPLE_RATE_HZ

    return LogData(samples=samples, sample_rate_hz=sample_rate_hz, format=fmt)
