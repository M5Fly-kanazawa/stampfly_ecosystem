"""
Plant Model Fitting from Closed-Loop Flight Data
閉ループフライトデータからのプラントモデル同定

Identifies open-loop plant parameters from P-control flight data:
  G_p(s) = K / (s * (tau_m * s + 1))

Where:
  K    : Plant gain [rad/s^2 per duty]
  tau_m: Motor time constant [s]

Algorithm:
  Since Kp is known, plant I/O can be reconstructed from telemetry. Two CSV
  schemas are auto-detected from the header (see _detect_csv_format):
    "stream" (current 400Hz Data Stream, `sf log wifi -o *.csv` -- shared by
              vehicle and workshop, see data_stream_wire.hpp):
      target(t) = rate_ref_<axis>(t)            <- already rad/s, no scaling
    "legacy" (pre-migration analysis CSV):
      target(t) = ctrl_<axis>(t) * rate_max     <- normalized stick * rate_max
  Either way:
    u_plant(t) = Kp * (target(t) - gyro(t))
    y_plant(t) = gyro(t)

  The open-loop model is fitted directly via MSE minimization:
    u_plant -> G_p(s) -> y_simulated
    minimize |y_simulated - y_plant|^2  ->  K, tau_m

閉ループ P 制御フライトデータから開ループプラントパラメータを同定する。
CSV ヘッダから2種類の形式を自動判別する（_detect_csv_format 参照）:
  "stream"（現行 400Hz Data Stream。`sf log wifi -o *.csv` — vehicle と
           workshop で共通、data_stream_wire.hpp 参照）:
    target(t) = rate_ref_<axis>(t)              <- 既に rad/s、スケール不要
  "legacy"（移行前の分析用 CSV）:
    target(t) = ctrl_<axis>(t) × rate_max       <- 正規化スティック × rate_max
いずれも:
  u_plant(t) = Kp × (target(t) − gyro(t))
  y_plant(t) = gyro(t)
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import minimize

from .defaults import get_flat_defaults


# Reference plant gains from L06 System Modeling
# L06 システムモデリングからの参照プラントゲイン
REFERENCE_PLANT_GAINS: Dict[str, float] = {
    'roll': 102.0,    # [rad/s^2 per duty]
    'pitch': 70.0,
    'yaw': 19.0,
}

# Body FRD axis order (roll about x, pitch about y, yaw about z) -- same
# convention used throughout the repo (see e.g. tools/log_analyzer/rate_sysid.py
# and data_stream_wire.hpp), kept identical here so `--axis` selection and the
# gyro sign/axis mapping never drift from the README's coordinate system.
# Body FRD 軸順（roll=x軸周り, pitch=y軸周り, yaw=z軸周り）— リポジトリ全体で
# 使われる規約と同一（例: tools/log_analyzer/rate_sysid.py, data_stream_wire.hpp）。
# --axis 選択とジャイロの軸・符号対応が README の座標系からずれないよう、
# ここでも同じ規約を使う。
_AXIS_NAMES: Tuple[str, str, str] = ('roll', 'pitch', 'yaw')

# "stream" format (sf log wifi -o *.csv): gyro column per axis.
# "stream" 形式（sf log wifi -o *.csv）: 軸ごとのジャイロ列。
_STREAM_GYRO_COL: Dict[str, str] = {'roll': 'gyro_x', 'pitch': 'gyro_y', 'yaw': 'gyro_z'}
# "stream" format: rate target column per axis (already rad/s, see
# workshop_control_task.cpp publishLogStream() / ws::set_rate_target()).
# "stream" 形式: 軸ごとの角速度目標列（既に rad/s。
# workshop_control_task.cpp publishLogStream() / ws::set_rate_target() 参照）。
_STREAM_TARGET_COL: Dict[str, str] = {
    'roll': 'rate_ref_roll', 'pitch': 'rate_ref_pitch', 'yaw': 'rate_ref_yaw',
}

# "legacy" format: normalized stick column per axis (multiplied by --rate-max).
# "legacy" 形式: 軸ごとの正規化スティック列（--rate-max を掛ける）。
_LEGACY_CTRL_COL: Dict[str, str] = {
    'roll': 'ctrl_roll', 'pitch': 'ctrl_pitch', 'yaw': 'ctrl_yaw',
}
# "legacy" format: gyro column per axis, preferring the bias-corrected value
# when present (falls back to _STREAM_GYRO_COL's raw gyro_x/y/z).
# "legacy" 形式: 軸ごとのジャイロ列。バイアス補正済みがあればそちらを優先
# （無ければ _STREAM_GYRO_COL の生ジャイロ gyro_x/y/z にフォールバック）。
_LEGACY_GYRO_COL: Dict[str, str] = {
    'roll': 'gyro_corrected_x', 'pitch': 'gyro_corrected_y', 'yaw': 'gyro_corrected_z',
}


@dataclass
class PlantFitResult:
    """Plant model identification result / プラントモデル同定結果

    Model: G_p(s) = K / (s * (tau_m * s + 1))
    """
    K: float              # Plant gain [rad/s^2 per duty]
    tau_m: float          # Motor time constant [s]
    K_std: float          # K uncertainty (std dev across segments)
    tau_m_std: float      # tau_m uncertainty (std dev across segments)
    r_squared: float      # Fit quality (mean R^2 across segments)
    rmse: float           # RMSE [rad/s] (mean across segments)
    axis: str             # 'roll', 'pitch', or 'yaw'
    kp_used: float        # Kp used for plant input reconstruction
    n_segments: int       # Number of segments used for fitting

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        defaults = get_flat_defaults()
        ref_K = REFERENCE_PLANT_GAINS.get(self.axis, 0.0)
        ref_tau_m = defaults['tau_m']

        result: Dict[str, Any] = {
            'method': 'plant_fit',
            'timestamp': datetime.now().isoformat(),
            'model': 'K / (s * (tau_m * s + 1))',
            'axis': self.axis,
            'kp_used': self.kp_used,
            'n_segments': self.n_segments,
            'estimated': {
                'K': round(self.K, 2),
                'K_uncertainty': round(self.K_std, 2),
                'tau_m': round(self.tau_m, 4),
                'tau_m_uncertainty': round(self.tau_m_std, 4),
            },
            'fit_quality': {
                'r_squared': round(self.r_squared, 4),
                'rmse': round(self.rmse, 4),
            },
            'reference': {
                'K': ref_K,
                'tau_m': ref_tau_m,
            },
            'comparison': {},
        }

        # Comparison with reference values
        # 参照値との比較
        if ref_K > 0:
            K_err = abs(self.K - ref_K) / ref_K * 100
            result['comparison']['K'] = {
                'error_percent': round(K_err, 1),
                'status': 'OK' if K_err < 30 else 'CHECK',
            }
        if ref_tau_m > 0:
            tau_err = abs(self.tau_m - ref_tau_m) / ref_tau_m * 100
            result['comparison']['tau_m'] = {
                'error_percent': round(tau_err, 1),
                'status': 'OK' if tau_err < 30 else 'CHECK',
            }

        # Derived: design Kp for zeta=0.7
        # Closed-loop: Kp = 1 / (4 * zeta^2 * K * tau_m)
        zeta = 0.7
        if self.K > 0 and self.tau_m > 0:
            Kp_design = 1.0 / (4.0 * zeta**2 * self.K * self.tau_m)
            result['derived'] = {
                'design_Kp_zeta_0_7': round(Kp_design, 4),
            }
            if ref_K > 0:
                Kp_ref = 1.0 / (4.0 * zeta**2 * ref_K * ref_tau_m)
                result['derived']['reference_Kp_zeta_0_7'] = round(Kp_ref, 4)

        return result


def _simulate_plant(
    K: float,
    tau_m: float,
    u: np.ndarray,
    dt: float,
    omega0: float,
    z0: float = 0.0,
) -> np.ndarray:
    """
    Simulate plant G_p(s) = K / (s * (tau_m * s + 1))
    プラントをシミュレート

    State space representation:
        x1_dot = x2               (omega_dot = z)
        x2_dot = -x2/tau_m + K*u/tau_m  (motor first-order dynamics)
        y      = x1               (output = angular velocity)

    Uses exact discretization for motor dynamics (x2)
    and trapezoidal integration for the integrator (x1).

    Args:
        K: Plant gain [rad/s^2 per duty]
        tau_m: Motor time constant [s]
        u: Plant input array (duty)
        dt: Sample period [s]
        omega0: Initial angular velocity [rad/s]
        z0: Initial motor-filtered acceleration [rad/s^2]

    Returns:
        Simulated angular velocity [rad/s]
    """
    n = len(u)
    alpha = np.exp(-dt / tau_m)
    gain = K * (1.0 - alpha)

    # Motor-filtered acceleration (x2 state)
    # 1次遅れ（モータ動特性）の厳密離散化
    z = np.empty(n)
    z[0] = z0
    for i in range(1, n):
        z[i] = alpha * z[i - 1] + gain * u[i - 1]

    # Integrate to angular velocity (trapezoidal rule)
    # 台形積分で角速度を計算
    omega = np.empty(n)
    omega[0] = omega0
    omega[1:] = omega0 + np.cumsum((z[:-1] + z[1:]) * 0.5 * dt)

    return omega


def _fit_segment(
    u_seg: np.ndarray,
    y_seg: np.ndarray,
    dt: float,
    K_init: float = 100.0,
    tau_m_init: float = 0.02,
) -> Optional[Tuple[float, float, float, float]]:
    """
    Fit K and tau_m for a single data segment
    単一セグメントの K, tau_m をフィット

    Args:
        u_seg: Plant input for segment
        y_seg: Measured output (gyro) for segment
        dt: Sample period [s]
        K_init: Initial guess for K
        tau_m_init: Initial guess for tau_m

    Returns:
        (K, tau_m, r_squared, rmse) or None if fitting fails
    """
    if len(u_seg) < 20:
        return None

    omega0 = y_seg[0]
    # Estimate initial angular acceleration from first few samples
    # 最初の数サンプルから初期角加速度を推定
    n_init = min(10, len(y_seg) - 1)
    z0 = float(y_seg[n_init] - y_seg[0]) / (n_init * dt)

    def objective(params):
        K = params[0]
        tau_m = np.exp(params[1])  # log transform ensures tau_m > 0
        y_sim = _simulate_plant(K, tau_m, u_seg, dt, omega0, z0)
        return np.mean((y_sim - y_seg) ** 2)

    try:
        result = minimize(
            objective,
            x0=[K_init, np.log(tau_m_init)],
            method='L-BFGS-B',
            bounds=[(1.0, 1000.0), (np.log(0.003), np.log(0.5))],
            options={'maxiter': 200},
        )
    except Exception:
        return None

    if not result.success and result.fun > 1.0:
        return None

    K_opt = result.x[0]
    tau_m_opt = np.exp(result.x[1])

    # Compute fit quality metrics
    # フィット品質の計算
    y_sim = _simulate_plant(K_opt, tau_m_opt, u_seg, dt, omega0, z0)
    residuals = y_seg - y_sim
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y_seg - np.mean(y_seg)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    return K_opt, tau_m_opt, r_squared, rmse


def _detect_csv_format(fieldnames: Optional[List[str]]) -> str:
    """
    Auto-detect which of the two CSV schemas a flight log uses, from its
    header alone.
    CSV のヘッダだけから、フライトログが2種類のうちどちらの形式かを自動判別。

    Returns:
        "stream" if rate_ref_<axis> + gyro_x/y/z columns are present (current
            400Hz Data Stream -- `sf log wifi -o *.csv`, shared by vehicle and
            workshop).
        "legacy" if ctrl_<axis> columns are present (pre-migration analysis
            CSV -- normalized stick + gyro_corrected_x/y/z, falling back to
            gyro_x/y/z).

    Raises:
        ValueError: neither schema's required columns are present.
    """
    cols = set(fieldnames or [])

    has_stream_target = any(c in cols for c in _STREAM_TARGET_COL.values())
    has_stream_gyro = all(c in cols for c in _STREAM_GYRO_COL.values())
    if has_stream_target and has_stream_gyro:
        return "stream"

    has_legacy_ctrl = any(c in cols for c in _LEGACY_CTRL_COL.values())
    if has_legacy_ctrl:
        return "legacy"

    raise ValueError(
        "CSV format not recognized -- need either the current Data Stream "
        "columns (rate_ref_roll/pitch/yaw + gyro_x/y/z, from `sf log wifi "
        "-o *.csv`) or the legacy analysis columns (ctrl_roll/pitch/yaw + "
        f"gyro_corrected_x/y/z or gyro_x/y/z). Header columns found: "
        f"{sorted(cols)}"
    )


def _load_axis_data(
    filepath: str | Path,
    axis: str,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, str]:
    """
    Load and extract axis-specific plant I/O from a flight-log CSV.
    CSV から軸固有のプラント入出力を読み込み・抽出する。

    Self-contained CSV reader (csv.DictReader + numpy only) that auto-detects
    the "stream" vs "legacy" schema via _detect_csv_format() -- see the module
    docstring for both schemas' column names and semantics.
    自己完結の CSV リーダ（csv.DictReader + numpy のみ）。
    _detect_csv_format() で "stream"/"legacy" を自動判別する — 両形式の列名・
    意味はモジュール docstring 参照。

    Returns:
        (time_s, target_raw, gyro, throttle, dt, fmt)
        target_raw is the RAW target column: already rad/s for "stream"
        (caller must NOT multiply by rate_max), normalized stick [-1, 1] for
        "legacy" (caller multiplies by rate_max). `fmt` tells the caller which.
        target_raw は列の生値: "stream" は既に rad/s（呼び出し側は rate_max を
        掛けてはいけない）、"legacy" は正規化スティック値 [-1, 1]（呼び出し側が
        rate_max を掛ける）。どちらかは `fmt` で判別する。
    """
    import csv as _csv

    if axis not in _AXIS_NAMES:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")

    with open(filepath, newline='') as f:
        reader = _csv.DictReader(f)
        fieldnames = reader.fieldnames
        fmt = _detect_csv_format(fieldnames)
        rows = list(reader)

    if len(rows) < 100:
        raise ValueError(f"Too few samples: {len(rows)}")

    cols = set(fieldnames or [])

    def col(row: Dict[str, str], name: str, default: float = 0.0) -> float:
        value = row.get(name)
        if value is None or value == '':
            return default
        try:
            return float(value)
        except ValueError:
            return default

    # --- Timestamp -> time_s, refining fs from the data when it looks sane ---
    # タイムスタンプ -> time_s。データから妥当な範囲ならサンプルレートを補正。
    ts_col = next((c for c in ('timestamp_us', 'timestamp', 'timestamp_ms') if c in cols), None)
    if ts_col is not None:
        scale_to_us = 1000.0 if ts_col == 'timestamp_ms' else 1.0
        ts_us = np.array([col(r, ts_col) for r in rows]) * scale_to_us
        time_s = (ts_us - ts_us[0]) / 1e6
        deltas = np.diff(time_s)
        deltas = deltas[deltas > 0]
        if len(deltas) > 10:
            detected_fs = 1.0 / float(np.median(deltas))
            if 0.5 * fs <= detected_fs <= 2.0 * fs:   # reject obviously-wrong units
                fs = detected_fs
    else:
        time_s = np.arange(len(rows)) / fs
    dt = 1.0 / fs

    # --- Target + gyro, per detected schema ---
    # 目標値 + ジャイロ（判別した形式に応じて）
    if fmt == "stream":
        target = np.array([col(r, _STREAM_TARGET_COL[axis]) for r in rows])
        gyro = np.array([col(r, _STREAM_GYRO_COL[axis]) for r in rows])
    else:
        target = np.array([col(r, _LEGACY_CTRL_COL[axis]) for r in rows])
        gyro_col = _LEGACY_GYRO_COL[axis] if _LEGACY_GYRO_COL[axis] in cols else _STREAM_GYRO_COL[axis]
        gyro = np.array([col(r, gyro_col) for r in rows])

    # --- Throttle-equivalent for flight-segment detection ---
    # 飛行区間検出用のスロットル相当量
    if fmt == "legacy":
        throttle = np.array([col(r, 'ctrl_throttle') for r in rows])
    elif 'total_thrust' in cols:
        throttle = np.array([col(r, 'total_thrust') for r in rows])
    elif all(f'motor_duty_{m}' in cols for m in ('FR', 'RR', 'RL', 'FL')):
        throttle = np.mean(
            [[col(r, f'motor_duty_{m}') for m in ('FR', 'RR', 'RL', 'FL')] for r in rows],
            axis=1,
        )
    else:
        raise ValueError(
            "stream-format CSV needs a flight-activity column to find flight "
            "segments: total_thrust (ws::motor_mixer thrust) or "
            "motor_duty_FR/RR/RL/FL. Neither was found in the header."
        )

    # Apply time range filter
    # 時間範囲フィルタを適用
    if time_range is not None:
        t_start, t_end = time_range
        mask = (time_s >= t_start) & (time_s <= t_end)
        time_s = time_s[mask]
        target = target[mask]
        gyro = gyro[mask]
        throttle = throttle[mask]

    return time_s, target, gyro, throttle, dt, fmt


def _find_flight_segments(
    throttle: np.ndarray,
    seg_samples: int,
    throttle_threshold: float = 0.3,
) -> List[Tuple[int, int]]:
    """
    Find flight segments where throttle > threshold
    スロットルが閾値以上の飛行区間を検出

    Returns:
        List of (start_idx, end_idx) tuples for analysis segments
    """
    in_flight = throttle > throttle_threshold

    # Find contiguous flight regions
    # 連続的な飛行区間を検出
    flight_starts = []
    flight_ends = []
    in_region = False
    for i in range(len(in_flight)):
        if in_flight[i] and not in_region:
            flight_starts.append(i)
            in_region = True
        elif not in_flight[i] and in_region:
            flight_ends.append(i)
            in_region = False
    if in_region:
        flight_ends.append(len(in_flight))

    # Split flight regions into analysis segments
    # 飛行区間を分析セグメントに分割
    min_seg = seg_samples // 2
    segments = []
    for start, end in zip(flight_starts, flight_ends):
        if end - start < min_seg:
            continue
        for seg_start in range(start, end - min_seg, seg_samples):
            seg_end = min(seg_start + seg_samples, end)
            if seg_end - seg_start >= min_seg:
                segments.append((seg_start, seg_end))

    return segments


def fit_plant(
    filepath: str | Path,
    axis: str = 'roll',
    kp: float = 0.5,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
    segment_length: float = 3.0,
    min_activity: float = 0.01,
) -> PlantFitResult:
    """
    Fit open-loop plant model from closed-loop flight data
    閉ループフライトデータから開ループプラントモデルを同定

    Args:
        filepath: Path to CSV flight log
        axis: 'roll', 'pitch', or 'yaw'
        kp: P gain used during flight (must match firmware value)
        rate_max: Maximum angular rate [rad/s] (maps ctrl [-1,+1] to rate)
        fs: Sample rate [Hz] (default: 400)
        time_range: Optional (start, end) in seconds to restrict analysis
        segment_length: Segment duration [s] for fitting (default: 3.0)
        min_activity: Minimum std of plant input to include segment

    Returns:
        PlantFitResult with identified K, tau_m and fit metrics

    Raises:
        ValueError: If data is insufficient or fitting fails
    """
    # Load data
    # データ読み込み
    time_s, target_raw, gyro, throttle, dt, fmt = _load_axis_data(
        filepath, axis, fs, time_range,
    )

    # Reconstruct plant I/O. "stream" CSVs already record the physical rate
    # target [rad/s] (ws::set_rate_target / vehicle rate_ref) so rate_max is
    # NOT applied; "legacy" CSVs store a normalized stick value that must be
    # scaled by rate_max first.
    # プラント入出力を復元。"stream" 形式は既に物理量の角速度目標 [rad/s]
    # （ws::set_rate_target / vehicle の rate_ref）を記録しているため rate_max
    # は適用しない。"legacy" 形式は正規化スティック値のため rate_max でスケール
    # する。
    #   u_plant(t) = Kp * (target(t) - gyro(t))
    #   y_plant(t) = gyro(t)
    target = target_raw if fmt == "stream" else target_raw * rate_max
    u_plant = kp * (target - gyro)
    y_plant = gyro

    # Find flight segments
    # 飛行区間の検出
    seg_samples = int(segment_length * (1.0 / dt))
    segments = _find_flight_segments(throttle, seg_samples)

    if not segments:
        raise ValueError(
            "No valid flight segments found. "
            "Check that throttle > 0.3 during flight."
        )

    # Filter segments by control activity level
    # 制御入力が十分な区間のみ抽出
    active_segments = []
    for seg_start, seg_end in segments:
        if np.std(u_plant[seg_start:seg_end]) > min_activity:
            active_segments.append((seg_start, seg_end))

    if not active_segments:
        raise ValueError(
            f"No segments with sufficient control activity (std > {min_activity}). "
            "Ensure the pilot made stick inputs during flight."
        )

    # Fit each segment
    # 各セグメントをフィット
    ref_K = REFERENCE_PLANT_GAINS.get(axis, 100.0)
    K_estimates: List[float] = []
    tau_m_estimates: List[float] = []
    r2_values: List[float] = []
    rmse_values: List[float] = []

    for seg_start, seg_end in active_segments:
        u_seg = u_plant[seg_start:seg_end]
        y_seg = y_plant[seg_start:seg_end]

        fit = _fit_segment(u_seg, y_seg, dt, K_init=ref_K, tau_m_init=0.02)
        if fit is not None:
            K, tau_m, r2, rmse = fit
            # Filter out unreasonable fits
            # 不合理なフィット結果を除外
            if r2 > 0.3 and K > 1.0 and 0.003 < tau_m < 0.5:
                K_estimates.append(K)
                tau_m_estimates.append(tau_m)
                r2_values.append(r2)
                rmse_values.append(rmse)

    if not K_estimates:
        raise ValueError(
            "Fitting failed for all segments. "
            "Check data quality and Kp value."
        )

    # Aggregate results using median (robust to outliers)
    # 中央値で集約（外れ値に頑健）
    K_final = float(np.median(K_estimates))
    tau_m_final = float(np.median(tau_m_estimates))
    K_std = float(np.std(K_estimates)) if len(K_estimates) > 1 else 0.0
    tau_m_std = float(np.std(tau_m_estimates)) if len(tau_m_estimates) > 1 else 0.0
    r2_mean = float(np.mean(r2_values))
    rmse_mean = float(np.mean(rmse_values))

    return PlantFitResult(
        K=K_final,
        tau_m=tau_m_final,
        K_std=K_std,
        tau_m_std=tau_m_std,
        r_squared=r2_mean,
        rmse=rmse_mean,
        axis=axis,
        kp_used=kp,
        n_segments=len(K_estimates),
    )


def compute_fit_timeseries(
    filepath: str | Path,
    result: PlantFitResult,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
) -> Dict[str, np.ndarray]:
    """
    Compute time series data for plotting fit results
    フィット結果のプロット用時系列データを計算

    Args:
        filepath: Path to CSV flight log (same file used for fitting)
        result: PlantFitResult from fit_plant()
        rate_max: Maximum angular rate [rad/s]
        fs: Sample rate [Hz]
        time_range: Optional (start, end) in seconds

    Returns:
        Dictionary with keys:
            'time': Time array [s]
            'u_plant': Reconstructed plant input
            'y_measured': Measured angular velocity (gyro)
            'y_simulated': Simulated angular velocity
            'residual': y_measured - y_simulated
    """
    time_s, target_raw, gyro, throttle, dt, fmt = _load_axis_data(
        filepath, result.axis, fs, time_range,
    )

    # Reconstruct plant I/O -- same fmt-dependent scaling as fit_plant()
    # プラント入出力を復元 -- fit_plant() と同じ fmt 依存のスケーリング
    target = target_raw if fmt == "stream" else target_raw * rate_max
    u_plant = result.kp_used * (target - gyro)
    y_measured = gyro

    # Simulate full time series with identified parameters
    # 同定パラメータで全時系列をシミュレート
    omega0 = y_measured[0]
    n_init = min(10, len(y_measured) - 1)
    z0 = float(y_measured[n_init] - y_measured[0]) / (n_init * dt)

    y_simulated = _simulate_plant(
        result.K, result.tau_m, u_plant, dt, omega0, z0,
    )

    return {
        'time': time_s,
        'u_plant': u_plant,
        'y_measured': y_measured,
        'y_simulated': y_simulated,
        'residual': y_measured - y_simulated,
    }


# =============================================================================
# Self-test: synthesize a "stream"-format flight from a KNOWN plant, recover
# it. Run via `sf sysid fit --selftest`.
# 自己テスト: 既知プラントから "stream" 形式のフライトを合成し、復元を検証。
# `sf sysid fit --selftest` から実行。
# =============================================================================

def selftest(verbose: bool = True) -> bool:
    """
    Closed-loop self-test for the "stream" (current Data Stream) CSV path.
    既知プラントを P 制御閉ループで離散シミュレーションし、`sf log wifi
    -o *.csv` と同じ "stream" 形式の CSV を合成、fit_plant() が K, tau_m を
    許容誤差内で復元することを確認する。

    Verifies end to end: _detect_csv_format() picks "stream", _load_axis_data()
    reads rate_ref_roll/gyro_x/total_thrust correctly, and the (format-
    independent) MSE fit in _fit_segment() recovers the plant that generated
    the data. Uses the EXACT same discretization as _simulate_plant() to
    generate the synthetic flight, so any recovery error reflects the fit
    tool's own accuracy, not a model mismatch.
    一気通貫の検証: _detect_csv_format() が "stream" を選び、_load_axis_data()
    が rate_ref_roll/gyro_x/total_thrust を正しく読み、（形式非依存の）
    _fit_segment() の MSE フィットが生成元のプラントを復元できることを確認
    する。合成フライトの生成には _simulate_plant() と全く同じ離散化を使うため、
    復元誤差はフィットツール自体の精度を反映し、モデル不整合には起因しない。
    """
    import csv as _csv
    import os
    import tempfile

    axis = 'roll'
    K_true = REFERENCE_PLANT_GAINS[axis]   # 102.0 [rad/s^2 per duty]
    tau_m_true = 0.02                      # [s] -- L06 nominal motor lag
    kp = 0.5
    fs = 400.0
    dt = 1.0 / fs
    n = 8000                               # 20 s @ 400 Hz

    alpha = np.exp(-dt / tau_m_true)
    gain = K_true * (1.0 - alpha)

    t = np.arange(n) * dt
    # Broadband log-chirp target (0.5->20 Hz over 5 s, repeated): similar
    # spectral richness to a pilot's stick doublets, wide enough to resolve
    # both the integrator gain K and the ~8 Hz motor-lag corner (tau_m=20ms).
    # 広帯域対数チャープ目標（0.5〜20Hz、5s周期で繰り返し）: パイロットの
    # スティックダブレットに近いスペクトルで、積分ゲイン K とモータ遅れの
    # コーナー周波数（tau_m=20ms、約8Hz）の両方を解ける帯域幅を持つ。
    f0, f1, period = 0.5, 20.0, 5.0
    k_chirp = (f1 / f0) ** (1.0 / period)
    tm = t % period
    target = 0.35 * np.sin(2 * np.pi * f0 * ((k_chirp ** tm) - 1.0) / np.log(k_chirp))

    # Closed-loop P-control simulation with the EXACT discretization
    # _simulate_plant() uses (motor-lag alpha filter + trapezoidal
    # integration) -- z[i]/omega[i] depend only on u[i-1], so this is causal
    # and needs no algebraic-loop solving.
    # _simulate_plant() と全く同じ離散化（モータ遅れの指数フィルタ + 台形
    # 積分）による閉ループ P 制御シミュレーション -- z[i]/omega[i] は
    # u[i-1] のみに依存するため、代数ループを解く必要のない因果的な計算。
    omega = np.zeros(n)
    z = np.zeros(n)
    u = np.zeros(n)
    for i in range(1, n):
        error = target[i - 1] - omega[i - 1]
        u[i - 1] = kp * error
        z[i] = alpha * z[i - 1] + gain * u[i - 1]
        omega[i] = omega[i - 1] + 0.5 * dt * (z[i - 1] + z[i])
    u[-1] = kp * (target[-1] - omega[-1])

    rng = np.random.default_rng(7)
    gyro_meas = omega + rng.normal(0.0, 0.003, n)   # [rad/s] BMI270-scale noise

    # Write a "stream"-format Data Stream CSV (same header shape as `sf log
    # wifi -o *.csv`): only the test axis carries nonzero rate_ref/gyro,
    # total_thrust is a constant in-flight value (> the 0.3 flight threshold).
    # "stream" 形式の Data Stream CSV を書き出す（`sf log wifi -o *.csv` と
    # 同じヘッダ形状）: テスト対象軸のみ rate_ref/gyro を非ゼロにし、
    # total_thrust は飛行中を示す定数値（飛行判定閾値 0.3 を超える）にする。
    fieldnames = (['timestamp_us']
                  + list(_STREAM_GYRO_COL.values())
                  + list(_STREAM_TARGET_COL.values())
                  + ['total_thrust'])
    gyro_col = _STREAM_GYRO_COL[axis]
    target_col = _STREAM_TARGET_COL[axis]

    fd, csv_path = tempfile.mkstemp(suffix='.csv', prefix='plant_fit_selftest_')
    try:
        with os.fdopen(fd, 'w', newline='') as f:
            writer = _csv.writer(f)
            writer.writerow(fieldnames)
            for i in range(n):
                row = {name: 0.0 for name in fieldnames}
                row['timestamp_us'] = t[i] * 1e6
                row[gyro_col] = gyro_meas[i]
                row[target_col] = target[i]
                row['total_thrust'] = 0.4
                writer.writerow([row[name] for name in fieldnames])

        result = fit_plant(csv_path, axis=axis, kp=kp, rate_max=1.0, fs=fs)
    finally:
        os.unlink(csv_path)

    K_err = abs(result.K / K_true - 1.0)
    tau_err = abs(result.tau_m / tau_m_true - 1.0)
    ok = K_err < 0.15 and tau_err < 0.30 and result.r_squared > 0.9

    if verbose:
        print(f"true : K={K_true:.1f} [rad/s^2/duty]  tau_m={tau_m_true * 1000:.1f} ms")
        print(f"fit  : K={result.K:.1f} ({K_err * 100:.1f}% err)  "
              f"tau_m={result.tau_m * 1000:.1f} ms ({tau_err * 100:.1f}% err)  "
              f"R^2={result.r_squared:.3f}  n_segments={result.n_segments}")
        print("SELFTEST:", "PASS" if ok else "FAIL")

    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)
