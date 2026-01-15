# MIT License
#
# Copyright (c) 2025 Kouhei Ito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Sensor Noise Models
センサノイズモデル

Provides realistic noise models for IMU and other sensors based on
Allan variance analysis and datasheet specifications.
Allan分散解析およびデータシート仕様に基づくIMU等のセンサの
リアリスティックなノイズモデルを提供。

References:
- IEEE Std 647-2006: IEEE Standard Specification Format Guide and Test
  Procedure for Single-Axis Interferometric Fiber Optic Gyros
- BMI270 Datasheet, Bosch Sensortec
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class AllanVarianceParams:
    """
    Allan Variance parameters for sensor noise characterization.
    センサノイズ特性評価のためのAllan分散パラメータ

    The Allan variance decomposes sensor noise into:
    - Quantization noise (Q): From analog-to-digital conversion
    - Angle/Velocity Random Walk (N): White noise on rate
    - Bias Instability (B): Low-frequency bias fluctuations
    - Rate Random Walk (K): Integrated white noise on bias
    - Rate Ramp (R): Deterministic drift

    Allan分散はセンサノイズを以下に分解：
    - 量子化ノイズ（Q）：A/D変換由来
    - 角度/速度ランダムウォーク（N）：レート上の白色ノイズ
    - バイアス不安定性（B）：低周波バイアス変動
    - レートランダムウォーク（K）：バイアス上の積分白色ノイズ
    - レートランプ（R）：決定論的ドリフト

    For gyroscope:
        N: deg/sqrt(hr) or rad/sqrt(s)
        B: deg/hr or rad/s
        K: deg/hr/sqrt(hr) or rad/s/sqrt(s)

    For accelerometer:
        N: m/s/sqrt(hr) or m/s^2/sqrt(s)
        B: m/s^2
        K: m/s^2/sqrt(s)
    """
    quantization_noise: float = 0.0  # Q
    random_walk: float = 0.0         # N (ARW for gyro, VRW for accel)
    bias_instability: float = 0.0    # B
    rate_random_walk: float = 0.0    # K
    rate_ramp: float = 0.0           # R


@dataclass
class BMI270_GyroParams:
    """
    BMI270 Gyroscope Allan Variance Parameters (from datasheet)
    BMI270 ジャイロスコープAllan分散パラメータ（データシートより）

    Typical values at ±2000 dps range, ODR 400Hz, normal mode
    ±2000 dpsレンジ、ODR 400Hz、通常モードでの代表値
    """
    # Noise density: 0.007 deg/s/sqrt(Hz) = 0.007 * sqrt(400) = 0.14 deg/s RMS
    # ノイズ密度：0.007 deg/s/√Hz = 0.007 * √400 = 0.14 deg/s RMS
    noise_density_dps_sqrthz: float = 0.007

    # Zero-rate offset (typical): ±1 deg/s
    # ゼロレートオフセット（代表値）：±1 deg/s
    zero_rate_offset_dps: float = 1.0

    # Zero-rate offset change vs temperature: 0.015 deg/s/K
    # ゼロレートオフセットの温度依存性：0.015 deg/s/K
    offset_temp_coeff_dps_per_k: float = 0.015

    def to_allan_params(self, odr_hz: float = 400.0) -> AllanVarianceParams:
        """Convert to Allan variance parameters"""
        # ARW (Angle Random Walk) = noise_density * sqrt(dt)
        # For deg/sqrt(hr): multiply by sqrt(3600)
        arw_deg_sqrt_hr = self.noise_density_dps_sqrthz * np.sqrt(3600)

        # Convert to rad/sqrt(s) for simulation
        arw_rad_sqrt_s = np.deg2rad(self.noise_density_dps_sqrthz)

        # Bias instability (estimate from offset stability)
        # バイアス不安定性（オフセット安定性から推定）
        bias_instability = np.deg2rad(self.zero_rate_offset_dps * 0.1)  # 10% of offset

        return AllanVarianceParams(
            random_walk=arw_rad_sqrt_s,
            bias_instability=bias_instability,
            rate_random_walk=arw_rad_sqrt_s * 0.01,  # Typical ratio
        )


@dataclass
class BMI270_AccelParams:
    """
    BMI270 Accelerometer Allan Variance Parameters (from datasheet)
    BMI270 加速度センサAllan分散パラメータ（データシートより）

    Typical values at ±8g range, ODR 400Hz, normal mode
    ±8gレンジ、ODR 400Hz、通常モードでの代表値
    """
    # Noise density: 120 µg/sqrt(Hz)
    # ノイズ密度：120 µg/√Hz
    noise_density_ug_sqrthz: float = 120.0

    # Zero-g offset (typical): ±20 mg
    # ゼロgオフセット（代表値）：±20 mg
    zero_g_offset_mg: float = 20.0

    # Offset temperature coefficient: ±0.2 mg/K
    # オフセット温度係数：±0.2 mg/K
    offset_temp_coeff_mg_per_k: float = 0.2

    def to_allan_params(self, odr_hz: float = 400.0) -> AllanVarianceParams:
        """Convert to Allan variance parameters"""
        # VRW (Velocity Random Walk) = noise_density * g * sqrt(dt)
        g = 9.80665
        vrw_m_s2_sqrt_s = (self.noise_density_ug_sqrthz * 1e-6) * g

        # Bias instability
        bias_instability = (self.zero_g_offset_mg * 1e-3) * g * 0.1

        return AllanVarianceParams(
            random_walk=vrw_m_s2_sqrt_s,
            bias_instability=bias_instability,
            rate_random_walk=vrw_m_s2_sqrt_s * 0.01,
        )


class IMUNoiseGenerator:
    """
    IMU noise generator using Allan variance model.
    Allan分散モデルを使用するIMUノイズ生成器

    Generates realistic IMU noise including:
    - White noise (from random walk parameter)
    - Bias drift (from bias instability and rate random walk)
    - Temperature effects (optional)

    以下を含むリアリスティックなIMUノイズを生成：
    - 白色ノイズ（ランダムウォークパラメータより）
    - バイアスドリフト（バイアス不安定性とレートランダムウォークより）
    - 温度効果（オプション）
    """

    def __init__(
        self,
        gyro_params: Optional[AllanVarianceParams] = None,
        accel_params: Optional[AllanVarianceParams] = None,
        sample_rate_hz: float = 400.0,
    ):
        """
        Initialize IMU noise generator.
        IMUノイズ生成器を初期化

        Args:
            gyro_params: Gyroscope Allan variance parameters
                        ジャイロスコープAllan分散パラメータ
            accel_params: Accelerometer Allan variance parameters
                         加速度センサAllan分散パラメータ
            sample_rate_hz: Sensor sample rate (Hz)
                           センササンプルレート（Hz）
        """
        # Use BMI270 defaults if not specified
        # 指定がなければBMI270デフォルトを使用
        if gyro_params is None:
            gyro_params = BMI270_GyroParams().to_allan_params(sample_rate_hz)
        if accel_params is None:
            accel_params = BMI270_AccelParams().to_allan_params(sample_rate_hz)

        self.gyro_params = gyro_params
        self.accel_params = accel_params
        self.dt = 1.0 / sample_rate_hz
        self.sqrt_dt = np.sqrt(self.dt)

        # Bias state (for correlated noise simulation)
        # バイアス状態（相関ノイズシミュレーション用）
        self._gyro_bias = np.zeros(3)
        self._accel_bias = np.zeros(3)

        # Initialize with small random bias
        # 小さいランダムバイアスで初期化
        self._gyro_bias = np.random.normal(
            0, self.gyro_params.bias_instability, 3
        )
        self._accel_bias = np.random.normal(
            0, self.accel_params.bias_instability, 3
        )

    def generate_gyro_noise(self) -> np.ndarray:
        """
        Generate gyroscope noise for one sample.
        1サンプル分のジャイロスコープノイズを生成

        Returns:
            3D noise vector [noise_x, noise_y, noise_z] (rad/s)
            3Dノイズベクトル [noise_x, noise_y, noise_z] (rad/s)
        """
        # White noise (Angle Random Walk)
        # 白色ノイズ（角度ランダムウォーク）
        white_noise = np.random.normal(
            0, self.gyro_params.random_walk / self.sqrt_dt, 3
        )

        # Bias random walk (update bias state)
        # バイアスランダムウォーク（バイアス状態を更新）
        bias_walk = np.random.normal(
            0, self.gyro_params.rate_random_walk * self.sqrt_dt, 3
        )
        self._gyro_bias += bias_walk

        # Limit bias magnitude to prevent runaway
        # バイアスの発散を防ぐため大きさを制限
        max_bias = self.gyro_params.bias_instability * 3
        self._gyro_bias = np.clip(self._gyro_bias, -max_bias, max_bias)

        return white_noise + self._gyro_bias

    def generate_accel_noise(self) -> np.ndarray:
        """
        Generate accelerometer noise for one sample.
        1サンプル分の加速度センサノイズを生成

        Returns:
            3D noise vector [noise_x, noise_y, noise_z] (m/s^2)
            3Dノイズベクトル [noise_x, noise_y, noise_z] (m/s^2)
        """
        # White noise (Velocity Random Walk)
        # 白色ノイズ（速度ランダムウォーク）
        white_noise = np.random.normal(
            0, self.accel_params.random_walk / self.sqrt_dt, 3
        )

        # Bias random walk
        # バイアスランダムウォーク
        bias_walk = np.random.normal(
            0, self.accel_params.rate_random_walk * self.sqrt_dt, 3
        )
        self._accel_bias += bias_walk

        # Limit bias magnitude
        # バイアスの大きさを制限
        max_bias = self.accel_params.bias_instability * 3
        self._accel_bias = np.clip(self._accel_bias, -max_bias, max_bias)

        return white_noise + self._accel_bias

    def reset_bias(self):
        """
        Reset bias states to initial values.
        バイアス状態を初期値にリセット
        """
        self._gyro_bias = np.random.normal(
            0, self.gyro_params.bias_instability, 3
        )
        self._accel_bias = np.random.normal(
            0, self.accel_params.bias_instability, 3
        )

    @property
    def gyro_bias(self) -> np.ndarray:
        """Current gyroscope bias / 現在のジャイロスコープバイアス"""
        return self._gyro_bias.copy()

    @property
    def accel_bias(self) -> np.ndarray:
        """Current accelerometer bias / 現在の加速度センサバイアス"""
        return self._accel_bias.copy()


def compute_allan_variance(
    data: np.ndarray,
    sample_rate_hz: float,
    tau_values: Optional[np.ndarray] = None,
) -> tuple:
    """
    Compute Allan variance from time series data.
    時系列データからAllan分散を計算

    Allan variance is computed as:
    σ²(τ) = 1/(2(N-1)) * Σ(θ_{k+1} - θ_k)²

    where θ_k is the average over the k-th cluster of size τ.

    Args:
        data: 1D time series data (sensor output)
             1D時系列データ（センサ出力）
        sample_rate_hz: Sample rate (Hz)
                       サンプルレート（Hz）
        tau_values: Cluster times to evaluate (s), auto-generated if None
                   評価するクラスタ時間（s）、Noneの場合自動生成

    Returns:
        Tuple of (tau_values, allan_variance, allan_deviation)
        (tau_values, Allan分散, Allan偏差) のタプル
    """
    n = len(data)
    dt = 1.0 / sample_rate_hz

    if tau_values is None:
        # Generate tau values from 1 sample to 1/10 of data length
        # 1サンプルからデータ長の1/10までのtau値を生成
        max_clusters = n // 10
        cluster_sizes = np.logspace(0, np.log10(max_clusters), 50).astype(int)
        cluster_sizes = np.unique(cluster_sizes)
        cluster_sizes = cluster_sizes[cluster_sizes > 0]
        tau_values = cluster_sizes * dt

    allan_var = []
    valid_tau = []

    for tau in tau_values:
        m = int(tau / dt)  # Cluster size in samples
        if m < 1 or 2 * m > n:
            continue

        # Number of clusters
        n_clusters = n // m

        if n_clusters < 2:
            continue

        # Compute cluster averages
        clusters = data[:n_clusters * m].reshape(n_clusters, m).mean(axis=1)

        # Allan variance
        diff = np.diff(clusters)
        avar = 0.5 * np.mean(diff ** 2)

        allan_var.append(avar)
        valid_tau.append(tau)

    allan_var = np.array(allan_var)
    valid_tau = np.array(valid_tau)
    allan_dev = np.sqrt(allan_var)

    return valid_tau, allan_var, allan_dev


def fit_allan_parameters(
    tau: np.ndarray,
    adev: np.ndarray,
) -> AllanVarianceParams:
    """
    Fit Allan variance parameters from Allan deviation curve.
    Allan偏差曲線からAllan分散パラメータをフィット

    Uses characteristic slopes:
    - N (Random Walk): slope -1/2, σ(τ) = N / sqrt(τ)
    - B (Bias Instability): slope 0, minimum of σ(τ)
    - K (Rate Random Walk): slope +1/2, σ(τ) = K * sqrt(τ/3)

    Args:
        tau: Cluster times (s) / クラスタ時間（s）
        adev: Allan deviation values / Allan偏差値

    Returns:
        Fitted Allan variance parameters
        フィットしたAllan分散パラメータ
    """
    # Find minimum (bias instability region)
    # 最小値を見つける（バイアス不安定性領域）
    min_idx = np.argmin(adev)
    bias_instability = adev[min_idx] * 0.664  # Factor for bias instability

    # Random walk: use short-tau region (first few points)
    # ランダムウォーク：短tau領域を使用（最初の数点）
    short_tau_idx = min(3, len(tau) - 1)
    random_walk = adev[0] * np.sqrt(tau[0])

    # Rate random walk: use long-tau region (last few points)
    # レートランダムウォーク：長tau領域を使用（最後の数点）
    if len(tau) > min_idx + 3:
        long_tau_idx = min_idx + 3
        rate_random_walk = adev[long_tau_idx] / np.sqrt(tau[long_tau_idx] / 3)
    else:
        rate_random_walk = 0.0

    return AllanVarianceParams(
        random_walk=random_walk,
        bias_instability=bias_instability,
        rate_random_walk=rate_random_walk,
    )
