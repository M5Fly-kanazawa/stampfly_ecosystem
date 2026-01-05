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
PMW3901 Optical Flow Sensor Model
PMW3901 オプティカルフローセンサモデル

Simulates PMW3901MB-TXQT optical flow sensor with realistic noise characteristics.
PMW3901MB-TXQTオプティカルフローセンサを現実的なノイズ特性でシミュレート

Reference: PMW3901MB-TXQT Datasheet, PixArt Imaging Inc.
"""

import numpy as np


class OpticalFlow:
    """
    PMW3901 Optical Flow Sensor Simulation Model
    PMW3901 オプティカルフローセンサシミュレーションモデル

    The PMW3901 measures motion by tracking surface features.
    It outputs pixel displacement (delta_x, delta_y) and surface quality (squal).
    PMW3901は表面特徴を追跡して動きを計測する。
    ピクセル変位（delta_x, delta_y）と表面品質（squal）を出力。

    Attributes:
        focal_length_px: Focal length in pixels
                        焦点距離（ピクセル）
        scaler: Flow to velocity scaling factor
               フローから速度への変換係数
        noise_std: Measurement noise standard deviation (pixels)
                  測定ノイズ標準偏差（ピクセル）
        min_height: Minimum valid height (m)
                   最小有効高度（m）
        max_height: Maximum valid height (m)
                   最大有効高度（m）
    """

    # PMW3901 specifications (from datasheet)
    # PMW3901仕様（データシートより）
    FRAME_SIZE = 35  # 35x35 pixel sensor
    MAX_DELTA = 127  # Maximum delta per frame (±127)
    SQUAL_MIN = 0    # Minimum surface quality
    SQUAL_MAX = 255  # Maximum surface quality

    # Typical operating parameters
    # 代表的な動作パラメータ
    DEFAULT_FOCAL_LENGTH_PX = 35.0  # Approximate focal length in pixels
    DEFAULT_FOV_DEG = 42.0  # Field of view (degrees)

    # Height limits for reliable operation
    # 信頼性のある動作のための高度制限
    MIN_HEIGHT_M = 0.08  # 80mm minimum (from datasheet)
    MAX_HEIGHT_M = 2.5   # ~2.5m maximum for typical surfaces

    def __init__(
        self,
        focal_length_px: float = DEFAULT_FOCAL_LENGTH_PX,
        flow_noise_std: float = 0.5,
        squal_noise_std: float = 10.0,
        min_height: float = MIN_HEIGHT_M,
        max_height: float = MAX_HEIGHT_M,
        frame_rate_hz: float = 121.0,
    ):
        """
        Initialize optical flow sensor model.
        オプティカルフローセンサモデルを初期化

        Args:
            focal_length_px: Focal length in pixels (affects flow-velocity scaling)
                            焦点距離（ピクセル）（フロー-速度変換に影響）
            flow_noise_std: Flow measurement noise std dev (pixels)
                           フロー測定ノイズ標準偏差（ピクセル）
            squal_noise_std: Surface quality noise std dev
                            表面品質ノイズ標準偏差
            min_height: Minimum valid height (m)
                       最小有効高度（m）
            max_height: Maximum valid height (m)
                       最大有効高度（m）
            frame_rate_hz: Sensor frame rate (Hz)
                          センサフレームレート（Hz）
        """
        self.focal_length_px = focal_length_px
        self.flow_noise_std = flow_noise_std
        self.squal_noise_std = squal_noise_std
        self.min_height = min_height
        self.max_height = max_height
        self.frame_rate_hz = frame_rate_hz

        # Derived parameters
        # 導出パラメータ
        self.dt = 1.0 / frame_rate_hz

        # Internal state for integration
        # 積算用内部状態
        self._accumulated_x = 0.0
        self._accumulated_y = 0.0
        self._last_velocity = np.array([0.0, 0.0])

    def velocity_to_flow(
        self,
        velocity_body: np.ndarray,
        angular_velocity_body: np.ndarray,
        height_m: float,
    ) -> tuple:
        """
        Convert body-frame velocity to optical flow (pixel displacement).
        機体座標系速度をオプティカルフロー（ピクセル変位）に変換

        The optical flow equation:
        オプティカルフロー方程式：
            flow_x = (v_x / h) * f * dt + omega_y * f * dt
            flow_y = (v_y / h) * f * dt - omega_x * f * dt

        Where:
            v_x, v_y: Body-frame horizontal velocities (m/s)
            h: Height above ground (m)
            f: Focal length (pixels)
            omega_x, omega_y: Angular velocities (rad/s)

        Args:
            velocity_body: Body-frame velocity [vx, vy, vz] (m/s)
                          機体座標系速度 [vx, vy, vz] (m/s)
                          Note: x=forward, y=right, z=down (body NED)
            angular_velocity_body: Body-frame angular velocity [p, q, r] (rad/s)
                                  機体座標系角速度 [p, q, r] (rad/s)
            height_m: Height above ground (m)
                     地面からの高度（m）

        Returns:
            Tuple of (delta_x, delta_y) in pixels
            ピクセル単位の (delta_x, delta_y) タプル
        """
        # Clamp height to valid range
        # 高度を有効範囲に制限
        h = np.clip(height_m, self.min_height, self.max_height)

        # Extract velocities and angular rates
        # 速度と角速度を抽出
        vx = velocity_body[0]  # Forward velocity
        vy = velocity_body[1]  # Rightward velocity
        p = angular_velocity_body[0]  # Roll rate
        q = angular_velocity_body[1]  # Pitch rate

        # Compute optical flow components
        # オプティカルフロー成分を計算
        # Translational component + rotational component
        # 並進成分 + 回転成分
        flow_x = (vx / h) * self.focal_length_px + q * self.focal_length_px
        flow_y = (vy / h) * self.focal_length_px - p * self.focal_length_px

        # Convert to per-frame displacement (multiply by dt)
        # フレーム毎の変位に変換（dtを掛ける）
        delta_x = flow_x * self.dt
        delta_y = flow_y * self.dt

        return delta_x, delta_y

    def compute_squal(self, height_m: float, surface_texture: float = 1.0) -> int:
        """
        Compute surface quality value based on conditions.
        状況に基づいて表面品質値を計算

        Surface quality depends on:
        - Height (too high = low quality)
        - Surface texture (smooth = low quality)
        - Lighting conditions

        表面品質は以下に依存：
        - 高度（高すぎると低品質）
        - 表面テクスチャ（滑らかだと低品質）
        - 照明条件

        Args:
            height_m: Height above ground (m)
                     地面からの高度（m）
            surface_texture: Texture quality factor (0-1)
                            テクスチャ品質係数（0-1）

        Returns:
            Surface quality value (0-255)
            表面品質値（0-255）
        """
        # Height-based quality reduction
        # 高度に基づく品質低下
        if height_m < self.min_height:
            height_factor = 0.0
        elif height_m > self.max_height:
            height_factor = 0.0
        else:
            # Quality peaks at around 0.3-0.5m
            # 品質は0.3-0.5mでピーク
            optimal_height = 0.4
            height_factor = np.exp(-((height_m - optimal_height) / 0.8) ** 2)

        # Combine factors
        # 係数を組み合わせ
        base_squal = 180  # Typical good quality value
        squal = base_squal * height_factor * surface_texture

        # Add noise
        # ノイズを追加
        squal += np.random.normal(0, self.squal_noise_std)

        # Clamp to valid range
        # 有効範囲に制限
        return int(np.clip(squal, self.SQUAL_MIN, self.SQUAL_MAX))

    def read(
        self,
        velocity_body: np.ndarray,
        angular_velocity_body: np.ndarray,
        height_m: float,
        surface_texture: float = 1.0,
    ) -> dict:
        """
        Simulate sensor reading with noise.
        ノイズを含むセンサ読み取りをシミュレート

        Args:
            velocity_body: Body-frame velocity [vx, vy, vz] (m/s)
                          機体座標系速度 [vx, vy, vz] (m/s)
            angular_velocity_body: Body-frame angular velocity [p, q, r] (rad/s)
                                  機体座標系角速度 [p, q, r] (rad/s)
            height_m: Height above ground (m)
                     地面からの高度（m）
            surface_texture: Texture quality factor (0-1)
                            テクスチャ品質係数（0-1）

        Returns:
            dict with delta_x, delta_y, squal, motion_detected
            delta_x, delta_y, squal, motion_detected を含む辞書
        """
        # Compute ideal flow
        # 理想的なフローを計算
        delta_x, delta_y = self.velocity_to_flow(
            velocity_body, angular_velocity_body, height_m
        )

        # Add measurement noise
        # 測定ノイズを追加
        delta_x += np.random.normal(0, self.flow_noise_std)
        delta_y += np.random.normal(0, self.flow_noise_std)

        # Quantize to integer (sensor output is 16-bit signed)
        # 整数に量子化（センサ出力は16ビット符号付き）
        delta_x_int = int(np.clip(np.round(delta_x), -32768, 32767))
        delta_y_int = int(np.clip(np.round(delta_y), -32768, 32767))

        # Compute surface quality
        # 表面品質を計算
        squal = self.compute_squal(height_m, surface_texture)

        # Motion detection (based on flow magnitude and quality)
        # モーション検出（フロー大きさと品質に基づく）
        motion_magnitude = np.sqrt(delta_x**2 + delta_y**2)
        motion_detected = squal > 20 and motion_magnitude > 0.1

        # Update internal state
        # 内部状態を更新
        self._accumulated_x += delta_x
        self._accumulated_y += delta_y
        self._last_velocity = velocity_body[:2].copy()

        return {
            'delta_x': delta_x_int,
            'delta_y': delta_y_int,
            'squal': squal,
            'motion_detected': motion_detected,
            'shutter': self._estimate_shutter(height_m),
        }

    def _estimate_shutter(self, height_m: float) -> int:
        """
        Estimate shutter value based on conditions.
        状況に基づいてシャッター値を推定

        Higher shutter = longer exposure = lower light or farther distance

        Args:
            height_m: Height above ground (m)

        Returns:
            Shutter value (16-bit unsigned)
        """
        # Base shutter increases with height (less light reflected)
        # 基本シャッターは高度とともに増加（反射光が減少）
        base_shutter = 500 + int(height_m * 200)
        noise = np.random.randint(-50, 50)
        return int(np.clip(base_shutter + noise, 0, 65535))

    def reset(self):
        """
        Reset accumulated values.
        累積値をリセット
        """
        self._accumulated_x = 0.0
        self._accumulated_y = 0.0
        self._last_velocity = np.array([0.0, 0.0])

    def flow_to_velocity(
        self,
        delta_x: int,
        delta_y: int,
        angular_velocity_body: np.ndarray,
        height_m: float,
    ) -> np.ndarray:
        """
        Convert optical flow to velocity (inverse of velocity_to_flow).
        オプティカルフローを速度に変換（velocity_to_flowの逆変換）

        This is what the ESKF would do to estimate velocity from flow.
        これはESKFがフローから速度を推定する際に行う処理。

        Args:
            delta_x: X displacement (pixels per frame)
            delta_y: Y displacement (pixels per frame)
            angular_velocity_body: Body-frame angular velocity [p, q, r] (rad/s)
            height_m: Height above ground (m)

        Returns:
            Estimated body-frame velocity [vx, vy] (m/s)
            推定機体座標系速度 [vx, vy] (m/s)
        """
        h = np.clip(height_m, self.min_height, self.max_height)
        p = angular_velocity_body[0]
        q = angular_velocity_body[1]

        # Convert from per-frame to rate
        # フレーム毎からレートに変換
        flow_x = delta_x / self.dt
        flow_y = delta_y / self.dt

        # Remove rotational component
        # 回転成分を除去
        flow_x_trans = flow_x - q * self.focal_length_px
        flow_y_trans = flow_y + p * self.focal_length_px

        # Convert to velocity
        # 速度に変換
        vx = (flow_x_trans / self.focal_length_px) * h
        vy = (flow_y_trans / self.focal_length_px) * h

        return np.array([vx, vy])
