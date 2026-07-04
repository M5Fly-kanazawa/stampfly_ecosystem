/**
 * @file eskf.hpp
 * @brief Error-State Kalman Filter with active_mask based P-matrix isolation
 *
 * active_mask に基づく状態管理を持つ ESKF
 *
 * Features / 主要機能:
 * - Unified sensor enable/disable via SensorGroup + active_mask
 *   SensorGroup + active_mask による統一的なセンサ有効/無効制御
 * - P-matrix isolation: frozen states have zero cross-covariance
 *   P行列隔離: 凍結状態のクロス共分散をゼロに
 * - Q gating: no process noise added to frozen states
 *   Qゲーティング: 凍結状態にプロセスノイズを加算しない
 * - dx masking: unified masking for bias update control
 *   dxマスキング: 統一マスクによるバイアス更新制御
 *
 * 15-state ESKF:
 * [0-2]  POS_X/Y/Z    Position [m] NED / 位置
 * [3-5]  VEL_X/Y/Z    Velocity [m/s] NED / 速度
 * [6-8]  ATT_X/Y/Z    Attitude error [rad] / 姿勢誤差
 * [9-11] BG_X/Y/Z     Gyro bias [rad/s] / ジャイロバイアス
 * [12-14]BA_X/Y/Z     Accel bias [m/s²] / 加速度バイアス
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "stampfly_math.hpp"

namespace stampfly {

using namespace math;

/**
 * @brief ESKF - active_mask based state management
 */
class ESKF {
public:
    // ========================================================================
    // Sensor groups and state masks
    // センサグループと状態マスク
    // ========================================================================

    /**
     * @brief Sensor groups that can be independently enabled/disabled
     * 独立に有効/無効を切り替えられるセンサグループ
     */
    enum SensorGroup {
        SENSOR_MAG  = 0,    // Magnetometer / 地磁気
        SENSOR_BARO = 1,    // Barometer / 気圧計
        SENSOR_TOF  = 2,    // Time-of-Flight / ToFセンサ
        SENSOR_FLOW = 3,    // Optical flow / オプティカルフロー
        SENSOR_COUNT = 4
    };

    // State indices
    // 状態インデックス
    enum StateIdx {
        POS_X = 0, POS_Y = 1, POS_Z = 2,
        VEL_X = 3, VEL_Y = 4, VEL_Z = 5,
        ATT_X = 6, ATT_Y = 7, ATT_Z = 8,
        BG_X  = 9, BG_Y = 10, BG_Z = 11,
        BA_X = 12, BA_Y = 13, BA_Z = 14
    };

    static constexpr int N_STATES = 15;

    // Frozen state masks per sensor group
    // 各センサグループがOFFの時に凍結する状態ビットマスク
    //
    // Sensor OFF → corresponding states become unobservable → freeze them
    // センサOFF → 対応する状態が不可観測 → 凍結
    static constexpr uint16_t MASK_MAG  = (1u << ATT_Z) | (1u << BG_Z);            // 0x0900
    static constexpr uint16_t MASK_BARO = (1u << POS_Z) | (1u << VEL_Z) | (1u << BA_Z);  // 0x4024
    static constexpr uint16_t MASK_TOF  = (1u << POS_Z) | (1u << VEL_Z) | (1u << BA_Z);  // 0x4024
    static constexpr uint16_t MASK_FLOW = (1u << POS_X) | (1u << POS_Y) |
                                          (1u << VEL_X) | (1u << VEL_Y);          // 0x0018|0x0003=0x001B
    // BA_X/BA_Y removed: PMW3901 resolution (1px=8.6cm/s) is too coarse
    // to observe accel bias (0.01 m/s²). Bias estimated via accel attitude only.
    // BA_X/BA_Y除去: PMW3901の分解能ではバイアス推定不可、重力ベクトルのみで推定

    // Mask lookup table indexed by SensorGroup
    // SensorGroupでインデックスされるマスクテーブル
    static constexpr uint16_t SENSOR_MASKS[SENSOR_COUNT] = {
        MASK_MAG, MASK_BARO, MASK_TOF, MASK_FLOW
    };

    // ========================================================================
    // Config
    // ========================================================================

    struct Config {
        // Process noise (Q components)
        // プロセスノイズ
        float gyro_noise;           // [rad/s/√Hz]
        float accel_noise;          // [m/s²/√Hz]
        float gyro_bias_noise;      // Gyro bias random walk / ジャイロバイアスランダムウォーク
        float accel_bias_noise;     // Accel bias random walk / 加速度バイアスランダムウォーク

        // Measurement noise (R components)
        // 観測ノイズ
        float baro_noise;           // [m]
        float tof_noise;            // [m]
        float mag_noise;            // [uT]
        float flow_noise;           // [m/s]
        float accel_att_noise;      // [m/s²]

        // Initial covariance std
        // 初期共分散標準偏差
        float init_pos_std;         // [m]
        float init_vel_std;         // [m/s]
        float init_att_std;         // [rad]
        float init_gyro_bias_std;   // [rad/s]
        float init_accel_bias_std;  // [m/s²]

        // Magnetic reference vector (NED)
        // 地磁気参照ベクトル
        Vector3 mag_ref;

        // Gravity
        // 重力加速度
        float gravity;

        // Outlier rejection threshold (squared Mahalanobis distance)
        // アウトライア棄却閾値
        float mahalanobis_threshold;

        // ToF tilt threshold [rad]
        // ToF傾き閾値
        float tof_tilt_threshold;

        // Innovation gates for position sensors (absolute, 0=disabled) [m]
        // 位置センサ用イノベーションゲート（絶対値、0=無効）
        // P-collapse prevents chi2 gates from working with position sensors
        // P崩壊によりχ²ゲートは位置センサで機能しないため絶対値ゲートを使用
        float baro_innov_gate;      // Baro max |innovation| [m]
        float tof_innov_gate;       // ToF max |innovation| [m]

        // Chi-squared gates for attitude sensors (0=disabled)
        // 姿勢センサ用カイ二乗ゲート (0=無効)
        // 2 DOF: 5.99=95%, 9.21=99%
        // 3 DOF: 7.81=95%, 11.34=99%
        float mag_chi2_gate;        // Mag (3 DOF)
        float flow_chi2_gate;       // Flow (2 DOF)
        float flow_innov_clamp;    // Flow innovation clamp [m/s] (0=disabled)
        float accel_att_chi2_gate;  // Accel attitude (3 DOF)

        // Accel attitude correction motion threshold [m/s²]
        // 加速度計姿勢補正のモーション閾値
        float accel_motion_threshold;

        // Optical flow height limits [m]
        // オプティカルフロー高度範囲
        float flow_min_height;
        float flow_max_height;

        // Flow tilt threshold (cos(tilt) lower bound)
        // フロー傾き閾値
        float flow_tilt_cos_threshold;

        // Optical flow calibration
        // オプティカルフローキャリブレーション
        float flow_rad_per_pixel;
        float flow_cam_to_body[4];  // {c2b_xx, c2b_xy, c2b_yx, c2b_yy}
        float flow_gyro_scale;
        float flow_offset[2];      // {dx_offset, dy_offset}

        // Sensor enable flags (replaces individual flags)
        // センサ有効フラグ (個別フラグを統一)
        bool sensor_enabled[SENSOR_COUNT];

        // Yaw estimation enable
        // ヨー推定有効フラグ
        bool yaw_estimation_enabled;

        // Adaptive R scaling coefficient for accel attitude correction
        // 適応的Rスケーリング係数 (加速度計姿勢補正)
        float k_adaptive;

        // Attitude correction clamp [rad]
        // クロス共分散経由の姿勢ジャンプ防止クランプ
        float att_correction_clamp;

        // Magnetometer norm valid range [uT]
        // 地磁気ノルム有効範囲
        float mag_norm_min;
        float mag_norm_max;

        /**
         * @brief Default configuration
         * デフォルト設定
         */
        static Config defaultConfig() {
            Config cfg;
            // Process noise (Q)
            cfg.gyro_noise = 0.009655f;
            cfg.accel_noise = 0.062885f;
            cfg.gyro_bias_noise = 0.000013f;
            cfg.accel_bias_noise = 0.0001f;

            // Measurement noise (R)
            cfg.baro_noise = 0.1f;
            cfg.tof_noise = 0.002540f;
            cfg.mag_noise = 2.0f;
            cfg.flow_noise = 0.005232f;
            cfg.accel_att_noise = 0.02f;

            // Initial covariance
            cfg.init_pos_std = 1.0f;
            cfg.init_vel_std = 0.5f;
            cfg.init_att_std = 0.1f;
            cfg.init_gyro_bias_std = 0.01f;
            cfg.init_accel_bias_std = 0.1f;

            // References
            cfg.mag_ref = Vector3(20.0f, 0.0f, 40.0f);
            cfg.gravity = 9.81f;

            // Thresholds
            cfg.mahalanobis_threshold = 15.0f;
            cfg.tof_tilt_threshold = 0.70f;
            cfg.accel_motion_threshold = 1.0f;

            // Innovation gates for position sensors [m] (0 = disabled)
            cfg.baro_innov_gate = 0.5f;       // [m]
            cfg.tof_innov_gate = 0.5f;        // [m] (normal P99.5 ≈ 10cm)

            // Chi-squared gates for attitude sensors (0 = disabled)
            cfg.mag_chi2_gate = 7.81f;        // 3 DOF, 95%
            cfg.flow_chi2_gate = 5.99f;       // 2 DOF, 95%
            cfg.flow_innov_clamp = 0.0f;      // disabled by default
            cfg.accel_att_chi2_gate = 7.81f;  // 3 DOF, 95%
            cfg.flow_min_height = 0.02f;
            cfg.flow_max_height = 4.0f;
            cfg.flow_tilt_cos_threshold = 0.866f;

            // Flow calibration
            cfg.flow_rad_per_pixel = 0.00222f;
            cfg.flow_cam_to_body[0] = 0.943f;
            cfg.flow_cam_to_body[1] = 0.0f;
            cfg.flow_cam_to_body[2] = 0.0f;
            cfg.flow_cam_to_body[3] = 1.015f;
            cfg.flow_gyro_scale = 1.0f;
            cfg.flow_offset[0] = 0.0f;
            cfg.flow_offset[1] = 0.0f;

            // Sensor enables (default: Mag OFF due to motor interference)
            cfg.sensor_enabled[SENSOR_MAG]  = false;
            cfg.sensor_enabled[SENSOR_BARO] = true;
            cfg.sensor_enabled[SENSOR_TOF]  = true;
            cfg.sensor_enabled[SENSOR_FLOW] = true;

            cfg.yaw_estimation_enabled = true;

            // Attitude correction
            cfg.k_adaptive = 0.0f;
            cfg.att_correction_clamp = 0.05f;  // ±0.05 rad (~2.9 deg)

            // Magnetometer norm range
            cfg.mag_norm_min = 10.0f;   // [uT]
            cfg.mag_norm_max = 100.0f;  // [uT]

            return cfg;
        }
    };

    // State structure
    // 状態構造体
    struct State {
        Vector3 position;           // [m] NED
        Vector3 velocity;           // [m/s]
        Quaternion orientation;     // Attitude quaternion / 姿勢クォータニオン
        Vector3 gyro_bias;          // [rad/s]
        Vector3 accel_bias;         // [m/s²]

        // Euler angles (convenience)
        // オイラー角 (便利用)
        float roll;                 // [rad]
        float pitch;                // [rad]
        float yaw;                  // [rad]
    };

    ESKF() = default;

    // ========================================================================
    // Initialization / Reset
    // 初期化 / リセット
    // ========================================================================

    esp_err_t init(const Config& config);
    void reset();
    void resetPositionVelocity();
    void holdPositionVelocity();
    void resetForLanding();

    // ========================================================================
    // Sensor enable/disable (dynamic)
    // センサ有効/無効 (動的切替)
    // ========================================================================

    /**
     * @brief Enable or disable a sensor group
     * センサグループの有効/無効を設定
     *
     * When disabled, the corresponding states are frozen:
     * 無効にすると対応する状態が凍結される:
     * - P-matrix cross-covariance zeroed / P行列クロス共分散ゼロ化
     * - No Q noise added in predict / predictでQ加算なし
     * - dx masked to zero in updates / 観測更新でdxをゼロマスク
     */
    void setSensorEnabled(SensorGroup group, bool enabled);

    /**
     * @brief Check if a sensor group is enabled
     * センサグループが有効かチェック
     */
    bool isSensorEnabled(SensorGroup group) const {
        return (group < SENSOR_COUNT) && config_.sensor_enabled[group];
    }

    /**
     * @brief Get current active mask
     * 現在のactive_maskを取得
     */
    uint16_t getActiveMask() const { return active_mask_; }

    // ========================================================================
    // Predict
    // 予測ステップ
    // ========================================================================

    void predict(const Vector3& accel, const Vector3& gyro, float dt, bool skip_position = false);

    // ========================================================================
    // Measurement updates
    // 観測更新
    // ========================================================================

    void updateAccelAttitude(const Vector3& accel);
    void updateBaro(float altitude);
    void updateToF(float distance);
    void updateMag(const Vector3& mag);
    void updateFlowRaw(int16_t flow_dx, int16_t flow_dy, float distance,
                       float dt, float gyro_x, float gyro_y);

    // ========================================================================
    // State access
    // 状態アクセス
    // ========================================================================

    State getState() const { return state_; }

    float getPositionVariance() const {
        return P_(0, 0) + P_(1, 1) + P_(2, 2);
    }

    float getVelocityVariance() const {
        return P_(3, 3) + P_(4, 4) + P_(5, 5);
    }

    const Matrix<15, 15>& getCovariance() const { return P_; }
    Matrix<15, 15>& getCovariance() { return P_; }

    Vector3 getMagReference() const { return config_.mag_ref; }
    bool isInitialized() const { return initialized_; }

    // ========================================================================
    // Bias / attitude setters
    // バイアス / 姿勢設定
    // ========================================================================

    void setGyroBias(const Vector3& bias);
    void setAccelBias(const Vector3& bias);
    void setMagReference(const Vector3& mag_ref);
    void initializeAttitude(const Vector3& accel, const Vector3& mag);
    void setAttitudeReference(const Vector3& level_accel, const Vector3& gyro_bias);

    void setFreezeAccelBias(bool freeze) {
        freeze_accel_bias_ = freeze;
        recomputeActiveMask();
        enforceCovarianceConstraints();
    }
    bool isAccelBiasFrozen() const { return freeze_accel_bias_; }

private:
    bool initialized_ = false;
    bool freeze_accel_bias_ = true;
    Config config_;
    State state_;

    // Active state mask: bit i = 1 means state i is active (updated)
    // アクティブ状態マスク: ビットi=1 はその状態がアクティブ(更新される)
    uint16_t active_mask_ = 0x7FFF;  // All 15 bits ON / 全15ビットON

    // Cached initial P diagonal for enforcement
    // enforcementのためにキャッシュされたP行列初期対角値
    float init_P_diag_[N_STATES];

    // Covariance matrix (15x15)
    // 共分散行列
    Matrix<15, 15> P_;

    // Temp matrices (avoid stack overflow on ESP32)
    // 一時行列 (ESP32のスタックオーバーフロー回避)
    Matrix<15, 15> temp1_;

    /**
     * @brief Recompute active_mask from sensor_enabled[] and freeze_accel_bias_
     * sensor_enabled[] と freeze_accel_bias_ から active_mask を再計算
     */
    void recomputeActiveMask();

    /**
     * @brief Enforce P-matrix constraints for frozen states
     * 凍結状態のP行列制約を強制
     *
     * For each frozen state i:
     *   P(i,i) = init_P_diag_[i]
     *   P(i,j) = P(j,i) = 0  for all j != i
     * For all states:
     *   Symmetry: P(i,j) = P(j,i) = average
     *   Diagonal floor: P(i,i) >= 1e-12
     *
     * Called after every predict() and update().
     * 全てのpredict()とupdate()の後に呼び出される。
     */
    void enforceCovarianceConstraints();

    /**
     * @brief Apply dx masking and inject error state
     * dxマスキングを適用してエラー状態を注入
     *
     * Frozen states: dx[i] = 0
     * freeze_accel_bias_: dx[BA_*] = 0
     */
    void applyMaskedErrorState(float dx[N_STATES]);
};

}  // namespace stampfly
