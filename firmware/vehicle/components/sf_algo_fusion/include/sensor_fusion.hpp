/**
 * @file sensor_fusion.hpp
 * @brief センサーフュージョン - ESKFを用いた状態推定
 *
 * このコンポーネントはESKFの呼び出しパターンをカプセル化し、
 * 「センサーフュージョンとは何か」を学ぶための入り口を提供する。
 *
 * 設計原則:
 * - FreeRTOS非依存（algo_*レイヤー）
 * - ESKFを内包し、更新タイミングを管理
 * - 発散検出・自動リセット機能
 *
 * 設定:
 * - 全ての設定は main/config.hpp の config::eskf 名前空間で定義
 * - 独自のConfig構造体は持たない（設定の二重管理を防止）
 */

#pragma once

#include "eskf.hpp"
#include "stampfly_math.hpp"

namespace sf {

/**
 * @brief センサーフュージョンクラス
 *
 * 使用例:
 * @code
 * sf::SensorFusion fusion;
 * stampfly::ESKF::Config config;
 * // config を main/config.hpp の値で初期化
 * fusion.init(config);
 *
 * // 400Hzメインループ
 * fusion.predictIMU(accel_body, gyro_body, dt);
 *
 * // 各センサー更新（非同期）
 * if (flow_ready) fusion.updateOpticalFlow(...);
 * if (tof_ready)  fusion.updateToF(...);
 * @endcode
 */
class SensorFusion {
public:
    /**
     * @brief 推定状態
     */
    struct State {
        float roll = 0.0f;           // [rad]
        float pitch = 0.0f;          // [rad]
        float yaw = 0.0f;            // [rad]
        stampfly::math::Vector3 position;   // [m] NED
        stampfly::math::Vector3 velocity;   // [m/s] NED
        stampfly::math::Vector3 gyro_bias;  // [rad/s]
        stampfly::math::Vector3 accel_bias; // [m/s²]
    };

    SensorFusion() = default;
    ~SensorFusion() = default;

    // コピー禁止
    SensorFusion(const SensorFusion&) = delete;
    SensorFusion& operator=(const SensorFusion&) = delete;

    /**
     * @brief センサー有効/無効設定と閾値
     * 全てのセンサースイッチと閾値を一箇所で管理
     */
    struct SensorEnables {
        // 有効/無効スイッチ
        bool optical_flow = true;   // オプティカルフロー
        bool barometer = true;      // 気圧センサー
        bool tof = true;            // ToFセンサー
        bool magnetometer = true;   // 地磁気センサー

        // オプティカルフロー閾値
        uint8_t flow_squal_min = 0x19;    // 最小品質閾値
        float flow_distance_min = 0.02f;  // [m]
        float flow_distance_max = 4.0f;   // [m]

        // ToF距離閾値
        float tof_distance_min = 0.01f;   // [m]
        float tof_distance_max = 4.0f;    // [m]
    };

    /**
     * @brief 初期化
     * @param config ESKF設定（main/config.hpp で構築）
     * @param enables センサー有効/無効設定
     * @param max_position 発散検出用の最大位置 [m]
     * @param max_velocity 発散検出用の最大速度 [m/s]
     */
    bool init(const stampfly::ESKF::Config& config,
              const SensorEnables& enables,
              float max_position = 100.0f,
              float max_velocity = 50.0f);

    /**
     * @brief 初期化済みか
     */
    bool isInitialized() const { return initialized_; }

    // =========================================================================
    // IMU更新（400Hz メインループから呼び出し）
    // =========================================================================

    /**
     * @brief IMU予測ステップ + 加速度計姿勢補正
     * @param accel_body 加速度 [m/s²] 機体座標系
     * @param gyro_body 角速度 [rad/s] 機体座標系
     * @param dt 時間刻み [s]
     * @param skip_position 位置・速度の更新をスキップ（接地中用）
     * @return 発散していない場合true
     */
    bool predictIMU(const stampfly::math::Vector3& accel_body,
                    const stampfly::math::Vector3& gyro_body,
                    float dt,
                    bool skip_position = false);

    // =========================================================================
    // 各センサー更新（非同期、データ準備時に呼び出し）
    // =========================================================================

    /**
     * @brief オプティカルフロー更新
     * @param dx X方向の生カウント
     * @param dy Y方向の生カウント
     * @param squal 表面品質（0x19以上で有効）
     * @param distance ToF距離 [m]
     * @param dt 時間刻み [s]
     * @param gyro_x 機体X軸角速度 [rad/s]
     * @param gyro_y 機体Y軸角速度 [rad/s]
     */
    void updateOpticalFlow(int16_t dx, int16_t dy, uint8_t squal,
                           float distance, float dt,
                           float gyro_x, float gyro_y);

    /**
     * @brief 気圧高度更新
     */
    void updateBarometer(float relative_altitude);

    /**
     * @brief ToF距離更新
     */
    void updateToF(float distance);

    /**
     * @brief 磁力計更新
     */
    void updateMagnetometer(const stampfly::math::Vector3& mag_body);

    // =========================================================================
    // 状態取得
    // =========================================================================

    /**
     * @brief 現在の推定状態を取得
     */
    State getState() const;

    /**
     * @brief 発散しているか
     */
    bool isDiverged() const { return diverged_; }

    /**
     * @brief 位置の共分散（対角成分の和）を取得
     * @return P(x,x) + P(y,y) + P(z,z) [m²]
     */
    float getPositionVariance() const {
        return eskf_.getPositionVariance();
    }

    /**
     * @brief 速度の共分散（対角成分の和）を取得
     */
    float getVelocityVariance() const {
        return eskf_.getVelocityVariance();
    }

    /**
     * @brief ESKFリセット
     */
    void reset();

    /**
     * @brief 位置・速度のみリセット（姿勢・バイアスは維持）
     *
     * 離陸時・着陸時・飛行中の任意タイミングで使用。
     * 現在位置を原点(0,0,0)にリセット。
     */
    void resetPositionVelocity();

    /**
     * @brief 位置・速度をゼロに保持（共分散は維持）
     *
     * 接地中の連続呼び出し用。離陸時の安定した推定開始のため
     * 共分散はリセットしない。
     */
    void holdPositionVelocity();

    /**
     * @brief 着陸時リセット（姿勢推定は継続）
     *
     * 位置・速度・加速度バイアスを0にリセット。
     * 姿勢とジャイロバイアスは維持。
     */
    void resetForLanding();

    /**
     * @brief ジャイロバイアス設定
     */
    void setGyroBias(const stampfly::math::Vector3& bias);

    /**
     * @brief 磁気リファレンス設定
     */
    void setMagReference(const stampfly::math::Vector3& ref);

    /**
     * @brief 加速度計と地磁気計から姿勢を初期化
     * @param accel 加速度 [m/s²] (ボディ座標系)
     * @param mag 地磁気 [uT] (ボディ座標系)
     *
     * ロール/ピッチを加速度計から計算し、ヨー=0で初期化。
     * 地磁気リファレンスも正しく設定される。
     * reset()の後に呼び出すこと。
     */
    void initializeAttitude(const stampfly::math::Vector3& accel,
                            const stampfly::math::Vector3& mag);

    /**
     * @brief 地磁気参照ベクトル取得 (デバッグ用)
     * @return mag_ref [uT] (NED座標系)
     */
    stampfly::math::Vector3 getMagReference() const {
        return eskf_.getMagReference();
    }

    /**
     * @brief 加速度バイアス推定のフリーズ設定
     * @param freeze true: バイアス推定を停止
     *
     * 接地中など可観測性がない状況で使用。
     * resetForLanding()で自動的にtrueに設定される。
     * 離陸時にfalseに戻す必要がある。
     */
    void setFreezeAccelBias(bool freeze) { eskf_.setFreezeAccelBias(freeze); }

    /**
     * @brief 加速度バイアス推定がフリーズ中か
     */
    bool isAccelBiasFrozen() const { return eskf_.isAccelBiasFrozen(); }

    /**
     * @brief 内部ESKFへの直接アクセス（上級者向け）
     */
    stampfly::ESKF& getESKF() { return eskf_; }
    const stampfly::ESKF& getESKF() const { return eskf_; }

private:
    bool initialized_ = false;
    bool diverged_ = false;
    float max_position_ = 100.0f;
    float max_velocity_ = 50.0f;
    SensorEnables enables_;
    stampfly::ESKF eskf_;

    bool checkDivergence(const stampfly::ESKF::State& state);
};

} // namespace sf
