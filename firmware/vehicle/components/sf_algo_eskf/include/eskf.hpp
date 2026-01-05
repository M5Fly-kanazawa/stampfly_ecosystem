/**
 * @file eskf.hpp
 * @brief Error-State Kalman Filter (ESKF) for Attitude, Position, and Velocity Estimation
 *
 * 15状態ESKF統合推定器:
 * - 位置 (3次元)
 * - 速度 (3次元)
 * - 姿勢 (クォータニオン→エラー状態は3次元回転ベクトル)
 * - ジャイロバイアス (3次元)
 * - 加速度バイアス (3次元)
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "stampfly_math.hpp"

namespace stampfly {

using namespace math;

/**
 * @brief ESKF 統合推定器
 */
class ESKF {
public:
    struct Config {
        // プロセスノイズ
        float gyro_noise;           // ジャイロノイズ [rad/s/√Hz]
        float accel_noise;          // 加速度ノイズ [m/s²/√Hz]
        float gyro_bias_noise;      // ジャイロバイアスランダムウォーク
        float accel_bias_noise;     // 加速度バイアスランダムウォーク

        // 観測ノイズ
        float baro_noise;           // 気圧高度ノイズ [m]
        float tof_noise;            // ToFノイズ [m]
        float mag_noise;            // 地磁気ノイズ [uT]
        float flow_noise;           // オプティカルフローノイズ
        float accel_att_noise;      // 加速度計姿勢補正ノイズ [m/s²]

        // 初期共分散
        float init_pos_std;
        float init_vel_std;
        float init_att_std;
        float init_gyro_bias_std;
        float init_accel_bias_std;

        // 地磁気参照ベクトル (NED)
        Vector3 mag_ref;

        // 重力加速度
        float gravity;

        // アウトライア棄却閾値 (Mahalanobis距離の二乗)
        float mahalanobis_threshold;

        // ToF傾き閾値 [rad] (これ以上傾いていると更新スキップ)
        float tof_tilt_threshold;

        // 加速度計姿勢補正のモーション閾値 [m/s²]
        float accel_motion_threshold;

        // オプティカルフロー高度閾値 [m]
        float flow_min_height;
        float flow_max_height;

        // オプティカルフロー傾き閾値 (R22 = cos(roll)*cos(pitch) の下限)
        // この値未満だと更新スキップ。cos(30°) = 0.866
        float flow_tilt_cos_threshold;

        // オプティカルフローキャリブレーション
        // PMW3901: FOV=42°, 35pixels → 0.0209 rad/pixel
        float flow_rad_per_pixel;       // 1ピクセルあたりの角度 [rad/pixel]

        // カメラ→機体座標変換行列 (2x2)
        // [flow_body_x]   [c2b_xx c2b_xy] [flow_cam_x]
        // [flow_body_y] = [c2b_yx c2b_yy] [flow_cam_y]
        float flow_cam_to_body[4];      // {c2b_xx, c2b_xy, c2b_yx, c2b_yy}

        // ========================================================================
        // ジャイロ→フロー回転補償（物理モデルベース）
        //
        // 物理モデル:
        //   flow_rot_x = gyro_scale × gyro_y  (ピッチ→X)
        //   flow_rot_y = -gyro_scale × gyro_x (ロール→Y)
        //
        // flow_gyro_scale: 理論値(1.0)からのずれを補正
        //   - 1.0 = 理論通り
        //   - < 1.0: 補正弱め
        //   - > 1.0: 補正強め
        // ========================================================================
        float flow_gyro_scale;          // ジャイロ補正スケール（デフォルト1.0）

        // フローセンサーオフセット [counts/sample]
        // 静止ホバリング時のフロー平均値をキャリブレーションで取得
        float flow_offset[2];           // {dx_offset, dy_offset}

        // 地磁気有効フラグ
        // false: 地磁気観測なし、ジャイロバイアスZは初期値から更新しない
        bool mag_enabled;

        // 姿勢補正モード
        // 0: 加速度絶対値フィルタのみ (accel_motion_thresholdで判定)
        // 1: 適応的R (水平加速度でRをスケーリング)
        // 2: 角速度フィルタ (高回転時にRを増加)
        // 3: 高回転時バイアス保護 (姿勢は更新、バイアスは保護)
        int att_update_mode;
        float k_adaptive;           // 適応的Rの係数 (モード1用)
        float gyro_att_threshold;   // 角速度閾値 [rad/s] (モード2, 3用)

        /**
         * @brief コンポーネントのデフォルト設定
         *
         * このコンポーネント単体で動作する際の標準値。
         * アプリケーション (main/) では、この値を取得して
         * 必要に応じて上書きする。
         *
         * 設定のカスタマイズは main/config.hpp で行う。
         */
        static Config defaultConfig() {
            Config cfg;
            // プロセスノイズ (Q)
            cfg.gyro_noise = 0.009655f;        // rad/s/√Hz
            cfg.accel_noise = 0.062885f;       // m/s²/√Hz
            cfg.gyro_bias_noise = 0.000013f;   // rad/s/√s
            cfg.accel_bias_noise = 0.0001f;    // m/s²/√s

            // 観測ノイズ (R)
            cfg.baro_noise = 0.1f;             // m
            cfg.tof_noise = 0.002540f;         // m
            cfg.mag_noise = 2.0f;              // uT
            cfg.flow_noise = 0.005232f;        // m/s
            cfg.accel_att_noise = 0.02f;       // m/s²

            // 初期共分散
            cfg.init_pos_std = 1.0f;
            cfg.init_vel_std = 0.5f;
            cfg.init_att_std = 0.1f;
            cfg.init_gyro_bias_std = 0.01f;
            cfg.init_accel_bias_std = 0.1f;

            // 参照値
            cfg.mag_ref = Vector3(20.0f, 0.0f, 40.0f);  // 日本近辺の概算
            cfg.gravity = 9.81f;

            // 閾値
            cfg.mahalanobis_threshold = 15.0f;
            cfg.tof_tilt_threshold = 0.70f;    // ~40度
            cfg.accel_motion_threshold = 1.0f; // m/s²
            cfg.flow_min_height = 0.02f;       // m
            cfg.flow_max_height = 4.0f;        // m
            cfg.flow_tilt_cos_threshold = 0.866f;  // cos(30°)

            // オプティカルフローキャリブレーション
            cfg.flow_rad_per_pixel = 0.00222f;
            cfg.flow_cam_to_body[0] = 0.943f;  // c2b_xx
            cfg.flow_cam_to_body[1] = 0.0f;    // c2b_xy
            cfg.flow_cam_to_body[2] = 0.0f;    // c2b_yx
            cfg.flow_cam_to_body[3] = 1.015f;  // c2b_yy
            cfg.flow_gyro_scale = 1.0f;
            cfg.flow_offset[0] = 0.0f;
            cfg.flow_offset[1] = 0.0f;

            cfg.mag_enabled = true;            // デフォルトは全センサー有効

            // 姿勢補正モード
            cfg.att_update_mode = 0;
            cfg.k_adaptive = 0.0f;
            cfg.gyro_att_threshold = 0.5f;

            return cfg;
        }
    };

    struct State {
        Vector3 position;           // 位置 [m] (NED)
        Vector3 velocity;           // 速度 [m/s]
        Quaternion orientation;     // 姿勢
        Vector3 gyro_bias;          // ジャイロバイアス [rad/s]
        Vector3 accel_bias;         // 加速度バイアス [m/s²]

        // オイラー角 (便利用)
        float roll;                 // [rad]
        float pitch;                // [rad]
        float yaw;                  // [rad]
    };

    ESKF() = default;

    /**
     * @brief 初期化
     */
    esp_err_t init(const Config& config);

    /**
     * @brief 予測ステップ (IMU入力)
     * @param accel 加速度 [m/s²] (ボディ座標系)
     * @param gyro 角速度 [rad/s] (ボディ座標系)
     * @param dt 時間刻み [s]
     * @param skip_position 位置・速度の更新をスキップ（接地中用）
     */
    void predict(const Vector3& accel, const Vector3& gyro, float dt, bool skip_position = false);

    /**
     * @brief 気圧高度更新
     * @param altitude 高度 [m]
     */
    void updateBaro(float altitude);

    /**
     * @brief ToF更新 (姿勢補正込み)
     * @param distance 距離 [m]
     */
    void updateToF(float distance);

    /**
     * @brief 地磁気更新 (ヨー補正)
     * @param mag 地磁気 [uT] (ボディ座標系)
     */
    void updateMag(const Vector3& mag);

    /**
     * @brief オプティカルフロー更新 (水平速度)
     * @param flow_x X方向フロー [rad/s]
     * @param flow_y Y方向フロー [rad/s]
     * @param distance ToFからの距離 [m]
     */
    void updateFlow(float flow_x, float flow_y, float distance);

    /**
     * @brief オプティカルフロー更新 (ジャイロ補償付き) [レガシーAPI]
     * @deprecated updateFlowRaw()を使用してください
     */
    void updateFlowWithGyro(float flow_x, float flow_y, float distance,
                            float gyro_x, float gyro_y);

    /**
     * @brief オプティカルフロー更新 (生データ入力、物理的に正しい計算)
     * @param flow_dx X方向ピクセル変位 [counts]
     * @param flow_dy Y方向ピクセル変位 [counts]
     * @param distance ToFからの距離 [m]
     * @param dt サンプリング間隔 [s]
     * @param gyro_x X軸角速度 [rad/s] (機体座標系)
     * @param gyro_y Y軸角速度 [rad/s] (機体座標系)
     *
     * 計算フロー:
     * 1. ピクセル変化 → ピクセル角速度 (rad_per_pixel / dt)
     * 2. 機体角速度 → カメラ角速度 (flow_cam_to_body変換)
     * 3. 回転成分除去 → 並進由来の角速度
     * 4. 並進速度算出 (ω × distance)
     */
    void updateFlowRaw(int16_t flow_dx, int16_t flow_dy, float distance,
                       float dt, float gyro_x, float gyro_y);

    /**
     * @brief 加速度計による姿勢補正 (Roll/Pitch)
     * @param accel 加速度 [m/s²] (ボディ座標系)
     *
     * 静止または低速移動時に加速度計から重力方向を推定し、
     * Roll/Pitchを補正する
     */
    void updateAccelAttitude(const Vector3& accel);

    /**
     * @brief 現在の状態取得
     */
    State getState() const { return state_; }

    /**
     * @brief 位置の共分散（対角成分の和）を取得
     * @return P(x,x) + P(y,y) + P(z,z) [m²]
     *
     * 共分散収束判定に使用。値が小さいほど推定が安定。
     */
    float getPositionVariance() const {
        return P_(0, 0) + P_(1, 1) + P_(2, 2);
    }

    /**
     * @brief 速度の共分散（対角成分の和）を取得
     * @return P(vx,vx) + P(vy,vy) + P(vz,vz) [(m/s)²]
     */
    float getVelocityVariance() const {
        return P_(3, 3) + P_(4, 4) + P_(5, 5);
    }

    /**
     * @brief 状態リセット
     */
    void reset();

    /**
     * @brief 位置・速度のみリセット（姿勢・バイアスは維持）
     *
     * 離陸時・着陸時・飛行中の任意タイミングで使用。
     * 現在位置を原点(0,0,0)、速度を(0,0,0)にリセット。
     * 姿勢、ジャイロバイアス、加速度バイアスは維持される。
     */
    void resetPositionVelocity();

    /**
     * @brief 位置・速度をゼロに保持（共分散は維持）
     *
     * 接地中の連続呼び出し用。共分散をリセットしないため、
     * 離陸時に安定した推定開始が可能。
     */
    void holdPositionVelocity();

    /**
     * @brief 着陸時リセット（姿勢推定は継続）
     *
     * 位置・速度・加速度バイアスを0にリセット。
     * 姿勢とジャイロバイアスは維持し、姿勢推定を継続。
     * 共分散は適切な初期値に設定。
     * 加速度バイアス推定も自動的にフリーズされる。
     */
    void resetForLanding();

    /**
     * @brief 加速度バイアス推定のフリーズ設定
     * @param freeze true: バイアス推定を停止（状態更新をスキップ）
     *
     * 接地中など、バイアスの可観測性がない状況で使用。
     * フリーズ中もKalman更新は実行されるが、dx[BA_*]は状態に適用されない。
     */
    void setFreezeAccelBias(bool freeze) { freeze_accel_bias_ = freeze; }

    /**
     * @brief 加速度バイアス推定がフリーズ中か
     */
    bool isAccelBiasFrozen() const { return freeze_accel_bias_; }

    /**
     * @brief ジャイロバイアスを設定
     * @param bias ジャイロバイアス [rad/s] (ボディ座標系)
     *
     * 起動時のキャリブレーションで取得した値を設定
     */
    void setGyroBias(const Vector3& bias);

    /**
     * @brief 加速度バイアスを設定
     * @param bias 加速度バイアス [m/s²] (ボディ座標系)
     */
    void setAccelBias(const Vector3& bias);

    /**
     * @brief 地磁気参照ベクトルを設定
     * @param mag_ref 地磁気参照ベクトル (ボディ座標系)
     *
     * 初回測定値を設定することで、起動時の向き=Yaw 0°となる
     */
    void setMagReference(const Vector3& mag_ref);

    /**
     * @brief 加速度計と地磁気計から姿勢を初期化
     * @param accel 加速度 [m/s²] (ボディ座標系)
     * @param mag 地磁気 [uT] (ボディ座標系)
     *
     * 加速度計からロール/ピッチを計算し、ヨー=0で初期化。
     * その後、地磁気リファレンスを正しくNED座標系で設定する。
     * これにより、初回の地磁気更新でジャンプが発生しなくなる。
     */
    void initializeAttitude(const Vector3& accel, const Vector3& mag);

    /**
     * @brief Yaw角を強制的に設定
     * @param yaw Yaw角 [rad]
     *
     * デバッグ用：Yawを固定値に設定する
     */
    void setYaw(float yaw);

    /**
     * @brief 共分散行列取得 (デバッグ用)
     */
    const Matrix<15, 15>& getCovariance() const { return P_; }

    /**
     * @brief 地磁気参照ベクトル取得 (デバッグ用)
     * @return mag_ref [uT] (NED座標系)
     */
    Vector3 getMagReference() const { return config_.mag_ref; }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    bool freeze_accel_bias_ = true;   // 加速度バイアス推定フリーズフラグ（起動時は着陸状態）
    Config config_;

    // 名目状態
    State state_;

    // エラー状態共分散行列 (15x15)
    // 状態順序: [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]
    Matrix<15, 15> P_;

    // 一時行列（スタック使用量削減のためメンバ変数化）
    // ESP32S3のスタックサイズ制限により、15x15行列をローカル変数として
    // 複数作成するとスタックオーバーフローが発生する
    Matrix<15, 15> F_;      // 状態遷移行列
    Matrix<15, 15> Q_;      // プロセスノイズ共分散
    Matrix<15, 15> temp1_;  // 一時計算用
    Matrix<15, 15> temp2_;  // 一時計算用

    /**
     * @brief エラー状態を名目状態に注入
     * @param dx 15次元エラー状態
     */
    void injectErrorState(const Matrix<15, 1>& dx);

    /**
     * @brief EKF更新（汎用）
     * @return true: 更新成功, false: アウトライア棄却または失敗
     */
    template<int M>
    bool measurementUpdate(const Matrix<M, 1>& z,
                           const Matrix<M, 1>& h,
                           const Matrix<M, 15>& H,
                           const Matrix<M, M>& R);

    /**
     * @brief 共分散行列の対称性・正定値性を強制
     */
    void enforceCovarianceSymmetry();
};

/**
 * @brief 簡易姿勢推定器 (相補フィルタ)
 *
 * ESKF不要の場合の軽量版
 */
class AttitudeEstimator {
public:
    struct Config {
        float gyro_weight;          // ジャイロ重み (default: 0.98)
        float mag_declination;      // 地磁気偏角 [rad]
    };

    struct State {
        Quaternion orientation;
        float roll;                 // [rad]
        float pitch;                // [rad]
        float yaw;                  // [rad]
        Vector3 gyro_bias;
    };

    AttitudeEstimator() = default;

    esp_err_t init(const Config& config);
    void update(const Vector3& accel, const Vector3& gyro, float dt);
    void updateMag(const Vector3& mag);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
};

/**
 * @brief 簡易高度推定器 (カルマンフィルタ)
 */
class AltitudeEstimator {
public:
    struct Config {
        float process_noise_alt;
        float process_noise_vel;
        float measurement_noise_baro;
        float measurement_noise_tof;
    };

    struct State {
        float altitude;             // [m]
        float velocity;             // [m/s]
    };

    AltitudeEstimator() = default;

    esp_err_t init(const Config& config);
    void predict(float accel_z, float dt);
    void updateBaro(float altitude);
    void updateToF(float distance, float pitch, float roll);
    State getState() const { return state_; }
    void reset();

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
    float P_[2][2] = {{1, 0}, {0, 1}};
};

/**
 * @brief 簡易速度推定器 (オプティカルフロー)
 */
class VelocityEstimator {
public:
    struct Config {
        float flow_scale;           // フロースケール
    };

    struct State {
        float vx;                   // [m/s] (ボディ座標系)
        float vy;                   // [m/s] (ボディ座標系)
    };

    VelocityEstimator() = default;

    esp_err_t init(const Config& config);
    void updateFlow(float flow_x, float flow_y, float height, float gyro_x, float gyro_y);
    State getState() const { return state_; }
    void reset();

    /**
     * @brief ジャイロバイアスを設定
     * @param bias_x X軸ジャイロバイアス [rad/s]
     * @param bias_y Y軸ジャイロバイアス [rad/s]
     */
    void setGyroBias(float bias_x, float bias_y) {
        gyro_bias_x_ = bias_x;
        gyro_bias_y_ = bias_y;
    }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Config config_;
    State state_;
    float gyro_bias_x_ = 0.0f;
    float gyro_bias_y_ = 0.0f;
};

}  // namespace stampfly
