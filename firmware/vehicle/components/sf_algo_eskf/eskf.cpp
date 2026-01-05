/**
 * @file eskf.cpp
 * @brief ESKF Implementation
 *
 * Error-State Kalman Filter (ESKF) による統合推定器
 * 15状態: [位置(3), 速度(3), 姿勢誤差(3), ジャイロバイアス(3), 加速度バイアス(3)]
 */

#include "eskf.hpp"
#include "esp_log.h"
#include <cmath>

static const char* TAG = "ESKF";

namespace stampfly {

// エラー状態インデックス
enum StateIdx {
    POS_X = 0, POS_Y = 1, POS_Z = 2,
    VEL_X = 3, VEL_Y = 4, VEL_Z = 5,
    ATT_X = 6, ATT_Y = 7, ATT_Z = 8,
    BG_X = 9, BG_Y = 10, BG_Z = 11,
    BA_X = 12, BA_Y = 13, BA_Z = 14
};

// ============================================================================
// ESKF Implementation
// ============================================================================

esp_err_t ESKF::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ESKF (15-state)");
    config_ = config;
    reset();
    initialized_ = true;
    ESP_LOGI(TAG, "ESKF initialized successfully");

    return ESP_OK;
}

void ESKF::reset()
{
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
    state_.orientation = Quaternion::identity();
    state_.gyro_bias = Vector3::zero();
    state_.accel_bias = Vector3::zero();
    state_.roll = state_.pitch = state_.yaw = 0.0f;

    // 共分散行列の初期化
    P_ = Matrix<15, 15>::zeros();
    float pos_var = config_.init_pos_std * config_.init_pos_std;
    float vel_var = config_.init_vel_std * config_.init_vel_std;
    float att_var = config_.init_att_std * config_.init_att_std;
    float bg_var = config_.init_gyro_bias_std * config_.init_gyro_bias_std;
    float ba_var = config_.init_accel_bias_std * config_.init_accel_bias_std;

    P_(POS_X, POS_X) = pos_var;
    P_(POS_Y, POS_Y) = pos_var;
    P_(POS_Z, POS_Z) = pos_var;
    P_(VEL_X, VEL_X) = vel_var;
    P_(VEL_Y, VEL_Y) = vel_var;
    P_(VEL_Z, VEL_Z) = vel_var;
    P_(ATT_X, ATT_X) = att_var;
    P_(ATT_Y, ATT_Y) = att_var;
    P_(ATT_Z, ATT_Z) = att_var;
    P_(BG_X, BG_X) = bg_var;
    P_(BG_Y, BG_Y) = bg_var;
    P_(BG_Z, BG_Z) = bg_var;
    P_(BA_X, BA_X) = ba_var;
    P_(BA_Y, BA_Y) = ba_var;
    P_(BA_Z, BA_Z) = ba_var;
}

void ESKF::resetPositionVelocity()
{
    // 位置・速度のみゼロにリセット（姿勢・バイアスは維持）
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();

    // 位置・速度の共分散を初期値に戻す
    float pos_var = config_.init_pos_std * config_.init_pos_std;
    float vel_var = config_.init_vel_std * config_.init_vel_std;

    // 位置・速度ブロック全体をクリアしてから対角成分を設定
    // （位置-速度間の相関、非対角成分も含めてクリア）
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_(i, j) = 0.0f;
        }
    }

    // 対角成分を設定
    P_(POS_X, POS_X) = pos_var;
    P_(POS_Y, POS_Y) = pos_var;
    P_(POS_Z, POS_Z) = pos_var;
    P_(VEL_X, VEL_X) = vel_var;
    P_(VEL_Y, VEL_Y) = vel_var;
    P_(VEL_Z, VEL_Z) = vel_var;

    // 位置・速度と他状態（姿勢、バイアス）の相関をクリア
    for (int i = 0; i < 6; i++) {
        for (int j = 6; j < 15; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }
    // ログは呼び出し側で管理（400Hz呼び出し時のスパム防止）
}

void ESKF::holdPositionVelocity()
{
    // 状態のみゼロに設定（共分散は維持）
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
}

void ESKF::resetForLanding()
{
    // 位置・速度・加速度バイアスを0にリセット
    // 姿勢・ジャイロバイアスは維持（姿勢推定継続）
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
    state_.accel_bias = Vector3::zero();

    // 加速度バイアス推定をフリーズ（接地中は可観測性がないため）
    freeze_accel_bias_ = true;

    // 共分散の設定
    float pos_var = config_.init_pos_std * config_.init_pos_std;
    float vel_var = config_.init_vel_std * config_.init_vel_std;
    float ba_var = config_.init_accel_bias_std * config_.init_accel_bias_std;

    // 位置・速度ブロック（0-5）をクリアして対角成分を設定
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_(i, j) = 0.0f;
        }
    }
    P_(POS_X, POS_X) = pos_var;
    P_(POS_Y, POS_Y) = pos_var;
    P_(POS_Z, POS_Z) = pos_var;
    P_(VEL_X, VEL_X) = vel_var;
    P_(VEL_Y, VEL_Y) = vel_var;
    P_(VEL_Z, VEL_Z) = vel_var;

    // 加速度バイアスブロック（12-14）をクリアして対角成分を設定
    for (int i = 12; i < 15; i++) {
        for (int j = 12; j < 15; j++) {
            P_(i, j) = 0.0f;
        }
    }
    P_(BA_X, BA_X) = ba_var;
    P_(BA_Y, BA_Y) = ba_var;
    P_(BA_Z, BA_Z) = ba_var;

    // 位置・速度と他状態（姿勢、バイアス）の相関をクリア
    for (int i = 0; i < 6; i++) {
        for (int j = 6; j < 15; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }

    // 加速度バイアスと姿勢・ジャイロバイアスの相関をクリア
    for (int i = 12; i < 15; i++) {
        for (int j = 6; j < 12; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }
}

void ESKF::setGyroBias(const Vector3& bias)
{
    state_.gyro_bias = bias;
}

void ESKF::setAccelBias(const Vector3& bias)
{
    state_.accel_bias = bias;
}

void ESKF::setMagReference(const Vector3& mag_ref)
{
    config_.mag_ref = mag_ref;
}

void ESKF::setYaw(float yaw)
{
    state_.yaw = yaw;
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, yaw);
}

void ESKF::initializeAttitude(const Vector3& accel, const Vector3& mag)
{
    // 加速度計からロール/ピッチを計算（重力の反力から）
    // 加速度計は重力の反力を測定: a = -R^T * g
    // NED座標系で静止時: a_body ≈ (0, 0, -g) （上向き = -Z）
    //
    // ロール右（右翼下）: a_body = (0, -g*sin(φ), -g*cos(φ))
    //   → roll = atan2(-ay, -az) = φ
    //
    // ピッチアップ: a_body = (g*sin(θ), 0, -g*cos(θ))
    //   → pitch = atan2(ax, sqrt(ay² + az²)) = θ
    float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (accel_norm < 1.0f) {
        // 加速度が小さすぎる場合は水平とみなす
        state_.roll = 0.0f;
        state_.pitch = 0.0f;
    } else {
        state_.roll = std::atan2(-accel.y, -accel.z);
        state_.pitch = std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
    }
    state_.yaw = 0.0f;

    // 姿勢クォータニオンを設定
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, state_.yaw);

    // 回転行列を計算（R: ボディ→NED）
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    float r00 = 1 - 2*(q2*q2 + q3*q3);
    float r01 = 2*(q1*q2 - q0*q3);
    float r02 = 2*(q1*q3 + q0*q2);
    float r10 = 2*(q1*q2 + q0*q3);
    float r11 = 1 - 2*(q1*q1 + q3*q3);
    float r12 = 2*(q2*q3 - q0*q1);
    float r20 = 2*(q1*q3 - q0*q2);
    float r21 = 2*(q2*q3 + q0*q1);
    float r22 = 1 - 2*(q1*q1 + q2*q2);

    // 地磁気をNED座標系に変換: mag_ref = R * mag_body
    // これにより、現在の向き=ヨー0度として設定される
    config_.mag_ref.x = r00 * mag.x + r01 * mag.y + r02 * mag.z;
    config_.mag_ref.y = r10 * mag.x + r11 * mag.y + r12 * mag.z;
    config_.mag_ref.z = r20 * mag.x + r21 * mag.y + r22 * mag.z;

    ESP_LOGI(TAG, "Attitude initialized: roll=%.1f° pitch=%.1f° yaw=0°",
             state_.roll * 180.0f / M_PI, state_.pitch * 180.0f / M_PI);
    ESP_LOGI(TAG, "Mag ref (NED): (%.1f, %.1f, %.1f) uT",
             config_.mag_ref.x, config_.mag_ref.y, config_.mag_ref.z);
}

void ESKF::predict(const Vector3& accel, const Vector3& gyro, float dt, bool skip_position)
{
    if (!initialized_ || dt <= 0) return;

    // バイアス補正
    Vector3 gyro_corrected = gyro - state_.gyro_bias;
    Vector3 accel_corrected = accel - state_.accel_bias;

    // mag_enabled=false時はYawレートを0に固定（ドリフト防止）
    if (!config_.mag_enabled) {
        gyro_corrected.z = 0.0f;
    }

    // 回転行列要素を直接計算（3x3行列を避ける）
    // 共分散更新でも使用するためif文の外で定義
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // R = q.toRotationMatrix() の要素
    float R00 = 1 - 2*(q2*q2 + q3*q3);
    float R01 = 2*(q1*q2 - q0*q3);
    float R02 = 2*(q1*q3 + q0*q2);
    float R10 = 2*(q1*q2 + q0*q3);
    float R11 = 1 - 2*(q1*q1 + q3*q3);
    float R12 = 2*(q2*q3 - q0*q1);
    float R20 = 2*(q1*q3 - q0*q2);
    float R21 = 2*(q2*q3 + q0*q1);
    float R22 = 1 - 2*(q1*q1 + q2*q2);

    // 加速度（共分散更新でも使用）
    float ax = accel_corrected.x, ay = accel_corrected.y, az = accel_corrected.z;

    // 位置・速度の更新（接地中はスキップ、共分散は更新継続）
    if (!skip_position) {
        // ワールド座標系での加速度 = R * accel_corrected + gravity
        float accel_world_x = R00*ax + R01*ay + R02*az;
        float accel_world_y = R10*ax + R11*ay + R12*az;
        float accel_world_z = R20*ax + R21*ay + R22*az + config_.gravity;

        // 名目状態の更新
        float half_dt_sq = 0.5f * dt * dt;
        state_.position.x += state_.velocity.x * dt + accel_world_x * half_dt_sq;
        state_.position.y += state_.velocity.y * dt + accel_world_y * half_dt_sq;
        state_.position.z += state_.velocity.z * dt + accel_world_z * half_dt_sq;
        state_.velocity.x += accel_world_x * dt;
        state_.velocity.y += accel_world_y * dt;
        state_.velocity.z += accel_world_z * dt;
    }

    // 姿勢: q = q ⊗ exp(ω*dt)
    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // mag_enabled=false時はYaw=0に固定
    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    // ========================================================================
    // 疎行列展開による共分散更新: P' = F * P * F^T + Q
    // ========================================================================
    // F行列の非ゼロ要素:
    //   対角: F[i][i] = 1
    //   F[0][3] = F[1][4] = F[2][5] = dt          (dp/dv)
    //   F[3..5][6..8] = D_va[3x3] = -R*skew(a)*dt (dv/dθ)
    //   F[3..5][12..14] = D_vb[3x3] = -R*dt       (dv/db_a)
    //   F[6][9] = F[7][10] = F[8][11] = -dt       (dθ/db_g)
    // ========================================================================

    // D_va = -R * skew(a) * dt = R * skew(-a*dt)
    // skew(-a*dt) = [  0   az*dt -ay*dt]
    //               [-az*dt  0   ax*dt]
    //               [ ay*dt -ax*dt  0 ]
    float adt_x = ax * dt, adt_y = ay * dt, adt_z = az * dt;

    // D_va = -R * skew(a) * dt の各要素を直接計算
    float D_va00 = R01*adt_z - R02*adt_y;
    float D_va01 = R02*adt_x - R00*adt_z;
    float D_va02 = R00*adt_y - R01*adt_x;
    float D_va10 = R11*adt_z - R12*adt_y;
    float D_va11 = R12*adt_x - R10*adt_z;
    float D_va12 = R10*adt_y - R11*adt_x;
    float D_va20 = R21*adt_z - R22*adt_y;
    float D_va21 = R22*adt_x - R20*adt_z;
    float D_va22 = R20*adt_y - R21*adt_x;

    // D_vb = -R * dt
    float D_vb00 = -R00*dt, D_vb01 = -R01*dt, D_vb02 = -R02*dt;
    float D_vb10 = -R10*dt, D_vb11 = -R11*dt, D_vb12 = -R12*dt;
    float D_vb20 = -R20*dt, D_vb21 = -R21*dt, D_vb22 = -R22*dt;

    float neg_dt = -dt;

    // プロセスノイズ（対角のみ）
    float gyro_var = config_.gyro_noise * config_.gyro_noise * dt;
    float accel_var = config_.accel_noise * config_.accel_noise * dt;
    float bg_var = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
    float ba_var = config_.accel_bias_noise * config_.accel_bias_noise * dt;
    float bg_z_var = config_.mag_enabled ? bg_var : 0.0f;

    // P行列の要素を直接参照するためのエイリアス
    // Pは対称行列なのでP[i][j] = P[j][i]

    // 一時変数: FP = F * P の必要な行を計算
    // FP[i][j] = sum_k F[i][k] * P[k][j]
    //
    // ブロック構造を利用:
    // pos行 (0-2): FP[i][j] = P[i][j] + dt * P[i+3][j]
    // vel行 (3-5): FP[i][j] = P[i][j] + D_va[i-3][k] * P[6+k][j] + D_vb[i-3][k] * P[12+k][j]
    // att行 (6-8): FP[i][j] = P[i][j] + neg_dt * P[i+3][j]
    // bg行 (9-11): FP[i][j] = P[i][j]
    // ba行 (12-14): FP[i][j] = P[i][j]

    // P' = FP * F^T + Q を直接計算
    // 対称性を利用して上三角のみ計算し、下三角にコピー

    // 新しいP行列を temp1_ に構築（P_を直接更新すると途中で値が変わるため）

    // ---- pos-pos ブロック (0-2, 0-2) ----
    // P'[i][j] = (P[i][j] + dt*P[i+3][j]) + dt*(P[j][i+3] + dt*P[i+3][j+3])
    //          = P[i][j] + dt*P[i+3][j] + dt*P[j][i+3] + dt^2*P[i+3][j+3]
    // 対称性より P[i+3][j] = P[j][i+3]^T
    float dt2 = dt * dt;
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            float val = P_(i, j) + dt * (P_(i+3, j) + P_(i, j+3)) + dt2 * P_(i+3, j+3);
            temp1_(i, j) = val;
            temp1_(j, i) = val;
        }
    }

    // ---- pos-vel ブロック (0-2, 3-5) ----
    // P'[i][j] = (P[i][k] + dt*P[i+3][k]) * F^T[k][j]
    // F^T[k][j] は F[j][k] なので、j=3..5に対してF[j][j]=1, F[j][6..8]=D_va, F[j][12..14]=D_vb
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int jj = j + 3;  // vel index
            // FP[i][k] for k relevant to col jj
            float fp_i_j3 = P_(i, jj) + dt * P_(i+3, jj);
            // Add contributions from att and ba blocks
            float fp_i_6 = P_(i, 6) + dt * P_(i+3, 6);
            float fp_i_7 = P_(i, 7) + dt * P_(i+3, 7);
            float fp_i_8 = P_(i, 8) + dt * P_(i+3, 8);
            float fp_i_12 = P_(i, 12) + dt * P_(i+3, 12);
            float fp_i_13 = P_(i, 13) + dt * P_(i+3, 13);
            float fp_i_14 = P_(i, 14) + dt * P_(i+3, 14);

            float D_va_j0, D_va_j1, D_va_j2, D_vb_j0, D_vb_j1, D_vb_j2;
            if (j == 0) {
                D_va_j0 = D_va00; D_va_j1 = D_va01; D_va_j2 = D_va02;
                D_vb_j0 = D_vb00; D_vb_j1 = D_vb01; D_vb_j2 = D_vb02;
            } else if (j == 1) {
                D_va_j0 = D_va10; D_va_j1 = D_va11; D_va_j2 = D_va12;
                D_vb_j0 = D_vb10; D_vb_j1 = D_vb11; D_vb_j2 = D_vb12;
            } else {
                D_va_j0 = D_va20; D_va_j1 = D_va21; D_va_j2 = D_va22;
                D_vb_j0 = D_vb20; D_vb_j1 = D_vb21; D_vb_j2 = D_vb22;
            }

            float val = fp_i_j3 + fp_i_6*D_va_j0 + fp_i_7*D_va_j1 + fp_i_8*D_va_j2
                                + fp_i_12*D_vb_j0 + fp_i_13*D_vb_j1 + fp_i_14*D_vb_j2;
            temp1_(i, jj) = val;
            temp1_(jj, i) = val;
        }
    }

    // ---- pos-att ブロック (0-2, 6-8) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int jj = j + 6;
            float fp_i_j6 = P_(i, jj) + dt * P_(i+3, jj);
            float fp_i_9 = P_(i, 9+j) + dt * P_(i+3, 9+j);  // bg対角
            float val = fp_i_j6 + fp_i_9 * neg_dt;
            temp1_(i, jj) = val;
            temp1_(jj, i) = val;
        }
    }

    // ---- pos-bg ブロック (0-2, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val = P_(i, 9+j) + dt * P_(i+3, 9+j);
            temp1_(i, 9+j) = val;
            temp1_(9+j, i) = val;
        }
    }

    // ---- pos-ba ブロック (0-2, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val = P_(i, 12+j) + dt * P_(i+3, 12+j);
            temp1_(i, 12+j) = val;
            temp1_(12+j, i) = val;
        }
    }

    // ---- vel-vel ブロック (3-5, 3-5) ----
    // 最も複雑: D_va と D_vb の両方が関与
    for (int i = 0; i < 3; i++) {
        float D_va_i0, D_va_i1, D_va_i2, D_vb_i0, D_vb_i1, D_vb_i2;
        if (i == 0) {
            D_va_i0 = D_va00; D_va_i1 = D_va01; D_va_i2 = D_va02;
            D_vb_i0 = D_vb00; D_vb_i1 = D_vb01; D_vb_i2 = D_vb02;
        } else if (i == 1) {
            D_va_i0 = D_va10; D_va_i1 = D_va11; D_va_i2 = D_va12;
            D_vb_i0 = D_vb10; D_vb_i1 = D_vb11; D_vb_i2 = D_vb12;
        } else {
            D_va_i0 = D_va20; D_va_i1 = D_va21; D_va_i2 = D_va22;
            D_vb_i0 = D_vb20; D_vb_i1 = D_vb21; D_vb_i2 = D_vb22;
        }

        // FP[3+i][k] = P[3+i][k] + D_va[i][m]*P[6+m][k] + D_vb[i][m]*P[12+m][k]
        for (int j = i; j < 3; j++) {
            int ii = 3 + i, jj = 3 + j;

            float D_va_j0, D_va_j1, D_va_j2, D_vb_j0, D_vb_j1, D_vb_j2;
            if (j == 0) {
                D_va_j0 = D_va00; D_va_j1 = D_va01; D_va_j2 = D_va02;
                D_vb_j0 = D_vb00; D_vb_j1 = D_vb01; D_vb_j2 = D_vb02;
            } else if (j == 1) {
                D_va_j0 = D_va10; D_va_j1 = D_va11; D_va_j2 = D_va12;
                D_vb_j0 = D_vb10; D_vb_j1 = D_vb11; D_vb_j2 = D_vb12;
            } else {
                D_va_j0 = D_va20; D_va_j1 = D_va21; D_va_j2 = D_va22;
                D_vb_j0 = D_vb20; D_vb_j1 = D_vb21; D_vb_j2 = D_vb22;
            }

            // 基本項: P[ii][jj]
            float val = P_(ii, jj);

            // D_va * P_att の寄与
            val += D_va_i0 * P_(6, jj) + D_va_i1 * P_(7, jj) + D_va_i2 * P_(8, jj);
            val += P_(ii, 6) * D_va_j0 + P_(ii, 7) * D_va_j1 + P_(ii, 8) * D_va_j2;

            // D_vb * P_ba の寄与
            val += D_vb_i0 * P_(12, jj) + D_vb_i1 * P_(13, jj) + D_vb_i2 * P_(14, jj);
            val += P_(ii, 12) * D_vb_j0 + P_(ii, 13) * D_vb_j1 + P_(ii, 14) * D_vb_j2;

            // D_va * P_aa * D_va^T
            for (int m = 0; m < 3; m++) {
                float D_va_im = (m==0) ? D_va_i0 : (m==1) ? D_va_i1 : D_va_i2;
                for (int n = 0; n < 3; n++) {
                    float D_va_jn = (n==0) ? D_va_j0 : (n==1) ? D_va_j1 : D_va_j2;
                    val += D_va_im * P_(6+m, 6+n) * D_va_jn;
                }
            }

            // D_va * P_ab * D_vb^T + D_vb * P_ba * D_va^T
            for (int m = 0; m < 3; m++) {
                float D_va_im = (m==0) ? D_va_i0 : (m==1) ? D_va_i1 : D_va_i2;
                float D_vb_im = (m==0) ? D_vb_i0 : (m==1) ? D_vb_i1 : D_vb_i2;
                for (int n = 0; n < 3; n++) {
                    float D_va_jn = (n==0) ? D_va_j0 : (n==1) ? D_va_j1 : D_va_j2;
                    float D_vb_jn = (n==0) ? D_vb_j0 : (n==1) ? D_vb_j1 : D_vb_j2;
                    val += D_va_im * P_(6+m, 12+n) * D_vb_jn;
                    val += D_vb_im * P_(12+m, 6+n) * D_va_jn;
                }
            }

            // D_vb * P_bb * D_vb^T
            for (int m = 0; m < 3; m++) {
                float D_vb_im = (m==0) ? D_vb_i0 : (m==1) ? D_vb_i1 : D_vb_i2;
                for (int n = 0; n < 3; n++) {
                    float D_vb_jn = (n==0) ? D_vb_j0 : (n==1) ? D_vb_j1 : D_vb_j2;
                    val += D_vb_im * P_(12+m, 12+n) * D_vb_jn;
                }
            }

            // Q項
            if (i == j) val += accel_var;

            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- vel-att ブロック (3-5, 6-8) ----
    for (int i = 0; i < 3; i++) {
        float D_va_i0, D_va_i1, D_va_i2, D_vb_i0, D_vb_i1, D_vb_i2;
        if (i == 0) {
            D_va_i0 = D_va00; D_va_i1 = D_va01; D_va_i2 = D_va02;
            D_vb_i0 = D_vb00; D_vb_i1 = D_vb01; D_vb_i2 = D_vb02;
        } else if (i == 1) {
            D_va_i0 = D_va10; D_va_i1 = D_va11; D_va_i2 = D_va12;
            D_vb_i0 = D_vb10; D_vb_i1 = D_vb11; D_vb_i2 = D_vb12;
        } else {
            D_va_i0 = D_va20; D_va_i1 = D_va21; D_va_i2 = D_va22;
            D_vb_i0 = D_vb20; D_vb_i1 = D_vb21; D_vb_i2 = D_vb22;
        }

        for (int j = 0; j < 3; j++) {
            int ii = 3 + i, jj = 6 + j;

            // FP[ii][k] の関連列
            float fp_ii_jj = P_(ii, jj) + D_va_i0*P_(6, jj) + D_va_i1*P_(7, jj) + D_va_i2*P_(8, jj)
                           + D_vb_i0*P_(12, jj) + D_vb_i1*P_(13, jj) + D_vb_i2*P_(14, jj);

            // F^T[k][jj] の非ゼロ: k=jj(1), k=9+j(-dt)
            float fp_ii_9j = P_(ii, 9+j) + D_va_i0*P_(6, 9+j) + D_va_i1*P_(7, 9+j) + D_va_i2*P_(8, 9+j)
                           + D_vb_i0*P_(12, 9+j) + D_vb_i1*P_(13, 9+j) + D_vb_i2*P_(14, 9+j);

            float val = fp_ii_jj + fp_ii_9j * neg_dt;
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- vel-bg ブロック (3-5, 9-11) ----
    for (int i = 0; i < 3; i++) {
        float D_va_i0, D_va_i1, D_va_i2, D_vb_i0, D_vb_i1, D_vb_i2;
        if (i == 0) {
            D_va_i0 = D_va00; D_va_i1 = D_va01; D_va_i2 = D_va02;
            D_vb_i0 = D_vb00; D_vb_i1 = D_vb01; D_vb_i2 = D_vb02;
        } else if (i == 1) {
            D_va_i0 = D_va10; D_va_i1 = D_va11; D_va_i2 = D_va12;
            D_vb_i0 = D_vb10; D_vb_i1 = D_vb11; D_vb_i2 = D_vb12;
        } else {
            D_va_i0 = D_va20; D_va_i1 = D_va21; D_va_i2 = D_va22;
            D_vb_i0 = D_vb20; D_vb_i1 = D_vb21; D_vb_i2 = D_vb22;
        }

        for (int j = 0; j < 3; j++) {
            int ii = 3 + i, jj = 9 + j;
            float val = P_(ii, jj) + D_va_i0*P_(6, jj) + D_va_i1*P_(7, jj) + D_va_i2*P_(8, jj)
                      + D_vb_i0*P_(12, jj) + D_vb_i1*P_(13, jj) + D_vb_i2*P_(14, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- vel-ba ブロック (3-5, 12-14) ----
    for (int i = 0; i < 3; i++) {
        float D_va_i0, D_va_i1, D_va_i2, D_vb_i0, D_vb_i1, D_vb_i2;
        if (i == 0) {
            D_va_i0 = D_va00; D_va_i1 = D_va01; D_va_i2 = D_va02;
            D_vb_i0 = D_vb00; D_vb_i1 = D_vb01; D_vb_i2 = D_vb02;
        } else if (i == 1) {
            D_va_i0 = D_va10; D_va_i1 = D_va11; D_va_i2 = D_va12;
            D_vb_i0 = D_vb10; D_vb_i1 = D_vb11; D_vb_i2 = D_vb12;
        } else {
            D_va_i0 = D_va20; D_va_i1 = D_va21; D_va_i2 = D_va22;
            D_vb_i0 = D_vb20; D_vb_i1 = D_vb21; D_vb_i2 = D_vb22;
        }

        for (int j = 0; j < 3; j++) {
            int ii = 3 + i, jj = 12 + j;
            float val = P_(ii, jj) + D_va_i0*P_(6, jj) + D_va_i1*P_(7, jj) + D_va_i2*P_(8, jj)
                      + D_vb_i0*P_(12, jj) + D_vb_i1*P_(13, jj) + D_vb_i2*P_(14, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- att-att ブロック (6-8, 6-8) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 6 + i, jj = 6 + j;
            // FP[ii][k] = P[ii][k] + neg_dt * P[9+i][k]
            // P'[ii][jj] = FP[ii][jj] + FP[ii][9+j] * neg_dt
            float fp_ii_jj = P_(ii, jj) + neg_dt * P_(9+i, jj);
            float fp_ii_9j = P_(ii, 9+j) + neg_dt * P_(9+i, 9+j);
            float val = fp_ii_jj + fp_ii_9j * neg_dt;
            if (i == j) val += gyro_var;
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- att-bg ブロック (6-8, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 6 + i, jj = 9 + j;
            float val = P_(ii, jj) + neg_dt * P_(9+i, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- att-ba ブロック (6-8, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 6 + i, jj = 12 + j;
            float val = P_(ii, jj) + neg_dt * P_(9+i, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- bg-bg ブロック (9-11, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 9 + i, jj = 9 + j;
            float val = P_(ii, jj);
            if (i == j) {
                val += (i == 2) ? bg_z_var : bg_var;
            }
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- bg-ba ブロック (9-11, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 9 + i, jj = 12 + j;
            temp1_(ii, jj) = P_(ii, jj);
            temp1_(jj, ii) = P_(ii, jj);
        }
    }

    // ---- ba-ba ブロック (12-14, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 12 + i, jj = 12 + j;
            float val = P_(ii, jj);
            if (i == j) val += ba_var;
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // 結果をP_にコピー
    P_ = temp1_;
}

void ESKF::updateBaro(float altitude)
{
    if (!initialized_) return;

    // ========================================================================
    // 疎行列展開による観測更新
    // H行列の非ゼロ要素: H[0][2]=1 (POS_Z) のみ
    // ========================================================================

    // 観測残差 y = z - h
    float y = -altitude - state_.position.z;

    float R_val = config_.baro_noise * config_.baro_noise;

    // S = H * P * H^T + R = P[2][2] + R (スカラ)
    float S = P_(2, 2) + R_val;
    if (S < 1e-10f) return;
    float S_inv = 1.0f / S;

    // カルマンゲイン K (15x1): K[i] = P[i][2] / S
    float K[15];
    for (int i = 0; i < 15; i++) {
        K[i] = P_(i, 2) * S_inv;
    }

    // dx = K * y
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i] * y;
    }

    // 状態注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // I_KH[i][j] = delta_ij - K[i]*(j==2)
    // (I_KH * P)[i][j] = P[i][j] - K[i] * P[2][j]
    // P'[i][j] = temp[i][j] - K[j] * temp[i][2] + R_val * K[i] * K[j]

    // Step 1: temp1_ = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki * P_(2, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float temp1_i2 = temp1_(i, 2);
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            float val = temp1_(i, j) - K[j] * temp1_i2;
            val += R_val * Ki * K[j];
            P_(i, j) = val;
        }
    }
}

void ESKF::updateToF(float distance)
{
    if (!initialized_) return;

    // 傾きが大きい場合はスキップ
    float tilt = std::sqrt(state_.roll * state_.roll + state_.pitch * state_.pitch);
    if (tilt > config_.tof_tilt_threshold) {
        return;
    }

    // ========================================================================
    // 疎行列展開による観測更新
    // H行列の非ゼロ要素: H[0][2]=1 (POS_Z) のみ
    // ========================================================================

    // 姿勢に基づく地上距離への変換
    float cos_roll = std::cos(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float height = distance * cos_roll * cos_pitch;

    // 観測残差 y = z - h
    float y = -height - state_.position.z;

    float R_val = config_.tof_noise * config_.tof_noise;

    // S = H * P * H^T + R = P[2][2] + R (スカラ)
    float S = P_(2, 2) + R_val;
    if (S < 1e-10f) return;
    float S_inv = 1.0f / S;

    // カルマンゲイン K (15x1): K[i] = P[i][2] / S
    float K[15];
    for (int i = 0; i < 15; i++) {
        K[i] = P_(i, 2) * S_inv;
    }

    // dx = K * y
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i] * y;
    }

    // 状態注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // I_KH[i][j] = delta_ij - K[i]*(j==2)
    // (I_KH * P)[i][j] = P[i][j] - K[i] * P[2][j]
    // P'[i][j] = temp[i][j] - K[j] * temp[i][2] + R_val * K[i] * K[j]

    // Step 1: temp1_ = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki * P_(2, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float temp1_i2 = temp1_(i, 2);
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            float val = temp1_(i, j) - K[j] * temp1_i2;
            val += R_val * Ki * K[j];
            P_(i, j) = val;
        }
    }
}

void ESKF::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    // 地磁気ベクトルのノルムチェック（異常値除去）
    float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
    if (mag_norm < 10.0f || mag_norm > 100.0f) {
        return;  // 異常な地磁気読み取り
    }

    // ========================================================================
    // 疎行列展開による観測更新
    // H行列の非ゼロ要素: 列6,7,8（ATT_X, ATT_Y, ATT_Z）のみ
    // H = -skew(h) の形式
    // ========================================================================

    // クォータニオンから回転行列要素を直接計算
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // R行列の全要素
    float r00 = 1 - 2*(q2*q2 + q3*q3);
    float r01 = 2*(q1*q2 - q0*q3);
    float r02 = 2*(q1*q3 + q0*q2);
    float r10 = 2*(q1*q2 + q0*q3);
    float r11 = 1 - 2*(q1*q1 + q3*q3);
    float r12 = 2*(q2*q3 - q0*q1);
    float r20 = 2*(q1*q3 - q0*q2);
    float r21 = 2*(q2*q3 + q0*q1);
    float r22 = 1 - 2*(q1*q1 + q2*q2);

    // mag_expected = R^T * mag_ref
    float hx = r00*config_.mag_ref.x + r10*config_.mag_ref.y + r20*config_.mag_ref.z;
    float hy = r01*config_.mag_ref.x + r11*config_.mag_ref.y + r21*config_.mag_ref.z;
    float hz = r02*config_.mag_ref.x + r12*config_.mag_ref.y + r22*config_.mag_ref.z;

    // 観測残差 y = z - h
    float y0 = mag.x - hx;
    float y1 = mag.y - hy;
    float y2 = mag.z - hz;

    // H行列の非ゼロ要素（-skew(h)形式）
    // H[0][6]=0,   H[0][7]=-hz, H[0][8]=hy
    // H[1][6]=hz,  H[1][7]=0,   H[1][8]=-hx
    // H[2][6]=-hy, H[2][7]=hx,  H[2][8]=0
    float H06 = 0.0f,  H07 = -hz, H08 = hy;
    float H16 = hz,    H17 = 0.0f, H18 = -hx;
    float H26 = -hy,   H27 = hx,   H28 = 0.0f;

    float R_val = config_.mag_noise * config_.mag_noise;

    // HP = H * P (3x15): 列6,7,8のみ非ゼロなので
    // HP[m][j] = H[m][6]*P[6][j] + H[m][7]*P[7][j] + H[m][8]*P[8][j]
    float HP[3][15];
    for (int j = 0; j < 15; j++) {
        float P6j = P_(6, j);
        float P7j = P_(7, j);
        float P8j = P_(8, j);
        HP[0][j] = H06*P6j + H07*P7j + H08*P8j;
        HP[1][j] = H16*P6j + H17*P7j + H18*P8j;
        HP[2][j] = H26*P6j + H27*P7j + H28*P8j;
    }

    // S = HP * H^T + R (3x3)
    // S[m][n] = HP[m][6]*H[n][6] + HP[m][7]*H[n][7] + HP[m][8]*H[n][8] + R_val*(m==n)
    float S[3][3];
    S[0][0] = HP[0][6]*H06 + HP[0][7]*H07 + HP[0][8]*H08 + R_val;
    S[0][1] = HP[0][6]*H16 + HP[0][7]*H17 + HP[0][8]*H18;
    S[0][2] = HP[0][6]*H26 + HP[0][7]*H27 + HP[0][8]*H28;
    S[1][0] = S[0][1];  // 対称
    S[1][1] = HP[1][6]*H16 + HP[1][7]*H17 + HP[1][8]*H18 + R_val;  // 修正: H1xを使用
    S[1][2] = HP[1][6]*H26 + HP[1][7]*H27 + HP[1][8]*H28;
    S[2][0] = S[0][2];  // 対称
    S[2][1] = S[1][2];  // 対称
    S[2][2] = HP[2][6]*H26 + HP[2][7]*H27 + HP[2][8]*H28 + R_val;  // 修正: H2xを使用

    // S^-1 (3x3 逆行列)
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;

    float Si[3][3];
    Si[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    Si[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * inv_det;
    Si[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    Si[1][0] = Si[0][1];  // 対称
    Si[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    Si[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det;
    Si[2][0] = Si[0][2];  // 対称
    Si[2][1] = Si[1][2];  // 対称
    Si[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    // PHT = P * H^T (15x3)
    // PHT[i][n] = P[i][6]*H[n][6] + P[i][7]*H[n][7] + P[i][8]*H[n][8]
    float PHT[15][3];
    for (int i = 0; i < 15; i++) {
        float Pi6 = P_(i, 6);
        float Pi7 = P_(i, 7);
        float Pi8 = P_(i, 8);
        PHT[i][0] = Pi6*H06 + Pi7*H07 + Pi8*H08;
        PHT[i][1] = Pi6*H16 + Pi7*H17 + Pi8*H18;
        PHT[i][2] = Pi6*H26 + Pi7*H27 + Pi8*H28;
    }

    // K = PHT * S^-1 (15x3)
    float K[15][3];
    for (int i = 0; i < 15; i++) {
        K[i][0] = PHT[i][0]*Si[0][0] + PHT[i][1]*Si[1][0] + PHT[i][2]*Si[2][0];
        K[i][1] = PHT[i][0]*Si[0][1] + PHT[i][1]*Si[1][1] + PHT[i][2]*Si[2][1];
        K[i][2] = PHT[i][0]*Si[0][2] + PHT[i][1]*Si[1][2] + PHT[i][2]*Si[2][2];
    }

    // dx = K * y
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
    }

    // 状態注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // I_KH[i][j] = delta_ij - K[i][0]*H[0][j] - K[i][1]*H[1][j] - K[i][2]*H[2][j]
    // Hは列6,7,8のみ非ゼロ

    // Step 1: temp1_ = (I - K*H) * P
    // temp1_[i][j] = P[i][j] - K[i][0]*HP[0][j] - K[i][1]*HP[1][j] - K[i][2]*HP[2][j]
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        float Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0*HP[0][j] - Ki1*HP[1][j] - Ki2*HP[2][j];
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    // temp1_i を列6,7,8について取得してI_KH^Tと乗算
    for (int i = 0; i < 15; i++) {
        float temp1_i6 = temp1_(i, 6);
        float temp1_i7 = temp1_(i, 7);
        float temp1_i8 = temp1_(i, 8);
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        float Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            // (I-KH)^T[k][j] の非ゼロ: k=j(1), k=6(-K[j][0]*H[0][6]-K[j][1]*H[1][6]-K[j][2]*H[2][6]), etc.
            float Kj0 = K[j][0];
            float Kj1 = K[j][1];
            float Kj2 = K[j][2];
            // temp1_ * (I-KH)^T = temp1_[i][j] - temp1_[i][6..8] * (K[j]*H)^T
            float corr = temp1_i6*(Kj0*H06 + Kj1*H16 + Kj2*H26)
                       + temp1_i7*(Kj0*H07 + Kj1*H17 + Kj2*H27)
                       + temp1_i8*(Kj0*H08 + Kj1*H18 + Kj2*H28);
            float val = temp1_(i, j) - corr;
            // K * R * K^T
            val += R_val * (Ki0*Kj0 + Ki1*Kj1 + Ki2*Kj2);
            P_(i, j) = val;
        }
    }
}

void ESKF::updateFlow(float flow_x, float flow_y, float height)
{
    updateFlowWithGyro(flow_x, flow_y, height, 0.0f, 0.0f);
}

void ESKF::updateFlowWithGyro(float flow_x, float flow_y, float distance,
                               float gyro_x, float gyro_y)
{
    if (!initialized_ || distance < config_.flow_min_height) return;

    // ========================================================================
    // 傾きチェック: R22 = cos(roll) × cos(pitch)
    // ========================================================================
    const Quaternion& q = state_.orientation;
    float R22 = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);

    if (R22 < config_.flow_tilt_cos_threshold) return;  // 傾きすぎ

    // ========================================================================
    // ジャイロ補償
    // ========================================================================
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;

    // ジャイロ補償係数（回帰分析で取得）
    constexpr float flow_scale = 0.23f;  // 実測キャリブレーション (2024-12-02)
    constexpr float k_xx = 1.35f * flow_scale;
    constexpr float k_xy = 9.30f * flow_scale;
    constexpr float k_yx = -2.65f * flow_scale;
    constexpr float k_yy = 0.0f * flow_scale;

    float flow_x_comp = flow_x - k_xx * gyro_x_corrected - k_xy * gyro_y_corrected;
    float flow_y_comp = flow_y - k_yx * gyro_x_corrected - k_yy * gyro_y_corrected;

    // ========================================================================
    // ボディフレームでの観測速度
    // distance: ToFセンサーの視線方向距離（傾き補正不要）
    // ========================================================================
    float vx_body_obs = flow_x_comp * distance;
    float vy_body_obs = flow_y_comp * distance;

    // ========================================================================
    // 回転行列要素（Body→NED）: R^T[i][j] = R[j][i]
    // v_body = R^T × v_ned より:
    //   vx_body = R00*vn + R10*ve + R20*vd
    //   vy_body = R01*vn + R11*ve + R21*vd
    // ========================================================================
    float R00 = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    float R01 = 2.0f*(q.x*q.y - q.w*q.z);
    float R10 = 2.0f*(q.x*q.y + q.w*q.z);
    float R11 = 1.0f - 2.0f*(q.x*q.x + q.z*q.z);
    float R20 = 2.0f*(q.x*q.z - q.w*q.y);
    float R21 = 2.0f*(q.y*q.z + q.w*q.x);

    // 状態からボディ速度を計算（期待値）
    float vn = state_.velocity.x;
    float ve = state_.velocity.y;
    float vd = state_.velocity.z;

    float vx_body_exp = R00*vn + R10*ve + R20*vd;
    float vy_body_exp = R01*vn + R11*ve + R21*vd;

    // ========================================================================
    // 観測残差 y = z - h（ボディフレーム）
    // ========================================================================
    float y0 = vx_body_obs - vx_body_exp;
    float y1 = vy_body_obs - vy_body_exp;

    float R_val = config_.flow_noise * config_.flow_noise;

    // ========================================================================
    // H行列（2×15）: 速度部分のみ非ゼロ
    // H[0][3]=R00, H[0][4]=R10, H[0][5]=R20
    // H[1][3]=R01, H[1][4]=R11, H[1][5]=R21
    // ========================================================================
    float H03 = R00, H04 = R10, H05 = R20;
    float H13 = R01, H14 = R11, H15 = R21;

    // ========================================================================
    // S = H * P * H^T + R (2×2)
    // ========================================================================
    float PHT[15][2];
    for (int i = 0; i < 15; i++) {
        PHT[i][0] = P_(i,3)*H03 + P_(i,4)*H04 + P_(i,5)*H05;
        PHT[i][1] = P_(i,3)*H13 + P_(i,4)*H14 + P_(i,5)*H15;
    }

    float S00 = H03*PHT[3][0] + H04*PHT[4][0] + H05*PHT[5][0] + R_val;
    float S01 = H03*PHT[3][1] + H04*PHT[4][1] + H05*PHT[5][1];
    float S11 = H13*PHT[3][1] + H14*PHT[4][1] + H15*PHT[5][1] + R_val;

    // S^-1 (2×2逆行列)
    float det = S00 * S11 - S01 * S01;
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;
    float Si00 = S11 * inv_det;
    float Si01 = -S01 * inv_det;
    float Si11 = S00 * inv_det;

    // ========================================================================
    // カルマンゲイン K = P * H^T * S^-1 (15×2)
    // ========================================================================
    float K[15][2];
    for (int i = 0; i < 15; i++) {
        K[i][0] = PHT[i][0]*Si00 + PHT[i][1]*Si01;
        K[i][1] = PHT[i][0]*Si01 + PHT[i][1]*Si11;
    }

    // ========================================================================
    // 状態更新 dx = K * y
    // ========================================================================
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i][0] * y0 + K[i][1] * y1;
    }

    // 状態注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // ========================================================================
    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // H行列は列3,4,5が非ゼロ
    // ========================================================================

    // Step 1: temp1_ = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        float IKH_i3 = -K[i][0]*H03 - K[i][1]*H13;
        float IKH_i4 = -K[i][0]*H04 - K[i][1]*H14;
        float IKH_i5 = -K[i][0]*H05 - K[i][1]*H15;

        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) + IKH_i3*P_(3, j) + IKH_i4*P_(4, j) + IKH_i5*P_(5, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float temp1_i3 = temp1_(i, 3);
        float temp1_i4 = temp1_(i, 4);
        float temp1_i5 = temp1_(i, 5);
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];

        for (int j = 0; j < 15; j++) {
            float IKH_j3 = -K[j][0]*H03 - K[j][1]*H13;
            float IKH_j4 = -K[j][0]*H04 - K[j][1]*H14;
            float IKH_j5 = -K[j][0]*H05 - K[j][1]*H15;

            float val = temp1_(i, j) + temp1_i3*IKH_j3 + temp1_i4*IKH_j4 + temp1_i5*IKH_j5;
            val += R_val * (Ki0 * K[j][0] + Ki1 * K[j][1]);
            P_(i, j) = val;
        }
    }
}

void ESKF::updateFlowRaw(int16_t flow_dx, int16_t flow_dy, float distance,
                          float dt, float gyro_x, float gyro_y)
{
    if (!initialized_ || distance < config_.flow_min_height || dt <= 0) return;

    // ================================================================
    // 1. フローオフセット補正 → ピクセル角速度 [rad/s]
    // ================================================================
    // オフセット補正（静止ホバリングキャリブレーションで取得）
    float flow_dx_corrected = static_cast<float>(flow_dx) - config_.flow_offset[0];
    float flow_dy_corrected = static_cast<float>(flow_dy) - config_.flow_offset[1];

    // PMW3901: FOV=42°, 35pixels → 0.0209 rad/pixel
    float flow_x_cam = flow_dx_corrected * config_.flow_rad_per_pixel / dt;
    float flow_y_cam = flow_dy_corrected * config_.flow_rad_per_pixel / dt;

    // ================================================================
    // 2. 回転成分除去（物理モデルに基づく補正）
    // ================================================================
    // ジャイロバイアス補正（predict()と同様）
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;

    // ========================================================================
    // 物理モデルに基づくジャイロ補正
    //
    // 下向きカメラの場合、機体の回転により地面が見かけ上動いて見える。
    // 機体が角速度 ω [rad/s] で回転すると、カメラから見た地面の
    // 見かけの角速度も ω [rad/s] となる。
    //
    // 重要: flow_x_cam は既に [rad/s] 単位に変換済みなので、
    //       ジャイロ補正も [rad/s] 単位で行う（1/rad_per_pixelは不要）
    //
    //   ピッチ回転（gyro_y, Y軸周り, 正=機首上げ）:
    //     → 地面が後方に移動して見える → flow_x に影響
    //
    //   ロール回転（gyro_x, X軸周り, 正=右翼下げ）:
    //     → 地面が左に移動して見える → flow_y に影響
    //
    // 符号はPMW3901のカメラ取り付け向きにより決定（要実測調整）
    //
    // flow_gyro_scale: 補正強度の調整係数（デフォルト1.0）
    // ========================================================================
    float gyro_scale = config_.flow_gyro_scale;

    // ピッチ（gyro_y）がカメラX軸フローに影響
    // 符号: 実測により決定（+gyro_y）
    float flow_rot_x_cam = gyro_scale * gyro_y_corrected;

    // ロール（gyro_x）がカメラY軸フローに影響
    // 符号: ロール右で地面が左に見える → 負のフロー → -gyro_x
    float flow_rot_y_cam = -gyro_scale * gyro_x_corrected;

    // カメラ座標系で回転成分を除去
    float flow_trans_x_cam = flow_x_cam - flow_rot_x_cam;
    float flow_trans_y_cam = flow_y_cam - flow_rot_y_cam;

    // ================================================================
    // 3. カメラ座標系 → 機体座標系
    // ================================================================
    // [flow_body_x]   [c2b_xx c2b_xy] [flow_cam_x]
    // [flow_body_y] = [c2b_yx c2b_yy] [flow_cam_y]
    float flow_trans_x = config_.flow_cam_to_body[0] * flow_trans_x_cam
                       + config_.flow_cam_to_body[1] * flow_trans_y_cam;
    float flow_trans_y = config_.flow_cam_to_body[2] * flow_trans_x_cam
                       + config_.flow_cam_to_body[3] * flow_trans_y_cam;

    // ================================================================
    // 4. 並進速度算出 [m/s]
    // ================================================================
    // v = ω × distance
    float vx_body = flow_trans_x * distance;
    float vy_body = flow_trans_y * distance;

    // ================================================================
    // 5. Body→NED変換
    // ================================================================
    float cos_yaw = std::cos(state_.yaw);
    float sin_yaw = std::sin(state_.yaw);

    float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;
    float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;

    // ========================================================================
    // 6. 疎行列展開による観測更新
    // H行列の非ゼロ要素: H[0][3]=1 (VEL_X), H[1][4]=1 (VEL_Y) のみ
    // ========================================================================

    // 観測残差 y = z - h
    float y0 = vx_ned - state_.velocity.x;
    float y1 = vy_ned - state_.velocity.y;

    float R_val = config_.flow_noise * config_.flow_noise;

    // S = H * P * H^T + R (2x2)
    float P33 = P_(3, 3);
    float P34 = P_(3, 4);
    float P44 = P_(4, 4);

    float S00 = P33 + R_val;
    float S01 = P34;
    float S11 = P44 + R_val;

    // S^-1 (2x2 逆行列)
    float det = S00 * S11 - S01 * S01;
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;
    float Si00 = S11 * inv_det;
    float Si01 = -S01 * inv_det;
    float Si11 = S00 * inv_det;

    // カルマンゲイン K (15x2)
    float K[15][2];
    for (int i = 0; i < 15; i++) {
        float Pi3 = P_(i, 3);
        float Pi4 = P_(i, 4);
        K[i][0] = Pi3 * Si00 + Pi4 * Si01;
        K[i][1] = Pi3 * Si01 + Pi4 * Si11;
    }

    // dx = K * y
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i][0] * y0 + K[i][1] * y1;
    }

    // 状態注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // Joseph形式共分散更新
    // Step 1: temp1_ = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0 * P_(3, j) - Ki1 * P_(4, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float temp1_i3 = temp1_(i, 3);
        float temp1_i4 = temp1_(i, 4);
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            float val = temp1_(i, j) - K[j][0] * temp1_i3 - K[j][1] * temp1_i4;
            val += R_val * (Ki0 * K[j][0] + Ki1 * K[j][1]);
            P_(i, j) = val;
        }
    }
}

void ESKF::updateAccelAttitude(const Vector3& accel)
{
    if (!initialized_) return;

    // バイアス補正された加速度
    float ax = accel.x - state_.accel_bias.x;
    float ay = accel.y - state_.accel_bias.y;
    float az = accel.z - state_.accel_bias.z;

    // 加速度ノルムをチェック（運動中は更新スキップ）
    float accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
    float gravity_diff = std::abs(accel_norm - config_.gravity);

    if (gravity_diff > config_.accel_motion_threshold) {
        return;
    }

    // ========================================================================
    // 3軸加速度観測によるカルマン更新
    // H行列は姿勢誤差（ATT_X=6, ATT_Y=7, ATT_Z=8）に対してのみ非ゼロ
    // ========================================================================

    // クォータニオンから回転行列の第3行を計算
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    float R20 = 2*(q1*q3 - q0*q2);
    float R21 = 2*(q2*q3 + q0*q1);
    float R22 = 1 - 2*(q1*q1 + q2*q2);

    // 期待される加速度計出力: g_expected = R^T * [0, 0, -g]
    float g = config_.gravity;
    float g_exp_x = -g * R20;
    float g_exp_y = -g * R21;
    float g_exp_z = -g * R22;

    // 観測残差 y = z - h（3次元）
    float y0 = ax - g_exp_x;
    float y1 = ay - g_exp_y;
    float y2 = az - g_exp_z;

    // ========================================================================
    // H行列（3x15）: 姿勢誤差と加速度バイアスに対して非ゼロ
    //
    // 観測モデル: h(x) = R^T * [0,0,-g] + accel_bias
    //
    // 姿勢誤差に対する微分: ∂h/∂δθ = [a_body]×
    // a_body = [g_exp_x, g_exp_y, g_exp_z] = [-g*R20, -g*R21, -g*R22]
    //
    // [a_body]× = |    0      -a_z     a_y  |   |    0       g*R22   -g*R21 |
    //             |   a_z       0     -a_x  | = | -g*R22       0      g*R20 |
    //             |  -a_y      a_x      0   |   |  g*R21    -g*R20      0   |
    //
    // 加速度バイアスに対する微分: ∂h/∂ba = I (3x3単位行列)
    // ========================================================================

    // 姿勢部分 (列6,7,8)
    float H06 = 0.0f,      H07 = g*R22,    H08 = -g*R21;
    float H16 = -g*R22,    H17 = 0.0f,     H18 = g*R20;
    float H26 = g*R21,     H27 = -g*R20,   H28 = 0.0f;
    // バイアス部分 (列12,13,14): H[0][12]=1, H[1][13]=1, H[2][14]=1

    // Adaptive R（水平加速度でスケーリング）
    float horiz_accel_sq = ax*ax + ay*ay;
    float R_scale = 1.0f + config_.k_adaptive * horiz_accel_sq;
    float base_noise_sq = config_.accel_att_noise * config_.accel_att_noise;
    float R_val = base_noise_sq * R_scale;

    // ========================================================================
    // P * H^T (15x3) - H行列は列6,7,8（姿勢）と列12,13,14（バイアス）が非ゼロ
    // PHT[i][m] = P[i][6]*Hm6 + P[i][7]*Hm7 + P[i][8]*Hm8 + P[i][12+m]*1
    // ========================================================================
    float PHT[15][3];
    for (int i = 0; i < 15; i++) {
        PHT[i][0] = P_(i,6)*H06 + P_(i,7)*H07 + P_(i,8)*H08 + P_(i,12);
        PHT[i][1] = P_(i,6)*H16 + P_(i,7)*H17 + P_(i,8)*H18 + P_(i,13);
        PHT[i][2] = P_(i,6)*H26 + P_(i,7)*H27 + P_(i,8)*H28 + P_(i,14);
    }

    // ========================================================================
    // S = H * P * H^T + R (3x3)
    // S[m][n] = Hm6*PHT[6][n] + Hm7*PHT[7][n] + Hm8*PHT[8][n] + 1*PHT[12+m][n]
    // ========================================================================
    float S[3][3];
    S[0][0] = H06*PHT[6][0] + H07*PHT[7][0] + H08*PHT[8][0] + PHT[12][0] + R_val;
    S[0][1] = H06*PHT[6][1] + H07*PHT[7][1] + H08*PHT[8][1] + PHT[12][1];
    S[0][2] = H06*PHT[6][2] + H07*PHT[7][2] + H08*PHT[8][2] + PHT[12][2];
    S[1][0] = H16*PHT[6][0] + H17*PHT[7][0] + H18*PHT[8][0] + PHT[13][0];
    S[1][1] = H16*PHT[6][1] + H17*PHT[7][1] + H18*PHT[8][1] + PHT[13][1] + R_val;
    S[1][2] = H16*PHT[6][2] + H17*PHT[7][2] + H18*PHT[8][2] + PHT[13][2];
    S[2][0] = H26*PHT[6][0] + H27*PHT[7][0] + H28*PHT[8][0] + PHT[14][0];
    S[2][1] = H26*PHT[6][1] + H27*PHT[7][1] + H28*PHT[8][1] + PHT[14][1];
    S[2][2] = H26*PHT[6][2] + H27*PHT[7][2] + H28*PHT[8][2] + PHT[14][2] + R_val;

    // ========================================================================
    // S^-1 (3x3 逆行列)
    // ========================================================================
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;

    float Si[3][3];
    Si[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    Si[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * inv_det;
    Si[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    Si[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) * inv_det;
    Si[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    Si[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det;
    Si[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    Si[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) * inv_det;
    Si[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    // ========================================================================
    // カルマンゲイン K = P * H^T * S^-1 (15x3)
    // ========================================================================
    float K[15][3];
    for (int i = 0; i < 15; i++) {
        K[i][0] = PHT[i][0]*Si[0][0] + PHT[i][1]*Si[1][0] + PHT[i][2]*Si[2][0];
        K[i][1] = PHT[i][0]*Si[0][1] + PHT[i][1]*Si[1][1] + PHT[i][2]*Si[2][1];
        K[i][2] = PHT[i][0]*Si[0][2] + PHT[i][1]*Si[1][2] + PHT[i][2]*Si[2][2];
    }

    // ========================================================================
    // dx = K * y (15次元)
    // ========================================================================
    float dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
    }

    // ========================================================================
    // ヨー更新の無効化（加速度観測はヨーを可観測でないため）
    //
    // 問題: 機体が傾いている場合、H行列のATT_Z列が非ゼロになり、
    //       加速度外乱がヨー推定をジャンプさせる。
    //
    // H08 = -g × R21 = -g × sin(roll) × cos(pitch)
    // H18 =  g × R20 = -g × sin(pitch)
    //
    // 物理的には加速度計は重力方向のみ観測可能であり、
    // ヨー（鉛直軸周り回転）は観測できない。
    // ヨー推定は地磁気計(updateMag)とジャイロ積分(predict)に委ねる。
    //
    // 詳細: docs/eskf_accel_yaw_coupling.md
    // 元に戻す場合: 以下の行をコメントアウト
    // ========================================================================
    dx[ATT_Z] = 0.0f;

    // ========================================================================
    // 状態注入
    // ========================================================================
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx[BG_Z];
    }
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx[BA_X];
        state_.accel_bias.y += dx[BA_Y];
        state_.accel_bias.z += dx[BA_Z];
    }

    // ========================================================================
    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // H行列は列6,7,8（姿勢）と列12,13,14（バイアス）が非ゼロ
    // (I-K*H)[i][j] = δij - K[i][0]*H0j - K[i][1]*H1j - K[i][2]*H2j
    // ========================================================================

    // Step 1: temp1_ = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        // (I-K*H)の行iを事前計算: 列6,7,8と列12,13,14が非単位
        // 姿勢部分
        float IKH_i6 = -K[i][0]*H06 - K[i][1]*H16 - K[i][2]*H26;
        float IKH_i7 = -K[i][0]*H07 - K[i][1]*H17 - K[i][2]*H27;
        float IKH_i8 = -K[i][0]*H08 - K[i][1]*H18 - K[i][2]*H28;
        // バイアス部分: H[m][12+m] = 1
        float IKH_i12 = -K[i][0];
        float IKH_i13 = -K[i][1];
        float IKH_i14 = -K[i][2];

        for (int j = 0; j < 15; j++) {
            // temp1_[i][j] = sum_k (I-K*H)[i][k] * P[k][j]
            temp1_(i, j) = P_(i, j)
                         + IKH_i6*P_(6, j) + IKH_i7*P_(7, j) + IKH_i8*P_(8, j)
                         + IKH_i12*P_(12, j) + IKH_i13*P_(13, j) + IKH_i14*P_(14, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0], Ki1 = K[i][1], Ki2 = K[i][2];

        for (int j = 0; j < 15; j++) {
            // (I-K*H)^T の列j = (I-K*H)の行j
            // 姿勢部分
            float IKH_j6 = -K[j][0]*H06 - K[j][1]*H16 - K[j][2]*H26;
            float IKH_j7 = -K[j][0]*H07 - K[j][1]*H17 - K[j][2]*H27;
            float IKH_j8 = -K[j][0]*H08 - K[j][1]*H18 - K[j][2]*H28;
            // バイアス部分
            float IKH_j12 = -K[j][0];
            float IKH_j13 = -K[j][1];
            float IKH_j14 = -K[j][2];

            float val = temp1_(i, j)
                      + temp1_(i, 6)*IKH_j6 + temp1_(i, 7)*IKH_j7 + temp1_(i, 8)*IKH_j8
                      + temp1_(i, 12)*IKH_j12 + temp1_(i, 13)*IKH_j13 + temp1_(i, 14)*IKH_j14;

            // K * R * K^T (Rはスカラー×単位行列)
            val += R_val * (Ki0*K[j][0] + Ki1*K[j][1] + Ki2*K[j][2]);

            P_(i, j) = val;
        }
    }
}

void ESKF::injectErrorState(const Matrix<15, 1>& dx)
{
    state_.position.x += dx(POS_X, 0);
    state_.position.y += dx(POS_Y, 0);
    state_.position.z += dx(POS_Z, 0);

    state_.velocity.x += dx(VEL_X, 0);
    state_.velocity.y += dx(VEL_Y, 0);
    state_.velocity.z += dx(VEL_Z, 0);

    Vector3 dtheta(dx(ATT_X, 0), dx(ATT_Y, 0), dx(ATT_Z, 0));
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    // mag_enabled=false時はYaw=0に固定
    if (!config_.mag_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    state_.gyro_bias.x += dx(BG_X, 0);
    state_.gyro_bias.y += dx(BG_Y, 0);
    // mag_enabled=false時はGyro Bias Zを更新しない
    if (config_.mag_enabled) {
        state_.gyro_bias.z += dx(BG_Z, 0);
    }

    // 加速度バイアス推定がフリーズ中は状態を更新しない
    // （接地中など可観測性がない状況でバイアス破壊を防止）
    if (!freeze_accel_bias_) {
        state_.accel_bias.x += dx(BA_X, 0);
        state_.accel_bias.y += dx(BA_Y, 0);
        state_.accel_bias.z += dx(BA_Z, 0);
    }
}

template<int M>
bool ESKF::measurementUpdate(const Matrix<M, 1>& z,
                              const Matrix<M, 1>& h,
                              const Matrix<M, 15>& H,
                              const Matrix<M, M>& R)
{
    Matrix<M, 1> y = z - h;

    Matrix<M, 15> HP = H * P_;
    Matrix<M, M> S = HP * H.transpose() + R;

    Matrix<15, M> PHT = P_ * H.transpose();
    Matrix<M, M> S_inv = inverse<M>(S);
    Matrix<15, M> K = PHT * S_inv;

    Matrix<15, 1> dx = K * y;
    injectErrorState(dx);

    // Joseph形式共分散更新
    temp1_ = Matrix<15, 15>::identity() - K * H;
    temp2_ = temp1_ * P_;
    F_ = temp2_ * temp1_.transpose();

    Matrix<15, M> KR = K * R;
    Q_ = KR * K.transpose();
    P_ = F_ + Q_;

    return true;
}

// テンプレートの明示的インスタンス化
template bool ESKF::measurementUpdate<1>(const Matrix<1, 1>&, const Matrix<1, 1>&,
                                          const Matrix<1, 15>&, const Matrix<1, 1>&);
template bool ESKF::measurementUpdate<2>(const Matrix<2, 1>&, const Matrix<2, 1>&,
                                          const Matrix<2, 15>&, const Matrix<2, 2>&);
template bool ESKF::measurementUpdate<3>(const Matrix<3, 1>&, const Matrix<3, 1>&,
                                          const Matrix<3, 15>&, const Matrix<3, 3>&);

// ============================================================================
// AttitudeEstimator Implementation (Complementary Filter)
// ============================================================================

esp_err_t AttitudeEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AttitudeEstimator (Complementary Filter)");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void AttitudeEstimator::update(const Vector3& accel, const Vector3& gyro, float dt)
{
    if (!initialized_ || dt <= 0) return;

    Vector3 gyro_corrected = gyro - state_.gyro_bias;

    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    Quaternion q_gyro = state_.orientation * dq;
    q_gyro.normalize();

    float accel_norm = accel.norm();
    if (accel_norm > 0.5f && accel_norm < 3.0f * 9.81f) {
        float roll_acc = std::atan2(accel.y, accel.z);
        float pitch_acc = std::atan2(-accel.x, std::sqrt(accel.y*accel.y + accel.z*accel.z));

        float roll_gyro, pitch_gyro, yaw_gyro;
        q_gyro.toEuler(roll_gyro, pitch_gyro, yaw_gyro);

        float alpha = config_.gyro_weight;
        float roll_fused = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
        float pitch_fused = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

        state_.orientation = Quaternion::fromEuler(roll_fused, pitch_fused, yaw_gyro);
    } else {
        state_.orientation = q_gyro;
    }

    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);
}

void AttitudeEstimator::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    float cos_roll = std::cos(state_.roll);
    float sin_roll = std::sin(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float sin_pitch = std::sin(state_.pitch);

    float mag_x = mag.x * cos_pitch + mag.z * sin_pitch;
    float mag_y = mag.x * sin_roll * sin_pitch + mag.y * cos_roll - mag.z * sin_roll * cos_pitch;

    float yaw_mag = std::atan2(-mag_y, mag_x) + config_.mag_declination;

    float alpha = config_.gyro_weight;
    state_.yaw = alpha * state_.yaw + (1.0f - alpha) * yaw_mag;

    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, state_.yaw);
}

void AttitudeEstimator::reset()
{
    state_.orientation = Quaternion::identity();
    state_.roll = state_.pitch = state_.yaw = 0.0f;
    state_.gyro_bias = Vector3::zero();
}

// ============================================================================
// AltitudeEstimator Implementation (Simple Kalman Filter)
// ============================================================================

esp_err_t AltitudeEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AltitudeEstimator (Kalman Filter)");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void AltitudeEstimator::predict(float accel_z, float dt)
{
    if (!initialized_ || dt <= 0) return;

    state_.altitude += state_.velocity * dt + 0.5f * accel_z * dt * dt;
    state_.velocity += accel_z * dt;

    float F[2][2] = {{1, dt}, {0, 1}};
    float FP[2][2];
    float FPFt[2][2];

    FP[0][0] = F[0][0] * P_[0][0] + F[0][1] * P_[1][0];
    FP[0][1] = F[0][0] * P_[0][1] + F[0][1] * P_[1][1];
    FP[1][0] = F[1][0] * P_[0][0] + F[1][1] * P_[1][0];
    FP[1][1] = F[1][0] * P_[0][1] + F[1][1] * P_[1][1];

    FPFt[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[0][1];
    FPFt[0][1] = FP[0][0] * F[1][0] + FP[0][1] * F[1][1];
    FPFt[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[0][1];
    FPFt[1][1] = FP[1][0] * F[1][0] + FP[1][1] * F[1][1];

    P_[0][0] = FPFt[0][0] + config_.process_noise_alt;
    P_[0][1] = FPFt[0][1];
    P_[1][0] = FPFt[1][0];
    P_[1][1] = FPFt[1][1] + config_.process_noise_vel;
}

void AltitudeEstimator::updateBaro(float altitude)
{
    if (!initialized_) return;

    float y = altitude - state_.altitude;
    float S = P_[0][0] + config_.measurement_noise_baro;
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    float P_new[2][2];
    P_new[0][0] = I_KH[0][0] * P_[0][0] + I_KH[0][1] * P_[1][0];
    P_new[0][1] = I_KH[0][0] * P_[0][1] + I_KH[0][1] * P_[1][1];
    P_new[1][0] = I_KH[1][0] * P_[0][0] + I_KH[1][1] * P_[1][0];
    P_new[1][1] = I_KH[1][0] * P_[0][1] + I_KH[1][1] * P_[1][1];

    P_[0][0] = P_new[0][0];
    P_[0][1] = P_new[0][1];
    P_[1][0] = P_new[1][0];
    P_[1][1] = P_new[1][1];
}

void AltitudeEstimator::updateToF(float distance, float pitch, float roll)
{
    if (!initialized_) return;

    float cos_roll = std::cos(roll);
    float cos_pitch = std::cos(pitch);
    float height = distance * cos_roll * cos_pitch;

    float y = height - state_.altitude;
    float S = P_[0][0] + config_.measurement_noise_tof;
    float K[2] = {P_[0][0] / S, P_[1][0] / S};

    state_.altitude += K[0] * y;
    state_.velocity += K[1] * y;

    float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    float P_new[2][2];
    P_new[0][0] = I_KH[0][0] * P_[0][0] + I_KH[0][1] * P_[1][0];
    P_new[0][1] = I_KH[0][0] * P_[0][1] + I_KH[0][1] * P_[1][1];
    P_new[1][0] = I_KH[1][0] * P_[0][0] + I_KH[1][1] * P_[1][0];
    P_new[1][1] = I_KH[1][0] * P_[0][1] + I_KH[1][1] * P_[1][1];

    P_[0][0] = P_new[0][0];
    P_[0][1] = P_new[0][1];
    P_[1][0] = P_new[1][0];
    P_[1][1] = P_new[1][1];
}

void AltitudeEstimator::reset()
{
    state_.altitude = 0.0f;
    state_.velocity = 0.0f;
    P_[0][0] = 1.0f; P_[0][1] = 0.0f;
    P_[1][0] = 0.0f; P_[1][1] = 1.0f;
}

// ============================================================================
// VelocityEstimator Implementation
// ============================================================================

esp_err_t VelocityEstimator::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing VelocityEstimator");
    config_ = config;
    reset();
    initialized_ = true;
    return ESP_OK;
}

void VelocityEstimator::updateFlow(float flow_x, float flow_y, float height, float gyro_x, float gyro_y)
{
    if (!initialized_ || height < 0.1f) return;

    // ジャイロバイアス補正
    float gyro_x_corrected = gyro_x - gyro_bias_x_;
    float gyro_y_corrected = gyro_y - gyro_bias_y_;

    float flow_x_comp = flow_x - gyro_y_corrected;
    float flow_y_comp = flow_y + gyro_x_corrected;

    state_.vx = flow_y_comp * height * config_.flow_scale;
    state_.vy = -flow_x_comp * height * config_.flow_scale;
}

void VelocityEstimator::reset()
{
    state_.vx = 0.0f;
    state_.vy = 0.0f;
}

}  // namespace stampfly
