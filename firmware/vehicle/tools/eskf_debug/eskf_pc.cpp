/**
 * @file eskf_pc.cpp
 * @brief ESKF Implementation for PC (ESP-IDF dependencies removed)
 *
 * SYNC: This file should be kept in sync with components/stampfly_eskf/eskf.cpp
 *
 * Error-State Kalman Filter (ESKF) による統合推定器
 * 15状態: [位置(3), 速度(3), 姿勢誤差(3), ジャイロバイアス(3), 加速度バイアス(3)]
 */

#include "eskf.hpp"
#include <cstdio>

// Stub for ESP logging
#define ESP_LOGI(tag, fmt, ...) printf("[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) printf("[ERROR] %s: " fmt "\n", tag, ##__VA_ARGS__)
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

void ESKF::predict(const Vector3& accel, const Vector3& gyro, float dt)
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

    // ワールド座標系での加速度 = R * accel_corrected + gravity
    float ax = accel_corrected.x, ay = accel_corrected.y, az = accel_corrected.z;
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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

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
    // 疎行列展開による観測更新（デバイス版と同一）
    // H行列の非ゼロ要素: 列6,7,8（ATT_X, ATT_Y, ATT_Z）のみ
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
    float H06 = 0.0f,  H07 = -hz, H08 = hy;
    float H16 = hz,    H17 = 0.0f, H18 = -hx;
    float H26 = -hy,   H27 = hx,   H28 = 0.0f;

    float R_val = config_.mag_noise * config_.mag_noise;

    // HP = H * P (3x15)
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
    float S[3][3];
    S[0][0] = HP[0][6]*H06 + HP[0][7]*H07 + HP[0][8]*H08 + R_val;
    S[0][1] = HP[0][6]*H16 + HP[0][7]*H17 + HP[0][8]*H18;
    S[0][2] = HP[0][6]*H26 + HP[0][7]*H27 + HP[0][8]*H28;
    S[1][0] = S[0][1];
    S[1][1] = HP[1][6]*H16 + HP[1][7]*H17 + HP[1][8]*H18 + R_val;  // 修正済み
    S[1][2] = HP[1][6]*H26 + HP[1][7]*H27 + HP[1][8]*H28;
    S[2][0] = S[0][2];
    S[2][1] = S[1][2];
    S[2][2] = HP[2][6]*H26 + HP[2][7]*H27 + HP[2][8]*H28 + R_val;  // 修正済み

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
    Si[1][0] = Si[0][1];
    Si[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    Si[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * inv_det;
    Si[2][0] = Si[0][2];
    Si[2][1] = Si[1][2];
    Si[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    // PHT = P * H^T (15x3)
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

    // ヨー更新の無効化（加速度観測はヨーを可観測でないため）
    // 詳細: docs/eskf_accel_yaw_coupling.md
    // 元に戻す場合: 以下の行をコメントアウト
    dx[ATT_Z] = 0.0f;

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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

    // Joseph形式共分散更新
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        float Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0*HP[0][j] - Ki1*HP[1][j] - Ki2*HP[2][j];
        }
    }

    for (int i = 0; i < 15; i++) {
        float temp1_i6 = temp1_(i, 6);
        float temp1_i7 = temp1_(i, 7);
        float temp1_i8 = temp1_(i, 8);
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        float Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            float Kj0 = K[j][0];
            float Kj1 = K[j][1];
            float Kj2 = K[j][2];
            float corr = temp1_i6*(Kj0*H06 + Kj1*H16 + Kj2*H26)
                       + temp1_i7*(Kj0*H07 + Kj1*H17 + Kj2*H27)
                       + temp1_i8*(Kj0*H08 + Kj1*H18 + Kj2*H28);
            float val = temp1_(i, j) - corr;
            val += R_val * (Ki0*Kj0 + Ki1*Kj1 + Ki2*Kj2);
            P_(i, j) = val;
        }
    }
}

void ESKF::updateFlow(float flow_x, float flow_y, float height)
{
    updateFlowWithGyro(flow_x, flow_y, height, 0.0f, 0.0f);
}

void ESKF::updateFlowWithGyro(float flow_x, float flow_y, float height,
                               float gyro_x, float gyro_y)
{
    if (!initialized_ || height < config_.flow_min_height) return;

    // ジャイロバイアス補正（predict()と同様）
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;

    // ジャイロ補償係数（回帰分析で取得）
    constexpr float flow_scale = 0.23f;  // 実測キャリブレーション (2024-12-02)
    constexpr float k_xx = 1.35f * flow_scale;
    constexpr float k_xy = 9.30f * flow_scale;
    constexpr float k_yx = -2.65f * flow_scale;
    constexpr float k_yy = 0.0f * flow_scale;

    // フローオフセット補正（動的検討時に再評価）
    constexpr float flow_dx_offset = 0.0f;
    constexpr float flow_dy_offset = 0.0f;

    float flow_x_comp = flow_x - k_xx * gyro_x_corrected - k_xy * gyro_y_corrected - flow_dx_offset;
    float flow_y_comp = flow_y - k_yx * gyro_x_corrected - k_yy * gyro_y_corrected - flow_dy_offset;

    // ボディ座標系での速度
    float vx_body = flow_x_comp * height;
    float vy_body = flow_y_comp * height;

    // Body→NED変換
    float cos_yaw = std::cos(state_.yaw);
    float sin_yaw = std::sin(state_.yaw);

    float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;
    float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;

    // ========================================================================
    // 疎行列展開による観測更新
    // H行列の非ゼロ要素: H[0][3]=1 (VEL_X), H[1][4]=1 (VEL_Y) のみ
    // ========================================================================

    // 観測残差 y = z - h
    float y0 = vx_ned - state_.velocity.x;
    float y1 = vy_ned - state_.velocity.y;

    float R_val = config_.flow_noise * config_.flow_noise;

    // S = H * P * H^T + R (2x2)
    // S[0][0] = P[3][3] + R_val
    // S[0][1] = S[1][0] = P[3][4]
    // S[1][1] = P[4][4] + R_val
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
    // K[i][0] = P[i][3]*Si00 + P[i][4]*Si01
    // K[i][1] = P[i][3]*Si01 + P[i][4]*Si11
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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // I_KH[i][j] = delta_ij - K[i][0]*(j==3) - K[i][1]*(j==4)

    // Step 1: temp1_ = (I - K*H) * P
    // temp1_[i][j] = P[i][j] - K[i][0]*P[3][j] - K[i][1]*P[4][j]
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0 * P_(3, j) - Ki1 * P_(4, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    // P'[i][j] = temp1_[i][j] - K[j][0]*temp1_[i][3] - K[j][1]*temp1_[i][4]
    //          + R_val * (K[i][0]*K[j][0] + K[i][1]*K[j][1])
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
    // 2. 回転成分除去（カメラ座標系で実施）
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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

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
    float accel_corrected_x = accel.x - state_.accel_bias.x;
    float accel_corrected_y = accel.y - state_.accel_bias.y;
    float accel_corrected_z = accel.z - state_.accel_bias.z;

    // 加速度ノルムをチェック（垂直方向の大きな加速を検出）
    float accel_norm = std::sqrt(accel_corrected_x*accel_corrected_x +
                                  accel_corrected_y*accel_corrected_y +
                                  accel_corrected_z*accel_corrected_z);
    float gravity_diff = std::abs(accel_norm - config_.gravity);

    if (gravity_diff > config_.accel_motion_threshold) {
        return;
    }

    // ========================================================================
    // 疎行列展開による観測更新
    // H行列の非ゼロ要素: H[0][7]=g, H[1][6]=-g のみ
    // ========================================================================

    // クォータニオンから必要な回転行列要素を直接計算
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    // R^T の第3列 = R の第3行
    float R20 = 2*(q1*q3 - q0*q2);
    float R21 = 2*(q2*q3 + q0*q1);

    // 期待される加速度計出力: g_expected = R^T * [0, 0, -g]
    float neg_g = -config_.gravity;
    float g_expected_x = R20 * neg_g;
    float g_expected_y = R21 * neg_g;

    // 観測残差 y = z - h（バイアス補正済み加速度を使用）
    float y0 = accel_corrected_x - g_expected_x;
    float y1 = accel_corrected_y - g_expected_y;

    // H行列定数
    float g = config_.gravity;
    float g2 = g * g;

    // Adaptive R（バイアス補正済み加速度を使用）
    float horiz_accel_sq = accel_corrected_x * accel_corrected_x +
                           accel_corrected_y * accel_corrected_y;
    float R_scale = 1.0f + config_.k_adaptive * horiz_accel_sq;
    float base_noise_sq = config_.accel_att_noise * config_.accel_att_noise;
    float R_val = base_noise_sq * R_scale;

    // S = H * P * H^T + R (2x2)
    // S[0][0] = g^2 * P[7][7] + R_val
    // S[0][1] = S[1][0] = -g^2 * P[6][7]
    // S[1][1] = g^2 * P[6][6] + R_val
    float P66 = P_(6, 6);
    float P67 = P_(6, 7);
    float P77 = P_(7, 7);

    float S00 = g2 * P77 + R_val;
    float S01 = -g2 * P67;
    float S11 = g2 * P66 + R_val;

    // S^-1 (2x2 逆行列)
    float det = S00 * S11 - S01 * S01;
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;
    float Si00 = S11 * inv_det;
    float Si01 = -S01 * inv_det;
    float Si11 = S00 * inv_det;

    // カルマンゲイン K (15x2)
    // PHT[i][0] = P[i][7] * g
    // PHT[i][1] = P[i][6] * (-g)
    // K[i][0] = PHT[i][0] * Si00 + PHT[i][1] * Si01
    // K[i][1] = PHT[i][0] * Si01 + PHT[i][1] * Si11
    float K[15][2];
    for (int i = 0; i < 15; i++) {
        float PHT_i0 = P_(i, 7) * g;
        float PHT_i1 = P_(i, 6) * (-g);
        K[i][0] = PHT_i0 * Si00 + PHT_i1 * Si01;
        K[i][1] = PHT_i0 * Si01 + PHT_i1 * Si11;
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
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];

    // Joseph形式共分散更新: P' = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    // I_KH[i][j] = delta_ij - K[i][0]*g*(j==7) + K[i][1]*g*(j==6)

    // Step 1: temp1_ = (I - K*H) * P
    // temp1_[i][j] = P[i][j] - K[i][0]*g*P[7][j] + K[i][1]*g*P[6][j]
    for (int i = 0; i < 15; i++) {
        float Ki0_g = K[i][0] * g;
        float Ki1_g = K[i][1] * g;
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0_g * P_(7, j) + Ki1_g * P_(6, j);
        }
    }

    // Step 2: P' = temp1_ * (I - K*H)^T + K * R * K^T
    // P'[i][j] = temp1_[i][j] - K[j][0]*g*temp1_[i][7] + K[j][1]*g*temp1_[i][6]
    //          + R_val * (K[i][0]*K[j][0] + K[i][1]*K[j][1])
    for (int i = 0; i < 15; i++) {
        float temp1_i6 = temp1_(i, 6);
        float temp1_i7 = temp1_(i, 7);
        float Ki0 = K[i][0];
        float Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            float Kj0_g = K[j][0] * g;
            float Kj1_g = K[j][1] * g;
            float val = temp1_(i, j) - Kj0_g * temp1_i7 + Kj1_g * temp1_i6;
            val += R_val * (Ki0 * K[j][0] + Ki1 * K[j][1]);
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

    state_.accel_bias.x += dx(BA_X, 0);
    state_.accel_bias.y += dx(BA_Y, 0);
    state_.accel_bias.z += dx(BA_Z, 0);
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
