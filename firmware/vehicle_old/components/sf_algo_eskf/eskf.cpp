/**
 * @file eskf.cpp
 * @brief ESKF Implementation - active_mask based P-matrix isolation
 *
 * Key features:
 * - active_mask based Q gating in predict()
 * - Unified dx masking in applyMaskedErrorState()
 * - enforceCovarianceConstraints() after every predict/update
 */

#include "eskf.hpp"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "ESKF";

namespace stampfly {

// ============================================================================
// Init / Reset
// ============================================================================

esp_err_t ESKF::init(const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ESKF (15-state, active_mask)");
    config_ = config;

    // Cache initial P diagonal values
    // P行列初期対角値をキャッシュ
    float pos_var = config_.init_pos_std * config_.init_pos_std;
    float vel_var = config_.init_vel_std * config_.init_vel_std;
    float att_var = config_.init_att_std * config_.init_att_std;
    float bg_var  = config_.init_gyro_bias_std * config_.init_gyro_bias_std;
    float ba_var  = config_.init_accel_bias_std * config_.init_accel_bias_std;

    init_P_diag_[POS_X] = pos_var; init_P_diag_[POS_Y] = pos_var; init_P_diag_[POS_Z] = pos_var;
    init_P_diag_[VEL_X] = vel_var; init_P_diag_[VEL_Y] = vel_var; init_P_diag_[VEL_Z] = vel_var;
    init_P_diag_[ATT_X] = att_var; init_P_diag_[ATT_Y] = att_var; init_P_diag_[ATT_Z] = att_var;
    init_P_diag_[BG_X]  = bg_var;  init_P_diag_[BG_Y]  = bg_var;  init_P_diag_[BG_Z]  = bg_var;
    init_P_diag_[BA_X]  = ba_var;  init_P_diag_[BA_Y]  = ba_var;  init_P_diag_[BA_Z]  = ba_var;

    reset();
    initialized_ = true;
    ESP_LOGI(TAG, "ESKF initialized (active_mask=0x%04X)", active_mask_);

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

    // Initialize P with cached diagonal
    // キャッシュされた対角値でPを初期化
    P_ = Matrix<15, 15>::zeros();
    for (int i = 0; i < N_STATES; i++) {
        P_(i, i) = init_P_diag_[i];
    }

    // Recompute active mask from config
    // コンフィグからactive_maskを再計算
    recomputeActiveMask();

    // Enforce constraints for initially frozen states
    // 初期凍結状態の制約を適用
    enforceCovarianceConstraints();
}

void ESKF::resetPositionVelocity()
{
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();

    // Clear position-velocity block and cross-covariance
    // 位置・速度ブロックとクロス共分散をクリア
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 15; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }

    // Set diagonal
    // 対角成分を設定
    for (int i = 0; i < 6; i++) {
        P_(i, i) = init_P_diag_[i];
    }

    enforceCovarianceConstraints();
}

void ESKF::holdPositionVelocity()
{
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
}

void ESKF::resetForLanding()
{
    state_.position = Vector3::zero();
    state_.velocity = Vector3::zero();
    freeze_accel_bias_ = true;
    recomputeActiveMask();

    // Clear position-velocity block
    // 位置・速度ブロックをクリア
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 15; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }
    for (int i = 0; i < 6; i++) {
        P_(i, i) = init_P_diag_[i];
    }

    // Clear accel bias cross-covariance with attitude/gyro bias
    // 加速度バイアスと姿勢/ジャイロバイアスのクロス共分散をクリア
    for (int i = BA_X; i <= BA_Z; i++) {
        for (int j = ATT_X; j <= BG_Z; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }

    // Shrink gyro bias covariance (trust flight estimate)
    // ジャイロバイアス共分散を縮小 (飛行中の推定値を信頼)
    float gb_var = init_P_diag_[BG_X] * 0.01f;
    P_(BG_X, BG_X) = gb_var;
    P_(BG_Y, BG_Y) = gb_var;
    P_(BG_Z, BG_Z) = gb_var;

    // Clear attitude-gyro bias cross-covariance
    // 姿勢-ジャイロバイアスのクロス共分散をクリア
    for (int i = ATT_X; i <= ATT_Z; i++) {
        for (int j = BG_X; j <= BG_Z; j++) {
            P_(i, j) = 0.0f;
            P_(j, i) = 0.0f;
        }
    }

    enforceCovarianceConstraints();
}

// ============================================================================
// Sensor enable/disable
// ============================================================================

void ESKF::setSensorEnabled(SensorGroup group, bool enabled)
{
    if (group >= SENSOR_COUNT) return;
    config_.sensor_enabled[group] = enabled;
    recomputeActiveMask();
    enforceCovarianceConstraints();
}

void ESKF::recomputeActiveMask()
{
    // Start with all states active
    // 全状態をアクティブで開始
    uint16_t mask = 0x7FFF;

    // For each disabled sensor, clear its frozen bits
    // 無効センサの凍結ビットをクリア
    for (int g = 0; g < SENSOR_COUNT; g++) {
        if (!config_.sensor_enabled[g]) {
            mask &= ~SENSOR_MASKS[g];
        }
    }

    // Baro and ToF share the same frozen states (POS_Z, VEL_Z, BA_Z)
    // If either is enabled, those states should stay active
    // 気圧計とToFは同じ凍結状態を共有 — どちらかが有効なら凍結しない
    if (config_.sensor_enabled[SENSOR_BARO] || config_.sensor_enabled[SENSOR_TOF]) {
        mask |= MASK_BARO;  // Re-enable the shared bits
    }

    // Freeze accel bias when requested
    // 加速度バイアスフリーズ要求時
    if (freeze_accel_bias_) {
        mask &= ~((1u << BA_X) | (1u << BA_Y) | (1u << BA_Z));
    }

    active_mask_ = mask;
}

// ============================================================================
// Covariance enforcement
// ============================================================================

void ESKF::enforceCovarianceConstraints()
{
    // For each frozen state: reset diagonal to initial, zero cross-covariance
    // 凍結状態: 対角を初期値に、クロス共分散をゼロに
    for (int i = 0; i < N_STATES; i++) {
        if ((active_mask_ >> i) & 1) continue;  // Active, skip / アクティブ、スキップ

        // Reset diagonal to initial value
        // 対角を初期値にリセット
        P_(i, i) = init_P_diag_[i];

        // Zero all cross-covariance
        // 全クロス共分散をゼロ化
        for (int j = 0; j < N_STATES; j++) {
            if (i != j) {
                P_(i, j) = 0.0f;
                P_(j, i) = 0.0f;
            }
        }
    }

    // Symmetry and diagonal floor for active states
    // アクティブ状態の対称性と対角下限
    for (int i = 0; i < N_STATES; i++) {
        if (P_(i, i) < 1e-12f) {
            P_(i, i) = 1e-12f;
        }
        for (int j = i + 1; j < N_STATES; j++) {
            float avg = 0.5f * (P_(i, j) + P_(j, i));
            P_(i, j) = avg;
            P_(j, i) = avg;
        }
    }
}

// ============================================================================
// Masked error state injection
// ============================================================================

void ESKF::applyMaskedErrorState(float dx[N_STATES])
{
    // Mask frozen states
    // 凍結状態をマスク
    for (int i = 0; i < N_STATES; i++) {
        if (!((active_mask_ >> i) & 1)) {
            dx[i] = 0.0f;
        }
    }

    // Attitude protection: clamp corrections, disable yaw from non-mag observations
    // 姿勢保護: 補正をクランプ、非磁気観測からのヨー更新を無効化
    float clamp = config_.att_correction_clamp;
    dx[ATT_X] = std::max(-clamp, std::min(clamp, dx[ATT_X]));
    dx[ATT_Y] = std::max(-clamp, std::min(clamp, dx[ATT_Y]));
    // Note: ATT_Z clamping is caller-specific (mag allows it, others zero it)
    // ATT_Z のクランプは呼び出し元固有 (magは許可、他はゼロ化)

    // Inject position
    // 位置注入
    state_.position.x += dx[POS_X];
    state_.position.y += dx[POS_Y];
    state_.position.z += dx[POS_Z];

    // Inject velocity
    // 速度注入
    state_.velocity.x += dx[VEL_X];
    state_.velocity.y += dx[VEL_Y];
    state_.velocity.z += dx[VEL_Z];

    // Inject attitude
    // 姿勢注入
    Vector3 dtheta(dx[ATT_X], dx[ATT_Y], dx[ATT_Z]);
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.yaw_estimation_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    // Inject gyro bias (active_mask already zeroed frozen components)
    // ジャイロバイアス注入 (active_maskで凍結成分は既にゼロ化済み)
    state_.gyro_bias.x += dx[BG_X];
    state_.gyro_bias.y += dx[BG_Y];
    state_.gyro_bias.z += dx[BG_Z];

    // Inject accel bias (active_mask already zeroed frozen components)
    // 加速度バイアス注入 (active_maskで凍結成分は既にゼロ化済み)
    state_.accel_bias.x += dx[BA_X];
    state_.accel_bias.y += dx[BA_Y];
    state_.accel_bias.z += dx[BA_Z];
}

// ============================================================================
// Bias / Attitude setters
// ============================================================================

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

void ESKF::initializeAttitude(const Vector3& accel, const Vector3& mag)
{
    float accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (accel_norm < 1.0f) {
        state_.roll = 0.0f;
        state_.pitch = 0.0f;
    } else {
        state_.roll = std::atan2(-accel.y, -accel.z);
        state_.pitch = std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
    }
    state_.yaw = 0.0f;
    state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, state_.yaw);

    // Compute mag reference in NED
    // NED座標系での地磁気参照を計算
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

    config_.mag_ref.x = r00 * mag.x + r01 * mag.y + r02 * mag.z;
    config_.mag_ref.y = r10 * mag.x + r11 * mag.y + r12 * mag.z;
    config_.mag_ref.z = r20 * mag.x + r21 * mag.y + r22 * mag.z;

    ESP_LOGI(TAG, "Attitude initialized: roll=%.1f° pitch=%.1f° yaw=0°",
             state_.roll * 180.0f / M_PI, state_.pitch * 180.0f / M_PI);
    ESP_LOGI(TAG, "Mag ref (NED): (%.1f, %.1f, %.1f) uT",
             config_.mag_ref.x, config_.mag_ref.y, config_.mag_ref.z);
}

void ESKF::setAttitudeReference(const Vector3& level_accel, const Vector3& gyro_bias)
{
    state_.gyro_bias = gyro_bias;
    float current_yaw = state_.yaw;

    float accel_norm = std::sqrt(level_accel.x * level_accel.x +
                                  level_accel.y * level_accel.y +
                                  level_accel.z * level_accel.z);
    if (accel_norm < 1.0f) {
        ESP_LOGW(TAG, "setAttitudeReference: accel norm too small (%.2f)", accel_norm);
        return;
    }

    state_.roll = 0.0f;
    state_.pitch = 0.0f;
    state_.yaw = current_yaw;
    state_.orientation = Quaternion::fromEuler(0.0f, 0.0f, current_yaw);

    state_.accel_bias = Vector3(
        level_accel.x,
        level_accel.y,
        level_accel.z - (-config_.gravity)
    );

    ESP_LOGI(TAG, "Accel bias set from level ref: (%.4f, %.4f, %.4f) m/s²",
             state_.accel_bias.x, state_.accel_bias.y, state_.accel_bias.z);
    ESP_LOGI(TAG, "Gyro bias set: (%.4f, %.4f, %.4f) rad/s",
             state_.gyro_bias.x, state_.gyro_bias.y, state_.gyro_bias.z);

    // Reset covariance for attitude and biases
    // 姿勢とバイアスの共分散をリセット
    P_(ATT_X, ATT_X) = init_P_diag_[ATT_X];
    P_(ATT_Y, ATT_Y) = init_P_diag_[ATT_Y];
    P_(ATT_Z, ATT_Z) = init_P_diag_[ATT_Z];
    P_(BA_X, BA_X) = init_P_diag_[BA_X];
    P_(BA_Y, BA_Y) = init_P_diag_[BA_Y];
    P_(BA_Z, BA_Z) = init_P_diag_[BA_Z];
    P_(BG_X, BG_X) = init_P_diag_[BG_X];
    P_(BG_Y, BG_Y) = init_P_diag_[BG_Y];
    P_(BG_Z, BG_Z) = init_P_diag_[BG_Z];

    freeze_accel_bias_ = false;
    recomputeActiveMask();
    enforceCovarianceConstraints();

    ESP_LOGI(TAG, "Attitude reference set: roll=0° pitch=0° yaw=%.1f°",
             current_yaw * 180.0f / M_PI);
}

// ============================================================================
// Predict
// ============================================================================

void ESKF::predict(const Vector3& accel, const Vector3& gyro, float dt, bool skip_position)
{
    if (!initialized_ || dt <= 0) return;

    // Bias correction
    // バイアス補正
    Vector3 gyro_corrected = gyro - state_.gyro_bias;
    Vector3 accel_corrected = accel - state_.accel_bias;

    if (!config_.yaw_estimation_enabled) {
        gyro_corrected.z = 0.0f;
    }

    // Rotation matrix elements
    // 回転行列要素
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    float R00 = 1 - 2*(q2*q2 + q3*q3);
    float R01 = 2*(q1*q2 - q0*q3);
    float R02 = 2*(q1*q3 + q0*q2);
    float R10 = 2*(q1*q2 + q0*q3);
    float R11 = 1 - 2*(q1*q1 + q3*q3);
    float R12 = 2*(q2*q3 - q0*q1);
    float R20 = 2*(q1*q3 - q0*q2);
    float R21 = 2*(q2*q3 + q0*q1);
    float R22 = 1 - 2*(q1*q1 + q2*q2);

    float ax = accel_corrected.x, ay = accel_corrected.y, az = accel_corrected.z;

    // Nominal state update
    // 名目状態の更新
    // Skip if: skip_position requested (grounded) OR position states frozen (active_mask)
    // 位置・速度の更新をスキップ: 接地中 または active_mask で凍結中
    // Check X/Y and Z independently — Flow controls X/Y, Baro/ToF controls Z
    // X/Y と Z を独立判定 — Flow が X/Y、Baro/ToF が Z を制御
    bool pos_xy_active = (active_mask_ >> POS_X) & 1;
    bool pos_z_active  = (active_mask_ >> POS_Z) & 1;

    if (!skip_position) {
        float accel_world_x = R00*ax + R01*ay + R02*az;
        float accel_world_y = R10*ax + R11*ay + R12*az;
        float accel_world_z = R20*ax + R21*ay + R22*az + config_.gravity;

        float half_dt_sq = 0.5f * dt * dt;

        if (pos_xy_active) {
            state_.position.x += state_.velocity.x * dt + accel_world_x * half_dt_sq;
            state_.position.y += state_.velocity.y * dt + accel_world_y * half_dt_sq;
            state_.velocity.x += accel_world_x * dt;
            state_.velocity.y += accel_world_y * dt;
        }
        if (pos_z_active) {
            state_.position.z += state_.velocity.z * dt + accel_world_z * half_dt_sq;
            state_.velocity.z += accel_world_z * dt;
        }
    }

    // Attitude update
    // 姿勢更新
    Vector3 dtheta = gyro_corrected * dt;
    Quaternion dq = Quaternion::fromRotationVector(dtheta);
    state_.orientation = state_.orientation * dq;
    state_.orientation.normalize();
    state_.orientation.toEuler(state_.roll, state_.pitch, state_.yaw);

    if (!config_.yaw_estimation_enabled) {
        state_.yaw = 0.0f;
        state_.orientation = Quaternion::fromEuler(state_.roll, state_.pitch, 0.0f);
    }

    // ========================================================================
    // Sparse covariance update: P' = F * P * F^T + Q (with active_mask gating)
    // 疎行列展開による共分散更新 (active_maskによるQゲーティング付き)
    // ========================================================================

    float adt_x = ax * dt, adt_y = ay * dt, adt_z = az * dt;

    // D_va = -R * skew(a) * dt
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

    // Process noise with active_mask gating
    // active_maskによるQゲーティング付きプロセスノイズ
    float gyro_var_base  = config_.gyro_noise * config_.gyro_noise * dt;
    float accel_var_base = config_.accel_noise * config_.accel_noise * dt;
    float bg_var_base    = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
    float ba_var_base    = config_.accel_bias_noise * config_.accel_bias_noise * dt;

    // Q gating: only add noise to active states
    // Qゲーティング: アクティブ状態にのみノイズを加算
    float q_att[3], q_bg[3], q_ba[3], q_vel[3];
    for (int i = 0; i < 3; i++) {
        q_att[i] = ((active_mask_ >> (ATT_X + i)) & 1) ? gyro_var_base : 0.0f;
        q_bg[i]  = ((active_mask_ >> (BG_X + i))  & 1) ? bg_var_base   : 0.0f;
        q_ba[i]  = ((active_mask_ >> (BA_X + i))  & 1) ? ba_var_base   : 0.0f;
        q_vel[i] = ((active_mask_ >> (VEL_X + i)) & 1) ? accel_var_base : 0.0f;
    }

    // ---- Sparse P' = F*P*F^T + Q ----

    float dt2 = dt * dt;

    // ---- pos-pos (0-2, 0-2) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            float val = P_(i, j) + dt * (P_(i+3, j) + P_(i, j+3)) + dt2 * P_(i+3, j+3);
            temp1_(i, j) = val;
            temp1_(j, i) = val;
        }
    }

    // ---- pos-vel (0-2, 3-5) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int jj = j + 3;
            float fp_i_j3 = P_(i, jj) + dt * P_(i+3, jj);
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

    // ---- pos-att (0-2, 6-8) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int jj = j + 6;
            float fp_i_j6 = P_(i, jj) + dt * P_(i+3, jj);
            float fp_i_9 = P_(i, 9+j) + dt * P_(i+3, 9+j);
            float val = fp_i_j6 + fp_i_9 * neg_dt;
            temp1_(i, jj) = val;
            temp1_(jj, i) = val;
        }
    }

    // ---- pos-bg (0-2, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val = P_(i, 9+j) + dt * P_(i+3, 9+j);
            temp1_(i, 9+j) = val;
            temp1_(9+j, i) = val;
        }
    }

    // ---- pos-ba (0-2, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val = P_(i, 12+j) + dt * P_(i+3, 12+j);
            temp1_(i, 12+j) = val;
            temp1_(12+j, i) = val;
        }
    }

    // ---- vel-vel (3-5, 3-5) ----
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

            float val = P_(ii, jj);

            // D_va * P_att contributions
            val += D_va_i0 * P_(6, jj) + D_va_i1 * P_(7, jj) + D_va_i2 * P_(8, jj);
            val += P_(ii, 6) * D_va_j0 + P_(ii, 7) * D_va_j1 + P_(ii, 8) * D_va_j2;

            // D_vb * P_ba contributions
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

            // Q term (gated)
            if (i == j) val += q_vel[i];

            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- vel-att (3-5, 6-8) ----
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

            float fp_ii_jj = P_(ii, jj) + D_va_i0*P_(6, jj) + D_va_i1*P_(7, jj) + D_va_i2*P_(8, jj)
                           + D_vb_i0*P_(12, jj) + D_vb_i1*P_(13, jj) + D_vb_i2*P_(14, jj);

            float fp_ii_9j = P_(ii, 9+j) + D_va_i0*P_(6, 9+j) + D_va_i1*P_(7, 9+j) + D_va_i2*P_(8, 9+j)
                           + D_vb_i0*P_(12, 9+j) + D_vb_i1*P_(13, 9+j) + D_vb_i2*P_(14, 9+j);

            float val = fp_ii_jj + fp_ii_9j * neg_dt;
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- vel-bg (3-5, 9-11) ----
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

    // ---- vel-ba (3-5, 12-14) ----
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

    // ---- att-att (6-8, 6-8) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 6 + i, jj = 6 + j;
            float fp_ii_jj = P_(ii, jj) + neg_dt * P_(9+i, jj);
            float fp_ii_9j = P_(ii, 9+j) + neg_dt * P_(9+i, 9+j);
            float val = fp_ii_jj + fp_ii_9j * neg_dt;
            if (i == j) val += q_att[i];
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- att-bg (6-8, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 6 + i, jj = 9 + j;
            float val = P_(ii, jj) + neg_dt * P_(9+i, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- att-ba (6-8, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 6 + i, jj = 12 + j;
            float val = P_(ii, jj) + neg_dt * P_(9+i, jj);
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- bg-bg (9-11, 9-11) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 9 + i, jj = 9 + j;
            float val = P_(ii, jj);
            if (i == j) val += q_bg[i];
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    // ---- bg-ba (9-11, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int ii = 9 + i, jj = 12 + j;
            temp1_(ii, jj) = P_(ii, jj);
            temp1_(jj, ii) = P_(ii, jj);
        }
    }

    // ---- ba-ba (12-14, 12-14) ----
    for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
            int ii = 12 + i, jj = 12 + j;
            float val = P_(ii, jj);
            if (i == j) val += q_ba[i];
            temp1_(ii, jj) = val;
            temp1_(jj, ii) = val;
        }
    }

    P_ = temp1_;
    enforceCovarianceConstraints();
}

// ============================================================================
// Measurement updates
// ============================================================================

// Helper: Joseph-form P update for scalar observation (H has single non-zero column col)
// ヘルパー: スカラー観測のJoseph形式P更新 (Hの非ゼロ列が1つ)
static void josephUpdateScalar(Matrix<15, 15>& P, Matrix<15, 15>& temp,
                               const float K[15], int col, float R_val)
{
    // Step 1: temp = (I - K*H) * P
    for (int i = 0; i < 15; i++) {
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            temp(i, j) = P(i, j) - Ki * P(col, j);
        }
    }

    // Step 2: P' = temp * (I - K*H)^T + K * R * K^T
    for (int i = 0; i < 15; i++) {
        float temp_i_col = temp(i, col);
        float Ki = K[i];
        for (int j = 0; j < 15; j++) {
            float val = temp(i, j) - K[j] * temp_i_col;
            val += R_val * Ki * K[j];
            P(i, j) = val;
        }
    }
}

void ESKF::updateBaro(float altitude)
{
    if (!initialized_) return;

    // H: H[0][2] = 1 (POS_Z only)
    float y = -altitude - state_.position.z;

    // Innovation gate (absolute threshold)
    // イノベーションゲート（絶対値閾値）
    if (config_.baro_innov_gate > 0.0f && std::fabs(y) > config_.baro_innov_gate) return;

    float R_val = config_.baro_noise * config_.baro_noise;

    float S = P_(2, 2) + R_val;
    if (S < 1e-10f) return;
    float S_inv = 1.0f / S;

    float K[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        K[i] = P_(i, 2) * S_inv;
    }

    float dx[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        dx[i] = K[i] * y;
    }

    // Yaw not observable from altitude
    // ヨーは高度観測から不可観測
    dx[ATT_Z] = 0.0f;

    applyMaskedErrorState(dx);
    josephUpdateScalar(P_, temp1_, K, 2, R_val);
    enforceCovarianceConstraints();
}

void ESKF::updateToF(float distance)
{
    if (!initialized_) return;

    // Tilt check
    // 傾きチェック
    float tilt = std::sqrt(state_.roll * state_.roll + state_.pitch * state_.pitch);
    if (tilt > config_.tof_tilt_threshold) return;

    // H: H[0][2] = 1 (POS_Z only)
    float cos_roll = std::cos(state_.roll);
    float cos_pitch = std::cos(state_.pitch);
    float height = distance * cos_roll * cos_pitch;

    float y = -height - state_.position.z;

    // Innovation gate (absolute threshold)
    // イノベーションゲート（絶対値閾値）
    if (config_.tof_innov_gate > 0.0f && std::fabs(y) > config_.tof_innov_gate) return;

    float R_val = config_.tof_noise * config_.tof_noise;

    float S = P_(2, 2) + R_val;
    if (S < 1e-10f) return;
    float S_inv = 1.0f / S;

    float K[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        K[i] = P_(i, 2) * S_inv;
    }

    float dx[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        dx[i] = K[i] * y;
    }

    dx[ATT_Z] = 0.0f;

    applyMaskedErrorState(dx);
    josephUpdateScalar(P_, temp1_, K, 2, R_val);
    enforceCovarianceConstraints();
}

void ESKF::updateMag(const Vector3& mag)
{
    if (!initialized_) return;

    // Magnitude check
    // ノルムチェック
    float mag_norm = std::sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
    if (mag_norm < config_.mag_norm_min || mag_norm > config_.mag_norm_max) return;

    // H: columns 6,7,8 (ATT_X, ATT_Y, ATT_Z) — skew(h) form
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

    // mag_expected = R^T * mag_ref
    float hx = r00*config_.mag_ref.x + r10*config_.mag_ref.y + r20*config_.mag_ref.z;
    float hy = r01*config_.mag_ref.x + r11*config_.mag_ref.y + r21*config_.mag_ref.z;
    float hz = r02*config_.mag_ref.x + r12*config_.mag_ref.y + r22*config_.mag_ref.z;

    float y0 = mag.x - hx;
    float y1 = mag.y - hy;
    float y2 = mag.z - hz;

    // H = -skew(h)
    float H06 = 0.0f,  H07 = -hz, H08 = hy;
    float H16 = hz,    H17 = 0.0f, H18 = -hx;
    float H26 = -hy,   H27 = hx,   H28 = 0.0f;

    float R_val = config_.mag_noise * config_.mag_noise;

    // HP = H * P (3x15)
    float HP[3][15];
    for (int j = 0; j < 15; j++) {
        float P6j = P_(6, j), P7j = P_(7, j), P8j = P_(8, j);
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
    S[1][1] = HP[1][6]*H16 + HP[1][7]*H17 + HP[1][8]*H18 + R_val;
    S[1][2] = HP[1][6]*H26 + HP[1][7]*H27 + HP[1][8]*H28;
    S[2][0] = S[0][2];
    S[2][1] = S[1][2];
    S[2][2] = HP[2][6]*H26 + HP[2][7]*H27 + HP[2][8]*H28 + R_val;

    // S^-1 (3x3)
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

    // Chi-squared outlier rejection (3 DOF): d² = y^T * S^-1 * y
    // χ²外れ値棄却
    if (config_.mag_chi2_gate > 0.0f) {
        float d2 = y0*(Si[0][0]*y0 + Si[0][1]*y1 + Si[0][2]*y2)
                 + y1*(Si[1][0]*y0 + Si[1][1]*y1 + Si[1][2]*y2)
                 + y2*(Si[2][0]*y0 + Si[2][1]*y1 + Si[2][2]*y2);
        if (d2 > config_.mag_chi2_gate) return;
    }

    // PHT = P * H^T (15x3)
    float PHT[15][3];
    for (int i = 0; i < 15; i++) {
        float Pi6 = P_(i, 6), Pi7 = P_(i, 7), Pi8 = P_(i, 8);
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
    float dx[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        dx[i] = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
    }

    // Mag observes yaw — don't zero ATT_Z (applyMaskedErrorState handles active_mask)
    // 地磁気はヨーを観測 — ATT_Zをゼロ化しない (applyMaskedErrorStateがactive_maskで処理)
    applyMaskedErrorState(dx);

    // Joseph-form P update (3x15 H, columns 6,7,8)
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0], Ki1 = K[i][1], Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0*HP[0][j] - Ki1*HP[1][j] - Ki2*HP[2][j];
        }
    }

    for (int i = 0; i < 15; i++) {
        float temp1_i6 = temp1_(i, 6), temp1_i7 = temp1_(i, 7), temp1_i8 = temp1_(i, 8);
        float Ki0 = K[i][0], Ki1 = K[i][1], Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            float Kj0 = K[j][0], Kj1 = K[j][1], Kj2 = K[j][2];
            float corr = temp1_i6*(Kj0*H06 + Kj1*H16 + Kj2*H26)
                       + temp1_i7*(Kj0*H07 + Kj1*H17 + Kj2*H27)
                       + temp1_i8*(Kj0*H08 + Kj1*H18 + Kj2*H28);
            float val = temp1_(i, j) - corr;
            val += R_val * (Ki0*Kj0 + Ki1*Kj1 + Ki2*Kj2);
            P_(i, j) = val;
        }
    }

    enforceCovarianceConstraints();
}

void ESKF::updateFlowRaw(int16_t flow_dx, int16_t flow_dy, float distance,
                              float dt, float gyro_x, float gyro_y)
{
    if (!initialized_ || distance < config_.flow_min_height || dt <= 0) return;

    // 1. Pixel -> angular rate [rad/s]
    float flow_dx_corrected = static_cast<float>(flow_dx) - config_.flow_offset[0];
    float flow_dy_corrected = static_cast<float>(flow_dy) - config_.flow_offset[1];
    float flow_x_cam = flow_dx_corrected * config_.flow_rad_per_pixel / dt;
    float flow_y_cam = flow_dy_corrected * config_.flow_rad_per_pixel / dt;

    // 2. Remove rotation component
    float gyro_x_corrected = gyro_x - state_.gyro_bias.x;
    float gyro_y_corrected = gyro_y - state_.gyro_bias.y;
    float gyro_scale = config_.flow_gyro_scale;
    float flow_rot_x_cam = gyro_scale * gyro_y_corrected;
    float flow_rot_y_cam = -gyro_scale * gyro_x_corrected;
    float flow_trans_x_cam = flow_x_cam - flow_rot_x_cam;
    float flow_trans_y_cam = flow_y_cam - flow_rot_y_cam;

    // 3. Camera -> body frame
    float flow_trans_x = config_.flow_cam_to_body[0] * flow_trans_x_cam
                       + config_.flow_cam_to_body[1] * flow_trans_y_cam;
    float flow_trans_y = config_.flow_cam_to_body[2] * flow_trans_x_cam
                       + config_.flow_cam_to_body[3] * flow_trans_y_cam;

    // 4. Translation velocity [m/s]
    float vx_body = flow_trans_x * distance;
    float vy_body = flow_trans_y * distance;

    // 5. Body -> NED
    float cos_yaw = std::cos(state_.yaw);
    float sin_yaw = std::sin(state_.yaw);
    float vx_ned = cos_yaw * vx_body - sin_yaw * vy_body;
    float vy_ned = sin_yaw * vx_body + cos_yaw * vy_body;

    // 6. H: H[0][3]=1 (VEL_X), H[1][4]=1 (VEL_Y)
    float y0 = vx_ned - state_.velocity.x;
    float y1 = vy_ned - state_.velocity.y;

    // Innovation clamp: limit velocity correction per update
    // イノベーションクランプ: 1回の更新での速度修正量を制限
    // Prevents flow spikes from causing sudden velocity jumps
    if (config_.flow_innov_clamp > 0.0f) {
        y0 = std::clamp(y0, -config_.flow_innov_clamp, config_.flow_innov_clamp);
        y1 = std::clamp(y1, -config_.flow_innov_clamp, config_.flow_innov_clamp);
    }

    float R_val = config_.flow_noise * config_.flow_noise;

    float P33 = P_(3, 3), P34 = P_(3, 4), P44 = P_(4, 4);
    float S00 = P33 + R_val;
    float S01 = P34;
    float S11 = P44 + R_val;

    float det = S00 * S11 - S01 * S01;
    if (std::abs(det) < 1e-10f) return;
    float inv_det = 1.0f / det;
    float Si00 = S11 * inv_det;
    float Si01 = -S01 * inv_det;
    float Si11 = S00 * inv_det;

    // Chi-squared outlier rejection (2 DOF): d² = y^T * S^-1 * y
    // χ²外れ値棄却
    if (config_.flow_chi2_gate > 0.0f) {
        float d2 = y0*(Si00*y0 + Si01*y1) + y1*(Si01*y0 + Si11*y1);
        if (d2 > config_.flow_chi2_gate) return;
    }

    float K[15][2];
    for (int i = 0; i < 15; i++) {
        float Pi3 = P_(i, 3), Pi4 = P_(i, 4);
        K[i][0] = Pi3 * Si00 + Pi4 * Si01;
        K[i][1] = Pi3 * Si01 + Pi4 * Si11;
    }

    float dx[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        dx[i] = K[i][0] * y0 + K[i][1] * y1;
    }

    dx[ATT_Z] = 0.0f;  // Yaw not observable from flow / ヨーはフローから不可観測

    applyMaskedErrorState(dx);

    // Joseph-form P update (H: columns 3,4)
    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0], Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j) - Ki0 * P_(3, j) - Ki1 * P_(4, j);
        }
    }

    for (int i = 0; i < 15; i++) {
        float temp1_i3 = temp1_(i, 3), temp1_i4 = temp1_(i, 4);
        float Ki0 = K[i][0], Ki1 = K[i][1];
        for (int j = 0; j < 15; j++) {
            float val = temp1_(i, j) - K[j][0] * temp1_i3 - K[j][1] * temp1_i4;
            val += R_val * (Ki0 * K[j][0] + Ki1 * K[j][1]);
            P_(i, j) = val;
        }
    }

    enforceCovarianceConstraints();
}

void ESKF::updateAccelAttitude(const Vector3& accel)
{
    if (!initialized_) return;

    // Bias-corrected acceleration
    // バイアス補正済み加速度
    float ax = accel.x - state_.accel_bias.x;
    float ay = accel.y - state_.accel_bias.y;
    float az = accel.z - state_.accel_bias.z;

    float accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
    float gravity_diff = std::abs(accel_norm - config_.gravity);

    // Rotation matrix third row
    // 回転行列の第3行
    const Quaternion& q = state_.orientation;
    float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;

    float R20 = 2*(q1*q3 - q0*q2);
    float R21 = 2*(q2*q3 + q0*q1);
    float R22 = 1 - 2*(q1*q1 + q2*q2);

    // Expected accel: g_expected = R^T * [0, 0, -g]
    float g = config_.gravity;
    float g_exp_x = -g * R20;
    float g_exp_y = -g * R21;
    float g_exp_z = -g * R22;

    float y0 = ax - g_exp_x;
    float y1 = ay - g_exp_y;
    float y2 = az - g_exp_z;

    // H matrix: attitude part (cols 6,7,8) and bias part (cols 12,13,14)
    float H06 = 0.0f,      H07 = g*R22,    H08 = -g*R21;
    float H16 = -g*R22,    H17 = 0.0f,     H18 = g*R20;
    float H26 = g*R21,     H27 = -g*R20,   H28 = 0.0f;

    // Adaptive R
    float gravity_diff_sq = gravity_diff * gravity_diff;
    float R_scale = 1.0f + config_.k_adaptive * gravity_diff_sq;
    float base_noise_sq = config_.accel_att_noise * config_.accel_att_noise;
    float R_val = base_noise_sq * R_scale;

    // PHT = P * H^T (15x3) — H has non-zero cols 6,7,8 and 12,13,14
    float PHT[15][3];
    for (int i = 0; i < 15; i++) {
        PHT[i][0] = P_(i,6)*H06 + P_(i,7)*H07 + P_(i,8)*H08 + P_(i,12);
        PHT[i][1] = P_(i,6)*H16 + P_(i,7)*H17 + P_(i,8)*H18 + P_(i,13);
        PHT[i][2] = P_(i,6)*H26 + P_(i,7)*H27 + P_(i,8)*H28 + P_(i,14);
    }

    // S = H * PHT + R (3x3)
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

    // S^-1 (3x3)
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

    // Chi-squared outlier rejection (3 DOF): d² = y^T * S^-1 * y
    // χ²外れ値棄却
    if (config_.accel_att_chi2_gate > 0.0f) {
        float d2 = y0*(Si[0][0]*y0 + Si[0][1]*y1 + Si[0][2]*y2)
                 + y1*(Si[1][0]*y0 + Si[1][1]*y1 + Si[1][2]*y2)
                 + y2*(Si[2][0]*y0 + Si[2][1]*y1 + Si[2][2]*y2);
        if (d2 > config_.accel_att_chi2_gate) return;
    }

    // K = PHT * S^-1 (15x3)
    float K[15][3];
    for (int i = 0; i < 15; i++) {
        K[i][0] = PHT[i][0]*Si[0][0] + PHT[i][1]*Si[1][0] + PHT[i][2]*Si[2][0];
        K[i][1] = PHT[i][0]*Si[0][1] + PHT[i][1]*Si[1][1] + PHT[i][2]*Si[2][1];
        K[i][2] = PHT[i][0]*Si[0][2] + PHT[i][1]*Si[1][2] + PHT[i][2]*Si[2][2];
    }

    // dx = K * y
    float dx[N_STATES];
    for (int i = 0; i < N_STATES; i++) {
        dx[i] = K[i][0]*y0 + K[i][1]*y1 + K[i][2]*y2;
    }

    // Yaw not observable from accel — zero ATT_Z before masking
    // ヨーは加速度から不可観測 — マスキング前にATT_Zをゼロ化
    dx[ATT_Z] = 0.0f;

    applyMaskedErrorState(dx);

    // Joseph-form P update (H: cols 6,7,8 and 12,13,14)
    for (int i = 0; i < 15; i++) {
        float IKH_i6 = -K[i][0]*H06 - K[i][1]*H16 - K[i][2]*H26;
        float IKH_i7 = -K[i][0]*H07 - K[i][1]*H17 - K[i][2]*H27;
        float IKH_i8 = -K[i][0]*H08 - K[i][1]*H18 - K[i][2]*H28;
        float IKH_i12 = -K[i][0];
        float IKH_i13 = -K[i][1];
        float IKH_i14 = -K[i][2];

        for (int j = 0; j < 15; j++) {
            temp1_(i, j) = P_(i, j)
                         + IKH_i6*P_(6, j) + IKH_i7*P_(7, j) + IKH_i8*P_(8, j)
                         + IKH_i12*P_(12, j) + IKH_i13*P_(13, j) + IKH_i14*P_(14, j);
        }
    }

    for (int i = 0; i < 15; i++) {
        float Ki0 = K[i][0], Ki1 = K[i][1], Ki2 = K[i][2];
        for (int j = 0; j < 15; j++) {
            float IKH_j6 = -K[j][0]*H06 - K[j][1]*H16 - K[j][2]*H26;
            float IKH_j7 = -K[j][0]*H07 - K[j][1]*H17 - K[j][2]*H27;
            float IKH_j8 = -K[j][0]*H08 - K[j][1]*H18 - K[j][2]*H28;
            float IKH_j12 = -K[j][0];
            float IKH_j13 = -K[j][1];
            float IKH_j14 = -K[j][2];

            float val = temp1_(i, j)
                      + temp1_(i, 6)*IKH_j6 + temp1_(i, 7)*IKH_j7 + temp1_(i, 8)*IKH_j8
                      + temp1_(i, 12)*IKH_j12 + temp1_(i, 13)*IKH_j13 + temp1_(i, 14)*IKH_j14;

            val += R_val * (Ki0*K[j][0] + Ki1*K[j][1] + Ki2*K[j][2]);
            P_(i, j) = val;
        }
    }

    enforceCovarianceConstraints();
}

}  // namespace stampfly
