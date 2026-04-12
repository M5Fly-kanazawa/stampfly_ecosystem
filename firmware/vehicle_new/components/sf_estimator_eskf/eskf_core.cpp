/**
 * @file eskf_core.cpp
 * @brief ESKF 15-state implementation — from mathematical foundations
 *        ESKF 15状態実装 — 数学的基礎から新規実装
 *
 * State: [pos(3), vel(3), att_err(3), gyro_bias(3), accel_bias(3)]
 *
 * Lessons from old firmware (not copied, applied as design knowledge):
 * - P-matrix isolation via active_mask prevents state corruption
 * - chi2 gates essential for attitude sensors
 * - Innovation absolute gates for position sensors (P-collapse protection)
 * - Joseph form for numerical stability
 *
 * 旧ファームからの教訓（コピーではなく設計知識として適用）:
 * - active_maskによるP行列隔離で状態破壊を防止
 * - 姿勢センサにはχ²ゲートが必須
 * - 位置センサには絶対値イノベーションゲート（P崩壊対策）
 * - 数値安定性のためJoseph形式
 *
 * @design detailed_design.md §5 — IEstimator                         [--]
 */

#include "eskf_core.hpp"
#include "esp_log.h"
#include <cmath>
#include <algorithm>

static const char* TAG = "ESKF_CORE";

namespace sf {

// =============================================================================
// Initialization / 初期化
// =============================================================================

void EskfCore::init(const EskfConfig& cfg)
{
    cfg_ = cfg;
    reset();
    recomputeActiveMask();
    ESP_LOGI(TAG, "ESKF core initialized (15-state)");
}

void EskfCore::reset()
{
    pos_ = {0, 0, 0};
    vel_ = {0, 0, 0};
    q_ = {1, 0, 0, 0};  // Identity / 単位クォータニオン
    bg_ = {0, 0, 0};
    ba_ = {0, 0, 0};

    // Initialize P with config standard deviations
    // 設定の標準偏差でPを初期化
    P_.zero();
    for (int i = 0; i < 3; i++) {
        P_.set_diag(POS_X + i, cfg_.init_pos_std * cfg_.init_pos_std);
        P_.set_diag(VEL_X + i, cfg_.init_vel_std * cfg_.init_vel_std);
        P_.set_diag(ATT_X + i, cfg_.init_att_std * cfg_.init_att_std);
        P_.set_diag(BG_X  + i, cfg_.init_bg_std * cfg_.init_bg_std);
        P_.set_diag(BA_X  + i, cfg_.init_ba_std * cfg_.init_ba_std);
    }

    freeze_accel_bias_ = false;
}

void EskfCore::resetPositionVelocity()
{
    pos_ = {0, 0, 0};
    vel_ = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        P_.set_diag(POS_X + i, cfg_.init_pos_std * cfg_.init_pos_std);
        P_.set_diag(VEL_X + i, cfg_.init_vel_std * cfg_.init_vel_std);
        // Zero cross-covariance with position/velocity
        // 位置/速度のクロス共分散をゼロ化
        for (int j = 0; j < N; j++) {
            if (j != POS_X + i) P_(POS_X + i, j) = P_(j, POS_X + i) = 0;
            if (j != VEL_X + i) P_(VEL_X + i, j) = P_(j, VEL_X + i) = 0;
        }
    }
}

// =============================================================================
// Prediction / 予測ステップ
//
// Nominal state integration + error-state covariance propagation
// 名目状態の積分 + 誤差状態の共分散伝搬
// =============================================================================

void EskfCore::predict(const Vec3& accel_raw, const Vec3& gyro_raw, float dt)
{
    // Bias-corrected IMU / バイアス補正済みIMU
    Vec3 accel = accel_raw - ba_;
    Vec3 gyro  = gyro_raw - bg_;

    // Rotation matrix from quaternion / クォータニオンから回転行列
    float R[3][3];
    q_.to_dcm(R);

    // Gravity in NED / NED座標系での重力
    Vec3 gravity = {0, 0, cfg_.gravity};

    // Rotate accel to NED frame / 加速度をNED座標系に回転
    Vec3 accel_ned;
    accel_ned.x = R[0][0]*accel.x + R[0][1]*accel.y + R[0][2]*accel.z;
    accel_ned.y = R[1][0]*accel.x + R[1][1]*accel.y + R[1][2]*accel.z;
    accel_ned.z = R[2][0]*accel.x + R[2][1]*accel.y + R[2][2]*accel.z;

    // Nominal state integration (Euler) / 名目状態の積分（オイラー法）
    pos_ += vel_ * dt;
    vel_ += (accel_ned + gravity) * dt;
    Quat dq = Quat::from_rotvec(gyro * dt);
    q_ = q_ * dq;
    q_.normalize();

    // =========================================================================
    // Covariance propagation: P' = F*P*F^T + Q
    // 共分散伝搬: P' = F*P*F^T + Q
    //
    // Full 15x15 matrix implementation for correctness.
    // 正確性のためフル15x15行列実装。
    //
    // F = I + dF where dF has non-zero blocks:
    //   dF[pos][vel]  = I*dt
    //   dF[vel][att]  = D_va = -R*[a_c×]*dt
    //   dF[vel][ba]   = D_vb = -R*dt
    //   dF[att][bg]   = -I*dt
    // =========================================================================

    // Build F matrix explicitly / F行列を明示的に構築
    float F[N][N] = {};
    for (int i = 0; i < N; i++) F[i][i] = 1.0f;  // Identity

    // dF[pos][vel] = I*dt
    for (int i = 0; i < 3; i++) F[POS_X+i][VEL_X+i] = dt;

    // dF[vel][att] = D_va = -R*[a_c×]*dt
    for (int i = 0; i < 3; i++) {
        F[VEL_X+i][ATT_X+0] = (R[i][1]*accel.z - R[i][2]*accel.y) * dt;
        F[VEL_X+i][ATT_X+1] = (R[i][2]*accel.x - R[i][0]*accel.z) * dt;
        F[VEL_X+i][ATT_X+2] = (R[i][0]*accel.y - R[i][1]*accel.x) * dt;
    }

    // dF[vel][ba] = -R*dt
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F[VEL_X+i][BA_X+j] = -R[i][j] * dt;

    // dF[att][bg] = -I*dt
    for (int i = 0; i < 3; i++) F[ATT_X+i][BG_X+i] = -dt;

    // Compute FP = F * P (15x15 matrix multiply)
    // FP = F * P を計算
    float FP[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float sum = 0;
            for (int k = 0; k < N; k++)
                sum += F[i][k] * P_(k, j);
            FP[i][j] = sum;
        }

    // Compute P' = FP * F^T (15x15 matrix multiply)
    // P' = FP * F^T を計算
    for (int i = 0; i < N; i++)
        for (int j = i; j < N; j++) {
            float sum = 0;
            for (int k = 0; k < N; k++)
                sum += FP[i][k] * F[j][k];  // F^T[k][j] = F[j][k]
            P_(i, j) = sum;
            P_(j, i) = sum;  // Symmetric
        }

    // Step 5: Add process noise Q (with active_mask gating)
    // ステップ5: プロセスノイズQを加算（active_maskゲーティング付き）
    float q_att  = cfg_.gyro_noise * cfg_.gyro_noise * dt;
    float q_vel  = cfg_.accel_noise * cfg_.accel_noise * dt;
    float q_bg   = cfg_.gyro_bias_noise * cfg_.gyro_bias_noise * dt;
    float q_ba   = cfg_.accel_bias_noise * cfg_.accel_bias_noise * dt;

    for (int i = 0; i < 3; i++) {
        if (active_mask_ & (1 << (ATT_X + i))) P_(ATT_X+i, ATT_X+i) += q_att;
        if (active_mask_ & (1 << (VEL_X + i))) P_(VEL_X+i, VEL_X+i) += q_vel;
        if (active_mask_ & (1 << (BG_X  + i))) P_(BG_X+i,  BG_X+i)  += q_bg;
        if (active_mask_ & (1 << (BA_X  + i))) P_(BA_X+i,  BA_X+i)  += q_ba;
    }

    enforceCovarianceConstraints();
}

// =============================================================================
// Scalar Kalman Update (1D observation) / スカラーカルマン更新
// Joseph form: P' = (I-KH)P(I-KH)^T + KRK^T
// =============================================================================

void EskfCore::scalarUpdate(const float H[N], float innovation, float R_val)
{
    // S = H*P*H^T + R
    float S = R_val;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            S += H[i] * P_(i, j) * H[j];

    if (S < 1e-10f) return;  // Singular / 特異

    // K = P*H^T / S
    float K[N];
    for (int i = 0; i < N; i++) {
        K[i] = 0;
        for (int j = 0; j < N; j++)
            K[i] += P_(i, j) * H[j];
        K[i] /= S;
    }

    // Error state: dx = K * innovation
    // 誤差状態: dx = K * innovation
    float dx[N];
    for (int i = 0; i < N; i++)
        dx[i] = K[i] * innovation;

    // Apply masked error state / マスク付き誤差状態を適用
    applyMaskedErrorState(dx);

    // Joseph form covariance update / Joseph形式で共分散更新
    // P' = (I-KH)*P*(I-KH)^T + K*R*K^T
    float IKH[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];

    SymMat15 P_new;
    for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
            float sum = K[i] * R_val * K[j];
            for (int k = 0; k < N; k++)
                for (int l = 0; l < N; l++)
                    sum += IKH[i][k] * P_(k, l) * IKH[j][l];
            P_new(i, j) = sum;
            P_new(j, i) = sum;
        }
    }
    P_ = P_new;

    enforceCovarianceConstraints();
}

// =============================================================================
// 3D Kalman Update / 3次元カルマン更新
// =============================================================================

void EskfCore::vectorUpdate3(const float H[3][N], const float innov[3], float R_val)
{
    // S = H*P*H^T + R*I (3x3)
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = (i == j) ? R_val : 0.0f;
            for (int k = 0; k < N; k++)
                for (int l = 0; l < N; l++)
                    S[i][j] += H[i][k] * P_(k, l) * H[j][l];
        }
    }

    // Invert 3x3 S analytically / 3x3 Sを解析的に逆行列
    float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-10f) return;

    float inv_det = 1.0f / det;
    float Si[3][3];
    Si[0][0] = (S[1][1]*S[2][2]-S[1][2]*S[2][1]) * inv_det;
    Si[0][1] = (S[0][2]*S[2][1]-S[0][1]*S[2][2]) * inv_det;
    Si[0][2] = (S[0][1]*S[1][2]-S[0][2]*S[1][1]) * inv_det;
    Si[1][0] = (S[1][2]*S[2][0]-S[1][0]*S[2][2]) * inv_det;
    Si[1][1] = (S[0][0]*S[2][2]-S[0][2]*S[2][0]) * inv_det;
    Si[1][2] = (S[0][2]*S[1][0]-S[0][0]*S[1][2]) * inv_det;
    Si[2][0] = (S[1][0]*S[2][1]-S[1][1]*S[2][0]) * inv_det;
    Si[2][1] = (S[0][1]*S[2][0]-S[0][0]*S[2][1]) * inv_det;
    Si[2][2] = (S[0][0]*S[1][1]-S[0][1]*S[1][0]) * inv_det;

    // Chi-squared outlier rejection: d² = y^T S⁻¹ y
    // χ²外れ値棄却: d² = y^T S⁻¹ y
    float d2 = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d2 += innov[i] * Si[i][j] * innov[j];

    // Debug: track chi2 gate statistics
    // デバッグ: χ²ゲートの統計を追跡
    static int gate_total = 0, gate_reject = 0;
    gate_total++;
    if (d2 > cfg_.accel_chi2_gate) {
        gate_reject++;
        if (gate_total % 400 == 0) {
            fprintf(stderr, "  chi2: d2=%.1f gate=%.1f reject=%d/%d (%.0f%%)\n",
                    d2, cfg_.accel_chi2_gate, gate_reject, gate_total,
                    100.0f * gate_reject / gate_total);
        }
        return;
    }
    if (gate_total % 400 == 0) {
        fprintf(stderr, "  chi2: d2=%.1f gate=%.1f reject=%d/%d (%.0f%%) R=%.4f\n",
                d2, cfg_.accel_chi2_gate, gate_reject, gate_total,
                100.0f * gate_reject / gate_total, R_val);
    }

    // K = P*H^T*S^-1 (Nx3)
    float K[N][3];
    for (int i = 0; i < N; i++) {
        float PHt[3] = {0, 0, 0};
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < N; k++)
                PHt[j] += P_(i, k) * H[j][k];
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 3; k++)
                K[i][j] += PHt[k] * Si[k][j];
        }
    }

    // dx = K * innov
    float dx[N];
    for (int i = 0; i < N; i++) {
        dx[i] = 0;
        for (int j = 0; j < 3; j++)
            dx[i] += K[i][j] * innov[j];
    }

    applyMaskedErrorState(dx);

    // Simplified covariance update: P' = (I - K*H)*P
    // 簡易共分散更新: P' = (I - K*H)*P
    SymMat15 P_new;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float kh = 0;
            for (int k = 0; k < 3; k++)
                kh += K[i][k] * H[k][j];
            float val = P_(i, j) - kh * P_(i, j);
            // Actually: sum over l of (I-KH)[i][l] * P[l][j]
            val = 0;
            for (int l = 0; l < N; l++) {
                float ikhl = (i == l ? 1.0f : 0.0f);
                for (int k = 0; k < 3; k++)
                    ikhl -= K[i][k] * H[k][l];
                val += ikhl * P_(l, j);
            }
            P_new(i, j) = val;
        }
    }
    P_ = P_new;
    P_.symmetrize();
    enforceCovarianceConstraints();
}

// =============================================================================
// Observation Updates / 観測更新
// =============================================================================

void EskfCore::updateToF(float distance)
{
    if (!cfg_.use_tof) return;

    // Tilt check / 傾きチェック
    Vec3 euler = q_.to_euler();
    float tilt = sqrtf(euler.x*euler.x + euler.y*euler.y);
    if (tilt > cfg_.tof_tilt_threshold) return;

    // Correct for tilt / 傾き補正
    float height = distance * cosf(euler.x) * cosf(euler.y);

    // H = [0,0,1, 0...0] (observes POS_Z)
    // H = [0,0,1, 0...0]（POS_Zを観測）
    float H[N] = {};
    H[POS_Z] = 1.0f;

    // Innovation: y = -height - pos_z (NED: z down)
    // イノベーション: y = -height - pos_z（NED: z下向き）
    float innovation = -height - pos_.z;

    // Absolute innovation gate / 絶対値イノベーションゲート
    if (fabsf(innovation) > cfg_.tof_innov_gate) return;

    scalarUpdate(H, innovation, cfg_.tof_noise * cfg_.tof_noise);
}

void EskfCore::updateToFVelocity(float distance, float dt)
{
    if (!cfg_.use_tof) return;
    if (dt < 1e-6f) return;

    // Tilt check
    Vec3 euler = q_.to_euler();
    float tilt = sqrtf(euler.x*euler.x + euler.y*euler.y);
    if (tilt > cfg_.tof_tilt_threshold) return;

    float height = distance * cosf(euler.x) * cosf(euler.y);

    // Velocity from ToF difference: v_z ≈ d(pos_z)/dt
    // ToF微分からの速度: v_z ≈ d(pos_z)/dt
    // Since pos_z = -height in NED: v_z = -d(height)/dt
    // Previous height stored internally
    static float prev_height = height;
    float vel_z_observed = -(height - prev_height) / dt;
    prev_height = height;

    // H = [0,0,0, 0,0,1, 0...0] (observes VEL_Z)
    float H[N] = {};
    H[VEL_Z] = 1.0f;

    float innovation = vel_z_observed - vel_.z;

    // Larger noise for velocity (derivative is noisy)
    // 速度は微分なのでノイズが大きい
    float vel_noise = cfg_.tof_noise * 5.0f;  // 5x position noise
    scalarUpdate(H, innovation, vel_noise * vel_noise);
}

void EskfCore::updateBaro(float altitude)
{
    if (!cfg_.use_baro) return;

    float H[N] = {};
    H[POS_Z] = 1.0f;

    float innovation = -altitude - pos_.z;

    if (fabsf(innovation) > cfg_.baro_innov_gate) return;

    scalarUpdate(H, innovation, cfg_.baro_noise * cfg_.baro_noise);
}

void EskfCore::updateMag(const Vec3& mag)
{
    if (!cfg_.use_mag) return;

    // Expected measurement: h = R^T * mag_ref (NED→body)
    // 期待値: h = R^T * mag_ref（NED→body）
    Vec3 h_expected = q_.inv_rotate(cfg_.mag_ref);

    // Innovation / イノベーション
    float innov[3] = {
        mag.x - h_expected.x,
        mag.y - h_expected.y,
        mag.z - h_expected.z
    };

    // H = -[h×] (skew-symmetric of expected, attitude columns only)
    // H = -[h×]（期待値のスキュー対称、姿勢列のみ）
    float H[3][N] = {};
    H[0][ATT_Y] = -h_expected.z;  H[0][ATT_Z] =  h_expected.y;
    H[1][ATT_X] =  h_expected.z;  H[1][ATT_Z] = -h_expected.x;
    H[2][ATT_X] = -h_expected.y;  H[2][ATT_Y] =  h_expected.x;

    // Chi-squared gate / χ²ゲート
    // TODO: compute chi2 and gate

    vectorUpdate3(H, innov, cfg_.mag_noise * cfg_.mag_noise);
}

void EskfCore::updateAccelAttitude(const Vec3& accel_raw)
{
    // Bias-corrected accel / バイアス補正済み加速度
    Vec3 accel = accel_raw - ba_;

    // Norm-based gate: skip when accel norm deviates from gravity
    // ノルムゲート: 加速度ノルムが重力から大きく逸脱したらスキップ
    // During thrust transients, |a| ≠ g, and accel cannot be used
    // for attitude estimation (specific force contaminates gravity)
    // 推力過渡期は|a| ≠ gとなり、比力が重力を汚染するため
    // 姿勢推定に加速度を使用できない
    float gravity_diff = accel.norm() - cfg_.gravity;
    float norm_ratio = fabsf(gravity_diff) / cfg_.gravity;
    if (norm_ratio > 0.1f) return;  // Skip if >10% deviation

    // Adaptive R scaling: R = R_base * (1 + k * |a - g|²)
    // 適応Rスケーリング: R = R_base * (1 + k * |a - g|²)
    float R_val = cfg_.accel_att_noise * cfg_.accel_att_noise
                * (1.0f + cfg_.k_adaptive * gravity_diff * gravity_diff);

    // Expected gravity in body: g_body = R^T * [0, 0, -g]
    // ボディ座標の期待重力: g_body = R^T * [0, 0, -g]
    Vec3 g_ned = {0, 0, -cfg_.gravity};
    Vec3 g_expected = q_.inv_rotate(g_ned);

    float innov[3] = {
        accel.x - g_expected.x,
        accel.y - g_expected.y,
        accel.z - g_expected.z
    };

    // H matrix: attitude part + accel bias part
    // H行列: 姿勢部分 + 加速度バイアス部分
    float R_dcm[3][3];
    q_.to_dcm(R_dcm);

    float H[3][N] = {};
    // Attitude columns: -[g_expected×]
    H[0][ATT_Y] = -g_expected.z;  H[0][ATT_Z] =  g_expected.y;
    H[1][ATT_X] =  g_expected.z;  H[1][ATT_Z] = -g_expected.x;
    H[2][ATT_X] = -g_expected.y;  H[2][ATT_Y] =  g_expected.x;

    // Accel bias columns: I (direct observation)
    // 加速度バイアス列: I（直接観測）
    H[0][BA_X] = 1.0f;
    H[1][BA_Y] = 1.0f;
    H[2][BA_Z] = 1.0f;

    vectorUpdate3(H, innov, R_val);
}

void EskfCore::updateFlowRaw(int16_t dx, int16_t dy, float height,
                              float dt, float gyro_x, float gyro_y)
{
    if (!cfg_.use_flow) return;
    if (height < cfg_.flow_min_height) return;
    if (dt < 1e-6f) return;

    // Pixel → angular rate → translational velocity
    // ピクセル → 角速度 → 並進速度
    float flow_rate_x = static_cast<float>(dx) * cfg_.flow_rad_per_pixel / dt;
    float flow_rate_y = static_cast<float>(dy) * cfg_.flow_rad_per_pixel / dt;

    // Remove rotation component (gyro compensation)
    // 回転成分を除去（ジャイロ補償）
    float trans_x = flow_rate_x - cfg_.flow_gyro_scale * gyro_y;
    float trans_y = flow_rate_y + cfg_.flow_gyro_scale * gyro_x;

    // Translational velocity in body frame
    // ボディフレームでの並進速度
    float vx_body = trans_x * height;
    float vy_body = trans_y * height;

    // Body → NED (yaw rotation only)
    // Body → NED（ヨー回転のみ）
    Vec3 euler = q_.to_euler();
    float cy = cosf(euler.z), sy = sinf(euler.z);
    float vx_ned = cy * vx_body - sy * vy_body;
    float vy_ned = sy * vx_body + cy * vy_body;

    // H = [[0,0,0, 1,0,0, ...], [0,0,0, 0,1,0, ...]] (observes VEL_X, VEL_Y)
    float H[N] = {};

    // Update X velocity / X速度を更新
    H[VEL_X] = 1.0f;
    float innov_x = vx_ned - vel_.x;
    if (cfg_.flow_innov_clamp > 0) {
        innov_x = fmaxf(-cfg_.flow_innov_clamp, fminf(cfg_.flow_innov_clamp, innov_x));
    }
    scalarUpdate(H, innov_x, cfg_.flow_noise * cfg_.flow_noise);

    // Update Y velocity / Y速度を更新
    memset(H, 0, sizeof(H));
    H[VEL_Y] = 1.0f;
    float innov_y = vy_ned - vel_.y;
    if (cfg_.flow_innov_clamp > 0) {
        innov_y = fmaxf(-cfg_.flow_innov_clamp, fminf(cfg_.flow_innov_clamp, innov_y));
    }
    scalarUpdate(H, innov_y, cfg_.flow_noise * cfg_.flow_noise);
}

// =============================================================================
// Active Mask / 有効マスク
// =============================================================================

void EskfCore::setSensorEnabled(int group, bool enabled)
{
    // group: 0=TOF, 1=BARO, 2=MAG, 3=FLOW
    switch (group) {
        case 0: cfg_.use_tof  = enabled; break;
        case 1: cfg_.use_baro = enabled; break;
        case 2: cfg_.use_mag  = enabled; break;
        case 3: cfg_.use_flow = enabled; break;
    }
    recomputeActiveMask();
}

void EskfCore::recomputeActiveMask()
{
    active_mask_ = 0x7FFF;  // All 15 bits on

    // If neither TOF nor BARO: freeze POS_Z, VEL_Z, BA_Z
    // TOFもBAROもなければ: POS_Z, VEL_Z, BA_Zをフリーズ
    if (!cfg_.use_tof && !cfg_.use_baro) {
        active_mask_ &= ~((1 << POS_Z) | (1 << VEL_Z) | (1 << BA_Z));
    }

    // If no FLOW: freeze POS_X, POS_Y, VEL_X, VEL_Y
    // FLOWなければ: POS_X, POS_Y, VEL_X, VEL_Yをフリーズ
    if (!cfg_.use_flow) {
        active_mask_ &= ~((1 << POS_X) | (1 << POS_Y) |
                          (1 << VEL_X) | (1 << VEL_Y));
    }

    // If no MAG: freeze ATT_Z, BG_Z
    // MAGなければ: ATT_Z, BG_Zをフリーズ
    if (!cfg_.use_mag) {
        active_mask_ &= ~((1 << ATT_Z) | (1 << BG_Z));
    }

    // Freeze accel bias if requested
    // 要求されていれば加速度バイアスをフリーズ
    if (freeze_accel_bias_) {
        active_mask_ &= ~((1 << BA_X) | (1 << BA_Y) | (1 << BA_Z));
    }
}

void EskfCore::setFreezeAccelBias(bool freeze)
{
    freeze_accel_bias_ = freeze;
    recomputeActiveMask();
}

void EskfCore::holdPositionVelocity()
{
    pos_ = {0, 0, 0};
    vel_ = {0, 0, 0};
}

void EskfCore::setAttitudeFromGravity(const Vec3& accel_avg)
{
    // Compute roll/pitch from gravity vector direction
    // 重力ベクトルの方向からroll/pitchを計算
    // accel_avg at rest ≈ [0, 0, -g] in NED body (level)
    // Non-level: gravity projects onto x,y axes
    //
    // roll  = atan2(ay, -az)  (rotation around x to align gravity with -z)
    // pitch = atan2(-ax, sqrt(ay² + az²))
    float roll  = atan2f(accel_avg.y, -accel_avg.z);
    float pitch = atan2f(-accel_avg.x,
                         sqrtf(accel_avg.y * accel_avg.y
                             + accel_avg.z * accel_avg.z));
    float yaw = 0;  // No heading reference from accel alone
                     // 加速度だけではヘディング基準なし

    // Set quaternion from Euler angles (ZYX convention)
    // オイラー角からクォータニオンを設定（ZYX順）
    // q = q_yaw * q_pitch * q_roll
    Quat qr = Quat::from_rotvec(Vec3(roll, 0, 0));
    Quat qp = Quat::from_rotvec(Vec3(0, pitch, 0));
    Quat qy = Quat::from_rotvec(Vec3(0, 0, yaw));
    q_ = qy * qp * qr;
    q_.normalize();
}

void EskfCore::shrinkBiasCovariance(float factor)
{
    // Shrink bias covariance to trust calibration
    // キャリブレーションを信頼するためバイアス共分散を縮小
    for (int i = 0; i < 3; i++) {
        P_.set_diag(BG_X + i, P_.data[BG_X + i][BG_X + i] * factor);
        P_.set_diag(BA_X + i, P_.data[BA_X + i][BA_X + i] * factor);
    }
}

// =============================================================================
// Covariance Constraints / 共分散制約
//
// Frozen states: reset diagonal to initial, zero cross-covariance
// フリーズ状態: 対角を初期値にリセット、クロス共分散をゼロ化
// =============================================================================

void EskfCore::enforceCovarianceConstraints()
{
    // Initial diagonal values for frozen states
    // フリーズ状態の初期対角値
    float init_diag[N];
    for (int i = 0; i < 3; i++) {
        init_diag[POS_X+i] = cfg_.init_pos_std * cfg_.init_pos_std;
        init_diag[VEL_X+i] = cfg_.init_vel_std * cfg_.init_vel_std;
        init_diag[ATT_X+i] = cfg_.init_att_std * cfg_.init_att_std;
        init_diag[BG_X+i]  = cfg_.init_bg_std * cfg_.init_bg_std;
        init_diag[BA_X+i]  = cfg_.init_ba_std * cfg_.init_ba_std;
    }

    for (int i = 0; i < N; i++) {
        if (!(active_mask_ & (1 << i))) {
            // Frozen: reset diagonal, zero cross-covariance
            // フリーズ: 対角リセット、クロス共分散ゼロ化
            P_(i, i) = init_diag[i];
            for (int j = 0; j < N; j++) {
                if (j != i) {
                    P_(i, j) = 0;
                    P_(j, i) = 0;
                }
            }
        } else {
            // Active: enforce minimum diagonal
            // アクティブ: 対角下限を強制
            if (P_(i, i) < 1e-12f) P_(i, i) = 1e-12f;
        }
    }

    // Enforce symmetry / 対称性を強制
    P_.symmetrize();
}

// =============================================================================
// Apply Masked Error State / マスク付き誤差状態の適用
// =============================================================================

void EskfCore::applyMaskedErrorState(float dx[N])
{
    // Zero frozen states / フリーズ状態をゼロ化
    for (int i = 0; i < N; i++) {
        if (!(active_mask_ & (1 << i))) {
            dx[i] = 0;
        }
    }

    // Apply position correction / 位置補正を適用
    pos_.x += dx[POS_X];
    pos_.y += dx[POS_Y];
    pos_.z += dx[POS_Z];

    // Apply velocity correction / 速度補正を適用
    vel_.x += dx[VEL_X];
    vel_.y += dx[VEL_Y];
    vel_.z += dx[VEL_Z];

    // Apply attitude correction with clamp / クランプ付き姿勢補正を適用
    float clamp = cfg_.att_correction_clamp;
    float att_dx[3] = {
        fmaxf(-clamp, fminf(clamp, dx[ATT_X])),
        fmaxf(-clamp, fminf(clamp, dx[ATT_Y])),
        fmaxf(-clamp, fminf(clamp, dx[ATT_Z]))
    };
    Vec3 dtheta(att_dx[0], att_dx[1], att_dx[2]);
    Quat dq = Quat::from_rotvec(dtheta);
    q_ = q_ * dq;
    q_.normalize();

    // Apply bias corrections / バイアス補正を適用
    bg_.x += dx[BG_X];
    bg_.y += dx[BG_Y];
    bg_.z += dx[BG_Z];
    ba_.x += dx[BA_X];
    ba_.y += dx[BA_Y];
    ba_.z += dx[BA_Z];
}

}  // namespace sf
