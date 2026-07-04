/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

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
 * @design detailed_design.md §5 — IEstimator                         [OK]
 */

#include "eskf_core.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstdio>
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
    bg_nominal_ = {0, 0, 0};   // re-anchored by the next calibration seed / 次の校正種付けで再固定
    ang_rate_ = {0, 0, 0};

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
    accel_att_lpf_ = {0, 0, 0};   // accel-attitude LPF re-seeds on the next sample
    accel_lpf_init_ = false;

    // Drop the accel-compensation flow history so the first post-reset flow sample
    // re-seeds the α-β tracker without a spurious large difference.
    // 運動加速度補償のフロー履歴を破棄し、リセット後最初のフローが α-β を暴れずに再シード。
    have_flow_vel_ = false;
    flow_vel_lpf_  = {0, 0, 0};
    a_kin_ned_     = {0, 0, 0};

    // Drop the ToF differentiation history as well / ToF 微分履歴も破棄
    tof_have_prev_height_ = false;
    tof_prev_height_ = 0;

    // Recompute the active mask so it reflects the just-cleared freeze flag and the
    // current sensor/mag-gate state. init() does this after reset(), but the
    // standalone EskfEstimator::reset() path would otherwise leave the mask stale
    // (e.g. accel-bias bits frozen after a future freeze→reset) (code_review L-16).
    // freeze フラグのクリアと現在のセンサ/mag ゲート状態を反映するよう active_mask を
    // 再計算する。init() は reset() 後にこれを行うが、単独の EskfEstimator::reset()
    // 経路ではマスクが古いまま残りうる（将来 freeze→reset 後に加速度バイアスビットが
    // 凍結のまま等）(L-16)。
    recomputeActiveMask();
}

void EskfCore::resetPositionVelocity()
{
    pos_ = {0, 0, 0};
    vel_ = {0, 0, 0};

    // Velocity was just reset, so the stored flow velocity is stale — re-seed the
    // α-β tracker on the next flow sample to avoid a one-shot bogus a_kin.
    // 速度をリセットした直後ゆえ保存フロー速度は陳腐化 → 次フローで α-β を再シード。
    have_flow_vel_ = false;
    flow_vel_lpf_  = {0, 0, 0};
    a_kin_ned_     = {0, 0, 0};
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

void EskfCore::inflateCovariance(uint16_t state_mask)
{
    // For each selected state: reset its covariance diagonal to the init value and zero
    // its cross-covariance with every other state. The state estimate (pos_/vel_/q_/
    // bg_/ba_) is left UNTOUCHED — only the confidence is reset. This is the same diagonal
    // values reset() uses, applied selectively without disturbing x.
    // 選んだ各状態について: 共分散対角を init 値に戻し、他全状態とのクロス共分散をゼロ化する。
    // 状態推定値(pos_/vel_/q_/bg_/ba_)は触らず、自信だけリセット。reset() と同じ対角値を、
    // x を乱さず選択的に適用する。
    float init_diag[N];
    for (int i = 0; i < 3; i++) {
        init_diag[POS_X+i] = cfg_.init_pos_std * cfg_.init_pos_std;
        init_diag[VEL_X+i] = cfg_.init_vel_std * cfg_.init_vel_std;
        init_diag[ATT_X+i] = cfg_.init_att_std * cfg_.init_att_std;
        init_diag[BG_X+i]  = cfg_.init_bg_std * cfg_.init_bg_std;
        init_diag[BA_X+i]  = cfg_.init_ba_std * cfg_.init_ba_std;
    }

    for (int i = 0; i < N; i++) {
        if (!(state_mask & (1 << i))) {
            continue;   // not selected — leave its covariance as-is / 非選択は据え置き
        }
        for (int j = 0; j < N; j++) {
            P_(i, j) = 0;
            P_(j, i) = 0;
        }
        P_(i, i) = init_diag[i];
    }

    // Flow / α-β tracker history depends on the velocity covariance; drop it when velocity
    // confidence was inflated so the next flow sample re-seeds cleanly (mirrors reset()).
    // フロー/α-β 履歴は速度共分散に依存する。速度の自信を膨張したら破棄し次フローで再シード。
    if (state_mask & ((1 << VEL_X) | (1 << VEL_Y) | (1 << VEL_Z))) {
        have_flow_vel_ = false;
        flow_vel_lpf_  = {0, 0, 0};
        a_kin_ned_     = {0, 0, 0};
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
    accel_lpf_dt_ = dt;   // for the accel-attitude LPF (same IMU cycle) / accel 姿勢 LPF 用

    // Bias-corrected IMU / バイアス補正済みIMU
    Vec3 accel = accel_raw - ba_;
    Vec3 gyro  = gyro_raw - bg_;

    // Cache the bias-corrected body rate for the rate controller (FRD).
    // レート制御器のためバイアス補正済み機体角速度を保持（FRD）。
    ang_rate_ = gyro;

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
    // Covariance propagation P' = F·P·Fᵀ + Q, EXPLOITING F's SPARSITY.
    // 共分散伝搬 P' = F·P·Fᵀ + Q — F の「疎」構造を利用する。
    //
    // F = I + A, and A has ONLY these nonzero blocks (24 entries out of 225):
    //   A[pos][vel] = I·dt            A[vel][att] = D_va = −R[a_c×]·dt
    //   A[vel][ba]  = D_vb = −R·dt    A[att][bg]  = −I·dt
    // so (I+A)·P·(I+A)ᵀ reduces to two sparse in-place passes:
    //   pass 1 (rows):    FP = P + A·P     — only rows pos/vel/att change
    //   pass 2 (columns): P' = FP + FP·Aᵀ  — only cols pos/vel/att change
    // The in-place order pos → vel → att is EXACT because each updated
    // row/column reads only rows/columns that are modified later (vel, ba, bg)
    // — never one already modified in the same pass.
    // ~720 multiply-adds instead of the dense 2×15³ ≈ 6750: at 400Hz this is
    // the difference between saturating a core and a few percent of it
    // (a dense triple loop at -Og starved StateTask on hardware — INIT-stuck).
    // F = I + A で、A の非ゼロは 225 要素中わずか 24（上記4ブロック）。よって
    // (I+A)·P·(I+A)ᵀ は2回の疎なインプレースパスに帰着する:
    //   パス1（行）: FP = P + A·P — pos/vel/att 行だけが変わる
    //   パス2（列）: P' = FP + FP·Aᵀ — pos/vel/att 列だけが変わる
    // pos → vel → att のインプレース順は「厳密」: 更新する行/列が読むのは後で
    // 変更される行/列（vel, ba, bg）だけで、同一パス内で変更済みのものは読まない。
    // 密な三重ループの 2×15³ ≈ 6750 積和が ~720 積和に — 400Hz ではコア飽和と
    // 数%負荷の分かれ目（-Og の密ループは実機で StateTask を飢餓させた）。
    // =========================================================================

    // D_va = −R[a_c×]·dt — right-perturbation convention (q = q̂⊗δq, see
    // applyMaskedErrorState): R = R̂(I+[δθ×]) → δv̇ = −R̂[f×]δθ, MINUS R[a×].
    // Component check: (R[a×])_{i,0} = R[i][1]·az − R[i][2]·ay → negated below.
    // D_va = −R[a_c×]·dt — 右側摂動規約（q = q̂⊗δq、applyMaskedErrorState 参照）:
    // R = R̂(I+[δθ×]) → δv̇ = −R̂[f×]δθ、つまり −R[a×]。
    // 成分確認: (R[a×])_{i,0} = R[i][1]·az − R[i][2]·ay → 下は符号反転形。
    float Dva[3][3];
    float Dvb[3][3];
    for (int i = 0; i < 3; i++) {
        Dva[i][0] = (R[i][2]*accel.y - R[i][1]*accel.z) * dt;
        Dva[i][1] = (R[i][0]*accel.z - R[i][2]*accel.x) * dt;
        Dva[i][2] = (R[i][1]*accel.x - R[i][0]*accel.y) * dt;
        for (int j = 0; j < 3; j++) {
            Dvb[i][j] = -R[i][j] * dt;
        }
    }

    // Pass 1 (rows): FP = P + A·P — every read row (vel, att, ba, bg) is still
    // the ORIGINAL P at the moment it is read (see ordering note above).
    // パス1（行）: FP = P + A·P — 読む行（vel, att, ba, bg）は読む時点で全て
    // 「元の」P（上の順序の注を参照）。
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < 3; i++) {
            P_(POS_X + i, j) += dt * P_(VEL_X + i, j);
        }
    }
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < 3; i++) {
            float acc = 0;
            for (int k = 0; k < 3; k++) {
                acc += Dva[i][k] * P_(ATT_X + k, j)
                     + Dvb[i][k] * P_(BA_X + k, j);
            }
            P_(VEL_X + i, j) += acc;
        }
    }
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < 3; i++) {
            P_(ATT_X + i, j) -= dt * P_(BG_X + i, j);
        }
    }

    // Pass 2 (columns): P' = FP + FP·Aᵀ — same ordering argument on columns.
    // パス2（列）: P' = FP + FP·Aᵀ — 列について同じ順序論法。
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            P_(i, POS_X + j) += dt * P_(i, VEL_X + j);
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            float acc = 0;
            for (int k = 0; k < 3; k++) {
                acc += P_(i, ATT_X + k) * Dva[j][k]
                     + P_(i, BA_X + k) * Dvb[j][k];
            }
            P_(i, VEL_X + j) += acc;
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            P_(i, ATT_X + j) -= dt * P_(i, BG_X + j);
        }
    }

    // Numerical-symmetry repair (the two passes are exact, but float rounding
    // differs between (i,j) and (j,i) paths).
    // 数値対称性の補修（2パスは厳密だが、(i,j) と (j,i) で丸めが異なる）。
    P_.symmetrize();

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
    // w = P·hᵀ — one O(N²) pass; everything below is rank-1 algebra on w.
    // w = P·hᵀ — O(N²) は1回だけ。以降は w のランク1代数。
    float w[N];
    for (int i = 0; i < N; i++) {
        w[i] = 0;
        for (int j = 0; j < N; j++)
            w[i] += P_(i, j) * H[j];
    }

    // S = h·P·hᵀ + R = h·w + R
    float S = R_val;
    for (int i = 0; i < N; i++)
        S += H[i] * w[i];

    if (S < 1e-10f) return;  // Singular / 特異

    // K = w / S
    float K[N];
    for (int i = 0; i < N; i++)
        K[i] = w[i] / S;

    // Error state: dx = K * innovation
    // 誤差状態: dx = K * innovation
    float dx[N];
    for (int i = 0; i < N; i++)
        dx[i] = K[i] * innovation;

    // Apply masked error state / マスク付き誤差状態を適用
    applyMaskedErrorState(dx);

    // Joseph covariance update, expanded into its RANK-1 form. With h a row
    // vector, (I−Khᵀ)P(I−Khᵀ)ᵀ + R·KKᵀ expands (using hᵀP = wᵀ by symmetry and
    // hᵀPh + R = S) to exactly:
    //   P' = P − K wᵀ − w Kᵀ + S·K Kᵀ
    // — three rank-1 terms, ~4·N²/2 multiply-adds. The previous literal
    // (I−KH)P(I−KH)ᵀ double-contraction was O(N⁴) ≈ 54 000 multiply-adds per
    // scalar observation (flow alone calls this 200×/s).
    // Joseph 共分散更新の「ランク1」展開形。h が行ベクトルのとき
    // (I−Khᵀ)P(I−Khᵀ)ᵀ + R·KKᵀ は（対称性より hᵀP = wᵀ、hᵀPh + R = S を使い）
    // 厳密に P' = P − K wᵀ − w Kᵀ + S·K Kᵀ の3つのランク1項に展開できる
    // （~4·N²/2 積和）。従来の (I−KH)P(I−KH)ᵀ の逐語的二重縮約は O(N⁴) ≈
    // 1観測あたり5.4万積和だった（フローだけで毎秒200回呼ぶ）。
    for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
            const float val = P_(i, j)
                            - K[i] * w[j] - w[i] * K[j]
                            + S * K[i] * K[j];
            P_(i, j) = val;
            P_(j, i) = val;
        }
    }

    enforceCovarianceConstraints();
}

// =============================================================================
// 3D Kalman Update / 3次元カルマン更新
// =============================================================================

void EskfCore::vectorUpdate3(const float H[3][N], const float innov[3], float R_val,
                             float chi2_gate)
{
    // =========================================================================
    // Exploit H's COLUMN SPARSITY. The 3×N observation Jacobians used here have
    // at most a handful of nonzero columns (accel-attitude: att + ba = 6 of 15;
    // mag: att = 3 of 15). Every contraction below runs over that support
    // instead of all N columns — at 400Hz (accel-attitude every predict) this
    // is the difference between ~14 000 and ~2 000 multiply-adds per call.
    // H の「列疎性」を利用する。ここで使う 3×N 観測ヤコビアンの非ゼロ列はごく少数
    // （accel-attitude: att+ba の 6/15、mag: att の 3/15）。以降の縮約は全 N 列で
    // なくこのサポート上だけを走る — 400Hz（accel-attitude は predict 毎）では
    // 1 回あたり ~14,000 積和と ~2,000 積和の分かれ目。
    // =========================================================================

    int cols[N];
    int n_cols = 0;
    for (int c = 0; c < N; c++) {
        if (H[0][c] != 0.0f || H[1][c] != 0.0f || H[2][c] != 0.0f) {
            cols[n_cols++] = c;
        }
    }
    if (n_cols == 0) {
        return;   // empty Jacobian — nothing to update / 空ヤコビアン
    }

    // PHt = P·Hᵀ (N×3), contracted over the support only.
    // PHt = P·Hᵀ（N×3）。縮約はサポート上のみ。
    float PHt[N][3];
    for (int i = 0; i < N; i++) {
        for (int k = 0; k < 3; k++) {
            float acc = 0;
            for (int m = 0; m < n_cols; m++) {
                acc += P_(i, cols[m]) * H[k][cols[m]];
            }
            PHt[i][k] = acc;
        }
    }

    // S = H·(P·Hᵀ) + R·I (3x3)
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = (i == j) ? R_val : 0.0f;
            for (int m = 0; m < n_cols; m++) {
                S[i][j] += H[i][cols[m]] * PHt[cols[m]][j];
            }
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

    // Chi-squared gate. The threshold is passed in by the caller so each vector
    // observation uses its own gate (accel → accel_chi2_gate, mag → mag_chi2_gate)
    // rather than sharing one constant.
    // χ²ゲート。閾値は呼び出し側が渡す。各ベクトル観測が自分のゲートを使う
    // (accel→accel_chi2_gate, mag→mag_chi2_gate)。1 定数を共有しない。
    if (d2 > chi2_gate) {
        return;
    }

    // K = (P·Hᵀ)·S⁻¹ (N×3) — PHt is already in hand.
    // K = (P·Hᵀ)·S⁻¹（N×3）— PHt は計算済み。
    float K[N][3];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 3; k++)
                K[i][j] += PHt[i][k] * Si[k][j];
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

    // Covariance update P' = (I − K·H)·P = P − (K·H)·P, exploiting H's column
    // sparsity twice: M = K·H is nonzero ONLY in the support columns, so
    // (M·P)(i,j) needs only the support ROWS of P. Those rows are snapshotted
    // first because the in-place update overwrites them. Cost: N·M·3 (build M)
    // + N·N·M (apply) ≈ 1 600 multiply-adds for M = 6 — the previous literal
    // expansion of (I−KH) was ~14 000 per call, at 400Hz.
    // 共分散更新 P' = (I − K·H)·P = P − (K·H)·P。H の列疎性を二重に利用する:
    // M = K·H の非ゼロはサポート列のみ → (M·P)(i,j) は P のサポート「行」しか
    // 要らない。その行はインプレース更新で上書きされるため先にスナップショット
    // する。コスト: N·M·3（M 構築）+ N·N·M（適用）≈ M=6 で 1,600 積和 — 従来の
    // (I−KH) の逐語展開は 1 回あたり ~14,000 積和を 400Hz で回していた。
    // Sized for the worst case n_cols = N (~900B each on the 16KB ImuTask
    // stack); the practical Jacobians here use 3–6 columns.
    // 最悪 n_cols = N でも収まるサイズ（各 ~900B、ImuTask スタックは 16KB）。
    // 実際のヤコビアンは 3〜6 列。
    float M[N][N];                  // K·H, support columns only / サポート列のみ
    float P_rows[N][N];             // snapshot of P's support rows / サポート行の写し
    for (int m = 0; m < n_cols; m++) {
        const int c = cols[m];
        for (int j = 0; j < N; j++) {
            P_rows[m][j] = P_(c, j);
        }
        for (int i = 0; i < N; i++) {
            float acc = 0;
            for (int k = 0; k < 3; k++)
                acc += K[i][k] * H[k][c];
            M[i][m] = acc;
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float acc = 0;
            for (int m = 0; m < n_cols; m++)
                acc += M[i][m] * P_rows[m][j];
            P_(i, j) -= acc;
        }
    }
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
    // Previous height is a member (NOT a function-local static): a static would
    // survive reset()/landing handoff and be shared between instances, injecting
    // a spurious velocity spike on the first sample after re-takeoff.
    // 前回高度はメンバ変数で保持（関数ローカル static は不可）: static だと reset()/
    // 着陸ハンドオフ後も履歴が残り、複数インスタンス間で共有され、再離陸後の初回
    // サンプルで偽の速度スパイクを注入してしまう。
    if (!tof_have_prev_height_) {
        tof_prev_height_ = height;
        tof_have_prev_height_ = true;
        return;
    }
    float vel_z_observed = -(height - tof_prev_height_) / dt;
    tof_prev_height_ = height;

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
    // Two independent gates: the param enable (cfg_.use_mag) AND the calibration
    // gate (mag_calib_gate_, survives reloadParams) — see eskf_core.hpp (L-5).
    // 2つの独立ゲート: param 有効化 (cfg_.use_mag) と校正ゲート (mag_calib_gate_,
    // reloadParams を生き延びる) — eskf_core.hpp 参照 (L-5)。
    if (!cfg_.use_mag || !mag_calib_gate_) return;

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

    // Chi-squared outlier rejection happens inside vectorUpdate3 using the
    // mag-specific gate (a magnetic disturbance shows up as a large innovation).
    // χ² 外れ値棄却は vectorUpdate3 内で mag 専用ゲートを使って行う
    // (磁気外乱は大きなイノベーションとして現れる)。
    vectorUpdate3(H, innov, cfg_.mag_noise * cfg_.mag_noise, cfg_.mag_chi2_gate);
}

void EskfCore::updateAccelAttitude(const Vec3& accel_raw)
{
    // Optional 1-pole LPF on the accel BEFORE the gravity comparison (cfg_.accel_att_lpf_hz).
    // Cleans airframe vibration from the gravity reference so the bias is pulled less and
    // fewer updates are χ²-rejected (flight-log sweep). Filters accel_raw (pre-bias), exactly
    // as the offline harness did — predict()'s velocity integration is left UNfiltered.
    // 重力比較の前に accel へ任意の1次 LPF（cfg_.accel_att_lpf_hz）。重力基準から機体振動を
    // 除き、バイアスの引きずりと χ² 棄却を減らす（実機ログ掃引）。accel_raw（バイアス前）を
    // 濾波＝オフラインハーネスと同一。predict() の速度積分は濾波しない。
    Vec3 accel_src = accel_raw;
    if (cfg_.accel_att_lpf_hz > 0.0f && accel_lpf_dt_ > 0.0f) {
        if (!accel_lpf_init_) { accel_att_lpf_ = accel_raw; accel_lpf_init_ = true; }
        const float a = 1.0f - expf(-2.0f * 3.14159265f * cfg_.accel_att_lpf_hz * accel_lpf_dt_);
        accel_att_lpf_.x += a * (accel_raw.x - accel_att_lpf_.x);
        accel_att_lpf_.y += a * (accel_raw.y - accel_att_lpf_.y);
        accel_att_lpf_.z += a * (accel_raw.z - accel_att_lpf_.z);
        accel_src = accel_att_lpf_;
    }

    // Bias-corrected accel / バイアス補正済み加速度
    Vec3 accel = accel_src - ba_;

    // Adaptive R scaling — DOWNWEIGHT (do not hard-gate) when |a| deviates from g.
    // R = R_base² * (1 + k * |‖a‖ - g|²). During thrust/maneuver transients the
    // specific force contaminates gravity, but a HARD norm gate (early return) discards
    // exactly the corrective updates while the craft is tilted and leaves the attitude
    // blind → it estimates "level" during a coordinated tilt+accelerate. The proven
    // firmware/vehicle has NO norm gate; it relies on the adaptive R plus the χ²
    // outlier gate inside vectorUpdate3. We do the same here.
    // 適応Rスケーリング — |a|がgから逸脱したら補正を弱める(ハードゲートしない)。推力/
    // マニューバ過渡で比力が重力を汚染するが、ハードな norm gate(早期return)は傾斜中の
    // corrective な更新ごと捨て姿勢を盲目化し、協調傾斜+加速で「水平」と誤推定する。実証済み
    // firmware/vehicle に norm gate は無く、適応R＋vectorUpdate3 内の χ² 外れ値ゲートで捌く。
    float gravity_diff = accel.norm() - cfg_.gravity;

    // Expected gravity in body: g_body = R^T * [0, 0, -g]
    // ボディ座標の期待重力: g_body = R^T * [0, 0, -g]
    Vec3 g_ned = {0, 0, -cfg_.gravity};
    Vec3 g_expected = q_.inv_rotate(g_ned);

    float R_val = cfg_.accel_att_noise * cfg_.accel_att_noise
                * (1.0f + cfg_.k_adaptive * gravity_diff * gravity_diff);

    // Predicted specific force. Plain: just gravity (g_expected). With acceleration
    // compensation: add the flow-derived kinematic acceleration in body, so f_pred =
    // g_expected + R^T·a_kin and the residual is the TRUE attitude error instead of the
    // kinematic term. At hover (a_kin≈0) h_vec == g_expected, so it reduces to the plain
    // update there. 予測比力。素では重力のみ(g_expected)。運動加速度補償ありではフロー由来の
    // 運動加速度を body で加算 f_pred = g_expected + R^T·a_kin → 残差が運動加速度項でなく真の
    // 姿勢誤差に。ホバー(a_kin≈0)では h_vec==g_expected ゆえ素の更新に一致。
    Vec3 h_vec = g_expected;
    if (cfg_.accel_comp_enable) {
        Vec3 a_kin_body = q_.inv_rotate(a_kin_ned_);
        h_vec.x += a_kin_body.x;
        h_vec.y += a_kin_body.y;
        h_vec.z += a_kin_body.z;
    }

    float innov[3] = {
        accel.x - h_vec.x,
        accel.y - h_vec.y,
        accel.z - h_vec.z
    };

    // H matrix: attitude part + accel bias part
    // H行列: 姿勢部分 + 加速度バイアス部分
    float R_dcm[3][3];
    q_.to_dcm(R_dcm);

    float H[3][N] = {};
    // Attitude columns: -[h_vec×] (h_vec = predicted specific force; = g_expected baseline)
    H[0][ATT_Y] = -h_vec.z;  H[0][ATT_Z] =  h_vec.y;
    H[1][ATT_X] =  h_vec.z;  H[1][ATT_Z] = -h_vec.x;
    H[2][ATT_X] = -h_vec.y;  H[2][ATT_Y] =  h_vec.x;

    // Accel bias columns: I (direct observation)
    // 加速度バイアス列: I（直接観測）
    H[0][BA_X] = 1.0f;
    H[1][BA_Y] = 1.0f;
    H[2][BA_Z] = 1.0f;

    vectorUpdate3(H, innov, R_val, cfg_.accel_chi2_gate);
}

void EskfCore::updateFlowRaw(int16_t dx, int16_t dy, float height,
                              float dt, float gyro_x, float gyro_y, uint8_t squal)
{
    if (!cfg_.use_flow) return;
    // Reject low-quality flow before it reaches the filter: a poor surface gives
    // noisy displacement that would inject spurious horizontal velocity into
    // POS_HOLD (code_review L-1). Gate FIRST so neither the Kalman update nor the
    // accel-comp α-β tracker downstream ever sees a bad sample.
    // 低品質フローはフィルタ前に棄却: 品質の悪い面はノイズ変位を出し POS_HOLD へ偽の
    // 水平速度を注入する (L-1)。先にゲートし、Kalman 更新も下流の accel-comp α-β
    // トラッカも不良サンプルを見ないようにする。
    if (squal < cfg_.flow_min_squal) return;
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

    // Acceleration-compensated accel-attitude (POS_HOLD): estimate the horizontal NED
    // kinematic acceleration a_kin with an α-β tracker on the flow velocity. The flow
    // velocity is used (NOT the fused vel_, which re-injects the accel via predict and makes
    // the pitch axis self-reinforce), so a_kin stays INDEPENDENT of attitude-from-accel. The
    // α-β filter (state = velocity flow_vel_lpf_ + acceleration a_kin_ned_) captures the
    // SUSTAINED drift acceleration — a naive high-pass derivative washes out the DC term and
    // only passes oscillation, which the per-axis tests held but the diagonal did not. Fixed
    // nominal period (jitter-proof) + physical clamp. updateAccelAttitude subtracts R^T·a_kin.
    // 運動加速度補償の accel-attitude（POS_HOLD）: フロー速度の α-β トラッカで水平 NED 運動加速度
    // a_kin を推定。フロー速度を使う（融合 vel_ は predict で accel を再注入しピッチ軸が自己強化
    // ゆえ不可）ので a_kin は姿勢-加速度と独立。α-β（状態=速度 flow_vel_lpf_ + 加速度 a_kin_ned_）
    // は持続ドリフト加速度を捉える — 単純高域微分は DC を washout し振動のみ通すため軸別は保持
    // できても斜めが破綻した。公称周期（ジッタ耐性）＋物理クランプ。updateAccelAttitude が R^T·a_kin を差し引く。
    if (cfg_.accel_comp_enable) {
        constexpr float kFlowDt = 0.01f;               // nominal flow period (100 Hz)
        const float alpha = cfg_.accel_comp_alpha;     // α-β velocity gain
        const float beta  = cfg_.accel_comp_beta;      // α-β acceleration gain
        const float amax  = cfg_.accel_comp_max;
        if (have_flow_vel_) {
            // Predict the velocity forward by the acceleration state, correct with the flow
            // velocity, fold the residual into BOTH states. 加速度状態で速度を予測→フロー速度で
            // 補正→残差を両状態へ。
            const float vpx = flow_vel_lpf_.x + a_kin_ned_.x * kFlowDt;
            const float vpy = flow_vel_lpf_.y + a_kin_ned_.y * kFlowDt;
            const float rx = vx_ned - vpx;             // velocity residual
            const float ry = vy_ned - vpy;
            flow_vel_lpf_.x = vpx + alpha * rx;
            flow_vel_lpf_.y = vpy + alpha * ry;
            a_kin_ned_.x += (beta / kFlowDt) * rx;
            a_kin_ned_.y += (beta / kFlowDt) * ry;
            a_kin_ned_.x = fmaxf(-amax, fminf(amax, a_kin_ned_.x));   // physical clamp
            a_kin_ned_.y = fmaxf(-amax, fminf(amax, a_kin_ned_.y));
        } else {
            flow_vel_lpf_ = {vx_ned, vy_ned, 0.0f};    // seed the velocity state
        }
        have_flow_vel_ = true;
    }
}

// =============================================================================
// Active Mask / 有効マスク
// =============================================================================

void EskfCore::setConfig(const EskfConfig& cfg)
{
    // State x and covariance P stay untouched — this is live tuning, not a
    // re-initialization. The active mask is recomputed because the use_*
    // switches may have changed (P isolation follows the mask).
    // 状態 x と共分散 P には触れない — これは再初期化でなくライブチューニング。
    // use_* スイッチが変わった可能性があるため active mask を再計算する
    // （P の隔離はマスクに従う）。
    cfg_ = cfg;
    recomputeActiveMask();
}

void EskfCore::setSensorEnabled(int group, bool enabled)
{
    // group: 0=TOF, 1=BARO, 2=MAG calibration gate, 3=FLOW.
    // Group 2 drives the calibration gate (mag_calib_gate_), NOT cfg_.use_mag:
    // the param enable comes from setConfig()/reloadParams(), and writing
    // cfg_.use_mag here would be undone by the next reload — exactly the bug L-5
    // fixes. The mag's param enable and calibration gate are separate; mag fuses
    // only when both hold. (TOF/BARO/FLOW have a single gate, so they map to cfg_.)
    // group: 0=TOF, 1=BARO, 2=MAG 校正ゲート, 3=FLOW。group 2 は校正ゲート
    // (mag_calib_gate_) を駆動し cfg_.use_mag は触らない: param 有効化は
    // setConfig()/reloadParams() 由来で、ここで cfg_.use_mag を書くと次の reload で
    // 取り消される（L-5 が直すバグそのもの）。mag の param 有効化と校正ゲートは別物で、
    // 両方成立時のみ融合する。（TOF/BARO/FLOW は単一ゲートゆえ cfg_ に対応。）
    switch (group) {
        case 0: cfg_.use_tof   = enabled; break;
        case 1: cfg_.use_baro  = enabled; break;
        case 2: mag_calib_gate_ = enabled; break;
        case 3: cfg_.use_flow  = enabled; break;
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

    // If no MAG aiding (param disabled OR uncalibrated gate down): freeze ATT_Z,
    // BG_Z — same condition as updateMag, so the yaw states are isolated whenever
    // mag is not actually fused (L-5).
    // MAG 補正なし（param 無効 or 未校正ゲート降下）: ATT_Z, BG_Z をフリーズ。
    // updateMag と同条件にし、mag が実際に融合されない間はヨー状態を隔離する (L-5)。
    if (!cfg_.use_mag || !mag_calib_gate_) {
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
    // Compute roll/pitch from the SPECIFIC FORCE direction (f = −gravity_body at
    // rest). accel_avg at rest ≈ [0, 0, −g] when level. For true roll +φ the rest
    // reading is f = [0, −g·sinφ, −g·cosφ]; for true pitch +θ it is
    // f = [+g·sinθ, 0, −g·cosθ]. Hence:
    //   roll  = atan2(−ay, −az)   (= φ)
    //   pitch = atan2(+ax, sqrt(ay² + az²))   (= θ)
    // (The previous atan2(ay, −az) / atan2(−ax, …) returned −φ/−θ — the sign
    // confusion between specific force f and gravity g.)
    // 「比力」の方向から roll/pitch を計算（静止時 f = −重力(機体)）。水平静止で
    // accel_avg ≈ [0,0,−g]。真のロール +φ では f = [0, −g·sinφ, −g·cosφ]、
    // 真のピッチ +θ では f = [+g·sinθ, 0, −g·cosθ]。よって:
    //   roll  = atan2(−ay, −az)（= φ）、pitch = atan2(+ax, √(ay²+az²))（= θ）。
    // （従来の atan2(ay,−az)/atan2(−ax,…) は −φ/−θ を返していた — 比力 f と重力 g の
    // 符号取り違え。）
    float roll  = atan2f(-accel_avg.y, -accel_avg.z);
    float pitch = atan2f(accel_avg.x,
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

    // Gyro-bias deviation clamp around the boot-calibration nominal (see
    // EskfConfig::bg_deviation_max). Every observation reaches the bias states
    // through the cross-covariances; this bounds how far a misbehaving sensor
    // can drag the bias that feeds the rate loop. The accel bias is NOT clamped:
    // it does not feed any control loop directly and its physical range
    // (offset + temperature drift) is far wider.
    // 起動校正ノミナルまわりのジャイロバイアス偏差クランプ（EskfConfig::
    // bg_deviation_max 参照）。全観測はクロス共分散経由でバイアス状態に届くため、
    // 異常センサがレートループに使うバイアスを引きずれる距離をここで有界化する。
    // 加速度バイアスはクランプしない: 制御ループに直接入らず、物理範囲
    // （オフセット＋温度ドリフト）もずっと広い。
    const float dev = cfg_.bg_deviation_max;
    bg_.x = fmaxf(bg_nominal_.x - dev, fminf(bg_nominal_.x + dev, bg_.x));
    bg_.y = fmaxf(bg_nominal_.y - dev, fminf(bg_nominal_.y + dev, bg_.y));
    bg_.z = fmaxf(bg_nominal_.z - dev, fminf(bg_nominal_.z + dev, bg_.z));
}

}  // namespace sf
