/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file eskf_estimator.cpp
 * @brief ESKF estimator — IEstimator adapter
 *        ESKF推定器 — IEstimatorアダプター
 *
 * @design detailed_design.md §5 — IEstimator                         [OK]
 * @design detailed_design.md §5 — Sensor observation switch           [OK]
 */

#include "eskf_estimator.hpp"
#include "params.hpp"
#include "esp_log.h"

static const char* TAG = "ESKF";

namespace sf {

// -----------------------------------------------------------------------------
// loadConfigFromParams — build an EskfConfig from the parameter system. Shared
// by init() (boot) and reloadParams() (live tuning via ReloadParams verb).
// loadConfigFromParams — パラメータシステムから EskfConfig を構築する。init()
// （起動時）と reloadParams()（ReloadParams verb によるライブチューニング）が共用。
// -----------------------------------------------------------------------------
static EskfConfig loadConfigFromParams()
{
    EskfConfig cfg;

    // Load from parameter system / パラメータシステムから読み込み
    params::get_float("eskf.process.gyro_noise", cfg.gyro_noise);
    params::get_float("eskf.process.accel_noise", cfg.accel_noise);
    params::get_float("eskf.process.gyro_bias", cfg.gyro_bias_noise);
    params::get_float("eskf.process.accel_bias", cfg.accel_bias_noise);
    params::get_float("eskf.obs.tof_noise", cfg.tof_noise);
    params::get_float("eskf.obs.flow_noise", cfg.flow_noise);
    params::get_float("eskf.obs.baro_noise", cfg.baro_noise);
    params::get_float("eskf.obs.mag_noise", cfg.mag_noise);
    params::get_float("eskf.obs.accel_att_noise", cfg.accel_att_noise);
    params::get_float("eskf.bias.gyro_dev_max", cfg.bg_deviation_max);
    params::get_float("eskf.gate.tof_innov", cfg.tof_innov_gate);
    params::get_float("eskf.gate.baro_innov", cfg.baro_innov_gate);
    params::get_float("eskf.gate.flow_clamp", cfg.flow_innov_clamp);

    // Accel-attitude (SSOT — these used to silently take the struct defaults).
    // 加速度-姿勢（SSOT — 以前は struct 既定値を暗黙使用していた）。
    params::get_float("eskf.att.k_adaptive", cfg.k_adaptive);
    params::get_float("eskf.att.chi2_gate",  cfg.accel_chi2_gate);
    params::get_float("eskf.att.corr_clamp", cfg.att_correction_clamp);

    params::get_bool("eskf.use_tof", cfg.use_tof);
    params::get_bool("eskf.use_flow", cfg.use_flow);
    params::get_bool("eskf.use_baro", cfg.use_baro);
    params::get_bool("eskf.use_mag", cfg.use_mag);

    // Acceleration-compensated accel-attitude (POS_HOLD; α-β flow-acceleration tracker).
    // 運動加速度補償の accel-attitude（POS_HOLD; α-β フロー加速度トラッカ）。
    params::get_bool("eskf.accel_comp.enable", cfg.accel_comp_enable);
    params::get_float("eskf.accel_comp.alpha", cfg.accel_comp_alpha);
    params::get_float("eskf.accel_comp.beta",  cfg.accel_comp_beta);
    params::get_float("eskf.accel_comp.max",   cfg.accel_comp_max);

    return cfg;
}

void EskfEstimator::init()
{
    const EskfConfig cfg = loadConfigFromParams();

    core_.init(cfg);
    cached_state_ = {};
    cached_state_.attitude[0] = 1.0f;

    ESP_LOGI(TAG, "ESKF initialized (tof=%d flow=%d baro=%d mag=%d accel_comp=%d a=%.2f b=%.3f)",
             cfg.use_tof, cfg.use_flow, cfg.use_baro, cfg.use_mag,
             cfg.accel_comp_enable, cfg.accel_comp_alpha, cfg.accel_comp_beta);
}

void EskfEstimator::reloadParams()
{
    // Live tuning: re-read every eskf.* parameter and apply WITHOUT resetting
    // the filter state (setConfig keeps x and P). Issued via
    // EstimatorCmd::ReloadParams from the params-change callback.
    // ライブチューニング: 全 eskf.* パラメータを読み直し、フィルタ状態をリセット
    // せずに適用する（setConfig は x と P を保持）。パラメータ変更コールバックの
    // EstimatorCmd::ReloadParams 経由で実行される。
    core_.setConfig(loadConfigFromParams());
    ESP_LOGI(TAG, "ESKF parameters reloaded (live)");
}

void EskfEstimator::predict(const ImuData& imu, float dt)
{
    math::Vec3 accel(imu.accel[0], imu.accel[1], imu.accel[2]);
    math::Vec3 gyro(imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    core_.predict(accel, gyro, dt);

    // Also run accel attitude update every predict cycle
    // 毎予測周期で加速度姿勢更新も実行
    core_.updateAccelAttitude(accel);

    cached_state_ = convertState(imu.timestamp);
}

void EskfEstimator::updateTof(const TofData& tof)
{
    if (!tof.valid) return;
    core_.updateToF(tof.distance);
}

void EskfEstimator::updateFlow(const FlowData& flow)
{
    // Wrap-safe dt: unsigned subtraction yields the true elapsed µs even across
    // the uint32 wrap (~71.6 min). Absurd gaps (>1 s) re-seed with the nominal dt.
    // ラップ安全な dt: 符号なし減算は uint32 ラップ（約71.6分）を跨いでも正しい経過 µs を
    // 返す。異常な間隔（>1秒）は公称 dt で再シード。
    float dt = 0.01f;
    if (last_flow_time_ != 0) {
        const uint32_t elapsed_us = flow.timestamp - last_flow_time_;
        if (elapsed_us > 0 && elapsed_us < 1000000) {
            dt = static_cast<float>(elapsed_us) * 1e-6f;
        }
    }
    last_flow_time_ = flow.timestamp;

    float height = -core_.getPosition().z;
    if (height < 0.02f) height = 0.02f;

    // Gyro compensation needs the body ANGULAR RATE (bias-corrected gyro), not the
    // gyro bias — pitch/roll rotation adds a rotational component to the optical flow
    // that updateFlowRaw removes via flow_gyro_scale * rate. Passing the bias (~0)
    // left the compensation effectively disabled, injecting spurious horizontal
    // velocity during rotation.
    // ジャイロ補償には body 角速度（バイアス補正済みジャイロ）が必要で、バイアスではない。
    // pitch/roll 回転はフローに回転成分を加え、updateFlowRaw が flow_gyro_scale·rate で
    // 除去する。バイアス(≈0)を渡すと補償が実質無効になり、回転中に偽の水平速度が入る。
    core_.updateFlowRaw(flow.dx, flow.dy, height, dt,
                        cached_state_.angular_rate[0],   // body roll rate (gyro_x, FRD)
                        cached_state_.angular_rate[1]);  // body pitch rate (gyro_y, FRD)
}

void EskfEstimator::updateMag(const MagData& mag)
{
    math::Vec3 m(mag.mag[0], mag.mag[1], mag.mag[2]);
    core_.updateMag(m);
}

void EskfEstimator::updateBaro(const BaroData& baro)
{
    core_.updateBaro(baro.altitude);
}

StateEstimate EskfEstimator::getState() const
{
    return cached_state_;
}

void EskfEstimator::reset()
{
    core_.reset();
    cached_state_ = {};
    cached_state_.attitude[0] = 1.0f;
}

void EskfEstimator::resetPositionVelocity()
{
    core_.resetPositionVelocity();
    for (int i = 0; i < 3; i++) {
        cached_state_.position[i] = 0;
        cached_state_.velocity[i] = 0;
    }
}

void EskfEstimator::holdPositionVelocity()
{
    // Clamp pos/vel to zero (no covariance change) — anchors the estimate at the
    // known ground state each cycle while the craft is on the ground.
    // pos/vel をゼロに固定（共分散は変えない）— 接地中、毎サイクル既知の地上状態に錨を打つ。
    core_.holdPositionVelocity();
    for (int i = 0; i < 3; i++) {
        cached_state_.position[i] = 0;
        cached_state_.velocity[i] = 0;
    }
}

void EskfEstimator::applyCalibration(const float gyro_bias[3], const float accel_bias[3])
{
    // Seed the filter's bias states with the rest-measured biases. The ESKF predict
    // subtracts these (accel − ba_, gyro − bg_), so a correct seed removes the raw
    // offset from cycle 1 — the filter need not converge from zero through the
    // takeoff transient (where an uncalibrated accel bias destabilizes attitude).
    // We deliberately do NOT shrink the bias covariance: keeping the normal
    // covariance lets the filter still track the slow in-flight bias drift (random
    // walk), and makes a zero-bias calibration (a clean IMU) an EXACT no-op, so the
    // default SIL path stays regression-neutral.
    // フィルタのバイアス状態を静止測定値で種付けする。ESKF predict はこれを引く
    // （accel−ba_, gyro−bg_）ので、正しい種は1サイクル目から生オフセットを除去する
    // （未校正の加速度バイアスが姿勢を不安定化する離陸過渡を、ゼロから収束しない）。
    // バイアス共分散は意図的に縮小しない: 通常の共分散を保てば飛行中の緩いバイアス
    // ドリフト（ランダムウォーク）を追え、ゼロバイアス校正（クリーンな IMU）が厳密
    // no-op になり、既定 SIL 経路が回帰中立に保たれる。
    core_.setGyroBias(math::Vec3(gyro_bias[0], gyro_bias[1], gyro_bias[2]));
    core_.setAccelBias(math::Vec3(accel_bias[0], accel_bias[1], accel_bias[2]));

    // Reflect the seeded biases in the cached snapshot immediately (the next predict
    // overwrites it, but getState() before then should report the calibration).
    // 種付けしたバイアスをキャッシュにも即反映（次の predict が上書きするが、それまでの
    // getState() は校正値を返すべき）。
    for (int i = 0; i < 3; ++i) {
        cached_state_.gyro_bias[i]  = gyro_bias[i];
        cached_state_.accel_bias[i] = accel_bias[i];
    }
}

void EskfEstimator::freezeBias()
{
    // Hold the accel bias constant (the ESKF core stops updating ba_). Called on the
    // ground / at landing so settling vibration does not perturb the calibrated bias.
    // 加速度バイアスを一定に保つ（ESKF コアが ba_ 更新を停止）。地上・着陸時に呼び、整定の
    // 振動が校正済みバイアスを乱さないようにする。
    core_.setFreezeAccelBias(true);
}

void EskfEstimator::unfreezeBias()
{
    // Resume online accel-bias estimation for the dynamic flight regime (at takeoff).
    // 動的な飛行域での加速度バイアスのオンライン推定を再開（離陸時）。
    core_.setFreezeAccelBias(false);
}

void EskfEstimator::inflateCovariance(uint16_t state_mask)
{
    // Inflate the selected states' covariance back to uncertain (keep the estimate). The
    // cached snapshot's state is unchanged; the covariance is internal to core_, so no
    // cached_state_ update is needed.
    // 選んだ状態の共分散を不確かへ膨張（推定値は保持）。キャッシュの状態は不変で、共分散は
    // core_ 内部ゆえ cached_state_ の更新は不要。
    core_.inflateCovariance(state_mask);
}

StateEstimate EskfEstimator::convertState(uint32_t timestamp) const
{
    StateEstimate s = {};

    auto q = core_.getAttitude();
    s.attitude[0] = q.w; s.attitude[1] = q.x;
    s.attitude[2] = q.y; s.attitude[3] = q.z;

    auto p = core_.getPosition();
    s.position[0] = p.x; s.position[1] = p.y; s.position[2] = p.z;

    auto v = core_.getVelocity();
    s.velocity[0] = v.x; s.velocity[1] = v.y; s.velocity[2] = v.z;

    auto gb = core_.getGyroBias();
    s.gyro_bias[0] = gb.x; s.gyro_bias[1] = gb.y; s.gyro_bias[2] = gb.z;

    auto ab = core_.getAccelBias();
    s.accel_bias[0] = ab.x; s.accel_bias[1] = ab.y; s.accel_bias[2] = ab.z;

    auto w = core_.getAngularRate();
    s.angular_rate[0] = w.x; s.angular_rate[1] = w.y; s.angular_rate[2] = w.z;

    s.sensor_mask = static_cast<uint8_t>(core_.getActiveMask() & 0xFF);
    s.timestamp = timestamp;

    return s;
}

}  // namespace sf
