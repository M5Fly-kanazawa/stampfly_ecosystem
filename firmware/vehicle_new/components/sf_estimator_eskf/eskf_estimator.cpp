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
 * @design detailed_design.md §5 — IEstimator                         [--]
 * @design detailed_design.md §5 — Sensor observation switch           [--]
 */

#include "eskf_estimator.hpp"
#include "params.hpp"
#include "esp_log.h"

static const char* TAG = "ESKF";

namespace sf {

void EskfEstimator::init()
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

    core_.init(cfg);
    cached_state_ = {};
    cached_state_.attitude[0] = 1.0f;

    ESP_LOGI(TAG, "ESKF initialized (tof=%d flow=%d baro=%d mag=%d accel_comp=%d a=%.2f b=%.3f)",
             cfg.use_tof, cfg.use_flow, cfg.use_baro, cfg.use_mag,
             cfg.accel_comp_enable, cfg.accel_comp_alpha, cfg.accel_comp_beta);
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
    float dt = (last_flow_time_ > 0)
        ? (flow.timestamp - last_flow_time_) * 1e-6f
        : 0.01f;
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
