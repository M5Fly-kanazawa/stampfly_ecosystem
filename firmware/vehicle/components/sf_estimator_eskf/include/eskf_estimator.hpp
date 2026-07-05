/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file eskf_estimator.hpp
 * @brief ESKF state estimator — IEstimator wrapping EskfCore
 *        ESKF状態推定器 — EskfCoreをラップするIEstimator
 *
 * @design requirements.md §4 — Component #2: replaceable estimation   [OK]
 * @design detailed_design.md §5 — IEstimator implementation           [OK]
 * @design detailed_design.md §5 — Sensor observation switch           [OK]
 */

#pragma once

#include "estimator.hpp"
#include "eskf_core.hpp"

namespace sf {

class EskfEstimator : public IEstimator {
public:
    void init();

    void predict(const ImuData& imu, float dt) override;
    void updateTof(const TofData& tof) override;
    void updateFlow(const FlowData& flow) override;
    void updateMag(const MagData& mag) override;
    void updateBaro(const BaroData& baro) override;
    StateEstimate getState() const override;
    void reset() override;
    void resetPositionVelocity() override;
    void holdPositionVelocity() override;
    void applyCalibration(const float gyro_bias[3], const float accel_bias[3]) override;
    void freezeBias() override;
    void unfreezeBias() override;
    void inflateCovariance(uint16_t state_mask) override;
    void reloadParams() override;
    void setMagReference(const float ned[3]) override;
    void setSensorEnabled(int group, bool enabled) override;

private:
    StateEstimate convertState(uint32_t timestamp) const;
    EskfCore core_;
    StateEstimate cached_state_ = {};
    // Last flow timestamp [us], kept as uint32 (NOT float): a float loses µs
    // resolution after ~17 min of uptime (24-bit mantissa), corrupting the flow
    // dt and thus the flow-velocity observation. Unsigned subtraction handles
    // the ~71.6 min uint32 wrap correctly. 0 = no sample yet.
    // 最終フロー時刻 [us]。float でなく uint32 で保持: float は稼働約17分で µs 分解能を
    // 失い（仮数24bit）、フロー dt＝速度観測が劣化する。符号なし減算は約71.6分の uint32
    // ラップも正しく扱える。0 = 未受信。
    uint32_t last_flow_time_ = 0;
};

}  // namespace sf
