/**
 * @file eskf_estimator.hpp
 * @brief ESKF state estimator implementation
 *        ESKF状態推定器実装
 *
 * Error-State Kalman Filter implementation of IEstimator.
 * Currently a stub — returns identity attitude and zero position/velocity.
 * Full ESKF implementation will be ported from vehicle firmware.
 *
 * IEstimatorのESKF実装。
 * 現在はスタブ — 単位姿勢と零位置/速度を返す。
 * 完全なESKF実装はvehicleファームから移植予定。
 *
 * @design requirements.md §4 — Component #2: replaceable estimation   [--]
 * @design detailed_design.md §5 — IEstimator implementation           [--]
 */

#pragma once

#include "estimator.hpp"

namespace sf {

class EskfEstimator : public IEstimator {
public:
    /// Initialize ESKF matrices and parameters
    /// ESKF行列とパラメータを初期化する
    void init();

    // IEstimator interface implementation
    // IEstimatorインターフェース実装

    void predict(const ImuData& imu, float dt) override;
    void updateTof(const TofData& tof) override;
    void updateFlow(const FlowData& flow) override;
    void updateMag(const MagData& mag) override;
    void updateBaro(const BaroData& baro) override;
    StateEstimate getState() const override;
    void reset() override;
    void resetPositionVelocity() override;

private:
    StateEstimate state_ = {};
};

}  // namespace sf
