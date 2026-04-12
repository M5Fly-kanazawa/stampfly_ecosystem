/**
 * @file eskf_estimator.hpp
 * @brief ESKF state estimator — IEstimator implementation
 *        ESKF状態推定器 — IEstimator実装
 *
 * Currently a stub. Full ESKF will be implemented from scratch
 * based on mathematical understanding, not copied from old code.
 *
 * 現在はスタブ。完全なESKFは旧コードのコピーではなく、
 * 数学的理解に基づいて新規実装予定。
 *
 * @design requirements.md §4 — Component #2: replaceable estimation   [--]
 * @design detailed_design.md §5 — IEstimator implementation           [--]
 * @design detailed_design.md §5 — Sensor observation switch           [--]
 */

#pragma once

#include "estimator.hpp"

namespace sf {

class EskfEstimator : public IEstimator {
public:
    /// Initialize ESKF with parameters
    /// パラメータでESKFを初期化する
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
