/**
 * @file eskf_estimator.hpp
 * @brief ESKF state estimator — IEstimator wrapping EskfCore
 *        ESKF状態推定器 — EskfCoreをラップするIEstimator
 *
 * @design requirements.md §4 — Component #2: replaceable estimation   [--]
 * @design detailed_design.md §5 — IEstimator implementation           [--]
 * @design detailed_design.md §5 — Sensor observation switch           [--]
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

private:
    StateEstimate convertState(uint32_t timestamp) const;
    EskfCore core_;
    StateEstimate cached_state_ = {};
    float last_flow_time_ = 0;
};

}  // namespace sf
