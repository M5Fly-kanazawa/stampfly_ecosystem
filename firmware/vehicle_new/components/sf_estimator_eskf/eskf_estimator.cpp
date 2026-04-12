/**
 * @file eskf_estimator.cpp
 * @brief ESKF estimator stub — to be implemented from mathematical foundations
 *        ESKFスタブ — 数学的基礎から新規実装予定
 *
 * @design detailed_design.md §5 — IEstimator                         [--]
 */

#include "eskf_estimator.hpp"
#include "esp_log.h"

static const char* TAG = "ESKF";

namespace sf {

void EskfEstimator::init()
{
    reset();
    ESP_LOGI(TAG, "ESKF estimator initialized (stub)");
}

void EskfEstimator::predict(const ImuData& imu, float dt)
{
    state_.timestamp = imu.timestamp;
    // TODO: Implement ESKF prediction from mathematical foundations
    // TODO: 数学的基礎からESKF予測を実装
}

void EskfEstimator::updateTof(const TofData& tof)   { /* TODO */ }
void EskfEstimator::updateFlow(const FlowData& flow) { /* TODO */ }
void EskfEstimator::updateMag(const MagData& mag)    { /* TODO */ }
void EskfEstimator::updateBaro(const BaroData& baro) { /* TODO */ }

StateEstimate EskfEstimator::getState() const { return state_; }

void EskfEstimator::reset()
{
    state_ = {};
    state_.attitude[0] = 1.0f;  // Identity quaternion w
    ESP_LOGI(TAG, "ESKF state reset");
}

void EskfEstimator::resetPositionVelocity()
{
    for (int i = 0; i < 3; i++) {
        state_.position[i] = 0.0f;
        state_.velocity[i] = 0.0f;
    }
    ESP_LOGI(TAG, "ESKF position/velocity reset");
}

}  // namespace sf
