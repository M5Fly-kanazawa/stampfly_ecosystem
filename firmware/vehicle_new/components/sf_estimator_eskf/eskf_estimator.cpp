/**
 * @file eskf_estimator.cpp
 * @brief ESKF estimator stub implementation
 *        ESKF推定器スタブ実装
 *
 * Minimal stub for pipeline integration testing.
 * Full ESKF will be ported from vehicle firmware.
 *
 * パイプライン結合テスト用の最小スタブ。
 * 完全なESKFはvehicleファームから移植予定。
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
    // Stub: store latest IMU timestamp
    // スタブ: 最新のIMUタイムスタンプを保存
    state_.timestamp = imu.timestamp;

    // TODO: Full ESKF prediction step
    // TODO: 完全なESKF予測ステップ
}

void EskfEstimator::updateTof(const TofData& tof)
{
    // TODO: ToF observation update
}

void EskfEstimator::updateFlow(const FlowData& flow)
{
    // TODO: Optical flow observation update
}

void EskfEstimator::updateMag(const MagData& mag)
{
    // TODO: Magnetometer observation update
}

void EskfEstimator::updateBaro(const BaroData& baro)
{
    // TODO: Barometer observation update
}

StateEstimate EskfEstimator::getState() const
{
    return state_;
}

void EskfEstimator::reset()
{
    state_ = {};
    // Identity quaternion [w,x,y,z]
    // 単位クォータニオン
    state_.attitude[0] = 1.0f;
    state_.attitude[1] = 0.0f;
    state_.attitude[2] = 0.0f;
    state_.attitude[3] = 0.0f;

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
