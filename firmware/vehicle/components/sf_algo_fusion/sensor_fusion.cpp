/**
 * @file sensor_fusion.cpp
 * @brief センサーフュージョン実装
 */

#include "sensor_fusion.hpp"
#include <cmath>

namespace sf {

bool SensorFusion::init(const stampfly::ESKF::Config& config,
                        const SensorEnables& enables,
                        float max_position,
                        float max_velocity) {
    max_position_ = max_position;
    max_velocity_ = max_velocity;
    enables_ = enables;

    // ESKF初期化（設定は main/config.hpp から構築されて渡される）
    if (eskf_.init(config) != ESP_OK) {
        return false;
    }

    initialized_ = true;
    diverged_ = false;
    return true;
}

bool SensorFusion::predictIMU(const stampfly::math::Vector3& accel_body,
                               const stampfly::math::Vector3& gyro_body,
                               float dt,
                               bool skip_position) {
    if (!initialized_) {
        return false;
    }

    // 入力値の事前チェック
    bool input_valid = std::isfinite(accel_body.x) && std::isfinite(accel_body.y) &&
                       std::isfinite(accel_body.z) && std::isfinite(gyro_body.x) &&
                       std::isfinite(gyro_body.y) && std::isfinite(gyro_body.z);

    if (!input_valid) {
        return false;
    }

    // ESKF予測ステップ（接地中はskip_position=trueで位置更新をスキップ）
    eskf_.predict(accel_body, gyro_body, dt, skip_position);

    // 加速度計による姿勢補正
    eskf_.updateAccelAttitude(accel_body);

    // 発散チェック
    auto state = eskf_.getState();
    diverged_ = checkDivergence(state);

    return !diverged_;
}

void SensorFusion::updateOpticalFlow(int16_t dx, int16_t dy, uint8_t squal,
                                      float distance, float dt,
                                      float gyro_x, float gyro_y) {
    if (!initialized_ || diverged_ || !enables_.optical_flow) {
        return;
    }

    // 品質チェック
    if (squal < enables_.flow_squal_min) {
        return;
    }

    // 距離の有効範囲チェック
    if (distance < enables_.flow_distance_min || distance > enables_.flow_distance_max) {
        return;
    }

    // ESKF更新
    eskf_.updateFlowRaw(dx, dy, distance, dt, gyro_x, gyro_y);
}

void SensorFusion::updateBarometer(float relative_altitude) {
    if (!initialized_ || diverged_ || !enables_.barometer) {
        return;
    }

    eskf_.updateBaro(relative_altitude);
}

void SensorFusion::updateToF(float distance) {
    if (!initialized_ || diverged_ || !enables_.tof) {
        return;
    }

    // 距離の有効範囲チェック
    if (distance < enables_.tof_distance_min || distance > enables_.tof_distance_max) {
        return;
    }

    eskf_.updateToF(distance);
}

void SensorFusion::updateMagnetometer(const stampfly::math::Vector3& mag_body) {
    if (!initialized_ || diverged_ || !enables_.magnetometer) {
        return;
    }

    eskf_.updateMag(mag_body);
}

SensorFusion::State SensorFusion::getState() const {
    State result;

    if (!initialized_) {
        return result;
    }

    auto eskf_state = eskf_.getState();
    result.roll = eskf_state.roll;
    result.pitch = eskf_state.pitch;
    result.yaw = eskf_state.yaw;
    result.position = eskf_state.position;
    result.velocity = eskf_state.velocity;
    result.gyro_bias = eskf_state.gyro_bias;
    result.accel_bias = eskf_state.accel_bias;

    return result;
}

void SensorFusion::reset() {
    if (!initialized_) {
        return;
    }

    eskf_.reset();
    diverged_ = false;
}

void SensorFusion::resetPositionVelocity() {
    if (!initialized_) {
        return;
    }

    eskf_.resetPositionVelocity();
    diverged_ = false;  // 発散状態もクリア
}

void SensorFusion::holdPositionVelocity() {
    if (!initialized_) {
        return;
    }

    eskf_.holdPositionVelocity();
}

void SensorFusion::resetForLanding() {
    if (!initialized_) {
        return;
    }

    eskf_.resetForLanding();
    diverged_ = false;
}

void SensorFusion::setGyroBias(const stampfly::math::Vector3& bias) {
    if (!initialized_) {
        return;
    }

    eskf_.setGyroBias(bias);
}

void SensorFusion::setMagReference(const stampfly::math::Vector3& ref) {
    if (!initialized_) {
        return;
    }

    eskf_.setMagReference(ref);
}

void SensorFusion::initializeAttitude(const stampfly::math::Vector3& accel,
                                       const stampfly::math::Vector3& mag) {
    if (!initialized_) {
        return;
    }

    eskf_.initializeAttitude(accel, mag);
}

bool SensorFusion::checkDivergence(const stampfly::ESKF::State& state) {
    // 姿勢の有効性チェック
    if (!std::isfinite(state.roll) || !std::isfinite(state.pitch)) {
        return true;
    }

    // 位置の発散検出
    if (std::abs(state.position.x) > max_position_ ||
        std::abs(state.position.y) > max_position_ ||
        std::abs(state.position.z) > max_position_) {
        return true;
    }

    // 速度の発散検出
    if (std::abs(state.velocity.x) > max_velocity_ ||
        std::abs(state.velocity.y) > max_velocity_ ||
        std::abs(state.velocity.z) > max_velocity_) {
        return true;
    }

    return false;
}

} // namespace sf
