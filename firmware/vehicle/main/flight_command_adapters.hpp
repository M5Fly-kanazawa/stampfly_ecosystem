/**
 * @file flight_command_adapters.hpp
 * @brief Concrete DI adapters for FlightCommandService
 *        FlightCommandService 用の具象 DI アダプタ
 *
 * These adapters bridge FlightCommandService's abstract interfaces
 * to vehicle/main's global objects (g_fusion, g_landing_handler, etc.).
 * FlightCommandService の抽象インターフェースと vehicle/main の
 * グローバルオブジェクトを橋渡しする。
 */

#pragma once

#include "flight_command_deps.hpp"
#include "sensor_fusion.hpp"
#include "landing_handler.hpp"
#include "position_controller.hpp"

namespace stampfly {

/**
 * @brief Adapts LandingHandler to IFlightReadiness
 *        LandingHandler を IFlightReadiness に適合
 */
class VehicleFlightReadiness : public IFlightReadiness {
public:
    explicit VehicleFlightReadiness(LandingHandler& handler) : handler_(handler) {}

    bool isLanded() const override { return handler_.isLanded(); }
    bool canArm() const override { return handler_.canArm(); }

private:
    LandingHandler& handler_;
};

/**
 * @brief Adapts SensorFusion + ToF buffer to IStateEstimator
 *        SensorFusion + ToF バッファを IStateEstimator に適合
 */
class VehicleStateEstimator : public IStateEstimator {
public:
    VehicleStateEstimator(sf::SensorFusion& fusion,
                          float* tof_buffer, int* tof_index, int* tof_count, int buffer_size)
        : fusion_(fusion)
        , tof_buffer_(tof_buffer)
        , tof_index_(tof_index)
        , tof_count_(tof_count)
        , buffer_size_(buffer_size) {}

    float getAltitude() const override {
        // NED frame: negative Z = up, return positive altitude
        // NED フレーム: Z が負 = 上方向、正の高度を返す
        auto state = fusion_.getState();
        return -state.position.z;
    }

    float getYaw() const override {
        auto state = fusion_.getState();
        return state.yaw;
    }

    void getPositionXY(float& x, float& y) const override {
        auto state = fusion_.getState();
        x = state.position.x;
        y = state.position.y;
    }

    bool getToFAltitude(float& altitude) const override {
        if (*tof_count_ > 0) {
            int latest_idx = (*tof_index_ - 1 + buffer_size_) % buffer_size_;
            altitude = tof_buffer_[latest_idx];
            return true;
        }
        return false;
    }

private:
    sf::SensorFusion& fusion_;
    float* tof_buffer_;
    int* tof_index_;
    int* tof_count_;
    int buffer_size_;
};

/**
 * @brief Adapts PositionController to IPositionControl
 *        PositionController を IPositionControl に適合
 */
class VehiclePositionControl : public IPositionControl {
public:
    explicit VehiclePositionControl(PositionController& controller) : controller_(controller) {}

    void capturePosition(float x, float y) override {
        controller_.capturePosition(x, y);
    }

    void setPositionSetpoint(float x, float y) override {
        controller_.pos_setpoint_x = x;
        controller_.pos_setpoint_y = y;
    }

private:
    PositionController& controller_;
};

}  // namespace stampfly
