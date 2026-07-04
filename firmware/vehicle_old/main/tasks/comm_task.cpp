/**
 * @file comm_task.cpp
 * @brief 通信タスク (50Hz) - ESP-NOWコントローラー通信処理
 */

#include "tasks_common.hpp"

static const char* TAG = "CommTask";

using namespace config;
using namespace globals;

void CommTask(void* pvParameters)
{
    ESP_LOGI(TAG, "CommTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    auto& state = stampfly::StampFlyState::getInstance();

    while (true) {
        if (g_comm.isInitialized()) {
            // Check connection timeout
            g_comm.tick();

            // Send telemetry if connected
            if (g_comm.isConnected()) {
                stampfly::TelemetryPacket telem = {};

                // Fill telemetry data
                telem.battery_mv = static_cast<uint16_t>(state.getVoltage() * 1000);
                telem.altitude_cm = static_cast<int16_t>(state.getAltitude() * 100);

                stampfly::StateVector3 vel = state.getVelocity();
                telem.velocity_x = static_cast<int16_t>(vel.x * 1000);
                telem.velocity_y = static_cast<int16_t>(vel.y * 1000);
                telem.velocity_z = static_cast<int16_t>(vel.z * 1000);

                stampfly::StateVector3 att = state.getAttitude();
                telem.roll_deg10 = static_cast<int16_t>(att.x * 180.0f / M_PI * 10);
                telem.pitch_deg10 = static_cast<int16_t>(att.y * 180.0f / M_PI * 10);
                telem.yaw_deg10 = static_cast<int16_t>(att.z * 180.0f / M_PI * 10);

                telem.state = static_cast<uint8_t>(state.getFlightState());

                // Set warning flags
                telem.flags = 0;
                if (g_power.isLowBattery()) {
                    telem.flags |= stampfly::TELEM_FLAG_LOW_BATTERY;
                }
                if (state.getErrorCode() != stampfly::ErrorCode::NONE) {
                    telem.flags |= stampfly::TELEM_FLAG_SENSOR_ERROR;
                }
                if (state.getFlightState() == stampfly::FlightState::CALIBRATING) {
                    telem.flags |= stampfly::TELEM_FLAG_CALIBRATING;
                }

                g_comm.sendTelemetry(telem);
            }

            // Update connection state
            if (!g_comm.isConnected() && state.getPairingState() == stampfly::PairingState::PAIRED) {
                state.setPairingState(stampfly::PairingState::PAIRED);  // Keep paired but disconnected
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
