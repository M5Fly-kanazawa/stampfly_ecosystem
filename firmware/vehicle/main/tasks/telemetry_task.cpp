/**
 * @file telemetry_task.cpp
 * @brief テレメトリタスク (50Hz) - WebSocketブロードキャスト
 */

#include "tasks_common.hpp"

static const char* TAG = "TelemetryTask";

using namespace config;
using namespace globals;

void TelemetryTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetryTask started");

    auto& telemetry = stampfly::Telemetry::getInstance();
    auto& state = stampfly::StampFlyState::getInstance();

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    // DEBUG: ESP32側送信カウンタ
    static uint32_t esp32_send_counter = 0;

    while (true) {
        // Only broadcast when clients are connected
        if (telemetry.hasClients()) {
            stampfly::TelemetryWSPacket pkt = {};
            pkt.header = 0xAA;
            pkt.packet_type = 0x20;  // v2 extended packet
            pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            // Get attitude from state (ESKF estimated)
            state.getAttitudeEuler(pkt.roll, pkt.pitch, pkt.yaw);

            // Get position and velocity (ESKF estimated)
            auto pos = state.getPosition();
            pkt.pos_x = pos.x;
            pkt.pos_y = pos.y;
            pkt.pos_z = pos.z;

            auto vel = state.getVelocity();
            pkt.vel_x = vel.x;
            pkt.vel_y = vel.y;
            pkt.vel_z = vel.z;

            // Get bias-corrected IMU data
            stampfly::Vec3 accel, gyro;
            state.getIMUCorrected(accel, gyro);
            pkt.gyro_x = gyro.x;
            pkt.gyro_y = gyro.y;
            pkt.gyro_z = gyro.z;
            pkt.accel_x = accel.x;
            pkt.accel_y = accel.y;
            pkt.accel_z = accel.z;

            // Get control inputs (normalized)
            state.getControlInput(pkt.ctrl_throttle, pkt.ctrl_roll,
                                  pkt.ctrl_pitch, pkt.ctrl_yaw);

            // Get magnetometer data
            stampfly::Vec3 mag;
            state.getMagData(mag);
            pkt.mag_x = mag.x;
            pkt.mag_y = mag.y;
            pkt.mag_z = mag.z;

            // Get battery voltage
            float voltage, current;
            state.getPowerData(voltage, current);
            pkt.voltage = voltage;

            // Get flight state
            pkt.flight_state = static_cast<uint8_t>(state.getFlightState());

            // Sensor status (placeholder - all OK for now)
            pkt.sensor_status = 0x1F;  // All sensors OK

            // Heartbeat counter
            esp32_send_counter++;
            pkt.heartbeat = esp32_send_counter;

            // Calculate checksum (XOR of all bytes before checksum field)
            uint8_t checksum = 0;
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
            // checksum is at offset 92 (sizeof(pkt) - 4 for padding - 1 for checksum)
            constexpr size_t checksum_offset = offsetof(stampfly::TelemetryWSPacket, checksum);
            for (size_t i = 0; i < checksum_offset; i++) {
                checksum ^= data[i];
            }
            pkt.checksum = checksum;

            // Broadcast to all connected clients
            telemetry.broadcast(&pkt, sizeof(pkt));
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
