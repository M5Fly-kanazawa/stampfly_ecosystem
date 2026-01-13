/**
 * @file telemetry_task.cpp
 * @brief テレメトリタスク - WebSocketブロードキャスト
 *
 * Rate is configurable via CLI 'fftmode' command
 * - Normal mode (50Hz): Full 116-byte packet
 * - FFT mode (>50Hz): Lightweight 32-byte packet (gyro + accel only)
 *
 * レートはCLI 'fftmode'コマンドで設定可能
 * - 通常モード (50Hz): フル116バイトパケット
 * - FFTモード (>50Hz): 軽量32バイトパケット (ジャイロ+加速度のみ)
 */

#include "tasks_common.hpp"

static const char* TAG = "TelemetryTask";

// Telemetry rate (Hz) - set by CLI fftmode command, persisted in NVS
// テレメトリレート - CLIのfftmodeコマンドで設定、NVSに保存
extern uint32_t g_telemetry_rate_hz;

using namespace config;
using namespace globals;

void TelemetryTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetryTask started");

    auto& telemetry = stampfly::Telemetry::getInstance();
    auto& state = stampfly::StampFlyState::getInstance();

    TickType_t last_wake_time = xTaskGetTickCount();

    // Calculate period from rate (checked at startup, persisted via NVS)
    // レートから周期を計算（起動時に読み込み、NVS経由で永続化）
    uint32_t period_ms = 1000 / g_telemetry_rate_hz;
    TickType_t period = pdMS_TO_TICKS(period_ms);

    // Determine packet mode: FFT mode uses lightweight packet
    // FFTモードは軽量パケットを使用
    bool fft_mode = (g_telemetry_rate_hz > 50);
    ESP_LOGI(TAG, "Telemetry rate: %lu Hz (%lu ms), mode: %s",
             (unsigned long)g_telemetry_rate_hz, (unsigned long)period_ms,
             fft_mode ? "FFT (32B)" : "Normal (116B)");

    // DEBUG: ESP32側送信カウンタ
    static uint32_t esp32_send_counter = 0;

    while (true) {
        // Only broadcast when clients are connected
        if (telemetry.hasClients()) {
            esp32_send_counter++;

            if (fft_mode) {
                // === FFT Mode: Lightweight packet (32 bytes) ===
                // FFTモード: 軽量パケット (32バイト)
                stampfly::TelemetryFFTPacket pkt = {};
                pkt.header = 0xBB;          // Different header for FFT packet
                pkt.packet_type = 0x30;     // FFT packet type
                pkt.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // Get bias-corrected IMU data
                stampfly::Vec3 accel, gyro;
                state.getIMUCorrected(accel, gyro);
                pkt.gyro_x = gyro.x;
                pkt.gyro_y = gyro.y;
                pkt.gyro_z = gyro.z;
                pkt.accel_x = accel.x;
                pkt.accel_y = accel.y;
                pkt.accel_z = accel.z;

                // Calculate checksum
                uint8_t checksum = 0;
                const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
                constexpr size_t checksum_offset = offsetof(stampfly::TelemetryFFTPacket, checksum);
                for (size_t i = 0; i < checksum_offset; i++) {
                    checksum ^= data[i];
                }
                pkt.checksum = checksum;

                telemetry.broadcast(&pkt, sizeof(pkt));

            } else {
                // === Normal Mode: Full packet (116 bytes) ===
                // 通常モード: フルパケット (116バイト)
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

                // Get ToF data (bottom and front)
                float tof_bottom, tof_front;
                state.getToFData(tof_bottom, tof_front);
                pkt.tof_bottom = tof_bottom;
                pkt.tof_front = tof_front;

                // Get flight state
                pkt.flight_state = static_cast<uint8_t>(state.getFlightState());

                // Sensor status (placeholder - all OK for now)
                pkt.sensor_status = 0x1F;  // All sensors OK

                // Heartbeat counter
                pkt.heartbeat = esp32_send_counter;

                // Calculate checksum
                uint8_t checksum = 0;
                const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
                constexpr size_t checksum_offset = offsetof(stampfly::TelemetryWSPacket, checksum);
                for (size_t i = 0; i < checksum_offset; i++) {
                    checksum ^= data[i];
                }
                pkt.checksum = checksum;

                telemetry.broadcast(&pkt, sizeof(pkt));
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
