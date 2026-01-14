/**
 * @file telemetry_task.cpp
 * @brief テレメトリタスク - WebSocketブロードキャスト
 *
 * Rate is configurable via CLI 'fftmode' command
 * - Normal mode (50Hz): Full 116-byte packet, uses vTaskDelayUntil
 * - FFT mode (>50Hz): Batch 120-byte packet (4 samples × 28B), synced with IMU
 *
 * レートはCLI 'fftmode'コマンドで設定可能
 * - 通常モード (50Hz): フル116バイトパケット、vTaskDelayUntil使用
 * - FFTモード (>50Hz): バッチ120バイトパケット、IMU同期
 *
 * FFT mode uses semaphore synchronization with IMU task to ensure
 * each sample has unique data. This overcomes per-frame overhead
 * (~155 fps limit) by batching 4 samples per frame.
 *
 * FFTモードはセマフォでIMUタスクと同期し、各サンプルが
 * ユニークなデータを持つことを保証。4サンプルをバッチ化して
 * フレームオーバーヘッド制限を回避。
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

    // Determine mode and calculate periods
    // FFTモードは高速サンプリング + バッチ送信 (IMU同期)
    bool fft_mode = (g_telemetry_rate_hz > 50);

    // Sample period for normal mode (FFT mode uses semaphore sync)
    // 通常モード用サンプリング周期（FFTモードはセマフォ同期）
    uint32_t sample_period_ms = 1000 / g_telemetry_rate_hz;
    TickType_t sample_period = pdMS_TO_TICKS(sample_period_ms);

    if (fft_mode) {
        ESP_LOGI(TAG, "FFT mode: 400Hz IMU-synced sampling, batch of %d, frame rate ~100Hz",
                 stampfly::FFT_BATCH_SIZE);
    } else {
        ESP_LOGI(TAG, "Normal mode: %lu Hz, 116B full packet",
                 (unsigned long)g_telemetry_rate_hz);
    }

    // Batch buffer for FFT mode
    // FFTモード用バッチバッファ
    stampfly::TelemetryFFTBatchPacket batch_pkt = {};
    int batch_index = 0;

    // Read index for ring buffer (FFT mode)
    // リングバッファ読み取りインデックス（FFTモード用）
    int telemetry_read_index = 0;

    // DEBUG: ESP32送信カウンタ
    static uint32_t esp32_send_counter = 0;

    // Wait for IMU to start populating the buffer
    // IMUがバッファにデータを入れ始めるのを待つ
    if (fft_mode) {
        vTaskDelay(pdMS_TO_TICKS(100));
        telemetry_read_index = g_accel_buffer_index;  // Start from current position
    }

    while (true) {
        if (fft_mode) {
            // === FFT Mode: Collect sample into batch (IMU-synced) ===
            // FFTモード: IMU同期でサンプルをバッチに蓄積

            // Wait for IMU update (400Hz, synchronized with IMU task)
            // IMU更新を待機（400Hz、IMUタスクと同期）
            if (xSemaphoreTake(g_telemetry_imu_semaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
                // Timeout - IMU task might be stuck, skip this iteration
                continue;
            }

            // Read from ring buffer at current read index
            // リングバッファから現在の読み取りインデックスで読む
            const auto& accel = g_accel_buffer[telemetry_read_index];
            const auto& gyro = g_gyro_buffer[telemetry_read_index];

            // Get bias-corrected IMU data (what control loop sees)
            // バイアス補正済みIMUデータ（制御ループが見る値）
            stampfly::Vec3 accel_corrected, gyro_corrected;
            state.getIMUCorrected(accel_corrected, gyro_corrected);

            // Get controller input from state
            // コントローラ入力をstateから取得
            float ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw;
            state.getControlInput(ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw);

            batch_pkt.samples[batch_index].timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            // Raw gyro (LPF only, no bias correction)
            batch_pkt.samples[batch_index].gyro_x = gyro.x;
            batch_pkt.samples[batch_index].gyro_y = gyro.y;
            batch_pkt.samples[batch_index].gyro_z = gyro.z;
            // Raw accel (LPF only, no bias correction)
            batch_pkt.samples[batch_index].accel_x = accel.x;
            batch_pkt.samples[batch_index].accel_y = accel.y;
            batch_pkt.samples[batch_index].accel_z = accel.z;
            // Bias-corrected gyro (what control loop uses)
            batch_pkt.samples[batch_index].gyro_corrected_x = gyro_corrected.x;
            batch_pkt.samples[batch_index].gyro_corrected_y = gyro_corrected.y;
            batch_pkt.samples[batch_index].gyro_corrected_z = gyro_corrected.z;
            // Controller inputs
            batch_pkt.samples[batch_index].ctrl_throttle = ctrl_throttle;
            batch_pkt.samples[batch_index].ctrl_roll = ctrl_roll;
            batch_pkt.samples[batch_index].ctrl_pitch = ctrl_pitch;
            batch_pkt.samples[batch_index].ctrl_yaw = ctrl_yaw;

            // Advance read index (ring buffer wrap)
            telemetry_read_index = (telemetry_read_index + 1) % REF_BUFFER_SIZE;

            batch_index++;

            // Send when batch is full
            // バッチが満杯になったら送信
            if (batch_index >= stampfly::FFT_BATCH_SIZE) {
                if (telemetry.hasClients()) {
                    // Fill header
                    batch_pkt.header = 0xBC;
                    batch_pkt.packet_type = 0x31;
                    batch_pkt.sample_count = stampfly::FFT_BATCH_SIZE;
                    batch_pkt.reserved = 0;

                    // Calculate checksum (XOR of bytes before checksum)
                    uint8_t checksum = 0;
                    const uint8_t* data = reinterpret_cast<const uint8_t*>(&batch_pkt);
                    constexpr size_t checksum_offset = offsetof(stampfly::TelemetryFFTBatchPacket, checksum);
                    for (size_t i = 0; i < checksum_offset; i++) {
                        checksum ^= data[i];
                    }
                    batch_pkt.checksum = checksum;

                    telemetry.broadcast(&batch_pkt, sizeof(batch_pkt));
                    esp32_send_counter++;
                }

                // Reset batch
                batch_index = 0;
            }

            // No vTaskDelayUntil here - timing is controlled by IMU semaphore
            // ここでは vTaskDelayUntil を使わない - タイミングはIMUセマフォで制御

        } else {
            // === Normal Mode: Full packet (116 bytes) ===
            // 通常モード: フルパケット (116バイト)

            if (telemetry.hasClients()) {
                esp32_send_counter++;

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

            // Normal mode uses periodic timing
            // 通常モードは周期的タイミングを使用
            vTaskDelayUntil(&last_wake_time, sample_period);
        }
    }
}
