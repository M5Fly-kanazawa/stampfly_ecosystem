/**
 * @file tof_task.cpp
 * @brief ToFタスク (30Hz) - VL53L3CX読み取り（底面・前方）
 */

#include "tasks_common.hpp"

static const char* TAG = "ToFTask";

using namespace config;
using namespace globals;

void ToFTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ToFTask started, bottom_init=%d, front_init=%d",
             g_tof_bottom.isInitialized(), g_tof_front.isInitialized());

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(static_cast<TickType_t>(TOF_DT * 1000.0f));

    auto& state = stampfly::StampFlyState::getInstance();

    // Error counters for sensor disable on repeated failures
    const int MAX_ERRORS = 10;
    // ヘルスモニター設定 (5連続成功/10連続失敗)
    g_health.tof.setThresholds(5, 10);
    // 距離の急激な変化検出用
    float tof_last_valid_distance = 0.0f;
    constexpr float TOF_MAX_CHANGE_RATE = 2.0f;  // 最大変化率 [m/s]
    const float TOF_MAX_CHANGE_PER_CYCLE = TOF_MAX_CHANGE_RATE * TOF_DT;  // TOF_DT秒あたりの最大変化
    int bottom_errors = 0;
    int front_errors = 0;
    bool bottom_disabled = false;
    bool front_disabled = false;

    static int log_count = 0;

    while (true) {
        // Bottom ToF
        if (g_tof_bottom.isInitialized() && !bottom_disabled) {
            // Check if data is ready
            bool data_ready = false;
            g_tof_bottom.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                esp_err_t ret = g_tof_bottom.getDistance(distance_mm, status);
                if (ret == ESP_OK) {
                    bottom_errors = 0;  // Reset on success

                    // Only update if valid measurement (status 0-4)
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;

                        // 距離の急激な変化をチェック
                        bool distance_jump_detected = false;
                        if (tof_last_valid_distance > 0.01f) {
                            float change = std::abs(distance_m - tof_last_valid_distance);
                            if (change > TOF_MAX_CHANGE_PER_CYCLE) {
                                distance_jump_detected = true;
                                ESP_LOGW(TAG, "ToF distance jump: %.3f -> %.3f (change=%.3f)",
                                         tof_last_valid_distance, distance_m, change);
                            }
                        }

                        state.updateToF(stampfly::ToFPosition::BOTTOM, distance_m, status);

                        if (!distance_jump_detected) {
                            g_health.tof.recordSuccess();
                            g_tof_task_healthy = g_health.tof.isHealthy();

                            // リングバッファに追加（常時更新）
                            g_tof_bottom_buffer[g_tof_bottom_buffer_index] = distance_m;
                            g_tof_bottom_buffer_index = (g_tof_bottom_buffer_index + 1) % REF_BUFFER_SIZE;
                            if (g_tof_bottom_buffer_count < REF_BUFFER_SIZE) {
                                g_tof_bottom_buffer_count++;
                            }
                            g_tof_bottom_data_ready = true;

                            // Fallback to simple altitude estimator (センサーフュージョン未使用時)
                            if (!g_fusion.isInitialized() && g_altitude_est.isInitialized() && g_attitude_est.isInitialized()) {
                                auto att = g_attitude_est.getState();
                                g_altitude_est.updateToF(distance_m, att.pitch, att.roll);
                            }
                        } else {
                            // 距離ジャンプ検出
                            g_health.tof.recordFailure();
                            g_tof_task_healthy = g_health.tof.isHealthy();
                        }

                        // 有効距離を記録（ジャンプ後も更新して追従可能に）
                        tof_last_valid_distance = distance_m;
                    } else {
                        // ステータス異常
                        g_health.tof.recordFailure();
                        g_tof_task_healthy = g_health.tof.isHealthy();
                    }

                    // Debug log every 300 readings (~10 seconds at 30Hz)
                    if (++log_count >= 300) {
                        ESP_LOGI(TAG, "ToFTask alive: bottom=%dmm status=%d, stack_free=%u",
                                 distance_mm, status, (unsigned)uxTaskGetStackHighWaterMark(nullptr));
                        log_count = 0;
                    }

                    // Clear interrupt and start next measurement
                    g_tof_bottom.clearInterruptAndStartMeasurement();
                } else {
                    g_health.tof.recordFailure();
                    g_tof_task_healthy = g_health.tof.isHealthy();
                    if (++bottom_errors >= MAX_ERRORS) {
                        ESP_LOGW(TAG, "Bottom ToF disabled: err=%s", esp_err_to_name(ret));
                        bottom_disabled = true;
                    }
                }
            }
        }

        // Front ToF
        if (g_tof_front.isInitialized() && !front_disabled) {
            bool data_ready = false;
            g_tof_front.isDataReady(data_ready);

            if (data_ready) {
                uint16_t distance_mm;
                uint8_t status;
                if (g_tof_front.getDistance(distance_mm, status) == ESP_OK) {
                    front_errors = 0;  // Reset on success
                    if (status <= 4) {
                        float distance_m = distance_mm * 0.001f;
                        state.updateToF(stampfly::ToFPosition::FRONT, distance_m, status);

                        // リングバッファに追加（常時更新）
                        g_tof_front_buffer[g_tof_front_buffer_index] = distance_m;
                        g_tof_front_buffer_index = (g_tof_front_buffer_index + 1) % REF_BUFFER_SIZE;
                        if (g_tof_front_buffer_count < REF_BUFFER_SIZE) {
                            g_tof_front_buffer_count++;
                        }
                        g_tof_front_data_ready = true;
                    }
                    g_tof_front.clearInterruptAndStartMeasurement();
                } else {
                    if (++front_errors >= MAX_ERRORS) {
                        ESP_LOGW(TAG, "Front ToF disabled due to repeated errors");
                        front_disabled = true;
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
