/**
 * @file baro_task.cpp
 * @brief 気圧タスク (50Hz) - BMP280読み取り
 */

#include "tasks_common.hpp"

static const char* TAG = "BaroTask";

using namespace config;
using namespace globals;

void BaroTask(void* pvParameters)
{
    ESP_LOGI(TAG, "BaroTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(static_cast<TickType_t>(BARO_DT * 1000.0f));

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスモニター設定
    g_health.baro.setThresholds(5, 10);  // 5連続成功/10連続失敗

    while (true) {
        if (g_baro.isInitialized()) {
            stampfly::BaroData baro;
            if (g_baro.read(baro) == ESP_OK) {
                g_health.baro.recordSuccess();
                g_baro_task_healthy = g_health.baro.isHealthy();

                // Use altitude from read() directly (already calculated)
                state.updateBaro(baro.pressure_pa, baro.temperature_c, baro.altitude_m);

                // 初回測定で基準高度を設定
                if (!g_baro_reference_set) {
                    g_baro_reference_altitude = baro.altitude_m;
                    g_baro_reference_set = true;
                    state.setBaroReferenceAltitude(g_baro_reference_altitude);
                    ESP_LOGI(TAG, "Baro reference set: %.3f m", g_baro_reference_altitude);
                }

                // リングバッファに相対高度を追加（安定判定用）
                float relative_alt = g_baro_reference_set ?
                    (baro.altitude_m - g_baro_reference_altitude) : 0.0f;
                g_baro_buffer[g_baro_buffer_index] = relative_alt;
                g_baro_buffer_index = (g_baro_buffer_index + 1) % REF_BUFFER_SIZE;
                if (g_baro_buffer_count < REF_BUFFER_SIZE) {
                    g_baro_buffer_count++;
                }
                g_baro_data_ready = true;

                // Fallback to simple altitude estimator (センサーフュージョン未使用時)
                if (!g_fusion.isInitialized() && g_altitude_est.isInitialized()) {
                    g_altitude_est.updateBaro(baro.altitude_m);
                }
            } else {
                g_health.baro.recordFailure();
                g_baro_task_healthy = g_health.baro.isHealthy();
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
