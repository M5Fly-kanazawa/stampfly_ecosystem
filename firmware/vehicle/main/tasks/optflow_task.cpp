/**
 * @file optflow_task.cpp
 * @brief オプティカルフロータスク (100Hz) - PMW3901読み取り
 */

#include "tasks_common.hpp"

static const char* TAG = "OptFlowTask";

using namespace config;
using namespace globals;

void OptFlowTask(void* pvParameters)
{
    ESP_LOGI(TAG, "OptFlowTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(static_cast<TickType_t>(OPTFLOW_DT * 1000.0f));

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスモニター設定（10連続で状態変化）
    g_health.optflow.setThresholds(10, 10);

    static uint32_t optflow_loop_counter = 0;
    static uint32_t last_logged_loop = 0;

    while (true) {
        g_optflow_checkpoint = 0;  // ループ開始
        optflow_loop_counter++;
        g_optflow_last_loop = optflow_loop_counter;

        // 10秒ごと（1000回 @ 100Hz）に生存確認
        if (optflow_loop_counter % 1000 == 0) {
            // 前回のログから進行しているかチェック
            if (optflow_loop_counter == last_logged_loop) {
                ESP_LOGW(TAG, "OptFlowTask STUCK at checkpoint=%u, loop=%lu",
                         g_optflow_checkpoint, optflow_loop_counter);
            } else {
                ESP_LOGI(TAG, "OptFlowTask alive: loop=%lu, stack_free=%u",
                         optflow_loop_counter, (unsigned)uxTaskGetStackHighWaterMark(nullptr));
            }
            last_logged_loop = optflow_loop_counter;
        }

        g_optflow_checkpoint = 1;  // readMotionBurst前

        if (g_optflow != nullptr) {
            try {
                auto burst = g_optflow->readMotionBurst();
                g_optflow_checkpoint = 2;  // readMotionBurst成功
                // Check quality
                if (stampfly::OutlierDetector::isFlowValid(burst.squal)) {
                    g_health.optflow.recordSuccess();
                    g_optflow_task_healthy = g_health.optflow.isHealthy();

                    // PMW3901座標系 → 中間座標系 変換
                    int16_t flow_body_x = -burst.delta_y;
                    int16_t flow_body_y =  burst.delta_x;

                    state.updateOpticalFlow(flow_body_x, flow_body_y, burst.squal);

                    // リングバッファに追加（常時更新）
                    g_optflow_buffer[g_optflow_buffer_index] = {flow_body_x, flow_body_y, burst.squal};
                    g_optflow_buffer_index = (g_optflow_buffer_index + 1) % REF_BUFFER_SIZE;
                    if (g_optflow_buffer_count < REF_BUFFER_SIZE) {
                        g_optflow_buffer_count++;
                    }
                    g_optflow_data_ready = true;
                } else {
                    g_health.optflow.recordFailure();
                    g_optflow_task_healthy = g_health.optflow.isHealthy();
                }
            } catch (const stampfly::PMW3901Exception& e) {
                g_health.optflow.recordFailure();
                g_optflow_task_healthy = g_health.optflow.isHealthy();
            }
        }

        g_optflow_checkpoint = 99;  // ループ完了
        vTaskDelayUntil(&last_wake_time, period);
    }
}
