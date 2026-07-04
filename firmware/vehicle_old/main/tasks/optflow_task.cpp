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
            // Always update timestamp on every poll attempt
            // ポーリング試行ごとにタイムスタンプを更新
            g_flow_last_timestamp_us = static_cast<uint32_t>(esp_timer_get_time());

            try {
                auto burst = g_optflow->readMotionBurst();
                g_optflow_checkpoint = 2;  // readMotionBurst成功

                // PMW3901座標系 → 中間座標系 変換
                int16_t flow_body_x = -burst.delta_y;
                int16_t flow_body_y =  burst.delta_x;

                // Record with actual quality (valid or not — let consumer decide)
                // 実際の品質値で記録（有効/無効はデータ利用側が判断）
                state.updateOpticalFlow(flow_body_x, flow_body_y, burst.squal);
                g_flow_buf.push(OptFlowData{flow_body_x, flow_body_y, burst.squal});
                g_optflow_data_ready = true;

                if (stampfly::OutlierDetector::isFlowValid(burst.squal)) {
                    g_health.optflow.recordSuccess();
                } else {
                    g_health.optflow.recordFailure();
                }
                g_optflow_task_healthy = g_health.optflow.isHealthy();
            } catch (const stampfly::PMW3901Exception& e) {
                // Read failed — record with quality=0 to indicate error
                // 読み取り失敗 — quality=0 でエラーを示す
                state.updateOpticalFlow(0, 0, 0);
                g_flow_buf.push(OptFlowData{0, 0, 0});
                g_optflow_data_ready = true;
                g_health.optflow.recordFailure();
                g_optflow_task_healthy = g_health.optflow.isHealthy();
            }
        }

        g_optflow_checkpoint = 99;  // ループ完了
        state.updateSensorDiag("flow", g_optflow_task_healthy, g_flow_last_timestamp_us);
        vTaskDelayUntil(&last_wake_time, period);
    }
}
