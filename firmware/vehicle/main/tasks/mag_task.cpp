/**
 * @file mag_task.cpp
 * @brief 地磁気タスク (100Hz) - BMM150読み取り、キャリブレーション適用
 */

#include "tasks_common.hpp"

static const char* TAG = "MagTask";

using namespace config;
using namespace globals;

void MagTask(void* pvParameters)
{
    ESP_LOGI(TAG, "MagTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(static_cast<TickType_t>(MAG_DT * 1000.0f));

    auto& state = stampfly::StampFlyState::getInstance();

    // ヘルスモニター設定
    g_health.mag.setThresholds(10, 10);

    while (true) {
        if (g_mag.isInitialized()) {
            stampfly::MagData mag;
            if (g_mag.read(mag) == ESP_OK) {
                g_health.mag.recordSuccess();
                g_mag_task_healthy = g_health.mag.isHealthy();
                // ============================================================
                // BMM150座標系 → 機体座標系(NED) 変換
                // 実測により確認:
                //   機体X = -センサY
                //   機体Y = センサX
                //   機体Z = センサZ
                // ============================================================
                float mag_body_x = -mag.y;  // 前方正
                float mag_body_y = mag.x;   // 右正
                float mag_body_z = mag.z;   // 下正 (NED)

                // キャリブレーションデータ収集中の場合
                if (g_mag_cal.getState() == stampfly::MagCalibrator::State::COLLECTING) {
                    g_mag_cal.addSample(mag_body_x, mag_body_y, mag_body_z);
                }

                // キャリブレーション適用
                float cal_mag_x, cal_mag_y, cal_mag_z;
                if (g_mag_cal.isCalibrated()) {
                    g_mag_cal.applyCalibration(mag_body_x, mag_body_y, mag_body_z,
                                                cal_mag_x, cal_mag_y, cal_mag_z);
                } else {
                    // キャリブレーション未実施の場合は生データをそのまま使用
                    cal_mag_x = mag_body_x;
                    cal_mag_y = mag_body_y;
                    cal_mag_z = mag_body_z;
                }

                // Update state with calibrated data
                state.updateMag(cal_mag_x, cal_mag_y, cal_mag_z);

                // キャリブレーション済みの場合のみESKF用データを準備
                if (g_mag_cal.isCalibrated()) {
                    stampfly::math::Vector3 m(cal_mag_x, cal_mag_y, cal_mag_z);

                    // リングバッファに追加（常時更新）
                    // 初期化時は平均計算、ESKF実行時は最新値を使用
                    g_mag_buffer[g_mag_buffer_index] = m;
                    g_mag_buffer_index = (g_mag_buffer_index + 1) % REF_BUFFER_SIZE;
                    if (g_mag_buffer_count < REF_BUFFER_SIZE) {
                        g_mag_buffer_count++;
                    }

                    // 新しいデータがあることを示すフラグ（100Hz）
                    // ESKF updateはIMUTask内で行う（レースコンディション防止）
                    g_mag_data_ready = true;
                }
            } else {
                g_health.mag.recordFailure();
                g_mag_task_healthy = g_health.mag.isHealthy();
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
