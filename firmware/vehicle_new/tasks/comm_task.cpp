/**
 * @file comm_task.cpp
 * @brief Communication task — owns WiFi + ESP-NOW receive loop
 *        通信タスク — WiFi + ESP-NOW 受信ループを所有する
 *
 * Owns the Comm singleton. Initializes WiFi (STA) and ESP-NOW once at
 * task entry, then runs a slow diagnostic poll while the actual decode
 * and publish happens in the recv callback (WiFi task context).
 *
 * Comm シングルトンを所有する。タスク起動時に WiFi (STA) と ESP-NOW を
 * 1 回だけ初期化し、以降はゆっくりとした診断ポーリングを行う。実際の
 * デコードと発行は受信コールバック（WiFi タスクコンテキスト）で行う。
 *
 * @design architecture.md §6 — CommTask: Communication + Command       [OK]
 * @design detailed_design.md §8 — CommTask: 50Hz, priority 15          [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "comm.hpp"

static const char* TAG = "CommTask";

/// Period at which CommTask's diagnostic poll runs. The ESP-NOW recv
/// callback is asynchronous — this period only gates the connection-
/// status update used by isEspNowConnected().
/// CommTask の診断ポーリング周期。ESP-NOW 受信コールバックは非同期
/// であり、本周期は isEspNowConnected() が使う接続状態更新のみを支配する。
static constexpr TickType_t kCommTaskPeriodTicks = pdMS_TO_TICKS(20);  // 50Hz

void CommTask(void* pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "CommTask started — initializing WiFi + ESP-NOW");

    // Singleton owned by the task. The static recv callback reaches it
    // via a file-scoped pointer set inside Comm::init().
    // タスクが所有するシングルトン。静的受信コールバックは Comm::init()
    // 内で設定されるファイルスコープのポインタ経由で到達する。
    static sf::Comm comm;
    comm.init();

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        // Refresh connection-state heartbeat.
        // 接続状態ハートビートを更新する。
        comm.update();

        vTaskDelayUntil(&last_wake, kCommTaskPeriodTicks);
    }
}
