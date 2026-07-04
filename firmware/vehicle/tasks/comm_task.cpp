/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file comm_task.cpp
 * @brief Communication task — owns WiFi + ESP-NOW receive + command processing
 *        通信タスク — WiFi + ESP-NOW 受信 + コマンド処理を所有する
 *
 * Owns BOTH the Comm singleton (HAL, #9) and the CommandProcessor (Service, #8),
 * and WIRES them together — the orchestration the design assigns to CommTask.
 * It injects the CommandProcessor into Comm as a RawInputSink, so each validated
 * ESP-NOW packet is normalized/deadbanded/decoded straight away in the receive
 * path (at packet time, no added latency). The task loop itself only refreshes
 * the connection-state heartbeat; the real per-packet work runs through the
 * injected sink.
 *
 * sf_comm never names sf_command — it calls a function-pointer sink over the
 * sf_core RawControlInput type — so the physical layer carries no dependency on
 * the command layer; CommTask (which owns both) supplies the dependency. The
 * marginally-stable POSITION_HOLD scenarios are sensitive to command-delivery
 * phase, so processing at packet time (rather than polling) preserves the proven
 * low-latency path.
 *
 * Comm シングルトン（HAL・#9）と CommandProcessor（Service・#8）の両方を所有し、両者を
 * 配線する — 設計が CommTask に割り当てる調整役。CommandProcessor を RawInputSink として
 * Comm に注入し、各検証済み ESP-NOW パケットを受信経路でそのまま正規化/デッドバンド/デコード
 * する（パケット到着時・追加レイテンシなし）。タスクループ自体は接続状態ハートビートを更新する
 * だけで、パケット毎の実処理は注入された sink を通る。
 *
 * sf_comm は sf_command を名指ししない — sf_core の RawControlInput 型に対する関数ポインタ
 * sink を呼ぶだけ — ので物理層はコマンド層に依存しない。依存は両方を所有する CommTask が
 * 供給する。限界安定の POSITION_HOLD シナリオはコマンド配送の位相に敏感なので、（ポーリング
 * でなく）パケット時処理が実証済みの低レイテンシ経路を保つ。
 *
 * @publisher command_setpoint, pilot_request (via the injected CommandProcessor)
 * @design architecture.md §6 — CommTask: Communication + Command       [OK]
 * @design detailed_design.md §8 — CommTask: 50Hz, priority 15          [OK]
 * @design architecture.md §3 — R13 @publisher/@subscriber annotation   [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "comm.hpp"
#include "command.hpp"

static const char* TAG = "CommTask";

/// CommTask period. The receive path does the per-packet command work via the
/// injected sink, so this loop only refreshes the connection-state heartbeat used
/// by isEspNowConnected(); 50Hz is ample for that diagnostic.
/// CommTask の周期。パケット毎のコマンド処理は注入 sink 経由で受信経路が行うため、本ループは
/// isEspNowConnected() が使う接続状態ハートビートの更新のみ。診断用途には 50Hz で十分。
static constexpr TickType_t kCommTaskPeriodTicks = pdMS_TO_TICKS(20);  // 50Hz

namespace {

/// CommandProcessor owned by CommTask (Service layer). File-scope so the plain
/// function-pointer sink below can reach it (no captures needed).
/// CommTask が所有する CommandProcessor（Service 層）。下の関数ポインタ sink から到達できる
/// ようファイルスコープに置く（キャプチャ不要）。
sf::CommandProcessor g_command;

/// RawInputSink: hand each validated raw input to the CommandProcessor. This is
/// what CommTask injects into Comm, so the comm receive path normalizes/decodes
/// via sf_command without ever naming it.
/// RawInputSink: 各検証済み生入力を CommandProcessor へ渡す。これを CommTask が Comm に注入し、
/// comm 受信経路が sf_command を名指しせずに正規化/デコードする。
void commandSink(const sf::RawControlInput& raw)
{
    g_command.process(raw);
}

}  // namespace

void CommTask(void* pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "CommTask started — initializing WiFi + ESP-NOW + command");

    // Comm singleton owned by the task. The static recv callback reaches it via a
    // file-scoped pointer set inside Comm::init().
    // タスクが所有する Comm シングルトン。静的受信コールバックは Comm::init() 内で設定される
    // ファイルスコープのポインタ経由で到達する。
    static sf::Comm comm;

    // Wire the Service layer in BEFORE init() (before the radio can deliver a
    // packet): init the CommandProcessor and inject it as Comm's raw-input sink.
    // 無線がパケットを配達し得る前（init() 前）に Service 層を配線する: CommandProcessor を
    // 初期化し、Comm の生入力 sink として注入する。
    g_command.init();
    comm.setRawInputSink(&commandSink);
    comm.init();

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        // Refresh connection-state heartbeat (per-packet work runs in the sink).
        // 接続状態ハートビートを更新する（パケット毎の処理は sink で実行）。
        comm.update();

        vTaskDelayUntil(&last_wake, kCommTaskPeriodTicks);
    }
}
