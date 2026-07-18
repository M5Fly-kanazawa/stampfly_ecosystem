/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (workshop firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file workshop_main.cpp
 * @brief StampFly workshop — Main Entry Point
 *        StampFly workshop — メインエントリポイント
 *
 * Rebuilt on the current firmware/vehicle component base (cross-cutting rule
 * R12, workshop_migration.md M5): same declarative 5-phase boot as
 * vehicle/main/main.cpp (NVS -> BSP -> topics -> params -> tasks). The only
 * difference from vehicle is Phase 4, which starts sf::workshop_tasks
 * (ControlTask replaced by WorkshopControlTask so learner code drives the
 * motors via ws::*), plus a one-time legacy NVS channel import right after
 * Phase 3.
 *
 * 現行 firmware/vehicle のコンポーネント基盤上に再構築（横断ルール R12、
 * workshop_migration.md M5）: vehicle/main/main.cpp と同じ宣言的 5 フェーズ
 * 起動（NVS → BSP → トピック → params → タスク）。vehicle との違いは Phase 4
 * のみ（sf::workshop_tasks を起動し、ControlTask を WorkshopControlTask に
 * 差し替え、学習者コードが ws::* 経由でモータを駆動する）と、Phase 3 直後の
 * 旧チャンネル NVS 移行シム。
 *
 * @design workshop_migration.md M5 — workshop rebuild on vehicle base  [OK]
 * @design hardware_init.md §4 — 起動シーケンス (Phase 0..4)           [OK]
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "topics.hpp"
#include "params.hpp"
#include "sf_board.hpp"

static const char* TAG = "workshop";

// WorkshopControlTask startup, defined in workshop_tasks.cpp. Declared here
// (not a separate header) — the workshop main component owns exactly one
// task-table entry point, mirroring vehicle's sf::tasks::start_all() shape.
// WorkshopControlTask の起動処理。workshop_tasks.cpp で定義（専用ヘッダは置かない）—
// workshop の main コンポーネントはタスク表の入口を1つだけ持ち、vehicle の
// sf::tasks::start_all() と同じ形にする。
namespace sf {
namespace workshop_tasks {
void start_all();
}  // namespace workshop_tasks
}  // namespace sf

/// Import the legacy workshop WiFi channel (NVS "stampfly"/"wifi_ch") into the
/// current sf::params "wifi.channel", exactly once. Old workshop builds wrote
/// the channel directly to that legacy key; the current param system uses its
/// own NVS namespace ("sf_params"), so without this shim a student's saved
/// channel would silently revert to the compiled default (1) on first boot of
/// the rebuilt firmware.
/// 旧 workshop の WiFi チャンネル（NVS "stampfly"/"wifi_ch"）を、現行の
/// sf::params "wifi.channel" へ一度だけ取り込む。旧 workshop はチャンネルを
/// その旧キーへ直接書き込んでいた。現行のパラメータ系は別の NVS 名前空間
/// （"sf_params"）を使うため、このシムが無いと再構築ファーム初回起動時に
/// 学生が保存したチャンネルが黙ってコンパイル時デフォルト(1)へ戻ってしまう。
///
/// @design W1_SPEC.md §2-3 — Legacy channel import shim                [OK]
static void importLegacyWifiChannel()
{
    if (sf::params::has_saved("wifi.channel")) {
        return;  // already migrated (or set) via the param system / 移行済み・設定済み
    }

    nvs_handle_t h;
    if (nvs_open("stampfly", NVS_READONLY, &h) != ESP_OK) {
        return;  // no legacy namespace — fresh device, nothing to import / 旧NVS無し
    }

    uint8_t legacy_ch = 0;
    if (nvs_get_u8(h, "wifi_ch", &legacy_ch) == ESP_OK &&
        legacy_ch >= 1 && legacy_ch <= 13) {
        sf::params::set_int("wifi.channel", legacy_ch);
        sf::params::save_one("wifi.channel");
        ESP_LOGI(TAG, "Imported legacy wifi channel %d", legacy_ch);
    }
    nvs_close(h);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== StampFly workshop starting ===");

    // =========================================================================
    // Phase 0: NVS (required by WiFi and parameter persistence)
    // Phase 0: NVS（WiFi とパラメータ永続化に必要）
    // =========================================================================
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Phase 0: NVS initialized");

    // =========================================================================
    // Phase 1: BSP — sf_board owns every shared HW resource (I2C/SPI/LEDC/netif).
    // Phase 1: BSP — sf_board が共有 HW 資源を一元所有。
    // =========================================================================
    ESP_ERROR_CHECK(sf::internal::board::init());
    ESP_LOGI(TAG, "Phase 1: BSP ready (sf_board)");

    // =========================================================================
    // Phase 2: Pub-Sub topics (buffers/mutexes ready before any task runs).
    // Phase 2: Pub-Sub トピック（タスク起動前にバッファ/mutex を準備）。
    // =========================================================================
    sf::topics_init();
    ESP_LOGI(TAG, "Phase 2: Topics initialized");

    // Workshop identity sound: select the school-chime boot melody (授業開始チャイム,
    // Buzzer::schoolChimeTone) instead of the vehicle's standard power-on chime
    // (C5→E5→G5, Buzzer::startTone). Published exactly once, right here — the
    // ui_command topic buffer now exists (topics_init() above) and no task has
    // started yet (Phase 4 below), so this sits in the queue when NotifyTask's very
    // first ~30Hz update() cycle drains ui_command (see notify.cpp/notify.hpp) —
    // long before the ~3s / 90-cycle deferred boot-tone actually fires. The vehicle
    // firmware never publishes UiCmd::BootMelody, so its boot sound (and every other
    // Notify::update() behaviour) is byte-for-byte unchanged.
    // workshop の識別音: vehicle の標準起動音（C5→E5→G5, Buzzer::startTone）の代わりに
    // 授業チャイム（授業開始チャイム, Buzzer::schoolChimeTone）を選択する。ここで一度
    // だけ publish する — ui_command トピックのバッファは（上の topics_init() で）
    // 既に存在し、まだどのタスクも起動していない（下の Phase 4 より前）ため、
    // NotifyTask の最初の約30Hz update() サイクルが ui_command を drain する時点で
    // 既にキューに入っている（notify.cpp/notify.hpp 参照）— 遅延起動音が実際に鳴る
    // 約3秒/90周期より十分前。vehicle は UiCmd::BootMelody を一切 publish しないため、
    // その起動音（および Notify::update() の他の挙動）はバイト単位で不変のまま。
    sf::ui_command.publish(sf::UiCommand{
        static_cast<uint8_t>(sf::UiCmd::BootMelody), 1, 0, 0, 0,
        static_cast<uint32_t>(esp_timer_get_time())});

    // =========================================================================
    // Phase 3: Parameters — restore tuned values from NVS, else compile-time
    // defaults. Followed by the one-time legacy channel import (see above).
    // Phase 3: パラメータ — NVS から調整値を復元、無ければコンパイル時デフォルト。
    // 続けて旧チャンネルの一度限りの取り込み（上記参照）。
    // =========================================================================
    sf::params::init();
    importLegacyWifiChannel();
    ESP_LOGI(TAG, "Phase 3: Parameters loaded");

    // =========================================================================
    // Phase 4: Tasks — same task table as vehicle, ControlTask replaced by
    // WorkshopControlTask (see sf::workshop_tasks::start_all()).
    // Phase 4: タスク — vehicle と同じタスク表、ControlTask のみ
    // WorkshopControlTask に置換（sf::workshop_tasks::start_all() 参照）。
    // =========================================================================
    sf::workshop_tasks::start_all();
    ESP_LOGI(TAG, "=== Phase 4: tasks started — workshop ready ===");
}
