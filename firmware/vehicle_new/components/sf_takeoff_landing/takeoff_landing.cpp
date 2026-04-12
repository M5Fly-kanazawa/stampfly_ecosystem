/**
 * @file takeoff_landing.cpp
 * @brief Takeoff/landing manager implementation
 *        離着陸マネージャー実装
 *
 * @design architecture.md §4 — State machine transitions               [--]
 * @design detailed_design.md §11 — Takeoff/landing logic               [--]
 */

#include "takeoff_landing.hpp"
#include "topics.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>

static const char* TAG = "takeoff_landing";

namespace sf {

// -----------------------------------------------------------------------------
// init — initialize with default config
// 初期化 — デフォルト設定で初期化
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::init()
{
    config_ = TakeoffLandingConfig{};
    ESP_LOGI(TAG, "TakeoffLandingMgr initialized (ground=%.2fm, airborne=%.2fm)",
             config_.ground_tof_m, config_.airborne_tof_m);
}

// -----------------------------------------------------------------------------
// init — initialize with custom config
// 初期化 — カスタム設定で初期化
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::init(const TakeoffLandingConfig& config)
{
    config_ = config;
    ESP_LOGI(TAG, "TakeoffLandingMgr initialized (custom config)");
}

// -----------------------------------------------------------------------------
// update — run takeoff/landing detection
// 更新 — 離陸/着陸検出を実行
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::update()
{
    // Clear one-shot event flags
    // ワンショットイベントフラグをクリア
    takeoff_detected_ = false;
    landing_detected_ = false;

    evaluateToF();
    detectTakeoff();
    detectLanding();
}

// -----------------------------------------------------------------------------
// evaluateToF — determine ground contact from ToF distance
// ToF評価 — ToF距離から地面接地を判定
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::evaluateToF()
{
    // Read ToF data from topic
    // トピックからToFデータを読み取る
    TofData tof;
    if (!sensor_tof.read(tof)) {
        return;  // No new data / 新しいデータなし
    }

    // Skip invalid readings
    // 無効な読み取りをスキップ
    if (!tof.valid) {
        return;
    }

    // Determine ground contact based on distance threshold
    // 距離閾値に基づき地面接地を判定
    if (tof.distance < config_.ground_tof_m) {
        on_ground_ = true;
    } else if (tof.distance > config_.airborne_tof_m) {
        on_ground_ = false;
    }
    // Hysteresis band between ground and airborne thresholds
    // 地上閾値と空中閾値の間はヒステリシスバンド
}

// -----------------------------------------------------------------------------
// detectTakeoff — sustained altitude above threshold → takeoff event
// 離陸検出 — 閾値以上の高度持続 → 離陸イベント
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::detectTakeoff()
{
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    if (!on_ground_) {
        // Start or continue takeoff timer
        // 離陸タイマーを開始または継続
        if (takeoff_start_ms_ == 0) {
            takeoff_start_ms_ = now_ms;
        }

        // Confirm takeoff after hold duration
        // 保持時間経過後に離陸を確認
        if (now_ms - takeoff_start_ms_ >= config_.takeoff_hold_ms) {
            takeoff_detected_ = true;
            takeoff_start_ms_ = 0;
            ESP_LOGI(TAG, "Takeoff detected");
        }
    } else {
        // Reset timer when on ground
        // 地上ではタイマーをリセット
        takeoff_start_ms_ = 0;
    }
}

// -----------------------------------------------------------------------------
// detectLanding — low altitude + low velocity sustained → landing event
// 着陸検出 — 低高度＋低速度の持続 → 着陸イベント
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::detectLanding()
{
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    // Read velocity from state estimate
    // 状態推定から速度を読み取る
    StateEstimate est = estimate_state.latest();
    float vz = std::fabs(est.velocity[2]);  // Vertical velocity / 垂直速度

    // Check landing conditions: on ground + low vertical velocity
    // 着陸条件をチェック: 地上＋低垂直速度
    if (on_ground_ && vz < config_.landing_vel_mps) {
        if (landing_start_ms_ == 0) {
            landing_start_ms_ = now_ms;
        }

        // Confirm landing after hold duration
        // 保持時間経過後に着陸を確認
        if (now_ms - landing_start_ms_ >= config_.landing_hold_ms) {
            landing_detected_ = true;
            landing_start_ms_ = 0;
            ESP_LOGI(TAG, "Landing detected");
        }
    } else {
        // Reset timer when conditions not met
        // 条件を満たさない場合はタイマーをリセット
        landing_start_ms_ = 0;
    }
}

}  // namespace sf
