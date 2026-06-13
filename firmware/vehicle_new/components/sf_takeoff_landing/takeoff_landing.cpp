/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_landing.cpp
 * @brief Takeoff/landing manager implementation
 *        離着陸マネージャー実装
 *
 * @design architecture.md §4 — State machine transitions               [OK]
 * @design detailed_design.md §3 — State transition table (TAKEOFF/LANDING) [OK]
 */

#include "takeoff_landing.hpp"
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
void TakeoffLandingMgr::update(const TofData& tof, bool armed, float vertical_velocity)
{
    // Disarmed → definitively on the ground (proven firmware/vehicle pattern). While
    // disarmed the only motion signal is the ToF, so evaluate held-in-hand here; clear the
    // landing latch and the debounce streaks for the next flight.
    // disarmed → 確実に接地（実証済みの firmware/vehicle パターン）。disarmed 中の唯一の動き信号は
    // ToF ゆえここで手持ちを判定する。着陸ラッチとデバウンス連続数を次飛行のためクリア。
    if (!armed) {
        on_ground_ = true;
        landing_detected_ = false;
        landing_start_ms_ = 0;
        airborne_streak_ = 0;
        ground_streak_   = 0;
        evaluateHeld(tof);
        return;
    }

    held_ = false;   // armed: by definition not hand-held / armed 中は定義上手持ちでない
    evaluateToF(tof);
    detectLanding(vertical_velocity);
}

// -----------------------------------------------------------------------------
// evaluateToF — determine ground contact from the injected ToF distance
// ToF評価 — 注入された ToF 距離から地面接地を判定
//
// TAKEOFF→FLYING (state_task, via the published airborne flag) and the ESKF
// vertical handoff (imu_task, on the ground↔air edge) both key off on_ground_,
// so a flip here must be glitch-proof: require flip_confirm_samples consecutive
// same-side readings before flipping. A single ghost/furniture reflection can
// no longer cross the craft into "airborne" (or vice versa) (M-6).
// TAKEOFF→FLYING（state_task、発行された airborne フラグ経由）と ESKF 鉛直ハンドオフ
// （imu_task、接地↔空中エッジ）は共に on_ground_ を起点にするため、ここの反転は
// グリッチ耐性が要る: flip_confirm_samples 回連続の同側読みで初めて反転する。単発の
// ゴースト/家具反射では「空中」へ（逆も）渡れなくなる (M-6)。
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::evaluateToF(const TofData& tof)
{
    // Skip invalid readings — on the ground the ToF sits below its minimum range
    // and returns invalid, so on_ground_ holds its last value (true at boot). An
    // invalid sample also breaks a building streak (a flip needs clean, consecutive
    // valid confirmations); it can only DELAY a flip, never cause a spurious one.
    // 無効な読み取りをスキップ — 接地中は ToF が最小レンジ未満で無効を返すため、
    // on_ground_ は直前値を保持（起動時 true）。無効サンプルは連続数も途切れさせる
    // （反転には連続した有効確認が要る）。反転を遅らせるだけで誤反転は起こさない。
    if (!tof.valid) {
        airborne_streak_ = 0;
        ground_streak_   = 0;
        return;
    }

    // Count consecutive same-side samples; flip only after flip_confirm_samples.
    // The hysteresis band (between the two thresholds) breaks both streaks so a
    // flip requires N consecutive readings fully past the SAME threshold.
    // 同側連続をカウントし flip_confirm_samples 到達で初めて反転。2閾値間の
    // ヒステリシス帯では両連続数を切り、反転には同一閾値を完全に越えた N 連続が要る。
    if (tof.distance > config_.airborne_tof_m) {
        ground_streak_ = 0;
        if (airborne_streak_ < 255) ++airborne_streak_;
        if (airborne_streak_ >= config_.flip_confirm_samples) {
            on_ground_ = false;
        }
    } else if (tof.distance < config_.ground_tof_m) {
        airborne_streak_ = 0;
        if (ground_streak_ < 255) ++ground_streak_;
        if (ground_streak_ >= config_.flip_confirm_samples) {
            on_ground_ = true;
        }
    } else {
        airborne_streak_ = 0;
        ground_streak_   = 0;
    }
}

// -----------------------------------------------------------------------------
// detectLanding — low altitude + low velocity sustained → landing event
// 着陸検出 — 低高度＋低速度の持続 → 着陸イベント
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::detectLanding(float vertical_velocity)
{
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    float vz = std::fabs(vertical_velocity);  // injected, not a topic read / 注入値

    // Landing condition: on the ground + low vertical velocity sustained for
    // landing_hold_ms. landing_detected_ is a LEVEL flag — true while the condition holds,
    // cleared when it breaks — so a 50 Hz consumer (StateTask) cannot miss it through a
    // Latest topic (unlike a one-shot pulse).
    // 着陸条件: 接地＋低鉛直速度が landing_hold_ms 持続。landing_detected_ はレベルフラグ
    // （条件成立中 true、崩れたらクリア）— ワンショットと違い 50Hz の消費者（StateTask）が
    // Latest トピック越しに取りこぼさない。
    if (on_ground_ && vz < config_.landing_vel_mps) {
        if (landing_start_ms_ == 0) {
            landing_start_ms_ = now_ms;
        }
        if (now_ms - landing_start_ms_ >= config_.landing_hold_ms) {
            if (!landing_detected_) {
                ESP_LOGI(TAG, "Landing detected");   // log once on the rising edge / 立上りで1回
            }
            landing_detected_ = true;
        }
    } else {
        landing_start_ms_ = 0;
        landing_detected_ = false;
    }
}

// -----------------------------------------------------------------------------
// evaluateHeld — held-in-hand detection from ToF distance (disarmed only)
// 手持ち判定 — ToF 距離から（disarmed 時のみ）
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::evaluateHeld(const TofData& tof)
{
    // A valid ToF distance above the airborne threshold → the craft has been lifted off
    // the ground (held). Invalid (below min range = sitting on the ground) or below the
    // ground threshold → placed down. Hysteresis between the two thresholds prevents
    // chatter when held near a threshold by hand.
    // 空中閾値以上の有効 ToF 距離 → 持ち上げられた（held）。無効（最小レンジ未満＝接地）or
    // 地上閾値未満 → 置かれた。2閾値間のヒステリシスで、手で閾値付近に保持した際のチャタリング防止。
    if (!tof.valid) {
        held_ = false;   // on the ground the ToF reads invalid below its minimum range
        return;
    }
    if (tof.distance > config_.airborne_tof_m) {
        held_ = true;
    } else if (tof.distance < config_.ground_tof_m) {
        held_ = false;
    }
    // between the two thresholds: hold the previous held_ (hysteresis)
}

}  // namespace sf
