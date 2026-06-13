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
// on_ground_ drives TAKEOFF→FLYING (state_task, via the published airborne flag)
// and the ESKF vertical handoff (imu_task, on the ground↔air edge). The two
// directions are treated ASYMMETRICALLY (M-6):
//   - ground→air (takeoff): IMMEDIATE. The handoff is a sample-precision class-B
//     reset (architecture.md §4); debouncing it delays the handoff and degrades
//     the take-off transient (measured duty_max 0.80 → 1.0). A spurious ground
//     ghost flipping airborne is bounded — TAKEOFF needs throttle>0.5 first, and
//     the on-ground handoff reset (pos/vel ≈ 0 already) is harmless.
//   - air→ground (false-ground mid-flight / landing): DEBOUNCED. A single near
//     reflection can no longer assert "on ground"; landing has landing_hold_ms
//     downstream so the ~66ms confirm is negligible.
// on_ground_ は TAKEOFF→FLYING（state_task、airborne フラグ経由）と ESKF 鉛直
// ハンドオフ（imu_task、接地↔空中エッジ）を駆動する。2方向を非対称に扱う (M-6):
//   - ground→air（離陸）= 即時。ハンドオフはサンプル精度のクラスB reset（§4）で、
//     デバウンスすると遅延し離陸過渡が劣化（実測 duty_max 0.80→1.0）。地上ゴーストで
//     空中へ誤反転する害は限定的（TAKEOFF は throttle>0.5 が前提、地上ハンドオフ reset は
//     pos/vel≈0 で無害）。
//   - air→ground（飛行中の偽接地/着陸）= デバウンス。単発近距離反射で「接地」を主張
//     できなくする。着陸は下流に landing_hold_ms があり ~66ms 確認は無視できる。
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::evaluateToF(const TofData& tof)
{
    // Skip invalid readings — on the ground the ToF sits below its minimum range
    // and returns invalid, so on_ground_ holds its last value (true at boot). An
    // invalid sample also breaks a building ground streak.
    // 無効な読み取りをスキップ — 接地中は ToF が最小レンジ未満で無効を返すため、
    // on_ground_ は直前値を保持（起動時 true）。無効サンプルは地上連続数も途切れさせる。
    if (!tof.valid) {
        ground_streak_ = 0;
        return;
    }

    if (tof.distance > config_.airborne_tof_m) {
        // Airborne: flip immediately (prompt class-B handoff at takeoff).
        // 空中: 即時反転（離陸時のクラスB ハンドオフを遅らせない）。
        ground_streak_ = 0;
        on_ground_ = false;
    } else if (tof.distance < config_.ground_tof_m) {
        // Ground: flip only after flip_confirm_samples consecutive ground reads
        // (reject a single near-reflection ghost mid-flight).
        // 接地: 地上読みが flip_confirm_samples 回連続して初めて反転（飛行中の単発
        // 近距離ゴーストを排除）。
        if (ground_streak_ < 255) ++ground_streak_;
        if (ground_streak_ >= config_.flip_confirm_samples) {
            on_ground_ = true;
        }
    } else {
        // Hysteresis band between the thresholds: hold the flag, reset the streak.
        // 2閾値間のヒステリシス帯: フラグ保持、連続数リセット。
        ground_streak_ = 0;
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
