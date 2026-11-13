/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_landing.cpp
 * @brief Takeoff/landing manager implementation
 *        離着陸マネージャー実装
 *
 * @design architecture.md §4 — State machine transitions               [OK]
 * @design detailed_design.md §3 — State transition table (TAKEOFF/LANDING) [OK]
 * @design detailed_design.md §3 注7 — stalled-descent touchdown; the rest evidence while the estimate is held [OK]
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
    configureBlindTouchdown();
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
    configureBlindTouchdown();
    ESP_LOGI(TAG, "TakeoffLandingMgr initialized (custom config)");
}

// -----------------------------------------------------------------------------
// reloadConfig — new configuration, detector state kept
// 設定の再読込 — 検出器の状態は保持
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::reloadConfig(const TakeoffLandingConfig& config)
{
    config_ = config;
    blind_touchdown_.setConfig(blindTouchdownConfig());
}

// -----------------------------------------------------------------------------
// update — run takeoff/landing detection
// 更新 — 離陸/着陸検出を実行
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::update(const TofData& tof, bool armed, float vertical_velocity,
                               bool in_landing_descent)
{
    // Disarmed → definitively on the ground (proven firmware/vehicle pattern). While
    // disarmed the only motion signal is the ToF, so evaluate held-in-hand here; clear the
    // landing latch and the debounce streaks/timers for the next flight.
    // disarmed → 確実に接地（実証済みの firmware/vehicle パターン）。disarmed 中の唯一の動き信号は
    // ToF ゆえここで手持ちを判定する。着陸ラッチ・デバウンス連続数・タイマーを次飛行のためクリア。
    if (!armed) {
        on_ground_ = true;
        landing_detected_ = false;
        landing_start_ms_ = 0;
        stall_start_ms_   = 0;
        ground_streak_   = 0;
        dead_zone_armed_ = false;
        rest_run_active_ = false;
        evaluateHeld(tof);
        return;
    }

    held_ = false;   // armed: by definition not hand-held / armed 中は定義上手持ちでない
    evaluateToF(tof);
    trackRest(tof);
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    trackDeadZone(tof, in_landing_descent, now_ms);
    detectLanding(vertical_velocity, in_landing_descent, now_ms);
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
//     reflection cannot assert "on ground"; landing has landing_hold_ms
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
        return;   // near_ground_ holds its last value across a brief invalid dropout
    }

    // Near-ground band for the stalled-descent touchdown (a wider zone than the 5cm
    // ground threshold, so the ground-effect float at ~5-12cm is covered).
    // 降下停滞接地用の近傍帯（5cm 地上閾値より広く、~5-12cm の地面効果フロートを覆う）。
    near_ground_ = (tof.distance < config_.near_ground_tof_m);

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
// noteLandingDescent — judge the landing afresh when the LANDING begins
// noteLandingDescent — LANDING が始まったら、着陸の判定を新たに行う
//
// @design detailed_design.md §3 注7 — a landing verdict of before the LANDING is no evidence of a touchdown in it [OK]
// @design architecture.md INV-3 — detection here (ImuTask feeds it every cycle), the decision stays in StateTask [OK]
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::noteLandingDescent(bool in_landing_descent)
{
    if (in_landing_descent && !was_landing_descent_) {
        landing_detected_ = false;   // the verdict of before the LANDING is no evidence of a touchdown in it / LANDING の前の判定は、その中の接地の証拠ではない
        landing_start_ms_ = 0;
        stall_start_ms_   = 0;
        dead_zone_armed_  = false;   // so is the dead-zone evidence of before it / 前から残った不感帯の証拠も同じ
        rest_run_active_  = false;   // and the stillness seen before it / その前に見た静止も同じ
    }
    was_landing_descent_ = in_landing_descent;
}

// -----------------------------------------------------------------------------
// detectLanding — low altitude + low velocity sustained → landing event
// 着陸検出 — 低高度＋低速度の持続 → 着陸イベント
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::detectLanding(float vertical_velocity, bool in_landing_descent, uint32_t now_ms)
{
    const bool at_rest = atRest(vertical_velocity);   // injected velocity, or the ToF while the estimate is anchored / 注入された速度、推定が固定されている間は ToF

    // Three independent touchdown paths, each a sustained level condition. landing_detected_
    // is a LEVEL flag (true while the condition holds, cleared when it breaks) so a 50 Hz
    // consumer (StateTask) cannot miss it through a Latest topic.
    //   (1) FIRM GROUND: ToF confirms <5cm (on_ground_) + at rest, held landing_hold_ms.
    //       The clean touchdown when the craft reaches the ground.
    //   (2) STALLED DESCENT: a landing descent is commanded + the ToF is within
    //       near_ground_tof_m + at rest, held stall_hold_ms. Catches the ground-effect
    //       float, where the craft holds altitude on reduced thrust and never reaches 5cm
    //       (so (1) never fires). Gated on in_landing_descent so a deliberate low hover is
    //       NOT a landing — only a STALLED descent is.
    //   (3) DEAD ZONE: invalid samples only after a valid one near the floor (deadZoneTouchdown()).
    // "At rest" in (1) and (2) is atRest(): the injected velocity while the ground flag is false, the ToF readings while it is true (the estimate is held at zero then); holdFor() lengthens the hold in the 5-15 cm band.
    // (1)(2) の「静止」は atRest(): 接地フラグが false の間は注入された速度、true の間は ToF の読み（そのとき推定は 0 に固定されている）。holdFor() は 5〜15 cm の帯で保持時間を長くする。
    // 3つの独立した接地経路。各々が持続レベル条件。landing_detected_ はレベルフラグ。
    //   (1) 確実な接地: ToF が <5cm を確認（on_ground_）＋静止を landing_hold_ms 持続。
    //   (2) 降下停滞: 着陸降下が指令＋ToF が near_ground_tof_m 以内＋静止を stall_hold_ms 持続。
    //       地面効果フロート（低推力で高度保持、5cm に届かず (1) が成立しない）を捕捉。
    //       in_landing_descent で判定し、意図的な低ホバーは着陸でない（停滞降下のみ着陸）。
    //   (3) 不感帯: 床の近くの有効な読みの後に無効サンプルだけ（deadZoneTouchdown()）。
    bool firm = false;
    if (on_ground_ && at_rest) {
        if (landing_start_ms_ == 0) landing_start_ms_ = now_ms;
        firm = (now_ms - landing_start_ms_ >= holdFor(config_.landing_hold_ms));
    } else {
        landing_start_ms_ = 0;
    }

    bool stalled = false;
    if (in_landing_descent && near_ground_ && at_rest) {
        if (stall_start_ms_ == 0) stall_start_ms_ = now_ms;
        stalled = (now_ms - stall_start_ms_ >= holdFor(config_.stall_hold_ms));
    } else {
        stall_start_ms_ = 0;
    }

    const bool dead_zone = deadZoneTouchdown(in_landing_descent, now_ms);   // path (3), see below / 経路 (3)、下記

    const bool detected = firm || stalled || dead_zone;
    if (detected && !landing_detected_) {
        logLandingDetected(firm, stalled, now_ms);   // once on the rising edge / 立上りで1回
    }
    landing_detected_ = detected;
}

// -----------------------------------------------------------------------------
// atRest — is the craft at rest, judged from evidence that is not a held zero?
// 静止か — 固定された 0 ではない証拠から判定する
//
// While the ground flag is false the injected vertical velocity is the estimator's own, so it is the evidence. While the flag is true ImuTask HOLDS the vertical state at zero
// (applyVerticalGroundHandoff), and the flag stays true up to kTakeoffAirborneBandM (0.15 m) after a craft has been below kTakeoffGroundBandM: a craft that climbs or descends at 0.2-0.4 m/s
// between 0.05 and 0.15 m then reports 0 m/s, which is not a measurement. There the evidence is the ToF itself (trackRest()): the readings of the present run stay within rest_band_m.
// 接地フラグが false の間、注入される鉛直速度は推定器自身のものなので、それが証拠になる。フラグが true の間は ImuTask が鉛直状態を 0 に「固定」し（applyVerticalGroundHandoff）、
// フラグは一度 kTakeoffGroundBandM を下回った機体では kTakeoffAirborneBandM（0.15 m）まで true のままなので、0.05〜0.15 m で 0.2〜0.4 m/s で上昇または降下する機体も 0 m/s を報告する。
// それは測定ではない。そこでの証拠は ToF そのもの（trackRest()）: 今の区間の読みが rest_band_m の中に収まっている。
//
// @design detailed_design.md §3 注7 — a held zero is no evidence of rest (positive evidence only) [OK]
// @design architecture.md INV-3 — detection in the detector, fed by ImuTask [OK]
// -----------------------------------------------------------------------------
bool TakeoffLandingMgr::atRest(float vertical_velocity) const
{
    if (!on_ground_) {
        return std::fabs(vertical_velocity) < config_.landing_vel_mps;
    }
    return rest_run_active_ && (rest_hi_m_ - rest_lo_m_ <= config_.rest_band_m);
}

// -----------------------------------------------------------------------------
// holdFor — the hold time of a touchdown path, longer while the stillness is judged from the ToF readings alone in the band where the ground flag is only a memory
// 保持時間 — 接地の経路の保持時間。接地フラグが記憶にすぎない帯で、ToF の読みだけから静止を判定している間は長くなる
//
// The readings of the run lie at or above kTakeoffGroundBandM (the flag cannot be the evidence there) and the estimate is held, so a craft that turns round (the top of a climb, the start of a descent) can read
// a narrow spread for a while (SILS: the ToF flattens the turn). A craft that stands still keeps its readings in the band for as long as it is asked to, a craft that moves does not, so a longer hold costs
// the float nothing but time. On the floor (readings below ground_tof_m) the base hold applies.
// 区間の読みは kTakeoffGroundBandM 以上にあり（そこでは接地フラグは証拠になれない）、推定は固定されているので、向きを変える機体（上昇の頂点、降下の始まり）は、しばらく狭い広がりを読みうる
// （SILS: ToF が折り返しを平らにする）。静止する機体は求められるだけ長く読みを帯の中に保ち、動く機体は保たないので、保持時間を長くしても、フロートには時間のほかに失うものは無い。
// 床の上（読みが ground_tof_m 未満）では基本の保持時間のまま。
// -----------------------------------------------------------------------------
uint32_t TakeoffLandingMgr::holdFor(uint32_t base_hold_ms) const
{
    const bool judged_by_readings_above_floor = on_ground_ && rest_run_active_ && (rest_hi_m_ >= config_.ground_tof_m);
    if (judged_by_readings_above_floor && config_.rest_hold_ms > base_hold_ms) {
        return config_.rest_hold_ms;
    }
    return base_hold_ms;
}

// -----------------------------------------------------------------------------
// trackRest — the run of ToF readings that stays within rest_band_m, while the ground flag is true
// 静止の区間 — 接地フラグが true の間、rest_band_m の中に収まる ToF の読みの区間
//
// A valid reading starts the run or widens it. A reading that makes the run wider than rest_band_m ends it: the craft moved, and the next valid reading starts a new run. An invalid
// sample neither starts a run nor ends one (no evidence either way). The run is not kept while the flag is false (atRest() uses the velocity then).
// 有効な読みは区間を始める、または広げる。区間の幅が rest_band_m を超える読みは区間を終わらせる: 機体は動いた。次の有効な読みが新しい区間を始める。
// 無効サンプルは区間を始めも終わらせもしない（どちらの証拠でもない）。フラグが false の間は区間を保たない（そのとき atRest() は速度を使う）。
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::trackRest(const TofData& tof)
{
    if (!on_ground_) {
        rest_run_active_ = false;
        return;
    }
    if (!tof.valid) {
        return;
    }
    if (!rest_run_active_) {
        rest_run_active_ = true;
        rest_lo_m_ = tof.distance;
        rest_hi_m_ = tof.distance;
        return;
    }
    rest_lo_m_ = std::fmin(rest_lo_m_, tof.distance);
    rest_hi_m_ = std::fmax(rest_hi_m_, tof.distance);
    if (rest_hi_m_ - rest_lo_m_ > config_.rest_band_m) {
        rest_run_active_ = false;   // the craft moved: atRest() is false from this sample, a new run starts with the next one / 機体は動いた: この読みから atRest() は false、新しい区間は次の読みから
    }
}

// -----------------------------------------------------------------------------
// deadZoneTouchdown — path (3) of detectLanding(): the touchdown seen as the sensor's dead zone
// 不感帯での接地 — detectLanding() の経路 (3): センサの不感帯として見える接地
//
// A landing descent is commanded AND a valid reading inside dead_zone_band_m was followed by ONLY invalid samples for
// dead_zone_hold_ms. The sensor reads "no target" below about 0.031 m, so a craft that reaches the floor leaves the
// valid range; this is the touchdown of a soft contact (under 3 G) that the firm-ground path (1) and the stalled-descent
// path (2) cannot see. No velocity condition: the craft's own velocity estimate is not reliable there.
// 着陸降下が指令されている ∧ dead_zone_band_m 以内の有効な読みの後に、無効サンプルだけが dead_zone_hold_ms 続く。
// センサは約 0.031 m より近いと「目標なし」を返すので、床に着いた機体は有効範囲を出る。確実な接地の経路 (1) と降下停滞
// の経路 (2) では見えない柔らかい接触（3 G 未満）の接地。速度の条件は無い: そこでは機体自身の速度推定を信用できない。
//
// @design detailed_design.md §3 注7 — dead-zone touchdown, the third path of the landing detector [OK]
// -----------------------------------------------------------------------------
bool TakeoffLandingMgr::deadZoneTouchdown(bool in_landing_descent, uint32_t now_ms) const
{
    return in_landing_descent && dead_zone_armed_ &&
           (now_ms - dead_zone_valid_ms_ >= config_.dead_zone_hold_ms);
}

// -----------------------------------------------------------------------------
// logLandingDetected — say which path detected the landing
// 着陸を検出した経路を記録する
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::logLandingDetected(bool firm, bool stalled, uint32_t now_ms) const
{
    if (!firm && !stalled) {
        ESP_LOGI(TAG, "Landing detected (ToF dead zone: only invalid samples for %lu ms after a valid %.3f m)",
                 static_cast<unsigned long>(now_ms - dead_zone_valid_ms_), static_cast<double>(dead_zone_valid_m_));
        return;
    }
    ESP_LOGI(TAG, "Landing detected (%s)", firm ? "ground" : "stalled descent / ground effect");
}

// -----------------------------------------------------------------------------
// trackDeadZone — evidence for the dead-zone touchdown from one ToF sample
// 不感帯での接地の証拠 — ToF サンプル 1 つから
//
// A valid reading inside the band (re)starts the count; an invalid sample keeps it; a valid reading ABOVE the band
// (the craft is not at the floor, or the sensor sees something else) cancels it. Only the landing descent arms it.
// 帯の中の有効な読みは数え直しを始める。無効サンプルは保持する。帯より「上」の有効な読み（機体は床にいない、または
// センサが別のものを見ている）は取り消す。着陸降下の間だけ始める。
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::trackDeadZone(const TofData& tof, bool in_landing_descent, uint32_t now_ms)
{
    if (!in_landing_descent) {
        dead_zone_armed_ = false;
        return;
    }
    if (!tof.valid) {
        return;   // an invalid sample neither arms nor cancels / 無効サンプルは始めも取り消しもしない
    }
    if (tof.distance <= config_.dead_zone_band_m) {
        dead_zone_armed_    = true;
        dead_zone_valid_ms_ = now_ms;
        dead_zone_valid_m_  = tof.distance;
    } else {
        dead_zone_armed_ = false;
    }
}

// -----------------------------------------------------------------------------
// markAirborne — flip the raw ground-contact flag to the safe side (ToF-silence episode, armed)
// 空中化 — 生の接地フラグを安全側へ倒す（ARM 中の ToF 出力停止エピソード）
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::markAirborne()
{
    if (on_ground_) {
        ESP_LOGW(TAG, "Ground contact unknown (ToF silent while armed): flag set to airborne");
    }
    on_ground_     = false;
    ground_streak_ = 0;       // the air->ground debounce restarts from zero / 空中→接地のデバウンスはゼロから
    near_ground_   = false;   // the near-ground band is also unknown / 接地近傍帯も不明
    landing_start_ms_ = 0;    // a firm-ground landing timer must not survive the evidence it stood on / 接地着陸のタイマーは、拠り所の証拠とともに破棄する
    stall_start_ms_   = 0;    // the descent-stall timer likewise / 降下停滞のタイマーも同様
    dead_zone_armed_  = false;   // the dead-zone evidence is also a ToF fact / 不感帯の証拠も ToF の事実
    rest_run_active_  = false;   // the stillness of the ToF readings is a ToF fact as well / ToF の読みの静止も ToF の事実
}

// -----------------------------------------------------------------------------
// configureBlindTouchdown — apply the impact settings of the config to the detector
// ToF なしの接地の設定 — config の衝撃の設定を検出器へ反映
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::configureBlindTouchdown()
{
    blind_touchdown_.init(blindTouchdownConfig());
}

// -----------------------------------------------------------------------------
// blindTouchdownConfig — the detector thresholds that config_ sets (the rest: sf::limits defaults)
// 検出器の閾値のうち config_ が決めるもの（残りは sf::limits の既定）
// -----------------------------------------------------------------------------
BlindTouchdownConfig TakeoffLandingMgr::blindTouchdownConfig() const
{
    BlindTouchdownConfig detector_config;   // stillness settings: sf::limits defaults / 静止の設定は sf::limits の既定
    detector_config.impact_accel_mps2 = config_.blind_touchdown_accel_mps2;
    detector_config.impact_samples    = config_.blind_touchdown_samples;
    detector_config.floor_s           = config_.blind_floor_s;
    return detector_config;
}

// -----------------------------------------------------------------------------
// updateBlindTouchdown — touchdown while the ToF is silent: impact, then stillness
// ToF なしの接地 — ToF 出力停止中の接地: 衝撃のあとの静止
//
// Without ToF samples there is no height and no stalled-descent evidence. An impact alone (e.g. |accel| over 1.4 g for 4 samples) is not evidence: real flight logs show 1.56 g of vibration
// in hover and 2.28 g in aggressive flight. A craft held up by the floor, after an impact, is not
// accelerating, is not descending, and its commanded thrust cannot hold it: BlindTouchdownDetector
// latches only when all of them hold for a set time (blind_touchdown.hpp). The latch stays until the
// blind landing ends. The 3 G crash alert (Failsafe) still cuts the motors for a hard crash.
// ToF のサンプルが無いと高さも降下停滞の証拠も無い。衝撃だけ（例: |加速度| が 1.4 g を 4 サンプル超過）
// では証拠にならない: 実機ログではホバーで 1.56 g、激しい飛行で 2.28 g の振動がある。床に支えられた機体は、
// 衝撃の後、加速せず、降下せず、指令推力では支えられない。BlindTouchdownDetector はそれらが全部、一定時間
// 成り立ったときだけラッチする（blind_touchdown.hpp）。ラッチはToF なしの着陸が終わるまで保持する。
// 3 G の墜落アラート（Failsafe）は、激しい墜落ではモータを止める。
// -----------------------------------------------------------------------------
void TakeoffLandingMgr::updateBlindTouchdown(const BlindTouchdownInput& input, bool blind_landing)
{
    if (!blind_landing) {
        blind_touchdown_.reset();
        return;
    }
    const bool was_latched = blind_touchdown_.touchdown();
    blind_touchdown_.update(input);
    if (blind_touchdown_.touchdown() && !was_latched) {
        if (blind_touchdown_.byFloorSupport()) {
            ESP_LOGI(TAG, "Touchdown detected (the floor holds the craft up for %.2f s, no impact; thrust ratio %.2f, "
                          "vz %.2f m/s, ToF silent)",
                     static_cast<double>(blind_touchdown_.floorSeconds()),
                     static_cast<double>(input.thrust_ratio),
                     static_cast<double>(input.vertical_velocity_up));
        } else {
            ESP_LOGI(TAG, "Touchdown detected (impact, then still %.2f s; thrust ratio %.2f, vz %.2f m/s, ToF silent)",
                     static_cast<double>(blind_touchdown_.stillSeconds()),
                     static_cast<double>(input.thrust_ratio),
                     static_cast<double>(input.vertical_velocity_up));
        }
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
