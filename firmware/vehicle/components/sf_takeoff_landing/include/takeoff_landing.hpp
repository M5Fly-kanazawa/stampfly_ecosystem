/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_landing.hpp
 * @brief Takeoff/landing manager — ground/airborne detection, sequences
 *        離着陸マネージャー — 地上/空中判定、シーケンス管理
 *
 * Uses ToF sensor data and state estimation to detect ground contact,
 * manage takeoff/landing sequences, and trigger state transitions.
 *
 * ToFセンサデータと状態推定を使用して地面接地を検出し、
 * 離陸/着陸シーケンスを管理し、状態遷移をトリガーする。
 *
 * @design architecture.md §4 — State machine transitions               [OK]
 * @design architecture.md INV-3 — detection here, decision in StateTask [OK]
 * @design detailed_design.md §3 — State transition table (TAKEOFF/LANDING) [OK]
 * @design detailed_design.md §3 注7 — stalled-descent touchdown detector [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "blind_touchdown.hpp"   // sf::BlindTouchdownDetector (impact, then stillness)
#include "flight_state.hpp"
#include "data_types.hpp"   // sf::TofData
#include "flight_limits.hpp"  // blind-touchdown thresholds
#include <cstdint>

namespace sf {

/// Takeoff/landing configuration
/// 離着陸設定
struct TakeoffLandingConfig {
    float ground_tof_m       = limits::kTakeoffGroundBandM;   // Ground threshold [m]    / 地上閾値
    float airborne_tof_m     = limits::kTakeoffAirborneBandM;   // Airborne threshold [m]  / 空中閾値
    float landing_vel_mps    = 0.05f;   // Landing velocity [m/s]  / 着陸速度
    uint32_t landing_hold_ms = 700;     // Firm-ground confirm [ms] (was 1000: faster disarm once settled) / 接地確認時間
    // Stalled-descent touchdown ("ground-effect float"). During a commanded landing
    // descent, near the ground the rotors make MORE lift (ground effect), so the craft
    // holds altitude on reduced thrust and the constant-velocity descent STALLS above the
    // 5cm ground threshold — a ToF-only detector never triggers and the craft floats
    // (pilot report 2026-06-14). Touchdown is then declared from KINEMATICS: while a
    // landing descent is commanded AND the ToF reads within near_ground_tof_m AND the
    // vertical velocity has stalled (< landing_vel_mps) — we are trying to descend but the
    // ground has stopped us. No thrust threshold (a fragile, hover-sensitive signal) is
    // needed; the stalled descent IS the weight-on-ground evidence.
    // 降下停滞による接地（「地面効果フロート」）。着陸降下の指令中、接地近傍ではローター揚力が
    // 増す（地面効果）ため機体は低推力で高度を保ち、一定速度降下が 5cm 地上閾値の上で停滞する
    // — ToF のみの検出器は作動せず機体は浮く（パイロット報告 2026-06-14）。そこで接地を
    // キネマティクスから宣言: 着陸降下が指令されており ∧ ToF が near_ground_tof_m 以内 ∧
    // 鉛直速度が停滞（< landing_vel_mps）＝降下しようとしているのに地面に止められている。脆い
    // （ホバー感度の高い）推力閾値は不要で、降下停滞そのものが地面支持の証拠。
    float near_ground_tof_m  = 0.15f;   // Near-ground band for stalled-descent touchdown [m] (was 0.12: engage the settle a touch earlier) / 停滞接地の近傍帯
    uint32_t stall_hold_ms   = 400;     // Stalled-descent confirm [ms] (was 600: with the settle assist the descent reaches rest quickly) / 停滞確認時間
    // Consecutive ground-side ToF samples required to flip on_ground_ false→true
    // (air→ground debounce). A single near-reflection ghost mid-flight can no
    // longer assert "on ground"; landing already needs landing_hold_ms downstream
    // so the 2-sample (~66ms at 30Hz) delay is negligible (M-6).
    // NOTE: the ground→air flip (takeoff) is INTENTIONALLY immediate — the ESKF
    // vertical handoff keys off that edge and is a sample-precision class-B reset
    // (architecture.md §4); debouncing it delays the handoff and degrades the
    // take-off transient (measured: duty_max 0.80 → 1.0 with a 2-sample delay).
    // on_ground_ を false→true に反転させるのに必要な地上側連続 ToF サンプル数
    // （air→ground デバウンス）。飛行中の単発近距離ゴーストで「接地」を主張できなくする。
    // 着陸は下流に landing_hold_ms があるので2サンプル(≈66ms)の遅延は無視できる (M-6)。
    // 注: ground→air（離陸）の反転は意図的に即時 — ESKF 鉛直ハンドオフがこのエッジを
    // 起点とするサンプル精度のクラスB reset（architecture.md §4）であり、デバウンスすると
    // ハンドオフが遅れ離陸過渡が劣化する（実測: 2サンプル遅延で duty_max 0.80→1.0）。
    uint8_t flip_confirm_samples = 2;

    // Touchdown in the ToF DEAD ZONE: the sensor reads "no target" below about 0.031 m, so the last centimetres of a
    // landing are INVALID samples (a soft touchdown, under 3 G, would otherwise never be detected: on_ground_ needs
    // valid readings below 5 cm). During the landing descent, a valid reading at or below dead_zone_band_m followed by
    // ONLY invalid samples for dead_zone_hold_ms means the craft is on the floor. CANDIDATE values: flight_limits.hpp.
    // ToF の「不感帯」での接地: センサは約 0.031 m より近いと「目標なし」を返すので、着陸の最後の数 cm は無効サンプルに
    // なる（3 G 未満の柔らかい接地は、on_ground_ が 5 cm 未満の有効な読みを要るため検出されない）。着陸降下中に、
    // dead_zone_band_m 以下の有効な読みの後、無効サンプルだけが dead_zone_hold_ms 続いたら、機体は床の上にある。
    // 候補値は flight_limits.hpp にある。
    float    dead_zone_band_m   = limits::kLandingDeadZoneBandM;    ///< Valid reading at or below this arms the test [m] / これ以下の有効な読みで判定を始める [m]
    uint32_t dead_zone_hold_ms  = limits::kLandingDeadZoneHoldMs;   ///< Invalid samples only for this long = on the floor [ms] / 無効サンプルだけがこの時間 = 床の上 [ms]

    // Touchdown while the downward ToF is SILENT (no height, no stalled-descent evidence): an IMPACT
    // (|accel| above blind_touchdown_accel_mps2 for blind_touchdown_samples consecutive IMU samples)
    // followed by STILLNESS (not accelerating, thrust too low to hold the craft, vertical velocity ~0)
    // for a set time; see blind_touchdown.hpp. CANDIDATE values live in flight_limits.hpp.
    // 下向き ToF が「出力停止」のとき（高さも降下停滞の証拠も無い）の接地: 「衝撃」（|加速度| が
    // blind_touchdown_accel_mps2 を blind_touchdown_samples サンプル連続で超える）の後に、一定時間の「静止」
    // （加速していない、機体を支えられない推力、鉛直速度がほぼ 0）。blind_touchdown.hpp 参照。
    // 候補値は flight_limits.hpp にある。
    float    blind_touchdown_accel_mps2 = limits::kBlindTouchdownAccelMps2;
    uint16_t blind_touchdown_samples    = limits::kBlindTouchdownSamples;
    float    blind_floor_s              = limits::kBlindFloorS;   ///< Second path: the floor holds the craft up this long [s] / 2 つ目の経路: 床が機体を支える時間 [s]
};

/// Takeoff/landing manager
/// 離着陸マネージャー
class TakeoffLandingMgr {
public:
    /// Initialize with default configuration
    /// デフォルト設定で初期化する
    void init();

    /// Initialize with custom configuration
    /// カスタム設定で初期化する
    void init(const TakeoffLandingConfig& config);

    /// Update detection logic with a fresh ToF sample (call when ToF data arrives).
    /// The owning task reads the sensor_tof topic ONCE and injects the sample here,
    /// so this manager does not compete with the estimator for the same queue.
    ///
    /// `armed` gates ground detection: while disarmed the craft is definitively on
    /// the ground (on the ground the ToF sits below its minimum range and returns
    /// invalid, so it cannot by itself confirm ground contact). This mirrors the
    /// proven firmware/vehicle landing handler ("landed whenever disarmed") and lets
    /// the manager re-anchor after landing and arm the next flight cleanly.
    /// 新しい ToF サンプルで検出ロジックを更新する（ToF データ到着時に呼ぶ）。
    /// 所有タスクが sensor_tof を1回だけ読んで注入するため推定器とキューを奪い合わない。
    /// `armed` で接地判定を制御: disarmed 中は確実に接地（接地中 ToF は最小レンジ未満で
    /// 無効を返し単独で接地を確認できない）。実証済みの firmware/vehicle（disarmed なら
    /// landed）と同じで、着地後の再錨付けと次飛行の混入のない開始を可能にする。
    /// `vertical_velocity` [m/s, NED down] is injected (not read from a topic) so the
    /// landing detector stays a pure function of its inputs — detection here, the
    /// transition decision in StateTask (architecture §2: separate detection from decision).
    /// `in_landing_descent` is true while the autonomous landing descent is commanded
    /// (FlightState::LANDING). It is a STATE-derived INPUT (like `armed`), not a decision —
    /// it enables the stalled-descent touchdown branch (a stall near the ground only means
    /// "landed" when we are actually trying to descend; a deliberate low hover is not) and the dead-zone
    /// touchdown branch (only invalid samples after a valid reading near the floor).
    /// `vertical_velocity`[m/s, NED down] を注入する（トピックから読まない）。着陸検出器を入力の
    /// 純粋な関数に保ち、検出はここ・遷移判断は StateTask（architecture §2: 検出と判断の分離）。
    /// `in_landing_descent` は自動着陸降下の指令中（FlightState::LANDING）に true。`armed` と同じ
    /// 状態由来の「入力」（判断ではない）で、降下停滞による接地判定を有効にする（接地近傍の停滞が
    /// 「着陸」を意味するのは実際に降下しようとしている時のみ。意図的な低ホバーは着陸でない）と、不感帯での接地の
    /// 判定（床の近くの有効な読みの後に無効サンプルだけ）を有効にする。
    void update(const TofData& tof, bool armed, float vertical_velocity,
                bool in_landing_descent);

    /// Replace the configuration at run time (a live parameter reload) WITHOUT resetting any detector state:
    /// the touchdown detector of the descent without a ToF (impact / floor thresholds) and the three landing paths use
    /// the new values from the next sample on.
    /// 設定を実行中に置き換える（パラメータのライブ再読込）。検出器の状態はリセットしない:
    /// ToF なし降下の接地検出器（衝撃・床の閾値）と 3 つの着陸経路は、次のサンプルから新しい値を使う。
    void reloadConfig(const TakeoffLandingConfig& config);

    /// Tell the manager, EVERY control cycle, whether the autonomous landing descent is commanded (FlightState::LANDING). On the rising edge the landing verdict is judged AFRESH: the level flag
    /// and the hold timers of the paths (1) and (2) are cleared, so only samples that arrive after LANDING began can end it. Without this a verdict left over from before (the hysteresis band between
    /// kTakeoffGroundBandM and kTakeoffAirborneBandM keeps the ground flag true at 0.05-0.15 m: a weak craft that rested there, or a hover held low) ended the LANDING in the cycle it began, and the motors
    /// stopped about 0.1 m above the floor (SILS, a low hover or a weak craft at 0.05-0.15 m, then a lost link). It is called every cycle rather than only from update() (a ToF sample every ~33 ms) so that the stale flag is gone before StateTask can read it.
    /// 自動着陸の降下が指令されているか（FlightState::LANDING）を、「毎制御周期」教える。立ち上がりで着陸の判定を「新たに」行う: 経路 (1) (2) のレベルフラグと保持タイマーを消し、
    /// LANDING が始まった後に届いたサンプルだけがそれを終わらせられる。これが無いと、以前から残った判定（kTakeoffGroundBandM と kTakeoffAirborneBandM の間のヒステリシス帯では接地フラグが
    /// 0.05〜0.15 m で true のまま: そこに居た弱い機体、低く保ったホバー）が、LANDING が始まった周期にそれを終わらせ、モータが床の約 0.1 m 上で止まった（SILS の、低いホバーや 0.05〜0.15 m にいる弱い機体の後のリンク途絶）。update()（約 33 ms ごとの
    /// ToF サンプル）からだけでなく毎周期呼ぶのは、古いフラグが StateTask に読まれる前に消えているようにするため。
    void noteLandingDescent(bool in_landing_descent);

    /// Check if vehicle is on the ground
    /// 機体が地上にあるか確認する
    bool isOnGround() const { return on_ground_; }

    /// Check if landing is detected (sustained low altitude + low vertical velocity).
    /// A LEVEL flag (true while the landing condition holds), not a one-shot pulse, so a
    /// slower consumer (StateTask at 50 Hz) cannot miss it through a Latest topic.
    /// 着陸を検出したか確認する（低高度＋低鉛直速度の持続）。ワンショットでなく条件成立中 true の
    /// レベルフラグ。低速の消費者（50Hz の StateTask）が Latest トピック越しに取りこぼさない。
    bool isLandingDetected() const { return landing_detected_; }

    /// Check if the vehicle is held in hand (disarmed + ToF reads a valid distance above
    /// the airborne threshold). Drives the IDLE_GROUND ↔ IDLE_HELD transition.
    /// 機体が手で持たれているか確認する（disarmed＋ToF が空中閾値以上の有効距離）。
    /// IDLE_GROUND ↔ IDLE_HELD 遷移を駆動する。
    bool isHeld() const { return held_; }

    /// Force the RAW ground-contact flag to "airborne". Called by ImuTask when a ToF-silence
    /// episode starts while ARMED: the flag is released by ToF samples only, so it froze at its
    /// last value, and a flag frozen "on the ground" kept the estimator pinning position/velocity
    /// to zero while the craft climbed (2026-11 accident). Nothing proves ground
    /// contact during the episode, so the flag is flipped to the safe side; it returns to "ground"
    /// only by the existing air->ground debounce (flip_confirm_samples consecutive VALID ground
    /// readings), never by an invalid or missing sample.
    /// 生の接地フラグを強制的に「空中」にする。ARM 中に ToF 出力停止のエピソードが始まったとき
    /// ImuTask が呼ぶ。フラグは ToF サンプルだけで解除されるため最後の値のまま固定され、「接地」のまま
    /// 固定されると機体が上昇しても推定器が位置・速度を 0 に固定し続けた（2026-11 の事故）。
    /// エピソード中は接地を示す証拠が無いので、フラグを安全側に倒す。「接地」へ
    /// 戻るのは既存の空中→接地デバウンス（有効な地上読み flip_confirm_samples 回連続）だけで、
    /// 無効なサンプルや欠測では戻らない。
    void markAirborne();

    /// Feed one IMU sample (400 Hz) to the blind-touchdown detector. `blind_landing` is true while
    /// the state is LANDING AND the downward ToF is silent; otherwise the detector is cleared.
    /// While blind, an impact followed by stillness (blind_touchdown.hpp) latches the flag until
    /// `blind_landing` ends. Unlike update(), this runs every IMU cycle, because a silent ToF
    /// delivers no samples to drive update().
    /// ToF なしの接地検出器へ IMU サンプル 1 つ（400 Hz）を与える。`blind_landing` は状態が LANDING
    /// かつ下向き ToF が出力停止のとき true で、そうでなければ検出器をクリアする。ToF なしの間、
    /// 衝撃の後の静止（blind_touchdown.hpp）が揃うと、`blind_landing` が終わるまでフラグをラッチする。
    /// update() と違い毎 IMU 周期に走る — 出力停止した ToF は update() を駆動するサンプルを届けないため。
    /// @param input Sample: cycle time, |accel|, thrust ratio, vertical velocity (blind_touchdown.hpp)
    ///              サンプル: 周期・|加速度|・推力比・鉛直速度（blind_touchdown.hpp）
    void updateBlindTouchdown(const BlindTouchdownInput& input, bool blind_landing);

    /// True when the blind-touchdown detector has latched (LANDING with a silent ToF).
    /// ToF なしの接地検出器がラッチしていれば true（ToF 出力停止中の LANDING）。
    bool isBlindTouchdown() const { return blind_touchdown_.touchdown(); }

private:
    /// Apply the impact settings of config_ to the blind-touchdown detector
    /// config_ の衝撃の設定をToF なしの接地検出器へ反映する
    void configureBlindTouchdown();

    /// The detector thresholds that config_ sets / config_ が決める検出器の閾値
    BlindTouchdownConfig blindTouchdownConfig() const;

    /// Evaluate ToF distance for ground contact
    /// ToF距離で地面接地を評価する
    void evaluateToF(const TofData& tof);

    /// Path (3) of detectLanding(): true while the dead-zone evidence holds for dead_zone_hold_ms in a landing descent
    /// detectLanding() の経路 (3): 着陸降下中、不感帯の証拠が dead_zone_hold_ms 続いている間 true
    bool deadZoneTouchdown(bool in_landing_descent, uint32_t now_ms) const;

    /// Log which path detected the landing (once, on the rising edge)
    /// 着陸を検出した経路を記録する（立ち上がりで 1 回）
    void logLandingDetected(bool firm, bool stalled, uint32_t now_ms) const;

    /// Track the dead-zone touchdown evidence from one ToF sample: the time of the last valid reading inside the band,
    /// kept only while only invalid samples follow it.
    /// ToF サンプル 1 つから不感帯での接地の証拠を追う: 帯の中の最後の有効な読みの時刻。それ以後が無効サンプル
    /// だけの間だけ保持する。
    void trackDeadZone(const TofData& tof, bool in_landing_descent, uint32_t now_ms);

    /// Detect landing: firm ground (ToF<5cm) OR a stalled landing descent near the ground (each with low vertical
    /// velocity sustained) OR the dead-zone touchdown (invalid samples only after a valid one in the band).
    /// Level flag, see getter.
    /// 着陸検出: 確実な接地（ToF<5cm）or 接地近傍で着陸降下が停滞（いずれも低鉛直速度の持続）or 不感帯での接地
    /// （帯の中の有効な読みの後に無効サンプルだけ）。レベルフラグ（getter 参照）。
    void detectLanding(float vertical_velocity, bool in_landing_descent, uint32_t now_ms);

    /// Evaluate held-in-hand from the ToF distance (disarmed only, hysteresis)
    /// ToF 距離から手持ちを評価する（disarmed 時のみ、ヒステリシス）
    void evaluateHeld(const TofData& tof);

    TakeoffLandingConfig config_;
    bool     on_ground_        = true;   // Ground contact flag (ToF<5cm) / 地面接地フラグ
    bool     near_ground_      = false;  // ToF within near_ground_tof_m  / 接地近傍フラグ
    bool     held_             = false;  // Held-in-hand flag    / 手持ちフラグ
    bool     landing_detected_ = false;  // Landing detected flag (level) / 着陸検出フラグ
    bool     was_landing_descent_ = false;   // The landing descent was commanded in the previous cycle (edge detector of noteLandingDescent) / 前の周期で着陸降下が指令されていた（noteLandingDescent のエッジ検出）
    uint32_t landing_start_ms_ = 0;      // Firm-ground landing timer start / 接地着陸タイマー開始
    uint32_t stall_start_ms_   = 0;      // Stalled-descent timer start  / 降下停滞タイマー開始
    bool     dead_zone_armed_  = false;  // A valid reading inside the band was seen and only invalid samples followed / 帯の中の有効な読みがあり、以後は無効サンプルだけ
    uint32_t dead_zone_valid_ms_ = 0;    // Time of that last valid reading [ms] / その最後の有効な読みの時刻 [ms]
    float    dead_zone_valid_m_  = 0.0f; // Its distance [m] (log only) / その距離 [m]（ログ用）
    BlindTouchdownDetector blind_touchdown_;   // Impact-then-stillness detector / 衝撃→静止の検出器
    uint8_t  ground_streak_    = 0;      // Consecutive ground-side ToF samples (air→ground debounce) / 地上側連続数
};

}  // namespace sf
