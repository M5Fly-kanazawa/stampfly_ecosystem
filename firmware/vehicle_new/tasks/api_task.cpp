/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file api_task.cpp
 * @brief Tello-style network API task — UDP :8889 text commands
 *        Tello 風ネットワーク API タスク — UDP :8889 テキストコマンド
 *
 * Lets a Python program (or any UDP sender) fly the StampFly with the Tello
 * SDK text protocol (requirements §7: command receive / TelloAPI):
 *
 *   command                       enter SDK mode (gate for everything else)
 *   takeoff / land / emergency    flight verbs → api_command → StateTask
 *   up/down/left/right/forward/back <cm>, cw/ccw <deg>, go <x> <y> <z> <speed>
 *                                 relative moves → command_target → controller
 *   stop                          halt: re-target the current position
 *   battery? / height? / attitude? / speed?     telemetry queries
 *
 * Authority split (architecture §2): this task PARSES and reports facts; the
 * StateManager executes flight verbs (with all pre-arm gates), and the
 * controller tracks guidance targets (with the pilot-stick cancel rule). Moves
 * compose on the TARGET (not the estimate), Tello-style, so consecutive moves
 * do not accumulate estimation drift.
 *
 * Python プログラム（や任意の UDP 送信者）が Tello SDK のテキストプロトコルで
 * StampFly を飛ばせるようにする（requirements §7: コマンド受信 / TelloAPI）。
 * 権限分担（architecture §2）: 本タスクは解析して事実を報告するだけ。飛行 verb は
 * StateManager が（事前ゲート込みで）実行し、誘導目標は制御器が（スティック解除則
 * 込みで）追従する。移動は推定値でなく「目標」に対して合成（Tello 流）するため、
 * 連続移動で推定ドリフトが蓄積しない。
 *
 * SIL: the inert socket shim never receives UDP, so the scenario engine injects
 * command lines via sf_api_inject_line() — the SAME parser/executor runs (Code
 * Identity); only the byte transport is bypassed. Replies are always logged so
 * the expect gates (and hardware debugging) can read them.
 * SIL: ソケットシムは UDP を受信しないため、シナリオエンジンが sf_api_inject_line()
 * でコマンド行を注入する — パーサ/実行系は「同一コード」が走り（Code Identity）、
 * バイト輸送だけを迂回する。応答は常にログにも出す（expect ゲートと実機デバッグ用）。
 *
 * @publisher  api_command (flight verbs), command_target (guidance)
 * @subscriber system_mode, system_status, estimate_state, sensor_power, controller_status
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI      [OK]
 * @design architecture.md §5 — コマンドフロー UDP/API                    [OK]
 * @design architecture.md §3 — R11 Guidance topic 契約 (command_target)  [OK]
 */

#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cerrno>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "topics.hpp"
#include "config.hpp"
#include "flight_state.hpp"
#include "params.hpp"      // autotune applies gains live / 自動チューンのライブ適用
#include "autotune.hpp"    // onboard fit + tune / オンボード同定＋設計

static const char* TAG = "ApiTask";

namespace {

// =============================================================================
// Protocol constants / プロトコル定数
// =============================================================================

constexpr uint16_t kApiPort        = 8889;    // Tello SDK port / Tello SDK ポート
constexpr int      kPollMs         = 20;      // main loop poll / 主ループ周期
constexpr float    kMoveMinCm      = 10.0f;   // Tello: 20; we allow finer indoor moves
constexpr float    kMoveMaxCm      = 300.0f;  // single-move clamp [cm] / 1回の移動上限
constexpr float    kAltMinM        = 0.2f;    // target altitude floor / 目標高度下限
constexpr float    kAltMaxM        = 2.0f;    // target altitude ceiling (indoor) / 上限
constexpr float    kDefaultSpeed   = 0.3f;    // [m/s] guidance approach / 既定接近速度
// Takeoff altitude is no longer an API constant: the auto-takeoff (controller-owned,
// unified with manual RC) climbs to its target, and cmdTakeoff holds whatever altitude
// FLYING was reached at (decision ④, 2026-06-14).
// 離陸高度はもう API 定数ではない: 自動離陸（制御器所有・手動 RC と統一）が目標まで
// 上昇し、cmdTakeoff は FLYING 到達時の高度を保持する（確定④, 2026-06-14）。
constexpr float    kReachRadiusM   = 0.15f;   // move "reached" tolerance / 到達判定半径
constexpr float    kReachYawRad    = 0.17f;   // ~10 deg yaw tolerance / ヨー到達判定
constexpr uint32_t kTakeoffTimeoutMs = 12000;
constexpr uint32_t kLandTimeoutMs    = 20000;
constexpr float    kBattEmptyV     = 3.3f;    // 0% / 100% mapping for battery?
constexpr float    kBattFullV      = 4.2f;

// =============================================================================
// SIL / test injection — a tiny critical-section FIFO of command lines.
// The scenario engine calls sf_api_inject_line() from outside this task.
// SIL / テスト注入 — クリティカルセクション保護の小さな行 FIFO。
// シナリオエンジンがタスク外から sf_api_inject_line() を呼ぶ。
// =============================================================================

// Single-producer (scenario engine) / single-consumer (this task) lock-free
// ring: the producer writes the slot THEN publishes head with release order;
// the consumer acquires head before reading the slot. No mutex needed, and it
// works identically on hardware and the SIL host FreeRTOS shim.
// 単一生産者（シナリオエンジン）/ 単一消費者（本タスク）のロックフリーリング:
// 生産者はスロットを書いてから head を release 順序で公開し、消費者は head を
// acquire してからスロットを読む。ミューテックス不要で、実機と SIL ホストの
// FreeRTOS シムで同一に動く。
constexpr int kInjectSlots = 8;
constexpr int kInjectLen   = 64;
char g_inject[kInjectSlots][kInjectLen];
std::atomic<int> g_inject_head{0};   // write index / 書込位置
std::atomic<int> g_inject_tail{0};   // read index  / 読出位置

bool popInjected(char* out, size_t out_len)
{
    const int tail = g_inject_tail.load(std::memory_order_relaxed);
    if (tail == g_inject_head.load(std::memory_order_acquire)) {
        return false;
    }
    std::strncpy(out, g_inject[tail % kInjectSlots], out_len - 1);
    out[out_len - 1] = '\0';
    g_inject_tail.store(tail + 1, std::memory_order_release);
    return true;
}

// =============================================================================
// API session state / API セッション状態
// =============================================================================

bool        g_sdk_mode = false;       // "command" received / "command" 受信済み
int         g_sock = -1;              // UDP socket (HW) / UDP ソケット（実機）
sockaddr_in g_client = {};            // last sender (reply target) / 返信先
bool        g_have_client = false;

// Guidance target bookkeeping — moves compose on the TARGET (Tello-style).
// 誘導目標の帳簿 — 移動は「目標」に対して合成（Tello 流）。
float g_target_ned[3] = {0, 0, 0};
float g_target_yaw    = 0;
bool  g_target_valid  = false;

// -----------------------------------------------------------------------------
// reply — send the answer to the UDP client AND log it (SIL gates / debugging).
// reply — UDP クライアントへ返信し、ログにも出す（SIL ゲート・デバッグ用）。
// -----------------------------------------------------------------------------
void reply(const char* text)
{
    ESP_LOGI(TAG, "reply: %s", text);
    if (g_sock >= 0 && g_have_client) {
        ::sendto(g_sock, text, std::strlen(text), 0,
                 reinterpret_cast<sockaddr*>(&g_client), sizeof(g_client));
    }
}

// -----------------------------------------------------------------------------
// Topic helpers / トピックヘルパ
// -----------------------------------------------------------------------------
sf::FlightState currentState()
{
    return static_cast<sf::FlightState>(sf::system_mode.latest().state);
}

void publishApiVerb(sf::ApiCmd verb, sf::FlightMode mode = sf::FlightMode::POS_HOLD)
{
    sf::api_command.publish({static_cast<uint8_t>(verb),
                             static_cast<uint8_t>(mode),
                             static_cast<uint32_t>(esp_timer_get_time())});
}

void publishGuidance(float speed)
{
    sf::GuidanceTarget t{};
    t.position[0] = g_target_ned[0];
    t.position[1] = g_target_ned[1];
    t.position[2] = g_target_ned[2];
    t.yaw         = g_target_yaw;
    t.speed       = speed;
    t.mode        = 1;
    t.timestamp   = static_cast<uint32_t>(esp_timer_get_time());
    sf::command_target.publish(t);
}

// Wait until `pred` returns true, polling at the loop period. Returns false on
// timeout. The API task may block here — it owns no control-path deadline.
// pred が真になるまでループ周期でポーリング。タイムアウトで false。本タスクは
// 制御経路の締切を持たないためブロックしてよい。
template <typename Pred>
bool waitUntil(uint32_t timeout_ms, Pred pred)
{
    const uint32_t start = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    while (static_cast<uint32_t>(esp_timer_get_time() / 1000) - start < timeout_ms) {
        if (pred()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

// -----------------------------------------------------------------------------
// Flight verbs / 飛行 verb
// -----------------------------------------------------------------------------

void cmdTakeoff()
{
    const sf::FlightState st = currentState();
    if (st != sf::FlightState::IDLE_GROUND && st != sf::FlightState::ARMED_GROUND) {
        reply("error not on ground");
        return;
    }
    if (!sf::system_status.latest().calibrated) {
        reply("error calibrating - keep the craft still");
        return;
    }
    publishApiVerb(sf::ApiCmd::Takeoff, sf::FlightMode::POS_HOLD);

    // Wait for the auto-takeoff chain (ARM → TAKEOFF climb → FLYING).
    // 自動離陸の鎖（ARM → TAKEOFF 上昇 → FLYING）を待つ。
    if (!waitUntil(kTakeoffTimeoutMs,
                   [] { return currentState() == sf::FlightState::FLYING; })) {
        reply("error takeoff timeout");
        return;
    }

    // Seed the guidance target at the CURRENT pose — FLYING means the auto-takeoff
    // has already climbed to and captured its target altitude, so we simply hold it.
    // The climb routine and target altitude are now UNIFIED with manual RC takeoff
    // (the controller owns both, decision ④); the API no longer commands its own
    // takeoff height. Composing subsequent moves from the TARGET starts here.
    // 誘導目標を「現在」の姿勢で初期化する — FLYING は自動離陸が既に目標高度まで上昇・
    // 捕捉したことを意味するので、そのまま保持する。上昇ルーチンと目標高度は手動 RC 離陸と
    // 統一された（制御器が両方を所有, 確定④）。API は自前の離陸高度を指令しない。以降の
    // 移動合成はこの「目標」から始まる。
    const sf::StateEstimate est = sf::estimate_state.latest();
    g_target_ned[0] = est.position[0];
    g_target_ned[1] = est.position[1];
    g_target_ned[2] = est.position[2];   // hold the auto-takeoff altitude / 離陸後高度を保持
    {
        const float w = est.attitude[0], x = est.attitude[1];
        const float y = est.attitude[2], z = est.attitude[3];
        g_target_yaw = atan2f(2.0f * (w * z + x * y),
                              1.0f - 2.0f * (y * y + z * z));
    }
    g_target_valid = true;
    publishGuidance(kDefaultSpeed);
    reply("ok");
}

void cmdLand()
{
    if (!sf::isAirborne(currentState())) {
        reply("error not flying");
        return;
    }
    g_target_valid = false;   // guidance ends with the flight / 誘導は飛行と共に終了
    publishApiVerb(sf::ApiCmd::Land);
    if (waitUntil(kLandTimeoutMs,
                  [] { return currentState() == sf::FlightState::IDLE_GROUND; })) {
        reply("ok");
    } else {
        reply("error land timeout");
    }
}

// Relative move in the TELLO body frame (x fwd, y LEFT, z UP, cm), rotated into
// NED by the CURRENT TARGET yaw, composed onto the current target.
// Tello 機体座標（x前/y左/z上, cm）の相対移動。目標ヨーで NED へ回転し、現在の
// 目標へ合成する。
void cmdMove(float fwd_cm, float left_cm, float up_cm, float speed_mps)
{
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        reply("error not flying");
        return;
    }
    const float dist = sqrtf(fwd_cm * fwd_cm + left_cm * left_cm + up_cm * up_cm);
    if (dist < kMoveMinCm || dist > kMoveMaxCm) {
        reply("error out of range");
        return;
    }
    const float fwd_m  = fwd_cm * 0.01f;
    const float left_m = left_cm * 0.01f;
    const float up_m   = up_cm * 0.01f;

    const float cy = cosf(g_target_yaw), sy = sinf(g_target_yaw);
    g_target_ned[0] += cy * fwd_m - sy * (-left_m);   // N (body y = right = -left)
    g_target_ned[1] += sy * fwd_m + cy * (-left_m);   // E
    g_target_ned[2] -= up_m;                          // D (up = -D)

    // Altitude safety clamp / 高度安全クランプ
    if (-g_target_ned[2] < kAltMinM) g_target_ned[2] = -kAltMinM;
    if (-g_target_ned[2] > kAltMaxM) g_target_ned[2] = -kAltMaxM;

    publishGuidance(speed_mps);

    // Block until reached (Tello semantics) — generous timeout from the path
    // length, then verify the estimate is inside the tolerance sphere.
    // 到達までブロック（Tello 流）— 経路長から余裕あるタイムアウトを取り、推定が
    // 許容球内に入ったか検証する。
    const uint32_t timeout_ms =
        static_cast<uint32_t>((dist * 0.01f / speed_mps) * 1000.0f) + 6000;
    const bool reached = waitUntil(timeout_ms, [] {
        const sf::StateEstimate e = sf::estimate_state.latest();
        const float dn = e.position[0] - g_target_ned[0];
        const float de = e.position[1] - g_target_ned[1];
        const float dd = e.position[2] - g_target_ned[2];
        return sqrtf(dn * dn + de * de + dd * dd) < kReachRadiusM;
    });
    reply(reached ? "ok" : "error move timeout");
}

void cmdRotate(float delta_yaw_rad, float speed_unused)
{
    (void)speed_unused;
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        reply("error not flying");
        return;
    }
    g_target_yaw += delta_yaw_rad;
    while (g_target_yaw >  3.14159265f) g_target_yaw -= 6.2831853f;
    while (g_target_yaw < -3.14159265f) g_target_yaw += 6.2831853f;
    publishGuidance(kDefaultSpeed);

    const bool reached = waitUntil(8000, [] {
        const sf::StateEstimate e = sf::estimate_state.latest();
        const float w = e.attitude[0], x = e.attitude[1];
        const float y = e.attitude[2], z = e.attitude[3];
        float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
        float err = g_target_yaw - yaw;
        while (err >  3.14159265f) err -= 6.2831853f;
        while (err < -3.14159265f) err += 6.2831853f;
        return fabsf(err) < kReachYawRad;
    });
    reply(reached ? "ok" : "error rotate timeout");
}

void cmdStop()
{
    // Require an active API target, like move/rotate: once the pilot cancels
    // guidance (or a mode change does), the API must NOT silently re-grab control
    // with a stop/hover — the operator re-engages explicitly via takeoff/go (M-3).
    // move/rotate と同様に有効な API 目標を要求する: パイロットが誘導を解除（またはモード
    // 変更で解除）した後、API が stop/hover で黙って制御を奪い返してはならない — 操縦者が
    // takeoff/go で明示的に再係合する (M-3)。
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        reply("error not flying or guidance released - takeoff/go to re-engage");
        return;
    }
    // Halt: re-target the spot we are at right now (the cascade brakes for us).
    // 停止: いまいる位置を目標にし直す（ブレーキはカスケードが掛ける）。
    const sf::StateEstimate est = sf::estimate_state.latest();
    g_target_ned[0] = est.position[0];
    g_target_ned[1] = est.position[1];
    g_target_ned[2] = est.position[2];
    g_target_valid  = true;
    publishGuidance(kDefaultSpeed);
    reply("ok");
}

// -----------------------------------------------------------------------------
// Queries (Tello formats) / クエリ（Tello 形式）
// -----------------------------------------------------------------------------

void cmdQuery(const char* what)
{
    char buf[96];
    if (std::strcmp(what, "battery?") == 0) {
        const float v = sf::sensor_power.latest().voltage;
        int pct = static_cast<int>((v - kBattEmptyV) / (kBattFullV - kBattEmptyV) * 100.0f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        std::snprintf(buf, sizeof(buf), "%d", pct);
    } else if (std::strcmp(what, "height?") == 0) {
        const float alt_cm = -sf::estimate_state.latest().position[2] * 100.0f;
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(alt_cm));
    } else if (std::strcmp(what, "attitude?") == 0) {
        const sf::StateEstimate e = sf::estimate_state.latest();
        const float w = e.attitude[0], x = e.attitude[1];
        const float y = e.attitude[2], z = e.attitude[3];
        const float r2d = 57.29578f;
        const float roll  = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * r2d;
        float sinp = 2*(w*y - z*x);
        if (sinp > 1) { sinp = 1; }
        if (sinp < -1) { sinp = -1; }
        const float pitch = asinf(sinp) * r2d;
        const float yaw   = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * r2d;
        std::snprintf(buf, sizeof(buf), "pitch:%d;roll:%d;yaw:%d;",
                      static_cast<int>(pitch), static_cast<int>(roll),
                      static_cast<int>(yaw));
    } else if (std::strcmp(what, "speed?") == 0) {
        const sf::StateEstimate e = sf::estimate_state.latest();
        const float sp = sqrtf(e.velocity[0]*e.velocity[0] +
                               e.velocity[1]*e.velocity[1] +
                               e.velocity[2]*e.velocity[2]) * 100.0f;
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(sp));
    } else {
        std::snprintf(buf, sizeof(buf), "error unknown query");
    }
    reply(buf);
}

// -----------------------------------------------------------------------------
// cmdAutotune — onboard rate-loop autotune: stepped-sine sweep → plant fit →
// phase-margin PID design → LIVE application (params, not persisted).
// cmdAutotune — オンボード自動チューン: ステップドサイン掃引 → プラントフィット →
// 位相余裕 PID 設計 → ライブ適用（params。NVS には保存しない）。
//
// Safety gates / 安全ゲート:
//  - FLYING hover only; each frequency point is a bounded, clamped excitation.
//  - The result is applied ONLY if: the fit residual is small, the verified
//    margins meet the spec, every gain passes the param-table range check, and
//    Kp stays within [1/4, 4]x the flying gain (no wild jumps). Otherwise the
//    OLD gains remain untouched and the reply says why.
//  - Nothing is written to NVS — land and `param save` after a check flight.
// -----------------------------------------------------------------------------
// autotuneCue — publish a buzzer tone (audible over WiFi-only/solo use; the pilot
// can't watch the serial log). Played by NotifyTask, so it never blocks the sweep.
// autotuneCue — ブザー音を発行（無線/ソロ運用で耳で分かる）。NotifyTask が鳴らすので掃引を阻害しない。
void autotuneCue(sf::NotifyEvent ev)
{
    sf::notify_command.publish({static_cast<uint8_t>(ev),
                                static_cast<uint32_t>(esp_timer_get_time())});
}

void cmdAutotune(uint8_t axis, float wc, float pm_deg)
{
    if (currentState() != sf::FlightState::FLYING) {
        reply("error not flying");
        return;
    }
    // Audible start cue + a scope guard that fires the FAIL tone on ANY early return;
    // the success path sets tune_ok and plays the OK tone explicitly. So the pilot
    // hears "starting", then "ok" or "fail" without reading any log.
    // 開始音＋スコープガード: どの早期 return でも FAIL 音を鳴らす。成功時は tune_ok を立て OK 音を
    // 明示再生。ログを読まずに「開始→成功/失敗」が耳で分かる。
    autotuneCue(sf::NotifyEvent::AutotuneStart);
    bool tune_ok = false;
    struct EndCue {
        bool& ok;
        ~EndCue() { if (!ok) autotuneCue(sf::NotifyEvent::AutotuneFail); }
    } end_cue{tune_ok};
    static const char* kAxisName[3] = {"roll", "pitch", "yaw"};
    // X-quad spec inertia Ixx/Iyy/Izz [kg·m²] — used ONLY as the Nelder-Mead seed
    // (1/J) for fitPlant; the final plant is fit to measured data, so this is a
    // starting guess, not a flight parameter. The flight rate gains are
    // flight-measured (params.cpp), not inertia-derived, so there is no inertia
    // SSOT to alias here (code_review L-13).
    // X-quad 仕様慣性 Ixx/Iyy/Izz [kg·m²] — fitPlant の Nelder-Mead 初期種(1/J)としてのみ
    // 使用。最終プラントは実測データにフィットするため、これは初期推定であり飛行パラメータ
    // ではない。飛行レートゲインは実測移植値(params.cpp)で慣性導出ではないため、ここで別名に
    // すべき慣性 SSOT は存在しない (L-13)。
    static const float kSpecInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};
    static const float kFreqsHz[] = {2.0f, 3.0f, 4.5f, 7.0f, 10.0f,
                                     14.0f, 20.0f, 27.0f, 35.0f};
    constexpr int kNumFreqs = sizeof(kFreqsHz) / sizeof(kFreqsHz[0]);
    constexpr float kAmpRadps = 0.35f;   // ~20 dps per point / 点あたり約20dps

    sf::autotune::FreqPoint points[kNumFreqs];
    int collected = 0;
    uint32_t last_seq = sf::sysid_result.latest().seq;

    ESP_LOGI(TAG, "Autotune %s: sweeping %d frequencies...",
             kAxisName[axis], kNumFreqs);
    for (int i = 0; i < kNumFreqs; i++) {
        const float f = kFreqsHz[i];
        // settle 2 cycles + measure ~8 cycles, clamped to [0.8, 2.5] s
        // 整定2周期＋測定約8周期、[0.8, 2.5]s にクランプ
        float dur = 10.0f / f;
        if (dur < 0.8f) dur = 0.8f;
        if (dur > 2.5f) dur = 2.5f;

        sf::SysidCommand cmd{};
        cmd.axis      = axis;
        cmd.waveform  = 2;          // stepped sine / ステップドサイン
        cmd.amplitude = kAmpRadps;
        cmd.duration  = dur;
        cmd.frequency = f;
        cmd.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        sf::sysid_command.publish(cmd);

        // Wait for this point's result (seq edge), with a margin.
        // この点の結果（seq エッジ）を余裕付きで待つ。
        const bool got = waitUntil(
            static_cast<uint32_t>(dur * 1000.0f) + 1500,
            [last_seq] { return sf::sysid_result.latest().seq != last_seq; });
        if (!got || currentState() != sf::FlightState::FLYING) {
            reply("error autotune aborted (excitation failed or not flying)");
            return;
        }
        const sf::SysidFreqResult res = sf::sysid_result.latest();
        last_seq = res.seq;
        points[collected].w  = res.w;
        points[collected].ur = res.ur;
        points[collected].ui = res.ui;
        points[collected].yr = res.yr;
        points[collected].yi = res.yi;
        // Coherence/SNR proxy: on-tone gyro power / (on-tone + off-tone noise floor).
        // coh→1 = clean, coh→0 = disturbance-dominated. The coherence-weighted fit uses
        // this per-point weight to ignore tones a disturbance corrupts (e.g. low-freq yaw).
        // コヒーレンス/SNR代理: オン音電力/(オン音+オフ音雑音床)。coh→1清浄, coh→0外乱支配。
        const float on_pow = res.yr * res.yr + res.yi * res.yi;
        const float coh = (on_pow + res.off_power > 1e-20f)
                        ? on_pow / (on_pow + res.off_power) : 1.0f;
        points[collected].coh = coh;
        collected++;
        // Diagnostic: log the RAW measured open-loop point G(jw)=Y/U (|G| in dB + phase).
        // The fitted model can hide a structural mismatch; the raw Bode is the ground
        // truth. Visible over wired `sf monitor`. |G|=|Y|/|U|, arg G = argY − argU.
        // 診断: 実測の開ループ点 G(jw)=Y/U（|G| dB＋位相）をログ。フィットは構造不一致を隠すが
        // 生 Bode は真値。有線 sf monitor で見える。
        {
            const float um = sqrtf(res.ur * res.ur + res.ui * res.ui);
            const float ym = sqrtf(res.yr * res.yr + res.yi * res.yi);
            const float gmag = (um > 1e-9f) ? ym / um : 0.0f;
            float gph = atan2f(res.yi, res.yr) - atan2f(res.ui, res.ur);
            while (gph >  3.14159265f) gph -= 6.2831853f;
            while (gph < -3.14159265f) gph += 6.2831853f;
            ESP_LOGI(TAG, "  bode %s f=%5.1fHz |G|=%9.1f (%+6.1fdB) ph=%+6.1fdeg coh=%.2f",
                     kAxisName[axis], static_cast<double>(res.w / 6.2831853f),
                     static_cast<double>(gmag),
                     static_cast<double>(20.0f * log10f(gmag + 1e-12f)),
                     static_cast<double>(gph * 57.29578f),
                     static_cast<double>(coh));
        }
        // NOTE: do NOT re-cue here. An earlier version re-published AutotuneStart after
        // every point to keep the buzzer audible; but each cue plays a ~0.9 s BLOCKING
        // warble in NotifyTask, and the high-freq points (~0.8 s) out-paced it, FILLING
        // the depth-8 notify_command queue (which drops on overflow) — so the final
        // AutotuneOk/Fail cue was silently dropped and yaw showed white-then-nothing.
        // The white LED already persists for the whole sweep via its 30 s safety
        // timeout, so the end cue (green/red) just needs the queue clear here.
        // ここで再合図しない。旧版は各点で AutotuneStart を再発行したが、各合図は NotifyTask で
        // 約0.9秒のブロッキング・ワーブルを鳴らし、高周波点(約0.8秒)が追い越して深さ8の
        // notify_command キューを満杯化(満杯時ドロップ)→ 最後の Ok/Fail 合図が落ち、yaw が
        // 「白→無」になっていた。白LEDは開始時の30秒タイマで掃引中ずっと維持されるため、
        // 終了の緑/赤はキューを空けておけば確実に出る。
    }

    // Fit + tune (pure math, sf_autotune — host-tested).
    // フィット＋設計（純数学, sf_autotune — ホストテスト済み）。
    // Uniform 3-param fit for ALL axes. The fit is coherence-WEIGHTED (each point carries
    // a coh weight set in the sweep loop above), so a structured disturbance on one axis
    // (e.g. yaw) is down-weighted rather than corrupting the fit — the onboard mirror of
    // the offline coherence-weighted ETFE. (A yaw reaction-torque RHP zero was tried and
    // refuted on hardware: tau_z≈0; see yaw_axis_model.md.)
    // 全軸共通の3パラ同定（コヒーレンス重み付き）。構造的外乱は軽視され、フィットを汚さない。
    sf::autotune::Plant plant{};
    const bool fit_ok =
        sf::autotune::fitPlant(points, collected, 1.0f / kSpecInertia[axis], plant);
    ESP_LOGI(TAG, "Autotune fit: b=%.0f T=%.1fms L=%.2fms residual=%.3f",
             static_cast<double>(plant.b), static_cast<double>(plant.T * 1e3),
             static_cast<double>(plant.L * 1e3), static_cast<double>(plant.residual));

    // Persist the identified plant ALWAYS (even a poor/rejected fit) so the result is
    // readable over WiFi via `param get` for diagnosis. The fit-reject path used to
    // RETURN before saving, leaving stale params — so a rejected yaw looked unchanged.
    // Only the DESIGN below is gated on the residual.
    // 同定プラントを常に保存（粗い/棄却フィットも）— 棄却経路が保存前に return して古い値が
    // 残り、yaw が変化なしに見えていた。設計のみ残差でゲートする。
    if (plant.b > 0.0f) {
        char pk[40];
        std::snprintf(pk, sizeof(pk), "autotune.%s.b",     kAxisName[axis]); sf::params::set_float(pk, plant.b);
        std::snprintf(pk, sizeof(pk), "autotune.%s.tau",   kAxisName[axis]); sf::params::set_float(pk, plant.T);
        std::snprintf(pk, sizeof(pk), "autotune.%s.delay", kAxisName[axis]); sf::params::set_float(pk, plant.L);
        std::snprintf(pk, sizeof(pk), "autotune.%s.resid", kAxisName[axis]); sf::params::set_float(pk, plant.residual);
    }
    if (!fit_ok || plant.residual > 0.3f) {
        ESP_LOGW(TAG, "Autotune fit rejected: residual=%.3f (saved for diagnosis)",
                 static_cast<double>(plant.residual));
        reply("error fit residual too high - gains unchanged (read autotune.* params)");
        return;
    }


    // Save the margins of the CURRENT (active) gains scored against the freshly
    // identified plant — so the stored wc/pm/gm ALWAYS reflect what is ACTUALLY flying.
    // If the design below is APPLIED, these are overwritten with the new gains' margins;
    // if it is REJECTED, the current gains' margins remain (answers "how safe are the
    // gains I am still flying, really?" — exactly what a rejected axis needs).
    // 現（実効）ゲインの余裕を新同定プラントで採点して保存 — 保存 wc/pm/gm は常に「実際に
    // 飛んでいる」ゲインを反映。下の設計が適用されれば新ゲインの余裕で上書き、棄却されれば
    // 据置ゲインの余裕が残る（棄却軸が知りたい「今飛んでいるゲインの本当の安全余裕」）。
    {
        float cur_kp = 0, cur_ti = 0, cur_td = 0;
        char rk[32];
        std::snprintf(rk, sizeof(rk), "rate.%s.kp", kAxisName[axis]); sf::params::get_float(rk, cur_kp);
        std::snprintf(rk, sizeof(rk), "rate.%s.ti", kAxisName[axis]); sf::params::get_float(rk, cur_ti);
        std::snprintf(rk, sizeof(rk), "rate.%s.td", kAxisName[axis]); sf::params::get_float(rk, cur_td);
        sf::autotune::TuneResult cur{};
        sf::autotune::evalMargins(plant, cur_kp, cur_ti, cur_td, cur);
        char pk[32];
        std::snprintf(pk, sizeof(pk), "autotune.%s.wc", kAxisName[axis]); sf::params::set_float(pk, cur.wc);
        std::snprintf(pk, sizeof(pk), "autotune.%s.pm", kAxisName[axis]); sf::params::set_float(pk, cur.pm_deg);
        std::snprintf(pk, sizeof(pk), "autotune.%s.gm", kAxisName[axis]); sf::params::set_float(pk, cur.gm_valid ? cur.gm_db : 99.0f);
    }

    sf::autotune::TuneResult tune{};
    if (!sf::autotune::tunePid(plant, wc, pm_deg, 10.0f, tune) ||
        fabsf(tune.pm_deg - pm_deg) > 5.0f) {
        reply("error tune infeasible (lower wc/pm) - gains unchanged");
        return;
    }

    // (Design margins are saved as the CURRENT gains' margins above, and overwritten
    // with these NEW gains' margins ONLY if the design passes the gates and is applied
    // below — so a rejected axis keeps the margins of the gains it is still flying.)
    // （設計余裕は上で現ゲインの余裕として保存済み。下のゲートを通過し適用された場合のみ
    // この新ゲインの余裕で上書き — 棄却軸は据置ゲインの余裕を保持する。）

    // Gain-margin floor: a design that meets the phase margin can still be too
    // close to gain-side instability (thin GM), and model error / nonlinear
    // torque effectiveness then drives oscillation. Reject thin-GM designs
    // before LIVE apply (H-1). gm_valid=false means no −180° crossing in the
    // sweep → GM effectively infinite → SAFE, so do not reject that (L-15).
    // ALL axes use 6 dB. Yaw previously used 8 dB to compensate for its UNMODELLED
    // reaction-torque RHP zero (the margin was unreliable, so be extra conservative).
    // The zero is now modelled (fitPlantYaw + plantResponse) and the wc cap (0.3/tau_z)
    // keeps the design clear of the non-minimum-phase bandwidth limit, so the GM is
    // trustworthy and yaw needs no special floor — the old 8 dB only blocked a sound
    // design (the wc-capped yaw design lands ~7-8 dB).
    // 全軸 6dB。yaw は以前、未モデルの反トルク RHP 零点を補うため 8dB にしていた（余裕が当てに
    // ならず保守的に）。今は零点をモデル化し wc 上限(0.3/tau_z)で非最小位相の帯域限界を避けるため
    // GM は信頼でき、特別な下限は不要 — 旧 8dB は健全な設計(wc上限で~7-8dB)を弾くだけだった。
    static const float kMinGmDb[3] = {6.0f, 6.0f, 6.0f};   // roll, pitch, yaw [dB]
    if (tune.gm_valid && tune.gm_db < kMinGmDb[axis]) {
        ESP_LOGW(TAG, "Autotune %s gain margin %.1f dB < floor %.1f dB",
                 kAxisName[axis], static_cast<double>(tune.gm_db),
                 static_cast<double>(kMinGmDb[axis]));
        reply("error gain margin too low - gains unchanged");
        return;
    }

    // Sanity vs the flying gain: no wild jumps. / 飛行中ゲイン比の暴れ防止。
    char key_kp[32], key_ti[32], key_td[32];
    std::snprintf(key_kp, sizeof(key_kp), "rate.%s.kp", kAxisName[axis]);
    std::snprintf(key_ti, sizeof(key_ti), "rate.%s.ti", kAxisName[axis]);
    std::snprintf(key_td, sizeof(key_td), "rate.%s.td", kAxisName[axis]);
    float kp_now = 0;
    sf::params::get_float(key_kp, kp_now);
    if (kp_now > 0 && (tune.kp < 0.25f * kp_now || tune.kp > 4.0f * kp_now)) {
        ESP_LOGW(TAG, "Autotune kp %.3e outside [0.25,4]x current %.3e",
                 static_cast<double>(tune.kp), static_cast<double>(kp_now));
        reply("error gains out of safe range - gains unchanged");
        return;
    }

    // Apply LIVE (set_float validates the table ranges and fires ReloadParams).
    // ライブ適用（set_float がテーブル範囲を検証し ReloadParams を発火）。
    if (!sf::params::set_float(key_kp, tune.kp) ||
        !sf::params::set_float(key_ti, tune.ti) ||
        !sf::params::set_float(key_td, tune.td)) {
        reply("error param range rejected - check param table");
        return;
    }
    // Applied: the NEW gains are now the active gains, so overwrite the saved margins
    // (which held the OLD gains' margins) with this design's margins.
    // 適用済み: 新ゲインが実効ゲインになったので、保存余裕（旧ゲイン分）を本設計の余裕で上書き。
    {
        char pk[32];
        std::snprintf(pk, sizeof(pk), "autotune.%s.wc", kAxisName[axis]); sf::params::set_float(pk, tune.wc);
        std::snprintf(pk, sizeof(pk), "autotune.%s.pm", kAxisName[axis]); sf::params::set_float(pk, tune.pm_deg);
        std::snprintf(pk, sizeof(pk), "autotune.%s.gm", kAxisName[axis]);
        sf::params::set_float(pk, tune.gm_valid ? tune.gm_db : 99.0f);
    }
    // Show "inf" for an undetected (effectively infinite) GM rather than the
    // misleading 0.0 the raw field carries when gm_valid is false (L-15).
    // GM 未検出（実質無限大）は raw フィールドの紛らわしい 0.0 でなく "inf" と表示する(L-15)。
    char gm_str[16];
    if (tune.gm_valid) {
        std::snprintf(gm_str, sizeof(gm_str), "%.1f", static_cast<double>(tune.gm_db));
    } else {
        std::snprintf(gm_str, sizeof(gm_str), "inf");
    }
    ESP_LOGI(TAG, "Autotune applied: %s kp=%.4e ti=%.3f td=%.4f "
                  "(wc=%.1f pm=%.1f gm=%sdB)",
             kAxisName[axis], static_cast<double>(tune.kp),
             static_cast<double>(tune.ti), static_cast<double>(tune.td),
             static_cast<double>(tune.wc), static_cast<double>(tune.pm_deg),
             gm_str);
    char buf[120];
    std::snprintf(buf, sizeof(buf),
                  "ok kp=%.4e ti=%.3f td=%.4f wc=%.1f pm=%.1f gm=%s",
                  static_cast<double>(tune.kp), static_cast<double>(tune.ti),
                  static_cast<double>(tune.td), static_cast<double>(tune.wc),
                  static_cast<double>(tune.pm_deg), gm_str);
    tune_ok = true;                                 // suppress the guard's FAIL tone
    autotuneCue(sf::NotifyEvent::AutotuneOk);       // success chime
    reply(buf);
}

// -----------------------------------------------------------------------------
// processLine — parse + execute one command line (shared HW / SIL path).
// processLine — 1 コマンド行の解析＋実行（実機 / SIL 共通経路）。
// -----------------------------------------------------------------------------
void processLine(char* line)
{
    // Trim trailing whitespace/newlines / 末尾の空白・改行を除去
    size_t n = std::strlen(line);
    while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r' || line[n-1] == ' ')) {
        line[--n] = '\0';
    }
    if (n == 0) return;
    ESP_LOGI(TAG, "cmd: %s", line);

    if (std::strcmp(line, "command") == 0) {
        g_sdk_mode = true;
        reply("ok");
        return;
    }
    if (!g_sdk_mode) {
        reply("error not in command mode");
        return;
    }

    // Emergency first — never behind any other gate.
    // emergency は最優先 — 他のゲートの後ろに置かない。
    if (std::strcmp(line, "emergency") == 0) {
        g_target_valid = false;
        publishApiVerb(sf::ApiCmd::Emergency);
        reply("ok");
        return;
    }
    if (std::strcmp(line, "takeoff") == 0) { cmdTakeoff(); return; }
    if (std::strcmp(line, "land") == 0)    { cmdLand();    return; }
    if (std::strcmp(line, "stop") == 0)    { cmdStop();    return; }
    if (std::strchr(line, '?') != nullptr) { cmdQuery(line); return; }

    // Moves: <verb> <cm> / 移動: <verb> <cm>
    char verb[16] = {};
    float a = 0, b = 0, c = 0, d = 0;
    const int got = std::sscanf(line, "%15s %f %f %f %f", verb, &a, &b, &c, &d);
    const float deg2rad = 0.017453293f;
    if (got == 2) {
        if (std::strcmp(verb, "up") == 0)      { cmdMove(0, 0,  a, kDefaultSpeed); return; }
        if (std::strcmp(verb, "down") == 0)    { cmdMove(0, 0, -a, kDefaultSpeed); return; }
        if (std::strcmp(verb, "left") == 0)    { cmdMove(0,  a, 0, kDefaultSpeed); return; }
        if (std::strcmp(verb, "right") == 0)   { cmdMove(0, -a, 0, kDefaultSpeed); return; }
        if (std::strcmp(verb, "forward") == 0) { cmdMove( a, 0, 0, kDefaultSpeed); return; }
        if (std::strcmp(verb, "back") == 0)    { cmdMove(-a, 0, 0, kDefaultSpeed); return; }
        // Tello: cw = clockwise (viewed from above) = +yaw in NED.
        // Tello: cw = 上から見て時計回り = NED では +yaw。
        if (std::strcmp(verb, "cw") == 0)      { cmdRotate( a * deg2rad, 0); return; }
        if (std::strcmp(verb, "ccw") == 0)     { cmdRotate(-a * deg2rad, 0); return; }
    }
    // autotune <roll|pitch|yaw> [wc_radps] [pm_deg] — onboard autotune.
    // autotune <軸> [ωc] [PM] — オンボード自動チューン。
    if (std::sscanf(line, "autotune %15s %f %f", verb, &a, &b) >= 1 &&
        std::strncmp(line, "autotune", 8) == 0) {
        uint8_t axis;
        if      (std::strcmp(verb, "roll") == 0)  axis = 0;
        else if (std::strcmp(verb, "pitch") == 0) axis = 1;
        else if (std::strcmp(verb, "yaw") == 0)   axis = 2;
        else { reply("error bad axis"); return; }
        // Default wc: 25 rad/s for roll/pitch; 18 for yaw — yaw's reaction-torque RHP
        // zero limits its bandwidth, so a lower default keeps the design predictable
        // (the in-fit wc cap rarely has to bind). An explicit wc arg overrides either.
        // 既定 wc: roll/pitch=25、yaw=18（反トルク RHP 零点が帯域制限。低めの既定で予測的）。
        const float wc_def = (axis == 2) ? 18.0f : 25.0f;
        const float wc = (a > 1.0f && a < 100.0f) ? a : wc_def;
        const float pm = (b > 20.0f && b < 80.0f) ? b : 60.0f;
        cmdAutotune(axis, wc, pm);
        return;
    }

    // sysid <roll|pitch|yaw> <doublet|chirp> <amp_dps> <dur_s> — rate-loop
    // identification excitation while hovering (see SysidCommand). Blocks for
    // the duration so the calling script naturally brackets its log capture.
    // sysid … — ホバー中のレートループ同定励振（SysidCommand 参照）。継続時間
    // ブロックするため、呼び出しスクリプトのログ取得区間と自然に揃う。
    char wave[16] = {};
    if (std::sscanf(line, "sysid %15s %15s %f %f", verb, wave, &a, &b) == 4) {
        if (currentState() != sf::FlightState::FLYING) {
            reply("error not flying");
            return;
        }
        uint8_t axis;
        if      (std::strcmp(verb, "roll") == 0)  axis = 0;
        else if (std::strcmp(verb, "pitch") == 0) axis = 1;
        else if (std::strcmp(verb, "yaw") == 0)   axis = 2;
        else { reply("error bad axis"); return; }
        uint8_t waveform;
        if      (std::strcmp(wave, "doublet") == 0) waveform = 0;
        else if (std::strcmp(wave, "chirp") == 0)   waveform = 1;
        else { reply("error bad waveform"); return; }

        sf::SysidCommand cmd{};
        cmd.axis      = axis;
        cmd.waveform  = waveform;
        cmd.amplitude = a * 0.017453293f;   // dps → rad/s
        cmd.duration  = b;
        cmd.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        sf::sysid_command.publish(cmd);

        const uint32_t wait_ms = static_cast<uint32_t>(b * 1000.0f) + 1000;
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        reply("ok");
        return;
    }

    if (got == 5 && std::strcmp(verb, "go") == 0) {
        // go x y z speed — x fwd, y left, z up [cm], speed [cm/s] (Tello frame)
        float speed = d * 0.01f;
        if (speed < 0.1f) speed = 0.1f;
        if (speed > 1.0f) speed = 1.0f;
        cmdMove(a, b, c, speed);
        return;
    }
    reply("error unknown command");
}

}  // namespace

// =============================================================================
// SIL / test injection entry point (C linkage for the scenario engine).
// SIL / テスト注入の入口（シナリオエンジン用 C リンケージ）。
// =============================================================================
extern "C" void sf_api_inject_line(const char* line)
{
    const int head = g_inject_head.load(std::memory_order_relaxed);
    if (head - g_inject_tail.load(std::memory_order_acquire) >= kInjectSlots) {
        return;   // ring full — drop (test feeder paces itself) / 満杯は破棄
    }
    std::strncpy(g_inject[head % kInjectSlots], line, kInjectLen - 1);
    g_inject[head % kInjectSlots][kInjectLen - 1] = '\0';
    g_inject_head.store(head + 1, std::memory_order_release);
}

void ApiTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "ApiTask started");

    // UDP server socket (non-blocking; the loop also drains SIL injections).
    // On the SIL host the shim socket simply never delivers datagrams.
    // UDP サーバソケット（ノンブロッキング。ループは SIL 注入も drain する）。
    // SIL ホストではシムのソケットにデータグラムが届かないだけ。
    g_sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock >= 0) {
        sockaddr_in addr = {};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(kApiPort);
        if (::bind(g_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ESP_LOGW(TAG, "API bind :%u failed (errno=%d) — UDP disabled",
                     kApiPort, errno);
            ::close(g_sock);
            g_sock = -1;
        } else {
            const int flags = ::fcntl(g_sock, F_GETFL, 0);
            ::fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);
            ESP_LOGI(TAG, "Tello-style API listening on UDP :%u", kApiPort);
        }
    } else {
        ESP_LOGW(TAG, "API socket() failed (errno=%d) — UDP disabled", errno);
    }

    char line[kInjectLen];
    // Tracks whether we have OBSERVED the controller engage our guidance target.
    // Only after seeing it active can a later inactive reading mean the controller
    // CANCELLED guidance (pilot stick / mode change) — this avoids a false release
    // in the window between setting a target and the controller engaging it (M-3).
    // 制御器が誘導目標を係合したのを「観測済み」かを追跡する。active を見た後に inactive を
    // 読んで初めて制御器が誘導を解除した（スティック/モード変更）と判断できる — 目標設定から
    // 制御器係合までの窓での誤解放を防ぐ (M-3)。
    bool guidance_seen_active = false;
    while (true) {
        // 0. Guidance-cancel sync (M-3): the controller self-cancels guidance on
        // pilot stick movement or a mode change. Release our API target on the
        // FALLING edge so a subsequent move/stop/rotate is rejected until the
        // operator re-engages with takeoff/go — keeps "the pilot always wins".
        // 0. 誘導解除の同期 (M-3): 制御器はスティック動作やモード変更で誘導を自発解除する。
        // 立下りエッジで API 目標を解放し、operator が takeoff/go で再係合するまで後続の
        // move/stop/rotate を拒否する —「パイロット優先」を保つ。
        const bool ctrl_guidance = sf::controller_status.latest().guidance_active;
        if (ctrl_guidance) {
            guidance_seen_active = true;
        } else if (guidance_seen_active && g_target_valid) {
            g_target_valid = false;
            guidance_seen_active = false;
            ESP_LOGI(TAG, "Guidance released (pilot override / mode change) — "
                          "API target dropped; takeoff/go to re-engage");
        }
        if (!g_target_valid) {
            guidance_seen_active = false;
        }

        // 0b. Scheduled autotune (solo pilot, hands-free): a single operator cannot
        // type `autotune` mid-flight. They set autotune.sched.axis/.delay on the
        // GROUND; here, once the craft has been FLYING for sched_delay seconds, fire
        // the SAME rate-loop autotune automatically. One-shot per flight; a beep cues
        // the pilot to hold a steady hover. The sweep blocks this task (~15-20 s) — the
        // control loop (control_task) keeps flying throughout; the operator just holds.
        // 0b. スケジュール autotune（ソロ操縦・ハンズフリー）: 地上で軸/遅延を設定し、FLYING
        // 到達から sched_delay 秒後に同じ autotune を自動起動。1飛行1回・ブザーで合図。掃引中は
        // 本タスクをブロックするが制御は control_task で継続、操縦者は定位置を保持するだけ。
        {
            static int64_t flying_since_us = 0;
            static bool    sched_fired     = false;
            if (currentState() != sf::FlightState::FLYING) {
                flying_since_us = 0;
                sched_fired     = false;
            } else {
                const int64_t now_us = esp_timer_get_time();
                if (flying_since_us == 0) flying_since_us = now_us;
                if (!sched_fired) {
                    int32_t ax    = -1;    sf::params::get_int("autotune.sched.axis", ax);
                    float   delay = 20.0f; sf::params::get_float("autotune.sched.delay", delay);
                    if (ax >= 0 && ax <= 2 &&
                        (now_us - flying_since_us) >= static_cast<int64_t>(delay * 1.0e6f)) {
                        sched_fired = true;   // one-shot BEFORE the blocking sweep
                        ESP_LOGI(TAG, "Scheduled autotune firing: axis=%ld after %.0fs FLYING",
                                 static_cast<long>(ax), static_cast<double>(delay));
                        cmdAutotune(static_cast<uint8_t>(ax),
                                    (ax == 2) ? 18.0f : 25.0f, 60.0f);  // yaw lower wc (RHP zero); plays tones
                    }
                }
            }
        }

        // 1. SIL/test injections / SIL・テスト注入
        while (popInjected(line, sizeof(line))) {
            processLine(line);
        }

        // 2. UDP datagrams (one command per datagram, Tello-style)
        // 2. UDP データグラム（1 データグラム = 1 コマンド、Tello 流）
        if (g_sock >= 0) {
            sockaddr_in from = {};
            socklen_t from_len = sizeof(from);
            // MSG_DONTWAIT (per-call non-blocking): the loop must keep draining
            // SIL injections; a parked blocking receive would starve them.
            // MSG_DONTWAIT（呼び出し単位の非ブロッキング）: ループは SIL 注入の
            // drain を続ける必要があり、ブロッキング受信のパークはそれを飢餓させる。
            const ssize_t r = ::recvfrom(g_sock, line, sizeof(line) - 1, MSG_DONTWAIT,
                                         reinterpret_cast<sockaddr*>(&from),
                                         &from_len);
            if (r > 0) {
                line[r] = '\0';
                g_client = from;          // reply to the latest sender / 最新送信者へ返信
                g_have_client = true;
                processLine(line);
                continue;                 // drain quickly when commands queue up
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}
