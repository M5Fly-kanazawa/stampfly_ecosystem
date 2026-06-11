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
 * @subscriber system_mode, system_status, estimate_state, sensor_power
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
constexpr float    kTakeoffAltM    = 0.8f;    // Tello-like takeoff hover height / 離陸後高度
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

    // Seed the guidance target at the hover point, then rise to the Tello-like
    // takeoff height. Composing from the TARGET starts here.
    // 誘導目標をホバー点で初期化し、Tello 風の離陸高度へ上げる。以降の合成は
    // この「目標」から始まる。
    const sf::StateEstimate est = sf::estimate_state.latest();
    g_target_ned[0] = est.position[0];
    g_target_ned[1] = est.position[1];
    g_target_ned[2] = -kTakeoffAltM;
    {
        const float w = est.attitude[0], x = est.attitude[1];
        const float y = est.attitude[2], z = est.attitude[3];
        g_target_yaw = atan2f(2.0f * (w * z + x * y),
                              1.0f - 2.0f * (y * y + z * z));
    }
    g_target_valid = true;
    publishGuidance(kDefaultSpeed);

    waitUntil(kTakeoffTimeoutMs, [] {
        const sf::StateEstimate e = sf::estimate_state.latest();
        return fabsf(-e.position[2] - kTakeoffAltM) < kReachRadiusM;
    });
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
    if (currentState() != sf::FlightState::FLYING) {
        reply("error not flying");
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
    while (true) {
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
