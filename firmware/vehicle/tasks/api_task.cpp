/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
 * StateManager が（事前判定込みで）実行し、誘導目標は制御器が（スティック解除則
 * 込みで）追従する。移動は推定値でなく「目標」に対して合成（Tello 流）するため、
 * 連続移動で推定ドリフトが蓄積しない。
 *
 * SILS: the inert socket shim never receives UDP, so the scenario engine injects
 * command lines via sf_api_inject_line() — the SAME parser/executor runs (Code
 * Identity); only the byte transport is bypassed. Replies are always logged so
 * the expect gates (and hardware debugging) can read them.
 * SILS: ソケットシムは UDP を受信しないため、シナリオエンジンが sf_api_inject_line()
 * でコマンド行を注入する — パーサ/実行系は「同一コード」が動き（Code Identity）、
 * バイト輸送だけを迂回する。応答は常にログにも出す（expect 判定と実機デバッグ用）。
 *
 * @publisher  api_command (flight verbs), command_target (guidance)
 * @subscriber system_mode, system_status, estimate_state, sensor_power, controller_status
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI      [OK]
 * @design architecture.md §5 — コマンドフロー UDP/API                    [OK]
 * @design architecture.md §3 — R11 Guidance topic 契約 (command_target)  [OK]
 * @design docs/plans/flip-maneuver-plan.md §4.4 — API `flip l/r/f/b`     [OK]
 * @design detailed_design.md §3 注13 — 待機中の emergency/stop 即時処理    [OK]
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
#include "esp_mac.h"        // esp_read_mac for `sn?` serial / シリアル用 MAC 読み
#include "lwip/sockets.h"

#include "topics.hpp"
#include "config.hpp"
#include "flight_limits.hpp"   // sf::limits::kAltitudeCeilingM (shared altitude ceiling)
#include "flight_state.hpp"
#include "tof_silence_policy.hpp"   // sf::flightModeNeedsAltitude (ALT_HOLD / POS_HOLD)
#include "params.hpp"        // autotune applies gains live / 自動チューンのライブ適用
#include "autotune.hpp"      // onboard fit + tune / オンボード同定＋設計
#include "tello_state.hpp"   // UDP:8890 state-string builder / 状態文字列ビルダ
#include "api_arguments.hpp" // range checks of the parsed numbers / 解析した数の範囲検査
#include "rotate_guidance.hpp" // guidance yaw target and wait of cw / ccw / cw・ccw の誘導ヨー目標と待ち時間
#include "sf_math.hpp"       // sf::math::wrapAngleRad, kPi

static const char* TAG = "ApiTask";

namespace {

// =============================================================================
// Protocol constants / プロトコル定数
// =============================================================================

constexpr uint16_t kApiPort        = 8889;    // Tello SDK port / Tello SDK ポート
constexpr int      kPollMs         = 20;      // main loop poll / 主ループ周期
// Poll period inside waitUntil [ms]: how often the predicate is evaluated AND the command
// sources are read, so an `emergency` / `stop` that arrives while a command blocks waits at
// most this long plus the processing time. The predicate period is 50 ms:
// 2.5 times the idle-loop latency (kPollMs = 20 ms), 1/240 of the 12 s takeoff timeout.
// (SILS only: its socket shim yields one tick per recvfrom, so the period reads 51 ms there.
// With 49 ms all 51 older scenarios were bit-identical to before; with 50 ms 11 API scenarios
// move by that 1 ms per poll and still pass.)
// waitUntil 内のポーリング周期 [ms]: 述語の評価とコマンド源の読み取りの周期。待機中に届いた
// `emergency` / `stop` の待ち時間は最大でこれ＋処理時間。述語周期は 50 ms:
// 待機していないときの遅れ（kPollMs = 20 ms）の 2.5 倍、離陸タイムアウト 12 s の 1/240。
// （SILS のみ: ソケットシムは recvfrom ごとに 1 tick 譲るので周期は 51 ms に見える。49 ms なら
// 既存 51 シナリオが変更前と完全一致、50 ms だと API 系 11 本が 1 回あたり 1 ms ずれる。全て PASS。）
constexpr int      kWaitPollMs     = 50;
// Capacity of the deferred-command queue: commands that arrive while another command is
// blocking, other than `emergency` / `stop`, wait here and run in arrival order afterwards.
// 8 = the injection ring size (kInjectSlots), larger than the lwIP UDP receive mailbox
// (default 6) the same commands would otherwise have queued in. CANDIDATE value.
// 後回しコマンド待ち行列の容量: 別のコマンドが待機中に届いた `emergency` / `stop` 以外の
// コマンドをここで待たせ、後で到着順に実行する。8 = 注入リングのサイズ（kInjectSlots）で、
// 同じコマンドが他に溜まりうる lwIP の UDP 受信メールボックス（既定 6）より大きい。候補値。
constexpr int      kDeferredSlots  = 8;
constexpr float    kMoveMinCm      = 10.0f;   // Tello: 20; we allow finer indoor moves
constexpr float    kMoveMaxCm      = 300.0f;  // single-move clamp [cm] / 1回の移動上限
constexpr float    kAltMinM        = 0.2f;    // target altitude floor / 目標高度下限
// Target altitude ceiling [m] — shared with the controller's auto-takeoff target through
// sf::limits. CANDIDATE value, basis in flight_limits.hpp.
// 目標高度の上限 [m] — 制御器の自動離陸目標と sf::limits で共有。
// 候補値、根拠は flight_limits.hpp。
constexpr float    kAltMaxM        = sf::limits::kAltitudeCeilingM;
constexpr float    kDefaultSpeed   = 0.3f;    // [m/s] guidance approach / 既定接近速度
// The takeoff altitude is not an API constant: the auto-takeoff (controller-owned, the same
// climb as a transmitter ARM) climbs to sf::limits::kAutoTakeoffTargetAltitudeM, and cmdTakeoff
// seeds its guidance target at that altitude, so the craft holds it even when the climb was
// declared complete by its time limit below the target.
// 離陸高度は API 定数ではない: 自動離陸（制御器所有・送信機の ARM と同じ上昇）が
// sf::limits::kAutoTakeoffTargetAltitudeM まで上昇し、cmdTakeoff は誘導目標をその高度で
// 初期化する。上昇が目標に届かないまま時間切れで完了した場合も、機体はその高度を保持する。
constexpr float    kReachRadiusM   = 0.15f;   // move "reached" tolerance / 到達判定半径
constexpr uint32_t kTakeoffTimeoutMs = 12000;
constexpr uint32_t kLandTimeoutMs    = 20000;
constexpr uint32_t kLandConfirmMs    = 500;     // wait for a land from ARMED_GROUND to take effect / ARMED_GROUND からの着陸が効くのを待つ上限
// Reply for a command whose number is not finite or outside its range.
// 数が有限でない、または範囲外のコマンドへの応答。
constexpr const char* kOutOfRangeReply = "error out of range";
// Reply for a wait that ended because the flight ended (see waitUntil), and for the lines that were parked behind it.
// 飛行が終わったために打ち切った待機（waitUntil 参照）と、その後ろに置かれていた行への応答。
constexpr const char* kFlightEndedReply          = "error flight ended";
constexpr const char* kFlightEndedCancelledReply = "error cancelled: flight ended";
constexpr float    kDegreesToRadians = sf::math::kPi / 180.0f;
constexpr float    kSpeedMinCmPerSec = 10.0f;    // Tello `speed` range [cm/s] / Tello `speed` の範囲
constexpr float    kSpeedMaxCmPerSec = 100.0f;
constexpr float    kGoSpeedMinMps    = 0.1f;     // `go` speed clamp [m/s] / `go` の速度クランプ
constexpr float    kGoSpeedMaxMps    = 1.0f;
constexpr uint32_t kSysidMarginMs    = 1000;     // wait after the excitation ends [ms] / 励振の終了後に待つ時間
constexpr float    kBattEmptyV     = 3.3f;    // 0% / 100% mapping for battery?
constexpr float    kBattFullV      = 4.2f;

// Tello `rc a b c d` full-scale (±100) → velocity command. Horizontal and vertical
// reuse the SAME params the POS_HOLD stick path uses (single source — read live in
// cmdRc); the yaw turn-rate matches the controller's guidance yaw limit.
// Tello `rc a b c d` の満舵（±100）→ 速度指令。水平/鉛直は POS_HOLD スティック経路と同じ
// param を再利用（単一源 — cmdRc でライブ取得）。ヨー回頭率は制御器の誘導ヨー上限に合わせる。
constexpr float    kRcHorizVelDef  = 0.4f;    // [m/s] fallback if param missing / param 既定
constexpr float    kRcClimbRateDef = 0.5f;    // [m/s] up   fallback / 上昇 既定
constexpr float    kRcDescentRateDef = 0.5f;  // [m/s] down fallback / 降下 既定
constexpr float    kRcYawRateMax   = sf::rotate_guidance::kYawRateLimitRadPerSec;  // [rad/s] at ±100 (= guide_yaw_rate_max_, one constant)

// =============================================================================
// SILS / test injection — a tiny critical-section FIFO of command lines.
// The scenario engine calls sf_api_inject_line() from outside this task.
// SILS / テスト注入 — クリティカルセクション保護の小さな行 FIFO。
// シナリオエンジンがタスク外から sf_api_inject_line() を呼ぶ。
// =============================================================================

// Single-producer (scenario engine) / single-consumer (this task) lock-free
// ring: the producer writes the slot THEN publishes head with release order;
// the consumer acquires head before reading the slot. No mutex needed, and it
// works identically on hardware and the SILS host FreeRTOS shim.
// 単一生産者（シナリオエンジン）/ 単一消費者（本タスク）のロックフリーリング:
// 生産者はスロットを書いてから head を release 順序で公開し、消費者は head を
// acquire してからスロットを読む。ミューテックス不要で、実機と SILS ホストの
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
// Deferred commands — lines read while another command is blocking in waitUntil.
// 後回しコマンド — 別のコマンドが waitUntil で待機中に読み取った行。
// =============================================================================
// While a command blocks (takeoff / land / move ...), waitUntil keeps reading the command
// sources so `emergency` and `stop` act at once. Every other line is parked here, in
// arrival order, and the main loop runs it after the blocking command has replied — the
// same order in which they would have waited in the socket / injection ring.
// Single task only (ApiTask), so plain indices suffice.
// コマンドが待機中（takeoff / land / move ...）も waitUntil はコマンド源を読み続け、
// `emergency` と `stop` を即座に実行する。それ以外の行は到着順にここへ置き、待機中だった
// コマンドの応答後に主ループが実行する — ソケット/注入リングで待たせた場合と同じ順序。
// ApiTask 単独で使うので素の添字でよい。
struct DeferredLine {
    char        text[kInjectLen];
    sockaddr_in from;        // sender, valid when from_udp / 送信元（from_udp のとき有効）
    bool        from_udp;
};
DeferredLine g_deferred[kDeferredSlots];
int          g_deferred_head = 0;   // write index / 書込位置
int          g_deferred_tail = 0;   // read index  / 読出位置

// What ended the last waitUntil early (None = it did not), read by waitFailureText().
// 直近の waitUntil を途中で打ち切った原因（None = 打ち切られていない）。waitFailureText() が読む。
enum class WaitInterrupt : uint8_t { None, Emergency, Stop, Land, FlightEnded };
WaitInterrupt g_wait_interrupt = WaitInterrupt::None;

// Forward declaration: defined after cmdStop (it needs cmdStop and cmdEmergency).
// 前方宣言: cmdStop の後で定義（cmdStop と cmdEmergency を使うため）。
bool servicePendingCommands();
void cancelDeferredLines(const char* reason_text);   // defined after deferLine / deferLine の後で定義

// =============================================================================
// API session state / API セッション状態
// =============================================================================

bool        g_sdk_mode = false;       // "command" received / "command" 受信済み
int         g_sock = -1;              // UDP socket (HW) / UDP ソケット（実機）
sockaddr_in g_client = {};            // reply target of the line being processed / 処理中の行の返信先
bool        g_have_client = false;

// Guidance target bookkeeping — moves compose on the TARGET (Tello-style).
// 誘導目標の帳簿 — 移動は「目標」に対して合成（Tello 流）。
float g_target_ned[3] = {0, 0, 0};
float g_target_yaw    = 0;
bool  g_target_valid  = false;

// Session cruise speed [m/s] for the verb moves (up/down/forward/...), set by the
// Tello `speed x` command (x in cm/s). `go x y z speed` carries its own speed.
// セッションの巡航速度 [m/s]（up/down/forward… の verb 移動用）。Tello `speed x`（cm/s）で
// 設定。`go x y z speed` は自前の速度を持つ。
float g_default_speed = kDefaultSpeed;

// Client address for the UDP:8890 state stream, shared with TelloStateTask.
// g_client (the sockaddr_in above) is read ONLY by ApiTask; TelloStateTask runs in
// its own context, so the client IPv4 is mirrored into an atomic (a 32-bit IP fits
// one word — no torn read), matching the lock-free style of g_inject_head/tail.
// The state stream targets <client-ip>:8890, NOT g_client.sin_port (= the 8889
// source port). flight_time_s is maintained by TelloStateTask and read by `time?`.
// UDP:8890 状態ストリーム用のクライアントアドレス（TelloStateTask と共有）。上の g_client は
// ApiTask 専用なので、クライアント IPv4 を 1 ワードのアトミックにミラーする（32bit IP は
// トーン無し、g_inject_head/tail と同じロックフリー流儀）。送出先は <client-ip>:8890 で、
// g_client.sin_port（= 8889 の送信元ポート）ではない。flight_time_s は TelloStateTask が
// 維持し `time?` が読む。
std::atomic<uint32_t> g_client_ip{0};       // network byte order / ネットワークバイト順
std::atomic<bool>     g_client_known{false};
std::atomic<int>      g_flight_time_s{0};

// Receive one datagram without blocking (MSG_DONTWAIT: the loop must keep draining SILS
// injections; a parked blocking receive would starve them) and mirror its sender for the
// state stream. It does NOT touch the reply target (g_client): each line is answered at its own
// sender (see ReplyTarget), so a line read while another command blocks cannot redirect that
// command's reply. Returns false when nothing is pending or there is no socket.
// 1 データグラムをブロックせず受信（MSG_DONTWAIT: ループは SILS 注入の drain を続ける必要があり、
// ブロッキング受信のパークはそれを飢餓させる）し、送信元を状態ストリーム用にミラーする。返信先
// （g_client）には触れない: 各行はその送信元へ返信する（ReplyTarget 参照）ので、別のコマンドの
// 待機中に読んだ行が、そのコマンドの返信先を書き換えることはない。何も届いていない、またはソケットが
// 無いとき false。
bool receiveDatagram(char* line, size_t line_size, sockaddr_in& from)
{
    if (g_sock < 0) {
        return false;
    }
    socklen_t from_len = sizeof(from);
    const ssize_t received = ::recvfrom(g_sock, line, line_size - 1, MSG_DONTWAIT,
                                        reinterpret_cast<sockaddr*>(&from), &from_len);
    if (received <= 0) {
        return false;
    }
    line[received] = '\0';
    // Mirror the client IP for TelloStateTask (UDP:8890 state stream).
    // Publishing it here means the stream starts the moment a client
    // sends anything — exactly when djitellopy connect() waits for state.
    // TelloStateTask 用にクライアント IP をミラー（UDP:8890 状態ストリーム）。
    // ここで公開することで、クライアントが何か送った瞬間＝djitellopy connect() が
    // 状態を待つ瞬間にストリームが始まる。
    g_client_ip.store(from.sin_addr.s_addr, std::memory_order_relaxed);
    g_client_known.store(true, std::memory_order_release);
    return true;
}

// batteryPercent — 1S LiPo voltage → 0..100% (shared by `battery?` and the state
// stream so the two never drift). 3.3V = 0%, 4.2V = 100%, clamped.
// batteryPercent — 1S LiPo 電圧 → 0..100%（`battery?` と状態ストリームで共有しドリフト防止）。
int batteryPercent()
{
    const float v = sf::sensor_power.latest().voltage;
    int pct = static_cast<int>((v - kBattEmptyV) / (kBattFullV - kBattEmptyV) * 100.0f);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

// -----------------------------------------------------------------------------
// reply — send the answer to the UDP client AND log it (SILS gates / debugging).
// reply — UDP クライアントへ返信し、ログにも出す（SILS 判定・デバッグ用）。
// -----------------------------------------------------------------------------
void reply(const char* text)
{
    ESP_LOGI(TAG, "reply: %s", text);
    if (g_sock >= 0 && g_have_client) {
        ::sendto(g_sock, text, std::strlen(text), 0,
                 reinterpret_cast<sockaddr*>(&g_client), sizeof(g_client));
    }
}

// Scope in which reply() answers the given sender instead of the current reply target; the
// previous target is restored on exit. `from == nullptr` (an injected line) leaves it unchanged.
// Used so that every line — executed at once, deferred or cancelled — is answered at its OWN
// sender, and a command that blocks keeps answering its original sender.
// このスコープの間、reply() は現在の返信先でなく指定の送信元へ返信し、抜けると元の返信先に戻す。
// `from == nullptr`（注入行）は変更しない。実行・後回し・取り消しのどの行も「自分の」送信元へ
// 返信し、待機中のコマンドは元の送信元へ返信し続けるようにするため。
class ReplyTarget {
public:
    explicit ReplyTarget(const sockaddr_in* from)
        : saved_client_(g_client), saved_have_client_(g_have_client)
    {
        if (from != nullptr) {
            g_client      = *from;
            g_have_client = true;
        }
    }
    ~ReplyTarget()
    {
        g_client      = saved_client_;
        g_have_client = saved_have_client_;
    }
    ReplyTarget(const ReplyTarget&)            = delete;
    ReplyTarget& operator=(const ReplyTarget&) = delete;

private:
    sockaddr_in saved_client_;
    bool        saved_have_client_;
};

// -----------------------------------------------------------------------------
// Topic helpers / トピックヘルパ
// -----------------------------------------------------------------------------
sf::FlightState currentState()
{
    return static_cast<sf::FlightState>(sf::system_mode.latest().state);
}

void publishApiVerb(sf::ApiCmd verb, sf::FlightMode mode = sf::FlightMode::POS_HOLD)
{
    // arg (FlipDirection, plan §4.4) is 0/unused here — every existing caller of
    // this helper is a directionless verb (Takeoff/Land/Emergency). cmdFlip(), a
    // later task, publishes ApiCommand directly with arg set instead of going
    // through this helper.
    // arg（FlipDirection、plan §4.4）はここでは 0/未使用 — このヘルパーの既存の
    // 呼び出し元は全て方向を持たない verb（Takeoff/Land/Emergency）。cmdFlip()（後続
    // タスク）は本ヘルパーを経由せず arg を設定した ApiCommand を直接発行する。
    sf::api_command.publish({static_cast<uint8_t>(verb),
                             static_cast<uint8_t>(mode),
                             static_cast<uint32_t>(esp_timer_get_time()),
                             0});
}

// Publish the guidance target with the heading `yaw_rad` (the final goal, or a point on the way to
// it during a turn — see cmdRotate).
// 誘導目標を向き `yaw_rad`（最終の目標、または回頭中はそこへ向かう途中の点 — cmdRotate 参照）で
// publish する。
void publishGuidanceYaw(float speed, float yaw_rad)
{
    sf::GuidanceTarget t{};
    t.position[0] = g_target_ned[0];
    t.position[1] = g_target_ned[1];
    t.position[2] = g_target_ned[2];
    t.yaw         = yaw_rad;
    t.speed       = speed;
    t.mode        = 1;
    t.timestamp   = static_cast<uint32_t>(esp_timer_get_time());
    sf::command_target.publish(t);
}

void publishGuidance(float speed)
{
    publishGuidanceYaw(speed, g_target_yaw);
}

// True while the craft is still in the flight a move / rotate / flip / excitation belongs to
// (FLYING, or FLIP inside a flip). Any other state means the flight has ended: a pilot's
// DISARM or landing, a failsafe landing, an emergency.
// 移動・回頭・flip・励振が属する飛行の中に機体がまだいる間 true（FLYING、または flip 中の
// FLIP）。それ以外の状態は飛行が終わったことを意味する: パイロットの DISARM・着陸、
// フェイルセーフの着陸、emergency。
bool isFlightContinuing()
{
    const sf::FlightState state = currentState();
    return state == sf::FlightState::FLYING || state == sf::FlightState::FLIP;
}

// True in the five states in which the motors are armed (ARMED_GROUND, TAKEOFF, FLYING, LANDING, FLIP).
// モータが ARM されている 5 つの状態（ARMED_GROUND・TAKEOFF・FLYING・LANDING・FLIP）なら true。
bool isArmedState(sf::FlightState state)
{
    return state == sf::FlightState::ARMED_GROUND || state == sf::FlightState::TAKEOFF ||
           state == sf::FlightState::FLYING || state == sf::FlightState::LANDING ||
           state == sf::FlightState::FLIP;
}

// The flight state seen by the previous watchDisarmEdge() call / 前回の watchDisarmEdge() が見た飛行状態。
sf::FlightState g_watched_state = sf::FlightState::INIT;

// The one place that drops the parked lines when the craft goes from armed back to the ground, whatever caused it (a
// transmitter DISARM, the on-board button, the CLI `disarm` / `emergency`, the API, a failsafe, the abort of a takeoff, the
// end of a landing): the lines were issued for the flight that has ended, and a parked `takeoff` would otherwise ARM the
// craft again. The decision stays in StateManager (INV-3): this only watches the published state and manages the task's own queue.
// Called at every poll of waitUntil (before its predicate, so a state that the predicate treats as the end of the wait is
// seen first) and at every pass of the main loop. A line that arrives after the edge is not touched.
// 機体が armed から地上へ戻ったとき、原因を問わず（送信機の DISARM・機体のボタン・CLI の `disarm` / `emergency`・API・
// フェイルセーフ・離陸の中止・着陸の完了）、後回しの行を捨てる唯一の場所: それらの行は終わった飛行のために出されたもので、
// 残ると後回しの `takeoff` が機体をまた ARM してしまう。判断は StateManager が持ったまま（INV-3）: ここは発行済みの状態を
// 観測し、自タスクの待ち行列を管理するだけ。waitUntil の各ポーリング（述語より前。述語が待機の終わりとみなす状態を先に見る
// ため）と主ループの各周回で呼ぶ。エッジより後に届いた行には触れない。
// @design detailed_design.md §3.1 — DISARM drops the parked API lines [OK]
void watchDisarmEdge()
{
    const sf::FlightState now_state = currentState();
    const bool disarmed = isArmedState(g_watched_state) && !isArmedState(now_state);
    g_watched_state = now_state;
    if (disarmed) {
        ESP_LOGW(TAG, "craft disarmed - parked lines cancelled");
        cancelDeferredLines(kFlightEndedCancelledReply);
    }
}

// End the wait in progress because the flight ended, and drop the lines parked behind it: they
// were issued for that flight, and a parked `takeoff` would otherwise ARM the craft again.
// 進行中の待機を、飛行が終わったために打ち切り、その後ろに置かれた行を捨てる: それらはその
// 飛行のために出された行で、後回しの `takeoff` が残ると機体をまた ARM してしまう。
void endWaitByFlightEnd()
{
    ESP_LOGW(TAG, "flight ended during a wait - parked lines cancelled");
    g_wait_interrupt = WaitInterrupt::FlightEnded;
    cancelDeferredLines(kFlightEndedCancelledReply);
}

// Wait until `pred` returns true, polling at kWaitPollMs. Returns false on timeout OR when
// an `emergency` / `stop` (or a `land` in ARMED_GROUND) read during the wait ended it early (see g_wait_interrupt and
// waitFailureText). With `ends_with_flight` the wait also ends early, returning false, as soon as
// the craft is no longer in flight (isFlightContinuing), so a move / rotate / flip / excitation
// does not run to its timeout after the flight is over. The API task may block here — it owns no
// control-path deadline — but it must stay reachable for the commands that cancel what it waits for.
// pred が真になるまで kWaitPollMs 周期でポーリング。タイムアウト、または待機中に読んだ
// `emergency` / `stop`（または ARMED_GROUND での `land`）で早期に打ち切られたとき false（g_wait_interrupt と waitFailureText
// 参照）。`ends_with_flight` のときは、機体が飛行中でなくなった（isFlightContinuing）時点でも待機を打ち切って
// false を返し、移動・回頭・flip・励振が飛行の終わった後でタイムアウトまで続かないようにする。
// 本タスクは制御経路の締切を持たないためブロックしてよいが、待っている対象を取り消すコマンドには応答できる状態を保つ。
template <typename Pred>
bool waitUntil(uint32_t timeout_ms, Pred pred, bool ends_with_flight = false)
{
    g_wait_interrupt = WaitInterrupt::None;
    const uint32_t start = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    while (static_cast<uint32_t>(esp_timer_get_time() / 1000) - start < timeout_ms) {
        watchDisarmEdge();   // before pred(): see the edge first / pred() より前: エッジを先に見る
        if (pred()) {
            return true;
        }
        if (ends_with_flight && !isFlightContinuing()) {
            endWaitByFlightEnd();
            return false;
        }
        if (servicePendingCommands()) {
            return false;   // emergency / stop took over / emergency か stop が引き取った
        }
        vTaskDelay(pdMS_TO_TICKS(kWaitPollMs));
    }
    return false;
}

// Reply text for a waitUntil that returned false: the interrupting command (`emergency`, `stop` or `land`), the end of the
// flight, else the caller's timeout text.
// false を返した waitUntil の応答文字列: 打ち切ったコマンド（`emergency`・`stop`・`land`）、飛行の終わり、なければ呼び出し側のタイムアウト文。
const char* waitFailureText(const char* timeout_text)
{
    switch (g_wait_interrupt) {
        case WaitInterrupt::Emergency: return "error interrupted by emergency";
        case WaitInterrupt::Stop:      return "error interrupted by stop";
        case WaitInterrupt::Land:      return "error cancelled by land";
        case WaitInterrupt::FlightEnded: return kFlightEndedReply;
        default:                       return timeout_text;
    }
}

// -----------------------------------------------------------------------------
// Flight verbs / 飛行 verb
// -----------------------------------------------------------------------------

// Yaw angle [rad] of an estimate (quaternion → ZYX yaw), the same formula the guidance
// seed and the rotate predicate use.
// 推定値のヨー角 [rad]（クォータニオン → ZYX ヨー）。cmdTakeoff・回頭の述語と同じ式。
float estimateYaw(const sf::StateEstimate& est)
{
    const float w = est.attitude[0], x = est.attitude[1];
    const float y = est.attitude[2], z = est.attitude[3];
    return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

// Seed the guidance target at the altitude `down_m` (NED down [m], negative above the origin) and the
// CURRENT heading, and publish it. The horizontal part is the point `hold_ned` (north, east [m]) when
// one is given, else the CURRENT position. Shared by cmdTakeoff (the auto-takeoff target altitude, the
// controller's hold point) and cmdStop (the altitude the craft is at, where it is).
// 誘導目標を高度 `down_m`（NED の下向き [m]、原点より上は負）と「現在」の向きで初期化して publish する。
// 水平の部分は、`hold_ned`（北・東 [m]）が与えられればその点、なければ「現在」の位置。cmdTakeoff
// （自動離陸の目標高度、制御器の保持点）と cmdStop（いまの高度、いまの位置）が共有する。
void seedGuidance(float down_m, const float* hold_ned = nullptr)
{
    const sf::StateEstimate est = sf::estimate_state.latest();
    g_target_ned[0] = (hold_ned != nullptr) ? hold_ned[0] : est.position[0];
    g_target_ned[1] = (hold_ned != nullptr) ? hold_ned[1] : est.position[1];
    g_target_ned[2] = down_m;
    g_target_yaw    = estimateYaw(est);
    g_target_valid  = true;
    publishGuidance(g_default_speed);
}

// Seed the guidance target where the craft is, altitude included, so it holds there.
// 誘導目標を、高度も含めて機体のいる場所に初期化し、その場に保持させる。
void seedGuidanceAtCurrentPose()
{
    seedGuidance(sf::estimate_state.latest().position[2]);
}

// Reply text for a pre-arm reason (SystemMode.arm_block) that refuses a takeoff; nullptr for None.
// ARM 前判定の理由（SystemMode.arm_block）に対する takeoff 拒否の応答文字列。None なら nullptr。
const char* armBlockRefusalText(sf::ArmBlock block)
{
    switch (block) {
        case sf::ArmBlock::Pairing:      return "error pairing in progress";
        case sf::ArmBlock::Battery:      return "error battery unsafe - USB power or low voltage";
        case sf::ArmBlock::Calibrating:  return "error calibrating - keep the craft still";
        case sf::ArmBlock::BenchRelevel: return "error bench motor test active";
        case sf::ArmBlock::TiltPending:  return "error attitude check pending - wait a moment";
        case sf::ArmBlock::TiltMismatch: return "error attitude mismatch - place the craft level";
        case sf::ArmBlock::TofSilent:    return "error tof silent - check the sensor";
        case sf::ArmBlock::None:
        default:                         return nullptr;
    }
}

// Reason text when a takeoff must be refused before the verb is published, else nullptr. The pre-arm
// reason is the one StateManager publishes (single source, StateManager::evaluateArmBlock): a takeoff
// that its ARM would refuse is answered at once instead of waiting for the takeoff timeout, and leaves
// nothing behind. The ToF silence applies on the ground in any state (nobody can see the climb); the
// other reasons only in IDLE_GROUND, the state in which ARM is evaluated (a craft already in ARMED_GROUND is armed).
// takeoff を verb の publish 前に拒否すべきときの理由文字列。なければ nullptr。ARM 前判定の理由は
// StateManager が発行したもの（単一の情報源、StateManager::evaluateArmBlock）: ARM が拒否する takeoff を、
// 離陸のタイムアウトまで待たずにすぐ応答し、何も残さない。ToF の出力停止は地上のどの状態でも適用
// （上昇を誰も見られない）、他の理由は ARM を判定する状態 IDLE_GROUND でだけ適用（ARMED_GROUND は ARM 済み）。
const char* takeoffRefusalText()
{
    const sf::FlightState st = currentState();
    if (st != sf::FlightState::IDLE_GROUND && st != sf::FlightState::ARMED_GROUND) {
        return "error not on ground";
    }
    if (!sf::system_status.latest().calibrated) {
        return "error calibrating - keep the craft still";
    }
    const auto block = static_cast<sf::ArmBlock>(sf::system_mode.latest().arm_block);
    if (block == sf::ArmBlock::TofSilent || st == sf::FlightState::IDLE_GROUND) {
        return armBlockRefusalText(block);
    }
    return nullptr;
}

// Wait for the auto-takeoff chain (ARM → TAKEOFF climb → FLYING). If the chain was entered
// (ARMED_GROUND or TAKEOFF seen) and the craft is then back on the ground or landing, the
// state machine aborted the climb (e.g. the ToF went silent): stop waiting. The abort can
// last only a few ms in LANDING, so IDLE_GROUND counts too. Returns waitUntil's result.
// 自動離陸の鎖（ARM → TAKEOFF 上昇 → FLYING）を待つ。鎖に入った（ARMED_GROUND か TAKEOFF を
// 見た）後に地上復帰か LANDING になっていれば、状態機械が上昇を中止した（例: ToF が出力停止）:
// 待つのをやめる。中止後の LANDING が数 ms で終わることがあるので IDLE_GROUND も数える。
// waitUntil の結果を返す。
bool waitTakeoffChain()
{
    bool chain_entered = false;
    return waitUntil(kTakeoffTimeoutMs, [&chain_entered] {
        const sf::FlightState now_state = currentState();
        if (now_state == sf::FlightState::ARMED_GROUND ||
            now_state == sf::FlightState::TAKEOFF) {
            chain_entered = true;
        }
        const bool aborted = chain_entered &&
                             (now_state == sf::FlightState::LANDING ||
                              now_state == sf::FlightState::IDLE_GROUND);
        return now_state == sf::FlightState::FLYING || aborted;
    });
}

// Seed the guidance target after an auto-takeoff: the auto-takeoff target altitude (NED down =
// -altitude) and the controller's horizontal hold point, read from controller_status. The hold point
// is the one the controller captured at the takeoff (the point where the estimator released its
// ground pin); the climb's drift is not taken into the target. A controller that publishes no hold
// point (position_hold_valid false) leaves the target at the current position, as before. One log
// line gives the target and its offset from the estimate of this moment.
// 自動離陸の後の誘導目標を初期化する: 自動離陸の目標高度（NED 下向き = -高度）と、controller_status
// から読む制御器の水平の保持点。保持点は、制御器が離陸のときに取り込んだ点（推定器が地上の固定を解いた
// 点）で、上昇中のずれは目標に取り込まない。保持点を publish しない制御器（position_hold_valid が false）
// では、従来どおり現在の位置を目標にする。目標と、この瞬間の推定の位置とのずれを 1 行のログに出す。
void seedGuidanceAtTakeoffHold()
{
    const sf::ControllerStatus controller = sf::controller_status.latest();
    const float* hold_ned = controller.position_hold_valid ? controller.position_hold_ned : nullptr;
    seedGuidance(-sf::limits::kAutoTakeoffTargetAltitudeM, hold_ned);   // NED down = -altitude / NED 下向き = -高度

    const sf::StateEstimate est = sf::estimate_state.latest();
    ESP_LOGI(TAG, "Takeoff hold: target N %.3f E %.3f m (%s), estimate N %.3f E %.3f m, target - estimate N %.3f E %.3f m",
             static_cast<double>(g_target_ned[0]), static_cast<double>(g_target_ned[1]),
             controller.position_hold_valid ? "controller hold point" : "current estimate",
             static_cast<double>(est.position[0]), static_cast<double>(est.position[1]),
             static_cast<double>(g_target_ned[0] - est.position[0]),
             static_cast<double>(g_target_ned[1] - est.position[1]));
}

void cmdTakeoff()
{
    const char* refusal = takeoffRefusalText();
    if (refusal != nullptr) {
        reply(refusal);
        return;
    }
    publishApiVerb(sf::ApiCmd::Takeoff, sf::FlightMode::POS_HOLD);

    if (!waitTakeoffChain()) {
        reply(waitFailureText("error takeoff timeout"));
        return;
    }
    if (currentState() != sf::FlightState::FLYING) {
        reply("error takeoff aborted");
        return;
    }

    // FLYING ends the auto-takeoff, which holds its target altitude from here on (the same as
    // after a transmitter ARM), also when its time limit ended the climb below the target.
    // Seed the guidance target at the auto-takeoff target altitude and the current heading, so the
    // craft keeps climbing to it, and at the controller's horizontal hold point, NOT at the
    // estimate of this moment: the drift of the climb stays corrected, as after a transmitter ARM.
    // Composing subsequent moves from the TARGET starts here.
    // FLYING で自動離陸が終わり、以降は目標高度を保持する（送信機の ARM の後と同じ）。上昇が
    // 時間切れで目標に届かずに終わった場合も同じ。誘導目標を、自動離陸の目標高度と現在の向き
    // （機体が目標まで上昇を続けるようにする）、水平は制御器の保持点で初期化する。この瞬間の
    // 推定の位置にはしない: 上昇中に生じたずれは、送信機の ARM の後と同じく戻される。
    // 以降の移動合成はこの「目標」から始まる。
    seedGuidanceAtTakeoffHold();
    reply("ok");
}

// True when `mode` (one reading of system_mode) says the craft is in ARMED_GROUND in an altitude mode (ALT_HOLD / POS_HOLD): the motors have begun to spool on the floor and StateManager answers a land request with an immediate DISARM
// (detailed_design.md §3 note 13). In a manual mode it does not: the craft may already be lifting off (the request is refused). Takes ONE reading, so the state and the mode come from the same instant.
// `mode`（system_mode の 1 回の読み）が、ARMED_GROUND にいて高度モード（ALT_HOLD / POS_HOLD）であることを示すとき true: モータは床の上でスプールを始めており、StateManager は着陸要求に即時 DISARM で応える
// （detailed_design.md §3 注13）。手動モードでは違う: 機体はもう浮き始めているかもしれない（要求は拒否する）。状態とモードが同じ瞬間のものになるよう、読みは 1 回だけ受け取る。
bool isSpoolingInAltitudeMode(const sf::SystemMode& mode)
{
    return static_cast<sf::FlightState>(mode.state) == sf::FlightState::ARMED_GROUND &&
           sf::flightModeNeedsAltitude(static_cast<sf::FlightMode>(mode.sub_mode));
}

// The same test on a fresh reading of system_mode.
// system_mode を新たに 1 回読んで同じ判定をする。
bool isSpoolingInAltitudeMode()
{
    return isSpoolingInAltitudeMode(sf::system_mode.latest());
}

// True once the craft has left TAKEOFF (for waitUntil).
// 機体が TAKEOFF を抜けたら true（waitUntil 用）。
bool hasLeftTakeoff()
{
    return currentState() != sf::FlightState::TAKEOFF;
}

// True when a `land` has finished: the craft is on the ground, or it entered TAKEOFF (a `land` issued from ARMED_GROUND lost the race against the auto-takeoff: StateManager ignores a land request in TAKEOFF and the `land` must be issued again after the takeoff).
// `land` が済んだら true: 機体が地上にいる、または TAKEOFF に入った（ARMED_GROUND から出した `land` が自動離陸との競合に負けた: StateManager は TAKEOFF での着陸要求を無視するので、離陸の後でもう一度出す必要がある）。
bool hasLandedOrTookOff()
{
    const sf::FlightState state = currentState();
    return state == sf::FlightState::IDLE_GROUND || state == sf::FlightState::TAKEOFF;
}

// `land`. A request that arrives while the craft is in TAKEOFF (e.g. the auto-takeoff an RC ARM started: no blocking command is waiting for it) is held until the takeoff has ended, as the one parked during a `takeoff` wait is
// (detailed_design.md §3 note 13); how the takeoff ended decides the answer: FLYING -> the landing is issued; LANDING -> that landing is waited for; on the ground -> "error not flying". A land issued from ARMED_GROUND is answered "ok" only once the
// craft is on the ground; if the craft entered TAKEOFF in between, StateManager has ignored it, and it is issued again after the takeoff.
// `land`。機体が TAKEOFF にいる間に届いた要求（例: RC の ARM が始めた自動離陸。待機中のコマンドは無い）は、`takeoff` の待機中に後回しにした行と同じく、離陸が終わるまで保留する（detailed_design.md §3 注13）。
// 離陸の終わり方が応答を決める: FLYING -> 着陸を発行する、LANDING -> その着陸を待つ、地上 -> "error not flying"。ARMED_GROUND から出した `land` は、機体が地上に降りてから "ok" を返す。
// その間に TAKEOFF へ移っていたら StateManager が無視しているので、離陸の後でもう一度出す。
// @design detailed_design.md §3.1 — FLYING -> LANDING / ARMED_GROUND -> IDLE_GROUND (land request) [OK]
void cmdLand()
{
    while (true) {
        if (!hasLeftTakeoff() && !waitUntil(kTakeoffTimeoutMs, hasLeftTakeoff)) {
            reply(waitFailureText("error land timeout"));
            return;
        }
        // The spool of an altitude mode counts: StateManager answers the land request with an immediate DISARM.
        // 高度モードのスプール中も受け付ける: StateManager は着陸要求に即時 DISARM で応える。
        const sf::SystemMode mode = sf::system_mode.latest();
        if (!sf::isAirborne(static_cast<sf::FlightState>(mode.state)) && !isSpoolingInAltitudeMode(mode)) {
            reply("error not flying");
            return;
        }
        g_target_valid = false;   // guidance ends with the flight / 誘導は飛行と共に終了
        publishApiVerb(sf::ApiCmd::Land);
        if (!waitUntil(kLandTimeoutMs, hasLandedOrTookOff)) {
            reply(waitFailureText("error land timeout"));
            return;
        }
        if (currentState() == sf::FlightState::IDLE_GROUND) {
            reply("ok");
            return;
        }
    }
}

// Altitude safety clamp of the guidance target. Same convention as the move-distance clamp in
// cmdMove(): clamp to the limit and still let the caller reply "ok" (the move is shorter than
// asked), log a warning.
// 誘導目標の高度安全クランプ。cmdMove() の移動距離クランプと同じ流儀: 上限へクランプし、
// 呼び出し側は "ok" を返せる（移動は要求より短くなる）。警告をログに出す。
void clampTargetAltitude()
{
    if (-g_target_ned[2] < kAltMinM) {
        g_target_ned[2] = -kAltMinM;
    }
    if (-g_target_ned[2] > kAltMaxM) {
        g_target_ned[2] = -kAltMaxM;
        ESP_LOGW(TAG, "target altitude clamped to the %.2f m ceiling",
                 static_cast<double>(kAltMaxM));
    }
}

// Block until the estimate is inside the tolerance sphere around the guidance target, or the
// timeout expires. Returns true when reached.
// 推定が誘導目標まわりの許容球に入るまで、またはタイムアウトまでブロックする。到達で true。
bool waitTargetReached(uint32_t timeout_ms)
{
    return waitUntil(timeout_ms, [] {
        const sf::StateEstimate e = sf::estimate_state.latest();
        const float dn = e.position[0] - g_target_ned[0];
        const float de = e.position[1] - g_target_ned[1];
        const float dd = e.position[2] - g_target_ned[2];
        return sqrtf(dn * dn + de * de + dd * dd) < kReachRadiusM;
    }, true);
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
    // A non-finite number must never be composed into the target (inf * 0 = NaN); dist is
    // checked too because squaring a huge finite number overflows to inf.
    // 有限でない数を目標へ合成してはならない（inf * 0 = NaN）。巨大な有限値は 2 乗で inf に
    // あふれるので、dist も検査する。
    float dist = sqrtf(fwd_cm * fwd_cm + left_cm * left_cm + up_cm * up_cm);
    if (!sf::api_arguments::allFinite({fwd_cm, left_cm, up_cm, speed_mps, dist}) ||
        dist < kMoveMinCm) {
        reply(kOutOfRangeReply);
        return;
    }
    // Tello accepts moves up to 500 cm; we clamp the ACTUAL travel to the indoor-safe
    // ceiling (kMoveMaxCm) but still reply "ok" so a program asking for a longer move
    // keeps running (it just travels less). The direction is preserved.
    // Tello は 500cm まで受理。実移動は屋内安全の上限（kMoveMaxCm）へクランプするが "ok" を返し、
    // 長距離を要求するプログラムも止めない（移動量が減るだけ）。方向は保つ。
    if (dist > kMoveMaxCm) {
        const float s = kMoveMaxCm / dist;
        fwd_cm *= s; left_cm *= s; up_cm *= s;
        dist = kMoveMaxCm;
        ESP_LOGW(TAG, "move clamped to %.0f cm (indoor ceiling)",
                 static_cast<double>(kMoveMaxCm));
    }
    const float fwd_m  = fwd_cm * 0.01f;
    const float left_m = left_cm * 0.01f;
    const float up_m   = up_cm * 0.01f;

    const float cy = cosf(g_target_yaw), sy = sinf(g_target_yaw);
    g_target_ned[0] += cy * fwd_m - sy * (-left_m);   // N (body y = right = -left)
    g_target_ned[1] += sy * fwd_m + cy * (-left_m);   // E
    g_target_ned[2] -= up_m;                          // D (up = -D)

    clampTargetAltitude();

    publishGuidance(speed_mps);

    // Block until reached (Tello semantics) — generous timeout from the path
    // length, then verify the estimate is inside the tolerance sphere.
    // 到達までブロック（Tello 流）— 経路長から余裕あるタイムアウトを取り、推定が
    // 許容球内に入ったか検証する。
    const uint32_t timeout_ms =
        static_cast<uint32_t>((dist * 0.01f / speed_mps) * 1000.0f) + 6000;
    reply(waitTargetReached(timeout_ms) ? "ok" : waitFailureText("error move timeout"));
}

// Wait for the turn to `goal_rad` (unwrapped heading), moving the guidance target ahead of the
// craft as it turns (rotate_guidance.hpp). Returns true when the turn completed. `published_rad`
// is the unwrapped target last handed to the controller; it ends at the goal unless the wait
// was cut short.
// `goal_rad`（アンラップした向き）への回頭を待ち、機体が回るにつれて誘導目標をその前へ進める
// （rotate_guidance.hpp）。回頭が完了したら true。`published_rad` は最後に制御器へ渡した
// アンラップ済みの目標で、待機が途中で打ち切られない限り最終の向きで終わる。
bool waitTurnComplete(float goal_rad, float start_heading_rad, float& published_rad)
{
    float heading_rad = start_heading_rad;
    const uint32_t timeout_ms = sf::rotate_guidance::rotateTimeoutMs(goal_rad - start_heading_rad);
    return waitUntil(timeout_ms, [&] {
        heading_rad = sf::rotate_guidance::unwrapHeading(
            heading_rad, estimateYaw(sf::estimate_state.latest()));
        const float next_rad = sf::rotate_guidance::nextGuidanceYaw(goal_rad, heading_rad, published_rad);
        if (next_rad != published_rad) {
            published_rad = next_rad;
            publishGuidanceYaw(g_default_speed, sf::math::wrapAngleRad(published_rad));
        }
        return sf::rotate_guidance::isYawTurnComplete(goal_rad, heading_rad);
    }, true);
}

// `cw` / `ccw`: turn by `delta_yaw_rad` (cw = +) from the previous TARGET heading, so repeated
// turns accumulate on the target and not on the estimate. The turn is a full `delta` in the
// commanded direction for every angle up to 360 degrees; replies "ok" when it is complete.
// `cw` / `ccw`: 直前の「目標」の向きから `delta_yaw_rad`（cw = +）だけ回る。連続した回頭は
// 推定値でなく目標に累積する。回頭は 360 度までのどの角度でも、指令の向きに `delta` ちょうど。
// 完了したら "ok" を返す。
void cmdRotate(float delta_yaw_rad, float speed_unused)
{
    (void)speed_unused;
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        reply("error not flying");
        return;
    }
    if (!sf::api_arguments::allFinite({delta_yaw_rad})) {
        reply(kOutOfRangeReply);
        return;
    }
    // The final heading, unwrapped around the measured one: the previous target is at most the
    // tolerance away from the heading, so the turn is counted from where the craft is.
    // 最終の向き。測った向きのまわりでアンラップする: 直前の目標は向きから高々許容差しか
    // 離れていないので、回頭は機体のいる向きから数える。
    const float start_heading_rad = estimateYaw(sf::estimate_state.latest());
    const float goal_rad = sf::rotate_guidance::composeFinalHeading(start_heading_rad, g_target_yaw,
                                                                    delta_yaw_rad);
    g_target_yaw = sf::math::wrapAngleRad(goal_rad);

    float published_rad = start_heading_rad;
    const bool reached = waitTurnComplete(goal_rad, start_heading_rad, published_rad);
    // A turn that timed out leaves the controller on the point it was last given, so the target
    // bookkeeping follows it (a stop or emergency has already set its own).
    // 時間切れの回頭では制御器は最後に渡された点に居るので、目標の帳簿もそれに合わせる
    // （stop・emergency は既に自分の値を設定済み）。
    if (!reached && g_wait_interrupt == WaitInterrupt::None) {
        g_target_yaw = sf::math::wrapAngleRad(published_rad);
    }
    reply(reached ? "ok" : waitFailureText("error rotate timeout"));
}

// Returns true when the stop was accepted (the target was re-set), false when rejected.
// 停止を受理した（目標を置き直した）とき true、拒否したとき false。
bool cmdStop()
{
    // Require an active API target, like move/rotate: once the pilot cancels
    // guidance (or a mode change does), the API must NOT silently re-grab control
    // with a stop/hover — the operator re-engages explicitly via takeoff/go (M-3).
    // move/rotate と同様に有効な API 目標を要求する: パイロットが誘導を解除（またはモード
    // 変更で解除）した後、API が stop/hover で黙って制御を奪い返してはならない — 操縦者が
    // takeoff/go で明示的に再係合する (M-3)。
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        reply("error not flying or guidance released - takeoff/go to re-engage");
        return false;
    }
    // Halt: re-target the spot AND heading we are at right now (the cascade brakes for us).
    // The heading matters when stop interrupts a rotate: left alone, the old target yaw
    // would keep the craft turning.
    // 停止: いまいる位置と向きを目標にし直す（ブレーキはカスケードが掛ける）。回頭中の stop
    // では向きが要る: 放置すると古い目標ヨーが機体を回し続ける。
    seedGuidanceAtCurrentPose();
    reply("ok");
    return true;
}

// Emergency: never behind any other gate — cut the motors through the state machine.
// 緊急停止: 他の判定の後ろに置かない — 状態機械経由でモータを止める。
void cmdEmergency()
{
    g_target_valid = false;
    publishApiVerb(sf::ApiCmd::Emergency);
    reply("ok");
}

// -----------------------------------------------------------------------------
// Reading commands while another command blocks (waitUntil) / 待機中のコマンド読み取り
// -----------------------------------------------------------------------------

// Remove trailing whitespace/newlines in place; returns the new length.
// 末尾の空白・改行をその場で除去し、新しい長さを返す。
size_t trimLine(char* line)
{
    size_t n = std::strlen(line);
    while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r' || line[n-1] == ' ')) {
        line[--n] = '\0';
    }
    return n;
}

// Park a line for the main loop (see DeferredLine). A full queue refuses it with a reply.
// 行を主ループ用に置く（DeferredLine 参照）。満杯なら応答で拒否する。
void deferLine(const char* text, const sockaddr_in* from)
{
    if (g_deferred_head - g_deferred_tail >= kDeferredSlots) {
        ESP_LOGW(TAG, "deferred queue full - dropped: %s", text);
        const ReplyTarget target(from);
        reply("error busy");
        return;
    }
    DeferredLine& slot = g_deferred[g_deferred_head % kDeferredSlots];
    std::strncpy(slot.text, text, kInjectLen - 1);
    slot.text[kInjectLen - 1] = '\0';
    slot.from_udp = (from != nullptr);
    if (from != nullptr) {
        slot.from = *from;
    }
    ++g_deferred_head;
}

// Discard every parked line without running it, answering each at its OWN sender with
// `reason_text`. Called when an emergency / accepted stop ends a wait: the parked lines were
// issued before it and belong to the operation it cancelled — replaying them afterwards could
// take off again right after an emergency.
// 後回しにした行を全て実行せずに捨て、それぞれ「自分の」送信元へ reason_text を返信する。
// emergency / 受理された stop が待機を終わらせたときに呼ぶ: 後回しの行はそれより前に出されたもので、
// 取り消された操作に属する — 後で再生すると emergency の直後に再離陸しうる。
void cancelDeferredLines(const char* reason_text)
{
    while (g_deferred_tail < g_deferred_head) {
        const DeferredLine& parked = g_deferred[g_deferred_tail % kDeferredSlots];
        ESP_LOGW(TAG, "deferred line cancelled: %s", parked.text);
        const ReplyTarget target(parked.from_udp ? &parked.from : nullptr);
        reply(reason_text);
        ++g_deferred_tail;
    }
}

// True for a Tello `rc a b c d` line (continuous manual control, fire-and-forget).
// Tello `rc a b c d` の行（連続マニュアル操作、撃ちっぱなし）なら true。
bool isRcLine(const char* line)
{
    return std::strncmp(line, "rc ", 3) == 0;
}

// Wait (at most kLandConfirmMs) until the craft is no longer in ARMED_GROUND, after a land request was published there. Returns true when it is on the ground (IDLE_GROUND: StateManager cut the motors); false when it entered TAKEOFF
// (StateManager ignores a land request in TAKEOFF: the request lost the race against the auto-takeoff) or when nothing changed in time. A plain poll, not waitUntil: the caller is already inside a wait.
// ARMED_GROUND で着陸要求を出した後、機体が ARMED_GROUND を抜けるまで（最長 kLandConfirmMs）待つ。地上（IDLE_GROUND: StateManager がモータを止めた）なら true。TAKEOFF に入った（StateManager は TAKEOFF での着陸要求を無視する:
// 要求が自動離陸との競合に負けた）か、時間内に変わらなかったなら false。waitUntil ではなく素のポーリング: 呼び出し側が既に待機の中にいるため。
bool waitLandOnFloorConfirmed()
{
    const uint32_t start = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    while (currentState() == sf::FlightState::ARMED_GROUND &&
           static_cast<uint32_t>(esp_timer_get_time() / 1000) - start < kLandConfirmMs) {
        vTaskDelay(pdMS_TO_TICKS(kWaitPollMs));
    }
    return currentState() == sf::FlightState::IDLE_GROUND;
}

// A `land` that arrives while the craft is still in ARMED_GROUND (the motors have begun to turn on the floor, the spool of the auto-takeoff; the wait is the `takeoff` chain): there is no height to land from, so the request is the immediate
// DISARM (StateManager::notifyLandingRequest). Once the craft is confirmed on the ground (waitLandOnFloorConfirmed) the line is answered here, "ok" as `emergency` is (the motors have stopped), and the wait ends (the waiting `takeoff`
// replies "error cancelled by land"). The lines parked before it are dropped, as `emergency` drops them (each answered "error cancelled by land"): they were issued before the `land` and a parked `takeoff` would otherwise ARM the craft again right after
// the DISARM. If the craft entered TAKEOFF in between (or did not change in time), the request was not carried out: the line is parked like a `land` read in TAKEOFF, the wait goes on, and the `land` runs after the takeoff has ended (the waiting
// `takeoff` replies as the takeoff ended; cmdLand answers the parked `land`). Returns true when the line ended the wait.
// 機体がまだ ARMED_GROUND の間に届いた `land`（モータは床の上で回り始めたところ、自動離陸のスプール中。待機は `takeoff` の鎖）: 降りる高さが無いので、要求は即時 DISARM（StateManager::notifyLandingRequest）。
// 機体が地上にいると確かめてから（waitLandOnFloorConfirmed）、行はここで応答する: `emergency` と同じ "ok"（モータは止まった）。待機を終える（待っていた `takeoff` は "error cancelled by land" を返す）。
// それより前に後回しにした行は `emergency` と同じく捨てる（それぞれに "error cancelled by land" を返す）: `land` より前に出された行で、後回しの `takeoff` が残ると、DISARM の直後にまた ARM する。
// その間に TAKEOFF へ移っていた（または時間内に変わらなかった）なら要求は実行されていない: TAKEOFF で読んだ `land` と同じく行を後回しにして待機を続け、`land` は離陸が終わってから実行する
// （待っていた `takeoff` は離陸の終わり方どおりに応答し、後回しの `land` には cmdLand が応える）。待機を終えたとき true。
// @design detailed_design.md §3.1 — ARMED_GROUND -> IDLE_GROUND (land request) [OK]
bool landOnFloorDuringWait(const sockaddr_in* from)
{
    g_target_valid = false;   // guidance ends with the flight / 誘導は飛行と共に終了
    publishApiVerb(sf::ApiCmd::Land);
    ESP_LOGI(TAG, "cmd: land (during a blocking command, ARMED_GROUND)");   // after the publish, not between the read of the state and the publish / 発行の後。状態を読んでから発行するまでの間には置かない
    if (!waitLandOnFloorConfirmed()) {
        ESP_LOGI(TAG, "land not carried out on the floor (the craft entered TAKEOFF) -> parked");
        deferLine("land", from);
        return false;
    }
    cancelDeferredLines("error cancelled by land");   // a parked `takeoff` must not ARM the craft again after the DISARM / 後回しの `takeoff` が、DISARM の後にまた ARM してはならない
    reply("ok");
    g_wait_interrupt = WaitInterrupt::Land;
    return true;
}

// Handle one line read during a wait. `emergency` and `stop` run at once through the same
// functions processLine uses, each answered at its own sender, and drop the parked lines
// (cancelDeferredLines). An `rc` line is DROPPED, not parked: it is a stale operator input
// the moment the wait ends, and replaying it would steer the craft with old sticks. Anything
// else is deferred, including a `land` read while the craft is in TAKEOFF (it runs after the takeoff has ended), except a `land` read while the craft is in ARMED_GROUND in an altitude mode
// (landOnFloorDuringWait: the immediate DISARM, answered at once once the craft is on the ground, and the wait ends; if the craft entered TAKEOFF meanwhile it is parked like the others). Returns true when the line ended the wait (an emergency, an accepted stop, or that `land`). A rejected stop (not flying: it is takeoff / land / flip that is waiting) only gets
// its error reply, the parked lines stay and the wait goes on.
// 待機中に読んだ 1 行を処理する。`emergency` と `stop` は processLine と同じ関数で即実行し
// （返信は各自の送信元へ）、後回しの行を捨てる（cancelDeferredLines）。`rc` の行は後回しにせず
// 「捨てる」: 待機が終わった時点で古い操作入力であり、再生すると古いスティックで機体を操縦して
// しまう。それ以外は後回し。その行が待機を終わらせたとき（emergency、または受理された stop）true。
// 拒否された stop（飛行中でない: takeoff / land / flip の待機中）は拒否の応答だけ返し、後回しの行は
// 残して待機を続ける。機体が TAKEOFF にいる間に読んだ `land` も後回しにして（離陸が終わってから実行する）、高度モードの ARMED_GROUND にいる間に読んだ `land` だけは、即時 DISARM をその場で、機体が地上に降りてから応答して待機を終える（landOnFloorDuringWait。その間に TAKEOFF へ移っていたら他と同じく後回しにする）。
bool dispatchDuringWait(char* line, const sockaddr_in* from)
{
    if (trimLine(line) == 0) {
        return false;
    }
    if (isRcLine(line)) {
        return false;   // dropped: fire-and-forget, no reply / 破棄: 撃ちっぱなしで応答なし
    }
    const ReplyTarget target(from);
    if (std::strcmp(line, "emergency") == 0) {
        ESP_LOGI(TAG, "cmd: %s (during a blocking command)", line);
        cmdEmergency();                       // cut the motors first / まずモータを止める
        cancelDeferredLines("error cancelled by emergency");
        g_wait_interrupt = WaitInterrupt::Emergency;
        return true;
    }
    if (std::strcmp(line, "stop") == 0) {
        ESP_LOGI(TAG, "cmd: %s (during a blocking command)", line);
        if (cmdStop()) {
            cancelDeferredLines("error cancelled by stop");
            g_wait_interrupt = WaitInterrupt::Stop;
            return true;
        }
        return false;
    }
    if (std::strcmp(line, "land") == 0 && isSpoolingInAltitudeMode()) {
        return landOnFloorDuringWait(from);
    }
    deferLine(line, from);
    return false;
}

// Read every pending command line (injection ring, then UDP) without blocking; see
// dispatchDuringWait. Returns true when the wait must end.
// 保留中のコマンド行を全て（注入リング、次に UDP）ブロックせず読む。dispatchDuringWait
// 参照。待機を終えるべきとき true。
bool servicePendingCommands()
{
    char line[kInjectLen];
    while (popInjected(line, sizeof(line))) {
        if (dispatchDuringWait(line, nullptr)) {
            return true;
        }
    }
    sockaddr_in from = {};
    while (receiveDatagram(line, sizeof(line), from)) {
        if (dispatchDuringWait(line, &from)) {
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// cmdFlip — Tello `flip l/r/f/b`: single-axis 360-degree flip
// (flip-maneuver-plan.md §4.4). Pre-gates mirror cmdTakeoff/cmdLand (state +
// mode); the controller's own execution-condition judgment (flip_ready /
// flip_block_reason, INV-3: detection is the controller's) is then surfaced
// as a specific error. Blocks through the whole maneuver like cmdTakeoff.
// cmdFlip — Tello `flip l/r/f/b`: 単軸360度宙返り（plan §4.4）。事前判定は
// cmdTakeoff/cmdLand と同型（状態＋モード）。その先の実行条件判定
// （flip_ready/flip_block_reason、INV-3: 検出は制御器の役割）を具体的な
// エラーとして表す。cmdTakeoff 同様マニューバ全体をブロックする。
// -----------------------------------------------------------------------------

bool parseFlipDirection(const char* token, sf::FlipDirection& dir)
{
    if (std::strcmp(token, "l") == 0) { dir = sf::FlipDirection::Left;    return true; }
    if (std::strcmp(token, "r") == 0) { dir = sf::FlipDirection::Right;   return true; }
    if (std::strcmp(token, "f") == 0) { dir = sf::FlipDirection::Forward; return true; }
    if (std::strcmp(token, "b") == 0) { dir = sf::FlipDirection::Back;    return true; }
    return false;
}

// Text for ControllerStatus.flip_block_reason (plan §3.1 C2-C8). C1 (FLYING)
// and "already FLIP" are judged by this task itself, so they never reach here.
// ControllerStatus.flip_block_reason（plan §3.1 C2-C8）の文字列。C1(FLYING)と
// 「既にFLIP中」は本タスク自身が判定するため、ここには来ない。
const char* flipBlockErrorText(sf::FlipBlockReason reason)
{
    switch (reason) {
        case sf::FlipBlockReason::TooLow:             return "too low";
        case sf::FlipBlockReason::NotSteady:          return "not steady";
        case sf::FlipBlockReason::BatteryLow:         return "battery low";
        case sf::FlipBlockReason::EstimatorUnhealthy: return "estimator";
        case sf::FlipBlockReason::Cooldown:           return "cooldown";
        case sf::FlipBlockReason::Busy:               return "busy";
        case sf::FlipBlockReason::SourceConflict:     return "busy";
        case sf::FlipBlockReason::Disabled:           return "disabled";
        default:                                       return "not ready";
    }
}

// Text for ControllerStatus.flip_result when it is not Ok (plan §3.5 abort
// paths). / flip_result が Ok でないときの文字列（plan §3.5 の打ち切り経路）。
const char* flipAbortErrorText(sf::FlipResult result)
{
    switch (result) {
        case sf::FlipResult::AbortedSpinTimeout:    return "spin_timeout";
        case sf::FlipResult::AbortedGyroLimit:      return "gyro_limit";
        case sf::FlipResult::AbortedRecoverTimeout: return "recover_timeout";
        case sf::FlipResult::AbortedBrakeTimeout:   return "brake_timeout";
        default:                                     return "unknown";
    }
}

// flipPreflight — direction parse + state/mode/execution-condition gates
// (plan §3.1 C1/C3-C8). Sends the error reply itself and returns false on
// any gate failure; on success fills dir/mode and sends nothing yet (the
// caller still has to publish and wait). Split out of cmdFlip to keep each
// function under coding_and_education.md's 50-line guideline.
// flipPreflight — 方向解析＋状態・モード・実行条件判定（plan §3.1
// C1/C3-C8）。不成立ならエラー応答を自分で送り false を返す。成立時は
// dir/mode を埋めて true を返す（応答はまだ送らない — 発行と待機は
// 呼び出し元の役割）。coding_and_education.md の50行目安を守るため
// cmdFlip から分離した。
bool flipPreflight(const char* direction_token, sf::FlipDirection& dir, sf::FlightMode& mode)
{
    if (!parseFlipDirection(direction_token, dir)) { reply("error flip: bad direction"); return false; }

    const sf::FlightState st = currentState();
    if (st == sf::FlightState::FLIP)   { reply("error flip: busy");       return false; }
    if (st != sf::FlightState::FLYING) { reply("error flip: not flying"); return false; }

    // API targets ALT_HOLD/POS_HOLD only (hovering assumed, plan §9
    // decision 3); ACRO/STABILIZE stay button-only (Phase 3, recovery
    // height is stick-driven there).
    // API の対象は ALT_HOLD/POS_HOLD のみ（ホバリング前提、plan §9 決定3）。
    // ACRO/STABILIZE はボタン専用のまま（Phase 3、回復高度はスティック）。
    mode = static_cast<sf::FlightMode>(sf::system_mode.latest().sub_mode);
    if (mode != sf::FlightMode::ALT_HOLD && mode != sf::FlightMode::POS_HOLD) {
        reply("error flip: mode");
        return false;
    }

    const sf::ControllerStatus cs = sf::controller_status.latest();
    if (!cs.flip_ready) {
        char buf[40];
        std::snprintf(buf, sizeof(buf), "error flip: %s",
                      flipBlockErrorText(static_cast<sf::FlipBlockReason>(cs.flip_block_reason)));
        reply(buf);
        return false;
    }
    return true;
}

// waitFlipOutcome — block through FLYING->FLIP->FLYING (or an abort path)
// and send the reply. The FLYING<->FLIP edges are executed by state_task,
// not this task (INV-3: detection vs. judgment/execution split) — this
// task only observes system_mode. A quick DISARM/emergency/IMPACT during
// the window lands in IDLE_GROUND instead of back in FLYING.
// waitFlipOutcome — FLYING→FLIP→FLYING（または打ち切り経路）をブロックして
// 応答を送る。FLYING<->FLIP の遷移実行は state_task の役割（本タスクでは
// ない、INV-3: 検出と判断・実行の分離）— system_mode を観測するのみ。窓内の
// DISARM/emergency/IMPACT は FLYING でなく IDLE_GROUND へ落ちる。
void waitFlipOutcome()
{
    if (!waitUntil(500, [] { return currentState() == sf::FlightState::FLIP; }, true)) {
        reply(waitFailureText("error flip: rejected"));
        return;
    }
    if (!waitUntil(3000, [] { return currentState() != sf::FlightState::FLIP; })) {
        reply(waitFailureText("error flip: timeout"));
        return;
    }
    if (currentState() != sf::FlightState::FLYING) {
        cancelDeferredLines(kFlightEndedCancelledReply);   // the flight is over / 飛行は終わった
        reply("error flip: aborted disarmed");
        return;
    }
    const auto result = static_cast<sf::FlipResult>(sf::controller_status.latest().flip_result);
    if (result == sf::FlipResult::Ok) {
        reply("ok");
        return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "error flip: aborted %s", flipAbortErrorText(result));
    reply(buf);
}

void cmdFlip(const char* direction_token)
{
    sf::FlipDirection dir;
    sf::FlightMode mode;
    if (!flipPreflight(direction_token, dir, mode)) return;

    // arg carries FlipDirection — same convention as EstimatorCommand.arg's
    // CovScope (data_types.hpp "Flip Maneuver Types"). Published directly,
    // not via publishApiVerb (that helper is for directionless verbs only,
    // see its own comment).
    // arg は FlipDirection を運ぶ（data_types.hpp「Flip Maneuver Types」節、
    // EstimatorCommand.arg の CovScope と同じ流儀）。publishApiVerb は方向を
    // 持たない verb 専用（同ヘルパーのコメント参照）なので直接発行する。
    sf::api_command.publish({static_cast<uint8_t>(sf::ApiCmd::Flip),
                             static_cast<uint8_t>(mode),
                             static_cast<uint32_t>(esp_timer_get_time()),
                             static_cast<uint8_t>(dir)});
    waitFlipOutcome();
}

// -----------------------------------------------------------------------------
// cmdRc — Tello `rc a b c d` continuous manual control (a=roll/right+, b=pitch/
// forward+, c=throttle/up+, d=yaw/cw+, each -100..100). FIRE-AND-FORGET: no reply,
// no blocking (djitellopy blasts rc at high rate and never reads a response). It
// publishes a VELOCITY guidance target (mode 2) that rides POS_HOLD's stick-velocity
// reposition path; moving the pilot's RC stick still cancels it instantly (INV-2).
// Ignored unless FLYING with an active API session (re-engage via takeoff/go after
// a pilot override — M-3). A stopped stream auto-releases to hold (R16, controller).
// cmdRc — Tello `rc a b c d` 連続マニュアル操作（a=右ロール+, b=前ピッチ+, c=上スロットル+,
// d=cwヨー+、各 -100..100）。撃ちっぱなし: 応答なし・ブロックなし（djitellopy は高レートで
// 送り続け応答を読まない）。速度誘導目標（mode 2）を publish し POS_HOLD のスティック速度
// 再配置経路に乗る。パイロットの RC スティック操作で即解除（INV-2）。FLYING かつ API セッション
// 有効時のみ有効（パイロット介入後は takeoff/go で再係合 — M-3）。送信停止時は保持へ自動復帰（R16）。
void cmdRc(float a, float b, float c, float d)
{
    if (currentState() != sf::FlightState::FLYING || !g_target_valid) {
        return;   // fire-and-forget: silently ignore off-state / 状態外は黙って無視
    }
    auto clamp100 = [](float v) {
        if (v >  100.0f) return  100.0f;
        if (v < -100.0f) return -100.0f;
        return v;
    };
    a = clamp100(a); b = clamp100(b); c = clamp100(c); d = clamp100(d);

    // Full-scale velocities from the live POS_HOLD params (single source).
    // 満舵速度は POS_HOLD の param からライブ取得（単一源）。
    float vmax = kRcHorizVelDef;    sf::params::get_float("position.stick_vel", vmax);
    float vup  = kRcClimbRateDef;   sf::params::get_float("altitude.climb_rate", vup);
    float vdn  = kRcDescentRateDef; sf::params::get_float("altitude.descent_rate", vdn);

    sf::GuidanceTarget t{};
    t.mode = 2;                                   // velocity guidance / 速度誘導
    t.vx   = (b / 100.0f) * vmax;                 // body forward [m/s] / 機体前後
    t.vy   = (a / 100.0f) * vmax;                 // body right   [m/s] / 機体左右
    t.vz   = (c >= 0.0f) ? (c / 100.0f) * vup     // up   [m/s] / 上昇
                         : (c / 100.0f) * vdn;    // down [m/s] / 降下
    t.vyaw = (d / 100.0f) * kRcYawRateMax;        // yaw rate cw+ [rad/s] / ヨーレート

    // Altitude ceiling: at or above it, an upward command is dropped to zero climb (the
    // craft holds; descending and horizontal motion stay available). Altitude = -NED z.
    // 高度上限: 上限以上では上昇指令を上昇ゼロに落とす（保持。降下・水平移動は可能）。
    // 高度 = −NED z。
    const float altitude_m = -sf::estimate_state.latest().position[2];
    if (t.vz > 0.0f && altitude_m >= kAltMaxM) {
        t.vz = 0.0f;
    }
    t.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::command_target.publish(t);
    // No reply — Tello rc is fire-and-forget. / 応答なし（rc は撃ちっぱなし）。
}

// -----------------------------------------------------------------------------
// Queries (Tello formats) / クエリ（Tello 形式）
// -----------------------------------------------------------------------------

void cmdQuery(const char* what)
{
    char buf[96];
    if (std::strcmp(what, "battery?") == 0) {
        std::snprintf(buf, sizeof(buf), "%d", batteryPercent());
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
        // Tello `speed?` returns the SET cruise speed [cm/s] (not the live ground
        // speed — that is the state stream's vgx/vgy/vgz / get_speed_x()).
        // Tello `speed?` は設定した巡航速度 [cm/s] を返す（実速度は状態ストリームの vgx/y/z）。
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(g_default_speed * 100.0f));
    } else if (std::strcmp(what, "sdk?") == 0) {
        // Report SDK 2.0 — the version djitellopy targets for the broadest set.
        // SDK 2.0 を返す — djitellopy が最大互換で前提にするバージョン。
        std::snprintf(buf, sizeof(buf), "20");
    } else if (std::strcmp(what, "sn?") == 0) {
        // Serial number from the STATION MAC tail (stable per board) -- the same
        // identity as the pairing label and the SoftAP SSID.
        // ステーション側 MAC 末尾由来のシリアル（ボード毎に一定）。ペアリング用
        // ラベル・SoftAP SSID と同じ識別子。
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        std::snprintf(buf, sizeof(buf), "STAMPFLY-%02X%02X%02X",
                      mac[3], mac[4], mac[5]);
    } else if (std::strcmp(what, "time?") == 0) {
        std::snprintf(buf, sizeof(buf), "%d", g_flight_time_s.load());
    } else if (std::strcmp(what, "wifi?") == 0) {
        // No live SNR source on the AP side — report a strong constant so programs
        // that gate on signal quality proceed. / AP 側に SNR 源がないため強信号定数を返す。
        std::snprintf(buf, sizeof(buf), "90");
    } else if (std::strcmp(what, "tof?") == 0) {
        const float tof_cm = sf::sensor_snapshot.latest().tof_distance * 100.0f;
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(tof_cm));
    } else if (std::strcmp(what, "temp?") == 0) {
        const int t = static_cast<int>(sf::sensor_imu.latest().temperature);
        std::snprintf(buf, sizeof(buf), "%dC", t);
    } else if (std::strcmp(what, "baro?") == 0) {
        std::snprintf(buf, sizeof(buf), "%.2f",
                      static_cast<double>(sf::sensor_snapshot.latest().baro_altitude));
    } else if (std::strcmp(what, "acceleration?") == 0) {
        // Tello 1.3 `acceleration?` → "agx:.. agy:.. agz:.." in 0.001 g (mg).
        const sf::ImuData d = sf::sensor_imu.latest();
        const float mg = 1000.0f / 9.80665f;   // m/s² → 0.001 g
        std::snprintf(buf, sizeof(buf), "agx:%.2f;agy:%.2f;agz:%.2f;",
                      static_cast<double>(d.accel[0] * mg),
                      static_cast<double>(d.accel[1] * mg),
                      static_cast<double>(d.accel[2] * mg));
    } else {
        std::snprintf(buf, sizeof(buf), "error unknown query");
    }
    reply(buf);
}

// -----------------------------------------------------------------------------
// gatherTelloState — snapshot the firmware topics into the Tello state fields.
// All physical-unit conversions live here (rad→deg, NED m/s→body cm/s, m→cm,
// accel→milli-g) so the builder (tello_state.hpp) stays pure formatting. Reads
// only Latest/RingBuffer topics (no Queue) so it never steals sensor samples.
// gatherTelloState — ファームのトピックを Tello 状態フィールドへスナップショット。物理単位
// 変換（rad→deg, NED m/s→機体系 cm/s, m→cm, 加速度→ミリ g）はここに集約し、ビルダ
// （tello_state.hpp）は純粋整形に保つ。Latest/RingBuffer のみ読み（Queue 不可）でサンプルを奪わない。
sf::tello::TelloStateInputs gatherTelloState()
{
    sf::tello::TelloStateInputs in{};
    const sf::StateEstimate e = sf::estimate_state.latest();

    // Attitude (quaternion → Euler, deg). Same extraction as `attitude?`.
    // 姿勢（四元数 → オイラー, deg）。`attitude?` と同じ抽出。
    const float w = e.attitude[0], x = e.attitude[1];
    const float y = e.attitude[2], z = e.attitude[3];
    const float r2d = 57.29578f;
    in.roll = static_cast<int>(atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * r2d);
    float sinp = 2*(w*y - z*x);
    if (sinp >  1) sinp =  1;
    if (sinp < -1) sinp = -1;
    in.pitch = static_cast<int>(asinf(sinp) * r2d);
    const float yaw = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    in.yaw = static_cast<int>(yaw * r2d);

    // Ground speed: NED velocity rotated into the body frame (x fwd, y right, z up).
    // NOTE: sign/frame vs djitellopy needs an empirical check on hardware.
    // 対地速度: NED 速度を機体系（x前/y右/z上）へ回転。符号/系は実機で djitellopy と要照合。
    const float cy = cosf(yaw), sy = sinf(yaw);
    in.vgx = static_cast<int>(( cy*e.velocity[0] + sy*e.velocity[1]) * 100.0f);
    in.vgy = static_cast<int>((-sy*e.velocity[0] + cy*e.velocity[1]) * 100.0f);
    in.vgz = static_cast<int>(-e.velocity[2] * 100.0f);   // up positive / 上を正

    // Height [cm] (= −Down). ToF [cm] from the async-sensor snapshot.
    // 高度 [cm]（= −Down）。ToF [cm] は非同期センサのスナップショットから。
    in.h   = static_cast<int>(-e.position[2] * 100.0f);
    in.tof = static_cast<int>(sf::sensor_snapshot.latest().tof_distance * 100.0f);

    // Barometer altitude in METERS — djitellopy get_barometer() multiplies by 100
    // to report cm, so the wire value is meters (matches the real Tello).
    // 気圧高度は「メートル」— djitellopy get_barometer() が ×100 して cm にするので電文は m。
    in.baro = sf::sensor_snapshot.latest().baro_altitude;

    // IMU temperature (one sensor → templ == temph). Acceleration in milli-g.
    // IMU 温度（1センサ → templ==temph）。加速度はミリ g。
    const sf::ImuData imu = sf::sensor_imu.latest();
    in.templ = in.temph = static_cast<int>(imu.temperature);
    const float mg = 1000.0f / 9.80665f;   // m/s² → 0.001 g
    in.agx = imu.accel[0] * mg;
    in.agy = imu.accel[1] * mg;
    in.agz = imu.accel[2] * mg;

    in.bat    = batteryPercent();
    in.time_s = g_flight_time_s.load(std::memory_order_relaxed);
    return in;
}

// -----------------------------------------------------------------------------
// cmdAutotune — onboard rate-loop autotune: stepped-sine sweep → plant fit →
// phase-margin PID design → LIVE application (params, not persisted).
// cmdAutotune — オンボード自動チューン: ステップドサイン掃引 → プラントフィット →
// 位相余裕 PID 設計 → ライブ適用（params。NVS には保存しない）。
//
// Safety gates / 安全判定:
//  - FLYING hover only; each frequency point is a bounded, clamped excitation.
//  - The result is applied ONLY if: enough effective-coherent points, the fit
//    residual is small, the plant lands in PHYSICAL bounds (b vs the spec inertia;
//    motor lag — yaw allows T→0), and the verified margins meet the spec (GM floor,
//    PM) + the param-table range check. Otherwise the OLD gains remain untouched and
//    the reply says why. (A Kp-vs-CURRENT-gain "no wild jumps" gate was REMOVED — its
//    reference was circular for an untuned axis; kp≈wc/b is already physically bounded
//    by the b-range gate. All gates are axis-uniform except yaw's T→0 allowance.)
//    適用条件は物理/絶対基準のみ（残差・b 物理境界・余裕仕様・param範囲）。現ゲイン比判定は削除。
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
    // Values follow control/models/stampfly_physical.yaml (2026-11-23: computed from the
    // part STL meshes and measured masses; before: 2-point suspension 9.16/13.3/20.4e-6).
    // 値は control/models/stampfly_physical.yaml に従う（2026-11-23: 部品の STL と実測質量
    // からの試算値。以前は2点吊り法の 9.16/13.3/20.4e-6）。
    static const float kSpecInertia[3] = {11.5e-6f, 15.3e-6f, 22.6e-6f};
    static const float kFreqsHz[] = {2.0f, 3.0f, 4.5f, 7.0f, 10.0f,
                                     14.0f, 20.0f, 27.0f, 35.0f};
    constexpr int kNumFreqs = sizeof(kFreqsHz) / sizeof(kFreqsHz[0]);
    constexpr float kAmpRadps = 0.35f;   // ~20 dps per point / 点あたり約20dps
    // A lock-in response below this amplitude [rad/s] is a dead/untrusted tone (coh=0).
    // この振幅未満のロックイン応答は無応答の周波数点とみなす（coh=0）。約0.57 dps。
    constexpr float kMinOnAmp = 0.01f;

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
            [last_seq] { return sf::sysid_result.latest().seq != last_seq; }, true);
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
        // A DEAD tone (excitation never reached the loop / saturation / disarm-edge) is the
        // LEAST trustworthy case ⇒ coh=0 (NOT 1, else it would inflate the fit's sufficiency
        // guard). Floor on the absolute lock-in AMPLITUDE (N-independent), clamp, NaN→0.
        // コヒーレンス/SNR代理: オン周波数点電力/(オン周波数点+オフ周波数点雑音床)。無応答の周波数点は最も信頼度を低く扱う→coh=0。
        const float on_pow = res.yr * res.yr + res.yi * res.yi;
        const float on_amp = (res.samples > 0)
                           ? 2.0f * sqrtf(on_pow) / static_cast<float>(res.samples) : 0.0f;
        float coh = (on_pow + res.off_power > 1e-20f)
                  ? on_pow / (on_pow + res.off_power) : 0.0f;
        if (on_amp < kMinOnAmp) coh = 0.0f;   // weak/dead tone ⇒ untrusted regardless of ratio
        if (!(coh >= 0.0f))     coh = 0.0f;   // NaN ⇒ 0
        if (coh > 1.0f)         coh = 1.0f;
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
        // NOTE: do NOT re-cue here. Re-publishing AutotuneStart after
        // every point would keep the buzzer audible; but each cue plays a ~0.9 s BLOCKING
        // warble in NotifyTask, and the high-freq points (~0.8 s) would out-pace it, FILLING
        // the depth-8 notify_command queue (which drops on overflow) — so the final
        // AutotuneOk/Fail cue would be silently dropped and yaw would show white-then-nothing.
        // The white LED already persists for the whole sweep via its 30 s safety
        // timeout, so the end cue (green/red) just needs the queue clear here.
        // ここで再合図しない。各点で AutotuneStart を再発行すると、各合図は NotifyTask で
        // 約0.9秒のブロッキング・ワーブルを鳴らし、高周波点(約0.8秒)が追い越して深さ8の
        // notify_command キューを満杯化(満杯時ドロップ)→ 最後の Ok/Fail 合図が落ち、yaw が
        // 「白→無」になる。白LEDは開始時の30秒タイマで掃引中ずっと維持されるため、
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
    // 残り、yaw が変化なしに見えていた。設計のみ残差で判定する。
    if (plant.b > 0.0f) {
        char pk[40];
        std::snprintf(pk, sizeof(pk), "autotune.%s.b",     kAxisName[axis]); sf::params::set_float(pk, plant.b);
        std::snprintf(pk, sizeof(pk), "autotune.%s.tau",   kAxisName[axis]); sf::params::set_float(pk, plant.T);
        std::snprintf(pk, sizeof(pk), "autotune.%s.delay", kAxisName[axis]); sf::params::set_float(pk, plant.L);
        std::snprintf(pk, sizeof(pk), "autotune.%s.resid", kAxisName[axis]); sf::params::set_float(pk, plant.residual);
    }
    // Reject-reason code, saved at EVERY exit so `param get autotune.<axis>.reject` shows
    // WHY a hands-free (scheduled) tune was rejected — there is no serial log in flight.
    // 0=applied, 1=insufficient coherent data, 2=bad/NaN fit, 3=residual>0.3, 4=out of
    // physical bounds, 5=design infeasible (wc too high), 6=phase margin below target,
    // 7=gain margin below floor.
    // 棄却理由コード（飛行中はシリアル無し→`param get autotune.<axis>.reject` で確認）。
    char rk_rej[40];
    std::snprintf(rk_rej, sizeof(rk_rej), "autotune.%s.reject", kAxisName[axis]);

    // Defense-in-depth before the hands-free LIVE apply: the residual is coherence-WEIGHTED
    // and thus spoofable by a degenerate coh distribution, so gate ALSO on a residual-
    // INDEPENDENT physical-bounds check — a noise fit usually lands outside [0.25,4]×b_seed
    // or a sane motor lag. (fitPlant already rejects too-few-effective-points via fit_ok.)
    // A solo pilot cannot abort, so a bad gain must never reach set_float.
    // ハンズフリー適用前の多重防御: 残差はコヒーレンス重みゆえ偽装可 → 残差非依存の物理境界でも防御。
    //
    // YAW EXCEPTION on the motor-lag bound: roll/pitch (thrust differential) show a real
    // ~31 ms motor pole, so T→0 there means a bad fit → require T≥2 ms. YAW does NOT: its
    // torque carries a reaction term (I_r·dω/dt) whose minimum-phase (LHP) zero
    // near-cancels the motor pole, leaving an integrator+delay with NO identifiable pole in
    // band (flight data: T→0; the individual zero/pole sit below the 2 Hz lowest tone and
    // are unidentifiable — and irrelevant, the wc≈2.9 Hz crossover is above them). So for
    // yaw, T→0 is the VALID design model, not a degenerate fit; allow T from 0.
    // ヨー例外: roll/pitch は実モータ極(~31ms)があり T→0 は不良フィット → T≥2ms 要求。ヨーは反トルク
    // の最小位相零点がモータ極を相殺し、帯域内に同定可能な極が無い積分器+遅れ(T→0が正しい設計モデル)。
    const float b_lo = 0.25f / kSpecInertia[axis];
    const float b_hi = 4.0f  / kSpecInertia[axis];
    const float t_lo = (axis == 2) ? 0.0f : 0.002f;   // yaw: integrator+delay, T→0 valid
    const bool phys_ok = plant.b >= b_lo && plant.b <= b_hi
                       && plant.T >= t_lo && plant.T <= 0.080f;
    if (!fit_ok || !(plant.residual <= 0.3f) || !phys_ok) {   // !(<=) also rejects NaN
        const int rej = !fit_ok ? plant.fit_reject                  // 1 or 2
                      : !(plant.residual <= 0.3f) ? 3 : 4;
        sf::params::set_float(rk_rej, static_cast<float>(rej));
        ESP_LOGW(TAG, "Autotune fit rejected (code %d): residual=%.3f coh_sum=%.1f b=%.0f T=%.1fms phys=%d",
                 rej, static_cast<double>(plant.residual), static_cast<double>(plant.coh_sum),
                 static_cast<double>(plant.b), static_cast<double>(plant.T * 1e3),
                 static_cast<int>(phys_ok));
        reply("error fit rejected - gains unchanged (read autotune.* params)");
        return;
    }


    // Save the margins of the CURRENT (active) gains scored against the freshly
    // identified plant — so the stored wc/pm/gm ALWAYS reflect what is ACTUALLY flying.
    // If the design below is APPLIED, these are overwritten with the new gains' margins;
    // if it is REJECTED, the current gains' margins remain (answers "how safe are the
    // gains I am still flying, really?" — exactly what a rejected axis needs).
    // 現（実効）ゲインの余裕を新同定プラントで採点して保存 — 保存 wc/pm/gm は常に「実際に
    // 飛んでいる」ゲインを反映。下の設計が適用されれば新ゲインの余裕で上書き、棄却されれば
    // 据置ゲインの余裕が残る（棄却軸で知りたいのは「今飛んでいるゲインの本当の安全余裕」）。
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
    if (!sf::autotune::tunePid(plant, wc, pm_deg, 10.0f, tune)) {
        // Required controller lead exceeds the PID's maximum ⇒ wc is too high for this plant.
        sf::params::set_float(rk_rej, 5.0f);
        reply("error tune infeasible (lower wc) - gains unchanged");
        return;
    }
    // Accept the achieved PM if it MEETS-OR-EXCEEDS the target (a higher PM is more damped =
    // SAFER). Reject only an UNDER-margin design (PM below target). An equality gate
    // |PM-target|>5 would wrongly reject an over-damped design — yaw at wc=18 lands at
    // PM~80 (PI-only already exceeds 60, so td clips to 0), which is safe, not a failure.
    // 達成PMが目標以上なら採用（高PM=より減衰=安全）。目標未満だけ棄却。等値判定は過減衰設計を
    // 誤棄却してしまう（yaw wc=18 は PM~80 で安全）。
    if (tune.pm_deg < pm_deg - 5.0f) {
        sf::params::set_float(rk_rej, 6.0f);
        reply("error phase margin below target - gains unchanged");
        return;
    }

    // (Design margins are saved as the CURRENT gains' margins above, and overwritten
    // with these NEW gains' margins ONLY if the design passes the gates and is applied
    // below — so a rejected axis keeps the margins of the gains it is still flying.)
    // （設計余裕は上で現ゲインの余裕として保存済み。下の判定を通過し適用された場合のみ
    // この新ゲインの余裕で上書き — 棄却軸は据置ゲインの余裕を保持する。）

    // Gain-margin floor: a design that meets the phase margin can still be too
    // close to gain-side instability (thin GM), and model error / nonlinear
    // torque effectiveness then drives oscillation. Reject thin-GM designs
    // before LIVE apply (H-1). gm_valid=false means no −180° crossing in the
    // sweep → GM effectively infinite → SAFE, so do not reject that (L-15).
    // ALL axes use 6 dB. (Yaw needs no larger floor for a hypothesised reaction-torque RHP
    // zero — refuted on hardware, tau_z≈0; the yaw plant is the same uniform 3-param model
    // and its GM is trustworthy.)
    // 全軸 6dB。（yaw に反トルク RHP 零点を見込んだ大きめの下限（8dB 等）は不要: 実機で反証済み(tau_z≈0)。
    // yaw も同じ3パラモデルで GM は信頼できる。）
    static const float kMinGmDb[3] = {6.0f, 6.0f, 6.0f};   // roll, pitch, yaw [dB]
    if (tune.gm_valid && tune.gm_db < kMinGmDb[axis]) {
        sf::params::set_float(rk_rej, 7.0f);
        ESP_LOGW(TAG, "Autotune %s gain margin %.1f dB < floor %.1f dB",
                 kAxisName[axis], static_cast<double>(tune.gm_db),
                 static_cast<double>(kMinGmDb[axis]));
        reply("error gain margin too low - gains unchanged");
        return;
    }

    // (REMOVED) the "no wild jumps vs the flying gain" kp-range gate [tune.kp must lie in
    // 0.25..4× the CURRENT gain]. Its reference was the current flying gain, which has no
    // claim to validity for an UNTUNED axis — it blocked the legitimate ~5× yaw kp
    // reduction exactly when autotuning mattered most (a circular reference: it assumed
    // the gain the autotune is trying to determine). The designed kp ≈ wc/b is ALREADY
    // physically bounded by the b-range gate above (b ∈ [0.25,4]× 1/Ispec ⇒ kp ∈
    // [0.25,4]× wc/Ispec), and the GM-floor + residual + coherence gates validate the
    // design — those are the real, physically-grounded safety. See the gate audit.
    // （削除）「現飛行ゲイン比4倍以内」判定: 基準が現ゲインで循環・未調整軸(yaw)で正当な補正を阻害。
    // 設計 kp≈wc/b は上の b 物理境界＋GM下限＋残差で既に物理的に束縛済み。
    char key_kp[32], key_ti[32], key_td[32];
    std::snprintf(key_kp, sizeof(key_kp), "rate.%s.kp", kAxisName[axis]);
    std::snprintf(key_ti, sizeof(key_ti), "rate.%s.ti", kAxisName[axis]);
    std::snprintf(key_td, sizeof(key_td), "rate.%s.td", kAxisName[axis]);

    // Apply LIVE (set_float validates the table ranges and fires ReloadParams).
    // ライブ適用（set_float がテーブル範囲を検証し ReloadParams を作動させる）。
    if (!sf::params::set_float(key_kp, tune.kp) ||
        !sf::params::set_float(key_ti, tune.ti) ||
        !sf::params::set_float(key_td, tune.td)) {
        sf::params::set_float(rk_rej, 8.0f);   // param-table range rejected the gain
        reply("error param range rejected - check param table");
        return;
    }
    sf::params::set_float(rk_rej, 0.0f);   // 0 = APPLIED / 適用成功
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
// Commands with numbers. Every number parsed from a line is checked at this boundary
// (sf::api_arguments) before a command uses it; a non-finite or out-of-range number is
// answered kOutOfRangeReply (`rc`, fire-and-forget, is ignored silently).
// 数を伴うコマンド。行から解析した数は、コマンドが使う前にこの境界で検査する
// （sf::api_arguments）。有限でない数・範囲外の数には kOutOfRangeReply を返す
// （撃ちっぱなしの `rc` は黙って無視する）。
// -----------------------------------------------------------------------------

// `speed x` — set the session cruise speed for the verb moves (Tello: 10-100 cm/s, clamped).
// `speed?` is a query and was already routed. Returns true when the line was a `speed` line.
// `speed x` — verb 移動の巡航速度を設定（Tello: 10〜100 cm/s、範囲外はクランプ）。
// `speed?` はクエリで先に振り分け済み。`speed` の行だったとき true。
bool handleSpeedLine(const char* line)
{
    float speed_cm_per_sec = 0.0f;
    if (std::sscanf(line, "speed %f", &speed_cm_per_sec) != 1) {
        return false;
    }
    if (!sf::api_arguments::allFinite({speed_cm_per_sec})) {
        reply(kOutOfRangeReply);
        return true;
    }
    if (speed_cm_per_sec < kSpeedMinCmPerSec) speed_cm_per_sec = kSpeedMinCmPerSec;
    if (speed_cm_per_sec > kSpeedMaxCmPerSec) speed_cm_per_sec = kSpeedMaxCmPerSec;
    g_default_speed = speed_cm_per_sec * 0.01f;   // cm/s → m/s
    reply("ok");
    return true;
}

// `rc a b c d` — continuous manual control (fire-and-forget). Parsed early since it arrives
// at a high rate. Returns true when the line was an `rc` line.
// `rc a b c d` — 連続マニュアル操作（撃ちっぱなし）。高レートで来るため先に解析。
// `rc` の行だったとき true。
bool handleRcLine(const char* line)
{
    float roll = 0, pitch = 0, throttle = 0, yaw = 0;
    if (std::sscanf(line, "rc %f %f %f %f", &roll, &pitch, &throttle, &yaw) != 4) {
        return false;
    }
    if (sf::api_arguments::allFinite({roll, pitch, throttle, yaw})) {
        cmdRc(roll, pitch, throttle, yaw);
    }
    return true;
}

// `cw <deg>` / `ccw <deg>` — Tello: cw = clockwise (viewed from above) = +yaw in NED; the angle
// is 1-360 degrees. Returns true when `verb` is a rotate verb.
// `cw <deg>` / `ccw <deg>` — Tello: cw = 上から見て時計回り = NED では +yaw。角度は 1〜360 度。
// `verb` が回頭の verb のとき true。
bool handleRotateVerb(const char* verb, float angle_degrees)
{
    const bool clockwise = std::strcmp(verb, "cw") == 0;
    if (!clockwise && std::strcmp(verb, "ccw") != 0) {
        return false;
    }
    if (!sf::api_arguments::isRotateAngleInRange(angle_degrees)) {
        reply(kOutOfRangeReply);
        return true;
    }
    cmdRotate((clockwise ? angle_degrees : -angle_degrees) * kDegreesToRadians, 0);
    return true;
}

// `up|down|left|right|forward|back <cm>` — one-axis move in the Tello body frame (x forward,
// y LEFT, z UP). Returns true when `verb` is a move verb.
// `up|down|left|right|forward|back <cm>` — Tello 機体座標（x前・y左・z上）の 1 軸移動。
// `verb` が移動の verb のとき true。
bool handleMoveVerb(const char* verb, float distance_cm)
{
    float forward_cm = 0, left_cm = 0, up_cm = 0;
    if      (std::strcmp(verb, "up") == 0)      { up_cm      =  distance_cm; }
    else if (std::strcmp(verb, "down") == 0)    { up_cm      = -distance_cm; }
    else if (std::strcmp(verb, "left") == 0)    { left_cm    =  distance_cm; }
    else if (std::strcmp(verb, "right") == 0)   { left_cm    = -distance_cm; }
    else if (std::strcmp(verb, "forward") == 0) { forward_cm =  distance_cm; }
    else if (std::strcmp(verb, "back") == 0)    { forward_cm = -distance_cm; }
    else { return false; }
    cmdMove(forward_cm, left_cm, up_cm, g_default_speed);
    return true;
}

// `go x y z speed` — x forward, y left, z up [cm], speed [cm/s] (Tello frame); the speed is
// clamped to 0.1-1.0 m/s.
// `go x y z speed` — x前・y左・z上 [cm]、speed [cm/s]（Tello 座標）。速度は 0.1〜1.0 m/s にクランプ。
void cmdGo(float forward_cm, float left_cm, float up_cm, float speed_cm_per_sec)
{
    if (!sf::api_arguments::allFinite({forward_cm, left_cm, up_cm, speed_cm_per_sec})) {
        reply(kOutOfRangeReply);
        return;
    }
    float speed = speed_cm_per_sec * 0.01f;
    if (speed < kGoSpeedMinMps) speed = kGoSpeedMinMps;
    if (speed > kGoSpeedMaxMps) speed = kGoSpeedMaxMps;
    cmdMove(forward_cm, left_cm, up_cm, speed);
}

// `sysid <roll|pitch|yaw> <doublet|chirp> <amp_dps> <dur_s>` — rate-loop identification
// excitation while hovering (see SysidCommand). Waits for the duration so the calling script
// brackets its log capture; the wait is abortable (`emergency`, `stop`, the end of the flight).
// The amplitude and duration must be finite, positive and within what the controller takes
// unchanged (sf::api_arguments), otherwise a mistyped number would hold ApiTask for minutes.
// `sysid <軸> <doublet|chirp> <振幅 dps> <継続 s>` — ホバー中のレートループ同定励振
// （SysidCommand 参照）。呼び出しスクリプトのログ取得区間と揃うよう継続時間だけ待つ。待機は
// 打ち切れる（`emergency`・`stop`・飛行の終わり）。振幅と継続時間は有限・正で、制御器が変更せずに
// 受け取る範囲内であること。さもないと打ち間違いの数が ApiTask を数分間止める。
void cmdSysid(const char* axis_name, const char* waveform_name, float amplitude_dps, float duration_s)
{
    if (currentState() != sf::FlightState::FLYING) {
        reply("error not flying");
        return;
    }
    uint8_t axis;
    if      (std::strcmp(axis_name, "roll") == 0)  axis = 0;
    else if (std::strcmp(axis_name, "pitch") == 0) axis = 1;
    else if (std::strcmp(axis_name, "yaw") == 0)   axis = 2;
    else { reply("error bad axis"); return; }
    uint8_t waveform;
    if      (std::strcmp(waveform_name, "doublet") == 0) waveform = 0;
    else if (std::strcmp(waveform_name, "chirp") == 0)   waveform = 1;
    else { reply("error bad waveform"); return; }
    if (!sf::api_arguments::isSysidRangeValid(amplitude_dps, duration_s)) {
        reply(kOutOfRangeReply);
        return;
    }

    sf::SysidCommand cmd{};
    cmd.axis      = axis;
    cmd.waveform  = waveform;
    cmd.amplitude = amplitude_dps * kDegreesToRadians;   // dps → rad/s
    cmd.duration  = duration_s;
    cmd.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::sysid_command.publish(cmd);

    const uint32_t wait_ms = static_cast<uint32_t>(duration_s * 1000.0f) + kSysidMarginMs;
    const uint32_t start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    const bool finished = waitUntil(wait_ms + kWaitPollMs, [start_ms, wait_ms] {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000) - start_ms >= wait_ms;
    }, true);
    reply(finished ? "ok" : waitFailureText("error sysid timeout"));
}

// `autotune <roll|pitch|yaw> [wc_radps] [pm_deg]` — onboard autotune. Returns true when the
// line was an `autotune` line.
// `autotune <軸> [ωc] [PM]` — オンボード自動チューン。`autotune` の行だったとき true。
bool handleAutotuneLine(const char* line)
{
    char axis_name[16] = {};
    float crossover = 0, phase_margin_deg = 0;
    if (std::strncmp(line, "autotune", 8) != 0 ||
        std::sscanf(line, "autotune %15s %f %f", axis_name, &crossover, &phase_margin_deg) < 1) {
        return false;
    }
    uint8_t axis;
    if      (std::strcmp(axis_name, "roll") == 0)  axis = 0;
    else if (std::strcmp(axis_name, "pitch") == 0) axis = 1;
    else if (std::strcmp(axis_name, "yaw") == 0)   axis = 2;
    else { reply("error bad axis"); return true; }
    if (!sf::api_arguments::allFinite({crossover, phase_margin_deg})) {
        reply(kOutOfRangeReply);
        return true;
    }
    // Default wc: 25 rad/s for roll/pitch; 18 for yaw. Yaw's empirically low control
    // authority + low-SNR identification make a conservative crossover the safe choice
    // (NOT a reaction-zero limit — that was refuted). An explicit wc arg overrides.
    // 既定 wc: roll/pitch=25、yaw=18（yaw は操縦権限が低く同定SNRも低いため保守的に。零点制限ではない）。
    const float wc_def = (axis == 2) ? 18.0f : 25.0f;
    const float wc = (crossover > 1.0f && crossover < 100.0f) ? crossover : wc_def;
    const float pm = (phase_margin_deg > 20.0f && phase_margin_deg < 80.0f) ? phase_margin_deg : 60.0f;
    cmdAutotune(axis, wc, pm);
    return true;
}

// True for the verbs that take one number: the six moves and the two rotations. Only these get
// the out-of-range reply; any other word stays an unknown command.
// 数を 1 つ取る verb（移動 6 つと回頭 2 つ）なら true。範囲外の応答はこれらだけで、
// それ以外の語は不明なコマンドのまま。
bool isOneNumberVerb(const char* verb)
{
    static const char* const kVerbs[] = {"up", "down", "left", "right", "forward", "back", "cw", "ccw"};
    for (const char* known : kVerbs) {
        if (std::strcmp(verb, known) == 0) {
            return true;
        }
    }
    return false;
}

// Lines of the form `<verb> <numbers...>` other than `speed` / `rc`: moves, rotation, `go`,
// `autotune`, `sysid`. Returns true when the line was one of them (answered or executed).
// `<verb> <数...>` の形の行（`speed` / `rc` 以外）: 移動・回頭・`go`・`autotune`・`sysid`。
// そのどれかだったとき true（応答または実行済み）。
bool handleNumberedLine(const char* line)
{
    char verb[16] = {};
    char waveform_name[16] = {};
    float a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(line, "sysid %15s %15s %f %f", verb, waveform_name, &a, &b) == 4) {
        cmdSysid(verb, waveform_name, a, b);
        return true;
    }
    if (handleAutotuneLine(line)) {
        return true;
    }
    const int parsed = std::sscanf(line, "%15s %f %f %f %f", verb, &a, &b, &c, &d);
    if (parsed == 2 && isOneNumberVerb(verb)) {
        if (!sf::api_arguments::allFinite({a})) {
            reply(kOutOfRangeReply);
            return true;
        }
        return handleRotateVerb(verb, a) || handleMoveVerb(verb, a);
    }
    if (parsed == 5 && std::strcmp(verb, "go") == 0) {
        cmdGo(a, b, c, d);
        return true;
    }
    return false;
}

// Lines that are a fixed word and carry no number to check: the camera toggles, `flip`, the mission pads.
// Returns true when the line was one of them (answered or executed).
// 数の検査が要らない、決まった語の行: カメラの切り替え・`flip`・ミッションパッド。
// そのどれかだったとき true（応答または実行済み）。
bool handleFixedVerbLine(const char* line)
{
    // Camera commands — StampFly has no camera, but reply "ok" so video-aware Tello
    // programs that merely toggle the stream (without reading frames) keep running.
    // The 11111 video port is never opened; get_frame_read() would just see nothing.
    // カメラ系 — StampFly にカメラは無いが、ストリームを切り替えるだけ（フレームを読まない）の
    // Tello プログラムが止まらないよう "ok" を返す。映像ポート 11111 は開かない。
    if (std::strcmp(line, "streamon") == 0 || std::strcmp(line, "streamoff") == 0) {
        reply("ok");
        return true;
    }
    // flip <l/r/f/b> — Tello-compatible single-axis 360-degree flip
    // (flip-maneuver-plan.md §4.4). Any malformed "flip ..." line also lands here
    // (prefix match) so it reports "bad direction" rather than falling
    // through to "unknown command".
    // flip <l/r/f/b> — Tello 互換の単軸360度宙返り（plan §4.4）。
    // 不正な "flip ..." 行も（前方一致で）ここに来るため "unknown command" でなく
    // "bad direction" を返す。
    if (std::strncmp(line, "flip", 4) == 0) {
        char dir_tok[8] = {};
        std::sscanf(line, "flip %7s", dir_tok);
        cmdFlip(dir_tok);
        return true;
    }
    // Mission pads — a Tello EDU/RoboMaster-TT-only feature we do not implement.
    // ミッションパッドは EDU/RoboMaster TT 専用機能で未対応。
    if (std::strcmp(line, "mon") == 0 || std::strcmp(line, "moff") == 0 ||
        std::strncmp(line, "mdirection", 10) == 0) {
        reply("error mission pads not supported");
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// processLine — parse + execute one command line (shared HW / SILS path).
// processLine — 1 コマンド行の解析＋実行（実機 / SILS 共通経路）。
// -----------------------------------------------------------------------------
void processLine(char* line)
{
    // Trim trailing whitespace/newlines / 末尾の空白・改行を除去
    if (trimLine(line) == 0) return;
    ESP_LOGI(TAG, "cmd: %s", line);

    // Emergency first — never behind any other gate, the command-mode gate included.
    // emergency は最優先 — コマンドモードの判定を含め、他の判定の後ろに置かない。
    if (std::strcmp(line, "emergency") == 0) { cmdEmergency(); return; }
    if (std::strcmp(line, "command") == 0) {
        g_sdk_mode = true;
        reply("ok");
        return;
    }
    if (!g_sdk_mode) {
        reply("error not in command mode");
        return;
    }

    if (std::strcmp(line, "takeoff") == 0) { cmdTakeoff(); return; }
    if (std::strcmp(line, "land") == 0)    { cmdLand();    return; }
    if (std::strcmp(line, "stop") == 0)    { (void)cmdStop(); return; }
    if (std::strchr(line, '?') != nullptr) { cmdQuery(line); return; }

    if (handleFixedVerbLine(line) || handleSpeedLine(line) || handleRcLine(line)) {
        return;
    }
    if (handleNumberedLine(line)) {
        return;
    }
    reply("error unknown command");
}

// Run the commands parked by dispatchDuringWait, oldest first, before anything newer is read
// (keeps the arrival order). A line may block and park more; those run in this same loop.
// dispatchDuringWait が後回しにした行を、新しい行を読む前に古いものから実行する（到着順を
// 保つ）。行が待機して更に後回しを積むことがあり、それも同じループで実行される。
void runDeferredCommands()
{
    char line[kInjectLen];
    while (g_deferred_tail < g_deferred_head) {
        const DeferredLine& parked = g_deferred[g_deferred_tail % kDeferredSlots];
        std::strncpy(line, parked.text, sizeof(line) - 1);
        line[sizeof(line) - 1] = '\0';
        const ReplyTarget target(parked.from_udp ? &parked.from : nullptr);   // own sender / 自分の送信元
        ++g_deferred_tail;
        processLine(line);
    }
}

}  // namespace

// =============================================================================
// SILS / test injection entry point (C linkage for the scenario engine).
// SILS / テスト注入の入口（シナリオエンジン用 C リンケージ）。
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

    // UDP server socket (non-blocking; the loop also drains SILS injections).
    // On the SILS host the shim socket simply never delivers datagrams.
    // UDP サーバソケット（ノンブロッキング。ループは SILS 注入も drain する）。
    // SILS ホストではシムのソケットにデータグラムが届かないだけ。
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
                                    (ax == 2) ? 18.0f : 25.0f, 60.0f);  // yaw lower wc (low authority/SNR); plays tones
                    }
                }
            }
        }

        // 1a. Commands parked while a blocking command ran, oldest first, unless the craft was
        // disarmed meanwhile (watchDisarmEdge drops them: DISARM wins over every parked line).
        // 1a. 待機コマンドの実行中に後回しにした行を古いものから。ただしその間に機体が DISARM
        // されていたら捨てる（watchDisarmEdge。DISARM は後回しのどの行よりも優先）。
        watchDisarmEdge();
        runDeferredCommands();

        // 1. SILS/test injections / SILS・テスト注入
        while (popInjected(line, sizeof(line))) {
            processLine(line);
        }

        // 2. UDP datagrams (one command per datagram, Tello-style)
        // 2. UDP データグラム（1 データグラム = 1 コマンド、Tello 流）
        sockaddr_in from = {};
        if (receiveDatagram(line, sizeof(line), from)) {
            g_client      = from;     // reply to this line's sender / この行の送信元へ返信
            g_have_client = true;
            processLine(line);
            continue;                 // drain quickly when commands queue up / コマンドが溜まっているときは素早く処理する
        }

        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}

// =============================================================================
// TelloStateTask — push the Tello state string to the connected client on
// UDP:8890 at 10 Hz. djitellopy's connect() REQUIRES at least one such packet,
// and every get_*() getter reads this stream. It runs in its OWN task so the
// stream keeps flowing while ApiTask BLOCKS in a move/autotune (ApiTask owns no
// control-path deadline). The send socket is the vehicle's OWN ephemeral socket
// (sending to <client>:8890 needs no local bind) so it never collides with the
// 8889 API server or the 8890 Data Stream server (`sf log wifi`).
// TelloStateTask — UDP:8890 で接続中クライアントへ Tello 状態文字列を 10Hz 送出。djitellopy
// の connect() はこのパケットが最低1個届くことを必須とし、全 get_*() がこれを読む。ApiTask が
// 移動/autotune でブロックしてもストリームを止めないよう独立タスクにする。送信は自前のエフェメラル
// ソケット（<client>:8890 への送信に bind 不要）ゆえ 8889 API サーバや 8890 データストリーム
// サーバ（`sf log wifi`）と衝突しない。
// =============================================================================
void TelloStateTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "TelloStateTask started");

    int sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGW(TAG, "Tello state socket() failed (errno=%d) — 8890 stream off", errno);
        vTaskDelete(nullptr);
        return;
    }

    int64_t    flying_since_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    char       buf[256];
    while (true) {
        // Flight time = whole seconds since FLYING began (0 while not flying).
        // 飛行時間 = FLYING 開始からの整数秒（非飛行中は 0）。
        if (currentState() == sf::FlightState::FLYING) {
            const int64_t now_us = esp_timer_get_time();
            if (flying_since_us == 0) flying_since_us = now_us;
            g_flight_time_s.store(static_cast<int>((now_us - flying_since_us) / 1000000),
                                  std::memory_order_relaxed);
        } else {
            flying_since_us = 0;
            g_flight_time_s.store(0, std::memory_order_relaxed);
        }

        // Push state once a client is known (it has sent ≥1 datagram on :8889).
        // クライアントが判明したら（:8889 へ ≥1 データグラム送信済み）状態を送出。
        if (g_client_known.load(std::memory_order_acquire)) {
            const sf::tello::TelloStateInputs in = gatherTelloState();
            const int len = sf::tello::buildTelloState(buf, sizeof(buf), in);
            if (len > 0 && len < static_cast<int>(sizeof(buf))) {
                sockaddr_in dst = {};
                dst.sin_family      = AF_INET;
                dst.sin_addr.s_addr = g_client_ip.load(std::memory_order_relaxed);
                dst.sin_port        = htons(sf::tello::kStatePort);
                ::sendto(sock, buf, static_cast<size_t>(len), 0,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));   // 10 Hz
    }
}
