/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file rtos_smoke.cpp
 * @brief P1.1 smoke test — run the real firmware tasks on the host RTOS
 *        emulator and show a deterministic schedule.
 *        P1.1 スモークテスト — 本体タスクをホスト RTOS エミュレータ上で
 *        走らせ、決定論的なスケジュールを示す。
 *
 * Creates the unmodified ImuTask / ControlTask / StateTask and runs them under
 * the cooperative scheduler. ImuTask (400Hz) drives ControlTask via a task
 * notification; StateTask is event-driven (parks). A scripted scenario injects
 * a FLYING mode + hover setpoint at 100 ms so ControlTask starts computing.
 *
 * 無改変の ImuTask / ControlTask / StateTask を生成し、協調スケジューラ上で
 * 走らせる。ImuTask（400Hz）がタスク通知で ControlTask を駆動; StateTask は
 * イベント駆動（待機）。台本シナリオが 100 ms で FLYING モード＋ホバー
 * セットポイントを注入し、ControlTask が計算を始める。
 *
 * Pass/criteria (P1.1 gate): the schedule is the expected multi-rate cadence
 * and is identical across runs (a trace hash printed for comparison).
 *
 * 合格基準（P1.1 ゲート）: スケジュールが期待の多レートのリズムで、実行間で
 * 同一（比較用にトレースのハッシュを表示）。
 */

#include <cstdint>
#include <cstdio>

#include "scheduler.hpp"
#include "topics.hpp"
#include "params.hpp"
#include "data_types.hpp"
#include "flight_state.hpp"
#include "config.hpp"

// Firmware task functions (unmodified) and the IMU→Control notification handle.
// 本体タスク関数（無改変）と IMU→Control 通知ハンドル。
void ImuTask(void*);
void ControlTask(void*);
void StateTask(void*);
extern TaskHandle_t g_control_task_handle;

using sil::rtos::Scheduler;

namespace {

constexpr int64_t kSimDurationUs = 500000;   // 0.5 s of virtual time
constexpr int64_t kArmTimeUs     = 100000;   // inject FLYING at 100 ms

// FNV-1a hash of the schedule trace — same across runs iff deterministic.
// スケジュールトレースの FNV-1a ハッシュ — 決定論的なら実行間で同一。
uint64_t trace_hash(const std::vector<sil::rtos::TraceEvent>& trace)
{
    uint64_t hash = 1469598103934665603ULL;
    for (const auto& event : trace) {
        uint64_t mixed = static_cast<uint64_t>(event.time_us) * 31u + event.task_id;
        for (int byte = 0; byte < 8; ++byte) {
            hash ^= (mixed >> (byte * 8)) & 0xff;
            hash *= 1099511628211ULL;
        }
    }
    return hash;
}

// Scripted flight scenario: arm + hover setpoint at kArmTimeUs.
// 台本の飛行シナリオ: kArmTimeUs で ARM ＋ ホバーセットポイント。
void scenario(int64_t now_us)
{
    static bool injected = false;
    if (injected || now_us < kArmTimeUs) return;
    injected = true;

    sf::SystemMode mode = {};
    mode.state = static_cast<uint8_t>(sf::FlightState::FLYING);
    mode.armed = true;
    mode.timestamp = static_cast<uint32_t>(now_us);
    sf::system_mode.publish(mode);

    sf::CommandSetpoint setpoint = {};
    setpoint.throttle = 0.5f;
    setpoint.timestamp = static_cast<uint32_t>(now_us);
    sf::command_setpoint.publish(setpoint);

    fprintf(stderr, "[scenario] t=%lld us: injected FLYING + hover setpoint\n",
            static_cast<long long>(now_us));
}

}  // namespace

int main()
{
    // Bring up the firmware infrastructure the tasks rely on.
    // タスクが頼る本体インフラを立ち上げる。
    sf::params::init();
    sf::topics_init();

    Scheduler& scheduler = Scheduler::instance();
    scheduler.set_on_advance(scenario);

    // Create tasks (Control before Imu so the notify handle is wired first).
    // タスク生成（通知ハンドルを先に配線するため Control を Imu より先に）。
    const char* task_name[3] = {"StateTask", "ControlTask", "ImuTask"};
    TaskHandle_t h_state = nullptr, h_control = nullptr, h_imu = nullptr;
    xTaskCreatePinnedToCore(StateTask,   "StateTask",   config::STACK_STATE,
                            nullptr, config::PRIORITY_STATE,   &h_state,   1);
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", config::STACK_CONTROL,
                            nullptr, config::PRIORITY_CONTROL, &h_control, 1);
    g_control_task_handle = h_control;
    xTaskCreatePinnedToCore(ImuTask,     "ImuTask",     config::STACK_IMU,
                            nullptr, config::PRIORITY_IMU,     &h_imu,     1);

    // Run the cooperative loop.
    // 協調ループを回す。
    scheduler.run(kSimDurationUs);

    // ---- Report ----
    const auto& trace = scheduler.trace();
    uint32_t count[3] = {0, 0, 0};
    for (const auto& event : trace) {
        if (event.task_id < 3) count[event.task_id]++;
    }

    printf("[rtos_smoke] sim time=%.3f s, scheduled events=%zu\n",
           scheduler.now_us() / 1e6, trace.size());
    printf("[rtos_smoke] runs: StateTask=%u  ControlTask=%u  ImuTask=%u\n",
           count[0], count[1], count[2]);

    printf("[rtos_smoke] first 12 events (t_us : task):\n");
    for (size_t i = 0; i < trace.size() && i < 12; ++i) {
        const char* name = trace[i].task_id < 3 ? task_name[trace[i].task_id] : "?";
        printf("    %6lld : %s\n", static_cast<long long>(trace[i].time_us), name);
    }

    // Final pipeline outputs — prove the closed path produced values.
    // パイプライン最終出力 — 閉じた経路が値を生んだことを示す。
    sf::StateEstimate state = sf::estimate_state.latest();
    sf::ControlOutput control = sf::control_output.latest();
    sf::MotorOutput motors = sf::actuator_motor.latest();
    printf("[rtos_smoke] final estimate quat_w=%.3f pos_z=%.3f\n",
           state.attitude[0], state.position[2]);
    printf("[rtos_smoke] final control thrust=%.4f  motors=[%.3f %.3f %.3f %.3f]\n",
           control.thrust, motors.duty[0], motors.duty[1], motors.duty[2], motors.duty[3]);

    printf("[rtos_smoke] trace hash = 0x%016llx (identical across runs ⇒ deterministic)\n",
           static_cast<unsigned long long>(trace_hash(trace)));

    // Expected cadence: ImuTask every 2 ms over 0.5 s ⇒ ~250 runs. Control must
    // match ImuTask EXACTLY (strict lock-step): each IMU cycle notifies Control
    // once and Control takes it. Requiring equality (not ≥-1) makes a silently
    // dropped/collapsed notification fail the gate.
    // 期待のリズム: ImuTask は 0.5 s で 2 ms ごと ⇒ 約250回。Control は ImuTask と
    // 厳密に一致（ロックステップ）すること: 各 IMU サイクルが Control を1回通知し
    // Control が受ける。≥-1 でなく等号を要求することで、静かに落ちた/まとめられた
    // 通知をゲートで弾く。
    const bool ok = (count[2] >= 240 && count[2] <= 260) && (count[1] == count[2]);
    printf("[rtos_smoke] %s — real tasks ran on the host RTOS emulator\n",
           ok ? "OK" : "UNEXPECTED cadence");

    // Clean teardown: scheduler.run() already unwound and joined every task
    // thread (stop_all), so a normal return runs static destructors safely.
    // クリーンな teardown: scheduler.run() が全タスクスレッドを巻き戻して join 済み
    // （stop_all）なので、通常 return で静的デストラクタが安全に走る。
    return ok ? 0 : 2;
}
