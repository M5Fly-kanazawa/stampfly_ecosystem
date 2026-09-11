/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_flightlog_vehicle.cpp
 * @brief firmware/vehicle topic glue for the flight-log bundle recorder —
 *        overrides emu_flightlog.cpp's weak hooks to read the REAL published
 *        topics (sf::sensor_imu, sf::estimate_state, sf::control_output,
 *        sf::actuator_motor, sf::command_setpoint, sf::system_mode,
 *        sf::sensor_power, sf::sensor_health, sf::sensor_snapshot) and write
 *        them as the "StampFly flight-log v1 bundle" streams
 *        (protocol/spec/flight_log.yaml).
 *        フライトログ一式レコーダの firmware/vehicle トピック glue —
 *        emu_flightlog.cpp の弱フックを上書きし、実発行トピックを読んで
 *        「StampFly フライトログ v1 一式」のストリームとして書く。
 *
 * Linked into emu_vehicle and emu_workshop (both run firmware/vehicle's
 * ImuTask/estimator/comm/telemetry topics — emu_workshop only swaps
 * ControlTask, see CMakeLists.txt), and into hover_smoke (which links the
 * same sf_core topics via sf_cores). NOT linked into emu_vehicle_old (the
 * frozen legacy firmware has none of these sf:: topics).
 * emu_vehicle と emu_workshop（どちらも firmware/vehicle の ImuTask/推定器/
 * comm/telemetry トピックを実行 -- emu_workshop は ControlTask のみ差し替え、
 * CMakeLists.txt 参照）、および hover_smoke（sf_cores 経由で同じ sf_core
 * トピックをリンク）にリンクする。emu_vehicle_old（凍結レガシーファーム）には
 * これらの sf:: トピックが無いためリンクしない。
 *
 * Every topic is read via `.latest()` (a non-destructive peek), NEVER
 * `.read()` (which would pop the shared single-consumer ring and race with
 * the real consumer) -- the same discipline the recorder this file replaces
 * (emu_rate_stream.cpp) used for sf::control_output/sf::sensor_imu.
 * 全トピックを `.latest()`（非破壊 peek）でのみ読み、`.read()`（共有リングを
 * 消費し実消費者と競合する）は使わない -- 本ファイルが置き換える旧レコーダ
 * （emu_rate_stream.cpp）が sf::control_output/sf::sensor_imu に使ったのと
 * 同じ規律。
 *
 * @design docs/plans/flight-log-format-plan.md §3.3 — SILS/vehicle unification [--]
 */

#include "emu_flightlog.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>   // std::getenv — re-derive the bundle dir for gains.json

#include "esp_system.h"    // esp_reset_reason() — host shim, see esp_idf_host/esp_system.h
#include "topics.hpp"       // sf::sensor_imu, sf::estimate_state, sf::control_output, ...
#include "data_types.hpp"   // sf::ImuData, sf::StateEstimate, sf::ControlOutput, ...
#include "params.hpp"       // sf::params::get_float — SSOT rate-loop gains

namespace {

constexpr int kCtrlRefDecimation = 8;    // 400Hz / 8 = 50Hz, matches CtrlRef's real rate
constexpr int kStatusDecimation  = 400;  // 400Hz / 400 = 1Hz, matches Status's real rate

// eskf_status bitmask literal. Not a live topic -- the real firmware's own
// Data Stream hardcodes the SAME literal (data_stream.cpp:410:
// "payload.eskf_status = 0x01; // estimator running"), so this mirrors it
// rather than inventing a new convention.
// eskf_status ビットマスク定数。ライブトピックではない -- 実ファーム自身の
// Data Stream も同じ定数をハードコードしている（data_stream.cpp:410）ため、
// 新しい規約を作らずそれを写す。
constexpr uint8_t kEskfStatusRunning = 0x01;

// Rate-loop output torque limit [Nm] for roll/pitch, mirrored from
// PidController::max_roll_pitch_torque_ (pid_controller.hpp) -- a fixed
// compile-time default, unlike rate.yaw.max_torque which IS a tunable param.
// Same mirror this recorder's predecessor (emu_rate_stream.cpp) used.
// ロール/ピッチのレートループ出力トルク上限[Nm]。PidController::
// max_roll_pitch_torque_ の固定コンパイル時既定値を写す（rate.yaw.max_torque
// とは異なりparam化されていない）。旧レコーダ（emu_rate_stream.cpp）と同じミラー。
constexpr float kRollPitchTorqueLimit = 5.2e-3f;   // [Nm] mirrors pid_controller.hpp:590

// --- Edge-detect state (one flight, one process -- plain statics are fine,
// same pattern as emu_rate_stream.cpp's g_last_ts). ---
// エッジ検出状態（1飛行1プロセスなので単純な static でよい。emu_rate_stream.cpp
// の g_last_ts と同じ流儀）。
uint32_t g_last_imu_ts   = 0;   // 0 = "no IMU sample yet" (never recurs once running)
uint32_t g_last_cmd_ts   = 0;   // pilot.csv edge detect
uint32_t g_last_ctrl_ts  = 0;   // rate_ref/ctrl_output edge detect (see write_rate_ref_and_ctrl_output_rows)
uint32_t g_last_motor_ts = 0;   // motor.csv edge detect (see write_motor_row)
uint32_t g_last_mag_ts   = 0;
uint32_t g_last_baro_ts  = 0;
uint32_t g_last_tof_ts   = 0;
uint32_t g_last_flow_ts  = 0;
int64_t  g_seq           = 0;   // running IMU-edge (control-cycle) counter, starts at 0
int64_t  g_ctrl_ref_edge = 0;   // counts GENUINELY fresh control_output publishes, for ctrl_ref decimation

// --- imu.csv / attitude.csv / posvel.csv (400Hz, keyed on the IMU edge) ---

void write_imu_row(int64_t ts, int64_t seq, const sf::ImuData& imu)
{
    std::FILE* f = sils_emu_flightlog_stream("imu",
        "timestamp_us,seq,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,"
        "gyro_raw_x,gyro_raw_y,gyro_raw_z,accel_raw_x,accel_raw_y,accel_raw_z");
    if (f == nullptr) return;
    // firmware/vehicle has no IMU-side LPF, so raw == filtered (sent for wire
    // compatibility with firmware that does filter — see flight_log.yaml).
    // vehicle は IMU 側 LPF を持たないため raw == filtered（フィルタを持つ
    // ファームとの電文互換のため両方送る）。
    std::fprintf(f,
        "%lld,%lld,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
        (long long)ts, (long long)seq,
        (double)imu.gyro[0], (double)imu.gyro[1], (double)imu.gyro[2],
        (double)imu.accel[0], (double)imu.accel[1], (double)imu.accel[2],
        (double)imu.gyro[0], (double)imu.gyro[1], (double)imu.gyro[2],
        (double)imu.accel[0], (double)imu.accel[1], (double)imu.accel[2]);
}

void write_attitude_row(int64_t ts, int64_t seq, const sf::StateEstimate& est)
{
    std::FILE* f = sils_emu_flightlog_stream("attitude",
        "timestamp_us,seq,quat_w,quat_x,quat_y,quat_z,"
        "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z");
    if (f == nullptr) return;
    std::fprintf(f, "%lld,%lld,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
        (long long)ts, (long long)seq,
        (double)est.attitude[0], (double)est.attitude[1],
        (double)est.attitude[2], (double)est.attitude[3],
        (double)est.gyro_bias[0], (double)est.gyro_bias[1], (double)est.gyro_bias[2],
        (double)est.accel_bias[0], (double)est.accel_bias[1], (double)est.accel_bias[2]);
}

void write_posvel_row(int64_t ts, int64_t seq, const sf::StateEstimate& est)
{
    std::FILE* f = sils_emu_flightlog_stream("posvel",
        "timestamp_us,seq,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z");
    if (f == nullptr) return;
    std::fprintf(f, "%lld,%lld,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
        (long long)ts, (long long)seq,
        (double)est.position[0], (double)est.position[1], (double)est.position[2],
        (double)est.velocity[0], (double)est.velocity[1], (double)est.velocity[2]);
}

// --- rate_ref.csv / ctrl_output.csv / motor.csv (400Hz, but only when the
// control loop actually published this cycle -- emu_workshop's
// WorkshopControlTask never publishes control_output). ---
// rate_ref.csv / ctrl_output.csv / motor.csv（400Hz。ただし制御ループがその
// 周期に実際に発行したときのみ -- emu_workshop の WorkshopControlTask は
// control_output を一切発行しない）。

// Returns true when a genuinely NEW control_output was published this edge (so
// the caller can decimate ctrl_ref.csv on real control cycles, not raw IMU
// edges). sf::control_output is a Latest(1) topic control_task.cpp only
// publishes on the ARMED path (control_task.cpp:376) -- when disarmed it
// publishes a zeroed sf::log_stream record instead (control_task.cpp:313) but
// leaves sf::control_output untouched, so .latest() keeps returning the SAME
// value forever once the control loop stops. Re-reading and re-writing that
// frozen value on every later IMU edge would be exactly the "fill with the
// previous value" anti-pattern flight_log.yaml's csv_rules forbid, so this
// also edge-detects on ctrl.timestamp CHANGING -- the same discipline this
// recorder's predecessor (emu_rate_stream.cpp) used.
// この周期に本当に新しい control_output が発行されたら true を返す（呼び出し側が
// ctrl_ref.csv を生の IMU エッジでなく実際の制御周期で間引けるように）。
// sf::control_output は Latest(1) トピックで、control_task.cpp は ARMED 経路
// でのみ発行する（control_task.cpp:376）-- disarm 中は代わりにゼロの
// sf::log_stream レコードを発行する（control_task.cpp:313）が sf::control_output
// には触れないため、制御ループが止まると .latest() は同じ値を永久に返し続ける。
// その凍結値を後続の全 IMU エッジで読み直して書くのは flight_log.yaml の
// csv_rules が禁じる「直前値埋め」そのものになるため、ctrl.timestamp の変化でも
// エッジ検出する -- 本レコーダの前身（emu_rate_stream.cpp）と同じ規律。
bool write_rate_ref_and_ctrl_output_rows(int64_t seq, const sf::ControlOutput& ctrl)
{
    if (ctrl.timestamp == 0 || ctrl.timestamp == g_last_ctrl_ts) return false;
    g_last_ctrl_ts = ctrl.timestamp;
    const int64_t ts = static_cast<int64_t>(ctrl.timestamp);

    std::FILE* rr = sils_emu_flightlog_stream("rate_ref",
        "timestamp_us,seq,rate_ref_roll,rate_ref_pitch,rate_ref_yaw");
    if (rr != nullptr) {
        std::fprintf(rr, "%lld,%lld,%.7g,%.7g,%.7g\n", (long long)ts, (long long)seq,
            (double)ctrl.rate_ref[0], (double)ctrl.rate_ref[1], (double)ctrl.rate_ref[2]);
    }

    std::FILE* co = sils_emu_flightlog_stream("ctrl_output",
        "timestamp_us,seq,thrust,torque_roll,torque_pitch,torque_yaw");
    if (co != nullptr) {
        std::fprintf(co, "%lld,%lld,%.7g,%.7g,%.7g,%.7g\n", (long long)ts, (long long)seq,
            (double)ctrl.thrust,
            (double)ctrl.torque[0], (double)ctrl.torque[1], (double)ctrl.torque[2]);
    }
    return true;
}

// One row per GENUINELY new actuator_motor publish (edge-detected on its
// timestamp, exactly like control_output above): sf::actuator_motor is a
// Latest(1) topic the mixer only publishes on the armed path, so re-writing
// its frozen last value on every later IMU edge would be the "fill with the
// previous value" pattern the format forbids. 0 = "never published".
// 本当に新しい actuator_motor 発行ごとに1行（control_output と同様にタイム
// スタンプでエッジ検出）: sf::actuator_motor は Latest(1) トピックでミキサーが
// ARMED 経路でのみ発行するため、凍結した最終値を後続の IMU エッジで書き直すと
// 形式が禁じる「直前値埋め」になる。0 =「未発行」。
void write_motor_row(int64_t seq, const sf::MotorOutput& motor)
{
    if (motor.timestamp == 0 || motor.timestamp == g_last_motor_ts) return;
    g_last_motor_ts = motor.timestamp;
    std::FILE* f = sils_emu_flightlog_stream("motor",
        "timestamp_us,seq,duty_FR,duty_RR,duty_RL,duty_FL");
    if (f == nullptr) return;
    std::fprintf(f, "%lld,%lld,%.7g,%.7g,%.7g,%.7g\n",
        (long long)motor.timestamp, (long long)seq,
        (double)motor.duty[0], (double)motor.duty[1], (double)motor.duty[2], (double)motor.duty[3]);
}

// --- ctrl_ref.csv (50Hz, decimated at the call site) ---

void write_ctrl_ref_row(const sf::ControlOutput& ctrl, const sf::MotorOutput& motor)
{
    const int64_t ts = static_cast<int64_t>(ctrl.timestamp);
    std::FILE* f = sils_emu_flightlog_stream("ctrl_ref",
        "timestamp_us,flight_mode,angle_ref_roll,angle_ref_pitch,total_thrust,"
        "duty_FR,duty_RR,duty_RL,duty_FL,alt_setpoint,alt_vel_target,climb_rate_cmd,"
        "pos_setpoint_x,pos_setpoint_y");
    if (f == nullptr) return;

    const sf::SystemMode mode = sf::system_mode.latest();
    // The trailing 5 cells (alt_setpoint/alt_vel_target/climb_rate_cmd/
    // pos_setpoint_x/y) are left EMPTY: no published topic carries the
    // PidController's PRIVATE cascade setpoints (flight_log.yaml's
    // empty-cell rule covers exactly this — "the firmware version does not
    // send that field at all").
    // 末尾5セルは空欄のまま: PidController 内部（非公開）のカスケード目標値を
    // 運ぶトピックが存在しない（flight_log.yaml の空欄規則「そのファーム版が
    // そもそも送らない場合」に該当）。
    std::fprintf(f, "%lld,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,,,,,\n",
        (long long)ts, (int)mode.sub_mode,
        (double)ctrl.angle_ref[0], (double)ctrl.angle_ref[1], (double)ctrl.thrust,
        (double)motor.duty[0], (double)motor.duty[1], (double)motor.duty[2], (double)motor.duty[3]);
}

// --- pilot.csv (edge-detected at its own native rate) ---

void write_pilot_row_if_new()
{
    const sf::CommandSetpoint cmd = sf::command_setpoint.latest();
    if (cmd.timestamp == 0 || cmd.timestamp == g_last_cmd_ts) return;
    g_last_cmd_ts = cmd.timestamp;

    std::FILE* f = sils_emu_flightlog_stream("pilot", "timestamp_us,throttle,roll,pitch,yaw");
    if (f == nullptr) return;
    std::fprintf(f, "%lld,%.7g,%.7g,%.7g,%.7g\n", (long long)cmd.timestamp,
        (double)cmd.throttle, (double)cmd.roll, (double)cmd.pitch, (double)cmd.yaw);
}

// --- status.csv (1Hz, decimated at the call site) ---

void write_status_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("status",
        "timestamp_us,uptime_ms,voltage,current_ma,flight_state,sensor_health,"
        "eskf_status,reset_reason,pid_roll_kp,pid_roll_ti,pid_roll_td,"
        "pid_pitch_kp,pid_pitch_ti,pid_pitch_td,pid_yaw_kp,pid_yaw_ti,pid_yaw_td");
    if (f == nullptr) return;

    const sf::SystemMode    mode   = sf::system_mode.latest();
    const sf::PowerData     pwr    = sf::sensor_power.latest();
    const sf::SensorHealth  health = sf::sensor_health.latest();

    // Nine rate-loop PID gains, read live exactly as PidController::
    // loadParams() does (same param names, params.cpp table) -- timestamped
    // here so an in-flight autotune/gain change is traceable over the flight.
    // 9個のレートループ PID ゲインを PidController::loadParams() と同じ名前で
    // ライブ読み出し -- 時刻付きで残すことで飛行中の自動チューニング/ゲイン
    // 変更を追跡できる。
    float roll_kp = 0.0f, roll_ti = 0.0f, roll_td = 0.0f;
    float pitch_kp = 0.0f, pitch_ti = 0.0f, pitch_td = 0.0f;
    float yaw_kp = 0.0f, yaw_ti = 0.0f, yaw_td = 0.0f;
    sf::params::get_float("rate.roll.kp",  roll_kp);
    sf::params::get_float("rate.roll.ti",  roll_ti);
    sf::params::get_float("rate.roll.td",  roll_td);
    sf::params::get_float("rate.pitch.kp", pitch_kp);
    sf::params::get_float("rate.pitch.ti", pitch_ti);
    sf::params::get_float("rate.pitch.td", pitch_td);
    sf::params::get_float("rate.yaw.kp",   yaw_kp);
    sf::params::get_float("rate.yaw.ti",   yaw_ti);
    sf::params::get_float("rate.yaw.td",   yaw_td);

    std::fprintf(f,
        "%lld,%lld,%.7g,%.7g,%d,%d,%d,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
        (long long)ts, (long long)(ts / 1000),
        (double)pwr.voltage, (double)pwr.current,
        (int)mode.state, (int)health.healthy_mask,
        (int)kEskfStatusRunning, (int)esp_reset_reason(),
        (double)roll_kp, (double)roll_ti, (double)roll_td,
        (double)pitch_kp, (double)pitch_ti, (double)pitch_td,
        (double)yaw_kp, (double)yaw_ti, (double)yaw_td);
}

// --- baro/tof_bottom/flow/mag (each edge-detected on its OWN per-sensor
// timestamp field inside the SensorSnapshot mirror -- the shared
// snapshot.timestamp cannot tell which sensor updated). ---
// baro/tof_bottom/flow/mag（SensorSnapshot ミラー内の各センサ「自身の」
// タイムスタンプ欄で個別にエッジ検出 -- 共有の snapshot.timestamp では
// どのセンサが更新したか分からない）。

void write_sensor_snapshot_rows()
{
    const sf::SensorSnapshot snap = sf::sensor_snapshot.latest();
    if (snap.timestamp == 0) return;   // ImuTask has not published a snapshot yet

    if (snap.baro_timestamp != 0 && snap.baro_timestamp != g_last_baro_ts) {
        g_last_baro_ts = snap.baro_timestamp;
        std::FILE* f = sils_emu_flightlog_stream("baro", "timestamp_us,altitude,pressure");
        if (f != nullptr) {
            std::fprintf(f, "%lld,%.7g,%.7g\n", (long long)snap.baro_timestamp,
                (double)snap.baro_altitude, (double)snap.baro_pressure);
        }
    }
    if (snap.tof_timestamp != 0 && snap.tof_timestamp != g_last_tof_ts) {
        g_last_tof_ts = snap.tof_timestamp;
        std::FILE* f = sils_emu_flightlog_stream("tof_bottom", "timestamp_us,distance,status");
        if (f != nullptr) {
            std::fprintf(f, "%lld,%.7g,%d\n", (long long)snap.tof_timestamp,
                (double)snap.tof_distance, (int)snap.tof_status);
        }
    }
    if (snap.flow_timestamp != 0 && snap.flow_timestamp != g_last_flow_ts) {
        g_last_flow_ts = snap.flow_timestamp;
        std::FILE* f = sils_emu_flightlog_stream("flow", "timestamp_us,dx,dy,quality");
        if (f != nullptr) {
            std::fprintf(f, "%lld,%d,%d,%d\n", (long long)snap.flow_timestamp,
                (int)snap.flow_dx, (int)snap.flow_dy, (int)snap.flow_squal);
        }
    }
    if (snap.mag_timestamp != 0 && snap.mag_timestamp != g_last_mag_ts) {
        g_last_mag_ts = snap.mag_timestamp;
        std::FILE* f = sils_emu_flightlog_stream("mag", "timestamp_us,x,y,z");
        if (f != nullptr) {
            std::fprintf(f, "%lld,%.7g,%.7g,%.7g\n", (long long)snap.mag_timestamp,
                (double)snap.mag[0], (double)snap.mag[1], (double)snap.mag[2]);
        }
    }
}

}  // namespace

extern "C" void sils_emu_flightlog_firmware_sample(int64_t now_us)
{
    (void)now_us;   // rows are timestamped from each topic's OWN timestamp field, not the caller's clock

    // Edge-detect a NEW IMU sample -- the same discipline emu_rate_stream.cpp
    // used for control_output (0 doubles as "no sample yet"; the virtual
    // clock only increases, so 0 never recurs once the estimator is running).
    // 新しい IMU サンプルをエッジ検出 -- emu_rate_stream.cpp が control_output に
    // 使ったのと同じ規律（0=「まだ無し」。仮想時計は単調増加なので推定器が
    // 動き出せば以後 0 は再来しない）。
    const sf::ImuData imu = sf::sensor_imu.latest();
    if (imu.timestamp == 0 || imu.timestamp == g_last_imu_ts) return;
    g_last_imu_ts = imu.timestamp;

    const int64_t ts  = static_cast<int64_t>(imu.timestamp);
    const int64_t seq = g_seq++;

    write_imu_row(ts, seq, imu);
    const sf::StateEstimate est = sf::estimate_state.latest();
    write_attitude_row(ts, seq, est);
    write_posvel_row(ts, seq, est);

    const sf::ControlOutput ctrl  = sf::control_output.latest();
    const sf::MotorOutput   motor = sf::actuator_motor.latest();
    write_motor_row(seq, motor);
    // Decimate ctrl_ref.csv on genuinely fresh control cycles (g_ctrl_ref_edge),
    // NOT on the raw IMU edge count -- otherwise it would keep emitting frozen
    // rows at 50Hz for as long as the IMU/estimator keep running after the
    // control loop itself has stopped (see write_rate_ref_and_ctrl_output_rows).
    // ctrl_ref.csv は生の IMU エッジ数でなく、実際に新しい制御周期が起きた回数
    // （g_ctrl_ref_edge）で間引く -- でなければ制御ループが止まった後も IMU/推定器が
    // 動き続ける限り凍結行を50Hzで出し続けてしまう。
    if (write_rate_ref_and_ctrl_output_rows(seq, ctrl)) {
        if (g_ctrl_ref_edge % kCtrlRefDecimation == 0) write_ctrl_ref_row(ctrl, motor);
        ++g_ctrl_ref_edge;
    }

    write_pilot_row_if_new();
    if (seq % kStatusDecimation == 0) write_status_row(ts);

    write_sensor_snapshot_rows();
}

extern "C" void sils_emu_flightlog_write_gains(void)
{
    // Re-derive the bundle directory from the SAME env var the sf CLI set for
    // sils_emu_flightlog_open() (rather than asking the core for its internal
    // state) -- unset/empty means the recorder is closed, so this is a no-op,
    // matching the default-OFF discipline.
    // sils_emu_flightlog_open() に渡したのと同じ env 変数からバンドルディレクトリを
    // 再導出する（core の内部状態に問い合わせない）-- 未設定/空ならレコーダは
    // 閉じているので no-op（既定 OFF の規律に一致）。
    const char* dir = std::getenv("SILS_EMU_FLIGHTLOG");
    if (dir == nullptr || dir[0] == '\0') return;

    float roll_kp = 0.0f, roll_ti = 0.0f, roll_td = 0.0f;
    float pitch_kp = 0.0f, pitch_ti = 0.0f, pitch_td = 0.0f;
    float yaw_kp = 0.0f, yaw_ti = 0.0f, yaw_td = 0.0f, yaw_limit = 0.0f;
    sf::params::get_float("rate.roll.kp",  roll_kp);
    sf::params::get_float("rate.roll.ti",  roll_ti);
    sf::params::get_float("rate.roll.td",  roll_td);
    sf::params::get_float("rate.pitch.kp", pitch_kp);
    sf::params::get_float("rate.pitch.ti", pitch_ti);
    sf::params::get_float("rate.pitch.td", pitch_td);
    sf::params::get_float("rate.yaw.kp",   yaw_kp);
    sf::params::get_float("rate.yaw.ti",   yaw_ti);
    sf::params::get_float("rate.yaw.td",   yaw_td);
    sf::params::get_float("rate.yaw.max_torque", yaw_limit);

    char path[600];
    std::snprintf(path, sizeof(path), "%s/gains.json", dir);
    std::FILE* gf = std::fopen(path, "w");
    if (gf == nullptr) return;
    // Hand-rolled JSON (no JSON library in the SILS host build -- same
    // convention emu_rate_stream.cpp used). Shape: {"<axis>": {kp,ti,td,limit}}.
    // JSON ライブラリ非依存の手書き出力（emu_rate_stream.cpp と同じ流儀）。
    std::fprintf(gf,
        "{\n"
        "  \"roll\":  {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g},\n"
        "  \"pitch\": {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g},\n"
        "  \"yaw\":   {\"kp\": %.8g, \"ti\": %.8g, \"td\": %.8g, \"limit\": %.8g}\n"
        "}\n",
        (double)roll_kp,  (double)roll_ti,  (double)roll_td,  (double)kRollPitchTorqueLimit,
        (double)pitch_kp, (double)pitch_ti, (double)pitch_td, (double)kRollPitchTorqueLimit,
        (double)yaw_kp,   (double)yaw_ti,   (double)yaw_td,   (double)yaw_limit);
    std::fclose(gf);
}
