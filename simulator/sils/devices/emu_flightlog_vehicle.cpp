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
 * same sf_core topics via sf_cores). (The frozen legacy firmware/vehicle_old,
 * which had none of these sf:: topics and so did not link this file, was
 * removed 2026-09-13 — see tag archive/2026-09-13.)
 * emu_vehicle と emu_workshop（どちらも firmware/vehicle の ImuTask/推定器/
 * comm/telemetry トピックを実行 -- emu_workshop は ControlTask のみ差し替え、
 * CMakeLists.txt 参照）、および hover_smoke（sf_cores 経由で同じ sf_core
 * トピックをリンク）にリンクする。（これらの sf:: トピックを持たずリンク
 * 対象外だった凍結レガシーファーム firmware/vehicle_old は2026-09-13に
 * 削除済み — タグ archive/2026-09-13 参照。）
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
#include "barometric_height_estimator.hpp"   // kBaroHeightFlag*, kBaroAge* — the baro_height flags and age units
#include "data_types.hpp"   // sf::ImuData, sf::StateEstimate, sf::ControlOutput, ...
#include "params.hpp"       // sf::params::get_float — SSOT rate-loop gains
#include "flight_state.hpp" // sf::isArmed — controller_status is only live while armed
#include "failsafe_status_guard.hpp"   // sf::isFailsafeStatusStaleForDisplay — same test as the Data Stream's status_stale bit
#include "log_sink.hpp"      // the WiFi exit of the log sink -> text_log.csv (the recorder is the capture here)

namespace {

constexpr int kCtrlRefDecimation = 8;    // 400Hz / 8 = 50Hz, matches CtrlRef's real rate
constexpr int kFlightFlagsDecimation = 8;   // 400Hz / 8 = 50Hz, matches FlightFlags' real rate
constexpr int kToFSilenceDecimation  = 8;   // 400Hz / 8 = 50Hz, matches the ToFSilence (0x4E) entry's real rate
constexpr double kBaroAgeNeverSeconds = -1.0;   // baro_height.offset_age of an offset never learned / オフセット未学習のときの baro_height.offset_age
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
uint32_t g_last_baro_ts  = 0;   // baro.csv and baro_height.csv edge detect
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

// --- flight_phase.csv (400Hz, keyed on the IMU edge) / flight_flags.csv (50Hz) ---
// Mirror the firmware Data Stream's FlightPhase400 (0x4C) / FlightFlags (0x4D)
// entries. controller_status is only republished by ControlTask while armed
// (the firmware's disarmed log record is zero-filled), so the flip fields read
// 0 on the ground for flight_phase; flight_flags keeps the last published
// value, exactly like the real Data Stream.
// ファームの Data Stream の FlightPhase400（0x4C）/ FlightFlags（0x4D）エントリを
// 写す。controller_status は armed 中のみ ControlTask が再発行する（disarm 中の
// ログレコードはゼロ埋め）ため、flight_phase の flip 系は地上で 0。flight_flags は
// 実 Data Stream と同じく最後に発行された値を保持する。

void write_flight_phase_row(int64_t ts, int64_t seq)
{
    std::FILE* f = sils_emu_flightlog_stream("flight_phase",
        "timestamp_us,seq,flight_state,flip_phase,flip_result,flip_phi");
    if (f == nullptr) return;
    const sf::SystemMode mode = sf::system_mode.latest();
    const bool armed = sf::isArmed(static_cast<sf::FlightState>(mode.state));
    const sf::ControllerStatus status = sf::controller_status.latest();
    std::fprintf(f, "%lld,%lld,%d,%d,%d,%.7g\n", (long long)ts, (long long)seq,
        (int)mode.state,
        armed ? (int)status.flip_phase : 0,
        armed ? (int)status.flip_result : 0,
        armed ? (double)status.flip_phi_rad : 0.0);
}

void write_flight_flags_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("flight_flags",
        "timestamp_us,pilot_arm,pilot_flip,flip_ready,flip_block_reason,arm_block,"
        "attitude_mismatch,attitude_verified");
    if (f == nullptr) return;
    const sf::SystemMode       mode   = sf::system_mode.latest();
    const sf::SystemStatus     sys    = sf::system_status.latest();
    const sf::PilotRequest     pilot  = sf::pilot_request.latest();
    const sf::ControllerStatus status = sf::controller_status.latest();
    std::fprintf(f, "%lld,%d,%d,%d,%d,%d,%d,%d\n", (long long)ts,
        (int)pilot.arm, (int)pilot.flip_button, (int)status.flip_ready,
        (int)status.flip_block_reason, (int)mode.arm_block,
        (int)sys.attitude_mismatch, (int)sys.attitude_verified);
}

// --- tof_silence.csv (50Hz) ---
// Mirror the firmware Data Stream's ToFSilence (0x4E) entry (DataStream::appendToFSilenceEntry): the
// accelerometer-based vertical velocity estimate, the estimated rise and the thrust ratio of the descent
// without a ToF (controller_status, last value published while armed), the failsafe verdict and its
// reason, the ToF health facts, and the refused-mode reason.
// ファームの Data Stream の ToFSilence（0x4E）エントリ（DataStream::appendToFSilenceEntry）を写す:
// 加速度計に基づく鉛直速度の推定・上昇量の推定・ToF なし降下の推力比（controller_status。armed 中に最後に
// 発行された値）、フェイルセーフの判定とその理由、ToF の健全状態、拒否されたモードの理由。

void write_tof_silence_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("tof_silence",
        "timestamp_us,vertical_velocity_up,vertical_rise_m,descent_thrust_ratio,tof_silent,tof_reason,"
        "tof_altitude_reliable,tof_invalid_stuck,tof_status_stale,mode_block");
    if (f == nullptr) return;
    const sf::FailsafeStatus   failsafe = sf::failsafe_status.latest();
    const sf::ToFHealth        health   = sf::tof_health.latest();
    const sf::ControllerStatus control  = sf::controller_status.latest();
    const sf::SystemMode       mode     = sf::system_mode.latest();
    const bool stale = sf::isFailsafeStatusStaleForDisplay(static_cast<uint32_t>(ts), failsafe.timestamp);
    std::fprintf(f, "%lld,%.7g,%.7g,%.7g,%d,%d,%d,%d,%d,%d\n", (long long)ts,
        (double)control.vertical_velocity_up, (double)control.vertical_rise_m, (double)control.blind_thrust_ratio,
        (int)failsafe.tof_silent, (int)failsafe.tof_reason,
        (int)health.altitude_reliable, (int)health.invalid_stuck, (int)stale, (int)mode.mode_block);
}

// --- ctrl_ref.csv (50Hz, decimated at the call site) ---

void write_ctrl_ref_row(const sf::ControlOutput& ctrl, const sf::MotorOutput& motor)
{
    const int64_t ts = static_cast<int64_t>(ctrl.timestamp);
    std::FILE* f = sils_emu_flightlog_stream("ctrl_ref",
        "timestamp_us,flight_mode,angle_ref_roll,angle_ref_pitch,total_thrust,"
        "duty_FR,duty_RR,duty_RL,duty_FL,alt_setpoint,alt_vel_target,climb_rate_cmd,"
        "pos_setpoint_x,pos_setpoint_y,thrust_excitation_state,thrust_excitation_amplitude");
    if (f == nullptr) return;

    const sf::SystemMode mode = sf::system_mode.latest();
    // The altitude and horizontal targets and the thrust-excitation state come from controller_status, as on the real
    // vehicle (DataStream::appendCtrlRefEntry): the same values, the same packing of the state byte (state in bits 0..2, the
    // waveform in bits 3..4) and the same rule that a disarmed craft writes 0.
    // 高さと水平の目標と推力励振の状態は、実機（DataStream::appendCtrlRefEntry）と同じく controller_status から取る:
    // 同じ値、同じ状態バイトの詰め方（状態は下位 3 ビット、波形はビット 3〜4）、disarm 中は 0 を書く同じ規則。
    const sf::ControllerStatus status = sf::controller_status.latest();
    const bool armed = mode.armed;
    const unsigned excitation_byte = armed
        ? ((status.thrust_excitation_state & 0x07u) | (static_cast<unsigned>(status.thrust_excitation_waveform) << 3))
        : 0u;
    const unsigned amplitude = armed ? static_cast<unsigned>(status.thrust_excitation_amplitude_permille) : 0u;
    std::fprintf(f, "%lld,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%u,%u\n",
        (long long)ts, (int)mode.sub_mode,
        (double)ctrl.angle_ref[0], (double)ctrl.angle_ref[1], (double)ctrl.thrust,
        (double)motor.duty[0], (double)motor.duty[1], (double)motor.duty[2], (double)motor.duty[3],
        (double)(armed ? status.reference_altitude_m : 0.0f),
        (double)(armed ? status.reference_climb_velocity_mps : 0.0f),
        (double)(armed ? status.reference_climb_rate_command_mps : 0.0f),
        (double)(armed ? status.reference_position_ned[0] : 0.0f),
        (double)(armed ? status.reference_position_ned[1] : 0.0f),
        excitation_byte, amplitude);
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

// baro_height.csv rides on the baro edge, as the firmware Data Stream writes the 0x52 entry next to the 0x45 entry
// (DataStream::appendBaroHeightEntry). The age is converted from the wire's 0.1 s units; the "never learned"
// value of the wire becomes -1, as udp_capture.py does.
// baro_height.csv は baro のエッジで書く（ファームの Data Stream が 0x52 を 0x45 の隣に書くのと同じ。
// DataStream::appendBaroHeightEntry）。経過時間は電文の 0.1 s 単位から換算し、電文の「未学習」の値は
// udp_capture.py と同じく -1 にする。
void write_baro_height_row(const sf::SensorSnapshot& snap)
{
    std::FILE* f = sils_emu_flightlog_stream("baro_height", "timestamp_us,height,offset,valid,learned,offset_age");
    if (f == nullptr) return;
    const bool never_learned = static_cast<float>(snap.baro_offset_age_decis) == sf::kBaroAgeNeverDecis;
    const double age_s = never_learned ? kBaroAgeNeverSeconds
                                       : static_cast<double>(snap.baro_offset_age_decis) / sf::kBaroAgeDecisPerSecond;
    std::fprintf(f, "%lld,%.7g,%.7g,%d,%d,%.7g\n", (long long)snap.baro_timestamp,
        (double)snap.baro_height, (double)snap.baro_offset,
        (snap.baro_height_flags & sf::kBaroHeightFlagValid) ? 1 : 0,
        (snap.baro_height_flags & sf::kBaroHeightFlagLearned) ? 1 : 0, age_s);
}

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
        write_baro_height_row(snap);
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

// --- tof_diagnostics / loop_timing / nonfinite / link_health (1Hz, the status packet v4's streams) ---
// The same facts the vehicle's Data Stream sends in the status packet v4 (data_stream.cpp
// buildStatusExtension()), read from the same topics. In the host simulation: the ToF signal / ambient /
// sigma values come from the chip model's synthesized histogram (see TofData in data_types.hpp), the front
// ToF is not fitted (all zero, like an absent sensor on the vehicle), and send_drops / entry_drops /
// log_stream_overflow are 0 because this recorder, not a socket, is the consumer.
// 車両の Data Stream がステータスパケット v4 で送るのと同じ事実を、同じトピックから読む（data_stream.cpp の
// buildStatusExtension()）。ホストのシミュレーションでは、ToF の signal / ambient / sigma はチップモデルが合成した
// ヒストグラム由来（data_types.hpp の TofData 参照）、前方 ToF は搭載なし（機体で無いセンサと同じくすべて 0）、
// send_drops / entry_drops / log_stream_overflow は消費者がソケットではなくこの記録器なので 0。

void write_tof_diagnostics_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("tof_diagnostics",
        "timestamp_us,bottom_range_status,bottom_signal_rate,bottom_ambient_rate,bottom_sigma_mm,"
        "front_range_status,front_signal_rate,front_ambient_rate,front_sigma_mm");
    if (f == nullptr) return;
    const sf::SensorSnapshot snap = sf::sensor_snapshot.latest();
    const bool front_fitted = snap.tof_front_timestamp != 0;
    std::fprintf(f, "%lld,%d,%.7g,%.7g,%.7g,%d,%.7g,%.7g,%.7g\n", (long long)ts,
        (int)snap.tof_status, (double)snap.tof_signal_rate, (double)snap.tof_ambient_rate, (double)snap.tof_sigma_mm,
        front_fitted ? (int)snap.tof_front_status : 0,
        front_fitted ? (double)snap.tof_front_signal_rate : 0.0,
        front_fitted ? (double)snap.tof_front_ambient_rate : 0.0,
        front_fitted ? (double)snap.tof_front_sigma_mm : 0.0);
}

// The eleven sf::LoopTiming counters of one task, in the column order of flight_log.yaml's loop_timing stream.
// 1 タスクの sf::LoopTiming の 11 個の数え上げ。flight_log.yaml の loop_timing ストリームの列順。
void write_loop_timing_columns(std::FILE* f, const sf::LoopTiming& t)
{
    std::fprintf(f, ",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        (unsigned)t.cycles, (unsigned)t.period_max_us, (unsigned)t.late_count, (unsigned)t.busy_max_us,
        (unsigned)t.over_budget_count, (unsigned)t.last_late_us, (unsigned)t.armed_cycles,
        (unsigned)t.armed_period_max_us, (unsigned)t.armed_late_count, (unsigned)t.armed_busy_max_us,
        (unsigned)t.armed_over_budget_count);
}

#define SILS_LOOP_TIMING_HEADER(task) \
    task "_cycles," task "_period_max_us," task "_late_count," task "_busy_max_us," \
    task "_over_budget_count," task "_last_late_us," task "_armed_cycles," task "_armed_period_max_us," \
    task "_armed_late_count," task "_armed_busy_max_us," task "_armed_over_budget_count"

void write_loop_timing_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("loop_timing",
        "timestamp_us," SILS_LOOP_TIMING_HEADER("imu") "," SILS_LOOP_TIMING_HEADER("control"));
    if (f == nullptr) return;
    std::fprintf(f, "%lld", (long long)ts);
    write_loop_timing_columns(f, sf::imu_loop_timing.latest());
    write_loop_timing_columns(f, sf::control_loop_timing.latest());
    std::fprintf(f, "\n");
}

void write_nonfinite_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("nonfinite",
        "timestamp_us,checked_cycles,detected_cycles,estimate_input,setpoint_input,control_reference,"
        "control_output,mixer_output,motor_command,first_us,first_stage");
    if (f == nullptr) return;
    const sf::NonFiniteStatus n = sf::nonfinite_status.latest();
    std::fprintf(f, "%lld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\n", (long long)ts, (unsigned)n.checked_cycles,
        (unsigned)n.detected_cycles, (unsigned)n.stage_count[0], (unsigned)n.stage_count[1],
        (unsigned)n.stage_count[2], (unsigned)n.stage_count[3], (unsigned)n.stage_count[4],
        (unsigned)n.stage_count[5], (unsigned)n.first_us, (int)n.first_stage);
}

void write_link_health_row(int64_t ts)
{
    std::FILE* f = sils_emu_flightlog_stream("link_health",
        "timestamp_us,log_stream_overflow,send_drops,entry_drops,text_dropped_lines");
    if (f == nullptr) return;
    std::fprintf(f, "%lld,0,0,0,%u\n", (long long)ts, (unsigned)sf::log_sink::wifiDroppedTotal());
}

void write_diagnostics_rows(int64_t ts)
{
    write_tof_diagnostics_row(ts);
    write_loop_timing_row(ts);
    write_nonfinite_row(ts);
    write_link_health_row(ts);
}

// --- text_log.csv (event-driven): the ESP_LOG lines, taken from the WiFi exit of the log sink ---
// The recorder plays the part of the capture client: it switches the exit on at the first call and drains it
// at every step, exactly as the Data Stream does on the vehicle (but without a byte budget: a file has no
// bandwidth limit). Every line of the log sink therefore appears in the bundle, in order, with its time.
// ESP_LOG の行を、ログシンクの WiFi 側の出口から取る。記録器が取得クライアントの役を務める: 最初の呼び出しで出口を
// 有効にし、毎ステップ排出する。機体の Data Stream と同じ動き（ただしバイト数の上限はない: ファイルに帯域の
// 上限はない）。ログシンクの行はすべて、順序どおりに時刻つきで一式に載る。

void write_text_log_row(const sf::log_sink::WifiLineInfo& info, const char* text, size_t length)
{
    std::FILE* f = sils_emu_flightlog_stream("text_log", "timestamp_us,truncated,dropped_before,text");
    if (f == nullptr) return;
    if (length > 0 && text[length - 1] == '\n') --length;   // one cell = one line, without its newline / 1 セル = 1 行（改行なし）
    std::fprintf(f, "%u,%d,%u,\"", (unsigned)info.timestamp_us, info.truncated ? 1 : 0, (unsigned)info.dropped_before);
    for (size_t i = 0; i < length; ++i) {
        if (text[i] == '"') std::fputc('"', f);   // CSV: a quote is doubled / CSV: 引用符は重ねる
        std::fputc(text[i], f);
    }
    std::fputs("\"\n", f);
}

void drain_text_log()
{
    static bool started = false;
    if (!started) {
        started = true;
        sf::log_sink::setWifiCapture(true);
    }
    char text[sf::log_sink::kMaxLineBytes];
    sf::log_sink::WifiLineInfo info;
    while (true) {
        const size_t length = sf::log_sink::takeWifiLine(text, sizeof(text), &info, sf::log_sink::kMaxLineBytes);
        if (length == 0) break;
        write_text_log_row(info, text, length);
    }
}

}  // namespace

extern "C" void sils_emu_flightlog_firmware_sample(int64_t now_us)
{
    (void)now_us;   // rows are timestamped from each topic's OWN timestamp field, not the caller's clock

    // The text log is event-driven, so it is drained at every step, before the IMU-edge early return below.
    // テキストログはイベント駆動なので、下の IMU エッジの早期 return より前に、毎ステップ排出する。
    drain_text_log();

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
    write_flight_phase_row(ts, seq);
    if (seq % kFlightFlagsDecimation == 0) write_flight_flags_row(ts);
    if (seq % kToFSilenceDecimation == 0) write_tof_silence_row(ts);

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
    if (seq % kStatusDecimation == 0) {
        write_status_row(ts);
        write_diagnostics_rows(ts);   // the status packet v4's four streams, with the same 1Hz rhythm / ステータスパケット v4 の 4 ストリーム。同じ 1Hz の拍で
    }

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
