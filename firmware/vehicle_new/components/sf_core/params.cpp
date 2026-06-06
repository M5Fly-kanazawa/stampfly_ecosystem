/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file params.cpp
 * @brief Parameter system and topic instance implementation
 *        パラメータシステムおよびトピックインスタンス実装
 *
 * @design detailed_design.md §6 — Parameter system                    [--]
 * @design requirements.md §3 — Parameter management                   [--]
 */

#include "topics.hpp"
#include "params.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cmath>

static const char* TAG = "Params";

namespace sf {

// =============================================================================
// Topic instances (defined here, declared extern in topics.hpp)
// トピックインスタンス（ここで定義、topics.hppでextern宣言）
// =============================================================================

Topic<ImuData,         RingBuffer, 8>  sensor_imu;
Topic<TofData,         Queue, 2>       sensor_tof;
Topic<FlowData,        Queue, 2>       sensor_flow;
Topic<MagData,         Queue, 2>       sensor_mag;
Topic<BaroData,        Queue, 2>       sensor_baro;
Topic<PowerData,       Latest, 1>      sensor_power;
Topic<StateEstimate,   Latest, 1>      estimate_state;
Topic<CommandSetpoint, Latest, 1>      command_setpoint;
Topic<PilotRequest,    Latest, 1>      pilot_request;
Topic<ControlOutput,   Latest, 1>      control_output;
Topic<MotorOutput,     Latest, 1>      actuator_motor;
Topic<SystemMode,      Latest, 1>      system_mode;
Topic<SystemAlert,     Queue, 4>       system_alert;
Topic<SystemStatus,    Latest, 1>      system_status;
Topic<EstimatorCommand,  Queue, 4>     estimator_command;
Topic<ControllerCommand, Queue, 4>     controller_command;
Topic<NotifyCommand,     Queue, 8>     notify_command;
Topic<SensorHealth,      Latest, 1>    sensor_health;
Topic<GuidanceTarget,    Latest, 1>    command_target;
Topic<NavigationPath,    Queue, 4>     nav_path;

void topics_init()
{
    sensor_imu.init();
    sensor_tof.init();
    sensor_flow.init();
    sensor_mag.init();
    sensor_baro.init();
    sensor_power.init();
    estimate_state.init();
    command_setpoint.init();
    pilot_request.init();
    control_output.init();
    actuator_motor.init();
    system_mode.init();
    system_alert.init();
    system_status.init();
    estimator_command.init();
    controller_command.init();
    notify_command.init();
    sensor_health.init();
    command_target.init();
    nav_path.init();
}

// =============================================================================
// Parameter System Implementation
// パラメータシステム実装
//
// @design detailed_design.md §6 — Single-macro definition             [--]
// =============================================================================

// Explicit parameter variable definitions
// 明示的なパラメータ変数定義
namespace param_vars {
    // Rate control
    // Rate gains are PHYSICAL [Nm/(rad/s)] now that the mixer is a B^-1 allocation
    // (actuator.cpp): the rate loop is ω̇ = u/I, so kp = I / τ_resp with the rate
    // time constant τ_resp = 0.05 s. They differ only by the body inertia
    // (Ixx/Iyy/Izz = 9.16/13.3/20.4 e-6), not by the old ad-hoc mixer scaling.
    // SIL-derived/verified (rate_tune). ti large = near-P inner loop (the outer
    // attitude loop carries the integral action, standard cascade design).
    // ミキサーが B^-1 配分になったのでレートゲインは物理 [Nm/(rad/s)]。レートループは
    // ω̇ = u/I なので kp = 慣性/応答時定数（τ_resp=0.05s）。各軸は慣性比だけで異なる。
    float rate_roll_kp    = 1.83e-4f;  // Ixx/τ_resp
    float rate_roll_ti    = 20.0f;     // near-P (outer attitude loop integrates)
    float rate_roll_td    = 0.01f;
    float rate_pitch_kp   = 2.66e-4f;  // Iyy/τ_resp
    float rate_pitch_ti   = 20.0f;
    float rate_pitch_td   = 0.01f;
    float rate_yaw_kp     = 4.08e-4f;  // Izz/τ_resp
    float rate_yaw_ti     = 20.0f;
    float rate_yaw_td     = 0.01f;

    // Estimator selection (RESET_PLAN P2: replaceable estimation). The IMU task's
    // factory reads this: 0 = ESKF (15-state), 1 = complementary filter. The SIL
    // bench swaps estimators via this parameter alone — no code change.
    // 推定器の選択（P2: 差し替え可能）。IMU タスクのファクトリが読む: 0=ESKF, 1=相補。
    int32_t estimator_type = 0;

    // Attitude control
    float att_roll_kp     = 5.0f;
    float att_roll_ti     = 4.0f;
    float att_roll_td     = 0.04f;
    float att_pitch_kp    = 5.0f;
    float att_pitch_ti    = 4.0f;
    float att_pitch_td    = 0.04f;

    // Altitude control — cascade alt → vertical-velocity → thrust [N].
    // SIL-validated (hover_smoke ALT_HOLD: holds within ~2 cm through a yaw spin;
    // see RESET_PLAN P1). Stronger than the legacy vehicle/ values (0.6/7/0.1/2.5)
    // because vehicle_new's loop outputs PHYSICAL thrust [N], not normalized
    // throttle — to re-validate on hardware (ToF altitude) before flight.
    // 高度制御 — カスケード 高度→鉛直速度→推力[N]。SIL検証済み（hover_smoke ALT_HOLD:
    // ヨー旋回中も約2cm以内で保持、RESET_PLAN P1）。旧 vehicle/ 値（0.6/7/0.1/2.5）より
    // 強い。vehicle_new は物理推力[N]を出力するため。実機（ToF高度）で再検証要。
    float alt_alt_kp      = 1.5f;
    float alt_alt_ti      = 8.0f;
    float alt_vel_kp      = 0.3f;
    float alt_vel_ti      = 2.0f;

    // Position control
    float pos_pos_kp      = 1.0f;
    float pos_pos_ti      = 5.0f;
    float pos_vel_kp      = 0.8f;   // tight hold: enabled by the accel-comp estimator fix
    float pos_vel_ti      = 2.0f;   // (the old 0.3 was the limit with the contaminated est)

    // ESKF process noise
    float eskf_gyro_noise   = 0.009655f;
    float eskf_accel_noise  = 0.3f;
    float eskf_gyro_bias    = 0.000013f;
    float eskf_accel_bias   = 0.0001f;

    // ESKF observation noise
    float eskf_tof_noise      = 0.03f;
    float eskf_flow_noise     = 0.30f;
    float eskf_baro_noise     = 0.1f;
    float eskf_mag_noise      = 1.0f;
    float eskf_accel_att      = 0.06f;

    // ESKF sensor enable
    bool eskf_use_tof   = true;
    bool eskf_use_flow  = true;
    bool eskf_use_baro  = false;
    bool eskf_use_mag   = false;

    // ESKF gates
    float eskf_mahalanobis  = 15.0f;
    float eskf_tof_innov    = 0.5f;
    float eskf_baro_innov   = 0.5f;
    float eskf_flow_clamp   = 0.3f;

    // ESKF accel-attitude (proven firmware/vehicle values). Registered here as the
    // single source of truth so they are NOT silently taken from the struct defaults.
    // 加速度-姿勢（実証済み firmware/vehicle 値）。SSOT として登録し struct 既定値に
    // 暗黙依存しないようにする。
    float eskf_att_k_adaptive = 10.0f;   // adaptive R: R *= (1 + k|a-g|²)
    float eskf_att_chi2_gate  = 7.81f;   // χ²(3, 0.95) accel-attitude outlier gate
    float eskf_att_corr_clamp = 0.05f;   // [rad] per-update roll/pitch correction clamp

    // ESKF acceleration-compensated accel-attitude (POS_HOLD). The accelerometer measures
    // specific force f = a_kin − g; during a horizontal maneuver the kinematic term a_kin
    // is mistaken for a tilt and the attitude sticks at the "apparent gravity" angle
    // atan(a/g), so POS_HOLD flies away. An α-β tracker on the flow velocity estimates
    // a_kin (state = velocity + acceleration; β small so the SUSTAINED drift acceleration
    // is captured, not washed out like a naive derivative), and the accel-attitude update
    // subtracts R^T·a_kin → the residual is the TRUE attitude error.
    // ESKF 運動加速度補償の accel-attitude（POS_HOLD）。加速度計は比力 f=a_kin−g を測り、水平
    // マニューバ中は運動加速度 a_kin を傾きと誤認し姿勢が「見かけの重力」角 atan(a/g) に張付き
    // POS_HOLD が飛び去る。フロー速度の α-β トラッカで a_kin を推定（状態=速度+加速度、β 小で
    // 持続ドリフト加速度を単純微分のように washout せず捕捉）、accel-attitude が R^T·a_kin を
    // 差し引き残差を真の姿勢誤差にする。
    bool  eskf_accel_comp_enable = true;  // on (adopted; SIL clean+N0 all 4 axes hold)
    float eskf_accel_comp_alpha  = 0.2f;  // α-β velocity gain
    float eskf_accel_comp_beta   = 0.02f; // α-β acceleration gain (small = capture DC drift)
    float eskf_accel_comp_max    = 5.0f;  // [m/s²] physical clamp on a_kin

    // Safety
    float safety_accel_g     = 3.0f;
    float safety_gyro_dps    = 800.0f;
    float safety_comm_timeout = 500.0f;
    float safety_low_v       = 3.4f;
    float safety_usb_v       = 3.3f;

    // Calibration — boot gyro/accel bias calibration on/off (ImuTask seeds the
    // estimator at rest before flight). Default on.
    // キャリブレーション — 起動時バイアス校正の ON/OFF（ImuTask が飛行前に静止で推定器へ
    // 種付け）。既定 ON。
    bool calibration_enable = true;
}

namespace params {

using namespace param_vars;

/// Parameter table — generated from params.def values
/// パラメータテーブル — params.defの値から生成
static const ParamEntry table[] = {
    // Rate control — PHYSICAL gains [Nm/(rad/s)] for the B^-1 mixer (actuator.cpp).
    // kp = I/τ_resp (τ_resp=0.05s); ti large = near-P inner loop. See the variable
    // declarations above for the rationale. Max 0.01 = ~25× headroom over kp.
    // レート制御 — B^-1 ミキサー用の物理ゲイン [Nm/(rad/s)]。kp = 慣性/τ_resp。
    {"rate.roll.kp",    ParamType::FLOAT, &rate_roll_kp,   1.83e-4f,  0.0f,  0.01f,  nullptr},
    {"rate.roll.ti",    ParamType::FLOAT, &rate_roll_ti,   20.0f,     0.01f, 100.0f, nullptr},
    {"rate.roll.td",    ParamType::FLOAT, &rate_roll_td,   0.01f,     0.0f,  1.0f,   nullptr},
    {"rate.pitch.kp",   ParamType::FLOAT, &rate_pitch_kp,  2.66e-4f,  0.0f,  0.01f,  nullptr},
    {"rate.pitch.ti",   ParamType::FLOAT, &rate_pitch_ti,  20.0f,     0.01f, 100.0f, nullptr},
    {"rate.pitch.td",   ParamType::FLOAT, &rate_pitch_td,  0.01f,     0.0f,  1.0f,   nullptr},
    {"rate.yaw.kp",     ParamType::FLOAT, &rate_yaw_kp,    4.08e-4f,  0.0f,  0.01f,  nullptr},
    {"rate.yaw.ti",     ParamType::FLOAT, &rate_yaw_ti,    20.0f,     0.01f, 100.0f, nullptr},
    {"rate.yaw.td",     ParamType::FLOAT, &rate_yaw_td,    0.01f,     0.0f,  1.0f,   nullptr},

    // Estimator selection (0 = ESKF, 1 = complementary) — RESET_PLAN P2.
    {"estimator.type",  ParamType::INT,   &estimator_type, 0.0f,      0.0f,  1.0f,   nullptr},

    // Attitude control
    {"attitude.roll.kp",  ParamType::FLOAT, &att_roll_kp,  5.0f,  0.0f,  50.0f,  nullptr},
    {"attitude.roll.ti",  ParamType::FLOAT, &att_roll_ti,  4.0f,  0.01f, 100.0f, nullptr},
    {"attitude.roll.td",  ParamType::FLOAT, &att_roll_td,  0.04f, 0.0f,  1.0f,   nullptr},
    {"attitude.pitch.kp", ParamType::FLOAT, &att_pitch_kp, 5.0f,  0.0f,  50.0f,  nullptr},
    {"attitude.pitch.ti", ParamType::FLOAT, &att_pitch_ti, 4.0f,  0.01f, 100.0f, nullptr},
    {"attitude.pitch.td", ParamType::FLOAT, &att_pitch_td, 0.04f, 0.0f,  1.0f,   nullptr},

    // Altitude control (SIL-validated; see the variable defaults above)
    {"altitude.alt.kp",   ParamType::FLOAT, &alt_alt_kp,  1.5f,  0.0f, 10.0f,  nullptr},
    {"altitude.alt.ti",   ParamType::FLOAT, &alt_alt_ti,  8.0f,  0.1f, 100.0f, nullptr},
    {"altitude.vel.kp",   ParamType::FLOAT, &alt_vel_kp,  0.3f,  0.0f, 10.0f,  nullptr},
    {"altitude.vel.ti",   ParamType::FLOAT, &alt_vel_ti,  2.0f,  0.1f, 100.0f, nullptr},

    // Position control
    {"position.pos.kp",   ParamType::FLOAT, &pos_pos_kp,  1.0f,  0.0f, 10.0f,  nullptr},
    {"position.pos.ti",   ParamType::FLOAT, &pos_pos_ti,  5.0f,  0.1f, 100.0f, nullptr},
    {"position.vel.kp",   ParamType::FLOAT, &pos_vel_kp,  0.8f,  0.0f, 10.0f,  nullptr},
    {"position.vel.ti",   ParamType::FLOAT, &pos_vel_ti,  2.0f,  0.1f, 100.0f, nullptr},

    // ESKF process noise
    {"eskf.process.gyro_noise",  ParamType::FLOAT, &eskf_gyro_noise,  0.009655f, 0.001f, 1.0f,  nullptr},
    {"eskf.process.accel_noise", ParamType::FLOAT, &eskf_accel_noise, 0.3f,      0.01f,  10.0f, nullptr},
    {"eskf.process.gyro_bias",   ParamType::FLOAT, &eskf_gyro_bias,   0.000013f, 1e-7f,  0.01f, nullptr},
    {"eskf.process.accel_bias",  ParamType::FLOAT, &eskf_accel_bias,  0.0001f,   1e-7f,  0.01f, nullptr},

    // ESKF observation noise
    {"eskf.obs.tof_noise",       ParamType::FLOAT, &eskf_tof_noise,     0.03f, 0.001f, 1.0f,  nullptr},
    {"eskf.obs.flow_noise",      ParamType::FLOAT, &eskf_flow_noise,    0.30f, 0.01f,  5.0f,  nullptr},
    {"eskf.obs.baro_noise",      ParamType::FLOAT, &eskf_baro_noise,    0.1f,  0.01f,  5.0f,  nullptr},
    {"eskf.obs.mag_noise",       ParamType::FLOAT, &eskf_mag_noise,     1.0f,  0.01f,  10.0f, nullptr},
    {"eskf.obs.accel_att_noise", ParamType::FLOAT, &eskf_accel_att,     0.06f, 0.001f, 1.0f,  nullptr},

    // ESKF sensor enable
    {"eskf.use_tof",  ParamType::BOOL, &eskf_use_tof,  1.0f, 0.0f, 1.0f, nullptr},
    {"eskf.use_flow", ParamType::BOOL, &eskf_use_flow, 1.0f, 0.0f, 1.0f, nullptr},
    {"eskf.use_baro", ParamType::BOOL, &eskf_use_baro, 0.0f, 0.0f, 1.0f, nullptr},
    {"eskf.use_mag",  ParamType::BOOL, &eskf_use_mag,  0.0f, 0.0f, 1.0f, nullptr},

    // ESKF gates
    {"eskf.gate.mahalanobis", ParamType::FLOAT, &eskf_mahalanobis, 15.0f, 1.0f,  100.0f, nullptr},
    {"eskf.gate.tof_innov",   ParamType::FLOAT, &eskf_tof_innov,   0.5f,  0.01f, 5.0f,   nullptr},
    {"eskf.gate.baro_innov",  ParamType::FLOAT, &eskf_baro_innov,  0.5f,  0.01f, 5.0f,   nullptr},
    {"eskf.gate.flow_clamp",  ParamType::FLOAT, &eskf_flow_clamp,  0.3f,  0.01f, 5.0f,   nullptr},

    // ESKF accel-attitude (SSOT — proven firmware/vehicle values)
    {"eskf.att.k_adaptive",   ParamType::FLOAT, &eskf_att_k_adaptive, 10.0f, 0.0f,  100.0f, nullptr},
    {"eskf.att.chi2_gate",    ParamType::FLOAT, &eskf_att_chi2_gate,  7.81f, 0.0f,  100.0f, nullptr},
    {"eskf.att.corr_clamp",   ParamType::FLOAT, &eskf_att_corr_clamp, 0.05f, 0.001f, 1.0f,  nullptr},

    // ESKF acceleration-compensated accel-attitude (POS_HOLD; α-β flow-acceleration tracker)
    {"eskf.accel_comp.enable", ParamType::BOOL,  &eskf_accel_comp_enable, 1.0f,  0.0f,  1.0f,  nullptr},
    {"eskf.accel_comp.alpha",  ParamType::FLOAT, &eskf_accel_comp_alpha,  0.2f,  0.01f, 1.0f,  nullptr},
    {"eskf.accel_comp.beta",   ParamType::FLOAT, &eskf_accel_comp_beta,   0.02f, 0.0f,  1.0f,  nullptr},
    {"eskf.accel_comp.max",    ParamType::FLOAT, &eskf_accel_comp_max,    5.0f,  0.5f,  20.0f, nullptr},

    // Safety
    {"safety.impact.accel_g",  ParamType::FLOAT, &safety_accel_g,     3.0f,   1.0f,   10.0f,   nullptr},
    {"safety.impact.gyro_dps", ParamType::FLOAT, &safety_gyro_dps,    800.0f, 100.0f, 2000.0f, nullptr},
    {"safety.comm.timeout_ms", ParamType::FLOAT, &safety_comm_timeout, 500.0f, 100.0f, 5000.0f, nullptr},
    {"safety.battery.low_v",   ParamType::FLOAT, &safety_low_v,       3.4f,   3.0f,   4.2f,    nullptr},
    {"safety.battery.usb_v",   ParamType::FLOAT, &safety_usb_v,       3.3f,   2.5f,   3.5f,    nullptr},

    // Calibration — boot gyro/accel bias calibration on/off
    {"calibration.enable",     ParamType::BOOL,  &calibration_enable, 1.0f,   0.0f,   1.0f,    nullptr},
};

static constexpr int TABLE_SIZE = sizeof(table) / sizeof(table[0]);
static const char* NVS_NAMESPACE = "sf_params";

// =============================================================================
// Find parameter by name
// 名前でパラメータを検索する
// =============================================================================

static const ParamEntry* find(const char* name)
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (strcmp(table[i].name, name) == 0) {
            return &table[i];
        }
    }
    return nullptr;
}

// =============================================================================
// Public API Implementation
// 公開API実装
// =============================================================================

void init()
{
    load();
    ESP_LOGI(TAG, "Parameter system initialized (%d params)", TABLE_SIZE);
}

bool get_float(const char* name, float& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) return false;
    out = *static_cast<float*>(e->value_ptr);
    return true;
}

bool get_bool(const char* name, bool& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;
    out = *static_cast<bool*>(e->value_ptr);
    return true;
}

bool get_int(const char* name, int32_t& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;
    out = *static_cast<int32_t*>(e->value_ptr);
    return true;
}

bool set_float(const char* name, float value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) {
        ESP_LOGW(TAG, "set_float: '%s' not found", name);
        return false;
    }

    // Validate range
    // 範囲を検証
    if (value < e->min_val || value > e->max_val) {
        ESP_LOGW(TAG, "set_float: '%s' = %f out of range [%f, %f]",
                 name, value, e->min_val, e->max_val);
        return false;
    }

    *static_cast<float*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %f", name, value);

    // Call callback if registered
    // コールバックが登録されていれば呼ぶ
    if (e->callback) {
        e->callback();
    }

    return true;
}

bool set_bool(const char* name, bool value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;

    *static_cast<bool*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %s", name, value ? "true" : "false");

    if (e->callback) {
        e->callback();
    }
    return true;
}

bool set_int(const char* name, int32_t value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;

    if (value < static_cast<int32_t>(e->min_val) ||
        value > static_cast<int32_t>(e->max_val)) {
        ESP_LOGW(TAG, "set_int: '%s' = %ld out of range", name, (long)value);
        return false;
    }

    *static_cast<int32_t*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %ld", name, (long)value);

    if (e->callback) {
        e->callback();
    }
    return true;
}

void save()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    int saved = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            float val = *static_cast<float*>(e.value_ptr);
            // Store float as uint32_t blob
            // floatをuint32_tとして保存
            uint32_t raw;
            memcpy(&raw, &val, sizeof(float));
            nvs_set_u32(handle, e.name, raw);
            saved++;
        } else if (e.type == ParamType::BOOL) {
            bool val = *static_cast<bool*>(e.value_ptr);
            nvs_set_u8(handle, e.name, val ? 1 : 0);
            saved++;
        }
    }

    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Saved %d parameters to NVS", saved);
}

void load()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // NVS not initialized or no data — use defaults
        // NVS未初期化またはデータなし — デフォルトを使用
        ESP_LOGI(TAG, "No saved parameters, using defaults");
        return;
    }

    int loaded = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            uint32_t raw;
            if (nvs_get_u32(handle, e.name, &raw) == ESP_OK) {
                float val;
                memcpy(&val, &raw, sizeof(float));
                // Validate range before applying
                // 適用前に範囲を検証
                if (val >= e.min_val && val <= e.max_val && !std::isnan(val)) {
                    *static_cast<float*>(e.value_ptr) = val;
                    loaded++;
                }
            }
        } else if (e.type == ParamType::BOOL) {
            uint8_t raw;
            if (nvs_get_u8(handle, e.name, &raw) == ESP_OK) {
                *static_cast<bool*>(e.value_ptr) = (raw != 0);
                loaded++;
            }
        }
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded %d parameters from NVS", loaded);
}

void reset_all()
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            *static_cast<float*>(e.value_ptr) = e.default_val;
        } else if (e.type == ParamType::BOOL) {
            *static_cast<bool*>(e.value_ptr) = (e.default_val != 0.0f);
        } else if (e.type == ParamType::INT) {
            *static_cast<int32_t*>(e.value_ptr) = static_cast<int32_t>(e.default_val);
        }
    }
    ESP_LOGI(TAG, "All %d parameters reset to defaults", TABLE_SIZE);
}

void list()
{
    ESP_LOGI(TAG, "=== Parameters (%d) ===", TABLE_SIZE);
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            ESP_LOGI(TAG, "  %-30s = %f  [%f, %f]",
                     e.name, *static_cast<float*>(e.value_ptr),
                     e.min_val, e.max_val);
        } else if (e.type == ParamType::BOOL) {
            ESP_LOGI(TAG, "  %-30s = %s",
                     e.name, *static_cast<bool*>(e.value_ptr) ? "true" : "false");
        }
    }
}

int count()
{
    return TABLE_SIZE;
}

const ParamEntry* entry(int index)
{
    if (index < 0 || index >= TABLE_SIZE) return nullptr;
    return &table[index];
}

}  // namespace params
}  // namespace sf
