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
Topic<ControlOutput,   Latest, 1>      control_output;
Topic<MotorOutput,     Latest, 1>      actuator_motor;
Topic<SystemMode,      Latest, 1>      system_mode;
Topic<SystemAlert,     Queue, 4>       system_alert;

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
    control_output.init();
    actuator_motor.init();
    system_mode.init();
    system_alert.init();
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

    // Attitude control
    float att_roll_kp     = 5.0f;
    float att_roll_ti     = 4.0f;
    float att_roll_td     = 0.04f;
    float att_pitch_kp    = 5.0f;
    float att_pitch_ti    = 4.0f;
    float att_pitch_td    = 0.04f;

    // Altitude control
    float alt_alt_kp      = 0.6f;
    float alt_alt_ti      = 7.0f;
    float alt_vel_kp      = 0.1f;
    float alt_vel_ti      = 2.5f;

    // Position control
    float pos_pos_kp      = 1.0f;
    float pos_pos_ti      = 5.0f;
    float pos_vel_kp      = 0.3f;
    float pos_vel_ti      = 2.0f;

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

    // Safety
    float safety_accel_g     = 3.0f;
    float safety_gyro_dps    = 800.0f;
    float safety_comm_timeout = 500.0f;
    float safety_low_v       = 3.4f;
    float safety_usb_v       = 3.3f;
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

    // Attitude control
    {"attitude.roll.kp",  ParamType::FLOAT, &att_roll_kp,  5.0f,  0.0f,  50.0f,  nullptr},
    {"attitude.roll.ti",  ParamType::FLOAT, &att_roll_ti,  4.0f,  0.01f, 100.0f, nullptr},
    {"attitude.roll.td",  ParamType::FLOAT, &att_roll_td,  0.04f, 0.0f,  1.0f,   nullptr},
    {"attitude.pitch.kp", ParamType::FLOAT, &att_pitch_kp, 5.0f,  0.0f,  50.0f,  nullptr},
    {"attitude.pitch.ti", ParamType::FLOAT, &att_pitch_ti, 4.0f,  0.01f, 100.0f, nullptr},
    {"attitude.pitch.td", ParamType::FLOAT, &att_pitch_td, 0.04f, 0.0f,  1.0f,   nullptr},

    // Altitude control
    {"altitude.alt.kp",   ParamType::FLOAT, &alt_alt_kp,  0.6f,  0.0f, 10.0f,  nullptr},
    {"altitude.alt.ti",   ParamType::FLOAT, &alt_alt_ti,  7.0f,  0.1f, 100.0f, nullptr},
    {"altitude.vel.kp",   ParamType::FLOAT, &alt_vel_kp,  0.1f,  0.0f, 10.0f,  nullptr},
    {"altitude.vel.ti",   ParamType::FLOAT, &alt_vel_ti,  2.5f,  0.1f, 100.0f, nullptr},

    // Position control
    {"position.pos.kp",   ParamType::FLOAT, &pos_pos_kp,  1.0f,  0.0f, 10.0f,  nullptr},
    {"position.pos.ti",   ParamType::FLOAT, &pos_pos_ti,  5.0f,  0.1f, 100.0f, nullptr},
    {"position.vel.kp",   ParamType::FLOAT, &pos_vel_kp,  0.3f,  0.0f, 10.0f,  nullptr},
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

    // Safety
    {"safety.impact.accel_g",  ParamType::FLOAT, &safety_accel_g,     3.0f,   1.0f,   10.0f,   nullptr},
    {"safety.impact.gyro_dps", ParamType::FLOAT, &safety_gyro_dps,    800.0f, 100.0f, 2000.0f, nullptr},
    {"safety.comm.timeout_ms", ParamType::FLOAT, &safety_comm_timeout, 500.0f, 100.0f, 5000.0f, nullptr},
    {"safety.battery.low_v",   ParamType::FLOAT, &safety_low_v,       3.4f,   3.0f,   4.2f,    nullptr},
    {"safety.battery.usb_v",   ParamType::FLOAT, &safety_usb_v,       3.3f,   2.5f,   3.5f,    nullptr},
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
