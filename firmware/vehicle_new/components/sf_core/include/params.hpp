/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file params.hpp
 * @brief Parameter system — public API. Definitions live in params.cpp (SSOT).
 *        パラメータシステム — 公開 API。定義は params.cpp（SSOT）にある。
 *
 * The single source of truth for every tunable is params.cpp: the typed
 * variables in namespace param_vars plus the explicit table[] that binds each
 * name → variable → default/min/max/callback. To add a parameter, add the
 * variable to param_vars AND a row to table[] (two edits, same file). The
 * explicit table is intentional — it reads clearly as an exemplar and keeps the
 * value/range/rationale visible (vs. hidden X-macro generation). Values are
 * accessed by name via the get_/set_ accessors, persisted to NVS by save/load.
 *
 * 全チューニング値の SSOT は params.cpp: namespace param_vars の型付き変数と、
 * 名前→変数→既定/最小/最大/コールバックを結ぶ明示的 table[]。パラメータ追加は
 * param_vars に変数を、table[] に行を足す（同一ファイル2箇所）。明示テーブルは
 * 意図的 — 模範コードとして読みやすく、値/範囲/根拠が見える（X-macro 生成で
 * 隠さない）。値は get_/set_ 関数で名前アクセス、save/load で NVS 永続化。
 *
 * @design requirements.md §3 — Parameter management                   [OK]
 * @design detailed_design.md §6 — Parameter table (SSOT = params.cpp)  [OK]
 * @design coding_and_education.md §2 — Bilingual comments              [OK]
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace sf {
namespace params {

// =============================================================================
// Parameter Types
// パラメータ型
// =============================================================================

enum class ParamType : uint8_t {
    FLOAT = 0,
    BOOL  = 1,
    INT   = 2,
};

/// Callback function type — called when parameter value changes
/// コールバック関数型 — パラメータ値の変更時に呼ばれる
using ParamCallback = void (*)(void);

// =============================================================================
// Parameter Entry (internal table structure)
// パラメータエントリ（内部テーブル構造体）
// =============================================================================

struct ParamEntry {
    const char* name;         // Parameter name / パラメータ名
    ParamType type;           // Value type / 値の型
    void* value_ptr;          // Pointer to variable / 変数へのポインタ
    float default_val;        // Default value / デフォルト値
    float min_val;            // Minimum value / 最小値
    float max_val;            // Maximum value / 最大値
    ParamCallback callback;   // Change callback (null = none) / 変更コールバック
};

// Parameter variables and the table[] that registers them are defined in
// params.cpp (the SSOT). They are not declared here: callers use the get_/set_
// API below by name, not the variables directly.
// パラメータ変数と登録 table[] は params.cpp（SSOT）で定義する。ここでは宣言しない:
// 呼び出し側は下の get_/set_ API で名前アクセスし、変数を直接触らない。

// =============================================================================
// Public API
// 公開API
//
// @design detailed_design.md §6 — Parameter access API                [OK]
// =============================================================================

/// Initialize parameter system — load from NVS or use defaults
/// パラメータシステムを初期化する — NVSから読み込みまたはデフォルトを使用
void init();

/// Get parameter value by name
/// 名前でパラメータ値を取得する
///
/// @return true if parameter found / パラメータが見つかればtrue
bool get_float(const char* name, float& out);
bool get_bool(const char* name, bool& out);
bool get_int(const char* name, int32_t& out);

/// Set parameter value by name
/// 名前でパラメータ値を設定する
///
/// Validates range, updates the RAM value, calls the callback. NOT persisted —
/// NVS write happens only on an explicit save() (CLI `param save`); per-set
/// flash writes would wear the flash during interactive tuning.
/// 範囲を検証し、RAM 上の値を更新し、コールバックを呼ぶ。永続化はしない —
/// NVS 書き込みは明示的な save()（CLI `param save`）のみ。set のたびに書くと
/// 対話チューニング中にフラッシュを磨耗させるため。
///
/// @return true if set succeeded / 設定成功ならtrue
bool set_float(const char* name, float value);
bool set_bool(const char* name, bool value);
bool set_int(const char* name, int32_t value);

/// Save all parameters to NVS
/// 全パラメータをNVSに保存する
void save();

/// Load all parameters from NVS
/// 全パラメータをNVSから読み込む
void load();

/// Reset all parameters to default values
/// 全パラメータをデフォルト値にリセットする
void reset_all();

/// Print all parameters (for CLI)
/// 全パラメータを表示する（CLI用）
void list();

/// Get number of registered parameters
/// 登録済みパラメータ数を取得する
int count();

/// Get parameter entry by index (for iteration)
/// インデックスでパラメータエントリを取得する（反復用）
const ParamEntry* entry(int index);

}  // namespace params
}  // namespace sf
