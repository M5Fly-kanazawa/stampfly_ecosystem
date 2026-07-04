// Stub esp_timer.h for PC unit testing — a controllable mock clock.
// PC単体テスト用の esp_timer.h スタブ — 制御可能なモッククロック。
//
// Tests drive g_mock_esp_time_us to advance virtual time between update() calls, so a
// time-based detector (TakeoffLandingMgr's landing_hold/stall_hold) can be exercised
// deterministically without sleeping.
// テストは g_mock_esp_time_us を進めて update() 呼び出し間の仮想時間を制御し、時間ベースの
// 検出器（TakeoffLandingMgr の landing_hold/stall_hold）を sleep なしで決定論的に検証する。
#pragma once
#include <cstdint>

extern int64_t g_mock_esp_time_us;   // defined in test_main.cpp / test_main.cpp で定義

static inline int64_t esp_timer_get_time() { return g_mock_esp_time_us; }
