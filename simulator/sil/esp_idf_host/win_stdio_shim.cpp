/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file win_stdio_shim.cpp
 * @brief Definitions backing esp_idf_host/cstdio's stdout/stderr shadow —
 *        see that header for why this shim exists. Windows-only (empty
 *        translation unit elsewhere).
 *        esp_idf_host/cstdio の stdout/stderr シャドウを実体化する定義
 *        （理由は同ヘッダ参照）。Windows 限定（他プラットフォームでは空）。
 */

#ifdef _WIN32
#include <cstdio>

// Call the CRT's internal accessor directly, bypassing the stdout/stderr
// macros entirely (this file's own <cstdio> include above resolves to this
// very shim, which has already #undef'd them) -- so this captures the REAL,
// original file streams once at process start, not a self-reference.
// CRT の内部アクセサを直接呼び、stdout/stderr マクロを経由しない（このファイル
// 自身の <cstdio> インクルードは既に #undef 済みの本シムを指すため）。よって
// プロセス開始時に「本物の」元ストリームを捕捉する（自己参照にならない）。
extern "C" FILE* __cdecl __acrt_iob_func(unsigned index);

FILE* g_sil_win_stdout = __acrt_iob_func(1);
FILE* g_sil_win_stderr = __acrt_iob_func(2);
#endif
