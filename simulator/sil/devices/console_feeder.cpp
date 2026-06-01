/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file console_feeder.cpp
 * @brief Deterministic console byte feeder implementation.
 *        決定論的なコンソール・バイトフィーダの実装。
 */

#include "console_feeder.hpp"
#include "emu_record.hpp"

#include <cerrno>
#include <cstdint>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

int g_cli_fd = -1;

// Bound a single write so a pathological scenario cannot spin forever.
// 1回の書き込みの上限（病的なシナリオで永久ループしないため）。
constexpr int kMaxYieldRetries = 4096;

}  // namespace

extern "C" {

void sil_console_set_fd(int fd) { g_cli_fd = fd; }

void sil_console_write(const char* bytes, int n)
{
    if (g_cli_fd < 0 || bytes == nullptr || n <= 0) return;

    // Record what we feed (virtual-time stamped) before delivering.
    // 配信前に投入バイトを記録（仮想時刻スタンプ）。
    sil_emu_record_bytes("key", reinterpret_cast<const uint8_t*>(bytes), n);

    int written = 0;
    int retries = 0;
    while (written < n) {
        ssize_t r = write(g_cli_fd, bytes + written, (size_t)(n - written));
        if (r > 0) {
            written += (int)r;
            retries = 0;
            continue;
        }
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // Pipe full: yield one tick so the firmware CLI task can drain it,
            // then retry. NEVER block the run-token in the kernel.
            // pipe 満杯: 1tick yield して CLI タスクに排出させ再試行。
            if (++retries > kMaxYieldRetries) return;  // give up (bounded)
            vTaskDelay(1);
            continue;
        }
        return;  // hard error (e.g. EPIPE) — stop
    }
}

}  // extern "C"
