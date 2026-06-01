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
#include <cstdio>
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
            if (++retries > kMaxYieldRetries) break;  // give up (bounded)
            vTaskDelay(1);
            continue;
        }
        break;  // hard error (e.g. EPIPE) — stop
    }

    // Record exactly the bytes the firmware actually received (virtual-time
    // stamped). Recording AFTER the loop keeps the events log truthful even on a
    // partial write; a short write is also flagged so the drop is visible.
    // ファームが実際に受け取ったバイトだけを記録（仮想時刻）。途中脱落も正直に残す。
    if (written > 0) {
        sil_emu_record_bytes("key", reinterpret_cast<const uint8_t*>(bytes), written);
    }
    if (written < n) {
        sil_emu_record_note("key_drop", "pipe never drained — bytes dropped");
        std::fprintf(stderr, "[console_feeder] pipe never drained: dropped %d/%d bytes\n",
                     n - written, n);
    }
}

}  // extern "C"
