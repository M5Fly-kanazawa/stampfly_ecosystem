/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_ring.hpp
 * @brief Fixed-capacity FIFO of text lines — the buffer behind the log sink
 *        固定容量の行 FIFO — ログ受け渡し（ログシンク）の背後にあるバッファ
 *
 * Why: an ESP_LOG call writes to the USB console, and the console can make the caller
 * wait (the transmit buffer is small; when the USB host stops taking data the wait runs
 * to 50 ms). A 400 Hz task that logs
 * once at ARM therefore misses its 2.5 ms deadline. The fix is to let those tasks only
 * COPY the finished line into this ring (microseconds) and let LogTask, which has no
 * deadline, write it to the console.
 *
 * This class is the pure data structure: no FreeRTOS, no lock, no ESP-IDF, so it runs on
 * the host test (test_log_ring.cpp). The caller serialises access (log_task.cpp wraps
 * push/pop in a short critical section). A line is stored as an eight-byte prefix (a two-byte
 * length, a one-byte count of the lines dropped just before it, a one-byte flag byte, a
 * four-byte time stamp) followed by its text, so a line is at most kMaxLineBytes long. The time
 * stamp and the truncation flag are what the WiFi exit of the sink sends with the line; the
 * console exit ignores them.
 *
 * 理由: ESP_LOG は USB コンソールへ書き、コンソールは呼び出し側を待たせうる（送信バッファは小さく、
 * USB のホストがデータを取り出さなくなると待ちは 50 ms に達する）。ARM の瞬間に 1 行ログを出す 400 Hz タスクは、そのため 2.5 ms の
 * 締切を落とす。対策は、そのタスクは出来上がった行をこのリングへ「コピーするだけ」（数 µs）に
 * して、締切を持たない LogTask がコンソールへ書くこと。
 *
 * このクラスは純粋なデータ構造: FreeRTOS・ロック・ESP-IDF を使わないのでホスト試験
 * （test_log_ring.cpp）で動く。排他は呼び出し側の責務（log_task.cpp が push/pop を短い
 * クリティカルセクションで包む）。1 行は「接頭辞 8 バイト（長さ 2 バイト・直前に落とした行数 1 バイト・
 * フラグ 1 バイト・時刻 4 バイト）＋本文」で格納するので、1 行は最大 kMaxLineBytes 文字。時刻と切り詰めの
 * フラグは、シンクの WiFi 側の出口が行と一緒に送るもの。コンソール側の出口は使わない。
 *
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console  [OK]
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace sf {

/// Fixed-capacity FIFO of text lines (not thread-safe: the caller holds the lock).
/// 固定容量の行 FIFO（スレッドセーフではない: 呼び出し側がロックを持つ）。
template <size_t kCapacityBytes>
class LogRing {
public:
    /// Longest line that can be stored (also the size of the caller's stack buffer, so it is
    /// kept modest). The longest log message of the vehicle is about 330 characters including
    /// the time and tag prefix; a longer line is cut by the caller.
    /// 格納できる最長の 1 行（呼び出し側のスタックバッファの大きさでもあるので控えめにする）。機体のログで
    /// 最長のメッセージは、時刻とタグの接頭辞を含めて約 330 文字。これより長い行は呼び出し側が切る。
    static constexpr size_t kMaxLineBytes = 384;

    /// Bytes of the prefix in front of each stored line. / 格納する各行の前に付く接頭辞のバイト数。
    static constexpr size_t kPrefixBytes = 8;

    /// Largest number of bytes one stored line occupies (prefix + longest text); a reader that
    /// budgets bytes per period needs this to know that one line always fits.
    /// 格納した 1 行が占める最大のバイト数（接頭辞＋最長の本文）。周期ごとにバイト数の予算を持つ読み手は、
    /// 1 行が必ず収まることを知るのにこれを使う。
    static constexpr size_t kMaxStoredLineBytes = kPrefixBytes + kMaxLineBytes;

    /// Flag bit of a stored line: the text was cut to kMaxLineBytes by the caller.
    /// 格納した行のフラグのビット: 呼び出し側が本文を kMaxLineBytes に切り詰めた。
    static constexpr uint8_t kFlagTruncated = 0x01;

    /// What a line carries besides its text. / 行が本文のほかに持つもの。
    struct LineInfo {
        uint32_t timestamp_us = 0;     ///< [us] when the line entered the ring / 行がリングに入った時刻
        bool     truncated    = false; ///< the text was cut by the caller / 本文が呼び出し側で切り詰められた
    };

    /// Largest drop count stored with one line (one byte). / 1 行と一緒に格納できる落とし数の上限（1 バイト）。
    static constexpr uint32_t kMaxDroppedBefore = 255;

    static_assert(kCapacityBytes > kMaxLineBytes + kPrefixBytes,
                  "the ring must hold at least one longest line / リングは最長の 1 行を入れられること");

    /// Append one line. Returns false (and counts a drop) when it does not fit or is
    /// longer than kMaxLineBytes; a stored line is never cut. The drops counted so far
    /// (up to kMaxDroppedBefore) are stored with this line, so the reader can report
    /// them at the place where the lines went missing. timestamp_us and truncated are stored
    /// with the line (truncated: the caller already cut the text and says so).
    /// 1 行を追加する。入らない、または kMaxLineBytes を超えるときは false を返し、落とした数を
    /// 数える。格納済みの行が途中で切れることはない。ここまでに数えた落とし数（最大
    /// kMaxDroppedBefore）はこの行と一緒に格納し、読む側が行が抜けた位置で報告できるようにする。
    /// timestamp_us と truncated は行と一緒に格納する（truncated: 呼び出し側が既に本文を切り詰めており、
    /// そのことを伝える）。
    bool push(const char* text, size_t length, uint32_t timestamp_us = 0, bool truncated = false)
    {
        if (length == 0) {
            return true;   // nothing to store / 格納するものがない
        }
        if (length > kMaxLineBytes || used_ + kPrefixBytes + length > kCapacityBytes) {
            ++dropped_;
            ++dropped_total_;
            return false;
        }
        const uint32_t dropped_before = (dropped_ < kMaxDroppedBefore) ? dropped_ : kMaxDroppedBefore;
        dropped_ -= dropped_before;
        const uint8_t prefix[kPrefixBytes] = {static_cast<uint8_t>(length & 0xFFu),
                                              static_cast<uint8_t>(length >> 8),
                                              static_cast<uint8_t>(dropped_before),
                                              static_cast<uint8_t>(truncated ? kFlagTruncated : 0),
                                              static_cast<uint8_t>(timestamp_us & 0xFFu),
                                              static_cast<uint8_t>((timestamp_us >> 8) & 0xFFu),
                                              static_cast<uint8_t>((timestamp_us >> 16) & 0xFFu),
                                              static_cast<uint8_t>(timestamp_us >> 24)};
        write(prefix, kPrefixBytes);
        write(reinterpret_cast<const uint8_t*>(text), length);
        return true;
    }

    /// Remove the oldest line into out (at most out_size bytes; no terminator is added).
    /// Returns the line length, or 0 when the ring is empty or out is too small for the
    /// oldest line (the line then stays; give a buffer of kMaxLineBytes to be safe).
    /// dropped_before receives the number of lines dropped just before this line; info (when
    /// given) receives the time stamp and the truncation flag stored with it.
    /// 最も古い行を out へ取り出す（最大 out_size バイト。終端文字は付けない）。行の長さを返す。
    /// 空のとき、または out が最古の行に対して小さいときは 0（その行は残る。安全には
    /// kMaxLineBytes の領域を渡す）。dropped_before にはこの行の直前に落とした行数が入る。info（指定時）には
    /// 行と一緒に格納した時刻と切り詰めのフラグが入る。
    size_t pop(char* out, size_t out_size, uint32_t* dropped_before, LineInfo* info = nullptr)
    {
        if (used_ == 0) {
            return 0;
        }
        uint8_t prefix[kPrefixBytes];
        peek(prefix, kPrefixBytes);
        const size_t length = static_cast<size_t>(prefix[0]) | (static_cast<size_t>(prefix[1]) << 8);
        if (length > out_size) {
            return 0;
        }
        *dropped_before = prefix[2];
        if (info != nullptr) {
            info->truncated    = (prefix[3] & kFlagTruncated) != 0;
            info->timestamp_us = static_cast<uint32_t>(prefix[4]) | (static_cast<uint32_t>(prefix[5]) << 8) |
                                 (static_cast<uint32_t>(prefix[6]) << 16) | (static_cast<uint32_t>(prefix[7]) << 24);
        }
        skip(kPrefixBytes);
        read(reinterpret_cast<uint8_t*>(out), length);
        return length;
    }

    /// Length of the oldest stored line (0 when empty), without removing it — a reader with a byte
    /// budget asks this before it pops.
    /// 最も古い格納済みの行の長さ（空なら 0）。取り出さずに知る — バイト数の予算を持つ読み手が、取り出す前に使う。
    size_t oldestLength() const
    {
        if (used_ == 0) {
            return 0;
        }
        uint8_t prefix[kPrefixBytes];
        peek(prefix, kPrefixBytes);
        return static_cast<size_t>(prefix[0]) | (static_cast<size_t>(prefix[1]) << 8);
    }

    /// Number of dropped lines not yet attached to a stored line (drops after the last
    /// stored line), and clear the count.
    /// まだ格納済みの行に付いていない落とし数（最後に格納した行より後の落とし）を返し、数を 0 に戻す。
    uint32_t takeDropped()
    {
        const uint32_t dropped = dropped_;
        dropped_ = 0;
        return dropped;
    }

    /// Lines refused since the last clear(), in total (never reduced by takeDropped() or by the
    /// drop counts attached to stored lines). The status packet carries it as the text-log loss.
    /// 前回の clear() 以降に断った行数の合計（takeDropped() や、格納した行に付けた落とし数では減らない）。
    /// ステータスパケットがテキストログの欠落として運ぶ。
    uint32_t totalDropped() const { return dropped_total_; }

    /// Forget every stored line and every count (a capture starts from "now").
    /// 格納した行と数をすべて忘れる（取得は「今」から始まる）。
    void clear()
    {
        head_          = 0;
        tail_          = 0;
        used_          = 0;
        dropped_       = 0;
        dropped_total_ = 0;
    }

    /// True when no line is waiting. / 待っている行がないとき true。
    bool empty() const { return used_ == 0; }

    /// Bytes in use (length prefixes included). / 使用中のバイト数（長さの接頭辞を含む）。
    size_t usedBytes() const { return used_; }

private:
    /// Copy count bytes in at head_, in at most two pieces (the end of the buffer splits them).
    /// head_ の位置へ count バイトを書く。バッファの末尾をまたぐときは最大 2 回に分けてコピーする。
    void write(const uint8_t* source, size_t count)
    {
        const size_t first = (count < kCapacityBytes - head_) ? count : kCapacityBytes - head_;
        std::memcpy(&buffer_[head_], source, first);
        std::memcpy(&buffer_[0], source + first, count - first);
        head_ = (head_ + count) % kCapacityBytes;
        used_ += count;
    }

    /// Copy count bytes out from tail_ without consuming them.
    /// tail_ の位置から count バイトを、消費せずに読む。
    void peek(uint8_t* destination, size_t count) const
    {
        const size_t first = (count < kCapacityBytes - tail_) ? count : kCapacityBytes - tail_;
        std::memcpy(destination, &buffer_[tail_], first);
        std::memcpy(destination + first, &buffer_[0], count - first);
    }

    /// Consume count bytes at tail_. / tail_ の位置の count バイトを消費する。
    void skip(size_t count)
    {
        tail_ = (tail_ + count) % kCapacityBytes;
        used_ -= count;
    }

    /// Copy count bytes out from tail_ and consume them. / tail_ から count バイトを読んで消費する。
    void read(uint8_t* destination, size_t count)
    {
        peek(destination, count);
        skip(count);
    }

    uint8_t  buffer_[kCapacityBytes] = {};
    size_t   head_    = 0;   // next write index / 次に書く位置
    size_t   tail_    = 0;   // next read index  / 次に読む位置
    size_t   used_    = 0;   // bytes in use     / 使用中のバイト数
    uint32_t dropped_ = 0;   // lines refused since the last takeDropped() / 前回以降に断った行数
    uint32_t dropped_total_ = 0;   // lines refused since the last clear() / 前回の clear() 以降に断った行数の合計
};

}  // namespace sf
