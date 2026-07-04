/**
 * @file ring_buffer.hpp
 * @brief Generic ring buffer template class for sensor data
 *        センサーデータ用汎用リングバッファテンプレートクラス
 *
 * Header-only implementation. Each buffer has its own compile-time size N,
 * avoiding the one-size-fits-all REF_BUFFER_SIZE problem.
 * ヘッダーオンリー実装。各バッファが個別のコンパイル時サイズNを持ち、
 * REF_BUFFER_SIZE統一問題を解消。
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <algorithm>

template<typename T, int N>
class RingBuffer {
public:
    // =========================================================================
    // Push (write)
    // =========================================================================

    /// Push a single sample (no timestamp)
    /// 1サンプル追加（タイムスタンプなし）
    void push(const T& value) {
        data_[head_] = value;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
    }

    /// Push a single sample with timestamp
    /// タイムスタンプ付き1サンプル追加
    void push(const T& value, uint32_t ts_us) {
        data_[head_] = value;
        timestamps_[head_] = ts_us;
        has_timestamps_ = true;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
    }

    // =========================================================================
    // Access
    // =========================================================================

    /// Latest value (most recently pushed)
    /// 最新値（直近にpushされた値）
    const T& latest() const {
        int idx = (head_ - 1 + N) % N;
        return data_[idx];
    }

    /// Get value n_back samples ago (0 = latest)
    /// n_back個前の値を取得（0=最新）
    const T& get(int n_back) const {
        int idx = (head_ - 1 - n_back + N * ((n_back / N) + 1)) % N;
        return data_[idx];
    }

    /// Number of samples stored (max N)
    /// 格納済みサンプル数（最大N）
    int count() const { return count_; }

    /// Buffer capacity
    /// バッファ容量
    static constexpr int capacity() { return N; }

    /// Whether the buffer has wrapped around at least once
    /// バッファが一周したか
    bool full() const { return count_ >= N; }

    // =========================================================================
    // Timestamp access
    // =========================================================================

    /// Get timestamp n_back samples ago
    /// n_back個前のタイムスタンプを取得
    uint32_t timestamp(int n_back) const {
        int idx = (head_ - 1 - n_back + N * ((n_back / N) + 1)) % N;
        return timestamps_[idx];
    }

    struct Stamped {
        T value;
        uint32_t ts_us;
    };

    /// Get value + timestamp n_back samples ago
    /// n_back個前のデータ+タイムスタンプを取得
    Stamped get_stamped(int n_back) const {
        int idx = (head_ - 1 - n_back + N * ((n_back / N) + 1)) % N;
        return {data_[idx], timestamps_[idx]};
    }

    // =========================================================================
    // Batch retrieval
    // =========================================================================

    /// Copy the latest n samples in chronological order (oldest first)
    /// 直近n個を時系列順（古い順）にコピー
    int get_latest_n(T* out, int n) const {
        int actual = std::min(n, count_);
        for (int i = 0; i < actual; i++) {
            out[i] = get(actual - 1 - i);
        }
        return actual;
    }

    // =========================================================================
    // Statistics (for arithmetic types and Vector3)
    // =========================================================================

    /// Mean of the latest n samples (-1 = all stored)
    /// 直近n個の平均（-1=全体）
    T mean(int n = -1) const {
        int actual = (n < 0) ? count_ : std::min(n, count_);
        if (actual == 0) return T{};
        T sum{};
        for (int i = 0; i < actual; i++) {
            sum += get(i);
        }
        return sum * (1.0f / actual);
    }

    // =========================================================================
    // Raw index access (for telemetry synchronization)
    // テレメトリ同期用の内部インデックス直接アクセス
    // =========================================================================

    /// Current write head index
    /// 現在の書き込みヘッドインデックス
    int raw_index() const { return head_; }

    /// Direct access by raw index (no bounds checking)
    /// インデックス指定で直接アクセス（境界チェックなし）
    const T& raw_at(int idx) const { return data_[idx]; }

    /// Direct timestamp access by raw index
    /// インデックス指定でタイムスタンプ直接アクセス
    uint32_t raw_timestamp_at(int idx) const { return timestamps_[idx]; }

    /// Check if a reader's index has been overrun by the writer.
    /// Returns true if the writer has lapped the reader, meaning data at
    /// read_idx is stale/overwritten.
    /// リーダーのインデックスがライターに追い越されたか判定。
    /// true の場合、read_idx のデータは上書き済みで不正。
    bool is_overrun(int read_idx) const {
        if (count_ < N) return false;  // Buffer hasn't wrapped yet
        // Distance from read_idx to head (write pointer)
        int distance = (head_ - read_idx + N) % N;
        // If distance is 0 the reader caught up to the writer (also overrun)
        // If distance > N/2 it means read is far behind (likely overrun)
        // Exact condition: buffer is full and distance is 0 (reader == writer)
        // But since writer advances continuously, any distance < safety_margin
        // means risk of reading partially-overwritten data.
        // Practical threshold: treat as overrun if reader is within 10 samples
        // of the write head (2.5% of buffer at size 400).
        return distance == 0 || distance > (N - 10);
    }

    /// Reset a reader's index to a safe position (latest minus margin).
    /// Returns the new read index.
    /// リーダーのインデックスを安全な位置にリセット（最新から margin 分戻す）。
    int safe_read_index(int margin = 10) const {
        return (head_ - margin + N) % N;
    }

    // =========================================================================
    // Reset
    // =========================================================================

    /// Clear all data
    /// 全データクリア
    void reset() {
        head_ = 0;
        count_ = 0;
        has_timestamps_ = false;
    }

private:
    T data_[N]{};
    uint32_t timestamps_[N]{};
    int head_ = 0;
    int count_ = 0;
    bool has_timestamps_ = false;
};
