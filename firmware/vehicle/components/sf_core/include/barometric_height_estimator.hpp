/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file barometric_height_estimator.hpp
 * @brief Height above the floor from the barometer, with a zero taken on the floor and an offset learned from the ToF
 *        床からの高さを気圧計から。ゼロは床で取り、オフセットは ToF から学習する
 *
 * Pure logic (no ESP-IDF / FreeRTOS), host unit-testable (test/test_barometric_height.cpp, and the offline evaluation
 * analysis/scripts/baro_offset_eval.cpp that compiles this very header). ImuTask owns one instance.
 * OBSERVATION ONLY: nothing in the control path, the estimator or the touchdown detection reads its output.
 *
 * What the BMP280 gives is an ABSOLUTE pressure altitude (44330 (1 - (p / 101325)^0.1903): real logs read -50 m, -14 m, +80 m, +780 m: the weather and the place),
 * so a height above the floor needs a reference, and in flight the reading also carries what the craft itself does to the pressure at the
 * sensor (the propeller wash raises the reading by 0.1-0.4 m on the floor with the motors running and by 0.2-0.9 m in hover) and a slow drift. Three parts:
 *   1. Floor zero: while the craft is DISARMED and on the floor, the absolute altitude is averaged (time constant kBaroZeroTauS)
 *      and the average is FROZEN at the ARM (the motors have not started: the propeller wash is not in it).
 *   2. Learned offset: while armed and the ToF is usable (valid, estimator-reliable, airborne, height inside
 *      [kBaroLearnMinHeightM, kBaroLearnMaxHeightM], tilt below kBaroLearnMaxTiltRad), the offset
 *      offset = (absolute altitude - floor zero) - ToF height is averaged (a running mean over the first kBaroLearnTauS, then a first-order filter
 *      of that time constant). It absorbs the propeller-wash term and the drift as they were while the ToF was alive.
 *   3. Hold: while the ToF is not usable the offset is HELD. The output is then
 *        height = absolute altitude - floor zero - offset,
 *      which follows the real height with the error of how much the wash term and the drift moved since the last learning (the age is reported).
 * New-sample detection: the BMP280 runs at about 13 Hz and BaroTask publishes at 50 Hz, so the same conversion arrives 3-4 times. The driver reports no conversion
 * time stamp (its time stamp is the read time) and the data-ready bit of the status register is not read, so a conversion is recognised by its PRESSURE VALUE: the 20-bit result
 * changes with almost every conversion (the noise is 2-4 LSB with the sensor's IIR filter), and a repeat is bit-identical. A new conversion that
 * happens to give the same value is taken as a repeat: it costs one sample (rate 13.3 Hz measured 12.0-13.8 Hz unique values in the real logs, no systematic loss), never a wrong value.
 *
 * ゼロや学習の考え方は上の英文のとおり（BMP280 は絶対の気圧高度を出す。床からの高さには基準が要り、飛行中は機体自身が
 * センサの圧力に与える影響（プロペラの吹き下ろしで、床でモータが回っているとき 0.1〜0.4 m、ホバーで 0.2〜0.9 m 高く読む）とゆっくりしたドリフトが乗る）。3 つの部分:
 *   1. 床のゼロ: DISARM で床の上にいる間、絶対高度を平均し（時定数 kBaroZeroTauS）、ARM で固定する（モータはまだ回っておらず、吹き下ろしは入っていない）。
 *   2. 学習するオフセット: ARM 中で ToF が使える間（有効・推定器が信頼・空中・高さが範囲内・傾きが kBaroLearnMaxTiltRad 未満）、
 *      オフセット =（絶対高度 − 床のゼロ）− ToF の高さ を平均する（最初の kBaroLearnTauS は累積平均、その後は同じ時定数の 1 次フィルタ）。ToF が生きていたときの
 *      吹き下ろしの項とドリフトを吸収する。
 *   3. 保持: ToF が使えない間はオフセットを「保持」する。出力は 高さ = 絶対高度 − 床のゼロ − オフセット。最後の学習から吹き下ろしの項とドリフトが動いた分だけ実際の高さからずれる（学習からの経過時間を報告する）。
 * 新サンプルの検出: BMP280 は約 13 Hz で、BaroTask は 50 Hz で発行するので、同じ変換結果が 3〜4 回届く。ドライバは変換の時刻を報告せず（時刻は読み出し時刻）、
 * ステータスレジスタの data-ready ビットも読まないので、変換は「気圧の値」で見分ける: 20 ビットの結果はほぼ毎回変わり（センサの IIR フィルタ付きでノイズは 2〜4 LSB）、繰り返しは
 * ビットまで同一。たまたま同じ値になった新しい変換は繰り返しとして扱う: 失うのは 1 サンプルで、誤った値にはならない（実機ログの一意な値は 12.0〜13.8 Hz、系統的な欠落なし）。
 *
 * @design detailed_design.md §8 — BaroTask / barometric height (observation only)  [OK]
 * @design requirements.md §5 — the barometer is not used for control                 [OK]
 * @design architecture.md INV-3 — facts here, no decision made from them             [OK]
 * @design coding_and_education.md §2 — Bilingual comments, named constants           [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "flight_limits.hpp"

namespace sf {

/// Bits of the published flags (SensorSnapshot::baro_height_flags, the Data Stream baro_height entry). 発行するフラグのビット。
inline constexpr uint8_t kBaroHeightFlagValid   = 0x01;   ///< a floor zero is held or an offset is learned / 床のゼロを持つか、オフセットを学習済み
inline constexpr uint8_t kBaroHeightFlagLearned = 0x02;   ///< the offset was learned since the last ARM / 最後の ARM 以降にオフセットを学習した
/// The age is published in tenths of a second, saturating at 0xFFFF - 1; 0xFFFF means never learned. 経過時間は 0.1 s 単位で、0xFFFE で飽和。0xFFFF は未学習。
inline constexpr float kBaroAgeDecisPerSecond = 10.0f;
inline constexpr float kBaroAgeSaturatedDecis = 65534.0f;
inline constexpr float kBaroAgeNeverDecis     = 65535.0f;

/// One barometer sample as published on sensor_baro. 気圧計の 1 サンプル（sensor_baro の内容）。
struct BarometricSample {
    float    altitude_m;      ///< Absolute pressure altitude [m]  絶対の気圧高度 [m]
    float    pressure_pa;     ///< Pressure [Pa] (identifies the conversion)  気圧 [Pa]（変換の識別に使う）
    uint32_t timestamp_us;    ///< Read time [us]  読み出し時刻 [us]
};

/// The craft state a barometer sample is judged in. 気圧計のサンプルを判断するときの機体の状態。
struct BarometricContext {
    bool armed;       ///< Motors armed (the zero is frozen, the offset is learned)  ARM 中（ゼロを固定し、オフセットを学習する）
    bool on_ground;   ///< On the floor, as the system may trust it (the zero is tracked only while disarmed AND on the floor)  床の上（システムが信用できる形で）。ゼロは DISARM かつ床の上の間だけ追跡する
};

/// One ToF height, as ImuTask derives it. ImuTask が作る ToF の高さ 1 つ。
struct BarometricToFSample {
    float    height_m;        ///< Tilt-corrected height above the floor [m]  傾き補正した床からの高さ [m]
    float    tilt_rad;        ///< Tilt of the craft [rad]  機体の傾き [rad]
    uint32_t timestamp_us;    ///< [us]
    bool     usable;          ///< Valid, estimator-reliable and airborne  有効で、推定器が信頼し、空中
};

/// Tunable settings (defaults: sf::limits; the offline evaluation sweeps them). 調整値（既定は sf::limits。オフライン評価が掃引する）。
struct BarometricHeightConfig {
    float    zero_tau_s          = limits::kBaroZeroTauS;          ///< [s] floor-zero time constant      床のゼロの時定数
    float    learn_tau_s         = limits::kBaroLearnTauS;         ///< [s] offset time constant          オフセットの時定数
    float    learn_min_height_m  = limits::kBaroLearnMinHeightM;   ///< [m]                                学習する高さの下限
    float    learn_max_height_m  = limits::kBaroLearnMaxHeightM;   ///< [m]                                学習する高さの上限
    float    learn_max_tilt_rad  = limits::kBaroLearnMaxTiltRad;   ///< [rad]                              学習する傾きの上限
    float    learn_gate_m        = limits::kBaroLearnGateM;        ///< [m] innovation gate               誤差の判定幅
    uint32_t learn_max_tof_age_us = limits::kBaroLearnMaxTofAgeUs; ///< [us]                               ToF サンプルの最大の古さ
    bool     learning_enabled    = true;                            ///< false: floor zero only (evaluation baseline)  false: 床のゼロだけ（評価の比較対象）
};

/// What the estimator reports. 推定器の出力。
struct BarometricHeightOutput {
    float height_m;        ///< Height above the floor [m] (0 until the first sample)  床からの高さ [m]（最初のサンプルまで 0）
    float offset_m;        ///< Learned offset [m]  学習したオフセット [m]
    bool  valid;           ///< A floor zero is held or an offset has been learned  床のゼロを持つか、オフセットを学習済み
    bool  learned;         ///< The offset has been learned since the last ARM  最後の ARM 以降にオフセットを学習した
    float offset_age_s;    ///< [s] since the offset was last updated; kNeverLearnedAgeS if never  オフセットを最後に更新してからの時間 [s]。一度もなければ kNeverLearnedAgeS
};

/// Floor-relative barometric height. 床基準の気圧高度。
class BarometricHeightEstimator {
public:
    /// Age reported while the offset has never been learned [s]. 一度も学習していない間に報告する経過時間 [s]。
    static constexpr float kNeverLearnedAgeS = 6553.5f;

    void setConfig(const BarometricHeightConfig& config) { config_ = config; }   ///< Apply settings / 設定を反映

    /// Back to power-on. 電源投入直後に戻す。
    void reset()
    {
        const BarometricHeightConfig keep = config_;
        *this = BarometricHeightEstimator();
        config_ = keep;
    }

    /// Feed the latest ToF height (every ToF sample, usable or not). 最新の ToF の高さを与える（有効・無効を問わず ToF サンプルごと）。
    void onToFSample(const BarometricToFSample& sample) { tof_ = sample; has_tof_ = true; }

    /// Feed one sensor_baro sample. Returns true when it is a NEW conversion (a repeat changes nothing).
    /// sensor_baro のサンプルを 1 つ与える。新しい変換なら true を返す（繰り返しは何も変えない）。
    bool onBaroSample(const BarometricSample& sample, const BarometricContext& context)
    {
        if (has_last_ && sample.pressure_pa == last_pressure_pa_) return false;
        const float step_s = has_last_ ? sampleStepS(sample.timestamp_us) : 0.0f;
        has_last_         = true;
        last_pressure_pa_ = sample.pressure_pa;
        last_timestamp_us_ = sample.timestamp_us;
        last_altitude_m_  = sample.altitude_m;
        if (context.armed != was_armed_) clearOffset();
        was_armed_ = context.armed;
        if (context.armed) {
            learnOffset(sample, step_s);
        } else if (context.on_ground) {
            trackZero(sample.altitude_m, step_s);
        }
        return true;
    }

    /// The output as of the last new sample; the age is counted to `now_us`. 最後の新サンプル時点の出力。経過時間は now_us まで数える。
    BarometricHeightOutput output(uint32_t now_us) const
    {
        BarometricHeightOutput out{};
        out.valid    = zero_set_ || learned_;
        out.learned  = learned_;
        out.offset_m = offset_m_;
        out.height_m = has_last_ ? (last_altitude_m_ - zero_m_ - offset_m_) : 0.0f;
        out.offset_age_s = learned_ ? microsecondsToSeconds(now_us - last_learn_us_) : kNeverLearnedAgeS;
        return out;
    }

    float zeroM() const { return zero_m_; }          ///< Floor zero [m] (absolute altitude) / 床のゼロ [m]（絶対高度）
    bool  zeroHeld() const { return zero_set_; }     ///< A floor zero has been taken / 床のゼロを取った

private:
    /// Longest step between two conversions counted in the filters [s]: three nominal periods (75 ms), so that a gap does not make one sample weigh more than a regular one.
    /// 2 つの変換の間隔のうちフィルタが数える上限 [s]: 公称周期（75 ms）の 3 倍。欠落のあとの 1 サンプルが通常より重くならないようにする。
    static constexpr float kMaxStepS = 0.25f;
    static constexpr float kMicrosecondsPerSecond = 1.0e6f;

    static float microsecondsToSeconds(uint32_t microseconds)
    {
        return static_cast<float>(microseconds) / kMicrosecondsPerSecond;
    }

    /// Time since the previous new conversion, bounded to [0, kMaxStepS]. 前の新しい変換からの時間（0〜kMaxStepS に制限）。
    float sampleStepS(uint32_t timestamp_us) const
    {
        const float step = microsecondsToSeconds(timestamp_us - last_timestamp_us_);
        return step > kMaxStepS ? kMaxStepS : step;
    }

    /// An ARM starts a new flight and a DISARM ends it: the offset belongs to one flight (unknown again, floor zero only).
    /// ARM で新しい飛行が始まり、DISARM で終わる: オフセットは 1 回の飛行のもの（再び未知で、床のゼロだけ）。
    void clearOffset()
    {
        offset_m_    = 0.0f;
        learned_     = false;
        learn_count_ = 0;
    }

    /// Disarmed on the floor: average the absolute altitude (the first sample seeds it). DISARM で床の上: 絶対高度を平均する（最初のサンプルが初期値）。
    void trackZero(float altitude_m, float step_s)
    {
        if (!zero_set_) {
            zero_m_   = altitude_m;
            zero_set_ = true;
            return;
        }
        float gain = step_s / config_.zero_tau_s;
        if (gain > 1.0f) gain = 1.0f;
        zero_m_ += gain * (altitude_m - zero_m_);
    }

    /// Is the newest ToF sample one the offset may be learned from, at the time of `sample`? 最新の ToF サンプルは、このサンプルの時点でオフセットを学習してよいものか。
    bool toFAcceptable(uint32_t timestamp_us) const
    {
        if (!has_tof_ || !tof_.usable) return false;
        const uint32_t age_us = timestamp_us - tof_.timestamp_us;
        if (age_us > config_.learn_max_tof_age_us) return false;
        if (tof_.height_m < config_.learn_min_height_m || tof_.height_m > config_.learn_max_height_m) return false;
        return tof_.tilt_rad < config_.learn_max_tilt_rad;
    }

    /// Armed: average (absolute altitude - zero) - ToF height while the ToF is usable; otherwise hold. ARM 中: ToF が使える間、（絶対高度 − ゼロ）− ToF の高さ を平均する。使えなければ保持。
    void learnOffset(const BarometricSample& sample, float step_s)
    {
        if (!config_.learning_enabled || !toFAcceptable(sample.timestamp_us)) return;
        const float observed = (sample.altitude_m - zero_m_) - tof_.height_m;
        const float error    = observed - offset_m_;
        if (learned_ && std::fabs(error) > config_.learn_gate_m) return;
        ++learn_count_;
        const float running_mean_gain = 1.0f / static_cast<float>(learn_count_);
        const float time_gain         = step_s / config_.learn_tau_s;
        const float gain = running_mean_gain > time_gain ? running_mean_gain : time_gain;
        offset_m_ += (gain > 1.0f ? 1.0f : gain) * error;
        learned_       = true;
        last_learn_us_ = sample.timestamp_us;
    }

    BarometricHeightConfig config_;
    BarometricToFSample    tof_{};
    bool     has_tof_           = false;
    bool     has_last_          = false;
    bool     was_armed_         = false;
    bool     zero_set_          = false;
    bool     learned_           = false;
    float    last_pressure_pa_  = 0.0f;
    float    last_altitude_m_   = 0.0f;
    uint32_t last_timestamp_us_ = 0;
    uint32_t last_learn_us_     = 0;
    uint32_t learn_count_       = 0;
    float    zero_m_            = 0.0f;
    float    offset_m_          = 0.0f;
};

}  // namespace sf
