/**
 * @file landing_handler.hpp
 * @brief Landing detection and calibration handler
 *
 * Single source of truth for landing state.
 * imu_task queries isLanded()/hasTakenOff() instead of maintaining its own state.
 *
 * 着陸状態の唯一の管理者。
 * imu_task は独自判定を持たず、isLanded()/hasTakenOff() を参照する。
 *
 * Safety guarantees:
 * 安全保証:
 * - isLanded() is NEVER true while armed
 *   armed 中に isLanded() が true を返すことは決してない
 * - justCalibrated() is NEVER true while armed
 *   armed 中に justCalibrated() が true を返すことは決してない
 * - ToF buffer empty (tof=0) is NOT treated as valid low altitude
 *   ToF バッファ空は有効な低高度として扱わない
 */

#pragma once

#include "stationary_detector.hpp"
#include "level_calibrator.hpp"
#include "stampfly_math.hpp"
#include "system_state.hpp"
#include "esp_log.h"

namespace stampfly {

class LandingHandler {
public:
    struct Config {
        float landing_altitude_threshold;  // [m] altitude below this = landed
        float takeoff_altitude_threshold;  // [m] altitude above this = taken off
        int landing_hold_samples;          // Consecutive samples required for landing
        int takeoff_hold_samples;          // Consecutive samples required for takeoff
        int tof_timeout_samples;           // ToF timeout for fallback (disarmed only)
        bool tof_available;                // Whether ToF sensor is enabled

        Config()
            : landing_altitude_threshold(0.05f)  // [m]
            , takeoff_altitude_threshold(0.10f)  // [m] hysteresis upper bound
            , landing_hold_samples(80)           // 200ms @ 400Hz
            , takeoff_hold_samples(80)           // 200ms @ 400Hz
            , tof_timeout_samples(800)           // 2s @ 400Hz
            , tof_available(true)
        {}
    };

    void init(const Config& config = Config()) {
        config_ = config;
        stationary_detector_.init();
        level_calibrator_.reset();
        reset();
        // Boot: start as landed + calibrated (main.cpp Phase 1-2 handles initial cal)
        // 起動時: 着陸・キャリブレーション済みで開始
        is_landed_ = true;
        has_taken_off_ = false;
        calibration_state_ = CalibrationState::COMPLETED;
        syncToSystemStateManager();
    }

    /**
     * @brief Reset to initial state (call on arm)
     * ARM時に呼び出し — 全状態をリセット
     */
    void reset() {
        calibration_state_ = CalibrationState::NOT_STARTED;
        is_landed_ = false;
        has_taken_off_ = false;
        landing_count_ = 0;
        takeoff_count_ = 0;
        tof_high_count_ = 0;
        just_landed_ = false;
        just_calibrated_ = false;
        just_taken_off_ = false;
        stationary_detector_.reset();
        syncToSystemStateManager();
    }

    /**
     * @brief Update handler state
     *
     * @param is_disarmed True if vehicle is disarmed
     * @param tof_altitude ToF altitude reading [m] (negative or NaN = invalid)
     * @param tof_valid True if ToF buffer has valid data
     * @param gyro Gyroscope reading [rad/s]
     * @param accel Accelerometer reading [m/s²]
     */
    void update(bool is_disarmed, float tof_altitude, bool tof_valid,
                const math::Vector3& gyro, const math::Vector3& accel) {
        just_landed_ = false;
        just_calibrated_ = false;
        just_taken_off_ = false;

        // === Armed: landing detection disabled, track takeoff only ===
        // === Armed: 着陸検出無効、離陸追跡のみ ===
        if (!is_disarmed) {
            // Clear landing state — armed 中は決して着陸状態にならない
            // Clear landing state — never landed while armed
            is_landed_ = false;
            landing_count_ = 0;
            tof_high_count_ = 0;

            if (calibration_state_ == CalibrationState::COMPLETED ||
                calibration_state_ == CalibrationState::CALIBRATING) {
                calibration_state_ = CalibrationState::WAITING_LANDING;
            }
            stationary_detector_.reset();

            // Takeoff detection (armed only)
            // 離陸検出 (armed 時のみ)
            if (!has_taken_off_ && tof_valid && config_.tof_available) {
                if (tof_altitude > config_.takeoff_altitude_threshold) {
                    takeoff_count_++;
                    if (takeoff_count_ >= config_.takeoff_hold_samples) {
                        has_taken_off_ = true;
                        just_taken_off_ = true;
                        ESP_LOGI("LandingHandler", "Takeoff detected (alt=%.3fm)", tof_altitude);
                    }
                } else {
                    takeoff_count_ = 0;
                }
            } else if (!has_taken_off_ && !config_.tof_available) {
                // Without ToF, assume taken off immediately when armed
                // ToF なしの場合、armed 時点で離陸とみなす
                has_taken_off_ = true;
                just_taken_off_ = true;
            }

            syncToSystemStateManager();
            return;
        }

        // === Disarmed: landing detection and calibration ===
        // === Disarmed: 着陸検出とキャリブレーション ===

        // ToF が無効な場合、高度ベースの判定をスキップ
        // Skip altitude-based detection if ToF is invalid
        if (!tof_valid || !config_.tof_available) {
            // Without valid ToF, use timeout fallback (disarmed only)
            // ToF 無効時はタイムアウトフォールバック (disarmed 時のみ)
            tof_high_count_++;

            if (tof_high_count_ >= config_.tof_timeout_samples && !is_landed_) {
                is_landed_ = true;
                just_landed_ = true;
                if (calibration_state_ != CalibrationState::COMPLETED) {
                    calibration_state_ = CalibrationState::CALIBRATING;
                    stationary_detector_.reset();
                    ESP_LOGW("LandingHandler",
                             "No valid ToF data - assuming landed after timeout");
                }
            }
        } else {
            // Valid ToF data available
            // 有効な ToF データあり
            bool altitude_low = (tof_altitude < config_.landing_altitude_threshold);

            if (altitude_low) {
                landing_count_++;
                tof_high_count_ = 0;
                if (landing_count_ >= config_.landing_hold_samples && !is_landed_) {
                    is_landed_ = true;
                    just_landed_ = true;
                    if (calibration_state_ != CalibrationState::COMPLETED) {
                        calibration_state_ = CalibrationState::CALIBRATING;
                        stationary_detector_.reset();
                    }
                }
            } else {
                landing_count_ = 0;
                tof_high_count_++;

                if (tof_high_count_ >= config_.tof_timeout_samples && !is_landed_) {
                    is_landed_ = true;
                    just_landed_ = true;
                    if (calibration_state_ != CalibrationState::COMPLETED) {
                        calibration_state_ = CalibrationState::CALIBRATING;
                        stationary_detector_.reset();
                        ESP_LOGW("LandingHandler",
                                 "ToF timeout (%d samples) - assuming landed",
                                 config_.tof_timeout_samples);
                    }
                }

                if (is_landed_ && tof_high_count_ < config_.tof_timeout_samples) {
                    // Picked up after landing
                    // 着陸後に持ち上げられた
                    is_landed_ = false;
                    if (calibration_state_ == CalibrationState::COMPLETED) {
                        calibration_state_ = CalibrationState::NOT_STARTED;
                    }
                }
            }
        }

        // Stationary detection for calibration (disarmed + landed)
        // 静止検出 (disarmed + 着陸中)
        if (is_landed_ && calibration_state_ == CalibrationState::CALIBRATING) {
            stationary_detector_.update(gyro, accel);

            if (stationary_detector_.isStationary()) {
                gyro_bias_ = stationary_detector_.getGyroAverage();
                accel_reference_ = stationary_detector_.getAccelAverage();
                level_calibrator_.setReference(accel_reference_);

                calibration_state_ = CalibrationState::COMPLETED;
                just_calibrated_ = true;
            }
        }

        syncToSystemStateManager();
    }

    // === State queries ===

    bool canArm() const {
        return calibration_state_ == CalibrationState::COMPLETED;
    }

    bool isCalibrating() const {
        return calibration_state_ == CalibrationState::CALIBRATING;
    }

    /**
     * @brief Check if vehicle is on the ground
     * Guaranteed false while armed.
     * armed 中は必ず false。
     */
    bool isLanded() const { return is_landed_; }

    /**
     * @brief Check if vehicle has taken off at least once since arm
     * 直近の arm 以降に一度でも離陸したか
     */
    bool hasTakenOff() const { return has_taken_off_; }

    CalibrationState getCalibrationState() const { return calibration_state_; }

    // === Event flags (single-shot, cleared each update) ===

    bool justLanded() const { return just_landed_; }
    bool justCalibrated() const { return just_calibrated_; }
    bool justTakenOff() const { return just_taken_off_; }

    // === Calibration results ===

    math::Vector3 getGyroBias() const { return gyro_bias_; }
    math::Vector3 getAccelReference() const { return accel_reference_; }
    const LevelCalibrator& getLevelCalibrator() const { return level_calibrator_; }

private:
    void syncToSystemStateManager() {
        auto& sys_state_mgr = SystemStateManager::getInstance();
        sys_state_mgr.updateCalibrationState(calibration_state_);
        sys_state_mgr.setReady(ReadinessFlags::CALIBRATED,
                               calibration_state_ == CalibrationState::COMPLETED);
        sys_state_mgr.setReady(ReadinessFlags::LANDED, is_landed_);
    }

    Config config_;
    StationaryDetector stationary_detector_;
    LevelCalibrator level_calibrator_;

    CalibrationState calibration_state_ = CalibrationState::NOT_STARTED;
    bool is_landed_ = false;
    bool has_taken_off_ = false;
    int landing_count_ = 0;
    int takeoff_count_ = 0;
    int tof_high_count_ = 0;

    // Event flags (single-shot)
    bool just_landed_ = false;
    bool just_calibrated_ = false;
    bool just_taken_off_ = false;

    // Calibration results
    math::Vector3 gyro_bias_;
    math::Vector3 accel_reference_;
};

}  // namespace stampfly
