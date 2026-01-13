/**
 * @file landing_handler.hpp
 * @brief Landing detection and calibration handler
 *
 * Integrates landing detection, stationary detection, and level calibration.
 * Manages the calibration state machine and provides arm permission control.
 *
 * 着陸検出・キャリブレーション統合ハンドラ。
 * 着陸検出、静止検出、姿勢基準キャリブレーションを統合管理。
 * Arm許可制御を提供。
 */

#pragma once

#include "stationary_detector.hpp"
#include "level_calibrator.hpp"
#include "stampfly_math.hpp"

namespace stampfly {

class LandingHandler {
public:
    /**
     * @brief Calibration state
     */
    enum class CalibrationState {
        NOT_STARTED,      // Initial state or after arm
        WAITING_LANDING,  // Waiting for landing (flying)
        CALIBRATING,      // Calibration in progress (white LED, arm blocked)
        COMPLETED         // Calibration done (green LED, arm allowed)
    };

    struct Config {
        float landing_altitude_threshold;
        int landing_hold_samples;

        Config()
            : landing_altitude_threshold(0.05f)  // [m] - altitude below this = landed
            , landing_hold_samples(80)           // 200ms @ 400Hz
        {}
    };

    /**
     * @brief Initialize handler
     */
    void init(const Config& config = Config()) {
        config_ = config;
        stationary_detector_.init();
        level_calibrator_.reset();
        reset();
    }

    /**
     * @brief Reset to initial state (call on arm)
     */
    void reset() {
        calibration_state_ = CalibrationState::NOT_STARTED;
        is_landed_ = false;
        landing_count_ = 0;
        just_landed_ = false;
        just_calibrated_ = false;
        stationary_detector_.reset();
    }

    /**
     * @brief Update handler state
     * @param is_disarmed True if vehicle is disarmed
     * @param tof_altitude ToF altitude reading [m]
     * @param gyro Gyroscope reading [rad/s]
     * @param accel Accelerometer reading [m/s²]
     */
    void update(bool is_disarmed, float tof_altitude,
                const math::Vector3& gyro, const math::Vector3& accel) {
        just_landed_ = false;
        just_calibrated_ = false;

        // If armed, reset state and wait for landing
        if (!is_disarmed) {
            if (calibration_state_ == CalibrationState::COMPLETED ||
                calibration_state_ == CalibrationState::CALIBRATING) {
                calibration_state_ = CalibrationState::WAITING_LANDING;
            }
            is_landed_ = false;
            landing_count_ = 0;
            stationary_detector_.reset();
            return;
        }

        // Disarmed - check for landing
        bool altitude_low = (tof_altitude < config_.landing_altitude_threshold);

        if (altitude_low) {
            landing_count_++;
            if (landing_count_ >= config_.landing_hold_samples && !is_landed_) {
                // Just landed
                is_landed_ = true;
                just_landed_ = true;

                // Start calibration if not already completed
                if (calibration_state_ != CalibrationState::COMPLETED) {
                    calibration_state_ = CalibrationState::CALIBRATING;
                    stationary_detector_.reset();
                }
            }
        } else {
            landing_count_ = 0;
            // If altitude increases while disarmed (picked up), restart calibration
            if (is_landed_) {
                is_landed_ = false;
                if (calibration_state_ == CalibrationState::COMPLETED) {
                    // Was calibrated but now lifted - need recalibration when put down
                    calibration_state_ = CalibrationState::NOT_STARTED;
                }
            }
        }

        // If landed and calibrating, run stationary detection
        if (is_landed_ && calibration_state_ == CalibrationState::CALIBRATING) {
            stationary_detector_.update(gyro, accel);

            if (stationary_detector_.isStationary()) {
                // Calibration complete
                gyro_bias_ = stationary_detector_.getGyroAverage();
                accel_reference_ = stationary_detector_.getAccelAverage();
                level_calibrator_.setReference(accel_reference_);

                calibration_state_ = CalibrationState::COMPLETED;
                just_calibrated_ = true;
            }
        }
    }

    // === State queries ===

    /**
     * @brief Check if arm is allowed
     */
    bool canArm() const {
        return calibration_state_ == CalibrationState::COMPLETED;
    }

    /**
     * @brief Check if calibration is in progress
     */
    bool isCalibrating() const {
        return calibration_state_ == CalibrationState::CALIBRATING;
    }

    /**
     * @brief Check if vehicle is landed
     */
    bool isLanded() const { return is_landed_; }

    /**
     * @brief Get calibration state
     */
    CalibrationState getCalibrationState() const { return calibration_state_; }

    // === Event flags (single-shot, cleared each update) ===

    /**
     * @brief Check if just landed this frame
     */
    bool justLanded() const { return just_landed_; }

    /**
     * @brief Check if calibration just completed this frame
     */
    bool justCalibrated() const { return just_calibrated_; }

    // === Calibration results ===

    /**
     * @brief Get gyro bias from calibration
     */
    math::Vector3 getGyroBias() const { return gyro_bias_; }

    /**
     * @brief Get acceleration reference from calibration
     */
    math::Vector3 getAccelReference() const { return accel_reference_; }

    /**
     * @brief Get level calibrator (for attitude offset computation)
     */
    const LevelCalibrator& getLevelCalibrator() const { return level_calibrator_; }

private:
    Config config_;
    StationaryDetector stationary_detector_;
    LevelCalibrator level_calibrator_;

    CalibrationState calibration_state_ = CalibrationState::NOT_STARTED;
    bool is_landed_ = false;
    int landing_count_ = 0;

    // Event flags
    bool just_landed_ = false;
    bool just_calibrated_ = false;

    // Calibration results
    math::Vector3 gyro_bias_;
    math::Vector3 accel_reference_;
};

}  // namespace stampfly
