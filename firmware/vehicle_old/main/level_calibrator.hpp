/**
 * @file level_calibrator.hpp
 * @brief Level calibration for attitude reference
 *
 * Sets the attitude reference based on accelerometer readings when
 * the vehicle is stationary on a level surface. The reference acceleration
 * vector defines the "zero attitude" orientation.
 *
 * 姿勢基準キャリブレーション。静止時の加速度ベクトルを基準として
 * 「姿勢ゼロ」を定義する。
 */

#pragma once

#include "stampfly_math.hpp"
#include <cmath>

namespace stampfly {

class LevelCalibrator {
public:
    /**
     * @brief Set reference acceleration vector (when stationary and level)
     * @param accel_avg Averaged accelerometer reading [m/s²]
     *
     * The reference vector is normalized and stored. This defines
     * the "down" direction for attitude calculation.
     */
    void setReference(const math::Vector3& accel_avg) {
        // Normalize to get direction only
        reference_accel_ = accel_avg.normalized();
        has_reference_ = true;
    }

    /**
     * @brief Check if reference has been set
     */
    bool hasReference() const { return has_reference_; }

    /**
     * @brief Get the reference acceleration vector (normalized)
     */
    math::Vector3 getReference() const { return reference_accel_; }

    /**
     * @brief Compute rotation from reference to current acceleration
     * @param accel_current Current accelerometer reading [m/s²]
     * @return Quaternion representing the rotation (attitude offset)
     *
     * This computes the rotation that transforms the reference vector
     * to the current acceleration vector. This rotation represents
     * the current attitude relative to the calibrated "level" position.
     */
    math::Quaternion computeRotationFromReference(const math::Vector3& accel_current) const {
        if (!has_reference_) {
            return math::Quaternion::identity();
        }

        // Normalize current acceleration
        math::Vector3 current = accel_current.normalized();

        // Compute rotation from reference to current using Rodrigues' formula
        // q = cos(theta/2) + sin(theta/2) * axis

        // Cross product gives rotation axis
        math::Vector3 axis = reference_accel_.cross(current);
        float axis_norm = axis.norm();

        // Dot product gives cos(theta)
        float cos_theta = reference_accel_.dot(current);

        // Handle nearly parallel vectors
        if (axis_norm < 1e-6f) {
            if (cos_theta > 0) {
                // Same direction - no rotation
                return math::Quaternion::identity();
            } else {
                // Opposite direction - 180 degree rotation around any perpendicular axis
                // Find a perpendicular axis
                math::Vector3 perp;
                if (std::abs(reference_accel_.x) < 0.9f) {
                    perp = reference_accel_.cross(math::Vector3::unitX());
                } else {
                    perp = reference_accel_.cross(math::Vector3::unitY());
                }
                perp.normalize();
                return math::Quaternion(0, perp.x, perp.y, perp.z);
            }
        }

        // Normalize axis
        axis = axis * (1.0f / axis_norm);

        // Compute half angle
        // cos(theta) = dot, sin(theta) = axis_norm (before normalization)
        // Using half-angle formulas:
        // cos(theta/2) = sqrt((1 + cos(theta)) / 2)
        // sin(theta/2) = sqrt((1 - cos(theta)) / 2)

        float half_cos = std::sqrt((1.0f + cos_theta) * 0.5f);
        float half_sin = std::sqrt((1.0f - cos_theta) * 0.5f);

        return math::Quaternion(half_cos, half_sin * axis.x, half_sin * axis.y, half_sin * axis.z);
    }

    /**
     * @brief Reset calibration
     */
    void reset() {
        has_reference_ = false;
        reference_accel_ = math::Vector3::zero();
    }

private:
    math::Vector3 reference_accel_;  // Normalized reference "down" vector
    bool has_reference_ = false;
};

}  // namespace stampfly
