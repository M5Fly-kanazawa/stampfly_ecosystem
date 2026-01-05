/**
 * @file sensor_health.hpp
 * @brief センサーヘルスチェック - 連続成功/失敗による健全性判定
 *
 * 使用例:
 * @code
 * sf::SensorHealth health;
 * health.setThresholds(10, 10);  // 10回連続で状態変化
 *
 * // センサー読み取りループ内
 * if (sensor_read_ok) {
 *     health.recordSuccess();
 * } else {
 *     health.recordFailure();
 * }
 *
 * if (health.isHealthy()) {
 *     // センサーデータを使用可能
 * }
 * @endcode
 */

#pragma once

#include <cstdint>

namespace sf {

/**
 * @brief 単一センサーの健全性管理
 *
 * 連続成功/失敗カウントに基づいてhealthy/unhealthy状態を管理
 */
class SensorHealth {
public:
    SensorHealth() = default;

    /**
     * @brief 閾値設定
     * @param healthy_threshold   healthy判定に必要な連続成功回数
     * @param unhealthy_threshold unhealthy判定に必要な連続失敗回数
     */
    void setThresholds(int healthy_threshold, int unhealthy_threshold) {
        healthy_threshold_ = healthy_threshold;
        unhealthy_threshold_ = unhealthy_threshold;
    }

    /**
     * @brief 成功を記録
     * 連続成功がhealthy_thresholdに達するとhealthy=trueになる
     */
    void recordSuccess() {
        consecutive_failure_ = 0;
        if (++consecutive_success_ >= healthy_threshold_) {
            healthy_ = true;
        }
    }

    /**
     * @brief 失敗を記録
     * 連続失敗がunhealthy_thresholdに達するとhealthy=falseになる
     */
    void recordFailure() {
        consecutive_success_ = 0;
        if (++consecutive_failure_ >= unhealthy_threshold_) {
            healthy_ = false;
        }
    }

    /**
     * @brief 健全性を取得
     */
    bool isHealthy() const { return healthy_; }

    /**
     * @brief リセット
     */
    void reset() {
        consecutive_success_ = 0;
        consecutive_failure_ = 0;
        healthy_ = false;
    }

    // デバッグ用アクセサ
    int getConsecutiveSuccess() const { return consecutive_success_; }
    int getConsecutiveFailure() const { return consecutive_failure_; }

private:
    int consecutive_success_ = 0;
    int consecutive_failure_ = 0;
    int healthy_threshold_ = 10;
    int unhealthy_threshold_ = 10;
    bool healthy_ = false;
};

/**
 * @brief 全センサーの健全性を一元管理
 */
class HealthMonitor {
public:
    // 各センサーのヘルス状態
    SensorHealth imu;
    SensorHealth mag;
    SensorHealth baro;
    SensorHealth tof;
    SensorHealth optflow;

    /**
     * @brief 健全性ビットマスク取得
     * @return bit0:IMU, bit1:Mag, bit2:Baro, bit3:ToF, bit4:OptFlow
     */
    uint32_t getHealthyMask() const {
        uint32_t mask = 0;
        if (imu.isHealthy())     mask |= (1 << 0);
        if (mag.isHealthy())     mask |= (1 << 1);
        if (baro.isHealthy())    mask |= (1 << 2);
        if (tof.isHealthy())     mask |= (1 << 3);
        if (optflow.isHealthy()) mask |= (1 << 4);
        return mask;
    }

    /**
     * @brief 必須センサーが全て健全か
     * @return IMU + ToF + OptFlow が全て healthy なら true
     */
    bool isFlightReady() const {
        return imu.isHealthy() && tof.isHealthy() && optflow.isHealthy();
    }

    /**
     * @brief 全センサーリセット
     */
    void reset() {
        imu.reset();
        mag.reset();
        baro.reset();
        tof.reset();
        optflow.reset();
    }
};

} // namespace sf
