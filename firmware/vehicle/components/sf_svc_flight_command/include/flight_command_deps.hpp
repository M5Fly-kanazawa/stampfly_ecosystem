/**
 * @file flight_command_deps.hpp
 * @brief Dependency interfaces for FlightCommandService
 *        FlightCommandService の依存インターフェース
 *
 * These interfaces decouple FlightCommandService from vehicle/main globals,
 * allowing the component to be used in any application.
 * これらのインターフェースにより FlightCommandService が vehicle/main の
 * グローバル変数から独立し、どのアプリからでも使用可能になる。
 */

#pragma once

namespace stampfly {

/**
 * @brief Interface for flight readiness checks
 *        飛行準備チェック用インターフェース
 *
 * Abstracts landing detection and arm safety checks.
 * 着陸検出と ARM 安全チェックを抽象化。
 */
class IFlightReadiness {
public:
    virtual ~IFlightReadiness() = default;

    /// Check if vehicle is currently landed / 機体が着陸中か
    virtual bool isLanded() const = 0;

    /// Check if vehicle can be armed / ARM 可能か
    virtual bool canArm() const = 0;
};

/**
 * @brief Interface for state estimation access
 *        状態推定アクセス用インターフェース
 *
 * Provides altitude, yaw, position, and ToF data.
 * 高度・ヨー・位置・ToFデータを提供。
 */
class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;

    /// Get altitude in meters (positive up) / 高度 [m]（正が上）
    virtual float getAltitude() const = 0;

    /// Get yaw angle in radians / ヨー角 [rad]
    virtual float getYaw() const = 0;

    /// Get position (NED X, Y) / 位置 (NED X, Y)
    virtual void getPositionXY(float& x, float& y) const = 0;

    /// Get latest ToF bottom altitude, returns false if unavailable
    /// 最新の ToF 底面高度を取得、利用不可なら false
    virtual bool getToFAltitude(float& altitude) const = 0;
};

/**
 * @brief Interface for position control
 *        位置制御用インターフェース
 *
 * Abstracts the position controller setpoint management.
 * 位置制御器のセットポイント管理を抽象化。
 */
class IPositionControl {
public:
    virtual ~IPositionControl() = default;

    /// Capture current position as reference / 現在位置を基準として取得
    virtual void capturePosition(float x, float y) = 0;

    /// Set position setpoint / 位置セットポイントを設定
    virtual void setPositionSetpoint(float x, float y) = 0;
};

}  // namespace stampfly
