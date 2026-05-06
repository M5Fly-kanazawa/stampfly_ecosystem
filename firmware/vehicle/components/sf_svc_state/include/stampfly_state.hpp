/**
 * @file stampfly_state.hpp
 * @brief StampFly State Manager
 *
 * Thread-safe, sensor data management, state transitions
 */

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "filter.hpp"

namespace stampfly {

enum class FlightState {
    INIT,
    CALIBRATING,
    IDLE,
    ARMED,
    FLYING,
    LANDING,
    ERROR
};

/**
 * @brief Flight control mode
 *
 * ACRO: Direct rate control (angular velocity)
 * STABILIZE: Cascade control (attitude → rate)
 */
enum class FlightMode {
    ACRO,           // Rate control - アクロモード（角速度制御）
    STABILIZE,      // Angle control - スタビライズモード（角度制御）
    ALTITUDE_HOLD,  // Angle + altitude control - 高度維持モード
    POSITION_HOLD   // Angle + altitude + position control - 位置保持モード
};

enum class PairingState {
    NOT_PAIRED,
    PAIRING,
    PAIRED
};

enum class ErrorCode {
    NONE,
    SENSOR_IMU,
    SENSOR_MAG,
    SENSOR_BARO,
    SENSOR_TOF,
    SENSOR_FLOW,
    SENSOR_POWER,
    LOW_BATTERY,
    COMM_LOST,
    CALIBRATION_FAILED
};

enum class ToFPosition {
    BOTTOM,
    FRONT
};

/**
 * @brief Simple 3D vector for state storage
 */
struct StateVector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    StateVector3() = default;
    StateVector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Centralized state manager (Singleton)
 */
class StampFlyState {
public:
    static StampFlyState& getInstance();

    // Delete copy/move
    StampFlyState(const StampFlyState&) = delete;
    StampFlyState& operator=(const StampFlyState&) = delete;

    esp_err_t init();

    // State access
    FlightState getFlightState() const;
    FlightMode getFlightMode() const;
    PairingState getPairingState() const;
    ErrorCode getErrorCode() const;

    // State transitions
    bool requestArm();
    bool requestDisarm();
    void setFlightState(FlightState state);
    void setFlightMode(FlightMode mode);
    void setPairingState(PairingState state);
    void setError(ErrorCode code);
    void clearError();

    // Debug mode (ignores errors for ARM)
    void setDebugMode(bool enable) { debug_mode_ = enable; }
    bool isDebugMode() const { return debug_mode_; }

    // Sensor data access (thread-safe)
    void getIMUData(Vec3& accel, Vec3& gyro) const;        // Raw (filtered, no bias correction)
    void getIMUCorrected(Vec3& accel, Vec3& gyro) const;   // Bias-corrected (for control)
    void getMagData(Vec3& mag) const;
    void getBaroData(float& altitude, float& pressure) const;
    void getToFData(float& bottom, float& front) const;
    void getToFStatus(uint8_t& bottom_status, uint8_t& front_status) const;
    void getFlowData(float& vx, float& vy) const;
    void getFlowRawData(int16_t& dx, int16_t& dy, uint8_t& squal) const;
    void getPowerData(float& voltage, float& current) const;

    // Sensor availability
    bool isFrontToFAvailable() const;
    void setFrontToFAvailable(bool available);

    // Simple getters
    float getAltitude() const;
    float getVoltage() const;
    StateVector3 getVelocity() const;
    StateVector3 getAttitude() const;
    StateVector3 getPosition() const;

    // Sensor data update (thread-safe)
    void updateIMU(const StateVector3& accel, const StateVector3& gyro);
    void updateMag(float x, float y, float z);
    void updateBaro(float pressure, float temperature, float altitude);
    void updateToF(ToFPosition position, float distance, uint8_t status);
    void updateOpticalFlow(int16_t delta_x, int16_t delta_y, uint8_t squal);
    void updatePower(float voltage, float current);

    // Attitude data
    void getAttitudeEuler(float& roll, float& pitch, float& yaw) const;
    void updateAttitude(float roll, float pitch, float yaw);

    // ESKF estimated state
    void updateEstimatedPosition(float x, float y, float z);
    void updateEstimatedVelocity(float vx, float vy, float vz);

    // ESKF bias estimates
    void updateGyroBias(float bx, float by, float bz);
    void updateAccelBias(float bx, float by, float bz);
    StateVector3 getGyroBias() const;
    StateVector3 getAccelBias() const;

    // ESKF status
    void setESKFInitialized(bool initialized);
    bool isESKFInitialized() const;

    // Sensor diagnostics
    // センサ診断情報
    struct SensorDiag {
        bool healthy = false;
        uint32_t last_timestamp_us = 0;  // Last poll timestamp (μs since boot)
        uint32_t loop_count = 0;         // Total loop count
        uint32_t first_timestamp_us = 0; // First timestamp for avg calculation
        uint32_t period_min_us = UINT32_MAX;
        uint32_t period_max_us = 0;
        uint64_t period_sum_us = 0;      // Sum for average
        uint32_t period_count = 0;       // Number of period samples
        uint64_t period_sq_sum = 0;      // Sum of squares for variance
    };
    void updateSensorDiag(const char* name, bool healthy, uint32_t timestamp_us);
    SensorDiag getSensorDiag(const char* name) const;

    // Barometer reference altitude (for binlog)
    void setBaroReferenceAltitude(float altitude);
    float getBaroReferenceAltitude() const;

    // Control input
    void getControlInput(float& throttle, float& roll, float& pitch, float& yaw) const;
    void getRawControlInput(uint16_t& throttle, uint16_t& roll, uint16_t& pitch, uint16_t& yaw) const;
    void updateControlInput(uint16_t throttle, uint16_t roll, uint16_t pitch, uint16_t yaw);
    void updateControlFlags(uint8_t flags);
    uint8_t getControlFlags() const;

    // Control loop reference values (targets computed in control_task)
    // 制御ループ目標値（control_taskで計算）
    void updateControlRef(float angle_ref_roll, float angle_ref_pitch,
                          float rate_ref_roll, float rate_ref_pitch, float rate_ref_yaw);
    void getControlRef(float& angle_ref_roll, float& angle_ref_pitch,
                       float& rate_ref_roll, float& rate_ref_pitch, float& rate_ref_yaw) const;

    /// Set/get actual total thrust command [N] (for telemetry)
    /// 実際の総推力指令 [N]（テレメトリ用）
    void setTotalThrust(float thrust) { total_thrust_ = thrust; }
    float getTotalThrust() const { return total_thrust_; }

    /// Set/get actual motor duties [0-1] (for telemetry)
    /// 実際のモータDuty [0-1]（テレメトリ用）
    void setMotorDuties(const float duties[4]) {
        for (int i = 0; i < 4; i++) motor_duties_[i] = duties[i];
    }
    void getMotorDuties(float duties[4]) const {
        for (int i = 0; i < 4; i++) duties[i] = motor_duties_[i];
    }

private:
    StampFlyState() = default;

    mutable SemaphoreHandle_t mutex_ = nullptr;

    FlightState flight_state_ = FlightState::INIT;
    FlightMode flight_mode_ = FlightMode::ACRO;  // Default: ACRO
    PairingState pairing_state_ = PairingState::NOT_PAIRED;
    ErrorCode error_code_ = ErrorCode::NONE;

    // Sensor data
    StateVector3 accel_ = {};
    StateVector3 gyro_ = {};
    StateVector3 mag_ = {};
    float baro_altitude_ = 0;
    float baro_pressure_ = 101325.0f;
    float baro_temperature_ = 25.0f;
    float tof_bottom_ = 0;
    float tof_front_ = 0;
    uint8_t tof_bottom_status_ = 0;
    uint8_t tof_front_status_ = 0;
    bool front_tof_available_ = false;
    int16_t flow_delta_x_ = 0;
    int16_t flow_delta_y_ = 0;
    uint8_t flow_squal_ = 0;
    float power_voltage_ = 0;
    float power_current_ = 0;

    // Estimated state (from ESKF)
    StateVector3 position_ = {};      // [m] NED
    StateVector3 velocity_ = {};      // [m/s]
    StateVector3 gyro_bias_ = {};     // [rad/s]
    StateVector3 accel_bias_ = {};    // [m/s²]
    bool eskf_initialized_ = false;

    // Barometer reference altitude [m] (set at boot)
    float baro_reference_altitude_ = 0.0f;

    // Attitude
    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;

    // Control input (raw values from controller)
    uint16_t ctrl_throttle_ = 0;
    uint16_t ctrl_roll_ = 2048;   // ADC center
    uint16_t ctrl_pitch_ = 2048;  // ADC center
    uint16_t ctrl_yaw_ = 2048;    // ADC center
    uint8_t ctrl_flags_ = 0;      // ARM, FLIP, MODE, ALT_MODE, POS_MODE

    // Control loop reference values (from control_task)
    // 制御ループ目標値（control_taskから）
    float ctrl_angle_ref_roll_ = 0;   // [rad] outer loop target angle
    float ctrl_angle_ref_pitch_ = 0;  // [rad]
    float ctrl_rate_ref_roll_ = 0;    // [rad/s] inner loop target rate
    float ctrl_rate_ref_pitch_ = 0;   // [rad/s]
    float ctrl_rate_ref_yaw_ = 0;     // [rad/s]
    float total_thrust_ = 0;          // [N] actual commanded total thrust
    float motor_duties_[4] = {};      // [0-1] actual motor duty (FR,RR,RL,FL)

    // Debug mode
    bool debug_mode_ = false;

    // Sensor diagnostics (keyed by short name)
    // センサ診断情報（短縮名でキー付け）
    static constexpr int MAX_SENSOR_DIAGS = 8;
    struct SensorDiagEntry {
        char name[8] = {};
        SensorDiag diag;
    };
    SensorDiagEntry sensor_diags_[MAX_SENSOR_DIAGS] = {};
    int sensor_diag_count_ = 0;
};

}  // namespace stampfly
