/**
 * @file main.cpp
 * @brief Simple ESKF test program
 *
 * Basic test to verify ESKF compiles and runs on PC
 */

#include <cstdio>
#include <cmath>
#include "eskf.hpp"

using namespace stampfly;
using namespace stampfly::math;

int main()
{
    printf("=== ESKF PC Debug Tool ===\n\n");

    // Initialize ESKF with default config
    ESKF eskf;
    ESKF::Config config = ESKF::Config::defaultConfig();

    printf("Initializing ESKF...\n");
    if (eskf.init(config) != ESP_OK) {
        printf("Failed to initialize ESKF\n");
        return 1;
    }

    // Simulate stationary IMU data
    printf("\nSimulating stationary IMU data (100 samples at 100Hz)...\n");

    Vector3 accel(0.0f, 0.0f, 9.81f);  // Gravity pointing up (sensor on flat surface)
    Vector3 gyro(0.0f, 0.0f, 0.0f);    // No rotation
    float dt = 0.01f;  // 100Hz

    for (int i = 0; i < 100; i++) {
        eskf.predict(accel, gyro, dt);

        if (i % 10 == 0) {
            auto state = eskf.getState();
            printf("  t=%4dms: roll=%.2f° pitch=%.2f° yaw=%.2f° alt=%.3fm\n",
                   (i + 1) * 10,
                   state.roll * 180.0f / M_PI,
                   state.pitch * 180.0f / M_PI,
                   state.yaw * 180.0f / M_PI,
                   state.position.z);
        }
    }

    // Test measurement updates
    printf("\nTesting measurement updates...\n");

    // Baro update
    eskf.updateBaro(1.0f);  // 1m altitude
    auto state = eskf.getState();
    printf("After Baro update (1m): alt=%.3fm\n", state.position.z);

    // ToF update
    eskf.updateToF(0.5f);  // 0.5m from ground
    state = eskf.getState();
    printf("After ToF update (0.5m): alt=%.3fm\n", state.position.z);

    // Mag update
    Vector3 mag(20.0f, 0.0f, 40.0f);  // Reference mag field
    eskf.updateMag(mag);
    state = eskf.getState();
    printf("After Mag update: yaw=%.2f°\n", state.yaw * 180.0f / M_PI);

    // Final state
    printf("\n=== Final State ===\n");
    printf("Position: [%.3f, %.3f, %.3f] m\n",
           state.position.x, state.position.y, state.position.z);
    printf("Velocity: [%.3f, %.3f, %.3f] m/s\n",
           state.velocity.x, state.velocity.y, state.velocity.z);
    printf("Attitude: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           state.roll * 180.0f / M_PI,
           state.pitch * 180.0f / M_PI,
           state.yaw * 180.0f / M_PI);
    printf("Gyro bias: [%.6f, %.6f, %.6f] rad/s\n",
           state.gyro_bias.x, state.gyro_bias.y, state.gyro_bias.z);
    printf("Accel bias: [%.6f, %.6f, %.6f] m/s²\n",
           state.accel_bias.x, state.accel_bias.y, state.accel_bias.z);

    printf("\nESKF test completed successfully!\n");

    return 0;
}
