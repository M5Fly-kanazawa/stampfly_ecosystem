/**
 * @file main.cpp
 * @brief PMW3901 Optical Flow Sensor C++ Example for StampFly
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pmw3901_wrapper.hpp"

static const char *TAG = "PMW3901_CPP_EXAMPLE";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== PMW3901 C++ Wrapper Example ===");

    try {
        // Create sensor with default StampFly configuration (RAII)
        stampfly::PMW3901 sensor;

        // Get sensor information
        uint8_t product_id = sensor.getProductId();
        uint8_t revision_id = sensor.getRevisionId();
        ESP_LOGI(TAG, "Sensor OK - Product ID: 0x%02X, Revision ID: 0x%02X",
                 product_id, revision_id);

        // Simulated altitude for velocity calculation
        // In real use, get this from an altitude sensor (e.g., VL53L3, BMP280)
        float altitude = 1.0f;  // 1 meter above ground
        float interval = 0.05f; // 20Hz = 50ms

        int loop_count = 0;

        ESP_LOGI(TAG, "Starting motion detection loop...");
        ESP_LOGI(TAG, "Waiting for motion...");

        // Teleplot: Output header comment
        printf("# Teleplot streaming enabled (C++ example)\n");
        printf("# Format: >variable_name:value\n");

        while (true) {
            // Use burst read for efficient data retrieval
            auto burst = sensor.readMotionBurst();

            // Calculate velocity using StampFly method
            auto velocity = sensor.calculateVelocityDirect(
                burst.delta_x, burst.delta_y, altitude, interval);

            // Teleplot streaming output (every sample for smooth graphs)
            printf(">delta_x:%d\n", burst.delta_x);
            printf(">delta_y:%d\n", burst.delta_y);
            printf(">velocity_x:%.3f\n", velocity.x);
            printf(">velocity_y:%.3f\n", velocity.y);
            printf(">squal:%d\n", burst.squal);
            printf(">shutter:%d\n", burst.shutter);
            printf(">raw_sum:%d\n", burst.raw_data_sum);

            // Detailed output every 10 samples (0.5 seconds)
            if (loop_count % 10 == 0) {
                ESP_LOGI(TAG,
                    "RAW: MOT=0x%02X OBS=0x%02X dX=%4d dY=%4d SQUAL=%3d "
                    "RawSum=%3d Max=%3d Min=%3d Shutter=%4d",
                    burst.motion, burst.observation,
                    burst.delta_x, burst.delta_y,
                    burst.squal, burst.raw_data_sum,
                    burst.max_raw_data, burst.min_raw_data,
                    burst.shutter);
            }

            // Check if there's any motion
            if (burst.delta_x != 0 || burst.delta_y != 0) {
                ESP_LOGI(TAG,
                    "MOTION: dX=%4d dY=%4d | Vel: X=%+.3f Y=%+.3f m/s | SQUAL=%d",
                    burst.delta_x, burst.delta_y,
                    velocity.x, velocity.y, burst.squal);
            }

            loop_count++;
            vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz sampling rate
        }

        // Destructor automatically called when sensor goes out of scope
        // (never reached in this example due to infinite loop)

    } catch (const stampfly::PMW3901Exception& e) {
        // Handle PMW3901-specific exceptions
        ESP_LOGE(TAG, "PMW3901 Error: %s", e.what());
        ESP_LOGE(TAG, "Error Code: %s",
                 stampfly::PMW3901Exception::errorCodeToString(e.code()));
        ESP_LOGE(TAG, "ESP Error: %s", esp_err_to_name(e.espError()));

    } catch (const std::exception& e) {
        // Handle other standard exceptions
        ESP_LOGE(TAG, "Unexpected error: %s", e.what());
    }

    ESP_LOGE(TAG, "Sensor initialization or operation failed. Halting.");
}
