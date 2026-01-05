/**
 * @file main.c
 * @brief PMW3901 Optical Flow Sensor Example for StampFly
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pmw3901.h"

static const char *TAG = "PMW3901_EXAMPLE";

void app_main(void)
{
    ESP_LOGI(TAG, "=== PMW3901 Optical Flow Sensor ===");

    // Create device handle and get default configuration
    pmw3901_t dev;
    pmw3901_config_t config;
    pmw3901_get_default_config(&config);

    // Initialize sensor
    esp_err_t ret = pmw3901_init(&dev, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization FAILED!");
        return;
    }

    // Get sensor information
    uint8_t product_id, revision_id;
    pmw3901_get_product_id(&dev, &product_id);
    pmw3901_get_revision_id(&dev, &revision_id);
    ESP_LOGI(TAG, "Sensor OK - ID:0x%02X Rev:0x%02X", product_id, revision_id);
    ESP_LOGI(TAG, "Waiting for motion...");

    // Simulated altitude for velocity calculation (in real use, get from altitude sensor)
    float altitude = 1.0f;  // 1 meter above ground
    float interval = 0.05f; // 20Hz = 50ms (slower for easier motion detection)

    int loop_count = 0;

    // Read SQUAL to check sensor surface quality
    uint8_t squal;
    pmw3901_read_register(&dev, 0x07, &squal);
    ESP_LOGI(TAG, "Initial SQUAL (Surface Quality): %d", squal);

    // Teleplot: Output header comment
    printf("# Teleplot streaming enabled\n");
    printf("# Format: >variable_name:value\n");

    while (1) {
        // Use burst read for more efficient data retrieval
        pmw3901_motion_burst_t burst;
        ret = pmw3901_read_motion_burst(&dev, &burst);

        if (ret == ESP_OK) {
            // Calculate velocity using StampFly method
            float vx, vy;
            pmw3901_calculate_velocity_direct(burst.delta_x, burst.delta_y,
                                              altitude, interval, &vx, &vy);

            // Teleplot streaming output (every sample for smooth graphs)
            printf(">delta_x:%d\n", burst.delta_x);
            printf(">delta_y:%d\n", burst.delta_y);
            printf(">velocity_x:%.3f\n", vx);
            printf(">velocity_y:%.3f\n", vy);
            printf(">squal:%d\n", burst.squal);
            printf(">shutter:%d\n", burst.shutter);
            printf(">raw_sum:%d\n", burst.raw_data_sum);

            // Always show detailed data for debugging (every 10 samples = 0.5 sec)
            if (loop_count % 10 == 0) {
                ESP_LOGI(TAG, "RAW: MOT=0x%02X OBS=0x%02X dX=%4d dY=%4d SQUAL=%3d RawSum=%3d Max=%3d Min=%3d Shutter=%4d",
                         burst.motion, burst.observation,
                         burst.delta_x, burst.delta_y,
                         burst.squal, burst.raw_data_sum,
                         burst.max_raw_data, burst.min_raw_data,
                         burst.shutter);
            }

            // Check if there's any motion
            if (burst.delta_x != 0 || burst.delta_y != 0) {
                // Detailed output
                ESP_LOGI(TAG, "MOTION: dX=%4d dY=%4d | Vel: X=%+.3f Y=%+.3f m/s | SQUAL=%d",
                         burst.delta_x, burst.delta_y, vx, vy, burst.squal);
            }
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(ret));
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz sampling rate
    }

    // Cleanup (never reached in this example)
    pmw3901_deinit(&dev);
}
