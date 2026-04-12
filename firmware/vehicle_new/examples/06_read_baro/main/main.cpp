/**
 * @file main.cpp
 * @brief Example 06 — Read barometric pressure and calculate altitude
 *        サンプル06 — 気圧を読んで高度を計算する
 *
 * Initializes the BMP280 barometric pressure sensor via I2C and
 * displays pressure (hPa), temperature (C), and estimated altitude (m)
 * at 5 Hz.
 *
 * BMP280気圧センサをI2C経由で初期化し、気圧（hPa）、温度（C）、
 * 推定高度（m）を5Hzで表示します。
 *
 * Hardware: StampFly — BMP280 on I2C (SDA=GPIO3, SCL=GPIO4)
 * ハードウェア: StampFly — I2CにBMP280 (SDA=GPIO3, SCL=GPIO4)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "bmp280.hpp"

// I2C bus configuration
// I2Cバス設定
static constexpr gpio_num_t GPIO_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t GPIO_I2C_SCL = GPIO_NUM_4;

static const char* TAG = "read_baro";

extern "C" void app_main(void)
{
    // Initialize I2C master bus
    // I2Cマスターバスを初期化
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port   = I2C_NUM_0;
    bus_config.sda_io_num = GPIO_I2C_SDA;
    bus_config.scl_io_num = GPIO_I2C_SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    // Initialize BMP280
    // BMP280を初期化
    stampfly::BMP280 baro;
    stampfly::BMP280::Config config;
    config.i2c_bus = i2c_bus;
    config.i2c_addr = stampfly::BMP280_I2C_ADDR_DEFAULT;  // 0x76

    ESP_LOGI(TAG, "Initializing BMP280...");
    esp_err_t ret = baro.init(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check I2C wiring: SDA=3, SCL=4, addr=0x76");
        return;
    }
    ESP_LOGI(TAG, "BMP280 initialized successfully!");

    // Print header
    // ヘッダーを表示
    printf("\n");
    printf("%-12s  %-10s  %-10s\n",
           "Press(hPa)", "Temp(C)", "Alt(m)");
    printf("--------------------------------------\n");

    while (true) {
        // Read barometer data (pressure, temperature, altitude)
        // 気圧データを読み取る（気圧、温度、高度）
        stampfly::BaroData data;
        ret = baro.read(data);

        if (ret == ESP_OK) {
            // Convert Pa to hPa for display
            // 表示用にPaをhPaに変換
            float pressure_hpa = data.pressure_pa / 100.0f;

            printf("%10.2f    %7.2f     %+7.2f\n",
                   pressure_hpa, data.temperature_c, data.altitude_m);
        } else {
            ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(ret));
        }

        // Read at 5 Hz (200 ms interval)
        // 5Hzで読み取り（200ms間隔）
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change the sea level pressure for your location
//    あなたの場所の海面気圧に変えてみよう
//    baro.setSeaLevelPressure(101500.0f);  // Pa
//
// 2. Try different oversampling settings for accuracy vs speed
//    オーバーサンプリングを変えて精度と速度のトレードオフを試そう
//
// 3. Move the sensor up/down and watch altitude change
//    センサを上下に動かして高度変化を観察してみよう
//
// 4. Calculate the altitude difference from a reference point
//    基準点からの高度差を計算してみよう（起動時の高度を0mとする）
// ============================================================
