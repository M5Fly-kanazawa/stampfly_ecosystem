/**
 * @file main.cpp
 * @brief Example 05 — Read distance with ToF sensor (VL53L3CX)
 *        サンプル05 — ToFセンサ（VL53L3CX）で距離を測る
 *
 * Initializes the bottom VL53L3CX Time-of-Flight sensor via I2C,
 * starts continuous ranging, and prints the distance in mm to
 * the serial console.
 *
 * I2C経由で底面のVL53L3CX ToFセンサを初期化し、連続測距を開始して、
 * 距離（mm）をシリアルコンソールに表示します。
 *
 * Hardware: StampFly — VL53L3CX on I2C (SDA=GPIO3, SCL=GPIO4)
 *           XSHUT(bottom)=GPIO7
 * ハードウェア: StampFly — I2CにVL53L3CX (SDA=GPIO3, SCL=GPIO4)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "vl53l3cx_wrapper.hpp"

// I2C bus configuration
// I2Cバス設定
static constexpr gpio_num_t GPIO_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t GPIO_I2C_SCL = GPIO_NUM_4;

static const char* TAG = "read_tof";

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

    // Initialize VL53L3CX (bottom sensor)
    // VL53L3CX（底面センサ）を初期化
    stampfly::VL53L3CXWrapper tof;
    auto config = stampfly::VL53L3CXWrapper::Config::defaultBottom(i2c_bus);

    ESP_LOGI(TAG, "Initializing VL53L3CX (bottom sensor)...");
    esp_err_t ret = tof.init(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L3CX init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Start continuous ranging
    // 連続測距を開始
    ESP_ERROR_CHECK(tof.startRanging());
    ESP_LOGI(TAG, "Ranging started. Hold objects under the sensor.");

    while (true) {
        // Check if new data is ready
        // 新しいデータが準備できたかチェック
        bool ready = false;
        tof.isDataReady(ready);

        if (ready) {
            // Read distance measurement
            // 距離の計測結果を読み取る
            stampfly::DistanceData data;
            ret = tof.getDistance(data);

            if (ret == ESP_OK && data.range_status == 0) {
                printf("Distance: %4d mm  (sigma: %.1f mm)\n",
                       data.distance_mm, data.sigma_mm);
            } else if (ret == ESP_OK) {
                printf("Distance: ---- mm  (status: %d)\n",
                       data.range_status);
            }

            // Clear interrupt and prepare for next measurement
            // 割り込みをクリアして次の測定を準備
            tof.clearInterruptAndStartMeasurement();
        }

        // Poll at 30 Hz
        // 30Hzでポーリング
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Change the distance mode to LONG for up to 4m range
//    距離モードをLONGに変えて4mまで測れるようにしてみよう
//    tof.setDistanceMode(stampfly::VL53L3CXWrapper::DistanceMode::LONG);
//
// 2. Add an LED indicator: green when close, red when far
//    LED表示を追加: 近い→緑、遠い→赤
//
// 3. Try using the front sensor (defaultFront) instead
//    前方センサ（defaultFront）に切り替えてみよう
//
// 4. Print the signal rate to see measurement quality
//    信号強度を表示して計測品質を確認してみよう
// ============================================================
