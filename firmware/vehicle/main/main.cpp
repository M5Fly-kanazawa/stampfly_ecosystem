/**
 * @file main.cpp
 * @brief StampFly RTOS Skeleton - Main Entry Point
 *
 * This is the main entry point for the StampFly flight controller skeleton.
 * It initializes all subsystems and starts the FreeRTOS tasks.
 */

#include <stdio.h>
#include <cmath>
#include <cstddef>  // for offsetof
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "mag_calibration.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators and peripherals
#include "motor_driver.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// State management and estimation
#include "stampfly_state.hpp"
#include "system_manager.hpp"
#include "eskf.hpp"
#include "sensor_fusion.hpp"
#include "filter.hpp"
#include "sensor_health.hpp"

// Communication
#include "controller_comm.hpp"

// CLI
#include "cli.hpp"

// Logger
#include "logger.hpp"

// Telemetry
#include "telemetry.hpp"

// Configuration
#include "config.hpp"
#include "globals.hpp"
#include "init.hpp"
#include "tasks/tasks.hpp"

static const char* TAG = "main";

// Namespace shortcuts for backward compatibility
using namespace config;
using namespace globals;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Mag参照ベクトルをバッファの平均値で設定
 *
 * ARM時またはbinlog開始時に呼び出す
 */
/**
 * @brief 加速度と地磁気のバッファ平均を使って姿勢を初期化
 *
 * 加速度計からロール/ピッチを計算し、ヨー=0で初期化。
 * 地磁気リファレンスも正しくNED座標系で設定される。
 */
void initializeAttitudeFromBuffers()
{
    if (g_accel_buffer_count == 0) {
        ESP_LOGW(TAG, "No accel samples in buffer, cannot initialize attitude");
        return;
    }
    if (g_mag_buffer_count == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot initialize attitude");
        return;
    }

    // 加速度バッファの平均を計算
    stampfly::math::Vector3 accel_sum = stampfly::math::Vector3::zero();
    int accel_count = std::min(g_accel_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < accel_count; i++) {
        accel_sum += g_accel_buffer[i];
    }
    stampfly::math::Vector3 accel_avg = accel_sum * (1.0f / accel_count);

    // 地磁気バッファの平均を計算
    stampfly::math::Vector3 mag_sum = stampfly::math::Vector3::zero();
    int mag_count = std::min(g_mag_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < mag_count; i++) {
        mag_sum += g_mag_buffer[i];
    }
    stampfly::math::Vector3 mag_avg = mag_sum * (1.0f / mag_count);

    // センサーフュージョンの姿勢を初期化
    if (g_fusion.isInitialized()) {
        g_fusion.initializeAttitude(accel_avg, mag_avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Attitude initialized from buffers: accel=%d samples, mag=%d samples",
                 accel_count, mag_count);
    }
}

/**
 * @brief 地磁気リファレンスのみをバッファから設定（レガシー互換）
 * @deprecated initializeAttitudeFromBuffers()を使用してください
 */
void setMagReferenceFromBuffer()
{
    if (g_mag_buffer_count == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot set reference");
        return;
    }

    // バッファの平均を計算
    stampfly::math::Vector3 sum = stampfly::math::Vector3::zero();
    int count = std::min(g_mag_buffer_count, REF_BUFFER_SIZE);
    for (int i = 0; i < count; i++) {
        sum += g_mag_buffer[i];
    }
    stampfly::math::Vector3 avg = sum * (1.0f / count);

    // センサーフュージョンに設定
    if (g_fusion.isInitialized()) {
        g_fusion.setMagReference(avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Mag reference set from %d samples: (%.1f, %.1f, %.1f) uT",
                 count, avg.x, avg.y, avg.z);
    }
}

/**
 * @brief binlog開始時のコールバック
 *
 * ESKFをリセットしてmag_refを設定する
 * PC版ESKFとの同期のため、binlog開始時に初期化
 */
void onBinlogStart()
{
    // センサーフュージョンをリセット（PC版と同じ初期状態にする）
    if (g_fusion.isInitialized()) {
        g_fusion.reset();
        // ジャイロバイアスを復元（reset()でゼロになるため）
        g_fusion.setGyroBias(g_initial_gyro_bias);
        ESP_LOGI(TAG, "Sensor fusion reset for binlog, gyro bias restored");
    }

    // 姿勢を初期化（バッファの最新値で更新）
    initializeAttitudeFromBuffers();
}

// =============================================================================
// Timer Callbacks
// =============================================================================

/**
 * @brief ESP Timer callback for 400Hz IMU timing
 *
 * This callback runs from the esp_timer task context and gives a semaphore
 * to wake up the IMU task with precise 2.5ms timing.
 *
 * Note: ESP_TIMER_TASK dispatch runs in a high-priority FreeRTOS task,
 * not in ISR context, so we use xSemaphoreGive instead of xSemaphoreGiveFromISR.
 */
static void imu_timer_callback(void* arg)
{
    static uint32_t timer_count = 0;
    static uint32_t last_imu_loop = 0;
    static uint32_t last_optflow_loop = 0;
    timer_count++;

    // 10秒ごと（4000回 @ 400Hz）にタイマー生存確認
    if (timer_count % 4000 == 0) {
        // IMUタスクが進行しているかチェック
        if (g_imu_last_loop == last_imu_loop) {
            ESP_LOGW(TAG, "IMU STUCK at checkpoint=%u, loop=%lu",
                     g_imu_checkpoint, g_imu_last_loop);
        } else {
            ESP_LOGI(TAG, "IMU timer alive: count=%lu, imu_loop=%lu",
                     timer_count, g_imu_last_loop);
        }
        last_imu_loop = g_imu_last_loop;

        // OptFlowタスクが進行しているかチェック（10秒で1000回 @ 100Hz）
        if (g_optflow_last_loop == last_optflow_loop && g_optflow_last_loop > 0) {
            ESP_LOGW(TAG, "OptFlow STUCK at checkpoint=%u, loop=%lu",
                     g_optflow_checkpoint, g_optflow_last_loop);
        }
        last_optflow_loop = g_optflow_last_loop;
    }
    xSemaphoreGive(g_imu_semaphore);
}

// =============================================================================
// Task Functions
// =============================================================================

/**
 * @brief IMU Task - 400Hz (2.5ms period)
 * Reads BMI270 FIFO, applies filters, updates estimators
 *
 * Timing: Uses ESP Timer for precise 2.5ms (400Hz) period.
 * The timer callback gives a semaphore which this task waits on.
 */

// =============================================================================
// Button Event Handler
// =============================================================================

void onButtonEvent(stampfly::Button::Event event)
{
    auto& state = stampfly::StampFlyState::getInstance();

    switch (event) {
        case stampfly::Button::Event::CLICK:
            ESP_LOGI(TAG, "Button: CLICK");
            // Toggle arm/disarm in IDLE state
            if (state.getFlightState() == stampfly::FlightState::IDLE) {
                if (state.requestArm()) {
                    // ARM時に姿勢を初期化（現在の向き=Yaw 0°）
                    initializeAttitudeFromBuffers();
                    g_motor.resetStats();  // モーター統計リセット
                    g_buzzer.armTone();
                    ESP_LOGI(TAG, "Motors ARMED");
                }
            } else if (state.getFlightState() == stampfly::FlightState::ARMED) {
                if (state.requestDisarm()) {
                    g_motor.saveStatsToNVS();  // モーター統計保存
                    g_buzzer.disarmTone();
                    ESP_LOGI(TAG, "Motors DISARMED");
                }
            }
            break;

        case stampfly::Button::Event::DOUBLE_CLICK:
            ESP_LOGI(TAG, "Button: DOUBLE_CLICK");
            break;

        case stampfly::Button::Event::LONG_PRESS_START:
            ESP_LOGI(TAG, "Button: LONG_PRESS_START");
            break;

        case stampfly::Button::Event::LONG_PRESS_3S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (3s) - Entering pairing mode");
            g_comm.enterPairingMode();
            state.setPairingState(stampfly::PairingState::PAIRING);
            // ペアリングモード: 青色高速点滅（PAIRING優先度）
            stampfly::LEDManager::getInstance().requestChannel(
                stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::PAIRING,
                stampfly::LEDPattern::BLINK_FAST, 0x0000FF);
            g_buzzer.beep();
            break;

        case stampfly::Button::Event::LONG_PRESS_5S:
            ESP_LOGI(TAG, "Button: LONG_PRESS (5s) - System reset");
            g_buzzer.beep();
            vTaskDelay(pdMS_TO_TICKS(600));
            esp_restart();
            break;

        default:
            break;
    }
}

// =============================================================================
// Control Packet Handler
// =============================================================================

void onControlPacket(const stampfly::ControlPacket& packet)
{
    auto& state = stampfly::StampFlyState::getInstance();

    // Update control inputs (raw ADC values)
    state.updateControlInput(
        packet.throttle,
        packet.roll,
        packet.pitch,
        packet.yaw
    );

    // Update control flags
    state.updateControlFlags(packet.flags);

    // Handle arm/disarm toggle from controller (rising edge detection)
    static bool prev_arm_flag = false;
    bool arm_flag = (packet.flags & stampfly::CTRL_FLAG_ARM) != 0;

    // Detect rising edge (button press)
    if (arm_flag && !prev_arm_flag) {
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state == stampfly::FlightState::IDLE ||
            flight_state == stampfly::FlightState::ERROR) {
            // IDLE/ERROR → ARM
            if (state.requestArm()) {
                g_motor.arm();  // Enable motor driver
                initializeAttitudeFromBuffers();
                g_motor.resetStats();  // モーター統計リセット
                g_buzzer.armTone();
                ESP_LOGI(TAG, "Motors ARMED (from controller)");
            }
        } else if (flight_state == stampfly::FlightState::ARMED) {
            // ARMED → DISARM
            if (state.requestDisarm()) {
                g_motor.saveStatsToNVS();  // モーター統計保存
                g_motor.disarm();  // Disable motor driver and stop motors
                g_buzzer.disarmTone();
                ESP_LOGI(TAG, "Motors DISARMED (from controller)");
            }
        }
    }
    prev_arm_flag = arm_flag;
}


static void startTasks()
{
    ESP_LOGI(TAG, "Starting FreeRTOS tasks...");

    // Peripheral tasks (Core 0)
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", STACK_SIZE_LED, nullptr,
                            PRIORITY_LED_TASK, &g_led_task_handle, 0);

    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", STACK_SIZE_BUTTON, nullptr,
                            PRIORITY_BUTTON_TASK, &g_button_task_handle, 0);

    xTaskCreatePinnedToCore(PowerTask, "PowerTask", STACK_SIZE_POWER, nullptr,
                            PRIORITY_POWER_TASK, &g_power_task_handle, 0);

    // Initialize IMU timer and semaphores for precise 400Hz timing
    g_imu_semaphore = xSemaphoreCreateBinary();
    if (g_imu_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create IMU semaphore");
    }

    g_control_semaphore = xSemaphoreCreateBinary();
    if (g_control_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create control semaphore");
    }

    esp_timer_create_args_t imu_timer_args = {
        .callback = imu_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,  // Run in esp_timer task context
        .name = "imu_timer",
        .skip_unhandled_events = true,      // Skip if semaphore already given
    };
    esp_err_t ret = esp_timer_create(&imu_timer_args, &g_imu_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IMU timer: %s", esp_err_to_name(ret));
    } else {
        // Start periodic timer: 2500μs = 2.5ms = 400Hz
        ret = esp_timer_start_periodic(g_imu_timer, 2500);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start IMU timer: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU timer started: 2.5ms period (400Hz)");
        }
    }

    // Sensor tasks (Core 1)
    xTaskCreatePinnedToCore(IMUTask, "IMUTask", STACK_SIZE_IMU, nullptr,
                            PRIORITY_IMU_TASK, &g_imu_task_handle, 1);

    // Control task (Core 1) - runs at 400Hz, synchronized with IMU via semaphore
    xTaskCreatePinnedToCore(ControlTask, "ControlTask", STACK_SIZE_CONTROL, nullptr,
                            PRIORITY_CONTROL_TASK, &g_control_task_handle, 1);

    // I2C sensor tasks (Core 0) - moved from Core 1 to avoid contention with 400Hz IMU
    xTaskCreatePinnedToCore(MagTask, "MagTask", STACK_SIZE_MAG, nullptr,
                            PRIORITY_MAG_TASK, &g_mag_task_handle, 0);

    xTaskCreatePinnedToCore(BaroTask, "BaroTask", STACK_SIZE_BARO, nullptr,
                            PRIORITY_BARO_TASK, &g_baro_task_handle, 0);

    xTaskCreatePinnedToCore(ToFTask, "ToFTask", STACK_SIZE_TOF, nullptr,
                            PRIORITY_TOF_TASK, &g_tof_task_handle, 0);

    xTaskCreatePinnedToCore(OptFlowTask, "OptFlowTask", STACK_SIZE_OPTFLOW, nullptr,
                            PRIORITY_OPTFLOW_TASK, &g_optflow_task_handle, 1);

    // Communication task (Core 0)
    xTaskCreatePinnedToCore(CommTask, "CommTask", STACK_SIZE_COMM, nullptr,
                            PRIORITY_COMM_TASK, &g_comm_task_handle, 0);

    // CLI task (Core 0)
    xTaskCreatePinnedToCore(CLITask, "CLITask", STACK_SIZE_CLI, nullptr,
                            PRIORITY_CLI_TASK, &g_cli_task_handle, 0);

    // Telemetry task (Core 0) - WebSocket broadcast at 50Hz
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", STACK_SIZE_TELEMETRY, nullptr,
                            PRIORITY_TELEMETRY_TASK, &g_telemetry_task_handle, 0);

    ESP_LOGI(TAG, "All tasks started");
}

// =============================================================================
// Main Entry Point
// =============================================================================

extern "C" void app_main(void)
{
    // Delay to allow USB to connect for debug output
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Direct printf for early debug (before ESP_LOG may be configured)
    printf("\n\n*** StampFly Boot Start ***\n\n");
    fflush(stdout);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  StampFly RTOS Skeleton");
    ESP_LOGI(TAG, "  ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "========================================");

    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize network interface and event loop (required for WiFi/ESP-NOW)
    ESP_LOGI(TAG, "Initializing network interface...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network interface initialized");

    // Initialize state manager
    ESP_LOGI(TAG, "Initializing state manager...");
    auto& state = stampfly::StampFlyState::getInstance();
    ESP_ERROR_CHECK(state.init());

    // Initialize system manager
    ESP_LOGI(TAG, "Initializing system manager...");
    auto& sys_mgr = stampfly::SystemManager::getInstance();
    stampfly::SystemManager::Config sys_cfg;
    sys_cfg.init_timeout_ms = 5000;
    sys_cfg.calib_timeout_ms = 10000;
    ESP_ERROR_CHECK(sys_mgr.init(sys_cfg));

    // Initialize I2C bus first (required for I2C sensors)
    ESP_LOGI(TAG, "Initializing I2C...");
    init::i2c();

    // Initialize actuators first (for buzzer feedback)
    ESP_LOGI(TAG, "Initializing actuators...");
    init::actuators();

    // LEDManagerを取得
    auto& led_mgr = stampfly::LEDManager::getInstance();

    // =========================================================================
    // Phase 1: Place on ground (White breathing) - 3 seconds
    // =========================================================================
    ESP_LOGI(TAG, "Phase 1: Place aircraft on ground (3 seconds)...");
    g_buzzer.startTone();
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::BREATHE, 0xFFFFFF);  // White
    for (int i = 3; i > 0; i--) {
        ESP_LOGI(TAG, "  %d...", i);
        // LED更新を手動で行う（LEDタスクはまだ起動していない）
        for (int j = 0; j < 30; j++) {
            led_mgr.update();
            vTaskDelay(pdMS_TO_TICKS(33));  // ~30Hz
        }
    }

    // =========================================================================
    // Phase 2: Sensor initialization (Blue breathing)
    // =========================================================================
    ESP_LOGI(TAG, "Phase 2: Initializing sensors...");
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::BREATHE, 0x0000FF);  // Blue

    // LED更新ヘルパー（初期化中に数回呼び出す）
    auto updateLedDuringInit = [&]() {
        for (int i = 0; i < 10; i++) {
            led_mgr.update();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    };

    updateLedDuringInit();
    init::sensors();
    updateLedDuringInit();

    // Initialize estimators
    ESP_LOGI(TAG, "Initializing estimators...");
    init::estimators();
    updateLedDuringInit();

    // Initialize communication (ESP-NOW)
    ESP_LOGI(TAG, "Initializing communication...");
    init::communication();
    updateLedDuringInit();

    // Initialize CLI
    ESP_LOGI(TAG, "Initializing CLI...");
    init::cli();
    updateLedDuringInit();

    // Initialize Logger (400Hz binary log)
    ESP_LOGI(TAG, "Initializing Logger...");
    init::logger();
    updateLedDuringInit();

    // Initialize Telemetry (WebSocket server)
    ESP_LOGI(TAG, "Initializing Telemetry...");
    init::telemetry();
    updateLedDuringInit();

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    startTasks();

    // =========================================================================
    // Phase 3: Sensor stabilization (Magenta blinking)
    // =========================================================================
    // g_eskf_ready = false なので IMUTask は sensor fusion 処理をスキップしている
    ESP_LOGI(TAG, "Phase 3: Waiting for sensors to stabilize...");
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::BLINK_SLOW, 0xFF00FF);  // Magenta blink

    // 安定判定パラメータ（config.hpp から取得）
    using namespace config::stability;

    int stable_count = 0;
    int elapsed_ms = 0;

    // 各センサーの最終std norm値を保持
    float last_accel_std_norm = 0.0f;
    float last_gyro_std_norm = 0.0f;
    float last_mag_std_norm = 0.0f;
    float last_baro_std = 0.0f;
    float last_tof_std = 0.0f;
    float last_optflow_std = 0.0f;

    int64_t start_time = esp_timer_get_time();

    // デバッグログ用（1秒ごとにログ出力）
    int last_log_sec = 0;

    while (elapsed_ms < MAX_WAIT_MS) {
        // LED更新（LEDタスクが動いていても念のため）
        led_mgr.update();
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
        elapsed_ms += CHECK_INTERVAL_MS;

        // 全センサーのバッファが最小サンプル数を満たしているかチェック
        bool buffers_ready =
            g_accel_buffer_count >= MIN_ACCEL_SAMPLES &&
            g_gyro_buffer_count >= MIN_GYRO_SAMPLES &&
            g_mag_buffer_count >= MIN_MAG_SAMPLES &&
            g_baro_buffer_count >= MIN_BARO_SAMPLES &&
            g_tof_bottom_buffer_count >= MIN_TOF_SAMPLES &&
            g_optflow_buffer_count >= MIN_OPTFLOW_SAMPLES;

        // バッファからstd normを計算
        if (buffers_ready) {
            // 加速度の平均と標準偏差を計算
            stampfly::math::Vector3 accel_sum = stampfly::math::Vector3::zero();
            stampfly::math::Vector3 accel_sum_sq = stampfly::math::Vector3::zero();
            int accel_n = std::min(g_accel_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < accel_n; i++) {
                accel_sum += g_accel_buffer[i];
                accel_sum_sq.x += g_accel_buffer[i].x * g_accel_buffer[i].x;
                accel_sum_sq.y += g_accel_buffer[i].y * g_accel_buffer[i].y;
                accel_sum_sq.z += g_accel_buffer[i].z * g_accel_buffer[i].z;
            }
            float n = static_cast<float>(accel_n);
            stampfly::math::Vector3 accel_avg = accel_sum * (1.0f / n);
            float accel_var_x = accel_sum_sq.x / n - accel_avg.x * accel_avg.x;
            float accel_var_y = accel_sum_sq.y / n - accel_avg.y * accel_avg.y;
            float accel_var_z = accel_sum_sq.z / n - accel_avg.z * accel_avg.z;
            stampfly::math::Vector3 accel_std(
                std::sqrt(std::max(0.0f, accel_var_x)),
                std::sqrt(std::max(0.0f, accel_var_y)),
                std::sqrt(std::max(0.0f, accel_var_z)));
            float accel_std_norm = std::sqrt(accel_var_x + accel_var_y + accel_var_z);
            accel_std_norm = std::sqrt(std::max(0.0f, accel_var_x) +
                                        std::max(0.0f, accel_var_y) +
                                        std::max(0.0f, accel_var_z));

            // 地磁気の平均と標準偏差を計算
            stampfly::math::Vector3 mag_sum = stampfly::math::Vector3::zero();
            stampfly::math::Vector3 mag_sum_sq = stampfly::math::Vector3::zero();
            int mag_n = std::min(g_mag_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < mag_n; i++) {
                mag_sum += g_mag_buffer[i];
                mag_sum_sq.x += g_mag_buffer[i].x * g_mag_buffer[i].x;
                mag_sum_sq.y += g_mag_buffer[i].y * g_mag_buffer[i].y;
                mag_sum_sq.z += g_mag_buffer[i].z * g_mag_buffer[i].z;
            }
            n = static_cast<float>(mag_n);
            stampfly::math::Vector3 mag_avg = mag_sum * (1.0f / n);
            float mag_var_x = mag_sum_sq.x / n - mag_avg.x * mag_avg.x;
            float mag_var_y = mag_sum_sq.y / n - mag_avg.y * mag_avg.y;
            float mag_var_z = mag_sum_sq.z / n - mag_avg.z * mag_avg.z;
            stampfly::math::Vector3 mag_std(
                std::sqrt(std::max(0.0f, mag_var_x)),
                std::sqrt(std::max(0.0f, mag_var_y)),
                std::sqrt(std::max(0.0f, mag_var_z)));
            float mag_std_norm = std::sqrt(std::max(0.0f, mag_var_x) +
                                            std::max(0.0f, mag_var_y) +
                                            std::max(0.0f, mag_var_z));

            // ジャイロの統計計算
            stampfly::math::Vector3 gyro_sum = stampfly::math::Vector3::zero();
            stampfly::math::Vector3 gyro_sum_sq = stampfly::math::Vector3::zero();
            int gyro_n = std::min(g_gyro_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < gyro_n; i++) {
                gyro_sum += g_gyro_buffer[i];
                gyro_sum_sq.x += g_gyro_buffer[i].x * g_gyro_buffer[i].x;
                gyro_sum_sq.y += g_gyro_buffer[i].y * g_gyro_buffer[i].y;
                gyro_sum_sq.z += g_gyro_buffer[i].z * g_gyro_buffer[i].z;
            }
            float gn = static_cast<float>(gyro_n);
            stampfly::math::Vector3 gyro_avg = gyro_sum * (1.0f / gn);
            float gyro_var_x = gyro_sum_sq.x / gn - gyro_avg.x * gyro_avg.x;
            float gyro_var_y = gyro_sum_sq.y / gn - gyro_avg.y * gyro_avg.y;
            float gyro_var_z = gyro_sum_sq.z / gn - gyro_avg.z * gyro_avg.z;
            stampfly::math::Vector3 gyro_std(
                std::sqrt(std::max(0.0f, gyro_var_x)),
                std::sqrt(std::max(0.0f, gyro_var_y)),
                std::sqrt(std::max(0.0f, gyro_var_z)));
            float gyro_std_norm = std::sqrt(std::max(0.0f, gyro_var_x) +
                                            std::max(0.0f, gyro_var_y) +
                                            std::max(0.0f, gyro_var_z));

            // 気圧高度の統計計算
            float baro_sum = 0.0f, baro_sum_sq = 0.0f;
            int baro_n = std::min(g_baro_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < baro_n; i++) {
                baro_sum += g_baro_buffer[i];
                baro_sum_sq += g_baro_buffer[i] * g_baro_buffer[i];
            }
            float baro_avg = baro_sum / baro_n;
            float baro_var = baro_sum_sq / baro_n - baro_avg * baro_avg;
            float baro_std = std::sqrt(std::max(0.0f, baro_var));

            // ToF底面の統計計算
            float tof_sum = 0.0f, tof_sum_sq = 0.0f;
            int tof_n = std::min(g_tof_bottom_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < tof_n; i++) {
                tof_sum += g_tof_bottom_buffer[i];
                tof_sum_sq += g_tof_bottom_buffer[i] * g_tof_bottom_buffer[i];
            }
            float tof_avg = tof_sum / tof_n;
            float tof_var = tof_sum_sq / tof_n - tof_avg * tof_avg;
            float tof_std = std::sqrt(std::max(0.0f, tof_var));

            // オプティカルフローの統計計算
            float dx_sum = 0.0f, dy_sum = 0.0f, squal_sum = 0.0f;
            float dx_sum_sq = 0.0f, dy_sum_sq = 0.0f;
            int flow_n = std::min(g_optflow_buffer_count, REF_BUFFER_SIZE);
            for (int i = 0; i < flow_n; i++) {
                dx_sum += g_optflow_buffer[i].dx;
                dy_sum += g_optflow_buffer[i].dy;
                squal_sum += g_optflow_buffer[i].squal;
                dx_sum_sq += g_optflow_buffer[i].dx * g_optflow_buffer[i].dx;
                dy_sum_sq += g_optflow_buffer[i].dy * g_optflow_buffer[i].dy;
            }
            float dx_avg = dx_sum / flow_n;
            float dy_avg = dy_sum / flow_n;
            (void)squal_sum;  // 将来のsqual安定判定用に保持
            float dx_var = dx_sum_sq / flow_n - dx_avg * dx_avg;
            float dy_var = dy_sum_sq / flow_n - dy_avg * dy_avg;
            float dx_std = std::sqrt(std::max(0.0f, dx_var));
            float dy_std = std::sqrt(std::max(0.0f, dy_var));
            float optflow_std = dx_std + dy_std;  // dx_std + dy_std で判定

            // 最終値を保存
            last_accel_std_norm = accel_std_norm;
            last_gyro_std_norm = gyro_std_norm;
            last_mag_std_norm = mag_std_norm;
            last_baro_std = baro_std;
            last_tof_std = tof_std;
            last_optflow_std = optflow_std;

            // 1秒ごとにデバッグログ出力
            int current_sec = elapsed_ms / 1000;
            if (current_sec > last_log_sec) {
                last_log_sec = current_sec;
                ESP_LOGI(TAG, "=== SENSOR STATS t=%ds ===", current_sec);
                ESP_LOGI(TAG, "  Accel std: %.4f (th=%.3f) avg=(%.3f,%.3f,%.3f) n=%d %s",
                         accel_std_norm, ACCEL_STD_THRESHOLD,
                         accel_avg.x, accel_avg.y, accel_avg.z, accel_n,
                         accel_std_norm < ACCEL_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  Gyro std:  %.5f (th=%.3f) avg=(%.4f,%.4f,%.4f) n=%d %s",
                         gyro_std_norm, GYRO_STD_THRESHOLD,
                         gyro_avg.x, gyro_avg.y, gyro_avg.z, gyro_n,
                         gyro_std_norm < GYRO_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  Mag std:   %.3f (th=%.1f) avg=(%.1f,%.1f,%.1f) n=%d %s",
                         mag_std_norm, MAG_STD_THRESHOLD,
                         mag_avg.x, mag_avg.y, mag_avg.z, mag_n,
                         mag_std_norm < MAG_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  Baro std:  %.4f m (th=%.2f) avg=%.2f m n=%d idx=%d %s",
                         baro_std, BARO_STD_THRESHOLD, baro_avg,
                         baro_n, g_baro_buffer_index,
                         baro_std < BARO_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  ToF std:   %.4f (th=%.3f) avg=%.1f mm n=%d %s",
                         tof_std, TOF_STD_THRESHOLD, tof_avg, tof_n,
                         tof_std < TOF_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  Flow std:  %.1f (th=%.1f) avg=(%.1f,%.1f) n=%d %s",
                         optflow_std, OPTFLOW_STD_THRESHOLD,
                         dx_avg, dy_avg, flow_n,
                         optflow_std < OPTFLOW_STD_THRESHOLD ? "OK" : "NG");
                ESP_LOGI(TAG, "  ----------------------------------------");
            }

            // 全センサーの安定判定（最小待機時間経過後）
            bool accel_stable = accel_std_norm < ACCEL_STD_THRESHOLD;
            bool gyro_stable = gyro_std_norm < GYRO_STD_THRESHOLD;
            bool mag_stable = mag_std_norm < MAG_STD_THRESHOLD;
            bool baro_stable = baro_std < BARO_STD_THRESHOLD;
            bool tof_stable = tof_std < TOF_STD_THRESHOLD;
            bool optflow_stable = optflow_std < OPTFLOW_STD_THRESHOLD;

            bool all_stable = accel_stable && gyro_stable && mag_stable &&
                              baro_stable && tof_stable && optflow_stable;

            if (elapsed_ms >= MIN_WAIT_MS && all_stable) {
                stable_count++;
                ESP_LOGI(TAG, "Stability check %d/%d: ALL SENSORS STABLE (t=%dms)",
                         stable_count, STABLE_COUNT_REQUIRED, elapsed_ms);

                if (stable_count >= STABLE_COUNT_REQUIRED) {
                    break;  // 安定条件を満たした
                }
            } else {
                if (stable_count > 0) {
                    ESP_LOGW(TAG, "Stability reset at t=%dms - unstable sensors:", elapsed_ms);
                    if (!accel_stable) ESP_LOGW(TAG, "  Accel: %.4f > %.3f", accel_std_norm, ACCEL_STD_THRESHOLD);
                    if (!gyro_stable) ESP_LOGW(TAG, "  Gyro: %.5f > %.3f", gyro_std_norm, GYRO_STD_THRESHOLD);
                    if (!mag_stable) ESP_LOGW(TAG, "  Mag: %.3f > %.1f", mag_std_norm, MAG_STD_THRESHOLD);
                    if (!baro_stable) ESP_LOGW(TAG, "  Baro: %.4f > %.2f", baro_std, BARO_STD_THRESHOLD);
                    if (!tof_stable) ESP_LOGW(TAG, "  ToF: %.4f > %.3f", tof_std, TOF_STD_THRESHOLD);
                    if (!optflow_stable) ESP_LOGW(TAG, "  Flow: %.1f > %.1f", optflow_std, OPTFLOW_STD_THRESHOLD);
                }
                stable_count = 0;  // 条件を満たさなければリセット
            }
        }
    }

    int64_t end_time = esp_timer_get_time();
    float stabilization_time_ms = (end_time - start_time) / 1000.0f;

    if (stable_count >= STABLE_COUNT_REQUIRED) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "ALL SENSORS STABILIZED after %.0f ms", stabilization_time_ms);
        ESP_LOGI(TAG, "  Accel: %.4f < %.3f", last_accel_std_norm, ACCEL_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Gyro:  %.5f < %.3f", last_gyro_std_norm, GYRO_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Mag:   %.3f < %.1f", last_mag_std_norm, MAG_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Baro:  %.4f < %.2f", last_baro_std, BARO_STD_THRESHOLD);
        ESP_LOGI(TAG, "  ToF:   %.4f < %.3f", last_tof_std, TOF_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Flow:  %.1f < %.1f", last_optflow_std, OPTFLOW_STD_THRESHOLD);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGW(TAG, "Sensor stabilization timeout after %.0f ms", stabilization_time_ms);
        ESP_LOGW(TAG, "Final sensor status:");
        ESP_LOGW(TAG, "  Accel: %.4f (th=%.3f) %s", last_accel_std_norm, ACCEL_STD_THRESHOLD,
                 last_accel_std_norm < ACCEL_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "  Gyro:  %.5f (th=%.3f) %s", last_gyro_std_norm, GYRO_STD_THRESHOLD,
                 last_gyro_std_norm < GYRO_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "  Mag:   %.3f (th=%.1f) %s", last_mag_std_norm, MAG_STD_THRESHOLD,
                 last_mag_std_norm < MAG_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "  Baro:  %.4f (th=%.2f) %s", last_baro_std, BARO_STD_THRESHOLD,
                 last_baro_std < BARO_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "  ToF:   %.4f (th=%.3f) %s", last_tof_std, TOF_STD_THRESHOLD,
                 last_tof_std < TOF_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "  Flow:  %.1f (th=%.1f) %s", last_optflow_std, OPTFLOW_STD_THRESHOLD,
                 last_optflow_std < OPTFLOW_STD_THRESHOLD ? "OK" : "NG");
        ESP_LOGW(TAG, "Proceeding with initialization anyway...");
    }

    // Initialize sensor fusion with fresh state and stable sensor data
    // センサーが安定した状態でリセット・キャリブレーション
    if (g_fusion.isInitialized()) {
        g_fusion.reset();
        g_fusion.setGyroBias(g_initial_gyro_bias);
        initializeAttitudeFromBuffers();

        // センサーフュージョン処理を開始
        g_eskf_ready = true;
        ESP_LOGI(TAG, "Sensor fusion initialized and ready to run");
    }

    // =========================================================================
    // Phase 4: Ready (Green solid)
    // =========================================================================
    ESP_LOGI(TAG, "Phase 4: System ready!");
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();
    vTaskDelay(pdMS_TO_TICKS(150));
    g_buzzer.beep();

    // IDLE状態に遷移（led_taskが通常のLED表示を開始）
    state.setFlightState(stampfly::FlightState::IDLE);

    // BOOTシーケンス終了 - 優先度を解放してled_taskに制御を移行
    led_mgr.releaseChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT);
    // led_taskがFlightState::IDLEを検出してFLIGHT_STATE優先度で緑表示を行う

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "StampFly initialized successfully!");
    ESP_LOGI(TAG, "ESKF ready - you can start logging now");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");
    // Log level is now managed by CLI (loaded from NVS at CLI::init())
    // Use 'loglevel' command to change. Default (no NVS): none

    // Main loop - just monitor system health
    while (true) {
        // Heap monitoring disabled by default (ESP_LOG suppressed)
        // Use 'loglevel info' CLI command to re-enable if needed

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
