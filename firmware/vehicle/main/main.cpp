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
                // Check if calibration is complete
                if (!g_landing_handler.canArm()) {
                    ESP_LOGW(TAG, "Cannot ARM: calibration in progress");
                    g_buzzer.errorTone();
                    break;
                }
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
                    g_motor.disarm();          // モーター停止（統計更新防止）
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
            // Check if calibration is complete
            if (!g_landing_handler.canArm()) {
                ESP_LOGW(TAG, "Cannot ARM: calibration in progress");
                g_buzzer.errorTone();
            } else if (state.requestArm()) {
                // IDLE/ERROR → ARM
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

    // Counting semaphore for telemetry FFT mode sync with IMU (400Hz)
    // カウンティングセマフォ: broadcast中のIMU信号をキューイング
    // Max count = 16 (enough to buffer during WebSocket transmission)
    g_telemetry_imu_semaphore = xSemaphoreCreateCounting(16, 0);
    if (g_telemetry_imu_semaphore == nullptr) {
        ESP_LOGE(TAG, "Failed to create telemetry IMU semaphore");
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
    // センサータスク初期読み取り待ち
    // =========================================================================
    // センサータスクは vTaskDelayUntil() で最初の読み取り前に10-33ms待機する
    // Phase 3でバッファが空または0初期化値で埋まるのを防ぐため、
    // 全センサーバッファに最低1サンプル入るまで待機
    ESP_LOGI(TAG, "Waiting for sensor tasks to start filling buffers...");
    {
        int wait_ms = 0;
        constexpr int MAX_SENSOR_INIT_WAIT_MS = 2000;  // 最大2秒待機
        while (wait_ms < MAX_SENSOR_INIT_WAIT_MS) {
            bool all_started =
                g_accel_buffer_count > 0 &&
                g_gyro_buffer_count > 0 &&
                g_mag_buffer_count > 0 &&
                g_baro_buffer_count > 0 &&
                g_tof_bottom_buffer_count > 0 &&
                g_optflow_buffer_count > 0;

            if (all_started) {
                ESP_LOGI(TAG, "All sensor buffers started filling after %d ms", wait_ms);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
            wait_ms += 10;
        }

        if (wait_ms >= MAX_SENSOR_INIT_WAIT_MS) {
            ESP_LOGW(TAG, "Sensor init timeout: accel=%d gyro=%d mag=%d baro=%d tof=%d flow=%d",
                     g_accel_buffer_count, g_gyro_buffer_count, g_mag_buffer_count,
                     g_baro_buffer_count, g_tof_bottom_buffer_count, g_optflow_buffer_count);
        }
    }

    // =========================================================================
    // Phase 3: Sensor stabilization (Magenta blinking)
    // =========================================================================
    // g_eskf_ready = false なので IMUTask は sensor fusion 処理をスキップしている
    ESP_LOGI(TAG, "Phase 3: Waiting for sensors to stabilize...");
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::BLINK_SLOW, 0xFF00FF);  // Magenta blink

    // 安定判定パラメータ（config.hpp から取得）
    using namespace config::stability;

    int elapsed_ms = 0;
    int64_t start_time = esp_timer_get_time();

    // 各センサーの合格フラグ（一度合格したらチェック終了）
    bool accel_passed = false;
    bool gyro_passed = false;
    bool mag_passed = false;
    bool baro_passed = false;
    bool tof_passed = false;

    // 各センサーの最終std値を保持
    float last_accel_std_norm = 0.0f;
    float last_gyro_std_norm = 0.0f;
    float last_mag_std_norm = 0.0f;
    float last_baro_std = 0.0f;
    float last_tof_std = 0.0f;

    // デバッグログ用（1秒ごとにログ出力）
    int last_log_sec = 0;

    // ヘルパー: Vector3の標準偏差ノルムを計算
    auto calcVector3StdNorm = [](const auto& buffer, int count, int buffer_size) {
        stampfly::math::Vector3 sum = stampfly::math::Vector3::zero();
        stampfly::math::Vector3 sum_sq = stampfly::math::Vector3::zero();
        int n = std::min(count, buffer_size);
        for (int i = 0; i < n; i++) {
            sum += buffer[i];
            sum_sq.x += buffer[i].x * buffer[i].x;
            sum_sq.y += buffer[i].y * buffer[i].y;
            sum_sq.z += buffer[i].z * buffer[i].z;
        }
        float fn = static_cast<float>(n);
        stampfly::math::Vector3 avg = sum * (1.0f / fn);
        float var_x = std::max(0.0f, sum_sq.x / fn - avg.x * avg.x);
        float var_y = std::max(0.0f, sum_sq.y / fn - avg.y * avg.y);
        float var_z = std::max(0.0f, sum_sq.z / fn - avg.z * avg.z);
        return std::sqrt(var_x + var_y + var_z);
    };

    // ヘルパー: スカラーの標準偏差を計算
    auto calcScalarStd = [](const auto& buffer, int count, int buffer_size) {
        float sum = 0.0f, sum_sq = 0.0f;
        int n = std::min(count, buffer_size);
        for (int i = 0; i < n; i++) {
            sum += buffer[i];
            sum_sq += buffer[i] * buffer[i];
        }
        float fn = static_cast<float>(n);
        float avg = sum / fn;
        float var = std::max(0.0f, sum_sq / fn - avg * avg);
        return std::sqrt(var);
    };

    while (elapsed_ms < MAX_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
        elapsed_ms += CHECK_INTERVAL_MS;

        // 1秒ごとにログ出力
        int current_sec = elapsed_ms / 1000;
        bool do_log = (current_sec > last_log_sec);
        if (do_log) {
            last_log_sec = current_sec;
            ESP_LOGI(TAG, "----------------------------------------");
            ESP_LOGI(TAG, "Stabilization t=%ds:", current_sec);
        }

        // === 加速度 ===
        if (!accel_passed && g_accel_buffer_count >= MIN_ACCEL_SAMPLES) {
            float std_norm = calcVector3StdNorm(g_accel_buffer, g_accel_buffer_count, REF_BUFFER_SIZE);
            last_accel_std_norm = std_norm;
            if (std_norm < ACCEL_STD_THRESHOLD) {
                accel_passed = true;
                ESP_LOGI(TAG, "  Accel: PASSED (%.4f < %.3f)", std_norm, ACCEL_STD_THRESHOLD);
            } else {
                // 不合格: バッファクリアして再サンプリング
                g_accel_buffer_count = 0;
                g_accel_buffer_index = 0;
                if (do_log) ESP_LOGW(TAG, "  Accel: NG (%.4f > %.3f) -> retry", std_norm, ACCEL_STD_THRESHOLD);
            }
        } else if (do_log && !accel_passed) {
            ESP_LOGI(TAG, "  Accel: waiting (%d/%d samples)", g_accel_buffer_count, MIN_ACCEL_SAMPLES);
        }

        // === ジャイロ ===
        if (!gyro_passed && g_gyro_buffer_count >= MIN_GYRO_SAMPLES) {
            float std_norm = calcVector3StdNorm(g_gyro_buffer, g_gyro_buffer_count, REF_BUFFER_SIZE);
            last_gyro_std_norm = std_norm;
            if (std_norm < GYRO_STD_THRESHOLD) {
                gyro_passed = true;
                ESP_LOGI(TAG, "  Gyro:  PASSED (%.5f < %.3f)", std_norm, GYRO_STD_THRESHOLD);
            } else {
                g_gyro_buffer_count = 0;
                g_gyro_buffer_index = 0;
                if (do_log) ESP_LOGW(TAG, "  Gyro:  NG (%.5f > %.3f) -> retry", std_norm, GYRO_STD_THRESHOLD);
            }
        } else if (do_log && !gyro_passed) {
            ESP_LOGI(TAG, "  Gyro:  waiting (%d/%d samples)", g_gyro_buffer_count, MIN_GYRO_SAMPLES);
        }

        // === 地磁気 ===
        if (!mag_passed && g_mag_buffer_count >= MIN_MAG_SAMPLES) {
            float std_norm = calcVector3StdNorm(g_mag_buffer, g_mag_buffer_count, REF_BUFFER_SIZE);
            last_mag_std_norm = std_norm;
            if (std_norm < MAG_STD_THRESHOLD) {
                mag_passed = true;
                ESP_LOGI(TAG, "  Mag:   PASSED (%.3f < %.1f)", std_norm, MAG_STD_THRESHOLD);
            } else {
                g_mag_buffer_count = 0;
                g_mag_buffer_index = 0;
                if (do_log) ESP_LOGW(TAG, "  Mag:   NG (%.3f > %.1f) -> retry", std_norm, MAG_STD_THRESHOLD);
            }
        } else if (do_log && !mag_passed) {
            ESP_LOGI(TAG, "  Mag:   waiting (%d/%d samples)", g_mag_buffer_count, MIN_MAG_SAMPLES);
        }

        // === 気圧 ===
        if (!baro_passed && g_baro_buffer_count >= MIN_BARO_SAMPLES) {
            float std_val = calcScalarStd(g_baro_buffer, g_baro_buffer_count, REF_BUFFER_SIZE);
            last_baro_std = std_val;
            if (std_val < BARO_STD_THRESHOLD) {
                baro_passed = true;
                ESP_LOGI(TAG, "  Baro:  PASSED (%.4f < %.2f)", std_val, BARO_STD_THRESHOLD);
            } else {
                g_baro_buffer_count = 0;
                g_baro_buffer_index = 0;
                if (do_log) ESP_LOGW(TAG, "  Baro:  NG (%.4f > %.2f) -> retry", std_val, BARO_STD_THRESHOLD);
            }
        } else if (do_log && !baro_passed) {
            ESP_LOGI(TAG, "  Baro:  waiting (%d/%d samples)", g_baro_buffer_count, MIN_BARO_SAMPLES);
        }

        // === ToF ===
        if (!tof_passed && g_tof_bottom_buffer_count >= MIN_TOF_SAMPLES) {
            float std_val = calcScalarStd(g_tof_bottom_buffer, g_tof_bottom_buffer_count, REF_BUFFER_SIZE);
            last_tof_std = std_val;
            if (std_val < TOF_STD_THRESHOLD) {
                tof_passed = true;
                ESP_LOGI(TAG, "  ToF:   PASSED (%.4f < %.3f)", std_val, TOF_STD_THRESHOLD);
            } else {
                g_tof_bottom_buffer_count = 0;
                g_tof_bottom_buffer_index = 0;
                if (do_log) ESP_LOGW(TAG, "  ToF:   NG (%.4f > %.3f) -> retry", std_val, TOF_STD_THRESHOLD);
            }
        } else if (do_log && !tof_passed) {
            ESP_LOGI(TAG, "  ToF:   waiting (%d/%d samples)", g_tof_bottom_buffer_count, MIN_TOF_SAMPLES);
        }

        // 全センサー合格したら終了
        if (accel_passed && gyro_passed && mag_passed && baro_passed && tof_passed) {
            break;
        }
    }

    int64_t end_time = esp_timer_get_time();
    float stabilization_time_ms = (end_time - start_time) / 1000.0f;

    bool all_passed = accel_passed && gyro_passed && mag_passed && baro_passed && tof_passed;

    if (all_passed) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "ALL SENSORS STABILIZED after %.0f ms", stabilization_time_ms);
        ESP_LOGI(TAG, "  Accel: %.4f < %.3f", last_accel_std_norm, ACCEL_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Gyro:  %.5f < %.3f", last_gyro_std_norm, GYRO_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Mag:   %.3f < %.1f", last_mag_std_norm, MAG_STD_THRESHOLD);
        ESP_LOGI(TAG, "  Baro:  %.4f < %.2f", last_baro_std, BARO_STD_THRESHOLD);
        ESP_LOGI(TAG, "  ToF:   %.4f < %.3f", last_tof_std, TOF_STD_THRESHOLD);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGW(TAG, "Sensor stabilization timeout after %.0f ms", stabilization_time_ms);
        ESP_LOGW(TAG, "Final sensor status:");
        ESP_LOGW(TAG, "  Accel: %s (%.4f, th=%.3f)", accel_passed ? "OK" : "NG", last_accel_std_norm, ACCEL_STD_THRESHOLD);
        ESP_LOGW(TAG, "  Gyro:  %s (%.5f, th=%.3f)", gyro_passed ? "OK" : "NG", last_gyro_std_norm, GYRO_STD_THRESHOLD);
        ESP_LOGW(TAG, "  Mag:   %s (%.3f, th=%.1f)", mag_passed ? "OK" : "NG", last_mag_std_norm, MAG_STD_THRESHOLD);
        ESP_LOGW(TAG, "  Baro:  %s (%.4f, th=%.2f)", baro_passed ? "OK" : "NG", last_baro_std, BARO_STD_THRESHOLD);
        ESP_LOGW(TAG, "  ToF:   %s (%.4f, th=%.3f)", tof_passed ? "OK" : "NG", last_tof_std, TOF_STD_THRESHOLD);
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
