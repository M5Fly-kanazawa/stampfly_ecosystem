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
#include "system_state.hpp"
#include "system_manager.hpp"
#include "sensor_fusion.hpp"
#include "filter.hpp"
#include "sensor_health.hpp"

// Communication
#include "controller_comm.hpp"
#include "control_arbiter.hpp"

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
    if (g_accel_buf.count() == 0) {
        ESP_LOGW(TAG, "No accel samples in buffer, cannot initialize attitude");
        return;
    }
    if (g_mag_buf.count() == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot initialize attitude");
        return;
    }

    // バッファ平均を計算
    stampfly::math::Vector3 accel_avg = g_accel_buf.mean();
    stampfly::math::Vector3 mag_avg = g_mag_buf.mean();

    // センサーフュージョンの姿勢を初期化
    if (g_fusion.isInitialized()) {
        g_fusion.initializeAttitude(accel_avg, mag_avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Attitude initialized from buffers: accel=%d samples, mag=%d samples",
                 g_accel_buf.count(), g_mag_buf.count());
    }
}

/**
 * @brief 地磁気リファレンスのみをバッファから設定（レガシー互換）
 * @deprecated initializeAttitudeFromBuffers()を使用してください
 */
void setMagReferenceFromBuffer()
{
    if (g_mag_buf.count() == 0) {
        ESP_LOGW(TAG, "No mag samples in buffer, cannot set reference");
        return;
    }

    // バッファの平均を計算
    stampfly::math::Vector3 avg = g_mag_buf.mean();

    // センサーフュージョンに設定
    if (g_fusion.isInitialized()) {
        g_fusion.setMagReference(avg);
        g_mag_ref_set = true;
        ESP_LOGI(TAG, "Mag reference set from %d samples: (%.1f, %.1f, %.1f) uT",
                 g_mag_buf.count(), avg.x, avg.y, avg.z);
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
        // バイアスを復元（reset()でゼロになるため）
        // Restore biases (reset() zeroes them)
        g_fusion.setGyroBias(g_initial_gyro_bias);
        g_fusion.setAccelBias(g_initial_accel_bias);
        ESP_LOGI(TAG, "Sensor fusion reset for binlog, biases restored");
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
                    g_landing_handler.reset();  // キャリブレーション状態をリセット
                    initializeAttitudeFromBuffers();
                    g_motor.resetStats();
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

// Shared arm flag state for edge detection (used by both ESP-NOW and UDP)
// エッジ検出用の共有ARMフラグ状態（ESP-NOWとUDP両方で使用）
static bool s_prev_arm_flag = false;

/**
 * @brief Shared control input handler for ESP-NOW and UDP
 *        ESP-NOWとUDP共通の制御入力ハンドラ
 *
 * This function is called from both ESP-NOW and UDP callbacks
 * to handle control input and arm/disarm logic.
 */
void handleControlInput(uint16_t throttle, uint16_t roll, uint16_t pitch,
                        uint16_t yaw, uint8_t flags)
{
    auto& state = stampfly::StampFlyState::getInstance();

    // Update control inputs (raw ADC values, 0-4095 range, center 2048)
    state.updateControlInput(throttle, roll, pitch, yaw);

    // Update control flags
    state.updateControlFlags(flags);

    // Handle arm/disarm toggle from controller (rising edge detection)
    bool arm_flag = (flags & stampfly::CTRL_FLAG_ARM) != 0;

    // Detect rising edge (button press)
    if (arm_flag && !s_prev_arm_flag) {
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state == stampfly::FlightState::IDLE ||
            flight_state == stampfly::FlightState::ERROR) {
            // Check if calibration is complete
            if (!g_landing_handler.canArm()) {
                ESP_LOGW(TAG, "Cannot ARM: calibration in progress");
                g_buzzer.errorTone();
            } else if (state.requestArm()) {
                // IDLE/ERROR → ARM
                g_landing_handler.reset();  // キャリブレーション状態をリセット
                g_motor.arm();
                initializeAttitudeFromBuffers();
                g_motor.resetStats();
                g_buzzer.armTone();
                ESP_LOGI(TAG, "Motors ARMED (from controller)");
            }
        } else if (flight_state == stampfly::FlightState::ARMED ||
                   flight_state == stampfly::FlightState::FLYING) {
            // ARMED/FLYING → DISARM
            if (state.requestDisarm()) {
                g_motor.saveStatsToNVS();  // モーター統計保存
                g_motor.disarm();  // Disable motor driver and stop motors
                g_buzzer.disarmTone();
                ESP_LOGI(TAG, "Motors DISARMED (from controller)");
            }
        }
    }
    s_prev_arm_flag = arm_flag;
}

void onControlPacket(const stampfly::ControlPacket& packet)
{
    // Only process ESP-NOW packets if in ESP-NOW mode
    // ESP-NOWモードの場合のみESP-NOWパケットを処理
    auto& arbiter = stampfly::ControlArbiter::getInstance();
    if (arbiter.getCommMode() != stampfly::CommMode::ESPNOW) {
        return;  // Ignore ESP-NOW packets in UDP mode
    }

    // Feed control data to ControlArbiter for the control loop
    // 制御ループ用にControlArbiterへデータを供給
    arbiter.updateFromESPNOW(packet.throttle, packet.roll, packet.pitch,
                             packet.yaw, packet.flags);

    handleControlInput(packet.throttle, packet.roll, packet.pitch,
                       packet.yaw, packet.flags);
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

    // CLI task (Core 0) - waits for g_boot_complete internally before showing prompt
    // CLIタスク - 内部で g_boot_complete を待ってからプロンプト表示
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

    // Initialize SystemStateManager first (required by other components)
    // SystemStateManager を最初に初期化（他のコンポーネントが依存）
    {
        auto& sys_state = stampfly::SystemStateManager::getInstance();
        esp_err_t ret = sys_state.init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SystemStateManager init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SystemStateManager initialized");
        }
    }

    // Initialize actuators first (for buzzer feedback)
    ESP_LOGI(TAG, "Initializing actuators...");
    init::actuators();

    auto& led_mgr = stampfly::LEDManager::getInstance();

    // =========================================================================
    // =========================================================================
    // Phase 1: Peripheral & sensor initialization (White solid LED)
    // Phase 1: ペリフェラル・センサー初期化（白LED点灯）
    //
    // White LED = "Place on flat surface and keep still"
    // 白LED = 「平らな場所に置いて静止してください」
    // Initialization runs during the settle time, then waits for remainder.
    // 初期化は静置時間中に実行し、残り時間を待つ。
    // =========================================================================
    constexpr int SETTLE_TIME_MS = 5000;  // Time for user to place aircraft
    int64_t phase1_start = esp_timer_get_time();

    ESP_LOGI(TAG, "Phase 1: Place on flat surface and keep still (%d s)...", SETTLE_TIME_MS / 1000);
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::SOLID, 0xFFFFFF);  // White solid
    led_mgr.update();  // Apply once (SOLID needs no periodic update)
    g_buzzer.startTone();

    // Run initialization while the user places the aircraft
    // ユーザーが機体を置く間に初期化を進める
    init::sensors();

    ESP_LOGI(TAG, "Initializing estimators...");
    init::estimators();

    ESP_LOGI(TAG, "Initializing communication...");
    init::communication();

    ESP_LOGI(TAG, "Initializing Console...");
    init::console();

    ESP_LOGI(TAG, "Initializing Logger...");
    init::logger();

    ESP_LOGI(TAG, "Initializing Telemetry...");
    init::telemetry();

    ESP_LOGI(TAG, "Initializing WiFi CLI...");
    init::wifi_cli();

    // Wait for remaining settle time (initialization takes ~1s, so ~4s remains)
    // 残りの静置時間を待つ（初期化に ~1s かかるので ~4s 残る）
    int64_t elapsed_us = esp_timer_get_time() - phase1_start;
    int remaining_ms = SETTLE_TIME_MS - static_cast<int>(elapsed_us / 1000);
    if (remaining_ms > 0) {
        ESP_LOGI(TAG, "Waiting %d ms for settle...", remaining_ms);
        vTaskDelay(pdMS_TO_TICKS(remaining_ms));
    }
    ESP_LOGI(TAG, "Phase 1 complete (%.1f s)", (esp_timer_get_time() - phase1_start) / 1e6f);

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    startTasks();

    // =========================================================================
    // センサータスク初期読み取り待ち（存在するセンサーのみ）
    // =========================================================================
    ESP_LOGI(TAG, "Waiting for sensor tasks to start filling buffers...");
    {
        // Detect which sensors are present
        // 存在するセンサーを検出
        const bool has_imu  = g_imu.isInitialized();
        const bool has_mag  = g_mag.isInitialized();
        const bool has_baro = g_baro.isInitialized();
        const bool has_tof  = g_tof_bottom.isInitialized();
        const bool has_flow = (g_optflow != nullptr);

        ESP_LOGI(TAG, "  Sensors: IMU=%d MAG=%d BARO=%d ToF=%d FLOW=%d",
                 has_imu, has_mag, has_baro, has_tof, has_flow);

        // Wait for required sensors (ToF is optional — may have no valid target)
        // 必須センサーを待つ（ToF はオプション — 有効ターゲットがない場合あり）
        int wait_ms = 0;
        constexpr int MAX_SENSOR_INIT_WAIT_MS = 2000;
        while (wait_ms < MAX_SENSOR_INIT_WAIT_MS) {
            bool required_started =
                (!has_imu  || (g_accel_buf.count() > 0 && g_gyro_buf.count() > 0)) &&
                (!has_mag  || g_mag_buf.count() > 0) &&
                (!has_baro || g_baro_buf.count() > 0) &&
                (!has_flow || g_flow_buf.count() > 0);
            // ToF: check but don't block — report status
            // ToF: 確認するがブロックしない

            if (required_started) {
                ESP_LOGI(TAG, "Sensor buffers ready after %d ms (tof=%d)",
                         wait_ms, g_tof_bottom_buf.count());
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(10));
            wait_ms += 10;
        }

        if (wait_ms >= MAX_SENSOR_INIT_WAIT_MS) {
            ESP_LOGW(TAG, "Sensor init timeout: accel=%d gyro=%d mag=%d baro=%d tof=%d flow=%d",
                     g_accel_buf.count(), g_gyro_buf.count(), g_mag_buf.count(),
                     g_baro_buf.count(), g_tof_bottom_buf.count(), g_flow_buf.count());
        }
    }

    // =========================================================================
    // Phase 2: Task startup & sensor stabilization (Magenta blinking)
    // Phase 2: タスク起動 & センサー安定化（マゼンタ点滅）
    // =========================================================================
    // g_eskf_ready = false なので IMUTask は sensor fusion 処理をスキップしている
    ESP_LOGI(TAG, "Phase 2: Waiting for sensors to stabilize...");
    led_mgr.requestChannel(stampfly::LEDChannel::SYSTEM, stampfly::LEDPriority::BOOT,
                           stampfly::LEDPattern::BLINK_SLOW, 0xFF00FF);  // Magenta blink

    // 安定判定パラメータ（config.hpp から取得）
    using namespace config::stability;

    int elapsed_ms = 0;
    int64_t start_time = esp_timer_get_time();

    // Adaptive: skip stabilization for sensors that are not present
    // 適応的: 存在しないセンサーは安定化チェックをスキップ
    // (Optical flow has no stabilization check — only IMU/MAG/BARO/ToF)
    const bool has_imu  = g_imu.isInitialized();
    const bool has_mag  = g_mag.isInitialized() && g_mag_cal.isCalibrated();
    const bool has_baro = g_baro.isInitialized();
    // ToF: initialized but may have no valid data (no target in range)
    // Use buffer count after fill wait — if still 0, no valid target, skip
    // ToF: 初期化済みでもデータなしの場合あり（充填待ち後のバッファで判定）
    const bool has_tof_bottom = g_tof_bottom.isInitialized() && g_tof_bottom_buf.count() > 0;
    const bool has_tof_front  = g_tof_front.isInitialized() && g_tof_front_buf.count() > 0;

    // 各センサーの合格フラグ（存在しない or データなしなら即合格）
    bool accel_passed     = !has_imu;
    bool gyro_passed      = !has_imu;
    bool mag_passed       = !has_mag;
    bool baro_passed      = !has_baro;
    bool tof_bottom_passed = !has_tof_bottom;
    bool tof_front_passed  = !has_tof_front;

    if (g_tof_bottom.isInitialized() && !has_tof_bottom) {
        ESP_LOGW(TAG, "  ToF bottom: initialized but no valid data - skipping");
    }
    if (g_tof_front.isInitialized() && !has_tof_front) {
        ESP_LOGW(TAG, "  ToF front: initialized but no valid data - skipping");
    }

    ESP_LOGI(TAG, "  Checking: IMU=%d MAG=%d BARO=%d ToF_B=%d ToF_F=%d",
             has_imu, has_mag, has_baro, has_tof_bottom, has_tof_front);

    // 各センサーの最終std値を保持
    float last_accel_std_norm = 0.0f;
    float last_gyro_std_norm = 0.0f;
    float last_mag_std_norm = 0.0f;
    float last_baro_std = 0.0f;
    float last_tof_bottom_std = 0.0f;
    float last_tof_front_std = 0.0f;

    // デバッグログ用（1秒ごとにログ出力）
    int last_log_sec = 0;

    // Online statistics accumulators (incremental, buffer-size independent)
    // 逐次統計量アキュムレータ（インクリメンタル、バッファサイズ非依存）
    struct Vec3Stats {
        int n = 0;
        float sx = 0, sy = 0, sz = 0;       // sum
        float sx2 = 0, sy2 = 0, sz2 = 0;    // sum of squares
        int read_idx = 0;                     // ring buffer read position

        void reset(int new_read_idx) {
            n = 0; sx = sy = sz = sx2 = sy2 = sz2 = 0;
            read_idx = new_read_idx;
        }
        void add(const stampfly::math::Vector3& v) {
            n++; sx += v.x; sy += v.y; sz += v.z;
            sx2 += v.x * v.x; sy2 += v.y * v.y; sz2 += v.z * v.z;
        }
        float std_norm() const {
            if (n < 2) return 0.0f;
            float fn = static_cast<float>(n);
            float vx = std::max(0.0f, sx2 / fn - (sx / fn) * (sx / fn));
            float vy = std::max(0.0f, sy2 / fn - (sy / fn) * (sy / fn));
            float vz = std::max(0.0f, sz2 / fn - (sz / fn) * (sz / fn));
            return std::sqrt(vx + vy + vz);
        }
    };

    struct ScalarStats {
        int n = 0;
        float s = 0, s2 = 0;
        int read_idx = 0;

        void reset(int new_read_idx) {
            n = 0; s = s2 = 0;
            read_idx = new_read_idx;
        }
        void add(float v) {
            n++; s += v; s2 += v * v;
        }
        float std_val() const {
            if (n < 2) return 0.0f;
            float fn = static_cast<float>(n);
            float var = std::max(0.0f, s2 / fn - (s / fn) * (s / fn));
            return std::sqrt(var);
        }
    };

    // Consume new samples from ring buffer into accumulator
    // リングバッファから新しいサンプルをアキュムレータに取り込む
    auto consumeVec3 = [](Vec3Stats& st, const auto& buf) {
        int write_idx = buf.raw_index();
        while (st.read_idx != write_idx) {
            st.add(buf.raw_at(st.read_idx));
            st.read_idx = (st.read_idx + 1) % buf.capacity();
        }
    };

    auto consumeScalar = [](ScalarStats& st, const auto& buf) {
        int write_idx = buf.raw_index();
        while (st.read_idx != write_idx) {
            st.add(buf.raw_at(st.read_idx));
            st.read_idx = (st.read_idx + 1) % buf.capacity();
        }
    };

    Vec3Stats accel_st, gyro_st, mag_st;
    ScalarStats baro_st, tof_bottom_st, tof_front_st;
    accel_st.read_idx      = g_accel_buf.raw_index();
    gyro_st.read_idx       = g_gyro_buf.raw_index();
    mag_st.read_idx        = g_mag_buf.raw_index();
    baro_st.read_idx       = g_baro_buf.raw_index();
    tof_bottom_st.read_idx = g_tof_bottom_buf.raw_index();
    tof_front_st.read_idx  = g_tof_front_buf.raw_index();

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

        // Consume new samples from ring buffers into accumulators
        // リングバッファの新サンプルをアキュムレータに取り込む
        if (!accel_passed) consumeVec3(accel_st, g_accel_buf);
        if (!gyro_passed)  consumeVec3(gyro_st, g_gyro_buf);
        if (!mag_passed)   consumeVec3(mag_st, g_mag_buf);
        if (!baro_passed)  consumeScalar(baro_st, g_baro_buf);
        if (!tof_bottom_passed) consumeScalar(tof_bottom_st, g_tof_bottom_buf);
        if (!tof_front_passed)  consumeScalar(tof_front_st, g_tof_front_buf);

        // === 加速度 ===
        if (!accel_passed && accel_st.n >= MIN_ACCEL_SAMPLES) {
            float std_norm = accel_st.std_norm();
            last_accel_std_norm = std_norm;
            if (std_norm < ACCEL_STD_THRESHOLD) {
                accel_passed = true;
                ESP_LOGI(TAG, "  Accel: PASSED (%.4f < %.3f, n=%d)", std_norm, ACCEL_STD_THRESHOLD, accel_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  Accel: NG (%.4f > %.3f, n=%d) -> retry", std_norm, ACCEL_STD_THRESHOLD, accel_st.n);
                accel_st.reset(g_accel_buf.raw_index());
            }
        } else if (do_log && !accel_passed) {
            ESP_LOGI(TAG, "  Accel: waiting (%d/%d samples)", accel_st.n, MIN_ACCEL_SAMPLES);
        }

        // === ジャイロ ===
        if (!gyro_passed && gyro_st.n >= MIN_GYRO_SAMPLES) {
            float std_norm = gyro_st.std_norm();
            last_gyro_std_norm = std_norm;
            if (std_norm < GYRO_STD_THRESHOLD) {
                gyro_passed = true;
                ESP_LOGI(TAG, "  Gyro:  PASSED (%.5f < %.3f, n=%d)", std_norm, GYRO_STD_THRESHOLD, gyro_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  Gyro:  NG (%.5f > %.3f, n=%d) -> retry", std_norm, GYRO_STD_THRESHOLD, gyro_st.n);
                gyro_st.reset(g_gyro_buf.raw_index());
            }
        } else if (do_log && !gyro_passed) {
            ESP_LOGI(TAG, "  Gyro:  waiting (%d/%d samples)", gyro_st.n, MIN_GYRO_SAMPLES);
        }

        // === 地磁気 ===
        if (!mag_passed && mag_st.n >= MIN_MAG_SAMPLES) {
            float std_norm = mag_st.std_norm();
            last_mag_std_norm = std_norm;
            if (std_norm < MAG_STD_THRESHOLD) {
                mag_passed = true;
                ESP_LOGI(TAG, "  Mag:   PASSED (%.3f < %.1f, n=%d)", std_norm, MAG_STD_THRESHOLD, mag_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  Mag:   NG (%.3f > %.1f, n=%d) -> retry", std_norm, MAG_STD_THRESHOLD, mag_st.n);
                mag_st.reset(g_mag_buf.raw_index());
            }
        } else if (do_log && !mag_passed) {
            ESP_LOGI(TAG, "  Mag:   waiting (%d/%d samples)", mag_st.n, MIN_MAG_SAMPLES);
        }

        // === 気圧 ===
        if (!baro_passed && baro_st.n >= MIN_BARO_SAMPLES) {
            float std_val = baro_st.std_val();
            last_baro_std = std_val;
            if (std_val < BARO_STD_THRESHOLD) {
                baro_passed = true;
                ESP_LOGI(TAG, "  Baro:  PASSED (%.4f < %.2f, n=%d)", std_val, BARO_STD_THRESHOLD, baro_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  Baro:  NG (%.4f > %.2f, n=%d) -> retry", std_val, BARO_STD_THRESHOLD, baro_st.n);
                baro_st.reset(g_baro_buf.raw_index());
            }
        } else if (do_log && !baro_passed) {
            ESP_LOGI(TAG, "  Baro:  waiting (%d/%d samples)", baro_st.n, MIN_BARO_SAMPLES);
        }

        // === ToF bottom ===
        if (!tof_bottom_passed && tof_bottom_st.n >= MIN_TOF_SAMPLES) {
            float std_val = tof_bottom_st.std_val();
            last_tof_bottom_std = std_val;
            if (std_val < TOF_STD_THRESHOLD) {
                tof_bottom_passed = true;
                ESP_LOGI(TAG, "  ToF_B: PASSED (%.4f < %.3f, n=%d)", std_val, TOF_STD_THRESHOLD, tof_bottom_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  ToF_B: NG (%.4f > %.3f, n=%d) -> retry", std_val, TOF_STD_THRESHOLD, tof_bottom_st.n);
                tof_bottom_st.reset(g_tof_bottom_buf.raw_index());
            }
        } else if (do_log && !tof_bottom_passed) {
            ESP_LOGI(TAG, "  ToF_B: waiting (%d/%d samples)", tof_bottom_st.n, MIN_TOF_SAMPLES);
        }

        // === ToF front ===
        if (!tof_front_passed && tof_front_st.n >= MIN_TOF_SAMPLES) {
            float std_val = tof_front_st.std_val();
            last_tof_front_std = std_val;
            if (std_val < TOF_STD_THRESHOLD) {
                tof_front_passed = true;
                ESP_LOGI(TAG, "  ToF_F: PASSED (%.4f < %.3f, n=%d)", std_val, TOF_STD_THRESHOLD, tof_front_st.n);
            } else {
                if (do_log) ESP_LOGW(TAG, "  ToF_F: NG (%.4f > %.3f, n=%d) -> retry", std_val, TOF_STD_THRESHOLD, tof_front_st.n);
                tof_front_st.reset(g_tof_front_buf.raw_index());
            }
        } else if (do_log && !tof_front_passed) {
            ESP_LOGI(TAG, "  ToF_F: waiting (%d/%d samples)", tof_front_st.n, MIN_TOF_SAMPLES);
        }

        // 全センサー合格したら終了
        if (accel_passed && gyro_passed && mag_passed && baro_passed && tof_bottom_passed && tof_front_passed) {
            break;
        }
    }

    int64_t end_time = esp_timer_get_time();
    float stabilization_time_ms = (end_time - start_time) / 1000.0f;

    bool all_passed = accel_passed && gyro_passed && mag_passed && baro_passed
                      && tof_bottom_passed && tof_front_passed;

    if (all_passed) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "ALL SENSORS STABILIZED after %.0f ms", stabilization_time_ms);
        if (has_imu)        ESP_LOGI(TAG, "  Accel: %.4f < %.3f", last_accel_std_norm, ACCEL_STD_THRESHOLD);
        if (has_imu)        ESP_LOGI(TAG, "  Gyro:  %.5f < %.3f", last_gyro_std_norm, GYRO_STD_THRESHOLD);
        if (has_mag)        ESP_LOGI(TAG, "  Mag:   %.3f < %.1f", last_mag_std_norm, MAG_STD_THRESHOLD);
        if (has_baro)       ESP_LOGI(TAG, "  Baro:  %.4f < %.2f", last_baro_std, BARO_STD_THRESHOLD);
        if (has_tof_bottom) ESP_LOGI(TAG, "  ToF_B: %.4f < %.3f", last_tof_bottom_std, TOF_STD_THRESHOLD);
        if (has_tof_front)  ESP_LOGI(TAG, "  ToF_F: %.4f < %.3f", last_tof_front_std, TOF_STD_THRESHOLD);
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGW(TAG, "Sensor stabilization timeout after %.0f ms", stabilization_time_ms);
        ESP_LOGW(TAG, "Final sensor status:");
        if (has_imu)        ESP_LOGW(TAG, "  Accel: %s (%.4f, th=%.3f)", accel_passed ? "OK" : "NG", last_accel_std_norm, ACCEL_STD_THRESHOLD);
        if (has_imu)        ESP_LOGW(TAG, "  Gyro:  %s (%.5f, th=%.3f)", gyro_passed ? "OK" : "NG", last_gyro_std_norm, GYRO_STD_THRESHOLD);
        if (has_mag)        ESP_LOGW(TAG, "  Mag:   %s (%.3f, th=%.1f)", mag_passed ? "OK" : "NG", last_mag_std_norm, MAG_STD_THRESHOLD);
        if (has_baro)       ESP_LOGW(TAG, "  Baro:  %s (%.4f, th=%.2f)", baro_passed ? "OK" : "NG", last_baro_std, BARO_STD_THRESHOLD);
        if (has_tof_bottom) ESP_LOGW(TAG, "  ToF_B: %s (%.4f, th=%.3f)", tof_bottom_passed ? "OK" : "NG", last_tof_bottom_std, TOF_STD_THRESHOLD);
        if (has_tof_front)  ESP_LOGW(TAG, "  ToF_F: %s (%.4f, th=%.3f)", tof_front_passed ? "OK" : "NG", last_tof_front_std, TOF_STD_THRESHOLD);
        ESP_LOGW(TAG, "Proceeding with initialization anyway...");
    }

    // Calculate biases from stabilized sensor buffers
    // 安定化後のセンサーバッファからバイアスを計算
    // Sensor tasks have been filling ring buffers during Phase 2.
    // Use the stable data to compute gyro/accel bias.
    // Phase 2 中にセンサータスクがリングバッファを蓄積済み。
    // 安定データからジャイロ/加速度バイアスを計算する。
    {
        int count = g_gyro_buf.count();
        if (count > 0) {
            // Gyro bias = average of raw gyro (stationary → bias only)
            // ジャイロバイアス = 生ジャイロの平均（静止時 → バイアス成分のみ）
            g_initial_gyro_bias = g_gyro_buf.mean();
            // Accel bias = deviation from gravity [0, 0, -g]
            // 加速度バイアス = 重力ベクトル [0, 0, -g] からの偏差
            stampfly::math::Vector3 accel_avg = g_accel_buf.mean();
            g_initial_accel_bias = stampfly::math::Vector3(
                accel_avg.x,
                accel_avg.y,
                accel_avg.z - (-config::eskf::GRAVITY)
            );
            ESP_LOGI(TAG, "Bias recalculated from stable buffers (%d samples):", count);
            ESP_LOGI(TAG, "  Gyro: [%.5f, %.5f, %.5f] rad/s",
                     g_initial_gyro_bias.x, g_initial_gyro_bias.y, g_initial_gyro_bias.z);
            ESP_LOGI(TAG, "  Accel: [%.4f, %.4f, %.4f] m/s²",
                     g_initial_accel_bias.x, g_initial_accel_bias.y, g_initial_accel_bias.z);
        }
    }

    // Initialize sensor fusion with fresh state and stable sensor data
    // センサーが安定した状態でリセット・キャリブレーション
    if (g_fusion.isInitialized()) {
        g_fusion.reset();
        g_fusion.setGyroBias(g_initial_gyro_bias);
        g_fusion.setAccelBias(g_initial_accel_bias);

        // Shrink bias covariance after setting calibrated values.
        // reset() sets P(bg) = init_gyro_bias_std² = 1e-4 which is too large;
        // ESKF observation updates (accel attitude, mag) drag the bias away
        // from the calibrated value within seconds. Use smaller P to trust
        // the calibration data which was computed from 100 stable samples.
        // キャリブレーション値設定後にバイアス共分散を縮小。
        // reset() の P(bg) = 1e-4 は大きすぎ、観測更新が数秒でバイアスを
        // 不正な方向に引っ張る。安定データ100サンプルのキャリブレーション値を信頼する。
        {
            auto& P = g_fusion.getESKF().getCovariance();
            // Gyro bias: trust calibration, reduce P by 100x
            float gb_var = config::eskf::INIT_GYRO_BIAS_STD * config::eskf::INIT_GYRO_BIAS_STD * 0.01f;
            P(9, 9) = gb_var;   // BG_X
            P(10, 10) = gb_var; // BG_Y
            P(11, 11) = gb_var; // BG_Z
            // Accel bias: trust calibration, reduce P by 100x
            float ab_var = config::eskf::INIT_ACCEL_BIAS_STD * config::eskf::INIT_ACCEL_BIAS_STD * 0.01f;
            P(12, 12) = ab_var; // BA_X
            P(13, 13) = ab_var; // BA_Y
            P(14, 14) = ab_var; // BA_Z
        }

        initializeAttitudeFromBuffers();

        // センサーフュージョン処理を開始
        g_eskf_ready = true;
        ESP_LOGI(TAG, "Sensor fusion initialized and ready to run");
    }

    // =========================================================================
    // Phase 3: System ready (Green solid)
    // Phase 3: システム準備完了（緑LED）
    // =========================================================================
    ESP_LOGI(TAG, "Phase 3: System ready!");
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

    // Signal CLI task that boot is complete (prompt can now be shown)
    // CLIタスクにブート完了を通知（プロンプト表示可能）
    g_boot_complete = true;

    // Main loop - just monitor system health
    while (true) {
        // Heap monitoring disabled by default (ESP_LOG suppressed)
        // Use 'loglevel info' CLI command to re-enable if needed

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
