/**
 * @file init.cpp
 * @brief 初期化関数の実装
 */

#include "init.hpp"
#include "config.hpp"
#include "globals.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "system_state.hpp"
#include "led_manager.hpp"

// Sensor drivers
#include "bmi270_wrapper.hpp"
#include "bmm150.hpp"
#include "bmp280.hpp"
#include "vl53l3cx_wrapper.hpp"
#include "pmw3901_wrapper.hpp"
#include "power_monitor.hpp"

// Actuators
#include "motor_driver.hpp"
#include "led.hpp"
#include "buzzer.hpp"
#include "button.hpp"

// Estimation
#include "sensor_fusion.hpp"
#include "system_manager.hpp"
#include "filter.hpp"

// Communication
#include "controller_comm.hpp"
#include "console.hpp"
#include "logger.hpp"
#include "telemetry.hpp"
#include "udp_telemetry.hpp"
#include "control_arbiter.hpp"
#include "udp_server.hpp"
#include "wifi_cli.hpp"
#include "flight_command.hpp"
#include "command_queue.hpp"

// NVS
#include "nvs_flash.h"
#include "nvs.h"

// State
#include "stampfly_state.hpp"

static const char* TAG = "init";

using namespace config;
using namespace globals;

// =============================================================================
// File-local variables
// =============================================================================

static i2c_master_bus_handle_t s_i2c_bus = nullptr;

// =============================================================================
// Callbacks (defined in main.cpp, declared here for reference)
// =============================================================================

extern void onButtonEvent(stampfly::Button::Event event);
extern void onControlPacket(const stampfly::ControlPacket& packet);
extern void onBinlogStart();

// =============================================================================
// Initialization Functions
// =============================================================================

namespace init {

i2c_master_bus_handle_t getI2CBus()
{
    return s_i2c_bus;
}

esp_err_t i2c()
{
    ESP_LOGI(TAG, "Initializing I2C bus...");

    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = static_cast<gpio_num_t>(GPIO_I2C_SCL);
    bus_config.sda_io_num = static_cast<gpio_num_t>(GPIO_I2C_SDA);
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

esp_err_t sensors()
{
    ESP_LOGI(TAG, "Initializing sensors...");
    esp_err_t ret;

    // IMU (BMI270) - SPI
    {
        auto cfg = stampfly::BMI270Wrapper::Config::defaultStampFly();
        // config.hppからSPI GPIO設定を適用
        cfg.pin_mosi = static_cast<gpio_num_t>(GPIO_SPI_MOSI);
        cfg.pin_miso = static_cast<gpio_num_t>(GPIO_SPI_MISO);
        cfg.pin_sclk = static_cast<gpio_num_t>(GPIO_SPI_SCK);
        cfg.pin_cs = static_cast<gpio_num_t>(GPIO_IMU_CS);
        cfg.other_cs = static_cast<gpio_num_t>(GPIO_FLOW_CS);

        ret = g_imu.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "IMU initialized");
        }
    }

    // Magnetometer (BMM150) - I2C
    {
        stampfly::BMM150::Config cfg;
        cfg.i2c_bus = s_i2c_bus;
        cfg.i2c_addr = stampfly::BMM150_I2C_ADDR_DEFAULT;
        cfg.data_rate = static_cast<stampfly::BMM150DataRate>(sensor::BMM150_DATA_RATE);
        cfg.preset = static_cast<stampfly::BMM150Preset>(sensor::BMM150_PRESET);

        ret = g_mag.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Magnetometer initialized");
        }
    }

    // Barometer (BMP280) - I2C
    {
        stampfly::BMP280::Config cfg;
        cfg.i2c_bus = s_i2c_bus;
        cfg.i2c_addr = stampfly::BMP280_I2C_ADDR_DEFAULT;
        cfg.mode = static_cast<stampfly::BMP280Mode>(sensor::BMP280_MODE);
        cfg.press_os = static_cast<stampfly::BMP280Oversampling>(sensor::BMP280_PRESS_OVERSAMPLING);
        cfg.temp_os = static_cast<stampfly::BMP280Oversampling>(sensor::BMP280_TEMP_OVERSAMPLING);
        cfg.standby = static_cast<stampfly::BMP280Standby>(sensor::BMP280_STANDBY);
        cfg.filter = static_cast<stampfly::BMP280Filter>(sensor::BMP280_FILTER);

        ret = g_baro.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Barometer init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Barometer initialized");
        }
    }

    // ToF sensors (VL53L3CX) - I2C with XSHUT control
    // Dual sensor initialization: bottom (altitude) and front (obstacle detection)
    // Note: Front ToF is optional (removable for battery adapter)
    {
        ret = stampfly::VL53L3CXWrapper::initDualSensors(
            g_tof_bottom,
            g_tof_front,
            s_i2c_bus,
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_BOTTOM),
            static_cast<gpio_num_t>(GPIO_TOF_XSHUT_FRONT)
        );
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ToF init failed: %s", esp_err_to_name(ret));
        } else {
            // Start ranging for initialized sensors
            if (g_tof_bottom.isInitialized()) {
                g_tof_bottom.startRanging();
                ESP_LOGI(TAG, "Bottom ToF initialized and ranging");
            }
            if (g_tof_front.isInitialized()) {
                g_tof_front.startRanging();
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(true);
                ESP_LOGI(TAG, "Front ToF initialized and ranging");
            } else {
                stampfly::StampFlyState::getInstance().setFrontToFAvailable(false);
                ESP_LOGI(TAG, "Front ToF not available (optional sensor)");
            }
        }
    }

    // Optical Flow (PMW3901) - SPI
    {
        try {
            auto cfg = stampfly::PMW3901::Config::defaultStampFly();
            // config.hppからSPI GPIO設定を適用
            cfg.pin_mosi = static_cast<gpio_num_t>(GPIO_SPI_MOSI);
            cfg.pin_miso = static_cast<gpio_num_t>(GPIO_SPI_MISO);
            cfg.pin_sclk = static_cast<gpio_num_t>(GPIO_SPI_SCK);
            cfg.pin_cs = static_cast<gpio_num_t>(GPIO_FLOW_CS);

            g_optflow = new stampfly::PMW3901(cfg);
            ESP_LOGI(TAG, "Optical Flow initialized");
        } catch (const stampfly::PMW3901Exception& e) {
            ESP_LOGW(TAG, "Optical Flow init failed: %s", e.what());
        }
    }

    // Power Monitor (INA3221) - I2C
    {
        stampfly::PowerMonitor::Config cfg;
        cfg.i2c_bus = s_i2c_bus;
        cfg.i2c_addr = stampfly::INA3221_I2C_ADDR_GND;
        cfg.battery_channel = sensor::POWER_BATTERY_CHANNEL;
        cfg.shunt_resistor_ohm = sensor::POWER_SHUNT_RESISTOR;

        ret = g_power.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power Monitor init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Power Monitor initialized");
        }
    }

    return ESP_OK;
}

esp_err_t actuators()
{
    ESP_LOGI(TAG, "Initializing actuators...");
    esp_err_t ret;

    // Motor Driver
    {
        stampfly::MotorDriver::Config cfg;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FR] = GPIO_MOTOR_M1;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RR] = GPIO_MOTOR_M2;
        cfg.gpio[stampfly::MotorDriver::MOTOR_RL] = GPIO_MOTOR_M3;
        cfg.gpio[stampfly::MotorDriver::MOTOR_FL] = GPIO_MOTOR_M4;
        cfg.pwm_freq_hz = motor::PWM_FREQ_HZ;
        cfg.pwm_resolution_bits = motor::PWM_RESOLUTION_BITS;

        ret = g_motor.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motor Driver init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "Motor Driver initialized");
        g_motor_ptr = &g_motor;  // Set pointer for CLI access
    }

    // LED Manager (3つのLEDを管理: MCU=GPIO21, BODY=GPIO39x2)
    {
        auto& led_mgr = stampfly::LEDManager::getInstance();
        ret = led_mgr.init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "LEDManager init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "LEDManager initialized (MCU=GPIO21, BODY=GPIO39x2)");
        }
    }

    // LED state callback will be registered after SystemStateManager::init()
    // LED状態コールバックはSystemStateManager::init()の後に登録

    // Legacy g_led は削除済み - LEDManager を使用
    // CLI用ポインタはLEDManager経由で提供予定

    // Buzzer
    {
        stampfly::Buzzer::Config cfg;
        cfg.gpio = GPIO_BUZZER;
        cfg.ledc_channel = buzzer::LEDC_CHANNEL;
        cfg.ledc_timer = buzzer::LEDC_TIMER;

        ret = g_buzzer.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Buzzer init failed: %s", esp_err_to_name(ret));
        } else {
            g_buzzer_ptr = &g_buzzer;  // Set pointer for CLI access
            g_buzzer.loadFromNVS();    // Load mute setting from NVS
            ESP_LOGI(TAG, "Buzzer initialized");
        }
    }

    // Button
    {
        stampfly::Button::Config cfg;
        cfg.gpio = GPIO_BUTTON;
        cfg.debounce_ms = button::DEBOUNCE_MS;

        ret = g_button.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Button init failed: %s", esp_err_to_name(ret));
        } else {
            g_button.setCallback(onButtonEvent);
            ESP_LOGI(TAG, "Button initialized");
        }
    }

    return ESP_OK;
}

esp_err_t estimators()
{
    ESP_LOGI(TAG, "Initializing estimators...");

    // Initialize IMU filters (config.hpp から設定)
    for (int i = 0; i < 3; i++) {
        g_accel_lpf[i].init(1.0f / IMU_DT, lpf::ACCEL_CUTOFF_HZ);
        g_gyro_lpf[i].init(1.0f / IMU_DT, lpf::GYRO_CUTOFF_HZ);
    }

    // Initialize magnetometer calibrator and load from NVS
    g_mag_calibrator = &g_mag_cal;  // Set global pointer for CLI access
    if (g_mag_cal.loadFromNVS() == ESP_OK) {
        ESP_LOGI(TAG, "Magnetometer calibration loaded from NVS");
        // デバッグ: isCalibrated()の状態を確認
        auto cal = g_mag_cal.getCalibration();
        ESP_LOGI(TAG, "  isCalibrated=%d, valid=%d, scale=[%.3f,%.3f,%.3f]",
                 g_mag_cal.isCalibrated(), cal.valid, cal.scale_x, cal.scale_y, cal.scale_z);
    } else {
        ESP_LOGW(TAG, "No magnetometer calibration found in NVS");
    }

    // センサーフュージョン (ESKFをラップ)
    // 1. コンポーネントのデフォルト設定を取得
    // 2. アプリ固有の設定 (main/config.hpp) で上書き
    {
        auto& state = stampfly::StampFlyState::getInstance();

        // コンポーネントのデフォルト設定を取得
        stampfly::ESKF::Config eskf_config = stampfly::ESKF::Config::defaultConfig();

        // =================================================================
        // アプリ固有の設定で上書き (main/config.hpp から)
        // 全設定をここで適用。学習者はconfig.hppを見れば全設定を把握できる
        // =================================================================

        // センサー品質閾値（SensorFusion用）
        // Sensor quality thresholds (for SensorFusion)
        sf::SensorFusion::SensorThresholds sensor_thresholds;
        sensor_thresholds.flow_squal_min = FLOW_SQUAL_MIN;
        sensor_thresholds.flow_distance_min = FLOW_DISTANCE_MIN;
        sensor_thresholds.flow_distance_max = FLOW_DISTANCE_MAX;
        sensor_thresholds.tof_distance_min = TOF_DISTANCE_MIN;
        sensor_thresholds.tof_distance_max = TOF_DISTANCE_MAX;

        // センサ有効フラグ — ESKF::Config が唯一の制御元
        // Sensor enable flags — ESKF::Config is the single source of truth
        eskf_config.sensor_enabled[stampfly::ESKF::SENSOR_MAG]  = config::eskf::USE_MAGNETOMETER;
        eskf_config.sensor_enabled[stampfly::ESKF::SENSOR_BARO] = config::eskf::USE_BAROMETER;
        eskf_config.sensor_enabled[stampfly::ESKF::SENSOR_TOF]  = config::eskf::USE_TOF;
        eskf_config.sensor_enabled[stampfly::ESKF::SENSOR_FLOW] = config::eskf::USE_OPTICAL_FLOW;
        eskf_config.yaw_estimation_enabled = config::eskf::ENABLE_YAW_ESTIMATION;

        // プロセスノイズ (Q行列)
        eskf_config.gyro_noise = config::eskf::GYRO_NOISE;
        eskf_config.accel_noise = config::eskf::ACCEL_NOISE;
        eskf_config.gyro_bias_noise = config::eskf::GYRO_BIAS_NOISE;
        eskf_config.accel_bias_noise = config::eskf::ACCEL_BIAS_NOISE;

        // 観測ノイズ (R行列)
        eskf_config.baro_noise = config::eskf::BARO_NOISE;
        eskf_config.tof_noise = config::eskf::TOF_NOISE;
        eskf_config.mag_noise = config::eskf::MAG_NOISE;
        eskf_config.flow_noise = config::eskf::FLOW_NOISE;
        eskf_config.accel_att_noise = config::eskf::ACCEL_ATT_NOISE;

        // 初期共分散 (P行列)
        eskf_config.init_pos_std = config::eskf::INIT_POS_STD;
        eskf_config.init_vel_std = config::eskf::INIT_VEL_STD;
        eskf_config.init_att_std = config::eskf::INIT_ATT_STD;
        eskf_config.init_gyro_bias_std = config::eskf::INIT_GYRO_BIAS_STD;
        eskf_config.init_accel_bias_std = config::eskf::INIT_ACCEL_BIAS_STD;

        // 物理定数
        eskf_config.gravity = config::eskf::GRAVITY;
        eskf_config.mag_ref = stampfly::math::Vector3(
            config::eskf::MAG_REF_X,
            config::eskf::MAG_REF_Y,
            config::eskf::MAG_REF_Z);

        // 閾値
        eskf_config.mahalanobis_threshold = config::eskf::MAHALANOBIS_THRESHOLD;
        eskf_config.tof_tilt_threshold = config::eskf::TOF_TILT_THRESHOLD;
        eskf_config.accel_motion_threshold = config::eskf::ACCEL_MOTION_THRESHOLD;

        // 外れ値棄却ゲート
        eskf_config.baro_innov_gate = config::eskf::BARO_INNOV_GATE;
        eskf_config.tof_innov_gate = config::eskf::TOF_INNOV_GATE;
        eskf_config.mag_chi2_gate = config::eskf::MAG_CHI2_GATE;
        eskf_config.flow_chi2_gate = config::eskf::FLOW_CHI2_GATE;
        eskf_config.flow_innov_clamp = config::eskf::FLOW_INNOV_CLAMP;
        eskf_config.accel_att_chi2_gate = config::eskf::ACCEL_ATT_CHI2_GATE;

        // オプティカルフロー設定
        eskf_config.flow_min_height = config::eskf::FLOW_MIN_HEIGHT;
        eskf_config.flow_max_height = config::eskf::FLOW_MAX_HEIGHT;
        eskf_config.flow_tilt_cos_threshold = config::eskf::FLOW_TILT_COS_THRESHOLD;
        eskf_config.flow_rad_per_pixel = config::eskf::FLOW_RAD_PER_PIXEL;
        eskf_config.flow_cam_to_body[0] = config::eskf::FLOW_CAM_TO_BODY_XX;
        eskf_config.flow_cam_to_body[1] = config::eskf::FLOW_CAM_TO_BODY_XY;
        eskf_config.flow_cam_to_body[2] = config::eskf::FLOW_CAM_TO_BODY_YX;
        eskf_config.flow_cam_to_body[3] = config::eskf::FLOW_CAM_TO_BODY_YY;
        eskf_config.flow_gyro_scale = config::eskf::FLOW_GYRO_SCALE;
        eskf_config.flow_offset[0] = config::eskf::FLOW_OFFSET_X;
        eskf_config.flow_offset[1] = config::eskf::FLOW_OFFSET_Y;

        // 姿勢補正
        eskf_config.k_adaptive = config::eskf::K_ADAPTIVE;
        eskf_config.att_correction_clamp = config::eskf::ATT_CORRECTION_CLAMP;
        eskf_config.mag_norm_min = config::eskf::MAG_NORM_MIN;
        eskf_config.mag_norm_max = config::eskf::MAG_NORM_MAX;

        // SensorFusion初期化
        bool ok = g_fusion.init(eskf_config,
                                sensor_thresholds,
                                config::eskf::MAX_POSITION,
                                config::eskf::MAX_VELOCITY);
        if (!ok) {
            ESP_LOGW(TAG, "Sensor fusion init failed");
            state.setESKFInitialized(false);
        } else {
            ESP_LOGI(TAG, "Sensor fusion initialized (predict at 400Hz)");
            ESP_LOGI(TAG, "  Sensors: flow=%d, baro=%d, tof=%d, mag=%d",
                     config::eskf::USE_OPTICAL_FLOW, config::eskf::USE_BAROMETER,
                     config::eskf::USE_TOF, config::eskf::USE_MAGNETOMETER);
            state.setESKFInitialized(true);
            g_fusion_ptr = &g_fusion;  // Set pointer for CLI access

            // Gyro/accel bias and mag reference are set after Phase 2
            // stabilization in main.cpp (using stable sensor data)
            // ジャイロ/加速度バイアスと磁気リファレンスは main.cpp の
            // Phase 2 安定化後に安定データから設定する
        }
    }

    // Landing Handler initialization
    // 着陸検出・キャリブレーション統合ハンドラ初期化
    {
        stampfly::LandingHandler::Config landing_cfg;
        landing_cfg.landing_altitude_threshold = config::eskf::LANDING_ALT_THRESHOLD;
        landing_cfg.takeoff_altitude_threshold = config::eskf::LANDING_ALT_THRESHOLD * 2.0f;
        landing_cfg.landing_hold_samples = 80;   // 200ms @ 400Hz
        landing_cfg.takeoff_hold_samples = 80;   // 200ms @ 400Hz
        landing_cfg.tof_available = config::eskf::USE_TOF;
        g_landing_handler.init(landing_cfg);
        ESP_LOGI(TAG, "Landing handler initialized (alt=%.2fm, tof=%s)",
                 landing_cfg.landing_altitude_threshold,
                 landing_cfg.tof_available ? "ON" : "OFF");
    }

    return ESP_OK;
}

esp_err_t communication()
{
    ESP_LOGI(TAG, "Initializing communication...");

    // ESP-NOW Controller Communication
    {
        stampfly::ControllerComm::Config cfg;
        cfg.timeout_ms = comm::TIMEOUT_MS;

        // Load WiFi channel from NVS, use default if not found
        int saved_channel = stampfly::ControllerComm::loadChannelFromNVS();
        if (saved_channel >= 1 && saved_channel <= 13) {
            cfg.wifi_channel = saved_channel;
            ESP_LOGI(TAG, "Loaded WiFi channel %d from NVS", saved_channel);
        } else {
            cfg.wifi_channel = comm::WIFI_CHANNEL;
            ESP_LOGI(TAG, "Using default WiFi channel %d", cfg.wifi_channel);
        }

        esp_err_t ret = g_comm.init(cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ControllerComm init failed: %s", esp_err_to_name(ret));
            return ret;
        }

        g_comm.setControlCallback(onControlPacket);

        // Load pairing from NVS
        if (g_comm.loadPairingFromNVS() == ESP_OK && g_comm.isPaired()) {
            ESP_LOGI(TAG, "Loaded pairing from NVS");
            stampfly::StampFlyState::getInstance().setPairingState(stampfly::PairingState::PAIRED);
        }

        g_comm.start();

        // Set global pointer for CLI access
        g_comm_ptr = &g_comm;

        ESP_LOGI(TAG, "ControllerComm initialized");
    }

    // Initialize Control Arbiter
    // Control Arbiterを初期化
    {
        auto& arbiter = stampfly::ControlArbiter::getInstance();
        esp_err_t ret = arbiter.init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ControlArbiter init failed: %s", esp_err_to_name(ret));
        }
    }

    // SystemStateManager is already initialized in main.cpp before all other components
    // SystemStateManagerは既にmain.cppで全コンポーネント前に初期化済み

    // Register LED state callback
    // LED状態コールバックを登録
    {
        auto& sys_state = stampfly::SystemStateManager::getInstance();
        sys_state.subscribeStateChange([](const stampfly::StateTransitionEvent& event) {
            // Update LED when FlightState changes
            // FlightState変更時にLEDを更新
            stampfly::LEDManager::getInstance().onFlightStateChanged(event.to_state);
            ESP_LOGI("StateCallback", "LED updated for state transition: %d -> %d (%s)",
                     static_cast<int>(event.from_state),
                     static_cast<int>(event.to_state),
                     event.reason ? event.reason : "");
        });
        ESP_LOGI(TAG, "LED state callback registered with SystemStateManager");
    }

    // Initialize CommandQueue (must be before FlightCommandService)
    // CommandQueue を初期化（FlightCommandServiceより前）
    {
        auto& cmd_queue = stampfly::CommandQueue::getInstance();
        esp_err_t ret = cmd_queue.init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CommandQueue init failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "CommandQueue initialized");
        }
    }

    // Initialize FlightCommandService
    // FlightCommandService を初期化
    {
        auto& flight_cmd = stampfly::FlightCommandService::getInstance();
        esp_err_t ret = flight_cmd.init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "FlightCommandService init failed: %s", esp_err_to_name(ret));
        }
    }

    // Restore comm mode from NVS
    // NVSから通信モードを復元
    {
        nvs_handle_t handle;
        esp_err_t ret = nvs_open("stampfly_cli", NVS_READONLY, &handle);
        if (ret == ESP_OK) {
            uint8_t saved_mode = 0;
            ret = nvs_get_u8(handle, "comm_mode", &saved_mode);
            nvs_close(handle);

            if (ret == ESP_OK && saved_mode == 1) {
                // UDP mode was saved - restore it
                // UDPモードが保存されていた - 復元する
                ESP_LOGI(TAG, "Restoring UDP mode from NVS");

                auto& arbiter = stampfly::ControlArbiter::getInstance();
                auto& udp_server = stampfly::UDPServer::getInstance();

                // Initialize and start UDP server
                ret = udp_server.init();
                if (ret == ESP_OK) {
                    ret = udp_server.start();
                    if (ret == ESP_OK) {
                        // Set callback for control packets
                        // 制御パケット用コールバックを設定
                        udp_server.setControlCallback([](const stampfly::udp::ControlPacket& pkt, const SockAddrIn*) {
                            auto& arb = stampfly::ControlArbiter::getInstance();
                            arb.updateFromUDP(pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw, pkt.flags);
                            handleControlInput(pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw, pkt.flags);
                        });

                        arbiter.setCommMode(stampfly::CommMode::UDP);
                        ESP_LOGI(TAG, "UDP mode restored - server started on port 8888");
                    } else {
                        ESP_LOGW(TAG, "Failed to start UDP server: %s", esp_err_to_name(ret));
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to init UDP server: %s", esp_err_to_name(ret));
                }
            }
        }
    }

    return ESP_OK;
}

esp_err_t console()
{
    ESP_LOGI(TAG, "Initializing Console...");

    auto& console = stampfly::Console::getInstance();

    esp_err_t ret = console.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Console init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Note: registerAllCommands() is called from CLITask after SerialREPL::init()
    // registerAllCommands() は SerialREPL::init() の後に CLITask から呼ばれる
    // because esp_console_init() is done inside esp_console_new_repl_usb_cdc()
    // esp_console_init() は esp_console_new_repl_usb_cdc() 内で行われるため

    ESP_LOGI(TAG, "Console pre-initialized");
    return ESP_OK;
}

esp_err_t logger()
{
    ESP_LOGI(TAG, "Initializing Logger...");

    // Logging rate from config
    esp_err_t ret = g_logger.init(logger::RATE_HZ);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Logger init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set start callback (ESKF reset + mag_ref setting)
    g_logger.setStartCallback(onBinlogStart);

    // Set global pointer for CLI access
    g_logger_ptr = &g_logger;

    ESP_LOGI(TAG, "Logger initialized at %dHz", logger::RATE_HZ);
    return ESP_OK;
}

esp_err_t telemetry()
{
    ESP_LOGI(TAG, "Initializing Telemetry...");

    auto& telem = stampfly::Telemetry::getInstance();
    stampfly::Telemetry::Config cfg;
    cfg.port = telemetry::PORT;
    cfg.rate_hz = telemetry::RATE_HZ;

    esp_err_t ret = telem.init(cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Telemetry init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Telemetry initialized - Connect to WiFi 'StampFly', open http://192.168.4.1");

    // Initialize UDP telemetry log server
    // UDP テレメトリログサーバーを初期化
    auto& udp_log = stampfly::udp_telem::UDPLogServer::getInstance();
    ret = udp_log.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UDP telemetry log server init failed: %s", esp_err_to_name(ret));
        // Non-fatal: WebSocket telemetry still works
    } else {
        ESP_LOGI(TAG, "UDP telemetry log server ready on port %d",
                 stampfly::udp_telem::UDP_LOG_PORT);
    }

    return ESP_OK;
}

esp_err_t wifi_cli()
{
    ESP_LOGI(TAG, "Initializing WiFi CLI...");

    auto& wifi_cli = stampfly::WiFiCLI::getInstance();
    stampfly::WiFiCLI::Config cfg;
    cfg.port = 23;  // Telnet standard port
    cfg.max_clients = 2;
    cfg.idle_timeout_ms = 300000;  // 5 minutes

    esp_err_t ret = wifi_cli.init(cfg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi CLI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start the server
    // サーバーを開始
    ret = wifi_cli.start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi CLI start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi CLI initialized - telnet 192.168.4.1");
    return ESP_OK;
}

} // namespace init
