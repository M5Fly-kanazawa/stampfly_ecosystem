/**
 * @file workshop_task.cpp
 * @brief Workshop 400Hz task - calls user's setup()/loop_400Hz()
 *        ワークショップ400Hzタスク - ユーザーの setup()/loop_400Hz() を呼び出す
 *
 * Replaces ControlTask from vehicle firmware. Waits on the same
 * g_control_semaphore (given by IMU task after sensor update).
 */

#include "workshop_globals.hpp"
#include "workshop_api.hpp"
#include "config.hpp"
#include "esp_log.h"

static const char* TAG = "WorkshopTask";

using namespace config;
using namespace globals;

// Shared arm flag state for edge detection
static bool s_prev_arm_flag = false;

void handleControlInput(uint16_t throttle, uint16_t roll, uint16_t pitch,
                        uint16_t yaw, uint8_t flags)
{
    auto& state = stampfly::StampFlyState::getInstance();
    state.updateControlInput(throttle, roll, pitch, yaw);
    state.updateControlFlags(flags);

    // Handle arm/disarm toggle (rising edge detection)
    bool arm_flag = (flags & stampfly::CTRL_FLAG_ARM) != 0;
    if (arm_flag && !s_prev_arm_flag) {
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state == stampfly::FlightState::IDLE ||
            flight_state == stampfly::FlightState::ERROR) {
            if (g_landing_handler.canArm() && state.requestArm()) {
                g_motor.arm();
                g_motor.resetStats();
                g_buzzer.armTone();
                ESP_LOGI(TAG, "Motors ARMED");
            }
        } else if (flight_state == stampfly::FlightState::ARMED ||
                   flight_state == stampfly::FlightState::FLYING) {
            if (state.requestDisarm()) {
                g_motor.saveStatsToNVS();
                g_motor.disarm();
                g_buzzer.disarmTone();
                ESP_LOGI(TAG, "Motors DISARMED");
            }
        }
    }
    s_prev_arm_flag = arm_flag;
}

// Yaw alert counter stub (referenced by led_task)
volatile int g_yaw_alert_counter = 0;

void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "WorkshopTask started (400Hz via semaphore)");

    // Call user's setup function before CLI banner is shown
    // CLIバナー表示前にユーザーの setup() を呼び出す
    // SerialCLI waits on g_setup_complete (weak symbol) so setup() output
    // appears in the boot log and the banner/prompt come after it.
    // SerialCLI は g_setup_complete (weak シンボル) を待つため、
    // setup() の出力はブートログに残り、その後にバナーとプロンプトが表示される
    setup();

    // Signal setup() done — SerialCLI shows the banner after this flips true
    // setup() 完了を通知 - SerialCLI はこれが true になった後にバナーを表示
    g_setup_complete = true;

    constexpr float dt = IMU_DT;

    while (true) {
        // Wait for semaphore from IMU task (400Hz sync)
        if (xSemaphoreTake(g_control_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        auto& state = stampfly::StampFlyState::getInstance();
        stampfly::FlightState flight_state = state.getFlightState();

        // Always call user loop (allows sensor reading / LED control while disarmed)
        // 常にユーザーループを呼ぶ（DISARM中もセンサ読取・LED制御を可能にする）
        loop_400Hz(dt);

        // Safety: always stop motors when not armed, regardless of user code
        // 安全対策: ARM状態でない場合は常にモーターを停止
        if (flight_state != stampfly::FlightState::ARMED &&
            flight_state != stampfly::FlightState::FLYING) {
            ws::motor_stop_all();
        }
    }
}
