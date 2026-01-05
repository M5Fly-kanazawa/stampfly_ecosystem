/*
* MIT License
* 
* Copyright (c) 2024 Kouhei Ito
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


//Controller for M5Fly
//#define DEBUG

#include <Arduino.h>
#include <esp_log.h>

// Log tags for different modules
static const char* TAG_MAIN = "MAIN";
static const char* TAG_BEACON = "BEACON";
static const char* TAG_DRONE = "DRONE";
static const char* TAG_TDMA = "TDMA";
static const char* TAG_PEER = "PEER";

// Global log level control (set at compile time)
// ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO, ESP_LOG_DEBUG, ESP_LOG_VERBOSE
#define GLOBAL_LOG_LEVEL ESP_LOG_INFO  // Change this to control verbosity
#include <M5AtomS3.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <MPU6886.h>
#include <MadgwickAHRS.h>
#include <atoms3joy.h>
#include <FS.h>
#include <SPIFFS.h>
#include "buzzer.h"

#define CHANNEL 1

// TDMA Settings
#define TDMA_DEVICE_ID 0         // Device ID: 0=Master, 1-9=Slave (manual setting)
#define TDMA_FRAME_US 20000      // 1 frame = 20ms (extended for 5+ devices)
#define TDMA_SLOT_US 2000        // 1 slot = 2ms (maximum margin for collision avoidance)
#define TDMA_NUM_SLOTS 10        // 10 slots per frame
#define TDMA_BEACON_ADVANCE_US 500  // Beacon fires 500us before slot 0 start (extended for better separation)

#define ANGLECONTROL 0
#define RATECONTROL 1
#define ANGLECONTROL_W_LOG 2
#define RATECONTROL_W_LOG 3
#define ALT_CONTROL_MODE 1
#define NOT_ALT_CONTROL_MODE 0
#define RESO10BIT (4096)

esp_now_peer_info_t dronePeer; // Peer for drone communication
esp_now_peer_info_t beaconPeer;  // Peer for beacon multicast

uint16_t Throttle;
uint16_t Phi, Theta, Psi;
int16_t Phi_bias =0;
int16_t Theta_bias = 0;
int16_t Psi_bias =0;
int16_t Throttle_bias = 0;
short xstick=0;
short ystick=0;
uint8_t Mode=ANGLECONTROL;
uint8_t AltMode=NOT_ALT_CONTROL_MODE;
volatile uint8_t Loop_flag=0;
float Timer = 0.0;
float dTime = 0.01;
uint8_t Timer_state = 0;
uint8_t StickMode = 2;
uint32_t espnow_version;
volatile uint8_t proactive_flag = 0;
unsigned long stime, etime, dtime;
uint8_t axp_cnt = 0;
uint8_t is_peering = 0;
uint8_t senddata[14]; //19->22->23->24->25->14
uint8_t disp_counter = 0;

//StampFly MAC ADDRESS
uint8_t Drone_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
// Broadcast address for TDMA beacons
uint8_t Beacon_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Channel
uint8_t Ch_counter;
volatile uint8_t Received_flag = 0;
volatile uint8_t Channel = CHANNEL;

// Loop synchronization (removed hw_timer, using TDMA semaphore only)
// volatile uint8_t Loop_flag=0;  // Deprecated - use beacon_sem instead

//TDMA
static esp_timer_handle_t beacon_timer;
static SemaphoreHandle_t beacon_sem;
static TaskHandle_t beacon_task_handle = NULL;  // Task for beacon transmission
// TDMA timing - simplified design
// frame_start_time_us = Current frame's reference time (timer fire time or beacon RX time)
// All slot calculations use this as the base reference
static volatile int64_t frame_start_time_us = 0;  // Current frame's timer fire time (64-bit for overflow protection)

// PLL for synchronization
static volatile int32_t pll_error_us = 0;      // Phase error in microseconds
static volatile int32_t pll_integral = 0;       // Integral term for PLL
static const float PLL_KP = 0.1;                // Proportional gain
static const float PLL_KI = 0.01;               // Integral gain
static const int32_t PLL_ERROR_CLAMP = 500;     // Max error for P term: ±0.5ms (half slot)
static const int32_t PLL_RESYNC_THRESHOLD = 800; // Resync if error > 0.8ms (80% of slot)
static volatile bool first_beacon_received = false; // First beacon flag for slaves
static volatile int64_t last_beacon_time_us = 0;    // Last beacon reception time (64-bit for overflow protection)
static const uint32_t BEACON_TIMEOUT_US = 50000;    // 50ms = 5 frames

// TDMA Send Task (Phase 1: Task separation)
static TaskHandle_t tdma_send_task_handle = NULL;  // Task handle for TDMA transmission
static uint8_t shared_senddata[14];                 // Shared buffer for TDMA packet
static SemaphoreHandle_t senddata_mutex = NULL;     // Mutex to protect shared_senddata

// Input Task (Phase 2: Sensor reading separation)
static TaskHandle_t input_task_handle = NULL;      // Task handle for input processing
static SemaphoreHandle_t input_mutex = NULL;        // Mutex to protect shared_inputdata
struct InputData {
    int16_t throttle_raw;     // Raw joystick values
    int16_t phi_raw;
    int16_t theta_raw;
    int16_t psi_raw;
    bool btn_pressed;         // Button state
    bool btn_long_pressed;
    uint8_t mode_changed;     // Mode change flags
    uint8_t alt_mode_changed;
};
static struct InputData shared_inputdata = {0};

// Actual TDMA transmission frequency (measured from send intervals)
static volatile uint32_t actual_send_freq_hz = 0;

// Function declarations
void wifi_esp_now_init(void);
void broadcast_beacon_init(void);
void drone_peer_init(void);
void peering(void);
void change_channel(uint8_t ch);
void save_data(void);
void load_data(void);
void data_send(void);
void show_battery_info();
void voltage_print(void);
void beacon_task(void* parameter);
void IRAM_ATTR beacon_timer_callback(void* arg);
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
float limit(float v, float vmin, float vmax);
uint8_t check_control_mode_change(void);
uint8_t check_alt_mode_change(void);

// Send callback statistics
static volatile uint32_t send_success_count = 0;
static volatile uint32_t send_fail_count = 0;
static volatile uint32_t beacon_cb_success = 0;
static volatile uint32_t beacon_cb_fail = 0;
static volatile uint32_t drone_cb_success = 0;
static volatile uint32_t drone_cb_fail = 0;

// Drone connectivity detection
static volatile uint32_t drone_consecutive_failures = 0;
static volatile bool drone_available = true;  // Assume available at start
static const uint32_t DRONE_FAILURE_THRESHOLD = 10;  // 10 consecutive failures = drone offline (100ms)
static volatile uint32_t drone_offline_time_ms = 0;  // Time when drone went offline
static const uint32_t DRONE_RETRY_INTERVAL_MS = 5000;  // Retry every 5 seconds

// Air time measurement (send start to callback)
static volatile int64_t send_start_time_us = 0;
static volatile int64_t min_air_time_us = INT64_MAX;
static volatile int64_t max_air_time_us = 0;
static volatile int64_t total_air_time_us = 0;
static volatile uint32_t air_time_sample_count = 0;

// Beacon transmission task (for master only)
// This task waits for notification from timer ISR and sends beacon
void beacon_task(void* parameter)
{
    uint8_t beacon_data[2] = {0xBE, 0xAC};
    static int64_t last_beacon_sent_us = 0;
    static uint32_t beacon_count = 0;
    static uint32_t error_count = 0;
    static uint32_t last_re_init = 0;

    while (1) {
        // Wait for notification from timer ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Check if beacon peer exists in peer table before sending
        if (!esp_now_is_peer_exist(beaconPeer.peer_addr)) {
            // Peer not found - try to re-register
            ESP_LOGW(TAG_BEACON, "Peer not found! Re-registering...");
            broadcast_beacon_init();
            beacon_count++;
            continue;  // Skip this beacon and wait for next timer
        }

        // Send beacon immediately upon notification (no delay from logging)
        esp_err_t result = esp_now_send(beaconPeer.peer_addr, beacon_data, sizeof(beacon_data));

        // Handle send errors
        if (result != ESP_OK) {
            error_count++;

            // ESP_ERR_ESPNOW_NOT_FOUND (12391 = 0x3067) - peer not found
            if (result == ESP_ERR_ESPNOW_NOT_FOUND || result == 12391) {
                // Double-check if peer really exists (debugging)
                bool peer_exists = esp_now_is_peer_exist(beaconPeer.peer_addr);
                if (beacon_count % 10 == 0 || beacon_count <= 100) {
                    ESP_LOGD(TAG_BEACON, "ESP_ERR_ESPNOW_NOT_FOUND but peer_exists=%d at #%u",
                             peer_exists, beacon_count);
                }
                // Retry immediately once
                delayMicroseconds(50);
                result = esp_now_send(beaconPeer.peer_addr, beacon_data, sizeof(beacon_data));

                // If still failing, re-initialize peer (once per second max)
                if (result != ESP_OK && beacon_count - last_re_init > 100) {
                    // Try to get peer info before deleting
                    esp_now_peer_info_t peer_info;
                    esp_err_t get_result = esp_now_get_peer(beaconPeer.peer_addr, &peer_info);
                    ESP_LOGD(TAG_BEACON, "Peer get_result=%d (0=found) at #%u", get_result, beacon_count);

                    if (get_result == ESP_OK) {
                        ESP_LOGD(TAG_BEACON, "Peer details: ch=%d, ifidx=%d, encrypt=%d",
                                 peer_info.channel, peer_info.ifidx, peer_info.encrypt);
                    }

                    esp_now_del_peer(beaconPeer.peer_addr);  // Remove old peer
                    delayMicroseconds(100);  // Short delay for peer cleanup (was delay(5))
                    broadcast_beacon_init();  // Re-register
                    last_re_init = beacon_count;
                    ESP_LOGW(TAG_BEACON, "Peer re-init at #%u (err=%d, total_err=%u)",
                             beacon_count, result, error_count);
                }
            }
            // ESP_ERR_ESPNOW_NO_MEM or other errors
            else {
                // Just log other errors (buffer full, etc.)
                if (beacon_count % 100 == 0) {
                    ESP_LOGW(TAG_BEACON, "Send error: %d at #%u", result, beacon_count);
                }
            }
        }

        beacon_count++;

        #if 0  // Disable logging to prevent blocking in task context
        int64_t current_time = esp_timer_get_time();
        int64_t interval_us = current_time - last_beacon_sent_us;

        // Log first 100 beacons with detailed interval timing
        if (beacon_count <= 100) {
            ESP_LOGD(TAG_BEACON, "Beacon TX #%u: interval=%lld us, result=%d",
                     beacon_count, interval_us, result);
        }
        // Then log every 100 beacons to monitor stability
        else if (beacon_count % 100 == 0) {
            ESP_LOGD(TAG_BEACON, "Beacon TX #%u: interval=%lld us, result=%d, errors=%u",
                     beacon_count, interval_us, result, error_count);
        }
        // Every 1000 beacons show full statistics
        if (beacon_count % 1000 == 0) {
            ESP_LOGI(TAG_BEACON, "Total CB: ok=%u fail=%u | Beacon CB: ok=%u fail=%u | Drone CB: ok=%u fail=%u",
                     send_success_count, send_fail_count,
                     beacon_cb_success, beacon_cb_fail,
                     drone_cb_success, drone_cb_fail);
        }

        last_beacon_sent_us = current_time;
        #endif
    }
}

// TDMA Send Task (Phase 1: Dedicated task for TDMA transmission)
// This task waits for beacon_sem and sends control data in assigned TDMA slot
// Priority: Highest (same as beacon_task) to minimize latency
void tdma_send_task(void *pvParameters) {
    uint8_t local_senddata[14];  // Local copy to minimize mutex lock time

    ESP_LOGI(TAG_TDMA, "TDMA Send Task started");

    // Skip first few frames to allow system stabilization
    static const uint32_t WARMUP_FRAMES = 5;
    static uint32_t frame_counter = 0;

    for(;;) {
        // Wait for TDMA timer semaphore (blocking wait)
        if (xSemaphoreTake(beacon_sem, portMAX_DELAY) == pdTRUE) {
            frame_counter++;

            // Skip first few frames for system warmup
            if (frame_counter <= WARMUP_FRAMES) {
                if (frame_counter == WARMUP_FRAMES) {
                    ESP_LOGI(TAG_TDMA, "System warmup complete. Starting TDMA transmission.");
                }
                continue;  // Skip this frame
            }
            // Copy shared data to local buffer (minimize critical section)
            if (xSemaphoreTake(senddata_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                memcpy(local_senddata, shared_senddata, 14);
                xSemaphoreGive(senddata_mutex);
            } else {
                // Mutex timeout - skip this frame
                static uint32_t mutex_timeout_count = 0;
                mutex_timeout_count++;
                if (mutex_timeout_count % 100 == 1) {
                    ESP_LOGW(TAG_TDMA, "Mutex timeout - skipping frame #%u", mutex_timeout_count);
                    beep_mutex_timeout();  // 2000Hz triple beep = mutex timeout
                }
                continue;
            }

            // Check if drone is available before sending (prevent queue overflow)
            if (!drone_available) {
                // Check if it's time to retry
                uint32_t current_time_ms = millis();
                if (current_time_ms - drone_offline_time_ms >= DRONE_RETRY_INTERVAL_MS) {
                    // Time to retry - reset drone_available
                    drone_available = true;
                    drone_consecutive_failures = 0;
                    ESP_LOGI(TAG_DRONE, "Retrying drone transmission (offline for %u ms)...",
                             current_time_ms - drone_offline_time_ms);
                } else {
                    // Still in offline pause period - skip transmission
                    static uint32_t skip_log_count = 0;
                    skip_log_count++;
                    if (skip_log_count % 100 == 1) {
                        ESP_LOGD(TAG_DRONE, "Skipping drone send (offline) #%u, will retry in %u ms",
                                 skip_log_count, DRONE_RETRY_INTERVAL_MS - (current_time_ms - drone_offline_time_ms));
                    }
                    continue;  // Skip this frame
                }
            }

            // Check if drone peer exists before sending
            static uint32_t peer_check_fail_count = 0;
            if (!esp_now_is_peer_exist(dronePeer.peer_addr)) {
                peer_check_fail_count++;
                if (peer_check_fail_count % 100 == 1) {  // Log first occurrence and every 100th
                    ESP_LOGW(TAG_PEER, "Drone peer not found! Re-registering... (count=%u)", peer_check_fail_count);
                    beep_drone_offline();  // 1000Hz low tone = drone offline
                }
                drone_peer_init();  // Re-register drone peer
            }

            // Calculate slot start time (simplified - no reverse calculation needed!)
            // frame_start_time_us = Timer fire time (master) or Beacon RX time (slave)
            // TDMA_BEACON_ADVANCE_US = 800us (beacon sent before slot 0)
            // Slot 0 starts at: frame_start_time_us + 800us
            // Our slot starts at: Slot 0 + (DEVICE_ID * 1000us)
            int64_t semaphore_acquired_time = esp_timer_get_time();  // Record when semaphore was acquired
            int64_t slot_0_start_us = frame_start_time_us + TDMA_BEACON_ADVANCE_US;
            int64_t slot_start_us = slot_0_start_us + (TDMA_DEVICE_ID * TDMA_SLOT_US);
            int64_t current_us = esp_timer_get_time();

            // Debug: Log timing calculations (first 20 frames)
            static uint32_t timing_debug_count = 0;
            timing_debug_count++;
            if (timing_debug_count <= 20) {
                ESP_LOGD(TAG_TDMA, "Frame #%u: sem_acq=%lld, frame_start=%lld, slot_start=%lld, current=%lld, diff=%lld",
                         timing_debug_count, semaphore_acquired_time, frame_start_time_us,
                         slot_start_us, current_us, slot_start_us - current_us);
            }

            // Precise microsecond timing for TDMA slot synchronization
            if (slot_start_us > current_us) {
                int64_t wait_us = slot_start_us - current_us;

                // If wait time > 20us, use delayMicroseconds for precise timing
                if (wait_us > 20) {
                    delayMicroseconds(wait_us - 10);  // Wait until 10us before slot start
                }

                // Final precise timing with busy wait
                while (esp_timer_get_time() < slot_start_us) {
                    // Busy wait for ultimate precision
                }
            } else {
                // Slot start time is in the past - this is a timing error!
                static uint32_t late_count = 0;
                late_count++;
                if (late_count <= 20 || late_count % 100 == 0) {
                    ESP_LOGW(TAG_TDMA, "Slot start missed #%u! slot_start was %lld us ago (sem_acq_delay=%lld us)",
                             late_count, current_us - slot_start_us, semaphore_acquired_time - frame_start_time_us);
                    if (late_count % 100 == 0) {
                        beep_slot_error();  // 3000Hz double beep = slot timing error
                    }
                }
            }

            // Send control data in our assigned slot (immediately after timing wait)
            int64_t actual_send_time = esp_timer_get_time();
            send_start_time_us = actual_send_time;  // Capture for air time measurement
            esp_err_t result = esp_now_send(dronePeer.peer_addr, local_senddata, sizeof(local_senddata));

            // Calculate actual transmission frequency from measured send intervals
            static int64_t last_send_time_us = 0;
            if (last_send_time_us > 0) {
                int64_t interval_us = actual_send_time - last_send_time_us;
                if (interval_us > 0) {
                    actual_send_freq_hz = 1000000 / interval_us;
                }
            }
            last_send_time_us = actual_send_time;

            // Log send result for debugging (controlled by ESP_LOG level)
            static uint32_t send_count = 0;
            send_count++;
            if (result != ESP_OK) {
                ESP_LOGE(TAG_DRONE, "Send FAILED #%u: err=%d (MAC=%02X:%02X)",
                         send_count, result, dronePeer.peer_addr[4], dronePeer.peer_addr[5]);
            } else if (send_count <= 20 || send_count % 100 == 0) {
                // Log timing for first 20 sends to debug slot timing
                int64_t timing_error = actual_send_time - slot_start_us;
                ESP_LOGD(TAG_DRONE, "Send OK #%u (MAC=%02X:%02X) slot_err=%lld us | CB: ok=%u fail=%u",
                         send_count, dronePeer.peer_addr[4], dronePeer.peer_addr[5], timing_error,
                         drone_cb_success, drone_cb_fail);
            }
        }
    }
}

// Input Task (Phase 2: Dedicated task for sensor reading)
// This task reads joystick and button inputs at 100Hz (10ms period)
// Priority: High (but lower than TDMA tasks) for responsive input
void input_task(void *pvParameters) {
    static const char* TAG_INPUT = "INPUT";
    ESP_LOGI(TAG_INPUT, "Input Task started");

    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100Hz = 10ms period
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Local variables for input processing
    struct InputData local_input;
    static uint32_t task_count = 0;

    for(;;) {
        // Wait for the next cycle (100Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        task_count++;

        // Update M5 button state and joystick
        M5.update();
        joy_update();

        // Read raw joystick values
        local_input.throttle_raw = getThrottle();
        local_input.phi_raw = getAileron();
        local_input.theta_raw = getElevator();
        local_input.psi_raw = getRudder();

        // Read button states
        local_input.btn_pressed = M5.Btn.wasPressed();
        local_input.btn_long_pressed = M5.Btn.pressedFor(400);

        // Check mode button changes (these functions use getModeButton/getOptionButton internally)
        local_input.mode_changed = check_control_mode_change();
        local_input.alt_mode_changed = check_alt_mode_change();

        // Update shared buffer with mutex protection
        if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            memcpy(&shared_inputdata, &local_input, sizeof(InputData));
            xSemaphoreGive(input_mutex);
        } else {
            // Mutex timeout - should be rare
            static uint32_t mutex_timeout_count = 0;
            mutex_timeout_count++;
            if (mutex_timeout_count % 100 == 1) {
                ESP_LOGW(TAG_INPUT, "Mutex timeout #%u", mutex_timeout_count);
                beep_mutex_timeout();  // 2000Hz triple beep = mutex timeout
            }
        }

        // Log task health (every 1 second = 100 cycles)
        if (task_count % 100 == 0) {
            ESP_LOGD(TAG_INPUT, "Input task healthy: count=%u, throttle=%d, phi=%d",
                     task_count, local_input.throttle_raw, local_input.phi_raw);
        }
    }
}

// TDMA beacon timer callback
// Note: Cannot call esp_now_send() here - not ISR safe!
void IRAM_ATTR beacon_timer_callback(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t callback_count = 0;
    static int64_t last_callback_time = 0;
    callback_count++;

    // Measure timer callback interval (master only, for debugging)
    #if 0  // Disable logging to prevent blocking in ISR context
    if (TDMA_DEVICE_ID == 0 && callback_count <= 100) {
        int64_t now = esp_timer_get_time();
        int64_t timer_interval = now - last_callback_time;
        if (callback_count > 1) {
            // Use ets_printf for ISR-safe logging
            ets_printf("Timer CB #%u: interval=%lld us\n", callback_count, timer_interval);
        }
        last_callback_time = now;
    }
    #endif

    // Record current frame start time (timer fire time)
    // This is the reference time for all slot calculations in this frame
    frame_start_time_us = esp_timer_get_time();

    // Master: Notify beacon task to send beacon immediately
    if (TDMA_DEVICE_ID == 0 && beacon_task_handle != NULL) {
        vTaskNotifyGiveFromISR(beacon_task_handle, &xHigherPriorityTaskWoken);
    }
    // Slave: frame_start_time_us will be overwritten when beacon is received (more accurate sync)
    //        If beacon is lost, this timer-based value serves as fallback

    // Give semaphore to signal that it's time to send control data
    xSemaphoreGiveFromISR(beacon_sem, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

float limit(float v, float vmin, float vmax)
{
  if (v<vmin)v=vmin;
  if (v>vmax)v=vmax;
  return v;
}

// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Capture callback time immediately for air time measurement
  int64_t callback_time_us = esp_timer_get_time();

  // Check if this is beacon (FF:FF:FF:FF:FF:FF) or drone packet
  bool is_beacon = (mac_addr[0] == 0xFF && mac_addr[1] == 0xFF &&
                    mac_addr[2] == 0xFF && mac_addr[3] == 0xFF &&
                    mac_addr[4] == 0xFF && mac_addr[5] == 0xFF);

  // Measure air time for drone packets (not beacons)
  if (!is_beacon && send_start_time_us > 0) {
    int64_t air_time_us = callback_time_us - send_start_time_us;

    // Update statistics (only for successful sends)
    if (status == ESP_NOW_SEND_SUCCESS && air_time_us > 0 && air_time_us < 10000) {
      if (air_time_us < min_air_time_us) min_air_time_us = air_time_us;
      if (air_time_us > max_air_time_us) max_air_time_us = air_time_us;
      total_air_time_us += air_time_us;
      air_time_sample_count++;

      // Log statistics every 100 successful transmissions (DEBUG level)
      if (air_time_sample_count % 100 == 0) {
        int64_t avg_air_time_us = total_air_time_us / air_time_sample_count;
        ESP_LOGD(TAG_DRONE, "AirTime Stats [n=%u]: min=%lld avg=%lld max=%lld us",
                 air_time_sample_count, min_air_time_us, avg_air_time_us, max_air_time_us);
      }
    }
  }

  // Track send statistics
  if (status == ESP_NOW_SEND_SUCCESS) {
    send_success_count++;
    if (is_beacon) {
      beacon_cb_success++;
    } else {
      drone_cb_success++;
      drone_consecutive_failures = 0;  // Reset failure counter on success
      if (!drone_available) {
        drone_available = true;  // Drone is back online
        ESP_LOGI(TAG_DRONE, "Drone reconnected!");
      }
    }
  } else {
    send_fail_count++;
    if (is_beacon) {
      beacon_cb_fail++;
    } else {
      drone_cb_fail++;
      drone_consecutive_failures++;

      // Log if many consecutive failures (stop transmitting to prevent queue overflow)
      if (drone_available && drone_consecutive_failures >= DRONE_FAILURE_THRESHOLD) {
        drone_available = false;
        drone_offline_time_ms = millis();  // Record when drone went offline
        ESP_LOGW(TAG_DRONE, "Drone offline detected (failures=%u). Pausing transmission (will retry in %u ms).",
                 drone_consecutive_failures, DRONE_RETRY_INTERVAL_MS);
      }
    }

    // Log send failures for debugging (rate limited)
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time > 1000) {  // Log once per second max
      if (is_beacon) {
        ESP_LOGW(TAG_BEACON, "Send CB: FAIL BEACON (beacon_ok=%u, beacon_fail=%u)",
                 beacon_cb_success, beacon_cb_fail);
      } else {
        ESP_LOGW(TAG_DRONE, "Send CB: FAIL DRONE %02X:%02X:%02X:%02X:%02X:%02X (drone_ok=%u, drone_fail=%u)",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                 drone_cb_success, drone_cb_fail);
      }
      last_log_time = now;
    }
  }
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len)
{
  if (is_peering) {
    if (recv_data[7] == 0xaa && recv_data[8] == 0x55 && recv_data[9] == 0x16 && recv_data[10] == 0x88) {
        Received_flag = 1;
        // TDMA mode: Use manually configured CHANNEL, ignore drone's channel
        // Channel = recv_data[0];  // Disabled for TDMA
        Drone_mac[0]      = recv_data[1];
        Drone_mac[1]      = recv_data[2];
        Drone_mac[2]      = recv_data[3];
        Drone_mac[3]      = recv_data[4];
        Drone_mac[4]      = recv_data[5];
        Drone_mac[5]      = recv_data[6];
        ESP_LOGI(TAG_PEER, "Receive ! (Using CHANNEL=%d)", CHANNEL);
    }
  }
  else {
    // Check if this is a beacon packet (for TDMA synchronization)
    // Beacon: exactly 2 bytes with 0xBE 0xAC header
    if (data_len == 2 && recv_data[0] == 0xBE && recv_data[1] == 0xAC) {
      // Beacon packet detected
      if (TDMA_DEVICE_ID != 0) {
        // Slave device received beacon from master
        // Update frame start time to beacon reception time (accurate synchronization)
        int64_t current_time = esp_timer_get_time();
        frame_start_time_us = current_time;  // This is the current frame's reference time
        last_beacon_time_us = current_time;  // Update last beacon time

        // Optional: PLL for monitoring synchronization quality (future use)
        #if 0  // Disable logging to prevent blocking in ISR context
        static int64_t last_recv_time = 0;
        int64_t interval_us = current_time - last_recv_time;
        static uint32_t beacon_recv_count = 0;
        beacon_recv_count++;

        // Calculate expected time and error for monitoring
        int64_t expected_time_for_log = 0;
        int32_t error_for_log = 0;
        if (beacon_recv_count > 1) {
          expected_time_for_log = last_recv_time + TDMA_FRAME_US;
          error_for_log = current_time - expected_time_for_log;
        }

        // Log first 100 beacons with detailed info
        if (beacon_recv_count <= 100) {
          if (beacon_recv_count == 1) {
            ESP_LOGD(TAG_BEACON, "Beacon RX #%u: interval=%lld us (first beacon)",
                     beacon_recv_count, interval_us);
          } else {
            ESP_LOGD(TAG_BEACON, "Beacon RX #%u: interval=%lld us, error=%d us",
                     beacon_recv_count, interval_us, error_for_log);
          }
        }
        // Then log every 100 beacons (every 1 second)
        else if (beacon_recv_count % 100 == 0) {
          ESP_LOGD(TAG_BEACON, "Beacon RX #%u: interval=%lld us, error=%d us",
                   beacon_recv_count, interval_us, error_for_log);
        }

        last_recv_time = current_time;
        #endif

        if (!first_beacon_received) {
          first_beacon_received = true;
          pll_error_us = 0;
          pll_integral = 0;
        }
      }
      // Master device ignores beacon (own echo-back)
      return;
    }

    #if 0
    //テレメトリーデータ受信
    // Check minimum length for telemetry data
    if (data_len < 2) {
      return;  // Too short, ignore
    }
    
    //データ受信時に実行したい内容をここに書く。
    float a;
    uint8_t *dummy;
    uint8_t offset = 2;

    //Channel_detected_flag++;
    //if(Channel_detected_flag>10)Channel_detected_flag=10;
    //Serial.printf("Channel=%d  ",Channel);
    dummy=(uint8_t*)&a;
    dummy[0]=recv_data[0];
    dummy[1]=recv_data[1];

    // Filter out invalid packets
    if (dummy[0]==0xF4)return;
    if (dummy[0]==0xBE && dummy[1]==0xAC)return;  // Double-check beacon filter

    // Validate data length before processing telemetry
    // Telemetry format: 2-byte header + N*4-byte floats
    if ((data_len - offset) % 4 != 0 || data_len < (offset + 4)) {
      // Invalid telemetry format, ignore
      return;
    }

    if ((dummy[0]==99)&&(dummy[1]==99))ESP_LOGI(TAG_DRONE, "#PID Gain P Ti Td Eta ");

    uint8_t num_floats = (data_len-offset)/4;
    ESP_LOGV(TAG_DRONE, "num_floats=%d ", num_floats);

    for (uint8_t i=0; i < num_floats; i++)
    {
      // Bounds check before accessing array
      uint16_t base_idx = i*4 + offset;
      if (base_idx + 3 < data_len) {
        dummy[0]=recv_data[base_idx + 0];
        dummy[1]=recv_data[base_idx + 1];
        dummy[2]=recv_data[base_idx + 2];
        dummy[3]=recv_data[base_idx + 3];
        ESP_LOGV(TAG_DRONE, "float[%d]=%9.4f ", i, a);
      }
    }
    ESP_LOGV(TAG_DRONE, "telemetry end");
    #endif
  }
}

#define BUF_SIZE 128
// EEPROMにデータを保存する
void save_data(void)
{
  SPIFFS.begin(true);
  /* CREATE FILE */
  File fp = SPIFFS.open("/peer_info.txt", FILE_WRITE); // 書き込み、存在すれば上書き
  char buf[BUF_SIZE + 1];
  // TDMA mode: Always save CHANNEL define value (not variable Channel)
  sprintf(buf, "%d,%02X,%02X,%02X,%02X,%02X,%02X",
          CHANNEL,  // Use define value, not variable
          Drone_mac[0],
          Drone_mac[1],
          Drone_mac[2],
          Drone_mac[3],
          Drone_mac[4],
          Drone_mac[5]);
  fp.write((uint8_t *)buf, BUF_SIZE);
  fp.close();
  SPIFFS.end();

  ESP_LOGI(TAG_PEER, "Saved Data: CH=%d, MAC=[%02X:%02X:%02X:%02X:%02X:%02X]",
           CHANNEL,  // Use define value
           Drone_mac[0],
           Drone_mac[1],
           Drone_mac[2],
           Drone_mac[3],
           Drone_mac[4],
           Drone_mac[5]);
}

// EEPROMからデータを読み出す
void load_data(void)
{
  SPIFFS.begin(true);
  File fp = SPIFFS.open("/peer_info.txt", FILE_READ);
  char buf[BUF_SIZE + 1];
  while (fp.read((uint8_t *)buf, BUF_SIZE) == BUF_SIZE)
  {
    //ESP_LOGV(TAG_PEER, "buf=%s", buf);
    uint8_t saved_channel;  // Temporary variable for saved channel
    sscanf(buf,"%hhd,%hhX,%hhX,%hhX,%hhX,%hhX,%hhX",
          &saved_channel,  // Read but don't use for TDMA
          &Drone_mac[0],
          &Drone_mac[1],
          &Drone_mac[2],
          &Drone_mac[3],
          &Drone_mac[4],
          &Drone_mac[5]);
    // TDMA mode: Always use CHANNEL define, ignore saved channel
    Channel = CHANNEL;
    ESP_LOGI(TAG_PEER, "Loaded MAC (using CHANNEL=%d): %02X:%02X:%02X:%02X:%02X:%02X",
             CHANNEL,
             Drone_mac[0],
             Drone_mac[1],
             Drone_mac[2],
             Drone_mac[3],
             Drone_mac[4],
             Drone_mac[5]);
  }
  fp.close();
  SPIFFS.end();
}

void wifi_esp_now_init(void)
{  
    // ESP-NOW初期化
    //WiFi.mode(WIFI_STA);
    //WiFi.disconnect();

    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);                      // 省電力OFF推奨
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);


    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_unregister_recv_cb();
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);  // Register send callback
        ESP_LOGI(TAG_MAIN, "ESPNow Init Success");
    } else {
        ESP_LOGE(TAG_MAIN, "ESPNow Init Failed");
        ESP.restart();
    }
  // ESP-NOWコールバック登録
  //esp_now_register_recv_cb(OnDataRecv);

#if 0
    memset(&dronePeer, 0, sizeof(dronePeer));
    memcpy(dronePeer.peer_addr, addr, 6);
    dronePeer.channel = ch;
    dronePeer.encrypt = false;
    uint8_t peer_mac_addre;
    while (esp_now_add_peer(&dronePeer) != ESP_OK) {
        ESP_LOGW(TAG_PEER, "Failed to add peer");
    }
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
#endif
}

void broadcast_beacon_init(void)
{
    memset(&beaconPeer, 0, sizeof(beaconPeer));
    memcpy(beaconPeer.peer_addr, Beacon_mac, 6);
    beaconPeer.channel = CHANNEL;
    beaconPeer.encrypt = false;
    beaconPeer.ifidx = WIFI_IF_STA;  // Explicitly set interface

    // Try to add peer, retry if failed
    esp_err_t result = esp_now_add_peer(&beaconPeer);
    uint8_t retry = 0;
    while (result != ESP_OK && retry < 5) {
        ESP_LOGW(TAG_PEER, "Failed to add beacon peer: %d (retry %d)", result, retry);
        delayMicroseconds(100);  // Short delay for peer cleanup (was delay(5))
        result = esp_now_add_peer(&beaconPeer);
        retry++;
    }

    if (result == ESP_OK) {
        ESP_LOGI(TAG_PEER, "Beacon peer added: %02X:%02X:%02X:%02X:%02X:%02X",
                 Beacon_mac[0], Beacon_mac[1], Beacon_mac[2],
                 Beacon_mac[3], Beacon_mac[4], Beacon_mac[5]);
    } else {
        ESP_LOGE(TAG_PEER, "Failed to add beacon peer after retries: %d", result);
    }

    // チャンネルの設定はESP-NOWの設定の前に行う必要があるかもしれない
    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    delayMicroseconds(100);  // Short delay for initialization (was delay(10))
}

void drone_peer_init(void)
{
    memset(&dronePeer, 0, sizeof(dronePeer));
    memcpy(dronePeer.peer_addr, Drone_mac, 6);
    dronePeer.channel = CHANNEL;
    dronePeer.encrypt = false;
    dronePeer.ifidx = WIFI_IF_STA;
    while (esp_now_add_peer(&dronePeer) != ESP_OK) {
        ESP_LOGW(TAG_PEER, "Failed to add drone peer");
        delayMicroseconds(100);  // Short delay for peer cleanup (was delay(10))
    }
    ESP_LOGI(TAG_PEER, "Success to add drone peer");
    // チャンネルの設定はESP-NOWの設定の前に行う必要があるかもしれない
    //esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    delayMicroseconds(100);  // Short delay for initialization (was delay(10))
}

void peering(void)
{
  uint8_t break_flag;
  uint32_t beep_delay = 0;

  if (M5.Btn.isPressed() || (Drone_mac[0] == 0xFF && Drone_mac[1] == 0xFF && Drone_mac[2] == 0xFF && Drone_mac[3] == 0xFF &&
                               Drone_mac[4] == 0xFF && Drone_mac[5] == 0xFF)) {
    M5.Lcd.println("Push LCD panel!");
    while (1) {
      M5.update();
      if (M5.Btn.wasPressed()) {
        is_peering = 1;
        break;
      }
    }
    ESP_LOGI(TAG_PEER, "Button pressed!");
    M5.Lcd.println(" ");
    M5.Lcd.println("Push StampFly");
    M5.Lcd.println("    Reset Button!");
    M5.Lcd.println(" ");
    M5.Lcd.println("Pairing...");

    //StampFlyはMACアドレスをFF:FF:FF:FF:FF:FFとして
    //StampFlyのMACアドレスをでブロードキャストする
    //その際にChannelが機体と送信機で同一でない場合は受け取れない
    //Wait receive StampFly MAC Address on the configured channel
    while(1)
    {
      for (uint8_t i =0;i<100;i++)
      {
            break_flag = 0;
            if (Received_flag == 1)
            {
              break_flag = 1;
              break;
            }
            usleep(100);
      }
      if (millis() - beep_delay >= 500) {
          beep();
          beep_delay = millis();
      }

      if (break_flag)break;
    }
    // Force Channel to be CHANNEL define value
    Channel = CHANNEL;
    save_data();
    is_peering = 0;
    ESP_LOGI(TAG_PEER, "Channel: %02d", Channel);
    ESP_LOGI(TAG_PEER, "StampFly MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             Drone_mac[0], Drone_mac[1], Drone_mac[2], Drone_mac[3], Drone_mac[4], Drone_mac[5]);
  }
}

void change_channel(uint8_t ch)
{
  #if 0
  dronePeer.channel = ch;
  if (esp_now_mod_peer(&dronePeer)!=ESP_OK)
  {
        ESP_LOGW(TAG_PEER, "Failed to modify peer");
        return;
  }
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  #endif
}

// hw_timer removed - using esp_timer (beacon_timer) for synchronization

void setup() {
  M5.begin();

  // Initialize ESP_LOG system
  esp_log_level_set("*", GLOBAL_LOG_LEVEL);  // Set global log level
  esp_log_level_set(TAG_MAIN, GLOBAL_LOG_LEVEL);
  esp_log_level_set(TAG_BEACON, GLOBAL_LOG_LEVEL);
  esp_log_level_set(TAG_DRONE, GLOBAL_LOG_LEVEL);
  esp_log_level_set(TAG_TDMA, GLOBAL_LOG_LEVEL);
  esp_log_level_set(TAG_PEER, GLOBAL_LOG_LEVEL);

  ESP_LOGI(TAG_MAIN, "StampFly Joy Controller starting...");

  Wire1.begin(38, 39, 400*1000);
  load_data();
  M5.update();
  setup_pwm_buzzer();
  M5.Lcd.setRotation( 2 );
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(4, 2);
  //0.5秒待つ
  delay(500);
  wifi_esp_now_init();

  // Broadcast peer needed for both master (beacon TX) and slave (pairing)
  broadcast_beacon_init();
  peering();
  drone_peer_init();
  
  M5.Lcd.fillScreen(BLACK);
  joy_update();

  StickMode = 2;
  if(getOptionButton())
  {
    StickMode = 3;
    M5.Lcd.println("Please release button.");
    while(getOptionButton())joy_update();
  }
  AltMode =NOT_ALT_CONTROL_MODE;
  delay(10);

  if (StickMode == 3)
  {
    THROTTLE = RIGHTY;
    AILERON = LEFTX;
    ELEVATOR = LEFTY;
    RUDDER = RIGHTX;
    ARM_BUTTON = RIGHT_STICK_BUTTON;
    FLIP_BUTTON = LEFT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }
  else
  {
    THROTTLE = LEFTY;
    AILERON = RIGHTX;
    ELEVATOR = RIGHTY;
    RUDDER = LEFTX;
    ARM_BUTTON = LEFT_STICK_BUTTON;
    FLIP_BUTTON = RIGHT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }

  uint8_t error, address;
  int nDevices;

////////////////////////////////////////////////////////
  ESP_LOGI(TAG_MAIN, "Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      ESP_LOGI(TAG_MAIN, "I2C device found at address 0x%02X !", address);
      nDevices++;
    }
    else if (error == 4)
    {
      ESP_LOGW(TAG_MAIN, "Unknown error at address 0x%02X", address);
    }
  }
  if (nDevices == 0)
    ESP_LOGI(TAG_MAIN, "No I2C devices found");
  else
    ESP_LOGI(TAG_MAIN, "I2C scan done");

  esp_now_get_version(&espnow_version);
  ESP_LOGI(TAG_MAIN, "ESP-NOW Version %d", espnow_version);

  // TDMA初期化 (hw_timer removed - using esp_timer only)
  beacon_sem = xSemaphoreCreateBinary();
  if (beacon_sem == NULL) {
    ESP_LOGE(TAG_TDMA, "Failed to create beacon semaphore");
  }

  // Create mutex for shared senddata protection
  senddata_mutex = xSemaphoreCreateMutex();
  if (senddata_mutex == NULL) {
    ESP_LOGE(TAG_TDMA, "Failed to create senddata mutex");
  } else {
    ESP_LOGI(TAG_TDMA, "Senddata mutex created");
  }

  // Initialize shared_senddata with safe default values (before task starts)
  // This prevents sending uninitialized data in the first frames
  memset(shared_senddata, 0, sizeof(shared_senddata));
  shared_senddata[0] = dronePeer.peer_addr[3];  // Drone MAC address (lower 3 bytes)
  shared_senddata[1] = dronePeer.peer_addr[4];
  shared_senddata[2] = dronePeer.peer_addr[5];
  // senddata[3-10]: Throttle, Phi, Theta, Psi = 0 (already set by memset)
  // senddata[11]: flags = 0 (Arm=0, Flip=0, Mode=0, AltMode=0)
  // senddata[12]: proactive_flag = 0
  // Calculate checksum
  shared_senddata[13] = 0;
  for (uint8_t i = 0; i < 13; i++) {
    shared_senddata[13] += shared_senddata[i];
  }
  ESP_LOGI(TAG_TDMA, "Shared senddata initialized with default values");

  // TDMA timer setup
  esp_timer_create_args_t beacon_timer_args;
  beacon_timer_args.callback = &beacon_timer_callback;
  beacon_timer_args.arg = NULL;
  beacon_timer_args.dispatch_method = ESP_TIMER_TASK;
  beacon_timer_args.name = "beacon_timer";

  esp_err_t err = esp_timer_create(&beacon_timer_args, &beacon_timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG_TDMA, "Failed to create TDMA timer: %d", err);
  }

  // Initialize frame start time
  frame_start_time_us = esp_timer_get_time();

  // Create beacon transmission task BEFORE starting timer (master only)
  // This ensures beacon_task_handle is set when timer callback fires
  if (TDMA_DEVICE_ID == 0) {
    BaseType_t task_result = xTaskCreatePinnedToCore(
        beacon_task,           // Task function
        "BeaconTask",          // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Task parameter
        configMAX_PRIORITIES - 1,  // High priority for precise timing
        &beacon_task_handle,   // Task handle
        1                      // Core 1 (Arduino loop runs on core 1)
    );

    if (task_result != pdPASS) {
      ESP_LOGE(TAG_TDMA, "Failed to create beacon task");
      beacon_task_handle = NULL;  // Ensure it's NULL on failure
    } else {
      ESP_LOGI(TAG_TDMA, "Beacon task created (handle=%p)", beacon_task_handle);
    }
  }

  // Create TDMA send task (both master and slave need this)
  BaseType_t tdma_task_result = xTaskCreatePinnedToCore(
      tdma_send_task,              // Task function
      "TDMASendTask",              // Task name
      8192,                        // Stack size (bytes)
      NULL,                        // Task parameter
      configMAX_PRIORITIES - 1,    // Highest priority for precise timing
      &tdma_send_task_handle,      // Task handle
      1                            // Core 1 (same as loop and beacon task)
  );

  if (tdma_task_result != pdPASS) {
    ESP_LOGE(TAG_TDMA, "Failed to create TDMA send task");
    tdma_send_task_handle = NULL;
  } else {
    ESP_LOGI(TAG_TDMA, "TDMA send task created (handle=%p)", tdma_send_task_handle);
  }

  // Create input mutex for shared input data protection
  input_mutex = xSemaphoreCreateMutex();
  if (input_mutex == NULL) {
    ESP_LOGE(TAG_MAIN, "Failed to create input mutex");
  } else {
    ESP_LOGI(TAG_MAIN, "Input mutex created");
  }

  // Create input task (Phase 2: Sensor reading separation)
  BaseType_t input_task_result = xTaskCreatePinnedToCore(
      input_task,                  // Task function
      "InputTask",                 // Task name
      4096,                        // Stack size (bytes)
      NULL,                        // Task parameter
      configMAX_PRIORITIES - 2,    // High priority (lower than TDMA)
      &input_task_handle,          // Task handle
      1                            // Core 1 (same as loop)
  );

  if (input_task_result != pdPASS) {
    ESP_LOGE(TAG_MAIN, "Failed to create input task");
    input_task_handle = NULL;
  } else {
    ESP_LOGI(TAG_MAIN, "Input task created (handle=%p)", input_task_handle);
  }

  // Wait for system stabilization before starting TDMA timer
  // This prevents transient timing errors during system startup
  ESP_LOGI(TAG_TDMA, "Waiting for system stabilization (200ms)...");
  delay(200);

  // Start TDMA timer AFTER creating beacon task (both master and slave use it for synchronization)
  err = esp_timer_start_periodic(beacon_timer, TDMA_FRAME_US);
  if (err != ESP_OK) {
    ESP_LOGE(TAG_TDMA, "Failed to start TDMA timer: %d", err);
  } else {
    if (TDMA_DEVICE_ID == 0) {
      ESP_LOGI(TAG_TDMA, "TDMA Master timer started (ID=%d, period=%d us)", TDMA_DEVICE_ID, TDMA_FRAME_US);
    } else {
      ESP_LOGI(TAG_TDMA, "TDMA Slave timer started (ID=%d, period=%d us)", TDMA_DEVICE_ID, TDMA_FRAME_US);
    }
  }

  ESP_LOGI(TAG_TDMA, "TDMA initialized. Device ID=%d, Channel=%d", TDMA_DEVICE_ID, CHANNEL);

  // Set runtime log levels to INFO (disable DEBUG output to avoid timing delays)
  esp_log_level_set("*", GLOBAL_LOG_LEVEL);           // Default for all tags
  esp_log_level_set("TDMA", GLOBAL_LOG_LEVEL);
  esp_log_level_set("DRONE", GLOBAL_LOG_LEVEL);
  esp_log_level_set("INPUT", GLOBAL_LOG_LEVEL);
  esp_log_level_set("MAIN", GLOBAL_LOG_LEVEL);
  esp_log_level_set("BEACON", GLOBAL_LOG_LEVEL);
  esp_log_level_set("PEER", GLOBAL_LOG_LEVEL);
  ESP_LOGI(TAG_MAIN, "Log level set to %s", GLOBAL_LOG_LEVEL == ESP_LOG_INFO ? "INFO" : "DEBUG");
}

uint8_t check_control_mode_change(void)
{
  uint8_t state = 0;
  static uint8_t button_state = 0;

  if (getModeButton() == 1 && button_state == 0) {
    state = 1;
    button_state = 1;
  } else if (getModeButton() == 0 && button_state == 1) {
    button_state = 0;
  }

  return state;
}

uint8_t check_alt_mode_change(void)
{
  uint8_t state = 0;
  static uint8_t button_state = 0;

  if (getOptionButton() == 1 && button_state == 0) {
    state = 1;
    button_state = 1;
  } else if (getOptionButton() == 0 && button_state == 1) {
    button_state = 0;
  }

  return state;
}

uint8_t average_counter = 0;

void loop() {
  int16_t _throttle;
  int16_t _phi;
  int16_t _theta;
  int16_t _psi;
  static uint8_t loop_counter = 0;

  // ============================================================================
  // Loop processing: Read input data from input_task, prepare control data for TDMA task
  // Input reading is handled by dedicated task (input_task)
  // TDMA transmission is handled by dedicated task (tdma_send_task)
  // ============================================================================
  etime = stime;
  stime = micros();
  dtime = stime - etime;
  loop_counter++;

  // Read input data from shared buffer (mutex-protected)
  struct InputData local_input;
  if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    memcpy(&local_input, &shared_inputdata, sizeof(InputData));
    xSemaphoreGive(input_mutex);
  } else {
    // Mutex timeout - use previous values (or zero)
    static uint32_t loop_mutex_timeout_count = 0;
    loop_mutex_timeout_count++;
    if (loop_mutex_timeout_count % 100 == 1) {
      ESP_LOGW(TAG_MAIN, "Input mutex timeout in loop() #%u", loop_mutex_timeout_count);
      beep_mutex_timeout();  // 2000Hz triple beep = mutex timeout
    }
    memset(&local_input, 0, sizeof(InputData));
  }

  // Process button events from input task
  if(local_input.btn_pressed == true)
  {
    if (Timer_state == 0)Timer_state = 1;
    else if (Timer_state == 1)Timer_state = 0;
  }

  if(local_input.btn_long_pressed == true)
  {
    Timer_state = 2;
  }

  if (Timer_state == 1)
  {
    //カウントアップ
    Timer = Timer + dTime;
  }
  else if (Timer_state == 2)
  {
    //タイマリセット
    Timer = 0.0;
    Timer_state = 0;
  }

  // Process mode changes from input task
  if (local_input.mode_changed == 1)
  {
    if (Mode==ANGLECONTROL)Mode=RATECONTROL;
    else Mode = ANGLECONTROL;
    ESP_LOGI(TAG_MAIN, "Mode toggled to: %s", Mode == ANGLECONTROL ? "STABILIZE" : "ACRO");

    // Clear the flag after processing to prevent repeated toggling
    if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      shared_inputdata.mode_changed = 0;
      xSemaphoreGive(input_mutex);
    }
  }

  if (local_input.alt_mode_changed == 1)
  {
    if (AltMode==ALT_CONTROL_MODE)AltMode=NOT_ALT_CONTROL_MODE;
    else AltMode = ALT_CONTROL_MODE;
    ESP_LOGI(TAG_MAIN, "AltMode toggled to: %s", AltMode == ALT_CONTROL_MODE ? "Auto ALT" : "Manual ALT");

    // Clear the flag after processing to prevent repeated toggling
    if (xSemaphoreTake(input_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      shared_inputdata.alt_mode_changed = 0;
      xSemaphoreGive(input_mutex);
    }
  }

  // Use raw values from input task
  _throttle = local_input.throttle_raw;
  _phi = local_input.phi_raw;
  _theta = local_input.theta_raw;
  _psi = local_input.psi_raw;

  if(average_counter<50)
  {
    average_counter++;
    Throttle_bias += (_throttle - 2048);
    Phi_bias += (_phi - 2048);
    Theta_bias += (_theta - 2048);
    Psi_bias += (_psi - 2048);
  }
  else if (average_counter==50)
  {
    average_counter++;
    Throttle_bias = Throttle_bias/50;
    Phi_bias = Phi_bias/50;
    Theta_bias = Theta_bias/50;
    Psi_bias = Psi_bias/50;
    average_counter=51;
  }
  else
  {
    if(_throttle < Throttle_bias) _throttle=0; else _throttle -= Throttle_bias;
    if(_phi < Phi_bias) _phi=0; else _phi -= Phi_bias;
    if(_theta < Theta_bias) _theta=0; else _theta -= Theta_bias;
    if(_psi < Psi_bias) _psi=0; else _psi -= Psi_bias;
  }

  //量産版
  //Throttle = -(float)(_throttle - 2048)/(float)(RESO10BIT*0.5);
  //Phi =       (float)(_phi - 2048)/(float)(RESO10BIT*0.5); 
  //Theta =     (float)(_theta - 2048)/(float)(RESO10BIT*0.5);
  //Psi =       (float)(_psi - 2048)/(float)(RESO10BIT*0.5);
  Throttle = 4095 - _throttle;
  Phi = _phi; 
  Theta = _theta;
  Psi = _psi;


  //最終試作版
  #if 0
  Throttle = (float)(_throttle - 2048)/(float)(RESO10BIT*0.5);
  Phi =      (float)(_phi - 2048)/(float)(RESO10BIT*0.5); 
  Theta =   -(float)(_theta - 2048)/(float)(RESO10BIT*0.5);
  Psi =      (float)(_psi - 2048)/(float)(RESO10BIT*0.5);
  #endif

  //Throttle = limit(Throttle, -1.0, 1.0);
  //Phi = limit(Phi, -1.0, 1.0);
  //Theta = limit(Theta, -1.0, 1.0);
  //Psi = limit(Psi, -1.0, 1.0);


  uint8_t* d_int;
  
  //ブロードキャストの混信を防止するためこの機体のMACアドレスに送られてきたものか判断する
  senddata[0] = dronePeer.peer_addr[3];////////////////////////////
  senddata[1] = dronePeer.peer_addr[4];////////////////////////////
  senddata[2] = dronePeer.peer_addr[5];////////////////////////////

  d_int = (uint8_t*)&Throttle;
  senddata[3]=d_int[0];
  senddata[4]=d_int[1];

  d_int = (uint8_t*)&Phi;
  senddata[5]=d_int[0];
  senddata[6]=d_int[1];

  d_int = (uint8_t*)&Theta;
  senddata[7]=d_int[0];
  senddata[8]=d_int[1];

  d_int = (uint8_t*)&Psi;
  senddata[9]=d_int[0];
  senddata[10]=d_int[1];

  senddata[11]=(0x01&AltMode)<<3|(0x01&Mode)<<2|(0x0001&getFlipButton())<<1|(0x0001&getArmButton());
  //senddata[11]=getArmButton();
  //senddata[12]=getFlipButton();
  //senddata[13]=Mode;
  //senddata[14]=AltMode;
  senddata[12]=proactive_flag;
  
  //checksum
  senddata[13]=0;
  for(uint8_t i=0;i<13;i++)senddata[13]=senddata[13]+senddata[i];

  // Update shared buffer for TDMA send task (mutex-protected)
  if (xSemaphoreTake(senddata_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    memcpy(shared_senddata, senddata, 14);
    xSemaphoreGive(senddata_mutex);
  } else {
    // Mutex timeout - should never happen in normal operation
    static uint32_t mutex_timeout_count = 0;
    mutex_timeout_count++;
    if (mutex_timeout_count % 100 == 1) {
      ESP_LOGW(TAG_TDMA, "Mutex timeout in loop() #%u", mutex_timeout_count);
      beep_mutex_timeout();  // 2000Hz triple beep = mutex timeout
    }
  }

  // Check for beacon loss (slave only)
  static uint32_t last_beep_time = 0;
  if (TDMA_DEVICE_ID != 0 && first_beacon_received) {
    int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
    if (time_since_beacon > BEACON_TIMEOUT_US) {
      // Beacon lost - beep every 500ms (4000Hz high pitch = urgent)
      uint32_t current_millis = millis();
      if (current_millis - last_beep_time >= 500) {
        beep_beacon_loss();
        last_beep_time = current_millis;
      }
    }
  }

  // TDMA transmission is now handled by dedicated task (tdma_send_task)
  // This loop only prepares data and updates shared_senddata buffer
  #ifdef DEBUG
  ESP_LOGD(TAG_DRONE, "Drone MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           dronePeer.peer_addr[0],
           dronePeer.peer_addr[1],
           dronePeer.peer_addr[2],
           dronePeer.peer_addr[3],
           dronePeer.peer_addr[4],
           dronePeer.peer_addr[5]);
  #endif

  
  //Display information
  //float vbat =0.0;// M5.Axp.GetBatVoltage();
  //int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(4, 2+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("MAC ADR %02X:%02X    ", dronePeer.peer_addr[4],dronePeer.peer_addr[5]);
      break;
    case 1:
      M5.Lcd.printf("BAT 1:%4.1f 2:%4.1f", Battery_voltage[0],Battery_voltage[1]);
      //M5.Lcd.printf("X:%4d",xstick);
      break;
    case 2:
      #ifdef NEW_ATOM_JOY
      M5.Lcd.printf("MODE: %d", StickMode);
      //M5.Lcd.printf("X:%4d",xstick);
      #endif
      break;
    case 3:
      M5.Lcd.printf("CH: %02d ID: %d  ",dronePeer.channel, TDMA_DEVICE_ID);
      break;
    case 4:
      if( AltMode == ALT_CONTROL_MODE ) M5.Lcd.printf("-Auto ALT-  ");
      else if ( AltMode == NOT_ALT_CONTROL_MODE )   M5.Lcd.printf("-Mnual ALT- ");
      break;
    case 5:
      if( Mode == ANGLECONTROL )      M5.Lcd.printf("-STABILIZE-");
      else if ( Mode == RATECONTROL ) M5.Lcd.printf("-ACRO-     ");
      break;
    case 6:
      //M5.Lcd.printf("Time:%7.2f",Timer);
      #if TDMA_DEVICE_ID == 0
        // Master device - show actual measured transmission frequency and role
        M5.Lcd.printf("Freq:%4d M[%3d]", (int)actual_send_freq_hz, loop_counter);
      #else
        // Slave device - show actual measured transmission frequency and PLL sync error or beacon loss
        if (first_beacon_received) {
          int64_t time_since_beacon = esp_timer_get_time() - last_beacon_time_us;
          if (time_since_beacon > BEACON_TIMEOUT_US) {
            M5.Lcd.printf("F:%4d LOST!  ", (int)actual_send_freq_hz);
          } else {
            M5.Lcd.printf("F:%4d E:%+4d  ", (int)actual_send_freq_hz, (int)pll_error_us);
          }
        } else {
          M5.Lcd.printf("F:%4d WAIT   ", (int)actual_send_freq_hz);
        }
      #endif
      break;
    case 8:
      break;
    case 9:
      break;
  }
  disp_counter++;
  if(disp_counter==11)disp_counter=0;

  //Reset
  if( /*M5.Axp.GetBtnPress() == 2*/ 0 ){
    // 電源ボタンクリック
    //M5.Lcd.println("AtomFly2.0"); 
    esp_restart();
  } 

}

void show_battery_info(){
  #if 0
  // バッテリー電圧表示
  double vbat = 0.0;
  int8_t bat_charge_p = 0;

  vbat = M5.Axp.GetBatVoltage();
  M5.Lcd.setCursor(5, 100);
  //M5.Lcd.setTextSize(1);
  M5.Lcd.printf("Volt:\n %8.2fV", vbat);

  // バッテリー残量表示
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.printf("Charge:\n %8d%%", bat_charge_p);
#endif
}

void voltage_print(void)
{
  M5.Lcd.setCursor(0, 17, 2);
  M5.Lcd.printf("%3.1fV", Battery_voltage);
}
