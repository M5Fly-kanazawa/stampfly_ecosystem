/**
 * @file control_arbiter.cpp
 * @brief Control input arbiter implementation
 *        制御入力アービター実装
 */

#include "control_arbiter.hpp"
#include "system_state.hpp"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = "ControlArbiter";

namespace stampfly {

ControlArbiter& ControlArbiter::getInstance() {
    static ControlArbiter instance;
    return instance;
}

esp_err_t ControlArbiter::init() {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Create mutex
    // ミューテックスを作成
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize inputs to neutral
    // 入力をニュートラルに初期化
    espnow_input_ = {};
    espnow_input_.throttle = 0.0f;
    espnow_input_.roll = 0.0f;
    espnow_input_.pitch = 0.0f;
    espnow_input_.yaw = 0.0f;
    espnow_input_.source = ControlSource::NONE;

    udp_input_ = espnow_input_;
    ws_input_ = espnow_input_;

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized (mode: %s, timeout: %lums)",
             getCommModeName(comm_mode_), timeout_ms_);

    return ESP_OK;
}

void ControlArbiter::setCommMode(CommMode mode) {
    if (mode == comm_mode_) {
        return;
    }

    comm_mode_ = mode;
    ESP_LOGI(TAG, "Communication mode changed to: %s", getCommModeName(mode));
}

const char* ControlArbiter::getCommModeName(CommMode mode) {
    switch (mode) {
        case CommMode::ESPNOW: return "ESP-NOW";
        case CommMode::UDP:    return "UDP";
        default:               return "Unknown";
    }
}

void ControlArbiter::updateFromESPNOW(uint16_t throttle, uint16_t roll,
                                       uint16_t pitch, uint16_t yaw, uint8_t flags) {
    if (!initialized_) return;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        espnow_input_.throttle = normalizeThrottle(throttle);
        espnow_input_.roll = normalizeAxis(roll);
        espnow_input_.pitch = normalizeAxis(pitch);
        espnow_input_.yaw = normalizeAxis(yaw);
        espnow_input_.flags = flags;
        espnow_input_.source = ControlSource::ESPNOW;
        espnow_input_.timestamp_ms = getCurrentTimeMs();
        espnow_count_++;
        xSemaphoreGive(mutex_);

        // Update SystemStateManager with active control source
        // SystemStateManagerにアクティブ制御ソースを通知
        SystemStateManager::getInstance().updateControlSource(ControlSource::ESPNOW);
    }
}

void ControlArbiter::updateFromUDP(uint16_t throttle, uint16_t roll,
                                    uint16_t pitch, uint16_t yaw, uint8_t flags) {
    if (!initialized_) return;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        udp_input_.throttle = normalizeThrottle(throttle);
        udp_input_.roll = normalizeAxis(roll);
        udp_input_.pitch = normalizeAxis(pitch);
        udp_input_.yaw = normalizeAxis(yaw);
        udp_input_.flags = flags;
        udp_input_.source = ControlSource::UDP;
        udp_input_.timestamp_ms = getCurrentTimeMs();
        udp_count_++;
        xSemaphoreGive(mutex_);

        // Update SystemStateManager with active control source
        // SystemStateManagerにアクティブ制御ソースを通知
        SystemStateManager::getInstance().updateControlSource(ControlSource::UDP);
    }
}

void ControlArbiter::updateFromWebSocket(float throttle, float roll,
                                          float pitch, float yaw, uint8_t flags) {
    if (!initialized_) return;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        ws_input_.throttle = std::clamp(throttle, 0.0f, 1.0f);
        ws_input_.roll = std::clamp(roll, -1.0f, 1.0f);
        ws_input_.pitch = std::clamp(pitch, -1.0f, 1.0f);
        ws_input_.yaw = std::clamp(yaw, -1.0f, 1.0f);
        ws_input_.flags = flags;
        ws_input_.source = ControlSource::WEBSOCKET;
        ws_input_.timestamp_ms = getCurrentTimeMs();
        ws_count_++;
        xSemaphoreGive(mutex_);

        // Update SystemStateManager with active control source
        // SystemStateManagerにアクティブ制御ソースを通知
        SystemStateManager::getInstance().updateControlSource(ControlSource::WEBSOCKET);
    }
}

bool ControlArbiter::getActiveControl(ControlInput& input) const {
    if (!initialized_) {
        return false;
    }

    uint32_t now = getCurrentTimeMs();
    bool found = false;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Get input based on communication mode
        // 通信モードに基づいて入力を取得
        const ControlInput* active_input = nullptr;

        // WebSocket has highest priority (WiFi CLI commands, always active)
        // WebSocketは最高優先度（WiFi CLIコマンド、常にアクティブ）
        if (ws_input_.source == ControlSource::WEBSOCKET &&
            (now - ws_input_.timestamp_ms) < timeout_ms_) {
            active_input = &ws_input_;
        }
        // Otherwise, use communication mode
        // それ以外は通信モードに従う
        else {
            switch (comm_mode_) {
                case CommMode::ESPNOW:
                    if (espnow_input_.source == ControlSource::ESPNOW &&
                        (now - espnow_input_.timestamp_ms) < timeout_ms_) {
                        active_input = &espnow_input_;
                    }
                    break;

                case CommMode::UDP:
                    if (udp_input_.source == ControlSource::UDP &&
                        (now - udp_input_.timestamp_ms) < timeout_ms_) {
                        active_input = &udp_input_;
                    }
                    break;
            }
        }

        if (active_input != nullptr) {
            input = *active_input;
            found = true;
        }

        xSemaphoreGive(mutex_);
    }

    return found;
}

ControlSource ControlArbiter::getActiveSource() const {
    if (!initialized_) {
        return ControlSource::NONE;
    }

    uint32_t now = getCurrentTimeMs();
    ControlSource source = ControlSource::NONE;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        // WebSocket has highest priority (always active)
        // WebSocketは最高優先度（常にアクティブ）
        if (ws_input_.source == ControlSource::WEBSOCKET &&
            (now - ws_input_.timestamp_ms) < timeout_ms_) {
            source = ControlSource::WEBSOCKET;
        }
        // Otherwise, use communication mode
        // それ以外は通信モードに従う
        else {
            switch (comm_mode_) {
                case CommMode::ESPNOW:
                    if (espnow_input_.source == ControlSource::ESPNOW &&
                        (now - espnow_input_.timestamp_ms) < timeout_ms_) {
                        source = ControlSource::ESPNOW;
                    }
                    break;

                case CommMode::UDP:
                    if (udp_input_.source == ControlSource::UDP &&
                        (now - udp_input_.timestamp_ms) < timeout_ms_) {
                        source = ControlSource::UDP;
                    }
                    break;
            }
        }
        xSemaphoreGive(mutex_);
    }

    return source;
}

bool ControlArbiter::hasActiveControl() const {
    return getActiveSource() != ControlSource::NONE;
}

uint32_t ControlArbiter::getTimeSinceLastInput() const {
    if (!initialized_) {
        return UINT32_MAX;
    }

    uint32_t now = getCurrentTimeMs();
    uint32_t min_time = UINT32_MAX;

    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        // WebSocket is always checked (highest priority)
        // WebSocketは常にチェック（最高優先度）
        if (ws_input_.source == ControlSource::WEBSOCKET) {
            min_time = std::min(min_time, now - ws_input_.timestamp_ms);
        }

        // Also check communication mode sources
        // 通信モードのソースもチェック
        switch (comm_mode_) {
            case CommMode::ESPNOW:
                if (espnow_input_.source == ControlSource::ESPNOW) {
                    min_time = std::min(min_time, now - espnow_input_.timestamp_ms);
                }
                break;

            case CommMode::UDP:
                if (udp_input_.source == ControlSource::UDP) {
                    min_time = std::min(min_time, now - udp_input_.timestamp_ms);
                }
                break;
        }
        xSemaphoreGive(mutex_);
    }

    return min_time;
}

void ControlArbiter::resetStats() {
    espnow_count_ = 0;
    udp_count_ = 0;
    ws_count_ = 0;
}

float ControlArbiter::normalizeThrottle(uint16_t raw) {
    // Throttle: 0-4095 → 0.0-1.0
    // Use upper half only: (raw - 2048) / 2048, clamped to [0, 1]
    // スロットル: 0-4095 → 0.0-1.0
    // 上半分のみ使用: (raw - 2048) / 2048、[0, 1]にクランプ
    float normalized = static_cast<float>(static_cast<int>(raw) - ADC_CENTER) / ADC_CENTER;
    return std::clamp(normalized, 0.0f, 1.0f);
}

float ControlArbiter::normalizeAxis(uint16_t raw) {
    // Axis: 0-4095, 2048=center → -1.0 to +1.0
    // 軸: 0-4095、2048=中央 → -1.0 ～ +1.0
    float normalized = static_cast<float>(static_cast<int>(raw) - ADC_CENTER) / ADC_CENTER;
    return std::clamp(normalized, -1.0f, 1.0f);
}

uint32_t ControlArbiter::getCurrentTimeMs() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

bool ControlArbiter::isSourceAllowed(ControlSource source) const {
    // WebSocket is always allowed (WiFi CLI commands)
    // WebSocketは常に許可（WiFi CLIコマンド）
    if (source == ControlSource::WEBSOCKET) {
        return true;
    }

    // Other sources depend on communication mode
    // その他のソースは通信モードに依存
    switch (comm_mode_) {
        case CommMode::ESPNOW:
            return source == ControlSource::ESPNOW;
        case CommMode::UDP:
            return source == ControlSource::UDP;
        default:
            return false;
    }
}

}  // namespace stampfly
