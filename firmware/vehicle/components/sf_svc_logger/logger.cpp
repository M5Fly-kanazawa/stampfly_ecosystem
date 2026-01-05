/**
 * @file logger.cpp
 * @brief High-precision binary logger implementation
 */

#include "logger.hpp"
#include <cstdio>
#include <cstring>
#include "esp_log.h"

static const char* TAG = "Logger";

namespace stampfly {

Logger::~Logger() {
    stop();

    if (timer_) {
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }

    if (writer_task_) {
        vTaskDelete(writer_task_);
        writer_task_ = nullptr;
    }

    if (queue_) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }

    if (write_sem_) {
        vSemaphoreDelete(write_sem_);
        write_sem_ = nullptr;
    }
}

esp_err_t Logger::init(uint32_t frequency_hz, size_t queue_size) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate frequency
    if (frequency_hz < MIN_FREQUENCY_HZ || frequency_hz > MAX_FREQUENCY_HZ) {
        ESP_LOGE(TAG, "Invalid frequency: %lu (must be %lu-%lu Hz)",
                 frequency_hz, MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ);
        return ESP_ERR_INVALID_ARG;
    }

    frequency_hz_ = frequency_hz;

    // Create queue
    queue_ = xQueueCreate(queue_size, sizeof(LogPacket));
    if (!queue_) {
        ESP_LOGE(TAG, "Failed to create queue");
        return ESP_ERR_NO_MEM;
    }

    // Create binary semaphore for timer->task synchronization
    write_sem_ = xSemaphoreCreateBinary();
    if (!write_sem_) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        vQueueDelete(queue_);
        queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Create writer task on Core 0 (ESKF runs on Core 1)
    BaseType_t ret = xTaskCreatePinnedToCore(
        writerTask,
        "log_writer",
        WRITER_TASK_STACK_SIZE,
        this,
        WRITER_TASK_PRIORITY,
        &writer_task_,
        0  // Core 0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create writer task");
        vSemaphoreDelete(write_sem_);
        write_sem_ = nullptr;
        vQueueDelete(queue_);
        queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // Create ESP Timer
    esp_timer_create_args_t timer_args = {
        .callback = timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "logger_timer",
        .skip_unhandled_events = true
    };

    esp_err_t err = esp_timer_create(&timer_args, &timer_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        vTaskDelete(writer_task_);
        writer_task_ = nullptr;
        vSemaphoreDelete(write_sem_);
        write_sem_ = nullptr;
        vQueueDelete(queue_);
        queue_ = nullptr;
        return err;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialized at %lu Hz, queue size=%d", frequency_hz_, queue_size);

    return ESP_OK;
}

void Logger::start() {
    if (!initialized_ || running_) {
        return;
    }

    // Clear queue before starting
    xQueueReset(queue_);

    // Call start callback if set
    if (start_callback_) {
        start_callback_();
    }

    // Calculate period in microseconds
    uint64_t period_us = 1000000 / frequency_hz_;

    // Start periodic timer
    esp_err_t err = esp_timer_start_periodic(timer_, period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
        return;
    }

    running_ = true;
    ESP_LOGI(TAG, "Started logging at %lu Hz (period=%llu us)", frequency_hz_, period_us);
}

void Logger::stop() {
    if (!initialized_ || !running_) {
        return;
    }

    // Stop timer
    esp_timer_stop(timer_);

    running_ = false;
    ESP_LOGI(TAG, "Stopped logging, count=%lu, dropped=%lu", log_count_, drop_count_);
}

bool Logger::pushData(const LogPacket& packet) {
    if (!initialized_ || !running_) {
        return false;
    }

    // Non-blocking queue send
    BaseType_t ret = xQueueSend(queue_, &packet, 0);
    if (ret != pdTRUE) {
        drop_count_++;
        return false;
    }

    return true;
}

esp_err_t Logger::setFrequency(uint32_t frequency_hz) {
    if (frequency_hz < MIN_FREQUENCY_HZ || frequency_hz > MAX_FREQUENCY_HZ) {
        ESP_LOGE(TAG, "Invalid frequency: %lu (must be %lu-%lu Hz)",
                 frequency_hz, MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ);
        return ESP_ERR_INVALID_ARG;
    }

    bool was_running = running_;

    if (was_running) {
        stop();
    }

    frequency_hz_ = frequency_hz;
    ESP_LOGI(TAG, "Frequency set to %lu Hz", frequency_hz_);

    if (was_running) {
        start();
    }

    return ESP_OK;
}

void Logger::timerCallback(void* arg) {
    Logger* self = static_cast<Logger*>(arg);

    // Give semaphore to wake up writer task
    // Note: ESP Timer callback runs in timer task context, not ISR
    xSemaphoreGive(self->write_sem_);
}

void Logger::writerTask(void* arg) {
    Logger* self = static_cast<Logger*>(arg);
    LogPacket pkt;

    ESP_LOGI(TAG, "Writer task started");

    while (true) {
        // Wait for timer notification
        if (xSemaphoreTake(self->write_sem_, portMAX_DELAY) == pdTRUE) {
            // Get latest data from queue (non-blocking)
            if (xQueueReceive(self->queue_, &pkt, 0) == pdTRUE) {
                // Calculate and set checksum
                pkt.checksum = self->calculateChecksum(pkt);

                // Write to USB (blocking is OK here)
                fwrite(&pkt, sizeof(pkt), 1, stdout);
                fflush(stdout);

                self->log_count_++;
            }
        }
    }
}

uint8_t Logger::calculateChecksum(const LogPacket& pkt) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t checksum = 0;

    // XOR bytes 2 to 126 (skip header[0], header[1], and checksum itself)
    for (size_t i = 2; i < sizeof(LogPacket) - 1; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

}  // namespace stampfly
