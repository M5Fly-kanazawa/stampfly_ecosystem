/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Kouhei Ito
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

/**
 * @file bmi270_spi.c
 * @brief BMI270 SPI communication layer
 *
 * This file implements the SPI communication functions for BMI270.
 * Key points:
 * - READ operations require 3-byte transaction (CMD + Dummy + Data)
 * - WRITE operations require 2-byte transaction (CMD + Data)
 * - Proper timing delays must be observed
 */

#include <string.h>
#include "bmi270_defs.h"
#include "bmi270_types.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

static const char *TAG = "BMI270_SPI";

// Experimental: Low-power mode delay override (0 = use default from bmi270_defs.h)
static uint32_t g_lowpower_delay_override = 0;

/**
 * @brief Initialize SPI bus and add BMI270 device
 *
 * @param dev Pointer to BMI270 device structure
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_spi_init(bmi270_dev_t *dev, const bmi270_config_t *config) {
    if (dev == NULL || config == NULL) {
        ESP_LOGE(TAG, "NULL pointer passed to bmi270_spi_init");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // Deactivate other devices on shared SPI bus (if specified)
    if (config->gpio_other_cs >= 0) {
        ESP_LOGI(TAG, "Deactivating other SPI device on GPIO%d (shared bus)", config->gpio_other_cs);
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->gpio_other_cs),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure other CS GPIO: %s", esp_err_to_name(ret));
            return ret;
        }
        // Set CS HIGH (inactive) for other device
        gpio_set_level(config->gpio_other_cs, 1);
        ESP_LOGI(TAG, "Other SPI device CS set to HIGH (inactive)");
    }

    // Store GPIO pins
    dev->gpio_mosi = config->gpio_mosi;
    dev->gpio_miso = config->gpio_miso;
    dev->gpio_sclk = config->gpio_sclk;
    dev->gpio_cs = config->gpio_cs;
    dev->spi_clock_hz = config->spi_clock_hz;
    dev->initialized = false;

    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = config->gpio_mosi,
        .miso_io_num = config->gpio_miso,
        .sclk_io_num = config->gpio_sclk,
        .quadwp_io_num = -1,  // Not used
        .quadhd_io_num = -1,  // Not used
        .max_transfer_sz = BMI270_CONFIG_FILE_SIZE + 2,  // For config file + header
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    // Initialize SPI bus with DMA
    ret = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI bus initialized on host %d", config->spi_host);

    // Configure BMI270 device
    spi_device_interface_config_t dev_config = {
        .mode = 0,  // SPI Mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = config->spi_clock_hz,
        .spics_io_num = config->gpio_cs,
        .queue_size = 7,  // Transaction queue size
        .flags = 0,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    // Add device to SPI bus
    ret = spi_bus_add_device(config->spi_host, &dev_config, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BMI270 to SPI bus: %s", esp_err_to_name(ret));
        spi_bus_free(config->spi_host);
        return ret;
    }

    ESP_LOGI(TAG, "BMI270 SPI device added successfully");
    ESP_LOGI(TAG, "GPIO - MOSI:%d MISO:%d SCLK:%d CS:%d",
             config->gpio_mosi, config->gpio_miso,
             config->gpio_sclk, config->gpio_cs);
    ESP_LOGI(TAG, "SPI Clock: %lu Hz", config->spi_clock_hz);

    dev->initialized = true;
    dev->init_complete = false;  // BMI270 initialization not yet complete (low-power mode)
    return ESP_OK;
}

/**
 * @brief Read single register from BMI270 (3-byte transaction)
 *
 * BMI270 SPI Read Protocol:
 *   TX: [CMD: R/W=1 + Addr] [Dummy] [Dummy]
 *   RX: [Echo]              [Dummy] [DATA]  <- Byte 3 is valid data
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Register address (0x00-0x7F)
 * @param data Pointer to store read data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_read_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data) {
    if (dev == NULL || data == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_read_register");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 3-byte transaction buffers
    uint8_t tx_data[3] = {
        reg_addr | BMI270_SPI_READ_BIT,  // Byte 1: Read command (bit7=1)
        0x00,                             // Byte 2: Dummy (TX side)
        0x00                              // Byte 3: Dummy (TX side)
    };
    uint8_t rx_data[3] = {0};

    spi_transaction_t trans = {
        .flags = 0,
        .length = 3 * 8,      // 3 bytes = 24 bits
        .rxlength = 3 * 8,    // Receive 3 bytes
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .user = NULL,
    };

    // Execute transaction (polling mode for reliability)
    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    if (ret == ESP_OK) {
        // DEBUG: Log all received bytes for troubleshooting
        ESP_LOGD(TAG, "Read reg 0x%02X: rx[0]=0x%02X rx[1]=0x%02X rx[2]=0x%02X",
                 reg_addr, rx_data[0], rx_data[1], rx_data[2]);

        // rx_data[0] = Command echo (discard)
        // rx_data[1] = Dummy (discard)
        *data = rx_data[2];  // ★ Byte 3 is valid data

        // Wait after read (timing depends on initialization state)
        if (dev->init_complete) {
            // Normal mode: 2µs
            esp_rom_delay_us(BMI270_DELAY_WRITE_NORMAL_US);
        } else {
            // Low-power mode: use override if set, otherwise use default
            uint32_t delay = (g_lowpower_delay_override > 0) ? g_lowpower_delay_override : BMI270_DELAY_ACCESS_LOWPOWER_US;
            esp_rom_delay_us(delay);
        }
    } else {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Write single register to BMI270 (2-byte transaction)
 *
 * BMI270 SPI Write Protocol:
 *   TX: [CMD: R/W=0 + Addr] [DATA]
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Register address (0x00-0x7F)
 * @param data Data to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bmi270_write_register(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t data) {
    if (dev == NULL) {
        ESP_LOGE(TAG, "NULL pointer in bmi270_write_register");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 2-byte transaction buffer
    uint8_t tx_data[2] = {
        reg_addr & ~BMI270_SPI_READ_BIT,  // Byte 1: Write command (bit7=0)
        data                               // Byte 2: Data
    };

    spi_transaction_t trans = {
        .flags = 0,
        .length = 2 * 8,      // 2 bytes = 16 bits
        .tx_buffer = tx_data,
        .rx_buffer = NULL,    // No receive needed
        .user = NULL,
    };

    // Execute transaction
    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    if (ret == ESP_OK) {
        // Wait after write (timing depends on initialization state)
        if (dev->init_complete) {
            // Normal mode: 2µs
            esp_rom_delay_us(BMI270_DELAY_WRITE_NORMAL_US);
        } else {
            // Low-power mode: use override if set, otherwise use default
            uint32_t delay = (g_lowpower_delay_override > 0) ? g_lowpower_delay_override : BMI270_DELAY_ACCESS_LOWPOWER_US;
            esp_rom_delay_us(delay);
        }
    } else {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Read multiple registers from BMI270 (burst read)
 *
 * Burst Read Protocol:
 *   TX: [CMD] [Dummy] [Dummy] [Dummy] ... [Dummy]
 *   RX: [Echo] [Dummy] [D0]   [D1]   ... [DN]
 *                       ↑ Valid data starts here
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Starting register address
 * @param data Pointer to buffer for read data
 * @param length Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
// デバッグ用: IMUタスクのチェックポイント（main.cppで定義）
extern volatile uint8_t g_imu_checkpoint;

// Threshold for stack vs DMA allocation in burst read
// ESP-IDF SPI driver uses CPU transfer for ≤32 bytes (no DMA benefit)
#define BMI270_BURST_READ_STACK_MAX 32

esp_err_t bmi270_read_burst(bmi270_dev_t *dev, uint8_t reg_addr, uint8_t *data, size_t length) {
    if (dev == NULL || data == NULL || length == 0) {
        ESP_LOGE(TAG, "Invalid parameters in bmi270_read_burst");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    g_imu_checkpoint = 40;  // read_burst開始

    // Total bytes = 1 (CMD) + 1 (Dummy) + length (Data)
    size_t total_bytes = 2 + length;

    // For small transfers (≤32 bytes): use stack buffer (no malloc overhead)
    // For large transfers (FIFO ~2KB): use DMA buffer
    uint8_t stack_tx[BMI270_BURST_READ_STACK_MAX];
    uint8_t stack_rx[BMI270_BURST_READ_STACK_MAX];
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    bool use_dma = (total_bytes > BMI270_BURST_READ_STACK_MAX);

    g_imu_checkpoint = 41;  // バッファ準備

    if (use_dma) {
        // Large transfer: allocate DMA-capable buffers
        tx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);
        rx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);
        if (!tx_buffer || !rx_buffer) {
            ESP_LOGE(TAG, "Failed to allocate DMA buffers for %zu bytes", total_bytes);
            heap_caps_free(tx_buffer);
            heap_caps_free(rx_buffer);
            return ESP_ERR_NO_MEM;
        }
    } else {
        // Small transfer: use stack buffers
        tx_buffer = stack_tx;
        rx_buffer = stack_rx;
    }

    // Prepare TX buffer
    tx_buffer[0] = reg_addr | BMI270_SPI_READ_BIT;  // Read command
    memset(&tx_buffer[1], 0x00, total_bytes - 1);   // Dummy bytes

    g_imu_checkpoint = 42;  // トランザクション準備完了

    spi_transaction_t trans = {
        .flags = 0,
        .length = total_bytes * 8,
        .rxlength = total_bytes * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .user = NULL,
    };

    g_imu_checkpoint = 43;  // SPI転送前

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    g_imu_checkpoint = 44;  // SPI転送後

    if (ret == ESP_OK) {
        // rx_buffer[0] = Command echo (discard)
        // rx_buffer[1] = Dummy (discard)
        // rx_buffer[2~] = Valid data
        memcpy(data, &rx_buffer[2], length);
    } else {
        ESP_LOGE(TAG, "SPI burst read failed: %s", esp_err_to_name(ret));
    }

    if (use_dma) {
        heap_caps_free(tx_buffer);
        heap_caps_free(rx_buffer);
    }

    return ret;
}

/**
 * @brief Write multiple registers to BMI270 (burst write)
 *
 * Burst Write Protocol:
 *   TX: [CMD] [D0] [D1] [D2] ... [DN]
 *         ↑    ↑ Data starts immediately after command
 *
 * @param dev Pointer to BMI270 device structure
 * @param reg_addr Starting register address
 * @param data Pointer to data to write
 * @param length Number of bytes to write
 * @return esp_err_t ESP_OK on success
 */
// Threshold for stack vs DMA allocation in write_burst
#define BMI270_BURST_WRITE_STACK_MAX 32

esp_err_t bmi270_write_burst(bmi270_dev_t *dev, uint8_t reg_addr, const uint8_t *data, size_t length) {
    if (dev == NULL || data == NULL || length == 0) {
        ESP_LOGE(TAG, "Invalid parameters in bmi270_write_burst");
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BMI270 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    // Total bytes = 1 (CMD) + length (Data)
    size_t total_bytes = 1 + length;

    // For small transfers: use stack buffer (no malloc overhead)
    // For large transfers (config file ~8KB): use DMA buffer
    uint8_t stack_buffer[BMI270_BURST_WRITE_STACK_MAX];
    uint8_t *tx_buffer;
    bool use_dma = (total_bytes > BMI270_BURST_WRITE_STACK_MAX);

    if (use_dma) {
        tx_buffer = heap_caps_malloc(total_bytes, MALLOC_CAP_DMA);
        if (!tx_buffer) {
            ESP_LOGE(TAG, "Failed to allocate DMA buffer for %zu bytes", total_bytes);
            return ESP_ERR_NO_MEM;
        }
    } else {
        tx_buffer = stack_buffer;
    }

    // Prepare TX buffer
    tx_buffer[0] = reg_addr & ~BMI270_SPI_READ_BIT;  // Write command
    memcpy(&tx_buffer[1], data, length);

    spi_transaction_t trans = {
        .flags = 0,
        .length = total_bytes * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL,
        .user = NULL,
    };

    ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    if (use_dma) {
        heap_caps_free(tx_buffer);
    }

    if (ret == ESP_OK) {
        // Wait after write (timing depends on initialization state)
        if (dev->init_complete) {
            // Normal mode: 2µs
            esp_rom_delay_us(BMI270_DELAY_WRITE_NORMAL_US);
        } else {
            // Low-power mode: use override if set, otherwise use default
            uint32_t delay = (g_lowpower_delay_override > 0) ? g_lowpower_delay_override : BMI270_DELAY_ACCESS_LOWPOWER_US;
            esp_rom_delay_us(delay);
        }
    } else {
        ESP_LOGE(TAG, "SPI burst write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Mark BMI270 initialization as complete
 *
 * Call this function after BMI270 initialization sequence is complete
 * to switch to normal mode timing (2µs instead of 1000µs).
 *
 * @param dev Pointer to BMI270 device structure
 */
void bmi270_set_init_complete(bmi270_dev_t *dev) {
    if (dev != NULL) {
        dev->init_complete = true;
        ESP_LOGI(TAG, "BMI270 initialization complete - switched to normal mode timing");
    }
}

/**
 * @brief Override low-power mode delay for testing (experimental)
 *
 * @param delay_us New delay time in microseconds (0 = use default)
 */
void bmi270_set_lowpower_delay_override(uint32_t delay_us) {
    g_lowpower_delay_override = delay_us;
    if (delay_us > 0) {
        ESP_LOGI(TAG, "Low-power delay override set to %lu µs (experimental)", delay_us);
    } else {
        ESP_LOGI(TAG, "Low-power delay override cleared (using default)");
    }
}
