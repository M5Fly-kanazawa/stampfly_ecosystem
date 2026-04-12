/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 StampFly ToF Driver Contributors
 */

/**
 * @file vl53lx_platform.c
 * @brief VL53LX Platform Layer Implementation for ESP-IDF
 *
 * Implements platform-specific functions for VL53LX API using ESP-IDF's I2C master API
 */

#include "vl53lx_platform.h"
#include "vl53lx_ll_def.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "VL53LX_PLATFORM";

// I2C transaction timeout in milliseconds
#define VL53LX_I2C_TIMEOUT_MS  100

//=============================================================================
// VL53LX API-compatible functions
//=============================================================================

/**
 * @brief Initialize communications interface
 */
VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz)
{
    (void)comms_type;       // I2C is the only supported type
    (void)comms_speed_khz;  // Speed is configured at bus level

    // Nothing to do here, I2C device is already added via VL53LX_PlatformInit
    ESP_LOGI(TAG, "Comms initialized (I2C, %d kHz)", comms_speed_khz);
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Close communications interface
 */
VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev)
{
    (void)pdev;
    // Nothing to do here, cleanup is handled by VL53LX_PlatformDeinit
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Write multiple bytes to VL53LX device
 */
VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (pdev == NULL || pdev->I2cDevAddr == 0 || pdata == NULL) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    // Allocate buffer: 2 bytes for register address + data
    uint8_t *buffer = (uint8_t *)malloc(count + 2);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    // Register address in big-endian (MSB first)
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    memcpy(&buffer[2], pdata, count);

    // Write data using I2C
    esp_err_t ret = i2c_master_transmit(pdev->I2cHandle, buffer, count + 2, VL53LX_I2C_TIMEOUT_MS);

    free(buffer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed at 0x%04X: %s", index, esp_err_to_name(ret));
        return (ret == ESP_ERR_TIMEOUT) ? VL53LX_ERROR_TIME_OUT : VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Read multiple bytes from VL53LX device
 */
VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (pdev == NULL || pdev->I2cDevAddr == 0 || pdata == NULL) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    // Register address in big-endian (MSB first)
    uint8_t reg_addr[2];
    reg_addr[0] = (index >> 8) & 0xFF;
    reg_addr[1] = index & 0xFF;

    // Write register address, then read data
    esp_err_t ret = i2c_master_transmit_receive(pdev->I2cHandle,
                                                reg_addr, 2,
                                                pdata, count,
                                                VL53LX_I2C_TIMEOUT_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed at 0x%04X: %s", index, esp_err_to_name(ret));
        return (ret == ESP_ERR_TIMEOUT) ? VL53LX_ERROR_TIME_OUT : VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Write single byte
 */
VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data)
{
    return VL53LX_WriteMulti(pdev, index, &data, 1);
}

/**
 * @brief Write 16-bit word (big-endian)
 */
VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;  // MSB
    buffer[1] = data & 0xFF;         // LSB
    return VL53LX_WriteMulti(pdev, index, buffer, 2);
}

/**
 * @brief Write 32-bit double word (big-endian)
 */
VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;  // MSB
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;          // LSB
    return VL53LX_WriteMulti(pdev, index, buffer, 4);
}

/**
 * @brief Read single byte
 */
VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata)
{
    return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

/**
 * @brief Read 16-bit word (big-endian)
 */
VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata)
{
    uint8_t buffer[2];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 2);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

/**
 * @brief Read 32-bit double word (big-endian)
 */
VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata)
{
    uint8_t buffer[4];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buffer, 4);
    if (status == VL53LX_ERROR_NONE) {
        *pdata = ((uint32_t)buffer[0] << 24) |
                 ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8) |
                 buffer[3];
    }
    return status;
}

/**
 * @brief Wait for specified microseconds
 */
VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us)
{
    (void)pdev;

    if (wait_us < 0) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    esp_rom_delay_us(wait_us);
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Wait for specified milliseconds
 */
VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms)
{
    (void)pdev;

    if (wait_ms < 0) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get timer frequency
 */
VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    // ESP32 timer runs at 1 MHz (microsecond resolution)
    *ptimer_freq_hz = 1000000;
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get current timer value in microseconds
 */
VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
    *ptimer_count = (int32_t)esp_timer_get_time();
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Wait for a register value to match expected value with mask
 *
 * Polls a register until the masked value equals the expected value or timeout occurs
 *
 * @param[in]   pdev          : pointer to device structure
 * @param[in]   timeout_ms    : timeout in milliseconds
 * @param[in]   index         : register index
 * @param[in]   value         : expected value after applying mask
 * @param[in]   mask          : mask to apply to read value
 * @param[in]   poll_delay_ms : delay between polls in milliseconds
 *
 * @return   VL53LX_ERROR_NONE      Success
 * @return   VL53LX_ERROR_TIME_OUT  Timeout occurred
 * @return   Other error codes
 */
VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t      timeout_ms,
    uint16_t      index,
    uint8_t       value,
    uint8_t       mask,
    uint32_t      poll_delay_ms)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint32_t polling_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;

    // Get start time (using FreeRTOS ticks converted to ms)
    start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Keep polling until timeout or value matches
    while ((status == VL53LX_ERROR_NONE) &&
           (polling_time_ms < timeout_ms) &&
           (found == 0)) {

        // Read register value
        status = VL53LX_RdByte(pdev, index, &byte_value);

        // Check if masked value matches expected value
        if ((byte_value & mask) == value) {
            found = 1;
        }

        // Update polling time
        current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        polling_time_ms = current_time_ms - start_time_ms;

        // Wait before next poll if not found yet
        if ((status == VL53LX_ERROR_NONE) && (found == 0) && (polling_time_ms < timeout_ms)) {
            status = VL53LX_WaitMs(pdev, poll_delay_ms);
        }
    }

    // If value not found, return timeout error
    if (found == 0 && status == VL53LX_ERROR_NONE) {
        status = VL53LX_ERROR_TIME_OUT;
    }

    return status;
}

/**
 * @brief Set GPIO mode (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
    (void)pin;
    (void)mode;
    // GPIO configuration is handled separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Set GPIO value (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
    (void)pin;
    (void)value;
    // GPIO control is handled separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get GPIO value (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
    (void)pin;
    (void)pvalue;
    // GPIO read is handled separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Control XSHUT pin (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioXshutdown(uint8_t value)
{
    (void)value;
    // XSHUT control is handled separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Select comms interface (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value)
{
    (void)value;
    // Not applicable for ESP-IDF implementation
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Enable power (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value)
{
    (void)value;
    // Power control is handled separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Get interrupt status (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
    (void)function;
    (void)edge_type;
    // Interrupt handling is done separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

/**
 * @brief Disable interrupt (not used in ESP-IDF implementation)
 */
VL53LX_Error VL53LX_GpioInterruptDisable(void)
{
    // Interrupt handling is done separately in ESP-IDF
    return VL53LX_ERROR_NONE;
}

//=============================================================================
// ESP-IDF specific helper functions for Stage 2 compatibility
//=============================================================================

/**
 * @brief Initialize VL53LX platform (ESP-IDF specific)
 */
int8_t VL53LX_PlatformInit(VL53LX_DEV pdev, i2c_master_bus_handle_t bus_handle, uint16_t device_address)
{
    if (pdev == NULL || bus_handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = 400000,  // 400 kHz (Fast mode)
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &pdev->I2cHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    pdev->I2cDevAddr = device_address;
    ESP_LOGI(TAG, "VL53LX platform initialized at address 0x%02X", device_address);

    return VL53LX_ERROR_NONE;
}

/**
 * @brief Deinitialize VL53LX platform (ESP-IDF specific)
 */
int8_t VL53LX_PlatformDeinit(VL53LX_DEV pdev)
{
    if (pdev == NULL || pdev->I2cHandle == NULL) {
        return VL53LX_ERROR_INVALID_PARAMS;
    }

    esp_err_t ret = i2c_master_bus_rm_device(pdev->I2cHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return VL53LX_ERROR_CONTROL_INTERFACE;
    }

    pdev->I2cHandle = NULL;
    ESP_LOGI(TAG, "VL53LX platform deinitialized");

    return VL53LX_ERROR_NONE;
}

// Legacy compatibility functions (for Stage 2)
int8_t VL53LX_WriteByte(VL53LX_DEV pdev, uint16_t index, uint8_t data)
{
    return VL53LX_WrByte(pdev, index, data);
}

int8_t VL53LX_ReadByte(VL53LX_DEV pdev, uint16_t index, uint8_t *pdata)
{
    return VL53LX_RdByte(pdev, index, pdata);
}

int8_t VL53LX_WriteWord(VL53LX_DEV pdev, uint16_t index, uint16_t data)
{
    return VL53LX_WrWord(pdev, index, data);
}

int8_t VL53LX_ReadWord(VL53LX_DEV pdev, uint16_t index, uint16_t *pdata)
{
    return VL53LX_RdWord(pdev, index, pdata);
}

int8_t VL53LX_WriteDWord(VL53LX_DEV pdev, uint16_t index, uint32_t data)
{
    return VL53LX_WrDWord(pdev, index, data);
}

int8_t VL53LX_ReadDWord(VL53LX_DEV pdev, uint16_t index, uint32_t *pdata)
{
    return VL53LX_RdDWord(pdev, index, pdata);
}
