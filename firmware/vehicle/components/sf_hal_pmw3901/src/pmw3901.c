/**
 * @file pmw3901.c
 * @brief PMW3901 Optical Flow Sensor Driver Implementation for StampFly (ESP32-S3)
 */

#include "pmw3901.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <string.h>
#include <math.h>

static const char *TAG = "PMW3901";

/**
 * @brief Delay in microseconds
 */
static inline void delay_us(uint32_t us)
{
    ets_delay_us(us);
}

/**
 * @brief Delay in milliseconds
 */
static inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Performance optimization register initialization
 * Based on official PMW3901 manual (how_to_use_pwm3901.md)
 * This sequence includes conditional logic and calibration value calculations
 */
static esp_err_t pmw3901_init_registers(pmw3901_t *dev)
{
    esp_err_t ret;
    uint8_t read_val;

    ESP_LOGI(TAG, "  Step 6.1: Initial settings");

    // Initial settings
    pmw3901_write_register(dev, 0x7F, 0x00);
    pmw3901_write_register(dev, 0x55, 0x01);
    pmw3901_write_register(dev, 0x50, 0x07);
    pmw3901_write_register(dev, 0x7F, 0x0E);

    // Verification sequence (retry up to 3 times)
    ESP_LOGI(TAG, "  Step 6.2: Verification sequence");
    bool verification_ok = false;
    for (int i = 0; i < 3; i++) {
        pmw3901_write_register(dev, 0x43, 0x10);
        ret = pmw3901_read_register(dev, 0x47, &read_val);
        if (ret == ESP_OK && read_val == 0x08) {
            verification_ok = true;
            ESP_LOGI(TAG, "    Verification passed on attempt %d", i + 1);
            break;
        }
        ESP_LOGW(TAG, "    Verification attempt %d failed (read 0x%02X, expected 0x08)", i + 1, read_val);
    }

    if (!verification_ok) {
        ESP_LOGE(TAG, "    Verification failed after 3 attempts!");
        return ESP_FAIL;
    }

    // Conditional write based on register 0x67 bit 7
    ESP_LOGI(TAG, "  Step 6.3: Conditional write (0x67 bit 7)");
    ret = pmw3901_read_register(dev, 0x67, &read_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "    Failed to read register 0x67");
        return ret;
    }

    if (read_val & 0x80) {
        ESP_LOGI(TAG, "    Bit 7 set: writing 0x04 to 0x48");
        pmw3901_write_register(dev, 0x48, 0x04);
    } else {
        ESP_LOGI(TAG, "    Bit 7 clear: writing 0x02 to 0x48");
        pmw3901_write_register(dev, 0x48, 0x02);
    }

    // Continue with settings
    ESP_LOGI(TAG, "  Step 6.4: Additional settings");
    pmw3901_write_register(dev, 0x7F, 0x00);
    pmw3901_write_register(dev, 0x51, 0x7B);
    pmw3901_write_register(dev, 0x50, 0x00);
    pmw3901_write_register(dev, 0x55, 0x00);
    pmw3901_write_register(dev, 0x7F, 0x0E);

    // C1/C2 calibration calculation
    ESP_LOGI(TAG, "  Step 6.5: C1/C2 calibration");
    ret = pmw3901_read_register(dev, 0x73, &read_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "    Failed to read register 0x73");
        return ret;
    }

    if (read_val != 0x00) {
        ESP_LOGI(TAG, "    Register 0x73 = 0x%02X (not 0x00), skipping C1/C2 calculation", read_val);
    } else {
        ESP_LOGI(TAG, "    Register 0x73 = 0x00, performing C1/C2 calculation");

        uint8_t c1, c2;
        pmw3901_read_register(dev, 0x70, &c1);
        pmw3901_read_register(dev, 0x71, &c2);
        ESP_LOGI(TAG, "    Original C1=0x%02X, C2=0x%02X", c1, c2);

        // Calculate new C1
        uint8_t new_c1;
        if (c1 <= 28) {
            new_c1 = c1 + 14;
        } else {
            new_c1 = c1 + 11;
        }
        if (new_c1 > 0x3F) {
            new_c1 = 0x3F;
        }

        // Calculate new C2
        uint8_t new_c2 = (c2 * 45) / 100;

        ESP_LOGI(TAG, "    Calculated new_C1=0x%02X, new_C2=0x%02X", new_c1, new_c2);

        // Write new calibration values
        pmw3901_write_register(dev, 0x7F, 0x00);
        pmw3901_write_register(dev, 0x61, 0xAD);
        pmw3901_write_register(dev, 0x51, new_c1);
        pmw3901_write_register(dev, 0x7F, 0x0E);
        pmw3901_write_register(dev, 0x70, new_c1);
        pmw3901_write_register(dev, 0x71, new_c2);
    }

    // Continue after C1/C2 (or if skipped)
    ESP_LOGI(TAG, "  Step 6.6: Main register configuration");
    pmw3901_write_register(dev, 0x7F, 0x00);
    pmw3901_write_register(dev, 0x61, 0xAD);
    pmw3901_write_register(dev, 0x7F, 0x03);
    pmw3901_write_register(dev, 0x40, 0x00);
    pmw3901_write_register(dev, 0x7F, 0x05);
    pmw3901_write_register(dev, 0x41, 0xB3);
    pmw3901_write_register(dev, 0x43, 0xF1);
    pmw3901_write_register(dev, 0x45, 0x14);
    pmw3901_write_register(dev, 0x5B, 0x32);
    pmw3901_write_register(dev, 0x5F, 0x34);
    pmw3901_write_register(dev, 0x7B, 0x08);
    pmw3901_write_register(dev, 0x7F, 0x06);
    pmw3901_write_register(dev, 0x44, 0x1B);
    pmw3901_write_register(dev, 0x40, 0xBF);
    pmw3901_write_register(dev, 0x4E, 0x3F);
    pmw3901_write_register(dev, 0x7F, 0x08);
    pmw3901_write_register(dev, 0x65, 0x20);
    pmw3901_write_register(dev, 0x6A, 0x18);
    pmw3901_write_register(dev, 0x7F, 0x09);
    pmw3901_write_register(dev, 0x4F, 0xAF);
    pmw3901_write_register(dev, 0x5F, 0x40);
    pmw3901_write_register(dev, 0x48, 0x80);
    pmw3901_write_register(dev, 0x49, 0x80);
    pmw3901_write_register(dev, 0x57, 0x77);
    pmw3901_write_register(dev, 0x60, 0x78);
    pmw3901_write_register(dev, 0x61, 0x78);
    pmw3901_write_register(dev, 0x62, 0x08);

    ESP_LOGI(TAG, "  Step 6.7: 10ms wait");
    delay_ms(10);

    ESP_LOGI(TAG, "  Step 6.8: Final register configuration");
    pmw3901_write_register(dev, 0x32, 0x44);
    pmw3901_write_register(dev, 0x7F, 0x07);
    pmw3901_write_register(dev, 0x63, 0x50);
    pmw3901_write_register(dev, 0x7F, 0x0A);
    pmw3901_write_register(dev, 0x45, 0x60);
    pmw3901_write_register(dev, 0x7F, 0x00);
    pmw3901_write_register(dev, 0x4D, 0x11);
    pmw3901_write_register(dev, 0x55, 0x80);
    pmw3901_write_register(dev, 0x74, 0x1F);
    pmw3901_write_register(dev, 0x75, 0x1F);
    pmw3901_write_register(dev, 0x4A, 0x78);
    pmw3901_write_register(dev, 0x4B, 0x78);
    pmw3901_write_register(dev, 0x44, 0x08);
    pmw3901_write_register(dev, 0x45, 0x50);
    pmw3901_write_register(dev, 0x64, 0xFF);
    pmw3901_write_register(dev, 0x65, 0x1F);
    pmw3901_write_register(dev, 0x7F, 0x14);
    pmw3901_write_register(dev, 0x65, 0x67);
    pmw3901_write_register(dev, 0x66, 0x08);
    pmw3901_write_register(dev, 0x63, 0x70);
    pmw3901_write_register(dev, 0x7F, 0x15);
    pmw3901_write_register(dev, 0x48, 0x48);
    pmw3901_write_register(dev, 0x7F, 0x07);
    pmw3901_write_register(dev, 0x41, 0x0D);
    pmw3901_write_register(dev, 0x43, 0x14);
    pmw3901_write_register(dev, 0x4B, 0x0E);
    pmw3901_write_register(dev, 0x45, 0x0F);
    pmw3901_write_register(dev, 0x44, 0x42);
    pmw3901_write_register(dev, 0x4C, 0x80);
    pmw3901_write_register(dev, 0x7F, 0x10);
    pmw3901_write_register(dev, 0x5B, 0x02);
    pmw3901_write_register(dev, 0x7F, 0x07);
    pmw3901_write_register(dev, 0x40, 0x41);
    pmw3901_write_register(dev, 0x70, 0x00);

    delay_ms(10);  // Brief stabilization delay

    pmw3901_write_register(dev, 0x40, 0x40);
    pmw3901_write_register(dev, 0x7F, 0x06);
    pmw3901_write_register(dev, 0x62, 0xF0);
    pmw3901_write_register(dev, 0x63, 0x00);
    pmw3901_write_register(dev, 0x7F, 0x0D);
    pmw3901_write_register(dev, 0x48, 0xC0);
    pmw3901_write_register(dev, 0x6F, 0xD5);
    pmw3901_write_register(dev, 0x7F, 0x00);
    pmw3901_write_register(dev, 0x5B, 0xA0);
    pmw3901_write_register(dev, 0x4E, 0xA8);
    pmw3901_write_register(dev, 0x5A, 0x50);
    pmw3901_write_register(dev, 0x40, 0x80);

    ESP_LOGI(TAG, "  Performance optimization complete");

    return ESP_OK;
}

void pmw3901_get_default_config(pmw3901_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->spi_host = SPI2_HOST;
    config->pin_miso = PMW3901_DEFAULT_PIN_MISO;
    config->pin_mosi = PMW3901_DEFAULT_PIN_MOSI;
    config->pin_sclk = PMW3901_DEFAULT_PIN_SCLK;
    config->pin_cs = PMW3901_DEFAULT_PIN_CS;
}

esp_err_t pmw3901_read_register(pmw3901_t *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || value == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_data[2] = {reg & 0x7F, 0x00};  // Clear MSB for read operation
    uint8_t rx_data[2] = {0};

    spi_transaction_t trans = {
        .length = 16,  // 2 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    *value = rx_data[1];  // Data is in second byte
    delay_us(PMW3901_SPI_READ_DELAY_US);

    return ESP_OK;
}

esp_err_t pmw3901_write_register(pmw3901_t *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_data[2] = {reg | 0x80, value};  // Set MSB for write operation

    spi_transaction_t trans = {
        .length = 16,  // 2 bytes * 8 bits
        .tx_buffer = tx_data,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return ret;
    }

    delay_us(PMW3901_SPI_WRITE_DELAY_US);

    return ESP_OK;
}

esp_err_t pmw3901_select_bank(pmw3901_t *dev, uint8_t bank)
{
    return pmw3901_write_register(dev, PMW3901_BANK_SELECT, bank);
}

esp_err_t pmw3901_reset(pmw3901_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Write 0x5A to reset register
    esp_err_t ret = pmw3901_write_register(dev, PMW3901_POWER_UP_RESET, 0x5A);
    if (ret != ESP_OK) {
        return ret;
    }

    delay_ms(PMW3901_RESET_DELAY_MS);

    return ESP_OK;
}

esp_err_t pmw3901_init(pmw3901_t *dev, const pmw3901_config_t *config)
{
    if (dev == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(pmw3901_t));
    dev->cs_pin = config->pin_cs;

    ESP_LOGI(TAG, "=== PMW3901 Initialization Start ===");

    // Official manual Step 1: Power-on sequence
    // "電源投入: VDDIOに電源を供給し、その後100ms以内にVDDに電源を供給します"
    // (Note: This is handled by hardware, not software)

    // Official manual Step 2: Wait for power stabilization
    // "待機時間: 電源が安定するまで、最低でも40ms待機します"
    ESP_LOGI(TAG, "Step 1: Power stabilization wait (45ms >= 40ms required)");
    delay_ms(PMW3901_POWER_UP_DELAY_MS);  // 45ms > 40ms required
    ESP_LOGI(TAG, "  Power stable: OK");

    // Configure CS pin as GPIO temporarily for SPI port reset
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Official manual Step 3: SPI port reset
    // "SPIポートリセット: NCSピンを一度HighにしてからLowに駆動し、SPIポートをリセットします"
    ESP_LOGI(TAG, "Step 1.5: SPI port reset (NCS High->Low)");
    gpio_set_level(config->pin_cs, 1);
    delay_ms(1);
    gpio_set_level(config->pin_cs, 0);
    delay_ms(1);
    gpio_set_level(config->pin_cs, 1);  // Leave high for SPI initialization
    delay_ms(1);
    ESP_LOGI(TAG, "  SPI port reset: OK");

    // Now configure SPI bus
    spi_bus_config_t bus_cfg = {
        .miso_io_num = config->pin_miso,
        .mosi_io_num = config->pin_mosi,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    // SPI bus initialization
    ESP_LOGI(TAG, "Step 2: SPI bus initialization");
    esp_err_t ret;
    if (config->skip_bus_init) {
        // Skip bus init - bus already initialized by another device (e.g., BMI270)
        ESP_LOGI(TAG, "  SPI bus: Skipped (shared with another device)");
    } else {
        ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "  SPI bus: OK");
    }

    // Configure SPI device (this will take over CS pin control)
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = PMW3901_SPI_CLOCK_SPEED_HZ,
        .mode = 3,  // SPI mode 3 (CPOL=1, CPHA=1)
        .spics_io_num = config->pin_cs,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
    };

    ret = spi_bus_add_device(config->spi_host, &dev_cfg, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(config->spi_host);
        return ret;
    }
    ESP_LOGI(TAG, "  SPI device: OK");

    dev->initialized = true;

    // Soft reset (Official manual Step 4)
    ESP_LOGI(TAG, "Step 3: Power-up reset (write 0x5A to reg 0x3A)");
    ret = pmw3901_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  Reset failed");
        goto cleanup;
    }
    ESP_LOGI(TAG, "  Reset: OK (waiting 5ms for completion)");

    // Clear motion registers FIRST (Official manual Step 6)
    // "モーションピンの状態に関わらず、レジスタ 0x02, 0x03, 0x04, 0x05, 0x06 を一度ずつ読み出します"
    ESP_LOGI(TAG, "Step 4: Initial motion register read (clear internal buffers)");
    uint8_t dummy;
    pmw3901_read_register(dev, PMW3901_MOTION, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_X_L, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_X_H, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_Y_L, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_Y_H, &dummy);
    ESP_LOGI(TAG, "  Motion registers read: OK");

    // NOW read and verify product ID
    ESP_LOGI(TAG, "Step 5: Reading and verifying chip IDs");
    uint8_t product_id, inverse_id;
    ret = pmw3901_read_register(dev, PMW3901_PRODUCT_ID, &product_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  Failed to read product ID");
        goto cleanup;
    }

    ret = pmw3901_read_register(dev, PMW3901_INVERSE_PRODUCT_ID, &inverse_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  Failed to read inverse product ID");
        goto cleanup;
    }

    ESP_LOGI(TAG, "  Product ID: 0x%02X (expected 0x%02X)", product_id, PMW3901_PRODUCT_ID_VALUE);
    ESP_LOGI(TAG, "  Inverse ID: 0x%02X (expected 0x%02X)", inverse_id, PMW3901_INVERSE_PRODUCT_ID_VALUE);

    if (product_id != PMW3901_PRODUCT_ID_VALUE) {
        ESP_LOGE(TAG, "  Invalid product ID!");
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    if (inverse_id != PMW3901_INVERSE_PRODUCT_ID_VALUE) {
        ESP_LOGE(TAG, "  Invalid inverse product ID!");
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    dev->product_id = product_id;

    // Read revision ID
    ret = pmw3901_read_register(dev, PMW3901_REVISION_ID, &dev->revision_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  Failed to read revision ID");
        goto cleanup;
    }
    ESP_LOGI(TAG, "  Revision ID: 0x%02X", dev->revision_id);
    ESP_LOGI(TAG, "  Chip verification: OK");

    // Initialize registers with optimal settings
    ESP_LOGI(TAG, "Step 6: Writing performance optimization registers");
    ret = pmw3901_init_registers(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  Failed to initialize registers");
        goto cleanup;
    }
    ESP_LOGI(TAG, "  Register initialization: OK");

    // Wait for sensor to stabilize after initialization
    ESP_LOGI(TAG, "Step 7: Sensor stabilization (100ms wait)");
    delay_ms(100);

    // Clear motion registers again after initialization
    ESP_LOGI(TAG, "Step 8: Final motion register clear");
    pmw3901_read_register(dev, PMW3901_MOTION, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_X_L, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_X_H, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_Y_L, &dummy);
    pmw3901_read_register(dev, PMW3901_DELTA_Y_H, &dummy);
    ESP_LOGI(TAG, "  Final clear: OK");

    ESP_LOGI(TAG, "=== PMW3901 Initialization Complete ===");

    return ESP_OK;

cleanup:
    spi_bus_remove_device(dev->spi_handle);
    dev->initialized = false;
    return ret;
}

esp_err_t pmw3901_deinit(pmw3901_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_remove_device(dev->spi_handle);
    dev->initialized = false;

    ESP_LOGD(TAG, "PMW3901 deinitialized");

    return ESP_OK;
}

esp_err_t pmw3901_read_motion(pmw3901_t *dev, int16_t *delta_x, int16_t *delta_y)
{
    if (dev == NULL || delta_x == NULL || delta_y == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t motion, xl, xh, yl, yh;
    esp_err_t ret;

    // Step 1: Read Motion register to latch (freeze) the delta values
    // This prevents values from updating during the read sequence
    ret = pmw3901_read_register(dev, PMW3901_MOTION, &motion);
    if (ret != ESP_OK) return ret;

    // Step 2: Check motion bit 7 (optional, but recommended by datasheet)
    // However, we always read delta registers even if bit 7 is 0,
    // because they may contain valid zero or small values

    // Step 3: Read delta registers
    // Read Delta X (low and high bytes)
    ret = pmw3901_read_register(dev, PMW3901_DELTA_X_L, &xl);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_register(dev, PMW3901_DELTA_X_H, &xh);
    if (ret != ESP_OK) return ret;

    // Read Delta Y (low and high bytes)
    ret = pmw3901_read_register(dev, PMW3901_DELTA_Y_L, &yl);
    if (ret != ESP_OK) return ret;

    ret = pmw3901_read_register(dev, PMW3901_DELTA_Y_H, &yh);
    if (ret != ESP_OK) return ret;

    // Combine bytes into 16-bit signed integers
    *delta_x = (int16_t)((xh << 8) | xl);
    *delta_y = (int16_t)((yh << 8) | yl);

    // PX4 validation: Reject impossibly large values (corrupted SPI data)
    if (*delta_x > 240 || *delta_x < -240 || *delta_y > 240 || *delta_y < -240) {
        ESP_LOGW(TAG, "Invalid motion data: dX=%d dY=%d (exceeds ±240)", *delta_x, *delta_y);
        *delta_x = 0;
        *delta_y = 0;
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

// デバッグ用: OptFlowタスクのチェックポイント（main.cppで定義）
extern volatile uint8_t g_optflow_checkpoint;

esp_err_t pmw3901_read_motion_burst(pmw3901_t *dev, pmw3901_motion_burst_t *burst)
{
    if (dev == NULL || burst == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    g_optflow_checkpoint = 50;  // burst read開始

    // PMW3901 burst read protocol:
    // Send 0x16 command, then immediately read 12 bytes of data
    // Use single SPI transaction with command + data to keep CS low throughout

    uint8_t tx_data[13];
    uint8_t rx_data[13];

    memset(tx_data, 0, sizeof(tx_data));
    memset(rx_data, 0, sizeof(rx_data));

    // First byte is burst read command
    tx_data[0] = PMW3901_MOTION_BURST & 0x7F;  // 0x16

    spi_transaction_t trans = {
        .length = 13 * 8,  // 13 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    g_optflow_checkpoint = 51;  // SPI転送前

    esp_err_t ret = spi_device_polling_transmit(dev->spi_handle, &trans);

    g_optflow_checkpoint = 52;  // SPI転送後

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Burst read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_optflow_checkpoint = 53;  // delay前

    delay_us(PMW3901_SPI_READ_DELAY_US);

    g_optflow_checkpoint = 54;  // delay後

    // Parse burst data according to datasheet
    // Byte 0: command echo (should be 0x16, but may vary)
    // Byte 1: Motion (0x02)
    // Byte 2: Observation (0x15)
    // Byte 3: Delta_X_L
    // Byte 4: Delta_X_H
    // Byte 5: Delta_Y_L
    // Byte 6: Delta_Y_H
    // Byte 7: SQUAL
    // Byte 8: Raw_Data_Sum
    // Byte 9: Maximum_Raw_Data
    // Byte 10: Minimum_Raw_Data
    // Byte 11: Shutter_Upper
    // Byte 12: Shutter_Lower

    burst->motion = rx_data[1];
    burst->observation = rx_data[2];
    burst->delta_x = (int16_t)((rx_data[4] << 8) | rx_data[3]);
    burst->delta_y = (int16_t)((rx_data[6] << 8) | rx_data[5]);
    burst->squal = rx_data[7];
    burst->raw_data_sum = rx_data[8];
    burst->max_raw_data = rx_data[9];
    burst->min_raw_data = rx_data[10];
    burst->shutter = (uint16_t)((rx_data[11] << 8) | rx_data[12]);

    // Official datasheet filtering (Section 4.0)
    // Discard data if SQUAL < 0x19 AND Shutter_Upper == 0x1F
    uint8_t shutter_upper = rx_data[11];
    if (burst->squal < 0x19 && shutter_upper == 0x1F) {
        ESP_LOGD(TAG, "Low quality data (SQUAL=%d, Shutter=0x%02X), discarding",
                 burst->squal, shutter_upper);
        burst->delta_x = 0;
        burst->delta_y = 0;
        return ESP_OK;  // Not an error, just low quality
    }

    // Additional validation: Reject impossibly large values (SPI corruption)
    if (burst->delta_x > 240 || burst->delta_x < -240 ||
        burst->delta_y > 240 || burst->delta_y < -240) {
        ESP_LOGW(TAG, "Invalid burst motion data: dX=%d dY=%d (exceeds ±240)",
                 burst->delta_x, burst->delta_y);
        burst->delta_x = 0;
        burst->delta_y = 0;
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t pmw3901_get_product_id(pmw3901_t *dev, uint8_t *product_id)
{
    if (dev == NULL || product_id == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    *product_id = dev->product_id;
    return ESP_OK;
}

esp_err_t pmw3901_get_revision_id(pmw3901_t *dev, uint8_t *revision_id)
{
    if (dev == NULL || revision_id == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    *revision_id = dev->revision_id;
    return ESP_OK;
}

esp_err_t pmw3901_is_motion_detected(pmw3901_t *dev, bool *motion_detected)
{
    if (dev == NULL || motion_detected == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t motion;
    esp_err_t ret = pmw3901_read_register(dev, PMW3901_MOTION, &motion);
    if (ret != ESP_OK) {
        return ret;
    }

    *motion_detected = (motion & 0x80) != 0;  // Bit 7 indicates motion

    return ESP_OK;
}

esp_err_t pmw3901_enable_frame_capture(pmw3901_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    // Enable frame capture mode
    return pmw3901_write_register(dev, PMW3901_RAW_DATA_GRAB, 0x00);
}

esp_err_t pmw3901_read_frame(pmw3901_t *dev, uint8_t *image)
{
    if (dev == NULL || image == NULL || !dev->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status;
    int pixel_count = 0;

    while (pixel_count < PMW3901_FRAME_SIZE) {
        // Check if data is ready
        esp_err_t ret = pmw3901_read_register(dev, PMW3901_RAW_DATA_GRAB_STATUS, &status);
        if (ret != ESP_OK) {
            return ret;
        }

        // Check bits 6-7 for data ready status
        if ((status & 0xC0) != 0) {
            // Read pixel data
            ret = pmw3901_read_register(dev, PMW3901_RAW_DATA_GRAB, &image[pixel_count]);
            if (ret != ESP_OK) {
                return ret;
            }
            pixel_count++;
        }
    }

    return ESP_OK;
}

/* Velocity calculation functions removed.
 * Use ESKF::updateFlowRaw() for proper velocity estimation.
 * See: components/stampfly_eskf/eskf.cpp
 */
