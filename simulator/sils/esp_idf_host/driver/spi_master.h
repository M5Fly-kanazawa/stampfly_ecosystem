/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file driver/spi_master.h
 * @brief Host shim for the ESP-IDF SPI master driver
 *        ESP-IDF SPI マスタドライバのホスト用シム
 *
 * Provides the SPI master API surface (types, enums, struct fields and stub
 * functions) so the real StampFly firmware — BMI270 / PMW3901 HALs, the
 * led_strip component and sf_board bus init — compiles and links on a host PC
 * without an ESP32. All bodies are inert: handles are filled with a dummy
 * non-NULL pointer, output buffers are zeroed and functions return ESP_OK.
 *
 * 本体ファーム（BMI270 / PMW3901 HAL、led_strip コンポーネント、sf_board の
 * バス初期化）が ESP32 なしでホスト上でコンパイル・リンクできるよう、SPI
 * マスタ API（型・列挙・構造体フィールド・スタブ関数）を提供する。実体は
 * 無動作: ハンドルにダミーの非 NULL ポインタを入れ、出力バッファをゼロ埋め
 * し、関数は ESP_OK を返す。
 */

#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* SPI host devices / SPI ホストデバイス                                      */
/* ------------------------------------------------------------------------- */

// SPI peripheral selector (which hardware SPI host)
// SPI ペリフェラル選択（どのハードウェア SPI ホストか）
typedef enum {
    SPI1_HOST = 0,  ///< SPI1
    SPI2_HOST = 1,  ///< SPI2
    SPI3_HOST = 2,  ///< SPI3
    SPI_HOST_MAX,   ///< Invalid host value / 無効なホスト値
} spi_host_device_t;

/* ------------------------------------------------------------------------- */
/* DMA channel selection / DMA チャネル選択                                   */
/* ------------------------------------------------------------------------- */

// DMA channel passed to spi_bus_initialize()
// spi_bus_initialize() に渡す DMA チャネル
typedef enum {
    SPI_DMA_DISABLED = 0,  ///< Do not use DMA / DMA を使わない
    SPI_DMA_CH_AUTO  = 3,  ///< Auto-select a DMA channel / DMA チャネル自動選択
} spi_common_dma_t;

// Alias kept for source that passes the raw enum type
// 生の列挙型を渡すソース向けの別名
typedef int spi_dma_chan_t;

/* ------------------------------------------------------------------------- */
/* Clock source / クロックソース                                              */
/* ------------------------------------------------------------------------- */

// SPI clock source (led_strip selects SPI_CLK_SRC_DEFAULT)
// SPI クロックソース（led_strip は SPI_CLK_SRC_DEFAULT を選択）
typedef enum {
    SPI_CLK_SRC_DEFAULT = 0,  ///< Default clock source / 既定のクロックソース
} spi_clock_source_t;

/* ------------------------------------------------------------------------- */
/* Bus flags / バスフラグ                                                     */
/* ------------------------------------------------------------------------- */

// spi_bus_config_t.flags bits (subset the firmware references)
// spi_bus_config_t.flags のビット（ファームが参照する範囲）
#define SPICOMMON_BUSFLAG_SLAVE        (1 << 0)  ///< Slave mode
#define SPICOMMON_BUSFLAG_MASTER       (1 << 1)  ///< Master mode
#define SPICOMMON_BUSFLAG_IOMUX_PINS   (1 << 2)  ///< Use IO-MUX pins
#define SPICOMMON_BUSFLAG_GPIO_PINS    (1 << 3)  ///< Route pins via GPIO matrix
#define SPICOMMON_BUSFLAG_SCLK         (1 << 7)  ///< SCLK present
#define SPICOMMON_BUSFLAG_MISO         (1 << 8)  ///< MISO present
#define SPICOMMON_BUSFLAG_MOSI         (1 << 9)  ///< MOSI present
#define SPICOMMON_BUSFLAG_DUAL         (1 << 10) ///< Dual (2-bit) capable
#define SPICOMMON_BUSFLAG_QUAD         (1 << 12) ///< Quad (4-bit) capable

// spi_transaction_t.flags bits / spi_transaction_t.flags のビット
#define SPI_TRANS_MODE_DIO            (1 << 0)  ///< Transmit using DIO
#define SPI_TRANS_MODE_QIO            (1 << 1)  ///< Transmit using QIO
#define SPI_TRANS_USE_RXDATA         (1 << 2)  ///< rx_data is used (not rx_buffer)
#define SPI_TRANS_USE_TXDATA         (1 << 3)  ///< tx_data is used (not tx_buffer)
#define SPI_TRANS_MODE_DIOQIO_ADDR   (1 << 4)  ///< Also send addr in DIO/QIO
#define SPI_TRANS_VARIABLE_CMD       (1 << 5)  ///< Use trans->command_bits
#define SPI_TRANS_VARIABLE_ADDR      (1 << 6)  ///< Use trans->address_bits
#define SPI_TRANS_VARIABLE_DUMMY     (1 << 7)  ///< Use trans->dummy_bits
#define SPI_TRANS_CS_KEEP_ACTIVE     (1 << 8)  ///< Keep CS active after transaction

// spi_device_interface_config_t.flags bits / デバイス設定 flags のビット
#define SPI_DEVICE_TXBIT_LSBFIRST    (1 << 0)  ///< Transmit LSB first
#define SPI_DEVICE_RXBIT_LSBFIRST    (1 << 1)  ///< Receive LSB first
#define SPI_DEVICE_BIT_LSBFIRST      (SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_RXBIT_LSBFIRST)
#define SPI_DEVICE_3WIRE             (1 << 2)  ///< 3-wire mode (MOSI is bidirectional)
#define SPI_DEVICE_POSITIVE_CS       (1 << 3)  ///< CS is active-high
#define SPI_DEVICE_HALFDUPLEX        (1 << 4)  ///< Half-duplex (RX after TX)
#define SPI_DEVICE_CLK_AS_CS         (1 << 5)  ///< Output clock on CS when CS active
#define SPI_DEVICE_NO_DUMMY          (1 << 6)  ///< No dummy phase
#define SPI_DEVICE_NO_RETURN_RESULT  (1 << 8)  ///< Do not enqueue result

/* ------------------------------------------------------------------------- */
/* Opaque handles / 不透明ハンドル                                            */
/* ------------------------------------------------------------------------- */

// SPI master bus handle (opaque) / SPI マスタバスハンドル（不透明）
typedef struct spi_master_bus_t* spi_master_bus_handle_t;

// SPI device handle (opaque) / SPI デバイスハンドル（不透明）
typedef struct spi_device_t* spi_device_handle_t;

// SILS: the handle carries its chip-select so a transaction routes to the right
// virtual chip. sils_board_spi_transfer() is defined in devices/virtual_board.cpp.
// SILS: ハンドルが CS を持ち、トランザクションを仮想チップへ振り分ける。
#include <stdlib.h>   // malloc/free for the device handle
struct spi_device_t { int cs; };
int sils_board_spi_transfer(int cs, const uint8_t* tx, uint8_t* rx, size_t nbytes);

/* ------------------------------------------------------------------------- */
/* Transaction / トランザクション                                             */
/* ------------------------------------------------------------------------- */

struct spi_transaction_t;  // forward decl for the callback typedef

// Pre/post transaction callback type (pre_cb / post_cb)
// トランザクション前後コールバック型（pre_cb / post_cb）
typedef void (*transaction_cb_t)(struct spi_transaction_t* trans);

// A single SPI transaction. Field order/names mirror ESP-IDF v5 so designated
// initializers in the firmware (.length, .tx_buffer, .rxlength, ...) compile.
// 単一 SPI トランザクション。フィールド順・名前は ESP-IDF v5 に合わせ、
// ファーム側の指定子初期化（.length 等）がそのままコンパイルできるようにする。
typedef struct spi_transaction_t {
    uint32_t    flags;       ///< Bitwise OR of SPI_TRANS_* flags
    uint16_t    cmd;         ///< Command value (length set by device cfg)
    uint64_t    addr;        ///< Address value (length set by device cfg)
    size_t      length;      ///< Total data length, in BITS
    size_t      rxlength;    ///< Receive length in BITS (0 => same as length)
    void*       user;        ///< User-defined value passed to callbacks
    union {
        const void* tx_buffer;  ///< Pointer to transmit buffer
        uint8_t     tx_data[4]; ///< Inline TX data (with SPI_TRANS_USE_TXDATA)
    };
    union {
        void*   rx_buffer;      ///< Pointer to receive buffer
        uint8_t rx_data[4];     ///< Inline RX data (with SPI_TRANS_USE_RXDATA)
    };
} spi_transaction_t;

/* ------------------------------------------------------------------------- */
/* Configuration structs / 設定構造体                                         */
/* ------------------------------------------------------------------------- */

// SPI bus configuration (pins + transfer limits + flags)
// SPI バス設定（ピン + 転送上限 + フラグ）
typedef struct {
    union {
        int mosi_io_num;     ///< MOSI / data0 GPIO (-1 if unused)
        int data0_io_num;
    };
    union {
        int miso_io_num;     ///< MISO / data1 GPIO (-1 if unused)
        int data1_io_num;
    };
    int sclk_io_num;         ///< SCLK GPIO (-1 if unused)
    union {
        int quadwp_io_num;   ///< WP / data2 GPIO (-1 if unused)
        int data2_io_num;
    };
    union {
        int quadhd_io_num;   ///< HD / data3 GPIO (-1 if unused)
        int data3_io_num;
    };
    int      data4_io_num;   ///< Data4 GPIO for octal (-1 if unused)
    int      data5_io_num;   ///< Data5 GPIO for octal (-1 if unused)
    int      data6_io_num;   ///< Data6 GPIO for octal (-1 if unused)
    int      data7_io_num;   ///< Data7 GPIO for octal (-1 if unused)
    int      max_transfer_sz;///< Max transfer size in bytes (0 => default)
    uint32_t flags;          ///< Bitwise OR of SPICOMMON_BUSFLAG_* flags
    int      isr_cpu_id;     ///< CPU affinity for the ISR
    int      intr_flags;     ///< Interrupt allocation flags
} spi_bus_config_t;

// Per-device interface configuration. Field set covers BMI270, PMW3901 and
// led_strip usage (.mode, .clock_speed_hz, .spics_io_num, .queue_size, .flags,
// .command_bits, .address_bits, .dummy_bits, .cs_ena_pretrans,
// .cs_ena_posttrans, .input_delay_ns, .clock_source, .pre_cb, .post_cb).
// デバイス毎のインターフェース設定。BMI270 / PMW3901 / led_strip が使う
// 全フィールドを網羅する。
typedef struct {
    uint8_t            command_bits;     ///< Default command phase length, in bits
    uint8_t            address_bits;     ///< Default address phase length, in bits
    uint8_t            dummy_bits;       ///< Dummy bits between addr and data
    uint8_t            mode;             ///< SPI mode 0..3 (CPOL, CPHA)
    spi_clock_source_t clock_source;     ///< Clock source select
    uint16_t           duty_cycle_pos;   ///< Duty cycle of positive clock (1/256)
    uint16_t           cs_ena_pretrans;  ///< CS active cycles before transmission
    uint8_t            cs_ena_posttrans; ///< CS active cycles after transmission
    int                clock_speed_hz;   ///< Clock speed, in Hz
    int                input_delay_ns;   ///< Max data-valid time after SCLK, in ns
    int                spics_io_num;     ///< CS GPIO (-1 if not used)
    uint32_t           flags;            ///< Bitwise OR of SPI_DEVICE_* flags
    int                queue_size;       ///< Transaction queue depth
    transaction_cb_t   pre_cb;           ///< Pre-transfer callback (may be NULL)
    transaction_cb_t   post_cb;          ///< Post-transfer callback (may be NULL)
} spi_device_interface_config_t;

/* ------------------------------------------------------------------------- */
/* Stub function bodies / スタブ関数本体                                       */
/* ------------------------------------------------------------------------- */

// Initialize an SPI bus on the given host. Inert on host: returns ESP_OK.
// 指定ホストの SPI バスを初期化。ホストでは無動作で ESP_OK を返す。
static inline esp_err_t spi_bus_initialize(spi_host_device_t host_id,
                                           const spi_bus_config_t* bus_config,
                                           spi_dma_chan_t dma_chan)
{
    (void)host_id;
    (void)bus_config;
    (void)dma_chan;
    return ESP_OK;
}

// Free a previously initialized SPI bus. Inert on host.
// 初期化済み SPI バスを解放。ホストでは無動作。
static inline esp_err_t spi_bus_free(spi_host_device_t host_id)
{
    (void)host_id;
    return ESP_OK;
}

// Add a device to an SPI bus; hands back a dummy non-NULL handle.
// SPI バスにデバイスを追加し、ダミーの非 NULL ハンドルを返す。
static inline esp_err_t spi_bus_add_device(spi_host_device_t host_id,
                                           const spi_device_interface_config_t* dev_config,
                                           spi_device_handle_t* handle)
{
    (void)host_id;
    if (handle != NULL) {
        // Allocate a handle that remembers the device's chip-select, so
        // spi_device_transmit can route the transaction to the right chip.
        // CS を覚えるハンドルを確保し、transmit で正しいチップへ振り分ける。
        struct spi_device_t* dev =
            (struct spi_device_t*)malloc(sizeof(struct spi_device_t));
        dev->cs = (dev_config != NULL) ? dev_config->spics_io_num : -1;
        *handle = dev;
    }
    return ESP_OK;
}

// Remove a device from its SPI bus. Inert on host.
// SPI バスからデバイスを取り外す。ホストでは無動作。
static inline esp_err_t spi_bus_remove_device(spi_device_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

// Queue + block until a transaction completes (interrupt-driven path).
// Zero-fills any receive buffer so callers read deterministic data.
// トランザクション完了までキュー+ブロック（割り込み駆動経路）。
// 受信バッファをゼロ埋めし、呼び出し側が決定的なデータを読めるようにする。
static inline esp_err_t spi_device_transmit(spi_device_handle_t handle,
                                            spi_transaction_t* trans)
{
    if (trans == NULL) return ESP_OK;
    // Route the full-duplex transfer to the device's virtual chip (by CS).
    // 全二重転送をデバイスの仮想チップ（CS）へ振り分ける。
    const uint8_t* tx = (trans->flags & SPI_TRANS_USE_TXDATA)
                            ? trans->tx_data : (const uint8_t*)trans->tx_buffer;
    uint8_t* rx = (trans->flags & SPI_TRANS_USE_RXDATA)
                      ? trans->rx_data : (uint8_t*)trans->rx_buffer;
    size_t nbytes = (size_t)(trans->length / 8);
    int cs = (handle != NULL) ? ((struct spi_device_t*)handle)->cs : -1;
    sils_board_spi_transfer(cs, tx, rx, nbytes);
    return ESP_OK;
}

// Polling variant of spi_device_transmit (busy-wait path, no ISR).
// spi_device_transmit のポーリング版（ビジーウェイト、ISR 不使用）。
static inline esp_err_t spi_device_polling_transmit(spi_device_handle_t handle,
                                                    spi_transaction_t* trans)
{
    return spi_device_transmit(handle, trans);
}

// Acquire exclusive use of the bus for the device. Inert on host.
// デバイスのためにバスを排他取得。ホストでは無動作。
static inline esp_err_t spi_device_acquire_bus(spi_device_handle_t device,
                                               int wait)
{
    (void)device;
    (void)wait;
    return ESP_OK;
}

// Release a bus previously acquired with spi_device_acquire_bus().
// spi_device_acquire_bus() で取得したバスを解放。
static inline void spi_device_release_bus(spi_device_handle_t device)
{
    (void)device;
}

// Report the device's actual SPI clock; writes a plausible value (kHz).
// デバイスの実 SPI クロックを返す。妥当な値（kHz）を書き込む。
static inline esp_err_t spi_device_get_actual_freq(spi_device_handle_t handle,
                                                   int* freq_khz)
{
    (void)handle;
    if (freq_khz != NULL) {
        // 2.5 MHz default (matches led_strip's expected resolution window).
        // 2.5 MHz 既定（led_strip の期待解像度ウィンドウに合致）。
        *freq_khz = 2500;
    }
    return ESP_OK;
}

#ifdef __cplusplus
}  // extern "C"
#endif
