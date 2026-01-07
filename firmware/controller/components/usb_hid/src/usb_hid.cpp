// USB HID Joystick implementation
// USB HID ジョイスティック実装

#include "usb_hid.hpp"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char* TAG = "usb_hid";

// ============================================================================
// USB Descriptors
// USBディスクリプタ
// ============================================================================

// HIDレポートディスクリプタ（Joystick - 4軸 + 8ボタン）
// HID Report Descriptor (Joystick - 4 axes + 8 buttons)
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xa1, 0x01,        // COLLECTION (Application)

    // 4軸アナログ (Throttle, Roll, Pitch, Yaw)
    // 4 analog axes
    0x09, 0x30,        //   USAGE (X) - Throttle
    0x09, 0x31,        //   USAGE (Y) - Roll
    0x09, 0x32,        //   USAGE (Z) - Pitch
    0x09, 0x33,        //   USAGE (Rx) - Yaw
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x04,        //   REPORT_COUNT (4)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // ボタン (8個)
    // Buttons (8)
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,        //   USAGE_MAXIMUM (Button 8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // 予約バイト
    // Reserved byte
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x81, 0x03,        //   INPUT (Const,Var,Abs)

    0xc0               // END_COLLECTION
};

// デバイスディスクリプタ
// Device Descriptor
static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,    // USB 2.0
    .bDeviceClass       = 0x00,      // Defined in interface
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,    // Espressif VID
    .idProduct          = 0x8001,    // StampFly Controller PID
    .bcdDevice          = 0x0100,    // Version 1.0
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// コンフィギュレーションディスクリプタ
// Configuration Descriptor
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID           0x81

static const uint8_t configuration_descriptor[] = {
    // Configuration Descriptor
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0, 100),
    // HID Descriptor (Interface 0)
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(hid_report_descriptor), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 10)
};

// 文字列ディスクリプタ
// String Descriptors
static const char* string_descriptor[] = {
    (const char[]){0x09, 0x04},  // 0: Language (English)
    "StampFly",                   // 1: Manufacturer
    "Controller",                 // 2: Product
    "123456",                     // 3: Serial Number
    "HID Interface"               // 4: HID Interface
};

// ============================================================================
// TinyUSB HIDコールバック
// TinyUSB HID callbacks
// ============================================================================

// HIDディスクリプタ取得コールバック（TinyUSBから呼ばれる）
// HID descriptor callback (called by TinyUSB)
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return hid_report_descriptor;
}

// GET_REPORT コールバック
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t* buffer,
                                uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

// SET_REPORT コールバック
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const* buffer,
                           uint16_t bufsize) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

// ============================================================================
// USB HID API
// ============================================================================

// USB HID初期化
// Initialize USB HID
esp_err_t usb_hid_init(void) {
    ESP_LOGI(TAG, "USB HID初期化開始 / Initializing USB HID");

    // TinyUSB設定（カスタムディスクリプタを指定）
    // TinyUSB configuration with custom descriptors
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &device_descriptor,
        .string_descriptor = string_descriptor,
        .string_descriptor_count = sizeof(string_descriptor) / sizeof(string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = configuration_descriptor,
        .self_powered = false,
        .vbus_monitor_io = -1,
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSBドライバインストール失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "USB HID初期化完了 / USB HID initialized");
    return ESP_OK;
}

// USB HID終了
// Deinitialize USB HID
void usb_hid_deinit(void) {
    ESP_LOGI(TAG, "USB HID終了 / USB HID deinitialized");
    // TinyUSBには明示的なdeinit APIがない
    // TinyUSB doesn't have explicit deinit API
}

// レポート送信
// Send HID report
bool usb_hid_send_report(const HIDJoystickReport* report) {
    if (!tud_mounted()) {
        return false;
    }
    if (!tud_hid_ready()) {
        return false;
    }

    return tud_hid_report(0, report, sizeof(HIDJoystickReport));
}

// 接続状態取得
// Get connection status
bool usb_hid_is_mounted(void) {
    return tud_mounted();
}
