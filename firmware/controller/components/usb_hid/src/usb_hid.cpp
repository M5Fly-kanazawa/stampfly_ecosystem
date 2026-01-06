// USB HID Joystick implementation
// USB HID ジョイスティック実装

#include "usb_hid.hpp"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char* TAG = "usb_hid";

// HIDレポートディスクリプタ（Joystick）
// HID Report Descriptor (Joystick)
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xa1, 0x01,        // COLLECTION (Application)

    // 4軸アナログ
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

// TinyUSB HIDコールバック
// TinyUSB HID callbacks

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

// USB HID初期化
// Initialize USB HID
esp_err_t usb_hid_init(void) {
    ESP_LOGI(TAG, "USB HID初期化開始 / Initializing USB HID");

    // TinyUSB設定
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = nullptr,
        .string_descriptor = nullptr,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = nullptr,
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

// HIDディスクリプタ取得コールバック（TinyUSBから呼ばれる）
// HID descriptor callback (called by TinyUSB)
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return hid_report_descriptor;
}
