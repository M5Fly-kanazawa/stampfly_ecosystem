#pragma once

#include <cstdint>
#include "esp_err.h"

// HIDレポート構造体
// HID Report structure
#pragma pack(push, 1)
typedef struct {
    uint8_t throttle;    // 0-255 (12bit → 8bit)
    uint8_t roll;        // 0-255
    uint8_t pitch;       // 0-255
    uint8_t yaw;         // 0-255
    uint8_t buttons;     // bit0:Arm, bit1:Flip, bit2:Mode, bit3:AltMode
    uint8_t reserved;
} HIDJoystickReport;
#pragma pack(pop)

// USB HID初期化
// Initialize USB HID
esp_err_t usb_hid_init(void);

// USB HID終了
// Deinitialize USB HID
void usb_hid_deinit(void);

// レポート送信
// Send HID report
bool usb_hid_send_report(const HIDJoystickReport* report);

// 接続状態取得
// Get connection status
bool usb_hid_is_mounted(void);

// 12bit → 8bit 変換ヘルパー
// 12bit to 8bit conversion helper
inline uint8_t convert_12bit_to_8bit(uint16_t value_12bit) {
    // 0-4095 → 0-255
    return (uint8_t)(value_12bit >> 4);
}
