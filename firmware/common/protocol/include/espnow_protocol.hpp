/**
 * @file espnow_protocol.hpp
 * @brief ESP-NOW communication protocol definition for StampFly
 *        StampFly用ESP-NOW通信プロトコル定義
 *
 * This header defines the packet structures and constants used for the
 * primary ESP-NOW + TDMA link between Controller and Vehicle: ControlPacket
 * (Controller -> Vehicle) and the PairingPacket constants (Vehicle -> Controller,
 * broadcast). Wire layout is the Single Source of Truth documented in
 * protocol/spec/messages.yaml; this header is its C++ implementation, shared
 * so the layout is defined exactly once instead of independently duplicated
 * in each firmware.
 * Controller-Vehicle間の主系統ESP-NOW+TDMAリンクで使うパケット構造と定数を定義。
 * ControlPacket(Controller→Vehicle)とPairingPacket定数(Vehicle→Controller、
 * broadcast)。電文レイアウトの単一の真実(SSOT)は protocol/spec/messages.yaml
 * に記載されており、本ヘッダはそのC++実装。各ファームで個別に重複定義するのではなく、
 * ここ1箇所だけでレイアウトを定義する。
 *
 * @note This file is shared between Vehicle and Controller firmware.
 *       このファイルはVehicleとControllerのファームウェアで共有される。
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace stampfly {
namespace protocol {

/**
 * @brief Control packet from controller to vehicle (14 bytes, ESP-NOW).
 *        コントローラから機体への制御パケット(14バイト、ESP-NOW)
 *
 * Layout (packed, little-endian, matches protocol/spec/messages.yaml):
 *   [0..2]  drone_mac  uint8x3  destination MAC lower 3 bytes
 *   [3..4]  throttle   uint16   12-bit ADC, 2048 = zero (spring-centred)
 *   [5..6]  roll       uint16   12-bit ADC, 2048 = center
 *   [7..8]  pitch      uint16   12-bit ADC, 2048 = center
 *   [9..10] yaw        uint16   12-bit ADC, 2048 = center
 *   [11]    flags      uint8    bit0=ARM,1=FLIP,2=MODE,3=ALT_MODE,4=POS_MODE
 *   [12]    reserved   uint8    proactive_flag (ignored)
 *   [13]    checksum   uint8    low 8 bits of the sum of bytes [0..12]
 */
struct ControlPacket {
    uint8_t  drone_mac[3];
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint8_t  flags;
    uint8_t  reserved;
    uint8_t  checksum;
} __attribute__((packed));

static_assert(sizeof(ControlPacket) == 14,
              "ControlPacket must match the protocol SSOT (14 bytes)");

// Control packet flags / 制御パケットフラグ
constexpr uint8_t CTRL_FLAG_ARM      = 0x01;
constexpr uint8_t CTRL_FLAG_FLIP     = 0x02;
constexpr uint8_t CTRL_FLAG_MODE     = 0x04;
constexpr uint8_t CTRL_FLAG_ALT_MODE = 0x08;
constexpr uint8_t CTRL_FLAG_POS_MODE = 0x10;

/**
 * @brief PairingPacket constants (11 bytes, broadcast by the vehicle).
 *        PairingPacket定数(11バイト、機体からbroadcast)
 *
 * Layout: [0] channel, [1..6] drone_mac (full 6 bytes), [7..10] signature.
 * レイアウト: [0] channel, [1..6] drone_mac(フル6バイト), [7..10] signature。
 */
constexpr size_t  kPairingPacketSize = 11;
constexpr uint8_t kPairingSignature[4] = {0xAA, 0x55, 0x16, 0x88};

/**
 * @brief Low 8 bits of the byte sum (ControlPacket checksum, byte 13).
 *        バイト総和の下位8ビット(ControlPacketのチェックサム、byte 13)
 */
inline uint8_t checksum8(const uint8_t* data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

}  // namespace protocol
}  // namespace stampfly
