#include "workshop_api.hpp"
#include <cmath>

// Tick counter for decimation
// デシメーション用のtickカウンタ
static uint32_t tick = 0;

void setup()
{
    ws::print("Lesson 10: API Overview & App Development");

    // TODO: Set your WiFi channel (1, 6, or 11)
    // TODO: 自分のWiFiチャンネルを設定する（1, 6, 11のいずれか）
    // ws::set_channel(1);
}

void loop_400Hz(float dt)
{
    tick++;

    // Decimation: output at 50 Hz (every 8 ticks at 400 Hz)
    // デシメーション: 50Hz出力（400Hzの8tick毎）
    if (tick % 8 != 0) return;

    // TODO: Get barometric data and output as Teleplot
    // TODO: 気圧データを取得してTeleplot形式で出力
    // ws::print(">baro_alt:%.2f", ws::baro_altitude());

    // TODO: Get ToF (Time-of-Flight) bottom distance
    // TODO: ToF下向き距離を取得
    // (Note: front ToF is not used by the vehicle sensing pipeline / 前方ToFはvehicleのセンシングでは未使用)
    // ws::print(">tof_bottom:%.3f", ws::tof_bottom());

    // TODO: Get ESKF estimation values (roll/pitch/yaw/altitude)
    // TODO: ESKF推定値を取得（ロール/ピッチ/ヨー/高度）
    // ws::print(">eskf_roll:%.1f", ws::estimated_roll() * 57.3f);
    // ws::print(">eskf_alt:%.2f", ws::estimated_altitude());

    // TODO: Get magnetic data and compute heading
    // TODO: 磁気データを取得して方位角を計算
    // float heading = atan2f(-ws::mag_y(), ws::mag_x()) * 57.3f;
    // ws::print(">heading:%.1f", heading);

    // TODO: Get optical flow velocity
    // TODO: 光学フロー速度を取得
    // ws::print(">flow_vx:%.3f", ws::flow_vx());
    // ws::print(">flow_vy:%.3f", ws::flow_vy());

    // TODO: Get battery voltage
    // TODO: バッテリ電圧を取得
    // ws::print(">voltage:%.2f", ws::battery_voltage());
}
