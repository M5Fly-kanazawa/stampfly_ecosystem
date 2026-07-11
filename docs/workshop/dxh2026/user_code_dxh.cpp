// DXH workshop exercise template (copied into firmware/workshop/main/user_code.cpp
// by the loaner-PC setup; participants only edit the duty values on the day).
// DXH講座 実習コードテンプレート（貸出PCセットアップ時に
// firmware/workshop/main/user_code.cpp へコピーする。当日参加者は Duty 値のみ書き換える）
#include "workshop_api.hpp"

// =========================================================================
// Motor duty test (hardcoded)
// 各モータの Duty をハードコードする
// =========================================================================
//
// Motor IDs / モータ ID:
//   1 = FR (右前)   2 = RR (右後)
//   3 = RL (左後)   4 = FL (左前)
//
// Duty range: 0.0 - 1.0
//   0.0  = stop / 止める
//   0.10 = spin at 10% / 10% で回す
//   このチュートリアルでは 0.15 を上限とする

void setup()
{
    ws::print("Motor duty test");

    // Do NOT auto-arm in code. Arming is controlled by the controller's
    // ARM button or a single click of the vehicle's onboard button, so
    // motors never spin until you intentionally press it.
    // コードでは自動 ARM しない。アーム/解除はコントローラの ARM ボタン
    // または機体本体ボタンの単クリックで行うので、意図的に押すまで
    // モータは絶対に回らない
}

void loop_400Hz(float dt)
{
    // ----- Set each motor's duty here / 各モータの Duty をここで設定 -----
    ws::motor_set_duty(1, 0.10f);   // FR (右前)
    ws::motor_set_duty(2, 0.00f);   // RR (右後)
    ws::motor_set_duty(3, 0.00f);   // RL (左後)
    ws::motor_set_duty(4, 0.00f);   // FL (左前)
    // -------------------------------------------------------------------
}
