// drive_rout_table.hpp -- R_out(duty) of the averaged capacitor-recirculation drive model.
// Generated from rout/rout_table_c01.csv (D = 0.01 ... 1.00 step 0.01, ohms). Do not edit by hand.
// コンデンサ還流方式の平均化駆動モデルの出力抵抗 R_out(duty) 表（D=0.01〜1.00、0.01 刻み、単位 Ω）。
#pragma once
#include <cstddef>

namespace sils_drive {

constexpr std::size_t kRoutCount = 100;   // entries for D = 0.01 .. 1.00
constexpr float kRoutDutyMin = 0.01f;     // below this the motor current is 0
constexpr float kRoutDutyStep = 0.01f;

constexpr float kRoutOhm[kRoutCount] = {
    65.370970f, 61.559885f, 56.301752f, 50.636193f, 45.182287f,
    40.227661f, 35.860443f, 32.067839f, 28.794401f, 25.972153f,
    23.534522f, 21.421883f, 19.583067f, 17.975096f, 16.562249f,
    15.314956f, 14.208752f, 13.223355f, 12.341901f, 11.550307f,
    10.836764f, 10.191320f, 9.605546f, 9.072269f, 8.585354f,
    8.139528f, 7.730236f, 7.353525f, 7.005948f, 6.684484f,
    6.386479f, 6.109583f, 5.851716f, 5.611024f, 5.385852f,
    5.174720f, 4.976301f, 4.789402f, 4.612956f, 4.446002f,
    4.287686f, 4.137242f, 3.993997f, 3.857354f, 3.726797f,
    3.601880f, 3.482225f, 3.367518f, 3.257502f, 3.151975f,
    3.050783f, 2.953814f, 2.860993f, 2.772274f, 2.687632f,
    2.607062f, 2.530565f, 2.458144f, 2.389798f, 2.325513f,
    2.265257f, 2.208976f, 2.156582f, 2.107952f, 2.062921f,
    2.021277f, 1.982754f, 1.947030f, 1.913721f, 1.882382f,
    1.852504f, 1.823518f, 1.794805f, 1.765705f, 1.735539f,
    1.703632f, 1.669346f, 1.632117f, 1.591494f, 1.547176f,
    1.499050f, 1.447206f, 1.391954f, 1.333806f, 1.273459f,
    1.211750f, 1.149609f, 1.088007f, 1.027903f, 0.970206f,
    0.915740f, 0.865228f, 0.819285f, 0.778420f, 0.743049f,
    0.713509f, 0.690074f, 0.672967f, 0.662362f, 0.658380f,
};

// Linear interpolation in duty; duty clamped to [0.01, 1.0] (caller handles duty < 0.01 -> I = 0).
// duty の線形補間。[0.01, 1.0] にクランプ（duty<0.01 は呼び出し側で電流 0）。
inline float routOhm(float duty)
{
    if (duty <= kRoutDutyMin) return kRoutOhm[0];
    if (duty >= 1.0f) return kRoutOhm[kRoutCount - 1];
    const float pos = (duty - kRoutDutyMin) / kRoutDutyStep;
    std::size_t i = static_cast<std::size_t>(pos);
    if (i >= kRoutCount - 1) i = kRoutCount - 2;
    const float frac = pos - static_cast<float>(i);
    return kRoutOhm[i] + frac * (kRoutOhm[i + 1] - kRoutOhm[i]);
}

}  // namespace sils_drive
