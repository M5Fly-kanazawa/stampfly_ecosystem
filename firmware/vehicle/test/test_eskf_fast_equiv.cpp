/**
 * @file test_eskf_fast_equiv.cpp
 * @brief Bit-exact equivalence driver for the SF_EXP_ESKF_FAST switch of the ESKF core.
 *        ESKF コアの SF_EXP_ESKF_FAST スイッチの「ビット同一性」確認ドライバ。
 *
 * Built TWICE (-DSF_EXP_ESKF_FAST=0 and =1, see Makefile target eskf_fast_equiv). Each binary
 * feeds the SAME pseudo-random sensor sequence (still, rotation, acceleration; IMU 400 Hz, flow
 * 100 Hz, ToF 50 Hz, mag/baro at their own rates; plus mid-run reset / setConfig / sensor
 * switching / flip-hold / bias freeze) to EskfCore and, after EVERY call, writes an FNV-1a hash of
 * the raw bytes of the nominal state (pos, vel, q, bg, ba, angular rate), the active mask and ALL
 * 225 elements of P. The two hash files must be byte-identical (cmp).
 * Host libm differs from the ESP32 one: this proves algorithmic identity, not the device bits.
 * 2 回ビルド（SF_EXP_ESKF_FAST=0/1）し、同一の擬似乱数センサ列を与え、毎呼び出し後に名目状態・
 * アクティブマスク・P 全 225 要素の生バイト列のハッシュを出力する。両ファイルが一致すること（cmp）。
 * ホストの libm は ESP32 と異なるため、これはアルゴリズム上の同一性の確認で、実機のビット列
 * そのものの確認ではない。
 *
 * Usage: test_eskf_fast_equiv <hash_output_file>
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define private public   // test-only: read the private covariance P_ / 試験専用: private の P_ を読む
#include "eskf_core.hpp"
#undef private

using namespace sf;

static uint32_t g_rng = 12345u;
static float uniform()   // [-1, 1)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return static_cast<float>(g_rng >> 8) / 8388608.0f - 1.0f;
}

static uint64_t g_hash = 0;
static FILE* g_out = nullptr;
static uint64_t g_calls = 0;

static void mix(const void* p, size_t n)
{
    const uint8_t* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < n; i++) { g_hash ^= b[i]; g_hash *= 1099511628211ull; }
}

static void snapshot(const EskfCore& c)
{
    g_hash = 1469598103934665603ull;
    mix(&c.pos_, sizeof(c.pos_));   mix(&c.vel_, sizeof(c.vel_));
    mix(&c.q_, sizeof(c.q_));       mix(&c.bg_, sizeof(c.bg_));
    mix(&c.ba_, sizeof(c.ba_));     mix(&c.ang_rate_, sizeof(c.ang_rate_));
    mix(&c.active_mask_, sizeof(c.active_mask_));
    mix(c.P_.data, sizeof(c.P_.data));
    mix(&c.last_tof_, sizeof(c.last_tof_));
    fwrite(&g_hash, sizeof(g_hash), 1, g_out);
    g_calls++;
}

struct Scenario { bool lpf; bool mag; bool baro; bool flow; bool comp; const char* name; };

static void run(const Scenario& sc, int steps)
{
    EskfConfig cfg;
    cfg.accel_att_lpf_hz = sc.lpf ? 30.0f : 0.0f;
    cfg.use_mag = sc.mag; cfg.use_baro = sc.baro; cfg.use_flow = sc.flow;
    cfg.accel_comp_enable = sc.comp;
    EskfCore core;
    core.init(cfg);
    snapshot(core);
    const float dt = 1.0f / 400.0f;
    float roll = 0, pitch = 0, height = 0.0f, vx = 0, vy = 0;
    for (int s = 0; s < steps; s++) {
        const float t = s * dt;
        // Phases: still / rotating / accelerating / mixed. 静止・回転・加速・混合
        const int phase = (s / 4000) % 4;
        float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
        if (phase == 1 || phase == 3) { gx = 1.5f * sinf(2.0f * t); gy = 1.2f * cosf(3.1f * t); gz = 0.8f * sinf(0.7f * t); }
        if (phase == 2 || phase == 3) { ax = 3.0f * sinf(1.3f * t); ay = 2.5f * cosf(0.9f * t); az = 2.0f * sinf(2.2f * t); }
        roll += gx * dt; pitch += gy * dt;
        height = 0.5f + 0.3f * sinf(0.5f * t);
        vx = ax * 0.1f; vy = ay * 0.1f;
        const Vec3 gyro(gx + 0.01f * uniform(), gy + 0.01f * uniform(), gz + 0.01f * uniform());
        const Vec3 accel(ax - 9.80665f * sinf(pitch) + 0.3f * uniform(),
                         ay + 9.80665f * sinf(roll) + 0.3f * uniform(),
                         az - 9.80665f * cosf(roll) * cosf(pitch) + 0.3f * uniform());

        if (s == 1000) core.setAttitudeFromGravity(accel);
        core.predict(accel, gyro, dt);                 snapshot(core);
        core.updateAccelAttitude(accel);               snapshot(core);
        if (s % 4 == 0) {   // flow 100 Hz
            core.updateFlowRaw(static_cast<int16_t>(vx * 40 + 3 * uniform()),
                               static_cast<int16_t>(vy * 40 + 3 * uniform()),
                               height > 0.05f ? height : 0.05f, 0.01f, gx, gy,
                               static_cast<uint8_t>(s % 97 < 3 ? 5 : 80));
            snapshot(core);
        }
        if (s % 8 == 0) {   // ToF 50 Hz (+ occasional outlier)
            core.updateToF(height + 0.02f * uniform() + (s % 1999 == 0 ? 1.0f : 0.0f)); snapshot(core);
            core.updateToFVelocity(height + 0.02f * uniform(), 0.02f);                   snapshot(core);
        }
        if (s % 40 == 0) {  // baro + mag 10 Hz
            core.updateBaro(height + 0.05f * uniform());                                  snapshot(core);
            core.updateMag(Vec3(20.0f + 2.0f * uniform(), 3.0f * uniform(), 40.0f + 2.0f * uniform())); snapshot(core);
        }
        // Mid-run events / 実行中のイベント
        if (s == 9000)  { core.holdAttitudeCorrection(true);  snapshot(core); }
        if (s == 9400)  { core.holdAttitudeCorrection(false); snapshot(core); }
        if (s == 11000) { core.setFreezeAccelBias(true);      snapshot(core); }
        if (s == 12000) { core.setFreezeAccelBias(false);     snapshot(core); }
        if (s == 13000) { core.setSensorEnabled(3, false);    snapshot(core); }
        if (s == 14000) { core.setSensorEnabled(3, true);     snapshot(core); }
        if (s == 15000) { EskfConfig c2 = cfg; c2.accel_att_lpf_hz = sc.lpf ? 12.0f : 5.0f; core.setConfig(c2); snapshot(core); }
        if (s == 17000) { core.setConfig(cfg);                snapshot(core); }
        if (s == 18000) { core.inflateCovariance(0x0038);     snapshot(core); }
        if (s == 19000) { core.reset();                       snapshot(core); }
    }
    printf("  scenario %-28s steps=%d  final q=(%.3f %.3f %.3f %.3f) P00=%.3e finite=%d\n", sc.name, steps,
           static_cast<double>(core.q_.w), static_cast<double>(core.q_.x), static_cast<double>(core.q_.y),
           static_cast<double>(core.q_.z), static_cast<double>(core.P_(0, 0)),
           std::isfinite(core.P_(0, 0)) && std::isfinite(core.q_.w) ? 1 : 0);
}

int main(int argc, char** argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <hash_file>\n", argv[0]); return 2; }
    g_out = fopen(argv[1], "wb");
    if (!g_out) return 2;
    printf("SF_EXP_ESKF_FAST=%d\n", SF_EXP_ESKF_FAST);
    const Scenario scs[] = {
        {false, false, false, true,  true,  "default"},
        {true,  false, false, true,  true,  "lpf30"},
        {true,  true,  true,  true,  true,  "lpf30+mag+baro"},
        {true,  true,  false, false, false, "lpf30+mag, no flow, no comp"},
        {false, true,  true,  true,  false, "mag+baro, no comp"},
    };
    for (const Scenario& sc : scs) run(sc, 24000);
    fclose(g_out);
    printf("  snapshots hashed: %llu\n", static_cast<unsigned long long>(g_calls));
    return 0;
}
