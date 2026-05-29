// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Kouhei Ito
//
// M7 differential diagnosis — real-flight-log replay through the ESKF.
//
// Replays a recorded sensor stream (produced by log_to_replay.py from a real
// firmware/vehicle flight) through the SAME legacy ESKF (stampfly::ESKF) the
// firmware ran, AS AN OPEN-LOOP OBSERVER (not in any control loop), and compares
// the SIL estimate against the firmware's OWN logged ESKF output. Because the
// SIL links the unmodified firmware eskf.cpp by reference (Code Identity), the
// logic of differential diagnosis applies:
//   * SIL reproduces the logged estimate  -> estimator is software-deterministic;
//     any residual localizes to HW timing / sampling jitter / concurrency / comm
//     (factors the clean uniform-rate replay does NOT model).
//   * SIL does NOT reproduce it           -> the behavior depends on something
//     outside the logged ESKF inputs (a HW glitch, a corrupted sample, timing).
// The new redesign (sf::EskfCore) is driven alongside as a bonus "what would the
// redesign have estimated on the same real sensors" A/B.
//
// M7 差分診断 — 実機フライトログをESKFに再生。
//
// log_to_replay.py が実機(firmware/vehicle)飛行から生成した記録センサ列を、
// ファームが動かしたのと同じ旧ESKF(stampfly::ESKF)に、オープンループ観測として
// (どの制御ループにも入れず)再生し、SIL推定をファーム自身のログESKF出力と照合する。
// SILは無改変のファーム eskf.cpp を参照リンクする(Code Identity)ため、差分診断の
// 論理が成立する:
//   * SILがログ推定を再現 -> 推定器はソフト的に決定論的; 残差はHWタイミング・
//     サンプリングジッタ・並行性・通信(一様レートのクリーン再生が表現しない要因)
//     に局在する。
//   * SILが再現しない     -> その挙動はログESKF入力の外の何か(HWグリッチ・破損
//     サンプル・タイミング)に依存する。
// 新設計(sf::EskfCore)を並走駆動し「同じ実機センサで新設計なら何を推定したか」の
// A/B をボーナスで提示する。
//
// @design project_sil_architecture (memory) §differential diagnosis (pillar 2) [--]
// @design development_roadmap.md §M7 — real-log injection, sim vs flight residual [--]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>

// New-side core (reused unmodified — Code Identity)
// 新側コア(無改変で再利用 — Code Identity)
#include "sf_math.hpp"
#include "eskf_core.hpp"   // new ESKF (sf::EskfCore) — redesign, bonus A/B

// Legacy-side: the ESKF that actually flew this log (firmware/vehicle)
// 旧側: このログを実際に飛ばしたESKF(firmware/vehicle)
#include "eskf.hpp"

using namespace sf;
using namespace sf::math;

namespace {

// -----------------------------------------------------------------------------
// Constants — all named, matching the legacy firmware's fixed-rate fusion loop.
// 定数 — 全て命名、旧ファームの固定レート融合ループに一致。
// -----------------------------------------------------------------------------
constexpr float DT       = 0.0025f;     // 400 Hz IMU/ESKF step [s]
constexpr float FLOW_DT  = 0.01f;       // dt the firmware passes to updateFlowRaw
constexpr float TOF_DT_NOMINAL = 1.0f / 30.0f;  // first ToF-velocity dt fallback
constexpr float GRAVITY  = 9.81f;       // [m/s^2] (matches config::eskf::GRAVITY)
constexpr float RAD2DEG  = 57.2957795f;
constexpr float DEFAULT_SKIP_S = 6.0f;  // skip warm-start + takeoff transient

// Firmware ground/takeoff gating (firmware/vehicle imu_task.cpp + config.hpp).
// The legacy firmware holds position/velocity at zero on the ground and resets
// the position covariance at takeoff; flow/ToF resume only in flight after a
// short skip. Replicating this is what keeps the velocity channel healthy.
// ファームの接地/離陸ゲート(imu_task.cpp + config.hpp)。接地中は位置・速度を0保持、
// 離陸時に位置共分散をリセット、飛行中にスキップ後フロー/ToFを再開。これの再現が
// 速度チャネルを健全に保つ鍵。
constexpr double TAKEOFF_EPS    = 1e-3;  // |ref pos|+|ref vel| > this => airborne
constexpr int    FLOW_TAKEOFF_SKIP = 20; // skip first 20 flow updates after takeoff
constexpr int    TOF_TAKEOFF_SKIP  = 10; // skip first 10 ToF updates after takeoff
constexpr int    FLOW_SQUAL_MIN = 0x19;  // config::flow::FLOW_SQUAL_MIN (=25)
constexpr float  FLOW_DIST_MIN  = 0.02f; // config::flow::FLOW_DISTANCE_MIN [m]
constexpr float  FLOW_DIST_MAX  = 4.0f;  // config::flow::FLOW_DISTANCE_MAX [m]

// Column layout of the replay CSV — MUST match log_to_replay.py COLUMNS.
// 再生CSVの列レイアウト — log_to_replay.py の COLUMNS と一致必須。
enum Col {
    C_TS = 0,
    C_GX, C_GY, C_GZ,
    C_AX, C_AY, C_AZ,
    C_FLOW_NEW, C_FDX, C_FDY, C_FQ,
    C_TOF_NEW, C_TOF,
    C_RQW, C_RQX, C_RQY, C_RQZ,
    C_RPX, C_RPY, C_RPZ,
    C_RVX, C_RVY, C_RVZ,
    C_RBGX, C_RBGY, C_RBGZ,
    C_RBAX, C_RBAY, C_RBAZ,
    NCOL
};

// -----------------------------------------------------------------------------
// Vec3 (sf::math) -> Vector3 (stampfly::math): identical x/y/z, both NED.
// Vec3(sf::math) -> Vector3(stampfly::math): x/y/z 同名、両 NED。
// -----------------------------------------------------------------------------
stampfly::math::Vector3 toOld(const Vec3& v)
{
    return stampfly::math::Vector3(v.x, v.y, v.z);
}

// -----------------------------------------------------------------------------
// Quaternion [w,x,y,z] -> ZYX Euler (rad). ONE convention applied uniformly to
// the reference, legacy, and redesign quaternions so the angles are comparable.
// クォータニオン[w,x,y,z] -> ZYXオイラー(rad)。参照・旧・新の全てに同一規約を
// 適用し角度を比較可能にする。
// -----------------------------------------------------------------------------
void quatToEuler(float w, float x, float y, float z,
                 float& roll, float& pitch, float& yaw)
{
    roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float s = 2.0f * (w * y - z * x);
    s = (s > 1.0f) ? 1.0f : (s < -1.0f ? -1.0f : s);
    pitch = std::asin(s);
    yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

// Wrap an angle difference to [-pi, pi] so yaw RMSE handles wrap-around.
// 角度差を [-pi, pi] に折り返し、ヨーRMSEのラップアラウンドを処理。
float wrapPi(float a)
{
    while (a >  M_PI) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -M_PI) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

// -----------------------------------------------------------------------------
// Legacy ESKF config to match the firmware/vehicle default (USE_OPTICAL_FLOW =
// true, USE_TOF = true, USE_BAROMETER = false, USE_MAGNETOMETER = false; flow
// chi2 gate left at the firmware default = ON). See imu_task.cpp / config.hpp.
// 旧ESKF設定をファーム(firmware/vehicle)既定に一致(flow ON, tof ON, baro/mag OFF;
// flow χ²ゲートはファーム既定=ON)。imu_task.cpp / config.hpp 参照。
// -----------------------------------------------------------------------------
stampfly::ESKF::Config makeLegacyConfig()
{
    stampfly::ESKF::Config cfg = stampfly::ESKF::Config::defaultConfig();
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_MAG]  = false;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_BARO] = false;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_TOF]  = true;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_FLOW] = true;
    return cfg;
}

// New ESKF config — mirrors integrated_sil's validated firmware redesign config.
// 新ESKF設定 — integrated_sil の検証済みファーム redesign 設定を踏襲。
sf::EskfConfig makeNewConfig()
{
    sf::EskfConfig cfg;
    cfg.use_tof  = true;
    cfg.use_baro = false;
    cfg.use_mag  = false;
    cfg.use_flow = true;
    cfg.accel_noise = 0.3f;
    cfg.gyro_noise  = 0.009655f;
    cfg.tof_noise   = 0.01f;
    return cfg;
}

// -----------------------------------------------------------------------------
// Parse one CSV line into NCOL doubles. Returns false on a short/empty line.
// CSV1行を NCOL 個の double へ解析。短い/空行なら false。
// -----------------------------------------------------------------------------
bool parseRow(char* line, double out[NCOL])
{
    int n = 0;
    char* save = nullptr;
    for (char* tok = strtok_r(line, ",", &save);
         tok && n < NCOL;
         tok = strtok_r(nullptr, ",", &save)) {
        out[n++] = atof(tok);
    }
    return n == NCOL;
}

// Running mean-square accumulator for per-axis RMSE.
// 軸別RMSE用の二乗平均アキュムレータ。
struct Rms {
    double sum = 0.0;
    long   n = 0;
    void add(double err) { sum += err * err; ++n; }
    double value() const { return n ? std::sqrt(sum / static_cast<double>(n)) : 0.0; }
};

}  // namespace

// =============================================================================
// Main: read the replay CSV, drive legacy (primary) + redesign (bonus) ESKFs as
// open-loop observers, write a per-cycle comparison CSV (stdout), report RMSE
// and the differential-diagnosis verdict (stderr).
// メイン: 再生CSVを読み、旧(主)+新(ボーナス)ESKFをオープンループ観測で駆動、
// 周期別比較CSV(stdout)を書き、RMSEと差分診断の判定(stderr)を報告。
// =============================================================================
int main(int argc, char** argv)
{
    const char* input = nullptr;
    float skip_s = DEFAULT_SKIP_S;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--input") && i + 1 < argc) {
            input = argv[++i];
        } else if (!strcmp(argv[i], "--skip") && i + 1 < argc) {
            skip_s = static_cast<float>(atof(argv[++i]));
        } else {
            fprintf(stderr, "usage: %s --input <replay.csv> [--skip <sec>]\n", argv[0]);
            return 2;
        }
    }
    if (!input) {
        fprintf(stderr, "usage: %s --input <replay.csv> [--skip <sec>]\n", argv[0]);
        return 2;
    }

    FILE* fp = fopen(input, "r");
    if (!fp) { fprintf(stderr, "cannot open %s\n", input); return 1; }

    // Skip the CSV header line.
    // CSVヘッダ行をスキップ。
    char* line = nullptr;
    size_t cap = 0;
    if (getline(&line, &cap, fp) < 0) {
        fprintf(stderr, "empty file\n"); fclose(fp); free(line); return 1;
    }

    // --- Initialize both ESKFs ---
    stampfly::ESKF eskf;          // legacy (under test)
    eskf.init(makeLegacyConfig());
    sf::EskfCore neweskf;         // redesign (bonus A/B)
    neweskf.init(makeNewConfig());

    bool warm = false;
    double t0_us = 0.0;
    double last_tof_us = 0.0;

    // Ground/takeoff gating state (mirrors the firmware's flight-phase logic).
    // 接地/離陸ゲートの状態(ファームの飛行局面ロジックを反映)。
    bool taken_off = false;
    int  flow_skip = 0;
    int  tof_skip = 0;

    // RMSE accumulators (legacy "old" + redesign "new" vs reference)
    // RMSEアキュムレータ(旧 + 新 vs 参照)
    Rms r_old[9], r_new[9];  // [0..2]=roll/pitch/yaw, [3..5]=pos, [6..8]=vel

    printf("t,ref_roll,ref_pitch,ref_yaw,old_roll,old_pitch,old_yaw,"
           "new_roll,new_pitch,new_yaw,"
           "ref_px,ref_py,ref_pz,old_px,old_py,old_pz,new_px,new_py,new_pz,"
           "ref_vx,ref_vy,ref_vz,old_vx,old_vy,old_vz,new_vx,new_vy,new_vz\n");

    double c[NCOL];
    while (getline(&line, &cap, fp) >= 0) {
        char buf[4096];
        strncpy(buf, line, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        if (!parseRow(buf, c)) continue;

        Vec3 accel(static_cast<float>(c[C_AX]), static_cast<float>(c[C_AY]),
                   static_cast<float>(c[C_AZ]));
        Vec3 gyro(static_cast<float>(c[C_GX]), static_cast<float>(c[C_GY]),
                  static_cast<float>(c[C_GZ]));

        // --- Warm-start both filters from the first logged sample so they begin
        //     at the firmware's already-converged state (no cold-start drift). ---
        // --- 最初のログサンプルで両フィルタをウォームスタートし、ファームの収束済み
        //     状態から開始(コールドスタートのドリフトを排除)。 ---
        if (!warm) {
            Vec3 bg0(static_cast<float>(c[C_RBGX]), static_cast<float>(c[C_RBGY]),
                     static_cast<float>(c[C_RBGZ]));
            // Legacy: gyro bias + level-attitude reference (re-freeze accel bias,
            // as the firmware does on the ground).
            // 旧: ジャイロバイアス + レベル姿勢基準(ファーム同様に地上で加速度
            // バイアスを再凍結)。
            eskf.setGyroBias(toOld(bg0));
            eskf.setAttitudeReference(toOld(accel), toOld(bg0));
            eskf.setFreezeAccelBias(true);
            // Redesign: identical calibration path as integrated_sil startup.
            // 新: integrated_sil 起動と同一の較正手順。
            Vec3 ba0(accel.x, accel.y, accel.z - (-GRAVITY));
            neweskf.setGyroBias(bg0);
            neweskf.setAccelBias(ba0);
            neweskf.setAttitudeFromGravity(accel);
            neweskf.shrinkBiasCovariance(0.01f);
            neweskf.setFreezeAccelBias(true);
            t0_us = c[C_TS];
            last_tof_us = c[C_TS];
            warm = true;
        }

        // --- Detect takeoff: the firmware holds pos/vel = 0 on the ground, so
        //     the first row where the logged reference pos/vel becomes nonzero
        //     marks when the firmware released the hold. ---
        // --- 離陸検出: ファームは接地中 pos/vel を0保持するため、ログ参照 pos/vel が
        //     初めて非零になった行が、ファームが保持を解除した瞬間。 ---
        double ref_motion = std::fabs(c[C_RPX]) + std::fabs(c[C_RPY]) +
                            std::fabs(c[C_RPZ]) + std::fabs(c[C_RVX]) +
                            std::fabs(c[C_RVY]) + std::fabs(c[C_RVZ]);
        bool just_took_off = false;
        if (!taken_off && ref_motion > TAKEOFF_EPS) {
            taken_off = true;
            just_took_off = true;
        }

        // --- Predict + accel-attitude update every cycle (both filters).
        //     skip_position freezes position integration while grounded. ---
        // --- 毎周期 予測 + 加速度姿勢更新(両フィルタ)。接地中は skip_position で
        //     位置積分を凍結。 ---
        eskf.predict(toOld(accel), toOld(gyro), DT, /*skip_position=*/!taken_off);
        eskf.updateAccelAttitude(toOld(accel));
        neweskf.predict(accel, gyro, DT);
        neweskf.updateAccelAttitude(accel);

        if (!taken_off) {
            // --- Grounded: hold pos/vel at zero; no flow/ToF updates. ---
            // --- 接地中: pos/vel を0保持; フロー/ToF更新なし。 ---
            eskf.holdPositionVelocity();
            neweskf.holdPositionVelocity();
        } else {
            // --- Takeoff event (once): reset position covariance + unfreeze
            //     accel bias, and arm the post-takeoff skip counters. ---
            // --- 離陸イベント(1回): 位置共分散リセット + 加速度バイアス凍結解除、
            //     離陸直後スキップカウンタを装填。 ---
            if (just_took_off) {
                eskf.resetPositionVelocity();
                // Keep the LEGACY accel bias FROZEN: the flight log shows the
                // firmware's accel bias X/Y stayed exactly constant for 60 s
                // (active_mask=0x06FF excludes BA). Unfreezing it (the literal
                // imu_task code path) makes the SIL estimate BA, which drifts on
                // real vibration and corrupts attitude (latent bug C). The new
                // redesign is robust, so it is unfrozen to match integrated_sil.
                // 旧ESKFの加速度バイアスは終始凍結: ログでは実機の accel bias X/Y は
                // 60秒間厳密に一定(active_mask=0x06FF が BA を除外)。解凍(imu_task
                // のコード上の経路)すると SIL は BA を推定し、実振動で漂流して姿勢を
                // 汚染する(潜在バグC)。新設計は頑健なので integrated_sil に合わせ解凍。
                neweskf.resetPositionVelocity();
                neweskf.setFreezeAccelBias(false);
                flow_skip = FLOW_TAKEOFF_SKIP;
                tof_skip  = TOF_TAKEOFF_SKIP;
            }

            // --- ToF update (10-sample post-takeoff skip) ---
            // --- ToF更新(離陸後10サンプルスキップ) ---
            if (c[C_TOF_NEW] != 0.0) {
                float dt_tof = static_cast<float>((c[C_TS] - last_tof_us) / 1e6);
                if (dt_tof <= 0.0f) dt_tof = TOF_DT_NOMINAL;
                last_tof_us = c[C_TS];
                if (tof_skip > 0) {
                    tof_skip--;
                } else {
                    float tof = static_cast<float>(c[C_TOF]);
                    eskf.updateToF(tof);
                    neweskf.updateToF(tof);
                    neweskf.updateToFVelocity(tof, dt_tof);  // redesign only
                }
            }

            // --- Optical-flow update (20-sample skip + squal/distance gate,
            //     exactly as SensorFusion::updateOpticalFlow gates it) ---
            // --- フロー更新(20スキップ + squal/距離ゲート、SensorFusion::
            //     updateOpticalFlow と同一条件) ---
            if (c[C_FLOW_NEW] != 0.0) {
                if (flow_skip > 0) {
                    flow_skip--;
                } else {
                    float tof = static_cast<float>(c[C_TOF]);
                    int   q   = static_cast<int>(c[C_FQ]);
                    if (q >= FLOW_SQUAL_MIN &&
                        tof >= FLOW_DIST_MIN && tof <= FLOW_DIST_MAX) {
                        int16_t dx = static_cast<int16_t>(lround(c[C_FDX]));
                        int16_t dy = static_cast<int16_t>(lround(c[C_FDY]));
                        eskf.updateFlowRaw(dx, dy, tof, FLOW_DT, gyro.x, gyro.y);
                        neweskf.updateFlowRaw(dx, dy, tof, FLOW_DT, gyro.x, gyro.y);
                    }
                }
            }
        }

        // --- Read estimates and convert attitudes to a common Euler basis ---
        // --- 推定値を読み、姿勢を共通オイラー基底へ変換 ---
        stampfly::ESKF::State so = eskf.getState();
        Quat qn = neweskf.getAttitude();
        Vec3 pn = neweskf.getPosition();
        Vec3 vn = neweskf.getVelocity();

        float rr, rp, ry, or_, op, oy, nr, np, ny;
        quatToEuler(static_cast<float>(c[C_RQW]), static_cast<float>(c[C_RQX]),
                    static_cast<float>(c[C_RQY]), static_cast<float>(c[C_RQZ]),
                    rr, rp, ry);
        quatToEuler(so.orientation.w, so.orientation.x, so.orientation.y,
                    so.orientation.z, or_, op, oy);
        quatToEuler(qn.w, qn.x, qn.y, qn.z, nr, np, ny);

        double t = (c[C_TS] - t0_us) / 1e6;

        printf("%.4f,"
               "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
               "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
               "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               t,
               rr * RAD2DEG, rp * RAD2DEG, ry * RAD2DEG,
               or_ * RAD2DEG, op * RAD2DEG, oy * RAD2DEG,
               nr * RAD2DEG, np * RAD2DEG, ny * RAD2DEG,
               c[C_RPX], c[C_RPY], c[C_RPZ],
               so.position.x, so.position.y, so.position.z,
               pn.x, pn.y, pn.z,
               c[C_RVX], c[C_RVY], c[C_RVZ],
               so.velocity.x, so.velocity.y, so.velocity.z,
               vn.x, vn.y, vn.z);

        // --- Accumulate RMSE after the skip window (warm-start + takeoff) ---
        // --- スキップ窓(ウォームスタート+離陸)後にRMSEを蓄積 ---
        if (t >= skip_s) {
            r_old[0].add(wrapPi(or_ - rr) * RAD2DEG);
            r_old[1].add(wrapPi(op - rp) * RAD2DEG);
            r_old[2].add(wrapPi(oy - ry) * RAD2DEG);
            r_old[3].add(so.position.x - c[C_RPX]);
            r_old[4].add(so.position.y - c[C_RPY]);
            r_old[5].add(so.position.z - c[C_RPZ]);
            r_old[6].add(so.velocity.x - c[C_RVX]);
            r_old[7].add(so.velocity.y - c[C_RVY]);
            r_old[8].add(so.velocity.z - c[C_RVZ]);

            r_new[0].add(wrapPi(nr - rr) * RAD2DEG);
            r_new[1].add(wrapPi(np - rp) * RAD2DEG);
            r_new[2].add(wrapPi(ny - ry) * RAD2DEG);
            r_new[3].add(pn.x - c[C_RPX]);
            r_new[4].add(pn.y - c[C_RPY]);
            r_new[5].add(pn.z - c[C_RPZ]);
            r_new[6].add(vn.x - c[C_RVX]);
            r_new[7].add(vn.y - c[C_RVY]);
            r_new[8].add(vn.z - c[C_RVZ]);
        }
    }
    fclose(fp);
    free(line);

    // --- RMSE summary + differential-diagnosis verdict (stderr) ---
    // --- RMSE要約 + 差分診断の判定(stderr) ---
    fprintf(stderr, "\n=== M7 differential diagnosis: SIL replay vs flight log ===\n");
    fprintf(stderr, "(RMSE over t >= %.1fs; LEGACY is the diagnostic primary)\n", skip_s);
    fprintf(stderr, "axis        | legacy(old) RMSE | redesign(new) RMSE\n");
    const char* labels[9] = {"roll  [deg]", "pitch [deg]", "yaw   [deg]",
                             "pos_x [m]  ", "pos_y [m]  ", "pos_z [m]  ",
                             "vel_x [m/s]", "vel_y [m/s]", "vel_z [m/s]"};
    for (int i = 0; i < 9; i++) {
        fprintf(stderr, "  %-11s | %15.4f  | %15.4f\n",
                labels[i], r_old[i].value(), r_new[i].value());
    }
    // Attitude is the most direct estimator-software signal (driven every cycle
    // by the same accel/gyro). Flag reproduction on the attitude RMSE.
    // 姿勢は最も直接的な推定器ソフト信号(毎周期同じaccel/gyroで駆動)。姿勢RMSE
    // で再現を判定。
    double att_rms = (r_old[0].value() + r_old[1].value() + r_old[2].value()) / 3.0;
    fprintf(stderr, "\nlegacy mean attitude RMSE = %.4f deg -> %s\n",
            att_rms,
            att_rms < 1.0
              ? "REPRODUCED (estimator software-deterministic; residual = HW timing/jitter/concurrency)"
              : "NOT reproduced (behavior depends on factors outside the logged ESKF inputs)");
    return 0;
}
