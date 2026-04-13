/**
 * @file eskf_diag_test.cpp
 * @brief ESKF diagnostic test — analyze norm gate, innovation, and attitude
 *        tracking during sine maneuver
 *        ESKF診断テスト — sine操作中のノルムゲート・イノベーション・姿勢
 *        追従を解析
 *
 * Outputs detailed CSV with ESKF internal metrics to diagnose why
 * the ESKF attitude estimate diverges from true attitude during
 * dynamic maneuvers (even at 0.3Hz).
 *
 * ESKFが動的操作中（0.3Hzでも）真の姿勢から乖離する原因を診断するため、
 * ESKF内部メトリクスを含む詳細CSVを出力。
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

// ESP_LOG stubs provided by ../test/esp_log.h via include path
// ESP_LOGスタブは../test/esp_log.hからインクルードパス経由で提供

#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"

using namespace sf; using namespace sf::math; using namespace sf::sim;

static void mixer(float T, const float tau[3], float d[4], float kt) {
    float L = 0.023f, k = 0.00971f;
    float t = T * 0.25f, r = tau[0]/(4*L), p = tau[1]/(4*L), y = tau[2]/(4*k);
    float t1=t-r+p+y, t2=t-r-p-y, t3=t+r-p+y, t4=t+r+p-y;
    for (int i = 0; i < 4; i++) {
        float ti = (i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if (ti < 0) ti = 0;
        d[i] = sqrtf(ti / kt);
        if (d[i] > 0.95f) d[i] = 0.95f;
    }
}

static void run_diag(float freq, FILE* csv) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);

    // Start airborne
    // 空中開始
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hover_duty = sqrtf(qp.mass * qp.gravity / (4.0f * qp.k_thrust));
    for (int i = 0; i < 4; i++) st.motor_speed[i] = hover_duty;

    EskfCore eskf;
    EskfConfig cfg;
    cfg.use_tof = true; cfg.use_baro = false; cfg.use_mag = false; cfg.use_flow = false;
    cfg.accel_noise = 0.3f; cfg.gyro_noise = 0.03f; cfg.tof_noise = 0.01f;
    cfg.accel_att_noise = 2.0f;
    eskf.init(cfg);
    eskf.setGyroBias({0,0,0});
    eskf.setAccelBias({0,0,0});
    eskf.shrinkBiasCovariance(0.01f);
    eskf.setFreezeAccelBias(true);

    PID rr, rp, ry, ar, ap, alp, alv;
    rr.kp=1.365e-3f; rr.ti=0.7f; rr.td=0.008f; rr.output_limit=5.2e-3f; rr.eta=0.125f;
    rp = rr; rp.kp = 1.995e-3f;
    ry.kp=5.31e-3f; ry.ti=1.6f; ry.output_limit=2.2e-3f;
    ar.kp=14.0f; ar.ti=4.0f; ar.output_limit=3.0f; ap = ar;
    alp.kp=1.2f; alp.ti=3.0f; alp.output_limit=0.5f;
    alv.kp=0.20f; alv.ti=1.0f; alv.output_limit=0.25f;

    float alpha_eskf = 0.20f, alpha_rate = 0.67f;
    Vec3 ge={}, ae={}, gr={}; bool li = false;

    float dt = 0.0025f;
    float sim_time = 2.0f + 3.0f / fmaxf(freq, 0.1f);
    if (sim_time > 15.0f) sim_time = 15.0f;
    int steps = (int)(sim_time / dt), tof_div = 13;
    float ht = qp.mass * qp.gravity;
    float amp = 15.0f * M_PI / 180.0f;

    int norm_gate_pass = 0, norm_gate_reject = 0;

    for (int s = 0; s < steps; s++) {
        float t = s * dt;

        SimSensors ss = quad.getSensors(dt);
        quad.updateBiases(dt);

        Vec3 araw(ss.accel.x, ss.accel.y, ss.accel.z);
        Vec3 graw(ss.gyro.x, ss.gyro.y, ss.gyro.z);
        if (!li) { ge = graw; ae = araw; gr = graw; li = true; }
        ge = ge * (1 - alpha_eskf) + graw * alpha_eskf;
        ae = ae * (1 - alpha_eskf) + araw * alpha_eskf;
        gr = gr * (1 - alpha_rate) + graw * alpha_rate;

        // Compute norm gate metric BEFORE ESKF update
        // ESKF更新前にノルムゲートメトリクスを計算
        Vec3 accel_bc = ae - eskf.getAccelBias();
        float accel_norm = accel_bc.norm();
        float norm_ratio = fabsf(accel_norm - cfg.gravity) / cfg.gravity;
        bool gate_pass = (norm_ratio <= 0.1f);

        // Compute innovation (what ESKF would see)
        // イノベーション計算（ESKFが見るもの）
        Vec3 g_ned_vec(0, 0, -cfg.gravity);
        Vec3 g_expected = eskf.getAttitude().inv_rotate(g_ned_vec);
        float innov_x = accel_bc.x - g_expected.x;
        float innov_y = accel_bc.y - g_expected.y;
        float innov_z = accel_bc.z - g_expected.z;
        float innov_norm = sqrtf(innov_x*innov_x + innov_y*innov_y + innov_z*innov_z);

        // Get P-matrix attitude diagonal before update
        // 更新前のP行列姿勢対角を取得
        float p_att_x = eskf.getPDiag(ATT_X);
        float p_att_y = eskf.getPDiag(ATT_Y);

        eskf.predict(ae, ge, dt);
        eskf.updateAccelAttitude(ae);
        if (s % tof_div == 0) {
            eskf.updateToF(ss.tof_bottom);
            eskf.updateToFVelocity(ss.tof_bottom, tof_div * dt);
        }

        if (gate_pass) norm_gate_pass++; else norm_gate_reject++;

        // True state
        // 真の状態
        auto ts = quad.getState();
        Vec3 te = ts.attitude.to_euler();
        Vec3 ee = eskf.getAttitude().to_euler();
        Vec3 ep = eskf.getPosition(), ev = eskf.getVelocity(), gb = eskf.getGyroBias();

        // Inertial acceleration (specific force contamination source)
        // 慣性加速度（比力汚染の発生源）
        Vec3 a_inertial = quad.getInertialAccelNed();

        // Compute true accel (no noise) for reference
        // 参照用に真の加速度（ノイズなし）を計算
        Vec3 g_ned_true(0, 0, qp.gravity);
        Vec3 true_accel_ned = a_inertial - g_ned_true;
        Vec3 true_accel_body = ts.attitude.inv_rotate(true_accel_ned);

        // Sine command
        // サインコマンド
        float cmd_roll = 0;
        if (t > 1.0f) {
            cmd_roll = amp * sinf(2.0f * M_PI * freq * (t - 1.0f));
        }

        // Control
        // 制御
        float er = ee.x, epi = ee.y;
        float rx = gr.x - gb.x, ry_ = gr.y - gb.y, rz = gr.z - gb.z;
        float eh = -ep.z, ec = -ev.z;

        float T = 0; float tau[3] = {};
        float vs = alp.compute(0.5f - eh, dt), tc = alv.compute(vs - ec, dt);
        T = ht + tc;
        float rs = ar.compute(cmd_roll - er, dt), ps = ap.compute(0 - epi, dt);
        tau[0] = rr.compute(rs - rx, dt); tau[1] = rp.compute(ps - ry_, dt);
        tau[2] = ry.compute(0 - rz, dt);

        float md[4]; mixer(T, tau, md, qp.k_thrust); quad.step(md, dt);

        // Output at 200Hz
        // 200Hzで出力
        if (s % 2 == 0) {
            fprintf(csv,
                "%.4f,"           // time
                "%.4f,%.4f,%.4f," // cmd_roll, true_roll, eskf_roll [deg]
                "%.4f,"           // attitude_error (true - eskf) [deg]
                "%.5f,"           // norm_ratio (gate metric)
                "%d,"             // gate_pass (1=pass, 0=reject)
                "%.4f,%.4f,%.4f," // innovation xyz [m/s²]
                "%.4f,"           // innovation norm [m/s²]
                "%.4f,%.4f,%.4f," // a_inertial NED [m/s²]
                "%.6f,%.6f,"      // P_att_x, P_att_y
                "%.4f,%.4f,%.4f," // true_accel_body xyz [m/s²]
                "%.4f,%.4f,"      // accel_filtered xy (ESKF input) [m/s²]
                "%.4f\n",         // altitude [m]
                t,
                cmd_roll * 57.3f, te.x * 57.3f, ee.x * 57.3f,
                (te.x - ee.x) * 57.3f,
                norm_ratio,
                gate_pass ? 1 : 0,
                innov_x, innov_y, innov_z,
                innov_norm,
                a_inertial.x, a_inertial.y, a_inertial.z,
                p_att_x, p_att_y,
                true_accel_body.x, true_accel_body.y, true_accel_body.z,
                ae.x, ae.y,
                -ts.position.z);
        }
    }

    float total = norm_gate_pass + norm_gate_reject;
    fprintf(stderr, "  %.1f Hz: gate pass=%.1f%% reject=%.1f%% (%d/%d)\n",
            freq,
            100.0f * norm_gate_pass / total,
            100.0f * norm_gate_reject / total,
            norm_gate_pass, (int)total);
}

int main() {
    float freqs[] = {0.3f, 1.0f, 3.0f};
    int nf = 3;

    fprintf(stderr, "=== ESKF Diagnostic Sine Test ===\n\n");

    for (int i = 0; i < nf; i++) {
        srand(42);
        char fn[64];
        snprintf(fn, sizeof(fn), "diag_sine_%.1fHz.csv", freqs[i]);
        FILE* csv = fopen(fn, "w");
        fprintf(csv, "time,cmd_roll,true_roll,eskf_roll,att_error,"
                     "norm_ratio,gate_pass,"
                     "innov_x,innov_y,innov_z,innov_norm,"
                     "a_inertial_x,a_inertial_y,a_inertial_z,"
                     "P_att_x,P_att_y,"
                     "true_accel_bx,true_accel_by,true_accel_bz,"
                     "accel_filt_x,accel_filt_y,"
                     "altitude\n");
        run_diag(freqs[i], csv);
        fclose(csv);
        fprintf(stderr, "    → %s\n\n", fn);
    }

    fprintf(stderr, "Done.\n");
    return 0;
}
