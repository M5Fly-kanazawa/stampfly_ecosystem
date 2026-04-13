/**
 * @file eskf_accel_compare.cpp
 * @brief Compare ESKF attitude estimation strategies
 *        ESKF姿勢推定戦略の比較テスト
 *
 * Tests the flight scenario (hover + gusts) with different accel
 * attitude observation configurations to find the optimal strategy.
 *
 * 異なる加速度姿勢観測設定でフライトシナリオ（ホバー+突風）をテストし、
 * 最適な戦略を見つける。
 *
 * Configurations tested:
 *   A: Baseline (current: R=2.0, norm_gate=10%)
 *   B: Accel attitude disabled (gyro-only)
 *   C: High R (R=10.0, norm_gate=10%)
 *   D: Adaptive R (R=2.0, k_adaptive=20)
 *   E: Tight norm gate (R=2.0, norm_gate=3%)
 *   F: Thrust-compensated observation model
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

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

struct TestConfig {
    const char* name;
    float accel_att_noise;     // R base
    float k_adaptive;          // adaptive R scaling
    float norm_gate_pct;       // norm gate threshold [%]
    bool disable_accel_att;    // completely disable accel attitude update
    bool thrust_compensate;    // subtract thrust from accel before observation
};

struct TestResult {
    float att_rms;       // attitude RMS [deg]
    float att_max;       // attitude max [deg]
    float alt_rms;       // altitude RMS [mm]
    float att_bias;      // mean attitude error [deg]
    int norm_gate_pass;
    int norm_gate_total;
};

static TestResult run_test(TestConfig cfg, bool write_csv = false)
{
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);

    // Start airborne at 50cm hover
    // 50cmホバーで空中開始
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hover_duty = sqrtf(qp.mass * qp.gravity / (4.0f * qp.k_thrust));
    for (int i = 0; i < 4; i++) st.motor_speed[i] = hover_duty;

    EskfCore eskf;
    EskfConfig ecfg;
    ecfg.use_tof=true; ecfg.use_baro=false; ecfg.use_mag=false; ecfg.use_flow=false;
    ecfg.accel_noise=0.3f; ecfg.gyro_noise=0.03f; ecfg.tof_noise=0.01f;
    ecfg.accel_att_noise = cfg.accel_att_noise;
    ecfg.k_adaptive = cfg.k_adaptive;
    ecfg.accel_norm_gate = cfg.norm_gate_pct / 100.0f;
    eskf.init(ecfg);
    eskf.setGyroBias({0,0,0});
    eskf.setAccelBias({0,0,0});
    eskf.shrinkBiasCovariance(0.01f);
    eskf.setFreezeAccelBias(true);

    // Tuned PIDs (from sweep results)
    // チューニング済みPID（スイープ結果から）
    PID rr, rp, ry, ar, ap, alp, alv;
    rr.kp=1.365e-3f; rr.ti=0.7f; rr.td=0.008f; rr.output_limit=5.2e-3f; rr.eta=0.125f;
    rp=rr; rp.kp=1.995e-3f;
    ry.kp=5.31e-3f; ry.ti=1.6f; ry.output_limit=2.2e-3f;
    ar.kp=14.0f; ar.ti=4.0f; ar.output_limit=3.0f; ap=ar;
    alp.kp=1.2f; alp.ti=3.0f; alp.output_limit=0.5f;
    alv.kp=0.20f; alv.ti=1.0f; alv.output_limit=0.25f;

    float alpha_eskf=0.20f, alpha_rate=0.67f;
    Vec3 ge={}, ae={}, gr={}; bool li=false;

    float dt=0.0025f, sim_time=14.0f;
    int steps=(int)(sim_time/dt), tof_div=13;
    float ht=qp.mass*qp.gravity;
    float norm_gate_thresh = cfg.norm_gate_pct / 100.0f;

    // Gust schedule (same as flight_scenario_test)
    // 突風スケジュール（flight_scenario_testと同じ）
    struct Gust { float t, dur; Vec3 force, torque; };
    Gust gusts[] = {
        {1.0f, 0.3f, {0.005f,0,0}, {0,3e-5f,0}},
        {3.0f, 0.5f, {0,-0.012f,0}, {6e-5f,0,0}},
        {5.0f, 0.2f, {0.015f,0.01f,0}, {-5e-5f,7e-5f,0}},
        {7.0f, 1.5f, {0.003f,-0.003f,0}, {2e-5f,-2e-5f,0}},
        {9.0f, 0.15f, {0.02f,0,0.005f}, {0,8e-5f,0}},
    };
    int n_gusts=5;

    FILE* csv = nullptr;
    if (write_csv) {
        char fn[128];
        snprintf(fn, sizeof(fn), "compare_%s.csv", cfg.name);
        csv = fopen(fn, "w");
        fprintf(csv, "time,true_roll,true_pitch,eskf_roll,eskf_pitch,"
                     "att_err_r,att_err_p,altitude,norm_ratio,gate\n");
    }

    TestResult result = {};
    float sum_r2=0, sum_p2=0, sum_alt2=0, sum_r=0, sum_p=0;
    int n_stat=0;

    for (int s=0; s<steps; s++) {
        float t = s * dt;

        SimSensors ss = quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 araw(ss.accel.x, ss.accel.y, ss.accel.z);
        Vec3 graw(ss.gyro.x, ss.gyro.y, ss.gyro.z);
        if(!li){ge=graw;ae=araw;gr=graw;li=true;}
        ge = ge*(1-alpha_eskf) + graw*alpha_eskf;
        ae = ae*(1-alpha_eskf) + araw*alpha_eskf;
        gr = gr*(1-alpha_rate) + graw*alpha_rate;

        // Norm gate metric (external calculation)
        // ノルムゲートメトリック（外部計算）
        Vec3 abc = ae - eskf.getAccelBias();
        float anorm = abc.norm();
        float nratio = fabsf(anorm - ecfg.gravity) / ecfg.gravity;
        bool gate_pass = (nratio <= norm_gate_thresh);

        // ESKF predict
        eskf.predict(ae, ge, dt);

        // Accel attitude update (strategy-dependent)
        // 加速度姿勢更新（戦略依存）
        if (!cfg.disable_accel_att) {
            if (cfg.thrust_compensate) {
                // Thrust-compensated: use specific force subtracted accel
                // 推力補償: 比力を差し引いた加速度を使用
                //
                // We know the thrust command T from the controller.
                // Accel measures [0, 0, -T/m] in body frame.
                // Compensated: accel + [0, 0, T/m] should ≈ [0, 0, 0]
                // But this removes ALL attitude info!
                //
                // Better approach: use norm gate but with stricter threshold
                // AND only when angular rate is low (near-hover)
                //
                // For now, we implement: only update when BOTH norm gate
                // passes AND estimated rate is below threshold
                //
                // 推力コマンドTがわかっている。
                // 加速度計はボディフレームで[0, 0, -T/m]を計測。
                // 補償: accel + [0, 0, T/m] ≈ [0, 0, 0]
                // しかしこれは姿勢情報を全て失う！
                //
                // より良いアプローチ: ノルムゲート + レート制限
                float rate_mag = sqrtf(gr.x*gr.x + gr.y*gr.y);
                float rate_thresh = 0.1f;  // ~5.7 deg/s
                if (gate_pass && rate_mag < rate_thresh) {
                    eskf.updateAccelAttitude(ae);
                }
            } else {
                // Standard: let ESKF's internal norm gate handle it
                // 標準: ESKFの内部ノルムゲートに任せる
                eskf.updateAccelAttitude(ae);
            }
        }

        // ToF updates
        if (s % tof_div == 0) {
            eskf.updateToF(ss.tof_bottom);
            eskf.updateToFVelocity(ss.tof_bottom, tof_div*dt);
        }

        // True state
        auto ts = quad.getState();
        Vec3 te = ts.attitude.to_euler();
        Vec3 ee = eskf.getAttitude().to_euler();
        Vec3 ep = eskf.getPosition(), ev = eskf.getVelocity(), gb = eskf.getGyroBias();

        // Control
        float er=ee.x, epi=ee.y;
        float rx=gr.x-gb.x, ry_=gr.y-gb.y, rz=gr.z-gb.z;
        float eh=-ep.z, ec=-ev.z;

        float T=0; float tau[3]={};
        float vs=alp.compute(0.5f-eh,dt), tc=alv.compute(vs-ec,dt);
        T=ht+tc;
        float rs=ar.compute(0-er,dt), ps=ap.compute(0-epi,dt);
        tau[0]=rr.compute(rs-rx,dt); tau[1]=rp.compute(ps-ry_,dt);
        tau[2]=ry.compute(0-rz,dt);

        // Gusts
        Vec3 gf={}, gt={};
        for (int gi=0; gi<n_gusts; gi++) {
            if (t >= gusts[gi].t && t < gusts[gi].t + gusts[gi].dur) {
                gf += gusts[gi].force; gt += gusts[gi].torque;
            }
        }
        quad.setExternalForceBody(gf, gt);

        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        // Statistics (after 2s settle)
        if (t > 2.0f) {
            float err_r = (te.x - ee.x) * 57.3f;
            float err_p = (te.y - ee.y) * 57.3f;
            float att_err = sqrtf(err_r*err_r + err_p*err_p);
            float alt_err = (-ts.position.z - 0.5f) * 1000.0f;  // mm

            sum_r2 += err_r*err_r; sum_p2 += err_p*err_p;
            sum_alt2 += alt_err*alt_err;
            sum_r += err_r; sum_p += err_p;
            n_stat++;

            if (att_err > result.att_max) result.att_max = att_err;
            if (gate_pass) result.norm_gate_pass++;
            result.norm_gate_total++;
        }

        // CSV output at 200Hz
        if (csv && s % 2 == 0) {
            fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%d\n",
                    t, te.x*57.3f, te.y*57.3f, ee.x*57.3f, ee.y*57.3f,
                    (te.x-ee.x)*57.3f, (te.y-ee.y)*57.3f,
                    -ts.position.z, nratio, gate_pass?1:0);
        }
    }

    if (csv) fclose(csv);

    result.att_rms = sqrtf((sum_r2 + sum_p2) / n_stat);
    result.alt_rms = sqrtf(sum_alt2 / n_stat);
    result.att_bias = sqrtf(sum_r*sum_r + sum_p*sum_p) / n_stat;

    return result;
}

int main()
{
    TestConfig configs[] = {
        {"A_baseline",    2.0f, 0.0f,  10.0f, false, false},
        {"B_gyro_only",   2.0f, 0.0f,  10.0f, true,  false},
        {"C_high_R10",   10.0f, 0.0f,  10.0f, false, false},
        {"D_high_R20",   20.0f, 0.0f,  10.0f, false, false},
        {"E_adaptive20",  2.0f, 20.0f, 10.0f, false, false},
        {"F_adaptive50",  2.0f, 50.0f, 10.0f, false, false},
        {"G_tight3pct",   2.0f, 0.0f,   3.0f, false, false},
        {"H_tight1pct",   2.0f, 0.0f,   1.0f, false, false},
        {"I_rate_gate",   2.0f, 0.0f,  10.0f, false, true},
        {"J_R5_adp10",    5.0f, 10.0f, 10.0f, false, false},
        {"K_R5_tight5",   5.0f, 0.0f,   5.0f, false, false},
    };
    int n_configs = sizeof(configs) / sizeof(configs[0]);

    fprintf(stderr, "=== ESKF Accel Attitude Strategy Comparison ===\n");
    fprintf(stderr, "Scenario: 14s hover at 50cm with 5 gusts\n");
    fprintf(stderr, "Tuned PIDs: att_kp=14, rate_td=0.008, alpha_eskf=0.20\n\n");

    fprintf(stderr, "%-20s %8s %8s %8s %8s %10s\n",
            "Config", "att_rms", "att_max", "alt_rms", "bias", "gate_pass");
    fprintf(stderr, "------------------------------------------------------------------------\n");

    for (int i = 0; i < n_configs; i++) {
        srand(42);
        bool write = (i == 0 || i == 1);  // Write CSV for baseline and gyro-only
        TestResult r = run_test(configs[i], write);
        fprintf(stderr, "%-20s %7.2f° %7.2f° %6.0fmm %6.2f° %5.1f%%\n",
                configs[i].name,
                r.att_rms, r.att_max, r.alt_rms, r.att_bias,
                100.0f * r.norm_gate_pass / fmaxf(1, r.norm_gate_total));
    }

    return 0;
}
