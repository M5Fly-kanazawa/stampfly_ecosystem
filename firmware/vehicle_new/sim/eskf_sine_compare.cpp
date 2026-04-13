/**
 * @file eskf_sine_compare.cpp
 * @brief Compare ESKF strategies on sine response (aggressive maneuver)
 *        sine応答（攻めた操作）でESKF戦略を比較
 *
 * Tests the ±15° roll sine at 0.3Hz and 1.0Hz with different configs.
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

// ESP_LOG stubs
#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"

using namespace sf; using namespace sf::math; using namespace sf::sim;

static void mixer(float T, const float tau[3], float d[4], float kt) {
    float L=0.023f,k=0.00971f;float t=T*.25f,r=tau[0]/(4*L),p=tau[1]/(4*L),y=tau[2]/(4*k);
    float t1=t-r+p+y,t2=t-r-p-y,t3=t+r-p+y,t4=t+r+p-y;
    for(int i=0;i<4;i++){float ti=(i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if(ti<0)ti=0;d[i]=sqrtf(ti/kt);if(d[i]>.95f)d[i]=.95f;}
}

struct Cfg {
    const char* name;
    float R; float k_adp; float gate_pct; bool disable;
};

struct Res {
    float gain;       // true_amplitude / cmd_amplitude
    float eskf_gain;  // eskf_amplitude / cmd_amplitude
    float track_err;  // RMS(true - eskf) [deg]
    float phase;      // phase of true vs cmd [deg]
};

static Res run_sine(Cfg cfg, float freq) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hover_duty = sqrtf(qp.mass * qp.gravity / (4.0f * qp.k_thrust));
    for (int i = 0; i < 4; i++) st.motor_speed[i] = hover_duty;

    EskfCore eskf;
    EskfConfig ecfg;
    ecfg.use_tof=true; ecfg.use_baro=false; ecfg.use_mag=false; ecfg.use_flow=false;
    ecfg.accel_noise=0.3f; ecfg.gyro_noise=0.03f; ecfg.tof_noise=0.01f;
    ecfg.accel_att_noise = cfg.R;
    ecfg.k_adaptive = cfg.k_adp;
    ecfg.accel_norm_gate = cfg.gate_pct / 100.0f;
    eskf.init(ecfg);
    eskf.setGyroBias({0,0,0}); eskf.setAccelBias({0,0,0});
    eskf.shrinkBiasCovariance(0.01f);
    eskf.setFreezeAccelBias(true);

    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f; rr.ti=0.7f; rr.td=0.008f; rr.output_limit=5.2e-3f; rr.eta=0.125f;
    rp=rr; rp.kp=1.995e-3f;
    ry.kp=5.31e-3f; ry.ti=1.6f; ry.output_limit=2.2e-3f;
    ar.kp=14.0f; ar.ti=4.0f; ar.output_limit=3.0f; ap=ar;
    alp.kp=1.2f; alp.ti=3.0f; alp.output_limit=0.5f;
    alv.kp=0.20f; alv.ti=1.0f; alv.output_limit=0.25f;

    float alpha_eskf=0.20f, alpha_rate=0.67f;
    Vec3 ge={},ae={},gr={}; bool li=false;
    float dt=0.0025f;
    float sim_time = 2.0f + 3.0f / fmaxf(freq, 0.1f);
    if (sim_time > 15.0f) sim_time = 15.0f;
    int steps=(int)(sim_time/dt), tof_div=13;
    float ht=qp.mass*qp.gravity, amp=15.0f*M_PI/180.0f;

    // Collect sine phase data
    float max_true=0, min_true=0, max_eskf=0, min_eskf=0;
    float sum_track_err2=0; int n_track=0;

    for (int s=0; s<steps; s++) {
        float t = s * dt;
        SimSensors ss=quad.getSensors(dt); quad.updateBiases(dt);
        Vec3 araw(ss.accel.x,ss.accel.y,ss.accel.z),graw(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=graw;ae=araw;gr=graw;li=true;}
        ge=ge*(1-alpha_eskf)+graw*alpha_eskf;
        ae=ae*(1-alpha_eskf)+araw*alpha_eskf;
        gr=gr*(1-alpha_rate)+graw*alpha_rate;

        eskf.predict(ae,ge,dt);
        if (!cfg.disable) eskf.updateAccelAttitude(ae);
        if(s%tof_div==0){eskf.updateToF(ss.tof_bottom);
            eskf.updateToFVelocity(ss.tof_bottom,tof_div*dt);}

        auto ts=quad.getState();
        Vec3 te=ts.attitude.to_euler();
        Vec3 ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();

        float cmd_roll = (t > 1.0f) ? amp * sinf(2*M_PI*freq*(t-1.0f)) : 0;

        float er=ee.x, epi=ee.y;
        float rx=gr.x-gb.x, ry_=gr.y-gb.y, rz=gr.z-gb.z;
        float eh=-ep.z, ec=-ev.z;
        float T=0; float tau[3]={};
        float vs=alp.compute(0.5f-eh,dt), tc=alv.compute(vs-ec,dt);
        T=ht+tc;
        float rs=ar.compute(cmd_roll-er,dt), ps=ap.compute(0-epi,dt);
        tau[0]=rr.compute(rs-rx,dt); tau[1]=rp.compute(ps-ry_,dt);
        tau[2]=ry.compute(0-rz,dt);
        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        // Statistics during sine phase (after 1.5s settle)
        if (t > 1.5f) {
            float true_r = te.x * 57.3f;
            float eskf_r = ee.x * 57.3f;
            if (true_r > max_true) max_true = true_r;
            if (true_r < min_true) min_true = true_r;
            if (eskf_r > max_eskf) max_eskf = eskf_r;
            if (eskf_r < min_eskf) min_eskf = eskf_r;
            float track_err = true_r - eskf_r;
            sum_track_err2 += track_err * track_err;
            n_track++;
        }
    }

    Res r;
    float true_amp = (max_true - min_true) / 2.0f;
    float eskf_amp = (max_eskf - min_eskf) / 2.0f;
    float cmd_amp = 15.0f;
    r.gain = true_amp / cmd_amp;
    r.eskf_gain = eskf_amp / cmd_amp;
    r.track_err = sqrtf(sum_track_err2 / n_track);
    r.phase = 0;
    return r;
}

int main() {
    Cfg configs[] = {
        {"A_base_R2_10%",    2.0f,  0.0f, 10.0f, false},
        {"B_gyro_only",      2.0f,  0.0f, 10.0f, true},
        {"C_adp50_10%",      2.0f, 50.0f, 10.0f, false},
        {"D_adp100_10%",     2.0f,100.0f, 10.0f, false},
        {"E_adp200_10%",     2.0f,200.0f, 10.0f, false},
        {"F_R2_3%",          2.0f,  0.0f,  3.0f, false},
        {"G_R2_1%",          2.0f,  0.0f,  1.0f, false},
        {"H_adp50_3%",       2.0f, 50.0f,  3.0f, false},
        {"I_adp100_3%",      2.0f,100.0f,  3.0f, false},
        {"J_adp200_3%",      2.0f,200.0f,  3.0f, false},
    };
    int nc = sizeof(configs) / sizeof(configs[0]);
    float freqs[] = {0.3f, 1.0f};
    int nf = 2;

    fprintf(stderr, "=== ESKF Sine Response Strategy Comparison ===\n");
    fprintf(stderr, "Roll ±15°, Tuned PIDs, alpha_eskf=0.20\n\n");

    for (int fi = 0; fi < nf; fi++) {
        fprintf(stderr, "--- %.1f Hz ---\n", freqs[fi]);
        fprintf(stderr, "%-20s %8s %8s %10s\n",
                "Config", "gain", "eskf_g", "track_err");
        fprintf(stderr, "-----------------------------------------------------\n");

        for (int ci = 0; ci < nc; ci++) {
            srand(42);
            Res r = run_sine(configs[ci], freqs[fi]);
            fprintf(stderr, "%-20s %7.2fx %7.2fx %8.2f°\n",
                    configs[ci].name, r.gain, r.eskf_gain, r.track_err);
        }
        fprintf(stderr, "\n");
    }

    // Ideal: gain=1.0, eskf_gain=1.0, track_err=0
    fprintf(stderr, "Ideal: gain=1.0x, eskf_gain=1.0x, track_err=0°\n");
    fprintf(stderr, "track_err = RMS(true_roll - eskf_roll) [deg]\n");
    return 0;
}
