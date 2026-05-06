/**
 * @file tune_sweep.cpp
 * @brief Parameter sweep for ESKF + PID tuning (L4 only)
 *        ESKF + PIDチューニング用パラメータスイープ（L4のみ）
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

#include "sf_math.hpp"
#include "eskf_core.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"
#include "params.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    float L = 0.023f, kappa = 0.00971f;
    float t = thrust_cmd * 0.25f;
    float r = torque[0] / (4.0f * L);
    float p = torque[1] / (4.0f * L);
    float y = torque[2] / (4.0f * kappa);
    float t1=t-r+p+y, t2=t-r-p-y, t3=t+r-p+y, t4=t+r+p-y;
    for (int i = 0; i < 4; i++) {
        float ti = (i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if (ti<0) ti=0;
        duty[i] = sqrtf(ti/k_thrust);
        if (duty[i]>0.95f) duty[i]=0.95f;
    }
}

struct TuneParams {
    float accel_att_noise;
    float alt_p_kp, alt_p_ti;
    float alt_v_kp, alt_v_ti, alt_v_limit;
};

struct TuneResult {
    float att_rms, att_max, alt_rms, alt_max;
    float chi2_reject_pct;
    bool stable;
};

static TuneResult run_l4(TuneParams tp, FILE* csv)
{
    QuadPhysics quad;
    QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);

    EskfCore eskf;
    bool eskf_ready = false;

    PID rate_r, rate_p, rate_y, att_r, att_p, alt_p, alt_v;
    sf::params::get_float("rate.roll.kp",  rate_r.kp);
    sf::params::get_float("rate.roll.ti",  rate_r.ti);
    sf::params::get_float("rate.roll.td",  rate_r.td);
    rate_r.output_limit=5.2e-3f; rate_r.eta=0.125f;
    rate_p=rate_r;
    sf::params::get_float("rate.pitch.kp", rate_p.kp);
    sf::params::get_float("rate.yaw.kp",   rate_y.kp);
    sf::params::get_float("rate.yaw.ti",   rate_y.ti);
    rate_y.output_limit=2.2e-3f;
    sf::params::get_float("attitude.roll.kp", att_r.kp);
    sf::params::get_float("attitude.roll.ti", att_r.ti);
    att_r.output_limit=3.0f;
    att_p=att_r;

    alt_p.kp=tp.alt_p_kp; alt_p.ti=tp.alt_p_ti; alt_p.output_limit=0.5f;
    alt_v.kp=tp.alt_v_kp; alt_v.ti=tp.alt_v_ti; alt_v.output_limit=tp.alt_v_limit;

    const float alpha_eskf=0.32f, alpha_rate=0.67f;
    Vec3 ge={},ae={},gr={};
    bool lpf_init=false;

    const float dt=0.0025f, sim_time=12.0f;
    const int steps=(int)(sim_time/dt), tof_div=13;
    const float hover_thrust=qp.mass*qp.gravity;

    Vec3 cal_a={},cal_g={}; int cal_n=0;

    // Gusts
    struct G{float t,d;Vec3 f,q;};
    G gusts[]={
        {2.5f,0.3f,{0.005f,0,0},{0,3e-5f,0}},
        {4.0f,0.5f,{0,-0.012f,0},{6e-5f,0,0}},
        {6.5f,0.2f,{0.015f,0.01f,0},{-5e-5f,7e-5f,0}},
        {8.5f,1.5f,{0.003f,-0.003f,0},{2e-5f,-2e-5f,0}},
        {10.5f,0.15f,{0.02f,0,0.005f},{0,8e-5f,0}},
    };

    // Stats
    float sum_att2=0, sum_alt2=0, max_att=0, max_alt=0;
    int stat_n=0;

    if (csv) fprintf(csv, "time,true_roll,true_pitch,true_alt,est_roll,est_pitch,est_alt\n");

    for (int step=0; step<steps; step++) {
        float t=step*dt;
        float zero[4]={0,0,0,0};

        SimSensors sens=quad.getSensors(dt);
        quad.updateBiases(dt);
        Vec3 ar(sens.accel.x,sens.accel.y,sens.accel.z);
        Vec3 graw(sens.gyro.x,sens.gyro.y,sens.gyro.z);

        if (!lpf_init){ge=graw;ae=ar;gr=graw;lpf_init=true;}
        ge=ge*(1-alpha_eskf)+graw*alpha_eskf;
        ae=ae*(1-alpha_eskf)+ar*alpha_eskf;
        gr=gr*(1-alpha_rate)+graw*alpha_rate;

        if (t<1.0f){quad.step(zero,dt);continue;}
        if (t<2.0f){
            quad.step(zero,dt);
            if(cal_n<400){cal_a+=ar;cal_g+=graw;cal_n++;}
            if(cal_n==400&&!eskf_ready){
                Vec3 aa=cal_a*(1.0f/400), ga=cal_g*(1.0f/400);
                EskfConfig cfg;
                cfg.use_tof=true;cfg.use_baro=false;cfg.use_mag=false;cfg.use_flow=false;
                sf::params::get_float("eskf.process.accel_noise", cfg.accel_noise);
                sf::params::get_float("eskf.process.gyro_noise",  cfg.gyro_noise);
                cfg.tof_noise=0.01f;
                cfg.accel_att_noise=tp.accel_att_noise;
                eskf.init(cfg);
                eskf.setGyroBias(ga);
                eskf.setAccelBias(Vec3(aa.x,aa.y,aa.z+qp.gravity));
                eskf.setAttitudeFromGravity(aa);
                eskf.shrinkBiasCovariance(0.01f);
                eskf.setFreezeAccelBias(true);
                eskf_ready=true;
            }
            continue;
        }

        if(eskf_ready){
            eskf.predict(ae,ge,dt);
            eskf.updateAccelAttitude(ae);
            if(step%tof_div==0){
                eskf.updateToF(sens.tof_bottom);
                eskf.updateToFVelocity(sens.tof_bottom,tof_div*dt);
            }
        }

        auto ts=quad.getState();
        Vec3 te=ts.attitude.to_euler();
        float th=-ts.position.z;

        Vec3 ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition();
        Vec3 ev=eskf.getVelocity();
        Vec3 gb=eskf.getGyroBias();

        float er=ee.x,epi=ee.y;
        float erx=gr.x-gb.x, ery=gr.y-gb.y, erz=gr.z-gb.z;
        float eh=-ep.z, ec=-ev.z;

        if(th<0.03f){eskf.resetPositionVelocity();eh=th;ec=-ts.velocity.z;}

        float target=0.5f;
        float thrust=0; float torque[3]={0,0,0};
        float vsp=alt_p.compute(target-eh,dt);
        float tc=alt_v.compute(vsp-ec,dt);
        thrust=hover_thrust+tc;
        float rsp=att_r.compute(0-er,dt);
        float psp=att_p.compute(0-epi,dt);
        torque[0]=rate_r.compute(rsp-erx,dt);
        torque[1]=rate_p.compute(psp-ery,dt);
        torque[2]=rate_y.compute(0-erz,dt);

        Vec3 gf={},gt={};
        for(int gi=0;gi<5;gi++){
            if(t>=gusts[gi].t&&t<gusts[gi].t+gusts[gi].d){
                gf+=gusts[gi].f;gt+=gusts[gi].q;
            }
        }
        quad.setExternalForceBody(gf,gt);

        float md[4]; mixer(thrust,torque,md,qp.k_thrust);
        quad.step(md,dt);

        // Check stability
        if(fabsf(te.x)>1.0f||fabsf(te.y)>1.0f){
            TuneResult r={};r.stable=false;return r;
        }

        if(t>3.5f){
            float ae2=te.x*te.x+te.y*te.y;
            float alt_err=th-0.5f;
            sum_att2+=ae2*57.3f*57.3f;
            sum_alt2+=alt_err*alt_err;
            float att_deg=sqrtf(te.x*te.x+te.y*te.y)*57.3f;
            if(att_deg>max_att) max_att=att_deg;
            if(fabsf(alt_err)>max_alt) max_alt=fabsf(alt_err);
            stat_n++;
        }

        if(csv && step%8==0 && t>=2.0f){
            fprintf(csv,"%.3f,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f\n",
                t,te.x*57.3f,te.y*57.3f,th,er*57.3f,epi*57.3f,eh);
        }
    }

    TuneResult r;
    r.att_rms=sqrtf(sum_att2/stat_n);
    r.att_max=max_att;
    r.alt_rms=sqrtf(sum_alt2/stat_n)*1000;
    r.alt_max=max_alt*1000;
    r.stable=true;
    return r;
}

int main()
{
    sf::params::init();
    fprintf(stderr,"=== ESKF + PID Tuning Sweep ===\n\n");

    // Phase 1: ESKF accel_att_noise sweep
    fprintf(stderr,"--- Phase 1: accel_att_noise sweep (alt PID: current) ---\n");
    fprintf(stderr,"%8s %8s %8s %8s %8s %6s\n",
            "R_noise","att_rms","att_max","alt_rms","alt_max","status");
    fprintf(stderr,"------------------------------------------------------\n");

    float best_noise=2.0f; float best_att_rms=999;
    float noise_vals[]={0.3f, 0.5f, 0.7f, 1.0f, 1.5f, 2.0f};
    for(float nv : noise_vals){
        srand(42);
        TuneParams tp={nv, 0.6f,7.0f, 0.1f,2.5f,0.15f};
        TuneResult r=run_l4(tp,nullptr);
        if(r.stable){
            fprintf(stderr,"%8.2f %7.2f° %7.2f° %7.0fmm %7.0fmm %6s",
                    nv,r.att_rms,r.att_max,r.alt_rms,r.alt_max,"OK");
            if(r.att_rms<best_att_rms){best_att_rms=r.att_rms;best_noise=nv;
                fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else {
            fprintf(stderr,"%8.2f %7s %7s %7s %7s %6s\n",nv,"—","—","—","—","UNSTABLE");
        }
    }
    fprintf(stderr,"Best accel_att_noise = %.2f\n\n", best_noise);

    // Phase 2: Alt PID sweep with best ESKF
    fprintf(stderr,"--- Phase 2: Alt PID sweep (accel_att_noise=%.2f) ---\n", best_noise);
    fprintf(stderr,"%6s %6s %6s %6s %6s %8s %8s %8s %8s\n",
            "ap_kp","ap_ti","av_kp","av_ti","av_lim","att_rms","att_max","alt_rms","alt_max");
    fprintf(stderr,"--------------------------------------------------------------------------\n");

    struct AltSet{float pk,pt,vk,vt,vl;};
    AltSet alt_sets[]={
        {0.6f, 7.0f, 0.1f, 2.5f, 0.15f},  // Current
        {0.8f, 5.0f, 0.15f,2.0f, 0.15f},  // Faster position loop
        {1.0f, 4.0f, 0.15f,1.5f, 0.20f},  // Aggressive
        {1.0f, 3.0f, 0.20f,1.5f, 0.20f},  // More aggressive
        {1.2f, 3.0f, 0.20f,1.0f, 0.25f},  // Very aggressive
        {0.8f, 4.0f, 0.20f,2.0f, 0.20f},  // Balanced
        {1.0f, 5.0f, 0.20f,2.0f, 0.20f},  // Balanced+
    };
    float best_alt_rms=999; int best_alt_idx=0;
    for(int i=0;i<7;i++){
        srand(42);
        TuneParams tp={best_noise,
            alt_sets[i].pk,alt_sets[i].pt,
            alt_sets[i].vk,alt_sets[i].vt,alt_sets[i].vl};
        TuneResult r=run_l4(tp,nullptr);
        if(r.stable){
            fprintf(stderr,"%6.1f %6.1f %6.2f %6.1f %6.2f %7.2f° %7.2f° %7.0fmm %7.0fmm",
                    alt_sets[i].pk,alt_sets[i].pt,
                    alt_sets[i].vk,alt_sets[i].vt,alt_sets[i].vl,
                    r.att_rms,r.att_max,r.alt_rms,r.alt_max);
            if(r.alt_rms<best_alt_rms){best_alt_rms=r.alt_rms;best_alt_idx=i;
                fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else {
            fprintf(stderr,"%6.1f %6.1f %6.2f %6.1f %6.2f %7s %7s %7s %7s UNSTABLE\n",
                    alt_sets[i].pk,alt_sets[i].pt,
                    alt_sets[i].vk,alt_sets[i].vt,alt_sets[i].vl,"—","—","—","—");
        }
    }

    // Final: best combination with CSV output
    fprintf(stderr,"\n--- Final: Best combination ---\n");
    srand(42);
    TuneParams best={best_noise,
        alt_sets[best_alt_idx].pk,alt_sets[best_alt_idx].pt,
        alt_sets[best_alt_idx].vk,alt_sets[best_alt_idx].vt,alt_sets[best_alt_idx].vl};
    FILE* csv=fopen("tune_best_L4.csv","w");
    TuneResult r=run_l4(best,csv);
    fclose(csv);
    fprintf(stderr,"accel_att_noise=%.2f  alt_p(kp=%.1f,ti=%.1f) alt_v(kp=%.2f,ti=%.1f,lim=%.2f)\n",
            best.accel_att_noise,best.alt_p_kp,best.alt_p_ti,
            best.alt_v_kp,best.alt_v_ti,best.alt_v_limit);
    fprintf(stderr,"att_rms=%.2f° att_max=%.2f° alt_rms=%.0fmm alt_max=%.0fmm\n",
            r.att_rms,r.att_max,r.alt_rms,r.alt_max);

    return 0;
}
