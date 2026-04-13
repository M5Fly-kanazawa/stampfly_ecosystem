/**
 * @file eskf_step_recovery_test.cpp
 * @brief Step command → return to 0 → hover recovery test
 *        ステップ指令 → 0復帰 → ホバー回復テスト
 *
 * Scenario: hover → step to target roll → hold → snap back to 0 → hover
 * This tests ESKF recovery after a sustained tilt maneuver.
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

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

static void run_step(float step_deg, float hold_sec, FILE* csv) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hd = sqrtf(qp.mass*qp.gravity/(4.0f*qp.k_thrust));
    for(int i=0;i<4;i++) st.motor_speed[i] = hd;

    EskfCore eskf; EskfConfig ec;
    ec.use_tof=true; ec.use_baro=false; ec.use_mag=false; ec.use_flow=false;
    ec.accel_noise=0.3f; ec.gyro_noise=0.03f; ec.tof_noise=0.01f;
    // k_adaptive=50 (current default)
    eskf.init(ec);
    eskf.setGyroBias({0,0,0}); eskf.setAccelBias({0,0,0});
    eskf.shrinkBiasCovariance(0.01f); eskf.setFreezeAccelBias(true);

    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f;rr.ti=0.7f;rr.td=0.008f;rr.output_limit=5.2e-3f;rr.eta=0.125f;
    rp=rr;rp.kp=1.995e-3f;
    ry.kp=5.31e-3f;ry.ti=1.6f;ry.output_limit=2.2e-3f;
    ar.kp=14.0f;ar.ti=4.0f;ar.output_limit=3.0f;ap=ar;
    alp.kp=1.2f;alp.ti=3.0f;alp.output_limit=0.5f;
    alv.kp=0.20f;alv.ti=1.0f;alv.output_limit=0.25f;

    float alpha_eskf=0.20f, alpha_rate=0.67f;
    Vec3 ge={},ae={},gr={}; bool li=false;
    float dt=0.0025f, tof_div=13;
    float ht=qp.mass*qp.gravity;
    float step_rad = step_deg * M_PI / 180.0f;

    // Timeline:
    //   0-2s:   hover (settle)
    //   2s:     step to target roll
    //   2+hold: snap back to 0
    //   rest:   hover recovery (5s)
    float t_step = 2.0f;
    float t_back = t_step + hold_sec;
    float t_end  = t_back + 5.0f;
    int steps = (int)(t_end / dt);

    for(int s=0; s<steps; s++){
        float t = s*dt;
        SimSensors ss=quad.getSensors(dt); quad.updateBiases(dt);
        Vec3 araw(ss.accel.x,ss.accel.y,ss.accel.z),graw(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=graw;ae=araw;gr=graw;li=true;}
        ge=ge*(1-alpha_eskf)+graw*alpha_eskf;
        ae=ae*(1-alpha_eskf)+araw*alpha_eskf;
        gr=gr*(1-alpha_rate)+graw*alpha_rate;

        eskf.predict(ae,ge,dt);
        eskf.updateAccelAttitude(ae);
        if(s%(int)tof_div==0){eskf.updateToF(ss.tof_bottom);
            eskf.updateToFVelocity(ss.tof_bottom,(int)tof_div*dt);}

        auto ts=quad.getState();
        Vec3 te=ts.attitude.to_euler(), ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(), ev=eskf.getVelocity(), gb=eskf.getGyroBias();

        // Command: step → hold → snap back
        float cmd = 0;
        if (t >= t_step && t < t_back) cmd = step_rad;

        float er=ee.x, epi=ee.y;
        float rx=gr.x-gb.x, ry_=gr.y-gb.y, rz=gr.z-gb.z;
        float eh=-ep.z, ec_=-ev.z;
        float T=ht+alv.compute(alp.compute(0.5f-eh,dt)-ec_,dt);
        float tau[3]={rr.compute(ar.compute(cmd-er,dt)-rx,dt),
                      rp.compute(ap.compute(0-epi,dt)-ry_,dt),
                      ry.compute(0-rz,dt)};
        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        // CSV at 200Hz
        if(s%2==0){
            fprintf(csv,"%.4f,%.3f,%.3f,%.3f,%.3f,%.4f\n",
                    t, cmd*57.3f, te.x*57.3f, ee.x*57.3f,
                    (te.x-ee.x)*57.3f, -ts.position.z);
        }
    }
}

int main(){
    struct Test { float deg; float hold; const char* name; };
    Test tests[] = {
        { 5.0f, 1.0f, "5deg_1s"},
        {10.0f, 1.0f, "10deg_1s"},
        {15.0f, 1.0f, "15deg_1s"},
        {15.0f, 2.0f, "15deg_2s"},
        {15.0f, 0.3f, "15deg_03s"},
    };
    int nt = sizeof(tests)/sizeof(tests[0]);

    fprintf(stderr,"=== Step → Recovery Test (k_adaptive=50) ===\n\n");

    for(int i=0;i<nt;i++){
        srand(42);
        char fn[64];
        snprintf(fn,sizeof(fn),"step_%s.csv",tests[i].name);
        FILE* csv=fopen(fn,"w");
        fprintf(csv,"time,cmd_roll,true_roll,eskf_roll,est_error,altitude\n");
        run_step(tests[i].deg, tests[i].hold, csv);
        fclose(csv);
        fprintf(stderr,"  %s → %s\n", tests[i].name, fn);
    }

    // Summary
    fprintf(stderr,"\nAnalyzing recovery...\n");
    for(int i=0;i<nt;i++){
        char fn[64];
        snprintf(fn,sizeof(fn),"step_%s.csv",tests[i].name);
        FILE* f=fopen(fn,"r");
        char hdr[256]; fgets(hdr,sizeof(hdr),f);
        float t,cmd,tr,er,est_e,alt;
        float t_back = 2.0f + tests[i].hold;
        // Find max error during hold and during recovery
        float max_hold_err=0, max_recov_err=0;
        float recov_settle_t=0; bool settled=false;
        while(fscanf(f,"%f,%f,%f,%f,%f,%f",&t,&cmd,&tr,&er,&est_e,&alt)==6){
            if(t>=2.0f && t<t_back){
                if(fabsf(est_e)>max_hold_err) max_hold_err=fabsf(est_e);
            }
            if(t>=t_back){
                if(fabsf(est_e)>max_recov_err) max_recov_err=fabsf(est_e);
                if(!settled && fabsf(est_e)<1.0f && t>t_back+0.5f){
                    recov_settle_t=t-t_back; settled=true;
                }
            }
        }
        fclose(f);
        fprintf(stderr,"  %-12s hold_err_max=%5.1f°  recov_err_max=%5.1f°  settle(<1°)=%.1fs\n",
                tests[i].name, max_hold_err, max_recov_err,
                settled?recov_settle_t:99.9f);
    }

    return 0;
}
