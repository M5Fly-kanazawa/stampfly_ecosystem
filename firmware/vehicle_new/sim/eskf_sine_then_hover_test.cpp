/**
 * @file eskf_sine_then_hover_test.cpp
 * @brief Sine command phase → snap to 0 → hover observation
 *        サイン指令フェーズ → 0に戻す → ホバー観察
 *
 * Scenario: hover(2s) → sine ±15° roll(duration) → cmd=0 → hover(5s)
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

static void run(float freq, float sine_dur, FILE* csv) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hd = sqrtf(qp.mass*qp.gravity/(4.0f*qp.k_thrust));
    for(int i=0;i<4;i++) st.motor_speed[i] = hd;

    EskfCore eskf; EskfConfig ec;
    ec.use_tof=true; ec.use_baro=false; ec.use_mag=false; ec.use_flow=false;
    ec.accel_noise=0.3f; ec.gyro_noise=0.03f; ec.tof_noise=0.01f;
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
    float dt=0.0025f; int tof_div=13;
    float ht=qp.mass*qp.gravity;
    float amp=15.0f*M_PI/180.0f;

    // Timeline:
    //   0 - 2s:            hover settle
    //   2 - 2+sine_dur:    sine ±15° at freq Hz
    //   2+sine_dur - end:  cmd=0, hover recovery (5s)
    float t_sine_start = 2.0f;
    float t_sine_end   = t_sine_start + sine_dur;
    float t_end        = t_sine_end + 5.0f;
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
        if(s%tof_div==0){eskf.updateToF(ss.tof_bottom);
            eskf.updateToFVelocity(ss.tof_bottom,tof_div*dt);}

        auto ts=quad.getState();
        Vec3 te=ts.attitude.to_euler(), ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(), ev=eskf.getVelocity(), gb=eskf.getGyroBias();

        // Command
        float cmd = 0;
        if (t >= t_sine_start && t < t_sine_end) {
            cmd = amp * sinf(2.0f * M_PI * freq * (t - t_sine_start));
        }

        float er=ee.x, epi=ee.y;
        float rx=gr.x-gb.x, ry_=gr.y-gb.y, rz=gr.z-gb.z;
        float eh=-ep.z, ec_=-ev.z;
        float T=ht+alv.compute(alp.compute(0.5f-eh,dt)-ec_,dt);
        float tau[3]={rr.compute(ar.compute(cmd-er,dt)-rx,dt),
                      rp.compute(ap.compute(0-epi,dt)-ry_,dt),
                      ry.compute(0-rz,dt)};
        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        if(s%2==0){
            fprintf(csv,"%.4f,%.3f,%.3f,%.3f,%.3f,%.4f\n",
                    t, cmd*57.3f, te.x*57.3f, ee.x*57.3f,
                    (te.x-ee.x)*57.3f, -ts.position.z);
        }
    }
}

int main(){
    struct Test { float freq; float dur; const char* name; };
    Test tests[] = {
        {0.3f, 5.0f, "03Hz_5s"},
        {0.3f, 3.0f, "03Hz_3s"},
        {1.0f, 3.0f, "1Hz_3s"},
        {1.0f, 5.0f, "1Hz_5s"},
        {3.0f, 3.0f, "3Hz_3s"},
    };
    int nt=sizeof(tests)/sizeof(tests[0]);

    fprintf(stderr,"=== Sine → Hover Recovery Test (k_adaptive=50) ===\n");
    fprintf(stderr,"Scenario: hover(2s) → sine ±15°(N s) → cmd=0 → hover(5s)\n\n");

    for(int i=0;i<nt;i++){
        srand(42);
        char fn[64]; snprintf(fn,sizeof(fn),"sine_hover_%s.csv",tests[i].name);
        FILE* csv=fopen(fn,"w");
        fprintf(csv,"time,cmd_roll,true_roll,eskf_roll,est_error,altitude\n");
        run(tests[i].freq, tests[i].dur, csv);
        fclose(csv);
        fprintf(stderr,"  %s → %s\n",tests[i].name,fn);
    }

    // Recovery analysis
    fprintf(stderr,"\n%-14s %8s %8s %8s %8s\n",
            "Scenario","sine_err","recov_pk","1deg_t","0.5deg_t");
    fprintf(stderr,"------------------------------------------------------\n");
    for(int i=0;i<nt;i++){
        char fn[64]; snprintf(fn,sizeof(fn),"sine_hover_%s.csv",tests[i].name);
        FILE* f=fopen(fn,"r"); char hdr[256]; fgets(hdr,sizeof(hdr),f);
        float t,cmd,tr,er,est_e,alt;
        float t_end_sine=2.0f+tests[i].dur;
        float sine_rms2=0; int ns=0;
        float recov_pk=0;
        float t_1deg=99, t_05deg=99;
        bool got1=false, got05=false;
        while(fscanf(f,"%f,%f,%f,%f,%f,%f",&t,&cmd,&tr,&er,&est_e,&alt)==6){
            if(t>=2.5f && t<t_end_sine){sine_rms2+=est_e*est_e;ns++;}
            if(t>=t_end_sine){
                if(fabsf(est_e)>recov_pk) recov_pk=fabsf(est_e);
                if(!got1 && t>t_end_sine+0.3f && fabsf(est_e)<1.0f){t_1deg=t-t_end_sine;got1=true;}
                if(!got05 && t>t_end_sine+0.3f && fabsf(est_e)<0.5f){t_05deg=t-t_end_sine;got05=true;}
            }
        }
        fclose(f);
        fprintf(stderr,"%-14s %7.1f° %7.1f° %7.1fs %7.1fs\n",
                tests[i].name, sqrtf(sine_rms2/fmaxf(1,ns)),
                recov_pk, t_1deg, t_05deg);
    }
    return 0;
}
