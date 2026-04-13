/**
 * @file eskf_final_compare.cpp
 * @brief Final comparison: top candidates on both hover AND sine
 *        最終比較: ホバーとsine両方でトップ候補を評価
 *
 * Tests the best configs from hover (F_adp50) and sine (J_adp200_3%)
 * on both scenarios to find the optimal single configuration.
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

struct Cfg { const char* name; float R; float k_adp; float gate_pct; };

struct HoverRes { float att_rms, att_max, alt_rms; };
struct SineRes  { float gain, track_err; };

static HoverRes run_hover(Cfg cfg) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hd = sqrtf(qp.mass*qp.gravity/(4.0f*qp.k_thrust));
    for(int i=0;i<4;i++) st.motor_speed[i]=hd;

    EskfCore eskf; EskfConfig ec;
    ec.use_tof=true; ec.use_baro=false; ec.use_mag=false; ec.use_flow=false;
    ec.accel_noise=0.3f; ec.gyro_noise=0.03f; ec.tof_noise=0.01f;
    ec.accel_att_noise=cfg.R; ec.k_adaptive=cfg.k_adp;
    ec.accel_norm_gate=cfg.gate_pct/100.0f;
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

    float ae_a=0.20f,ar_a=0.67f;
    Vec3 ge={},ae={},gr={};bool li=false;
    float dt=0.0025f;int steps=(int)(14.0f/dt),tof_div=13;
    float ht=qp.mass*qp.gravity;

    struct G{float t,dur;Vec3 f,tq;};
    G gusts[]={{1.0f,0.3f,{0.005f,0,0},{0,3e-5f,0}},
              {3.0f,0.5f,{0,-0.012f,0},{6e-5f,0,0}},
              {5.0f,0.2f,{0.015f,0.01f,0},{-5e-5f,7e-5f,0}},
              {7.0f,1.5f,{0.003f,-0.003f,0},{2e-5f,-2e-5f,0}},
              {9.0f,0.15f,{0.02f,0,0.005f},{0,8e-5f,0}}};

    float sum_r2=0,sum_p2=0,sum_a2=0,max_att=0;int n=0;

    for(int s=0;s<steps;s++){
        float t=s*dt;
        SimSensors ss=quad.getSensors(dt);quad.updateBiases(dt);
        Vec3 araw(ss.accel.x,ss.accel.y,ss.accel.z),graw(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=graw;ae=araw;gr=graw;li=true;}
        ge=ge*(1-ae_a)+graw*ae_a; ae=ae*(1-ae_a)+araw*ae_a; gr=gr*(1-ar_a)+graw*ar_a;
        eskf.predict(ae,ge,dt); eskf.updateAccelAttitude(ae);
        if(s%tof_div==0){eskf.updateToF(ss.tof_bottom);eskf.updateToFVelocity(ss.tof_bottom,tof_div*dt);}

        auto ts=quad.getState();Vec3 te=ts.attitude.to_euler(),ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();

        float er=ee.x,epi=ee.y,rx=gr.x-gb.x,ry_=gr.y-gb.y,rz=gr.z-gb.z;
        float eh=-ep.z,ec_=-ev.z;
        float T=ht+alv.compute(alp.compute(0.5f-eh,dt)-ec_,dt);
        float tau[3]={rr.compute(ar.compute(0-er,dt)-rx,dt),
                      rp.compute(ap.compute(0-epi,dt)-ry_,dt),
                      ry.compute(0-rz,dt)};

        Vec3 gf={},gt={};
        for(int gi=0;gi<5;gi++)if(t>=gusts[gi].t&&t<gusts[gi].t+gusts[gi].dur){gf+=gusts[gi].f;gt+=gusts[gi].tq;}
        quad.setExternalForceBody(gf,gt);
        float md[4];mixer(T,tau,md,qp.k_thrust);quad.step(md,dt);

        if(t>2.0f){
            float dr=(te.x-ee.x)*57.3f,dp=(te.y-ee.y)*57.3f;
            float da=sqrtf(dr*dr+dp*dp);
            sum_r2+=dr*dr;sum_p2+=dp*dp;
            sum_a2+=(-ts.position.z-0.5f)*(-ts.position.z-0.5f)*1e6f;
            if(da>max_att)max_att=da;n++;
        }
    }
    return {sqrtf((sum_r2+sum_p2)/n), max_att, sqrtf(sum_a2/n)};
}

static SineRes run_sine(Cfg cfg, float freq) {
    QuadPhysics quad;QuadParams qp;ContactParams cp;SensorNoiseParams np;
    quad.init(qp,cp,np);
    auto& st=const_cast<QuadState&>(quad.getState());
    st.position.z=-0.5f;
    float hd=sqrtf(qp.mass*qp.gravity/(4.0f*qp.k_thrust));
    for(int i=0;i<4;i++)st.motor_speed[i]=hd;

    EskfCore eskf;EskfConfig ec;
    ec.use_tof=true;ec.use_baro=false;ec.use_mag=false;ec.use_flow=false;
    ec.accel_noise=0.3f;ec.gyro_noise=0.03f;ec.tof_noise=0.01f;
    ec.accel_att_noise=cfg.R;ec.k_adaptive=cfg.k_adp;
    ec.accel_norm_gate=cfg.gate_pct/100.0f;
    eskf.init(ec);
    eskf.setGyroBias({0,0,0});eskf.setAccelBias({0,0,0});
    eskf.shrinkBiasCovariance(0.01f);eskf.setFreezeAccelBias(true);

    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f;rr.ti=0.7f;rr.td=0.008f;rr.output_limit=5.2e-3f;rr.eta=0.125f;
    rp=rr;rp.kp=1.995e-3f;
    ry.kp=5.31e-3f;ry.ti=1.6f;ry.output_limit=2.2e-3f;
    ar.kp=14.0f;ar.ti=4.0f;ar.output_limit=3.0f;ap=ar;
    alp.kp=1.2f;alp.ti=3.0f;alp.output_limit=0.5f;
    alv.kp=0.20f;alv.ti=1.0f;alv.output_limit=0.25f;

    float ae_a=0.20f,ar_a=0.67f;
    Vec3 ge={},ae={},gr={};bool li=false;
    float dt=0.0025f,amp=15.0f*M_PI/180.0f;
    float sim_time=2.0f+3.0f/fmaxf(freq,0.1f);if(sim_time>15)sim_time=15;
    int steps=(int)(sim_time/dt),tof_div=13;
    float ht=qp.mass*qp.gravity;

    float max_t=0,min_t=0,sum_te2=0;int nt=0;

    for(int s=0;s<steps;s++){
        float t=s*dt;
        SimSensors ss=quad.getSensors(dt);quad.updateBiases(dt);
        Vec3 araw(ss.accel.x,ss.accel.y,ss.accel.z),graw(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=graw;ae=araw;gr=graw;li=true;}
        ge=ge*(1-ae_a)+graw*ae_a;ae=ae*(1-ae_a)+araw*ae_a;gr=gr*(1-ar_a)+graw*ar_a;
        eskf.predict(ae,ge,dt);eskf.updateAccelAttitude(ae);
        if(s%tof_div==0){eskf.updateToF(ss.tof_bottom);eskf.updateToFVelocity(ss.tof_bottom,tof_div*dt);}

        auto ts=quad.getState();Vec3 te=ts.attitude.to_euler(),ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();
        float cmd=(t>1.0f)?amp*sinf(2*M_PI*freq*(t-1.0f)):0;
        float er=ee.x,epi=ee.y,rx=gr.x-gb.x,ry_=gr.y-gb.y,rz=gr.z-gb.z;
        float T=ht+alv.compute(alp.compute(0.5f-(-ep.z),dt)-(-ev.z),dt);
        float tau[3]={rr.compute(ar.compute(cmd-er,dt)-rx,dt),
                      rp.compute(ap.compute(0-epi,dt)-ry_,dt),ry.compute(0-rz,dt)};
        float md[4];mixer(T,tau,md,qp.k_thrust);quad.step(md,dt);

        if(t>1.5f){
            float tr=te.x*57.3f,er2=ee.x*57.3f;
            if(tr>max_t)max_t=tr;if(tr<min_t)min_t=tr;
            float te2=tr-er2;sum_te2+=te2*te2;nt++;
        }
    }
    float ta=(max_t-min_t)/2.0f;
    return {ta/15.0f, sqrtf(sum_te2/nt)};
}

int main(){
    Cfg cfgs[]={
        {"A_base_R2_10%",  2.0f,   0.0f, 10.0f},
        {"B_adp50_10%",    2.0f,  50.0f, 10.0f},
        {"C_adp100_10%",   2.0f, 100.0f, 10.0f},
        {"D_adp200_10%",   2.0f, 200.0f, 10.0f},
        {"E_adp200_3%",    2.0f, 200.0f,  3.0f},
        {"F_adp100_3%",    2.0f, 100.0f,  3.0f},
        {"G_adp50_5%",     2.0f,  50.0f,  5.0f},
        {"H_R2_3%",        2.0f,   0.0f,  3.0f},
    };
    int nc=sizeof(cfgs)/sizeof(cfgs[0]);

    fprintf(stderr,"=== Final Comparison: Hover + Sine ===\n\n");
    fprintf(stderr,"%-18s | %8s %8s %7s | %8s %8s | %8s %8s\n",
            "Config","hvr_rms","hvr_max","alt_mm","s03_g","s03_te","s10_g","s10_te");
    fprintf(stderr,"------------------------------------------------------------------------------------\n");

    for(int i=0;i<nc;i++){
        srand(42); HoverRes h=run_hover(cfgs[i]);
        srand(42); SineRes s03=run_sine(cfgs[i],0.3f);
        srand(42); SineRes s10=run_sine(cfgs[i],1.0f);
        fprintf(stderr,"%-18s | %7.2f° %7.2f° %5.0fmm | %6.2fx %7.2f° | %6.2fx %7.2f°\n",
                cfgs[i].name,h.att_rms,h.att_max,h.alt_rms,
                s03.gain,s03.track_err,s10.gain,s10.track_err);
    }
    fprintf(stderr,"\nhvr = hover+gusts 14s, s03/s10 = sine 0.3/1.0Hz ±15°\n");
    fprintf(stderr,"g = true_amp/cmd_amp (ideal=1.0x), te = RMS(true-eskf) [deg]\n");
    return 0;
}
