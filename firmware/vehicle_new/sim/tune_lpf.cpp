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

using namespace sf; using namespace sf::math; using namespace sf::sim;

static void mixer(float T,const float tau[3],float d[4],float kt){
    float L=0.023f,k=0.00971f;float t=T*.25f,r=tau[0]/(4*L),p=tau[1]/(4*L),y=tau[2]/(4*k);
    float t1=t-r+p+y,t2=t-r-p-y,t3=t+r-p+y,t4=t+r+p-y;
    for(int i=0;i<4;i++){float ti=(i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if(ti<0)ti=0;d[i]=sqrtf(ti/kt);if(d[i]>.95f)d[i]=.95f;}}

// =============================================================================
// 2nd-order Butterworth LPF (bilinear transform)
// 2次バターワースLPF（双一次変換）
// =============================================================================
struct Biquad {
    float b0,b1,b2,a1,a2;
    float x1=0,x2=0,y1=0,y2=0;

    void init(float fc, float fs) {
        // Pre-warp / プリワープ
        float wd = tanf(M_PI * fc / fs);
        float wd2 = wd * wd;
        float sq2 = sqrtf(2.0f);
        float a0 = 1.0f + sq2*wd + wd2;
        b0 = wd2 / a0;
        b1 = 2.0f * wd2 / a0;
        b2 = wd2 / a0;
        a1 = 2.0f * (wd2 - 1.0f) / a0;
        a2 = (1.0f - sq2*wd + wd2) / a0;
        x1=x2=y1=y2=0;
    }

    float apply(float x) {
        float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2=x1; x1=x;
        y2=y1; y1=y;
        return y;
    }

    void seed(float v) { x1=x2=y1=y2=v; }
};

struct LPF6 {
    // 6-channel LPF (accel xyz + gyro xyz)
    // 6チャネルLPF
    enum Type { FIRST_ORDER, BUTTERWORTH2 };
    Type type;
    float alpha;  // For 1st order
    Biquad bq[6]; // For butterworth
    float state[6];
    bool initialized;

    void init(Type t, float fc, float fs) {
        type = t;
        initialized = false;
        if (type == FIRST_ORDER) {
            // alpha from cutoff: alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt)
            float dt = 1.0f / fs;
            alpha = 2.0f * M_PI * fc * dt / (1.0f + 2.0f * M_PI * fc * dt);
        } else {
            for (int i = 0; i < 6; i++) bq[i].init(fc, fs);
        }
        for (int i = 0; i < 6; i++) state[i] = 0;
    }

    void apply(const Vec3& accel, const Vec3& gyro,
               Vec3& accel_out, Vec3& gyro_out) {
        float in[6] = {accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z};
        if (!initialized) {
            for (int i = 0; i < 6; i++) {
                state[i] = in[i];
                if (type == BUTTERWORTH2) bq[i].seed(in[i]);
            }
            initialized = true;
        }
        float out[6];
        if (type == FIRST_ORDER) {
            for (int i = 0; i < 6; i++) {
                state[i] = state[i] * (1 - alpha) + in[i] * alpha;
                out[i] = state[i];
            }
        } else {
            for (int i = 0; i < 6; i++) out[i] = bq[i].apply(in[i]);
        }
        accel_out = {out[0], out[1], out[2]};
        gyro_out = {out[3], out[4], out[5]};
    }
};

struct Res { float ar,am,alr; bool ok; };

static Res run(LPF6::Type lpf_type, float fc) {
    QuadPhysics quad;QuadParams qp;ContactParams cp;SensorNoiseParams np;quad.init(qp,cp,np);
    EskfCore eskf;bool eskf_ready=false;
    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f;rr.ti=0.7f;rr.td=0.008f;rr.output_limit=5.2e-3f;rr.eta=0.125f;
    rp=rr;rp.kp=1.995e-3f;
    ry.kp=5.31e-3f;ry.ti=1.6f;ry.output_limit=2.2e-3f;
    ar.kp=14.0f;ar.ti=4.0f;ar.output_limit=3.0f;ap=ar;
    alp.kp=1.2f;alp.ti=3.0f;alp.output_limit=0.5f;
    alv.kp=0.20f;alv.ti=1.0f;alv.output_limit=0.25f;

    // ESKF LPF
    LPF6 eskf_lpf; eskf_lpf.init(lpf_type, fc, 400.0f);
    // Rate LPF (always 1st order 80Hz)
    float rate_alpha = 0.67f;
    Vec3 gr={}; bool gr_init=false;

    float dt=0.0025f;int steps=(int)(12/dt),tof_div=13;float ht=qp.mass*qp.gravity;
    Vec3 ca={},cg={};int cn=0;
    struct G{float t,d;Vec3 f,q;};
    G gs[]={{2.5f,.3f,{.005f,0,0},{0,3e-5f,0}},{4,.5f,{0,-.012f,0},{6e-5f,0,0}},
           {6.5f,.2f,{.015f,.01f,0},{-5e-5f,7e-5f,0}},{8.5f,1.5f,{.003f,-.003f,0},{2e-5f,-2e-5f,0}},
           {10.5f,.15f,{.02f,0,.005f},{0,8e-5f,0}}};
    float sa=0,sl=0,ma=0;int sn=0;

    for(int s=0;s<steps;s++){
        float t=s*dt;float z[4]={};
        SimSensors ss=quad.getSensors(dt);quad.updateBiases(dt);
        Vec3 araw(ss.accel.x,ss.accel.y,ss.accel.z),graw(ss.gyro.x,ss.gyro.y,ss.gyro.z);

        // Apply LPFs
        Vec3 ae,ge;
        eskf_lpf.apply(araw, graw, ae, ge);
        if(!gr_init){gr=graw;gr_init=true;}
        gr=gr*(1-rate_alpha)+graw*rate_alpha;

        if(t<1){quad.step(z,dt);continue;}
        if(t<2){quad.step(z,dt);if(cn<400){ca+=araw;cg+=graw;cn++;}
            if(cn==400&&!eskf_ready){Vec3 aa=ca*(1.f/400),ga=cg*(1.f/400);
                EskfConfig c;c.use_tof=true;c.use_baro=false;c.use_mag=false;c.use_flow=false;
                c.accel_noise=0.3f;c.gyro_noise=0.03f;c.tof_noise=0.01f;c.accel_att_noise=2.0f;
                eskf.init(c);eskf.setGyroBias(ga);
                eskf.setAccelBias(Vec3(aa.x,aa.y,aa.z+qp.gravity));
                eskf.setAttitudeFromGravity(aa);eskf.shrinkBiasCovariance(.01f);
                eskf.setFreezeAccelBias(true);eskf_ready=true;}continue;}
        if(eskf_ready){eskf.predict(ae,ge,dt);eskf.updateAccelAttitude(ae);
            if(s%tof_div==0){eskf.updateToF(ss.tof_bottom);eskf.updateToFVelocity(ss.tof_bottom,tof_div*dt);}}
        auto ts=quad.getState();Vec3 te=ts.attitude.to_euler();float th=-ts.position.z;
        Vec3 ee=eskf.getAttitude().to_euler(),ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();
        float er=ee.x,epi=ee.y,rx=gr.x-gb.x,ry_=gr.y-gb.y,rz=gr.z-gb.z;
        float eh=-ep.z,ec=-ev.z;
        if(th<.03f){eskf.resetPositionVelocity();eh=th;ec=-ts.velocity.z;}
        float T=0;float tau[3]={};
        float vs=alp.compute(.5f-eh,dt),tc=alv.compute(vs-ec,dt);T=ht+tc;
        float rs=ar.compute(0-er,dt),ps=ap.compute(0-epi,dt);
        tau[0]=rr.compute(rs-rx,dt);tau[1]=rp.compute(ps-ry_,dt);tau[2]=ry.compute(0-rz,dt);
        Vec3 gf={},gt={};for(int i=0;i<5;i++){if(t>=gs[i].t&&t<gs[i].t+gs[i].d){gf+=gs[i].f;gt+=gs[i].q;}}
        quad.setExternalForceBody(gf,gt);
        float md[4];mixer(T,tau,md,qp.k_thrust);quad.step(md,dt);
        if(fabsf(te.x)>1||fabsf(te.y)>1){Res r={};r.ok=false;return r;}
        if(t>3.5f){float a2=(te.x*te.x+te.y*te.y)*57.3f*57.3f;float ad=sqrtf(te.x*te.x+te.y*te.y)*57.3f;
            sa+=a2;sl+=(th-.5f)*(th-.5f);if(ad>ma)ma=ad;sn++;}}
    return{sqrtf(sa/sn),ma,sqrtf(sl/sn)*1000,true};
}

int main(){
    fprintf(stderr,"=== LPF Comparison: 1st-order IIR vs 2nd-order Butterworth ===\n");
    fprintf(stderr,"(att_kp=14, rate_td=0.008, R=2.0, gq=0.03)\n\n");
    fprintf(stderr,"%12s %6s %8s %8s %8s\n","Filter","fc","att_rms","att_max","alt_rms");
    fprintf(stderr,"----------------------------------------------\n");

    float fcs[] = {10,12,15,18,20,25,30,40};

    for(float fc : fcs){
        srand(42);
        Res r1 = run(LPF6::FIRST_ORDER, fc);
        srand(42);
        Res r2 = run(LPF6::BUTTERWORTH2, fc);

        if(r1.ok) fprintf(stderr,"%12s %5.0fHz %7.2f° %7.2f° %7.0fmm\n",
                          "1st-order",fc,r1.ar,r1.am,r1.alr);
        else fprintf(stderr,"%12s %5.0fHz UNSTABLE\n","1st-order",fc);

        if(r2.ok) fprintf(stderr,"%12s %5.0fHz %7.2f° %7.2f° %7.0fmm",
                          "Butterworth2",fc,r2.ar,r2.am,r2.alr);
        else fprintf(stderr,"%12s %5.0fHz UNSTABLE","Butterworth2",fc);

        if(r1.ok && r2.ok && r2.ar < r1.ar) fprintf(stderr," ← better");
        fprintf(stderr,"\n\n");
    }
    return 0;
}
