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
    float L=0.023f,k=0.00971f;
    float t=T*0.25f,r=tau[0]/(4*L),p=tau[1]/(4*L),y=tau[2]/(4*k);
    float t1=t-r+p+y,t2=t-r-p-y,t3=t+r-p+y,t4=t+r+p-y;
    for(int i=0;i<4;i++){float ti=(i==0)?t1:(i==1)?t2:(i==2)?t3:t4;
        if(ti<0)ti=0;d[i]=sqrtf(ti/kt);if(d[i]>0.95f)d[i]=0.95f;}
}

struct P{float R,gq,al,ng,ak;};
struct Res{float ar,am,alr; bool ok;};

static Res run(P p){
    QuadPhysics quad;QuadParams qp;ContactParams cp;SensorNoiseParams np;
    quad.init(qp,cp,np);
    EskfCore eskf;bool er=false;
    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f;rr.ti=0.7f;rr.td=0.01f;rr.output_limit=5.2e-3f;rr.eta=0.125f;
    rp=rr;rp.kp=1.995e-3f;
    ry.kp=5.31e-3f;ry.ti=1.6f;ry.output_limit=2.2e-3f;
    ar.kp=p.ak;ar.ti=4.0f;ar.output_limit=3.0f;ap=ar;
    alp.kp=1.2f;alp.ti=3.0f;alp.output_limit=0.5f;
    alv.kp=0.20f;alv.ti=1.0f;alv.output_limit=0.25f;
    Vec3 ge={},ae={},gr={};bool li=false;
    float dt=0.0025f;int steps=(int)(12/dt),td=13;float ht=qp.mass*qp.gravity;
    Vec3 ca={},cg={};int cn=0;
    struct G{float t,d;Vec3 f,q;};
    G gs[]={{2.5f,.3f,{.005f,0,0},{0,3e-5f,0}},{4,.5f,{0,-.012f,0},{6e-5f,0,0}},
           {6.5f,.2f,{.015f,.01f,0},{-5e-5f,7e-5f,0}},{8.5f,1.5f,{.003f,-.003f,0},{2e-5f,-2e-5f,0}},
           {10.5f,.15f,{.02f,0,.005f},{0,8e-5f,0}}};
    float sa=0,sl=0,ma=0;int sn=0;
    for(int s=0;s<steps;s++){
        float t=s*dt;float z[4]={};
        SimSensors ss=quad.getSensors(dt);quad.updateBiases(dt);
        Vec3 a(ss.accel.x,ss.accel.y,ss.accel.z),g(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=g;ae=a;gr=g;li=true;}
        ge=ge*(1-p.al)+g*p.al;ae=ae*(1-p.al)+a*p.al;gr=gr*(1-0.67f)+g*0.67f;
        if(t<1){quad.step(z,dt);continue;}
        if(t<2){quad.step(z,dt);
            if(cn<400){ca+=a;cg+=g;cn++;}
            if(cn==400&&!er){Vec3 aa=ca*(1.0f/400),ga=cg*(1.0f/400);
                EskfConfig c;c.use_tof=true;c.use_baro=false;c.use_mag=false;c.use_flow=false;
                c.accel_noise=0.3f;c.gyro_noise=p.gq;c.tof_noise=0.01f;c.accel_att_noise=p.R;
                eskf.init(c);eskf.setGyroBias(ga);
                eskf.setAccelBias(Vec3(aa.x,aa.y,aa.z+qp.gravity));
                eskf.setAttitudeFromGravity(aa);eskf.shrinkBiasCovariance(0.01f);
                eskf.setFreezeAccelBias(true);er=true;}
            continue;}
        if(er){eskf.predict(ae,ge,dt);eskf.updateAccelAttitude(ae);
            if(s%td==0){eskf.updateToF(ss.tof_bottom);eskf.updateToFVelocity(ss.tof_bottom,td*dt);}}
        auto ts=quad.getState();Vec3 te=ts.attitude.to_euler();float th=-ts.position.z;
        Vec3 ee=eskf.getAttitude().to_euler(),ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();
        float _er=ee.x,_ep=ee.y,rx=gr.x-gb.x,_ry=gr.y-gb.y,rz=gr.z-gb.z;
        float eh=-ep.z,ec=-ev.z;
        if(th<0.03f){eskf.resetPositionVelocity();eh=th;ec=-ts.velocity.z;}
        float T=0;float tau[3]={};
        float vs=alp.compute(.5f-eh,dt),tc=alv.compute(vs-ec,dt);T=ht+tc;
        float rs=ar.compute(0-_er,dt),ps=ap.compute(0-_ep,dt);
        tau[0]=rr.compute(rs-rx,dt);tau[1]=rp.compute(ps-_ry,dt);tau[2]=ry.compute(0-rz,dt);
        Vec3 gf={},gt={};
        for(int i=0;i<5;i++){if(t>=gs[i].t&&t<gs[i].t+gs[i].d){gf+=gs[i].f;gt+=gs[i].q;}}
        quad.setExternalForceBody(gf,gt);
        float md[4];mixer(T,tau,md,qp.k_thrust);quad.step(md,dt);
        if(fabsf(te.x)>1||fabsf(te.y)>1){Res r={};r.ok=false;return r;}
        if(t>3.5f){float a2=(te.x*te.x+te.y*te.y)*57.3f*57.3f;
            float ad=sqrtf(te.x*te.x+te.y*te.y)*57.3f;
            sa+=a2;sl+=(th-.5f)*(th-.5f);if(ad>ma)ma=ad;sn++;}
    }
    return{sqrtf(sa/sn),ma,sqrtf(sl/sn)*1000,true};
}

int main(){
    fprintf(stderr,"=== Fine Tuning Grid ===\n\n");
    fprintf(stderr,"%6s %6s %6s %8s %8s %8s\n","alpha","att_kp","R","att_rms","att_max","alt_rms");
    fprintf(stderr,"--------------------------------------------------\n");

    float als[]={0.12f,0.15f,0.18f,0.20f};
    float aks[]={6.0f,8.0f,10.0f,12.0f};
    float Rs[]={1.5f,2.0f,2.5f};

    float best=999;float ba=0,bk=0,br=0;
    for(float al:als) for(float ak:aks) for(float R:Rs){
        srand(42);
        Res r=run({R,0.03f,al,0.05f,ak});
        if(r.ok){
            fprintf(stderr,"%6.2f %6.1f %6.1f %7.2f° %7.2f° %7.0fmm",
                    al,ak,R,r.ar,r.am,r.alr);
            if(r.ar<best&&r.am<7.0f&&r.alr<60){
                best=r.ar;ba=al;bk=ak;br=R;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%6.2f %6.1f %6.1f  UNSTABLE\n",al,ak,R);
    }

    fprintf(stderr,"\n=== Best: alpha=%.2f att_kp=%.1f R=%.1f → att_rms=%.2f° ===\n",ba,bk,br,best);
    return 0;
}

// Append: extended search
// Already has run() defined, just add main2
