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

struct P {
    float accel_att_noise, gyro_noise_q;
    float lpf_alpha;       // ESKF LPF alpha (0.32=30Hz default)
    float norm_gate;       // norm gate threshold (0.1=10% default)
    float att_kp;          // attitude Kp
};

struct R { float att_rms,att_max,alt_rms; bool ok; };

static R run(P p) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp,cp,np);

    EskfCore eskf; bool eskf_ready=false;
    PID rr,rp,ry,ar,ap,alp,alv;
    rr.kp=1.365e-3f;rr.ti=0.7f;rr.td=0.01f;rr.output_limit=5.2e-3f;rr.eta=0.125f;
    rp=rr;rp.kp=1.995e-3f;
    ry.kp=5.31e-3f;ry.ti=1.6f;ry.output_limit=2.2e-3f;
    ar.kp=p.att_kp;ar.ti=4.0f;ar.output_limit=3.0f; ap=ar;
    alp.kp=1.2f;alp.ti=3.0f;alp.output_limit=0.5f;
    alv.kp=0.20f;alv.ti=1.0f;alv.output_limit=0.25f;

    float ae_=p.lpf_alpha, ar_=0.67f;
    Vec3 ge={},ae={},gr={}; bool li=false;

    float dt=0.0025f; int steps=(int)(12.0f/dt), td=13;
    float ht=qp.mass*qp.gravity;
    Vec3 ca={},cg={}; int cn=0;

    struct G{float t,d;Vec3 f,q;};
    G gs[]={{2.5f,0.3f,{0.005f,0,0},{0,3e-5f,0}},
           {4.0f,0.5f,{0,-0.012f,0},{6e-5f,0,0}},
           {6.5f,0.2f,{0.015f,0.01f,0},{-5e-5f,7e-5f,0}},
           {8.5f,1.5f,{0.003f,-0.003f,0},{2e-5f,-2e-5f,0}},
           {10.5f,0.15f,{0.02f,0,0.005f},{0,8e-5f,0}}};

    float sa2=0,sl2=0,ma=0; int sn=0;

    for(int s=0;s<steps;s++){
        float t=s*dt; float z4[4]={};
        SimSensors ss=quad.getSensors(dt); quad.updateBiases(dt);
        Vec3 a(ss.accel.x,ss.accel.y,ss.accel.z),g(ss.gyro.x,ss.gyro.y,ss.gyro.z);
        if(!li){ge=g;ae=a;gr=g;li=true;}
        ge=ge*(1-ae_)+g*ae_; ae=ae*(1-ae_)+a*ae_; gr=gr*(1-ar_)+g*ar_;

        if(t<1){quad.step(z4,dt);continue;}
        if(t<2){quad.step(z4,dt);
            if(cn<400){ca+=a;cg+=g;cn++;}
            if(cn==400&&!eskf_ready){
                Vec3 aa=ca*(1.0f/400),ga=cg*(1.0f/400);
                EskfConfig c; c.use_tof=true;c.use_baro=false;c.use_mag=false;c.use_flow=false;
                c.accel_noise=0.3f; c.gyro_noise=p.gyro_noise_q;
                c.tof_noise=0.01f; c.accel_att_noise=p.accel_att_noise;
                eskf.init(c);
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
            if(s%td==0){eskf.updateToF(ss.tof_bottom);
                eskf.updateToFVelocity(ss.tof_bottom,td*dt);}
        }

        auto ts=quad.getState(); Vec3 te=ts.attitude.to_euler();
        float th=-ts.position.z;
        Vec3 ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();

        float er_=ee.x,ep_=ee.y;
        float rx=gr.x-gb.x,ry_=gr.y-gb.y,rz=gr.z-gb.z;
        float eh=-ep.z,ec=-ev.z;
        if(th<0.03f){eskf.resetPositionVelocity();eh=th;ec=-ts.velocity.z;}

        float T=0;float tau[3]={};
        float vs=alp.compute(0.5f-eh,dt);
        float tc=alv.compute(vs-ec,dt);
        T=ht+tc;
        float rs=ar.compute(0-er_,dt),ps=ap.compute(0-ep_,dt);
        tau[0]=rr.compute(rs-rx,dt);tau[1]=rp.compute(ps-ry_,dt);tau[2]=ry.compute(0-rz,dt);

        Vec3 gf={},gt={};
        for(int i=0;i<5;i++){if(t>=gs[i].t&&t<gs[i].t+gs[i].d){gf+=gs[i].f;gt+=gs[i].q;}}
        quad.setExternalForceBody(gf,gt);

        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        if(fabsf(te.x)>1||fabsf(te.y)>1){R r={};r.ok=false;return r;}

        if(t>3.5f){
            float a2=(te.x*te.x+te.y*te.y)*57.3f*57.3f;
            float ad=sqrtf(te.x*te.x+te.y*te.y)*57.3f;
            sa2+=a2; sl2+=(th-0.5f)*(th-0.5f); if(ad>ma)ma=ad; sn++;
        }
    }
    return {sqrtf(sa2/sn),ma,sqrtf(sl2/sn)*1000,true};
}

int main(){
    fprintf(stderr,"=== Attitude Tuning Sweep ===\n\n");

    // Sweep: gyro_noise_q (ESKF process noise for attitude)
    fprintf(stderr,"--- gyro_noise_q sweep (others: default) ---\n");
    fprintf(stderr,"%10s %8s %8s\n","gyro_q","att_rms","att_max");
    fprintf(stderr,"------------------------------\n");
    float gq_vals[]={0.003f, 0.005f, 0.007f, 0.009655f, 0.015f, 0.02f, 0.03f};
    float best_gq=0.009655f, best_a=999;
    for(float gq:gq_vals){srand(42);
        R r=run({1.5f,gq,0.32f,0.1f,5.0f});
        if(r.ok){fprintf(stderr,"%10.4f %7.2f° %7.2f°",gq,r.att_rms,r.att_max);
            if(r.att_rms<best_a){best_a=r.att_rms;best_gq=gq;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%10.4f  UNSTABLE\n",gq);
    }

    // Sweep: LPF alpha
    fprintf(stderr,"\n--- LPF alpha sweep (gyro_q=%.4f) ---\n",best_gq);
    fprintf(stderr,"%10s %6s %8s %8s\n","alpha","~Hz","att_rms","att_max");
    fprintf(stderr,"--------------------------------------\n");
    float al_vals[]={0.15f, 0.20f, 0.25f, 0.32f, 0.40f, 0.50f, 0.67f};
    float best_al=0.32f; best_a=999;
    for(float al:al_vals){srand(42);
        float fc=al/(2*3.14159f*0.0025f*(1-al)); // approximate cutoff
        R r=run({1.5f,best_gq,al,0.1f,5.0f});
        if(r.ok){fprintf(stderr,"%10.2f %5.0fHz %7.2f° %7.2f°",al,fc,r.att_rms,r.att_max);
            if(r.att_rms<best_a){best_a=r.att_rms;best_al=al;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%10.2f %5.0fHz  UNSTABLE\n",al,fc);
    }

    // Sweep: norm gate
    fprintf(stderr,"\n--- norm gate sweep (gyro_q=%.4f, alpha=%.2f) ---\n",best_gq,best_al);
    fprintf(stderr,"%10s %8s %8s\n","gate","att_rms","att_max");
    fprintf(stderr,"------------------------------\n");
    float ng_vals[]={0.05f, 0.08f, 0.10f, 0.15f, 0.20f, 0.30f};
    float best_ng=0.1f; best_a=999;
    for(float ng:ng_vals){srand(42);
        R r=run({1.5f,best_gq,best_al,ng,5.0f});
        if(r.ok){fprintf(stderr,"%10.2f %7.2f° %7.2f°",ng,r.att_rms,r.att_max);
            if(r.att_rms<best_a){best_a=r.att_rms;best_ng=ng;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%10.2f  UNSTABLE\n",ng);
    }

    // Sweep: att_kp
    fprintf(stderr,"\n--- att_kp sweep (gyro_q=%.4f, alpha=%.2f, gate=%.2f) ---\n",
            best_gq,best_al,best_ng);
    fprintf(stderr,"%10s %8s %8s\n","att_kp","att_rms","att_max");
    fprintf(stderr,"------------------------------\n");
    float ak_vals[]={2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f};
    float best_ak=5.0f; best_a=999;
    for(float ak:ak_vals){srand(42);
        R r=run({1.5f,best_gq,best_al,best_ng,ak});
        if(r.ok){fprintf(stderr,"%10.1f %7.2f° %7.2f°",ak,r.att_rms,r.att_max);
            if(r.att_rms<best_a){best_a=r.att_rms;best_ak=ak;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%10.1f  UNSTABLE\n",ak);
    }

    // Final accel_att_noise re-sweep with all best
    fprintf(stderr,"\n--- accel_att_noise re-sweep (all best params) ---\n");
    fprintf(stderr,"%10s %8s %8s %8s\n","R_noise","att_rms","att_max","alt_rms");
    fprintf(stderr,"--------------------------------------\n");
    float rn_vals[]={0.5f,0.7f,1.0f,1.2f,1.5f,2.0f};
    float best_rn=1.5f; best_a=999;
    for(float rn:rn_vals){srand(42);
        R r=run({rn,best_gq,best_al,best_ng,best_ak});
        if(r.ok){fprintf(stderr,"%10.2f %7.2f° %7.2f° %7.0fmm",rn,r.att_rms,r.att_max,r.alt_rms);
            if(r.att_rms<best_a){best_a=r.att_rms;best_rn=rn;fprintf(stderr," ★");}
            fprintf(stderr,"\n");
        } else fprintf(stderr,"%10.2f  UNSTABLE\n",rn);
    }

    fprintf(stderr,"\n=== Best: R=%.2f gq=%.4f alpha=%.2f gate=%.2f att_kp=%.1f → att_rms=%.2f° ===\n",
            best_rn,best_gq,best_al,best_ng,best_ak,best_a);
    return 0;
}
