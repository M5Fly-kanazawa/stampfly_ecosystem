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

static void run_sine(float freq, FILE* csv) {
    QuadPhysics quad; QuadParams qp; ContactParams cp; SensorNoiseParams np;
    quad.init(qp, cp, np);

    // Start airborne with motors at hover
    auto& st = const_cast<QuadState&>(quad.getState());
    st.position.z = -0.5f;
    float hover_duty = sqrtf(qp.mass * qp.gravity / (4.0f * qp.k_thrust));
    for (int i = 0; i < 4; i++) st.motor_speed[i] = hover_duty;

    EskfCore eskf;
    EskfConfig cfg;
    cfg.use_tof=true; cfg.use_baro=false; cfg.use_mag=false; cfg.use_flow=false;
    cfg.accel_noise=0.3f; cfg.gyro_noise=0.03f; cfg.tof_noise=0.01f;
    cfg.accel_att_noise=2.0f;
    eskf.init(cfg);
    // Simple cal: assume level, no bias (airborne start)
    eskf.setGyroBias({0,0,0});
    eskf.setAccelBias({0,0,0});
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
    float sim_time = 2.0f + 3.0f / fmaxf(freq, 0.1f);  // 2s settle + 3 cycles
    if (sim_time > 15.0f) sim_time = 15.0f;
    int steps=(int)(sim_time/dt), tof_div=13;
    float ht=qp.mass*qp.gravity;
    float amp = 15.0f * M_PI / 180.0f;  // 15 degrees in radians

    for (int s=0; s<steps; s++) {
        float t = s * dt;

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
        Vec3 te=ts.attitude.to_euler();
        Vec3 ee=eskf.getAttitude().to_euler();
        Vec3 ep=eskf.getPosition(),ev=eskf.getVelocity(),gb=eskf.getGyroBias();

        // Sine wave command (starts after 1s settle)
        float cmd_roll = 0, cmd_pitch = 0;
        if (t > 1.0f) {
            cmd_roll = amp * sinf(2.0f * M_PI * freq * (t - 1.0f));
        }

        float er=ee.x, epi=ee.y;
        float rx=gr.x-gb.x, ry_=gr.y-gb.y, rz=gr.z-gb.z;
        float eh=-ep.z, ec=-ev.z;

        float T=0; float tau[3]={};
        float vs=alp.compute(0.5f-eh,dt), tc=alv.compute(vs-ec,dt);
        T=ht+tc;
        float rs=ar.compute(cmd_roll-er,dt), ps=ap.compute(cmd_pitch-epi,dt);
        tau[0]=rr.compute(rs-rx,dt); tau[1]=rp.compute(ps-ry_,dt);
        tau[2]=ry.compute(0-rz,dt);

        float md[4]; mixer(T,tau,md,qp.k_thrust); quad.step(md,dt);

        // CSV at 200Hz
        if (s % 2 == 0) {
            fprintf(csv, "%.4f,%.3f,%.3f,%.3f,%.3f,%.4f\n",
                    t,
                    cmd_roll * 57.3f,
                    te.x * 57.3f,
                    ee.x * 57.3f,
                    (te.x - cmd_roll) * 57.3f,
                    -ts.position.z);
        }
    }
}

int main() {
    float freqs[] = {0.3f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 5.0f};
    int nf = 7;

    fprintf(stderr, "=== Sine Response Test (L4 ESKF, roll ±15°) ===\n\n");

    for (int i = 0; i < nf; i++) {
        srand(42);
        char fn[64];
        snprintf(fn, sizeof(fn), "sine_%.1fHz.csv", freqs[i]);
        FILE* csv = fopen(fn, "w");
        fprintf(csv, "time,cmd_roll,true_roll,eskf_roll,error,altitude\n");
        run_sine(freqs[i], csv);
        fclose(csv);
        fprintf(stderr, "  %.1f Hz → %s\n", freqs[i], fn);
    }

    fprintf(stderr, "\nDone. Plot with sine_response_plot.py\n");
    return 0;
}
