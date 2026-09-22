#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SensorlessStartup.h"
#include "SensorlessTravel90.h"
static float wr(float t){return t-floorf(t);}
static float df(float t){return t-floorf(t+0.5f);}
static SL_PROBE p;
static SL_PROBE_INPUT x;
static uint32_t tick;
static int32_t theta;
static float id,iq;
static int dir;
static unsigned checks,variant;
static void initial(int direction)
{
    uint32_t n;
    memset(&p,0,sizeof(p));p.a2=variant==1;p.a3=variant==2;memset(&x,0,sizeof(x));dir=direction;theta=0;
    x.encoder_valid=x.chain_valid=x.locked=1;
    x.emf_a=x.emf=0.004f;x.id=x.id_ref=id=0.03125f;iq=0.0f;
    for(n=1;n<=8000;++n){x.tick=n;x.encoder_read_tick=n&~15U;
        x.encoder_request_tick=x.encoder_read_tick-16U;
        assert(!SlProbe_Step(&p,&x,1,n,(int16_t)dir,0,0,&theta,&id,&iq));}
    tick=8000U;
    assert(p.baseline_count==64U);
    x.emf_a=x.emf=0.03f;x.phase=0.04f;x.id=x.id_ref=id=0.0f;
    x.iq=x.iq_ref=iq=(float)dir*0.03125f;x.smo_rpm=(float)dir*100.0f;
}
static uint32_t sample(uint32_t n,unsigned fault)
{
    float f=wr((float)dir*(float)n/960.0f);
    uint32_t out;
    x.tick=++tick;x.encoder_read_tick=x.tick&~15U;x.encoder_request_tick=x.encoder_read_tick-16U;
    x.encoder_raw=(uint32_t)(wr((float)dir*(float)(x.encoder_read_tick-8000U)/9600.0f)*131072.0f);
    x.smo_theta_q24=(int32_t)(wr(f+(float)dir*45.85f/360.0f)*16777216.0f);
    x.actual_theta_q24=theta;
    x.id=x.id_ref=id;x.iq=x.iq_ref=iq;
    if(fault==1)x.locked=0;
    if(fault==2)x.encoder_valid=0;
    if(fault==3)x.chain_valid=0;
    if(fault==4)x.saturated=1;
    if(fault==5)x.phase=0.2f;
    if(fault==6)x.emf=0.0f;
    if(fault==7)x.smo_rpm= -(float)dir*100.0f;
    if(fault==8)x.iq=(float)dir*0.037f;
    if(fault==9)x.id=0.01f;
    if(fault==10)x.smo_theta_q24=(int32_t)(wr(f+0.25f)*16777216.0f);
    if(fault==11)p.stop=1;
    if(fault==12)x.id=NAN;
    if(fault==13)x.phase=NAN;
    if(fault==14)x.smo_rpm=150.0f*(float)dir;
    if(fault==15)x.encoder_raw=(x.encoder_raw+(uint32_t)1000)&131071U;
    out=SlProbe_Step(&p,&x,0,n,(int16_t)dir,(int32_t)(f*16777216.0f),
        (float)dir*100.0f,&theta,&id,&iq);
    return out;
}
static uint32_t enter(void)
{
    uint32_t n;
    for(n=1;n<=48000U;++n){assert(!sample(n,0));if(p.phase==SL_P_RISE)break;}
    assert(p.baseline_valid && p.phase==SL_P_RISE);
    assert(n>=35839U && n<=35842U);return n;
}
static void normal(int direction)
{
    uint32_t n;float prev=0;
    initial(direction);n=enter();
    for(;n<60000U;++n){
        float f=wr((float)dir*(float)(n+1U)/960.0f);
        assert(!sample(n+1U,0));
        assert(fabsf(p.offset)<=(variant==2?15.0f:(variant?6.0f:3.0f))/360.0f);
        if(p.phase==SL_P_HOLD)assert(fabsf(p.ratio-(variant==2?0.25f:(variant?0.10f:0.05f)))<0.000001f);
        assert(fabsf(p.offset-prev)<=1.0f/96000.0f+0.0000001f);
        assert(fabsf(df((float)theta/16777216.0f-f)-p.offset)<0.000001f);
        assert(iq==(float)dir*0.03125f && id==0.0f);prev=p.offset;
        if(p.phase==SL_P_DONE)break;
    }
    assert(p.phase==SL_P_DONE && p.offset==0.0f && p.max_offset>2.2f/360.0f);
    assert(p.done_tick-p.entry_tick==(variant==2?17919U:(variant?8319U:5119U)));
    assert(p.max_offset>(variant==2?11.0f:(variant?4.4f:2.2f))/360.0f);
    assert(!p.reason);++checks;
}
static void failures(int direction)
{
    unsigned fault,stage;uint32_t n,j,out;float last;
    for(stage=0;stage<3;++stage)for(fault=1;fault<=15;++fault){
        initial(direction);n=enter();
        for(j=0;j<(stage==0?400U:(stage==1?(variant==2?8200U:(variant?3400U:1800U)):(variant==2?10200U:(variant?5400U:3800U))));++j)assert(!sample(++n,0));
        last=(float)theta/16777216.0f;
        for(j=0;j<500U;++j){out=sample(++n,fault);if(out || p.phase==SL_P_RELEASE)break;}
        if(fault==12){assert(out==2U);++checks;continue;}
        assert(p.phase==SL_P_RELEASE && p.reason);
        if(fault==1 || fault==2 || fault==3 || fault==7 || fault==10 || fault==11 || fault==13)
            assert(j==0U);
        if(fault==4 || fault==5 || fault==6 || fault==8 || fault==9)assert(j==31U);
        if(j==0U)assert(fabsf(df((float)theta/16777216.0f-last)-(float)dir/960.0f)<0.000001f);
        for(j=0;j<850U;++j){float prev=iq;out=sample(++n,0);
            assert(fabsf(iq)<=fabsf(prev)+0.0000001f);
            if(out)break;}
        assert(out==1U && p.phase==SL_P_OFF && p.cycles<=800U && iq==0 && id==0);
        ++checks;
    }
}
static void skip_tests(void)
{
    uint32_t n;
    initial(1);p.baseline_count=63;
    for(n=1;n<=48010;++n)assert(!sample(n,0));
    assert(p.phase==SL_P_SKIP && !p.baseline_valid && p.offset==0);++checks;
    initial(1);x.phase=0.12f;
    for(n=1;n<=48010;++n)assert(!sample(n,0));
    assert(p.phase==SL_P_SKIP && (p.failed_ever&SL_P_PHASE));++checks;
}
static void timeout_stop(void)
{
    uint32_t n,j,out=0;
    initial(1);n=enter();assert(!sample(++n,11));
    for(j=1;j<800;++j){x.tick=++tick;x.id=0.01f;x.iq=0;out=SlProbe_Step(&p,&x,0,++n,1,0,100,&theta,&id,&iq);
        if(j<799)assert(!out);}
    assert(out==1U && p.cycles==800);++checks;
}
static void startup_integration(void)
{
    SENSORLESS_STARTUP_CONFIG c;SENSORLESS_STARTUP_RUNTIME r;SENSORLESS_STARTUP_INPUT in;
    uint32_t n;
    SensorlessStartup_DefaultConfig(&c,1.0f/16000.0f,10,8192,600,0.03125f);
    c.debug_mode=variant==2?SENSORLESS_DEBUG_SMO_A3:(variant?SENSORLESS_DEBUG_SMO_A2:SENSORLESS_DEBUG_SMO_A1);c.hfi_enable=0;c.allow_open_loop_fallback=1;
    c.align_cycles=8000;c.current_ramp_cycles=3200;c.align_current_pu=c.startup_iq_pu=0.03125f;
    c.handoff_speed_rpm=100;c.sensorless_exit_speed_rpm=20;
    c.open_loop_accel_rpm_s=c.run_accel_rpm_s=100;
    c.ramp_timeout_cycles=288000;c.total_timeout_cycles=320000;
    assert(SensorlessStartup_ValidateConfig(&c));
    SensorlessStartup_Init(&r,&c);memset(&in,0,sizeof(in));in.drive_enabled=in.params_valid=1;
    assert(SensorlessStartup_RequestStart(&r,1,100));
    for(n=0;n<10000;++n)SensorlessStartup_Step(&r,&in);
    assert(r.state==SENSORLESS_START_OPEN_LOOP_RAMP && r.output.control_active);
    assert(r.probe.a2==(variant==1) && r.probe.a3==(variant==2));
    SensorlessStartup_RequestStop(&r);assert(r.probe.stop && r.output.control_active);
    for(n=0;n<1000 && r.state!=SENSORLESS_START_IDLE;++n)SensorlessStartup_Step(&r,&in);
    assert(r.state==SENSORLESS_START_IDLE && !r.output.control_active && r.probe.phase==SL_P_OFF);
    assert(r.speed_integrator_pu==0.0f);++checks;
    assert(SensorlessStartup_RequestStart(&r,-1,100));SensorlessStartup_Step(&r,&in);
    SensorlessStartup_Step(&r,&in);in.observer_fault_count=1;SensorlessStartup_Step(&r,&in);
    assert(r.state==SENSORLESS_START_FAULT && !r.output.control_active);++checks;
}
static void traces(void)
{
    static SL_PROBE_TRACE t;SL_PROBE_LONG s;uint32_t n;
    memset(&s,0,sizeof(s));memset(&p,0,sizeof(p));SlProbeTrace_Begin(&t,14);
    for(n=1;n<=16000;++n){s.tick=n;if(n==6400)p.entry_tick=n;
        if(n==7000)p.reason=SL_P_LOCK;
        SlProbeTrace_Record(&t,&p,&s,123,-456);}
    assert(t.h.frozen && t.h.long_count==SL_PROBE_LONG_N && t.h.fast_count==96);
    assert(t.h.fast_trigger_tick==7000 && t.fast[(t.h.fast_write+95)%96].tick==7016);
    assert(t.slow[t.h.long_write].tick<=4800);++checks;
}
static void live_filter(void)
{
    uint32_t n,j,phase;int32_t before_theta;float before_iq,before_id;
    for(phase=SL_P_DONE;phase<=SL_P_SKIP;++phase) {
        initial(1);n=enter();p.phase=phase;p.offset=0.0f;
        before_theta=theta;before_iq=iq;before_id=id;x.smo_rpm=105.0f;
        for(j=0;j<704;++j)assert(!sample(++n,0));
        assert(p.filter_valid && fabsf(p.smo_mean-105.0f)<0.001f);
        assert(fabsf(p.enc_rpm-100.0f)<0.1f);
        assert(theta==before_theta && iq==before_iq && id==before_id);
        x.encoder_valid=0;
        for(j=0;j<32;++j)assert(!sample(++n,0));
        assert(!p.filter_valid);
        x.encoder_valid=1;x.smo_rpm=97.0f;
        for(j=0;j<704;++j)assert(!sample(++n,15));
        assert(p.filter_valid && fabsf(p.smo_mean-97.0f)<0.001f);
        assert(fabsf(p.follow-p.follow_zero)>0.07f);
        assert(p.phase==phase && !p.reason && theta==before_theta);
        ++checks;
    }
}
static void extended_traces(void)
{
    static SL_PROBE_TRACE t;SL_PROBE_LONG s;SL_PROBE probe;
    uint32_t n,k,done,found,mode;
    for(mode=0;mode<3;++mode) {
        memset(&probe,0,sizeof(probe));memset(&s,0,sizeof(s));
        probe.a2=variant==1;probe.a3=variant==2;
        done=20000U+(mode==0?(variant==2?17919U:8319U):(mode==1?(variant==2?19519U:9919U):22000U));
        SlProbeTrace_Begin(&t,16);
        for(n=1;n<=done+1920U;++n) {
            s.tick=n;
            if(n==20000 && mode!=2)probe.entry_tick=n;
            if(n==done) {probe.phase=mode==2?SL_P_SKIP:SL_P_DONE;probe.done_tick=n;}
            s.phase=probe.phase;SlProbeTrace_Record(&t,&probe,&s,123,-456);
            if(n<done+1920U)assert(t.h.active);
        }
        assert(t.h.frozen && t.h.abi==4 && t.h.long_count==SL_PROBE_LONG_N);
        assert(t.slow[(t.h.long_write+SL_PROBE_LONG_N-1U)%SL_PROBE_LONG_N].tick==done+1920U);
        found=0;for(k=0;k<SL_PROBE_LONG_N;++k)if(t.slow[k].tick==done)found=1;
        assert(found);
        if(mode!=2)assert(t.slow[t.h.long_write].tick+3840U<=probe.entry_tick);
        ++checks;
    }
}
int main(void)
{
    for(variant=0;variant<=2;++variant) {
        normal(1);normal(-1);failures(1);failures(-1);skip_tests();timeout_stop();
        startup_integration();live_filter();
    }
    traces();for(variant=1;variant<=2;++variant)extended_traces();
    printf("PASS A1/A2/A3: %u scenarios, bidirectional wrap, gates, release, skip, hard priority, trace\n",checks);
    return 0;
}
