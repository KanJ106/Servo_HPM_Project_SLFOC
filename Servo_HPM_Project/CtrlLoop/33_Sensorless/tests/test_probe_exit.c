#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SensorlessStartup.h"
static float wr(float f){return f-floorf(f);}
static SL_PROBE p;static SL_PROBE_INPUT x;static int32_t theta;static float id,iq;
static void init(void)
{
    memset(&p,0,sizeof(p));memset(&x,0,sizeof(x));p.a3=1;
    p.baseline_done=p.baseline_valid=1;p.entry_emf=.01f;p.exit_emf=.009f;
    p.phase=SL_P_DONE;x.encoder_valid=x.chain_valid=x.locked=1;
    x.emf=x.emf_a=.02f;x.smo_rpm=100;x.iq=x.iq_ref=iq=.03125f;id=0;
}
static unsigned step(unsigned n,float drift)
{
    float f=wr((float)n/960.0f);unsigned rd=n&~15U;
    x.tick=n;x.encoder_read_tick=rd;x.encoder_request_tick=rd-16U;
    x.encoder_raw=(uint32_t)(wr((float)rd/9600.0f+drift/3600.0f)*131072.0f);
    x.actual_theta_q24=(int32_t)((n*1024U)&0xffffffU);
    x.smo_theta_q24=(int32_t)(wr(f+.125f)*16777216.0f);
    return SlProbe_Step(&p,&x,0,40000U+n,1,(int32_t)(f*16777216.0f),100,&theta,&id,&iq);
}
static void follow_freeze(void)
{
    unsigned n;SL_PROBE_EXIT_DIAG first;
    init();for(n=1;n<=400;n++)assert(!step(n,0));
    p.follow_zero=p.follow_sum*.1f;p.phase=SL_P_RISE;
    for(;n<6000;n++) {
        float d=(float)(n-400U)*16.0f/4800.0f;
        assert(!step(n,d));if(p.phase==SL_P_RELEASE)break;
    }
    assert(p.reason==SL_P_FOLLOW && p.bad[6]==80);
    first=p.exit_diag;
    assert(first.abi==1 && first.first_valid && first.first_reason==SL_P_FOLLOW);
    assert(first.first_phase==SL_P_RISE && first.first_tick==n && first.bad[6]==80);
    assert(first.follow.valid && first.follow.reference_valid);
    assert(first.follow.reference_tick==first.follow.read_tick);
    assert(first.follow.reference_theta_q24==(int32_t)((first.follow.read_tick*1024U)&0xffffffU));
    assert(fabsf(first.follow.follow-first.follow_zero)>15.0f/360.0f);
    assert(first.input.tick==n && first.follow.tick<=n && n-first.follow.tick<32);
    x.id=.01f;for(unsigned j=1;j<800;j++)step(++n,30);
    assert(p.exit_diag.end_reason==SL_PE_DEADLINE && p.exit_diag.end_cycles==800);
    assert(!memcmp(&p.exit_diag.input,&first.input,sizeof(x)));
    assert(!memcmp(&p.exit_diag.follow,&first.follow,sizeof(first.follow)));
    assert(p.exit_diag.first_reason==first.first_reason && !(p.exit_diag.sequence&1U));
    SlProbe_EndExternal(&p,n+200,123);assert(p.exit_diag.end_reason==SL_PE_DEADLINE);
}
static void terminal(void)
{
    for(unsigned mode=0;mode<3;mode++) {
        unsigned n,out=0;init();p.phase=SL_P_RELEASE;p.reason=SL_P_FOLLOW;
        p.release_iq=.03125f;p.last_speed=100;x.iq=0;
        for(n=1;n<=800;n++) {
            x.id=mode==0?0.0f:(mode==1?.01f:(n<641?.01f:0.0f));
            x.tick=n;out=SlProbe_Step(&p,&x,0,n,1,0,100,&theta,&id,&iq);
            if(out)break;
        }
        assert(out==1 && id==0 && iq==0 && p.reason==SL_P_FOLLOW);
        assert(n==(mode==0?479U:800U));
        assert(p.exit_diag.end_reason==(mode==1?SL_PE_DEADLINE:SL_PE_LOW_CURRENT));
        assert(p.exit_diag.end_tick==n && p.exit_diag.end_cycles==n);
    }
    init();SlProbe_EndExternal(&p,10,123);assert(p.exit_diag.end_reason==SL_PE_HARD_FAULT);
    assert(p.exit_diag.hard_detail==123 && !p.exit_diag.first_valid);
    init();SlProbe_EndExternal(&p,10,0);assert(p.exit_diag.end_reason==SL_PE_EXTERNAL_OFF);
    init();x.id=NAN;assert(step(1,0)==2);assert(p.exit_diag.first_reason==SL_P_NUMERIC);
    assert(p.exit_diag.end_reason==SL_PE_HARD_FAULT);
}
static void reference_validity(void)
{
    init();for(unsigned n=1;n<=32;n++)step(n,0);
    assert(p.follow_snapshot.reference_valid);
    x.encoder_valid=0;for(unsigned n=33;n<=64;n++)step(n,0);
    assert(!p.follow_snapshot.valid && !p.follow_snapshot.reference_valid);
    init();p.sample_div=31;x.tick=1000;x.encoder_read_tick=990;
    SlProbe_Step(&p,&x,0,1,1,0,100,&theta,&id,&iq);
    assert(p.follow_snapshot.valid && !p.follow_snapshot.reference_valid);
    init();for(unsigned n=1;n<=96;n++)step(n,0);
    p.sample_div=31;x.tick=200;x.encoder_read_tick=96;
    SlProbe_Step(&p,&x,0,1,1,0,100,&theta,&id,&iq);
    assert(!p.follow_snapshot.reference_valid);
    init();for(unsigned j=0;j<64;j++) {
        x.tick=0xffffffe0U+j;x.encoder_read_tick=x.tick;x.encoder_request_tick=x.tick-16;
        x.actual_theta_q24=(int32_t)(j*32U);
        SlProbe_Step(&p,&x,0,1,1,0,100,&theta,&id,&iq);
    }
    assert(p.follow_snapshot.reference_valid && p.follow_snapshot.reference_tick==31);
}
static void reset_and_return(void)
{
    SENSORLESS_STARTUP_CONFIG c;SENSORLESS_STARTUP_RUNTIME s;
    SensorlessStartup_DefaultConfig(&c,1.0f/16000,10,8192,600,.03125f);
    c.debug_mode=SENSORLESS_DEBUG_SMO_A3;c.hfi_enable=0;c.allow_open_loop_fallback=1;
    c.align_cycles=8000;c.current_ramp_cycles=3200;c.align_current_pu=c.startup_iq_pu=.03125f;
    c.handoff_speed_rpm=100;c.sensorless_exit_speed_rpm=20;
    c.open_loop_accel_rpm_s=c.run_accel_rpm_s=100;
    c.ramp_timeout_cycles=288000;c.total_timeout_cycles=320000;
    SensorlessStartup_Init(&s,&c);s.probe.exit_diag.first_valid=1;s.probe.reference_written[0]=~0U;
    assert(SensorlessStartup_RequestStart(&s,1,100));
    assert(!s.probe.exit_diag.first_valid && !s.probe.reference_written[0]);
    init();for(unsigned n=1;n<=400;n++)step(n,0);
    p.phase=SL_P_RETURN;p.cycles=9599;p.offset=.04f;p.follow_zero=p.follow;
    step(401,0);assert(p.stop && p.exit_diag.return_timeout_valid);
    assert(p.exit_diag.return_timeout_tick==401 && !p.exit_diag.first_valid);
    step(402,0);assert(p.exit_diag.first_reason==SL_P_STOP && p.phase==SL_P_RELEASE);
}
int main(void)
{
    follow_freeze();terminal();reference_validity();reset_and_return();
    puts("PASS exit diagnostics: frozen FOLLOW, deadline/low-current/tie/hard/external, stale/missing/wrap reference, reset, return timeout");
    return 0;
}
