#include <stdio.h>
#include "SensorlessSmoTrace.h"
static SL_SMO_TRACE trace;
static int failures;
#define CHECK(x) do {if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);failures++;}}while(0)
int main(void)
{
    SMO_PLL_PARAMS p={0};SMO_PLL_INPUT in={0};SMO_PLL_STATE s={0};
    SL_SMO_TRACE_CONTEXT c={0};
    p.sample_time_s=1.0f/16000;in.voltage_alpha_pu=.125f;
    c.state=SENSORLESS_START_OPEN_LOOP_RAMP;c.direction=-1;c.chain_valid=1;
    SlSmoTrace_Begin(&trace,&p,1,10);
    for(unsigned n=0;n<6010;n++) {
        c.tick=n+100;c.stage_cycles=n;c.chain_tick=c.tick-1;
        s.locked=(n>=10 && n<5910)?1:0;
        s.current_hat_alpha_pu=(float)n;
        SlSmoTrace_Sample(&trace,&in,&s,&c);
    }
    CHECK(trace.header.count==160 && trace.header.frozen && trace.header.active);
    CHECK(trace.header.total_samples==6010 && trace.header.locked_samples==5900);
    CHECK(trace.header.longest_locked_run==5900 && trace.header.locked_run==0);
    CHECK(trace.header.first_tick==110 && trace.header.last_tick==5198);
    CHECK(trace.samples[0].voltage_alpha==.125f && trace.samples[0].context.direction==-1);
    CHECK(trace.samples[159].current_hat_alpha==5098);
    for(unsigned n=1;n<160;n++)CHECK(trace.samples[n].context.tick==trace.samples[n-1].context.tick+32);
    SlSmoTrace_Finish(&trace,1);SlSmoTrace_Sample(&trace,&in,&s,&c);
    CHECK(!trace.header.active && trace.header.stop_reason==1 && !(trace.header.sequence&1));
    CHECK(trace.header.total_samples==6010);
    SlSmoTrace_Begin(&trace,&p,1,0);
    c.stage_cycles=0;SlSmoTrace_Sample(&trace,&in,&s,&c);
    SlSmoTrace_Finish(&trace,2);SlSmoTrace_Finish(&trace,3);
    CHECK(trace.header.count==1 && trace.header.stop_reason==2 && trace.header.record_id==2);
    SlSmoTrace_Begin(&trace,&p,1,10);
    trace.header.align_trigger_cycles=5952;
    c.state=SENSORLESS_START_ALIGN;c.encoder_valid=1;c.encoder_raw=131070;
    c.id_ref=.03125f;c.control_theta_q24=0;
    for(unsigned n=0;n<8000;n++) {
        c.tick=n;c.stage_cycles=n;
        SlSmoTrace_Sample(&trace,&in,&s,&c);
    }
    CHECK(trace.header.align_count==64 && trace.header.count==0);
    CHECK(trace.align_samples[0].context.tick==5952);
    CHECK(trace.align_samples[63].context.tick==7968);
    CHECK(trace.align_samples[63].context.encoder_raw==131070);
    CHECK(trace.align_samples[63].context.id_ref==.03125f);
    c.state=SENSORLESS_START_OPEN_LOOP_RAMP;
    for(unsigned n=0;n<6010;n++) {
        c.tick=n+8000;c.stage_cycles=n;
        SlSmoTrace_Sample(&trace,&in,&s,&c);
    }
    CHECK(trace.header.count==160 && trace.header.align_count==64);
    CHECK(trace.header.total_samples==14010);
    SlSmoTrace_Finish(&trace,1);
    SlSmoTrace_Begin(&trace,&p,0,0);SlSmoTrace_Sample(&trace,&in,&s,&c);
    CHECK(!trace.header.active && trace.header.count==0 && trace.header.total_samples==0);
    printf("SMO trace continuity, bounded window, whole-run lock and freeze: %d failures\n",failures);
    return failures?1:0;
}
