#include <stdio.h>
#include "SensorlessTravel90.h"
#include "SensorlessStartup.h"
static int failures;
#define CHECK(x) do {if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);failures++;}}while(0)

/* Real startup + independent travel monitor, no motor model. */
static void CheckNormalFinish(const SENSORLESS_STARTUP_CONFIG *config)
{
    for(int direction=-1;direction<=1;direction+=2) {
        for(unsigned lead_case=0;lead_case<2;lead_case++) {
            SENSORLESS_STARTUP_RUNTIME s;
            SENSORLESS_STARTUP_INPUT in={0};
            SL_TRAVEL_MONITOR monitor;
            float path=0.0f,turns=0.0f;
            float release_samples[3]={0};
            unsigned release_seen=0;
            in.drive_enabled=in.params_valid=1;
            in.observer_locked=1;in.observer_quality=1000;
            SensorlessStartup_Init(&s,config);
            CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,100));
            SlTravel_Begin(&monitor,0,0,0,0);
            for(unsigned n=1;n<=300000;n++) {
                SensorlessStartup_Step(&s,&in);
                if(s.state==SENSORLESS_START_OPEN_LOOP_RAMP) {
                    path=(float)s.state_cycles*config->sample_time_s;
                    turns=SlTravel_Turns(path);
                    for(unsigned k=0;k<3;k++) {
                        if(path>=SL_TRAVEL_MOVE_S+(float)(k+1)*0.05f
                            && !(release_seen&(1U<<k))) {
                            release_samples[k]=s.output.iq_ref_pu*(float)direction;
                            release_seen|=1U<<k;
                        }
                    }
                    CHECK(s.output.source==ROTOR_SOURCE_OPEN_LOOP);
                    CHECK(s.output.speed_ref_rpm*(float)direction>=0.0f);
                }
                if((n%16U)==0U) {
                    /* Zero lag and a 2-degree output lead must both complete.
                     * Do not subtract an artificial offset to hide the cutoff. */
                    float bias=2.0f*(float)lead_case*fminf((float)n/8000.0f,1.0f);
                    int32_t motor=(int32_t)((float)direction*turns*131072.0f);
                    int32_t output=(int32_t)((float)direction*
                        (turns*524288.0f/101.0f+bias*524288.0f/360.0f));
                    uint32_t mr=(uint32_t)((motor%131072+131072)%131072);
                    uint32_t out=(uint32_t)((output%524288+524288)%524288);
                    uint32_t enabled=!s.debug_completed && s.state!=SENSORLESS_START_FAULT;
                    SlTravel_Sample(&monitor,n/16U,n/8U,1,mr,out,
                        s.state==SENSORLESS_START_OPEN_LOOP_RAMP,path,direction,enabled);
                    if(monitor.reason && enabled)
                        SensorlessStartup_Trip(&s,SENSORLESS_FAULT_EXTERNAL);
                }
            }
            CHECK(release_seen==7U);
            for(unsigned k=0;k<3;k++)
                CHECK(fabsf(release_samples[k]-(0.75f-0.25f*(float)k)*
                    config->startup_iq_pu)<0.0001f);
            CHECK(s.debug_completed && s.state==SENSORLESS_START_IDLE);
            CHECK(!s.output.control_active && !s.failure.fault);
            CHECK(s.output.iq_ref_pu==0.0f && s.output.id_ref_pu==0.0f);
            CHECK(monitor.done && !monitor.reason && !monitor.overflow);
            CHECK(fabsf((float)monitor.output_counts*360.0f/524288.0f)>80.0f);
            CHECK(fabsf((float)monitor.output_counts*360.0f/524288.0f)<89.0f);
        }
    }
}


/* Mode5 alignment slew, real startup state machine; no motor model. */
static void CheckAlignRamp(const SENSORLESS_STARTUP_CONFIG *config)
{
    for(int direction=-1;direction<=1;direction+=2) {
        SENSORLESS_STARTUP_RUNTIME s;
        SENSORLESS_STARTUP_INPUT in={0};
        float previous=0;
        in.drive_enabled=in.params_valid=1;
        SensorlessStartup_Init(&s,config);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,100));
        SensorlessStartup_Step(&s,&in); /* IDLE -> ALIGN, no current ownership yet */
        for(unsigned n=1;n<=config->align_cycles;n++) {
            float expected=config->align_current_pu*fminf((float)n/(float)config->current_ramp_cycles,1.0f);
            SensorlessStartup_Step(&s,&in);
            CHECK(fabsf(s.output.id_ref_pu-expected)<1e-7f);
            CHECK(s.output.id_ref_pu>=previous);
            CHECK(s.output.iq_ref_pu==0);
            CHECK(s.output.theta_e_q24==0);
            if(n<config->align_cycles)CHECK(s.state==SENSORLESS_START_ALIGN);
            previous=s.output.id_ref_pu;
        }
        CHECK(s.state==SENSORLESS_START_OPEN_LOOP_RAMP);
        CHECK(s.output.id_ref_pu==config->align_current_pu);
        CHECK(!s.failure.fault);
        SensorlessStartup_Init(&s,config);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,100));
        for(unsigned n=0;n<200;n++)SensorlessStartup_Step(&s,&in);
        CHECK(s.output.id_ref_pu<config->align_current_pu);
        SensorlessStartup_Trip(&s,SENSORLESS_FAULT_CURRENT_OVER);
        CHECK(s.state==SENSORLESS_START_FAULT && !s.output.control_active);
        CHECK(s.output.id_ref_pu==0 && s.output.iq_ref_pu==0);
        SensorlessStartup_Init(&s,config);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,100));
        for(unsigned n=0;n<200;n++)SensorlessStartup_Step(&s,&in);
        in.drive_enabled=0;SensorlessStartup_Step(&s,&in);
        CHECK(s.state==SENSORLESS_START_IDLE && !s.output.control_active);
        in.drive_enabled=1;
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,100));
        SensorlessStartup_Step(&s,&in);SensorlessStartup_Step(&s,&in);
        CHECK(fabsf(s.output.id_ref_pu-config->align_current_pu/(float)config->current_ramp_cycles)<1e-7f);
    }
    { /* A shorter valid alignment caps slew duration without extending the phase. */
        SENSORLESS_STARTUP_CONFIG c=*config;
        SENSORLESS_STARTUP_RUNTIME s;
        SENSORLESS_STARTUP_INPUT in={0};
        c.align_cycles=16;c.current_ramp_cycles=32;
        in.drive_enabled=in.params_valid=1;
        SensorlessStartup_Init(&s,&c);CHECK(SensorlessStartup_RequestStart(&s,1,100));
        SensorlessStartup_Step(&s,&in);
        for(unsigned n=1;n<=16;n++) {
            SensorlessStartup_Step(&s,&in);
            CHECK(fabsf(s.output.id_ref_pu-c.align_current_pu*(float)n/16.0f)<1e-7f);
        }
        CHECK(s.state==SENSORLESS_START_OPEN_LOOP_RAMP);
    }
}

int main(void)
{
    SL_TRAVEL_MONITOR m;
    SENSORLESS_STARTUP_CONFIG c;
    SENSORLESS_STARTUP_RUNTIME s;
    SENSORLESS_STARTUP_INPUT in={0};
    float sum=0,last=0;
    SensorlessStartup_DefaultConfig(&c,1.0f/16000.0f,10,8192,100,0.03125f);
    c.debug_mode=SENSORLESS_DEBUG_TRAVEL90;c.hfi_enable=0;c.allow_open_loop_fallback=1;
    c.align_current_pu=c.startup_iq_pu=0.03125f;
    c.handoff_speed_rpm=100;c.sensorless_exit_speed_rpm=20;
    c.open_loop_accel_rpm_s=c.run_accel_rpm_s=100;
    c.align_cycles=8000;c.current_ramp_cycles=3200;
    c.ramp_timeout_cycles=18*16000;c.total_timeout_cycles=20*16000;
    CHECK(SensorlessStartup_ValidateConfig(&c));
    c.pole_pairs=5;CHECK(!SensorlessStartup_ValidateConfig(&c));c.pole_pairs=10;
    CheckAlignRamp(&c);
    CheckNormalFinish(&c);
    in.drive_enabled=in.params_valid=1;in.observer_locked=1;in.observer_quality=1000;
    SensorlessStartup_Init(&s,&c);CHECK(SensorlessStartup_RequestStart(&s,1,100));
    for(unsigned n=0;n<280000;n++) {
        SensorlessStartup_Step(&s,&in);
        if(s.state==SENSORLESS_START_OPEN_LOOP_RAMP && s.output.control_active) {
            float now=(float)s.output.theta_e_q24/16777216.0f;
            float d=now-last;if(d<-.5f)d+=1;if(d>.5f)d-=1;
            sum+=d;last=now;
            CHECK(s.output.source==ROTOR_SOURCE_OPEN_LOOP);
            CHECK(s.output.speed_ref_rpm>=0 && s.output.speed_ref_rpm<=100.001f);
            CHECK(s.output.id_ref_pu*s.output.id_ref_pu+s.output.iq_ref_pu*s.output.iq_ref_pu<=0.031251f*0.031251f);
        }
    }
    CHECK(s.debug_completed && s.state==SENSORLESS_START_IDLE && !s.output.control_active);
    CHECK(fabsf(sum*360.0f/(10.0f*101.0f)-85.0f)<.01f);CHECK(!s.failure.fault);
    CHECK(fabsf(SlTravel_Turns(20.0f)*360.0f/101.0f-85.0f)<.0001f);
    /* Independent multi-turn observation, includes 1 s after output disable. */
    SlTravel_Begin(&m,0,0,0,0);
    for(unsigned n=1;n<=18000;n++) {
        float t=(float)n*.001f;
        uint32_t motor=(uint32_t)(SlTravel_Turns(t)*131072.0f)%131072U;
        uint32_t output=(uint32_t)(SlTravel_Turns(t)*524288.0f/101.0f);
        uint32_t enabled=(t<=SL_TRAVEL_MOVE_S+SL_TRAVEL_RELEASE_S);
        SlTravel_Sample(&m,n,n*2,1,motor,output,enabled,t,1,enabled);
    }
    CHECK(m.done && !m.reason && !m.overflow && m.count>300);
    CHECK(fabsf((float)m.motor_counts-85.0f*101.0f*131072.0f/360.0f)<5.0f);
    CHECK(fabsf((float)m.output_counts*360.0f/524288.0f-85.0f)<.001f);
    SlTravel_Begin(&m,0,0,0,0);
    SlTravel_Sample(&m,11,0,1,0,0,0,0,1,1);CHECK(m.reason==1);
    SlTravel_Sample(&m,12,2,1,0,140000,0,0,1,1);CHECK(m.reason==1);
    SlTravel_Begin(&m,0,0,0,0);
    SlTravel_Sample(&m,1,2,1,0,131072,0,0,1,1);CHECK(m.reason==2);
    SlTravel_Begin(&m,0,0,0,0);
    for(unsigned n=1;n<1000;n++)SlTravel_Sample(&m,n,n*2,1,0,0,1,(float)n*.001f,1,1);
    CHECK(m.reason==3);
    SlTravel_Begin(&m,0,0,0,0);
    SlTravel_Sample(&m,1,2,1,0,524288-1000,0,0,1,1);CHECK(m.reason==4);
    SlTravel_Begin(&m,0,0,0,0);
    SlTravel_Sample(&m,20000,2,1,0,0,0,0,1,1);CHECK(m.reason==5);
    printf("travel90 trajectory, multi-turn, stale/boundary/follow/reverse/timeout: %d failures; NO MOTOR\n",failures);
    return failures?1:0;
}
