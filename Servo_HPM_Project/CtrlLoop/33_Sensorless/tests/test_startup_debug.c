#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "SmoPll.h"
#include "SourceFoc.h"
#include "SensorlessStartup.h"
static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); failures++; } } while(0)
static SENSORLESS_STARTUP_CONFIG cfg(void) {
    SENSORLESS_STARTUP_CONFIG c;
    SensorlessStartup_DefaultConfig(&c,0.001f,4,3000,3000,0.2f);
    c.hfi_enable=0; c.allow_open_loop_fallback=1;
    c.align_cycles=3; c.current_ramp_cycles=4;
    c.handoff_speed_rpm=60; c.sensorless_exit_speed_rpm=20;
    c.open_loop_accel_rpm_s=60000; c.ramp_timeout_cycles=20;
    c.lock_timeout_cycles=20; c.lock_dwell_cycles=3;
    c.blend_cycles=5; c.iq_hold_cycles=3; c.if_hold_cycles=7;
    c.total_timeout_cycles=100; c.run_test_cycles=10;
    return c;
}
static void tick(SENSORLESS_STARTUP_RUNTIME *s,SENSORLESS_STARTUP_INPUT *in) {
    in->observer_theta_q24=s->output.theta_e_q24;
    in->observer_speed_rpm=(int32_t)s->open_loop_speed_rpm;
    SensorlessStartup_Step(s,in);
}
static void startup_tests(void) {
    SENSORLESS_STARTUP_CONFIG c=cfg();
    SENSORLESS_STARTUP_RUNTIME s;
    SENSORLESS_STARTUP_INPUT in={0};
    unsigned i;
    in.drive_enabled=in.params_valid=1; in.observer_locked=1; in.observer_quality=1000;
    c.debug_mode=SENSORLESS_DEBUG_ALIGN;
    SensorlessStartup_Init(&s,&c); CHECK(SensorlessStartup_RequestStart(&s,1,500));
    for(i=0;i<8;i++) tick(&s,&in);
    CHECK(s.state==SENSORLESS_START_IDLE && s.debug_completed && !s.output.control_active);
    c.debug_mode=SENSORLESS_DEBUG_IF;
    SensorlessStartup_Init(&s,&c); CHECK(SensorlessStartup_RequestStart(&s,1,500));
    for(i=0;i<30;i++) {
        tick(&s,&in);
        CHECK(s.state!=SENSORLESS_START_BLEND && s.state!=SENSORLESS_START_RUN);
    }
    CHECK(s.state==SENSORLESS_START_IDLE && s.debug_completed && !s.output.control_active);
    c.debug_mode=SENSORLESS_DEBUG_FULL;
    SensorlessStartup_Init(&s,&c); CHECK(SensorlessStartup_RequestStart(&s,1,500));
    for(i=0;i<40 && s.state!=SENSORLESS_START_BLEND;i++) tick(&s,&in);
    CHECK(s.state==SENSORLESS_START_BLEND);
    tick(&s,&in); in.observer_locked=0; tick(&s,&in);
    CHECK(s.state==SENSORLESS_START_FAULT);
    CHECK(s.failure.fault==SENSORLESS_FAULT_BLEND_FAILED);
    CHECK(s.failure.state==SENSORLESS_START_BLEND && s.failure.blend>0);
    CHECK(!s.output.control_active && s.output.iq_ref_pu==0 && s.output.fault_request);
    SensorlessStartup_Trip(&s,SENSORLESS_FAULT_OBSERVER_NUMERIC);
    CHECK(s.failure.fault==SENSORLESS_FAULT_BLEND_FAILED);
    SensorlessStartup_ClearFault(&s);
    CHECK(SensorlessStartup_RequestStart(&s,-1,500));
    CHECK(s.total_start_cycles==0 && s.blend_fraction==0 && s.lock_good_cycles==0 && s.failure.fault==0);
    in.observer_locked=1;
    for(i=0;i<70 && s.state!=SENSORLESS_START_RUN;i++) tick(&s,&in);
    CHECK(s.state==SENSORLESS_START_RUN && s.speed_pi_pending);
    { float iq=s.output.iq_ref_pu; in.observer_speed_rpm=-100;
      SensorlessStartup_Step(&s,&in);
      CHECK(fabsf(s.output.iq_ref_pu-iq)<1e-7f && !s.speed_pi_pending); }
    /* Stage-specific failures, and total timeout covering every pre-RUN phase. */
    for(i=0;i<5;i++) {
        SENSORLESS_START_STATE stage[]={SENSORLESS_START_ALIGN,SENSORLESS_START_OPEN_LOOP_RAMP,
            SENSORLESS_START_WAIT_LOCK,SENSORLESS_START_BLEND,SENSORLESS_START_IQ_HOLD};
        SensorlessStartup_Init(&s,&c); s.state=stage[i]; s.start_requested=1;
        s.total_start_cycles=c.total_timeout_cycles-1;
        SensorlessStartup_Step(&s,&in);
        CHECK(s.failure.fault==SENSORLESS_FAULT_START_TIMEOUT && !s.output.control_active);
    }
    SensorlessStartup_Init(&s,&c); s.state=SENSORLESS_START_OPEN_LOOP_RAMP;
    s.start_requested=1; s.state_cycles=c.ramp_timeout_cycles-1;
    SensorlessStartup_Step(&s,&in); CHECK(s.failure.fault==SENSORLESS_FAULT_RAMP_TIMEOUT);
    SensorlessStartup_Init(&s,&c); s.state=SENSORLESS_START_WAIT_LOCK;
    s.start_requested=1; s.state_cycles=c.lock_timeout_cycles-1;
    in.observer_locked=0;
    SensorlessStartup_Step(&s,&in); CHECK(s.failure.fault==SENSORLESS_FAULT_LOCK_TIMEOUT);
    printf("startup modes, timeouts, failure latch and PI handover checked\n");
}
static void d_axis_tests(void) {
    SENSORLESS_STARTUP_CONFIG c=cfg();
    SENSORLESS_STARTUP_RUNTIME s;
    SENSORLESS_STARTUP_INPUT in={0};
    c.sample_time_s=1.0f/16000.0f; c.pole_pairs=10;
    c.debug_mode=SENSORLESS_DEBUG_D_AXIS;
    c.align_current_pu=0.4f/25.6f; c.startup_iq_pu=c.align_current_pu;
    c.iq_limit_pu=0.5f/25.6f;
    c.align_cycles=8000; c.current_ramp_cycles=1600; c.if_hold_cycles=1600;
    c.handoff_speed_rpm=30; c.sensorless_exit_speed_rpm=20;
    c.open_loop_accel_rpm_s=100; c.ramp_timeout_cycles=8000;
    c.total_timeout_cycles=19200;
    in.drive_enabled=in.params_valid=1;
    in.observer_locked=1; in.observer_quality=1000;
    CHECK(SensorlessStartup_ValidateConfig(&c));
    for(int direction=-1;direction<=1;direction+=2) {
        unsigned n, rotating=0; int32_t prev=0;
        SensorlessStartup_Init(&s,&c);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)direction,30));
        for(n=0;n<19201;n++) {
            tick(&s,&in);
            CHECK(s.state!=SENSORLESS_START_BLEND && s.state!=SENSORLESS_START_RUN);
            if(s.output.control_active) {
                CHECK(fabsf(s.output.id_ref_pu-c.align_current_pu)<1e-7f);
                CHECK(s.output.iq_ref_pu==0.0f);
                CHECK(s.output.source==ROTOR_SOURCE_OPEN_LOOP);
                if(s.state==SENSORLESS_START_OPEN_LOOP_RAMP || s.state==SENSORLESS_START_WAIT_LOCK) {
                    int32_t delta=RotorFeedback_AngleErrorQ24(s.output.theta_e_q24,prev);
                    CHECK(direction*delta>=0 && direction*delta<6000);
                    CHECK(fabsf(s.output.speed_ref_rpm)<=30.01f);
                    rotating++;
                }
                prev=s.output.theta_e_q24;
            }
            if(s.debug_completed) break;
        }
        CHECK(n>14000 && n<14500 && rotating>6000);
        CHECK(s.debug_completed && !s.output.control_active);
        CHECK(s.output.id_ref_pu==0 && s.output.iq_ref_pu==0);
        CHECK(s.failure.fault==0);
    }
    c.hfi_enable=1; CHECK(!SensorlessStartup_ValidateConfig(&c)); c.hfi_enable=0;
    c.allow_open_loop_fallback=0; CHECK(!SensorlessStartup_ValidateConfig(&c));
    c.allow_open_loop_fallback=1;
    for(unsigned k=0;k<2;k++) {
        SensorlessStartup_Init(&s,&c);
        s.state=SENSORLESS_START_OPEN_LOOP_RAMP; s.start_requested=1;
        if(k==0) s.total_start_cycles=c.total_timeout_cycles-1;
        else s.state_cycles=c.ramp_timeout_cycles-1;
        tick(&s,&in);
        CHECK(s.state==SENSORLESS_START_FAULT && !s.output.control_active);
        CHECK(s.failure.fault==(k==0?SENSORLESS_FAULT_START_TIMEOUT:SENSORLESS_FAULT_RAMP_TIMEOUT));
        CHECK(s.output.id_ref_pu==0 && s.output.iq_ref_pu==0);
    }
    printf("d-axis real 16kHz 900ms trajectory, both directions and timeouts checked\\n");
}
static void scan_tests(void) {
    SENSORLESS_STARTUP_CONFIG c=cfg();
    SENSORLESS_STARTUP_RUNTIME s;
    SENSORLESS_STARTUP_INPUT in={0};
    c.sample_time_s=1.0f/16000.0f; c.pole_pairs=10;
    c.debug_mode=SENSORLESS_DEBUG_D_AXIS_SCAN;
    c.align_angle_turns=0.9f;
    c.align_current_pu=0.8f/25.6f; c.iq_limit_pu=c.align_current_pu;
    c.startup_iq_pu=c.align_current_pu;
    c.if_hold_cycles=8000;
    c.align_cycles=8000; c.ramp_timeout_cycles=17600;
    c.total_timeout_cycles=28800; c.open_loop_accel_rpm_s=100;
    CHECK(SensorlessStartup_ValidateConfig(&c));
    in.drive_enabled=in.params_valid=in.observer_locked=1; in.observer_quality=1000;
    for(int dir=-1;dir<=1;dir+=2) {
        int32_t prev=(int32_t)(c.align_angle_turns*16777216.0f);
        float last_speed=0;
        unsigned samples=0;
        SensorlessStartup_Init(&s,&c);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)dir,30));
        for(unsigned n=0;n<28801;n++) {
            tick(&s,&in);
            CHECK(s.state!=SENSORLESS_START_BLEND && s.state!=SENSORLESS_START_RUN);
            if(s.output.control_active) {
                CHECK(s.output.source==ROTOR_SOURCE_OPEN_LOOP);
                CHECK(fabsf(s.output.id_ref_pu-c.align_current_pu)<1e-7f);
                CHECK(s.output.iq_ref_pu==0);
                int32_t delta=RotorFeedback_AngleErrorQ24(s.output.theta_e_q24,prev);
                CHECK(abs(delta)<1700);
                CHECK(fabsf(s.output.speed_ref_rpm)<=9.001f);
                CHECK(fabsf(s.output.speed_ref_rpm-last_speed)<=0.0046f);
                if(s.state==SENSORLESS_START_OPEN_LOOP_RAMP && s.state_cycles>0) {
                    unsigned k=s.state_cycles;
                    CHECK((k<=8000?dir:-dir)*delta>=-2);
                    if(k==8000) CHECK(abs(RotorFeedback_AngleErrorQ24(
                        s.output.theta_e_q24,prev))<5 && fabsf(s.output.speed_ref_rpm)<1e-6f);
                    samples++;
                }
                last_speed=s.output.speed_ref_rpm; prev=s.output.theta_e_q24;
            }
            if(s.debug_completed) break;
        }
        CHECK(samples==16000 && s.debug_completed && !s.output.control_active);
        CHECK(abs(RotorFeedback_AngleErrorQ24(prev,(int32_t)(c.align_angle_turns*16777216.0f)))<4);
        CHECK(s.failure.fault==0 && s.output.id_ref_pu==0 && s.output.iq_ref_pu==0);
        CHECK(SensorlessStartup_RequestStart(&s,(int16_t)dir,30));
        CHECK(!s.debug_completed && s.total_start_cycles==0);
    }
    for(unsigned k=0;k<4;k++) {
        SensorlessStartup_Init(&s,&c);
        CHECK(SensorlessStartup_RequestStart(&s,1,30));
        for(unsigned n=0;n<8500;n++) tick(&s,&in);
        if(k==0) SensorlessStartup_RequestStop(&s);
        if(k==1) { s.total_start_cycles=c.total_timeout_cycles-1; tick(&s,&in); }
        if(k==2) { s.state_cycles=c.ramp_timeout_cycles-1; tick(&s,&in); }
        if(k==3) SensorlessStartup_Trip(&s,SENSORLESS_FAULT_CURRENT_OVER);
        CHECK(!s.output.control_active && s.output.id_ref_pu==0 && s.output.iq_ref_pu==0);
        if(k==1) CHECK(s.failure.fault==SENSORLESS_FAULT_START_TIMEOUT);
        if(k==2) CHECK(s.failure.fault==SENSORLESS_FAULT_RAMP_TIMEOUT);
        if(k==3) CHECK(s.failure.fault==SENSORLESS_FAULT_CURRENT_OVER);
    }
    c.open_loop_accel_rpm_s=70; CHECK(!SensorlessStartup_ValidateConfig(&c));
    c.open_loop_accel_rpm_s=100; c.ramp_timeout_cycles=16001;
    CHECK(!SensorlessStartup_ValidateConfig(&c));
    c.ramp_timeout_cycles=17600; c.total_timeout_cycles=24002;
    CHECK(!SensorlessStartup_ValidateConfig(&c));
    c.total_timeout_cycles=80000; c.ramp_timeout_cycles=70000;
    for(unsigned leg=8000;leg<=32000;leg*=2) {
        c.if_hold_cycles=leg;
        CHECK(SensorlessStartup_ValidateConfig(&c));
        for(int dir=-1;dir<=1;dir+=2) {
            float peak=0; unsigned count=0;
            SensorlessStartup_Init(&s,&c);
            CHECK(SensorlessStartup_RequestStart(&s,(int16_t)dir,30));
            for(unsigned n=0;n<c.total_timeout_cycles;n++) {
                tick(&s,&in);
                CHECK(s.state!=SENSORLESS_START_BLEND && s.state!=SENSORLESS_START_RUN);
                if(s.state==SENSORLESS_START_OPEN_LOOP_RAMP && s.state_cycles>0) {
                    count++;
                    if(fabsf(s.output.speed_ref_rpm)>peak) peak=fabsf(s.output.speed_ref_rpm);
                    CHECK(s.output.iq_ref_pu==0 && s.output.control_active);
                }
                if(s.debug_completed) break;
            }
            CHECK(count==2*leg && s.debug_completed && !s.output.control_active);
            CHECK(fabsf(peak-72000.0f/(float)leg)<0.001f);
            CHECK(s.failure.fault==0);
        }
    }
    c.if_hold_cycles=7999; CHECK(!SensorlessStartup_ValidateConfig(&c));
    c.if_hold_cycles=32001; CHECK(!SensorlessStartup_ValidateConfig(&c));
    printf("bounded configurable scan 500/1000/2000ms and protection checked\n");
}
static void phase_tests(void) {
    SENSORLESS_MOTOR_PROFILE p;
    SOURCE_FOC_RECIPE r={1,1,1,0.003f,0.004f,10,100,2,1,3000,200,4,1};
    SMO_PLL_PARAMS cfg;
    SMO_PLL_STATE s;
    SMO_PLL_INPUT in={0};
    unsigned i;
    CHECK(SourceFoc_BuildProfile(&r,16000,3000,&p));
    MotorParamProfile_ApplyToSmo(&p,&cfg);
    cfg.pll_kp=cfg.pll_ki=0; cfg.emf_lpf_alpha=1; cfg.emf_delay_comp_gain=0;
    cfg.lock_cycles=2; cfg.unlock_cycles=2;
    in.expected_direction=1;
    /* Feed the switching model to obtain a known EMF vector with frozen PLL. */
    for(i=0;i<4;i++) {
        float angle=(i==0)?0.5f:(i==1?0.01f:0.0f);
        SmoPll_Reset(&s,(i==1)?0.99f:0.0f);
        for(unsigned j=0;j<4;j++) {
            float amp=(i>=2)?0.0f:0.1f;
            s.current_hat_alpha_pu=-amp*sinf(angle*6.28318530718f)*cfg.boundary_pu/cfg.switching_gain;
            s.current_hat_beta_pu=amp*cosf(angle*6.28318530718f)*cfg.boundary_pu/cfg.switching_gain;
            SmoPll_Step(&s,&cfg,&in);
        }
        if(i==0) CHECK(!s.locked && s.quality==0 && fabsf(s.phase_error_rad)>3.0f && s.lock_reason==3);
        if(i==1) CHECK(s.locked && fabsf(s.phase_error_rad-0.02f*6.28318530718f)<0.001f);
        if(i>=2) CHECK(!s.locked && s.lock_reason==2);
    }
    printf("180 degree, wraparound and weak EMF lock rejection checked\n");
}
static void fault_tests(void) {
    SOURCE_FOC_RECIPE r={1,1,1,0.003f,0.004f,10,100,2,1,3000,200,4,1};
    SENSORLESS_MOTOR_PROFILE p; SOURCE_FOC_CURRENT s;
    CHECK(SourceFoc_BuildProfile(&r,16000,3000,&p));
    CHECK(SourceFoc_Configure(&s,&p,200));
    CHECK(!SourceFoc_Step(&s,0,0,0.3f,0,0.5f) && s.failure_reason==SOURCE_FOC_OVER_CURRENT);
    CHECK(!SourceFoc_Step(&s,NAN,0,0,0,0.5f) && s.failure_reason==SOURCE_FOC_INPUT_INVALID);
    CHECK(!SourceFoc_Step(&s,0,0,0,0,NAN) && s.failure_reason==SOURCE_FOC_LIMIT_INVALID);
    s.integral_d=NAN;
    CHECK(!SourceFoc_Step(&s,0,0,0,0,0.5f) && s.failure_reason==SOURCE_FOC_CALC_INVALID);
    s.configured=0;
    CHECK(!SourceFoc_Step(&s,0,0,0,0,0.5f) && s.failure_reason==SOURCE_FOC_CONFIG_INVALID);
}
int main(void) {
    phase_tests(); startup_tests(); d_axis_tests(); scan_tests(); fault_tests();
    printf("startup debug P0: %d failures (software only, no motor)\n",failures);
    return failures?1:0;
}
