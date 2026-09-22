/* Real transport + real control modules; simulated board, no motor model. */
#define main legacy_debug_chain_main
#include "test_debug_chain.c"
#undef main
#include "SensorlessCanopen.h"
#include "SensorlessEncoderMonitor.h"
#include "StartupTiming.h"
#include "SensorlessTravel90.h"
volatile STARTUP_TIMING g_startup_timing;
extern volatile SL_TRAVEL_MONITOR g_sl_travel;
volatile SL_ENCODER_MONITOR g_sl_encoder;
#include <string.h>
static uint16_t platform_ready=1;
uint16_t SensorlessCanopen_PlatformReady(void) { return platform_ready && StateMachine.RegulFlg && !(FaultP.FaultStatus&FP_ERR); }
uint16_t SensorlessJlink_PlatformReady(void) { return SensorlessCanopen_PlatformReady(); }
uint16_t SensorlessJlink_PlatformOff(void) { return !StateMachine.RegulFlg; }
static void debug_command(uint32_t command,uint32_t key)
{
    g_sensorless_jlink.command=command;
    g_sensorless_jlink.arm_key=key;
    ++g_sensorless_jlink.request_seq;
    SensorlessCanopen_Service1ms();
}
static uint32_t seq;
static uint32_t read_wire(uint16_t idx,uint8_t sub) { uint32_t v=0;CHECK(SensorlessCanopen_Read(idx,sub,&v));return v; }
static uint32_t commit(uint32_t cmd) {
    CHECK(SensorlessCanopen_Write(0x2F00,3,cmd));
    CHECK(SensorlessCanopen_Write(0x2F00,2,++seq));
    CHECK(!SensorlessCanopen_Write(0x2F00,3,12));
    SensorlessCanopen_Service1ms();
    CHECK(read_wire(0x2F00,7)==seq);
    CHECK(!SensorlessCanopen_Write(0x2F00,2,seq));
    printf("wire cmd=%lu result=%lu state=%u ready=%u fault=%lu heartbeat=%lu\n",(unsigned long)cmd,(unsigned long)read_wire(0x2F00,8),(unsigned)g_sensorless_shadow.startup.state,(unsigned)g_source_foc_ready,(unsigned long)FaultP.FaultStatus,(unsigned long)read_wire(0x2F00,9));
    return read_wire(0x2F00,8);
}
int main(void) {
    /* Reuse baseline end-to-end assertions and calibrated fixture. */
    SensorlessCanopen_Init();
    CHECK(g_sl_sampling.abi==1 && g_sl_sampling.sequence==0);
    SensorlessSampling_Record(0,INT16_MIN,INT16_MIN,0,0);
    SensorlessSampling_Record(2,40,-40,0,0);
    SensorlessSampling_Record(2,42,-41,41,-40);
    SensorlessSampling_Record(3,41,-40,41,-40);
    CHECK(g_sl_sampling.phase[0].rail_samples==1);
    CHECK(g_sl_sampling.phase[2].samples==2 && g_sl_sampling.phase[2].rail_samples==0);
    CHECK(g_sl_sampling.phase[2].min_u==40 && g_sl_sampling.phase[2].max_u==42);
    CHECK(g_sl_sampling.phase[2].min_v==-41 && g_sl_sampling.phase[2].max_v==-40);
    CHECK(g_sl_sampling.phase[3].zero_u==41 && g_sl_sampling.phase[3].zero_v==-40);
    CHECK(g_sl_sampling.sequence==8 && g_sl_sampling.last_phase==3);
    SensorlessSampling_Record(4,0,0,0,0);
    CHECK(g_sl_sampling.sequence==8);
    SensorlessCanopen_Init();
    CHECK(g_sl_sampling.phase[0].samples==0 && g_sl_sampling.phase[2].samples==0);
    CHECK(SensorlessCanopen_Write(0x2F00,9,1));
    g_sl_encoder.initialized=1U; g_sl_encoder.valid=1U;
    g_sl_encoder.sequence=2U; g_sl_encoder.tick_ms=123U;
    g_sl_encoder.last_good_ms=123U;
    g_sl_encoder.motor_raw=131000U; g_sl_encoder.output_raw=524000U;
    CHECK(legacy_debug_chain_main()==0);
    CHECK(g_sensorless_trace.abi==3U);
    CHECK(sizeof(SENSORLESS_TRACE_SAMPLE)==264U);
    CHECK(g_sensorless_trace.count>0U);
    for (uint32_t n=0;n<g_sensorless_trace.count;n++) {
        CHECK(g_sensorless_trace.samples[n].encoder_valid==1U);
        CHECK(g_sensorless_trace.samples[n].encoder_motor_raw==131000U);
        CHECK(g_sensorless_trace.samples[n].encoder_output_raw==524000U);
        CHECK(g_sensorless_trace.samples[n].encoder_tick_ms==123U);
    }
    g_sl_encoder.sequence=3U; /* publisher preempted: must never spin */

    StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
    SensorlessShadow_Reset();SensorlessProduction_Service1ms();
    SensorlessCanopen_Init();
    CHECK(!SensorlessCanopen_Write(0x2F00,1,0));
    CHECK(!SensorlessCanopen_Write(0x2F00,2,1));
    CHECK(SensorlessCanopen_Write(0x2F00,1,1));
    CHECK(read_wire(0x2F00,1)==1);
    CHECK(!SensorlessCanopen_Write(0x2F03,1,99));
    CHECK(!SensorlessCanopen_Write(0x2F01,12,65536));
    /* Copy explicit recipe field values through scalar wire format. */
    for(uint8_t i=1;i<=SensorlessCanopen_FieldCount(0x2F01);i++)
        CHECK(SensorlessCanopen_Write(0x2F01,i,read_wire(0x2F03,i)));
    CHECK(commit(12)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(read_wire(0x2F03,1)==321);
    CHECK(SensorlessCanopen_Write(0x2F02,1,512));
    CHECK(SensorlessCanopen_Write(0x2F02,61,SENSORLESS_DEBUG_D_AXIS));
    CHECK(SensorlessCanopen_Write(0x2F02,62,100));
    CHECK(SensorlessCanopen_Write(0x2F02,63,1000));
    CHECK(SensorlessCanopen_Write(0x2F02,64,200));
    CHECK(SensorlessCanopen_Write(0x2F02,65,3000));
    CHECK(SensorlessCanopen_Write(0x2F02,66,8000));
    CHECK(SensorlessCanopen_Write(0x2F02,67,2000));
    CHECK(commit(2)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_production.effective_startup.if_hold_cycles==16000);
    CHECK(g_sensorless_production.effective_startup.debug_mode==SENSORLESS_DEBUG_D_AXIS);
    CHECK(SensorlessCanopen_Write(0x2F02,61,SENSORLESS_DEBUG_D_AXIS_SCAN));
    CHECK(commit(2)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_production.effective_startup.debug_mode==SENSORLESS_DEBUG_D_AXIS_SCAN);
    uint32_t params=read_wire(0x2F00,15);
    SensorlessCanopen_Service1ms();
    CHECK(read_wire(0x2F00,15)==params);
    CHECK(SensorlessCanopen_Write(0x2F01,3,0x7FC00000)); /* NaN Rs rejected atomically */
    CHECK(commit(12)==SENSORLESS_PROD_RESULT_INVALID_PROFILE);
    CHECK(read_wire(0x2F03,1)==321 && read_wire(0x2F00,15)==params);
    StateMachine.RegulFlg=1;
    CHECK(!SensorlessCanopen_Write(0x2F01,1,99));
    CHECK(SensorlessCanopen_Write(0x2F00,4,SENSORLESS_PRODUCTION_ARM_KEY));
    CHECK(SensorlessCanopen_Write(0x2F00,6,(uint32_t)g_sensorless_shadow.startup.config.handoff_speed_rpm));
    CHECK(commit(3)==SENSORLESS_PROD_RESULT_SERVO_NOT_READY);
    CHECK(SensorlessCanopen_Write(0x2F00,9,1));
    CHECK(!SensorlessCanopen_Write(0x2F00,9,1));
    CHECK(SensorlessCanopen_Write(0x2F00,4,SENSORLESS_PRODUCTION_ARM_KEY));
    platform_ready=0;
    CHECK(commit(3)==SENSORLESS_PROD_RESULT_SERVO_NOT_READY);
    platform_ready=1;
    CHECK(SensorlessCanopen_Write(0x2F00,4,SENSORLESS_PRODUCTION_ARM_KEY));
    CHECK(commit(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(read_wire(0x2F00,4)==0);
    /* Disable between acceptance and first PWM must prevent even one output. */
    platform_ready=0;SensorlessCanopen_PwmGuard();tick();
    CHECK(!g_sensorless_diag.control_override_active);
    CHECK(g_sensorless_trace.frozen);
    CHECK(g_sl_can_abort_reason==2);
    CHECK(g_sensorless_trace.count>0U);
    for (uint32_t n=0;n<g_sensorless_trace.count;n++)
        CHECK(g_sensorless_trace.samples[n].encoder_valid==0U);
    uint8_t dump[sizeof(SENSORLESS_TRACE)];
    for(unsigned i=0;i<sizeof(dump);i+=7){
        unsigned len=sizeof(dump)-i;if(len>7)len=7;
        CHECK(SensorlessCanopen_TraceRead(i,dump+i,len));
    }
    CHECK(!memcmp(dump,(const void *)&g_sensorless_trace,sizeof(dump)));
    CHECK(!SensorlessCanopen_TraceRead(sizeof(dump),dump,1));
    StateMachine.RegulFlg=0;
    CHECK(commit(6)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(!SensorlessCanopen_TraceRead(7,dump,7));
    FaultP.FaultStatus=0;StateMachine.RegulFlg=1;platform_ready=1;
    CHECK(SensorlessCanopen_Write(0x2F00,4,SENSORLESS_PRODUCTION_ARM_KEY));
    CHECK(commit(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
    SensorlessCanopen_PwmGuard();tick();SensorlessCanopen_PwmGuard();tick();CHECK(g_sensorless_diag.control_override_active);
    for(unsigned i=0;i<SL_CAN_WATCHDOG_MS;i++)SensorlessCanopen_Tick1ms();
    SensorlessCanopen_PwmGuard();tick();
    CHECK(g_sl_can_abort_reason==1 && g_sensorless_trace.frozen);
    CHECK(!g_sensorless_diag.control_override_active);
    CHECK(commit(5)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(read_wire(0x2F06,7)==0);
    /* The same real control chain, now without CAN heartbeat/NMT. */
    StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
    SensorlessShadow_Reset();SensorlessCanopen_Init();
    g_sensorless_jlink.duration_ms=30;
    g_sensorless_jlink.direction=1;
    g_sensorless_jlink.target_speed_rpm=(int32_t)g_sensorless_shadow.startup.config.handoff_speed_rpm;
    CHECK(!SensorlessJlink_OwnsControl());
    debug_command(1,0);
    CHECK(g_sensorless_jlink.result==1 && !SensorlessJlink_EnableRequested());
    g_sensorless_jlink.duration_ms=SL_JLINK_MAX_MS+1;
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==3);
    g_sensorless_jlink.duration_ms=30;
    StateMachine.RegulFlg=1;
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==4);
    StateMachine.RegulFlg=0;
    debug_command(1,SL_JLINK_KEY);
    CHECK(g_sensorless_jlink.result==0 && SensorlessJlink_EnableRequested());
    CHECK(g_sensorless_shadow.startup.state==SENSORLESS_START_IDLE);
    CHECK(!SensorlessCanopen_Write(0x2F00,9,99));
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==2);
    /* Normal platform readiness, never force-enable inside the transport. */
    StateMachine.RegulFlg=1;
    SensorlessCanopen_Service1ms();
    CHECK(g_sensorless_production.command_result==SENSORLESS_PROD_RESULT_ACCEPTED);
    SensorlessCanopen_PwmGuard();tick();SensorlessCanopen_PwmGuard();tick();
    CHECK(g_sensorless_diag.control_override_active);
    g_sensorless_jlink.duration_ms=10000; /* cannot extend a live lease */
    for(unsigned i=0;i<30;i++)SensorlessCanopen_Tick1ms();
    SensorlessCanopen_PwmGuard();tick();
    CHECK(!SensorlessJlink_EnableRequested() && SensorlessJlink_OwnsControl());
    CHECK(g_sensorless_jlink.exit_reason==1);
    CHECK(!g_sensorless_diag.control_override_active && g_sensorless_trace.frozen);
    debug_command(2,0);CHECK(g_sensorless_jlink.exit_reason==1);
    debug_command(3,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==4);
    StateMachine.RegulFlg=0;
    debug_command(3,SL_JLINK_KEY);
    CHECK(!SensorlessJlink_OwnsControl() && !SensorlessCanopen_MotionAllowed());
    /* Repeated run after explicit reset; abort before ready must not start. */
    FaultP.FaultStatus=0;SensorlessShadow_Reset();
    g_sensorless_jlink.duration_ms=30;
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==0);
    debug_command(2,0);
    StateMachine.RegulFlg=1;SensorlessCanopen_Service1ms();
    CHECK(!SensorlessJlink_EnableRequested() && !g_sensorless_shadow.startup.start_requested);
    CHECK(g_sensorless_jlink.exit_reason==2);
    /* New mode cannot use the old 10 s lease or enter through CAN directly. */
    StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
    SensorlessShadow_Reset();SensorlessCanopen_Init();
    SENSORLESS_STARTUP_CONFIG *tc=&g_sensorless_shadow.startup.config;
    SensorlessStartup_DefaultConfig(tc,1.0f/16000.0f,10,8192,100,0.03125f);
    tc->sensorless_exit_speed_rpm=20;
    tc->debug_mode=SENSORLESS_DEBUG_TRAVEL90;tc->pole_pairs=10;
    tc->hfi_enable=0;tc->allow_open_loop_fallback=1;
    tc->max_speed_rpm=tc->handoff_speed_rpm=100;
    tc->open_loop_accel_rpm_s=tc->run_accel_rpm_s=100;
    tc->align_cycles=8000;tc->current_ramp_cycles=3200;
    tc->ramp_timeout_cycles=18*16000;tc->total_timeout_cycles=20*16000;
    tc->align_current_pu=tc->startup_iq_pu=0.03125f;tc->iq_limit_pu=0.03125f;
    CHECK(SensorlessStartup_ValidateConfig(tc));
    CHECK(!SensorlessCanopen_MotionAllowed());
    g_sensorless_jlink.direction=1;g_sensorless_jlink.target_speed_rpm=100;
    g_sl_encoder.sequence=4;g_sl_encoder.valid=1;g_sl_encoder.initialized=1;g_sl_encoder.age_ms=0;
    g_sensorless_jlink.duration_ms=10000;
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==3);
    g_sensorless_jlink.duration_ms=20000;
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==0 && g_sl_travel.active);
    for(unsigned n=0;n<11;n++)SensorlessCanopen_Tick1ms();
    SensorlessCanopen_Service1ms();
    CHECK(g_sl_travel.reason==1 && !SensorlessJlink_EnableRequested());
    CHECK(g_sensorless_jlink.exit_reason==6);
    debug_command(1,SL_JLINK_KEY);CHECK(g_sensorless_jlink.result==2);

    /* Full mailbox/startup/trace/disable path in both directions.
     * Encoder fixture follows the commanded motor trajectory; no motor model. */
    for(int test_direction=-1;test_direction<=1;test_direction+=2) {
        StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
        SensorlessShadow_Reset();SensorlessCanopen_Init();
        tc=&g_sensorless_shadow.startup.config;
        SensorlessStartup_DefaultConfig(tc,1.0f/16000.0f,10,8192,100,0.03125f);
        tc->sensorless_exit_speed_rpm=20;
        tc->debug_mode=SENSORLESS_DEBUG_TRAVEL90;
        tc->hfi_enable=0;tc->allow_open_loop_fallback=1;
        tc->max_speed_rpm=tc->handoff_speed_rpm=100;
        tc->open_loop_accel_rpm_s=tc->run_accel_rpm_s=100;
        tc->align_cycles=8000;tc->current_ramp_cycles=3200;
        tc->ramp_timeout_cycles=18*16000;tc->total_timeout_cycles=20*16000;
        tc->align_current_pu=tc->startup_iq_pu=tc->iq_limit_pu=0.03125f;
        CHECK(SensorlessStartup_ValidateConfig(tc));
        memset((void *)&g_sl_encoder,0,sizeof(g_sl_encoder));
        g_sl_encoder.sequence=2;g_sl_encoder.initialized=g_sl_encoder.valid=1;
        g_sensorless_jlink.direction=test_direction;
        g_sensorless_jlink.target_speed_rpm=100;g_sensorless_jlink.duration_ms=20000;
        debug_command(1,SL_JLINK_KEY);
        CHECK(g_sensorless_jlink.result==0 && SensorlessJlink_EnableRequested());
        StateMachine.RegulFlg=1;SensorlessCanopen_Service1ms();
        float turns=0.0f;
        for(unsigned ms=1;ms<=18000;ms++) {
            for(unsigned cycle=0;cycle<16;cycle++) {
                SensorlessCanopen_PwmGuard();tick();
            }
            if(g_sensorless_shadow.startup.state==SENSORLESS_START_OPEN_LOOP_RAMP)
                turns=SlTravel_Turns((float)g_sensorless_shadow.startup.state_cycles*tc->sample_time_s);
            int32_t motor=(int32_t)((float)test_direction*turns*131072.0f);
            float bias=0.1f*fminf((float)ms/500.0f,1.0f);
            int32_t output=(int32_t)((float)test_direction*
                (turns*524288.0f/101.0f+bias*524288.0f/360.0f));
            g_sl_encoder.motor_raw=(uint32_t)((motor%131072+131072)%131072);
            g_sl_encoder.output_raw=(uint32_t)((output%524288+524288)%524288);
            g_sl_encoder.sequence+=2;g_sl_encoder.tick_ms=ms;g_sl_encoder.last_good_ms=ms;
            SensorlessCanopen_Tick1ms();SensorlessCanopen_Service1ms();
            if(!SensorlessJlink_EnableRequested())StateMachine.RegulFlg=0;
        }
        CHECK(g_sensorless_shadow.startup.debug_completed);
        CHECK(g_sensorless_shadow.startup.state==SENSORLESS_START_IDLE);
        CHECK(!g_sensorless_shadow.startup.failure.fault && !FaultP.FaultStatus);
        CHECK(g_sensorless_jlink.exit_reason==5 && !SensorlessJlink_EnableRequested());
        CHECK(g_sensorless_jlink.arm_key==0);
        CHECK(g_sensorless_jlink.request_seq==g_sensorless_jlink.acknowledge_seq);
        CHECK(!g_sensorless_diag.control_override_active && !StateMachine.RegulFlg);
        CHECK(g_sl_travel.done && !g_sl_travel.reason && !g_sl_travel.overflow);
        CHECK(g_sl_travel.count>300 && !g_sl_travel.invalid_samples);
        CHECK(g_sensorless_trace.frozen && !g_sensorless_trace.active);
        CHECK(g_sensorless_trace.count>80 && g_sensorless_trace.count<128);
        CHECK(fabsf((float)g_sl_travel.output_counts*360.0f/524288.0f)>84.0f);
        CHECK(fabsf((float)g_sl_travel.output_counts*360.0f/524288.0f)<86.0f);
    }
    /* A1/A2/A3 real mailbox admission and bounded encoder publication cache. */
    for(uint16_t probe_mode=SENSORLESS_DEBUG_SMO_A1;probe_mode<=SENSORLESS_DEBUG_SMO_FRAME;++probe_mode) {
    StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
    SensorlessShadow_Reset();SensorlessCanopen_Init();
    tc=&g_sensorless_shadow.startup.config;
    SensorlessStartup_DefaultConfig(tc,1.0f/16000.0f,10,8192,100,0.03125f);
    tc->debug_mode=probe_mode;tc->sensorless_exit_speed_rpm=20;
    tc->hfi_enable=0;tc->allow_open_loop_fallback=1;
    tc->max_speed_rpm=tc->handoff_speed_rpm=100;
    tc->open_loop_accel_rpm_s=tc->run_accel_rpm_s=100;
    tc->align_cycles=8000;tc->current_ramp_cycles=3200;
    tc->ramp_timeout_cycles=288000;tc->total_timeout_cycles=320000;
    tc->align_current_pu=tc->startup_iq_pu=tc->iq_limit_pu=0.03125f;
    CHECK(SensorlessStartup_ValidateConfig(tc));
    memset((void *)&g_sl_encoder,0,sizeof(g_sl_encoder));
    g_sl_encoder.sequence=2;g_sl_encoder.initialized=g_sl_encoder.valid=1;
    g_sl_encoder.header=0x6A;g_sl_encoder.tick_ms=1;
    g_sl_encoder_timing.sequence=2;g_sl_encoder_timing.frame_tick_ms=1;
    g_sl_encoder_timing.frame_read_tick=100;g_sl_encoder_timing.frame_request_tick=84;
    g_sensorless_jlink.direction=1;g_sensorless_jlink.target_speed_rpm=100;
    g_sensorless_jlink.duration_ms=20000;
    debug_command(1,SL_JLINK_KEY);
    CHECK(!g_sensorless_jlink.result && SensorlessJlink_EnableRequested());
    StateMachine.RegulFlg=1;SensorlessCanopen_Service1ms();
    CHECK(g_observation_kind==2 && g_probe_trace.h.active && g_probe_trace.h.abi==4);
    g_sensorless_shadow.startup.state=SENSORLESS_START_OPEN_LOOP_RAMP;
    g_sensorless_shadow.startup.state_cycles=33000;
    g_sensorless_shadow.startup.probe.baseline_valid=1;
    g_sensorless_shadow.startup.probe.baseline_done=1;
    g_sensorless_shadow.pwm_counter=100;g_sensorless_chain.chain_tick=99;
    SensorlessShadow_PreFoc();
    CHECK(!(g_sensorless_shadow.startup.probe.gates&SL_P_ENCODER));
    g_sl_encoder_timing.sequence=3;g_sl_encoder.sequence=3;
    g_sl_encoder.motor_raw=777; /* torn value must not leak into the cache */
    g_sensorless_shadow.pwm_counter=110;g_sensorless_chain.chain_tick=109;
    SensorlessShadow_PreFoc();SensorlessShadow_RecordAlignCurrent(.5f,1);
    CHECK(!(g_sensorless_shadow.startup.probe.gates&SL_P_ENCODER));
    CHECK(g_probe_diagnostics.total_samples==1);
    SlProbeTrace_Finish(&g_probe_trace,&g_sensorless_shadow.startup.probe,1U);
    g_source_foc_current.saturated=1;g_source_foc_current.failure_reason=0;
    g_sensorless_shadow.pwm_counter=111;
    SensorlessShadow_RecordAlignCurrent(.5f,1);
    CHECK(g_probe_diagnostics.active && g_probe_diagnostics.total_samples==2);
    CHECK(g_probe_diagnostics.total_saturated==1 && g_probe_diagnostics.max_run_any==1);
    g_source_foc_current.saturated=0;
    g_sensorless_shadow.pwm_counter=133;SensorlessShadow_PreFoc();
    CHECK(g_sensorless_shadow.startup.probe.gates&SL_P_ENCODER);
    g_sl_encoder_timing.sequence=4;g_sl_encoder.sequence=4;g_sl_encoder.valid=0;
    g_sl_encoder_timing.frame_read_tick=134;g_sl_encoder_timing.frame_request_tick=118;
    g_sensorless_shadow.pwm_counter=134;SensorlessShadow_PreFoc();
    CHECK(g_sensorless_shadow.startup.probe.gates&SL_P_ENCODER);
    g_sl_encoder_timing.sequence=5;g_sl_encoder.sequence=5;g_sl_encoder.valid=1;
    g_sensorless_shadow.pwm_counter=135;SensorlessShadow_PreFoc();
    CHECK(g_sensorless_shadow.startup.probe.gates&SL_P_ENCODER);
    g_sl_encoder_timing.sequence=6;g_sl_encoder.sequence=6;g_sl_encoder.valid=1;
    g_sl_encoder.motor_raw=0;g_sl_encoder.age_ms=0;
    debug_command(2,0);
    CHECK(g_sensorless_shadow.startup.probe.stop && SensorlessJlink_EnableRequested());
    SensorlessShadow_PreFoc();
    CHECK(g_sensorless_shadow.startup.probe.phase==SL_P_RELEASE);
    CHECK(g_sensorless_shadow.startup.output.control_active);
    SensorlessStartup_Trip(&g_sensorless_shadow.startup,SENSORLESS_FAULT_CURRENT_OVER);
    SensorlessShadow_PreFoc();SensorlessCanopen_Service1ms();
    CHECK(!g_sensorless_shadow.startup.output.control_active && !SensorlessJlink_EnableRequested());
    CHECK(g_probe_trace.h.frozen && !g_probe_diagnostics.active);
    CHECK(g_sensorless_shadow.startup.probe.a2==(probe_mode==SENSORLESS_DEBUG_SMO_A2));
    CHECK(g_sensorless_shadow.startup.probe.a3==(probe_mode==SENSORLESS_DEBUG_SMO_A3));
    CHECK(g_sensorless_shadow.startup.probe.frame==(probe_mode==SENSORLESS_DEBUG_SMO_FRAME));
    }
    printf("CANopen + J-Link transport: %d failures; simulated board only\\n",failures);
    return failures?1:0;
}

