#include <stdio.h>
#include <math.h>
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SensorlessShadow.h"
#include "SensorlessProduction.h"
SENSORLESS_CHECK_DRVCOEFF DrvCoeff;
SENSORLESS_CHECK_ETHETA Etheta;
SENSORLESS_CHECK_IDREF IdRef;
SENSORLESS_CHECK_IQREF IqRef;
SENSORLESS_CHECK_SPDFB SpdFb;
SENSORLESS_CHECK_SVPWM Svpwm;
SENSORLESS_CHECK_CLARK Clark;
SENSORLESS_CHECK_PARK Park;
SENSORLESS_CHECK_ADREAD AdRead;
SENSORLESS_CHECK_STATE_MACHINE StateMachine;
SENSORLESS_CHECK_FAULT FaultP;
static int failures;
static unsigned ticks;
#define CHECK(x) do { if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);failures++;} } while(0)
void FaultPrtt_FaultInterface(Uint16 fault) { (void)fault; FaultP.FaultStatus |= FP_ERR; }
static unsigned command(unsigned cmd) {
    g_sensorless_production.command=(uint16_t)cmd;
    g_sensorless_production.request_seq++;
    SensorlessProduction_Service1ms();
    CHECK(g_sensorless_production.acknowledge_seq==g_sensorless_production.request_seq);
    return g_sensorless_production.command_result;
}
static void tick(void) {
    SensorlessShadow_PreFoc();
    Etheta.Ethetapk=Etheta.EtaInerGet;
    if(g_sensorless_diag.control_override_active) {
        CHECK(SourceFoc_Step(&g_source_foc_current,g_sensorless_diag.id_command_pu,
              g_sensorless_diag.iq_command_pu,0,0,0.5f));
    } else SourceFoc_Reset(&g_source_foc_current);
    /* Transport/state-machine fixture only: no power stage or motor model. */
    /* Fixture represents one completed hardware acquisition/PWM pair. */
    g_sensorless_chain.chain_tick=(int32_t)g_sensorless_shadow.pwm_counter;
    g_sensorless_chain.chain_valid=1;
    g_sensorless_chain.sdm_u=-1234;
    g_sensorless_chain.sdm_v=2345;
    g_sensorless_chain.pwm_u=321;
    SensorlessShadow_PostFoc();
    if(++ticks%16==0) SensorlessProduction_Service1ms();
}
static void save(const char *path) {
    FILE *f=fopen(path,"wb");
    CHECK(g_sensorless_trace.frozen && !(g_sensorless_trace.sequence&1U));
    CHECK(f!=0);
    if(f){ CHECK(fwrite((const void *)&g_sensorless_trace,sizeof(g_sensorless_trace),1,f)==1);fclose(f); }
}
int main(void) {
    SOURCE_FOC_RECIPE recipe={321,1,1,0.003f,0.004f,10,100,2,1,3000,200,4,1};
    unsigned i, first_record;
    DrvCoeff.TpwmFrq=16000;DrvCoeff.SpdBase=3000;DrvCoeff.SpdMax=3000;
    DrvCoeff.MotPoles=4;DrvCoeff.Rs=_IQ(1);DrvCoeff.Ld=_IQ(0.003);DrvCoeff.Lq=_IQ(0.004);
    DrvCoeff.MotIe=_IQ(0.2);DrvCoeff.SoftOcPonit=_IQ(0.3);
    DrvCoeff.UdcBase=100;DrvCoeff.UdcEpu=_IQ(1);DrvCoeff.DrvIbase=1000;
    AdRead.Udc=_IQ(1);Svpwm.Va=Svpwm.Vb=Svpwm.Vc=_IQ(0.5);
    SensorlessShadow_Init();SensorlessProduction_Init();
    g_sensorless_production.source_recipe=recipe;
    CHECK(command(12)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_production.effective_recipe.recipe_id==321);
    g_sensorless_production.tuning.mask=SENSORLESS_TUNING_DEBUG|SENSORLESS_TUNING_TIMING|SENSORLESS_TUNING_SPEED;
    g_sensorless_production.tuning.debug_mode=SENSORLESS_DEBUG_IF;
    g_sensorless_production.tuning.current_ramp_ms=2;
    g_sensorless_production.tuning.if_hold_ms=5;
    g_sensorless_production.tuning.iq_hold_ms=2;
    g_sensorless_production.tuning.ramp_timeout_ms=100;
    g_sensorless_production.tuning.total_timeout_ms=300;
    g_sensorless_production.tuning.run_test_ms=10;
    g_sensorless_production.tuning.align_ms=2;
    g_sensorless_production.tuning.lock_dwell_ms=2;
    g_sensorless_production.tuning.blend_ms=2;
    g_sensorless_production.tuning.lock_timeout_ms=100;
    g_sensorless_production.tuning.hfi_lock_dwell_ms=1;
    g_sensorless_production.tuning.hfi_timeout_ms=1;
    g_sensorless_production.tuning.unlock_dwell_ms=2;
    g_sensorless_production.tuning.monitor_dwell_ms=2;
    g_sensorless_production.tuning.fallback_ready_ms=2;
    g_sensorless_production.tuning.stop_dwell_ms=2;
    g_sensorless_production.tuning.stop_timeout_ms=100;
    g_sensorless_production.tuning.handoff_speed_rpm=150;
    g_sensorless_production.tuning.sensorless_exit_speed_rpm=30;
    g_sensorless_production.tuning.open_loop_accel_rpm_s=5000;
    g_sensorless_production.tuning.run_accel_rpm_s=1000;
    g_sensorless_production.tuning.stop_speed_rpm=30;
    CHECK(command(2)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_production.effective_startup.if_hold_cycles==80);
    g_sensorless_production.target_speed_rpm=150;
    g_sensorless_production.direction=1;
    g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_SERVO_NOT_READY);
    StateMachine.RegulFlg=1; /* explicit simulated platform enable */
    g_sensorless_production.arm_key=0;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_BAD_ARM_KEY);
    g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_production.arm_key==0);
    CHECK(g_sensorless_trace.abi==3 && g_sensorless_trace.sample_bytes==264);
    first_record=g_sensorless_trace.record_id;
    tick();tick();
    CHECK(command(2)==SENSORLESS_PROD_RESULT_INVALID_TUNING);
    for(i=0;i<2000 && !g_sensorless_trace.frozen;i++){
        tick();
        CHECK(g_sensorless_shadow.startup.state!=SENSORLESS_START_BLEND);
        CHECK(g_sensorless_shadow.startup.state!=SENSORLESS_START_RUN);
    }
    SensorlessProduction_Service1ms();
    CHECK(g_sensorless_shadow.startup.debug_completed && g_sensorless_trace.frozen);
    CHECK(!g_sensorless_diag.control_override_active && !g_sensorless_diag.encoder_valid);
    CHECK(StateMachine.RegulFlg==1); /* finishing test is NOT platform Servo OFF */
    CHECK(g_sensorless_trace.samples[0].chain.chain_valid==1);
    CHECK(g_sensorless_trace.samples[0].chain.sdm_u==-1234);
    CHECK(g_sensorless_trace.samples[0].chain.pwm_u==321);
    save("debug_chain_if_simulated.bin");
    g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_trace.record_id==first_record+1 && !g_sensorless_trace.frozen);
    tick();tick();
    CHECK(!SourceFoc_Step(&g_source_foc_current,0,0,0.3f,0,0.5f));
    SensorlessShadow_CurrentFault(g_source_foc_current.failure_reason);
    SensorlessProduction_Service1ms();
    CHECK(g_sensorless_production.first_failure.fault==SENSORLESS_FAULT_CURRENT_OVER);
    CHECK(FaultP.FaultStatus&FP_ERR);
    save("debug_chain_fault_simulated.bin");
    StateMachine.RegulFlg=0; /* simulated platform disable, not direct HW operation */
    CHECK(command(6)==SENSORLESS_PROD_RESULT_ACCEPTED);
    CHECK(g_sensorless_shadow.startup.failure.fault==0 && !g_sensorless_trace.frozen);
    CHECK(FaultP.FaultStatus&FP_ERR); /* mailbox clear does not clear drive faults */
    StateMachine.RegulFlg=1;
    g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_SERVO_NOT_READY);
    StateMachine.RegulFlg=0;FaultP.FaultStatus=0; /* simulate external platform reset */
    StateMachine.RegulFlg=1;
    g_sensorless_production.abi_version=0x00060000;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_INVALID_COMMAND);
    g_sensorless_production.abi_version=SENSORLESS_PRODUCTION_ABI_VERSION;
    CHECK(command(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
    tick();tick();
    CHECK(command(5)==SENSORLESS_PROD_RESULT_ACCEPTED);
    tick();
    CHECK(g_sensorless_shadow.startup.state==SENSORLESS_START_IDLE && !g_sensorless_diag.control_override_active);
    for(unsigned leg_ms=500;leg_ms<=2000;leg_ms*=2) {
        StateMachine.RegulFlg=0;
        g_sensorless_production.tuning.debug_mode=SENSORLESS_DEBUG_D_AXIS_SCAN;
        g_sensorless_production.tuning.if_hold_ms=leg_ms;
        g_sensorless_production.tuning.align_ms=500;
        g_sensorless_production.tuning.ramp_timeout_ms=2*leg_ms+100;
        g_sensorless_production.tuning.total_timeout_ms=2*leg_ms+800;
        CHECK(command(2)==SENSORLESS_PROD_RESULT_ACCEPTED);
        CHECK(g_sensorless_production.effective_startup.if_hold_cycles==leg_ms*16);
        StateMachine.RegulFlg=1;
        g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
        CHECK(command(3)==SENSORLESS_PROD_RESULT_ACCEPTED);
        CHECK(command(2)==SENSORLESS_PROD_RESULT_INVALID_TUNING);
        for(i=0;i<(2*leg_ms+800)*16 && !g_sensorless_trace.frozen;i++) tick();
        CHECK(g_sensorless_shadow.startup.debug_completed && g_sensorless_trace.frozen);
        CHECK(!g_sensorless_diag.control_override_active);
        CHECK(g_sensorless_trace.count>100 && g_sensorless_trace.count<SENSORLESS_TRACE_CAPACITY);
        CHECK(g_sensorless_trace.samples[0].total_cycles<10);
        CHECK(g_sensorless_trace.samples[g_sensorless_trace.count-1].state==SENSORLESS_START_IDLE);
        CHECK(g_sensorless_trace.samples[g_sensorless_trace.count-1].total_cycles>=(500+2*leg_ms)*16);
        CHECK(g_sensorless_trace.samples[g_sensorless_trace.count-1].first_fault==0);
        save("debug_chain_scan_simulated.bin");
    }
    printf("debug chain real mailbox + shadow + startup: %d failures; simulated hardware only\n",failures);
    return failures?1:0;
}

