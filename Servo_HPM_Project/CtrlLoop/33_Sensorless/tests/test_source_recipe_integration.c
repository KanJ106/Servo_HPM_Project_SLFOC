#include <stdio.h>
#include <math.h>
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SensorlessShadow.h"
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
volatile uint32_t g_sensorless_fault_detail;
void Sensorless_GlobalFaultHook(uint32_t flags) { if (!g_sensorless_fault_detail) g_sensorless_fault_detail=flags; }
static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); failures++; } } while(0)
int main(void)
{
    SOURCE_FOC_RECIPE recipe={123,1,1,0.003f,0.004f,10,100,2,1,3000,200,4,1};
    SOURCE_FOC_RECIPE invalid;
    SOURCE_FOC_CURRENT before;
    SMO_PLL_PARAMS smo;
    SENSORLESS_STARTUP_CONFIG startup;
    DrvCoeff.TpwmFrq=16000; DrvCoeff.SpdBase=3000;
    DrvCoeff.SpdMax=3000; DrvCoeff.MotPoles=4;
    DrvCoeff.Rs=_IQ(1); DrvCoeff.Ld=_IQ(0.003); DrvCoeff.Lq=_IQ(0.004);
    DrvCoeff.MotIe=_IQ(0.2); DrvCoeff.SoftOcPonit=_IQ(0.3);
    DrvCoeff.UdcBase=100; DrvCoeff.UdcEpu=_IQ(1); DrvCoeff.DrvIbase=1000;
    SensorlessShadow_Init();
    StateMachine.RegulFlg=1;
    CHECK(!SensorlessShadow_RequestStart(1,500));
    CHECK(!SensorlessShadow_ApplySourceRecipe(&recipe));
    StateMachine.RegulFlg=0;
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    CHECK(g_source_foc_ready && g_source_foc_recipe.recipe_id==123);
    CHECK(!g_sensorless_shadow.startup.config.hfi_enable);
    CHECK(fabsf(g_sensorless_shadow.params.stator_resistance_pu-0.1f)<1e-6f);
    CHECK(fabsf(g_source_foc_current.kp_d-0.003f*0.1f*6.28318530718f*200)<1e-6f);
    before=g_source_foc_current;
    invalid=recipe; invalid.phase_ld_h=NAN;
    CHECK(!SensorlessShadow_ApplySourceRecipe(&invalid));
    CHECK(g_source_foc_current.kp_d==before.kp_d && g_source_foc_recipe.phase_ld_h==recipe.phase_ld_h);
    CHECK(!SensorlessShadow_ConfigureMotorOverride(0));
    smo=g_sensorless_shadow.params; smo.stator_resistance_pu*=2;
    CHECK(!SensorlessShadow_ConfigureSmo(&smo));
    startup=g_sensorless_shadow.startup.config; startup.hfi_enable=1;
    CHECK(!SensorlessShadow_ConfigureStartup(&startup));
    StateMachine.RegulFlg=1;
    g_sensorless_shadow.observer.locked=1;
    g_sensorless_shadow.observer.omega_integrator_e_rad_s=123;
    CHECK(SensorlessShadow_RequestStart(1,500));
    CHECK(!g_sensorless_shadow.observer.locked && g_sensorless_shadow.observer.omega_integrator_e_rad_s==0);
    CHECK(!g_sensorless_shadow.feedback_mux.sensorless.valid);
    CHECK(!SensorlessShadow_StartParamIdentification(0));
    SensorlessShadow_PreFoc(); /* IDLE -> ALIGN */
    SensorlessShadow_PreFoc(); /* first alignment output */
    CHECK(g_sensorless_diag.control_override_active);
    /* Current sample must observe the previous nominal voltage, not new PWM. */
    AdRead.Udc=_IQ(1); Svpwm.Va=_IQ(0.65); Svpwm.Vb=_IQ(0.425); Svpwm.Vc=_IQ(0.425);
    SensorlessShadow_PostFoc();
    CHECK(fabsf(g_sensorless_diag.voltage_alpha_pu)<1e-6f);
    Svpwm.Va=Svpwm.Vb=Svpwm.Vc=_IQ(0.5);
    SensorlessShadow_PostFoc();
    CHECK(fabsf(g_sensorless_diag.voltage_alpha_pu-0.15f)<1e-5f);
    SensorlessShadow_RequestStop();
    SensorlessShadow_PreFoc();
    CHECK(!g_sensorless_shadow.hfi.active);
    StateMachine.RegulFlg=0;
    SensorlessShadow_Reset();
    CHECK(g_source_foc_current.voltage_d==0 && !g_sensorless_diag.control_override_active);
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    CHECK(g_sensorless_shadow.startup.config.debug_mode==SENSORLESS_DEBUG_IF);
    StateMachine.RegulFlg=1;
    CHECK(SensorlessShadow_RequestStart(1,500));
    SensorlessShadow_PreFoc(); SensorlessShadow_PreFoc(); SensorlessShadow_PostFoc();
    CHECK(!g_sensorless_diag.encoder_valid && g_sensorless_diag.encoder_speed_rpm==0);
    CHECK(!g_sensorless_shadow.feedback_mux.encoder.valid);
    CHECK(!SourceFoc_Step(&g_source_foc_current,0,0,0.3f,0,0.5f));
    SensorlessShadow_CurrentFault(g_source_foc_current.failure_reason);
    CHECK(g_sensorless_shadow.startup.failure.fault==SENSORLESS_FAULT_CURRENT_OVER);
    CHECK(g_sensorless_trace.frozen && !g_sensorless_trace.active && g_sensorless_trace.count>0);
    CHECK(g_sensorless_trace.samples[g_sensorless_trace.write_index-1].current_failure==SOURCE_FOC_OVER_CURRENT);
    CHECK(!g_sensorless_diag.control_override_active && g_sensorless_diag.fault_request);
    {
        unsigned count=g_sensorless_trace.count;
        SensorlessShadow_CurrentFault(SOURCE_FOC_INPUT_INVALID);
        SensorlessShadow_PostFoc();
        CHECK(g_sensorless_trace.count==count);
        CHECK(g_sensorless_shadow.startup.failure.fault==SENSORLESS_FAULT_CURRENT_OVER);
        StateMachine.RegulFlg=0; SensorlessShadow_Reset();
        CHECK(g_sensorless_shadow.startup.failure.fault==SENSORLESS_FAULT_CURRENT_OVER);
        CHECK(g_sensorless_trace.frozen);
        { FILE *f=fopen("startup_failure_simulated.bin","wb");
          CHECK(f!=0); if(f){ fwrite((const void *)&g_sensorless_trace,sizeof(g_sensorless_trace),1,f); fclose(f); } }
    }
    SensorlessShadow_ClearFault();
    CHECK(!g_sensorless_trace.frozen);
    recipe.allow_continuous_rotation=0;
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    CHECK(g_sensorless_shadow.startup.config.debug_mode==SENSORLESS_DEBUG_ALIGN);
    startup=g_sensorless_shadow.startup.config;
    startup.debug_mode=SENSORLESS_DEBUG_IF;
    CHECK(SensorlessShadow_ConfigureStartup(&startup));
    StateMachine.RegulFlg=1;
    CHECK(!SensorlessShadow_RequestStart(1,500));
    CHECK(!g_sensorless_shadow.startup.start_requested);
    StateMachine.RegulFlg=0;
    startup.debug_mode=SENSORLESS_DEBUG_FULL;
    CHECK(SensorlessShadow_ConfigureStartup(&startup));
    StateMachine.RegulFlg=1;
    CHECK(!SensorlessShadow_RequestStart(1,500));
    StateMachine.RegulFlg=0;
    startup.debug_mode=SENSORLESS_DEBUG_ALIGN;
    CHECK(SensorlessShadow_ConfigureStartup(&startup));
    StateMachine.RegulFlg=1;
    CHECK(SensorlessShadow_RequestStart(1,500));
    SensorlessShadow_PreFoc(); SensorlessShadow_PreFoc();
    CHECK(g_sensorless_diag.control_override_active);
    CHECK(g_sensorless_shadow.startup.state==SENSORLESS_START_ALIGN);
    /* Actual 48 V joint recipe at the board's 8192 rpm / 25.6 A bases. */
    StateMachine.RegulFlg=0;
    SensorlessShadow_Reset();
    DrvCoeff.SpdBase=8192;
    recipe=(SOURCE_FOC_RECIPE){0x480901,1,0.23166667f,0.00046f,0.00046f,
        25.6f,48.0f,6.363961f,0.5f,600.0f,100.0f,10,0};
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    CHECK(g_source_foc_ready);
    CHECK(g_sensorless_shadow.motor_profile.effective.ld_pu>2.0f);
    CHECK(fabsf(g_source_foc_current.current_limit_pu-0.01953125f)<1e-7f);
    CHECK(fabsf(g_source_foc_current.measured_trip_pu-0.0390625f)<1e-7f);
    startup=g_sensorless_shadow.startup.config;
    startup.align_cycles=3200;
    startup.total_timeout_cycles=16000;
    CHECK(SensorlessShadow_ConfigureStartup(&startup));
    CHECK(startup.debug_mode==SENSORLESS_DEBUG_ALIGN);
    invalid=recipe; invalid.phase_ld_h=0.001f; /* outside bounded 4 pu range */
    CHECK(!SensorlessShadow_ApplySourceRecipe(&invalid));
    invalid=recipe; invalid.phase_lq_h=0.001f;
    CHECK(!SensorlessShadow_ApplySourceRecipe(&invalid));
    StateMachine.RegulFlg=1;
    CHECK(SensorlessShadow_RequestStart(1,600));
    SensorlessShadow_PreFoc(); SensorlessShadow_PreFoc();
    CHECK(g_sensorless_diag.control_override_active);
    CHECK(fabsf(g_sensorless_diag.id_command_pu-0.0078125f)<1e-7f);
    CHECK(g_sensorless_diag.iq_command_pu==0.0f);
    { unsigned i;
      for(i=0;i<3202;++i) SensorlessShadow_PreFoc();
      CHECK(g_sensorless_shadow.startup.debug_completed);
      CHECK(!g_sensorless_diag.control_override_active);
    }
    /* Real 21.33 A-base recipe must remain stable across tuning and reapply. */
    StateMachine.RegulFlg=0; SensorlessShadow_Reset();
    recipe.current_base_a=21.33f; recipe.current_limit_a=21.33f*.03125f;
    recipe.allow_continuous_rotation=1; DrvCoeff.DrvIbase=2133;
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    smo=g_sensorless_shadow.params;
    CHECK(smo.boundary_pu>.078f && smo.boundary_pu<.080f);
    smo.boundary_pu=.02f;
    CHECK(!SensorlessShadow_ConfigureSmo(&smo));
    CHECK(g_sensorless_shadow.params.boundary_pu>.078f);
    smo=g_sensorless_shadow.params; smo.boundary_pu=.12f;
    CHECK(SensorlessShadow_ConfigureSmo(&smo));
    CHECK(SensorlessShadow_ApplySourceRecipe(&recipe));
    CHECK(g_sensorless_shadow.params.boundary_pu>.078f
        && g_sensorless_shadow.params.boundary_pu<.080f);
    smo=g_sensorless_shadow.params;
    g_sensorless_shadow.params.boundary_pu=.02f;
    StateMachine.RegulFlg=1;
    CHECK(!SensorlessShadow_RequestStart(-1,100));
    StateMachine.RegulFlg=0; g_sensorless_shadow.params=smo;
    {
        SENSORLESS_MOTOR_PROFILE candidate;
        SOURCE_FOC_CURRENT candidate_current;
        invalid=recipe; invalid.phase_rs_ohm=4.0f;
        invalid.phase_ld_h=invalid.phase_lq_h=.000282f;
        CHECK(SourceFoc_BuildProfile(&invalid,16000,8192,&candidate));
        CHECK(SourceFoc_Configure(&candidate_current,&candidate,100));
        CHECK(!SensorlessShadow_ApplySourceRecipe(&invalid));
        CHECK(g_source_foc_ready && g_source_foc_recipe.phase_rs_ohm==recipe.phase_rs_ohm);
        CHECK(g_sensorless_shadow.params.current_model_gain==smo.current_model_gain);
    }
    g_observation_request=0; /* Preserve coverage of the original SMO window. */
    /* Exercise the real PostFoc sampling hook, not just the trace helper. */
    startup=g_sensorless_shadow.startup.config;
    startup.debug_mode=SENSORLESS_DEBUG_TRAVEL90;
    startup.handoff_speed_rpm=100;startup.open_loop_accel_rpm_s=100;
    startup.run_accel_rpm_s=100;startup.align_cycles=8000;
    startup.sensorless_exit_speed_rpm=20;
    startup.ramp_timeout_cycles=288000;startup.total_timeout_cycles=320000;
    CHECK(SensorlessShadow_ConfigureStartup(&startup));
    StateMachine.RegulFlg=1;CHECK(SensorlessShadow_RequestStart(-1,100));
    CHECK(g_smo_trace.header.active && g_smo_trace.header.trigger_stage_cycles==32000);
    g_smo_trace.header.trigger_stage_cycles=0; /* shorten only this host fixture */
    AdRead.Udc=_IQ(1);DrvCoeff.UdcEpu=_IQ(1);
    Clark.Alpha=_IQ(.001f);Clark.Beta=_IQ(.002f);
    for(unsigned n=0;n<14000;n++) {
        SensorlessShadow_PreFoc();
        g_sensorless_chain.chain_tick=(int32_t)g_sensorless_shadow.pwm_counter;
        g_sensorless_chain.chain_valid=1;
        Svpwm.Va=_IQ(.65f);Svpwm.Vb=Svpwm.Vc=_IQ(.425f);
        SensorlessShadow_PostFoc();
        if(g_smo_trace.header.count==1 && g_smo_trace.samples[0].context.tick==g_sensorless_shadow.pwm_counter) {
            CHECK(fabsf(g_smo_trace.samples[0].voltage_alpha-.15f)<1e-5f);
            CHECK(g_smo_trace.samples[0].current_hat_alpha==g_sensorless_shadow.observer.current_hat_alpha_pu);
            CHECK(g_smo_trace.samples[0].context.chain_valid==1);
        }
    }
    CHECK(g_smo_trace.header.count==SL_SMO_TRACE_CAPACITY && g_smo_trace.header.frozen);
    CHECK(g_smo_trace.header.active && g_smo_trace.header.total_samples==14000);
    CHECK(g_smo_trace.samples[0].context.direction==-1);
    for(unsigned n=1;n<SL_SMO_TRACE_CAPACITY;n++)
        CHECK(g_smo_trace.samples[n].context.tick==g_smo_trace.samples[n-1].context.tick+SL_SMO_TRACE_DIVIDER);
    SensorlessShadow_RequestStop();SensorlessShadow_PreFoc();SensorlessShadow_PostFoc();
    CHECK(!g_smo_trace.header.active && g_smo_trace.header.frozen);
    CHECK(g_smo_trace.header.stop_reason==1);
    { FILE *f=fopen("smo_window_simulated.bin","wb");
      CHECK(f!=0);if(f){fwrite((const void *)&g_smo_trace,sizeof(g_smo_trace),1,f);fclose(f);} }
    StateMachine.RegulFlg=0;
    printf("source recipe integration: %d failures\n",failures);
    return failures?1:0;
}
