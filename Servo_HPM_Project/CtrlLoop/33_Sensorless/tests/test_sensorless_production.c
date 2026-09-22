#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Drive.h"
#include "SensorlessProduction.h"
#include "SensorlessShadow.h"
#include "SV_FaultProtect.h"

SENSORLESS_CHECK_DRVCOEFF DrvCoeff;
SENSORLESS_CHECK_ETHETA Etheta;
SENSORLESS_CHECK_IDREF IdRef;
SENSORLESS_CHECK_IQREF IqRef;
SENSORLESS_CHECK_SPDFB SpdFb;
SENSORLESS_CHECK_SVPWM Svpwm;
SENSORLESS_CHECK_CLARK Clark;
SENSORLESS_CHECK_ADREAD AdRead;
SENSORLESS_CHECK_STATE_MACHINE StateMachine;
SENSORLESS_CHECK_FAULT FaultP;
SENSORLESS_SHADOW_RUNTIME g_sensorless_shadow;
volatile SENSORLESS_SHADOW_DIAG g_sensorless_diag;
volatile uint16_t g_source_foc_ready;
SOURCE_FOC_RECIPE g_source_foc_recipe;
SOURCE_FOC_CURRENT g_source_foc_current;
uint16_t SensorlessShadow_ApplySourceRecipe(const SOURCE_FOC_RECIPE *r)
{
    if (!r || StateMachine.RegulFlg) return 0U;
    g_source_foc_recipe = *r;
    g_source_foc_ready = 1U;
    return 1U;
}

static int g_failures;
static uint16_t g_fault_code;
static uint16_t g_start_calls;
static uint16_t g_stop_calls;
static uint16_t g_configure_calls;
static uint16_t g_ident_start_calls;
static uint16_t g_ident_abort_calls;
static uint16_t g_ident_apply_calls;

extern void Sensorless_GlobalFaultHook(uint32_t fault_flags);
extern volatile uint32_t g_sensorless_fault_detail;

static void Test_Check(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

void FaultPrtt_FaultInterface(Uint16 fault)
{
    g_fault_code = fault;
}

uint16_t SensorlessShadow_ConfigureMotorOverride(
    const SENSORLESS_MOTOR_OVERRIDE *override_values)
{
    (void)override_values;
    g_configure_calls++;
    return 1U;
}

uint16_t SensorlessShadow_ConfigureStartup(
    const SENSORLESS_STARTUP_CONFIG *config)
{
    if (SensorlessStartup_ValidateConfig(config) == 0U) {
        return 0U;
    }
    g_sensorless_shadow.startup.config = *config;
    g_configure_calls++;
    return 1U;
}

uint16_t SensorlessShadow_ConfigureHfi(
    const HFI_ESTIMATOR_PARAMS *params)
{
    if (HfiEstimator_ValidateParams(params) == 0U) {
        return 0U;
    }
    g_sensorless_shadow.hfi_params = *params;
    g_configure_calls++;
    return 1U;
}

uint16_t SensorlessShadow_ConfigureSmo(
    const SMO_PLL_PARAMS *params)
{
    if (SmoPll_ValidateParams(params) == 0U) {
        return 0U;
    }
    g_sensorless_shadow.params = *params;
    g_configure_calls++;
    return 1U;
}

uint16_t SensorlessShadow_StartParamIdentification(
    const MOTOR_PARAM_IDENT_CONFIG *config)
{
    g_ident_start_calls++;
    g_sensorless_shadow.identification.config = *config;
    g_sensorless_shadow.identification.state = MOTOR_IDENT_HFI_ACQUIRE;
    return 1U;
}

void SensorlessShadow_AbortParamIdentification(void)
{
    g_ident_abort_calls++;
    g_sensorless_shadow.identification.state = MOTOR_IDENT_FAILED;
}

uint16_t SensorlessShadow_ApplyIdentifiedMotor(void)
{
    g_ident_apply_calls++;
    return (uint16_t)(((StateMachine.RegulFlg == 0U)
        && (g_sensorless_shadow.identification.result.valid != 0U)) ? 1U : 0U);
}

uint16_t SensorlessShadow_RequestStart(int16_t direction, int32_t target_speed_rpm)
{
    (void)direction;
    (void)target_speed_rpm;
    g_start_calls++;
    g_sensorless_shadow.startup.state = SENSORLESS_START_ALIGN;
    return 1U;
}

uint16_t SensorlessShadow_UpdateTarget(int16_t direction,
                                       int32_t target_speed_rpm)
{
    return SensorlessStartup_UpdateTarget(&g_sensorless_shadow.startup,
                                          direction, target_speed_rpm);
}

void SensorlessShadow_RequestStop(void)
{
    g_stop_calls++;
    g_sensorless_shadow.startup.state = SENSORLESS_START_STOPPING;
}

void SensorlessShadow_ClearFault(void)
{
    g_sensorless_shadow.startup.state = SENSORLESS_START_IDLE;
}

static void Test_PrepareRuntime(void)
{
    SENSORLESS_STARTUP_CONFIG config;

    memset(&g_sensorless_shadow, 0, sizeof(g_sensorless_shadow));
    memset((void *)&g_sensorless_diag, 0, sizeof(g_sensorless_diag));
    memset(&StateMachine, 0, sizeof(StateMachine));
    memset(&FaultP, 0, sizeof(FaultP));
    SensorlessStartup_DefaultConfig(&config, 0.001f, 4.0f,
                                    3000.0f, 3000.0f, 0.20f);
    SensorlessStartup_Init(&g_sensorless_shadow.startup, &config);
    HfiEstimator_DefaultParams(&g_sensorless_shadow.hfi_params,
        0.001f, 4000.0f, 0.20f, 1);
    g_sensorless_shadow.params.sample_time_s = 0.001f;
    g_sensorless_shadow.params.stator_resistance_pu = 0.05f;
    g_sensorless_shadow.params.current_model_gain = 0.04f;
    g_sensorless_shadow.params.switching_gain = 0.25f;
    g_sensorless_shadow.params.boundary_pu = 0.01f;
    g_sensorless_shadow.params.emf_lpf_alpha = 0.10f;
    g_sensorless_shadow.params.emf_delay_comp_gain = 1.0f;
    g_sensorless_shadow.params.pll_kp = 200.0f;
    g_sensorless_shadow.params.pll_ki = 10000.0f;
    g_sensorless_shadow.params.pll_speed_lpf_alpha = 0.05f;
    g_sensorless_shadow.params.weak_emf_speed_decay = 0.001f;
    g_sensorless_shadow.params.min_emf_pu = 0.01f;
    g_sensorless_shadow.params.unlock_emf_ratio = 0.70f;
    g_sensorless_shadow.params.lock_phase_error = 0.35f;
    g_sensorless_shadow.params.max_electrical_rad_s = 4000.0f;
    g_sensorless_shadow.params.max_accel_e_rad_s2 = 50000.0f;
    g_sensorless_shadow.params.current_hat_limit_pu = 2.0f;
    g_sensorless_shadow.params.lock_cycles = 30U;
    g_sensorless_shadow.params.unlock_cycles = 10U;
    g_sensorless_shadow.motor_profile.valid_for_control = 1U;
    g_sensorless_shadow.motor_profile.hfi_capable = 1U;
    g_sensorless_shadow.motor_profile.effective.max_speed_rpm = 3000.0f;
    MotorParamIdentification_DefaultConfig(
        &g_sensorless_shadow.identification.config,
        0.001f, 1000.0f, 4.0f, 200.0f, 10.0f, 0.20f);
    g_sensorless_diag.profile_valid_for_control = 1U;
    g_sensorless_diag.encoder_valid = 0U;
    g_sensorless_diag.hfi_capable = 1U;
    g_sensorless_diag.hfi_locked = 1U;
    g_sensorless_diag.hfi_quality = 900U;
    g_sensorless_diag.observer_locked = 1U;
    g_sensorless_diag.observer_quality = 900U;
    g_sensorless_diag.observer_speed_rpm = 500;
    g_sensorless_diag.encoder_speed_rpm = 500;
    g_fault_code = 0U;
    g_sensorless_fault_detail = 0U;
    g_start_calls = 0U;
    g_stop_calls = 0U;
    g_configure_calls = 0U;
    g_ident_start_calls = 0U;
    g_ident_abort_calls = 0U;
    g_ident_apply_calls = 0U;
    SensorlessProduction_Init();
    g_source_foc_ready = 1U;
    g_sensorless_production.metric_limits = (PRODUCTION_METRIC_LIMITS){0.0f, 2.0f, 4.0f, 1.0f, 100.0f, 1U};
}

static void Test_ArmingAndAutomaticRun(void)
{
    uint32_t index;

    Test_PrepareRuntime();
    StateMachine.RegulFlg = 1U;
    g_sensorless_production.command = SENSORLESS_PROD_CMD_RUN_TEST;
    g_sensorless_production.direction = 1;
    g_sensorless_production.target_speed_rpm = 500;
    g_sensorless_production.test_hold_ms = 3U;
    g_sensorless_production.test_timeout_ms = 1000U;
    g_sensorless_production.request_seq = 1U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_BAD_ARM_KEY,
               "automatic test started without the arm key");

    g_sensorless_production.arm_key = SENSORLESS_PRODUCTION_ARM_KEY;
    g_sensorless_production.request_seq = 2U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_TEST_RUNNING,
               "armed automatic test was not accepted");
    Test_Check(g_start_calls == 1U, "start API was not called exactly once");

    g_sensorless_shadow.startup.state = SENSORLESS_START_RUN;
    g_sensorless_diag.startup_state = SENSORLESS_START_RUN;
    for (index = 0U; index < 205U; index++) {
        ProductionMetrics_Sample(0.0f, 0.0f, 500.0f, 48.0f);
        SensorlessProduction_Service1ms();
    }
    Test_Check(g_stop_calls == 1U,
               "successful hold did not request a controlled stop");
    Test_Check(g_sensorless_production.production_state
               == SENSORLESS_PROD_STATE_STOPPING,
               "production sequence did not enter STOPPING");

    g_sensorless_shadow.startup.state = SENSORLESS_START_IDLE;
    g_sensorless_diag.startup_state = SENSORLESS_START_IDLE;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_TEST_PASS,
               "completed automatic run was not marked PASS");
    Test_Check(g_sensorless_production.completed_tests == 1U,
               "completed-test counter was not updated");
    Test_Check(g_sensorless_production.arm_key == 0U,
               "arm key was not consumed after the test");
}

static void Test_TimeoutStopIsIssuedOnce(void)
{
    uint32_t index;

    Test_PrepareRuntime();
    StateMachine.RegulFlg = 1U;
    g_sensorless_production.command = SENSORLESS_PROD_CMD_RUN_TEST;
    g_sensorless_production.arm_key = SENSORLESS_PRODUCTION_ARM_KEY;
    g_sensorless_production.direction = 1;
    g_sensorless_production.target_speed_rpm = 500;
    g_sensorless_production.test_hold_ms = 1000U;
    g_sensorless_production.test_timeout_ms = 2U;
    g_sensorless_production.request_seq = 1U;
    SensorlessProduction_Service1ms();

    g_sensorless_shadow.startup.state = SENSORLESS_START_RUN;
    g_sensorless_diag.startup_state = SENSORLESS_START_RUN;
    SensorlessProduction_Service1ms();
    for (index = 0U; index < 10U; index++) {
        SensorlessProduction_Service1ms();
    }
    Test_Check(g_stop_calls == 1U,
               "timeout path issued more than one stop request");

    g_sensorless_shadow.startup.state = SENSORLESS_START_IDLE;
    g_sensorless_diag.startup_state = SENSORLESS_START_IDLE;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_TEST_TIMEOUT,
               "timeout path did not preserve the TIMEOUT result");
}

static void Test_TuningAndFaultMapping(void)
{
    Test_PrepareRuntime();
    g_sensorless_production.command = SENSORLESS_PROD_CMD_APPLY_TUNING;
    g_sensorless_production.tuning.mask = SENSORLESS_TUNING_PI;
    g_sensorless_production.tuning.speed_kp = 0.3f;
    g_sensorless_production.tuning.speed_ki = 3.0f;
    g_sensorless_production.request_seq = 1U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_ACCEPTED,
               "valid PI tuning was rejected");
    Test_Check(g_configure_calls == 1U,
               "tuning command did not reach the configuration API");

    Sensorless_GlobalFaultHook(SENSORLESS_FAULT_PARAM_INVALID);
    Test_Check(g_fault_code == SensorlessParamErr,
               "parameter fault was mapped to the wrong drive fault");
    Sensorless_GlobalFaultHook(SENSORLESS_FAULT_LOCK_LOST);
    Test_Check(g_fault_code == SensorlessTrackErr,
               "tracking fault was mapped to the wrong drive fault");
    Test_Check(g_sensorless_fault_detail == SENSORLESS_FAULT_PARAM_INVALID,
               "first fault detail was overwritten");
}

static void Test_HfiPoleTuningAndStatus(void)
{
    Test_PrepareRuntime();
    g_sensorless_production.command=SENSORLESS_PROD_CMD_APPLY_TUNING;
    g_sensorless_production.tuning.mask=SENSORLESS_TUNING_HFI;
    g_sensorless_production.request_seq=1;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result==SENSORLESS_PROD_RESULT_INVALID_TUNING
        && g_configure_calls==0, "disabled HFI tuning changed active configuration");
}

static void Test_SmoTuningAndStatus(void)
{
    volatile SENSORLESS_PRODUCTION_TUNING *tuning;

    Test_PrepareRuntime();
    tuning = &g_sensorless_production.tuning;
    tuning->mask = SENSORLESS_TUNING_SMO;
    tuning->smo_switching_gain = 0.30f;
    tuning->smo_boundary_pu = 0.015f;
    tuning->smo_emf_lpf_alpha = 0.12f;
    tuning->smo_emf_delay_comp_gain = 0.90f;
    tuning->smo_pll_kp = 220.0f;
    tuning->smo_pll_ki = 11000.0f;
    tuning->smo_pll_speed_lpf_alpha = 0.08f;
    tuning->smo_weak_emf_speed_decay = 0.002f;
    tuning->smo_min_emf_pu = 0.012f;
    tuning->smo_unlock_emf_ratio = 0.65f;
    tuning->smo_lock_phase_error = 0.30f;
    tuning->smo_max_accel_e_rad_s2 = 45000.0f;
    tuning->smo_current_hat_limit_pu = 1.5f;
    tuning->smo_lock_ms = 40U;
    tuning->smo_unlock_ms = 12U;

    g_sensorless_diag.observer_raw_theta_q24 = 123456;
    g_sensorless_diag.observer_phase_advance_turns = 0.012f;
    g_sensorless_diag.observer_omega_tracking_e_rad_s = 345.0f;
    g_sensorless_diag.observer_emf_confidence = 0.85f;
    g_sensorless_diag.observer_current_error_pu = 0.022f;
    g_sensorless_diag.observer_weak_emf_counter = 17U;

    g_sensorless_production.command = SENSORLESS_PROD_CMD_APPLY_TUNING;
    g_sensorless_production.request_seq = 1U;
    SensorlessProduction_Service1ms();

    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_ACCEPTED,
               "valid SMO tuning was rejected");
    Test_Check(g_configure_calls == 2U,
               "SMO tuning did not configure startup and observer");
    Test_Check(g_sensorless_shadow.params.emf_delay_comp_gain
               == tuning->smo_emf_delay_comp_gain,
               "SMO delay compensation did not reach the observer");
    Test_Check(g_sensorless_shadow.params.pll_speed_lpf_alpha
               == tuning->smo_pll_speed_lpf_alpha,
               "SMO speed filter did not reach the observer");
    Test_Check(g_sensorless_shadow.params.lock_cycles == 40U,
               "SMO lock time was not converted to observer cycles");
    Test_Check(g_sensorless_shadow.params.unlock_cycles == 12U,
               "SMO unlock time was not converted to observer cycles");
    Test_Check(g_sensorless_production.observer_raw_theta_q24 == 123456,
               "SMO raw angle was not published to the mailbox");
    Test_Check(g_sensorless_production.observer_emf_confidence
               == g_sensorless_diag.observer_emf_confidence,
               "SMO EMF confidence was not published to the mailbox");
    Test_Check(g_sensorless_production.observer_weak_emf_counter == 17U,
               "SMO weak-EMF counter was not published to the mailbox");
}

static void Test_ParameterIdentificationCommands(void)
{
    Test_PrepareRuntime();
    StateMachine.RegulFlg = 1U;
    g_sensorless_production.command =
        SENSORLESS_PROD_CMD_RUN_IDENTIFICATION;
    g_sensorless_production.arm_key = SENSORLESS_PRODUCTION_ARM_KEY;
    g_sensorless_production.request_seq = 1U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_IDENT_RUNNING,
               "parameter identification command was not accepted");
    Test_Check(g_ident_start_calls == 1U,
               "parameter identification API was not called");

    g_sensorless_shadow.identification.result.rs_pu = 0.10f;
    g_sensorless_shadow.identification.result.ld_pu = 0.02f;
    g_sensorless_shadow.identification.result.lq_pu = 0.03f;
    g_sensorless_shadow.identification.result.ls_pu = 0.025f;
    g_sensorless_shadow.identification.result.flux_pu = 0.40f;
    g_sensorless_shadow.identification.result.quality = 850U;
    g_sensorless_shadow.identification.result.valid = 1U;
    g_sensorless_shadow.identification.state = MOTOR_IDENT_COMPLETE;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_IDENT_PASS,
               "completed parameter identification was not marked PASS");
    Test_Check(g_sensorless_production.completed_identifications == 1U,
               "identification completion counter was not updated");
    Test_Check(g_sensorless_production.identification_result.valid != 0U,
               "identified result was not published to the mailbox");

    g_sensorless_production.command =
        SENSORLESS_PROD_CMD_APPLY_IDENTIFIED_MOTOR;
    g_sensorless_production.request_seq = 2U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_SERVO_NOT_READY,
               "identified parameters were applied while servo was enabled");

    StateMachine.RegulFlg = 0U;
    g_sensorless_production.request_seq = 3U;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result
               == SENSORLESS_PROD_RESULT_ACCEPTED,
               "identified parameters were not applied at Servo OFF");
    Test_Check(g_ident_apply_calls == 2U,
               "apply-identification API call count is incorrect");
}

static void Test_SequenceAndRejectedWrites(void)
{
    unsigned i, step;
    SENSORLESS_TEST_POINT point = {1,0,500,50,3,1000,{0,2,4,1,100,1}};
    Test_PrepareRuntime();
    StateMachine.RegulFlg = 1U;
    g_sensorless_production.sequence.count = 2;
    g_sensorless_production.sequence.points[0] = point;
    point.direction = -1; point.target_speed_rpm = 600;
    g_sensorless_production.sequence.points[1] = point;
    g_sensorless_production.arm_key = SENSORLESS_PRODUCTION_ARM_KEY;
    g_sensorless_production.command = SENSORLESS_PROD_CMD_RUN_SEQUENCE;
    g_sensorless_production.request_seq = 1;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.sequence_active, "sequence did not start");
    for (step=0; step<2; ++step) {
        g_sensorless_shadow.startup.state = SENSORLESS_START_RUN;
        g_sensorless_diag.observer_speed_rpm = step ? -600 : 500;
        if (!step) {
            /* Cannot retarget an armed production sequence or mutate its recipe. */
            g_sensorless_production.sequence.points[1].target_speed_rpm = 9999;
            g_sensorless_production.command = SENSORLESS_PROD_CMD_UPDATE_TARGET;
            g_sensorless_production.request_seq = 2;
            SensorlessProduction_Service1ms();
            Test_Check(g_sensorless_production.command_result == SENSORLESS_PROD_RESULT_BUSY,
                       "active sequence accepted retarget");
        }
        for(i=0;i<205;i++) {
            ProductionMetrics_Sample(0,0,step ? -600.0f : 500.0f,48);
            SensorlessProduction_Service1ms();
        }
        g_sensorless_shadow.startup.state = SENSORLESS_START_IDLE;
        SensorlessProduction_Service1ms();
    }
    Test_Check(!g_sensorless_production.sequence_active
        && g_sensorless_production.sequence_completed==2
        && g_sensorless_production.sequence_result==SENSORLESS_PROD_RESULT_TEST_PASS,
        "two-point frozen sequence did not pass");
    Test_Check(g_start_calls==2, "sequence did not run exactly two points");
    Test_Check(g_sensorless_production.sequence_results[0].samples>0
        && g_sensorless_production.sequence_results[1].samples>0,
        "sequence did not retain per-point measurements");

    Test_PrepareRuntime();
    g_source_foc_ready = 0;
    StateMachine.RegulFlg=1;
    g_sensorless_production.command=SENSORLESS_PROD_CMD_RUN_TEST;
    g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
    g_sensorless_production.request_seq=1;
    SensorlessProduction_Service1ms();
    Test_Check(!g_start_calls, "missing source recipe started motion");
    g_source_foc_ready=1;
    g_sensorless_production.abi_version=0x00060000;
    g_sensorless_production.request_seq=2;
    SensorlessProduction_Service1ms();
    Test_Check(!g_start_calls, "old ABI started motion");
}

static void Test_DebugModeReadbackAndFailures(void)
{
    Test_PrepareRuntime();
    g_sensorless_production.command=SENSORLESS_PROD_CMD_APPLY_TUNING;
    g_sensorless_production.tuning.mask=SENSORLESS_TUNING_DEBUG;
    g_sensorless_production.tuning.debug_mode=SENSORLESS_DEBUG_IF;
    g_sensorless_production.tuning.current_ramp_ms=50;
    g_sensorless_production.tuning.if_hold_ms=500;
    g_sensorless_production.tuning.iq_hold_ms=100;
    g_sensorless_production.tuning.ramp_timeout_ms=2000;
    g_sensorless_production.tuning.total_timeout_ms=5000;
    g_sensorless_production.tuning.run_test_ms=1000;
    g_sensorless_production.request_seq=1;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.command_result==SENSORLESS_PROD_RESULT_ACCEPTED,
        "debug tuning rejected");
    Test_Check(g_sensorless_production.effective_startup.debug_mode==SENSORLESS_DEBUG_IF
        && g_sensorless_production.effective_startup.if_hold_cycles==500,
        "effective debug parameters not read back");
    Test_Check(g_sensorless_production.firmware_version==SENSORLESS_FIRMWARE_VERSION
        && !(g_sensorless_production.parameter_sequence & 1U), "invalid readback version");
    g_sensorless_shadow.startup.failure.fault=SENSORLESS_FAULT_CURRENT_OVER;
    g_sensorless_shadow.startup.failure.state=SENSORLESS_START_BLEND;
    g_sensorless_diag.current_failure=SOURCE_FOC_OVER_CURRENT;
    SensorlessProduction_Service1ms();
    Test_Check(g_sensorless_production.first_failure.fault==SENSORLESS_FAULT_CURRENT_OVER
        && g_sensorless_production.current_failure==SOURCE_FOC_OVER_CURRENT,
        "classified failure not published");
}
int main(void)
{
    Test_DebugModeReadbackAndFailures();
    Test_SequenceAndRejectedWrites();
    Test_ArmingAndAutomaticRun();
    Test_TimeoutStopIsIssuedOnce();
    Test_TuningAndFaultMapping();
    Test_HfiPoleTuningAndStatus();
    Test_SmoTuningAndStatus();
    Test_ParameterIdentificationCommands();

    if (g_failures == 0) {
        printf("PASS: sensorless production interface tests\n");
        return 0;
    }
    printf("FAILURES: %d\n", g_failures);
    return 1;
}
