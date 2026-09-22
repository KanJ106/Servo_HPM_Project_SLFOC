#include "SensorlessProduction.h"

#include <math.h>
#include <string.h>

#include "SensorlessShadow.h"
#include "SV_FaultProtect.h"
#include "SV_StateMachine.h"

#define SENSORLESS_PROD_DEFAULT_HOLD_MS       (2000UL)
#define SENSORLESS_PROD_DEFAULT_TIMEOUT_MS    (10000UL)
#define SENSORLESS_PROD_DEFAULT_TOLERANCE_RPM (100)
#define SENSORLESS_PROD_STABLE_REQUIRED_MS    (200UL)

volatile SENSORLESS_PRODUCTION_MAILBOX g_sensorless_production;

static uint32_t g_sensorless_last_request_seq;
static uint16_t g_sensorless_test_active;
static uint16_t g_sensorless_test_stop_result;
static uint16_t g_sensorless_ident_active;
static PRODUCTION_METRIC_LIMITS g_test_metric_limits;
static int32_t g_test_target, g_test_tolerance;
static int16_t g_test_direction;
static uint32_t g_test_hold_ms, g_test_timeout_ms;
static SENSORLESS_TEST_SEQUENCE g_sequence;
static uint16_t g_sequence_active, g_sequence_index;


static int32_t SensorlessProduction_Abs32(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (value < 0) ? -value : value;
}

static uint32_t SensorlessProduction_MsToCycles(uint32_t milliseconds,
                                                float sample_time_s)
{
    float cycles = ((float)milliseconds * 0.001f) / sample_time_s;

    if (cycles < 1.0f) {
        return 1U;
    }
    if (cycles > 4294967040.0f) {
        return 0xffffffffUL;
    }
    return (uint32_t)floorf(cycles + 0.5f); /* nearest PWM cycle, not float truncation */
}

static uint16_t SensorlessProduction_MsToU16Cycles(
    uint32_t milliseconds, float sample_time_s)
{
    uint32_t cycles = SensorlessProduction_MsToCycles(
        milliseconds, sample_time_s);

    return (uint16_t)((cycles > 65535U) ? 65535U : cycles);
}

static void SensorlessProduction_ApplyHfiFields(
    HFI_ESTIMATOR_PARAMS *params)
{
    const volatile SENSORLESS_PRODUCTION_TUNING *tuning =
        &g_sensorless_production.tuning;

    if ((tuning->mask & SENSORLESS_TUNING_HFI) == 0U) {
        return;
    }
    params->injection_frequency_hz = tuning->hfi_injection_frequency_hz;
    params->injection_current_pu = tuning->hfi_injection_current_pu;
    params->response_lpf_alpha = tuning->hfi_response_lpf_alpha;
    params->speed_lpf_alpha = tuning->hfi_speed_lpf_alpha;
    params->tracking_kp = tuning->hfi_tracking_kp;
    params->tracking_ki = tuning->hfi_tracking_ki;
    params->min_response_pu = tuning->hfi_min_response_pu;
    params->lock_error = tuning->hfi_lock_error;
    params->lock_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->hfi_estimator_lock_ms, params->sample_time_s);
    params->unlock_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->hfi_estimator_unlock_ms, params->sample_time_s);
    params->pole_detect_current_pu =
        tuning->hfi_pole_detect_current_pu;
    params->pole_min_contrast = tuning->hfi_pole_min_contrast;
    params->pole_response_polarity =
        tuning->hfi_pole_response_polarity;
    params->pole_settle_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->hfi_pole_settle_ms, params->sample_time_s);
    params->pole_measure_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->hfi_pole_measure_ms, params->sample_time_s);
}

static void SensorlessProduction_ApplySmoFields(
    SMO_PLL_PARAMS *params)
{
    const volatile SENSORLESS_PRODUCTION_TUNING *tuning =
        &g_sensorless_production.tuning;

    if ((tuning->mask & SENSORLESS_TUNING_SMO) == 0U) {
        return;
    }
    params->switching_gain = tuning->smo_switching_gain;
    params->boundary_pu = tuning->smo_boundary_pu;
    params->emf_lpf_alpha = tuning->smo_emf_lpf_alpha;
    params->emf_delay_comp_gain =
        tuning->smo_emf_delay_comp_gain;
    params->pll_kp = tuning->smo_pll_kp;
    params->pll_ki = tuning->smo_pll_ki;
    params->pll_speed_lpf_alpha =
        tuning->smo_pll_speed_lpf_alpha;
    params->weak_emf_speed_decay =
        tuning->smo_weak_emf_speed_decay;
    params->min_emf_pu = tuning->smo_min_emf_pu;
    params->unlock_emf_ratio =
        tuning->smo_unlock_emf_ratio;
    params->lock_phase_error =
        tuning->smo_lock_phase_error;
    params->max_accel_e_rad_s2 =
        tuning->smo_max_accel_e_rad_s2;
    params->current_hat_limit_pu =
        tuning->smo_current_hat_limit_pu;
    params->lock_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->smo_lock_ms, params->sample_time_s);
    params->unlock_cycles = SensorlessProduction_MsToU16Cycles(
        tuning->smo_unlock_ms, params->sample_time_s);
}

static int32_t SensorlessProduction_DegToQ24(int32_t degrees)
{
    if (degrees < 0) {
        degrees = -degrees;
    }
    if (degrees > 180) {
        degrees = 180;
    }
    return (int32_t)(((int64_t)degrees * ROTOR_ANGLE_ONE_TURN_Q24) / 360);
}

static void SensorlessProduction_CopyMotorOverride(
    SENSORLESS_MOTOR_OVERRIDE *destination)
{
    destination->mask = g_sensorless_production.motor.mask;
    destination->stator_resistance_pu =
        g_sensorless_production.motor.stator_resistance_pu;
    destination->ld_pu = g_sensorless_production.motor.ld_pu;
    destination->lq_pu = g_sensorless_production.motor.lq_pu;
    destination->max_speed_rpm = g_sensorless_production.motor.max_speed_rpm;
    destination->pole_pairs = g_sensorless_production.motor.pole_pairs;
    destination->rated_current_pu =
        g_sensorless_production.motor.rated_current_pu;
    destination->current_limit_pu =
        g_sensorless_production.motor.current_limit_pu;
}

static void SensorlessProduction_ApplyTuningFields(
    SENSORLESS_STARTUP_CONFIG *config)
{
    const volatile SENSORLESS_PRODUCTION_TUNING *tuning =
        &g_sensorless_production.tuning;

    if ((tuning->mask & SENSORLESS_TUNING_DEBUG) != 0U) {
        config->debug_mode = tuning->debug_mode;
        config->current_ramp_cycles = SensorlessProduction_MsToCycles(tuning->current_ramp_ms, config->sample_time_s);
        config->if_hold_cycles = SensorlessProduction_MsToCycles(tuning->if_hold_ms, config->sample_time_s);
        config->iq_hold_cycles = SensorlessProduction_MsToCycles(tuning->iq_hold_ms, config->sample_time_s);
        config->ramp_timeout_cycles = SensorlessProduction_MsToCycles(tuning->ramp_timeout_ms, config->sample_time_s);
        config->total_timeout_cycles = SensorlessProduction_MsToCycles(tuning->total_timeout_ms, config->sample_time_s);
        config->run_test_cycles = SensorlessProduction_MsToCycles(tuning->run_test_ms, config->sample_time_s);
    }
    if ((tuning->mask & SENSORLESS_TUNING_ALIGN) != 0U) {
        config->align_angle_turns = tuning->align_angle_turns;
        config->align_current_pu = tuning->align_current_pu;
    }
    if ((tuning->mask & SENSORLESS_TUNING_CURRENT) != 0U) {
        config->startup_iq_pu = tuning->startup_iq_pu;
        config->iq_limit_pu = tuning->iq_limit_pu;
    }
    if ((tuning->mask & SENSORLESS_TUNING_SPEED) != 0U) {
        config->handoff_speed_rpm = tuning->handoff_speed_rpm;
        config->sensorless_exit_speed_rpm =
            tuning->sensorless_exit_speed_rpm;
        config->open_loop_accel_rpm_s = tuning->open_loop_accel_rpm_s;
        config->run_accel_rpm_s = tuning->run_accel_rpm_s;
        config->stop_speed_rpm = tuning->stop_speed_rpm;
    }
    if ((tuning->mask & SENSORLESS_TUNING_PI) != 0U) {
        config->speed_kp = tuning->speed_kp;
        config->speed_ki = tuning->speed_ki;
    }
    if ((tuning->mask & SENSORLESS_TUNING_TIMING) != 0U) {
        config->align_cycles = SensorlessProduction_MsToCycles(
            tuning->align_ms, config->sample_time_s);
        config->lock_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->lock_dwell_ms, config->sample_time_s);
        config->blend_cycles = SensorlessProduction_MsToCycles(
            tuning->blend_ms, config->sample_time_s);
        config->lock_timeout_cycles = SensorlessProduction_MsToCycles(
            tuning->lock_timeout_ms, config->sample_time_s);
        config->hfi_lock_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->hfi_lock_dwell_ms, config->sample_time_s);
        config->hfi_timeout_cycles = SensorlessProduction_MsToCycles(
            tuning->hfi_timeout_ms, config->sample_time_s);
        config->unlock_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->unlock_dwell_ms, config->sample_time_s);
        config->monitor_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->monitor_dwell_ms, config->sample_time_s);
        config->fallback_ready_cycles = SensorlessProduction_MsToCycles(
            tuning->fallback_ready_ms, config->sample_time_s);
        config->stop_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->stop_dwell_ms, config->sample_time_s);
        config->stop_timeout_cycles = SensorlessProduction_MsToCycles(
            tuning->stop_timeout_ms, config->sample_time_s);
    }
    if ((tuning->mask & SENSORLESS_TUNING_LOCK) != 0U) {
        config->minimum_lock_quality = tuning->minimum_lock_quality;
        config->minimum_hfi_quality = tuning->minimum_hfi_quality;
    }
    if ((tuning->mask & SENSORLESS_TUNING_MONITOR) != 0U) {
        config->fallback_max_angle_error_q24 = SensorlessProduction_DegToQ24(
            tuning->fallback_max_angle_error_deg);
        config->monitor_max_angle_error_q24 = SensorlessProduction_DegToQ24(
            tuning->monitor_max_angle_error_deg);
        config->monitor_max_speed_error_rpm =
            tuning->monitor_max_speed_error_rpm;
    }
    if ((tuning->mask & SENSORLESS_TUNING_HFI) != 0U) {
        config->sensorless_exit_speed_rpm =
            tuning->sensorless_exit_speed_rpm;
        config->minimum_hfi_quality = tuning->minimum_hfi_quality;
        config->hfi_enable = tuning->hfi_enable;
        config->allow_open_loop_fallback =
            tuning->allow_open_loop_fallback;
        config->hfi_lock_dwell_cycles = SensorlessProduction_MsToCycles(
            tuning->hfi_lock_dwell_ms, config->sample_time_s);
        config->hfi_timeout_cycles = SensorlessProduction_MsToCycles(
            tuning->hfi_timeout_ms, config->sample_time_s);
    }
}

static uint16_t SensorlessProduction_Start(uint16_t automatic_test)
{
    int32_t target = SensorlessProduction_Abs32(
        g_sensorless_production.target_speed_rpm);
    float handoff = g_sensorless_shadow.startup.config.handoff_speed_rpm;
    float maximum = g_sensorless_shadow.motor_profile.effective.max_speed_rpm;

    if (automatic_test && g_sensorless_shadow.startup.config.debug_mode != SENSORLESS_DEBUG_FULL)
        return SENSORLESS_PROD_RESULT_INVALID_TUNING;
    if (g_sensorless_test_active) return SENSORLESS_PROD_RESULT_BUSY;
    if (!g_source_foc_ready) return SENSORLESS_PROD_RESULT_INVALID_PROFILE;
    if (automatic_test) {
        PRODUCTION_METRIC_LIMITS limits = g_sensorless_production.metric_limits;
        if (!ProductionMetrics_Validate(&limits))
            return SENSORLESS_PROD_RESULT_INVALID_TUNING;
        g_test_metric_limits = limits;
    }
    if (g_sensorless_production.arm_key != SENSORLESS_PRODUCTION_ARM_KEY) {
        return SENSORLESS_PROD_RESULT_BAD_ARM_KEY;
    }
    if (g_sensorless_ident_active != 0U) {
        return SENSORLESS_PROD_RESULT_BUSY;
    }
    if ((StateMachine.RegulFlg == 0U)
        || ((FaultP.FaultStatus & FP_ERR) != 0U)) {
        return SENSORLESS_PROD_RESULT_SERVO_NOT_READY;
    }
    if (g_sensorless_shadow.motor_profile.valid_for_control == 0U) {
        return SENSORLESS_PROD_RESULT_INVALID_PROFILE;
    }
    if ((float)target > maximum) {
        return SENSORLESS_PROD_RESULT_INVALID_TUNING;
    }
    if (((float)target < handoff)
        && ((g_sensorless_shadow.startup.config.hfi_enable == 0U)
            || (g_sensorless_shadow.motor_profile.hfi_capable == 0U))) {
        return SENSORLESS_PROD_RESULT_INVALID_PROFILE;
    }
    if (SensorlessShadow_RequestStart(g_sensorless_production.direction,
                                      target) == 0U) {
        return SENSORLESS_PROD_RESULT_BUSY;
    }

    g_sensorless_production.arm_key = 0U;
    if (automatic_test != 0U) {
        ProductionMetrics_Cancel();
        memset((void *)&g_sensorless_production.metric_result, 0,
               sizeof(g_sensorless_production.metric_result));
        g_test_target = target;
        g_test_direction = g_sensorless_production.direction;
        g_test_tolerance = SensorlessProduction_Abs32(g_sensorless_production.speed_tolerance_rpm);
        g_test_hold_ms = g_sensorless_production.test_hold_ms;
        g_test_timeout_ms = g_sensorless_production.test_timeout_ms;
        g_sensorless_test_active = 1U;
        g_sensorless_test_stop_result = SENSORLESS_PROD_RESULT_TEST_PASS;
        g_sensorless_production.production_state =
            SENSORLESS_PROD_STATE_STARTING;
        g_sensorless_production.test_elapsed_ms = 0U;
        g_sensorless_production.test_stable_ms = 0U;
        g_sensorless_production.test_hold_elapsed_ms = 0U;
        return SENSORLESS_PROD_RESULT_TEST_RUNNING;
    }
    return SENSORLESS_PROD_RESULT_ACCEPTED;
}

static uint16_t SensorlessProduction_StartSequencePoint(void)
{
    const SENSORLESS_TEST_POINT *point = &g_sequence.points[g_sequence_index];
    g_sensorless_production.direction = point->direction;
    g_sensorless_production.target_speed_rpm = point->target_speed_rpm;
    g_sensorless_production.speed_tolerance_rpm = point->speed_tolerance_rpm;
    g_sensorless_production.test_hold_ms = point->hold_ms;
    g_sensorless_production.test_timeout_ms = point->timeout_ms;
    g_sensorless_production.metric_limits = point->limits;
    g_sensorless_production.arm_key = SENSORLESS_PRODUCTION_ARM_KEY;
    return SensorlessProduction_Start(1U);
}

static uint16_t SensorlessProduction_StartSequence(void)
{
    SENSORLESS_TEST_SEQUENCE candidate = g_sensorless_production.sequence;
    uint16_t i, result;
    if (g_sensorless_test_active || g_sequence_active) return SENSORLESS_PROD_RESULT_BUSY;
    if (g_sensorless_production.arm_key != SENSORLESS_PRODUCTION_ARM_KEY)
        return SENSORLESS_PROD_RESULT_BAD_ARM_KEY;
    if (!g_source_foc_ready) return SENSORLESS_PROD_RESULT_INVALID_PROFILE;
    if (!candidate.count || candidate.count > SENSORLESS_SEQUENCE_MAX_POINTS)
        return SENSORLESS_PROD_RESULT_INVALID_TUNING;
    for (i=0; i<candidate.count; ++i) {
        const SENSORLESS_TEST_POINT *p = &candidate.points[i];
        if ((p->direction != 1 && p->direction != -1)
            || p->target_speed_rpm < g_sensorless_shadow.startup.config.handoff_speed_rpm
            || p->target_speed_rpm > g_sensorless_shadow.motor_profile.effective.max_speed_rpm
            || p->speed_tolerance_rpm <= 0 || !p->hold_ms || p->hold_ms > 60000U
            || p->timeout_ms <= p->hold_ms + SENSORLESS_PROD_STABLE_REQUIRED_MS
            || p->timeout_ms > 120000U
            || !ProductionMetrics_Validate(&p->limits))
            return SENSORLESS_PROD_RESULT_INVALID_TUNING;
    }
    g_sequence = candidate;
    g_sequence_index = 0U;
    result = SensorlessProduction_StartSequencePoint();
    if (result != SENSORLESS_PROD_RESULT_TEST_RUNNING) return result;
    memset((void *)g_sensorless_production.sequence_results,0,
           sizeof(g_sensorless_production.sequence_results));
    g_sensorless_production.sequence_completed = 0U;
    g_sensorless_production.sequence_result = SENSORLESS_PROD_RESULT_TEST_RUNNING;
    g_sequence_active = 1U;
    return result;
}

static void SensorlessProduction_RunSequence(void)
{
    uint16_t result;
    if (!g_sequence_active || g_sensorless_test_active) return;
    result = g_sensorless_production.command_result;
    g_sensorless_production.sequence_results[g_sequence_index] =
        g_sensorless_production.metric_result;
    if (result != SENSORLESS_PROD_RESULT_TEST_PASS) {
        g_sequence_active = 0U;
        g_sensorless_production.sequence_result = result;
        return;
    }
    g_sensorless_production.sequence_completed++;
    if ((uint16_t)(g_sequence_index + 1U) >= g_sequence.count) {
        g_sequence_active = 0U;
        g_sensorless_production.sequence_result = SENSORLESS_PROD_RESULT_TEST_PASS;
        return;
    }
    g_sequence_index++;
    result = SensorlessProduction_StartSequencePoint();
    g_sensorless_production.command_result = result;
    if (result != SENSORLESS_PROD_RESULT_TEST_RUNNING) {
        g_sequence_active = 0U;
        g_sensorless_production.sequence_result = result;
    }
}

static void SensorlessProduction_ProcessCommand(void)
{
    SENSORLESS_MOTOR_OVERRIDE motor_override;
    SENSORLESS_STARTUP_CONFIG startup_config;
    HFI_ESTIMATOR_PARAMS hfi_params;
    SMO_PLL_PARAMS smo_params;
    MOTOR_PARAM_IDENT_CONFIG ident_config;
    uint16_t result = SENSORLESS_PROD_RESULT_INVALID_COMMAND;

    if (g_sensorless_production.abi_version != SENSORLESS_PRODUCTION_ABI_VERSION) {
        g_sensorless_production.command_result = SENSORLESS_PROD_RESULT_INVALID_COMMAND;
        g_sensorless_production.acknowledge_seq = g_sensorless_production.request_seq;
        g_sensorless_last_request_seq = g_sensorless_production.request_seq;
        return;
    }
    if ((g_sensorless_test_active || g_sequence_active) && g_sensorless_production.command != SENSORLESS_PROD_CMD_STOP) {
        g_sensorless_production.command_result = SENSORLESS_PROD_RESULT_BUSY;
        g_sensorless_production.acknowledge_seq = g_sensorless_production.request_seq;
        g_sensorless_last_request_seq = g_sensorless_production.request_seq;
        return;
    }
    switch (g_sensorless_production.command) {
    case SENSORLESS_PROD_CMD_RUN_SEQUENCE:
        result = SensorlessProduction_StartSequence();
        break;
    case SENSORLESS_PROD_CMD_APPLY_SOURCE_RECIPE:
        if (g_sensorless_test_active || g_sensorless_ident_active) {
            result = SENSORLESS_PROD_RESULT_BUSY;
        } else {
            SOURCE_FOC_RECIPE recipe = g_sensorless_production.source_recipe;
            result = SensorlessShadow_ApplySourceRecipe(&recipe)
                ? SENSORLESS_PROD_RESULT_ACCEPTED : SENSORLESS_PROD_RESULT_INVALID_PROFILE;
        }
        break;
    case SENSORLESS_PROD_CMD_NONE:
        result = SENSORLESS_PROD_RESULT_IDLE;
        break;

    case SENSORLESS_PROD_CMD_APPLY_MOTOR:
        if ((g_sensorless_test_active != 0U)
            || (g_sensorless_ident_active != 0U)) {
            result = SENSORLESS_PROD_RESULT_BUSY;
            break;
        }
        SensorlessProduction_CopyMotorOverride(&motor_override);
        result = (SensorlessShadow_ConfigureMotorOverride(&motor_override) != 0U)
            ? SENSORLESS_PROD_RESULT_ACCEPTED
            : SENSORLESS_PROD_RESULT_INVALID_PROFILE;
        break;

    case SENSORLESS_PROD_CMD_APPLY_TUNING:
        if ((g_sensorless_test_active != 0U)
            || (g_sensorless_ident_active != 0U)) {
            result = SENSORLESS_PROD_RESULT_BUSY;
            break;
        }
        if ((g_sensorless_production.tuning.mask & SENSORLESS_TUNING_HFI) != 0U) {
            result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
            break;
        }
        startup_config = g_sensorless_shadow.startup.config;
        hfi_params = g_sensorless_shadow.hfi_params;
        smo_params = g_sensorless_shadow.params;
        SensorlessProduction_ApplyTuningFields(&startup_config);
        SensorlessProduction_ApplyHfiFields(&hfi_params);
        SensorlessProduction_ApplySmoFields(&smo_params);
        if ((SensorlessStartup_ValidateConfig(&startup_config) == 0U)
            || (((g_sensorless_production.tuning.mask
                    & SENSORLESS_TUNING_HFI) != 0U)
                && (HfiEstimator_ValidateParams(&hfi_params) == 0U))
            || (((g_sensorless_production.tuning.mask
                    & SENSORLESS_TUNING_SMO) != 0U)
                && (SmoPll_ValidateParams(&smo_params) == 0U))) {
            result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
        } else if (SensorlessShadow_ConfigureStartup(&startup_config) == 0U) {
            result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
        } else if (((g_sensorless_production.tuning.mask
                    & SENSORLESS_TUNING_HFI) != 0U)
            && (SensorlessShadow_ConfigureHfi(&hfi_params) == 0U)) {
            result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
        } else if (((g_sensorless_production.tuning.mask
                    & SENSORLESS_TUNING_SMO) != 0U)
            && (SensorlessShadow_ConfigureSmo(&smo_params) == 0U)) {
            result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
        } else {
            result = SENSORLESS_PROD_RESULT_ACCEPTED;
        }
        break;

    case SENSORLESS_PROD_CMD_START:
        result = SensorlessProduction_Start(0U);
        break;

    case SENSORLESS_PROD_CMD_RUN_TEST:
        result = SensorlessProduction_Start(1U);
        break;

    case SENSORLESS_PROD_CMD_STOP:
        if (g_sequence_active) {
            g_sequence_active = 0U;
            g_sensorless_production.sequence_result = SENSORLESS_PROD_RESULT_TEST_FAIL;
        }
        ProductionMetrics_Cancel();
        SensorlessShadow_AbortParamIdentification();
        SensorlessShadow_RequestStop();
        g_sensorless_production.arm_key = 0U;
        if (g_sensorless_test_active != 0U) {
            g_sensorless_test_stop_result = SENSORLESS_PROD_RESULT_TEST_FAIL;
            g_sensorless_production.production_state =
                SENSORLESS_PROD_STATE_STOPPING;
        }
        result = SENSORLESS_PROD_RESULT_ACCEPTED;
        break;

    case SENSORLESS_PROD_CMD_CLEAR_FAULT:
        if (StateMachine.RegulFlg != 0U) {
            result = SENSORLESS_PROD_RESULT_SERVO_NOT_READY;
        } else {
            SensorlessShadow_ClearFault();
            g_sensorless_test_active = 0U;
            g_sensorless_production.production_state =
                SENSORLESS_PROD_STATE_IDLE;
            result = SENSORLESS_PROD_RESULT_ACCEPTED;
        }
        break;

    case SENSORLESS_PROD_CMD_UPDATE_TARGET:
        if (g_sensorless_production.arm_key
            != SENSORLESS_PRODUCTION_ARM_KEY) {
            result = SENSORLESS_PROD_RESULT_BAD_ARM_KEY;
        } else {
            result = (SensorlessShadow_UpdateTarget(
                g_sensorless_production.direction,
                SensorlessProduction_Abs32(
                    g_sensorless_production.target_speed_rpm)) != 0U)
                ? SENSORLESS_PROD_RESULT_ACCEPTED
                : SENSORLESS_PROD_RESULT_BUSY;
        }
        break;

    case SENSORLESS_PROD_CMD_RESTORE_DEFAULTS:
        if ((g_sensorless_test_active != 0U)
            || (g_sensorless_ident_active != 0U)) {
            result = SENSORLESS_PROD_RESULT_BUSY;
            break;
        }
        result = (SensorlessShadow_ConfigureMotorOverride(0) != 0U)
            ? SENSORLESS_PROD_RESULT_ACCEPTED
            : SENSORLESS_PROD_RESULT_INVALID_PROFILE;
        break;

    case SENSORLESS_PROD_CMD_RUN_IDENTIFICATION:
        if (g_sensorless_production.arm_key
            != SENSORLESS_PRODUCTION_ARM_KEY) {
            result = SENSORLESS_PROD_RESULT_BAD_ARM_KEY;
        } else if ((g_sensorless_test_active != 0U)
            || (g_sensorless_ident_active != 0U)) {
            result = SENSORLESS_PROD_RESULT_BUSY;
        } else if ((StateMachine.RegulFlg == 0U)
            || ((FaultP.FaultStatus & FP_ERR) != 0U)) {
            result = SENSORLESS_PROD_RESULT_SERVO_NOT_READY;
        } else {
            ident_config = g_sensorless_production.identification_config;
            if (SensorlessShadow_StartParamIdentification(
                    &ident_config) == 0U) {
                result = SENSORLESS_PROD_RESULT_INVALID_TUNING;
            } else {
                g_sensorless_ident_active = 1U;
                result = SENSORLESS_PROD_RESULT_IDENT_RUNNING;
            }
        }
        break;

    case SENSORLESS_PROD_CMD_APPLY_IDENTIFIED_MOTOR:
        if ((g_sensorless_test_active != 0U)
            || (g_sensorless_ident_active != 0U)) {
            result = SENSORLESS_PROD_RESULT_BUSY;
        } else {
            result = (SensorlessShadow_ApplyIdentifiedMotor() != 0U)
                ? SENSORLESS_PROD_RESULT_ACCEPTED
                : SENSORLESS_PROD_RESULT_SERVO_NOT_READY;
        }
        break;

    case SENSORLESS_PROD_CMD_ABORT_IDENTIFICATION:
        SensorlessShadow_AbortParamIdentification();
        g_sensorless_production.arm_key = 0U;
        result = SENSORLESS_PROD_RESULT_ACCEPTED;
        break;

    default:
        result = SENSORLESS_PROD_RESULT_INVALID_COMMAND;
        break;
    }

    g_sensorless_production.command_result = result;
    g_sensorless_production.acknowledge_seq =
        g_sensorless_production.request_seq;
    g_sensorless_last_request_seq = g_sensorless_production.request_seq;
}

static uint16_t SensorlessProduction_AtTestPoint(void)
{
    uint16_t use_hfi = (uint16_t)
        ((g_sensorless_shadow.startup.state == SENSORLESS_START_HFI_RUN)
            ? 1U : 0U);
    int32_t feedback_speed = (use_hfi != 0U)
        ? g_sensorless_diag.hfi_speed_rpm
        : g_sensorless_diag.observer_speed_rpm;
    int32_t speed_error = feedback_speed - (g_test_direction * g_test_target);
    int32_t tolerance = g_test_tolerance;
    uint16_t locked = (use_hfi != 0U)
        ? g_sensorless_diag.hfi_locked : g_sensorless_diag.observer_locked;
    uint16_t quality = (use_hfi != 0U)
        ? g_sensorless_diag.hfi_quality : g_sensorless_diag.observer_quality;
    uint16_t minimum_quality = (use_hfi != 0U)
        ? g_sensorless_shadow.startup.config.minimum_hfi_quality
        : g_sensorless_shadow.startup.config.minimum_lock_quality;

    if (tolerance == 0) {
        tolerance = SENSORLESS_PROD_DEFAULT_TOLERANCE_RPM;
    }
    return (uint16_t)(((SensorlessProduction_Abs32(speed_error) <= tolerance)
        && (locked != 0U) && (quality >= minimum_quality)) ? 1U : 0U);
}

static void SensorlessProduction_RunAutomaticTest(void)
{
    uint32_t hold_ms = g_test_hold_ms;
    uint32_t timeout_ms = g_test_timeout_ms;

    if (g_sensorless_test_active == 0U) {
        return;
    }
    if (hold_ms == 0U) {
        hold_ms = SENSORLESS_PROD_DEFAULT_HOLD_MS;
    }
    if (timeout_ms == 0U) {
        timeout_ms = SENSORLESS_PROD_DEFAULT_TIMEOUT_MS;
    }
    if (g_sensorless_production.test_elapsed_ms < 0xffffffffUL) {
        g_sensorless_production.test_elapsed_ms++;
    }

    if ((g_sensorless_shadow.startup.state == SENSORLESS_START_FAULT)
        || ((FaultP.FaultStatus & FP_ERR) != 0U)) {
        PRODUCTION_METRIC_RESULT metrics;
        ProductionMetrics_Finish(&g_test_metric_limits, &metrics);
        g_sensorless_production.metric_result = metrics;
        g_sensorless_test_stop_result = SENSORLESS_PROD_RESULT_TEST_FAIL;
        g_sensorless_production.production_state = SENSORLESS_PROD_STATE_FAIL;
        g_sensorless_production.command_result = SENSORLESS_PROD_RESULT_TEST_FAIL;
        g_sensorless_production.failed_tests++;
        g_sensorless_test_active = 0U;
        return;
    }
    if ((g_sensorless_production.production_state
         != SENSORLESS_PROD_STATE_STOPPING)
        && (g_sensorless_production.test_elapsed_ms >= timeout_ms)) {
        SensorlessShadow_RequestStop();
        PRODUCTION_METRIC_RESULT metrics;
        ProductionMetrics_Finish(&g_test_metric_limits, &metrics);
        g_sensorless_production.metric_result = metrics;
        g_sensorless_test_stop_result = SENSORLESS_PROD_RESULT_TEST_TIMEOUT;
        g_sensorless_production.production_state =
            SENSORLESS_PROD_STATE_STOPPING;
    }

    switch (g_sensorless_production.production_state) {
    case SENSORLESS_PROD_STATE_STARTING:
        if ((g_sensorless_shadow.startup.state == SENSORLESS_START_RUN)
            || (g_sensorless_shadow.startup.state
                == SENSORLESS_START_HFI_RUN)) {
            g_sensorless_production.production_state =
                SENSORLESS_PROD_STATE_STABILIZING;
        }
        break;

    case SENSORLESS_PROD_STATE_STABILIZING:
        if (SensorlessProduction_AtTestPoint() != 0U) {
            g_sensorless_production.test_stable_ms++;
            if (g_sensorless_production.test_stable_ms
                >= SENSORLESS_PROD_STABLE_REQUIRED_MS) {
                ProductionMetrics_Begin((float)(g_test_direction * g_test_target));
                g_sensorless_production.production_state =
                    SENSORLESS_PROD_STATE_HOLDING;
            }
        } else {
            g_sensorless_production.test_stable_ms = 0U;
        }
        break;

    case SENSORLESS_PROD_STATE_HOLDING:
        if (SensorlessProduction_AtTestPoint() != 0U) {
            g_sensorless_production.test_hold_elapsed_ms++;
            if (g_sensorless_production.test_hold_elapsed_ms >= hold_ms) {
                PRODUCTION_METRIC_RESULT metrics;
                ProductionMetrics_Finish(&g_test_metric_limits, &metrics);
                g_sensorless_production.metric_result = metrics;
                SensorlessShadow_RequestStop();
                g_sensorless_test_stop_result = (metrics.failure_flags == 0U)
                    ? SENSORLESS_PROD_RESULT_TEST_PASS : SENSORLESS_PROD_RESULT_TEST_FAIL;
                g_sensorless_production.production_state =
                    SENSORLESS_PROD_STATE_STOPPING;
            }
        } else {
            ProductionMetrics_Cancel();
            g_sensorless_production.test_stable_ms = 0U;
            g_sensorless_production.test_hold_elapsed_ms = 0U;
            g_sensorless_production.production_state =
                SENSORLESS_PROD_STATE_STABILIZING;
        }
        break;

    case SENSORLESS_PROD_STATE_STOPPING:
        if (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) != 0U) {
            g_sensorless_test_active = 0U;
            g_sensorless_production.arm_key = 0U;
            if (g_sensorless_test_stop_result == SENSORLESS_PROD_RESULT_TEST_PASS) {
                g_sensorless_production.production_state =
                    SENSORLESS_PROD_STATE_PASS;
                g_sensorless_production.command_result =
                    SENSORLESS_PROD_RESULT_TEST_PASS;
                g_sensorless_production.completed_tests++;
            } else {
                g_sensorless_production.production_state =
                    SENSORLESS_PROD_STATE_FAIL;
                g_sensorless_production.command_result =
                    g_sensorless_test_stop_result;
                g_sensorless_production.failed_tests++;
            }
        }
        break;

    default:
        break;
    }
}

static void SensorlessProduction_RunIdentification(void)
{
    MOTOR_PARAM_IDENT_STATE state;

    if (g_sensorless_ident_active == 0U) {
        return;
    }
    state = g_sensorless_shadow.identification.state;
    if (state == MOTOR_IDENT_COMPLETE) {
        g_sensorless_ident_active = 0U;
        g_sensorless_production.arm_key = 0U;
        g_sensorless_production.command_result =
            SENSORLESS_PROD_RESULT_IDENT_PASS;
        g_sensorless_production.completed_identifications++;
    } else if (state == MOTOR_IDENT_FAILED) {
        g_sensorless_ident_active = 0U;
        g_sensorless_production.arm_key = 0U;
        g_sensorless_production.command_result =
            SENSORLESS_PROD_RESULT_IDENT_FAIL;
        g_sensorless_production.failed_identifications++;
    }
}

static void SensorlessProduction_UpdateStatus(void)
{
    g_sensorless_production.firmware_version = SENSORLESS_FIRMWARE_VERSION;
    g_sensorless_production.parameter_version = SENSORLESS_PARAMETER_VERSION;
    if (StateMachine.RegulFlg == 0U &&
        (memcmp((const void *)&g_sensorless_production.effective_recipe, &g_source_foc_recipe, sizeof(g_source_foc_recipe)) ||
         memcmp((const void *)&g_sensorless_production.effective_startup, &g_sensorless_shadow.startup.config, sizeof(g_sensorless_shadow.startup.config)) ||
         memcmp((const void *)&g_sensorless_production.effective_smo, &g_sensorless_shadow.params, sizeof(g_sensorless_shadow.params)) ||
         memcmp((const void *)&g_sensorless_production.effective_motor, &g_sensorless_shadow.motor_profile, sizeof(g_sensorless_shadow.motor_profile)) ||
         memcmp((const void *)&g_sensorless_production.effective_current, &g_source_foc_current, sizeof(g_source_foc_current)))) {
        g_sensorless_production.parameter_sequence++;
        g_sensorless_production.effective_recipe = g_source_foc_recipe;
        g_sensorless_production.effective_startup = g_sensorless_shadow.startup.config;
        g_sensorless_production.effective_smo = g_sensorless_shadow.params;
        g_sensorless_production.effective_motor = g_sensorless_shadow.motor_profile;
        g_sensorless_production.effective_current = g_source_foc_current;
        g_sensorless_production.parameter_sequence++;
    }
    g_sensorless_production.first_failure = g_sensorless_shadow.startup.failure;
    g_sensorless_production.current_failure = g_sensorless_diag.current_failure;
    g_sensorless_production.debug_completed = g_sensorless_shadow.startup.debug_completed;
    g_sensorless_production.sequence_active = g_sequence_active;
    g_sensorless_production.sequence_index = g_sequence_index;
    g_sensorless_production.source_foc_ready = g_source_foc_ready;
    g_sensorless_production.applied_recipe_id = g_source_foc_recipe.recipe_id;
    g_sensorless_production.applied_recipe_revision = g_source_foc_recipe.recipe_revision;
    int32_t active_speed =
        (g_sensorless_diag.control_source == ROTOR_SOURCE_HFI)
        ? g_sensorless_diag.hfi_speed_rpm
        : g_sensorless_diag.observer_speed_rpm;
    int32_t speed_error = active_speed
        - (g_sensorless_production.direction
            * SensorlessProduction_Abs32(
                g_sensorless_production.target_speed_rpm));

    g_sensorless_production.startup_state = g_sensorless_diag.startup_state;
    g_sensorless_production.control_source = g_sensorless_diag.control_source;
    g_sensorless_production.profile_valid =
        g_sensorless_diag.profile_valid_for_control;
    g_sensorless_production.observer_locked = g_sensorless_diag.observer_locked;
    g_sensorless_production.observer_quality = g_sensorless_diag.observer_quality;
    g_sensorless_production.encoder_valid = g_sensorless_diag.encoder_valid;
    g_sensorless_production.hfi_capable = g_sensorless_diag.hfi_capable;
    g_sensorless_production.hfi_active = g_sensorless_diag.hfi_active;
    g_sensorless_production.hfi_locked = g_sensorless_diag.hfi_locked;
    g_sensorless_production.hfi_quality = g_sensorless_diag.hfi_quality;
    g_sensorless_production.control_active =
        g_sensorless_diag.control_override_active;
    g_sensorless_production.startup_fault_flags =
        g_sensorless_diag.startup_fault_flags;
    g_sensorless_production.motor_validation_flags =
        g_sensorless_diag.motor_validation_flags;
    g_sensorless_production.encoder_speed_rpm =
        g_sensorless_diag.encoder_speed_rpm;
    g_sensorless_production.observer_speed_rpm =
        g_sensorless_diag.observer_speed_rpm;
    g_sensorless_production.hfi_speed_rpm =
        g_sensorless_diag.hfi_speed_rpm;
    g_sensorless_production.speed_error_rpm = speed_error;
    g_sensorless_production.theta_error_q24 =
        g_sensorless_diag.theta_error_q24;
    g_sensorless_production.hfi_theta_q24 =
        g_sensorless_diag.hfi_theta_q24;
    g_sensorless_production.hfi_injection_current_pu =
        g_sensorless_diag.hfi_injection_current_pu;
    g_sensorless_production.hfi_phase_error =
        g_sensorless_diag.hfi_phase_error;
    g_sensorless_production.saliency_ratio =
        g_sensorless_diag.saliency_ratio;
    g_sensorless_production.hfi_pole_detect_current_pu =
        g_sensorless_diag.hfi_pole_detect_current_pu;
    g_sensorless_production.hfi_pole_positive_response_pu =
        g_sensorless_diag.hfi_pole_positive_response_pu;
    g_sensorless_production.hfi_pole_negative_response_pu =
        g_sensorless_diag.hfi_pole_negative_response_pu;
    g_sensorless_production.hfi_pole_contrast =
        g_sensorless_diag.hfi_pole_contrast;
    g_sensorless_production.hfi_axis_locked =
        g_sensorless_diag.hfi_axis_locked;
    g_sensorless_production.hfi_pole_detected =
        g_sensorless_diag.hfi_pole_detected;
    g_sensorless_production.hfi_polarity_corrected =
        g_sensorless_diag.hfi_polarity_corrected;
    g_sensorless_production.hfi_pole_state =
        g_sensorless_diag.hfi_pole_state;
    g_sensorless_production.observer_raw_theta_q24 =
        g_sensorless_diag.observer_raw_theta_q24;
    g_sensorless_production.observer_phase_advance_turns =
        g_sensorless_diag.observer_phase_advance_turns;
    g_sensorless_production.observer_omega_tracking_e_rad_s =
        g_sensorless_diag.observer_omega_tracking_e_rad_s;
    g_sensorless_production.observer_emf_confidence =
        g_sensorless_diag.observer_emf_confidence;
    g_sensorless_production.observer_current_error_pu =
        g_sensorless_diag.observer_current_error_pu;
    g_sensorless_production.observer_weak_emf_counter =
        g_sensorless_diag.observer_weak_emf_counter;
    g_sensorless_production.identification_state =
        (uint16_t)g_sensorless_shadow.identification.state;
    g_sensorless_production.identification_active =
        MotorParamIdentification_IsBusy(
            &g_sensorless_shadow.identification);
    g_sensorless_production.identification_valid =
        g_sensorless_shadow.identification.result.valid;
    g_sensorless_production.identification_quality =
        g_sensorless_shadow.identification.result.quality;
    g_sensorless_production.identification_fault_flags =
        g_sensorless_shadow.identification.fault_flags;
    g_sensorless_production.identification_result =
        g_sensorless_shadow.identification.result;
}

void SensorlessProduction_Init(void)
{
    ProductionMetrics_Cancel();
    memset((void *)&g_sensorless_production, 0,
           sizeof(g_sensorless_production));
    g_sensorless_production.magic = SENSORLESS_PRODUCTION_MAGIC;
    g_sensorless_production.abi_version = SENSORLESS_PRODUCTION_ABI_VERSION;
    g_sensorless_production.command_result = SENSORLESS_PROD_RESULT_IDLE;
    g_sensorless_production.production_state = SENSORLESS_PROD_STATE_IDLE;
    g_sensorless_production.direction = 1;
    g_sensorless_production.test_hold_ms =
        SENSORLESS_PROD_DEFAULT_HOLD_MS;
    g_sensorless_production.test_timeout_ms =
        SENSORLESS_PROD_DEFAULT_TIMEOUT_MS;
    g_sensorless_production.speed_tolerance_rpm =
        SENSORLESS_PROD_DEFAULT_TOLERANCE_RPM;
    g_sequence_active = 0U;
    g_sequence_index = 0U;
    g_sensorless_last_request_seq = 0U;
    g_sensorless_test_active = 0U;
    g_sensorless_test_stop_result = SENSORLESS_PROD_RESULT_IDLE;
    g_sensorless_ident_active = 0U;
    g_sensorless_production.identification_config =
        g_sensorless_shadow.identification.config;
    SensorlessProduction_UpdateStatus();
}

void SensorlessProduction_Service1ms(void)
{
    if (g_sensorless_production.magic != SENSORLESS_PRODUCTION_MAGIC) {
        SensorlessShadow_RequestStop();
        SensorlessProduction_Init();
        return;
    }

    if (g_sensorless_production.request_seq != g_sensorless_last_request_seq) {
        SensorlessProduction_ProcessCommand();
    }
    SensorlessProduction_RunAutomaticTest();
    SensorlessProduction_RunSequence();
    SensorlessProduction_RunIdentification();
    SensorlessProduction_UpdateStatus();
}
