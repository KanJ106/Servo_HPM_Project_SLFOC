#include "MotorParamIdentification.h"

#include <math.h>
#include <string.h>

#define MOTOR_IDENT_TWO_PI_F          (6.2831853071795864769f)
#define MOTOR_IDENT_MIN_PARAM_PU      (0.000001f)
#define MOTOR_IDENT_MAX_PARAM_PU      (2.0f)
#define MOTOR_IDENT_MAX_FLUX_PU       (10.0f)

static float MotorIdent_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float MotorIdent_Min(float a, float b)
{
    return (a < b) ? a : b;
}

static uint16_t MotorIdent_InRange(float value, float minimum, float maximum)
{
    return (uint16_t)((isfinite(value) && (value >= minimum)
        && (value <= maximum)) ? 1U : 0U);
}

static void MotorIdent_ToDq(float alpha, float beta, float theta_turns,
                            float *d, float *q)
{
    float angle = MOTOR_IDENT_TWO_PI_F * theta_turns;
    float sine = sinf(angle);
    float cosine = cosf(angle);

    *d = alpha * cosine + beta * sine;
    *q = -alpha * sine + beta * cosine;
}

static void MotorIdent_ResetAccumulators(MOTOR_PARAM_IDENT_RUNTIME *runtime)
{
    runtime->sample_count = 0U;
    runtime->sum_current = 0.0f;
    runtime->sum_voltage = 0.0f;
    runtime->inductance_numerator = 0.0f;
    runtime->inductance_denominator = 0.0f;
    runtime->previous_axis_current = 0.0f;
    runtime->previous_axis_voltage = 0.0f;
    runtime->previous_d_current = 0.0f;
    runtime->previous_q_current = 0.0f;
    runtime->previous_sample_valid = 0U;
}

static void MotorIdent_Enter(MOTOR_PARAM_IDENT_RUNTIME *runtime,
                             MOTOR_PARAM_IDENT_STATE state)
{
    runtime->state = state;
    runtime->state_cycles = 0U;
    MotorIdent_ResetAccumulators(runtime);
}

static void MotorIdent_SetStationaryOutput(
    MOTOR_PARAM_IDENT_RUNTIME *runtime, float id_ref, float iq_ref)
{
    runtime->output.control_active = 1U;
    runtime->output.angle_override = 1U;
    runtime->output.current_override = 1U;
    runtime->output.theta_turns = runtime->captured_theta_turns;
    runtime->output.id_ref_pu = id_ref;
    runtime->output.iq_ref_pu = iq_ref;
}

static void MotorIdent_Fail(MOTOR_PARAM_IDENT_RUNTIME *runtime,
                            uint32_t fault_flags)
{
    runtime->fault_flags |= fault_flags;
    memset(&runtime->output, 0, sizeof(runtime->output));
    runtime->output.request_stop = 1U;
    runtime->state = MOTOR_IDENT_FAILED;
    runtime->result.valid = 0U;
}

static uint16_t MotorIdent_StationaryState(MOTOR_PARAM_IDENT_STATE state)
{
    return (uint16_t)(((state >= MOTOR_IDENT_RS_POS_SETTLE)
        && (state <= MOTOR_IDENT_ZERO_BEFORE_FLUX)) ? 1U : 0U);
}

void MotorParamIdentification_DefaultConfig(
    MOTOR_PARAM_IDENT_CONFIG *config,
    float sample_time_s,
    float base_electrical_rad_s,
    float pole_pairs,
    float voltage_base_v,
    float current_base_a,
    float current_limit_pu)
{
    float test_current;

    if (config == 0) {
        return;
    }
    memset(config, 0, sizeof(*config));
    test_current = 0.15f * current_limit_pu;
    if (test_current < 0.005f) {
        test_current = 0.005f;
    }
    if (test_current > 0.05f) {
        test_current = 0.05f;
    }
    config->sample_time_s = sample_time_s;
    config->base_electrical_rad_s = base_electrical_rad_s;
    config->pole_pairs = pole_pairs;
    config->voltage_base_v = voltage_base_v;
    config->current_base_a = current_base_a;
    config->current_limit_pu = current_limit_pu;
    config->resistance_current_pu = test_current;
    config->inductance_current_pu = test_current;
    config->flux_speed_rpm = 100.0f;
    config->flux_speed_tolerance_rpm = 20.0f;
    config->stationary_speed_limit_rpm = 30.0f;
    config->minimum_current_delta_pu = 0.00005f;
    config->minimum_flux_voltage_pu = 0.002f;
    config->minimum_hfi_quality = 600U;
    config->hfi_lock_cycles = (uint32_t)(0.050f / sample_time_s);
    config->settle_cycles = (uint32_t)(0.100f / sample_time_s);
    config->average_cycles = (uint32_t)(0.050f / sample_time_s);
    config->inductance_half_period_cycles = (uint32_t)(0.001f / sample_time_s);
    config->inductance_test_cycles = (uint32_t)(0.050f / sample_time_s);
    config->flux_settle_cycles = (uint32_t)(0.500f / sample_time_s);
    config->flux_average_cycles = (uint32_t)(0.500f / sample_time_s);
    config->timeout_cycles = (uint32_t)(15.0f / sample_time_s);
    if (config->hfi_lock_cycles == 0U) {
        config->hfi_lock_cycles = 1U;
    }
    if (config->settle_cycles == 0U) {
        config->settle_cycles = 1U;
    }
    if (config->average_cycles == 0U) {
        config->average_cycles = 1U;
    }
    if (config->inductance_half_period_cycles < 2U) {
        config->inductance_half_period_cycles = 2U;
    }
    if (config->inductance_test_cycles
        < (4U * config->inductance_half_period_cycles)) {
        config->inductance_test_cycles =
            4U * config->inductance_half_period_cycles;
    }
}

uint16_t MotorParamIdentification_ValidateConfig(
    const MOTOR_PARAM_IDENT_CONFIG *config)
{
    if ((config == 0)
        || (MotorIdent_InRange(config->sample_time_s,
                               0.000005f, 0.001f) == 0U)
        || (MotorIdent_InRange(config->base_electrical_rad_s,
                               1.0f, 100000.0f) == 0U)
        || (MotorIdent_InRange(config->pole_pairs, 1.0f, 64.0f) == 0U)
        || (MotorIdent_InRange(config->voltage_base_v, 1.0f, 1000.0f) == 0U)
        || (MotorIdent_InRange(config->current_base_a, 0.01f, 1000.0f) == 0U)
        || (MotorIdent_InRange(config->current_limit_pu,
                               0.001f, 4.0f) == 0U)
        || (MotorIdent_InRange(config->resistance_current_pu,
                               0.001f, 0.30f * config->current_limit_pu) == 0U)
        || (MotorIdent_InRange(config->inductance_current_pu,
                               0.001f, 0.30f * config->current_limit_pu) == 0U)
        || (MotorIdent_InRange(config->flux_speed_rpm, 10.0f, 1000.0f) == 0U)
        || (MotorIdent_InRange(config->flux_speed_tolerance_rpm,
                               1.0f, config->flux_speed_rpm) == 0U)
        || (MotorIdent_InRange(config->stationary_speed_limit_rpm,
                               1.0f, 200.0f) == 0U)
        || (MotorIdent_InRange(config->minimum_current_delta_pu,
                               0.000001f, 0.05f) == 0U)
        || (MotorIdent_InRange(config->minimum_flux_voltage_pu,
                               0.000001f, 0.5f) == 0U)
        || (config->minimum_hfi_quality > 1000U)
        || (config->hfi_lock_cycles == 0U)
        || (config->settle_cycles == 0U)
        || (config->average_cycles == 0U)
        || (config->inductance_half_period_cycles < 2U)
        || (config->inductance_test_cycles
            < (4U * config->inductance_half_period_cycles))
        || (config->flux_settle_cycles == 0U)
        || (config->flux_average_cycles < 4U)
        || (config->timeout_cycles == 0U)
        || (config->hfi_lock_cycles >= config->timeout_cycles)
        || (config->settle_cycles >= config->timeout_cycles)
        || (config->average_cycles >= config->timeout_cycles)
        || (config->inductance_test_cycles >= config->timeout_cycles)
        || (config->flux_settle_cycles >= config->timeout_cycles)
        || (config->flux_average_cycles >= config->timeout_cycles)) {
        return 0U;
    }
    return 1U;
}

void MotorParamIdentification_Init(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    const MOTOR_PARAM_IDENT_CONFIG *config)
{
    if (runtime == 0) {
        return;
    }
    memset(runtime, 0, sizeof(*runtime));
    if (config != 0) {
        runtime->config = *config;
    }
    runtime->state = MOTOR_IDENT_IDLE;
}

uint16_t MotorParamIdentification_Start(MOTOR_PARAM_IDENT_RUNTIME *runtime)
{
    MOTOR_PARAM_IDENT_CONFIG config;

    if ((runtime == 0)
        || (MotorParamIdentification_IsBusy(runtime) != 0U)
        || (MotorParamIdentification_ValidateConfig(&runtime->config) == 0U)) {
        return 0U;
    }
    config = runtime->config;
    MotorParamIdentification_Init(runtime, &config);
    runtime->state = MOTOR_IDENT_HFI_ACQUIRE;
    runtime->output.control_active = 1U;
    runtime->output.angle_override = 1U;
    runtime->output.current_override = 1U;
    runtime->output.requires_hfi = 1U;
    return 1U;
}

void MotorParamIdentification_Abort(MOTOR_PARAM_IDENT_RUNTIME *runtime,
                                    uint32_t fault_flags)
{
    if (runtime != 0) {
        MotorIdent_Fail(runtime, (fault_flags != 0U)
            ? fault_flags : MOTOR_IDENT_FAULT_DRIVE);
    }
}

void MotorParamIdentification_AcknowledgeStart(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    uint16_t accepted)
{
    if ((runtime == 0) || (runtime->state != MOTOR_IDENT_FLUX_START)) {
        return;
    }
    runtime->output.request_start = 0U;
    if (accepted == 0U) {
        MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_START_REJECTED);
        return;
    }
    runtime->start_acknowledged = 1U;
    MotorIdent_Enter(runtime, MOTOR_IDENT_FLUX_SETTLE);
}

uint16_t MotorParamIdentification_IsBusy(
    const MOTOR_PARAM_IDENT_RUNTIME *runtime)
{
    if (runtime == 0) {
        return 0U;
    }
    return (uint16_t)(((runtime->state != MOTOR_IDENT_IDLE)
        && (runtime->state != MOTOR_IDENT_COMPLETE)
        && (runtime->state != MOTOR_IDENT_FAILED)) ? 1U : 0U);
}

static void MotorIdent_AccumulateResistance(
    MOTOR_PARAM_IDENT_RUNTIME *runtime, float current, float voltage)
{
    runtime->sum_current += current;
    runtime->sum_voltage += voltage;
    runtime->sample_count++;
}

static uint16_t MotorIdent_FinishInductance(
    MOTOR_PARAM_IDENT_RUNTIME *runtime, float *inductance)
{
    float value;

    if ((runtime->sample_count < 8U)
        || (runtime->inductance_denominator <= 1.0e-12f)) {
        return 0U;
    }
    value = runtime->config.base_electrical_rad_s
        * runtime->config.sample_time_s
        * runtime->inductance_numerator
        / runtime->inductance_denominator;
    if (MotorIdent_InRange(value, MOTOR_IDENT_MIN_PARAM_PU,
                           MOTOR_IDENT_MAX_PARAM_PU) == 0U) {
        return 0U;
    }
    *inductance = value;
    return 1U;
}

static void MotorIdent_AccumulateInductance(
    MOTOR_PARAM_IDENT_RUNTIME *runtime, float current, float voltage)
{
    if (runtime->previous_sample_valid != 0U) {
        float delta_current = current - runtime->previous_axis_current;
        float excitation = runtime->previous_axis_voltage
            - runtime->result.rs_pu * runtime->previous_axis_current;

        if ((MotorIdent_Abs(delta_current)
             >= runtime->config.minimum_current_delta_pu)
            && ((excitation * delta_current) > 0.0f)) {
            runtime->inductance_numerator += excitation * delta_current;
            runtime->inductance_denominator += delta_current * delta_current;
            runtime->sample_count++;
        }
    }
    runtime->previous_axis_current = current;
    runtime->previous_axis_voltage = voltage;
    runtime->previous_sample_valid = 1U;
}

static void MotorIdent_Complete(MOTOR_PARAM_IDENT_RUNTIME *runtime)
{
    float inductance_scale = runtime->config.voltage_base_v
        / (runtime->config.current_base_a
           * runtime->config.base_electrical_rad_s);
    float flux_scale = runtime->config.voltage_base_v
        / runtime->config.base_electrical_rad_s;
    float desired_inductance_samples =
        (float)runtime->config.inductance_test_cycles * 0.10f;
    float ld_quality = ((float)runtime->result.ld_samples
        / desired_inductance_samples) * 1000.0f;
    float lq_quality = ((float)runtime->result.lq_samples
        / desired_inductance_samples) * 1000.0f;
    float flux_average = runtime->flux_sum
        / (float)runtime->result.flux_samples;
    float variance = runtime->flux_sum_square
        / (float)runtime->result.flux_samples
        - flux_average * flux_average;
    float coefficient_of_variation;
    float quality;

    if (variance < 0.0f) {
        variance = 0.0f;
    }
    coefficient_of_variation = sqrtf(variance)
        / ((flux_average > 1.0e-9f) ? flux_average : 1.0e-9f);
    quality = 1000.0f / (1.0f + 20.0f * coefficient_of_variation);
    quality = MotorIdent_Min(quality, ld_quality);
    quality = MotorIdent_Min(quality, lq_quality);
    if (quality > 1000.0f) {
        quality = 1000.0f;
    }
    if (quality < 0.0f) {
        quality = 0.0f;
    }

    runtime->result.ls_pu = 0.5f
        * (runtime->result.ld_pu + runtime->result.lq_pu);
    runtime->result.flux_pu = flux_average;
    runtime->result.rs_ohm = runtime->result.rs_pu
        * runtime->config.voltage_base_v / runtime->config.current_base_a;
    runtime->result.ld_h = runtime->result.ld_pu * inductance_scale;
    runtime->result.lq_h = runtime->result.lq_pu * inductance_scale;
    runtime->result.ls_h = runtime->result.ls_pu * inductance_scale;
    runtime->result.flux_wb = runtime->result.flux_pu * flux_scale;
    runtime->result.quality = (uint16_t)quality;
    runtime->result.valid = 1U;
    memset(&runtime->output, 0, sizeof(runtime->output));
    runtime->state = MOTOR_IDENT_COMPLETE;
}

void MotorParamIdentification_Step(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    const MOTOR_PARAM_IDENT_INPUT *input)
{
    float current_d;
    float current_q;
    float voltage_d;
    float voltage_q;
    float command;

    if ((runtime == 0) || (input == 0)
        || (MotorParamIdentification_IsBusy(runtime) == 0U)) {
        return;
    }
    if ((input->drive_enabled == 0U) || (input->fault_active != 0U)) {
        MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_DRIVE);
        return;
    }
    if (runtime->total_cycles < 0xffffffffUL) {
        runtime->total_cycles++;
    }
    if (runtime->total_cycles > runtime->config.timeout_cycles) {
        MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_TIMEOUT);
        return;
    }
    if ((MotorIdent_StationaryState(runtime->state) != 0U)
        && (MotorIdent_Abs(input->speed_rpm)
            > runtime->config.stationary_speed_limit_rpm)) {
        MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_EXCESS_SPEED);
        return;
    }

    runtime->state_cycles++;
    memset(&runtime->output, 0, sizeof(runtime->output));
    MotorIdent_ToDq(input->current_alpha_pu, input->current_beta_pu,
        (runtime->state <= MOTOR_IDENT_ZERO_BEFORE_FLUX)
            ? runtime->captured_theta_turns : input->control_theta_turns,
        &current_d, &current_q);
    MotorIdent_ToDq(input->voltage_alpha_pu, input->voltage_beta_pu,
        (runtime->state <= MOTOR_IDENT_ZERO_BEFORE_FLUX)
            ? runtime->captured_theta_turns : input->control_theta_turns,
        &voltage_d, &voltage_q);

    switch (runtime->state) {
    case MOTOR_IDENT_HFI_ACQUIRE:
        runtime->output.control_active = 1U;
        runtime->output.angle_override = 1U;
        runtime->output.current_override = 1U;
        runtime->output.requires_hfi = 1U;
        runtime->output.theta_turns = input->hfi_theta_turns;
        if ((input->hfi_locked != 0U)
            && (input->hfi_quality >= runtime->config.minimum_hfi_quality)) {
            runtime->hfi_good_cycles++;
            if (runtime->hfi_good_cycles >= runtime->config.hfi_lock_cycles) {
                runtime->captured_theta_turns = input->hfi_theta_turns;
                MotorIdent_Enter(runtime, MOTOR_IDENT_RS_POS_SETTLE);
            }
        } else {
            runtime->hfi_good_cycles = 0U;
            if (runtime->state_cycles
                > (20U * runtime->config.hfi_lock_cycles)) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_HFI_TIMEOUT);
            }
        }
        break;

    case MOTOR_IDENT_RS_POS_SETTLE:
        MotorIdent_SetStationaryOutput(runtime,
            runtime->config.resistance_current_pu, 0.0f);
        if (runtime->state_cycles >= runtime->config.settle_cycles) {
            MotorIdent_Enter(runtime, MOTOR_IDENT_RS_POS_SAMPLE);
        }
        break;

    case MOTOR_IDENT_RS_POS_SAMPLE:
        MotorIdent_SetStationaryOutput(runtime,
            runtime->config.resistance_current_pu, 0.0f);
        MotorIdent_AccumulateResistance(runtime, current_d, voltage_d);
        if (runtime->sample_count >= runtime->config.average_cycles) {
            runtime->rs_pos_current = runtime->sum_current
                / (float)runtime->sample_count;
            runtime->rs_pos_voltage = runtime->sum_voltage
                / (float)runtime->sample_count;
            MotorIdent_Enter(runtime, MOTOR_IDENT_RS_NEG_SETTLE);
        }
        break;

    case MOTOR_IDENT_RS_NEG_SETTLE:
        MotorIdent_SetStationaryOutput(runtime,
            -runtime->config.resistance_current_pu, 0.0f);
        if (runtime->state_cycles >= runtime->config.settle_cycles) {
            MotorIdent_Enter(runtime, MOTOR_IDENT_RS_NEG_SAMPLE);
        }
        break;

    case MOTOR_IDENT_RS_NEG_SAMPLE:
        MotorIdent_SetStationaryOutput(runtime,
            -runtime->config.resistance_current_pu, 0.0f);
        MotorIdent_AccumulateResistance(runtime, current_d, voltage_d);
        if (runtime->sample_count >= runtime->config.average_cycles) {
            float negative_current = runtime->sum_current
                / (float)runtime->sample_count;
            float negative_voltage = runtime->sum_voltage
                / (float)runtime->sample_count;
            float current_span = runtime->rs_pos_current - negative_current;

            runtime->result.rs_positive_current_pu = runtime->rs_pos_current;
            runtime->result.rs_positive_voltage_pu = runtime->rs_pos_voltage;
            runtime->result.rs_negative_current_pu = negative_current;
            runtime->result.rs_negative_voltage_pu = negative_voltage;
            if (MotorIdent_Abs(current_span)
                < (10.0f * runtime->config.minimum_current_delta_pu)) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_SIGNAL);
                break;
            }
            runtime->result.rs_pu =
                (runtime->rs_pos_voltage - negative_voltage) / current_span;
            if (MotorIdent_InRange(runtime->result.rs_pu,
                    MOTOR_IDENT_MIN_PARAM_PU, MOTOR_IDENT_MAX_PARAM_PU) == 0U) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_RS_RANGE);
                break;
            }
            MotorIdent_Enter(runtime, MOTOR_IDENT_ZERO_BEFORE_LD);
        }
        break;

    case MOTOR_IDENT_ZERO_BEFORE_LD:
        MotorIdent_SetStationaryOutput(runtime, 0.0f, 0.0f);
        if (runtime->state_cycles >= runtime->config.settle_cycles) {
            MotorIdent_Enter(runtime, MOTOR_IDENT_LD_TEST);
        }
        break;

    case MOTOR_IDENT_LD_TEST:
        command = (((runtime->state_cycles
            / runtime->config.inductance_half_period_cycles) & 1U) == 0U)
            ? runtime->config.inductance_current_pu
            : -runtime->config.inductance_current_pu;
        MotorIdent_SetStationaryOutput(runtime, command, 0.0f);
        MotorIdent_AccumulateInductance(runtime, current_d, voltage_d);
        if (runtime->state_cycles >= runtime->config.inductance_test_cycles) {
            runtime->result.ld_samples = runtime->sample_count;
            if (MotorIdent_FinishInductance(runtime,
                                            &runtime->result.ld_pu) == 0U) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_LD_RANGE);
                break;
            }
            MotorIdent_Enter(runtime, MOTOR_IDENT_ZERO_BEFORE_LQ);
        }
        break;

    case MOTOR_IDENT_ZERO_BEFORE_LQ:
        MotorIdent_SetStationaryOutput(runtime, 0.0f, 0.0f);
        if (runtime->state_cycles >= runtime->config.settle_cycles) {
            MotorIdent_Enter(runtime, MOTOR_IDENT_LQ_TEST);
        }
        break;

    case MOTOR_IDENT_LQ_TEST:
        command = (((runtime->state_cycles
            / runtime->config.inductance_half_period_cycles) & 1U) == 0U)
            ? runtime->config.inductance_current_pu
            : -runtime->config.inductance_current_pu;
        MotorIdent_SetStationaryOutput(runtime, 0.0f, command);
        MotorIdent_AccumulateInductance(runtime, current_q, voltage_q);
        if (runtime->state_cycles >= runtime->config.inductance_test_cycles) {
            runtime->result.lq_samples = runtime->sample_count;
            if (MotorIdent_FinishInductance(runtime,
                                            &runtime->result.lq_pu) == 0U) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_LQ_RANGE);
                break;
            }
            MotorIdent_Enter(runtime, MOTOR_IDENT_ZERO_BEFORE_FLUX);
        }
        break;

    case MOTOR_IDENT_ZERO_BEFORE_FLUX:
        MotorIdent_SetStationaryOutput(runtime, 0.0f, 0.0f);
        if (runtime->state_cycles >= runtime->config.settle_cycles) {
            MotorIdent_Enter(runtime, MOTOR_IDENT_FLUX_START);
        }
        break;

    case MOTOR_IDENT_FLUX_START:
        runtime->output.request_start = 1U;
        runtime->output.start_direction = 1;
        runtime->output.start_speed_rpm =
            (int32_t)runtime->config.flux_speed_rpm;
        break;

    case MOTOR_IDENT_FLUX_SETTLE:
        if ((input->startup_running != 0U)
            && (input->feedback_valid != 0U)
            && (MotorIdent_Abs(input->speed_rpm
                    - runtime->config.flux_speed_rpm)
                <= runtime->config.flux_speed_tolerance_rpm)) {
            runtime->sample_count++;
            if (runtime->sample_count >= runtime->config.flux_settle_cycles) {
                MotorIdent_Enter(runtime, MOTOR_IDENT_FLUX_SAMPLE);
            }
        } else {
            runtime->sample_count = 0U;
        }
        break;

    case MOTOR_IDENT_FLUX_SAMPLE:
        if ((input->startup_running == 0U) || (input->feedback_valid == 0U)) {
            MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_FEEDBACK);
            break;
        }
        if (MotorIdent_Abs(input->speed_rpm - runtime->config.flux_speed_rpm)
            <= runtime->config.flux_speed_tolerance_rpm) {
            float omega_e = input->speed_rpm * runtime->config.pole_pairs
                * MOTOR_IDENT_TWO_PI_F / 60.0f;

            if (runtime->previous_sample_valid != 0U
                && (MotorIdent_Abs(omega_e) > 1.0f)) {
                float d_current_rate = (current_d
                    - runtime->previous_d_current)
                    / runtime->config.sample_time_s;
                float q_current_rate = (current_q
                    - runtime->previous_q_current)
                    / runtime->config.sample_time_s;
                float emf_q = voltage_q
                    - runtime->result.rs_pu * current_q
                    - (runtime->result.lq_pu
                        / runtime->config.base_electrical_rad_s)
                        * q_current_rate
                    - (omega_e / runtime->config.base_electrical_rad_s)
                        * runtime->result.ld_pu * current_d;
                float flux = MotorIdent_Abs(
                    runtime->config.base_electrical_rad_s * emf_q / omega_e);

                (void)d_current_rate;
                if ((MotorIdent_Abs(emf_q)
                        >= runtime->config.minimum_flux_voltage_pu)
                    && isfinite(flux)) {
                    runtime->flux_sum += flux;
                    runtime->flux_sum_square += flux * flux;
                    runtime->result.flux_samples++;
                }
            }
            runtime->previous_d_current = current_d;
            runtime->previous_q_current = current_q;
            runtime->previous_sample_valid = 1U;
        }
        if (runtime->state_cycles >= runtime->config.flux_average_cycles) {
            if (runtime->result.flux_samples
                < (runtime->config.flux_average_cycles / 4U)) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_SIGNAL);
                break;
            }
            if (MotorIdent_InRange(runtime->flux_sum
                    / (float)runtime->result.flux_samples,
                    MOTOR_IDENT_MIN_PARAM_PU, MOTOR_IDENT_MAX_FLUX_PU) == 0U) {
                MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_FLUX_RANGE);
                break;
            }
            memset(&runtime->output, 0, sizeof(runtime->output));
            runtime->output.request_stop = 1U;
            MotorIdent_Enter(runtime, MOTOR_IDENT_STOPPING);
            runtime->output.request_stop = 1U;
        }
        break;

    case MOTOR_IDENT_STOPPING:
        runtime->output.request_stop = 1U;
        if (input->startup_idle != 0U) {
            runtime->output.request_stop = 0U;
            MotorIdent_Complete(runtime);
        }
        break;

    default:
        MotorIdent_Fail(runtime, MOTOR_IDENT_FAULT_CONFIG);
        break;
    }
}
