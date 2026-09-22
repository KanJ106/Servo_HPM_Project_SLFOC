#include "SensorlessStartup.h"
#include "SensorlessTravel90.h"

#include <math.h>
#include <string.h>

#define STARTUP_TWO_PI_F       (6.2831853071795864769f)
#define STARTUP_Q24_SCALE_F    (16777216.0f)

static float Startup_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float Startup_WrapTurns(float turns)
{
    turns -= floorf(turns);
    if (turns < 0.0f) {
        turns += 1.0f;
    }
    return turns;
}

static int32_t Startup_TurnsToQ24(float turns)
{
    return (int32_t)(Startup_WrapTurns(turns) * STARTUP_Q24_SCALE_F);
}


static float Startup_Approach(float value, float target, float step)
{
    if (value < target) {
        value += step;
        return (value > target) ? target : value;
    }
    if (value > target) {
        value -= step;
        return (value < target) ? target : value;
    }
    return value;
}

static uint32_t Startup_Cycles(float seconds, float sample_time_s)
{
    float cycles = seconds / sample_time_s;
    if (cycles < 1.0f) {
        return 1U;
    }
    if (cycles > 4294967040.0f) {
        return 0xffffffffUL;
    }
    return (uint32_t)cycles;
}

void SensorlessStartup_DefaultConfig(SENSORLESS_STARTUP_CONFIG *config,
                                     float sample_time_s,
                                     float pole_pairs,
                                     float speed_base_rpm,
                                     float max_speed_rpm,
                                     float iq_limit_pu)
{
    if (config == 0) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->sample_time_s = sample_time_s;
    config->pole_pairs = pole_pairs;
    config->speed_base_rpm = speed_base_rpm;
    config->max_speed_rpm = max_speed_rpm;
    config->align_angle_turns = 0.0f;
    config->align_current_pu = Startup_Clamp(0.5f * iq_limit_pu, 0.02f, 0.10f);
    config->startup_iq_pu = Startup_Clamp(0.6f * iq_limit_pu, 0.03f, 0.12f);
    config->iq_limit_pu = Startup_Clamp(iq_limit_pu, 0.03f, 0.25f);
    config->handoff_speed_rpm = Startup_Clamp(0.10f * speed_base_rpm, 150.0f, 600.0f);
    config->sensorless_exit_speed_rpm = Startup_Clamp(
        0.65f * config->handoff_speed_rpm, 30.0f,
        config->handoff_speed_rpm - 1.0f);
    config->open_loop_accel_rpm_s = 600.0f;
    config->run_accel_rpm_s = 1000.0f;
    config->speed_kp = 0.20f;
    config->speed_ki = 2.0f;
    config->minimum_lock_quality = 450U;
    config->minimum_hfi_quality = 400U;
    config->hfi_enable = 1U;
    config->allow_open_loop_fallback = 0U;
    config->align_cycles = Startup_Cycles(0.40f, sample_time_s);
    config->lock_dwell_cycles = Startup_Cycles(0.08f, sample_time_s);
    config->blend_cycles = Startup_Cycles(0.15f, sample_time_s);
    config->lock_timeout_cycles = Startup_Cycles(3.0f, sample_time_s);
    config->hfi_lock_dwell_cycles = Startup_Cycles(0.030f, sample_time_s);
    config->hfi_timeout_cycles = Startup_Cycles(1.0f, sample_time_s);
    config->unlock_dwell_cycles = Startup_Cycles(0.020f, sample_time_s);
    config->monitor_dwell_cycles = Startup_Cycles(0.100f, sample_time_s);
    config->fallback_ready_cycles = Startup_Cycles(0.200f, sample_time_s);
    config->stop_dwell_cycles = Startup_Cycles(0.100f, sample_time_s);
    config->stop_timeout_cycles = Startup_Cycles(3.0f, sample_time_s);
    config->fallback_max_angle_error_q24 = ROTOR_ANGLE_ONE_TURN_Q24 / 8;
    config->monitor_max_angle_error_q24 = ROTOR_ANGLE_ONE_TURN_Q24 / 6;
    config->monitor_max_speed_error_rpm = 300;
    config->stop_speed_rpm = 30;
    config->debug_mode = SENSORLESS_DEBUG_FULL;
    config->current_ramp_cycles = Startup_Cycles(0.10f, sample_time_s);
    config->if_hold_cycles = Startup_Cycles(1.0f, sample_time_s);
    config->iq_hold_cycles = Startup_Cycles(0.20f, sample_time_s);
    config->ramp_timeout_cycles = Startup_Cycles(3.0f, sample_time_s);
    config->total_timeout_cycles = Startup_Cycles(8.0f, sample_time_s);
    config->run_test_cycles = Startup_Cycles(2.0f, sample_time_s);
}

uint16_t SensorlessStartup_ValidateConfig(
    const SENSORLESS_STARTUP_CONFIG *config)
{
    if (config == 0) {
        return 0U;
    }
    if ((!isfinite(config->sample_time_s))
        || (config->sample_time_s < 0.000001f)
        || (config->sample_time_s > 0.001f)
        || (!isfinite(config->pole_pairs))
        || (config->pole_pairs < 1.0f) || (config->pole_pairs > 64.0f)
        || (!isfinite(config->speed_base_rpm))
        || (config->speed_base_rpm < 100.0f)
        || (!isfinite(config->max_speed_rpm))
        || (config->max_speed_rpm < 10.0f)
        || (!isfinite(config->align_angle_turns))
        || (!isfinite(config->align_current_pu))
        || (!isfinite(config->startup_iq_pu))
        || (!isfinite(config->iq_limit_pu))
        || (config->iq_limit_pu < 0.001f) || (config->iq_limit_pu > 0.50f)
        || (config->align_current_pu < 0.0f)
        || (config->align_current_pu > config->iq_limit_pu)
        || (config->startup_iq_pu < 0.0f)
        || (config->startup_iq_pu > config->iq_limit_pu)
        || (!isfinite(config->handoff_speed_rpm))
        || (config->handoff_speed_rpm < 30.0f)
        || (config->handoff_speed_rpm > config->max_speed_rpm)
        || (!isfinite(config->sensorless_exit_speed_rpm))
        || (config->sensorless_exit_speed_rpm < 0.0f)
        || (config->sensorless_exit_speed_rpm >= config->handoff_speed_rpm)
        || (!isfinite(config->open_loop_accel_rpm_s))
        || (config->open_loop_accel_rpm_s < 1.0f)
        || (config->open_loop_accel_rpm_s > 100000.0f)
        || (!isfinite(config->run_accel_rpm_s))
        || (config->run_accel_rpm_s < 1.0f)
        || (config->run_accel_rpm_s > 100000.0f)
        || (!isfinite(config->speed_kp)) || (config->speed_kp < 0.0f)
        || (config->speed_kp > 10.0f)
        || (!isfinite(config->speed_ki)) || (config->speed_ki < 0.0f)
        || (config->speed_ki > 1000.0f)
        || (config->minimum_lock_quality > 1000U)
        || (config->minimum_hfi_quality > 1000U)
        || (config->hfi_enable > 1U)
        || (config->allow_open_loop_fallback > 1U)
        || (config->align_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->current_ramp_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->if_hold_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->iq_hold_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->blend_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->lock_timeout_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->ramp_timeout_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->total_timeout_cycles > Startup_Cycles(120.0f, config->sample_time_s))
        || (config->run_test_cycles > Startup_Cycles(60.0f, config->sample_time_s))
        || (config->debug_mode > SENSORLESS_DEBUG_SMO_FRAME)
        || ((config->debug_mode == SENSORLESS_DEBUG_D_AXIS
             || config->debug_mode == SENSORLESS_DEBUG_D_AXIS_SCAN)
            && (config->hfi_enable || !config->allow_open_loop_fallback))
        || (config->current_ramp_cycles == 0U)
        || (config->if_hold_cycles == 0U)
        || (config->iq_hold_cycles == 0U)
        || (config->ramp_timeout_cycles == 0U)
        || (config->total_timeout_cycles == 0U)
        || (config->run_test_cycles == 0U)
        || (config->align_cycles == 0U)
        || (config->lock_dwell_cycles == 0U)
        || (config->blend_cycles == 0U)
        || (config->lock_timeout_cycles == 0U)
        || (config->hfi_lock_dwell_cycles == 0U)
        || (config->hfi_timeout_cycles == 0U)
        || (config->unlock_dwell_cycles == 0U)
        || (config->monitor_dwell_cycles == 0U)
        || (config->fallback_ready_cycles == 0U)
        || (config->stop_dwell_cycles == 0U)
        || (config->stop_timeout_cycles == 0U)
        || (config->fallback_max_angle_error_q24 <= 0)
        || (config->fallback_max_angle_error_q24 > ROTOR_ANGLE_HALF_TURN_Q24)
        || (config->monitor_max_angle_error_q24 <= 0)
        || (config->monitor_max_angle_error_q24 > ROTOR_ANGLE_HALF_TURN_Q24)
        || (config->monitor_max_speed_error_rpm <= 0)
        || (config->stop_speed_rpm < 0) || (config->stop_speed_rpm > 500)) {
        return 0U;
    }
    if (SENSORLESS_IS_TRAVEL(config->debug_mode)) {
        float ramp_s=(float)config->ramp_timeout_cycles*config->sample_time_s;
        float total_s=(float)config->total_timeout_cycles*config->sample_time_s;
        if(config->hfi_enable || !config->allow_open_loop_fallback
            || config->pole_pairs!=10.0f || config->handoff_speed_rpm!=100.0f
            || config->max_speed_rpm<100.0f
            || config->open_loop_accel_rpm_s!=100.0f
            || config->run_accel_rpm_s!=100.0f
            || config->align_current_pu<=0.0f || config->startup_iq_pu<=0.0f
            || ramp_s<16.5f || ramp_s>18.0f || total_s<17.0f || total_s>20.0f
            || (float)config->align_cycles*config->sample_time_s>0.501f)
            return 0U;
    }
    if (SENSORLESS_IS_PROBE(config->debug_mode)
        && (config->sample_time_s != 1.0f/16000.0f
            || config->align_cycles != 8000U || config->current_ramp_cycles != 3200U
            || config->align_current_pu != 0.03125f || config->startup_iq_pu != 0.03125f
            || config->iq_limit_pu != 0.03125f || config->align_angle_turns != 0.0f)) return 0U;
    if (config->debug_mode == SENSORLESS_DEBUG_D_AXIS_SCAN) {
        uint32_t leg = config->if_hold_cycles;
        float duration = (float)leg * config->sample_time_s;
        float peak_speed = 45.0f / (duration * config->pole_pairs);
        float peak_accel = 180.0f / (duration * duration * config->pole_pairs);
        if (duration < 0.49999f || duration > 2.00001f
            || config->align_current_pu <= 0.0f
            || peak_speed > config->max_speed_rpm
            || peak_speed > config->handoff_speed_rpm
            || peak_accel > config->open_loop_accel_rpm_s
            || config->ramp_timeout_cycles <= 2U * leg + 1U
            || config->total_timeout_cycles <= config->align_cycles + 2U * leg + 2U)
            return 0U;
    }
    return 1U;
}

void SensorlessStartup_Init(SENSORLESS_STARTUP_RUNTIME *runtime,
                            const SENSORLESS_STARTUP_CONFIG *config)
{
    if ((runtime == 0) || (config == 0)) {
        return;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->config = *config;
    runtime->state = SENSORLESS_START_IDLE;
    runtime->direction = 1;
}

uint16_t SensorlessStartup_RequestStart(SENSORLESS_STARTUP_RUNTIME *runtime,
                                        int16_t direction,
                                        int32_t target_speed_rpm)
{
    int64_t speed_magnitude;

    if ((runtime == 0) || (runtime->state != SENSORLESS_START_IDLE)
        || (SensorlessStartup_ValidateConfig(&runtime->config) == 0U)) {
        return 0U;
    }

    /* New record: reset all historical counters, retain only validated config. */
    {
        SENSORLESS_STARTUP_CONFIG saved = runtime->config;
        SensorlessStartup_Init(runtime, &saved);
    }
    runtime->direction = (direction < 0) ? -1 : 1;
    speed_magnitude = (int64_t)target_speed_rpm;
    if (speed_magnitude < 0) {
        speed_magnitude = -speed_magnitude;
    }
    if ((speed_magnitude == 0) && (runtime->config.hfi_enable == 0U)) {
        speed_magnitude = (int64_t)runtime->config.handoff_speed_rpm;
    }
    if ((float)speed_magnitude > runtime->config.max_speed_rpm) {
        speed_magnitude = (int64_t)runtime->config.max_speed_rpm;
    }
    runtime->target_speed_rpm = runtime->direction * (int32_t)speed_magnitude;
    runtime->start_requested = 1U;
    runtime->output.fault_flags = 0U;
    return 1U;
}

uint16_t SensorlessStartup_UpdateTarget(SENSORLESS_STARTUP_RUNTIME *runtime,
                                        int16_t direction,
                                        int32_t target_speed_rpm)
{
    int64_t speed_magnitude;
    int16_t requested_direction;

    if ((runtime == 0) || (runtime->state == SENSORLESS_START_IDLE)
        || (runtime->state == SENSORLESS_START_FAULT)) {
        return 0U;
    }
    requested_direction = (direction < 0) ? -1 : 1;
    speed_magnitude = (int64_t)target_speed_rpm;
    if (speed_magnitude < 0) {
        speed_magnitude = -speed_magnitude;
    }
    if ((float)speed_magnitude > runtime->config.max_speed_rpm) {
        return 0U;
    }
    if ((requested_direction != runtime->direction)
        && (fabsf(runtime->run_speed_ref_rpm)
            > runtime->config.sensorless_exit_speed_rpm)) {
        return 0U;
    }
    runtime->direction = requested_direction;
    runtime->target_speed_rpm = requested_direction * (int32_t)speed_magnitude;
    return 1U;
}

void SensorlessStartup_RequestStop(SENSORLESS_STARTUP_RUNTIME *runtime)
{
    if (runtime == 0) {
        return;
    }

    if (SENSORLESS_IS_PROBE(runtime->config.debug_mode)
        && runtime->output.control_active
        && runtime->state != SENSORLESS_START_FAULT
        && (runtime->output.id_ref_pu != 0.0f || runtime->output.iq_ref_pu != 0.0f)) {
        runtime->probe.stop = 1U;
        return;
    }
    runtime->start_requested = 0U;
    if (runtime->config.debug_mode != SENSORLESS_DEBUG_FULL
        && runtime->state != SENSORLESS_START_FAULT) {
        runtime->output.control_active = runtime->output.current_override = 0U;
        runtime->output.angle_override = 0U;
        runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
        runtime->state = SENSORLESS_START_IDLE;
        runtime->state_cycles = 0U;
        return;
    }
    if ((runtime->state != SENSORLESS_START_IDLE)
        && (runtime->state != SENSORLESS_START_FAULT)) {
        runtime->state = SENSORLESS_START_STOPPING;
        runtime->state_cycles = 0U;
        runtime->stop_good_cycles = 0U;
    }
}

void SensorlessStartup_ClearFault(SENSORLESS_STARTUP_RUNTIME *runtime)
{
    if ((runtime != 0) && (runtime->state == SENSORLESS_START_FAULT)) {
        runtime->state = SENSORLESS_START_IDLE;
        runtime->output.fault_flags = 0U;
        runtime->output.fault_request = 0U;
        memset(&runtime->failure, 0, sizeof(runtime->failure));
        runtime->start_requested = 0U;
        runtime->state_cycles = 0U;
    }
}

uint16_t SensorlessStartup_IsIdle(const SENSORLESS_STARTUP_RUNTIME *runtime)
{
    return (uint16_t)(((runtime != 0) && (runtime->state == SENSORLESS_START_IDLE))
        ? 1U : 0U);
}

static void Startup_Enter(SENSORLESS_STARTUP_RUNTIME *runtime,
                          SENSORLESS_START_STATE state)
{
    runtime->state = state;
    runtime->state_cycles = 0U;
}

static void Startup_EnterFault(SENSORLESS_STARTUP_RUNTIME *runtime, uint32_t fault)
{
    if (runtime->failure.fault == 0U) {
        runtime->failure.fault = fault;
        runtime->failure.state = (uint16_t)runtime->state;
        runtime->failure.state_cycles = runtime->state_cycles;
        runtime->failure.total_cycles = runtime->total_start_cycles;
        runtime->failure.blend = runtime->blend_fraction;
        runtime->failure.open_theta_q24 = Startup_TurnsToQ24(runtime->open_loop_theta_turns);
        runtime->failure.observer_theta_q24 = runtime->last_observer_theta_q24;
        runtime->failure.control_theta_q24 = runtime->output.theta_e_q24;
        runtime->failure.iq = runtime->output.iq_ref_pu;
    }
    runtime->output.fault_flags |= fault;
    runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
    runtime->output.control_active = runtime->output.angle_override = 0U;
    runtime->output.current_override = 0U;
    runtime->output.fault_request = 1U;
    runtime->start_requested = 0U;
    Startup_Enter(runtime, SENSORLESS_START_FAULT);
}

void SensorlessStartup_Trip(SENSORLESS_STARTUP_RUNTIME *runtime, uint32_t fault)
{
    if (runtime != 0) Startup_EnterFault(runtime, fault);
}

static float Startup_SpeedPi(SENSORLESS_STARTUP_RUNTIME *runtime,
                             float speed_rpm)
{
    float error_pu;
    float proportional;
    float integrator_candidate;
    float unsaturated_iq;
    float saturated_iq;

    error_pu = (runtime->run_speed_ref_rpm - speed_rpm)
        / runtime->config.speed_base_rpm;
    proportional = runtime->config.speed_kp * error_pu;
    if (runtime->speed_pi_pending != 0U) {
        runtime->speed_integrator_pu = runtime->output.iq_ref_pu - proportional;
        runtime->speed_pi_pending = 0U;
        return runtime->output.iq_ref_pu;
    }
    integrator_candidate = runtime->speed_integrator_pu
        + runtime->config.speed_ki * error_pu
            * runtime->config.sample_time_s;
    unsaturated_iq = proportional + integrator_candidate;
    saturated_iq = Startup_Clamp(unsaturated_iq,
        -runtime->config.iq_limit_pu, runtime->config.iq_limit_pu);

    if ((unsaturated_iq == saturated_iq)
        || ((unsaturated_iq > saturated_iq) && (error_pu < 0.0f))
        || ((unsaturated_iq < saturated_iq) && (error_pu > 0.0f))) {
        runtime->speed_integrator_pu = integrator_candidate;
    }
    return Startup_Clamp(proportional + runtime->speed_integrator_pu,
        -runtime->config.iq_limit_pu, runtime->config.iq_limit_pu);
}

static void Startup_SetActiveOutput(SENSORLESS_STARTUP_RUNTIME *runtime,
                                    uint16_t source,
                                    int32_t theta_q24,
                                    float id_ref_pu,
                                    float iq_ref_pu)
{
    runtime->output.control_active = 1U;
    runtime->output.angle_override = 1U;
    runtime->output.current_override = 1U;
    runtime->output.source = source;
    runtime->output.theta_e_q24 = RotorFeedback_WrapAngleQ24(theta_q24);
    runtime->output.id_ref_pu = id_ref_pu;
    runtime->output.iq_ref_pu = iq_ref_pu;
    runtime->output.speed_ref_rpm = runtime->run_speed_ref_rpm;
}

static void Startup_UpdateOpenLoop(SENSORLESS_STARTUP_RUNTIME *runtime)
{
    float target_rpm = (float)runtime->direction * runtime->config.handoff_speed_rpm;
    float speed_step = runtime->config.open_loop_accel_rpm_s
        * runtime->config.sample_time_s;
    float electrical_turns_per_second;

    runtime->open_loop_speed_rpm = Startup_Approach(runtime->open_loop_speed_rpm,
        target_rpm, speed_step);
    electrical_turns_per_second = runtime->open_loop_speed_rpm
        * runtime->config.pole_pairs / 60.0f;
    runtime->open_loop_theta_turns = Startup_WrapTurns(runtime->open_loop_theta_turns
        + electrical_turns_per_second * runtime->config.sample_time_s);
}

static uint16_t Startup_LockGood(const SENSORLESS_STARTUP_RUNTIME *runtime,
                                 const SENSORLESS_STARTUP_INPUT *input)
{
    return (uint16_t)(((input->observer_locked != 0U)
        && (runtime->state == SENSORLESS_START_STOPPING
            || (int64_t)input->observer_speed_rpm * runtime->direction > 0)
        && (input->observer_quality >= runtime->config.minimum_lock_quality)) ? 1U : 0U);
}

static uint16_t Startup_HfiGood(const SENSORLESS_STARTUP_RUNTIME *runtime,
                                 const SENSORLESS_STARTUP_INPUT *input)
{
    return (uint16_t)(((input->hfi_capable != 0U)
        && (input->hfi_locked != 0U)
        && (input->hfi_quality >= runtime->config.minimum_hfi_quality)
        && (input->hfi_fault_count == runtime->hfi_fault_snapshot)) ? 1U : 0U);
}

static uint16_t Startup_FeedbackAgreement(
    const SENSORLESS_STARTUP_RUNTIME *runtime,
    int32_t first_theta_q24,
    int32_t first_speed_rpm,
    int32_t second_theta_q24,
    int32_t second_speed_rpm)
{
    int32_t angle_error = RotorFeedback_AngleErrorQ24(
        first_theta_q24, second_theta_q24);
    int64_t speed_error = (int64_t)first_speed_rpm
        - (int64_t)second_speed_rpm;

    if (angle_error < 0) {
        angle_error = -angle_error;
    }
    if (speed_error < 0) {
        speed_error = -speed_error;
    }
    return (uint16_t)(((angle_error
            <= runtime->config.monitor_max_angle_error_q24)
        && (speed_error
            <= runtime->config.monitor_max_speed_error_rpm)) ? 1U : 0U);
}

static void Startup_CheckRunMonitors(SENSORLESS_STARTUP_RUNTIME *runtime,
                                     const SENSORLESS_STARTUP_INPUT *input)
{
    if ((input->observer_fault_count != runtime->observer_fault_snapshot)
        && (input->observer_fault_count != 0U)) {
        Startup_EnterFault(runtime, SENSORLESS_FAULT_OBSERVER_NUMERIC);
        return;
    }

    if (Startup_LockGood(runtime, input) == 0U) {
        if (runtime->unlock_bad_cycles < runtime->config.unlock_dwell_cycles) {
            runtime->unlock_bad_cycles++;
        }
    } else {
        runtime->unlock_bad_cycles = 0U;
    }

    if (runtime->unlock_bad_cycles >= runtime->config.unlock_dwell_cycles) {
        runtime->output.fault_flags |= SENSORLESS_FAULT_LOCK_LOST;
        if ((runtime->config.hfi_enable != 0U)
            && (input->hfi_capable != 0U)) {
            runtime->hfi_fault_snapshot = input->hfi_fault_count;
            runtime->hfi_good_cycles = 0U;
            runtime->hfi_bad_cycles = 0U;
            runtime->unlock_bad_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_HFI_REACQUIRE);
        } else {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_LOCK_LOST);
        }
    }
}

void SensorlessStartup_Step(SENSORLESS_STARTUP_RUNTIME *runtime,
                            const SENSORLESS_STARTUP_INPUT *input)
{
    float blend;
    int32_t blend_error;
    int32_t blended_theta;
    float speed_step;
    float run_speed;
    float from_speed;
    int32_t from_theta;
    float stop_iq;
    int32_t stop_theta;
    uint16_t stop_source;
    uint16_t stop_feedback_safe;

    if ((runtime == 0) || (input == 0)) {
        return;
    }

    runtime->output.control_active = 0U;
    runtime->output.angle_override = 0U;
    runtime->output.current_override = 0U;
    runtime->output.fault_request = 0U;
    runtime->output.source = ROTOR_SOURCE_NONE;
    runtime->output.fault_flags &= ~SENSORLESS_STATUS_HFI_ACTIVE;
    runtime->state_cycles++;
    runtime->last_observer_theta_q24 = input->observer_theta_q24;
    if (runtime->start_requested && runtime->state != SENSORLESS_START_RUN
        && runtime->state != SENSORLESS_START_STOPPING
        && runtime->state != SENSORLESS_START_FAULT) {
        if (runtime->total_start_cycles < 0xffffffffUL) runtime->total_start_cycles++;
        if (runtime->total_start_cycles >= runtime->config.total_timeout_cycles)
            Startup_EnterFault(runtime, SENSORLESS_FAULT_START_TIMEOUT);
    }
    if (runtime->state != SENSORLESS_START_IDLE && runtime->state != SENSORLESS_START_FAULT
        && input->observer_fault_count != runtime->observer_fault_snapshot)
        Startup_EnterFault(runtime, SENSORLESS_FAULT_OBSERVER_NUMERIC);

    if ((runtime->state != SENSORLESS_START_IDLE)
        && (runtime->state != SENSORLESS_START_FAULT)
        && (input->drive_enabled == 0U)) {
        Startup_Enter(runtime, SENSORLESS_START_IDLE);
        runtime->start_requested = 0U;
    }

    switch (runtime->state) {
    case SENSORLESS_START_IDLE:
        runtime->speed_integrator_pu = 0.0f;
        runtime->run_speed_ref_rpm = 0.0f;
        runtime->open_loop_speed_rpm = 0.0f;
        if ((runtime->start_requested != 0U) && (input->drive_enabled != 0U)) {
            if (input->params_valid == 0U) {
                Startup_EnterFault(runtime, SENSORLESS_FAULT_PARAM_INVALID);
                break;
            }
            runtime->open_loop_theta_turns = Startup_WrapTurns(
                runtime->config.align_angle_turns);
            runtime->observer_fault_snapshot = input->observer_fault_count;
            runtime->hfi_fault_snapshot = input->hfi_fault_count;
            runtime->hfi_good_cycles = 0U;
            runtime->hfi_bad_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_ALIGN);
        }
        break;

    case SENSORLESS_START_ALIGN:
        {
            float align_current = runtime->config.align_current_pu;
            /* Mode5 only: slew the fixed-angle alignment current, then retain
             * the existing dwell and trajectory. Do not relax current trips. */
            if (SENSORLESS_IS_TRAVEL(runtime->config.debug_mode)) {
                uint32_t ramp_cycles = runtime->config.current_ramp_cycles;
                if (ramp_cycles > runtime->config.align_cycles)
                    ramp_cycles = runtime->config.align_cycles;
                if (ramp_cycles && runtime->state_cycles < ramp_cycles)
                    align_current *= (float)runtime->state_cycles / (float)ramp_cycles;
            }
            Startup_SetActiveOutput(runtime, ROTOR_SOURCE_OPEN_LOOP,
                Startup_TurnsToQ24(runtime->open_loop_theta_turns),
                align_current, 0.0f);
        }
        if (runtime->state_cycles >= runtime->config.align_cycles) {
            if (runtime->config.debug_mode == SENSORLESS_DEBUG_ALIGN) {
                runtime->debug_completed = 1U;
                runtime->start_requested = 0U;
                runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
                runtime->output.control_active = runtime->output.current_override = 0U;
                runtime->output.angle_override = 0U;
                Startup_Enter(runtime, SENSORLESS_START_IDLE);
            } else if ((runtime->config.hfi_enable != 0U)
                && (input->hfi_capable != 0U)) {
                Startup_Enter(runtime, SENSORLESS_START_HFI_ACQUIRE);
            } else if (runtime->config.allow_open_loop_fallback != 0U) {
                runtime->output.fault_flags |= SENSORLESS_STATUS_IF_FALLBACK;
                Startup_Enter(runtime, SENSORLESS_START_OPEN_LOOP_RAMP);
            } else {
                Startup_EnterFault(runtime,
                    SENSORLESS_FAULT_HFI_UNAVAILABLE);
            }
        }
        break;

    case SENSORLESS_START_HFI_ACQUIRE:
        runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_HFI,
            (runtime->state_cycles <= 1U)
                ? runtime->output.theta_e_q24 : input->hfi_theta_q24,
            0.0f, 0.0f);
        if ((input->hfi_fault_count != runtime->hfi_fault_snapshot)
            && (input->hfi_fault_count != 0U)) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_HFI_LOCK_LOST);
            break;
        }
        if (Startup_HfiGood(runtime, input) != 0U) {
            if (runtime->hfi_good_cycles
                < runtime->config.hfi_lock_dwell_cycles) {
                runtime->hfi_good_cycles++;
            }
        } else {
            runtime->hfi_good_cycles = 0U;
        }
        if (runtime->hfi_good_cycles
            >= runtime->config.hfi_lock_dwell_cycles) {
            runtime->run_speed_ref_rpm = (float)input->hfi_speed_rpm;
            runtime->speed_integrator_pu = 0.0f;
            runtime->lock_good_cycles = 0U;
            runtime->hfi_bad_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_HFI_RUN);
        } else if (runtime->state_cycles >= runtime->config.hfi_timeout_cycles) {
            if (runtime->config.allow_open_loop_fallback != 0U) {
                runtime->output.fault_flags |= SENSORLESS_STATUS_IF_FALLBACK;
                runtime->open_loop_theta_turns = (float)
                    RotorFeedback_WrapAngleQ24(runtime->output.theta_e_q24)
                    / STARTUP_Q24_SCALE_F;
                Startup_Enter(runtime, SENSORLESS_START_OPEN_LOOP_RAMP);
            } else {
                Startup_EnterFault(runtime, SENSORLESS_FAULT_HFI_LOCK_TIMEOUT);
            }
        }
        break;

    case SENSORLESS_START_HFI_RUN:
        runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
        speed_step = runtime->config.run_accel_rpm_s
            * runtime->config.sample_time_s;
        runtime->run_speed_ref_rpm = Startup_Approach(
            runtime->run_speed_ref_rpm, (float)runtime->target_speed_rpm,
            speed_step);
        run_speed = (float)input->hfi_speed_rpm;
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_HFI,
            input->hfi_theta_q24, 0.0f,
            Startup_SpeedPi(runtime, run_speed));
        if (Startup_HfiGood(runtime, input) == 0U) {
            if (runtime->hfi_bad_cycles
                < runtime->config.unlock_dwell_cycles) {
                runtime->hfi_bad_cycles++;
            }
        } else {
            runtime->hfi_bad_cycles = 0U;
        }
        if (runtime->hfi_bad_cycles >= runtime->config.unlock_dwell_cycles) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_HFI_LOCK_LOST);
            break;
        }
        if ((fabsf((float)input->hfi_speed_rpm)
                >= runtime->config.handoff_speed_rpm)
            && (Startup_LockGood(runtime, input) != 0U)
            && (Startup_FeedbackAgreement(runtime,
                    input->observer_theta_q24,
                    input->observer_speed_rpm,
                    input->hfi_theta_q24,
                    input->hfi_speed_rpm) != 0U)) {
            if (runtime->lock_good_cycles
                < runtime->config.lock_dwell_cycles) {
                runtime->lock_good_cycles++;
            }
        } else {
            runtime->lock_good_cycles = 0U;
        }
        if (runtime->lock_good_cycles >= runtime->config.lock_dwell_cycles) {
            runtime->blend_from_source = ROTOR_SOURCE_HFI;
            Startup_Enter(runtime, SENSORLESS_START_BLEND);
        }
        break;

    case SENSORLESS_START_OPEN_LOOP_RAMP:
        if (runtime->state_cycles >= runtime->config.ramp_timeout_cycles) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_RAMP_TIMEOUT);
            break;
        }
        if (SENSORLESS_IS_TRAVEL(runtime->config.debug_mode)) {
            float t=(float)runtime->state_cycles*runtime->config.sample_time_s;
            float release=(t>SL_TRAVEL_MOVE_S)?Startup_Clamp(
                1.0f-(t-SL_TRAVEL_MOVE_S)/SL_TRAVEL_RELEASE_S,0.0f,1.0f):1.0f;
            runtime->open_loop_theta_turns=Startup_WrapTurns(runtime->config.align_angle_turns
                +(float)runtime->direction*runtime->config.pole_pairs*SlTravel_Turns(t));
            runtime->open_loop_speed_rpm=(float)runtime->direction*SlTravel_Speed(t);
            Startup_SetActiveOutput(runtime,ROTOR_SOURCE_OPEN_LOOP,
                Startup_TurnsToQ24(runtime->open_loop_theta_turns),
                Startup_Approach(runtime->output.id_ref_pu,0.0f,
                    runtime->config.align_current_pu/(float)runtime->config.current_ramp_cycles),
                /* Cruise has reached startup_iq_pu before deceleration.
                 * Scale that fixed value, not the already-released previous output. */
                (t>=SL_TRAVEL_MOVE_S)
                    ? release*(float)runtime->direction*runtime->config.startup_iq_pu
                    : Startup_Approach(runtime->output.iq_ref_pu,
                        (float)runtime->direction*runtime->config.startup_iq_pu,
                        runtime->config.startup_iq_pu/(float)runtime->config.current_ramp_cycles));
            runtime->output.speed_ref_rpm=runtime->open_loop_speed_rpm;
            if(t>=SL_TRAVEL_MOVE_S+SL_TRAVEL_RELEASE_S) {
                SensorlessStartup_RequestStop(runtime);
                runtime->debug_completed=1U;
            }
            break;
        }
        if (runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS_SCAN) {
            uint32_t leg = runtime->config.if_hold_cycles;
            uint32_t n = runtime->state_cycles;
            float u, position, velocity;
            if (n > 2U * leg) {
                SensorlessStartup_RequestStop(runtime);
                runtime->debug_completed = 1U;
                break;
            }
            u = (float)((n <= leg) ? n : n - leg) / (float)leg;
            position = u * u * (3.0f - 2.0f * u);
            velocity = 6.0f * u * (1.0f - u);
            if (n > leg) { position = 1.0f - position; velocity = -velocity; }
            runtime->open_loop_theta_turns = Startup_WrapTurns(
                runtime->config.align_angle_turns
                + (float)runtime->direction * 0.5f * position);
            runtime->open_loop_speed_rpm = (float)runtime->direction
                * 30.0f * velocity / ((float)leg
                    * runtime->config.sample_time_s * runtime->config.pole_pairs);
            Startup_SetActiveOutput(runtime, ROTOR_SOURCE_OPEN_LOOP,
                Startup_TurnsToQ24(runtime->open_loop_theta_turns),
                runtime->config.align_current_pu, 0.0f);
            runtime->output.speed_ref_rpm = runtime->open_loop_speed_rpm;
            break;
        }
        Startup_UpdateOpenLoop(runtime);
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_OPEN_LOOP,
            Startup_TurnsToQ24(runtime->open_loop_theta_turns),
            runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS
                ? runtime->config.align_current_pu
                : Startup_Approach(runtime->output.id_ref_pu, 0.0f,
                runtime->config.align_current_pu / (float)runtime->config.current_ramp_cycles),
            runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS
                ? 0.0f : Startup_Approach(runtime->output.iq_ref_pu,
                (float)runtime->direction * runtime->config.startup_iq_pu,
                runtime->config.startup_iq_pu / (float)runtime->config.current_ramp_cycles));
        runtime->output.speed_ref_rpm = runtime->open_loop_speed_rpm;
        if (fabsf(runtime->open_loop_speed_rpm)
            >= (0.999f * runtime->config.handoff_speed_rpm)
            && runtime->state_cycles >= runtime->config.current_ramp_cycles) {
            Startup_Enter(runtime, SENSORLESS_START_WAIT_LOCK);
        }
        break;

    case SENSORLESS_START_WAIT_LOCK:
        if ((runtime->config.debug_mode == SENSORLESS_DEBUG_IF
             || runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS)
            && runtime->state_cycles >= runtime->config.if_hold_cycles) {
            /* Observer never owns torque/angle in this test. End with neutral PWM,
             * not a closed-loop stop based on unverified feedback. Rotor may coast. */
            runtime->debug_completed = 1U;
            runtime->start_requested = 0U;
            runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
            Startup_Enter(runtime, SENSORLESS_START_IDLE);
            break;
        }
        Startup_UpdateOpenLoop(runtime);
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_OPEN_LOOP,
            Startup_TurnsToQ24(runtime->open_loop_theta_turns),
            runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS
                ? runtime->config.align_current_pu
                : Startup_Approach(runtime->output.id_ref_pu, 0.0f,
                runtime->config.align_current_pu / (float)runtime->config.current_ramp_cycles),
            runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS
                ? 0.0f : Startup_Approach(runtime->output.iq_ref_pu,
                (float)runtime->direction * runtime->config.startup_iq_pu,
                runtime->config.startup_iq_pu / (float)runtime->config.current_ramp_cycles));
        runtime->output.speed_ref_rpm = runtime->open_loop_speed_rpm;
        if (runtime->config.debug_mode == SENSORLESS_DEBUG_IF
            || runtime->config.debug_mode == SENSORLESS_DEBUG_D_AXIS) break;
        if ((Startup_LockGood(runtime, input) != 0U)
            && (Startup_FeedbackAgreement(runtime,
                    input->observer_theta_q24,
                    input->observer_speed_rpm,
                    Startup_TurnsToQ24(runtime->open_loop_theta_turns),
                    (int32_t)runtime->open_loop_speed_rpm) != 0U)) {
            runtime->lock_good_cycles++;
        } else {
            runtime->lock_good_cycles = 0U;
        }
        if (runtime->lock_good_cycles >= runtime->config.lock_dwell_cycles) {
            runtime->run_speed_ref_rpm = runtime->open_loop_speed_rpm;
            runtime->speed_integrator_pu = (float)runtime->direction
                * runtime->config.startup_iq_pu;
            runtime->blend_from_source = ROTOR_SOURCE_OPEN_LOOP;
            Startup_Enter(runtime, SENSORLESS_START_BLEND);
        } else if (runtime->state_cycles >= runtime->config.lock_timeout_cycles) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_LOCK_TIMEOUT);
        }
        break;

    case SENSORLESS_START_BLEND:
        blend = (float)runtime->state_cycles / (float)runtime->config.blend_cycles;
        blend = Startup_Clamp(blend, 0.0f, 1.0f);
        blend = blend * blend * (3.0f - 2.0f * blend);
        if (runtime->blend_from_source == ROTOR_SOURCE_HFI) {
            runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
            from_theta = input->hfi_theta_q24;
            from_speed = (float)input->hfi_speed_rpm;
        } else {
            Startup_UpdateOpenLoop(runtime);
            from_theta = Startup_TurnsToQ24(runtime->open_loop_theta_turns);
            from_speed = runtime->open_loop_speed_rpm;
        }
        runtime->blend_fraction = blend;
        blend_error = RotorFeedback_AngleErrorQ24(
            input->observer_theta_q24, from_theta);
        blended_theta = from_theta + (int32_t)(blend * (float)blend_error);
        run_speed = from_speed + blend
            * ((float)input->observer_speed_rpm - from_speed);
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_BLEND, blended_theta,
            0.0f, (runtime->blend_from_source == ROTOR_SOURCE_HFI)
                ? Startup_SpeedPi(runtime, run_speed)
                : (float)runtime->direction * runtime->config.startup_iq_pu);
        runtime->output.speed_ref_rpm = (runtime->blend_from_source
            == ROTOR_SOURCE_HFI) ? runtime->run_speed_ref_rpm
                                 : runtime->open_loop_speed_rpm;
        if ((runtime->blend_from_source == ROTOR_SOURCE_HFI)
            && (Startup_HfiGood(runtime, input) == 0U)) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_HFI_LOCK_LOST);
        } else if ((Startup_LockGood(runtime, input) == 0U)
            || (Startup_FeedbackAgreement(runtime,
                    input->observer_theta_q24,
                    input->observer_speed_rpm,
                    from_theta, (int32_t)from_speed) == 0U)) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_BLEND_FAILED);
        } else if (runtime->state_cycles >= runtime->config.blend_cycles) {
            runtime->unlock_bad_cycles = 0U;
            runtime->monitor_bad_cycles = 0U;
            runtime->hfi_good_cycles = 0U;
            Startup_Enter(runtime, runtime->blend_from_source == ROTOR_SOURCE_HFI
                ? SENSORLESS_START_RUN : SENSORLESS_START_IQ_HOLD);
        }
        break;

    case SENSORLESS_START_IQ_HOLD:
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_SENSORLESS,
            input->observer_theta_q24, 0.0f, runtime->output.iq_ref_pu);
        Startup_CheckRunMonitors(runtime, input);
        if (runtime->state == SENSORLESS_START_IQ_HOLD
            && runtime->state_cycles >= runtime->config.iq_hold_cycles) {
            runtime->run_speed_ref_rpm = (float)input->observer_speed_rpm;
            runtime->speed_pi_pending = 1U;
            Startup_Enter(runtime, SENSORLESS_START_RUN);
        }
        break;

    case SENSORLESS_START_RUN:
        if (runtime->state_cycles >= runtime->config.run_test_cycles) {
            runtime->debug_completed = 1U;
            SensorlessStartup_RequestStop(runtime);
            break;
        }
        speed_step = runtime->config.run_accel_rpm_s * runtime->config.sample_time_s;
        runtime->run_speed_ref_rpm = Startup_Approach(runtime->run_speed_ref_rpm,
            (float)runtime->target_speed_rpm, speed_step);
        run_speed = (float)input->observer_speed_rpm;
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_SENSORLESS,
            input->observer_theta_q24, 0.0f, Startup_SpeedPi(runtime, run_speed));
        Startup_CheckRunMonitors(runtime, input);
        if ((runtime->state == SENSORLESS_START_RUN)
            && (runtime->config.hfi_enable != 0U)
            && (input->hfi_capable != 0U)
            && (fabsf((float)input->observer_speed_rpm)
                <= runtime->config.sensorless_exit_speed_rpm)) {
            runtime->hfi_fault_snapshot = input->hfi_fault_count;
            runtime->hfi_good_cycles = 0U;
            runtime->hfi_bad_cycles = 0U;
            runtime->unlock_bad_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_HFI_REACQUIRE);
        }
        break;

    case SENSORLESS_START_HFI_REACQUIRE:
        speed_step = runtime->config.run_accel_rpm_s
            * runtime->config.sample_time_s;
        runtime->run_speed_ref_rpm = Startup_Approach(
            runtime->run_speed_ref_rpm, (float)runtime->target_speed_rpm,
            speed_step);
        run_speed = (float)input->observer_speed_rpm;
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_SENSORLESS,
            input->observer_theta_q24, 0.0f,
            (Startup_LockGood(runtime, input) != 0U)
                ? Startup_SpeedPi(runtime, run_speed) : 0.0f);
        runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
        if ((fabsf((float)input->observer_speed_rpm)
                >= runtime->config.handoff_speed_rpm)
            && (fabsf((float)runtime->target_speed_rpm)
                >= runtime->config.handoff_speed_rpm)
            && (Startup_LockGood(runtime, input) != 0U)) {
            runtime->unlock_bad_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_RUN);
            break;
        }
        if ((Startup_HfiGood(runtime, input) != 0U)
            && (Startup_FeedbackAgreement(runtime,
                    input->observer_theta_q24,
                    input->observer_speed_rpm,
                    input->hfi_theta_q24,
                    input->hfi_speed_rpm) != 0U)) {
            if (runtime->hfi_good_cycles
                < runtime->config.hfi_lock_dwell_cycles) {
                runtime->hfi_good_cycles++;
            }
        } else {
            runtime->hfi_good_cycles = 0U;
        }
        if (Startup_LockGood(runtime, input) == 0U) {
            if (runtime->unlock_bad_cycles
                < runtime->config.unlock_dwell_cycles) {
                runtime->unlock_bad_cycles++;
            }
        } else {
            runtime->unlock_bad_cycles = 0U;
        }
        if (runtime->hfi_good_cycles
            >= runtime->config.hfi_lock_dwell_cycles) {
            Startup_Enter(runtime, SENSORLESS_START_BLEND_TO_HFI);
        } else if (runtime->state_cycles
            >= runtime->config.hfi_timeout_cycles) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_HFI_LOCK_TIMEOUT);
        }
        break;

    case SENSORLESS_START_BLEND_TO_HFI:
        runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
        if ((Startup_HfiGood(runtime, input) == 0U)
            || (Startup_FeedbackAgreement(runtime,
                    input->observer_theta_q24,
                    input->observer_speed_rpm,
                    input->hfi_theta_q24,
                    input->hfi_speed_rpm) == 0U)) {
            runtime->hfi_good_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_HFI_REACQUIRE);
            break;
        }
        blend = (float)runtime->state_cycles / (float)runtime->config.blend_cycles;
        blend = Startup_Clamp(blend, 0.0f, 1.0f);
        blend = blend * blend * (3.0f - 2.0f * blend);
        blend_error = RotorFeedback_AngleErrorQ24(
            input->hfi_theta_q24, input->observer_theta_q24);
        blended_theta = input->observer_theta_q24
            + (int32_t)(blend * (float)blend_error);
        run_speed = (float)input->observer_speed_rpm + blend
            * ((float)input->hfi_speed_rpm
                - (float)input->observer_speed_rpm);
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_BLEND,
            blended_theta, 0.0f, Startup_SpeedPi(runtime, run_speed));
        if ((fabsf((float)input->observer_speed_rpm)
                >= runtime->config.handoff_speed_rpm)
            && (fabsf((float)runtime->target_speed_rpm)
                >= runtime->config.handoff_speed_rpm)
            && (Startup_LockGood(runtime, input) != 0U)) {
            Startup_Enter(runtime, SENSORLESS_START_RUN);
        } else if (runtime->state_cycles >= runtime->config.blend_cycles) {
            runtime->hfi_bad_cycles = 0U;
            runtime->lock_good_cycles = 0U;
            Startup_Enter(runtime, SENSORLESS_START_HFI_RUN);
        }
        break;

    case SENSORLESS_START_ENCODER_FALLBACK:
        speed_step = runtime->config.run_accel_rpm_s * runtime->config.sample_time_s;
        runtime->run_speed_ref_rpm = Startup_Approach(runtime->run_speed_ref_rpm,
            (float)runtime->target_speed_rpm, speed_step);
        Startup_SetActiveOutput(runtime, ROTOR_SOURCE_ENCODER,
            input->encoder_theta_q24, 0.0f,
            Startup_SpeedPi(runtime, (float)input->encoder_speed_rpm));
        if (input->encoder_valid == 0U) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_LOCK_LOST);
        }
        break;

    case SENSORLESS_START_STOPPING:
        runtime->run_speed_ref_rpm = Startup_Approach(runtime->run_speed_ref_rpm,
            0.0f, runtime->config.run_accel_rpm_s * runtime->config.sample_time_s);
        stop_theta = runtime->output.theta_e_q24;
        stop_source = runtime->output.source;
        stop_feedback_safe = 0U;
        run_speed = (float)input->observer_speed_rpm;

        if ((Startup_HfiGood(runtime, input) != 0U)
            && (fabsf(run_speed)
                <= runtime->config.handoff_speed_rpm)) {
            stop_theta = input->hfi_theta_q24;
            stop_source = ROTOR_SOURCE_HFI;
            run_speed = (float)input->hfi_speed_rpm;
            stop_feedback_safe = 1U;
            runtime->output.fault_flags |= SENSORLESS_STATUS_HFI_ACTIVE;
        } else if (Startup_LockGood(runtime, input) != 0U) {
            stop_theta = input->observer_theta_q24;
            stop_source = ROTOR_SOURCE_SENSORLESS;
            stop_feedback_safe = 1U;
        }

        stop_iq = (stop_feedback_safe != 0U)
            ? Startup_SpeedPi(runtime, run_speed) : 0.0f;
        Startup_SetActiveOutput(runtime, stop_source, stop_theta, 0.0f, stop_iq);
        if ((fabsf(runtime->run_speed_ref_rpm) < 1.0f)
            && (fabsf(run_speed) <= (float)runtime->config.stop_speed_rpm)) {
            if (runtime->stop_good_cycles < runtime->config.stop_dwell_cycles) {
                runtime->stop_good_cycles++;
            }
        } else {
            runtime->stop_good_cycles = 0U;
        }
        if (runtime->stop_good_cycles >= runtime->config.stop_dwell_cycles) {
            Startup_Enter(runtime, SENSORLESS_START_IDLE);
        } else if (runtime->state_cycles >= runtime->config.stop_timeout_cycles) {
            Startup_EnterFault(runtime, SENSORLESS_FAULT_STOP_TIMEOUT);
        }
        break;

    case SENSORLESS_START_FAULT:
        runtime->output.fault_request = 1U;
        runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
        break;

    default:
        Startup_EnterFault(runtime, SENSORLESS_FAULT_PARAM_INVALID);
        break;
    }
    if (SENSORLESS_IS_PROBE(runtime->config.debug_mode)
        && runtime->output.control_active && input->drive_enabled
        && runtime->state != SENSORLESS_START_FAULT) {
        uint32_t result;
        runtime->probe.a2 = runtime->config.debug_mode == SENSORLESS_DEBUG_SMO_A2;
        runtime->probe.a3 = runtime->config.debug_mode == SENSORLESS_DEBUG_SMO_A3;
        runtime->probe.frame = runtime->config.debug_mode == SENSORLESS_DEBUG_SMO_FRAME;
        result = SlProbe_Step(&runtime->probe, &input->probe,
            runtime->state == SENSORLESS_START_ALIGN, runtime->state_cycles,
            runtime->direction, Startup_TurnsToQ24(runtime->open_loop_theta_turns),
            runtime->open_loop_speed_rpm, &runtime->output.theta_e_q24,
            &runtime->output.id_ref_pu, &runtime->output.iq_ref_pu);
        if(runtime->probe.frame_committed && runtime->probe.phase==SL_P_HOLD)
            runtime->output.source=ROTOR_SOURCE_SENSORLESS;
        if (result == 2U) Startup_EnterFault(runtime, SENSORLESS_FAULT_CURRENT_INPUT);
        else if (result == 1U) {
            runtime->output.control_active = runtime->output.current_override = 0U;
            runtime->output.angle_override = runtime->start_requested = 0U;
            runtime->output.id_ref_pu = runtime->output.iq_ref_pu = 0.0f;
            Startup_Enter(runtime, SENSORLESS_START_IDLE);
        }
    }
}
