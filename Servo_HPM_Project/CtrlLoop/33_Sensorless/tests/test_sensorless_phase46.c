#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "MotorParamProfile.h"
#include "SensorlessStartup.h"

static int g_failures;

static void Test_Check(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static SENSORLESS_MOTOR_RAW Test_ValidRawMotor(void)
{
    SENSORLESS_MOTOR_RAW raw;

    raw.sample_hz = 16000.0f;
    raw.stator_resistance_pu = 0.05f;
    raw.ld_pu = 0.020f;
    raw.lq_pu = 0.025f;
    raw.speed_base_rpm = 3000.0f;
    raw.max_speed_rpm = 6000.0f;
    raw.pole_pairs = 4.0f;
    raw.rated_current_pu = 0.50f;
    raw.current_limit_pu = 0.80f;
    return raw;
}

static void Test_MotorProfile(void)
{
    SENSORLESS_MOTOR_RAW raw = Test_ValidRawMotor();
    SENSORLESS_MOTOR_OVERRIDE override_values;
    SENSORLESS_MOTOR_PROFILE profile;
    SMO_PLL_PARAMS params;

    MotorParamProfile_ClearOverride(&override_values);
    MotorParamProfile_Build(&raw, &override_values, &profile);
    Test_Check(profile.valid_for_control != 0U,
               "valid motor profile was rejected");
    Test_Check(profile.effective.pole_pairs == 4.0f,
               "pole-pair snapshot is wrong");
    Test_Check(profile.hfi_capable != 0U,
               "salient motor was not marked HFI-capable");
    Test_Check((profile.observer_model_gain >= 0.0001f)
               && (profile.observer_model_gain <= 0.50f),
               "derived observer gain is outside limits");

    memset(&params, 0, sizeof(params));
    MotorParamProfile_ApplyToSmo(&profile, &params);
    Test_Check(fabsf(params.sample_time_s - (1.0f / 16000.0f)) < 1.0e-8f,
               "SMO sample time was not derived from the motor profile");

    raw.lq_pu = raw.ld_pu;
    MotorParamProfile_Build(&raw, 0, &profile);
    Test_Check(profile.hfi_capable == 0U,
               "non-salient motor was marked HFI-capable");
    Test_Check((profile.validation_flags & MOTOR_PARAM_FLAG_HFI_SALIENCY) != 0U,
               "low saliency was not reported");

    raw.lq_pu = 0.025f;
    raw.ld_pu = 0.0f;
    MotorParamProfile_Build(&raw, 0, &profile);
    Test_Check(profile.valid_for_shadow != 0U,
               "invalid profile must remain usable for shadow diagnostics");
    Test_Check(profile.valid_for_control == 0U,
               "invalid inductance must block sensorless control");
    Test_Check((profile.validation_flags & MOTOR_PARAM_FLAG_LD) != 0U,
               "invalid Ld was not reported");

    MotorParamProfile_ClearOverride(&override_values);
    override_values.mask = MOTOR_PARAM_OVERRIDE_LD;
    override_values.ld_pu = 0.020f;
    MotorParamProfile_Build(&raw, &override_values, &profile);
    Test_Check(profile.valid_for_control != 0U,
               "validated recipe override did not repair the profile");
    Test_Check(profile.override_mask == MOTOR_PARAM_OVERRIDE_LD,
               "override provenance was not retained");
}

static void Test_FastStartupConfig(SENSORLESS_STARTUP_CONFIG *config)
{
    SensorlessStartup_DefaultConfig(config, 0.001f, 4.0f,
                                    3000.0f, 3000.0f, 0.20f);
    config->align_cycles = 3U;
    config->current_ramp_cycles = 1U;
    config->iq_hold_cycles = 1U;
    config->handoff_speed_rpm = 60.0f;
    config->sensorless_exit_speed_rpm = 30.0f;
    config->hfi_enable = 0U;
    config->allow_open_loop_fallback = 1U;
    config->open_loop_accel_rpm_s = 60000.0f;
    config->lock_dwell_cycles = 2U;
    config->blend_cycles = 3U;
    config->lock_timeout_cycles = 30U;
    config->unlock_dwell_cycles = 3U;
    config->monitor_dwell_cycles = 3U;
    config->fallback_ready_cycles = 3U;
    config->stop_dwell_cycles = 3U;
    config->stop_timeout_cycles = 30U;
    config->run_accel_rpm_s = 60000.0f;
    config->minimum_lock_quality = 400U;
}

static void Test_DriveToRun(SENSORLESS_STARTUP_RUNTIME *runtime,
                            SENSORLESS_STARTUP_INPUT *input)
{
    uint32_t index;

    SensorlessStartup_RequestStart(runtime, 1, 500);
    SensorlessStartup_Step(runtime, input);
    for (index = 0U; index < runtime->config.align_cycles; index++) {
        SensorlessStartup_Step(runtime, input);
    }
    SensorlessStartup_Step(runtime, input);

    input->observer_locked = 1U;
    input->observer_quality = 900U;
    for (index = 0U; index < runtime->config.lock_dwell_cycles; index++) {
        input->observer_theta_q24 = runtime->output.theta_e_q24;
        input->encoder_theta_q24 = runtime->output.theta_e_q24;
        input->observer_speed_rpm = 60;
        input->encoder_speed_rpm = 60;
        SensorlessStartup_Step(runtime, input);
    }
    for (index = 0U; index < runtime->config.blend_cycles; index++) {
        input->observer_theta_q24 = runtime->output.theta_e_q24;
        input->encoder_theta_q24 = runtime->output.theta_e_q24;
        SensorlessStartup_Step(runtime, input);
    }
    SensorlessStartup_Step(runtime, input); /* fixed-Iq hold before speed PI */
}

static void Test_StartupAndFallback(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_STARTUP_RUNTIME runtime;
    SENSORLESS_STARTUP_INPUT input;
    uint32_t index;

    Test_FastStartupConfig(&config);
    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.encoder_valid = 1U;

    Test_DriveToRun(&runtime, &input);
    Test_Check(runtime.state == SENSORLESS_START_RUN,
               "ALIGN/I-F/lock/blend sequence did not reach RUN");
    Test_Check(runtime.output.control_active != 0U,
               "RUN did not retain FOC override ownership");
    Test_Check(SensorlessStartup_ValidateConfig(&config) != 0U,
               "valid startup configuration was rejected");

    runtime.config.hfi_enable = 1U;
    input.hfi_capable = 1U;
    input.observer_locked = 0U;
    input.observer_quality = 0U;
    for (index = 0U; index < config.unlock_dwell_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_HFI_REACQUIRE,
               "loss of SMO lock did not start HFI reacquisition");
    SensorlessStartup_Step(&runtime, &input);
    Test_Check(runtime.output.iq_ref_pu == 0.0f,
               "SMO loss retained speed-loop torque before HFI reacquired");
    input.hfi_locked = 1U;
    input.hfi_quality = 900U;
    input.hfi_theta_q24 = input.observer_theta_q24;
    input.hfi_speed_rpm = 20;
    for (index = 0U; index < config.hfi_lock_dwell_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    for (index = 0U; index < config.blend_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_HFI_RUN,
               "HFI did not take over after SMO loss");
    Test_Check((runtime.output.fault_flags & SENSORLESS_FAULT_LOCK_LOST) != 0U,
               "loss of SMO lock was not retained for diagnostics");
}

static void Test_HfiLowSpeedAndBidirectionalBlend(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_STARTUP_RUNTIME runtime;
    SENSORLESS_STARTUP_INPUT input;
    uint32_t index;

    Test_FastStartupConfig(&config);
    config.hfi_enable = 1U;
    config.allow_open_loop_fallback = 0U;
    config.hfi_lock_dwell_cycles = 2U;
    config.hfi_timeout_cycles = 20U;
    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.hfi_capable = 1U;
    input.hfi_locked = 1U;
    input.hfi_quality = 900U;

    Test_Check(SensorlessStartup_RequestStart(&runtime, 1, 20) != 0U,
               "zero/low-speed HFI start request was rejected");
    SensorlessStartup_Step(&runtime, &input);
    for (index = 0U; index < config.align_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    for (index = 0U; index < config.hfi_lock_dwell_cycles; index++) {
        input.hfi_theta_q24 = runtime.output.theta_e_q24;
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_HFI_RUN,
               "HFI acquisition did not reach low-speed run");
    Test_Check(runtime.output.source == ROTOR_SOURCE_HFI,
               "low-speed run did not use HFI angle");

    Test_Check(SensorlessStartup_UpdateTarget(&runtime, 1, 500) != 0U,
               "active HFI target update was rejected");
    input.hfi_speed_rpm = 60;
    input.observer_speed_rpm = 60;
    input.observer_locked = 1U;
    input.observer_quality = 900U;
    input.observer_theta_q24 = input.hfi_theta_q24;
    for (index = 0U; index < config.lock_dwell_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    for (index = 0U; index < config.blend_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_RUN,
               "HFI-to-SMO blend did not reach sensorless RUN");

    Test_Check(SensorlessStartup_UpdateTarget(&runtime, 1, 0) != 0U,
               "zero-speed target update was rejected");
    input.observer_speed_rpm = 20;
    input.hfi_speed_rpm = 20;
    SensorlessStartup_Step(&runtime, &input);
    Test_Check(runtime.state == SENSORLESS_START_HFI_REACQUIRE,
               "low-speed hysteresis did not start HFI reacquisition");
    for (index = 0U; index < config.hfi_lock_dwell_cycles; index++) {
        input.hfi_theta_q24 = input.observer_theta_q24;
        SensorlessStartup_Step(&runtime, &input);
    }
    for (index = 0U; index < config.blend_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_HFI_RUN,
               "SMO-to-HFI blend did not return to low-speed run");
}

static void Test_ControlledStop(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_STARTUP_RUNTIME runtime;
    SENSORLESS_STARTUP_INPUT input;
    uint32_t index;

    Test_FastStartupConfig(&config);
    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.encoder_valid = 1U;
    Test_DriveToRun(&runtime, &input);

    for (index = 0U; index < config.fallback_ready_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    SensorlessStartup_RequestStop(&runtime);
    input.observer_locked = 1U;
    input.observer_quality = 900U;
    input.observer_speed_rpm = 0;
    input.encoder_speed_rpm = 0;
    for (index = 0U; index < 10U; index++) {
        SensorlessStartup_Step(&runtime, &input);
        if (runtime.state == SENSORLESS_START_IDLE) {
            break;
        }
    }
    Test_Check(runtime.state == SENSORLESS_START_IDLE,
               "controlled stop did not wait for zero speed and return IDLE");

    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.encoder_valid = 1U;
    Test_DriveToRun(&runtime, &input);
    SensorlessStartup_RequestStop(&runtime);
    runtime.config.stop_timeout_cycles = 3U;
    input.observer_locked = 0U;
    input.encoder_valid = 0U;
    input.observer_speed_rpm = 500;
    for (index = 0U; index < 3U; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_FAULT,
               "stop timeout did not latch a fault");
    Test_Check((runtime.output.fault_flags & SENSORLESS_FAULT_STOP_TIMEOUT) != 0U,
               "stop-timeout fault bit is missing");
}

static void Test_UnsafeLossAndMonitorFault(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_STARTUP_RUNTIME runtime;
    SENSORLESS_STARTUP_INPUT input;
    uint32_t index;

    Test_FastStartupConfig(&config);
    config.hfi_enable = 1U;
    config.allow_open_loop_fallback = 0U;
    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.hfi_capable = 0U;
    SensorlessStartup_RequestStart(&runtime, 1, 20);
    SensorlessStartup_Step(&runtime, &input);
    for (index = 0U; index < config.align_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_FAULT,
               "non-salient motor was allowed into HFI low-speed control");
    Test_Check((runtime.output.fault_flags
                & SENSORLESS_FAULT_HFI_UNAVAILABLE) != 0U,
               "HFI-unavailable fault bit is missing");

    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.params_valid = 0U;
    SensorlessStartup_RequestStart(&runtime, 1, 500);
    SensorlessStartup_Step(&runtime, &input);
    Test_Check(runtime.state == SENSORLESS_START_FAULT,
               "invalid motor parameters did not block startup");
    Test_Check((runtime.output.fault_flags & SENSORLESS_FAULT_PARAM_INVALID) != 0U,
               "parameter-validation fault bit is missing");
}

static void Test_EstimatorAgreementGatesHandoff(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_STARTUP_RUNTIME runtime;
    SENSORLESS_STARTUP_INPUT input;
    uint32_t index;

    Test_FastStartupConfig(&config);
    config.hfi_enable = 1U;
    config.allow_open_loop_fallback = 0U;
    SensorlessStartup_Init(&runtime, &config);
    memset(&input, 0, sizeof(input));
    runtime.state = SENSORLESS_START_HFI_RUN;
    runtime.start_requested = 1U;
    runtime.target_speed_rpm = 500;
    runtime.run_speed_ref_rpm = config.handoff_speed_rpm;
    input.drive_enabled = 1U;
    input.params_valid = 1U;
    input.hfi_capable = 1U;
    input.hfi_locked = 1U;
    input.hfi_quality = 900U;
    input.hfi_speed_rpm = 60;
    input.hfi_theta_q24 = 100000;
    input.observer_locked = 1U;
    input.observer_quality = 900U;
    input.observer_speed_rpm = 500;
    input.observer_theta_q24 = input.hfi_theta_q24;

    for (index = 0U; index < (config.lock_dwell_cycles + 2U); index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_HFI_RUN,
               "SMO/HFI speed disagreement was allowed to hand off");

    input.observer_speed_rpm = input.hfi_speed_rpm;
    for (index = 0U; index < config.lock_dwell_cycles; index++) {
        SensorlessStartup_Step(&runtime, &input);
    }
    Test_Check(runtime.state == SENSORLESS_START_BLEND,
               "agreeing SMO/HFI feedback did not start handoff");

    input.observer_speed_rpm = 500;
    SensorlessStartup_Step(&runtime, &input);
    Test_Check(runtime.state == SENSORLESS_START_FAULT
               && runtime.failure.fault == SENSORLESS_FAULT_BLEND_FAILED,
               "speed disagreement during blend did not latch failure");
}

int main(void)
{
    Test_MotorProfile();
    Test_StartupAndFallback();
    Test_HfiLowSpeedAndBidirectionalBlend();
    Test_ControlledStop();
    Test_UnsafeLossAndMonitorFault();
    Test_EstimatorAgreementGatesHandoff();

    if (g_failures == 0) {
        printf("PASS: sensorless phase-4-to-6 unit tests\n");
        return 0;
    }

    printf("FAILURES: %d\n", g_failures);
    return 1;
}
