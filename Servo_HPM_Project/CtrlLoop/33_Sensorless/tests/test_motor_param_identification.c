#include <math.h>
#include <stdio.h>
#include <string.h>

#include "MotorParamIdentification.h"

static int g_failures;

static void Test_Check(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static int Test_Near(float value, float expected, float tolerance)
{
    return fabsf(value - expected) <= tolerance;
}

static MOTOR_PARAM_IDENT_CONFIG Test_Config(void)
{
    MOTOR_PARAM_IDENT_CONFIG config;

    MotorParamIdentification_DefaultConfig(&config,
        0.0001f, 1000.0f, 4.0f, 200.0f, 10.0f, 0.5f);
    config.resistance_current_pu = 0.05f;
    config.inductance_current_pu = 0.05f;
    config.flux_speed_rpm = 100.0f;
    config.flux_speed_tolerance_rpm = 10.0f;
    config.stationary_speed_limit_rpm = 20.0f;
    config.minimum_current_delta_pu = 0.00001f;
    config.minimum_flux_voltage_pu = 0.0001f;
    config.minimum_hfi_quality = 600U;
    config.hfi_lock_cycles = 2U;
    config.settle_cycles = 2U;
    config.average_cycles = 2U;
    config.inductance_half_period_cycles = 2U;
    config.inductance_test_cycles = 8U;
    config.flux_settle_cycles = 2U;
    config.flux_average_cycles = 4U;
    config.timeout_cycles = 1000U;
    return config;
}

static void Test_HfiAcquireAndResistance(void)
{
    MOTOR_PARAM_IDENT_CONFIG config = Test_Config();
    MOTOR_PARAM_IDENT_RUNTIME runtime;
    MOTOR_PARAM_IDENT_INPUT input;

    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    input.hfi_locked = 1U;
    input.hfi_quality = 900U;
    input.hfi_theta_turns = 0.125f;
    MotorParamIdentification_Init(&runtime, &config);
    Test_Check(MotorParamIdentification_Start(&runtime) != 0U,
               "valid identification did not start");
    MotorParamIdentification_Step(&runtime, &input);
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_RS_POS_SETTLE,
               "HFI lock did not advance to resistance test");
    Test_Check(Test_Near(runtime.captured_theta_turns, 0.125f, 1.0e-6f),
               "HFI rotor angle was not captured");

    runtime.state = MOTOR_IDENT_RS_NEG_SAMPLE;
    runtime.state_cycles = 0U;
    runtime.sample_count = 1U;
    runtime.rs_pos_current = 0.05f;
    runtime.rs_pos_voltage = 0.005f;
    runtime.sum_current = -0.05f;
    runtime.sum_voltage = -0.005f;
    input.hfi_theta_turns = 0.0f;
    input.current_alpha_pu = -0.05f;
    input.current_beta_pu = 0.0f;
    input.voltage_alpha_pu = -0.005f;
    input.voltage_beta_pu = 0.0f;
    runtime.captured_theta_turns = 0.0f;
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_ZERO_BEFORE_LD,
               "bipolar resistance test did not complete");
    Test_Check(Test_Near(runtime.result.rs_pu, 0.1f, 1.0e-5f),
               "bipolar resistance estimate is incorrect");
}

static void Test_InductanceFluxAndSiConversion(void)
{
    MOTOR_PARAM_IDENT_CONFIG config = Test_Config();
    MOTOR_PARAM_IDENT_RUNTIME runtime;
    MOTOR_PARAM_IDENT_INPUT input;
    float expected_ld = 0.02f;
    float expected_lq = 0.03f;
    float expected_flux = 0.40f;

    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    MotorParamIdentification_Init(&runtime, &config);
    runtime.result.rs_pu = 0.10f;

    runtime.state = MOTOR_IDENT_LD_TEST;
    runtime.state_cycles = config.inductance_test_cycles - 1U;
    runtime.sample_count = 8U;
    runtime.inductance_denominator = 1.0f;
    runtime.inductance_numerator = expected_ld
        / (config.base_electrical_rad_s * config.sample_time_s);
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_ZERO_BEFORE_LQ,
               "Ld test did not complete");
    Test_Check(Test_Near(runtime.result.ld_pu, expected_ld, 1.0e-6f),
               "Ld least-squares result is incorrect");

    runtime.state = MOTOR_IDENT_LQ_TEST;
    runtime.state_cycles = config.inductance_test_cycles - 1U;
    runtime.sample_count = 8U;
    runtime.inductance_denominator = 1.0f;
    runtime.inductance_numerator = expected_lq
        / (config.base_electrical_rad_s * config.sample_time_s);
    runtime.previous_sample_valid = 0U;
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_ZERO_BEFORE_FLUX,
               "Lq test did not complete");
    Test_Check(Test_Near(runtime.result.lq_pu, expected_lq, 1.0e-6f),
               "Lq least-squares result is incorrect");

    runtime.state = MOTOR_IDENT_FLUX_SAMPLE;
    runtime.state_cycles = config.flux_average_cycles - 1U;
    runtime.result.flux_samples = config.flux_average_cycles;
    runtime.flux_sum = expected_flux
        * (float)runtime.result.flux_samples;
    runtime.flux_sum_square = expected_flux * expected_flux
        * (float)runtime.result.flux_samples;
    input.startup_running = 1U;
    input.feedback_valid = 1U;
    input.speed_rpm = config.flux_speed_rpm;
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_STOPPING,
               "flux test did not request a controlled stop");
    Test_Check(runtime.output.request_stop != 0U,
               "flux completion did not request stop");

    input.startup_idle = 1U;
    input.startup_running = 0U;
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_COMPLETE,
               "identification did not complete after stop");
    Test_Check(runtime.result.valid != 0U,
               "completed identification result was not valid");
    Test_Check(Test_Near(runtime.result.ls_pu, 0.025f, 1.0e-6f),
               "Ls is not the mean of Ld and Lq");
    Test_Check(Test_Near(runtime.result.flux_pu, expected_flux, 1.0e-6f),
               "flux average is incorrect");
    Test_Check(Test_Near(runtime.result.rs_ohm, 2.0f, 1.0e-5f),
               "per-unit resistance was not converted to ohms");
    Test_Check(Test_Near(runtime.result.ld_h, 0.0004f, 1.0e-7f),
               "per-unit Ld was not converted to henries");
    Test_Check(Test_Near(runtime.result.flux_wb, 0.08f, 1.0e-5f),
               "per-unit flux was not converted to webers");
}

static void Test_SafetyFailures(void)
{
    MOTOR_PARAM_IDENT_CONFIG config = Test_Config();
    MOTOR_PARAM_IDENT_RUNTIME runtime;
    MOTOR_PARAM_IDENT_INPUT input;

    memset(&input, 0, sizeof(input));
    input.drive_enabled = 1U;
    MotorParamIdentification_Init(&runtime, &config);
    MotorParamIdentification_Start(&runtime);
    runtime.state = MOTOR_IDENT_LQ_TEST;
    input.speed_rpm = config.stationary_speed_limit_rpm + 1.0f;
    MotorParamIdentification_Step(&runtime, &input);
    Test_Check(runtime.state == MOTOR_IDENT_FAILED,
               "stationary over-speed did not fail identification");
    Test_Check((runtime.fault_flags & MOTOR_IDENT_FAULT_EXCESS_SPEED) != 0U,
               "stationary over-speed fault flag is missing");
    Test_Check(runtime.output.request_stop != 0U,
               "identification failure did not request stop");
}

int main(void)
{
    MOTOR_PARAM_IDENT_CONFIG config = Test_Config();

    Test_Check(MotorParamIdentification_ValidateConfig(&config) != 0U,
               "default test configuration is invalid");
    Test_HfiAcquireAndResistance();
    Test_InductanceFluxAndSiConversion();
    Test_SafetyFailures();

    if (g_failures == 0) {
        printf("PASS: motor parameter identification tests\n");
        return 0;
    }
    printf("FAILURES: %d\n", g_failures);
    return 1;
}
