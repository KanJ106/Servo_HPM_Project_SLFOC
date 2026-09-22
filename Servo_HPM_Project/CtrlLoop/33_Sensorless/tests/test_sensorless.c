#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "RotorFeedback.h"
#include "SmoPll.h"

#define TEST_TWO_PI_F (6.2831853071795864769f)

static int g_failures;

static void Test_Check(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static float Test_WrapTurns(float turns)
{
    turns -= floorf(turns);
    if (turns < 0.0f) {
        turns += 1.0f;
    }
    return turns;
}

static float Test_AngleErrorTurns(float angle, float reference)
{
    float error = Test_WrapTurns(angle) - Test_WrapTurns(reference);
    if (error >= 0.5f) {
        error -= 1.0f;
    } else if (error < -0.5f) {
        error += 1.0f;
    }
    return error;
}

static void Test_RotorFeedbackMux(void)
{
    ROTOR_FEEDBACK_MUX mux;
    ROTOR_FEEDBACK encoder = {0};
    ROTOR_FEEDBACK sensorless = {0};
    const ROTOR_FEEDBACK *selected;

    RotorFeedbackMux_Init(&mux);
    encoder.theta_e_q24 = 1234;
    encoder.valid = 1U;
    sensorless.theta_e_q24 = 5678;
    sensorless.valid = 1U;
    RotorFeedbackMux_UpdateEncoder(&mux, &encoder);
    RotorFeedbackMux_UpdateSensorless(&mux, &sensorless);

    selected = RotorFeedbackMux_Select(&mux);
    Test_Check(selected->source == ROTOR_SOURCE_ENCODER,
               "shadow mode must keep encoder in control");
    Test_Check(selected->theta_e_q24 == 1234,
               "shadow mode selected the wrong angle");

    RotorFeedbackMux_SetMode(&mux, ROTOR_MODE_SENSORLESS_ONLY);
    selected = RotorFeedbackMux_Select(&mux);
    Test_Check(selected->source == ROTOR_SOURCE_SENSORLESS,
               "sensorless-only mode must select a valid observer");

    sensorless.valid = 0U;
    RotorFeedbackMux_UpdateSensorless(&mux, &sensorless);
    selected = RotorFeedbackMux_Select(&mux);
    Test_Check(selected->source == ROTOR_SOURCE_SENSORLESS,
               "sensorless-only mode must expose invalid observer state");

    Test_Check(RotorFeedback_WrapAngleQ24(-1) == (ROTOR_ANGLE_ONE_TURN_Q24 - 1),
               "negative angle wrap failed");
    Test_Check(RotorFeedback_AngleErrorQ24(1, ROTOR_ANGLE_ONE_TURN_Q24 - 1) == 2,
               "angle error wrap failed");
}

static void Test_DefaultSmoParams(SMO_PLL_PARAMS *params)
{
    params->sample_time_s = 1.0f / 16000.0f;
    params->stator_resistance_pu = 0.05f;
    params->current_model_gain = 0.04f;
    params->switching_gain = 0.25f;
    params->boundary_pu = 0.01f;
    params->emf_lpf_alpha = 0.08f;
    params->emf_delay_comp_gain = 1.0f;
    params->pll_kp = 240.0f;
    params->pll_ki = 12000.0f;
    params->pll_speed_lpf_alpha = 0.05f;
    params->weak_emf_speed_decay = 0.001f;
    params->min_emf_pu = 0.01f;
    params->unlock_emf_ratio = 0.70f;
    params->lock_phase_error = 0.35f;
    params->max_electrical_rad_s = 2000.0f;
    params->max_accel_e_rad_s2 = 50000.0f;
    params->current_hat_limit_pu = 2.0f;
    params->lock_cycles = 400U;
    params->unlock_cycles = 80U;
}

static void Test_RunSyntheticEmf(SMO_PLL_STATE *state,
                                 const SMO_PLL_PARAMS *params,
                                 float *theta_turns,
                                 float electrical_rad_s,
                                 float emf_amplitude,
                                 int16_t direction,
                                 uint32_t cycles)
{
    SMO_PLL_INPUT input;
    float theta_rad;
    float signed_emf;
    uint32_t index;

    for (index = 0U; index < cycles; index++) {
        *theta_turns = Test_WrapTurns(*theta_turns
            + electrical_rad_s * params->sample_time_s / TEST_TWO_PI_F);
        theta_rad = *theta_turns * TEST_TWO_PI_F;
        signed_emf = (direction < 0) ? -emf_amplitude : emf_amplitude;
        input.voltage_alpha_pu = -signed_emf * sinf(theta_rad);
        input.voltage_beta_pu = signed_emf * cosf(theta_rad);
        input.current_alpha_pu = 0.0f;
        input.current_beta_pu = 0.0f;
        input.expected_direction = direction;
        SmoPll_Step(state, params, &input);
    }
}

static void Test_ForwardSmoPllTracking(void)
{
    SMO_PLL_PARAMS params;
    SMO_PLL_STATE state;
    const float electrical_rad_s = TEST_TWO_PI_F * 80.0f;
    float theta_turns = 0.0f;
    float angle_error;

    Test_DefaultSmoParams(&params);
    Test_Check(SmoPll_ValidateParams(&params) != 0U,
               "default SMO/PLL parameters were rejected");
    SmoPll_Init(&state, &params, theta_turns);
    Test_RunSyntheticEmf(&state, &params, &theta_turns,
        electrical_rad_s, 0.12f, 1, 32000U);

    angle_error = fabsf(Test_AngleErrorTurns(
        state.theta_pll_turns, theta_turns));
    Test_Check(state.locked != 0U,
               "PLL did not lock on synthetic forward EMF");
    Test_Check(angle_error < 0.08f,
               "PLL angle error exceeds 0.08 electrical turn");
    Test_Check(fabsf(state.omega_e_rad_s - electrical_rad_s) < 100.0f,
               "PLL electrical speed error is too large");
    Test_Check(state.emf_confidence > 0.80f,
               "valid forward EMF produced low confidence");
}

static void Test_ReverseSmoPllTracking(void)
{
    SMO_PLL_PARAMS params;
    SMO_PLL_STATE state;
    const float electrical_rad_s = -TEST_TWO_PI_F * 60.0f;
    float theta_turns = 0.35f;
    float angle_error;

    Test_DefaultSmoParams(&params);
    SmoPll_Init(&state, &params, theta_turns);
    Test_RunSyntheticEmf(&state, &params, &theta_turns,
        electrical_rad_s, 0.10f, -1, 32000U);

    angle_error = fabsf(Test_AngleErrorTurns(
        state.theta_pll_turns, theta_turns));
    Test_Check(state.locked != 0U,
               "PLL did not lock on synthetic reverse EMF");
    Test_Check(angle_error < 0.08f,
               "reverse PLL angle error exceeds 0.08 electrical turn");
    Test_Check(fabsf(state.omega_e_rad_s - electrical_rad_s) < 100.0f,
               "reverse PLL electrical speed error is too large");
    Test_Check(state.phase_advance_turns < 0.0f,
               "reverse LPF compensation used the wrong sign");
}

static void Test_EmfDelayCompensation(void)
{
    SMO_PLL_PARAMS compensated_params;
    SMO_PLL_PARAMS uncompensated_params;
    SMO_PLL_STATE compensated;
    SMO_PLL_STATE uncompensated;
    const float electrical_rad_s = TEST_TWO_PI_F * 120.0f;
    float compensated_theta = 0.0f;
    float uncompensated_theta = 0.0f;
    float compensated_error;
    float uncompensated_error;

    Test_DefaultSmoParams(&compensated_params);
    uncompensated_params = compensated_params;
    uncompensated_params.emf_delay_comp_gain = 0.0f;
    SmoPll_Init(&compensated, &compensated_params, 0.0f);
    SmoPll_Init(&uncompensated, &uncompensated_params, 0.0f);
    Test_RunSyntheticEmf(&compensated, &compensated_params,
        &compensated_theta, electrical_rad_s, 0.14f, 1, 32000U);
    Test_RunSyntheticEmf(&uncompensated, &uncompensated_params,
        &uncompensated_theta, electrical_rad_s, 0.14f, 1, 32000U);

    compensated_error = fabsf(Test_AngleErrorTurns(
        compensated.theta_pll_turns, compensated_theta));
    uncompensated_error = fabsf(Test_AngleErrorTurns(
        uncompensated.theta_pll_turns, uncompensated_theta));
    Test_Check(compensated.phase_advance_turns > 0.0f,
               "forward LPF delay compensation was not applied");
    Test_Check(compensated_error <= (uncompensated_error + 0.005f),
               "LPF delay compensation increased steady-state angle error");
}

static void Test_WeakEmfDecayAndNumericGuard(void)
{
    SMO_PLL_PARAMS params;
    SMO_PLL_STATE state;
    SMO_PLL_INPUT input = {0};
    uint32_t index;

    Test_DefaultSmoParams(&params);
    SmoPll_Init(&state, &params, 0.20f);
    SmoPll_SeedTracking(&state, 0.20f, 500.0f);
    input.expected_direction = 1;
    for (index = 0U; index < 12000U; index++) {
        SmoPll_Step(&state, &params, &input);
    }
    Test_Check(state.locked == 0U,
               "weak EMF incorrectly produced observer lock");
    Test_Check(state.quality == 0U,
               "weak EMF incorrectly retained observer quality");
    Test_Check(fabsf(state.omega_e_rad_s) < 1.0f,
               "weak-EMF speed did not decay toward zero");
    Test_Check(state.weak_emf_counter >= 12000U,
               "weak-EMF dwell counter did not advance");

    input.voltage_alpha_pu = NAN;
    SmoPll_Step(&state, &params, &input);
    Test_Check(state.numeric_fault_count == 1U,
               "non-finite observer input was not rejected");
    Test_Check(isfinite(state.theta_pll_turns),
               "numeric recovery left a non-finite PLL angle");

    params.emf_lpf_alpha = 0.0f;
    Test_Check(SmoPll_ValidateParams(&params) == 0U,
               "unsafe EMF filter coefficient was accepted");
}

int main(void)
{
    Test_RotorFeedbackMux();
    Test_ForwardSmoPllTracking();
    Test_ReverseSmoPllTracking();
    Test_EmfDelayCompensation();
    Test_WeakEmfDecayAndNumericGuard();

    if (g_failures == 0) {
        printf("PASS: sensorless phase-3 unit tests\n");
        return 0;
    }

    printf("FAILURES: %d\n", g_failures);
    return 1;
}
