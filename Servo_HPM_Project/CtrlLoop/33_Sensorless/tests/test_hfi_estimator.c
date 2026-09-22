#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "HfiEstimator.h"

#define TEST_TWO_PI_F (6.2831853071795864769f)

static int g_failures;

static void Test_Check(int condition, const char *message)
{
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static float Test_AngleError(float angle, float reference)
{
    float error = angle - reference;
    while (error >= 0.5f) {
        error -= 1.0f;
    }
    while (error < -0.5f) {
        error += 1.0f;
    }
    return error;
}

static void Test_RunSyntheticSample(HFI_ESTIMATOR_STATE *state,
                                    const HFI_ESTIMATOR_PARAMS *params,
                                    float true_theta,
                                    float base_alpha,
                                    float base_beta,
                                    float positive_axis_response,
                                    float negative_axis_response)
{
    float true_angle_rad = true_theta * TEST_TWO_PI_F;
    float estimate_error_rad =
        (state->theta_turns - true_theta) * TEST_TWO_PI_F;
    float injection = HfiEstimator_GetInjectionCurrent(state);
    float pole_current = HfiEstimator_GetPoleDetectCurrent(state);
    float physical_pole_current = pole_current * cosf(estimate_error_rad);
    float response = 0.030f;
    float carrier_sign = (injection >= 0.0f) ? 1.0f : -1.0f;
    float current_alpha;
    float current_beta;

    if (physical_pole_current > 0.0001f) {
        response = positive_axis_response;
    } else if (physical_pole_current < -0.0001f) {
        response = negative_axis_response;
    }
    current_alpha = base_alpha
        + carrier_sign * response * cosf(true_angle_rad);
    current_beta = base_beta
        + carrier_sign * response * sinf(true_angle_rad);
    HfiEstimator_Step(state, params, current_alpha, current_beta);
}

static void Test_TracksSyntheticSaliencyAndRejectsBaseCurrent(void)
{
    HFI_ESTIMATOR_PARAMS params;
    HFI_ESTIMATOR_STATE state;
    const float true_theta = 0.08f;
    const float base_alpha = 0.160f;
    const float base_beta = -0.110f;
    uint32_t index;

    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.lock_cycles = 80U;
    params.unlock_cycles = 20U;
    params.pole_settle_cycles = 32U;
    params.pole_measure_cycles = 32U;
    Test_Check(HfiEstimator_ValidateParams(&params) != 0U,
               "default HFI parameters were rejected");
    HfiEstimator_Enable(&state, &params, 0.0f);

    for (index = 0U; index < 32000U; index++) {
        Test_RunSyntheticSample(&state, &params, true_theta,
            base_alpha, base_beta, 0.024f, 0.036f);
    }

    Test_Check(state.locked != 0U,
               "HFI estimator did not lock on synthetic saliency response");
    Test_Check(state.pole_detected != 0U,
               "HFI estimator did not complete magnetic pole detection");
    Test_Check(state.polarity_corrected == 0U,
               "correct magnetic polarity was unnecessarily inverted");
    Test_Check(fabsf(Test_AngleError(state.theta_turns, true_theta)) < 0.03f,
               "HFI angle error exceeds 0.03 electrical turn");
    Test_Check(state.quality > 500U,
               "HFI quality did not reflect the valid response");
    Test_Check(fabsf(state.base_current_alpha_pu - base_alpha) < 0.001f,
               "positive/negative carrier pair did not reject alpha base current");
    Test_Check(fabsf(state.base_current_beta_pu - base_beta) < 0.001f,
               "positive/negative carrier pair did not reject beta base current");
    Test_Check(fabsf(state.hf_current_alpha_pu) < 0.050f,
               "demodulated alpha response retained excessive base current");

    HfiEstimator_Disable(&state);
    Test_Check(HfiEstimator_GetInjectionCurrent(&state) == 0.0f,
               "disabled HFI estimator still requested injection current");
    Test_Check(HfiEstimator_GetPoleDetectCurrent(&state) == 0.0f,
               "disabled HFI estimator still requested pole-detect current");
}

static void Test_CorrectsHalfTurnPolarityAmbiguity(void)
{
    HFI_ESTIMATOR_PARAMS params;
    HFI_ESTIMATOR_STATE state;
    const float true_theta = 0.08f;
    uint32_t index;

    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.lock_cycles = 80U;
    params.unlock_cycles = 20U;
    params.pole_settle_cycles = 32U;
    params.pole_measure_cycles = 32U;
    HfiEstimator_Enable(&state, &params, true_theta + 0.5f);

    for (index = 0U; index < 32000U; index++) {
        Test_RunSyntheticSample(&state, &params, true_theta,
            0.080f, 0.040f, 0.024f, 0.036f);
    }

    Test_Check(state.locked != 0U,
               "HFI estimator did not lock after half-turn correction");
    Test_Check(state.polarity_corrected != 0U,
               "HFI estimator did not report half-turn correction");
    Test_Check(fabsf(Test_AngleError(state.theta_turns, true_theta)) < 0.03f,
               "magnetic pole correction did not resolve the half-turn ambiguity");
    Test_Check(state.pole_contrast > params.pole_min_contrast,
               "valid magnetic pole contrast was not retained");
}

static void Test_RejectsAmbiguousPoleContrast(void)
{
    HFI_ESTIMATOR_PARAMS params;
    HFI_ESTIMATOR_STATE state;
    uint32_t index;

    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.lock_cycles = 80U;
    params.unlock_cycles = 20U;
    params.pole_settle_cycles = 32U;
    params.pole_measure_cycles = 32U;
    HfiEstimator_Enable(&state, &params, 0.0f);

    for (index = 0U; index < 8000U; index++) {
        Test_RunSyntheticSample(&state, &params, 0.0f,
            0.0f, 0.0f, 0.030f, 0.030f);
    }

    Test_Check(state.pole_state == (uint16_t)HFI_POLE_FAILED,
               "ambiguous magnetic pole response did not fail closed");
    Test_Check(state.locked == 0U,
               "ambiguous magnetic pole response was incorrectly accepted");
    Test_Check(state.quality == 0U,
               "failed magnetic pole detection retained nonzero quality");
}

static void Test_RejectsUnsafeParameters(void)
{
    HFI_ESTIMATOR_PARAMS params;

    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.injection_current_pu = 0.20f;
    Test_Check(HfiEstimator_ValidateParams(&params) == 0U,
               "unsafe injection current was accepted");
    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.injection_frequency_hz = 5000.0f;
    Test_Check(HfiEstimator_ValidateParams(&params) == 0U,
               "injection frequency above Fs/4 was accepted");
    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.pole_detect_current_pu = 0.20f;
    Test_Check(HfiEstimator_ValidateParams(&params) == 0U,
               "unsafe pole-detect current was accepted");
    HfiEstimator_DefaultParams(&params, 1.0f / 16000.0f,
                               4000.0f, 0.20f, 1);
    params.pole_response_polarity = 0;
    Test_Check(HfiEstimator_ValidateParams(&params) == 0U,
               "invalid pole-response polarity was accepted");
}

int main(void)
{
    Test_TracksSyntheticSaliencyAndRejectsBaseCurrent();
    Test_CorrectsHalfTurnPolarityAmbiguity();
    Test_RejectsAmbiguousPoleContrast();
    Test_RejectsUnsafeParameters();

    if (g_failures == 0) {
        printf("PASS: HFI estimator unit tests\n");
        return 0;
    }
    printf("FAILURES: %d\n", g_failures);
    return 1;
}
