#include "HfiEstimator.h"

#include <math.h>
#include <string.h>

#define HFI_TWO_PI_F  (6.2831853071795864769f)
#define HFI_EPSILON_F (1.0e-8f)

static float Hfi_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float Hfi_WrapTurns(float turns)
{
    turns -= floorf(turns);
    if (turns < 0.0f) {
        turns += 1.0f;
    }
    return turns;
}

void HfiEstimator_DefaultParams(HFI_ESTIMATOR_PARAMS *params,
                                float sample_time_s,
                                float max_electrical_rad_s,
                                float current_limit_pu,
                                int16_t saliency_polarity)
{
    float sample_hz;
    float injection;
    float pole_current;

    if (params == 0) {
        return;
    }

    memset(params, 0, sizeof(*params));
    sample_hz = (sample_time_s > HFI_EPSILON_F)
        ? (1.0f / sample_time_s) : 16000.0f;
    injection = Hfi_Clamp(0.10f * current_limit_pu, 0.005f, 0.020f);
    pole_current = Hfi_Clamp(0.10f * current_limit_pu, 0.005f, 0.030f);
    if (injection > current_limit_pu) {
        injection = current_limit_pu;
    }
    if (pole_current > current_limit_pu) {
        pole_current = current_limit_pu;
    }

    params->sample_time_s = sample_time_s;
    params->injection_frequency_hz = Hfi_Clamp(sample_hz / 16.0f,
                                                400.0f, 1200.0f);
    params->injection_current_pu = injection;
    params->pole_detect_current_pu = pole_current;
    params->pole_min_contrast = 0.05f;
    params->response_lpf_alpha = 0.05f;
    params->speed_lpf_alpha = 0.05f;
    params->tracking_kp = 120.0f;
    params->tracking_ki = 4000.0f;
    params->min_response_pu = Hfi_Clamp(0.15f * injection,
                                        0.0005f, 0.010f);
    params->lock_error = 0.20f;
    params->max_electrical_rad_s = max_electrical_rad_s;
    params->saliency_polarity = (saliency_polarity < 0) ? -1 : 1;
    params->pole_response_polarity = 1;
    params->lock_cycles = (uint16_t)Hfi_Clamp(0.030f * sample_hz,
                                              10.0f, 60000.0f);
    params->unlock_cycles = (uint16_t)Hfi_Clamp(0.010f * sample_hz,
                                                5.0f, 60000.0f);
    params->pole_settle_cycles = (uint16_t)Hfi_Clamp(0.005f * sample_hz,
                                                     8.0f, 60000.0f);
    params->pole_measure_cycles = (uint16_t)Hfi_Clamp(0.005f * sample_hz,
                                                      8.0f, 60000.0f);
}

uint16_t HfiEstimator_ValidateParams(const HFI_ESTIMATOR_PARAMS *params)
{
    float sample_hz;

    if ((params == 0) || (!isfinite(params->sample_time_s))
        || (params->sample_time_s < 0.000001f)
        || (params->sample_time_s > 0.001f)) {
        return 0U;
    }
    sample_hz = 1.0f / params->sample_time_s;
    if ((!isfinite(params->injection_frequency_hz))
        || (params->injection_frequency_hz < 100.0f)
        || (params->injection_frequency_hz > (0.25f * sample_hz))
        || (!isfinite(params->injection_current_pu))
        || (params->injection_current_pu <= 0.0f)
        || (params->injection_current_pu > 0.10f)
        || (!isfinite(params->pole_detect_current_pu))
        || (params->pole_detect_current_pu <= 0.0f)
        || (params->pole_detect_current_pu > 0.10f)
        || (!isfinite(params->pole_min_contrast))
        || (params->pole_min_contrast <= 0.0f)
        || (params->pole_min_contrast > 1.0f)
        || (!isfinite(params->response_lpf_alpha))
        || (params->response_lpf_alpha <= 0.0f)
        || (params->response_lpf_alpha > 1.0f)
        || (!isfinite(params->speed_lpf_alpha))
        || (params->speed_lpf_alpha <= 0.0f)
        || (params->speed_lpf_alpha > 1.0f)
        || (!isfinite(params->tracking_kp)) || (params->tracking_kp < 0.0f)
        || (params->tracking_kp > 5000.0f)
        || (!isfinite(params->tracking_ki)) || (params->tracking_ki < 0.0f)
        || (params->tracking_ki > 1000000.0f)
        || (!isfinite(params->min_response_pu))
        || (params->min_response_pu <= 0.0f)
        || (params->min_response_pu >= params->injection_current_pu)
        || (!isfinite(params->lock_error)) || (params->lock_error <= 0.0f)
        || (params->lock_error > 1.0f)
        || (!isfinite(params->max_electrical_rad_s))
        || (params->max_electrical_rad_s <= 0.0f)
        || ((params->saliency_polarity != 1)
            && (params->saliency_polarity != -1))
        || ((params->pole_response_polarity != 1)
            && (params->pole_response_polarity != -1))
        || (params->lock_cycles == 0U) || (params->unlock_cycles == 0U)
        || (params->pole_settle_cycles == 0U)
        || (params->pole_measure_cycles == 0U)) {
        return 0U;
    }
    return 1U;
}

static uint32_t Hfi_AddLimited(uint32_t value,
                               uint32_t increment,
                               uint32_t limit)
{
    if ((value >= limit) || (increment >= (limit - value))) {
        return limit;
    }
    return value + increment;
}

static float Hfi_EffectiveAlpha(float sample_alpha, uint32_t elapsed_cycles)
{
    return 1.0f - powf(1.0f - sample_alpha, (float)elapsed_cycles);
}

static void Hfi_ResetPoleDetection(HFI_ESTIMATOR_STATE *state)
{
    state->locked = 0U;
    state->pole_detected = 0U;
    state->polarity_corrected = 0U;
    state->pole_state = (uint16_t)HFI_POLE_WAIT_AXIS;
    state->pole_stage_cycles = 0U;
    state->pole_measure_count = 0U;
    state->pole_measure_sum_pu = 0.0f;
    state->pole_positive_response_pu = 0.0f;
    state->pole_negative_response_pu = 0.0f;
    state->pole_contrast = 0.0f;
    state->pole_detect_current_pu = 0.0f;
}

void HfiEstimator_Init(HFI_ESTIMATOR_STATE *state,
                       const HFI_ESTIMATOR_PARAMS *params,
                       float initial_theta_turns)
{
    float sample_hz;
    float half_period;

    if (state == 0) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->theta_turns = Hfi_WrapTurns(initial_theta_turns);
    state->carrier_sign = 1;
    state->pole_state = (uint16_t)HFI_POLE_WAIT_AXIS;
    if (HfiEstimator_ValidateParams(params) == 0U) {
        return;
    }
    sample_hz = 1.0f / params->sample_time_s;
    half_period = sample_hz / (2.0f * params->injection_frequency_hz);
    state->carrier_half_period_cycles = (uint32_t)Hfi_Clamp(
        half_period + 0.5f, 2.0f, 100000.0f);
}

void HfiEstimator_Enable(HFI_ESTIMATOR_STATE *state,
                         const HFI_ESTIMATOR_PARAMS *params,
                         float seed_theta_turns)
{
    if ((state == 0) || (HfiEstimator_ValidateParams(params) == 0U)) {
        return;
    }
    HfiEstimator_Init(state, params, seed_theta_turns);
    state->active = 1U;
    state->injection_current_pu = params->injection_current_pu;
}

void HfiEstimator_Disable(HFI_ESTIMATOR_STATE *state)
{
    if (state != 0) {
        state->active = 0U;
        state->axis_locked = 0U;
        state->injection_current_pu = 0.0f;
        state->half_sum_alpha_pu = 0.0f;
        state->half_sum_beta_pu = 0.0f;
        state->half_sample_count = 0U;
        state->lock_counter = 0U;
        state->unlock_counter = 0U;
        Hfi_ResetPoleDetection(state);
    }
}

static void Hfi_UpdateLockState(HFI_ESTIMATOR_STATE *state,
                                const HFI_ESTIMATOR_PARAMS *params,
                                float response,
                                uint32_t elapsed_cycles)
{
    float response_score;
    float error_score;
    uint16_t lock_candidate;

    response_score = Hfi_Clamp((response - params->min_response_pu)
        / (2.0f * params->min_response_pu + HFI_EPSILON_F), 0.0f, 1.0f);
    error_score = Hfi_Clamp(1.0f - fabsf(state->phase_error)
        / (params->lock_error + HFI_EPSILON_F), 0.0f, 1.0f);
    state->quality = (uint16_t)(1000.0f * response_score * error_score);
    lock_candidate = (uint16_t)(((response >= params->min_response_pu)
        && (fabsf(state->phase_error) <= params->lock_error)) ? 1U : 0U);

    if (lock_candidate != 0U) {
        state->unlock_counter = 0U;
        state->lock_counter = Hfi_AddLimited(state->lock_counter,
            elapsed_cycles, params->lock_cycles);
        if (state->lock_counter >= params->lock_cycles) {
            state->axis_locked = 1U;
        }
    } else {
        state->lock_counter = 0U;
        state->unlock_counter = Hfi_AddLimited(state->unlock_counter,
            elapsed_cycles, params->unlock_cycles);
        if (state->unlock_counter >= params->unlock_cycles) {
            state->axis_locked = 0U;
            Hfi_ResetPoleDetection(state);
        }
    }
}

static void Hfi_UpdatePoleDetection(HFI_ESTIMATOR_STATE *state,
                                    const HFI_ESTIMATOR_PARAMS *params,
                                    float response,
                                    uint32_t elapsed_cycles)
{
    float maximum_response;
    float signed_difference;

    if (state->axis_locked == 0U) {
        if (state->pole_state != (uint16_t)HFI_POLE_WAIT_AXIS) {
            Hfi_ResetPoleDetection(state);
        }
        return;
    }

    switch ((HFI_POLE_STATE)state->pole_state) {
    case HFI_POLE_WAIT_AXIS:
        state->pole_state = (uint16_t)HFI_POLE_POSITIVE_SETTLE;
        state->pole_stage_cycles = 0U;
        state->pole_detect_current_pu = params->pole_detect_current_pu;
        break;

    case HFI_POLE_POSITIVE_SETTLE:
        state->pole_stage_cycles = Hfi_AddLimited(state->pole_stage_cycles,
            elapsed_cycles, params->pole_settle_cycles);
        if (state->pole_stage_cycles >= params->pole_settle_cycles) {
            state->pole_state = (uint16_t)HFI_POLE_POSITIVE_MEASURE;
            state->pole_stage_cycles = 0U;
            state->pole_measure_count = 0U;
            state->pole_measure_sum_pu = 0.0f;
        }
        break;

    case HFI_POLE_POSITIVE_MEASURE:
        state->pole_measure_sum_pu += response;
        state->pole_measure_count++;
        state->pole_stage_cycles = Hfi_AddLimited(state->pole_stage_cycles,
            elapsed_cycles, params->pole_measure_cycles);
        if (state->pole_stage_cycles >= params->pole_measure_cycles) {
            state->pole_positive_response_pu = state->pole_measure_sum_pu
                / (float)state->pole_measure_count;
            state->pole_state = (uint16_t)HFI_POLE_ZERO_SETTLE;
            state->pole_stage_cycles = 0U;
            state->pole_detect_current_pu = 0.0f;
        }
        break;

    case HFI_POLE_ZERO_SETTLE:
        state->pole_stage_cycles = Hfi_AddLimited(state->pole_stage_cycles,
            elapsed_cycles, params->pole_settle_cycles);
        if (state->pole_stage_cycles >= params->pole_settle_cycles) {
            state->pole_state = (uint16_t)HFI_POLE_NEGATIVE_SETTLE;
            state->pole_stage_cycles = 0U;
            state->pole_detect_current_pu = -params->pole_detect_current_pu;
        }
        break;

    case HFI_POLE_NEGATIVE_SETTLE:
        state->pole_stage_cycles = Hfi_AddLimited(state->pole_stage_cycles,
            elapsed_cycles, params->pole_settle_cycles);
        if (state->pole_stage_cycles >= params->pole_settle_cycles) {
            state->pole_state = (uint16_t)HFI_POLE_NEGATIVE_MEASURE;
            state->pole_stage_cycles = 0U;
            state->pole_measure_count = 0U;
            state->pole_measure_sum_pu = 0.0f;
        }
        break;

    case HFI_POLE_NEGATIVE_MEASURE:
        state->pole_measure_sum_pu += response;
        state->pole_measure_count++;
        state->pole_stage_cycles = Hfi_AddLimited(state->pole_stage_cycles,
            elapsed_cycles, params->pole_measure_cycles);
        if (state->pole_stage_cycles >= params->pole_measure_cycles) {
            state->pole_negative_response_pu = state->pole_measure_sum_pu
                / (float)state->pole_measure_count;
            maximum_response = fmaxf(state->pole_positive_response_pu,
                                     state->pole_negative_response_pu);
            state->pole_contrast = fabsf(state->pole_positive_response_pu
                - state->pole_negative_response_pu)
                / (maximum_response + HFI_EPSILON_F);
            signed_difference = (float)params->pole_response_polarity
                * (state->pole_positive_response_pu
                    - state->pole_negative_response_pu);
            state->pole_detect_current_pu = 0.0f;
            if (state->pole_contrast >= params->pole_min_contrast) {
                if (signed_difference > 0.0f) {
                    state->theta_turns =
                        Hfi_WrapTurns(state->theta_turns + 0.5f);
                    state->polarity_corrected = 1U;
                }
                state->pole_detected = 1U;
                state->locked = 1U;
                state->pole_state = (uint16_t)HFI_POLE_COMPLETE;
            } else {
                state->quality = 0U;
                state->locked = 0U;
                state->pole_state = (uint16_t)HFI_POLE_FAILED;
            }
        }
        break;

    case HFI_POLE_COMPLETE:
        state->locked = state->axis_locked;
        break;

    case HFI_POLE_FAILED:
    default:
        state->quality = 0U;
        state->locked = 0U;
        state->pole_detect_current_pu = 0.0f;
        break;
    }
}

static void Hfi_UpdateAxisEstimator(HFI_ESTIMATOR_STATE *state,
                                    const HFI_ESTIMATOR_PARAMS *params,
                                    uint32_t elapsed_cycles)
{
    float theta_rad = state->theta_turns * HFI_TWO_PI_F;
    float cosine = cosf(theta_rad);
    float sine = sinf(theta_rad);
    float raw_d = state->hf_current_alpha_pu * cosine
        + state->hf_current_beta_pu * sine;
    float raw_q = -state->hf_current_alpha_pu * sine
        + state->hf_current_beta_pu * cosine;
    float response_alpha = Hfi_EffectiveAlpha(params->response_lpf_alpha,
                                               elapsed_cycles);
    float speed_alpha = Hfi_EffectiveAlpha(params->speed_lpf_alpha,
                                            elapsed_cycles);
    float elapsed_time_s = params->sample_time_s * (float)elapsed_cycles;
    float response;
    float raw_error;
    float omega_target;

    state->d_response_pu += response_alpha
        * (raw_d - state->d_response_pu);
    state->q_response_pu += response_alpha
        * (raw_q - state->q_response_pu);
    response = fabsf(state->d_response_pu);
    raw_error = ((float)params->saliency_polarity * state->q_response_pu)
        / ((response > params->min_response_pu)
            ? response : params->min_response_pu);
    state->phase_error = Hfi_Clamp(raw_error, -1.0f, 1.0f);
    state->omega_integrator_e_rad_s += params->tracking_ki
        * state->phase_error * elapsed_time_s;
    state->omega_integrator_e_rad_s = Hfi_Clamp(
        state->omega_integrator_e_rad_s, -params->max_electrical_rad_s,
        params->max_electrical_rad_s);
    omega_target = Hfi_Clamp(state->omega_integrator_e_rad_s
        + params->tracking_kp * state->phase_error,
        -params->max_electrical_rad_s, params->max_electrical_rad_s);
    state->omega_e_rad_s += speed_alpha
        * (omega_target - state->omega_e_rad_s);

    Hfi_UpdateLockState(state, params, response, elapsed_cycles);
    Hfi_UpdatePoleDetection(state, params, fabsf(raw_d), elapsed_cycles);
    state->demod_update_counter++;
}

void HfiEstimator_Step(HFI_ESTIMATOR_STATE *state,
                       const HFI_ESTIMATOR_PARAMS *params,
                       float current_alpha_pu,
                       float current_beta_pu)
{
    float half_alpha;
    float half_beta;
    float negative_half_alpha;
    float negative_half_beta;
    float seed_theta;
    uint32_t elapsed_cycles;

    if ((state == 0) || (state->active == 0U)
        || (HfiEstimator_ValidateParams(params) == 0U)) {
        return;
    }
    if ((!isfinite(current_alpha_pu)) || (!isfinite(current_beta_pu))) {
        uint32_t fault_count = state->numeric_fault_count + 1U;
        seed_theta = isfinite(state->theta_turns) ? state->theta_turns : 0.0f;
        HfiEstimator_Enable(state, params, seed_theta);
        state->numeric_fault_count = fault_count;
        return;
    }

    state->theta_turns = Hfi_WrapTurns(state->theta_turns
        + state->omega_e_rad_s * params->sample_time_s / HFI_TWO_PI_F);
    state->half_sum_alpha_pu += current_alpha_pu;
    state->half_sum_beta_pu += current_beta_pu;
    state->half_sample_count++;
    state->carrier_cycles++;

    if (state->carrier_cycles >= state->carrier_half_period_cycles) {
        half_alpha = state->half_sum_alpha_pu
            / (float)state->half_sample_count;
        half_beta = state->half_sum_beta_pu
            / (float)state->half_sample_count;
        state->carrier_cycles = 0U;
        state->half_sum_alpha_pu = 0.0f;
        state->half_sum_beta_pu = 0.0f;
        state->half_sample_count = 0U;

        if (state->carrier_sign > 0) {
            state->positive_half_alpha_pu = half_alpha;
            state->positive_half_beta_pu = half_beta;
        } else {
            negative_half_alpha = half_alpha;
            negative_half_beta = half_beta;
            state->base_current_alpha_pu = 0.5f
                * (state->positive_half_alpha_pu + negative_half_alpha);
            state->base_current_beta_pu = 0.5f
                * (state->positive_half_beta_pu + negative_half_beta);
            state->hf_current_alpha_pu = 0.5f
                * (state->positive_half_alpha_pu - negative_half_alpha);
            state->hf_current_beta_pu = 0.5f
                * (state->positive_half_beta_pu - negative_half_beta);
            elapsed_cycles = 2U * state->carrier_half_period_cycles;
            Hfi_UpdateAxisEstimator(state, params, elapsed_cycles);
        }

        state->carrier_sign = (int16_t)-state->carrier_sign;
        state->injection_current_pu = (float)state->carrier_sign
            * params->injection_current_pu;
    }

    if ((!isfinite(state->theta_turns))
        || (!isfinite(state->omega_integrator_e_rad_s))
        || (!isfinite(state->omega_e_rad_s))
        || (!isfinite(state->d_response_pu))
        || (!isfinite(state->q_response_pu))
        || (!isfinite(state->base_current_alpha_pu))
        || (!isfinite(state->base_current_beta_pu))
        || (!isfinite(state->hf_current_alpha_pu))
        || (!isfinite(state->hf_current_beta_pu))
        || (!isfinite(state->pole_contrast))) {
        uint32_t fault_count = state->numeric_fault_count + 1U;
        seed_theta = isfinite(state->theta_turns) ? state->theta_turns : 0.0f;
        HfiEstimator_Enable(state, params, seed_theta);
        state->numeric_fault_count = fault_count;
        return;
    }
    state->update_counter++;
}

float HfiEstimator_GetInjectionCurrent(const HFI_ESTIMATOR_STATE *state)
{
    return ((state != 0) && (state->active != 0U))
        ? state->injection_current_pu : 0.0f;
}

float HfiEstimator_GetPoleDetectCurrent(const HFI_ESTIMATOR_STATE *state)
{
    return ((state != 0) && (state->active != 0U))
        ? state->pole_detect_current_pu : 0.0f;
}
