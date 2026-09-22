#include "SmoPll.h"

#include <math.h>
#include <string.h>

#define SMO_TWO_PI_F       (6.2831853071795864769f)
#define SMO_EPSILON_F      (1.0e-8f)

static float SmoPll_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float SmoPll_WrapTurns(float turns)
{
    turns = turns - floorf(turns);
    if (turns < 0.0f) {
        turns += 1.0f;
    }
    return turns;
}

static float SmoPll_Saturation(float error, float boundary)
{
    if (boundary <= SMO_EPSILON_F) {
        return (error >= 0.0f) ? 1.0f : -1.0f;
    }
    return SmoPll_Clamp(error / boundary, -1.0f, 1.0f);
}

static float SmoPll_AngleErrorTurns(float angle, float reference)
{
    float error = SmoPll_WrapTurns(angle) - SmoPll_WrapTurns(reference);

    if (error >= 0.5f) {
        error -= 1.0f;
    } else if (error < -0.5f) {
        error += 1.0f;
    }
    return error;
}

uint16_t SmoPll_ValidateParams(const SMO_PLL_PARAMS *params)
{
    float damping;
    if ((params == 0)
        || (!isfinite(params->sample_time_s))
        || (params->sample_time_s < 0.000001f)
        || (params->sample_time_s > 0.001f)
        || (!isfinite(params->stator_resistance_pu))
        || (params->stator_resistance_pu < 0.0f)
        || (params->stator_resistance_pu > 2.0f)
        || (!isfinite(params->current_model_gain))
        || (params->current_model_gain <= 0.0f)
        || (params->current_model_gain > 1.0f)
        || (!isfinite(params->switching_gain))
        || (params->switching_gain <= 0.0f)
        || (params->switching_gain > 2.0f)
        || (!isfinite(params->boundary_pu))
        || (params->boundary_pu <= 0.0f)
        || (params->boundary_pu > 1.0f)
        || (!isfinite(params->emf_lpf_alpha))
        || (params->emf_lpf_alpha <= 0.0f)
        || (params->emf_lpf_alpha > 1.0f)
        || (!isfinite(params->emf_delay_comp_gain))
        || (params->emf_delay_comp_gain < 0.0f)
        || (params->emf_delay_comp_gain > 2.0f)
        || (!isfinite(params->pll_kp)) || (params->pll_kp < 0.0f)
        || (params->pll_kp > 5000.0f)
        || (!isfinite(params->pll_ki)) || (params->pll_ki < 0.0f)
        || (params->pll_ki > 1000000.0f)
        || (!isfinite(params->pll_speed_lpf_alpha))
        || (params->pll_speed_lpf_alpha <= 0.0f)
        || (params->pll_speed_lpf_alpha > 1.0f)
        || (!isfinite(params->weak_emf_speed_decay))
        || (params->weak_emf_speed_decay < 0.0f)
        || (params->weak_emf_speed_decay > 0.10f)
        || (!isfinite(params->min_emf_pu))
        || (params->min_emf_pu <= 0.0f)
        || (params->min_emf_pu > 1.0f)
        || (!isfinite(params->unlock_emf_ratio))
        || (params->unlock_emf_ratio <= 0.0f)
        || (params->unlock_emf_ratio > 1.0f)
        || (!isfinite(params->lock_phase_error))
        || (params->lock_phase_error <= 0.0f)
        || (params->lock_phase_error > 1.0f)
        || (!isfinite(params->max_electrical_rad_s))
        || (params->max_electrical_rad_s <= 0.0f)
        || (!isfinite(params->max_accel_e_rad_s2))
        || (params->max_accel_e_rad_s2 <= 0.0f)
        || (params->max_accel_e_rad_s2 > 10000000.0f)
        || (!isfinite(params->current_hat_limit_pu))
        || (params->current_hat_limit_pu <= 0.0f)
        || (params->current_hat_limit_pu > 10.0f)
        || (params->lock_cycles == 0U)
        || (params->unlock_cycles == 0U)) {
        return 0U;
    }
    /* Independent bounds alone admit a two-cycle switching limit cycle. */
    damping = params->current_model_gain * (params->stator_resistance_pu
        + params->switching_gain / params->boundary_pu);
    if (!isfinite(damping) || damping <= 0.0f
        || damping >= SMO_PLL_STABILITY_LIMIT) {
        return 0U;
    }
    return 1U;
}

void SmoPll_Reset(SMO_PLL_STATE *state, float initial_theta_turns)
{
    if (state == 0) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->theta_pll_turns = SmoPll_WrapTurns(initial_theta_turns);
    state->theta_raw_turns = state->theta_pll_turns;
}

void SmoPll_Init(SMO_PLL_STATE *state, const SMO_PLL_PARAMS *params, float initial_theta_turns)
{
    SmoPll_Reset(state, initial_theta_turns);
    if (SmoPll_ValidateParams(params) == 0U) {
        return;
    }
}

void SmoPll_SeedTracking(SMO_PLL_STATE *state,
                         float theta_turns,
                         float omega_e_rad_s)
{
    if ((state == 0) || (!isfinite(theta_turns))
        || (!isfinite(omega_e_rad_s))) {
        return;
    }
    state->theta_raw_turns = SmoPll_WrapTurns(theta_turns);
    state->theta_pll_turns = state->theta_raw_turns;
    state->omega_integrator_e_rad_s = omega_e_rad_s;
    state->omega_tracking_e_rad_s = omega_e_rad_s;
    state->omega_e_rad_s = omega_e_rad_s;
    state->phase_error = 0.0f;
}

void SmoPll_Step(SMO_PLL_STATE *state, const SMO_PLL_PARAMS *params,
                 const SMO_PLL_INPUT *input)
{
    float current_error_alpha;
    float current_error_beta;
    float direction;
    float emf_alpha_directed;
    float emf_beta_directed;
    float raw_theta_turns;
    float filter_time_constant_s;
    float phase_error_turns;
    float phase_for_lock;
    float phase_abs;
    float emf_score;
    float phase_score;
    float emf_unlock_threshold;
    float speed_increment;
    float maximum_speed_increment;
    float seed_theta;
    uint16_t lock_candidate;

    if (state == 0) return;
    if (input == 0 || SmoPll_ValidateParams(params) == 0U) {
        uint32_t numeric_fault_count = state->numeric_fault_count + 1U;
        SmoPll_Reset(state, 0.0f);
        state->numeric_fault_count = numeric_fault_count;
        state->lock_reason = 4U;
        return;
    }
    if ((!isfinite(input->voltage_alpha_pu))
        || (!isfinite(input->voltage_beta_pu))
        || (!isfinite(input->current_alpha_pu))
        || (!isfinite(input->current_beta_pu))) {
        uint32_t numeric_fault_count = state->numeric_fault_count + 1U;
        seed_theta = isfinite(state->theta_pll_turns)
            ? state->theta_pll_turns : 0.0f;
        SmoPll_Reset(state, seed_theta);
        state->numeric_fault_count = numeric_fault_count;
        state->lock_reason = 4U;
        return;
    }

    current_error_alpha =
        state->current_hat_alpha_pu - input->current_alpha_pu;
    current_error_beta =
        state->current_hat_beta_pu - input->current_beta_pu;
    state->current_error_magnitude_pu = sqrtf(
        current_error_alpha * current_error_alpha
        + current_error_beta * current_error_beta);

    state->switching_alpha_pu = params->switching_gain
        * SmoPll_Saturation(current_error_alpha, params->boundary_pu);
    state->switching_beta_pu = params->switching_gain
        * SmoPll_Saturation(current_error_beta, params->boundary_pu);

    state->current_hat_alpha_pu += params->current_model_gain
        * (input->voltage_alpha_pu
           - params->stator_resistance_pu * state->current_hat_alpha_pu
           - state->switching_alpha_pu);
    state->current_hat_beta_pu += params->current_model_gain
        * (input->voltage_beta_pu
           - params->stator_resistance_pu * state->current_hat_beta_pu
           - state->switching_beta_pu);
    state->current_hat_alpha_pu = SmoPll_Clamp(
        state->current_hat_alpha_pu, -params->current_hat_limit_pu,
        params->current_hat_limit_pu);
    state->current_hat_beta_pu = SmoPll_Clamp(
        state->current_hat_beta_pu, -params->current_hat_limit_pu,
        params->current_hat_limit_pu);

    state->emf_alpha_pu += params->emf_lpf_alpha
        * (state->switching_alpha_pu - state->emf_alpha_pu);
    state->emf_beta_pu += params->emf_lpf_alpha
        * (state->switching_beta_pu - state->emf_beta_pu);
    state->emf_magnitude_pu = sqrtf(
        state->emf_alpha_pu * state->emf_alpha_pu
        + state->emf_beta_pu * state->emf_beta_pu);

    emf_unlock_threshold =
        params->unlock_emf_ratio * params->min_emf_pu;
    state->emf_confidence = SmoPll_Clamp(
        (state->emf_magnitude_pu - emf_unlock_threshold)
        / ((2.0f * params->min_emf_pu - emf_unlock_threshold)
            + SMO_EPSILON_F), 0.0f, 1.0f);

    direction = (input->expected_direction < 0) ? -1.0f : 1.0f;
    emf_alpha_directed = direction * state->emf_alpha_pu;
    emf_beta_directed = direction * state->emf_beta_pu;
    raw_theta_turns = atan2f(-emf_alpha_directed, emf_beta_directed)
        / SMO_TWO_PI_F;
    filter_time_constant_s = params->sample_time_s
        * (1.0f - params->emf_lpf_alpha) / params->emf_lpf_alpha;
    state->phase_advance_turns = params->emf_delay_comp_gain
        * atanf(state->omega_tracking_e_rad_s
                * filter_time_constant_s) / SMO_TWO_PI_F;
    state->theta_raw_turns = SmoPll_WrapTurns(
        raw_theta_turns + state->phase_advance_turns);

    phase_error_turns = SmoPll_AngleErrorTurns(
        state->theta_raw_turns, state->theta_pll_turns);
    phase_for_lock = sinf(phase_error_turns * SMO_TWO_PI_F);
    state->phase_error = SmoPll_Clamp(
        phase_for_lock * state->emf_confidence, -1.0f, 1.0f);

    maximum_speed_increment =
        params->max_accel_e_rad_s2 * params->sample_time_s;
    speed_increment = params->pll_ki * state->phase_error
        * params->sample_time_s;
    speed_increment = SmoPll_Clamp(speed_increment,
        -maximum_speed_increment, maximum_speed_increment);
    if (state->emf_magnitude_pu >= emf_unlock_threshold) {
        state->omega_integrator_e_rad_s += speed_increment;
        state->weak_emf_counter = 0U;
    } else {
        state->omega_integrator_e_rad_s *=
            (1.0f - params->weak_emf_speed_decay);
        if (state->weak_emf_counter < 0xffffffffUL) {
            state->weak_emf_counter++;
        }
    }
    state->omega_integrator_e_rad_s = SmoPll_Clamp(
        state->omega_integrator_e_rad_s,
        -params->max_electrical_rad_s,
        params->max_electrical_rad_s);
    state->omega_tracking_e_rad_s = SmoPll_Clamp(
        state->omega_integrator_e_rad_s
            + params->pll_kp * state->phase_error,
        -params->max_electrical_rad_s,
        params->max_electrical_rad_s);
    state->omega_e_rad_s += params->pll_speed_lpf_alpha
        * (state->omega_tracking_e_rad_s - state->omega_e_rad_s);
    state->theta_pll_turns = SmoPll_WrapTurns(
        state->theta_pll_turns + state->omega_tracking_e_rad_s
            * params->sample_time_s / SMO_TWO_PI_F);

    state->phase_error_rad = phase_error_turns * SMO_TWO_PI_F;
    phase_abs = fabsf(state->phase_error_rad);
    emf_score = SmoPll_Clamp(
        (state->emf_magnitude_pu - params->min_emf_pu)
        / ((2.0f * params->min_emf_pu) + SMO_EPSILON_F),
        0.0f, 1.0f);
    phase_score = SmoPll_Clamp(1.0f - phase_abs
        / (params->lock_phase_error + SMO_EPSILON_F), 0.0f, 1.0f);
    state->quality = (uint16_t)(1000.0f * emf_score * phase_score);

    lock_candidate = (uint16_t)
        (((state->emf_magnitude_pu >= ((state->locked != 0U)
            ? emf_unlock_threshold : params->min_emf_pu))
        && (phase_abs <= params->lock_phase_error)) ? 1U : 0U);
    state->lock_reason = (state->emf_magnitude_pu < ((state->locked != 0U)
        ? emf_unlock_threshold : params->min_emf_pu)) ? 2U
        : ((phase_abs > params->lock_phase_error) ? 3U : 0U);
    if (lock_candidate != 0U) {
        state->unlock_counter = 0U;
        if (state->lock_counter < params->lock_cycles) {
            state->lock_counter++;
        }
        if (state->lock_counter >= params->lock_cycles) {
            state->locked = 1U;
            state->lock_reason = 1U;
        }
    } else {
        state->lock_counter = 0U;
        if (state->unlock_counter < params->unlock_cycles) {
            state->unlock_counter++;
        }
        if (state->unlock_counter >= params->unlock_cycles) {
            state->locked = 0U;
        }
    }

    if ((!isfinite(state->current_hat_alpha_pu))
        || (!isfinite(state->current_hat_beta_pu))
        || (!isfinite(state->current_error_magnitude_pu))
        || (!isfinite(state->emf_alpha_pu))
        || (!isfinite(state->emf_beta_pu))
        || (!isfinite(state->emf_magnitude_pu))
        || (!isfinite(state->theta_raw_turns))
        || (!isfinite(state->theta_pll_turns))
        || (!isfinite(state->omega_integrator_e_rad_s))
        || (!isfinite(state->omega_tracking_e_rad_s))
        || (!isfinite(state->omega_e_rad_s))) {
        uint32_t numeric_fault_count = state->numeric_fault_count + 1U;
        seed_theta = isfinite(state->theta_pll_turns)
            ? state->theta_pll_turns : 0.0f;
        SmoPll_Reset(state, seed_theta);
        state->numeric_fault_count = numeric_fault_count;
        state->lock_reason = 4U;
        return;
    }

    state->update_counter++;
}
