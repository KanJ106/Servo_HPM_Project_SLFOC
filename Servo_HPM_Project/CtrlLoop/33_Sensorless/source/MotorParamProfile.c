#include "MotorParamProfile.h"

#include <math.h>
#include <string.h>

#include "SensorlessConfig.h"

#define MOTOR_PARAM_TWO_PI_F          (6.2831853071795864769f)
#define MOTOR_PARAM_HFI_MIN_SALIENCY  (0.05f)
/* Board bases can put the 0.46 mH joint above 2 pu (about 2.105 pu).
 * This remains a bounded input range, not a physical inductance limit. */
#define MOTOR_PARAM_INDUCTANCE_MAX_PU (4.0f)

static float MotorParam_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static uint16_t MotorParam_InRange(float value, float minimum, float maximum)
{
    return (uint16_t)((isfinite(value) && (value >= minimum) && (value <= maximum))
        ? 1U : 0U);
}

void MotorParamProfile_ClearOverride(SENSORLESS_MOTOR_OVERRIDE *override_values)
{
    if (override_values != 0) {
        memset(override_values, 0, sizeof(*override_values));
    }
}

static void MotorParam_ApplyOverride(SENSORLESS_MOTOR_RAW *effective,
                                     const SENSORLESS_MOTOR_OVERRIDE *override_values)
{
    if (override_values == 0) {
        return;
    }

    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_RS) != 0U) {
        effective->stator_resistance_pu = override_values->stator_resistance_pu;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_LD) != 0U) {
        effective->ld_pu = override_values->ld_pu;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_LQ) != 0U) {
        effective->lq_pu = override_values->lq_pu;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_MAX_SPEED) != 0U) {
        effective->max_speed_rpm = override_values->max_speed_rpm;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_POLE_PAIRS) != 0U) {
        effective->pole_pairs = override_values->pole_pairs;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_RATED_CURRENT) != 0U) {
        effective->rated_current_pu = override_values->rated_current_pu;
    }
    if ((override_values->mask & MOTOR_PARAM_OVERRIDE_CURRENT_LIMIT) != 0U) {
        effective->current_limit_pu = override_values->current_limit_pu;
    }
}

void MotorParamProfile_Build(const SENSORLESS_MOTOR_RAW *raw,
                             const SENSORLESS_MOTOR_OVERRIDE *override_values,
                             SENSORLESS_MOTOR_PROFILE *profile)
{
    float sample_time_s;
    float unclamped_gain;
    uint32_t critical_flags;

    if (profile == 0) {
        return;
    }

    memset(profile, 0, sizeof(*profile));
    if (raw != 0) {
        profile->effective = *raw;
    }
    if (override_values != 0) {
        profile->override_mask = override_values->mask;
        MotorParam_ApplyOverride(&profile->effective, override_values);
    }

    if (MotorParam_InRange(profile->effective.sample_hz, 4000.0f, 100000.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_SAMPLE_RATE;
        profile->effective.sample_hz = SENSORLESS_DEFAULT_SAMPLE_HZ;
    }
    if (MotorParam_InRange(profile->effective.stator_resistance_pu, 0.000001f, 2.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_RS;
        profile->effective.stator_resistance_pu = SENSORLESS_DEFAULT_RS_PU;
    }
    if (MotorParam_InRange(profile->effective.ld_pu, 0.000001f, MOTOR_PARAM_INDUCTANCE_MAX_PU) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_LD;
        profile->effective.ld_pu = 0.0f;
    }
    if (MotorParam_InRange(profile->effective.lq_pu, 0.000001f, MOTOR_PARAM_INDUCTANCE_MAX_PU) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_LQ;
        profile->effective.lq_pu = 0.0f;
    }
    if (MotorParam_InRange(profile->effective.speed_base_rpm, 100.0f, 100000.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_SPEED_BASE;
        profile->effective.speed_base_rpm = 3000.0f;
    }
    if (MotorParam_InRange(profile->effective.max_speed_rpm, 10.0f, 100000.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_MAX_SPEED;
        profile->effective.max_speed_rpm = profile->effective.speed_base_rpm;
    }
    if (MotorParam_InRange(profile->effective.pole_pairs, 1.0f, 64.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_POLE_PAIRS;
        profile->effective.pole_pairs = 1.0f;
    }
    if (MotorParam_InRange(profile->effective.rated_current_pu, 0.001f, 4.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_RATED_CURRENT;
        profile->effective.rated_current_pu = 0.10f;
    }
    if (MotorParam_InRange(profile->effective.current_limit_pu, 0.001f, 4.0f) == 0U) {
        profile->validation_flags |= MOTOR_PARAM_FLAG_CURRENT_LIMIT;
        profile->effective.current_limit_pu = profile->effective.rated_current_pu;
    }

    profile->average_inductance_pu = 0.5f
        * (profile->effective.ld_pu + profile->effective.lq_pu);
    profile->saliency_polarity = (profile->effective.lq_pu
        >= profile->effective.ld_pu) ? 1 : -1;
    if (profile->average_inductance_pu > 0.0f) {
        profile->saliency_ratio = fabsf(profile->effective.lq_pu
            - profile->effective.ld_pu) / profile->average_inductance_pu;
    }
    if (profile->saliency_ratio >= MOTOR_PARAM_HFI_MIN_SALIENCY) {
        profile->hfi_capable = 1U;
    } else {
        profile->validation_flags |= MOTOR_PARAM_FLAG_HFI_SALIENCY;
    }
    profile->base_electrical_rad_s = profile->effective.speed_base_rpm
        * profile->effective.pole_pairs * MOTOR_PARAM_TWO_PI_F / 60.0f;
    profile->max_electrical_rad_s = profile->effective.max_speed_rpm
        * profile->effective.pole_pairs * MOTOR_PARAM_TWO_PI_F / 60.0f;

    sample_time_s = 1.0f / profile->effective.sample_hz;
    if (profile->average_inductance_pu > 0.0f) {
        unclamped_gain = profile->base_electrical_rad_s * sample_time_s
            / profile->average_inductance_pu;
        profile->observer_model_gain = MotorParam_Clamp(unclamped_gain,
            SENSORLESS_MODEL_GAIN_MIN, SENSORLESS_MODEL_GAIN_MAX);
        if (profile->observer_model_gain != unclamped_gain) {
            profile->validation_flags |= MOTOR_PARAM_FLAG_MODEL_GAIN_CLAMPED;
        }
    } else {
        profile->observer_model_gain = SENSORLESS_DEFAULT_MODEL_GAIN;
    }

    critical_flags = MOTOR_PARAM_FLAG_SAMPLE_RATE | MOTOR_PARAM_FLAG_RS
        | MOTOR_PARAM_FLAG_LD | MOTOR_PARAM_FLAG_LQ | MOTOR_PARAM_FLAG_SPEED_BASE
        | MOTOR_PARAM_FLAG_MAX_SPEED | MOTOR_PARAM_FLAG_POLE_PAIRS
        | MOTOR_PARAM_FLAG_RATED_CURRENT | MOTOR_PARAM_FLAG_CURRENT_LIMIT;
    profile->valid_for_shadow = 1U;
    profile->valid_for_control = (uint16_t)
        (((profile->validation_flags & critical_flags) == 0U) ? 1U : 0U);
}

void MotorParamProfile_ApplyToSmo(const SENSORLESS_MOTOR_PROFILE *profile,
                                  SMO_PLL_PARAMS *params)
{
    float sample_hz;
    float emf_lpf_omega;
    float stability_room;
    float required_boundary;

    if ((profile == 0) || (params == 0)) {
        return;
    }

    sample_hz = profile->effective.sample_hz;
    emf_lpf_omega = MOTOR_PARAM_TWO_PI_F * SENSORLESS_DEFAULT_EMF_LPF_HZ;

    params->sample_time_s = 1.0f / sample_hz;
    params->stator_resistance_pu = profile->effective.stator_resistance_pu;
    params->current_model_gain = profile->observer_model_gain;
    params->switching_gain = SENSORLESS_DEFAULT_SWITCH_GAIN;
    params->boundary_pu = SENSORLESS_DEFAULT_BOUNDARY_PU;
    /* Keep physical g and maximum injection K; widen only the linear layer.
     * An impossible combination remains invalid rather than hiding it by
     * clamping g, Rs or the boundary into an unsafe range. */
    stability_room = SMO_PLL_STABILITY_TARGET
        - params->current_model_gain * params->stator_resistance_pu;
    if (isfinite(stability_room) && stability_room > 0.0f) {
        required_boundary = params->current_model_gain * params->switching_gain
            / stability_room;
        if (!isfinite(required_boundary)) {
            params->boundary_pu = 0.0f;
        } else if (required_boundary > params->boundary_pu) {
            params->boundary_pu = required_boundary;
        }
    } else {
        params->boundary_pu = 0.0f;
    }
    params->emf_lpf_alpha = MotorParam_Clamp(
        emf_lpf_omega / (emf_lpf_omega + sample_hz), 0.0f, 1.0f);
    params->emf_delay_comp_gain =
        SENSORLESS_DEFAULT_EMF_DELAY_COMP_GAIN;
    params->pll_kp = SENSORLESS_DEFAULT_PLL_KP;
    params->pll_ki = SENSORLESS_DEFAULT_PLL_KI;
    params->pll_speed_lpf_alpha =
        SENSORLESS_DEFAULT_PLL_SPEED_LPF_ALPHA;
    params->weak_emf_speed_decay =
        SENSORLESS_DEFAULT_WEAK_EMF_DECAY;
    params->min_emf_pu = SENSORLESS_DEFAULT_MIN_EMF_PU;
    params->unlock_emf_ratio =
        SENSORLESS_DEFAULT_UNLOCK_EMF_RATIO;
    params->lock_phase_error = SENSORLESS_DEFAULT_LOCK_PHASE_ERR;
    params->max_electrical_rad_s = (profile->max_electrical_rad_s > 0.0f)
        ? (1.25f * profile->max_electrical_rad_s)
        : SENSORLESS_DEFAULT_MAX_ELEC_RAD_S;
    params->max_accel_e_rad_s2 =
        SENSORLESS_DEFAULT_MAX_ACCEL_E_RAD_S2;
    params->current_hat_limit_pu =
        SENSORLESS_DEFAULT_CURRENT_HAT_LIMIT_PU;
    params->lock_cycles = SENSORLESS_DEFAULT_LOCK_CYCLES;
    params->unlock_cycles = SENSORLESS_DEFAULT_UNLOCK_CYCLES;
}
