#include "SourceFoc.h"
#include <math.h>
#include <string.h>

static uint16_t positive(float x) { return (uint16_t)(isfinite(x) && x > 0.0f); }

uint16_t SourceFoc_BuildProfile(const SOURCE_FOC_RECIPE *r,
    float sample_hz, float speed_base_rpm, SENSORLESS_MOTOR_PROFILE *p)
{
    SENSORLESS_MOTOR_RAW raw;
    float impedance_base, omega_base;
    if (!r || !p || !positive(sample_hz) || !positive(speed_base_rpm)
        || !positive(r->phase_rs_ohm) || !positive(r->phase_ld_h)
        || !positive(r->phase_lq_h) || !positive(r->current_base_a)
        || !positive(r->voltage_base_v) || !positive(r->rated_current_a)
        || !positive(r->current_limit_a) || !positive(r->max_speed_rpm)
        || r->current_limit_a > r->rated_current_a
        || r->pole_pairs < 1U || r->pole_pairs > 64U
        || r->allow_continuous_rotation > 1U
        || r->recipe_id == 0U || r->recipe_revision == 0U) return 0U;
    impedance_base = r->voltage_base_v / r->current_base_a;
    omega_base = speed_base_rpm * (float)r->pole_pairs * 6.28318530718f / 60.0f;
    memset(&raw, 0, sizeof(raw));
    raw.sample_hz = sample_hz;
    raw.speed_base_rpm = speed_base_rpm;
    raw.pole_pairs = (float)r->pole_pairs;
    raw.stator_resistance_pu = r->phase_rs_ohm / impedance_base;
    raw.ld_pu = r->phase_ld_h * omega_base / impedance_base;
    raw.lq_pu = r->phase_lq_h * omega_base / impedance_base;
    raw.rated_current_pu = r->rated_current_a / r->current_base_a;
    raw.current_limit_pu = r->current_limit_a / r->current_base_a;
    raw.max_speed_rpm = r->max_speed_rpm;
    MotorParamProfile_Build(&raw, 0, p);
    /* A model coefficient clipped for stability is not a calibrated model. */
    return (uint16_t)(p->valid_for_control &&
        !(p->validation_flags & MOTOR_PARAM_FLAG_MODEL_GAIN_CLAMPED));
}

uint16_t SourceFoc_Configure(SOURCE_FOC_CURRENT *s,
    const SENSORLESS_MOTOR_PROFILE *p, float bandwidth_hz)
{
    SOURCE_FOC_CURRENT candidate;
    float omega;
    if (!s || !p || !p->valid_for_control || !positive(bandwidth_hz)
        || bandwidth_hz > p->effective.sample_hz * 0.05f
        || !positive(p->base_electrical_rad_s)) return 0U;
    memset(&candidate, 0, sizeof(candidate));
    omega = 6.28318530718f * bandwidth_hz;
    candidate.kp_d = p->effective.ld_pu * omega / p->base_electrical_rad_s;
    candidate.kp_q = p->effective.lq_pu * omega / p->base_electrical_rad_s;
    candidate.ki_step = p->effective.stator_resistance_pu * omega / p->effective.sample_hz;
    candidate.current_limit_pu = fminf(p->effective.current_limit_pu,
                                      p->effective.rated_current_pu);
    if (!positive(candidate.kp_d) || !positive(candidate.kp_q)
        || !positive(candidate.ki_step) || !positive(candidate.current_limit_pu)
        || candidate.current_limit_pu > 0.5f) return 0U;
    candidate.measured_trip_pu = fminf(p->effective.rated_current_pu,
        2.0f * candidate.current_limit_pu);
    candidate.configured = 1U;
    *s = candidate;
    return 1U;
}

void SourceFoc_Reset(SOURCE_FOC_CURRENT *s)
{
    if (!s) return;
    s->integral_d = s->integral_q = 0.0f;
    s->voltage_d = s->voltage_q = 0.0f;
    s->id_reference_pu = s->iq_reference_pu = 0.0f;
    s->saturated = 0U;
}

#ifdef __riscv
__attribute__((section(".ramfunctiontext")))
#endif
uint16_t SourceFoc_Step(SOURCE_FOC_CURRENT *s, float id_ref, float iq_ref,
    float id_fb, float iq_fb, float limit)
{
    float magnitude, scale, ed, eq, vd, vq;
    if (!s) return 0U;
    s->failure_reason = SOURCE_FOC_OK;
    if (!s->configured || !positive(s->kp_d) || !positive(s->kp_q)
        || !positive(s->ki_step) || !positive(s->current_limit_pu)
        || s->current_limit_pu > 0.5f || !positive(s->measured_trip_pu)) {
        s->failure_reason = SOURCE_FOC_CONFIG_INVALID;
    } else if (!positive(limit) || limit > 1.0f) {
        s->failure_reason = SOURCE_FOC_LIMIT_INVALID;
    } else if (!isfinite(id_ref) || !isfinite(iq_ref)
        || !isfinite(id_fb) || !isfinite(iq_fb)) {
        s->failure_reason = SOURCE_FOC_INPUT_INVALID;
    }
    if (s->failure_reason != SOURCE_FOC_OK) {
        SourceFoc_Reset(s); s->numeric_faults++; return 0U;
    }
    if (hypotf(id_fb, iq_fb) > s->measured_trip_pu) {
        s->failure_reason = SOURCE_FOC_OVER_CURRENT;
        SourceFoc_Reset(s); s->overcurrent_faults++; return 0U;
    }
    magnitude = hypotf(id_ref, iq_ref);
    if (magnitude > s->current_limit_pu) {
        scale = s->current_limit_pu / magnitude;
        id_ref *= scale; iq_ref *= scale;
    }
    s->id_reference_pu = id_ref; s->iq_reference_pu = iq_ref;
    ed = id_ref - id_fb; eq = iq_ref - iq_fb;
    vd = s->kp_d * ed + s->integral_d + s->ki_step * ed;
    vq = s->kp_q * eq + s->integral_q + s->ki_step * eq;
    magnitude = hypotf(vd, vq);
    if (!isfinite(magnitude)) {
        s->failure_reason = SOURCE_FOC_CALC_INVALID;
        SourceFoc_Reset(s); s->numeric_faults++; return 0U;
    }
    s->saturated = (uint16_t)(magnitude > limit);
    scale = s->saturated ? limit / magnitude : 1.0f;
    s->voltage_d = vd * scale; s->voltage_q = vq * scale;
    /* Tracking anti-windup acts on the final vector, including both axes. */
    s->integral_d = s->voltage_d - s->kp_d * ed;
    s->integral_q = s->voltage_q - s->kp_q * eq;
    return 1U;
}

#ifdef __riscv
__attribute__((section(".ramfunctiontext")))
#endif
uint16_t SourceFoc_PrepareFrame(const SOURCE_FOC_CURRENT *source,
    SOURCE_FOC_CURRENT *destination, float id_ref, float iq_ref,
    float old_id, float old_iq, float new_id, float new_iq,
    float c, float s, float limit)
{
    SOURCE_FOC_CURRENT next=*source;
    float d,q,norm,scale;
    if(!isfinite(c)||!isfinite(s)||fabsf(c*c+s*s-1.0f)>0.000002f
        ||!isfinite(new_id)||!isfinite(new_iq))
        return SOURCE_FOC_INPUT_INVALID;
    /* Exactly one PI integration, on this PWM's fresh old-frame feedback. */
    if(!SourceFoc_Step(&next,id_ref,iq_ref,old_id,old_iq,limit))
        return next.failure_reason;
    if(source->saturated || next.saturated)return SOURCE_FOC_LIMIT_INVALID;
    if(new_id*new_id+new_iq*new_iq>next.measured_trip_pu*next.measured_trip_pu)return SOURCE_FOC_OVER_CURRENT;
    d=c*next.id_reference_pu+s*next.iq_reference_pu;
    q=c*next.iq_reference_pu-s*next.id_reference_pu;
    norm=d*d+q*q;
    /* Float rotation can overshoot the circle by one ULP. Never raise its cap. */
    if(norm>next.current_limit_pu*next.current_limit_pu) {
        scale=next.current_limit_pu/sqrtf(norm);d*=scale;q*=scale;
    }
    next.id_reference_pu=d;next.iq_reference_pu=q;
    d=c*next.voltage_d+s*next.voltage_q;
    q=c*next.voltage_q-s*next.voltage_d;
    if(!isfinite(d)||!isfinite(q)||d*d+q*q>limit*limit)
        return SOURCE_FOC_LIMIT_INVALID;
    next.voltage_d=d;next.voltage_q=q;
    /* Match the POST-step anti-windup state to fresh Q24 feedback in the new
     * basis. This also handles unequal Kp without assuming rotational symmetry. */
    next.integral_d=d-next.kp_d*(next.id_reference_pu-new_id);
    next.integral_q=q-next.kp_q*(next.iq_reference_pu-new_iq);
    if(!isfinite(next.integral_d)||!isfinite(next.integral_q))
        return SOURCE_FOC_CALC_INVALID;
    *destination=next;
    return SOURCE_FOC_OK;
}
