#ifndef SOURCE_FOC_H_
#define SOURCE_FOC_H_
#include <stdint.h>
#include "MotorParamProfile.h"

/* All SI currents/voltages are phase peak values, not line/RMS values.
 * voltage_base_v is the voltage represented by Svpwm.Ualpha == 1 pu.
 * A board calibration must establish these two bases before motion. */
typedef struct {
    uint32_t recipe_id, recipe_revision;
    float phase_rs_ohm, phase_ld_h, phase_lq_h;
    float current_base_a, voltage_base_v;
    float rated_current_a, current_limit_a;
    float max_speed_rpm, current_bandwidth_hz;
    uint16_t pole_pairs;
    uint16_t allow_continuous_rotation;
} SOURCE_FOC_RECIPE;

typedef enum {
    SOURCE_FOC_OK = 0, SOURCE_FOC_OVER_CURRENT, SOURCE_FOC_CONFIG_INVALID,
    SOURCE_FOC_INPUT_INVALID, SOURCE_FOC_CALC_INVALID, SOURCE_FOC_LIMIT_INVALID
} SOURCE_FOC_FAILURE;
typedef struct {
    float kp_d, kp_q, ki_step, current_limit_pu;
    float integral_d, integral_q;
    float id_reference_pu, iq_reference_pu, measured_trip_pu;
    float voltage_d, voltage_q;
    uint16_t failure_reason, reserved;
    uint32_t numeric_faults, overcurrent_faults;
    uint16_t configured, saturated;
} SOURCE_FOC_CURRENT;

uint16_t SourceFoc_BuildProfile(const SOURCE_FOC_RECIPE *recipe,
    float sample_hz, float speed_base_rpm, SENSORLESS_MOTOR_PROFILE *profile);
uint16_t SourceFoc_Configure(SOURCE_FOC_CURRENT *state,
    const SENSORLESS_MOTOR_PROFILE *profile, float bandwidth_hz);
void SourceFoc_Reset(SOURCE_FOC_CURRENT *state);
uint16_t SourceFoc_Step(SOURCE_FOC_CURRENT *state, float id_ref, float iq_ref,
    float id_fb, float iq_fb, float voltage_limit_pu);
/* Prepare one complete current step and basis change in local storage.
 * Returns a SOURCE_FOC_FAILURE; neither source nor destination changes on failure.
 * Fresh old/new feedback must come from the SAME Clarke sample. */
uint16_t SourceFoc_PrepareFrame(const SOURCE_FOC_CURRENT *source,
    SOURCE_FOC_CURRENT *destination, float id_ref, float iq_ref,
    float old_id, float old_iq, float new_id, float new_iq,
    float cosine_delta, float sine_delta, float voltage_limit_pu);
#endif
