#ifndef MOTOR_PARAM_PROFILE_H_
#define MOTOR_PARAM_PROFILE_H_

#include <stdint.h>

#include "SmoPll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PARAM_OVERRIDE_RS              (1UL << 0)
#define MOTOR_PARAM_OVERRIDE_LD              (1UL << 1)
#define MOTOR_PARAM_OVERRIDE_LQ              (1UL << 2)
#define MOTOR_PARAM_OVERRIDE_POLE_PAIRS      (1UL << 3)
#define MOTOR_PARAM_OVERRIDE_MAX_SPEED       (1UL << 4)
#define MOTOR_PARAM_OVERRIDE_RATED_CURRENT   (1UL << 5)
#define MOTOR_PARAM_OVERRIDE_CURRENT_LIMIT   (1UL << 6)

#define MOTOR_PARAM_FLAG_SAMPLE_RATE         (1UL << 0)
#define MOTOR_PARAM_FLAG_RS                  (1UL << 1)
#define MOTOR_PARAM_FLAG_LD                  (1UL << 2)
#define MOTOR_PARAM_FLAG_LQ                  (1UL << 3)
#define MOTOR_PARAM_FLAG_SPEED_BASE          (1UL << 4)
#define MOTOR_PARAM_FLAG_MAX_SPEED           (1UL << 5)
#define MOTOR_PARAM_FLAG_POLE_PAIRS          (1UL << 6)
#define MOTOR_PARAM_FLAG_RATED_CURRENT       (1UL << 7)
#define MOTOR_PARAM_FLAG_CURRENT_LIMIT       (1UL << 8)
#define MOTOR_PARAM_FLAG_MODEL_GAIN_CLAMPED  (1UL << 9)
#define MOTOR_PARAM_FLAG_HFI_SALIENCY        (1UL << 10)

typedef struct {
    float sample_hz;
    float stator_resistance_pu;
    float ld_pu;
    float lq_pu;
    float speed_base_rpm;
    float max_speed_rpm;
    float pole_pairs;
    float rated_current_pu;
    float current_limit_pu;
} SENSORLESS_MOTOR_RAW;

typedef struct {
    uint32_t mask;
    float stator_resistance_pu;
    float ld_pu;
    float lq_pu;
    float max_speed_rpm;
    float pole_pairs;
    float rated_current_pu;
    float current_limit_pu;
} SENSORLESS_MOTOR_OVERRIDE;

typedef struct {
    SENSORLESS_MOTOR_RAW effective;
    float average_inductance_pu;
    float base_electrical_rad_s;
    float max_electrical_rad_s;
    float observer_model_gain;
    float saliency_ratio;
    int16_t saliency_polarity;
    uint16_t hfi_capable;
    uint32_t validation_flags;
    uint32_t override_mask;
    uint16_t valid_for_shadow;
    uint16_t valid_for_control;
} SENSORLESS_MOTOR_PROFILE;

void MotorParamProfile_ClearOverride(SENSORLESS_MOTOR_OVERRIDE *override_values);
void MotorParamProfile_Build(const SENSORLESS_MOTOR_RAW *raw,
                             const SENSORLESS_MOTOR_OVERRIDE *override_values,
                             SENSORLESS_MOTOR_PROFILE *profile);
void MotorParamProfile_ApplyToSmo(const SENSORLESS_MOTOR_PROFILE *profile,
                                  SMO_PLL_PARAMS *params);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAM_PROFILE_H_ */
