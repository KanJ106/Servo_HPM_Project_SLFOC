#ifndef HFI_ESTIMATOR_H_
#define HFI_ESTIMATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HFI_POLE_WAIT_AXIS = 0,
    HFI_POLE_POSITIVE_SETTLE,
    HFI_POLE_POSITIVE_MEASURE,
    HFI_POLE_ZERO_SETTLE,
    HFI_POLE_NEGATIVE_SETTLE,
    HFI_POLE_NEGATIVE_MEASURE,
    HFI_POLE_COMPLETE,
    HFI_POLE_FAILED
} HFI_POLE_STATE;

typedef struct {
    float sample_time_s;
    float injection_frequency_hz;
    float injection_current_pu;
    float pole_detect_current_pu;
    float pole_min_contrast;
    float response_lpf_alpha;
    float speed_lpf_alpha;
    float tracking_kp;
    float tracking_ki;
    float min_response_pu;
    float lock_error;
    float max_electrical_rad_s;
    int16_t saliency_polarity;
    int16_t pole_response_polarity;
    uint16_t lock_cycles;
    uint16_t unlock_cycles;
    uint16_t pole_settle_cycles;
    uint16_t pole_measure_cycles;
} HFI_ESTIMATOR_PARAMS;

typedef struct {
    float theta_turns;
    float omega_integrator_e_rad_s;
    float omega_e_rad_s;
    float phase_error;
    float d_response_pu;
    float q_response_pu;
    float base_current_alpha_pu;
    float base_current_beta_pu;
    float hf_current_alpha_pu;
    float hf_current_beta_pu;
    float positive_half_alpha_pu;
    float positive_half_beta_pu;
    float half_sum_alpha_pu;
    float half_sum_beta_pu;
    float pole_positive_response_pu;
    float pole_negative_response_pu;
    float pole_contrast;
    float pole_measure_sum_pu;
    float injection_current_pu;
    float pole_detect_current_pu;
    uint16_t quality;
    uint16_t locked;
    uint16_t axis_locked;
    uint16_t pole_detected;
    uint16_t polarity_corrected;
    uint16_t active;
    uint16_t pole_state;
    int16_t carrier_sign;
    uint32_t carrier_half_period_cycles;
    uint32_t carrier_cycles;
    uint32_t half_sample_count;
    uint32_t lock_counter;
    uint32_t unlock_counter;
    uint32_t pole_stage_cycles;
    uint32_t pole_measure_count;
    uint32_t update_counter;
    uint32_t demod_update_counter;
    uint32_t numeric_fault_count;
} HFI_ESTIMATOR_STATE;

void HfiEstimator_DefaultParams(HFI_ESTIMATOR_PARAMS *params,
                                float sample_time_s,
                                float max_electrical_rad_s,
                                float current_limit_pu,
                                int16_t saliency_polarity);
uint16_t HfiEstimator_ValidateParams(const HFI_ESTIMATOR_PARAMS *params);
void HfiEstimator_Init(HFI_ESTIMATOR_STATE *state,
                       const HFI_ESTIMATOR_PARAMS *params,
                       float initial_theta_turns);
void HfiEstimator_Enable(HFI_ESTIMATOR_STATE *state,
                         const HFI_ESTIMATOR_PARAMS *params,
                         float seed_theta_turns);
void HfiEstimator_Disable(HFI_ESTIMATOR_STATE *state);
void HfiEstimator_Step(HFI_ESTIMATOR_STATE *state,
                       const HFI_ESTIMATOR_PARAMS *params,
                       float current_alpha_pu,
                       float current_beta_pu);
float HfiEstimator_GetInjectionCurrent(const HFI_ESTIMATOR_STATE *state);
float HfiEstimator_GetPoleDetectCurrent(const HFI_ESTIMATOR_STATE *state);

#ifdef __cplusplus
}
#endif

#endif /* HFI_ESTIMATOR_H_ */
