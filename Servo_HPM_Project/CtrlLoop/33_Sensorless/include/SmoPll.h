#ifndef SMO_PLL_H_
#define SMO_PLL_H_

#include <stdint.h>

/* Unsaturated scalar current-error pole is 1 - g*(Rs + K/boundary). */
#define SMO_PLL_STABILITY_TARGET (1.0f)
#define SMO_PLL_STABILITY_LIMIT (2.0f)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float sample_time_s;
    float stator_resistance_pu;
    float current_model_gain;
    float switching_gain;
    float boundary_pu;
    float emf_lpf_alpha;
    float emf_delay_comp_gain;
    float pll_kp;
    float pll_ki;
    float pll_speed_lpf_alpha;
    float weak_emf_speed_decay;
    float min_emf_pu;
    float unlock_emf_ratio;
    float lock_phase_error; /* radians, absolute wrapped raw-to-PLL error */
    float max_electrical_rad_s;
    float max_accel_e_rad_s2;
    float current_hat_limit_pu;
    uint16_t lock_cycles;
    uint16_t unlock_cycles;
} SMO_PLL_PARAMS;

typedef struct {
    float voltage_alpha_pu;
    float voltage_beta_pu;
    float current_alpha_pu;
    float current_beta_pu;
    int16_t expected_direction;
} SMO_PLL_INPUT;

typedef struct {
    float current_hat_alpha_pu;
    float current_hat_beta_pu;
    float current_error_magnitude_pu;
    float switching_alpha_pu;
    float switching_beta_pu;
    float emf_alpha_pu;
    float emf_beta_pu;
    float emf_magnitude_pu;
    float emf_confidence;
    float theta_raw_turns;
    float theta_pll_turns;
    float phase_advance_turns;
    float omega_integrator_e_rad_s;
    float omega_tracking_e_rad_s;
    float omega_e_rad_s;
    float phase_error; /* sine detector retained for PLL regulation */
    float phase_error_rad;
    uint16_t lock_reason; /* 0 qualifying, 1 locked, 2 weak EMF, 3 phase, 4 numeric */
    uint16_t reserved_lock;
    uint16_t quality;
    uint16_t locked;
    uint16_t lock_counter;
    uint16_t unlock_counter;
    uint32_t update_counter;
    uint32_t weak_emf_counter;
    uint32_t numeric_fault_count;
} SMO_PLL_STATE;

uint16_t SmoPll_ValidateParams(const SMO_PLL_PARAMS *params);
void SmoPll_Init(SMO_PLL_STATE *state, const SMO_PLL_PARAMS *params, float initial_theta_turns);
void SmoPll_Reset(SMO_PLL_STATE *state, float initial_theta_turns);
void SmoPll_SeedTracking(SMO_PLL_STATE *state,
                         float theta_turns,
                         float omega_e_rad_s);
void SmoPll_Step(SMO_PLL_STATE *state, const SMO_PLL_PARAMS *params, const SMO_PLL_INPUT *input);

#ifdef __cplusplus
}
#endif

#endif /* SMO_PLL_H_ */
