#ifndef SENSORLESS_SHADOW_H_
#define SENSORLESS_SHADOW_H_

#include <stdint.h>

#include "HfiEstimator.h"
#include "MotorParamProfile.h"
#include "MotorParamIdentification.h"
#include "RotorFeedback.h"
#include "SensorlessStartup.h"
#include "SmoPll.h"
#include "SensorlessSmoTrace.h"
#include "SensorlessAlignTrace.h"
typedef union { SL_SMO_TRACE smo; SL_ALIGN_TRACE align; SL_PROBE_TRACE probe; } SL_OBSERVATION_TRACE;
typedef char sl_observation_size[(sizeof(SL_OBSERVATION_TRACE)==sizeof(SL_SMO_TRACE))?1:-1];
extern volatile SL_OBSERVATION_TRACE g_observation_trace;
/* Request is latched only at accepted START: 1 ALIGN, 0 original SMO window. */
extern volatile uint32_t g_observation_request, g_observation_kind;
#define g_smo_trace (g_observation_trace.smo)
#define g_align_trace (g_observation_trace.align)
#define g_probe_trace (g_observation_trace.probe)
void SensorlessShadow_RecordAlignCurrent(float voltage_limit, uint16_t raw_sdm);
#include "SourceFoc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORLESS_CONFIG_FALLBACK_SAMPLE_RATE  (1UL << 0)
#define SENSORLESS_CONFIG_FALLBACK_RS           (1UL << 1)
#define SENSORLESS_CONFIG_FALLBACK_LS           (1UL << 2)
#define SENSORLESS_CONFIG_FALLBACK_SPEED_BASE   (1UL << 3)
#define SENSORLESS_CONFIG_FALLBACK_POLE_PAIRS   (1UL << 4)

typedef struct {
    ROTOR_FEEDBACK_MUX feedback_mux;
    SMO_PLL_PARAMS params;
    SMO_PLL_STATE observer;
    HFI_ESTIMATOR_PARAMS hfi_params;
    HFI_ESTIMATOR_STATE hfi;
    SENSORLESS_MOTOR_PROFILE motor_profile;
    SENSORLESS_MOTOR_OVERRIDE motor_override;
    MOTOR_PARAM_IDENT_RUNTIME identification;
    SENSORLESS_STARTUP_RUNTIME startup;
    float voltage_scale;
    int16_t expected_direction;
    uint16_t initialized;
    uint16_t output_owned;
    uint16_t fault_hook_sent;
    uint16_t encoder_valid;
    uint16_t hfi_was_required;
    uint16_t reserved;
    uint32_t config_flags;
    uint32_t pwm_counter;
    uint32_t encoder_stale_cycles;
    int32_t encoder_last_theta_q24;
} SENSORLESS_SHADOW_RUNTIME;

typedef struct {
    uint16_t initialized;
    uint16_t mode;
    uint16_t control_source;
    uint16_t control_override_active;
    uint16_t profile_valid_for_control;
    uint16_t startup_state;
    uint16_t fault_request;
    uint16_t observer_locked;
    uint16_t observer_quality;
    uint16_t encoder_valid;
    uint16_t hfi_capable;
    uint16_t hfi_active;
    uint16_t hfi_locked;
    uint16_t hfi_quality;
    int16_t expected_direction;
    uint16_t reserved;
    uint32_t config_flags;
    uint32_t pwm_counter;
    uint32_t encoder_stale_cycles;
    uint32_t encoder_good_cycles;
    int32_t encoder_theta_q24;
    int32_t observer_theta_q24;
    int32_t theta_error_q24;
    int32_t encoder_speed_rpm;
    int32_t observer_speed_rpm;
    float current_alpha_pu;
    float current_beta_pu;
    float voltage_alpha_pu;
    float voltage_beta_pu;
    float bus_voltage_pu;
    float emf_alpha_pu;
    float emf_beta_pu;
    float emf_magnitude_pu;
    float pll_phase_error;
    float pll_omega_e_rad_s;
    int32_t hfi_theta_q24;
    int32_t hfi_speed_rpm;
    float hfi_injection_current_pu;
    float hfi_d_response_pu;
    float hfi_q_response_pu;
    float hfi_phase_error;
    float saliency_ratio;
    float model_gain;
    float stator_resistance_pu;
    float id_command_pu;
    float iq_command_pu;
    float speed_command_rpm;
    uint32_t startup_fault_flags;
    uint32_t motor_validation_flags;
    float hfi_pole_detect_current_pu;
    float hfi_base_current_alpha_pu;
    float hfi_base_current_beta_pu;
    float hfi_hf_current_alpha_pu;
    float hfi_hf_current_beta_pu;
    float hfi_pole_positive_response_pu;
    float hfi_pole_negative_response_pu;
    float hfi_pole_contrast;
    uint16_t hfi_axis_locked;
    uint16_t hfi_pole_detected;
    uint16_t hfi_polarity_corrected;
    uint16_t hfi_pole_state;
    uint32_t hfi_demod_update_counter;
    int32_t observer_raw_theta_q24;
    float observer_phase_advance_turns;
    float observer_omega_tracking_e_rad_s;
    float observer_emf_confidence;
    float observer_current_error_pu;
    uint32_t observer_weak_emf_counter;
    uint16_t identification_state;
    uint16_t identification_quality;
    uint16_t identification_valid;
    uint16_t identification_active;
    uint32_t identification_fault_flags;
    float identified_rs_pu;
    float identified_ld_pu;
    float identified_lq_pu;
    float identified_ls_pu;
    float identified_flux_pu;
    float identified_rs_ohm;
    float identified_ld_h;
    float identified_lq_h;
    float identified_ls_h;
    float identified_flux_wb;
    uint32_t first_fault, failure_state, stage_cycles, total_cycles;
    uint32_t debug_mode, debug_completed, current_failure;
    int32_t open_theta_q24, control_theta_q24, observer_open_error_q24;
    float blend_fraction, phase_error_rad;
    float id_feedback_pu, iq_feedback_pu, id_limited_pu, iq_limited_pu;
    float ud_pu, uq_pu;
    uint32_t voltage_saturated, lock_reason;
} SENSORLESS_SHADOW_DIAG;

/* Fixed 32-bit fields, little-endian IEEE float; trace ABI independent of mailbox. */
#define SENSORLESS_TRACE_CAPACITY 128U
/* 10 ms at 16 kHz: 1.27 s plus forced state/final samples; diagnostic only. */
#define SENSORLESS_TRACE_DIVIDER 160U
typedef struct {
    int32_t chain_tick, chain_valid, sdm_u, sdm_v, adc_u, adc_v, adc_w, zero_u, zero_v, zero_w, iu_q24, iv_q24, iw_q24, clark_u_q24, clark_v_q24, alpha_q24, beta_q24, sample_window, phase_mode, ualpha_q24, ubeta_q24, pwm_u, pwm_v, pwm_w, pwm_period, duty_a_q24, duty_b_q24, duty_c_q24, bus_reference_q24;
} SENSORLESS_CHAIN_SAMPLE;
extern SENSORLESS_CHAIN_SAMPLE g_sensorless_chain;

typedef struct {
    uint32_t tick, mode, state, stage_cycles, total_cycles, first_fault;
    int32_t open_theta_q24, observer_theta_q24, control_theta_q24, angle_error_q24;
    float speed_ref_rpm, observer_speed_rpm, reference_speed_rpm;
    uint32_t reference_valid;
    float id_ref, iq_ref, id_fb, iq_fb, id_limited, iq_limited;
    float bus_pu, ud, uq, emf, phase_error_rad, blend;
    uint32_t saturated, quality, locked, lock_reason, current_failure;
    uint32_t encoder_tick_ms, encoder_motor_raw, encoder_output_raw;
    uint32_t encoder_valid, encoder_sequence, encoder_last_good_ms;
    SENSORLESS_CHAIN_SAMPLE chain;
} SENSORLESS_TRACE_SAMPLE;
typedef struct {
    uint32_t magic, abi, sample_bytes, capacity;
    uint32_t sequence, write_index, count, frozen, active, record_id;
    SENSORLESS_TRACE_SAMPLE samples[SENSORLESS_TRACE_CAPACITY];
} SENSORLESS_TRACE;
extern volatile SENSORLESS_TRACE g_sensorless_trace;
void SensorlessShadow_CurrentFault(uint16_t reason);
void SensorlessShadow_ClearTrace(void);
extern SENSORLESS_SHADOW_RUNTIME g_sensorless_shadow;
extern volatile SENSORLESS_SHADOW_DIAG g_sensorless_diag;

extern SOURCE_FOC_CURRENT g_source_foc_current;
extern SOURCE_FOC_RECIPE g_source_foc_recipe;
extern volatile uint16_t g_source_foc_ready;
uint16_t SensorlessShadow_ApplySourceRecipe(const SOURCE_FOC_RECIPE *recipe);
void SensorlessShadow_Init(void);
void SensorlessShadow_Reset(void);
void SensorlessShadow_SetMode(ROTOR_MODE mode);
uint16_t SensorlessShadow_ConfigureMotorOverride(
    const SENSORLESS_MOTOR_OVERRIDE *override_values);
uint16_t SensorlessShadow_ConfigureStartup(
    const SENSORLESS_STARTUP_CONFIG *config);
uint16_t SensorlessShadow_ConfigureHfi(
    const HFI_ESTIMATOR_PARAMS *params);
uint16_t SensorlessShadow_ConfigureSmo(
    const SMO_PLL_PARAMS *params);
uint16_t SensorlessShadow_StartParamIdentification(
    const MOTOR_PARAM_IDENT_CONFIG *config);
void SensorlessShadow_AbortParamIdentification(void);
uint16_t SensorlessShadow_ApplyIdentifiedMotor(void);
uint16_t SensorlessShadow_RequestStart(int16_t direction, int32_t target_speed_rpm);
uint16_t SensorlessShadow_UpdateTarget(int16_t direction, int32_t target_speed_rpm);
void SensorlessShadow_RequestStop(void);
void SensorlessShadow_ClearFault(void);
void SensorlessShadow_PreFoc(void);
void SensorlessShadow_PostFoc(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLESS_SHADOW_H_ */
