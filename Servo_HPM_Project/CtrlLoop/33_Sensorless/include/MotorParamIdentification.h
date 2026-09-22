#ifndef MOTOR_PARAM_IDENTIFICATION_H_
#define MOTOR_PARAM_IDENTIFICATION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_IDENT_IDLE = 0,
    MOTOR_IDENT_HFI_ACQUIRE,
    MOTOR_IDENT_RS_POS_SETTLE,
    MOTOR_IDENT_RS_POS_SAMPLE,
    MOTOR_IDENT_RS_NEG_SETTLE,
    MOTOR_IDENT_RS_NEG_SAMPLE,
    MOTOR_IDENT_ZERO_BEFORE_LD,
    MOTOR_IDENT_LD_TEST,
    MOTOR_IDENT_ZERO_BEFORE_LQ,
    MOTOR_IDENT_LQ_TEST,
    MOTOR_IDENT_ZERO_BEFORE_FLUX,
    MOTOR_IDENT_FLUX_START,
    MOTOR_IDENT_FLUX_SETTLE,
    MOTOR_IDENT_FLUX_SAMPLE,
    MOTOR_IDENT_STOPPING,
    MOTOR_IDENT_COMPLETE,
    MOTOR_IDENT_FAILED
} MOTOR_PARAM_IDENT_STATE;

#define MOTOR_IDENT_FAULT_CONFIG          (1UL << 0)
#define MOTOR_IDENT_FAULT_DRIVE           (1UL << 1)
#define MOTOR_IDENT_FAULT_HFI_TIMEOUT     (1UL << 2)
#define MOTOR_IDENT_FAULT_HFI_LOST        (1UL << 3)
#define MOTOR_IDENT_FAULT_EXCESS_SPEED    (1UL << 4)
#define MOTOR_IDENT_FAULT_SIGNAL          (1UL << 5)
#define MOTOR_IDENT_FAULT_RS_RANGE        (1UL << 6)
#define MOTOR_IDENT_FAULT_LD_RANGE        (1UL << 7)
#define MOTOR_IDENT_FAULT_LQ_RANGE        (1UL << 8)
#define MOTOR_IDENT_FAULT_FLUX_RANGE      (1UL << 9)
#define MOTOR_IDENT_FAULT_START_REJECTED  (1UL << 10)
#define MOTOR_IDENT_FAULT_TIMEOUT         (1UL << 11)
#define MOTOR_IDENT_FAULT_FEEDBACK        (1UL << 12)

typedef struct {
    float sample_time_s;
    float base_electrical_rad_s;
    float pole_pairs;
    float voltage_base_v;
    float current_base_a;
    float current_limit_pu;
    float resistance_current_pu;
    float inductance_current_pu;
    float flux_speed_rpm;
    float flux_speed_tolerance_rpm;
    float stationary_speed_limit_rpm;
    float minimum_current_delta_pu;
    float minimum_flux_voltage_pu;
    uint16_t minimum_hfi_quality;
    uint16_t reserved;
    uint32_t hfi_lock_cycles;
    uint32_t settle_cycles;
    uint32_t average_cycles;
    uint32_t inductance_half_period_cycles;
    uint32_t inductance_test_cycles;
    uint32_t flux_settle_cycles;
    uint32_t flux_average_cycles;
    uint32_t timeout_cycles;
} MOTOR_PARAM_IDENT_CONFIG;

typedef struct {
    float rs_pu;
    float ld_pu;
    float lq_pu;
    float ls_pu;
    float flux_pu;
    float rs_ohm;
    float ld_h;
    float lq_h;
    float ls_h;
    float flux_wb;
    float rs_positive_current_pu;
    float rs_negative_current_pu;
    float rs_positive_voltage_pu;
    float rs_negative_voltage_pu;
    uint32_t ld_samples;
    uint32_t lq_samples;
    uint32_t flux_samples;
    uint16_t quality;
    uint16_t valid;
} MOTOR_PARAM_IDENT_RESULT;

typedef struct {
    uint16_t drive_enabled;
    uint16_t fault_active;
    uint16_t hfi_locked;
    uint16_t hfi_quality;
    uint16_t feedback_valid;
    uint16_t startup_idle;
    uint16_t startup_running;
    uint16_t reserved;
    float hfi_theta_turns;
    float control_theta_turns;
    float speed_rpm;
    float voltage_alpha_pu;
    float voltage_beta_pu;
    float current_alpha_pu;
    float current_beta_pu;
} MOTOR_PARAM_IDENT_INPUT;

typedef struct {
    uint16_t control_active;
    uint16_t angle_override;
    uint16_t current_override;
    uint16_t request_start;
    uint16_t request_stop;
    uint16_t requires_hfi;
    uint16_t reserved0;
    uint16_t reserved1;
    float theta_turns;
    float id_ref_pu;
    float iq_ref_pu;
    int16_t start_direction;
    uint16_t reserved2;
    int32_t start_speed_rpm;
} MOTOR_PARAM_IDENT_OUTPUT;

typedef struct {
    MOTOR_PARAM_IDENT_CONFIG config;
    MOTOR_PARAM_IDENT_RESULT result;
    MOTOR_PARAM_IDENT_OUTPUT output;
    MOTOR_PARAM_IDENT_STATE state;
    uint32_t fault_flags;
    uint32_t state_cycles;
    uint32_t total_cycles;
    uint32_t hfi_good_cycles;
    uint32_t sample_count;
    float captured_theta_turns;
    float sum_current;
    float sum_voltage;
    float rs_pos_current;
    float rs_pos_voltage;
    float inductance_numerator;
    float inductance_denominator;
    float previous_axis_current;
    float previous_axis_voltage;
    float flux_sum;
    float flux_sum_square;
    float previous_d_current;
    float previous_q_current;
    uint16_t previous_sample_valid;
    uint16_t start_acknowledged;
} MOTOR_PARAM_IDENT_RUNTIME;

void MotorParamIdentification_DefaultConfig(
    MOTOR_PARAM_IDENT_CONFIG *config,
    float sample_time_s,
    float base_electrical_rad_s,
    float pole_pairs,
    float voltage_base_v,
    float current_base_a,
    float current_limit_pu);
uint16_t MotorParamIdentification_ValidateConfig(
    const MOTOR_PARAM_IDENT_CONFIG *config);
void MotorParamIdentification_Init(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    const MOTOR_PARAM_IDENT_CONFIG *config);
uint16_t MotorParamIdentification_Start(MOTOR_PARAM_IDENT_RUNTIME *runtime);
void MotorParamIdentification_Abort(MOTOR_PARAM_IDENT_RUNTIME *runtime,
                                    uint32_t fault_flags);
void MotorParamIdentification_AcknowledgeStart(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    uint16_t accepted);
void MotorParamIdentification_Step(
    MOTOR_PARAM_IDENT_RUNTIME *runtime,
    const MOTOR_PARAM_IDENT_INPUT *input);
uint16_t MotorParamIdentification_IsBusy(
    const MOTOR_PARAM_IDENT_RUNTIME *runtime);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAM_IDENTIFICATION_H_ */
