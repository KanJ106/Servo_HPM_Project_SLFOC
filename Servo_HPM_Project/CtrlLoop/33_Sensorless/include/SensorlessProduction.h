#ifndef SENSORLESS_PRODUCTION_H_
#define SENSORLESS_PRODUCTION_H_

#include <stdint.h>

#include "MotorParamProfile.h"
#include "MotorParamIdentification.h"
#include "SourceFoc.h"
#include "ProductionMetrics.h"
#include "SensorlessStartup.h"
#include "SmoPll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORLESS_PRODUCTION_MAGIC          (0x534C4637UL)
#define SENSORLESS_PRODUCTION_ABI_VERSION    (0x00070000UL)
#define SENSORLESS_PRODUCTION_ARM_KEY        (0x534CU)

#define SENSORLESS_FIRMWARE_VERSION (0x20260919UL)
#define SENSORLESS_PARAMETER_VERSION (22UL) /* Isolated one-shot RAM frame transaction */
#define SENSORLESS_TUNING_DEBUG (1UL << 9)
#define SENSORLESS_TUNING_ALIGN              (1UL << 0)
#define SENSORLESS_TUNING_CURRENT            (1UL << 1)
#define SENSORLESS_TUNING_SPEED              (1UL << 2)
#define SENSORLESS_TUNING_PI                 (1UL << 3)
#define SENSORLESS_TUNING_TIMING             (1UL << 4)
#define SENSORLESS_TUNING_LOCK               (1UL << 5)
#define SENSORLESS_TUNING_MONITOR            (1UL << 6)
#define SENSORLESS_TUNING_HFI                (1UL << 7)
#define SENSORLESS_TUNING_SMO                (1UL << 8)

typedef enum {
    SENSORLESS_PROD_CMD_NONE = 0,
    SENSORLESS_PROD_CMD_APPLY_MOTOR = 1,
    SENSORLESS_PROD_CMD_APPLY_TUNING = 2,
    SENSORLESS_PROD_CMD_START = 3,
    SENSORLESS_PROD_CMD_RUN_TEST = 4,
    SENSORLESS_PROD_CMD_STOP = 5,
    SENSORLESS_PROD_CMD_CLEAR_FAULT = 6,
    SENSORLESS_PROD_CMD_RESTORE_DEFAULTS = 7,
    SENSORLESS_PROD_CMD_UPDATE_TARGET = 8,
    SENSORLESS_PROD_CMD_RUN_IDENTIFICATION = 9,
    SENSORLESS_PROD_CMD_APPLY_IDENTIFIED_MOTOR = 10,
    SENSORLESS_PROD_CMD_ABORT_IDENTIFICATION = 11,
    SENSORLESS_PROD_CMD_APPLY_SOURCE_RECIPE = 12,
    SENSORLESS_PROD_CMD_RUN_SEQUENCE = 13
} SENSORLESS_PRODUCTION_COMMAND;

typedef enum {
    SENSORLESS_PROD_RESULT_IDLE = 0,
    SENSORLESS_PROD_RESULT_ACCEPTED = 1,
    SENSORLESS_PROD_RESULT_BUSY = 2,
    SENSORLESS_PROD_RESULT_BAD_ARM_KEY = 3,
    SENSORLESS_PROD_RESULT_SERVO_NOT_READY = 4,
    SENSORLESS_PROD_RESULT_INVALID_PROFILE = 5,
    SENSORLESS_PROD_RESULT_INVALID_TUNING = 6,
    SENSORLESS_PROD_RESULT_INVALID_COMMAND = 7,
    SENSORLESS_PROD_RESULT_TEST_RUNNING = 8,
    SENSORLESS_PROD_RESULT_TEST_PASS = 9,
    SENSORLESS_PROD_RESULT_TEST_FAIL = 10,
    SENSORLESS_PROD_RESULT_TEST_TIMEOUT = 11,
    SENSORLESS_PROD_RESULT_IDENT_RUNNING = 12,
    SENSORLESS_PROD_RESULT_IDENT_PASS = 13,
    SENSORLESS_PROD_RESULT_IDENT_FAIL = 14
} SENSORLESS_PRODUCTION_RESULT;

typedef enum {
    SENSORLESS_PROD_STATE_IDLE = 0,
    SENSORLESS_PROD_STATE_STARTING,
    SENSORLESS_PROD_STATE_STABILIZING,
    SENSORLESS_PROD_STATE_HOLDING,
    SENSORLESS_PROD_STATE_STOPPING,
    SENSORLESS_PROD_STATE_PASS,
    SENSORLESS_PROD_STATE_FAIL
} SENSORLESS_PRODUCTION_STATE;

typedef struct {
    uint32_t mask;
    float align_angle_turns;
    float align_current_pu;
    float startup_iq_pu;
    float iq_limit_pu;
    float handoff_speed_rpm;
    float open_loop_accel_rpm_s;
    float run_accel_rpm_s;
    float speed_kp;
    float speed_ki;
    uint16_t minimum_lock_quality;
    uint16_t reserved;
    uint32_t align_ms;
    uint32_t lock_dwell_ms;
    uint32_t blend_ms;
    uint32_t lock_timeout_ms;
    uint32_t unlock_dwell_ms;
    uint32_t monitor_dwell_ms;
    uint32_t fallback_ready_ms;
    uint32_t stop_dwell_ms;
    uint32_t stop_timeout_ms;
    int32_t fallback_max_angle_error_deg;
    int32_t monitor_max_angle_error_deg;
    int32_t monitor_max_speed_error_rpm;
    int32_t stop_speed_rpm;
    float sensorless_exit_speed_rpm;
    uint16_t minimum_hfi_quality;
    uint16_t hfi_enable;
    uint16_t allow_open_loop_fallback;
    uint16_t reserved1;
    uint32_t hfi_lock_dwell_ms;
    uint32_t hfi_timeout_ms;
    float hfi_injection_frequency_hz;
    float hfi_injection_current_pu;
    float hfi_response_lpf_alpha;
    float hfi_speed_lpf_alpha;
    float hfi_tracking_kp;
    float hfi_tracking_ki;
    float hfi_min_response_pu;
    float hfi_lock_error;
    uint32_t hfi_estimator_lock_ms;
    uint32_t hfi_estimator_unlock_ms;
    float hfi_pole_detect_current_pu;
    float hfi_pole_min_contrast;
    int16_t hfi_pole_response_polarity;
    uint16_t reserved2;
    uint32_t hfi_pole_settle_ms;
    uint32_t hfi_pole_measure_ms;
    float smo_switching_gain;
    float smo_boundary_pu;
    float smo_emf_lpf_alpha;
    float smo_emf_delay_comp_gain;
    float smo_pll_kp;
    float smo_pll_ki;
    float smo_pll_speed_lpf_alpha;
    float smo_weak_emf_speed_decay;
    float smo_min_emf_pu;
    float smo_unlock_emf_ratio;
    float smo_lock_phase_error;
    float smo_max_accel_e_rad_s2;
    float smo_current_hat_limit_pu;
    uint32_t smo_lock_ms;
    uint32_t smo_unlock_ms;
    uint16_t debug_mode, reserved_debug;
    uint32_t current_ramp_ms, if_hold_ms, iq_hold_ms;
    uint32_t ramp_timeout_ms, total_timeout_ms, run_test_ms;
} SENSORLESS_PRODUCTION_TUNING;

#define SENSORLESS_SEQUENCE_MAX_POINTS (6U)
typedef struct {
    int16_t direction;
    uint16_t reserved;
    int32_t target_speed_rpm, speed_tolerance_rpm;
    uint32_t hold_ms, timeout_ms;
    PRODUCTION_METRIC_LIMITS limits;
} SENSORLESS_TEST_POINT;
typedef struct {
    uint16_t count, reserved;
    SENSORLESS_TEST_POINT points[SENSORLESS_SEQUENCE_MAX_POINTS];
} SENSORLESS_TEST_SEQUENCE;

typedef struct {
    uint32_t magic;
    uint32_t abi_version;
    uint32_t request_seq;
    uint32_t acknowledge_seq;
    uint16_t command;
    uint16_t arm_key;
    int16_t direction;
    uint16_t reserved0;
    int32_t target_speed_rpm;
    uint32_t test_hold_ms;
    uint32_t test_timeout_ms;
    int32_t speed_tolerance_rpm;
    MOTOR_PARAM_IDENT_CONFIG identification_config;
    SENSORLESS_MOTOR_OVERRIDE motor;
    SENSORLESS_PRODUCTION_TUNING tuning;
    uint16_t command_result;
    uint16_t production_state;
    uint16_t startup_state;
    uint16_t control_source;
    uint16_t profile_valid;
    uint16_t observer_locked;
    uint16_t observer_quality;
    uint16_t encoder_valid;
    uint16_t hfi_capable;
    uint16_t hfi_active;
    uint16_t hfi_locked;
    uint16_t hfi_quality;
    uint16_t control_active;
    uint16_t reserved1;
    uint32_t startup_fault_flags;
    uint32_t motor_validation_flags;
    uint32_t test_elapsed_ms;
    uint32_t test_stable_ms;
    uint32_t test_hold_elapsed_ms;
    uint32_t completed_tests;
    uint32_t failed_tests;
    int32_t encoder_speed_rpm;
    int32_t observer_speed_rpm;
    int32_t hfi_speed_rpm;
    int32_t speed_error_rpm;
    int32_t theta_error_q24;
    int32_t hfi_theta_q24;
    float hfi_injection_current_pu;
    float hfi_phase_error;
    float saliency_ratio;
    float hfi_pole_detect_current_pu;
    float hfi_pole_positive_response_pu;
    float hfi_pole_negative_response_pu;
    float hfi_pole_contrast;
    uint16_t hfi_axis_locked;
    uint16_t hfi_pole_detected;
    uint16_t hfi_polarity_corrected;
    uint16_t hfi_pole_state;
    int32_t observer_raw_theta_q24;
    float observer_phase_advance_turns;
    float observer_omega_tracking_e_rad_s;
    float observer_emf_confidence;
    float observer_current_error_pu;
    uint32_t observer_weak_emf_counter;
    uint16_t identification_state;
    uint16_t identification_active;
    uint16_t identification_valid;
    uint16_t identification_quality;
    uint32_t identification_fault_flags;
    uint32_t completed_identifications;
    uint32_t failed_identifications;
    MOTOR_PARAM_IDENT_RESULT identification_result;
    SOURCE_FOC_RECIPE source_recipe;
    PRODUCTION_METRIC_LIMITS metric_limits;
    PRODUCTION_METRIC_RESULT metric_result;
    uint32_t applied_recipe_id, applied_recipe_revision;
    uint16_t source_foc_ready, reserved_source;
    SENSORLESS_TEST_SEQUENCE sequence;
    PRODUCTION_METRIC_RESULT sequence_results[SENSORLESS_SEQUENCE_MAX_POINTS];
    uint16_t sequence_active, sequence_index, sequence_completed, sequence_result;
    uint32_t firmware_version, parameter_version, parameter_sequence;
    SOURCE_FOC_RECIPE effective_recipe;
    SENSORLESS_STARTUP_CONFIG effective_startup;
    SMO_PLL_PARAMS effective_smo;
    SENSORLESS_MOTOR_PROFILE effective_motor;
    SOURCE_FOC_CURRENT effective_current;
    SENSORLESS_START_FAILURE first_failure;
    uint32_t current_failure, debug_completed;
} SENSORLESS_PRODUCTION_MAILBOX;

extern volatile SENSORLESS_PRODUCTION_MAILBOX g_sensorless_production;

void SensorlessProduction_Init(void);
void SensorlessProduction_Service1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLESS_PRODUCTION_H_ */
