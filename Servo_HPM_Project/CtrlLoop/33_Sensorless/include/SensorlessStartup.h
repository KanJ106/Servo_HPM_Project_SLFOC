#ifndef SENSORLESS_STARTUP_H_
#define SENSORLESS_STARTUP_H_

#include <stdint.h>

#include "RotorFeedback.h"
#include "SensorlessTakeoverProbe.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORLESS_START_IDLE = 0,
    SENSORLESS_START_ALIGN,
    SENSORLESS_START_HFI_ACQUIRE,
    SENSORLESS_START_HFI_RUN,
    SENSORLESS_START_OPEN_LOOP_RAMP,
    SENSORLESS_START_WAIT_LOCK,
    SENSORLESS_START_BLEND,
    SENSORLESS_START_RUN,
    SENSORLESS_START_HFI_REACQUIRE,
    SENSORLESS_START_BLEND_TO_HFI,
    SENSORLESS_START_ENCODER_FALLBACK,
    SENSORLESS_START_STOPPING,
    SENSORLESS_START_FAULT,
    SENSORLESS_START_IQ_HOLD
} SENSORLESS_START_STATE;

#define SENSORLESS_FAULT_PARAM_INVALID       (1UL << 0)
#define SENSORLESS_FAULT_LOCK_TIMEOUT        (1UL << 1)
#define SENSORLESS_FAULT_LOCK_LOST           (1UL << 2)
#define SENSORLESS_FAULT_ENCODER_DISAGREE    (1UL << 3)
#define SENSORLESS_FAULT_OBSERVER_NUMERIC    (1UL << 4)
#define SENSORLESS_FAULT_STOP_TIMEOUT        (1UL << 5)
#define SENSORLESS_FAULT_HFI_UNAVAILABLE     (1UL << 6)
#define SENSORLESS_FAULT_HFI_LOCK_TIMEOUT    (1UL << 7)
#define SENSORLESS_FAULT_HFI_LOCK_LOST       (1UL << 8)
#define SENSORLESS_FAULT_RAMP_TIMEOUT        (1UL << 9)
#define SENSORLESS_FAULT_BLEND_FAILED        (1UL << 10)
#define SENSORLESS_FAULT_START_TIMEOUT       (1UL << 11)
#define SENSORLESS_FAULT_CURRENT_OVER        (1UL << 12)
#define SENSORLESS_FAULT_CURRENT_CONFIG      (1UL << 13)
#define SENSORLESS_FAULT_CURRENT_INPUT       (1UL << 14)
#define SENSORLESS_FAULT_CURRENT_CALC        (1UL << 15)
#define SENSORLESS_FAULT_VOLTAGE_LIMIT       (1UL << 19)
#define SENSORLESS_FAULT_EXTERNAL            (1UL << 20)
#define SENSORLESS_DEBUG_FULL  (0U)
#define SENSORLESS_DEBUG_ALIGN (1U)
#define SENSORLESS_DEBUG_IF    (2U)
/* Diagnostic rotating d-axis field; never hands over to an observer. */
#define SENSORLESS_DEBUG_D_AXIS (3U)
/* 0.5 electrical turn out/back; if_hold_cycles is each leg (0.5..2 s). */
#define SENSORLESS_DEBUG_D_AXIS_SCAN (4U)
#define SENSORLESS_DEBUG_TRAVEL90 (5U)
#define SENSORLESS_DEBUG_SMO_A1 (6U)
#define SENSORLESS_DEBUG_SMO_A2 (7U)
#define SENSORLESS_DEBUG_SMO_A3 (8U)
#define SENSORLESS_DEBUG_SMO_FRAME (9U) /* One basis change, 300 ms observer hold, off. */
#define SENSORLESS_IS_PROBE(m) ((m)==SENSORLESS_DEBUG_SMO_A1 || (m)==SENSORLESS_DEBUG_SMO_A2 || (m)==SENSORLESS_DEBUG_SMO_A3 || (m)==SENSORLESS_DEBUG_SMO_FRAME)
#define SENSORLESS_IS_TRAVEL(m) ((m)==SENSORLESS_DEBUG_TRAVEL90 || SENSORLESS_IS_PROBE(m))
#define SENSORLESS_STATUS_ENCODER_FALLBACK   (1UL << 16)
#define SENSORLESS_STATUS_HFI_ACTIVE         (1UL << 17)
#define SENSORLESS_STATUS_IF_FALLBACK        (1UL << 18)

typedef struct {
    float sample_time_s;
    float pole_pairs;
    float speed_base_rpm;
    float max_speed_rpm;
    float align_angle_turns;
    float align_current_pu;
    float startup_iq_pu;
    float iq_limit_pu;
    float handoff_speed_rpm;
    float sensorless_exit_speed_rpm;
    float open_loop_accel_rpm_s;
    float run_accel_rpm_s;
    float speed_kp;
    float speed_ki;
    uint16_t minimum_lock_quality;
    uint16_t minimum_hfi_quality;
    uint16_t hfi_enable;
    uint16_t allow_open_loop_fallback;
    uint32_t align_cycles;
    uint32_t lock_dwell_cycles;
    uint32_t blend_cycles;
    uint32_t lock_timeout_cycles;
    uint32_t hfi_lock_dwell_cycles;
    uint32_t hfi_timeout_cycles;
    uint32_t unlock_dwell_cycles;
    uint32_t monitor_dwell_cycles;
    uint32_t fallback_ready_cycles;
    uint32_t stop_dwell_cycles;
    uint32_t stop_timeout_cycles;
    int32_t fallback_max_angle_error_q24;
    int32_t monitor_max_angle_error_q24;
    int32_t monitor_max_speed_error_rpm;
    int32_t stop_speed_rpm;
    uint16_t debug_mode, reserved_debug;
    uint32_t current_ramp_cycles, if_hold_cycles, iq_hold_cycles;
    uint32_t ramp_timeout_cycles, total_timeout_cycles, run_test_cycles;
} SENSORLESS_STARTUP_CONFIG;

typedef struct {
    uint32_t fault, state_cycles, total_cycles;
    uint16_t state, reserved;
    float blend, iq;
    int32_t open_theta_q24, observer_theta_q24, control_theta_q24;
} SENSORLESS_START_FAILURE;

typedef struct {
    uint16_t drive_enabled;
    uint16_t params_valid;
    uint16_t observer_locked;
    uint16_t observer_quality;
    uint32_t observer_fault_count;
    int32_t observer_theta_q24;
    int32_t observer_speed_rpm;
    uint16_t hfi_capable;
    uint16_t hfi_locked;
    uint16_t hfi_quality;
    uint16_t reserved;
    uint32_t hfi_fault_count;
    int32_t hfi_theta_q24;
    int32_t hfi_speed_rpm;
    uint16_t encoder_valid;
    int32_t encoder_theta_q24;
    int32_t encoder_speed_rpm;
    SL_PROBE_INPUT probe;
} SENSORLESS_STARTUP_INPUT;

typedef struct {
    uint16_t control_active;
    uint16_t angle_override;
    uint16_t current_override;
    uint16_t fault_request;
    uint16_t source;
    int32_t theta_e_q24;
    float id_ref_pu;
    float iq_ref_pu;
    float speed_ref_rpm;
    uint32_t fault_flags;
} SENSORLESS_STARTUP_OUTPUT;

typedef struct {
    SENSORLESS_STARTUP_CONFIG config;
    SENSORLESS_START_STATE state;
    SENSORLESS_STARTUP_OUTPUT output;
    uint16_t start_requested;
    int16_t direction;
    int32_t target_speed_rpm;
    uint32_t state_cycles;
    uint32_t total_start_cycles;
    uint32_t lock_good_cycles;
    uint32_t unlock_bad_cycles;
    uint32_t monitor_bad_cycles;
    uint32_t hfi_good_cycles;
    uint32_t hfi_bad_cycles;
    uint32_t encoder_good_cycles;
    uint32_t stop_good_cycles;
    uint32_t observer_fault_snapshot;
    uint32_t hfi_fault_snapshot;
    uint16_t blend_from_source;
    uint16_t reserved;
    float open_loop_theta_turns;
    float open_loop_speed_rpm;
    float run_speed_ref_rpm;
    float speed_integrator_pu;
    float blend_fraction;
    uint16_t speed_pi_pending, debug_completed;
    int32_t last_observer_theta_q24;
    SENSORLESS_START_FAILURE failure;
    SL_PROBE probe;
} SENSORLESS_STARTUP_RUNTIME;

void SensorlessStartup_DefaultConfig(SENSORLESS_STARTUP_CONFIG *config,
                                     float sample_time_s,
                                     float pole_pairs,
                                     float speed_base_rpm,
                                     float max_speed_rpm,
                                     float iq_limit_pu);
uint16_t SensorlessStartup_ValidateConfig(
    const SENSORLESS_STARTUP_CONFIG *config);
void SensorlessStartup_Init(SENSORLESS_STARTUP_RUNTIME *runtime,
                            const SENSORLESS_STARTUP_CONFIG *config);
uint16_t SensorlessStartup_RequestStart(SENSORLESS_STARTUP_RUNTIME *runtime,
                                        int16_t direction,
                                        int32_t target_speed_rpm);
uint16_t SensorlessStartup_UpdateTarget(SENSORLESS_STARTUP_RUNTIME *runtime,
                                        int16_t direction,
                                        int32_t target_speed_rpm);
void SensorlessStartup_Trip(SENSORLESS_STARTUP_RUNTIME *runtime, uint32_t fault);
void SensorlessStartup_RequestStop(SENSORLESS_STARTUP_RUNTIME *runtime);
void SensorlessStartup_ClearFault(SENSORLESS_STARTUP_RUNTIME *runtime);
void SensorlessStartup_Step(SENSORLESS_STARTUP_RUNTIME *runtime,
                            const SENSORLESS_STARTUP_INPUT *input);
uint16_t SensorlessStartup_IsIdle(const SENSORLESS_STARTUP_RUNTIME *runtime);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLESS_STARTUP_H_ */
