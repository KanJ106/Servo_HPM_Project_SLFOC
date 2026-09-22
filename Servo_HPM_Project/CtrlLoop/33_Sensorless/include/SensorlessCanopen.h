#ifndef SENSORLESS_CANOPEN_H_
#define SENSORLESS_CANOPEN_H_
#include <stdint.h>
/* Independent RAM mailbox. Publish request_seq last; source does not auto-run. */
#define SL_JLINK_ABI 1U
#define SL_JLINK_KEY 0x4A4C494EU
#define SL_JLINK_MAX_MS 10000U
typedef struct {
    uint32_t abi, request_seq, acknowledge_seq, command, arm_key, duration_ms;
    int32_t direction, target_speed_rpm;
    uint32_t result, owner, enable_requested, elapsed_ms, exit_reason;
} SENSORLESS_JLINK_MAILBOX;
extern volatile SENSORLESS_JLINK_MAILBOX g_sensorless_jlink;
/* Commands: 1=one bounded ALIGN/IF run, 2=abort, 3=release while OFF.
 * Results: 0=accepted, 1=key/ABI, 2=busy, 3=parameters, 4=not OFF.
 * Exit: 1=deadline, 2=user abort, 3=start rejected, 4=fault, 5=completed.
 * Ownership stays latched after completion; no fallback to CAN until release. */
uint16_t SensorlessJlink_OwnsControl(void);
uint16_t SensorlessJlink_EnableRequested(void);
uint16_t SensorlessJlink_PlatformReady(void);
uint16_t SensorlessJlink_PlatformOff(void);
/* Passive sampling evidence: 0=OFF, 1=charge, 2=offset calibration, 3=enabled.
 * Counters accumulate from boot. Read with unchanged even sequence.
 * Rail values in OFF do not constitute a motion-readiness judgement. */
typedef struct {
    uint32_t samples, rail_samples;
    int32_t min_u, max_u, min_v, max_v, last_u, last_v, zero_u, zero_v;
} SL_SAMPLING_PHASE;
typedef struct {
    uint32_t abi, sequence, last_phase, reserved;
    SL_SAMPLING_PHASE phase[4];
} SL_SAMPLING_DIAG;
extern volatile SL_SAMPLING_DIAG g_sl_sampling;
void SensorlessSampling_Record(uint16_t phase, int16_t u, int16_t v,
                               int16_t zero_u, int16_t zero_v);
#define SL_CAN_PROTOCOL 1U
#define SL_CAN_WATCHDOG_MS 1000U
#define SL_CAN_OD_FIRST 0x2F00U
#define SL_CAN_OD_COUNT 8U
void SensorlessCanopen_Init(void);
void SensorlessCanopen_Tick1ms(void);
void SensorlessCanopen_Service1ms(void);
void SensorlessCanopen_PwmGuard(void);
uint16_t SensorlessCanopen_MotionAllowed(void);
/* Platform callback: NMT operational, CiA402 enabled, no halt, ready power stage. */
uint16_t SensorlessCanopen_PlatformReady(void);
/* Scalar wire API. All nonzero subindices are exactly 32 bits, LE. */
uint16_t SensorlessCanopen_Read(uint16_t index, uint8_t sub, uint32_t *value);
uint16_t SensorlessCanopen_Write(uint16_t index, uint8_t sub, uint32_t value);
uint16_t SensorlessCanopen_FieldCount(uint16_t index);
uint16_t SensorlessCanopen_TraceRead(uint32_t offset, void *dst, uint32_t bytes);
extern volatile uint32_t g_sl_can_elapsed_ms, g_sl_can_abort_reason;
#endif

