#ifndef SENSORLESS_ENCODER_MONITOR_H
#define SENSORLESS_ENCODER_MONITOR_H
#include <stdint.h>
#include <string.h>

/* Raw RT 0x6A observation only; never a control feedback source.
 * Counts/revolution follow the original RT decoder: motor 2^17, output 2^19.
 * sequence is odd while publishing; readers require the same even value twice. */
#define SL_ENCODER_TRACE_CAPACITY 512U
#define SL_ENCODER_TRACE_INTERVAL_MS 5U
/* Separate diagnostic publication; existing encoder ABI remains unchanged.
 * request_tick is the host request write; read_tick is FIFO service time.
 * Neither is an encoder-side physical latch or UART RX interrupt timestamp. */
typedef struct {
    uint32_t sequence, pending_request_tick, frame_request_tick;
    uint32_t frame_read_tick, frame_tick_ms;
} SL_ENCODER_TIMING;
extern volatile SL_ENCODER_TIMING g_sl_encoder_timing;
typedef struct {
    uint32_t tick_ms, motor_raw, output_raw, flags;
} SL_ENCODER_TRACE_POINT;

typedef struct {
    uint32_t abi, sequence, initialized, encoder_type;
    uint32_t tick_ms, requests, good_frames, bad_length, bad_crc, bad_status;
    uint32_t valid, reason, last_good_ms, age_ms, status, header;
    uint32_t motor_raw, output_raw, drive_active, record_id;
    uint32_t before_valid, after_valid, run_valid, before_ms, after_ms;
    uint32_t before_motor, before_output, after_motor, after_output;
    int32_t motor_delta, output_delta, motor_min_delta, motor_max_delta;
    int32_t output_min_delta, output_max_delta;
    uint32_t run_samples, run_errors;
    uint32_t trace_count, trace_overflow, trace_elapsed, trace_interval_valid;
    SL_ENCODER_TRACE_POINT trace[SL_ENCODER_TRACE_CAPACITY];
} SL_ENCODER_MONITOR;

static inline int32_t SlEncoder_Delta(uint32_t now, uint32_t before, uint32_t modulus)
{
    int32_t d = (int32_t)now - (int32_t)before;
    if (d > (int32_t)(modulus/2U)) d -= (int32_t)modulus;
    if (d < -(int32_t)(modulus/2U)) d += (int32_t)modulus;
    return d;
}

static inline void SlEncoder_Init(volatile SL_ENCODER_MONITOR *s, uint32_t type)
{
    memset((void *)s, 0, sizeof(*s));
    s->abi = 2U; s->encoder_type = type;
    s->reason = 1U; /* not initialized / unsupported */
}

/* One call per 1ms. Bad data invalidates immediately; old positions remain
 * diagnostic values only. reason: 0 valid, 1 disabled, 2 length, 3 CRC,
 * 4 encoder status, 5 range. No movement is needed to establish freshness. */
static inline void SlEncoder_Sample(volatile SL_ENCODER_MONITOR *s,
    const uint8_t *frame, uint32_t length, uint32_t drive_active)
{
    uint32_t i, motor=0, output=0, ok=0, old_valid=s->valid;
    uint8_t crc=0;
    s->sequence++;
    s->tick_ms++;
    drive_active = drive_active ? 1U : 0U;
    if (drive_active && !s->drive_active) {
        s->record_id++;
        s->before_valid=old_valid; s->run_valid=old_valid; s->after_valid=0U;
        s->before_motor=s->motor_raw; s->before_output=s->output_raw;
        s->before_ms=s->last_good_ms;
        s->motor_delta=s->output_delta=0;
        s->motor_min_delta=s->motor_max_delta=0;
        s->output_min_delta=s->output_max_delta=0;
        s->run_samples=s->run_errors=0U;
        s->trace_count=s->trace_overflow=s->trace_elapsed=0U;
        s->trace_interval_valid=old_valid;
    }
    if (length != 11U || !frame) {
        s->bad_length++; s->reason=2U;
    } else {
        s->header=frame[0]; s->status=frame[9];
        for(i=0;i<10U;i++) crc ^= frame[i];
        motor=(uint32_t)frame[1] | ((uint32_t)frame[2]<<8) | ((uint32_t)frame[3]<<16);
        output=(uint32_t)frame[4] | ((uint32_t)frame[5]<<8) | ((uint32_t)frame[6]<<16);
        if (crc != frame[10]) { s->bad_crc++; s->reason=3U; }
        else if (frame[9]) { s->bad_status++; s->reason=4U; }
        else if (motor >= 131072U || output >= 524288U) { s->reason=5U; }
        else { ok=1U; s->reason=0U; }
    }
    s->valid=ok;
    if(ok) {
        s->motor_raw=motor; s->output_raw=output;
        s->last_good_ms=s->tick_ms; s->good_frames++;
    }
    s->age_ms=s->tick_ms-s->last_good_ms;
    if(drive_active || s->drive_active) {
        if(!ok) { s->run_errors++; s->run_valid=0U; }
        else {
            s->run_samples++;
            s->after_motor=motor; s->after_output=output; s->after_ms=s->tick_ms;
            if(s->before_valid) {
                s->motor_delta=SlEncoder_Delta(motor,s->before_motor,131072U);
                s->output_delta=SlEncoder_Delta(output,s->before_output,524288U);
                if(s->motor_delta<s->motor_min_delta) s->motor_min_delta=s->motor_delta;
                if(s->motor_delta>s->motor_max_delta) s->motor_max_delta=s->motor_delta;
                if(s->output_delta<s->output_min_delta) s->output_min_delta=s->output_delta;
                if(s->output_delta>s->output_max_delta) s->output_max_delta=s->output_delta;
            }
        }
        if(!drive_active) s->after_valid=ok;
        s->trace_interval_valid &= ok;
        s->trace_elapsed++;
        if(s->trace_count==0U || !drive_active
            || s->trace_elapsed>=SL_ENCODER_TRACE_INTERVAL_MS) {
            /* Reserve the final slot for the falling-edge sample. Never wrap:
             * a full buffer must report incomplete coverage, not hide history. */
            if(s->trace_count<SL_ENCODER_TRACE_CAPACITY-1U
                || (!drive_active && s->trace_count<SL_ENCODER_TRACE_CAPACITY)) {
                uint32_t n=s->trace_count;
                s->trace[n].tick_ms=s->tick_ms;
                s->trace[n].motor_raw=s->motor_raw;
                s->trace[n].output_raw=s->output_raw;
                s->trace[n].flags=ok | (s->trace_interval_valid<<1)
                    | (drive_active<<2);
                s->trace_count=n+1U;
            } else s->trace_overflow=1U;
            s->trace_elapsed=0U;
            s->trace_interval_valid=1U;
        }
    }
    s->drive_active=drive_active;
    s->sequence++;
}
#endif
