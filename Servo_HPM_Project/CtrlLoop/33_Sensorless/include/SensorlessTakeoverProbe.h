#ifndef SENSORLESS_TAKEOVER_PROBE_H_
#define SENSORLESS_TAKEOVER_PROBE_H_
#include <stdint.h>
/* A1/A2/A3 only. Fixed 16 kHz / 10 pole-pair experiment, never a speed controller. */
enum { SL_P_QUALIFY, SL_P_RISE, SL_P_HOLD, SL_P_RETURN,
       SL_P_DONE, SL_P_SKIP, SL_P_RELEASE, SL_P_OFF };
enum { SL_P_BASE=1, SL_P_EMF=2, SL_P_PHASE=4, SL_P_LOCK=8,
       SL_P_CHAIN=16, SL_P_ENCODER=32, SL_P_SPEED=64, SL_P_ANGLE=128,
       SL_P_CURRENT=256, SL_P_TRACK=512, SL_P_SAT=1024,
       SL_P_FOLLOW=2048, SL_P_STOP=4096, SL_P_NUMERIC=8192,
       SL_P_RETURN_TIMEOUT=16384 };
typedef struct {
    uint32_t tick, encoder_raw, encoder_read_tick, encoder_request_tick;
    uint32_t encoder_valid, chain_valid, saturated, locked;
    float emf_a, emf_b, emf, phase, smo_rpm;
    float id, iq, id_ref, iq_ref, ud, uq;
    int32_t smo_theta_q24, actual_theta_q24;
} SL_PROBE_INPUT;
/* Diagnostics only: reference is the completed angle visible at PreFoc for
 * the named host PWM counter, NOT the encoder's unknown physical latch. */
enum { SL_PE_NONE, SL_PE_LOW_CURRENT, SL_PE_DEADLINE, SL_PE_HARD_FAULT,
       SL_PE_EXTERNAL_OFF };
typedef struct {
    uint32_t valid, tick, encoder_raw, read_tick, request_tick;
    uint32_t reference_valid, reference_tick;
    int32_t reference_theta_q24, forced_theta_q24;
    float forced_rpm, follow;
} SL_PROBE_FOLLOW_SNAPSHOT;
typedef struct {
    uint32_t abi, sequence, first_valid, first_tick, first_phase, first_reason;
    uint32_t return_timeout_valid, return_timeout_tick;
    uint32_t end_reason, end_tick, end_cycles, end_low_current, hard_detail;
    int32_t forced_theta_q24;
    float follow_zero, offset, ratio;
    uint32_t bad[7];
    SL_PROBE_INPUT input;
    SL_PROBE_FOLLOW_SNAPSHOT follow, entry_follow;
} SL_PROBE_EXIT_DIAG;

typedef struct {
    uint32_t phase, cycles, good, max_good, gates, failed_ever, reason, stop;
    uint32_t a2, a3, done_tick; /* Explicit profiles; terminal tick anchors extended capture. */
    uint32_t entry_tick, exit_tick, baseline_count, baseline_valid, baseline_done;
    uint32_t baseline_first_tick, baseline_last_tick, baseline_raw, baseline_last_read;
    int32_t baseline_min, baseline_max;
    float sum_a, sum_b, sum_e2, b0, sigma, entry_emf, exit_emf;
    uint32_t sample_div, ring_index, ring_count, encoder_ticks[11], encoder_raws[11];
    float speeds[10], follows[10], speed_sum, follow_sum, smo_mean, enc_rpm;
    float follow, follow_zero, delta, previous_delta, offset, ratio, last_speed;
    float release_theta, release_id, release_iq, last_id, last_iq;
    uint32_t delta_valid, filter_valid, low_current, bad[7];
    uint32_t gate_counts[11];
    float max_current2, max_tracking2, max_phase, min_emf, max_speed_error, max_offset;
    uint32_t reference_ticks[64], reference_written[2];
    int32_t reference_angles[64];
    SL_PROBE_FOLLOW_SNAPSHOT follow_snapshot;
    SL_PROBE_EXIT_DIAG exit_diag;
    /* Control state only; no additional ISR record or metric chain. */
    uint32_t frame, frame_committed, frame_tick;
    int32_t previous_smo;
    float frame_id, frame_iq;
} SL_PROBE;
/* Return 1: PWM off; return 2: invalid current/control, immediate hard trip. */
uint32_t SlProbe_Step(SL_PROBE *p, const SL_PROBE_INPUT *x,
    uint32_t align, uint32_t stage_cycles, int16_t direction,
    int32_t forced_q24, float forced_rpm, int32_t *theta, float *id, float *iq);

/* Terminal metadata only; never changes control or the first exit snapshot. */
void SlProbe_EndExternal(SL_PROBE *p,uint32_t tick,uint32_t hard_detail);

/* Separate whole-attempt diagnostics; trace storage remains 43160 bytes.
 * Buckets 0..7 are post-PreFoc probe phases; bucket 8 is fixed alignment. */
typedef struct {
    uint32_t magic, abi, sequence, active, record_id;
    uint32_t samples[9], saturated[9], max_run[9];
    uint32_t run, bucket_run, total_saturated, max_run_any, previous_bucket, last_tick, gap_count;
    uint32_t first_saturated_tick, last_saturated_tick, failed_samples, total_samples;
} SL_PROBE_DIAGNOSTICS;
extern volatile SL_PROBE_DIAGNOSTICS g_probe_diagnostics;
void SlProbeDiagnostics_Begin(volatile SL_PROBE_DIAGNOSTICS *d, uint32_t record);
void SlProbeDiagnostics_Record(volatile SL_PROBE_DIAGNOSTICS *d,
    uint32_t bucket, uint32_t tick, uint32_t saturated, uint32_t failed);
void SlProbeDiagnostics_Finish(volatile SL_PROBE_DIAGNOSTICS *d);

#define SL_PROBE_LONG_N 384U
#define SL_PROBE_FAST_N 96U
typedef struct {
    uint32_t tick, phase, gates, reason;
    int32_t forced, smo, applied;
    uint32_t encoder_raw, encoder_read_tick, encoder_request_tick;
    float id_ref, iq_ref, id, iq, ud, uq, emf, phase_error;
    float smo_rpm, encoder_rpm, offset, follow;
    uint32_t flags, stage_cycles;
} SL_PROBE_LONG; /* ABI4: 96 bytes */
typedef struct {
    uint32_t tick, phase, gates, reason;
    int32_t applied, forced;
    float id_ref, iq_ref, id, iq, ud, uq;
    int32_t raw_u, raw_v;
    uint32_t flags;
    float offset;
} SL_PROBE_FAST; /* 64 bytes */
typedef struct {
    uint32_t magic, abi, active, frozen, sequence, record_id;
    uint32_t long_write, long_count, fast_write, fast_count;
    uint32_t trigger_tick, fast_trigger_tick, fast_frozen, finish_reason;
    uint32_t divider, baseline_valid, baseline_count, max_good;
    float b0, sigma, entry_emf, exit_emf;
    uint32_t gate_counts[11];
    float max_current2, max_tracking2, max_phase, min_emf, max_speed_error;
} SL_PROBE_HEADER; /* 152 bytes */
typedef struct {
    SL_PROBE_HEADER h;
    SL_PROBE_LONG slow[SL_PROBE_LONG_N];
    SL_PROBE_FAST fast[SL_PROBE_FAST_N];
} SL_PROBE_TRACE;
typedef char sl_probe_long_size[sizeof(SL_PROBE_LONG)==96?1:-1];
typedef char sl_probe_fast_size[sizeof(SL_PROBE_FAST)==64?1:-1];
typedef char sl_probe_header_size[sizeof(SL_PROBE_HEADER)==152?1:-1];
/* Pure scheduling query: does not advance the divider or change capture epochs. */
static inline uint32_t SlProbeTrace_LongDue(const volatile SL_PROBE_TRACE *t,
    const SL_PROBE *p,uint32_t tick)
{
    uint32_t extended=p->a2 || p->a3 || p->frame;
    return t->h.active && (t->h.divider+1U>=((p->a3 || p->frame)?80U:(p->a2?48U:32U))
        || (extended && p->done_tick
            && (tick==p->done_tick || tick-p->done_tick>=1920U)));
}
void SlProbeTrace_Begin(volatile SL_PROBE_TRACE *t, uint32_t record);
void SlProbeTrace_Finish(volatile SL_PROBE_TRACE *t, const SL_PROBE *p, uint32_t reason);
void SlProbeTrace_Record(volatile SL_PROBE_TRACE *t, const SL_PROBE *p,
    const SL_PROBE_LONG *s, int32_t raw_u, int32_t raw_v);
#endif
