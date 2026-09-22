#ifndef SENSORLESS_ALIGN_TRACE_H_
#define SENSORLESS_ALIGN_TRACE_H_
#include <stdint.h>

/* ALIGN-only first 8 ms + last 26 ms at 16 kHz. Fits the existing SMO buffer.
 * Every sample is after the current-loop decision and PWM update. On a trip,
 * PI outputs are reset to zero; the preceding sample retains prior outputs. */
#define SL_ALIGN_FIRST 128U
#define SL_ALIGN_TAIL 416U
#define SL_ALIGN_CAPACITY (SL_ALIGN_FIRST + SL_ALIGN_TAIL)
typedef struct {
    uint32_t tick, stage_cycles;
    float id_ref, iq_ref, id_fb, iq_fb;
    int32_t sdm_u, sdm_v, zero_u, zero_v;
    float voltage_d, voltage_q, voltage_limit;
    uint16_t saturated, current_failure;
    uint32_t flags; /* bit0 SDM input, bit1 PI executed; bits8..15 sample window */
} SL_ALIGN_SAMPLE;
typedef struct {
    uint32_t magic, abi, sample_bytes, capacity, sequence, active, frozen, record_id;
    uint32_t total_samples, first_count, tail_count, tail_next, stop_reason, sample_hz;
    uint32_t first_tick, last_tick;
    float current_base_a, id_command, current_limit, measured_trip;
    float kp_d, kp_q, ki_step, align_angle_turns;
} SL_ALIGN_HEADER;
typedef struct {
    SL_ALIGN_HEADER header;
    SL_ALIGN_SAMPLE samples[SL_ALIGN_CAPACITY];
} SL_ALIGN_TRACE;
typedef char sl_align_sample_size[(sizeof(SL_ALIGN_SAMPLE)==60)?1:-1];
typedef char sl_align_header_size[(sizeof(SL_ALIGN_HEADER)==96)?1:-1];
static inline void SlAlignTrace_Begin(volatile SL_ALIGN_TRACE *t,
    SL_ALIGN_HEADER h, uint32_t record_id)
{
    h.magic=0x414C4731UL; h.abi=1; h.sample_bytes=sizeof(SL_ALIGN_SAMPLE);
    h.capacity=SL_ALIGN_CAPACITY; h.sequence=1; h.active=1;
    h.record_id=record_id;
    t->header=h; t->header.sequence=2;
}
static inline void SlAlignTrace_Finish(volatile SL_ALIGN_TRACE *t,uint32_t reason)
{
    if(!t->header.active)return;
    t->header.sequence++;
    t->header.stop_reason=reason;t->header.active=0;t->header.frozen=1;
    t->header.sequence++;
}
static inline void SlAlignTrace_Sample(volatile SL_ALIGN_TRACE *t,
    const SL_ALIGN_SAMPLE *s)
{
    uint32_t slot;
    if(!t->header.active)return;
    t->header.sequence++;
    if(!t->header.total_samples)t->header.first_tick=s->tick;
    if(t->header.first_count<SL_ALIGN_FIRST)slot=t->header.first_count++;
    else {
        slot=SL_ALIGN_FIRST+t->header.tail_next;
        if(++t->header.tail_next==SL_ALIGN_TAIL)t->header.tail_next=0;
        if(t->header.tail_count<SL_ALIGN_TAIL)t->header.tail_count++;
    }
    t->samples[slot]=*s;
    t->header.total_samples++;t->header.last_tick=s->tick;
    t->header.sequence++;
}
#endif
