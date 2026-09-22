#ifndef SENSORLESS_SMO_TRACE_H_
#define SENSORLESS_SMO_TRACE_H_
#include <stdint.h>
#include "SmoPll.h"
#include "SensorlessStartup.h"

/* ABI 2: 318 ms observer window + 126 ms fixed-axis tail at 16 kHz.
 * Decimation affects storage only; whole-attempt lock statistics stay at PWM rate.
 * Encoder timestamps bracket host request/service read, NOT physical latching. */
#define SL_SMO_TRACE_CAPACITY 160U
#define SL_SMO_ALIGN_CAPACITY 64U
#define SL_SMO_TRACE_DIVIDER 32U
typedef struct {
    uint32_t tick, stage_cycles, state;
    int32_t direction;
    uint32_t chain_tick, chain_valid, sample_window, pwm_u, pwm_v, pwm_w, pwm_period;
    uint32_t encoder_sequence, encoder_raw, encoder_tick_ms;
    uint32_t encoder_request_tick, encoder_read_tick, encoder_valid;
    int32_t forced_theta_q24, control_theta_q24;
    float id_ref, iq_ref, id_fb, iq_fb, ud, uq;
    uint32_t saturated, current_failure;
} SL_SMO_TRACE_CONTEXT;
typedef struct {
    SL_SMO_TRACE_CONTEXT context;
    uint32_t locked, lock_reason, lock_counter, unlock_counter;
    float voltage_alpha, voltage_beta, current_alpha, current_beta;
    float current_hat_alpha, current_hat_beta, switching_alpha, switching_beta;
    float emf_alpha, emf_beta, theta_raw, theta_pll, omega_tracking, omega_filtered;
    float phase_error_rad, emf_magnitude, phase_advance;
} SL_SMO_TRACE_SAMPLE;
typedef struct {
    uint32_t magic, abi, sample_bytes, capacity, sequence, count, frozen, active;
    uint32_t record_id, trigger_stage_cycles, stop_reason, total_samples;
    uint32_t locked_samples, longest_locked_run, locked_run, first_tick, last_tick;
    uint32_t sample_divider, align_count, align_trigger_cycles;
} SL_SMO_TRACE_HEADER;
typedef struct {
    SL_SMO_TRACE_HEADER header;
    SMO_PLL_PARAMS params;
    SL_SMO_TRACE_SAMPLE samples[SL_SMO_TRACE_CAPACITY];
    SL_SMO_TRACE_SAMPLE align_samples[SL_SMO_ALIGN_CAPACITY];
} SL_SMO_TRACE;
typedef char sl_smo_sample_size_check[(sizeof(SL_SMO_TRACE_SAMPLE)==192)?1:-1];
typedef char sl_smo_header_size_check[(sizeof(SL_SMO_TRACE_HEADER)==80)?1:-1];
typedef char sl_smo_params_size_check[(sizeof(SMO_PLL_PARAMS)==72)?1:-1];

static inline void SlSmoTrace_Begin(volatile SL_SMO_TRACE *t,
    const SMO_PLL_PARAMS *p, uint16_t enabled, uint32_t trigger)
{
    SL_SMO_TRACE_HEADER h={0};
    h.record_id=t->header.record_id+1U;
    t->header.sequence++;
    h.sequence=t->header.sequence;
    h.magic=0x534D5431UL; h.abi=2; h.sample_bytes=sizeof(SL_SMO_TRACE_SAMPLE);
    h.capacity=SL_SMO_TRACE_CAPACITY; h.active=enabled?1U:0U;
    h.frozen=enabled?0U:1U; h.trigger_stage_cycles=trigger;
    h.sample_divider=SL_SMO_TRACE_DIVIDER;
    h.align_trigger_cycles=0xffffffffUL; /* set from accepted ALIGN duration */
    t->header=h; t->params=*p;
    t->header.sequence++;
}
static inline void SlSmoTrace_Finish(volatile SL_SMO_TRACE *t,uint32_t reason)
{
    if(!t->header.active)return;
    t->header.sequence++;
    t->header.active=0; t->header.frozen=1; t->header.stop_reason=reason;
    t->header.sequence++;
}
static inline uint32_t SlSmoTrace_WantsSample(const volatile SL_SMO_TRACE *t,
    uint32_t state, uint32_t cycles)
{
    if (!t->header.active) return 0U;
    if (state==SENSORLESS_START_ALIGN)
        return t->header.align_count<SL_SMO_ALIGN_CAPACITY
            && cycles>=t->header.align_trigger_cycles
            && (cycles-t->header.align_trigger_cycles)%SL_SMO_TRACE_DIVIDER==0U;
    return !t->header.frozen && state==SENSORLESS_START_OPEN_LOOP_RAMP
        && cycles>=t->header.trigger_stage_cycles
        && (cycles-t->header.trigger_stage_cycles)%SL_SMO_TRACE_DIVIDER==0U;
}
static inline void SlSmoTrace_Sample(volatile SL_SMO_TRACE *t,
    const SMO_PLL_INPUT *in,const SMO_PLL_STATE *s,const SL_SMO_TRACE_CONTEXT *c)
{
    if(!t->header.active)return;
    t->header.sequence++;
    t->header.total_samples++;
    if(s->locked) {
        t->header.locked_samples++; t->header.locked_run++;
        if(t->header.locked_run>t->header.longest_locked_run)
            t->header.longest_locked_run=t->header.locked_run;
    } else t->header.locked_run=0;
    if(SlSmoTrace_WantsSample(t,c->state,c->stage_cycles)) {
        SL_SMO_TRACE_SAMPLE v;
        v.context=*c;
        v.locked=s->locked;v.lock_reason=s->lock_reason;
        v.lock_counter=s->lock_counter;v.unlock_counter=s->unlock_counter;
        v.voltage_alpha=in->voltage_alpha_pu;v.voltage_beta=in->voltage_beta_pu;
        v.current_alpha=in->current_alpha_pu;v.current_beta=in->current_beta_pu;
        v.current_hat_alpha=s->current_hat_alpha_pu;v.current_hat_beta=s->current_hat_beta_pu;
        v.switching_alpha=s->switching_alpha_pu;v.switching_beta=s->switching_beta_pu;
        v.emf_alpha=s->emf_alpha_pu;v.emf_beta=s->emf_beta_pu;
        v.theta_raw=s->theta_raw_turns;v.theta_pll=s->theta_pll_turns;
        v.omega_tracking=s->omega_tracking_e_rad_s;v.omega_filtered=s->omega_e_rad_s;
        v.phase_error_rad=s->phase_error_rad;v.emf_magnitude=s->emf_magnitude_pu;
        v.phase_advance=s->phase_advance_turns;
        if(c->state==SENSORLESS_START_ALIGN) {
            t->align_samples[t->header.align_count++]=v;
        } else {
            if(!t->header.count)t->header.first_tick=c->tick;
            t->samples[t->header.count]=v;
            t->header.last_tick=c->tick;
            if(++t->header.count==SL_SMO_TRACE_CAPACITY)t->header.frozen=1;
        }
    }
    t->header.sequence++;
}
#endif
