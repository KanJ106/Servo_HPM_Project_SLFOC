#include "SensorlessTakeoverProbe.h"
#include <math.h>
#include <string.h>
#ifdef _MSC_VER
#include <intrin.h>
#endif
static void diag_barrier(void)
{
#ifdef __riscv
    __asm__ volatile("fence rw,rw" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#else
    __asm__ volatile("" ::: "memory");
#endif
}
static void diag_begin(volatile SL_PROBE_EXIT_DIAG *d)
{ ++d->sequence;diag_barrier();d->abi=1U; }
static void diag_end(volatile SL_PROBE_EXIT_DIAG *d)
{ diag_barrier();++d->sequence; }

static float wrap(float t) { return t-floorf(t); }
static float diff(float t) { return t-floorf(t+0.5f); }
static float sq(float v) { return v*v; }
static float maxf(float a,float b) { return a>b?a:b; }
static int32_t rawdiff(uint32_t a,uint32_t b)
{ int32_t d=(int32_t)a-(int32_t)b; if(d>65536)d-=131072; if(d< -65536)d+=131072; return d; }
static uint32_t dwell(uint32_t *n,uint32_t bad,uint32_t limit)
{ *n=bad?(*n<limit?*n+1U:limit):0U; return *n>=limit; }
static float ease(float u) { return u*u*(3.0f-2.0f*u); }
static uint32_t finite_input(const SL_PROBE_INPUT *x)
{
    return isfinite(x->emf_a)&&isfinite(x->emf_b)&&isfinite(x->emf)
        &&isfinite(x->phase)&&isfinite(x->smo_rpm)&&isfinite(x->ud)&&isfinite(x->uq);
}

static void exit_capture(SL_PROBE *p,const SL_PROBE_INPUT *x,
    uint32_t reason,int32_t forced)
{
    volatile SL_PROBE_EXIT_DIAG *d=&p->exit_diag;uint32_t i;
    if(d->first_valid)return;
    diag_begin(d);
    d->first_tick=x->tick;d->first_phase=p->phase;d->first_reason=reason;
    d->forced_theta_q24=forced;d->follow_zero=p->follow_zero;
    d->offset=p->offset;d->ratio=p->ratio;d->input=*x;
    d->follow=p->follow_snapshot;
    for(i=0;i<7U;++i)d->bad[i]=p->bad[i];
    d->first_valid=1U;diag_end(d);
}
static void exit_end(SL_PROBE *p,uint32_t tick,uint32_t reason,uint32_t detail)
{
    volatile SL_PROBE_EXIT_DIAG *d=&p->exit_diag;
    if(d->end_reason)return;
    diag_begin(d);d->end_tick=tick;d->end_reason=reason;
    d->end_cycles=p->cycles;d->end_low_current=p->low_current;d->hard_detail=detail;
    diag_end(d);
}
void SlProbe_EndExternal(SL_PROBE *p,uint32_t tick,uint32_t hard_detail)
{
    exit_end(p,tick,hard_detail?SL_PE_HARD_FAULT:SL_PE_EXTERNAL_OFF,hard_detail);
}


static void baseline(SL_PROBE *p,const SL_PROBE_INPUT *x,uint32_t n)
{
    int32_t d;
    if(n<5952U || n>7968U || (n&31U))return;
    if(!x->encoder_valid || !x->chain_valid || x->saturated || !finite_input(x)
        || x->encoder_read_tick==p->baseline_last_read
        || fabsf(x->id_ref-0.03125f)>0.000001f || fabsf(x->iq_ref)>0.000001f
        || sq(x->id-x->id_ref)+sq(x->iq-x->iq_ref)>sq(0.003125f))return;
    if(!p->baseline_count) {
        p->baseline_first_tick=x->tick; p->baseline_raw=x->encoder_raw;
    }
    d=rawdiff(x->encoder_raw,p->baseline_raw);
    if(d<p->baseline_min)p->baseline_min=d;
    if(d>p->baseline_max)p->baseline_max=d;
    p->baseline_last_tick=x->tick; p->baseline_last_read=x->encoder_read_tick;
    p->sum_a+=x->emf_a; p->sum_b+=x->emf_b;
    p->sum_e2+=sq(x->emf_a)+sq(x->emf_b); ++p->baseline_count;
}
static void finish_baseline(SL_PROBE *p)
{
    p->baseline_done=1U; p->entry_emf=0.010f; p->exit_emf=0.009f;
    if(p->baseline_count!=64U || p->baseline_last_tick-p->baseline_first_tick<2016U
       || p->baseline_max-p->baseline_min>18)return;
    p->b0=sqrtf(sq(p->sum_a/64.0f)+sq(p->sum_b/64.0f));
    p->sigma=sqrtf(maxf(0.0f,p->sum_e2/64.0f-sq(p->b0)));
    p->baseline_valid=isfinite(p->b0)&&isfinite(p->sigma);
    p->entry_emf=maxf(0.010f,maxf(2.0f*p->b0,6.0f*p->sigma));
    p->exit_emf=maxf(0.009f,maxf(1.5f*p->b0,4.0f*p->sigma));
}
/* Bounded 2 ms sampling, ten intervals = 20 ms. No busy-wait on encoder. */
static void filter(SL_PROBE *p,const SL_PROBE_INPUT *x,float forced,float rpm)
{
    uint32_t k,old,dt;
    float f;
    if(++p->sample_div<32U)return;
    p->sample_div=0U;
    p->follow_snapshot.valid=0U;p->follow_snapshot.reference_valid=0U;
    p->follow_snapshot.tick=x->tick;p->follow_snapshot.encoder_raw=x->encoder_raw;
    p->follow_snapshot.read_tick=x->encoder_read_tick;
    p->follow_snapshot.request_tick=x->encoder_request_tick;
    p->follow_snapshot.forced_theta_q24=(int32_t)(forced*16777216.0f);
    p->follow_snapshot.forced_rpm=rpm;
    k=x->encoder_read_tick&63U;
    if(x->encoder_valid && x->tick-x->encoder_read_tick<=32U
        && (p->reference_written[k>>5]&(1UL<<(k&31U)))
        && p->reference_ticks[k]==x->encoder_read_tick) {
        p->follow_snapshot.reference_tick=p->reference_ticks[k];
        p->follow_snapshot.reference_theta_q24=p->reference_angles[k];
        p->follow_snapshot.reference_valid=1U;
    }
    if(!x->encoder_valid || !isfinite(x->smo_rpm)) {
        p->ring_count=0; p->ring_index=0; p->filter_valid=0;
        p->speed_sum=p->follow_sum=0.0f; return;
    }
    k=p->ring_index;
    p->encoder_ticks[k]=x->encoder_read_tick; p->encoder_raws[k]=x->encoder_raw;
    /* Project carrier back to host read time, not an invented physical latch. */
    f=diff((float)x->encoder_raw*(10.0f/131072.0f)-forced
        +rpm*(10.0f/(60.0f*16000.0f))*(float)(x->tick-x->encoder_read_tick));
    if(p->ring_count) f=p->follow+diff(f-p->follow);
    p->follow=f;
    p->follow_snapshot.follow=f;p->follow_snapshot.valid=1U;
    old=p->ring_count%10U;
    if(p->ring_count>=10U) {
        p->speed_sum-=p->speeds[old]; p->follow_sum-=p->follows[old];
    }
    p->speeds[old]=x->smo_rpm; p->follows[old]=f;
    p->speed_sum+=x->smo_rpm; p->follow_sum+=f;
    if(p->ring_count>=10U) {
        old=(k+1U)%11U; dt=x->encoder_read_tick-p->encoder_ticks[old];
        p->filter_valid=dt>=288U && dt<=352U;
        if(p->filter_valid) {
            p->enc_rpm=(float)rawdiff(x->encoder_raw,p->encoder_raws[old])
                *(60.0f*16000.0f/131072.0f)/(float)dt;
            p->smo_mean=p->speed_sum*0.1f;
        }
    }
    ++p->ring_count; /* < 320,000 / 32 per attempt, no overflow */
    p->ring_index=(k+1U)%11U;
}
uint32_t SlProbe_Step(SL_PROBE *p,const SL_PROBE_INPUT *x,
    uint32_t align,uint32_t n,int16_t direction,int32_t forced_q24,
    float rpm,int32_t *theta,float *id,float *iq)
{
    float forced=(float)forced_q24/16777216.0f, current2,tracking2,delta,jump;
    float speed_error,goal,step,ratio=0.0f,scale;
    uint32_t gates=0U,reason=0U,i,active;
    /* No control reads this ring. It holds the completed angle observed at
     * PreFoc, indexed by the same host counter used by encoder publication. */
    i=x->tick&63U;p->reference_ticks[i]=x->tick;
    p->reference_angles[i]=x->actual_theta_q24;
    p->reference_written[i>>5]|=1UL<<(i&31U);
    if(!isfinite(x->id)||!isfinite(x->iq)||!isfinite(*id)||!isfinite(*iq)
        ||!isfinite(rpm)||!isfinite(p->offset)||!isfinite(x->ud)||!isfinite(x->uq)) {
        exit_capture(p,x,SL_P_NUMERIC,forced_q24);
        exit_end(p,x->tick,SL_PE_HARD_FAULT,SL_P_NUMERIC);return 2U;
    }
    current2=sq(x->id)+sq(x->iq);
    tracking2=sq(x->id-x->id_ref)+sq(x->iq-x->iq_ref);
    if(!isfinite(current2)||!isfinite(tracking2)) {
        exit_capture(p,x,SL_P_NUMERIC,forced_q24);
        exit_end(p,x->tick,SL_PE_HARD_FAULT,SL_P_NUMERIC);return 2U;
    }
    if(align && !p->stop) { baseline(p,x,n); goto remember; }
    if(!p->baseline_done)finish_baseline(p);
    if(p->phase==SL_P_OFF)return 1U;
    if(p->phase==SL_P_DONE || p->phase==SL_P_SKIP) {
        if(!p->stop) { filter(p,x,forced,rpm); goto remember; }
    }
    if(p->phase==SL_P_RELEASE) { filter(p,x,forced,rpm); goto release; }
    /* Warm the 20 ms filter just before qualification; no early-ramp gate work. */
    if(p->phase==SL_P_QUALIFY && n<31648U && !p->stop)goto remember;
    filter(p,x,forced,rpm);
    delta=diff((float)x->smo_theta_q24/16777216.0f-forced);
    jump=p->delta_valid?fabsf(diff(delta-p->previous_delta)):0.0f;
    /* A basis change is not a jump of the observer itself. Keep both guards. */
    if(p->frame) {
        if(x->smo_theta_q24<0 || x->smo_theta_q24>=16777216)gates|=SL_P_NUMERIC;
        if(p->delta_valid && fabsf(diff((float)(x->smo_theta_q24-p->previous_smo)
            /16777216.0f))>5.0f/360.0f)gates|=SL_P_ANGLE;
        p->previous_smo=x->smo_theta_q24;
    }
    p->delta=delta; p->previous_delta=delta; p->delta_valid=1U;
    speed_error=maxf(fabsf(p->smo_mean-rpm),maxf(fabsf(p->enc_rpm-rpm),
        fabsf(p->smo_mean-p->enc_rpm)));
    if(!p->baseline_valid)gates|=SL_P_BASE;
    if(!finite_input(x))gates|=SL_P_NUMERIC;
    if(x->emf<p->entry_emf)gates|=SL_P_EMF;
    if(fabsf(x->phase)>0.10f)gates|=SL_P_PHASE;
    if(!x->locked)gates|=SL_P_LOCK;
    if(!x->chain_valid)gates|=SL_P_CHAIN;
    if(!x->encoder_valid)gates|=SL_P_ENCODER;
    if(!p->filter_valid || speed_error>10.0f || fabsf(rpm)<90.0f || fabsf(rpm)>110.0f
        || x->smo_rpm*direction<=0.0f || p->enc_rpm*direction<=0.0f
        || rpm*direction<=0.0f)gates|=SL_P_SPEED;
    if(fabsf(delta)>60.0f/360.0f || jump>5.0f/360.0f)gates|=SL_P_ANGLE;
    if(current2>sq(0.03515625f))gates|=SL_P_CURRENT;
    if(tracking2>sq(0.003125f))gates|=SL_P_TRACK;
    if(x->saturated)gates|=SL_P_SAT;
    p->gates=gates;
    active=p->phase>=SL_P_RISE && p->phase<=SL_P_RETURN;
    if(n>32000U && n<=48000U && p->phase==SL_P_QUALIFY) {
        p->failed_ever|=gates;
        for(i=0;i<11U;++i)if(gates&(1UL<<i))++p->gate_counts[i];
        p->good=gates?0U:p->good+1U;
        if(p->good>p->max_good)p->max_good=p->good;
        if(p->good>=3840U) {
            p->phase=SL_P_RISE; p->cycles=0; p->entry_tick=x->tick;
            p->follow_zero=p->follow_sum*0.1f; p->min_emf=x->emf; active=1U;
            diag_begin(&p->exit_diag);
            p->exit_diag.entry_follow=p->follow_snapshot;diag_end(&p->exit_diag);
        }
    }
    if(n>=48000U && p->phase==SL_P_QUALIFY) {p->phase=SL_P_SKIP;p->done_tick=x->tick;}
    if(active) {
        p->max_current2=maxf(p->max_current2,current2);
        p->max_tracking2=maxf(p->max_tracking2,tracking2);
        p->max_phase=maxf(p->max_phase,fabsf(x->phase));
        if(x->emf<p->min_emf)p->min_emf=x->emf;
        p->max_speed_error=maxf(p->max_speed_error,speed_error);
        reason=gates&(SL_P_LOCK|SL_P_CHAIN|SL_P_ENCODER|SL_P_ANGLE|SL_P_NUMERIC);
        if(x->smo_rpm*direction<=0.0f || p->enc_rpm*direction<=0.0f
            || rpm*direction<=0.0f)reason|=SL_P_SPEED;
        if(dwell(&p->bad[0],current2>sq(0.03515625f),32U))reason|=SL_P_CURRENT;
        if(dwell(&p->bad[1],tracking2>sq(0.00625f),32U))reason|=SL_P_TRACK;
        if(dwell(&p->bad[2],x->saturated,32U))reason|=SL_P_SAT;
        if(dwell(&p->bad[3],fabsf(x->phase)>0.15f,32U))reason|=SL_P_PHASE;
        if(dwell(&p->bad[4],x->emf<p->exit_emf,32U))reason|=SL_P_EMF;
        if(dwell(&p->bad[5],!p->filter_valid || speed_error>20.0f
            || fabsf(p->enc_rpm)<80.0f || fabsf(p->enc_rpm)>120.0f,80U))reason|=SL_P_SPEED;
        if(dwell(&p->bad[6],fabsf(p->follow-p->follow_zero)>15.0f/360.0f,80U))reason|=SL_P_FOLLOW;
    }
    if(p->frame && p->phase==SL_P_SKIP)p->stop=1U;
    if(p->stop)reason|=SL_P_STOP;
    if(reason || (p->frame_committed && p->phase==SL_P_HOLD && p->cycles>=4800U)) {
        /* Timed completion has first_reason=0; terminal low-current/deadline
         * remains independently recorded by exit_end. */
        if(!reason)p->done_tick=x->tick;
        exit_capture(p,x,reason,forced_q24);
        p->reason=reason; p->exit_tick=x->tick; p->phase=SL_P_RELEASE; p->cycles=0;
        p->release_theta=(float)x->actual_theta_q24/16777216.0f;
        p->release_id=p->last_id; p->release_iq=p->last_iq;
        goto release;
    }
    if(active && p->frame) {
        if(p->frame_committed) {
            ++p->cycles;
            *theta=x->smo_theta_q24; *id=p->frame_id; *iq=p->frame_iq;
            p->offset=diff((float)*theta/16777216.0f-forced);
            p->ratio=1.0f;p->max_offset=maxf(p->max_offset,fabsf(p->offset));
        }
        /* RISE requests a transaction in ToqLoop after this cycle's acquisition.
         * Until commit, the complete old frame remains authoritative. */
        goto remember;
    }
    if(active) {
        uint32_t ramp=p->a3?8000U:(p->a2?3200U:1600U);
        float peak=p->a3?0.25f:(p->a2?0.10f:0.05f);
        float cap=p->a3?15.0f/360.0f:(p->a2?6.0f/360.0f:3.0f/360.0f);
        float inverse=p->a3?1.0f/8000.0f:(p->a2?1.0f/3200.0f:1.0f/1600.0f);
        ++p->cycles;
        if(p->phase==SL_P_RISE) {
            ratio=peak*ease((float)p->cycles*inverse);
            if(p->cycles>=ramp){p->phase=SL_P_HOLD;p->cycles=0U;}
        } else if(p->phase==SL_P_HOLD) {
            ratio=peak;
            if(p->cycles>=1920U){p->phase=SL_P_RETURN;p->cycles=0U;}
        } else if(p->cycles<ramp)ratio=peak*(1.0f-ease((float)p->cycles*inverse));
        goal=ratio*delta;
        if(goal>cap)goal=cap;
        if(goal< -cap)goal= -cap;
        step=goal-p->offset;
        if(step>1.0f/96000.0f)step=1.0f/96000.0f;
        if(step< -1.0f/96000.0f)step= -1.0f/96000.0f;
        p->offset+=step; p->ratio=ratio;
        p->max_offset=maxf(p->max_offset,fabsf(p->offset));
        *theta=(int32_t)(wrap(forced+p->offset)*16777216.0f);
        if(p->phase==SL_P_RETURN && p->cycles>=ramp && p->offset==0.0f) {
            p->phase=SL_P_DONE;p->done_tick=x->tick;
        }
        if(p->phase==SL_P_RETURN && p->cycles>=ramp+1600U) {
            diag_begin(&p->exit_diag);
            p->exit_diag.return_timeout_valid=1U;p->exit_diag.return_timeout_tick=x->tick;
            diag_end(&p->exit_diag);
            p->stop=1U; /* Next sample releases from the actual angle, never jumps. */
        }
    }
remember:
    p->last_id=*id;p->last_iq=*iq;
    p->last_speed=p->frame?x->smo_rpm:rpm;
    return 0U;
release:
    ++p->cycles;
    p->release_theta=wrap(p->release_theta+p->last_speed*(10.0f/(60.0f*16000.0f)));
    *theta=(int32_t)(p->release_theta*16777216.0f);
    scale=p->cycles<320U?1.0f-(float)p->cycles/320.0f:0.0f;
    *id=p->release_id*scale; *iq=p->release_iq*scale;
    if(dwell(&p->low_current,p->cycles>=320U && current2<sq(0.002f),160U) || p->cycles>=800U) {
        exit_end(p,x->tick,p->low_current>=160U?SL_PE_LOW_CURRENT:SL_PE_DEADLINE,0U);
        p->phase=SL_P_OFF;*id=*iq=0.0f;return 1U;
    }
    return 0U;
}


volatile SL_PROBE_DIAGNOSTICS g_probe_diagnostics;
void SlProbeDiagnostics_Begin(volatile SL_PROBE_DIAGNOSTICS *d,uint32_t record)
{
    d->active=0U;
    memset((void *)d,0,sizeof(*d));
    d->magic=0x50444931UL;d->abi=1U;d->record_id=record;
    d->previous_bucket=9U;d->active=1U;
}
void SlProbeDiagnostics_Record(volatile SL_PROBE_DIAGNOSTICS *d,
    uint32_t bucket,uint32_t tick,uint32_t saturated,uint32_t failed)
{
    uint32_t same_bucket;
    if(!d->active || bucket>8U)return;
    ++d->sequence;
    same_bucket=d->previous_bucket==bucket;
    if(d->total_samples && tick-d->last_tick!=1U) {
        ++d->gap_count;d->run=d->bucket_run=0U;
    }
    ++d->samples[bucket];++d->total_samples;
    if(failed)++d->failed_samples;
    if(saturated && !failed) {
        if(!d->total_saturated)d->first_saturated_tick=tick;
        ++d->total_saturated;
        d->last_saturated_tick=tick;++d->saturated[bucket];++d->run;
        if(d->run>d->max_run_any)d->max_run_any=d->run;
        /* Per-bucket maximum excludes samples in the preceding bucket.
         * Global run stays continuous across phase transitions. */
        if(!same_bucket)d->bucket_run=0U;
        ++d->bucket_run;
        if(d->bucket_run>d->max_run[bucket])d->max_run[bucket]=d->bucket_run;
    } else d->run=d->bucket_run=0U;
    d->previous_bucket=bucket;d->last_tick=tick;
    ++d->sequence;
}
void SlProbeDiagnostics_Finish(volatile SL_PROBE_DIAGNOSTICS *d)
{
    if(!d->active)return;
    ++d->sequence;d->active=0U;++d->sequence;
}

void SlProbeTrace_Begin(volatile SL_PROBE_TRACE *t,uint32_t record)
{
    memset((void *)&t->h,0,sizeof(t->h));
    t->h.magic=0x41315054UL;t->h.abi=4U;t->h.record_id=record;t->h.active=1U;
}
void SlProbeTrace_Finish(volatile SL_PROBE_TRACE *t,const SL_PROBE *p,uint32_t reason)
{
    uint32_t i;
    if(!t->h.active)return;
    ++t->h.sequence;t->h.active=0;t->h.frozen=1;t->h.finish_reason=reason;
    t->h.baseline_valid=p->baseline_valid;t->h.baseline_count=p->baseline_count;
    t->h.b0=p->b0;t->h.sigma=p->sigma;t->h.entry_emf=p->entry_emf;t->h.exit_emf=p->exit_emf;
    t->h.max_good=p->max_good;
    for(i=0;i<11U;++i)t->h.gate_counts[i]=p->gate_counts[i];
    t->h.max_current2=p->max_current2;t->h.max_tracking2=p->max_tracking2;
    t->h.max_phase=p->max_phase;t->h.min_emf=p->min_emf;t->h.max_speed_error=p->max_speed_error;
    ++t->h.sequence;
}
void SlProbeTrace_Record(volatile SL_PROBE_TRACE *t,const SL_PROBE *p,
    const SL_PROBE_LONG *s,int32_t raw_u,int32_t raw_v)
{
    SL_PROBE_FAST f;
    uint32_t extended=p->a2 || p->a3 || p->frame;
    uint32_t end=extended && p->done_tick && s->tick-p->done_tick>=1920U;
    uint32_t long_due=SlProbeTrace_LongDue(t,p,s->tick);
    if(!t->h.active)return;
    ++t->h.sequence;
    if(!t->h.trigger_tick && (p->entry_tick || p->phase==SL_P_SKIP || p->reason))
        t->h.trigger_tick=s->tick;
    if(!t->h.fast_trigger_tick && (p->reason || (s->flags&0x80000000UL)))
        t->h.fast_trigger_tick=s->tick;
    if(!t->h.fast_frozen) {
        f.tick=s->tick;f.phase=s->phase;f.gates=s->gates;f.reason=s->reason;
        f.applied=s->applied;f.forced=s->forced;f.id_ref=s->id_ref;f.iq_ref=s->iq_ref;
        f.id=s->id;f.iq=s->iq;f.ud=s->ud;f.uq=s->uq;f.raw_u=raw_u;f.raw_v=raw_v;
        f.flags=s->flags;f.offset=s->offset;
        t->fast[t->h.fast_write]=f;t->h.fast_write=(t->h.fast_write+1U)%SL_PROBE_FAST_N;
        if(t->h.fast_count<SL_PROBE_FAST_N)++t->h.fast_count;
        if(t->h.fast_trigger_tick && s->tick-t->h.fast_trigger_tick>=16U)t->h.fast_frozen=1U;
    }
    /* Extended profiles save terminal and +120 ms boundaries off cadence. */
    ++t->h.divider;
    if(long_due) {
        t->h.divider=0;t->slow[t->h.long_write]=*s;
        t->h.long_write=(t->h.long_write+1U)%SL_PROBE_LONG_N;
        if(t->h.long_count<SL_PROBE_LONG_N)++t->h.long_count;
    }
    ++t->h.sequence;
    if(end || (!extended && t->h.trigger_tick && s->tick-t->h.trigger_tick>=8320U))
        SlProbeTrace_Finish(t,p,1U);
}
