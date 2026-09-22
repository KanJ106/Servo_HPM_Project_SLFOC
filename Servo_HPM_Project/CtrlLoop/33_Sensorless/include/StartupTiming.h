#ifndef STARTUP_TIMING_H
#define STARTUP_TIMING_H
#include <stdint.h>
/* Fixed uint32 layout, independent of trace/mailbox ABI. */
typedef struct {
    uint32_t count,last,max,over,max_tick,first_over_tick;
} STARTUP_TIMING_BIN;
typedef struct {
    uint32_t tick,entry_state,exit_state,trace_written,elapsed;
    uint32_t pre,torque,post,communication,scope;
    uint32_t acquisition,transform,current_pi,pwm,monitor;
} STARTUP_TIMING_EVENT;
typedef struct {
    uint32_t magic,abi,sequence,cpu_hz,sample_hz,budget_cycles;
    uint32_t pre,torque,post;
    uint32_t acquisition,transform,current_pi,pwm,monitor;
    STARTUP_TIMING_EVENT worst,first_over;
    STARTUP_TIMING_BIN bins[32];
} STARTUP_TIMING;
extern volatile STARTUP_TIMING g_startup_timing;
static inline uint32_t StartupTiming_Cycle(void) {
#ifdef __riscv
    uint32_t v; __asm volatile ("csrr %0, mcycle" : "=r"(v) :: "memory"); return v;
#else
    return 0;
#endif
}
static inline void StartupTiming_Accumulate(volatile STARTUP_TIMING *s,
    const STARTUP_TIMING_EVENT *e,uint32_t drive_enabled) {
    uint32_t stage=drive_enabled ? e->entry_state+1U : 0U;
    uint32_t index=(stage<16U?stage:15U)*2U+(e->trace_written?1U:0U);
    volatile STARTUP_TIMING_BIN *b=&s->bins[index];
    s->sequence++;
    if(b->count!=0xffffffffU)b->count++;
    b->last=e->elapsed;
    if(e->elapsed>b->max){b->max=e->elapsed;b->max_tick=e->tick;}
    if(e->elapsed>s->worst.elapsed)s->worst=*e;
    if(s->budget_cycles && e->elapsed>=s->budget_cycles) {
        if(!b->over)b->first_over_tick=e->tick;
        if(b->over!=0xffffffffU)b->over++;
        if(!s->first_over.elapsed)s->first_over=*e;
    }
    s->sequence++;
}
#endif
