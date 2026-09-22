#ifndef SENSORLESS_TRAVEL90_H_
#define SENSORLESS_TRAVEL90_H_
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Bounded continuous-pull test, motor rpm, 101:1 reducer.
 * Mode 5 keeps its legacy TRAVEL90 name; 90 deg is the independent guard.
 * The nominal 85-deg path leaves room for alignment, load angle and release. */
#define SL_TRAVEL_TARGET_OUTPUT_DEG 85.0f
#define SL_TRAVEL_MOTOR_TURNS (SL_TRAVEL_TARGET_OUTPUT_DEG * 101.0f / 360.0f)
#define SL_TRAVEL_MOVE_S (SL_TRAVEL_MOTOR_TURNS * 60.0f / 100.0f + 1.0f)
#define SL_TRAVEL_DECEL_START_S (SL_TRAVEL_MOVE_S - 1.0f)
#define SL_TRAVEL_OUTPUT_LIMIT_COUNTS 131072
#define SL_TRAVEL_RELEASE_S 0.20f
#define SL_TRAVEL_MAX_MS 20000U
#define SL_TRAVEL_POINTS 512U
static inline float SlTravel_Turns(float t)
{
    if(t<=0.0f) return 0.0f;
    if(t<1.0f) return (50.0f/60.0f)*t*t;
    if(t<SL_TRAVEL_DECEL_START_S) return (100.0f/60.0f)*(t-0.5f);
    if(t<SL_TRAVEL_MOVE_S) {float left=SL_TRAVEL_MOVE_S-t; return SL_TRAVEL_MOTOR_TURNS-(50.0f/60.0f)*left*left;}
    return SL_TRAVEL_MOTOR_TURNS;
}
static inline float SlTravel_Speed(float t)
{
    if(t<0.0f || t>=SL_TRAVEL_MOVE_S) return 0.0f;
    if(t<1.0f) return 100.0f*t;
    if(t<SL_TRAVEL_DECEL_START_S) return 100.0f;
    return 100.0f*(SL_TRAVEL_MOVE_S-t);
}
typedef struct {uint32_t ms,motor,output,flags;} SL_TRAVEL_POINT;
typedef struct {
    uint32_t abi,active,done,reason,start_ms,elapsed_ms,last_ms,last_sequence;
    uint32_t motor0,output0,motor_last,output_last,count,overflow,post_ms;
    int32_t motor_counts,output_counts,motor_motion0,output_peak;
    uint32_t ramp_seen,bad_follow_ms,run_samples,invalid_samples;
    float max_follow_turns;
    SL_TRAVEL_POINT points[SL_TRAVEL_POINTS];
} SL_TRAVEL_MONITOR;
static inline int32_t SlTravel_Delta(uint32_t a,uint32_t b,uint32_t period)
{
    int32_t d=(int32_t)a-(int32_t)b;
    if(d>(int32_t)(period/2U)) d-=(int32_t)period;
    if(d<-(int32_t)(period/2U)) d+=(int32_t)period;
    return d;
}
static inline void SlTravel_Begin(volatile SL_TRAVEL_MONITOR *s,uint32_t ms,
    uint32_t seq,uint32_t motor,uint32_t output)
{
    memset((void*)s,0,sizeof(*s));s->abi=1;s->active=1;
    s->start_ms=s->last_ms=ms;s->last_sequence=seq;
    s->motor0=s->motor_last=motor;s->output0=s->output_last=output;
}
/* Reasons: 1 encoder invalid/stale, 2 output boundary, 3 following error,
 * 4 reverse travel, 5 total timeout. First reason is immutable.
 * Encoder signs follow the previously verified positive motor/output convention.
 * No encoder value is used as the control angle or a speed-PI feedback. */
static inline void SlTravel_Sample(volatile SL_TRAVEL_MONITOR *s,uint32_t ms,
    uint32_t seq,uint32_t valid,uint32_t motor,uint32_t output,
    uint32_t ramp,float path_time,int32_t direction,uint32_t enabled)
{
    uint32_t reason=0,record=0;
    if(!s->active)return;
    s->elapsed_ms=ms-s->start_ms;
    if(!valid) {s->invalid_samples++;reason=1;}
    else if(seq!=s->last_sequence) {
        s->motor_counts+=SlTravel_Delta(motor,s->motor_last,131072U);
        s->output_counts+=SlTravel_Delta(output,s->output_last,524288U);
        s->motor_last=motor;s->output_last=output;
        s->last_sequence=seq;s->last_ms=ms;s->run_samples++;
    }
    if(ms-s->last_ms>10U)reason=1;
    if(enabled) {
        int32_t out=s->output_counts*direction;
        if(out>s->output_peak)s->output_peak=out;
        if(out>=SL_TRAVEL_OUTPUT_LIMIT_COUNTS || out<=-SL_TRAVEL_OUTPUT_LIMIT_COUNTS)reason=2;
        if(out < -728)reason=4; /* 0.5 output degree backwards */
        if(ramp) {
            if(!s->ramp_seen) {s->motor_motion0=s->motor_counts;s->ramp_seen=1;}
            float error=fabsf((float)((s->motor_counts-s->motor_motion0)*direction)/131072.0f-SlTravel_Turns(path_time));
            if(error>s->max_follow_turns)s->max_follow_turns=error;
            if(path_time>0.5f && error>0.05f) {
                if(!s->bad_follow_ms)s->bad_follow_ms=ms;
                if(ms-s->bad_follow_ms>=100U)reason=3;
            } else s->bad_follow_ms=0;
        }
        if(s->elapsed_ms>=SL_TRAVEL_MAX_MS)reason=5;
    } else {
        if(!s->post_ms)s->post_ms=ms;
        if(ms-s->post_ms>=1000U) {s->active=0;s->done=1;record=1;}
    }
    if(reason && !s->reason){s->reason=reason;record=1;}
    if(!s->count || record || s->elapsed_ms-s->points[s->count-1U].ms>=50U) {
        if(s->count<SL_TRAVEL_POINTS) {
            uint32_t n=s->count++;s->points[n].ms=s->elapsed_ms;
            s->points[n].motor=motor;s->points[n].output=output;
            s->points[n].flags=(valid?1U:0U)|(enabled?2U:0U)|(ramp?4U:0U);
        } else s->overflow=1;
    }
}
#endif
