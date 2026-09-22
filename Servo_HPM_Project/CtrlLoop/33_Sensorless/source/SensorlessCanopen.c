#include "SensorlessCanopen.h"
#include "SensorlessProduction.h"
#include "SensorlessShadow.h"
#include "SensorlessTravel90.h"
#include "SensorlessEncoderMonitor.h"
#include "StartupTiming.h"
extern volatile SL_ENCODER_MONITOR g_sl_encoder;
volatile SL_TRAVEL_MONITOR g_sl_travel;
static uint32_t travel_over_snapshot;
static uint32_t travel_overruns(void)
{
    uint32_t n,sum=0;
    for(n=2;n<32;n++)sum+=g_startup_timing.bins[n].over;
    return sum;
}
#include "Drive.h"
#include <stddef.h>
#include <string.h>
#include <limits.h>

typedef enum { SL_BITS32, SL_U16, SL_I16, SL_FLOAT } SL_KIND;
typedef struct { size_t offset; SL_KIND kind; } SL_FIELD;
static const SL_FIELD recipe_fields[] = {
    {offsetof(SOURCE_FOC_RECIPE, recipe_id), SL_BITS32},
    {offsetof(SOURCE_FOC_RECIPE, recipe_revision), SL_BITS32},
    {offsetof(SOURCE_FOC_RECIPE, phase_rs_ohm), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, phase_ld_h), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, phase_lq_h), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, current_base_a), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, voltage_base_v), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, rated_current_a), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, current_limit_a), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, max_speed_rpm), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, current_bandwidth_hz), SL_FLOAT},
    {offsetof(SOURCE_FOC_RECIPE, pole_pairs), SL_U16},
    {offsetof(SOURCE_FOC_RECIPE, allow_continuous_rotation), SL_U16},
};
static const SL_FIELD tuning_fields[] = {
    {offsetof(SENSORLESS_PRODUCTION_TUNING, mask), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, align_angle_turns), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, align_current_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, startup_iq_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, iq_limit_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, handoff_speed_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, open_loop_accel_rpm_s), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, run_accel_rpm_s), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, speed_kp), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, speed_ki), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, minimum_lock_quality), SL_U16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, align_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, lock_dwell_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, blend_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, lock_timeout_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, unlock_dwell_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, monitor_dwell_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, fallback_ready_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, stop_dwell_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, stop_timeout_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, fallback_max_angle_error_deg), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, monitor_max_angle_error_deg), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, monitor_max_speed_error_rpm), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, stop_speed_rpm), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, sensorless_exit_speed_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, minimum_hfi_quality), SL_U16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_enable), SL_U16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, allow_open_loop_fallback), SL_U16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_lock_dwell_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_timeout_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_injection_frequency_hz), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_injection_current_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_response_lpf_alpha), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_speed_lpf_alpha), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_tracking_kp), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_tracking_ki), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_min_response_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_lock_error), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_estimator_lock_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_estimator_unlock_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_pole_detect_current_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_pole_min_contrast), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_pole_response_polarity), SL_I16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_pole_settle_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, hfi_pole_measure_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_switching_gain), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_boundary_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_emf_lpf_alpha), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_emf_delay_comp_gain), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_pll_kp), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_pll_ki), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_pll_speed_lpf_alpha), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_weak_emf_speed_decay), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_min_emf_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_unlock_emf_ratio), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_lock_phase_error), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_max_accel_e_rad_s2), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_current_hat_limit_pu), SL_FLOAT},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_lock_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, smo_unlock_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, debug_mode), SL_U16},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, current_ramp_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, if_hold_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, iq_hold_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, ramp_timeout_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, total_timeout_ms), SL_BITS32},
    {offsetof(SENSORLESS_PRODUCTION_TUNING, run_test_ms), SL_BITS32},
};
static const SL_FIELD startup_fields[] = {
    {offsetof(SENSORLESS_STARTUP_CONFIG, sample_time_s), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, pole_pairs), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, speed_base_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, max_speed_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, align_angle_turns), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, align_current_pu), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, startup_iq_pu), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, iq_limit_pu), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, handoff_speed_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, sensorless_exit_speed_rpm), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, open_loop_accel_rpm_s), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, run_accel_rpm_s), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, speed_kp), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, speed_ki), SL_FLOAT},
    {offsetof(SENSORLESS_STARTUP_CONFIG, minimum_lock_quality), SL_U16},
    {offsetof(SENSORLESS_STARTUP_CONFIG, minimum_hfi_quality), SL_U16},
    {offsetof(SENSORLESS_STARTUP_CONFIG, hfi_enable), SL_U16},
    {offsetof(SENSORLESS_STARTUP_CONFIG, allow_open_loop_fallback), SL_U16},
    {offsetof(SENSORLESS_STARTUP_CONFIG, align_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, lock_dwell_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, blend_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, lock_timeout_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, hfi_lock_dwell_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, hfi_timeout_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, unlock_dwell_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, monitor_dwell_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, fallback_ready_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, stop_dwell_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, stop_timeout_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, fallback_max_angle_error_q24), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, monitor_max_angle_error_q24), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, monitor_max_speed_error_rpm), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, stop_speed_rpm), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, debug_mode), SL_U16},
    {offsetof(SENSORLESS_STARTUP_CONFIG, current_ramp_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, if_hold_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, iq_hold_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, ramp_timeout_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, total_timeout_cycles), SL_BITS32},
    {offsetof(SENSORLESS_STARTUP_CONFIG, run_test_cycles), SL_BITS32},
};
static const SL_FIELD smo_fields[] = {
    {offsetof(SMO_PLL_PARAMS, sample_time_s), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, stator_resistance_pu), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, current_model_gain), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, switching_gain), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, boundary_pu), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, emf_lpf_alpha), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, emf_delay_comp_gain), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, pll_kp), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, pll_ki), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, pll_speed_lpf_alpha), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, weak_emf_speed_decay), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, min_emf_pu), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, unlock_emf_ratio), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, lock_phase_error), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, max_electrical_rad_s), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, max_accel_e_rad_s2), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, current_hat_limit_pu), SL_FLOAT},
    {offsetof(SMO_PLL_PARAMS, lock_cycles), SL_U16},
    {offsetof(SMO_PLL_PARAMS, unlock_cycles), SL_U16},
};
#define COUNT(a) ((uint16_t)(sizeof(a)/sizeof((a)[0])))
static SOURCE_FOC_RECIPE staged_recipe;
static SENSORLESS_PRODUCTION_TUNING staged_tuning;
static volatile uint32_t request, acknowledge, result, pending;
static volatile uint32_t command, arm, wire_version, heartbeat, heartbeat_ms, heartbeat_valid;
static volatile int32_t direction, speed;
static volatile uint32_t trace_record;
volatile uint32_t g_sl_can_elapsed_ms, g_sl_can_abort_reason;
static uint32_t last_sequence;
volatile SL_SAMPLING_DIAG g_sl_sampling;
void SensorlessSampling_Record(uint16_t phase, int16_t u, int16_t v,
                               int16_t zero_u, int16_t zero_v)
{
    volatile SL_SAMPLING_PHASE *p;
    if(phase>3U)return;
    ++g_sl_sampling.sequence;
    p=&g_sl_sampling.phase[phase];
    if(!p->samples) {
        p->min_u=p->max_u=u;
        p->min_v=p->max_v=v;
    } else {
        if(u<p->min_u)p->min_u=u;
        if(u>p->max_u)p->max_u=u;
        if(v<p->min_v)p->min_v=v;
        if(v>p->max_v)p->max_v=v;
    }
    if(p->samples!=UINT32_MAX)++p->samples;
    if((u==INT16_MIN || u==INT16_MAX || v==INT16_MIN || v==INT16_MAX) &&
       p->rail_samples!=UINT32_MAX)++p->rail_samples;
    p->last_u=u; p->last_v=v; p->zero_u=zero_u; p->zero_v=zero_v;
    g_sl_sampling.last_phase=phase;
    ++g_sl_sampling.sequence;
}

volatile SENSORLESS_JLINK_MAILBOX g_sensorless_jlink;
static volatile uint32_t jl_owner, jl_enable, jl_begin, jl_limit;
static uint32_t jl_wait_start, jl_started;
static int32_t jl_direction, jl_speed;
uint16_t SensorlessJlink_OwnsControl(void) { return (uint16_t)jl_owner; }
uint16_t SensorlessJlink_EnableRequested(void) { return (uint16_t)jl_enable; }
static void jlink_exit(uint32_t reason)
{
    jl_enable=0; jl_wait_start=0;
    if(!g_sensorless_jlink.exit_reason)g_sensorless_jlink.exit_reason=reason;
    if((g_sensorless_shadow.startup.state!=SENSORLESS_START_IDLE ||
        g_sensorless_shadow.startup.start_requested) &&
        g_sensorless_shadow.startup.state!=SENSORLESS_START_FAULT)
        SensorlessStartup_Trip(&g_sensorless_shadow.startup,SENSORLESS_FAULT_EXTERNAL);
}
static void jlink_service(void)
{
    uint32_t req=g_sensorless_jlink.request_seq;
    uint32_t travel=SENSORLESS_IS_TRAVEL(g_sensorless_shadow.startup.config.debug_mode);
    if(req!=g_sensorless_jlink.acknowledge_seq) {
        uint32_t cmd=g_sensorless_jlink.command, result=0;
        if(cmd==2) {
            if(jl_owner && jl_enable && jl_started
                && SENSORLESS_IS_PROBE(g_sensorless_shadow.startup.config.debug_mode))
                SensorlessShadow_RequestStop();
            else if(jl_owner)jlink_exit(2);
        }
        else if(g_sensorless_jlink.abi!=SL_JLINK_ABI ||
                g_sensorless_jlink.arm_key!=SL_JLINK_KEY)result=1;
        else if(cmd==3) {
            if(jl_enable || !SensorlessJlink_PlatformOff())result=4;
            else { heartbeat_valid=0; jl_owner=0; }
        } else if(cmd!=1)result=3;
        else if(jl_enable || g_sl_travel.active || pending ||
                g_sensorless_production.request_seq!=g_sensorless_production.acknowledge_seq ||
                g_sensorless_shadow.startup.state!=SENSORLESS_START_IDLE ||
                g_sensorless_shadow.startup.start_requested)result=2;
        else if(!SensorlessJlink_PlatformOff())result=4;
        else if(!g_source_foc_ready || !g_sensorless_shadow.initialized ||
            !SensorlessStartup_ValidateConfig(&g_sensorless_shadow.startup.config) ||
            (g_sensorless_shadow.startup.config.debug_mode!=SENSORLESS_DEBUG_ALIGN &&
             g_sensorless_shadow.startup.config.debug_mode!=SENSORLESS_DEBUG_IF &&
             g_sensorless_shadow.startup.config.debug_mode!=SENSORLESS_DEBUG_D_AXIS &&
             g_sensorless_shadow.startup.config.debug_mode!=SENSORLESS_DEBUG_D_AXIS_SCAN &&
             g_sensorless_shadow.startup.config.debug_mode!=SENSORLESS_DEBUG_TRAVEL90 &&
             !SENSORLESS_IS_PROBE(g_sensorless_shadow.startup.config.debug_mode)) ||
            g_sensorless_shadow.startup.config.hfi_enable ||
            ((g_sensorless_shadow.startup.config.debug_mode==SENSORLESS_DEBUG_IF ||
              g_sensorless_shadow.startup.config.debug_mode==SENSORLESS_DEBUG_D_AXIS ||
              g_sensorless_shadow.startup.config.debug_mode==SENSORLESS_DEBUG_D_AXIS_SCAN || travel) &&
             !g_source_foc_recipe.allow_continuous_rotation) ||
            g_sensorless_jlink.duration_ms==0 ||
            g_sensorless_jlink.duration_ms>(travel?SL_TRAVEL_MAX_MS:SL_JLINK_MAX_MS) ||
            (travel && (g_sensorless_jlink.duration_ms!=SL_TRAVEL_MAX_MS
             || g_sensorless_jlink.target_speed_rpm!=100
             || !g_sl_encoder.initialized || !g_sl_encoder.valid
             || g_sl_encoder.age_ms || (g_sl_encoder.sequence&1U))) ||
            (g_sensorless_jlink.direction!=1 && g_sensorless_jlink.direction!=-1) ||
            g_sensorless_jlink.target_speed_rpm<=0 ||
            !((float)g_sensorless_jlink.target_speed_rpm>=g_sensorless_shadow.startup.config.handoff_speed_rpm &&
              (float)g_sensorless_jlink.target_speed_rpm<=g_sensorless_shadow.startup.config.max_speed_rpm))
                result=3;
        else {
            if(travel)travel_over_snapshot=travel_overruns();
            if(travel) SlTravel_Begin(&g_sl_travel,g_sl_can_elapsed_ms,
                g_sl_encoder.sequence,g_sl_encoder.motor_raw,g_sl_encoder.output_raw);
            jl_begin=g_sl_can_elapsed_ms; jl_limit=g_sensorless_jlink.duration_ms;
            jl_direction=g_sensorless_jlink.direction; jl_speed=g_sensorless_jlink.target_speed_rpm;
            jl_started=0; jl_wait_start=1;
            g_sensorless_jlink.exit_reason=0; g_sensorless_jlink.elapsed_ms=0;
            heartbeat_valid=0;
            jl_owner=1; jl_enable=1;
        }
        g_sensorless_jlink.arm_key=0;
        g_sensorless_jlink.result=result;
        g_sensorless_jlink.acknowledge_seq=req;
    }
    if(jl_owner && jl_enable) {
        if(g_sensorless_shadow.startup.state==SENSORLESS_START_FAULT)jlink_exit(4);
        else if(jl_started && !g_sensorless_shadow.startup.start_requested &&
                g_sensorless_shadow.startup.state==SENSORLESS_START_IDLE)
            jlink_exit(g_sensorless_shadow.startup.probe.reason ? 7U : 5U);
        else if(jl_wait_start && SensorlessJlink_PlatformReady()) {
            jl_wait_start=0;
            g_sensorless_production.command=3;
            g_sensorless_production.arm_key=SENSORLESS_PRODUCTION_ARM_KEY;
            g_sensorless_production.direction=(int16_t)jl_direction;
            g_sensorless_production.target_speed_rpm=jl_speed;
            ++g_sensorless_production.request_seq;
            SensorlessProduction_Service1ms();
            if(g_sensorless_production.command_result!=SENSORLESS_PROD_RESULT_ACCEPTED)
                jlink_exit(3);
            else jl_started=1;
        }
    }
    if(g_sl_travel.active) {
        uint32_t seq=g_sl_encoder.sequence, valid=g_sl_encoder.initialized &&
            g_sl_encoder.valid && !g_sl_encoder.age_ms && !(seq&1U);
        uint32_t motor=g_sl_encoder.motor_raw, output=g_sl_encoder.output_raw;
        if(seq!=g_sl_encoder.sequence)valid=0;
        SlTravel_Sample(&g_sl_travel,g_sl_can_elapsed_ms,seq,valid,motor,output,
            g_sensorless_shadow.startup.state==SENSORLESS_START_OPEN_LOOP_RAMP,
            (float)g_sensorless_shadow.startup.state_cycles*
                g_sensorless_shadow.startup.config.sample_time_s,
            jl_direction,jl_enable);
        if(jl_enable && travel_overruns()!=travel_over_snapshot && !g_sl_travel.reason)
            g_sl_travel.reason=6U;
        if(g_sl_travel.reason && jl_enable)jlink_exit(6);
    }
    g_sensorless_jlink.owner=jl_owner;
    g_sensorless_jlink.enable_requested=jl_enable;
}


static uint16_t fields_for(uint16_t index, const SL_FIELD **fields,
                          volatile void **base, uint16_t *count)
{
    switch(index) {
    case 0x2F01: *fields=recipe_fields; *base=&staged_recipe; *count=COUNT(recipe_fields); return 1;
    case 0x2F02: *fields=tuning_fields; *base=&staged_tuning; *count=COUNT(tuning_fields); return 1;
    case 0x2F03: *fields=recipe_fields; *base=&g_sensorless_production.effective_recipe; *count=COUNT(recipe_fields); return 1;
    case 0x2F04: *fields=startup_fields; *base=&g_sensorless_production.effective_startup; *count=COUNT(startup_fields); return 1;
    case 0x2F05: *fields=smo_fields; *base=&g_sensorless_production.effective_smo; *count=COUNT(smo_fields); return 1;
    default: return 0;
    }
}
uint16_t SensorlessCanopen_FieldCount(uint16_t index)
{
    const SL_FIELD *f; volatile void *b; uint16_t count;
    if(index==0x2F00) return 16;
    if(index==0x2F06) return 16;
    return fields_for(index,&f,&b,&count)?count:0;
}
static uint32_t field_read(volatile void *base, const SL_FIELD *f)
{
    volatile uint8_t *p=(volatile uint8_t *)base+f->offset;
    if(f->kind==SL_U16) return *(volatile uint16_t *)p;
    if(f->kind==SL_I16) return (uint32_t)(int32_t)*(volatile int16_t *)p;
    if(f->kind==SL_FLOAT) { float v=*(volatile float *)p; uint32_t bits; memcpy(&bits,&v,4); return bits; }
    return *(volatile uint32_t *)p;
}
static uint16_t field_write(volatile void *base,const SL_FIELD *f,uint32_t value)
{
    volatile uint8_t *p=(volatile uint8_t *)base+f->offset;
    if(f->kind==SL_U16) { if(value>UINT16_MAX)return 0; *(volatile uint16_t *)p=(uint16_t)value; }
    else if(f->kind==SL_I16) { if((int32_t)value<INT16_MIN || (int32_t)value>INT16_MAX)return 0; *(volatile int16_t *)p=(int16_t)value; }
    else if(f->kind==SL_FLOAT) { float v; memcpy(&v,&value,4); *(volatile float *)p=v; }
    else *(volatile uint32_t *)p=value;
    return 1;
}
static uint32_t float_bits(float v) { uint32_t b; memcpy(&b,&v,4);return b; }
void SensorlessCanopen_Init(void)
{
    memset((void *)&g_sl_sampling,0,sizeof(g_sl_sampling));
    g_sl_sampling.abi=1U;
    memset(&staged_recipe,0,sizeof(staged_recipe));
    staged_tuning=g_sensorless_production.tuning;
    request=acknowledge=result=pending=command=arm=wire_version=0;
    heartbeat=heartbeat_ms=heartbeat_valid=0;
    g_sl_can_elapsed_ms=g_sl_can_abort_reason=last_sequence=trace_record=0;
    direction=1; speed=0;
    memset((void *)&g_sensorless_jlink,0,sizeof(g_sensorless_jlink));
    g_sensorless_jlink.abi=SL_JLINK_ABI;
    memset((void *)&g_sl_travel,0,sizeof(g_sl_travel));
    g_sl_travel.abi=1U;
    jl_owner=jl_enable=jl_begin=jl_limit=jl_wait_start=jl_started=0;
}
void SensorlessCanopen_Tick1ms(void)
{
    ++g_sl_can_elapsed_ms;
    if(jl_owner && jl_enable) {
        g_sensorless_jlink.elapsed_ms=(uint32_t)(g_sl_can_elapsed_ms-jl_begin);
        if(g_sensorless_jlink.elapsed_ms>=jl_limit)jlink_exit(1);
    }
}
uint16_t SensorlessCanopen_MotionAllowed(void)
{
    if(SENSORLESS_IS_TRAVEL(g_sensorless_shadow.startup.config.debug_mode)
        && (!jl_owner || !g_sl_travel.active || g_sl_travel.reason))return 0;
    if(jl_owner)return (uint16_t)(jl_enable &&
        (uint32_t)(g_sl_can_elapsed_ms-jl_begin)<jl_limit &&
        SensorlessJlink_PlatformReady());
    return (uint16_t)(heartbeat_valid &&
        (uint32_t)(g_sl_can_elapsed_ms-heartbeat_ms)<SL_CAN_WATCHDOG_MS &&
        SensorlessCanopen_PlatformReady());
}
void SensorlessCanopen_PwmGuard(void)
{
    uint16_t state=(uint16_t)g_sensorless_shadow.startup.state;
    if((state!=SENSORLESS_START_IDLE || g_sensorless_shadow.startup.start_requested) && state!=SENSORLESS_START_FAULT &&
       !SensorlessCanopen_MotionAllowed()) {
        if(!g_sl_can_abort_reason) g_sl_can_abort_reason=
            jl_owner?3U:((!heartbeat_valid || (uint32_t)(g_sl_can_elapsed_ms-heartbeat_ms)>=SL_CAN_WATCHDOG_MS)?1U:2U);
        SensorlessStartup_Trip(&g_sensorless_shadow.startup,SENSORLESS_FAULT_EXTERNAL);
    }
}
uint16_t SensorlessCanopen_Read(uint16_t index,uint8_t sub,uint32_t *value)
{
    const SL_FIELD *f; volatile void *base; uint16_t count;
    if(!value)return 0;
    if(sub==0) { *value=SensorlessCanopen_FieldCount(index); return *value!=0; }
    if(index==0x2F00) {
        switch(sub) {
        case 1:*value=SL_CAN_PROTOCOL;break;
        case 2:*value=request;break;
        case 3:*value=command;break;
        case 4:*value=arm;break;
        case 5:*value=(uint32_t)direction;break;
        case 6:*value=(uint32_t)speed;break;
        case 7:*value=acknowledge;break;
        case 8:*value=result;break;
        case 9:*value=heartbeat;break;
        case 10:*value=SL_CAN_WATCHDOG_MS;break;
        case 11:*value=g_sl_can_abort_reason;break;
        case 12:*value=pending;break;
        case 13:*value=SENSORLESS_FIRMWARE_VERSION;break;
        case 14:*value=SENSORLESS_PARAMETER_VERSION;break;
        case 15:*value=g_sensorless_production.parameter_sequence;break;
        case 16:*value=g_sl_can_elapsed_ms;break;
        default:return 0;
        } return 1;
    }
    if(index==0x2F06) {
        switch(sub) {
        case 1:*value=g_sensorless_diag.startup_state;break;
        case 2:*value=g_sensorless_diag.first_fault;break;
        case 3:*value=(uint32_t)g_sensorless_diag.observer_speed_rpm;break;
        case 4:*value=float_bits(g_sensorless_diag.iq_feedback_pu);break;
        case 5:*value=g_sensorless_diag.observer_locked;break;
        case 6:*value=g_sensorless_diag.observer_quality;break;
        case 7:*value=g_sensorless_diag.encoder_valid;break;
        case 8:*value=g_sensorless_trace.record_id;break;
        case 9:*value=g_sensorless_trace.frozen;break;
        case 10:*value=sizeof(g_sensorless_trace);break;
        case 11:*value=g_sensorless_diag.current_failure;break;
        case 12:*value=g_sensorless_diag.debug_completed;break;
        case 13:*value=g_sensorless_diag.pwm_counter;break;
        case 14:*value=g_source_foc_ready;break;
        case 15:*value=g_sensorless_diag.control_override_active;break;
        case 16:*value=g_sensorless_production.command_result;break;
        default:return 0;
        } return 1;
    }
    if(!fields_for(index,&f,&base,&count) || sub>count)return 0;
    *value=field_read(base,&f[sub-1]);return 1;
}
uint16_t SensorlessCanopen_Write(uint16_t index,uint8_t sub,uint32_t value)
{
    const SL_FIELD *f; volatile void *base; uint16_t count;
    if(jl_owner || g_sensorless_jlink.request_seq!=g_sensorless_jlink.acknowledge_seq)
        return 0; /* reserve ownership while a debugger request is being handled */
    if(index==0x2F00 && sub==9) {
        if(value==heartbeat)return 0;
        heartbeat=value; heartbeat_ms=g_sl_can_elapsed_ms; heartbeat_valid=1;return 1;
    }
    if(pending)return 0;
    if(index==0x2F00) {
        switch(sub) {
        case 1: if(value!=SL_CAN_PROTOCOL)return 0; wire_version=value;return 1;
        case 2:
            if(wire_version!=SL_CAN_PROTOCOL || value==0 || value==last_sequence)return 0;
            request=value; last_sequence=value; pending=1; return 1; /* publish last */
        case 3:
            if(value!=0 && value!=2 && value!=3 && value!=5 && value!=6 && value!=12)return 0;
            command=value;return 1;
        case 4: if(value>UINT16_MAX)return 0;arm=value;return 1;
        case 5: if((int32_t)value!=1 && (int32_t)value!=-1)return 0;direction=(int32_t)value;return 1;
        case 6: if(value>INT32_MAX)return 0;speed=(int32_t)value;return 1;
        default:return 0;
        }
    }
    if(index!=0x2F01 && index!=0x2F02)return 0;
    if(StateMachine.RegulFlg || !SensorlessStartup_IsIdle(&g_sensorless_shadow.startup))return 0;
    if(sub==0 || !fields_for(index,&f,&base,&count) || sub>count)return 0;
    return field_write(base,&f[sub-1],value);
}
void SensorlessCanopen_Service1ms(void)
{
    uint16_t submitted=0;
    jlink_service();
    if(pending) {
        uint32_t seq=g_sensorless_production.request_seq;
        if(seq!=g_sensorless_production.acknowledge_seq) result=SENSORLESS_PROD_RESULT_BUSY;
        else if(command==3 && !SensorlessCanopen_MotionAllowed())
            result=SENSORLESS_PROD_RESULT_SERVO_NOT_READY;
        else {
            if(command==12)g_sensorless_production.source_recipe=staged_recipe;
            if(command==2)g_sensorless_production.tuning=staged_tuning;
            g_sensorless_production.command=(uint16_t)command;
            g_sensorless_production.arm_key=(uint16_t)arm;
            g_sensorless_production.direction=(int16_t)direction;
            g_sensorless_production.target_speed_rpm=speed;
            g_sensorless_production.request_seq=seq+1U;
            submitted=1;
        }
    }
    SensorlessProduction_Service1ms();
    if(pending) {
        if(submitted) result=g_sensorless_production.command_result;
        if(command==3 && result==SENSORLESS_PROD_RESULT_ACCEPTED)g_sl_can_abort_reason=0;
        arm=0; acknowledge=request; pending=0; /* release staged values last */
    }
}
uint16_t SensorlessCanopen_TraceRead(uint32_t offset,void *dst,uint32_t bytes)
{
    uint8_t *out=(uint8_t *)dst;
    volatile const uint8_t *in=(volatile const uint8_t *)&g_sensorless_trace;
    if(!dst || offset>sizeof(g_sensorless_trace) || bytes>sizeof(g_sensorless_trace)-offset ||
       !g_sensorless_trace.frozen || g_sensorless_trace.active || (g_sensorless_trace.sequence&1U))return 0;
    if(offset==0)trace_record=g_sensorless_trace.record_id;
    if(trace_record!=g_sensorless_trace.record_id)return 0;
    uint32_t seq=g_sensorless_trace.sequence;
    for(uint32_t i=0;i<bytes;i++)out[i]=in[offset+i];
    return (uint16_t)(seq==g_sensorless_trace.sequence && g_sensorless_trace.frozen &&
        !g_sensorless_trace.active && trace_record==g_sensorless_trace.record_id);
}

