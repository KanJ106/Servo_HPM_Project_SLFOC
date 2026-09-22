#include "SensorlessShadow.h"
#ifdef SENSORLESS_CANOPEN_BUILD
#include "SensorlessCanopen.h"
#include "SensorlessEncoderMonitor.h"
extern volatile SL_ENCODER_MONITOR g_sl_encoder;
volatile SL_ENCODER_TIMING g_sl_encoder_timing;
#endif

#include <math.h>
#include <string.h>

#include "Drive.h"
#include "SensorlessConfig.h"
#include "ProductionMetrics.h"
#include "SV_FaultProtect.h"
#include "SV_StateMachine.h"

#define SENSORLESS_TWO_PI_F        (6.2831853071795864769f)
#define SENSORLESS_Q24_SCALE_F     (16777216.0f)
#define SENSORLESS_SPEED_DEADBAND        (0.001f)
#define SENSORLESS_ENCODER_STALE_SECONDS (0.020f)
#define SENSORLESS_ENCODER_ACTIVE_RPM    (100)

SENSORLESS_SHADOW_RUNTIME g_sensorless_shadow;
volatile SENSORLESS_SHADOW_DIAG g_sensorless_diag;
SOURCE_FOC_CURRENT g_source_foc_current;
SOURCE_FOC_RECIPE g_source_foc_recipe;
volatile uint16_t g_source_foc_ready;
extern volatile uint32_t g_sensorless_fault_detail;
static float g_previous_voltage_alpha, g_previous_voltage_beta;

#if SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U
extern void Sensorless_GlobalFaultHook(uint32_t fault_flags);
#endif


static float Sensorless_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float Sensorless_ElectricalToMechanicalRpm(float omega_e_rad_s)
{
    float pole_pairs = g_sensorless_shadow.motor_profile.effective.pole_pairs;

    return (pole_pairs > 0.0f)
        ? (omega_e_rad_s * 60.0f / (SENSORLESS_TWO_PI_F * pole_pairs))
        : 0.0f;
}

static int32_t Sensorless_TurnsToQ24(float turns)
{
    turns -= floorf(turns);
    if (turns < 0.0f) {
        turns += 1.0f;
    }
    return (int32_t)(turns * SENSORLESS_Q24_SCALE_F);
}

static float Sensorless_Q24ToTurns(int32_t q24)
{
    return (float)RotorFeedback_WrapAngleQ24(q24) / SENSORLESS_Q24_SCALE_F;
}

static void Sensorless_ReadMotorRaw(SENSORLESS_MOTOR_RAW *raw)
{
    raw->sample_hz = (float)DrvCoeff.TpwmFrq;
    raw->stator_resistance_pu = _IQtoF(DrvCoeff.Rs);
    raw->ld_pu = _IQtoF(DrvCoeff.Ld);
    raw->lq_pu = _IQtoF(DrvCoeff.Lq);
    raw->speed_base_rpm = (float)DrvCoeff.SpdBase;
    raw->max_speed_rpm = (float)DrvCoeff.SpdMax;
    raw->pole_pairs = (float)DrvCoeff.MotPoles;
    raw->rated_current_pu = _IQtoF(DrvCoeff.MotIe);
    raw->current_limit_pu = _IQtoF(DrvCoeff.SoftOcPonit);
}

static void Sensorless_BuildMotorProfile(
    const SENSORLESS_MOTOR_OVERRIDE *override_values)
{
    SENSORLESS_MOTOR_RAW raw;

    Sensorless_ReadMotorRaw(&raw);
    MotorParamProfile_Build(&raw, override_values,
                            &g_sensorless_shadow.motor_profile);
    MotorParamProfile_ApplyToSmo(&g_sensorless_shadow.motor_profile,
                                 &g_sensorless_shadow.params);
    g_sensorless_shadow.config_flags =
        g_sensorless_shadow.motor_profile.validation_flags;
}

static void Sensorless_InitStartup(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    const SENSORLESS_MOTOR_PROFILE *profile = &g_sensorless_shadow.motor_profile;
    float iq_limit = profile->effective.current_limit_pu;

    if (profile->effective.rated_current_pu < iq_limit) {
        iq_limit = profile->effective.rated_current_pu;
    }
    SensorlessStartup_DefaultConfig(&config,
        g_sensorless_shadow.params.sample_time_s,
        profile->effective.pole_pairs,
        profile->effective.speed_base_rpm,
        profile->effective.max_speed_rpm,
        iq_limit);
    SensorlessStartup_Init(&g_sensorless_shadow.startup, &config);
}

static void Sensorless_InitHfi(float initial_theta)
{
    const SENSORLESS_MOTOR_PROFILE *profile =
        &g_sensorless_shadow.motor_profile;
    float current_limit = profile->effective.current_limit_pu;

    if (profile->effective.rated_current_pu < current_limit) {
        current_limit = profile->effective.rated_current_pu;
    }
    HfiEstimator_DefaultParams(&g_sensorless_shadow.hfi_params,
        g_sensorless_shadow.params.sample_time_s,
        g_sensorless_shadow.params.max_electrical_rad_s,
        current_limit, profile->saliency_polarity);
    HfiEstimator_Init(&g_sensorless_shadow.hfi,
        &g_sensorless_shadow.hfi_params, initial_theta);
}

static void Sensorless_InitIdentification(void)
{
    MOTOR_PARAM_IDENT_CONFIG config;
    const SENSORLESS_MOTOR_PROFILE *profile =
        &g_sensorless_shadow.motor_profile;
    float current_limit = profile->effective.current_limit_pu;
    float voltage_base_v = (DrvCoeff.UdcBase > 0U)
        ? (float)DrvCoeff.UdcBase : 1.0f;
    float current_base_a = (DrvCoeff.DrvIbase > 0U)
        ? (0.01f * (float)DrvCoeff.DrvIbase) : 0.01f;

    if (profile->effective.rated_current_pu < current_limit) {
        current_limit = profile->effective.rated_current_pu;
    }
    MotorParamIdentification_DefaultConfig(&config,
        g_sensorless_shadow.params.sample_time_s,
        profile->base_electrical_rad_s,
        profile->effective.pole_pairs,
        voltage_base_v,
        current_base_a,
        current_limit);
    MotorParamIdentification_Init(&g_sensorless_shadow.identification,
                                  &config);
}

static uint16_t Sensorless_HfiRequired(void)
{
    SENSORLESS_START_STATE state = g_sensorless_shadow.startup.state;

    if (g_sensorless_shadow.startup.config.hfi_enable == 0U) return 0U;
    return (uint16_t)(((state == SENSORLESS_START_HFI_ACQUIRE)
        || (state == SENSORLESS_START_HFI_RUN)
        || (state == SENSORLESS_START_HFI_REACQUIRE)
        || (state == SENSORLESS_START_BLEND_TO_HFI)
        || ((state == SENSORLESS_START_BLEND)
            && (g_sensorless_shadow.startup.blend_from_source
                == ROTOR_SOURCE_HFI))
        || (state == SENSORLESS_START_STOPPING)
        || (g_sensorless_shadow.identification.output.requires_hfi != 0U))
        ? 1U : 0U);
}

static void Sensorless_ReleaseControlOutputs(void)
{
    g_sensorless_diag.control_override_active = 0U;
    g_sensorless_diag.id_command_pu = 0.0f;
    g_sensorless_diag.iq_command_pu = 0.0f;
#if SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U
    if (g_sensorless_shadow.output_owned != 0U) {
        Etheta.EtaGetSwt = 0U;
        IdRef.IdRefGet = 0U;
        IqRef.IqRefGet = 0U;
        IdRef.IdRefInner = 0;
        IqRef.IqRefInner = 0;
        g_sensorless_shadow.output_owned = 0U;
    }
#endif
}


volatile SENSORLESS_TRACE g_sensorless_trace;
volatile SL_OBSERVATION_TRACE g_observation_trace;
volatile uint32_t g_observation_request = 1U, g_observation_kind = 1U;
SENSORLESS_CHAIN_SAMPLE g_sensorless_chain;
static uint32_t trace_divider, trace_last_state;
static uint32_t trace_sample_divider = SENSORLESS_TRACE_DIVIDER;

static SL_PROBE_INPUT probe_input_cache;
static void Sensorless_ProbeInput(SL_PROBE_INPUT *x)
{
    SL_PROBE_INPUT *c=&probe_input_cache;
    const SMO_PLL_STATE *o=&g_sensorless_shadow.observer;
    c->tick=g_sensorless_shadow.pwm_counter;
#ifdef SENSORLESS_CANOPEN_BUILD
    {
        uint32_t ts=g_sl_encoder_timing.sequence, es=g_sl_encoder.sequence;
        if(!(ts&1U) && !(es&1U)) {
            uint32_t raw=g_sl_encoder.motor_raw, rd=g_sl_encoder_timing.frame_read_tick;
            uint32_t rq=g_sl_encoder_timing.frame_request_tick;
            uint32_t valid=g_sl_encoder.initialized && g_sl_encoder.valid
                && !g_sl_encoder.age_ms && g_sl_encoder.header==0x6AU
                && !g_sl_encoder.status && raw<131072U
                && g_sl_encoder.tick_ms==g_sl_encoder_timing.frame_tick_ms
                && rd-rq<=24U;
            if(ts==g_sl_encoder_timing.sequence && es==g_sl_encoder.sequence) {
                c->encoder_raw=raw;c->encoder_read_tick=rd;c->encoder_request_tick=rq;
                c->encoder_valid=valid;
            }
        }
        /* Odd publisher: reuse only a still-fresh coherent frame. Bad frames
         * explicitly invalidate, and cannot be hidden behind the old cache. */
        if(c->tick-c->encoder_read_tick>32U)c->encoder_valid=0;
    }
#else
    c->encoder_valid=0U;
#endif
    c->chain_valid=g_sensorless_chain.chain_valid
        && (uint32_t)g_sensorless_chain.chain_tick+1U==c->tick;
    c->saturated=g_source_foc_current.saturated;c->locked=o->locked;
    c->emf_a=o->emf_alpha_pu;c->emf_b=o->emf_beta_pu;c->emf=o->emf_magnitude_pu;
    c->phase=o->phase_error_rad;c->smo_rpm=o->omega_e_rad_s*(60.0f/(6.28318530718f*10.0f));
    c->id=_IQtoF(Park.Ds);c->iq=_IQtoF(Park.Qs);
    c->id_ref=g_source_foc_current.id_reference_pu;c->iq_ref=g_source_foc_current.iq_reference_pu;
    c->ud=g_source_foc_current.voltage_d;c->uq=g_source_foc_current.voltage_q;
    c->smo_theta_q24=g_sensorless_shadow.feedback_mux.sensorless.theta_e_q24;
    c->actual_theta_q24=(int32_t)Etheta.Ethetapk;
    *x=*c;
}
static void Sensorless_ProbeRecord(void)
{
    SENSORLESS_STARTUP_RUNTIME *s=&g_sensorless_shadow.startup;
    const SL_PROBE_INPUT *x=&probe_input_cache;
    SL_PROBE_LONG p;
    uint32_t live;
    SlProbeDiagnostics_Record(&g_probe_diagnostics,
        s->state==SENSORLESS_START_ALIGN ? 8U : s->probe.phase,
        g_sensorless_shadow.pwm_counter,g_source_foc_current.saturated,
        g_source_foc_current.failure_reason);
    if(!g_probe_trace.h.active)return;
    live=s->probe.filter_valid && x->encoder_valid && x->tick-x->encoder_read_tick<=32U
        && isfinite(s->probe.smo_mean) && isfinite(s->probe.enc_rpm)
        && isfinite(s->probe.follow);
    p.tick=g_sensorless_shadow.pwm_counter;p.phase=s->probe.phase;
    p.gates=s->probe.gates;p.reason=s->probe.reason;
    p.forced=Sensorless_TurnsToQ24(s->open_loop_theta_turns);
    p.applied=(int32_t)Etheta.Ethetapk;
    p.id_ref=g_sensorless_diag.id_command_pu;p.iq_ref=g_sensorless_diag.iq_command_pu;
    p.id=_IQtoF(Park.Ds);p.iq=_IQtoF(Park.Qs);
    p.ud=g_source_foc_current.voltage_d;p.uq=g_source_foc_current.voltage_q;
    p.offset=s->probe.offset;
    /* Fast fields stay per PWM; long-only fields are built only when saved. */
    if(SlProbeTrace_LongDue(&g_probe_trace,&s->probe,p.tick)) {
        p.smo=x->smo_theta_q24;
        p.encoder_raw=x->encoder_raw;p.encoder_read_tick=x->encoder_read_tick;
        p.encoder_request_tick=x->encoder_request_tick;
        p.emf=x->emf;p.phase_error=x->phase;p.smo_rpm=live?s->probe.smo_mean:0.0f;
        p.encoder_rpm=live?s->probe.enc_rpm:0.0f;
        p.follow=live?s->probe.follow-s->probe.follow_zero:0.0f;p.stage_cycles=s->state_cycles;
    }
    p.flags=(x->encoder_valid?1U:0U)|(x->chain_valid?2U:0U)
        |(g_source_foc_current.saturated?4U:0U)|(x->locked?8U:0U)
        |(live?16U:0U)|(s->probe.a2?64U:0U)|(s->probe.a3?128U:0U)
        |((s->probe.phase>=SL_P_RISE && s->probe.phase<=SL_P_RETURN)
            || (s->probe.phase==SL_P_QUALIFY && s->state_cycles>=32000U)?32U:0U)
        |(s->probe.frame_committed?0x40000UL:0U)
        |((uint32_t)g_sensorless_shadow.observer.quality<<8)
        |(g_source_foc_current.failure_reason?0x80000000UL:0U);
    SlProbeTrace_Record(&g_probe_trace,&s->probe,&p,g_sensorless_chain.sdm_u,g_sensorless_chain.sdm_v);
}

void SensorlessShadow_ClearTrace(void)
{
    if (StateMachine.RegulFlg != 0U) return;
    memset((void *)&g_sensorless_trace, 0, sizeof(g_sensorless_trace));
    g_sensorless_trace.magic = 0x534C5437UL;
    g_sensorless_trace.abi = 3U;
    g_sensorless_trace.sample_bytes = sizeof(SENSORLESS_TRACE_SAMPLE);
    g_sensorless_trace.capacity = SENSORLESS_TRACE_CAPACITY;
}
static void Sensorless_TraceBegin(void)
{
    /* Called only after accepted START; PWM cannot record until active is set. */
    g_sensorless_trace.active = 0U;
    g_sensorless_trace.sequence++;
    g_sensorless_trace.magic = 0x534C5437UL;
    g_sensorless_trace.abi = 3U;
    g_sensorless_trace.sample_bytes = sizeof(SENSORLESS_TRACE_SAMPLE);
    g_sensorless_trace.capacity = SENSORLESS_TRACE_CAPACITY;
    g_sensorless_trace.write_index = g_sensorless_trace.count = 0U;
    g_sensorless_trace.frozen = 0U;
    g_sensorless_trace.record_id++;
    g_sensorless_chain.chain_valid = 0;
    trace_sample_divider = SENSORLESS_TRACE_DIVIDER;
    if (g_sensorless_shadow.startup.config.debug_mode == SENSORLESS_DEBUG_D_AXIS_SCAN) {
        const SENSORLESS_STARTUP_CONFIG *c = &g_sensorless_shadow.startup.config;
        /* Reserve eight records for transitions and finish; compute only at START. */
        uint32_t budget = SENSORLESS_TRACE_CAPACITY - 8U;
        uint32_t span = c->align_cycles + 2U * c->if_hold_cycles + 3U;
        uint32_t divider = (span + budget - 1U) / budget;
        if (divider > trace_sample_divider) trace_sample_divider = divider;
    }
    if (SENSORLESS_IS_TRAVEL(g_sensorless_shadow.startup.config.debug_mode)) {
        const SENSORLESS_STARTUP_CONFIG *c = &g_sensorless_shadow.startup.config;
        uint32_t budget = SENSORLESS_TRACE_CAPACITY - 16U;
        uint32_t divider = (c->total_timeout_cycles + budget - 1U) / budget;
        if (divider > trace_sample_divider) trace_sample_divider = divider;
    }
    trace_divider = 0U; trace_last_state = 0xffffffffUL;
    g_sensorless_trace.sequence++;
    g_sensorless_trace.active = 1U;
    g_observation_kind = g_observation_request == 1U ? 1U : 0U;
    if (SENSORLESS_IS_PROBE(g_sensorless_shadow.startup.config.debug_mode)) {
        g_observation_kind=2U;
        memset(&probe_input_cache,0,sizeof(probe_input_cache));
        SlProbeTrace_Begin(&g_probe_trace,g_sensorless_trace.record_id);
        SlProbeDiagnostics_Begin(&g_probe_diagnostics,g_sensorless_trace.record_id);
    } else if (g_observation_kind == 1U) {
        SL_ALIGN_HEADER h = {0};
        h.sample_hz = DrvCoeff.TpwmFrq;
        h.current_base_a = g_source_foc_recipe.current_base_a;
        h.id_command = g_sensorless_shadow.startup.config.align_current_pu;
        h.current_limit = g_source_foc_current.current_limit_pu;
        h.measured_trip = g_source_foc_current.measured_trip_pu;
        h.kp_d = g_source_foc_current.kp_d; h.kp_q = g_source_foc_current.kp_q;
        h.ki_step = g_source_foc_current.ki_step;
        h.align_angle_turns = g_sensorless_shadow.startup.config.align_angle_turns;
        SlAlignTrace_Begin(&g_align_trace, h, g_sensorless_trace.record_id);
    } else {
        SlSmoTrace_Begin(&g_smo_trace, &g_sensorless_shadow.params,
            g_sensorless_shadow.startup.config.debug_mode == SENSORLESS_DEBUG_TRAVEL90,
            (uint32_t)(2.0f / g_sensorless_shadow.params.sample_time_s + 0.5f));
        g_smo_trace.header.record_id = g_sensorless_trace.record_id;
        {
            uint32_t span=SL_SMO_ALIGN_CAPACITY*SL_SMO_TRACE_DIVIDER;
            uint32_t align=g_sensorless_shadow.startup.config.align_cycles;
            g_smo_trace.header.align_trigger_cycles=align>span?align-span:0U;
        }
    }
}
static void Sensorless_TraceRecord(uint16_t force)
{
    SENSORLESS_STARTUP_RUNTIME *s = &g_sensorless_shadow.startup;
    SENSORLESS_TRACE_SAMPLE p;
    uint32_t finish = (s->state == SENSORLESS_START_FAULT)
        || (s->state == SENSORLESS_START_IDLE && !s->start_requested);
    if (!g_sensorless_trace.active || g_sensorless_trace.frozen) return;
    if (!force && !finish && trace_last_state == (uint32_t)s->state
        && ++trace_divider < trace_sample_divider) return;
    trace_divider = 0U; trace_last_state = (uint32_t)s->state;
    memset(&p, 0, sizeof(p));
    p.tick = g_sensorless_shadow.pwm_counter;
    p.chain = g_sensorless_chain;
    if ((uint32_t)p.chain.chain_tick + 1U != p.tick) p.chain.chain_valid = 0;
    p.mode = s->config.debug_mode; p.state = (uint32_t)s->state;
    p.stage_cycles = s->state_cycles; p.total_cycles = s->total_start_cycles;
    p.first_fault = s->failure.fault;
    p.open_theta_q24 = Sensorless_TurnsToQ24(s->open_loop_theta_turns);
    p.observer_theta_q24 = g_sensorless_shadow.feedback_mux.sensorless.theta_e_q24;
    p.control_theta_q24 = (int32_t)Etheta.Ethetapk; /* actually used by Park/PWM */
    p.angle_error_q24 = RotorFeedback_AngleErrorQ24(p.observer_theta_q24, p.open_theta_q24);
    p.speed_ref_rpm = s->output.speed_ref_rpm;
    p.observer_speed_rpm = (float)g_sensorless_shadow.feedback_mux.sensorless.speed_rpm;
    /* No independent encoder acquisition in this build. Zero + invalid explicitly. */
    p.reference_speed_rpm = 0.0f; p.reference_valid = 0U;
    p.id_ref = s->output.id_ref_pu; p.iq_ref = s->output.iq_ref_pu;
    p.id_fb = _IQtoF(Park.Ds); p.iq_fb = _IQtoF(Park.Qs);
    p.id_limited = g_source_foc_current.id_reference_pu;
    p.iq_limited = g_source_foc_current.iq_reference_pu;
    p.bus_pu = _IQtoF(AdRead.Udc);
    p.ud = g_source_foc_current.voltage_d; p.uq = g_source_foc_current.voltage_q;
    p.emf = g_sensorless_shadow.observer.emf_magnitude_pu;
    p.phase_error_rad = g_sensorless_shadow.observer.phase_error_rad;
    p.blend = s->blend_fraction;
    p.saturated = g_source_foc_current.saturated;
    p.quality = g_sensorless_shadow.observer.quality;
    p.locked = g_sensorless_shadow.observer.locked;
    p.lock_reason = g_sensorless_shadow.observer.lock_reason;
    p.current_failure = g_source_foc_current.failure_reason;
#ifdef SENSORLESS_CANOPEN_BUILD
    /* Bounded snapshot: never spin if PWM preempts the 1ms publisher.
     * Invalid snapshots retain no usable positions. Task ms is not wall time. */
    p.encoder_sequence = g_sl_encoder.sequence;
    if (!(p.encoder_sequence & 1U)) {
        p.encoder_tick_ms = g_sl_encoder.tick_ms;
        p.encoder_motor_raw = g_sl_encoder.motor_raw;
        p.encoder_output_raw = g_sl_encoder.output_raw;
        p.encoder_last_good_ms = g_sl_encoder.last_good_ms;
        p.encoder_valid = g_sl_encoder.initialized && g_sl_encoder.valid
            && g_sl_encoder.age_ms == 0U;
        if (p.encoder_sequence != g_sl_encoder.sequence) p.encoder_valid = 0U;
    }
#endif
    g_sensorless_trace.sequence++;
    g_sensorless_trace.samples[g_sensorless_trace.write_index] = p;
    g_sensorless_trace.write_index = (g_sensorless_trace.write_index + 1U) % SENSORLESS_TRACE_CAPACITY;
    if (g_sensorless_trace.count < SENSORLESS_TRACE_CAPACITY) g_sensorless_trace.count++;
    if (finish) {
        g_sensorless_trace.frozen = 1U; g_sensorless_trace.active = 0U;
        if (g_observation_kind == 1U)
            SlAlignTrace_Finish(&g_align_trace, s->state == SENSORLESS_START_FAULT ? 2U : 3U);
        else if (g_observation_kind == 2U) {
            SlProbe_EndExternal(&s->probe,g_sensorless_shadow.pwm_counter,
                s->state==SENSORLESS_START_FAULT?s->output.fault_flags:0U);
            SlProbeTrace_Finish(&g_probe_trace,&s->probe,s->state==SENSORLESS_START_FAULT?2U:3U);
            SlProbeDiagnostics_Finish(&g_probe_diagnostics);
        }
        else SlSmoTrace_Finish(&g_smo_trace, s->state == SENSORLESS_START_FAULT ? 2U : 1U);
    }
    g_sensorless_trace.sequence++;
}
/* Called by ToqLoop after PWM update, including neutral PWM on current failure.
 * Never recomputes the PI and never changes a current or protection parameter. */
#ifdef __riscv
__attribute__((section(".ramfunctiontext")))
#endif
void SensorlessShadow_RecordAlignCurrent(float voltage_limit, uint16_t raw_sdm)
{
    SL_ALIGN_SAMPLE p;
    if (g_observation_kind == 2U) { Sensorless_ProbeRecord(); return; }
    if (g_observation_kind != 1U || !g_align_trace.header.active
        || g_sensorless_shadow.startup.state != SENSORLESS_START_ALIGN) return;
    p.tick = g_sensorless_shadow.pwm_counter;
    p.stage_cycles = g_sensorless_shadow.startup.state_cycles;
    p.id_ref = g_sensorless_diag.id_command_pu;
    p.iq_ref = g_sensorless_diag.iq_command_pu;
    p.id_fb = _IQtoF(Park.Ds); p.iq_fb = _IQtoF(Park.Qs);
    p.sdm_u = g_sensorless_chain.sdm_u; p.sdm_v = g_sensorless_chain.sdm_v;
    p.zero_u = g_sensorless_chain.zero_u; p.zero_v = g_sensorless_chain.zero_v;
    p.voltage_d = g_source_foc_current.voltage_d;
    p.voltage_q = g_source_foc_current.voltage_q;
    p.voltage_limit = voltage_limit;
    p.saturated = g_source_foc_current.saturated;
    p.current_failure = g_source_foc_current.failure_reason;
    p.flags = (raw_sdm ? 1U : 0U) | (!p.current_failure ? 2U : 0U)
        | (((uint32_t)g_sensorless_chain.sample_window & 255U) << 8);
    SlAlignTrace_Sample(&g_align_trace, &p);
    if (p.current_failure) SlAlignTrace_Finish(&g_align_trace, 2U);
}
void SensorlessShadow_CurrentFault(uint16_t reason)
{
    uint32_t flag;
    switch (reason) {
    case SOURCE_FOC_OVER_CURRENT: flag = SENSORLESS_FAULT_CURRENT_OVER; break;
    case SOURCE_FOC_CONFIG_INVALID: flag = SENSORLESS_FAULT_CURRENT_CONFIG; break;
    case SOURCE_FOC_INPUT_INVALID: flag = SENSORLESS_FAULT_CURRENT_INPUT; break;
    case SOURCE_FOC_LIMIT_INVALID: flag = SENSORLESS_FAULT_VOLTAGE_LIMIT; break;
    default: flag = SENSORLESS_FAULT_CURRENT_CALC; break;
    }
    Sensorless_TraceRecord(1U); /* preserve pre-trip commands */
    if (g_sensorless_shadow.startup.failure.fault == 0U) {
        SensorlessStartup_Trip(&g_sensorless_shadow.startup, flag);
        g_sensorless_shadow.startup.failure.control_theta_q24 = (int32_t)Etheta.Ethetapk;
    } else SensorlessStartup_Trip(&g_sensorless_shadow.startup, flag);
    g_sensorless_diag.first_fault = g_sensorless_shadow.startup.failure.fault;
    g_sensorless_diag.startup_fault_flags = g_sensorless_shadow.startup.output.fault_flags;
    g_sensorless_diag.fault_request = 1U;
    g_sensorless_diag.current_failure = reason;
    Sensorless_ReleaseControlOutputs();
    Sensorless_TraceRecord(1U);
#if SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U
    Sensorless_GlobalFaultHook(flag);
    g_sensorless_shadow.fault_hook_sent = 1U;
#endif
}

void SensorlessShadow_Init(void)
{
    float initial_theta;

    memset(&g_sensorless_shadow, 0, sizeof(g_sensorless_shadow));
    memset((void *)&g_sensorless_diag, 0, sizeof(g_sensorless_diag));

    g_source_foc_ready = 0U;
    memset(&g_source_foc_current, 0, sizeof(g_source_foc_current));
    g_previous_voltage_alpha = g_previous_voltage_beta = 0.0f;
    RotorFeedbackMux_Init(&g_sensorless_shadow.feedback_mux);
    MotorParamProfile_ClearOverride(&g_sensorless_shadow.motor_override);
    Sensorless_BuildMotorProfile(&g_sensorless_shadow.motor_override);
    Sensorless_InitStartup();
    initial_theta = Sensorless_Q24ToTurns((int32_t)Etheta.Ethetapk);
    SmoPll_Init(&g_sensorless_shadow.observer,
                &g_sensorless_shadow.params,
                initial_theta);
    Sensorless_InitHfi(initial_theta);
    Sensorless_InitIdentification();

    g_sensorless_shadow.voltage_scale = SENSORLESS_DEFAULT_VOLTAGE_SCALE;
    g_sensorless_shadow.expected_direction = 1;
    g_sensorless_shadow.initialized = 1U;

    g_sensorless_diag.initialized = 1U;
    g_sensorless_diag.mode = (uint16_t)g_sensorless_shadow.feedback_mux.mode;
    g_sensorless_diag.config_flags = g_sensorless_shadow.config_flags;
    g_sensorless_diag.profile_valid_for_control =
        g_sensorless_shadow.motor_profile.valid_for_control;
    g_sensorless_diag.motor_validation_flags =
        g_sensorless_shadow.motor_profile.validation_flags;
    g_sensorless_diag.hfi_capable =
        g_sensorless_shadow.motor_profile.hfi_capable;
    g_sensorless_diag.saliency_ratio =
        g_sensorless_shadow.motor_profile.saliency_ratio;
    g_sensorless_diag.model_gain = g_sensorless_shadow.params.current_model_gain;
    g_sensorless_diag.stator_resistance_pu = g_sensorless_shadow.params.stator_resistance_pu;
}

void SensorlessShadow_Reset(void)
{
    SENSORLESS_STARTUP_CONFIG config;
    MOTOR_PARAM_IDENT_CONFIG ident_config;

    if (g_sensorless_shadow.initialized == 0U) {
        return;
    }

    if (g_sensorless_trace.active) {
        if (g_sensorless_shadow.startup.state != SENSORLESS_START_IDLE
            && g_sensorless_shadow.startup.state != SENSORLESS_START_FAULT)
            SensorlessStartup_Trip(&g_sensorless_shadow.startup, SENSORLESS_FAULT_EXTERNAL);
        Sensorless_TraceRecord(1U);
    }
    Sensorless_ReleaseControlOutputs();
    SourceFoc_Reset(&g_source_foc_current);
    g_previous_voltage_alpha = g_previous_voltage_beta = 0.0f;
    SmoPll_Reset(&g_sensorless_shadow.observer,
                 Sensorless_Q24ToTurns((int32_t)Etheta.Ethetapk));
    HfiEstimator_Init(&g_sensorless_shadow.hfi,
        &g_sensorless_shadow.hfi_params,
        Sensorless_Q24ToTurns((int32_t)Etheta.Ethetapk));
    config = g_sensorless_shadow.startup.config;
    {
        SENSORLESS_START_FAILURE saved_failure = g_sensorless_shadow.startup.failure;
        SensorlessStartup_Init(&g_sensorless_shadow.startup, &config);
        g_sensorless_shadow.startup.failure = saved_failure;
    }
    ident_config = g_sensorless_shadow.identification.config;
    MotorParamIdentification_Init(&g_sensorless_shadow.identification,
                                  &ident_config);
    RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux,
                             ROTOR_MODE_SENSORLESS_SHADOW);
    g_sensorless_shadow.pwm_counter = 0U;
    g_sensorless_shadow.encoder_stale_cycles = 0U;
    g_sensorless_shadow.encoder_last_theta_q24 = (int32_t)Etheta.Etheta;
    g_sensorless_shadow.encoder_valid = 0U;
    g_sensorless_shadow.hfi_was_required = 0U;
    g_sensorless_shadow.expected_direction = 1;
    g_sensorless_shadow.fault_hook_sent = 0U;
}

void SensorlessShadow_SetMode(ROTOR_MODE mode)
{
    if (g_sensorless_shadow.initialized == 0U || StateMachine.RegulFlg != 0U) {
        return;
    }
    mode = ROTOR_MODE_SENSORLESS_SHADOW; /* external encoder/fallback selection disabled */

#if SENSORLESS_CONTROL_OUTPUT_ENABLE == 0U
    if ((mode != ROTOR_MODE_ENCODER) && (mode != ROTOR_MODE_SENSORLESS_SHADOW)) {
        mode = ROTOR_MODE_SENSORLESS_SHADOW;
    }
#endif

    RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux, mode);
}

uint16_t SensorlessShadow_ApplySourceRecipe(const SOURCE_FOC_RECIPE *recipe)
{
    SENSORLESS_MOTOR_PROFILE profile;
    SOURCE_FOC_CURRENT current;
    SMO_PLL_PARAMS smo;
    SENSORLESS_STARTUP_CONFIG startup;
    if (!recipe || !g_sensorless_shadow.initialized
        || StateMachine.RegulFlg != 0U
        || !SensorlessStartup_IsIdle(&g_sensorless_shadow.startup)
        || MotorParamIdentification_IsBusy(&g_sensorless_shadow.identification)
        || !SourceFoc_BuildProfile(recipe, (float)DrvCoeff.TpwmFrq,
                                  (float)DrvCoeff.SpdBase, &profile)
        || !SourceFoc_Configure(&current, &profile, recipe->current_bandwidth_hz))
        return 0U;
    MotorParamProfile_ApplyToSmo(&profile, &smo);
    if (!SmoPll_ValidateParams(&smo)) return 0U;
    SensorlessStartup_DefaultConfig(&startup, 1.0f / profile.effective.sample_hz,
        profile.effective.pole_pairs, profile.effective.speed_base_rpm,
        profile.effective.max_speed_rpm, current.current_limit_pu);
    /* Without rotation permission only bounded alignment may start. */
    startup.debug_mode = recipe->allow_continuous_rotation
        ? SENSORLESS_DEBUG_IF : SENSORLESS_DEBUG_ALIGN;
    startup.hfi_enable = 0U;
    startup.allow_open_loop_fallback = 1U;
    startup.iq_limit_pu = current.current_limit_pu;
    startup.align_current_pu = 0.4f * current.current_limit_pu;
    startup.startup_iq_pu = 0.5f * current.current_limit_pu;
    if (!SensorlessStartup_ValidateConfig(&startup)
        || startup.handoff_speed_rpm > profile.effective.max_speed_rpm)
        return 0U;
    g_source_foc_ready = 0U;
    g_previous_voltage_alpha = g_previous_voltage_beta = 0.0f;
    g_source_foc_recipe = *recipe;
    g_source_foc_current = current;
    g_sensorless_shadow.motor_profile = profile;
    MotorParamProfile_ClearOverride(&g_sensorless_shadow.motor_override);
    g_sensorless_shadow.params = smo;
    g_sensorless_shadow.config_flags = profile.validation_flags;
    {
        SENSORLESS_START_FAILURE saved = g_sensorless_shadow.startup.failure;
        SensorlessStartup_Init(&g_sensorless_shadow.startup, &startup);
        g_sensorless_shadow.startup.failure = saved;
    }
    SmoPll_Init(&g_sensorless_shadow.observer, &g_sensorless_shadow.params, 0.0f);
    Sensorless_InitHfi(0.0f);
    Sensorless_InitIdentification();
    g_source_foc_ready = 1U;
    return 1U;
}

uint16_t SensorlessShadow_ConfigureMotorOverride(
    const SENSORLESS_MOTOR_OVERRIDE *override_values)
{
    /* Legacy pu writes cannot leave the FOC, observer and recipe inconsistent. */
    (void)override_values;
    return 0U;
}

uint16_t SensorlessShadow_ConfigureStartup(
    const SENSORLESS_STARTUP_CONFIG *config)
{
    SENSORLESS_STARTUP_CONFIG candidate;
    float current_limit;

    if ((config == 0) || (g_sensorless_shadow.initialized == 0U)
        || (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) == 0U)
        || (StateMachine.RegulFlg != 0U)) {
        return 0U;
    }

    if (config->hfi_enable != 0U || config->allow_open_loop_fallback != 1U)
        return 0U;
    candidate = *config;
    candidate.sample_time_s = g_sensorless_shadow.params.sample_time_s;
    candidate.pole_pairs =
        g_sensorless_shadow.motor_profile.effective.pole_pairs;
    candidate.speed_base_rpm =
        g_sensorless_shadow.motor_profile.effective.speed_base_rpm;
    candidate.max_speed_rpm =
        g_sensorless_shadow.motor_profile.effective.max_speed_rpm;
    current_limit = g_sensorless_shadow.motor_profile.effective.current_limit_pu;
    if (g_sensorless_shadow.motor_profile.effective.rated_current_pu
        < current_limit) {
        current_limit =
            g_sensorless_shadow.motor_profile.effective.rated_current_pu;
    }
    if ((candidate.iq_limit_pu > current_limit)
        || (SensorlessStartup_ValidateConfig(&candidate) == 0U)) {
        return 0U;
    }

    {
        SENSORLESS_START_FAILURE saved = g_sensorless_shadow.startup.failure;
        SensorlessStartup_Init(&g_sensorless_shadow.startup, &candidate);
        g_sensorless_shadow.startup.failure = saved;
    }
    return 1U;
}

uint16_t SensorlessShadow_ConfigureHfi(const HFI_ESTIMATOR_PARAMS *params)
{
    (void)params;
    return 0U; /* fixed I/F -> SMO build */
}

uint16_t SensorlessShadow_ConfigureSmo(
    const SMO_PLL_PARAMS *params)
{
    float initial_theta;

    if ((params == 0) || (g_sensorless_shadow.initialized == 0U)
        || (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) == 0U)
        || (StateMachine.RegulFlg != 0U)
        || (SmoPll_ValidateParams(params) == 0U)) {
        return 0U;
    }

    if (params->sample_time_s != g_sensorless_shadow.params.sample_time_s
        || params->stator_resistance_pu != g_sensorless_shadow.params.stator_resistance_pu
        || params->current_model_gain != g_sensorless_shadow.params.current_model_gain)
        return 0U;
    g_sensorless_shadow.params = *params;
    initial_theta = Sensorless_Q24ToTurns(
        g_sensorless_diag.observer_theta_q24);
    SmoPll_Init(&g_sensorless_shadow.observer,
        &g_sensorless_shadow.params, initial_theta);
    return 1U;
}

uint16_t SensorlessShadow_StartParamIdentification(const MOTOR_PARAM_IDENT_CONFIG *config)
{
    (void)config;
    return 0U; /* identification requires a separately validated release */
}

void SensorlessShadow_AbortParamIdentification(void)
{
    if (MotorParamIdentification_IsBusy(
            &g_sensorless_shadow.identification) != 0U) {
        MotorParamIdentification_Abort(
            &g_sensorless_shadow.identification,
            MOTOR_IDENT_FAULT_DRIVE);
        SensorlessStartup_RequestStop(&g_sensorless_shadow.startup);
    }
}

uint16_t SensorlessShadow_ApplyIdentifiedMotor(void)
{
    SENSORLESS_MOTOR_OVERRIDE identified =
        g_sensorless_shadow.motor_override;
    MOTOR_PARAM_IDENT_RESULT result =
        g_sensorless_shadow.identification.result;

    if ((result.valid == 0U)
        || (StateMachine.RegulFlg != 0U)
        || (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) == 0U)) {
        return 0U;
    }
    identified.mask |= MOTOR_PARAM_OVERRIDE_RS
        | MOTOR_PARAM_OVERRIDE_LD | MOTOR_PARAM_OVERRIDE_LQ;
    identified.stator_resistance_pu = result.rs_pu;
    identified.ld_pu = result.ld_pu;
    identified.lq_pu = result.lq_pu;
    if (SensorlessShadow_ConfigureMotorOverride(&identified) == 0U) {
        return 0U;
    }
    g_sensorless_shadow.identification.result = result;
    g_sensorless_shadow.identification.state = MOTOR_IDENT_COMPLETE;
    return 1U;
}

uint16_t SensorlessShadow_RequestStart(int16_t direction, int32_t target_speed_rpm)
{
    /* Alignment may move to one fixed angle; rotating modes require separate
       permission even when a caller changes debug mode after recipe apply. */
    if (!g_source_foc_recipe.allow_continuous_rotation &&
        g_sensorless_shadow.startup.config.debug_mode != SENSORLESS_DEBUG_ALIGN)
        return 0U;
#ifdef SENSORLESS_CANOPEN_BUILD
    if (!SensorlessCanopen_MotionAllowed()) return 0U;
#endif
#if SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U
    if ((g_source_foc_ready == 0U)
        || (g_sensorless_shadow.initialized == 0U)
        || (g_sensorless_shadow.motor_profile.valid_for_control == 0U)
        || (MotorParamIdentification_IsBusy(
                &g_sensorless_shadow.identification) != 0U)
        || ((g_sensorless_shadow.startup.config.hfi_enable != 0U)
            && (g_sensorless_shadow.motor_profile.hfi_capable == 0U)
            && (g_sensorless_shadow.startup.config.allow_open_loop_fallback == 0U))
        || (StateMachine.RegulFlg == 0U)
        || ((FaultP.FaultStatus & FP_ERR) != 0U)) {
        return 0U;
    }
    if (!SensorlessStartup_IsIdle(&g_sensorless_shadow.startup)
        || g_sensorless_shadow.startup.start_requested
        || !SensorlessStartup_ValidateConfig(&g_sensorless_shadow.startup.config)
        || !SmoPll_ValidateParams(&g_sensorless_shadow.params))
        return 0U;
    /* Repeated starts must not inherit a previous lock or PI integrator. */
    SourceFoc_Reset(&g_source_foc_current);
    SmoPll_Reset(&g_sensorless_shadow.observer,
        g_sensorless_shadow.startup.config.align_angle_turns);
    memset(&g_sensorless_shadow.feedback_mux.sensorless, 0,
        sizeof(g_sensorless_shadow.feedback_mux.sensorless));
    g_previous_voltage_alpha = g_previous_voltage_beta = 0.0f;
    if (!SensorlessStartup_RequestStart(&g_sensorless_shadow.startup, direction, target_speed_rpm))
        return 0U;
    g_sensorless_shadow.fault_hook_sent = 0U;
    g_sensorless_fault_detail = 0U;
    g_source_foc_current.failure_reason = SOURCE_FOC_OK;
    Sensorless_TraceBegin();
    return 1U;
#else
    (void)direction;
    (void)target_speed_rpm;
    return 0U;
#endif
}

uint16_t SensorlessShadow_UpdateTarget(int16_t direction,
                                       int32_t target_speed_rpm)
{
    return SensorlessStartup_UpdateTarget(&g_sensorless_shadow.startup,
                                          direction, target_speed_rpm);
}

void SensorlessShadow_RequestStop(void)
{
    SensorlessStartup_RequestStop(&g_sensorless_shadow.startup);
}

void SensorlessShadow_ClearFault(void)
{
    if (StateMachine.RegulFlg == 0U) {
        SensorlessStartup_ClearFault(&g_sensorless_shadow.startup);
        memset(&g_sensorless_shadow.startup.failure, 0, sizeof(g_sensorless_shadow.startup.failure));
        g_sensorless_fault_detail = 0U;
        SensorlessShadow_ClearTrace();
        g_sensorless_shadow.fault_hook_sent = 0U;
    }
}

void SensorlessShadow_PreFoc(void)
{
    SENSORLESS_STARTUP_INPUT startup_input;
    ROTOR_FEEDBACK open_loop_feedback;
    const ROTOR_FEEDBACK *selected;
    const SENSORLESS_STARTUP_OUTPUT *output;
    const MOTOR_PARAM_IDENT_OUTPUT *ident_output;
    uint16_t hfi_required;
    uint16_t control_active;
    uint16_t angle_override;
    uint16_t current_override;
    uint16_t control_source;
    int32_t control_theta_q24;
    float hfi_speed_rpm;
    float id_command;
    float iq_command;
    float speed_command_rpm;

    if (g_sensorless_shadow.initialized == 0U) {
        return;
    }

    if (StateMachine.RegulFlg == 0U) {
        if (g_sensorless_trace.active
            && g_sensorless_shadow.startup.state != SENSORLESS_START_IDLE
            && g_sensorless_shadow.startup.state != SENSORLESS_START_FAULT) {
            SensorlessStartup_Trip(&g_sensorless_shadow.startup, SENSORLESS_FAULT_EXTERNAL);
            Sensorless_TraceRecord(1U);
        }
        Sensorless_ReleaseControlOutputs();
        return;
    }
    memset(&startup_input, 0, sizeof(startup_input));
    startup_input.drive_enabled = (StateMachine.RegulFlg != 0U) ? 1U : 0U;
    startup_input.params_valid = g_sensorless_shadow.motor_profile.valid_for_control;
    startup_input.observer_locked = g_sensorless_shadow.observer.locked;
    startup_input.observer_quality = g_sensorless_shadow.observer.quality;
    startup_input.observer_fault_count =
        g_sensorless_shadow.observer.numeric_fault_count;
    startup_input.observer_theta_q24 =
        g_sensorless_shadow.feedback_mux.sensorless.theta_e_q24;
    startup_input.observer_speed_rpm =
        g_sensorless_shadow.feedback_mux.sensorless.speed_rpm;
    hfi_speed_rpm = Sensorless_ElectricalToMechanicalRpm(
        g_sensorless_shadow.hfi.omega_e_rad_s);
    startup_input.hfi_capable =
        g_sensorless_shadow.motor_profile.hfi_capable;
    startup_input.hfi_locked = g_sensorless_shadow.hfi.locked;
    startup_input.hfi_quality = g_sensorless_shadow.hfi.quality;
    startup_input.hfi_fault_count =
        g_sensorless_shadow.hfi.numeric_fault_count;
    startup_input.hfi_theta_q24 = Sensorless_TurnsToQ24(
        g_sensorless_shadow.hfi.theta_turns);
    startup_input.hfi_speed_rpm = (int32_t)hfi_speed_rpm;
    startup_input.encoder_valid = g_sensorless_shadow.feedback_mux.encoder.valid;
    startup_input.encoder_theta_q24 =
        g_sensorless_shadow.feedback_mux.encoder.theta_e_q24;
    startup_input.encoder_speed_rpm =
        g_sensorless_shadow.feedback_mux.encoder.speed_rpm;

    startup_input.encoder_valid = 0U;
    if (SENSORLESS_IS_PROBE(g_sensorless_shadow.startup.config.debug_mode))
        Sensorless_ProbeInput(&startup_input.probe);
    {
        uint32_t previous_fault = g_sensorless_shadow.startup.failure.fault;
        SensorlessStartup_Step(&g_sensorless_shadow.startup, &startup_input);
        if (!previous_fault && g_sensorless_shadow.startup.failure.fault)
            g_sensorless_shadow.startup.failure.control_theta_q24 = (int32_t)Etheta.Ethetapk;
    }
    if (g_observation_kind == 1U && g_align_trace.header.active
        && g_align_trace.header.total_samples
        && g_sensorless_shadow.startup.state != SENSORLESS_START_ALIGN) {
        SlAlignTrace_Finish(&g_align_trace,
            g_sensorless_shadow.startup.state == SENSORLESS_START_FAULT ? 2U :
            (g_sensorless_shadow.startup.state == SENSORLESS_START_IDLE ? 3U : 1U));
    }
    if (g_sensorless_shadow.startup.state == SENSORLESS_START_FAULT)
        Sensorless_TraceRecord(1U);
    output = &g_sensorless_shadow.startup.output;
    ident_output = &g_sensorless_shadow.identification.output;

    if ((ident_output->request_start != 0U)
        && (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) != 0U)) {
        uint16_t accepted = SensorlessStartup_RequestStart(
            &g_sensorless_shadow.startup,
            ident_output->start_direction,
            ident_output->start_speed_rpm);
        MotorParamIdentification_AcknowledgeStart(
            &g_sensorless_shadow.identification, accepted);
    }
    if (ident_output->request_stop != 0U) {
        SensorlessStartup_RequestStop(&g_sensorless_shadow.startup);
    }

    if (ident_output->control_active != 0U) {
        control_active = 1U;
        angle_override = ident_output->angle_override;
        current_override = ident_output->current_override;
        control_theta_q24 = Sensorless_TurnsToQ24(ident_output->theta_turns);
        id_command = ident_output->id_ref_pu;
        iq_command = ident_output->iq_ref_pu;
        speed_command_rpm = 0.0f;
        control_source = (ident_output->requires_hfi != 0U)
            ? ROTOR_SOURCE_HFI : ROTOR_SOURCE_OPEN_LOOP;
    } else {
        control_active = output->control_active;
        angle_override = output->angle_override;
        current_override = output->current_override;
        control_theta_q24 = output->theta_e_q24;
        id_command = output->id_ref_pu;
        iq_command = output->iq_ref_pu;
        speed_command_rpm = output->speed_ref_rpm;
        control_source = output->source;
    }

    hfi_required = Sensorless_HfiRequired();
    if ((hfi_required != 0U)
        && (g_sensorless_shadow.hfi.active == 0U)) {
        HfiEstimator_Enable(&g_sensorless_shadow.hfi,
            &g_sensorless_shadow.hfi_params,
            Sensorless_Q24ToTurns(control_theta_q24));
    } else if ((hfi_required == 0U)
        && (g_sensorless_shadow.hfi.active != 0U)) {
        HfiEstimator_Disable(&g_sensorless_shadow.hfi);
    }
    g_sensorless_shadow.hfi_was_required = hfi_required;
    id_command += HfiEstimator_GetInjectionCurrent(
        &g_sensorless_shadow.hfi)
        + HfiEstimator_GetPoleDetectCurrent(&g_sensorless_shadow.hfi);
    id_command = Sensorless_Clamp(id_command,
        -g_sensorless_shadow.startup.config.iq_limit_pu,
         g_sensorless_shadow.startup.config.iq_limit_pu);

    if (control_active != 0U) {
        memset(&open_loop_feedback, 0, sizeof(open_loop_feedback));
        open_loop_feedback.theta_e_q24 = control_theta_q24;
        open_loop_feedback.speed_rpm = (int32_t)speed_command_rpm;
        open_loop_feedback.direction = g_sensorless_shadow.startup.direction;
        open_loop_feedback.valid = 1U;
        open_loop_feedback.quality = 1000U;
        open_loop_feedback.timestamp = g_sensorless_shadow.pwm_counter;
        RotorFeedbackMux_UpdateOpenLoop(&g_sensorless_shadow.feedback_mux,
                                        &open_loop_feedback);
    }

    if (g_sensorless_shadow.startup.state == SENSORLESS_START_RUN) {
        RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux,
                                 ROTOR_MODE_SENSORLESS_ONLY);
    } else if (g_sensorless_shadow.startup.state
               == SENSORLESS_START_ENCODER_FALLBACK) {
        RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux,
                                 ROTOR_MODE_ENCODER);
    } else if (control_active != 0U) {
        RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux,
                                 ROTOR_MODE_OPEN_LOOP);
    } else {
        RotorFeedbackMux_SetMode(&g_sensorless_shadow.feedback_mux,
                                 ROTOR_MODE_SENSORLESS_SHADOW);
    }

    selected = RotorFeedbackMux_Select(&g_sensorless_shadow.feedback_mux);
    g_sensorless_diag.mode = (uint16_t)g_sensorless_shadow.feedback_mux.mode;
    g_sensorless_diag.control_source = (control_active != 0U)
        ? control_source
        : ((selected != 0) ? selected->source : ROTOR_SOURCE_NONE);
    g_sensorless_diag.control_override_active = 0U;
    g_sensorless_diag.startup_state = (uint16_t)g_sensorless_shadow.startup.state;
    g_sensorless_diag.fault_request = output->fault_request;
    g_sensorless_diag.startup_fault_flags = output->fault_flags;
    g_sensorless_diag.hfi_capable =
        g_sensorless_shadow.motor_profile.hfi_capable;
    g_sensorless_diag.hfi_active = g_sensorless_shadow.hfi.active;
    g_sensorless_diag.hfi_locked = g_sensorless_shadow.hfi.locked;
    g_sensorless_diag.hfi_quality = g_sensorless_shadow.hfi.quality;
    g_sensorless_diag.id_command_pu = id_command;
    g_sensorless_diag.iq_command_pu = iq_command;
    g_sensorless_diag.speed_command_rpm = speed_command_rpm;

#if SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U
    if (output->fault_request && !g_sensorless_shadow.fault_hook_sent) {
        Sensorless_GlobalFaultHook(output->fault_flags);
        g_sensorless_shadow.fault_hook_sent = 1U;
    }
    if (control_active != 0U) {
        if (angle_override != 0U) {
            Etheta.EtaInerGet = control_theta_q24;
            Etheta.EtaGetSwt = 1U;
        }
        if (current_override != 0U) {
            IdRef.IdRefInner = _IQ(id_command);
            IqRef.IqRefInner = _IQ(iq_command);
            IdRef.IdRefGet = 1U;
            IqRef.IqRefGet = 1U;
        }
        g_sensorless_shadow.output_owned = 1U;
        g_sensorless_diag.control_override_active = 1U;
    } else {
        Sensorless_ReleaseControlOutputs();
    }
#else
    Sensorless_ReleaseControlOutputs();
#endif
}

void SensorlessShadow_PostFoc(void)
{
    ROTOR_FEEDBACK encoder_feedback;
    ROTOR_FEEDBACK sensorless_feedback;
    SMO_PLL_INPUT input;
    MOTOR_PARAM_IDENT_INPUT ident_input;
    float encoder_speed_pu;
    float encoder_speed_rpm;
    float observer_speed_rpm;
    float hfi_speed_rpm;
    float current_alpha_pu;
    float current_beta_pu;
    float pole_pairs;
    int32_t encoder_theta_q24;
    int32_t encoder_delta_q24;
    int32_t monitor_speed_rpm;
    uint32_t encoder_stale_limit;

    if (g_sensorless_shadow.initialized == 0U) {
        return;
    }

    if (StateMachine.RegulFlg == 0U) {
        if (g_observation_kind == 1U) SlAlignTrace_Finish(&g_align_trace, 3U);
        else if (g_observation_kind == 2U) {
            SlProbe_EndExternal(&g_sensorless_shadow.startup.probe,g_sensorless_shadow.pwm_counter,
                g_sensorless_shadow.startup.state==SENSORLESS_START_FAULT
                    ?g_sensorless_shadow.startup.output.fault_flags:0U);
            SlProbeTrace_Finish(&g_probe_trace,&g_sensorless_shadow.startup.probe,3U);
            SlProbeDiagnostics_Finish(&g_probe_diagnostics);
        }
        else SlSmoTrace_Finish(&g_smo_trace, 3U);
        g_previous_voltage_alpha = g_previous_voltage_beta = 0.0f;
        return;
    }
    g_sensorless_shadow.pwm_counter++;
    encoder_speed_pu = _IQtoF(SpdFb.Fb);
    encoder_speed_rpm = encoder_speed_pu * (float)DrvCoeff.SpdBase;

    if (SensorlessStartup_IsIdle(&g_sensorless_shadow.startup) == 0U) {
        g_sensorless_shadow.expected_direction = g_sensorless_shadow.startup.direction;
    } else if (encoder_speed_pu > SENSORLESS_SPEED_DEADBAND) {
        g_sensorless_shadow.expected_direction = 1;
    } else if (encoder_speed_pu < -SENSORLESS_SPEED_DEADBAND) {
        g_sensorless_shadow.expected_direction = -1;
    }

    encoder_theta_q24 = RotorFeedback_WrapAngleQ24((int32_t)Etheta.Etheta);
    encoder_delta_q24 = RotorFeedback_AngleErrorQ24(
        encoder_theta_q24, g_sensorless_shadow.encoder_last_theta_q24);
    if (encoder_delta_q24 != 0) {
        g_sensorless_shadow.encoder_stale_cycles = 0U;
    } else if (g_sensorless_shadow.encoder_stale_cycles < 0xffffffffUL) {
        g_sensorless_shadow.encoder_stale_cycles++;
    }
    g_sensorless_shadow.encoder_last_theta_q24 = encoder_theta_q24;
    g_sensorless_shadow.encoder_valid = (uint16_t)
        (((Etheta.ZEncAlmFlg == 0U) && (Etheta.ABEncAlmFlg == 0U)
        && ((FaultP.FaultStatus & FP_ERR) == 0U)) ? 1U : 0U);
    encoder_stale_limit = (uint32_t)(SENSORLESS_ENCODER_STALE_SECONDS
        / g_sensorless_shadow.params.sample_time_s);
    monitor_speed_rpm = g_sensorless_shadow.feedback_mux.sensorless.speed_rpm;
    if (monitor_speed_rpm < 0) {
        monitor_speed_rpm = -monitor_speed_rpm;
    }
    if ((monitor_speed_rpm >= SENSORLESS_ENCODER_ACTIVE_RPM)
        && (g_sensorless_shadow.encoder_stale_cycles >= encoder_stale_limit)) {
        g_sensorless_shadow.encoder_valid = 0U;
    }

    /* Acquisition is compiled out: never relabel estimated speed as encoder. */
    g_sensorless_shadow.encoder_valid = 0U;
    encoder_theta_q24 = 0; encoder_speed_rpm = 0.0f;
    memset(&encoder_feedback, 0, sizeof(encoder_feedback));
    encoder_feedback.theta_e_q24 = encoder_theta_q24;
    encoder_feedback.omega_e_q24 = (int32_t)SpdFb.Wfb;
    encoder_feedback.speed_rpm = (int32_t)encoder_speed_rpm;
    encoder_feedback.valid = g_sensorless_shadow.encoder_valid;
    encoder_feedback.quality = (g_sensorless_shadow.encoder_valid != 0U)
        ? 1000U : 0U;
    encoder_feedback.direction = g_sensorless_shadow.expected_direction;
    encoder_feedback.timestamp = g_sensorless_shadow.pwm_counter;
    RotorFeedbackMux_UpdateEncoder(&g_sensorless_shadow.feedback_mux, &encoder_feedback);

    /* Pair i[k] with the preceding nominal applied PWM interval.
     * Hardware latch delay and dead-time errors require board verification. */
    input.voltage_alpha_pu = g_previous_voltage_alpha;
    input.voltage_beta_pu = g_previous_voltage_beta;
    {
        float a = _IQtoF(Svpwm.Va), b = _IQtoF(Svpwm.Vb);
        float c = _IQtoF(Svpwm.Vc);
        float bus_ratio = (DrvCoeff.UdcEpu > 0)
            ? _IQtoF(AdRead.Udc) / _IQtoF(DrvCoeff.UdcEpu) : 0.0f;
        if (StateMachine.RegulFlg == 0U) bus_ratio = 0.0f;
        g_previous_voltage_alpha = (2.0f*a-b-c) / 3.0f * bus_ratio;
        g_previous_voltage_beta = (b-c) * 0.57735026919f * bus_ratio;
    }
    current_alpha_pu = _IQtoF(Clark.Alpha);
    current_beta_pu = _IQtoF(Clark.Beta);
    if (g_source_foc_ready && StateMachine.RegulFlg == 1U) {
        ProductionMetrics_Sample(current_alpha_pu * g_source_foc_recipe.current_base_a,
            current_beta_pu * g_source_foc_recipe.current_base_a,
            (float)g_sensorless_diag.observer_speed_rpm,
            _IQtoF(AdRead.Udc) * (float)DrvCoeff.UdcBase);
    }
    input.current_alpha_pu = current_alpha_pu;
    input.current_beta_pu = current_beta_pu;
    input.expected_direction = g_sensorless_shadow.expected_direction;

    HfiEstimator_Step(&g_sensorless_shadow.hfi,
        &g_sensorless_shadow.hfi_params,
        current_alpha_pu, current_beta_pu);
    if ((g_sensorless_shadow.hfi.active != 0U)
        && (g_sensorless_shadow.hfi.demod_update_counter != 0U)) {
        input.current_alpha_pu =
            g_sensorless_shadow.hfi.base_current_alpha_pu;
        input.current_beta_pu =
            g_sensorless_shadow.hfi.base_current_beta_pu;
    }
    if ((g_sensorless_shadow.hfi.locked != 0U)
        && (g_sensorless_shadow.observer.emf_magnitude_pu
            < g_sensorless_shadow.params.min_emf_pu)) {
        SmoPll_SeedTracking(&g_sensorless_shadow.observer,
            g_sensorless_shadow.hfi.theta_turns,
            g_sensorless_shadow.hfi.omega_e_rad_s);
    }
    SmoPll_Step(&g_sensorless_shadow.observer,
        &g_sensorless_shadow.params, &input);
    if (g_observation_kind == 0U && g_smo_trace.header.active) {
        SL_SMO_TRACE_CONTEXT c={0};
        c.tick = g_sensorless_shadow.pwm_counter;
        c.stage_cycles = g_sensorless_shadow.startup.state_cycles;
        c.state = (uint32_t)g_sensorless_shadow.startup.state;
        c.direction = input.expected_direction;
        c.chain_tick = (uint32_t)g_sensorless_chain.chain_tick;
        c.chain_valid = g_sensorless_chain.chain_valid
            && c.chain_tick + 1U == c.tick;
        c.sample_window = (uint32_t)g_sensorless_chain.sample_window;
        c.pwm_u = (uint32_t)g_sensorless_chain.pwm_u;
        c.pwm_v = (uint32_t)g_sensorless_chain.pwm_v;
        c.pwm_w = (uint32_t)g_sensorless_chain.pwm_w;
        c.pwm_period = (uint32_t)g_sensorless_chain.pwm_period;
        if (SlSmoTrace_WantsSample(&g_smo_trace,c.state,c.stage_cycles)) {
            c.encoder_sequence=c.encoder_raw=c.encoder_tick_ms=0U;
            c.encoder_request_tick=c.encoder_read_tick=c.encoder_valid=0U;
#ifdef SENSORLESS_CANOPEN_BUILD
            {
                uint32_t ts=g_sl_encoder_timing.sequence;
                c.encoder_sequence=g_sl_encoder.sequence;
                if (!(ts&1U) && !(c.encoder_sequence&1U)) {
                    c.encoder_raw=g_sl_encoder.motor_raw;
                    c.encoder_tick_ms=g_sl_encoder.tick_ms;
                    c.encoder_request_tick=g_sl_encoder_timing.frame_request_tick;
                    c.encoder_read_tick=g_sl_encoder_timing.frame_read_tick;
                    c.encoder_valid=g_sl_encoder.initialized && g_sl_encoder.valid
                        && g_sl_encoder.age_ms==0U
                        && c.encoder_tick_ms==g_sl_encoder_timing.frame_tick_ms;
                    if (ts!=g_sl_encoder_timing.sequence
                        || c.encoder_sequence!=g_sl_encoder.sequence) c.encoder_valid=0U;
                }
            }
#endif
            c.forced_theta_q24=Sensorless_TurnsToQ24(
                g_sensorless_shadow.startup.open_loop_theta_turns);
            c.control_theta_q24=(int32_t)Etheta.Ethetapk;
            c.id_ref=g_sensorless_diag.id_command_pu;
            c.iq_ref=g_sensorless_diag.iq_command_pu;
            c.id_fb=_IQtoF(Park.Ds);c.iq_fb=_IQtoF(Park.Qs);
            c.ud=g_source_foc_current.voltage_d;c.uq=g_source_foc_current.voltage_q;
            c.saturated=g_source_foc_current.saturated;
            c.current_failure=g_source_foc_current.failure_reason;
        }
        SlSmoTrace_Sample(&g_smo_trace, &input, &g_sensorless_shadow.observer, &c);
    }

    pole_pairs = g_sensorless_shadow.motor_profile.effective.pole_pairs;
    observer_speed_rpm = g_sensorless_shadow.observer.omega_e_rad_s
                       * 60.0f / (SENSORLESS_TWO_PI_F * pole_pairs);
    hfi_speed_rpm = Sensorless_ElectricalToMechanicalRpm(
        g_sensorless_shadow.hfi.omega_e_rad_s);
    if (g_source_foc_ready) {
        /* Existing application speed monitors consume this normalized speed. */
        SpdFb.Fb = _IQ(observer_speed_rpm /
            g_sensorless_shadow.motor_profile.effective.speed_base_rpm);
        g_sensorless_shadow.encoder_valid = 0U;
    }

    memset(&ident_input, 0, sizeof(ident_input));
    ident_input.drive_enabled = (StateMachine.RegulFlg != 0U) ? 1U : 0U;
    ident_input.fault_active =
        ((FaultP.FaultStatus & FP_ERR) != 0U) ? 1U : 0U;
    ident_input.hfi_locked = g_sensorless_shadow.hfi.locked;
    ident_input.hfi_quality = g_sensorless_shadow.hfi.quality;
    ident_input.hfi_theta_turns = g_sensorless_shadow.hfi.theta_turns;
    ident_input.control_theta_turns = Sensorless_Q24ToTurns(
        g_sensorless_shadow.startup.output.theta_e_q24);
    ident_input.startup_idle = SensorlessStartup_IsIdle(
        &g_sensorless_shadow.startup);
    ident_input.startup_running = (uint16_t)
        (((g_sensorless_shadow.startup.state == SENSORLESS_START_RUN)
        || (g_sensorless_shadow.startup.state == SENSORLESS_START_HFI_RUN))
        ? 1U : 0U);
    if (g_sensorless_shadow.startup.state == SENSORLESS_START_HFI_RUN) {
        ident_input.speed_rpm = hfi_speed_rpm;
        ident_input.feedback_valid = g_sensorless_shadow.hfi.locked;
    } else if (g_sensorless_shadow.startup.state == SENSORLESS_START_RUN) {
        ident_input.speed_rpm = observer_speed_rpm;
        ident_input.feedback_valid = g_sensorless_shadow.observer.locked;
    } else {
        ident_input.speed_rpm = hfi_speed_rpm;
        ident_input.feedback_valid = g_sensorless_shadow.hfi.locked;
    }
    ident_input.voltage_alpha_pu = input.voltage_alpha_pu;
    ident_input.voltage_beta_pu = input.voltage_beta_pu;
    ident_input.current_alpha_pu = current_alpha_pu;
    ident_input.current_beta_pu = current_beta_pu;
    MotorParamIdentification_Step(&g_sensorless_shadow.identification,
                                  &ident_input);

    memset(&sensorless_feedback, 0, sizeof(sensorless_feedback));
    sensorless_feedback.theta_e_q24 = Sensorless_TurnsToQ24(
        g_sensorless_shadow.observer.theta_pll_turns);
    sensorless_feedback.omega_e_q24 =
        (g_sensorless_shadow.motor_profile.effective.speed_base_rpm > 0.0f)
        ? _IQ(observer_speed_rpm
            / g_sensorless_shadow.motor_profile.effective.speed_base_rpm) : 0;
    sensorless_feedback.speed_rpm = (int32_t)observer_speed_rpm;
    sensorless_feedback.valid = g_sensorless_shadow.observer.locked;
    sensorless_feedback.quality = g_sensorless_shadow.observer.quality;
    sensorless_feedback.direction = g_sensorless_shadow.expected_direction;
    sensorless_feedback.timestamp = g_sensorless_shadow.pwm_counter;
    sensorless_feedback.fault = g_sensorless_shadow.observer.numeric_fault_count;
    RotorFeedbackMux_UpdateSensorless(&g_sensorless_shadow.feedback_mux, &sensorless_feedback);

    g_sensorless_diag.observer_locked = g_sensorless_shadow.observer.locked;
    g_sensorless_diag.observer_quality = g_sensorless_shadow.observer.quality;
    g_sensorless_diag.hfi_capable =
        g_sensorless_shadow.motor_profile.hfi_capable;
    g_sensorless_diag.hfi_active = g_sensorless_shadow.hfi.active;
    g_sensorless_diag.hfi_locked = g_sensorless_shadow.hfi.locked;
    g_sensorless_diag.hfi_quality = g_sensorless_shadow.hfi.quality;
    g_sensorless_diag.encoder_valid = g_sensorless_shadow.encoder_valid;
    g_sensorless_diag.encoder_stale_cycles =
        g_sensorless_shadow.encoder_stale_cycles;
    g_sensorless_diag.encoder_good_cycles =
        g_sensorless_shadow.startup.encoder_good_cycles;
    g_sensorless_diag.expected_direction = g_sensorless_shadow.expected_direction;
    g_sensorless_diag.config_flags = g_sensorless_shadow.config_flags;
    g_sensorless_diag.profile_valid_for_control =
        g_sensorless_shadow.motor_profile.valid_for_control;
    g_sensorless_diag.motor_validation_flags =
        g_sensorless_shadow.motor_profile.validation_flags;
    g_sensorless_diag.startup_state = (uint16_t)g_sensorless_shadow.startup.state;
    g_sensorless_diag.startup_fault_flags =
        g_sensorless_shadow.startup.output.fault_flags;
    g_sensorless_diag.pwm_counter = g_sensorless_shadow.pwm_counter;
    g_sensorless_diag.encoder_theta_q24 = encoder_feedback.theta_e_q24;
    g_sensorless_diag.observer_theta_q24 = sensorless_feedback.theta_e_q24;
    g_sensorless_diag.theta_error_q24 = RotorFeedback_AngleErrorQ24(
        sensorless_feedback.theta_e_q24, encoder_feedback.theta_e_q24);
    g_sensorless_diag.encoder_speed_rpm = encoder_feedback.speed_rpm;
    g_sensorless_diag.observer_speed_rpm = sensorless_feedback.speed_rpm;
    g_sensorless_diag.hfi_theta_q24 = Sensorless_TurnsToQ24(
        g_sensorless_shadow.hfi.theta_turns);
    g_sensorless_diag.hfi_speed_rpm = (int32_t)hfi_speed_rpm;
    g_sensorless_diag.hfi_injection_current_pu =
        HfiEstimator_GetInjectionCurrent(&g_sensorless_shadow.hfi);
    g_sensorless_diag.hfi_d_response_pu =
        g_sensorless_shadow.hfi.d_response_pu;
    g_sensorless_diag.hfi_q_response_pu =
        g_sensorless_shadow.hfi.q_response_pu;
    g_sensorless_diag.hfi_phase_error =
        g_sensorless_shadow.hfi.phase_error;
    g_sensorless_diag.hfi_pole_detect_current_pu =
        HfiEstimator_GetPoleDetectCurrent(&g_sensorless_shadow.hfi);
    g_sensorless_diag.hfi_base_current_alpha_pu =
        g_sensorless_shadow.hfi.base_current_alpha_pu;
    g_sensorless_diag.hfi_base_current_beta_pu =
        g_sensorless_shadow.hfi.base_current_beta_pu;
    g_sensorless_diag.hfi_hf_current_alpha_pu =
        g_sensorless_shadow.hfi.hf_current_alpha_pu;
    g_sensorless_diag.hfi_hf_current_beta_pu =
        g_sensorless_shadow.hfi.hf_current_beta_pu;
    g_sensorless_diag.hfi_pole_positive_response_pu =
        g_sensorless_shadow.hfi.pole_positive_response_pu;
    g_sensorless_diag.hfi_pole_negative_response_pu =
        g_sensorless_shadow.hfi.pole_negative_response_pu;
    g_sensorless_diag.hfi_pole_contrast =
        g_sensorless_shadow.hfi.pole_contrast;
    g_sensorless_diag.hfi_axis_locked =
        g_sensorless_shadow.hfi.axis_locked;
    g_sensorless_diag.hfi_pole_detected =
        g_sensorless_shadow.hfi.pole_detected;
    g_sensorless_diag.hfi_polarity_corrected =
        g_sensorless_shadow.hfi.polarity_corrected;
    g_sensorless_diag.hfi_pole_state =
        g_sensorless_shadow.hfi.pole_state;
    g_sensorless_diag.hfi_demod_update_counter =
        g_sensorless_shadow.hfi.demod_update_counter;
    g_sensorless_diag.saliency_ratio =
        g_sensorless_shadow.motor_profile.saliency_ratio;
    g_sensorless_diag.current_alpha_pu = current_alpha_pu;
    g_sensorless_diag.current_beta_pu = current_beta_pu;
    g_sensorless_diag.voltage_alpha_pu = input.voltage_alpha_pu;
    g_sensorless_diag.voltage_beta_pu = input.voltage_beta_pu;
    g_sensorless_diag.bus_voltage_pu = _IQtoF(AdRead.Udc);
    g_sensorless_diag.emf_alpha_pu = g_sensorless_shadow.observer.emf_alpha_pu;
    g_sensorless_diag.emf_beta_pu = g_sensorless_shadow.observer.emf_beta_pu;
    g_sensorless_diag.emf_magnitude_pu = g_sensorless_shadow.observer.emf_magnitude_pu;
    g_sensorless_diag.pll_phase_error =
        g_sensorless_shadow.observer.phase_error;
    g_sensorless_diag.pll_omega_e_rad_s =
        g_sensorless_shadow.observer.omega_e_rad_s;
    g_sensorless_diag.observer_raw_theta_q24 = Sensorless_TurnsToQ24(
        g_sensorless_shadow.observer.theta_raw_turns);
    g_sensorless_diag.observer_phase_advance_turns =
        g_sensorless_shadow.observer.phase_advance_turns;
    g_sensorless_diag.observer_omega_tracking_e_rad_s =
        g_sensorless_shadow.observer.omega_tracking_e_rad_s;
    g_sensorless_diag.observer_emf_confidence =
        g_sensorless_shadow.observer.emf_confidence;
    g_sensorless_diag.observer_current_error_pu =
        g_sensorless_shadow.observer.current_error_magnitude_pu;
    g_sensorless_diag.observer_weak_emf_counter =
        g_sensorless_shadow.observer.weak_emf_counter;
    g_sensorless_diag.identification_state =
        (uint16_t)g_sensorless_shadow.identification.state;
    g_sensorless_diag.identification_quality =
        g_sensorless_shadow.identification.result.quality;
    g_sensorless_diag.identification_valid =
        g_sensorless_shadow.identification.result.valid;
    g_sensorless_diag.identification_active =
        MotorParamIdentification_IsBusy(
            &g_sensorless_shadow.identification);
    g_sensorless_diag.identification_fault_flags =
        g_sensorless_shadow.identification.fault_flags;
    g_sensorless_diag.identified_rs_pu =
        g_sensorless_shadow.identification.result.rs_pu;
    g_sensorless_diag.identified_ld_pu =
        g_sensorless_shadow.identification.result.ld_pu;
    g_sensorless_diag.identified_lq_pu =
        g_sensorless_shadow.identification.result.lq_pu;
    g_sensorless_diag.identified_ls_pu =
        g_sensorless_shadow.identification.result.ls_pu;
    g_sensorless_diag.identified_flux_pu =
        g_sensorless_shadow.identification.result.flux_pu;
    g_sensorless_diag.identified_rs_ohm =
        g_sensorless_shadow.identification.result.rs_ohm;
    g_sensorless_diag.identified_ld_h =
        g_sensorless_shadow.identification.result.ld_h;
    g_sensorless_diag.identified_lq_h =
        g_sensorless_shadow.identification.result.lq_h;
    g_sensorless_diag.identified_ls_h =
        g_sensorless_shadow.identification.result.ls_h;
    g_sensorless_diag.identified_flux_wb =
        g_sensorless_shadow.identification.result.flux_wb;
    g_sensorless_diag.first_fault = g_sensorless_shadow.startup.failure.fault;
    g_sensorless_diag.failure_state = g_sensorless_shadow.startup.failure.state;
    g_sensorless_diag.stage_cycles = g_sensorless_shadow.startup.state_cycles;
    g_sensorless_diag.total_cycles = g_sensorless_shadow.startup.total_start_cycles;
    g_sensorless_diag.debug_mode = g_sensorless_shadow.startup.config.debug_mode;
    g_sensorless_diag.debug_completed = g_sensorless_shadow.startup.debug_completed;
    g_sensorless_diag.open_theta_q24 = Sensorless_TurnsToQ24(g_sensorless_shadow.startup.open_loop_theta_turns);
    g_sensorless_diag.control_theta_q24 = (int32_t)Etheta.Ethetapk;
    g_sensorless_diag.observer_open_error_q24 = RotorFeedback_AngleErrorQ24(
        g_sensorless_diag.observer_theta_q24, g_sensorless_diag.open_theta_q24);
    g_sensorless_diag.blend_fraction = g_sensorless_shadow.startup.blend_fraction;
    g_sensorless_diag.phase_error_rad = g_sensorless_shadow.observer.phase_error_rad;
    g_sensorless_diag.lock_reason = g_sensorless_shadow.observer.lock_reason;
    g_sensorless_diag.id_feedback_pu = _IQtoF(Park.Ds);
    g_sensorless_diag.iq_feedback_pu = _IQtoF(Park.Qs);
    g_sensorless_diag.id_limited_pu = g_source_foc_current.id_reference_pu;
    g_sensorless_diag.iq_limited_pu = g_source_foc_current.iq_reference_pu;
    g_sensorless_diag.ud_pu = g_source_foc_current.voltage_d;
    g_sensorless_diag.uq_pu = g_source_foc_current.voltage_q;
    g_sensorless_diag.voltage_saturated = g_source_foc_current.saturated;
    g_sensorless_diag.current_failure = g_source_foc_current.failure_reason;
    Sensorless_TraceRecord(0U);
}
