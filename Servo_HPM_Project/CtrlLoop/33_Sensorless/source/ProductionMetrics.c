#include "ProductionMetrics.h"
#include <math.h>
#include <string.h>
typedef struct {
    uint32_t count, invalid;
    float sum_square[3], peak, error_sum, error_square_sum, target;
    float bus_min, bus_max;
} METRIC_ACC;
static volatile METRIC_ACC accumulator;
static volatile uint16_t active;

uint16_t ProductionMetrics_Validate(const PRODUCTION_METRIC_LIMITS *l)
{
    return (uint16_t)(l && isfinite(l->min_phase_rms_a)
        && isfinite(l->max_phase_rms_a) && isfinite(l->max_phase_peak_a)
        && isfinite(l->max_rms_imbalance_ratio) && isfinite(l->max_speed_std_rpm)
        && l->min_phase_rms_a >= 0 && l->max_phase_rms_a > l->min_phase_rms_a
        && l->max_phase_peak_a >= l->max_phase_rms_a
        && l->max_rms_imbalance_ratio > 0 && l->max_rms_imbalance_ratio <= 1
        && l->max_speed_std_rpm > 0 && l->min_samples > 0 && l->min_samples <= 2000000U);
}
void ProductionMetrics_Begin(float target_rpm)
{
    METRIC_ACC empty = {0};
    active = 0U;
    empty.target = target_rpm;
    empty.bus_min = INFINITY;
    accumulator = empty;
    active = 1U;
}
void ProductionMetrics_Cancel(void) { active = 0U; }
void ProductionMetrics_Sample(float alpha, float beta, float rpm, float bus)
{
    float phases[3], error;
    unsigned k;
    if (!active) return;
    if (!isfinite(alpha) || !isfinite(beta) || !isfinite(rpm) || !isfinite(bus)
        || fabsf(alpha) > 10000 || fabsf(beta) > 10000 || fabsf(rpm) > 100000
        || bus < 0 || bus > 10000 || accumulator.count >= 2000000U) {
        if (accumulator.invalid != UINT32_MAX) accumulator.invalid++;
        return;
    }
    phases[0] = alpha;
    phases[1] = -0.5f*alpha + 0.86602540378f*beta;
    phases[2] = -0.5f*alpha - 0.86602540378f*beta;
    for (k=0; k<3; ++k) {
        accumulator.sum_square[k] += phases[k]*phases[k];
        if (fabsf(phases[k]) > accumulator.peak) accumulator.peak = fabsf(phases[k]);
    }
    error = rpm - accumulator.target;
    accumulator.error_sum += error;
    accumulator.error_square_sum += error*error;
    if (bus < accumulator.bus_min) accumulator.bus_min = bus;
    if (bus > accumulator.bus_max) accumulator.bus_max = bus;
    accumulator.count++;
}
void ProductionMetrics_Finish(const PRODUCTION_METRIC_LIMITS *l, PRODUCTION_METRIC_RESULT *r)
{
    METRIC_ACC a;
    float average=0, lo=INFINITY, hi=0, mean, variance;
    unsigned k;
    active = 0U;
    if (!r) return;
    a = accumulator;
    memset(r, 0, sizeof(*r));
    r->samples = a.count; r->invalid_samples = a.invalid;
    if (!ProductionMetrics_Validate(l) || a.count < l->min_samples)
        r->failure_flags |= PROD_METRIC_NO_DATA;
    if (a.invalid) r->failure_flags |= PROD_METRIC_NUMERIC;
    if (!a.count || !ProductionMetrics_Validate(l)) return;
    for (k=0; k<3; ++k) {
        r->phase_rms_a[k] = sqrtf(a.sum_square[k] / (float)a.count);
        average += r->phase_rms_a[k] / 3.0f;
        lo = fminf(lo, r->phase_rms_a[k]); hi = fmaxf(hi, r->phase_rms_a[k]);
        if (r->phase_rms_a[k] < l->min_phase_rms_a || r->phase_rms_a[k] > l->max_phase_rms_a)
            r->failure_flags |= PROD_METRIC_CURRENT;
    }
    r->phase_peak_a = a.peak;
    r->rms_imbalance_ratio = average > 1e-6f ? (hi-lo)/average : 0;
    mean = a.error_sum / (float)a.count;
    variance = a.error_square_sum / (float)a.count - mean*mean;
    r->speed_mean_rpm = a.target + mean;
    r->speed_std_rpm = sqrtf(fmaxf(variance, 0));
    r->bus_min_v = a.bus_min; r->bus_max_v = a.bus_max;
    if (a.peak > l->max_phase_peak_a) r->failure_flags |= PROD_METRIC_PEAK;
    if (r->rms_imbalance_ratio > l->max_rms_imbalance_ratio) r->failure_flags |= PROD_METRIC_IMBALANCE;
    if (r->speed_std_rpm > l->max_speed_std_rpm) r->failure_flags |= PROD_METRIC_SPEED;
}
