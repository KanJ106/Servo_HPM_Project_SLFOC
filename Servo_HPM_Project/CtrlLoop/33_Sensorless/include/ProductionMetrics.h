#ifndef PRODUCTION_METRICS_H_
#define PRODUCTION_METRICS_H_
#include <stdint.h>
typedef struct {
    float min_phase_rms_a, max_phase_rms_a, max_phase_peak_a;
    float max_rms_imbalance_ratio, max_speed_std_rpm;
    uint32_t min_samples;
} PRODUCTION_METRIC_LIMITS;
typedef struct {
    uint32_t samples, invalid_samples, failure_flags;
    float phase_rms_a[3], phase_peak_a;
    float rms_imbalance_ratio, speed_mean_rpm, speed_std_rpm;
    float bus_min_v, bus_max_v;
} PRODUCTION_METRIC_RESULT;
#define PROD_METRIC_NO_DATA (1UL << 0)
#define PROD_METRIC_NUMERIC (1UL << 1)
#define PROD_METRIC_CURRENT (1UL << 2)
#define PROD_METRIC_PEAK (1UL << 3)
#define PROD_METRIC_IMBALANCE (1UL << 4)
#define PROD_METRIC_SPEED (1UL << 5)
uint16_t ProductionMetrics_Validate(const PRODUCTION_METRIC_LIMITS *limits);
/* Begin/Finish: 1ms service; Sample: higher-priority PWM ISR, single core.
 * W current is reconstructed from alpha/beta, not an independent sensor. */
void ProductionMetrics_Begin(float target_rpm);
void ProductionMetrics_Sample(float alpha_a, float beta_a, float rpm, float bus_v);
void ProductionMetrics_Finish(const PRODUCTION_METRIC_LIMITS *limits,
                              PRODUCTION_METRIC_RESULT *result);
void ProductionMetrics_Cancel(void);
#endif
