#ifndef SENSORLESS_CONFIG_H_
#define SENSORLESS_CONFIG_H_

/* This project is the free-rotation source-FOC production-test candidate. */
#define SENSORLESS_SOURCE_FOC_BUILD (1U)

/* Final production-test build: takeover code and the mapped level-1 fault hook
 * are compiled in. Motion still requires an armed, explicit start command. */
#ifndef SENSORLESS_CONTROL_OUTPUT_ENABLE
#define SENSORLESS_CONTROL_OUTPUT_ENABLE       (1U)
#endif
#ifndef SENSORLESS_GLOBAL_FAULT_HOOK_READY
#define SENSORLESS_GLOBAL_FAULT_HOOK_READY      (1U)
#endif

#if (SENSORLESS_CONTROL_OUTPUT_ENABLE != 0U) && (SENSORLESS_GLOBAL_FAULT_HOOK_READY == 0U)
#error "Sensorless control requires a mapped global fault hook before output can be enabled"
#endif

#define SENSORLESS_DEFAULT_SAMPLE_HZ           (16000.0f)
#define SENSORLESS_DEFAULT_RS_PU               (0.05f)
#define SENSORLESS_DEFAULT_MODEL_GAIN          (0.02f)
#define SENSORLESS_MODEL_GAIN_MIN              (0.0001f)
#define SENSORLESS_MODEL_GAIN_MAX              (0.50f)
#define SENSORLESS_DEFAULT_SWITCH_GAIN         (0.25f)
#define SENSORLESS_DEFAULT_BOUNDARY_PU          (0.02f)
#define SENSORLESS_DEFAULT_EMF_LPF_HZ           (300.0f)
#define SENSORLESS_DEFAULT_EMF_DELAY_COMP_GAIN  (1.0f)
#define SENSORLESS_DEFAULT_PLL_KP               (200.0f)
#define SENSORLESS_DEFAULT_PLL_KI               (10000.0f)
#define SENSORLESS_DEFAULT_PLL_SPEED_LPF_ALPHA  (0.05f)
#define SENSORLESS_DEFAULT_WEAK_EMF_DECAY       (0.001f)
#define SENSORLESS_DEFAULT_MIN_EMF_PU           (0.01f)
#define SENSORLESS_DEFAULT_UNLOCK_EMF_RATIO     (0.70f)
#define SENSORLESS_DEFAULT_LOCK_PHASE_ERR       (0.35f)
#define SENSORLESS_DEFAULT_MAX_ACCEL_E_RAD_S2   (50000.0f)
#define SENSORLESS_DEFAULT_CURRENT_HAT_LIMIT_PU (2.0f)
#define SENSORLESS_DEFAULT_LOCK_CYCLES          (800U)
#define SENSORLESS_DEFAULT_UNLOCK_CYCLES        (160U)
#define SENSORLESS_DEFAULT_VOLTAGE_SCALE        (1.0f)
#define SENSORLESS_DEFAULT_MAX_ELEC_RAD_S       (4000.0f)

#endif /* SENSORLESS_CONFIG_H_ */
