#ifndef ROTOR_FEEDBACK_H_
#define ROTOR_FEEDBACK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROTOR_ANGLE_ONE_TURN_Q24    (16777216L)
#define ROTOR_ANGLE_HALF_TURN_Q24   (8388608L)

typedef enum {
    ROTOR_MODE_ENCODER = 0,
    ROTOR_MODE_SENSORLESS_SHADOW,
    ROTOR_MODE_SENSORLESS_MONITOR,
    ROTOR_MODE_SENSORLESS_ONLY,
    ROTOR_MODE_OPEN_LOOP
} ROTOR_MODE;

typedef enum {
    ROTOR_SOURCE_NONE = 0,
    ROTOR_SOURCE_ENCODER,
    ROTOR_SOURCE_SENSORLESS,
    ROTOR_SOURCE_OPEN_LOOP,
    ROTOR_SOURCE_HFI,
    ROTOR_SOURCE_BLEND
} ROTOR_SOURCE;

typedef struct {
    int32_t theta_e_q24;
    int32_t omega_e_q24;
    int32_t theta_m_q24;
    int32_t speed_rpm;
    uint16_t valid;
    uint16_t quality;
    uint16_t source;
    int16_t direction;
    uint32_t timestamp;
    uint32_t fault;
} ROTOR_FEEDBACK;

typedef struct {
    ROTOR_MODE mode;
    ROTOR_FEEDBACK encoder;
    ROTOR_FEEDBACK sensorless;
    ROTOR_FEEDBACK open_loop;
    ROTOR_FEEDBACK selected;
    uint32_t fallback_count;
} ROTOR_FEEDBACK_MUX;

void RotorFeedbackMux_Init(ROTOR_FEEDBACK_MUX *mux);
void RotorFeedbackMux_SetMode(ROTOR_FEEDBACK_MUX *mux, ROTOR_MODE mode);
void RotorFeedbackMux_UpdateEncoder(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback);
void RotorFeedbackMux_UpdateSensorless(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback);
void RotorFeedbackMux_UpdateOpenLoop(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback);
const ROTOR_FEEDBACK *RotorFeedbackMux_Select(ROTOR_FEEDBACK_MUX *mux);
int32_t RotorFeedback_WrapAngleQ24(int32_t angle_q24);
int32_t RotorFeedback_AngleErrorQ24(int32_t angle_q24, int32_t reference_q24);

#ifdef __cplusplus
}
#endif

#endif /* ROTOR_FEEDBACK_H_ */
