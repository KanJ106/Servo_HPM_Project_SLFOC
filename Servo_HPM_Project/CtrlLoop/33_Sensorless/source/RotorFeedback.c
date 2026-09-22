#include "RotorFeedback.h"

#include <string.h>

int32_t RotorFeedback_WrapAngleQ24(int32_t angle_q24)
{
    int32_t wrapped = angle_q24 % ROTOR_ANGLE_ONE_TURN_Q24;

    if (wrapped < 0) {
        wrapped += ROTOR_ANGLE_ONE_TURN_Q24;
    }

    return wrapped;
}

int32_t RotorFeedback_AngleErrorQ24(int32_t angle_q24, int32_t reference_q24)
{
    int32_t error = RotorFeedback_WrapAngleQ24(angle_q24)
                  - RotorFeedback_WrapAngleQ24(reference_q24);

    if (error >= ROTOR_ANGLE_HALF_TURN_Q24) {
        error -= ROTOR_ANGLE_ONE_TURN_Q24;
    } else if (error < -ROTOR_ANGLE_HALF_TURN_Q24) {
        error += ROTOR_ANGLE_ONE_TURN_Q24;
    }

    return error;
}

void RotorFeedbackMux_Init(ROTOR_FEEDBACK_MUX *mux)
{
    if (mux == 0) {
        return;
    }

    memset(mux, 0, sizeof(*mux));
    mux->mode = ROTOR_MODE_SENSORLESS_SHADOW;
    mux->selected.source = ROTOR_SOURCE_NONE;
}

void RotorFeedbackMux_SetMode(ROTOR_FEEDBACK_MUX *mux, ROTOR_MODE mode)
{
    if (mux != 0) {
        mux->mode = mode;
    }
}

void RotorFeedbackMux_UpdateEncoder(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback)
{
    if ((mux != 0) && (feedback != 0)) {
        mux->encoder = *feedback;
        mux->encoder.source = ROTOR_SOURCE_ENCODER;
    }
}

void RotorFeedbackMux_UpdateSensorless(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback)
{
    if ((mux != 0) && (feedback != 0)) {
        mux->sensorless = *feedback;
        mux->sensorless.source = ROTOR_SOURCE_SENSORLESS;
    }
}

void RotorFeedbackMux_UpdateOpenLoop(ROTOR_FEEDBACK_MUX *mux, const ROTOR_FEEDBACK *feedback)
{
    if ((mux != 0) && (feedback != 0)) {
        mux->open_loop = *feedback;
        mux->open_loop.source = ROTOR_SOURCE_OPEN_LOOP;
    }
}

const ROTOR_FEEDBACK *RotorFeedbackMux_Select(ROTOR_FEEDBACK_MUX *mux)
{
    const ROTOR_FEEDBACK *candidate;

    if (mux == 0) {
        return 0;
    }

    candidate = &mux->encoder;

    if ((mux->mode == ROTOR_MODE_SENSORLESS_MONITOR)
        || (mux->mode == ROTOR_MODE_SENSORLESS_ONLY)) {
        candidate = &mux->sensorless;
    } else if (mux->mode == ROTOR_MODE_OPEN_LOOP) {
        candidate = &mux->open_loop;
    }

    if ((candidate->valid == 0U)
        && (mux->mode == ROTOR_MODE_SENSORLESS_MONITOR)) {
        candidate = &mux->encoder;
        mux->fallback_count++;
    }

    mux->selected = *candidate;
    return &mux->selected;
}
