#include <stdint.h>

#include "SensorlessStartup.h"
#include "SV_FaultProtect.h"

volatile uint32_t g_sensorless_fault_detail;

void Sensorless_GlobalFaultHook(uint32_t fault_flags)
{
    if (g_sensorless_fault_detail == 0U) g_sensorless_fault_detail = fault_flags;

    if ((fault_flags & (SENSORLESS_FAULT_PARAM_INVALID | SENSORLESS_FAULT_CURRENT_CONFIG)) != 0U) {
        FaultPrtt_FaultInterface(SensorlessParamErr);
    } else {
        FaultPrtt_FaultInterface(SensorlessTrackErr);
    }
}
