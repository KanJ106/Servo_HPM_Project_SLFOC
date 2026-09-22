#ifndef SENSORLESS_VOFA_H
#define SENSORLESS_VOFA_H
#include <stdint.h>
#include <string.h>
/* JustFloat: seven little-endian IEEE754 binary32 channels, then +Inf tail.
 * Read-only telemetry, not a motor command protocol. */
#define SL_VOFA_FRAME_BYTES 32U
typedef struct {
    uint32_t abi, initialized, init_errors, frames, busy_skips, snapshot_skips;
} SL_VOFA_DIAG;
static inline void SlVofa_Pack(uint8_t out[SL_VOFA_FRAME_BYTES],
    uint32_t motor, uint32_t output, uint32_t valid,
    uint32_t drive, uint32_t frame_id)
{
    float ch[7];
    uint32_t bits, i, j;
    ch[0]=(float)motor; ch[1]=(float)output;
    ch[2]=(float)motor*(360.0f/131072.0f);
    ch[3]=(float)output*(360.0f/524288.0f);
    ch[4]=valid?1.0f:0.0f; ch[5]=drive?1.0f:0.0f;
    ch[6]=(float)(frame_id & 65535U);
    for(i=0;i<7U;i++) {
        /* Quiet NaN for invalid positions; never send retained positions as good. */
        if(i<4U && !valid) bits=0x7fc00000UL;
        else memcpy(&bits,&ch[i],4U);
        for(j=0;j<4U;j++) out[i*4U+j]=(uint8_t)(bits>>(8U*j));
    }
    out[28]=0; out[29]=0; out[30]=0x80U; out[31]=0x7fU;
}
#endif
