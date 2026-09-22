#include <stdio.h>
#include "SensorlessEncoderMonitor.h"
static int failures;
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failures++;}}while(0)
static void frame(uint8_t *f,uint32_t m,uint32_t o,uint8_t status)
{
    unsigned i;
    memset(f,0,11); f[0]=0x6a;
    f[1]=(uint8_t)m;f[2]=(uint8_t)(m>>8);f[3]=(uint8_t)(m>>16);
    f[4]=(uint8_t)o;f[5]=(uint8_t)(o>>8);f[6]=(uint8_t)(o>>16);
    f[9]=status;for(i=0;i<10;i++)f[10]^=f[i];
}
int main(void)
{
    SL_ENCODER_MONITOR s;
    uint8_t f[11];
    SlEncoder_Init(&s,12); CHECK(s.abi==2 && !s.valid);
    SlEncoder_Sample(&s,0,0,0); CHECK(!s.valid && s.reason==2);
    frame(f,131070,524286,0); SlEncoder_Sample(&s,f,11,0);
    CHECK(s.valid && s.motor_raw==131070 && s.output_raw==524286);
    SlEncoder_Sample(&s,f,11,0); CHECK(s.valid && s.age_ms==0); /* stationary is fresh */
    frame(f,2,3,0);SlEncoder_Sample(&s,f,11,1);
    CHECK(s.record_id==1 && s.before_valid && s.run_valid);
    CHECK(s.motor_delta==4 && s.output_delta==5);
    frame(f,131069,524285,0);SlEncoder_Sample(&s,f,11,1);
    CHECK(s.motor_delta==-1 && s.output_delta==-1);
    CHECK(s.motor_min_delta==-1 && s.motor_max_delta==4);
    SlEncoder_Sample(&s,f,11,0);CHECK(s.after_valid && s.run_valid && !s.drive_active);
    frame(f,100,200,0);SlEncoder_Sample(&s,f,11,0);
    CHECK(s.after_motor==131069 && s.motor_delta==-1); /* stopped record preserved */
    f[10]^=1;SlEncoder_Sample(&s,f,11,0);
    CHECK(!s.valid && s.reason==3 && s.bad_crc==1 && s.motor_raw==100);
    frame(f,100,200,0);SlEncoder_Sample(&s,f,11,1);
    CHECK(s.record_id==2 && !s.before_valid && !s.run_valid);
    SlEncoder_Sample(&s,f,10,1);CHECK(!s.valid && s.run_errors==1);
    SlEncoder_Sample(&s,f,11,0);CHECK(s.after_valid && !s.run_valid);
    SlEncoder_Sample(&s,f,11,1);CHECK(s.record_id==3 && s.run_valid);
    frame(f,101,201,1);SlEncoder_Sample(&s,f,11,1);
    CHECK(!s.valid && s.reason==4 && !s.run_valid && s.bad_status==1);
    frame(f,131072,201,0);SlEncoder_Sample(&s,f,11,0);
    CHECK(!s.valid && s.reason==5 && !s.after_valid);
    frame(f,101,524288,0);SlEncoder_Sample(&s,f,11,0);CHECK(!s.valid && s.reason==5);
    CHECK(!(s.sequence&1U));
    CHECK(SlEncoder_Delta(131070,2,131072)==-4);
    SlEncoder_Init(&s,4);CHECK(!s.initialized && !s.valid && s.reason==1);
    /* Five-ms trajectory, intermediate invalidity, frozen stop and overflow. */
    SlEncoder_Init(&s,12);
    frame(f,131060,524280,0); SlEncoder_Sample(&s,f,11,0);
    for(unsigned n=0;n<21;n++) {
        frame(f,(131060+n*10)%131072,(524280+n)%524288,0);
        if(n==2) SlEncoder_Sample(&s,0,0,1);
        else SlEncoder_Sample(&s,f,11,1);
    }
    CHECK(s.trace_count==5 && !s.trace_overflow);
    CHECK(s.trace[1].tick_ms-s.trace[0].tick_ms==5);
    CHECK((s.trace[1].flags&1U) && !(s.trace[1].flags&2U));
    CHECK((s.trace[2].flags&3U)==3U);
    CHECK(SlEncoder_Delta(s.trace[1].motor_raw,s.trace[0].motor_raw,131072)==50);
    SlEncoder_Sample(&s,f,11,0);
    CHECK(s.trace_count==6 && !(s.trace[5].flags&4U));
    for(unsigned n=0;n<10;n++) SlEncoder_Sample(&s,f,11,0);
    CHECK(s.trace_count==6); /* no overwriting after stop */
    for(unsigned n=0;n<2700;n++) SlEncoder_Sample(&s,f,11,1);
    CHECK(s.trace_count==511 && s.trace_overflow);
    SlEncoder_Sample(&s,f,11,0);
    CHECK(s.trace_count==512 && !(s.trace[511].flags&4U));
    SlEncoder_Sample(&s,f,11,1);
    CHECK(s.trace_count==1 && !s.trace_overflow);
    CHECK(sizeof(s)==8356);
    printf("encoder read-only monitor: %d failures\n",failures);
    return failures?1:0;
}
