#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "SensorlessTakeoverProbe.h"
static SL_PROBE_DIAGNOSTICS d;
int main(void)
{
    uint32_t n;
    SlProbeDiagnostics_Begin(&d,15);
    for(n=0;n<64;++n)SlProbeDiagnostics_Record(&d,SL_P_HOLD,n,n==17,0);
    assert(d.total_samples==64 && d.total_saturated==1);
    assert(d.first_saturated_tick==17 && d.last_saturated_tick==17);
    assert(d.max_run_any==1 && d.max_run[SL_P_HOLD]==1);
    for(n=64;n<70;++n)SlProbeDiagnostics_Record(&d,n<67?SL_P_HOLD:SL_P_RETURN,n,1,0);
    assert(d.max_run_any==6 && d.max_run[SL_P_HOLD]==3 && d.max_run[SL_P_RETURN]==3);
    SlProbeDiagnostics_Record(&d,SL_P_RETURN,70,1,1);
    assert(d.total_saturated==7 && d.failed_samples==1 && d.run==0);
    SlProbeDiagnostics_Record(&d,SL_P_RETURN,71,1,0);
    SlProbeDiagnostics_Record(&d,SL_P_RETURN,73,1,0);
    assert(d.gap_count==1 && d.run==1 && d.max_run_any==6);
    n=d.total_samples;SlProbeDiagnostics_Record(&d,9,74,1,0);
    assert(d.total_samples==n && !(d.sequence&1U));
    SlProbeDiagnostics_Finish(&d);SlProbeDiagnostics_Record(&d,SL_P_DONE,74,1,0);
    assert(!d.active && d.total_samples==n && !(d.sequence&1U));
    SlProbeDiagnostics_Begin(&d,16);
    assert(d.active && d.record_id==16 && !d.total_samples && !d.total_saturated);
    SlProbeDiagnostics_Record(&d,8,UINT32_MAX,1,0);
    SlProbeDiagnostics_Record(&d,8,0,1,0);
    assert(!d.gap_count && d.max_run_any==2 && d.first_saturated_tick==UINT32_MAX);
    SlProbeDiagnostics_Begin(&d,17);
    SlProbeDiagnostics_Record(&d,8,0,1,0);
    SlProbeDiagnostics_Record(&d,8,1,1,0);
    assert(d.first_saturated_tick==0 && d.last_saturated_tick==1);
    puts("PASS PWM diagnostics: pulse, phase crossing, failure, gap, finish, reset, tick wrap");
    return 0;
}
