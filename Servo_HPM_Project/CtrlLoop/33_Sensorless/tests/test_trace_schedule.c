/* Compare lazy payload construction against the frozen V18 recording routine.
 * Old and new recorders receive identical common fields; unsaved long fields
 * are poisoned. Complete rings and metadata must match after every PWM. */
#include "SensorlessTakeoverProbe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SlProbe_Step Ref_Step
#define SlProbe_EndExternal Ref_EndExternal
#define SlProbeDiagnostics_Begin Ref_Diagnostics_Begin
#define SlProbeDiagnostics_Record Ref_Diagnostics_Record
#define SlProbeDiagnostics_Finish Ref_Diagnostics_Finish
#define SlProbeTrace_Begin Ref_Trace_Begin
#define SlProbeTrace_Finish Ref_Trace_Finish
#define SlProbeTrace_Record Ref_Trace_Record
#define g_probe_diagnostics ref_probe_diagnostics
#include "../../../diagnostics/smo_shadow_candidate_v19_20260921/reference_v18/CtrlLoop/33_Sensorless/source/SensorlessTakeoverProbe.c"
#undef SlProbe_Step
#undef SlProbe_EndExternal
#undef SlProbeDiagnostics_Begin
#undef SlProbeDiagnostics_Record
#undef SlProbeDiagnostics_Finish
#undef SlProbeTrace_Begin
#undef SlProbeTrace_Finish
#undef SlProbeTrace_Record
#undef g_probe_diagnostics
#define VERIFY(c) do { if(!(c)) {fprintf(stderr,"trace schedule failure line %d\n",__LINE__);exit(2);} } while(0)
static SL_PROBE_TRACE old_trace,new_trace;
int main(void)
{
    unsigned profile,scenario,n,due_count=0,total=0;
    for(profile=0;profile<3;++profile)for(scenario=0;scenario<5;++scenario) {
        SL_PROBE probe; SL_PROBE_LONG full,lazy; uint32_t done=31537U+scenario*73U;
        memset(&probe,0,sizeof(probe));probe.a2=profile==1;probe.a3=profile==2;
        memset(&old_trace,0,sizeof(old_trace));memset(&new_trace,0,sizeof(new_trace));
        Ref_Trace_Begin(&old_trace,7);SlProbeTrace_Begin(&new_trace,7);
        for(n=1;n<done+2100U;++n) {
            uint32_t before_divider=new_trace.h.divider,due;
            if(n==12001U)probe.entry_tick=n;
            if(n==done){probe.done_tick=n;probe.phase=scenario==1?SL_P_SKIP:SL_P_DONE;}
            if(scenario==2 && n==20007U)probe.reason=SL_P_FOLLOW;
            memset(&full,0,sizeof(full));full.tick=n;full.phase=probe.phase;
            full.gates=n;full.reason=probe.reason;full.applied=(int32_t)n*31;
            full.forced=(int32_t)n*29;full.id_ref=.01f;full.iq_ref=-.03f;
            full.id=.011f;full.iq=-.031f;full.ud=.001f;full.uq=.02f;full.offset=.003f;
            full.flags=scenario==3 && n>=20011U?0x80000000UL:16U;
            full.smo=(int32_t)n*33;full.encoder_raw=n&131071U;
            full.encoder_read_tick=n-15U;full.encoder_request_tick=n-30U;
            full.emf=.02f;full.phase_error=.03f;full.smo_rpm=-100.1f;
            full.encoder_rpm=-100.2f;full.follow=.004f;full.stage_cycles=n;
            due=SlProbeTrace_LongDue(&new_trace,&probe,n);
            VERIFY(new_trace.h.divider==before_divider);
            memset(&lazy,0xa5,sizeof(lazy));
            lazy.tick=full.tick;lazy.phase=full.phase;lazy.gates=full.gates;lazy.reason=full.reason;
            lazy.applied=full.applied;lazy.forced=full.forced;lazy.id_ref=full.id_ref;lazy.iq_ref=full.iq_ref;
            lazy.id=full.id;lazy.iq=full.iq;lazy.ud=full.ud;lazy.uq=full.uq;lazy.flags=full.flags;lazy.offset=full.offset;
            if(due){lazy=full;++due_count;}
            Ref_Trace_Record(&old_trace,&probe,&full,123,-456);
            SlProbeTrace_Record(&new_trace,&probe,&lazy,123,-456);
            VERIFY(memcmp(&old_trace,&new_trace,sizeof(old_trace))==0);++total;
            if(scenario==4 && n==24013U) {
                Ref_Trace_Finish(&old_trace,&probe,2);SlProbeTrace_Finish(&new_trace,&probe,2);
                VERIFY(memcmp(&old_trace,&new_trace,sizeof(old_trace))==0);
            }
        }
    }
    printf("PASS lazy trace: 15 scenarios, %u PWM comparisons, %u long constructions; full capture equals V18\n",total,due_count);
    return 0;
}
