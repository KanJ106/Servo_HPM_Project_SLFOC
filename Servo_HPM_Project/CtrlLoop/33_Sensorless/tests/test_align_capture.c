#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Drive.h"
#include "SV_FaultProtect.h"
#include "SensorlessShadow.h"
SENSORLESS_CHECK_DRVCOEFF DrvCoeff;
SENSORLESS_CHECK_ETHETA Etheta;
SENSORLESS_CHECK_IDREF IdRef;
SENSORLESS_CHECK_IQREF IqRef;
SENSORLESS_CHECK_SPDFB SpdFb;
SENSORLESS_CHECK_SVPWM Svpwm;
SENSORLESS_CHECK_CLARK Clark;
SENSORLESS_CHECK_PARK Park;
SENSORLESS_CHECK_ADREAD AdRead;
SENSORLESS_CHECK_STATE_MACHINE StateMachine;
SENSORLESS_CHECK_FAULT FaultP;
volatile uint32_t g_sensorless_fault_detail;
void Sensorless_GlobalFaultHook(uint32_t f){g_sensorless_fault_detail=f;}
static int failures;
#define CHECK(x) do{if(!(x)){printf("FAIL %d %s\n",__LINE__,#x);failures++;}}while(0)
static void setup(void)
{
    SOURCE_FOC_RECIPE r={123,1,.2316666667f,.00046f,.00046f,21.33f,48,
        6.363961f,21.33f*.03125f,600,100,10,1};
    SENSORLESS_STARTUP_CONFIG c;
    StateMachine.RegulFlg=0;FaultP.FaultStatus=0;
    DrvCoeff.TpwmFrq=16000;DrvCoeff.SpdBase=8192;DrvCoeff.SpdMax=600;
    DrvCoeff.MotPoles=10;DrvCoeff.DrvIbase=2133;DrvCoeff.UdcBase=48;
    DrvCoeff.UdcEpu=_IQ(1);AdRead.Udc=_IQ(1);
    SensorlessShadow_Init();
    CHECK(SensorlessShadow_ApplySourceRecipe(&r));
    c=g_sensorless_shadow.startup.config;
    c.debug_mode=SENSORLESS_DEBUG_TRAVEL90;
    c.align_current_pu=c.startup_iq_pu=c.iq_limit_pu=.03125f;
    c.align_cycles=8000;c.current_ramp_cycles=3200;
    c.handoff_speed_rpm=100;c.open_loop_accel_rpm_s=c.run_accel_rpm_s=100;
    c.sensorless_exit_speed_rpm=20;c.ramp_timeout_cycles=288000;c.total_timeout_cycles=320000;
    CHECK(SensorlessShadow_ConfigureStartup(&c));
    g_source_foc_current.measured_trip_pu=.0390625f;
    g_observation_request=1;StateMachine.RegulFlg=1;
    CHECK(SensorlessShadow_RequestStart(1,100));
}
static void cycle(float id,float iq)
{
    SOURCE_FOC_CURRENT expected;
    uint16_t ok;
    SensorlessShadow_PreFoc();
    Park.Ds=_IQ(id);Park.Qs=_IQ(iq);
    g_sensorless_chain.sdm_u=1234;g_sensorless_chain.sdm_v=-2345;
    g_sensorless_chain.zero_u=28;g_sensorless_chain.zero_v=22;
    expected=g_source_foc_current;
    ok=SourceFoc_Step(&g_source_foc_current,g_sensorless_diag.id_command_pu,
        g_sensorless_diag.iq_command_pu,_IQtoF(Park.Ds),_IQtoF(Park.Qs),.5f);
    SourceFoc_Step(&expected,g_sensorless_diag.id_command_pu,
        g_sensorless_diag.iq_command_pu,_IQtoF(Park.Ds),_IQtoF(Park.Qs),.5f);
    SensorlessShadow_RecordAlignCurrent(.5f,1);
    CHECK(memcmp(&expected,&g_source_foc_current,sizeof(expected))==0);
    if(!ok)SensorlessShadow_CurrentFault(g_source_foc_current.failure_reason);
    SensorlessShadow_PostFoc();
}
int main(void)
{
    unsigned i;
    float previous_v;
    setup();
    for(i=0;i<523;i++)cycle(.02f,0);
    previous_v=g_source_foc_current.voltage_d;
    cycle(.03759765625f,.01138204336f);
    CHECK(g_align_trace.header.total_samples==524);
    CHECK(g_align_trace.header.first_count==128 && g_align_trace.header.tail_count==396);
    CHECK(g_align_trace.header.frozen && !g_align_trace.header.active);
    CHECK(g_align_trace.header.stop_reason==2);
    CHECK(g_align_trace.samples[522].voltage_d==previous_v);
    CHECK(g_align_trace.samples[523].voltage_d==0);
    CHECK(fabsf(g_align_trace.samples[523].id_ref-.03125f *
        (float)g_align_trace.samples[523].stage_cycles/3200.0f)<1e-8f);
    CHECK(g_align_trace.samples[523].current_failure==SOURCE_FOC_OVER_CURRENT);
    CHECK(g_align_trace.samples[523].flags==1);
    CHECK(g_align_trace.samples[523].sdm_u==1234 && g_align_trace.samples[523].zero_v==22);
    CHECK(g_source_foc_current.measured_trip_pu==.0390625f);
    for(i=1;i<524;i++)CHECK(g_align_trace.samples[i].tick==g_align_trace.samples[i-1].tick+1);
    {FILE *f=fopen("align_fault_simulated.bin","wb");CHECK(f!=0);
     if(f){fwrite((const void *)&g_align_trace,sizeof(g_align_trace),1,f);fclose(f);}}
    SensorlessShadow_Reset();
    CHECK(g_align_trace.header.total_samples==524 && g_align_trace.header.frozen);
    setup();
    g_observation_request=0; /* Changing request mid-run must not reinterpret storage. */
    for(i=0;i<8002;i++)cycle(.02f,0);
    CHECK(g_observation_kind==1 && !g_align_trace.header.active);
    CHECK(g_align_trace.header.stop_reason==1 && g_align_trace.header.total_samples==8000);
    CHECK(g_align_trace.header.first_count==128 && g_align_trace.header.tail_count==416);
    CHECK(g_sensorless_shadow.startup.state==SENSORLESS_START_OPEN_LOOP_RAMP);
    {FILE *f=fopen("align_complete_simulated.bin","wb");CHECK(f!=0);
     if(f){fwrite((const void *)&g_align_trace,sizeof(g_align_trace),1,f);fclose(f);}}
    setup();
    SensorlessShadow_PreFoc();
    Park.Ds=Park.Qs=0;
    CHECK(SourceFoc_Step(&g_source_foc_current,.03125f,0,0,0,.0001f));
    SensorlessShadow_RecordAlignCurrent(.0001f,1);
    CHECK(g_align_trace.samples[0].saturated==1);
    CHECK(fabsf(g_align_trace.samples[0].voltage_d-.0001f)<1e-8f);
    StateMachine.RegulFlg=0;SensorlessShadow_PreFoc();SensorlessShadow_PostFoc();
    CHECK(g_align_trace.header.frozen);
    setup();for(i=0;i<3;i++)cycle(.02f,0);
    StateMachine.RegulFlg=0;SensorlessShadow_PreFoc();SensorlessShadow_PostFoc();
    CHECK(!g_align_trace.header.active && g_align_trace.header.frozen);
    CHECK(g_align_trace.header.total_samples==3);
    printf("ALIGN capture integration: %d failures\n",failures);
    return failures?1:0;
}
