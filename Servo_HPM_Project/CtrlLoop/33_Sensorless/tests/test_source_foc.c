#include <stdio.h>
#include <math.h>
#include "SourceFoc.h"
#include "ProductionMetrics.h"
static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); failures++; } } while(0)

static void test_motor(float r, float ld, float lq)
{
    SOURCE_FOC_RECIPE recipe = {1,1,r,ld,lq,10,100,2,1,3000,200,4,1};
    SENSORLESS_MOTOR_PROFILE profile;
    SOURCE_FOC_CURRENT pi;
    float id=0, iq=0, ref, previous_kp;
    unsigned i;
    CHECK(SourceFoc_BuildProfile(&recipe,16000,3000,&profile));
    CHECK(fabsf(profile.effective.stator_resistance_pu-r*0.1f)<1e-6f);
    CHECK(SourceFoc_Configure(&pi,&profile,200));
    for(i=0;i<16000;i++) {
        ref = i < 8000 ? 0.08f : -0.08f;
        CHECK(SourceFoc_Step(&pi,0,ref,id,iq,0.5f));
        /* Independent physical RL plant, amperes and volts converted explicitly. */
        id += (pi.voltage_d*100-r*id*10) / ld /16000/10;
        iq += (pi.voltage_q*100-r*iq*10) / lq /16000/10;
        if(i==7999 || i==15999) CHECK(fabsf(iq-ref)<0.001f);
    }
    previous_kp = pi.kp_d;
    CHECK(!SourceFoc_Configure(&pi,&profile,NAN));
    CHECK(pi.kp_d==previous_kp);
    for(i=0;i<4000;i++) CHECK(SourceFoc_Step(&pi,0.5f,0.5f,0,0,0.005f));
    CHECK(hypotf(pi.voltage_d,pi.voltage_q)<=0.005001f);
    for(i=0;i<8000;i++) {
        CHECK(SourceFoc_Step(&pi,0,0,id,iq,0.5f));
        id += (pi.voltage_d*100-r*id*10)/ld/16000/10;
        iq += (pi.voltage_q*100-r*iq*10)/lq/16000/10;
    }
    CHECK(fabsf(id)+fabsf(iq)<0.001f);
    CHECK(!SourceFoc_Step(&pi,NAN,0,0,0,0.5f));
    CHECK(pi.voltage_d==0 && pi.voltage_q==0 && pi.numeric_faults==1);
    CHECK(!SourceFoc_Step(&pi,0,0,0.3f,0,0.5f));
    CHECK(pi.overcurrent_faults==1 && pi.voltage_d==0);
    recipe.allow_continuous_rotation=0;
    CHECK(SourceFoc_BuildProfile(&recipe,16000,3000,&profile));
    recipe.allow_continuous_rotation=2;
    CHECK(!SourceFoc_BuildProfile(&recipe,16000,3000,&profile));
}
static void test_metrics(void)
{
    PRODUCTION_METRIC_LIMITS limits={0.6f,0.8f,1.1f,0.05f,2,1000};
    PRODUCTION_METRIC_RESULT result;
    unsigned i;
    ProductionMetrics_Begin(500);
    for(i=0;i<16000;i++) {
        float a=6.28318530718f*(float)i/160.0f;
        ProductionMetrics_Sample(cosf(a),sinf(a),500,48);
    }
    ProductionMetrics_Finish(&limits,&result);
    CHECK(result.failure_flags==0);
    CHECK(fabsf(result.phase_rms_a[0]-0.70710678f)<0.001f);
    CHECK(result.samples==16000 && result.speed_std_rpm<0.01f);
    ProductionMetrics_Begin(500);
    ProductionMetrics_Finish(&limits,&result);
    CHECK(result.failure_flags & PROD_METRIC_NO_DATA);
    ProductionMetrics_Begin(500);
    for(i=0;i<1200;i++) ProductionMetrics_Sample(3,0,(i&1)?600.0f:400.0f,48);
    ProductionMetrics_Sample(NAN,0,500,48);
    ProductionMetrics_Finish(&limits,&result);
    CHECK(result.failure_flags & PROD_METRIC_CURRENT);
    CHECK(result.failure_flags & PROD_METRIC_PEAK);
    CHECK(result.failure_flags & PROD_METRIC_IMBALANCE);
    CHECK(result.failure_flags & PROD_METRIC_SPEED);
    CHECK(result.failure_flags & PROD_METRIC_NUMERIC);
}
int main(void)
{
    test_motor(1,0.003f,0.004f);
    test_motor(2,0.008f,0.008f);
    test_metrics();
    printf("source FOC / metrics: %d failures\n",failures);
    return failures ? 1 : 0;
}
