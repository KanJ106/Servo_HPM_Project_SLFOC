#include <stdio.h>
#include <math.h>
#include "SourceFoc.h"
#include "SmoPll.h"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); failures++; } } while(0)
#define TWO_PI 6.2831853071795864769f
static SMO_PLL_PARAMS joint(float current_base)
{
    SOURCE_FOC_RECIPE r={0x480901,1,.23166667f,.00046f,.00046f,
        current_base,48.0f,6.363961f,current_base*.03125f,600,100,10,1};
    SENSORLESS_MOTOR_PROFILE profile;
    SMO_PLL_PARAMS p;
    CHECK(SourceFoc_BuildProfile(&r,16000,8192,&profile));
    MotorParamProfile_ApplyToSmo(&profile,&p);
    CHECK(SmoPll_ValidateParams(&p));
    CHECK(fabsf(p.current_model_gain-48/(16000*.00046f*current_base))<1e-6f);
    CHECK(p.switching_gain==.25f);
    CHECK(p.current_model_gain*(p.stator_resistance_pu+p.switching_gain/p.boundary_pu)<=1.00001f);
    return p;
}
static void zero_input(SMO_PLL_PARAMS p)
{
    SMO_PLL_INPUT in={0};
    SMO_PLL_STATE s;
    SmoPll_Init(&s,&p,0);
    s.current_hat_alpha_pu=.001f; s.current_hat_beta_pu=-.002f;
    in.expected_direction=-1;
    for(int i=0;i<10000;i++)SmoPll_Step(&s,&p,&in);
    CHECK(fabsf(s.current_hat_alpha_pu)<1e-6f && fabsf(s.current_hat_beta_pu)<1e-6f);
    CHECK(s.emf_magnitude_pu<1e-6f && !s.locked && !s.numeric_fault_count);
}
static void spin(SMO_PLL_PARAMS p,int direction,float emf,int expect_lock)
{
    SMO_PLL_STATE s;
    SMO_PLL_INPUT in={0};
    float omega=direction*100*10*TWO_PI/60, theta=0;
    float ia=0,ib=direction*.03125f,max_error=0,max_angle=0;
    unsigned locks=0;
    SmoPll_Init(&s,&p,0);
    SmoPll_SeedTracking(&s,0,omega);
    in.expected_direction=(int16_t)direction;
    for(int n=0;n<64000;n++) {
        float next=theta+omega*p.sample_time_s;
        float a=-direction*.03125f*sinf(next),b=direction*.03125f*cosf(next);
        in.voltage_alpha_pu=p.stator_resistance_pu*ia+(a-ia)/p.current_model_gain-direction*emf*sinf(theta);
        in.voltage_beta_pu=p.stator_resistance_pu*ib+(b-ib)/p.current_model_gain+direction*emf*cosf(theta);
        in.current_alpha_pu=a;in.current_beta_pu=b;
        SmoPll_Step(&s,&p,&in);
        ia=a;ib=b;theta=next;
        if(theta>TWO_PI)theta-=TWO_PI;
        if(theta<0)theta+=TWO_PI;
        if(n>=32000) {
            float err=fabsf(s.omega_e_rad_s*60/(10*TWO_PI)-direction*100);
            float angle=s.theta_pll_turns-theta/TWO_PI;
            angle=fabsf(angle-floorf(angle+.5f))*360;
            if(err>max_error)max_error=err;
            if(angle>max_angle)max_angle=angle;
            locks+=s.locked;
        }
    }
    CHECK(!s.numeric_fault_count);
    CHECK(expect_lock ? locks==32000 : locks==0);
    CHECK(max_error<.05f && max_angle<1.0f);
}
int main(void)
{
    SMO_PLL_PARAMS p=joint(21.33f), bad;
    zero_input(p);zero_input(joint(25.6f));
    for(int d=-1;d<=1;d+=2) {
        spin(p,d,.02f,1);
        spin(p,d,.01f,0);
    }
    bad=p;bad.boundary_pu=.02f;CHECK(!SmoPll_ValidateParams(&bad));
    bad=p;bad.current_model_gain=1;bad.stator_resistance_pu=2;
    bad.boundary_pu=1;CHECK(!SmoPll_ValidateParams(&bad));
    bad=p;bad.current_model_gain=.5f;bad.stator_resistance_pu=0;
    bad.switching_gain=1;bad.boundary_pu=.25f;CHECK(!SmoPll_ValidateParams(&bad));
    bad.boundary_pu=.251f;CHECK(SmoPll_ValidateParams(&bad));
    bad.boundary_pu=NAN;CHECK(!SmoPll_ValidateParams(&bad));
    {
        SENSORLESS_MOTOR_PROFILE impossible={0};
        impossible.effective.sample_hz=16000;
        impossible.effective.stator_resistance_pu=2;
        impossible.observer_model_gain=1;
        MotorParamProfile_ApplyToSmo(&impossible,&bad);
        CHECK(!SmoPll_ValidateParams(&bad));
    }
    printf("SMO recipe stability and synthetic rotation: %d failures (software only)\n",failures);
    return failures?1:0;
}
