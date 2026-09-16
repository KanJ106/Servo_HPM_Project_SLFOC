#ifndef MTU3_H
#define MTU3_H
#include <stdint.h>
#include "r_cg_project.h"
#include "MCU_Fw.h"

#if SERVO_MCU == HPM_6E00

#if   HARDWARE_VER_SEL == HARDWARE_VER_0
#define PWM                  HPM_PWM1
#define PWM_CLOCK_NAME       clock_pwm1
#define BOARD_APP_PWM_IRQ    IRQn_PWM1
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
#define PWM                  HPM_PWM0
#define PWM_CLOCK_NAME       clock_pwm0
#define BOARD_APP_PWM_IRQ    IRQn_PWM0

#define PWM1                  HPM_PWM1
#define PWM_CLOCK_NAME1       clock_pwm1
#define Pos_Loop_IRQ          IRQn_PWM1
#else
#define PWM                  HPM_PWM1
#define PWM_CLOCK_NAME       clock_pwm1
#define BOARD_APP_PWM_IRQ    IRQn_PWM1
#endif

#define PWM_OUTPUT_PIN0 pwm_channel_0  // First pair
#define PWM_OUTPUT_PIN1 pwm_channel_1  
#define PWM_OUTPUT_PIN2 pwm_channel_2  // Sec pair
#define PWM_OUTPUT_PIN3 pwm_channel_3  
#define PWM_OUTPUT_PIN4 pwm_channel_4  // Thr pair
#define PWM_OUTPUT_PIN5 pwm_channel_5 

//#define BOARD_APP_PWM_Channel_0         pwm_channel_0
//#define BOARD_APP_PWM_Channel_1         pwm_channel_1
//#define BOARD_APP_PWM_Channel_2         pwm_channel_2
//#define BOARD_APP_PWM_Channel_3         pwm_channel_3
//#define BOARD_APP_PWM_Channel_4         pwm_channel_4
//#define BOARD_APP_PWM_Channel_5         pwm_channel_5

#define BOARD_APP_PWM_FAULT_PIN         (5)

#define TRGM BOARD_APP_TRGM
#define TRGM_PWM_OUTPUT BOARD_APP_TRGM_PWM_OUTPUT

#define BOARD_APP_TRGM                  HPM_TRGM0
#define BOARD_APP_TRGM_PWM_OUTPUT       HPM_TRGM0_OUTPUT_SRC_PWM1_TRIG_IN0
#define BOARD_APP_TRGM_PWM_OUTPUT1      HPM_TRGM0_OUTPUT_SRC_PWM1_TRIG_IN1
#define BOARD_APP_TRGM_PWM_OUTPUT2      HPM_TRGM0_OUTPUT_SRC_PWM1_TRIG_IN2
#define BOARD_APP_TRGM_PWM_INPUT        HPM_TRGM0_INPUT_SRC_PWM1_TRGO_0

#define PWM_PERIOD_IN_MS (1)  // PWM reload 1ms
#define DEAD_TIME_TICKS 200   // dead time

extern uint32_t reload;

#define PWMCLKASE  200000000
#else
extern uint16_t PWMA_SET_COMPAREVALUE;
extern uint16_t PWMB_SET_COMPAREVALUE;
extern uint16_t PWMC_SET_COMPAREVALUE;
#define PWMCLKASE  200000000
#endif

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void PWMV2_Init(uint16_t pwmmode,uint16_t db,uint16_t Pwmflag);
void MTU3PWMOff(void);
void MTU3PWMOffDB(void);
void PWMOutEnable(void);
void PWMCounterStart(void);
void PWMGpioOutEnable(void);
void PwmDutyUpdata(uint16_t uduty,uint16_t vduty,uint16_t wduty,uint16_t T);
void SetG7Duty(uint16_t Duty);
extern uint16_t EPWM_PRD_DEFAULT;
extern uint32_t ESPD_PRD_DEFAULT; 
extern uint32_t EPOS_PRD_DEFAULT;  
extern uint16_t EPWM_DB;  
#endif
