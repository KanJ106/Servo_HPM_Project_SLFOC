#ifndef S_ANALOG_SAMPLING_H
#define S_ANALOG_SAMPLING_H
#include "r_cg_Project.h"
#include "MCU_Fw.h"

#if SERVO_MCU != NO_MCU
#define ADCBIT 16
#define APP_ADC16_SEQ_START_POS              (0U)
#define APP_ADC16_CH_SAMPLE_CYCLE            (20U)

#ifndef APP_ADC16_CORE
#define APP_ADC16_CORE BOARD_RUNNING_CORE
#endif

#define APP_ADC16_HW_TRGM                    HPM_TRGM0
#define APP_ADC16_SEQ_IRQ_EVENT              adc16_event_seq_single_complete

#if HARDWARE_VER_SEL == HARDWARE_VER_0
#define APP_ADC16_HW_TRGM_IN                 HPM_TRGM0_INPUT_SRC_PWM1_TRGO_0
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
#define APP_ADC16_HW_TRGM_IN                 HPM_TRGM0_INPUT_SRC_PWM0_TRGO_0
#else
#define APP_ADC16_HW_TRGM_IN                 HPM_TRGM0_INPUT_SRC_PWM1_TRGO_0
#endif

extern uint32_t seq_complete_flag;

extern ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc0_seq_buff[8];
extern ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc1_seq_buff[8];
extern ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc2_seq_buff[8];

#define  GPTMR_PWM_SYNC_CHANNEL   2
#define  CurSample_SDM            HPM_SDM0
#define  CurSample_SDMCLK         clock_sdm0
#define  SDM_Channel1             0 
#define  SDM_Channel2             2

#if SERVO_HARDWARE == HARDWARE_DC0
#define U_ADVALUE1         ((uint16_t)(adc0_seq_buff[0] & 0x0000FFFF))
#define V_ADVALUE1         ((uint16_t)(adc1_seq_buff[0] & 0x0000FFFF))
#define W_ADVALUE1         ((uint16_t)(adc2_seq_buff[0] & 0x0000FFFF))
#define UDC1_ADVALUE       ((uint16_t)(adc2_seq_buff[1] & 0x0000FFFF))
#define UDC2_ADVALUE       ((uint16_t)(adc2_seq_buff[1] & 0x0000FFFF))
#define UDC3_ADVALUE       ((uint16_t)(adc2_seq_buff[1] & 0x0000FFFF))
#define MOTORTEMP_ADVALUE  (65535)
#define IGBTTEMP_ADVALUE   (65535)
#define A1V65_ADVALUE      (32768)
#define GND_ADVALUE        (0)
#endif

#if SERVO_HARDWARE == HARDWARE_AC0
#define UDC1_ADVALUE       (0)
#define UDC2_ADVALUE       (0)
#define UDC3_ADVALUE       (0)
#define MOTORTEMP_ADVALUE  (0)
#define IGBTTEMP_ADVALUE   (0)
#define A1V65_ADVALUE      (0)
#define GND_ADVALUE        (0)
#endif
#else
#if SERVO_HARDWARE == HARDWARE_DC0
#define UDC1_ADVALUE       (0)
#define UDC2_ADVALUE       (0)
#define UDC3_ADVALUE       (0)
#define MOTORTEMP_ADVALUE  (0)
#define IGBTTEMP_ADVALUE   (0)
#define A1V65_ADVALUE      (0)
#define GND_ADVALUE        (0)
#endif

#if SERVO_HARDWARE == HARDWARE_AC0
#define UDC1_ADVALUE       (0)
#define UDC2_ADVALUE       (0)
#define UDC3_ADVALUE       (0)
#define MOTORTEMP_ADVALUE  (0)
#define IGBTTEMP_ADVALUE   (0)
#define A1V65_ADVALUE      (0)
#define GND_ADVALUE        (0)
#endif
#endif


extern uint16_t ADCBuf[21];
void Adc_Init(void);
void Sigma_Delta_Init(void);
void Get_AdcValue(void);
#endif
