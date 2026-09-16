/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "s_time_init.h"

#if SERVO_MCU == HPM_6E00
void R_CMT0_Create(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    gptmr_channel_get_default_config(BOARD_CALLBACK_TIMER, &config);

    clock_add_to_group(BOARD_CALLBACK_TIMER_CLK_NAME, 0);
    gptmr_freq = clock_get_frequency(BOARD_CALLBACK_TIMER_CLK_NAME);

    config.reload = gptmr_freq / 1000;
    gptmr_channel_config(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH, &config, false);

    gptmr_enable_irq(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_IRQ_MASK(BOARD_CALLBACK_TIMER_CH));
    intc_m_enable_irq_with_priority(BOARD_CALLBACK_TIMER_IRQ, 1);

}

/***********************************************************************************************************************
* Function Name: R_CMT0_Start
* Description  : This function starts the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Start(void)
{
    gptmr_start_counter(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH);
}

/***********************************************************************************************************************
* Function Name: R_CMT0_Stop
* Description  : This function stops the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Stop(void)
{
    gptmr_stop_counter(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH);
}

TestGptmr  mqbtest = {0, 0, 0};
#define    CheckTimeCLK clock_gptmr1
#define    CheckTime    HPM_GPTMR1
#define    CheckTimeCh  0
void CheckTimeInit(void)
{
    uint32_t gptmr_freq;   
    gptmr_channel_config_t config;

    gptmr_channel_get_default_config(CheckTime, &config);
    clock_set_source_divider(CheckTimeCLK, clk_src_pll1_clk2, 250);

    clock_add_to_group(CheckTimeCLK, 0);
    gptmr_freq = clock_get_frequency(CheckTimeCLK);

    gptmr_channel_config(CheckTime, CheckTimeCh, &config, false);

    gptmr_start_counter(CheckTime, CheckTimeCh);
}

void TestTimer_Start(void)
{
    gptmr_start_counter(CheckTime, CheckTimeCh);
}

void TestTimer_Stop(void)
{
    gptmr_stop_counter(CheckTime, CheckTimeCh);
}

uint32_t ReadCount(void)
{
    return CheckTime->CHANNEL[CheckTimeCh].CNT;
}

void ResetCount(void)
{
    CheckTime->CHANNEL[CheckTimeCh].CR = 1  << 14;  //mqb复位计数器
    CheckTime->CHANNEL[CheckTimeCh].CR &= 0 << 14;  //mqb清零复位计数器
    CheckTime->CHANNEL[CheckTimeCh].CR = 1  << 10;  //mqb使能计数器
}
#else
void R_CMT0_Create(void)
{

}

/***********************************************************************************************************************
* Function Name: R_CMT0_Start
* Description  : This function starts the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Start(void)
{

}

/***********************************************************************************************************************
* Function Name: R_CMT0_Stop
* Description  : This function stops the CMT0 channel counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CMT0_Stop(void)
{

}
#endif
