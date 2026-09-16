#include "s_sys_init.h"


#if SERVO_MCU != NO_MCU
EWDG_Type * testewg;
/***********************************************************************************************************************
* Function Name: EnableDog
* Description  : This function initializes IWDT module.
* Arguments    : None
* Return Value : None   100ms
***********************************************************************************************************************/
void Ewdg_Init(void)
{ 
    //ewdg_config_t config;
    //clock_add_to_group(clock_watchdog0, 0);
    //ewdg_get_default_config(HPM_EWDG0, &config);

    //config.enable_watchdog = true;
    //config.ctrl_config.enable_refresh_lock = false;
    //config.int_rst_config.enable_timeout_reset = true;
    //config.ctrl_config.use_lowlevel_timeout = false;
    //uint32_t ewdg_src_clk_freq = EWDG_CNT_CLK_FREQ;  
    //config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;

    //config.cnt_src_freq = ewdg_src_clk_freq; 
    //config.ctrl_config.timeout_reset_us = 200000;//200ms

    //testewg = HPM_EWDG0;
    //ewdg_init(HPM_EWDG0, &config);
}

void EnableDog(void)
{ 
    //if(HPM_EWDG0->WDT_EN == 0)
    //{
    //    ewdg_enable(HPM_EWDG0);
    //}
}

void DisableDog(void)
{ 
    //if(HPM_EWDG0->WDT_EN == 1)
    //{
    //    ewdg_disable(HPM_EWDG0);
    //}
}


/***********************************************************************************************************************
* Function Name: ServiceDog
* Description  : This function restarts WDT0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void ServiceDog(void)
{
    //if(HPM_EWDG0->WDT_EN == 1)
    //{
    //    HPM_EWDG0->REF_PROT = EWDG_REFRESH_UNLOCK_FIXED_KEY;
    //    HPM_EWDG0->WDT_REFRESH_REG = EWDG_REFRESH_KEY;
    //}
}

void ResetCPU(void)
{
    disable_global_irq(CSR_MSTATUS_MIE_MASK);
    ppor_sw_reset(HPM_PPOR, 1000);
}
#else
/***********************************************************************************************************************
* Function Name: EnableDog
* Description  : This function initializes IWDT module.
* Arguments    : None
* Return Value : None   8.533ms
***********************************************************************************************************************/
void EnableDog(void)
{ 

}

/***********************************************************************************************************************
* Function Name: ServiceDog
* Description  : This function restarts WDT0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void ServiceDog(void)
{
  
}

void ResetCPU(void)
{

}
#endif

