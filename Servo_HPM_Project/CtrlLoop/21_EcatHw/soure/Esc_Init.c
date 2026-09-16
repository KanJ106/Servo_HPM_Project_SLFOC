#include "hpm_esc_drv.h"
#include "hpm_clock_drv.h"
#include "pinmux.h"
#include "hpm_gpio_drv.h"
#include "board.h"
#include "hpm_ecat_phy.h"
#include "hpm_gptmr_drv.h"
#include "ecat_escinit.h"
#include "Flash_Eeprom.h"
//#include "user_config.h"
//#include "eeprom_emulation.h"
//#include "hpm_ecat_e2p_emulation.h"

/* ecat timer */
#define ECAT_TIMER_GPTMR        HPM_GPTMR0
#define ECAT_TIMER_GPTMR_CH     (0U)
#define ECAT_TIMER_GPTMR_CLK    (clock_gptmr0)

uint16_t * pEsc;//

void DISABLE_ESC_INT(void)
{

}


void ENABLE_ESC_INT(void)
{

}

/* enable gptmr 1ms tick for ecat */
void ecat_timer_ms_enable(void)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    clock_add_to_group(ECAT_TIMER_GPTMR_CLK, 0);
    gptmr_channel_get_default_config(ECAT_TIMER_GPTMR, &config);
    gptmr_freq = clock_get_frequency(ECAT_TIMER_GPTMR_CLK);
    config.reload = gptmr_freq / 1000; /* 1ms */
    gptmr_channel_config(ECAT_TIMER_GPTMR, ECAT_TIMER_GPTMR_CH, &config, false);
    gptmr_start_counter(ECAT_TIMER_GPTMR, ECAT_TIMER_GPTMR_CH);
}

uint16_t HW_GetTimer(void)
{
    return ((ECAT_TIMER_GPTMR->SR & GPTMR_CH_RLD_STAT_MASK(ECAT_TIMER_GPTMR_CH)) == (GPTMR_CH_RLD_STAT_MASK(ECAT_TIMER_GPTMR_CH)));
}

void HW_ClearTimer(void)
{
    ECAT_TIMER_GPTMR->SR = (GPTMR_CH_RLD_STAT_MASK(ECAT_TIMER_GPTMR_CH));
}

void HW_Release(void)
{

}

void HW_SetLed(uint8_t RunLed,uint8_t ErrLed)
{
    if(RunLed == 0x00)
    {
        HPM_GPIO0->DO[GPIO_DI_GPIOC].SET = 1 << 17;
    }
    else
    {
        HPM_GPIO0->DO[GPIO_DI_GPIOC].CLEAR = 1 << 17;
    }

    if(ErrLed == 0x00)
    {
        HPM_GPIO0->DO[GPIO_DI_GPIOC].SET = 1 << 18;
    }
    else
    {
        HPM_GPIO0->DO[GPIO_DI_GPIOC].CLEAR = 1 << 18;
    }
}

void ecat_eeprom_emulation_reload_response(void);
uint16_t ecat_write_alias(void);
uint16_t HW_Init(void)
{
    hpm_stat_t stat;
    uint16_t  Esc_Status = 0;
    esc_eeprom_clock_config_t esc_config;

    ESC_Type *esc_ptr = HPM_ESC;

    board_init_ethercat(HPM_ESC);


    //init_esc_eeprom_pin(); /* config ESC pin for EEPROM */
    //esc_config.eeprom_emulation = false;
    ///* esc_config.eeprom_size_over_16kbit = true; */
    //esc_config.eeprom_size_over_16kbit = 1; /* single eeprom usage */
    //esc_config_eeprom_and_clock(HPM_ESC, &esc_config);

    Eeprom_flash_eeprom_init();
    esc_config.eeprom_emulation = true;
    esc_config.eeprom_size_over_16kbit = false; /* eeprom emulation not use this */
    esc_config_eeprom_and_clock(esc_ptr, &esc_config);

    /* eeprom reload */
    ecat_eeprom_emulation_reload_response();

    /* Wait Loading ESC EEPROM and check */
    stat = esc_check_eeprom_loading(HPM_ESC);
    if (stat != status_success)
    {
        Esc_Status = 65535;
    }
    else
    {
        Esc_Status = 0;   
    }

    /* PHY reset */
    ecat_phy_reset();

    /* Set PHY OFFSET */
    esc_set_phy_offset(HPM_ESC, BOARD_ECAT_PHY_ADDR_OFFSET);

    /* PHY LED mode configuration */
    stat = ecat_phy_config(HPM_ESC);
    if (stat != status_success)
    {
        Esc_Status = 65535;
        //return Esc_Status;
    }
    else
    {
        Esc_Status = 0;   
    }

    /* assign ESC_CTRLx to specific function, check with hardware connection */
    esc_config_ctrl_signal_function(esc_ptr, BOARD_ECAT_NMII_LINK0_CTRL_INDEX, esc_ctrl_signal_func_alt_nmii_link0, BOARD_ECAT_PORT0_LINK_INVERT); /* NMII_LINK0 */
#if defined(BOARD_ECAT_SUPPORT_PORT1) && BOARD_ECAT_SUPPORT_PORT1
    esc_config_ctrl_signal_function(esc_ptr, BOARD_ECAT_NMII_LINK1_CTRL_INDEX, esc_ctrl_signal_func_alt_nmii_link1, BOARD_ECAT_PORT1_LINK_INVERT); /* NMII_LINK1 */
#endif
#if defined(BOARD_ECAT_SUPPORT_PORT2) && BOARD_ECAT_SUPPORT_PORT2
    esc_config_ctrl_signal_function(esc_ptr, BOARD_ECAT_NMII_LINK2_CTRL_INDEX, esc_ctrl_signal_func_alt_nmii_link2, BOARD_ECAT_PORT2_LINK_INVERT); /* NMII_LINK2 */
#endif
    esc_config_nmii_link_source(esc_ptr, true, BOARD_ECAT_SUPPORT_PORT1, BOARD_ECAT_SUPPORT_PORT2); /* configure nmii_link source */

#if defined(BOARD_ECAT_SUPPORT_RUN_ERROR_LED) && BOARD_ECAT_SUPPORT_RUN_ERROR_LED
    esc_config_ctrl_signal_function(esc_ptr, BOARD_ECAT_LED_RUN_CTRL_INDEX, esc_ctrl_signal_func_alt_led_run, false);   /* LED_RUN */
    esc_config_ctrl_signal_function(esc_ptr, BOARD_ECAT_LED_ERROR_CTRL_INDEX, esc_ctrl_signal_func_alt_led_err, false); /* LED_ERROR */
#endif

    if (status_success == stat) {
        /* Open Timer driver */
        ecat_timer_ms_enable();

        /* Initialize the EtherCAT Slave Interface of the SSC */
        /* set esc register base address */
        pEsc = (uint16_t *) HPM_ESC_BASE;

        /* Enable EtherCAT interrupts */
        //ENABLE_ESC_INT();
        {
            HPM_ESC->GPR_CFG1 |= ESC_GPR_CFG1_SYNC0_IRQ_EN_MASK | ESC_GPR_CFG1_SYNC1_IRQ_EN_MASK | ESC_GPR_CFG1_RSTO_IRQ_EN_MASK;

            intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, 3);
            //intc_m_enable_irq_with_priority(IRQn_ESC_SYNC1, 3);
            intc_m_enable_irq_with_priority(IRQn_ESC, 3);
            //intc_m_enable_irq_with_priority(IRQn_ESC_RESET, 3);
        }
    }
    else 
    {
        /* Close operation */
    }

    HW_EscWriteWord(ecat_write_alias(), 0x12);//写入Alias

    return Esc_Status;
}

/* In case that pAPPL_EEPROM_Reload is implemented the function
“HW_EepromReload()” is not used.*/
uint16_t HW_EepromReload(void)
{
    return 0;
}
