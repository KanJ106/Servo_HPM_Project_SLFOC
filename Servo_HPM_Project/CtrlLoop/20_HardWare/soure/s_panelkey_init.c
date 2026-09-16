#include "s_panelkey_init.h"


#if SERVO_MCU != NO_MCU
#define TEST_SPI                    BOARD_APP_SPI_BASE
/***********************************************************************************************************************
* Function Name: Panel_KeyInit
* Description  : This function initializes the RSPI1 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Panel_KeyInit(void)
{
    spi_initialize_config_t init_config;

    board_init_spi_clock(TEST_SPI);

    HPM_IOC->PAD[IOC_PAD_PF25].FUNC_CTL = IOC_PF25_FUNC_CTL_GPIO_F_25;
    HPM_IOC->PAD[IOC_PAD_PF26].FUNC_CTL = IOC_PF26_FUNC_CTL_SPI7_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    HPM_IOC->PAD[IOC_PAD_PF28].FUNC_CTL = IOC_PF28_FUNC_CTL_SPI7_MISO;
    HPM_IOC->PAD[IOC_PAD_PF29].FUNC_CTL = IOC_PF29_FUNC_CTL_SPI7_MOSI;

    HPM_GPIO0->OE[GPIO_DO_GPIOF].SET = 1 << 25;
    HPM_GPIO0->DO[GPIO_DO_GPIOF].CLEAR = 1 << 25;

    hpm_spi_get_default_init_config(&init_config);
    init_config.direction = spi_msb_first;
    init_config.mode = spi_master_mode;
    init_config.clk_phase = spi_sclk_sampling_even_clk_edges;
    init_config.clk_polarity = spi_sclk_high_idle;
    init_config.data_len = 16;
    /* step.1  initialize spi */
    hpm_spi_initialize(TEST_SPI, &init_config);
   
    /* step.2  set spi sclk frequency for master */
    hpm_spi_set_sclk_frequency(TEST_SPI, 500000); 
}

uint16_t Panel_Key_RxTxInquire(uint16_t Txdata)
{
    volatile uint8_t i = 15;
    uint16_t Rxdata;

    HPM_GPIO0->DO[GPIO_DO_GPIOF].CLEAR = 1 << 25;
    while(i)
    {
        i--;
    }
    HPM_GPIO0->DO[GPIO_DO_GPIOF].SET = 1 << 25;
    
    Rxdata = TEST_SPI->DATA;
    TEST_SPI->DATA = Txdata;

    TEST_SPI->CMD = 0xFF;

    return Rxdata;
}
#else
/***********************************************************************************************************************
* Function Name: Panel_KeyInit
* Description  : This function initializes the RSPI1 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void Panel_KeyInit(void)
{

}

uint16_t Panel_Key_RxTxInquire(uint16_t Txdata)
{

}

#endif


