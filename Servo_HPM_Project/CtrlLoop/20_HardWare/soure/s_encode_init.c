#include "s_encode_init.h"

uint8_t  UartRxuff[22] = {0x00,0x00,0x00,0x00,0x00,0x00};

#if SERVO_MCU == HPM_6E00

void EncodeSci_Main(void)
{
    #if   HARDWARE_VER_SEL == HARDWARE_VER_0
    HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART1_TXD;
    HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
    HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART1_RXD;
    HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
    HPM_IOC->PAD[IOC_PAD_PY05].FUNC_CTL = IOC_PY05_FUNC_CTL_UART1_DE;
    HPM_PIOC->PAD[IOC_PAD_PY05].FUNC_CTL = PIOC_PY05_FUNC_CTL_SOC_PY_05;
    #elif HARDWARE_VER_SEL == HARDWARE_VER_1
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PC09_FUNC_CTL_UART2_RXD;
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PC08_FUNC_CTL_UART2_TXD;
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PC10_FUNC_CTL_UART2_DE;
    #else
    HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART1_TXD;
    HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
    HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART1_RXD;
    HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
    HPM_IOC->PAD[IOC_PAD_PY05].FUNC_CTL = IOC_PY05_FUNC_CTL_UART1_DE;
    HPM_PIOC->PAD[IOC_PAD_PY05].FUNC_CTL = PIOC_PY05_FUNC_CTL_SOC_PY_05;
    #endif

    board_init_uart_clock(EncodeSCI);

    uart_config_t config = {0};
    uart_default_config(EncodeSCI, &config);
    config.baudrate = 2500000U;
    config.fifo_enable = true;
    config.src_freq_in_hz = clock_get_frequency(clock_uart1);
    config.rx_fifo_level = uart_fifo_16_bytes;
    config.tx_fifo_level = uart_fifo_16_bytes;

    uart_init(EncodeSCI, &config);

    //uart_trig_config_t trig_config = {0};
    //trig_config.en_stop_bit_insert = false;
    //trig_config.trig_clr_rxfifo = true;    /* clear rx fifo after being triggered */
    //trig_config.trig_mode = true;          /* start transmission after being triggered */
    //trig_config.hardware_trig = true;      /* enable hardware trigger */
    //uart_config_transfer_trig_mode(EncodeSCI, &trig_config);
}
#else

void EncodeSci_Main(void)
{

}

#endif



