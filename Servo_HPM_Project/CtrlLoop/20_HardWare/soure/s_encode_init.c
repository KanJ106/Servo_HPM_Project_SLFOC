#include "s_encode_init.h"

#if defined(SENSORLESS_CANOPEN_BUILD)
#include "SensorlessShadow.h"
volatile SL_ENCODER_MONITOR g_sl_encoder;
static uint16_t encoder_request_pending;

void SensorlessEncoder_Init(uint16_t type)
{
    SlEncoder_Init(&g_sl_encoder, type);
    encoder_request_pending=0U;
    memset((void *)&g_sl_encoder_timing,0,sizeof(g_sl_encoder_timing));
    if(type!=12U && type!=13U) return;
    EncodeSci_Main();
    EncodeSCI_ResetFIFO;
    g_sl_encoder.initialized=1U;
}

void SensorlessEncoder_Service1ms(uint16_t drive_active)
{
    uint8_t frame[11];
    uint32_t count, i;
    if(!g_sl_encoder.initialized) return;
    g_sl_encoder_timing.sequence++;
    if(encoder_request_pending) {
        g_sl_encoder_timing.frame_request_tick=g_sl_encoder_timing.pending_request_tick;
        g_sl_encoder_timing.frame_read_tick=g_sensorless_shadow.pwm_counter;
        count=EncodeSCI_FIFO_NUM;
        if(count==11U) {
            for(i=0;i<11U;i++) frame[i]=(uint8_t)EncodeSCI_RxData;
            SlEncoder_Sample(&g_sl_encoder,frame,11U,drive_active);
        } else {
            SlEncoder_Sample(&g_sl_encoder,0,count,drive_active);
        }
    }
    /* Discard partial/unsolicited data; exactly one read request per service.
     * No legacy RT_Process: it also contains calibration/zeroing commands. */
    g_sl_encoder_timing.frame_tick_ms=g_sl_encoder.tick_ms;
    EncodeSCI_ResetFIFO;
    g_sl_encoder_timing.pending_request_tick=g_sensorless_shadow.pwm_counter;
    EncodeSCI_TxData=0x6AU;
    g_sl_encoder.requests++;
    encoder_request_pending=1U;
    g_sl_encoder_timing.sequence++;
}
#endif

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
    #if HARDWARE_VER_SEL == HARDWARE_VER_1
    config.src_freq_in_hz = clock_get_frequency(clock_uart2);
#else
    config.src_freq_in_hz = clock_get_frequency(clock_uart1);
#endif
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



