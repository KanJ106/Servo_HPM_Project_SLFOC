#include "s_analog_init.h"
#include "hpm_sdm_drv.h"
#include "IQmathLib.h"
#include "hpm_gptmr_drv.h"
#include "hpm_acmp_drv.h"

uint16_t ADCBuf[21]= {3276};

#if SERVO_MCU == HPM_6E00

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc0_seq_buff[8];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc1_seq_buff[8];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc2_seq_buff[8];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc3_seq_buff[8];

uint8_t seq_adc0_channel[] = {14,14,14};
uint8_t seq_adc1_channel[] = {10,10,10};
#if CURRENT_MODE == CURRENT_ADC
uint8_t seq_adc2_channel[] = {11,11,11};
#endif
uint8_t seq_adc3_channel[] = {7,1,0,5};

void Init_adc0_clock(void)
{
   uint32_t freq = 0;

   clock_add_to_group(clock_adc0, 0);
   clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);
   freq = clock_get_frequency(clock_adc0);
}

void Init_adc1_clock(void)
{
   uint32_t freq = 0;
   clock_add_to_group(clock_adc1, 0);
   clock_set_adc_source(clock_adc1, clk_adc_src_ahb0);
   freq = clock_get_frequency(clock_adc1);
}

void Init_adc2_clock(void)
{
   uint32_t freq = 0;
   clock_add_to_group(clock_adc2, 0);
   clock_set_adc_source(clock_adc2, clk_adc_src_ahb0);
   freq = clock_get_frequency(clock_adc2);
}

void Init_adc3_clock(void)
{
   uint32_t freq = 0;
   clock_add_to_group(clock_adc3, 0);
   clock_set_adc_source(clock_adc3, clk_adc_src_ahb0);
   freq = clock_get_frequency(clock_adc3);
}

void Init_adc_clock(void)
{
#if CURRENT_MODE == CURRENT_ADC
    Init_adc0_clock();
    Init_adc1_clock();
    Init_adc2_clock();
    Init_adc3_clock();
#endif

#if CURRENT_MODE == CURRENT_SDFM
    Init_adc3_clock();
#endif
}

/**
 * @brief: 初始化ADC基本配置
 * @author: mqb
 */
hpm_stat_t init_common_config(adc16_conversion_mode_t conv_mode)
{
    adc16_config_t cfg;

    /* initialize an ADC instance */
    adc16_get_default_config(&cfg);

    cfg.res            = adc16_res_16_bits;
    cfg.conv_mode      = conv_mode;
    cfg.adc_clk_div    = adc16_clock_divider_4;
    cfg.sel_sync_ahb   = true;

    if (cfg.conv_mode == adc16_conv_mode_sequence ||
        cfg.conv_mode == adc16_conv_mode_preemption) {
        cfg.adc_ahb_en = true;
    }

    #if CURRENT_MODE == CURRENT_ADC
    /* adc16 initialization */
    if (adc16_init(HPM_ADC0, &cfg) == status_success) {
        /* enable irq */
        //intc_m_enable_irq_with_priority(BOARD_APP_ADC16_IRQn, 0);
    } else return status_fail;
    
    /* adc16 initialization */
    if (adc16_init(HPM_ADC1, &cfg) == status_success) 
    {
        //intc_m_enable_irq_with_priority(IRQn_ADC1, 0);
    }
    else return status_fail;

    if (adc16_init(HPM_ADC2, &cfg) == status_success) {
        //intc_m_enable_irq_with_priority(IRQn_ADC2, 0);
        //return status_success;
    } else {
        return status_fail;
    }
    #endif

    if (adc16_init(HPM_ADC3, &cfg) == status_success) {
        //intc_m_enable_irq_with_priority(IRQn_ADC2, 0);
        return status_success;
    } else {
        return status_fail;
    }

    /* adc16 initialization */
}
/**
 * @brief: 配置ADC序列循环模式
 * @author: mqb
 */
void set_sequence_config(ADC16_Type *adc_base, adc16_seq_config_t *adcx_seq_cfg, uint8_t seq_adcx_channel_count, const uint8_t * seq_adcx_channel)
{
    adcx_seq_cfg->seq_len = seq_adcx_channel_count;
    adcx_seq_cfg->cont_en = true;
    adcx_seq_cfg->hw_trig_en = true;
    adcx_seq_cfg->restart_en = false;
    adcx_seq_cfg->sw_trig_en = false;
    for(int i = APP_ADC16_SEQ_START_POS; i < seq_adcx_channel_count; i++)
    {
      adcx_seq_cfg->queue[i].seq_int_en = false;
      adcx_seq_cfg->queue[i].ch = seq_adcx_channel[i];
    }
    adcx_seq_cfg->queue[adcx_seq_cfg->seq_len - 1].seq_int_en = true;
    adc16_set_seq_config(adc_base, adcx_seq_cfg);
}

/**
 * @brief: 初始化ADC的TRGM
 * @author: mqb
 */
void init_trigger_mux(TRGM_Type *ptr, uint8_t input, uint8_t output)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type = trgm_output_same_as_input;

    trgm_output_cfg.input  = input;
    trgm_output_config(ptr, output, &trgm_output_cfg);
}

/**
 * @brief:  初始化ADC采集
 * @author: mqb
 */
void init_sequence_config(void)
{
    #if CURRENT_MODE == CURRENT_ADC
    adc16_seq_config_t adc0_seq_cfg;
    adc16_seq_config_t adc1_seq_cfg;
    adc16_seq_config_t adc2_seq_cfg;
    #endif
    adc16_seq_config_t adc3_seq_cfg;

    #if CURRENT_MODE == CURRENT_ADC
    adc16_dma_config_t adc0_dma_cfg;
    adc16_dma_config_t adc1_dma_cfg;
    adc16_dma_config_t adc2_dma_cfg;
    #endif
    adc16_dma_config_t adc3_dma_cfg;

    adc16_channel_config_t ch_cfg;

    /* get a default channel config */
    adc16_get_channel_default_config(&ch_cfg);

    /* initialize an ADC channel */
    ch_cfg.sample_cycle = APP_ADC16_CH_SAMPLE_CYCLE;

    #if CURRENT_MODE == CURRENT_ADC
    ch_cfg.thshdh = 0xFFFF;
    ch_cfg.thshdl = 0xFFFF;
    for (uint32_t i = 0; i < sizeof(seq_adc0_channel); i++) {
        ch_cfg.ch           = seq_adc0_channel[i];
        adc16_init_channel(HPM_ADC0, &ch_cfg);
    }
    for (uint32_t i = 0; i < sizeof(seq_adc1_channel); i++) {
        ch_cfg.ch           = seq_adc1_channel[i];
        adc16_init_channel(HPM_ADC1, &ch_cfg);
    }
    for (uint32_t i = 0; i < sizeof(seq_adc2_channel); i++) {
        ch_cfg.ch           = seq_adc2_channel[i];
        adc16_init_channel(HPM_ADC2, &ch_cfg);
    }
    #endif
    ch_cfg.thshdh = 0xFFFF;
    ch_cfg.thshdl = 0xFFFF;
    for (uint32_t i = 0; i < sizeof(seq_adc3_channel); i++) {
        ch_cfg.ch           = seq_adc3_channel[i];
        adc16_init_channel(HPM_ADC3, &ch_cfg);
    }
  
    #if CURRENT_MODE == CURRENT_ADC
    set_sequence_config(HPM_ADC0, &adc0_seq_cfg, 
                          sizeof(seq_adc0_channel),seq_adc0_channel);
    set_sequence_config(HPM_ADC1, &adc1_seq_cfg, 
                          sizeof(seq_adc1_channel),seq_adc1_channel);
    set_sequence_config(HPM_ADC2, &adc2_seq_cfg, 
                          sizeof(seq_adc2_channel),seq_adc2_channel);
    #endif
    set_sequence_config(HPM_ADC3, &adc3_seq_cfg, 
                          sizeof(seq_adc3_channel),seq_adc3_channel);

    /* Set a DMA config */
    #if CURRENT_MODE == CURRENT_ADC
    adc0_dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc0_seq_buff);
    adc0_dma_cfg.buff_len_in_4bytes = sizeof(seq_adc0_channel);
    adc0_dma_cfg.stop_en            = false;
    adc0_dma_cfg.stop_pos           = 0;

    adc1_dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc1_seq_buff);
    adc1_dma_cfg.buff_len_in_4bytes = sizeof(seq_adc1_channel);
    adc1_dma_cfg.stop_en            = false;
    adc1_dma_cfg.stop_pos           = 0;

    adc2_dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc2_seq_buff);
    adc2_dma_cfg.buff_len_in_4bytes = sizeof(seq_adc2_channel);
    adc2_dma_cfg.stop_en            = false;
    adc2_dma_cfg.stop_pos           = 0;
    #endif

    adc3_dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(APP_ADC16_CORE, (uint32_t)adc3_seq_buff);
    adc3_dma_cfg.buff_len_in_4bytes = sizeof(seq_adc3_channel);
    adc3_dma_cfg.stop_en            = false;
    adc3_dma_cfg.stop_pos           = 0;

    /* Initialize DMA for the sequence mode */
    #if CURRENT_MODE == CURRENT_ADC
    adc16_init_seq_dma(HPM_ADC0, &adc0_dma_cfg);
    adc16_init_seq_dma(HPM_ADC1, &adc1_dma_cfg);
    adc16_init_seq_dma(HPM_ADC2, &adc2_dma_cfg);
    #endif
    adc16_init_seq_dma(HPM_ADC3, &adc3_dma_cfg);

#if !defined(ADC_SOC_NO_HW_TRIG_SRC) && !defined(__ADC16_USE_SW_TRIG)
    /* Trigger mux initialization */
    #if CURRENT_MODE == CURRENT_ADC
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC0_STRGI);
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC1_STRGI);
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC2_STRGI);
    #endif
    init_trigger_mux(APP_ADC16_HW_TRGM, APP_ADC16_HW_TRGM_IN, TRGM_TRGOCFG_ADC3_STRGI);
#endif
}

/**
 * @brief: 初始化ADC引脚
 * @author: mqb
 */
void Init_adc16_pins(void)
{
    #if CURRENT_MODE == CURRENT_ADC
    HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* U ADC0.IN14/ADC1.IN14 */
    HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* V ADC1.IN10/ADC0.IN10 */
    HPM_IOC->PAD[IOC_PAD_PF19].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* W ADC2.IN11/ADC3.IN11 */
    #endif

    HPM_IOC->PAD[IOC_PAD_PF31].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* UDC ADC2.IN7/ADC3.IN7 */
    HPM_IOC->PAD[IOC_PAD_PF26].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* UDC ADC3.IN1/ADC2.IN1 */
    HPM_IOC->PAD[IOC_PAD_PF27].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* UDC ADC2.IN0/ADC3.IN0 */
    HPM_IOC->PAD[IOC_PAD_PF28].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;        /* UDC ADC2.IN5/ADC3.IN5 */
}

#define ACMPSEL  HPM_ACMP2
#define ACMPCLK  clock_acmp2
#define ACMPIRQ  IRQn_ACMP2_0
#define ACMPNEL  ACMP_CHANNEL_CHN0
ACMP_Type *TestAcmp;
TRGM_Type *TestTRGM;
void Acmp_Init(void)
{
    acmp_channel_config_t acmp_channel_configure;
    TestAcmp = ACMPSEL;

    clock_add_to_group(ACMPCLK, HPM_CORE0 & 0x1);
    /* configure to CMP0_INN4 function */
    HPM_IOC->PAD[IOC_PAD_PF24].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

    HPM_IOC->PAD[IOC_PAD_PF04].FUNC_CTL = IOC_PF04_FUNC_CTL_TRGM_P_04;
    HPM_IOC->PAD[IOC_PAD_PF04].PAD_CTL  = IOC_PAD_PAD_CTL_PE_SET(1) |  // 上拉使能
                                          IOC_PAD_PAD_CTL_PS_SET(1);   // 上拉

    acmp_channel_get_default_config(HPM_ACMP2, &acmp_channel_configure);

    acmp_channel_configure.plus_input = ACMP_INPUT_ANALOG_5;    //INP 电压高于 INN 电压时，比较结果为 1
    acmp_channel_configure.minus_input =  ACMP_INPUT_DAC_OUT;   //INN

    acmp_channel_configure.enable_dac = true; 
    acmp_channel_configure.enable_cmp_output = true;
    acmp_channel_configure.hyst_level = ACMP_HYST_LEVEL_3;
    //acmp_channel_configure.enable_hpmode = true;

    acmp_channel_configure.bypass_filter = false; //滤波器开启
    acmp_channel_configure.filter_mode = ACMP_FILTER_MODE_CHANGE_AFTER_FILTER; //滤波器延时
    acmp_channel_configure.filter_length = 20;//200MHZ --> 100ns

    acmp_channel_config(HPM_ACMP2, ACMP_CHANNEL_CHN0, &acmp_channel_configure, true);                                                         
    acmp_channel_config_dac(HPM_ACMP2, ACMP_CHANNEL_CHN0,128);  
    acmp_channel_clear_status(HPM_ACMP2, ACMP_CHANNEL_CHN0, ACMP_EVENT_RISING_EDGE);

    TestTRGM = HPM_TRGM0;
    trgm_output_t trgm_output_cfg;
    trgm_output_cfg.invert = true;
    trgm_output_cfg.type   = trgm_output_same_as_input;
    trgm_output_cfg.input  = HPM_TRGM0_INPUT_SRC_ACMP2_CH0_OUT;
    trgm_output_config(HPM_TRGM0, TRGM_TRGOCFG_TRGM_P_04, &trgm_output_cfg);

    trgm_filter_t trgm_filter_cfg;
    trgm_filter_cfg.mode = trgm_filter_mode_delay;
    trgm_filter_cfg.filter_length = 20; //200MHZ --> 100ns
    trgm_input_filter_config(HPM_TRGM0, HPM_TRGM0_INPUT_SRC_ACMP2_CH0_OUT, &trgm_filter_cfg);

    trgm_enable_io_output(HPM_TRGM0, 0x10);
}

/**
 * @brief: ADC初始化
 * @author: mqb
 */
void Adc_Init(void)
{
    Init_adc16_pins();  
    
    Init_adc_clock();

    init_common_config(2);
    init_sequence_config();

    Acmp_Init();
}

/*
MCU内置Sigma_Delta初始化

*/
SDM_Type * TESTSDM;
void Sigma_Delta_Init(void)
{
    sdm_control_t control;
    sdm_filter_config_t filter_config;
    gptmr_channel_config_t config;

    HPM_IOC->PAD[IOC_PAD_PF17].FUNC_CTL = IOC_PF17_FUNC_CTL_SDM0_CLK_0;
    HPM_IOC->PAD[IOC_PAD_PF16].FUNC_CTL = IOC_PF16_FUNC_CTL_SDM0_DAT_0;

    HPM_IOC->PAD[IOC_PAD_PF22].FUNC_CTL = IOC_PF22_FUNC_CTL_SDM0_CLK_2;
    HPM_IOC->PAD[IOC_PAD_PF21].FUNC_CTL = IOC_PF21_FUNC_CTL_SDM0_DAT_2;

    HPM_IOC->PAD[IOC_PAD_PF10].FUNC_CTL = IOC_PF10_FUNC_CTL_GPTMR4_COMP_2;

    clock_set_source_divider(clock_gptmr4, clk_src_pll1_clk0, 10);//时钟80M
    clock_add_to_group(clock_gptmr4, 0);
    gptmr_channel_get_default_config(HPM_GPTMR4, &config);
    //gptmr_freq = clock_get_frequency(clock_gptmr4); //时钟80M

    config.reload = 4;
    config.cmp_initial_polarity_high = 1;
    gptmr_stop_counter(HPM_GPTMR4, GPTMR_PWM_SYNC_CHANNEL);
    gptmr_channel_config(HPM_GPTMR4, GPTMR_PWM_SYNC_CHANNEL, &config, false);
    gptmr_channel_reset_count(HPM_GPTMR4, GPTMR_PWM_SYNC_CHANNEL);

    HPM_GPTMR4->CHANNEL[2].CMP[0] = 1;
    HPM_GPTMR4->CHANNEL[2].CMP[1] = 4;
    gptmr_start_counter(HPM_GPTMR4, GPTMR_PWM_SYNC_CHANNEL);

    clock_add_to_group(CurSample_SDMCLK, 0);
   /* config sdm module: clock signal sync, data signal sync and interrupt */
    sdm_get_default_module_control(CurSample_SDM, &control);
    sdm_init_module(CurSample_SDM, &control);

    /* config channel: sampling mode and interrupt */
    sdm_channel_common_config_t ch_config;
    ch_config.sampling_mode = sdm_sampling_rising_clk_edge;
    ch_config.enable_err_interrupt = false;
    ch_config.enable_data_ready_interrupt = false;
    sdm_config_channel_common_setting(CurSample_SDM, SDM_Channel1, &ch_config);
    sdm_config_channel_common_setting(CurSample_SDM, SDM_Channel2, &ch_config);

    /* config filter: filter type, oversampling rate, output data length, etc. */
    sdm_get_channel_default_filter_config(CurSample_SDM, &filter_config);
    filter_config.filter_type = sdm_filter_sinc3;
    filter_config.oversampling_rate = 128; /* 1- 256 */
    filter_config.ignore_invalid_samples = 2;
    filter_config.output_32bit = 0;   //mqb 输出16位的数据
    filter_config.output_offset = 6;  //mqb 偏移6位
    sdm_config_channel_filter(CurSample_SDM, SDM_Channel1, &filter_config);
    sdm_config_channel_filter(CurSample_SDM, SDM_Channel2, &filter_config);

    sdm_enable_channel(CurSample_SDM, SDM_Channel1, true);
    sdm_enable_channel(CurSample_SDM, SDM_Channel2, true);

    TESTSDM = CurSample_SDM;
}

ATTR_RAMFUNC
void Get_AdcValue(void)
{
#if CURRENT_MODE == CURRENT_ADC 
    uint16_t buff[3];
    uint16_t max,min,temp16;
    uint32_t sum = 0;
    int32_t  Adcout;
    int16    out;

    buff[0] = (uint16_t)(adc0_seq_buff[0] & 0x0000FFFF);
    buff[1] = (uint16_t)(adc0_seq_buff[1] & 0x0000FFFF);
    buff[2] = (uint16_t)(adc0_seq_buff[2] & 0x0000FFFF);

    max = buff[0]>buff[1]?(buff[0]>buff[2]?buff[0]:buff[2]):(buff[1]>buff[2]?buff[1]:buff[2]);
    min = buff[0]<buff[1]?(buff[0]<buff[2]?buff[0]:buff[2]):(buff[1]<buff[2]?buff[1]:buff[2]);
    sum = buff[0] + buff[1] + buff[2];
    Adcout = sum - max - min - 32768;
    out = _IQsat(Adcout,32767,-32768);
    ADCBuf[0] = (uint16_t)out;  //U

    buff[0] = (uint16_t)(adc1_seq_buff[0] & 0x0000FFFF);
    buff[1] = (uint16_t)(adc1_seq_buff[1] & 0x0000FFFF);
    buff[2] = (uint16_t)(adc1_seq_buff[2] & 0x0000FFFF);

    max = buff[0]>buff[1]?(buff[0]>buff[2]?buff[0]:buff[2]):(buff[1]>buff[2]?buff[1]:buff[2]);
    min = buff[0]<buff[1]?(buff[0]<buff[2]?buff[0]:buff[2]):(buff[1]<buff[2]?buff[1]:buff[2]);
    sum = buff[0] + buff[1] + buff[2];
    Adcout = sum - max - min - 32768;
    out = _IQsat(Adcout,32767,-32768);
    ADCBuf[1] = (uint16_t)out;  //V

    buff[0] = (uint16_t)(adc2_seq_buff[0] & 0x0000FFFF);
    buff[1] = (uint16_t)(adc2_seq_buff[1] & 0x0000FFFF);
    buff[2] = (uint16_t)(adc2_seq_buff[2] & 0x0000FFFF);

    max = buff[0]>buff[1]?(buff[0]>buff[2]?buff[0]:buff[2]):(buff[1]>buff[2]?buff[1]:buff[2]);
    min = buff[0]<buff[1]?(buff[0]<buff[2]?buff[0]:buff[2]):(buff[1]<buff[2]?buff[1]:buff[2]);
    sum = buff[0] + buff[1] + buff[2];
    Adcout = sum - max - min - 32768;
    out = _IQsat(Adcout,32767,-32768);
    ADCBuf[2] = (uint16_t)out;  //W

    ADCBuf[8] = (uint16_t)(adc3_seq_buff[1] & 0x0000FFFF);     //IGBT
    ADCBuf[9] = 32768;                                         //1.65V
    ADCBuf[10] = (uint16_t)(adc3_seq_buff[0] & 0x0000FFFF);    //UDC
    ADCBuf[12] = 0;                                            //0
    ADCBuf[13] = 65535;                                        //Motor
#endif

#if CURRENT_MODE == CURRENT_SDFM 
    int16_t temp1,temp2,temp3;
    int32_t temp4;

    temp1 = CurSample_SDM->CH[SDM_Channel1].SDATA >> 16;
    temp1 = -temp1;
    ADCBuf[0] = (uint16_t)temp1;

    temp2 = CurSample_SDM->CH[SDM_Channel2].SDATA >> 16;
    temp2 = -temp2;
    ADCBuf[1] = (uint16_t)temp2;

    temp4 = -(temp1 + temp2);
    temp3 = _IQsat(temp4,32767,-32768);
    ADCBuf[2] = (uint16_t)temp3;

    ADCBuf[8]  = (uint16_t)(adc3_seq_buff[1] & 0x0000FFFF);    //IGBT
    ADCBuf[9]  = 32768;//(uint16_t)(adc3_seq_buff[3] & 0x0000FFFF);    //1.65V
    ADCBuf[10] = (uint16_t)(adc3_seq_buff[0] & 0x0000FFFF);    //UDC
    ADCBuf[12] = 0;                                            //0
    ADCBuf[13] = (uint16_t)(adc3_seq_buff[2] & 0x0000FFFF);    //Motor
#endif
}

#else
/*
MCU内置ADC初始化

*/
void Adc_Init(void)
{
  
}

/*
MCU内置Sigma_Delta初始化

*/
void Sigma_Delta_Init(void)
{

}
#endif






