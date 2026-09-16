/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "s_pwm_init.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
uint16_t EPWM_PRD_DEFAULT = 0;    
uint32_t ESPD_PRD_DEFAULT = 0; 
uint32_t EPOS_PRD_DEFAULT = 0;   
uint16_t EPWM_DB = 0; 

#if SERVO_MCU != NO_MCU
uint32_t reload;                 // PWM重载值
PWMV2_Type *teatPwm;
PWMV2_Type *teatPwm1;
/**
 * @brief: 初始化pwm故障保护引脚
 * @author: mqb
 */
void Init_pwm_fault_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PF05].FUNC_CTL = IOC_PF05_FUNC_CTL_TRGM_P_05;
    HPM_IOC->PAD[IOC_PAD_PF05].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) |  // 上拉使能
                                         IOC_PAD_PAD_CTL_PS_SET(1);   // 上拉
}
/**
 * @brief: 初始化pwm引脚
 * @author: mqb
 */
void Init_pwm_pins(void)
{
#if HARDWARE_VER_SEL == HARDWARE_VER_0
    HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
    HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
    HPM_IOC->PAD[IOC_PAD_PE10].FUNC_CTL = IOC_PE10_FUNC_CTL_PWM1_P_2;
    HPM_IOC->PAD[IOC_PAD_PE11].FUNC_CTL = IOC_PE11_FUNC_CTL_PWM1_P_3;
    HPM_IOC->PAD[IOC_PAD_PE12].FUNC_CTL = IOC_PE12_FUNC_CTL_PWM1_P_4;
    HPM_IOC->PAD[IOC_PAD_PE13].FUNC_CTL = IOC_PE13_FUNC_CTL_PWM1_P_5;
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
    HPM_IOC->PAD[IOC_PAD_PE00].FUNC_CTL = IOC_PE00_FUNC_CTL_PWM0_P_0;
    HPM_IOC->PAD[IOC_PAD_PE01].FUNC_CTL = IOC_PE01_FUNC_CTL_PWM0_P_1;
    HPM_IOC->PAD[IOC_PAD_PE02].FUNC_CTL = IOC_PE02_FUNC_CTL_PWM0_P_2;
    HPM_IOC->PAD[IOC_PAD_PE03].FUNC_CTL = IOC_PE03_FUNC_CTL_PWM0_P_3;
    HPM_IOC->PAD[IOC_PAD_PE04].FUNC_CTL = IOC_PE04_FUNC_CTL_PWM0_P_4;
    HPM_IOC->PAD[IOC_PAD_PE05].FUNC_CTL = IOC_PE05_FUNC_CTL_PWM0_P_5;
#else
    HPM_IOC->PAD[IOC_PAD_PE08].FUNC_CTL = IOC_PE08_FUNC_CTL_PWM1_P_0;
    HPM_IOC->PAD[IOC_PAD_PE09].FUNC_CTL = IOC_PE09_FUNC_CTL_PWM1_P_1;
    HPM_IOC->PAD[IOC_PAD_PE10].FUNC_CTL = IOC_PE10_FUNC_CTL_PWM1_P_2;
    HPM_IOC->PAD[IOC_PAD_PE11].FUNC_CTL = IOC_PE11_FUNC_CTL_PWM1_P_3;
    HPM_IOC->PAD[IOC_PAD_PE12].FUNC_CTL = IOC_PE12_FUNC_CTL_PWM1_P_4;
    HPM_IOC->PAD[IOC_PAD_PE13].FUNC_CTL = IOC_PE13_FUNC_CTL_PWM1_P_5;
#endif
}


/**
 * @brief: Enable pwm counter
 * @author: mqb
 */
static inline void pwmv2_enable_counter1(PWMV2_Type *pwm_x, pwm_counter_t counter)
{
    pwm_x->CNT_GLBCFG |= PWMV2_CNT_GLBCFG_TIMER_ENABLE_SET((0xF << counter));
}
/**
 * @brief: Disable pwm counter
 * @author: mqb
 */
static inline void pwmv2_disable_counter1(PWMV2_Type *pwm_x, pwm_counter_t counter)
{
    pwm_x->CNT_GLBCFG &= ~PWMV2_CNT_GLBCFG_TIMER_ENABLE_SET((0xF << counter));
}
/**
 * @brief: start pwm output
 * @author: mqb
 */
static inline void pwmv2_start_pwm_output1(PWMV2_Type *pwm_x, pwm_counter_t counter)
{
    pwm_x->CNT_GLBCFG |= PWMV2_CNT_GLBCFG_CNT_SW_START_SET((7 << counter));
}


/**
 * @brief: PWM故障保护配置
 * @author: mqb
 */
void pwm_fault_async(void)
{
    pwmv2_async_fault_source_config_t fault_cfg;
    fault_cfg.async_signal_from_pad_index = 5;
    fault_cfg.fault_async_pad_level = pad_fault_active_low;
    
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN0, &fault_cfg);
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN1, &fault_cfg);
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN2, &fault_cfg);
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN3, &fault_cfg);
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN4, &fault_cfg);
    pwmv2_config_async_fault_source(PWM, PWM_OUTPUT_PIN5, &fault_cfg);
    
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN0, pwm_fault_output_0);
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN1, pwm_fault_output_0);
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN2, pwm_fault_output_0);
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN3, pwm_fault_output_0);
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN4, pwm_fault_output_0);
    pwmv2_set_fault_mode(PWM, PWM_OUTPUT_PIN5, pwm_fault_output_0);

    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN0, pwm_fault_recovery_on_fault_clear);
    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN1, pwm_fault_recovery_on_fault_clear);
    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN2, pwm_fault_recovery_on_fault_clear);
    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN3, pwm_fault_recovery_on_fault_clear);
    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN4, pwm_fault_recovery_on_fault_clear);
    pwmv2_set_fault_recovery_time(PWM, PWM_OUTPUT_PIN5, pwm_fault_recovery_on_fault_clear);
    
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN0);
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN1);
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN2);
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN3);
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN4);
    pwmv2_enable_async_fault(PWM, PWM_OUTPUT_PIN5);

    pwmv2_enable_fault_irq(PWM, PWM_OUTPUT_PIN0);
}


/**
 * @brief: 故障保护引脚
 * @author: mqb
 */
void Set_WritePin_gpiob5(uint8_t level)
{
    if(level == 0)
    {
       HPM_IOC->PAD[IOC_PAD_PB05].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) |  // 上拉使能
                                           IOC_PAD_PAD_CTL_PS_SET(0);   // 上拉
    }
    else if(1 == level)
    {
       HPM_IOC->PAD[IOC_PAD_PB05].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) |  // 上拉使能
                                           IOC_PAD_PAD_CTL_PS_SET(1);   // 上拉
    }
}


/******************************************

U_PWM CH:0/1; PIN:PE8/9  ; SHADOW:1/2; CMP:0/1; COUNTER:counter0; PERIOD:SHADOW0   
V_PWM CH:2/3; PIN:PE10/11; SHADOW:6/7; CMP:4/5; COUNTER:counter1; PERIOD:SHADOW0
W_PWM CH:4/5; PIN:PE12/13; SHADOW:8/9; CMP:8/9; COUNTER:counter2; PERIOD:SHADOW0

*******************************************/
/**
 * @brief: 配置PWM生成互补波形和中断
 * @author: mqb
 */
void UVW_PWM_With_CMPinterrupt(uint32_t U_duty, uint32_t V_duty, uint32_t W_duty,uint16_t db)
{
    pwmv2_deinit(PWM);
    pwmv2_shadow_register_unlock(PWM);
    
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(0), reload, 0, false);               //PWM1 CH0 and CH1 period
    //PWMV2 CH 0/1  PE8/9     U
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(1), (reload - U_duty) >> 1, 0, false);
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(2), (reload + U_duty) >> 1, 0, false);
    //PWMV2 CH 2/3  PE10/11   V
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(6), (reload - V_duty) >> 1, 0, false);
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(7), (reload + V_duty) >> 1, 0, false);
    //PWMV2 CH 4/5  PE12/13   W
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(8), (reload - W_duty) >> 1, 0, false);
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(9), (reload + W_duty) >> 1, 0, false);
    
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(4), reload >> 1, 0, false);          // ADC 触发
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(3), reload >> 1, 0, false);          // 16K interrupt 
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(5), (reload >> 1) - 1000, 0, false); // The interrupt that occurred 5 us before the 16K interrupt

    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(10), 0, 0, false);              //uart
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(11), (reload), 0, false);       //uart

    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(12), 400, 0, false);            //CAN

    pwmv2_counter_select_data_offset_from_shadow_value(PWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_select_data_offset_from_shadow_value(PWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_select_data_offset_from_shadow_value(PWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_select_data_offset_from_shadow_value(PWM, pwm_counter_3, PWMV2_SHADOW_INDEX(0));

    pwmv2_set_reload_update_time(PWM, pwm_counter_0, pwm_reload_update_on_reload); 
    pwmv2_set_reload_update_time(PWM, pwm_counter_1, pwm_reload_update_on_reload);
    pwmv2_set_reload_update_time(PWM, pwm_counter_2, pwm_reload_update_on_reload);
    pwmv2_set_reload_update_time(PWM, pwm_counter_3, pwm_reload_update_on_reload);

    pwmv2_counter_burst_disable(PWM, pwm_counter_0);
    pwmv2_counter_burst_disable(PWM, pwm_counter_1);
    pwmv2_counter_burst_disable(PWM, pwm_counter_2);
    pwmv2_counter_burst_disable(PWM, pwm_counter_3);
    
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(16), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(3));     //16K环路
    pwmv2_cmp_select_counter(PWM, PWMV2_CMP_INDEX(16), pwm_counter_3);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(16), pwm_shadow_register_update_on_modify);

    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(17), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(4));     //ADC触发
    pwmv2_cmp_select_counter(PWM, PWMV2_CMP_INDEX(17), pwm_counter_0); //
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(17), pwm_shadow_register_update_on_modify);
    pwmv2_set_trigout_cmp_index(PWM, 0, PWMV2_CMP_INDEX(17));

    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(18), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(12));   //canfd 触发2us比较中断
    pwmv2_cmp_select_counter(PWM, PWMV2_CMP_INDEX(18), pwm_counter_3);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(18), pwm_shadow_register_update_on_modify);
    
    if(db < 50)  db = 50;
    else if(db > 300) db = 300;
    db = db * 2;

    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN0, db);  // Dead zone of channel 0
    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN1, db);  // Dead zone of channel 1
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN0);                // pair mode of channel 0
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN1);                // pair mode of channel 1

    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN2, db);  
    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN3, db);  
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN2);                
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN3);                

    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN4, db);  
    pwmv2_set_dead_area(PWM, PWM_OUTPUT_PIN5, db);  
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN4);                
    pwmv2_enable_pair_mode(PWM, PWM_OUTPUT_PIN5);                
    
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(0), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(1));//CMP a
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(1), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(2));//CMP b
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(0), pwm_shadow_register_update_on_reload);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(1), pwm_shadow_register_update_on_reload);
    
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(4), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(6));
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(5), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(7));
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(4), pwm_shadow_register_update_on_reload);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(5), pwm_shadow_register_update_on_reload);
    
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(8), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(8));
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(9), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(9));
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(8), pwm_shadow_register_update_on_reload);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(9), pwm_shadow_register_update_on_reload);

    //pwmv2_select_cmp_source(HPM_PWM1, 19, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(10));//uart
    //pwmv2_cmp_select_counter(PWM, PWMV2_CMP_INDEX(19), pwm_counter_3); //uart
    //pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(19), pwm_shadow_register_update_on_modify);

    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN0);
    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN1);
    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN2);
    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN3);
    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN4);
    pwmv2_disable_four_cmp(PWM, PWM_OUTPUT_PIN5);
    
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN0);
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN1);
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN2);
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN3);
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN4);
    pwmv2_channel_enable_output(PWM, PWM_OUTPUT_PIN5);
    
    pwmv2_clear_cmp_irq_status(PWM, 0xFFFFFF);
    pwmv2_enable_cmp_irq(PWM, PWMV2_CMP_INDEX(16)); //CTRL_LOOP
    pwmv2_enable_cmp_irq(PWM, PWMV2_CMP_INDEX(18)); //CAN 
    intc_m_enable_irq_with_priority(BOARD_APP_PWM_IRQ, 5);
    
    //pwmv2_shadow_register_lock(PWM);
}

void Pwmv2_Pos_Loop(void)
{
    uint32_t Cnt = 200000L - 1;
    EPOS_PRD_DEFAULT = Cnt;
    pwmv2_deinit(PWM1);
    pwmv2_shadow_register_unlock(PWM1);
    
    pwmv2_set_shadow_val(PWM1, PWMV2_SHADOW_INDEX(0), Cnt, 0, false);                //PWM1 CH0 and CH1 period
    pwmv2_set_shadow_val(PWM1, PWMV2_SHADOW_INDEX(3), Cnt - 12500, 0, false);           // 16K interrupt 

    pwmv2_counter_select_data_offset_from_shadow_value(PWM1, pwm_counter_3, PWMV2_SHADOW_INDEX(0));
    pwmv2_set_reload_update_time(PWM1, pwm_counter_3, pwm_reload_update_on_reload);
    pwmv2_counter_burst_disable(PWM1, pwm_counter_3);
    
    pwmv2_select_cmp_source(PWM1, PWMV2_CMP_INDEX(16), cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(3));     //16K环路
    pwmv2_cmp_select_counter(PWM1, PWMV2_CMP_INDEX(16), pwm_counter_3);
    pwmv2_cmp_update_trig_time(PWM1, PWMV2_CMP_INDEX(16), pwm_shadow_register_update_on_reload);
    
    pwmv2_clear_cmp_irq_status(PWM1, 0xFFFFFF);
    pwmv2_enable_cmp_irq(PWM1, PWMV2_CMP_INDEX(16));  //CTRL_LOOP
    intc_m_enable_irq_with_priority(Pos_Loop_IRQ, 5);
    
    //pwmv2_shadow_register_lock(PWM);
}
/**
 * @brief: This function initializes the MTU3 Unit0 module.
 * @author: mqb
 */
void PWMV2_Init(uint16_t pwmmode,uint16_t db,uint16_t Pwmflag)  
{
    uint32_t freq = clock_get_frequency(PWM_CLOCK_NAME);
    if(4 == pwmmode)
    {
        reload = freq / 16000 - 1;  //reload = 12500, pwm freq 16K
    }
    else if(5 == pwmmode) 
    {
        reload = freq / 20000 - 1;  //reload = 10000, pwm freq 20K
    }
    else
    {
        reload = freq / 16000 - 1;  //reload = 12500, pwm freq 16K
    }
    EPWM_PRD_DEFAULT = reload;
    teatPwm1 = PWM1;
    teatPwm = PWM;

    Init_pwm_pins();

    Init_pwm_fault_pins();
    UVW_PWM_With_CMPinterrupt(reload >> 1, reload >> 1, reload >> 1,db);
    Pwmv2_Pos_Loop();
    pwm_fault_async();

    pwmv2_enable_force_by_software(PWM, 0);
    pwmv2_enable_force_by_software(PWM, 1);
    pwmv2_enable_force_by_software(PWM, 2);
    pwmv2_enable_force_by_software(PWM, 3);
    pwmv2_enable_force_by_software(PWM, 4);
    pwmv2_enable_force_by_software(PWM, 5);

    pwmv2_force_update_time_by_shadow(PWM, 0, pwm_force_update_shadow_at_reload);
    pwmv2_force_update_time_by_shadow(PWM, 1, pwm_force_update_shadow_at_reload);
    pwmv2_force_update_time_by_shadow(PWM, 2, pwm_force_update_shadow_at_reload);
    pwmv2_force_update_time_by_shadow(PWM, 3, pwm_force_update_shadow_at_reload);
    pwmv2_force_update_time_by_shadow(PWM, 4, pwm_force_update_shadow_at_reload);
    pwmv2_force_update_time_by_shadow(PWM, 5, pwm_force_update_shadow_at_reload);

    if(Pwmflag == 55)
    {
        pwmv2_force_output(PWM,0, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,1, pwm_force_output_1, 0); //输出低电平
        pwmv2_force_output(PWM,2, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,3, pwm_force_output_1, 0); //输出低电平
        pwmv2_force_output(PWM,4, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,5, pwm_force_output_1, 0); //输出低电平 
    }
    else
    {
        pwmv2_force_output(PWM,0, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,1, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,2, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,3, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,4, pwm_force_output_0, 0); //输出低电平
        pwmv2_force_output(PWM,5, pwm_force_output_0, 0); //输出低电平 
    }

    pwmv2_set_force_update_time(PWM, 0, pwm_force_at_reload);
    pwmv2_set_force_update_time(PWM, 1, pwm_force_at_reload);
    pwmv2_set_force_update_time(PWM, 2, pwm_force_at_reload);
    pwmv2_set_force_update_time(PWM, 3, pwm_force_at_reload);
    pwmv2_set_force_update_time(PWM, 4, pwm_force_at_reload);
    pwmv2_set_force_update_time(PWM, 5, pwm_force_at_reload);

    pwmv2_enable_software_force(PWM, 0);
    pwmv2_enable_software_force(PWM, 1);
    pwmv2_enable_software_force(PWM, 2);
    pwmv2_enable_software_force(PWM, 3);
    pwmv2_enable_software_force(PWM, 4);
    pwmv2_enable_software_force(PWM, 5);

    pwmv2_start_pwm_output1(PWM, pwm_counter_0);
}


/************************************************************************
 * 函数名：void PWMOutDisable(void)
 * 描述：G1~G6输出禁止
 * 输入：无
 * 输出：无
 * 返回值：无
 ************************************************************************/
ATTR_RAMFUNC
void MTU3PWMOff(void)
{    
   PWM->GLB_CTRL |= 0x003F0000;
}

ATTR_RAMFUNC
void MTU3PWMOffDB(void)
{    

}

/************************************************************************
 * 函数名：void PWMOutEnable(void)
 * 描述：G1~G6输出允许
 * 输入：无
 * 输出：无
 * 返回值：无
 ************************************************************************/
ATTR_RAMFUNC
void PWMOutEnable(void)
{ 
    PWM->GLB_CTRL &= 0xFFC0FFFF;
}

void PWMCounterStart(void)
{
    pwmv2_enable_counter1(PWM, pwm_counter_0);
    #if SERVOTYPE == SERVO_CANOPEN 
    pwmv2_enable_counter1(PWM1, pwm_counter_0);
    #endif
}

// 0  ---> H 为低              L 为高   0 ~ 100         H 为低
// T/2---> H 低电平时间为死区   L 为低   T/2 ~ T/2 -100  L 为低
//uduty为上桥高电平时间，最大为T/2  T为周期计数器值
ATTR_RAMFUNC
void PwmDutyUpdata(uint16_t uduty,uint16_t vduty,uint16_t wduty,uint16_t T)
{
    uint32_t uh,ul,vh,vl,wh,wl;

    uh = T - uduty;
    ul = T + uduty;

    vh = T - vduty;
    vl = T + vduty;

    wh = T - wduty;
    wl = T + wduty;

    PWM->SHADOW_VAL[1] = uh << 8;   // UH
    PWM->SHADOW_VAL[2] = ul << 8;   // UL

    PWM->SHADOW_VAL[6] = vh << 8;   // VH
    PWM->SHADOW_VAL[7] = vl << 8;   // VL

    PWM->SHADOW_VAL[8] = wh << 8;   // WH
    PWM->SHADOW_VAL[9] = wl << 8;   // WL
}

ATTR_RAMFUNC
void SetG7Duty(uint16_t Duty)
{
    (void)Duty;
}
#else
uint16_t PWMA_SET_COMPAREVALUE;
uint16_t PWMB_SET_COMPAREVALUE;
uint16_t PWMC_SET_COMPAREVALUE;
/***********************************************************************************************************************
* Function Name: PWMV2_Init
* Description  : This function initializes the MTU3 Unit0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void PWMV2_Init(uint16_t pwmmode)
{
    (void)pwmmode;
}

/************************************************************************
 * 函数名：void PWMOutDisable(void)
 * 描述：G1~G6输出禁止
 * 输入：无
 * 输出：无
 * 返回值：无
 ************************************************************************/
void MTU3PWMOff(void)
{    

}

void MTU3PWMOffDB(void)
{    

}

/************************************************************************
 * 函数名：void PWMOutEnable(void)
 * 描述：G1~G6输出允许
 * 输入：无
 * 输出：无
 * 返回值：无
 ************************************************************************/
void PWMOutEnable(void)
{ 

}

void SetG7Duty(uint16_t Duty)
{
    (void)Duty;
}

#endif
