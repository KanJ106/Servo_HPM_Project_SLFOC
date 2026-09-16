#include "Hpm_ISR.h"
#include "hpm_interrupt.h"
#include "hpm_soc_irq.h"
#include "ecatappl.h"
#include "ecat_escinit.h"
#include "cia402appl.h"
#include "Pos_FullClosedLoop.h"
#include "Drive.h"
#include "MCU_Fw.h"
#include "r_can_api.h"
#include "SV_CanFirm.h"
#include "SV_CanbusCtrl.h"
#include "s_tsensor_init.h"
#include "SV_ModbusInit.h"
#include "SV_ModbusFirm.h"

uint16_t testpwmcnt;

extern void AppTime1Ms(void);
extern void EcatDcOffsetCacl(void);
extern void EcatDcCalibCacl(void);
extern void Updata_EcatCmd(void);
extern void Updata_EcatFb(void);
extern void OscilloscopeSampling_us(void);
extern void OscilloscopeSampling_Fault(void);

extern void CanSync_init(void);
extern void CanSyncOffsetCacl(void);
extern void CanSyncCalibCacl(void);
extern void Tor_SensorUart_PerMod(void);
extern void CanopenProccessFromIsr(void);

extern volatile uint8_t SnycCmdFlag;
extern uint16_t SmPeriodtime;
extern uint16_t SmDcOffsettime;
volatile uint8_t CanxPosFlag = 0;
/***********************************************************************************************************************
* Function Name: 
* Description  : This function is the interrupt service routine for Timer 3.
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
SDK_DECLARE_EXT_ISR_M(BOARD_APP_PWM_IRQ, CMP_isr_pwm)
void CMP_isr_pwm(void)
{
    volatile uint32_t flags;

    flags = pwmv2_get_cmp_irq_status(PWM);
    pwmv2_clear_cmp_irq_status(PWM, flags);

    if(PWM->IRQ_EN_FAULT && PWM->IRQ_STS_FAULT)
    {
        FaultPrtt_FaultInterface(SC_ERR);// 

        pwmv2_disable_fault_irq(PWM, PWM_OUTPUT_PIN0);
    }
    
    // 16K interrupt
    if (flags & PWM_IRQ_STS_CMP(16)) 
    {
        #if SERVOTYPE == SERVO_ETHERCAT
        EcatDcCalibCacl();
        #endif

        #if SERVOTYPE == SERVO_CAN
        CanSyncCalibCacl();
        #endif

        #if SERVOTYPE == SERVO_CANOPEN
        CanopenProccessFromIsr();  //PDO
        //CanSyncCalibCacl();
        #endif

        if(RamPC->TorFullSel > 0)
        {
            Tor_Sensor_Precess();
        }
        
        CtrLoop_CalcFirst();
        
        OscilloscopeSampling_us();
        OscilloscopeSampling_Fault();
    }

    #if SERVOTYPE == SERVO_CAN
    //mqb用于处理canfd的接收和反馈
    if(flags & PWM_IRQ_STS_CMP(18))
    {
        Can_RxProcess(&s_can_rx_buf); 
        Can_TxCheckProcess();                    
        CanBusCtrl_Process();                            
        Can_TxProcess();                                 
    }
    #endif

    #if SERVOTYPE == SERVO_MODBUS
    ModBus_Process();
    #endif

    //testpwmcnt = PWM->CNT_VAL[0] >> 8;

    //// 5 us before the 16K interrupt
    //if (flags & PWM_IRQ_STS_CMP(18)) 
    //{
    //    testpwmcnt++;
    //}
}

SDK_DECLARE_EXT_ISR_M(Pos_Loop_IRQ, Pos_Loop_IRQ1)
void Pos_Loop_IRQ1(void)
{
    volatile uint32_t flags;

    testpwmcnt = PWM->CNT_VAL[0] >> 8;
    CanxPosFlag = 1;

    flags = pwmv2_get_cmp_irq_status(PWM1);
    pwmv2_clear_cmp_irq_status(PWM1, flags);
}

#if SERVOTYPE == SERVO_ETHERCAT
/* ECAT PDI IRQ handler */
SDK_DECLARE_EXT_ISR_M(IRQn_ESC, ecat_pdi_isr)
void ecat_pdi_isr(void)
{
    SmPeriodtime = (uint16_t)ReadCount();
    ResetCount();
    PDI_Isr();
}

/* ECAT SYNC0 IRQ handler */
SDK_DECLARE_EXT_ISR_M(IRQn_ESC_SYNC0, ecat_sync0_isr)
void ecat_sync0_isr(void)
{
    volatile UINT16 SyncState = 0;
    HW_EscReadWord(SyncState, 0x098C); /* Read Sync0/1 Status Register to acknowledge */
    (void)SyncState;

    EcatDcOffsetCacl();//ECAT 同步处理
    SmDcOffsettime = (uint16_t)ReadCount();

    Updata_EcatCmd();

    Updata_EcatFb();

    Sync0_Isr();
}
#endif

#if SERVOTYPE == SERVO_CAN
mcan_rx_message_t *FIFO_Rx_msg;
SDK_DECLARE_EXT_ISR_M(CAN_IRQn, Can_isr)
void Can_isr(void)
{
    //uint8_t Txflag = 0;

    CanSyncOffsetCacl();

    if(CANSelect->IR & MCAN_IE_HPME_MASK)
    {
        if(0 == RamPA->SyncMode)
        {
            SmDcOffsettime = (uint16_t)ReadCount();
            hpm_flag = true;
        }
        else if (1 == RamPA->SyncMode)
        {
            Radio_flag = true;;  
        }
        //CanSyncOffsetCacl();
    
        
        PosRef.PosNow = CM_BusCtrl.TargetSyncPos;
        CM_BusCtrl.TargetSyncSpd = CM_BusCtrl.TargetSpdVelA;
        CM_BusCtrl.TargetSyncTor = CM_BusCtrl.TargetTorqueA;
    
        SnycCmdFlag = 1;     
        CM_BusVar.SMCount++; 
        CM_BusVar.SyncCount = 0;  
    }
    CANSelect->IR = MCAN_IE_HPME_MASK;
}
#endif

extern int16_t  EctDcOffsetime;


#if SERVOTYPE == SERVO_CANOPEN
//IRQn_MCAN0
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, hpm_canopen_isr);
void hpm_canopen_isr(void)
{
    uint32_t flags = mcan_get_interrupt_flags(HPM_MCAN0);

    if ((flags & MCAN_INT_HIGH_PRIORITY_MSG) != 0U)
    {        
        CanSyncOffsetCacl();

        Updata_EcatCmd();

        Updata_EcatFb();
    }

    /* Error happened */
    // if ((flags & MCAN_EVENT_ERROR) != 0)
    // {

    // }

    mcan_clear_interrupt_flags(HPM_MCAN0, flags);
}
#endif

/***********************************************************************************************************************
* Function Name: GPTMER ISR
* Description  : This function is the interrupt service routine for Timer 3.
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
SDK_DECLARE_EXT_ISR_M(BOARD_CALLBACK_TIMER_IRQ, gptmer3_isr)
void gptmer3_isr(void)
{
    if (gptmr_check_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH)))
    {
        gptmr_clear_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH));

        AppTime1Ms();
    }
}




        //testPWMMMM3 = PWM->CMP_VAL_WORK[0] >> 8;
        //if(testPWMMMM3 == testPWMMMM1)
        //{
        //    testpwmcnt++;
        //}
        //else
        //{
        //    testpwmcnt2++;
        //}
        //testPWMMMM1++;

        //if(testPWMMMM1 > 6000) testPWMMMM1 = 0;
        //PwmDutyUpdata(testPWMMMM1,3125,3125,DrvCoeff.EpwmPrd);


                //testPWMMMM2 = PWM->CMP_VAL_WORK[0] >> 8;

        //testPWMMMM4 = testPWMMMM2 - testPWMMMM1;

        //if(testPWMMMM4 == 6000 || testPWMMMM4 == -1)
        //{
        //    testpwmcnt++;
        //}
        //else
        //{
        //    testpwmcnt1++;
        //}

