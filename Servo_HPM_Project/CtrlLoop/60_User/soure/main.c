/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "board.h"
#include "StartupTiming.h"
#include "hpm_debug_console.h"
#include "Hpm_ISR.h"

#include "Hpm_ISR.h"
#include "MCU_Fw.h"
#include "s_analog_init.h"
#include "s_time_init.h"
#include "s_gpio_init.h"
#include "s_pwm_init.h"
#include "s_comm_init.h"
#include "s_encode_init.h"
#include "s_sys_init.h"
#include "s_iic_init.h"
#include "r_can_api.h"
#include "ota.h"

#include "SV_FuncVar.h"
#include "SV_FuncCode.h"
#include "SV_AppDriveInterface.h"
#include "SV_PanelCtl.h"
#include "SV_I2c.h"
#include "SV_Sci.h"
#include "SV_FunCodeAttrLit.h"
#include "SV_UVWShortProtect.h"
#include "SV_IncEncode.h"
#include "Drive.h"
#include "SV_MagEncode.h"
#include "coeappl.h"
#include "cia402appl.h"
#include "SV_FuncCodeObj.h"
#include "Pos_FullClosedLoop.h"
#include "SV_ModbusInit.h"
#include "SV_ModbusConTrolVar.h"
#include "SV_CanFirm.h"
#include "SV_CanbusCtrl.h"
#include "SV_KingKongEnc.h"
#include "s_tsensor_init.h"
#include "SV_RtT036.h"
#include "SV_ModbusInit.h"
#include "Flash_Eeprom.h"
#include "Version.h"
#include "SV_Sweep.h"
#include "Mit_CtrLoop.h"

/*
2   ------>  CMT0(1ms)   
3   ------>  sync0_isr  PDI_ISR
5   ------>  PWM——ISR
*/

/*
1ms   GPTMR3
ECAT  GPTMR0
SDFM  GPTMR4

CAN ID 
0x480
0x500
0x580
0x200 指令1
0x280 指令2
0x300 指令3
0x380 指令4
0x400 指令5

0x100
0x180
0x080

0x600普通CAN MIT指令
0x680普通CAN MIT反馈
0x700
0x780
*/
uint16_t Glo_IIcReadAll;

#if SERVOTYPE == SERVO_ETHERCAT
uint16_t EctDcRefer = 0;
int16_t  EctDcAdjustPmax = 0;
int16_t  EctDcAdjustNmax = 0;
int16_t  EctDcOffset = 0;
int16_t  EctDcOffsetRemder = 0;
int16_t  EctDcOffsetime =0;
uint16_t  EcatPwmTime = 0;
uint16_t  EcatCnt = 0;
volatile uint8_t EcatSnycCmdFlag = 0;
volatile uint8_t EcatSnycFlag = 0;
volatile uint8_t EcatSnycCmd = 1;
uint16_t SmPeriodtime = 0;
uint16_t SmDcOffsettime = 0;
void EctDcCyc_Cacl(void);
extern void BL_Foe_Init(void);
#endif

#if SERVOTYPE == SERVO_CAN
uint16_t SyncRefer = 0;
int16_t  SyncAdjustPmax = 0;
int16_t  SyncAdjustNmax = 0;
int16_t  SyncOffset = 0;
int16_t  SyncOffsetRemder = 0;
int16_t  SyncOffsetime =0;
volatile uint8_t SnycCmdFlag = 0;
volatile uint8_t SnycFlag = 0;
uint16_t SmPeriodtime = 0;
uint16_t SmDcOffsettime = 0;
void CanSync_init(void);
#endif

#if SERVOTYPE == SERVO_MODBUS
uint16_t SmPeriodtime = 0;
uint16_t SmDcOffsettime = 0;
#endif

#if SERVOTYPE == SERVO_CANOPEN
uint32_t SyncRefer = 0;
int16_t  SyncAdjustPmax = 0;
int16_t  SyncAdjustNmax = 0;
int16_t  SyncOffset = 0;
int16_t  SyncOffsetRemder = 0;
int16_t  SyncOffsetime =0;
volatile uint8_t SnycCmdFlag = 0;
volatile uint8_t SnycFlag = 0;
uint16_t SmPeriodtime = 0;
uint16_t SmDcOffsettime = 0;
int16_t  EctDcOffsetime = 0;
void CanSync_init(void);
extern uint8_t NMT_State;
#endif

extern void AppTime1msMainLoop(void);
extern void SN_init(void);
extern uint8_t canopen_init(void);
extern uint8_t canopen_task(void);
extern void CiA402_Application(TCiA402Axis *pCiA402Axis);
extern void CiA402_StateMachine(void);//状态转换函数
extern void CiA402_Init(void);
extern void CiA402_rst(void);
int main(void) 
{
    uint32_t LedCount = 0;

    #if SERVOTYPE == SERVO_ETHERCAT
    uint16_t Esc_Status = 0;
    #endif

    #if defined(_FOE_LDRPRM_BANK0_)
    Bank_Sel_Cacl();
    #endif  

    board_init();
    g_startup_timing.cpu_hz = clock_get_frequency(clock_cpu0);

    #if HARDWARE_VER_SEL == HARDWARE_VER_0
    board_init_led_pins();
    #else 
    port_init();
    #endif

    #if SERVOTYPE == SERVO_ETHERCAT
    Esc_Status = HW_Init();  //需优先初始化
    #endif

    Eeprom_flash_eeprom_init();

    InitFuncVar();
    IIcVarInit();

    I2CA_Init();
    R_CMT0_Create();
    R_CMT0_Start();

    PowerOnReadFunc();
    Glo_IIcReadAll = 1;
    R_CMT0_Stop(); 

    RamAid->SoftReset = 0;    //P8-00上电为0
    RamAid->FaultReset = 0;
    RamAid->JodRun = 0;
    RamAid->SpdJog = 100;     //
    RamAid->SON = 0;          //P8-05上电为0
    RamAid->BrakeLogic = 0;   
    RamAid->Set_JodRun = 0;
    RamBasePara->PosInternalMode = 0;
    RamMotor->TuningType = 0;

    #if SERVOTYPE == SERVO_ETHERCAT
    MainInit();
    CiA402_Init();                                     /*Initialize Axes structures*/
    APPL_GenerateMapping(&nPdInputSize,&nPdOutputSize);/*Create basic mapping*/
    APPL_FanCodeObjInit();
    AddObjectsToObjDictionary((TOBJECT OBJMEM *) DefMITObjDic);//0x4000
    OBJ_GetNoOfObjects(0);
    BL_Foe_Init();
    if(Esc_Status == 0)    bRunApplication = TRUE;
    #endif

    CheckTimeInit();
    #if SERVOTYPE == SERVO_CAN
    CanInit(RamCommu->CanBaud, RamCommu->CanFdBaud, RamCommu->CanID,RamCommu->CanMode);
    #endif

    #if SERVOTYPE == SERVO_MODBUS
    Modbus_Init(RamCommu->Baud1,RamCommu->Format1);
    #endif


    PWMV2_Init(RamServo->Carrier,RamServo->DbTime,RamServo->Boot_Power); 

    VarInit();

    SN_init();

    #if SERVOTYPE == SERVO_CANOPEN
    canopen_init();
    CiA402_Init();
    LocalAxes.bAxisIsActive = 1;
    #endif

    Panel_KeyInit();
    Sci_TorSensorInit();

    /* Sensorless build uses isolated, read-only RT position diagnostics. */
#if !defined(SENSORLESS_CANOPEN_BUILD)
    if(DPI_EncType == 2 || DPI_EncType == 3)
    {
        DPT_ParaInit();
        EncodeSci_Main();
    }
    else if(DPI_EncType == 4 || DPI_EncType == 5)
    {
        EncodeSci_Main();
    }
    else if(DPI_EncType == 12 || DPI_EncType == 13)
    {
        RT_ParaInit();
        EncodeSci_Main();
    }
    else
    {
    }

#else
    SensorlessEncoder_Init(DPI_EncType);
#endif
    Adc_Init();
    #if CURRENT_MODE == CURRENT_SDFM
    Sigma_Delta_Init();
    #endif

    board_delay_ms(1000);
    Scope_Init(0, 0, 0);
    BL_Ota_Init(PRODUCTCODE);
    PWMCounterStart();

    PWM_ON();

    Ewdg_Init();
    EnableDog();

    R_CMT0_Start();

    UVWShortProtect.ShortCheck = 1; 
    while (1) 
    {
        ServiceDog();

        LedCount++;
        if(LedCount == 40000)
        {
            #if defined(_FOE_LDRPRM_BANK0_)
            LedCount = 0;
            #else
            LedCount = 35000;
            #endif

            #if   HARDWARE_VER_SEL == HARDWARE_VER_0
            gpio_toggle_pin(BOARD_R_GPIO_CTRL, BOARD_R_GPIO_INDEX, BOARD_R_GPIO_PIN);
            #endif

            #if   HARDWARE_VER_SEL == HARDWARE_VER_1
            gpio_toggle_pin(BOARD_R_GPIO_CTRL, CRENLEDPORT, CRENLEDINDEX);
            #endif
        }

        #if SERVOTYPE == SERVO_ETHERCAT
        if((bRunApplication == TRUE) && (UartBootFlag == 0))
        {
            MainLoop();
            CiA402_rst();
            EctDcCyc_Cacl();
        }
        #endif

        #if SERVOTYPE == SERVO_CAN
        CanSync_init();
        Can_Boot_Process();
        #endif 

        #if SERVOTYPE == SERVO_CANOPEN
        CanSync_init();
        canopen_task();
        CiA402_rst();
        CiA402_StateMachine();//状态转换函数
        CiA402_Application(&LocalAxes);
        #endif

        AppTime1msMainLoop();
        SM_ParaRefresh();

        SWEEP_BackgroundTask(&sweep_controller); // 扫频状态管理        
        process_communication_command();         // 处理通信命令

        StateMachine.RegulFlg = StateMachine.RegilFlgISR;
    }
}

#if SERVOTYPE == SERVO_ETHERCAT
void EctDcCyc_Cacl(void)
{
    uint32_t temp = 0;
    int16_t  temp1;
    
    if((nAlStatus & 0x0F) <= STATE_SAFEOP)
    {
        temp1 = CumObj.sSyncManOutPar.u32CycleTime / 1000000 - 2;  //
        
        if(temp1 < 0) temp1 = 0;
        
        EctDcAdjustPmax = DCMAXTIME + 100 * temp1;         //3us
        EctDcAdjustNmax = -EctDcAdjustPmax;
        
        if((DPI_Carrier & 0x00f0) == 0)//单刷
        {
            temp = DrvCoeff.Tpwm * 10;                   
        }
        else
        {
            temp = DrvCoeff.Tpwm * 20;                     
        }
        DrvCoeff.pwmAdjustCnt = CumObj.sSyncManOutPar.u32CycleTime/temp;
        
        PosRef.Interpdata = 0;
        PosRef.InterpdataRem = 0;
        PosRef.EcatRem = 0;
    }
    
    if((nAlStatus & 0x0F) < STATE_SAFEOP)
    {
        DrvCoeff.EpwmPrd = EPWM_PRD_DEFAULT;
        EctDcRefer = EPWM_PRD_DEFAULT >> 2;        //位置环前16.125us

        if(FullCloseLoop.FullFlag == 1)
        {
            Cia402_PP.GetPos6063 = ExtPosFb.PosFbTotle;
            Cia402_PP.GetVel606C = SpdFb.Fb; 
        }
        else
        {
            Cia402_PP.GetPos6063 = PosFb.PosFbTotle;
            Cia402_PP.GetVel606C = SpdFb.Fb; 
        }
              
        Cia402_PP.GetTor6077 = CurMonitor.TorqRatsDispS;
        
        Cia402_PP.PosDemVal60FC = PosRef.PosNow;         
        Cia402_PP.VelDemVal606B = Spd_Ref.Ref;           
        Cia402_PP.TorDemVal6074 = CurMonitor.TorqRatsRef;       
      
        cia402_FeedbackHanle(&Cia402_PP);
    }
}

//uint8_t  PwmDir = 0;
void EcatDcOffsetCacl(void)
{
    int16 temp;
    temp = (PWM->CNT_VAL[0] >> 8)  - EctDcRefer; //SYNC控制在上升计数
    //temp = EcatPwmTime - EctDcRefer;
    
    EctDcOffsetime = temp;
    
    if(temp <= EctDcAdjustPmax && temp >= EctDcAdjustNmax)
    {
        EctDcOffset = temp / DrvCoeff.pwmAdjustCnt;
        EctDcOffsetRemder = temp % DrvCoeff.pwmAdjustCnt;
        EcatSnycFlag = (EcatSnycCmd == 1) ? 1 : 0;
    } 
    else
    {
        if(temp > EctDcAdjustPmax)
        {
            temp = EctDcAdjustPmax;
            EctDcOffset = temp / DrvCoeff.pwmAdjustCnt;
            EctDcOffsetRemder = temp % DrvCoeff.pwmAdjustCnt;
        }
        else if(temp < EctDcAdjustNmax)
        {
            temp = EctDcAdjustNmax;
            EctDcOffset = temp / DrvCoeff.pwmAdjustCnt;
            EctDcOffsetRemder = temp % DrvCoeff.pwmAdjustCnt;
        }
        else
        {
            
        }
        
        EcatCnt++;
    }
}
  
void EcatDcCalibCacl(void)
{
    uint16_t temp;
    
    if(EctDcOffsetRemder > 0)
    {
        temp = EPWM_PRD_DEFAULT + EctDcOffset + 1;
        DrvCoeff.EpwmPrd = temp;
        EctDcOffsetRemder -= 1;
    }
    else if(EctDcOffsetRemder < 0)
    { 
        temp = EPWM_PRD_DEFAULT + EctDcOffset - 1;
        DrvCoeff.EpwmPrd = temp;
        EctDcOffsetRemder += 1;
    }
    else
    {
        temp = EPWM_PRD_DEFAULT + EctDcOffset;
        DrvCoeff.EpwmPrd = temp;
    }

    PWM->SHADOW_VAL[0] = temp << 8;  
}
#endif

#if SERVOTYPE == SERVO_CAN
void CanSync_init(void)
{
    uint32_t temp = 0;
    int16_t  temp1;
    
    if(CM_BusVar.CanState < 2)
    {
        DrvCoeff.EpwmPrd = EPWM_PRD_DEFAULT;
        
        temp1 = CM_BusCtrl.SyncTime / 1000 - 2;  //
        
        if(temp1 < 0) temp1 = 0;
        
        SyncAdjustPmax = DCMAXTIME + DCMTIME1US * temp1;         //3us
        SyncAdjustNmax = -SyncAdjustPmax;
        
        if((DPI_Carrier & 0x00f0) == 0)//单刷
        {
            temp = DrvCoeff.Tpwm * 10;                   
        }
        else
        {
            temp = DrvCoeff.Tpwm * 20;                     
        }
        DrvCoeff.pwmAdjustCnt = (int64)CM_BusCtrl.SyncTime * 1000L/temp;
        
        SyncRefer = EPWM_PRD_DEFAULT >> 2;
        
        PosRef.Interpdata = 0;
        PosRef.InterpdataRem = 0;
        PosRef.EcatRem = 0;   
    }
}

void CanSyncOffsetCacl(void)
{
    int16 temp;
    
    temp = (PWM->CNT_VAL[0] >> 8)  - SyncRefer; //SYNC控制在上升计数
    
    if(temp <= SyncAdjustPmax && temp >= SyncAdjustNmax)
    {
        SyncOffset = temp / DrvCoeff.pwmAdjustCnt;
        SyncOffsetRemder = temp % DrvCoeff.pwmAdjustCnt;
        SnycFlag = 1;
    } 
    else
    {
        if(temp > SyncAdjustPmax)
        {
            temp = SyncAdjustPmax;
            SyncOffset = temp / DrvCoeff.pwmAdjustCnt;
            SyncOffsetRemder = 0;
        }
        else if(temp < SyncAdjustNmax)
        {
            temp = SyncAdjustNmax;
            SyncOffset = temp / DrvCoeff.pwmAdjustCnt;
            SyncOffsetRemder = 0;
        }
    }

    SyncOffsetime = temp;
}

void CanSyncCalibCacl(void)
{
    uint16_t temp;
    
    if(SyncOffsetRemder > 0)
    {
        temp = EPWM_PRD_DEFAULT + SyncOffset + 1;
        DrvCoeff.EpwmPrd = temp;
        SyncOffsetRemder -= 1;
    }
    else if(SyncOffsetRemder < 0)
    { 
        temp = EPWM_PRD_DEFAULT + SyncOffset - 1;
        DrvCoeff.EpwmPrd = temp;
        SyncOffsetRemder += 1;
    }
    else
    {
        temp = EPWM_PRD_DEFAULT + SyncOffset;
        DrvCoeff.EpwmPrd = temp;
    }

    PWM->SHADOW_VAL[0] = temp << 8;
}
#endif

#if SERVOTYPE == SERVO_CANOPEN
void CanSync_init(void)
{
    int32_t  temp1;
    
    if(NMT_State == 127)//处于预操作状态下
    {
        DrvCoeff.EpwmPrd = EPWM_PRD_DEFAULT;
        
        temp1 = CumObj.x1006_communicationCyclePeriod / 1000 - 2;  //
        
        if(temp1 <= 0)
        {
            SyncAdjustPmax = 600; //3us
        }
        else
        {
            SyncAdjustPmax = 200 * 8 * temp1;        
            SyncAdjustNmax = -SyncAdjustPmax;
        }
        
        DrvCoeff.pwmAdjustCnt = (int64)CumObj.x1006_communicationCyclePeriod * 100L/DrvCoeff.SpdTsamp;
        SyncRefer   = EPOS_PRD_DEFAULT >> 1;    //位置环周期一半

        PosRef.Interpdata = 0;
        PosRef.InterpdataRem = 0;
        PosRef.EcatRem = 0;   
    }
}


void CanSyncOffsetCacl(void)
{
    int32 temp;

    SyncRefer = EPOS_PRD_DEFAULT >> 1;
    temp = (PWM1->CNT_VAL[3] >> 8)  - SyncRefer; //SYNC控制在上升计数
    temp = (int32_t)((float)temp * 0.5f + (float)SyncOffsetime * 0.5f);
    SyncOffsetime = temp;
    EctDcOffsetime = SyncOffsetime;
        
    if(temp <= SyncAdjustPmax && temp >= SyncAdjustNmax)
    {
        SyncOffset = SyncOffsetime / DrvCoeff.pwmAdjustCnt;
        SnycFlag = 1;
    } 
    else
    {
        if(temp > SyncAdjustPmax)
        {
            SyncOffset = SyncAdjustPmax/DrvCoeff.pwmAdjustCnt;
        }
        else if(temp < SyncAdjustNmax)
        {
            SyncOffset = SyncAdjustNmax/DrvCoeff.pwmAdjustCnt;
        }
    }
}

void CanSyncCalibCacl(void)
{
    uint32_t temp;
    uint32_t TorTime;

    temp = EPWM_PRD_DEFAULT + SyncOffset;
    DrvCoeff.EpwmPrd = temp;
    PWM->SHADOW_VAL[0] = temp << 8;
    TorTime = temp;

    temp = ((temp + 1) << 4) - 1;
    //EPOS_PRD_DEFAULT = temp;

    PWM1->SHADOW_VAL[0] = temp << 8;
    PWM1->SHADOW_VAL[3] = (temp - TorTime) << 8;
}
#endif

