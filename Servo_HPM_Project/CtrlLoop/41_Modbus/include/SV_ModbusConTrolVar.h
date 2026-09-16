#ifndef SV_MODBUSCONTROLVAR_H_
#define SV_MODBUSCONTROLVAR_H_

#include "IQmathLib.h" 
#include "Pos_Ref.h"
#include "SV_StateMachine.h" 
#include "SV_FaultProtect.h"
#include "PosGenerator.h"
#include "PosFb.h"
#include "SV_ModbusFirm.h"//

#if SERVOTYPE == SERVO_MODBUS

#define MODBUSCONTROLATARTADDRES (ModBusConTrolAddres[0])

#define CMBUSMODE_POS   1
#define CMBUSMODE_SPD   2
#define CMBUSMODE_TOR   3
#define CMBUSMODE_HOME  4
#define CMBUSMODE_MIT   5  

#define CMBUSMODE_SYNCPOS   8
#define CMBUSMODE_SYNCSPD   9
#define CMBUSMODE_SYNCTOR   10

#define POSABSOLUTE      0
#define POSRELATIVE      1
#define POSCYCLE         3
#define POSCONTINUOUS    2

struct ModbusStatus_BITS {
    Uint16 RunFlag:1;
	Uint16 PosReachFlag:1;
    Uint16 ErrStatus:2;
    Uint16 HaltFlag:1;
    Uint16 EstopFlag:1;
    Uint16 HomeOkFlag:1;
    
	Uint16 Rsvd1:9;
};

union STUTASFLAG {
   Uint16                    all;
   struct ModbusStatus_BITS  bit;
};

typedef struct
{
    uint16_t Run;
    uint16_t ExecuteFlag;
    uint16_t Halt;
    uint16_t Restart;
    
    int32_t  TargetPos1;  
    int32_t  TargetPos2;
    int32_t  TargetSpd_p;
    uint32_t delayTime;
    
    int32_t  TargetSpd_S;
    int16_t  TargetTor_M;
    
    uint16_t ErrCode;
    uint16_t Status;
    uint16_t rsd1;
    
    int32_t  PosFbAct;
    int32_t  VelFbAct;
    int16_t  TorFbAct;
    uint16_t rsd2;
    int32_t  FollowingError;
    int32_t  Udc_mv;
    uint16_t MosTemper;
    uint16_t rsd3;
    int32_t  SinglePos_Mortor;
    int32_t  SinglePos_Reduce;
    
    int32_t  PosDemAct;
    int32_t  VelDemAct;
    int16_t  TorDemAct; 
    
    uint16_t EStop;

}TYPE_CMbusComm;

typedef struct
{
    uint16_t ModbusSel;
    uint16_t Mode;    
    uint16_t PosTurn;
    uint16_t ZeroFlag;
    uint16_t PosLimFlag;
    
    uint16_t MuliSeg;
    uint16_t MuliStopFlag;
    uint16_t PosTurnFlag;
    uint16_t PosCoeff_1ms;
    
    uint32_t DelayCnt;
    uint32_t DelayCntAll;
    
    uint32_t CntMax_1S;
    double   Pos_SpdCmdCoeff;
    double   Spd_CmdCoeff;
    double   Spd_FbCoeff;
    double   AccDec_Coeff_P;
    double   AccDec_Coeff_S;
    
    int32_t  MaxSpd_internal;
    int32_t  MaxSpd;
    int16_t  MaxTorque_internal;
    int16_t  MaxTorque;
    
    int32_t TargetPosA;
    int32_t TargetPosBuff[2];
    int32_t TargetPosVelA;
    int32_t TargetPosAccA;
    int32_t TargetPosDecA;
    
    int32_t TargetSpdVelA;
    int32_t TargetSpdAccA;
    int32_t TargetSpdDecA;
    
    int16_t TargetTorqueA;
    int16_t rsd1;
    int32_t TargetSlopA;
    int32_t TorqueSpdLim;
    
    int32_t TargetSyncPos;
    int32_t TargetSyncSpd;
    int16_t TargetSyncTor;
    
    int32_t PosFbActualValue;
    
    union STUTASFLAG Status;
    uint16_t EstopFlag;
    uint16_t Run;
    
    uint16_t HomeStep;
    uint16_t HomeCnt;
}TYPE_CMbusConTrol;

extern uint16_t* const    ModBusConTrolAddres[MODBUSMAXADDRES];
extern TYPE_CMbusComm     CM_BusVar;
extern TYPE_CMbusConTrol  CM_BusCtrl;

void CM_busCtrl_init(TYPE_CMbusConTrol *v);
void CM_busCtrl_rst(TYPE_CMbusConTrol *v);
void CM_busCtrl_updata(TYPE_CMbusConTrol *v);

uint8_t CMbusControlMode(void);
void Modbus_GetCmd(void);
void CMbusPosRefcalc(POS_REF *v);
void CMbusHomecalc(POS_REF *v);
uint16_t CMbus_Stop(void);
void Modbus_GetFbDataOff(void);
void ModBus_Process(void);
int32 Canbus_GetMITOutPut(void);
#endif

#endif








