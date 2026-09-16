#ifndef SV_CANBUSCTRL_H_
#define SV_CANBUSCTRL_H_
#include "r_cg_Project.h"
#include "Drive.h"
#include "SV_CanFirm.h"//
#include "IQmathLib.h" 

#if SERVOTYPE == SERVO_CAN

#define CMBUSMODE_POS   1
#define CMBUSMODE_SPD   2
#define CMBUSMODE_TOR   3
#define CMBUSMODE_HOME  4
#define CMBUSMODE_MIT   5  

#define CMBUSMODE_SYNCPOS   8
#define CMBUSMODE_SYNCSPD   9
#define CMBUSMODE_SYNCTOR   10

#define PI              3.1415926f    


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
    uint16_t ModbusSel;        
    uint16_t Mode;             
    uint16_t PosTurn;          
    uint16_t ZeroFlag;        
    
    uint16_t MuliSeg;          
    uint16_t MuliStopFlag;     
    uint16_t PosTurnFlag;      
    uint16_t PosCoeff_1ms;
    
    uint32_t DelayCnt;
    uint32_t DelayCntAll;
    
    uint32_t CntMax_1S;
    double   Pos_CmdCoeff;       
    double   Spd_CmdCoeff;       
    double   Spd_FbCoeff;        
    double   AccDec_Coeff_P;     
    double   AccDec_Coeff_S;     
    
    int16_t  MaxSpd_internal;
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
    
    uint32_t SyncTime;       
    
    union STUTASFLAG Status;
    uint16_t EstopFlag;
    uint16_t Run;
    
    uint16_t HomeStep;
    uint16_t HomeCnt;

    //MIT
    float   TargetMITPos;
    float   TargetMITVel;
    float   TargetMITTor;
    float   TargetMITKp;
    float   TargetMITKd;

    double   ActualMITPos;
    double   ActualMITVel;
    double   ActualMITTor;

    double  PosFbMitCoeff;
    double  SpdFbMitCoeff;
    double  TorFbMitCoeff;
    double  TorCmdMitCoeff;
    //mqbmit
    float   MITPos_CmdCoeff;
    float   MITSpd_CmdCoeff;
    float   MITKp_CmdCoeff ;
    float   MITKd_CmdCoeff ;
    float   MITTor_CmdCoeff;

    float   MITFbPos_CmdCoeff;
    float   MITFbSpd_CmdCoeff;
    float   MITFbTor_CmdCoeff;

    int32   ScopePos;
    int32   ScopeVel;
    int16   ScopeTor;
}TYPE_CMbusConTrol;

extern TYPE_CMbusConTrol CM_BusCtrl;

extern void CM_busCtrl_init(TYPE_CMbusConTrol *v);
extern void CM_busCtrl_updata(TYPE_CMbusConTrol *v);
extern void CM_busCtrl_rst(TYPE_CMbusConTrol *v);

extern uint16_t CMbusControlMode(void);
extern void CanBusCtrl_Process(void);
extern void CMbusPosRefcalc(POS_REF *v);
extern void CMbusHomecalc(POS_REF *v);
extern uint16_t CMbus_Stop(void);
extern void CanbusMITFbData(void);
int32 Canbus_GetMITOutPut(void);

#elif SERVOTYPE == SERVO_MODBUS

#else
#define CMBUSMODE_POS   1
#define CMBUSMODE_SPD   2
#define CMBUSMODE_TOR   3
#define CMBUSMODE_HOME  4
#define CMBUSMODE_MIT   5  

#define CMBUSMODE_SYNCPOS   8
#define CMBUSMODE_SYNCSPD   9
#define CMBUSMODE_SYNCTOR   10


struct ModbusStatus_BITS {
    Uint16 RunFlag:1;
	Uint16 PosReachFlag:1;
    Uint16 ErrStatus:2;
    
	Uint16 Rsvd1:12;
};

union STUTASFLAG {
   Uint16                    all;
   struct ModbusStatus_BITS  bit;
};

typedef struct
{
    uint16_t Run;
    uint16_t ModbusSel;
    uint16_t Mode;    
    uint16_t PosTurn;
    uint16_t ZeroFlag;
    
    uint16_t MuliSeg;
    uint16_t MuliStopFlag;
    uint16_t PosTurnFlag;
    uint16_t PosCoeff_1ms;
    
    uint32_t DelayCnt;
    uint32_t DelayCntAll;
    
    uint32_t CntMax_1S;
    double   Spd_CmdCoeff;
    double   Spd_FbCoeff;
    double   AccDec_Coeff_P;
    double   AccDec_Coeff_S;
    
    int16_t  MaxSpd_internal;
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
    
    uint16_t SyncTime;
    
    union STUTASFLAG Status;
    uint16_t EstopFlag;

    uint16_t HomeStep;
    uint16_t HomeCnt;

    //MIT
    float   TargetMITPos;
    float   TargetMITVel;
    float   TargetMITTor;
    float   TargetMITKp;
    float   TargetMITKd;

    float   ActualMITPos;
    float   ActualMITVel;
    float   ActualMITTor;

    double  PosFbMitCoeff;
    double  SpdFbMitCoeff;
    double  TorFbMitCoeff;
    double  TorCmdMitCoeff;

}TYPE_CMbusConTrol;

extern TYPE_CMbusConTrol CM_BusCtrl;

extern uint8_t CMbusControlMode(void);
extern void CMbusPosRefcalc(POS_REF *v);
extern void CMbusHomecalc(POS_REF *v);
extern uint16_t CMbus_Stop(void);
extern void CanbusMITFbData(void);
extern int32 Canbus_GetMITOutPut(void);
#endif

#endif

