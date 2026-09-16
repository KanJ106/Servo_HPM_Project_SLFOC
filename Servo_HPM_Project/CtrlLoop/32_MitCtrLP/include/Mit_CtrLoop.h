#ifndef MIT_CTRLOOP_H_
#define MIT_CTRLOOP_H_
#include "objdef.h"
#include "ecat_def.h"//
#include "DrvCoeff.h"

typedef struct 	
{
    float PosTarget;
    float VelTarget;
    float TorTarget;
    float Kp;
    float Kd;
    float Kt;
    uint16_t  GearRatio;

    float PosActual;
    float VelActual;
    float TorActual;

    float PosTargetBuff;
    float VelTargetBuff;
    float TorTargetBuff;
    float KpBuff;
    float KdBuff;
    float KtBuff;
    uint16_t  GearRatioBuff;

    float PosActualBuff;
    float VelActualBuff;
    float TorActualBuff;
}Type_MIT_COM;

typedef struct 	
{
    float PosCmd;
    float VelCmd;
    float TorCmd;
    float Kp;
    float Kd;

    float PosFb;
    float VelFb;
    float TorFb;

    double  PosFbCoeff;
    double  SpdFbCoeff;
    double  TorFbCoeff;
    double  TorCmdCoeff;
}Type_MIT_CTRL;


extern Type_MIT_COM MitCom;
#if SERVOTYPE == SERVO_ETHERCAT
extern TOBJECT    OBJMEM DefMITObjDic[];
#endif
void Mit_Init(void);
void Mit_rst(void);
void Mit_ReadEsc(void);
void Mit_PdoReceive(void);
void Mit_PdoSend(void);
void Mit_WriteEsc(void);
void Mit_FbCacl(int32 Pos,int32 Vel,int16 Tor,uint16_t flag);
int32 Mit_PdCacl(int32 Pos,int32 Vel);
#endif














