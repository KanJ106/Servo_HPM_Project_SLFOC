#include "Mit_CtrLoop.h"
#include "SV_FuncVar.h"
#include "SV_FuncCode.h"

Type_MIT_COM  MitCom = {0};
Type_MIT_CTRL MitCtrl= {0};

#if SERVOTYPE == SERVO_ETHERCAT
/** \brief Object 0x4000 (Mit TargetPos) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4000 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4000 (Mit TargetPos) object name*/
OBJCONST UCHAR OBJMEM aName0x4000[] = "Mit TargetPos";

/** \brief Object 0x4001 (Mit TargetVel) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4001 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4001 (Mit TargetVel) object name*/
OBJCONST UCHAR OBJMEM aName0x4001[] = "Mit TargetVel";

/** \brief Object 0x4002 (Mit TargeTor) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4002 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4002 (Mit TargeTor) object name*/
OBJCONST UCHAR OBJMEM aName0x4002[] = "Mit TargeTor";

/** \brief Object 0x4003 (Mit Kp) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4003 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4003 (Mit Kp) object name*/
OBJCONST UCHAR OBJMEM aName0x4003[] = "Mit Kp";

/** \brief Object 0x4004 (Mit Kd) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4004 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4004 (Mit Kd) object name*/
OBJCONST UCHAR OBJMEM aName0x4004[] = "Mit Kd";

/** \brief Object 0x4005 (Mit Kd) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4005 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4005 (Mit Kd) object name*/
OBJCONST UCHAR OBJMEM aName0x4005[] = "Mit Kt";

/** \brief Object 0x4006 (Mit Kd) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4006 = {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x4006 (Mit Kd) object name*/
OBJCONST UCHAR OBJMEM aName0x4006[] = "Mit Gear Ratio";

/** \brief Object 0x4007 (Mit PosFb) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4007 = {DEFTYPE_REAL32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x4007 (Mit PosFb) object name*/
OBJCONST UCHAR OBJMEM aName0x4007[] = "Mit ActualPos";

/** \brief Object 0x4008 (Mit VelFb) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4008 = {DEFTYPE_REAL32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x4008 (Mit VelFb) object name*/
OBJCONST UCHAR OBJMEM aName0x4008[] = "Mit ActualVel";

/** \brief Object 0x4009 (Mit TorFb) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4009 = {DEFTYPE_REAL32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x4009 (Mit TorFb) object name*/
OBJCONST UCHAR OBJMEM aName0x4009[] = "Mit ActualTor";

/** \brief Object 0x4020 (Mit TorFb) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4020 = {DEFTYPE_REAL32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x4020 (Mit TorFb) object name*/
OBJCONST UCHAR OBJMEM aName0x4020[] = "Torque Sensor Value";

/** \brief Object 0x4021 (Mit TorFb) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x4021 = {DEFTYPE_REAL32, 0x20, ACCESS_READWRITE|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x4021 (Mit TorFb) object name*/
OBJCONST UCHAR OBJMEM aName0x4021[] = "Target Torque closed Loop";


TOBJECT    OBJMEM DefMITObjDic[]
= {
   //last next 
   /* Object   0x4000 */
   {NULL,NULL, 0x4000, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4000, aName0x4000, &MitCom.PosTarget,    NULL, NULL, 0x0000 },
   /* Object   0x4001 */
   {NULL,NULL, 0x4001, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4001, aName0x4001, &MitCom.VelTarget,    NULL, NULL, 0x0000 },
   /* Object   0x4002 */
   {NULL,NULL, 0x4002, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4002, aName0x4002, &MitCom.TorTarget,    NULL, NULL, 0x0000 },
   /* Object   0x4003 */
   {NULL,NULL, 0x4003, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4003, aName0x4003, &MitCom.Kp,           NULL, NULL, 0x0000 },
   /* Object   0x4004 */
   {NULL,NULL, 0x4004, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4004, aName0x4004, &MitCom.Kd,           NULL, NULL, 0x0000 },
   /* Object   0x4005 */
   {NULL,NULL, 0x4005, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4005, aName0x4005, &MitCom.Kt,           NULL, NULL, 0x0000 },  
   /* Object   0x4006 */
   {NULL,NULL, 0x4006, {DEFTYPE_UNSIGNED16, 0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4006, aName0x4006, &MitCom.GearRatio,    NULL, NULL, 0x0000 },  
   /* Object   0x4007 */
   {NULL,NULL, 0x4007, {DEFTYPE_REAL32 ,    0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4007, aName0x4007, &MitCom.PosActual,    NULL, NULL, 0x0000 },
   /* Object   0x4008 */
   {NULL,NULL, 0x4008, {DEFTYPE_REAL32,     0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4008, aName0x4008, &MitCom.VelActual,    NULL, NULL, 0x0000 },
   /* Object   0x4009 */
   {NULL,NULL, 0x4009, {DEFTYPE_REAL32,     0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4009, aName0x4009, &MitCom.TorActual,    NULL, NULL, 0x0000 },
   
   {NULL,NULL, 0x4020, {DEFTYPE_REAL32,     0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4020, aName0x4020, &RamFuncCode.PC[26],  NULL, NULL, 0x0000 },
   {NULL,NULL, 0x4021, {DEFTYPE_REAL32,     0 | (OBJCODE_VAR << 8)}, &sEntryDesc0x4021, aName0x4021, &RamFuncCode.PC[24],  NULL, NULL, 0x0000 },
   {NULL,NULL, 0xFFFF, {0,0}, NULL, NULL, NULL, NULL, NULL, 0x0000}
};
#endif

void Mit_Init(void)
{
    MitCom.GearRatio     = RamPD->Reduction;
    MitCom.Kt = DrvCoeff.MotTorqe / 100.0f;

    MitCtrl.PosFbCoeff  = 2.0 * 3.14159265359 / (double)(DrvCoeff.MotEncSglRevLns * MitCom.GearRatio);
    MitCtrl.SpdFbCoeff  = 8192.0 / (16777216.0 * MitCom.GearRatio) / 60.0 * 2.0 * 3.14159265359;
    MitCtrl.TorFbCoeff  = (double)DrvCoeff.MotTorqe * MitCom.GearRatio / 100000.0;    
    MitCtrl.TorCmdCoeff = (double)DrvCoeff.MotIe * 100.0 / ((double)DrvCoeff.MotTorqe * MitCom.GearRatio); 
}

void Mit_updata(void)
{

}

void Mit_rst(void)
{
    static uint16_t GearRatio = 0;

    if(GearRatio != MitCom.GearRatio)
    {
        GearRatio = MitCom.GearRatio;

        MitCtrl.PosFbCoeff  = 2.0 * 3.14159265359 / (double)(DrvCoeff.MotEncSglRevLns * MitCom.GearRatio);
        MitCtrl.SpdFbCoeff  = 8192.0 / (16777216.0 * MitCom.GearRatio) / 60.0 * 2.0 * 3.14159265359;
        MitCtrl.TorFbCoeff  = (double)DrvCoeff.MotTorqe * MitCom.GearRatio / 100000.0;    
        MitCtrl.TorCmdCoeff = (double)DrvCoeff.MotIe * 100.0 / ((double)DrvCoeff.MotTorqe * MitCom.GearRatio); 
    }
}

void Mit_ReadEsc(void)
{
    MitCom.PosTargetBuff = MitCom.PosTarget;
    MitCom.VelTargetBuff = MitCom.VelTarget;
    MitCom.TorTargetBuff = MitCom.TorTarget;
    MitCom.KpBuff = MitCom.Kp;
    MitCom.KdBuff = MitCom.Kd;
}

void Mit_PdoReceive(void)
{
    MitCtrl.PosCmd = MitCom.PosTargetBuff;
    MitCtrl.VelCmd = MitCom.VelTargetBuff;
    MitCtrl.TorCmd = MitCom.TorTargetBuff;
    MitCtrl.Kp = MitCom.KpBuff;
    MitCtrl.Kd = MitCom.KdBuff;
}

void Mit_PdoSend(void)
{
    MitCom.PosActualBuff = MitCtrl.PosFb;
    MitCom.VelActualBuff = MitCtrl.VelFb;
    MitCom.TorActualBuff = MitCtrl.TorFb;
}

void Mit_WriteEsc(void)
{
    MitCom.PosActual = MitCom.PosActualBuff;
    MitCom.VelActual = MitCom.VelActualBuff;
    MitCom.TorActual = MitCom.TorActualBuff;
}

void Mit_FbCacl(int32 Pos,int32 Vel,int16 Tor,uint16_t flag)
{
    if(flag > 0)
    {
        MitCtrl.TorFb = (float)((double)Tor * MitCtrl.TorFbCoeff);
    }
    else
    {
        MitCtrl.PosFb = (float)((double)Pos * MitCtrl.PosFbCoeff);
        MitCtrl.VelFb = (float)((double)Vel * MitCtrl.SpdFbCoeff);
        MitCtrl.TorFb = (float)((double)Tor * MitCtrl.TorFbCoeff);
    }
}

int32 Mit_PdCacl(int32 Pos,int32 Vel)
{
    float output;
    int32 TorOut;

    MitCtrl.PosFb = (float)((double)Pos * MitCtrl.PosFbCoeff);
    MitCtrl.VelFb = (float)((double)Vel * MitCtrl.SpdFbCoeff);
    
    output = MitCtrl.Kp*(MitCtrl.PosCmd - MitCtrl.PosFb) + \
             MitCtrl.Kd*(MitCtrl.VelCmd - MitCtrl.VelFb) + \
             MitCtrl.TorCmd; 
    //output  Nm

    TorOut = (int32)(MitCtrl.TorCmdCoeff * (double)output);

    return TorOut;
}





