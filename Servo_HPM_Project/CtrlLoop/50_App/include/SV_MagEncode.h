#ifndef SV_MAGENCODE_H
#define SV_MAGENCODE_H
#include "userdefine.h"
#include "IQmathLib.h" 

#define FIRSTREVLNS         (131072)
//
struct TYPE_STRUCT_32bit
{
    Uint32 Crc:6;
    Uint32 Err:2;
    Uint32 Pos:19;
    Uint32 ID:5;
};
typedef union
{
    Uint32 All;
    struct TYPE_STRUCT_32bit Word;
}TYPE_UNION_32bit;

typedef struct
{
    int32  Pos;
    int32  PosLast;
    int32  PosOffset;
    int32  PosComPensation;
    int32  PosOrig;
    int16  MuiltPos;
    Uint16  MuiltPosOrig;
    Uint16  MuiltPosOffset;
    
    Uint16 RxLostCount;
    Uint16 RxErrCount;
    Uint16 CeErrCount;
    
    Uint8 RxLostcnt;
    Uint8 RxCrccnt;
    Uint8 CeErrcnt;
    
    Uint8 MagComPenflag;
    
    Uint8 BissTxCnt;
    Uint8 BissTxNum;
    Uint8 EncTxflg;
    
}MAG_DataTYPE;

extern MAG_DataTYPE MagReg;
extern MAG_DataTYPE MagFirst;

extern int32_t SECONDREVLNS;
extern int32_t SECONDZEROFFFSET;

extern void  EccodeBissPosCacl_First(void);

extern void  EccodeBissPosZero_Second(void);
extern void  EccodeBissPosCacl_Second(void);
extern int32 EncodePosInit_Second(void);
extern void EccodeBissPos_Alignment(void);
extern void  EncodeBiss_Second(void);
extern void EccodeBissMuliZero_Second(void);

uint16_t IcmuRead_Register(uint8_t Addr);
void delay_us(uint8_t us);
void delay_ms(uint8_t ms);

void MagEncodePosRead_Second(void);
void MagEncodePosReq_Second(void);
void BissCrcTableGen(void);
void Biss_IO_Second(void);

#endif
