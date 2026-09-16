#ifndef __SV_KINGKONGENC_H__
#define __SV_KINGKONGENC_H__
#include "userdefine.h"
#include "IQmathLib.h" 

//#define SECONDZEROFFFSET 8388608

typedef enum{ 
	CF_ResetOutEnc = 0x29,			//外圈编码器置零
	CF_ReadOutEnc = 0x32,			//
	CF_ReadAll = 0x43,		        //读取所有数据
}ENUM_DPT;//未注释部分单圈编码器无此ID,只能用有注释的

typedef struct{
	Uint8 Near_In:1;		//内转子过近
	Uint8 Far_In:1;			//内转子过远
	Uint8 Near_Out:1;		//外转子过近
	Uint8 Far_Out:1;		//外转子过远

	Uint8 bit4:1;			//
	Uint8 bit5:1;			//
	Uint8 Wrn:1;			//警告
	Uint8 Err:1;			//错误
}TYPE_S_DPT;

typedef struct
{
	union UNION_Status{
	Uint8 All;		//编码器故障信息
	TYPE_S_DPT Bit;
	}S;
    Uint8 EncRxNum;
    Uint8 EncTxcnt;
    
    Uint8 NoAckTime;			//无数据返回计时
	Uint8 ErrCRCcnt;			//CRC校验错误次数
    
    Uint8 CrcErrFlag;
    Uint8 NoAckFlag;
    
    Uint32 MotABS;
    Uint32 OutABS;
    Uint32 OutABSOrig;
    Uint32 OutPosOffset;
    uint8_t Dir;
}Encode_DPT;

void DPT_Process(void);
void DPT_FaultHandle(void);
void DPTPosZero(void);
void DPTPos_Alignment(void);
void DPT_ParaInit(void);
extern Encode_DPT DPT;

#endif


