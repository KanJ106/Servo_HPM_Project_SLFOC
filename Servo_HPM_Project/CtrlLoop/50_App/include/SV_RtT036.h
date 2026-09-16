#ifndef __SV_RTT036_H__
#define __SV_RTT036_H__
#include "userdefine.h"
#include "IQmathLib.h" 

#define MotorAngErrMax  588//0.355731
#define ReduceAngErrMax 989//0.588235

typedef struct{
	Uint8 OS:1;			//OverSpeed
	Uint8 FS:1;			//Full absolute status
	Uint8 CE:1;			//Countering Error
	Uint8 OF:1;			//Counter Overflow

	Uint8 OH:1;			//OverHeat
	Uint8 ME:1;			//Multi_turn Error
	Uint8 BE:1;			//Battery Error
	Uint8 BA:1;			//Battery Alarm
}TYPE_S_RT;

typedef struct
{
	union UNION_Status1{
	Uint8 All;		//编码器故障信息
	TYPE_S_RT Bit;
	}S;
    Uint8 EncRxNum;
    Uint8 EncTxcnt;
    
    Uint16 NoAckTime;			//无数据返回计时
	Uint16 ErrCRCcnt;			//CRC校验错误次数
    Uint8 Cecnt;			    //CE校验错误次数
    
    Uint8 CrcErrFlag;
    Uint8 NoAckFlag;
    
    Uint32 MotABS;
    Uint32 OutABS;
    Uint32 OutABSOrig;
    int32  OutPosOffset;
    uint8_t Dir;
    uint8_t OutDir;
    uint8_t CalStep;//编码器校准步骤
    uint8_t CalStat;//编码器校准状态
    Uint32 CalCnt1;//编码器校准时间计数1
    Uint32 CalCnt2;//编码器校准时间计数2
    Uint32 CalCnt3;
    uint8_t Calflag;//编码器校准标志
    uint8_t CalNote1;//编码器校准信息1

    int CalNote2;//编码器校准信息1
    uint8_t Calflag2;//编码器校准标志2
    uint8_t SetZeroStep;//编码器置零步骤
    uint8_t SetZeroStat;//编码器置零状态
    uint8_t SetZeroflag;//编码器置零标志
//    uint8_t SetZeroState1;//编码器置零状态1
 //   uint8_t SetZeroState2;//编码器置零状态2
    Uint32 SetZeroCnt1;//编码器置零时间计数1
    Uint32 SetZeroCnt2;//编码器置零时间计数2

    int16_t NsinVpp;       //主码道正弦峰值
    int16_t NsinOffset;    //主码道正弦中心点电压
    int16_t NcosVpp;       //主码道余弦峰值
    int16_t NcosOffset;    //主码道余弦中心点电压
    int16_t MsinVpp;       //辅码道正弦峰值
    int16_t MsinOffset;    //辅码道正弦中心点电压
    int16_t McosVpp;       //辅码道余弦峰值
    int16_t McosOffset;    //辅码道余弦中心点电压

    int16_t ONsinVpp;       //主码道正弦峰值
    int16_t ONsinOffset;    //主码道正弦中心点电压
    int16_t ONcosVpp;       //主码道余弦峰值
    int16_t ONcosOffset;    //主码道余弦中心点电压
    int16_t OMsinVpp;       //辅码道正弦峰值
    int16_t OMsinOffset;    //辅码道正弦中心点电压
    int16_t OMcosVpp;       //辅码道余弦峰值
    int16_t OMcosOffset;    //辅码道余弦中心点电压

    int16_t CalMotorAglErr2;//编码器校准的电机端主辅码道角度差2
    int16_t CalReducerAglErr2;//编码器校准的减速机端主辅码道角度差2

    int16_t CalMotorAglErr3;//编码器校准的电机端主辅码道角度差3
    int16_t CalReducerAglErr3;//编码器校准的减速机端主辅码道角度差3
}Encode_RT;

extern Encode_RT RTData;
extern void RT_ParaInit(void);
extern void RT_Process(void);
extern void RT_FaultHandle(void);
#endif
