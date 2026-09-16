/*
 * SV_AbsEncode.h
 *
 *  Created on: 2015-12-2
 *      Author: rd0217
 */
#ifndef __SV_ABSENCODE_H__
#define __SV_ABSENCODE_H__
#include "userdefine.h"//
#include "IQmathLib.h" 
////////////////////////////////////////////////////////////////////////////
#define ENCODE_RADEONE		(0)
#define ENCODE_RADEALL		(1)
#define ENCODE_READTYPE		ENCODE_RADEALL	//此处用于选择读取数据类型
#if ENCODE_READTYPE == ENCODE_RADEALL
#define ENCODE_READNUM		(11)			//读脉冲返回数据
#else
#define ENCODE_READNUM		(6)
#endif
//故障检测使能开关
#define ENCODE_CRCERR_Schedule		(1)
#define ENCODE_CRCERR_CtrlLoop		(2)
#define ENCODE_CRCERR		(ENCODE_CRCERR_CtrlLoop)//此宏定义决定CRC放到控制环还是调度中执行
#define ENCODE_ERR_CRC		(1)				//CRC校验故障
#define ENCODE_ERR_NACK		(1)				//总线无响应故障
#define ENCODE_ERR_BA		(1)				//电池警告
#define ENCODE_ERR_BE		(1)				//电池故障
#define ENCODE_ERR_ME		(1)				//多圈错误
#define ENCODE_ERR_OH		(1)				//过热故障
#define ENCODE_ERR_OF		(0)				//计数溢出
#define ENCODE_ERR_CE		(1)				//计数错误
#define ENCODE_ERR_FS		(0)				//多圈状态
#define ENCODE_ERR_OS		(0)				//超速
////////////////////////////////////////////////////////////////////////////
typedef enum{
	CF_ID0_ReadOne1 = 0x02,			//读单圈数
	CF_ID3_ReadAll1 = 0x1A,			//读取所有数据
	CF_ID6_WriteEEPROM1 = 0x32,		//写EEPROM
	CF_IDD_ReadEEPROM1 = 0xEA,		//读EEPROM
	CF_IDC_Reset1 = 0x62,			//写复位

	//以下未注释只是部分编码器有，故为通用，不采用；带注释部分为不建议采用；
	CF_ID1_ReadMulti = 0x8A,
	CF_ID2_ReadENID = 0x92,			//读编码器ID
	CF_ID7_Reset1 = 0xBA,
	CF_ID8_Reset2 = 0xC2,			//也是返回单圈数据
	CF_IDC_Reset3 = 0x62
}ENUM_CF;//未注释部分单圈编码器无此ID,只能用有注释的

typedef enum{
	ETime_TxCF = 40 + 5,			//读数据请求时间(0.1us)
	ETime_TxREep = 40*3 + 5,		//读EEP数据请求时间(0.1us)
	ETime_TxWEep = 40*4 + 5,		//写EEP数据请求时间(0.1us)

	ETime_RxROne = 25 + 40*6 + 2,	//读单圈数据返回时间(0.1us) 30-5=25
	ETime_RxRAll = 5150,	        //读所有数据返回时间(0.1us)
	ETime_RxREep = 25 + 40*4 + 2,	//读EEP数据返回时间(0.1us)
	ETime_RxWEep = 25 + 40*4 + 2,	//写EEP数据返回时间(0.1us)

	Etime_DlEepPageChg = 18,		//EEP页改变后操作延时时间(ms)
	Etime_Error_CRC = 10,			//CRC校验错误检测时间(次)
	Etime_Error_NACK = 10,			//无响应检测时间(次)
	Etime_Error_BA = 5,				//检测时间(ms)
	Etime_Error_BE = 5,				//检测时间(ms)
	Etime_Error_ME = 5,				//检测时间(ms)
	Etime_Error_OH = 100,			//检测时间(ms)
	Etime_Error_OF = 5,				//检测时间(ms)
	Etime_Error_CE = 5,				//检测时间(ms)
	Etime_Error_FS = 5,				//检测时间(ms)
	Etime_Error_OS = 5,				//检测时间(ms)
	Etime_Other = 100				//其它(us/ms)
}ENUM_ENCTIME;//各操作时间定义

typedef enum
{
	IDLEL =0,				//无操作
	RPOS =1, 				//读编码器状态及位置值
	REEPL =2, 				//读EEPROM
	REEPH =3,				//读EEPROM
	WEEPL =4, 				//写EEPROM
	WEEPH =5,				//写EEPROM
	REST_Err = 6,			//复位
	REST_Multi = 7          //复位
}ENCTXTYPE;

typedef struct
{
	Uint8 WMotorcode:1;		//写电机代码标志（键盘判断PE-47=1置1）
	Uint8 RMotorcode:1;		//读电机代码标志(没有地方置1)
	Uint8 ScibBusy:1;			//Scib互斥信号量，进行读写电机代码时，不允许读取编码器位置信息帧发送
	Uint8 PowerupRead:1;		//上电读取编码器电机代码标志
	Uint8 CrcErrFlag:1;
	Uint8 NoAckFlag:1;
	Uint8 Flag6:1;
	Uint8 Flag7:1;
}EncodeFlag_Type;
typedef struct{
	Uint8 dd:4;			//Information(固定0)

	Uint8 ea0_CE:1;		//Counting Error
	Uint8 ea1_Err:1;	//其他状态
	Uint8 ca0_PE:1;		//Parity Error
	Uint8 ca1_DE:1;		//Delimiter Error
}TYPE_SF_DEFINE;
typedef struct{
	Uint8 OS:1;			//OverSpeed
	Uint8 FS:1;			//Full absolute status
	Uint8 CE:1;			//Countering Error
	Uint8 OF:1;			//Counter Overflow

	Uint8 OH:1;			//OverHeat
	Uint8 ME:1;			//Multi_turn Error
	Uint8 BE:1;			//Battery Error
	Uint8 BA:1;			//Battery Alarm
}TYPE_ALMC_DEFINE;
typedef struct
{
	Uint8 CF;			//控制域
	union UNION_SF{
	Uint8 SF_all;		//状态域
	TYPE_SF_DEFINE SF_f;
	}SF;
	Uint8 ABS0;			//单圈绝对位置值
	Uint8 ABS1;
	Uint8 ABS2;
	Uint8 ENID;			//编码器ID
	Uint8 ABM0;			//多圈位置值
	Uint8 ABM1;
	Uint8 ABM2;
	union UNION_ALMC{
	Uint8 ALMC_all;		//编码器故障信息
	TYPE_ALMC_DEFINE ALMC_f;
	}ALMC;
	Uint8 CRC1;
    
    Uint32 ABS;
    Uint16 ABM;
}Encode_BYTEType;
typedef struct
{
	Encode_BYTEType Encdata;	//编码器数据
	EncodeFlag_Type EncFlag;	//编码器控制寄存器
	Uint8 DelayR;				//读编码器EEP数据帧延时
	Uint8 DelayW;				//写编码器EEP数据帧延时
	Uint8 ReadMotorcode;		//从编码器读到的电机代码
	Uint8 WriteMotorcode;		//往编码器写的电机代码

	Uint8 NoAckTime;			//无数据返回计时
	Uint8 ErrCRCcnt;			//CRC校验错误次数
	Uint8 EncBAcnt;			//编码器电池电压过低警告
	Uint8 EncBEcnt;			//编码器电池电压过低多圈数据丢失
	Uint8 EncMEcnt;			//
	Uint8 EncOHcnt;			//编码器过热次数
	Uint8 EncOFcnt;			//
	Uint8 EncCEcnt;		    //编码器 计数错误
	Uint8 EncFScnt;			//
	Uint8 EncOScnt;			//编码器超速次数

	Uint8 EncTxcnt;            //编码器复位多圈值使用
    Uint8 EncRxNum;
    
    Uint8 EncCycTime;
    Uint8 EncCyccnt;
    Uint8 EncTxflag;
}Encode_Type;

extern Encode_BYTEType Encdat;
extern Encode_Type EncInfo;
extern void EncodeTxRequest(Uint8 data);
extern void EncodeRxRequest(void);
extern void EncFaultHandle(void);
extern void T_FormatProcess(void);
#endif
