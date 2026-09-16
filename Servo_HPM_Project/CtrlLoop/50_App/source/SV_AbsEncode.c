#include "userdefine.h"
#include "SV_AbsEncode.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"//
#include "string.h"
#include "Drive.h"
#include "s_comm_init.h"

//TODO:电机CODE相关信息存入编码器EEPROM //
Encode_Type EncInfo = {
/* Encode_BYTEType Encdata;		*/{0},
/* EncodeFlag_Type EncFlag;		*/{0},
/* Uint16 DelayR;				*/0,
/* Uint16 DelayW;				*/0,
/* Uint16 ReadMotorcode;		*/0,
/* Uint16 WriteMotorcode;		*/0,
/*------*/
/* Uint16 NoAckTime;			*/0,
/* Uint16 ErrCRCcnt;			*/0,
/* Uint16 EncBAcnt;				*/0,
/* Uint16 EncBEcnt;				*/0,
/* Uint16 EncMEcnt;				*/0,
/* Uint16 EncOHcnt;				*/0,
/* Uint16 EncOFcnt;				*/0,
/* Uint16 EncCEcnt;				*/0,
/* Uint16 EncFScnt;				*/0,
/* Uint16 EncOScnt;				*/0,

/* Uint16 EncTxcnt;             */0,//编码器复位多圈值和错误使用
/* Uint16 EncRxNum;             */5,
/*        EncCycTime            */0,
/*         EncCyccnt            */0,
};

Encode_BYTEType Encdat;
extern Uint8 UartRxuff[22];

void EncodeTxRequest(Uint8 data)
{
    EncodeSCI_TxData = data;
}

void EncodeRxRequest(void)
{ 
    uint8_t RxNum;
    uint8_t crc = 0;
    uint8_t flag;

    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;

    RxNum = EncodeSCI_FIFO_NUM;
    if(RxNum == EncInfo.EncRxNum)
    {
        UartRxuff[0] = EncodeSCI_RxData;
        UartRxuff[1] = EncodeSCI_RxData;
        UartRxuff[2] = EncodeSCI_RxData;
        UartRxuff[3] = EncodeSCI_RxData;
        UartRxuff[4] = EncodeSCI_RxData;
        UartRxuff[5] = EncodeSCI_RxData;
        UartRxuff[6] = EncodeSCI_RxData;
        UartRxuff[7] = EncodeSCI_RxData;
        UartRxuff[8] = EncodeSCI_RxData;
        UartRxuff[9] = EncodeSCI_RxData;
        UartRxuff[10] = EncodeSCI_RxData;

        crc ^= UartRxuff[0];
        crc ^= UartRxuff[1];
        crc ^= UartRxuff[2];
        crc ^= UartRxuff[3];
        crc ^= UartRxuff[4];
        crc ^= UartRxuff[5];
        crc ^= UartRxuff[6];
        crc ^= UartRxuff[7];
        crc ^= UartRxuff[8];
        crc ^= UartRxuff[9];

        flag = (crc == UartRxuff[10]) ? 1 : 0;
        rxnoackcnt = 0;
    }
    else if(RxNum == EncInfo.EncRxNum)
    {
        UartRxuff[0] = EncodeSCI_RxData;
        UartRxuff[1] = EncodeSCI_RxData;
        UartRxuff[2] = EncodeSCI_RxData;
        UartRxuff[3] = EncodeSCI_RxData;
        UartRxuff[4] = EncodeSCI_RxData;
        UartRxuff[5] = EncodeSCI_RxData;

        crc ^= UartRxuff[0];
        crc ^= UartRxuff[1];
        crc ^= UartRxuff[2];
        crc ^= UartRxuff[3];
        crc ^= UartRxuff[4];

        flag = (crc == UartRxuff[5]) ? 1 : 0;
    }
    else
    {
        rxnoackcnt++;
        if(rxnoackcnt >= Etime_Error_NACK)
        {
            EncInfo.EncFlag.NoAckFlag = 1;
        }
        EncInfo.NoAckTime++;

        EncodeSCI_ResetFIFO;

        return ;
    }

    if(flag == 1)
    {
        Uint32 pos;
        
        pos = UartRxuff[4];
        pos = pos<<8;
        pos += UartRxuff[3];
        pos = pos<<8;
        pos += UartRxuff[2];
        EncInfo.Encdata.ABS = pos;
        
        pos = UartRxuff[7];
        pos = pos<<8;
        pos += UartRxuff[6];
        EncInfo.Encdata.ABM = (Uint16)pos; 
        
        EncInfo.Encdata.SF.SF_all = UartRxuff[1];
        EncInfo.Encdata.ALMC.ALMC_all = UartRxuff[9];

        rxcrcerrcnt = 0;        
    }
    else
    {
        rxcrcerrcnt++;
        if(rxcrcerrcnt >= Etime_Error_CRC)
        {
            EncInfo.EncFlag.CrcErrFlag = 1;
        }
        EncInfo.ErrCRCcnt++;
    }
}

void T_FormatProcess(void)
{
    if(++EncInfo.EncCyccnt == EncInfo.EncCycTime)
    {
        EncodeRxRequest();     //
        if(RamAid->FaultReset == 0 || RamAid->FaultReset == 1)
        {
            EncodeTxRequest(0x1A); //    1300 - 950
            EncInfo.EncRxNum = 11;
        }
        else if(RamAid->FaultReset == 2)
        {
            EncodeTxRequest(0xBA); //    1300 - 950
            EncInfo.EncRxNum = 6;
            EncInfo.EncTxcnt++;
            if(EncInfo.EncTxcnt == 10)
            {
                EncInfo.EncTxcnt = 0;
                RamAid->FaultReset = 0;
            }
        }
        else if(RamAid->FaultReset == 5)
        {
            EncodeTxRequest(0x62); //    1300 - 950
            EncInfo.EncRxNum = 6;
            EncInfo.EncTxcnt++;
            if(EncInfo.EncTxcnt == 10)
            {
                EncInfo.EncTxcnt = 0;
                RamAid->FaultReset = 0;
            }
        }
        else if(RamAid->FaultReset == 8)
        {
            EncodeTxRequest(0xC2); //    1300 - 950
            EncInfo.EncRxNum = 6;
            EncInfo.EncTxcnt++;
            if(EncInfo.EncTxcnt == 10)
            {
                EncInfo.EncTxcnt = 0;
                RamAid->FaultReset = 0;
            }
        }
        else
        {
            EncInfo.EncRxNum = 0;
        }
        
        EncInfo.EncCyccnt = 0;
        EncInfo.EncTxflag = 1;
    }
}

//编码器相关故障检测（1ms定时器调用）
void EncFaultHandle(void)
{
	//1.CRC校验检测
    if(EncInfo.EncFlag.CrcErrFlag == 1)
    {
        EncInfo.EncFlag.CrcErrFlag = 0;
        FaultPrtt_FaultInterface(EncoderErr_AB);
    }

	//2.无响应检测
#if ENCODE_ERR_NACK
	if(EncInfo.EncFlag.NoAckFlag == 1)
	{
		EncInfo.EncFlag.NoAckFlag = 0;
		FaultPrtt_FaultInterface(EncoderErr_1);
	}
#endif

#if ENCODE_READTYPE == ENCODE_RADEALL//以下信息多圈返回数据才有
	//3.电压检测
#if ENCODE_ERR_BA
	if(EncInfo.Encdata.ALMC.ALMC_f.BA)
	{
		EncInfo.EncBAcnt ++;
		if(EncInfo.EncBAcnt >= Etime_Error_BA)
		{
			EncInfo.EncBAcnt = Etime_Error_BA;
			FaultPrtt_FaultInterface(AbsEncBaErr);
		}
	}
	else
	{
		EncInfo.EncBAcnt = 0;
	}

#endif
#if ENCODE_ERR_BE
	if(EncInfo.Encdata.ALMC.ALMC_f.BE)
	{
		EncInfo.EncBEcnt ++;
		if(EncInfo.EncBEcnt >= Etime_Error_BE)
		{
			EncInfo.EncBEcnt = Etime_Error_BE;
			FaultPrtt_FaultInterface(AbsEncBeErr);
		}
	}
	else
	{
		EncInfo.EncBEcnt = 0;
	}
#endif

	//5.多圈检测
#if ENCODE_ERR_ME
	if(EncInfo.Encdata.ALMC.ALMC_f.ME)
	{
		EncInfo.EncMEcnt++;
		if(EncInfo.EncMEcnt >= Etime_Error_ME)
		{
			EncInfo.EncMEcnt = Etime_Error_ME;
            if(RamMotor->ABSEnc == 0x01)
            {
                FaultPrtt_FaultInterface(EncoderErr_M);
            }
		}
	}
	else
	{
		EncInfo.EncMEcnt = 0;
	}
#endif

	//6.过热检测
#if ENCODE_ERR_OH
	if(EncInfo.Encdata.ALMC.ALMC_f.OH)
	{
		EncInfo.EncOHcnt++;
		if(EncInfo.EncOHcnt >= Etime_Error_OH)
		{
			EncInfo.EncOHcnt = Etime_Error_OH;
			FaultPrtt_FaultInterface(AbsEncOhErr);
		}
	}
	else
	{
		EncInfo.EncOHcnt = 0;
	}
#endif

	//7.溢出检测
#if ENCODE_ERR_OF
	if(EncInfo.Encdata.ALMC.ALMC_f.OF)
	{
		EncInfo.EncOFcnt++;
		if(EncInfo.EncOFcnt >= Etime_Error_OF)
		{
			EncInfo.EncOFcnt = Etime_Error_OF;
		}
	}
	else
	{
		EncInfo.EncOFcnt = 0;
	}
#endif

	//8.计数错误检测
#if ENCODE_ERR_CE
	if(EncInfo.Encdata.ALMC.ALMC_f.CE)
	{
		EncInfo.EncCEcnt++;
		if(EncInfo.EncCEcnt >= Etime_Error_CE)
		{
			EncInfo.EncCEcnt = Etime_Error_CE;
			FaultPrtt_FaultInterface(EncoderErr_Z);
		}
	}
	else
	{
		EncInfo.EncCEcnt = 0;
	}
#endif

	//9.绝对位置检测
#if ENCODE_ERR_FS
	if(EncInfo.Encdata.ALMC.ALMC_f.FS)
	{
		EncInfo.EncFScnt++;
		if(EncInfo.EncFScnt >= Etime_Error_FS)
		{
			EncInfo.EncFScnt = Etime_Error_FS;
		}
	}
	else
	{
		EncInfo.EncFScnt = 0;
	}
#endif

	//10.超速检测
#if ENCODE_ERR_OS
	if(EncInfo.Encdata.ALMC.ALMC_f.OS)
	{
		EncInfo.EncOScnt++;
		if(EncInfo.EncOScnt >= Etime_Error_OS)
		{
			EncInfo.EncOScnt = Etime_Error_OS;
		}
	}
	else
	{
		EncInfo.EncOScnt = 0;
	}
#endif
#endif
}

