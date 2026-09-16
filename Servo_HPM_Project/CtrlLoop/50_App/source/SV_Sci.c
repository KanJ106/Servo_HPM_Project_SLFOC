/*
 * SV_Sci.c
 *
 *  Created on: 2015-12-24//
 *      Author: rd0217
 *///


/******************************************************************************
**                           深圳市同川科技有限公司
**                               TC200伺服驱动器
**                               www.tc-tech.net
**
----------------------------------文件信息-------------------------------------
**文   件   名：SCI_Process.c
**创   建   人：杨玉亮
**最后修改日期：2015-7-8
**对外接口函数 	void SCI_Pruint16_tUint16 ch)
** 调用驱动函数 SCIA_GetBufRxNum,	SCIA_GetBufTxNum,		SCIA_ClrRxBuf,			SCIA_ClrTxBuf,			SCIA_BufRx,
	             SCIA_BufTx,

**描        述：IIC存储及读取调度函数，1ms 执行任务一次，
**其        它：无
**函 数 清  单：
				0x03 执行时间22us
				0x10 42 写8个功能码需要最长运行时间80us
**历        史：修改历史记录列表，每条记录包括修改日期、修改者及修改内容简述
                1.日期：
                作者：
                描述：
                2....
******************************************************************************/
#include "SV_FuncCode.h"
#include "SV_Sci.h"
#include "string.h"
#include "SV_PanelCtl.h"
#include "userdefine.h"
#include "r_cg_Project.h"
#include "SV_DataType.h"
#include "SV_I2c.h"
#include "cia402appl.h"
#include "coeappl.h"
#include "SV_ModbusConTrolVar.h"
#include "ota.h"
#include "s_sys_init.h"
     
#define EEPROM_ADDR		        (0x20)
#define EEPROM_ADDRAND	        (0xE0)

//uint16_t crc_table[256] = {
//0x0000	,0xC0C1	,0xC181	,0x0140	,0xC301	,0x03C0	,0x0280	,0xC241	,0xC601	,0x06C0,
//0x0780	,0xC741	,0x0500	,0xC5C1	,0xC481	,0x0440	,0xCC01	,0x0CC0	,0x0D80	,0xCD41,
//0x0F00	,0xCFC1	,0xCE81	,0x0E40	,0x0A00	,0xCAC1	,0xCB81	,0x0B40	,0xC901	,0x09C0,
//0x0880	,0xC841	,0xD801	,0x18C0	,0x1980	,0xD941	,0x1B00	,0xDBC1	,0xDA81	,0x1A40,
//0x1E00	,0xDEC1	,0xDF81	,0x1F40	,0xDD01	,0x1DC0	,0x1C80	,0xDC41	,0x1400	,0xD4C1,
//0xD581	,0x1540	,0xD701	,0x17C0	,0x1680	,0xD641	,0xD201	,0x12C0	,0x1380	,0xD341,
//0x1100	,0xD1C1	,0xD081	,0x1040	,0xF001	,0x30C0	,0x3180	,0xF141	,0x3300	,0xF3C1,
//0xF281	,0x3240	,0x3600	,0xF6C1	,0xF781	,0x3740	,0xF501	,0x35C0	,0x3480	,0xF441,
//0x3C00	,0xFCC1	,0xFD81	,0x3D40	,0xFF01	,0x3FC0	,0x3E80	,0xFE41	,0xFA01	,0x3AC0,
//0x3B80	,0xFB41	,0x3900	,0xF9C1	,0xF881	,0x3840	,0x2800	,0xE8C1	,0xE981	,0x2940,
//0xEB01	,0x2BC0	,0x2A80	,0xEA41	,0xEE01	,0x2EC0	,0x2F80	,0xEF41	,0x2D00	,0xEDC1,
//0xEC81	,0x2C40	,0xE401	,0x24C0	,0x2580	,0xE541	,0x2700	,0xE7C1	,0xE681	,0x2640,
//0x2200	,0xE2C1	,0xE381	,0x2340	,0xE101	,0x21C0	,0x2080	,0xE041	,0xA001	,0x60C0,
//0x6180	,0xA141	,0x6300	,0xA3C1	,0xA281	,0x6240	,0x6600	,0xA6C1	,0xA781	,0x6740,
//0xA501	,0x65C0	,0x6480	,0xA441	,0x6C00	,0xACC1	,0xAD81	,0x6D40	,0xAF01	,0x6FC0,
//0x6E80	,0xAE41	,0xAA01	,0x6AC0	,0x6B80	,0xAB41	,0x6900	,0xA9C1	,0xA881	,0x6840,
//0x7800	,0xB8C1	,0xB981	,0x7940	,0xBB01	,0x7BC0	,0x7A80	,0xBA41	,0xBE01	,0x7EC0,
//0x7F80	,0xBF41	,0x7D00	,0xBDC1	,0xBC81	,0x7C40	,0xB401	,0x74C0	,0x7580	,0xB541,
//0x7700	,0xB7C1	,0xB681	,0x7640	,0x7200	,0xB2C1	,0xB381	,0x7340	,0xB101	,0x71C0,
//0x7080	,0xB041	,0x5000	,0x90C1	,0x9181	,0x5140	,0x9301	,0x53C0	,0x5280	,0x9241,
//0x9601	,0x56C0	,0x5780	,0x9741	,0x5500	,0x95C1	,0x9481	,0x5440	,0x9C01	,0x5CC0,
//0x5D80	,0x9D41	,0x5F00	,0x9FC1	,0x9E81	,0x5E40	,0x5A00	,0x9AC1	,0x9B81	,0x5B40,
//0x9901	,0x59C0	,0x5880	,0x9841	,0x8801	,0x48C0	,0x4980	,0x8941	,0x4B00	,0x8BC1,
//0x8A81	,0x4A40	,0x4E00	,0x8EC1	,0x8F81	,0x4F40	,0x8D01	,0x4DC0	,0x4C80	,0x8C41,
//0x4400	,0x84C1	,0x8581	,0x4540	,0x8701	,0x47C0	,0x4680	,0x8641	,0x8201	,0x42C0,
//0x4380	,0x8341	,0x4100	,0x81C1	,0x8081	,0x4040};

SCICtlVal_Type *SCICtlVal;		//sci结构体指针

SCICtlVal_Type SCICtlValA = SCIA_INIT;

TYPE_SCI_PreData SCI_PreData =
{
	/*BaudRate=*/1,/*DataFormat = */0,/*SCIModeSel=*/0,/*CardRS232Baud=*/ 0
};

Uint16 SCIcrc = 0;

volatile uint8_t UartBootFlag;
volatile uint8_t UartBootRxEndFlag;
extern uint16_t SnWriteFlag;
////计算CRC校验，最终结果为实际CRC结果，发送时请颠倒
//uint16_t CRC16(uint8_t *msg , uint16_t len){
//	TYPE_WORD crc;
//	uint8_t index;

//	crc.all = 0xFFFF;
//	while(len--)
//    {
//		index = crc.Byte.Low ^ *msg++;
//		crc.all = crc_table[index] ^ (crc.Byte.High);
//	}
//	return crc.all;
//}

/******************************************************************************
**函 数 名：void SCI_SetBaud(void)
**描    述：波特率设置函数
**调    用：无
**输    入：
**输    出：
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_SetBaud(void)
{
	//switch(RamCommu->Baud)	 //P7-01 通信波特率
	//{
	//	case SCI_BUAD_4800:
	//		SetBaud(SCI_BUAD_4800);//波特率为4800
	//		SCICtlVal->SCI_Delay = 8 + RamCommu->AckDelay1;//SCI没有接收到数据的时间长度
	//		break;
	//	case SCI_BUAD_9600:
	//		SetBaud(SCI_BUAD_9600);//波特率为9600
	//		SCICtlVal->SCI_Delay = 4 + RamCommu->AckDelay1;
	//		break;
	//	case SCI_BUAD_19200:
	//		SetBaud(SCI_BUAD_19200);//波特率为19200
	//		SCICtlVal->SCI_Delay = 2 + RamCommu->AckDelay1;
	//		break;
	//	case SCI_BUAD_38400:
	//		SetBaud(SCI_BUAD_38400);//波特率为38400
	//		SCICtlVal->SCI_Delay = 2 + RamCommu->AckDelay1;
	//		break;
	//	case SCI_BUAD_57600:
	//		SetBaud(SCI_BUAD_57600);//波特率为57600
	//		SCICtlVal->SCI_Delay = 2 + RamCommu->AckDelay1;
	//		break;
	//	case SCI_BUAD_115200:
	//		SetBaud(SCI_BUAD_115200);//波特率为115200
	//		SCICtlVal->SCI_Delay = 2 + RamCommu->AckDelay1;
	//		break;
 //       case SCI_BUAD_500000:
	//		SetBaud(SCI_BUAD_500000);//波特率为500000
	//		SCICtlVal->SCI_Delay = 2 + RamCommu->AckDelay1;
	//		break;
	//	default:
	//		SetBaud(SCI_BUAD_9600);//波特率为9600
	//		SCICtlVal->SCI_Delay = 4 + RamCommu->AckDelay1;
	//		break;

	//}
}
/******************************************************************************
**函 数 名：void SCIA_DataForuint16_tnt16 Format)
**描    述：485通信的数据格式设置
**调    用：无
**输    入：
**输    出：
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_DataFormat(uint8_t Format)
{
	//switch(Format)
	//{
	//	case FORMAT_N_STOP1://1位开始位+8位数据位+1位停止位
	//		SetStopBit(SCI_STOPBITP_1);
	//		SetParity(SCI_NOPARITY);
	//	 	break;
	//	case FORMAT_O_STOP1://1位开始位+8位数据位+1位奇校验+1位停止位
	//		SetStopBit(SCI_STOPBITP_1);
	//		SetParity(SCI_ODDPARTTY);
	//	 	break;
	//	case FORMAT_E_STOP1://1位开始位+8位数据位+1位偶校验+1位停止位
	//		SetStopBit(SCI_STOPBITP_1);
	//		SetParity(SCI_EVEN_PARITY);
	//	 	break;
	//	case FORMAT_N_STOP2://1位开始位+8位数据位+2位停止位
	//		SetStopBit(SCI_STOPBITP_2);
	//		SetParity(SCI_NOPARITY);
	//	 	break;
	//	case FORMAT_O_STOP2://1位开始位+8位数据位+1位奇校验+2位停止位
	//		SetStopBit(SCI_STOPBITP_2);
	//		SetParity(SCI_ODDPARTTY);
	//	 	break;
	//	case FORMAT_E_STOP2://1位开始位+8位数据位+1位偶校验+2位停止位
	//		SetStopBit(SCI_STOPBITP_2);
	//		SetParity(SCI_EVEN_PARITY);
	//	 	break;
	//	default:
 //           SetStopBit(SCI_STOPBITP_1);
	//		SetParity(SCI_NOPARITY);
 //         break;
	//}
}
/******************************************************************************
**函 数 名：void SCI_Iniuint16_tnt16 SCI_CH)
**描    述：通道初始化
**调    用：无
**输    入：
**输    出：
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_InitCh(uint8_t SCI_CH)
{
    if(SCICtlVal->TxStatus == TX_READY_STATUS)
    {
        switch(SCI_CH)
        {
            case SCI_CHA:									//485通信结构体
                SCICtlValA.Ch=SCI_CHA;
                SCICtlVal = &SCICtlValA;
                break;
            case SCI_CHB:									//键盘通信
                SCICtlValA.Ch=SCI_CHB;
                SCICtlVal = &SCICtlValA;
                break;
            default:
                break;
        }
        
        SCICtlVal->AckDelay = RamCommu->AckDelay;
        SCICtlVal->SCI_Addr = RamCommu->Addrss;

        //数据格式改变
        if(RamCommu->Format!=SCI_PreData.DataFormat)
        {
            SCI_DataFormat(RamCommu->Format);	//设置数据格式
            SCI_PreData.DataFormat = RamCommu->Format;
        }
        
        //波特率设置改变
        if((RamCommu->Baud != SCI_PreData.BaudRate))
        {
            SCI_SetBaud();							//设置波特率
            SCI_PreData.BaudRate = RamCommu->Baud;
        }
    } 
}
/******************************************************************************
**函 数 名：接收数据包函数
**描    述：5ms没有收到数据则认为已收完一帧数据包
**调    用：无
**输    入：SCI_Flag.RX_Byte：收到字节标志位
**输    出：SCI_Flag.RX_Word：收完一帧数据标志
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_RX_Data(void)
{
    SCICtlVal->CurRxByte_Cnt = SCICtlVal->SCI_GetBufRxNum();
    
    //比较当前接收到的数据个数是否有变化
	if((SCICtlVal->CurRxByte_Cnt == SCICtlVal->PreRxByte_Cnt)&&(SCICtlVal->CurRxByte_Cnt != 0))
	{
		//没有接收到数据，时间长度加1
		SCICtlVal->RxEndT_Cnt ++;
		if(SCICtlVal->RxEndT_Cnt >= SCICtlVal->SCI_Delay)  //一帧数据接收完成
		{
			SCICtlVal->RxEndT_Cnt =0;					       //将接收数据的时间清0
			SCICtlVal->RxByte_Cnt = SCICtlVal->CurRxByte_Cnt;  //保持当前接收数据个数
			SCICtlVal->CurRxByte_Cnt = 0;                      //清除当前接收数据个数
			SCICtlVal->PreRxByte_Cnt = 0;                      //清除上次接收数据个数
			memset(&SCICtlVal->SCI_RX_BUF,0,SCI_FRAM_SIZE);    //将接收区清0
			SCICtlVal->SCI_RxData(SCICtlVal->SCI_RX_BUF,SCICtlVal->RxByte_Cnt); //取数据
			SCICtlVal->SCI_ClrRxBuf(); 						   //清除接收区数据
            #if SERVO_UpComputer == HarmoCore
            SCIcrc = CrcCheck5;
            #elif SERVO_UpComputer == ServoTool
            SCIcrc = CrcCheck3;
            #endif
            CrcCheck = 0xFFFF;
            
            if(SCICtlVal->SCI_GetBufRxbufflag() == 0)          //判断是否溢出，排除SCOPE发送数据
            {
                SCICtlVal->RxEndFlag = RX_DATA_END;        	   //接收完成   
            }
		}
	}
	else
	{
		SCICtlVal->PreRxByte_Cnt = SCICtlVal->CurRxByte_Cnt; //保存前一个解释数据的个数
		SCICtlVal->RxEndT_Cnt =0;
	}
}

/******************************************************************************
**函 数 名：错误信息函数
**描    述：通信发生错误时，准备错误信息响应
**调    用：CRC()校验函数
**输    入：SCI_RX_BUF: 接收数据帧数组
**输    出：SCI_TX_BUF：发送数据帧数组
			SCI_ByteNum.TX：发送字节个数
			SCI_Flag.TX_Ready：准备好一帧发送数据
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
static void SCI_Fault_Func(void)
{
	TYPE_WORD SCI_Check;
	//组帧要发送的错误数据帧
	SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];			 //地址码
	SCICtlVal->SCI_TX_BUF[1] = SCICtlVal->SCI_RX_BUF[1] +SCI_CMD_ERR;//功能码加0x80，
	SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->FaultCode;				 //故障代码

	//SCI_Check.all= CRC16(SCICtlVal->SCI_TX_BUF, 3);
	//SCICtlVal->SCI_TX_BUF[3] = SCI_Check.Byte.Low;					  //校验区
	//SCICtlVal->SCI_TX_BUF[4] = SCI_Check.Byte.High;

	//发送的数据位5个
	SCICtlVal->TxByte_Cnt = 5;
	SCICtlVal->FaultCode = 0;
	SCICtlVal->TX_Ready = TX_READY_YES;
}
static void SCI_HarmoCore_Fault_Func(void)
{
	TYPE_WORD SCI_Check;
	//组帧要发送的错误数据帧
	SCICtlVal->SCI_TX_BUF[0] = 0x3C;			 
	SCICtlVal->SCI_TX_BUF[1] = 0x3C;
	SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->SCI_RX_BUF[2]; //地址码
    SCICtlVal->SCI_TX_BUF[3] = SCICtlVal->SCI_RX_BUF[3] +SCI_CMD_ERR;//功能码加0x80，
    SCICtlVal->SCI_TX_BUF[4] = SCICtlVal->FaultCode;				 //故障代码
	
    SCICtlVal->SCI_TX_BUF[7] = 0x3E;			 
	SCICtlVal->SCI_TX_BUF[8] = 0x3E;

	//SCI_Check.all= CRC16(SCICtlVal->SCI_TX_BUF, 3);
	//SCICtlVal->SCI_TX_BUF[3] = SCI_Check.Byte.Low;					  //校验区
	//SCICtlVal->SCI_TX_BUF[4] = SCI_Check.Byte.High;

	//发送的数据位5个
	SCICtlVal->TxByte_Cnt = 9;
	SCICtlVal->FaultCode = 0;
	SCICtlVal->TX_Ready = TX_READY_YES;
}
/******************************************************************************
**函 数 名：数据包正确性判断函数
**描    述：对数据包的地址码、长度和校验值进行判断
**调    用：SCI_Fault_Func()：错误信息数据函数
**输    入：SCI_Flag.RxEndFlag：收完一帧数据
            SCI_Flag.Fault：错误标志
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/

void SCI_Frame(void)
{
    TYPE_WORD Rx_OpByteCnt; 
	TYPE_WORD SCI_Crc;
	TYPE_WORD SCI_Check;
//	TYPE_WORD Func;
	TYPE_WORD Rx_Cnt;//读写功能码的个数
	//判定一帧数据是否接收完成
	if(SCICtlVal->RxEndFlag == RX_DATA_END)
	{
		//如果不是本机地址，也不是广播地址，则丢掉该数据帧
		if((SCICtlVal->SCI_RX_BUF[Servo_addr] !=SCICtlVal->SCI_Addr) && (SCICtlVal->SCI_RX_BUF[Servo_addr] != BROADCASTADDR))
		{
			SCICtlVal->RxEndFlag = RX_DATA_NOEND;
			//数据长度错误，地址错误，
		}
        else if(SCICtlVal->RxByte_Cnt < MinByteNum)
        {
            SCICtlVal->RxEndFlag = RX_DATA_NOEND;  //数据长度错误
        }
		else
		{
			switch(SCICtlVal->SCI_RX_BUF[FunctiCode])
			{
				case SCI_CMD_03:
					Rx_Cnt.Byte.High = SCICtlVal->SCI_RX_BUF[OpByteNum_H];
					Rx_Cnt.Byte.Low= SCICtlVal->SCI_RX_BUF[OpByteNum_L];
                    #if SERVO_UpComputer == HarmoCore
                    SCICtlVal->OpRegister = Rx_Cnt.all >> 1; 
					#endif
					//数据帧长度超过8个字节，或者读连续功能码超过16个字节，或者读的功能码为0 丢掉该数据帧
                    if(SCICtlVal->RxByte_Cnt != FRAME_03_SIZE )
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else if((Rx_Cnt.all == 0) || (Rx_Cnt.all > FRAME_03_READ_SIZE ))
					{
						SCICtlVal->FaultCode = ERR_ADDR_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
					break;
				case SCI_CMD_08://线路检测
				case SCI_CMD_6B://发送示波器特殊命令
					if(SCICtlVal->RxByte_Cnt < 3)
					{
						SCICtlVal->RxEndFlag = RX_DATA_NOEND;
					}
					else if(SCICtlVal->RxByte_Cnt != FRAME_08_SIZE)
					{
//						SCICtlVal->RxEndFlag = RX_DATA_NOEND;
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					break;
                #if SERVO_UpComputer == HarmoCore
				case SCI_CMD_06: 
				case SCI_CMD_41:
                    
                    Rx_OpByteCnt.Byte.High = SCICtlVal->SCI_RX_BUF[6];
                    Rx_OpByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
                    SCICtlVal->OpRegister = Rx_OpByteCnt.all >> 1; 
					//数据帧的长度 = 1字节地址 + 1字节功能码 + 2字节地址 + 2 字节寄存器数目 +1 字节内容数 + 2字节校验 + 可变内容字节数
					//数据长度不正确，数据写的功能码个数个字节数不匹配，则丢掉该数据帧
					if(SCICtlVal->RxByte_Cnt != 12 + Rx_OpByteCnt.all)
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else if((0 == Rx_OpByteCnt.all) || (Rx_OpByteCnt.all > FRAME_06_WRITE_SIZE))
					{
						SCICtlVal->FaultCode = ERR_ADDR_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
					break;
                case SCI_CMD_07: //mqb新上位机断电保存
                    
                    Rx_OpByteCnt.Byte.High = SCICtlVal->SCI_RX_BUF[6];
                    Rx_OpByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
                    SCICtlVal->OpRegister = Rx_OpByteCnt.all >> 1; 
					//数据帧的长度 = 1字节地址 + 1字节功能码 + 2字节地址 + 2 字节寄存器数目 +1 字节内容数 + 2字节校验 + 可变内容字节数
					//数据长度不正确，数据写的功能码个数个字节数不匹配，则丢掉该数据帧
					if(SCICtlVal->RxByte_Cnt != 12 + Rx_OpByteCnt.all)
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else if((0 == Rx_OpByteCnt.all) || (Rx_OpByteCnt.all > FRAME_06_WRITE_SIZE))
					{
						SCICtlVal->FaultCode = ERR_ADDR_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
                    break;
                //case SCI_CMD_20: //mqb开始
                //    SCICtlVal->FaultCode = SCI_NO_ERR;
                //    break;
                //case SCI_CMD_30:
                //    SCICtlVal->FaultCode = SCI_NO_ERR;
                //    break;
                //case SCI_CMD_40: //mqb结束
                //    SCICtlVal->FaultCode = SCI_NO_ERR;
                //    break;
                case SCI_CMD_64: 
                    
                    Rx_OpByteCnt.Byte.High = SCICtlVal->SCI_RX_BUF[6];
                    Rx_OpByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
					//数据帧的长度 = 1字节地址 + 1字节功能码 + 2字节地址 + 2 字节寄存器数目 +1 字节内容数 + 2字节校验 + 可变内容字节数
					//数据长度不正确，数据写的功能码个数个字节数不匹配，则丢掉该数据帧
					if(SCICtlVal->RxByte_Cnt != 12)
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
                    break;
                case SCI_CMD_6F:
                    
                    Rx_OpByteCnt.Byte.High = SCICtlVal->SCI_RX_BUF[6];
                    Rx_OpByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
					//数据帧的长度 = 1字节地址 + 1字节功能码 + 2字节地址 + 2 字节寄存器数目 +1 字节内容数 + 2字节校验 + 可变内容字节数
					//数据长度不正确，数据写的功能码个数个字节数不匹配，则丢掉该数据帧
					if(SCICtlVal->RxByte_Cnt != 12)
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
                    break;
                #elif SERVO_UpComputer == ServoTool
				case SCI_CMD_06:		//写功能码的长度不是等于8，则丢掉该数据帧
				case SCI_CMD_41:
                    if(SCICtlVal->RxByte_Cnt != FRAME_06_SIZE)
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
					break;
				case SCI_CMD_10:
				case SCI_CMD_42:
					Rx_Cnt.Byte.High =SCICtlVal->SCI_RX_BUF[4];
					Rx_Cnt.Byte.Low  =SCICtlVal->SCI_RX_BUF[5];
					//数据帧的长度 = 1字节地址 + 1字节功能码 + 2字节地址 + 2 字节寄存器数目 +1 字节内容数 + 2字节校验 + 可变内容字节数
					//数据长度不正确，数据写的功能码个数个字节数不匹配，则丢掉该数据帧
					if(SCICtlVal->RxByte_Cnt != 9 + SCICtlVal->SCI_RX_BUF[6])
					{
						SCICtlVal->FaultCode = ERR_FRAME_INVALID;
					}
					else if((Rx_Cnt.all == 0) || (SCICtlVal->SCI_RX_BUF[6] != (Rx_Cnt.all << 1)) || ( Rx_Cnt.all > FRAME_10_WRITE_SIZE))
					{
						SCICtlVal->FaultCode = ERR_ADDR_INVALID;
					}
					else
					{
                        SCICtlVal->FaultCode = SCI_NO_ERR;
					}
					break;
                #endif
                case SCI_CMD_5A:
                    SCICtlVal->FaultCode = SCI_NO_ERR;
                break;
                case SCI_CMD_A5:
                    if(SCICtlVal->RxByte_Cnt == 262 || SCICtlVal->RxByte_Cnt == 1030)
                    {                        
                        SCICtlVal->FaultCode = SCI_NO_ERR;
                    }
                    else
                    {                        
                        SCICtlVal->FaultCode = ERR_FRAME_INVALID;    
                    }
                break;
                case SCI_CMD_90 :
              
                break;
                case SCI_CMD_91 :
              
                break;
				default:
					//if(SCICtlVal->RxByte_Cnt < 3)
					//{
					//	SCICtlVal->RxEndFlag = RX_DATA_NOEND;
					//}
					//SCICtlVal->FaultCode = ERR_FUNC_INVALID;

                    if(SCICtlVal->SCI_RX_BUF[FunctiCode] == 0x20)
                    {
                        SCICtlVal->FaultCode = SCI_NO_ERR;
                    }
                    else if(SCICtlVal->SCI_RX_BUF[FunctiCode] == 0x30)
                    {
                        SCICtlVal->FaultCode = SCI_NO_ERR;
                    }
                    else if(SCICtlVal->SCI_RX_BUF[FunctiCode] == 0x40)
                    {
                        SCICtlVal->FaultCode = SCI_NO_ERR;
                    }
					else
					{
						SCICtlVal->RxEndFlag = RX_DATA_NOEND;
                        SCICtlVal->FaultCode = ERR_FUNC_INVALID;
					}	
			    break;
			}
		}

		if(SCICtlVal->RxEndFlag != RX_DATA_NOEND)//如果是有效数据帧，则进行CRC校验
		{
			SCI_Crc.Byte.Low = SCICtlVal->SCI_RX_BUF[SCICtlVal->RxByte_Cnt - CrcCheck_L];//校验低字节
			SCI_Crc.Byte.High= SCICtlVal->SCI_RX_BUF[SCICtlVal->RxByte_Cnt - CrcCheck_H];//校验高字节
			//SCI_Check.all= CRC16(SCICtlVal->SCI_RX_BUF,SCICtlVal->RxByte_Cnt -2);  //接收的数据校验计算
            SCI_Check.all = SCIcrc;    //2024.02.24 修改为边接收边计算
			//校验判定
			if(SCI_Check.all != SCI_Crc.all)
			{
				SCICtlVal->FaultCode = ERR_FRAME_INVALID;
			}
            
			if(SCICtlVal->SCI_RX_BUF[Servo_addr] == BROADCASTADDR)//判定是否为广播地址，如果是广播地址，则不响应数据发送
			{
				SCICtlVal->TX_Ack = TX_ACK_NO;	//接收到的是广播地址，不需要相应
			}
			else
			{
				SCICtlVal->TX_Ack = TX_ACK_YES;//需要应答相应数据
			}
		}
	}
}
/******************************************************************************
**函 数 名：void SCI_Addr_Liuint16_tnt16 Func_Aduint16_tnt16 Func_Size)
**描    述：功能码地址限定
**调    用：
**输    入：

**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Addr_Limit(uint16_t Func_Addr, uint16_t Cnt)
{
    //uint16_t Addr;
    TYPE_WORD Rx_Func;

	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
		SCICtlVal->FaultCode = AddrRangLit(Func_Addr);	          //起始地址是否范围内
        if(Cnt > 1)
        {
            SCICtlVal->FaultCode += AddrRangLit(Func_Addr + Cnt - 1); //终止地址是否范围内  2025.09.26 YH
	    }
    }
    
	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
		//Addr = SCICtlVal->FuncAddr;
		Rx_Func.all = Func_Addr;

		//密码没打开，相关参数不可操作
		if(	(Rx_Func.Byte.High == PE_FUN_GROUP) && ((Rx_Func.Byte.Low + Cnt - 1) != 0) &&\
				(Func.Key.ControlBit.PEPassword == !PASSWORDOPEN)
		  )
		{
			SCICtlVal->FaultCode= ERR_PWD_PROTECT; //非法数据地址错误
			return;
		}
		else if((Rx_Func.Byte.High == PF_FUN_GROUP) && \
				(Func.Key.ControlBit.PFPassword == !PASSWORDOPEN)
		  )
		{
			SCICtlVal->FaultCode= ERR_PWD_PROTECT; //非法数据地址错误
			return;
		}

//        //可能要删除
//		if((Addr + Cnt -1) > ModbusOptSize)
//		{
//			SCICtlVal->FaultCode= ERR_ADDR_INVALID; //非法数据地址错误
//		}
	}

//	TYPE_Attribute *Attr = FuncAttrStartAddr + SCI_Limit_Addr;
//	//读取数据为不显示，返回地址错误？？？（是否需要加）
//	for(i=0; i<Cnt; i++)
//	{
//		if((Attr+i)->IsShow == HIDE)
//		{
//			SCICtlVal->FaultCode= ERR_ADDR_INVALID; //非法数据地址错误
//			break;
//		}
//	}
}
/******************************************************************************
 **函 数 名：ExceptionWrDuint16_tnt16 Functuint16_tnt16 *Value)
 **描    述：写例外处理
 *******************************************************************************/
#define Func_FactoryPassword2		(0x013F)
#define Func_MotorPassword			(0x0E00)
void ExceptionWrDeal(uint16_t Function,uint16_t *Value)
{
 	switch(Function)
 	{
 		case Func_FactoryPassword2://厂家密码2 RamBasePara->FactoryCode2
 			if(*Value == PFPASSWORDNUM)//密码正确
		 	{
                Func.Key.ControlBit.PEPassword = 1;
		 		Func.Key.ControlBit.PFPassword = 1;
		 	}
 			*Value = 0;
			//SCICtlValA.SCI_WSave = SCI_W_NOSAVE; //这里不能执行吧，0x10会出问题
 		break;

 		case Func_MotorPassword://电机组密码 RamMotor->MotorPSW
 			if(*Value == PEPASSWORDNUM)//密码正确
		 	{
		 		Func.Key.ControlBit.PEPassword = 1;
                Func.Key.ControlBit.PFPassword = 1;
		 	}
 			*Value = 0;
			//SCICtlValA.SCI_WSave = SCI_W_NOSAVE;
 		break;
 	}
}

void ExceptionWrDeal_10(uint16_t Function,uint16_t *Value)
{
 	switch(Function)
 	{
        case 0x0802:
        case 0x0804:
        case 0x0805:
        case 0x0806:
        case 0x081E:
            *Value = 0;
        break;

 		case Func_FactoryPassword2://厂家密码2 RamBasePara->FactoryCode2
 			if(*Value == PFPASSWORDNUM)//密码正确
		 	{
                Func.Key.ControlBit.PEPassword = 1;
		 		Func.Key.ControlBit.PFPassword = 1;
		 	}
 			*Value = 0;
			//SCICtlValA.SCI_WSave = SCI_W_NOSAVE; //这里不能执行吧，0x10会出问题
 		break;

 		case Func_MotorPassword://电机组密码 RamMotor->MotorPSW
 			if(*Value == PEPASSWORDNUM)//密码正确
		 	{
		 		Func.Key.ControlBit.PEPassword = 1;
                Func.Key.ControlBit.PFPassword = 1;
		 	}
 			*Value = 0;
			//SCICtlValA.SCI_WSave = SCI_W_NOSAVE;
 		break;
 	}
}

Uint16 * SCI_Get_Addr(Uint16 addr)
{
    TYPE_WORD Rx_Func;
    Uint16 *FuncAddr;
    
    Rx_Func.all = addr;
    
    if(Rx_Func.Byte.High <= F_LAST_FUN_GROUP)
    {
        FuncAddr = RamFuncStartAddr + GetFuncAddr(addr);   //获取功能码的地址
    }
    else 
    {
        switch(Rx_Func.Byte.High)
        {
            case E0x1000_GROUP://0x1000
            FuncAddr = ModbusCumObj_StartAddr + addr - 0x1000;
            break;

            case E0x6000_GROUP://0x6000
            FuncAddr = RamCIA402Obj_StartAddr + addr - 0x1800;
            break;
            
            case F76H_FUN_GROUP://示波器配置地址
            FuncAddr = ScopeParaStartAddr + Rx_Func.Byte.Low;   //获取功能码的地址
            break;

            case F77H_FUN_GROUP://示波器配置地址
            FuncAddr = F77HStartAddr + Rx_Func.Byte.Low;   //获取功能码的地址
            break;

            case F78H_FUN_GROUP://示波器配置地址
            FuncAddr = F78HStartAddr + Rx_Func.Byte.Low;   //获取功能码的地址
            break;
            
            default:
            FuncAddr = 0; 
            break;
        }
    }
    
    return FuncAddr;
}

/******************************************************************************
**函 数 名：03功能码函数
**描    述：对03功能码接收数据进行解析
**调    用：SCI_03_Ram()地址映射函数
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
			SCI_Flag.Fault：错误标志
**输    出：SCI_ByteNum.TX：发送数据帧字节个数
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Func_03(void)
{
	uint16_t i;
	TYPE_WORD Rx_Cnt;  //读数据的个数
	TYPE_WORD Rx_Func; //要读的功能码
	TYPE_WORD Rx_Data; //读到的数据
	//TYPE_WORD SCI_Check;
	uint16_t *FuncAddr;

	//接收到的数据个数
	Rx_Cnt.Byte.High = SCICtlVal->SCI_RX_BUF[OpByteNum_H];
    Rx_Cnt.Byte.Low =  SCICtlVal->SCI_RX_BUF[OpByteNum_L];
	//要读的功能码
    Rx_Func.Byte.High = SCICtlVal->SCI_RX_BUF[OpAddr_H];
    Rx_Func.Byte.Low  = SCICtlVal->SCI_RX_BUF[OpAddr_L];

    #if SERVO_UpComputer == HarmoCore
	SCI_Addr_Limit(Rx_Func.all, SCICtlVal->OpRegister);    //读取地址范围限定
    #elif SERVO_UpComputer == ServoTool
	SCI_Addr_Limit(Rx_Func.all, Rx_Cnt.all);    //读取地址范围限定
    #endif

	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
		//FuncAddr = RamFuncStartAddr + SCICtlVal->FuncAddr;//获取功能码的地址
        
        FuncAddr = SCI_Get_Addr(Rx_Func.all);
        #if SERVO_UpComputer == ServoTool
        for(i = 0; i < Rx_Cnt.all; i++)
        {
             Rx_Data.all= *(FuncAddr + i);
              /*读取内容低位 */
              SCICtlVal->SCI_TX_BUF[3 + i*2]= Rx_Data.Byte.High;
              SCICtlVal->SCI_TX_BUF[4 + i*2]= Rx_Data.Byte.Low;
        }
        
		/*地址码*/
	    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
		 /*功能码 */
		SCICtlVal->SCI_TX_BUF[1] = SCI_CMD_03;
		/*读取字节个数 */
		SCICtlVal->SCI_TX_BUF[2] = Rx_Cnt.all * 2;
        
//	    /*CRC校验*/
//	    SCI_Check.all = CRC16(SCICtlVal->SCI_TX_BUF, Rx_Cnt.all *2 +3);
//		SCICtlVal->SCI_TX_BUF[Rx_Cnt.all *2 +3] = SCI_Check.Byte.Low;
//		SCICtlVal->SCI_TX_BUF[Rx_Cnt.all *2 +4] = SCI_Check.Byte.High;
        
		/*发送字节个数*/
		SCICtlVal->TxByte_Cnt = Rx_Cnt.all *2 + 5;
        #elif SERVO_UpComputer == HarmoCore
        for(i = 0; i < SCICtlVal->OpRegister; i++)
        {
             Rx_Data.all= *(FuncAddr + i); 
              /*读取内容低位 */
              SCICtlVal->SCI_TX_BUF[8 + i*2]= Rx_Data.Byte.High;
              SCICtlVal->SCI_TX_BUF[9 + i*2]= Rx_Data.Byte.Low;
        }
		/*帧头*/
	    SCICtlVal->SCI_TX_BUF[0] = 0x3C;//SCICtlVal->SCI_RX_BUF[0];
		 /*帧头*/
		SCICtlVal->SCI_TX_BUF[1] = 0x3C;//SCICtlVal->SCI_RX_BUF[1];
		/*地址码 */
		SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->SCI_RX_BUF[2];
        /*功能码*/
        SCICtlVal->SCI_TX_BUF[3] = SCI_CMD_03;
        /*操作地址高*/
        SCICtlVal->SCI_TX_BUF[4] = SCICtlVal->SCI_RX_BUF[4];
        /*操作地址低*/
        SCICtlVal->SCI_TX_BUF[5] = SCICtlVal->SCI_RX_BUF[5];
        /*读取字节个数高*/
        SCICtlVal->SCI_TX_BUF[6] = Rx_Cnt.Byte.High;
        /*读取字节个数低*/
        SCICtlVal->SCI_TX_BUF[7] = Rx_Cnt.Byte.Low;
        /*帧尾*/
        SCICtlVal->SCI_TX_BUF[10 + Rx_Cnt.all] = 0x3E;
        /*帧尾*/
        SCICtlVal->SCI_TX_BUF[11 + Rx_Cnt.all] = 0x3E;
		/*发送字节个数*/
		SCICtlVal->TxByte_Cnt = Rx_Cnt.all + 12;
        #endif
		 /*准备好一帧发送数据*/
		SCICtlVal->TX_Ready = TX_READY_YES;
	}
}

/******************************************************************************
**函 数 名：写数据限制函数函数
**描    述：写数据是，数据内容是否在该功能码的属性，内容是
            否在最大值和最小值范围内是否为参数只读，参数运行时不可更改，参数受密
			码保护。
**调    用：无
**输    入：SCI_Limit_Addr：输入通信地址
            SCI_Limit_Data：内容
**输    出：SCI_Fault_Flg：错误标志
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
uint16_t  SCI_Write_Limit(uint16_t SCI_Limit_Addr, uint16_t *Limit_Data) //mqb 参数2修改为指针类型的
{
	TYPE_Attribute *Attr;
    Uint16 *Addr;
	uint16_t ret = SCI_NO_ERR;
    
	Attr = FuncAttrStartAddr + GetFuncAddr(SCI_Limit_Addr);
    Addr = SCI_Get_Addr(SCI_Limit_Addr);

	if(Attr->Operation == ATTR_OPE_ReadOnly)
	{
		//如果功能码为仅读，写该功能码是无效的，并报错误
        if(SCICtlVal->OpRegister < 2)
        {
            ret = ERR_PARA_READONLY;
        }
        else 
        {
            
            *Limit_Data = *Addr;
        }
		
	}
	else
	{
		/*如果功能码的属性是一个整体，直接比较最大值，最小值*/
//		if((Attr->ParaType == ATTR_TYPE_Single)||(Attr->ParaType == ATTR_TYPE_Bit))
		{
			if(Attr->Sign == USIGN)
			{
				if((*Limit_Data > Attr->Max.U) || (*Limit_Data < Attr->Min.U))
				{
					ret = ERR_DATA_RANGE;
				}
				else
				{
					ret = SCI_NO_ERR;
				}
			}
			else
			{
				if(((int16_t)*Limit_Data > Attr->Max.S) || ((int16_t)*Limit_Data < Attr->Min.S))
				{
					ret = ERR_DATA_RANGE;
				}
				else
				{
					ret = SCI_NO_ERR;
				}
			}
		}
	}
	return ret;
}
/******************************************************************************
**函 数 名：void SCI_Write_uint16_tnt16 SCI_EEP_Aduint16_tnt16 *SCI_EEP_Dauint16_tnt16 SCI_EEP_Number)
**描    述：写功能码函数

**调    用：无
**输    入：

**输    出
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Write_EEP(uint16_t Addr, uint16_t *SCI_EEP_Data, uint16_t SCI_EEP_Number)
{
	uint16_t *SCI_Ram_Addr;

    SCI_Ram_Addr = SCI_Get_Addr(Addr);
    
    //将这次的功能码保存，作为以后恢复参数时需要恢复的地址
    
    memcpy(SCI_Ram_Addr,SCI_EEP_Data,SCI_EEP_Number*2);
    
    //将这次的功能码保存，作为以后恢复参数时需要恢复的地址
    SCICtlVal->I2cRamFuncAddr = SCI_Ram_Addr;
    SCICtlVal->I2cSize = SCI_EEP_Number;
    SCICtlVal->FuncAddr = GetFuncAddr(Addr);
    SCICtlVal->FunctionCode = Addr;
    
    IIcInterfaceC(SCI_Ram_Addr,SCI_EEP_Number);//IIC写存储函数
    
    ////SCICtlVal->TX_Ready = TX_READY_WAIT;
    
//	switch(SCICtlVal->SCI_WSave)
//	{
//		case SCI_W_SAVE://写保存命令
//		    //memcpy(SCICtlVal->I2cBuf,SCI_Ram_Addr,SCI_EEP_Number*2); //对数据缓存，在存储出错时恢复
//
//			memcpy(SCI_Ram_Addr,SCI_EEP_Data,SCI_EEP_Number*2);
//
//			SCICtlVal->I2cRamFuncAddr = SCI_Ram_Addr;
//			SCICtlVal->I2cSize = SCI_EEP_Number;
//			IIcInterfaceC(SCI_Ram_Addr,SCI_EEP_Number);//掉电IIC写存储函数
//			//这为了防止写的功能码刚好的这些密码，写保存后，不清ram区的密码，密码就会对外显示
//			SCICtlVal->TX_Ready = TX_READY_WAIT;
//			break;
//		case SCI_W_NOSAVE://写不保存命令
//			memcpy(SCI_Ram_Addr,SCI_EEP_Data,SCI_EEP_Number*2);
//			 /*准备好一帧发送数据*/
//	    	SCICtlVal->TX_Ready = TX_READY_YES;
//			break;
//		case SCI_W_RESTORE://参数恢复命令
//			default:break;
//	}
}
/******************************************************************************
**函 数 名：06功能码函数
**描    述：对06功能码接收数据进行解析
**调    用：SCI_In_Limit()：数据限制
            SCI_Write_EEP()：修改数据函数
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
			p：包含IIC地址、数据和写EEPROM标志位的指针
**输    出：SCI_ByteNum.TX：发送字节个数
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Func_06(void)
{
    TYPE_WORD Rx_Func;
    TYPE_WORD W_Data;
    Uint16 *Addr;

    Rx_Func.Byte.High =  SCICtlVal->SCI_RX_BUF[2];/*要写的功能码*/
    Rx_Func.Byte.Low  =  SCICtlVal->SCI_RX_BUF[3];
                                                  
    if((SCICtlVal->SCI_RX_BUF[2] & EEPROM_ADDRAND) == EEPROM_ADDR) //考虑地址+EEPROM_ADDR的情况
    {
        SCICtlVal->SCI_WSave = SCI_W_SAVE;
        Rx_Func.Byte.High = Rx_Func.Byte.High - EEPROM_ADDR;
    }

    W_Data.Byte.High =  SCICtlVal->SCI_RX_BUF[4];/*要写的数据*/
    W_Data.Byte.Low  =  SCICtlVal->SCI_RX_BUF[5];

    SCI_Addr_Limit(Rx_Func.all,1);               //地址范围限定
    
    if(SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        Addr = SCI_Get_Addr(Rx_Func.all);
    }
    
    if(Rx_Func.Byte.High <= PF_FUN_GROUP)
    {
        SCICtlVal->FaultCode = SCI_Write_Limit(Rx_Func.all, &W_Data.all);//写数据限定
    }
    
    if(SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        ExceptionWrDeal(Rx_Func.all, &W_Data.all);
        
        if(*Addr == W_Data.all)
        {
            SCICtlVal->SCI_WSave = SCI_W_NOSAVE;
        }
        
        if(SCICtlVal->SCI_WSave == SCI_W_NOSAVE)
        {
            *Addr = W_Data.all;
            SCICtlVal->TX_Ready = TX_READY_YES;  
        }
        else
        {
            SCI_Write_EEP(Rx_Func.all,&W_Data.all,1);		//存储
            //SCICtlVal->TX_Ready = TX_READY_WAIT; 
            SCICtlVal->TX_Ready = TX_READY_YES;
        }
        memcpy(SCICtlVal->SCI_TX_BUF,SCICtlVal->SCI_RX_BUF,8);
        SCICtlVal->TxByte_Cnt = 8;  						/*响应帧字节个数*/
    }
}
/******************************************************************************
**函 数 名：HarmoCore上位机06功能码函数
**描    述：对06功能码接收数据进行解析
**调    用：SCI_In_Limit()：数据限制
            SCI_Write_EEP()：修改数据函数
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
**输    出：SCI_ByteNum.TX：发送字节个数
**返    回：无
**其    它：无
**日    期：2025-9-2
*******************************************************************************/

void SCI_HarmoCore_Func_06(void)
{
	TYPE_WORD Rx_Func;
	TYPE_WORD W_Data;    //mqb操作的数据
	TYPE_WORD W_ByteCnt; //mqb取字节数目
    uint16_t RegisterNum;//mqb取寄存器数目
  	Uint16 * Addr;

    uint16_t i;

    W_ByteCnt.Byte.High= SCICtlVal->SCI_RX_BUF[6];  
    W_ByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
    if (SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_06) {
        RegisterNum = W_ByteCnt.all >> 1;
    }
    else if ((SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_6F) || (SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_64)) {
        RegisterNum = 1;
        W_ByteCnt.all = 2; 
    }

	Rx_Func.Byte.High= SCICtlVal->SCI_RX_BUF[4]; /*取寄存器的地址*/
	Rx_Func.Byte.Low = SCICtlVal->SCI_RX_BUF[5];

	/*//考虑地址+EEPROM_ADDR的情况
	if((SCICtlVal->SCI_RX_BUF[4] & EEPROM_ADDRAND) == EEPROM_ADDR)
	{
        SCICtlVal->SCI_WSave = SCI_W_SAVE;
		Rx_Func.Byte.High = Rx_Func.Byte.High - EEPROM_ADDR;
	}*/

	SCI_Addr_Limit(Rx_Func.all, RegisterNum);
    if(SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        
        Addr = SCI_Get_Addr(Rx_Func.all);
    }

	if(SCICtlVal->FaultCode == SCI_NO_ERR)//如果没有错误
	{ 
        for(i = 0; i < RegisterNum; i++)
        {
            if(SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_06)
            {
           		W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[8+i*2];
           	    W_Data.Byte.Low = SCICtlVal->SCI_RX_BUF[9+i*2];
            }
            else if ((SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_6F) || (SCICtlVal->SCI_RX_BUF[3] == SCI_CMD_64)) 
            {
                //W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[6];
                //W_Data.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
                W_Data.all = 1; 
            }

            if(Rx_Func.Byte.High <= PF_FUN_GROUP)
            {
                SCICtlVal->FaultCode = SCI_Write_Limit(Rx_Func.all + i, &W_Data.all);
            }
        
            if(SCICtlVal->FaultCode == SCI_NO_ERR)
            {
                //MQB如果操作的寄存器数量大于1,表示要修改多个参数或者是下发参数,此时要注意不能被意外使能
                if(SCICtlVal->OpRegister > 1)
                {
                    ExceptionWrDeal_10(Rx_Func.all + i, &W_Data.all);	
                }
                else {
                    ExceptionWrDeal(Rx_Func.all + i, &W_Data.all);	
                }
                
                SCICtlVal->Write[i] = W_Data.all;//没有错误		
            }
            else  break;
        }
        
        if(SCICtlVal->FaultCode == SCI_NO_ERR)
        {
            //if(SCICtlVal->SCI_WSave == SCI_W_NOSAVE)
            {
                //Addr = SCI_Get_Addr(Rx_Func.all);
                memcpy(Addr, SCICtlVal->Write, W_ByteCnt.all);
                SCICtlVal->TX_Ready = TX_READY_YES;     
            }
            /*else
            {
                
                SCI_Write_EEP(Rx_Func.all, SCICtlVal->Write, RegisterNum);		//存储
                //SCICtlVal->TX_Ready = TX_READY_WAIT;
                SCICtlVal->TX_Ready = TX_READY_YES;
            }*/
        }

	}

    if((Rx_Func.Byte.High == PE_FUN_GROUP && Rx_Func.Byte.Low == 0) || \
       (Rx_Func.Byte.High == P1_FUN_GROUP && Rx_Func.Byte.Low == 0x3f))
    {
        W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[8];
		W_Data.Byte.Low  = SCICtlVal->SCI_RX_BUF[9];
        ExceptionWrDeal(Rx_Func.all, &W_Data.all);	
        if(1 == Func.Key.ControlBit.PEPassword) 
        {
            SCICtlVal->FaultCode = SCI_NO_ERR;
        }
    }

	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
        memcpy(SCICtlVal->SCI_TX_BUF, SCICtlVal->SCI_RX_BUF, 8 + W_ByteCnt.all);
        SCICtlVal->SCI_TX_BUF[W_ByteCnt.all + 10] = 0x3E;
        SCICtlVal->SCI_TX_BUF[W_ByteCnt.all + 11] = 0x3E;
//      SCI_Check.all = CRC16(SCICtlVal->SCI_TX_BUF,6);/*校验字节*/
//		SCICtlVal->SCI_TX_BUF[6] = SCI_Check.Byte.Low;
//		SCICtlVal->SCI_TX_BUF[7] = SCI_Check.Byte.High;
      	SCICtlVal->TxByte_Cnt = W_ByteCnt.all + 12;
	}

    if(Rx_Func.all == 0x7800)
    {
        SnWriteFlag = 1;
    }
}
/******************************************************************************
**函 数 名：HarmoCore上位机07功能码函数
**描    述：对07功能码接收数据进行解析
**调    用：SCI_In_Limit()：数据限制
            SCI_Write_EEP()：修改数据函数
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
**输    出：SCI_ByteNum.TX：发送字节个数
**返    回：无
**其    它：无
**日    期：2025-9-2
*******************************************************************************/

void SCI_HarmoCore_Func_07(void)
{
	TYPE_WORD Rx_Func;
	TYPE_WORD W_Data;    
	TYPE_WORD W_ByteCnt; 
    uint16_t RegisterNum;
  	Uint16 * Addr;
    uint16_t i;

    W_ByteCnt.Byte.High= SCICtlVal->SCI_RX_BUF[6];  
    W_ByteCnt.Byte.Low = SCICtlVal->SCI_RX_BUF[7];
    RegisterNum = W_ByteCnt.all >> 1;

	Rx_Func.Byte.High= SCICtlVal->SCI_RX_BUF[4]; /*取寄存器的地址*/
	Rx_Func.Byte.Low = SCICtlVal->SCI_RX_BUF[5];

	/*//考虑地址+EEPROM_ADDR的情况
	if((SCICtlVal->SCI_RX_BUF[4] & EEPROM_ADDRAND) == EEPROM_ADDR)
	{
        SCICtlVal->SCI_WSave = SCI_W_SAVE;
		Rx_Func.Byte.High = Rx_Func.Byte.High - EEPROM_ADDR;
	}*/

	SCI_Addr_Limit(Rx_Func.all, RegisterNum);
    if(SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        //MQB获取写入寄存器的地址
        Addr = SCI_Get_Addr(Rx_Func.all);
    }

	if(SCICtlVal->FaultCode == SCI_NO_ERR)//如果没有错误
	{
		for(i = 0; i < RegisterNum; i++)
		{
			W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[8+i*2];
		    W_Data.Byte.Low = SCICtlVal->SCI_RX_BUF[9+i*2];

            if(Rx_Func.Byte.High <= PF_FUN_GROUP)
            {
			    SCICtlVal->FaultCode = SCI_Write_Limit(Rx_Func.all + i, &W_Data.all);
            }

            if(SCICtlVal->FaultCode == SCI_NO_ERR)
			{
                if(RegisterNum > 1)
                {
                    ExceptionWrDeal_10(Rx_Func.all + i, &W_Data.all);
                }
                else
                {
                    ExceptionWrDeal(Rx_Func.all + i, &W_Data.all);	
                }
                SCICtlVal->Write[i] = W_Data.all;//没有错误		
			}
            else {
                break;
            }

		}
        if(SCICtlVal->FaultCode == SCI_NO_ERR)
        {
            //if(SCICtlVal->SCI_WSave == SCI_W_NOSAVE)
            //{
            //    //Addr = SCI_Get_Addr(Rx_Func.all);
            //    memcpy(Addr, SCICtlVal->Write, W_ByteCnt.all);
            //    SCICtlVal->TX_Ready = TX_READY_YES;     
            //}
            //else
            {
                
                SCI_Write_EEP(Rx_Func.all, SCICtlVal->Write, RegisterNum);		//存储
                //SCICtlVal->TX_Ready = TX_READY_WAIT;
                SCICtlVal->TX_Ready = TX_READY_YES;
            }
        }

	}
    if((Rx_Func.Byte.High == PE_FUN_GROUP && Rx_Func.Byte.Low == 0) || \
       (Rx_Func.Byte.High == P1_FUN_GROUP && Rx_Func.Byte.Low == 0x3f))
    {
        W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[8];
		W_Data.Byte.Low  = SCICtlVal->SCI_RX_BUF[9];
        ExceptionWrDeal(Rx_Func.all, &W_Data.all);	
        if(1 == Func.Key.ControlBit.PEPassword) 
        {
            SCICtlVal->FaultCode = SCI_NO_ERR;
        }
    }

	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
        memcpy(SCICtlVal->SCI_TX_BUF, SCICtlVal->SCI_RX_BUF, 8 + W_ByteCnt.all);
        SCICtlVal->SCI_TX_BUF[W_ByteCnt.all + 10] = 0x3E;
        SCICtlVal->SCI_TX_BUF[W_ByteCnt.all + 11] = 0x3E;
//      SCI_Check.all = CRC16(SCICtlVal->SCI_TX_BUF,6);/*校验字节*/
//		SCICtlVal->SCI_TX_BUF[6] = SCI_Check.Byte.Low;
//		SCICtlVal->SCI_TX_BUF[7] = SCI_Check.Byte.High;
      	SCICtlVal->TxByte_Cnt = W_ByteCnt.all + 12;
	}
}
/******************************************************************************
**函 数 名：10功能码函数
**描    述：对10功能码接收数据进行解析
**调    用：SCI_In_Limit()：数据限制
            SCI_Write_EEP()：修改数据函数
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
**输    出：SCI_ByteNum.TX：发送字节个数
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Func_10(void)
{
	TYPE_WORD Rx_Func;
	TYPE_WORD W_Data;
	TYPE_WORD W_Cnt;
  	Uint16 * Addr;
    uint16_t i;

    W_Cnt.Byte.High= SCICtlVal->SCI_RX_BUF[4];   /*取寄存器数目*/
    W_Cnt.Byte.Low = SCICtlVal->SCI_RX_BUF[5];

	Rx_Func.Byte.High= SCICtlVal->SCI_RX_BUF[2]; /*取寄存器的地址*/
	Rx_Func.Byte.Low = SCICtlVal->SCI_RX_BUF[3];

	//考虑地址+EEPROM_ADDR的情况
	if((SCICtlVal->SCI_RX_BUF[2] & EEPROM_ADDRAND) == EEPROM_ADDR)
	{
        SCICtlVal->SCI_WSave = SCI_W_SAVE;
		Rx_Func.Byte.High = Rx_Func.Byte.High - EEPROM_ADDR;
	}
    
	SCI_Addr_Limit(Rx_Func.all,W_Cnt.all);//地址限定

	if(SCICtlVal->FaultCode == SCI_NO_ERR)//如果没有错误
	{
		for(i = 0; i < W_Cnt.all; i++)
		{
			W_Data.Byte.High = SCICtlVal->SCI_RX_BUF[7+i*2];
		    W_Data.Byte.Low = SCICtlVal->SCI_RX_BUF[8+i*2];

            if(Rx_Func.Byte.High <= PF_FUN_GROUP)
            {
			    SCICtlVal->FaultCode = SCI_Write_Limit(Rx_Func.all + i, &W_Data.all);
            }
            
			if(SCICtlVal->FaultCode == SCI_NO_ERR)
			{
				ExceptionWrDeal_10(Rx_Func.all + i, &W_Data.all); 
				SCICtlVal->Write[i] = W_Data.all;//没有错误
			}
			else
			{
				break;
			}
		}
	}
    
	if(SCICtlVal->FaultCode == SCI_NO_ERR)
	{
        if(SCICtlVal->SCI_WSave == SCI_W_NOSAVE)
        {
            Addr = SCI_Get_Addr(Rx_Func.all);
            memcpy(Addr,SCICtlVal->Write,(W_Cnt.all << 1));
            SCICtlVal->TX_Ready = TX_READY_YES;     
        }
        else
        {
            SCI_Write_EEP(Rx_Func.all,SCICtlVal->Write,W_Cnt.all);		//存储
            //SCICtlVal->TX_Ready = TX_READY_WAIT;
            SCICtlVal->TX_Ready = TX_READY_YES;
        }
        
        memcpy(SCICtlVal->SCI_TX_BUF,SCICtlVal->SCI_RX_BUF,6);
//      SCI_Check.all = CRC16(SCICtlVal->SCI_TX_BUF,6);/*校验字节*/
//		SCICtlVal->SCI_TX_BUF[6] = SCI_Check.Byte.Low;
//		SCICtlVal->SCI_TX_BUF[7] = SCI_Check.Byte.High;
      	SCICtlVal->TxByte_Cnt = 8;
	}
}

/******************************************************************************
**函 数 名：08功能码函数
**描    述：对08功能码接收数据进行解析
**调    用：无
**输    入：SCI_RX_BUF：接收数据
            SCI_TX_BUF：发送数据数组
**输    出：SCI_ByteNum.TX: 发送数据帧字节个数
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Func_08(void)
{
	SCICtlVal->TxByte_Cnt = 8;
	memcpy(SCICtlVal->SCI_TX_BUF,SCICtlVal->SCI_RX_BUF,SCICtlVal->TxByte_Cnt);
	SCICtlVal->TX_Ready = TX_READY_YES;
}

void SCI_Func_20(void)
{
    memcpy(SCICtlVal->SCI_TX_BUF,SCICtlVal->SCI_RX_BUF,20);

    if(SCICtlVal->SCI_RX_BUF[6]  == OtaNameHeader[0]  && \
       SCICtlVal->SCI_RX_BUF[7]  == OtaNameHeader[1]  && \
       SCICtlVal->SCI_RX_BUF[8]  == OtaNameHeader[2]  && \
       SCICtlVal->SCI_RX_BUF[9]  == OtaNameHeader[3]  && \
       SCICtlVal->SCI_RX_BUF[10] == OtaNameHeader[4]  && \
       SCICtlVal->SCI_RX_BUF[11] == OtaNameHeader[5]  && \
       SCICtlVal->SCI_RX_BUF[12] == OtaNameHeader[6]  && \
       ((SCICtlVal->SCI_RX_BUF[13] == OtaNameHeader[7]) || (SCICtlVal->SCI_RX_BUF[13] == 0x30)) && \
       ((SCICtlVal->SCI_RX_BUF[14] == OtaNameHeader[8]) || (SCICtlVal->SCI_RX_BUF[14] == 0x30)) && \
       SCICtlVal->SCI_RX_BUF[15] == OtaNameHeader[9]  && \
       SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        if (StateMachine.RegulFlg == 0) 
        {
            SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
            UartBootFlag = 1;
            SCICtlVal->TxByte_Cnt = 20;
        }
        else {
            SCICtlVal->SCI_TX_BUF[5] = ERR_Regulator_Enable;    
            SCICtlVal->TX_Ready = TX_READY_YES; 
            SCICtlVal->TxByte_Cnt = 20;
        }
    }
    else
    {
        SCICtlVal->SCI_TX_BUF[5] = ERR_CRC_OR_FirmData;    
        SCICtlVal->TX_Ready = TX_READY_YES; 
        SCICtlVal->TxByte_Cnt = 20;
    }
}

void SCI_Func_30(void)
{
    SCICtlVal->SCI_TX_BUF[0] = 0x3C;
    SCICtlVal->SCI_TX_BUF[1] = 0x3C;
    SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->SCI_RX_BUF[2]; 
    SCICtlVal->SCI_TX_BUF[3] = 0x30; 
    SCICtlVal->SCI_TX_BUF[4] = 0x00; 
    SCICtlVal->SCI_TX_BUF[6] = SCICtlVal->SCI_RX_BUF[6]; 
    SCICtlVal->SCI_RX_BUF[7] = 0x00;
    SCICtlVal->SCI_TX_BUF[10] = 0x3E; 
    SCICtlVal->SCI_TX_BUF[11] = 0x3E;

    if((0x3C == SCICtlVal->SCI_RX_BUF[0]) && (0x3C == SCICtlVal->SCI_RX_BUF[1]) && \
       (0x30 == SCICtlVal->SCI_RX_BUF[3]) && (0x00 == SCICtlVal->SCI_RX_BUF[4]) && \
       (0x00 == SCICtlVal->SCI_RX_BUF[5]) && (0x00 == SCICtlVal->SCI_RX_BUF[7]) && \
       (0x3E == SCICtlVal->SCI_RX_BUF[SCICtlVal->RxByte_Cnt - 2]) && \
       (0x3E == SCICtlVal->SCI_RX_BUF[SCICtlVal->RxByte_Cnt - 1]) && \
       SCICtlVal->FaultCode == SCI_NO_ERR)
    {                
        if(0x01 == SCICtlVal->SCI_RX_BUF[6] || 0x04 == SCICtlVal->SCI_RX_BUF[6] || 0x08 == SCICtlVal->SCI_RX_BUF[6]) 
        {
            SCICtlVal->SCI_TX_BUF[5] = NO_ERR; 
            UartBootRxEndFlag = 1;
        }
        else 
        {
            SCICtlVal->SCI_TX_BUF[5] = ERR_FirmDataNum;       
            SCICtlVal->TX_Ready = TX_READY_YES;
            SCICtlVal->TxByte_Cnt = 12;
            //UartBootRxEndFlag = 3;  
        } 
    }
    else
    {
        SCICtlVal->SCI_TX_BUF[5] = ERR_CRC_OR_FirmData;
        SCICtlVal->TX_Ready = TX_READY_YES;
        SCICtlVal->TxByte_Cnt = 12; 
        //UartBootRxEndFlag = 3;
    }
}

void SCI_Func_40(void)
{
    SCICtlVal->SCI_TX_BUF[0]  = 0x3C;
    SCICtlVal->SCI_TX_BUF[1]  = 0x3C;
    SCICtlVal->SCI_TX_BUF[2]  = SCICtlVal->SCI_RX_BUF[2];
    SCICtlVal->SCI_TX_BUF[3]  = 0x40;
    SCICtlVal->SCI_TX_BUF[4]  = 0x00;
    SCICtlVal->SCI_TX_BUF[6]  = 0x00;
    SCICtlVal->SCI_TX_BUF[7]  = 0x02;
    SCICtlVal->SCI_TX_BUF[10] = 0x3E;
    SCICtlVal->SCI_TX_BUF[11] = 0x3E;

    if(SCICtlVal->SCI_RX_BUF[0] == 0x3C && SCICtlVal->SCI_RX_BUF[1] == 0x3C && \
       SCICtlVal->SCI_RX_BUF[3] == 0x40 && SCICtlVal->SCI_RX_BUF[4] == 0x00 && \
       SCICtlVal->SCI_RX_BUF[5] == 0x00 && SCICtlVal->SCI_RX_BUF[6] == 0x00 && \
       SCICtlVal->SCI_RX_BUF[7] == 0x02 &&
       SCICtlVal->SCI_RX_BUF[10] == 0x3E && SCICtlVal->SCI_RX_BUF[11] == 0x3E && \
       SCICtlVal->FaultCode == SCI_NO_ERR)
    {
        SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
        UartBootRxEndFlag = 2;
    }
    else
    {
        SCICtlVal->SCI_TX_BUF[5] = ERR_CRC_OR_FirmData;   
        SCICtlVal->TX_Ready = TX_READY_YES; 
        SCICtlVal->TxByte_Cnt = 12;
    }
}
/******************************************************************************
**函 数 名：void SCI_Analysis(void)
**描    述：帧分析函数
**调    用：无
**
**输    出：
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
//TYPE_DWORD BootDataCnt; //MQB固件升级总字节数
void SCI_Analysis(void)
{
	uint16_t SCI_Func_Code;
	SCI_Func_Code = SCICtlVal->SCI_RX_BUF[FunctiCode];
	if(SCICtlVal->RxEndFlag == RX_DATA_END)
	{
		memset(SCICtlVal->SCI_TX_BUF,0,SCI_FRAM_SIZE);

		switch(SCI_Func_Code)
		{
			case SCI_CMD_03:
				/*读变频器参数*/
				SCI_Func_03();
				break;
			case SCI_CMD_08:
				/*改写变频器多个功能代码或控制参数*/
		        SCI_Func_08();
				break;
            #if SERVO_UpComputer == HarmoCore
			case SCI_CMD_06: 
		        SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
                SCI_HarmoCore_Func_06();
				break;
            case SCI_CMD_07: //mqb新上位机 断电保存
		        SCICtlVal->SCI_WSave = SCI_W_SAVE;//写保存
                SCI_HarmoCore_Func_07();
                break;
            
            //case SCI_CMD_20: //mqb开始
            //    memcpy(SCICtlVal->SCI_TX_BUF,SCICtlVal->SCI_RX_BUF,20);

            //    if(SCICtlVal->SCI_RX_BUF[6]  == OtaNameHeader[0]  && \
            //       SCICtlVal->SCI_RX_BUF[7]  == OtaNameHeader[1]  && \
            //       SCICtlVal->SCI_RX_BUF[8]  == OtaNameHeader[2]  && \
            //       SCICtlVal->SCI_RX_BUF[9]  == OtaNameHeader[3]  && \
            //       SCICtlVal->SCI_RX_BUF[10] == OtaNameHeader[4]  && \
            //       SCICtlVal->SCI_RX_BUF[11] == OtaNameHeader[5]  && \
            //       SCICtlVal->SCI_RX_BUF[12] == OtaNameHeader[6]  && \
            //       ((SCICtlVal->SCI_RX_BUF[13] == OtaNameHeader[7]) || (SCICtlVal->SCI_RX_BUF[13] == 0x30)) && \
            //       ((SCICtlVal->SCI_RX_BUF[14] == OtaNameHeader[8]) || (SCICtlVal->SCI_RX_BUF[14] == 0x30)) && \
            //       SCICtlVal->SCI_RX_BUF[15] == OtaNameHeader[9]  && \
            //       SCICtlVal->FaultCode == SCI_NO_ERR)
            //    {
            //        if (StateMachine.RegulFlg == 0) 
            //        {
            //            SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
            //            UartBootFlag = 1;
            //            SCICtlVal->TxByte_Cnt = 20;
            //        }
            //        else {
            //            SCICtlVal->SCI_TX_BUF[5] = ERR_Regulator_Enable;    
            //            SCICtlVal->TX_Ready = TX_READY_YES; 
            //            SCICtlVal->TxByte_Cnt = 20;
            //        }
            //    }
            //    else
            //    {
            //        SCICtlVal->SCI_TX_BUF[5] = ERR_CRC_OR_FirmData;    
            //        SCICtlVal->TX_Ready = TX_READY_YES; 
            //        SCICtlVal->TxByte_Cnt = 20;
            //    }
            //    break;
            //case SCI_CMD_40: //mqb固件升级结束
            //    SCICtlVal->SCI_TX_BUF[0]  = 0x3C;
            //    SCICtlVal->SCI_TX_BUF[1]  = 0x3C;
            //    SCICtlVal->SCI_TX_BUF[2]  = SCICtlVal->SCI_RX_BUF[2];
            //    SCICtlVal->SCI_TX_BUF[3]  = 0x40;
            //    SCICtlVal->SCI_TX_BUF[4]  = 0x00;
            //    SCICtlVal->SCI_TX_BUF[6]  = 0x00;
            //    SCICtlVal->SCI_TX_BUF[7]  = 0x02;
            //    SCICtlVal->SCI_TX_BUF[10] = 0x3E;
            //    SCICtlVal->SCI_TX_BUF[11] = 0x3E;

            //    if(SCICtlVal->SCI_RX_BUF[0] == 0x3C && SCICtlVal->SCI_RX_BUF[1] == 0x3C && \
            //       SCICtlVal->SCI_RX_BUF[3] == 0x40 && SCICtlVal->SCI_RX_BUF[4] == 0x00 && \
            //       SCICtlVal->SCI_RX_BUF[5] == 0x00 && SCICtlVal->SCI_RX_BUF[6] == 0x00 && \
            //       SCICtlVal->SCI_RX_BUF[7] == 0x02 &&
            //       SCICtlVal->SCI_RX_BUF[10] == 0x3E && SCICtlVal->SCI_RX_BUF[11] == 0x3E && \
            //       SCICtlVal->FaultCode == SCI_NO_ERR)
            //    {
            //        SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
            //        UartBootRxEndFlag = 2;
            //    }
            //    else
            //    {
            //        SCICtlVal->SCI_TX_BUF[5] = ERR_CRC_OR_FirmData;   
            //        SCICtlVal->TX_Ready = TX_READY_YES; 
            //        SCICtlVal->TxByte_Cnt = 12;
            //    }
            //    break;
            case SCI_CMD_64: //mqb新上位机 示波器
		        SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
                SCI_HarmoCore_Func_06();
                break;
            case SCI_CMD_6F: //mqb新上位机 示波器
		        SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
                SCI_HarmoCore_Func_06();
                break;
            #elif SERVO_UpComputer == ServoTool
			case SCI_CMD_06:
				/*写变频器单一功能代码或控制参数*/
				if(RamCommu->ModbusType == 0)	//+EEPROM_ADDR,存EEP
				{
					if((SCICtlVal->SCI_RX_BUF[2] & EEPROM_ADDRAND) == EEPROM_ADDR)
					{
						SCICtlVal->SCI_WSave = SCI_W_SAVE;//写保存
					}
					else
					{
						SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
					}
				}
				else	//不+EEPROM_ADDR,存EEP
				{
					if(SCICtlVal->SCI_RX_BUF[2] < EEPROM_ADDR)
					{
						SCICtlVal->SCI_WSave = SCI_W_SAVE;//写保存
					}
					else
					{
						SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
					}
				}

				if(RamCommu->NotEEP)//全不存
				{
					SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
				}
	            SCI_Func_06();
				break;
			case SCI_CMD_10:
				/*改写变频器多个功能代码或控制参数*/
				if(RamCommu->ModbusType == 0)	//+EEPROM_ADDR,存EEP
				{
					if((SCICtlVal->SCI_RX_BUF[2] & EEPROM_ADDRAND) == EEPROM_ADDR)
					{
						SCICtlVal->SCI_WSave = SCI_W_SAVE;//写保存
					}
					else
					{
						SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
					}
				}
				else	//不+EEPROM_ADDR,存EEP
				{
					if(SCICtlVal->SCI_RX_BUF[2] < EEPROM_ADDR)
					{
						SCICtlVal->SCI_WSave = SCI_W_SAVE;//写保存
					}
					else
					{
						SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
					}
				}

				if(RamCommu->NotEEP)//全不存
				{
					SCICtlVal->SCI_WSave = SCI_W_NOSAVE;//写不保存
				}
		        SCI_Func_10();
				break;

            case SCI_CMD_5A://开始
                if(SCICtlVal->SCI_RX_BUF[2] == 0x42 &&
                   SCICtlVal->SCI_RX_BUF[3] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[4] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[5] == 0x54 &&
                   SCICtlVal->SCI_RX_BUF[6] == 0x4C &&
                   SCICtlVal->SCI_RX_BUF[7] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[8] == 0x44 &&
                   SCICtlVal->SCI_RX_BUF[9] == 0x45 &&
                   SCICtlVal->SCI_RX_BUF[10] == 0x52)
                {
                    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
                    SCICtlVal->SCI_TX_BUF[1] = 0x5A;
                    SCICtlVal->SCI_TX_BUF[2] = 0x4F;
                    SCICtlVal->SCI_TX_BUF[3] = 0x4B;
                    
                    if(StateMachine.RegulFlg == 0) //2025.06.16增加 //MQB判断调节去是否关闭
                    {
                        SCICtlVal->SCI_TX_BUF[4] = 0x00;
                        UartBootFlag = 1;
                    }
                    else
                    {
                        SCICtlVal->SCI_TX_BUF[4] = 0x01;
                        UartBootFlag = 0;    
                    }
                    
                    //SCICtlVal->TX_Ready = TX_READY_YES; 
                    //SCICtlVal->TxByte_Cnt = 7;
                    
                    
                }
                else if(SCICtlVal->SCI_RX_BUF[2] == 0x42 &&
                   SCICtlVal->SCI_RX_BUF[3] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[4] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[5] == 0x54 &&
                   SCICtlVal->SCI_RX_BUF[6] == 0x4F &&
                   SCICtlVal->SCI_RX_BUF[7] == 0x4B )  //结束
                {
                    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
                    SCICtlVal->SCI_TX_BUF[1] = 0x5A;
                    SCICtlVal->SCI_TX_BUF[2] = 0x4F;
                    SCICtlVal->SCI_TX_BUF[3] = 0x4B;
                    SCICtlVal->SCI_TX_BUF[4] = 0x00;  
                    
                    //SCICtlVal->TX_Ready = TX_READY_YES; 
                    //SCICtlVal->TxByte_Cnt = 7;
                    
                    UartBootRxEndFlag = 2;
                }
                else
                {
                    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
                    SCICtlVal->SCI_TX_BUF[1] = 0x5A;
                    SCICtlVal->SCI_TX_BUF[2] = 0x4F;
                    SCICtlVal->SCI_TX_BUF[3] = 0x4B;
                    SCICtlVal->SCI_TX_BUF[4] = 0x01;  
                    
                    SCICtlVal->TX_Ready = TX_READY_YES; 
                    SCICtlVal->TxByte_Cnt = 7;
                }
                SCICtlVal->FaultCode = SCI_NO_ERR;
				break;
            case SCI_CMD_A5:
                if(SCICtlVal->FaultCode == SCI_NO_ERR)//如果没有错误  //代码数据
                {
                    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
                    SCICtlVal->SCI_TX_BUF[1] = SCICtlVal->SCI_RX_BUF[1]; 
                    SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->SCI_RX_BUF[2];
                    SCICtlVal->SCI_TX_BUF[3] = SCICtlVal->SCI_RX_BUF[3];
                    SCICtlVal->SCI_TX_BUF[4] = 0x00;
                    
                    UartBootRxEndFlag = 1;
                    
                    //SCICtlVal->TX_Ready = TX_READY_YES; 
                    //SCICtlVal->TxByte_Cnt = 7;
                }
                else
                {
                    SCICtlVal->SCI_TX_BUF[0] = SCICtlVal->SCI_RX_BUF[0];
                    SCICtlVal->SCI_TX_BUF[1] = SCICtlVal->SCI_RX_BUF[1];
                    SCICtlVal->SCI_TX_BUF[2] = SCICtlVal->SCI_RX_BUF[2];
                    SCICtlVal->SCI_TX_BUF[3] = SCICtlVal->SCI_RX_BUF[3];
                    SCICtlVal->SCI_TX_BUF[4] = 0x01; 
                    
                    SCICtlVal->TX_Ready = TX_READY_YES; 
                    SCICtlVal->TxByte_Cnt = 7;
                }
				break;
            #endif
			default:
        //        if(SCI_Func_Code == 0x30)
        //        {
        //            SCI_Func_30();
        //        }
        //        else
        //        {
				    //SCICtlVal->FaultCode = ERR_FUNC_INVALID;	//功能码错误
        //        }

                if(SCI_Func_Code == 0x20)
                {
                    SCI_Func_20();
                }
                else if(SCI_Func_Code == 0x30)
                {
                    SCI_Func_30();
                }
                else if(SCI_Func_Code == 0x40)
                {
                    SCI_Func_40();
                }
                else
                {
				    SCICtlVal->FaultCode= ERR_FUNC_INVALID;	//功能码错误
                }
				break;
		}

		if(SCICtlVal->FaultCode != SCI_NO_ERR)
		{
            #if  SERVO_UpComputer == HarmoCore
            SCI_HarmoCore_Fault_Func();
            #elif  SERVO_UpComputer == ServoTool
			SCI_Fault_Func();//故障数据组帧
            #endif
		}
		if(SCICtlVal->RxEndFlag == RX_DATA_END)
		{
			  /*清收完一帧数据标志*/
			SCICtlVal->RxEndFlag = RX_DATA_NOEND;
		}
	}
}
/******************************************************************************
**函 数 名：void Tx_Wait_Save(void)
**描    述：帧分析函数
**调    用：无
**
**输    出：
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void Tx_Wait_Save(void)
{
	if(SCICtlVal->TX_Ready == TX_READY_WAIT)//写等待状态
	{
		//必须等待数据写完成，才发生数据，同时如果写数据失败必须响应内存中正确的数据，这是和以前不一样的地方。
		if(IICFlag.bit.Task_Scia == TASK_IDLE)
		{
			if(SCICtlVal->SCI_RX_BUF[1] == SCI_CMD_06)
			{
				SCICtlVal->TX_Ready = TX_READY_YES;
			}
			else if(SCICtlVal->SCI_RX_BUF[1] == SCI_CMD_10)
			{
				SCICtlVal->TX_Ready = TX_READY_YES;
			}
		}
	}
}
/******************************************************************************
**函 数 名：SCI发送函数
**描    述：发送数据
**调    用：无
**输    入：SCI_TX_BUF：帧信息
            SCI_ByteNum.TX：帧信息的字节个数
			SCI_Flag.Ack：从机响应标志
			SCI_Flag.TX_Over：发完数据标志
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
static void SCI_TX_Data(void)
{
	//从机或者有数据发送//如果不为广播地址，则需响应
	if((SCICtlVal->TX_Ready == TX_READY_YES) && (SCICtlVal->TX_Ack == TX_ACK_YES))
	{
		switch(SCICtlVal->TxStatus)
		{
			case TX_READY_STATUS://发送准备状态
//				SCICtlVal->SCI_SelectToTx();//
			    SCI_SelectToTx();           //置为发送方式  2021.2.20
				SCICtlVal->TxTimeCnt = 1;   //定时器清0
				SCICtlVal->TxStatus = TX_START_STATUS;
				break;
			case TX_START_STATUS://发送开始
				if(SCICtlVal->TxTimeCnt >= SCICtlVal->AckDelay)//等待发送响应延时
				{
					SCICtlVal->TxTimeCnt = 1;//定时器清0 调度会额外+1周期，用此抵消
					SCICtlVal->SCI_TxData(SCICtlVal->SCI_TX_BUF, SCICtlVal->TxByte_Cnt); //开始发送数据
					SCICtlVal->TxStatus = TX_END_STATUS;//设置为发送结束状态
				}
				else
				{
					SCICtlVal->TxTimeCnt++;//时间累加
				}
				break;
			case TX_END_STATUS://发送结束判决
				if(SCICtlVal->SCI_GetBufTxNum() == 0)
				{
					SCICtlVal->TX_Ready = TX_READY_NO;
					SCICtlVal->TX_Ack = TX_ACK_NO;
					SCICtlVal->TxStatus = TX_READY_STATUS;
					SCI_SelectToRx();           //置为接收方式  2021.2.20
				}
				break;
			default:break;
		}
	}
}
/******************************************************************************
**函 数 名：void SCI_ErrCheck(void)
**描    述：错误帧解析函数
**调    用：无
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_ErrCheck(void)
{
	SCICtlVal->Sci_ErrCheck();
}

void SCI_CheckScopeStatus(void)
{
    if((ScopePara->SeriesEnable) == 1) 
    {
        SCICtlVal->TX_Ack = TX_ACK_NO;
    }
}
//static uint8_t BootStatus = 0;

volatile uint8_t BootStatus = 0;
uint32_t UartFlashOffsetAddress = 0x00000000;
volatile uint8_t resetcnt = 0;
void BootLoaderProcess(void)
{
    uint16_t retval = 0;

    if(BootStatus == 0)
    {
        DISABLE_GLOBAL_INTERRUPT();    //关总中断
        BootStatus = 1;
        DisableDog(); 
    }
    else if(BootStatus == 1)
    {
        retval = Foe_EraseBank1();   

        if(retval == 0)
        {
            BootStatus = 2; 
            #if SERVO_UpComputer == ServoTool
            SCICtlVal->SCI_TX_BUF[4] = 0x00;
            SCICtlVal->TxByte_Cnt = 7;
            #elif SERVO_UpComputer == HarmoCore 
            SCICtlVal->SCI_TX_BUF[5] = NO_ERR;    
            //SCICtlVal->TxByte_Cnt = 20;
            #endif
            SCICtlVal->TX_Ack = TX_ACK_YES;
            SCICtlVal->TX_Ready = TX_READY_YES; 
            UartFlashOffsetAddress = 0;
        }
        else
        {
            BootStatus = 6; 
            #if SERVO_UpComputer == ServoTool
            SCICtlVal->SCI_TX_BUF[4] = 0x01;
            SCICtlVal->TxByte_Cnt = 7;
            #elif SERVO_UpComputer == HarmoCore  
            SCICtlVal->SCI_TX_BUF[5] = ERR_Erase_Flash;    
            SCICtlVal->TxByte_Cnt = 14;
            #endif
            SCICtlVal->TX_Ack = TX_ACK_YES;
            SCICtlVal->TX_Ready = TX_READY_YES;             
        }
    }
    else if(BootStatus == 2)
    {
       if(UartBootRxEndFlag == 1)
       {
           #if SERVO_UpComputer == ServoTool
           if(SCICtlVal->RxByte_Cnt == 262)
           {
               //memcpy(UartBootBuff,&SCICtlVal->SCI_RX_BUF[4],256);
             
               //retval = foe_support_store_data(FOE_BANK1_ADDR + UartFlashOffsetAddress, (uint32_t *)&SCICtlVal->SCI_RX_BUF[4], (uint32_t)256);
               retval = Foe_Data_Write(&SCICtlVal->SCI_RX_BUF[4], 256, &UartFlashOffsetAddress);

               //UartFlashOffsetAddress = UartFlashOffsetAddress + 256;
           }
           else if(SCICtlVal->RxByte_Cnt == 1030)
           {
               //memcpy(UartBootBuff,&SCICtlVal->SCI_RX_BUF[4],256);
               //retval = foe_support_store_data(FOE_BANK1_ADDR + UartFlashOffsetAddress, (uint32_t *)&SCICtlVal->SCI_RX_BUF[4], (uint32_t)1024);
               retval = Foe_Data_Write(&SCICtlVal->SCI_RX_BUF[4], 1024, &UartFlashOffsetAddress);
           }
           else
           {
               retval = 1;
           }
           #elif SERVO_UpComputer == HarmoCore 
           if(SCICtlVal->RxByte_Cnt == 268)
           {
               //memcpy(UartBootBuff,&SCICtlVal->SCI_RX_BUF[4],256);
             
               //retval = foe_support_store_data(FOE_BANK1_ADDR + UartFlashOffsetAddress, (uint32_t *)&SCICtlVal->SCI_RX_BUF[4], (uint32_t)256);
               retval = Foe_Data_Write(&SCICtlVal->SCI_RX_BUF[8], 256, &UartFlashOffsetAddress);

               //UartFlashOffsetAddress = UartFlashOffsetAddress + 256;
           }
           else if(SCICtlVal->RxByte_Cnt == 1036)
           {
               //memcpy(UartBootBuff,&SCICtlVal->SCI_RX_BUF[4],256);
               //retval = foe_support_store_data(FOE_BANK1_ADDR + UartFlashOffsetAddress, (uint32_t *)&SCICtlVal->SCI_RX_BUF[4], (uint32_t)1024);
               retval = Foe_Data_Write(&SCICtlVal->SCI_RX_BUF[8], 1024, &UartFlashOffsetAddress);
           }
           else if(SCICtlVal->RxByte_Cnt == 2060)
           {
               //memcpy(UartBootBuff,&SCICtlVal->SCI_RX_BUF[4],256);
               //retval = foe_support_store_data(FOE_BANK1_ADDR + UartFlashOffsetAddress, (uint32_t *)&SCICtlVal->SCI_RX_BUF[4], (uint32_t)1024);
               retval = Foe_Data_Write(&SCICtlVal->SCI_RX_BUF[8], 2048, &UartFlashOffsetAddress);
           }
           else
           {
               retval = 1;
           }
           #endif
         
           UartBootRxEndFlag = 0;
         
           if(retval == 0)
           {
               #if SERVO_UpComputer == ServoTool
               SCICtlVal->SCI_TX_BUF[4] = 0x00;
               SCICtlVal->TxByte_Cnt = 7;
               #elif SERVO_UpComputer == HarmoCore  
               SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
               SCICtlVal->TxByte_Cnt = 12;
               #endif
               SCICtlVal->TX_Ack = TX_ACK_YES;
               SCICtlVal->TX_Ready = TX_READY_YES; 
           }
           else
           {
               BootStatus = 6; 
               #if SERVO_UpComputer == ServoTool
               SCICtlVal->SCI_TX_BUF[4] = 0x01;
               SCICtlVal->TxByte_Cnt = 7;
               #elif SERVO_UpComputer == HarmoCore  
               SCICtlVal->SCI_TX_BUF[5] = ERR_Write_Flash;    
               SCICtlVal->TxByte_Cnt = 12;
               #endif
               SCICtlVal->TX_Ack = TX_ACK_YES;
               SCICtlVal->TX_Ready = TX_READY_YES;
           }
       }
       else if(UartBootRxEndFlag == 2)
       {
           #if SERVO_UpComputer == ServoTool
           if(Foe_CheckSum(UartFlashOffsetAddress) == 1)
           {
               BootStatus = 3; 
           }
           #elif SERVO_UpComputer == HarmoCore
           if((Foe_CheckSum(UartFlashOffsetAddress) == 1))
           {
               BootStatus = 3; 
               SCICtlVal->BootDataCnt.all = 0;
           }
           #endif
           else
           {
               BootStatus = 6; 
               #if SERVO_UpComputer == ServoTool
               SCICtlVal->SCI_TX_BUF[4] = 0x01;
               SCICtlVal->TxByte_Cnt = 7;
               #elif SERVO_UpComputer == HarmoCore 
               SCICtlVal->BootDataCnt.all = 0;
               SCICtlVal->SCI_TX_BUF[5] = ERR_FOE_Check;     
               SCICtlVal->TxByte_Cnt = 12;
               #endif
               SCICtlVal->TX_Ack = TX_ACK_YES;
               SCICtlVal->TX_Ready = TX_READY_YES;
           }

           UartBootRxEndFlag = 0;
       }
       else
       {
           BootStatus = 2; 
       }
    }
    else if(BootStatus == 3)
    {
       retval = Foe_WriteBank1Flag();

       if(retval == 0) 
       {
           BootStatus = 4; 
           resetcnt = 0;
           #if SERVO_UpComputer == ServoTool
           SCICtlVal->SCI_TX_BUF[4] = 0x00;
           SCICtlVal->TxByte_Cnt = 7;
           #elif SERVO_UpComputer == HarmoCore  
           SCICtlVal->SCI_TX_BUF[5] = NO_ERR;
           SCICtlVal->TxByte_Cnt = 12;
           #endif
           SCICtlVal->TX_Ack = TX_ACK_YES;
           SCICtlVal->TX_Ready = TX_READY_YES; 
       }
       else
       {
           BootStatus = 6; 
           #if SERVO_UpComputer == ServoTool
           SCICtlVal->SCI_TX_BUF[4] = 0x01;
           SCICtlVal->TxByte_Cnt = 7;
           #elif SERVO_UpComputer == HarmoCore  
           SCICtlVal->SCI_TX_BUF[5] = ERR_Upgrade_Flag;    
           SCICtlVal->TxByte_Cnt = 12;
           #endif
           SCICtlVal->TX_Ack = TX_ACK_YES;
           SCICtlVal->TX_Ready = TX_READY_YES;
       }
    }
    else if(BootStatus == 4)
    {
        resetcnt++;
        if(resetcnt == 100)
        {
            foe_support_soc_reset();
        }
    }
   else
   {
   }
}

/******************************************************************************
**函 数 名：void SCI_Procuint16_tnt16 ch)
**描    述：通信处理函数 1ms执行一次，
**调    用：无
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void SCI_Process(uint8_t ch)
{
    if((IIcTask.Handle != TASK_FACSAVE )&& (IIcTask.Handle != TASK_DEFSAVE))//恢复出厂值和默认值时不能通信
    {
        SCI_RxInquire();
        SCI_InitCh(ch);  //通道初始化
        SCI_RX_Data();   //通信数据接收函数
        SCI_Frame();    //帧处理函数
        SCI_Analysis(); //命令码分析函数
        Tx_Wait_Save(); //i2c写数据等待
        SCI_CheckScopeStatus(); 
        if(UartBootFlag == 1)  
        {
            BootLoaderProcess();
        }
        SCI_TX_Data();  //数据发送函数
        SCI_ErrCheck(); //错误检测
        SCI_TxInquire();
    }
}



