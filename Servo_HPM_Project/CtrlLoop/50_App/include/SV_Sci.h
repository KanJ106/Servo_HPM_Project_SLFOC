/*
 * SV_Sci.h
 *
 *  Created on: 2015-12-24
 *      Author: rd0217
 */
//
#ifndef SV_SCI_H_
#define SV_SCI_H_
//
#include "s_comm_init.h"
#include "SV_DataType.h"

#define     HarmoCore    1
#define     ServoTool    2
#define     SERVO_UpComputer  HarmoCore

#define CARD_ADDR         0X02   	//选件卡的地址，固定为2
#define SCI_FRAM_SIZE     SCI_BUFF_SIZE      //sci一帧的数据个数
#define SCI_FRAM_MINSIZE  7         //一帧数据的最小个数

#define RX_DATA_END 1               //接收数据完成
#define RX_DATA_NOEND 0             //接收数据没有完成

#define BROADCASTADDR 0             //广播地址

#if SCOPE_EN == 1
#if SERVO_UpComputer == HarmoCore
#define FRAME_03_READ_SIZE 32       //读的最大个数
#elif SERVO_UpComputer == ServoTool
#define FRAME_03_READ_SIZE 16       //读的最大个数
#define FRAME_10_WRITE_SIZE 16      //读的最大个数
#endif
#else
//#define FRAME_03_READ_SIZE  127     //读的最大个数   //2024.04.08  8-->100
#define FRAME_10_WRITE_SIZE 127       //读的最大个数   //2024.04.08  8-->100
#endif

#if SERVO_UpComputer == HarmoCore
#define FRAME_03_READ_SIZE  512      //读的最大个数   //2024.04.08  8-->100  //2025.09.26  100-->512 
#define FRAME_03_SIZE 12	         //读功能码帧的数据长度是8  
#define FRAME_06_WRITE_SIZE 512
#elif SERVO_UpComputer == ServoTool
#define FRAME_03_READ_SIZE  127     //读的最大个数   //2024.04.08  8-->100
#define FRAME_03_SIZE 8	            //读功能码帧的数据长度是8  
#endif
  
#define FRAME_08_SIZE 8	            //读功能码帧的数据长度是8
#define FRAME_06_SIZE 8	            //写单个字节数据长度是8
#define FRAME_41_SIZE 8	            //写单个字节不保存长度是8
#define FRARME_08_SIZE 8	        //线路检测数据帧长度是8


#define SCIMODELMODBUS	0			//当前选择Modbus协议
#define SCIMODELPROFIBUSDP	1		//当前选择Profibus-DP协议
#define SCIMODELCANOPEN	2			//当前选择CANopen协议
#define SCIMODELDEVICENET	3		//当前选择DeviceNet协议


#if SERVO_UpComputer == HarmoCore
#define MinByteNum       12 //最小字节数
#define Servo_addr       2  //驱动器地址
#define FunctiCode       3  //功能码
#define OpAddr_H         4  //操作地址高
#define OpAddr_L         5  //操作地址低
#define OpByteNum_H      6  //操作字节数高
#define OpByteNum_L      7  //操作字节数低
#define CrcCheck_L       4 
#define CrcCheck_H       3
#define Frame_tail       4 //帧尾以及CRC的位数
#elif SERVO_UpComputer == ServoTool
#define MinByteNum       8  //最小字节数
#define Servo_addr       0  //驱动器地址
#define FunctiCode       1  //功能码
#define OpAddr_H         2  //操作地址高
#define OpAddr_L         3  //操作地址低
#define OpByteNum_H      4  //操作字节数高
#define OpByteNum_L      5  //操作字节数低
#define CrcCheck_L       2 
#define CrcCheck_H       1
#define Frame_tail       2 //CRC的位数
#endif

typedef enum
{
	TX_ACK_YES = 0,	//需要相应数据帧
	TX_ACK_NO =1 	//接收到的是广播地址，不需要相应
}TYPE_TX_ACK;
typedef enum
{
	TX_READY_NO =0,
	TX_READY_YES =1,
	TX_READY_WAIT =2
}TYPE_TX_READY;
typedef enum
{
	SCI_W_NOSAVE =0,  	//不保存
 	SCI_W_SAVE =1,		//写保存
 	SCI_W_RESTORE =2, 	//参数恢复
 	SCI_W_CMD =3 		//写命令
}TYPE_SAVE_CMD;

typedef enum
{
	SCI_CMD_03 = 0x03,   //03功能码
 	SCI_CMD_06 = 0x06,   //06功能码
 	SCI_CMD_10 = 0x10,   //10功能码
 	SCI_CMD_17 = 0x17,   //17功能码
 	SCI_CMD_08 = 0x08,   //08功能码
 	SCI_CMD_41 = 0x41,   //41功能码
 	SCI_CMD_42 = 0x42,   //42功能码
    SCI_CMD_5A = 0x5A,   //5A功能码
 	SCI_CMD_6A = 0x6A,   //6a功能码
 	SCI_CMD_6B = 0x6B,   //6a功能码
 	SCI_CMD_ERR= 0x80,   //80功能码
    SCI_CMD_90 = 0x90,   //90功能码
    SCI_CMD_91 = 0x91,   //90功能码
    SCI_CMD_A5 = 0xA5,   //A5功能码

    SCI_CMD_07 = 0x07,   //mqb新上位机07功能码
    SCI_CMD_20 = 0x20,   //mqb新上位机20功能码
    SCI_CMD_30 = 0x30,   //mqb新上位机30功能码
    SCI_CMD_40 = 0x40,   //mqb新上位机40功能码
    SCI_CMD_64 = 0x64,   //mqb新上位机64功能码
    SCI_CMD_6F = 0x6F,   //mqb新上位机6F功能码
}TYPE_CMD_FUN;
typedef enum
{
	SCI_NO_ERR =0,
	ERR_FUNC_INVALID = 1,//非法功能码,接收到的功能码超出配置范围
	ERR_ADDR_INVALID = 2,//非法地址，接收到的地址是不允许的，寄存器的起始地址和传输长度组合是无效的
	ERR_FRAME_INVALID= 3,//非法数据帧，数据长度或CRC错误
	ERR_DEVICE_FAULT = 4,//设备故障错误试图执行请求操作时发送不可以恢复的差错。可能原因有逻辑错误，写eeprom失败
	ERR_DATA_RANGE   = 5,//数据超范围（最大值、最小值）
	ERR_PARA_READONLY= 6,//参数只读（参数只读或者密码保护等）
	ERR_RUN_INVALID  = 7,//参数运行中不可改
	ERR_PWD_PROTECT  = 8 //参数受密码保护
}TYPE_ENUM_SCI_ERR;
typedef enum
{
    NO_ERR               = 0,//  
    ERR_Regulator_Enable = 1,//错误码01：调节器未下使能
    ERR_CRC_OR_FirmData  = 2,//错误码02：固定数据部分或CRC校验错误
    ERR_FirmDataNum      = 3,//错误码03：数据帧有效字节数错误
    ERR_Erase_Flash      = 4,//错误码04：擦除flash失败
    ERR_Write_Flash      = 5,//错误码05：写入flash失败
    ERR_FOE_Check        = 6,//错误码06：FOE校验失败
    ERR_Upgrade_Flag     = 7 //错误码07：修改升级标志位失败
} TYPE_FirmUpgrade_ERR;  //mqb新上位机固件升级
#if 0
#define SEARCH_SIZE 80
typedef enum
{
	CUR_FUNC = 0x7401,	//当前功能代码
	DOWM_FUNC = 0x7410,	//下个功能代码
	UP_FUNC	 = 0x7411	//上个功能代码
}TYPE_17_CMD;
typedef enum
{
	SEARCH_IDLE = 0,//没有查找状态
	SEARCH_START =1,//查找开始
	SEARCH_UP_BUSY = 2,//地址码大于当前功能码查找
	SEARCH_DOWN_BUSY =3,//地址码小于当前功能码查找
	SEARCH_END =4//查找结束
}TYPE_SerchStatus;
typedef enum
{
	SEARCH_FUNC_IDLE = 0,//没有查找状态
	SEARCH_FUNC_UP = 1,//地址码大于当前功能码查找
	SEARCH_FUNC_DOWN =2//地址码小于当前功能码查找
}TYPE_SerchFunc;
#endif

typedef enum
{
	TX_READY_STATUS =0,//发送准备状态
	TX_START_STATUS =1,//发送开始
	TX_END_STATUS =2   //发送结束
}TYPE_TX_STATUS;

typedef enum
{
	RS485_LINK_CHECK =0,//连接检查状态
	RS485_TIMEOUT_CHECK =1//超时检测状态
}TYPE_SciaCheckStatus;
typedef struct
{
	uint16_t Ch;                    //sci通道数
	uint16_t CurRxByte_Cnt;         //sci当前接收字节个数
	uint16_t PreRxByte_Cnt;		    //SCI前一次接收数据的个数，
	uint16_t RxByte_Cnt;			//接收数据的个数
	uint16_t TxByte_Cnt;			//发送数据的字节个数

	uint16_t RxEndT_Cnt;            //SCI接收到最后一个数据到目前的时间
	uint16_t SCI_Delay;			    //SCI没有接收到数据的时间长度
	uint16_t SCI_Addr;
	uint16_t RxEndFlag;			    //一帧数据接收完成标志
	uint16_t FaultCode;			    //错误代码

	uint16_t TX_Ack;				//是否需要发送数据响应，广播地址时不需要发送，
	uint16_t TX_Ready;			    //数据发送准备好
	uint16_t TxTimeSpace;			//当作为主机发送数据时，需要数据发送完成后，间隔一段时间发送数据
	uint16_t Tx_DelayTime;  		//作为主机时发送间隔时间
	uint16_t SCI_WSave;

	uint16_t FunctionCode;		    //作为参数恢复用（写EEP时）
	uint16_t FuncAddr;
	uint16_t TimeoutCnt;			//通信超时计数
	uint16_t Timeout;	     		//通信超时时间
	uint16_t TxTimeCnt;     		//切换发送数据，需要等待1ms时间

	uint16_t AckDelay;
	uint16_t TxStatus;    		    //发送状态
	uint16_t I2cSize;
	uint16_t SciLinkCheck;

	uint16_t (*SCI_GetBufRxNum)(void);                      //获取接收数据个数函数指针
	uint16_t (*SCI_GetBufTxNum)(void);                      //获取当前发送数据个数指针
	void     (*SCI_ClrRxBuf)(void);                         //清除接收数据区
	void     (*SCI_ClrTxBuf)(void);                         //清除发送数据区
	void     (*SCI_RxData)(uint8_t *pData, uint16_t size);  //SCI接收数据函数
	void     (*SCI_TxData)(uint8_t *pData, uint16_t Size);  //SCI发送数据函数
	uint16_t (*SCI_GetBufRxbufflag)(void);
    void     (*Sci_ErrCheck)(void); 	                    //错误检查

	uint16_t I2cBuf[128];	  //写EEP缓存

	uint16_t *I2cRomFuncAddr;
	uint16_t *I2cRamFuncAddr;

	uint16_t Read[128];       //读数据数组
    uint16_t Write[128];	  //写数据数组

	uint8_t SCI_RX_BUF[SCI_FRAM_SIZE];			 //接收数据数组
    uint8_t SCI_TX_BUF[SCI_FRAM_SIZE];			 //发送数据数组
    uint16_t OpRegister;  
    TYPE_DWORD BootDataCnt;   //MQB固件升级总字节数
}SCICtlVal_Type;
#define SCIA_INIT {\
	/*Ch;            */SCI_CHA,\
	/*CurRxByte_Cnt; */0,\
	/*PreRxByte_Cnt; */0,\
	/*RxByte_Cnt;	 */0,\
	/*TxByte_Cnt;	 */0,\
	\
	/*RxEndT_Cnt;	 */0,\
	/*SCI_Delay;	 */4,\
	/*SCI_Addr;      */1,\
	/*RxEndFlag;	 */0,\
	/*FaultCode;	 */0,\
	\
	/*TX_Ack;        */0,\
	/*TX_Ready;      */TX_READY_NO,\
	/*TxTimeSpace;	 */0,\
	/*Tx_DelayTime;  */0,\
	/*SCI_WSave;	 */0,\
	\
	/*FunctionCode;	 */0,\
	/*FuncAddr       */0,\
	/*TimeoutCnt;	 */0,\
	/* Timeout      */0,\
	/*TxTimeCnt;  	 */1,\
	\
	/*AckDelay       */0,\
	/*TxStatus   	 */TX_READY_STATUS,\
	/*I2cSize 	     */0,\
	/*SciLinkCheck	 */RS485_LINK_CHECK,\
	\
		SCI_GetBufRxNum,\
		SCI_GetBufTxNum,\
		SCI_ClrRxBuf,\
		SCI_ClrTxBuf,\
		SCI_BufRx,\
		SCI_BufTx,\
		SCI_GetBufRxbufflag,\
        SCI_ErrClear\
	}
typedef struct
{
	uint8_t BaudRate;		//前次通信的波特率
	uint8_t DataFormat;		//前次通信的数据格式
	uint8_t SCIModeSel;		//前次通信的协议选择
	uint8_t CardRS232Baud;	//卡和232通信速率的选择
}TYPE_SCI_PreData;

extern SCICtlVal_Type SCICtlValA ;
extern SCICtlVal_Type *SCICtlVal;   //sci全局变量

void SCI_Process(uint8_t ch);

extern void PassProc(uint16_t Function);
//extern uint16_t crc_table[256];
extern volatile uint8_t UartBootFlag;
extern volatile uint8_t UartBootRxEndFlag;
#endif /* SV_SCI_H_ */
