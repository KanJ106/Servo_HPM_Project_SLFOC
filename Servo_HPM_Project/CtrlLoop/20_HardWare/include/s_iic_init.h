#ifndef RIIC_H
#define RIIC_H
#include "r_cg_Project.h"
#include "MCU_Fw.h"

typedef enum
{
    IIC_BUS_IDLE = 0,	//总线空闲状态
    IIC_BUS_RETRY =1,	//总线重写
    IIC_BUS_BUSY = 2,   //总线忙状态
    IIC_BUS_NACK = 3,	//总线错误状态
    IIC_BUS_INITERR = 4	//初始化错误
}IIC_BUS_STATUS;

typedef struct 
{
    uint8_t MsgTxStatus;	            //I2C发送状态
    uint8_t MsgRxStatus;	            //I2C发送状态
    uint8_t MsgMode;			        //I2C发送或接受
    uint8_t DeviceAddress;		        //EER设备地址
    uint8_t WordAddressH;	            //EEP字地址高8位
    uint8_t WordAddressL;	            //EER字地址低8位
    uint16_t WordAddress;	            //EEP字地址高8位
    uint16_t NumOfBytes;			    //I2C总读/写字节个数
    uint16_t TxCnt;                     //I2C当前已写个数
    uint16_t RxCnt;                     //I2C当前已读个数
    uint16_t NAckCnt;                   //NACK统计次数
    uint8_t *MsgBuffer;
}I2CMSG;


// EEPROM参数配置
#define EEPROM_PAGE_SIZE         16     // 每页大小
#define EEPROM_MAX_RETRY         500     // 最大重试次数
#define EEPROM_WRITE_DELAY_MS    5       // 写入等待时间

// 操作结果状态
typedef enum {
    EEPROM_OK = 0,
    EEPROM_BUSY,
    EEPROM_ERROR,
    EEPROM_TIMEOUT
} IIC_RESULT_STATE;

// EEPROM操作状态
typedef enum {
    IIC_IDLE,
    IIC_BUS_WRITE_DATA,
    IIC_BUS_READ_DATA
} IIC_opt_state;

//写函数状态机
typedef enum{
    INIT_SEND,
    CHECK_DEVICE,
    WRITE_ADDR_OR_DATA,
    CHECK_SEND_FINSH
} send_status;

//读函数状态机
typedef enum{
    CHECK_IIC_BUS,
    INIT_READ,
    CHECK_IIC_DEVICE,
    READ_IIC_DATA,
    CHECK_READ_FINSH
} read_status;


#if SERVO_MCU != NO_MCU

#else


#endif


/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void I2CA_Init(void);
void I2CA_WriteData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size);
void I2CA_ReadData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size);
void I2c_RstState(void);
void eeprom_process(void);
extern uint8_t I2cBusState;

#endif
