/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "s_comm_init.h"
#include "string.h"
#include <stdint.h>
#include "SV_DataBase.h"
#include "SV_Sci.h"
#if defined(SENSORLESS_CANOPEN_BUILD)
#include "s_encode_init.h"
#include "SensorlessCanopen.h"
#include "SensorlessVofa.h"
#endif

uint16_t CrcCheck = 0xFFFF;
uint16_t CrcCheck1 = 0xFFFF;
uint16_t CrcCheck2 = 0xFFFF;
uint16_t CrcCheck3 = 0xFFFF;
uint16_t CrcCheck4 = 0xFFFF; //mqb新上位机
uint16_t CrcCheck5 = 0xFFFF; //mqb新上位机
uint16_t TxCrcCheck = 0xFFFF;

UART_Type *testUART;
#if SENSORLESS_VOFA_ENABLE
volatile SL_VOFA_DIAG g_sl_vofa = {1U,0U,0U,0U,0U,0U};
static uint32_t vofa_last_ms;
static uint16_t vofa_tx_active;

/* Main-loop service only. At most one 32-byte FIFO fill, no waits, no printf.
 * UART0 is dedicated to output in this build; RX cannot issue motor commands. */
void SensorlessVofa_Service(void)
{
    uint32_t now=g_sl_can_elapsed_ms, seq, motor, output, valid, drive, age, tick;
    uint8_t packet[SL_VOFA_FRAME_BYTES];
    unsigned i;
    if(!g_sl_vofa.initialized) return;
    if(vofa_tx_active) {
        if(!SCOPE_TXEND_FLAG) return; /* keep DE until final stop bit is out */
        SCI_SelectToRx();
        vofa_tx_active=0U;
    }
    /* Drop received bytes without parsing any legacy control/boot commands. */
    SCOPE_ResetFIFO;
    if((uint32_t)(now-vofa_last_ms)<10U) return;
    vofa_last_ms=now; /* no backlog bursts when the main loop is late */
    if(!SCOPE_TXEND_FLAG || SCOPE_TXFIF0_NUM) {
        g_sl_vofa.busy_skips++; return;
    }
    seq=g_sl_encoder.sequence;
    if(seq&1U) {g_sl_vofa.snapshot_skips++;return;}
    motor=g_sl_encoder.motor_raw; output=g_sl_encoder.output_raw;
    valid=g_sl_encoder.valid; drive=g_sl_encoder.drive_active;
    age=g_sl_encoder.age_ms; tick=g_sl_encoder.tick_ms;
    if(seq!=g_sl_encoder.sequence) {g_sl_vofa.snapshot_skips++;return;}
    /* A frozen encoder service must not keep claiming fresh positions. */
    {
        static uint32_t last_tick, changed_at;
        if(tick!=last_tick) {last_tick=tick; changed_at=now;}
        if(age>2U || (uint32_t)(now-changed_at)>20U) valid=0U;
    }
    SlVofa_Pack(packet,motor,output,valid,drive,g_sl_vofa.frames);
    SCI_SelectToTx();
    for(i=0;i<SL_VOFA_FRAME_BYTES;i++) SCOPE_TX_DATA=packet[i];
    vofa_tx_active=1U;
    g_sl_vofa.frames++;
}
#endif

Uart_INFO SCI_CtlVal=
{
    /*RxCnt = */0,
    /*TxCnt = */0,
    /*TxType = */0,
    /*RxSum = */0,
    /*TxSum = */0,
    /*RxBuffull = */0,
    {0},
    {0}
};

#if SERVO_MCU == HPM_6E00

//初始化485串口
void Scope_Init(uint8_t ch, uint16_t baud, uint8_t data_format)
{
    (void)ch;
    (void)baud;
    (void)data_format;

    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_UART0_TXD;
    HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_UART0_RXD;

    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_GPIO_A_09; 
    HPM_IOC->PAD[IOC_PAD_PA09].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
    HPM_GPIO0->OE[GPIO_DI_GPIOA].SET = 1 << 9;
    HPM_GPIO0->DO[GPIO_DI_GPIOA].SET = 1 << 9;

    clock_add_to_group(ModbusCLK, 0);
#if SENSORLESS_VOFA_ENABLE
    /* Dedicated UART0 telemetry: 80 MHz / (8 * 4) = 2.5 Mbaud. */
    clock_set_source_divider(ModbusCLK, clk_src_pll1_clk0, 10);
#endif
    testUART = ModbusSci;
    uart_config_t config = {0};
    uart_default_config(ModbusSci, &config);
#if SENSORLESS_VOFA_ENABLE
    config.baudrate = 2500000U;
#else
    config.baudrate = 115200U;
#endif
    config.dma_enable = (SENSORLESS_VOFA_ENABLE == 0); 
    config.src_freq_in_hz = clock_get_frequency(ModbusCLK);
    config.fifo_enable = true;
    config.rx_fifo_level = uart_fifo_16_bytes;
    config.tx_fifo_level = uart_fifo_16_bytes;

#if SENSORLESS_VOFA_ENABLE
    g_sl_vofa.initialized=0U;
    if(uart_init(ModbusSci, &config)!=status_success) {
        g_sl_vofa.init_errors++;
        SCI_SelectToRx();
        return;
    }
    g_sl_vofa.initialized=1U;
    vofa_last_ms=g_sl_can_elapsed_ms;
    vofa_tx_active=0U;
    SCI_SelectToRx();
    return; /* telemetry uses bounded FIFO writes, never DMA or legacy CRC */
#else
    uart_init(ModbusSci, &config);
#endif

    dmamux_config(HPM_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_HDMA, 0), HPM_DMA_SRC_UART0_TX, true);

    {
        uint32_t sys_addr = core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)&data_format);

        dma_handshake_config_t config;
        dma_default_handshake_config(HPM_HDMA, &config);
        config.ch_index = 0;
        config.dst = (uint32_t)&ModbusSci->THR;
        config.dst_fixed = true;
        config.src = sys_addr;
        config.src_fixed = false;
        config.data_width = DMA_TRANSFER_WIDTH_BYTE;
        config.size_in_byte = 8;

        dma_channel_config_t channel_config = {0};
        dma_default_channel_config(HPM_HDMA, &channel_config);
        if(true == config.dst_fixed)
        {
            channel_config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
            channel_config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
        }
        if(true == config.src_fixed)
        {
            channel_config.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
            channel_config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
        }
        channel_config.en_infiniteloop = config.en_infiniteloop;
        channel_config.interrupt_mask = config.interrupt_mask;
        channel_config.src_width = config.data_width;
        channel_config.dst_width = config.data_width;
        channel_config.src_addr = config.src;
        channel_config.dst_addr = config.dst;
        channel_config.size_in_byte = config.size_in_byte;
        channel_config.src_burst_size = 0;
        dma_setup_channel(HPM_HDMA, config.ch_index, &channel_config,  false);
    }

    HPM_GPIO0->DO[GPIO_DI_GPIOA].CLEAR = 1 << 9;

    return ;   
}

/******************************************************************************
**函 数 名：int32 UART_ChABufTx(Uint8 *pData, Uint32 Size)
**描    述：  发送数据函数，为上层调用函数，最大一次可以发送数据为50个字节
**调    用： UART_StartTx(); 启动发送配置
**输    入：Uint8 *pData 要发送数据的地址，Uint8 Size 发送数据的个数
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_BufTx(uint8_t *pData, uint16_t Size)
{		
    //如果发送的数据大于
    if( Size >= SCI_BUFF_SIZE )
    {
        Size = SCI_BUFF_SIZE;
    }
    memcpy(SCI_CtlVal.TxBuf,pData,Size);  //将数据拷贝到发送数据区
    //记录要发送数据的个数
    SCI_CtlVal.TxSum = Size;
    //发送数据第0个开始
    SCI_CtlVal.TxCnt =0;
    SCI_CtlVal.TxType = 1;
}

/******************************************************************************
**函 数 名：void UART_ChABufRx(Uint8 *pData, Uint8 size)
**描    述： 获取接收的数据
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_BufRx(uint8_t *pData, uint16_t size)
{
    memcpy(pData,SCI_CtlVal.RxBuf,size);
}

/******************************************************************************
**函 数 名：void UART_ChABufRx(Uint8 *pData, Uint8 size)
**描    述： 获取接收的数据的个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
uint16_t SCI_GetBufRxNum(void)
{
    return SCI_CtlVal.RxCnt;
}
/******************************************************************************
**函 数 名：Uint8 UART_ChAGetBufTxNum(void)
**描    述： 获取发送数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
uint16_t SCI_GetBufTxNum(void)
{
    if((SCOPE_TXEND_FLAG == 1) && (SCI_CtlVal.TxCnt == 0))
    {
        return 0;
    }
    else
    {
        return 1;
    }

    return 0;
}

uint16_t SCI_GetBufRxbufflag(void)
{
    uint16_t temp;
    
    temp = SCI_CtlVal.RxBuffull;
  
    return temp;
}

/******************************************************************************
**函 数 名：void UART_ChAClrRxBuf(void)
**描    述： 清空接收区数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_ClrRxBuf(void)
{
    memset(SCI_CtlVal.RxBuf,0,SCI_BUFF_SIZE);
    SCI_CtlVal.RxCnt = 0;
}
/******************************************************************************
**函 数 名：void UART_ChAClrRxBuf(void)
**描    述： 清空发送区数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_ClrTxBuf(void)
{
    memset(SCI_CtlVal.TxBuf,0,SCI_BUFF_SIZE);
    SCI_CtlVal.TxCnt = 0;
    SCI_CtlVal.TxSum =0;
}

/******************************************************************************
**函 数 名：void SCIA_TxInqure(void)
**描    述：SCI发送查询函数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_TxInquire(void)
{
    uint8_t FIFO_Available,i;
    uint8_t index;
    
    if(SCI_CtlVal.TxType != 1)
    {
            return;
    }

    FIFO_Available = FIFO_DEPTH - SCOPE_TXFIF0_NUM;

    //如果没有发送完数据，就继续发送

    if(FIFO_Available>0)
    {
        for(i=0;i < FIFO_Available;i++)
        {
            if(SCI_CtlVal.TxCnt < SCI_CtlVal.TxSum)
            {
                SCOPE_TX_DATA = SCI_CtlVal.TxBuf[SCI_CtlVal.TxCnt];
                
                if(SCI_CtlVal.TxCnt < SCI_CtlVal.TxSum - Frame_tail)
                {
                    index = (TxCrcCheck&0x00FF) ^ SCI_CtlVal.TxBuf[SCI_CtlVal.TxCnt];
		            TxCrcCheck = Crc16_table[index] ^ (TxCrcCheck >> 8);
                    
                    SCI_CtlVal.TxBuf[SCI_CtlVal.TxSum - CrcCheck_L] = TxCrcCheck & 0x00FF;
                    SCI_CtlVal.TxBuf[SCI_CtlVal.TxSum - CrcCheck_H] = TxCrcCheck >> 8;
                }
                
                SCI_CtlVal.TxCnt++;
            }
            else
            {
                SCI_CtlVal.TxCnt  = 0;
                SCI_CtlVal.TxSum = 0;
                SCI_CtlVal.TxType = 0;
                TxCrcCheck = 0xFFFF;
                break;		//当发送FIFO为空又没有数据发送时，直接跳出FOR循环
            }
        }
    }
}

uint8_t SCI_GetTxType(void)
{
    return SCI_CtlVal.TxType;
}

void SCI_ErrClear(void)
{
    uint32_t status;

    status = SCOPE_R_STUATS;

    if(status & 0x82)
    {
        SCOPE_ResetFIFO;
    }
}

void SCI_RxInquire(void)		//接收数据查询
{
    uint8_t ValidRxNum,i;
    uint8_t index;
    
    ValidRxNum = SCOPE_RXFIF0_NUM;	//获取FIFO已接收数据数

    if(ValidRxNum > 0)		//FIFO接收数据不为空
    {
        //接收数据处理
        for(i=0;i < ValidRxNum;i++)
        {
            if(SCI_CtlVal.RxCnt < SCI_BUFF_SIZE)
            {
                SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt] = (SCOPE_RX_DATA)&0x00FF;
            }
            else
            {
                SCI_CtlVal.RxCnt = 0;
                SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt] = (SCOPE_RX_DATA)&0x00FF;
                SCI_CtlVal.RxBuffull = 1;
            }
            //mqb新上位机 CRC校验
            #if SERVO_UpComputer == HarmoCore
            {
                index = (CrcCheck&0x00FF) ^ SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt];
    		    CrcCheck = Crc16_table[index] ^ (CrcCheck >> 8);
                CrcCheck5 = CrcCheck4;
                CrcCheck4 = CrcCheck3;
                CrcCheck3 = CrcCheck2;
                CrcCheck2 = CrcCheck1;
                CrcCheck1 = CrcCheck;
            }
            #elif SERVO_UpComputer == ServoTool
            index = (CrcCheck&0x00FF) ^ SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt];
		    CrcCheck = Crc16_table[index] ^ (CrcCheck >> 8);
            
            CrcCheck3 = CrcCheck2;
            CrcCheck2 = CrcCheck1;
            CrcCheck1 = CrcCheck;
            #endif
            
            SCI_CtlVal.RxCnt++;
        }
    }
}

void Scope_DmaSend(uint8_t* buff,uint16_t Len)
{
#if SENSORLESS_VOFA_ENABLE
    (void)buff; (void)Len;
    return;
#endif
    if(Len == 0) return;

    HPM_HDMA->CHCTRL[0].CTRL &= 0xFFFFFFFE;
    HPM_HDMA->CHCTRL[0].SRCADDR = (uint32_t)buff;
    HPM_HDMA->CHCTRL[0].TRANSIZE = Len;
    HPM_HDMA->CHCTRL[0].CTRL |= 0x1;
}
#else
uint8_t Virtual_Uart_TXData;
void Scope_Init(uint8_t ch, uint16_t baud, uint8_t data_format)
{
    (void)ch;
    (void)baud;
    (void)data_format;    
}

/******************************************************************************
**函 数 名：int32 UART_ChABufTx(Uint8 *pData, Uint32 Size)
**描    述：  发送数据函数，为上层调用函数，最大一次可以发送数据为50个字节
**调    用： UART_StartTx(); 启动发送配置
**输    入：Uint8 *pData 要发送数据的地址，Uint8 Size 发送数据的个数
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_BufTx(uint8_t *pData, uint16_t Size)
{		
    //如果发送的数据大于
    if( Size >= SCI_BUFF_SIZE )
    {
        Size = SCI_BUFF_SIZE;
    }
    memcpy(SCI_CtlVal.TxBuf,pData,Size);  //将数据拷贝到发送数据区
    //记录要发送数据的个数
    SCI_CtlVal.TxSum = Size;
    //发送数据第0个开始
    SCI_CtlVal.TxCnt =0;
    SCI_CtlVal.TxType = 1;
}

void SCI_ScopeTx(void)
{

}
/******************************************************************************
**函 数 名：void UART_ChABufRx(Uint8 *pData, Uint8 size)
**描    述： 获取接收的数据
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_BufRx(uint8_t *pData, uint16_t size)
{
    memcpy(pData,SCI_CtlVal.RxBuf,size);
}

/******************************************************************************
**函 数 名：void UART_ChABufRx(Uint8 *pData, Uint8 size)
**描    述： 获取接收的数据的个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
uint16_t SCI_GetBufRxNum(void)
{
    return SCI_CtlVal.RxCnt;
}
/******************************************************************************
**函 数 名：Uint8 UART_ChAGetBufTxNum(void)
**描    述： 获取发送数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
uint16_t SCI_GetBufTxNum(void)
{
    /*
    if((SCOPE_TXEND_FLAG == 1) && (SCI_CtlVal.TxCnt == 0))
    {
        return 0;
    }
    else
    {
        return 1;
    }
    */

    return 0;
}

uint16_t SCI_GetBufRxbufflag(void)
{
    uint16_t temp;
    
    temp = SCI_CtlVal.RxBuffull;
  
    return temp;
}

/******************************************************************************
**函 数 名：void UART_ChAClrRxBuf(void)
**描    述： 清空接收区数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_ClrRxBuf(void)
{
    memset(SCI_CtlVal.RxBuf,0,SCI_BUFF_SIZE);
    SCI_CtlVal.RxCnt = 0;
}
/******************************************************************************
**函 数 名：void UART_ChAClrRxBuf(void)
**描    述： 清空发送区数据个数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_ClrTxBuf(void)
{
    memset(SCI_CtlVal.TxBuf,0,SCI_BUFF_SIZE);
    SCI_CtlVal.TxCnt = 0;
    SCI_CtlVal.TxSum =0;
}

/******************************************************************************
**函 数 名：void SCIA_TxInqure(void)
**描    述：SCI发送查询函数
**调    用： 
**输    入：
**输    出：无
            
**返    回：无
**其    它：无
**日    期：2013-3—27
*******************************************************************************/
void SCI_TxInquire(void)
{
    /*
    uint8_t FIFO_Available,i;
    uint8_t index;
    
    if(SCI_CtlVal.TxType != 1)
    {
            return;
    }

    FIFO_Available = FIFO_DEPTH - SCOPE_TXFIF0_NUM;

    //如果没有发送完数据，就继续发送

    if(FIFO_Available>0)
    {
        for(i=0;i < FIFO_Available;i++)
        {
            if(SCI_CtlVal.TxCnt < SCI_CtlVal.TxSum)
            {
                SCOPE_TX_DATA = SCI_CtlVal.TxBuf[SCI_CtlVal.TxCnt];
                
                if(SCI_CtlVal.TxCnt < SCI_CtlVal.TxSum - 2)
                {
                    index = (TxCrcCheck&0x00FF) ^ SCI_CtlVal.TxBuf[SCI_CtlVal.TxCnt];
		            TxCrcCheck = crc_table[index] ^ (TxCrcCheck >> 8);
                    
                    SCI_CtlVal.TxBuf[SCI_CtlVal.TxSum - 2] = TxCrcCheck & 0x00FF;
                    SCI_CtlVal.TxBuf[SCI_CtlVal.TxSum - 1] = TxCrcCheck >> 8;
                }
                
                SCI_CtlVal.TxCnt++;
            }
            else
            {
                SCI_CtlVal.TxCnt  = 0;
                SCI_CtlVal.TxSum = 0;
                SCI_CtlVal.TxType = 0;
                TxCrcCheck = 0xFFFF;
                break;		//当发送FIFO为空又没有数据发送时，直接跳出FOR循环
            }
        }
    }
    */
}

/******************************************************************************
函数名：SCIA_TxScopeData(Uint8 ch)
*******************************************************************************/
void SCI_TxScopeData(void)
{

}

uint8_t SCI_GetTxType(void)
{
    return SCI_CtlVal.TxType;
}

void SCI_ErrClear(void)
{

}

void SCI_RxInquire(void)		//接收数据查询
{
/*
    uint8_t ValidRxNum,i;
    uint8_t index;
    
    ValidRxNum = SCOPE_RXFIF0_NUM;	//获取FIFO已接收数据数

    if(ValidRxNum > 0)		//FIFO接收数据不为空
    {
        //接收数据处理
        for(i=0;i < ValidRxNum;i++)
        {
            if(SCI_CtlVal.RxCnt < SCI_BUFF_SIZE)
            {
                SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt] = (SCOPE_RX_DATA)&0x00FF;
            }
            else
            {
                SCI_CtlVal.RxCnt = 0;
                SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt] = (SCOPE_RX_DATA)&0x00FF;
                SCI_CtlVal.RxBuffull = 1;
            }
            
            index = (CrcCheck&0x00FF) ^ SCI_CtlVal.RxBuf[SCI_CtlVal.RxCnt];
		    CrcCheck = crc_table[index] ^ (CrcCheck >> 8);
            
            CrcCheck3 = CrcCheck2;
            CrcCheck2 = CrcCheck1;
            CrcCheck1 = CrcCheck;
            
            SCI_CtlVal.RxCnt++;
        }
    }
*/
}

void Scope_DmaSend(uint8_t * buff,uint16_t Len)
{
    (void)buff;
    (void)Len;
}


void Modbus_Init(uint16_t baud,uint16_t data_format)
{
    (void)baud;
    (void)data_format;
}


uint8_t Modbus_RxInquire(uint8_t *data,uint16_t *crc,uint8_t *flag)
{
    (void)data;
    (void)crc;
    (void)flag;
/*
    uint8_t ValidRxNum,i;
    uint8_t index;
    uint8_t temp;
    static uint16_t Crc = 0xFFFF,Crc1 = 0,Crc2 = 0,Crc3 = 0;
    
    ValidRxNum = MODBUS_RXFIF0_NUM;	//获取FIFO已接收数据数
    
    if(*flag == 1) 
    {
        *flag = 0;
        Crc  = 0xFFFF;
        Crc1 = 0xFFFF;
        Crc2 = 0xFFFF;
        Crc3 = 0xFFFF;
    }

    if(ValidRxNum > 0)		        //FIFO接收数据不为空
    {
        //接收数据处理
        for(i=0;i < ValidRxNum; i++)
        {
            temp = (MODBUS_RX_DATA)&0x00FF;
            data[i] = temp;
            
            index = (Crc&0x00FF) ^ temp;
		    Crc = crc_table[index] ^ (Crc >> 8);
            
            Crc3 = Crc2;
            Crc2 = Crc1;
            Crc1 = Crc;
            
            *crc = Crc3;
        }
    }
*/
    
    return 0;
}

uint8_t Modbus_TxInquire(uint8_t *data,uint16_t txCnt,uint16_t Len)
{
    (void)data;
    (void)txCnt;
    (void)Len;

   /*
   uint8_t FIFO_Available,i;
   uint8_t index;
   uint8_t temp,TxCnt;
   static uint16_t TxCrc = 0xFFFF;
    
   FIFO_Available = FIFO_DEPTH - MODBUS_TXFIF0_NUM;
   TxCnt = txCnt;
    //如果没有发送完数据，就继续发送

    if(FIFO_Available > 0)
    {
        for(i=0;i < FIFO_Available; i++)
        {
            if(TxCnt < Len)
            {
                if(TxCnt < Len - 2)
                {
                    temp = data[i];
                    MODBUS_TX_DATA = temp;
                    
                    index = (TxCrc&0x00FF) ^ temp;
		            TxCrc = crc_table[index] ^ (TxCrc >> 8);
                }
                else
                {
                    MODBUS_TX_DATA = (TxCrc);
                    data[i] = TxCrc;
                    TxCrc = (TxCrc >> 8);
                    
                }
                
                TxCnt++;
            }
            else
            {
                TxCrc = 0xFFFF;
                break;		//当发送FIFO为空又没有数据发送时，直接跳出FOR循环
            }
        }
    }
    
    
    return (TxCnt - txCnt);

*/
return 0;

}

void Modbus_CheckErr(void)
{
    
}
#endif

