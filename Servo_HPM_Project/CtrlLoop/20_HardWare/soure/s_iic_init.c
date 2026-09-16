/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "s_iic_init.h"

I2CMSG  CurrentMsgPtr;
uint8_t I2cBusState = 0;

#if SERVO_MCU == HPM_6E00

#if   HARDWARE_VER_SEL == HARDWARE_VER_0
#define EEPROMIIC HPM_I2C1
#elif HARDWARE_VER_SEL == HARDWARE_VER_1
#define EEPROMIIC HPM_I2C2
#else
#define EEPROMIIC HPM_I2C1
#endif

void Init_i2c_pins(void)
{
    #if   HARDWARE_VER_SEL == HARDWARE_VER_0
    HPM_IOC->PAD[IOC_PAD_PA12].FUNC_CTL = IOC_PA12_FUNC_CTL_I2C1_SDA | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_I2C1_SCL | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PA12].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PA13].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    #elif HARDWARE_VER_SEL == HARDWARE_VER_1
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PD02_FUNC_CTL_I2C2_SCL | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PD03_FUNC_CTL_I2C2_SDA | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PD02].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PD03].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    #else
    HPM_IOC->PAD[IOC_PAD_PA12].FUNC_CTL = IOC_PA12_FUNC_CTL_I2C1_SDA | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_I2C1_SCL | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PA12].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PA13].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    #endif
}

/***********************************************************************************************************************
* Function Name: I2CA_Init
* Description  : This function initializes the RIIC0 Bus Interface.
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void I2CA_Init(void)
{
    i2c_config_t config;
    uint32_t freq;

    freq = board_init_i2c_clock(EEPROMIIC);
    Init_i2c_pins();

    HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PD04_FUNC_CTL_GPIO_D_04; 
    HPM_IOC->PAD[IOC_PAD_PD04].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);
    HPM_GPIO0->OE[GPIO_DI_GPIOD].SET = 1 << 4;
    HPM_GPIO0->DO[GPIO_DI_GPIOD].CLEAR = 1 << 4;
    
    config.i2c_mode = i2c_mode_normal;
    config.is_10bit_addressing = false;
    i2c_init_master(EEPROMIIC, freq, &config);
    clock_cpu_delay_ms(10);

    if (i2c_get_line_sda_status(EEPROMIIC) == false)
    {
        i2c_gen_reset_signal(EEPROMIIC, 9);
        clock_cpu_delay_ms(10);
    }

    if((i2c_get_line_sda_status(EEPROMIIC) == false))
    {
        clock_cpu_delay_ms(10);
        if((i2c_get_line_sda_status(EEPROMIIC) == false))
        {
            I2cBusState = 4;
        }
    }
}

/***********************************************************************************************************************
* Function Name: I2CA_WriteData
* Description  : 
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void I2CA_WriteData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size)
{
    uint16_t IIc_Address = 0;
    
    IIc_Address = I2cAddr * 2;
    CurrentMsgPtr.DeviceAddress = 0xA0;
    CurrentMsgPtr.WordAddress = IIc_Address;  //要写入的地址
    CurrentMsgPtr.MsgBuffer = (uint8_t *)pDate;  //要写的数据
    CurrentMsgPtr.NumOfBytes = size * 2;  //要写数据的大小
    CurrentMsgPtr.MsgTxStatus = 0;
    CurrentMsgPtr.MsgMode = IIC_BUS_WRITE_DATA;
    CurrentMsgPtr.TxCnt = 0;
    CurrentMsgPtr.NAckCnt = 0;

    I2cBusState = IIC_BUS_BUSY ;//IIC总线忙状态
}

uint8_t I2C_Write_Process(uint16_t Addres,uint8_t* Data,uint16_t Len)
{
   uint8_t *pdata;
   static uint8_t offset = 0;
   uint8_t i,Send_Status;

   Send_Status = CurrentMsgPtr.MsgTxStatus;

   pdata = Data;
   switch (Send_Status) 
   {
        case 0: 
            if (EEPROMIIC->STATUS & I2C_STATUS_BUSBUSY_MASK) 
            {
                break;
            }

            EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK; 

            EEPROMIIC->CMD = I2C_CMD_CLEAR_FIFO;
            EEPROMIIC->ADDR = I2C_ADDR_ADDR_SET(0x50);
            EEPROMIIC->CTRL = I2C_CTRL_PHASE_START_MASK
                | I2C_CTRL_PHASE_STOP_MASK
                | I2C_CTRL_PHASE_ADDR_MASK
                | I2C_CTRL_PHASE_DATA_MASK
                | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE)
        #ifdef I2C_CTRL_DATACNT_HIGH_MASK
                | I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(Len + 2) >> 8U)
        #endif
                | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(Len + 2));

            EEPROMIIC->DATA = Addres >> 8;
            EEPROMIIC->DATA = Addres;
            EEPROMIIC->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
            Send_Status = 1; 
            break;
        case 1: 
            if (!(EEPROMIIC->STATUS & I2C_STATUS_ADDRHIT_MASK)) 
            {
                //return status_i2c_no_addr_hit; 
            }
            else
            {
                EEPROMIIC->STATUS = I2C_STATUS_ADDRHIT_MASK; 
                offset = 0;
                Send_Status = 2;
            } 
            break;
        case 2: 
             i = 4;
            do
            {
                if (!(EEPROMIIC->STATUS & I2C_STATUS_FIFOFULL_MASK)) 
                {
                    EEPROMIIC->DATA = *(pdata + offset);
                    offset++;
                    if(offset >= Len)
                    {
                        Send_Status = 3;
                        break;
                    }

                } 
                else 
                {
                    break;
                }
                i--;
            }while(i);
            break;
        case 3: 
            if (!(EEPROMIIC->STATUS & I2C_STATUS_CMPL_MASK)) 
            {
                //return IIC_BUS_CONTINUE;
            }
            else
            {
                Send_Status = 0; 
                offset = 0;
                I2cBusState = IIC_BUS_IDLE;
                CurrentMsgPtr.MsgMode = IIC_IDLE;

                EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK; 
            }
            break;
        default:
            break;
    }

    CurrentMsgPtr.MsgTxStatus = Send_Status;

    return Send_Status;
}

/***********************************************************************************************************************
* Function Name: I2CA_ReadData
* Description  : 
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void I2CA_ReadData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size)
{ 
    uint16_t IIc_Address = 0;
    
    IIc_Address = I2cAddr * 2;
    CurrentMsgPtr.DeviceAddress = 0xA0;
    CurrentMsgPtr.WordAddress = IIc_Address;  //要写入的地址
    CurrentMsgPtr.MsgBuffer = (uint8_t *)pDate;  //要写的数据
    CurrentMsgPtr.NumOfBytes = size * 2;  //要写数据的大小
    CurrentMsgPtr.MsgRxStatus = 0;
    CurrentMsgPtr.MsgMode = IIC_BUS_READ_DATA;
    CurrentMsgPtr.RxCnt = 0;
    CurrentMsgPtr.NAckCnt = 0;

    I2cBusState = IIC_BUS_BUSY ;//IIC总线忙状态 
}

uint8_t I2C_Read_Process(uint16_t Addres,uint8_t* Data,uint16_t Len)
{
   uint8_t *pdata;
   static uint16_t offset = 0;
   static uint8_t LostCnt = 0;
   uint8_t i,Send_Status;

   Send_Status = CurrentMsgPtr.MsgRxStatus;
   pdata = (uint8_t *)Data;

   switch (Send_Status) 
   {
        case 0: 
            if (EEPROMIIC->STATUS & I2C_STATUS_BUSBUSY_MASK) 
            {
                break;
            }

            EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK; 

            EEPROMIIC->CMD = I2C_CMD_CLEAR_FIFO;
            EEPROMIIC->ADDR = I2C_ADDR_ADDR_SET(0x50);
            EEPROMIIC->CTRL = I2C_CTRL_PHASE_START_MASK
            | I2C_CTRL_PHASE_ADDR_MASK
            | I2C_CTRL_PHASE_DATA_MASK
            | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE)
    #ifdef I2C_CTRL_DATACNT_HIGH_MASK
            | I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(2) >> 8U)
    #endif
            | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(2));

            EEPROMIIC->DATA = Addres >> 8;
            EEPROMIIC->DATA = Addres;
            EEPROMIIC->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
            Send_Status = 1; 
            LostCnt = 0;
            break;

        case 1: 
            if (!(EEPROMIIC->STATUS & I2C_STATUS_ADDRHIT_MASK)) 
            {
                LostCnt++;
                if(LostCnt > 10)
                {
                    EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK;
                    EEPROMIIC->CTRL = I2C_CTRL_PHASE_STOP_MASK;
                    EEPROMIIC->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
                    LostCnt = 0;
                }
            }
            else
            {
                EEPROMIIC->STATUS = I2C_STATUS_ADDRHIT_MASK; 
                offset = 0;
                Send_Status = 3;
                LostCnt = 0;

                /* W1C, clear CMPL bit to avoid blocking the transmission */
                EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK;

                EEPROMIIC->CMD = I2C_CMD_CLEAR_FIFO;
                EEPROMIIC->CTRL = I2C_CTRL_PHASE_START_MASK
                | I2C_CTRL_PHASE_STOP_MASK
                | I2C_CTRL_PHASE_ADDR_MASK
                | I2C_CTRL_PHASE_DATA_MASK
                | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_READ)
                #ifdef I2C_CTRL_DATACNT_HIGH_MASK
                | I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(Len) >> 8U)
                #endif
                | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(Len));
                 EEPROMIIC->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
            } 
            break;
            case 2:
            if (!(EEPROMIIC->STATUS & I2C_STATUS_CMPL_MASK)) 
            {

            }
            else
            {
                /* W1C, clear CMPL bit to avoid blocking the transmission */
                EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK;

                EEPROMIIC->CMD = I2C_CMD_CLEAR_FIFO;
                EEPROMIIC->CTRL = I2C_CTRL_PHASE_START_MASK
                | I2C_CTRL_PHASE_STOP_MASK
                | I2C_CTRL_PHASE_ADDR_MASK
                | I2C_CTRL_PHASE_DATA_MASK
                | I2C_CTRL_DIR_SET(I2C_DIR_MASTER_READ)
                #ifdef I2C_CTRL_DATACNT_HIGH_MASK
                | I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(Len) >> 8U)
                #endif
                | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(Len));
                 EEPROMIIC->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;

                 Send_Status = 3;
            }
            break;

        case 3: 
            i = 4;
            do
            {
                if (!(EEPROMIIC->STATUS & I2C_STATUS_FIFOEMPTY_MASK)) 
                {
                    *(pdata + offset) = EEPROMIIC->DATA;
                    offset++;
                    if(offset >= Len)
                    {
                        Send_Status = 4;
                        break;
                    }
                    CurrentMsgPtr.RxCnt = offset;
                } 
                else 
                {
                    break;
                }
                i--;
            }while(i);
            break;
        case 4: 
            if (!(EEPROMIIC->STATUS & I2C_STATUS_CMPL_MASK)) 
            {
                //return IIC_BUS_CONTINUE;
            }
            else
            {
                Send_Status = 0; 
                offset = 0;
                I2cBusState = IIC_BUS_IDLE;
                CurrentMsgPtr.MsgMode = IIC_IDLE;

                EEPROMIIC->STATUS = I2C_STATUS_CMPL_MASK; 
            }
            break;
        default:
            break;
    }

    CurrentMsgPtr.MsgRxStatus = Send_Status;

    return Send_Status;
}

/***********************************************************************************************************************
* Function Name: eeprom_process
* Description  : 
* author       : MaQianBing
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void eeprom_process(void)
{
    if(CurrentMsgPtr.MsgMode == IIC_BUS_WRITE_DATA)
    {
        I2C_Write_Process(CurrentMsgPtr.WordAddress,CurrentMsgPtr.MsgBuffer,CurrentMsgPtr.NumOfBytes);
    }
    else if(CurrentMsgPtr.MsgMode == IIC_BUS_READ_DATA)
    {
        I2C_Read_Process(CurrentMsgPtr.WordAddress,CurrentMsgPtr.MsgBuffer,CurrentMsgPtr.NumOfBytes);
    } 
    else
    {

    } 
}

void I2c_RstState(void)
{

}
#else
void I2c_Delay4us(void);

/***********************************************************************************************************************
* Function Name: R_RIIC0_Create
* Description  : This function initializes the RIIC0 Bus Interface.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void I2CA_Init(void)
{

}

void I2CA_WriteData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size)
{
    (void)I2cAddr;  
    (void)pDate;
    (void)size; 

    uint16_t IIc_Address;
    
    IIc_Address = I2cAddr * 2;
    CurrentMsgPtr.DeviceAddress = 0xA0;
    CurrentMsgPtr.WordAddress = IIc_Address;
    CurrentMsgPtr.MsgBuffer = pDate;
    CurrentMsgPtr.NumOfBytes = size;//字节长度
    CurrentMsgPtr.MsgTxStatus = 0;
    CurrentMsgPtr.MsgMode = 0;
    CurrentMsgPtr.TxCnt = 0;
    CurrentMsgPtr.NAckCnt = 0;

    I2cBusState = IIC_BUS_BUSY;   //总线在忙
}

void I2CA_ReadData(uint16_t I2cAddr,uint16_t *pDate,uint16_t size)
{
    (void)I2cAddr;
    (void)pDate;
    (void)size; 
}

void I2c_RstState(void)
{

}

void I2c_Delay4us(void)
{
    uint8_t loop;
    
    for (loop = 0; loop < 100 ; loop++ )  
    {
        asm volatile("nop");  
    }
}

#endif
