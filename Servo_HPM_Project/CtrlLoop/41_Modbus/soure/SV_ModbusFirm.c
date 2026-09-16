#include "SV_ModbusFirm.h"
#include "s_comm_init.h"
#include "string.h"
#include "SV_FuncVar.h"//
#include "SV_ModbusInit.h"

#if SERVOTYPE == SERVO_MODBUS

ModbusVal_Type ModbusVal = {0};
extern uint16_t* const ModBusConTrolAddres[MODBUSMAXADDRES];
extern TYPE_AttRI_Modbus AttCode_Modbus[MODBUSMAXADDRES];

static void Modbus_Config_Init(void)
{
    static uint8_t Initflag = 0;
    
    if(Initflag == 0)
    {
        memset(&ModbusVal,0,sizeof(ModbusVal_Type));
        
        ModbusVal.Addr = RamCommu->Addrss1;
        
        if(RamCommu->Baud1 == 0)       //4800
        {
            ModbusVal.SCIDelay = 128 + 2;   // 8ms  
        }
        else if(RamCommu->Baud1 == 1)  //9600
        {
            ModbusVal.SCIDelay = 64 + 2;   // 4ms
        }
        else if(RamCommu->Baud1 == 2)  //19200
        {
            ModbusVal.SCIDelay = 32 + 2;   // 2ms
        }
        else if(RamCommu->Baud1 == 3)  //38400
        {
            ModbusVal.SCIDelay = 16 + 2;   // 1ms
        }
        else if(RamCommu->Baud1 == 4)  //57600
        {
            ModbusVal.SCIDelay = 16 + 2;   // 1ms
        }
        else if(RamCommu->Baud1 == 5)  //115200
        {
            ModbusVal.SCIDelay = 16 + 2;   // 1ms
        }
        else
        {
            ModbusVal.SCIDelay = 16 + 2;   // 1ms            
        }
 
        ModbusVal.TxDelay = RamCommu->AckDelay2 * 16;
        Initflag = 1;
    }
}

static void Modbus_RX_Data(void)
{
    uint8_t Num;
    
    Num = Modbus_RxInquire(&ModbusVal.RxData[ModbusVal.RxByteCnt],&ModbusVal.CrcData,&ModbusVal.CrcFlg);
    ModbusVal.RxByteCnt +=Num;
    
    if(ModbusVal.RxByteCnt == ModbusVal.LastRxByteCnt && ModbusVal.RxByteCnt != 0)
    {
        ModbusVal.RxEndCnt++;
        
        if(ModbusVal.RxEndCnt > ModbusVal.SCIDelay)
        {
            ModbusVal.CrcFlg = 1;
            ModbusVal.RxEndFlg = 1;
            ModbusVal.RxEndNum = ModbusVal.RxByteCnt;
            ModbusVal.RxEndCnt = 0;
            ModbusVal.RxByteCnt = 0;
            ModbusVal.LastRxByteCnt = 0;
            //ModbusVal.CrcData = 0;
            ModbusVal.Fault_Code = 0;  //新的一帧接收完毕，错误置零
        }
    }
    else
    {
        ModbusVal.LastRxByteCnt = ModbusVal.RxByteCnt;  
        ModbusVal.RxEndCnt = 0;
    }
}

static void Modbus_Frame(void)
{
    uint8_t Errcode = 0;
    uint16_t crc;
    
    uint16_t RxWord;
    
    if(ModbusVal.RxEndFlg == MODBUS_RX_END)
    {
 		//如果不是本机地址，也不是广播地址，则丢掉该数据帧
		if((ModbusVal.RxData[0] != ModbusVal.Addr) && (ModbusVal.RxData[0] != 0))
		{
            ModbusVal.RxEndFlg = 0;
            return;
		} 
        else if(ModbusVal.RxEndNum < 8)
        {
            Errcode = MODBUS_FRAME_ERR;
        }
        else
        {
            RxWord = ModbusVal.RxData[4];
            RxWord = (RxWord << 8) + ModbusVal.RxData[5];
            switch(ModbusVal.RxData[1])
            {
                case 0x03:
                    if((RxWord > MODBUS_MAX_LEN) || (ModbusVal.RxEndNum > 8))
                    {
                        Errcode = MODBUS_FRAME_ERR;                    
                    }
                break;
                case 0x06:
                    if(ModbusVal.RxEndNum > 8)
                    {
                        Errcode = MODBUS_FRAME_ERR;
                    }
                break; 
                case 0x10:
                    if((RxWord > MODBUS_MAX_LEN) || (ModbusVal.RxEndNum != ((RxWord << 1) + 9)) )
                    {
                        Errcode = MODBUS_FRAME_ERR;
                    }
                break;
                default: 
                    Errcode = MODBUS_CODE_ERR;
                break;
            } 
        }
        
        if(Errcode == 0)
        {
            crc = ModbusVal.RxData[ModbusVal.RxEndNum-1];//校验高字节
            crc = (crc << 8) + ModbusVal.RxData[ModbusVal.RxEndNum-2];//校验低字节
            
            if(ModbusVal.CrcData == crc)
            {
                ModbusVal.Fault_Code = 0; 
                ModbusVal.Conflg = 1;
            }
            else
            { 
                ModbusVal.Fault_Code = MODBUS_CRC_ERR;
            }
        }
        else
        {
            ModbusVal.Fault_Code = Errcode;    
        }
    }
}

static uint8_t Addres_Limit(uint16_t Addres,uint16_t Len)
{
     if(Addres + Len > 0x1A5F)
     {
         return MODBUS_ADDRES_ERR; 
     }
     else if(Addres < 0x1A00)
     {
         return MODBUS_ADDRES_ERR; 
     }
     else
     {
         return 0;    
     }
}

static void Modbus_Func_03(void)
{
    uint16_t temp;
    uint16_t RxNum;
    uint16_t FuncAddres;
    uint8_t  Errcode = 0;
    uint16_t *FuncAddr;
    uint16_t i;
    uint16_t data;
    
    temp = ModbusVal.RxData[4];
    temp = (temp << 8) + ModbusVal.RxData[5];
    RxNum = temp;
    
    temp = ModbusVal.RxData[2];
    temp = (temp << 8) + ModbusVal.RxData[3];
    FuncAddres = temp;
    
    Errcode = Addres_Limit(FuncAddres,RxNum);
        
    if(Errcode == 0)
    {
        ModbusVal.TxData[0] = ModbusVal.RxData[0];
        ModbusVal.TxData[1] = ModbusVal.RxData[1];
        ModbusVal.TxData[2] = (RxNum << 1);
  
        for(i=0; i<RxNum; i++)
        {
            FuncAddr = ModBusConTrolAddres[FuncAddres - 0x1A00];
            
            data = *FuncAddr;
            ModbusVal.TxData[3 + i*2]= (data >> 8);
            ModbusVal.TxData[4 + i*2]= data&0x00FF;
            FuncAddres++;
        }
        
		/*发送字节个数*/
		ModbusVal.TxNum = (RxNum << 1) + 5;
        
        ModbusVal.TXReady = 1;
    }
    
    ModbusVal.Fault_Code = Errcode;
}

static uint8_t FuncValLimit(uint16_t Addres,uint16_t Data)
{
    TYPE_AttRI_Modbus temp;
    uint8_t Errcode;
    int16_t data;

    temp = AttCode_Modbus[Addres - 0x1A00];
    
    if(temp.RW == 0x00) //可写
    {
        if(temp.Type == 0) //无符号
        {
            if(Data > temp.Max || Data < temp.Min)
            {
                Errcode = MODBUS_LIM_ERR;
            }
            else
            {
                Errcode = 0;
            }            
        }
        else  //有符号
        {
            data = (int16_t)Data;
            
            if(data > (int16_t)temp.Max || data < (int16_t)temp.Min)
            {
                Errcode = MODBUS_LIM_ERR;
            }
            else
            {
                Errcode = 0;
            }
        }
    }
    else
    {
        Errcode = MODBUS_RW_ERR;
    }
    
    return Errcode;
}

static void Modbus_Func_06(void)
{
    uint16_t temp;
    uint16_t FuncAddres;
    uint8_t  Errcode = 0;
    uint16_t *FuncAddr;
    uint16_t data;
     
    temp = ModbusVal.RxData[2];
    temp = (temp << 8) + ModbusVal.RxData[3];
    FuncAddres = temp;
    
    Errcode = Addres_Limit(FuncAddres,1);
    
    if(Errcode == 0)
    {
        temp = ModbusVal.RxData[4];
        data = (temp << 8) + ModbusVal.RxData[5];
        
        Errcode = FuncValLimit(FuncAddres,data);  
        
        if(Errcode == 0)
        {
            FuncAddr = ModBusConTrolAddres[FuncAddres - 0x1A00];
            
            *FuncAddr = data;

            ModbusVal.TxData[0] = ModbusVal.RxData[0];
            ModbusVal.TxData[1] = ModbusVal.RxData[1];
            ModbusVal.TxData[2] = ModbusVal.RxData[2];
            ModbusVal.TxData[3] = ModbusVal.RxData[3];
            ModbusVal.TxData[4] = ModbusVal.RxData[4];
            ModbusVal.TxData[5] = ModbusVal.RxData[5];
            //ModbusVal.TxData[6] = ModbusVal.RxData[6];
            //ModbusVal.TxData[7] = ModbusVal.RxData[7];

            ModbusVal.TXReady = 1;
            
            /*发送字节个数*/
		    ModbusVal.TxNum = 8;
        } 
        else
        {
            ModbusVal.Fault_Code = Errcode;           
        }
    }
    else
    {
        ModbusVal.Fault_Code = Errcode;
    }
}

static void Modbus_Func_10(void)
{
    uint16_t RxNum;
    uint16_t temp;
    uint16_t FuncAddres;
    uint8_t  Errcode = 0;
    uint16_t *FuncAddr;
    uint16_t i;
     
    temp = ModbusVal.RxData[4];
    temp = (temp << 8) + ModbusVal.RxData[5];
    RxNum = temp;
    
    temp = ModbusVal.RxData[2];
    temp = (temp << 8) + ModbusVal.RxData[3];
    FuncAddres = temp;
    
    Errcode = Addres_Limit(FuncAddres,RxNum);
    
    if(Errcode == 0)
    {
        for(i=0; i<RxNum; i++)
        {
            temp = ModbusVal.RxData[7 + (i<<1)];
            ModbusVal.WriteWord[i] = (temp << 8) + ModbusVal.RxData[8 + (i<<1)];            
        }
    }
    else
    {
        ModbusVal.Fault_Code = MODBUS_ADDRES_ERR;
        return;
    }
    
    for(i=0; i<RxNum; i++)
    {
        Errcode += FuncValLimit(FuncAddres+i,ModbusVal.WriteWord[i]);        
    }
       
    if(Errcode == 0)
    {  
        for(i=0; i<RxNum; i++)
        {
            FuncAddr = ModBusConTrolAddres[FuncAddres - 0x1A00];
            *FuncAddr = ModbusVal.WriteWord[i];     
            FuncAddres++;
        }

        ModbusVal.TxData[0] = ModbusVal.RxData[0];
        ModbusVal.TxData[1] = ModbusVal.RxData[1];
        ModbusVal.TxData[2] = ModbusVal.RxData[2];
        ModbusVal.TxData[3] = ModbusVal.RxData[3];
        ModbusVal.TxData[4] = ModbusVal.RxData[4];
        ModbusVal.TxData[5] = ModbusVal.RxData[5];
        //ModbusVal.TxData[6] = ModbusVal.RxData[6];
        //ModbusVal.TxData[7] = ModbusVal.RxData[7];

        ModbusVal.TXReady = 1;
        /*发送字节个数*/
		ModbusVal.TxNum = 8;
    } 
    else
    {
        ModbusVal.Fault_Code = Errcode;        
    }
}

static void Modbus_Analysis(void)
{
    if(ModbusVal.RxEndFlg == MODBUS_RX_END)   
    {
        if(ModbusVal.Fault_Code == 0)
        {
            switch(ModbusVal.RxData[1])
            {
                case 0x03:
                    Modbus_Func_03();
                break;
                case 0x06:
                    Modbus_Func_06();
                break;
                case 0x10:
                    Modbus_Func_10();
                break;
                default: 
                    ModbusVal.Fault_Code = MODBUS_CODE_ERR;
                break;
            }          
        }
        
        if(ModbusVal.Fault_Code > 0)
        {
            ModbusVal.TXReady = 1;
            
            ModbusVal.TxData[0] = ModbusVal.Addr;
            ModbusVal.TxData[1] = 0x80;
            ModbusVal.TxData[2] = ModbusVal.Fault_Code;
            ModbusVal.TxNum = 5;  
        }
        
        if((ModbusVal.RxData[0] == 0x00) && (ModbusVal.TXReady == 1)) 
        {
            ModbusVal.TXReady = 0;
            ModbusVal.TxNum = 0;
        }
                
        ModbusVal.RxEndFlg = 0; 
    }
}

void Modbus_TXProcess(void)
{
    uint8_t Num;
    
    if(ModbusVal.TXReady == 1)
    {
		switch(ModbusVal.Tx_Status)
		{
			case 0://发送准备状态
                ModbusVal.TxTimeCnt++;
                if(ModbusVal.TxTimeCnt > ModbusVal.TxDelay)
                {
                    ModbusVal.Tx_Status = 1;
                    ModbusVal.TxTimeCnt = 0;
                    ModbusVal.TxCrc = 0xFFFF;
                    //PORTF.PODR.BIT.B7 = 1;   
                }
				break;
			case 1://发送开始
                Num = Modbus_TxInquire(&ModbusVal.TxData[ModbusVal.TxCnt],ModbusVal.TxCnt,ModbusVal.TxNum,&ModbusVal.TxCrc);
                ModbusVal.TxCnt += Num;
                
                if(ModbusVal.TxCnt >= ModbusVal.TxNum)
                {
                    ModbusVal.Tx_Status = 2;
                    ModbusVal.TxTimeCnt = 0;
                }
				break;
			case 2://发送结束判决
                if(MODBUS_TXEND_FLAG == 1)
                {
                    ModbusVal.TxTimeCnt++;
                    if(ModbusVal.TxTimeCnt >= 2)
                    {
                        ModbusVal.TxNum = 0;
                        ModbusVal.TxCnt = 0;
                        ModbusVal.TXReady = 0;
                        ModbusVal.Tx_Status = 0;
                        ModbusVal.TxTimeCnt = 0;
                    }
                }
				break;
			default:break;
		}        
    }
}

void Modbus_RXProcess(void)
{
    Modbus_RX_Data();
    Modbus_Frame();
    Modbus_Analysis();
    Modbus_CheckErr();
    Modbus_Config_Init();
}

#endif
