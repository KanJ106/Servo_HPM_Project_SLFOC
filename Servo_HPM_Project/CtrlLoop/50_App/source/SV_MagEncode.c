#include "s_panelkey_init.h"
#include "s_encode_init.h"
#include "SV_MagEncode.h"
#include "SV_FaultProtect.h"
#include "SV_FuncVar.h"
#include "DrvCoeff.h"
#include "Drive.h"
#include "SV_I2c.h"
#include "Pos_FullClosedLoop.h"

MAG_DataTYPE MagReg   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
MAG_DataTYPE MagFirst = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t BissCrcTable[256] = {0};

int32_t SECONDREVLNS = 0;        //(524288L)
int32_t SECONDZEROFFFSET = 0;    //(262144L)//
uint8_t  Shiftbit = 0;

#if SERVO_MCU == RZ_RZT1
#pragma optimize = none
#endif
void BissCrcTableGen(void)
{
    uint16_t i;
    uint8_t j;
    uint8_t crc1 = 0;
    uint8_t temp = 0;
    
    for(i = 0; i < 256;i++)
    {
        crc1 = 0;
        
        temp = (uint8_t)(i & 0xFC);
        crc1 = crc1 ^ temp;
        crc1 = crc1 & 0xFC;
        for(j = 0; j < 6; j++)
        {
            if(crc1&0x80)
            {
                crc1 = (crc1<<1) ^ 0x0C;
                crc1 = crc1 & 0xFC;
            }
            else
            {
                crc1 = crc1<<1;
            }
        }
        BissCrcTable[i] = crc1 & 0xFC;
    }    
}

Uint8 Crc6_S(Uint32 Value)
{ 
    Uint8  crc2 = 0;
    Uint8  temp = 0;
    
    temp = (Uint8)(Value >> 24);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(Value >> 18);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(Value >> 12);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(Value >> 6);
    crc2 = BissCrcTable[temp ^ crc2];
 
    crc2 = crc2 >> 2;
    crc2 = (~crc2)&0x3F; 
    
    return crc2;
}

//
Uint8 Crc6_M(Uint32 ValueH,Uint16 ValueL)
{ 
    Uint8  crc2 = 0;
    Uint8  temp = 0;
    
    temp = (Uint8)(ValueH >> 26);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueH >> 20);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueH >> 14);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueH >> 8);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueH >> 2);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueL >> 8);
    crc2 = BissCrcTable[temp ^ crc2];
    
    temp = (Uint8)(ValueL >> 2);
    crc2 = BissCrcTable[temp ^ crc2];
 
    crc2 = crc2 >> 2;
    crc2 = (~crc2)&0x3F; 
    
    return crc2;
}


void MagEncodePosReq_First(void)
{

}

void MagEncodePosReq_Second(void)
{   

}

void MagEncodePosRead_First(void)
{

}

void MagEncodePosRead_Second(void)
{

}

void delay_us(uint8_t us)
{
    volatile uint8_t i; 
    
    do
    {
        us--;
        for(i=0; i<25; i++);
    }while(us > 0);
}

void delay_ms(uint8_t ms)
{
    volatile uint16_t i; 
    
    do
    {
        ms--;
        for(i=0; i<25000; i++);
    }while(ms > 0);  
}

uint16_t IcmuRead_Register(uint8_t Addr)
{
(void)Addr;
	
}

uint16_t IcmuWrite_Register(uint8_t Addr,uint8_t Data)
{
(void)Addr;
(void)Data;
}

void EccodeBissPosZero_Second(void)
{
    int32 offset = 0;
    int32 pos1,pos2;
    int32_t temp;
    
    if(RamAid->AbsType == 2)
    {
        if(RamPD->ExEncType == 1)
        {
            if(FullCloseLoop.FullFlag == 0)
            {
                temp = DrvCoeff.MotEncSglRevLns * RamPD->Reduction;
                pos1 = PosFb.PosFbTotle;
                
                offset = (int16)RamServo->Mulpos1 * 65536L;
                offset = offset + RamServo->Mulpos0;      
                pos1 = pos1 + offset;
                pos2 = pos1 / temp;
                pos1 = pos1 - pos2 * temp;
                if(pos1 < 0) pos1 = pos1 + temp;

                RamServo->Mulpos0 = pos1&0x0000FFFF;
                RamServo->Mulpos1 = (pos1>>16)&0x0000FFFF;
                
                pos1 = MagReg.Pos;
                
                RamServo->Mulpos2 = pos1&0x0000FFFF;
                RamServo->Mulpos3 = (pos1>>16)&0x0000FFFF;
                
                IIcInterfaceB(&RamServo->Mulpos0,4);
                
                ExtPosFb.PosFbTotle = 0;
                PosFb.PosFbTotle = 0; //2025.04.02
            }
            else
            {
                offset = MagReg.PosOrig - SECONDZEROFFFSET;
                MagReg.PosOffset = offset;

                MagReg.Pos = SECONDZEROFFFSET;
                MagReg.PosLast = MagReg.Pos;

                temp = (Uint32)MagReg.PosOffset;
                RamServo->Mulpos2 = temp&0x0000FFFF;
                RamServo->Mulpos3 = (temp>>16)&0x0000FFFF;

                IIcInterfaceB(&RamServo->Mulpos2,2);

                ExtPosFb.PosFbTotle = MagReg.Pos - SECONDZEROFFFSET;
                ExtPosFb.EncSinglePosOld = MagReg.Pos;
                
                PosFb.PosFbTotle = 0; //2025.04.02
            }
        }
        else if(RamPD->ExEncType == 2)
        {
            if(FullCloseLoop.FullFlag == 0)
            {
                pos1 = PosFb.PosFbTotle;
                
                offset = (int16)RamServo->Mulpos1 * 65536L;
                offset = offset + RamServo->Mulpos0;      
                pos1 = pos1 + offset;

                RamServo->Mulpos0 = pos1&0x0000FFFF;
                RamServo->Mulpos1 = (pos1>>16)&0x0000FFFF;
                
                IIcInterfaceB(&RamServo->Mulpos0,2);
                
                offset = (int16)RamServo->Mulpos3 * 65536L;
                offset = offset + RamServo->Mulpos2; 
                pos1 = ExtPosFb.PosFbTotle + offset;
                
                RamServo->Mulpos2 = pos1&0x0000FFFF;
                RamServo->Mulpos3 = (pos1>>16)&0x0000FFFF;
                
                IIcInterfaceB(&RamServo->Mulpos0,4);
                
                ExtPosFb.PosFbTotle = 0;
                PosFb.PosFbTotle = 0; //2025.04.02           
            }
            else
            {
                MagReg.PosOffset = MagReg.PosOrig;
                MagReg.MuiltPosOffset = MagReg.MuiltPosOrig;
                  
                MagReg.Pos = 0;
                MagReg.PosLast = 0;

                temp = (Uint32)MagReg.PosOffset;
                RamServo->Mulpos2 = temp&0x0000FFFF;
                RamServo->Mulpos3 = (temp>>16)&0x0000FFFF;
                RamServo->Mulpos4 = MagReg.MuiltPosOffset;
                
                IIcInterfaceB(&RamServo->Mulpos2,3);

                ExtPosFb.PosFbTotle = 0;
                ExtPosFb.EncSinglePosOld = 0;   
                
                PosFb.PosFbTotle = 0; //2025.04.02
            }
        }
    }
    else
    {
        int32 offset = 0;
        Uint32 tempu;
        
        if(RamPD->ExEncType == 1)
        {
            offset = MagReg.PosOrig - SECONDZEROFFFSET;
            MagReg.PosOffset = offset;

            MagReg.Pos = SECONDZEROFFFSET;
            MagReg.PosLast = MagReg.Pos;

            tempu = (Uint32)MagReg.PosOffset;
            RamServo->Mulpos0 = tempu&0x0000FFFF;
            RamServo->Mulpos1 = (tempu>>16)&0x0000FFFF;

            IIcInterfaceB(&RamServo->Mulpos0,2);

            ExtPosFb.PosFbTotle = MagReg.Pos - SECONDZEROFFFSET;
            ExtPosFb.EncSinglePosOld = MagReg.Pos;
            
            PosFb.PosFbTotle = 0; //2025.04.02 
        }
        else if(RamPD->ExEncType == 2)
        {
            MagReg.PosOffset = MagReg.PosOrig;
            MagReg.MuiltPosOffset = MagReg.MuiltPosOrig;
              
            MagReg.Pos = 0;
            MagReg.PosLast = 0;

            tempu = (Uint32)MagReg.PosOffset;
            RamServo->Mulpos0 = tempu&0x0000FFFF;
            RamServo->Mulpos1 = (tempu>>16)&0x0000FFFF;
            RamServo->Mulpos2 = MagReg.MuiltPosOffset;
            
            IIcInterfaceB(&RamServo->Mulpos0,3);

            ExtPosFb.PosFbTotle = 0;
            ExtPosFb.EncSinglePosOld = 0;    
            
            PosFb.PosFbTotle = 0; //2025.04.02 
        }
        else
        {
            
            
        }
    }
}

void EccodeBissMuliZero_Second(void)
{
    MagReg.MuiltPosOffset = MagReg.MuiltPosOrig;   
    RamServo->Mulpos4 = MagReg.MuiltPosOffset;
    IIcInterfaceB(&RamServo->Mulpos4,1);
}

void EccodeBissPos_Alignment(void)
{
     int32 Pos_single,Pos_muli;
     float temp;
     int32 rem,div,muli;
     
     Pos_single = MagFirst.Pos;
     Pos_muli   = MagReg.PosOrig;
     
     div = SECONDREVLNS / RamPD->Reduction;
     muli = Pos_muli / div;
     rem = Pos_muli - div * muli;
     temp = (float)rem / (float)div;
     rem  = (int32)(temp * (float)DrvCoeff.MotEncSglRevLns);
     
     Pos_muli = rem - Pos_single;
     
     if(Pos_muli < 0) Pos_muli = Pos_muli + DrvCoeff.MotEncSglRevLns;
     
     
     Pos_muli = Pos_muli / RamPD->Reduction;
     
     
    RamServo->Abs0 = Pos_muli&0x0000FFFF;
    RamServo->Abs1 = (Pos_muli>>16)&0x0000FFFF; 
    
    IIcInterfaceB(&RamServo->Abs0,2); 
}

void Icmu_Init(void)
{
    uint8_t status1,status2,i;
    uint8_t Read_Value = 0;
    
    delay_ms(10);
    
    /************************上电读取IC_MU的状态寄存器***********/
    status1 = IcmuRead_Register(0x78);// 判断SPI通信
    
    if(status1 != 0x4D)
    {
        status1 = IcmuRead_Register(0x78);// 判断SPI通信 
        if(status1 != 0x4D)
        {
            status1 = IcmuRead_Register(0x78);// 判断SPI通信
            if(status1 != 0x4D)
            {
                FaultPrtt_FaultInterface(EncoderConfig_Err);
                return;
            }
        }
    }
    
    IcmuWrite_Register(0x75,0x07);//soft reset 
    delay_ms(100);
    delay_ms(100);
    
    i=0;
	/*************************I2C_RAM_START**************************/
	do
	{
		i++;
		if(i > 5)
		{
			break;
		}
		delay_us(50);
		IcmuWrite_Register(0x5C,0x61);
		delay_us(50);
		Read_Value = IcmuRead_Register(0x5C);	
	}while(Read_Value != 0x61);
	/*************************I2C_RAM_START**************************/
	
	i=0;
	
	/*************************I2C_RAM_END**************************/
	do
	{
		i++;
		if(i>5)
		{
			break;
		}
		delay_us(50);
		IcmuWrite_Register(0x5D,0x63);
		delay_us(50);
		Read_Value = IcmuRead_Register(0x5D);
	}while(Read_Value != 0x63);
	/*************************I2C_RAM_END**************************/
    
    /************************上电读取IC_PVL的状态寄存器***********/
	IcmuWrite_Register(0x5B,0x00);//读IC_PVL的寄存器地址
	delay_us(50);
	IcmuWrite_Register(0x5E,0xC1);//读取IC_PVL寄存器的指令
	delay_us(50);
	IcmuWrite_Register(0x75,0x0A);//I2C指令，开启读取IC_PVL的寄存器
	delay_ms(20);
	status1 = IcmuRead_Register(0x61);
	delay_us(50);
	status2 = IcmuRead_Register(0x63);
	/************************上电读取IC_PVL的状态寄存器***********/
    
    if(RamPD->ExEncType == 2)
    {
        if((status1 & 0x30) == 0x20)//WARN
        {
            FaultPrtt_FaultInterface(EncoderBa_Err);      
        }
        
        if(status1 & 0xD0)//Err
        {
            if(RamAid->AbsType == 2)
            {
                if(RamServo->Mulpos4 > 0)
                {
                    RamServo->Mulpos4 = 0;
                    IIcInterfaceB(&RamServo->Mulpos4,1);
                }
            }
            else
            {
                if(RamServo->Mulpos2 > 0)
                {
                    RamServo->Mulpos2 = 0;
                    IIcInterfaceB(&RamServo->Mulpos2,1);
                }                
            }
            
            /*
            if((RamMotor->Mulpos0 > 0) || (RamMotor->Mulpos1 > 0))
            {
                RamMotor->Mulpos0 = 0;
                RamMotor->Mulpos1 = 0;
                IIcInterfaceB(&RamMotor->Mulpos0,2); 
            }
            */
            
            FaultPrtt_FaultInterface(EncoderBe_Err);       
        }
        
        if((status1 & 0x0F) || (status2 & 0x7))
        {
            FaultPrtt_FaultInterface(EncoderErr_CE);     
        }
    }
    
    /************************上电清除IC_PVL的复位错误***********/
	IcmuWrite_Register(0x5B,0x01);//设置写入IC_PVL的寄存器地址0x01
	delay_us(50);
	IcmuWrite_Register(0x5E,0xC0);//0xC0表示该操作为写IC_PVL的寄存器
	delay_us(50);
	IcmuWrite_Register(0x61,0x05);//0x05写入IC_PVL后，会清除IC_PVL的警报
	delay_us(50);
	IcmuWrite_Register(0x75,0x0A);//开启写操作
	delay_ms(50);                 //等待清除完毕
	/************************上电清除IC_PVL的复位错误***********/
    
    IcmuWrite_Register(0x0B,0x0A);//BISS SET
    delay_ms(50); 
}

extern uint32_t g_rstsr0;
void EncodeBiss_Second(void)
{
    int32 temp;
    
    if(RamAid->AbsType == 2)
    {
        if(RamPD->FullCloseMode == 0)
        {
            temp = RamServo->Abs1 * 65536;
            MagReg.PosOffset = temp + RamServo->Abs0; //计算偏移
            
            MagReg.MuiltPosOffset = RamServo->Mulpos4;
        }
        else
        {
            temp = RamServo->Mulpos3 * 65536;
            MagReg.PosOffset = temp + RamServo->Mulpos2; //计算偏移
            MagReg.MuiltPosOffset = RamServo->Mulpos4;
        }
    }
    else if(RamAid->AbsType == 1)
    {
        temp = RamServo->Mulpos1 * 65536;
        MagReg.PosOffset = temp + RamServo->Mulpos0; //计算偏移
        MagReg.MuiltPosOffset = RamServo->Mulpos2;        
    }
    else
    {
        temp = RamServo->Mulpos1 * 65536;
        MagReg.PosOffset = temp + RamServo->Mulpos0; //计算偏移
        MagReg.MuiltPosOffset = RamServo->Mulpos2;           
    }
    
   
    if(RamPD->ExEncLine == 32768)
    {
        SECONDREVLNS = 131072L;
        SECONDZEROFFFSET = 65536;
        Shiftbit = 2;
    }
    else if(RamPD->ExEncLine == 131072)
    {
        SECONDREVLNS = 524288L;
        SECONDZEROFFFSET = 262144L;
        Shiftbit = 0;
    } 
    else
    {
        SECONDREVLNS = 524288L;
        SECONDZEROFFFSET = 262144L;
        Shiftbit = 0;    
    }
    
    BissCrcTableGen();
    /*
    if(RST_SOURCE_RES == g_rstsr0)// 复位引脚复位  上电复位
    {
        Spi_IO_Second();
        Icmu_Init();      
    }
    */
    Biss_IO_Second();
}

void EccodeBissPosCacl_First(void)
{
    MagEncodePosRead_First();
    MagEncodePosReq_First();  

    MagFirst.EncTxflg  = 1;  
}

void EccodeBissPosCacl_Second(void)
{
    MagEncodePosRead_Second();
    MagEncodePosReq_Second();
    //400/150 us    
}

int32 EncodePosInit_Second(void)
{
    int32_t temp;
    int32_t Out;
    int32_t Div;
    int32_t SingleDiv2;
    int32_t Offset;
    
    int16_t Muli;
    int32_t MuliRem;
    int32_t MuliToSingPos;
    
    int32_t First_Data,Second_Data;
    
    if(RamAid->AbsType == 2)
    {
        if(FullCloseLoop.FullFlag == 0)
        {
            if(RamPD->ExEncType == 1)
            {
                First_Data = MagFirst.Pos;
                Second_Data = MagReg.Pos;
            }
            else if(RamPD->ExEncType == 2)
            {
                First_Data = MagFirst.Pos;
                Second_Data = MagReg.MuiltPos * 131072 + MagReg.Pos;        
            }
            else
            {
                First_Data = 0;
                Second_Data = 0;        
            }
                
            Div = SECONDREVLNS / RamPD->Reduction;
            SingleDiv2 = DrvCoeff.MotEncSglRevLns / 2;
            
            Muli = (int64)Second_Data / Div;                                                //RamPD->Reduction / 131072L
            MuliRem = Second_Data - (int64)Muli * Div;                                      //131072L  / RamPD->Reduction 
            MuliToSingPos = (int32_t)(((float)MuliRem / Div) * DrvCoeff.MotEncSglRevLns);   //RamPD->Reduction / 131072.0f
            
            if(Second_Data >= 0)
            {
                temp = MuliToSingPos - First_Data;
                if(temp < SingleDiv2 && temp > -SingleDiv2)
                {
                    Out = Muli * DrvCoeff.MotEncSglRevLns + First_Data;
                }
                else if(temp >= SingleDiv2)
                {
                    Out = (Muli + 1) * DrvCoeff.MotEncSglRevLns + First_Data;
                }
                else
                {
                    Out = (Muli - 1) * DrvCoeff.MotEncSglRevLns + First_Data;
                }            
            }
            else
            {
                temp = MuliToSingPos - (First_Data - DrvCoeff.MotEncSglRevLns + 1);
                if(temp < SingleDiv2 && temp > -SingleDiv2)
                {
                    Out = Muli * DrvCoeff.MotEncSglRevLns + (First_Data - DrvCoeff.MotEncSglRevLns + 1);
                }
                else if(temp <= SingleDiv2)
                {
                    Out = (Muli - 1) * DrvCoeff.MotEncSglRevLns + (First_Data - DrvCoeff.MotEncSglRevLns + 1);
                }
                else
                {
                    Out = (Muli + 1) * DrvCoeff.MotEncSglRevLns + (First_Data - DrvCoeff.MotEncSglRevLns + 1);
                }
            }
            
            Offset = (int16)RamServo->Mulpos1 * 65536L;
            Offset = Offset + RamServo->Mulpos0;
            Out = Out - Offset;
            
            if(RamPD->ExEncType == 1)
            {           
                temp = (DrvCoeff.MotEncSglRevLns * RamPD->Reduction) >> 1;
                if(Out < -temp) Out = Out + DrvCoeff.MotEncSglRevLns * RamPD->Reduction; 
            }
        }
        else
        {
            temp = DrvCoeff.MotEncSglRevLns * RamPD->Reduction;
            Out = (int64)ExtPosFb.PosFbTotle * temp / SECONDREVLNS;
        }
    }
    else if(RamAid->AbsType == 1)
    {
        float tempf  = 0.0f;
        int32 temp1 = 0;
        int32 temp2 = 0;
        int32 SingPos;
    
        if(RamPD->ExEncType == 1)
        {
            SingPos = MagReg.Pos;
            tempf = (float)SECONDREVLNS;
            tempf = (SingPos - SECONDZEROFFFSET)/tempf;
            temp1 = DrvCoeff.MotEncSglRevLns*RamPD->Reduction;
            temp2 = (int32)(temp1*tempf);
        }
        else if(RamPD->ExEncType == 2)
        {
            SingPos = MagReg.MuiltPos * SECONDREVLNS + MagReg.Pos;
            tempf = (float)SECONDREVLNS;
            tempf = (float)SingPos / tempf;
            temp1 = DrvCoeff.MotEncSglRevLns*RamPD->Reduction;
            temp2 = (int32)(temp1*tempf);    
        }
        else
        {
            SingPos = MagReg.Pos;
            tempf = (float)SECONDREVLNS;
            tempf = (SingPos - SECONDZEROFFFSET)/tempf;
            temp1 = DrvCoeff.MotEncSglRevLns*RamPD->Reduction;
            temp2 = (int32)(temp1*tempf);
        } 
        
        Out = temp2;
    }
    else
    {
        Out = 0;
    }
    
    return (Out);
}
