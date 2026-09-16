#include "userdefine.h"
#include "SV_IncEncode.h"
#include "SV_FuncVar.h"
#include "Cur_Etheta.h"
#include "SV_FaultProtect.h"//
#include "SV_AppDriveInterface.h"
#include "r_cg_Project.h"

CUR_UVWETHETA Hall={0,0,0};

uint8_t GET_HALLSTATUS(void);

//增量式编码器模块变量初始化//
void Hall_Encoder1_Init(void)
{
    uint8_t temp = 0,i = 0;
    uint8_t j = 0;
    
    do
    {
       if(++i > 10)
       {
           FaultPrtt_FaultInterface(EncoderErr_1);
           
           for(j = 0;j < 100; j++);
           
           break;
       }
       temp = GET_HALLSTATUS();
    }while(temp == 0||temp == 7);
    
    Hall.State = temp;

//    //                         W   V   U
//    if(Hall.State==2)       // 0   1   0                           2
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+2100;
//    }
//    else if(Hall.State==6)  // 1   1   0                           3
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+2700;
//    }
//    else if(Hall.State==4)  // 1   0   0                           1
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+3300;
//    }
//    else if(Hall.State==5)  // 1   0   1                           5
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+300;
//    }
//    else if(Hall.State==1)  // 0   0   1                           4
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+900;
//    }
//    else if(Hall.State==3)  // 0   1   1                           6
//    {
//        Hall.OffsetAngle = DPI_UElecAngle+1500;
//    }

    //                         W   V   U
    if(Hall.State==2)       // 0   1   0                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 900;
    }
    else if(Hall.State==6)  // 1   1   0                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 300;
    }
    else if(Hall.State==4)  // 1   0   0                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 3300;
    }
    else if(Hall.State==5)  // 1   0   1                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 2700;
    }
    else if(Hall.State==1)  // 0   0   1                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 2100;
    }
    else if(Hall.State==3)  // 0   1   1                           
    {
        Hall.OffsetAngle = DPI_UElecAngle + 1500;
    }    
    
    if(Hall.OffsetAngle >= 3600)
        Hall.OffsetAngle = Hall.OffsetAngle - 3600;
}

uint8_t GET_HALLSTATUS(void)                //(PORTA.PIDR.BYTE & 0x07)
{
    uint8_t temp = 0;
    
    #if SERVO_MCU == RZ_RZT1
    temp = PORTA.PIDR.BYTE;
    
    Hall.State = (temp & 0x07);    
    
    RamMonitor->EncSector = Hall.State;   //用于显示
    #endif
    
    return (temp & 0x07);
}


