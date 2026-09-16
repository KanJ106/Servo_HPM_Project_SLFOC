#include "Version.h"
#include "SV_FuncVar.h"
#include "Flash_Eeprom.h"
#include "r_cg_Project.h"
#include <stdio.h>

char ProSnCode[32];

void SN_init(void)
{
    memcpy(ProductInf.SNCode,&Flash_EeData[SNADDRES],0x40);

    ProSnCode[0] = ProductInf.SNCode[0];

    ProSnCode[1] = ProductInf.SNCode[3];
    ProSnCode[2] = ProductInf.SNCode[2];

    ProSnCode[3] = ProductInf.SNCode[5];
    ProSnCode[4] = ProductInf.SNCode[4];
       
    ProSnCode[5] = ProductInf.SNCode[7];
    ProSnCode[6] = ProductInf.SNCode[6];

    ProSnCode[7] = ProductInf.SNCode[9];
    ProSnCode[8] = ProductInf.SNCode[8];

    ProSnCode[9] = ProductInf.SNCode[11];
    ProSnCode[10] = ProductInf.SNCode[10];

    ProSnCode[11] = ProductInf.SNCode[13];
    ProSnCode[12] = ProductInf.SNCode[12];

    ProSnCode[13] = ProductInf.SNCode[15];
    ProSnCode[14] = ProductInf.SNCode[14];

    ProSnCode[15] = ProductInf.SNCode[17];

    #if SERVOTYPE == SERVO_ETHERCAT
    ProductInf.ServoName[0] = 'C';
    ProductInf.ServoName[1] = 'T';
    ProductInf.ServoName[2] = '0';
    ProductInf.ServoName[3] = '6';    
    ProductInf.ServoName[4] = 'E';
    ProductInf.ServoName[5] = '0';
    ProductInf.ServoType    = 1;
    #endif

    #if SERVOTYPE == SERVO_CAN
    ProductInf.ServoName[0] = 'C';
    ProductInf.ServoName[1] = 'T';
    ProductInf.ServoName[2] = '0';
    ProductInf.ServoName[3] = '6';    
    ProductInf.ServoName[4] = 'C';
    ProductInf.ServoName[5] = '0';
    ProductInf.ServoType    = 2;
    #endif

    #if SERVOTYPE == SERVO_MODBUS
    ProductInf.ServoName[0] = 'C';
    ProductInf.ServoName[1] = 'T';
    ProductInf.ServoName[2] = '0';
    ProductInf.ServoName[3] = '6';    
    ProductInf.ServoName[4] = 'M';
    ProductInf.ServoName[5] = '0';
    ProductInf.ServoType    = 3;
    #endif

    #if SERVOTYPE == SERVO_CANOPEN
    ProductInf.ServoName[0] = 'C';
    ProductInf.ServoName[1] = 'T';
    ProductInf.ServoName[2] = '0';
    ProductInf.ServoName[3] = '6';    
    ProductInf.ServoName[4] = 'O';
    ProductInf.ServoName[5] = '0';
    ProductInf.ServoType    = 4;
    #endif

    ProductInf.SoftCode = PRODUCTCODE;
    ProductInf.HardCode = Flash_EeData[HARDADDRES];

    ProductInf.MortorIe = RamMotor->Ie;
    ProductInf.MortorMaxIe = (Uint16)((Uint32)RamMotor->Ie * (Uint32)RamMotor->TrqConst / 1000L);
    ProductInf.MortorTe = RamMotor->Te;

}