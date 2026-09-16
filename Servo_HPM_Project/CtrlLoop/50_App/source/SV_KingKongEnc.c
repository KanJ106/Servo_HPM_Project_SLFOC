
#include "SV_KingKongEnc.h"
#include "s_comm_init.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"//
#include "Drive.h"
#include "SV_I2c.h"
#include "Pos_FullClosedLoop.h"

uint8_t KingKongTable [256] = { 
0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96, 0xBA, 0x2D, 0x03, 0x94, 
0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C, 0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 
0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75, 0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 
0x58, 0xCF, 0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7, 0xEB, 0x7C, 0x52, 
0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D, 0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 
0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24, 0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 
0x27, 0x09, 0x9E, 0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34, 0x18, 0x8F, 
0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E, 0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 
0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7, 0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 
0x43, 0xD4, 0xFA, 0x6D, 0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65, 0x49, 
0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF, 0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 
0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86, 0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 
0xD9, 0x12, 0x85, 0xAB, 0x3C 
};

Encode_DPT DPT = {{0},0,0,0,0,0,0,0,0,0,0};
ENUM_DPT DptTxCmd[20] = {CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc,
                         CF_ResetOutEnc,CF_ReadOutEnc };
extern Uint8 UartRxuff[22];

static Uint32 SECONDREVLNS1;
static Uint32 SECONDZEROFFFSET1;

#define ENCDIR  0     //  0不取反  1取反

void KingKongTxRequest(ENUM_DPT data)
{
    if(data == CF_ReadAll)
    {
        EncodeSCI_TxData  = (uint8_t)data;
        DPT.EncRxNum = 8;
    }
    else if(data == CF_ReadOutEnc)
    {
        EncodeSCI_TxData  = (uint8_t)data;
        DPT.EncRxNum = 4;  
    }
    else if(data == CF_ResetOutEnc)
    {
        EncodeSCI_TxData  = (uint8_t)data;
        DPT.EncRxNum = 2;         
    }
    else
    {
    }
}

void KingKongRxRequest(void)
{
    Uint8 temp;
    Uint8 crc = 0;
    
    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;
    
    if(temp == DPT.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(DPT.EncRxNum == 8)
        {
            UartRxuff[0] = EncodeSCI_RxData;
            UartRxuff[1] = EncodeSCI_RxData;
            UartRxuff[2] = EncodeSCI_RxData;
            UartRxuff[3] = EncodeSCI_RxData;
            UartRxuff[4] = EncodeSCI_RxData;
            UartRxuff[5] = EncodeSCI_RxData;
            UartRxuff[6] = EncodeSCI_RxData;
            UartRxuff[7] = EncodeSCI_RxData;

            crc = UartRxuff[0];
            crc = UartRxuff[1] ^ KingKongTable[crc];
            crc = UartRxuff[2] ^ KingKongTable[crc];
            crc = UartRxuff[3] ^ KingKongTable[crc];
            crc = UartRxuff[4] ^ KingKongTable[crc];
            crc = UartRxuff[5] ^ KingKongTable[crc];
            crc = UartRxuff[6] ^ KingKongTable[crc];
            crc = KingKongTable[crc];
        }
        else if(DPT.EncRxNum == 6 || DPT.EncRxNum == 2)
        {
            return;
        }
        else
        {
            crc = UartRxuff[DPT.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[DPT.EncRxNum - 1])
        {
            int32 pos;

            pos = UartRxuff[5];
            pos = pos<<8;
            pos += UartRxuff[4];
            pos = pos<<8;
            pos += UartRxuff[3];
            #if ENCDIR
            pos = (16777215 - pos) >> 1;
            #else
            pos = pos >> 1;
            #endif

            if(DPT.Dir == 0)
            {
                DPT.MotABS = pos;
            }
            else
            {
                DPT.MotABS = 8388608L - pos; 
            }

            pos = UartRxuff[2];
            pos = pos<<8;
            pos += UartRxuff[1];
            pos = pos<<8;
            pos += UartRxuff[0];
            #if ENCDIR
            pos = (16777215 - pos) >> 1;
            #else
            pos = pos >> 1;
            #endif

            if(DPT.Dir == 0)
            {
                DPT.OutABSOrig = pos;
            }
            else
            {
                DPT.OutABSOrig = 8388608L - pos; 
            }

            pos = DPT.OutABSOrig - DPT.OutPosOffset;
            if(pos < 0)              
            {
                DPT.OutABS = 8388608L + pos;
            }
            else if(pos >= 8388608L)
            {
                DPT.OutABS = pos - 8388608L;
            }
            else 
            {
                DPT.OutABS = pos;
            }
            
            DPT.S.All = UartRxuff[6];
          
            rxcrcerrcnt = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 10)
            {
                DPT.CrcErrFlag = 1;
            }
            DPT.ErrCRCcnt++;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 10)
        {
            DPT.NoAckFlag = 1;
        }
        DPT.NoAckTime++;      
    }
}

void DPT_Process(void)
{
    KingKongRxRequest();
    KingKongTxRequest(CF_ReadAll);

    if(RamAid->FaultReset == 5)
    {
        DPTPosZero();
        RamAid->FaultReset = 0;
    }

    if(RamAid->FaultReset == 8)
    {
        DPTPos_Alignment();
        RamAid->FaultReset = 0;
    }
}

void DPT_ParaInit(void)
{
    int32 temp,temp1;
     
    SECONDREVLNS1 = 8388608;
    SECONDZEROFFFSET1 = SECONDREVLNS1 >> 1;  
    
    if(RamBasePara->MotDir == 1)
    {
        DPT.Dir = 1; 
    } 
    else
    {
        DPT.Dir = 0;
    }
  
    if(RamAid->AbsType == 2)
    {
        if(RamPD->FullCloseMode == 0)
        {
            if(RamBasePara->MotDir == 0)
            {
                temp = RamServo->Abs1 * 65536;
                DPT.OutPosOffset = temp + RamServo->Abs0; //计算偏移
            }
            else
            {
                temp = RamServo->Abs1 * 65536;
                temp = temp + RamServo->Abs0; //计算偏移 
                
                DPT.OutPosOffset = SECONDREVLNS1 / RamPD->Reduction - temp; 
            }
        }
        else
        {
            temp = RamServo->Mulpos3 * 65536;
            DPT.OutPosOffset = temp + RamServo->Mulpos2; //计算偏移
        }
    }
    else if(RamAid->AbsType == 1)
    {
        temp = RamServo->Mulpos1 * 65536;
        temp = temp + RamServo->Mulpos0; //计算偏移
        
        temp1 = temp / SECONDREVLNS1;
        DPT.OutPosOffset = temp - temp1 * SECONDREVLNS1;       
    }
    else
    {
        temp = RamServo->Mulpos1 * 65536;
        DPT.OutPosOffset = temp + RamServo->Mulpos0; //计算偏移          
    } 
}

void DPTPos_Alignment(void)
{
     int32 Pos_single,Pos_muli;
     float temp;
     int32 rem,div,muli;
     
     Pos_single = DPT.MotABS;
     Pos_muli   = DPT.OutABSOrig;
     
     div = SECONDREVLNS1 / RamPD->Reduction;
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

void DPTPosZero(void)
{
    int32 offset = 0;
    int32 pos1,pos2;
    int32_t temp;
    
    if(RamAid->AbsType == 2)
    {
        if(RamPD->ExEncType > 0)
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
                
                pos1 = DPT.OutABS;
                
                RamServo->Mulpos2 = pos1&0x0000FFFF;
                RamServo->Mulpos3 = (pos1>>16)&0x0000FFFF;
                
                IIcInterfaceB(&RamServo->Mulpos0,4);
                
                ExtPosFb.PosFbTotle = 0;
                PosFb.PosFbTotle = 0; //2025.04.02
            }
            else
            {
                offset = DPT.OutABSOrig - SECONDZEROFFFSET1;
                DPT.OutPosOffset = offset;

                DPT.OutABS = SECONDZEROFFFSET1;

                temp = (Uint32)DPT.OutPosOffset;
                RamServo->Mulpos2 = temp&0x0000FFFF;
                RamServo->Mulpos3 = (temp>>16)&0x0000FFFF;

                IIcInterfaceB(&RamServo->Mulpos2,2);

                ExtPosFb.PosFbTotle = DPT.OutABS - SECONDZEROFFFSET1;
                ExtPosFb.EncSinglePosOld = DPT.OutABS;
                
                PosFb.PosFbTotle = 0; //2025.04.02
            }
        }
        else
        {

        }
    }
    else
    {
        int32 offset = 0;
        Uint32 tempu;
        
        if(RamPD->ExEncType > 0)
        {
            offset = DPT.OutABSOrig - SECONDZEROFFFSET1;
            DPT.OutPosOffset = offset;

            DPT.OutABS = SECONDZEROFFFSET1;

            tempu = (Uint32)DPT.OutPosOffset;
            RamServo->Mulpos0 = tempu&0x0000FFFF;
            RamServo->Mulpos1 = (tempu>>16)&0x0000FFFF;

            IIcInterfaceB(&RamServo->Mulpos0,2);

            ExtPosFb.PosFbTotle = DPT.OutABS - SECONDZEROFFFSET1;
            ExtPosFb.EncSinglePosOld = DPT.OutABS;
            
            PosFb.PosFbTotle = 0; //2025.04.02 
        }
        else
        {
            
            
        }
    }
}

void DPT_FaultHandle(void)
{
	//1.CRC校验检测
    if(DPT.CrcErrFlag == 1)
    {
        DPT.CrcErrFlag = 0;
        FaultPrtt_FaultInterface(EncoderErr_AB);
    }

	//2.无响应检测
	if(DPT.NoAckFlag == 1)
	{
		DPT.NoAckFlag = 0;
		FaultPrtt_FaultInterface(EncoderErr_1);
	} 
    
    //内部报错
}


