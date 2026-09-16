#include "SV_RtT036.h"
#include "s_comm_init.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"//
#include "Drive.h"
#include "SV_I2c.h"
#include "Pos_FullClosedLoop.h"
#include "Cur_MotPaLearn.h"

Encode_RT RTData = {{0},0,0,0,0,0,0,0,0,0,0,0};

extern Uint8 UartRxuff[22];

static Uint32 SECONDREVLNS1;
static Uint32 SECONDZEROFFFSET1;

#define ENCDIR  0     //  0不取反  1取反

void RT_TxRequest(void)
{
    EncodeSCI_TxData  = 0x6A;
    RTData.EncRxNum = 11;
}

void RT_RxRequest(void)
{
    Uint8 temp;
    Uint8 crc = 0;
    
    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;
    
    if(temp == RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 11)
        {
            UartRxuff[0] = EncodeSCI_RxData;
            UartRxuff[1] = EncodeSCI_RxData;
            UartRxuff[2] = EncodeSCI_RxData;
            UartRxuff[3] = EncodeSCI_RxData;
            UartRxuff[4] = EncodeSCI_RxData;
            UartRxuff[5] = EncodeSCI_RxData;
            UartRxuff[6] = EncodeSCI_RxData;
            UartRxuff[7] = EncodeSCI_RxData;
            UartRxuff[8] = EncodeSCI_RxData;
            UartRxuff[9] = EncodeSCI_RxData;
            UartRxuff[10] = EncodeSCI_RxData;

            crc ^= UartRxuff[0];
            crc ^= UartRxuff[1];
            crc ^= UartRxuff[2];
            crc ^= UartRxuff[3];
            crc ^= UartRxuff[4];
            crc ^= UartRxuff[5];
            crc ^= UartRxuff[6];
            crc ^= UartRxuff[7];
            crc ^= UartRxuff[8];
            crc ^= UartRxuff[9];
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;  
            EncodeSCI_ResetFIFO; 
            EncodeSCI_R_STUATS; 
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
            int32 pos;

            pos = UartRxuff[3];
            pos = pos<<8;
            pos += UartRxuff[2];
            pos = pos<<8;
            pos += UartRxuff[1];
            #if ENCDIR
            pos = (131071 - pos);
            #else
            pos = pos;
            #endif

            if(RTData.Dir == 0)
            {
                RTData.MotABS = pos;
            }
            else
            {
                RTData.MotABS = 131071 - pos; 
            }

            pos = UartRxuff[6];
            pos = pos<<8;
            pos += UartRxuff[5];
            pos = pos<<8;
            pos += UartRxuff[4];
            #if ENCDIR
            pos = (524287L - pos);
            #else
            pos = pos;
            #endif

            if(RTData.OutDir == 1)
            {
                pos = (524287L - pos);
            }

            if(RTData.Dir == 0)
            {
                RTData.OutABSOrig = pos;
            }
            else
            {
                RTData.OutABSOrig = 524287L - pos; 
            }

            pos = RTData.OutABSOrig - RTData.OutPosOffset;
            if(pos < 0)              
            {
                RTData.OutABS = 524288L + pos;
            }
            else if(pos >= 524288L)
            {
                RTData.OutABS = pos - 524288L;
            }
            else 
            {
                RTData.OutABS = pos;
            }
            
            RTData.S.All = UartRxuff[9];
          
            rxcrcerrcnt = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 10)
            {
                RTData.CrcErrFlag = 1;
            }
            RTData.ErrCRCcnt++;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 10)
        {
            RTData.NoAckFlag = 1;
        }
        RTData.NoAckTime++;      
    }
}

void RT_ParaInit(void)
{
    int32 temp,temp1;
     
    SECONDREVLNS1 = DrvCoeff.OutEncSglRevLns;
    SECONDZEROFFFSET1 = SECONDREVLNS1 >> 1;  
    
    if(RamBasePara->MotDir == 1)
    {
        RTData.Dir = 1; 
    } 
    else
    {
        RTData.Dir = 0;
    }

    if(RamMotor->OutDir == 1)
    {
        RTData.OutDir = 1; 
    } 
    else
    {
        RTData.OutDir = 0;
    }
  
    if(RamAid->AbsType == 2)
    {
        if(RamPD->FullCloseMode == 0)
        {
            if(RamBasePara->MotDir == 0)
            {
                temp = RamServo->Abs1 * 65536 + RamServo->Abs0;
                RTData.OutPosOffset = temp; //计算偏移
            }
            else
            {
                temp = RamServo->Abs1 * 65536;
                temp = temp + RamServo->Abs0; //计算偏移 

                if(temp == 0)
                {
                    RTData.OutPosOffset = 0;
                }
                else
                {
                    RTData.OutPosOffset = SECONDREVLNS1 / RamPD->Reduction - temp;
                }
            }
        }
        else
        {
            temp = RamServo->Mulpos3 * 65536;
            RTData.OutPosOffset = temp + RamServo->Mulpos2; //计算偏移
        }
    }
    else if(RamAid->AbsType == 1)
    {
        temp = RamServo->Mulpos1 * 65536;
        temp = temp + RamServo->Mulpos0; //计算偏移
        
        temp1 = temp / SECONDREVLNS1;
        RTData.OutPosOffset = temp - temp1 * SECONDREVLNS1;       
    }
    else
    {
        temp = RamServo->Mulpos1 * 65536;
        RTData.OutPosOffset = temp + RamServo->Mulpos0; //计算偏移          
    } 
}

void RTPos_Alignment(void)
{
     int32 Pos_single,Pos_muli;
     float temp;
     int32 rem,div,muli;
     
     Pos_single = RTData.MotABS;
     Pos_muli   = RTData.OutABSOrig;
     
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

void RTPosZero(void)
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
                
                pos1 = RTData.OutABS;
                
                RamServo->Mulpos2 = pos1&0x0000FFFF;
                RamServo->Mulpos3 = (pos1>>16)&0x0000FFFF;
                
                IIcInterfaceB(&RamServo->Mulpos0,4);
                
                ExtPosFb.PosFbTotle = 0;
                PosFb.PosFbTotle = 0; //2025.04.02
            }
            else
            {
                offset = RTData.OutABSOrig - SECONDZEROFFFSET1;
                RTData.OutPosOffset = offset;

                RTData.OutABS = SECONDZEROFFFSET1;

                temp = (Uint32)RTData.OutPosOffset;
                RamServo->Mulpos2 = temp&0x0000FFFF;
                RamServo->Mulpos3 = (temp>>16)&0x0000FFFF;

                IIcInterfaceB(&RamServo->Mulpos2,2);

                ExtPosFb.PosFbTotle = RTData.OutABS - SECONDZEROFFFSET1;
                ExtPosFb.EncSinglePosOld = RTData.OutABS;
                
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
            offset = RTData.OutABSOrig - SECONDZEROFFFSET1;
            RTData.OutPosOffset = offset;

            RTData.OutABS = SECONDZEROFFFSET1;

            tempu = (Uint32)RTData.OutPosOffset;
            RamServo->Mulpos0 = tempu&0x0000FFFF;
            RamServo->Mulpos1 = (tempu>>16)&0x0000FFFF;

            IIcInterfaceB(&RamServo->Mulpos0,2);

            ExtPosFb.PosFbTotle = RTData.OutABS - SECONDZEROFFFSET1;
            ExtPosFb.EncSinglePosOld = RTData.OutABS;
            
            PosFb.PosFbTotle = 0; //2025.04.02 
        }
        else
        {
            
            
        }
    }
}

void RT_TxRequestCal1(void)
{
    EncodeSCI_TxData  = 0x76;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x76;
    RTData.EncRxNum = 5;
}

void RT_TxRequestCal2(void)
{
    EncodeSCI_TxData  = 0x76;
    EncodeSCI_TxData  = 0x01;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x77;
    RTData.EncRxNum = 5;
}

void RT_TxRequestCal3(void)
{
    EncodeSCI_TxData  = 0x76;
    EncodeSCI_TxData  = 0x02;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x74;
    RTData.EncRxNum = 22;
}

void RT_TxRequestCal4(void)
{
    EncodeSCI_TxData  = 0x76;
    EncodeSCI_TxData  = 0x04;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x72;
    RTData.EncRxNum = 22;
}

void RT_TxRequestSetPosZero1(void)//输入端单圈清零
{
    EncodeSCI_TxData  = 0xC2;
    RTData.EncRxNum = 6;
}

void RT_TxRequestSetPosZero2(void)//输出端单圈清零
{
    EncodeSCI_TxData  = 0x62;
    RTData.EncRxNum = 6;
}

void RT_TxRequestSetPosZero3(void)//输出端多圈清零
{
    EncodeSCI_TxData  = 0x6B;
    RTData.EncRxNum = 6;
}

void RT_TxRequestRdErr1(void)//将编码器切换到电机端误差输出指令
{
    EncodeSCI_TxData  = 0x91;
    EncodeSCI_TxData  = 0x10;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x81;
    RTData.EncRxNum = 7;
}

void RT_TxRequestRdErr2(void)//读取编码器电机端误差指令
{
    EncodeSCI_TxData  = 0x91;
    EncodeSCI_TxData  = 0x11;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x80;
    RTData.EncRxNum = 7;
}

void RT_TxRequestRdErr3(void)//将编码器切换到减速机端误差输出指令
{
    EncodeSCI_TxData  = 0x91;
    EncodeSCI_TxData  = 0x20;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0xB1;
    RTData.EncRxNum = 7;
}

void RT_TxRequestRdErr4(void)//读取编码器减速机端误差指令
{
    EncodeSCI_TxData  = 0x91;
    EncodeSCI_TxData  = 0x21;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0x00;
    EncodeSCI_TxData  = 0xB0;
    RTData.EncRxNum = 7;
}

void RT_RxRequestRdErr1(void)
{
    Uint8 temp;
    Uint8 crc = 0;
    Uint8 rxcnt;
    
    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;
    
    if(temp == RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 7)
        {
            for(rxcnt=0;rxcnt<RTData.EncRxNum;rxcnt++)
            {
                UartRxuff[rxcnt] = EncodeSCI_RxData;
            }

            for(rxcnt=0;rxcnt<RTData.EncRxNum-1;rxcnt++)
            {
                crc ^= UartRxuff[rxcnt];
            }
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
          //  RTData.CalStat = 1;
         //   RTData.CalStep = 1;
            rxcrcerrcnt = 0;
         //   RTData.CalCnt1 = 0;
            RTData.Calflag = 1;
            RTData.ErrCRCcnt = 0;
            RTData.NoAckTime = 0;
            RTData.CalCnt3++;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 24)
            {
                RTData.CrcErrFlag = 1;
                RTData.Calflag = 2;
            }
            RTData.ErrCRCcnt++;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 4)
        {
            RTData.NoAckFlag = 1;
            RTData.Calflag = 3;
        }
        RTData.NoAckTime++;      
    }
}


void RT_RxRequestRdErr2(void)
{
    Uint8 temp,rxcnt;
    Uint8 crc = 0;
    int32_t errtemp;
    
    static Uint16 rxnoackcnt = 0;
    static Uint16 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;

    RTData.CalNote2 = EncodeSCI_FIFO_NUM;
    
    if(temp >= RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 7)
        {
            for(rxcnt=0;rxcnt<RTData.EncRxNum;rxcnt++)
            {
                UartRxuff[rxcnt] = EncodeSCI_RxData;
            }

            for(rxcnt=0;rxcnt<RTData.EncRxNum-1;rxcnt++)
            {
                crc ^= UartRxuff[rxcnt];
            }
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
            if(UartRxuff[1]==0x11)//电机端误差结果
            {
                errtemp = (int32)((uint32_t)UartRxuff[2]|((uint32_t)UartRxuff[3]<<8)|((uint32_t)UartRxuff[4]<<16)|((uint32_t)UartRxuff[5]<<24));
                RTData.CalMotorAglErr3 = (int16)((float)errtemp * 0.0429153f);//扩大1000
                //RTData.CalMotorAglErr4 = (int32)errtemp*0.0000429153f;//errtemp*360/131072;

                if(RTData.CalMotorAglErr3>MotorAngErrMax || RTData.CalMotorAglErr3<-MotorAngErrMax){//当误差超出范围时，报错并记录
                    //RTData.CalMotorAglErrMAX = RTData.CalMotorAglErr4;
                    RTData.Calflag2 = 1;
                }

                //RTData.CalStat = 6;
                //RTData.CalStep = 6;
                rxcrcerrcnt = 0;
                RTData.Calflag = 10;

            }
            else if(UartRxuff[1]==0x21)//减速机端误差结果
            {
                errtemp = (int32)((uint32_t)UartRxuff[2]|((uint32_t)UartRxuff[3]<<8)|((uint32_t)UartRxuff[4]<<16)|((uint32_t)UartRxuff[5]<<24));
                RTData.CalReducerAglErr3 = (int16)((float)errtemp * 0.0429153f);//扩大1000
                //RTData.CalReducerAglErr4 = (int32)errtemp*0.0000429153f;//errtemp*360/(131072*4);

                if(RTData.CalReducerAglErr3>ReduceAngErrMax || RTData.CalReducerAglErr3<-ReduceAngErrMax){//当误差超出范围时，报错并记录
                    //RTData.CalReducerAglErrMAX = RTData.CalReducerAglErr4;
                    RTData.Calflag2 = 2;
                }

                //RTData.CalStat = 8;
                //RTData.CalStep = 8;
                rxcrcerrcnt = 0;
                RTData.Calflag = 11;
            }
            rxcrcerrcnt = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 65534)
            {
                RTData.CalStat = 2;
                RTData.CalStep = 2;
                //rxcrcerrcnt = 0;
                RTData.Calflag2 = 3;

                RTData.CrcErrFlag = 1;
                RTData.Calflag = 12;
            }
            RTData.ErrCRCcnt++;

        //RTData.CalStat = 2;
        //RTData.CalStep = 2;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 65534)
        {

                RTData.CalStat = 2;
                RTData.CalStep = 2;
               // rxnoackcnt = 0;
               RTData.Calflag2 = 4;

            RTData.NoAckFlag = 1;
            RTData.Calflag = 13;
        }
        RTData.NoAckTime++;   

        //RTData.CalStat = 2;
        //RTData.CalStep = 2;
          
    }
}

void RT_RxRequestCal1(void)
{
    Uint8 temp;
    Uint8 crc = 0;
    
    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;
    
    if(temp == RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 5)
        {
            UartRxuff[0] = EncodeSCI_RxData;
            UartRxuff[1] = EncodeSCI_RxData;
            UartRxuff[2] = EncodeSCI_RxData;
            UartRxuff[3] = EncodeSCI_RxData;
            UartRxuff[4] = EncodeSCI_RxData;

            if(UartRxuff[0]==0x76&&UartRxuff[1]==0x00&&UartRxuff[2]==0x00&&UartRxuff[3]==0x00&&UartRxuff[4]==0x76)
            {
                crc = 1;
            }
        }
        else
        {
            crc = 0;
        }

    
        if(crc == 1)
        {
            RTData.CalStat = 1;
            RTData.CalStep = 1;
            rxcrcerrcnt = 0;
            RTData.CalCnt1 = 0;
            RTData.Calflag = 1;
            RTData.ErrCRCcnt = 0;
            RTData.NoAckTime = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 24)
            {
                RTData.CrcErrFlag = 1;
                RTData.Calflag = 2;
            }
            RTData.ErrCRCcnt++;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 4)
        {
            RTData.NoAckFlag = 1;
            RTData.Calflag = 3;
        }
        RTData.NoAckTime++;      
    }
}

void RT_RxRequestCal2(void)
{
    Uint8 temp;
    Uint8 crc = 0;
    
    static Uint8 rxnoackcnt = 0;
    static Uint8 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;
    
    if(temp == RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 5)
        {
            UartRxuff[0] = EncodeSCI_RxData;
            UartRxuff[1] = EncodeSCI_RxData;
            UartRxuff[2] = EncodeSCI_RxData;
            UartRxuff[3] = EncodeSCI_RxData;
            UartRxuff[4] = EncodeSCI_RxData;

            crc ^= UartRxuff[0];
            crc ^= UartRxuff[1];
            crc ^= UartRxuff[2];
            crc ^= UartRxuff[3];
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
            
            if(UartRxuff[2]==0x11)//校准成功
            {
                RTData.CalStat = 2;
                RTData.CalStep = 2;
                rxcrcerrcnt = 0;
                
                RTData.CalNote1 = UartRxuff[2];
                RTData.Calflag = 5;
            }
            else if(UartRxuff[2]==0x3f)//校准失败
            {
                RTData.CalStat = 3;
                RTData.CalStep = 3;
                rxcrcerrcnt = 0;

                RTData.CalNote1 = UartRxuff[2];
                RTData.Calflag = 6;
            }
            

            rxcrcerrcnt = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 4)
            {
                RTData.CrcErrFlag = 1;
                RTData.Calflag = 7;
            }
            RTData.ErrCRCcnt++;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 4)
        {
            RTData.NoAckFlag = 1;
            RTData.Calflag = 8;
        }
        RTData.NoAckTime++;      
    }
}

void RT_RxRequestCal3(void)
{
    Uint8 temp,rxcnt;
    Uint8 crc = 0;
    //uint16_t errtemp;
    
    static Uint16 rxnoackcnt = 0;
    static Uint16 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;

    RTData.CalNote2 = EncodeSCI_FIFO_NUM;
    
    if(temp >= RTData.EncRxNum)
    {
        rxnoackcnt = 0;
                        
        if(RTData.EncRxNum == 22)
        {
            for(rxcnt=0;rxcnt<RTData.EncRxNum;rxcnt++)
            {
                UartRxuff[rxcnt] = EncodeSCI_RxData;
            }

            for(rxcnt=0;rxcnt<RTData.EncRxNum-1;rxcnt++)
            {
                crc ^= UartRxuff[rxcnt];
            }
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
            int16_t errtemp;

            if(UartRxuff[0]==0x76&&UartRxuff[1]==0x02)//电机端校准结果
            {
                errtemp = (int16)((uint16_t)UartRxuff[2]|((uint16_t)UartRxuff[3]<<8));
                RTData.NsinVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[4]|((uint16_t)UartRxuff[5]<<8));
                RTData.NsinOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[6]|((uint16_t)UartRxuff[7]<<8));
                RTData.NcosVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[8]|((uint16_t)UartRxuff[9]<<8));
                RTData.NcosOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[10]|((uint16_t)UartRxuff[11]<<8));
                RTData.MsinVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[12]|((uint16_t)UartRxuff[13]<<8));
                RTData.MsinOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[14]|((uint16_t)UartRxuff[15]<<8));
                RTData.McosVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[16]|((uint16_t)UartRxuff[17]<<8));
                RTData.McosOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[18]|((uint16_t)UartRxuff[19]<<8));
                RTData.CalMotorAglErr2 = (int16_t)((float)errtemp*2.746582f);//errtemp*360/131072;
                //RTData.CalMotorAglErr = RTData.CalMotorAglErr2*0.002746582f;//

                RTData.CalStat = 6;
                RTData.CalStep = 6;
                rxcrcerrcnt = 0;
                RTData.Calflag = 10;

            }
            else if(UartRxuff[0]==0x76&&UartRxuff[1]==0x04)//减速机端校准结果
            {
                errtemp = (int16)((uint16_t)UartRxuff[2]|((uint16_t)UartRxuff[3]<<8));
                RTData.ONsinVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[4]|((uint16_t)UartRxuff[5]<<8));
                RTData.ONsinOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[6]|((uint16_t)UartRxuff[7]<<8));
                RTData.ONcosVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[8]|((uint16_t)UartRxuff[9]<<8));
                RTData.ONcosOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[10]|((uint16_t)UartRxuff[11]<<8));
                RTData.OMsinVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[12]|((uint16_t)UartRxuff[13]<<8));
                RTData.OMsinOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[14]|((uint16_t)UartRxuff[15]<<8));
                RTData.OMcosVpp = (int16)((float)errtemp * 0.22f);

                errtemp = (int16)((uint16_t)UartRxuff[16]|((uint16_t)UartRxuff[17]<<8));
                RTData.OMcosOffset = (int16)((float)errtemp * 0.44f);

                errtemp = (int16)((uint16_t)UartRxuff[18]|((uint16_t)UartRxuff[19]<<8));
                RTData.CalReducerAglErr2 = (int16_t)((float)errtemp * 0.686646f);//errtemp*360/(131072*4);
                //RTData.CalReducerAglErr = RTData.CalReducerAglErr2*0.0006866455f;

                RTData.CalStat = 8;
                RTData.CalStep = 8;
                rxcrcerrcnt = 0;
                RTData.Calflag = 11;
            }
            rxcrcerrcnt = 0;
        }
        else
        {
            rxcrcerrcnt++;
            if(rxcrcerrcnt >= 65534)
            {
                //RTData.CalStat = 2;
                //RTData.CalStep = 2;
                //rxcrcerrcnt = 0;

                RTData.CrcErrFlag = 1;
                RTData.Calflag = 12;
            }
            RTData.ErrCRCcnt++;

        RTData.CalStat = 2;
        RTData.CalStep = 2;
        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

        rxnoackcnt++;
        if(rxnoackcnt >= 65534)
        {

               // RTData.CalStat = 2;
               // RTData.CalStep = 2;
               // rxnoackcnt = 0;

            RTData.NoAckFlag = 1;
            RTData.Calflag = 13;
        }
        RTData.NoAckTime++;   

        RTData.CalStat = 2;
        RTData.CalStep = 2;
          
    }
}


void RT_RxRequestCal4(void)
{
    Uint8 temp,rxcnt;
    Uint8 crc = 0;
    //uint16_t errtemp;
    
    //static Uint16 rxnoackcnt = 0;
    //static Uint16 rxcrcerrcnt = 0;
    
    temp = EncodeSCI_FIFO_NUM;

    RTData.CalNote2 = EncodeSCI_FIFO_NUM;
    
    if(temp >= RTData.EncRxNum)
    {
        //rxnoackcnt = 0;
        RTData.NoAckTime = 0;
                        
        if(RTData.EncRxNum == 6)
        {
            for(rxcnt=0;rxcnt<RTData.EncRxNum;rxcnt++)
            {
                UartRxuff[rxcnt] = EncodeSCI_RxData;
            }

            for(rxcnt=0;rxcnt<RTData.EncRxNum-1;rxcnt++)
            {
                crc ^= UartRxuff[rxcnt];
            }
        }
        else
        {
            crc = UartRxuff[RTData.EncRxNum - 1] + 1;   
        }
    
        if(crc == UartRxuff[RTData.EncRxNum - 1])
        {
            if(RTData.SetZeroCnt2<65535)
            {
                RTData.SetZeroCnt2++;
            }
            else
            {
                RTData.SetZeroflag = 6;
            }

            if(UartRxuff[0]==0xC2)//&&UartRxuff[2]==0x00&&UartRxuff[3]==0x00&&UartRxuff[4]==0x00)//输入端单圈清零
            {//需要连续10次以上，则置零成功，则进入下一阶段
                if(RTData.SetZeroCnt2>15)
                {
                    RTData.SetZeroStat = 1;
                    RTData.SetZeroStep = 1;
                    RTData.SetZeroflag = 1;
                    RTData.SetZeroCnt2 = 0;
                    RTData.SetZeroCnt1 = 0;
                    RTData.ErrCRCcnt = 0;
                    RTData.NoAckTime = 0;
                }
            }
            else if(UartRxuff[0]==0x62)//输出端单圈清零
            {//需要连续10次以上，则置零成功，则进入下一阶段
                if(RTData.SetZeroCnt2>15)
                {
                    RTData.SetZeroStat = 2;
                    RTData.SetZeroStep = 2;
                    RTData.SetZeroflag = 2;
                    RTData.SetZeroCnt2 = 0;
                    RTData.SetZeroCnt1 = 0;
                    RTData.ErrCRCcnt = 0;
                    RTData.NoAckTime = 0;
                }
            }
            else if(UartRxuff[0]==0x6B)//输出端多圈清零
            {//需要连续10次以上，则置零成功，则进入下一阶段
                if(RTData.SetZeroCnt2>15)
                {
                    RTData.SetZeroStat = 3;
                    RTData.SetZeroStep = 3;
                    RTData.SetZeroflag = 3;
                    RTData.SetZeroCnt2 = 0;
                    RTData.SetZeroCnt1 = 0;
                    RTData.ErrCRCcnt = 0;
                    RTData.NoAckTime = 0;
 //                   MotPaLearn.LearnMod.bit.CmpFlg = 1;
                }
            
            }

           // rxcrcerrcnt = 0;
            RTData.ErrCRCcnt = 0;
        }
        else
        {
           // rxcrcerrcnt++;
            if(RTData.ErrCRCcnt >= 65534)
            {
                RTData.CrcErrFlag = 1;
                RTData.SetZeroflag = 4;
            }
            RTData.ErrCRCcnt++;


        }
    }
    else
    {
        EncodeSCI_ResetFIFO; 
        EncodeSCI_R_STUATS;

       // rxnoackcnt++;
        if(RTData.NoAckTime >= 65534)
        {

            RTData.NoAckFlag = 1;
            RTData.SetZeroflag = 5;
        }
        RTData.NoAckTime++;   
          
    }
}


void RT_Process(void)
{
    static Uint8 rxcnt1 = 0;
    static Uint8 state = 0;

    if((MotPaLearn.LearnMod.bit.Mode == 5) && (RamMotor->TuningType == 5) && (MotPaLearn.LearnMod.bit.CmpFlg == 0))//当编码器校准模式开启时
    {
        if(RTData.CalCnt1<2147483648)
        {
            RTData.CalCnt1++;
        }

        switch (RTData.CalStep){
            case 0://发送校准启动指令
                RT_RxRequestCal1();
                RT_TxRequestCal1();
                
                if(RTData.CalCnt1>3)//超时报错,187.5us
                {
                    RTData.NoAckFlag = 1;
                    RTData.CalStat = 3;
                    RTData.CalStep = 3;
                    RTData.Calflag = 4;
                }

                MotPaLearn.LearnMod.bit.CmpFlg = 0;
                break;
             case 1://校准状态查询指令
                if(RTData.CalCnt1 > 90000)
                {
                    RT_RxRequestCal2();
                    RT_TxRequestCal2();
                    RTData.CalCnt2++;
                    RTData.CalCnt1 = 0;
                }

                RTData.CalNote1 = UartRxuff[2];
                
                if(RTData.CalCnt2 > 300)//超时报错，20s
                {
                   
                    RTData.NoAckFlag = 1;
                    RTData.CalStat = 3;
                    RTData.CalStep = 3;
                    RTData.Calflag = 9;
                }
                break;
            case 2://校准完成，查询校准结果阶段1
                RT_TxRequestCal3();
                RTData.CalStat = 4;
                RTData.CalStep = 4;
                rxcnt1 = 0;

                break;
            case 3://校准失败，退出校准模式
                //RTData.CalCnt1 = 0;
                //RTData.CalCnt2 = 0;
                //MotPaLearn.LearnMod.bit.CmpFlg = 1;

                RT_TxRequestCal3();
                RTData.CalStat = 4;
                RTData.CalStep = 4;
                rxcnt1 = 0;
                break;
            case 4://校准完成，查询校准结果阶段2
                rxcnt1++;
                if(rxcnt1>=2)
                {
                    RTData.CalStat = 5;
                    RTData.CalStep = 5;
                }


                break;
            case 5://校准完成，查询校准结果阶段3
                RT_RxRequestCal3();

                break;
            case 6://校准完成，查询校准结果阶段4
                RT_TxRequestCal4();
                RTData.CalStat = 7;
                RTData.CalStep = 7;

                break;
            case 7://校准完成，查询校准结果阶段5
                RTData.CalStat = 5;
                RTData.CalStep = 5;

                break;

            case 8://校准完成，查询完成，退出校准模式
                RTData.CalCnt1 = 0;
                RTData.CalCnt2 = 0;

                //RamAid->EncSetZeroSel = 1;//校准完成，再启动置零

                MotPaLearn.LearnMod.bit.CmpFlg = 1;
                break;
            default:break;
        }
        state = 0;
    }
    else if(RamMotor->TuningType == 7)//读取电机端和减速机端误差
    {

        if(RTData.CalCnt1<2147483648)
        {
            RTData.CalCnt1++;
        }

        switch (RTData.CalStep)
        {
            case 0://发送电机端切换指令

                if(RTData.CalCnt2==0)//在指令切换的起始阶段，先清空一次缓存
                {
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;
                    MotPaLearn.LearnMod.bit.CmpFlg = 0;
                }
                else if(RTData.CalCnt2 > 30){//连续10次以上发送完成，则跳转到下一步骤               
                    if(RTData.CalCnt3 < 10)
                    {
                        FaultPrtt_FaultInterface(EncoderErr_1);//在30次发送中，正常返回不超过10次，则报错
                    }

                    RTData.CalStep = 1;
                    RTData.CalCnt1 = 0; 
                    RTData.CalCnt2 = 0;
                    RTData.CalCnt3 = 0;
                }

                if(RTData.CalCnt1>100){//间隔1ms以上发送一次指令

                    if(RTData.CalCnt2<32768)
                    {
                        RTData.CalCnt2++;
                    }
                
                    RT_RxRequestRdErr1();
                    RT_TxRequestRdErr1();//将编码器切换到电机端误差输出指令   
                    
                    RTData.CalCnt1 = 0;   
                }

                break;
             case 1://读取电机端误差
                if(RTData.CalCnt2==0)//在指令切换的起始阶段，先清空一次缓存
                {
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;
                }
                else if(RTData.CalCnt2 > 65535){//连续10次以上发送完成，则跳转到下一步骤               
                    RTData.CalStep = 2;
                    RTData.CalCnt1 = 0; 
                    RTData.CalCnt2 = 0;
                }

                if(RTData.CalCnt1>20){//间隔1ms以上发送一次指令

                    if(RTData.CalCnt2<8388608)
                    {
                        RTData.CalCnt2++;
                    }
                
                    RT_RxRequestRdErr2();
                    RT_TxRequestRdErr2();//读取编码器电机端误差   
                    
                    RTData.CalCnt1 = 0;   
                }

                break;
            case 2://发送输出端切换指令
                if(RTData.CalCnt2==0)//在指令切换的起始阶段，先清空一次缓存
                {
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;
                    MotPaLearn.LearnMod.bit.CmpFlg = 0;
                }
                else if(RTData.CalCnt2 > 30){//连续10次以上发送完成，则跳转到下一步骤               
                    if(RTData.CalCnt3 < 10)
                    {
                        FaultPrtt_FaultInterface(EncoderErr_1);//在30次发送中，正常返回不超过10次，则报错
                    }

                    RTData.CalStep = 3;
                    RTData.CalCnt1 = 0; 
                    RTData.CalCnt2 = 0;
                    RTData.CalCnt3 = 0;
                }

                if(RTData.CalCnt1>100){//间隔1ms以上发送一次指令

                    if(RTData.CalCnt2<32768)
                    {
                        RTData.CalCnt2++;
                    }
                    else
                    {

                    }
                
                    RT_RxRequestRdErr1();
                    RT_TxRequestRdErr3();//将编码器切换到输出端误差输出指令   
                    
                    RTData.CalCnt1 = 0;   
                }

                break;
            case 3://读取输出端误差
                if(RTData.CalCnt2==0)//在指令切换的起始阶段，先清空一次缓存
                {
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;

                    RTData.CalNote1 = 9;
                }
                else if(RTData.CalCnt2 > 65535){//连续10次以上发送完成，则跳转到下一步骤               
                    RTData.CalStep = 4;
                    RTData.CalCnt1 = 0; 
                    RTData.CalCnt2 = 0;
                }

                if(RTData.CalCnt1>20){//间隔1ms以上发送一次指令

                    if(RTData.CalCnt2<8388608)
                    {
                        RTData.CalCnt2++;
                    }
                
                    RT_RxRequestRdErr2();
                    RT_TxRequestRdErr4();//读取编码器输出端误差   
                    
                    RTData.CalCnt1 = 0;   
                }

                break;
            case 4://校准完成，查询校准结果阶段2
                if(RTData.Calflag2>0){//当误差超范围或者读取误差数据错误时，报错
                    FaultPrtt_FaultInterface(EncoderErr_Z);
                }

                RTData.CalCnt1 = 0;
                RTData.CalCnt2 = 0;

                MotPaLearn.LearnMod.bit.CmpFlg = 1;
                break;
            default:break;
        }
        state = 0;
    }
    else if(RamMotor->TuningType == 128)//当编码器置零模式开启时
    {
        if(RTData.SetZeroCnt1<8388608)
        {
            RTData.SetZeroCnt1++;
        }
        else
        {
            RTData.SetZeroflag = 7;
        }

        if(RTData.SetZeroflag > 3)//置零失败时，报错
        {
            RTData.SetZeroflag = 0;
            FaultPrtt_FaultInterface(EncoderErr_1);
        }

        switch (RTData.SetZeroStep){
            case 0://
                if(RTData.SetZeroCnt1<500)
                {
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;
                }


                if(RTData.SetZeroCnt1>8000){
                
                    RT_RxRequestCal4();//输入端单圈清零
                    RT_TxRequestSetPosZero1();           
                }

          //      MotPaLearn.LearnMod.bit.CmpFlg = 0;

                break;
            case 1://
                if(RTData.SetZeroCnt1>100){                    
                    RT_RxRequestCal4();//输出端单圈清零
                    RT_TxRequestSetPosZero2();             
                }

                break;
            case 2://
                if(RTData.SetZeroCnt1>100){                    
                    RT_RxRequestCal4();//输出端多圈清零
                    RT_TxRequestSetPosZero3();           
                }

                break;
            case 3://清零结束
                if(RTData.SetZeroCnt1>100){
                    EncodeSCI_ResetFIFO; 
                    EncodeSCI_R_STUATS;
                   // MotPaLearn.LearnMod.bit.CmpFlg = 1;
                   
                    RTData.SetZeroCnt1 = 0;
                    RTData.SetZeroCnt2 = 0;
                    RTData.SetZeroStat = 0;
                    RTData.SetZeroStep = 0;

                    RamMotor->TuningType = 0;      
                }
                break;    
            default:break;
            }
        state = 0;
    }
    else if(RamMotor->TuningType == 129)//只读取校准结果
    {
        if(state == 0)
        {
            RT_RxRequest();
            RT_TxRequestCal3();
            state = 1;
        }
        else if(state == 1)
        {
            state = 2;
        }
        else if(state == 2)
        {
            state = 3;
            RT_RxRequestCal3();
            RT_TxRequestCal4();
        }
        else if(state == 3)
        {
            state = 4;
        }
        else if(state == 4)
        {
            RT_RxRequestCal3();
            RT_TxRequest();
            RamMotor->TuningType = 0;
            RTData.CalStat = 0;
            RTData.CalStep = 0;
            state = 0;
        }
        else
        {
            state = 0;
        }
    }
    else
    {
        RT_RxRequest();
        RT_TxRequest();

        if(RamAid->FaultReset == 5)
        {
            RTPosZero();
            RamAid->FaultReset = 0;
        }

        if(RamAid->FaultReset == 8)
        {
            RTPos_Alignment();
            RamAid->FaultReset = 0;
        }
        state = 0;
    }
}

void RT_FaultHandle(void)
{
	//1.CRC校验检测
    if(RTData.CrcErrFlag == 1)
    {
        RTData.CrcErrFlag = 0;
        FaultPrtt_FaultInterface(EncoderErr_AB);
    }

	//2.无响应检测
	if(RTData.NoAckFlag == 1)
	{
		RTData.NoAckFlag = 0;
		FaultPrtt_FaultInterface(EncoderErr_1);
	} 
    
    if(RTData.S.Bit.CE)
    {
        RTData.Cecnt++;
        if(RTData.Cecnt >= 10)
        {
            FaultPrtt_FaultInterface(EncoderErr_Z);
        }
    }
    //内部报错
}

