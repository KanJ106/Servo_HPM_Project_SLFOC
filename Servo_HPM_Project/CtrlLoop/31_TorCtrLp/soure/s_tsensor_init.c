#include "s_tsensor_init.h"
#include "SV_FaultProtect.h"//
#include "SV_AppDriveInterface.h"
#include "SV_I2c.h"

SENSOR_Type Sensor = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int32_t TorQueData = 0;
uint8_t TorFbFlag = 0;
uint8_t SensorData[32];
extern float ActualTorqueClose;
uint16_t crc_table_Tor[256] = {
0x0000	,0xC0C1	,0xC181	,0x0140	,0xC301	,0x03C0	,0x0280	,0xC241	,0xC601	,0x06C0,
0x0780	,0xC741	,0x0500	,0xC5C1	,0xC481	,0x0440	,0xCC01	,0x0CC0	,0x0D80	,0xCD41,
0x0F00	,0xCFC1	,0xCE81	,0x0E40	,0x0A00	,0xCAC1	,0xCB81	,0x0B40	,0xC901	,0x09C0,
0x0880	,0xC841	,0xD801	,0x18C0	,0x1980	,0xD941	,0x1B00	,0xDBC1	,0xDA81	,0x1A40,
0x1E00	,0xDEC1	,0xDF81	,0x1F40	,0xDD01	,0x1DC0	,0x1C80	,0xDC41	,0x1400	,0xD4C1,
0xD581	,0x1540	,0xD701	,0x17C0	,0x1680	,0xD641	,0xD201	,0x12C0	,0x1380	,0xD341,
0x1100	,0xD1C1	,0xD081	,0x1040	,0xF001	,0x30C0	,0x3180	,0xF141	,0x3300	,0xF3C1,
0xF281	,0x3240	,0x3600	,0xF6C1	,0xF781	,0x3740	,0xF501	,0x35C0	,0x3480	,0xF441,
0x3C00	,0xFCC1	,0xFD81	,0x3D40	,0xFF01	,0x3FC0	,0x3E80	,0xFE41	,0xFA01	,0x3AC0,
0x3B80	,0xFB41	,0x3900	,0xF9C1	,0xF881	,0x3840	,0x2800	,0xE8C1	,0xE981	,0x2940,
0xEB01	,0x2BC0	,0x2A80	,0xEA41	,0xEE01	,0x2EC0	,0x2F80	,0xEF41	,0x2D00	,0xEDC1,
0xEC81	,0x2C40	,0xE401	,0x24C0	,0x2580	,0xE541	,0x2700	,0xE7C1	,0xE681	,0x2640,
0x2200	,0xE2C1	,0xE381	,0x2340	,0xE101	,0x21C0	,0x2080	,0xE041	,0xA001	,0x60C0,
0x6180	,0xA141	,0x6300	,0xA3C1	,0xA281	,0x6240	,0x6600	,0xA6C1	,0xA781	,0x6740,
0xA501	,0x65C0	,0x6480	,0xA441	,0x6C00	,0xACC1	,0xAD81	,0x6D40	,0xAF01	,0x6FC0,
0x6E80	,0xAE41	,0xAA01	,0x6AC0	,0x6B80	,0xAB41	,0x6900	,0xA9C1	,0xA881	,0x6840,
0x7800	,0xB8C1	,0xB981	,0x7940	,0xBB01	,0x7BC0	,0x7A80	,0xBA41	,0xBE01	,0x7EC0,
0x7F80	,0xBF41	,0x7D00	,0xBDC1	,0xBC81	,0x7C40	,0xB401	,0x74C0	,0x7580	,0xB541,
0x7700	,0xB7C1	,0xB681	,0x7640	,0x7200	,0xB2C1	,0xB381	,0x7340	,0xB101	,0x71C0,
0x7080	,0xB041	,0x5000	,0x90C1	,0x9181	,0x5140	,0x9301	,0x53C0	,0x5280	,0x9241,
0x9601	,0x56C0	,0x5780	,0x9741	,0x5500	,0x95C1	,0x9481	,0x5440	,0x9C01	,0x5CC0,
0x5D80	,0x9D41	,0x5F00	,0x9FC1	,0x9E81	,0x5E40	,0x5A00	,0x9AC1	,0x9B81	,0x5B40,
0x9901	,0x59C0	,0x5880	,0x9841	,0x8801	,0x48C0	,0x4980	,0x8941	,0x4B00	,0x8BC1,
0x8A81	,0x4A40	,0x4E00	,0x8EC1	,0x8F81	,0x4F40	,0x8D01	,0x4DC0	,0x4C80	,0x8C41,
0x4400	,0x84C1	,0x8581	,0x4540	,0x8701	,0x47C0	,0x4680	,0x8641	,0x8201	,0x42C0,
0x4380	,0x8341	,0x4100	,0x81C1	,0x8081	,0x4040};

void Sci_TorSensorInit(void)
{
    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PC14_FUNC_CTL_UART3_RXD;
    HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PC15_FUNC_CTL_UART3_TXD;
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_UART3_DE;

    clock_add_to_group(SENSORSCICLK, 0);

    uart_config_t config = {0};
    uart_default_config(SENSORSCI, &config);
    config.baudrate = 921600U;//460800U;
    config.fifo_enable = true;
    config.src_freq_in_hz = clock_get_frequency(SENSORSCICLK);
    config.rx_fifo_level = uart_fifo_16_bytes;
    config.tx_fifo_level = uart_fifo_16_bytes;

    uart_init(SENSORSCI, &config);

    //memset(&Sensor,0,sizeof(SENSOR_Type));
    Sensor.Initflag = 1;
}

//计算CRC校验，最终结果为实际CRC结果，发送时请颠倒
uint16_t CRC16_Tor(uint8_t *msg , uint16_t len)
{
	SENSOR_WORD crc;
	uint8_t index;

	crc.all = 0xFFFF;
	while(len--)
    {
		index = crc.Byte.Low ^ *msg++;
		crc.all = crc_table_Tor[index] ^ (crc.Byte.High);
	}
	return crc.all;
}

//力矩传感器值置零处理
void TorSensorSetZeroPrc(void)
{
    if(RamAid->FaultReset == 10)
    {
        DPI_TorSensorOffsetVal = Sensor.TorqueOrg;
        Sensor.TorqueOffset    = Sensor.TorqueOrg;
        IIcInterfaceB((uint16_t *)(&DPI_TorSensorOffsetVal),2);//
        RamAid->FaultReset = 0;
    }

    if(Sensor.Initflag == 1){
    
        Sensor.TorqueOffset = DPI_TorSensorOffsetVal;
        Sensor.Initflag = 0;
    }
}

void Tor_TxRequest2K(void)
{
    SENSORSCI_TxData  = 0x02;
    Sensor.EncRxNum = 7;
}

void Tor_TxRequestSetEE2K(void)
{
    if(RamPC->TorSenFrq==0){//配置1.2K ADC采样率
        SENSORSCI_TxData  = 0x32;
        SENSORSCI_TxData  = 0x00;
        SENSORSCI_TxData  = 0x00;
        SENSORSCI_TxData  = 0x32;   
    }else{//配置2K ADC采样率
        SENSORSCI_TxData  = 0x32;
        SENSORSCI_TxData  = 0x00;
        SENSORSCI_TxData  = 0x01;
        SENSORSCI_TxData  = 0x33;   
    }

    Sensor.EncRxNum = 4;
}

void Tor_TxRequestReadEE2K(void)
{
    SENSORSCI_TxData  = 0xEA;
    SENSORSCI_TxData  = 0x00;
    SENSORSCI_TxData  = 0xEA;   

    Sensor.EncRxNum = 4;
}

void Tor_RxRequest2K(void)
{
    Uint8 temp,rxcnt;
    Uint8 crc = 0;
    
    temp = SENSORSCI_RXFIF0_NUM;
    Sensor.EncRxNum2 = temp;
       
    if(temp == Sensor.EncRxNum)
    {
        Sensor.NoAckTime = 0;

        for(rxcnt=0;rxcnt<Sensor.EncRxNum;rxcnt++)
        {
            SensorData[rxcnt] = SENSORSCI_RxData;
        }

        for(rxcnt=0;rxcnt<Sensor.EncRxNum-1;rxcnt++)
        {
            crc ^= SensorData[rxcnt];
        }
    
        if(crc == SensorData[Sensor.EncRxNum - 1]) 
        {
            if(SensorData[0]==0x02){//返回帧是 力矩值返回帧
                
                TYPE_FLOAT GetFloat;

                GetFloat.Byte[0] = SensorData[2];
                GetFloat.Byte[1] = SensorData[3];
                GetFloat.Byte[2] = SensorData[4];
                GetFloat.Byte[3] = SensorData[5];

                Sensor.TorqueOrg = GetFloat.F * Sensor.ToreGainf;
                ActualTorqueClose = -(Sensor.TorqueOrg - Sensor.TorqueOffset);
                TorQueData = (int32)(ActualTorqueClose * 100.0f);
                TorFbFlag = 1;

            }else if(SensorData[0]==0xEA)
            {//返回帧是 ADC配置值返回帧
            
                if(SensorData[2]==0x00&&RamPC->TorSenFrq==0){//力矩传感器配置值成功，且与驱动器中设定值一致
                    Sensor.SetADCflag = 3;
                }else if(SensorData[2]==0x01&&RamPC->TorSenFrq==1){
                    Sensor.SetADCflag = 3;
                }else{
                    Sensor.SetADCflag = 1;
                }
            
            }else if(SensorData[0]==0x32)
            {//返回帧是 ADC配置成功状态返回帧
            
                Sensor.SetADCflag = 2;
            }

            Sensor.ErrCRCcnt = 0;
        }
        else
        {
            Sensor.ErrCRCcnt++;
            if(Sensor.ErrCRCcnt >= 60 && Sensor.SetADCflag > 2)//if(rxcrcerrcnt >= 10)
            {
                FaultPrtt_FaultInterface(TorSorErr);
                Sensor.ErrCRCcnt = 0;
                Sensor.CrcErrFlag = 1;
            }
        }
    }
    else
    {

        SENSORSCI_ResetFIFO;//清除FIFO
        SENSORSCI_R_STUATS;

        Sensor.NoAckTime++;
        if(Sensor.NoAckTime >= 60 && Sensor.SetADCflag > 2)//if(rxnoackcnt >= 10)
        {
            FaultPrtt_FaultInterface(TorSorErr);
            Sensor.NoAckTime = 0;
            Sensor.NoAckFlag = 1;
        }  
    }
}

void Tor_SensorUart_PerMod(void)
{
    uint8_t Status;

    static uint16_t  Rx_NumLast = 0;
    static uint16_t  Num_Old = 0;
    static uint16_t  Rx_LossCnt = 0,CrcErrCnt;

    SENSOR_WORD crc;
    uint16_t Crc_Check = 0;
    uint16_t Num = 0;

    Status = Sensor.Step;

    switch(Status)
    {
        case 0:
            /*
            SENSORSCI_TxData = 0x01;//1000hz主动上传命令
            SENSORSCI_TxData = 0x10;
            SENSORSCI_TxData = 0x01;    
            SENSORSCI_TxData = 0x9A;
            SENSORSCI_TxData = 0x00;
            SENSORSCI_TxData = 0x01;
            SENSORSCI_TxData = 0x02;
            SENSORSCI_TxData = 0x00;
            SENSORSCI_TxData = 0x03;
            SENSORSCI_TxData = 0xEB;
            SENSORSCI_TxData = 0x6B;
            */

            if(RamPC->TorSenFrq == 1)
            {
                SENSORSCI_TxData = 0x01;//1000hz主动上传命令
                SENSORSCI_TxData = 0x10;
                SENSORSCI_TxData = 0x01;    
                SENSORSCI_TxData = 0x9A;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x6A;
                SENSORSCI_TxData = 0xAA;
            }
            else if(RamPC->TorSenFrq == 2)
            {
                SENSORSCI_TxData = 0x01;//2000hz主动上传命令
                SENSORSCI_TxData = 0x10;
                SENSORSCI_TxData = 0x01;    
                SENSORSCI_TxData = 0x9A;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x2A;
                SENSORSCI_TxData = 0xAB;
            }
            else if(RamPC->TorSenFrq == 3)
            {
                SENSORSCI_TxData = 0x01;//4000hz主动上传命令
                SENSORSCI_TxData = 0x10;
                SENSORSCI_TxData = 0x01;    
                SENSORSCI_TxData = 0x9A;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x03;
                SENSORSCI_TxData = 0xEB;
                SENSORSCI_TxData = 0x6B;
            }
            else if(RamPC->TorSenFrq == 4)
            {
                SENSORSCI_TxData = 0x01;//8000hz主动上传命令
                SENSORSCI_TxData = 0x10;
                SENSORSCI_TxData = 0x01;    
                SENSORSCI_TxData = 0x9A;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x04;
                SENSORSCI_TxData = 0xAA;
                SENSORSCI_TxData = 0xA9;
            }
            else
            {
                SENSORSCI_TxData = 0x01;//4000hz主动上传命令
                SENSORSCI_TxData = 0x10;
                SENSORSCI_TxData = 0x01;    
                SENSORSCI_TxData = 0x9A;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x01;
                SENSORSCI_TxData = 0x02;
                SENSORSCI_TxData = 0x00;
                SENSORSCI_TxData = 0x03;
                SENSORSCI_TxData = 0xEB;
                SENSORSCI_TxData = 0x6B;
            }


            SENSORSCI_ResetFIFO;//清除FIFO
            SENSORSCI_R_STUATS;
            Sensor.TorqueOffset = DPI_TorSensorOffsetVal;
            Status = 1;
        break;
        case 1:
            Num =  SENSORSCI_RXFIF0_NUM;//当前接收的数据

            if((Num == Num_Old) && (Num != 0))
            {
                Rx_NumLast++;
                //if(Rx_NumLast == 2)
                {
                    if(Num == 8)
                    {
                        SensorData[0] = SENSORSCI_RxData;
                        SensorData[1] = SENSORSCI_RxData;

                        SensorData[2] = SENSORSCI_RxData;
                        SensorData[3] = SENSORSCI_RxData;
                        SensorData[4] = SENSORSCI_RxData;
                        SensorData[5] = SENSORSCI_RxData;

                        SensorData[6] = SENSORSCI_RxData;
                        SensorData[7] = SENSORSCI_RxData;

                        Crc_Check = CRC16_Tor(SensorData,6);
                        crc.Byte.Low = SensorData[6];
                        crc.Byte.High = SensorData[7];

                        if(Crc_Check == crc.all)
                        {
                            TYPE_FLOAT GetFloat;

            				GetFloat.Byte[0] = SensorData[5];
            				GetFloat.Byte[1] = SensorData[4];
            				GetFloat.Byte[2] = SensorData[3];
            				GetFloat.Byte[3] = SensorData[2];
				
                            Sensor.TorqueOrg = GetFloat.F;
                            ActualTorqueClose = -(Sensor.TorqueOrg - Sensor.TorqueOffset);
                            TorQueData = (int32)(ActualTorqueClose * 100.0f);
                            TorFbFlag = 1;
                            CrcErrCnt = 0;
                        }
                        else
                        {
                            CrcErrCnt++;
                            if(CrcErrCnt > 10)
                            {
                                if(RamPC->TorFullSel == 2) //力矩闭环
                                FaultPrtt_FaultInterface(TorSorErr);
                            }
                        }
                    }
                    else if(Num == 14)
                    {
                        TYPE_FLOAT GetFloat;

                        SensorData[0] = SENSORSCI_RxData;
                        SensorData[1] = SENSORSCI_RxData;
                        SensorData[2] = SENSORSCI_RxData;
                        SensorData[3] = SENSORSCI_RxData;
                        SensorData[4] = SENSORSCI_RxData;
                        SensorData[5] = SENSORSCI_RxData;
                        SensorData[6] = SENSORSCI_RxData;
                        SensorData[7] = SENSORSCI_RxData;

                        SensorData[8]  = SENSORSCI_RxData;
                        SensorData[9]  = SENSORSCI_RxData;
                        SensorData[10] = SENSORSCI_RxData;
                        SensorData[11] = SENSORSCI_RxData;
                        SensorData[12] = SENSORSCI_RxData;
                        SensorData[13] = SENSORSCI_RxData;

                        if(SensorData[0] == 0x4E)
                        {
                            Crc_Check = CRC16_Tor(SensorData,6);
                            crc.Byte.Low = SensorData[6];
                            crc.Byte.High = SensorData[7];

                            GetFloat.Byte[0] = SensorData[5];
            				GetFloat.Byte[1] = SensorData[4];
            				GetFloat.Byte[2] = SensorData[3];
            				GetFloat.Byte[3] = SensorData[2];
                        }
                        else
                        {
                            Crc_Check = CRC16_Tor(&SensorData[6],6);
                            crc.Byte.Low = SensorData[12];
                            crc.Byte.High = SensorData[13];

                            GetFloat.Byte[0] = SensorData[11];
            				GetFloat.Byte[1] = SensorData[10];
            				GetFloat.Byte[2] = SensorData[9];
            				GetFloat.Byte[3] = SensorData[8];
                        }

                        if(Crc_Check == crc.all)
                        {
                            Sensor.TorqueOrg = GetFloat.F;
                            ActualTorqueClose = -(Sensor.TorqueOrg - Sensor.TorqueOffset);
                            TorQueData = (int32)(ActualTorqueClose * 100.0f);
                            TorFbFlag = 1;
                            CrcErrCnt = 0;
                        }
                        else
                        {
                            CrcErrCnt++;
                            if(CrcErrCnt > 10)
                            {
                                if(RamPC->TorFullSel == 2) //力矩闭环
                                FaultPrtt_FaultInterface(TorSorErr);
                            }
                        }
                    }
                    else
                    {
                        SENSORSCI_ResetFIFO;//清除FIFO
                    }
                    Rx_NumLast = 0;
                } 
                Rx_LossCnt = 0; 
                Sensor.NoAckTime = 0;            
            }
            else
            {
                Rx_LossCnt++;
                if(Rx_LossCnt > 160)
                {
                    Rx_LossCnt = 0;
                    Sensor.NoAckTime++;
                    if(Sensor.NoAckTime > 10)
                    {
                        Status = 5;
                        if(RamPC->TorFullSel > 0) //力矩闭环
                        FaultPrtt_FaultInterface(TorSorErr);
                    }
                }
            }
            Num_Old = Num;
        break;
        case 5:
        break;
        default:break;
    }
    Sensor.Step = Status;

    TorSensorSetZeroPrc();
}

void TorGetVersion_Tx(void)
{
    SENSORSCI_TxData = 0x01;
    SENSORSCI_TxData = 0x2B;
    SENSORSCI_TxData = 0x0E;
    SENSORSCI_TxData = 0x01;
    SENSORSCI_TxData = 0x00;
    SENSORSCI_TxData = 0x70;
    SENSORSCI_TxData = 0x77;
}

uint16_t TorGetVersion_Rx(void)
{
    uint8_t Num,rxcnt;
    uint16_t crc,crc1;

    Num =  SENSORSCI_RXFIF0_NUM;//当前接收的数据 
    if(Num > 0)
    {
        for(rxcnt=0;rxcnt < Num;rxcnt++)
        {
            SensorData[rxcnt] = SENSORSCI_RxData;
        }

        crc = ((uint16_t)SensorData[Num - 1] << 8) + SensorData[Num - 2];
        crc1 = CRC16_Tor(SensorData,29);
        if(crc1 == crc) 
        {
            Sensor.Vendor  = 0;
            Sensor.ProId   = (SensorData[18] - 0x30) * 100;
            Sensor.ProId  += (SensorData[19] - 0x30) * 10;
            Sensor.ProId  += (SensorData[20] - 0x30);

            Sensor.Version  = (SensorData[24] - 0x30) * 100;
            Sensor.Version += (SensorData[26] - 0x30) * 10;
            Sensor.Version += (SensorData[28] - 0x30);
        }
        else
        {
            return 1;
        }
    }
    else
    {
        SENSORSCI_ResetFIFO;//清除FIFO
        SENSORSCI_R_STUATS;
        return 1;
    }

    return 0;
}

void Tor_Sensor_Precess(void)
{
    uint16_t temp;

    switch (Sensor.Step)
    {
        case 0:
            TorGetVersion_Tx();
            Sensor.DelayCnt = 0;
            Sensor.Step = 1;
        break;
        case 1:
            Sensor.DelayCnt++;

            if(Sensor.DelayCnt >= 100)
            {
               if(RamMotor->TorGain == 0) Sensor.ToreGainf = 1.0f;
               else Sensor.ToreGainf = RamMotor->TorGain / 100.0f;

               Sensor.DelayCnt = 0;
               Sensor.Step = 2;
            }
        break;
        case 2:
            temp = TorGetVersion_Rx();
            if(temp == 0)
            {
                Sensor.DelayCnt = 0;
                Sensor.Step = 3;
            }
            else
            {
                Sensor.Errcnt++;
                Sensor.Step = 0;
                if(Sensor.Errcnt > 10)
                {
                    Sensor.Step = 3;
                    FaultPrtt_FaultInterface(TorSorErr);
                }
            }
        break;
        case 3:
            Sensor.DelayCnt++;
            if(Sensor.DelayCnt >= 4)
            {
                Tor_RxRequest2K();//
                Tor_TxRequest2K();
                TorSensorSetZeroPrc();
                Sensor.DelayCnt = 0;
            }
        break;
        default:break;
    }    
}


//void Tor_Sensor_Precess(void)
//{
//    if(Sensor.DelayCnt<64){
//        Sensor.DelayCnt++;
//    }

//    switch (Sensor.Step)
//    {
//        case 0:
//            if(Sensor.DelayCnt>=10)
//            {
//               Sensor.SetADCflag=3;
//               // Tor_RxRequest2K();
//               // Tor_TxRequestReadEE2K();//第一步读取力矩传感器中的ADC采样率与驱动器参数是否匹配，若是匹配则切换到正常读取力矩值模式
//                Sensor.DelayCnt = 0;//若是不匹配，则进入ADC配置通信帧模式

//                if(Sensor.DelayCnt2<65534){
//                    Sensor.DelayCnt2++;
//                }
//            }

//            if(Sensor.DelayCnt2<2000){
//                if(Sensor.SetADCflag==3){
//                    Sensor.Step = 2;
//                    Sensor.DelayCnt2 = 0;
//                }
//                else if(Sensor.SetADCflag==1){
//                    Sensor.Step = 1;
//                    Sensor.DelayCnt2 = 0;
//                }
//            }else{
//                FaultPrtt_FaultInterface(TorSorErr);
//                Sensor.NoAckFlag = 1;
//            }

//        break;
//        case 1:
//            if(Sensor.DelayCnt>=10)
//            {
//                Tor_RxRequest2K();
//                //Tor_TxRequestReadEE2K();
//                Tor_TxRequestSetEE2K();
//                Sensor.DelayCnt = 0;

//                if(Sensor.DelayCnt2<65534){
//                    Sensor.DelayCnt2++;
//                }
//            }

//            if(Sensor.DelayCnt2<2000){
//                if(Sensor.SetADCflag==2){
//                    Sensor.Step = 2;
//                    Sensor.DelayCnt2 = 0;
//                }
//            }else{
//                FaultPrtt_FaultInterface(TorSorErr);
//                Sensor.NoAckFlag = 1;
//            }

//        break;
//        case 2:
//            if(Sensor.DelayCnt>=3)
//            {
//                Tor_RxRequest2K();//
//                Tor_TxRequest2K();
//                TorSensorSetZeroPrc();
//                Sensor.DelayCnt = 0;
//            }

//        break;
//        default:break;
//    }    
//}























