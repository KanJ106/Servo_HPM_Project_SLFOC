
#include <string.h>
#include "SV_CanFirm.h"
#include "SV_CanbusCtrl.h"//
#include "r_can_api.h"
#include "ota.h"
#include "s_sys_init.h"
#include "SV_Scope.h"

#if SERVOTYPE == SERVO_CAN
TYPE_CanbusComm    CM_BusVar;
can_Sdo_Rx         SdoRxdata;
can_Sdo_Tx         SdoTxdata;

TYPE_OTA  CanOta = {0,0,0,0,0,0,0,0,0,{0},{0},{0}};

extern uint16_t* const CanBusConTrolAddres[100];
extern TYPE_AttRI AttCode[100];
extern uint16_t SmPeriodtime;
extern uint16_t SmDcOffsettime;

uint8_t SalveId = 0;
uint8_t CANFDSalveId = 0;
uint32_t FIFO_Index = 0; //此刻FIFO的索引
float output;

uint8_t CAN_RxData_To_CtrlBuff(TYPE_CtrlWord *msg, const mcan_rx_message_t *rx_msg) 
{
    uint8_t id;
    uint16_t temp;
    uint8_t Len = rx_msg->dlc;
    
    if(Len >= 0x02)
    {
        id = rx_msg->data_8[0];
        if(id == RamCommu->CanID || id == 0) //判断是否为该从机的ID或广播
        {
            temp = rx_msg->data_8[1];
            msg->Byte = temp;
            return 1;
        }
    }
    return 0;
}

void CAN_RxData_To_Sdo(can_Sdo_Rx *msg, const mcan_rx_message_t *rx_msg)
{
    udata_t temp;
    
    msg->Cmd = rx_msg->data_8[0];
    temp.BYTE[0] = rx_msg->data_8[1]; 
    temp.BYTE[1] = rx_msg->data_8[2];
    msg->Addres = temp.Word[0];
    msg->Len = rx_msg->data_8[3];
    msg->Data.LONG = rx_msg->data_32[1];//rx_msg->data_8[7] << 24 | rx_msg->data_8[6] << 16 | rx_msg->data_8[5] << 8 | rx_msg->data_8[4];

    msg->Num = rx_msg->dlc;  //此处为数据帧中数据的字节数

}
uint8_t CAN_RxData_To_PvtCtrl(can_rx_Syncctrl *msg, const mcan_rx_message_t *rx_msg)
{
    uint8_t Len = rx_msg->dlc;
    
    if(Len == 0x04 || Len == 0x06 || Len == 0x08)
    {
        msg->Pos = rx_msg->data_32[0];
        msg->Vel = rx_msg->data_8[5] << 8 | rx_msg->data_8[4];
        msg->Tor = rx_msg->data_8[7] << 8 | rx_msg->data_8[6];

        return 1;
    }
    return 0;
}
//mqb CANFD位置同步模式
uint8_t CANFD_RxData_To_PvtCtrl(can_rx_Syncctrl *msg, const mcan_rx_message_t *rx_msg) //mqb同步位置模式
{
    uint8_t Len = rx_msg->dlc;

    uint8_t msgid  = rx_msg->std_id;
    bool flag = (8 == Len || 10 == Len || 12 == Len || 13 == Len \
                || 14 == Len || 15 == Len && 0x50 == msgid && \
                CANFDSalveId >= 1 && CANFDSalveId <= 8) ? true : false;
    if(flag)
    {
        uint8_t pvtdata[8] = {0};
        uint8_t startpos = (CANFDSalveId - 1) * 8;
        for(uint8_t i = 0; i < 8; i++)
        {
            pvtdata[i] = rx_msg->data_8[startpos + i];
        }
        msg->Pos = pvtdata[3] << 24 | pvtdata[2] << 16 | pvtdata[1] << 8 | pvtdata[0];
        msg->Vel = pvtdata[5] << 8 | pvtdata[4];
        msg->Tor = pvtdata[7] << 8 | pvtdata[6];

        return 1;       
    }
    return 0;
}

uint8_t CAN_RxData_To_PosCtrl(can_rx_posctrl *msg, const mcan_rx_message_t *rx_msg)
{
    uint8_t Len = rx_msg->dlc;
    
    if(Len >= 0x07)
    {
        msg->Pos = rx_msg->data_32[0];
        msg->Vel = rx_msg->data_8[5] << 8 | rx_msg->data_8[4];
        msg->Mode = rx_msg->data_8[6];
        msg->Flag = 1;

        return 1;
    }
    return 0;
}

uint8_t CAN_RxData_To_VelCtrl(can_rx_velctrl *msg, const mcan_rx_message_t *rx_msg)
{
    uint8_t Len = rx_msg->dlc;
    
    if(Len >= 0x02)
    {
        msg->Vel = rx_msg->data_8[1] << 8 | rx_msg->data_8[0];

        return 1;
    }
    return 0;
}

uint8_t CAN_RxData_To_TorCtrl(can_rx_torctrl *msg, const mcan_rx_message_t *rx_msg)
{
    uint8_t Len = rx_msg->dlc;
    
    if(Len >= 0x02)
    {
        msg->Torque = rx_msg->data_8[1] << 8 | rx_msg->data_8[0]; 

        return 1;     
    }                 
    return 0;         
}

float MitUintToFloat(int x_int, float x_min, float x_max, int bits) {
     float span = x_max- x_min;
     float offset = x_min;
     return ((float)x_int) * span / ((float)((1 << bits)- 1)) + offset;
}

uint8_t CAN_RxData_To_MITCtrl(can_rx_mitctrl *msg, const mcan_rx_message_t *rx_msg)
{
    uint8_t Len = rx_msg->dlc;
    uint8_t msgid  = rx_msg->std_id;
    bool flag = (8 == Len || 10 == Len || 12 == Len || 13 == Len \
                || 14 == Len || 15 == Len && 0x20 == msgid && \
                CANFDSalveId >= 1 && CANFDSalveId <= 8) ? true : false;
    if(flag)
    {
        //根据从机ID计算从哪里开始取数据
        uint8_t mitdata[8] = {0};
        uint8_t startpos = (CANFDSalveId - 1) * 8;
        for(uint8_t i = 0; i < 8; i++)
        {
            mitdata[i] = rx_msg->data_8[startpos + i];
        }

        msg->uPos = (mitdata[0] << 8) | mitdata[1];
        msg->uVel = (mitdata[2] << 4) | (mitdata[3] >> 4 & 0xf);
        msg->uKp  = ((mitdata[3] & 0xf) << 8) | mitdata[4];
        msg->uKd  = (mitdata[5] << 4) | (mitdata[6] >> 4 & 0xf);
        msg->uTor = ((mitdata[6] & 0xf) << 8) | mitdata[7];

        msg->Pos = msg->uPos - MitMedPos;
        msg->Vel = msg->uVel - MitMedSpd;
        msg->Kp  = msg->uKp  - MitMedKp ;
        msg->Kd  = msg->uKd  - MitMedKd ;
        msg->Tor = msg->uTor - MitMedTor; 

        msg->fPos = CM_BusCtrl.MITPos_CmdCoeff * msg->Pos;
        msg->fVel = CM_BusCtrl.MITSpd_CmdCoeff * msg->Vel;
        msg->fKp  = CM_BusCtrl.MITKp_CmdCoeff  * msg->Kp ;
        msg->fKd  = CM_BusCtrl.MITKd_CmdCoeff  * msg->Kd ;
        msg->fTor = CM_BusCtrl.MITTor_CmdCoeff * msg->Tor;

        return 1;       
    }
    return 0;
}


void CAN_TxData_To_EmergencyBuff(uint16_t Errcode, mcan_tx_frame_t *tx_msg)
{
    memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
    tx_msg->std_id = 0x80 + SalveId;  
    tx_msg->canfd_frame = 1;
    tx_msg->bitrate_switch = 1;
    tx_msg->dlc = 2;
    tx_msg->data_8[0] = Errcode & 0xff;
    tx_msg->data_8[1] = Errcode >> 8;
}

void CAN_TxData_To_SdoBuff(can_Sdo_Tx *msg, mcan_tx_frame_t *tx_msg)
{
    memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
    tx_msg->std_id = 0x180 + SalveId;
    tx_msg->canfd_frame = 1;
    tx_msg->bitrate_switch = 1;
    tx_msg->dlc = 8;
    
    if(msg->ErrFlag > 0)
    {
        tx_msg->data_8[0] = 0x80;
        tx_msg->data_8[1] = msg->ErrFlag;
        tx_msg->data_8[2] = 0;
        tx_msg->data_8[3] = 0;
        tx_msg->data_32[1] = 0;
        msg->ErrFlag = 0;
    }
    else
    {
        tx_msg->data_8[0] = msg->Cmd;
        tx_msg->data_8[1] = (uint8_t)(msg->Addres & 0xff);
        tx_msg->data_8[2] = (uint8_t)(msg->Addres >> 8);
        tx_msg->data_8[3] = msg->Len;
        tx_msg->data_32[1] = msg->Data.LONG;
    }

}


//发送反馈帧1
void CAN_TxData_To_FbBuff(can_tx_fb *msg, mcan_tx_frame_t *tx_msg)
{
    float SenValue;
    uint16_t SpdVel;

    memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
    tx_msg->std_id = 0x480 +SalveId;
    tx_msg->canfd_frame = 1;
    tx_msg->bitrate_switch = 1;
    tx_msg->data_32[0] = msg->Pos;

    SpdVel = (uint16_t)msg->Vel;
    tx_msg->data_8[4] = SpdVel & 0xff;
    tx_msg->data_8[5] = (SpdVel >> 8) & 0xff;
    
    uint16_t temp = (uint16_t)(msg->Torque << 3);
    temp |= (msg->State << 1);
    temp |= msg->Enable;
    tx_msg->data_8[6] = temp & 0xff;
    tx_msg->data_8[7] = (temp >> 8) & 0xff;

    if(RamCommu->CanMode == 1)
    {
        tx_msg->dlc = 9;
        SenValue = msg->TorSenValue;
        memcpy(&tx_msg->data_8[8],&SenValue,4);
    }
    else
    {
        tx_msg->dlc = 8;
    }
}

//发送反馈帧2
void CAN_TxData_To_ReqBuff(can_tx_req *msg, mcan_tx_frame_t *tx_msg)
{

    udata_t data;
    memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
    tx_msg->std_id = 0x500 + SalveId;
    tx_msg->canfd_frame = 1;
    tx_msg->bitrate_switch = 1;
    tx_msg->dlc = 8;
    data.LONG = msg->Pos;
    tx_msg->data_32[0] = msg->Pos;

    data.Word[0] = msg->Vel;
    data.Word[1] = msg->Torque;
    tx_msg->data_32[1] = data.LONG;
}

//发送反馈帧3
void CAN_TxData_To_StatusBuff(can_tx_Status *msg, mcan_tx_frame_t *tx_msg)
{

    udata_t data;
    memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
    tx_msg->std_id = 0x580 + SalveId;
    tx_msg->canfd_frame = 1;
    tx_msg->bitrate_switch = 1;
    tx_msg->dlc = 8;

    tx_msg->data_32[0] = msg->PosErr;
    
    data.Word[0] = msg->Flag;
    data.Word[1] = msg->Errcode;
    tx_msg->data_32[1] = data.LONG;
}

int MitFloatToUint(float x, float x_min, float x_max, int bits) {
     float span = x_max- x_min;
     float offset = x_min;
     return (int)((x- offset) * ((float)((1 << bits)- 1)) / span);
}
//MIT反馈帧
/******************************************************************************
                     主站侧解析值
*位置：(接收帧 - 32768) * 6280.0f / 32768.0f = 输出侧当前位置
*速度：(接收帧 - 2048) * 50.f / 2048.0f = 输出侧的当前速度
*力矩：(接收帧 - 2048) * 600.0f / 2048 = 输出侧的当前力矩

                     从站侧发送值
*位置：CM_BusVar.MitFb.FbfPos * 32768.0f / 6280.0f + 32768 = 发送的当前位置
*速度：CM_BusVar.MitFb.FbfVel * 2048.0f / 50.0f + 2048 = 发送的当前速度
*力矩：CM_BusVar.MitFb.FbfTor * 2048.0f / 600.0f + 2048 = 发送的当前力矩
******************************************************************************/
void CAN_TxData_To_MITBuff(can_mit_fb *msg, mcan_tx_frame_t *tx_msg)
{

    if(CANFDSalveId >= 1 && CANFDSalveId <= 8)
    {
        msg->Pos = (int16_t)(CM_BusVar.MitFb.FbfPos * CM_BusCtrl.MITFbPos_CmdCoeff); //mqb
        msg->Motor_MitPos = msg->Pos + MitMedPos;                                                       
                                                                                                
        msg->Vel = (int16_t)(CM_BusVar.MitFb.FbfVel * CM_BusCtrl.MITFbSpd_CmdCoeff); //mqb
        msg->Motor_MitVel = msg->Vel + MitMedSpd;

        msg->Tor = (int16_t)(CM_BusVar.MitFb.FbfTor * CM_BusCtrl.MITFbTor_CmdCoeff);
        msg->Motor_MitTor = msg->Tor + MitMedTor;

        memset(tx_msg, 0, sizeof(mcan_tx_frame_t));
        tx_msg->std_id = SalveId;
        tx_msg->canfd_frame = 1;
        tx_msg->bitrate_switch = 1;
        tx_msg->dlc = 8;

        tx_msg->data_8[0] = (msg->Motor_MitPos >> 8) & 0xff;
        tx_msg->data_8[1] = msg->Motor_MitPos & 0xff;
        tx_msg->data_8[2] = (msg->Motor_MitVel >> 4) & 0xff;
        tx_msg->data_8[3] = ((msg->Motor_MitVel & 0xf) << 4) | ((msg->Motor_MitTor >> 8) & 0xf);
        tx_msg->data_8[4] = msg->Motor_MitTor & 0xff;
        tx_msg->data_8[5] = msg->State;
        tx_msg->data_8[6] = msg->ErrorCode & 0xff;
        tx_msg->data_8[7] = msg->ErrorCode >> 8;
    }
    else return;
}

void Sdo_Service(can_Sdo_Rx *rx,can_Sdo_Tx *tx)
{
    uint16_t Addres;
    uint16_t *Word;
    uint32_t *DWord;
    TYPE_AttRI temp;  
    
    if(rx->Num < 4)
    {
        tx->ErrFlag = 5;
        return;    
    }  
    
    Addres = rx->Addres;

    if(Addres < 0x0DFF)
    {
        uint32_t temp2;

        if(rx->Cmd == 0)//写
        {
            tx->ErrFlag = Can_Func_Cacl(rx->Addres,rx->Len,rx->Cmd,rx->Data.LONG,&temp2);

            tx->Addres = Addres;
            tx->Cmd = rx->Cmd;
            tx->Len = rx->Len;
            tx->Data.LONG = rx->Data.LONG;
        }
        else if(rx->Cmd == 1)//读
        {
            tx->ErrFlag = Can_Func_Cacl(rx->Addres,rx->Len,rx->Cmd,0,&temp2);

            tx->Addres = Addres;
            tx->Cmd = rx->Cmd;
            tx->Len = rx->Len;
            tx->Data.LONG = temp2;
        }
        else
        {
            tx->ErrFlag = 2;
            return;
        }
    }
    else if(Addres >= 0x1A00 && Addres <= 0x1A63)
    {
        Word = CanBusConTrolAddres[Addres - 0x1A00];  
        DWord = (uint32_t *)Word;        
        temp = AttCode[Addres - 0x1A00];   
        
        if((rx->Cmd == 0) && (temp.RW == 0)) //写
        {
            if((rx->Len == 0x02) && (temp.Size == 0x02)) //16bit
            {
                if(rx->Num < 6)
                {
                    tx->ErrFlag = 5;
                    return;    
                }
            
                if(temp.Type == 0x00) //正
                {
                    if((rx->Data.Word[0] <= (uint16_t)temp.Max) && (rx->Data.Word[0] >= (uint16_t)temp.Min))
                    {
                        *Word = rx->Data.Word[0];
                    }
                    else
                    {
                        tx->ErrFlag = 4;
                        return;
                    }                
                }
                else
                {
                    if(((int16_t)rx->Data.Word[0] <= (int16_t)temp.Max) && ((int16_t)rx->Data.Word[0] >= (int16_t)temp.Min))
                    {
                        *Word = rx->Data.Word[0];
                    }
                    else
                    {
                        tx->ErrFlag = 4;
                        return;
                    }
                }
            }
            else if((rx->Len == 0x04) && (temp.Size == 0x04)) //32bit
            {
                if(rx->Num < 8)
                {
                    tx->ErrFlag = 5;
                    return;    
                }
            
                if(temp.Type == 0x00) //正
                {
                    uint32_t data32;
                    data32 = (uint32_t)rx->Data.LONG;
                    if((data32 <= (uint32_t)temp.Max) && (data32 >= (uint32_t)temp.Min))
                    {
                        *DWord = data32;
                    }
                    else
                    {
                        tx->ErrFlag = 4;
                        return;
                    }                 
                }
                else
                {
                    int32_t data32;
                    data32 = (int32_t)rx->Data.LONG;
                    if((data32 <= (int32_t)temp.Max) && (data32 >= (int32_t)temp.Min))
                    {
                        *DWord = data32;
                    }
                    else
                    {
                        tx->ErrFlag = 4;
                        return;
                    }   
                }
            }
            else
            {
                tx->ErrFlag = 3;
                return;
            }  
        
            tx->Addres = Addres;
            tx->Cmd = rx->Cmd;
            tx->Len = rx->Len;
            tx->Data.LONG = rx->Data.LONG;
        }
        else if(rx->Cmd == 1) //读
        {
            Addres = rx->Addres;
        
            tx->Addres = Addres;
            tx->Cmd = rx->Cmd;
            tx->Len = rx->Len;
        
            if((rx->Len == 0x02) && (temp.Size == 0x02))
            {
                tx->Data.Word[0] = *Word;
                tx->Data.Word[1] = 0;
            }
            else if((rx->Len == 0x04) && (temp.Size == 0x04))
            {
                tx->Data.LONG = *DWord; 
            }
            else
            {
                tx->ErrFlag = 3;
                return;
            }       
        }
        else
        {
            tx->ErrFlag = 2;
            return;
        }     
    }
    else
    {
        tx->ErrFlag = 1;
        return;
    }
}

uint8_t CanTxFlag[8] = {0,0,0,0,0,0,0,0};
//#define EMCTXBuff     0
//#define SDOTXBuff     1
//#define FBTXBuff      2
//#define ReQTXBuff     3
//#define STATUSTXBuff  4
//#define MITTXBuff     5
void Can_RxProcess(mcan_rx_message_t *rx_msg)
{

    uint32_t CanRxFlag = 0;
    uint8_t Txflag = 0;

    CanRxFlag = RXflag;
    //Ctrl
    if(CanRxFlag & 0x01) {
        MCAN_Read_Rxbuff(CANSelect, 0, rx_msg);
        RXflag = 0x01;    
        CAN_RxData_To_CtrlBuff(&CM_BusVar.CtrlWord, rx_msg); 
    }
    //Sdo
    if(CanRxFlag & 0x02) {
        MCAN_Read_Rxbuff(CANSelect, 1, rx_msg);
        RXflag = 0x02;    
        CAN_RxData_To_Sdo(&SdoRxdata, rx_msg);

        Sdo_Service(&SdoRxdata, &SdoTxdata);   
        CanTxFlag[SDOTXBuff] = 1;
    }
    //PVT
    if(CanRxFlag & 0x04) { 
        SmPeriodtime = (uint16_t)ReadCount();
        ResetCount();
        MCAN_Read_Rxbuff(CANSelect, 2, rx_msg);
        RXflag = 0x04;  
        Txflag = CAN_RxData_To_PvtCtrl(&CM_BusVar.SyncTarget, rx_msg);
        CM_BusVar.SMCount = 0;
        if(RamPA->SyncFrameFbFlag == 1) CanTxFlag[FBTXBuff] = Txflag; 

    }
    //Pos
    if(CanRxFlag & 0x08) {
        MCAN_Read_Rxbuff(CANSelect, 3, rx_msg);
        RXflag = 0x08;
        Txflag = CAN_RxData_To_PosCtrl(&CM_BusVar.PosTarget, rx_msg);
        CanTxFlag[FBTXBuff] = Txflag;
    }
    //Vel
    if(CanRxFlag & 0x10) {
        MCAN_Read_Rxbuff(CANSelect, 4, rx_msg);
        RXflag = 0x10;
        Txflag = CAN_RxData_To_VelCtrl(&CM_BusVar.SpdTarget, rx_msg);
        CanTxFlag[FBTXBuff] = Txflag;
    }
    //Tor
    if(CanRxFlag & 0x20) {
        MCAN_Read_Rxbuff(CANSelect, 5, rx_msg);
        RXflag = 0x20;
        Txflag = CAN_RxData_To_TorCtrl(&CM_BusVar.TorTarget, rx_msg);
        CanTxFlag[FBTXBuff] = Txflag;
    }
    //请求反馈帧
    if(CanRxFlag & 0x40) {
        MCAN_Read_Rxbuff(CANSelect, 6, rx_msg);
        RXflag = 0x40;
        uint8_t temp = rx_msg->data_8[0];
        temp = (temp > 3) ? STATUSTXBuff : (temp + FBTXBuff);
        CanTxFlag[temp] = 1;
    }
    //MIT
    if(CanRxFlag & 0x80) {

        MCAN_Read_Rxbuff(CANSelect, 7, rx_msg);
        RXflag = 0x80;
        Txflag = CAN_RxData_To_MITCtrl(&CM_BusVar.MITTarget, rx_msg);
        CanTxFlag[MITTXBuff] = Txflag;     

    }
    //mqb PVT CANFD位置同步模式
    if(CanRxFlag & 0x100)
    {
        SmPeriodtime = (uint16_t)ReadCount();
        ResetCount();
        MCAN_Read_Rxbuff(CANSelect, 8, rx_msg);
        RXflag = 0x100;  
        Txflag = CANFD_RxData_To_PvtCtrl(&CM_BusVar.SyncTarget, rx_msg);
        CM_BusVar.SMCount = 0;
        if(RamPA->SyncFrameFbFlag == 1) CanTxFlag[FBTXBuff] = Txflag; 
    }
    //同步帧
    if(hpm_flag) {
        hpm_flag = false;
        if(RamPA->SyncFrameFbFlag == 2) CanTxFlag[FBTXBuff] = 1; 
    }

    //OTA指令
    if(CanRxFlag & 0x400)
    {
        MCAN_Read_Rxbuff(CANSelect, 10, rx_msg);
        RXflag = 0x400; 
        memcpy(&CanOta.FirmCode,&rx_msg->data_8[0],10);
        Can_Start_Process();
    }

    //故障触发发送指令
    if(CanRxFlag & 0x1000)
    {
        uint16_t temp;

        MCAN_Read_Rxbuff(CANSelect, 12, rx_msg);
        RXflag = 0x1000; 
        CanTxFlag[FAULTTXBuff] = 1;

        temp = (((uint16_t)rx_msg->data_8[1] << 8) + (rx_msg->data_8[0]));
        temp = (temp > 1023) ? 1023 : temp;//获得接收指令索引

        temp = (STR_Osci.StartIndex >> 2) + temp;
        CM_BusVar.FaultIndex = (temp > 1023) ? temp - 1024 :temp;
    }

    if(0 == CM_BusCtrl.Run)
    {
        CM_BusVar.SyncTarget.Tor = 0;
        CM_BusVar.SyncTarget.Vel = 0;
        CM_BusVar.SpdTarget.Vel = 0;
        CM_BusVar.TorTarget.Torque = 0;
        CM_BusVar.PosTarget.Vel = 0;
    }
    if(Radio_flag) {
        Radio_flag = false;
        SmPeriodtime = (uint16_t)ReadCount();
        ResetCount();
        MCAN_Read_Rxfifo(CANSelect, rx_msg);  
        Txflag = CANFD_RxData_To_PvtCtrl(&CM_BusVar.SyncTarget, rx_msg);
        CM_BusVar.SMCount = 0;
        if(RamPA->SyncFrameFbFlag == 1) CanTxFlag[FBTXBuff] = Txflag; 
    }
    
    if((CM_BusCtrl.ModbusSel == 0) && (CanRxFlag)) {
        CM_BusCtrl.ModbusSel = 1;
    }
}

void Can_TxCheckProcess(void)
{
    uint32_t flag;

    flag = CANSelect->CCCR;

    if(flag & 0x1) 
    {
        CANSelect->CCCR &= 0xFFFFFFFE;
    }    
}

static mcan_tx_frame_t tx_msgs[8];
void Can_TxProcess(void)
{
    //static mcan_tx_frame_t tx_msgs[8];
    hpm_stat_t status;
    //uint32_t put_index = 0;
    static uint16_t TxCnt = 0;
    static uint8_t ErrFirst = 0;
    static uint16_t Code = 0;

    if(CM_BusVar.Status.Errcode < Code) ErrFirst = 0;

    if((CM_BusCtrl.ModbusSel == 1) && (CM_BusVar.Status.Errcode > 0) && (ErrFirst == 0))   
    {
        CAN_TxData_To_EmergencyBuff(CM_BusVar.Status.Errcode, &tx_msgs[0]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[0]);
        if(status == status_success) {
            ErrFirst = 1;
            CanTxFlag[EMCTXBuff] = 0;
            TxCnt = 0;
        }
    }
    Code = CM_BusVar.Status.Errcode;

    if(CanTxFlag[SDOTXBuff] == 1) {
        CAN_TxData_To_SdoBuff(&SdoTxdata, &tx_msgs[SDOTXBuff]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[SDOTXBuff]);
        if(status == status_success) 
        {
            CanTxFlag[SDOTXBuff] = 0; 
            TxCnt = 0;
        }

    }
    
    if(CanTxFlag[FBTXBuff] == 1) {
        CAN_TxData_To_FbBuff(&CM_BusVar.ActualValue, &tx_msgs[FBTXBuff]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[FBTXBuff]);
        if(status == status_success) 
        {
            CanTxFlag[FBTXBuff] = 0;
            TxCnt = 0;
        }
    }
    
    if(CanTxFlag[ReQTXBuff] == 1) {
        CAN_TxData_To_ReqBuff(&CM_BusVar.DemandValue, &tx_msgs[ReQTXBuff]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[ReQTXBuff]);
        if(status == status_success) 
        {
            CanTxFlag[ReQTXBuff] = 0;
            TxCnt = 0;
        }
    }
    
    if(CanTxFlag[STATUSTXBuff] == 1) {
        CAN_TxData_To_StatusBuff(&CM_BusVar.Status, &tx_msgs[STATUSTXBuff]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[STATUSTXBuff]);
        if(status == status_success) 
        {
            CanTxFlag[STATUSTXBuff] = 0;
            TxCnt = 0;
        }
    }
    
    if(CanTxFlag[MITTXBuff] == 1) {
        CAN_TxData_To_MITBuff(&CM_BusVar.MitFb, &tx_msgs[MITTXBuff]);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[MITTXBuff]);
        if(status == status_success)
        {
            CanTxFlag[MITTXBuff] = 0;
            TxCnt = 0;
        }
        
    }
    if(CanTxFlag[FAULTTXBuff] == 1)
    {
        tx_msgs[FAULTTXBuff].std_id = 0x700 + SalveId;
        tx_msgs[FAULTTXBuff].canfd_frame = 1;
        tx_msgs[FAULTTXBuff].bitrate_switch = 1;
        tx_msgs[FAULTTXBuff].dlc = 15;
        memcpy(&tx_msgs[FAULTTXBuff].data_8[0],(&UNI_OsciBuffer.all_8Bits[CM_BusVar.FaultIndex << 6]),64);
        status = MCAN_Tranmit(CANSelect, &tx_msgs[FAULTTXBuff]);
        if(status == status_success)
        {
            CanTxFlag[FAULTTXBuff] = 0;
            TxCnt = 0;
        }
    }

    if(TxCnt++ > 10) {
        if(!(TX_Success & (0x1 << FIFO_Index))) 
        {
            Cancel_Send = 0x1 << FIFO_Index;
        }
        else {
            ;
        }
            
        TxCnt = 0;
    }


}

//1ms调用
void CanSyncCheck(void)
{
    uint16_t temp;
    
    if(CM_BusVar.CanState >= 2)
    {
        CM_BusVar.SyncCount++;
        if(CM_BusVar.SyncCount >= CM_BusVar.SyncValue)
        {
            //baocuo
            CM_BusVar.CanState = 1;
        }
        
        if(CM_BusVar.SMCount >= CM_BusVar.SMValue)
        {
            CM_BusVar.CanState = 1; 
        }
    }
    else
    {
        CM_BusVar.CanState = 1;
        CM_BusVar.SyncCount = 0;
        CM_BusVar.SMCount = 0;
        
        temp = RamCommu->SMLost;
        if(temp == 0) temp = 1; 
        CM_BusVar.SyncValue = (CM_BusVar.CanSyncTime * temp);
        CM_BusVar.SMValue = temp;
    } 
}

extern Uint16 CanAddrRangLit(Uint16 Functioncode);
extern uint16_t P0_PF_Write_Limit(uint16_t SCI_Limit_Addr, uint16_t *Limit_Data);
uint16_t Can_Func_Cacl(uint16_t Addres,uint16_t Len,uint16_t Rw,uint32_t WValue,uint32_t *RValue)
{
    uint16_t code,ret;
    Uint16 *FuncAddr;
    uint16_t rvalue;
    uint16_t wvalue;

    if(Len == 0x02)
    {
        code = CanAddrRangLit(Addres);
    }
    else if(Len == 0x04)
    {
        code = CanAddrRangLit(Addres + 1);
    }
    else
    {
        return 3;
    }

    FuncAddr = RamFuncStartAddr + GetFuncAddr(Addres);   //获取功能码的地址

    if(Rw == 1)//读
    {
        if(Len == 0x02)
        {
            rvalue = *FuncAddr;
            *RValue = (uint32_t)rvalue;
        }
        else
        {
            *RValue = *(uint32_t *)FuncAddr;
        }
    }
    else//写
    {
        if(Len == 0x02)
        {
            wvalue = (uint16_t)(WValue&0x0000FFFF);
            ret = P0_PF_Write_Limit(Addres, &wvalue);
            if(ret == 0)
            {
                *FuncAddr = wvalue;
            }
            else
            {
                code = 4;
            }
        }
        else
        {
            wvalue = (uint16_t)(WValue&0x0000FFFF);
            ret = P0_PF_Write_Limit(Addres, &wvalue);

            wvalue = (uint16_t)(WValue >> 16);
            ret += P0_PF_Write_Limit(Addres + 1, &wvalue);
            if(ret == 0)
            {
                *FuncAddr = (uint16_t)(WValue&0x0000FFFF);
                *(FuncAddr + 1) = (uint16_t)(WValue >> 16);
            }
            else
            {
                code = 4;
            }
        }
    }

    return code;
}

void Can_Flash_Process(void)
{
    uint16_t retval = 0;

    if(CanOta.BootStatus == 0)
    {
        if((CanOta.StartFlag == 1) && (StateMachine.RegulFlg == 0))
        {
            CanOta.BootStatus = 1;
        }
        else
        {
           CanOta.TxFlag = 1;
           CanOta.Txstate = 1;
           CanOta.StartFlag = 0;
           CanOta.BootFlag = 0;
        }
    }
    else if(CanOta.BootStatus == 1)
    {
        CanOta.BootStatus = 2;
        CanOta.StartFlag = 0;
        DisableDog();
    }
    else if(CanOta.BootStatus == 2)
    {
        retval = Foe_EraseBank1();   

        if(retval == 0)
        {
            CanOta.BootStatus = 3; 
            CanOta.OffsetAddress = 0;

            CanOta.TxFlag = 1;
            CanOta.Txstate = 0;
        }
        else
        {
            CanOta.BootStatus = 6; 

            CanOta.TxFlag = 1;
            CanOta.Txstate = 1;
        }
    }
    else if(CanOta.BootStatus == 3)
    {
       if(CanOta.RxEndFlag == 1)
       {
           memcpy(&CanOta.FirmBuff[CanOta.BuffCnt],CanOta.FirmData,64);
           CanOta.BuffCnt += 64;
           if(CanOta.BuffCnt == 1024)
           {
               CanOta.BuffCnt = 0;
               retval = Foe_Data_Write(&CanOta.FirmBuff[0], 1024, &CanOta.OffsetAddress);
           }
           else
           {
               retval = 0;
           }

            if(retval == 0)
            {
               CanOta.TxFlag = 1;
               CanOta.Txstate = 0;
            }
            else
            {
               CanOta.TxFlag = 1;
               CanOta.Txstate = 1;
               CanOta.BootStatus = 6; 
            }

           CanOta.RxEndFlag = 0;
       }
       else if(CanOta.RxEndFlag == 2)
       {
           if((Foe_CheckSum(CanOta.OffsetAddress) == 1))
           {
               CanOta.BootStatus = 4;  
           }
           else
           {
               CanOta.TxFlag = 1;
               CanOta.Txstate = 1;  
           }
       }
       else
       {
           CanOta.BootStatus = 3; 
       }

       CanOta.RxEndFlag = 0;
    }
    else if(CanOta.BootStatus == 4)
    {
       retval = Foe_WriteBank1Flag();

       if(retval == 0) 
       {
           CanOta.BootStatus = 5; 
           CanOta.TxFlag = 1;
           CanOta.Txstate = 0;
       }
       else
       {
           CanOta.Cnt = 0;
           CanOta.BootStatus = 6; 
           CanOta.TxFlag = 1;
           CanOta.Txstate = 1;
       }
    }
    else if(CanOta.BootStatus == 5)
    {
        if(CanOta.RxEndFlag == 3)
        {
            CanOta.Cnt++;
            if(CanOta.Cnt >= 100)
            {
                CanOta.Cnt = 0;
                CanOta.BootStatus = 6;
                foe_support_soc_reset();
            }
        }
    }
    else
    {
    }
}

void Can_Start_Process(void)
{
    if(CanOta.FirmCode[0] == OtaNameHeader[0] &&\
       CanOta.FirmCode[1] == OtaNameHeader[1] &&\
       CanOta.FirmCode[2] == OtaNameHeader[2] &&\
       CanOta.FirmCode[3] == OtaNameHeader[3] &&\
       CanOta.FirmCode[4] == OtaNameHeader[4] &&\
       CanOta.FirmCode[5] == OtaNameHeader[5] &&\
       CanOta.FirmCode[6] == OtaNameHeader[6] &&\
       CanOta.FirmCode[7] == OtaNameHeader[7] &&\
       CanOta.FirmCode[8] == OtaNameHeader[8] &&\
       CanOta.FirmCode[9] == OtaNameHeader[9])
    {
        CanOta.StartFlag = 1;
        CanOta.BootFlag = 1;
    }
    else
    {
        CanOta.StartFlag = 0;
    }
}

void Can_Boot_Process(void)
{
    uint32_t CanRxFlag;
    mcan_rx_message_t rx_msg;

    while(CanOta.BootFlag)
    {
        CanRxFlag = RXflag;

        //OTA指令
        if(CanRxFlag & 0x400)
        {
            MCAN_Read_Rxbuff(CANSelect, 10, &rx_msg);
            memcpy(&CanOta.FirmCode,&rx_msg.data_8[0],6);
            if(CanOta.FirmCode[0] == 0x4F &&\
               CanOta.FirmCode[1] == 0x54 &&\
               CanOta.FirmCode[2] == 0x41 &&\
               CanOta.FirmCode[3] == 0x45 &&\
               CanOta.FirmCode[4] == 0x4E &&\
               CanOta.FirmCode[5] == 0x44)//OTAEND
            {
                CanOta.RxEndFlag = 2;
            }
            else if(CanOta.FirmCode[0] == 0x52 &&\
               CanOta.FirmCode[1] == 0x45 &&\
               CanOta.FirmCode[2] == 0x53 &&\
               CanOta.FirmCode[3] == 0x45 &&\
               CanOta.FirmCode[4] == 0x54)//Reset
            {
                CanOta.RxEndFlag = 3;
            }
            else
            {
                CanOta.RxEndFlag = 0;
            }
            RXflag = 0x400; 
        }

        //OTA数据
        if(CanRxFlag & 0x800)
        {
            MCAN_Read_Rxbuff(CANSelect, 11, &rx_msg);
            RXflag = 0x800; 
            memcpy(&CanOta.FirmData,&rx_msg.data_8[0],64);
            CanOta.RxEndFlag = 1;
        }

        Can_Flash_Process();

        if(CanOta.TxFlag == 1)
        {
            mcan_tx_frame_t tx_msg;

            tx_msg.std_id = 0x7C0 + SalveId;
            tx_msg.canfd_frame = 1;
            tx_msg.bitrate_switch = 1;
            tx_msg.dlc = 4;

            tx_msg.data_8[0] = 0x41;//A
            tx_msg.data_8[1] = 0x43;//C
            tx_msg.data_8[2] = 0x4B;//K
            tx_msg.data_8[3] = CanOta.Txstate;
            CanOta.Txstate = 0;
            CanOta.TxFlag = 0;

            MCAN_Tranmit(CANSelect, &tx_msg);
        }
    }
}
#endif

