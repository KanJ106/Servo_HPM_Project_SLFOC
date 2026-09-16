#ifndef SV_CANFIRM_H_
#define SV_CANFIRM_H_
#include "r_can_api.h"
#include "r_cg_project.h"//

#if SERVOTYPE == SERVO_CAN

#define RXflag          CANSelect->NDAT1
#define TX_Success      CANSelect->TXBTO
#define Cancel_Send     CANSelect->TXBCR

#define EMCTXBuff     0
#define SDOTXBuff     1
#define FBTXBuff      2
#define ReQTXBuff     3
#define STATUSTXBuff  4
#define MITTXBuff     5
#define FAULTTXBuff   6

#define pos_min         -6.28
#define pos_max          6.28
#define vel_min         -12.56
#define vel_max          12.56
#define tor_min         -100
#define tor_max          100
#define kp_min           0
#define kp_max           500
#define kd_min           0
#define kd_max           8

#define  MitMedPos        32767
#define  MitMedSpd        2047
#define  MitMedTor        2047
#define  MitMedKp         2047
#define  MitMedKd         2047


typedef struct {
    int32_t  Pos;
    int16_t  Vel;
    int16_t  Torque;
    uint8_t Flag;
} can_rx_pvtctrl;

typedef struct {
    int32_t  Pos;
    int16_t  Vel;
    uint8_t  Mode;
    uint8_t Flag;
} can_rx_posctrl;

typedef struct {
    int16_t Vel;
    uint16_t AccDec;
} can_rx_velctrl;

typedef struct {
    int16_t Torque;
    uint16_t VelLim;
} can_rx_torctrl;

typedef struct {
    int32_t  Pos;
    int16_t  Vel;
    int16_t  Tor;
} can_rx_Syncctrl;

//MIT指令帧
typedef struct
{
    float fPos;
    float fVel;
    float fTor;
    float fKp;
    float fKd;
    
    uint16_t uPos;
    uint16_t uVel;
    uint16_t uTor;
    uint16_t uKp;
    uint16_t uKd;
    
    int16_t Pos;
    int16_t Vel;
    int16_t Tor;
    int16_t Kp;
    int16_t Kd;

    double LimitfPos_L;
    double LimitfPos_N;
    double LimitfVel_L;
    double LimitfVel_N;
    double LimitfTor_L;
    double LimitfTor_N;
   
}can_rx_mitctrl;

//MIT反馈帧
typedef struct
{
    float fPos;
    float fVel;
    float fTor;
    double FbfPos;
    double FbfVel;
    double FbfTor;
  
    int16_t Pos;
    int16_t Vel;
    int16_t Tor;
  
    uint16_t Motor_MitPos;
    uint16_t Motor_MitVel;
    uint16_t Motor_MitTor;

    uint8_t State;
    uint16_t ErrorCode;
}can_mit_fb;

typedef struct {
    int32_t Pos;
    int16_t Vel;
    int16_t Torque;
    uint16_t State;
    uint16_t Enable;
    float    TorSenValue;
} can_tx_fb;

typedef struct {
    int32_t Pos;
    int16_t Vel;
    int16_t Torque;
} can_tx_req;

typedef struct {
    int32_t  PosErr;
    uint16_t Errcode;
    uint16_t Flag;
} can_tx_Status;

typedef struct {
    uint8_t  Cmd;
    uint16_t Addres;
    uint8_t  Len;
    udata_t  Data;
    uint8_t  Num;
} can_Sdo_Rx;

typedef struct {
    uint8_t  Cmd;
    uint16_t Addres;
    uint8_t  Len;
    udata_t  Data;
    uint8_t  ErrFlag;
} can_Sdo_Tx;

typedef struct {
    uint8_t  Cmd;
    uint16_t Addres;
    uint8_t  Len;
    udata_t  Data;
    uint8_t  ErrFlag;
} can_Emc_Tx;


typedef struct
{
   uint8_t Enable:1;
   uint8_t Halt:1;
   uint8_t EStop:1;
   uint8_t rsd:5;
}TYPE_CtrlWordBIT;

typedef union {
    uint8_t Byte;
    TYPE_CtrlWordBIT Bit;
} TYPE_CtrlWord;

typedef struct
{
    TYPE_CtrlWord   CtrlWord;
    can_rx_posctrl  PosTarget;
    can_rx_velctrl  SpdTarget;
    can_rx_torctrl  TorTarget;
    can_rx_Syncctrl SyncTarget;
    
    can_tx_fb       ActualValue;
    can_tx_req      DemandValue;
    can_tx_Status   Status;
    
    uint16_t        CanState;
    uint32_t        CanSyncTime;  
    uint16_t        SyncCount;
    uint16_t        SyncValue;
    uint16_t        SMCount;
    uint16_t        SMValue;
    
    int32_t  Udc_mv;
    uint16_t MosTemper;
    uint16_t rsd3;
    int32_t  SinglePos_Mortor;
    int32_t  SinglePos_Reduce;
    //uint16_t EStop;
    uint16_t ModeDisplay;
    uint16_t CtrlDisplay;

    can_rx_mitctrl  MITTarget;
    can_mit_fb      MitFb;
    uint16_t FaultIndex;
}TYPE_CanbusComm;

typedef struct
{
    uint8_t  Type;
    uint8_t  Size;
    uint8_t  RW;
    uint32_t Max;
    uint32_t Min;
}TYPE_AttRI;

typedef struct
{
    uint8_t  StartFlag;
    uint8_t  BootFlag;
    uint8_t  RxEndFlag;
    uint8_t  BootStatus;
    uint8_t  TxFlag;
    uint8_t  Txstate;
    uint16_t BuffCnt;
    uint16_t Cnt;
    uint32_t OffsetAddress;
    uint8_t  FirmCode[12];
    uint8_t  FirmData[64];
    uint8_t  FirmBuff[1024];
}TYPE_OTA;

extern void CAN_TxData_To_FbBuff(can_tx_fb *msg, mcan_tx_frame_t *tx_msg);
extern void CAN_TxData_To_SdoBuff(can_Sdo_Tx *msg, mcan_tx_frame_t *tx_msg);
extern void Sdo_Service(can_Sdo_Rx *rx,can_Sdo_Tx *tx);
uint8_t CANFD_RxData_To_PvtCtrl(can_rx_Syncctrl *msg, const mcan_rx_message_t *rx_msg);

extern void Can_TxProcess(void);
extern void Can_RxProcess(mcan_rx_message_t *rx_msg);
extern void CanSyncCheck(void);
extern uint16_t Can_Func_Cacl(uint16_t Addres,uint16_t Len,uint16_t Rw,uint32_t WValue,uint32_t *RValue);
void Can_TxCheckProcess(void);
void Can_Boot_Process(void);
void Can_Start_Process(void);

extern can_rx_pvtctrl PvtCtrl;
extern can_rx_posctrl PosCtrl;
extern can_rx_velctrl VelCtrl;
extern can_rx_torctrl TorCtrl;
extern can_tx_fb      FbCtrl;
extern can_Sdo_Rx     SdoRxdata;
extern can_Sdo_Tx     SdoTxdata;
extern uint16_t       CanCtrlWord;
extern TYPE_CanbusComm    CM_BusVar;
extern uint8_t SalveId;
extern uint8_t CANFDSalveId;
extern uint32_t FIFO_Index;
extern float output;
extern uint8_t CanTxFlag[8];
#endif
#endif

