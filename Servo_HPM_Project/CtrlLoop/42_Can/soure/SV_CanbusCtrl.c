#include "r_can_api.h"
#include "SV_CanFirm.h"
#include <string.h>
#include "SV_CanbusCtrl.h"
#include "Pos_FullClosedLoop.h"
#include "SV_OTProtect.h"
#include "SV_UdcCtrl.h"
#include "SV_MagEncode.h"
#include "main.h"//
#include "s_tsensor_init.h"
#include "Version.h"

#if SERVOTYPE == SERVO_CAN
#define  GET32VAR_ADDRL(x)   ((uint16_t*)&(x))
#define  GET32VAR_ADDRH(x)   ((uint16_t*)&(x) + 1)

TYPE_CMbusConTrol  CM_BusCtrl;

uint32_t CanbusNull = 0;
extern float ActualTorqueClose;
uint16_t* const CanBusConTrolAddres[100] = 
{ 
    (uint16_t*)&RamFuncCode.PA[0],                  //    控制模式              0x1A00
    (uint16_t*)(&CM_BusVar.CanState),               //    同步状态              0x1A01
    (uint16_t*)(&RamFuncCode.PA[24]),               //    同步周期              0x1A02
    (uint16_t*)(&RamFuncCode.PA[25]),               //    反馈帧模式            0x1A03
    (uint16_t*)(&CM_BusVar.CtrlDisplay),            //    控制字显示            0x1A04
    (uint16_t*)(&CM_BusVar.ModeDisplay),            //    控制模式显示          0x1A05
    (uint16_t*)&RamFuncCode.PA[2],                  //    二级故障停车模式      0x1A06
    (uint16_t*)&RamFuncCode.PA[3],                  //    关使能停车模式        0x1A07
    (uint16_t*)&RamFuncCode.P8[1],                  //    复位故障              0x1A08
    (uint16_t*)&RamFuncCode.P8[0],                  //    软复位                0x1A09  
     
    /*************位置模式**************************/
    GET32VAR_ADDRL(CM_BusVar.PosTarget.Pos),        //                          0x1A0A
    (uint16_t*)&CanbusNull,                         //                          0x1A0B
    (uint16_t*)(&CanbusNull),                       //                          0x1A0C
    (uint16_t*)(&CanbusNull),                       //                          0x1A0D
    GET32VAR_ADDRL(CM_BusVar.PosTarget.Vel),        //                          0x1A0E
    (uint16_t*)(&CanbusNull),                       //                          0x1A0F
    (uint16_t*)(&CanbusNull),                       //                          0x1A10
    (uint16_t*)&RamFuncCode.PA[4],                  //    位置加速度            0x1A11
    (uint16_t*)&RamFuncCode.PA[6],                  //    位置减速度            0x1A12
     /*************位置模式*************************/   
    
    /*************速度模式**************************/
    GET32VAR_ADDRL(CM_BusVar.SpdTarget.Vel),        //                          0x1A13
    (uint16_t*)(&CanbusNull),                       //                          0x1A14
    (uint16_t*)(&CanbusNull),                       //                          0x1A15
    /*************速度模式**************************/

    /*************力矩模式**************************/    
    (uint16_t*)&CM_BusVar.TorTarget.Torque,         //                          0x1A16
    (uint16_t*)&RamFuncCode.PA[10],                 //    力矩速度限制          0x1A17 
    (uint16_t*)&RamFuncCode.PA[8],                  //    力矩加速度L           0x1A18 
    (uint16_t*)&RamFuncCode.PA[9],                  //    力矩加速度H           0x1A19
    /*************力矩模式**************************/  
    
    GET32VAR_ADDRL(CM_BusVar.ActualValue.Pos),      //    位置反馈值L           0x1A1A 
    (uint16_t*)(&CanbusNull),                       //     rsd                  0x1A1B    
    GET32VAR_ADDRL(CM_BusVar.ActualValue.Vel),      //    速度反馈值            0x1A1C   
    (uint16_t*)&(CM_BusVar.ActualValue.Torque),     //    力矩反馈值L           0x1A1D 
  
    GET32VAR_ADDRL(CM_BusVar.Status.PosErr),        //    位置偏差反馈值L       0x1A1E
    (uint16_t*)(&CanbusNull),                       //     rsd                  0x1A1F
    (uint16_t*)&CM_BusVar.Status.Errcode,           //    错误码                0x1A20
    (uint16_t*)&CM_BusVar.Status.Flag,              //    状态码                0x1A21
    
    GET32VAR_ADDRL(CM_BusVar.DemandValue.Pos),      //    位置需求值L           0x1A22
    (uint16_t*)(&CanbusNull),                       //     rsd                  0x1A23  
    GET32VAR_ADDRL(CM_BusVar.DemandValue.Vel),      //    速度需求值L           0x1A24 
    GET32VAR_ADDRL(CM_BusVar.DemandValue.Torque),   //    力矩需求值L           0x1A25 

    GET32VAR_ADDRL(CM_BusVar.SinglePos_Mortor),     //    电机端编码器值L       0x1A26 
    (uint16_t*)(&CanbusNull),                       //     rsd                  0x1A27   
    GET32VAR_ADDRL(CM_BusVar.SinglePos_Reduce),     //    减速机端编码器值L     0x1A28
    GET32VAR_ADDRL(TorQueData),                     //    力矩传感器值           0x1A29
    
    GET32VAR_ADDRL(CM_BusVar.Udc_mv),               //    母线电压L             0x1A2A 
    (uint16_t*)(&CanbusNull),                       //     rsd                  0x1A2B      
    (uint16_t*)&(CM_BusVar.MosTemper),              //    MOS温度               0x1A2C     
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A2D
    
    (uint16_t*)&RamFuncCode.PA[21],                 //    位置到达精度          0x1A2E
    (uint16_t*)&RamFuncCode.PA[22],                 //    位置到达时间          0x1A2F
    (uint16_t*)&RamFuncCode.PA[12],                 //    位置正极限L           0x1A30
    (uint16_t*)&RamFuncCode.PA[30],                 //    是否带同步帧          0x1A31  
    (uint16_t*)&RamFuncCode.PA[14],                 //    位置负极限L           0x1A32
    (uint16_t*)&RamFuncCode.PD[06],                 //    减速比                0x1A33  
    (uint16_t*)&RamFuncCode.PA[18],                 //    速度最大值            0x1A34     
    (uint16_t*)&RamFuncCode.PA[20],                 //    力矩最大值            0x1A35
    (uint16_t*)&RamFuncCode.PA[16],                 //    位置偏差阈值L         0x1A36
    (uint16_t*)&RamFuncCode.PA[17],                 //    位置偏差阈值H         0x1A37
    (uint16_t*)&RamFuncCode.P1[18],                 //    零速停车减速度        0x1A38  
    (uint16_t*)&RamFuncCode.P1[20],                 //    二级故障零速停车时间  0x1A39    
    (uint16_t*)&RamFuncCode.P7[16],                 //    CAN  ID               0x1A3A 
    (uint16_t*)&RamFuncCode.P7[17],                 //    CAN  BAud             0x1A3B  
    (uint16_t*)&RamFuncCode.P1[61],                 //    保存                  0x1A3C
    
    (uint16_t*)&RamFuncCode.P7[18],                 //    rsd                   0x1A3D
    (uint16_t*)&RamFuncCode.P7[19],                 //    rsd                   0x1A3E
    (uint16_t*)&RamFuncCode.P7[20],                 //    rsd                   0x1A3F  
    (uint16_t*)&RamFuncCode.P8[30],                 //    强制抱闸输出           0x1A40
    (uint16_t*)&RamFuncCode.F76H[0x12],             //    rsd                   0x1A41
    (uint16_t*)&RamFuncCode.F76H[4],                //    rsd                   0x1A42
    (uint16_t*)&RamFuncCode.F76H[5],                //    rsd                   0x1A43
    (uint16_t*)&RamFuncCode.F76H[6],                //    rsd                   0x1A44
    (uint16_t*)&RamFuncCode.F76H[7],                //    rsd                   0x1A45
    (uint16_t*)&RamFuncCode.F76H[1],                //    rsd                   0x1A46
    (uint16_t*)&RamFuncCode.F76H[0x13],             //    rsd                   0x1A47
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A48
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A49
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4A
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4B
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4C
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4D
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4E
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A4F

    (uint16_t*)&RamFuncCode.P0[32],                 //    程序版本               0x1A50
    (uint16_t*)&ProSnCode[0],                       //    SN码                   0x1A51
    (uint16_t*)&ProSnCode[4],                       //    SN码                   0x1A52
    (uint16_t*)&ProSnCode[8],                       //    SN码                   0x1A53
    (uint16_t*)&ProSnCode[12],                      //    SN码                   0x1A54
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A55
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A56
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A57
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A58
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A59
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A60
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A61
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A62
    (uint16_t*)&CanbusNull,                         //    rsd                   0x1A63
};

TYPE_AttRI AttCode[100] =
{
    /*符号 Size  R(0)RW(1)  MAX           MIN  */
    {0x00, 0x02,  0x00, 0x0000000A,   0x00000001},  //    控制模式              0x1A00
    {0x00, 0x02,  0x00, 0x00000002,   0x00000000},  //    同步状态              0x1A01
    {0x00, 0x02,  0x00, 0x0000000C,   0x00000002},  //    同步周期              0x1A02  //mqb 2025/11/18修改最小同步周期为2
    {0x00, 0x02,  0x00, 0x00000002,   0x00000000},  //    反馈帧模式            0x1A03
    {0x00, 0x02,  0x01, 0x000000FF,   0x00000000},  //    控制字显示            0x1A04
    {0x00, 0x02,  0x01, 0x0000000A,   0x00000001},  //    控制模式显示          0x1A05
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    二级故障停车模式      0x1A06
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    关使能停车            0x1A07
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    复位故障              0x1A08
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    软复位                0x1A09
    
    /*************位置模式**************************/
    {0x01, 0x04,  0x00, 0x7FFFFFFF,   0x80000000},  //    目标位置              0x1A0A
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A0B
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A0C
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A0D
    {0x01, 0x02,  0x00, 0x00007FFF,   0xFFFF8000},  //    位置速度              0x1A0E
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A0F
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A10
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000032},  //    位置加速度            0x1A11
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000032},  //    位置减速度            0x1A12
     /*************位置模式*************************/   
    
    /*************速度模式**************************/
    {0x01, 0x02,  0x00, 0x00007FFF,   0xFFFF8000},  //    目标速度               0x1A13
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A14
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A15
    /*************速度模式**************************/

    /*************力矩模式**************************/    
    {0x01, 0x02,  0x00, 0x00007FFF,   0xFFFF8000},  //    目标力矩              0x1A16
    {0x00, 0x02,  0x00, 0x00007FFF,   0x00000000},  //    力矩速度限制          0x1A17 
    {0x00, 0x04,  0x00, 0xFFFFFFFF,   0x00000000},  //    力矩加速度            0x1A18 
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A19
    /*************力矩模式**************************/  
    
    {0x01, 0x04,  0x01, 0x7FFFFFFF,   0x80000000},  //    位置反馈值L           0x1A1A 
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A1B    
    {0x01, 0x02,  0x01, 0x00007FFF,   0xFFFF8000},  //    速度反馈值            0x1A1C   
    {0x01, 0x02,  0x01, 0x00007FFF,   0xFFFF8000},  //    力矩反馈值L           0x1A1D 
  
    {0x01, 0x04,  0x00, 0x7FFFFFFF,   0x80000000},  //    位置偏差反馈值L       0x1A1E
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A1F
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    错误码                0x1A20
    {0x00, 0x02,  0x01, 0x0000FFFF,   0x00000000},  //    状态码                0x1A21
    
    {0x01, 0x04,  0x01, 0x7FFFFFFF,   0x80000000},  //    位置需求值L           0x1A22
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A23  
    {0x01, 0x02,  0x01, 0x00007FFF,   0xFFFF8000},  //    速度需求值L           0x1A24 
    {0x01, 0x02,  0x01, 0x00007FFF,   0xFFFF8000},  //    力矩需求值L           0x1A25 

    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    电机端编码器值L       0x1A26 
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A27   
    {0x01, 0x04,  0x01, 0x7FFFFFFF,   0x80000000},  //    减速机端编码器值L     0x1A28
    {0x01, 0x04,  0x00, 0x7FFFFFFF,   0x80000000},  //    力矩传感器值          0x1A29
    
    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    母线电压L             0x1A2A 
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A2B      
    {0x00, 0x02,  0x01, 0x0000FFFF,   0x00000000},  //    MOS温度               0x1A2C     
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A2D
    
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    位置到达精度          0x1A2E
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    位置到达时间          0x1A2F
    {0x01, 0x04,  0x00, 0x7FFFFFFF,   0x80000000},  //    位置正极限L           0x1A30
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    是否使用同步帧        0x1A31
    {0x01, 0x04,  0x00, 0x7FFFFFFF,   0x80000000},  //    位置负极限L           0x1A32
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    减速比                0x1A33
    {0x00, 0x02,  0x00, 0x00007FFF,   0x00000000},  //    速度最大值            0x1A34     
    {0x00, 0x02,  0x00, 0x00000BB8,   0x00000000},  //    力矩最大值            0x1A35
    {0x00, 0x04,  0x00, 0xFFFFFFFF,   0x00000000},  //    位置偏差阈值          0x1A36
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A37
    {0x00, 0x02,  0x00, 0x00007530,   0x00000032},  //    零速停车减速度         0x1A38  
    {0x00, 0x02,  0x00, 0x00007530,   0x00000032},  //    二级故障零速停车时间    0x1A39    
    {0x00, 0x02,  0x00, 0x0000007F,   0x00000001},  //    CAN  ID               0x1A3A 
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    CAN  BAud             0x1A3B  
    {0x00, 0x02,  0x00, 0x00000042,   0x00000037},  //    保存配置               0x1A3C
    {0x00, 0x02,  0x00, 0x000000FF,   0x00000004},  //    rsd                   0x1A3D
    {0x00, 0x02,  0x00, 0x00000004,   0x00000000},  //    rsd                   0x1A3E
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A3F
    {0x00, 0x02,  0x00, 0x00000080,   0x00000000},  //    强制抱闸输出           0x1A40
    {0x00, 0x02,  0x00, 0x00000001,   0x00000000},  //    rsd                   0x1A41
    {0x00, 0x02,  0x00, 0x0000004F,   0x00000000},  //    rsd                   0x1A42
    {0x00, 0x02,  0x00, 0x0000004F,   0x00000000},  //    rsd                   0x1A43
    {0x00, 0x02,  0x00, 0x0000004F,   0x00000000},  //    rsd                   0x1A44
    {0x00, 0x02,  0x00, 0x0000004F,   0x00000000},  //    rsd                   0x1A45
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A46
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A47
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A48
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A49
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4A
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4B
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4C
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4D
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4E
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A4F

    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    程序版本               0x1A50
    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    程序版本               0x1A51
    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    程序版本               0x1A52
    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    程序版本               0x1A53
    {0x00, 0x04,  0x01, 0xFFFFFFFF,   0x00000000},  //    程序版本               0x1A54
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A55
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A56
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A57
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A58
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A59
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A60
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A61
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A62
    {0x00, 0x02,  0x00, 0x0000FFFF,   0x00000000},  //    rsd                   0x1A63
};

void CM_busCtrl_init(TYPE_CMbusConTrol *v)
{
    double temp1;   
    temp1 = 16777216.0 / (double)DrvCoeff.SpdBase; 
    temp1 = temp1 * 0.18310546875;   // 6000rpm == 32768
    
    (void)v;

    if(FullCloseLoop.FullFlag > 0)
    {
        CM_BusCtrl.Pos_CmdCoeff = temp1 / RamPD->Reduction;   
    }
    else
    {
        CM_BusCtrl.Pos_CmdCoeff = temp1;
    }
    temp1 = 16777216.0 / (double)DrvCoeff.SpdBase; 
    temp1 = temp1 * 0.18310546875;   // 6000rpm == 32768
    CM_BusCtrl.Spd_CmdCoeff = temp1;    
    temp1 = (double)DrvCoeff.SpdBase / 16777216.0;
    temp1 = temp1 * 5.46133333333;
    CM_BusCtrl.Spd_FbCoeff = temp1;    
    
    temp1 = RamMotor->Nmax / 6000.0;
    temp1 = temp1 * 32768.0;
    CM_BusCtrl.MaxSpd_internal = (int16_t)temp1;  
    
    if(RamPA->MaxSpd > (uint32_t)CM_BusCtrl.MaxSpd_internal)
        CM_BusCtrl.MaxSpd = CM_BusCtrl.MaxSpd_internal;
    else
        CM_BusCtrl.MaxSpd = RamPA->MaxSpd;
    
    CM_BusCtrl.MaxTorque_internal = RamServo->TorqLimt;
    
    if(RamPA->MaxTor > CM_BusCtrl.MaxTorque_internal)
        CM_BusCtrl.MaxTorque = CM_BusCtrl.MaxTorque_internal;
    else
        CM_BusCtrl.MaxTorque = RamPA->MaxTor;
    
    CM_BusCtrl.CntMax_1S = (int64_t)100000000 / DrvCoeff.SpdTsamp; 
    CM_BusCtrl.PosCoeff_1ms = 100000L / DrvCoeff.PosTsamp;         
    
    CM_BusCtrl.SyncTime = 4000L;   
    //MQB MIT  以下系数有待确认
    //20电机 最大速度4.5rad/s;最大力矩212Nm;位置转到15.?的时候翻头
    /*CM_BusCtrl.MITPos_CmdCoeff = 6.28f   / 32768.0f;     //15.28    
    CM_BusCtrl.MITSpd_CmdCoeff = 12.56f  / 2048.0f;      //4.5    
    CM_BusCtrl.MITKp_CmdCoeff  = 500.0f  / 2048.0f;          
    CM_BusCtrl.MITKd_CmdCoeff  = 8.0f    / 2048.0f;          
    CM_BusCtrl.MITTor_CmdCoeff = 600.0f  / 2048.0f;      //70.6    
    
    CM_BusCtrl.MITFbPos_CmdCoeff = 32768.0f / 6.28f;        
    CM_BusCtrl.MITFbSpd_CmdCoeff = 2048.0f  / 12.56f;        
    CM_BusCtrl.MITFbTor_CmdCoeff = 2048.0f  / 600.0f;  */ 
    //20电机 最大速度4.5rad/s;最大力矩212Nm;位置转到15.?的时候翻头
    CM_BusCtrl.MITPos_CmdCoeff = 15.28f   / 32768.0f;     //15.28    
    CM_BusCtrl.MITSpd_CmdCoeff = 4.5f  / 2048.0f;      //4.5    
    CM_BusCtrl.MITKp_CmdCoeff  = 500.0f  / 2048.0f;          
    CM_BusCtrl.MITKd_CmdCoeff  = 8.0f    / 2048.0f;          
    CM_BusCtrl.MITTor_CmdCoeff = 70.6f  / 2048.0f;      //70.6    
    
    CM_BusCtrl.MITFbPos_CmdCoeff = 32768.0f / 15.28f;        
    CM_BusCtrl.MITFbSpd_CmdCoeff = 2048.0f  / 4.5f;        
    CM_BusCtrl.MITFbTor_CmdCoeff = 2048.0f  / 70.6f;
    //mqb位置的正负限位的系数需要进一步确认(RamPA->POS_PLim的单位是编码器的分辨率吗?如果是需要这样做 RamPA->POS_PLim / 编码器分辨率 * 6.28 / 减速比)
    CM_BusVar.MITTarget.LimitfPos_L = (float)RamPA->POS_PLim / DrvCoeff.MotEncSglRevLns / RamPD->Reduction * 6.28f;
    CM_BusVar.MITTarget.LimitfPos_N = (float)RamPA->POS_NLim / DrvCoeff.MotEncSglRevLns / RamPD->Reduction * 6.28f;

    CM_BusVar.MITTarget.LimitfVel_L = (float)CM_BusCtrl.MaxSpd / RamPD->Reduction / 60.0f * 2 * PI;
    CM_BusVar.MITTarget.LimitfVel_N = (float)(-CM_BusCtrl.MaxSpd) / RamPD->Reduction / 60.0f * 2 * PI;
    CM_BusVar.MITTarget.LimitfTor_L = (float)CM_BusCtrl.MaxTorque * RamPD->Reduction;
    CM_BusVar.MITTarget.LimitfTor_N = (float)(-CM_BusCtrl.MaxTorque) * RamPD->Reduction;
    /*
    CM_BusCtrl.MITPos_CmdCoeff = 6.28f   / 65536.0f;         //v->MITPosScope / 65536.0f;
    CM_BusCtrl.MITSpd_CmdCoeff = 12.56f  / 4096.0f;          //v->MITSpdScope / 4096.0f;
    CM_BusCtrl.MITKp_CmdCoeff  = 500.0f  / 4096.0f;          //v->MITKpScope  / 4096.0f;
    CM_BusCtrl.MITKd_CmdCoeff  = 8.0f    / 4096.0f;          //v->MITKdScope  / 4096.0f;
    CM_BusCtrl.MITTor_CmdCoeff = 1200.0f / 4096.0f;          //v->MITTorScope / 4096.0f;
    
    CM_BusCtrl.MITFbPos_CmdCoeff = 65536.0f / 12.56f;        //65536.0f / v->MITPosScope;
    CM_BusCtrl.MITFbSpd_CmdCoeff = 4096.0f  / 25.12f;        //4096.0f  / v->MITSpdScope;
    CM_BusCtrl.MITFbTor_CmdCoeff = 4096.0f  / 1200.0f;       //4096.0f  / v->MITTorScope;  
    */
    if(FullCloseLoop.FullFlag > 0)
    {
        CM_BusCtrl.PosFbMitCoeff = 2 * PI / (double)DrvCoeff.OutEncSglRevLns;
        CM_BusCtrl.SpdFbMitCoeff = 8192.0f / 16777216.0f / 60.0f * 2 * PI; 
    }
    else
    {
        CM_BusCtrl.PosFbMitCoeff = 2 * PI / (double)(DrvCoeff.MotEncSglRevLns * RamPD->Reduction);
        CM_BusCtrl.SpdFbMitCoeff = 8192.0f / (16777216.0f * RamPD->Reduction) / 60.0f * 2 * PI;
    } 
    CM_BusCtrl.TorFbMitCoeff = (double)DrvCoeff.MotTorqe * RamPD->Reduction / 100000.0;    
    CM_BusCtrl.TorCmdMitCoeff = (double)DrvCoeff.MotIe * 100.0 / ((double)DrvCoeff.MotTorqe * RamPD->Reduction);              
}

void CM_busCtrl_rst(TYPE_CMbusConTrol *v)
{
    uint16_t Acctime;
    int32_t  temp64;
        
    if(FullCloseLoop.FullFlag > 0)
    {
    	v->TargetPosA = ExtPosFb.PosFbTotle;
    }
    else
    {
	    v->TargetPosA = PosFb.PosFbTotle;
    }
    
    Acctime = (uint16_t)RamPA->PosAcc;
    if(Acctime == 0)Acctime = 50;
    if(FullCloseLoop.FullFlag > 0)
    {
         temp64 = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.PosTsamp)<<2)/(3125L*Acctime));
         v->TargetPosAccA = temp64 / RamPD->Reduction;
    }
    else
    {
        v->TargetPosAccA = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.PosTsamp)<<2)/(3125L*Acctime));
    }
    v->TargetSpdAccA = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.SpdTsamp)<<2)/(3125L*Acctime)); 
    
    Acctime = (uint16_t)RamPA->PosDec;
    if(Acctime == 0) Acctime = 50;
    if(FullCloseLoop.FullFlag > 0)
    {
         temp64 = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.PosTsamp)<<2)/(3125L*Acctime));
         v->TargetPosDecA = temp64 / RamPD->Reduction;
    }
    else
    {
        v->TargetPosDecA = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.PosTsamp)<<2)/(3125L*Acctime));
    }
    v->TargetSpdDecA = (int32)((((Uint64)DrvCoeff.SpdNePu*DrvCoeff.SpdTsamp)<<2)/(3125L*Acctime));

    v->HomeStep = 0;
}

void CM_busCtrl_updata(TYPE_CMbusConTrol *v)
{
    uint16_t time;

    if(CM_BusVar.CanState < 2)
    {
        time = RamPA->SyncTime;
        if(time < 2) time = 2;
        v->SyncTime = time * 1000L;
        CM_BusVar.CanSyncTime = time;
    }
}

uint16_t CMbusControlMode(void)
{
    uint16_t flag = 0;
    uint16_t DrvMode;
    
    DrvMode = RamPA->ModbusDrvMode;  
   
    switch(DrvMode)
    {
       case 1:
           CM_BusCtrl.Mode = CMBUSMODE_POS;
           flag = 1; 
       break;
       case 2:
           CM_BusCtrl.Mode = CMBUSMODE_SPD;
           flag = 2;
       break;
       case 3:
           CM_BusCtrl.Mode = CMBUSMODE_TOR;
           flag = 3;
       break;
       case 4:
           CM_BusCtrl.Mode = CMBUSMODE_HOME;
           flag = 1;
       break;
       case 5:
           CM_BusCtrl.Mode = CMBUSMODE_MIT;
           flag = 3;   //待定
        break;
       case 8:
           CM_BusCtrl.Mode = CMBUSMODE_SYNCPOS;
           flag = 1;
       break;
       case 9:
           CM_BusCtrl.Mode = CMBUSMODE_SYNCSPD;
           flag = 2;
       break;
       case 10:
           CM_BusCtrl.Mode = CMBUSMODE_SYNCTOR;
           flag = 3;
       break;

       default: break;
    }
    
    CM_BusVar.ModeDisplay = CM_BusCtrl.Mode;
   
    return flag;
}

void CanbusPosGetData(void)
{
    int32_t pulse;
    int32_t v_user;
    
    TYPE_STATEMACHINE *sm = &StateMachine;
    //绝对位置和相对位置的位置超限标志位
    static uint8_t OtpFlag1 = 0;
    static uint8_t OtpFlag2 = 0;
    
    if(CM_BusVar.CtrlWord.Bit.EStop)
    {
        CM_BusCtrl.EstopFlag = 1;      
    }
    else
    {
        CM_BusCtrl.EstopFlag = 0;
    }
    
    if(CM_BusVar.CtrlWord.Bit.Halt)
    {
        CM_BusCtrl.ZeroFlag = 1;
    }
    else
    {
        CM_BusCtrl.ZeroFlag = 0;       
    }
    
    if(((OtpFlag1 == 1) || (OtpFlag2 == 1)) && (sm->SrvOnStatus == 0))
    {
        OtpFlag1 = 0;
        OtpFlag2 = 0;
        FaultPrtt_WarnInterfaceClr(POTWarn);
        FaultPrtt_WarnInterfaceClr(NOTWarn);
    }
    
    if(CM_BusVar.PosTarget.Flag == 0) return;
    CM_BusVar.PosTarget.Flag = 0;
    
    CM_BusCtrl.PosTurn = CM_BusVar.PosTarget.Mode;
    
    if(CM_BusCtrl.PosTurn == 0)            //绝对位置     
    {   
        pulse = CM_BusVar.PosTarget.Pos;
        
        if((DPI_SoftOTSel == 1) && (pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
        {
            pulse = CM_BusCtrl.TargetPosA; 
            OtpFlag1 = 1;
            FaultPrtt_FaultInterface(POTWarn);
        }
        else if((DPI_SoftOTSel == 1) && (pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
        {
            pulse = CM_BusCtrl.TargetPosA; 
            OtpFlag1 = 1; 
            FaultPrtt_FaultInterface(NOTWarn);
        }
        else 
        {
            if(OtpFlag1 == 1)
            {
                OtpFlag1 = 0;
                FaultPrtt_WarnInterfaceClr(POTWarn);
                FaultPrtt_WarnInterfaceClr(NOTWarn);
            }
        }
        
        CM_BusCtrl.TargetPosA = pulse;
    }
    else if(CM_BusCtrl.PosTurn == 1) //相对位置模式
    {
        pulse = CM_BusCtrl.TargetPosA + CM_BusVar.PosTarget.Pos;
        if((DPI_SoftOTSel == 1) && (pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
        {
            pulse = CM_BusCtrl.TargetPosA; 
            OtpFlag2 = 1;
            FaultPrtt_FaultInterface(POTWarn);
        }
        else if((DPI_SoftOTSel == 1) && (pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
        {
            pulse = CM_BusCtrl.TargetPosA; 
            OtpFlag2 = 1; 
            FaultPrtt_FaultInterface(NOTWarn);
        }
        else 
        {
            if(OtpFlag2 == 1)
            {
                OtpFlag2 = 0;
                FaultPrtt_WarnInterfaceClr(POTWarn);
                FaultPrtt_WarnInterfaceClr(NOTWarn);
            }
        }
        
        CM_BusCtrl.TargetPosA = pulse;        
    }
    else 
    {
        
    }

    if(CM_BusVar.PosTarget.Vel >= 0)
    {
        if(CM_BusVar.PosTarget.Vel > CM_BusCtrl.MaxSpd)
        {
            v_user = CM_BusCtrl.MaxSpd;    
        }
        else
        {
            v_user = CM_BusVar.PosTarget.Vel;    
        }
    }
    else
    {
        if(CM_BusVar.PosTarget.Vel < -CM_BusCtrl.MaxSpd)
        {
            v_user = -CM_BusCtrl.MaxSpd;    
        }
        else
        {
            v_user = CM_BusVar.PosTarget.Vel;    
        }        
    }
    
    v_user = (int32_t)((double)v_user * CM_BusCtrl.Pos_CmdCoeff);
    CM_BusCtrl.TargetPosVelA = v_user;

    CM_BusCtrl.ScopePos = CM_BusVar.PosTarget.Pos;
    CM_BusCtrl.ScopeVel = CM_BusVar.PosTarget.Vel;
}

void CanbusSpdGetData(void)
{
    int32_t v_user;
    int16_t SpdCmd;
    
    if(CM_BusVar.CtrlWord.Bit.EStop)
    {
        CM_BusCtrl.EstopFlag = 1;      
    }
    else
    {
        CM_BusCtrl.EstopFlag = 0;
    }
    
    SpdCmd = CM_BusVar.SpdTarget.Vel;
    
    if(SpdCmd >= 0)
    {
        if(SpdCmd > CM_BusCtrl.MaxSpd)
        {
            SpdCmd = CM_BusCtrl.MaxSpd;    
        }
        else
        {
            SpdCmd = SpdCmd;    
        }         
    }
    else
    {
        if(SpdCmd < -CM_BusCtrl.MaxSpd)
        {
            SpdCmd = -CM_BusCtrl.MaxSpd;    
        }
        else
        {
            SpdCmd = SpdCmd;    
        }         
    }
    
    v_user = (int32_t)((double)SpdCmd * CM_BusCtrl.Spd_CmdCoeff);
    CM_BusCtrl.TargetSpdVelA = v_user;

    CM_BusCtrl.ScopeVel = CM_BusVar.SpdTarget.Vel;
}

void CanbusTorqueGetData(void)
{
    int32_t acc_user;
    int32_t v_user;
    int16_t Torque;
    float   temp;
    
    if(CM_BusVar.CtrlWord.Bit.EStop)
    {
        CM_BusCtrl.EstopFlag = 1;      
    }
    else
    {
        CM_BusCtrl.EstopFlag = 0;
    }
    
    Torque = CM_BusVar.TorTarget.Torque;
    
    if(Torque >= 0)
    {
        if(Torque > CM_BusCtrl.MaxTorque)
            CM_BusCtrl.TargetTorqueA = CM_BusCtrl.MaxTorque; 
        else
            CM_BusCtrl.TargetTorqueA = Torque;
    }
    else
    {
        if(Torque < -CM_BusCtrl.MaxTorque)
            CM_BusCtrl.TargetTorqueA = -CM_BusCtrl.MaxTorque; 
        else
            CM_BusCtrl.TargetTorqueA = Torque;        
    }
    
    if((int32_t)RamPA->SpdTorLim > CM_BusCtrl.MaxSpd)
    {
        v_user = CM_BusCtrl.MaxSpd;    
    }
    else
    {
        v_user = (int32_t)RamPA->SpdTorLim;    
    }
    
    CM_BusCtrl.TorqueSpdLim = (int32_t)((float)v_user * CM_BusCtrl.Spd_CmdCoeff);

    temp = RamPA->TorqueSlop * 65536.0f;
    acc_user = (int32_t)(temp / (float)CM_BusCtrl.CntMax_1S);
    CM_BusCtrl.TargetSlopA = acc_user;

    CM_BusCtrl.ScopeTor = CM_BusVar.TorTarget.Torque;
}

void CanbusSyncModeGetData(void)
{
    int32_t v_user;
    int16_t SpdCmd;
    int16_t Torque;
    
    if(CM_BusCtrl.Mode == CMBUSMODE_SYNCPOS)
    {
        CM_BusCtrl.TargetSyncPos = CM_BusVar.SyncTarget.Pos;
    }
    else if(CM_BusCtrl.Mode == CMBUSMODE_SYNCSPD)
    {
        SpdCmd = CM_BusVar.SyncTarget.Vel;
        
        if(SpdCmd >= 0)
        {
            if(SpdCmd > CM_BusCtrl.MaxSpd)
            {
                SpdCmd = CM_BusCtrl.MaxSpd;    
            }
            else
            {
                SpdCmd = SpdCmd;    
            }         
        }
        else
        {
            if(SpdCmd < -CM_BusCtrl.MaxSpd)
            {
                SpdCmd = -CM_BusCtrl.MaxSpd;    
            }
            else
            {
                SpdCmd = SpdCmd;    
            }         
        }
        
        v_user = (int32_t)((double)SpdCmd * CM_BusCtrl.Spd_CmdCoeff);
        CM_BusCtrl.TargetSpdVelA = v_user;
    }
    else 
    {
        Torque = CM_BusVar.SyncTarget.Tor;
        
        if(Torque  >= 0)
        {
            if(Torque > CM_BusCtrl.MaxTorque)
                CM_BusCtrl.TargetTorqueA = CM_BusCtrl.MaxTorque; 
            else
                CM_BusCtrl.TargetTorqueA = Torque;
        }
        else
        {
            if(Torque < -CM_BusCtrl.MaxTorque)
                CM_BusCtrl.TargetTorqueA = -CM_BusCtrl.MaxTorque; 
            else
                CM_BusCtrl.TargetTorqueA = Torque;        
        }
        
        if((int32_t)RamPA->SpdTorLim > CM_BusCtrl.MaxSpd)
        {
            v_user = CM_BusCtrl.MaxSpd;    
        }
        else
        {
            v_user = (int32_t)RamPA->SpdTorLim;    
        }
        
        CM_BusCtrl.TorqueSpdLim = (int32_t)((float)v_user * CM_BusCtrl.Spd_CmdCoeff);
    }

    CM_BusCtrl.ScopePos = CM_BusVar.SyncTarget.Pos;
    CM_BusCtrl.ScopeVel = CM_BusVar.SyncTarget.Vel;
    CM_BusCtrl.ScopeTor = CM_BusVar.SyncTarget.Tor;
}


/*void CanbusMITGetData(void)
{
    TYPE_STATEMACHINE *MIT_MacState = &StateMachine;
    static uint8_t MIT_OtpFlag = 0;

    
    if(CM_BusVar.CtrlWord.Bit.EStop) CM_BusCtrl.EstopFlag = 1;
    else CM_BusCtrl.EstopFlag = 0;
    if(CM_BusVar.CtrlWord.Bit.Halt) CM_BusCtrl.ZeroFlag = 1;
    else CM_BusCtrl.ZeroFlag = 0;

    if(1 == MIT_OtpFlag && (MIT_MacState->SrvOnStatus == 0)) 
    {
        MIT_OtpFlag = 0;
        FaultPrtt_WarnInterfaceClr(POTWarn);
        FaultPrtt_WarnInterfaceClr(NOTWarn);
    }

    // 位置(速度) / 减速比 / 60 * 6.28 = 输出端位置(速度)的限制
    // 力矩 * 减速比 = 输出端力矩的限制
    if((DPI_SoftOTSel == 1) && (CM_BusVar.MITTarget.Pos > RamPA->POS_PLim) && MIT_MacState->SrvOnStatus && (RamPA->POS_PLim != 0))
    { 
        MIT_OtpFlag = 1;
        FaultPrtt_FaultInterface(POTWarn);
    }
    else if((DPI_SoftOTSel == 1) && (CM_BusVar.MITTarget.Pos < RamPA->POS_NLim) && MIT_MacState->SrvOnStatus && (RamPA->POS_NLim != 0))
    { 
        MIT_OtpFlag = 1; 
        FaultPrtt_FaultInterface(NOTWarn);
    }
    else 
    {
        CM_BusCtrl.TargetMITPos = CM_BusVar.MITTarget.Pos;
        if(MIT_OtpFlag == 1)
        {
            MIT_OtpFlag = 0;
            FaultPrtt_WarnInterfaceClr(POTWarn);
            FaultPrtt_WarnInterfaceClr(NOTWarn);
        }
    }
    
    
    if(CM_BusVar.MITTarget.Vel >= 0)
    {
        if(CM_BusVar.MITTarget.Vel > CM_BusCtrl.MaxSpd)
        {
            CM_BusCtrl.TargetMITVel = CM_BusCtrl.MaxSpd;
        }
        else 
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.Vel;
        }
    }
    else 
    {
        if(CM_BusVar.MITTarget.Vel < -CM_BusCtrl.MaxSpd)
        {
            CM_BusCtrl.TargetMITVel = -CM_BusCtrl.MaxSpd;
        }
        else 
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.Vel;
        }
    }
    
    if(CM_BusVar.MITTarget.Tor >= 0)
    {
        if (CM_BusVar.MITTarget.Tor > CM_BusCtrl.MaxTorque) 
        {
            CM_BusCtrl.TargetMITTor = CM_BusCtrl.MaxTorque;
        }
        else 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.Tor;
        }
    }
    else 
    {
        if (CM_BusVar.MITTarget.Tor < -CM_BusCtrl.MaxTorque) 
        {
            CM_BusCtrl.TargetMITTor = -CM_BusCtrl.MaxTorque;
        }
        else 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.Tor;
        }
    }
    CM_BusCtrl.TargetMITKp  = CM_BusVar.MITTarget.Kp ;
    CM_BusCtrl.TargetMITKd  = CM_BusVar.MITTarget.Kd ;
 
}
*/
//mqb MIT
void CanbusMITGetData(void)
{
int32_t v_user;
    TYPE_STATEMACHINE *MIT_MacState = &StateMachine;
    static uint8_t MIT_OtpFlag = 0;

    
    if(CM_BusVar.CtrlWord.Bit.EStop) CM_BusCtrl.EstopFlag = 1;
    else CM_BusCtrl.EstopFlag = 0;
    if(CM_BusVar.CtrlWord.Bit.Halt) CM_BusCtrl.ZeroFlag = 1;
    else CM_BusCtrl.ZeroFlag = 0;

    if(1 == MIT_OtpFlag && (MIT_MacState->SrvOnStatus == 0)) 
    {
        MIT_OtpFlag = 0;
        FaultPrtt_WarnInterfaceClr(POTWarn);
        FaultPrtt_WarnInterfaceClr(NOTWarn);
    }

    // 位置(速度) / 减速比 / 60 * 6.28 = 输出端位置(速度)的限制
    // 力矩 * 减速比 = 输出端力矩的限制
    if((DPI_SoftOTSel == 1) && ((int32_t)CM_BusVar.MITTarget.fPos > (int32_t)CM_BusVar.MITTarget.LimitfPos_L) && MIT_MacState->SrvOnStatus && (RamPA->POS_PLim != 0))
    { 
        MIT_OtpFlag = 1;
        FaultPrtt_FaultInterface(POTWarn);
    }
    else if((DPI_SoftOTSel == 1) && ((int32_t)CM_BusVar.MITTarget.fPos < (int32_t)CM_BusVar.MITTarget.LimitfPos_N) && MIT_MacState->SrvOnStatus && (RamPA->POS_NLim != 0))
    { 
        MIT_OtpFlag = 1; 
        FaultPrtt_FaultInterface(NOTWarn);
    }
    else 
    {
        CM_BusCtrl.TargetMITPos = CM_BusVar.MITTarget.fPos;
        if(MIT_OtpFlag == 1)
        {
            MIT_OtpFlag = 0;
            FaultPrtt_WarnInterfaceClr(POTWarn);
            FaultPrtt_WarnInterfaceClr(NOTWarn);
        }
    }
    
    
    if(CM_BusVar.MITTarget.Vel >= 0)
    {
        if((int32_t)CM_BusVar.MITTarget.fVel > (int32_t)CM_BusVar.MITTarget.LimitfVel_L)
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.LimitfVel_L;
        }
        else 
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.fVel;
        }
    }
    else 
    {
        if((int32_t)CM_BusVar.MITTarget.fVel < (int32_t)CM_BusVar.MITTarget.LimitfVel_N)
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.LimitfVel_N;
        }
        else 
        {
            CM_BusCtrl.TargetMITVel = CM_BusVar.MITTarget.fVel;
        }
    }
    
    if(CM_BusVar.MITTarget.Tor >= 0)
    {
        if ((int32_t)CM_BusVar.MITTarget.fTor > (int32_t)CM_BusVar.MITTarget.LimitfTor_L) 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.LimitfTor_L;
        }
        else 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.fTor;
        }
    }
    else 
    {
        if ((int32_t)CM_BusVar.MITTarget.fTor < (int32_t)CM_BusVar.MITTarget.LimitfTor_N) 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.LimitfTor_N;
        }
        else 
        {
            CM_BusCtrl.TargetMITTor = CM_BusVar.MITTarget.fTor;
        }
    }

    if((int32_t)RamPA->SpdTorLim > CM_BusCtrl.MaxSpd)
    {
        v_user = CM_BusCtrl.MaxSpd;    
    }
    else
    {
        v_user = (int32_t)RamPA->SpdTorLim;    
    }
    
    CM_BusCtrl.TorqueSpdLim = (int32_t)((float)v_user * CM_BusCtrl.Spd_CmdCoeff);  //mqb 为什么加上这个就可以跑了呢

    
    CM_BusCtrl.TargetMITKp  = CM_BusVar.MITTarget.fKp ;
    CM_BusCtrl.TargetMITKd  = CM_BusVar.MITTarget.fKd ;
}


/*mqb该函数中的值是用于内部计算的,CM_BusCtrl.ActualMITPos、CM_BusCtrl.ActualMITVel、ActualMITTor均是目前电机输出端的值
 *其中CM_BusCtrl.ActualMITPos、CM_BusCtrl.ActualMITVel在Canbus_GetMITOutPut()中有用到,目的是为了调整电机当前状态,使其维持到指令所给的状态
 */
void CanbusMITFbData(void)  
{
    int32 spd;

    if(FullCloseLoop.FullFlag > 0)
    {
        CM_BusCtrl.ActualMITPos = (double)((double)ExtPosFb.PosFbTotle * CM_BusCtrl.PosFbMitCoeff);
        spd = (ExtPosFb.SpdMode == 2) ? ExtPosFb.SpdFbOut : SpdFb.Fb;
        CM_BusCtrl.ActualMITVel = (float)((double)spd * CM_BusCtrl.SpdFbMitCoeff);
    }
    else
    {
        CM_BusCtrl.ActualMITPos = (double)((double)PosFb.PosFbTotle * CM_BusCtrl.PosFbMitCoeff);
        CM_BusCtrl.ActualMITVel = (float)((double)SpdFb.Fb * CM_BusCtrl.SpdFbMitCoeff);
    }
    CM_BusCtrl.ActualMITTor = (float)((double)CurMonitor.TorqRatsDispS * CM_BusCtrl.TorFbMitCoeff);
}

//mqb MIT
/*计算公式: output = Kp * (SetValuePos - FbPos) + Kd * (SetValueVel - FbVel) + SetValueTor
 *kp:为位置增益;
 *kd:为速度增益;
 *SetValuePos:输出端位置设定值
 *FbPos:输出端位置反馈值
 *SetValueVel:输出端速度设定值
 *FbVel:输出点速度反馈值
 *SetValueTor:输出端力矩设定值
 */
 int32 TorOut;
int32 Canbus_GetMITOutPut(void)
{
    //float Posfb = (float)(PosFeedback * 6.28 / 131072.00);
    //float Spdfb = (float)((int16_t)(SpdFb.Fb * CM_BusCtrl.Spd_FbCoeff) / 60.00 * 6.28);

    float output;
    
    output = CM_BusCtrl.TargetMITKp*(CM_BusCtrl.TargetMITPos - CM_BusCtrl.ActualMITPos) + \
             CM_BusCtrl.TargetMITKd*(CM_BusCtrl.TargetMITVel - CM_BusCtrl.ActualMITVel) + \
             CM_BusCtrl.TargetMITTor; 
    //output  Nm

    TorOut = (int32)(CM_BusCtrl.TorCmdMitCoeff * (double)output);

    return TorOut;
}

void CanBusLim_Cacl(void)
{
    INT32 Min_PositionLimit,Max_PositionLimit,PosFb; 
    TYPE_STATEMACHINE *sm = &StateMachine; 
    
    Min_PositionLimit = RamPA->POS_NLim;
	Max_PositionLimit = RamPA->POS_PLim;

    PosFb = CM_BusVar.ActualValue.Pos;
	if(PosFb > Max_PositionLimit && sm->SrvOnStatus && Max_PositionLimit != 0)
    {
		//位置正向给定极限
        OTP.Ccw_CwFlg = 1;
	}
    else if(PosFb < Min_PositionLimit && sm->SrvOnStatus && Min_PositionLimit != 0)
    {
        //位置负向给定极限
      
        OTP.Ccw_CwFlg = 2;
    }
    else
    {
        //OTP.Ccw_CwFlg = 0; //在超程保护哪里清除
    }    
}

void Canbus_GetCmd(void)
{
    uint8_t flag;  
   
    switch(CM_BusCtrl.Mode)
    {
        case CMBUSMODE_POS:
            CanbusPosGetData();
            flag = 0;
        break;
        
        case CMBUSMODE_SPD:
            CanbusSpdGetData();
            CanBusLim_Cacl();
            flag = 0;
        break;
        
        case CMBUSMODE_TOR:
            CanbusTorqueGetData();
            CanBusLim_Cacl();
            flag = 0;
        break;
        
        case CMBUSMODE_HOME:
            flag = 0;
        break;
        case CMBUSMODE_MIT:    //mqb mit
            CanbusMITGetData();
            CanBusLim_Cacl();
            flag = 0;
        break;
        
        case CMBUSMODE_SYNCPOS:
        case CMBUSMODE_SYNCSPD:
        case CMBUSMODE_SYNCTOR:
            CanbusSyncModeGetData();
            CanBusLim_Cacl();
            flag = 1;
        break;
        
        default:
            flag = 0;
        break;
    }
    
    if(flag == 0)
    {
        CM_BusCtrl.Run = CM_BusVar.CtrlWord.Bit.Enable;
    }
    else 
    {
		(CM_BusVar.CanState >= 2) ? (CM_BusCtrl.Run = CM_BusVar.CtrlWord.Bit.Enable) : (CM_BusCtrl.Run = 0);
    }
    
    CM_BusVar.CtrlDisplay = CM_BusVar.CtrlWord.Byte;
}

void Canbus_GetFbData(void)
{
    if(FullCloseLoop.FullFlag == 1)
    {
        CM_BusVar.ActualValue.Pos= ExtPosFb.PosFbTotle;
        CM_BusCtrl.PosFbActualValue = ExtPosFb.PosFbTotle;
        CM_BusVar.ActualValue.Vel= (int16_t)(SpdFb.Fb * CM_BusCtrl.Spd_FbCoeff);
    }
    else
    {
        CM_BusVar.ActualValue.Pos = PosFb.PosFbTotle;
        CM_BusCtrl.PosFbActualValue = PosFb.PosFbTotle;
        CM_BusVar.ActualValue.Vel = (int16_t)(SpdFb.Fb * CM_BusCtrl.Spd_FbCoeff);
    }    
    CM_BusVar.ActualValue.Torque = CurMonitor.TorqRatsDispS; 
    CM_BusVar.ActualValue.TorSenValue = ActualTorqueClose; 
    
    CM_BusVar.DemandValue.Pos = PosRef.PosNow;
    CM_BusVar.DemandValue.Vel = (int16_t)(Spd_Ref.Ref * CM_BusCtrl.Spd_FbCoeff);
    CM_BusVar.DemandValue.Torque = CurMonitor.TorqRatsRef; 

    CM_BusVar.MitFb.FbfPos = CM_BusCtrl.ActualMITPos; 
    CM_BusVar.MitFb.FbfVel = CM_BusCtrl.ActualMITVel;
    CM_BusVar.MitFb.FbfTor = CM_BusCtrl.ActualMITTor;
}

int32_t Canbus_FollowErr(void)
{
	int32 tmp1 = 0;
	uint32_t tmp = 0;
    static uint8_t  Errcnt = 0;
    static uint8_t  PosReachedTime = 0;

	TYPE_STATEMACHINE *sm = &StateMachine;
	TYPE_OTPROTECT *ot = &OTP;
    
    if(CM_BusCtrl.Mode == CMBUSMODE_POS || CM_BusCtrl.Mode == CMBUSMODE_SYNCPOS)
    {
        
    }
    else
    {
        Errcnt = 0;
        PosReachedTime = 0;
        
        return  0;
    }
    
    tmp1 = PosReg.TotalErr;
    tmp = abs(tmp1);
    
	if((!sm->OffZSpdStopFlg) && (RamPA->PosErrMax != 0))//在执行其他停车方式时不动作
	{
		if(tmp >= RamPA->PosErrMax && sm->RegulFlg && 0 == ot->POTNOTFlg)//DirOT 会清零，如果中断打断可能会误报位置偏差
		{
			Errcnt++;
		}
		else
		{
			Errcnt = 0;
		}
        
		//跟随误差过大
		if(Errcnt >= 2)
		{
            Errcnt = 0;
			FaultPrtt_FaultInterface(OvPosErr);//位置偏差过大
		}
	}
    
    if(tmp <= RamPA->PosWindow)
    {
        PosReachedTime++;    
    }
    else
    {
        PosReachedTime = 0;
    }
    
    if(PosReachedTime >= RamPA->PosWindowTime)
    {
        CM_BusCtrl.Status.bit.PosReachFlag = 1;    
    }
    else
    {
        CM_BusCtrl.Status.bit.PosReachFlag = 0;     
    }
    
    return tmp1;
}

void Canbus_StatusFeedBack(void)
{
    CM_BusVar.Udc_mv = (((int64_t)AdRead.Udc * UDC_VOLTBASE * 1000L) >> 24);
    CM_BusVar.MosTemper = RamMonitor->TIPM;
    CM_BusVar.SinglePos_Mortor = EncPosFb.EncSinglePos;
    CM_BusVar.SinglePos_Reduce = MagReg.Pos;
    
    CM_BusVar.Status.PosErr = Canbus_FollowErr();   
    
    CM_BusCtrl.Status.bit.RunFlag   = StateMachine.SrvOnStatus > 0 ? 1 : 0;
    CM_BusCtrl.Status.bit.EstopFlag = CM_BusCtrl.EstopFlag;
    CM_BusCtrl.Status.bit.HaltFlag  = CM_BusCtrl.ZeroFlag;
    CM_BusCtrl.Status.bit.HomeOkFlag = (CM_BusCtrl.HomeStep == 3) ? 1 : 0;
    
    if(FaultP.FaultStatus & 0x3)//有警告或者故障发生
	{
        CM_BusVar.Status.Errcode = FaultP.FaultDisp;
        CM_BusCtrl.Status.bit.ErrStatus = FaultP.FaultStatus;
        CM_BusVar.MitFb.ErrorCode = FaultP.FaultDisp;  //mqb MIT错误码
	}
	else
	{
		CM_BusVar.Status.Errcode = 0;
        CM_BusVar.MitFb.ErrorCode = 0;
        CM_BusCtrl.Status.bit.ErrStatus = 0;       
	}
    
    CM_BusVar.Status.Flag = CM_BusCtrl.Status.all;
    CM_BusVar.MitFb.State = (uint8_t)CM_BusCtrl.Status.all; //mqb MIT状态码
    
    CM_BusVar.ActualValue.State = CM_BusCtrl.Status.bit.ErrStatus;
    CM_BusVar.ActualValue.Enable = CM_BusCtrl.Status.bit.RunFlag;
}

void CanBusCtrl_Process(void)
{
    Canbus_GetCmd(); 
    
    Canbus_GetFbData();
    
    Canbus_StatusFeedBack();
}

int32_t SyncPosRef(int32_t delta,POS_REF *v,uint8_t flag)
{
    int32_t poscmd,rem;
    
    if(flag == 1)
    {   
        if(v->SyncCnt == (v->SYNCPeriodRatio - 1))      // 说明插补完成
        {
            rem = delta + v->InterpdataRem;       
            v->Interpdata = rem / v->SYNCPeriodRatio; 
            v->InterpdataRem = rem % v->SYNCPeriodRatio;
        }
        else if(v->SyncCnt < (v->SYNCPeriodRatio - 1)) //说明同步信号提前到来
        {
            rem = delta + v->InterpdataRem + v->Interpdata * (v->SYNCPeriodRatio - v->SyncCnt - 1);
            v->Interpdata = rem / v->SYNCPeriodRatio;
            v->InterpdataRem = rem % v->SYNCPeriodRatio;  
        }
        else                                           //说明同步信号滞后
        {
            rem = delta - v->EcatRem + v->InterpdataRem;
            v->Interpdata = rem / v->SYNCPeriodRatio;
            v->InterpdataRem = rem % v->SYNCPeriodRatio; 
            v->EcatRem = 0;
        }
        v->SyncCnt = 0; 
    }
    else 
    {
        v->SyncCnt++;   
        if(v->SyncCnt >= v->SYNCPeriodRatio)
        {
            v->EcatRem += v->Interpdata;
        }
    } 
    
    if(v->InterpdataRem > 0)
    {
        poscmd = v->Interpdata + 1; 
        v->InterpdataRem = v->InterpdataRem - 1;
    }
    else if(v->InterpdataRem < 0)
    {
        poscmd = v->Interpdata - 1; 
        v->InterpdataRem = v->InterpdataRem + 1;    
    }
    else
    {
        poscmd = v->Interpdata; 
    }
    
    return poscmd;
}

void CMbusPosRefcalc(POS_REF *v)
{
     int64_t temp1;
     int32_t temp2;
     uint8_t syncflag;
     
    if(CM_BusCtrl.Mode == CMBUSMODE_SYNCPOS)
    {
        syncflag = SnycCmdFlag; 
        if(syncflag == 0x01)
        {
            SnycCmdFlag = 0; 
            temp2 = v->PosNow - v->PosOld;
        }
        else
        {
            temp2 = 0;
        }
        v->Delta = SyncPosRef(temp2, v ,syncflag);   
    }
    else if(CM_BusCtrl.Mode == CMBUSMODE_POS)
    {
        if(CM_BusCtrl.ZeroFlag == 0)     
        {
            PosGenerator.Pt = CM_BusCtrl.TargetPosA;
            PosGenerator.SpdInput = labs(CM_BusCtrl.TargetPosVelA);
            PosGenerator.Delta    = labs(CM_BusCtrl.TargetPosAccA);
            PosGenerator.DeltaDec = labs(CM_BusCtrl.TargetPosDecA);
            PosGenerator.calc(&PosGenerator);
            v->PosNow = PosGenerator.Po;
        }
        else //暂停减速
        {
            PosGenerator.Target = 0;
            PosGenerator.DeltaDec = labs(CM_BusCtrl.TargetPosDecA);
            PosGenerator.deltacalc(&PosGenerator);
            temp1 = ((int64)PosGenerator.OutPut * PosRef.DeltaMax >> 24);
            temp2 = (temp1 >> 10);
            PosGenerator.Po += temp2;
            PosGenerator.Potemp = (int64)PosGenerator.Po<<10;
            v->PosNow = PosGenerator.Po;
        }
        v->Delta = v->PosNow - v->PosOld;
    }
    else
    {
        v->Delta = 0;
    }
    
    v->PosOld = v->PosNow;
}

void CMbusHomecalc(POS_REF *v)
{
    if(CM_BusCtrl.HomeStep == 0)
    {
         RamAid->FaultReset = 5;
         CM_BusCtrl.HomeStep = 1;
         CM_BusCtrl.HomeCnt = 0;
    }
    else if(CM_BusCtrl.HomeStep == 1)
    {
        CM_BusCtrl.HomeCnt++;
        if(CM_BusCtrl.HomeCnt == 8000L)
        {
            CM_BusCtrl.HomeStep = 2;
            CM_BusCtrl.HomeCnt = 0;
        }
    }
    else if(CM_BusCtrl.HomeStep >= 2)
    {
        CM_BusCtrl.HomeStep = 3;
        CM_BusCtrl.TargetSyncPos = CM_BusVar.ActualValue.Pos;  //mqb2025.12.03修复回零模式切换到同步位置模式时的bug
        if(FullCloseLoop.FullFlag > 0)
        {
            ExtPosFb.PosFbTotle = 0;
            
            FullCloseLoop.LowPassInput = 0;
            FullCloseLoop.LowPassOutput = 0;
            FullCloseLoop.LowPassOutputLatch = 0;
            FullCloseLoop.LowPassRemainder = 0;

            FullCloseLoop.Err = 0;
        }
        else
        {
            SpdPosFb.PosFbTotle = 0;
            PosFb.PosFbTotle = 0;                      
        } 
    }
    else
    {
        
    }
}

uint16_t CMbus_Stop(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
    Uint16 stopmode = 0;
    
	if(sm->Ready.bit.ERR)
	{
		if(FAULT_DEAL_BIT(FaultP.ErrCode,FAULT_BIT_LEVEL) == FAULT_LEVEL1)//在1ms中进行处理是否时间太短
		{
			stopmode = 0;//1级故障，自由停车
		}
		else
		{
			stopmode = RamPA->SecFaultStop > 0 ? STOPMODEFAULT : 0;  //2级故障，按停车方式停车
		}
	}
    else if(CM_BusCtrl.EstopFlag == 1) //急停
    {
        stopmode = STOPMODEEMER;  
    }
	else
	{
		stopmode = RamPA->NormalStop > 0 ? STOPMODEZERO : 0;//正常，按设定方式停车
	}
    
	return stopmode;   
}
#elif SERVOTYPE == SERVO_MODBUS

#else
TYPE_CMbusConTrol  CM_BusCtrl;

uint16_t CMbus_Stop(void)
{
    return 0;
}

uint8_t CMbusControlMode(void)
{
    return 0;
}

void CMbusPosRefcalc(POS_REF *v)
{
    
}

void CMbusHomecalc(POS_REF *v)
{
    
}

void CanbusMITFbData(void)
{
}

//mqb MIT
int32 Canbus_GetMITOutPut(void)
{
    return 0;
}
#endif
