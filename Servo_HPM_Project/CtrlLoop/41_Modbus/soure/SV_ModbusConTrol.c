#include <stdint.h>
#include <stdlib.h>
#include "SV_FuncVar.h"
#include "SV_AppDriveInterface.h"
#include "SV_ModbusConTrolVar.h"
#include "DrvCoeff.h"
#include "Pos_FullClosedLoop.h"
#include "SV_OTProtect.h"
#include "Pos_Reg.h"
#include "Drive.h"
#include "SV_UdcCtrl.h"
#include "SV_MagEncode.h" 
#include "r_cg_Project.h"//


#if SERVOTYPE == SERVO_MODBUS

#define  GET32VAR_ADDRL(x)   ((uint16_t*)&(x))
#define  GET32VAR_ADDRH(x)   ((uint16_t*)&(x) + 1)

extern int32_t TorQueData;

uint16_t ModbusNull = 0;
TYPE_CMbusComm     CM_BusVar;
TYPE_CMbusConTrol  CM_BusCtrl;

uint16_t* const ModBusConTrolAddres[MODBUSMAXADDRES] = 
{ 
    (uint16_t*)&RamFuncCode.PA[0],                  //    控制模式              0x1A00
    (uint16_t*)&CM_BusVar.Run,                      //    使能                  0x1A01
    (uint16_t*)&CM_BusVar.Halt,                     //    位置暂停              0x1A02
    (uint16_t*)&RamFuncCode.PA[1],                  //    位置运动模式          0x1A03
    (uint16_t*)&CM_BusVar.ExecuteFlag,              //    位置执行              0x1A04
    (uint16_t*)&CM_BusVar.Restart,                  //    重启指令              0x1A05
    (uint16_t*)&RamFuncCode.PA[2],                  //    二级故障停车模式      0x1A06
    (uint16_t*)&RamFuncCode.PA[3],                  //    关使能停车            0x1A07
    (uint16_t*)&RamFuncCode.P8[1],                  //    复位故障              0x1A08
    (uint16_t*)&CM_BusVar.EStop,                    //    急停                  0x1A09
    
    /*************位置模式**************************/
    GET32VAR_ADDRL(CM_BusVar.TargetPos1),           //                          0x1A0A
    GET32VAR_ADDRH(CM_BusVar.TargetPos1),           //                          0x1A0B
    GET32VAR_ADDRL(CM_BusVar.TargetPos2),           //                          0x1A0C
    GET32VAR_ADDRH(CM_BusVar.TargetPos2),           //                          0x1A0D
    GET32VAR_ADDRL(CM_BusVar.TargetSpd_p),          //                          0x1A0E
    GET32VAR_ADDRH(CM_BusVar.TargetSpd_p),          //                          0x1A0F
    (uint16_t*)&RamFuncCode.PA[4],                  //    位置加速度L           0x1A10
    (uint16_t*)&RamFuncCode.PA[5],                  //    位置加速度H           0x1A11
    (uint16_t*)&RamFuncCode.PA[6],                  //    位置减速度L           0x1A12
    (uint16_t*)&RamFuncCode.PA[7],                  //    位置减速度H           0x1A13
    GET32VAR_ADDRL(CM_BusVar.delayTime),            //    位置运动停止时间L     0x1A14
    GET32VAR_ADDRH(CM_BusVar.delayTime),            //    位置运动停止时间H     0x1A15
    /*************位置模式**************************/
    
    /*************速度模式**************************/
    GET32VAR_ADDRL(CM_BusVar.TargetSpd_S),          //    速度模式速度目标值L   0x1A16
    GET32VAR_ADDRH(CM_BusVar.TargetSpd_S),          //    速度模式速度目标值H   0x1A17   
    /*************速度模式**************************/
 
    /*************力矩模式**************************/
    (uint16_t*)&CM_BusVar.TargetTor_M,              //    力矩目标值L           0x1A18 
    (uint16_t*)&ModbusNull,                         //    力矩目标值H           0x1A19 
    (uint16_t*)&RamFuncCode.PA[8],                  //    力矩加速度L           0x1A1A 
    (uint16_t*)&RamFuncCode.PA[9],                  //    力矩加速度H           0x1A1B 
    (uint16_t*)&RamFuncCode.PA[10],                 //    力矩速度限制L         0x1A1C 
    (uint16_t*)&RamFuncCode.PA[11],                 //    力矩速度限制H         0x1A1D 
    /*************力矩模式**************************/  
    
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A1E
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A1F
   
    (uint16_t*)&CM_BusVar.ErrCode,                  //    错误码                0x1A20
    (uint16_t*)&CM_BusVar.Status,                   //    状态码                0x1A21
    
    GET32VAR_ADDRL(CM_BusVar.PosFbAct),             //    位置反馈值L           0x1A22 
    GET32VAR_ADDRH(CM_BusVar.PosFbAct),             //    位置反馈值H           0x1A23    
    GET32VAR_ADDRL(CM_BusVar.VelFbAct),             //    速度反馈值L           0x1A24 
    GET32VAR_ADDRH(CM_BusVar.VelFbAct),             //    速度反馈值H           0x1A25    
    (uint16_t*)&(CM_BusVar.TorFbAct),               //    力矩反馈值L           0x1A26     
    GET32VAR_ADDRL(CM_BusVar.FollowingError),       //    位置偏差反馈值L       0x1A27
    GET32VAR_ADDRH(CM_BusVar.FollowingError),       //    位置偏差反馈值H       0x1A28
    
    GET32VAR_ADDRL(CM_BusVar.PosDemAct),            //    位置需求值L           0x1A29 
    GET32VAR_ADDRH(CM_BusVar.PosDemAct),            //    位置需求值H           0x1A2A  
    GET32VAR_ADDRL(CM_BusVar.VelDemAct),            //    速度需求值L           0x1A2B 
    GET32VAR_ADDRH(CM_BusVar.VelDemAct),            //    速度需求值H           0x1A2C
    GET32VAR_ADDRL(CM_BusVar.TorDemAct),            //    力矩需求值L           0x1A2D 

    GET32VAR_ADDRL(CM_BusVar.SinglePos_Mortor),     //    电机端编码器值L       0x1A2E 
    GET32VAR_ADDRH(CM_BusVar.SinglePos_Mortor),     //    电机端编码器值H       0x1A2F    
    GET32VAR_ADDRL(CM_BusVar.SinglePos_Reduce),     //    减速机端编码器值L     0x1A30 
    GET32VAR_ADDRH(CM_BusVar.SinglePos_Reduce),     //    减速机端编码器值H     0x1A31 
    
    GET32VAR_ADDRL(CM_BusVar.Udc_mv),               //    母线电压L             0x1A32 
    GET32VAR_ADDRH(CM_BusVar.Udc_mv),               //    母线电压H             0x1A33      
    (uint16_t*)&(CM_BusVar.MosTemper),              //    MOS温度               0x1A34 
    
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A35
    GET32VAR_ADDRL(TorQueData),                     //    rsd                   0x1A36
    GET32VAR_ADDRL(TorQueData),                     //    rsd                   0x1A37
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A38
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A39
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3A
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3B
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3C
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3D
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3E
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A3F
    
    (uint16_t*)&RamFuncCode.PA[21],                 //    位置到达精度          0x1A40
    (uint16_t*)&RamFuncCode.PA[22],                 //    位置到达时间          0x1A41
    (uint16_t*)&RamFuncCode.PA[12],                 //    位置正极限L           0x1A42
    (uint16_t*)&RamFuncCode.PA[13],                 //    位置正极限H           0x1A43
    (uint16_t*)&RamFuncCode.PA[14],                 //    位置负极限L           0x1A44
    (uint16_t*)&RamFuncCode.PA[15],                 //    位置负极限H           0x1A45
    
    (uint16_t*)&RamFuncCode.PA[18],                 //    速度最大值L           0x1A46   
    (uint16_t*)&RamFuncCode.PA[19],                 //    速度最大值H           0x1A47
    
    (uint16_t*)&RamFuncCode.PA[20],                 //    力矩最大值            0x1A48
    
    (uint16_t*)&RamFuncCode.PA[16],                 //    位置偏差阈值L         0x1A49
    (uint16_t*)&RamFuncCode.PA[17],                 //    位置偏差阈值H         0x1A4A
    
    (uint16_t*)&RamFuncCode.P1[18],                 //    零速停车减速度        0x1A4B  
    (uint16_t*)&RamFuncCode.P1[20],                 //    二级故障零速停车时间  0x1A4C    
    (uint16_t*)&RamFuncCode.P7[24],                 //    Modbus ID             0x1A4D 
    (uint16_t*)&RamFuncCode.P7[25],                 //    rsd                   0x1A4E
    (uint16_t*)&RamFuncCode.P7[26],                 //    rsd                   0x1A4F 
    
    (uint16_t*)&RamFuncCode.P1[61],                 //    保存                  0x1A50  
    
    (uint16_t*)&RamFuncCode.P1[19],                 //    极限停车时间          0x1A51
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A52
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A53
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A54
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A55
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A56
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A57
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A58
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A59
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5A
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5B
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5C
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5D
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5E
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A5F
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A60
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A61
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A62
    (uint16_t*)&ModbusNull,                         //    rsd                   0x1A63
};

TYPE_AttRI_Modbus AttCode_Modbus[MODBUSMAXADDRES] =
{
    /*符号 Size    RW    MAX        MIN  */
    {0x00, 0x02,  0x00, 0x0004,   0x0001},  //    控制模式              0x1A00
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    使能                  0x1A01
    {0x00, 0x02,  0x00, 0x0002,   0x0000},  //    位置暂停              0x1A02
    {0x00, 0x02,  0x00, 0x0003,   0x0000},  //    位置运动模式          0x1A03
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    位置执行              0x1A04
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    重启指令              0x1A05
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    二级故障停车模式      0x1A06
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    关使能停车            0x1A07
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    复位故障              0x1A08
    {0x00, 0x02,  0x00, 0x0001,   0x0000},  //    急停                  0x1A09
    
    /*************位置模式**************************/
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置指令1L            0x1A0A
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置指令1H            0x1A0B
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置指令2L            0x1A0C
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置指令2H            0x1A0D
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置速度L             0x1A0E
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置速度H             0x1A0F
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置加速度L           0x1A10
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置加速度H           0x1A11
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置加速度L           0x1A12
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置加速度H           0x1A13
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置运动停止时间L     0x1A14
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置运动停止时间H     0x1A15
     /*************位置模式*************************/   
    
    /*************速度模式**************************/
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    速度模式速度目标值L   0x1A16
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    速度模式速度目标值H   0x1A17 
    /*************速度模式**************************/

    /*************力矩模式**************************/    
    {0x01, 0x02,  0x00, 0x0BB8,   0xF448},  //    力矩                  0x1A18 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A19
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    力矩加速度L           0x1A1A 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    力矩加速度H           0x1A1B    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    力矩速度限制L         0x1A1C   
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    力矩速度限制H         0x1A1D 
    /*************力矩模式**************************/  
    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A1E
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A1F
    
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    错误码                0x1A20
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    状态码                0x1A21
    
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    位置反馈值L           0x1A22
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    位置反馈值H           0x1A23  
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    速度反馈值L           0x1A24 
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    速度反馈值H           0x1A25 

    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    力矩反馈值L           0x1A26 
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    位置偏差反馈值L       0x1A27   
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    位置偏差反馈值H       0x1A28
    
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    位置需求值L           0x1A29 
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    位置需求值H           0x1A2A  
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    速度需求值L           0x1A2B 
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    速度需求值H           0x1A2C
    {0x01, 0x02,  0x01, 0x7FFF,   0x8000},  //    力矩需求值L           0x1A2D 

    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    电机端编码器值L       0x1A2E 
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    电机端编码器值H       0x1A2F    
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    减速机端编码器值L     0x1A30 
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    减速机端编码器值H     0x1A31 
    
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    母线电压L             0x1A32 
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    母线电压H             0x1A33      
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    MOS温度               0x1A34 

    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A35
    {0x00, 0x02,  0x01, 0xFFFF,   0x0000},  //    rsd                   0x1A36
    {0x01, 0x02,  0x01, 0xFFFF,   0x0000},  //    rsd                   0x1A37
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A38
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A39
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3A
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3B
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3C
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3D
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3E
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A3F
    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A40
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A41
    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置正极限L           0x1A42
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置正极限H           0x1A43 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置负极限L           0x1A44
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置负极限H           0x1A45

    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    速度最大值L           0x1A46
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    速度最大值H           0x1A47     
    {0x01, 0x02,  0x00, 0x0BB8,   0xF448},  //    力矩最大值            0x1A48   
    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    位置偏差阈值L         0x1A49
    {0x01, 0x02,  0x00, 0x7FFF,   0x8000},  //    位置偏差阈值H         0x1A4A 

    {0x00, 0x02,  0x00, 0x7530,   0x0032},  //    零速停车减速度        0x1A4B
    {0x00, 0x02,  0x00, 0x7530,   0x0032},  //    二级故障零速停车时间  0x1A4C  
    {0x00, 0x02,  0x00, 0x00FF,   0x0001},  //    Modbus ID             0x1A4D 
    {0x00, 0x02,  0x00, 0x0005,   0x0000},  //    Modbus BAUD           0x1A4E 
    {0x00, 0x02,  0x00, 0x0005,   0x0000},  //    Modbus FRAME          0x1A4F
    
    {0x00, 0x02,  0x00, 0x0042,   0x0037},  //    保存                  0x1A50 
    {0x00, 0x02,  0x00, 0x7530,   0x0032},  //    极限停车              0x1A51
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A52
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A53
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A54
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A55
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A56 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A57
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A58
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A59
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5A
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5B
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5C 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5D
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5E
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A5F
    
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A60
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A61
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A62 
    {0x00, 0x02,  0x00, 0xFFFF,   0x0000},  //    rsd                   0x1A63
};

void CM_busCtrl_init(TYPE_CMbusConTrol *v)
{
    double temp1,temp2,temp3,temp4;    
    
    temp1 = (60.0 * 16777216.0)/((float64)DrvCoeff.MotEncSglRevLns*DrvCoeff.SpdBase); 

    temp2 = DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdBase;
    temp3 = 60.0 * 16777216.0;
    temp4 = temp2 / temp3;
    
    if(FullCloseLoop.FullFlag > 0)
    {
        v->Pos_SpdCmdCoeff = temp1 * FullCloseLoop.ExtRev / DrvCoeff.MotEncSglRevLns; //转换到电机端的编码器单位;              
        v->Spd_CmdCoeff    = temp1 * FullCloseLoop.InnerRev / FullCloseLoop.ExtRev;  
        v->Spd_FbCoeff     = temp4 * FullCloseLoop.ExtRev / FullCloseLoop.InnerRev;
    }
    else
    {
        v->Pos_SpdCmdCoeff = temp1;
        v->Spd_CmdCoeff    = temp1;       
        v->Spd_FbCoeff     = temp4;
    }
    
    temp2 = (uint64_t)DrvCoeff.MotEncSglRevLns * DrvCoeff.SpdBase * 100000000L;
    temp1 = 2147483648.0 * 60.0 * DrvCoeff.PosTsamp;                      
    temp1 = temp1 / temp2;                                   //加减速扩大128
   
    temp2 = temp1 * DrvCoeff.SpdTsamp / DrvCoeff.PosTsamp;
    
    if(FullCloseLoop.FullFlag > 0)
    {
        temp1 = temp1 * FullCloseLoop.ExtRev / DrvCoeff.MotEncSglRevLns;
        v->AccDec_Coeff_P = temp1;
        
        temp2 = temp2 * FullCloseLoop.InnerRev / FullCloseLoop.ExtRev;
        v->AccDec_Coeff_S = temp2;        
    }
    else
    {
        v->AccDec_Coeff_S = temp2;
        v->AccDec_Coeff_P = temp1;
    }
    
    v->CntMax_1S = (int64_t)100000000 / DrvCoeff.SpdTsamp;
    v->PosCoeff_1ms = 100000L / DrvCoeff.PosTsamp;
    
    v->MaxTorque_internal = RamServo->TorqLimt;
    
    if(FullCloseLoop.FullFlag > 0)
    {
        temp1 = (double)RamMotor->Nmax / RamPD->Reduction;
        temp1 = temp1 * FullCloseLoop.ExtRev / 60.0;
        v->MaxSpd_internal = (int32_t)temp1;
    }
    else
    {
        v->MaxSpd_internal = (uint64_t)RamMotor->Ne * DrvCoeff.MotEncSglRevLns / 60L;
    }   
}

void CM_busCtrl_updata(TYPE_CMbusConTrol *v)
{
    if(RamPA->MaxSpd > CM_BusCtrl.MaxSpd_internal)
        CM_BusCtrl.MaxSpd = CM_BusCtrl.MaxSpd_internal;
    else
        CM_BusCtrl.MaxSpd = (int32_t)RamPA->MaxSpd;
    
    if(RamPA->MaxTor > CM_BusCtrl.MaxTorque_internal)
        CM_BusCtrl.MaxTorque = CM_BusCtrl.MaxTorque_internal;
    else
        CM_BusCtrl.MaxTorque = RamPA->MaxTor;
}

void CM_busCtrl_rst(TYPE_CMbusConTrol *v)
{
    if(FullCloseLoop.FullFlag > 0)
    {
    	v->TargetPosA = ExtPosFb.PosFbTotle;
    }
    else
    {
	    v->TargetPosA = PosFb.PosFbTotle;
    }
    
    v->TargetPosBuff[0] = v->TargetPosA;
    v->TargetPosBuff[1] = v->TargetPosA;
    CM_BusCtrl.DelayCnt = 0;
    CM_BusCtrl.PosTurnFlag = 0;
    CM_BusCtrl.MuliSeg = 0;
    CM_BusCtrl.PosLimFlag = 0;
    CM_BusCtrl.HomeStep = 0;
} 

uint8_t CMbusControlMode(void)
{
    uint8_t flag = 0;
    
    if(RamPA->ModbusDrvMode > 4)  RamPA->ModbusDrvMode = 0;
   
    switch(RamPA->ModbusDrvMode)
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
       default: break;
    }
   
    return flag;
}

void ModbusPosGetData(void)
{
    int32_t pulse;
    int32_t v_user;
    int32_t acc_user;
    
    TYPE_STATEMACHINE *sm = &StateMachine;
    static uint8_t OtpFlag1 = 0;
    static uint8_t OtpFlag2 = 0;
      
    if(CM_BusVar.Halt == 1)
    {
        CM_BusCtrl.ZeroFlag = 1;
        CM_BusVar.Halt = 0;
    }
    else if(CM_BusVar.Halt == 2)//执行完毕当前的目标值
    {
        CM_BusCtrl.MuliStopFlag = 1;
        CM_BusVar.Halt = 0;
    }
    
    if(CM_BusVar.Restart == 1)
    {
        CM_BusVar.Halt = 0;
        CM_BusVar.Restart = 0; 
        CM_BusCtrl.ZeroFlag = 0;
        CM_BusCtrl.MuliStopFlag = 0;
    }
    
    if(CM_BusVar.ExecuteFlag == 1)
    {
        CM_BusCtrl.PosTurn = RamPA->RunMode;
            
        if(CM_BusVar.TargetSpd_p >= 0)
        {
            if(CM_BusVar.TargetSpd_p > CM_BusCtrl.MaxSpd)
            {
                v_user = CM_BusCtrl.MaxSpd;    
            }
            else
            {
                v_user = CM_BusVar.TargetSpd_p;    
            }
        }
        else
        {
            if(CM_BusVar.TargetSpd_p < -CM_BusCtrl.MaxSpd)
            {
                v_user = -CM_BusCtrl.MaxSpd;    
            }
            else
            {
                v_user = CM_BusVar.TargetSpd_p;    
            }        
        }
        
        v_user = (int32_t)((double)v_user * CM_BusCtrl.Pos_SpdCmdCoeff);
        CM_BusCtrl.TargetPosVelA = v_user;
        
        if(CM_BusCtrl.PosTurn == POSABSOLUTE)            //绝对位置     
        {   
            pulse = CM_BusVar.TargetPos1;
            if((pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosA; 
                OtpFlag1 = 1;
                FaultPrtt_FaultInterface(POTWarn);
            }
            else if((pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
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
            CM_BusCtrl.PosTurnFlag = POSABSOLUTE;
        }
        else if(CM_BusCtrl.PosTurn == POSRELATIVE)       //相对位置  
        {
            pulse = CM_BusCtrl.TargetPosA + CM_BusVar.TargetPos1;
            
            if((pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosA; 
                OtpFlag1 = 1;
                FaultPrtt_FaultInterface(POTWarn);
            }
            else if((pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
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
            CM_BusCtrl.PosTurnFlag = POSRELATIVE;
        }
        else if(CM_BusCtrl.PosTurn == POSCYCLE)     //往返位置 
        {
            pulse = CM_BusVar.TargetPos1;
            if((pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosBuff[0]; 
                OtpFlag1 = 1;
                FaultPrtt_FaultInterface(POTWarn);
            }
            else if((pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosBuff[0]; 
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
            CM_BusCtrl.TargetPosBuff[0]= pulse;
            
            pulse = CM_BusVar.TargetPos2;
            if((pulse > RamPA->POS_PLim) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosBuff[1]; 
                OtpFlag2 = 1;
                FaultPrtt_FaultInterface(POTWarn);
            }
            else if((pulse < RamPA->POS_NLim) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
            {
                pulse = CM_BusCtrl.TargetPosBuff[1]; 
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
            
            CM_BusCtrl.TargetPosBuff[1]= pulse;
            CM_BusCtrl.MuliSeg = 0; 
            CM_BusCtrl.DelayCnt = 0;
            CM_BusCtrl.PosTurnFlag = POSCYCLE;
        }
        else if(CM_BusCtrl.PosTurn == POSCONTINUOUS)       //连续运动 
        {
            CM_BusCtrl.PosTurnFlag = POSCONTINUOUS;                           
        }
        else
        {
            
        }

        CM_BusVar.ExecuteFlag = 0;
        CM_BusCtrl.ZeroFlag = 0;
        CM_BusCtrl.MuliStopFlag = 0;
    }
    
    if(CM_BusCtrl.PosTurnFlag == POSCONTINUOUS)
    {
        pulse = CM_BusCtrl.PosFbActualValue;
        if((pulse > RamPA->POS_PLim) && (CM_BusVar.TargetSpd_p > 0) && sm->SrvOnStatus && (RamPA->POS_PLim != 0))
        {
            OtpFlag1 = 1;
            FaultPrtt_FaultInterface(POTWarn);
            CM_BusCtrl.TargetPosVelA = 0;
        }
        else if((pulse < RamPA->POS_NLim) && (CM_BusVar.TargetSpd_p < 0) && sm->SrvOnStatus && (RamPA->POS_NLim != 0))
        {
            OtpFlag1 = 1;
            FaultPrtt_FaultInterface(NOTWarn);
            CM_BusCtrl.TargetPosVelA = 0;
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
    }

    acc_user = (int32_t)((double)RamPA->PosAcc * CM_BusCtrl.AccDec_Coeff_P);
    if(acc_user == 0) acc_user = 1;
    CM_BusCtrl.TargetPosAccA = acc_user;
    
    acc_user = (int32_t)((double)RamPA->PosDec * CM_BusCtrl.AccDec_Coeff_P);
    if(acc_user == 0) acc_user = 1;
    CM_BusCtrl.TargetPosDecA = acc_user;   
    
    if(CM_BusVar.delayTime > 268435456L) CM_BusVar.delayTime = 268435456L;
    CM_BusCtrl.DelayCntAll = CM_BusVar.delayTime * CM_BusCtrl.PosCoeff_1ms;
    
    if(((OtpFlag1 == 1) || (OtpFlag2 == 1)) && (sm->SrvOnStatus == 0))
    {
        OtpFlag1 = 0;
        OtpFlag2 = 0;
        FaultPrtt_WarnInterfaceClr(POTWarn);
        FaultPrtt_WarnInterfaceClr(NOTWarn);
    }    
}

int32_t Modbus_FollowErr(void)
{
	int32 tmp1 = 0;
	uint32_t tmp = 0;
    static uint8_t  Errcnt = 0;
    static uint8_t  PosReachedTime = 0;

	TYPE_STATEMACHINE *sm = &StateMachine;
	TYPE_OTPROTECT *ot = &OTP;
    
    if(CM_BusCtrl.Mode != CMBUSMODE_POS)
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

void ModbusSpdGetData(void)
{
    int32_t v_user;
    int32_t acc_user;
    
    if(CM_BusVar.TargetSpd_S >= 0)
    {
        if(CM_BusVar.TargetSpd_S > CM_BusCtrl.MaxSpd)
        {
            v_user = CM_BusCtrl.MaxSpd;    
        }
        else
        {
            v_user = CM_BusVar.TargetSpd_S;    
        }         
    }
    else
    {
        if(CM_BusVar.TargetSpd_S < -CM_BusCtrl.MaxSpd)
        {
            v_user = -CM_BusCtrl.MaxSpd;    
        }
        else
        {
            v_user = CM_BusVar.TargetSpd_S;    
        }         
    }
    
    v_user = (int32_t)((double)v_user * CM_BusCtrl.Spd_CmdCoeff);
    CM_BusCtrl.TargetSpdVelA = v_user;
    
    acc_user = (int32_t)((double)RamPA->PosAcc * CM_BusCtrl.AccDec_Coeff_S);
    if(acc_user == 0) acc_user = 1;
    CM_BusCtrl.TargetSpdAccA = acc_user;
    
    acc_user = (int32_t)((double)RamPA->PosAcc * CM_BusCtrl.AccDec_Coeff_S);
    if(acc_user == 0) acc_user = 1;
    CM_BusCtrl.TargetSpdDecA = acc_user;
    
    if(CM_BusVar.ExecuteFlag == 1) CM_BusVar.ExecuteFlag = 0;
}

void ModbusTorqueGetData(void)
{
    int32_t acc_user;
    int32_t v_user;
    
    float temp;
    
    if(CM_BusVar.TargetTor_M  >= 0)
    {
        if(CM_BusVar.TargetTor_M > CM_BusCtrl.MaxTorque)
            CM_BusCtrl.TargetTorqueA = CM_BusCtrl.MaxTorque; 
        else
            CM_BusCtrl.TargetTorqueA = CM_BusVar.TargetTor_M;
    }
    else
    {
        if(CM_BusVar.TargetTor_M < -CM_BusCtrl.MaxTorque)
            CM_BusCtrl.TargetTorqueA = -CM_BusCtrl.MaxTorque; 
        else
            CM_BusCtrl.TargetTorqueA = CM_BusVar.TargetTor_M;        
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
    
    if(CM_BusVar.ExecuteFlag == 1) CM_BusVar.ExecuteFlag = 0;
}

void ModbusLim_Cacl(void)
{
    int32_t Min_PositionLimit,Max_PositionLimit,PosFb; 
    TYPE_STATEMACHINE *sm = &StateMachine; 
    
    Min_PositionLimit = RamPA->POS_NLim;
	Max_PositionLimit = RamPA->POS_PLim;

    PosFb = CM_BusCtrl.PosFbActualValue;
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

void Modbus_GetCmd(void)
{
    CM_BusCtrl.Run = CM_BusVar.Run;
    CM_BusCtrl.ModbusSel = ModbusVal.Conflg;
    
    if(CM_BusVar.EStop)
    {
        CM_BusCtrl.EstopFlag = 1;
        CM_BusVar.Run = 0;
    }
    else
    {
        CM_BusCtrl.EstopFlag = 0;        
    }
    
    switch(CM_BusCtrl.Mode)
    {
        case CMBUSMODE_POS:
            ModbusPosGetData();
//            if(CM_BusCtrl.PosTurn == POSCONTINUOUS)
//            {
//                ModbusLim_Cacl();    
//            }
        break;
        
        case CMBUSMODE_SPD:
            ModbusSpdGetData();
            ModbusLim_Cacl();
        break;
        
        case CMBUSMODE_TOR:
            ModbusTorqueGetData();
            ModbusLim_Cacl();
        break;
        
        default:break;
    }
}

void Modbus_GetFbData(void)
{
    if(FullCloseLoop.FullFlag == 1)
    {
        CM_BusVar.PosFbAct = ExtPosFb.PosFbTotle;
        CM_BusVar.VelFbAct = SpdFb.Fb; 
    }
    else
    {
        CM_BusVar.PosFbAct = PosFb.PosFbTotle;
        CM_BusVar.VelFbAct = SpdFb.Fb; 
    }    
    CM_BusCtrl.PosFbActualValue = CM_BusVar.PosFbAct;
    CM_BusVar.TorFbAct = CurMonitor.TorqRatsDispS;    
    
    CM_BusVar.PosDemAct = PosRef.PosNow;
    CM_BusVar.VelDemAct = Spd_Ref.Ref;
    CM_BusVar.TorDemAct = CurMonitor.TorqRatsRef; 
}

void CMbusPosRefcalc(POS_REF *v)
{
     int64_t temp1;
     int32_t temp2;
        
    if(CM_BusCtrl.ZeroFlag == 0 && CM_BusCtrl.PosLimFlag == 0)     
    {
        if(CM_BusCtrl.PosTurnFlag == POSCYCLE)           //往返位置 
        {
            PosGenerator.Pt = CM_BusCtrl.TargetPosA;
            PosGenerator.SpdInput = labs(CM_BusCtrl.TargetPosVelA);
            PosGenerator.Delta    = labs(CM_BusCtrl.TargetPosAccA);
            PosGenerator.DeltaDec = labs(CM_BusCtrl.TargetPosDecA);
            PosGenerator.calc(&PosGenerator);
            v->PosNow = PosGenerator.Po;
            
            if((PosGenerator.Pgen.bit.PcmpFlg == 1) && (CM_BusCtrl.MuliStopFlag < 2))
            {
                CM_BusCtrl.DelayCnt++; 
                if(CM_BusCtrl.DelayCnt >= CM_BusCtrl.DelayCntAll)
                {
                    if(CM_BusCtrl.MuliStopFlag == 0)  //正常运动
                    {
                        CM_BusCtrl.TargetPosA = CM_BusCtrl.TargetPosBuff[CM_BusCtrl.MuliSeg];
                        CM_BusCtrl.MuliSeg++;  
                        if(CM_BusCtrl.MuliSeg == 2) CM_BusCtrl.MuliSeg = 0;
                        CM_BusCtrl.DelayCnt = 0;
                    }
                    else                              //暂停
                    {
                        CM_BusCtrl.MuliStopFlag = 2;
                        CM_BusCtrl.DelayCnt = CM_BusCtrl.DelayCntAll - 1;
                    }
                    PosGenerator.Pgen.bit.PcmpFlg = 0;
                }
            }
        }
        else if(CM_BusCtrl.PosTurnFlag == POSCONTINUOUS)       //连续运动 
        {
            PosGenerator.Target = CM_BusCtrl.TargetPosVelA;
            PosGenerator.Delta    = labs(CM_BusCtrl.TargetPosAccA);
            PosGenerator.DeltaDec = labs(CM_BusCtrl.TargetPosDecA);
            PosGenerator.deltacalc(&PosGenerator);
            temp1 = ((int64)PosGenerator.OutPut * PosRef.DeltaMax >> 24);
            temp2 = (temp1 >> 10);
            PosGenerator.Po += temp2;
            PosGenerator.Pt           = PosGenerator.Po;
            CM_BusCtrl.TargetPosA  = PosGenerator.Po; 
            PosGenerator.Potemp = (int64)PosGenerator.Po<<10;
            
            v->PosNow = PosGenerator.Po;            
        }
        else                                       //绝对位置模式或者相对位置模式
        {
            PosGenerator.Pt = CM_BusCtrl.TargetPosA;
            PosGenerator.SpdInput = labs(CM_BusCtrl.TargetPosVelA);
            PosGenerator.Delta    = labs(CM_BusCtrl.TargetPosAccA);
            PosGenerator.DeltaDec = labs(CM_BusCtrl.TargetPosDecA);
            PosGenerator.calc(&PosGenerator);
            v->PosNow = PosGenerator.Po;
        }
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
    v->PosOld = v->PosNow;
    v->PosRefTotle = v->PosNow;
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

void Modbus_StatusFeedBack(void)
{
    CM_BusVar.FollowingError = Modbus_FollowErr();    
    CM_BusVar.Udc_mv = (((int64_t)AdRead.Udc * UDC_VOLTBASE * 1000L) >> 24);
    CM_BusVar.MosTemper = RamMonitor->TIPM;
    CM_BusVar.SinglePos_Mortor = EncPosFb.EncSinglePos;
    CM_BusVar.SinglePos_Reduce = MagReg.Pos;
    
    CM_BusCtrl.Status.bit.RunFlag    = StateMachine.SrvOnStatus > 0 ? 1 : 0;
    CM_BusCtrl.Status.bit.EstopFlag  = CM_BusCtrl.EstopFlag > 0 ? 1 : 0;
    CM_BusCtrl.Status.bit.HaltFlag   =  (CM_BusCtrl.ZeroFlag > 0 || CM_BusCtrl.MuliStopFlag > 0) ? 1 : 0;
    CM_BusCtrl.Status.bit.HomeOkFlag = (CM_BusCtrl.HomeStep == 3) ? 1 : 0;
    
    if(FaultP.FaultStatus & 0x3)//有警告或者故障发生
	{
        CM_BusVar.ErrCode = FaultP.FaultDisp;
        CM_BusCtrl.Status.bit.ErrStatus = FaultP.FaultStatus;
        if(FaultP.FaultStatus & 0x0002) CM_BusVar.Run = 0;
	}
	else
	{
		CM_BusVar.ErrCode = 0;
        CM_BusCtrl.Status.bit.ErrStatus = 0;
	}
    
    CM_BusVar.Status = CM_BusCtrl.Status.all;
}

void ModBus_Process(void)
{
    Modbus_RXProcess();
        
    Modbus_GetCmd(); 
    
    Modbus_GetFbData();
    
    Modbus_StatusFeedBack();
    
    Modbus_TXProcess();
}

//mqb MIT
int32 Canbus_GetMITOutPut(void)
{
    return 0;
}


#endif


