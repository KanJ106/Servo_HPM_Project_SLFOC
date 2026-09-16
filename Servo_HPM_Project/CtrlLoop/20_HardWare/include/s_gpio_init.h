#ifndef GPIO_H
#define GPIO_H
#include "r_cg_Project.h"
#include "MCU_Fw.h"

#if SERVO_MCU != NO_MCU

#define CRENLEDPORT   GPIO_DI_GPIOC
#define CRENLEDINDEX  19

//------------------------------SV_Di.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
//端口数量
#define DI_NUM_PORT         8       //DI数字输入端口总数
#define DI_INAVALID         0       //无效
#define DI_AVALID           1       //有效
//输入
#define GetX1_Pin       (0)
#define GetX2_Pin       (0)
#define GetX3_Pin       (0)
#define GetX4_Pin       (0)
#define GetX5_Pin       (0)
#define GetX6_Pin       (0)
#define GetX7_Pin       (0)
#define GetX8_Pin       (0)
#endif

#if SERVO_HARDWARE == HARDWARE_DC0
//端口数量
#define DI_NUM_PORT         5       //DI数字输入端口总数
#define DI_INAVALID         0       //无效
#define DI_AVALID           1       //有效
//输入
#define GetX1_Pin       (0)
#define GetX2_Pin       (0)
#define GetX3_Pin       (0)
#define GetX4_Pin       (0)
#define GetX5_Pin       (0)
#endif
//------------------------------SV_Di.h-------------------------------------//

//------------------------------SV_D0.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
//端口数量
#define DO_NUM_PORT     4   //数字输出端口总数
#define DO_INAVALID     0   //DO状态无效
#define DO_AVALID       1   //DO状态有效
#define DO_DELAY        0   //端子延时功能无效

#define SetY1_Valid     do{}while(0)
#define SetY2_Valid     do{}while(0)
#define SetY3_Valid     do{}while(0)
#define SetY4_Valid     do{}while(0)

#define SetY1_Invaild   do{}while(0)
#define SetY2_Invaild   do{}while(0)
#define SetY3_Invaild   do{}while(0)
#define SetY4_Invaild   do{}while(0)
#endif


#if SERVO_HARDWARE == HARDWARE_DC0
//端口数量
#define DO_NUM_PORT     2   //数字输出端口总数
#define DO_INAVALID     0   //DO状态无效
#define DO_AVALID       1   //DO状态有效
#define DO_DELAY        0   //端子延时功能无效

#define SetY1_Valid     do{}while(0)
#define SetY2_Valid     do{}while(0)

#define SetY1_Invaild   do{}while(0)
#define SetY2_Invaild   do{}while(0)


#define SM_BRAKECTRL_OUT1_LOW    do{}while(0)
#define SM_BRAKECTRL_OUT1_HIGH   do{}while(0)

#define SM_BRAKECTRL_OUT2_LOW    do{HPM_GPIO0->DO[GPIO_DI_GPIOC].CLEAR = (1 << 22);}while(0)
#define SM_BRAKECTRL_OUT2_HIGH   do{HPM_GPIO0->DO[GPIO_DI_GPIOC].SET   = (1 << 22);}while(0)

#endif
//------------------------------SV_Do.h-------------------------------------//

//------------------------------SV_FanCtl.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
#define FAN_ON                   do{}while(0)
#define FAN_OFF                  do{}while(0)
#endif
//------------------------------SV_FanCtl.h-------------------------------------//


//------------------------------SV_UdcCtrl.h-------------------------------------//
#if (SERVO_HARDWARE == HARDWARE_AC0)
#define UDC_SOFTON()            do{}while(0)
#define UDC_SOFTOFF()           do{}while(0)
#endif

#if (SERVO_HARDWARE == HARDWARE_DC0)
#define UDC_SOFTON()            do{}while(0)
#define UDC_SOFTOFF()           do{}while(0)
#endif
//------------------------------SV_UdcCtrl.h-------------------------------------//

//------------------------------SV_Fault.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
#define FJ_ILP_SIGNAL           do{}while(0)
#endif
//------------------------------SV_Fault.h-------------------------------------//

//------------------------------SV_SCI.h-------------------------------------//
//------------------------------SV_SCI.h-------------------------------------//

//------------------------------PWM-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_DC0
#define PWM_OFF()               do{HPM_GPIO0->DO[GPIO_DI_GPIOC].CLEAR = 1 << 23;}while(0) 
#define PWM_ON()                do{HPM_GPIO0->DO[GPIO_DI_GPIOC].SET = 1 << 23;}while(0)             
#endif

#if SERVO_HARDWARE == HARDWARE_AC0
#define PWM_OFF()               do{}while(0)
#define PWM_ON()                do{}while(0)
#endif
//------------------------------PWM-------------------------------------//
#else
//------------------------------SV_Di.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
//端口数量
#define DI_NUM_PORT         8       //DI数字输入端口总数
#define DI_INAVALID         0       //无效
#define DI_AVALID           1       //有效
//输入
#define GetX1_Pin       (0)
#define GetX2_Pin       (0)
#define GetX3_Pin       (0)
#define GetX4_Pin       (0)
#define GetX5_Pin       (0)
#define GetX6_Pin       (0)
#define GetX7_Pin       (0)
#define GetX8_Pin       (0)
#endif

#if SERVO_HARDWARE == HARDWARE_DC0
//端口数量
#define DI_NUM_PORT         5       //DI数字输入端口总数
#define DI_INAVALID         0       //无效
#define DI_AVALID           1       //有效
//输入
#define GetX1_Pin       (0)
#define GetX2_Pin       (0)
#define GetX3_Pin       (0)
#define GetX4_Pin       (0)
#define GetX5_Pin       (0)
#endif
//------------------------------SV_Di.h-------------------------------------//

//------------------------------SV_D0.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
//端口数量
#define DO_NUM_PORT     4   //数字输出端口总数
#define DO_INAVALID     0   //DO状态无效
#define DO_AVALID       1   //DO状态有效
#define DO_DELAY        0   //端子延时功能无效

#define SetY1_Valid     do{}while(0)
#define SetY2_Valid     do{}while(0)
#define SetY3_Valid     do{}while(0)
#define SetY4_Valid     do{}while(0)

#define SetY1_Invaild   do{}while(0)
#define SetY2_Invaild   do{}while(0)
#define SetY3_Invaild   do{}while(0)
#define SetY4_Invaild   do{}while(0)
#endif


#if SERVO_HARDWARE == HARDWARE_DC0
//端口数量
#define DO_NUM_PORT     2   //数字输出端口总数
#define DO_INAVALID     0   //DO状态无效
#define DO_AVALID       1   //DO状态有效
#define DO_DELAY        0   //端子延时功能无效

#define SetY1_Valid     do{}while(0)
#define SetY2_Valid     do{}while(0)

#define SetY1_Invaild   do{}while(0)
#define SetY2_Invaild   do{}while(0)


#define SM_BRAKECTRL_OUT1_LOW    do{}while(0)
#define SM_BRAKECTRL_OUT1_HIGH   do{}while(0)

#define SM_BRAKECTRL_OUT2_LOW    do{}while(0)
#define SM_BRAKECTRL_OUT2_HIGH   do{}while(0)

#define PWM_BRAKE_LOW            do{}while(0)
#define PWM_BRAKE_HIGH           do{}while(0)
#endif
//------------------------------SV_Do.h-------------------------------------//

//------------------------------SV_FanCtl.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
#define FAN_ON                   do{}while(0)
#define FAN_OFF                  do{}while(0)
#endif
//------------------------------SV_FanCtl.h-------------------------------------//


//------------------------------SV_UdcCtrl.h-------------------------------------//
#if (SERVO_HARDWARE == HARDWARE_AC0)
#define UDC_SOFTON()            do{}while(0)
#define UDC_SOFTOFF()           do{}while(0)
#endif

#if (SERVO_HARDWARE == HARDWARE_DC0)
#define UDC_SOFTON()            do{}while(0)
#define UDC_SOFTOFF()           do{}while(0)
#endif
//------------------------------SV_UdcCtrl.h-------------------------------------//

//------------------------------SV_Fault.h-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_AC0
#define FJ_ILP_SIGNAL           do{}while(0)
#endif
//------------------------------SV_Fault.h-------------------------------------//

//------------------------------SV_SCI.h-------------------------------------//
#define SCI_SelectToRx()        do{}while(0)
#define SCI_SelectToTx()        do{}while(0)
//------------------------------SV_SCI.h-------------------------------------//

//------------------------------PWM-------------------------------------//
#if SERVO_HARDWARE == HARDWARE_DC0
#define PWM_OFF()               do{}while(0)
#define PWM_ON()                do{}while(0)
#endif

#if SERVO_HARDWARE == HARDWARE_AC0
#define PWM_OFF()               do{}while(0)
#define PWM_ON()                do{}while(0)
#endif
//------------------------------PWM-------------------------------------//

#endif


void port_init(void);

#endif
