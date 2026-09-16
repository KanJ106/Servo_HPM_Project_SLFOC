#ifndef _r_cg_project_H
#define _r_cg_project_H

/***********************************************************************************************************************
User definitions
***********************************************************************************************************************/
#include <stdint.h>

#define     NO_MCU     0
#define     TI_C2000   1
#define     ST_STM32   2
#define     RZ_RZT1    3
#define     HPM_6E00   4
#define     SERVO_MCU  HPM_6E00

#define     UR_11   5
#define     UR_14   0
#define     UR_17   1
#define     UR_20   2
#define     UR_25   3
#define     UR_32   4
#define     Other1  6
#define     SERVO_PRODUCTNUM  UR_17

//选择控制板硬件版本
#define     HARDWARE_AC0    0       //交流总线 
#define     HARDWARE_DC0    1       //低压总线 
#define     HARDWARE_AC1    2       //交流脉冲
#define     HARDWARE_DC1    3       //低压脉冲
#define     SERVO_HARDWARE  HARDWARE_DC0

//选择伺服是否低压伺服
#define     SERVO_DC   0            //低压伺服
#define     SERVO_AC   1            //交流伺服
#define     SERVOPOWER SERVO_DC     //交流伺服

//设置伺服驱动器类型
#define     SERVO_PULSE    0                //0:脉冲型伺服
#define     SERVO_ETHERCAT 1                //1:EhterCAT型伺服
#define     SERVO_CANOPEN  2                //2:CANOPEN型伺服
#define     SERVO_MODBUS   3                //3:Modbus型伺服
#define     SERVO_CAN      4                //4:CAN型伺服

//#define     SERVOTYPE SERVO_ETHERCAT        //总线型
//#define     SERVOTYPE SERVO_PULSE           //脉冲型
#define     SERVOTYPE SERVO_CANOPEN         //CANOPEN型伺服
//#define     SERVOTYPE SERVO_MODBUS          //Modbus型伺服
//#define     SERVOTYPE SERVO_CAN             //CAN型伺服

#define     CURRENT_ADC   0
#define     CURRENT_SDFM  1
//#define     CURRENT_MODE  CURRENT_ADC
#define     CURRENT_MODE  CURRENT_SDFM

/**调试期间可以通过打开该宏启动一下功能：
*1、屏蔽键盘PE组、PF组密码
**/
#define OPENTEST        (0) //测试宏定义（PE/PF组无密码、故障可复位）
#define OPALLFAULTRESET (0) //故障复位清除轻易
#define POSFEEBACKTEST  (0) //pwm和ECAT按照周期接收数据
#define ECATUSEIRQ      (0) //使用外部中断接收数据
#define TIMETEST        (0) //时间测试宏定义（DO无输出，DO1~4用于时间输出，低电平表示程序执行）
#define ECATAD          (0) //0关闭   1打开
#define PROTIMETEST     (0)
#define SCOPEADDRES     (0)

#define DCMAXTIME       (600)
#define DCMTIME1US      (200)  

/* End user code. Do not edit comment generated here */
#endif
