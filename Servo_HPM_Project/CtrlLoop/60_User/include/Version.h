#include "r_cg_Project.h"


#if SERVOPOWER == SERVO_DC
#if SERVOTYPE == SERVO_ETHERCAT
//10000
//10001
//      10002 可以串口升级
//24081 10002 方向可以调整
//24082 10002 PE ,PF可以保存
//24083 10002 加入示波器功能,默认打开EC-1A报警，修补位置插补问题   2024.11.07
//24110 10002 485通信间隔在理论基础增加2ms
//24111 10002 使用pr1.04可以取反输出端编码器
//24112 10002 调整过载曲线，加入堵转保护
//24113 10002 调整ECAT升级，检验出错，死机问题
//24114 10002 增加导入参数后发送保存指令，复位参数指令用77，修复保存时P0=65535把参数初始化
//24115 10002 负方向加入堵转保护
//24116 10002 多摩川编码器加入回零
//24117 10002 25,32模块加入新过载曲线
//24118 10002 Scope可以连续4ms，捕捉
//24119 10002 加入硬件版本号
//25000 10002 无编码器类型，学习电角度强制为30°,修改pr0.28计算方法
//25001 10002 程序为测量两路温度传感器
//25002 10002 添加0x3e0C
//25003 10002 闭环参数调整
//25004 10002 上位机位置运动，闭环点动加余数补偿
//25005 10002  默认驱动器代码
//25006 10002  修复PrD.02 = 1回零
//25007 10002  修复PrD.02 = 2回零 使能可切换ECAT控制模式
//25008 10002  P1.65和p1.66的对象字典
//25009 10002  修复TCM17的F.06参数
//25010 10002  修改pr8.34使用方法,SYNC容错，DC参考时间,屏蔽ECT看门狗  
//             EEPROM闭环保存，闭环加减速系数，0x607F初值，全闭环的位置反馈 
//25011 10002  综合版本，保留版本未使用
//25012 10002  综合版本
//25013 10002  过渡版本，参数参在错误
//25014 10002  修改DC同步计算，修复大通信周期电机抖动问题
//25015 10002  MODBUS更换IO口，升级程序关PWM输出，示波器串口加入错误清除 MODBUS加入极限状态PosGenerator清除
//25016 10002  增加0x3032,0x3034SDO,修改0x3010,0x301C映射地址，增加控制源显示，
//             增加自举开关，u16PendingOptionCode关使能置零  （PosNew - PosOld）加入赋值限定
//25017 10002  修改非全闭环位置反馈初始值计算方式，以及输出端和电机端编码器对齐 ,
//             增加0x3104 SDO  修改TCM20默认减速比
//25000 10     HPM初版
//25001 10     SDO写EEPROM相同值不保存，0x3E10修改为32bit，修改版本号显示，DC 可以125us
//25002 10     加入金刚编码器程序
//25003 10     新款上位机
//25004 10     SDFM电流采样，对上位机通信加入地址限定
//25005 10     prE.22调整电机方向，以编码器方向为正方向，传感器自动上传，可以置零
//25006 10     PDO与XML1.1对齐，可以支持125us,不报错，升级正常，PDO读取正常 更新XML数组
//25007 10     增加保存所有参数,修改抱闸生效延时，增加力矩传感器选择,增加电角度补偿，完善MIT计算,力矩模式不计算加速度反馈，重力补偿
//2500100      增加弓望编码器，从站别名默认0
//2500200      17和20是SDFM,带过流保护,上位机示波器可以显示力矩传感器数据，硬件保护带滤波
//2500201      硬件保护带滤波
//2500300      用于测试
//2500400      力矩取反，关闭无用时钟  401 14模组屏蔽硬件过流保护
//2500500      力矩传感器使用912600波特率，力矩模式时，伪速度模式与力矩切换判断条件更改
//2500600      修复内部力矩模式转速限制失败，增加非断线检测，ECAT的1ms与检测DC周期定时器分开，SCI文件不优化
//2500700      修复上位机卡死，修订报警码复位属性  701增加编码器校准  702编码器校准显示  703编码器校准显示OK  704解决Pos_Fir数组越界
//2500800      看门狗失效问题，Er0.35被意外屏蔽问题,使能才能超速报警，飞车报警，添加0x3C00的PDO

//100.2        初版
//100.3        修改编码器校准后不能置零   上位机控制加减速赋初值  
//100.4        传感器上电不在配置采样率 
//100.5        解决BANK1不能升级问题，解决复位驱动器485连接断线问题
//100.6        SN码补齐,发指令读取编码器校准结果
//100.7        0x1000保存地址修改 但是引入11电机PE参数初始化错误
//100.8        修复11电机PE组初始化数据（PE8~PE15重复初始化）
//100.9        修复TwinCat连续两次升级导致死机，增加PE组可以取反输出端编码器
//101.0        修复全闭环编码器上电就是0
//101.1        修复0x6000保存bug，添加PDO填充，丢帧检测清零，0x302A数据映射,
//101.2        Er.022阈值可调，编码器校准赋值可查,0x3020映射类型
//101.3        Ecat增加位置偏移
//101.4        增加编码器角度偏差实时值
//101.5        0x1601索引赋值 0x6065全闭环赋值  取消DC抖动差补  非DC模式默认通信２ｍｓ
//101.6        增加传感器型号，量程，版本查看  0x4000组能SDO写、能在线加载，
//             使能修改位置环增益 速度环增益，速度环积分时间   优化从站ID写入
//101.7        增加力矩传感器放大增益
//101.8        力矩传感器型号、量程、版本可以SDO读取
#define SERIESCODE      10
#define HARDCODE        10
#define MCUCODE         0
#define SOFTCODE        2
#define VERSIONCODE     101
#define SUBVERSIONCODE  8
#define PRODUCTCODE     ((SERIESCODE * 100000000L) + (HARDCODE * 1000000L)  + (MCUCODE * 100000L) + (SOFTCODE * 10000L) + (VERSIONCODE * 10L) + SUBVERSIONCODE) 

#define PRDCTSer1       (PRODUCTCODE%65536)   
#define CUSTOMER        (PRODUCTCODE/65536)   
#define EEPROMSer1      (11)    //EEPROM版本  
#define HARDWARENUM     (15)    //硬件版本       
#elif SERVOTYPE == SERVO_PULSE
#define SERIESCODE      10
#define HARDCODE        20
#define MCUCODE         0
#define SOFTCODE        0
#define VERSIONCODE     100
#define SUBVERSIONCODE  1
#define PRODUCTCODE     ((SERIESCODE * 100000000L) + (HARDCODE * 1000000L)  + (MCUCODE * 100000L) + (SOFTCODE * 10000L) + (VERSIONCODE * 10L) + SUBVERSIONCODE) 

#define PRDCTSer1       (PRODUCTCODE%65536)   
#define CUSTOMER        (PRODUCTCODE/65536)   
#define EEPROMSer1      (21)    //EEPROM版本  
#define HARDWARENUM     (15)    //硬件版本 
#elif SERVOTYPE == SERVO_CANOPEN
#define SERIESCODE      10
#define HARDCODE        30
#define MCUCODE         0
#define SOFTCODE        2
#define VERSIONCODE     100
#define SUBVERSIONCODE  5
#define PRODUCTCODE     ((SERIESCODE * 100000000L) + (HARDCODE * 1000000L)  + (MCUCODE * 100000L) + (SOFTCODE * 10000L) + (VERSIONCODE * 10L) + SUBVERSIONCODE) 

#define PRDCTSer1       (PRODUCTCODE%65536)   
#define CUSTOMER        (PRODUCTCODE/65536)   
#define EEPROMSer1      (32)    //EEPROM版本  
#define HARDWARENUM     (15)    //硬件版本 
#elif SERVOTYPE == SERVO_MODBUS
//100   初版   101  添加编码器校准
#define SERIESCODE      10
#define HARDCODE        40
#define MCUCODE         0
#define SOFTCODE        0
#define VERSIONCODE     100
#define SUBVERSIONCODE  2
#define PRODUCTCODE     ((SERIESCODE * 100000000L) + (HARDCODE * 1000000L)  + (MCUCODE * 100000L) + (SOFTCODE * 10000L) + (VERSIONCODE * 10L) + SUBVERSIONCODE) 

#define PRDCTSer1       (PRODUCTCODE%65536)   
#define CUSTOMER        (PRODUCTCODE/65536)   
#define EEPROMSer1      (41)    //EEPROM版本  
#define HARDWARENUM     (15)    //硬件版本 
#elif SERVOTYPE == SERVO_CAN
//100.2        初版
//100.3        修改编码器校准后不能置零   上位机控制加减速赋初值  
//100.4        传感器上电不在配置采样率
//100.5        解决BANK1不能升级问题，解决复位驱动器485连接断线问题
//100.6        SN码补齐,发指令读取编码器校准结果，添加强制抱闸打开,连接P1~PF
//100.7        OTA功能  通信自恢复 上位机示波器显示位置，速度，力矩置零 修正上位机显示的控制字和状态字   反馈帧1添加力矩传感器
//100.8        修改CANFD的采样点
//100.9        CAN通信OTA在使能时禁止
//101.0        修复全闭环位置上电为0
//101.1        带故障存储功能
#define SERIESCODE      10
#define HARDCODE        50
#define MCUCODE         0
#define SOFTCODE        2
#define VERSIONCODE     101
#define SUBVERSIONCODE  1
#define PRODUCTCODE     ((SERIESCODE * 100000000L) + (HARDCODE * 1000000L)  + (MCUCODE * 100000L) + (SOFTCODE * 10000L) + (VERSIONCODE * 10L) + SUBVERSIONCODE) 

#define PRDCTSer1       (PRODUCTCODE%65536)   
#define CUSTOMER        (PRODUCTCODE/65536)   
#define EEPROMSer1      (51)    //EEPROM版本  
#define HARDWARENUM     (15)    //硬件版本 
#endif


extern char ProSnCode[32];
#endif