/*
 * SV_Servocode.c
 *
 *  Created on: 2016-2-24//
 *      Author: rd0217
 */

#include "SV_Servocode.h"
#include "SV_FuncVar.h"
#include "SV_FaultProtect.h"
#include "SV_I2c.h"//



//伺服CODE标号宏定义（每增加一个code，下面几处均需更新）
#define SERVOCODE0_DEFINE       (0)
#define SERVOCODE1_1R2_2        (1)
#define SERVOCODE2_1R8_2        (2)
#define SERVOCODE3_2R8_2        (3)
#define SERVOCODE4_5R7_2        (4)
#define SERVOCODE5_7R3_2        (5)
#define SERVOCODE6_8R8_2        (6)
#define SERVOCODE7_12R_2        (7)

#define SERVOCODE_48_0_11     (11)
#define SERVOCODE_48_0_14     (12)
#define SERVOCODE_48_0_17     (13)
#define SERVOCODE_48_0_20     (14)
#define SERVOCODE_48_0_25     (15)
#define SERVOCODE_48_0_32     (16)
#define SERVOCODE_48_0_40     (17)

#define SERVOCODE_48_1_11     (18)
#define SERVOCODE_48_1_14     (19)
#define SERVOCODE_48_1_17     (20)
#define SERVOCODE_48_1_20     (21)
#define SERVOCODE_48_1_25     (22)
#define SERVOCODE_48_1_32     (23)
#define SERVOCODE_48_1_40     (24)


#define SERVOCODE_48_2_11     (25)
#define SERVOCODE_48_2_14     (26)
#define SERVOCODE_48_2_17     (27)
#define SERVOCODE_48_2_20     (28)
#define SERVOCODE_48_2_25     (29)
#define SERVOCODE_48_2_32     (30)
#define SERVOCODE_48_2_40     (21)

//#define SERVOCODE8_018_2        (8)
//#define SERVOCODE9_021_2        (9)
//#define SERVOCODE10_026_2        (10)
//
//#define SERVOCODE11_5R4_3       (11)
//#define SERVOCODE12_8R4_3       (12)
//#define SERVOCODE13_012_3       (13)
//#define SERVOCODE14_018_3       (14)
//#define SERVOCODE15_021_3       (15)
//#define SERVOCODE16_026_3       (16)



//电流放大倍数物理意义：测量范围对应额定值的倍数
//电流放大倍数宏定义   1.5V / 5.5A / 7.5mΩ / 8 / 0.84 / 1.414 = 3.83
//电流放大倍数定义 eg. 20A传感器 ==> 0.3~2.7V ==> 0~3V ==> 25A ==> 25A/5A/1.414 = 3.536
//电流放大倍数宏定义   1.5V / 5.5A / 7.5mΩ / 8 / 0.84 / 1.414 = 3.83
#define     IB_DIV_IR1R2          329       //1.2A放大倍数  40mΩ       100W
//#define   IB_DIV_IR1R8          351       //1.8A放大倍数  25mΩ       200W
#define     IB_DIV_IR1R8          438       //1.8A放大倍数  20mΩ       200W   2019.7.18
#define     IB_DIV_IR2R8          375       //2.8A放大倍数  15mΩ       400W
#define     IB_DIV_IR5R7          369       //5.7A放大倍数  7.5mΩ      1000W
#define     IB_DIV_IR7R3          360       //7.3A放大倍数  6mΩ        1000W
#define     IB_DIV_IR8R8          359       //8.8A放大倍数  5mΩ        1500W
#define     IB_DIV_IR12R          263       //12A放大倍数   5mΩ        2000W

#define     IB_DIV_IR4R2          501       //4.2A放大倍数  7.5mΩ
#define     IB_DIV_IR5R4          380       //5.4A放大倍数  7.5mΩ
#define     IB_DIV_IR5R5          383       //5.5A放大倍数  7.5mΩ      750W
#define     IB_DIV_IR7R6          346       //7.6A放大倍数  6mΩ        1000W
#define     IB_DIV_IR8R4          367       //8.4A放大倍数  5.12mΩ
#define     IB_DIV_IR10R          316       //10A放大倍数       5mΩ       1500W
//#define     IB_DIV_IR12R          385       //12A放大倍数       3.42mΩ
#define     IB_DIV_IR18R          351       //18A放大倍数       2.5mΩ     2000W
#define     IB_DIV_IR21R          450       //21A放大倍数       1.67mΩ    3000W
#define     IB_DIV_IR26R          368       //26A放大倍数       1.65mΩ
#define     IB_DIV_IRERR          385       //错误输入时放大倍数

#define     IB_DIV_TXM11_48       302       //       
#define     IB_DIV_TXM14_48       302       //                
#define     IB_DIV_TXM17_48       181       //      
#define     IB_DIV_TXM20_48       151       //  
#define     IB_DIV_TXM20_48       151       //  
#define     IB_DIV_TXM20_48       151       //  
#define     IB_DIV_TXM20_48       151       //  

//伺服code相关初始化-固定参数部分
void ServoparaInit_Fix(void)
{
    switch(RamServo->ServoCode) //PrF.06驱动器代码
    {
        case SERVOCODE0_DEFINE:     //驱动器代码为0时可以自行设定
        break;

        case SERVOCODE1_1R2_2://1.2A,100w,220v----1
            RamServo->IeSampleGain = IB_DIV_IR1R2;
            RamServo->ServoIe = 120;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;

        case SERVOCODE2_1R8_2://1.8A,200w,220v----2
            RamServo->IeSampleGain = IB_DIV_IR1R8;
            RamServo->ServoIe = 180;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;

        case SERVOCODE3_2R8_2://2.8A,0.4KW,220V-----3
            RamServo->ServoIe = 280;//额定电流
            RamServo->IeSampleGain = IB_DIV_IR2R8;//电路放大倍数
            RamServo->ServoVolLv = 220;//驱动器电压等级
            RamServo->ILPMaskFlag = 1;//缺相保护屏蔽
        break;

        case SERVOCODE4_5R7_2://  5.7A,0.75/1kw,220v---- 4
            RamServo->IeSampleGain = IB_DIV_IR5R7;
            RamServo->ServoIe = 570;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;

        case SERVOCODE5_7R3_2://     7.3A,1.0KW,220V-----5
            RamServo->ServoIe = 730;
            RamServo->IeSampleGain = IB_DIV_IR7R3;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;

        case SERVOCODE6_8R8_2://8.8A,1.5KW,220V-----6
            RamServo->IeSampleGain = IB_DIV_IR8R8;
            RamServo->ServoIe = 880;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;

        case SERVOCODE7_12R_2://12.00A,2.0KW,220V-----7
            RamServo->IeSampleGain = IB_DIV_IR10R;
            RamServo->ServoIe = 1200;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
        break;
        
        case SERVOCODE_48_0_11://4.40A, 4mΩ,100W,48V
            RamServo->IeSampleGain = 257;
            RamServo->ServoIe = 440;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2400;//驱动器2.4倍过载
        break;

        case SERVOCODE_48_0_14://4.40A, 4mΩ,100W,48V
            RamServo->IeSampleGain = 257;
            RamServo->ServoIe = 440;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2400;//驱动器2.4倍过载
        break;

        case SERVOCODE_48_0_17://10.04A, 2mΩ,100W,48V
            RamServo->IeSampleGain = 225;
            RamServo->ServoIe = 1040;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2000;//驱动器2.0倍过载
        break;

        case SERVOCODE_48_0_20://10.04A, 2mΩ,100W,48V
            RamServo->IeSampleGain = 225;
            RamServo->ServoIe = 1040;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2000;//驱动器2.0倍过载
        break;

        case SERVOCODE_48_0_25://24.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 189;
            RamServo->ServoIe = 2400;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 1600;//驱动器1.6倍过载
        break;
        
        case SERVOCODE_48_0_32://30.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 151;
            RamServo->ServoIe = 3000;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 1200;//驱动器1.2倍过载
        break;
        
        case SERVOCODE_48_0_40://30.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 151;
            RamServo->ServoIe = 3000;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 1200;//驱动器1.2倍过载
        break;
        
        case SERVOCODE_48_1_11://2.55A, 6mΩ,100W,48V
            RamServo->IeSampleGain = 295;
            RamServo->ServoIe = 255;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器2.5倍过载
        break;

        case SERVOCODE_48_1_14://4.40A, 4mΩ,100W,48V
            RamServo->IeSampleGain = 295;
            RamServo->ServoIe = 255;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器2.5倍过载
        break;

        case SERVOCODE_48_1_17://4.20A, 4mΩ,100W,48V
            RamServo->IeSampleGain = 269;
            RamServo->ServoIe = 420;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器2.5倍过载
        break;

        case SERVOCODE_48_1_20://10.04A, 2mΩ,100W,48V
            RamServo->IeSampleGain = 296;
            RamServo->ServoIe = 765;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器2.5倍过载
        break;

        case SERVOCODE_48_1_25://24.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 279;
            RamServo->ServoIe = 1620;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器2.5倍过载
        break;
        
        case SERVOCODE_48_1_32://24.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 189;
            RamServo->ServoIe = 2400;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 1600;//驱动器1.6倍过载
        break;
        
        case SERVOCODE_48_1_40://24.00A, 1mΩ,100W,48V
            RamServo->IeSampleGain = 189;
            RamServo->ServoIe = 2400;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 1600;//驱动器1.6倍过载
        break;

        case SERVOCODE_48_2_14://5.00A, 5mΩ,100W,48V
            RamServo->IeSampleGain = 849;
            RamServo->ServoIe = 250;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器1.6倍过载
        case SERVOCODE_48_2_17://5.00A, 5mΩ,100W,48V
            RamServo->IeSampleGain = 386;
            RamServo->ServoIe = 550;
            RamServo->ServoVolLv = 48;
            RamServo->TorqLimt = 2500;//驱动器1.6倍过载
        break;
        
        default:
            RamServo->ServoIe = 550;
            RamServo->IeSampleGain = IB_DIV_IR5R5;
            RamServo->ServoVolLv = 220;
            RamServo->ILPMaskFlag = 1;
            FaultPrtt_FaultInterface(DrvIeeIptErr);
        break;
    }
}

//伺服code相关初始化-后期可设部分
void ServoparaInit_Adjust(void)
{
    switch(RamServo->ServoCode) //PF.06驱动器代码
    {
        case SERVOCODE0_DEFINE:     //驱动器代码为0时可以自行设定
        break;

        case SERVOCODE1_1R2_2://1.2A,100w,220v----1
            RamAid->ResistorBrk = 100;
            RamAid->CapacityResistorBrk = 60;
            RamServo->TorqLimt = 3000;//驱动器3倍过载
        break;

        case SERVOCODE2_1R8_2://1.8A,200w,220v----2
            RamAid->ResistorBrk = 100;
            RamAid->CapacityResistorBrk = 60;
            RamServo->TorqLimt = 3000;//驱动器3倍过载
        break;

        case SERVOCODE3_2R8_2://2.8A,0.4KW,220V-----3
            RamAid->ResistorBrk = 100;//制动电阻值
            RamAid->CapacityResistorBrk = 60;//制动电阻容量
            RamServo->TorqLimt = 3000;//驱动器3倍过载
        break;

        case SERVOCODE4_5R7_2://5.7A,0.75KW/1KW,220V-----4
            RamAid->ResistorBrk = 60;
            RamAid->CapacityResistorBrk = 400;
            RamServo->TorqLimt = 3000;//驱动器3倍过载
        break;

        case SERVOCODE5_7R3_2://7.3A,1KW,220V-----5
            RamAid->ResistorBrk = 60;
            RamAid->CapacityResistorBrk = 400;
            RamServo->TorqLimt = 3000;//驱动器3倍过载
        break;

        case SERVOCODE6_8R8_2://8.8A,1.5KW,220V-----6
            RamAid->ResistorBrk = 60;
            RamAid->CapacityResistorBrk = 400;
            RamServo->TorqLimt = 3000;//驱动器2倍过载
        break;

        case SERVOCODE7_12R_2://12.00A,2.0KW,220V-----7
            RamAid->ResistorBrk = 60;
            RamAid->CapacityResistorBrk = 400;
            RamServo->TorqLimt = 2500;//驱动器2倍过载
        break;
        
        case SERVOCODE_48_0_11://4.40A, 4mΩ,100W,48V

        break;

        case SERVOCODE_48_0_14://4.40A, 4mΩ,100W,48V

        break;

        case SERVOCODE_48_0_17://10.04A, 2mΩ,100W,48V

        break;

        case SERVOCODE_48_0_20://10.04A, 2mΩ,100W,48V

        break;

        case SERVOCODE_48_0_25://24.00A, 1mΩ,100W,48V

        break;
        
        case SERVOCODE_48_0_32://30.00A, 1mΩ,100W,48V

        break;
        
        case SERVOCODE_48_0_40://30.00A, 1mΩ,100W,48V

        break;
        
        case SERVOCODE_48_1_11://2.55A, 6mΩ,100W,48V

        break;

        case SERVOCODE_48_1_14://4.40A, 4mΩ,100W,48V

        break;

        case SERVOCODE_48_1_17://4.20A, 4mΩ,100W,48V

        break;

        case SERVOCODE_48_1_20://10.04A, 2mΩ,100W,48V

        break;

        case SERVOCODE_48_1_25://24.00A, 1mΩ,100W,48V

        break;
        
        case SERVOCODE_48_1_32://30.00A, 1mΩ,100W,48V

        break;
        
        case SERVOCODE_48_1_40://30.00A, 1mΩ,100W,48V

        break;

        case SERVOCODE_48_2_14://5.00A, 5mΩ,100W,48V
        case SERVOCODE_48_2_17://5.00A, 5mΩ,100W,48V
        break;

        default:
            RamAid->ResistorBrk = 50;
            RamAid->CapacityResistorBrk = 100;
            RamServo->TorqLimt = 2000;//驱动器2倍过载
            FaultPrtt_FaultInterface(DrvIeeIptErr);
        break;
    }
    //相关参数存储
    IIcInterfaceB(&RamAid->ResistorBrk,2);
    IIcInterfaceB(&RamServo->TorqLimt ,1);
}


















////伺服CODE标号宏定义（每增加一个code，下面几处均需更新）
//#define SERVOCODE0_DEFINE     (0)
//#define SERVOCODE1_1R6_2      (1)
//#define SERVOCODE2_2R8_2      (2)
//#define SERVOCODE3_4R2_2      (3)
//#define SERVOCODE4_5R5_2      (4)
//#define SERVOCODE5_7R6_2      (5)
//#define SERVOCODE6_010_2      (6)
//#define SERVOCODE7_018_2      (7)
//#define SERVOCODE8_021_2      (8)
//#define SERVOCODE9_026_2      (9)
//
//#define SERVOCODE10_5R4_3     (10)
//#define SERVOCODE11_8R4_3     (11)
//#define SERVOCODE12_012_3     (12)
//#define SERVOCODE13_018_3     (13)
//#define SERVOCODE14_021_3     (14)
//#define SERVOCODE15_026_3     (15)
//
//#define SERVOCODE16_2R0_48        (16)
//#define SERVOCODE17_4R0_48        (17)
//#define SERVOCODE18_6R0_48        (18)
//#define SERVOCODE19_010_48        (19)
////电流放大倍数宏定义   1.5V / 5.5A / 7.5mΩ / 8 / 0.84 / 1.414 = 3.83
//#define     IB_DIV_IR1R6        395       //1.6A放大倍数  25mΩ
//#define     IB_DIV_IR2R8        375       //2.8A放大倍数  15mΩ
//#define     IB_DIV_IR4R2        501       //4.2A放大倍数  7.5mΩ
//#define     IB_DIV_IR5R4        380       //5.4A放大倍数  7.5mΩ
//#define     IB_DIV_IR5R5        383       //5.5A放大倍数  7.5mΩ
//#define     IB_DIV_IR7R6        346       //7.6A放大倍数  6mΩ
//#define     IB_DIV_IR8R4          367         //8.4A放大倍数  5.12mΩ
//#define     IB_DIV_IR10R          421         //10A放大倍数       3.75mΩ
//#define     IB_DIV_IR12R          385         //12A放大倍数       3.42mΩ
//#define     IB_DIV_IR18R          372         //18A放大倍数       2.357mΩ
//#define     IB_DIV_IR21R          358         //21A放大倍数       2.1mΩ
//#define     IB_DIV_IR26R          368         //26A放大倍数       1.65mΩ
//#define     IB_DIV_IRERR          385         //错误输入时放大倍数
//
//#define     IB_DIV_IR2R0_48         354       //2.5A 放大倍数     10A
//#define     IB_DIV_IR4R0_48         383       //5.0A 放大倍数     20A
//#define     IB_DIV_IR6R0_48         354       //7.5A 放大倍数     30A
//#define     IB_DIV_IR010_48         295       //15.0A放大倍数     50A





////伺服code相关初始化-固定参数部分
//void ServoparaInit_Fix(void)
//{
//	switch(RamServo->ServoCode) //PrF.06驱动器代码
//	{
//		case SERVOCODE0_DEFINE:		//驱动器代码为0时可以自行设定
//		break;
//
//	    case SERVOCODE1_1R6_2://1.6A,100w,220v----1
//		    RamServo->IeSampleGain = IB_DIV_IR1R6;
//			RamServo->ServoIe = 160;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 1;
//		break;
//
//		case SERVOCODE2_2R8_2://2.8A,0.4KW,220V-----2
//			RamServo->ServoIe = 280;//额定电流
//			RamServo->IeSampleGain = IB_DIV_IR2R8;//电路放大倍数
//			RamServo->ServoVolLv = 220;//驱动器电压等级
//			RamServo->ILPMaskFlag = 1;//缺相保护屏蔽
//		break;
//
//	    case SERVOCODE3_4R2_2://4.2A,0.75kw,220v----3
//		    RamServo->IeSampleGain = IB_DIV_IR4R2;
//			RamServo->ServoIe = 420;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 1;
//		break;
//
//	    case SERVOCODE4_5R5_2://5.5A,0.75KW,220V-----4
//		    RamServo->ServoIe = 550;
//		    RamServo->IeSampleGain = IB_DIV_IR5R5;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE5_7R6_2://7.6A,1.0KW,220V-----5
//		    RamServo->IeSampleGain = IB_DIV_IR7R6;
//			RamServo->ServoIe = 760;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE6_010_2://10.00A,1.5KW,220V-----6
//		    RamServo->IeSampleGain = IB_DIV_IR10R;
//			RamServo->ServoIe = 1000;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE7_018_2://18.00A,2.0KW,220V-----7
//		    RamServo->IeSampleGain = IB_DIV_IR18R;
//			RamServo->ServoIe = 1800;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 0;
//	   	break;
//
//	    case SERVOCODE8_021_2://21.00A,3.2KW,220V-----8
//		    RamServo->IeSampleGain = IB_DIV_IR21R;
//			RamServo->ServoIe = 2100;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 0;
//	   	break;
//
//	    case SERVOCODE9_026_2://26.00A,4.5KW,220V-----9
//		    RamServo->IeSampleGain = IB_DIV_IR26R;
//			RamServo->ServoIe = 3000;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE10_5R4_3://5.4A,1.5KW,380V-----10
//		    RamServo->IeSampleGain = IB_DIV_IR5R4;
//			RamServo->ServoIe = 540;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE11_8R4_3://8.4A,2.0KW,380V-----11
//		    RamServo->IeSampleGain = IB_DIV_IR8R4;
//			RamServo->ServoIe = 840;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE12_012_3://12.00A,3.0KW,380V-----12
//		    RamServo->IeSampleGain = IB_DIV_IR12R;
//			RamServo->ServoIe = 1200;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE13_018_3://18.00A,4.0KW,380V-----13
//		    RamServo->IeSampleGain = IB_DIV_IR18R;
//			RamServo->ServoIe = 1800;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE14_021_3://21.00A,5.6KW,380V-----14
//		    RamServo->IeSampleGain = IB_DIV_IR21R;
//			RamServo->ServoIe = 2100;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE15_026_3://26.00A,7.5KW,380V-----15
//		    RamServo->IeSampleGain = IB_DIV_IR26R;
//			RamServo->ServoIe = 2600;
//			RamServo->ServoVolLv = 380;
//			RamServo->ILPMaskFlag = 0;
//	    break;
//
//	    case SERVOCODE16_2R0_48://2.00A,100W,48V-----16
//		    RamServo->IeSampleGain = IB_DIV_IR2R0_48;
//			RamServo->ServoIe = 200;
//			RamServo->ServoVolLv = 48;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE17_4R0_48://4.00A,200W,48V-----17
//		    RamServo->IeSampleGain = IB_DIV_IR4R0_48;
//			RamServo->ServoIe = 500;
//			RamServo->ServoVolLv = 48;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE18_6R0_48://6.00A,300W,48V-----18
//		    RamServo->IeSampleGain = IB_DIV_IR6R0_48;
//			RamServo->ServoIe = 600;
//			RamServo->ServoVolLv = 48;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	    case SERVOCODE19_010_48://10.00A,500W,48V-----19
//		    RamServo->IeSampleGain = IB_DIV_IR010_48;
//			RamServo->ServoIe = 1000;
//			RamServo->ServoVolLv = 48;
//			RamServo->ILPMaskFlag = 1;
//	    break;
//
//	  	default:
//		    RamServo->ServoIe = 550;
//		    RamServo->IeSampleGain = IB_DIV_IR5R5;
//			RamServo->ServoVolLv = 220;
//			RamServo->ILPMaskFlag = 1;
//			FaultPrtt_FaultInterface(DrvIeeIptErr);
//		break;
//	}
//}
//
////伺服code相关初始化-后期可设部分
//void ServoparaInit_Adjust(void)
//{
//	switch(RamServo->ServoCode) //PrF.06驱动器代码
//	{
//		case SERVOCODE0_DEFINE:		//驱动器代码为0时可以自行设定
//		break;
//
//	    case SERVOCODE1_1R6_2://1.6A,100w,220v----1
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//		break;
//
//		case SERVOCODE2_2R8_2://2.8A,0.4KW,220V-----2
//			RamAid->ResistorBrk = 50;//制动电阻值
//			RamAid->CapacityResistorBrk = 100;//制动电阻容量
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//		break;
//
//	    case SERVOCODE4_5R5_2://5.5A,0.75KW,220V-----4
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	    case SERVOCODE5_7R6_2://7.6A,1.0KW,220V-----5
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE6_010_2://10.00A,1.5KW,220V-----6
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE7_018_2://18.00A,2.0KW,220V-----7
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	   	break;
//
//	    case SERVOCODE8_021_2://21.00A,3.2KW,220V-----8
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2500;//驱动器2倍过载
//	   	break;
//
//	    case SERVOCODE9_026_2://26.00A,4.5KW,220V-----9
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2000;//驱动器1.5倍过载
//	    break;
//
//	    case SERVOCODE10_5R4_3://5.4A,1.5KW,380V-----10
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE11_8R4_3://8.4A,2.0KW,380V-----11
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	    case SERVOCODE12_012_3://12.00A,3.0KW,380V-----12
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE13_018_3://18.00A,4.0KW,380V-----13
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE14_021_3://21.00A,5.6KW,380V-----14
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2500;//驱动器2倍过载
//	    break;
//
//	    case SERVOCODE15_026_3://26.00A,7.5KW,380V-----15
//			RamAid->ResistorBrk = 40;
//			RamAid->CapacityResistorBrk = 150;
//			RamServo->TorqLimt = 2000;//驱动器1.5倍过载
//	    break;
//
//	    case SERVOCODE16_2R0_48://2.00A,100W,48V-----16
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	    case SERVOCODE17_4R0_48://4.00A,200W,48V-----17
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	    case SERVOCODE18_6R0_48://6.00A,300W,48V-----18
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	    case SERVOCODE19_010_48://10.00A,500W,48V-----19
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 3000;//驱动器3倍过载
//	    break;
//
//	  	default:
//			RamAid->ResistorBrk = 50;
//			RamAid->CapacityResistorBrk = 100;
//			RamServo->TorqLimt = 2000;//驱动器2倍过载
//			FaultPrtt_FaultInterface(DrvIeeIptErr);
//		break;
//	}
//	//相关参数存储
//	IIcInterfaceB(&RamAid->ResistorBrk,2);
//	IIcInterfaceB(&RamServo->TorqLimt ,1);
//}

//       /*********************以上为使用的代码**********************************/
//
//        case SERVOCODE8_018_2://18.00A,2.0KW,220V-----8
//            RamServo->IeSampleGain = IB_DIV_IR18R;
//            RamServo->ServoIe = 1800;
//            RamServo->ServoVolLv = 220;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE9_021_2://21.00A,3KW,220V-----9
//            RamServo->IeSampleGain = IB_DIV_IR21R;
//            RamServo->ServoIe = 2100;
//            RamServo->ServoVolLv = 220;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE10_026_2://26.00A,4.5KW,220V-----10
//            RamServo->IeSampleGain = IB_DIV_IR26R;
//            RamServo->ServoIe = 3000;
//            RamServo->ServoVolLv = 220;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE11_5R4_3://5.4A,1.5KW,380V-----11
//            RamServo->IeSampleGain = IB_DIV_IR5R4;
//            RamServo->ServoIe = 540;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 1;
//        break;
//
//        case SERVOCODE12_8R4_3://8.4A,2.0KW,380V-----12
//            RamServo->IeSampleGain = IB_DIV_IR8R4;
//            RamServo->ServoIe = 840;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE13_012_3://12.00A,3.0KW,380V-----13
//            RamServo->IeSampleGain = IB_DIV_IR12R;
//            RamServo->ServoIe = 1200;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE14_018_3://18.00A,4.0KW,380V-----14
//            RamServo->IeSampleGain = IB_DIV_IR18R;
//            RamServo->ServoIe = 1800;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE15_021_3://21.00A,5.6KW,380V-----15
//            RamServo->IeSampleGain = IB_DIV_IR21R;
//            RamServo->ServoIe = 2100;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 0;
//        break;
//
//        case SERVOCODE16_026_3://26.00A,7.5KW,380V-----16
//            RamServo->IeSampleGain = IB_DIV_IR26R;
//            RamServo->ServoIe = 2600;
//            RamServo->ServoVolLv = 380;
//            RamServo->ILPMaskFlag = 0;
//        break;

