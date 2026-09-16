#include "string.h"
#include "SV_PanelCtl.h"
#include "SV_FuncCode.h"
#include "SV_FuncVar.h"
#include "SV_Sci.h"
#include "SV_I2c.h"
#include "SV_FaultProtect.h"
#include "SV_StateMachine.h"
#include "SV_CtrlMode.h"
#include "SV_UdcCtrl.h"
#include "SV_DataType.h"
#include "userdefine.h"
#include "r_cg_Project.h"
#include "cia402ControlMode.h"
#include "SV_FuncCodeObj.h"
#include "Drive.h"
#include "DrvCoeff.h"//

#define TIME_DISLEVEL4		(1000)		//LEVEL4,end???????
#define TIME_POWONDISP		(10000)		//?????,????????power-ON
#define TIME_FAULTDISP		(10000)		//?????,????????FAULT
#define TIME_TC200DISP		(1000)		//TC200??1s

/*******************?????*******************/
/*********** ?????***********/
Servo_Panel Servo2Panel = SERVO2PANELDEFAULT;
/************LED?????********/
LedControl LedDisplay = LEDDISPLAYDEFAULT;
/*******??????????*********/
FuncCodeOPeration Func = {FUNCODEDEF};
NoFacCodeOperation NoFacFunc= NONEFUNCODEDEF;
/*******???????*************/
KeyInfo_Type KeyInfo = KEYINFODEF;
extern uint16_t EcatErrCode; 
const uint16_t NUMDIS_DEC2HEX[16]={'0','1','2','3','4','5','6','7','8','9','A','b','C','d','E','F'};

/**
 * 工厂自检决定显示哪一个数码管,因这些笔画在ASCII规范中没有，在此自定义从0x80开始处理。
 */
typedef enum{
	LED_UPV = 0x80,
	LED_RIGHTUPH = 0x81,
	LED_RIGHTDOWNH = 0x82,
	LED_DOWNV = 0x83,
	LED_LEFTDOWNH = 0x84,
	LED_LEFTUPH = 0x85,
	LED_MIDV = 0x86,
	LED_POINT =0x87,
	LED_ALL = 0x88
}FACTESTLED;
const uint16_t FacTestLed[] = {LED_UPV,LED_RIGHTUPH,LED_RIGHTDOWNH,LED_DOWNV,LED_LEFTDOWNH,\
							 LED_LEFTUPH,LED_MIDV,LED_POINT,LED_ALL};
/********************************************数据处理模块*************************************/
//大模块：显示按照ASCII码形式显示，因此将各种格式转变成ASCII码
/**
 * 将ASCII码转换成数码管可以显示的电平
 */
uint16_t AsciiToSeg(uint8_t Character)
{
	uint16_t SegValue ;
    switch(Character)
    {
        case '0':
        	SegValue = Seg7Disp_0;
        	break;
        case '1':
        	SegValue = Seg7Disp_1;
        break;
        case '2':
        	SegValue = Seg7Disp_2;
        break;
        case '3':
        	SegValue = Seg7Disp_3;
        break;
        case '4' :
        	SegValue = Seg7Disp_4;
        break;
        case '5' :
        	SegValue = Seg7Disp_5;
        break;
        case '6' :
        	SegValue = Seg7Disp_6;
        break;
        case '7' :
        	SegValue = Seg7Disp_7;
        break;
        case '8' :
        	SegValue = Seg7Disp_8;
        break;
        case '9' :
        	SegValue = Seg7Disp_9;
        break;
        case 'A':
        case 'a':
        	SegValue = Seg7Disp_A;
        break;
        case 'B':
        case 'b':
        	SegValue = Seg7Disp_b;
        break;
        case 'C':
        case 'c':
        	SegValue = Seg7Disp_C;
        break;
        case 'D':
        case 'd':
        	SegValue = Seg7Disp_d;
        break;
        case 'E':
        case 'e':
        	SegValue = Seg7Disp_E;
        break;
        case 'F':
        case 'f':
        	SegValue = Seg7Disp_F;
        break;
        case 'G':
        case 'g':
        	SegValue = Seg7Disp_G;
        break;
        case 'H':
        case 'h':
        	SegValue = Seg7Disp_H;
        break;
        case 'I':
        case 'i':
        	SegValue = Seg7Disp_I;
        break;
        case 'j':
        case 'J':
        	SegValue = Seg7Disp_J;
        break;
        case 'K':
        case 'k':
        	SegValue = Seg7Disp_K;
        break;
        case 'L':
        case 'l':
        	SegValue = Seg7Disp_L;
        break;
        case 'M':
        case 'm':
        	SegValue = Seg7Disp_M;
        break;
        case 'N':
        case 'n':
        	SegValue = Seg7Disp_n;
        break;
        case 'O':
        case 'o':
        	SegValue = Seg7Disp_o;
        break;
        case 'P':
        case 'p':
        	SegValue = Seg7Disp_P;
        break;
        case 'Q':
        case 'q':
        	SegValue = Seg7Disp_q;
        break;
        case 'R':
        case 'r':
        	SegValue = Seg7Disp_r;
        break;
        case 'S':
        case 's':
        	SegValue = Seg7Disp_s;
        break;
        case 'T':
        case 't':
        	SegValue = Seg7Disp_t;
        break;
        case 'u':
        case 'U':
        	SegValue = Seg7Disp_u;
        break;
        case 'v':
        case 'V':
        	SegValue = Seg7Disp_V;
        break;
        case 'W':
        case 'w':
        	SegValue = Seg7Disp_W;
        break;
        case 'X':
        case 'x':
        	SegValue = Seg7Disp_X;
        break;
        case 'Y':
        case 'y':
        	SegValue = Seg7Disp_y;
        break;
        case 'Z':
        case 'z':
        	SegValue = Seg7Disp_Z;
        break;
        case ' ':
        	SegValue = Seg7Disp_NULL;
        break;
        case '-':
        case LED_MIDV:
        	SegValue = Seg7Disp_mid;
        break;
        case '~'://代表的是上横线
        case LED_UPV:
        	SegValue = Seg7Disp_up;
        break;
        case '_':
        case LED_DOWNV:
        	SegValue = Seg7Disp_dn;
        break;
        case '.'://小数点
        case LED_POINT:
        	SegValue = Seg7LED_7;
        	break;
        case LED_LEFTUPH://左上竖
        	SegValue = Seg7LED_5;
        break;
        case LED_LEFTDOWNH://左下竖
            SegValue = Seg7LED_4;
        break;
        case LED_RIGHTUPH://右上竖
            SegValue = Seg7LED_1;
        break;
        case LED_RIGHTDOWNH://右下竖
            SegValue = Seg7LED_2;
        break;
        case LED_ALL://全显示
            SegValue = Seg7LED_ALL;
        break;
        default :
        	SegValue = Seg7Disp_NULL;
        break;

    }
    return  SegValue ;
}
//大模块:整数拆分成十进制单个数字、二进制单个数字、十六进制单个数字然后放入数组中供数码管显示和他们的反向操作
/**
 * 将十进制无符号数拆分然后放入数组中
 */
void DataToDecimalArray(uint8_t *Dec,uint32_t Src,uint8_t size)
{
	uint8_t i;
	for(i=0;i<=size;i++)
	{
		*(Dec+i) = Src %10 + '0';
		Src = Src / 10;
	}
}
/**
 * 将十进制有符号数拆分然后放入数组中
 */
void SDataToDecimalArray(uint8_t *Dec,int32_t Src,uint8_t size)
{
	uint8_t i;
	for(i=0;i<=size;i++)
	{
		*(Dec+i) = Src % 10 + '0';
		Src = Src / 10 ;
	}
}
/**
 * 将十进制无符号数拆分成二进制数放入数组中
 */
void DataToBinaryArray(uint8_t * Dec, uint16_t Src, uint8_t size)
{
	uint8_t i;
	for(i=0;i<=size;i++)
	{
		*(Dec+i) = (Src & 0x1) +'0';
		Src = Src >> 1;
	}
}
/**
 * 将十进制无符号数拆分成十六进制数拆分放入数组中
 */
void DataToHexArray(uint8_t * Dec, uint16_t Src, uint8_t size)
{
	uint8_t i;
	for(i=0;i<=size;i++)
	{
		*(Dec +i) = NUMDIS_DEC2HEX[Src&0xf];
		//*(Dec+i) =  Src&0xf;
		Src = Src>>4;
	}
}
//大模块：数字加减处理，用于功能码的值得加减
/**
 * 无符号数加
 */
void SigleDataUnSignAdd(void)
{
	uint64_t ret = 0,Wight=0;
	switch(Servo2Panel.Cursor)
	{
		case 0:
			Wight = 1;
		break;
		case 1:
			Wight = 10;
		break;
		case 2:
			Wight = 100;
		break;
		case 3:
			Wight = 1000;
		break;
		case 4:
			Wight = 10000;
		break;
		case 5:
			Wight = 100000;
		break;
		case 6:
			Wight = 1000000;
		break;
		case 7:
			Wight = 10000000;
		break;
		case 8:
			Wight = 100000000;
		break;
		case 9:
			Wight = 1000000000;
		break;
		default :
			Wight = 0;
		break;
	}
	ret = Func.Key.AddDecValue.UnSign32 + Wight;
	if(ret > Func.Key.Attr.Max.UnSign32)         //范围限制
	{
		ret = Func.Key.Attr.Max.UnSign32;
	}
	Func.Key.AddDecValue.UnSign32 = ret;
}
/**
 * 无符号数减
 */
void SigleDataUnSignDec(void)
{
	uint32_t ret = 0,Wight = 0;
	switch(Servo2Panel.Cursor)
	{
		case 0:
			Wight = 1;
		break;
		case 1:
			Wight = 10;
		break;
		case 2:
			Wight = 100;
		break;
		case 3:
			Wight = 1000;
		break;
		case 4:
			Wight = 10000;
		break;
		case 5:
			Wight = 100000;
		break;
		case 6:
			Wight = 1000000;
		break;
		case 7:
			Wight = 10000000;
		break;
		case 8:
			Wight = 100000000;
		break;
		case 9:
			Wight = 1000000000;
		break;
		default :break;
	}
	if(Func.Key.AddDecValue.UnSign32 < Wight)//
	{
		ret =0;
	}
	else
	{
		ret = Func.Key.AddDecValue.UnSign32 - Wight;
	}
	if(ret < Func.Key.Attr.Min.UnSign32)
	{
		ret =Func.Key.Attr.Min.UnSign32;
	}
	Func.Key.AddDecValue.UnSign32= ret;
}
/**
 * 有符号数加
 */
void SigleDataSignAdd(void)
{
	int64_t ret = 0;
	uint64_t Wight = 0;
	switch(Servo2Panel.Cursor)
	{
		case 0:
			Wight = 1;
			break;
		case 1:
			Wight = 10;
			break;
		case 2:
			Wight = 100;
			break;
		case 3:
			Wight = 1000;
			break;
		case 4:
			Wight = 10000;
			break;
		case 5:
			Wight = 100000;
			break;
		case 6:
			Wight = 1000000;
			break;
		case 7:
			Wight = 10000000;
			break;
		case 8:
			Wight = 100000000;
			break;
		case 9:
			Wight = 1000000000;
			break;
		default :break;
	}
	ret = Func.Key.AddDecValue.Sign32 + Wight;
	if(ret > Func.Key.Attr.Max.Sign32)
	{
		ret = Func.Key.Attr.Max.Sign32;
	}
	Func.Key.AddDecValue.Sign32 = ret;
}
/**
 * 有符号数减
 */
void SigleDataSignDec(void)
{
	int64_t ret = 0;
	int64_t Wight = 0;
	switch(Servo2Panel.Cursor)
	{
		case 0:
			Wight = 1;
		break;
		case 1:
			Wight = 10;
		break;
		case 2:
			Wight = 100;
		break;
		case 3:
			Wight = 1000;
		break;
		case 4:
			Wight = 10000;
		break;
		case 5:
			Wight = 100000;
			break;
		case 6:
			Wight = 1000000;
			break;
		case 7:
			Wight = 10000000;
			break;
		case 8:
			Wight = 100000000;
			break;
		case 9:
			Wight = 1000000000;
			break;
		default :break;
	}
	ret = Func.Key.AddDecValue.Sign32 - Wight;
	if(ret < Func.Key.Attr.Min.Sign32)
	{
		ret = Func.Key.Attr.Min.Sign32;
	}
	Func.Key.AddDecValue.Sign32 = ret;
}
/**
 * 二进制加
 */
void DecimalAdd(uint32_t *Data,uint8_t Cursor)
{
	if(!(*Data&(1<<Cursor)))
	{
		*Data |= (1<<Cursor);
		if(*Data > Func.Key.Attr.Max.UnSign32)
		{
			*Data &= (~(1<<Cursor));
		}
	}
}
/**
 * 二进制减
 */
void DecimalDec(uint32_t *Data,uint8_t Cursor)
{
	if(*Data&(1<<Cursor))
	{
		*Data &= (~(1<<Cursor));
	}
}
/**
 * 十六进制加
 */
void HexAdd(uint32_t *Data,uint8_t Cursor)
{
	if(((*Data&(0xf<<((Cursor<<2))))<(0xf<<((Cursor<<2))))
		&&(*Data&(0xf<<((Cursor<<2)))) < (Func.Key.Attr.Max.UnSign32 & (0xf<<((Cursor<<2))))	)
		*Data += (1<<(Cursor<<2));
}
/**
 * 十六进制减
 */
void HexDec(uint32_t *Data,uint8_t Cursor)
{
	if(*Data&(0xf<<(Cursor<<2)))
	{
		*Data -= (0x1<<(Cursor<<2));
	}
}
/**
 * 功能码参数值增加入口函数
 */
void FunCodeValueAdd(void)
{
	switch(Func.Key.Attr.Property.Bit.ParaType)
	{
	case ATTR_TYPE_Single://整体十进制加
		if(Func.Key.Attr.Property.Bit.Sign == 0)
		{
			Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
			SigleDataUnSignAdd();
		}
		else
		{
			Func.Key.AddDecValue.Sign32 = Func.Key.Value.Sign32;
			SigleDataSignAdd();
		}
		break;
	case ATTR_TYPE_Bit://二进制加
		Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
		DecimalAdd(&Func.Key.AddDecValue.UnSign32,Servo2Panel.Cursor);
		break;
	case ATTR_TYPE_Hex://十六进制加
		Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
		HexAdd(&Func.Key.AddDecValue.UnSign32,Servo2Panel.Cursor);
		break;
	default:break;
	}
}
/**
 * 功能码参数值减
 */
void FunCodeValueDec(void)
{
	switch(Func.Key.Attr.Property.Bit.ParaType)
	{
		case ATTR_TYPE_Single:
			Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
			if(Func.Key.Attr.Property.Bit.Sign == 0)
			{
				SigleDataUnSignDec();
			}
			else
			{
				SigleDataSignDec();
			}
			break;
		case ATTR_TYPE_Bit:
			Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
			DecimalDec(&Func.Key.AddDecValue.UnSign32,Servo2Panel.Cursor);
			break;
		case ATTR_TYPE_Hex:
			Func.Key.AddDecValue.UnSign32 = Func.Key.Value.UnSign32;
			HexDec(&Func.Key.AddDecValue.UnSign32,Servo2Panel.Cursor);
			break;
		default:break;
	}
	Func.Key.Value.UnSign32 = Func.Key.AddDecValue.UnSign32;
}
//大模块：功能码的加减处理
/**
 * 功能码增加
 */
void AddFunCode(uint16_t *Funcode)
{
 	uint16_t Size_Cnt = 0;
	uint8_t i = 0;
	uint16_t Group_Cnt = 0;
	uint16_t Group_Max = 0;
	uint16_t *Group =  Func.Key.FunCodegroup.FuncGroup;
	uint16_t *Size = Func.Key.FunCodegroup.FuncGroupSize;
	uint16_t FuncAddr = 0;
	TYPE_Attribute *FuncAttrAddr = 0;
	UnionType16 Addr;
	Group_Max = Func.Key.FunCodegroup.Group_Max;
	//定位当前功能码
	Addr.All= *Funcode;
	Size_Cnt = Addr.Bit8.DataLow;//当前低地址
	if(Addr.Bit8.DataHigh == PF_FUN_GROUP)
	{
		Group_Cnt =  Func.Key.FunCodegroup.Group_Max;//当前的组数
	}
	else
	{
	    i=0;
		while(Group[i]!=Addr.Bit8.DataHigh)
		{
			i++;
		}
		Group_Cnt =i;//找到当前是第几组
	}
	//增加功能码，根据鼠标位置增加
 	switch(Servo2Panel.Cursor)
 	{
 	case 0:
		if(Size_Cnt ==(Size[Group_Cnt]-1))
		{
			Size_Cnt =0;
		}
		else
		{
			Size_Cnt++;
		}
		break;
	case 1:
		if(Size_Cnt ==(Size[Group_Cnt]-1))
		{
			Size_Cnt =0;
		}
		else if( (Size_Cnt+10) >(Size[Group_Cnt]-1))
		{
			Size_Cnt = Size[Group_Cnt]-1;
		}
		else
		{
			Size_Cnt += 10;
		}
		break;
 	case 2:
 		if(Group_Cnt >= Group_Max)
 		{
			Group_Cnt = 0;
		}
		else
		{
            #if PASYSTEST
			if(FUNCCODEFACTEST == 0 && Group[Group_Cnt + 1] == FUNCCODEGROUPNUM)//在非工厂参数模式下要跳过PA组
			{
				Group_Cnt +=2;
			}
			else
			{
				Group_Cnt ++;
			}
            #else
            Group_Cnt ++;   
            #endif
		}
		Size_Cnt =0;
		break;
 	default:
		break;
 	}
 	//根据当前功能码的属性判断是否跳跃
	Addr.Bit8.DataLow = Size_Cnt;
	Addr.Bit8.DataHigh = Group[Group_Cnt];
	FuncAddr = GetFuncAddr(Addr.All);
	FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
	//memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(FunCodeAttribute)-4);  //TI
	memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(TYPE_Attribute));        //ARM
	while(Func.Exchange.Attr.Property.Bit.IsShow == HIDE)//当前功能码属于不显示的情况
	{
		if(Size_Cnt ==(Size[Group_Cnt]-1))
		{
			Size_Cnt =0;
		}
		else
		{
			Size_Cnt++;
		}
		Addr.Bit8.DataLow = Size_Cnt;
		Addr.Bit8.DataHigh = Group[Group_Cnt];
		FuncAddr = GetFuncAddr(Addr.All);
		FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
		//memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(FunCodeAttribute)-4);   //TI
		memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(TYPE_Attribute));         //ARM
	}
	*Funcode = Addr.All;
}
/**
 * 功能码减小
 */
void DecFunCode(uint16_t *Funcode)
{
	uint16_t Size_Cnt;
	uint16_t i;
	uint16_t Group_Cnt;
	uint16_t Group_Max;
	uint16_t FuncAddr;
	TYPE_Attribute *FuncAttrAddr;
	UnionType16 Addr;
	uint16_t *Size =  Func.Key.FunCodegroup.FuncGroupSize;
	uint16_t *Group = Func.Key.FunCodegroup.FuncGroup;
	Group_Max = Func.Key.FunCodegroup.Group_Max;
	Addr.All= *Funcode;
	Size_Cnt = Addr.Bit8.DataLow;
	if(Addr.Bit8.DataHigh == PF_FUN_GROUP)
	{
		Group_Cnt =  Func.Key.FunCodegroup.Group_Max;
	}
	else
	{
		 i=0;
		while(Group[i]!=Addr.Bit8.DataHigh)
		{
			i++;
		}
		Group_Cnt =i;
	}
 	switch(Servo2Panel.Cursor)
 	{
 		case 0:
			if(Size_Cnt == 0)
			{
				Size_Cnt =Size[Group_Cnt]-1;
			}
			else
			{
				Size_Cnt--;
			}
			break;
		case 1:
			if(Size_Cnt == 0)
			{
				Size_Cnt =Size[Group_Cnt]-1;
			}
			else	if( Size_Cnt<= 10 )
			{
				Size_Cnt = 0;
			}
			else
			{
				Size_Cnt -= 10;
			}
			break;
 		case 2:
			if(Group_Cnt == 0)
			{
				Group_Cnt = Group_Max;
			}
			else
			{
                #if PASYSTEST
				if(FUNCCODEFACTEST == 0 && Group[Group_Cnt - 1] == FUNCCODEGROUPNUM)//在非工厂参数模式下要跳过PA组
				{
					Group_Cnt -=2;
				}
				else
				{
					Group_Cnt --;
				}
                #else
                Group_Cnt --;
                #endif
			}
			Size_Cnt =0;
			break;

		default:	break;
 	}
	Addr.Bit8.DataLow = Size_Cnt;
	Addr.Bit8.DataHigh = Group[Group_Cnt];
	FuncAddr = GetFuncAddr(Addr.All);
	FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
	//memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(FunCodeAttribute)-2);   //TI
	memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(TYPE_Attribute));         //ARM
	while(Func.Exchange.Attr.Property.Bit.IsShow == HIDE)//当前功能码属于不显示的情况
	{
		if(Servo2Panel.Cursor == 3)
		{
			if(Size_Cnt ==(Size[Group_Cnt]-1))
			{
				Size_Cnt =0;
			}
			else
			{
				Size_Cnt++;
			}
		}
		else
		{
			if(Size_Cnt == 0)
			{
				Size_Cnt =Size[Group_Cnt]-1;
			}
			else
			{
				Size_Cnt--;
			}
		}
		Addr.Bit8.DataLow = Size_Cnt;
		Addr.Bit8.DataHigh = Group[Group_Cnt];
		FuncAddr = GetFuncAddr(Addr.All);
		FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
		//memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(FunCodeAttribute)-2);   //TI
		memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(TYPE_Attribute));         //ARM
	}
	*Funcode = Addr.All;
 }
/**
 * 查找上一个非出厂值功能码，功能码减小
 */
void FinePreNoneFacFunCode( )
{
	uint16_t TmpFuncode = NoFacFunc.CurAddr.All;//当前的非出厂值功能码
	uint16_t FuncAddr;//当前功能码
	TYPE_Attribute *FuncAttrAddr;
	uint16_t *FacAddr = 0;//出厂值地址
	uint16_t *RamRddr = 0;//内存地址
	//每次查找10次
	uint16_t i;
	uint16_t FindTimes = 0;//查找次数
	uint16_t Size_Cnt;//功能码内部第几个
	uint16_t Group_Cnt;//第几组
	uint16_t Group_Max;//最大组数

	UnionType16 Addr;
	uint16_t *Size =  Func.Key.FunCodegroup.FuncGroupSize;
	uint16_t *Group = Func.Key.FunCodegroup.FuncGroup;
	Group_Max = Func.Key.FunCodegroup.Group_Max;
	Addr.All= TmpFuncode;//记录本次的功能码
	Size_Cnt = Addr.Bit8.DataLow;//记录功能码低位
	if(Addr.Bit8.DataHigh == PF_FUN_GROUP)//记录高8位是第几组
	{
		Group_Cnt =  Func.Key.FunCodegroup.Group_Max;
	}
	else
	{
		i=0;
		while(Group[i]!=Addr.Bit8.DataHigh)
		{
			i++;
		}
		Group_Cnt =i;
	}
	do//查找上一个功能码
	{
		FindTimes ++;
		do{
			if(Size_Cnt == 0)//已经到了最低一个了
			{
				if(Group_Cnt <= 1)//已经到了第一组了
				{
					Group_Cnt = Group_Max;
				}
				else
				{
                    #if PASYSTEST
					if(FUNCCODEFACTEST == 0 && Group[Group_Cnt-1] == FUNCCODEGROUPNUM)//在非工厂参数模式下要跳过PA组
					{
						Group_Cnt -=2;
					}
					else
					{
						Group_Cnt --;
					}
                    #else
                    Group_Cnt --;
                    #endif
				}
				Size_Cnt = Size[Group_Cnt]-1;
			}
			else
			{
				Size_Cnt--;
			}
			Addr.Bit8.DataLow = Size_Cnt;
			Addr.Bit8.DataHigh = Group[Group_Cnt];
			FuncAddr = GetFuncAddr(Addr.All);
			FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
			//memcpy(&NoFacFunc.Property, FuncAttrAddr, 1);//复制属性    //TI
			memcpy(&NoFacFunc.Property, FuncAttrAddr, 2);//复制属性      //ARM
		}while(Func.Exchange.Attr.Property.Bit.IsShow == HIDE);

		NoFacFunc.CurAddr.All = Addr.All;//查看该功能码是否是
		RamRddr = RamFuncStartAddr  + GetFuncAddr(Addr.All) ;//功能码ram区功能码地址
		FacAddr = FactoryFuncStartAddr  + GetFuncAddr(Addr.All) ;//出厂值起始地址
		if(NoFacFunc.Property.Bit.BitSize == ATTR_BIT_32)
		{
			NoFacFunc.Property.Bit.Sign = (FuncAttrAddr+1)->Sign;//更新属性
		}
		if(NoFacFunc.Property.Bit.BitSize == ATTR_BIT_16)//单字节
		{
			NoFacFunc.Value.UnSign32 =(uint16_t) *RamRddr;
			NoFacFunc.FacValue.UnSign32 =(uint16_t) *FacAddr;
		}
		else
		{
			NoFacFunc.Value.Byte32.DataHigh =(uint16_t) *(RamRddr+1);
			NoFacFunc.Value.Byte32.DataLow=(uint16_t) *(RamRddr);
			NoFacFunc.FacValue.Byte32.DataHigh =(uint16_t) *(FacAddr+1);
			NoFacFunc.FacValue.Byte32.DataLow=(uint16_t) *(FacAddr);
		}
		if(NoFacFunc.Value.UnSign32 != NoFacFunc.FacValue.UnSign32)
		{
			NoFacFunc.FineResult = FINDOUT;//找到了
			NoFacFunc.FinalAddr.All = NoFacFunc.CurAddr.All;
			NoFacFunc.FirstAddr.All = NoFacFunc.FinalAddr.All;
			break;
		}
		if(NoFacFunc.FirstAddr.All == NoFacFunc.CurAddr.All)
		{
			NoFacFunc.FineResult = FINDFULL;//查找完了一遍
		}
	}while(FindTimes<10 && NoFacFunc.FineResult != FINDOUT && NoFacFunc.FineResult != FINDFULL);//没有找到，时间到了，找完一遍都要结束

}
void FontNextNoneFacFuncode()
{
	uint16_t TmpFuncode = NoFacFunc.CurAddr.All;
	uint16_t *FacAddr = 0;
	uint16_t *RamRddr = 0;
	//每次查找一个
	uint16_t FindTimes = 0;
	uint16_t Size_Cnt;
	uint16_t i;
	uint16_t Group_Cnt;
	uint16_t Group_Max;
	uint16_t FuncAddr;
	TYPE_Attribute *FuncAttrAddr;
	UnionType16 Addr;
	uint16_t *Size =  Func.Key.FunCodegroup.FuncGroupSize;
	uint16_t *Group = Func.Key.FunCodegroup.FuncGroup;
	Group_Max = Func.Key.FunCodegroup.Group_Max;
	Addr.All= TmpFuncode;//记录本次的功能码
	Size_Cnt = Addr.Bit8.DataLow;//记录功能码低位
	if(Addr.Bit8.DataHigh == PF_FUN_GROUP)//记录高8位是第几组
	{
		Group_Cnt =  Func.Key.FunCodegroup.Group_Max;
	}
	else
	{
		i=0;
		while(Group[i]!=Addr.Bit8.DataHigh)
		{
			i++;
		}
		Group_Cnt =i;
	}
	do//查找下一个功能码
	{
		FindTimes ++;
		do{
			if(Size_Cnt >= Size[Group_Cnt]-1)//已经到了功能码组最后一个
			{
				if(Group_Cnt >= Group_Max)//已经到了第一组了
				{
					Group_Cnt = 1;
				}
				else
				{
                    #if PASYSTEST
					if(FUNCCODEFACTEST == 0 && Group[Group_Cnt + 1] == FUNCCODEGROUPNUM)//在非工厂参数模式下要跳过PA组
					{
						Group_Cnt +=2;
					}
					else
					{
						Group_Cnt ++;
					}
                    #else
                    Group_Cnt ++;
                    #endif
				}
				Size_Cnt =0;
			}
			else
			{
				Size_Cnt ++;
			}
			Addr.Bit8.DataLow = Size_Cnt;
			Addr.Bit8.DataHigh = Group[Group_Cnt];
			FuncAddr = GetFuncAddr(Addr.All);
			FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
			//memcpy(&NoFacFunc.Property, FuncAttrAddr, 1);//复制属性   //TI
			memcpy(&NoFacFunc.Property, FuncAttrAddr, 2);//复制属性     //ARM
		}while(Func.Exchange.Attr.Property.Bit.IsShow == HIDE);
		NoFacFunc.CurAddr.All = Addr.All;
		RamRddr = RamFuncStartAddr  + GetFuncAddr(Addr.All) ;//功能码ram区功能码地址
		FacAddr = FactoryFuncStartAddr  + GetFuncAddr(Addr.All) ;//出厂值起始地址
		if(NoFacFunc.Property.Bit.BitSize == ATTR_BIT_32)
		{
			NoFacFunc.Property.Bit.Sign = (FuncAttrAddr+1)->Sign;//更新属性
		}
		if(NoFacFunc.Property.Bit.BitSize == ATTR_BIT_16)//单字节
		{
			NoFacFunc.Value.UnSign32 =(uint16_t) *RamRddr;
			NoFacFunc.FacValue.UnSign32 =(uint16_t) *FacAddr;
		}
		else
		{
			NoFacFunc.Value.Byte32.DataHigh =(uint16_t) *(RamRddr+1);
			NoFacFunc.Value.Byte32.DataLow=(uint16_t) *(RamRddr);
			NoFacFunc.FacValue.Byte32.DataHigh =(uint16_t) *(FacAddr+1);
			NoFacFunc.FacValue.Byte32.DataLow=(uint16_t) *(FacAddr);
		}
		if(NoFacFunc.Value.UnSign32 != NoFacFunc.FacValue.UnSign32)
		{
			NoFacFunc.FineResult = FINDOUT;//找到了
			NoFacFunc.FinalAddr.All = NoFacFunc.CurAddr.All;
			NoFacFunc.FirstAddr.All = NoFacFunc.FinalAddr.All;
		}
		if(NoFacFunc.FirstAddr.All == NoFacFunc.CurAddr.All)
		{
			NoFacFunc.FineResult = FINDFULL;//查找完了一遍
		}
	}while(FindTimes<10 && NoFacFunc.FineResult != FINDOUT && NoFacFunc.FineResult != FINDFULL);//没有找到，时间到了，找完一遍都要结束

}
//大模块：光标处理模块
/**
 * 十进制下求光标最大值
 */
void DecimalCussorLimit(void)
{
	uint32_t LimitData = 0;
	 if(Func.Key.Attr.Property.Bit.Sign == TYPE_UNSIGNED)
	 {
		 LimitData = Func.Key.Attr.Max.UnSign32;
	 }
	 else
	 {
		 LimitData = (uint32_t)-Func.Key.Attr.Min.Sign32;
		 if(LimitData < Func.Key.Attr.Max.UnSign32)
		 {
			 LimitData = Func.Key.Attr.Max.UnSign32;
		 }
	 }
	if(LimitData < 10)
	{
		Servo2Panel.CursorMAX = 0;
	}
	else if(LimitData < 100)
	{
		Servo2Panel.CursorMAX = 1;
	}
	else if(LimitData < 1000)
	{
		Servo2Panel.CursorMAX = 2;
	}
	else if(LimitData < 10000)
	{
		Servo2Panel.CursorMAX = 3;
	}
	else if(LimitData < 100000)
	{
		Servo2Panel.CursorMAX = 4;
	}
	else if(LimitData < 1000000)
	{
		Servo2Panel.CursorMAX = 5;
	}
	else if(LimitData < 10000000)
	{
		Servo2Panel.CursorMAX = 6;
	}
	else if(LimitData < 100000000)
	{
		Servo2Panel.CursorMAX = 7;
	}
	else if(LimitData < 1000000000)
	{
		Servo2Panel.CursorMAX = 8;
	}
	else
	{
		Servo2Panel.CursorMAX = 9;
	}
}
/**
 * 二进制情况下求光标最大值
 */
void BinaryCussorLimit(void)
{
	uint32_t tmpcursor = 31;
	while(!(Func.Key.Attr.Max.UnSign32&((uint32_t)0x1<<tmpcursor))&&tmpcursor>0)
	{
		tmpcursor --;
	}
	Servo2Panel.CursorMAX = tmpcursor;
}
/**
 * 十六进制下求光标最大值
 */
void HexCussorLimit()
{
	uint32_t tmpcursor = 7;
	uint32_t Max = Func.Key.Attr.Max.UnSign32;
	uint32_t tmp1 = ((uint32_t)0x0000000f<<(tmpcursor<<2));
	uint32_t tmp = Max&tmp1;
	while((!tmp)&&(tmpcursor>0))
	{
		tmpcursor --;
		tmp1 = ((uint32_t)0x0000000f<<(tmpcursor<<2));
		tmp = Max&tmp1;
	}
	Servo2Panel.CursorMAX = tmpcursor;
}
//大模块：功能码存储接口
/**
 * 通过SET按键将修改过得功能码写入EEPROM中的任务触发函数
 */                         //表示功能码                                参数值指针                               数量
void Panel_Write_EEP(uint16_t EEP_Addr, uint16_t *EEP_Data, uint16_t EEP_Number)
{
	TYPE_WORD Rx_Func;
	uint16_t *Ram_Addr;//RAM地址
	uint16_t FuncAddr;
    TYPE_Attribute Attributefun;
    TYPE_Attribute *FuncAttrAddr;
    
	Rx_Func.all = EEP_Addr;//要写的功能码

	FuncAddr = GetFuncAddr(Rx_Func.all);
	Ram_Addr = RamFuncStartAddr + FuncAddr ;//ram取得地址
    
    FuncAttrAddr = FuncAttrStartAddr + FuncAddr;
    memcpy(&Attributefun,FuncAttrAddr,2);//功能码属性
    
    if(StateMachine.RegulFlg > 0 && Attributefun.Operation == ATTR_EFT_RunOff)
    {
        return;
    }
    
    //从这里判断读写属性
	switch(Func.Key.ControlBit.FuncChoose)
	{
		case FUNC_SAVE://写保存命令
			//memcpy(Ram_Addr,EEP_Data,EEP_Number);   //TI
		    memcpy(Ram_Addr,EEP_Data,EEP_Number*2);   //ARM
			{
				IIcInterfaceA(Ram_Addr,EEP_Number);
			}
			break;
		case FUNC_NOSAVE://写不保存命令
			//memcpy(Ram_Addr,EEP_Data,EEP_Number);   //TI
		    memcpy(Ram_Addr,EEP_Data,EEP_Number*2);   //ARM
			 /*准备好一帧发送数据*/
			break;
			default:break;
	}
}
/**
* 按键存储操作
*/
void PanelEEPROMHandle(void)
{
	TYPE_WORD Rx_Func;
	TYPE_DWORD W_Data;
	Rx_Func.Byte.High =  Func.Key.CurAddr.Bit8.DataHigh;/*要写的功能码*/
	Rx_Func.Byte.Low  =  Func.Key.CurAddr.Bit8.DataLow;
	if(Func.Key.Attr.Property.Bit.BitSize == ATTR_BIT_16)//16位长度
	{
		W_Data.Byte.Low  =  Func.Key.AddDecValue.UnSign32&0xffff;
		Panel_Write_EEP(Rx_Func.all,W_Data.DWord,1);		//存储
	}
	else if(Func.Key.Attr.Property.Bit.BitSize == ATTR_BIT_32)//32位长度
	{
		W_Data.Byte.Low =  Func.Key.AddDecValue.Byte32.DataLow;/*要写的数据*/
		W_Data.Byte.High  =  Func.Key.AddDecValue.Byte32.DataHigh;
		Panel_Write_EEP(Rx_Func.all,W_Data.DWord,2);		//存储
	}
}
/***************************************按键模块*************************************/
/**
 * 按键扫描函数
 * 函数功能：扫描当前按键值，进行一定时间的滤波，确定最终按键类型。
 * 本函数可以检测到按键是单击按下（15ms）或者是长时间按下（400ms）
 */
void Panel_KeyScan(void)
{
   KeyInfo.KeyPort = KeyInfo.KeyReceiveData;		  /*把键盘口全部拉高， */
   KeyInfo.KeyType =0;
   KeyInfo.CurKeyValue  =0;
   if(KeyInfo.KeyPort != KEY_CODE_IDLE)               /*如果检查到低电平，则说明有按键按下*/
   {
       if(KeyInfo.KeyDownCnt ==0)
       {
           KeyInfo.PreKeyValue=KeyInfo.KeyPort;
		   KeyInfo.KeyDownCnt++;//按键按下计时
       }
	   KeyInfo.KeyUpCnt=0;
	   KeyInfo.keyDownHold =1;//按键保持按下
   	}
    else
   	{
    	//按键弹起
    	KeyInfo.keyHold.Bit.KeyLongSet = 0;
   		KeyInfo.keyDownHold =0;//按键弹起
   		if(KeyInfo.PreKeyValue)
   		{
   			KeyInfo.KeyUpCnt++;
   		}
		else
		{
			KeyInfo.KeyUpCnt =0;
		}
   	}
   if(KeyInfo.KeyDownCnt == KEY_SHAKE_DELAY)
   {
   		if(KeyInfo.PreKeyValue == KeyInfo.KeyPort ) //本次按键值跟20ms前键值相等
   		{
   			 KeyInfo.KeyType = KeyInfo.PreKeyValue;//按键有效
			 KeyInfo.KeyDownCnt++;//按键保持按下计时
   		}
		else
		{

			KeyInfo.KeyDownCnt=0;//按键按下计时清零
		}
   }
   else if(KeyInfo.KeyDownCnt >= KEY_KEEP_DOWN_TIME)
   {
	    KeyInfo.keyHold.Bit.KeyLongSet = 1;
   		KeyInfo.KeyDownCnt = 320;//大于两次按键有效判断间隔时间按键按下时间清0
   		if(KeyInfo.PreKeyValue == KeyInfo.KeyPort ) //本次按键值跟20ms前键值相等
   		{
   		   	KeyInfo.KeyType = KeyInfo.PreKeyValue;//按键有效
   			KeyInfo.KeyDownCnt++;//按键保持按下计时
   		}
   		else
   		{
   			KeyInfo.KeyDownCnt=0;//按键按下计时清零
   		}
   }
   else	if(KeyInfo.KeyDownCnt!=0)//按键按下但未到达去抖判断时间
   {
	   KeyInfo.keyHold.Bit.KeyLongSet = 0;
	   if(KeyInfo.KeyPort != KEY_CODE_IDLE) //当前按键键值不为0
		 {
			 KeyInfo.KeyDownCnt ++;//按键按下计时
		 }
		 else
		 {
			 KeyInfo.KeyDownCnt =0;

		 }
   }
	switch(KeyInfo.KeyType)
	{
	case KEY_CODE_NULL:
		 KeyInfo.CurKeyValue = 0;
		 break;
	case KEY_CODE_SET:
		 KeyInfo.CurKeyValue = KEY_SET ;
	     break;
	case KEY_CODE_SHIFT:
	     KeyInfo.CurKeyValue = KEY_SHIFT ;
	     break;
	case KEY_CODE_UP:
	     KeyInfo.CurKeyValue = KEY_UP ;
	     break;
	case KEY_CODE_DOWN:
	     KeyInfo.CurKeyValue = KEY_DOWN ;
	     break;
	case KEY_CODE_MODE:
		 KeyInfo.CurKeyValue = KEY_MODE ;
		 break;
	default :
	     KeyInfo.CurKeyValue = 0;
	     break;
	}
	if(KeyInfo.KeyUpCnt > KEY_SHAKE_DELAY)//按键弹起
	{
		if(KeyInfo.keyHold.Bit.key_Up)
		{
			KeyInfo.CurKeyValue = KEY_UP_UP;
		}
		else if(KeyInfo.keyHold.Bit.key_Down)
		{
			KeyInfo.CurKeyValue = KEY_DOWN_UP;
		}
	}
}
//Set按键处理模块
/**
 * Set按键Level1入口函数
 * 函数功能：进入Level2
 */
void PanelSetLevel1Handle(void)
{
	Servo2Panel.Level = PANELCTL_LEVEL2;
	Servo2Panel.Cursor = 0;
	if(RamAid->PanelType)
	{
		switch(Servo2Panel.MenuType_AC)
		{
		case MENUTYPE_A:
		default:
			if(Func.Key.CurAddr.Bit8.DataHigh == 0)
			{
				Func.Key.CurAddr.All = Func.Key.PreIsP0Addr.All;
				Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
				Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			}
			Servo2Panel.Cursor =  2;
			break;
		case MENUTYPE_C:
			if(Func.Key.ControlBit.PFPassword == PASSWORDOPEN)
			{
				NoFacFunc.FirstAddr.Bit8.DataHigh = Func.Key.FunCodegroup.FuncGroup[Func.Key.FunCodegroup.Group_Max-1];
				NoFacFunc.FirstAddr.Bit8.DataLow = Func.Key.FunCodegroup.FuncGroupSize[Func.Key.FunCodegroup.Group_Max-1]-1;
			}
			else
			{
				NoFacFunc.FirstAddr.Bit8.DataHigh = Func.Key.FunCodegroup.FuncGroup[Func.Key.FunCodegroup.Group_Max-2];
				NoFacFunc.FirstAddr.Bit8.DataLow = Func.Key.FunCodegroup.FuncGroupSize[Func.Key.FunCodegroup.Group_Max-2]-1;

			}
			NoFacFunc.CurAddr.All = Func.Key.FunCodegroup.FuncGroupSize[0]-1;
			NoFacFunc.FineResult = FINDUP;
			break;
		}
	}
	else
	{
		if(Func.Key.CurAddr.Bit8.DataHigh == 0)
		{
			Func.Key.CurAddr.All = Func.Key.PreIsP0Addr.All;
			Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		}
	}
}
/**
 * Set按键Level2入口函数
 * 函数功能：进入Level3
 */
void PanelSetLevel2Handle(void)
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	Servo2Panel.Bit.P0Count = 0;
	Servo2Panel.P0CountDown = 0;
	if(RamAid->PanelType)
	{
		switch(Servo2Panel.MenuType_AC)
		{
		case MENUTYPE_A:
		default:
			if(Func.Key.CurAddr.Bit8.DataHigh)//当前功能码是否是P0组
			{
				Func.Key.PreNotP0Addr.All = Func.Key.CurAddr.All;
			}
			else
			{
				Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
			}
			break;
		case MENUTYPE_C:
			Func.Key.CurAddr.All = NoFacFunc.FinalAddr.All;
			break;
		}
	}
	else
	{
		if(Func.Key.CurAddr.Bit8.DataHigh)//当前功能码是否是P0组
		{
			Func.Key.PreNotP0Addr.All = Func.Key.CurAddr.All;
		}
		else
		{
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
		}
	}
	if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检
	{
        #if KEYCHECK
		SELFKEY_FUNCCODE = 4;
        #endif
		Servo2Panel.Level = PANELCTL_LEVEL3;
		Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Cursor = 0;
	}
	else if(Func.Key.CurAddr.All == FUNCODE_JOG)//点动使能命令
	{
		if((Servo2Panel.Bit.Fault != 1)&&(Servo2Panel.Bit.Warning != 1) && (sm->RegulFlg != 1))
	    {
            Servo2Panel.JOG_FWD_REV = JOG_STOP;
            //发出使能命令
            Servo2Panel.Bit.JOG = 1;
            Servo2Panel.Level = PANELCTL_LEVEL3;
            Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
            Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
            Servo2Panel.Cursor = 0;
		}
	}
	else if(Func.Key.CurAddr.All == FUNCODE_FFT)//FFT
	{
		if((Servo2Panel.Bit.Fault != 1)&&(Servo2Panel.Bit.Warning != 1) && (sm->RegulFlg != 1))
		{
			//发出使能命令
			Servo2Panel.Level = PANELCTL_LEVEL3;
			Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			Servo2Panel.Cursor = 0;
		}
	}
	else if(Func.Key.CurAddr.All == FUNCODE_SELFLEAN)//自学习
	{
		if((Servo2Panel.Bit.Fault != 1)&&(Servo2Panel.Bit.Warning != 1) && (sm->RegulFlg != 1))
		{
			Servo2Panel.Level = PANELCTL_LEVEL3;
			Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			Servo2Panel.Cursor = 0;
		}
	}
	else if(Func.Key.CurAddr.All == FUNCODE_INERTIALEAN)//离线惯量辨识
	{
		if((Servo2Panel.Bit.Fault != 1)&&(Servo2Panel.Bit.Warning != 1) && (sm->RegulFlg != 1))
		{
			Servo2Panel.Level = PANELCTL_LEVEL3;
			Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			Servo2Panel.Cursor = 0;
		}
	}
	else if(Func.Key.CurAddr.All ==FUNCODE_RUN)//RUN
	{
		 if( !StateMachine.RegulFlg||(StateMachine.RegulFlg &&RamAid->SON))//已经使能，RUN使能
		 {
			 Servo2Panel.Level = PANELCTL_LEVEL3;
			 Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			 Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			 Servo2Panel.Cursor = 0;
		 }
	}
	else//其他
	{
		Servo2Panel.Level = PANELCTL_LEVEL3;
		Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Cursor = 0;
	}
}
/**
 * Set按键Level3入口函数
 */
void PanelSetLevel3Handle(void)
{
	//TYPE_STATEMACHINE *sm = &StateMachine;
	if(Func.Key.CurAddr.All ==FUNCODE_SELFLEAN &&Func.Key.Value.UnSign32 != 0)//当前是自学习功能码
	{
		RamMotor->TuningType = Func.Key.Value.UnSign32;
		//Servo2Panel.Bit.StartSelfLean = 1;//启动自学习 2024.09.03屏蔽
	}
	else if(Func.Key.CurAddr.All == FUNCODE_INERTIALEAN && Func.Key.Value.UnSign32 != 0)//当前是离线惯量自学习
	{

		Servo2Panel.Bit.StartLeanInertia = 1;//离线惯量自学习
		RamAid->InertiaIdentificate = Func.Key.Value.UnSign32;
	}
	else if(Func.Key.CurAddr.All == FUNCODE_FFT && Func.Key.Value.UnSign32 != 0)//当前是FFT
	{
		Servo2Panel.Bit.FFt = 1;//离线惯量自学习
		RamAid->FFT = Func.Key.Value.UnSign32;
	}
	else if(Func.Key.CurAddr.All == 0x0E2F&&Func.Key.Value.UnSign32 != 0)
	{
		//RamMotor->WRMotorCode = 0;     // 2024.09.10
		//EncInfo.EncFlag.WMotorcode = 1;//写电机代码  2022.6.7
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检
	{
		/*
		if(SELFKEY_FUNCCODE == 4)//按
		{
			SELFKEY_FUNCCODE = 0;
			Func.Key.CurAddr.All = SELFINSPECTION_LED;
			Servo2Panel.Level --;
		}
		*/
        #if KEYCHECK
		SELFKEY_FUNCCODE = 4;
        #endif
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_LED)//LED自检，退出到上一级菜单
	{
		Servo2Panel.Level --;
        #if KEYCHECK
		SELFLED_FUNCCODE = 0;
        #endif
	}
	else if(Func.Key.CurAddr.Bit8.DataHigh != 0x00&&Func.Key.CurAddr.All != FUNCODE_JOG)
	{
		if(Func.Key.CurAddr.All == PFPASSWORDFUNCODE)//P163,PF组密码状态
		{
			Func.Key.ControlBit.FuncChoose = FUNC_NOSAVE;
			if(Func.Key.AddDecValue.UnSign32 == PFPASSWORDNUM)
			{
				Func.Key.ControlBit.PFPassword = ~Func.Key.ControlBit.PFPassword;
			}
			Func.Key.AddDecValue.UnSign32 = 0;
		}
		else if(Func.Key.CurAddr.All == PEPASSWORDFUNCODE)//PE00,密码
		{
			if(Func.Key.AddDecValue.UnSign32 == PEPASSWORDNUM)
			{
				Func.Key.ControlBit.PEPassword = ~Func.Key.ControlBit.PEPassword;
			}
			Func.Key.AddDecValue.UnSign32 = 0;
			Func.Key.ControlBit.FuncChoose = FUNC_NOSAVE;
		}
		else
		{
			Func.Key.ControlBit.FuncChoose = FUNC_SAVE;
		}
		Servo2Panel.Cursor = 0;
		PanelEEPROMHandle();
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		if(Func.Key.Attr.Property.Bit.Operation == ATTR_EFT_PowerOn)//需重新上电有效
		{
			Servo2Panel.Bit.PoOn = 1;//需要重新上电才能有效，显示用
			Servo2Panel.Bit.IsPoOn = 1;//表示有这种操作
			Servo2Panel.PoOnTimeDown = 0;
		}
		else
		{
			Servo2Panel.Bit.PoOn = 0;
		}
		Servo2Panel.Level = PANELCTL_LEVEL4;
		Servo2Panel.Level4TimeDown = 1000;
	}
	else
	{
	}
}
/**
 * Set按键接口函数
 */
void KeyProcess_Set()
{
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0://零级菜单用于显示提示信息
		break;
	case PANELCTL_LEVEL1:
		PanelSetLevel1Handle();
		break;
	case PANELCTL_LEVEL2:
		PanelSetLevel2Handle();
		break;
	case PANELCTL_LEVEL3://用于发送存储命令
		PanelSetLevel3Handle();
		break;
	case PANELCTL_LEVEL4:
		break;
	}
}
//Mode按键处理模块
/**
 * Mode按键1级菜单入口函数
 */
void PanelModeLevel1Handle(void)
{
	if(Func.Key.CurAddr.Bit8.DataHigh != 0)//非P0组
	{
		Func.Key.PreNotP0Addr.All = Func.Key.CurAddr.All;
	}
	Func.Key.CurAddr.All = Func.Key.PreIsP0Addr.All;
	Servo2Panel.MenuType_AC = MENUTYPE_A;
	Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
	Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
	Servo2Panel.Level = PANELCTL_LEVEL2;
	Servo2Panel.Cursor = 0;
	Servo2Panel.Bit.P0Count = 1;
	Servo2Panel.P0CountDown = 1000;
}
/**
 * Mode按键2级菜单入口函数
 */
void PanelModeLevel2Handle(void)
{
	if(Func.Key.CurAddr.Bit8.DataHigh == 0)//监视的是P0组
	{
		Servo2Panel.Bit.P0Count = 0;
		Servo2Panel.P0CountDown = 0;
		if(Servo2Panel.Bit.P0Count)
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
		Servo2Panel.Level = PANELCTL_LEVEL1;
		Servo2Panel.Cursor = 2;
	}
	else
	{
		Servo2Panel.Level = PANELCTL_LEVEL1;
		Servo2Panel.Cursor = 2;
	}
}
/**
 * Mode按键3级菜单入口函数
 */
void PanelModeLevel3Handle(void)
{
	if(Func.Key.CurAddr.Bit8.DataHigh == 0)//监视的是P0组
	{
		Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
		Func.Key.CurAddr.All = Func.Key.PreNotP0Addr.All;
		Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Level = PANELCTL_LEVEL1;
		Servo2Panel.Cursor = 2;
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检,MODE按键退出
	{
        #if KEYCHECK
		SELFKEY_FUNCCODE = 0;
        #endif
		Func.Key.CurAddr.All = SELFINSPECTION_LED;
		Servo2Panel.Level --;
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_LED)//LED自检，退出到上一级菜单
	{
		Servo2Panel.Level --;
        #if KEYCHECK
		SELFLED_FUNCCODE = 0;
        #endif
	}
	else
	{
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Level = PANELCTL_LEVEL2;
		Servo2Panel.Cursor = 0;
	}
}
/**
 * Mode按键入口函数
 */
void KeyProcess_Mode()
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	if(Servo2Panel.Bit.JOG == 1)
	{
		Servo2Panel.Bit.JOG = 0;
	}
	if(Servo2Panel.Status == STATUS_SELF)//自学习过程中，按Mode按键可以退出
	{
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Level = PANELCTL_LEVEL2;
		Servo2Panel.Cursor = 0;
		Servo2Panel.Bit.StartSelfLean = 0;
		sm->CmdIn.bit.LRN = 0;
	}
	if(Servo2Panel.Status == STATUS_INERTIALEAN)//离线惯量辨识过程中，按Mode按键可以退出
	{
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Level = PANELCTL_LEVEL2;
		Servo2Panel.Cursor = 0;
		Servo2Panel.Bit.StartLeanInertia = 0;
		sm->CmdIn.bit.Inertia = 0;
	}
	if(Servo2Panel.Status == STATUS_FFT)//FFT
	{
		Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
		Servo2Panel.Level = PANELCTL_LEVEL2;
		Servo2Panel.Cursor = 0;
		Servo2Panel.Bit.FFt = 0;
		sm->CmdIn.bit.FFt = 0;
	}
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0:
		break;
	case PANELCTL_LEVEL1:
		PanelModeLevel1Handle();
		break;
	case PANELCTL_LEVEL2:
		PanelModeLevel2Handle();
		break;
	case PANELCTL_LEVEL3:
		PanelModeLevel3Handle();
		break;
	case PANELCTL_LEVEL4:
		Servo2Panel.Level = PANELCTL_LEVEL2;
	case PANELCTL_LEVEL5:
		Servo2Panel.Level = PANELCTL_LEVEL2;
		break;
	}
}
//UP按键处理模块
/**
 * Up按键1级菜单处理入口
 */
void PanelUpLevel1Handle(void)
{
	if(RamAid->PanelType)
	{
		if(Servo2Panel.MenuType_AC != 'A')
		{
			Servo2Panel.MenuType_AC = 'A';
		}
		else
		{
			Servo2Panel.MenuType_AC = 'C';
		}
	}
	else
	{
		AddFunCode(&Func.Key.CurAddr.All);
		if(Servo2Panel.Bit.P0Count)
		{
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
			Servo2Panel.P0CountDown = 3000;
		}
	}
}
/**
 * Up按键2级菜单处理入口
 */
void PanelUpLevel2Handle(void)
{
	if(Servo2Panel.Bit.P0Count)//监视组
	{
		Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
		Servo2Panel.P0CountDown = 3000;
		AddFunCode(&Func.Key.CurAddr.All);
	}
	else
	{
		if(RamAid->PanelType)//查找上一个非出厂值功能码
		{
			switch(Servo2Panel.MenuType_AC)
			{
			case MENUTYPE_A:
			default:
				AddFunCode(&Func.Key.CurAddr.All);
				break;
			case MENUTYPE_C:
				NoFacFunc.FineResult = FINDUP;
				break;
			}
		}
		else
		{
			AddFunCode(&Func.Key.CurAddr.All);
		}
	}
}
/**
 * Up按键3级菜单处理入口
 */
void PanelUpLevel3Handle(void)
{
	if (Func.Key.CurAddr.All == JOGFUNCODE&&!Servo2Panel.Bit.Fault&&!Servo2Panel.Bit.Warning)//UP功能
	{
		//点动正转命令
		KeyInfo.keyHold.Bit.key_Up = 1;
		Servo2Panel.JOG_FWD_REV = JOG_FWD;
	}
	else
	{
		if(Func.Key.CurAddr.Bit8.DataHigh == 0)//P0组处理方式
		{
			Servo2Panel.Bit.P0Count = 1;
			Servo2Panel.P0CountDown = 3000;
			Servo2Panel.Cursor = 0;
			AddFunCode(&Func.Key.CurAddr.All);
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
			Servo2Panel.Level = PANELCTL_LEVEL2;
		}
		else if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检
		{
            #if KEYCHECK
			SELFKEY_FUNCCODE = 1;
            #endif
		}
		else if(Func.Key.CurAddr.All == SELFINSPECTION_LED)//LED自检，无作用
		{

		}
		else
		{
			if(Func.Key.Attr.Property.Bit.Operation != ATTR_OPE_ReadOnly)
			{
				Func.Led.ControlBit.ShowUpDownValue = SHOWUPDOWNVALUE;//当前显示Up/Down之后的值
				FunCodeValueAdd();
				Func.Key.ControlBit.FuncChoose = FUNC_NOSAVE;
			}
		}
	}
}
/**
 * Up按键入口函数
 */
void KeyProcess_Up()
{
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0://零级菜单用于显示提示信息，
		break;
	case PANELCTL_LEVEL1:
		PanelUpLevel1Handle();
		break;
	case PANELCTL_LEVEL2:
		PanelUpLevel2Handle();
		break;
	case PANELCTL_LEVEL3://用于增加参数，P0组用于切换下一个功能码
		PanelUpLevel3Handle();
		break;
	case PANELCTL_LEVEL4:
		break;
	}
}
//DOWN按键处理模块
/**
 * Down按键Level1处理模块
 */
void PanelDownLevel1Handle(void)
{
	if(RamAid->PanelType)
	{
		if(Servo2Panel.MenuType_AC != 'A')
		{
			Servo2Panel.MenuType_AC = 'A';
		}
		else
		{
			Servo2Panel.MenuType_AC = 'C';
		}
	}
	else
	{
		DecFunCode(&Func.Key.CurAddr.All);
		if(Servo2Panel.Bit.P0Count)
		{
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
			Servo2Panel.P0CountDown = 3000;
		}
	}
}
/**
 * Down按键Level2处理模块
 */
void PanelDownLevel2Handle(void)
{

	if(Servo2Panel.Bit.P0Count)
	{
		DecFunCode(&Func.Key.CurAddr.All);
		Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
		Servo2Panel.P0CountDown = 3000;
	}
	else
	{
		if(RamAid->PanelType)
		{
			switch(Servo2Panel.MenuType_AC)
			{
			case MENUTYPE_A:
			default:
				DecFunCode(&Func.Key.CurAddr.All);
				break;
			case MENUTYPE_C:
				NoFacFunc.FineResult = FINTDOWN;
				break;
			}
		}
		else
		{
			DecFunCode(&Func.Key.CurAddr.All);
		}
	}
}
/**
 * Down按键Level3处理模块
 */
void PanelDownLevel3Handle(void)
{
	if (Func.Key.CurAddr.All == 0x0802&&!Servo2Panel.Bit.Fault&&!Servo2Panel.Bit.Warning)//DOWN功能
	{
		KeyInfo.keyHold.Bit.key_Down = 1;
		Servo2Panel.JOG_FWD_REV = JOG_REV;
	}
	if(Func.Key.CurAddr.Bit8.DataHigh == 0)
	{
		Servo2Panel.Bit.P0Count = 1;
		Servo2Panel.P0CountDown = 3000;
		Servo2Panel.Cursor = 0;
		DecFunCode(&Func.Key.CurAddr.All);
		Servo2Panel.Level = PANELCTL_LEVEL2;
		if(Func.Key.CurAddr.Bit8.DataHigh == 0)
			Func.Key.PreIsP0Addr.All = Func.Key.CurAddr.All;
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检,DOWN
	{
        #if KEYCHECK
		SELFKEY_FUNCCODE =2;
        #endif
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_LED)//LED自检，无作用
	{

	}
	else
	{
		if(Func.Key.Attr.Property.Bit.Operation != ATTR_OPE_ReadOnly)
		{
			Func.Led.ControlBit.ShowUpDownValue = SHOWUPDOWNVALUE;//当前显示Up/Down之后的值
			FunCodeValueDec();
			Func.Key.ControlBit.FuncChoose = FUNC_NOSAVE;
		}
	}

}
void KeyProcess_Down()
{
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0:
		break;
	case PANELCTL_LEVEL1:
		PanelDownLevel1Handle();
		break;
	case PANELCTL_LEVEL2:
		PanelDownLevel2Handle();
		break;
	case PANELCTL_LEVEL3:
		PanelDownLevel3Handle();
		break;
	case PANELCTL_LEVEL4:
		break;
	}
}
//SHIFT按键处理模块
/**
 * SHIFT按键Level2处理模块
 */
void PanelShiftLevel2Handle(void)
{
	uint16_t Direction =RamAid->CursorDirection;
	if(Servo2Panel.Bit.P0Count == 0)
	{
		if(Servo2Panel.Cursor == 0)
		{
			Servo2Panel.Cursor = (Direction == CURSORMOVELEFT)?1:2;
		}
		else if(Servo2Panel.Cursor == 1)
		{
			Servo2Panel.Cursor = (Direction == CURSORMOVELEFT)?2:0;
		}
		else if(Servo2Panel.Cursor == 2)
		{
			Servo2Panel.Cursor = (Direction == CURSORMOVELEFT)?0:1;
		}
	}
}
/**
 * SHIFT按键Level3处理模块
 */
void PanelShiftLevel3Handle(void)
{
	uint16_t Direction = RamAid->CursorDirection;
	if(Func.Led.CurAddr.Bit8.DataHigh == 0)//P0组
	{
		if(Servo2Panel.Cursor < 5 && Servo2Panel.CursorMAX >4)
		{
			Servo2Panel.Cursor = 5;
		}
		else
		{
			Servo2Panel.Cursor = 0;
		}
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_KEY)//按键自检，SHIFT
	{
        #if KEYCHECK
		SELFKEY_FUNCCODE = 3;
        #endif
	}
	else if(Func.Key.CurAddr.All == SELFINSPECTION_LED)//LED自检，移位处理
	{
		if(SELFLED_FUNCCODE >=8 )
		{
            #if KEYCHECK
			SELFLED_FUNCCODE = 0;
            #endif
		}
		else
		{
            #if KEYCHECK
			SELFLED_FUNCCODE ++;
            #endif
		}
	}
	else if(CURSORMOVELEFT == Direction)
	{
		if(Servo2Panel.Cursor >= Servo2Panel.CursorMAX)
		{
			Servo2Panel.Cursor = 0;
		}
		else
		{
			Servo2Panel.Cursor ++;
		}
	}
	else
	{
		if(Servo2Panel.Cursor == 0)
		{
			Servo2Panel.Cursor = Servo2Panel.CursorMAX;
		}
		else
		{
			Servo2Panel.Cursor --;
		}
	}
}
void KeyProcess_Shift()
{
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0://零级菜单用于显示提示信息，
		break;
	case PANELCTL_LEVEL1:
		break;
	case PANELCTL_LEVEL2:
		PanelShiftLevel2Handle();
		break;
	case PANELCTL_LEVEL3://修改参数时的移位
		PanelShiftLevel3Handle();
		break;
	case PANELCTL_LEVEL4:
		break;
	}
}
/**
 * UP按键弹起处理模块
 */
void KeyProcess_Up_Up()
{
	switch(Servo2Panel.Level)
	{
		case PANELCTL_LEVEL0://零级菜单用于显示提示信息，
		case PANELCTL_LEVEL1:
		case PANELCTL_LEVEL2:
			break;
		case PANELCTL_LEVEL3://用于增加参数，P0组用于切换下一个功能码
			if (Func.Key.CurAddr.All == 0x0802)//UP功能
			{
				Servo2Panel.JOG_FWD_REV = JOG_STOP;
				//点动正转取消命令
			}
			break;
	}
}
/**
 * DOWN按键弹起处理模块
 */
void KeyProcess_Down_Up()
{
	switch(Servo2Panel.Level)
	{
		case PANELCTL_LEVEL0://零级菜单用于显示提示信息，
		case PANELCTL_LEVEL1:
		case PANELCTL_LEVEL2:
			break;
		case PANELCTL_LEVEL3://用于增加参数，P0组用于切换下一个功能码
			if (Func.Key.CurAddr.All == 0x0802)//UP功能
			{
				Servo2Panel.JOG_FWD_REV = JOG_STOP;
				//点动正转取消命令
			}
			break;
	}
}
/**
 * 在有弹出界面（非3级菜单字母提示界面）或伺服处于故障状态时的处理
 * 目前弹出界面是指警告界面或重新上电提示界面
 */
uint8_t PanelFaultKeyHanle(void)
{
	if(KeyInfo.CurKeyValue != 0)//任意按键按下
	{
		if(!KeyInfo.keyHold.Bit.KeyLongSet)//不是长时间按键
		{
			if(Servo2Panel.Bit.Fault)
				Servo2Panel.FaultCountDown = 60000;//计数器重新计数
			if(Servo2Panel.Bit.Warning)
				Servo2Panel.WarningCountDown = 60000;//计数器重新计数
			Func.Led.BackToPoTime = 0;
			Servo2Panel.PoOnTimeDown = 0;
			if(Servo2Panel.Status == STATUS_FAULT||Servo2Panel.Status == STATUS_WARNING)//退出警告界面
			{
				Servo2Panel.Status = STATUS_NORMAL;
				return 0;//中断执行
			}
			else if(Servo2Panel.Status == STATUS_POWERON)//退出重新上电界面
			{
				Servo2Panel.Status = STATUS_NORMAL;
				return 0;
			}
		}
		else//长时间按键
		{
			//键盘显示正常，按键Mode按下
			if(Servo2Panel.Status == STATUS_NORMAL&&KeyInfo.CurKeyValue == KEY_MODE)
			{
				if(Servo2Panel.Bit.Fault)//伺服实际有故障发生
				{
					Servo2Panel.Status =STATUS_FAULT;
					Servo2Panel.FaultCountDown = 0;
					Servo2Panel.WarningCountDown = 0;
				}
				else if(Servo2Panel.Bit.Warning)//伺服实际有警告发生
				{
					Servo2Panel.Status =STATUS_WARNING;
					Servo2Panel.FaultCountDown = 0;
					Servo2Panel.WarningCountDown = 0;
				}
			}
		}
	}
	return 1;
}
void PanelKeyProcess()
{
	//第一步：按键扫描，确定按键类型
	Panel_KeyScan();//按键扫描
	//第二步：弹出界面或故障按键处理
	if(!PanelFaultKeyHanle())
	{
		return;//中断程序执行
	}
	//第三步：正常模式下按键处理
	switch(KeyInfo.CurKeyValue)
	{
	case KEY_SET:
		 KeyProcess_Set();
		 break;
	case KEY_MODE:
		 KeyProcess_Mode();
		 break;
	case KEY_UP:
		 KeyProcess_Up();
		 break;
	case KEY_DOWN:
		 KeyProcess_Down();
		 break;
	case KEY_SHIFT:
		 KeyProcess_Shift();
		 break;
	case KEY_UP_UP:
		KeyProcess_Up_Up();
		 break;
	case KEY_DOWN_UP:
		KeyProcess_Down_Up();
		break;
	default:
		break;
	}
}
/************************************显示模块*********************************/
/**
 * 上电初始化显示模块，显示TC200
 */
void Led_InitDisp(void)
{
#if OPENTEST
	Func.Key.ControlBit.PEPassword = PASSWORDOPEN;
	Func.Key.ControlBit.PFPassword = PASSWORDOPEN;
#endif

#if SERVOTYPE == SERVO_ETHERCAT
	LedDisplay.Num[0] = '0';
	LedDisplay.Num[1] = '0';
	LedDisplay.Num[2] = '6';
	LedDisplay.Num[3] = 'C';
	LedDisplay.Num[4] = 'T';
#endif
    
#if SERVOTYPE == SERVO_PULSE
    LedDisplay.Num[0] = '0';
    LedDisplay.Num[1] = '0';
    LedDisplay.Num[2] = '5';
    LedDisplay.Num[3] = 'C';
    LedDisplay.Num[4] = 'T';
#endif
    
#if SERVOTYPE == SERVO_MODBUS
    LedDisplay.Num[0] = '0';
    LedDisplay.Num[1] = '0';
    LedDisplay.Num[2] = '7';
    LedDisplay.Num[3] = 'C';
    LedDisplay.Num[4] = 'T';
#endif

#if SERVOTYPE == SERVO_CAN
    LedDisplay.Num[0] = '0';
    LedDisplay.Num[1] = '0';
    LedDisplay.Num[2] = '8';
    LedDisplay.Num[3] = 'C';
    LedDisplay.Num[4] = 'T';
#endif

	memset(LedDisplay.Point,0,5);
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,0,5);
}
/**
 * 自学习模块，显示tune+'x'，x表示当前自学习所处的阶段
 */
void ParalearnDisp()//自学习显示
{
	LedDisplay.Num[0] = MotPaLearn.OutStep + '1';
	LedDisplay.Num[1] = 'E';
	LedDisplay.Num[2] = 'n';
	LedDisplay.Num[3] = 'u';
	LedDisplay.Num[4] = 't';
	memset(LedDisplay.Point,0,5);
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,0,5);
}
/**
 * 惯量辨识显示,显示jidt
 */
void ParaInertialearnDisp()
{
	LedDisplay.Num[0] = 't';
	LedDisplay.Num[1] = 'd';
	LedDisplay.Num[2] = 'i';
	LedDisplay.Num[3] = 'J';
	LedDisplay.Num[4] = ' ';
	memset(LedDisplay.Point,0,5);
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,0,5);

}
/**
 * 重新上电提醒，显示Po-on
 */
void ParaInertiaPoOn()
{
	LedDisplay.Num[0] = 'n';
	LedDisplay.Num[1] = 'o';
	LedDisplay.Num[2] = '-';
	LedDisplay.Num[3] = 'o';
	LedDisplay.Num[4] = 'P';
	memset(LedDisplay.Point,0,5);
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,0,5);
}
/**
 * FFT变换
 */
void Panelfft(void)
{
	LedDisplay.Num[0] = 't';
	LedDisplay.Num[1] = 'f';
	LedDisplay.Num[2] = 'd';
	LedDisplay.Num[3] = 'y';
	LedDisplay.Num[4] = 'e';
	memset(LedDisplay.Point,0,5);
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,0,5);
}
/**
 * 故障与警告显示函数
 */
void PanelFault()
{
    //故障显示方式为ER.0XX，‘X’表示故障号
    LedDisplay.Num[0] = NUMDIS_DEC2HEX[FaultP.FaultDisp&0x000F];
    LedDisplay.Num[1] = NUMDIS_DEC2HEX[FaultP.FaultDisp>>4];
    LedDisplay.Num[2] = '0';
    LedDisplay.Num[3] = 'R';
    LedDisplay.Num[4] = 'E';        
   
	memset(LedDisplay.ShowHidden,1,5);
	memset(LedDisplay.Twinkling,1,5);
	memset(LedDisplay.Point,0,5);

	LedDisplay.Point[3] = 1;
}
void PanelWarning()
{
    memset(LedDisplay.ShowHidden,1,5);//YH
    memset(LedDisplay.Twinkling,1,5);//YH
    memset(LedDisplay.Point,0,5);//YH
	//警告显示，警告显示方式为ER.EXX,XX告示警告号，其中//ER.E07 正向超程警告和ER.E08 反向超程警告做特殊处理
	if(FaultP.FaultDisp == FAULT_DEAL_BIT(POTWarn,FAULT_BIT_DISP))
	{
		LedDisplay.Num[0] = 'T';
		LedDisplay.Num[1] = 'O';
		LedDisplay.Num[2] = '-';
		LedDisplay.Num[3] = 'P';
		LedDisplay.Num[4] = ' ';
	}
	else if(FaultP.FaultDisp == FAULT_DEAL_BIT(NOTWarn,FAULT_BIT_DISP))
	{
		LedDisplay.Num[0] = 'T';
		LedDisplay.Num[1] = 'O';
		LedDisplay.Num[2] = '-';
		LedDisplay.Num[3] = 'N';
		LedDisplay.Num[4] = ' ';
	}
	else
	{
		LedDisplay.Num[0] = NUMDIS_DEC2HEX[FaultP.FaultDisp&0x000F];
		LedDisplay.Num[1] = NUMDIS_DEC2HEX[FaultP.FaultDisp>>4];
		LedDisplay.Num[2] = 'E';
		LedDisplay.Num[3] = 'R';
		LedDisplay.Num[4] = 'E';
		LedDisplay.Point[3] = 1;
	}
//	memset(LedDisplay.ShowHidden,1,5);
//	memset(LedDisplay.Twinkling,1,5);
//	memset(LedDisplay.Point,0,5);

}
/**
 * 将要写入的数据放入缓冲数组中，根据光标位置取数据
 */
void Led_DispData(uint8_t *Buf)
{
	if(Servo2Panel.Cursor<5)
	{
		memcpy(LedDisplay.Num,Buf,5);
	}
	else if((Servo2Panel.Cursor<10)&&(Servo2Panel.Cursor>=5))
	{

		memcpy(LedDisplay.Num,(Buf+5),5);
	}
}
/**
 * 第五个数码管显示伺服正反转，第4个数码管显示控制方式
 */
void ShowServoStatusP013(uint8_t*Buf)
{
	switch(Servo2Panel.ForRev)//正反转
	{
	case RUN_REV:
		Buf[4] = '_';
		break;
	case RUN_STOP:
		Buf[4] = '-';
		break;
	case RUN_FWD:
		Buf[4] = '~';
		break;
	default :
		break;
	}
    
    switch(Servo2Panel.ControlType)//控制类型
    {
        case ACTUALMODE_SPD://速度模式
            Buf[3] = '-';
            break;
        case ACTUALMODE_POS://位置模式
            Buf[3] = '_';
            break;
        case ACTUALMODE_TOR://转矩模式
            Buf[3] = '~';
            break;
        case ACTUALMODE_HOME://回零模式
            Buf[3] = 'h';
            break;
        default :
            break;
    }
    
    #if SERVOTYPE == SERVO_ETHERCAT 
    if(EcatErrCode != 0)
    {
        Buf[4] = 'R';
        Buf[3] = 'W';
    }
    #endif
}
/**
 * 正常操作状态显示数据处理，最终将其转换为ASCII码
 */
void DataDispProc(void)
{
	uint8_t Buf[10] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
	//根据键盘所处菜单等级进行处理，在正常模式下，伺服可以进入Level1，Level2，Level3，Level4
	switch(Servo2Panel.Level)
	{
		case PANELCTL_LEVEL0://零级菜单用于显示提示信息，
			break;
		case PANELCTL_LEVEL2:
			//在Level2中，显示PX-nn，X代表第几组，nn代表第几个  PrX.nn
			if(RamAid->PanelType)//在AC模式下的操作
			{
				switch(Servo2Panel.MenuType_AC)
				{
				case MENUTYPE_A:
				default:
					/*Buf[1] = Func.Led.CurAddr.Bit8.DataLow/10 + '0';
					Buf[0] = Func.Led.CurAddr.Bit8.DataLow%10 + '0';
					Buf[2] = '-';
					Buf[4] = 'P';
					Buf[3] =NUMDIS_DEC2HEX[Func.Led.CurAddr.Bit8.DataHigh];*/
			        Buf[1] = Func.Led.CurAddr.Bit8.DataLow/10 + '0';
					Buf[0] = Func.Led.CurAddr.Bit8.DataLow%10 + '0';
					Buf[2] = NUMDIS_DEC2HEX[Func.Led.CurAddr.Bit8.DataHigh];
					Buf[4] = 'P';
				    Buf[3] = 'r';

					break;
				case MENUTYPE_C:
					/*Buf[1] = NoFacFunc.FinalAddr.Bit8.DataLow/10 + '0';
					Buf[0] = NoFacFunc.FinalAddr.Bit8.DataLow%10 + '0';
					Buf[2] = '-';
					Buf[4] = 'P';
					Buf[3] = NUMDIS_DEC2HEX[NoFacFunc.FinalAddr.Bit8.DataHigh];*/
					Buf[1] = NoFacFunc.FinalAddr.Bit8.DataLow/10 + '0';
					Buf[0] = NoFacFunc.FinalAddr.Bit8.DataLow%10 + '0';
					Buf[2] = NUMDIS_DEC2HEX[NoFacFunc.FinalAddr.Bit8.DataHigh];
					Buf[4] = 'P';
					Buf[3] = 'r';
					break;
				}
			}
			else
			{
				/*Buf[0] = Func.Led.CurAddr.Bit8.DataLow%10 + '0';
				Buf[1] = Func.Led.CurAddr.Bit8.DataLow/10 + '0';
				Buf[2] = '-';
				Buf[4] = 'P';
				Buf[3] = NUMDIS_DEC2HEX[Func.Led.CurAddr.Bit8.DataHigh];*/

				Buf[0] = Func.Led.CurAddr.Bit8.DataLow%10 + '0';
				Buf[1] = Func.Led.CurAddr.Bit8.DataLow/10 + '0';
				Buf[2] = NUMDIS_DEC2HEX[Func.Led.CurAddr.Bit8.DataHigh];
				Buf[4] = 'P';
			    Buf[3] = 'r';
			}
			break;
			//注意此处没有break
		case PANELCTL_LEVEL1:
			if(RamAid->PanelType)
			{
				Buf[1] = '-';
				Buf[0] = '-';
				Buf[2] = Servo2Panel.MenuType_AC;
				Buf[3] = '-';
				Buf[4] = '-';
			}
			else
			{
				//在Level1中，键盘显示PX，X表示第几组
				Buf[4] = 'P';
				Buf[3] = 'r';
				Buf[2] = NUMDIS_DEC2HEX[Func.Led.CurAddr.Bit8.DataHigh];
			}
			break;
		case PANELCTL_LEVEL3:
			//在Level3中，显示功能码的值，功能码的值分为十进制，二进制，字母显示（P0.13，P0.18，P8.02）
			if(Func.Led.CurAddr.All == SERVOSTATUS_FUNCODE)
			{
				/**P0.13显示当前伺服的状态，P0.13=1表示准备好，显示rdy；P0.13=2表示运行，显示run；
				P0.13=3表示故障或警告，显示ER；P0.13=3表示，显示，其他状态显示P0.13的值；
				第4个数码管显示当前的控制模式_表示，-表示，~表示；第5个数码管表示正反转，_表示，-表示，~表示**/
				switch(Func.Led.Value.UnSign32)
				{
				case SERVOSTATUS_NRDY:
					Buf[0] = 'y';
					Buf[1] = 'd';
					Buf[2] = 'n';
                    
                    #if SERVOTYPE == SERVO_ETHERCAT
                    if(EcatErrCode != 0)
                    {
                        Buf[0] = NUMDIS_DEC2HEX[EcatErrCode&0x000F];
                        Buf[1] = NUMDIS_DEC2HEX[EcatErrCode>>4];
					    Buf[2] = '-';    
                    }
                    #endif

					ShowServoStatusP013(Buf);
					break;
				case SERVOSTATUS_RDY:   //准备好的状态
					Buf[0] = 'y';
					Buf[1] = 'd';
					Buf[2] = 'r';
                    
                    #if SERVOTYPE == SERVO_ETHERCAT
                    if(EcatErrCode != 0)
                    {
                        Buf[0] = NUMDIS_DEC2HEX[EcatErrCode&0x000F];
                        Buf[1] = NUMDIS_DEC2HEX[EcatErrCode>>4];
					    Buf[2] = '-';    
                    }
                    #endif

					ShowServoStatusP013(Buf);
					break;
				case SERVOSTATUS_RUN:  //运行状态
					Buf[0] = 'n';
					Buf[1] = 'u';
					Buf[2] = 'r';
                    
                    #if SERVOTYPE == SERVO_ETHERCAT
                    if(EcatErrCode != 0)
                    {
                        Buf[0] = NUMDIS_DEC2HEX[EcatErrCode&0x000F];
                        Buf[1] = NUMDIS_DEC2HEX[EcatErrCode>>4];
					    Buf[2] = '-';    
                    }
                    #endif
                    
					ShowServoStatusP013(Buf);
					break;
				case SERVOSTATUS_FAULT:  //故障状态
				case SERVOSTATUS_WORNING:
					Buf[1] = 'R';
					Buf[2] = 'E';
					ShowServoStatusP013(Buf);
					break;
				case SERVOSTATUS_HOMING:
					//显示使能和完成Hrun，poo运行
#if SERVOTYPE == SERVO_ETHERCAT
					if(!Cia402_PP.HMstatusWord.Bit.HomingAttained && !Cia402_PP.HMstatusWord.Bit.TargetReached)
					{
						//正在处理
						Buf[1] = 'o';
						Buf[2] = 'o';
						Buf[3] = 'P';
						switch(Servo2Panel.ForRev)//正反转
						{
						case RUN_REV:
							Buf[4] = '_';
							Buf[0] = '_';
							break;
						case RUN_STOP:
							Buf[4] = '-';
							Buf[0] = '-';
							break;
						case RUN_FWD:
							Buf[4] = '~';
							Buf[0] = '~';
							break;
						default :
							break;
						}
					}
					else if(!Cia402_PP.HMstatusWord.Bit.HomingAttained && Cia402_PP.HMstatusWord.Bit.TargetReached)
					{
						//仅仅使能
						Buf[0] = 'n';
						Buf[1] = 'u';
						Buf[2] = 'r';
						Buf[3] = 'h';
						switch(Servo2Panel.ForRev)//正反转
						{
						case RUN_REV:
							Buf[4] = '_';
							break;
						case RUN_STOP:
							Buf[4] = '-';
							break;
						case RUN_FWD:
							Buf[4] = '~';
							break;
						default :
							break;
						}
					}
					if(Cia402_PP.HMstatusWord.Bit.HomingAttained && Cia402_PP.HMstatusWord.Bit.TargetReached)
					{
						//仅仅使能
						Buf[0] = 'n';
						Buf[1] = 'u';
						Buf[2] = 'r';
						Buf[3] = 'h';
						switch(Servo2Panel.ForRev)//正反转
						{
						case RUN_REV:
							Buf[4] = '_';
							break;
						case RUN_STOP:
							Buf[4] = '-';
							break;
						case RUN_FWD:
							Buf[4] = '~';
							break;
						default :
							break;
						}
					}
#else
					Buf[1] = 'o';
					Buf[2] = 'o';
					Buf[3] = 'P';
					switch(Servo2Panel.ForRev)//正反转
					{
					case RUN_REV:
						Buf[4] = '_';
						Buf[0] = '_';
						break;
					case RUN_STOP:
						Buf[4] = '-';
						Buf[0] = '-';
						break;
					case RUN_FWD:
						Buf[4] = '~';
						Buf[0] = '~';
						break;
					default :
						break;
					}
#endif
					break;
				default :
					SDataToDecimalArray(Buf,Func.Led.Value.UnSign32,10);//分解成单个数字
					//DataToAsicii(Buf,10);
					break;
				}             
			}
			else if(Func.Led.CurAddr.All == FAULTFUNCODE)//P0-18显示故障代码
			{
				//P0.18显示故障号，显示方式为ER.0XX，XX表示故障号
				Buf[4] = 'E';
				Buf[3] = 'R';
				Buf[2] = '0';
				Buf[1] = NUMDIS_DEC2HEX[RamMonitor->FaultType>>4];
				Buf[0] = NUMDIS_DEC2HEX[RamMonitor->FaultType&0x000F];
			}
			else if(Func.Led.CurAddr.All == JOGFUNCODE)//点动
			{
				//P8.02显示jog，表示点动
				Buf[2] = 'J';
				Buf[1] = 'O';
				Buf[0] = 'G';
				ShowServoStatusP013(Buf);
				//保证不退回监视界面
				Func.Led.BackToPoTime = 0;
				Servo2Panel.PoOnTimeDown = 0;
				/*****************************2019.5.13修改，添加程序强制推出JOG***********************************************/
                #if SERVOTYPE == SERVO_ETHERCAT
                if(nAlStatus == STATE_OP)
                {
                    Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
                    Servo2Panel.Level = PANELCTL_LEVEL2;
                    Servo2Panel.Cursor = 0;
                }
                #endif
                /*****************************2019.5.13修改，添加程序强制推出JOG***********************************************/
			}
			else if(Func.Led.CurAddr.All == SELFINSPECTION_KEY)//工厂自检，按键自检
			{
				switch(SELFKEY_FUNCCODE)//按键
				{
				case 0://显示MODE
				default:
					Buf[4] = ' ';
					Buf[3] = 'M';
					Buf[2] = 'O';
					Buf[1] = 'D';
					Buf[0] = 'E';
					break;
				case 1://显示UP
					Buf[4] = ' ';
					Buf[3] = ' ';
					Buf[2] = ' ';
					Buf[1] = 'U';
					Buf[0] = 'P';
					break;
				case 2://显示DOWN
					Buf[4] = ' ';
					Buf[3] = 'D';
					Buf[2] = 'O';
					Buf[1] = 'W';
					Buf[0] = 'N';
					break;
				case 3://显示SHIFT
					Buf[4] = 'S';
					Buf[3] = 'H';
					Buf[2] = 'I';
					Buf[1] = 'F';
					Buf[0] = 'T';
					break;
				case 4://显示SET
					Buf[4] = ' ';
					Buf[3] = ' ';
					Buf[2] = 'S';
					Buf[1] = 'E';
					Buf[0] = 'T';
					break;
				}
			}
			else if(Func.Led.CurAddr.All == SELFINSPECTION_LED)//工厂自检，LED按键
			{
                #if KEYCHECK
				//LED自检分别在显示每一个笔画，在最后显示整个数字
				Buf[4] = FacTestLed[SELFLED_FUNCCODE];
				Buf[3] = FacTestLed[SELFLED_FUNCCODE];
				Buf[2] = FacTestLed[SELFLED_FUNCCODE];
				Buf[1] = FacTestLed[SELFLED_FUNCCODE];
				Buf[0] = FacTestLed[SELFLED_FUNCCODE];
                #endif
			}
			else
			{
				//非字母显示处理方式
				switch (Func.Led.Attr.Property.Bit.ParaType)
				{
				case ATTR_TYPE_Single://十进制
					//十进制分为有符号和无符号显示
					DecimalCussorLimit();
					if(Func.Led.Attr.Property.Bit.Sign == TYPE_UNSIGNED)//无符号的数的操作方式
					{
						//对于无符号数的处理过程交简单，将其分解然后放入数组中
						DataToDecimalArray(Buf,Func.Led.Value.UnSign32,Servo2Panel.CursorMAX);
					}
					else
					{
						//有符号数分为正数和负数两种处理方式
						if(Func.Led.Value.Sign32 < 0)
						{
							//负数的情况，将负数转为正数然后放入数组中
							DataToDecimalArray(Buf,(uint32_t)-Func.Led.Value.Sign32,Servo2Panel.CursorMAX);
							//对于负数的情况，如果长度是16位，则最高位可以显示负号，如果是32位，则不显示负号，第4个数码管显示小数点，小数点不在这处理
							if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_16)
							{
								Buf[4] = '-';
							}
						}
						else
						{
							//正数处理过程
							SDataToDecimalArray(Buf,(uint32_t)Func.Led.Value.Sign32,Servo2Panel.CursorMAX);
						}
					}
					break;
				case ATTR_TYPE_Bit://二进制
					BinaryCussorLimit();
					DataToBinaryArray(Buf,Func.Led.Value.UnSign32,Servo2Panel.CursorMAX);
					break;
				case ATTR_TYPE_Hex://十六进制
					HexCussorLimit();
					DataToHexArray(Buf,Func.Led.Value.UnSign32,Servo2Panel.CursorMAX);
					break;
				default:break;
				}
			}
			break;
		case PANELCTL_LEVEL4:
			//表示设置完参数后的操作，普通参数显示end，显示一段时间后自动返回Level2，需重新上电的参数显示po-on，显示一段时间后返回Level2
			if(Servo2Panel.Level4TimeDown == 0)
			{
				Servo2Panel.Level = PANELCTL_LEVEL2;
			}
			else
			{
				Servo2Panel.Level4TimeDown -- ;
			}
			switch(Servo2Panel.Bit.PoOn)
			{
			case 0:
				Buf[0] = '-';
				Buf[1] = 'D';
				Buf[2] = 'n';
				Buf[3] = 'E';
				Buf[4] = '-';
				break;
			case 1:
				Buf[0] = 'n';
				Buf[1] = 'o';
				Buf[2] = '-';
				Buf[3] = 'o';
				Buf[4] = 'P';
				break;
			}
			break;
		case PANELCTL_LEVEL5://工厂自检显示
			Buf[0] = 'T';
			Buf[1] = 'S';
			Buf[2] = 'E';
			Buf[3] = 'T';
			Buf[4] = 'F';
			break;
		default:break;
	}
	Led_DispData(Buf);
}
void ShowLedNum(uint8_t size)
{
	memset(LedDisplay.ShowHidden+size,0,5-size);
}
void ShowHideBit16(uint32_t Date,uint8_t Dot)
{
	if(Date < 10)//小于10，则要保证小数点前面有一个0
	{
		ShowLedNum(Dot + 1);
	}
	else if(Date < 100)//两位数，
	{
		if(Dot < 2)
		{
			ShowLedNum(2);
		}
		else
		{
			ShowLedNum(Dot + 1);
		}
	}
	else if(Date < 1000)
	{
		if(Dot < 3)
		{
			ShowLedNum(3);
		}
		else
		{
			ShowLedNum(Dot + 1);
		}
	}
	else if(Date < 10000)
	{
		if(Dot < 4)
		{
			ShowLedNum(4);
		}
		else
		{
			ShowLedNum(Dot + 1);
		}
	}
	else
	{
		ShowLedNum(5);
	}
}
void ShowHideBit32(uint32_t Date,uint8_t Dot)
{
	if(Date < 100000)//小于10，则要保证小数点前面有一个0
	{
		ShowLedNum(0);
	}
	else if(Date < 1000000)
	{
		ShowLedNum(1);
	}
	else if(Date < 10000000)
	{
		ShowLedNum(2);
	}
	else if(Date < 100000000)
	{
		ShowLedNum(3);
	}
	else if(Date < 1000000000)
	{
		ShowLedNum(4);
	}
	else
	{
		ShowLedNum(5);
	}
}
/**
 * 数码管显示高位的0进行隐藏处理
 * Data表示要显示的数字，Dot表示小数点数量
 */
void ZeroHideProc(uint32_t Date ,uint16_t Dot)
{
	if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_16 )
	{//16位数字则根据小数点位数和当前数字确定要隐藏
		ShowHideBit16(Date,Dot);
	}
	else if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_32 )//长度是32位，则要分成高
	{
		if(Servo2Panel.Cursor <= 4)
		{
			//显示的是低5位时，正常处理
			ShowHideBit16(Date,Dot);
		}
		else
		{
			//显示的是高5位时，第五个数码管要显示小数点
			ShowHideBit32(Date,Dot);
			LedDisplay.ShowHidden[4] = 1;
			if(LedDisplay.Num[4] == '0')
			{
				LedDisplay.Num[4] = ' ';
			}
		}
		}
}
/**
 * 数字显示隐藏入口函数
 */
void ShowHideProc(void)
{
	uint8_t i=0,j=0;
	memset(LedDisplay.ShowHidden,1,5);
	switch(Servo2Panel.Level)
	{
	case PANELCTL_LEVEL0:
		break;
	case PANELCTL_LEVEL1:
		//全部设置为显示，因为当数字设置为' '（空格）时,LED默认为不显示，所以在此只能显示PX。
		//此处无break；
	case PANELCTL_LEVEL2:
		//全部设置为显示，因为当数字设置为' '（空格）时,LED默认为不显示，所以在此只能显示PX-nn。
		break;
	case PANELCTL_LEVEL3:
		//根据要显示的数据类型不同进行不同的处理，分忧十进制，二进制，十六进制
		switch (Func.Led.Attr.Property.Bit.ParaType)
		{
		case ATTR_TYPE_Single:
			//十进制中，对于字母显示的，设置为全部显示，因为当数字设置为' '（空格）时,LED默认为不显示
			if(Func.Led.CurAddr.All == SERVOSTATUS_FUNCODE
			||Func.Led.CurAddr.All == FAULTFUNCODE
			||Func.Led.CurAddr.All == JOGFUNCODE
			||Func.Led.CurAddr.All == SELFINSPECTION_KEY
			||Func.Led.CurAddr.All == SELFINSPECTION_LED)
			{
			}
			else if(Func.Led.Attr.Property.Bit.Sign == TYPE_SIGNED)
			{
				//对于有符号数
				if(Func.Exchange.Value.Byte32.DataHigh & 0x8000)
				{
					//数字最高位为1表示此数字为负数，首先将负数转换为正数然后进行处理
					ZeroHideProc((uint32_t)-Func.Led.Value.UnSign32,Func.Led.Attr.Property.Bit.Dotbits);
					//16位数字负号与32位数字小数点显示处理
					if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_16)
					{
						//16位数字，第5个数码管要显示负号，因此要显示
						LedDisplay.ShowHidden[4] = 1;
					}
					else
					{
						//32位数字，第4个数码管要显示小数点，当该位原本不显示数字时，要将该位要显示数字设置为' '(空格)，然后设为为显示
						if(LedDisplay.ShowHidden[3] == 0)
						{
							LedDisplay.Num[3] = ' ';
							LedDisplay.ShowHidden[3] = 1;
						}
					}
				}
				else
				{
					//数字最高位为0表示正数，按正数显示处理
					ZeroHideProc(Func.Led.Value.UnSign32,Func.Led.Attr.Property.Bit.Dotbits);
				}
			}
			else
			{
				//无符号数处理
				ZeroHideProc(Func.Led.Value.UnSign32,Func.Led.Attr.Property.Bit.Dotbits);
			}
			break;
		case ATTR_TYPE_Bit:
			//二进制数字处理
			BinaryCussorLimit();
			j = Servo2Panel.Cursor > 4 ? Servo2Panel.CursorMAX -5 : Servo2Panel.CursorMAX;
			for(i = j+1;i < 5;i++)
			{
				LedDisplay.ShowHidden[i] = LED_SHOW_OFF;
			}
			if(Servo2Panel.Cursor>4 && LedDisplay.ShowHidden[4] == LED_SHOW_OFF)
			{
				LedDisplay.Num[4] = ' ';
			}
			break;
		case ATTR_TYPE_Hex:
			//十六进制处理
			HexCussorLimit();
			//显示与隐藏数量统计
			j = Servo2Panel.Cursor > 4 ? Servo2Panel.CursorMAX -5 : Servo2Panel.CursorMAX;
			for(i = j + 1;i < 5;i++)
			{
				LedDisplay.ShowHidden[i] = LED_SHOW_OFF;
			}
			//大于5位时最高位小数点闪烁，保证能够显示
			if(Servo2Panel.Cursor>4 && LedDisplay.ShowHidden[4] == LED_SHOW_OFF)
			{
				LedDisplay.Num[4] = ' ';
			}
			default:break;
		}
		break;
	case PANELCTL_LEVEL4://显示提示字符，可以全部显示
	case PANELCTL_LEVEL5://工厂自检显示
		break;
	default:break;
	}
}
/**
 * 闪烁处理入口函数
 */
void LedTwinkling(void)
{
	uint8_t Cursor,Level;
	Level = Servo2Panel.Level;
	Cursor = Servo2Panel.Cursor;
	memset(LedDisplay.Twinkling,0,5);
	switch(Level)
	{
	case PANELCTL_LEVEL0:
		break;
	case PANELCTL_LEVEL1:
		if(RamAid->PanelType)
		{

		}
		else
		{
			//PX，判断X是否该闪烁
			if(KeyInfo.keyDownHold == 0)
			{
				LedDisplay.Twinkling[2] = LED_TWINK_ON;//无按键按下，闪烁
			}
		}
		break;
	case PANELCTL_LEVEL2:
		//显示PX-nn，分P0组显示方式和非P0组的显示
		if(Servo2Panel.Bit.P0Count != 1)//仅仅是在监视功能下P0组监视时，不允许闪烁
		{
			if(RamAid->PanelType)
			{
				switch(Servo2Panel.MenuType_AC)
				{
				case MENUTYPE_A:
				default:
					if(KeyInfo.keyDownHold == 0)//无按键按下
					{
						LedDisplay.Twinkling[Cursor] = LED_TWINK_ON;//光标所在位置闪烁
					}
					break;
				case MENUTYPE_C:
					break;
				}
			}
			else
			{
				if(KeyInfo.keyDownHold == 0)
				{
					LedDisplay.Twinkling[Cursor] = LED_TWINK_ON;//无按键按下，闪烁
				}
			}
		}
		break;
	case PANELCTL_LEVEL3:
		//显示功能码的值
		switch(Func.Led.Attr.Property.Bit.Operation)
		{
		case ATTR_EFT_Now:
		case ATTR_EFT_PowerOn:
        case ATTR_EFT_RunOff:
			//参数属性为运行可写
			Cursor = Cursor % 5;//保证光标不大于5
			if(KeyInfo.keyDownHold == 0 &&\
			   (Func.Led.CurAddr.Bit8.DataHigh != 0 &&\
			    Func.Led.CurAddr.All != JOGFUNCODE &&\
			    Func.Led.CurAddr.All != SELFINSPECTION_KEY &&\
			    Func.Led.CurAddr.All != SELFINSPECTION_LED))
			{
				LedDisplay.Twinkling[Cursor] = LED_TWINK_ON;
			}
			if(LedDisplay.ShowHidden[Cursor] == 0|| LedDisplay.Num[Cursor] == ' ')
			{
				//在隐藏的数码管的位置，要显示'_'(下划线).
				LedDisplay.Num[Cursor] = '_';
			}
			break;
		case ATTR_OPE_ReadOnly:
			break;
		default:break;
		}
		break;
	case PANELCTL_LEVEL4:
	case PANELCTL_LEVEL5://工厂自检显示
	break;
	default:break;
	}
}
/**
 * 警告，故障，32位负数小数点闪烁
 */
void LedPointBlinking()
{
	//LedDisplay.Point[2] = LED_PIONT_ON;//杨昊 增加
	//小数点闪烁处理，处理故障警告和32位数字的负号
	if(LedDisplay.Disp_Scheduling	< 200)
	{
		//故障与警告小数点处理，故障或警告时，第一个小数点闪烁，显示的数字不可以闪烁，
		if(Func.Led.CurAddr.All != 0x000D&&(Servo2Panel.Bit.Fault||Servo2Panel.Bit.Warning))
		{
			LedDisplay.Point[0] = LED_PIONT_ON;

		}
		if((Func.Exchange.Value.Byte32.DataHigh & 0x8000) && Servo2Panel.Level == 3 &&Func.Exchange.Attr.Property.Bit.Sign == SIGN)
		{
			//32位数字，负数，第4个数码管闪烁
			if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_32)
			{
				LedDisplay.Point[3] = LED_PIONT_ON;
			}
		}
	}
	else if(LedDisplay.Disp_Scheduling	< 400)
	{
		if(Func.Led.CurAddr.All != 0x000D)
		{
			LedDisplay.Point[0] = LED_PIONT_OFF;
		}
		if(Func.Exchange.Value.Byte32.DataHigh & 0x8000)
		{
			if(Func.Led.Attr.Property.Bit.BitSize == ATTR_BIT_32)
			{
				LedDisplay.Point[3] = LED_PIONT_OFF;
			}
		}
	}
}
/**
 * 小数点处理入口函数
 */
void LedDispDot(void)
{
	uint8_t Dot =Func.Led.Attr.Property.Bit.Dotbits;
	memset(LedDisplay.Point,0,5);
	switch(Servo2Panel.Level)
	{
		case PANELCTL_LEVEL0://工厂自检显示:
			break;

		//case PANELCTL_LEVEL1:
		//case PANELCTL_LEVEL2:
			//LedPointBlinking();
			//break;//原版
		/******************************************/
		case PANELCTL_LEVEL1:
			LedPointBlinking();
			break;
		case PANELCTL_LEVEL2:
			LedPointBlinking();
			LedDisplay.Point[2] = LED_PIONT_ON;//杨昊 增加
			break;
		/**********杨昊更改*************************/
		case PANELCTL_LEVEL3:
			if(Func.Led.CurAddr.All == SERVOSTATUS_FUNCODE)//P0.13
			{
				//处理抱闸
				LedDisplay.Point[4] = SM_BRAKECTRL_OUT;
				//处理制动电阻
				if(UdcCtrl.State == UDC_BRAKE)
				{
					Servo2Panel.Bit.Brakingresistor = 1;
				}
				else
				{
					Servo2Panel.Bit.Brakingresistor = 0;
				}
				LedDisplay.Point[3] = Servo2Panel.Bit.Brakingresistor;
				if((Func.Led.AddDecValue.UnSign32 == SERVOSTATUS_FAULT)||(Func.Led.AddDecValue.UnSign32 == SERVOSTATUS_WORNING))
				{
					LedDisplay.Point[0] = 1;
					LedDisplay.Point[1] = 1;
				}

			}
			else if(Func.Led.CurAddr.All == FAULTFUNCODE)//P018
			{
				LedDisplay.Point[3] = 1;
			}
			else if(Func.Led.CurAddr.All == SELFINSPECTION_LED)
			{
				if(SELFLED_FUNCCODE >=7)
				{
					LedDisplay.Point[0] = LED_PIONT_ON;
					LedDisplay.Point[1] = LED_PIONT_ON;
					LedDisplay.Point[2] = LED_PIONT_ON;
					LedDisplay.Point[3] = LED_PIONT_ON;
					LedDisplay.Point[4] = LED_PIONT_ON;
				}
			}
			else
			{
				LedPointBlinking();
				if(Dot!=0)//根据小数点所在位置进行处理
				{
					LedDisplay.Point[Dot] = LED_PIONT_ON;
				}
				if(Servo2Panel.Cursor>4)//当长度大于4的时候第5个数码管显示
				{
					LedDisplay.Point[4] = LED_PIONT_ON;
				}
			}
			break;
		case PANELCTL_LEVEL4:
		case PANELCTL_LEVEL5:
			break;
		default:break;
	}
}
void NorMalDisp()
{
	DataDispProc();//显示数据处理
	ShowHideProc();//隐藏处理
	LedTwinkling();//闪烁处理
	LedDispDot();//小数点处理
}
void Led_Proc(void)
{
	switch(Servo2Panel.Status)
	{
		case STATUS_POWERON_INIT://上电初始化
			Led_InitDisp();
			break;
		case STATUS_SELF://自学习PE.20非0时调用
			ParalearnDisp();
			break;
		case STATUS_FAULT://故障显示模块
			PanelFault();
			break;
		case STATUS_WARNING://警告显示
			PanelWarning();
			break;
		case STATUS_INERTIALEAN://惯量辨识，P8.非0时，启动惯量辨识
			ParaInertialearnDisp();
			break;
		case STATUS_POWERON://重新上电提醒，当修改一个需重新上电的功能码时，提醒需重新上电
			ParaInertiaPoOn();
			break;
		case  STATUS_NORMAL://正常
			NorMalDisp();
			break;
		case STATUS_FFT://FFT算法
			Panelfft();
		default:break;
	}
}
uint16_t WriteSegToSPIATxBuf(uint8_t DispNum,uint8_t point,uint8_t ShowHidden,uint8_t Twinkling)
{
    uint16_t  LedSeg;
	if(Twinkling == 1)
	{
		if(LedDisplay.Disp_Scheduling	< 200)
		{
			ShowHidden = 1;
		}
		else	if(LedDisplay.Disp_Scheduling	< 400)
		{
			ShowHidden = 0;
		}
	}
    if(ShowHidden == 1)
    {

        LedSeg = AsciiToSeg(DispNum);
        if(point)
        {
        	LedSeg = LedSeg & 0x7f;
        }
        else
        {
            LedSeg = LedSeg | 0x80;
        }
	    return LedSeg;
    }
    else
    {
    	//return 0xff;//更改前

    	/********************/
    	if(LedDisplay.Sel_Led==2&&point==1)
    	    return 0x7f;
    	else
    	    return 0xff;
    	/********************///杨昊改
    }
}

void Disp_Drive(void)
{
	LedDisplay.Disp_Scheduling++;//用来控制闪烁的
	if(LedDisplay.Disp_Scheduling >=400)
	{
		LedDisplay.Disp_Scheduling = 0;
	}
	LedDisplay.DataToLEDSeg = ((1<<(LedDisplay.Led_Selectshift[4-LedDisplay.Sel_Led]))|\
			                   (WriteSegToSPIATxBuf(LedDisplay.Num[LedDisplay.Sel_Led],\
			                    LedDisplay.Point[LedDisplay.Sel_Led],\
			                    LedDisplay.ShowHidden[LedDisplay.Sel_Led],\
			                    LedDisplay.Twinkling[LedDisplay.Sel_Led])<<8));
	if(LedDisplay.Sel_Led >=4)
	{
		LedDisplay.Sel_Led = 0;
	}
	else
	{
		LedDisplay.Sel_Led++;
	}

#if (SERVO_HARDWARE == HARDWARE_AC0||SERVO_HARDWARE == HARDWARE_DC0)
	//McbspSpi_RxTxInquire(LedDisplay.DataToLEDSeg,&KeyInfo.KeyReceiveData);//数据发送与接收
    KeyInfo.KeyReceiveData = Panel_Key_RxTxInquire(LedDisplay.DataToLEDSeg);
#endif

#if SERVO_HARDWARE == HARDWARE_AC1
    uint16_t temp=0;
    temp=*ExRam_KeyCode;
    KeyInfo.KeyReceiveData=(temp<<8);

    temp=(LedDisplay.DataToLEDSeg&0xFF00)>>8;
    *ExRam_LED=(LedDisplay.DataToLEDSeg<<8)|temp;
#endif
}
/**
 * 进入故障状态时要清除一些标志位
 */
void FaultClearSignes()
{
	TYPE_STATEMACHINE *sm = &StateMachine;
	Servo2Panel.Bit.StartSelfLean = 0;//停止自学习
	RamMotor->TuningType = 0;//停止自学习
	sm->CmdIn.bit.LRN = 0;
	Servo2Panel.Bit.FFt = 0;//停止FFT
	RamAid->FFT = 0;//停止FFT
	sm->CmdIn.bit.FFt = 0;
	Servo2Panel.Bit.StartLeanInertia = 0;//停止惯量辨识
	sm->CmdIn.bit.Inertia = 0;
	RamAid->InertiaIdentificate = 0;
	Servo2Panel.JOG_FWD_REV = JOG_STOP;//停止JOG
	Servo2Panel.Bit.JOG = 0;
	KeyInfo.keyHold.All = 0;
}
void WarningClearSignes()
{
	/*KeyInfo.keyHold.All = 0;*///如果.ALL=0则在点动时出现警告，松开点动的正反转，电机继续转动，
	                        //因为KeyInfo.keyHold.Bit.key_Up清0后，不会执行按键弹起函数
	KeyInfo.keyHold.Bit.KeyLongSet = 0;//2018.10.24警告时可以继续点动，松开按键后点动停止


}
/**
 * 状态转换函数
 */
void PanelKeyInfoExchange()
{
	uint32_t Code;
	uint16_t FuncAddr;
	uint16_t *Ram_Addr;
	TYPE_Attribute *FuncAttrAddr;
	TYPE_STATEMACHINE *sm = &StateMachine;
	if(Servo2Panel.Status != STATUS_POWERON_INIT)
	{
		//P0.13处理，伺服
		Servo2Panel.ServeOn = sm->RegulFlg;
		if(Servo2Panel.ServeOn)//使能
		{
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
			if(ETHERCATSTATE_HM == CtrlMode.ECATCtrlMode)
			{
				RamMonitor->ServoState = SERVOSTATUS_HOMING;//P0.13=HOMING
			}
			else
			{
				RamMonitor->ServoState = SERVOSTATUS_RUN;//P0.13=RUN
			}
#else
			RamMonitor->ServoState = SERVOSTATUS_RUN;//P0.13=RUN
#endif
		}
		else//不使能
		{
			if(sm->Ready.all)
			{
				RamMonitor->ServoState = SERVOSTATUS_NRDY;////P0.13=NRDY
			}
			else
			{
				RamMonitor->ServoState = SERVOSTATUS_RDY;////P0.13=RDY
			}
		}
		//伺服最新状态，查询是否故障、警告、自学习

		Servo2Panel.Bit.Fault = FaultP.FaultStatus & 0x2 ? 1 : 0;
		Servo2Panel.Bit.Warning = FaultP.FaultStatus & 0x1 ?1 :0;
		if(Servo2Panel.Bit.Fault)//读到当前的状态是故障状态
		{
			//在故障状态，
			RamMonitor->ServoState = SERVOSTATUS_FAULT;//将P0.13 = FAULT
			FaultClearSignes();//清除相关标志位
			//根据键盘显示状态进行处理，1）故障未显示且故障计数器为0；2）故障未显示且故障计数器不等于0但发生了新的故障；3）
			if( Servo2Panel.Status != STATUS_FAULT)
			{
				//减计数到0或者发生新的故障
				if(Servo2Panel.FaultCountDown == 0||Servo2Panel.FaultNum != FaultP.FaultDisp)//显示非古战状态，倒计时已经到0，则跳转到故障状态
				{
					Servo2Panel.Status = STATUS_FAULT;
					Servo2Panel.Bit.Warning = 0;
					Servo2Panel.FaultNum = FaultP.FaultDisp;
					Servo2Panel.FaultCountDown = 0;
					Servo2Panel.Level4TimeDown = 0;
				}
				else
				{
					Servo2Panel.FaultCountDown --;
				}
			}
			else//当前已经是故障或者警告状态
			{
				Servo2Panel.FaultNum = FaultP.FaultDisp;
				Servo2Panel.FaultCountDown = 0;
			}
		}
		else if(Servo2Panel.Bit.Warning)//读到的当前的状态是警告状态
		{
			RamMonitor->ServoState = SERVOSTATUS_FAULT;//P0.13 = FAULT
			WarningClearSignes();
			if( Servo2Panel.Status != STATUS_WARNING)
			{
				if(Servo2Panel.WarningCountDown == 0||Servo2Panel.WarningNum != FaultP.FaultDisp)//显示非警告状态，倒计时已经到0，则跳转到警告状态
				{
					 Servo2Panel.Status = STATUS_WARNING;
					 Servo2Panel.WarningNum = FaultP.FaultDisp;
					 Servo2Panel.WarningCountDown = 0;
				}
				else
				{
					Servo2Panel.WarningCountDown --;
				}
			}
			else//当前已经是警告或故障状态状态
			{
				Servo2Panel.WarningNum = FaultP.FaultDisp;
				Servo2Panel.WarningCountDown = 0;
			}
		}
		else if(Servo2Panel.Bit.StartSelfLean == 1)//读到的当前的状态是自学习状态
		{
			if( Servo2Panel.Status != STATUS_SELF)//不是自学习状态
			{
				 Servo2Panel.Status = STATUS_SELF;
				//发出自学习命令
				sm->CmdIn.bit.LRN = 1;
			}
		}
		else if(Servo2Panel.Bit.FFt == 1)//读到的是FFT
		{
			if( Servo2Panel.Status != STATUS_FFT)//不是FFT
			{
				 Servo2Panel.Status = STATUS_FFT;
				//发出自学习命令
				sm->CmdIn.bit.FFt = 1;
			}
		}
		else if(Servo2Panel.Bit.StartLeanInertia == 1)//离线惯量辨识
		{
			if( Servo2Panel.Status != STATUS_INERTIALEAN)//不是自学习状态
			{
				Servo2Panel.Status = STATUS_INERTIALEAN;
				//发出自学习命令
				sm->CmdIn.bit.Inertia = 1;
			}
		}
		else if(Servo2Panel.Bit.IsPoOn && Servo2Panel.Status == STATUS_NORMAL)//有需要重新上电生效的
		{
			if(Servo2Panel.PoOnTimeDown >= 60000)
				Servo2Panel.Status = STATUS_POWERON;
			else
				Servo2Panel.PoOnTimeDown ++;
		}
		//键盘显示的当前状态，如果不是正常状态，返回正常状态
		else if(Servo2Panel.Status == STATUS_SELF)//自学习
		{
			Servo2Panel.Status = STATUS_NORMAL;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			RamMotor->TuningType = 0;//自学习清0
			Servo2Panel.Level = PANELCTL_LEVEL2;
			Servo2Panel.Cursor = 0;
			Servo2Panel.Bit.StartSelfLean = 0;
			sm->CmdIn.bit.LRN = 0;
		}
		else if(Servo2Panel.Status == STATUS_INERTIALEAN)//惯量辨识
		{
			Servo2Panel.Status = STATUS_NORMAL;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			RamAid->InertiaIdentificate = 0;//惯量辨识清0
			Servo2Panel.Level = PANELCTL_LEVEL2;
			Servo2Panel.Cursor = 0;
			Servo2Panel.Bit.StartLeanInertia = 0;
			sm->CmdIn.bit.Inertia = 0;
		}
		else if(Servo2Panel.Status == STATUS_FFT)//惯量辨识
		{
			Servo2Panel.Status = STATUS_NORMAL;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			RamAid->FFT = 0;//惯量辨识清0
			Servo2Panel.Level = PANELCTL_LEVEL2;
			Servo2Panel.Cursor = 0;
			Servo2Panel.Bit.FFt = 0;
			sm->CmdIn.bit.FFt = 0;
		}
		else
		{
			//正常状态
			Servo2Panel.FaultNum = 0;
			Servo2Panel.Status = STATUS_NORMAL;
			Servo2Panel.FaultNum = 0;
			Servo2Panel.WarningNum = 0;
			Servo2Panel.PoOnTimeDown = 0;
			Servo2Panel.Bit.Warning = 0;
			Servo2Panel.Bit.Fault = 0;
			Servo2Panel.FaultCountDown = 0;
			//当前显示的不是监视模式，在1分钟过后自动跳转到监控模式
			if(Func.Led.CurAddr.Bit8.DataHigh != 0 || Servo2Panel.Level != PANELCTL_LEVEL3)
			{
				if(Func.Led.BackToPoTime > BACKTOPOTIMEDEF)//计时时间到
				{
					if(Func.Led.CurAddr.Bit8.DataHigh != 0)//当前非P0组
					{
						Func.Led.PreNotP0Addr.All = Func.Led.CurAddr.All;//记录地址
						Func.Led.CurAddr.All = Func.Led.PreIsP0Addr.All;//更新地址
					}
					Func.Led.ControlBit.ReadFuncAttr = READFUNCODEATTR;//启动读属性
					Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值,并且放弃原来的修改
					Servo2Panel.Level = PANELCTL_LEVEL3;//当前显示菜单为3级
				}
				else
				{
					Func.Led.BackToPoTime ++;
				}
			}
			else
			{
				Func.Led.BackToPoTime = 0;
			}
		}
	}
	else
	{
#if SERVO_HARDWARE == HARDWARE_AC1
        //判断新的状态
        Servo2Panel.Status = STATUS_NORMAL;//键盘转换正常运行状态
        if(!Servo2Panel.Bit.LastFacTest)
        {
            Code = RamAid->MonitorDisplay;//得到P8.07的值
            Func.Exchange.CurAddr.Bit8.DataHigh=0x00;//功能码的地址赋值
            Func.Exchange.CurAddr.Bit8.DataLow=Code;
            memcpy(&Func.Exchange.Attr, & FuncCode_GetAttr(P0[Code]), sizeof(FunCodeAttribute));//填充要监视功能码属性
            Servo2Panel.Level = PANELCTL_LEVEL3;//菜单等级设为3
            Func.Key.PreNotP0Addr.All=0x100;//非监视组地址设为P1.00
            Servo2Panel.Cursor = 0;//光标位置设为0
        }
#else
        //初始化过程等待1000ms，完成后读P8.07，根据P8.07的值选择监视P0组的哪一个参数
        if(LedDisplay.TimeCont > 2500)
        {
            //判断新的状态
            Servo2Panel.Status = STATUS_NORMAL;//键盘转换正常运行状态
            if(!Servo2Panel.Bit.LastFacTest)
            {
                Code = RamAid->MonitorDisplay;//得到P8.07的值
                Func.Exchange.CurAddr.Bit8.DataHigh=0x00;//功能码的地址赋值
                Func.Exchange.CurAddr.Bit8.DataLow=Code;
                memcpy(&Func.Exchange.Attr, & FuncCode_GetAttr(P0[Code]), sizeof(FunCodeAttribute));//填充要监视功能码属性
                Servo2Panel.Level = PANELCTL_LEVEL3;//菜单等级设为3
                Func.Key.PreNotP0Addr.All=0x100;//非监视组地址设为P1.00
                Servo2Panel.Cursor = 0;//光标位置设为0
            }
        }
        else
        {
            LedDisplay.TimeCont++;
        }
#endif
	}
	//P0组监视组显示方式切换
	if(Servo2Panel.Bit.P0Count != 0)
	{
		if(Servo2Panel.P0CountDown != 0)
		{
			Servo2Panel.P0CountDown--;
		}
		else
		{
			Servo2Panel.Bit.P0Count = 0;
			if(Func.Key.CurAddr.Bit8.DataHigh>0)
			{
				Func.Key.PreNotP0Addr.All = Func.Key.CurAddr.All;
			}
			Func.Key.CurAddr.All = Func.Key.PreIsP0Addr.All;
			Func.Key.ControlBit.ReadFuncAttr = READFUNCODEATTR;
			Func.Led.ControlBit.ShowUpDownValue = SHOWREALVALUE;//当前显示实时值
			Servo2Panel.Level = PANELCTL_LEVEL3;
			Servo2Panel.Cursor = 0;
		}
	}
	//功能码值更新
	if(SHOWREALVALUE == Func.Exchange.ControlBit.ShowUpDownValue)//显示REAL值
	{
		FuncAddr = GetFuncAddr(Func.Exchange.CurAddr.All);
		FuncAttrAddr = FuncAttrStartAddr + FuncAddr ;//功能码属性地址
		Ram_Addr = RamFuncStartAddr  + FuncAddr ;//功能码ram区功能码地址
		if(Func.Exchange.Attr.Property.Bit.BitSize == ATTR_BIT_16)//单字节
		{
			Func.Exchange.Value.Byte32.DataLow =(uint16_t) *Ram_Addr;
			if(Func.Exchange.Attr.Property.Bit.Sign == TYPE_SIGNED && Func.Exchange.Value.Byte32.DataLow & 0x8000)
			{
				Func.Exchange.Value.Byte32.DataHigh =0xffff;
			}
			else
			{
				Func.Exchange.Value.Byte32.DataHigh=0;
			}
		}
		else
		{
			Func.Exchange.Value.Byte32.DataHigh =(uint16_t) *(Ram_Addr+1);
			Func.Exchange.Value.Byte32.DataLow=(uint16_t) *(Ram_Addr);
		}
		Func.Exchange.AddDecValue.UnSign32 = Func.Exchange.Value.UnSign32;

	}
	else
	{
		Func.Exchange.Value.UnSign32 = Func.Exchange.AddDecValue.UnSign32;
	}
	//读功能码属性
	if(Func.Exchange.ControlBit.ReadFuncAttr == READFUNCODEATTR)//要读属性
	{
		//memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(FunCodeAttribute)-2);  //TI
		memcpy(&Func.Exchange.Attr, FuncAttrAddr, sizeof(TYPE_Attribute));    //ARM
		if(Func.Exchange.Attr.Property.Bit.BitSize == ATTR_BIT_32)//如果是两个位
		{
			Func.Exchange.Attr.Property.Bit.Sign = (FuncAttrAddr+1)->Sign;
		}
		Func.Exchange.Attr.Max.Byte32.DataHigh = 0;
		Func.Exchange.Attr.Max.Byte32.DataLow = FuncAttrAddr->Max.U;
		Func.Exchange.Attr.Min.Byte32.DataHigh = 0;
		Func.Exchange.Attr.Min.Byte32.DataLow = FuncAttrAddr->Min.U;
		if(Func.Exchange.Attr.Property.Bit.BitSize == ATTR_BIT_32)//如果是两个位
		{
			FuncAttrAddr++;
			Func.Exchange.Attr.Max.Byte32.DataHigh = FuncAttrAddr->Max.U;
			Func.Exchange.Attr.Min.Byte32.DataHigh = FuncAttrAddr->Min.U;
		}
		else if(Func.Exchange.Attr.Property.Bit.Sign == TYPE_SIGNED)
		{
			if(Func.Exchange.Attr.Min.Byte32.DataLow & 0x8000)//
			{
				Func.Exchange.Attr.Min.Byte32.DataHigh =0xffff;
			}
			if(Func.Exchange.Attr.Max.Byte32.DataLow & 0x8000)//
			{
				Func.Exchange.Attr.Max.Byte32.DataHigh =0xffff;
			}
		}
		Func.Exchange.AddDecValue.UnSign32 = Func.Exchange.Value.UnSign32;
		Func.Exchange.ControlBit.ReadFuncAttr = 0;
	}
	//正反转判断
	if(RamMonitor->RotatingSpd > 0)//正反转判断
	{
		Servo2Panel.ForRev = RUN_FWD;
	}
	else if(RamMonitor->RotatingSpd == 0)
	{
		Servo2Panel.ForRev = RUN_STOP;
	}
	else
	{
		Servo2Panel.ForRev = RUN_REV;
	}
    
	Servo2Panel.ControlType = CtrlMode.ActualMode;
}
/**
 * 键盘处理Main函数中进行修改参数属性
 */
void Panel_mainloop(void)
{
	uint16_t Code = 0,i = 0;
	uint16_t *Group =  Func.Key.FunCodegroup.FuncGroup;
	uint16_t *Size = Func.Key.FunCodegroup.FuncGroupSize;
    
	//工厂自检，上升沿处理
//	if(RamServo->FactortTest)
//	{
//		FUNCCODEFACTEST = 1;
//	}
    
	if(FUNCCODEFACTEST && !Servo2Panel.Bit.LastFacTest)
	{
		Servo2Panel.Bit.LastFacTest = 1;
		Func.Key.CurAddr.All = SELFINSPECTION_KEY;
		Servo2Panel.Level = PANELCTL_LEVEL5;
	}
	else
	{
		Servo2Panel.Bit.LastFacTest = 0;
	}
    
	//PF组密码保护
	if(Func.Key.ControlBit.PFPassword == PASSWORDOPEN || DPI_FactoryCode2 == PFPASSWORDNUM)//PF组已经打开
	{
		Func.Key.FunCodegroup.Group_Max = FUNCODEGROUPMAX - 1;
#if SERVOTYPE == SERVO_ETHERCAT
		 Objlinkedlist = OBJLISTNUM;//决定在Twincat中是否显示PE组，PF组对应的厂家参数
         
         if(DPI_FactoryCode2 == PFPASSWORDNUM)
         {
             DPI_FactoryCode2 = 0;
             Func.Key.ControlBit.PFPassword = PASSWORDOPEN;
             Func.Key.ControlBit.PEPassword = PASSWORDOPEN;
         }
#endif
	}
	else
	{
		Func.Key.FunCodegroup.Group_Max = FUNCODEGROUPMAX - 2;
#if SERVOTYPE == SERVO_ETHERCAT
		Objlinkedlist = OBJLISTNUM - 2;
#endif
	}
    
	//PE组密码保护
	if( Func.Key.ControlBit.PEPassword == PASSWORDOPEN)//PE组已经打开
	{
		i = FUNCODEGROUPMAX -1;
		while(Group[i] != PE_FUN_GROUP)
		{
			i --;
		}
		Size[i] =PE_FUN_GROUP_SIZE;//找到当前是第几组
	}
	else
	{
		i = FUNCODEGROUPMAX -1;
		while(Group[i] != PE_FUN_GROUP)
		{
			i --;
		}
		Size[i] =1;//找到当前是第几组
	}
    
    //来自ETHERCAT的自学习指令
    if((RamMotor->TuningType > 0) && (RamMotor->TuningType < 128))
    {
        if(Servo2Panel.Bit.StartSelfLean == 0)
        {
            _iq theta;
            
            theta = _IQdiv(Etheta.ZoffsetAngle,3600);//
	        Etheta.ZPos = _IQmpyI32int(theta,DrvCoeff.PerAngleLines);//Z信号处偏移角对应的脉冲个数
            
            DrvCoeff.DrvFlg.bit.EncDir = 0;
            //DrvCoeff.DrvFlg.bit.SvpwmMode = 0;
            if(DPI_ABPhaseSequ == 1)
            {
                DrvCoeff.DrvFlg.bit.SvpwmMode = 1;
            }
            else
            {
                DrvCoeff.DrvFlg.bit.SvpwmMode = 0; 
            }
            Servo2Panel.Bit.StartSelfLean = 1;
        }
    }
    else
    {
        Servo2Panel.Bit.StartSelfLean = 0;
    }
      
	//功能码属性更新，增益调整模式
	Code = REGULATEMODE;
	if(Code != Servo2Panel.RegulateMode)
	{
		if(Code)
		{
//			ChangeFuncAttr_Op(P1[9]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(P1[22]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(P5[0]) = ATTR_OPE_ReadOnly;
//			ChangeFuncAttr_Op(P5[1]) = ATTR_OPE_ReadOnly;
//			ChangeFuncAttr_Op(P5[2]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(P5[4]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(P5[5]) = ATTR_OPE_ReadOnly;
//			ChangeFuncAttr_Op(P5[6]) = ATTR_OPE_ReadOnly;
		}
		else
		{
//			ChangeFuncAttr_Op(P1[9]) = ATTR_EFT_Now;
			ChangeFuncAttr_Op(P1[22]) = ATTR_EFT_Now;
			ChangeFuncAttr_Op(P5[0]) = ATTR_EFT_Now;
//			ChangeFuncAttr_Op(P5[1]) = ATTR_EFT_Now;
//			ChangeFuncAttr_Op(P5[2]) = ATTR_EFT_Now;
			ChangeFuncAttr_Op(P5[4]) = ATTR_EFT_Now;
			ChangeFuncAttr_Op(P5[5]) = ATTR_EFT_Now;
//			ChangeFuncAttr_Op(P5[6]) = ATTR_EFT_Now;
		}
		Servo2Panel.RegulateMode = Code;
		//Func.Exchange.ControlBit.ReadFuncAttr = READFUNCODEATTR;
	}
    
	//电机代码
	Code = MOTOR_CODE;
	if(Code != Servo2Panel.MoterCode)
	{
		if(Code)
		{
			for( i = 2; i<=MOTOR_CODE_LMTSIZE; i++)
			{
				ChangeFuncAttr_Op(PE[i]) = ATTR_OPE_ReadOnly;
			}
			ChangeFuncAttr_Op(PE[15]) = ATTR_EFT_PowerOn;//参数可改
            ChangeFuncAttr_Op(PE[18]) = ATTR_EFT_PowerOn;//参数可改
			ChangeFuncAttr_Op(PE[20]) = ATTR_EFT_PowerOn;//参数可改
            ChangeFuncAttr_Op(PE[21]) = ATTR_EFT_PowerOn;//参数可改
		}
		else
		{
			//当电机Code从有值改为0（自定义状态时），把当前电机code值写入EEPROM
			if(RamMotor->EncLine != 0){
				IIcInterfaceB(&RamMotor->Pe,(uint16_t)((&RamMotor->ACR_Id) + 1 - (&RamMotor->Pe)));
			}
			for( i = 2; i<=MOTOR_CODE_LMTSIZE; i++)
			{
				ChangeFuncAttr_Op(PE[i]) = ATTR_EFT_PowerOn;
			}
		}
		Servo2Panel.MoterCode = Code;
		//初始化一些功能码的值
	}
    
	//驱动器代码
	Code = SERVO_CODE;
	if(Code != Servo2Panel.ServoCode)
	{
		if(Code)
		{
			ChangeFuncAttr_Op(PF[4]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(PF[5]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(PF[8]) = ATTR_OPE_ReadOnly;
			ChangeFuncAttr_Op(PF[42]) = ATTR_OPE_ReadOnly;
		}
		else
		{
			ChangeFuncAttr_Op(PF[4]) = ATTR_EFT_PowerOn;
			ChangeFuncAttr_Op(PF[5]) = ATTR_EFT_PowerOn;
			ChangeFuncAttr_Op(PF[8]) = ATTR_EFT_PowerOn;
			ChangeFuncAttr_Op(PF[42]) = ATTR_EFT_PowerOn;
		}
		Servo2Panel.ServoCode = Code;
	}

}
void Panel_Init(void)
{
    //显示数据处理，将处理的结果放入缓冲区
    //Led_InitDisp();
    LedDisplay.Num[0] = '8';
    LedDisplay.Num[1] = '8';
    LedDisplay.Num[2] = '8';
    LedDisplay.Num[3] = '8';
    LedDisplay.Num[4] = '8';

    memset(LedDisplay.Point,0,5);
    memset(LedDisplay.ShowHidden,1,5);
    memset(LedDisplay.Twinkling,0,5);
    //将缓冲区内的数据显示到LED
    Disp_Drive();
}
/**
 * 函数功能：键盘1ms入口函数
 * 键盘执行时间：平均30us，最大40us
 */
void PanelCtl(void)
{
	//显示数据处理，将处理的结果放入缓冲区
	Led_Proc();
	//将缓冲区内的数据显示到LED
	Disp_Drive();
	//按键过程处理函数
	PanelKeyProcess();
	//状态查询，与其他模块数据交换
	PanelKeyInfoExchange();
	//查找非出厂值功能码，为了保证查找过程占用较少时间，每次查找最多10个
	if(NoFacFunc.FineResult == FINDUP)
	{
		FontNextNoneFacFuncode();//找到下一个非出厂值的功能码
	}
	else if(NoFacFunc.FineResult == FINTDOWN)
	{
		FinePreNoneFacFunCode();
	}

}




