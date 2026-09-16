/******************************************************************************
**                           深圳市同川科技有限公司
**                               功能算法小组
**                               www.tc-tech.net
**
----------------------------------文件信息-------------------------------------
**文   件   名：datatype.h
**创   建   人：杨玉亮
**最后修改日期：2015-10-11
**接 口 函  数：无
**描        述：数据类型定义
**其        它：无
**函 数 清  单：无
**历        史：修改历史记录列表，每条记录包括修改日期、修改者及修改内容简述
                1.日期：
                作者：
                描述：
                2....
******************************************************************************/
//
#ifndef __DATATYPE_H__
#define __DATATYPE_H__

#include "IQmathLib.h" 


//#ifndef DSP28_DATA_TYPES
//#define DSP28_DATA_TYPES
//
//////typedef char						int8;
////typedef int							int16;//short也行
////typedef long						int32;
////typedef long long					int64;
//////typedef unsigned char				Uint16;
////typedef unsigned int				Uint16;//short也行
////typedef unsigned long				Uint32;
////typedef unsigned long long			Uint64;
////typedef float						float32;
////typedef long double					float64;
////#endif
//
//#if 0
//#ifndef _TI_STD_TYPES
//#define _TI_STD_TYPES
//
//typedef int             Int;
//typedef unsigned        Uns;
//typedef char            Char;
//typedef char            *String;
//typedef void            *Ptr;
//typedef unsigned short	Bool;
//
//typedef unsigned long   Uint32;
//typedef unsigned int    Uint16;
//typedef unsigned char   Uint16;
//
//typedef long            Int32;
//typedef int             Int16;
//typedef char            Int8;
//
//#endif
//#endif

typedef struct{
	Uint16 bit0:1;
	Uint16 bit1:1;
	Uint16 bit2:1;
	Uint16 bit3:1;

	Uint16 bit4:1;
	Uint16 bit5:1;
	Uint16 bit6:1;
	Uint16 bit7:1;

	Uint16 bit8:1;
	Uint16 bit9:1;
	Uint16 bit10:1;
	Uint16 bit11:1;

	Uint16 bit12:1;
	Uint16 bit13:1;
	Uint16 bit14:1;
	Uint16 bit15:1;
}TYPE_STRUCT_16BIT;
typedef struct{
	Uint16 bit0:1;
	Uint16 bit1:1;
	Uint16 bit2:1;
	Uint16 bit3:1;

	Uint16 bit4:1;
	Uint16 bit5:1;
	Uint16 bit6:1;
	Uint16 bit7:1;

	Uint16 bit8:1;
	Uint16 bit9:1;
	Uint16 bit10:1;
	Uint16 bit11:1;

	Uint16 bit12:1;
	Uint16 bit13:1;
	Uint16 bit14:1;
	Uint16 bit15:1;

	Uint16 bit16:1;
	Uint16 bit17:1;
	Uint16 bit18:1;
	Uint16 bit19:1;

	Uint16 bit20:1;
	Uint16 bit21:1;
	Uint16 bit22:1;
	Uint16 bit23:1;

	Uint16 bit24:1;
	Uint16 bit25:1;
	Uint16 bit26:1;
	Uint16 bit27:1;

	Uint16 bit28:1;
	Uint16 bit29:1;
	Uint16 bit30:1;
	Uint16 bit31:1;
}TYPE_STRUCT_32BIT;
typedef struct{
	Uint16 LowByte:8;
	Uint16 HighByte:8;
}TYPE_STRUCT_WORD;
typedef struct{
	Uint32 LowWord:16;
	Uint32 HighWord:16;
}TYPE_STRUCT_DWORD;
typedef struct{
	Uint32 Byte0:8;
	Uint32 Byte1:8;
	Uint32 Byte2:8;
	Uint32 Byte3:8;
}TYPE_STRUCT_DWORDBYTE;
typedef union{
	Uint16 all;
	TYPE_STRUCT_WORD Word;
}TYPE_UNION_WORD;
typedef union{
	Uint32 all;
	TYPE_STRUCT_DWORD DWord;
	TYPE_STRUCT_DWORDBYTE Byte;
}TYPE_UNION_DWORD;

typedef union{
	Uint16 all;
	TYPE_STRUCT_16BIT	bit;
	TYPE_STRUCT_WORD	byte;
}TYPE_UNION_16BIT;

typedef union{
	Uint32 all;
	TYPE_STRUCT_32BIT	bit;
	TYPE_STRUCT_DWORD	byte;
}TYPE_UNION_32BIT;





#endif


