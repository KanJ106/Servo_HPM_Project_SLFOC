#ifndef _SV_DATATYPE_H
#define _SV_DATATYPE_H
#include <stdint.h>//


typedef struct
{
    uint16_t Low:8;   //字的低位
    uint16_t High:8;  //字的高位
}TYPE_BYTE;

typedef struct
{
    uint16_t Low;   //字的高位
    uint16_t High;  //字的地位
}TYPE_UINT16;

typedef union
{
    uint16_t all;
    TYPE_BYTE   Byte;
}TYPE_WORD;

typedef union
{
	uint32_t all;
	uint16_t DWord[2];
	TYPE_UINT16  Byte;
}TYPE_DWORD;


#endif
