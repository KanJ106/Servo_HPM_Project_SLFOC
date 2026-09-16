/*
 * SV_UVWShortProtect.h
 *
 *  Created on: 2016-1-25
 *      Author: rd0217
 */

#ifndef SV_UVWSHORTPROTECT_H_
#define SV_UVWSHORTPROTECT_H_

#include "SV_UdcCtrl.h"
#include "Drive.h"
typedef struct
{
	Uint16 Iucnt;
	Uint16 Ivcnt;
	Uint16 ShortCheck;		//UVW短路检测完成标志，可以进行状态机切换，吸合软启动电阻
}UVW_ShortPRO;

extern void Fault_UVWShrotProtect(void);
extern UVW_ShortPRO UVWShortProtect;

#endif /* SV_UVWSHORTPROTECT_H_ */
