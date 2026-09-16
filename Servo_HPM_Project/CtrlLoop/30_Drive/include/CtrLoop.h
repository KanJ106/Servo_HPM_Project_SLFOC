/*
 * CtrLoop.h
 *
 *  Created on: 2015-12-23
 *      Author: w//
 */

#ifndef CTRLOOP_H_
#define CTRLOOP_H_
#include "userdefine.h"//

typedef struct
{
    Uint16 PosLoopCnt;				//位置环路调度计数器
    Uint16 SpdLoopCnt;				//速度环路调度计数器
    Uint16 JidtLoopCnt;				//在线惯量辨识调度计数器
    Uint16 ToqLoopCnt;				//电流环路调度计数器
    Uint16 PosLoopTimes;				//位置环路调度次数上限
    Uint16 SpdLoopTimes;				//速度环路调度次数上限
    Uint16 ToqLoopTimes;				//电流环路调度次数上限
	int32 tst1;
	int32 tst2;
	void (*init)();
	void (*rst)();
	void (*Interface)();
}CTRLOOP_TYPE;
typedef CTRLOOP_TYPE *CTRLOOP_TYPE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CTRLOOP_TYPE object.
-----------------------------------------------------------------------------*/
#define CTRLOOP_TYPE_DEFAULTS { \
		0,0,0,0,0,0,0,0,0,\
		(void (*)(long)) CtrLoop_init,\
		(void (*)(long)) CtrLoop_rst,\
		(void (*)(long)) CtrLoop_Interface}
void CtrLoop_init(CTRLOOP_TYPE_handle);
void CtrLoop_rst(CTRLOOP_TYPE_handle);
void CtrLoop_Interface(CTRLOOP_TYPE_handle);
extern void CtrLoop_CalcFirst(void);
extern void CtrLoop_CalcSecond(void);//Code in PWM ISR
extern CTRLOOP_TYPE CtrLoop;
#endif /* CTRLOOP_H_ */














