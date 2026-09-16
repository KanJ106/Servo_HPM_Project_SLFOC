#ifndef	_SV_FUNCODEATTRLIT_H_
#define _SV_FUNCODEATTRLIT_H_
//
#include "Datatype.h"//

typedef struct
{
	Uint16 Ne;									//PE-06电机额定转速
	Uint16 Nmax;								//PE-07电机最大转速
    Uint16 Tmax;								//PE-07电机最大过载
	Uint16 ServoCode;							//PF-06驱动器代码
}TYPE_PreFunc;

extern void FuncLitInit(void);					//功能码限定模块初始化
extern void FuncLitProc_sch(void);				//功能码限定-调度处理

#endif
