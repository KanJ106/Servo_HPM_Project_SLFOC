/*
 * SV_UVWShortProtect.c
 *
 *  Created on: 2016-1-25
 *      Author: rd0217////
 */

#include "SV_UVWShortProtect.h"
#include "Drive.h"
#define UVWShortIe   (_IQdiv(200,DrvCoeff.DrvIbase))
#define UVWPRO 0
//#define UVWPRO 1


_iq IU[20];//测试观察用变量，正式版本注释掉
_iq	IV[20];//测试观察用变量，正式版本注释掉
long In[20];
long AB;//测试观察用变量，正式版本注释掉
UVW_ShortPRO UVWShortProtect ={0,0,0};
void Fault_UVWShrotProtect(void)
{
#if 0
	Uint16 i;
	UDC_SOFTOFF();		//软启动电阻断开，保持在母线回路
//  ON_PWMABridge();	//打开逆变电路上桥
	ON_PWMBBridge();	//打开逆变电路下桥，注意不同功率段软启电阻所处桥臂不同，大功率软启电阻在上桥臂，开上桥不能开下桥，否则炸机
	DELAY_US(200);
	AB = UVWShortIe;
//  PWMOutEnable();		//PWM输出使能
//判断U、V采样电流是否满足短路时大小
    for(i=0;i<20;i++)
    {
//    	ON_PWMBBridge();
    	PWMOutEnUVDisW();		//开通U V相，关断W相
    	DELAY_US(200);//AD触发周期为125us，开通时间应大于125US
    	AdRead.Iuvcalc(&AdRead);//采样UV相电流
    	if(_IQabs(AdRead.Iu) >UVWShortIe)//判断U相电流是否大于阈值
    	{
    		 UVWShortProtect.Iucnt++;
    		 In[i] =  _IQabs(AdRead.Iu);
    	}
    	if(_IQabs(AdRead.Iv) >UVWShortIe)//判断V相电流是否大于阈值
//      if(AdRead.Iv >_IQdiv(300,DrvCoeff.DrvIbase))_IQmpyI32int(AdRead.Iv,DrvCoeff.DrvIbase)
    	 {
    	    UVWShortProtect.Ivcnt++;
    	 }
    	IV[i] = AdRead.Iv;//测试用语句
    	IU[i] = AdRead.Iu;//测试用语句
    	PWMOutDisable();//关断800us
    	DELAY_US(800);
   };

	PWMOutDisable();
	EALLOW;
#if 0
	EPwm1Regs.CMPA.half.CMPA = EPWM_PRD_DEFAULT / 2;  //默认占空比50%
	EPwm2Regs.CMPA.half.CMPA = EPWM_PRD_DEFAULT / 2;  //默认占空比50%
	EPwm3Regs.CMPA.half.CMPA = EPWM_PRD_DEFAULT / 2;  //默认占空比50%
#endif
	EPwm1Regs.CMPA.bit.CMPA = EPWM_PRD_DEFAULT >>1;  //默认占空比50%
	EPwm2Regs.CMPA.bit.CMPA = EPWM_PRD_DEFAULT >>1;  //默认占空比50%
	EPwm3Regs.CMPA.bit.CMPA = EPWM_PRD_DEFAULT >>1;  //默认占空比50%
	EDIS;

    if((UVWShortProtect.Iucnt > 3)||(UVWShortProtect.Ivcnt > 3))
    {
    					//检测到U/V相有电流，置UVW短路故障标志
    	FaultPrtt_FaultInterface(UVWPEErr);

    }
#endif
    UVWShortProtect.ShortCheck = 1;

}


