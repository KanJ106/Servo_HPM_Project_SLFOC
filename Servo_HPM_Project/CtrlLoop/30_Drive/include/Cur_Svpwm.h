/*
 * Cur_Svpwm.h
 *
 *  Created on: 2015-12-19
 *      Author: w
 *///

#ifndef CUR_SVPWM_H_
#define CUR_SVPWM_H_
#include "IQmathLib.h" 
typedef struct {
				  _iq  	Ualpha;   			//输入定子电压1
				  _iq  	Ubeta;   			//输入定子电压2
				  _iq  	Va;					//变换后的U相电压给定
				  _iq  	Vb;					//变换后的V相电压给定
				  _iq  	Vc;					//变换后的W相电压给定
				  _iq  	Vz;					//谐波注入法三次谐波零序电压
				  _iq   Udc;				//当前母线电压
                  _iq   Ialpha;
                  _iq   Ibeta;
                  _iq   Ia;
                  _iq   Slope;
                  int16 OnDelayComp;
				 Uint16 AvrSw;				//电压自动调节开关
				  _iq   MaxDuty;			//最大占空比
                  uint16_t SamWinMode;
		 	 	  void  (*init)();
		 	 	  void  (*rst)();
		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } CUR_SVPWM;

typedef CUR_SVPWM *CUR_SVPWM_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_SVPWM object.
-----------------------------------------------------------------------------*/
#define CUR_SVPWM_DEFAULTS {  \
/*Ualpha                 */0, \
/*Ubeta                  */0, \
/*Va                     */0, \
/*Vb                     */0, \
/*Vc                     */0, \
/*Vz                     */0, \
/*Udc                    */0, \
/*Ialpha                 */0, \
/*Ibeta                  */0, \
/*Ia                     */0, \
/*Slope                  */0, \
/*OnDelayComp            */0, \
/*AvrSw                  */0, \
/*MaxDuty                */0, \
/*SamWinMode              */0, \
                          (void (*)(Uint32))Cur_Svpwm_init,\
                          (void (*)(Uint32))Cur_Svpwm_rst,\
              			  (void (*)(Uint32))Cur_Svpwm_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_Svpwm.c
------------------------------------------------------------------------------*/
void Cur_Svpwm_init(CUR_SVPWM_handle);
void Cur_Svpwm_rst(CUR_SVPWM_handle);
void Cur_Svpwm_calc(CUR_SVPWM_handle);
void Svpwm_Calc(void);
extern CUR_SVPWM Svpwm;
#endif /* CUR_SVPWM_H_ */


