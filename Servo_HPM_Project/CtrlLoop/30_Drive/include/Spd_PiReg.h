/*
 * Spd_PiReg.h
 *
 *  Created on: 2015-12-29
 *      Author: w
 *///

#ifndef SPD_PIREG_H_
#define SPD_PIREG_H_
#include "IQmathLib.h" 
struct FLG_BITS {
	Uint16 PiSatFlg:2;
	Uint16 KpUseFlg:1;
	Uint16 OutLimFlg:2;//0:�����ģʽת�����ƣ�1��ת��ģʽ��TQ/CST��ת������
	Uint16 Rsvd1:11;
};

union FLG {
   Uint16              all;
   struct FLG_BITS    bit;
};

typedef struct {
				  union FLG Flg;
                  _iq   Torout;
                  int16 ToroutRef;
				  _iq  	Ref;   			// Input: Reference input
				  _iq  	Fdb;   			// Input: Feedback input
				  _iq  	Err;			// Variable: Error
				  _iq  	Err1;
				  _iq  	Err2;
				  _iq  	Kp;				// Parameter: Proportional gain
				  _iq  	Up;				// Variable: Proportional output
				  _iq  	Updf;
				  _iq  	Ui;				// Variable: Integral output
				  _iq  	Ud;				// Variable: Derivative output
				  _iq  	OutPreSat; 		// Variable: Pre-saturated output
				  _iq  	OutMax;		    // Parameter: Maximum output
				  _iq  	OutMin;	    	// Parameter: Minimum output
				  _iq  	Out;   			// Output: PID output
				  _iq  	SatErr;			// Variable: Saturated difference
				  _iq  	Ki;			    // Parameter: Integral gain
				  _iq  	Kc;		     	// Parameter: Integral correction gain
				  _iq  	Kd; 		    // Parameter: Derivative gain
				  _iq  	Up1;		   	// History: Previous proportional output
				  _iq   PdffCofe;
				  void  (*init)();
				  void  (*update)();
				  void  (*rst)();
				  void  (*calc)();	  	// Pointer to calculation function
				 } SPD_PIREG;

typedef SPD_PIREG *SPD_PIREG_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SPD_PIREG object.
-----------------------------------------------------------------------------*/
#define SPD_PIREG_DEFAULTS {  \
/*Flg                    */0, \
/*Torout                 */0, \
/*Torout                 */0, \
/*Ref                    */0, \
/*Fdb                    */0, \
/*Err                    */0, \
/*Err1                   */0, \
/*Err2                   */0, \
/*Kp                     */_IQ(1.3), \
/*Up                     */0, \
/*Updf                   */0, \
/*Ui                     */0, \
/*Ud                     */0, \
/*OutPreSat              */0, \
/*OutMax                 */_IQ(1), \
/*OutMin                 */_IQ(-1), \
/*Out                    */0, \
/*SatErr                 */0, \
/*Ki                     */_IQ(0.02), \
/*Kc                     */_IQ(0.05), \
/*Kd                     */_IQ(0.0), \
/*Up1                    */0, \
/*PdffCofe               */_IQ(1.0), \
/*(*init)()              */(void (*)(Uint32))Spd_PiReg_init, \
/*(*update)()            */(void (*)(Uint32))Spd_PiReg_update, \
/*(*rst)()               */(void (*)(Uint32))Spd_PiReg_rst, \
/*(*calc)()              */(void (*)(Uint32))Spd_PiReg_calc}
/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_IqPiReg.c
------------------------------------------------------------------------------*/
void Spd_PiReg_init(SPD_PIREG_handle);
void Spd_PiReg_update(SPD_PIREG_handle);
void Spd_PiReg_rst(SPD_PIREG_handle);
void Spd_PiReg_calc(SPD_PIREG_handle);
extern void SpdPiReg_Calc(void);



typedef struct {  _iq  	Ref;   			// Input: Reference input
				  _iq  	Ref0;
				  _iq  	Err;			// Variable: Error
				  _iq  	Out;   			// Output: PID output
				  _iq  	Out0;
				  _iq   FK1;
				  _iq   FK2;
				  _iq   Rates;
				  _iq   Coeff;
				  _iq   Max;
				  _iq   Min;
				  void  (*init)();
				  void  (*update)();
				  void  (*rst)();
				  void  (*calc)();	  	// Pointer to calculation function
				 }ACCFWD;

typedef ACCFWD *ACCFWD_handle;

#define ACCFWD_DEFAULTS {   \
/*Ref                  */0, \
/*Ref0                 */0, \
/*Err                  */0, \
/*Out                  */0, \
/*Out0                 */0, \
/*FK1                  */0, \
/*FK2                  */0, \
/*Rates                */0, \
/*Coeff                */0, \
/*Max                  */_IQ(1), \
/*Min                  */_IQ(-1), \
/*(*init)()            */(void (*)(Uint32))AccFwd_init, \
/*(*update)()          */(void (*)(Uint32))AccFwd_update, \
/*(*rst)()             */(void (*)(Uint32))AccFwd_rst, \
/*(*calc)()            */(void (*)(Uint32))AccFwd_calc}
void AccFwd_init(ACCFWD_handle);
void AccFwd_update(ACCFWD_handle);
void AccFwd_rst(ACCFWD_handle);
void AccFwd_calc(ACCFWD_handle);
extern void AccFwd_Calc(void);
extern SPD_PIREG Spd_PiReg;
extern ACCFWD AccFwd;
#endif /* SPD_PIREG_H_ */
