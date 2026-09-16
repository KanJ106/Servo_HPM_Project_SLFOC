/*
 * SV_Global.h
 *
 *  Created on: 2013-6-13
 *      Author: lizhao
 */
//
#ifndef  SV_GLOBALVAL_H_
#define  SV_GLOBALVAL_H_
#define PARAREDY  1
#define PARAWAIT  0//
#if 0
typedef enum
{
	DRVMODE_VF = 0,		//VFģʽ
	DRVMODE_VVF = 0,	//VVFģʽ
	DRVMODE_SVC = 1,	//SVCģʽ
	DRVMODE_FVC = 2		//FVCģʽ
}TYPE_DRVMODE;

typedef enum
{
	CTRLMODE_SPEED = 0,	//�ٶȿ���ģʽ
	CTRLMODE_TORQ = 1	//ת�ؿ���ģʽ
}TYPE_SPDTORQCTRLMODE;
#endif
extern	Uint16 Glo_IIcReadAll;
#endif
