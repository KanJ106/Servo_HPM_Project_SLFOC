/*
 * Spd_OfflineJidt.h
 *
 *  Created on: 2016-1-23
 *      Author: rd0213
 *///

#ifndef SPD_OFFLINEJIDT_H_
#define SPD_OFFLINEJIDT_H_
#include "IQmathLib.h" 
typedef struct
{
	Uint16 Step;						//��ʶ����
	int64 TorqCur1;					//��һ�׶��ܵ�ת�ص�����
	int64 TorqCur2;					//�ڶ��׶��ܵ�ת�ص�����
	int64 TorqCur3;					//�����׶��ܵ�ת�ص�����
	int64 TorqCur4;					//���Ľ׶��ܵ�ת�ص�����
	_iq   Te1;						//��һ�׶�ƽ�����
	_iq   Te2;						//�ڶ��׶�ƽ�����
	_iq   Te3;						//�����׶�ƽ�����
	_iq   Te4;						//���Ľ׶�ƽ�����
	Uint32 Lcnt1;					//��һ�׶ν����Ĵ���
	Uint32 Lcnt2;					//�ڶ��׶ν����Ĵ���
	Uint32 Lcnt3;					//�����׶ν����Ĵ���
	Uint32 Lcnt4;					//���Ľ׶ν����Ĵ���
	_iq J1;							//��תʱ�����Ĺ����
	_iq J2;							//��תʱ�����Ĺ����
	_iq J;							//ƽ�������(J1+J2)/2
	_iq Torqin;						//ע����ת�ص���
	_iq Coeff;						//ת��ϵ��
	int32 PosTotle;					//��λ��
	Uint16   Lflag;				    //ѧϰ��־
	Uint32  DelayCnt;				//��ʱ������
	void (*init)();
	void (*rst)();
	void (*calc)();	    /* */
}SPD_OFFLINEJIDT;
typedef SPD_OFFLINEJIDT *SPD_OFFLINEJIDT_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SPD_OFFLINEJIDT object.
-----------------------------------------------------------------------------*/
#define SPD_OFFLINEJIDT_DEFAULTS { \
/*Step                        */0, \
/*TorqCur1                    */0, \
/*TorqCur2                    */0, \
/*TorqCur3                    */0, \
/*TorqCur4                    */0, \
/*Te1                         */0, \
/*Te2                         */0, \
/*Te3                         */0, \
/*Te4                         */0, \
/*Lcnt1                       */0, \
/*Lcnt2                       */0, \
/*Lcnt3                       */0, \
/*Lcnt4                       */0, \
/*J1                          */0, \
/*J2                          */0, \
/*J                           */0, \
/*Torqin                      */0, \
/*Coeff                       */0, \
/*PosTotle                    */0, \
/*Lflag                       */0, \
/*DelayCnt                    */0, \
/*(*init)()                   */(void (*)(long)) OffLineJidt_init,\
/*(*rst)()                    */(void (*)(long)) OffLineJidt_rst,\
/*(*calc)()                   */(void (*)(long)) OffLineJidt_calc }
void OffLineJidt_init(SPD_OFFLINEJIDT_handle);
void OffLineJidt_rst(SPD_OFFLINEJIDT_handle);
void OffLineJidt_calc(SPD_OFFLINEJIDT_handle);
extern SPD_OFFLINEJIDT OffLineJidt;
#endif /* SPD_OFFLINEJIDT_H_ */
