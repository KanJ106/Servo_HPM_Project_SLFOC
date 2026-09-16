/*
 * Cur_IdRef.h
 *
 *  Created on: 2015-12-30
 *      Author: w//
 */
#ifndef CUR_IDREF_H_
#define CUR_IDREF_H_
#include "IQmathLib.h" 
typedef struct 	{
             Uint16 IdRefGet;			//D�����������Դѡ����Ϊ1ʱΪ�ڲ�ǿ�Ƹ���
				_iq Ref;				//D����������
				_iq IdRefInner;			//D�������ڲ�ǿ�Ƹ���ʱ��ֵ
				_iq IdComRef;			//����ģʽ�µ�id����
				void (*init)();
				void (*rst)();
				void (*calc)();
				} CUR_IDREF;
typedef CUR_IDREF *CUR_IDREF_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CUR_IDREF object.
-----------------------------------------------------------------------------*/
#define CUR_IDREF_DEFAULTS { \
/*IdRefGet              */0, \
/*Ref                   */0, \
/*IdRefInner            */0, \
/*IdComRef              */0, \
						(void (*)(long))Cur_IdRef_init,\
						(void (*)(long))Cur_IdRef_rst,\
              			(void (*)(long))Cur_IdRef_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in Cur_IdRef.C
------------------------------------------------------------------------------*/
void Cur_IdRef_init(CUR_IDREF_handle);
void Cur_IdRef_rst(CUR_IDREF_handle);
void Cur_IdRef_calc(CUR_IDREF_handle);
extern void IdRef_Calc(void);
extern CUR_IDREF IdRef;
#endif /* CUR_IDREF_H_ */




