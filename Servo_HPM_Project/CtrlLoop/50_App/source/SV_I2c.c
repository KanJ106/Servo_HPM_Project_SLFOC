/******************************************************************************
**                           深圳市同川科技有限公司
**                               TC200伺服驱动器
**                               www.tc-tech.net
**
----------------------------------文件信息-------------------------------------
**文   件   名：IIC.c
**创   建   人：杨玉亮//
**最后修改日期：2015-7-8
**对外接口函数 		void IIC_Process(void)
** 调用驱动函数 	void I2CA_WriteData(Uint16 Addr,Uint16 *pDate,Uint16 size) 平台写函数
					void I2CA_ReadData(Uint16 Addr,Uint16 *pDate,Uint16 size)  平台读函数
**描        述：IIC存储及读取调度函数，1ms 执行任务一次，
**其        它：无
**函 数 清  单：
**历        史：修改历史记录列表，每条记录包括修改日期、修改者及修改内容简述
                1.日期：
                作者：
                描述：
                2....
******************************************************************************/
#include "userdefine.h"
#include "SV_I2c.h"
#include "SV_FuncCode.h"
#include "SV_Sci.h"
#include "string.h"
#include "SV_PanelCtl.h"
#include "cia402appl.h"
#include "coeappl.h"
#include "SV_Global.h"
#include "SV_FaultProtect.h"
#include "SV_Servocode.h"
#include "SV_FuncVar.h"
#include "DrvCoeff.h"
#include "coeappl.h"
#include "Canopenappl.h"
#if SERVO_MCU == RZ_RZT1
#include "sflash.h"
#endif
#include "Flash_Eeprom.h"
#include "s_sys_init.h"
#include "Version.h"

#if 1
IIcTaskSchedule_Type  IIcTask = IICTASK_DEFAULT;
I2CCtlVal_Type        I2CCtlVal = I2CCTLlVALDEF;
TYPE_IICFlag          IICFlag = {0};
Uint16 *PowerOffBuf[P00_FUN_GROUP_SIZE] ;

Uint16 PoweroffParaCnt =0 ;//掉电参数个数
uint16_t SnWriteFlag = 0;
/******************************************************************************
**函 数 名：void IIcVarInit(void)
**描    述：掉电参数变量初始化函数
**调    用：
**输    入：无
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
#define PowerOff_NULL	0
void IIcVarInit(void)
{
	//掉电保存的参数 时间
	PowerOffBuf[0] = PowerOff_NULL;
	PowerOffBuf[1] = PowerOff_NULL;
	PowerOffBuf[2] = PowerOff_NULL;
	PowerOffBuf[3] = PowerOff_NULL;
	//掉电保存的参数 电能
	PowerOffBuf[4] = PowerOff_NULL;
	PowerOffBuf[5] = PowerOff_NULL;
	//掉电保存的参数 plc
	PowerOffBuf[6] = PowerOff_NULL;
	PowerOffBuf[7] = PowerOff_NULL;
	
	PowerOffBuf[8] = PowerOff_NULL;
	PowerOffBuf[9] = PowerOff_NULL;
	PowerOffBuf[10] =PowerOff_NULL;
	PowerOffBuf[11] = PowerOff_NULL;
	
	PowerOffBuf[12] = PowerOff_NULL;
	PowerOffBuf[13] = PowerOff_NULL;
	PowerOffBuf[14] = PowerOff_NULL;

	PowerOffBuf[P00_FUN_GROUP_SIZE -1] = &PowerDownArg->Check;
	IICFlag.All = 0;
}	
/******************************************************************************
**函 数 名：void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**描    述：IIC读写接口函数，进行读写操作
**调    用：无
**输    入：Uint16 IIcAddr 写入到芯片的芯片绝对地址
			*Src_Rom_Addr 写数据源地址，读操作时，读到的数据存放地址
			Uint16 Num 要读写的数据个数，8位字节数
			Uint16 Mode 读操作，还是写操作 0 代表写操作，1代表读操作
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIcPort(Uint16 IIcAddr,Uint16 *Src_Addr,Uint16 Num,IIC_WRITEREADMODE Mode)
{
	I2CCtlVal.IIcAddr = IIcAddr;		//初始化IIC结构体值
	I2CCtlVal.Src_Addr=Src_Addr;        //数据的源地址
	I2CCtlVal.IIc_TR_Max = Num;         //写数据的个数
	I2CCtlVal.AddrCnt =0;            //完成写的数据个数
	switch(Mode)
	{
		case IIC_WRITE:										//iic写数据操作
			I2CCtlVal.IIcStatus = IIC_OPE_WRITE_START;      //启动开始写
			break;
		case IIC_READ:										//iic读数据操作
			I2CCtlVal.IIcStatus = IIC_OPE_READ_START;       //启动开始读
			break;
		default:break;
	}
}
/******************************************************************************
**函 数 名：void IIC_StatusSchedule(void)
**描    述：读写数据状态的调度函数，完成数据的读写，
			每1ms执行一次，查看当前的状态，判定完成情况
**调    用：GetI2cStatus()获取IIC写的状态
			void I2CA_WriteData(Uint16 Addr,Uint16 *pDate,Uint16 size) 平台写函数
			void I2CA_ReadData(Uint16 Addr,Uint16 *pDate,Uint16 size)  平台读函数
**输    入：无
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIC_StatusSchedule(void)
{
	Uint16 CurPageSize;
	Uint16 BusyWaitTime;
    
	switch(I2cBusState)//底层状态
	{
		case IIC_BUS_IDLE:                  //iic底层数据收发处于空闲状态，进行应用层的任务调度
		{
			switch(I2CCtlVal.IIcStatus)     //状态调度
			{
				case IIC_OPE_WRITE_START:	 //iic处于写开始状态
					CurPageSize =IIC_PAGE_SIZE - I2CCtlVal.IIcAddr%IIC_PAGE_SIZE;      //计算该页剩余的空间		
					if(I2CCtlVal.IIc_TR_Max <= CurPageSize)
					{						
						I2CCtlVal.IIc_TR_Size=I2CCtlVal.IIc_TR_Max;						//该页的剩余空间可以一次将数据写完成						
						I2CCtlVal.IIcStatus = IIC_OPE_WRITE_STOP;						//数据写结束
					}
					else
					{
						I2CCtlVal.IIc_TR_Size =CurPageSize;								//需要写的个数大于剩余空间					
						//I2CCtlVal.IIcStatus = IIC_OPE_WRITE_NOSTOP;	//进入写未完成状态
                        I2CCtlVal.IIcStatus = IIC_OPE_WRITE_DELAY;      //还有数据未写，下次写前先延时
					}											
					//调用底层写接口函数
					I2CA_WriteData(I2CCtlVal.IIcAddr+I2CCtlVal.AddrCnt,I2CCtlVal.Src_Addr+I2CCtlVal.AddrCnt,I2CCtlVal.IIc_TR_Size);
					break;
					
				case  IIC_OPE_READ_START: 												//读取数据					
					I2CA_ReadData(I2CCtlVal.IIcAddr,I2CCtlVal.Src_Addr,I2CCtlVal.IIc_TR_Max);					
					I2CCtlVal.IIcStatus = IIC_OPE_READ_STOP;		//iic处于接收忙状态
					break;
					
				case IIC_OPE_WRITE_NOSTOP:							//iic处于继续写数据状态					
					I2CCtlVal.AddrCnt +=I2CCtlVal.IIc_TR_Size; 		//记录下一次要写的地址					
					I2CCtlVal.IIc_TR_Max -= I2CCtlVal.IIc_TR_Size;	//记录下一次要写的数据,CurPageSize表示已经写了的数据																						//记录IIC的开始地址，当前地址，加上已经写了的地址
					if(I2CCtlVal.IIc_TR_Max <= IIC_PAGE_SIZE)
					{

						I2CCtlVal.IIc_TR_Size =I2CCtlVal.IIc_TR_Max;//是最后一帧的数据，将该页写完成
						I2CCtlVal.IIcStatus = IIC_OPE_WRITE_STOP; 	//数据写结束状态
					}
					else
					{
																	//任然需要分页写数据，这次写数据的个数为一页数据的大小
						I2CCtlVal.IIc_TR_Size = IIC_PAGE_SIZE;		//发送没有完成
                        I2CCtlVal.IIcStatus = IIC_OPE_WRITE_DELAY;      //还有数据未写，下次写前先延时
					}
																	//调用底层写接口函数
					I2CA_WriteData(I2CCtlVal.IIcAddr+I2CCtlVal.AddrCnt,I2CCtlVal.Src_Addr+I2CCtlVal.AddrCnt,I2CCtlVal.IIc_TR_Size);																						
                    break;
                case IIC_OPE_WRITE_DELAY:
                    I2CCtlVal.WritedelayCnt++;
                    if(I2CCtlVal.WritedelayCnt > 5)
                    {
                        I2CCtlVal.WritedelayCnt = 0;
                        I2CCtlVal.IIcStatus = IIC_OPE_WRITE_NOSTOP;	//进入写未完成状态
                    }
                  break;
				case IIC_OPE_WRITE_STOP: 							//最后一帧数据写成功，IIC回到空闲状态
                    I2CCtlVal.WritedelayCnt++;
                    if(I2CCtlVal.WritedelayCnt > 5)
                    {
					    I2CCtlVal.IIcStatus = IIC_OPE_IDLE;
                        I2CCtlVal.WritedelayCnt = 0;
                    }
					break;
				case IIC_OPE_READ_STOP:								//数据读完成，IIC回到空闲状态
					I2CCtlVal.IIcStatus = IIC_OPE_IDLE;
					break;
				case IIC_OPE_IDLE:				
					if(I2CCtlVal.ReadEerFlag)
					{
						FaultPrtt_FaultInterface(PramRecErr);
					}
					break;
				default :break;
			}
			I2CCtlVal.IIcBusyCnt = 0;
			break;
		}		
		case IIC_BUS_RETRY://底层处于忙状态
		{
			switch(I2CCtlVal.IIcStatus)
			 {
			 	case IIC_OPE_WRITE_STOP:							//中间数据 需要重发
				case IIC_OPE_WRITE_NOSTOP:							//最后一张帧数据需要重发
					I2CA_WriteData(I2CCtlVal.IIcAddr+I2CCtlVal.AddrCnt,I2CCtlVal.Src_Addr+I2CCtlVal.AddrCnt,I2CCtlVal.IIc_TR_Size);
					break;
				case IIC_OPE_READ_STOP:		
				
					I2CCtlVal.I2cErrFlag = 1;
					FaultPrtt_FaultInterface(PramRecErr);
				//	I2CCtlVal.IIcStatus = IIC_BUS_IDLE;
					//I2CA_ReadData(I2CCtlVal.IIcAddr,I2CCtlVal.Src_Addr,I2CCtlVal.IIc_TR_Max);
					break;
				default :break;
			 }
			break;
		}
		case IIC_BUS_NACK:                     //收到nack信号，报iic故障
			FaultPrtt_FaultInterface(PramRecErr);
			I2CCtlVal.I2cErrFlag = 1;
			I2c_RstState();
			Glo_IIcReadAll =1;
			I2cBusState = IIC_BUS_IDLE;        //总线处于空闲状态
			I2CCtlVal.IIcStatus = IIC_OPE_IDLE;//状态调度处于空闲状态
			break;

		case IIC_BUS_BUSY:						//总线忙
        {
            Uint16 RW_Flag;

			I2CCtlVal.IIcBusyCnt++;            //如果连续1秒钟处于忙状态，则报故障
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
			if((IIcTask.Handle ==TASK_READALL)||(IIcTask.Handle ==TASK_1000READ)
			||(IIcTask.Handle ==TASK_6000READ)||(IIcTask.Handle == TASK_FAULTREAD)
            ||(IIcTask.Handle == TASK_6000READFull) 
             )
#else
			if((IIcTask.Handle == TASK_READALL)||(IIcTask.Handle == TASK_FAULTREAD))
#endif
			{
				BusyWaitTime = 800;//如果是读，等待时间为300ms
                RW_Flag = 1;
			}
			else
			{
				BusyWaitTime = 15;//经过测试，写等待时间不超过5ms
                RW_Flag = 0;
			}   
            
			if(I2CCtlVal.IIcBusyCnt > 2*BusyWaitTime)
			{
				I2CCtlVal.IIcStatus = IIC_OPE_IDLE;//写状态回到空闲
				I2cBusState = IIC_BUS_IDLE;
                if(RW_Flag == 1)
                {
				    FaultPrtt_FaultInterface(PramRecErr);
                }
                else
                {
				    FaultPrtt_FaultInterface(PramWErr);
                }
				I2CCtlVal.I2cErrFlag = 1;
				Glo_IIcReadAll =1;
				I2CCtlVal.IIcBusyCnt= 0;
				I2c_RstState();
			}	
            /*									
			else if(I2CCtlVal.IIcBusyCnt == BusyWaitTime)
			{
				I2c_RstState();
				switch(I2CCtlVal.IIcStatus)
				 {
				 	case IIC_OPE_WRITE_STOP:							//中间数据 需要重发
					case IIC_OPE_WRITE_NOSTOP:							//最后一张帧数据需要重发
						I2CA_WriteData(I2CCtlVal.IIcAddr+I2CCtlVal.AddrCnt,I2CCtlVal.Src_Addr+I2CCtlVal.AddrCnt,I2CCtlVal.IIc_TR_Size);
						break;
					case IIC_OPE_READ_STOP:								//如果总线处于接收忙状态，而且需要重新读取操作
						I2CA_ReadData(I2CCtlVal.IIcAddr,I2CCtlVal.Src_Addr,I2CCtlVal.IIc_TR_Max);
						break;
					default :break;
				 }
				break;

			}
            */
            }
			break;
            case IIC_BUS_INITERR:	
            {
                static Uint16 firstFlag = 0;
                if(firstFlag == 0)
                {
                    IIcTask.ServoAllRead = 1;
                    FaultPrtt_FaultInterface(SysPramErr);
                    firstFlag = 1;
                }
            }
            break;
		default :break;
	}
}

/******************************************************************************
**函 数 名：void IIc_TaskSCIA(void)
**描    述：SCIA通信的读写操作，SCI通信写2个数据时
			需要读出进行检查判定的和写的数据是否相同，判定其IIC是否有故障
**调    用：调用读写操作接口函数
			void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：无
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/

void IIc_TaskSCIA(void)
{
//	switch(IIcTask.Status)
//	{
//		case TASK_STATUS_IDLE:					//将数据写入到IIC中
//			IIcPort(IIcTask.SCIA_Arg.IIcAddr,IIcTask.SCIA_Arg.Src_Addr,IIcTask.SCIA_Arg.size,IIC_WRITE);
//			IIcTask.Status = TASK_STATUS_WRITE;
//			break;
//		case TASK_STATUS_WRITE: 				//数据已经成功写入存储芯片
//		    if(I2CCtlVal.I2cErrFlag == 1)       //如果出现存储错误，则没有保存成功
//		    {
//		    	//数据恢复到RAM区
//		    	I2CCtlVal.I2cErrFlag =0;
//				memcpy(SCICtlValA.I2cRamFuncAddr,SCICtlValA.I2cBuf,SCICtlValA.I2cSize * 2);
//		    }
//			IIcTask.Status = TASK_STATUS_IDLE;  //scia释放iic总线控制权限
//			IICFlag.bit.Task_Scia = TASK_IDLE;
//			IIcTask.Handle = TASK_IDLE;
//			break;
//		default:
//			break;
//	}

	IIcArg_type *CurrentTask;

	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			if(IIcTask.FrontSci == IIcTask.RearSci) //功能码存储队列空
			{
				return;
			}
			else
			{
				CurrentTask = &IIcTask.TASKSCI[IIcTask.FrontSci];		    //循环队列按FIFO出队
				IIcTask.FrontSci = (IIcTask.FrontSci+1) % SCITASKMAX;	//队列头指针前移
				IIcPort(CurrentTask->IIcAddr, CurrentTask->Src_Addr, CurrentTask->size, IIC_WRITE);
				IIcTask.Status = TASK_STATUS_WRITE;
			}
			break;

		case TASK_STATUS_WRITE:
            if(I2CCtlVal.I2cErrFlag == 1)
            {
            
            }
			IIcTask.Status = TASK_STATUS_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default:break;
	}
}

void IIc_TaskSdo(void)
{
	IIcArg_type *CurrentTask;

	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			if(IIcTask.FrontSdo == IIcTask.RearSdo) //功能码存储队列空
			{
				return;
			}
			else
			{
				CurrentTask = &IIcTask.TASKSDO[IIcTask.FrontSdo];		    //循环队列按FIFO出队
				IIcTask.FrontSdo = (IIcTask.FrontSdo+1) % SCITASKMAX;	    //队列头指针前移
				IIcPort(CurrentTask->IIcAddr, CurrentTask->Src_Addr, CurrentTask->size, IIC_WRITE);
				IIcTask.Status = TASK_STATUS_WRITE;
			}
			break;

		case TASK_STATUS_WRITE:
            if(I2CCtlVal.I2cErrFlag == 1)
            {
            
            }
			IIcTask.Status = TASK_STATUS_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default:break;
	}
}

/******************************************************************************
**函 数 名：void IIc_TaskPanelSave(void)
**描    述：键盘参数存储功能函数
			存储修改的功能码
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：无
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskPanelSave(void)
{
	switch(IIcTask.Status)
	{
	case TASK_STATUS_IDLE:
		IIcPort(IIcTask.PANEL_Arg.IIcAddr,IIcTask.PANEL_Arg.Src_Addr,IIcTask.PANEL_Arg.size,IIC_WRITE);
		IIcTask.Status =TASK_STATUS_WRITE;
		break;
	case TASK_STATUS_WRITE:
		if(I2CCtlVal.I2cErrFlag == 1)//如果出现存储错误，则没有保存成功
		{
			/*
			//数据恢复到RAM区
			I2CCtlVal.I2cErrFlag =0;
			memcpy(SCICtlValA.I2cRamFuncAddr,SCICtlValA.I2cBuf,SCICtlValA.I2cSize);
			memcpy(SCICtlValA.I2cRomFuncAddr,SCICtlValA.I2cBuf,SCICtlValA.I2cSize);
			*/
		}
		IIcTask.Status = TASK_STATUS_IDLE;
		IIcTask.Handle = TASK_IDLE;
		break;
	default:break;
	}
}
/******************************************************************************
**函 数 名：void IIc_TaskReadAll(void)
**描    述：上电的时候需要全读
			将所有参数都读出
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：无
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskReadAll(void)
{	
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	//上电全读功能码和掉电参数

			IIcPort(IIC_INIT_ADDR,PwoerSaveStartAddr,FUNC_SIZE + P00_FUN_GROUP_SIZE,IIC_READ);//读iic接口函数
			IIcTask.Status =TASK_STATUS_READ;
			break;
		case TASK_STATUS_READ:
			if(I2CCtlVal.I2cErrFlag == 1)//如果读数据时I2C故障
			{
				I2CCtlVal.ReadEerFlag =1;//置全读参数错误故障标志
				//I2c_RstState();
			}
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
			else
			{
				IICFlag.bit.TASK_1000Rd = TASK_NEED;//读功能码完成启动读取1000系列对象字典
			}
#else
            else
            {
                IICFlag.bit.Task_FaultRead = TASK_NEED;
            }
#endif
			IICFlag.bit.Task_PURead= TASK_IDLE;      //读参数没有错误，启动
			IIcTask.Status = TASK_STATUS_IDLE;       //释放iic权限		
			IIcTask.Handle = TASK_IDLE;			
			break;
		default :break;
	}
}
/******************************************************************************
**函 数 名：IIc_Task402_1000UpRd(void)
**描    述：上电读取402协议1000系列对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
void IIc_Task402_1000UpRd(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			IIcPort(IIC_0x1000_STATR,CUMOBJEEPROM_STATR, CUMOBJEEPROM_SIZE,IIC_READ);//读iic接口函数
            IIcTask.Status =TASK_STATUS_READ;
			break;
		case TASK_STATUS_READ:
			if(I2CCtlVal.I2cErrFlag == 1)//如果读数据I2C错误产生
			{
				I2CCtlVal.ReadEerFlag =1;//置读参数故障标志
			}
			else
			{
				//初始化软件版本 
                snprintf(CumObj.acSoftwareversion,sizeof(CumObj.acSoftwareversion), "%d", PRODUCTCODE);
                snprintf(CumObj.acHardwareversion,sizeof(CumObj.acHardwareversion), "%d", HARDWARENUM);
                       
                #if _FOE_LDRPRM_BANK0_
                CumObj.sIdentity.u32Serialnumber = 0;
                #endif
            
                #if _FOE_LDRPRM_BANK1_
                CumObj.sIdentity.u32Serialnumber = 1;
                #endif
                
                if(RamPD->FullCloseMode == 0)
                {
				    IICFlag.bit.TASK_6000Rd = TASK_NEED;//读1000完成读取6000
                }
                else
                {
                    IICFlag.bit.TASK_6000RdFull = TASK_NEED;//读1000完成读取6000
                }
			}
			IICFlag.bit.TASK_1000Rd = TASK_IDLE;//读参数没有错误，启动
			IIcTask.Status = TASK_STATUS_IDLE;                   //释放iic权限
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}

}
#endif

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
/******************************************************************************
**函 数 名：IIc_Task402_6000UpRd(void)
**描    述：上电读取402协议6000系列对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_6000UpRd(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	//上电全读功能码，包括掉电参数

			IIcPort(IIC_0x6000_STATR,RamCIA402Obj_StartAddr,RAMCIA402OBJ_SIZE,IIC_READ);//读iic接口函数
			IIcTask.Status =TASK_STATUS_READ;
			break;
		case TASK_STATUS_READ:
			if(I2CCtlVal.I2cErrFlag == 1)//如果读数据I2C错误产生
			{
				I2CCtlVal.ReadEerFlag =1;//置读参数故障标志
			}
			else
			{
			    IICFlag.bit.Task_FaultRead = TASK_NEED;
			}
            
			IICFlag.bit.TASK_6000Rd = TASK_IDLE;//读参数没有错误，初始化完毕
			IIcTask.Status = TASK_STATUS_IDLE;                   //释放iic权限
			IIcTask.Handle = TASK_IDLE;
           
//			IICFlag.bit.Task_PURead= TASK_IDLE;
			break;
		default :break;
	}
}

/******************************************************************************
**函 数 名：IIc_Task402_6000UpRdFull(void)
**描    述：上电读取402协议6000系列全闭环对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_6000UpRdFull(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	//上电全读功能码，包括掉电参数
			IIcPort(IIC_ECATFULL_ADDR,RamCIA402Obj_StartAddr,RAMCIA402OBJ_SIZE,IIC_READ);//读iic接口函数
			IIcTask.Status =TASK_STATUS_READ;
			break;
		case TASK_STATUS_READ:
			if(I2CCtlVal.I2cErrFlag == 1)//如果读数据I2C错误产生
			{
				I2CCtlVal.ReadEerFlag =1;//置读参数故障标志
			}
			else
			{
			    IICFlag.bit.Task_FaultRead = TASK_NEED;
			}
            
			IICFlag.bit.TASK_6000RdFull = TASK_IDLE;             //读参数没有错误，初始化完毕
			IIcTask.Status = TASK_STATUS_IDLE;               //释放iic权限
			IIcTask.Handle = TASK_IDLE;
           
//			IICFlag.bit.Task_PURead= TASK_IDLE;
			break;
		default :break;
	}
}
#endif

/******************************************************************************
**函 数 名：void IIc_TaskFaultUpRd(void)
**描    述：上电读取保存的故障
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskFaultUpRd(void)
{
	Uint16 IIcAddr ;
	
	switch(IIcTask.Status)
	{	
		case TASK_STATUS_IDLE:	
			IIcAddr = IIC_FUNC_ADDR + FUNC_SIZE; //获取故障信息存储地址
			IIcPort(IIcAddr,(Uint16 *)&FaultInfo,sizeof(TYPE_FAULTINFO)/2,IIC_READ);//读iic接口函数
			IIcTask.Status =TASK_STATUS_READ;
			break;
		case TASK_STATUS_READ:
			IIcTask.Status = TASK_STATUS_IDLE;
			IICFlag.bit.Task_FaultRead= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;//释放总线权限
			//IICFlag.bit.Task_PURead= TASK_IDLE;//上电读取功能码完毕
            
            IIcTask.ServoAllRead = 0;
            
			break;
		default :break;
	}
}

/******************************************************************************
**函 数 名：void IIc_TaskRestorFactory(void)
**描      述：恢复出厂值函数，包括PE，PF(第一次上电)
**调      用：void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输      入：
**输      出：无
**返      回：无
**其      它：无
**日      期：2013-7-8
*******************************************************************************/
void IIc_TaskRestorFactory(void)
{
    Uint16 HwNum;
    
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
            HwNum = RamMonitor->HardWareNum;
            
			memcpy(FactoryStartAddr,FactoryFuncStartAddr,FACTORY_SIZE * 2);
            //从FLASH中加载程序版本号，和EEPROM版本号
            RamMonitor->CPUASer1    = tbl_identifyRAM[0]; 
            //RamMonitor->EEASer1     = tbl_identifyRAM[1];
            RamServo->SoftVersion   = tbl_identifyRAM[1];
            RamMonitor->HardWareNum = HwNum;
            
			IIcPort(IIC_FUNC_ADDR,FactoryStartAddr,FACTORY_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE: 											//退出写状态
			I2CCtlVal.I2cWrAllFunc =1;
			IIcTask.Status = TASK_STATUS_IDLE;                              //释放总线
			IICFlag.bit.Task_FacWrtie= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
            
            #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
            if(IIcTask.ServoInit)
            {
                IICFlag.bit.Task_1000WrFac = TASK_NEED;
            }
            #else
            if(IIcTask.ServoInit == 1)
            {
                IICFlag.bit.Task_FaultSave1 = TASK_NEED;
            }
            #endif
            
			break;
		default :break;
	}
}

/******************************************************************************
**函 数 名：void IIc_TaskUpPowerFactory(void)
**描      述：恢复出厂值函数，包括PE，PF部分参数
**调      用：void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输      入：
**输      出：无
**返      回：无
**其      它：无
**日      期：2013-7-8
*******************************************************************************/
void IIc_TaskResetFactory(void)
{
    Uint16 HwNum;
    
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
            HwNum = RamMonitor->HardWareNum;
            
			memcpy(FactoryStartAddr,FactoryFuncStartAddr,RESET_SIZE * 2);
            //从FLASH中加载程序版本号，和EEPROM版本号
            RamMonitor->CPUASer1    = tbl_identifyRAM[0]; 
            //RamMonitor->EEASer1     = tbl_identifyRAM[1];
            RamServo->SoftVersion   = tbl_identifyRAM[1];
            RamMonitor->HardWareNum = HwNum;
            
			IIcPort(IIC_FUNC_ADDR,FactoryStartAddr,RESET_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE: 											//退出写状态
			I2CCtlVal.I2cWrAllFunc = 1;
			IIcTask.Status = TASK_STATUS_IDLE;                              //释放总线
			IICFlag.bit.Task_Reset= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
            
            #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
            if(IIcTask.ServoInit)
            {
                IICFlag.bit.Task_1000WrFac = TASK_NEED;
            }
            #else
            if(IIcTask.ServoInit == 1)
            {
                IICFlag.bit.Task_FaultSave1 = TASK_NEED;
            }
            #endif
            
			break;
		default :break;
	}
}

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
/******************************************************************************
**函 数 名：IIc_Task402_1000UpWrFac(void)
**描    述：存储402协议1000系列对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_1000UpWrFac(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //写1000对象字典默认值
			IIcPort(IIC_0x1000_STATR,FanCumObjEEPROM_StartAddr,FANCUMOBJEEPROM_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
			memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
			
            //初始化软件版本 
            snprintf(CumObj.acSoftwareversion,sizeof(CumObj.acSoftwareversion), "%d", PRODUCTCODE);
            snprintf(CumObj.acHardwareversion,sizeof(CumObj.acHardwareversion), "%d", HARDWARENUM);
            
            #if _FOE_LDRPRM_BANK0_
            CumObj.sIdentity.u32Serialnumber = 0;
            #endif
            
            #if _FOE_LDRPRM_BANK1_
            CumObj.sIdentity.u32Serialnumber = 1;
            #endif
            
			IICFlag.bit.Task_1000WrFac = TASK_IDLE;
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限
			IIcTask.Handle = TASK_IDLE;
			if(IIcTask.ServoInit)//在初始化的时候要进行该操作
			{
				IICFlag.bit.Task_6000WrFac = TASK_NEED;		//启动写6000对象字典默认值
			}
			break;
		default :break;
	}
}
#endif

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
/******************************************************************************
**函 数 名：IIc_Task402_6000UpWrFac(void)
**描    述：存储402协议6000系列对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_6000UpWrFac(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //写6000对象字典默认值
            if(RamPD->FullCloseMode == 0)
            {
			    memcpy(RamCIA402Obj_StartAddr,FanCIA402Obj_StartAddr,FANCIA402OBJ_SIZE*2);	
            }//将6000对象字典默认值写入ram
            Cia402_FirstPwoerUp_Init_M();//2025.09.06
			IIcPort(IIC_0x6000_STATR,RamCIA402Obj_StartAddr,FANCIA402OBJ_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
       //     if(RamPD->FullCloseMode == 0)
       //     {
			    //memcpy(RamCIA402Obj_StartAddr,FanCIA402Obj_StartAddr,FANCIA402OBJ_SIZE*2);	
       //     }//将6000对象字典默认值写入ram
			IICFlag.bit.Task_6000WrFac = TASK_IDLE;
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限初始化完毕
			IIcTask.Handle = TASK_IDLE;
      
            IICFlag.bit.Task_6000WrFacFull = TASK_NEED;
			break;
		default :break;
	}
}
/******************************************************************************
**函 数 名：IIc_Task402_6000UpWrFacFull(void)
**描    述：存储402协议6000系列全闭环对象字典
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_6000UpWrFacFull(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //写6000对象字典默认值
            if(RamPD->FullCloseMode > 0)
            {
			    memcpy(RamCIA402Obj_StartAddr,FanCIA402Obj_StartAddrFull,FANCIA402OBJ_SIZE*2);
            }//将6000对象字典默认值写入ram
            Cia402_FirstPwoerUp_Init_O();//2025.09.06
			IIcPort(IIC_ECATFULL_ADDR,RamCIA402Obj_StartAddr,FANCIA402OBJ_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
       //     if(RamPD->FullCloseMode > 0)
       //     {
			    //memcpy(RamCIA402Obj_StartAddr,FanCIA402Obj_StartAddrFull,FANCIA402OBJ_SIZE*2);
       //     }//将6000对象字典默认值写入ram
			IICFlag.bit.Task_6000WrFacFull = TASK_IDLE;
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限初始化完毕
			IIcTask.Handle = TASK_IDLE;
           
            if(IIcTask.ServoInit == 1)
            {
                IICFlag.bit.Task_FaultSave1 = TASK_NEED;
            }
			break;
		default :break;
	}
}
#endif

/******************************************************************************
**函 数 名：void IIc_TaskFaultWr(void)
**描    述：上电第一次初始化故障值和故障发生时共用，
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/	
void IIc_TaskFaultWr(void)
{
	Uint16 IIcAddr ;
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	
			//故障1的已经在产生故障时赋值了
			IIcAddr = IIC_FUNC_ADDR + FUNC_SIZE; //获取故障信息存储地址
			//memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));//在PowerOnReadFunc已经置零
            IIcPort(IIcAddr,(Uint16 *)&FaultInfo,sizeof(TYPE_FAULTINFO)/sizeof(Uint16),IIC_WRITE);
			IIcTask.Status = TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:			                 //故障写成功，释放总线
			IIcTask.Status = TASK_STATUS_IDLE;
			IICFlag.bit.Task_FaultSave1= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
			FaultP.EepFlg = 0;							 //清除故障写标志
            
            IIcTask.ServoInit = 0;         
            
			break;
		default :break;
	}
}

#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
/******************************************************************************
**函 数 名：IIc_Task402_1000UpWr(void)
**描    述：存储402协议1000 RAM值保存EEPROM
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_1000Save(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //写1000对象字典
			IIcPort(IIC_0x1000_STATR,CUMOBJEEPROM_STATR,CUMOBJEEPROM_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
			IICFlag.bit.TASK_1000Wr = TASK_IDLE;
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}

}
#endif
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
/******************************************************************************
**函 数 名：IIc_Task402_6000UpWr(void)
**描    述：//0x6000 RAM值保存EEPROM
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Task402_6000Save(void)
{
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //写6000对象字典
            if(RamPD->FullCloseMode == 0)
            {
			    IIcPort(IIC_0x6000_STATR,RamCIA402Obj_StartAddr,RAMCIA402OBJ_SIZE,IIC_WRITE);
            }
            else
            {
                IIcPort(IIC_ECATFULL_ADDR,RamCIA402Obj_StartAddr,RAMCIA402OBJ_SIZE,IIC_WRITE);
            }
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
			IICFlag.bit.TASK_6000Wr = TASK_IDLE;
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限初始化完毕
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}
}
#endif
/******************************************************************************
**函 数 名：void IIc_TaskRestorP1_PD(void)
**描    述：恢复P1 ~ PD
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskRestorP1_PD(void)
{
    Uint16 HwNum;
    
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:	              //恢复默认值
            HwNum = RamMonitor->HardWareNum;
			memcpy(DefaultStartAddr,FactoryFuncStartAddr,DEFAULT_SIZE * 2);
            RamMonitor->HardWareNum = HwNum;
            
			IIcPort(IIC_FUNC_ADDR,DefaultStartAddr,DEFAULT_SIZE,IIC_WRITE);
            
			//TODO:重新上电参数和静止更新参数需更新
			ServoparaInit_Adjust();	//伺服code相关参数更新
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:
			IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限
			IICFlag.bit.Task_DefWrite= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}
}

void IIc_TaskP1_PD_FuncSave(void)
{
	switch(IIcTask.Status)
	{
	case TASK_STATUS_IDLE:
		IIcPort(IIC_FUNC_ADDR,ECATFuncCodeStartAddr,ECATFUNCCODE_SIZE,IIC_WRITE);
		IIcTask.Status =TASK_STATUS_WRITE;
		break;
	case TASK_STATUS_WRITE:
		IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限
		IICFlag.bit.Task_P1_PDFuncSave= TASK_IDLE;
		IIcTask.Handle = TASK_IDLE;
		break;
	}
}

void IIc_TaskPE_PF_FuncSave(void)
{
	switch(IIcTask.Status)
	{
	case TASK_STATUS_IDLE:
		IIcPort((IIC_FUNC_ADDR + ECATFUNCCODE_SIZE),RamFuncCode.PE,(PE_FUN_GROUP_SIZE + PF_FUN_GROUP_SIZE),IIC_WRITE);
		IIcTask.Status =TASK_STATUS_WRITE;
		break;
	case TASK_STATUS_WRITE:
		IIcTask.Status = TASK_STATUS_IDLE;//释放总线权限
		IICFlag.bit.Task_PE_PFFuncSave= TASK_IDLE;
		IIcTask.Handle = TASK_IDLE;
		break;
	}
}
/******************************************************************************
**函 数 名：void IIc_TaskPowerDownWr(void)
**描    述：写入掉电参数
**调    用：
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskPowerDownWr(void)
{
	Uint16 i;
	
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			for(i = 0;i < P00_FUN_GROUP_SIZE - 1; i++)
			{
				if(PowerOffBuf[i] == PowerOff_NULL)
				{
					continue;
				}
				*(PwoerSaveStartAddr + i) = *(PowerOffBuf[i]);
			}
			PowerDownArg->Check = IIC_POWERDOWN_CHECK;
			IIcPort(IIC_POWEROFF_ADDR,PwoerSaveStartAddr,P00_FUN_GROUP_SIZE,IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;
			break;
		case TASK_STATUS_WRITE:								   //存储成功，释放总线
			IIcTask.Status = TASK_STATUS_IDLE;
			IICFlag.bit.Task_PDSave= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}
}


/**
 * 清除历史故障
 */
void IIc_TaskFaultClr(void)
{
	Uint16 IIcAddr ;
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			IIcAddr = IIC_FUNC_ADDR + FUNC_SIZE; //获取故障信息存储地址
			memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));//故障代码写函数
			IIcPort(IIcAddr,(Uint16 *)&FaultInfo,sizeof(TYPE_FAULTINFO)/sizeof(Uint16),IIC_WRITE);
			IIcTask.Status =TASK_STATUS_WRITE;           //切换状态
			break;
		case TASK_STATUS_WRITE:
			//RamServo->Clear &= 0xfffe;      //将工厂参数的清除历史故障的rom,ram区清0
			//IIcAddr = IIC_FUNC_ADDR + (&RamServo->Clear -RamFuncStartAddr);
			//IIcPort(IIcAddr,&RamServo->Clear,1,IIC_WRITE);//写到i2c中
			//IIcTask.Status =TASK_STATUS_END;
            
            IIcTask.Status = TASK_STATUS_IDLE;          //释放i2c资源
			IICFlag.bit.Task_FaultClr = TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default :break;
	}
}
/**
 * 清除运行时间
 */
void IIc_TaskRunTimeClr(void)
{
	Uint16 IIcAddr ;
	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			RamMonitor->T_SysRun = 0;
			IIcAddr = IIC_FUNC_ADDR + ((Uint16*)&RamMonitor->T_SysRun -RamFuncStartAddr);
			IIcPort(IIcAddr,(Uint16*)&RamMonitor->T_SysRun,2,IIC_WRITE);//写到i2c中
			IIcTask.Status =TASK_STATUS_WRITE;           //切换状态
			break;
		case TASK_STATUS_WRITE:
//			RamServo->Clear &= 0xfffd;            //将工厂参数的清除历史故障的rom,ram区清0
//			IIcAddr = IIC_FUNC_ADDR + (&RamServo->Clear -RamFuncStartAddr);
//			IIcPort(IIcAddr,&RamServo->Clear,1,IIC_WRITE);//写到i2c中
//			IIcTask.Status =TASK_STATUS_END;
          
          	IIcTask.Status = TASK_STATUS_IDLE;          //释放i2c资源
			IICFlag.bit.Task_FaultClr= TASK_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
//		case TASK_STATUS_END:
//			IIcTask.Status = TASK_STATUS_IDLE;          //释放i2c资源
//			IICFlag.bit.Task_FaultClr= TASK_IDLE;
//			IIcTask.Handle = TASK_IDLE;
//			break;
		default :break;
	}
}
/******************************************************************************
**函 数 名：void IIc_TaskFuncSave(void)
**描    述：软件内部少量存储的功能码
** 调用  void IIcPort(Uint16 IIcAddr,Uint16 *Src_Rom_Addr,Uint16 Num,Uint16 Mode)
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskFuncSave(void)
{
	IIcArg_type *CurrentTask;

	switch(IIcTask.Status)
	{
		case TASK_STATUS_IDLE:
			if(IIcTask.Front == IIcTask.Rear) //功能码存储队列空
			{
				return;
			}
			else
			{
				CurrentTask = &IIcTask.TASK[IIcTask.Front];		//循环队列按FIFO出队
				IIcTask.Front = (IIcTask.Front+1) % PENDINGTASKMAX;	//队列头指针前移
				IIcPort(CurrentTask->IIcAddr, CurrentTask->Src_Addr, CurrentTask->size, IIC_WRITE);
				IIcTask.Status = TASK_STATUS_WRITE;
			}
			break;

		case TASK_STATUS_WRITE:
			IIcTask.Status = TASK_STATUS_IDLE;
			IIcTask.Handle = TASK_IDLE;
			break;
		default:break;
	}
}

/******************************************************************************
**函 数 名：void IIc_TaskSel(void)
**描    述：根据标志位切换任务
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/

void IIc_TaskSel(void)
{
	if(IICFlag.All !=0)
	{
		if(IICFlag.Dword.LowWord != 0)
		{
			TYPE_UNION_WORD Word;
			Word.all = IICFlag.Dword.LowWord;

			if(Word.Word.LowByte != 0)
			{
				if(IICFlag.bit.Task_PURead == TASK_NEED)
				{
					IIcTask.Handle = TASK_READALL;	//上电全读
				}
				else	if(IICFlag.bit.Task_PDSave== TASK_NEED)
				{
					IIcTask.Handle = TASK_PDSAVE;	//掉电参数存储
				}
				else	if(IICFlag.bit.Task_FaultSave1== TASK_NEED)
				{
					IIcTask.Handle = TASK_FAULTSAVE1;	//故障参数存储函数
				}
				else	if(IICFlag.bit.Task_FaultSave2== TASK_NEED)
				{
					IIcTask.Handle = TASK_FAULTSAVE2;	//故障参数存储函数
				}
				else	if(IICFlag.bit.Task_Scia == TASK_NEED)
				{
					IIcTask.Handle = TASK_SCIA;	//SCIA写
				}
//				else	if(IICFlag.bit.Task_Scib== TASK_NEED)
//				{
//					IIcTask.Handle = TASK_SDO;	//SCIb写
//				}
				else	if(IICFlag.bit.Task_FaultRead == TASK_NEED)
				{
					IIcTask.Handle = TASK_FAULTREAD;//上电后读取故障信息
				}
				else	if(IICFlag.bit.Task_FacWrtie == TASK_NEED)
				{
					IIcTask.Handle = TASK_FACSAVE;		//恢复出厂值参数
				}
			}
			else
			{
				if(IICFlag.bit.Task_DefWrite == TASK_NEED)
				{
					IIcTask.Handle = TASK_DEFSAVE;		//恢复默认值参数
				}
				else	if(IICFlag.bit.Task_FaultClr == TASK_NEED)
				{
					IIcTask.Handle = TASK_FAULTCLR;		 //故障清除
				}
				else	if(IICFlag.bit.Task_TimeClr == TASK_NEED)
				{
					IIcTask.Handle = TASK_TIMECLR;			//时间清除
				}
				if(IICFlag.bit.Task_FuncSave)	//功能码存储，（ECAT用于存储P1组到PD组之间的功能码）
				{
					IICFlag.bit.Task_FuncSave =0;
					IIcTask.Handle = TASK_FUNCSAVE;
				}
				else if(IICFlag.bit.Task_PanelSave)
				{
					IICFlag.bit.Task_PanelSave =0;		//键盘参数存储
					IIcTask.Handle = TASK_PANELSAVE;
				}
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
				else	if(IICFlag.bit.TASK_1000Rd)
				{
					//IICFlag.bit.TASK_1000Rd = 0;
					IIcTask.Handle = TASK_1000READ;
				}
				else	if(IICFlag.bit.TASK_1000Wr)
				{
					IICFlag.bit.TASK_1000Wr =0;
					IIcTask.Handle = TASK_1000SAVE;
				}
				else  if(IICFlag.bit.TASK_6000Rd)
				{
					IICFlag.bit.TASK_6000Rd =0;
					IIcTask.Handle = TASK_6000READ;
				}
#else
				else if(IICFlag.bit.Task_14)
				{
					IICFlag.bit.Task_14 =0;
					IIcTask.Handle = TASK_14;
				}
				else if(IICFlag.bit.Task_15)
				{
					IICFlag.bit.Task_15 =0;
					IIcTask.Handle = TASK_15;
				}
				else if(IICFlag.bit.Task_16)
				{
					IICFlag.bit.Task_16 =0;
					IIcTask.Handle = TASK_16;
				}
#endif
			}
		}
		else if(IICFlag.Dword.HighWord != 0)
		{
			TYPE_UNION_WORD Word;
			Word.all = IICFlag.Dword.HighWord;

			if(Word.Word.LowByte != 0)
			{
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
				if(IICFlag.bit.TASK_6000Wr)
				{
					IICFlag.bit.TASK_6000Wr =0;
					IIcTask.Handle = TASK_6000SAVE;
				}
				else if(IICFlag.bit.Task_1000WrFac)
				{
					IICFlag.bit.Task_1000WrFac =0;
					IIcTask.Handle = TASK_1000FACSAVE ;
				}
				else if(IICFlag.bit.Task_6000WrFac)
				{
					IICFlag.bit.Task_6000WrFac =0;
					IIcTask.Handle = TASK_6000FACSAVE;
				}
#else
				if(IICFlag.bit.Task_17)
				{
					IICFlag.bit.Task_17 = 0;
					IIcTask.Handle = TASK_17;
				}
				else if(IICFlag.bit.Task_18)
				{
					IICFlag.bit.Task_18 = 0;
					IIcTask.Handle = TASK_18;
				}
				else if(IICFlag.bit.Task_19)
				{
					IICFlag.bit.Task_19 = 0;
					IIcTask.Handle = TASK_19;
				}               
#endif
				else if(IICFlag.bit.Task_P1_PDFuncSave)
				{
					IICFlag.bit.Task_P1_PDFuncSave = 0;
					IIcTask.Handle = Task_P1_PDFUNCSAVE;
				}
				else if(IICFlag.bit.Task_PE_PFFuncSave)
				{
					IICFlag.bit.Task_PE_PFFuncSave = 0;
					IIcTask.Handle = Task_PE_PFFUNCSAVE;
				}
				else if(IICFlag.bit.Task_Reset)
				{
					IICFlag.bit.Task_Reset = 0;
					IIcTask.Handle = TASK_RESET;
				} 
                else if(IICFlag.bit.Task_23)
				{
					IICFlag.bit.Task_23 = 0;
					IIcTask.Handle = TASK_23;
				}             
				else if(IICFlag.bit.Task_6000WrFacFull)
				{
					IICFlag.bit.Task_6000WrFacFull = 0;
					IIcTask.Handle = TASK_6000FACSAVEFULL;
				}
			}
			else
			{
				if(IICFlag.bit.TASK_6000RdFull)
				{
					IICFlag.bit.TASK_6000RdFull = 0;
					IIcTask.Handle = TASK_6000READFull;
				}
				else if(IICFlag.bit.Task_26)
				{
					IICFlag.bit.Task_26 =0;
					IIcTask.Handle = TASK_26;
				}
				else if(IICFlag.bit.Task_27)
				{
					IICFlag.bit.Task_27 =0;
					IIcTask.Handle = TASK_27;
				}
				else if(IICFlag.bit.Task_28)
				{
					IICFlag.bit.Task_28 =0;
					IIcTask.Handle = TASK_28;
				}
				else if(IICFlag.bit.Task_29)
				{
					IICFlag.bit.Task_29 =0;
					IIcTask.Handle = TASK_29;
				}
				else if(IICFlag.bit.Task_30)
				{
					IICFlag.bit.Task_30 =0;
					IIcTask.Handle = TASK_30;
				}
				else if(IICFlag.bit.Task_31)
				{
					IICFlag.bit.Task_31 =0;
					IIcTask.Handle = TASK_31;
				}
				else if(IICFlag.bit.Task_32)
				{
					IICFlag.bit.Task_32 =0;
					IIcTask.Handle = TASK_32;
				}

			}
		}
	}
	else if(IIcTask.Front!=IIcTask.Rear)		    //功能码存储队列不空即有功能码需要存储
	{
		IIcTask.Handle = TASK_FUNCSAVE;
	}
    else if(IIcTask.FrontSci!=IIcTask.RearSci)		//功能码存储队列不空即有功能码需要存储
	{
		IIcTask.Handle = TASK_SCIA;
	}
    else if(IIcTask.FrontSdo!=IIcTask.RearSdo)		//功能码存储队列不空即有功能码需要存储
	{
		IIcTask.Handle = TASK_SDO;
	}
    else
    {
        
    }
}
/******************************************************************************
**函 数 名：void IIc_TaskSchedule(void)
**描    述：选择执行任务函数
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_TaskSchedule(void)
{
	if(I2CCtlVal.IIcStatus == IIC_OPE_IDLE)		//IIC总线处于空闲状态
	{
		switch(IIcTask.Handle)
		{
			case TASK_IDLE:
				IIc_TaskSel();                           //任务调度
				break;
			case TASK_PDSAVE:							//读取掉电参数函数，目前没有改功能
				IIc_TaskPowerDownWr();
				break;
			case TASK_FAULTSAVE1:
                IIc_TaskFaultWr();						//第一次上电初始化故障参数
				break;
			case TASK_FAULTSAVE2:
				break;
			case TASK_SCIA:								//scia需要进行总线的读写
				IIc_TaskSCIA();
				break;
			case TASK_SDO:
                IIc_TaskSdo();                          //SDO存储
				break;
			case TASK_READALL: 							//上电全读
				IIc_TaskReadAll();
				break;
			case TASK_FAULTREAD:						//上电读故障信息函数
				IIc_TaskFaultUpRd();
				break;
			case TASK_FACSAVE:							//恢复出厂值函数，包括PE，PF(第一次上电)
				IIc_TaskRestorFactory();
				break;
			case TASK_DEFSAVE:
				IIc_TaskRestorP1_PD();                 //恢复默认值 P1~PD
				break;
			case TASK_FAULTCLR:
				IIc_TaskFaultClr();                     //清除历史故障
				break;
			case TASK_TIMECLR:
				IIc_TaskRunTimeClr();
				break;
			case TASK_FUNCSAVE:
				IIc_TaskFuncSave();						//功能码存储
				break;
			case TASK_PANELSAVE:
				IIc_TaskPanelSave();
				break;
            case Task_P1_PDFUNCSAVE:
			    IIc_TaskP1_PD_FuncSave();
			break;
            case Task_PE_PFFUNCSAVE:
			    IIc_TaskPE_PF_FuncSave();
			break;
            case TASK_RESET:
			    IIc_TaskResetFactory();
			break;
#if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
			case TASK_1000READ:
				IIc_Task402_1000UpRd();
				break;
			case TASK_1000SAVE:
				IIc_Task402_1000Save();    //RAM值保存EEPROM
				break;
			case TASK_6000READ:
				IIc_Task402_6000UpRd();
				break;
			case TASK_6000SAVE:
				IIc_Task402_6000Save();    //RAM值保存EEPROM
				break;
			case TASK_1000FACSAVE:
				IIc_Task402_1000UpWrFac(); //写入默认值
				break;
			case TASK_6000FACSAVE:
				IIc_Task402_6000UpWrFac(); //写入默认值
				break;
            case TASK_6000FACSAVEFULL:
                IIc_Task402_6000UpWrFacFull();
            break;           
            case TASK_6000READFull:
                IIc_Task402_6000UpRdFull();
            break;            
#endif
			default :
				break;				
		}
	}
}
/******************************************************************************
**函 数 名：void IIcInterfaceA(Uint16 Task,Uint16 Function,Uint16 size)
**描    述：iic存储对外接口函数，键盘存储
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIcInterfaceA(Uint16 *Func_Addr,Uint16 size)
{
    Uint16 StartAddr;
    Uint16 OffsetAddr;

	if((Func_Addr >= RamFuncStartAddr) && Func_Addr <= RamFuncEndAddr)
	{
         StartAddr = IIC_FUNC_ADDR;
		 OffsetAddr = Func_Addr - RamFuncStartAddr;
	}
	else
	{
		return;
	}

    IICFlag.bit.Task_PanelSave = TASK_NEED;
    IIcTask.PANEL_Arg.IIcAddr = StartAddr + OffsetAddr;
    IIcTask.PANEL_Arg.size = size;
    IIcTask.PANEL_Arg.Src_Addr = Func_Addr;
    
    
//	switch(Task)
//	{
//		case TASK_SCIA:        //scia存储接口
//			IICFlag.bit.Task_Scia = TASK_NEED;
//			IIcTask.SCIA_Arg.IIcAddr = StartAddr + OffsetAddr;
//			IIcTask.SCIA_Arg.size = size;
//			IIcTask.SCIA_Arg.Src_Addr = Func_Addr;
//			break;
//		case TASK_SCIB:       //scib存储接口
//			IICFlag.bit.Task_Scib= TASK_NEED;
//			IIcTask.SCIB_Arg.IIcAddr = StartAddr + OffsetAddr;
//			IIcTask.SCIB_Arg.size = size;
//			IIcTask.SCIB_Arg.Src_Addr = Func_Addr;
//			break;
//
//		case TASK_PANELSAVE:	//键盘存储接口
//			IICFlag.bit.Task_PanelSave = TASK_NEED;
//			IIcTask.PANEL_Arg.IIcAddr = StartAddr + OffsetAddr;
//			IIcTask.PANEL_Arg.size = size;
//			IIcTask.PANEL_Arg.Src_Addr = Func_Addr;
//			break;
//		default:
//			break;
//	}
}
/******************************************************************************
**函 数 名：void IIcInterfaceB(Uint16 *RomFunc_Addr, Uint16 size)
**描    述：程序内部存储，不需要传递任务号
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIcInterfaceB(Uint16 *RomFunc_Addr, Uint16 size)
{
    Uint16 OffsetAddr;
    Uint16 Rear;

	if((RomFunc_Addr >= RamFuncStartAddr) && RomFunc_Addr <=RamFuncEndAddr)
	{
		 OffsetAddr = RomFunc_Addr - RamFuncStartAddr;
	}
	else
	{
		return;
	}
    
    Rear = IIcTask.Rear;
    
	if(IIcTask.Front == ((Rear+1)%PENDINGTASKMAX) )  //功能码存储循环队列满
	{
		//报EEPROM故障
	}
	else
	{
		IIcTask.TASK[Rear].IIcAddr = IIC_FUNC_ADDR + OffsetAddr;		//循环队列入队
		IIcTask.TASK[Rear].size = size;
		IIcTask.TASK[Rear].Src_Addr= RomFunc_Addr;
		IIcTask.Rear = (IIcTask.Rear+1)%PENDINGTASKMAX;					//循环队列尾指针前移
	}
}

/******************************************************************************
**函 数 名：void IIcInterfaceB(Uint16 *RomFunc_Addr, Uint16 size)
**描    述：485通信存储，不需要传递任务号
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2024-04-10
*******************************************************************************/
void IIcInterfaceC(Uint16 *Func_Addr,Uint16 size)
{
    Uint16 StartAddr;
    Uint16 OffsetAddr;
    Uint16 Rear;

	if((Func_Addr >= RamFuncStartAddr) && Func_Addr <= RamFuncEndAddr)
	{
         StartAddr = IIC_FUNC_ADDR;
		 OffsetAddr = Func_Addr - RamFuncStartAddr;
	}
    else if((Func_Addr >= ModbusCumObj_StartAddr) && Func_Addr <= ModbusCumObj_EndAddr)
	{
         StartAddr = ModbusIIC_ADDR_0x1000;
		 OffsetAddr = Func_Addr - ModbusCumObj_StartAddr;
	}
    else if((Func_Addr >= ModbusCIA402Obj_StartAddr) && Func_Addr <= (ModbusCIA402Obj_StartAddr + ModbusCIA402OBJ_SIZE))
	{
         if(RamPD->FullCloseMode == 0)
         {
             StartAddr = IIC_0x6000_STATR;
         }
         else
         {
             StartAddr = IIC_ECATFULL_ADDR; 
         }
		 OffsetAddr = Func_Addr - ModbusCIA402Obj_StartAddr;
	}
	else
	{
		return;
	}

    Rear = IIcTask.RearSci;
    
	if(IIcTask.FrontSci == ((Rear+1)%SCITASKMAX) )  //功能码存储循环队列满
	{
		//报EEPROM故障
	}
	else
	{
		IIcTask.TASKSCI[Rear].IIcAddr = StartAddr + OffsetAddr;		//循环队列入队
		IIcTask.TASKSCI[Rear].size = size;
		IIcTask.TASKSCI[Rear].Src_Addr= Func_Addr;
		IIcTask.RearSci = (IIcTask.RearSci+1)%SCITASKMAX;		//循环队列尾指针前移
	}
}

/******************************************************************************
**函 数 名：void IIcInterfaceD(Uint16 index,Uint16 subindex,Uint16 *Func_Addr,Uint16 size)
**描    述：SDO通信存储，不需要传递任务号
** 调用
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2025-08-08
*******************************************************************************/
void IIcInterfaceD(Uint16 index,Uint16 subindex,Uint16 *Func_Addr,Uint16 size)
{
    Uint16 StartAddr;
    Uint16 OffsetAddr;
    Uint16 Rear;
    Uint16 flag;
    
	if((Func_Addr >= RamFuncStartAddr) && Func_Addr <= RamFuncEndAddr)
	{
         index = index - 0x3000;

         flag = 0;
         switch(index)
         {
             case 0x013D:
                 flag = 1;
             break;
             case 0x013F:
                 flag = 1;
             break;
             case 0x0800:
                 flag = 1;
             break;
             case 0x0801:
                 flag = 1;
             break;
             case 0x081E:
                 flag = 1;
             break;
             case 0x0E00:
                 flag = 1;
             break;
             case 0x0E14:
                 flag = 1;
             break;
             case 0x0F37:  
                 flag = 1;
             break;
             default:
                 flag = 0;
             break;
         }

         if(flag == 1)
         {
             return;
         }
         StartAddr = IIC_FUNC_ADDR;
		 OffsetAddr = Func_Addr - RamFuncStartAddr;
	}
    else if((Func_Addr >= ModbusCumObj_StartAddr) && Func_Addr <= ModbusCumObj_EndAddr)
	{
         if(index == 0x10F1 && subindex == 0x02)
         {
             StartAddr = ModbusIIC_ADDR_0x1000;
		     OffsetAddr = Func_Addr - ModbusCumObj_StartAddr;
         }
         else
         {
             return;
         }
	}
    else if((Func_Addr >= ModbusCIA402Obj_StartAddr) && Func_Addr <= (ModbusCIA402Obj_StartAddr + ModbusCIA402OBJ_SIZE))
	{
         if(RamPD->FullCloseMode == 0)
         {
             StartAddr = IIC_0x6000_STATR;
         }
         else
         {
             StartAddr = IIC_ECATFULL_ADDR; 
         }
		 OffsetAddr = Func_Addr - ModbusCIA402Obj_StartAddr;
	}
	else
	{
		return;
	}

    Rear = IIcTask.RearSdo;
    
	if(IIcTask.FrontSdo == ((Rear+1)%SCITASKMAX) )  //功能码存储循环队列满
	{
		//报EEPROM故障
	}
	else
	{
		IIcTask.TASKSDO[Rear].IIcAddr = StartAddr + OffsetAddr;		//循环队列入队
		IIcTask.TASKSDO[Rear].size = size;
		IIcTask.TASKSDO[Rear].Src_Addr= Func_Addr;
		IIcTask.RearSdo = (IIcTask.RearSdo+1)%SCITASKMAX;		//循环队列尾指针前移
	}
}

/******************************************************************************
**函 数 名：void PowerOnReadFunc(void)
**描    述：上电全读功能码，包括掉电参数
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void PowerOnReadFunc(void)
{	
    Uint8 i = 0;
    Uint16 EEPROMser = 0;
    Uint16 HWNum = 0;
    
    for ( i = 0; i < 4 ; i++)											// get new firmware identify 
    {
        tbl_identifyRAM[i] = tbl_identify[i];
    }
  
    EEPROMser = tbl_identifyRAM[1];
    
	IICFlag.bit.Task_PURead = TASK_NEED;           //上电第一次读
    IIcTask.ServoAllRead = 1;
    IIcTask.Cnt_1ms = 0;
    while(IIcTask.ServoAllRead == 1)
    {
        if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
        {
            memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
            memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
            #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
            memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
            memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
            #endif
            RamMonitor->CPUASer1 = tbl_identifyRAM[0]; 
            //RamMonitor->EEASer1  = tbl_identifyRAM[1];
            RamServo->SoftVersion = tbl_identifyRAM[1];

            FaultPrtt_FaultInterface(PramRecErr);
            return ;
        }
    } 
    
	if(RamFuncCode.PF[0] == IIC_DATE_NULL && RamFuncCode.P1[62] == IIC_DATE_NULL)
	{
		IICFlag.bit.Task_PURead = TASK_NEED;//上电第二次读
		//while(IICFlag.bit.Task_PURead == TASK_NEED);
        IIcTask.ServoAllRead = 1;
        IIcTask.Cnt_1ms = 0;
        while(IIcTask.ServoAllRead == 1)
        {
            if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
            {
                memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
                memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
                #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
                memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
                memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
                #endif
                
                RamMonitor->CPUASer1 = tbl_identifyRAM[0]; 
                //RamMonitor->EEASer1  = tbl_identifyRAM[1];
                RamServo->SoftVersion = tbl_identifyRAM[1];  
              
                FaultPrtt_FaultInterface(PramRecErr);
                return ;
            }
        }
        
		if(RamFuncCode.PF[0] == IIC_DATE_NULL && RamFuncCode.P1[62] == IIC_DATE_NULL)
		{
			IICFlag.bit.Task_PURead = TASK_NEED; //上电第3次读
			//while(IICFlag.bit.Task_PURead == TASK_NEED);
            IIcTask.ServoAllRead = 1;
            IIcTask.Cnt_1ms = 0;
            while(IIcTask.ServoAllRead == 1)
            {
                if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
                {
                    memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
                    memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
                    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
                    memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
                    memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
                    #endif
                    RamMonitor->CPUASer1 = tbl_identifyRAM[0]; 
                    //RamMonitor->EEASer1  = tbl_identifyRAM[1];
                    RamServo->SoftVersion = tbl_identifyRAM[1];    
                  
                    FaultPrtt_FaultInterface(PramRecErr);
                    return ;
                }
            } 
            
            if(RamFuncCode.PF[0] == IIC_DATE_NULL && RamFuncCode.P1[62] == IIC_DATE_NULL)
            {         
                memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
			    memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
                #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
                memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
                memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
                #endif
                
                IICFlag.bit.Task_FacWrtie = TASK_NEED;//恢复出厂值
                IIcTask.ServoInit = 1;                //表示这是初始化过程,空EEPROM在这里写入
                IIcTask.Cnt_1ms = 0;
                while(IIcTask.ServoInit == 1)
                {
                    if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
                    {
                        FaultPrtt_FaultInterface(PramWErr);
                        return ;
                    }
                } 
            }
            else 
            {
                if((RamServo->SoftVersion != EEPROMser) && (I2CCtlVal.I2cErrFlag == 0))
                {
                    IICFlag.bit.Task_FacWrtie = TASK_NEED;//恢复出厂值
                    IIcTask.ServoInit = 1;                //表示这是初始化过程
                
                    HWNum = RamMonitor->HardWareNum;
                    memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
    			    memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
                    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
                    memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
                    memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
                    #endif
                    RamMonitor->HardWareNum = HWNum;
                
                    IIcTask.Cnt_1ms = 0;
                    while(IIcTask.ServoInit == 1)
                    {
                        if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
                        {
                            FaultPrtt_FaultInterface(PramWErr);
                            return ;
                        }
                    }     
                }
            }
		}
        else 
        {
            if((RamServo->SoftVersion != EEPROMser) && (I2CCtlVal.I2cErrFlag == 0))
    		{
    			IICFlag.bit.Task_FacWrtie = TASK_NEED;//恢复出厂值
                IIcTask.ServoInit = 1;                //表示这是初始化过程
            
                HWNum = RamMonitor->HardWareNum;
            
                memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));//赋出厂值，例外处理
    			memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
                #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
                memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
                memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
                #endif
            
                RamMonitor->HardWareNum = HWNum;
            
                IIcTask.Cnt_1ms = 0;
                while(IIcTask.ServoInit == 1)
                {
                    if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
                    {
                        FaultPrtt_FaultInterface(PramWErr);
                        return ;
                    }
                } 
    		}
        }
	}
	else 
    {
        if((RamServo->SoftVersion != EEPROMser) && (I2CCtlVal.I2cErrFlag == 0))
    	{
    		IICFlag.bit.Task_FacWrtie = TASK_NEED;//恢复出厂值
            IIcTask.ServoInit = 1;//表示这是初始化过程
        
            HWNum = RamMonitor->HardWareNum;
        
            memcpy(&RamFuncCode,&FactoryFunc,sizeof(TYPE_FuncCodeVal));             //赋出厂值，例外处理
    	    memset(&FaultInfo,0,sizeof(TYPE_FAULTINFO));
            #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
            memcpy(RamCumObj_StartAddr,FanCumObj_StartAddr,FANCUMOBJ_SIZE*2);		//将1000对象字典默认值写入ram
            memcpy(&LocalAxes.Objects,&DefCiA402ObjectValues,SIZEOF(CiA402Objects));
            #endif
        
            RamMonitor->HardWareNum = HWNum;
        
            IIcTask.Cnt_1ms = 0;
            while(IIcTask.ServoInit == 1)
            {
                if(IIcTask.Cnt_1ms > IIC_POWERUP_TIMEMAX)
                {
                    FaultPrtt_FaultInterface(PramWErr);
                    return;
                }
            } 
    	}
    }
    
    HWNum = RamMonitor->HardWareNum;
    memset(&RamFuncCode.P0[0],0,sizeof(TYPE_MONITOR));
    RamMonitor->HardWareNum = HWNum;
   
    RamMonitor->CPUASer1 = PRODUCTCODE;

    IIC_PowerUpCheck();
    Flash_Eeprom_Init();

    if(I2CCtlVal.I2cErrFlag == 1)//防止EEPROM出错导致通信完全断开
    {
        RamCommu->Addrss = 255;
        RamCommu->Baud   = 5;
        RamCommu->Format = 0;
    }
}
/******************************************************************************
**函 数 名：void IIc_Fun(void
**描    述：iic附加功能，检测是否恢复工厂值，默认值，清除历史故障
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIc_Fun(void)
{
    static Uint16 ErrSaveCnt = 0;
    
    if(RamBasePara->ParaInit == 0xfffD)        //所有参数恢复出厂值
	{     
        IIcTask.ServoInit = 1;                 //置1可以复位ECAT参数
		IICFlag.bit.Task_Reset= TASK_NEED;     //P1~PF恢复出厂值
        
        RamBasePara->ParaInit = 0;
	}

    if(RamBasePara->ParaInit == 32768)
    {
        RamBasePara->ParaInit = 0;
        Flash_Eeprom_WrFac();
    }
      
    if(RamBasePara->ParaInit == 0xfffC)             //保存PE，PF
	{
		IICFlag.bit.Task_PE_PFFuncSave = TASK_NEED;  
        RamBasePara->ParaInit = 0;
	}

    if(RamBasePara->ParaInit == 0xfffB)             //保存所有参数
	{
        IICFlag.bit.Task_P1_PDFuncSave = TASK_NEED;//P1-PD保存
        #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
        IICFlag.bit.TASK_1000Wr = TASK_NEED;//1000保存
        IICFlag.bit.TASK_6000Wr = TASK_NEED;//6000保存
        #endif
		IICFlag.bit.Task_PE_PFFuncSave = TASK_NEED;  
        RamBasePara->ParaInit = 0;
	}

    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    if(RamBasePara->ParaInit== 11)//
	{
		IICFlag.bit.Task_1000WrFac = TASK_NEED;//1000系列恢复出厂值
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit== 22)//
	{
        if(RamPD->FullCloseMode == 0)
		    IICFlag.bit.Task_6000WrFac = TASK_NEED;    //6000系列恢复出厂值
        else
            IICFlag.bit.Task_6000WrFacFull = TASK_NEED;//6000系列恢复出厂值
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit== 33)//
	{
		IICFlag.bit.TASK_1000Wr = TASK_NEED;//1000保存
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit== 44)//
	{
		IICFlag.bit.TASK_6000Wr = TASK_NEED;//6000保存
		RamBasePara->ParaInit = 0;
	}
    #endif 
    
    if(RamBasePara->ParaInit == 55)//
	{
		IICFlag.bit.Task_P1_PDFuncSave = TASK_NEED;//P1-PD保存
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit == 66)//
	{
		IICFlag.bit.Task_DefWrite = TASK_NEED;//P1-PD恢复
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit == 77)          //恢复用户参数
	{
		IICFlag.bit.Task_DefWrite= TASK_NEED;//恢复p1-PD默认值值
        
        #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
        if(RamPD->FullCloseMode == 0)
		    IICFlag.bit.Task_6000WrFac = TASK_NEED;    //6000系列恢复出厂值
        else
            IICFlag.bit.Task_6000WrFacFull = TASK_NEED;//6000系列恢复出厂值
		IICFlag.bit.Task_1000WrFac = TASK_NEED;//1000系列恢复出厂值
        #endif
        
		RamBasePara->ParaInit = 0;
	}
    
    if(RamBasePara->ParaInit == 88)//      //保存用户参数
	{
		IICFlag.bit.Task_P1_PDFuncSave = TASK_NEED;//P1-PD保存
        #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
        IICFlag.bit.TASK_1000Wr = TASK_NEED;//1000保存
        IICFlag.bit.TASK_6000Wr = TASK_NEED;//6000保存
        #endif
        
		RamBasePara->ParaInit = 0;
	}
    
	if(1 == FaultP.EepFlg)//存储历史故障      
	{
        ErrSaveCnt++;
        if(ErrSaveCnt > 5000)//排除断电时出现ER.001 和 ER.00A,存储在EEPROM
        {
            ErrSaveCnt = 0;
            FaultP.EepFlg = 0;
		    IICFlag.bit.Task_FaultSave1 = TASK_NEED;	//故障储存任务有效
        }
	}
    else
    {
        ErrSaveCnt = 0;      
    }

	if(RamBasePara->ParaInit == 0x0001)//清除历史故障
	{
        RamBasePara->ParaInit = 0;
		IICFlag.bit.Task_FaultClr = TASK_NEED;
	}
    
//	if(RamServo->Clear == 0x0002)//清除程序运行时间
//	{
//        RamServo->Clear = 0;
//		IICFlag.bit.Task_TimeClr = TASK_NEED;
//	}
}

void IIC_PowerUpCheck(void)
{
    RamServo->MASK_E35 = 0;

    #if SERVOTYPE == SERVO_ETHERCAT || SERVOTYPE == SERVO_CANOPEN
    if((RamBasePara->FactoryCode1 == FactoryFunc.P1[62]) && 
       (RamServo->ACR_Test        == FactoryFunc.PF[61]) &&      
       (CumObj.sIdentity.u32VendorID == DefCommuObjectsValues.sIdentity.u32VendorID) &&
       (LocalAxes.Objects.objSupportedDriveModes == DefCiA402ObjectValues.objSupportedDriveModes))
    {

    }
    else
    {
        FaultPrtt_FaultInterface(IICCheckErr);
        I2CCtlVal.I2cErrFlag = 1;
    }
    #else
    if((RamBasePara->FactoryCode1 == FactoryFunc.P1[62]) && 
       (RamServo->ACR_Test        == FactoryFunc.PF[61]))
    {

    }
    else
    {
        FaultPrtt_FaultInterface(IICCheckErr);
        I2CCtlVal.I2cErrFlag = 1;
    }
    #endif
}

void Flash_Eeprom_Init(void)
{
    Uint16 temp;
    flash_read((uint8_t *)&Flash_EeData, MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
   
    if(I2CCtlVal.I2cErrFlag == 1) return;

    if(Flash_EeData[0] == 0xA55A)
    {
        if(RamMotor->EeWFlag == 0x5AA5)
        {
            Flash_EeData[1 + 18] = RamFuncCode.PE[18];
            Flash_EeData[1 + 19] = RamFuncCode.PE[19];
            Flash_EeData[1 + 23] = RamFuncCode.PE[23];
            Flash_EeData[1 + 24] = RamFuncCode.PE[24];
            Flash_EeData[1 + 25] = RamFuncCode.PE[25];
            Flash_EeData[1 + 26] = RamFuncCode.PE[26];

            flash_erase(MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
            flash_write((uint8_t *)&Flash_EeData, MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
            RamMotor->EeWFlag = 0;
            IIcInterfaceB(&RamMotor->EeWFlag,1);//将学习结果保存 

            temp = sizeof(TYPE_MOTOR)/2;
            memcpy(RamFuncCode.PE,&Flash_EeData[1],sizeof(TYPE_MOTOR));
            RamFuncCode.PF[4] = Flash_EeData[temp + 1];
            RamFuncCode.PF[6] = Flash_EeData[temp + 2];
            RamFuncCode.PF[8] = Flash_EeData[temp + 3];
            RamFuncCode.PF[14] = Flash_EeData[temp + 4];
        }
        else
        {
            temp = sizeof(TYPE_MOTOR)/2;
            memcpy(RamFuncCode.PE,&Flash_EeData[1],sizeof(TYPE_MOTOR));
            RamFuncCode.PF[4] = Flash_EeData[temp + 1];
            RamFuncCode.PF[6] = Flash_EeData[temp + 2];
            RamFuncCode.PF[8] = Flash_EeData[temp + 3];
            RamFuncCode.PF[14] = Flash_EeData[temp + 4];
        }
    }
    else
    {
        flash_erase(MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
        temp = sizeof(TYPE_MOTOR)/2;
        Flash_EeData[0] = 0xA55A;
        memcpy(&Flash_EeData[1],RamFuncCode.PE,sizeof(TYPE_MOTOR));
        Flash_EeData[temp + 1] = RamFuncCode.PF[4];
        Flash_EeData[temp + 2] = RamFuncCode.PF[6];
        Flash_EeData[temp + 3] = RamFuncCode.PF[8];
        Flash_EeData[temp + 4] = RamFuncCode.PF[14];        
        Flash_EeData[HARDADDRES] = HARDWARENUM;
        flash_write((uint8_t *)&Flash_EeData, MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
    }
}

//电机组和伺服组参数复位
void Flash_Eeprom_WrFac(void)
{
    Uint16 temp;

    ServiceDog();
    flash_erase(MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
    ServiceDog();
    temp = sizeof(TYPE_MOTOR)/2;
    memcpy(&Flash_EeData[1],RamFuncCode.PE,sizeof(TYPE_MOTOR));
    Flash_EeData[temp + 1] = RamFuncCode.PF[4];
    Flash_EeData[temp + 2] = RamFuncCode.PF[6];
    Flash_EeData[temp + 3] = RamFuncCode.PF[8];
    Flash_EeData[temp + 4] = RamFuncCode.PF[14];
    ServiceDog();
    flash_write((uint8_t *)&Flash_EeData, MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
    ServiceDog();
}

void Write_SN_Process(void)
{
    if(SnWriteFlag == 1)
    {
        memcpy(&Flash_EeData[SNADDRES],ProductInf.SNCode,0x40);

        ServiceDog();
        flash_erase(MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
        ServiceDog();
        flash_write((uint8_t *)&Flash_EeData, MOTOREEPROM_START_ADDRES, EEPROM_SECTOR_SIZE);
        ServiceDog();

        SnWriteFlag = 0;
    }
}

/******************************************************************************
**函 数 名：void IIC_Process(void)
**描    述：IIC处理函数，1MS调用一次
** 调用  
**输    入：
**输    出：无
**返    回：无
**其    它：无
**日    期：2013-7-8
*******************************************************************************/
void IIC_Process(void)
{
    IIcTask.Cnt_1ms++;
	IIc_Fun();             //附加任务
	IIc_TaskSchedule();    //任务调度
	IIC_StatusSchedule();  //底层状态调度
    eeprom_process();

    Write_SN_Process();
}
#endif

