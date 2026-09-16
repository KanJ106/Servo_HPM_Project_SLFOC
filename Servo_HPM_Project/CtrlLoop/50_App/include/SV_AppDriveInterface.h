/*
 *
 *
 *  Created on: 2015-12-10
 *      Author:  rd0217
 */

#ifndef  SV_APPDRIVEINTERFACE_H_
#define  SV_APPDRIVEINTERFACE_H_
//
#include "SV_FuncCode.h"
#include "SV_FuncVar.h"//
//APP To Drive Interface Variable
//基本参数组
#define DPI_Drvmode 									(RamBasePara->DrvMode)//控制模式
#define DPI_PosCmdSource 								(RamBasePara->PosCmdSource)//位置指令源
#define DPI_PulseCmdType								(RamBasePara->PulseCmdType)//外部脉冲指令输入形式
#define DPI_MulSegPosTurn								(RamBasePara->MulSegPosTurn)//内部多段位置指令执行选择
#define DPI_SpdCmdSource								(RamBasePara->SpdCmdSource)//速度指令源
#define DPI_MulSegSpdTurn								(RamBasePara->MulSegSpdTurn)//多段速选择
//#define DPI_TrqCmdSource								(RamBasePara->TrqCmdSource)//转矩指令源


//#define DPI_ResonFrequency1								(RamBasePara->ResonFrequency1)//第一共振频率
//
//
//#define DPI_ResonFrequency2								(RamBasePara->ResonFrequency2)//第二共振频率
//#define DPI_NotchDepth2									(RamBasePara->NotchDepth2)//第二陷波深度
//#define DPI_ResonFrequency3								(RamBasePara->ResonFrequency3)//第三共振频率
//#define DPI_NotchDepth3									(RamBasePara->NotchDepth3)//第三陷波深度

#define DPI_T_CCW_AccDecPos								(RamBasePara->T_CCW_AccDecPos)//位置S型平滑正转指令加减速时间
#define DPI_T_CW_AccDecPos								(RamBasePara->T_CW_AccDecPos)//位置S型平滑反转指令加减速时间
#define DPI_T_PosLpf									(RamBasePara->T_PosLpf)//位置S型平滑曲线加减速滤波时间常数


#define DPI_T_SpdAcc									(RamBasePara->T_SpdAcc)//速度S型平滑指令加速时间常数
#define DPI_T_SpdDec									(RamBasePara->T_SpdDec)//速度S型平滑指令减速时间常数
#define DPI_T_SpdAccDecLpf								(RamBasePara->T_SpdAccDecLpf)//速度S型平滑曲线加减速滤波时间常数

//TODO:部分有两套变量

//#define DPI_NumeratorGain								(RamBasePara->NumeratorGain)//位置指令分倍频分子倍率
//#define DPI_PrescalerNumerator1						    (RamBasePara->PrescalerNumerator1)//P1-27电子齿轮分倍频分子1
//#define DPI_PrescalerDenominator						(RamBasePara->PrescalerDenominator)//P1-28电子齿轮分倍频分母
//#define DPI_PrescalerNumerator2							(RamBasePara->PrescalerNumerator2)//电子齿轮分倍频分子2
//#define DPI_PrescalerNumerator3							(RamBasePara->PrescalerNumerator3)//电子齿轮分倍频分子3
//#define DPI_PrescalerNumerator4							(RamBasePara->PrescalerNumerator4)//电子齿轮分倍频分子4

#define DPI_PosApproachWidth							(RamBasePara->PosApproachWidth)//定位接近宽度
#define DPI_PosFinishWidth								(RamBasePara->PosFinishWidth)//定位完成宽度
#define DPI_DIClearOffset								(RamBasePara->DIClearOffset)//位置偏差计数器清除选择
#define DPI_AutoClearOffset								(RamBasePara->AutoClearOffset)//位置偏差自动清除选择
#define DPI_OffsetFlowWarnLmt							(RamBasePara->OffsetFlowWarnLmt)//位置误差跟随警告值
#define DPI_OffsetFlowFaultLmt							(RamBasePara->OffsetFlowFaultLmt)//位置误差跟随故障值
#define DPI_TrqArriveLmt								(RamBasePara->TrqArriveLmt)//转矩到达门限值
#define DPI_SpdMaxLmt									(RamBasePara->SpdMaxLmt)//最高转速限定
#define DPI_ZeroSpdLmt									(RamBasePara->ZeroSpdLmt)//零速信号输出值
#define DPI_RotatingSpdLmt								(RamBasePara->RotatingSpdLmt)//旋转信号输出值
#define DPI_SpdApproachLmt								(RamBasePara->SpdApproachLmt)//速度接近门限
#define DPI_SpdArriveLmt								(RamBasePara->SpdArriveLmt)//速度到达门限
#define DPI_AnoSpdZeroLmt								(RamBasePara->AnoSpdZeroLmt)//模拟速度零位固定值
#define DPI_CCW_TorqMaxLmt								(RamBasePara->CCW_TorqMaxLmt)//正转最大转矩限定
#define DPI_CW_TorqMaxLmt								(RamBasePara->CW_TorqMaxLmt)//反转最大转矩限定
#define DPI_SysMaxTorq	 								(RamMotor->TrqConst)//系统力矩限制
#define DPI_TrqLmtResource								(RamBasePara->TrqLmtResource)//转矩限制来源选择
#define DPI_SpdObsEn									(RamBasePara->SpdObservere)//速度观测器使能
#define DPI_T_ZeroStop									(RamBasePara->T_ZeroStop)//零速停车减速时间
#define DPI_T_Overtravel								(RamBasePara->T_Overtravel)//超程保护减速时间
#define DPI_StopMode									(RamBasePara->StopMode)//停机模式选择
#define DPI_ServoType									(RamBasePara->ServoType)
#define DPI_FactoryCode2                                (RamBasePara->FactoryCode2)
//#define DPI_AdapFltSelection                   			(RamBasePara->AdapFltSelection)//自适应滤波器模式设定
//#define DPI_NotchFrequency1                    			(RamBasePara->NotchFrequency1)//第一陷波频率
//#define DPI_NotchFrqWidthSelection1            			(RamBasePara->NotchFrqWidthSelection1)//第一陷波宽度选择
//
//#define DPI_NotchFrqDepthSelection1            			(RamBasePara->NotchFrqDepthSelection1)//第一陷波深度选择
//#define DPI_NotchFrequency2                    			(RamBasePara->NotchFrequency2)//第二陷波频率
//#define DPI_NotchFrqWidthSelection2           	 		(RamBasePara->NotchFrqWidthSelection2)//第二陷波宽度选择
//#define DPI_NotchFrqDepthSelection2         	   		(RamBasePara->NotchFrqDepthSelection2)//第二陷波深度选择
//#define DPI_NotchFrequency3               	    		(RamBasePara->NotchFrequency3)//第三陷波频率
//
//#define DPI_NotchFrqWidthSelection3       	    		(RamBasePara->NotchFrqWidthSelection3)//第三陷波宽度选择
//#define DPI_NotchFrqDepthSelection3     	       		(RamBasePara->NotchFrqDepthSelection3)//第三陷波深度选择
//#define DPI_NotchFrequency4            	        		(RamBasePara->NotchFrequency4)//第四陷波频率
//#define DPI_NotchFrqWidthSelection4   	         		(RamBasePara->NotchFrqWidthSelection4)//第四陷波宽度选择
//#define DPI_NotchFrqDepthSelection4  	          		(RamBasePara->NotchFrqDepthSelection4)//第四陷波深度选择
//
//#define DPI_ShockFltSelection      	            		(RamBasePara->ShockFltSelection)//减震滤波器切换选择
//#define DPI_VibrationFrquency1     	            		(RamBasePara->VibrationFrquency1)//低频抑制频率
//#define DPI_VibrationFrqSelection1   	          		(RamBasePara->VibrationFrqSelection1)//低频抑制增益
//#define DPI_VibrationFrquency2      	           		(RamBasePara->VibrationFrquency2)//第二减震频率
//#define DPI_VibrationFrqSelection2 	            		(RamBasePara->VibrationFrqSelection2)//第二减震滤波器设定
//
//#define DPI_VibrationFrquency3       	          		(RamBasePara->VibrationFrquency3)//第三减震频率
//#define DPI_VibrationFrqSelection3  	           		(RamBasePara->VibrationFrqSelection3)//第三减震滤波器设定
//#define DPI_VibrationFrquency4     	            		(RamBasePara->VibrationFrquency4)//第四减震频率
//#define DPI_VibrationFrqSelection4    	         		(RamBasePara->VibrationFrqSelection4)//第四减震滤波器设定
#define DPI_SpdDir    	         						(RamBasePara->SpdDir)//速度指令反向（速度模式有效）
#define DPI_RotPolar    	         					(RamBasePara->RotPolar)//旋转方向极性旋转
//转矩控制参数
#define DPI_TrqCmd										(RamTrqCtrl->TrqCmd)//内部转矩指令
#define DPI_TrqSpdLmt									(RamTrqCtrl->TrqSpdLmt)//转矩控制时速度限幅
#define DPI_TrqSpdLmtResource							(RamTrqCtrl->TrqSpdLmtResource)//转矩控制时速度限幅来源
#define DPI_TrqSpdLmtGain								(RamTrqCtrl->TrqSpdLmtGain)//转矩速度限制指令增益
#define DPI_TrqCompensation								(RamTrqCtrl->TrqCompensation)//转矩指令补偿

// 调谐参数
#define DPI_APR_P										(RamTuning->APR_P)//位置调节器比例增益
#define DPI_APR_Ap										(RamTuning->APR_Ap)//位置增益变动比率
#define DPI_APR_Kp										(RamTuning->APR_Kp)//位置调节器前馈增益
#define DPI_T_APRLpf									(RamTuning->T_APRLpf)//位置前馈滤波时间
#define DPI_ASR_P										(RamTuning->ASR_P)//速度调节器比例增益
#define DPI_ASR_Ti										(RamTuning->ASR_Ti)//速度环积分时间常数
#define DPI_ASR_Ap										(RamTuning->ASR_Ap)//速度增益变动比率
#define DPI_RegulateMode								(RamTuning->RegulateMode)//增益调整模式
#define DPI_LoadIR										(RamTuning->LoadIR)//负载惯量比
#define DPI_LoadIR2										(RamTuning->LoadIR2)//负载惯量比2
#define DPI_T_FdSpdLpf									(RamTuning->T_FdSpdLpf)//速度反馈低通滤波时间常数
#define DPI_ICPercent									(RamTuning->ICPercent)//离线学习惯量时注入电流百分比
#define DPI_SpdDf										(RamTuning->SpdDf)//速度频宽
#define DPI_Kpdff										(RamTuning->Kpdff)//PDFF控制系数
#define DPI_KpSwitch									(RamTuning->KpSwitch)//增益切换条件
#define DPI_T_KpSwitch									(RamTuning->T_KpSwitch)//增益切换时间
#define DPI_T_KpSwitchDelay								(RamTuning->T_KpSwitchDelay)//增益切换延迟时间
#define DPI_SpdKpSwitch									(RamTuning->SpdKpSwitch)//增益切换阈值
#define DPI_KpCtl										(RamTuning->KpCtl)//控制环路系数
#define DPI_T_Pos_FIR									(RamTuning->Pos_FIR)//外部脉冲滤波FIR时间常数
#define DPI_StiffnessCoefficient						(RamTuning->StiffnessCoefficient)//低频刚性系数
#define DPI_Kds											(RamTuning->Kds)//外部扰动抵抗增益
#define DPI_ACC_Kp									    (RamTuning->ACC_Kp)//加速度前馈增益
#define DPI_T_ACCLpf									(RamTuning->T_ACCLpf)//加速度前馈滤波时间常数
#define DPI_TorqueAddedValue							(RamTuning->TorqueAddedValue)//转矩指令加算值
#define DPI_TorqueCompensate_P							(RamTuning->TorqueCompensate_P)//正方向转矩补偿值
#define DPI_TorqueCompensate_N							(RamTuning->TorqueCompensate_N)//正方向转矩补偿值
#define DPI_TFricCps									(RamTuning->TFricCps)//摩擦力补偿平滑时间常数
#define DPI_Accfeedback									(RamTuning->Accfeedback)//加速度反馈功能
#define DPI_Pulsesuppression							(RamTuning->Pulsesuppression)//N脉冲抑制功能
#define DPI_LowSpdIntegral								(RamTuning->LowSpdIntegral)//低速积分功能
#define DPI_SPDObs_Function1							(RamTuning->SPDObs_Function1)//速度观测器功能
#define DPI_SPDObs_Function2							(RamTuning->SPDObs_Function2)//速度观测器功能低速无效
#define DPI_SPDObs_Spddeadline							(RamTuning->Spddeadline)//速度观测器截止频率
#define DPI_SPDObs_Function1							(RamTuning->SPDObs_Function1)//速度观测器使能
#define DPI_SPDObs_Function2							(RamTuning->SPDObs_Function2)//速度观测器使能1
#define DPI_T_SpdCmdLpf									(RamTuning->T_SpdCmdLpf)//速度指令低通滤波时间
#define DPI_T_TrqCmdLpf									(RamTuning->T_TrqCmdLpf)//转矩指令低通滤波时间
#define DPI_T_TrqCmdLpfRam								(RamTuning->T_TrqCmdLpf)//转矩指令低通滤波时间
#define DPI_T_InputPulseLpf								(RamTuning->T_InputPulseLpf)//外部脉冲滤波时间常数
//辅助功能参数
#define DPI_JodRun										(RamAid->JodRun)//点动运行
#define DPI_SpdJog										(RamAid->SpdJog)//点动速度
#define DPI_InertiaIdentificate							(RamAid->InertiaIdentificate)//离线惯量辨识
#define DPI_SON											(RamAid->SON)//内部SON指令
#define DPI_StopCondition								(RamAid->StopCondition)//停车SON条件选择
#define DPI_SONCondition								(RamAid->SONCondition)//SON条件
#define DPI_T_SONDelay									(RamAid->T_SONDelay)//SOFF后SON有效时间间隔
#define DPI_SpdSON										(RamAid->SpdSON)//SON有效速度设定
#define DPI_JOGTAccDec									(RamAid->JOGTAccDec)//JOG加减速设置
#define DPI_JidtMaxCircle								(RamAid->JidtMaxCircle)//离线惯量辨识旋转圈数限制
#define DPI_SoftOTSel                                   (RamAid->SoftOTSel)//软件软限位开关
#define DPI_EcatMinCycle                                (RamAid->EcatMinCycle)//最小通信周期

#define DPI_VibrationFrquency1     	            		(RamP9->VibrationFrquency1)//低频抑制频率
#define DPI_VibrationFrqSelection1   	          		(RamP9->VibrationFrqSelection1)//低频抑制增益

#define DPI_NotchFre1									(RamP9->NotchFre1)//第一陷波
#define DPI_NotchWidth1                                 (RamP9->NotchWidth1)//第一陷波
#define DPI_NotchDepth1									(RamP9->NotchDepth1)//第一陷波深度

//力矩闭环参数
#define DPI_TorRefCmd									(RamPC->TorRefCmd)//力矩闭环力矩命令给定

//电机参数
#define DPI_MotorCode									(RamMotor->MotorCode)//电机代码
#define DPI_Pe											(RamMotor->Pe)//电机额定功率
#define DPI_Ie											(RamMotor->Ie)//电机额定电流
#define DPI_Te											(RamMotor->Te)//电机额定转矩
#define DPI_Ue											(RamMotor->Ue)//电机额定电压
#define DPI_Ne											(RamMotor->Ne)//电机额定转速
#define DPI_Nmax										(RamMotor->Nmax)//电机最大转速
#define DPI_PoleNum										(RamMotor->PoleNum)//电机极对数
#define DPI_Lq											(RamMotor->Lq)//Q轴电感
#define DPI_LqRam										(RamMotor->Lq)//Q轴电感
#define DPI_Ld											(RamMotor->Ld)//D轴电感
#define DPI_LdRam										(RamMotor->Ld)//D轴电感
#define DPI_Rs											(RamMotor->Rs)//线间电阻
#define DPI_RsRam										(RamMotor->Rs)//线间电阻
#define DPI_Jmot										(RamMotor->Jmot)//电机转子惯量
#define DPI_EncType										(RamMotor->EncType)//编码器类型
#define DPI_EncTypeRam									(RamMotor->EncType)//编码器类型
#define DPI_EncLine										(RamMotor->EncLine)//编码器线数
#define DPI_ZeroElecAngle								(RamMotor->ZeroElecAngle)//编码器原点电角度
#define DPI_UElecAngle									(RamMotor->UElecAngle)//编码器U相上升沿电角度
#define DPI_TuningType									(RamMotor->TuningType)//电机参数自学习方式
#define DPI_ABSEnc										(RamMotor->ABSEnc)//绝对值编码器使用方法
#define DPI_ABPhaseSequ									(RamMotor->ABPhaseSequ)//增量式编码器AB相序关系
#define DPI_ACR_Pq										(RamMotor->ACR_Pq)//电流调节器Q轴比例增益
#define DPI_ACR_PqRam									(RamMotor->ACR_Pq)//电流调节器Q轴比例增益
#define DPI_ACR_Pd										(RamMotor->ACR_Pd)//电流调节器D轴比例增益
#define DPI_ACR_PdRam									(RamMotor->ACR_Pd)//电流调节器D轴比例增益
#define DPI_ACR_Iq										(RamMotor->ACR_Iq)//电流调节器Q轴积分增益
#define DPI_ACR_IqRam									(RamMotor->ACR_Iq)//电流调节器Q轴积分增益
#define DPI_ACR_Id										(RamMotor->ACR_Id)//电流调节器D轴积分增益
#define DPI_ACR_IdRam									(RamMotor->ACR_Id)//电流调节器D轴积分增益
#define DPI_ACR_KEQ										(RamMotor->ACR_KeqGain)//Q轴反电动势补偿增益
#define DPI_ACR_KED										(RamMotor->ACR_KedGain)//D轴电压补偿增益

//驱动器参数
#define DPI_ServoIe										(RamServo->ServoIe)//驱动器额定电流
#define DPI_SampleGain									(RamServo->IeSampleGain)//电流采样放大倍数
#define DPI_ServoVolLv									(RamServo->ServoVolLv)//驱动器电压等级
#define DPI_ServoCode									(RamServo->ServoCode)//驱动器代码
#define DPI_BOOT_POWER                                  (RamServo->Boot_Power)//电流采样放大倍数
#define DPI_VF_Test										(RamServo->VF_Test)//VF测试控制
#define DPI_VF_Voltage									(RamServo->VF_Voltage)//VF测试电压
#define DPI_VF_Fre										(RamServo->VF_Fre)//VF给定频率
#define VF_AddTimeS										(RamServo->VF_AddTimeS)//VF加速时间
#define DPI_InertiaTunSpdLmt							(RamServo->InertiaTunSpdLmt)//三角波注入惯量辨识速度限制
#define DPI_ACR_RES2									(RamServo->ACR_RES2)//电流两倍增益
#define DPI_ACR_HalfPI									(RamServo->ACR_HalfPI)//1/2P增益
#define DPI_FftFrq										(RamServo->reserve36)//

#define DPI_CURSTEPTEST									(RamServo->ACR_Test)//电流环增益测试开关
#define DPI_MechFftTorqRats								(RamServo->MechFftTorqRats)//机械频率特性分析注入扰动转矩大小
#define DPI_Pos_div										(RamServo->Pos_div)//位置环分频系数
#define DPI_ASR_div										(RamServo->ASR_div)//速度环分频系数
//#define DPI_Carrier									(RamServo->Carrier)//载波
#define DPI_ASR_AntiwindupTi							(RamServo->ASR_AntiwindupTi)//速度调节器积分抗饱和增益
#define DPI_ACR_AntiwindupTi							(RamServo->ACR_AntiwindupTi)//电流调节器积分抗饱和增益
#define DPI_ACR_AVR                                     (RamServo->ACR_AVR)//AVR开关
#define DPI_ACR_UFWD                                    (RamServo->ACR_UFWD)//电压前馈开关
#define DPI_Carrier										(RamServo->Carrier)//载波设置
#define DPI_ASR_RegKiSat								(RamServo->ASR_RegKiSat)//速度调节器积分饱和处理选择
#define DPI_ACR_RegKiSat								(RamServo->ACR_RegKiSat)//电流调节器积分饱和处理选择
#define DPI_IDRef										(RamServo->IDRef)//D轴电流给定
#define DPI_TorSensorOffsetVal							(RamServo->TorSensorOffsetVal)//力矩传感器偏置值

//多段位置参数
#define DPI_MultiPos1H									(RamMultiPos->MultiPos1H)//内部位置指令1脉冲数
#define DPI_MultiPos1L									(RamMultiPos->MultiPos1L)
#define DPI_MultiPos1Spd								(RamMultiPos->MultiPos1Spd)//内部位置指令1移动速度
#define DPI_T_Pos1Delay									(RamMultiPos->T_Pos1Delay)//Pr1完成后进入Pr2等待时间

#define DPI_MultiPos2H									(RamMultiPos->MultiPos2H)//内部位置指令1脉冲数
#define DPI_MultiPos2L									(RamMultiPos->MultiPos2L)
#define DPI_MultiPos2Spd								(RamMultiPos->MultiPos2Spd)//内部位置指令2移动速度
#define DPI_T_Pos2Delay									(RamMultiPos->T_Pos2Delay)//Pr2完成后进入Pr3等待时间

#define DPI_MultiPos3H									(RamMultiPos->MultiPos3H)//内部位置指令2脉冲数
#define DPI_MultiPos3L									(RamMultiPos->MultiPos3L)
#define DPI_MultiPos3Spd								(RamMultiPos->MultiPos3Spd)//内部位置指令3移动速度
#define DPI_T_Pos3Delay									(RamMultiPos->T_Pos3Delay)//Pr3完成后进入Pr4等待时间

#define DPI_MultiPos4H									(RamMultiPos->MultiPos4H)//内部位置指令4脉冲数
#define DPI_MultiPos4L									(RamMultiPos->MultiPos4L)
#define DPI_MultiPos4Spd								(RamMultiPos->MultiPos4Spd)//内部位置指令4移动速度
#define DPI_T_Pos4Delay									(RamMultiPos->T_Pos4Delay)//Pr4完成后进入Pr5等待时间

#define DPI_MultiPos5H									(RamMultiPos->MultiPos5H)//内部位置指令5脉冲数
#define DPI_MultiPos5L									(RamMultiPos->MultiPos5L)
#define DPI_MultiPos5Spd								(RamMultiPos->MultiPos5Spd)//内部位置指令5移动速度
#define DPI_T_Pos5Delay									(RamMultiPos->T_Pos5Delay)//Pr5完成后进入Pr6等待时间

#define DPI_MultiPos6H									(RamMultiPos->MultiPos6H)//内部位置指令6脉冲数
#define DPI_MultiPos6L									(RamMultiPos->MultiPos6L)
#define DPI_MultiPos6Spd								(RamMultiPos->MultiPos6Spd)//内部位置指令6移动速度
#define DPI_T_Pos6Delay									(RamMultiPos->T_Pos6Delay)//Pr6完成后进入Pr7等待时间

#define DPI_MultiPos7H									(RamMultiPos->MultiPos7H)//内部位置指令7脉冲数
#define DPI_MultiPos7L									(RamMultiPos->MultiPos7L)
#define DPI_MultiPos7Spd								(RamMultiPos->MultiPos7Spd)//内部位置指令7移动速度
#define DPI_T_Pos7Delay									(RamMultiPos->T_Pos7Delay)//Pr7完成后进入Pr8等待时间

#define DPI_MultiPos8H									(RamMultiPos->MultiPos8H)//内部位置指令8脉冲数
#define DPI_MultiPos8L									(RamMultiPos->MultiPos8L)
#define DPI_MultiPos8Spd								(RamMultiPos->MultiPos8Spd)//内部位置指令8移动速度
#define DPI_T_Pos8Delay									(RamMultiPos->T_Pos8Delay)//Pr8完成后进入Pr9等待时间

#define DPI_MultiPos9H									(RamMultiPos->MultiPos9H)//内部位置指令9脉冲数
#define DPI_MultiPos9L									(RamMultiPos->MultiPos9L)
#define DPI_MultiPos9Spd								(RamMultiPos->MultiPos9Spd)//内部位置指令9移动速度
#define DPI_T_Pos9Delay									(RamMultiPos->T_Pos9Delay)//Pr9完成后进入Pr10等待时间

#define DPI_MultiPos10H									(RamMultiPos->MultiPos10H)//内部位置指令10脉冲数
#define DPI_MultiPos10L									(RamMultiPos->MultiPos10L)
#define DPI_MultiPos10Spd								(RamMultiPos->MultiPos10Spd)//内部位置指令11移动速度
#define DPI_T_Pos10Delay								(RamMultiPos->T_Pos10Delay)//Pr10完成后进入Pr11等待时间

#define DPI_MultiPos11H									(RamMultiPos->MultiPos11H)//内部位置指令11脉冲数
#define DPI_MultiPos11L									(RamMultiPos->MultiPos11L)
#define DPI_MultiPos11Spd								(RamMultiPos->MultiPos11Spd)//内部位置指令11移动速度
#define DPI_T_Pos11Delay								(RamMultiPos->T_Pos11Delay)//Pr11完成后进入Pr12等待时间

#define DPI_MultiPos12H									(RamMultiPos->MultiPos12H)//内部位置指令12脉冲数
#define DPI_MultiPos12L									(RamMultiPos->MultiPos12L)
#define DPI_MultiPos12Spd								(RamMultiPos->MultiPos12Spd)//内部位置指令12移动速度
#define DPI_T_Pos12Delay								(RamMultiPos->T_Pos12Delay)//Pr12完成后进入Pr13等待时间

#define DPI_MultiPos13H									(RamMultiPos->MultiPos13H)//内部位置指令13脉冲数
#define DPI_MultiPos13L									(RamMultiPos->MultiPos13L)
#define DPI_MultiPos13Spd								(RamMultiPos->MultiPos13Spd)//内部位置指令13移动速度
#define DPI_T_Pos13Delay								(RamMultiPos->T_Pos13Delay)//Pr13完成后进入Pr14等待时间

#define DPI_MultiPos14H									(RamMultiPos->MultiPos14H)//内部位置指令14脉冲数
#define DPI_MultiPos14L									(RamMultiPos->MultiPos14L)
#define DPI_MultiPos14Spd								(RamMultiPos->MultiPos14Spd)//内部位置指令1移动速度
#define DPI_T_Pos14Delay								(RamMultiPos->T_Pos14Delay)//Pr14完成后进入Pr15等待时间

#define DPI_MultiPos15H									(RamMultiPos->MultiPos15H)//内部位置指令15脉冲数
#define DPI_MultiPos15L									(RamMultiPos->MultiPos15L)
#define DPI_MultiPos15Spd								(RamMultiPos->MultiPos15Spd)//内部位置指令15移动速度
#define DPI_T_Pos15Delay								(RamMultiPos->T_Pos15Delay)//Pr15完成后进入Pr16等待时间

#define DPI_MultiPos16H									(RamMultiPos->MultiPos16H)//内部位置指令16脉冲数
#define DPI_MultiPos16L									(RamMultiPos->MultiPos16L)
#define DPI_MultiPos16Spd								(RamMultiPos->MultiPos16Spd)//内部位置指令16移动速度
#define DPI_T_Pos16Delay								(RamMultiPos->T_Pos16Delay)//Pr16完成后进入Pr1等待时间

//多段速度参数

#define DPI_MultiSpd1									(RamMultiSpeed->MultiSpd1)//内部速度指令1
#define DPI_T_RunSpd1									(RamMultiSpeed->T_RunSpd1)//内部速度指令1运行时间

#define DPI_MultiSpd2									(RamMultiSpeed->MultiSpd2)//内部速度指令2
#define DPI_T_RunSpd2									(RamMultiSpeed->T_RunSpd2)//内部速度指令2运行时间

#define DPI_MultiSpd3									(RamMultiSpeed->MultiSpd3)//内部速度指令3
#define DPI_T_RunSpd3									(RamMultiSpeed->T_RunSpd3)//内部速度指令3运行时间

#define DPI_MultiSpd4									(RamMultiSpeed->MultiSpd4)//内部速度指令4
#define DPI_T_RunSpd4									(RamMultiSpeed->T_RunSpd4)//内部速度指令4运行时间

#define DPI_MultiSpd5									(RamMultiSpeed->MultiSpd5)//内部速度指令5
#define DPI_T_RunSpd5									(RamMultiSpeed->T_RunSpd5)//内部速度指令5运行时间

#define DPI_MultiSpd6									(RamMultiSpeed->MultiSpd6)//内部速度指令6
#define DPI_T_RunSpd6									(RamMultiSpeed->T_RunSpd6)//内部速度指令6运行时间

#define DPI_MultiSpd7									(RamMultiSpeed->MultiSpd7)//内部速度指令7
#define DPI_T_RunSpd7									(RamMultiSpeed->T_RunSpd7)//内部速度指令7运行时间

#define DPI_MultiSpd8									(RamMultiSpeed->MultiSpd8)//内部速度指令8
#define DPI_T_RunSpd8									(RamMultiSpeed->T_RunSpd8)//内部速度指令8运行时间

#define DPI_MultiSpd9									(RamMultiSpeed->MultiSpd9)//内部速度指令9
#define DPI_T_RunSpd9									(RamMultiSpeed->T_RunSpd9)//内部速度指令9运行时间

#define DPI_MultiSpd10									(RamMultiSpeed->MultiSpd10)//内部速度指令10
#define DPI_T_RunSpd10									(RamMultiSpeed->T_RunSpd10)//内部速度指令10运行时间

#define DPI_MultiSpd11									(RamMultiSpeed->MultiSpd11)//内部速度指令11
#define DPI_T_RunSpd11									(RamMultiSpeed->T_RunSpd11)//内部速度指令11运行时间

#define DPI_MultiSpd12									(RamMultiSpeed->MultiSpd12)//内部速度指令12
#define DPI_T_RunSpd12									(RamMultiSpeed->T_RunSpd12)//内部速度指令12运行时间

#define DPI_MultiSpd13									(RamMultiSpeed->MultiSpd13)//内部速度指令13
#define DPI_T_RunSpd13									(RamMultiSpeed->T_RunSpd13)//内部速度指令13运行时间

#define DPI_MultiSpd14									(RamMultiSpeed->MultiSpd14)//内部速度指令14
#define DPI_T_RunSpd14									(RamMultiSpeed->T_RunSpd14)//内部速度指令14运行时间

#define DPI_MultiSpd15									(RamMultiSpeed->MultiSpd15)//内部速度指令15
#define DPI_T_RunSpd15									(RamMultiSpeed->T_RunSpd15)//内部速度指令15运行时间

#define DPI_MultiSpd16									(RamMultiSpeed->MultiSpd16)//内部速度指令16
#define DPI_T_RunSpd16									(RamMultiSpeed->T_RunSpd16)//内部速度指令16运行时间
#define DPI_T_ZeroFault									(RamZeroReturn->T_ZeroFault)//原点回归超时时间
#define DPI_ZeroTrigger									(RamZeroReturn->ZeroTrigger)//原点回归方法
#endif
