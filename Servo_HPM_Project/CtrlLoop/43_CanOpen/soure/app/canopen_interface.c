#include "canopen_interface.h"
#include "SV_FuncVar.h"
#include "SV_I2c.h"
#include "SV_FaultProtect.h"

#define TMR_TASK_INTERVAL   (1000)

//**************************CANOPEN初始化**************************//
//NMT 节点网络控制
#define NMT_CONTROL   (CO_NMT_ERR_ON_ERR_REG      | \
                       CO_ERR_REG_GENERIC_ERR      | \
                       CO_ERR_REG_COMMUNICATION    | \
                       CO_ERR_REG_MANUFACTURER)

#define FIRST_HB_TIME        500

//服务端超时时间
#define SDO_SRV_TIMEOUT_TIME 1000
//客户端超时时间
#define SDO_CLI_TIMEOUT_TIME 500

#define SDO_CLI_BLOCK        false

#define OD_STATUS_BITS       NULL

#define MAX_CANOPEN_DEVICE 1

#define HEARTBEAT_SYNC_GUARD_US  (500U)

CO_t *co[MAX_CANOPEN_DEVICE];

//设备列表
struct canopen_context canptr[MAX_CANOPEN_DEVICE];

//CAN硬件配置寄存器
hpm_can_config_t hpm_canopen_config[MAX_CANOPEN_DEVICE] = {0};

hpm_can_data_t hpm_canopen_data[MAX_CANOPEN_DEVICE] = {0};

struct device hpm_canopen_dev[MAX_CANOPEN_DEVICE] = {0};

uint64_t  timestamp[MAX_CANOPEN_DEVICE];

uint32_t  elapsed[MAX_CANOPEN_DEVICE];

uint64_t last_time[MAX_CANOPEN_DEVICE];

CO_NMT_reset_cmd_t reset = CO_RESET_COMM;

uint8_t slave_node_id = 1; 

uint32_t mchtmr_freq;

uint32_t BITRATE = 0;

uint8_t NMT_State = 0xff;//0；节点上线 4:停止 5:操作 127:预操作


//CAN消息缓冲区
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
ATTR_PLACE_AT(".ahb_sram") uint32_t board_app_mcan_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

//CAN硬件配置  
can_info_t s_can_info[] = {
    {
        .can_base = BOARD_APP_CAN_BASE,     //HPM_MCAN0
        .irq_num = BOARD_APP_CAN_IRQn,      //IRQn_MCAN0
        .priority = 6,                      //中断优先级
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
        .ram_base = (uint32_t) &board_app_mcan_msg_buf,     //缓冲区基址
        .ram_size = sizeof(board_app_mcan_msg_buf),         //缓冲器大小
#endif
    },
    /* init more can base here */
};

/*
 * CANopenNode EMCY错误状态位分配：
 *
 * 0x30：电机警告
 *       发送EMCY，但不置0x1001 bit7，不退出NMT Operational。
 *
 * 0x40：电机故障
 *       发送EMCY，并置0x1001 bit7，使NMT退出Operational。
 *
 * EMCY数据：
 * Byte0~1 = 0xFF00，线上顺序为 00 FF
 * Byte2   = 0x1001错误寄存器
 * Byte3   = CANopenNode错误状态位编号
 * Byte4   = FaultP.FaultDisp
 * Byte5~7 = 0
 */
#define CO_EM_MOTOR_WARNING    ((uint8_t)0x30U)
#define CO_EM_MOTOR_FAULT      ((uint8_t)0x40U)

static uint16_t s_lastMotorFaultDisp = 0U;


/**
 * @brief 将电机警告/故障同步到CANopen EMCY。
 *
 * FaultStatus bit0：警告
 * FaultStatus bit1：故障
 *
 * 故障优先级高于警告。
 */
static void Canopen_MotorEmcyProcess(CO_t *coObj)
{
    bool_t warningReported;
    bool_t faultReported;
    uint16_t faultDisp;

    if ((coObj == NULL) || (coObj->em == NULL))
    {
        return;
    }

    warningReported = CO_isError(coObj->em, CO_EM_MOTOR_WARNING);
    faultReported   = CO_isError(coObj->em, CO_EM_MOTOR_FAULT);
    faultDisp       = FaultP.FaultDisp;

    /*
     * FaultStatus bit1置位：
     * FP_ERR或FP_WARNERR，按停机故障处理。
     */
    if ((FaultP.FaultStatus & 0x02U) != 0U)
    {
        /*
         * 故障优先。
         * 如果之前存在警告EMCY，先结束警告。
         */
        if (warningReported)
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_WARNING,
                          0U);
        }

        /*
         * CANopenNode不会重复上报已经置位的errorBit。
         * 如果故障代码发生变化，需要先结束旧事件，
         * 再发送携带新FaultDisp的EMCY。
         */
        if (faultReported && (s_lastMotorFaultDisp != faultDisp))
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_FAULT,
                          0U);

            faultReported = false;
        }

        if (!faultReported)
        {
            CO_errorReport(coObj->em,
                           CO_EM_MOTOR_FAULT,
                           CO_EMC_DEVICE_SPECIFIC,
                           (uint32_t)(faultDisp & 0x00FFU));
        }

        s_lastMotorFaultDisp = faultDisp;
    }
    /*
     * 只有FaultStatus bit0置位：
     * FP_WARN，按非停机警告处理。
     */
    else if ((FaultP.FaultStatus & 0x01U) != 0U)
    {
        if (faultReported)
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_FAULT,
                          0U);
        }

        if (warningReported && (s_lastMotorFaultDisp != faultDisp))
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_WARNING,
                          0U);

            warningReported = false;
        }

        if (!warningReported)
        {
            CO_errorReport(coObj->em,
                           CO_EM_MOTOR_WARNING,
                           CO_EMC_DEVICE_SPECIFIC,
                           (uint32_t)(faultDisp & 0x00FFU));
        }

        s_lastMotorFaultDisp = faultDisp;
    }
    /*
     * 无警告、无故障：
     * 结束已经上报的电机EMCY事件。
     */
    else
    {
        if (faultReported)
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_FAULT,
                          0U);
        }

        if (warningReported)
        {
            CO_errorReset(coObj->em,
                          CO_EM_MOTOR_WARNING,
                          0U);
        }

        s_lastMotorFaultDisp = 0U;
    }
}

uint32_t runcnt = 0;
void CanopenHeartbeatSyncGuard(CO_t *coObj, uint32_t diff_us)
{
    uint32_t period_us;
    uint32_t elapsed_us;
    uint32_t remain_us;

    if ((coObj == NULL) ||
        (coObj->SYNC == NULL) ||
        (coObj->NMT == NULL) ||
        (coObj->SYNC->OD_1006_period == NULL)) {
        return;
    }

    /* 没有启用生产者心跳 */
    if (coObj->NMT->HBproducerTime_us == 0U) {
        return;
    }

    /*
     * 0：尚未收到SYNC
     * 1：SYNC正常
     * 2：SYNC已经超时
     */
    if (coObj->SYNC->timeoutError != 1U) {
        return;
    }

    period_us = *coObj->SYNC->OD_1006_period;
    if (period_us == 0U) {
        return;
    }

    elapsed_us = coObj->SYNC->timer;

    /*
     * 超过预计SYNC时刻说明本次SYNC没有按时到达。
     * 不再阻止心跳发送。
     */
    if (elapsed_us >= period_us) {
        return;
    }

    remain_us = period_us - elapsed_us;

    /* 当前不在SYNC保护窗口 */
    if (remain_us > HEARTBEAT_SYNC_GUARD_US) {
        return;
    }

    /* 本次CO_process还不会发送周期心跳 */
    if (coObj->NMT->HBproducerTimer > diff_us) {
        return;
    }

    /*
     * 将周期心跳推迟一个CANopen处理周期。
     * CO_NMT_process扣除diff_us后还剩1us，本次不会发送。
     */
    coObj->NMT->HBproducerTimer = diff_us + 1U;

    runcnt++;
}

uint8_t canopen_init(void)
{
    CO_config_t *config_ptr = NULL;
    uint32_t heap_memory_used;
    
    mchtmr_freq = clock_get_frequency(clock_mchtmr0);//canopen 时间
    slave_node_id = RamCommu->CanID;

    //对象字典初始化
    CO_init_OD();

    for (uint8_t i = 0; i < MAX_CANOPEN_DEVICE; i++) 
    {
        /* initialize CAN controller */
        //硬件初始化

        if(RamCommu->CanBaud == 0)
        {
            BITRATE = 500000;
        }
        else
        {
            BITRATE = 1000000;   
        }

        canopen_controller_init(&canptr[i], &s_can_info[i], (uint32_t)BITRATE, i);

        //创建CanOpen对象 CO各个成员分配内存空间
        co[i] = CO_new(NULL, NULL);

        if (co[i] == NULL) 
        {
            return 0;
        } 

    }

    return 1;

}


void CanopenProccessFromIsr(void)
{
    bool_t sync_was;   
    static uint64_t time_stamp = 0;
    static uint64_t last_time = 0;
    uint16_t interval_time = 300; 
    uint32_t diff_us;

    //处于复位状态时禁止处理
    if(reset != CO_RESET_NOT) 
    {
        time_stamp = 0;
        last_time = 0;
        return;
    }

    //处理接收FIFO
    if(can_recvMsg_process(0) == 0)
    {
        //接收到PDO或者SYNC数据后，立刻处理
        interval_time = 0;
    }

    //记录时间
    time_stamp = mchtmr_get_count(HPM_MCHTMR);

    diff_us = (time_stamp - last_time) * 1000000 / mchtmr_freq;

    //减少空闲运行频率
    if(diff_us < interval_time)
    {
        return;
    }

    //PDO处理
    /* Process Sync */
    sync_was = CO_process_SYNC(co[0], diff_us, NULL);

    /* Read inputs */
    CO_process_RPDO(co[0], sync_was, diff_us, NULL);
    /* Write outputs */
    CO_process_TPDO(co[0], sync_was, diff_us, NULL);

    last_time = time_stamp;
}



uint8_t canopen_task(void)
{
    CO_ReturnError_t err;
    uint32_t timeout;
    static uint8_t step = 0;
    const uint8_t faultstep = 0xff;
    static uint8_t restartFlg = 0;
    static uint8_t restartStep = 0;

    switch(step)
    {
        case 0:
        {
            if( (reset == CO_RESET_APP) || (reset == CO_RESET_COMM) )
            {

                /* CANopen复位 */
                if(restartFlg == 1)
                {
                    switch(restartStep)
                    {
                        case 0:
                        {
                            //重置OD数据
                            IICFlag.bit.TASK_1000Rd = TASK_NEED;
                            restartStep = 1;
                            break;
                        }
                        case 1:
                        {
                            //等待IIC任务完成
                            if(IICFlag.bit.TASK_1000Rd == TASK_NEED)
                            {
                                break;
                            }

                            restartStep = 2;
                            break;
                        }             
                        case 2:
                        {
                            //重置CO对象
                            canopen_controller_init(&canptr[0], &s_can_info[0], (uint32_t)BITRATE, 0);

                            //创建CanOpen对象 CO各个成员分配内存空间
                            co[0] = CO_new(NULL, NULL);

                            if (co[0] == NULL) 
                            {
                                step = faultstep;
                                return 0;
                            }
                            restartStep = 3;
                            break;
                        }
                        default:
                        {
                            /* code */
                            break;
                        }
                    }

                    if(restartStep == 3)
                    {
                        restartStep = 0; 
                        restartFlg = 0;
                    }
                    else
                    {
                        break;
                    }
                    
                }

                for (uint8_t i = 0; i < MAX_CANOPEN_DEVICE; i++) 
                {

                    last_time[i] = mchtmr_get_count(HPM_MCHTMR);

                    co[i]->CANmodule->CANnormal = false;
          
                    //CAN Stop 处于设置模式时，关闭CAN总线
                    CO_CANsetConfigurationMode((void *)&canptr[i]);
                    //关闭所有接收过滤器
                    CO_CANmodule_disable(co[i]->CANmodule);

                    /* initialize CANopen */
                    err = CO_CANinit(co[i], &canptr[i], 125);
                    if (err != CO_ERROR_NO) 
                    {
                        step = faultstep;
                        return 0;
                    }

                    uint32_t err_info = 0;

                    //CanOpen初始化
                    err = CO_CANopenInit(co[i],                   /* CANopen object */
                                         NULL,                 /* alternate NMT */
                                         NULL,                 /* alternate em */
                                         OD,                   /* Object dictionary */
                                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                                         NMT_CONTROL,          /* CO_NMT_control_t */
                                         FIRST_HB_TIME,        /* firstHBTime_ms */
                                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                                         slave_node_id, 
                                         &err_info);

                    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
                    {
                        step = faultstep;
                        return 0;
                    }
            
                    //PDO初始化
                    err = CO_CANopenInitPDO(co[i], co[i]->em, OD, slave_node_id, &err_info);
                    if (err != CO_ERROR_NO) 
                    {
                        step = faultstep;
                        return 0;
                    }
            

                    /* start CAN */
                    CO_CANsetNormalMode(co[i]->CANmodule);

                    reset = CO_RESET_NOT;
                }

                step++;
            }
            
        }
        break;

        case 1:
        {
            if(reset == CO_RESET_NOT) 
            {
                //读取FIFO_1数据
                can_recvMsg_process(1);

                /* CANopen process */
                for (uint8_t i = 0; i < MAX_CANOPEN_DEVICE; i++) 
                {
                    //记录时间
                    timestamp[i] = mchtmr_get_count(HPM_MCHTMR);

                    uint64_t diff_ticks = timestamp[i] - last_time[i];
                    uint32_t diff_us = (uint32_t)(diff_ticks * 1000000 / mchtmr_freq);

                    //最大值待定
                    if (diff_us > 100000) 
                    {
                        diff_us = 100000; // 限幅
                    }
                    //最小间隔时间300us
                    if(diff_us < 300)
                    {
                        continue;
                    }

                    Canopen_MotorEmcyProcess(co[i]);

                    CanopenHeartbeatSyncGuard(co[i],diff_us);

                    //NMT SDO处理
                    reset = CO_process(co[i], false, diff_us, &timeout);
                    last_time[i] = timestamp[i];

                    //PDO放入中断处理

                } 
            }
            else
            {
                step = 2;
            }
        }
        break;

        case 2:
        {
            /* delete objects from memory */
            for (uint8_t i = 0; i < MAX_CANOPEN_DEVICE; i++) 
            {
                CO_CANsetConfigurationMode((void *)&canptr[i]);
                CO_delete(co[i]);
            }

            restartFlg = 1;

            step++;

        }
        break;

        case 3:
        {
            step = 0;
        }
        break;  

        default:
        {

        }

    }

    NMT_State = CO_NMT_getInternalState(co[0]->NMT);

    return  0;
}




