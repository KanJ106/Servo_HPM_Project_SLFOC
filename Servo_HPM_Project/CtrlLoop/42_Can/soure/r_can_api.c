#include <stdint.h>//

#include "r_can_api.h"
#include "SV_CanFirm.h"
#include "SV_CanbusCtrl.h"
#include "hpm_mcan_drv.h"

#if SERVOTYPE == SERVO_CAN 
//这片区域用来配置<消息存储示意图>(包括:过滤器、rxfifo、rxbuffer、txbuffer、tx event fifo)
ATTR_PLACE_AT(".ahb_sram") uint32_t mcan_msg_buf[1000];

MCAN_Type *MqbTestMcan;

mcan_rx_message_t s_can_rx_buf;
mcan_tx_frame_t tx_buf;

typedef struct {
    uint32_t ram_base;
    uint32_t ram_size;
} can_ram_t;


bool hpm_flag = false;  //远程帧中断标志位
bool Radio_flag = false;
volatile uint32_t txfifo_addr = 0;
volatile uint32_t rxbuff_addr = 0;
volatile uint32_t rxfifo_addr = 0;


uint8_t MCAN_Get_DataSize(uint8_t dlc)
{
    uint32_t msg_size;
    if (dlc <= 8U) {
        msg_size = dlc;
    } else if (dlc <= 12U) {
        msg_size = 8 + (dlc - 8) * 4;
    } else {
        msg_size = 32 + (dlc - 13) * 16U;
    }
    return msg_size;
}

hpm_stat_t MCAN_Tranmit(MCAN_Type *ptr, mcan_tx_frame_t *tx_frame) 
{
    if (ptr->TXFQS & 0x200000UL) {
        return status_fail;  
    }

    FIFO_Index = (ptr->TXFQS & 0x1F0000) >> 16;
    uint32_t *msg_hdr = (uint32_t *)(txfifo_addr + 72U * FIFO_Index);

    // 计算总字节数 = 消息头 (8B) + 数据部分
    uint8_t data_bytes = MCAN_Get_DataSize(tx_frame->dlc);  
    uint16_t total_bytes = 8 + data_bytes;  

    memcpy(msg_hdr, tx_frame, total_bytes);
    ptr->TXBAR = (1UL << FIFO_Index);  // 触发发送

    return status_success;
}

hpm_stat_t MCAN_Read_Rxbuff(MCAN_Type *ptr, uint32_t index, mcan_rx_message_t *rx_frame)
{
    hpm_stat_t status = status_invalid_argument;

    if (!(ptr->NDAT1 & (1UL << index))) {return status;}

    uint32_t *msg_hdr = (uint32_t *) (rxbuff_addr + 72U * index);
    uint8_t dlc = (uint8_t)(msg_hdr[1] >> 16 & 0x000f);
    uint8_t size_bytes = MCAN_Get_DataSize(dlc) + 8; 

    memcpy(rx_frame, msg_hdr, size_bytes);

    status = status_success;


    return status;
}

hpm_stat_t MCAN_Read_Rxfifo(MCAN_Type *ptr, mcan_rx_message_t *rx_frame)
{
    hpm_stat_t status = status_invalid_argument;

    uint32_t elem_index;  
    if (!(ptr->RXF0S & 0x7f)) {return status;}

    elem_index = (ptr->RXF0S & 0x3f00) >> 8;

    uint32_t elem_addr = rxfifo_addr + 72U * elem_index;   //获取元素基地址
    uint32_t *msg_hdr = (uint32_t *) elem_addr;                //获取元素头部基地址


    uint8_t dlc = (uint8_t) (msg_hdr[1] >> 16 & 0x000f);
    uint8_t size_bytes = MCAN_Get_DataSize(dlc) + 8;

    memcpy(rx_frame, msg_hdr, size_bytes);
    

    ptr->RXF0A = elem_index;

    status = status_success;

    return status;

}

void Init_can_pins(void)
{
    #if HARDWARE_VER_SEL == HARDWARE_VER_0
    HPM_IOC->PAD[IOC_PAD_PZ00].FUNC_CTL = IOC_PZ00_FUNC_CTL_MCAN4_TXD;
    HPM_BIOC->PAD[IOC_PAD_PZ00].FUNC_CTL = BIOC_PZ00_FUNC_CTL_SOC_PZ_00;
    HPM_IOC->PAD[IOC_PAD_PZ01].FUNC_CTL = IOC_PZ01_FUNC_CTL_MCAN4_RXD;
    HPM_BIOC->PAD[IOC_PAD_PZ01].FUNC_CTL = BIOC_PZ01_FUNC_CTL_SOC_PZ_01;
    HPM_IOC->PAD[IOC_PAD_PZ02].FUNC_CTL = IOC_PZ02_FUNC_CTL_MCAN4_STBY;
    HPM_BIOC->PAD[IOC_PAD_PZ02].FUNC_CTL = BIOC_PZ02_FUNC_CTL_SOC_PZ_02;
    #elif HARDWARE_VER_SEL == HARDWARE_VER_1  //MQB CANFD调试
    HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_MCAN0_TXD;
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_MCAN0_RXD;
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_MCAN0_STBY;
    #endif
}


void mcan_get_default_ram_config1(MCAN_Type *ptr, mcan_ram_config_t *simple_config, bool enable_canfd)
{
    (void) memset(simple_config, 0, sizeof(mcan_ram_config_t));
    uint32_t start_addr = mcan_get_ram_offset(ptr);

    if (!enable_canfd) {
        simple_config->enable_std_filter = true;
        simple_config->std_filter_elem_count = MCAN_FILTER_SIZE_CAN_DEFAULT;
        start_addr += MCAN_FILTER_ELEM_STD_ID_SIZE * MCAN_FILTER_SIZE_CAN_DEFAULT;  //一个标准过滤器的大小 * 标准过滤器的个数

        simple_config->enable_ext_filter = true;
        simple_config->ext_filter_elem_count = MCAN_FILTER_SIZE_CAN_DEFAULT;
        start_addr += MCAN_FILTER_ELEM_EXT_ID_SIZE * MCAN_FILTER_SIZE_CAN_DEFAULT;

        simple_config->enable_txbuf = true;
        simple_config->txbuf_dedicated_txbuf_elem_count = MCAN_TXBUF_SIZE_CAN_DEFAULT / 2;
        simple_config->txbuf_fifo_or_queue_elem_count = MCAN_TXBUF_SIZE_CAN_DEFAULT / 2;
        simple_config->txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;
        simple_config->txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
        start_addr += MCAN_TXBUF_SIZE_CAN_DEFAULT * MCAN_TXRX_ELEM_SIZE_CAN_MAX;

        for (uint32_t i = 0; i < ARRAY_SIZE(simple_config->rxfifos); i++) {
            simple_config->rxfifos[i].enable = true;
            simple_config->rxfifos[i].elem_count = MCAN_RXFIFO_SIZE_CAN_DEFAULT;
            simple_config->rxfifos[i].operation_mode = MCAN_FIFO_OPERATION_MODE_BLOCKING;
            simple_config->rxfifos[i].watermark = 10U;
            simple_config->rxfifos[i].data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
            start_addr += MCAN_RXFIFO_SIZE_CAN_DEFAULT * MCAN_TXRX_ELEM_SIZE_CAN_MAX;
        }
        simple_config->enable_rxbuf = true;
        simple_config->rxbuf_elem_count = MCAN_RXBUF_SIZE_CAN_DEFAULT;
        simple_config->rxbuf_data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
        start_addr += MCAN_RXBUF_SIZE_CAN_DEFAULT * MCAN_TXRX_ELEM_SIZE_CAN_MAX;
    } else {
                            /****************************/
                            /*     标准过滤器   0-128    */
                            /*     扩展过滤器   0-64     */
                            /*      rxfifo0    0-64     */
                            /*      rxfifo1    0-64     */
                            /*      rxbuffs    0-64     */
                            /*     txeventfifo 0-32     */
                            /*     txbuffers   0-32     */
                            /****************************/ 
        simple_config->enable_std_filter = true;
        simple_config->std_filter_elem_count = MCAN_FILTER_SIZE_CANFD_DEFAULT;
        start_addr += MCAN_FILTER_ELEM_STD_ID_SIZE * MCAN_FILTER_SIZE_CANFD_DEFAULT;    
                                                                                                            
        simple_config->enable_ext_filter = true;                                                            
        simple_config->ext_filter_elem_count = MCAN_FILTER_SIZE_CANFD_DEFAULT;                              
        start_addr += MCAN_FILTER_ELEM_EXT_ID_SIZE * MCAN_FILTER_SIZE_CANFD_DEFAULT;     
                                                                                                            
        simple_config->enable_txbuf = true;                                                                 
        simple_config->txbuf_dedicated_txbuf_elem_count = 16 / 2;     
        simple_config->txbuf_fifo_or_queue_elem_count = 16 / 2;          
        simple_config->txfifo_or_txqueue_mode = MCAN_TXBUF_OPERATION_MODE_FIFO;                             
        simple_config->txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        start_addr += 16 * MCAN_TXRX_ELEM_SIZE_CANFD_MAX;

        for (uint32_t i = 0; i < ARRAY_SIZE(simple_config->rxfifos); i++) {
            simple_config->rxfifos[i].enable = true;
            simple_config->rxfifos[i].elem_count = MCAN_RXFIFO_SIZE_CANFD_DEFAULT;
            simple_config->rxfifos[i].operation_mode = MCAN_FIFO_OPERATION_MODE_BLOCKING;
            simple_config->rxfifos[i].watermark = 1U;
            simple_config->rxfifos[i].data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
            start_addr += MCAN_RXFIFO_SIZE_CANFD_DEFAULT * MCAN_TXRX_ELEM_SIZE_CANFD_MAX; 
        } 
        simple_config->enable_rxbuf = true;
        simple_config->rxbuf_elem_count = 16;   
        simple_config->rxbuf_data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        start_addr += 16 * MCAN_TXRX_ELEM_SIZE_CANFD_MAX; 
    }
    simple_config->enable_tx_evt_fifo = true;
    uint32_t tx_fifo_elem_count =
        simple_config->txbuf_dedicated_txbuf_elem_count + simple_config->txbuf_fifo_or_queue_elem_count;
    simple_config->tx_evt_fifo_elem_count = tx_fifo_elem_count;
    simple_config->tx_evt_fifo_watermark = 1U;

    start_addr += MCAN_TXEVT_ELEM_SIZE * tx_fifo_elem_count; 

    assert((start_addr - mcan_get_ram_offset(ptr)) <= mcan_get_ram_size(ptr));
    (void) start_addr; /* Suppress warnings in release build  */
}


#define MCAN_CAN_BAUDRATE_DEFAULT   (500UL * 1000UL)          /*!< Default CAN2.0 baudrate:500 kbps */
#define MCAN_CANFD_BAUDRATE_DEFAULT (2UL * 1000UL * 1000UL)   /*!< Default CANFD baudrate: 2 Mbps */
#define MCAN_CANSAMPONT             800UL
#define MCAN_CANFDSAMPONT           800UL
#define MCAN_FASTCANFDSAMPONT       750UL

void mcan_get_default_config1(MCAN_Type *ptr, mcan_config_t *config,uint16_t buad)
{
    (void) memset(config, 0, sizeof(mcan_config_t));
    config->baudrate = MCAN_CAN_BAUDRATE_DEFAULT;
    config->baudrate_fd = MCAN_CANFD_BAUDRATE_DEFAULT;
    config->can20_samplepoint_max = MCAN_CANSAMPONT;
    config->can20_samplepoint_min = MCAN_CANSAMPONT;
    if(4 == buad) 
    {
        config->canfd_samplepoint_min = MCAN_FASTCANFDSAMPONT;
        config->canfd_samplepoint_max = MCAN_FASTCANFDSAMPONT;
    }
    else 
    {
        config->canfd_samplepoint_min = MCAN_CANFDSAMPONT;
        config->canfd_samplepoint_max = MCAN_CANFDSAMPONT;
    }

    config->mode = mcan_mode_normal;
    config->enable_canfd = false;  

    /* Default Filter settings */
    mcan_all_filters_config_t *filters_config = &config->all_filters_config;
    filters_config->global_filter_config.reject_remote_ext_frame = false;
    filters_config->global_filter_config.reject_remote_std_frame = false;
    filters_config->global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
    filters_config->global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
    filters_config->ext_id_mask = 0x1FFFFFFFUL;
    filters_config->std_id_filter_list.filter_elem_list = 0;
    filters_config->std_id_filter_list.mcan_filter_elem_count = 1;
    filters_config->ext_id_filter_list.filter_elem_list = 0;
    filters_config->ext_id_filter_list.mcan_filter_elem_count = 1;

    /* Default MCAN RAM Configuration */
    mcan_ram_config_t *ram_config = &config->ram_config;
    mcan_get_default_ram_config1(ptr, ram_config, false);

    /* Default Internal Timestamp Configuration */
    mcan_internal_timestamp_config_t *ts_config = &config->timestamp_cfg;
    ts_config->counter_prescaler = 1U;
    ts_config->timestamp_selection = MCAN_TIMESTAMP_SEL_VALUE_ALWAYS_ZERO;

    /* Default TSU configuration */
    mcan_tsu_config_t *tsu_config = &config->tsu_config;
    tsu_config->prescaler = 1U;
#if defined(MCAN_SOC_TSU_SRC_TWO_STAGES) && (MCAN_SOC_TSU_SRC_TWO_STAGES == 1)
    tsu_config->ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_TBSEL_0;
    tsu_config->tbsel_option = MCAN_TSU_TBSEL_PTPC0;
#else
    tsu_config->ext_timebase_src = MCAN_TSU_EXT_TIMEBASE_SRC_PTPC;
#endif
    tsu_config->use_ext_timebase = false;
    tsu_config->capture_on_sof = false;
    tsu_config->enable_tsu = false;

    config->timeout_cfg.enable_timeout_counter = false;
    config->timeout_cfg.timeout_period = 0xFFFFU;
    config->timeout_cfg.timeout_sel = mcan_timeout_continuous_operation;
}


void CanInit(uint16_t Baud1, uint16_t Baud2, uint16_t Id,uint16_t Canmode)
{
    uint32_t ArbBaud, DataBaud;

    if(Baud1 == 0)
    {
        ArbBaud = 500000;
    }
    else if(Baud1 == 1)
    {
        ArbBaud = 1000000;
    }
    else
    {
        ArbBaud = 1000000;
    }

    if(Baud2 == 0)
    {
        DataBaud = 1000000;
    }
    else if(Baud2 == 1)
    {
        DataBaud = 2000000;
    }
    else if(Baud2 == 2)
    {
        DataBaud = 3000000;
    }
    else if(Baud2 == 3)
    {
        DataBaud = 4000000;
    }
    else if(Baud2 == 4)
    {
        DataBaud = 5000000;
    }

    else
    {
        DataBaud = 1000000;
    }
   
    SalveId = Id;
    CANFDSalveId = SalveId % 10;
    MqbTestMcan = CANSelect;
    /* Initialize CAN */
    Init_can_pins();
    can_ram_t ram_info = {0};
    ram_info.ram_size = sizeof(mcan_msg_buf);
    ram_info.ram_base = (uint32_t) &mcan_msg_buf;
    
    mcan_msg_buf_attr_t attr = {ram_info.ram_base, ram_info.ram_size};
    mcan_set_msg_buf_attr(CANSelect, &attr);//设置目标can的ram地址和大小

    mcan_config_t can_config;
    hpm_stat_t status;
    mcan_filter_elem_t can_filters[16];
    mcan_get_default_config1(CANSelect, &can_config,Baud2);
    can_config.baudrate = ArbBaud; /* 1000kbps */
    can_config.baudrate_fd = DataBaud;
    can_config.enable_tdc = true;
    can_config.mode = mcan_mode_normal;
    mcan_get_default_ram_config1(CANSelect, &can_config.ram_config, true);
    if(Canmode == 0)
    {
        can_config.enable_canfd = false;
    }
    else 
    {
        can_config.enable_canfd = true;
    }
    uint32_t can_src_clk_freq = board_init_can_clock(CANSelect);
    //Ctrl 
    uint32_t rxbuf_index = 0;     
    can_filters[0].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   //过滤器的类型
    can_filters[0].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; //过滤器配置,如果匹配放到哪里(取值0-7,7为放到rxbuffer)
    can_filters[0].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   //数据帧ID的种类(扩展帧/标准帧)
    can_filters[0].match_id = (uint32_t)(0x10);      
    can_filters[0].offset = rxbuf_index;   //存到rxbuffer的哪个位置
    can_filters[0].filter_event = 0;       //控制扩展接口上的滤波器事件引脚
    can_filters[0].store_location = 0;     //接收到的消息存放到缓存还是为调试信息(0为存到缓存)
    //Sdo 读写帧
    rxbuf_index = 1;    
    can_filters[1].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[1].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[1].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[1].match_id = (uint32_t)(0x100 + Id);
    can_filters[1].offset = rxbuf_index;  
    can_filters[1].filter_event = 0;       
    can_filters[1].store_location = 0;     
    //PVT CAN指令帧1
    rxbuf_index = 2;    
    can_filters[2].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[2].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[2].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[2].match_id = (uint32_t)(0x200 + Id); 
    can_filters[2].offset = rxbuf_index;  
    can_filters[2].filter_event = 0;       
    can_filters[2].store_location = 0; 
    //Pos 指令帧2
    rxbuf_index = 3;    
    can_filters[3].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[3].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[3].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[3].match_id = (uint32_t)(0x280 + Id);      
    can_filters[3].offset = rxbuf_index;  
    can_filters[3].filter_event = 0;       
    can_filters[3].store_location = 0; 
    //Vel 指令帧3
    rxbuf_index = 4;    
    can_filters[4].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[4].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[4].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[4].match_id = (uint32_t)(0x300 + Id);      
    can_filters[4].offset = rxbuf_index;  
    can_filters[4].filter_event = 0;       
    can_filters[4].store_location = 0; 
    //Tor 指令帧4
    rxbuf_index = 5;    
    can_filters[5].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[5].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[5].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[5].match_id = (uint32_t)(0x380 + Id);      
    can_filters[5].offset = rxbuf_index;  
    can_filters[5].filter_event = 0;       
    can_filters[5].store_location = 0; 
    //请求反馈帧 指令帧5
    rxbuf_index = 6;    
    can_filters[6].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[6].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[6].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[6].match_id = (uint32_t)(0x400 + Id);    
    can_filters[6].offset = rxbuf_index;  
    can_filters[6].filter_event = 0;       
    can_filters[6].store_location = 0; 
    //MIT
    rxbuf_index = 7;    
    can_filters[7].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[7].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[7].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[7].match_id = (uint32_t)(0x20);      
    can_filters[7].offset = rxbuf_index;  
    can_filters[7].filter_event = 0;       
    can_filters[7].store_location = 0;  
    //mqb PVT CANFD指令帧
    if(0 == RamPA->SyncMode)
    {
        rxbuf_index = 8;    
        can_filters[9].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
        can_filters[9].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
        can_filters[9].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
        can_filters[9].match_id = (uint32_t)(0x50); //mqb同步位置模式
        can_filters[9].offset = rxbuf_index;  
        can_filters[9].filter_event = 0;       
        can_filters[9].store_location = 0; 
    }
    else if (1 == RamPA->SyncMode) 
    {
        
        //rxbuf_index = 9;      //
        can_filters[9].filter_type = MCAN_FILTER_TYPE_SPECIFIED_ID_FILTER;   
        can_filters[9].filter_config = MCAN_FILTER_ELEM_CFG_SET_PRIORITY_AND_STORE_IN_FIFO0_IF_MATCH;    
        can_filters[9].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
        can_filters[9].id1 = (uint32_t)(0x50); 
        can_filters[9].id2 = (uint32_t)(0x50);    
    }
 
    //同步帧
    //rxbuf_index = 8;    
    can_filters[8].filter_type = MCAN_FILTER_TYPE_SPECIFIED_ID_FILTER;   
    can_filters[8].filter_config = MCAN_FILTER_ELEM_CFG_SET_PRIORITY_AND_STORE_IN_FIFO0_IF_MATCH; 
    can_filters[8].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[8].id1 = (uint32_t)(0x40); 
    can_filters[8].id2 = (uint32_t)(0x40); 
    
        //OTA指令
    rxbuf_index = 10;    
    can_filters[rxbuf_index].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[rxbuf_index].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[rxbuf_index].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[rxbuf_index].match_id = (uint32_t)(0x740 + Id);      
    can_filters[rxbuf_index].offset = rxbuf_index;  
    can_filters[rxbuf_index].filter_event = 0;       
    can_filters[rxbuf_index].store_location = 0; 

    //OTA数据
    rxbuf_index = 11;    
    can_filters[rxbuf_index].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[rxbuf_index].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[rxbuf_index].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[rxbuf_index].match_id = (uint32_t)(0x780 + Id);      
    can_filters[rxbuf_index].offset = rxbuf_index;  
    can_filters[rxbuf_index].filter_event = 0;       
    can_filters[rxbuf_index].store_location = 0; 

    //OTA触发数据
    rxbuf_index = 12;    
    can_filters[rxbuf_index].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;   
    can_filters[rxbuf_index].filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG; 
    can_filters[rxbuf_index].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;   
    can_filters[rxbuf_index].match_id = (uint32_t)(0x6C0 + Id);      
    can_filters[rxbuf_index].offset = rxbuf_index;  
    can_filters[rxbuf_index].filter_event = 0;       
    can_filters[rxbuf_index].store_location = 0; 
    
    can_config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0;
    can_config.all_filters_config.std_id_filter_list.filter_elem_list = can_filters;
    can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 13; 
    can_config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.reject_remote_std_frame = false;
    can_config.all_filters_config.global_filter_config.reject_remote_ext_frame = true;

    status = mcan_init(CANSelect, &can_config, can_src_clk_freq);
    if (status != status_success) {
        return;
    }
    mcan_enable_interrupts(CANSelect, MCAN_IE_HPME_MASK);
    intc_m_enable_irq_with_priority(CAN_IRQn, 10);

    txfifo_addr = 0xF0200000UL + (CANSelect->TXBC  & 0xfffc);
    rxbuff_addr = 0xF0200000UL + (CANSelect->RXBC  & 0xfffc);
    rxfifo_addr = 0xF0200000UL + (CANSelect->RXF0C & 0xfffc);

}

#endif
















