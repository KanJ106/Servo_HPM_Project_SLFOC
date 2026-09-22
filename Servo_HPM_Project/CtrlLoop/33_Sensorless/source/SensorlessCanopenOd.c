#include "SensorlessCanopen.h"
#include "SensorlessShadow.h"
#include "CO_ODinterface.h"
#include "canopen_interface.h"
#include "Drive.h"
#include "SV_FaultProtect.h"
#include <string.h>
extern uint8_t NMT_State;
extern CO_NMT_reset_cmd_t reset;
uint16_t SensorlessCanopen_PlatformReady(void)
{
    return (uint16_t)(reset==CO_RESET_NOT && NMT_State==CO_NMT_OPERATIONAL &&
        LocalAxes.i16State==STATE_OPERATION_ENABLED &&
        (LocalAxes.Objects.objControlWord & 0x008FU)==0x000FU &&
        !(LocalAxes.Objects.objControlWord & 0x0100U) &&
        StateMachine.RegulFlg==1U && StateMachine.RegilFlgISR==1U &&
        !(FaultP.FaultStatus & FP_ERR));
}
uint16_t SensorlessJlink_PlatformReady(void)
{
    return (uint16_t)(StateMachine.RegulFlg==1U && StateMachine.RegilFlgISR==1U &&
        StateMachine.Ready.all==0U && !(FaultP.FaultStatus & FP_ERR));
}
uint16_t SensorlessJlink_PlatformOff(void)
{
    return (uint16_t)(!StateMachine.RegulFlg && !StateMachine.RegilFlgISR &&
        !StateMachine.SvpwmOnFlg && !StateMachine.SrvOnStatus &&
        !LocalAxes.bAxisFunctionEnabled);
}
static ODR_t sl_read(OD_stream_t *s,void *buf,OD_size_t count,OD_size_t *done)
{
    uint32_t value; OD_size_t len;
    if(!s || !buf || !done)return ODR_DEV_INCOMPAT;
    *done=0;
    if(s->index==0x2F07) {
        if(s->dataOffset>sizeof(g_sensorless_trace))return ODR_DATA_LONG;
        len=(OD_size_t)sizeof(g_sensorless_trace)-s->dataOffset;
        if(count<len)len=count;
        if(!SensorlessCanopen_TraceRead(s->dataOffset,buf,len))return ODR_DATA_DEV_STATE;
        *done=len;s->dataOffset+=len;
        if(s->dataOffset<sizeof(g_sensorless_trace))return ODR_PARTIAL;
        s->dataOffset=0;return ODR_OK;
    }
    len=s->subIndex?4U:1U;
    if(count<len || s->dataOffset)return ODR_TYPE_MISMATCH;
    if(!SensorlessCanopen_Read(s->index,s->subIndex,&value))return ODR_SUB_NOT_EXIST;
    memcpy(buf,&value,len);*done=len;return ODR_OK;
}
static ODR_t sl_write(OD_stream_t *s,const void *buf,OD_size_t count,OD_size_t *done)
{
    uint32_t value;
    if(!s || !buf || !done)return ODR_DEV_INCOMPAT;
    *done=0;
    if(!s->subIndex || s->index>0x2F02)return ODR_READONLY;
    if(count!=4U || s->dataOffset)return ODR_TYPE_MISMATCH;
    memcpy(&value,buf,4);
    if(!SensorlessCanopen_Write(s->index,s->subIndex,value))return ODR_DATA_DEV_STATE;
    *done=4;return ODR_OK;
}
static OD_extension_t extension={.object=NULL,.read=sl_read,.write=sl_write};
static uint32_t placeholder;
static uint8_t count0=16;
static OD_obj_record_t rec0[17];
static uint8_t count1=13;
static OD_obj_record_t rec1[14];
static uint8_t count2=67;
static OD_obj_record_t rec2[68];
static uint8_t count3=13;
static OD_obj_record_t rec3[14];
static uint8_t count4=40;
static OD_obj_record_t rec4[41];
static uint8_t count5=19;
static OD_obj_record_t rec5[20];
static uint8_t count6=16;
static OD_obj_record_t rec6[17];
static OD_obj_var_t trace_var={.dataOrig=&placeholder,.attribute=ODA_SDO_R,
                              .dataLength=sizeof(g_sensorless_trace)};
OD_entry_t g_sl_can_od[SL_CAN_OD_COUNT];
void SensorlessCanopen_InitOd(void)
{
    OD_obj_record_t *records[]={rec0,rec1,rec2,rec3,rec4,rec5,rec6};
    uint8_t *counts[]={&count0,&count1,&count2,&count3,&count4,&count5,&count6};
    for(uint16_t i=0;i<7;i++) {
        records[i][0]=(OD_obj_record_t){.dataOrig=counts[i],.subIndex=0,
                                     .attribute=ODA_SDO_R,.dataLength=1};
        for(uint16_t j=1;j<=*counts[i];j++) {
            OD_attr_t attr=ODA_SDO_R|ODA_MB;
            if(i==1 || i==2 || (i==0 && (j<=6 || j==9)))attr|=ODA_SDO_W;
            if(i==6)attr|=ODA_TPDO;
            records[i][j]=(OD_obj_record_t){.dataOrig=&placeholder,.subIndex=(uint8_t)j,
                                          .attribute=attr,.dataLength=4};
        }
        g_sl_can_od[i]=(OD_entry_t){.index=(uint16_t)(SL_CAN_OD_FIRST+i),
            .subEntriesCount=(uint8_t)(*counts[i]+1U),.odObjectType=ODT_REC,
            .odObject=records[i],.extension=&extension};
    }
    g_sl_can_od[7]=(OD_entry_t){.index=0x2F07,.subEntriesCount=1,
        .odObjectType=ODT_VAR,.odObject=&trace_var,.extension=&extension};
}

