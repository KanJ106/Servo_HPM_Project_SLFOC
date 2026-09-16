#ifndef __COM_TIME__
#define __COM_TIME__

#include "board.h"
#include "hpm_mchtmr_drv.h"

uint64_t SetTimer(int time);
uint8_t TimerOut(uint64_t time);
uint8_t GetTimeStampString(char *str , uint8_t strsize);
void TimerInit(void);
void TimerUpdate(void);

void watch_start(uint8_t index);
uint16_t watch_stop(uint8_t index, uint8_t saveflg);
void watch_runtime_get_result(uint16_t *p_result, uint8_t array_flg);
uint32_t watch_timer_point(uint8_t index);
uint32_t watch_timer_once(uint8_t index);

extern uint64_t TimerCount;

#endif /* __COM_TIME__ */