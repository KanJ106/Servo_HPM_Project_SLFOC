#include "com_time.h"

#define DEBUG_WATCH __attribute__((section(".noncacheable")))
#define WATCH_CH 3
#define WATCH_NUM 500
uint64_t TimerCount = 0;
uint64_t Comtimer_freq = 0;


void TimerInit(void)
{
    clock_add_to_group(clock_mchtmr0, 0);
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_update_core_clock();

    Comtimer_freq = clock_get_frequency(clock_mchtmr0);
}

void TimerUpdate(void)
{
    uint64_t timetemp;

    timetemp = mchtmr_get_count(HPM_MCHTMR);

    //转换成ms
    TimerCount = (uint32_t)(timetemp * 1000 / Comtimer_freq);
}


uint64_t SetTimer(int time)
{
    // 传入超时时间（毫秒），返回未来时刻的绝对时间戳
    // 若 time <= 0，则返回当前时刻（表示立即超时）
    TimerUpdate();

    if (time <= 0)
    {
        return TimerCount;
    }
    return TimerCount + (uint64_t)time;
}

uint8_t TimerOut(uint64_t time)
{
    TimerUpdate();

    // 判断传入的时间戳是否已经超时
    // 当前时间 >= 目标时间时返回 1（超时），否则返回 0
    return (TimerCount >= time) ? 1 : 0;
}

//todo: ELOG的时间戳显示


/**
 * @brief ELOG的时间戳显示 格式 xxxxx:xxx
 *
 * @param char *str  :      时间戳字符数组
 * @param uint8_t strsize : 数组大小,12个以内
 * @return uint8_t :        0 表示成功，其他表示失败
 */
uint8_t GetTimeStampString(char *str , uint8_t strsize)
{
    char timeStampSecond[20] = "";
    char timeStampMillisecond[4] = "";

    if ( (strsize < 12) || (str == NULL) )
    {
        return 1;
    }

    TimerUpdate();

    // todo: 
    //获取当前毫秒值
    uint32_t millisecond = TimerCount % 1000;
    sprintf(timeStampMillisecond, "%03d", millisecond);

    //获取当前秒值
    uint32_t second = TimerCount / 1000;
    sprintf(timeStampSecond, "%d", second);

    //将当前时间转换成 xxxxx:xxx形式
    sprintf(str, "%s:%s", timeStampSecond, timeStampMillisecond);

    return 0;
}

uint64_t watch_timer_1[WATCH_CH]   = {0};
uint64_t watch_timer_2[WATCH_CH]   = {0};
uint32_t watch_result_us[WATCH_CH] = {0};
uint16_t watch_result_ms[WATCH_CH] = {0};

DEBUG_WATCH uint16_t runTimeArray1[WATCH_NUM];
DEBUG_WATCH uint16_t runTimeArray2[WATCH_NUM];
DEBUG_WATCH uint16_t runTimeArray3[WATCH_NUM];

DEBUG_WATCH uint32_t runTime1 = 0;
DEBUG_WATCH uint32_t runTime2 = 0;
DEBUG_WATCH uint32_t runTime3 = 0;

uint8_t watch_start_flg = 0;

uint16_t runTimeIndex[WATCH_CH] = {0};

void watch_start(uint8_t index)
{
    //记录开始时间
    watch_timer_1[index] = mchtmr_get_count(HPM_MCHTMR);   
}


uint16_t watch_stop(uint8_t index, uint8_t saveflg)
{
    if(index >= WATCH_CH)
    {
        return 0xffff;
    }

    //记录结束时间
    watch_timer_2[index] = mchtmr_get_count(HPM_MCHTMR); 

    //计算运行时间
    watch_result_us[index] = (uint16_t)((watch_timer_2[index] - watch_timer_1[index]) * 100000000 / Comtimer_freq);

    if(saveflg == 1)
    {
        if(runTimeIndex[index] > WATCH_NUM)
        {
            runTimeIndex[index] = 0;
        }

        if(index == 0)
        {
            runTime1 = watch_result_us[index];
            runTimeArray1[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 1)
        {
            runTime2 = watch_result_us[index];
            runTimeArray2[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 2)
        {
            runTime3 = watch_result_us[index];
            runTimeArray3[runTimeIndex[index]++] = watch_result_us[index];
        }
        
    }

    return watch_result_us[index];
}

uint32_t watch_timer_point(uint8_t index)
{
    uint32_t ret_time = 0;

    if(index >= WATCH_CH)
    {
        return 0xffff;
    }

    //启动
    if((watch_start_flg & (1 << index)) == 0)
    {
        //记录开始时间
        watch_timer_1[index] = mchtmr_get_count(HPM_MCHTMR);        
    }
    //停止
    else
    {
        //记录结束时间
        watch_timer_2[index] = mchtmr_get_count(HPM_MCHTMR); 

        //计算运行时间 10ns
        watch_result_us[index] = (watch_timer_2[index] - watch_timer_1[index]) * 100000000 / Comtimer_freq;

        if(runTimeIndex[index] > WATCH_NUM)
        {
            runTimeIndex[index] = 0;
        }

        if(index == 0)
        {
            runTime1 = watch_result_us[index];
            runTimeArray1[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 1)
        {
            runTime2 = watch_result_us[index];
            runTimeArray2[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 2)
        {
            runTime3 = watch_result_us[index];
            runTimeArray3[runTimeIndex[index]++] = watch_result_us[index];
        }

        ret_time = watch_result_us[index];
    }

    watch_start_flg ^= (1 << index);

    return ret_time;
     
}

uint32_t watch_timer_once(uint8_t index)
{
    uint32_t ret_time = 0;
    static uint64_t lastTimerCount = 0;

    if(index >= WATCH_CH)
    {
        return 0xffff;
    }

    if(lastTimerCount == 0)
    {
        lastTimerCount = mchtmr_get_count(HPM_MCHTMR);
        return 0;
    }

    {
        //记录结束时间
        watch_timer_2[index] = mchtmr_get_count(HPM_MCHTMR); 

        //计算运行时间 1us
        watch_result_us[index] = (watch_timer_2[index] - lastTimerCount) * 1000000 / Comtimer_freq;
        lastTimerCount = watch_timer_2[index];

        if(runTimeIndex[index] > WATCH_NUM)
        {
            runTimeIndex[index] = 0;
        }

        if(index == 0)
        {
            runTime1 = watch_result_us[index];

            if(watch_result_us[index] >= 4000)
            {
                runTimeArray1[runTimeIndex[index]++] = watch_result_us[index] - 4000;
            }
            else
            {
                runTimeArray1[runTimeIndex[index]++] = 4000 - watch_result_us[index];
            }
            
        }
        else if(index == 1)
        {
            runTime2 = watch_result_us[index];
            runTimeArray2[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 2)
        {
            runTime3 = watch_result_us[index];
            runTimeArray3[runTimeIndex[index]++] = watch_result_us[index];
        }

        ret_time = watch_result_us[index];
    }

    return ret_time;
     
}

uint32_t watch_timer_once2(uint8_t index)
{
    uint32_t ret_time = 0;
    static uint64_t lastTimerCount = 0;

    if(index >= WATCH_CH)
    {
        return 0xffff;
    }

    if(lastTimerCount == 0)
    {
        lastTimerCount = mchtmr_get_count(HPM_MCHTMR);
        return 0;
    }

    {
        //记录结束时间
        watch_timer_2[index] = mchtmr_get_count(HPM_MCHTMR); 

        //计算运行时间 1us
        watch_result_us[index] = (watch_timer_2[index] - lastTimerCount) * 1000000 / Comtimer_freq;
        lastTimerCount = watch_timer_2[index];

        if(runTimeIndex[index] > WATCH_NUM)
        {
            runTimeIndex[index] = 0;
        }

        if(index == 0)
        {
            runTime1 = watch_result_us[index];
            runTimeArray1[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 1)
        {
            runTime2 = watch_result_us[index];
            runTimeArray2[runTimeIndex[index]++] = watch_result_us[index];
        }
        else if(index == 2)
        {
            runTime3 = watch_result_us[index];
            runTimeArray3[runTimeIndex[index]++] = watch_result_us[index];
        }

        ret_time = watch_result_us[index];
    }

    return ret_time;
     
}




//void watch_runtime_result_to_array(void)
//{
//    if(runTimeIndex > 999)
//    {
//        runTimeIndex = 0;
//    }

//    runTimeArray[runTimeIndex++] = watch_result_us;
//}

//void watch_runtime_get_result(uint16_t *p_result, uint8_t array_flg)
//{
//    //计算运行时间
//    watch_result_us = (uint16_t)((watch_timer_2 - watch_timer_1) * 1000000 / Comtimer_freq);

//    if(p_result != NULL)
//    {
//        *p_result = watch_result_us;
//    }
    
//    if(array_flg)
//    {
//        if(runTimeIndex > 999)
//        {
//            runTimeIndex = 0;
//        }

//        runTimeArray[runTimeIndex++] = watch_result_us;
//    }
    
//}


