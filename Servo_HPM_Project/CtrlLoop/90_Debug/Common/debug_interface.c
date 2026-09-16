#include "debug_interface.h"

void debug_interface_init(void) 
{
    TimerInit();
    RTT_Init();
    RTT_Test();
    elog_func_init();
    test_elog();
}

//debug start------------------------------------------------------

//debug end--------------------------------------------------------

//Ê¾Àý
void test_example(void) 
{
    //-------------------------------------------------------------
    static int64_t  MyTestRunTimerCnt = 0;
    static uint64_t MyTestTimer_1 = 0;

    if(TimerOut(MyTestTimer_1))
    {
        MyTestTimer_1 = SetTimer(1);
        MyTestRunTimerCnt++;
    }
    //-------------------------------------------------------------

    //-------------------------------------------------------------
    int data = MyTestRunTimerCnt;
    ELOG_A("ASSERT", "Hello EasyLogger!%d", data);
    ELOG_E("ERROR",  "Hello EasyLogger!%d", data);
    ELOG_W("WARN",   "Hello EasyLogger!%d", data);
    ELOG_I("INFO",   "Hello EasyLogger!%d", data);
    ELOG_D("DEBUG",  "Hello EasyLogger!%d", data);
    ELOG_V("VERBOSE","Hello EasyLogger!%d", data);
    //-------------------------------------------------------------
}